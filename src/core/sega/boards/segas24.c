/*  Sega System 24. Ported from MAME src/mame/sega/segas24.cpp, segaic24.cpp (the tilemaps, sprites and mixer),
    315_5296.cpp, fd1094.cpp, src/devices/machine/upd4701.cpp and msm6253.cpp (BSD-3-Clause, copyright-holders:
    Olivier Galibert, David Haywood, Aaron Giles, AJR).
    Two 68000s at 10 MHz (the second one an FD1094 on the floppy games, its program in its work RAM), held and reset
    by the I/O chip's CNT1. The interrupt controller: a 12-bit timer (hsync or 8 MHz), the vblank (line 384), the
    "sprite" interrupt (line 0), the YM2151 and the FRC; each CPU's own enable mask. The floppy controller hands out
    the disk image (in RAM, written back) a track at a time; the ROM board games bank 256 KB windows instead.
    Video: four 64x64 tilemaps of 8x8 tiles from RAM (two pairs, a window mask per 8 pixels), tile-based zooming
    sprites as a linked list, a mixer giving the priorities; 496x384 at 57.5 Hz, 8192 colours with shadow copies.
    Timing follows MAME's scheduler: the CPUs run to the next timer, an input line change ends the slice of the CPU
    making it and the other one catches up to that time before it applies.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../sound/rbx_ym.h"
#include "../../video/decoband.h"
#include "../machine/fd1094.h"

extern void (*rbx_m68k_cmpild_hook)(int reg, unsigned int imm);
extern void (*rbx_m68k_rte_hook)(void);
extern void (*rbx_m68k_irqack_hook)(int level);

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// time in ticks of 280 MHz (the 10 MHz CPUs, 7 MHz while Gain Ground's hack slows the second one, the 8 MHz timer,
// the hsync and the YM2151's 4 MHz all whole)
#define TPS 280000000ull
#define T_CYC 28                    // a 10 MHz cycle
#define T_CYC_SLOW 40               // at 0.7
#define T_LINE 11480                // 656 pixels at 16 MHz
#define T_TIMER 35                  // 8 MHz
#define T_FRC 43008                 // (MASTER_CLOCK / 2) / 1536
#define LINES 424
#define T_FRAME ((uint64_t)T_LINE * LINES)
#define FRAME_MHZ 57524             // 16 MHz / (656 x 424)
#define YM_LINE 164                 // YM2151 clocks a line
#define NEVER UINT64_MAX
#define SW 496
#define SH 384
#define NPAL 8192
#define PAL_BLACK (2 * NPAL)

enum { IRQ_YM2151 = 1, IRQ_TIMER = 2, IRQ_VBLANK = 3, IRQ_SPRITE = 4, IRQ_FRC = 5 };
enum { TM_SCAN, TM_CLEAR, TM_IRQ, TM_FRC, TM_SYNC, TM_HACK, NTIMERS };

// ---- configuration ----
static uint8_t *s_bios, *s_romboard, *s_floppy, *s_key;
static uint32_t s_bios_size, s_romboard_size, s_floppy_size;
static int s_track_size, s_fd1094_on, s_gground;
static const uint8_t *s_mlatch_table;
enum { IO_GENERIC, IO_DCCLUB, IO_MAHMAJN };
enum { EXT_NONE, EXT_UPD, EXT_HOTROD };
static int s_io_kind, s_ext;
static int s_port_p1, s_port_p2, s_port_p3, s_port_service, s_port_coinage, s_port_dsw, s_port_paddle;
static int s_port_mj[8], s_port_pedal[4];
static fd1094_t s_fd1094;

static const uint8_t s_mahmajn_mlt[8]  = { 5, 1, 6, 2, 3, 7, 4, 0 };
static const uint8_t s_mahmajn2_mlt[8] = { 6, 0, 5, 3, 1, 4, 2, 7 };
static const uint8_t s_qgh_mlt[8]      = { 3, 7, 4, 0, 2, 6, 5, 1 };
static const uint8_t s_bnzabros_mlt[8] = { 2, 4, 0, 5, 7, 3, 1, 6 };
static const uint8_t s_qrouka_mlt[8]   = { 1, 6, 4, 7, 0, 5, 3, 2 };
static const uint8_t s_quizmeku_mlt[8] = { 0, 3, 2, 4, 6, 1, 7, 5 };
static const uint8_t s_dcclub_mlt[8]   = { 4, 7, 3, 0, 2, 6, 5, 1 };

// ---- the state (saved; on the heap while the game runs, ADR-040) ----
static struct s24_state {
	uint16_t mainram[0x20000];     // share1: 0x080000 / 0xf80000
	uint16_t subram[0x20000];      // subcpu: CPU 2's 0x000000, 0xf00000
	uint16_t tileram[0x8000];
	uint16_t charram[0x10000];     // (the 128 KB the bus reaches: tile codes are 12 bits)
	uint16_t spriteram[0x20000];
	uint16_t paletteram[0x2000];
	uint16_t mixer[16];
	// the timers (MAME's emu_timers): expire time and insertion order
	uint64_t texp[NTIMERS];
	uint32_t tseq[NTIMERS], seq;
	uint64_t base, lt[2];          // the scheduler's base time, each CPU's local time
	uint32_t per[2];               // ticks a cycle
	uint64_t cyc_total[2], last_vpa[2];   // each CPU's cycles (the E clock), its last autovector acknowledge
	uint32_t want[2];              // the interrupt lines (bit n = level n) as set, applied at the next sync
	uint8_t halted, halt_want, reset_want;
	uint16_t scan_line;
	uint8_t vbl_on;                // (the scanline timer started: from the first line 0)
	// 315-5296
	uint8_t io_dir, io_cnt, io_latch[8];
	uint8_t cnt1, ym_reset_state, dac;
	// the FDC
	uint8_t fdc_track_side, fdc_mode;
	int32_t fdc_status, fdc_track, fdc_sector, fdc_data, fdc_phys_track, fdc_span, fdc_index_count;
	uint8_t fdc_irq, fdc_drq;
	uint32_t fdc_pos;              // (m_fdc_pt as an offset into the image)
	// misc
	uint8_t curbank, mlatch, cur_input_line, frc_mode;
	uint64_t frc_start;
	// the interrupt controller
	uint16_t irq_tdata, irq_tval;
	uint8_t irq_tmode, irq_allow0, irq_allow1, irq_timer_pend0, irq_timer_pend1;
	uint8_t irq_yms, irq_vblank, irq_sprite;
	uint64_t irq_synctime, irq_vsynctime;
	// uPD4701s (x / y counters) and the MSM6253s
	uint16_t upd_x[2], upd_y[2];
	uint8_t adc_shift[2];
	uint8_t fd_st, fd_irqmode;
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)

static uint8_t s_ctx[2][2048];
static int s_cur;
static uint32_t s_irqp[2];
static m68k_bus_t s_bus1;
static int s_exec;                 // a CPU is running (in m68k_execute)
static uint64_t s_target;          // its slice's end
static uint64_t s_cbtime;          // the time outside the CPUs (a timer's expire time)
static int s_frame_done;

static int32_t *s_mix_l, *s_mix_r;
static int s_mix_n, s_mix_cap;

static m68k_bus_t *bus_of(int n) { return n ? &s_bus1 : &m68k_bus; }

// ---- the two 68000s on one Musashi core ----
static void fd_update(void);
static void cpu_switch(int n)
{
	if (n == s_cur)
		return;
	m68k_get_context(s_ctx[s_cur]);
	s_irqp[s_cur] = m68k_bus_irq_pending();
	m68k_set_context(s_ctx[n]);
	m68k_bus_select(n ? &s_bus1 : NULL);
	s_cur = n;   // (before the lines: Musashi may take an interrupt there, the FD1094's hook asks which CPU)
	m68k_bus_set_irq_pending(s_irqp[n]);
	m68k_bus_update_irq();
}

static uint64_t now_t(void)
{
	if (s_exec)
		return b.lt[s_cur] + (uint64_t)m68k_cycles_run() * b.per[s_cur];
	return s_cbtime;
}

// ---- timers ----
static void t_adjust(int i, uint64_t when)
{
	b.texp[i] = when;
	b.tseq[i] = ++b.seq;
	if (s_exec && when < s_target)   // (the new head timer: the running CPU's slice ends)
		m68k_end_timeslice();
}

// ---- input lines (set_input_line: synced, applied once the other CPU has caught up) ----
static void apply_lines(void)
{
	for (int c = 0; c < 2; c++) {
		m68k_bus_t *bs = bus_of(c);
		const uint32_t keep = bs->irq_asserted & ~0xfeu;
		bs->irq_asserted = keep | b.want[c];
		if (c == s_cur)
			m68k_bus_update_irq();
	}
}
static void apply_sync(void);
static void sync_request(void)
{
	if (!s_exec) {   // (from a timer: at once)
		apply_sync();
		return;
	}
	const uint64_t t = now_t();
	if (b.texp[TM_SYNC] == NEVER || t < b.texp[TM_SYNC]) {
		b.texp[TM_SYNC] = t;
		b.tseq[TM_SYNC] = ++b.seq;
	}
	m68k_end_timeslice();
}
static void set_line(int cpu, int irq, int state)   // irq: IRQ_*, the 68000 level irq + 1
{
	const uint32_t bit = 1u << (irq + 1);
	b.want[cpu] = state ? (b.want[cpu] | bit) : (b.want[cpu] & ~bit);
	sync_request();
}

static void sub_reset(void)
{
	const int prev = s_cur;
	cpu_switch(1);
	if (s_fd1094_on) {   // (the FD1094's reset: the cache flushed, decrypted again from the RAM)
		fd1094_exit(&s_fd1094);
		fd1094_change_state(&s_fd1094, FD1094_STATE_RESET);
		fd_update();
	}
	m68k_pulse_reset();
	m68k_bus_update_irq();
	cpu_switch(prev);
}
static void apply_sync(void)
{
	apply_lines();
	if (b.reset_want) {
		b.reset_want = 0;
		b.halted = 0;
		sub_reset();
	}
	b.halted = b.halt_want;
}
static void tm_sync(void)
{
	b.texp[TM_SYNC] = NEVER;
	apply_sync();
}

// ---- the FD1094 (the second CPU) ----
static void fd_update(void)
{
	if (!s_fd1094_on)
		return;
	const uint16_t *op = fd1094_opcodes(&s_fd1094, fd1094_state(&s_fd1094));
	s_bus1.op = (const uint8_t *)op;
	s_bus1.op_size = op ? 0x40000 : 0;
}
static void fd_cmpild(int reg, unsigned int imm)
{
	if (s_cur == 1 && reg == 0 && (imm & 0xffff) == 0xffff) {
		fd1094_change_state(&s_fd1094, (int)(imm >> 16));
		fd_update();
	}
}
static void fd_rte(void)
{
	if (s_cur != 1)
		return;
	fd1094_change_state(&s_fd1094, FD1094_STATE_RTE);
	fd_update();
}
static void irqack(int level)
{
	(void)level;
	// MAME's autovector acknowledge: VPA, synchronized with the E clock (m68000_device::vpa_sync) at the vector read,
	// 10 clocks into the interrupt's processing
	const uint64_t t = b.cyc_total[s_cur] + (s_exec ? (uint64_t)m68k_cycles_run() : 0) + 10;
	uint64_t at = b.last_vpa[s_cur];
	if (at < t) {
		const uint64_t mod = t % 10;
		at = t - mod + (mod < 7 ? 10 : 20);
		b.last_vpa[s_cur] = at;
	}
	m68k_add_int_cycles((int)(at - t) + 1);
	if (s_cur != 1 || !s_fd1094_on)
		return;
	fd1094_change_state(&s_fd1094, FD1094_STATE_IRQ);
	fd_update();
}

// ---- the interrupt controller ----
static void irq_timer_sync(void)
{
	const uint64_t ctime = now_t();
	switch (b.irq_tmode) {
	case 1: {   // (MAME's doubles: the floor of the hsyncs since the vsync)
		const double hs = 16000000.0 / 656.0;
		const int ppos = (int)floor((double)(int64_t)(b.irq_synctime - b.irq_vsynctime) / (double)TPS * hs);
		const int cpos = (int)floor((double)(int64_t)(ctime - b.irq_vsynctime) / (double)TPS * hs);
		b.irq_tval = (uint16_t)(b.irq_tval + cpos - ppos);
		break;
	}
	case 3: {
		const int ppos = (int)floor((double)(int64_t)(b.irq_synctime - b.irq_vsynctime) / (double)TPS * 8000000.0);
		const int cpos = (int)floor((double)(int64_t)(ctime - b.irq_vsynctime) / (double)TPS * 8000000.0);
		b.irq_tval = (uint16_t)(b.irq_tval + cpos - ppos);
		break;
	}
	}
	b.irq_synctime = ctime;
}
static void irq_timer_start(int old_tmode)
{
	switch (b.irq_tmode) {
	case 0:
		if (old_tmode) {
			b.irq_tval++;
			if (b.irq_tval == 0x1000)
				t_adjust(TM_IRQ, now_t());
			else
				b.texp[TM_IRQ] = NEVER;
		}
		break;
	case 1:
		t_adjust(TM_IRQ, now_t() + (uint64_t)T_LINE * (uint32_t)(0x1000 - b.irq_tval));
		break;
	case 2:   // (MAME: fatalerror)
		break;
	case 3:
		t_adjust(TM_IRQ, now_t() + (uint64_t)T_TIMER * (uint32_t)(0x1000 - b.irq_tval));
		break;
	}
}
static void tm_irq(void)
{
	b.texp[TM_IRQ] = NEVER;
	irq_timer_sync();
	b.irq_tval = b.irq_tdata;
	irq_timer_start(b.irq_tmode);
	b.irq_timer_pend0 = b.irq_timer_pend1 = 1;
	if (b.irq_allow0 & (1 << IRQ_TIMER))
		set_line(0, IRQ_TIMER, 1);
	if (b.irq_allow1 & (1 << IRQ_TIMER))
		set_line(1, IRQ_TIMER, 1);
}
static void tm_clear(void)
{
	b.texp[TM_CLEAR] = NEVER;
	b.irq_sprite = b.irq_vblank = 0;
	set_line(0, IRQ_VBLANK, 0);
	set_line(0, IRQ_SPRITE, 0);
	set_line(1, IRQ_VBLANK, 0);
	set_line(1, IRQ_SPRITE, 0);
}
static void tm_frc(void)
{
	b.texp[TM_FRC] += T_FRC;
	b.tseq[TM_FRC] = ++b.seq;
	if ((b.irq_allow0 & (1 << IRQ_FRC)) && b.frc_mode == 1)
		set_line(0, IRQ_FRC, 1);
	if ((b.irq_allow1 & (1 << IRQ_FRC)) && b.frc_mode == 1)
		set_line(1, IRQ_FRC, 1);
}
static void irq_w(int offset, uint16_t data, uint16_t mask)
{
	switch (offset) {
	case 0:
		irq_timer_sync();
		b.irq_tdata = (uint16_t)((b.irq_tdata & ~mask) | (data & mask));
		b.irq_tdata &= 0xfff;
		irq_timer_start(b.irq_tmode);
		break;
	case 1:
		if (mask & 0xff) {
			const uint8_t old = b.irq_tmode;
			irq_timer_sync();
			b.irq_tmode = data & 3;
			irq_timer_start(old);
		}
		break;
	case 2:
	case 3: {
		const int c = offset - 2;
		const uint8_t allow = data & 0x3f;
		if (c == 0) {
			b.irq_allow0 = allow;
			b.irq_timer_pend0 = 0;
		} else {
			b.irq_allow1 = allow;
			b.irq_timer_pend1 = 0;
		}
		set_line(c, IRQ_TIMER, 0);
		set_line(c, IRQ_YM2151, b.irq_yms && (allow & (1 << IRQ_YM2151)));
		set_line(c, IRQ_VBLANK, b.irq_vblank && (allow & (1 << IRQ_VBLANK)));
		set_line(c, IRQ_SPRITE, b.irq_sprite && (allow & (1 << IRQ_SPRITE)));
		break;
	}
	}
}
static uint16_t irq_r(int offset)
{
	switch (offset) {
	case 2:
		b.irq_timer_pend0 = 0;
		set_line(0, IRQ_TIMER, 0);
		break;
	case 3:
		b.irq_timer_pend1 = 0;
		set_line(1, IRQ_TIMER, 0);
		break;
	}
	irq_timer_sync();
	return b.irq_tval & 0xfff;
}
static void irq_ym(int state)
{
	b.irq_yms = (uint8_t)(state != 0);
	set_line(0, IRQ_YM2151, b.irq_yms && (b.irq_allow0 & (1 << IRQ_YM2151)));
	set_line(1, IRQ_YM2151, b.irq_yms && (b.irq_allow1 & (1 << IRQ_YM2151)));
}

// ---- the FDC ----
static uint16_t fdc_r(int offset)
{
	if (!s_track_size)
		return 0xffff;
	switch (offset) {
	case 0:
		b.fdc_irq = 0;
		return (uint16_t)b.fdc_status;
	case 1:
		return (uint16_t)b.fdc_track;
	case 2:
		return (uint16_t)b.fdc_sector;
	default: {
		const int res = b.fdc_data;
		if (b.fdc_drq) {
			b.fdc_span--;
			if (b.fdc_span) {
				b.fdc_pos++;
				b.fdc_data = b.fdc_pos < s_floppy_size ? s_floppy[b.fdc_pos] : 0;
			} else {
				b.fdc_drq = 0;
				b.fdc_status = 0;
				b.fdc_irq = 1;
			}
		}
		return (uint16_t)res;
	}
	}
}
static void fdc_w(int offset, uint16_t data, uint16_t mask)
{
	if (!s_track_size || !(mask & 0xff))
		return;
	data &= 0xff;
	switch (offset) {
	case 0:
		b.fdc_irq = 0;
		switch (data >> 4) {
		case 0x0:
			b.fdc_phys_track = b.fdc_track = 0;
			b.fdc_irq = 1;
			b.fdc_status = 4;
			break;
		case 0x1:
			b.fdc_phys_track = b.fdc_track = b.fdc_data;
			b.fdc_irq = 1;
			b.fdc_status = b.fdc_track ? 0 : 4;
			break;
		case 0x9:
		case 0xb:
			b.fdc_track_side = (data & 8) ? 1 : 0;
			b.fdc_mode = (uint8_t)(data >> 4);
			b.fdc_pos = (uint32_t)(s_track_size * (2 * b.fdc_phys_track + b.fdc_track_side));
			b.fdc_span = s_track_size;
			b.fdc_status = 3;
			b.fdc_drq = 1;
			if ((data >> 4) == 0x9)
				b.fdc_data = b.fdc_pos < s_floppy_size ? s_floppy[b.fdc_pos] : 0;
			break;
		case 0xd:
			b.fdc_mode = 0xd;
			b.fdc_span = 0;
			b.fdc_drq = 0;
			b.fdc_irq = data & 1;
			b.fdc_status = 0;
			break;
		}
		break;
	case 1:
		b.fdc_track = data;
		break;
	case 2:
		b.fdc_sector = data;
		break;
	case 3:
		if (b.fdc_drq) {
			if (b.fdc_pos < s_floppy_size)
				s_floppy[b.fdc_pos] = (uint8_t)data;
			b.fdc_pos++;
			b.fdc_span--;
			if (!b.fdc_span) {
				b.fdc_drq = 0;
				b.fdc_status = 0;
				b.fdc_irq = 1;
			}
		}
		b.fdc_data = data;
		break;
	}
}
static uint16_t fdc_status_r(void)
{
	if (!s_track_size)
		return 0xffff;
	return (uint16_t)(0x90 | (b.fdc_irq ? 2 : 0) | (b.fdc_drq ? 1 : 0) | (b.fdc_phys_track ? 0x40 : 0) |
	                  (b.fdc_index_count ? 0x20 : 0));
}

// ---- the 315-5296 I/O chip and its ports ----
static uint8_t port8(int p, uint8_t def) { return p >= 0 ? (uint8_t)rbx_port_read(p) : def; }
static uint8_t dcclub_p1_r(void)
{
	static const uint8_t pos[16] = { 0, 1, 3, 2, 6, 4, 12, 8, 9, 0, 0, 0 };
	return (uint8_t)((port8(s_port_p1, 0xff) & 0xf) | ((~pos[port8(s_port_paddle, 0) >> 4] << 4) & 0xf0));
}
static uint8_t dcclub_p3_r(void)
{
	static const uint8_t pos[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 3, 2 };
	return (uint8_t)((~pos[port8(s_port_paddle, 0) >> 4] & 0x03) | 0xfc);
}
static void cnt1_w(int state)
{
	if ((state != 0) != (b.cnt1 != 0)) {
		if (state) {   // (the halt released and a reset pulse, synced)
			b.halt_want = 0;
			b.reset_want = 1;
			sync_request();
			if (s_gground) {   // (Gain Ground's hack: the second CPU slowed for 2 seconds so a check passes)
				b.per[1] = T_CYC_SLOW;
				t_adjust(TM_HACK, now_t() + 2 * TPS);
			}
		} else {
			b.halt_want = 1;
			sync_request();
		}
	}
	b.cnt1 = (uint8_t)(state != 0);
}
static void ym_reset_w(int state)
{
	if (state != b.ym_reset_state) {
		b.ym_reset_state = (uint8_t)state;
		if (state)
			rbx_ym_reset(0);
	}
}
static void io_out_port(int port, uint8_t data)
{
	if (port == 3) {   // port D: the lamps, Mahjong's input mux
		if (s_io_kind == IO_MAHMAJN && (data & 4))
			b.cur_input_line = (b.cur_input_line + 1) & 7;
	} else if (port == 7) {   // port H: the DAC
		b.dac = data;
	}
}
static uint8_t io_in_port(int port)
{
	switch (port) {
	case 0:
		if (s_io_kind == IO_DCCLUB) return dcclub_p1_r();
		if (s_io_kind == IO_MAHMAJN) return (uint8_t)~(1 << b.cur_input_line);
		return port8(s_port_p1, 0xff);
	case 1: return port8(s_port_p2, 0xff);
	case 2:
		if (s_io_kind == IO_DCCLUB) return dcclub_p3_r();
		if (s_io_kind == IO_MAHMAJN) return port8(s_port_mj[b.cur_input_line], 0xff);
		return port8(s_port_p3, 0xff);
	case 4: return port8(s_port_service, 0xff);
	case 5: return port8(s_port_coinage, 0xff);
	case 6: return port8(s_port_dsw, 0xff);
	}
	return 0xff;   // (D and H: outputs)
}
static uint8_t io_r(int offset)
{
	offset &= 0x3f;
	if (offset < 8)
		return ((b.io_dir >> offset) & 1) ? b.io_latch[offset] : io_in_port(offset);
	switch (offset) {
	case 0x8: return 'S';
	case 0x9: return 'E';
	case 0xa: return 'G';
	case 0xb: return 'A';
	case 0xc: case 0xe: return b.io_cnt;
	case 0xd: case 0xf: return b.io_dir;
	}
	return 0xff;
}
static void io_w(int offset, uint8_t data)
{
	offset &= 0x3f;
	if (offset < 8) {
		if ((b.io_dir >> offset) & 1)
			io_out_port(offset, data);
		b.io_latch[offset] = data;
	} else if (offset == 0xe) {   // CNT0 (unused), CNT1 the second CPU, CNT2 the YM2151's reset
		cnt1_w((data >> 1) & 1);
		ym_reset_w((data >> 2) & 1);
		b.io_cnt = data;
	} else if (offset == 0xf) {
		for (int i = 0; i < 8; i++)
			if (((b.io_dir ^ data) >> i) & 1)
				io_out_port(i, ((data >> i) & 1) ? b.io_latch[i] : 0);
		b.io_dir = data;
	}
}

// ---- misc chips ----
static void reset_bank(void);
static void frc_mode_w(uint8_t data)
{
	b.frc_start = now_t();
	b.frc_mode = data & 1;
}
static uint8_t frc_r(void)
{
	// (MAME: elapsed() * (u32)clock: 416666 / 6510 a second)
	const uint64_t el = now_t() - b.frc_start;
	const uint64_t sec = el / TPS, rem = el % TPS;
	const uint32_t hz = b.frc_mode ? 6510 : 416666;
	const int32_t result = (int32_t)(sec * hz + rem * hz / TPS);
	return (uint8_t)(result % (b.frc_mode ? 0x67 : 0x100));
}
static void frc_w(void)   // (Bonanza Bros acknowledges with it)
{
	set_line(0, IRQ_FRC, 0);
	set_line(1, IRQ_FRC, 0);
}
static void mlatch_w(uint8_t data)
{
	if (!s_mlatch_table)
		return;
	if (data != 0xff) {
		uint8_t mxor = 0;
		for (int i = 0; i < 8; i++)
			if (b.mlatch & (1 << i))
				mxor |= (uint8_t)(1 << s_mlatch_table[i]);
		b.mlatch = data ^ mxor;
	} else
		b.mlatch = 0;
}
static uint8_t misc_r(int o)   // 0xbc0000 / 0xcc0000: o = address & 7 (odd bytes)
{
	switch (o) {
	case 1: return s_romboard ? b.curbank : 0;
	case 3: return b.frc_mode & 1;
	case 5: return frc_r();
	case 7: return b.mlatch;
	}
	return 0;
}
static void misc_w(int o, uint8_t data)
{
	switch (o) {
	case 1:
		if (s_romboard) {
			b.curbank = data;
			reset_bank();
		}
		break;
	case 3: frc_mode_w(data); break;
	case 5: frc_w(); break;
	case 7: mlatch_w(data); break;
	}
}
static uint8_t upd_read(int n, int offset)   // read_xy: the counter latched (no switches)
{
	const uint16_t v = (uint16_t)((offset & 2) ? b.upd_y[n] : b.upd_x[n]) & 0xfff;
	return (uint8_t)((offset & 1) ? v >> 8 : v);
}
static uint8_t adc_input(int n, int ch)
{
	if (n == 1)
		return 0xff;   // (the second MSM6253: no inputs)
	return port8(s_port_pedal[ch], 0xff);
}

// ---- buses ----
static uint16_t common_r16(uint32_t a, uint16_t mask)
{
	a &= 0xffffff;
	switch (a >> 21) {
	case 0x200000 >> 21:   // 0x200000-0x3fffff: tile / char RAM pages; the scroll registers' window
		return 0;
	case 0x400000 >> 21:   // the mixer (the palette is on pages)
		return b.mixer[(a >> 1) & 0xf];
	case 0x800000 >> 21:
		if ((a & 0x1c0) == 0x040) return 0xffff;   // (the I/O daughterboard)
		if (!(mask & 0xff)) return 0;   // (the 8-bit devices on the low byte)
		if ((a & 0x1c0) == 0x000) return io_r((int)((a >> 1) & 0x1f));
		if ((a & 0x1fc) == 0x100) return rbx_ym_read(0, (a >> 1) & 1);
		return 0;
	case 0xa00000 >> 21:
		if (a < 0xb00000) return irq_r((int)((a >> 1) & 3));
		if (a < 0xb80000) return (a & 8) ? fdc_status_r() : fdc_r((int)((a >> 1) & 3));
		if (a >= 0xbc0000) return (mask & 0xff) ? misc_r((int)(a & 7) | 1) : 0;
		return 0;
	case 0xc00000 >> 21:
		if (a >= 0xcc0000 && a < 0xd00000) return (mask & 0xff) ? misc_r((int)(a & 7) | 1) : 0;
		if (a < 0xc80000) {
			if (s_ext == EXT_UPD && !(a & 0x18)) return upd_read(0, (int)((a >> 1) & 3));
			if (s_ext == EXT_HOTROD) {
				if (!(a & 0x10)) return upd_read((a >> 3) & 1, (int)((a >> 1) & 3));
				if (mask & 0xff) {
					const int n = (a >> 1) & 1;   // d7_r: the shift register's MSB
					const uint8_t bit = b.adc_shift[n] >> 7;
					b.adc_shift[n] = (uint8_t)(b.adc_shift[n] << 1);
					return (uint16_t)(bit << 7);
				}
			}
		}
		return 0;
	}
	return 0;
}
static void common_w16(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	switch (a >> 21) {
	case 0x400000 >> 21: {
		uint16_t *r = &b.mixer[(a >> 1) & 0xf];
		*r = (uint16_t)((*r & ~mask) | (v & mask));
		break;
	}
	case 0x800000 >> 21:
		if (!(mask & 0xff))
			break;
		if ((a & 0x1c0) == 0x000) io_w((int)((a >> 1) & 0x1f), (uint8_t)v);
		else if ((a & 0x1fc) == 0x100) rbx_ym_write(0, (a >> 1) & 1, (uint8_t)v);
		break;
	case 0xa00000 >> 21:
		if (a < 0xb00000) irq_w((int)((a >> 1) & 3), v, mask);
		else if (a < 0xb80000) {
			if (!(a & 8)) fdc_w((int)((a >> 1) & 3), v, mask);
		} else if (a >= 0xbc0000 && (mask & 0xff))
			misc_w((int)(a & 7) | 1, (uint8_t)v);
		break;
	case 0xc00000 >> 21:
		if (a >= 0xcc0000 && a < 0xd00000) {
			if (mask & 0xff)
				misc_w((int)(a & 7) | 1, (uint8_t)v);
		} else if (a < 0xc80000 && s_ext == EXT_HOTROD && (a & 0x10) && (mask & 0xff)) {
			const int n = (a >> 1) & 1;   // select_w
			b.adc_shift[n] = adc_input(n, v & 3);
		}
		break;
	}
}
static uint16_t c_r16(uint32_t a) { return common_r16(a, 0xffff); }
static uint8_t c_r8(uint32_t a)
{
	const uint16_t v = common_r16(a & ~1u, a & 1 ? 0x00ff : 0xff00);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static void c_w16(uint32_t a, uint16_t v) { common_w16(a, v, 0xffff); }
static void c_w8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	common_w16(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

static void reset_bank(void)
{
	if (!s_romboard)
		return;
	const int prev = s_cur;
	uint8_t *p = s_romboard + (uint32_t)(b.curbank & 15) * 0x40000;
	for (int c = 0; c < 2; c++) {
		m68k_bus_select(c ? &s_bus1 : NULL);
		m68k_bus_map(0xb80000, 0xbbffff, p, 0x40000, 1, 0);
		m68k_bus_map(0xc80000, 0xcbffff, p, 0x40000, 1, 0);
	}
	m68k_bus_select(prev ? &s_bus1 : NULL);
}

static void map_bus(int c)
{
	m68k_bus_select(c ? &s_bus1 : NULL);
	m68k_bus_reset_map();
	if (c == 0)
		m68k_bus_map(0x000000, 0x07ffff, s_bios, 0x40000, 1, 0);
	else
		m68k_bus_map(0x000000, 0x07ffff, b.subram, sizeof(b.subram), 1, 1);
	m68k_bus_map(0x080000, 0x0fffff, b.mainram, sizeof(b.mainram), 1, 1);
	m68k_bus_map(0x100000, 0x1fffff, s_bios, 0x40000, 1, 0);
	for (uint32_t m = 0x200000; m < 0x400000; m += 0x100000) {
		m68k_bus_map(m + 0x00000, m + 0x1ffff, b.tileram, sizeof(b.tileram), 1, 1);
		m68k_bus_map(m + 0x80000, m + 0xfffff, b.charram, sizeof(b.charram), 1, 1);
	}
	for (uint32_t m = 0x400000; m < 0x600000; m += 0x8000)
		m68k_bus_map(m, m + 0x3fff, b.paletteram, sizeof(b.paletteram), 1, 1);
	for (uint32_t m = 0x600000; m < 0x800000; m += 0x80000)
		m68k_bus_map(m, m + 0x3ffff, b.spriteram, sizeof(b.spriteram), 1, 1);
	m68k_bus_map(0xf00000, 0xf7ffff, b.subram, sizeof(b.subram), 1, 1);
	m68k_bus_map(0xf80000, 0xffffff, b.mainram, sizeof(b.mainram), 1, 1);
	m68k_bus_cur->read8 = c_r8;
	m68k_bus_cur->read16 = c_r16;
	m68k_bus_cur->write8 = c_w8;
	m68k_bus_cur->write16 = c_w16;
	m68k_bus_irq_reset();   // (not the last game's lines: taken here as an interrupt otherwise)
	m68k_bus_cur->irq_asserted = 0;
	m68k_bus_cur->nmi_taken = 0;
	m68k_bus_set_line(1, 0);   // (the bus's line hook)
}

// ---- video ----
static struct {
	uint16_t tileram[0x8000];
	uint16_t charram[0x10000];
	uint16_t spriteram[0x20000];
	uint16_t paletteram[0x2000];
	uint16_t mixer[16];
	uint16_t sprd[0x2000], clip[0x2000];   // the sprite list (offsets of the entries, 0xffff: no clip)
	int nspr;
	int order[12];
	int pal_dirty;
} *s_snap;
static uint16_t *s_pal_conv;
static int s_pal_valid;
static uint8_t *s_pri;   // the priority bitmap (SW x SH)

static void pal_entry(int i, uint16_t data)   // paletteram_w
{
	int r = (data & 0x00f) << 4, g = data & 0x0f0, bb = (data & 0xf00) >> 4;
	if (data & 0x1000) r |= 8;
	if (data & 0x2000) g |= 8;
	if (data & 0x4000) bb |= 8;
	r |= r >> 5;
	g |= g >> 5;
	bb |= bb >> 5;
	rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | bb);
	if (data & 0x8000) {
		r = (int)(255 - 0.6 * (255 - r));
		g = (int)(255 - 0.6 * (255 - g));
		bb = (int)(255 - 0.6 * (255 - bb));
	} else {
		r = (int)(0.6 * r);
		g = (int)(0.6 * g);
		bb = (int)(0.6 * bb);
	}
	rbx_video.palette[i + NPAL] = (uint32_t)(r << 16 | g << 8 | bb);
}

// a tilemap pixel: palette index (colour * 16 + pen); *fl the tilemap flags (0x10 opaque | the category)
static inline uint16_t tm_pixel(const uint16_t *tr, const uint16_t *cr, int t, int sx, int sy, uint8_t *fl)
{
	const uint16_t v = tr[(t << 12) | ((sy >> 3) << 6) | (sx >> 3)];
	const int pen = (cr[((v & 0xfff) << 4) | ((sy & 7) << 1) | ((sx & 7) >> 2)] >> ((3 - (sx & 3)) << 2)) & 0xf;
	*fl = (uint8_t)((pen ? 0x10 : 0) | (v >> 15));
	return (uint16_t)((((v >> 7) & 0xff) << 4) | pen);
}

// a char byte's two pixels (the left one in the high nibble) as two 16-bit lanes, the lanes' mask (pen != 0) and the
// same mask as two byte lanes (the priority bitmap)
static uint32_t s_pen2[256], s_msk2[256];
static uint16_t s_pm2[256];
static void pen_tables(void)
{
	for (int i = 0; i < 256; i++) {
		const uint32_t hi = (uint32_t)i >> 4, lo = (uint32_t)i & 15;
		s_pen2[i] = hi | lo << 16;
		s_msk2[i] = (hi ? 0xffffu : 0) | (lo ? 0xffff0000u : 0);
		s_pm2[i] = (uint16_t)((hi ? 0xffu : 0) | (lo ? 0xff00u : 0));
	}
}
// per tilemap and row of tiles, the tiles of category 1 (bit: column), from the frame's tile RAM (s24_render)
static uint64_t s_cat1[4][64];

// (unaligned 32-bit words: single ldr/str on ARMv7)
static inline uint32_t ld32(const void *p)
{
	uint32_t v;
	memcpy(&v, p, 4);
	return v;
}
static inline void st32(void *p, uint32_t v) { memcpy(p, &v, 4); }
// a whole tile row (8 pixels from its first one) at d / pr: char word pair w, colour col
static inline void tile8(uint16_t *d, uint8_t *pr, uint32_t w, uint16_t col, int write_zero, uint8_t lpri)
{
	const uint32_t cc = (uint32_t)col * 0x10001u, lp4 = (uint32_t)lpri * 0x01010101u;
	const uint32_t b0 = w >> 24, b1 = (w >> 16) & 255, b2 = (w >> 8) & 255, b3 = w & 255;
	uint32_t p0 = cc | s_pen2[b0], p1 = cc | s_pen2[b1], p2 = cc | s_pen2[b2], p3 = cc | s_pen2[b3];
	uint32_t q0 = ld32(pr), q1 = ld32(pr + 4);
	uint32_t nz = w | (w >> 1);
	nz |= nz >> 2;
	if (write_zero || (nz & 0x11111111u) == 0x11111111u) {
		q0 |= lp4;
		q1 |= lp4;
	} else {
		const uint32_t m0 = s_msk2[b0], m1 = s_msk2[b1], m2 = s_msk2[b2], m3 = s_msk2[b3];
		p0 = (ld32(d) & ~m0) | (p0 & m0);
		p1 = (ld32(d + 2) & ~m1) | (p1 & m1);
		p2 = (ld32(d + 4) & ~m2) | (p2 & m2);
		p3 = (ld32(d + 6) & ~m3) | (p3 & m3);
		q0 |= lp4 & ((uint32_t)s_pm2[b0] | (uint32_t)s_pm2[b1] << 16);
		q1 |= lp4 & ((uint32_t)s_pm2[b2] | (uint32_t)s_pm2[b3] << 16);
	}
	st32(d, p0);
	st32(d + 2, p1);
	st32(d + 4, p2);
	st32(d + 6, p3);
	st32(pr, q0);
	st32(pr + 4, q1);
}

// a span [xa, xb) of one row of tilemap t (src x = x + dx, row sy), by tiles: the tile and its two char words looked
// up once per tile. need_cat: only tiles of category tpri; write_zero: pen 0 written too (else transparent).
// The same pixels as tm_pixel per pixel (whole tiles by tile8, rows without a tile of the category skipped)
static void span_tilemap(uint16_t *d, uint8_t *pr, int t, int xa, int xb, int dx, int sy, int need_cat, int tpri,
                         int write_zero, uint8_t lpri)
{
	const uint64_t cm = tpri ? s_cat1[t][sy >> 3] : ~s_cat1[t][sy >> 3];   // (the row's tiles of the category)
	if (need_cat && !cm)
		return;
	const uint16_t *tr = s_snap->tileram + ((t << 12) | ((sy >> 3) << 6)), *cr = s_snap->charram;
	const int cy = (sy & 7) << 1;
	for (int x = xa; x < xb;) {
		const int sx = (x + dx) & 511, px = sx & 7;
		int n = 8 - px;
		if (n > xb - x)
			n = xb - x;
		const uint16_t v = tr[sx >> 3];
		if (need_cat && (int)(v >> 15) != tpri) {   // (on to the next tile of the category, cm bit c: column c)
			const int c = sx >> 3;
			x += __builtin_ctzll(cm >> c | cm << ((64 - c) & 63)) * 8 - px;
			continue;
		}
		const uint32_t base = ((uint32_t)(v & 0xfff) << 4) | (uint32_t)cy;
		const uint32_t w = (uint32_t)cr[base] << 16 | cr[base + 1];   // (pixel q: bits 28 - 4q)
		if (!write_zero && !w) {
			x += n;
			continue;
		}
		const uint16_t col = (uint16_t)(((v >> 7) & 0xff) << 4);
		uint16_t *dd = d + x;
		uint8_t *pp = pr + x;
		if (n == 8)
			tile8(dd, pp, w, col, write_zero, lpri);
		else if (write_zero)
			for (int i = 0; i < n; i++) {
				dd[i] = (uint16_t)(col | ((w >> (28 - 4 * (px + i))) & 0xf));
				pp[i] |= lpri;
			}
		else
			for (int i = 0; i < n; i++) {
				const uint32_t pen = (w >> (28 - 4 * (px + i))) & 0xf;
				if (pen) {
					dd[i] = (uint16_t)(col | pen);
					pp[i] |= lpri;
				}
			}
		x += n;
	}
}

// tilemap::draw of one row's [x0, x1): tilemap t scrolled (src x = x + dx, src y), category tpri
static void row_tilemap(uint16_t *d, uint8_t *pr, int t, int x0, int x1, int dx, int sy, int tpri, int opaque,
                        uint8_t lpri)
{
	if (x1 > SW) x1 = SW;
	if (x0 < 0) x0 = 0;
	if (x0 < x1)
		span_tilemap(d, pr, t, x0, x1, dx, sy & 511, 1, tpri, opaque, lpri);
}
// draw_rect of one row: the whole width, the window mask per 8 pixels (runs of open groups drawn as spans)
// skip: the 8-pixel groups (bit x >> 3) left out as if masked (opaque groups an opaque layer drawn later covers)
#define GROUPS_ALL ((1ull << (SW / 8)) - 1)
// the 8-pixel groups (bit x >> 3) of a row the window mask (4 words, bit 0x8000 >> n: group 16 k + n masked) leaves
// open; win: the mask inverted
static uint64_t win_open(const uint16_t *mask, int win)
{
	uint64_t c = 0;
	for (int k = 0; k < 4; k++) {
		uint32_t m = mask[k];
		if (win)
			m = ~m & 0xffff;
		m = ((m >> 1) & 0x5555) | ((m & 0x5555) << 1);   // (bit-reversed: group 16 k + n at bit n)
		m = ((m >> 2) & 0x3333) | ((m & 0x3333) << 2);
		m = ((m >> 4) & 0x0f0f) | ((m & 0x0f0f) << 4);
		m = ((m >> 8) & 0x00ff) | ((m & 0x00ff) << 8);
		c |= (uint64_t)(~m & 0xffff) << (16 * k);
	}
	return c & GROUPS_ALL;
}
static void row_rect(uint16_t *d, uint8_t *pr, int t, int dx, int sy, int tpri, int opaque, uint8_t lpri,
                     const uint16_t *mask, int win, uint64_t skip)
{
	uint64_t open = win_open(mask, win) & ~skip;
	while (open) {   // (runs of open groups drawn as spans)
		const int g = __builtin_ctzll(open);
		const int len = __builtin_ctzll(~(open >> g));
		span_tilemap(d, pr, t, g * 8, (g + len) * 8, dx, sy & 511, !opaque, tpri, opaque, lpri);
		open &= ~(((1ull << len) - 1) << g);
	}
}

// the 8-pixel groups of row y an opaque draw_layer(layer) surely writes: the open ones of the window mask (draw_rect
// writes every tile there), none in the line scroll modes (row_tilemap writes only its category's tiles) or when off
static uint64_t layer_cover(int layer, int y)
{
	const uint16_t *tr = s_snap->tileram;
	const uint16_t vscr = tr[0x5004 + (layer >> 1)];
	const uint16_t ctrl = tr[0x5004 + ((layer >> 1) & 2)];
	const int t = layer >> 1;
	if (vscr & 0x8000)
		return 0;
	if (ctrl & 0x6000)   // (row_tilemap draws only the tiles of the layer's category: rows with others are not covered)
		return 0;
	return win_open(tr + ((layer & 4) ? 0x6800 : 0x6000) + y * 4, t & 1);
}
static uint64_t s_skip[4][SH];   // (per band rows: the threads drawing bands write rows of their own)

// segas24_tile_device::draw_common for the rows [y0, y1). skip (opaque draws): per row, the groups an opaque layer
// drawn later covers, left out (the same pixels: the later layer writes every pixel of them, the priority bit too)
static void draw_layer(int layer, int lpri, int opaque, int y0, int y1, const uint64_t *skip)
{
	const uint16_t *tr = s_snap->tileram;
	uint16_t hscr = tr[0x5000 + (layer >> 1)];
	const uint16_t vscr = tr[0x5004 + (layer >> 1)];
	const uint16_t ctrl = tr[0x5004 + ((layer >> 1) & 2)];
	const uint16_t *mask = tr + ((layer & 4) ? 0x6800 : 0x6000);
	const int tpri = layer & 1;
	const uint8_t lp = (uint8_t)(1 << lpri);
	int t = layer >> 1;
	if (vscr & 0x8000)
		return;
	if (ctrl & 0x6000) {
		if (t & 1)
			return;
		const int vs = vscr & 0x1ff;
		const int mode = (ctrl & 0x6000) >> 13;
		if (hscr & 0x8000) {
			const uint16_t *hs = tr + 0x4000 + 0x200 * t;
			if (mode == 1) {
				const int v = (-vscr) & 0x1ff;
				if (!((-vscr) & 0x200))
					t ^= 1;
				for (int y = y0; y < y1; y++) {
					if (skip && skip[y] == GROUPS_ALL)
						continue;
					const int h = hs[y] & 0x1ff;
					row_tilemap(rbx_video.bitmap + y * rbx_video.pitch, s_pri + y * SW, y >= v ? t ^ 1 : t, 0, SW,
					            -h, y + vs, tpri, opaque, lp);
				}
			} else {
				for (int y = y0; y < y1; y++) {
					if (skip && skip[y] == GROUPS_ALL)
						continue;
					const uint16_t hv = hs[y];
					const int h = hv & 0x1ff;
					int l1 = t;
					if (!(hv & 0x200))
						l1 ^= 1;
					uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
					row_tilemap(d, s_pri + y * SW, l1, 0, h, -h, y + vs, tpri, opaque, lp);
					row_tilemap(d, s_pri + y * SW, l1 ^ 1, h, SW, -h, y + vs, tpri, opaque, lp);
				}
			}
		} else {
			const int dx = -(hscr & 0x1ff);
			if (mode == 1) {
				const int v = (-vscr) & 0x1ff;
				if (!((-vscr) & 0x200))
					t ^= 1;
				for (int y = y0; y < y1; y++)
					if (!(skip && skip[y] == GROUPS_ALL))
						row_tilemap(rbx_video.bitmap + y * rbx_video.pitch, s_pri + y * SW, y < v ? t : t ^ 1, 0, SW,
						            dx, y + vs, tpri, opaque, lp);
			} else {
				const int h = hscr & 0x1ff;
				if (!(hscr & 0x200))
					t ^= 1;
				for (int y = y0; y < y1; y++) {
					if (skip && skip[y] == GROUPS_ALL)
						continue;
					uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
					row_tilemap(d, s_pri + y * SW, t, 0, h, dx, y + vs, tpri, opaque, lp);
					row_tilemap(d, s_pri + y * SW, t ^ 1, h, SW, dx, y + vs, tpri, opaque, lp);
				}
			}
		}
	} else {
		const int win = t & 1;
		if (hscr & 0x8000) {
			const uint16_t *hs = tr + 0x4000 + 0x200 * t;
			for (int y = y0; y < y1; y++)
				row_rect(rbx_video.bitmap + y * rbx_video.pitch, s_pri + y * SW, t, (-hs[y]) & 0x1ff,
				         (vscr + y) & 0x1ff, tpri, opaque, lp, mask + y * 4, win, skip ? skip[y] : 0);
		} else {
			hscr = (uint16_t)((-hscr) & 0x1ff);
			for (int y = y0; y < y1; y++)
				row_rect(rbx_video.bitmap + y * rbx_video.pitch, s_pri + y * SW, t, hscr, (vscr + y) & 0x1ff, tpri,
				         opaque, lp, mask + y * 4, win, skip ? skip[y] : 0);
		}
	}
}

// the sprite list (segas24_sprite_device::draw's first loop), from the live RAM into the snapshot
static void sprite_list(void)
{
	const uint16_t *ram = b.spriteram;
	uint16_t curspr = 0;
	int count = 0;
	uint16_t cclip = 0xffff;
	for (int seen = 0; seen < 0x2000; seen++) {
		const uint16_t *source = ram + (curspr << 3);
		const uint16_t so = curspr;
		if (curspr == 0 && source[0] == 0)
			break;
		curspr = source[0];
		const uint16_t type = curspr & 0xc000;
		curspr &= 0x1fff;
		if (type == 0xc000)
			break;
		if (type == 0x8000)
			continue;
		if (type == 0x4000) {
			cclip = so;
			continue;
		}
		s_snap->sprd[count] = so;
		s_snap->clip[count] = cclip;
		count++;
		if (!curspr)
			break;
	}
	s_snap->nspr = count;
}

// segas24_sprite_device::draw for the rows [y0, y1)
static void draw_sprites(int y0, int y1, const int *spri)
{
	const uint16_t *ram = s_snap->spriteram;
	uint8_t pmt[4];
	for (int i = 0; i < 4; i++)
		pmt[i] = (uint8_t)(0xff << (1 + spri[3 - i]));
	for (int n = s_snap->nspr - 1; n >= 0; n--) {
		const uint16_t *source = ram + (s_snap->sprd[n] << 3);
		const uint16_t *cclip = s_snap->clip[n] != 0xffff ? ram + (s_snap->clip[n] << 3) : NULL;
		int min_x, min_y, max_x, max_y, clip_first, clip_last;
		if (cclip) {
			const int rev = (cclip[1] & 0x2000) >> 13;
			min_x = (cclip[3] & 511) - 8;
			if (min_x < 0) min_x = 0;
			max_x = (cclip[5] & 511) - 8;
			if (max_x > SW - 1) max_x = SW - 1;
			if (rev) {
				min_y = y0;
				max_y = y1 - 1;
				clip_first = (cclip[2] & 511) - 1;
				clip_last = (cclip[4] & 511) + 1;
			} else {
				min_y = cclip[2] & 511;
				if (min_y < y0) min_y = y0;
				max_y = cclip[4] & 511;
				if (max_y > y1 - 1) max_y = y1 - 1;
				clip_first = max_y;
				clip_last = min_y;
			}
		} else {
			min_x = 0;
			max_x = SW - 1;
			min_y = y0;
			max_y = y1 - 1;
			clip_first = max_y;
			clip_last = min_y;
		}
		int zoomx, zoomy;
		if (!(source[0] & 0x2000))
			zoomx = zoomy = source[1] & 0xff;
		else {
			zoomx = source[1] >> 8;
			zoomy = source[1] & 0xff;
		}
		if (!zoomx) zoomx = 0x3f;
		if (!zoomy) zoomy = 0x3f;
		zoomx++;
		zoomy++;
		int x = source[5] & 0xfff;
		const int flipx = source[5] & 0x8000;
		if (x & 0x800) x -= 0x1000;
		const int sx = 1 << ((source[5] & 0x7000) >> 12);
		x -= 8;
		int y = source[4] & 0xfff;
		if (y & 0x800) y -= 0x1000;
		const int flipy = source[4] & 0x8000;
		const int sy = 1 << ((source[4] & 0x7000) >> 12);
		// (the rows it covers: one each time the 8 sy steps of zoomy from 0x20 pass 0x40; none in the clip: nothing drawn)
		if (y > max_y || y + ((0x20 + 8 * sy * zoomy) >> 6) <= min_y)
			continue;
		const uint16_t *pix = ram + (source[3] & 0x3fff) * 0x8;
		uint16_t colors[16];
		uint8_t pm[16];
		for (int px = 0; px < 8; px++) {
			int c = pix[px] >> 8;
			pm[px * 2] = pmt[c >> 6];
			if (c > 1) c |= 0x1000;
			colors[px * 2] = (uint16_t)c;
			c = pix[px] & 0xff;
			pm[px * 2 + 1] = pmt[c >> 6];
			if (c > 1) c |= 0x1000;
			colors[px * 2 + 1] = (uint16_t)c;
		}
		const uint32_t offset = (source[2] & 0x7fff) * 0x10;
		int xmod = 0x20, ymod = 0x20;
		for (int py = 0; py < sy; py++) {
			int xmod1 = xmod, xpos1 = x, ypos1 = y, ymod1 = ymod;
			for (int px = 0; px < sx; px++) {
				int xmod2 = xmod1, xpos2 = xpos1;
				const uint32_t addoffset = 0x10 * (uint32_t)(flipx ? sx - px - 1 : px) +
				                           0x10 * (uint32_t)sx * (uint32_t)(flipy ? sy - py - 1 : py) + (flipy ? 7 * 2 : 0);
				uint32_t newoffset = offset + addoffset;
				ymod1 = ymod;
				ypos1 = y;
				for (int zy = 0; zy < 8; zy++) {
					ymod1 += zoomy;
					while (ymod1 >= 0x40) {
						if (ypos1 >= min_y && ypos1 <= max_y && (ypos1 <= clip_first || ypos1 >= clip_last)) {
							xmod2 = xmod1;
							xpos2 = xpos1;
							uint16_t *drow = rbx_video.bitmap + ypos1 * rbx_video.pitch;
							uint8_t *prow = s_pri + ypos1 * SW;
							// (the row's 8 pens: pixel zx1 at bits 28 - 4 zx1)
							const uint32_t rw = (uint32_t)ram[newoffset & 0x1ffff] << 16 | ram[(newoffset + 1) & 0x1ffff];
							for (int zx = 0; zx < 8; zx++) {
								xmod2 += zoomx;
								while (xmod2 >= 0x40) {
									if (xpos2 >= min_x && xpos2 <= max_x) {
										const int zx1 = flipx ? 7 - zx : zx;
										int c = (int)(rw >> (28 - 4 * zx1)) & 0xf;
										uint8_t *pri = prow + xpos2;
										if (!(*pri & pm[c])) {
											c = colors[c];
											if (c) {
												uint16_t *dst = drow + xpos2;
												if (c == 1)
													*dst |= 0x2000;
												else
													*dst = (uint16_t)c;
												*pri = 0xff;
											}
										}
									}
									xmod2 -= 0x40;
									xpos2++;
								}
							}
						}
						ymod1 -= 0x40;
						ypos1++;
					}
					if (flipy)
						newoffset -= 2;
					else
						newoffset += 2;
				}
				xpos1 = xpos2;
				xmod1 = xmod2;
			}
			y = ypos1;
			ymod = ymod1;
		}
	}
}

#ifdef S24_PROF
#include <time.h>
static double s_ph[4];
static int s_prof_n;
static void prof_print(void) { int n = s_prof_n ? s_prof_n : 1; fprintf(stderr, "S24PROF %d frames clr %.3f opq %.3f trn %.3f spr %.3f\n", s_prof_n, s_ph[0] / n, s_ph[1] / n, s_ph[2] / n, s_ph[3] / n); }
static double prof_now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * 1e3 + t.tv_nsec / 1e6; }
#define PROF_T(v) const double v = prof_now()
#else
#define PROF_T(v)
#endif
#define BANDS 8
static void s24_draw(int y0, int y1)
{
	if (s_snap->mixer[13] & 1) {
		for (int y = y0; y < y1; y++)
			for (int x = 0; x < SW; x++)
				rbx_video.bitmap[y * rbx_video.pitch + x] = PAL_BLACK;
		return;
	}
	PROF_T(t0);
	for (int y = y0; y < y1; y++) {
		memset(rbx_video.bitmap + y * rbx_video.pitch, 0, SW * sizeof(uint16_t));
		memset(s_pri + y * SW, 0, SW);
	}
	PROF_T(t1);
	const int *order = s_snap->order;
	// zero pixels of the bottommost layer show colour 0 of their palette: the layers drawn opaque in reverse, each
	// without the groups the ones drawn after it cover
	int el[4], ne = 0;
	for (int i = 11; i >= 0; i--)
		if (order[i] < 8 && (order[i] & 1) == 0)
			el[ne++] = order[i];
	for (int y = y0; y < y1; y++) {
		uint64_t later = 0;
		for (int k = ne - 1; k >= 0; k--) {
			s_skip[k][y] = later;
			later |= layer_cover(el[k], y);
		}
	}
	for (int k = 0; k < ne; k++)
		draw_layer(el[k], 0, 1, y0, y1, s_skip[k]);
	PROF_T(t2);
	int spri[4] = { 0, 0, 0, 0 }, level = 0;
	// the first layer of the order when it is the last opaque one (category 0, level 0): it would write again the
	// pixels and priority bit its opaque draw wrote last, left out
	for (int i = (ne && order[0] == el[ne - 1]) ? 1 : 0; i < 12; i++) {
		if (order[i] < 8)
			draw_layer(order[i], level, 0, y0, y1, NULL);
		else
			spri[order[i] - 8] = level++;
	}
	PROF_T(t3);
	draw_sprites(y0, y1, spri);
#ifdef S24_PROF
	PROF_T(t4);
	static int reg;
	if (!reg) { reg = 1; atexit(prof_print); }
	s_ph[0] += t1 - t0; s_ph[1] += t2 - t1; s_ph[2] += t3 - t2; s_ph[3] += t4 - t3;
	if (y0 == 0) s_prof_n++;
#endif
}

// the frame's output written by the bands (the core's video_out on one thread took ~6 ms of the render thread on the
// Vita): the palette converted to the frontend's colour (s_olut, the entries that changed, as video_out's table) and
// each band's rows written with it. The same pixels as video_out
static uint32_t *s_olut, *s_olut_src;
static uint32_t (*s_olut_fn)(int, int, int);   // (no parameter names: b is the board macro)
static int s_olut_bpp, s_out;
static void band_out(int y0, int y1)
{
	const int mask = rbx_video.palette_size - 1;
	for (int y = y0; y < y1; y++) {
		const uint16_t *s = rbx_video.bitmap + y * rbx_video.pitch;
		uint8_t *op = (uint8_t *)rbx_video.out + (size_t)y * rbx_video.out_pitch;
		if (s_olut_bpp == 2) {
			uint16_t *d = (uint16_t *)op;
			for (int x = 0; x < SW; x++)
				d[x] = (uint16_t)s_olut[s[x] & mask];
		} else {
			uint32_t *d = (uint32_t *)op;
			for (int x = 0; x < SW; x++)
				d[x] = s_olut[s[x] & mask];
		}
	}
}
static void s24_band(int band)
{
	const int y0 = SH * band / BANDS, y1 = SH * (band + 1) / BANDS;
	s24_draw(y0, y1);
	if (s_out)
		band_out(y0, y1);
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, s24_band };
static int s24_help(int helper) { return decoband_help(&s_bands, helper); }

static void out_table(void)   // (video_out's table: the entries that changed, all of them for another format)
{
	const int all = s_olut_fn != rbx_video.out_color || s_olut_bpp != rbx_video.out_bpp;
	uint32_t (*fn)(int, int, int) = rbx_video.out_color;
	for (int i = 0; i < rbx_video.palette_size; i++) {
		const uint32_t c = rbx_video.palette[i];
		if (!all && s_olut_src[i] == c)
			continue;
		s_olut_src[i] = c;
		s_olut[i] = fn((int)(c >> 16) & 0xff, (int)(c >> 8) & 0xff, (int)c & 0xff);
	}
	s_olut_fn = fn;
	s_olut_bpp = rbx_video.out_bpp;
}

static void s24_render(void)
{
	if (s_snap->pal_dirty || !s_pal_valid) {
		for (int i = 0; i < NPAL; i++) {
			const uint16_t v = s_snap->paletteram[i];
			if (s_pal_valid && s_pal_conv[i] == v)
				continue;
			s_pal_conv[i] = v;
			pal_entry(i, v);
		}
		rbx_video.palette[PAL_BLACK] = 0;
		s_pal_valid = 1;
		s_snap->pal_dirty = 0;
	}
	if (!s_pen2[1])
		pen_tables();
	for (int r = 0; r < 4 * 64; r++) {
		const uint16_t *tr = s_snap->tileram + r * 64;
		uint32_t lo = 0, hi = 0;
		for (int c = 0; c < 32; c++) {
			lo |= (uint32_t)(tr[c] >> 15) << c;
			hi |= (uint32_t)(tr[c + 32] >> 15) << c;
		}
		s_cat1[r >> 6][r & 63] = (uint64_t)hi << 32 | lo;
	}
	s_out = rbx_video.out && !rbx_video.blank && s_olut && (rbx_video.out_bpp == 2 || rbx_video.out_bpp == 4);
	if (s_out)
		out_table();
	decoband_run(&s_bands);
	if (s_out)
		rbx_video.out_done = 1;
}

static void vblank_draw(void)   // (screen_update at the vblank's end)
{
	rbx_video_wait_idle();
	if (memcmp(s_snap->paletteram, b.paletteram, sizeof(b.paletteram))) {
		memcpy(s_snap->paletteram, b.paletteram, sizeof(b.paletteram));
		s_snap->pal_dirty = 1;
	}
	memcpy(s_snap->tileram, b.tileram, sizeof(b.tileram));
	memcpy(s_snap->charram, b.charram, sizeof(b.charram));
	memcpy(s_snap->spriteram, b.spriteram, sizeof(b.spriteram));
	memcpy(s_snap->mixer, b.mixer, sizeof(b.mixer));
	sprite_list();
	// the layers' order: by the mixer's priorities, then the default one (higher first)
	static const int default_pri[12] = { 0, 1, 2, 3, 4, 5, 6, 7, -4, -3, -2, -1 };
	int *order = s_snap->order;
	for (int i = 0; i < 12; i++)
		order[i] = i;
	for (int i = 1; i < 12; i++) {   // (insertion sort on a total order: std::sort's result)
		const int v = order[i];
		int j = i - 1;
		for (; j >= 0; j--) {
			const int l1 = v, l2 = order[j];
			const int p1 = b.mixer[l1] & 7, p2 = b.mixer[l2] & 7;
			const int before = p1 != p2 ? p1 < p2 : default_pri[l2] < default_pri[l1];
			if (!before)
				break;
			order[j + 1] = order[j];
		}
		order[j + 1] = v;
	}
	rbx_video_frame_done();
}

// ---- sound ----
static void sound_line(void)
{
	const int n = rbx_ym_advance_lr(0, YM_LINE, s_mix_l + s_mix_n, s_mix_r + s_mix_n, s_mix_cap - s_mix_n);
	const int32_t d = (int32_t)b.dac * 64;   // (the 8-bit DAC at 0.5 on both sides)
	for (int k = 0; k < n; k++) {
		s_mix_l[s_mix_n + k] += d;
		s_mix_r[s_mix_n + k] += d;
	}
	s_mix_n += n;
}
static void sound_frame_end(void)
{
	rbx_ym_async_take_lr(0, s_mix_l, s_mix_r, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put_stereo(s_mix_l[i], s_mix_r[i]);
	memset(s_mix_l, 0, sizeof(int32_t) * (size_t)s_mix_n);
	memset(s_mix_r, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

// ---- the scanline timer (irq_vbl), the frame ----
static void tm_scan(void)
{
	const int line = b.scan_line;
	const uint64_t t = b.texp[TM_SCAN];
	b.scan_line = (uint16_t)((line + 1) % LINES);
	t_adjust(TM_SCAN, t + T_LINE);
	sound_line();
	// MAME's screen starts in the vblank (line 384) with the scanline timer at the next line 0, its first vblank_end
	// (the frame's update) a frame after that
	const int first = !b.vbl_on;
	if (line == 0)
		b.vbl_on = 1;
	if (!b.vbl_on)
		return;
	if (line == 0 && !first) {
		vblank_draw();
		s_frame_done = 1;
	}
	int irq;
	if (line == 0) {
		irq = IRQ_SPRITE;
		b.irq_sprite = 1;
	} else if (line == 384) {
		irq = IRQ_VBLANK;
		b.irq_vblank = 1;
	} else
		return;
	t_adjust(TM_CLEAR, t + T_LINE);
	if (b.irq_allow0 & (1 << irq))
		set_line(0, irq, 1);
	if (b.irq_allow1 & (1 << irq))
		set_line(1, irq, 1);
	if (line == 384) {   // (an index pulse every 20 frames)
		b.fdc_index_count++;
		if (b.fdc_index_count >= 20)
			b.fdc_index_count = 0;
	}
	irq_timer_sync();
	b.irq_vsynctime = t;
}

static void execute_timers(void)
{
	for (;;) {
		int best = -1;
		for (int i = 0; i < NTIMERS; i++)
			if (b.texp[i] <= b.base && (best < 0 || b.texp[i] < b.texp[best] ||
			                            (b.texp[i] == b.texp[best] && (int32_t)(b.tseq[i] - b.tseq[best]) < 0)))
				best = i;
		if (best < 0)
			break;
		s_cbtime = b.texp[best];
		switch (best) {
		case TM_SCAN: tm_scan(); break;
		case TM_CLEAR: tm_clear(); break;
		case TM_IRQ: tm_irq(); break;
		case TM_FRC: tm_frc(); break;
		case TM_SYNC: tm_sync(); break;
		case TM_HACK:
			b.texp[TM_HACK] = NEVER;
			b.per[1] = T_CYC;
			break;
		}
	}
	s_cbtime = b.base;
}

static uint64_t next_timer(void)
{
	uint64_t t = NEVER;
	for (int i = 0; i < NTIMERS; i++)
		if (b.texp[i] < t)
			t = b.texp[i];
	return t;
}

// device_scheduler::timeslice: the CPUs to the next timer (a CPU's slice cut short moves the target back)
static void timeslice(void)
{
	while (b.base < next_timer()) {
		uint64_t target = next_timer();
		for (int i = 0; i < 2; i++) {
			if (target <= b.lt[i])
				continue;
			const uint64_t per = b.per[i];
			const uint64_t delta = target - b.lt[i];
			if (delta < per)
				continue;
			const int cycles = (int)(delta / per);
			int ran;
			if (i == 1 && b.halted)
				ran = cycles;   // (halted: its cycles eaten)
			else {
				cpu_switch(i);
				s_exec = 1;
				s_target = target;
				ran = m68k_execute(cycles);
				s_exec = 0;
			}
			b.lt[i] += (uint64_t)ran * per;
			b.cyc_total[i] += (uint64_t)ran;
			if (b.lt[i] < target)
				target = b.lt[i] > b.base ? b.lt[i] : b.base;
		}
		b.base = target;
		s_cbtime = b.base;
	}
	execute_timers();
}

// ---- the dials (uPD4701: the counters moved by the frame's deltas) ----
static void analog_frame(void)
{
	if (s_ext == EXT_NONE)
		return;
	const int nd = s_ext == EXT_HOTROD ? 4 : 2;
	for (int i = 0; i < nd; i++) {
		const int d = rbx_analog_dpad(i + 1, RBX_IPT_DIAL, 15);
		uint16_t *c = (i & 1) ? &b.upd_y[i >> 1] : &b.upd_x[i >> 1];
		*c = (uint16_t)((*c + d) & 0xfff);
	}
}

// ---- init / frame ----
static void s24_reset(void)
{
	memset(s_bp, 0, offsetof(struct s24_state, state_end));
	for (int i = 0; i < NTIMERS; i++)
		b.texp[i] = NEVER;
	b.per[0] = b.per[1] = T_CYC;
	b.ym_reset_state = 1;   // (the 315-5296's reset: the ports and CNT outputs to 0)
	ym_reset_w(0);
	b.cnt1 = 0;
	// machine_reset: the second CPU halted, CNT1 taken as high
	b.halted = b.halt_want = 1;
	b.cnt1 = 1;
	b.scan_line = 384;
	b.texp[TM_SCAN] = 0;   // (line 384 at time 0: the sound's lines; the interrupts from line 0)
	b.tseq[TM_SCAN] = ++b.seq;
	b.texp[TM_FRC] = 0;   // (a periodic timer device: first at its start delay, 0)
	b.tseq[TM_FRC] = ++b.seq;
	rbx_ym_reset(0);
	s_mix_n = 0;
	s_exec = 0;
	s_cbtime = 0;
	if (s_fd1094_on) {
		fd1094_exit(&s_fd1094);
		fd1094_change_state(&s_fd1094, FD1094_STATE_RESET);
	}
	m68k_bus_select(NULL);
	s_cur = 0;
	for (int i = 1; i >= 0; i--) {
		m68k_bus_select(i ? &s_bus1 : NULL);
		m68k_bus_irq_reset();
		m68k_bus_cur->irq_asserted = 0;
		if (i)
			fd_update();
		m68k_pulse_reset();
		m68k_bus_update_irq();   // (the core's interrupt level from the lines: not the last game's)
		if (i)
			m68k_get_context(s_ctx[i]);
		s_irqp[i] = 0;
	}
	reset_bank();
	s_pal_valid = 0;
}

static void s24_exit(void);

static int s24_init(const rbx_game *g)
{
	char prefix[48];
	{
		const char *c = strchr(g->machine, ':');
		size_t n = c ? (size_t)(c - g->machine) : strlen(g->machine);
		if (n >= sizeof(prefix))
			n = sizeof(prefix) - 1;
		memcpy(prefix, g->machine, n);
		prefix[n] = 0;
	}
	s_fd1094_on = strstr(prefix, "fd1094") != NULL || !strcmp(prefix, "system24_floppy_fd_upd") ||
	              !strcmp(prefix, "system24_floppy_dcclub") || !strcmp(prefix, "sgmastj");
	s_io_kind = !strcmp(prefix, "mahmajn") ? IO_MAHMAJN
	          : (!strcmp(prefix, "dcclub") || !strcmp(prefix, "dcclubj") || !strcmp(prefix, "system24_floppy_dcclub") ||
	             !strcmp(prefix, "sgmastj")) ? IO_DCCLUB : IO_GENERIC;
	s_ext = !strcmp(prefix, "system24_floppy_hotrod") ? EXT_HOTROD
	      : (!strcmp(prefix, "system24_floppy_fd_upd") || !strcmp(prefix, "dcclubj") || !strcmp(prefix, "sgmastj")) ? EXT_UPD
	      : EXT_NONE;
	const int rombd = !strcmp(prefix, "system24_rom") || !strcmp(prefix, "mahmajn") || !strcmp(prefix, "dcclub") ||
	                  !strcmp(prefix, "dcclubj") || !strcmp(prefix, "system24_floppy_rom");
	const int floppy = !strncmp(prefix, "system24_floppy", 15) || !strcmp(prefix, "sgmastj");
	// the driver inits (init_*): the magic latch's table, the floppy's track size
	const char *s = g->set;
	s_mlatch_table = NULL;
	s_track_size = 0;
	s_gground = 0;
	if (!strncmp(s, "qgh", 3)) s_mlatch_table = s_qgh_mlt;
	else if (!strcmp(s, "dcclubfd")) { s_mlatch_table = s_dcclub_mlt; s_track_size = 0x2d00; }
	else if (!strncmp(s, "dcclub", 6)) s_mlatch_table = s_dcclub_mlt;
	else if (!strncmp(s, "qrouka", 6)) s_mlatch_table = s_qrouka_mlt;
	else if (!strncmp(s, "quizmeku", 8)) s_mlatch_table = s_quizmeku_mlt;
	else if (!strcmp(s, "mahmajn2")) s_mlatch_table = s_mahmajn2_mlt;
	else if (!strncmp(s, "mahmajn", 7)) s_mlatch_table = s_mahmajn_mlt;
	else if (!strncmp(s, "hotrod", 6)) s_track_size = 0x2f00;
	else if (!strncmp(s, "bnzabros", 8)) { s_mlatch_table = s_bnzabros_mlt; s_track_size = 0x2d00; }
	else if (!strcmp(s, "sspiritj")) s_track_size = 0x2f00;
	else if (!strncmp(s, "gground", 7)) { s_track_size = 0x2d00; s_gground = 1; }
	else s_track_size = 0x2d00;   // (sspirits, sspirtfc, sgmast*, qsww, crkdown*, roughrac)
	if (!floppy)
		s_track_size = 0;

	if (!s_bp && !(s_bp = calloc(1, sizeof(*s_bp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_bios = rbx_region("maincpu", &s_bios_size);
	s_romboard = rombd ? rbx_region("romboard", &s_romboard_size) : NULL;
	s_floppy = floppy ? rbx_region("floppy", &s_floppy_size) : NULL;
	if (!s_bios || s_bios_size < 0x40000 || (rombd && (!s_romboard || s_romboard_size < 0x400000)) ||
	    (floppy && !s_floppy)) {
		rbx_fail("%s: ROM regions missing", g->set);
		s24_exit();
		return -1;
	}
	if (floppy && s_floppy_size < (uint32_t)s_track_size * 160)
		s_track_size = 0;
	m68k_bus_swap16(s_bios, s_bios_size);
	if (s_romboard)
		m68k_bus_swap16(s_romboard, s_romboard_size);
	memset(&s_fd1094, 0, sizeof(s_fd1094));
	if (s_fd1094_on) {
		uint32_t ksz;
		s_key = rbx_region("subcpu:key", &ksz);
		if (!s_key || ksz < 0x2000) {
			rbx_fail("%s: no FD1094 key", g->set);
			s24_exit();
			return -1;
		}
		fd1094_init(&s_fd1094, b.subram, 0x40000, s_key);
	}
	s_snap = calloc(1, sizeof(*s_snap));
	s_pal_conv = (uint16_t *)calloc(NPAL, sizeof(uint16_t));
	s_pri = (uint8_t *)calloc(SW * SH, 1);
	if (!s_snap || !s_pal_conv || !s_pri) {
		rbx_fail("%s: out of memory", g->set);
		s24_exit();
		return -1;
	}
	s_port_p1 = rbx_port_find("P1");
	s_port_p2 = rbx_port_find("P2");
	s_port_p3 = rbx_port_find("P3");
	s_port_service = rbx_port_find("SERVICE");
	s_port_coinage = rbx_port_find("COINAGE");
	s_port_dsw = rbx_port_find("DSW");
	s_port_paddle = rbx_port_find("PADDLE");
	{
		static const char *const mj[8] = { "MJ0", "MJ1", "MJ2", "MJ3", "MJ4", "MJ5", "P1", "P2" };
		for (int i = 0; i < 8; i++)
			s_port_mj[i] = rbx_port_find(mj[i]);
		for (int i = 0; i < 4; i++) {
			char tag[8];
			snprintf(tag, sizeof(tag), "PEDAL%d", i + 1);
			s_port_pedal[i] = rbx_port_find(tag);
		}
	}
	// sound: the YM2151 at 4 MHz, 0.50 on each side
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = 4000000;
	yc.gain[0] = yc.gain[1] = 128;
	yc.irq = irq_ym;
	if (rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 2)) {
		rbx_fail("%s: out of memory", g->set);
		s24_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix_l = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_mix_r = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix_l || !s_mix_r || rbx_audio_setup_stereo(rbx_ym_rate(0)) || rbx_video_setup(SW, SH, PAL_BLACK + 1)) {
		rbx_fail("%s: out of memory", g->set);
		s24_exit();
		return -1;
	}
	// (the bands' output table, by the palette's size from rbx_video_setup; without it the core converts the frame)
	s_olut = (uint32_t *)calloc((size_t)rbx_video.palette_size, sizeof(uint32_t));
	s_olut_src = (uint32_t *)calloc((size_t)rbx_video.palette_size, sizeof(uint32_t));
	s_olut_fn = NULL;
	s_olut_bpp = 0;
	if (!s_olut || !s_olut_src) {
		free(s_olut);
		free(s_olut_src);
		s_olut = s_olut_src = NULL;
	}
	if (m68k_context_size() > sizeof(s_ctx[0])) {
		rbx_fail("%s: CPU context too large", g->set);
		s24_exit();
		return -1;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);   // (MAME's DIVx / MULx / ADDQ.W An times)
	map_bus(1);
	map_bus(0);
	if (s_fd1094_on) {
		rbx_m68k_cmpild_hook = fd_cmpild;
		rbx_m68k_rte_hook = fd_rte;
	}
	rbx_m68k_irqack_hook = irqack;
	s_cur = 0;
	s24_reset();
	return 0;
}

static void s24_exit(void)
{
	m68k_bus_select(NULL);
	s_cur = 0;
	s_bus1.op = NULL;
	s_bus1.op_size = 0;
	rbx_m68k_cmpild_hook = NULL;
	rbx_m68k_rte_hook = NULL;
	rbx_m68k_irqack_hook = NULL;
	fd1094_exit(&s_fd1094);
	s_fd1094_on = 0;
	m68k_set_mame_timing(0);
	rbx_ym_exit(0);
	free(s_mix_l);
	free(s_mix_r);
	s_mix_l = s_mix_r = NULL;
	free(s_snap);
	s_snap = NULL;
	free(s_pal_conv);
	s_pal_conv = NULL;
	free(s_olut);
	free(s_olut_src);
	s_olut = s_olut_src = NULL;
	free(s_pri);
	s_pri = NULL;
	free(s_bp);
	s_bp = NULL;
}

static void s24_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	analog_frame();
	s_frame_done = 0;
	while (!s_frame_done)
		timeslice();
	cpu_switch(0);
	sound_frame_end();
}

static int s24_state_io(void *ctx, int load)
{
	cpu_switch(0);
	const uint32_t csize = m68k_context_size();
	if (!load) {
		m68k_get_context(s_ctx[0]);
		b.fd_st = s_fd1094.st;
		b.fd_irqmode = s_fd1094.irqmode;
	}
	uint32_t irq[4] = { m68k_bus_irq_pending(), s_irqp[1], m68k_bus.irq_asserted, s_bus1.irq_asserted };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", s_ctx[0], csize);
	rc |= rbx_state_io(ctx, "m68k_sub", s_ctx[1], csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "s24", s_bp, offsetof(struct s24_state, state_end));
	rc |= rbx_ym_state(0, ctx, load);
	if (!rc && load) {
		m68k_set_context(s_ctx[0]);
		m68k_bus_set_irq_pending(irq[0]);
		s_irqp[1] = irq[1];
		m68k_bus.irq_asserted = irq[2];
		s_bus1.irq_asserted = irq[3];
		m68k_bus_update_irq();
		if (s_fd1094_on) {
			fd1094_exit(&s_fd1094);
			s_fd1094.st = b.fd_st;
			s_fd1094.irqmode = b.fd_irqmode;
			fd_update();
		}
		reset_bank();
		s_cbtime = b.base;
		s_pal_valid = 0;
	}
	return rc ? -1 : 0;
}

static int s24_save(void *ctx) { return s24_state_io(ctx, 0); }
static int s24_load(void *ctx) { return s24_state_io(ctx, 1); }
static int s24_supported(const char *machine)
{
	static const char *const ok[] = { "system24:", "system24_rom:", "mahmajn:", "system24_floppy:", "system24_floppy_rom:",
	                                  "system24_floppy_hotrod:", "system24_floppy_fd1094:", "system24_floppy_fd_upd:",
	                                  "dcclub:", "dcclubj:", "system24_floppy_dcclub:", "sgmastj:", NULL };
	for (int i = 0; ok[i]; i++)
		if (!strncmp(machine, ok[i], strlen(ok[i])))
			return 1;
	return 0;
}

const rbx_driver rbx_drv_segas24 = { "Sega System 24", s24_init, s24_exit, s24_reset, s24_frame, s24_save, s24_load,
                                     s24_supported, s24_render, s24_help };
