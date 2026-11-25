/*  Sega Out Run board: Out Run, Turbo Out Run (and Super Hang-On, not yet). Ported from MAME src/mame/sega/segaorun.cpp,
    315_5195.cpp (the memory mapper), segaic16.cpp (the 16B tilemaps), segaic16_road.cpp, sega16sp.cpp (the Out Run
    sprites), src/devices/machine/i8255.cpp, adc0804.cpp and src/devices/sound/segapcm.cpp (BSD-3-Clause,
    copyright-holders: Aaron Giles, David Haywood, Hiromitsu Shioya, Olivier Galibert, AJR).
    A 68000 at 10 MHz (maybe an FD1089A or FD1094) behind the 315-5195 memory mapper, a second 68000 at 10 MHz with the
    road, their IRQs from the scanline timer (IRQ 2 at lines 65, 129 and 193, the vblank's IRQ 4 at line 223 for one
    line); the 16B tilemaps, the road (two layers from ROM), the zooming sprites (a buffer swapped by a write), 4096
    colours with the shadow / hilight copies. Sound: a Z80 at 4 MHz with a YM2151 and a SegaPCM (315-5218), its NMI on
    the mapper's sound latch. 400x262 at 6.29 MHz, 320x224 shown.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../rbx_core.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"
#include "../../video/decoband.h"
#include "../../cpu/m68k_spin.h"
#include "../machine/fd1089.h"
#include "../machine/fd1094.h"
#include "../machine/segapcm.h"
#include "../machine/sega_zspin.h"

extern void (*rbx_m68k_cmpild_hook)(int reg, unsigned int imm);
extern void (*rbx_m68k_rte_hook)(void);
extern void (*rbx_m68k_irqack_hook)(int level);

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 10000000         // 40 MHz / 4
#define Z80_CLOCK 4000000          // 16 MHz / 4
#define YM_CLOCK 4000000
#define FRAME_MHZ 60055            // 25.1748 MHz / 4 / (400 x 262)
#define LINES 262
#define VIS_W 320
#define VIS_H 224
#define NPAL 4096
#define PAL_BLACK (2 * NPAL)

// ---- ROMs ----
static uint8_t *s_rom, *s_subrom, *s_tiles, *s_spr, *s_road, *s_zrom, *s_pcmrom;
static uint32_t s_rom_size, s_subrom_size, s_tiles_size, s_spr_size, s_road_size, s_zrom_size, s_pcm_size;
static int s_fd1094_on, s_outrunb, s_shangon;
static fd1094_t s_fd1094;
static uint16_t *s_fd1089_op;   // (FD1089A: the opcodes decrypted at init)
static int s_port_dig[4], s_port_adc[8];
static uint8_t s_adc_def[8], s_adc_type[8];

// ---- the state (saved; on the heap while the game runs, ADR-040) ----
static struct orun_state {
	uint16_t workram[0x4000];        // 32 KB (the NVRAM)
	uint16_t tileram[0x8000];
	uint16_t textram[0x800];
	uint16_t spriteram[0x800], spritebuf[0x800];
	uint16_t paletteram[0x1000];
	uint16_t subram[0x4000];         // cpu1ram, 32 KB
	uint16_t roadram[0x800], roadbuf[0x800];
	uint16_t latched_page[4], latched_yscroll[4], latched_xscroll[4];
	uint8_t regs[0x20];              // the 315-5195's
	uint8_t to_sound, reset_held, pbf;
	uint8_t road_control, display_enable, adc_select, mute;
	uint8_t irq2, vblank_irq, sound_reset, sub_reset;
	uint8_t ppi_latch[3], ppi_ctrl;
	uint8_t adc_result, adc_intr, adc_busy;
	uint64_t adc_done;                         // (the main CPU's time the conversion ends, in its cycles)
	uint8_t adc_val[8];                        // (the analog channels as the controls move them)
	uint64_t main_time;                        // the main CPU's cycles at the start of its slice
	uint64_t cpu_frac[2], z80_frac, ym_frac;
	uint32_t pcm_phase;
	int32_t pcm_hl, pcm_hr;
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)

static m68k_spin_t s_spin[2];
static int s_spin_on = 1;
static uint8_t s_ctx[2][2048];
static int s_cur;
static uint32_t s_irqp[2];
static m68k_bus_t s_bus_sub;

static z80 s_z80;
static int s_zirq;
static uint8_t s_zram[0x800];
static const uint8_t *s_zmap[256];
static segapcm_t s_pcm;
static int32_t *s_mix_l, *s_mix_r;
static int s_mix_n, s_mix_cap;

// ---- the two 68000s on one Musashi core ----
static void cpu_switch(int n)
{
	if (n == s_cur)
		return;
	s_irqp[s_cur] = m68k_bus_irq_pending();
	m68k_switch_context(s_ctx[s_cur], s_ctx[n]);
	m68k_bus_select(n ? &s_bus_sub : NULL);
	s_cur = n;
	m68k_bus_set_irq_pending(s_irqp[n]);
	m68k_bus_update_irq();
}

static uint64_t main_now(void) { return b.main_time + (uint64_t)(s_cur == 0 ? m68k_cycles_run() : 0); }

// ---- the FD1094 (main CPU only) ----
static void fd_update(void)
{
	if (!s_fd1094_on)
		return;
	const uint16_t *op = fd1094_opcodes(&s_fd1094, fd1094_state(&s_fd1094));
	m68k_bus.op = (const uint8_t *)op;
	m68k_bus.op_size = op ? s_rom_size : 0;
}
static void fd_cmpild(int reg, unsigned int imm)
{
	if (s_cur == 0 && reg == 0 && (imm & 0xffff) == 0xffff) {
		fd1094_change_state(&s_fd1094, (int)(imm >> 16));
		fd_update();
	}
}
static void fd_rte(void)
{
	if (s_cur != 0)
		return;
	fd1094_change_state(&s_fd1094, FD1094_STATE_RTE);
	fd_update();
}
static void fd_irqack(int level)
{
	(void)level;
	if (s_cur != 0)
		return;
	fd1094_change_state(&s_fd1094, FD1094_STATE_IRQ);
	fd_update();
}

// ---- interrupts (update_main_irqs; the main CPU's context selected) ----
static void update_main_irqs(void)
{
	m68k_bus_set_line(2, b.irq2);
	m68k_bus_set_line(4, b.vblank_irq);
	m68k_bus_set_line(6, b.irq2 && b.vblank_irq);
}

static void reset_instr(void)   // m68k_reset_callback: the main CPU's RESET resets the sub CPU
{
	if (s_cur == 0) {
		b.sub_reset = 1;
		m68k_end_timeslice();
	}
}

// ---- the ADC0804, the i8255 ----
static void adc_poll(void)
{
	if (b.adc_busy && main_now() >= b.adc_done) {   // conversion_done
		b.adc_busy = 0;
		b.adc_result = s_port_adc[b.adc_select] >= 0 ? b.adc_val[b.adc_select] : 0x10;   // analog_r
		b.adc_intr = 1;
	}
}
static uint8_t adc_r(void)
{
	adc_poll();
	b.adc_intr = 0;
	return b.adc_result;
}
static void adc_w(void)
{
	adc_poll();
	b.adc_intr = 0;
	if (!b.adc_busy) {   // (about 74 clocks of 1.05 MHz: 705 of the CPU's)
		b.adc_busy = 1;
		b.adc_done = main_now() + 705;
	}
}

static void z80_bind(void);
static void video_control_w(uint8_t data)
{
	b.display_enable = (data & 0x20) != 0;
	b.adc_select = (data >> 2) & 7;
	const uint8_t reset = !(data & 1);
	if (reset && !b.sound_reset) {   // (held in reset: started again on release)
		const unsigned long cyc = s_z80.cyc;
		z80_init(&s_z80);
		z80_bind();
		s_z80.cyc = cyc;
	}
	b.sound_reset = reset;
}
static uint8_t ppi_in(int port)
{
	if (port == 0) {   // bankmotor_limit_r: the motor at the centre
		adc_poll();
		uint8_t v = 0xff ^ 0x10;
		if (b.adc_intr)
			v ^= 0x40;
		return v;
	}
	return 0;
}
static uint8_t ppi_r(int off)
{
	off &= 3;
	const uint8_t c = b.ppi_ctrl;
	switch (off) {
	case 0: return (c & 0x10) ? ppi_in(0) : b.ppi_latch[0];
	case 1: return (c & 0x02) ? ppi_in(1) : b.ppi_latch[1];
	case 2: {
		uint8_t v = b.ppi_latch[2];
		if (c & 0x01) v = (uint8_t)((v & 0xf0) | (ppi_in(2) & 0x0f));
		if (c & 0x08) v = (uint8_t)((v & 0x0f) | (ppi_in(2) & 0xf0));
		return v;
	}
	}
	return 0xff;
}
static void ppi_out_c(void)
{
	uint8_t m = 0;
	if (!(b.ppi_ctrl & 0x01)) m |= 0x0f;
	if (!(b.ppi_ctrl & 0x08)) m |= 0xf0;
	video_control_w((uint8_t)(b.ppi_latch[2] & m));
}
static void ppi_w(int off, uint8_t data)
{
	off &= 3;
	if (off < 3) {
		b.ppi_latch[off] = data;
		if (off == 2)
			ppi_out_c();
		return;
	}
	if (data & 0x80) {   // mode set: the outputs cleared
		b.ppi_ctrl = data;
		memset(b.ppi_latch, 0, sizeof(b.ppi_latch));
		ppi_out_c();
	} else {             // port C bit set / reset
		const int bit = (data >> 1) & 7;
		if (data & 1)
			b.ppi_latch[2] |= (uint8_t)(1 << bit);
		else
			b.ppi_latch[2] &= (uint8_t)~(1 << bit);
		ppi_out_c();
	}
}

// ---- the road's and the sprites' buffers ----
static uint16_t road_control_r(void)
{
	uint32_t *s = (uint32_t *)b.roadram, *d = (uint32_t *)b.roadbuf;
	for (int i = 0; i < 0x1000 / 4; i++) {
		const uint32_t t = s[i];
		s[i] = d[i];
		d[i] = t;
	}
	return 0xffff;
}
static void sprite_draw_write(void)
{
	uint32_t *s = (uint32_t *)b.spriteram, *d = (uint32_t *)b.spritebuf;
	for (int i = 0; i < 0x1000 / 4; i++) {
		const uint32_t t = s[i];
		s[i] = d[i];
		d[i] = t;
	}
	b.spriteram[0] = 0xffff;
}

// ---- sound timing (the Z80 kept level with the main CPU when it writes the latch) ----
static unsigned long s_zt0, s_zt1;
static uint64_t s_line_mt;
static void z_run(unsigned long end);
static void sound_catchup(void)
{
	if (b.sound_reset)
		return;
	const uint64_t el = main_now() - s_line_mt;
	unsigned long t = s_zt0 + (unsigned long)(el * Z80_CLOCK / CPU_CLOCK);
	if ((long)(t - s_zt1) > 0)
		t = s_zt1;
	if ((long)(t - s_z80.cyc) > 0)
		z_run(t);
}

// ---- the 315-5195 memory mapper ----
enum { E_MEM, E_IO, E_ROADCTL };
typedef struct {
	uint32_t start, end, mirror, wbase, wend;
	int kind, rd, wr;                           // rd / wr: 1 handled, 0 falls through, -1 (writes) dropped
	uint8_t *mem;
	uint32_t memsize;
} mapent_t;
static mapent_t s_map[24];
static int s_nmap, s_curregion;

static void map_add(uint32_t offset, uint32_t length, uint32_t mirror, int kind, uint8_t *mem, uint32_t memsize, int rd, int wr)
{
	static const uint32_t region_size_map[4] = { 0x00ffff, 0x01ffff, 0x07ffff, 0x1fffff };
	const uint32_t size_mask = region_size_map[b.regs[0x10 + 2 * s_curregion] & 3];
	const uint32_t base = ((uint32_t)b.regs[0x11 + 2 * s_curregion] << 16) & ~size_mask;
	if (s_nmap >= (int)(sizeof(s_map) / sizeof(s_map[0])))
		return;
	mapent_t *e = &s_map[s_nmap++];
	e->mirror = mirror & size_mask;
	e->start = base + (offset & size_mask);
	e->end = e->start + (length - 1 < size_mask ? length - 1 : size_mask);
	e->wbase = base;
	e->wend = base + size_mask;
	e->kind = kind;
	e->mem = mem;
	e->memsize = memsize;
	e->rd = rd;
	e->wr = wr;
}
// segaorun_state::memory_mapper
static void memory_mapper(int index)
{
	switch (index) {
	case 5:
		map_add(0x90000, 0x10000, 0xf00000, E_ROADCTL, NULL, 0, 1, 1);
		map_add(0x80000, 0x01000, 0xf0f000, E_MEM, (uint8_t *)b.roadram, 0x1000, 1, 1);
		map_add(0x60000, 0x08000, 0xf18000, E_MEM, (uint8_t *)b.subram, 0x8000, 1, 1);
		map_add(0x00000, 0x60000, 0xf00000, E_MEM, s_subrom, s_subrom_size < 0x60000 ? s_subrom_size : 0x60000, 1, -1);
		break;
	case 4: map_add(0x90000, 0x10000, 0xf00000, E_IO, NULL, 0, 1, 1); break;
	case 3: map_add(0x00000, 0x01000, 0xfff000, E_MEM, (uint8_t *)b.spriteram, 0x1000, 1, 1); break;
	case 2: map_add(0x00000, 0x02000, 0xffe000, E_MEM, (uint8_t *)b.paletteram, 0x2000, 1, 1); break;
	case 1:
		map_add(0x00000, 0x10000, 0xfe0000, E_MEM, (uint8_t *)b.tileram, 0x10000, 1, 1);
		map_add(0x10000, 0x01000, 0xfef000, E_MEM, (uint8_t *)b.textram, 0x1000, 1, 1);
		break;
	case 0:
		map_add(0x60000, 0x08000, 0xf98000, E_MEM, (uint8_t *)b.workram, 0x8000, 1, 1);
		map_add(0x00000, 0x60000, 0xf80000, E_MEM, s_rom, s_rom_size < 0x60000 ? s_rom_size : 0x60000, 1, -1);
		break;
	}
}

// the page table: a page goes straight to memory when one entry covers all of it with no later entry in its window
static void update_mapping(void)
{
	s_nmap = 0;
	for (int index = 7; index >= 0; index--) {
		s_curregion = index;
		memory_mapper(index);
	}
	const uint32_t ps = 1u << M68K_PAGE_BITS;
	for (uint32_t p = 0; p < M68K_PAGES; p++) {
		const uint32_t a0 = p * ps, a1 = a0 + ps - 1;
		m68k_bus.rd[p] = m68k_bus.wr[p] = NULL;
		for (int i = s_nmap - 1; i >= 0; i--) {
			const mapent_t *e = &s_map[i];
			if (a1 < e->wbase || a0 > e->wend)
				continue;
			if (e->kind == E_MEM && !(e->mirror & (ps - 1))) {
				const uint32_t ae = a0 & ~e->mirror;
				if (ae >= e->start && ae + ps - 1 <= e->end && ae + ps - 1 - e->start < e->memsize) {
					m68k_bus.rd[p] = e->mem + (ae - e->start);
					if (e->wr > 0)
						m68k_bus.wr[p] = e->mem + (ae - e->start);
				}
			}
			break;
		}
	}
}

static uint16_t io_r(uint32_t offset);
static void io_w(uint32_t offset, uint16_t data, uint16_t mask);
static void mapper_w(int offset, uint8_t data);
static uint8_t mapper_r(int offset);

static uint16_t main_r16(uint32_t a)
{
	a &= 0xfffffe;
	for (int i = s_nmap - 1; i >= 0; i--) {
		const mapent_t *e = &s_map[i];
		const uint32_t ae = a & ~e->mirror;
		if (ae < e->start || ae > e->end || e->rd <= 0)
			continue;
		const uint32_t o = ae - e->start;
		switch (e->kind) {
		case E_MEM: return o + 1 < e->memsize ? *(const uint16_t *)(e->mem + o) : 0xffff;
		case E_IO: return io_r(o >> 1);
		case E_ROADCTL: return road_control_r();
		}
	}
	return (uint16_t)(0xff00 | mapper_r((int)((a >> 1) & 0x1f)));
}
static uint8_t main_r8(uint32_t a)
{
	const uint16_t w = main_r16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}
static void main_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xfffffe;
	for (int i = s_nmap - 1; i >= 0; i--) {
		const mapent_t *e = &s_map[i];
		const uint32_t ae = a & ~e->mirror;
		if (ae < e->start || ae > e->end || e->wr == 0)
			continue;
		if (e->wr < 0)
			return;
		const uint32_t o = ae - e->start;
		switch (e->kind) {
		case E_MEM:
			if (o + 1 < e->memsize) {
				uint16_t *p = (uint16_t *)(e->mem + o);
				*p = (uint16_t)((v & mask) | (*p & ~mask));
			}
			return;
		case E_IO: io_w(o >> 1, v, mask); return;
		case E_ROADCTL:
			if (mask & 0xff)
				b.road_control = v & 3;
			return;
		}
	}
	if (mask & 0xff)
		mapper_w((int)((a >> 1) & 0x1f), (uint8_t)v);
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v)
{
	main_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00);
}

static int s_open_bus_recurse;
static uint16_t open_bus(void)   // the word at the PC
{
	if (s_open_bus_recurse)
		return 0xffff;
	s_open_bus_recurse = 1;
	const uint16_t r = main_r16(m68k_get_reg(NULL, M68K_REG_PC));
	s_open_bus_recurse = 0;
	return r;
}

static void mapper_w(int offset, uint8_t data)
{
	offset &= 0x1f;
	const uint8_t oldval = b.regs[offset];
	b.regs[offset] = data;
	switch (offset) {
	case 0x02:   // the 68000's reset line
		if ((oldval ^ data) & 3) {
			const int held = (data & 3) == 3;
			if (held && !b.reset_held) {
				if (s_fd1094_on) {
					fd1094_change_state(&s_fd1094, FD1094_STATE_RESET);
					fd_update();
				}
				m68k_pulse_reset();
			}
			b.reset_held = (uint8_t)held;
		}
		break;
	case 0x03:   // to the sound CPU (its NMI)
		if (s_cur == 0)
			sound_catchup();
		b.to_sound = data;
		if (!b.pbf)
			z80_gen_nmi(&s_z80);
		b.pbf = 1;
		break;
	case 0x04:   // an IRQ to the 68000
		if ((data & 7) != 7)
			m68k_bus_hold_irq(~data & 7);
		break;
	case 0x05:
		if (data == 0x01) {
			const uint32_t addr = ((uint32_t)b.regs[0x0a] << 17) | ((uint32_t)b.regs[0x0b] << 9) | ((uint32_t)b.regs[0x0c] << 1);
			main_w16(addr, (uint16_t)(b.regs[0x00] << 8 | b.regs[0x01]));
		} else if (data == 0x02) {
			const uint32_t addr = ((uint32_t)b.regs[0x07] << 17) | ((uint32_t)b.regs[0x08] << 9) | ((uint32_t)b.regs[0x09] << 1);
			const uint16_t result = main_r16(addr);
			b.regs[0x00] = (uint8_t)(result >> 8);
			b.regs[0x01] = (uint8_t)result;
		}
		break;
	default:
		if (offset >= 0x10 && oldval != data)
			update_mapping();
		break;
	}
}
static uint8_t mapper_r(int offset)
{
	offset &= 0x1f;
	switch (offset) {
	case 0x00: case 0x01: return b.regs[offset];
	case 0x02: return (b.regs[0x02] & 3) == 3 ? 0x00 : 0x0f;
	case 0x03: return 0;   // (from the sound CPU: not wired)
	}
	return (uint8_t)open_bus();
}

// shangon_custom_io_r / _w
static uint16_t shangon_io_r(uint32_t offset)
{
	offset &= 0x303f / 2;
	if (offset >= 0x1000 / 2 && offset <= 0x1006 / 2) {
		const int p = s_port_dig[offset & 3];
		return (uint16_t)(0xff00 | (p >= 0 ? rbx_port_read(p) & 0xff : 0xff));
	}
	if (offset == 0x3020 / 2)
		return (uint16_t)(0xff00 | adc_r());
	return open_bus();
}
static void shangon_io_w(uint32_t offset, uint16_t data, uint16_t mask)
{
	offset &= 0x303f / 2;
	if (offset == 0x0000 / 2 && (mask & 0xff)) {
		b.adc_select = (data >> 6) & 3;
		b.display_enable = (data & 0x20) != 0;
	} else if (offset == 0x0020 / 2 && (mask & 0xff)) {
		const uint8_t reset = !(data & 1);
		if (reset && !b.sound_reset) {
			const unsigned long cyc = s_z80.cyc;
			z80_init(&s_z80);
			z80_bind();
			s_z80.cyc = cyc;
		}
		b.sound_reset = reset;
	} else if (offset == 0x3020 / 2)
		adc_w();
}

// outrun_custom_io_r / _w
static uint16_t io_r(uint32_t offset)
{
	if (s_shangon)
		return shangon_io_r(offset);
	offset &= 0x7f / 2;
	switch (offset & (0x70 / 2)) {
	case 0x00 / 2: return (uint16_t)(0xff00 | ppi_r((int)offset));
	case 0x10 / 2: {
		const int p = s_port_dig[offset & 3];
		return (uint16_t)(0xff00 | (p >= 0 ? rbx_port_read(p) & 0xff : 0xff));
	}
	case 0x30 / 2: return (uint16_t)(0xff00 | adc_r());
	case 0x60 / 2: return 0xffff;   // (the watchdog)
	}
	return open_bus();
}
static void io_w(uint32_t offset, uint16_t data, uint16_t mask)
{
	if (s_shangon) {
		shangon_io_w(offset, data, mask);
		return;
	}
	offset &= 0x7f / 2;
	switch (offset & (0x70 / 2)) {
	case 0x00 / 2:
		if (mask & 0xff)
			ppi_w((int)offset, (uint8_t)data);
		return;
	case 0x20 / 2:
		if (mask & 0xff)
			b.mute = !(data & 0x80);
		return;
	case 0x30 / 2: adc_w(); return;
	case 0x70 / 2: sprite_draw_write(); return;
	}
}

// ---- sub CPU bus ----
static uint16_t sub_r16(uint32_t a)
{
	a &= 0xfffff;
	if (a >= 0x90000 && a < 0xa0000)
		return road_control_r();
	return 0xffff;
}
static uint8_t sub_r8(uint32_t a)
{
	const uint16_t v = sub_r16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static void sub_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xfffff;
	if (a >= 0x90000 && a < 0xa0000 && (mask & 0xff))
		b.road_control = v & 3;
}
static void sub_w16(uint32_t a, uint16_t v) { sub_w16m(a, v, 0xffff); }
static void sub_w8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	sub_w16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- sound ----
static uint8_t z_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xf000) return a < s_zrom_size ? s_zrom[a] : 0xff;
	if (a < 0xf800) return segapcm_r(&s_pcm, (uint8_t)a);
	return s_zram[a & 0x7ff];
}
static void z_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xf800) s_zram[a & 0x7ff] = v;
	else if (a >= 0xf000) segapcm_w(&s_pcm, (uint8_t)a, v);
}
static uint8_t z_in(z80 *z, uint8_t p)
{
	(void)z;
	if ((p & 0xc0) == 0x00)
		return rbx_ym_read(0, p & 1);
	if ((p & 0xc0) == 0x40) {   // the mapper's pread: the PBF line cleared
		b.pbf = 0;
		return b.to_sound;
	}
	return 0xff;
}
static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if ((p & 0xc0) == 0x00)
		rbx_ym_write(0, p & 1, v);
}
static void ym_irq(int state) { s_zirq = state; }
static void z80_bind(void)
{
	s_z80.read_byte = z_read;
	s_z80.write_byte = z_write;
	s_z80.port_in = z_in;
	s_z80.port_out = z_out;
	s_z80.userdata = NULL;
	s_z80.read_map = s_zmap;
}

static void sound_line_begin(void)
{
	const int zc = rbx_line_cycles(&b.z80_frac, Z80_CLOCK, FRAME_MHZ, LINES);
	s_zt0 = s_z80.cyc;
	s_zt1 = s_zt0 + (unsigned long)zc;
	s_line_mt = b.main_time;
}
static int z_port_fixed(const z80 *z, uint8_t p) { (void)z; return (p & 0xc0) != 0x40; }
static void z_run(unsigned long end)
{
	if (s_spin_on)
		sega_zspin(&s_z80, end, &s_zirq, z_port_fixed);
	z80_run(&s_z80, end, &s_zirq, 0);
}

static void sound_line(void)
{
	if (!b.sound_reset && (long)(s_zt1 - s_z80.cyc) > 0)
		z_run(s_zt1);
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&b.ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
	const int n = rbx_ym_advance_lr(0, clocks, s_mix_l + s_mix_n, s_mix_r + s_mix_n, s_mix_cap - s_mix_n);
	for (int k = 0; k < n; k++) {   // (the PCM at half the YM's rate: 4 MHz / 128 against 4 MHz / 64)
		if (!(b.pcm_phase++ & 1)) {
			int32_t l, r;
			segapcm_tick(&s_pcm, &l, &r);
			b.pcm_hl = l * 179 / 256;   // 0.70
			b.pcm_hr = r * 179 / 256;
		}
		s_mix_l[s_mix_n + k] += b.pcm_hl;
		s_mix_r[s_mix_n + k] += b.pcm_hr;
	}
	s_mix_n += n;
}

static void sound_frame_end(void)
{
	rbx_ym_async_take_lr(0, s_mix_l, s_mix_r, s_mix_n);
	if (b.mute)
		for (int i = 0; i < s_mix_n; i++)
			rbx_audio_put_stereo(0, 0);
	else
		for (int i = 0; i < s_mix_n; i++)
			rbx_audio_put_stereo(s_mix_l[i], s_mix_r[i]);
	memset(s_mix_l, 0, sizeof(int32_t) * (size_t)s_mix_n);
	memset(s_mix_r, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

// ---- video ----
static struct {
	uint16_t paletteram[0x1000];
	uint16_t tileram[0x8000];
	uint16_t textram[0x800];
	uint16_t spritebuf[0x800];
	uint16_t roadbuf[0x800];
	uint16_t latched_page[4], latched_yscroll[4], latched_xscroll[4];
	uint8_t display_enable, road_control;
	int pal_dirty;
} *s_snap;
static int s_tcol_shift = 6, s_text_mask = 0x1ff, s_text_shift = 9;   // (TILEMAP_16B_ALT: 5, 0xff, 8)
static uint16_t *s_pal_conv;
static int s_pal_valid;
static uint8_t s_pal_normal[32], s_pal_shadow[32], s_pal_hilight[32];
static uint8_t *s_roadgfx;   // (256 x 2 + 1) rows of 512

static void palette_init(void)   // sega_16bit_common_base::palette_init (compute_resistor_weights, 6 resistors)
{
	static const double rn[6] = { 3900, 2000, 1000, 1000.0 / 2, 1000.0 / 4, 0 };
	static const double rs[6] = { 3900, 2000, 1000, 1000.0 / 2, 1000.0 / 4, 470 };
	double wn[6], ws[6];
	for (int set = 0; set < 2; set++) {
		const double *r = set ? rs : rn;
		double *w = set ? ws : wn, max = 0;
		for (int n = 0; n < 6; n++) {
			double r0 = 1.0 / 1e12, r1 = 1.0 / 1e12;
			for (int j = 0; j < 6; j++) {
				if (r[j] == 0)
					continue;
				if (j == n)
					r1 += 1.0 / r[j];
				else
					r0 += 1.0 / r[j];
			}
			r0 = 1.0 / r0;
			r1 = 1.0 / r1;
			double v = 255.0 * r0 / (r1 + r0);
			if (v > 255) v = 255;
			w[n] = r[n] != 0 ? v : 0;
			max += w[n];
		}
		const double scale = 255.0 / max;
		for (int n = 0; n < 6; n++)
			w[n] *= scale;
	}
	for (int v = 0; v < 32; v++) {
		double n = 0, s = 0;
		for (int bit = 0; bit < 5; bit++)
			if ((v >> bit) & 1) {
				n += wn[bit];
				s += ws[bit];
			}
		s_pal_normal[v] = (uint8_t)(int)(n + 0.5);
		s_pal_shadow[v] = (uint8_t)(int)(s + 0.5);
		s_pal_hilight[v] = (uint8_t)(int)(s + ws[5] + 0.5);
	}
}

static uint8_t *s_tdec;
static uint8_t *s_tmask;   // each decoded tile row's pens other than 0 (bit n: pixel n)
static uint32_t s_tdec_n;
static void decode_tiles(void)   // gfx_8x8x3_planar
{
	const uint32_t third = s_tiles_size / 3;
	s_tdec_n = third / 8;
	s_tdec = (uint8_t *)malloc((size_t)s_tdec_n * 64 + 64);
	if (!s_tdec)
		return;
	for (uint32_t c = 0; c < s_tdec_n; c++)
		for (int py = 0; py < 8; py++) {
			const uint32_t o = c * 8 + (uint32_t)py;
			const uint8_t p0 = s_tiles[o], p1 = s_tiles[third + o], p2 = s_tiles[2 * third + o];
			for (int px = 0; px < 8; px++) {
				const int bit = 7 - px;
				s_tdec[c * 64 + (uint32_t)py * 8 + (uint32_t)px] =
					(uint8_t)(((p2 >> bit) & 1) << 2 | ((p1 >> bit) & 1) << 1 | ((p0 >> bit) & 1));
			}
		}
	memset(s_tdec + (size_t)s_tdec_n * 64, 0, 64);
	s_tmask = (uint8_t *)malloc((size_t)s_tdec_n * 8 + 8);
	if (!s_tmask)
		return;
	for (uint32_t r = 0; r < s_tdec_n * 8 + 8; r++) {
		uint8_t m = 0;
		for (int px = 0; px < 8; px++)
			m |= (uint8_t)((s_tdec[r * 8 + (uint32_t)px] != 0) << px);
		s_tmask[r] = m;
	}
}
static inline uint32_t tile_ri(uint32_t code, int py) { return (code < s_tdec_n ? code : s_tdec_n) * 8 + (uint32_t)py; }

static void road_decode(void)   // segaic16_road_outrun_decode
{
	const uint32_t len = s_road_size;
	for (int y = 0; y < 256 * 2; y++) {
		const uint8_t *src = s_road + (((uint32_t)(y & 0xff) * 0x40 + (uint32_t)(y >> 8) * 0x8000) % len);
		uint8_t *dst = s_roadgfx + y * 512;
		for (int x = 0; x < 512; x++) {
			dst[x] = (uint8_t)((((src[x >> 3] >> (~x & 7)) & 1) << 0) | (((src[(x >> 3) + 0x4000] >> (~x & 7)) & 1) << 1));
			if (x >= 256 - 8 && x < 256 && dst[x] == 3)
				dst[x] |= 4;
		}
	}
	memset(s_roadgfx + 256 * 2 * 512, 3, 512);
}

// one layer's line of the 16B tilemaps (tilemap_16b_draw_layer), colour base 0: pri |= hi for the tiles of category 1,
// lo for the others
static void layer_line(int which, int y, uint16_t *d, uint8_t *pri, uint8_t lo, uint8_t hi)
{
	const uint16_t *textram = s_snap->textram;
	const uint16_t xscroll = s_snap->latched_xscroll[which], yscroll = s_snap->latched_yscroll[which];
	const uint16_t rowscroll = textram[0xf80 / 2 + 0x40 / 2 * which + y / 8];
	uint16_t pages = s_snap->latched_page[which];
	uint16_t effx = (xscroll & 0x8000) ? rowscroll : xscroll, effy = yscroll;
	const int alt = (rowscroll & 0x8000) != 0;
	if (alt) {
		effx = s_snap->latched_xscroll[which + 2];
		effy = s_snap->latched_yscroll[which + 2];
		pages = s_snap->latched_page[which + 2];
	}
	effx = (uint16_t)((0xc0 - effx) & 0x3ff);
	const int colscroll = (yscroll & 0x8000) && !alt;
	for (int x = 0; x < VIS_W;) {
		int ey = effy, n = VIS_W - x;
		if (colscroll) {
			ey = textram[0xf16 / 2 + 0x40 / 2 * which + (x + 8) / 16];
			const int next = ((x + 8) / 16) * 16 + 8;
			n = (next > x ? next : next + 16) - x;
			if (n > VIS_W - x)
				n = VIS_W - x;
		}
		const int vy = (y + (ey & 0x1ff)) & 0x1ff, py = vy & 0xff;
		const int pbase = ((vy >> 8) & 1) * 8;
		const int end = x + n;
		while (x < end) {
			const int vx = (x + effx) & 0x3ff, px = vx & 0x1ff;
			const int page = (pages >> (pbase + ((vx >> 9) & 1) * 4)) & 15;
			const uint16_t data = s_snap->tileram[page * 64 * 32 + (py >> 3) * 64 + (px >> 3)];
			const int sub = px & 7;
			int k = 8 - sub;
			if (k > end - x)
				k = end - x;
			const uint32_t ri = tile_ri((uint32_t)(data & 0x1fff), py & 7);
			const unsigned full = (1u << k) - 1, m = ((unsigned)s_tmask[ri] >> sub) & full;
			if (m) {
				const uint8_t *row = s_tdec + ri * 8 + (uint32_t)sub;
				const uint16_t cb = (uint16_t)(((data >> s_tcol_shift) & 0x7f) * 8);
				const uint8_t pv = (data & 0x8000) ? hi : lo;
				uint8_t *pp = pri + x;
				if (!d) {
					for (int i = 0; i < k; i++)
						if (row[i])
							pp[i] |= pv;
					x += k;
					continue;
				}
				uint16_t *dd = d + x;
				if (m == full)
					for (int i = 0; i < k; i++) {
						dd[i] = (uint16_t)(cb + row[i]);
						pp[i] |= pv;
					}
				else
					for (int i = 0; i < k; i++)
						if (row[i]) {
							dd[i] = (uint16_t)(cb + row[i]);
							pp[i] |= pv;
						}
			}
			x += k;
		}
	}
}

// segaic16_road_outrun_draw (ROAD_OUTRUN: colour bases 0x400, 0x420, 0x780, xoffs 0)
static int ROAD_CB1 = 0x400, ROAD_CB2 = 0x420, ROAD_CB3 = 0x780;   // (Super Hang-On: 0x7f6, 0x7c0, 0x7c0)
static void road_line(int y, uint16_t *dest, int fg)
{
	static const uint8_t priority_map[2][8] = {
		{ 0x80, 0x81, 0x81, 0x87, 0, 0, 0, 0x00 },
		{ 0x81, 0x81, 0x81, 0x8f, 0, 0, 0, 0x80 }
	};
	const uint16_t *roadram = s_snap->roadbuf;
	const uint8_t ctl = s_snap->road_control;
	const uint16_t data0 = roadram[0x000 + y], data1 = roadram[0x100 + y];
	if (!fg) {
		int color = -1;
		switch (ctl & 3) {
		case 0: if (data0 & 0x800) color = data0 & 0x7f; break;
		case 1:
			if (data0 & 0x800) color = data0 & 0x7f;
			else if (data1 & 0x800) color = data1 & 0x7f;
			break;
		case 2:
			if (data1 & 0x800) color = data1 & 0x7f;
			else if (data0 & 0x800) color = data0 & 0x7f;
			break;
		case 3: if (data1 & 0x800) color = data1 & 0x7f; break;
		}
		if (color != -1)
			for (int x = 0; x < VIS_W; x++)
				dest[x] = (uint16_t)(color | ROAD_CB3);
		return;
	}
	if ((data0 & 0x800) && (data1 & 0x800))
		return;
	const uint8_t *src0 = (data0 & 0x800) ? s_roadgfx + 256 * 2 * 512 : s_roadgfx + (0x000 + ((data0 >> 1) & 0xff)) * 512;
	int hpos0 = roadram[0x200 + (data0 & 0x1ff)] & 0xfff;
	const int color0 = roadram[0x600 + (data0 & 0x1ff)];
	const uint8_t *src1 = (data1 & 0x800) ? s_roadgfx + 256 * 2 * 512 : s_roadgfx + (0x100 + ((data1 >> 1) & 0xff)) * 512;
	int hpos1 = roadram[0x400 + (data1 & 0x1ff)] & 0xfff;
	const int color1 = roadram[0x600 + (data1 & 0x1ff)];
	uint16_t ct[32];
	ct[0x00] = (uint16_t)(ROAD_CB1 ^ 0x00 ^ ((color0 >> 0) & 1));
	ct[0x01] = (uint16_t)(ROAD_CB1 ^ 0x02 ^ ((color0 >> 1) & 1));
	ct[0x02] = (uint16_t)(ROAD_CB1 ^ 0x04 ^ ((color0 >> 2) & 1));
	ct[0x03] = (data0 & 0x200) ? ct[0x00] : (uint16_t)(ROAD_CB2 ^ 0x00 ^ ((color0 >> 8) & 0xf));
	ct[0x07] = (uint16_t)(ROAD_CB1 ^ 0x06 ^ ((color0 >> 3) & 1));
	ct[0x10] = (uint16_t)(ROAD_CB1 ^ 0x08 ^ ((color1 >> 4) & 1));
	ct[0x11] = (uint16_t)(ROAD_CB1 ^ 0x0a ^ ((color1 >> 5) & 1));
	ct[0x12] = (uint16_t)(ROAD_CB1 ^ 0x0c ^ ((color1 >> 6) & 1));
	ct[0x13] = (data1 & 0x200) ? ct[0x10] : (uint16_t)(ROAD_CB2 ^ 0x10 ^ ((color1 >> 8) & 0xf));
	ct[0x17] = (uint16_t)(ROAD_CB1 ^ 0x0e ^ ((color1 >> 7) & 1));
	const int xo = 0x5f8;
	const uint8_t *r3 = s_roadgfx + 256 * 2 * 512;
	switch (ctl & 3) {
	case 0: case 3: {
		if ((ctl & 3) == 0 ? (data0 & 0x800) : (data1 & 0x800))
			return;
		const uint8_t *src = (ctl & 3) == 0 ? src0 : src1;
		const uint16_t *c = (ctl & 3) == 0 ? ct : ct + 0x10;
		int h = (((ctl & 3) == 0 ? hpos0 : hpos1) - xo) & 0xfff;
		for (int x = 0; x < VIS_W;) {
			int n = (h < 0x200 ? 0x200 : 0x1000) - h;
			if (n > VIS_W - x)
				n = VIS_W - x;
			if (h < 0x200) {
				const uint8_t *s = src + h;
				for (int i = 0; i < n; i++)
					dest[x + i] = c[s[i]];
			} else {
				const uint16_t v = c[3];
				for (int i = 0; i < n; i++)
					dest[x + i] = v;
			}
			x += n;
			h = (h + n) & 0xfff;
		}
		break;
	}
	case 1: case 2: {
		static const uint8_t pens[5] = { 0, 1, 2, 3, 7 };
		const uint8_t *pm = priority_map[(ctl & 3) - 1];
		uint16_t tab[64];   // (pen0 * 8 + pen1)
		for (int i = 0; i < 5; i++)
			for (int j = 0; j < 5; j++) {
				const int p0 = pens[i], p1 = pens[j];
				tab[p0 * 8 + p1] = ((pm[p0] >> p1) & 1) ? ct[0x10 + p1] : ct[0x00 + p0];
			}
		hpos0 = (hpos0 - xo) & 0xfff;
		hpos1 = (hpos1 - xo) & 0xfff;
		for (int x = 0; x < VIS_W;) {
			int n = (hpos0 < 0x200 ? 0x200 : 0x1000) - hpos0;
			const int n1 = (hpos1 < 0x200 ? 0x200 : 0x1000) - hpos1;
			if (n > n1)
				n = n1;
			if (n > VIS_W - x)
				n = VIS_W - x;
			const uint8_t *s0 = hpos0 < 0x200 ? src0 + hpos0 : r3, *s1 = hpos1 < 0x200 ? src1 + hpos1 : r3;
			for (int i = 0; i < n; i++)
				dest[x + i] = tab[s0[i] * 8 + s1[i]];
			x += n;
			hpos0 = (hpos0 + n) & 0xfff;
			hpos1 = (hpos1 + n) & 0xfff;
		}
		break;
	}
	}
}

static int road_fg_full(int y)
{
	const uint16_t data0 = s_snap->roadbuf[0x000 + y], data1 = s_snap->roadbuf[0x100 + y];
	switch (s_snap->road_control & 3) {
	case 0: return !(data0 & 0x800);
	case 3: return !(data1 & 0x800);
	default: return !((data0 & 0x800) && (data1 & 0x800));
	}
}

// the sprites of a band (sega_outrun_sprite_device::draw; origin x 189), into s_sbuf (0xffff: none; the mix puts it
// back), each row's columns written within [s_slo, s_shi]
#define BANDS 8
static uint16_t (*s_sbuf_hp)[VIS_H][VIS_W];
#define s_sbuf (*s_sbuf_hp)
static int16_t s_slo[VIS_H], s_shi[VIS_H];

static void draw_sprites(int y0, int y1, const uint16_t *list)
{
	const int minx = 189, maxx = 189 + VIS_W - 1;
	const uint32_t *spritebase = (const uint32_t *)s_spr;
	const uint8_t numbanks = (uint8_t)(s_spr_size / 0x40000);
	for (const uint16_t *data = list; data < list + 0x800; data += 8) {
		if (data[0] & 0x8000)
			break;
		const int hide = data[0] & 0x5000;
		int bank = (data[0] >> 9) & 7;
		const int top = (data[0] & 0x1ff) - 0x100;
		uint16_t addr = data[1];
		const int pitch = (int16_t)((data[2] >> 1) | ((data[4] & 0x1000) << 3)) >> 8;
		int xpos = data[2] & 0x1ff;
		int vzoom = data[3] & 0x7ff;
		const int ydelta = (data[4] & 0x8000) ? 1 : -1;
		const int flip = (~data[4] >> 14) & 1;
		const int xdelta = (data[4] & 0x2000) ? 1 : -1;
		int hzoom = data[4] & 0x7ff;
		const int height = (data[5] >> 8) + 1;
		const int colpri = ((data[5] & 0x7f) << 4) | (((data[3] >> 12) & 7) << 12);
		if (xpos < 0x80 && xdelta < 0)
			xpos += 0x200;
		if (hide)
			continue;
		if (ydelta > 0 ? (top >= y1 || top + height <= y0) : (top < y0 || top - height >= y1 - 1))
			continue;
		if (numbanks)
			bank %= numbanks;
		const uint32_t *spritedata = spritebase + 0x10000 * bank;
		if (vzoom < 0x40) vzoom = 0x40;
		if (hzoom < 0x40) hzoom = 0x40;
		int yacc = 0;
		const int ytarget = top + ydelta * height;
		for (int y = top; y != ytarget; y += ydelta) {
			if (ydelta > 0 ? y >= y1 : y < y0)
				break;
			if (y >= y0 && y < y1) {
				uint16_t *dm = s_sbuf[y] - minx;   // (indexed by x)
				int xacc = 0, x = xpos, sk = 0;
				uint16_t a7 = addr;
				while (xdelta > 0 ? x <= maxx : x >= minx) {
					uint32_t pixels = spritedata[a7];
					if (flip)
						a7--;
					else {
						a7++;
						pixels = ((pixels << 28) & 0xf0000000) | ((pixels << 20) & 0x0f000000) |
						         ((pixels << 12) & 0x00f00000) | ((pixels << 4) & 0x000f0000) |
						         ((pixels >> 4) & 0x0000f000) | ((pixels >> 12) & 0x00000f00) |
						         ((pixels >> 20) & 0x000000f0) | ((pixels >> 28) & 0x0000000f);
					}
					const int last = (pixels & 0x0f000000) == 0x0f000000;
					if (hzoom == 0x200 && (unsigned)(x - minx) < VIS_W && (unsigned)(x + 7 * xdelta - minx) < VIS_W) {
						for (int k = 0; k < 8; k++, x += xdelta) {
							const int pix = pixels & 0xf;
							if ((unsigned)(pix - 1) < 14)
								dm[x] = (uint16_t)(colpri | pix);
							pixels >>= 4;
						}
						if (last)
							break;
						continue;
					}
					if (hzoom >= 0x200) {
						int k = sk;
						for (; k < 8; ) {
							const int pix = (int)(pixels >> (4 * k)) & 0xf;
							if ((unsigned)(pix - 1) < 14 && (unsigned)(x - minx) < VIS_W)
								dm[x] = (uint16_t)(colpri | pix);
							x += xdelta;
							xacc += hzoom - 0x200;
							k += 1 + (xacc >> 9);
							xacc &= 0x1ff;
						}
						sk = k - 8;
						if (xdelta > 0 ? x > maxx : x < minx)
							goto row_done;
						if (last)
							break;
						continue;
					}
					for (int k = 0; k < 8; k++) {
						const int pix = pixels & 0xf;
						int n = 0;
						while (xacc < 0x200) {
							n++;
							xacc += hzoom;
						}
						xacc -= 0x200;
						pixels >>= 4;
						if (xdelta > 0) {
							if (n && pix != 0 && pix != 15) {
								int i = x < minx ? minx : x, e = x + n - 1;
								if (e > maxx)
									e = maxx;
								const uint16_t v = (uint16_t)(colpri | pix);
								for (; i <= e; i++)
									dm[i] = v;
							}
							x += n;
							if (x > maxx)
								goto row_done;
						} else {
							if (n && pix != 0 && pix != 15) {
								int i = x - n + 1, e = x > maxx ? maxx : x;
								if (i < minx)
									i = minx;
								const uint16_t v = (uint16_t)(colpri | pix);
								for (; i <= e; i++)
									dm[i] = v;
							}
							x -= n;
							if (x < minx)
								goto row_done;
						}
					}
					if (last)
						break;
				}
			row_done:;
				int lo = xdelta > 0 ? xpos : x, hi = xdelta > 0 ? x : xpos;
				if (lo < minx) lo = minx;
				if (hi > maxx) hi = maxx;
				if (lo <= hi) {
					if (lo - minx < s_slo[y]) s_slo[y] = (int16_t)(lo - minx);
					if (hi - minx > s_shi[y]) s_shi[y] = (int16_t)(hi - minx);
				}
			}
			yacc += vzoom;
			addr = (uint16_t)(addr + pitch * (yacc >> 9));
			yacc &= 0x1ff;
		}
	}
}

// screen_update_outrun
static void orun_band(int band)
{
	const int y0 = VIS_H * band / BANDS, y1 = VIS_H * (band + 1) / BANDS;
	if (!s_snap->display_enable) {
		for (int y = y0; y < y1; y++)
			for (int x = 0; x < VIS_W; x++)
				rbx_video.bitmap[y * rbx_video.pitch + x] = PAL_BLACK;
		return;
	}
	draw_sprites(y0, y1, s_snap->spritebuf);
	uint8_t pri[VIS_W];
	for (int y = y0; y < y1; y++) {
		uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
		memset(pri, 0, sizeof(pri));
		// (a foreground road line covers the whole line: the background road under it isn't seen, nor the layers'
		// colours, only their priorities)
		const int full = road_fg_full(y);
		if (!full)
			road_line(y, d, 0);
		uint16_t *ld = full ? NULL : d;
		layer_line(1, y, ld, pri, 0x01, 0x02);
		layer_line(0, y, ld, pri, 0x02, 0x04);
		if (full)
			road_line(y, d, 1);
		// the text (tilemap columns 24-63, aligned to the tiles)
		const uint16_t *text = &s_snap->textram[(y >> 3) * 64 + 24];
		for (int t = 0; t < VIS_W / 8; t++) {
			const uint16_t data = text[t];
			const uint32_t ri = tile_ri((uint32_t)(data & 0x1ff), y & 7);
			const unsigned m = s_tmask[ri];
			if (!m)
				continue;
			const uint8_t *row = s_tdec + ri * 8;
			const uint16_t cb = (uint16_t)(((data >> 9) & 7) * 8);
			const uint8_t pv = (data & 0x8000) ? 0x08 : 0x04;
			uint16_t *dd = d + t * 8;
			uint8_t *pp = pri + t * 8;
			if (m == 0xff)
				for (int i = 0; i < 8; i++) {
					dd[i] = (uint16_t)(cb + row[i]);
					pp[i] |= pv;
				}
			else
				for (int i = 0; i < 8; i++)
					if (row[i]) {
						dd[i] = (uint16_t)(cb + row[i]);
						pp[i] |= pv;
					}
		}
		// the sprites (and the row put back to none)
		const int lo = s_slo[y], hi = s_shi[y];
		if (lo <= hi) {
			uint16_t *sp = s_sbuf[y];
			for (int x = lo; x <= hi; x++) {
				const uint16_t pix = sp[x];
				if (pix == 0xffff)
					continue;
				sp[x] = 0xffff;
				if ((1 << ((pix >> 12) & 3)) > pri[x]) {
					if ((pix & 0x400f) == 0x400a)
						d[x] = (uint16_t)(d[x] + NPAL);
					else
						d[x] = (uint16_t)(0x800 | (pix & 0x7ff));
				}
			}
			s_slo[y] = VIS_W;
			s_shi[y] = -1;
		}
	}
}
// the System 16B sprites of a band (sega_sys16b_sprite_device::draw, Super Hang-On: origin x 184), into s_sbuf
static void draw_sprites_16b(int y0, int y1, const uint16_t *ram)
{
	const uint16_t *spritebase = (const uint16_t *)s_spr;
	const uint8_t numbanks = (uint8_t)(s_spr_size / 0x20000);
	for (const uint16_t *data = ram; data < ram + 0x800; data += 8) {
		if (data[2] & 0x8000)
			break;
		const int bottom = data[0] >> 8, top = data[0] & 0xff;
		const int xpos = data[1] & 0x1ff;
		const int hide = data[2] & 0x4000, flip = data[2] & 0x100;
		const int pitch = (int8_t)(data[2] & 0xff);
		uint16_t addr = data[3];
		int bank = (data[4] >> 8) & 0xf;
		const int colpri = ((data[4] & 0xff) << 4) | (((data[1] >> 9) & 0xf) << 12);
		const int vzoom = (data[5] >> 5) & 0x1f, hzoom = data[5] & 0x1f;
		if (hide || top >= bottom || top >= y1 || bottom <= y0)
			continue;
		if (numbanks)
			bank %= numbanks;
		const uint16_t *sd = spritebase + 0x10000 * bank;
		uint16_t zacc = (uint16_t)(data[5] & 0x03ff);
		for (int y = top; y < bottom; y++) {
			addr = (uint16_t)(addr + pitch);
			zacc = (uint16_t)(zacc + (vzoom << 10));
			if (zacc & 0x8000) {
				addr = (uint16_t)(addr + pitch);
				zacc &= ~0x8000;
			}
			if (y < y0 || y >= y1 || y >= VIS_H)
				continue;
			uint16_t *dest = s_sbuf[y];
			int xacc = 4 * hzoom, x;
			uint16_t a7;
			if (!flip) {
				a7 = (uint16_t)(addr - 1);
				for (x = xpos; ((xpos - x) & 0x1ff) != 1;) {
					const uint16_t pixels = sd[++a7];
					int pix = 0;
					for (int sh = 12; sh >= 0; sh -= 4) {
						pix = (pixels >> sh) & 15;
						xacc = (xacc & 0x3f) + hzoom;
						if (xacc < 0x40) {
							const int sx = x - 184;
							if (sx >= 0 && sx < VIS_W && pix != 0 && pix != 15)
								dest[sx] = (uint16_t)(colpri | pix);
							x++;
						}
					}
					if (pix == 15)
						break;
				}
			} else {
				a7 = (uint16_t)(addr + 1);
				for (x = xpos; ((xpos - x) & 0x1ff) != 1;) {
					const uint16_t pixels = sd[--a7];
					int pix = 0;
					for (int sh = 0; sh <= 12; sh += 4) {
						pix = (pixels >> sh) & 15;
						xacc = (xacc & 0x3f) + hzoom;
						if (xacc < 0x40) {
							const int sx = x - 184;
							if (sx >= 0 && sx < VIS_W && pix != 0 && pix != 15)
								dest[sx] = (uint16_t)(colpri | pix);
							x++;
						}
					}
					if (pix == 15)
						break;
				}
			}
		}
	}
}

// screen_update_shangon (no display-enable check; the text layer above everything at 0x08)
static void shangon_band(int band)
{
	const int y0 = VIS_H * band / BANDS, y1 = VIS_H * (band + 1) / BANDS;
	draw_sprites_16b(y0, y1, s_snap->spritebuf);
	uint8_t pri[VIS_W];
	for (int y = y0; y < y1; y++) {
		uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
		memset(pri, 0, sizeof(pri));
		const int full = road_fg_full(y);
		if (!full)
			road_line(y, d, 0);
		uint16_t *ld = full ? NULL : d;
		layer_line(1, y, ld, pri, 0x01, 0x02);
		layer_line(0, y, ld, pri, 0x02, 0x04);
		if (full)
			road_line(y, d, 1);
		const uint16_t *text = &s_snap->textram[(y >> 3) * 64 + 24];
		for (int t = 0; t < VIS_W / 8; t++) {
			const uint16_t data = text[t];
			const uint32_t ri = tile_ri((uint32_t)(data & s_text_mask), y & 7);
			if (!s_tmask[ri])
				continue;
			const uint8_t *row = s_tdec + ri * 8;
			const uint16_t cb = (uint16_t)(((data >> s_text_shift) & 7) * 8);
			for (int i = 0; i < 8; i++)
				if (row[i]) {
					d[t * 8 + i] = (uint16_t)(cb + row[i]);
					pri[t * 8 + i] |= 0x08;
				}
		}
		uint16_t *sp = s_sbuf[y];
		for (int x = 0; x < VIS_W; x++) {
			const uint16_t pix = sp[x];
			if (pix == 0xffff)
				continue;
			sp[x] = 0xffff;
			if ((1 << ((pix >> 10) & 3)) > pri[x]) {
				if ((pix & 0x03f0) == 0x03f0)
					d[x] = (uint16_t)(d[x] + NPAL);
				else
					d[x] = (uint16_t)(0x400 | (pix & 0x3ff));
			}
		}
	}
}
static void any_band(int band) { if (s_shangon) shangon_band(band); else orun_band(band); }
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, any_band };
static int orun_help(int helper) { return decoband_help(&s_bands, helper); }

static void orun_render(void)
{
	if (s_snap->pal_dirty || !s_pal_valid) {
		for (int i = 0; i < NPAL; i++) {
			const uint16_t v = s_snap->paletteram[i];
			if (s_pal_valid && s_pal_conv[i] == v)
				continue;
			s_pal_conv[i] = v;
			const int r = ((v >> 12) & 1) | ((v << 1) & 0x1e), g = ((v >> 13) & 1) | ((v >> 3) & 0x1e),
			          bb = ((v >> 14) & 1) | ((v >> 7) & 0x1e);
			rbx_video.palette[i] = (uint32_t)(s_pal_normal[r] << 16 | s_pal_normal[g] << 8 | s_pal_normal[bb]);
			rbx_video.palette[i + NPAL] = (v & 0x8000)
				? (uint32_t)(s_pal_hilight[r] << 16 | s_pal_hilight[g] << 8 | s_pal_hilight[bb])
				: (uint32_t)(s_pal_shadow[r] << 16 | s_pal_shadow[g] << 8 | s_pal_shadow[bb]);
		}
		rbx_video.palette[PAL_BLACK] = 0;
		s_pal_valid = 1;
		s_snap->pal_dirty = 0;
	}
	decoband_run(&s_bands);
}

static void vblank_draw(void)
{
	rbx_video_wait_idle();
	if (memcmp(s_snap->paletteram, b.paletteram, sizeof(b.paletteram))) {
		memcpy(s_snap->paletteram, b.paletteram, sizeof(b.paletteram));
		s_snap->pal_dirty = 1;
	}
	memcpy(s_snap->tileram, b.tileram, sizeof(b.tileram));
	memcpy(s_snap->textram, b.textram, sizeof(b.textram));
	memcpy(s_snap->spritebuf, s_shangon ? b.spriteram : b.spritebuf, sizeof(b.spritebuf));   // (16B sprites: not buffered)
	memcpy(s_snap->roadbuf, b.roadbuf, sizeof(b.roadbuf));
	memcpy(s_snap->latched_page, b.latched_page, sizeof(b.latched_page));
	memcpy(s_snap->latched_yscroll, b.latched_yscroll, sizeof(b.latched_yscroll));
	memcpy(s_snap->latched_xscroll, b.latched_xscroll, sizeof(b.latched_xscroll));
	s_snap->display_enable = b.display_enable;
	s_snap->road_control = b.road_control;
	rbx_video_frame_done();
}

// ---- the analog controls: the wheel from the d-pad / stick (back to the centre when let go), the pedals from
// buttons 1 / 2 (the stick up the gas too), back to rest when let go ----
static void analog_frame(void)
{
	for (int i = 0; i < 8; i++) {
		if (s_port_adc[i] < 0)
			continue;
		const int def = s_adc_def[i];
		if (s_adc_type[i] >= 3) {
			const int gas = s_adc_type[i] == 3, step = gas ? 20 : 40;
			const int on = (rbx_input_bits(1) & (gas ? RBX_IN_B1 : RBX_IN_B2)) || (gas && rbx_analog(1, RBX_IPT_PEDAL) > 2);
			int v = b.adc_val[i] + (on ? step : -step);
			v = v < def ? def : v > 255 ? 255 : v;
			b.adc_val[i] = (uint8_t)v;
			continue;
		}
		if (s_adc_type[i] != 1) {
			b.adc_val[i] = (uint8_t)def;
			continue;
		}
		const int d = rbx_analog_dpad(1, RBX_IPT_PADDLE, 4);
		int v = b.adc_val[i];
		if (d)
			v += d;
		else if (v < def)
			v = v + 4 > def ? def : v + 4;
		else if (v > def)
			v = v - 4 < def ? def : v - 4;
		if (v < def - 0x60) v = def - 0x60;
		if (v > def + 0x60) v = def + 0x60;
		b.adc_val[i] = (uint8_t)v;
	}
}

static uint16_t spin_key(void) { return (uint16_t)(s_fd1094_on ? 0x100 | fd1094_state(&s_fd1094) : 0); }

// ---- init / frame ----
static void orun_reset(void)
{
	// (the work RAM is the NVRAM: kept)
	uint16_t *keep = (uint16_t *)malloc(sizeof(b.workram));
	if (keep)
		memcpy(keep, b.workram, sizeof(b.workram));
	memset(s_bp, 0, sizeof(*s_bp));
	if (keep) {
		memcpy(b.workram, keep, sizeof(b.workram));
		free(keep);
	}
	for (int i = 0; i < 8; i++)
		b.adc_val[i] = s_adc_def[i];
	b.ppi_ctrl = 0x9b;   // (all inputs)
	memset(s_zram, 0, sizeof(s_zram));
	memset(s_pcm.v, 0, sizeof(s_pcm.v));
	memset(s_pcm.ram, 0xff, sizeof(s_pcm.ram));
	z80_init(&s_z80);
	z80_bind();
	s_zirq = 0;
	rbx_ym_reset(0);
	s_mix_n = 0;
	if (s_fd1094_on) {
		fd1094_change_state(&s_fd1094, FD1094_STATE_RESET);
		fd_update();
	}
	m68k_bus_select(NULL);
	s_cur = 0;
	if (s_outrunb) {   // init_outrunb: a memory map of its own
		static const uint8_t memory_map[16] = { 0x02, 0x00, 0x0d, 0x10, 0x00, 0x12, 0x0c, 0x13, 0x08, 0x14, 0x0f, 0x20, 0x00, 0x00, 0x00, 0x00 };
		memcpy(&b.regs[0x10], memory_map, 16);
	}
	update_mapping();
	// the sub CPU, then the main one (left selected)
	m68k_bus_select(&s_bus_sub);
	m68k_bus_irq_reset();
	s_bus_sub.irq_asserted = 0;
	m68k_pulse_reset();
	m68k_get_context(s_ctx[1]);
	s_irqp[1] = 0;
	m68k_bus_select(NULL);
	m68k_bus_irq_reset();
	m68k_bus.irq_asserted = 0;
	m68k_pulse_reset();
	s_irqp[0] = 0;
	s_pal_valid = 0;
}

static void orun_exit(void);

static uint16_t bitswap16(uint16_t v, const int *bits)   // bits[0] is the new bit 15
{
	uint16_t r = 0;
	for (int i = 0; i < 16; i++)
		r |= (uint16_t)(((v >> bits[i]) & 1) << (15 - i));
	return r;
}
static uint8_t bitswap8(uint8_t v, const int *bits)
{
	uint8_t r = 0;
	for (int i = 0; i < 8; i++)
		r |= (uint8_t)(((v >> bits[i]) & 1) << (7 - i));
	return r;
}
static void outrunb_decode(void)   // init_outrunb (before the byte swap: the words as MAME reads them)
{
	static const int mb[16] = { 15, 14, 11, 12, 13, 10, 9, 8, 6, 7, 5, 4, 3, 2, 1, 0 };
	static const int sb[16] = { 14, 15, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 2, 3, 1, 0 };
	static const int r2[8] = { 6, 7, 5, 4, 3, 2, 1, 0 }, r3[8] = { 7, 5, 6, 4, 3, 2, 1, 0 };
	for (uint32_t i = 0; i + 1 < s_rom_size; i += 2) {
		const uint16_t w = bitswap16((uint16_t)(s_rom[i] << 8 | s_rom[i + 1]), mb);
		s_rom[i] = (uint8_t)(w >> 8);
		s_rom[i + 1] = (uint8_t)w;
	}
	for (uint32_t i = 0; i + 1 < s_subrom_size; i += 2) {
		const uint16_t w = bitswap16((uint16_t)(s_subrom[i] << 8 | s_subrom[i + 1]), sb);
		s_subrom[i] = (uint8_t)(w >> 8);
		s_subrom[i + 1] = (uint8_t)w;
	}
	const uint32_t half = s_road_size / 2;
	for (uint32_t i = 0; i < half; i++) {
		s_road[i] = bitswap8(s_road[i], r2);
		s_road[i + half] = bitswap8(s_road[i + half], r3);
	}
	for (uint32_t i = 0; i < s_zrom_size; i++)
		s_zrom[i] = bitswap8(s_zrom[i], r3);
}

static int orun_init(const rbx_game *g)
{
	const char *m = g->machine;
	if (!s_bp && !(s_bp = calloc(1, sizeof(*s_bp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_fd1094_on = strncmp(m, "outrun_fd1094", 13) == 0;
	s_outrunb = strcmp(g->set, "outrunb") == 0;
	s_shangon = strncmp(m, "shangon", 7) == 0;
	ROAD_CB1 = s_shangon ? 0x7f6 : 0x400;   // (video_start: segaic16_road_init)
	ROAD_CB2 = ROAD_CB3 = s_shangon ? 0x7c0 : 0x420;
	if (!s_shangon)
		ROAD_CB3 = 0x780;
	s_tcol_shift = s_shangon ? 5 : 6;        // (TILEMAP_16B_ALT for Super Hang-On)
	s_text_mask = s_shangon ? 0xff : 0x1ff;
	s_text_shift = s_shangon ? 8 : 9;
	m68k_spin_clear(&s_spin[0]);
	m68k_spin_clear(&s_spin[1]);
	s_spin_on = getenv("RBX_NOSPIN") == NULL;
	uint32_t sz;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_subrom = rbx_region("subcpu", &s_subrom_size);
	s_tiles = rbx_region("gfx1", &s_tiles_size);
	s_spr = rbx_region("sprites", &s_spr_size);
	s_road = rbx_region("segaic16road", &s_road_size);
	s_zrom = rbx_region("soundcpu", &s_zrom_size);
	s_pcmrom = rbx_region("pcm", &s_pcm_size);
	if (!s_rom || s_rom_size < 0x60000 || !s_subrom || s_subrom_size < 0x60000 || !s_tiles || !s_spr || !s_road ||
	    !s_zrom || !s_pcmrom) {
		rbx_fail("%s: ROM regions missing", g->set);
		orun_exit();
		return -1;
	}
	if (s_outrunb)
		outrunb_decode();
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_bus_swap16(s_subrom, s_subrom_size);
	if (s_shangon)   // (the 16B sprites read as words)
		m68k_bus_swap16(s_spr, s_spr_size);
	free(s_fd1089_op);
	s_fd1089_op = NULL;
	if (!strncmp(m, "outrun_fd1089a", 14) || !strncmp(m, "shangon_fd1089b", 15)) {
		const uint8_t *key = rbx_region("maincpu:key", &sz);
		s_fd1089_op = (uint16_t *)malloc(s_rom_size);
		if (!key || sz < 0x2000 || !s_fd1089_op) {
			rbx_fail("%s: no FD1089 key", g->set);
			orun_exit();
			return -1;
		}
		fd1089_decrypt(s_shangon, (uint16_t *)s_rom, s_rom_size, key, s_fd1089_op);
	}
	memset(&s_fd1094, 0, sizeof(s_fd1094));
	if (s_fd1094_on) {
		const uint8_t *key = rbx_region("maincpu:key", &sz);
		if (!key || sz < 0x2000) {
			rbx_fail("%s: no FD1094 key", g->set);
			orun_exit();
			return -1;
		}
		fd1094_init(&s_fd1094, (const uint16_t *)s_rom, s_rom_size, key);
	}
	decode_tiles();
	s_roadgfx = (uint8_t *)malloc((256 * 2 + 1) * 512);
	s_snap = calloc(1, sizeof(*s_snap));
	s_pal_conv = (uint16_t *)calloc(NPAL, sizeof(uint16_t));
	s_sbuf_hp = malloc(sizeof(*s_sbuf_hp));
	if (!s_tdec || !s_tmask || !s_roadgfx || !s_snap || !s_pal_conv || !s_sbuf_hp) {
		rbx_fail("%s: out of memory", g->set);
		orun_exit();
		return -1;
	}
	memset(s_sbuf_hp, 0xff, sizeof(*s_sbuf_hp));
	for (int y = 0; y < VIS_H; y++) {
		s_slo[y] = VIS_W;
		s_shi[y] = -1;
	}
	road_decode();
	palette_init();
	{
		s_pcm.rom = s_pcmrom;
		uint32_t p2 = 1;
		while (p2 < s_pcm_size)
			p2 <<= 1;
		s_pcm.mask = p2 - 1;
		s_pcm.bankshift = 12;   // (BANK_512)
		s_pcm.bankmask = 0x70;
	}
	{
		static const char *const dig[4] = { "SERVICE", "UNKNOWN", "COINAGE", "DSW" };
		for (int i = 0; i < 4; i++)
			s_port_dig[i] = rbx_port_find(dig[i]);
		for (int i = 0; i < 8; i++) {
			char tag[8];
			snprintf(tag, sizeof(tag), "ADC.%d", i);
			s_port_adc[i] = rbx_port_find(tag);
			s_adc_def[i] = 0x80;
			s_adc_type[i] = 0;
			for (const rbx_ipt_port *pt = g->ports; pt && pt->tag; pt++)
				if (!strcmp(pt->tag, tag) && pt->count) {   // (the field's own default: MAME's analog value at rest)
					const rbx_ipt_field *f = &g->fields[pt->first];
					s_adc_def[i] = f->type == RBX_IPT_CUSTOM ? 0x80 : (uint8_t)f->def;   // (the bank motor: centred)
					s_adc_type[i] = f->type == RBX_IPT_PADDLE || f->type == RBX_IPT_AD_STICK_X ? 1
					              : f->type == RBX_IPT_PEDAL ? 3 : f->type == RBX_IPT_PEDAL2 ? 4 : 0;
				}
		}
	}
	// sound: YM2151 0.30 (L / R), the SegaPCM 0.70
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 77;
	yc.irq = ym_irq;
	if (rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 2)) {
		rbx_fail("%s: out of memory", g->set);
		orun_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix_l = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_mix_r = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix_l || !s_mix_r || rbx_audio_setup_stereo(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, PAL_BLACK + 1)) {
		rbx_fail("%s: out of memory", g->set);
		orun_exit();
		return -1;
	}
	memset(s_zmap, 0, sizeof(s_zmap));
	for (uint32_t p = 0; p < 0xf0 && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	for (uint32_t p = 0; p < 8; p++)
		s_zmap[0xf8 + p] = s_zram + p * 256;
	if (m68k_context_size() > sizeof(s_ctx[0])) {
		rbx_fail("%s: CPU context too large", g->set);
		orun_exit();
		return -1;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_reset_instr_callback(reset_instr);
	// the main CPU: everything through the mapper (update_mapping puts the pages that go straight to memory)
	m68k_bus_select(NULL);
	m68k_bus_reset_map();
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
	// the sub CPU (a 20-bit bus: mirrored every 1 MB)
	m68k_bus_select(&s_bus_sub);
	m68k_bus_reset_map();
	for (uint32_t mir = 0; mir < 0x1000000; mir += 0x100000) {
		m68k_bus_map(mir + 0x000000, mir + 0x05ffff, s_subrom, 0x60000, 1, 0);
		for (uint32_t r = 0x60000; r < 0x80000; r += 0x8000)
			m68k_bus_map(mir + r, mir + r + 0x7fff, b.subram, sizeof(b.subram), 1, 1);
		for (uint32_t r = 0x80000; r < 0x90000; r += 0x1000)
			m68k_bus_map(mir + r, mir + r + 0xfff, b.roadram, sizeof(b.roadram), 1, 1);
	}
	s_bus_sub.read8 = sub_r8;
	s_bus_sub.read16 = sub_r16;
	s_bus_sub.write8 = sub_w8;
	s_bus_sub.write16 = sub_w16;
	m68k_bus_select(NULL);
	if (s_fd1089_op) {
		m68k_bus.op = (const uint8_t *)s_fd1089_op;
		m68k_bus.op_size = s_rom_size;
	}
	if (s_fd1094_on) {
		rbx_m68k_cmpild_hook = fd_cmpild;
		rbx_m68k_rte_hook = fd_rte;
		rbx_m68k_irqack_hook = fd_irqack;
	}
	orun_reset();
	return 0;
}

static void orun_exit(void)
{
	m68k_spin_free(&s_spin[0]);
	m68k_spin_free(&s_spin[1]);
	m68k_bus_select(NULL);
	s_cur = 0;
	m68k_bus.op = NULL;
	m68k_bus.op_size = 0;
	m68k_set_reset_instr_callback(NULL);
	if (s_fd1094_on) {
		rbx_m68k_cmpild_hook = NULL;
		rbx_m68k_rte_hook = NULL;
		rbx_m68k_irqack_hook = NULL;
	}
	fd1094_exit(&s_fd1094);
	s_fd1094_on = 0;
	free(s_fd1089_op);
	s_fd1089_op = NULL;
	rbx_ym_exit(0);
	free(s_mix_l);
	free(s_mix_r);
	s_mix_l = s_mix_r = NULL;
	free(s_tdec);
	s_tdec = NULL;
	free(s_tmask);
	s_tmask = NULL;
	free(s_roadgfx);
	s_roadgfx = NULL;
	free(s_snap);
	s_snap = NULL;
	free(s_pal_conv);
	s_pal_conv = NULL;
	free(s_sbuf_hp);
	s_sbuf_hp = NULL;
	free(s_bp);
	s_bp = NULL;
}

static void orun_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	analog_frame();
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_H)   // the frame drawn at the vblank's start
			vblank_draw();
		if (line == 261)   // tilemap_16b_latch_values
			for (int i = 0; i < 4; i++) {
				b.latched_page[i] = b.textram[0xe80 / 2 + i];
				b.latched_yscroll[i] = b.textram[0xe90 / 2 + i];
				b.latched_xscroll[i] = b.textram[0xe98 / 2 + i];
			}
		sound_line_begin();
		// scanline_tick: IRQ 2 from the end of lines 65, 129, 193 to the start of the next; the vblank's IRQ 4 for line
		// 223 (both CPUs)
		cpu_switch(0);
		if (line == 66 || line == 130 || line == 194)
			b.irq2 = 0;
		if (line == 223 || line == 224)
			b.vblank_irq = line == 223;
		update_main_irqs();
		const int irq2_line = line == 65 || line == 129 || line == 193;
		int ml = rbx_line_cycles(&b.cpu_frac[0], CPU_CLOCK, FRAME_MHZ, LINES);
		int sl = rbx_line_cycles(&b.cpu_frac[1], CPU_CLOCK, FRAME_MHZ, LINES);
		const int irq2_at = irq2_line ? ml * VIS_W / 400 : -1;   // (the timer at the end of the visible part)
		int done = 0, sub_events = 1;
		while (ml > 0) {
			cpu_switch(0);
			if (irq2_at >= 0 && done >= irq2_at && !b.irq2) {
				b.irq2 = 1;
				update_main_irqs();
			}
			int chunk = ml;
			if (irq2_at >= 0 && done < irq2_at && irq2_at - done < chunk)
				chunk = irq2_at - done;
			const int e = s_spin_on ? m68k_spin_execute(&s_spin[0], chunk, spin_key()) : m68k_execute(chunk);
			b.main_time += (uint64_t)e;
			ml -= e;
			done += e;
			cpu_switch(1);
			if (b.sub_reset) {
				b.sub_reset = 0;
				m68k_pulse_reset();
			}
			if (sub_events) {
				sub_events = 0;
				if (line == 223 || line == 224)
					m68k_bus_set_line(4, line == 223);
			}
			const int se = ml > 0 ? (e < sl ? e : sl) : sl;
			if (se > 0)
				sl -= s_spin_on ? m68k_spin_execute(&s_spin[1], se, 0) : m68k_execute(se);
		}
		if (irq2_at >= 0 && !b.irq2) {
			cpu_switch(0);
			b.irq2 = 1;
			update_main_irqs();
		}
		sound_line();
	}
	cpu_switch(0);
	sound_frame_end();
}

static int orun_state_io(void *ctx, int load)
{
	cpu_switch(0);
	const uint32_t csize = m68k_context_size();
	if (!load)
		m68k_get_context(s_ctx[0]);
	uint32_t irq[4] = { m68k_bus_irq_pending(), m68k_bus.irq_asserted, s_irqp[1], s_bus_sub.irq_asserted };
	z80 z = s_z80;
	int32_t zi = s_zirq;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", s_ctx[0], csize);
	rc |= rbx_state_io(ctx, "m68k_sub", s_ctx[1], csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "orun", s_bp, offsetof(struct orun_state, state_end));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zirq", &zi, sizeof(zi));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= rbx_state_io(ctx, "pcmv", s_pcm.v, sizeof(s_pcm.v));
	rc |= rbx_state_io(ctx, "pcmr", s_pcm.ram, sizeof(s_pcm.ram));
	rc |= rbx_ym_state(0, ctx, load);
	if (s_fd1094_on) {
		uint8_t fs[2] = { s_fd1094.st, s_fd1094.irqmode };
		rc |= rbx_state_io(ctx, "fd1094", fs, sizeof(fs));
		if (!rc && load) {
			s_fd1094.st = fs[0];
			s_fd1094.irqmode = fs[1];
		}
	}
	if (!rc && load) {
		m68k_set_context(s_ctx[0]);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus.irq_asserted = irq[1];
		s_irqp[1] = irq[2];
		s_bus_sub.irq_asserted = irq[3];
		m68k_bus_update_irq();
		s_z80 = z;
		z80_bind();
		s_zirq = zi;
		update_mapping();
		fd_update();
		s_pal_valid = 0;
	}
	return rc ? -1 : 0;
}

static int orun_save(void *ctx) { return orun_state_io(ctx, 0); }
static int orun_load(void *ctx) { return orun_state_io(ctx, 1); }
static int orun_supported(const char *machine)
{
	return strncmp(machine, "outrun", 6) == 0 || strncmp(machine, "shangon", 7) == 0;
}

const rbx_driver rbx_drv_segaorun = { "Sega Out Run", orun_init, orun_exit, orun_reset, orun_frame, orun_save, orun_load,
                                      orun_supported, orun_render, orun_help };
