/*  Sega X-Board: After Burner, After Burner II, Thunder Blade, Last Survivor, Line of Fire, Racing Hero, Super Monaco
    GP, A.B. Cop and GP Rider (one board). Ported from MAME src/mame/sega/segaxbd.cpp, segaic16.cpp (the 16B tilemaps),
    segaic16_road.cpp, segaic16_m.cpp (315-5248 / 5249 / 5250), sega16sp.cpp (the X-Board sprites),
    src/devices/machine/cxd1095.cpp, adc0804.cpp and src/devices/sound/segapcm.cpp (BSD-3-Clause, copyright-holders:
    Aaron Giles, David Haywood, Hiromitsu Shioya, Olivier Galibert, AJR).
    Two 68000s at 12.5 MHz (the main one maybe an FD1094), each with a multiplier, a divider and a compare timer (the main
    one's raising IRQ 2 and the sound NMI), the main CPU's IRQ 4 and the sub CPU's held for one line at line 223; the 16B
    tilemaps, the road (two layers from ROM), the zooming sprites (a buffer swapped by a write), 8192 colours with the
    shadow / hilight copies. Sound: a Z80 at 4 MHz with a YM2151 and a SegaPCM (315-5218; Super Monaco GP: a second Z80 and
    SegaPCM for the rear speakers). 400x262 at 6.25 MHz, 320x224 shown.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"
#include "../../video/decoband.h"
#include "../../cpu/m68k_spin.h"
#include "../machine/fd1094.h"
#include "../machine/segaic16m.h"
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

#define CPU_CLOCK 12500000         // 50 MHz / 4
#define Z80_CLOCK 4000000          // 16 MHz / 4
#define YM_CLOCK 4000000
#define FRAME_MHZ 59637            // 50 MHz / 8 / (400 x 262)
#define LINES 262
#define VIS_W 320
#define VIS_H 224
#define NPAL 8192
#define PAL_BLACK (2 * NPAL)

// ---- ROMs ----
static uint8_t *s_rom, *s_subrom, *s_tiles, *s_spr, *s_road, *s_zrom, *s_zrom2, *s_pcmrom, *s_pcmrom2;
static uint32_t s_rom_size, s_subrom_size, s_tiles_size, s_spr_size, s_road_size, s_zrom_size, s_zrom2_size;
static uint32_t s_pcm_size, s_pcm2_size;
static int s_loffire;
static int s_fd1094_on, s_road_priority, s_gprider_hack, s_lastsurv, s_motor, s_smgp;
static uint8_t s_adc_reverse[8];
static fd1094_t s_fd1094;
static int s_port_io0a, s_port_io0b, s_port_io1[4], s_port_adc[8], s_port_mux[4];

// ---- the state (saved; on the heap while the game runs, ADR-040) ----
static struct xbd_state {
	uint16_t backup1[0x2000], backup2[0x2000];
	uint16_t tileram[0x8000];
	uint16_t textram[0x800];
	uint16_t spriteram[0x800], spritebuf[0x800];
	uint16_t paletteram[0x2000];
	uint16_t subram0[0x2000], subram1[0x2000];
	uint16_t roadram[0x800], roadbuf[0x800];
	uint16_t latched_page[4], latched_yscroll[4], latched_xscroll[4];
	uint16_t mult[2][2], divr[2][8], cmp[2][16];
	uint16_t cmp_counter;
	uint8_t cmp_bit[2], cmp_exck, zint;
	uint8_t road_control, display_enable, pc_0, lastsurv_mux;
	uint8_t timer_irq, vblank_irq, sound_reset, sub_reset;
	uint8_t io_latch[2][5], io_dir[2][5];
	uint8_t adc_result, adc_intr, adc_busy, mute;
	uint64_t adc_done;                         // (the main CPU's time the conversion ends, in its cycles)
	uint8_t adc_val[8];                        // (the analog channels as the controls move them)
	uint64_t main_time;                        // the main CPU's cycles at the start of its slice
	uint64_t cpu_frac[2], z80_frac[2], ym_frac;
	int32_t sync_burst;                        // (Line of Fire: cycles left of the close interleave)
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

static z80 s_z80[2];
static int s_zirq[2];
static uint8_t s_zram[2][0x800];
static const uint8_t *s_zmap[2][256];
static segapcm_t s_pcm[2];
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
	s_cur = n;   // (before the interrupt check: an interrupt taken there runs the hooks for this CPU)
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
	int irq = 0;
	if (b.timer_irq)
		irq |= 2;
	else
		m68k_bus_set_line(2, 0);
	if (b.vblank_irq)
		irq |= 4;
	else
		m68k_bus_set_line(4, 0);
	if (s_gprider_hack && irq > 4)
		irq = 4;
	if (irq != 6)
		m68k_bus_set_line(6, 0);
	if (irq)
		m68k_bus_set_line(irq, 1);
}

static void reset_instr(void)   // m68k_reset_callback: the main CPU's RESET resets the sub CPU
{
	if (s_cur == 0) {   // (at once: MAME's pulse and add_quantum of 100 us)
		b.sub_reset = 1;
		b.sync_burst = CPU_CLOCK / 10000;
		m68k_end_timeslice();
	}
}

// ---- the 315-5248 multiplier, 315-5249 divider, 315-5250 compare timer ----
static void sound_catchup(void);
static void cmp_execute(int c, int update_history)
{
	uint16_t *r = b.cmp[c];
	const int16_t b1 = (int16_t)r[0], b2 = (int16_t)r[1], value = (int16_t)r[2];
	const int16_t mn = b1 < b2 ? b1 : b2, mx = b1 > b2 ? b1 : b2;
	if (value < mn) {
		r[7] = (uint16_t)mn;
		r[3] = 0x8000;
	} else if (value > mx) {
		r[7] = (uint16_t)mx;
		r[3] = 0x4000;
	} else {
		r[7] = (uint16_t)value;
		r[3] = 0x0000;
	}
	if (update_history)
		r[4] |= (uint16_t)((r[3] == 0) << b.cmp_bit[c]++);
}
static void zint_set(int state)
{
	if (state && !b.zint) {   // (the Z80s' NMI: on the edge)
		z80_gen_nmi(&s_z80[0]);
		if (s_smgp)
			z80_gen_nmi(&s_z80[1]);
	}
	b.zint = (uint8_t)state;
}
static uint16_t cmp_r(int c, uint32_t off)
{
	uint16_t *r = b.cmp[c];
	switch (off & 15) {
	case 0x0: return r[0];
	case 0x1: return r[1];
	case 0x2: return r[2];
	case 0x3: return r[3];
	case 0x4: return r[4];
	case 0x5: return r[1];
	case 0x6: return r[2];
	case 0x7: return r[7];
	case 0x9: case 0xd:
		if (c == 0) {   // interrupt_ack
			b.timer_irq = 0;
			update_main_irqs();
		}
		break;
	}
	return 0xffff;
}
static void cmp_w(int c, uint32_t off, uint16_t v, uint16_t mask)
{
	uint16_t *r = b.cmp[c];
#define COMBINE(i) (r[i] = (uint16_t)((r[i] & ~mask) | (v & mask)))
	switch (off & 15) {
	case 0x0: COMBINE(0); cmp_execute(c, 0); break;
	case 0x1: COMBINE(1); cmp_execute(c, 0); break;
	case 0x2: COMBINE(2); cmp_execute(c, 1); break;
	case 0x4: r[4] = 0; b.cmp_bit[c] = 0; break;
	case 0x6: COMBINE(2); cmp_execute(c, 0); break;
	case 0x8: case 0xc: COMBINE(8); break;
	case 0x9: case 0xd:
		if (c == 0) {
			b.timer_irq = 0;
			update_main_irqs();
		}
		break;
	case 0xa: case 0xe: COMBINE(10); break;
	case 0xb: case 0xf:   // write_to_sound
		if (c == 0 && s_cur == 0)
			sound_catchup();
		r[11] = v & 0xff;
		if (c == 0)
			zint_set(1);
		break;
	}
#undef COMBINE
}
static void cmp_exck(int state)   // the main timer clocked with V0
{
	if (b.cmp_exck == state)
		return;
	b.cmp_exck = (uint8_t)state;
	if (!state)
		return;
	const uint16_t old = b.cmp_counter;
	if (b.cmp[0][10] & 1)
		b.cmp_counter++;
	if (old == 0xfff) {   // (the IRQ lines updated by the caller, with the vblank's: one change seen by the CPU)
		b.timer_irq = 1;
		b.cmp_counter = b.cmp[0][8] & 0xfff;
	}
}

// ---- the I/O chips (CXD1095) and the ADC0804 ----
static void adc_poll(void)
{
	if (b.adc_busy && main_now() >= b.adc_done) {   // conversion_done
		b.adc_busy = 0;
		const int which = (b.pc_0 >> 2) & 7;   // analog_r
		uint8_t v = s_port_adc[which] >= 0 ? b.adc_val[which] : 0x10;
		if (s_adc_reverse[which])
			v = (uint8_t)(255 - v);
		b.adc_result = v;
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
	if (!b.adc_busy) {   // 74 clocks of 1.25 MHz: 740 of the CPU's
		b.adc_busy = 1;
		b.adc_done = main_now() + 740;
	}
}

static uint8_t io_in(int chip, int port)
{
	if (chip == 0) {
		if (port == 0) {
			adc_poll();
			uint8_t v = (uint8_t)((rbx_port_read(s_port_io0a) & ~0x40u) | (b.adc_intr ? 0 : 0x40));
			if (s_motor == 1)   // aburner2_motor_r
				v = (uint8_t)((v & 0xc0) | 0x3f);
			else if (s_motor == 2)   // smgp_motor_r
				v &= 0xc0;
			return v;
		}
		if (port == 1)
			return (uint8_t)rbx_port_read(s_port_io0b);
		return 0;   // (C, D: outputs; E: none)
	}
	if (port == 1 && s_lastsurv)   // lastsurv_port_r
		return s_port_mux[b.lastsurv_mux] >= 0 ? (uint8_t)rbx_port_read(s_port_mux[b.lastsurv_mux]) : 0xff;
	return port < 4 ? (uint8_t)rbx_port_read(s_port_io1[port]) : 0;
}

static void z80_bind(int i);
static void pc_0_w(uint8_t data)
{
	b.pc_0 = data;
	b.display_enable = (data & 0x20) != 0;
	const uint8_t reset = !(data & 1);
	if (reset && !b.sound_reset) {   // (held in reset: started again on release)
		for (int i = 0; i < 2; i++) {   // (the cycle count kept: the line's targets are in it)
			const unsigned long cyc = s_z80[i].cyc;
			z80_init(&s_z80[i]);
			z80_bind(i);
			s_z80[i].cyc = cyc;
		}
	}
	b.sound_reset = reset;
}

static uint8_t io_r(int chip, uint32_t off)
{
	off &= 7;
	if (off < 5) {
		uint8_t mask = b.io_dir[chip][off];
		if (off == 4)
			mask &= 0x0f;
		const uint8_t in = mask ? (uint8_t)(io_in(chip, (int)off) & mask) : 0;
		return (uint8_t)(in | (b.io_latch[chip][off] & ~b.io_dir[chip][off]));
	}
	return 0;
}
static void io_w(int chip, uint32_t off, uint8_t data)
{
	off &= 7;
	if (off < 5) {
		if (off == 4)
			data &= 0x0f;
		b.io_latch[chip][off] = data;
		const uint8_t out = (uint8_t)(data & ~b.io_dir[chip][off]);
		if (chip == 0 && off == 2)
			pc_0_w(out);
		else if (chip == 0 && off == 3) {   // pd_0_w: D7 the amplifier on (system_mute)
			b.mute = !(out & 0x80);
			if (s_lastsurv)   // lastsurv_muxer_w
				b.lastsurv_mux = (out >> 5) & 3;
		}
	} else if (off == 6) {
		for (int p = 0; p < 4; p++) {
			b.io_dir[chip][p] = (uint8_t)(((data & 1) ? 0x0f : 0) | ((data & 2) ? 0xf0 : 0));
			data >>= 2;
		}
	} else if (off == 7)
		b.io_dir[chip][4] = (uint8_t)((data & 0x0f) | 0xf0);
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
	b.spriteram[0] = 0xffff;   // (the hack for Thunder Blade)
}

// ---- main CPU bus (the handlers' part) ----
static uint16_t main_r16(uint32_t a)
{
	a &= 0x3fffff;
	if ((a & 0x3ff000) == 0x29c000)   // (Line of Fire: the page with the handshake words, not mapped)
		return b.subram0[(a & 0x3fff) >> 1];
	switch (a >> 16) {
	case 0x0e:
		if (a < 0x0e4000) return sega_mult_r(b.mult[0], a >> 1);
		if (a < 0x0e8000) return sega_div_r(b.divr[0], a >> 1);
		if (a < 0x0ec000) return cmp_r(0, a >> 1);
		break;
	case 0x13: return (uint16_t)(0xff00 | adc_r());
	case 0x14: return (uint16_t)(0xff00 | io_r(0, a >> 1));
	case 0x15: return (uint16_t)(0xff00 | io_r(1, a >> 1));
	case 0x2e:
		if (a < 0x2e4000) return sega_mult_r(b.mult[1], a >> 1);
		if (a < 0x2e8000) return sega_div_r(b.divr[1], a >> 1);
		if (a < 0x2ec000) return cmp_r(1, (a >> 1) & 7);
		if (a >= 0x2ee000) return road_control_r();
		break;
	}
	return 0xffff;
}
static uint8_t main_r8(uint32_t a)
{
	const uint32_t m = a & 0x3fffff;
	if ((m >> 16) == 0x13)
		return (a & 1) ? adc_r() : 0xff;
	if ((m >> 16) == 0x14 || (m >> 16) == 0x15)
		return (a & 1) ? io_r((int)((m >> 16) - 0x14), m >> 1) : 0xff;
	const uint16_t v = main_r16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static void main_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0x3fffff;
	if ((a & 0x3ff000) == 0x29c000) {   // loffire_sync0_w: 0x29c000 - 0x29c011 run the CPUs close for 10 us
		uint16_t *w = &b.subram0[(a & 0x3fff) >> 1];
		*w = (uint16_t)((*w & ~mask) | (v & mask));
		if ((a & 0xfff) < 0x12 && s_cur == 0) {
			b.sync_burst = CPU_CLOCK / 100000;
			m68k_end_timeslice();
		}
		return;
	}
	switch (a >> 16) {
	case 0x0e:
		if (a < 0x0e4000) sega_mult_w(b.mult[0], a >> 1, v, mask);
		else if (a < 0x0e8000) sega_div_w(b.divr[0], a >> 1, v, mask);
		else if (a < 0x0ec000) cmp_w(0, a >> 1, v, mask);
		break;
	case 0x11: sprite_draw_write(); break;
	case 0x13: if (mask & 0xff) adc_w(); break;
	case 0x14: if (mask & 0xff) io_w(0, a >> 1, (uint8_t)v); break;
	case 0x15: if (mask & 0xff) io_w(1, a >> 1, (uint8_t)v); break;
	case 0x2e:
		if (a < 0x2e4000) sega_mult_w(b.mult[1], a >> 1, v, mask);
		else if (a < 0x2e8000) sega_div_w(b.divr[1], a >> 1, v, mask);
		else if (a < 0x2ec000) cmp_w(1, (a >> 1) & 7, v, mask);
		else if (a >= 0x2ee000 && (mask & 0xff)) b.road_control = v & 7;
		break;
	}
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_w16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- sub CPU bus ----
static uint16_t sub_r16(uint32_t a)
{
	a &= 0xfffff;
	if ((a >> 16) == 0x0e) {
		if (a < 0x0e4000) return sega_mult_r(b.mult[1], a >> 1);
		if (a < 0x0e8000) return sega_div_r(b.divr[1], a >> 1);
		if (a < 0x0ec000) return cmp_r(1, (a >> 1) & 7);
		if (a >= 0x0ee000) return road_control_r();
	}
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
	if ((a >> 16) == 0x0e) {
		if (a < 0x0e4000) sega_mult_w(b.mult[1], a >> 1, v, mask);
		else if (a < 0x0e8000) sega_div_w(b.divr[1], a >> 1, v, mask);
		else if (a < 0x0ec000) cmp_w(1, (a >> 1) & 7, v, mask);
		else if (a >= 0x0ee000 && (mask & 0xff)) b.road_control = v & 7;
	}
}
static void sub_w16(uint32_t a, uint16_t v) { sub_w16m(a, v, 0xffff); }
static void sub_w8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	sub_w16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- sound ----
static uint8_t zread(void)   // the compare timer's latch (zread)
{
	zint_set(0);
	return (uint8_t)b.cmp[0][11];
}
static uint8_t z_read(void *u, uint16_t a)
{
	const int i = u ? 1 : 0;
	const uint8_t *rom = i ? s_zrom2 : s_zrom;
	const uint32_t sz = i ? s_zrom2_size : s_zrom_size;
	if (a < 0xf000) return a < sz ? rom[a] : 0xff;
	if (a < 0xf800) return segapcm_r(&s_pcm[i], (uint8_t)a);
	return s_zram[i][a & 0x7ff];
}
static void z_write(void *u, uint16_t a, uint8_t v)
{
	const int i = u ? 1 : 0;
	if (a >= 0xf800) s_zram[i][a & 0x7ff] = v;
	else if (a >= 0xf000) segapcm_w(&s_pcm[i], (uint8_t)a, v);
}
static uint8_t z_in(z80 *z, uint8_t p)
{
	if (z->userdata == NULL && (p & 0xc0) == 0x00)
		return rbx_ym_read(0, p & 1);
	if ((p & 0xc0) == 0x40)
		return zread();
	return 0xff;
}
static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	if (z->userdata == NULL && (p & 0xc0) == 0x00)
		rbx_ym_write(0, p & 1, v);
}
static void ym_irq(int state) { s_zirq[0] = state; }
static void z80_bind(int i)
{
	s_z80[i].read_byte = z_read;
	s_z80[i].write_byte = z_write;
	s_z80[i].port_in = z_in;
	s_z80[i].port_out = z_out;
	s_z80[i].userdata = i ? (void *)&s_z80[1] : NULL;
	s_z80[i].read_map = s_zmap[i];
}

// the Z80s' line: from s_zt0 to s_zt1 (their cycles); run up to the main CPU's time when it writes the sound latch
// (the compare timer's write_to_sound is a synchronize(): the Z80 reaches that time first, then gets the NMI)
static unsigned long s_zt0[2], s_zt1[2];
static uint64_t s_line_mt;
static void sound_line_begin(void)
{
	for (int i = 0; i < (s_smgp ? 2 : 1); i++) {
		const int zc = rbx_line_cycles(&b.z80_frac[i], Z80_CLOCK, FRAME_MHZ, LINES);
		s_zt0[i] = s_z80[i].cyc;
		s_zt1[i] = s_zt0[i] + (unsigned long)zc;
	}
	s_line_mt = b.main_time;
}
// the ports a Z80 reads the same all through a run: all but the compare timer's latch (the YM2151's status changes
// only with its writes and as it is advanced, after the run)
static int z_port_fixed(const z80 *z, uint8_t p) { (void)z; return (p & 0xc0) != 0x40; }
static void z_run(int i, unsigned long end)
{
	if (s_spin_on)
		sega_zspin(&s_z80[i], end, &s_zirq[i], z_port_fixed);
	z80_run(&s_z80[i], end, &s_zirq[i], 0);
}
static void sound_catchup(void)
{
	if (b.sound_reset)
		return;
	const uint64_t el = main_now() - s_line_mt;
	for (int i = 0; i < (s_smgp ? 2 : 1); i++) {
		unsigned long t = s_zt0[i] + (unsigned long)(el * Z80_CLOCK / CPU_CLOCK);
		if ((long)(t - s_zt1[i]) > 0)
			t = s_zt1[i];
		if ((long)(t - s_z80[i].cyc) > 0)
			z_run(i, t);
	}
}

static void sound_line(void)
{
	for (int i = 0; i < (s_smgp ? 2 : 1); i++)
		if (!b.sound_reset && (long)(s_zt1[i] - s_z80[i].cyc) > 0)
			z_run(i, s_zt1[i]);
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&b.ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
	const int n = rbx_ym_advance_lr(0, clocks, s_mix_l + s_mix_n, s_mix_r + s_mix_n, s_mix_cap - s_mix_n);
	for (int k = 0; k < n; k++) {   // (the PCM at half the YM's rate: 4 MHz / 128 against 4 MHz / 64)
		if (!(b.pcm_phase++ & 1)) {
			int32_t l, r, hl, hr;
			segapcm_tick(&s_pcm[0], &l, &r);
			hl = l;
			hr = r;
			if (s_smgp) {   // (the rear speakers)
				segapcm_tick(&s_pcm[1], &l, &r);
				hl += l;
				hr += r;
			}
			b.pcm_hl = hl * 90 / 256;   // 0.35
			b.pcm_hr = hr * 90 / 256;
		}
		s_mix_l[s_mix_n + k] += b.pcm_hl;
		s_mix_r[s_mix_n + k] += b.pcm_hr;
	}
	s_mix_n += n;
}

static void sound_frame_end(void)
{
	if (s_lastsurv) {   // (the YM2151's sides swapped: take them into scratch)
		static int32_t tl[4096], tr[4096];
		const int n = s_mix_n < 4096 ? s_mix_n : 4096;
		memset(tl, 0, sizeof(int32_t) * (size_t)n);
		memset(tr, 0, sizeof(int32_t) * (size_t)n);
		rbx_ym_async_take_lr(0, tl, tr, n);
		for (int i = 0; i < n; i++) {
			s_mix_l[i] += tr[i];
			s_mix_r[i] += tl[i];
		}
	} else
		rbx_ym_async_take_lr(0, s_mix_l, s_mix_r, s_mix_n);
	if (b.mute)   // (the frame's samples muted together: the amplifier switched at frame grain)
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
	uint16_t paletteram[0x2000];
	uint16_t tileram[0x8000];
	uint16_t textram[0x800];
	uint16_t spritebuf[0x800];
	uint16_t roadbuf[0x800];
	uint16_t latched_page[4], latched_yscroll[4], latched_xscroll[4];
	uint8_t display_enable, road_control;
	int pal_dirty;
} *s_snap;
static uint16_t *s_pal_conv;
static int s_pal_valid;
static uint8_t s_pal_normal[32], s_pal_shadow[32], s_pal_hilight[32];
static uint8_t *s_roadgfx;   // (256 x 2 + 1) rows of 512

static void palette_init(void)   // segaxbd_state::palette_init (compute_resistor_weights, 6 resistors)
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
// a tile's row index (code * 8 + py; the codes past the ROM: an empty tile), into s_tdec (x 8) and s_tmask
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

// one layer's line of the 16B tilemaps (tilemap_16b_draw_layer) drawn over d by tile spans, where the pen isn't 0:
// pri |= hi for the tiles of category 1, lo for the others
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
				const uint16_t cb = (uint16_t)(0x1c00 + ((data >> 6) & 0x7f) * 8);
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

static void road_line(int y, uint16_t *dest, int fg)   // segaic16_road_outrun_draw (ROAD_XBOARD, xoffs -166)
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
				dest[x] = (uint16_t)(color | 0x1780);
		return;
	}
	if ((data0 & 0x800) && (data1 & 0x800))
		return;
	const uint8_t *src0 = (data0 & 0x800) ? s_roadgfx + 256 * 2 * 512 : s_roadgfx + (0x000 + ((data0 >> 1) & 0xff)) * 512;
	int hpos0 = roadram[0x200 + ((ctl & 4) ? y : (data0 & 0x1ff))] & 0xfff;
	const int color0 = roadram[0x600 + ((ctl & 4) ? y : (data0 & 0x1ff))];
	const uint8_t *src1 = (data1 & 0x800) ? s_roadgfx + 256 * 2 * 512 : s_roadgfx + (0x100 + ((data1 >> 1) & 0xff)) * 512;
	int hpos1 = roadram[0x400 + ((ctl & 4) ? (0x100 + y) : (data1 & 0x1ff))] & 0xfff;
	const int color1 = roadram[0x600 + ((ctl & 4) ? (0x100 + y) : (data1 & 0x1ff))];
	uint16_t ct[32];
	ct[0x00] = (uint16_t)(0x1700 ^ 0x00 ^ ((color0 >> 0) & 1));
	ct[0x01] = (uint16_t)(0x1700 ^ 0x02 ^ ((color0 >> 1) & 1));
	ct[0x02] = (uint16_t)(0x1700 ^ 0x04 ^ ((color0 >> 2) & 1));
	ct[0x03] = (data0 & 0x200) ? ct[0x00] : (uint16_t)(0x1720 ^ 0x00 ^ ((color0 >> 8) & 0xf));
	ct[0x07] = (uint16_t)(0x1700 ^ 0x06 ^ ((color0 >> 3) & 1));
	ct[0x10] = (uint16_t)(0x1700 ^ 0x08 ^ ((color1 >> 4) & 1));
	ct[0x11] = (uint16_t)(0x1700 ^ 0x0a ^ ((color1 >> 5) & 1));
	ct[0x12] = (uint16_t)(0x1700 ^ 0x0c ^ ((color1 >> 6) & 1));
	ct[0x13] = (data1 & 0x200) ? ct[0x10] : (uint16_t)(0x1720 ^ 0x10 ^ ((color1 >> 8) & 0xf));
	ct[0x17] = (uint16_t)(0x1700 ^ 0x0e ^ ((color1 >> 7) & 1));
	// by runs where neither layer's position crosses 0x200 or wraps (outside 0-0x1ff the pen is 3: the row of 3s)
	const int xo = 0x5f8 - 166;
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

// whether road_line(y, fg 1) draws the whole line (else nothing)
static int road_fg_full(int y)
{
	const uint16_t data0 = s_snap->roadbuf[0x000 + y], data1 = s_snap->roadbuf[0x100 + y];
	switch (s_snap->road_control & 3) {
	case 0: return !(data0 & 0x800);
	case 3: return !(data1 & 0x800);
	default: return !((data0 & 0x800) && (data1 & 0x800));
	}
}

// the sprites of a band (sega_outrun_sprite_device::draw, X-Board; origin x 190), into s_sbuf (0xffff: none; the mix
// puts it back), each row's columns written within [s_slo, s_shi]
#define BANDS 8
static uint16_t (*s_sbuf_hp)[VIS_H][VIS_W];
#define s_sbuf (*s_sbuf_hp)
static int16_t s_slo[VIS_H], s_shi[VIS_H];

static void draw_sprites(int y0, int y1, const uint16_t *list)
{
	const int minx = 190, maxx = 190 + VIS_W - 1;
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
		const int height = (data[5] & 0xfff) + 1;
		const int colpri = ((data[6] & 0xff) << 4) | (((data[3] >> 12) & 7) << 12);
		if (xpos < 0x80 && xdelta < 0)
			xpos += 0x200;
		if (hide)
			continue;
		// (rows outside the band: nothing to draw)
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
				// each pen's run of n pixels (the zoom stepping) filled where it's on the screen; past the far
				// edge nothing more is drawn
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
						// (1:1 with the 8 pixels on the screen; xacc stays 0)
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
						// 1:1 or smaller: a pixel or none per pen. After a pen drawn, xacc + hzoom - 0x200 >= 0x200 skips
						// that many 0x200s of pens (xacc >> 9, the rest in xacc), across words (sk)
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

static void xbd_band(int band)
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
		// colours when it's drawn over them, only their priorities)
		const int full = road_fg_full(y);
		if (!full)
			road_line(y, d, 0);
		else if (s_road_priority == 0)
			road_line(y, d, 1);
		uint16_t *ld = (full && s_road_priority == 1) ? NULL : d;
		layer_line(1, y, ld, pri, 0x01, 0x02);
		layer_line(0, y, ld, pri, 0x02, 0x04);
		if (full && s_road_priority == 1)
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
			const uint16_t cb = (uint16_t)(0x1c00 + ((data >> 9) & 7) * 8);
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
						d[x] = pix & 0xfff;
				}
			}
			s_slo[y] = VIS_W;
			s_shi[y] = -1;
		}
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, xbd_band };
static int xbd_help(int helper) { return decoband_help(&s_bands, helper); }

static void xbd_render(void)
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
	memcpy(s_snap->spritebuf, b.spritebuf, sizeof(b.spritebuf));
	memcpy(s_snap->roadbuf, b.roadbuf, sizeof(b.roadbuf));
	memcpy(s_snap->latched_page, b.latched_page, sizeof(b.latched_page));
	memcpy(s_snap->latched_yscroll, b.latched_yscroll, sizeof(b.latched_yscroll));
	memcpy(s_snap->latched_xscroll, b.latched_xscroll, sizeof(b.latched_xscroll));
	s_snap->display_enable = b.display_enable;
	s_snap->road_control = b.road_control;
	rbx_video_frame_done();
}

// ---- the analog controls (the ADC's channels): d-pad / stick moving them, back to the centre when let go ----
static void analog_frame(void)
{
	for (int i = 0; i < 8; i++) {
		const int p = s_port_adc[i];
		if (p < 0)
			continue;
		const int def = (int)(rbx_port_read(p) & 0xff);
		int type = -1;
		if (i == 0) type = RBX_IPT_AD_STICK_X;
		else if (i == 1) type = RBX_IPT_AD_STICK_Y;
		if (type < 0) {
			b.adc_val[i] = (uint8_t)def;
			continue;
		}
		const int d = rbx_analog_dpad(1, type, 4);
		int v = b.adc_val[i];
		if (d)
			v += (i == 1) ? -d : d;
		else if (v < def)
			v = v + 4 > def ? def : v + 4;
		else if (v > def)
			v = v - 4 < def ? def : v - 4;
		if (v < def - 0x60) v = def - 0x60;
		if (v > def + 0x60) v = def + 0x60;
		if (v < 0) v = 0;
		if (v > 255) v = 255;
		b.adc_val[i] = (uint8_t)v;
	}
}

static uint16_t spin_key(void) { return (uint16_t)(s_fd1094_on ? 0x100 | fd1094_state(&s_fd1094) : 0); }

// ---- init / frame ----
static void xbd_reset(void)
{
	memset(s_bp, 0, sizeof(*s_bp));
	memset(b.io_dir, 0xff, sizeof(b.io_dir));
	for (int i = 0; i < 8; i++)
		b.adc_val[i] = s_port_adc[i] >= 0 ? (uint8_t)rbx_port_read(s_port_adc[i]) : 0x10;
	b.display_enable = 0;
	memset(s_zram, 0, sizeof(s_zram));
	for (int i = 0; i < 2; i++) {
		memset(s_pcm[i].v, 0, sizeof(s_pcm[i].v));
		memset(s_pcm[i].ram, 0xff, sizeof(s_pcm[i].ram));
		z80_init(&s_z80[i]);
		z80_bind(i);
		s_zirq[i] = 0;
	}
	rbx_ym_reset(0);
	s_mix_n = 0;
	if (s_fd1094_on) {
		fd1094_change_state(&s_fd1094, FD1094_STATE_RESET);
		fd_update();
	}
	// the sub CPU, then the main one (left selected)
	m68k_bus_select(NULL);
	s_cur = 0;
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

static void xbd_exit(void);

static int xbd_init(const rbx_game *g)
{
	const char *m = g->machine;
	if (!s_bp && !(s_bp = calloc(1, sizeof(*s_bp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_fd1094_on = strstr(m, "fd1094") != NULL;
	s_road_priority = strstr(m, ":aburner2") ? 0 : 1;
	s_gprider_hack = strstr(m, ":gprider") != NULL;
	s_lastsurv = strstr(m, "lastsurv") != NULL;
	s_smgp = strncmp(m, "sega_smgp", 9) == 0;
	s_motor = strncmp(m, "sega_aburner2", 13) == 0 ? 1 : s_smgp ? 2 : 0;
	memset(s_adc_reverse, 0, sizeof(s_adc_reverse));
	m68k_spin_clear(&s_spin[0]);
	m68k_spin_clear(&s_spin[1]);
	s_spin_on = getenv("RBX_NOSPIN") == NULL;
	s_loffire = strstr(m, ":loffire") != NULL;
	if (s_loffire)
		s_adc_reverse[1] = s_adc_reverse[3] = 1;
	uint32_t sz;
	s_rom = rbx_region("mainpcb:maincpu", &s_rom_size);
	s_subrom = rbx_region("mainpcb:subcpu", &s_subrom_size);
	s_tiles = rbx_region("mainpcb:gfx1", &s_tiles_size);
	s_spr = rbx_region("mainpcb:sprites", &s_spr_size);
	s_road = rbx_region("mainpcb:segaic16road", &s_road_size);
	s_zrom = rbx_region("mainpcb:soundcpu", &s_zrom_size);
	s_pcmrom = rbx_region("mainpcb:pcm", &s_pcm_size);
	s_zrom2 = s_smgp ? rbx_region_opt("mainpcb:soundcpu2", &s_zrom2_size) : NULL;
	s_pcmrom2 = s_smgp ? rbx_region_opt("mainpcb:pcm2", &s_pcm2_size) : NULL;
	if (!s_rom || s_rom_size < 0x80000 || !s_subrom || s_subrom_size < 0x80000 || !s_tiles || !s_spr || !s_road ||
	    !s_zrom || !s_pcmrom) {
		rbx_fail("%s: ROM regions missing", g->set);
		xbd_exit();
		return -1;
	}
	if (s_smgp && !s_zrom2)
		s_smgp = 0;
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_bus_swap16(s_subrom, s_subrom_size);
	memset(&s_fd1094, 0, sizeof(s_fd1094));
	if (s_fd1094_on) {
		const uint8_t *key = rbx_region("mainpcb:maincpu:key", &sz);
		if (!key || sz < 0x2000) {
			rbx_fail("%s: no FD1094 key", g->set);
			xbd_exit();
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
		xbd_exit();
		return -1;
	}
	memset(s_sbuf_hp, 0xff, sizeof(*s_sbuf_hp));
	for (int y = 0; y < VIS_H; y++) {
		s_slo[y] = VIS_W;
		s_shi[y] = -1;
	}
	road_decode();
	palette_init();
	for (int i = 0; i < 2; i++) {
		s_pcm[i].rom = i ? s_pcmrom2 : s_pcmrom;
		uint32_t n = i ? s_pcm2_size : s_pcm_size, p2 = 1;
		while (p2 < n)
			p2 <<= 1;
		s_pcm[i].mask = p2 - 1;
		s_pcm[i].bankshift = 12;   // (BANK_512)
		s_pcm[i].bankmask = 0x70;
	}
	{
		char tag[32];
		s_port_io0a = rbx_port_find("mainpcb:IO0PORTA");
		s_port_io0b = rbx_port_find("mainpcb:IO0PORTB");
		for (int i = 0; i < 4; i++) {
			snprintf(tag, sizeof(tag), "mainpcb:IO1PORT%c", 'A' + i);
			s_port_io1[i] = rbx_port_find(tag);
			snprintf(tag, sizeof(tag), "mainpcb:MUX%d", i);
			s_port_mux[i] = rbx_port_find(tag);
		}
		for (int i = 0; i < 8; i++) {
			snprintf(tag, sizeof(tag), "mainpcb:ADC%d", i);
			s_port_adc[i] = rbx_port_find(tag);
		}
	}
	// sound: YM2151 0.15 (L / R), the SegaPCM 0.35
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 38;
	yc.irq = ym_irq;
	if (rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 2)) {
		rbx_fail("%s: out of memory", g->set);
		xbd_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix_l = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_mix_r = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix_l || !s_mix_r || rbx_audio_setup_stereo(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, PAL_BLACK + 1)) {
		rbx_fail("%s: out of memory", g->set);
		xbd_exit();
		return -1;
	}
	for (int i = 0; i < 2; i++) {
		memset(s_zmap[i], 0, sizeof(s_zmap[i]));
		const uint8_t *rom = i ? s_zrom2 : s_zrom;
		const uint32_t rsz = i ? s_zrom2_size : s_zrom_size;
		for (uint32_t p = 0; p < 0xf0 && rom && p * 256 + 256 <= rsz; p++)
			s_zmap[i][p] = rom + p * 256;
		for (uint32_t p = 0; p < 8; p++)
			s_zmap[i][0xf8 + p] = s_zram[i] + p * 256;
	}
	if (m68k_context_size() > sizeof(s_ctx[0])) {
		rbx_fail("%s: CPU context too large", g->set);
		xbd_exit();
		return -1;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_reset_instr_callback(reset_instr);
	// the main CPU (a 22-bit bus: everything mirrored every 4 MB; the stack at 0xfffffe)
	m68k_bus_select(NULL);
	m68k_bus_reset_map();
	for (uint32_t mir = 0; mir < 0x1000000; mir += 0x400000) {
		m68k_bus_map(mir + 0x000000, mir + 0x07ffff, s_rom, 0x80000, 1, 0);
		m68k_bus_map(mir + 0x080000, mir + 0x09ffff, b.backup1, sizeof(b.backup1), 1, 1);
		m68k_bus_map(mir + 0x0a0000, mir + 0x0bffff, b.backup2, sizeof(b.backup2), 1, 1);
		m68k_bus_map(mir + 0x0c0000, mir + 0x0cffff, b.tileram, sizeof(b.tileram), 1, 1);
		m68k_bus_map(mir + 0x0d0000, mir + 0x0dffff, b.textram, sizeof(b.textram), 1, 1);
		m68k_bus_map(mir + 0x100000, mir + 0x10ffff, b.spriteram, sizeof(b.spriteram), 1, 1);
		m68k_bus_map(mir + 0x120000, mir + 0x12ffff, b.paletteram, sizeof(b.paletteram), 1, 1);
		m68k_bus_map(mir + 0x200000, mir + 0x27ffff, s_subrom, 0x80000, 1, 0);
		m68k_bus_map(mir + 0x280000, mir + 0x29ffff, b.subram0, sizeof(b.subram0), 1, 1);
		m68k_bus_map(mir + 0x2a0000, mir + 0x2bffff, b.subram1, sizeof(b.subram1), 1, 1);
		m68k_bus_map(mir + 0x2ec000, mir + 0x2edfff, b.roadram, sizeof(b.roadram), 1, 1);
		m68k_bus_map(mir + 0x3f8000, mir + 0x3fbfff, b.backup1, sizeof(b.backup1), 1, 1);
		m68k_bus_map(mir + 0x3fc000, mir + 0x3fffff, b.backup2, sizeof(b.backup2), 1, 1);
	}
	if (s_loffire)   // (the handshake page through the handlers)
		for (uint32_t mir = 0; mir < 0x1000000; mir += 0x400000)
			m68k_bus.rd[(mir + 0x29c000) >> M68K_PAGE_BITS] = m68k_bus.wr[(mir + 0x29c000) >> M68K_PAGE_BITS] = NULL;
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
	// the sub CPU (a 20-bit bus: mirrored every 1 MB)
	m68k_bus_select(&s_bus_sub);
	m68k_bus_reset_map();
	for (uint32_t mir = 0; mir < 0x1000000; mir += 0x100000) {
		m68k_bus_map(mir + 0x000000, mir + 0x07ffff, s_subrom, 0x80000, 1, 0);
		m68k_bus_map(mir + 0x080000, mir + 0x09ffff, b.subram0, sizeof(b.subram0), 1, 1);
		m68k_bus_map(mir + 0x0a0000, mir + 0x0bffff, b.subram1, sizeof(b.subram1), 1, 1);
		m68k_bus_map(mir + 0x0ec000, mir + 0x0edfff, b.roadram, sizeof(b.roadram), 1, 1);
	}
	s_bus_sub.read8 = sub_r8;
	s_bus_sub.read16 = sub_r16;
	s_bus_sub.write8 = sub_w8;
	s_bus_sub.write16 = sub_w16;
	m68k_bus_select(NULL);
	if (s_fd1094_on) {
		rbx_m68k_cmpild_hook = fd_cmpild;
		rbx_m68k_rte_hook = fd_rte;
		rbx_m68k_irqack_hook = fd_irqack;
	}
	xbd_reset();
	return 0;
}

static void xbd_exit(void)
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

static void xbd_frame(void)
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
		// the main CPU: scanline_tick (the timer on V0, the vblank IRQ at 223 for one line)
		cpu_switch(0);
		{
			const uint8_t t0 = b.timer_irq;
			cmp_exck(line & 1);
			if (line == 223 || line == 224)
				b.vblank_irq = line == 223;
			if (t0 != b.timer_irq || line == 223 || line == 224)
				update_main_irqs();
		}
		// the two CPUs through the line: the sub CPU kept level with the main one; Line of Fire's handshake writes run
		// them a few cycles apart for 10 us (loffire_sync0_w: add_quantum)
		int ml = rbx_line_cycles(&b.cpu_frac[0], CPU_CLOCK, FRAME_MHZ, LINES);
		int sl = rbx_line_cycles(&b.cpu_frac[1], CPU_CLOCK, FRAME_MHZ, LINES);
		int sub_events = 1;
		while (ml > 0) {
			cpu_switch(0);
			int chunk = b.sync_burst > 0 ? 16 : ml;
			if (chunk > ml)
				chunk = ml;
			const int e = s_spin_on ? m68k_spin_execute(&s_spin[0], chunk, spin_key()) : m68k_execute(chunk);
			b.main_time += (uint64_t)e;
			ml -= e;
			if (b.sync_burst > 0)
				b.sync_burst -= e;
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
		sound_line();
	}
	cpu_switch(0);
	sound_frame_end();
}

static int xbd_state_io(void *ctx, int load)
{
	cpu_switch(0);
	const uint32_t csize = m68k_context_size();
	if (!load)
		m68k_get_context(s_ctx[0]);
	uint32_t irq[4] = { m68k_bus_irq_pending(), m68k_bus.irq_asserted, s_irqp[1], s_bus_sub.irq_asserted };
	z80 z[2] = { s_z80[0], s_z80[1] };
	int32_t zi[2] = { s_zirq[0], s_zirq[1] };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", s_ctx[0], csize);
	rc |= rbx_state_io(ctx, "m68k_sub", s_ctx[1], csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "xbd", s_bp, offsetof(struct xbd_state, state_end));
	rc |= rbx_state_io(ctx, "z80", z, sizeof(z));
	rc |= rbx_state_io(ctx, "zirq", zi, sizeof(zi));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	for (int i = 0; i < 2; i++) {
		rc |= rbx_state_io(ctx, i ? "pcm2v" : "pcmv", s_pcm[i].v, sizeof(s_pcm[i].v));
		rc |= rbx_state_io(ctx, i ? "pcm2r" : "pcmr", s_pcm[i].ram, sizeof(s_pcm[i].ram));
	}
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
		for (int i = 0; i < 2; i++) {
			s_z80[i] = z[i];
			z80_bind(i);
			s_zirq[i] = zi[i];
		}
		fd_update();
		s_pal_valid = 0;
	}
	return rc ? -1 : 0;
}

static int xbd_save(void *ctx) { return xbd_state_io(ctx, 0); }
static int xbd_load(void *ctx) { return xbd_state_io(ctx, 1); }
static int xbd_supported(const char *machine)
{
	return strncmp(machine, "sega_", 5) == 0 && !strstr(machine, "_double") && strncmp(machine, "sega_rascot", 11) != 0;
}

const rbx_driver rbx_drv_segaxbd = { "Sega X-Board", xbd_init, xbd_exit, xbd_reset, xbd_frame, xbd_save, xbd_load,
                                     xbd_supported, xbd_render, xbd_help };
