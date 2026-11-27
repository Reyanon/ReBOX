/*  Data East "dec0" (DE-0297 / DE-0289 ...): Heavy Barrel, Bad Dudes / Dragonninja, Robocop, Hippodrome / Fighting
    Fantasy, and the bootlegs. Ported from MAME src/mame/dataeast/dec0.cpp, dec0_m.cpp, dec0_v.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail, David Haywood).
    68000 at 10 MHz (IRQ 6 at vblank until its acknowledge write), three BAC06 tilemap chips, the MXC06 sprites (their
    list buffered by a write), the i8751 MCU of Heavy Barrel / Bad Dudes (its command and return words, IRQ 5) or the
    bootleg's table, the sound: R65C02 at 1.5 MHz with YM2203, YM3812 (their IRQs on its IRQ) and an OKI, the latch on
    its NMI. Heavy Barrel's 12-way rotary controls. Robocop: a HuC6280 talking through an MB8421 (its interrupts:
    the 68000's IRQ 4, the HuC6280's IRQ1). Hippodrome: a HuC6280 (its program encrypted) with 32 shared bytes, the
    third tilemap chip as 8-bit registers and a protection, IRQ2 at vblank. Birdie Try / Bandit: two uPD4701
    trackball counters.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/h6280/h6280.h"
#include "../../cpu/m6502/m6502.h"
#include "../../cpu/mcs51/mcs51.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../video/bac06.h"
#include "../video/decmxc06.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 10000000       // 20 MHz / 2
#define SND_CLOCK 1500000        // 12 MHz / 8
#define MCU_CLOCK (8000000 / 12) // i8751 machine cycles
#define YM1_CLOCK 1500000        // YM2203: 12 MHz / 8
#define YM2_CLOCK 3000000        // YM3812: 12 MHz / 4
#define FRAME_MHZ 57445          // set_raw(12 MHz / 2, 384, 0, 256, 272, 8, 248)
#define LINES 272
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 256

enum { V_HBARREL, V_BADDUDES, V_DRGNINJAB, V_ROBOCOPB, V_FFANTASYBL, V_ROBOCOP, V_HIPPODRM, V_BIRDTRY, V_BANDIT };
static int s_var;
static uint8_t *s_rom, *s_mcu_rom;
static uint32_t s_rom_size, s_mcu_size;
static uint16_t s_ram[0x2000];             // 0xff8000 .. 0xffbfff (mirrored at 0x318000)
static uint16_t s_spriteram[0x400], s_spriteram_buf[0x400];
static uint16_t s_pal[0x400], s_palext[0x400];
static uint16_t s_ram242800[0xc00];        // 0x242800 .. 0x243fff (Robocop)
static uint16_t s_ram24c880[0x1c0];        // (ffantasybl)
static uint16_t s_pri;
static bac06_t s_tile[3];
static decmxc06_t s_spr;
static rbx_gfx s_gfx8, s_gfx16a, s_gfx16b, s_gfxspr;
static int s_port_in, s_port_sys, s_port_dsw, s_port_vbl;
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac, s_snd_frac, s_mcu_frac, s_ym1_frac, s_ym2_frac;
static uint8_t s_pri_buf[VIS_W * VIS_H];
static int s_rotary[2];                    // (Heavy Barrel: 0 .. 11, before the reverse)
static rbx_rotary s_rot[2];
// (a step of the value clockwise turns the player 45 degrees clockwise. Its facing, x4, 0 = up: RAM 0xff8066 /
// 0xff80aa in hbarrel, found with the harness; the US sets not checked (no ROMs): -1, the right stick turns only)
static int s_hb_face;
static int hb_facing(int p) { return s_hb_face ? (s_ram[p ? 0x55 : 0x33] >> 10) & 7 : -1; }
// the uPD4701s (Birdie Try, Bandit): the counters (12 bits), their starts (reset_x / y), x reversed
static int32_t s_tb[2][2], s_tb_start[2][2];
static struct {
	bac06_t tile[3];
	uint16_t spriteram[0x400], pal[0x400], palext[0x400], pri;
	uint32_t frame;
} *s_snap;

// ---- i8751 (Heavy Barrel, Bad Dudes): command word in, return word out, IRQ 5 ----
static mcs51_t s_mcu;
static int s_has_mcu;
static uint16_t s_i8751_return, s_i8751_command;
static uint8_t s_i8751_ports[4];
static int s_irq5_hold;

static uint8_t mcu_in(void *u, int port)
{
	(void)u;
	if (port != 0)
		return 0xff;
	uint8_t r = 0xff;   // dec0_mcu_port0_r
	if (!(s_i8751_ports[2] & 0x10)) r &= (uint8_t)(s_i8751_command >> 8);
	if (!(s_i8751_ports[2] & 0x20)) r &= (uint8_t)(s_i8751_command & 0xff);
	return r;
}
static void mcu_out(void *u, int port, uint8_t data)
{
	(void)u;
	if (port == 2) {   // dec0_mcu_port2_w
		if (!(data & 0x04) && (s_i8751_ports[2] & 0x04))
			s_irq5_hold = 1;
		if (!(data & 0x08))
			mcs51_set_input(&s_mcu, MCS51_INT1_LINE, 0);
		if ((data & 0x40) && !(s_i8751_ports[2] & 0x40))
			s_i8751_return = (uint16_t)((s_i8751_return & 0xff00) | s_i8751_ports[0]);
		if ((data & 0x80) && !(s_i8751_ports[2] & 0x80))
			s_i8751_return = (uint16_t)((s_i8751_return & 0x00ff) | (s_i8751_ports[0] << 8));
	}
	s_i8751_ports[port & 3] = data;
}
static void i8751_w(uint16_t data)
{
	if (s_var == V_DRGNINJAB) {   // (the bootleg: the MCU's answers as a table)
		static const uint16_t cmd[16] = { 0x714, 0x73b, 0x72c, 0x73f, 0x755, 0x722, 0x72b, 0x724, 0x728, 0x735, 0x71d,
		                                  0x721, 0x73e, 0x761, 0x753, 0x75b };
		s_i8751_return = 0;
		for (int i = 0; i < 16; i++)
			if (cmd[i] == data)
				s_i8751_return = (uint16_t)(0x700 + i);
		s_irq5_hold = 1;
		return;
	}
	s_i8751_command = data;
	if (s_has_mcu && (s_i8751_ports[2] & 0x08))
		mcs51_set_input(&s_mcu, MCS51_INT1_LINE, 1);
}

// ---- the HuC6280 of Robocop (MB8421) / Hippodrome (shared bytes, protection) ----
#define SUB_CLOCK (21477272 / 16)
static h6280_t s_sub;
static int s_has_sub;
static uint8_t s_sub_ram[0x2000];
static uint8_t s_mb[0x800];                // the MB8421 (Robocop)
static int s_mb_intl, s_mb_intr;
static uint8_t s_shared[0x20];             // (Hippodrome)
static uint8_t s_prot_msb, s_prot_lsb;
static uint64_t s_sub_frac;

static uint8_t mb_left_r(int o)
{
	o &= 0x7ff;
	if (o == 0x7fe && s_mb_intl) { s_mb_intl = 0; m68k_bus_set_line(4, 0); }
	return s_mb[o];
}
static void mb_left_w(int o, uint8_t v)
{
	o &= 0x7ff;
	s_mb[o] = v;
	if (o == 0x7ff) { s_mb_intr = 1; h6280_set_irq(&s_sub, H6280_IRQ1, 1); }
}
static uint8_t sub_r(void *ctx, uint32_t pa)
{
	(void)ctx;
	if (s_var == V_ROBOCOP) {
		if (pa >= 0x1f2000 && pa < 0x1f2800) {
			const int o = (int)(pa & 0x7ff);
			if (o == 0x7ff && s_mb_intr) { s_mb_intr = 0; h6280_set_irq(&s_sub, H6280_IRQ1, 0); }
			return s_mb[o];
		}
		return 0;
	}
	if (pa >= 0x180000 && pa < 0x180020) return s_shared[pa & 0x1f];
	if (pa >= 0x1a1000 && pa < 0x1a1800) {   // vram8_r<true>
		const uint32_t o = pa & 0x7ff;
		return (uint8_t)(bac06_vram_r(&s_tile[2], (int)(o >> 1)) >> ((o & 1) << 3));
	}
	if (pa >= 0x1d0000 && pa < 0x1d0100)   // prot_r
		return s_prot_lsb == 0x45 ? 0x4e : s_prot_lsb == 0x92 ? 0x15 : 0;
	return 0;
}
static void sub_w(void *ctx, uint32_t pa, uint8_t v)
{
	(void)ctx;
	if (s_var == V_ROBOCOP) {
		if (pa >= 0x1f2000 && pa < 0x1f2800) {
			const int o = (int)(pa & 0x7ff);
			s_mb[o] = v;
			if (o == 0x7fe) { s_mb_intl = 1; m68k_bus_set_line(4, 1); }
		}
		return;
	}
	if (pa >= 0x180000 && pa < 0x180020) s_shared[pa & 0x1f] = v;
	else if (pa >= 0x1a0000 && pa < 0x1a0008) {   // ctrlreg8_packed_w<true>
		const uint32_t o = pa & 7;
		bac06_ctrl_w(&s_tile[2], (int)(o >> 1), (uint16_t)(v << ((o & 1) << 3)), (uint16_t)(0xff << ((o & 1) << 3)));
	} else if (pa >= 0x1a0010 && pa < 0x1a0020) {   // scrollreg8_w<true> (from offset 4 on: 8-bit, mirrored)
		const uint32_t o = pa & 0xf, sh = o < 4 ? (o & 1) << 3 : 0;
		bac06_scroll_w(&s_tile[2], (int)(o >> 1), (uint16_t)(v << sh), (uint16_t)(0xff << sh));
	} else if (pa >= 0x1a1000 && pa < 0x1a1800) {   // vram8_w<true>
		const uint32_t o = pa & 0x7ff;
		bac06_vram_w(&s_tile[2], (int)(o >> 1), (uint16_t)(v << ((o & 1) << 3)), (uint16_t)(0xff << ((o & 1) << 3)));
	} else if (pa >= 0x1d0000 && pa < 0x1d0100) {   // prot_w
		if ((pa & 0xff) == 4) s_prot_msb = v;
		else if ((pa & 0xff) == 5) s_prot_lsb = v;
	}
}

// ---- sound: R65C02, YM2203, YM3812, OKI ----
static m6502_t s_snd;
static uint8_t s_sram[0x800];
static uint8_t *s_zrom;
static uint32_t s_zrom_size;
static okim6295_t s_oki;
static uint8_t s_latch;
static int s_latch_pending, s_ym_irq[2];
static uint32_t s_oki_acc;
static int32_t s_oki_hold;
static int32_t *s_mix, *s_ym2buf;
static int s_mix_n, s_mix_cap, s_ym2_n, s_ym2_cap;

static void snd_irq(void) { m6502_set_irq(&s_snd, s_ym_irq[0] | s_ym_irq[1]); }
static void ym1_irq(int state) { s_ym_irq[0] = state; snd_irq(); }
static void ym2_irq(int state) { s_ym_irq[1] = state; snd_irq(); }
static void latch_w(uint8_t v)   // (generic latch: its data pending on the NMI)
{
	s_latch = v;
	s_latch_pending = 1;
	m6502_set_nmi(&s_snd, 1);
}
static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x0800) return s_sram[a];
	if (a == 0x0800 || a == 0x0801) return rbx_ym_read(0, a & 1);
	if (a == 0x1000 || a == 0x1001) return rbx_ym_read(1, a & 1);
	if (a == 0x3000) {
		s_latch_pending = 0;
		m6502_set_nmi(&s_snd, 0);
		return s_latch;
	}
	if (a == 0x3800) return oki_read(&s_oki);
	if (a >= 0x8000) return a < s_zrom_size ? s_zrom[a] : 0;
	return 0;
}
static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0800) s_sram[a] = v;
	else if (a == 0x0800 || a == 0x0801) rbx_ym_write(0, a & 1, v);
	else if (a == 0x1000 || a == 0x1001) rbx_ym_write(1, a & 1, v);
	else if (a == 0x3800) oki_write(&s_oki, v);
}

// the 6502 and the chips through one line: the YMs' samples on the worker, the YM3812's at its own rate (fitted to the
// YM2203's at the frame's end), the OKI held at the YM2203's rate
static void sound_line(void)
{
	s_snd_frac += (uint64_t)SND_CLOCK * 1000;
	const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
	const int64_t cyc = (int64_t)(s_snd_frac / den);
	s_snd_frac -= (uint64_t)cyc * den;
	m6502_run_until(&s_snd, s_snd.cycles + cyc);
	const int n = rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&s_ym1_frac, YM1_CLOCK, FRAME_MHZ, LINES), s_mix + s_mix_n,
	                             s_mix_cap - s_mix_n);
	s_ym2_n += rbx_ym_advance(1, (uint32_t)rbx_line_cycles(&s_ym2_frac, YM2_CLOCK, FRAME_MHZ, LINES), s_ym2buf + s_ym2_n,
	                          s_ym2_cap - s_ym2_n);
	const uint32_t yr = rbx_ym_rate(0);
	for (int i = 0; i < n; i++) {
		s_oki_acc += s_oki.rate;
		while (s_oki_acc >= yr) {
			s_oki_acc -= yr;
			s_oki_hold = oki_sample(&s_oki);
		}
		s_mix[s_mix_n + i] += s_oki_hold;
	}
	s_mix_n += n;
}
static void sound_frame_end(void)
{
	rbx_ym_async_take(0, s_mix, s_mix_n);
	rbx_ym_async_take(1, s_ym2buf, s_ym2_n);
	if (s_ym2_n > 0)
		for (int i = 0; i < s_mix_n; i++)
			s_mix[i] += s_ym2buf[(int)((int64_t)i * s_ym2_n / s_mix_n)];
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	memset(s_ym2buf, 0, sizeof(int32_t) * (size_t)s_ym2_n);
	s_mix_n = s_ym2_n = 0;
}

// ---- bus ----
static int vblank(void) { return s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H; }

static uint16_t rotary(int p)   // (12 positions, MAME's remap table, reversed)
{
	static const uint16_t table[12] = { 0xfffe, 0xfffd, 0xfffb, 0xfff7, 0xffef, 0xffdf, 0xffbf, 0xff7f,
	                                    0xfeff, 0xfdff, 0xfbff, 0xf7ff };
	return table[(12 - s_rotary[p]) % 12];
}

static bac06_t *tile_of(uint32_t a, uint32_t *o)
{
	static const uint32_t base[3] = { 0x240000, 0x246000, 0x24c000 };
	for (int i = 0; i < 3; i++)
		if (a >= base[i] && a < base[i] + (i == 0 ? 0x6000u : i == 1 ? 0x4800u : 0x1800u)) {
			*o = a - base[i];
			return &s_tile[i];
		}
	return NULL;
}
// the three chips' registers and RAM (MAME dec0_map: the text layer's at 0x240000, the others at 0x246000, 0x24c000)
static int tile_r(uint32_t a, uint16_t *v)
{
	uint32_t o;
	bac06_t *t = tile_of(a, &o);
	if (!t)
		return 0;
	const int i = (int)(t - s_tile);
	const uint32_t cs = i == 0 ? 0x2000 : i == 1 ? 0x2000 : 0x800;   // (column scroll, row scroll, VRAM offsets)
	const uint32_t rs = cs + (i == 2 ? 0x400 : 0x400), vr = i == 0 ? 0x4000 : i == 1 ? 0x4000 : 0x1000;
	if (i == 0 && o >= 0x2800 && o < 0x4000) *v = s_ram242800[(o - 0x2800) >> 1];   // (Robocop's RAM)
	else if (i == 2 && o >= 0x880 && o < 0xc00) *v = s_ram24c880[(o - 0x880) >> 1];   // (ffantasybl)
	else if (o >= cs && o < cs + 0x80) *v = t->colscroll[(o - cs) >> 1];
	else if (o >= rs && o < rs + 0x400) *v = t->rowscroll[(o - rs) >> 1];
	else if (o >= vr && o < vr + (i == 0 ? 0x2000u : 0x800u)) *v = bac06_vram_r(t, (int)((o - vr) >> 1));
	else return 0;
	return 1;
}
static int tile_w(uint32_t a, uint16_t v, uint16_t mask)
{
	uint32_t o;
	bac06_t *t = tile_of(a, &o);
	if (!t)
		return 0;
	const int i = (int)(t - s_tile);
	const uint32_t cs = i == 0 ? 0x2000 : i == 1 ? 0x2000 : 0x800;
	const uint32_t rs = cs + 0x400, vr = i == 0 ? 0x4000 : i == 1 ? 0x4000 : 0x1000;
	if (o < 0x8) bac06_ctrl_w(t, (int)(o >> 1), v, mask);
	else if (o >= 0x10 && o < 0x18) bac06_scroll_w(t, (int)((o - 0x10) >> 1), v, mask);
	else if (i == 0 && o >= 0x2800 && o < 0x4000) { uint16_t *p = &s_ram242800[(o - 0x2800) >> 1]; *p = (uint16_t)((*p & ~mask) | (v & mask)); }
	else if (i == 2 && o >= 0x880 && o < 0xc00) { uint16_t *p = &s_ram24c880[(o - 0x880) >> 1]; *p = (uint16_t)((*p & ~mask) | (v & mask)); }
	else if (o >= cs && o < cs + 0x80) { uint16_t *p = &t->colscroll[(o - cs) >> 1]; *p = (uint16_t)((*p & ~mask) | (v & mask)); }
	else if (o >= rs && o < rs + 0x400) { uint16_t *p = &t->rowscroll[(o - rs) >> 1]; *p = (uint16_t)((*p & ~mask) | (v & mask)); }
	else if (o >= vr && o < vr + (i == 0 ? 0x2000u : 0x800u)) bac06_vram_w(t, (int)((o - vr) >> 1), v, mask);
	else return 0;
	return 1;
}

static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	uint16_t v;
	if (tile_r(a, &v)) return v;
	if (s_var == V_FFANTASYBL && a == 0x242024) return 0xffff;   // ffantasybl_242024_r
	if (s_var == V_ROBOCOP && a >= 0x180000 && a < 0x181000) return mb_left_r((int)((a - 0x180000) >> 1));   // (umask 0x00ff)
	if (s_var == V_HIPPODRM && a >= 0x180000 && a < 0x180040) return s_shared[(a >> 1) & 0x1f];   // sharedram_r
	if (s_var == V_HIPPODRM && a >= 0xffc000 && a < 0xffd000) return a < 0xffc800 ? s_spriteram[(a & 0x7ff) >> 1] : 0;
	if ((s_var == V_BIRDTRY || s_var == V_BANDIT) && a >= 0x300010 && a < 0x300020) {   // read_xy (umask 0x00ff)
		const int chip = a >= 0x300018, o = (int)((a >> 1) & 3);
		const uint16_t v = (uint16_t)((s_tb[chip][o >> 1] - s_tb_start[chip][o >> 1]) & 0xfff);
		return (uint16_t)(o & 1 ? v >> 8 : v & 0xff);
	}
	if (a == 0x300000) return s_var == V_HBARREL ? rotary(0) : (uint16_t)0xffff;   // AN0
	if (a == 0x300008) return s_var == V_HBARREL ? rotary(1) : (uint16_t)0xffff;   // AN1
	if (a >= 0x30c000 && a < 0x30c00c) {   // dec0_controls_r
		switch (a & 0xe) {
		case 0x0: return (uint16_t)rbx_port_read(s_port_in);
		case 0x2:
			rbx_port_custom(s_port_sys, 0x0080, vblank() ? 0x0080 : 0);
			return (uint16_t)rbx_port_read(s_port_sys);
		case 0x4: return (uint16_t)rbx_port_read(s_port_dsw);
		case 0x8: return (s_has_mcu || s_var == V_DRGNINJAB) ? s_i8751_return : 0;
		}
		return 0xffff;
	}
	if (s_var == V_FFANTASYBL && a == 0xff87ee) {   // VBLANK
		rbx_port_custom(s_port_vbl, 0xffff, vblank() ? 0xffff : 0);
		return (uint16_t)rbx_port_read(s_port_vbl);
	}
	if (a >= 0xff8000 && a < 0xff9000) return s_ram[(a - 0xff8000) >> 1];   // (ffantasybl: that page read here)
	return 0;
}
static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (tile_w(a, v, mask))
		return;
	if ((s_var == V_BIRDTRY || s_var == V_BANDIT) && a >= 0x300000 && a < 0x300020 && (a & 7) == 0 && (mask & 0xff)) {
		// reset_x_w / reset_y_w of the two chips (0x300001, 0x300009, 0x300011, 0x300019)
		const int chip = (a >> 4) & 1, axis = (a >> 3) & 1;
		s_tb_start[chip][axis] = s_tb[chip][axis];
		return;
	}
	if (s_var == V_ROBOCOP && a >= 0x180000 && a < 0x181000) {
		if (mask & 0xff)
			mb_left_w((int)((a - 0x180000) >> 1), (uint8_t)v);
		return;
	}
	if (s_var == V_HIPPODRM && a >= 0x180000 && a < 0x180040) {   // sharedram_w
		if (mask & 0xff)
			s_shared[(a >> 1) & 0x1f] = (uint8_t)v;
		return;
	}
	if (s_var == V_HIPPODRM && a >= 0xffc000 && a < 0xffd000) {   // (0xffc800: sprite_mirror_w)
		uint16_t *w = &s_spriteram[(a & 0x7ff) >> 1];
		*w = (uint16_t)((*w & ~mask) | (v & mask));
		return;
	}
	if (a >= 0x30c010 && a < 0x30c020) {   // dec0_control_w
		switch (a & 0xe) {
		case 0x0: s_pri = (uint16_t)((s_pri & ~mask) | (v & mask)); break;
		case 0x2: memcpy(s_spriteram_buf, s_spriteram, sizeof(s_spriteram_buf)); break;   // the sprite DMA
		case 0x4: if (mask & 0xff) latch_w((uint8_t)v); break;
		case 0x6: if (s_has_mcu || s_var == V_DRGNINJAB) i8751_w(v); break;
		case 0x8: m68k_bus_set_line(6, 0); break;   // the VBL IRQ ack
		case 0xe: if (s_has_mcu || s_var == V_DRGNINJAB) s_i8751_return = s_i8751_command = 0; break;
		}
	}
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static void hbarrel_colpri(void *u, uint32_t *colour, uint32_t *pri_mask)
{
	(void)u;
	*pri_mask = 0xf0;   // (GFX_PMASK_4: above the playfields; colour bit 3: behind the foreground, GFX_PMASK_2)
	if (*colour & 8)
		*pri_mask |= 0xcc;
}
static void robocop_colpri(void *u, uint32_t *colour, uint32_t *pri_mask)
{
	(void)u;
	*pri_mask = 0;
	if (s_snap->pri & 2) {
		const uint32_t trans = (s_snap->pri & 4) ? 0x08 : 0x00;
		if ((*colour & 0x08) == trans)
			*pri_mask |= 0xcc;
	}
}
static void bandit_colpri(void *u, uint32_t *colour, uint32_t *pri_mask)
{
	(void)u;
	(void)colour;
	*pri_mask = 0;   // (above all)
	if (s_snap->pri == 0)
		*pri_mask |= 0xf0;   // (GFX_PMASK_4: behind the foreground)
	if (s_snap->pri == 7)
		*pri_mask |= 0xf0 | 0xcc;
}
static void baddudes_tile_cb(uint32_t *tile, uint32_t *colour, uint32_t *flags)
{
	(void)tile;
	if (*colour & 8)
		*flags |= BAC06_GROUP1;
}

static void dc_render(void)
{
	for (int i = 0; i < 0x400; i++)   // xBGR_888 split: red / green in the palette, blue in the extension
		rbx_video.palette[i] = (uint32_t)(s_snap->pal[i] & 0xff) << 16 | (s_snap->pal[i] & 0xff00) | (s_snap->palext[i] & 0xff);
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	bac06_t *t0 = &s_snap->tile[0], *t1 = &s_snap->tile[1], *t2 = &s_snap->tile[2];
	const int flip = (t0->ctrlreg[0] & 0x80) != 0;
	for (int i = 0; i < 3; i++)
		s_snap->tile[i].flip_screen = flip;
	decmxc06_t spr = s_spr;
	spr.flip_screen = flip;
	memset(s_pri_buf, 0, sizeof(s_pri_buf));
	const uint16_t pri = s_snap->pri;
	switch (s_var) {
	case V_BIRDTRY:   // (no third chip on the board: its palette shows through)
		for (int y = 0; y < VIS_H; y++)
			for (int x = 0; x < VIS_W; x++)
				bm[y * pitch + x] = 768;
		bac06_draw(t1, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
		decmxc06_draw(&spr, s_snap->spriteram, 0x400, bm, NULL, pitch, VIS_W, VIS_H, VIS_Y0, s_snap->frame);
		bac06_draw(t0, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
		break;
	case V_BANDIT:
		if (pri == 7) {
			bac06_draw(t1, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, BAC06_OPAQUE, 1, 0xff);
			bac06_draw(t2, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 2, 0xff);
		} else {
			bac06_draw(t2, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, BAC06_OPAQUE, 1, 0xff);
			bac06_draw(t1, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 2, 0xff);
		}
		bac06_draw(t0, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 4, 0xff);
		decmxc06_draw(&spr, s_snap->spriteram, 0x400, bm, s_pri_buf, pitch, VIS_W, VIS_H, VIS_Y0, s_snap->frame);
		break;
	case V_HBARREL:   // (pf2 always over pf3)
		bac06_draw(t2, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, BAC06_OPAQUE, 1, 0xff);
		bac06_draw(t1, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 2, 0xff);
		bac06_draw(t0, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 4, 0xff);
		decmxc06_draw(&spr, s_snap->spriteram, 0x400, bm, s_pri_buf, pitch, VIS_W, VIS_H, VIS_Y0, s_snap->frame);
		break;
	case V_BADDUDES:
	case V_DRGNINJAB: {   // (inverted against Midnight Resistance)
		bac06_t *fg = (pri & 1) ? t1 : t2, *bg = (pri & 1) ? t2 : t1;
		bac06_draw(bg, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, BAC06_OPAQUE, 0, 0xff);
		bac06_draw(fg, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, BAC06_LAYER0 | BAC06_LAYER1, 0, 0xff);
		if (pri & 2)
			bac06_draw(bg, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, BAC06_LAYER0, 0, 0xff);
		decmxc06_draw(&spr, s_snap->spriteram, 0x400, bm, NULL, pitch, VIS_W, VIS_H, VIS_Y0, s_snap->frame);
		if (pri & 4)
			bac06_draw(fg, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, BAC06_LAYER0, 0, 0xff);
		bac06_draw(t0, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
		break;
	}
	default: {   // screen_update_robocop
		bac06_t *fg = (pri & 1) ? t2 : t1, *bg = (pri & 1) ? t1 : t2;
		bac06_draw(bg, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, BAC06_OPAQUE, 1, 0xff);
		bac06_draw(fg, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 2, 0xff);
		decmxc06_draw(&spr, s_snap->spriteram, 0x400, bm, s_pri_buf, pitch, VIS_W, VIS_H, VIS_Y0, s_snap->frame);
		bac06_draw(t0, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
		break;
	}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->tile, s_tile, sizeof(s_tile));
	memcpy(s_snap->spriteram, s_spriteram_buf, sizeof(s_spriteram_buf));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	memcpy(s_snap->palext, s_palext, sizeof(s_palext));
	s_snap->pri = s_pri;
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void dc_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_spriteram_buf, 0, sizeof(s_spriteram_buf));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_palext, 0, sizeof(s_palext));
	memset(s_ram242800, 0, sizeof(s_ram242800));
	memset(s_ram24c880, 0, sizeof(s_ram24c880));
	s_pri = 0;
	for (int i = 0; i < 3; i++) {
		memset(s_tile[i].vram, 0, sizeof(s_tile[i].vram));
		memset(s_tile[i].rowscroll, 0, sizeof(s_tile[i].rowscroll));
		memset(s_tile[i].colscroll, 0, sizeof(s_tile[i].colscroll));
		bac06_reset(&s_tile[i]);
	}
	s_i8751_return = s_i8751_command = 0;
	memset(s_i8751_ports, 0, sizeof(s_i8751_ports));
	s_irq5_hold = 0;
	if (s_has_mcu)
		mcs51_reset(&s_mcu);
	if (s_has_sub) {
		memset(s_sub_ram, 0, sizeof(s_sub_ram));
		memset(s_mb, 0, sizeof(s_mb));
		memset(s_shared, 0, sizeof(s_shared));
		s_mb_intl = s_mb_intr = 0;
		s_prot_msb = s_prot_lsb = 0;
		h6280_reset(&s_sub);
		s_sub_frac = 0;
	}
	memset(s_sram, 0, sizeof(s_sram));
	s_latch = 0;
	s_latch_pending = 0;
	s_ym_irq[0] = s_ym_irq[1] = 0;
	m6502_reset(&s_snd);
	m6502_set_irq(&s_snd, 0);
	m6502_set_nmi(&s_snd, 0);
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	oki_reset(&s_oki);
	s_oki_acc = 0;
	s_oki_hold = 0;
	s_mix_n = s_ym2_n = 0;
	s_rotary[0] = s_rotary[1] = 0;
	memset(s_rot, 0, sizeof(s_rot));
	memset(s_tb, 0, sizeof(s_tb));
	memset(s_tb_start, 0, sizeof(s_tb_start));
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = s_snd_frac = s_mcu_frac = s_ym1_frac = s_ym2_frac = 0;
}

static void dc_exit(void);

static int dc_init(const rbx_game *g)
{
	s_var = !strcmp(g->machine, "hbarrel") ? V_HBARREL : !strcmp(g->machine, "baddudes") ? V_BADDUDES
	      : !strcmp(g->machine, "drgninjab") ? V_DRGNINJAB : !strcmp(g->machine, "ffantasybl") ? V_FFANTASYBL
	      : !strcmp(g->machine, "robocop") ? V_ROBOCOP : !strcmp(g->machine, "hippodrm") ? V_HIPPODRM
	      : !strcmp(g->machine, "birdtry") ? V_BIRDTRY : !strcmp(g->machine, "bandit") ? V_BANDIT : V_ROBOCOPB;
	uint32_t csize, t1s, t2s, ssize, osize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_mcu_rom = rbx_region("mcu", &s_mcu_size);
	uint8_t *chars = rbx_region("char", &csize), *t1 = rbx_region("tiles1", &t1s), *t2 = rbx_region("tiles2", &t2s),
	        *spr = rbx_region("sprites", &ssize), *oki = rbx_region("oki", &osize);
	if (!s_rom || !s_zrom || !chars || !t1 || !t2 || !spr || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_has_mcu = (s_var == V_HBARREL || s_var == V_BADDUDES || s_var == V_BIRDTRY || s_var == V_BANDIT) && s_mcu_rom &&
	            s_mcu_size >= 0x1000;
	s_has_sub = s_var == V_ROBOCOP || s_var == V_HIPPODRM;
	if (s_has_sub) {
		uint32_t sub_size;
		uint8_t *sub = rbx_region("sub", &sub_size);
		if (!sub || sub_size < 0x10000) {
			rbx_fail("%s: ROM regions missing", g->set);
			return -1;
		}
		if (s_var == V_HIPPODRM) {   // init_hippodrm: h6280_decrypt (bits 0 and 7 swapped), the protection's RTSs
			for (uint32_t i = 0; i < 0x10000; i++)
				sub[i] = (uint8_t)((sub[i] & 0x7e) | ((sub[i] & 1) << 7) | ((sub[i] & 0x80) >> 7));
			sub[0x189] = sub[0x1af] = sub[0x1db] = sub[0x21a] = 0x60;
		}
		h6280_init(&s_sub);
		for (uint32_t pg = 0; pg < 8; pg++)
			s_sub.rpage[pg] = sub + pg * 0x2000;
		s_sub.rpage[0x1f0000 >> 13] = s_sub.wpage[0x1f0000 >> 13] = s_sub_ram;
		s_sub.read = sub_r;
		s_sub.write = sub_w;
	}
	{	// charlayout (RGN_FRAC(1,4), planes 0, 2, 1, 3 quarters), tilelayout (planes 1, 3, 0, 2 quarters)
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 8);
			x16[i] = (uint32_t)(16 * 8 + i);
			x16[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 8);
		const uint32_t qc = csize * 2, q1 = t1s * 2, q2 = t2s * 2, qs = ssize * 2;   // (a quarter in bits)
		const uint32_t pc[4] = { 0, 2 * qc, qc, 3 * qc };
		const uint32_t p1[4] = { q1, 3 * q1, 0, 2 * q1 }, p2[4] = { q2, 3 * q2, 0, 2 * q2 }, ps[4] = { qs, 3 * qs, 0, 2 * qs };
		const rbx_gfx_layout l8 = { 8, 8, 4, pc, x8, y8, 8 * 8 };
		const rbx_gfx_layout la = { 16, 16, 4, p1, x16, y16, 16 * 16 }, lb = { 16, 16, 4, p2, x16, y16, 16 * 16 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, x16, y16, 16 * 16 };
		if (rbx_gfx_decode(&s_gfx8, &l8, chars, csize / 4) || rbx_gfx_decode(&s_gfx16a, &la, t1, t1s / 4) ||
		    rbx_gfx_decode(&s_gfx16b, &lb, t2, t2s / 4) || rbx_gfx_decode(&s_gfxspr, &ls, spr, ssize / 4)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			dc_exit();
			return -1;
		}
	}
	memset(s_tile, 0, sizeof(s_tile));
	for (int i = 0; i < 3; i++)   // (set_gfx_region_wide(0, i, 0): the chars, tiles1 from 512, tiles2 from 768)
		s_tile[i].gfx8 = &s_gfx8;
	s_tile[1].gfx16 = &s_gfx16a;
	s_tile[1].base16 = 512;
	s_tile[2].gfx16 = &s_gfx16b;
	s_tile[2].base16 = 768;
	if (s_var == V_BADDUDES || s_var == V_DRGNINJAB)
		for (int i = 1; i < 3; i++) {   // (video_start baddudes: set_transmask(0, 0xffff, 0x0001), (1, 0x00ff, 0xff01))
			s_tile[i].tile_cb = baddudes_tile_cb;
			s_tile[i].transmask_on = 1;
			s_tile[i].fgmask[0] = 0xffff;
			s_tile[i].bgmask[0] = 0x0001;
			s_tile[i].fgmask[1] = 0x00ff;
			s_tile[i].bgmask[1] = 0xff01;
		}
	memset(&s_spr, 0, sizeof(s_spr));
	s_spr.gfx = &s_gfxspr;
	s_spr.color_base = 256;
	s_spr.colpri_cb = s_var == V_HBARREL ? hbarrel_colpri : (s_var == V_ROBOCOPB || s_var == V_ROBOCOP) ? robocop_colpri
	                : s_var == V_BANDIT ? bandit_colpri : NULL;
	if (s_has_mcu) {
		mcs51_init(&s_mcu, s_mcu_rom, 0x1000);
		s_mcu.port_in = mcu_in;
		s_mcu.port_out = mcu_out;
	}
	s_snap = calloc(1, sizeof(*s_snap));
	// sound: YM2203 1.5 MHz (SSG 0.81, FM 0.32), YM3812 3 MHz 0.32, OKI 1 MHz 0.66
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM1_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = 207;
	yc.gain[3] = 82;
	yc.irq = ym1_irq;
	int rc = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM3812;
	yc.clock = YM2_CLOCK;
	yc.gain[0] = 82;
	yc.irq = ym2_irq;
	rc = rc || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	if (!rc) {
		s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
		s_ym2_cap = (int)(rbx_ym_rate(1) / 50) + 256;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		s_ym2buf = (int32_t *)calloc((size_t)s_ym2_cap, sizeof(int32_t));
		rc = !s_mix || !s_ym2buf || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 0x400);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		dc_exit();
		return -1;
	}
	oki_init(&s_oki, 20000000 / 2 / 10, 1, oki, osize, 169);
	m6502_init(&s_snd, M6502_R65C02);
	for (int p = 0; p < 8; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = s_sram + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		if ((uint32_t)p * 256 + 256 <= s_zrom_size)
			s_snd.rpage[p] = s_zrom + p * 256;
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	s_port_in = rbx_port_find("INPUTS");
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_dsw = rbx_port_find("DSW");
	s_port_vbl = rbx_port_find("VBLANK");
	rbx_set_rstick_dial(s_var == V_HBARREL);   // (the rotary aims with the right stick)
	s_hb_face = !strcmp(g->set, "hbarrel");
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x5ffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x310000, 0x3107ff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus_map(0x314000, 0x3147ff, s_palext, sizeof(s_palext), 1, 1);
	m68k_bus_map(0x318000, 0x31bfff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x31c000, 0x31c7ff, s_spriteram, sizeof(s_spriteram), 1, 1);
	m68k_bus_map(0xff8000, 0xffbfff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0xffc000, 0xffc7ff, s_spriteram, sizeof(s_spriteram), 1, 1);
	if (s_var == V_FFANTASYBL)   // (0xff87ee: the VBLANK port, through the handler)
		m68k_bus.rd[0xff8000 >> M68K_PAGE_BITS] = NULL;
	if (s_var == V_HIPPODRM)   // (0xffc800: the sprite RAM's write mirror, through the handler)
		m68k_bus.rd[0xffc000 >> M68K_PAGE_BITS] = m68k_bus.wr[0xffc000 >> M68K_PAGE_BITS] = NULL;
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	dc_reset();
	return 0;
}

static void dc_exit(void)
{
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix);
	free(s_ym2buf);
	s_mix = s_ym2buf = NULL;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16a);
	rbx_gfx_free(&s_gfx16b);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
}

static void dc_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	if (s_var == V_HBARREL)
		for (int p = 0; p < 2; p++)   // (the rotary: the positional control, 12 a turn, on the right stick)
			s_rotary[p] = rbx_rotary_turn(&s_rot[p], p + 1, 12, s_rotary[p], 1, 8, hb_facing(p));
	if (s_var == V_BIRDTRY || s_var == V_BANDIT)
		for (int p = 0; p < 2; p++) {   // (the trackballs: key delta 24 x sensitivity 50%; x reversed)
			s_tb[p][0] -= rbx_analog_dpad(p + 1, RBX_IPT_TRACKBALL_X, 12);
			s_tb[p][1] += rbx_analog_dpad(p + 1, RBX_IPT_TRACKBALL_Y, 12);
		}
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // irq6_line_assert at vblank (until the ack write)
			vblank_capture();
			m68k_bus_set_line(6, 1);
		}
		const int cyc = rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		if (s_has_mcu) {   // (the 68000 and the MCU interleaved; Birdie Try needs them tight: sixteenths)
			const int mc = rbx_line_cycles(&s_mcu_frac, MCU_CLOCK, FRAME_MHZ, LINES), n = s_var == V_BIRDTRY ? 16 : 4;
			for (int k = 0; k < n; k++) {
				if (s_irq5_hold) {
					s_irq5_hold = 0;
					m68k_bus_hold_irq(5);
				}
				m68k_execute(cyc * (k + 1) / n - cyc * k / n);
				mcs51_execute(&s_mcu, mc * (k + 1) / n - mc * k / n);
			}
		} else if (s_has_sub) {   // (the 68000 and the HuC6280 interleaved in quarters of a line)
			if (s_var == V_HIPPODRM)   // (the screen's vblank on IRQ2)
				h6280_set_irq(&s_sub, H6280_IRQ2, vblank());
			s_sub_frac += (uint64_t)SUB_CLOCK * 1000;
			const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
			const int64_t sc = (int64_t)(s_sub_frac / den);
			s_sub_frac -= (uint64_t)sc * den;
			const int64_t s0 = s_sub.cycles;
			for (int k = 0; k < 4; k++) {
				m68k_execute(cyc * (k + 1) / 4 - cyc * k / 4);
				h6280_run_until(&s_sub, s0 + sc * (k + 1) / 4);
			}
		} else {
			if (s_irq5_hold) {
				s_irq5_hold = 0;
				m68k_bus_hold_irq(5);
			}
			m68k_execute(cyc);
		}
		sound_line();
	}
	sound_frame_end();
	s_frame++;
}

static int dc_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[5] = { s_cpu_frac, s_snd_frac, s_mcu_frac, s_ym1_frac, s_ym2_frac };
	int32_t misc[12] = { (int32_t)s_frame, s_pri, s_i8751_return, s_i8751_command, s_irq5_hold, s_latch,
	                     s_latch_pending, s_ym_irq[0], s_ym_irq[1], (int32_t)s_oki_acc, s_oki_hold, 0 };
	uint8_t snd[M6502_STATE_SIZE];
	if (!load)
		memcpy(snd, &s_snd, M6502_STATE_SIZE);
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "sprb", s_spriteram_buf, sizeof(s_spriteram_buf));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "palx", s_palext, sizeof(s_palext));
	rc |= rbx_state_io(ctx, "r2428", s_ram242800, sizeof(s_ram242800));
	rc |= rbx_state_io(ctx, "r24c8", s_ram24c880, sizeof(s_ram24c880));
	for (int i = 0; i < 3; i++) {   // (the chips' RAM and registers)
		static const char *const names[3] = { "bac0", "bac1", "bac2" };
		rc |= rbx_state_io(ctx, names[i], &s_tile[i], offsetof(bac06_t, gfx8));
	}
	rc |= rbx_state_io(ctx, "i8751p", s_i8751_ports, sizeof(s_i8751_ports));
	if (s_has_mcu)
		rc |= rbx_state_io(ctx, "mcu", &s_mcu.st, sizeof(s_mcu.st));
	if (s_has_sub) {
		uint8_t sub[H6280_STATE_SIZE];
		if (!load)
			memcpy(sub, &s_sub, H6280_STATE_SIZE);
		int32_t sm[5] = { s_mb_intl, s_mb_intr, s_prot_msb, s_prot_lsb, 0 };
		uint64_t sf = s_sub_frac;
		rc |= rbx_state_io(ctx, "h6280", sub, H6280_STATE_SIZE);
		rc |= rbx_state_io(ctx, "subram", s_sub_ram, sizeof(s_sub_ram));
		rc |= rbx_state_io(ctx, "mb8421", s_mb, sizeof(s_mb));
		rc |= rbx_state_io(ctx, "shared", s_shared, sizeof(s_shared));
		rc |= rbx_state_io(ctx, "submisc", sm, sizeof(sm));
		rc |= rbx_state_io(ctx, "subfrac", &sf, sizeof(sf));
		if (!rc && load) {
			memcpy(&s_sub, sub, H6280_STATE_SIZE);
			s_mb_intl = sm[0];
			s_mb_intr = sm[1];
			s_prot_msb = (uint8_t)sm[2];
			s_prot_lsb = (uint8_t)sm[3];
			s_sub_frac = sf;
		}
	}
	rc |= rbx_state_io(ctx, "m6502", snd, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_state_io(ctx, "oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	rc |= rbx_state_io(ctx, "rot", s_rotary, sizeof(s_rotary));
	rc |= rbx_state_io(ctx, "tb", s_tb, sizeof(s_tb));
	rc |= rbx_state_io(ctx, "tbs", s_tb_start, sizeof(s_tb_start));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		memcpy(&s_snd, snd, M6502_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_mcu_frac = fr[2];
		s_ym1_frac = fr[3];
		s_ym2_frac = fr[4];
		s_frame = (uint32_t)misc[0];
		s_pri = (uint16_t)misc[1];
		s_i8751_return = (uint16_t)misc[2];
		s_i8751_command = (uint16_t)misc[3];
		s_irq5_hold = misc[4];
		s_latch = (uint8_t)misc[5];
		s_latch_pending = misc[6];
		s_ym_irq[0] = misc[7];
		s_ym_irq[1] = misc[8];
		s_oki_acc = (uint32_t)misc[9];
		s_oki_hold = misc[10];
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int dc_save(void *ctx) { return dc_state(ctx, 0); }
static int dc_load(void *ctx) { return dc_state(ctx, 1); }
static int dc_supported(const char *machine)
{
	return !strcmp(machine, "hbarrel") || !strcmp(machine, "baddudes") || !strcmp(machine, "drgninjab") ||
	       !strcmp(machine, "robocopb") || !strcmp(machine, "robocop") || !strcmp(machine, "hippodrm") ||
	       !strcmp(machine, "birdtry") || !strcmp(machine, "bandit");
	// (ffantasybl: a black screen)
}

// the dec1 boards (dec1.c: Sly Spy, Boulder Dash, Midnight Resistance) through this driver: the game table is dec0.cpp's
extern const rbx_driver rbx_drv_dec1;
static int s_d1;
static int d0_init(const rbx_game *g)
{
	s_d1 = rbx_drv_dec1.supported(g->machine);
	return s_d1 ? rbx_drv_dec1.init(g) : dc_init(g);
}
static void d0_exit(void) { if (s_d1) rbx_drv_dec1.exit(); else dc_exit(); }
static void d0_reset(void) { if (s_d1) rbx_drv_dec1.reset(); else dc_reset(); }
static void d0_frame(void) { if (s_d1) rbx_drv_dec1.run_frame(); else dc_frame(); }
static int d0_save(void *ctx) { return s_d1 ? rbx_drv_dec1.state_save(ctx) : dc_save(ctx); }
static int d0_load(void *ctx) { return s_d1 ? rbx_drv_dec1.state_load(ctx) : dc_load(ctx); }
static void d0_render(void) { if (s_d1) rbx_drv_dec1.render(); else dc_render(); }
static int d0_supported(const char *machine) { return dc_supported(machine) || rbx_drv_dec1.supported(machine); }

const rbx_driver rbx_drv_dec0 = { "DECO dec0", d0_init, d0_exit, d0_reset, d0_frame, d0_save, d0_load,
                                  d0_supported, d0_render, NULL };
