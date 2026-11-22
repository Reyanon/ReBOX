/*  SNK / Alpha Denshi Alpha 68K I and N boards: Paddle Mania, The Next Space (I); Super Stingray, Kyros and Mahjong
    Block Jongbou (N). Ported from MAME src/mame/alpha/alpha68k_i.cpp, alpha68k_n.cpp, alpha68k_v.cpp
    (alpha68k_prom_state::palette_init) and alpha68k.cpp (BSD-3-Clause, copyright-holders: Pierpaolo Prazzoli,
    Bryan McPhail, Stephane Humbert, Angelo Salese).
    One 68000 and a sound Z80; columns of 32 8x8 sprite tiles, colors through PROM lookup tables (no tile layers).
    I: YM3812. N: YM2203, two AY-3-8910 and an 8-bit DAC (Jongbou: one AY); Kyros and Jongbou with the Alpha MCU
    simulated as MAME does, Super Stingray with its 8748 MCU (the "Alpha-8511") emulated, the 68000 halted while it works.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/mcs48/mcs48.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/ay8910.h"

#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 59186            // set_raw(24 MHz / 4, 384, 0, 256, 264, 16, 240)
#define LINES 264
#define VIS_Y0 16
#define VIS_W 256
#define VIS_H 224
#define MCU_CLOCK 9263750          // the 8748 (divided by 15 inside: machine cycles)
#define AY_CLOCK 2000000

enum { G_PADDLEMA, G_TNEXTSPC, G_SSTINGRY, G_KYROS, G_JONGBOU };

static int s_game, s_board_n;     // s_board_n: the N board (else I)
static uint32_t s_cpu_clock, s_z80_clock;
static int s_irq2_hz, s_yshift, s_transchar, s_bankshift, s_stingray, s_npens, s_ncolors, s_nay;
static uint32_t s_mcu_id, s_coin_id, s_zper;   // s_zper: the Z80's periodic NMI / IRQ period in cycles, 16.16
static int s_zper_irq;            // the periodic event is an IRQ (Jongbou), else an NMI
static uint8_t *s_rom, *s_zrom, *s_mrom;
static uint32_t s_rom_size, s_zrom_size;
static const uint8_t *s_cprom;    // color_proms (sprite colors)
static uint32_t s_cprom_size;
static uint32_t s_rgb[256], s_pens[1024];
static int s_port[5], s_ay_div;

// graphics: up to 8 banks (gfx elements) of 8x8 tiles, one byte per pixel
static uint8_t *s_gfx_pix, *s_gfx_used;
static uint32_t s_gfx_base[8], s_gfx_count[8];
static int s_gfx_banks;

// machine state (saved: everything up to state_end)
static struct {
	uint16_t ram[0x2000];          // I: work RAM (0x4000 bytes); N: the shared RAM (0x1000 bytes)
	uint16_t spr[0x2000];          // sprite RAM
	uint16_t vram;                 // N: 0x060000 (LSB: the background color)
	uint8_t zram[0x800];
	uint8_t latch, latch_pending, flip, dac, zfc00;   // zfc00: Paddle Mania's Z80 0xfc00 (one byte of RAM)
	int ym_irq, zline;
	// the Alpha MCU simulation
	uint32_t trigstate, deposits1, deposits2, credits, coinvalue, mcu_data;
	int mcu_latch;
	// Super Stingray's 8748
	uint8_t mcu_addr, mcu_control, mcu_read_mode, halted, sync_pending;
	uint32_t sync_param;
	int64_t mcu_acc;
	uint8_t dial[2];
	// timing
	uint64_t cpu_frac, z80_frac, ym_frac, ay_frac, irq2_acc;
	uint32_t zper_f;
	int32_t zper_left;             // Z80 cycles to the next periodic event
	uint8_t state_end;
} *s_m;

static struct {
	uint16_t spr[0x1000];
	uint16_t vram;
	uint8_t flip;
} *s_snap;

static z80 s_z80;
static mcs48 s_mcu;
static ay8910_t s_ay[2];
static int32_t *s_mix, *s_aybuf;
static int s_mix_n, s_mix_cap, s_ay_n, s_ay_cap;
static const uint8_t *s_zmap[256];
static uint16_t (*s_frame_hp)[VIS_H][VIS_W];
#define s_frame (*s_frame_hp)

static uint8_t in_port(int i) { return (uint8_t)rbx_port_read(s_port[i]); }

// ---- sound ----
static void zline_update(void)
{
	if (!s_board_n)   // (the latch's data pending, HOLD_LINE until it is read, and the YM3812's IRQ on line 0)
		s_m->zline = s_m->ym_irq || (s_game == G_PADDLEMA && s_m->latch_pending);
}
static void ym_irq(int state)
{
	s_m->ym_irq = state;
	zline_update();
}
static uint8_t latch_r(void)
{
	s_m->latch_pending = 0;
	zline_update();
	return s_m->latch;
}

static uint8_t z80_read(void *u, uint16_t a)
{
	(void)u;
	switch (s_game) {
	case G_PADDLEMA:
		if (a < 0xa000) return a < s_zrom_size ? s_zrom[a] : 0;
		if (a == 0xe000) return latch_r();
		if (a == 0xe800) return rbx_ym_read(0, 0);
		if (a >= 0xf000 && a < 0xf800) return s_m->zram[a - 0xf000];
		if (a == 0xfc00) return s_m->zfc00;
		return 0;
	case G_TNEXTSPC:
		if (a < 0xf000) return a < s_zrom_size ? s_zrom[a] : 0;
		if (a < 0xf800) return s_m->zram[a - 0xf000];
		if (a == 0xf800) return latch_r();
		return 0;
	case G_KYROS:
		if (a < 0xc000) return a < s_zrom_size ? s_zrom[a] : 0;
		if (a < 0xc800) return s_m->zram[a - 0xc000];
		if (a == 0xe000) return latch_r();
		return 0;
	case G_SSTINGRY:
		if (a < 0x8000) return a < s_zrom_size ? s_zrom[a] : 0;
		if (a < 0x8800) return s_m->zram[a - 0x8000];
		if (a == 0xc100) return latch_r();
		return 0;
	default:   // Jongbou
		if (a < 0x8000) return a < s_zrom_size ? s_zrom[a] : 0;
		if (a < 0x8400) return s_m->zram[a - 0x8000];
		return 0;
	}
}
static void z80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	switch (s_game) {
	case G_PADDLEMA:
		if (a == 0xe000) s_m->latch = 0;   // (clear_w)
		else if (a == 0xe800) rbx_ym_write(0, 0, v);
		else if (a == 0xec00) rbx_ym_write(0, 1, v);
		else if (a >= 0xf000 && a < 0xf800) s_m->zram[a - 0xf000] = v;
		else if (a == 0xfc00) s_m->zfc00 = v;
		break;
	case G_TNEXTSPC:
		if (a >= 0xf000 && a < 0xf800) s_m->zram[a - 0xf000] = v;
		else if (a == 0xf800) s_m->latch = 0;
		break;
	case G_KYROS:
	case G_SSTINGRY: {
		const uint16_t io = s_game == G_KYROS ? 0xe000 : 0xc100;
		if (s_game == G_KYROS ? a >= 0xc000 && a < 0xc800 : a >= 0x8000 && a < 0x8800)
			s_m->zram[a & 0x7ff] = v;
		else if (a == io + 2) s_m->latch = 0;
		else if (a == io + 4) s_m->dac = v;
		break;
	}
	default:
		if (a >= 0x8000 && a < 0x8400) s_m->zram[a - 0x8000] = v;
		break;
	}
}
static uint8_t z80_in(z80 *z, uint8_t port)
{
	(void)z;
	if (s_game == G_TNEXTSPC && port == 0x00) return rbx_ym_read(0, 0);
	if (s_game == G_JONGBOU && port == 0x01) return ay8910_data_r(&s_ay[0]);
	return 0;
}
static void z80_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	switch (s_game) {
	case G_TNEXTSPC:
		if (port == 0x00) rbx_ym_write(0, 0, v);
		else if (port == 0x20) rbx_ym_write(0, 1, v);
		break;
	case G_KYROS:
	case G_SSTINGRY:
		switch (port) {
		case 0x10: case 0x11: rbx_ym_write(0, port & 1, v); break;
		case 0x80: ay8910_data_w(&s_ay[0], v); break;
		case 0x81: ay8910_address_w(&s_ay[0], v); break;
		case 0x90: ay8910_data_w(&s_ay[1], v); break;
		case 0x91: ay8910_address_w(&s_ay[1], v); break;
		}
		break;
	case G_JONGBOU:
		if (port == 0x00) ay8910_address_w(&s_ay[0], v);
		else if (port == 0x01) ay8910_data_w(&s_ay[0], v);
		else if (port == 0x02) s_m->latch = 0;
		break;
	}
}
static uint8_t ay_port_a(void *u)
{
	(void)u;
	return latch_r();
}

static void z80_bind(z80 *z)
{
	z->read_byte = z80_read;
	z->write_byte = z80_write;
	z->port_in = z80_in;
	z->port_out = z80_out;
	z->userdata = NULL;
	z->read_map = s_zmap;
}

static void sound_line(void)
{
	const int cycles = rbx_line_cycles(&s_m->z80_frac, s_z80_clock, FRAME_MHZ, LINES);
	const unsigned long end = s_z80.cyc + (unsigned long)cycles;
	if (s_zper) {   // the periodic NMI (Kyros, Super Stingray) or IRQ (Jongbou)
		while ((long)(s_z80.cyc + (unsigned long)s_m->zper_left - end) <= 0) {
			const unsigned long t = s_z80.cyc + (unsigned long)s_m->zper_left;
			z80_run(&s_z80, t, &s_m->zline, s_board_n);
			if (s_zper_irq)
				s_m->zline = 1;
			else
				z80_gen_nmi(&s_z80);
			s_m->zper_f += s_zper;
			s_m->zper_left = (int32_t)(t - s_z80.cyc) + (int32_t)(s_m->zper_f >> 16);
			s_m->zper_f &= 0xffff;
		}
		const unsigned long t0 = s_z80.cyc;
		z80_run(&s_z80, end, &s_m->zline, s_board_n);
		s_m->zper_left -= (int32_t)(s_z80.cyc - t0);
	} else
		z80_run(&s_z80, end, &s_m->zline, s_board_n);
	int n = 0;
	if (s_game != G_JONGBOU)
		n = rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&s_m->ym_frac, s_board_n ? AY_CLOCK : 4000000, FRAME_MHZ, LINES),
		                   s_mix + s_mix_n, s_mix_cap - s_mix_n);
	if (s_nay) {
		const uint32_t ac = (uint32_t)rbx_line_cycles(&s_m->ay_frac, AY_CLOCK, FRAME_MHZ, LINES);
		int32_t *dst = s_game == G_JONGBOU ? s_mix + s_mix_n : s_aybuf + s_ay_n;
		const int room = s_game == G_JONGBOU ? s_mix_cap - s_mix_n : s_ay_cap - s_ay_n;
		int k = ay8910_render(&s_ay[0], ac, dst, room);
		if (s_nay > 1)
			ay8910_render(&s_ay[1], ac, dst, room);
		if (s_game == G_JONGBOU)
			n = k;
		else
			s_ay_n += k;
	}
	if (s_board_n && s_game != G_JONGBOU) {
		const int32_t dac = ((int32_t)s_m->dac - 0x80) * 128;   // (0.50 of +-1)
		for (int i = 0; i < n; i++)
			s_mix[s_mix_n + i] += dac;
	}
	s_mix_n += n;
}

static void sound_frame_end(void)
{
	if (s_game != G_JONGBOU)
		rbx_ym_async_take(0, s_mix, s_mix_n);
	if (s_ay_n > 0 && s_mix_n > 0)
		for (int i = 0; i < s_mix_n; i++)
			s_mix[i] += s_aybuf[(int)((int64_t)i * s_ay_n / s_mix_n)];
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	memset(s_aybuf, 0, sizeof(int32_t) * (size_t)s_ay_n);
	s_mix_n = s_ay_n = 0;
}

// ---- the Alpha MCU simulation (kyros_alpha_trigger_r) ----
static void kyros_trigger(uint32_t offset)
{
	static const uint8_t c1[8][2] = { {1,1}, {1,5}, {1,3}, {2,3}, {1,2}, {1,6}, {1,4}, {3,2} };
	static const uint8_t c2[8][2] = { {1,1}, {5,1}, {3,1}, {7,1}, {2,1}, {6,1}, {4,1}, {8,1} };
	uint16_t *ram = s_m->ram;
	const uint16_t source = ram[offset];
	switch (offset) {
	case 0x22:   // the coin value
		ram[0x22] = (uint16_t)((source & 0xff00) | (s_m->credits & 0xff));
		break;
	case 0x29: {   // a coin inserted?
		const uint8_t in2 = in_port(2);
		s_m->trigstate++;
		if ((in2 & 3) == 3)
			s_m->mcu_latch = 0;
		if (((in2 & 1) == 0 || (in2 & 2) == 0) && !s_m->mcu_latch) {
			const int b = (in2 & 1) != 0;   // coin B
			ram[0x29] = (uint16_t)((source & 0xff00) | (b ? s_coin_id >> 8 : s_coin_id & 0xff));
			ram[0x22] = (uint16_t)(source & 0xff00);
			s_m->mcu_latch = 1;
			s_m->coinvalue = (~in_port(1) >> 1) & 7;
			uint32_t *dep = b ? &s_m->deposits2 : &s_m->deposits1;
			const uint8_t (*c)[2] = b ? c2 : c1;
			if (++*dep == c[s_m->coinvalue][0]) {
				s_m->credits = c[s_m->coinvalue][1];
				*dep = 0;
			} else
				s_m->credits = 0;
		} else {
			s_m->mcu_data = 0;
			if (s_mcu_id == 0x00ff || s_game == G_JONGBOU) {
				if (s_m->trigstate >= 12 || s_game == G_JONGBOU) {
					s_m->trigstate = 0;
					s_m->mcu_data = 0x21;   // (the timer)
				}
			}
			ram[0x29] = (uint16_t)((source & 0xff00) | s_m->mcu_data);
		}
		break;
	}
	case 0xff:   // the ID, at boot
		ram[0xff] = (uint16_t)((source & 0xff00) | (s_mcu_id & 0xff));
		break;
	}
}

// ---- Super Stingray's 8748 ----
static void mcu_command(uint32_t offset, uint32_t data)   // alpha8511_command_r / _w: the sync timer
{
	s_m->sync_param = offset << 8 | data;
	s_m->sync_pending = 1;
	m68k_end_timeslice();
}
static void mcu_sync(void)
{
	const uint32_t p = s_m->sync_param;
	s_m->sync_pending = 0;
	s_m->mcu_data = p & 0xff;
	s_m->mcu_addr = (uint8_t)(p >> 8);
	s_m->mcu_read_mode = (p >> 16) & 1;
	if (s_m->mcu_control & 0x10) {
		mcs48_set_irq(&s_mcu, 1);
		s_m->halted = 1;
	}
}
static uint8_t mcu_port_r(void *u, int port)
{
	(void)u;
	return port == 1 ? s_m->mcu_addr : s_m->mcu_read_mode ? 0xff : 0xf7;
}
static void mcu_port_w(void *u, int port, uint8_t data)
{
	(void)u;
	if (port != 2)
		return;
	if (!(data & 0x10)) {
		mcs48_set_irq(&s_mcu, 0);
		s_m->halted = 0;
	}
	if (data & 0x20)
		m68k_bus_hold_irq(2);
	if (!(data & 0x40))
		s_m->ram[s_m->mcu_addr] = (uint16_t)((s_m->ram[s_m->mcu_addr] & 0xff00) | (s_m->mcu_data & 0xff));
	s_m->flip = !(data & 0x80);
	s_m->mcu_control = data;
}
static int mcu_test_r(void *u, int t)
{
	(void)u;
	return (in_port(2) >> t) & 1;
}
static uint8_t mcu_bus_r(void *u)
{
	(void)u;
	return (uint8_t)s_m->mcu_data;
}
static void mcu_bus_w(void *u, uint8_t v)
{
	(void)u;
	s_m->mcu_data = v;
}
static void mcu_bind(void)
{
	s_mcu.rom = s_mrom;
	s_mcu.rom_mask = 0x3ff;
	s_mcu.userdata = NULL;
	s_mcu.port_r = mcu_port_r;
	s_mcu.port_w = mcu_port_w;
	s_mcu.test_r = mcu_test_r;
	s_mcu.bus_r = mcu_bus_r;
	s_mcu.bus_w = mcu_bus_w;
}
static void mcu_advance(int cpu_cycles)   // the 8748 kept up with the 68000
{
	s_m->mcu_acc += (int64_t)cpu_cycles * MCU_CLOCK;
	const int64_t unit = (int64_t)s_cpu_clock * 15;
	const int64_t want = s_m->mcu_acc / unit;
	if (want > 0)
		s_m->mcu_acc -= (int64_t)mcs48_execute(&s_mcu, (int)want) * unit;
}

// ---- bus ----
static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	switch (s_game) {
	case G_PADDLEMA:
		switch (a) {
		case 0x180000: return in_port(3);
		case 0x180008: return in_port(4);
		case 0x300000: return (uint16_t)rbx_port_read(s_port[0]);
		case 0x340000: return (uint16_t)rbx_port_read(s_port[1]);
		case 0x380000: return (uint16_t)rbx_port_read(s_port[2]);
		}
		return 0;
	case G_TNEXTSPC:
		switch (a) {
		case 0x0e0000: return in_port(0);
		case 0x0e0002: return in_port(1);
		case 0x0e0004: return in_port(2);
		case 0x0e0008: return in_port(3);
		case 0x0e000a: return in_port(4);
		case 0x0e0018: return 1;   // (sound_cpu_r)
		}
		return 0;
	default:
		if (a == 0x060000) return s_m->vram;
		if (a == 0x0c0000) {
			if (s_game != G_JONGBOU)
				return (uint16_t)rbx_port_read(s_port[0]);
			const uint8_t d1 = s_m->dial[0], d2 = s_m->dial[1];   // dial_inputs_r: the bits reversed
			const uint8_t i1 = (uint8_t)(((d1 & 1) << 3) | ((d1 & 2) << 1) | ((d1 & 4) >> 1) | ((d1 & 8) >> 3));
			const uint8_t i2 = (uint8_t)(((d2 & 1) << 3) | ((d2 & 2) << 1) | ((d2 & 4) >> 1) | ((d2 & 8) >> 3));
			return (uint16_t)(rbx_port_read(s_port[0]) | i1 | i2 << 4);
		}
		if (a == 0x0e0000) return (uint16_t)(in_port(1) << 8);
		if (a >= 0x080000 && a < 0x080200 && !s_stingray)
			kyros_trigger((a - 0x080000) >> 1);
		return 0;
	}
}
static uint8_t main_read8(uint32_t a)
{
	a &= 0xffffff;
	if (s_stingray && a >= 0x080000 && a < 0x080200) {   // (umask 0x00ff)
		if (a & 1)
			mcu_command((a - 0x080000) >> 1, 0x100ff);
		return 0;
	}
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static uint16_t main_read16w(uint32_t a)
{
	if (s_stingray && (a & 0xffffff) >= 0x080000 && (a & 0xffffff) < 0x080200)
		return main_read8(a | 1);
	return main_read16(a);
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	switch (s_game) {
	case G_PADDLEMA:
		if (a == 0x380000 && (mask & 0x00ff)) {
			s_m->latch = (uint8_t)v;
			s_m->latch_pending = 1;
			zline_update();
		}
		return;
	case G_TNEXTSPC:
		if (a == 0x0f0000) {
			s_m->flip = (v >> 8) & 1;
		} else if (a == 0x0f0008 && (mask & 0x00ff)) {
			s_m->latch = (uint8_t)v;
			z80_gen_nmi(&s_z80);
		}
		return;
	default:
		if (a == 0x060000)
			s_m->vram = (uint16_t)((s_m->vram & ~mask) | (v & mask));
		else if (a == 0x0e0000 && (mask & 0xff00))
			s_m->latch = (uint8_t)(v >> 8);
		else if (a >= 0x080000 && a < 0x080200) {
			const uint32_t o = (a - 0x080000) >> 1;
			if (s_stingray) {
				if (mask & 0x00ff)
					mcu_command(o, v & 0xff);
			} else if (o == 0x2d && (mask & 0x00ff))   // alpha_microcontroller_w
				s_m->flip = v & 1;
		}
		return;
	}
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << 8 | v), (uint16_t)(0xff << sh));
}

// ---- video ----
static void draw_tile(int bank, uint32_t code, int color, int fx, int fy, int sx, int sy)
{
	if (bank >= s_gfx_banks)
		return;
	code %= s_gfx_count[bank];
	const uint32_t idx = s_gfx_base[bank] + code;
	if (!s_gfx_used[idx])
		return;
	const uint8_t *g = s_gfx_pix + idx * 64;
	const int depth = s_board_n ? 8 : 16;
	const uint16_t c0 = (uint16_t)((color % s_ncolors) * depth);
	sy -= VIS_Y0;
	for (int j = 0; j < 8; j++) {
		const int yy = sy + j;
		if ((unsigned)yy >= VIS_H)
			continue;
		const uint8_t *src = g + (fy ? 7 - j : j) * 8;
		uint16_t *d = s_frame[yy];
		for (int k = 0; k < 8; k++) {
			const int xx = sx + k;
			if ((unsigned)xx >= VIS_W)
				continue;
			const uint8_t p = src[fx ? 7 - k : k];
			if (p)
				d[xx] = (uint16_t)(c0 + p);
		}
	}
}

static void draw_sprites_i(int c, int d)
{
	const uint16_t *sp = s_snap->spr;
	for (int offs = 0; offs < 0x400; offs += 0x20) {
		int mx = sp[offs + c];
		int my = (s_yshift - (mx >> 8)) & 0xff;
		mx &= 0xff;
		for (int i = 0; i < 0x20; i++) {
			const uint16_t data = sp[offs + d + i];
			const uint32_t tile = data & 0x3fff;
			const uint32_t ci = (tile << 1 | data >> 15) % s_cprom_size;
			draw_tile(0, tile, s_cprom[ci], 0, (data & 0x4000) != 0, mx, my);
			my = (my + 8) & 0xff;
		}
	}
}

static void draw_sprites_n(int c, int d)
{
	const uint16_t *sp = s_snap->spr;
	const int flip = s_snap->flip;
	for (int offs = 0; offs < 0x400; offs += 0x20) {
		int mx = sp[offs + c];
		int my = -(mx >> 8) & 0xff;
		mx &= 0xff;
		if (s_stingray && mx > 0xf8)
			mx -= 0x100;
		if (flip)
			my = 249 - my;
		for (int i = 0; i < 0x20; i++) {
			const uint16_t data = sp[offs + d + i];
			if (data != s_transchar) {
				int color, bank = (data >> 10) & 3;
				uint32_t tile = data & 0x3ff;
				if (s_stingray)
					color = ((data >> 7) & 0x18) | ((data >> 13) & 7);
				else {
					const uint32_t ci = (uint32_t)(((data >> 1) & 0x1000) | (data & 0xffc) | ((data >> 14) & 3));
					color = s_cprom && ci < s_cprom_size ? s_cprom[ci] : 0;
					bank += (data >> s_bankshift) & 4;
					tile += (data >> 3) & 0x400;
				}
				if (color != 0xff || !s_stingray)
					draw_tile(bank, tile, color, flip, ((data & 0x1000) >> 12) ^ flip, mx, my);
			}
			my = flip ? (my - 8) & 0xff : (my + 8) & 0xff;
		}
	}
}

static void an_render(void)
{
	memcpy(rbx_video.palette, s_pens, sizeof(uint32_t) * (size_t)(s_npens));
	const uint16_t bg = (uint16_t)s_npens;   // I: black; N: the background color register
	rbx_video.palette[bg] = s_board_n ? s_rgb[s_snap->vram & 0xff] : 0;
	for (int y = 0; y < VIS_H; y++)
		for (int x = 0; x < VIS_W; x++)
			s_frame[y][x] = bg;
	if (s_board_n) {
		draw_sprites_n(2, 0x0800);
		draw_sprites_n(3, 0x0c00);
		draw_sprites_n(1, 0x0400);
	} else {
		draw_sprites_i(2, 0x0800);
		draw_sprites_i(3, 0x0c00);
		draw_sprites_i(1, 0x0400);
	}
	for (int y = 0; y < VIS_H; y++)
		memcpy(rbx_video.bitmap + y * rbx_video.pitch, s_frame[y], VIS_W * 2);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spr, s_m->spr, sizeof(s_snap->spr));
	s_snap->vram = s_m->vram;
	s_snap->flip = s_m->flip;
	rbx_video_frame_done();
}

// gfx_layout decoding to one byte per pixel (8x8; bit offsets as in MAME, plane 0 the most significant)
static void gfx_decode(int bank, const uint8_t *rom, uint32_t size, uint32_t start, uint32_t count, int planes,
                       const uint32_t *po, const uint32_t *xo, const uint32_t *yo, uint32_t inc)
{
	const uint32_t base = bank ? s_gfx_base[bank - 1] + s_gfx_count[bank - 1] : 0;
	s_gfx_base[bank] = base;
	s_gfx_count[bank] = count;
	for (uint32_t n = 0; n < count; n++) {
		uint8_t *dst = s_gfx_pix + (base + n) * 64;
		uint8_t any = 0;
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				uint8_t p = 0;
				for (int pl = 0; pl < planes; pl++) {
					const uint32_t bit = start * 8 + n * inc + po[pl] + yo[y] + xo[x];
					const int v = (bit >> 3) < size ? (rom[bit >> 3] >> (7 - (bit & 7))) & 1 : 0;
					p = (uint8_t)(p | v << (planes - 1 - pl));
				}
				dst[y * 8 + x] = p;
				any |= p;
			}
		s_gfx_used[base + n] = any != 0;
	}
}

// ---- init / frame ----
static void an_reset(void)
{
	memset(s_m, 0, offsetof(__typeof__(*s_m), state_end));
	s_m->dac = 0x80;
	s_m->mcu_control = 0;
	z80_init(&s_z80);
	z80_bind(&s_z80);
	s_z80.int_data = 0xff;
	if (s_zper) {
		s_m->zper_f = s_zper & 0xffff;
		s_m->zper_left = (int32_t)(s_zper >> 16);
	}
	if (s_game != G_JONGBOU)
		rbx_ym_reset(0);
	for (int i = 0; i < s_nay; i++)
		ay8910_reset(&s_ay[i]);
	if (s_stingray) {
		mcs48_reset(&s_mcu);
		mcs48_set_irq(&s_mcu, 0);
	}
	s_mix_n = s_ay_n = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
}

static void an_exit(void);

static int an_init(const rbx_game *g)
{
	const char *m = g->machine;
	s_game = !strcmp(m, "paddlema") ? G_PADDLEMA : !strcmp(m, "tnextspc") ? G_TNEXTSPC
	       : !strcmp(m, "sstingry") ? G_SSTINGRY : !strcmp(m, "kyros") ? G_KYROS : G_JONGBOU;
	s_board_n = s_game >= G_SSTINGRY;
	s_stingray = s_game == G_SSTINGRY;
	s_cpu_clock = s_game == G_TNEXTSPC ? 9000000 : s_game == G_JONGBOU ? 8000000 : 6000000;
	s_z80_clock = s_game == G_SSTINGRY ? 3579545 : 4000000;
	s_irq2_hz = s_game == G_KYROS ? 60 : s_game == G_JONGBOU ? 960 : 0;
	s_yshift = s_game == G_TNEXTSPC ? 1 : 0;
	s_transchar = s_stingray ? 0x40 : 0x20;
	s_bankshift = s_game == G_KYROS ? 13 : 11;
	s_nay = s_game == G_KYROS || s_stingray ? 2 : s_game == G_JONGBOU ? 1 : 0;
	s_zper_irq = s_game == G_JONGBOU;
	s_zper = s_board_n ? (uint32_t)((double)s_z80_clock * 65536.0 / (s_zper_irq ? 9600.0 : 4000.0) + 0.5) : 0;
	s_mcu_id = s_game == G_KYROS || !strcmp(g->set, "jongbou2") ? 0x0012 : 0x00ff;
	s_coin_id = s_game == G_JONGBOU ? 0x23 | 0x24 << 8 : 0x22 | 0x22 << 8;

	if (!s_frame_hp && !(s_frame_hp = calloc(1, sizeof(*s_frame_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t ss, ps, cs, ms;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *spr = rbx_region("sprites", &ss);
	const uint8_t *prom = rbx_region("proms", &ps), *clut = rbx_region("clut_proms", &cs);
	s_cprom = rbx_region_opt("color_proms", &s_cprom_size);
	s_mrom = s_stingray ? rbx_region("alpha8511", &ms) : NULL;
	if (!s_rom || s_rom_size < 0x20000 || !s_zrom || !spr || !prom || ps < 0x300 || !clut || (!s_board_n && !s_cprom)
	    || (s_stingray && !s_mrom)) {
		rbx_fail("%s: ROM regions missing", g->set);
		an_exit();
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	// palette_init: 256 colors from the RGB PROMs, the pens through the CLUT PROMs
	for (int i = 0; i < 256; i++)
		s_rgb[i] = (uint32_t)((prom[i] & 15) * 0x11) << 16 | (uint32_t)((prom[i + 0x100] & 15) * 0x11) << 8
		         | (uint32_t)((prom[i + 0x200] & 15) * 0x11);
	s_npens = (int)(cs / 2);
	if (s_npens > 1024)
		s_npens = 1024;
	for (int i = 0; i < s_npens; i++)
		s_pens[i] = s_rgb[((clut[i + cs / 2] & 0x0f) << 4) | (clut[i] & 0x0f)];
	s_ncolors = s_board_n ? 32 : 64;
	{
		static const uint32_t xi[8] = { 131, 130, 129, 128, 3, 2, 1, 0 }, pi[4] = { 0, 4, 8, 12 };
		static const uint32_t xn[8] = { 8 * 8 + 3, 8 * 8 + 2, 8 * 8 + 1, 8 * 8 + 0, 3, 2, 1, 0 };
		uint32_t yi[8], yn[8];
		for (int i = 0; i < 8; i++) {
			yi[i] = (uint32_t)i * 16;
			yn[i] = (uint32_t)i * 8;
		}
		uint32_t total;
		if (!s_board_n)
			total = ss / 32;
		else
			total = s_game == G_KYROS ? 8 * 2048 : s_stingray ? 4 * 1024 : 8 * 1024;
		s_gfx_pix = (uint8_t *)malloc((size_t)total * 64);
		s_gfx_used = (uint8_t *)malloc(total);
		if (!s_gfx_pix || !s_gfx_used) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			an_exit();
			return -1;
		}
		if (!s_board_n) {   // charlayout
			s_gfx_banks = 1;
			gfx_decode(0, spr, ss, 0, total, 4, pi, xi, yi, 32 * 8);
		} else if (s_game == G_KYROS) {   // gfx_kyros
			static const uint32_t l1[3] = { 4, 0x8000 * 8, 0x8000 * 8 + 4 }, l2[3] = { 0, 0x10000 * 8, 0x10000 * 8 + 4 };
			s_gfx_banks = 8;
			for (int b = 0; b < 8; b++)
				gfx_decode(b, spr, ss, (uint32_t)(b >> 1) * 0x18000, 2048, 3, b & 1 ? l2 : l1, xn, yn, 16 * 8);
		} else if (s_stingray) {   // gfx_sstingry
			static const uint32_t l1[3] = { 4, 0x10000 * 4, 4 + 0x8000 * 8 }, l2[3] = { 0, 0x28000 * 8, 4 + 0x28000 * 8 };
			static const uint32_t l3[3] = { 0, 0x10000 * 8, 4 + 0x10000 * 8 };
			s_gfx_banks = 4;
			gfx_decode(0, spr, ss, 0, 1024, 3, l1, xn, yn, 16 * 8);
			gfx_decode(1, spr, ss, 0, 1024, 3, l2, xn, yn, 16 * 8);
			gfx_decode(2, spr, ss, 0x10000, 1024, 3, l1, xn, yn, 16 * 8);
			gfx_decode(3, spr, ss, 0x10000, 1024, 3, l3, xn, yn, 16 * 8);
		} else {   // gfx_jongbou
			static const uint32_t l1[3] = { 4, 0x8000 * 8, 4 + 0x8000 * 8 }, l2[3] = { 0, 0x10000 * 8, 4 + 0x10000 * 8 };
			static const uint32_t st[4] = { 0, 0x18000, 0x04000, 0x1c000 };
			s_gfx_banks = 8;
			for (int b = 0; b < 8; b++)
				gfx_decode(b, spr, ss, st[b >> 1], 1024, 3, b & 1 ? l2 : l1, xn, yn, 16 * 8);
		}
	}
	s_m = calloc(1, sizeof(*s_m));
	s_snap = calloc(1, sizeof(*s_snap));
	int rc = !s_m || !s_snap;
	uint32_t rate = 0;
	if (!rc && s_game != G_JONGBOU) {
		rbx_ym_config yc;
		memset(&yc, 0, sizeof(yc));
		if (s_board_n) {   // YM2203 0.30
			yc.type = RBX_YM2203;
			yc.clock = AY_CLOCK;
			yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 77;
		} else {           // YM3812 1.0
			yc.type = RBX_YM3812;
			yc.clock = 4000000;
			yc.gain[0] = 256;
			yc.irq = ym_irq;
		}
		rc = rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
		if (!rc)
			rate = rbx_ym_rate(0);
	}
	if (!rc && s_nay) {
		static const int gains[3][2] = { { 77, 115 }, { 77, 154 }, { 166, 0 } };   // (sstingry, kyros, jongbou)
		const int gi = s_stingray ? 0 : s_game == G_KYROS ? 1 : 2;
		s_ay_div = s_game == G_JONGBOU ? 6 : 8;
		for (int i = 0; i < s_nay; i++)
			ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, gains[gi][i], s_ay_div, 0);
		if (s_game == G_JONGBOU) {
			s_ay[0].port_a_read = ay_port_a;
			rate = ay8910_rate(&s_ay[0]);
		}
		s_ay_cap = (int)(ay8910_rate(&s_ay[0]) / 50) + 256;
	}
	if (!rc) {
		s_mix_cap = (int)(rate / 50) + 256;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		s_aybuf = (int32_t *)calloc((size_t)(s_ay_cap ? s_ay_cap : 1), sizeof(int32_t));
		rc = !s_mix || !s_aybuf || rbx_audio_setup(rate) || rbx_video_setup(VIS_W, VIS_H, s_npens + 1);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		an_exit();
		return -1;
	}
	// the Z80's read map: its ROM (I: up to 0x9fff / 0xefff, N: 0xbfff / 0x7fff)
	memset(s_zmap, 0, sizeof(s_zmap));
	const uint32_t zrom_end = s_game == G_PADDLEMA ? 0xa000 : s_game == G_TNEXTSPC ? 0xf000 : s_game == G_KYROS ? 0xc000 : 0x8000;
	for (uint32_t p = 0; p * 256 < zrom_end && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	if (s_stingray) {
		mcs48_init(&s_mcu, MCS48_I8048);   // (an I8748: the 8048 with EPROM)
		mcu_bind();
	}
	static const char *const n_i[5] = { "IN0", "IN1", "IN2", "IN3", "IN4" };
	static const char *const n_t[5] = { "P1", "P2", "SYSTEM", "DSW1", "DSW2" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(s_game == G_TNEXTSPC ? n_t[i] : n_i[i]);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);
	m68k_bus_reset_map();
	if (!s_board_n) {
		const uint32_t ram = s_game == G_PADDLEMA ? 0x080000 : 0x070000, spr_a = s_game == G_PADDLEMA ? 0x100000 : 0x0a0000;
		m68k_bus_map(0, 0x3ffff, s_rom, s_rom_size < 0x40000 ? s_rom_size : 0x40000, 1, 0);
		m68k_bus_map(ram, ram + 0x3fff, s_m->ram, 0x4000, 1, 1);
		m68k_bus_map(spr_a, spr_a + 0x3fff, s_m->spr, 0x4000, 1, 1);
	} else {
		m68k_bus_map(0, 0x1ffff, s_rom, 0x20000, 1, 0);
		m68k_bus_map(0x020000, 0x020fff, s_m->ram, 0x1000, 1, 1);
		m68k_bus_map(0x040000, 0x041fff, s_m->spr, 0x2000, 1, 1);
	}
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16w;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	an_reset();
	return 0;
}

static void an_exit(void)
{
	rbx_video_wait_idle();
	m68k_set_mame_timing(0);
	rbx_ym_exit(0);
	free(s_mix);
	free(s_aybuf);
	s_mix = s_aybuf = NULL;
	s_mix_cap = s_ay_cap = s_mix_n = s_ay_n = 0;
	free(s_gfx_pix);
	free(s_gfx_used);
	s_gfx_pix = s_gfx_used = NULL;
	s_gfx_banks = 0;
	free(s_snap);
	s_snap = NULL;
	free(s_m);
	s_m = NULL;
	free(s_frame_hp);
	s_frame_hp = NULL;
}

static void run_main(int cycles)
{
	if (!s_stingray) {
		m68k_execute(cycles);
		return;
	}
	int left = cycles;
	while (left > 0) {
		if (s_m->halted) {   // (INPUT_LINE_HALT: the 8748 works alone)
			const int step = left < 32 ? left : 32;
			mcu_advance(step);
			left -= step;
			continue;
		}
		const int ran = m68k_execute(left);
		mcu_advance(ran);
		left -= ran;
		if (s_m->sync_pending)
			mcu_sync();
	}
}

static void an_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	if (s_game == G_JONGBOU) {   // the dials (key delta 20 x sensitivity 50%)
		s_m->dial[0] = (uint8_t)((s_m->dial[0] + rbx_analog_dpad(1, RBX_IPT_DIAL, 10)) & 0x0f);
		s_m->dial[1] = (uint8_t)((s_m->dial[1] + rbx_analog_dpad(2, RBX_IPT_DIAL, 10)) & 0x0f);
	}
	const uint64_t irq2_step = (uint64_t)FRAME_MHZ * LINES;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0 + VIS_H) {   // the vblank: the frame drawn, irq1_line_hold (N: the Z80's irq0_line_hold too)
			vblank_capture();
			m68k_bus_hold_irq(1);
			if (s_board_n && s_game != G_JONGBOU)
				s_m->zline = 1;
		}
		if (s_irq2_hz) {   // the MCU's periodic IRQ 2 (held)
			s_m->irq2_acc += (uint64_t)s_irq2_hz * 1000;
			if (s_m->irq2_acc >= irq2_step) {
				s_m->irq2_acc -= irq2_step;
				m68k_bus_hold_irq(2);
			}
		}
		run_main(rbx_line_cycles(&s_m->cpu_frac, s_cpu_clock, FRAME_MHZ, LINES));
		sound_line();
	}
	sound_frame_end();
}

static int an_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	z80 z = s_z80;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "state", s_m, (uint32_t)offsetof(__typeof__(*s_m), state_end));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	if (s_game != G_JONGBOU)
		rc |= rbx_ym_state(0, ctx, load);
	for (int i = 0; i < s_nay; i++)
		rc |= rbx_state_io(ctx, i ? "ay1" : "ay0", &s_ay[i], ay8910_state_size());
	if (s_stingray)
		rc |= rbx_state_io(ctx, "mcu", (uint8_t *)&s_mcu + MCS48_STATE_OFFSET, (uint32_t)MCS48_STATE_SIZE);
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		z80_bind(&z);
		s_z80 = z;
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int an_save(void *ctx) { return an_state(ctx, 0); }
static int an_load(void *ctx) { return an_state(ctx, 1); }
static int an_supported(const char *machine)
{
	static const char *const ok[] = { "paddlema", "tnextspc", "sstingry", "kyros", "jongbou" };
	for (unsigned i = 0; i < sizeof(ok) / sizeof(ok[0]); i++)
		if (!strcmp(machine, ok[i]))
			return 1;
	return 0;
}

const rbx_driver rbx_drv_alpha68k_n = { "SNK Alpha 68K I / N", an_init, an_exit, an_reset, an_frame, an_save, an_load,
                                        an_supported, an_render, NULL };
