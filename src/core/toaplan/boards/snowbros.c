/*  Snow Bros. and the boards built on it: Toaplan's Snow Bros (Kaneko Pandora sprites, Z80 + YM3812), the Winter
    Bobble bootleg, SemiCom's games (Z80 + YM2151 + M6295, the AT89C52 protection MCU or the data it leaves in RAM),
    Final Tetris, Puzzle King, Barko's Honey Doll and Twin Adventure, Snow Brothers 3 (its sound MCU as MAME's HLE),
    Puzzle Yutnori.
    Ported from MAME 0.289 src/mame/kaneko/snowbros.cpp, snowbros.h, snowbros_v.cpp (BSD-3-Clause, copyright-holders:
    David Haywood, Mike Coates, Luca Elia) and src/mame/kaneko/kan_pand.cpp (BSD-3-Clause, copyright-holders: David
    Haywood, Luca Elia).
    A 68000 with three scanline interrupts (lines 32, 128, 240, each held until acknowledged), sprites only: the
    Pandora chip draws its list into a frame buffer at vblank that the next frame shows; the bootleg chips draw the
    list as the screen is drawn.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../rbx_prof.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/m68k_spin.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/mcs51/mcs51.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define VIS_Y0 16
#define VIS_H 224
#define SCREEN_W 256

// ---- machine configurations ----
enum { MAP_SNOWBROS, MAP_WINTBOB, MAP_HYPERPAC, MAP_FINALTTR, MAP_HONEYDOL, MAP_TWINADV, MAP_SNOWBRO3, MAP_YUTNORI };
enum { VID_PANDORA, VID_WB, VID_HONEYDOL, VID_TWINADV, VID_SB3 };
enum { SND_3812, SND_2151, SND_3812_OKI, SND_OKI_Z80, SND_SB3, SND_NONE };
enum { LAY_SB, LAY_HP, LAY_WB };

typedef struct {
	const char *name;
	uint8_t map, vid, snd, lay;
	uint32_t cpu_clock, z80_clock, ym_clock, oki_clock;
	uint16_t ym_gain, oki_gain;   // 1/256
	uint32_t fps_mhz;
	uint16_t lines, pal;
	uint8_t mcu;                  // the AT89C52 runs (semicom_mcu)
	int32_t protdata;             // the MCU's data copied to RAM at reset (byte offset), -1 none
} machine_t;

static const machine_t s_machines[] = {
	{ "snowbros", MAP_SNOWBROS, VID_PANDORA, SND_3812, LAY_SB, 8000000, 6000000, 3000000, 0, 256, 0, 57500, 262, 256, 0, -1 },
	{ "wintbob", MAP_WINTBOB, VID_WB, SND_3812, LAY_WB, 10000000, 6000000, 3000000, 0, 256, 0, 57500, 262, 256, 0, -1 },
	{ "semicom_mcu", MAP_HYPERPAC, VID_PANDORA, SND_2151, LAY_HP, 12000000, 4000000, 4000000, 1000000, 26, 256, 57500, 262, 256, 1, -1 },
	{ "semicom", MAP_HYPERPAC, VID_PANDORA, SND_2151, LAY_HP, 12000000, 4000000, 4000000, 1000000, 26, 256, 57500, 262, 256, 0, -1 },
	{ "semiprot", MAP_HYPERPAC, VID_PANDORA, SND_2151, LAY_HP, 12000000, 4000000, 4000000, 1000000, 26, 256, 57500, 262, 256, 0, 0xf000 },
	{ "finalttr", MAP_FINALTTR, VID_PANDORA, SND_2151, LAY_HP, 12000000, 3579545, 3579545, 999900, 20, 102, 57500, 262, 256, 0, 0x2000 },
	{ "_4in1", MAP_HYPERPAC, VID_PANDORA, SND_2151, LAY_SB, 12000000, 4000000, 4000000, 1000000, 26, 256, 57500, 262, 256, 0, -1 },
	{ "honeydol", MAP_HONEYDOL, VID_HONEYDOL, SND_3812_OKI, LAY_SB, 12000000, 4000000, 3000000, 1000000, 256, 256, 57500, 262, 1024, 0, -1 },
	{ "twinadv", MAP_TWINADV, VID_TWINADV, SND_OKI_Z80, LAY_SB, 12000000, 4000000, 0, 1000000, 0, 256, 57500, 262, 256, 0, -1 },
	{ "snowbro3", MAP_SNOWBRO3, VID_SB3, SND_SB3, LAY_SB, 12000000, 0, 0, 1000000, 0, 256, 60000, 256, 512, 0, -1 },
	{ "yutnori", MAP_YUTNORI, VID_PANDORA, SND_NONE, LAY_HP, 8000000, 0, 0, 0, 0, 0, 57500, 262, 256, 0, -1 },
};
#define NMACHINES ((int)(sizeof(s_machines) / sizeof(s_machines[0])))

static const machine_t *s_mc;
enum { INIT_NONE, INIT_TOTO, INIT_HYPERPAC, INIT_COOKBIB2, INIT_PZLBREAK, INIT_3IN1, INIT_COOKBIB3, INIT_4IN1,
       INIT_SNOWBRO3, INIT_BALLBOY3P, INIT_YUTNORI, INIT_SUTJAROD, INIT_GWASU };
static int s_init;

// ---- ROMs, chips ----
static uint8_t *s_rom, *s_zrom, *s_oki_rom, *s_prot;
static uint32_t s_rom_size, s_zrom_size, s_oki_size, s_prot_size;
static int s_prot_swapped;
static const uint8_t *s_zmap[256];
static rbx_gfx s_gfx[2];          // [0] the 4 bpp sprites, [1] the 8 bpp ones (Honey Doll, Snow Bros 3)
static int s_port_dsw1, s_port_dsw2, s_port_sys;
static uint16_t s_prot_base;       // the MCU's window into RAM (words)
static uint16_t s_bg_pen;          // the Pandora frame buffer's fill
static uint16_t s_extra_read;      // 0x200000's value (init_3in1semi, init_cookbib3, init_4in1boot), 0 = unmapped
static okim6295_t s_oki;
static mcs51_t s_mcu;
static z80 s_z80;
static m68k_spin_t s_spin;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static uint32_t s_out_rate;

enum { SB_PA_M68K = RBX_PA_DRV, SB_PA_Z80, SB_PA_MCU, SB_PA_SND, SB_PA_END };
static const char *const s_pa_names[] = { "m68k", "z80", "mcu", "sound" };

static struct snow_state {   // (on the heap while the game runs, ADR-040)
	uint16_t ram[0x10000 / 2];
	uint16_t pal[0x800 / 2];
	uint16_t spr[0x2200 / 2];       // the bootleg chips' sprite RAM
	uint8_t pand[0x1000];           // the Pandora chip's sprite RAM
	uint8_t pand_prev[0x1000];      // its list as drawn into the frame buffer at the last vblank
	uint8_t pand_flip_prev, flip, pand_flip;
	uint8_t latch[2], latch_pending;
	uint8_t z80_ram[0x800];
	uint8_t prot_offset;
	uint8_t sb3_playing, sb3_music, sb3_bank;
	uint8_t oki_bank;
	uint16_t yut_prot;
	int z80_irq, z80_vbl;           // the YM's IRQ line; Twin Adventure's vblank IRQ (held)
	uint64_t cpu_frac, z80_frac, ym_frac, mcu_frac, out_frac;
	uint64_t z80_target;
	uint32_t oki_acc;
	int32_t oki_hold;
	int state_end;
} *s_bp;
#define b (*s_bp)

static struct {
	uint16_t pal[0x800 / 2];
	uint16_t spr[0x2200 / 2];
	uint8_t pand[0x1000];
	uint8_t flip;
} *s_snap;

// ---- graphics decoding (MAME gfx_layouts whose pixels are nibbles: one nibble a pixel, or two for 8 bpp) ----
static int decode_nib(rbx_gfx *g, const uint8_t *rom, uint32_t count, uint32_t inc, const uint32_t *xo,
                      const uint32_t *yo, uint32_t hi, int64_t lo)
{
	memset(g, 0, sizeof(*g));
	if (!count)
		return -1;
	g->pix = (uint8_t *)malloc((size_t)count * 257);
	if (!g->pix)
		return -1;
	g->used = g->pix + (size_t)count * 256;
	g->count = count;
	g->width = g->height = 16;
	uint32_t off[256];
	for (int y = 0; y < 16; y++)
		for (int x = 0; x < 16; x++)
			off[y * 16 + x] = yo[y] + xo[x];
	for (uint32_t c = 0; c < count; c++) {
		const uint64_t base = (uint64_t)c * inc;
		uint8_t *d = g->pix + (size_t)c * 256, any = 0;
		for (int i = 0; i < 256; i++) {
			uint64_t bp = base + off[i] + hi;
			uint8_t v = rom[bp >> 3];
			v = (bp & 4) ? (v & 15) : (v >> 4);
			if (lo >= 0) {
				const uint64_t bl = base + off[i] + (uint64_t)lo;
				const uint8_t w = rom[bl >> 3];
				v = (uint8_t)(v << 4 | ((bl & 4) ? (w & 15) : (w >> 4)));
			}
			d[i] = v;
			any |= v;
		}
		g->used[c] = any != 0;
	}
	return 0;
}

static int decode_gfx(void)
{
	uint32_t sz = 0, sz2 = 0;
	uint8_t *r = rbx_region("gfx1", &sz);
	if (!r)
		return -1;
	uint32_t xo[16], yo[16];
	switch (s_mc->lay) {
	case LAY_SB:   // gfx_8x8x4_row_2x2_group_packed_msb
		for (int i = 0; i < 16; i++) {
			xo[i] = (uint32_t)(i < 8 ? i * 4 : 256 + (i - 8) * 4);
			yo[i] = (uint32_t)(i < 8 ? i * 32 : 512 + (i - 8) * 32);
		}
		break;
	case LAY_HP: {   // hyperpac_tilelayout
		static const uint32_t x[16] = { 4, 0, 8*32+4, 8*32+0, 20, 16, 8*32+20, 8*32+16,
		                                12, 8, 8*32+12, 8*32+8, 28, 24, 8*32+28, 8*32+24 };
		static const uint32_t y[16] = { 0*32, 2*32, 1*32, 3*32, 16*32+0*32, 16*32+2*32, 16*32+1*32, 16*32+3*32,
		                                4*32, 6*32, 5*32, 7*32, 16*32+4*32, 16*32+6*32, 16*32+5*32, 16*32+7*32 };
		memcpy(xo, x, sizeof(xo));
		memcpy(yo, y, sizeof(yo));
		break;
	}
	default:   // tilelayout_wb
		for (int i = 0; i < 16; i++) {
			xo[i] = (uint32_t)((i & ~3) * 4 + (3 - (i & 3)) * 4);
			yo[i] = (uint32_t)(i * 64);
		}
		break;
	}
	if (decode_nib(&s_gfx[0], r, sz / 128, 1024, xo, yo, 0, -1))
		return -1;
	if (s_mc->vid == VID_HONEYDOL) {   // honeydol_tilelayout8bpp: planes 0-3 and RGN_FRAC(1,2) + 0-3
		uint8_t *r2 = rbx_region("gfx2", &sz2);
		if (!r2)
			return -1;
		for (int i = 0; i < 16; i++) {
			xo[i] = (uint32_t)(i < 8 ? i * 4 : 8 * 32 + (i - 8) * 4);
			yo[i] = (uint32_t)(i < 8 ? i * 32 : 16 * 32 + (i - 8) * 32);
		}
		if (decode_nib(&s_gfx[1], r2, sz2 / 2 / 128, 1024, xo, yo, 0, (int64_t)(sz2 / 2) * 8))
			return -1;
	} else if (s_mc->vid == VID_SB3) {   // sb3_tilebglayout: planes 8-11, 0-3
		uint8_t *r2 = rbx_region("gfx2", &sz2);
		if (!r2)
			return -1;
		static const uint32_t x[16] = { 0, 4, 16, 20, 32, 36, 48, 52, 512+0, 512+4, 512+16, 512+20, 512+32, 512+36, 512+48, 512+52 };
		static const uint32_t y[16] = { 0*64, 1*64, 2*64, 3*64, 4*64, 5*64, 6*64, 7*64,
		                                1024+0*16, 1024+1*64, 1024+2*64, 1024+3*64, 1024+4*64, 1024+5*64, 1024+6*64, 1024+7*64 };
		if (decode_nib(&s_gfx[1], r2, sz2 / 256, 2048, x, y, 8, 0))
			return -1;
	}
	return 0;
}

// ---- interrupts ----
static void irq_set(int level) { m68k_bus_set_line(level, 1); }
static void irq_ack(int level) { m68k_bus_set_line(level, 0); }

// ---- Snow Bros 3: the sound MCU as MAME's HLE (sb3_sound_w) ----
static void sb3_play_sound(int data)
{
	const uint8_t status = oki_read(&s_oki);
	if (!(status & 1)) {
		oki_write(&s_oki, (uint8_t)(0x80 | data));
		oki_write(&s_oki, 0x12);
	} else if (!(status & 2)) {
		oki_write(&s_oki, (uint8_t)(0x80 | data));
		oki_write(&s_oki, 0x22);
	} else if (!(status & 4)) {
		oki_write(&s_oki, (uint8_t)(0x80 | data));
		oki_write(&s_oki, 0x42);
	}
}
static void sb3_play_music(int data)
{
	b.sb3_music = (uint8_t)data;   // (the sample is played in the scanline callback so that it loops)
	switch (data) {
	case 0x23: case 0x26: b.sb3_bank = 0; b.sb3_playing = 1; break;
	case 0x24: b.sb3_bank = 1; b.sb3_playing = 1; break;
	case 0x25: case 0x27: case 0x28: case 0x29: case 0x2a: case 0x2b: case 0x2c: case 0x2d:
		b.sb3_bank = 2; b.sb3_playing = 1; break;
	case 0x2e: b.sb3_playing = 0; break;
	}
}
static void sb3_sound_w(uint16_t data)
{
	if (data == 0x00fe) {
		b.sb3_playing = 0;
		oki_write(&s_oki, 0x78);   // (stop sounds)
		return;
	}
	const int d = data >> 8;
	if (d <= 0x21)
		sb3_play_sound(d);
	if (d >= 0x22 && d <= 0x31)
		sb3_play_music(d);
	if (d >= 0x30 && d <= 0x51)
		sb3_play_sound(d - 0x30);
	if (d >= 0x52 && d <= 0x5f)
		sb3_play_music(d - 0x30);
}
static void sb3_scanline(void)
{
	const uint8_t status = oki_read(&s_oki);
	if (b.sb3_playing) {
		if (!(status & 8)) {
			oki_write(&s_oki, (uint8_t)(0x80 | b.sb3_music));
			oki_write(&s_oki, 0x82);
		}
	} else if (status & 8)
		oki_write(&s_oki, 0x40);   // (stop playing music)
}
static uint8_t oki_read_sb3(void *u, uint32_t a)   // snowbros3_oki_map
{
	(void)u;
	const uint32_t o = a < 0x20000 ? a : 0x20000 + (uint32_t)b.sb3_bank * 0x20000 + (a - 0x20000);
	return o < s_oki_size ? s_oki_rom[o] : 0;
}

// ---- the main CPU's devices ----
static uint16_t port16(int p) { return p >= 0 ? (uint16_t)rbx_port_read(p) : 0xffff; }

static uint16_t main_r16(uint32_t a)
{
	switch (s_mc->map) {
	case MAP_HONEYDOL:
		if (a >= 0x900000 && a <= 0x900005)
			return port16(a == 0x900000 ? s_port_dsw1 : a == 0x900002 ? s_port_dsw2 : s_port_sys);
		return 0;
	case MAP_YUTNORI:
		if (a >= 0x300000 && a <= 0x300005)
			return port16(a == 0x300000 ? s_port_dsw1 : a == 0x300002 ? s_port_dsw2 : s_port_sys);
		if (a >= 0x600000 && a <= 0x601fff)
			return (uint16_t)(b.pand[(a >> 1) & 0xfff] * 0x101);
		if (a == 0x200000)   // yutnori_prot_r: the reply to the fourth byte of the sequence
			return b.yut_prot == 0x33 ? 0xcc : b.yut_prot == 0x60 ? 0xf9 : b.yut_prot == 0xb8 ? 0x74 : 0;
		return 0;
	default:
		break;
	}
	if (a >= 0x500000 && a <= 0x500005)
		return port16(a == 0x500000 ? s_port_dsw1 : a == 0x500002 ? s_port_dsw2 : s_port_sys);
	if (a >= 0x700000 && a <= 0x701fff && s_mc->vid == VID_PANDORA)
		return (uint16_t)(b.pand[(a >> 1) & 0xfff] * 0x101);
	if (a == 0x300000) {
		if (s_mc->map == MAP_SNOWBRO3)
			return 0x0003;   // sb3_sound_r
		if (s_mc->map == MAP_HYPERPAC || s_mc->map == MAP_FINALTTR)
			return 0;
		return b.latch[1];
	}
	if (a == 0x500006 && s_init == INIT_TOTO)
		return 0x0700;   // toto_read
	if (a == 0x200000 && s_extra_read)
		return s_extra_read;
	return 0;
}

static uint8_t main_r8(uint32_t a)
{
	return (uint8_t)(main_r16(a & ~1u) >> ((a & 1) ? 0 : 8));
}

static void main_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	switch (s_mc->map) {
	case MAP_HONEYDOL:
		if (a == 0x300000 && (mask & 0xff)) {
			b.latch[0] = (uint8_t)v;
			if (!b.latch_pending)   // (the latch's pending line on the NMI)
				z80_gen_nmi(&s_z80);
			b.latch_pending = 1;
		} else if (a == 0x400000)
			irq_ack(4);
		else if (a == 0x500000)
			irq_ack(3);
		else if (a == 0x600000)
			irq_ack(2);
		return;
	case MAP_YUTNORI:
		if (a >= 0x600000 && a <= 0x601fff) {
			const uint32_t o = (a >> 1) & 0xfff;
			if (mask & 0xff00)
				b.pand[o] = (uint8_t)(v >> 8);
			if (mask & 0x00ff)
				b.pand[o] = (uint8_t)v;
		} else if (a == 0x200000)
			b.yut_prot = (uint16_t)((b.yut_prot & ~mask) | (v & mask));
		else if (a == 0x800000)
			irq_ack(4);
		else if (a == 0x900000)
			irq_ack(3);
		else if (a == 0xa00000)
			irq_ack(2);
		return;
	default:
		break;
	}
	if (a >= 0x700000 && a <= 0x701fff && s_mc->vid == VID_PANDORA) {   // spriteram_lsb_w
		const uint32_t o = (a >> 1) & 0xfff;
		if (mask & 0xff00)
			b.pand[o] = (uint8_t)(v >> 8);
		if (mask & 0x00ff)
			b.pand[o] = (uint8_t)v;
		return;
	}
	switch (a) {
	case 0x300000:
		if (s_mc->map == MAP_SNOWBRO3) {
			sb3_sound_w(v);
			return;
		}
		if (mask & 0xff) {
			b.latch[0] = (uint8_t)v;
			if (s_mc->snd != SND_2151) {   // (SemiCom: the latch's pending line is not connected)
				if (!b.latch_pending)
					z80_gen_nmi(&s_z80);
				b.latch_pending = 1;
			}
		}
		return;
	case 0x400000:   // flipscreen_w (the Pandora's or the bootleg's: both !bit 7)
		if ((mask & 0xff00) && s_init != INIT_BALLBOY3P && s_mc->map != MAP_HYPERPAC && s_mc->map != MAP_FINALTTR)
			b.flip = !((v >> 15) & 1);
		return;
	case 0x800000: irq_ack(4); return;
	case 0x900000: irq_ack(3); return;
	case 0xa00000: irq_ack(2); return;
	}
}

static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v)
{
	if (a & 1)
		main_w16m(a & ~1u, v, 0x00ff);
	else
		main_w16m(a, (uint16_t)(v << 8), 0xff00);
}

// ---- the sound CPU ----
static uint8_t z_read(void *u, uint16_t a)
{
	(void)u;
	if (s_mc->map == MAP_HYPERPAC || s_mc->map == MAP_FINALTTR) {   // hyperpac_sound_map
		if (a < 0xd000)
			return a < s_zrom_size ? s_zrom[a] : 0;
		if (a < 0xd800)
			return b.z80_ram[a - 0xd000];
		if (a == 0xf000 || a == 0xf001)
			return rbx_ym_read(0, a & 1);
		if (a == 0xf002)
			return oki_read(&s_oki);
		if (a == 0xf008)
			return b.latch[0];
		return 0;
	}
	if (a < 0x8000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0x8800)
		return b.z80_ram[a - 0x8000];
	if (a == 0xe010 && s_mc->map == MAP_HONEYDOL)
		return oki_read(&s_oki);
	return 0;
}
static void z_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_mc->map == MAP_HYPERPAC || s_mc->map == MAP_FINALTTR) {
		if (a >= 0xd000 && a < 0xd800)
			b.z80_ram[a - 0xd000] = v;
		else if (a == 0xf000 || a == 0xf001)
			rbx_ym_write(0, a & 1, v);
		else if (a == 0xf002)
			oki_write(&s_oki, v);
		return;
	}
	if (a >= 0x8000 && a < 0x8800)
		b.z80_ram[a - 0x8000] = v;
	else if (a == 0xe010 && s_mc->map == MAP_HONEYDOL)
		oki_write(&s_oki, v);
}
static uint8_t z_in(z80 *z, uint8_t port)
{
	(void)z;
	if (s_mc->map == MAP_TWINADV) {   // twinadv_sound_io_map
		if (port == 0x02) {
			b.latch_pending = 0;
			return b.latch[0];
		}
		if (port == 0x06)
			return oki_read(&s_oki);
		return 0;
	}
	if (port == 0x02 || port == 0x03)
		return rbx_ym_read(0, port & 1);
	if (port == 0x04) {
		b.latch_pending = 0;
		return b.latch[0];
	}
	return 0;
}
static void z_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	if (s_mc->map == MAP_TWINADV) {
		if (port == 0x02)
			b.latch[1] = v;
		else if (port == 0x04) {   // twinadv_oki_bank_w
			b.oki_bank = (v >> 1) & 1;
			s_oki.bank = (uint32_t)b.oki_bank * 0x40000;
		} else if (port == 0x06)
			oki_write(&s_oki, v);
		return;
	}
	if (port == 0x02 || port == 0x03)
		rbx_ym_write(0, port & 1, v);
	else if (port == 0x04 && s_mc->map != MAP_HONEYDOL)
		b.latch[1] = v;
}
static void z_setup(void)
{
	s_z80.read_byte = z_read;
	s_z80.write_byte = z_write;
	s_z80.port_in = z_in;
	s_z80.port_out = z_out;
	s_z80.userdata = NULL;
	s_z80.read_map = s_zmap;
}
static void ym_irq(int state) { b.z80_irq = state; }

// ---- the SemiCom protection MCU: its ports write a word of the 68000's RAM ----
static void prot_port_w(void *u, int port, uint8_t v)
{
	(void)u;
	uint16_t *w = &b.ram[(s_prot_base + b.prot_offset) & 0x7fff];
	if (port == 0)        // prot_p0_w
		*w = (uint16_t)((*w & 0xff00) | v);
	else if (port == 1)   // prot_p1_w
		*w = (uint16_t)((*w & 0x00ff) | v << 8);
	else if (port == 2)   // prot_p2_w
		b.prot_offset = v;
}

// ---- video ----
// transpen 0 of a 16x16 element, clipped to [x0, x1) x [VIS_Y0, VIS_Y0 + VIS_H)
static void blit(const rbx_gfx *g, uint32_t code, uint32_t cbase, int fx, int fy, int sx, int sy, int x0, int x1)
{
	code = rbx_wrap(code, g->count);
	if (!g->used[code])
		return;
	int cx0 = sx < x0 ? x0 - sx : 0, cx1 = sx + 16 > x1 ? x1 - sx : 16;
	int cy0 = sy < VIS_Y0 ? VIS_Y0 - sy : 0, cy1 = sy + 16 > VIS_Y0 + VIS_H ? VIS_Y0 + VIS_H - sy : 16;
	if (cx0 >= cx1 || cy0 >= cy1)
		return;
	const uint8_t *t = g->pix + (size_t)code * 256;
	const int pitch = rbx_video.pitch;
	for (int y = cy0; y < cy1; y++) {
		const uint8_t *row = t + (fy ? 15 - y : y) * 16;
		uint16_t *d = rbx_video.bitmap + (sy + y - VIS_Y0) * pitch + sx;
		if (fx) {
			for (int x = cx0; x < cx1; x++) {
				const uint8_t p = row[15 - x];
				if (p)
					d[x] = (uint16_t)(cbase + p);
			}
		} else {
			for (int x = cx0; x < cx1; x++) {
				const uint8_t p = row[x];
				if (p)
					d[x] = (uint16_t)(cbase + p);
			}
		}
	}
}

static void fill(uint16_t pen)
{
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
		for (int x = 0; x < SCREEN_W; x++)
			d[x] = pen;
	}
}

static inline int sext9(int v) { v &= 0x1ff; return (v & 0x100) ? v - 0x200 : v; }

static void draw_pandora(void)   // kaneko_pandora_device::draw into its 256x256 frame buffer, then update
{
	fill(s_bg_pen ? s_bg_pen : 0xf0);
	const uint8_t *s = s_snap->pand;
	const int flip = s_snap->flip;
	int x = 0, y = 0;
	for (int offs = 0; offs < 0x1000; offs += 8) {
		int dx = s[offs + 4], dy = s[offs + 5];
		const uint8_t col = s[offs + 3], attr = s[offs + 7];
		int fx = (attr >> 7) & 1, fy = (attr >> 6) & 1;
		const uint32_t tile = (uint32_t)((attr & 0x3f) << 8) + s[offs + 6];
		if (col & 1)
			dx |= 0x100;
		if (col & 2)
			dy |= 0x100;
		if (col & 4) {
			x += dx;
			y += dy;
		} else {
			x = dx;
			y = dy;
		}
		int sx = x, sy = y;
		if (flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			fx = !fx;
			fy = !fy;
		}
		blit(&s_gfx[0], tile, (uint32_t)(col & 0xf0), fx, fy, sext9(sx), sext9(sy), 0, 256);
	}
}

static void draw_honeydol(void)
{
	fill(0xf0);
	const uint16_t *s = s_snap->spr;
	const int flip = s_snap->flip;
	for (int offs = 0; offs < 0x2000 / 2; offs += 8) {
		int dx = (s[offs + 4] >> 8) & 0xff, dy = (s[offs + 5] >> 8) & 0xff;
		int col = (s[offs + 3] >> 8) & 3;
		int attr = s[offs + 7] >> 8;
		int fx = (attr >> 7) & 1, fy = (attr >> 6) & 1;
		uint32_t tile = (uint32_t)((attr & 0x3f) << 8) + ((s[offs + 6] >> 8) & 0xff);
		if (flip) {
			dx = 240 - dx;
			dy = 240 - dy;
			fx = !fx;
			fy = !fy;
		}
		blit(&s_gfx[1], tile, (uint32_t)col * 256, fx, fy, dx, dy, 0, 256);
		// the second list interleaved with the first (4 bpp)
		dx = s[offs + 4] & 0xff;
		dy = s[offs + 5] & 0xff;
		col = s[offs + 3];
		attr = s[offs + 7];
		fx = (attr >> 7) & 1;
		fy = (attr >> 6) & 1;
		tile = (uint32_t)((attr & 0x3f) << 8) + (s[offs + 6] & 0xff);
		if (flip) {
			dx = 240 - dx;
			dy = 240 - dy;
			fx = !fx;
			fy = !fy;
		}
		col = ((col & 0x03f0) >> 4) ^ 0x3f;
		blit(&s_gfx[0], tile, (uint32_t)col * 16, fx, fy, dx, dy, 0, 256);
	}
}

static void draw_twinadv(void)
{
	fill(0xf0);
	const uint16_t *s = s_snap->spr;
	const int flip = s_snap->flip;
	for (int offs = 0; offs < 0x2000 / 2; offs += 8) {
		int dx = s[offs + 4] & 0xff, dy = s[offs + 5] & 0xff;
		int col = s[offs + 3];
		const int attr = s[offs + 7] & 0xff;
		int fx = (attr >> 7) & 1, fy = (attr >> 6) & 1;
		const uint32_t tile = (uint32_t)((attr & 0x3f) << 8) + (s[offs + 6] & 0xff);
		if (flip) {
			dx = 240 - dx;
			dy = 240 - dy;
			fx = !fx;
			fy = !fy;
		}
		col = ((col & 0x00f0) >> 4) ^ 0xf;
		blit(&s_gfx[0], tile, (uint32_t)col * 16, fx, fy, dx, dy, 0, 256);
	}
}

static void draw_wintbob(void)
{
	fill((uint16_t)s_mc->pal);   // (black_pen)
	const uint16_t *s = s_snap->spr;
	const int flip = s_snap->flip;
	for (int offs = 0; offs < 0x2000 / 2; offs += 8) {
		const int attr = s[offs + 1] & 0xff;
		if (attr & 2)
			continue;
		int xpos = s[offs] & 0xff, ypos = s[offs + 4] & 0xff;
		const int colr = (attr & 0xf0) >> 4;
		const int tf = s[offs + 2] & 0xff;
		const uint32_t tile = (uint32_t)(tf << 8) + (s[offs + 3] & 0xff);
		int fx = (tf >> 7) & 1, fy = (tf >> 6) & 1;
		if (attr & 8)
			xpos -= 256;
		if (flip) {
			xpos = 240 - xpos;
			ypos = 240 - ypos;
			fx = !fx;
			fy = !fy;
		}
		if (xpos > -16 && ypos > 0 && xpos < 256 && ypos < 240)
			blit(&s_gfx[0], tile, (uint32_t)colr * 16, fx, fy, xpos, ypos, 0, 256);
	}
}

static void draw_snowbro3(void)
{
	fill((uint16_t)s_mc->pal);   // (black_pen)
	const uint16_t *s = s_snap->spr;
	const int flip = s_snap->flip;
	int x = 0, y = 0;
	for (int offs = 0; offs < 0x2200 / 2; offs += 8) {
		int dx = s[offs + 4] & 0xff, dy = s[offs + 5] & 0xff;
		int col = s[offs + 3] & 0xff;
		const int attr = s[offs + 7] & 0xff;
		int fx = (attr >> 7) & 1, fy = (attr >> 6) & 1;
		const uint32_t tile = (uint32_t)(attr << 8) + (s[offs + 6] & 0xff);
		if (col & 1)
			dx = -1 - (dx ^ 0xff);
		if (col & 2)
			dy = -1 - (dy ^ 0xff);
		if (col & 4) {
			x += dx;
			y += dy;
		} else {
			x = dx;
			y = dy;
		}
		if (x > 511) x &= 0x1ff;
		if (y > 511) y &= 0x1ff;
		int sx = x, sy = y;
		if (flip) {
			sx = 240 - x;
			sy = 240 - y;
			fx = !fx;
			fy = !fy;
		}
		int gfx = 0;
		if (offs < 0x800) {
			gfx = 1;
			col = 0x10;
		}
		blit(&s_gfx[gfx], tile, (uint32_t)((col & 0xf0) >> 4) * (gfx ? 256 : 16), fx, fy, sx, sy, 0, 256);
	}
}

static void sb_render(void)
{
	const int n = s_mc->pal;
	const uint16_t *p = s_snap->pal;
	for (int i = 0; i < n; i++) {   // xBGR_555
		int r = p[i] & 31, g = (p[i] >> 5) & 31, bl = (p[i] >> 10) & 31;
		r = (r << 3) | (r >> 2);
		g = (g << 3) | (g >> 2);
		bl = (bl << 3) | (bl >> 2);
		rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | bl);
	}
	rbx_video.palette[n] = 0;   // (black_pen)
	switch (s_mc->vid) {
	case VID_PANDORA: draw_pandora(); break;
	case VID_HONEYDOL: draw_honeydol(); break;
	case VID_TWINADV: draw_twinadv(); break;
	case VID_WB: draw_wintbob(); break;
	default: draw_snowbro3(); break;
	}
}

// the vblank: the screen drawn first (MAME's frame update before the vblank callbacks), then the Pandora's eof
static void vblank(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->pal, b.pal, sizeof(b.pal));
	if (s_mc->vid == VID_PANDORA) {
		memcpy(s_snap->pand, b.pand_prev, sizeof(b.pand_prev));
		s_snap->flip = b.pand_flip_prev;
	} else {
		memcpy(s_snap->spr, b.spr, sizeof(b.spr));
		s_snap->flip = b.flip;
	}
	rbx_video_frame_done();
	if (s_mc->vid == VID_PANDORA) {   // eof: the list drawn into the other buffer, shown by the next frame
		memcpy(b.pand_prev, b.pand, sizeof(b.pand));
		b.pand_flip_prev = b.flip;
	}
}

// ---- frame ----
static void sound_line(void)
{
	const machine_t *mc = s_mc;
	int32_t *out = s_mix + s_mix_n;
	const int room = s_mix_cap - s_mix_n;
	int n = 0;
	const unsigned prof = rbx_prof_enter(SB_PA_SND);
	switch (mc->snd) {
	case SND_3812: case SND_2151: case SND_3812_OKI:
		n = rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&b.ym_frac, mc->ym_clock, mc->fps_mhz, mc->lines), out, room);
		if (mc->snd != SND_3812)
			for (int i = 0; i < n; i++) {
				b.oki_acc += s_oki.rate;
				while (b.oki_acc >= s_out_rate) {
					b.oki_acc -= s_out_rate;
					b.oki_hold = oki_sample(&s_oki);
				}
				out[i] += b.oki_hold;
			}
		break;
	case SND_OKI_Z80: case SND_SB3:
		n = rbx_line_cycles(&b.out_frac, s_out_rate, mc->fps_mhz, mc->lines);
		if (n > room)
			n = room;
		for (int i = 0; i < n; i++)
			out[i] += oki_sample(&s_oki);
		break;
	default:
		n = rbx_line_cycles(&b.out_frac, s_out_rate, mc->fps_mhz, mc->lines);
		if (n > room)
			n = room;
		break;
	}
	rbx_prof_leave(prof);
	s_mix_n += n;
}

static int has_ym(void) { return s_mc->snd == SND_3812 || s_mc->snd == SND_2151 || s_mc->snd == SND_3812_OKI; }

static void sb_frame(void)
{
	const machine_t *mc = s_mc;
	rbx_audio.frame_mhz = mc->fps_mhz;
	for (int line = 0; line < mc->lines; line++) {
		// the scanline timer (snowbros_state::scanline) and the vblank, at the start of the line
		if (line == 240) {
			irq_set(2);
			vblank();
			if (mc->snd == SND_OKI_Z80)
				b.z80_vbl = 1;   // (irq0_line_hold)
		} else if (line == 128)
			irq_set(3);
		else if (line == 32)
			irq_set(4);
		if (mc->snd == SND_SB3)
			sb3_scanline();
		rbx_prof_set(SB_PA_M68K);
		// (its polling loops skipped within the slice: m68k_spin, the same cycles and state)
		m68k_spin_execute(&s_spin, rbx_line_cycles(&b.cpu_frac, mc->cpu_clock, mc->fps_mhz, mc->lines), 0);
		if (mc->z80_clock) {
			rbx_prof_set(SB_PA_Z80);
			b.z80_target += (uint64_t)rbx_line_cycles(&b.z80_frac, mc->z80_clock, mc->fps_mhz, mc->lines);
			if (mc->snd == SND_OKI_Z80)
				z80_run(&s_z80, (unsigned long)b.z80_target, &b.z80_vbl, 1);
			else
				z80_run(&s_z80, (unsigned long)b.z80_target, &b.z80_irq, 0);
		}
		if (mc->mcu) {   // (16 MHz: machine cycles of 12 clocks)
			rbx_prof_set(SB_PA_MCU);
			mcs51_execute(&s_mcu, rbx_line_cycles(&b.mcu_frac, 16000000 / 12, mc->fps_mhz, mc->lines));
		}
		sound_line();
	}
	if (has_ym())
		rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static void map_mem(void)
{
	m68k_bus_reset_map();
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
	const int map = s_mc->map;
	const uint32_t rom_end = (map == MAP_HYPERPAC || map == MAP_FINALTTR) ? 0x0fffff : 0x03ffff;
	m68k_bus_map(0, rom_end, s_rom, s_rom_size < rom_end + 1 ? s_rom_size : rom_end + 1, 1, 0);
	switch (map) {
	case MAP_HONEYDOL:
		m68k_bus_map(0x100000, 0x10ffff, b.ram, 0x10000, 1, 1);
		m68k_bus_map(0xa00000, 0xa007ff, b.pal, 0x800, 1, 1);
		m68k_bus_map(0xb00000, 0xb01fff, b.spr, 0x2000, 1, 1);
		break;
	case MAP_YUTNORI:
		m68k_bus_map(0x500000, 0x5001ff, b.pal, 0x200, 1, 1);
		m68k_bus_map(0x700000, 0x70ffff, b.ram, 0x10000, 1, 1);
		break;
	default:
		if (map == MAP_HYPERPAC || map == MAP_TWINADV)
			m68k_bus_map(0x100000, 0x10ffff, b.ram, 0x10000, 1, 1);
		else
			m68k_bus_map(0x100000, 0x103fff, b.ram, 0x4000, 1, 1);
		m68k_bus_map(0x600000, map == MAP_SNOWBRO3 ? 0x6003ff : 0x6001ff, b.pal, map == MAP_SNOWBRO3 ? 0x400 : 0x200, 1, 1);
		if (s_mc->vid != VID_PANDORA)
			m68k_bus_map(0x700000, map == MAP_SNOWBRO3 ? 0x7021ff : 0x701fff, b.spr, map == MAP_SNOWBRO3 ? 0x2200 : 0x2000, 1, 1);
		break;
	}
}

static void sb_reset(void)
{
	m68k_spin_clear(&s_spin);
	rbx_video_wait_idle();
	memset(&b, 0, offsetof(struct snow_state, state_end));
	if (s_mc->protdata >= 0 && s_prot) {   // MACHINE_RESET semiprot / finalttr: the MCU's data put in RAM
		for (uint32_t i = 0; i < 0x200 / 2 && i * 2 + 1 < s_prot_size; i++)
			b.ram[s_mc->protdata / 2 + i] = s_prot_swapped ? (uint16_t)(s_prot[i * 2 + 1] << 8 | s_prot[i * 2]) : (uint16_t)(s_prot[i * 2] << 8 | s_prot[i * 2 + 1]);
	}
	map_mem();
	m68k_bus_irq_reset();
	m68k_bus_cur->irq_asserted = 0;
	m68k_pulse_reset();
	if (s_mc->z80_clock) {
		z80_init(&s_z80);
		z_setup();
	}
	if (s_mc->mcu)
		mcs51_reset(&s_mcu);
	if (has_ym())
		rbx_ym_reset(0);
	if (s_mc->oki_clock) {
		oki_reset(&s_oki);
		s_oki.bank = 0;
	}
	s_mix_n = 0;
}

static const machine_t *find_machine(const char *m)
{
	for (int i = 0; i < NMACHINES; i++) {
		const size_t l = strlen(s_machines[i].name);
		if (!strncmp(m, s_machines[i].name, l) && (m[l] == ':' || !m[l]))
			return &s_machines[i];
	}
	return NULL;
}
static int sb_supported(const char *machine) { return find_machine(machine) != NULL; }

static void sb_exit(void)
{
	m68k_spin_free(&s_spin);
	rbx_video_wait_idle();
	if (s_mc && has_ym())
		rbx_ym_exit(0);
	rbx_gfx_free(&s_gfx[0]);
	rbx_gfx_free(&s_gfx[1]);
	free(s_mix);
	s_mix = NULL;
	free(s_snap);
	s_snap = NULL;
	free(s_bp);
	s_bp = NULL;
}

static int sb_init(const rbx_game *g)
{
	s_mc = find_machine(g->machine);
	if (!s_mc) {
		rbx_fail("%s: machine %s not emulated", g->set, g->machine);
		return -1;
	}
	const char *init = strchr(g->machine, ':');
	init = init ? init + 1 : "";
	static const char *const inits[] = { "", "toto", "hyperpac", "cookbib2", "pzlbreak", "3in1semi", "cookbib3", "4in1boot",
	                                     "snowbro3", "ballboy3p", "yutnori", "sutjarod", "gwasu" };
	s_init = INIT_NONE;
	for (int i = 1; i < (int)(sizeof(inits) / sizeof(inits[0])); i++)
		if (!strcmp(init, inits[i]))
			s_init = i;
	if (!(s_bp = (struct snow_state *)calloc(1, sizeof(*s_bp))) || !(s_snap = calloc(1, sizeof(*s_snap)))) {
		rbx_fail("%s: out of memory", g->set);
		sb_exit();
		return -1;
	}
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom_size = 0;
	s_zrom = s_mc->z80_clock ? rbx_region("soundcpu", &s_zrom_size) : NULL;
	if (!s_rom || (s_mc->z80_clock && !s_zrom)) {
		rbx_fail("%s: ROM regions missing", g->set);
		sb_exit();
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	// the inits (on the host-order words, as MAME's on a little-endian host)
	if (s_init == INIT_TOTO) {   // every ROM has bits 0x10 and 0x08 swapped
		uint32_t gs = 0;
		uint8_t *gfx = rbx_region("gfx1", &gs);
		uint8_t *rr[3] = { s_rom, gfx, s_zrom };
		const uint32_t rs[3] = { s_rom_size, gs, s_zrom_size };
		for (int k = 0; k < 3; k++)
			for (uint32_t i = 0; rr[k] && i < rs[k]; i++) {
				const uint8_t v = rr[k][i];
				rr[k][i] = (uint8_t)((v & 0xe7) | ((v >> 1) & 0x08) | ((v << 1) & 0x10));
			}
	}
	if (s_init == INIT_4IN1) {
		for (uint32_t i = 1; i < s_rom_size; i += 2) {   // bitswap<8>(6,7,5,4,3,2,1,0) of the odd bytes
			const uint8_t v = s_rom[i];
			s_rom[i] = (uint8_t)((v & 0x3f) | ((v >> 1) & 0x40) | ((v << 1) & 0x80));
		}
		uint8_t *buf = (uint8_t *)malloc(s_zrom_size);
		if (!buf) {
			rbx_fail("%s: out of memory", g->set);
			sb_exit();
			return -1;
		}
		for (uint32_t i = 0; i < s_zrom_size; i++)
			buf[i] = s_zrom[(i ^ 0x4000) < s_zrom_size ? (i ^ 0x4000) : i];
		memcpy(s_zrom, buf, s_zrom_size);
		free(buf);
	}
	if (s_init == INIT_SNOWBRO3 || s_init == INIT_BALLBOY3P) {   // bitswap<24> of the address: bits 3 / 4 and 1 / 2
		uint8_t *buf = (uint8_t *)malloc(s_rom_size);
		if (!buf) {
			rbx_fail("%s: out of memory", g->set);
			sb_exit();
			return -1;
		}
		for (uint32_t i = 0; i < s_rom_size; i++) {
			const uint32_t a = (i & ~0x1eu) | ((i >> 1) & 0x08) | ((i << 1) & 0x10) | ((i >> 1) & 0x02) | ((i << 1) & 0x04);
			buf[i] = s_rom[a];
		}
		memcpy(s_rom, buf, s_rom_size);
		free(buf);
	}
	if ((s_init == INIT_SUTJAROD || s_init == INIT_GWASU) && s_rom_size >= 0x10000) {   // the RAM key and the checksum
		uint16_t *r16 = (uint16_t *)s_rom;
		const uint32_t k = s_init == INIT_SUTJAROD ? 0xb994 : 0xfda8;
		r16[k / 2] = 0x434b;
		r16[k / 2 + 1] = 0x3345;
		r16[0x54 / 2] = s_init == INIT_SUTJAROD ? 0x2508 : 0x11ef;
	}
	s_extra_read = s_init == INIT_3IN1 ? 0x000a : s_init == INIT_COOKBIB3 ? 0x2a2a : s_init == INIT_4IN1 ? 0x0202 : 0;
	s_bg_pen = s_init == INIT_PZLBREAK ? 0xc0 : s_init == INIT_YUTNORI ? 0xf0 : 0;
	s_prot_base = s_init == INIT_HYPERPAC ? 0xe000 / 2 : 0xf000 / 2;
	s_prot = NULL;
	s_prot_size = 0;
	if (s_mc->protdata >= 0)
		s_prot = rbx_region_opt("protdata", &s_prot_size);
	// its words: big-endian in the file (ROM_REGION16_BE), or swapped by the loader (ROM_LOAD16_WORD_SWAP)
	s_prot_swapped = 0;
	for (int r = 0; s_prot && g->regions[r].name; r++)
		if (!strcmp(g->regions[r].name, "protdata"))
			for (const rbx_rom_load *l = g->loads; l->op != RBX_ROM_END; l++)
				if (l->region == r && l->group == 2 && l->reverse == 1)
					s_prot_swapped = 1;
	if (decode_gfx()) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		sb_exit();
		return -1;
	}
	// the Z80's reads straight from its ROM and RAM
	memset(s_zmap, 0, sizeof(s_zmap));
	if (s_zrom) {
		const uint32_t rom_end = (s_mc->map == MAP_HYPERPAC || s_mc->map == MAP_FINALTTR) ? 0xd000 : 0x8000;
		for (uint32_t p = 0; p * 256 < rom_end && p * 256 + 256 <= s_zrom_size; p++)
			s_zmap[p] = s_zrom + p * 256;
		const uint32_t ram = rom_end == 0xd000 ? 0xd000 : 0x8000;
		for (uint32_t p = 0; p < 8; p++)
			s_zmap[(ram >> 8) + p] = b.z80_ram + p * 256;
	}
	s_port_dsw1 = rbx_port_find("DSW1");
	s_port_dsw2 = rbx_port_find("DSW2");
	s_port_sys = rbx_port_find("SYSTEM");
	// sound
	memset(&s_oki, 0, sizeof(s_oki));
	if (s_mc->oki_clock) {
		s_oki_rom = rbx_region_opt("oki1", &s_oki_size);
		oki_init(&s_oki, s_mc->oki_clock, 1, s_oki_rom, s_oki_size, s_mc->oki_gain);
		if (s_mc->snd == SND_SB3)
			s_oki.read = oki_read_sb3;
	}
	if (s_mc->mcu) {
		uint32_t ps = 0;
		const uint8_t *prog = rbx_region("protection", &ps);
		if (!prog) {
			sb_exit();
			return -1;
		}
		mcs51_init(&s_mcu, prog, ps < 0x2000 ? ps : 0x2000);
		s_mcu.is_8052 = 1;
		s_mcu.port_out = prot_port_w;
	}
	if (has_ym()) {
		rbx_ym_config yc;
		memset(&yc, 0, sizeof(yc));
		yc.type = s_mc->snd == SND_2151 ? RBX_YM2151 : RBX_YM3812;
		yc.clock = s_mc->ym_clock;
		yc.gain[0] = yc.gain[1] = s_mc->ym_gain;
		yc.irq = ym_irq;
		if (rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
			rbx_fail("%s: out of memory", g->set);
			sb_exit();
			return -1;
		}
		s_out_rate = rbx_ym_rate(0);
	} else
		s_out_rate = s_mc->oki_clock ? s_oki.rate : 8000;
	s_mix_cap = (int)(s_out_rate / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(s_out_rate) || rbx_video_setup(SCREEN_W, VIS_H, s_mc->pal + 1)) {
		rbx_fail("%s: out of memory", g->set);
		sb_exit();
		return -1;
	}
	rbx_prof_name(RBX_PA_DRV, s_pa_names, SB_PA_END - RBX_PA_DRV);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	sb_reset();
	return 0;
}

static int sb_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (load)
		rbx_video_wait_idle();
	else
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "snow", &b, (uint32_t)offsetof(struct snow_state, state_end));
	if (s_mc->z80_clock) {
		z80 z = s_z80;
		rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
		if (!rc && load) {
			s_z80 = z;
			z_setup();
		}
	}
	if (s_mc->mcu) {
		rc |= rbx_state_io(ctx, "mcu", &s_mcu.st, sizeof(s_mcu.st));
		rc |= rbx_state_io(ctx, "mcu2", &s_mcu.st2, sizeof(s_mcu.st2));
	}
	if (s_mc->oki_clock)
		rc |= rbx_state_io(ctx, "oki", &s_oki, (uint32_t)offsetof(okim6295_t, rom));
	if (has_ym())
		rc |= rbx_ym_state(0, ctx, load);
	if (!rc && load) {
		m68k_spin_clear(&s_spin);
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		if (s_mc->snd == SND_OKI_Z80)
			s_oki.bank = (uint32_t)b.oki_bank * 0x40000;
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int sb_save(void *ctx) { return sb_state(ctx, 0); }
static int sb_load(void *ctx) { return sb_state(ctx, 1); }

const rbx_driver rbx_drv_snowbros = { "Snow Bros", sb_init, sb_exit, sb_reset, sb_frame, sb_save, sb_load,
                                      sb_supported, sb_render };
