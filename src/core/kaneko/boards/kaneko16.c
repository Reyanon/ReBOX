/*  Kaneko 16-bit boards. Ported from MAME 0.289 src/mame/kaneko/kaneko16.cpp, kaneko16.h, kaneko16_v.cpp,
    kaneko_tmap.cpp (VIEW2 tilemaps), kaneko_spr.cpp (VU-002 / KC-002 sprites), kaneko_toybox.cpp (TOYBOX MCU
    simulation), kaneko_hit.cpp (CALC hitbox) (BSD-3-Clause, copyright-holders: Luca Elia, David Haywood).
    A 68000 at 12 / 16 MHz; one or two VIEW2 chips (two 512x512 layers of 16x16x4 tiles each, line scroll), the
    VU-002 (4 bpp) or KC-002 (8 bpp) sprites drawn at line 224 into a frame buffer kept between frames (double
    buffered), a palette of 2048 / 32768 xGRB 555. Sound: an OKI with two YM2149s (Berlin Wall, Bakuretsu Breaker,
    Magical Crystals), a Z80 with a YM2151 (+ OKI: Wing Force), or two OKIs (the TOYBOX games). The Berlin Wall's
    32 high colour backgrounds. Not emulated yet: the CALC3 games (Shogun Warriors, B.Rap Boys).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../machine/eepromser.h"
#include "../../sound/ay8910.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/decoband.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

enum { M_BERLWALL, M_BAKUBRKR, M_MGCRYSTL, M_BLAZEON, M_WINGFORC, M_GTMR, M_GTMRE, M_GTMR2, M_BLOODWAR, M_BONKADV,
       M_SHOGWARR, M_BRAPBOYS, M_COUNT };
static const char *const s_mnames[M_COUNT] = { "berlwall", "bakubrkr", "mgcrystl", "blazeon", "wingforc", "gtmr",
	"gtmre", "gtmr2", "bloodwar", "bonkadv", "shogwarr", "brapboys" };

typedef struct {
	uint32_t cpu_clock, fps_mhz;
	int lines, sw, vx0, vx1, vy0, vy1;   // screen width, visible area
	int nview2, view2_colbase, dx, dy, xdim, ydim;
	int spr_type, spr_colbase, spr_xoffs, spr_yoffs, spr_pri[4];
	int pal_size, v2pri;
	int fliptype, irq_base;   // (Shogun Warriors: the flips not latched; its interrupts 4 / 3 / 2)
} machine_t;

static const machine_t s_machines[M_COUNT] = {
	[M_BERLWALL] = { 12000000, 60000, 256, 256, 0, 255, 16, 239, 1, 0x400, 0x5b, -8, 256, 256, 0, 0, 0, -64, { 1, 2, 3, 8 }, 2048, 0 },
	[M_BAKUBRKR] = { 12000000, 59000, 256, 256, 0, 255, 16, 239, 2, 0x400, 0x5b, -8, 256, 240, 0, 0, 0, 0, { 8, 8, 8, 8 }, 2048, 1 },
	[M_MGCRYSTL] = { 12000000, 60000, 256, 256, 0, 255, 16, 239, 2, 0x400, 0x5b, -8, 256, 240, 0, 0, 0, 0, { 2, 3, 5, 7 }, 2048, 0 },
	[M_BLAZEON]  = { 12000000, 60000, 240, 320, 0, 319, 0, 231, 1, 0x400, 0x33, 8, 320, 240, 0, 0, 0x10000 - 0x680, 0, { 1, 2, 8, 8 }, 2048, 0 },
	[M_WINGFORC] = { 16000000, 59185, 240, 320, 0, 319, 0, 223, 1, 0x400, 0x33, 9, 320, 240, 0, 0, 0x10000 - 0x680, 0, { 1, 2, 8, 8 }, 2048, 0 },
	[M_GTMR]     = { 16000000, 60000, 240, 320, 0, 319, 0, 239, 2, 0, 0x33, 0, 320, 240, 1, 0x4000, 0, 0, { 1, 2, 3, 8 }, 32768, 1 },
	[M_GTMRE]    = { 16000000, 60000, 240, 320, 0, 319, 0, 239, 2, 0, 0x33, 0, 320, 240, 1, 0x4000, 0, 0, { 1, 2, 3, 8 }, 32768, 1 },
	[M_GTMR2]    = { 16000000, 60000, 240, 320, 0, 319, 0, 239, 2, 0, 0x33, 0, 320, 240, 1, 0x4000, 0, 0, { 1, 2, 3, 8 }, 32768, 1 },
	[M_BLOODWAR] = { 16000000, 60000, 240, 320, 0, 319, 0, 239, 2, 0, 0x33, 0, 320, 240, 1, 0x4000, 0, 0, { 2, 3, 5, 7 }, 32768, 1 },
	[M_BONKADV]  = { 16000000, 60000, 240, 320, 0, 319, 0, 239, 2, 0, 0x33, 0, 320, 240, 1, 0x4000, 0, 0, { 2, 3, 5, 7 }, 32768, 1 },
	[M_SHOGWARR] = { 12000000, 59185, 240, 320, 40, 295, 16, 239, 1, 0x400, 0x33, -8, 320, 240, 0, 0, 0xa00, -0x40, { 1, 3, 5, 7 }, 2048, 0, 1, 1 },
	[M_BRAPBOYS] = { 12000000, 59185, 240, 320, 40, 295, 16, 239, 1, 0x400, 0x33, -8, 320, 240, 0, 0, 0xa00, -0x40, { 1, 3, 5, 7 }, 2048, 0, 1, 1 },
};

static int s_m;
static const machine_t *s_mc;
static int s_toybox;                 // the TOYBOX games
static int s_calc3;                  // the CALC3 games
static uint8_t *s_calc3_rom;
static uint16_t s_calc3_crc;
static uint8_t *s_rom, *s_spr_gfx, *s_tiles[2], *s_oki_rom[2], *s_mcudata, *s_bitmap, *s_zrom;
static uint32_t s_rom_size, s_spr_size, s_tiles_size[2], s_oki_size[2], s_zrom_size;
static uint32_t s_spr_count, s_tile_count[2];
static int s_port_p1, s_port_p2, s_port_sys, s_port_unk, s_port_dsw1, s_port_dsw2, s_port_extra, s_port_wheel[3],
           s_port_fake;
static okim6295_t s_oki[2];
static int s_noki;
static ay8910_t s_ay[2];
static eepser_t s_eeprom;
static int s_has_eeprom;
static z80 s_z80;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static uint32_t s_out_rate;

// ---- state ----
static struct kaneko16_state {   // (on the heap while the game runs, ADR-040)
	uint16_t ram[0x10000 / 2];      // work RAM
	uint16_t ram2[0x18000 / 2];     // gtmr 0x310000-0x327fff
	uint16_t mcuram[0x10000 / 2];
	uint16_t pal[0x10000 / 2];
	uint16_t spr[0x2000 / 2];       // live sprite RAM
	uint16_t spr_buf[0x2000 / 2];   // the buffered copy (BUFFERED_SPRITERAM16)
	uint16_t view2[2][0x4000 / 2];  // vram_1, vram_0, scroll_1, scroll_0
	uint16_t v2regs[2][0x10];
	uint16_t spr_regs[0x10];
	uint16_t bg15_scroll;
	uint8_t bg15_select, bg15_bright;
	uint8_t spr_flipx, spr_flipy, keep_sprites, buffer;
	uint16_t disp_enable;
	uint16_t mcu_com[4];
	struct { uint16_t x1p, y1p, x1s, y1s, x2p, y2p, x2s, y2s; uint16_t mult_a, mult_b; } hit;
	struct { int32_t x1p, y1p, z1p, x1s, y1s, z1s, x2p, y2p, z2p, x2s, y2s, z2s, x1po, y1po, z1po, x1so, y1so, z1so,
	         x2po, y2po, z2po, x2so, y2so, z2so, x_coll, y_coll, z_coll, x1tox2, y1toy2, z1toz2;
	         uint16_t flags, mode; } hit3;
	struct {
		int32_t status, command_offset;
		uint8_t decryption_key_byte, alternateswaps, shift, subtracttype, mode, blocksize_offset;
		uint16_t dataend, database;
		int32_t data_header[2];
		uint32_t writeaddress, writeaddress_current;
		uint16_t dsw_addr, eeprom_addr, poll_addr, checksumaddress;
	} c3;
	uint32_t rng;
	uint8_t oki_bank[2];
	uint8_t latch, zram[0x2000];
	uint64_t cpu_frac, out_frac, ym_frac, z80_frac, z80_target;
	uint32_t ay_frac;
	uint32_t oki_acc[2];
	uint32_t c3_frames;
	int32_t oki_hold[2];
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)

// the sprite frame buffers (MAME m_sprites_bitmap / m_sprites_maskmap), kept between frames
static uint16_t *s_sprbmp[2];
static uint8_t *s_sprmask[2];
#define SPR_W 512

// ---- drawing snapshot ----
static struct {
	uint16_t view2[2][0x4000 / 2];
	uint16_t v2regs[2][0x10];
	uint32_t pens[0x8000 + 2048];
	uint16_t spr[256 * 512];         // the visible rows of the sprite frame buffer (y from vy0)
	uint16_t bg15_scroll;
	uint8_t bg15_select, bg15_bright, disp_enable;
	int pal_dirty;
} *s_snap;
static uint16_t *s_bg15;   // Berlin Wall: the 32 decrypted 256x256 backgrounds (15-bit colours)

static inline int pal5(int v) { return (v << 3) | (v >> 2); }

// MAME running_machine::rand
static uint16_t rand16(void)
{
	b.rng = 1664525u * b.rng + 1013904223u;
	return (uint16_t)((b.rng >> 16) | (b.rng << 16));
}

// ---- sound ----
static uint8_t oki_read_bank(void *user, uint32_t a)
{
	const int c = (int)(intptr_t)user;
	const uint8_t *rom = s_oki_rom[c];
	const uint32_t size = s_oki_size[c];
	uint32_t off;
	const int bank = b.oki_bank[c];
	if (s_m == M_BAKUBRKR || s_m == M_WINGFORC) {   // bakubrkr_oki1_map: 0x20000 fixed, 0x20000 banked
		if (a < 0x20000)
			off = a;
		else {
			const int max_bank = (int)((size - 0x20000) / 0x20000);
			off = (bank < max_bank ? 0x20000 + (uint32_t)bank * 0x20000 : size - 0x20000) + (a - 0x20000);
		}
	} else if (s_calc3) {   // kaneko16_state common_oki_bank_install: oki1 0x30000 + 0x10000 banks, oki2 0x40000 banks
		if (c == 0) {          // (B.Rap Boys' oki2: 0x20000 + 0x20000)
			const int max_bank = (int)((size - 0x30000) / 0x10000);
			off = a < 0x30000 ? a : (bank < max_bank ? 0x30000 + (uint32_t)bank * 0x10000 : size - 0x10000) + (a - 0x30000);
		} else if (s_m == M_BRAPBOYS) {
			const int max_bank = (int)((size - 0x20000) / 0x20000);
			off = a < 0x20000 ? a : (bank < max_bank ? 0x20000 + (uint32_t)bank * 0x20000 : size - 0x20000) + (a - 0x20000);
		} else {
			const uint32_t banks = size / 0x40000;
			off = (banks ? rbx_wrap((uint32_t)bank, banks) : 0) * 0x40000 + a;
		}
	} else if (c == 0) {   // gtmr_oki1_map: 0x30000 fixed, 0x10000 banked (banks 0-2 = the first banked block)
		if (a < 0x30000)
			off = a;
		else
			off = (bank < 3 ? 0x30000 : (uint32_t)bank * 0x10000) + (a - 0x30000);
	} else {   // gtmr_oki2_map: 0x40000 banked
		const uint32_t banks = size / 0x40000;
		off = (banks ? rbx_wrap((uint32_t)bank, banks) : 0) * 0x40000 + a;
	}
	return off < size ? rom[off] : 0;
}

static void oki_bank_w(int c, uint8_t v) { b.oki_bank[c] = v; }

static uint8_t ay0_porta(void *u) { (void)u; return (uint8_t)rbx_port_read(s_port_dsw1); }
static uint8_t ay0_portb(void *u) { (void)u; return (uint8_t)rbx_port_read(s_port_dsw2); }
static void ay0_portb_w(void *u, uint8_t v) { (void)u; oki_bank_w(0, v & 7); }
static int64_t s_cyc_base;   // (the 68000's clock at the start of the current slice)
static int64_t cpu_now(void) { return s_cyc_base + m68k_cycles_run(); }
static uint8_t ay1_porta(void *u) { (void)u; return (uint8_t)eepser_do_read_93cxx(&s_eeprom, cpu_now()); }
static void ay1_portb_w(void *u, uint8_t v) { (void)u; eepser_cs_write(&s_eeprom, v != 0, cpu_now()); }

static uint16_t ym2149_r(int c, int off)
{
	ay8910_address_w(&s_ay[c], (uint8_t)off);
	return ay8910_data_r(&s_ay[c]);
}
static void ym2149_w(int c, int off, uint16_t data, int lo)
{
	ay8910_address_w(&s_ay[c], (uint8_t)off);
	ay8910_data_w(&s_ay[c], lo ? (uint8_t)data : (uint8_t)(data >> 8));
}

// ---- the TOYBOX MCU (simulation) ----
#include "kaneko_toybox.inc"

// ---- the CALC3 MCU (simulation) ----
#include "kaneko_calc3.inc"

// ---- the CALC hitbox (type 2: B.Rap Boys) ----
static int t2_compute(int x1, int w1, int x2, int w2)
{
	if (x2 >= x1 && x2 + w2 <= x1 + w1)
		return w2;
	if (x1 >= x2 && x1 + w1 <= x2 + w2)
		return w1;
	if (x2 < x1) {
		int t = x1; x1 = x2; x2 = t;
		t = w1; w1 = w2; w2 = t;
	}
	return x1 + w1 - x2;
}
static void t2_org(int mode, int x0, int s0, int32_t *x1, int32_t *s1)
{
	switch (mode) {
	case 0: *x1 = x0; *s1 = s0; break;
	case 1: *x1 = x0 - s0 / 2; *s1 = s0; break;
	case 2: *x1 = x0 - s0; *s1 = s0; break;
	case 3: *x1 = x0 - s0; *s1 = 2 * s0; break;
	}
}
static void t2_recalc(void)
{
	const int mode = b.hit3.mode;
	b.hit3.flags = 0;
	t2_org(mode & 3, b.hit3.x1po, b.hit3.x1so, &b.hit3.x1p, &b.hit3.x1s);
	t2_org((mode >> 2) & 3, b.hit3.y1po, b.hit3.y1so, &b.hit3.y1p, &b.hit3.y1s);
	t2_org((mode >> 4) & 3, b.hit3.z1po, b.hit3.z1so, &b.hit3.z1p, &b.hit3.z1s);
	t2_org((mode >> 8) & 3, b.hit3.x2po, b.hit3.x2so, &b.hit3.x2p, &b.hit3.x2s);
	t2_org((mode >> 10) & 3, b.hit3.y2po, b.hit3.y2so, &b.hit3.y2p, &b.hit3.y2s);
	t2_org((mode >> 12) & 3, b.hit3.z2po, b.hit3.z2so, &b.hit3.z2p, &b.hit3.z2s);
	b.hit3.x1tox2 = abs(b.hit3.x2po - b.hit3.x1po);
	b.hit3.y1toy2 = abs(b.hit3.y2po - b.hit3.y1po);
	b.hit3.z1toz2 = abs(b.hit3.z2po - b.hit3.z1po);
	b.hit3.x_coll = t2_compute(b.hit3.x1p, b.hit3.x1s, b.hit3.x2p, b.hit3.x2s);
	b.hit3.y_coll = t2_compute(b.hit3.y1p, b.hit3.y1s, b.hit3.y2p, b.hit3.y2s);
	b.hit3.z_coll = t2_compute(b.hit3.z1p, b.hit3.z1s, b.hit3.z2p, b.hit3.z2s);
	uint16_t f = 0;
	if (b.hit3.y1p > b.hit3.y2p) f |= 0x2000; else if (b.hit3.y1p == b.hit3.y2p) f |= 0x4000; else f |= 0x8000;
	if (b.hit3.y_coll < 0) f |= 0x1000;
	if (b.hit3.x1p > b.hit3.x2p) f |= 0x0200; else if (b.hit3.x1p == b.hit3.x2p) f |= 0x0400; else f |= 0x0800;
	if (b.hit3.x_coll < 0) f |= 0x0100;
	if (b.hit3.z1p > b.hit3.z2p) f |= 0x0020; else if (b.hit3.z1p == b.hit3.z2p) f |= 0x0040; else f |= 0x0080;
	if (b.hit3.z_coll < 0) f |= 0x0010;
	if (b.hit3.x_coll >= 0 && b.hit3.y_coll >= 0 && b.hit3.z_coll >= 0) f |= 0x0008;
	if (b.hit3.x_coll >= 0 && b.hit3.z_coll >= 0) f |= 0x0004;
	if (b.hit3.y_coll >= 0 && b.hit3.z_coll >= 0) f |= 0x0002;
	if (b.hit3.x_coll >= 0 && b.hit3.y_coll >= 0) f |= 0x0001;
	b.hit3.flags = f;
}
static void hit2_w(int off, uint16_t data, uint16_t mask)
{
	const int v = data & mask;
	switch (off * 4) {
	case 0x00: case 0x28: b.hit3.x1po = v; break;
	case 0x04: case 0x2c: b.hit3.x1so = v; break;
	case 0x08: case 0x30: b.hit3.y1po = v; break;
	case 0x0c: case 0x34: b.hit3.y1so = v; break;
	case 0x10: case 0x58: b.hit3.x2po = v; break;
	case 0x14: case 0x5c: b.hit3.x2so = v; break;
	case 0x18: case 0x60: b.hit3.y2po = v; break;
	case 0x1c: case 0x64: b.hit3.y2so = v; break;
	case 0x38: case 0x50: b.hit3.z1po = v; break;
	case 0x3c: case 0x54: b.hit3.z1so = v; break;
	case 0x20: case 0x68: b.hit3.z2po = v; break;
	case 0x24: case 0x6c: b.hit3.z2so = v; break;
	case 0x70: b.hit3.mode = (uint16_t)v; break;
	}
	t2_recalc();
}
static uint16_t hit2_r(int off)
{
	switch (off * 4) {
	case 0x00: case 0x10: return (uint16_t)b.hit3.x_coll;
	case 0x04: case 0x14: return (uint16_t)b.hit3.y_coll;
	case 0x18: return (uint16_t)b.hit3.z_coll;
	case 0x08: case 0x1c: return b.hit3.flags;
	case 0x28: return rand16();
	case 0x40: return (uint16_t)b.hit3.x1po;
	case 0x44: return (uint16_t)b.hit3.x1so;
	case 0x48: return (uint16_t)b.hit3.y1po;
	case 0x4c: return (uint16_t)b.hit3.y1so;
	case 0x50: return (uint16_t)b.hit3.z1po;
	case 0x54: return (uint16_t)b.hit3.z1so;
	case 0x58: return (uint16_t)b.hit3.x2po;
	case 0x5c: return (uint16_t)b.hit3.x2so;
	case 0x60: return (uint16_t)b.hit3.y2po;
	case 0x64: return (uint16_t)b.hit3.y2so;
	case 0x68: return (uint16_t)b.hit3.z2po;
	case 0x6c: return (uint16_t)b.hit3.z2so;
	case 0x80: return (uint16_t)b.hit3.x1tox2;
	case 0x84: return (uint16_t)b.hit3.y1toy2;
	case 0x88: return (uint16_t)b.hit3.z1toz2;
	default: return 0;
	}
}

// ---- the CALC hitbox ----
static int16_t calc_x(void)
{
	const int x1p = b.hit.x1p, x1s = b.hit.x1s, x2p = b.hit.x2p, x2s = b.hit.x2s;
	if (b.hit.x2p >= b.hit.x1p && x2p < x1p + x1s)
		return (int16_t)(x1s - (x2p - x1p));
	if (b.hit.x1p >= b.hit.x2p && x1p < x2p + x2s)
		return (int16_t)(x2s - (x1p - x2p));
	return (int16_t)((x1s + x2s) / 2 - abs((x1p + x1s / 2) - (x2p + x2s / 2)));
}
static int16_t calc_y(void)
{
	const int y1p = b.hit.y1p, y1s = b.hit.y1s, y2p = b.hit.y2p, y2s = b.hit.y2s;
	if (b.hit.y2p >= b.hit.y1p && y2p < y1p + y1s)
		return (int16_t)(y1s - (y2p - y1p));
	if (b.hit.y1p >= b.hit.y2p && y1p < y2p + y2s)
		return (int16_t)(y2s - (y1p - y2p));
	return (int16_t)((y1s + y2s) / 2 - abs((y1p + y1s / 2) - (y2p + y2s / 2)));
}

static uint16_t hit_r(int off)
{
	uint16_t data = 0;
	if (s_m == M_BONKADV) {   // type 0
		switch (off) {
		case 0x00 / 2: return 0;
		case 0x04 / 2: {
			if (b.hit.x1p > b.hit.x2p) data |= 0x0200;
			else if (b.hit.x1p == b.hit.x2p) data |= 0x0400;
			else data |= 0x0800;
			if (b.hit.y1p > b.hit.y2p) data |= 0x2000;
			else if (b.hit.y1p == b.hit.y2p) data |= 0x4000;
			else data |= 0x8000;
			const int16_t x12 = (int16_t)(b.hit.x1p - (b.hit.x2p + b.hit.x2s)), y12 = (int16_t)(b.hit.y1p - (b.hit.y2p + b.hit.y2s));
			const int16_t x21 = (int16_t)(b.hit.x1p + b.hit.x1s - b.hit.x2p), y21 = (int16_t)(b.hit.y1p + b.hit.y1s - b.hit.y2p);
			if (x12 < 0 && y12 < 0 && x21 >= 0 && y21 >= 0)
				data |= 0x0001;
			return data;
		}
		case 0x10 / 2: return (uint16_t)(((uint32_t)b.hit.mult_a * b.hit.mult_b) >> 16);
		case 0x12 / 2: return (uint16_t)((uint32_t)b.hit.mult_a * b.hit.mult_b);
		case 0x14 / 2: return rand16();
		default: return 0;
		}
	}
	const int16_t x_coll = calc_x(), y_coll = calc_y();   // type 1
	switch (off) {
	case 0x00 / 2: return (uint16_t)x_coll;
	case 0x02 / 2: return (uint16_t)y_coll;
	case 0x04 / 2:
		if (b.hit.y1p > b.hit.y2p) data |= 0x2000;
		else if (b.hit.y1p == b.hit.y2p) data |= 0x4000;
		else data |= 0x8000;
		if (y_coll < 0) data |= 0x1000;
		if (b.hit.x1p > b.hit.x2p) data |= 0x0200;
		else if (b.hit.x1p == b.hit.x2p) data |= 0x0400;
		else data |= 0x0800;
		if (x_coll < 0) data |= 0x0100;
		data |= 0x0040;
		if (x_coll >= 0) data |= 0x0004;
		if (y_coll >= 0) data |= 0x0002;
		if (x_coll >= 0 && y_coll >= 0) data |= 0x000f;
		return data;
	case 0x14 / 2: return rand16();
	case 0x20 / 2: return b.hit.x1p;
	case 0x22 / 2: return b.hit.x1s;
	case 0x24 / 2: return b.hit.y1p;
	case 0x26 / 2: return b.hit.y1s;
	case 0x2c / 2: return b.hit.x2p;
	case 0x2e / 2: return b.hit.x2s;
	case 0x30 / 2: return b.hit.y2p;
	case 0x32 / 2: return b.hit.y2s;
	default: return 0;
	}
}

static void hit_w(int off, uint16_t data, uint16_t mask)
{
	data &= mask;
	if (s_m == M_BONKADV) {
		switch (off) {
		case 0x00 / 2: b.hit.x1p = data; break;
		case 0x02 / 2: b.hit.x1s = data; break;
		case 0x04 / 2: b.hit.y1p = data; break;
		case 0x06 / 2: b.hit.y1s = data; break;
		case 0x08 / 2: b.hit.x2p = data; break;
		case 0x0a / 2: b.hit.x2s = data; break;
		case 0x0c / 2: b.hit.y2p = data; break;
		case 0x0e / 2: b.hit.y2s = data; break;
		case 0x10 / 2: b.hit.mult_a = data; break;
		case 0x12 / 2: b.hit.mult_b = data; break;
		}
		return;
	}
	switch (off) {
	case 0x20 / 2: b.hit.x1p = data; break;
	case 0x22 / 2: b.hit.x1s = data; break;
	case 0x24 / 2: b.hit.y1p = data; break;
	case 0x26 / 2: b.hit.y1s = data; break;
	case 0x2c / 2: b.hit.x2p = data; break;
	case 0x2e / 2: b.hit.x2s = data; break;
	case 0x30 / 2: b.hit.y2p = data; break;
	case 0x32 / 2: b.hit.y2s = data; break;
	}
}

// ---- sprite registers ----
static void spr_regs_w(int off, uint16_t data, uint16_t mask)
{
	b.spr_regs[off] = (uint16_t)((b.spr_regs[off] & ~mask) | (data & mask));
	if (off == 0 && (mask & 0x00ff)) {
		const uint16_t v = b.spr_regs[0];
		b.spr_flipx = (v >> 1) & 1;
		b.spr_flipy = v & 1;
		if (s_mc->spr_type == 0)
			b.keep_sprites = !((v >> 2) & 1);
	}
}

// Berlin Wall's scrambled sprite RAM / registers
static int berl_spr_off(int off)   // (word offset in 0x1000 words)
{
	const int o = (off & 0xf003) | ((off & 0x0004) << 9) | ((off & 0x0ff8) >> 1);
	return o ^ 0x800;
}
static int berl_reg_off(int off)
{
	return (off & 0xe3) | ((off & 0x04) << 2) | ((off & 0x10) >> 1) | ((off & 0x08) >> 1);
}

// ---- the main CPU's handlers ----
static uint16_t wheel_r(void)
{
	const uint32_t dsw = rbx_port_read(s_port_dsw1);
	if (s_m == M_GTMR2) {
		switch (dsw & 0x1800) {
		case 0x0000: return (uint16_t)rbx_port_read(s_port_wheel[0]);
		case 0x1000: return (uint16_t)(rbx_port_read(s_port_wheel[1]) << 8);
		case 0x0800: return (uint16_t)(rbx_port_read(s_port_wheel[2]) << 8);
		default: return 0xffff;
		}
	}
	return (uint16_t)rbx_port_read((dsw & 0x1000) ? s_port_wheel[1] : s_port_wheel[0]);
}

static uint16_t port16(int p) { return p >= 0 ? (uint16_t)rbx_port_read(p) : 0xffff; }

static void soundlatch_w(uint8_t v)
{
	b.latch = v;
	z80_gen_nmi(&s_z80);
}

// the board's I/O (not RAM): read a word at a (even)
static uint16_t io_r16(uint32_t a)
{
	switch (s_m) {
	case M_BERLWALL:
		if (a >= 0x30e000 && a < 0x310000)
			return b.spr[berl_spr_off((int)((a - 0x30e000) >> 1))];
		if (a == 0x480000) return b.bg15_scroll;
		if (a == 0x500000) return b.bg15_bright;
		if (a == 0x580000) return b.bg15_select;
		if (a >= 0x600000 && a < 0x600040) {
			const int off = (int)((a - 0x600000) >> 1);
			return (off & 4) ? 0 : b.spr_regs[berl_reg_off(off) & 0xf];
		}
		if (a == 0x680000) return port16(s_port_p1);
		if (a == 0x680002) return port16(s_port_p2);
		if (a == 0x680004) return port16(s_port_sys);
		if (a >= 0x800000 && a < 0x800020) return ym2149_r(0, (int)((a >> 1) & 15));
		if (a >= 0x800200 && a < 0x800220) return ym2149_r(1, (int)((a >> 1) & 15));
		if (a == 0x800400) return oki_read(&s_oki[0]);
		if (a >= 0xd00000 && a < 0xd00020) return b.v2regs[0][(a >> 1) & 0xf];
		return 0;
	case M_BAKUBRKR:
		if (a >= 0x400000 && a < 0x400020) return ym2149_r(0, (int)((a >> 1) & 15));
		if (a >= 0x400200 && a < 0x400220) return ym2149_r(1, (int)((a >> 1) & 15));
		if (a == 0x400400) return oki_read(&s_oki[0]);
		if (a >= 0x800000 && a < 0x800020) return b.v2regs[0][(a >> 1) & 0xf];
		if (a >= 0x900000 && a < 0x900020) return b.spr_regs[(a >> 1) & 0xf];
		if (a >= 0xb00000 && a < 0xb00020) return b.v2regs[1][(a >> 1) & 0xf];
		if (a == 0xe00000) return port16(s_port_p1);
		if (a == 0xe00002) return port16(s_port_p2);
		if (a == 0xe00004) return port16(s_port_sys);
		if (a == 0xe00006) return port16(s_port_unk);
		return 0;
	case M_MGCRYSTL:
		if (a >= 0x400000 && a < 0x400020) return ym2149_r(0, (int)((a >> 1) & 15));
		if (a >= 0x400200 && a < 0x400220) return ym2149_r(1, (int)((a >> 1) & 15));
		if (a == 0x400400) return oki_read(&s_oki[0]);
		if (a >= 0x800000 && a < 0x800020) return b.v2regs[0][(a >> 1) & 0xf];
		if (a >= 0x900000 && a < 0x900020) return b.spr_regs[(a >> 1) & 0xf];
		if (a >= 0xb00000 && a < 0xb00020) return b.v2regs[1][(a >> 1) & 0xf];
		if (a == 0xc00000) return port16(s_port_p1);
		if (a == 0xc00002) return port16(s_port_p2);
		if (a == 0xc00004) return port16(s_port_sys);
		return 0;
	case M_BLAZEON:
	case M_WINGFORC:
		if (a >= 0x800000 && a < 0x800020) return b.v2regs[0][(a >> 1) & 0xf];
		if (a >= 0x900000 && a < 0x900020) return b.spr_regs[(a >> 1) & 0xf];
		if (a == 0xc00000) return port16(s_port_p1);
		if (a == 0xc00002) return port16(s_port_p2);
		if (a == 0xc00004) return port16(s_port_unk);
		if (a == 0xc00006) return port16(s_port_sys);
		return 0;
	case M_SHOGWARR:
	case M_BRAPBOYS:
		if (a == 0x400000) return oki_read(&s_oki[0]);
		if (a == 0x480000) return oki_read(&s_oki[1]);
		if (a >= 0x800000 && a < 0x800020) return b.v2regs[0][(a >> 1) & 0xf];
		if (a >= 0x900000 && a < 0x900020) return b.spr_regs[(a >> 1) & 0xf];
		if (a >= 0xa00000 && a < 0xa00080) return s_m == M_BRAPBOYS ? hit2_r((int)((a - 0xa00000) >> 1)) : hit_r((int)((a - 0xa00000) >> 1));
		if (a == 0xb80000) return port16(s_port_p1);
		if (a == 0xb80002) return port16(s_port_p2);
		if (a == 0xb80004) return port16(s_port_sys);
		if (a == 0xb80006) return port16(s_port_unk);
		return 0;
	default:   // the TOYBOX boards
		if (a == 0x0ffffe && (s_m == M_GTMR || s_m == M_GTMRE || s_m == M_GTMR2)) return wheel_r();
		if (a >= 0x600000 && a < 0x600020) return b.v2regs[0][(a >> 1) & 0xf];
		if (a >= 0x680000 && a < 0x680020) return b.v2regs[1][(a >> 1) & 0xf];
		if (a >= 0x700000 && a < 0x700020) return b.spr_regs[(a >> 1) & 0xf];
		if (a == 0x800000) return oki_read(&s_oki[0]);
		if (a == 0x880000) return oki_read(&s_oki[1]);
		if (a >= 0x900000 && a < (s_m == M_BONKADV ? 0x900016u : 0x90003au)) return hit_r((int)((a - 0x900000) >> 1));
		if (a == 0xb00000) return port16(s_port_p1);
		if (a == 0xb00002) {
			if (s_m == M_GTMR2)
				return (uint16_t)(port16(s_port_p2) & (port16(s_port_fake) | ~0x7100));
			return port16(s_port_p2);
		}
		if (a == 0xb00004) return port16(s_port_sys);
		if (a == 0xb00006) return port16(s_m == M_BLOODWAR || s_m == M_GTMR2 ? s_port_extra : s_port_unk);
		if (a == 0xd00000) return 0;   // (mcu_status_r)
		return 0;
	}
}

static void io_w16(uint32_t a, uint16_t data, uint16_t mask)
{
	const int lo = (mask & 0x00ff) != 0;
	switch (s_m) {
	case M_BERLWALL:
		if (a >= 0x30e000 && a < 0x310000) {
			uint16_t *w = &b.spr[berl_spr_off((int)((a - 0x30e000) >> 1))];
			*w = (uint16_t)((*w & ~mask) | (data & mask));
		} else if (a == 0x480000)
			b.bg15_scroll = (uint16_t)((b.bg15_scroll & ~mask) | (data & mask));
		else if (a == 0x500000) {
			if (lo) b.bg15_bright = (uint8_t)data;
		} else if (a == 0x580000) {
			if (lo) b.bg15_select = (uint8_t)data;
		} else if (a >= 0x600000 && a < 0x600040) {
			const int off = (int)((a - 0x600000) >> 1);
			if (!(off & 4))
				spr_regs_w(berl_reg_off(off) & 0xf, data, mask);
		} else if (a >= 0x800000 && a < 0x800020)
			ym2149_w(0, (int)((a >> 1) & 15), data, lo);
		else if (a >= 0x800200 && a < 0x800220)
			ym2149_w(1, (int)((a >> 1) & 15), data, lo);
		else if (a == 0x800400)   // berlwall_oki_w: the upper byte alone mirrors the lower one
			oki_write(&s_oki[0], mask == 0xff00 ? (uint8_t)(data >> 8) : (uint8_t)data);
		else if (a >= 0xd00000 && a < 0xd00020) {
			uint16_t *w = &b.v2regs[0][(a >> 1) & 0xf];
			*w = (uint16_t)((*w & ~mask) | (data & mask));
		}
		return;
	case M_BAKUBRKR:
	case M_MGCRYSTL:
		if (a >= 0x400000 && a < 0x400020)
			ym2149_w(0, (int)((a >> 1) & 15), data, lo);
		else if (a >= 0x400200 && a < 0x400220)
			ym2149_w(1, (int)((a >> 1) & 15), data, lo);
		else if (a == 0x400400) {
			if (lo) oki_write(&s_oki[0], (uint8_t)data);
		} else if (a >= 0x800000 && a < 0x800020) {
			uint16_t *w = &b.v2regs[0][(a >> 1) & 0xf];
			*w = (uint16_t)((*w & ~mask) | (data & mask));
		} else if (a >= 0x900000 && a < 0x900020)
			spr_regs_w((int)((a >> 1) & 0xf), data, mask);
		else if (a >= 0xb00000 && a < 0xb00020) {
			uint16_t *w = &b.v2regs[1][(a >> 1) & 0xf];
			*w = (uint16_t)((*w & ~mask) | (data & mask));
		} else if (a == 0xd00000 && lo && s_has_eeprom) {   // eeprom_w: EEPROMOUT (bit 0 clk, then bit 1 di)
			eepser_clk_write(&s_eeprom, data & 1, cpu_now());
			eepser_di_write(&s_eeprom, (data >> 1) & 1);
		}
		return;
	case M_BLAZEON:
	case M_WINGFORC:
		if (a >= 0x800000 && a < 0x800020) {
			uint16_t *w = &b.v2regs[0][(a >> 1) & 0xf];
			*w = (uint16_t)((*w & ~mask) | (data & mask));
		} else if (a >= 0x900000 && a < 0x900020)
			spr_regs_w((int)((a >> 1) & 0xf), data, mask);
		else if (a == 0xe00000 && (mask & 0xff00))
			soundlatch_w((uint8_t)(data >> 8));
		return;
	case M_SHOGWARR:
	case M_BRAPBOYS:
		if (a == 0x280000) b.c3.status |= 1;
		else if (a == 0x290000) b.c3.status |= 2;
		else if (a == 0x2b0000) b.c3.status |= 4;
		else if (a == 0x2d0000) b.c3.status |= 8;
		else if (a == 0x400000) {
			if (lo) oki_write(&s_oki[0], (uint8_t)data);
		} else if (a == 0x480000) {
			if (lo) oki_write(&s_oki[1], (uint8_t)data);
		} else if (a >= 0x800000 && a < 0x800020) {
			uint16_t *w = &b.v2regs[0][(a >> 1) & 0xf];
			*w = (uint16_t)((*w & ~mask) | (data & mask));
		} else if (a >= 0x900000 && a < 0x900020)
			spr_regs_w((int)((a >> 1) & 0xf), data, mask);
		else if (a >= 0xa00000 && a < 0xa00080) {
			if (s_m == M_BRAPBOYS)
				hit2_w((int)((a - 0xa00000) >> 1), data, mask);
			else
				hit_w((int)((a - 0xa00000) >> 1), data, mask);
		} else if (a == 0xe00000 && lo) {   // shogwarr_oki_bank_w
			oki_bank_w(0, (uint8_t)((data >> 4) & 0xf));
			oki_bank_w(1, (uint8_t)(data & 0xf));
		}
		return;
	default:
		if (a >= 0x2a0000 && a < 0x2e0000 && !(a & 0xffff) && a != 0x2e0000) {
			const int n = (int)((a - 0x2a0000) >> 16);
			if (n < 4)
				toybox_com_w(n, data, mask);
		} else if (a >= 0x600000 && a < 0x600020) {
			uint16_t *w = &b.v2regs[0][(a >> 1) & 0xf];
			*w = (uint16_t)((*w & ~mask) | (data & mask));
		} else if (a >= 0x680000 && a < 0x680020) {
			uint16_t *w = &b.v2regs[1][(a >> 1) & 0xf];
			*w = (uint16_t)((*w & ~mask) | (data & mask));
		} else if (a >= 0x700000 && a < 0x700020)
			spr_regs_w((int)((a >> 1) & 0xf), data, mask);
		else if (a == 0x800000) {
			if (lo) oki_write(&s_oki[0], (uint8_t)data);
		} else if (a == 0x880000) {
			if (lo) oki_write(&s_oki[1], (uint8_t)data);
		} else if (a >= 0x900000 && a < (s_m == M_BONKADV ? 0x900016u : 0x90003au))
			hit_w((int)((a - 0x900000) >> 1), data, mask);
		else if (a == 0xc00000)
			b.disp_enable = (uint16_t)((b.disp_enable & ~mask) | (data & mask));
		else if (a == 0xe00000 && lo)
			oki_bank_w(0, (uint8_t)(data & 0xf));
		else if (a == 0xe80000 && lo)
			oki_bank_w(1, (uint8_t)(data & (s_m == M_BLOODWAR ? 0x3 : s_m == M_BONKADV ? 0xf : 0x1)));
		return;
	}
}

static uint8_t r8(uint32_t a)
{
	const uint16_t w = io_r16(a & 0xfffffe);
	return (a & 1) ? (uint8_t)w : (uint8_t)(w >> 8);
}
static uint16_t r16(uint32_t a) { return io_r16(a & 0xfffffe); }
static void w8(uint32_t a, uint8_t v)
{
	if (a & 1)
		io_w16(a & 0xfffffe, v, 0x00ff);
	else
		io_w16(a, (uint16_t)(v << 8), 0xff00);
}
static void w16(uint32_t a, uint16_t v) { io_w16(a & 0xfffffe, v, 0xffff); }

static void map_mem(void)
{
	m68k_bus_reset_map();
	m68k_bus.read8 = r8;
	m68k_bus.read16 = r16;
	m68k_bus.write8 = w8;
	m68k_bus.write16 = w16;
	switch (s_m) {
	case M_BERLWALL:
		m68k_bus_map(0x000000, 0x03ffff, s_rom, s_rom_size, 1, 0);
		m68k_bus_map(0x200000, 0x20ffff, b.ram, sizeof(b.ram), 1, 1);
		m68k_bus_map(0x400000, 0x400fff, b.pal, 0x1000, 1, 1);
		m68k_bus_map(0xc00000, 0xc03fff, b.view2[0], 0x4000, 1, 1);
		break;
	case M_BAKUBRKR:
		m68k_bus_map(0x000000, 0x07ffff, s_rom, s_rom_size, 1, 0);
		m68k_bus_map(0x100000, 0x10ffff, b.ram, sizeof(b.ram), 1, 1);
		m68k_bus_map(0x500000, 0x503fff, b.view2[0], 0x4000, 1, 1);
		m68k_bus_map(0x580000, 0x583fff, b.view2[1], 0x4000, 1, 1);
		m68k_bus_map(0x600000, 0x601fff, b.spr, 0x2000, 1, 1);
		m68k_bus_map(0x700000, 0x700fff, b.pal, 0x1000, 1, 1);
		break;
	case M_MGCRYSTL:
		m68k_bus_map(0x000000, 0x07ffff, s_rom, s_rom_size, 1, 0);
		m68k_bus_map(0x300000, 0x30ffff, b.ram, sizeof(b.ram), 1, 1);
		m68k_bus_map(0x500000, 0x500fff, b.pal, 0x1000, 1, 1);
		m68k_bus_map(0x600000, 0x603fff, b.view2[0], 0x4000, 1, 1);
		m68k_bus_map(0x680000, 0x683fff, b.view2[1], 0x4000, 1, 1);
		m68k_bus_map(0x700000, 0x701fff, b.spr, 0x2000, 1, 1);
		break;
	case M_BLAZEON:
	case M_WINGFORC:
		m68k_bus_map(0x000000, 0x0fffff, s_rom, s_rom_size, 1, 0);
		m68k_bus_map(0x300000, 0x30ffff, b.ram, sizeof(b.ram), 1, 1);
		m68k_bus_map(0x500000, 0x500fff, b.pal, 0x1000, 1, 1);
		m68k_bus_map(0x600000, 0x603fff, b.view2[0], 0x4000, 1, 1);
		m68k_bus_map(0x700000, 0x700fff, b.spr, 0x1000, 1, 1);
		m68k_bus_map(0x980000, 0x980fff, b.ram2, 0x1000, 1, 1);   // (Sprites Regs #2: RAM)
		break;
	case M_SHOGWARR:
	case M_BRAPBOYS:
		m68k_bus_map(0x000000, 0x03ffff, s_rom, s_rom_size, 1, 0);
		m68k_bus_map(0x100000, 0x10ffff, b.ram, sizeof(b.ram), 1, 1);
		m68k_bus_map(0x200000, 0x20ffff, b.mcuram, sizeof(b.mcuram), 1, 1);
		m68k_bus_map(0x380000, 0x380fff, b.pal, 0x1000, 1, 1);
		m68k_bus_map(0x580000, 0x581fff, b.spr, 0x2000, 1, 1);
		m68k_bus_map(0x600000, 0x603fff, b.view2[0], 0x4000, 1, 1);
		break;
	default:
		m68k_bus_map(0x000000, 0x0fefff, s_rom, s_rom_size, 1, 0);
		if (s_m == M_BLOODWAR || s_m == M_BONKADV)
			m68k_bus_map(0x0ff000, 0x0fffff, s_rom + 0xff000, 0x1000, 1, 0);
		else
			m68k_bus_map(0x0ff000, 0x0ffffd, s_rom + 0xff000, 0xffe, 1, 0);
		m68k_bus_map(0x100000, 0x10ffff, b.ram, sizeof(b.ram), 1, 1);
		m68k_bus_map(0x200000, 0x20ffff, b.mcuram, sizeof(b.mcuram), 1, 1);
		m68k_bus_map(0x300000, 0x30ffff, b.pal, 0x10000, 1, 1);
		if (s_m == M_GTMR || s_m == M_GTMRE || s_m == M_GTMR2)
			m68k_bus_map(0x310000, 0x327fff, b.ram2, sizeof(b.ram2), 1, 1);
		m68k_bus_map(0x400000, 0x401fff, b.spr, 0x2000, 1, 1);
		m68k_bus_map(0x500000, 0x503fff, b.view2[0], 0x4000, 1, 1);
		m68k_bus_map(0x580000, 0x583fff, b.view2[1], 0x4000, 1, 1);
		break;
	}
}

// ---- the sound CPU (Blaze On, Wing Force) ----
static uint8_t z80_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xc000)
		return a < s_zrom_size ? s_zrom[a] : 0xff;
	if (a < 0xe000)
		return b.zram[a - 0xc000];
	return 0xff;
}
static void z80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xe000)
		b.zram[a - 0xc000] = v;
}
static uint8_t z80_in(z80 *z, uint8_t port)
{
	(void)z;
	switch (port) {
	case 0x02: case 0x03: return rbx_ym_read(0, port & 1);
	case 0x06: return b.latch;
	case 0x0a: return s_m == M_WINGFORC ? oki_read(&s_oki[0]) : 0xff;
	default: return 0xff;
	}
}
static void z80_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	switch (port) {
	case 0x02: case 0x03: rbx_ym_write(0, port & 1, v); break;
	case 0x0a: if (s_m == M_WINGFORC) oki_write(&s_oki[0], v); break;
	case 0x0c: if (s_m == M_WINGFORC) oki_bank_w(0, v & 3); break;
	}
}

// ---- video ----
// MAME tilemap mapping (effective_rowscroll / colscroll with the flip around the visible area's centre)
static inline int wrap(int v, int n) { return v < 0 ? n - (-v) % n : v % n; }

// a 4 bpp 16x16 tile pixel (gfx_8x8x4_row_2x2_group_packed lsb / msb)
static inline int tile4(const uint8_t *t, int x, int y, int msb)
{
	const uint8_t v = t[((y & 8) << 3) | ((x & 8) << 2) | ((y & 7) << 2) | ((x & 7) >> 1)];
	return ((x & 1) ^ msb) ? (v >> 4) : (v & 15);
}

// one VIEW2 layer (l 0: vram_0 / scroll_0, 1: vram_1 / scroll_1) of chip c on screen line y, every category at
// once: MAME draws the categories 0-7 in turn, each over the four layers in order, so a pixel shows the opaque one of
// the highest key (category * 4 + layer order L); prival as MAME's priority buffer value (the second chip's: 0 unless
// view2_2_pri)
static void draw_layer_line(int c, int l, int y, int L, int pri_cat, uint8_t *best, uint16_t *d, uint8_t *pr)
{
	const uint16_t *regs = s_snap->v2regs[c];
	const uint16_t ctrl = regs[4];
	if (l == 0 ? (ctrl & 0x1000) : (ctrl & 0x0010))
		return;
	const int flipy = (ctrl >> 8) & 1, flipx = (ctrl >> 9) & 1;
	const uint16_t *vram = s_snap->view2[c] + (l == 0 ? 0x800 : 0x000);
	const uint16_t *lscroll = s_snap->view2[c] + (l == 0 ? 0x1800 : 0x1000);
	const int lsen = l == 0 ? (ctrl >> 11) & 1 : (ctrl >> 3) & 1;
	const uint16_t scx = l == 0 ? regs[2] : regs[0];
	const int scy = (l == 0 ? regs[3] : regs[1]) >> 6;
	const int ldx = s_mc->dx + (l ? 2 : 0);
	const int dx = -ldx, dxf = s_mc->xdim + ldx - 1, dy = -s_mc->dy, dyf = s_mc->ydim + s_mc->dy - 1;
	const int xext = s_mc->vx0 + s_mc->vx1 + 1, yext = s_mc->vy0 + s_mc->vy1 + 1;
	const int yv = wrap(!flipy ? dy - scy : yext - 512 - (dyf - scy), 512);
	const int py = wrap(y - yv, 512);
	const int ly = flipy ? 511 - py : py;
	const int rs = (scx + (lsen ? lscroll[ly] : 0)) >> 6;
	const int xv = wrap(!flipx ? dx - rs : xext - 512 - (dxf - rs), 512);
	const uint8_t *gfx = s_tiles[c];
	const uint32_t count = s_tile_count[c];
	if (!count)
		return;
	const uint16_t *row = vram + (ly >> 4) * 64;
	const int x1 = s_mc->vx1;
	for (int x = s_mc->vx0; x <= x1;) {
		// (a span of one tile column: the logical x runs up (no flip) or down (flip) by one a pixel)
		const int px = (x - xv) & 511;
		const int lx = flipx ? 511 - px : px;
		const int fx0 = lx & 15;
		int n = flipx ? fx0 + 1 : 16 - fx0;
		if (n > x1 - x + 1)
			n = x1 - x + 1;
		const uint16_t attr = row[(lx >> 4) * 2], code = row[(lx >> 4) * 2 + 1];
		const int cat = (attr >> 8) & 7;
		const int key = cat * 4 + L;
		const uint8_t *t = gfx + rbx_wrap(code, count) * 128;
		const int tflipx = attr & 1;
		const int fy = (attr & 2) ? 15 - (ly & 15) : (ly & 15);
		const uint16_t pbase = (uint16_t)(s_mc->view2_colbase + ((attr >> 2) & 0x3f) * 16);
		const uint8_t prival = (uint8_t)(pri_cat ? cat : 0);
		const int step = flipx ? -1 : 1;
		const uint8_t *r = t + ((fy & 8) << 3) + ((fy & 7) << 2);
		if (!(r[0] | r[1] | r[2] | r[3] | r[32] | r[33] | r[34] | r[35])) {   // (a transparent row)
			x += n;
			continue;
		}
		uint8_t tp[16];   // (the tile's row, lsb nibble first)
		for (int k = 0; k < 4; k++) {
			tp[2 * k] = (uint8_t)(r[k] & 15);
			tp[2 * k + 1] = (uint8_t)(r[k] >> 4);
			tp[8 + 2 * k] = (uint8_t)(r[32 + k] & 15);
			tp[8 + 2 * k + 1] = (uint8_t)(r[32 + k] >> 4);
		}
		for (int i = 0, fx = fx0; i < n; i++, fx += step) {
			const int pix = tp[tflipx ? 15 - fx : fx];
			const int xx = x + i;
			if (pix && key >= best[xx]) {
				best[xx] = (uint8_t)key;
				d[xx] = (uint16_t)(pbase + pix);
				pr[xx] = prival;
			}
		}
		x += n;
	}
}

#define BANDS 8
static void ka_band(int band)
{
	const machine_t *mc = s_mc;
	const int w = mc->vx1 - mc->vx0 + 1, h = mc->vy1 - mc->vy0 + 1;
	const int r0 = mc->vy0 + h * band / BANDS, r1 = mc->vy0 + h * (band + 1) / BANDS;
	for (int y = r0; y < r1; y++) {
		uint16_t line[512];
		uint8_t pr[512], best[512];
		uint16_t *d = line;
		memset(pr, 0, sizeof(pr));
		memset(best, 0, sizeof(best));
		if (s_m == M_BERLWALL) {   // the high colour background
			const int screen = s_snap->bg15_select & 0x1f, flip = (s_snap->bg15_select >> 5) & 1;
			int scrollx = s_snap->bg15_scroll & 0xff, scrolly = (s_snap->bg15_scroll >> 8) & 0xff;
			if (!flip) { scrollx -= 0x80; scrolly -= 0x08; }
			else { scrollx -= 0xff - 0x80; scrolly -= 0xff - 0x08; }
			const uint16_t *src = s_bg15 + screen * 65536 + ((flip ? ~(y - scrolly) : (y - scrolly)) & 0xff) * 256;
			for (int x = mc->vx0; x <= mc->vx1; x++)
				d[x] = (uint16_t)(2048 + src[(flip ? ~(x - scrollx) : (x - scrollx)) & 0xff]);
		} else {
			const uint16_t pen = mc->spr_type == 1 ? 0x7f00 : 0;
			for (int x = mc->vx0; x <= mc->vx1; x++)
				d[x] = pen;
		}
		if (s_snap->disp_enable) {
			draw_layer_line(0, 0, y, 0, 1, best, d, pr);
			draw_layer_line(0, 1, y, 1, 1, best, d, pr);
			if (mc->nview2 > 1) {
				draw_layer_line(1, 0, y, 2, mc->v2pri, best, d, pr);
				draw_layer_line(1, 1, y, 3, mc->v2pri, best, d, pr);
			}
			const uint16_t *sp = s_snap->spr + (y - mc->vy0) * 512;
			for (int x = mc->vx0; x <= mc->vx1; x++) {   // copybitmap
				const uint16_t v = sp[x];
				const uint16_t pix = v & 0x3fff;
				if (pix && mc->spr_pri[v >> 14] > pr[x])
					d[x] = (uint16_t)(mc->spr_colbase + pix);
			}
		}
		memcpy(rbx_video.bitmap + (y - mc->vy0) * rbx_video.pitch, d + mc->vx0, sizeof(uint16_t) * (size_t)w);
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, ka_band };
static int ka_help(int helper) { return decoband_help(&s_bands, helper); }

static void ka_render(void)
{
	memcpy(rbx_video.palette, s_snap->pens, sizeof(uint32_t) * (size_t)rbx_video.palette_size);
	decoband_run(&s_bands);
}

// ---- sprites (render_sprites at line 224) ----
static void draw_one(int code, int color, int flipx, int flipy, int sx, int sy, int priority)
{
	const machine_t *mc = s_mc;
	const int x0 = mc->vx0, x1 = mc->vx1, y0 = mc->vy0, y1 = mc->vy1;
	if (!s_spr_count || sx > x1 || sx + 15 < x0 || sy > y1 || sy + 15 < y0)
		return;
	uint16_t *bmp = s_sprbmp[b.buffer];
	uint8_t *mask = s_sprmask[b.buffer];
	const uint32_t t = rbx_wrap((uint32_t)code, s_spr_count);
	const uint8_t *g = s_spr_gfx + t * (mc->spr_type ? 256u : 128u);
	const int pen_base = mc->spr_type ? (color & 0x3f) * 256 : (color & 0x3f) * 16;
	const uint16_t pv = (uint16_t)((priority & 3) << 14);
	const int xs = sx < x0 ? x0 - sx : 0, xe = sx + 15 > x1 ? x1 - sx : 15;
	const int ys = sy < y0 ? y0 - sy : 0, ye = sy + 15 > y1 ? y1 - sy : 15;
	for (int yy = ys; yy <= ye; yy++) {
		const int ty = flipy ? 15 - yy : yy;
		uint16_t *d = bmp + (sy + yy) * SPR_W + sx;
		uint8_t *m = mask + (sy + yy) * SPR_W + sx;
		uint8_t px[16];
		if (mc->spr_type) {   // (16x16x8 made of four 8x8x8 blocks: 01 / 23)
			const uint8_t *r = g + ((ty & 8) << 4) + ((ty & 7) << 3);
			memcpy(px, r, 8);
			memcpy(px + 8, r + 64, 8);
		} else {
			const uint8_t *r = g + ((ty & 8) << 3) + ((ty & 7) << 2);
			for (int k = 0; k < 4; k++) {
				px[2 * k] = (uint8_t)(r[k] >> 4);
				px[2 * k + 1] = (uint8_t)(r[k] & 15);
				px[8 + 2 * k] = (uint8_t)(r[32 + k] >> 4);
				px[8 + 2 * k + 1] = (uint8_t)(r[32 + k] & 15);
			}
		}
		for (int xx = xs; xx <= xe; xx++) {
			const int c = px[flipx ? 15 - xx : xx];
			if (c) {
				if (!m[xx])
					d[xx] = (uint16_t)(((pen_base + c) & 0x3fff) + pv);
				m[xx] = 0xff;
			}
		}
	}
}

typedef struct { int x, y, code, color, priority, xoffs, yoffs, flipx, flipy; } tsprite;
static tsprite (*s_ts_hp)[0x400];
#define s_ts (*s_ts_hp)

static void render_sprites(void)
{
	const machine_t *mc = s_mc;
	b.buffer ^= 1;
	uint16_t *bmp = s_sprbmp[b.buffer];
	uint8_t *mask = s_sprmask[b.buffer];
	for (int y = mc->vy0; y <= mc->vy1; y++) {
		memset(mask + y * SPR_W + mc->vx0, 0, (size_t)(mc->vx1 - mc->vx0 + 1));
		if (!b.keep_sprites)
			memset(bmp + y * SPR_W + mc->vx0, 0, sizeof(uint16_t) * (size_t)(mc->vx1 - mc->vx0 + 1));
	}
	const uint16_t *ram = b.spr_buf;
	const int words = (s_m == M_BLAZEON || s_m == M_WINGFORC) ? 0x800 : 0x1000;
	const int max = mc->sw > 0x100 ? (0x200 << 6) : (0x100 << 6);
	int x = 0, y = 0, code = 0, color = 0, priority = 0, xoffs = 0, yoffs = 0, flipx = 0, flipy = 0;
	int n = 0;
	for (int i = 0; i < 0x400; i++) {
		const int offs = i * 4;
		if (offs >= words)
			break;
		tsprite *s = &s_ts[n];
		const uint16_t attr = ram[offs];
		s->code = ram[offs + 1];
		s->x = ram[offs + 2];
		s->y = ram[offs + 3];
		if (mc->spr_type == 1) {
			s->color = attr & 0x3f;
			s->priority = (attr >> 6) & 3;
			s->flipy = (attr >> 8) & 1;
			s->flipx = (attr >> 9) & 1;
			s->code += (s->y & 1) << 16;
		} else {
			s->flipy = attr & 1;
			s->flipx = (attr >> 1) & 1;
			s->color = (attr >> 2) & 0x3f;
			s->priority = (attr >> 8) & 3;
		}
		const int xi = (attr >> 11) & 3;
		s->yoffs = (uint16_t)b.spr_regs[8 + xi * 2 + 1];
		s->xoffs = (uint16_t)b.spr_regs[8 + xi * 2 + 0];
		s->yoffs = (uint16_t)(s->yoffs - b.spr_regs[1]);
		if (b.spr_flipy)
			s->yoffs = (uint16_t)(s->yoffs - (mc->vy0 << 6));
		else
			s->yoffs = (uint16_t)(s->yoffs + (mc->vy0 << 6));
		if (attr & 0x8000)
			s->code = ++code;
		else
			code = s->code;
		if (attr & 0x4000) {
			s->color = color;
			s->priority = priority;
			s->xoffs = xoffs;
			s->yoffs = yoffs;
			if (!mc->fliptype) {
				s->flipx = flipx;
				s->flipy = flipy;
			}
		} else {
			color = s->color;
			priority = s->priority;
			xoffs = s->xoffs;
			yoffs = s->yoffs;
			if (!mc->fliptype) {
				flipx = s->flipx;
				flipy = s->flipy;
			}
		}
		if (mc->fliptype) {   // (B.Rap Boys: the flips not latched)
			flipx = s->flipx;
			flipy = s->flipy;
		}
		if (attr & 0x2000) {
			s->x += x;
			s->y += y;
		}
		x = s->x;
		y = s->y;
		s->x = s->xoffs + s->x;
		s->y = s->yoffs + s->y;
		s->x += mc->spr_xoffs;
		s->y += mc->spr_yoffs;
		if (b.spr_flipx) { s->x = max - s->x - (16 << 6); s->flipx = !s->flipx; }
		if (b.spr_flipy) { s->y = max - s->y - (16 << 6); s->flipy = !s->flipy; }
		s->x = ((s->x & 0x7fc0) - (s->x & 0x8000)) / 0x40;
		s->y = ((s->y & 0x7fc0) - (s->y & 0x8000)) / 0x40;
		n++;
	}
	for (int i = n - 1; i >= 0; i--)
		draw_one(s_ts[i].code, s_ts[i].color, s_ts[i].flipx, s_ts[i].flipy, s_ts[i].x, s_ts[i].y, s_ts[i].priority);
	(void)bmp;
}

static void vblank_capture(void)
{
	const machine_t *mc = s_mc;
	rbx_video_wait_idle();
	memcpy(s_snap->view2, b.view2, sizeof(b.view2));
	memcpy(s_snap->v2regs, b.v2regs, sizeof(b.v2regs));
	for (int i = 0; i < mc->pal_size; i++) {   // xGRB 555
		const uint16_t v = b.pal[i];
		s_snap->pens[i] = (uint32_t)(pal5((v >> 5) & 31) << 16 | pal5((v >> 10) & 31) << 8 | pal5(v & 31));
	}
	if (s_m == M_BERLWALL) {   // the background's 32768 colours with the brightness (MAME pen contrast)
		const float brt = (float)(b.bg15_bright / 255.0);
		static uint8_t last = 0xff;
		static int done;
		if (!done || last != b.bg15_bright) {
			for (int i = 0; i < 0x8000; i++) {   // GRB 555
				const int r = (int)((float)pal5((i >> 5) & 31) * brt), g = (int)((float)pal5((i >> 10) & 31) * brt),
				          bl = (int)((float)pal5(i & 31) * brt);
				s_snap->pens[2048 + i] = (uint32_t)(r << 16 | g << 8 | bl);
			}
			last = b.bg15_bright;
			done = 1;
		}
	}
	const uint16_t *sp = s_sprbmp[b.buffer];
	for (int y = mc->vy0; y <= mc->vy1; y++)
		memcpy(s_snap->spr + (y - mc->vy0) * 512, sp + y * SPR_W, sizeof(uint16_t) * 512);
	s_snap->bg15_scroll = b.bg15_scroll;
	s_snap->bg15_select = b.bg15_select;
	s_snap->bg15_bright = b.bg15_bright;
	s_snap->disp_enable = b.disp_enable != 0;
	rbx_video_frame_done();
}

// ---- frame ----
static void sound_line(void)
{
	int n = rbx_line_cycles(&b.out_frac, s_out_rate, s_mc->fps_mhz, s_mc->lines);
	if (n > s_mix_cap - s_mix_n)
		n = s_mix_cap - s_mix_n;
	if (n <= 0)
		return;
	int32_t *out = s_mix + s_mix_n;
	if (s_m == M_BLAZEON || s_m == M_WINGFORC) {
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&b.ym_frac, 4000000, s_mc->fps_mhz, s_mc->lines);
		n = rbx_ym_advance(0, clocks, out, s_mix_cap - s_mix_n);
	} else if (s_ay[0].clock) {   // the YM2149s at the output rate (their native rate / the divider)
		const uint32_t clocks = (uint32_t)n * s_ay[0].clk_per_sample * s_ay[0].divider;
		for (int c = 0; c < 2; c++)
			ay8910_render(&s_ay[c], clocks, out, n);
	}
	for (int i = 0; i < n; i++) {
		int32_t s = 0;
		for (int c = 0; c < s_noki; c++) {
			b.oki_acc[c] += s_oki[c].rate;
			while (b.oki_acc[c] >= s_out_rate) {
				b.oki_acc[c] -= s_out_rate;
				b.oki_hold[c] = oki_sample(&s_oki[c]);
			}
			s += b.oki_hold[c];
		}
		out[i] += s;
	}
	s_mix_n += n;
}

static void ka_frame(void)
{
	const machine_t *mc = s_mc;
	rbx_audio.frame_mhz = mc->fps_mhz;
	const int vbl = mc->vy1 + 1;
	for (int line = 0; line < mc->lines; line++) {
		if (line == vbl)
			vblank_capture();
		if (line == 0 && s_calc3 && b.c3_frames++)   // (its timer: a frame period from the reset)
			calc3_run();
		if (line == 224) {
			render_sprites();
			memcpy(b.spr_buf, b.spr, sizeof(b.spr));
			m68k_bus_hold_irq(5 - mc->irq_base);
		}
		if (line == 64)
			m68k_bus_hold_irq(4 - mc->irq_base);
		if (line == 144)
			m68k_bus_hold_irq(3 - mc->irq_base);
		s_cyc_base += m68k_execute(rbx_line_cycles(&b.cpu_frac, mc->cpu_clock, mc->fps_mhz, mc->lines));
		if (s_zrom) {
			b.z80_target += (uint64_t)rbx_line_cycles(&b.z80_frac, 4000000, mc->fps_mhz, mc->lines);
			int irq = 0;
			z80_run(&s_z80, (unsigned long)b.z80_target, &irq, 0);
		}
		sound_line();
	}
	if (vbl >= mc->lines)
		vblank_capture();
	if (s_m == M_BLAZEON || s_m == M_WINGFORC)
		rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static void ka_reset(void)
{
	rbx_video_wait_idle();
	memset(&b, 0, sizeof(b));
	b.disp_enable = 1;
	b.bg15_bright = 0xff;
	b.rng = 0x9d14abd7;
	for (int i = 0; i < 2; i++) {
		memset(s_sprbmp[i], 0, sizeof(uint16_t) * SPR_W * 512);
		memset(s_sprmask[i], 0, SPR_W * 512);
	}
	map_mem();
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	for (int c = 0; c < s_noki; c++)
		oki_reset(&s_oki[c]);
	if (s_ay[0].clock) {
		ay8910_reset(&s_ay[0]);
		ay8910_reset(&s_ay[1]);
	}
	if (s_zrom) {
		z80_init(&s_z80);
		s_z80.read_byte = z80_read;
		s_z80.write_byte = z80_write;
		s_z80.port_in = z80_in;
		s_z80.port_out = z80_out;
		rbx_ym_reset(0);
	}
	s_mix_n = 0;
}

static int ka_supported(const char *machine)
{
	for (int i = 0; i < M_COUNT; i++) {
		const size_t l = strlen(s_mnames[i]);
		if (!strncmp(machine, s_mnames[i], l) && (machine[l] == ':' || !machine[l]))
			return 1;
	}
	return 0;
}

static void ka_exit(void)
{
	rbx_video_wait_idle();
	if (s_has_eeprom)
		eepser_nv_save(&s_eeprom);
	if (s_zrom)
		rbx_ym_exit(0);
	free(s_snap);
	s_snap = NULL;
	free(s_mix);
	s_mix = NULL;
	free(s_bg15);
	s_bg15 = NULL;
	for (int i = 0; i < 2; i++) {
		free(s_sprbmp[i]);
		free(s_sprmask[i]);
		s_sprbmp[i] = NULL;
		s_sprmask[i] = NULL;
	}
	free(s_bp);   // (last: the render thread and the saves are done with it)
	s_bp = NULL;
	free(s_ts_hp);
	s_ts_hp = NULL;
}

static int ka_init(const rbx_game *g)
{
	if (!s_ts_hp && !(s_ts_hp = calloc(1, sizeof(*s_ts_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_bp && !(s_bp = (struct kaneko16_state *)calloc(1, sizeof(*s_bp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_m = -1;
	for (int i = 0; i < M_COUNT; i++) {
		const size_t l = strlen(s_mnames[i]);
		if (!strncmp(g->machine, s_mnames[i], l) && (g->machine[l] == ':' || !g->machine[l]))
			s_m = i;
	}
	if (s_m < 0) {
		rbx_fail("%s: machine %s not emulated", g->set, g->machine);
		return -1;
	}
	s_mc = &s_machines[s_m];
	s_toybox = s_m >= M_GTMR && s_m <= M_BONKADV;
	s_calc3 = s_m == M_SHOGWARR || s_m == M_BRAPBOYS;
	s_calc3_rom = rbx_region("calc3_rom", NULL);
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_spr_gfx = rbx_region("kan_spr", &s_spr_size);
	s_tiles[0] = rbx_region("view2_0", &s_tiles_size[0]);
	s_tiles[1] = rbx_region("view2_1", &s_tiles_size[1]);
	s_oki_rom[0] = rbx_region("oki1", &s_oki_size[0]);
	s_oki_rom[1] = rbx_region("oki2", &s_oki_size[1]);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint32_t msz = 0;
	s_mcudata = rbx_region("mcudata", &msz);
	uint32_t bsz = 0;
	s_bitmap = rbx_region("bitmap", &bsz);
	if (!s_rom || !s_spr_gfx || !s_tiles[0] || (s_toybox && !s_mcudata) || (s_calc3 && !s_calc3_rom)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	s_spr_count = s_spr_size / (s_mc->spr_type ? 256u : 128u);
	for (int c = 0; c < 2; c++)
		s_tile_count[c] = s_tiles[c] ? s_tiles_size[c] / 128 : 0;
	if (s_m == M_BERLWALL) {   // init_berlwall*: the protection's branch patched, as in MAME
		uint16_t *rom = (uint16_t *)s_rom;
		const char *init = strchr(g->machine, ':');
		if (init && !strcmp(init + 1, "berlwall")) { rom[0x1a3ea / 2] = 0x602c; rom[0x3fffe / 2] = 0xc40d; }
		else if (init && !strcmp(init + 1, "berlwallt")) { rom[0x1cf48 / 2] = 0x602c; rom[0x3fffe / 2] = 0xaed4; }
		else if (init && !strcmp(init + 1, "berlwallk")) { rom[0x1ceb0 / 2] = 0x602c; rom[0x3fffe / 2] = 0x8364; }
	}
	if (s_toybox)
		toybox_decrypt();
	if (s_calc3)
		calc3_init_crc();
	s_port_p1 = rbx_port_find(s_m == M_BLAZEON || s_m == M_WINGFORC ? "DSW2_P1" : s_m == M_MGCRYSTL ? "DSW_P1" : "P1");
	s_port_p2 = rbx_port_find(s_m == M_BLAZEON || s_m == M_WINGFORC ? "DSW1_P2" : "P2");
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_unk = rbx_port_find("UNK");
	s_port_dsw1 = rbx_port_find("DSW1");
	s_port_dsw2 = rbx_port_find("DSW2");
	s_port_extra = rbx_port_find("EXTRA");
	s_port_fake = rbx_port_find("FAKE");
	s_port_wheel[0] = rbx_port_find("WHEEL0");
	s_port_wheel[1] = rbx_port_find("WHEEL1");
	s_port_wheel[2] = rbx_port_find("WHEEL2");
	// sound
	memset(s_ay, 0, sizeof(s_ay));
	memset(s_oki, 0, sizeof(s_oki));
	s_noki = 0;
	switch (s_m) {
	case M_BERLWALL:
		oki_init(&s_oki[0], 2000000, 0, s_oki_rom[0], s_oki_size[0], 128);
		s_noki = 1;
		for (int c = 0; c < 2; c++)
			ay8910_init(&s_ay[c], AY8910_TYPE_YM2149, 1000000, AY8910_LEGACY_OUTPUT, 128, 4, 1);
		s_ay[0].port_a_read = ay0_porta;
		s_ay[0].port_b_read = ay0_portb;
		break;
	case M_BAKUBRKR:
	case M_MGCRYSTL:
		oki_init(&s_oki[0], 2000000, 1, s_oki_rom[0], s_oki_size[0], 128);
		if (s_m == M_BAKUBRKR) {
			s_oki[0].read = oki_read_bank;
			s_oki[0].user = (void *)(intptr_t)0;
		}
		s_noki = 1;
		for (int c = 0; c < 2; c++)
			ay8910_init(&s_ay[c], AY8910_TYPE_YM2149, 2000000, AY8910_LEGACY_OUTPUT, 128, 8, 1);
		if (s_m == M_BAKUBRKR)
			s_ay[0].port_b_write = ay0_portb_w;
		s_ay[1].port_a_read = ay1_porta;
		s_ay[1].port_b_write = ay1_portb_w;
		break;
	case M_WINGFORC:
		oki_init(&s_oki[0], 1000000, 1, s_oki_rom[0], s_oki_size[0], 128);
		s_oki[0].read = oki_read_bank;
		s_oki[0].user = (void *)(intptr_t)0;
		s_noki = 1;
		break;
	case M_BLAZEON:
		break;
	default:   // (TOYBOX, CALC3: two OKIs at 16 MHz / 8)
		for (int c = 0; c < 2; c++) {
			oki_init(&s_oki[c], 2000000, 0, s_oki_rom[c], s_oki_size[c], 128);
			s_oki[c].read = oki_read_bank;
			s_oki[c].user = (void *)(intptr_t)c;
		}
		s_noki = 2;
		break;
	}
	s_has_eeprom = s_m == M_BAKUBRKR || s_m == M_MGCRYSTL || s_toybox || s_calc3;
	eepser_init(&s_eeprom, EEPSER_93CXX, 64, 16, s_mc->cpu_clock);
	if (s_calc3) {   // (the board's default EEPROM data)
		static const uint16_t shogwarr_ee[64] = {
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x4B41, 0x4E45, 0x4B4F, 0x2F41, 0x544F, 0x5020, 0x3139, 0x3932,
			0x4655, 0x4A49, 0x5941, 0x4D41, 0x2042, 0x5553, 0x5445, 0x5220,
			0x2053, 0x484F, 0x4755, 0x4E20, 0x5741, 0x5252, 0x494F, 0x5253,
			0x636F, 0x7079, 0x7269, 0x6768, 0x7420, 0x4B41, 0x4E45, 0x4B4F,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFFFF,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0010, 0x0000, 0x0000, 0xFFFF };
		static const uint16_t brapboys_ee[64] = {
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0005, 0x0006, 0x2030, 0x0003, 0x6818, 0x0101, 0x0101,
			0x0101, 0x0001, 0x0004, 0x0008, 0x4B41, 0x4E45, 0x4B4F, 0x2020,
			0x4265, 0x2052, 0x6170, 0x2042, 0x6F79, 0x7300, 0x3030, 0x302E,
			0x3038, 0x10FF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
			0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
			0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
			0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x0035, 0xFFFF, 0xFFFF, 0xFFFF };
		memcpy(s_eeprom.data, s_m == M_BRAPBOYS ? brapboys_ee : shogwarr_ee, sizeof(shogwarr_ee));
	}
	if (s_has_eeprom)
		eepser_nv_load(&s_eeprom, g->set);
	if (s_m == M_BLAZEON || s_m == M_WINGFORC) {
		if (!s_zrom) {
			rbx_fail("%s: audiocpu missing", g->set);
			return -1;
		}
		rbx_ym_config yc;
		memset(&yc, 0, sizeof(yc));
		yc.type = RBX_YM2151;
		yc.clock = 4000000;
		yc.gain[0] = yc.gain[1] = s_m == M_WINGFORC ? 51 : 128;
		if (rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
		s_out_rate = rbx_ym_rate(0);
	} else if (s_ay[0].clock)
		s_out_rate = ay8910_rate(&s_ay[0]);
	else
		s_out_rate = s_oki[0].rate;
	s_mix_cap = (int)(s_out_rate / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_snap = calloc(1, sizeof(*s_snap));
	for (int i = 0; i < 2; i++) {
		s_sprbmp[i] = (uint16_t *)calloc(SPR_W * 512, sizeof(uint16_t));
		s_sprmask[i] = (uint8_t *)calloc(SPR_W * 512, 1);
	}
	if (s_m == M_BERLWALL && s_bitmap) {   // video_start: the 32 backgrounds decrypted
		s_bg15 = (uint16_t *)malloc(32 * 65536 * sizeof(uint16_t));
		if (s_bg15)
			for (int scr = 0; scr < 32; scr++)
				for (int x = 0; x < 256; x++)
					for (int y = 0; y < 256; y++) {
						const int addr = scr * 65536 + x + y * 256;
						const int data = s_bitmap[addr * 2] * 256 + s_bitmap[addr * 2 + 1];
						int r = (data & 0x07c0) >> 6, gg = (data & 0xf800) >> 11, bb = (data & 0x003e) >> 1;
						r ^= 0x09;
						if (~gg & 0x08) gg ^= 0x10;
						gg = (gg - 1) & 0x1f;
						bb ^= 0x03;
						if (~bb & 0x08) bb ^= 0x10;
						bb = (bb + 2) & 0x1f;
						if ((r & 0x10) && (bb & 0x10))
							gg = (gg - 1) & 0x1f;
						s_bg15[scr * 65536 + y * 256 + x] = (uint16_t)(((gg << 10) | (r << 5) | bb) & 0x7fff);
					}
	}
	const int pal_n = s_m == M_BERLWALL ? 2048 + 0x8000 : s_mc->pal_size;
	if (!s_mix || !s_snap || !s_sprbmp[0] || !s_sprbmp[1] || !s_sprmask[0] || !s_sprmask[1] ||
	    (s_m == M_BERLWALL && !s_bg15) || rbx_audio_setup(s_out_rate) ||
	    rbx_video_setup(s_mc->vx1 - s_mc->vx0 + 1, s_mc->vy1 - s_mc->vy0 + 1, pal_n > 0x8000 ? 0x8000 + 2048 : pal_n)) {
		rbx_fail("%s: out of memory", g->set);
		ka_exit();
		return -1;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	ka_reset();
	return 0;
}

static int ka_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (load)
		rbx_video_wait_idle();
	else
		m68k_get_context(cpu);
	uint32_t irq = m68k_bus_irq_pending();
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", &irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "kaneko", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	for (int c = 0; c < s_noki; c++)
		rc |= rbx_state_io(ctx, c ? "oki1" : "oki0", &s_oki[c], (uint32_t)offsetof(okim6295_t, rom));
	if (s_ay[0].clock)
		for (int c = 0; c < 2; c++)
			rc |= rbx_state_io(ctx, c ? "ay1" : "ay0", &s_ay[c], (uint32_t)AY8910_STATE_END);
	rc |= eepser_state(&s_eeprom, ctx, load, "eeprom");
	for (int i = 0; i < 2; i++) {
		rc |= rbx_state_io(ctx, i ? "sprbmp1" : "sprbmp0", s_sprbmp[i], sizeof(uint16_t) * SPR_W * 512);
		rc |= rbx_state_io(ctx, i ? "sprmsk1" : "sprmsk0", s_sprmask[i], SPR_W * 512);
	}
	if (s_zrom) {
		z80 z = s_z80;
		rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
		rc |= rbx_ym_state(0, ctx, load);
		if (!rc && load) {
			z.read_byte = z80_read;
			z.write_byte = z80_write;
			z.port_in = z80_in;
			z.port_out = z80_out;
			s_z80 = z;
		}
	}
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
		m68k_bus_update_irq();
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int ka_save(void *ctx) { return ka_state(ctx, 0); }
static int ka_load(void *ctx) { return ka_state(ctx, 1); }

const rbx_driver rbx_drv_kaneko16 = { "Kaneko 16", ka_init, ka_exit, ka_reset, ka_frame, ka_save, ka_load,
                                      ka_supported, ka_render, ka_help };
