/*  Psikyo 68EC020 boards (SH201B, KA302C, SH403 / SH404). Ported from MAME src/mame/psikyo/psikyo.cpp, psikyo_v.cpp
    (BSD-3-Clause, copyright-holders: Luca Elia, Olivier Galibert, Paul Priest).
    A 68EC020 at 16 MHz, two 16x16 tile layers of four shapes (64x64 .. 256x16 tiles in a 4096x2048 map) with line
    scroll, up to 0x300 sprites of 16x16 tiles through a code look-up ROM, zoomed, buffered two frames; a palette of
    4096 xRGB 555. Sound: a Z80 (or LZ8420M) with a YM2610 (Sengoku Ace, Gunbird, Battle K-Road, Strikers 1945
    unprotected) or a YMF278B (Strikers 1945, Tengai; their PIC simulated as in MAME). 342x262 (456 / 2 dots),
    59.92 Hz, 320x224 shown.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 16000000
#define FRAME_MHZ 59923          // 14.318181 MHz / 2 / (456 x 262)
#define LINES 262
#define VIS_W 320
#define VIS_H 224
#define PAL_BLACK 0x1000

enum { B_SNGKACE, B_GUNBIRD, B_S1945N, B_S1945 };

// ---- ROMs ----
static uint8_t *s_rom, *s_zrom, *s_spr, *s_tiles, *s_adpcma, *s_adpcmb, *s_ymf;
static uint32_t s_rom_size, s_zrom_size, s_spr_size, s_tiles_size, s_adpcma_size, s_adpcmb_size, s_ymf_size;
static const uint16_t *s_lut;
static uint32_t s_lut_n;
static int s_board, s_ka302c;
static const uint8_t *s_mcu_table;
static int s_port_p1p2, s_port_dsw, s_port_coin;

static const uint8_t s1945_table[256] = {
	0x00, 0x00, 0x64, 0xae, 0x00, 0x00, 0x26, 0x2c, 0x00, 0x00, 0x2c, 0xda, 0x00, 0x00, 0x2c, 0xbc,
	0x00, 0x00, 0x2c, 0x9e, 0x00, 0x00, 0x2f, 0x0e, 0x00, 0x00, 0x31, 0x10, 0x00, 0x00, 0xc5, 0x1e,
	0x00, 0x00, 0x32, 0x90, 0x00, 0x00, 0xac, 0x5c, 0x00, 0x00, 0x2b, 0xc0
};
static const uint8_t s1945a_table[256] = {
	0x00, 0x00, 0x64, 0xbe, 0x00, 0x00, 0x26, 0x2c, 0x00, 0x00, 0x2c, 0xda, 0x00, 0x00, 0x2c, 0xbc,
	0x00, 0x00, 0x2c, 0x9e, 0x00, 0x00, 0x2f, 0x0e, 0x00, 0x00, 0x31, 0x10, 0x00, 0x00, 0xc7, 0x2a,
	0x00, 0x00, 0x32, 0x90, 0x00, 0x00, 0xad, 0x4c, 0x00, 0x00, 0x2b, 0xc0
};
static const uint8_t s1945j_table[256] = {
	0x00, 0x00, 0x64, 0xb6, 0x00, 0x00, 0x26, 0x2c, 0x00, 0x00, 0x2c, 0xda, 0x00, 0x00, 0x2c, 0xbc,
	0x00, 0x00, 0x2c, 0x9e, 0x00, 0x00, 0x2f, 0x0e, 0x00, 0x00, 0x31, 0x10, 0x00, 0x00, 0xc5, 0x92,
	0x00, 0x00, 0x32, 0x90, 0x00, 0x00, 0xac, 0x64, 0x00, 0x00, 0x2b, 0xc0
};

// ---- the state (saved) ----
static struct psikyo_state {   // (on the heap while the game runs, ADR-040)
	uint16_t spriteram[0x1000];   // 8 KB (host-order words)
	uint16_t paletteram[0x1000];
	uint16_t vram[2][0x1000];
	uint16_t vregs[0x2000];       // 0x804000-0x807fff
	uint16_t ram[0x10000];        // 0xfe0000-0xffffff
	uint16_t sprbuf[0x1000];      // BUFFERED_SPRITERAM32: the copy made at the last vblank
	uint16_t sprlist_src[0x1000]; // the copy the drawn list was built from (the vblank before)
	uint8_t tilemap_bank[2];
	uint8_t soundlatch, latch_pending, zbank, mcu_status;
	uint8_t mcu_direction, mcu_inlatch, mcu_latch1, mcu_latch2, mcu_latching, mcu_control, mcu_index, mcu_mode;
	uint8_t mcu_bctrl, pad[3];
	uint64_t cpu_frac, z80_frac, ym_frac;
	uint32_t frame;
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)

static z80 s_z80;
static int s_zirq;
static uint8_t s_zram[0x800];
static const uint8_t *s_zmap[256];
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static int s_vblank;

// ---- the s1945 / tengai PIC (MAME's simulation) ----
static void mcu_command_w(uint8_t data)
{
	switch (data | (b.mcu_direction ? 0x100 : 0)) {
	case 0x11c:
		b.mcu_latching = 5;
		b.mcu_index = b.mcu_inlatch;
		break;
	case 0x013:
		b.mcu_latching = 1;
		if (s_mcu_table)
			b.mcu_latch1 = s_mcu_table[b.mcu_index];
		break;
	case 0x113:
		b.mcu_mode = b.mcu_inlatch;
		if (b.mcu_mode == 1) {
			b.mcu_latching &= ~1;
			b.mcu_latch2 = 0x55;
		} else {
			b.mcu_latching &= ~1;
			b.mcu_latching |= 2;
		}
		b.mcu_latching &= ~4;
		b.mcu_latch1 = b.mcu_inlatch;
		break;
	case 0x010: case 0x110:
		b.mcu_latching |= 4;
		break;
	}
}
static uint32_t mcu_data_r(void)
{
	uint32_t res;
	if (b.mcu_control & 16) {
		res = (b.mcu_latching & 4) ? 0x0000ff00 : (uint32_t)b.mcu_latch1 << 8;
		b.mcu_latching |= 4;
	} else {
		res = (b.mcu_latching & 1) ? 0x0000ff00 : (uint32_t)b.mcu_latch2 << 8;
		b.mcu_latching |= 1;
	}
	return res | (b.mcu_bctrl & 0xf0);
}

// ---- the 68EC020 (psikyo_map + the boards' I/O) ----
static void set_customs(void)
{
	if (s_board == B_SNGKACE) {
		rbx_port_custom(s_port_coin, 0x00000001, s_vblank ? 0x00000001 : 0);
		rbx_port_custom(s_port_coin, 0x00800000, b.latch_pending ? 0x00800000 : 0);
	} else {
		rbx_port_custom(s_port_p1p2, 0x80, b.latch_pending ? 0x80 : 0);
		rbx_port_custom(s_port_dsw, 0x80, s_vblank ? 0x80 : 0);
	}
}
static uint32_t input_r(int offset, int side)   // sngkace_input_r, gunbird_input_r, s1945_input_r (the longs)
{
	set_customs();
	switch (offset) {
	case 0: {
		uint32_t v = s_port_p1p2 >= 0 ? rbx_port_read(s_port_p1p2) : 0xffffffff;
		if (s_board == B_S1945) {   // mcu_status_r (the hack toggling a bit on each read)
			v = (v & ~4u) | (b.mcu_status ? 4u : 0u);
			if (side)
				b.mcu_status = !b.mcu_status;
		}
		return v;
	}
	case 1: {
		const uint32_t v = s_port_dsw >= 0 ? rbx_port_read(s_port_dsw) : 0xffffffff;
		if (s_board != B_S1945)
			return v;
		if (!side) {   // (the high word: no side effects)
			const uint8_t l = b.mcu_latching;
			const uint32_t r = (v & 0xffff000f) | mcu_data_r();
			b.mcu_latching = l;
			return r;
		}
		return (v & 0xffff000f) | mcu_data_r();
	}
	case 2:
		if (s_board == B_SNGKACE)
			return s_port_coin >= 0 ? rbx_port_read(s_port_coin) : 0xffffffff;
		return 0;
	}
	return 0;
}
static uint16_t main_r16(uint32_t a)
{
	a &= 0xfffffe;
	if (a >= 0x800000 && a < 0x802000) return b.vram[0][(a & 0x1fff) >> 1];
	if (a >= 0x802000 && a < 0x804000) return b.vram[1][(a & 0x1fff) >> 1];
	if (a >= 0xc00000 && a < (s_board == B_S1945 ? 0xc00008u : 0xc0000cu)) {
		// (a long read is two word reads: the side effects (the PIC's latches, the status toggle) with the low word,
		// whose bits they are)
		const int off = (int)((a - 0xc00000) >> 2);
		return (a & 2) ? (uint16_t)input_r(off, 1) : (uint16_t)(input_r(off, 0) >> 16);
	}
	if (s_board == B_S1945 && a == 0xc00008)   // s1945_mcu_control_r (byte c00008)
		return (uint16_t)((b.mcu_latching | 0x08) << 8 | 0xff);
	return 0xffff;
}
static uint8_t main_r8(uint32_t a)
{
	const uint16_t w = main_r16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}
static void soundlatch_w(uint8_t v)
{
	b.soundlatch = v;
	if (!b.latch_pending)
		z80_gen_nmi(&s_z80);
	b.latch_pending = 1;
}
static void main_w8(uint32_t a, uint8_t v)
{
	a &= 0xffffff;
	if (a >= 0x800000 && a < 0x804000) {
		uint8_t *p = (uint8_t *)b.vram[(a >> 13) & 1];
		p[(a & 0x1fff) ^ M68K_BYTE_XOR] = v;
		return;
	}
	if (s_board == B_S1945) {
		switch (a) {
		case 0xc00006: b.mcu_inlatch = v; return;
		case 0xc00007:   // s1945_mcu_bctrl_w: the tile banks
			b.tilemap_bank[1] = (v >> 6) & 3;
			b.tilemap_bank[0] = (v >> 4) & 3;
			b.mcu_bctrl = v;
			return;
		case 0xc00008: b.mcu_control = v; return;
		case 0xc00009: b.mcu_direction = v; return;
		case 0xc0000b: mcu_command_w(v); return;
		case 0xc00011: soundlatch_w(v); return;
		}
		return;
	}
	if (a == (s_board == B_S1945N ? 0xc00011u : 0xc00013u))
		soundlatch_w(v);
}
static void main_w16(uint32_t a, uint16_t v)
{
	a &= 0xfffffe;
	if (a >= 0x800000 && a < 0x804000) {
		b.vram[(a >> 13) & 1][(a & 0x1fff) >> 1] = v;
		return;
	}
	main_w8(a, (uint8_t)(v >> 8));
	main_w8(a + 1, (uint8_t)v);
}

// ---- the Z80 ----
static void z_bank(void)
{
	// sngkace: 8000-ffff the bank (4 of 0x8000); the others: 8200-ffff from the ROM + 0x200 (the same offsets)
	for (int p = 0x80; p < 0x100; p++) {
		if (s_board != B_SNGKACE && p < 0x82) {
			s_zmap[p] = NULL;
			continue;
		}
		const uint32_t o = (uint32_t)b.zbank * 0x8000 + (uint32_t)(p - 0x80) * 256;
		s_zmap[p] = o + 256 <= s_zrom_size ? s_zrom + o : NULL;
	}
}
static uint8_t z_read(void *u, uint16_t a)
{
	(void)u;
	const uint8_t *p = s_zmap[a >> 8];
	if (p)
		return p[a & 0xff];
	if (s_board == B_SNGKACE && a >= 0x7800 && a < 0x8000)
		return s_zram[a - 0x7800];
	if (s_board != B_SNGKACE && a >= 0x8000 && a < 0x8200)
		return s_zram[a - 0x8000];
	return 0xff;
}
static void z_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_board == B_SNGKACE && a >= 0x7800 && a < 0x8000)
		s_zram[a - 0x7800] = v;
	else if (s_board != B_SNGKACE && a >= 0x8000 && a < 0x8200)
		s_zram[a - 0x8000] = v;
}
static uint8_t z_in(z80 *z, uint8_t p)
{
	(void)z;
	switch (s_board) {
	case B_SNGKACE:
		if (p <= 0x03) return rbx_ym_read(0, p & 3);
		if (p == 0x08) return b.soundlatch;
		break;
	case B_GUNBIRD: case B_S1945N:
		if (p >= 0x04 && p <= 0x07) return rbx_ym_read(0, p & 3);
		if (p == 0x08) return b.soundlatch;
		break;
	case B_S1945:
		if (p >= 0x08 && p <= 0x0d) return rbx_ym_read(0, (uint32_t)(p - 0x08));
		if (p == 0x10) return b.soundlatch;
		break;
	}
	return 0xff;
}
static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	switch (s_board) {
	case B_SNGKACE:
		if (p <= 0x03) rbx_ym_write(0, p & 3, v);
		else if (p == 0x04) { b.zbank = v & 3; z_bank(); }
		else if (p == 0x0c) b.latch_pending = 0;
		break;
	case B_GUNBIRD: case B_S1945N:
		if (p == 0x00) { b.zbank = (v >> 4) & 3; z_bank(); }
		else if (p >= 0x04 && p <= 0x07) rbx_ym_write(0, p & 3, v);
		else if (p == 0x0c) b.latch_pending = 0;
		break;
	case B_S1945:
		if (p == 0x00) { b.zbank = (v >> 4) & 3; z_bank(); }
		else if (p >= 0x08 && p <= 0x0d) rbx_ym_write(0, (uint32_t)(p - 0x08), v);
		else if (p == 0x18) b.latch_pending = 0;
		break;
	}
}
static void ym_irq(int state) { s_zirq = state; }
static void z_start(void)
{
	z80_init(&s_z80);
	s_z80.read_byte = z_read;
	s_z80.write_byte = z_write;
	s_z80.port_in = z_in;
	s_z80.port_out = z_out;
	s_z80.read_map = s_zmap;
}

// ---- drawing ----
typedef struct { uint32_t code; uint16_t color; uint8_t flipx, flipy; int32_t x, y; uint32_t zoomx, zoomy; uint32_t primask; } spr_t;
static struct {
	uint16_t paletteram[0x1000];
	uint16_t vram[2][0x1000];
	uint16_t vregs[0x2000];
	uint8_t tilemap_bank[2];
	int pal_dirty;
	int nspr;
	uint16_t sprite_ctrl;
	spr_t spr[0x800 / 2 * 8 * 8];
} *s_snap;
static uint16_t s_pal_conv[0x1000];
static int s_pal_valid;

static inline uint16_t vreg(const uint16_t *v, int byteoff) { return v[byteoff >> 1]; }

// get_sprites: the list from the buffered sprite RAM (the order kept: drawn from its end)
static void get_sprites(const uint16_t *buf)
{
	static const uint32_t pri[] = { 0, 0xfc, 0xff, 0xff };
	s_snap->sprite_ctrl = buf[0x1ffe / 2];
	s_snap->nspr = 0;
	if (s_snap->sprite_ctrl & 1)
		return;
	const uint16_t *spritelist = buf + 0x1800 / 2;
	for (int offs = 0; offs < (0x800 - 2) / 2; offs++) {
		uint16_t sprite = spritelist[offs];
		if (sprite == 0xffff)
			break;
		sprite %= 0x300;
		const uint16_t *src = buf + sprite * 8 / 2;
		int32_t y = src[0], x = src[1];
		const uint16_t attr = src[2];
		uint32_t code = ((uint32_t)src[2] << 16 | src[3]) & 0x1ffff;
		int flipx = (attr & 0x4000) != 0, flipy = (attr & 0x8000) != 0;
		uint32_t zoomx = (uint32_t)(x & 0xf000) >> 12, zoomy = (uint32_t)(y & 0xf000) >> 12;
		const int nx = ((x & 0x0e00) >> 9) + 1, ny = ((y & 0x0e00) >> 9) + 1;
		x = x & 0x01ff;
		y = (y & 0x00ff) - (y & 0x100);
		if (x >= 0x180)
			x -= 0x200;
		x += (int32_t)((uint32_t)nx * zoomx + 2) / 4;
		y += (int32_t)((uint32_t)ny * zoomy + 2) / 4;
		zoomx = 32 - zoomx;
		zoomy = 32 - zoomy;
		const int xstart = flipx ? nx - 1 : 0, xend = flipx ? -1 : nx, xinc = flipx ? -1 : 1;
		const int ystart = flipy ? ny - 1 : 0, yend = flipy ? -1 : ny, yinc = flipy ? -1 : 1;
		for (int dy = ystart; dy != yend; dy += yinc)
			for (int dx = xstart; dx != xend; dx += xinc) {
				spr_t *s = &s_snap->spr[s_snap->nspr++];
				s->code = s_lut_n ? s_lut[code & (s_lut_n - 1)] : 0;
				s->color = (uint16_t)(attr >> 8);
				s->flipx = (uint8_t)flipx;
				s->flipy = (uint8_t)flipy;
				if (zoomx == 32 && zoomy == 32) {
					s->x = x + dx * 16;
					s->y = y + dy * 16;
				} else {
					s->x = x + (dx * (int)zoomx) / 2;
					s->y = y + (dy * (int)zoomy) / 2;
				}
				s->zoomx = zoomx << 11;
				s->zoomy = zoomy << 11;
				s->primask = pri[(attr & 0xc0) >> 6];
				code++;
			}
	}
}

static inline uint8_t gfx_pix(const uint8_t *base, uint32_t count, uint32_t code, int x, int y)   // 16x16x4 packed msb
{
	const uint8_t byte = base[(rbx_wrap(code, count)) * 128 + (uint32_t)y * 8 + (uint32_t)(x >> 1)];
	return (x & 1) ? (byte & 0x0f) : (byte >> 4);
}

// a tilemap's index (tile_scan) of a column and row of the 256 x 128 tile map
static inline int tile_index(int size, int col, int row)
{
	switch (size) {
	case 0: return (col & 0x3f) | ((row & 0x3f) << 6) | ((col & 0xc0) << 6) | ((row & 0x40) << 8);
	case 1: return (col & 0x7f) | ((row & 0x1f) << 7) | ((col & 0x80) << 5) | ((row & 0x60) << 8);
	case 2: return (col & 0xff) | ((row & 0x0f) << 8) | ((row & 0x70) << 8);
	default: return (col & 0x1f) | ((row & 0x7f) << 5) | ((col & 0xe0) << 7);
	}
}

#define BANDS 4
static uint8_t (*s_pri_hp)[VIS_H][VIS_W];
#define s_pri (*s_pri_hp)
static int s_band_next, s_band_left, s_band_active, s_band_helpers;

static void draw_layer_line(int layer, int y, uint16_t *d, uint8_t *pri)
{
	const uint16_t *v = s_snap->vregs;
	const uint16_t ctrl = vreg(v, layer ? 0x416 : 0x412);
	if (ctrl & 1)
		return;   // (disabled)
	const uint16_t scrolly = vreg(v, layer ? 0x40a : 0x402), scrollx = vreg(v, layer ? 0x40e : 0x406);
	const int size = (ctrl & 0xc0) >> 6, opaque = (ctrl & 2) != 0;
	const int transpen = (ctrl & 8) ? 0 : 15;
	int effx = scrollx;
	if (ctrl & 0x0300) {
		const int trs = (ctrl & 0x0200) >> 7;
		effx = scrollx + v[layer * 0x200 / 2 + (y >> trs)];
	}
	const int vy = (y + scrolly) & 0x7ff, row = vy >> 4, py = vy & 15;
	const uint32_t count = s_tiles_size / 128;
	const uint16_t *vram = s_snap->vram[layer];
	const int bank = s_snap->tilemap_bank[layer];
	for (int x = 0; x < VIS_W;) {
		const int vx = (x + effx) & 0xfff, col = vx >> 4, px = vx & 15;
		const uint16_t code = vram[tile_index(size, col, row) & 0xfff];
		const uint32_t tile = (uint32_t)(code & 0x1fff) + 0x2000u * (uint32_t)bank;
		const uint16_t color = (uint16_t)(0x800 + (((code >> 13) & 7) + layer * 0x40) * 16);
		int k = 16 - px;
		if (k > VIS_W - x)
			k = VIS_W - x;
		for (int i = 0; i < k; i++) {
			const uint8_t pen = gfx_pix(s_tiles, count, tile, px + i, py);
			if (opaque || pen != transpen) {
				d[x + i] = (uint16_t)(color + pen);
				pri[x + i] |= (uint8_t)(layer + 1);
			}
		}
		x += k;
	}
}

// prio_zoom_transmask / prio_transmask (MAME drawgfxzoom_core): the first sprite drawn on a pixel decides
static void draw_sprite(const spr_t *s, int y0, int y1, uint32_t transmask)
{
	const uint32_t count = s_spr_size / 128;
	if (!count)
		return;
	const uint32_t scalex = s->zoomx, scaley = s->zoomy;
	const uint32_t dstw = (scalex * 16 + 0x8000) >> 16, dsth = (scaley * 16 + 0x8000) >> 16;
	if (dstw < 1 || dsth < 1)
		return;
	int32_t dx = (16 << 16) / (int32_t)dstw, dy = (16 << 16) / (int32_t)dsth;
	if (scalex == 0x10000 && scaley == 0x10000)
		dx = dy = 0x10000;
	int32_t destx = s->x, desty = s->y;
	int32_t endx = destx + (int32_t)dstw - 1, endy = desty + (int32_t)dsth - 1;
	if (destx > VIS_W - 1 || endx < 0 || desty > y1 - 1 || endy < y0)
		return;
	int32_t srcx = 0, srcy = 0;
	if (destx < 0) { srcx = -destx * dx; destx = 0; }
	if (endx > VIS_W - 1) endx = VIS_W - 1;
	if (desty < y0) { srcy = (y0 - desty) * dy; desty = y0; }
	if (endy > y1 - 1) endy = y1 - 1;
	if (s->flipx) { srcx = ((int32_t)dstw - 1) * dx - srcx; dx = -dx; }
	if (s->flipy) { srcy = ((int32_t)dsth - 1) * dy - srcy; dy = -dy; }
	const uint16_t color = (uint16_t)((s->color & 0x1f) * 16);   // (0x20 colours of 16)
	const uint32_t pmask = s->primask | 0x80000000u;
	for (int y = desty; y <= endy; y++, srcy += dy) {
		uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
		uint8_t *p = s_pri[y];
		int32_t sx = srcx;
		for (int x = destx; x <= endx; x++, sx += dx) {
			const uint8_t pen = gfx_pix(s_spr, count, s->code, sx >> 16, srcy >> 16);
			if (!((transmask >> pen) & 1)) {
				if (!((1u << (p[x] & 0x1f)) & pmask))
					d[x] = (uint16_t)(color + pen);
				p[x] = 31;
			}
		}
	}
}

static void run_band(int band)
{
	const int y0 = VIS_H * band / BANDS, y1 = VIS_H * (band + 1) / BANDS;
	const uint16_t ctrl0 = vreg(s_snap->vregs, 0x412);
	const uint16_t bgpen = (ctrl0 & 8) ? 0x800 : 0x80f;
	for (int y = y0; y < y1; y++) {
		uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
		uint8_t *pri = s_pri[y];
		for (int x = 0; x < VIS_W; x++)
			d[x] = bgpen;
		memset(pri, 0, VIS_W);
		draw_layer_line(0, y, d, pri);
		draw_layer_line(1, y, d, pri);
	}
	uint32_t transmask = 0;
	if (s_snap->sprite_ctrl & 4) transmask |= 1u << 0;
	if (s_snap->sprite_ctrl & 8) transmask |= 1u << 15;
	for (int i = s_snap->nspr - 1; i >= 0; i--)
		draw_sprite(&s_snap->spr[i], y0, y1, transmask);
}

static void psikyo_render(void)
{
	if (s_snap->pal_dirty || !s_pal_valid) {
		for (int i = 0; i < 0x1000; i++) {
			const uint16_t v = s_snap->paletteram[i];
			if (s_pal_valid && s_pal_conv[i] == v)
				continue;
			s_pal_conv[i] = v;
			const int r = (v >> 10) & 31, g = (v >> 5) & 31, bb = v & 31;
			rbx_video.palette[i] = (uint32_t)(((r << 3) | (r >> 2)) << 16 | ((g << 3) | (g >> 2)) << 8 | ((bb << 3) | (bb >> 2)));
		}
		rbx_video.palette[PAL_BLACK] = 0;
		s_pal_valid = 1;
		s_snap->pal_dirty = 0;
	}
	__atomic_store_n(&s_band_next, 0, __ATOMIC_SEQ_CST);
	__atomic_store_n(&s_band_left, BANDS, __ATOMIC_SEQ_CST);
	__atomic_store_n(&s_band_active, 1, __ATOMIC_SEQ_CST);
	for (;;) {
		const int bnd = __atomic_fetch_add(&s_band_next, 1, __ATOMIC_ACQ_REL);
		if (bnd >= BANDS)
			break;
		run_band(bnd);
		__atomic_sub_fetch(&s_band_left, 1, __ATOMIC_ACQ_REL);
	}
	while (__atomic_load_n(&s_band_left, __ATOMIC_ACQUIRE) > 0)
		;
	__atomic_store_n(&s_band_active, 0, __ATOMIC_SEQ_CST);
	while (__atomic_load_n(&s_band_helpers, __ATOMIC_ACQUIRE) > 0)
		;
}

static int psikyo_render_help(int helper)
{
	const int one = helper & 0x100;
	helper &= 0xff;
	if (helper < 1 || helper > 2 || !__atomic_load_n(&s_band_active, __ATOMIC_SEQ_CST))
		return 0;
	__atomic_add_fetch(&s_band_helpers, 1, __ATOMIC_SEQ_CST);
	int did = 0;
	if (__atomic_load_n(&s_band_active, __ATOMIC_SEQ_CST))
		for (;;) {
			const int bnd = __atomic_fetch_add(&s_band_next, 1, __ATOMIC_ACQ_REL);
			if (bnd >= BANDS)
				break;
			run_band(bnd);
			__atomic_sub_fetch(&s_band_left, 1, __ATOMIC_ACQ_REL);
			did = 1;
			if (one)
				break;
		}
	__atomic_sub_fetch(&s_band_helpers, 1, __ATOMIC_SEQ_CST);
	return did;
}

// the vblank: the frame drawn (MAME's screen update before it: the list built at the last vblank, from the RAM of
// the one before), then get_sprites and the copy
static void vblank(void)
{
	rbx_video_wait_idle();
	if (memcmp(s_snap->paletteram, b.paletteram, sizeof(b.paletteram))) {
		memcpy(s_snap->paletteram, b.paletteram, sizeof(b.paletteram));
		s_snap->pal_dirty = 1;
	}
	memcpy(s_snap->vram, b.vram, sizeof(b.vram));
	memcpy(s_snap->vregs, b.vregs, sizeof(b.vregs));
	if (s_ka302c) {   // (the KA302C's tile banks from the layer controls)
		b.tilemap_bank[0] = (uint8_t)((b.vregs[0x412 / 2] & 0x400) >> 10);
		b.tilemap_bank[1] = (uint8_t)((b.vregs[0x416 / 2] & 0x400) >> 10);
	}
	memcpy(s_snap->tilemap_bank, b.tilemap_bank, sizeof(b.tilemap_bank));
	get_sprites(b.sprlist_src);
	rbx_video_frame_done();
	// (the next frame's list: get_sprites at this vblank reads the buffer, then the RAM is copied)
	memcpy(b.sprlist_src, b.sprbuf, sizeof(b.sprbuf));
	memcpy(b.sprbuf, b.spriteram, sizeof(b.spriteram));
}

// ---- the main loop's wait (an exact skip) ----
// The games wait for the vblank interrupt in "loop: addq.l #1,(counter).l / tst.w (flag).l / beq loop", the flag set by
// the interrupt handler. No interrupt comes inside a line's slice, so while the flag is 0 whole turns of the loop are
// done at once: the counter raised by the turns, their cycles taken off the slice (the same state as running them).
#define IDLE_MAX 8
static uint32_t s_idle_pcs[IDLE_MAX], s_idle_counters[IDLE_MAX], s_idle_flags[IDLE_MAX];
static int s_idle_n;
static uint32_t s_idle_pc, s_idle_counter, s_idle_flag;
static int s_idle_cycles;   // (one turn's cycles, measured on the first one: the same code everywhere)
static void find_idle_loop(void)
{
	s_idle_n = 0;
	s_idle_cycles = 0;
	const uint16_t *w = (const uint16_t *)s_rom;   // (host-order words)
	for (uint32_t i = 0; i + 8 <= s_rom_size / 2 && s_idle_n < IDLE_MAX; i++)
		if (w[i] == 0x52b9 && w[i + 1] == 0xfffe && w[i + 3] == 0x4a79 && w[i + 4] == 0xfffe && w[i + 6] == 0x67f2) {
			s_idle_pcs[s_idle_n] = i * 2;
			s_idle_counters[s_idle_n] = 0xfe0000 | w[i + 2];
			s_idle_flags[s_idle_n] = 0xfe0000 | w[i + 5];
			s_idle_n++;
		}
}
static inline uint16_t ram_w16(uint32_t a) { return b.ram[(a & 0x1ffff) >> 1]; }
static int idle_skip(int mc)
{
	if (!s_idle_n || mc <= 0)
		return mc;
	const uint32_t pc = (uint32_t)m68k_get_reg(NULL, M68K_REG_PC);
	int j = 0;   // (the slice may have ended on any of the loop's three instructions)
	while (j < s_idle_n && pc != s_idle_pcs[j] && pc != s_idle_pcs[j] + 6 && pc != s_idle_pcs[j] + 12)
		j++;
	if (j == s_idle_n)
		return mc;
	if (ram_w16(s_idle_flags[j]))
		return mc;
	while ((uint32_t)m68k_get_reg(NULL, M68K_REG_PC) != s_idle_pcs[j] && mc > 0)   // (on to its start)
		mc -= m68k_execute(1);
	if ((uint32_t)m68k_get_reg(NULL, M68K_REG_PC) != s_idle_pcs[j] || mc <= 0)
		return mc;
	s_idle_pc = s_idle_pcs[j];
	s_idle_counter = s_idle_counters[j];
	s_idle_flag = s_idle_flags[j];
	if (ram_w16(s_idle_flag))
		return mc;
	if (!s_idle_cycles) {   // (the first turn run, its cycles counted)
		int c = 0;
		do
			c += m68k_execute(1);
		while ((uint32_t)m68k_get_reg(NULL, M68K_REG_PC) != s_idle_pc && c < 200);
		if ((uint32_t)m68k_get_reg(NULL, M68K_REG_PC) != s_idle_pc || c <= 0) {
			s_idle_n = 0;   // (not the loop it looked like: no skipping)
			return mc - c;
		}
		s_idle_cycles = c;
		return mc - c;
	}
	const int k = mc / s_idle_cycles - 2;   // (whole turns, leaving more than one to run)
	if (k <= 0)
		return mc;
	uint16_t *p = &b.ram[(s_idle_counter & 0x1ffff) >> 1];
	const uint32_t v = ((uint32_t)p[0] << 16 | p[1]) + (uint32_t)k;
	p[0] = (uint16_t)(v >> 16);
	p[1] = (uint16_t)v;
	return mc - k * s_idle_cycles;
}

// ---- the frame ----
static void psikyo_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_H) {
			s_vblank = 1;
			vblank();
			m68k_bus_hold_irq(4);
		} else if (line == 0)
			s_vblank = 0;
		const int mc = idle_skip(rbx_line_cycles(&b.cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		if (mc > 0)
			m68k_execute(mc);
		const int zc = rbx_line_cycles(&b.z80_frac, s_board == B_SNGKACE ? 4000000 : 8000000, FRAME_MHZ, LINES);
		z80_run(&s_z80, s_z80.cyc + (unsigned long)zc, &s_zirq, 0);
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&b.ym_frac, s_board == B_S1945 ? 33868800 : 8000000, FRAME_MHZ, LINES);
		s_mix_n += rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	b.frame++;
}

static void map_mem(void)
{
	m68k_bus_reset_map();
	m68k_bus_map(0x000000, 0x0fffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x400000, 0x401fff, b.spriteram, sizeof(b.spriteram), 1, 1);
	m68k_bus_map(0x600000, 0x601fff, b.paletteram, sizeof(b.paletteram), 1, 1);
	m68k_bus_map(0x804000, 0x807fff, b.vregs, sizeof(b.vregs), 1, 1);
	m68k_bus_map(0xfe0000, 0xffffff, b.ram, sizeof(b.ram), 1, 1);
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
}

static void psikyo_reset(void)
{
	memset(&b, 0, offsetof(__typeof__(b), state_end));
	if (s_board == B_SNGKACE) {   // VIDEO_START sngkace: the layers' fixed banks
		b.tilemap_bank[0] = 0;
		b.tilemap_bank[1] = 1;
	}
	b.mcu_inlatch = 0xff;   // (s1945_state::machine_start)
	b.mcu_latch1 = 0xff;
	b.mcu_latch2 = 0xff;
	b.mcu_latching = 0x5;
	b.mcu_control = 0xff;
	memset(s_zram, 0, sizeof(s_zram));
	memset(s_zmap, 0, sizeof(s_zmap));
	const int fixed = s_board == B_SNGKACE ? 0x78 : 0x80;
	for (int p = 0; p < fixed && (uint32_t)p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	z_bank();
	z_start();
	s_zirq = 0;
	s_vblank = 0;
	rbx_ym_reset(0);
	s_mix_n = 0;
	s_pal_valid = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
}

static void psikyo_exit(void)
{
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	free(s_snap);
	s_snap = NULL;
	free(s_bp);   // (last: the render thread and the saves are done with it)
	s_bp = NULL;
	free(s_pri_hp);
	s_pri_hp = NULL;
}

static int board_of(const char *machine)
{
	if (!strncmp(machine, "sngkace:", 8)) return B_SNGKACE;
	if (!strncmp(machine, "gunbird:", 8)) return B_GUNBIRD;
	if (!strncmp(machine, "s1945n:", 7)) return B_S1945N;
	if (!strncmp(machine, "s1945:", 6)) return B_S1945;
	return -1;   // (the bootlegs on OKI: not yet)
}
static int psikyo_supported(const char *machine) { return board_of(machine) >= 0; }

static int psikyo_init(const rbx_game *g)
{
	if (!s_pri_hp && !(s_pri_hp = calloc(1, sizeof(*s_pri_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_bp && !(s_bp = (struct psikyo_state *)calloc(1, sizeof(*s_bp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_board = board_of(g->machine);
	if (s_board < 0) {
		rbx_fail("%s: machine %s not emulated", g->set, g->machine);
		return -1;
	}
	uint32_t lut_size = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_spr = rbx_region("sprites", &s_spr_size);
	s_tiles = rbx_region("tiles", &s_tiles_size);
	s_lut = (const uint16_t *)rbx_region("spritelut", &lut_size);
	if (!s_rom || !s_zrom || !s_spr || !s_tiles || !s_lut) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_lut_n = lut_size / 2;
	m68k_bus_swap16(s_rom, s_rom_size);
	find_idle_loop();
	s_ka302c = s_board == B_GUNBIRD || s_board == B_S1945N;
	const char *init = strchr(g->machine, ':') + 1;
	s_mcu_table = !strcmp(init, "s1945") ? s1945_table : !strcmp(init, "s1945a") ? s1945a_table
	            : !strcmp(init, "s1945j") ? s1945j_table : NULL;   // (tengai: none)
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.irq = ym_irq;
	if (s_board == B_S1945) {
		s_ymf = rbx_region("ymf", &s_ymf_size);
		yc.type = RBX_YMF278B;
		yc.clock = 33868800;
		yc.adpcma = s_ymf;
		yc.adpcma_size = s_ymf_size;
		yc.gain[0] = 256;
	} else {
		s_adpcma = rbx_region("ymsnd:adpcma", &s_adpcma_size);
		s_adpcmb = rbx_region("ymsnd:adpcmb", &s_adpcmb_size);
		if (s_board == B_SNGKACE && s_adpcma) {   // init_sngkace: bits 6 and 7 of the samples swapped
			for (uint32_t i = 0; i < s_adpcma_size; i++) {
				const uint8_t x = s_adpcma[i];
				s_adpcma[i] = (uint8_t)(((x & 0x40) << 1) | ((x & 0x80) >> 1) | (x & 0x3f));
			}
		}
		yc.type = RBX_YM2610;
		yc.clock = 8000000;
		yc.adpcma = s_adpcma;
		yc.adpcma_size = s_adpcma_size;
		yc.adpcmb = s_adpcmb ? s_adpcmb : s_adpcma;   // (no delta-T region: the ADPCM-A one)
		yc.adpcmb_size = s_adpcmb ? s_adpcmb_size : s_adpcma_size;
		yc.gain[0] = yc.gain[1] = yc.gain[2] = s_board == B_SNGKACE ? 256 : 141;   // (1.0 / 0.55)
	}
	s_port_p1p2 = rbx_port_find("P1_P2");
	s_port_dsw = rbx_port_find("DSW");
	s_port_coin = rbx_port_find("COIN");
	memset(&b, 0, sizeof(b));
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68EC020);
	map_mem();
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		psikyo_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, PAL_BLACK + 1)) {
		rbx_fail("%s: out of memory", g->set);
		psikyo_exit();
		return -1;
	}
	psikyo_reset();
	return 0;
}

static int psikyo_state(void *ctx, int load)
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
	rc |= rbx_state_io(ctx, "psikyo", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= rbx_state_io(ctx, "zirq", &s_zirq, sizeof(s_zirq));
	rc |= rbx_ym_state(0, ctx, load);
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		z.read_byte = z_read;
		z.write_byte = z_write;
		z.port_in = z_in;
		z.port_out = z_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		s_z80 = z;
		z_bank();
		s_pal_valid = 0;
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int psikyo_save(void *ctx) { return psikyo_state(ctx, 0); }
static int psikyo_load(void *ctx) { return psikyo_state(ctx, 1); }

const rbx_driver rbx_drv_psikyo = { "Psikyo 68EC020", psikyo_init, psikyo_exit, psikyo_reset, psikyo_frame, psikyo_save,
                                    psikyo_load, psikyo_supported, psikyo_render, psikyo_render_help };
