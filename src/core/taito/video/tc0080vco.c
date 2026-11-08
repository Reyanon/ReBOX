/*  Taito TC0080VCO, ported from MAME src/mame/taito/tc0080vco.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria).
    Layers are drawn straight from RAM each frame (no tilemap cache), one tile row segment at a time: tiles are
    decoded at load to a byte per pixel with per row flags (empty / partly / fully set), the text characters are
    decoded from char RAM once per frame. Screen flip follows MAME tilemap flipping (scroll mirrored over the screen
    size) and the flipped formulas of bg0_tilemap_draw / bg1_tilemap_draw / get_sprite_params.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "tc0080vco.h"

#include <stdlib.h>
#include <string.h>

// RAM layout (word offsets)
#define CHAIN0   0x0000   // chain RAM 0 / char RAM low planes (0x0000-0x07ff)
#define TX0      0x0800
#define BG0_0    0x6000
#define BG1_0    0x7000
#define CHAIN1   0x8000   // chain RAM 1 / char RAM high plane
#define BG0_1    0xe000
#define BG1_1    0xf000
#define BGSCROLL 0x10000
#define SPRITES  VCO_SPRITES
#define SCROLL   VCO_SCROLL

#define SHL16(x) ((uint32_t)(x) << 16)

// byte n of entry b = bit (7 - n) of b: eight text pixels of one plane at once
static uint64_t s_spread[256];

int vco_init(tc0080vco_t *v, const uint8_t *gfx, uint32_t size, int bg_xoffs, int bg_yoffs, int bg_flip_yoffs)
{
	memset(v, 0, sizeof(*v));
	v->bg_xoffs = bg_xoffs;
	v->bg_yoffs = bg_yoffs;
	v->bg_flip_yoffs = bg_flip_yoffs;
	v->screen_w = v->screen_h = 1024;
	v->ntiles = size / 128;
	v->tiles = (uint8_t *)malloc((size_t)v->ntiles * 256);
	v->opaque = (uint8_t *)malloc(v->ntiles);
	v->rowflags = (uint8_t *)malloc((size_t)v->ntiles * 16);
	if (!v->tiles || !v->opaque || !v->rowflags || !v->ntiles)
		return -1;
	for (int b = 0; b < 256; b++) {
		uint8_t px[8];
		for (int n = 0; n < 8; n++)
			px[n] = (uint8_t)((b >> (7 - n)) & 1);
		memcpy(&s_spread[b], px, 8);
	}
	// 16x16, 4 bits per pixel packed: row = 8 bytes, pixel x in byte 7 - x/2 (even x: low nibble)
	for (uint32_t t = 0; t < v->ntiles; t++) {
		int set = 0;
		for (int y = 0; y < 16; y++) {
			const uint8_t *row = gfx + t * 128 + y * 8;
			int rs = 0;
			for (int x = 0; x < 16; x++) {
				uint8_t b = row[7 - (x >> 1)];
				uint8_t c = (x & 1) ? (uint8_t)(b >> 4) : (uint8_t)(b & 15);
				v->tiles[t * 256 + y * 16 + x] = c;
				rs += c != 0;
			}
			v->rowflags[t * 16 + y] = rs == 0 ? 0 : rs == 16 ? 2 : 1;
			set += rs;
		}
		v->opaque[t] = set == 0 ? 0 : set == 256 ? 2 : 1;
	}
	return 0;
}

void vco_exit(tc0080vco_t *v)
{
	free(v->tiles);
	free(v->opaque);
	free(v->rowflags);
	v->tiles = v->opaque = v->rowflags = NULL;
}

void vco_reset(tc0080vco_t *v)
{
	memset(v->ram, 0, sizeof(v->ram));
}

void vco_share(tc0080vco_t *snap, const tc0080vco_t *v)
{
	snap->tiles = v->tiles;
	snap->opaque = v->opaque;
	snap->rowflags = v->rowflags;
	snap->ntiles = v->ntiles;
	snap->bg_xoffs = v->bg_xoffs;
	snap->bg_yoffs = v->bg_yoffs;
	snap->bg_flip_yoffs = v->bg_flip_yoffs;
	snap->screen_w = v->screen_w;
	snap->screen_h = v->screen_h;
}

static inline uint32_t wrap_code(const tc0080vco_t *v, uint32_t code)
{
	return code < v->ntiles ? code : code % v->ntiles;
}

// one row of a 16x16 layer at normal size: tilemap line ty, tilemap column tx at bitmap pixel 0, walking right
// (dir 1) or left (dir -1, flipped screen)
static void layer_row(const tc0080vco_t *v, uint16_t *d, int w, int ram0, int ram1, int tx, int ty, int dir, int opaque)
{
	const int py = ty & 15;
	const uint16_t *r0 = v->ram + ram0 + ((ty >> 4) & 63) * 64;
	const uint16_t *r1 = v->ram + ram1 + ((ty >> 4) & 63) * 64;
	int x = 0;
	while (x < w) {
		tx &= 1023;
		int px = tx & 15, n = dir > 0 ? 16 - px : px + 1;
		if (n > w - x)
			n = w - x;
		int cell = tx >> 4;
		uint16_t attr = r1[cell];
		uint32_t code = wrap_code(v, r0[cell] & 0x7fff);
		int sy = (attr & 0x80) ? 15 - py : py;
		int f = v->rowflags[code * 16 + sy];
		if (f || opaque) {
			uint16_t col = (uint16_t)((attr & 0x1f) << 4);
			const uint8_t *s = v->tiles + code * 256 + sy * 16 + ((attr & 0x40) ? 15 - px : px);
			uint16_t *o = d + x;
			int fwd = ((attr & 0x40) != 0) == (dir < 0);
			if (f == 2 || opaque) {
				if (fwd)
					for (int k = 0; k < n; k++) o[k] = (uint16_t)(col | s[k]);
				else
					for (int k = 0; k < n; k++) o[k] = (uint16_t)(col | s[-k]);
			} else {
				if (fwd) {
					for (int k = 0; k < n; k++)
						if (s[k]) o[k] = (uint16_t)(col | s[k]);
				} else {
					for (int k = 0; k < n; k++)
						if (s[-k]) o[k] = (uint16_t)(col | s[-k]);
				}
			}
		}
		x += n;
		tx += n * dir;
	}
}

// one row of a zoomed 16x16 layer: tilemap line ty, source x (16.16) xi stepping by step per pixel; xflip = 1023 for
// the flipped pixmap; bounded = no wraparound (copyrozbitmap), else the 1024 pixel tilemap wraps
static void zoom_row(const tc0080vco_t *v, uint16_t *d, int w, int ram0, int ram1, int ty, uint32_t xi, uint32_t step,
                     int xflip, int opaque, int bounded)
{
	const int py = ty & 15;
	const uint16_t *r0 = v->ram + ram0 + ((ty >> 4) & 63) * 64;
	const uint16_t *r1 = v->ram + ram1 + ((ty >> 4) & 63) * 64;
	int last = -1, fx = 0;
	const uint8_t *src = v->tiles;
	uint16_t col = 0;
	for (int x = 0; x < w; x++, xi += step) {
		uint32_t sx = xi >> 16;
		if (bounded) {
			if (sx >= 1024)
				continue;
		} else
			sx &= 1023;
		int tx = (int)sx ^ xflip, cell = tx >> 4;
		if (cell != last) {
			last = cell;
			uint16_t attr = r1[cell];
			uint32_t code = wrap_code(v, r0[cell] & 0x7fff);
			src = v->tiles + code * 256 + ((attr & 0x80) ? 15 - py : py) * 16;
			fx = (attr & 0x40) ? 15 : 0;
			col = (uint16_t)((attr & 0x1f) << 4);
		}
		uint8_t c = src[(tx & 15) ^ fx];
		if (c || opaque)
			d[x] = (uint16_t)(col | c);
	}
}

static void zoom_calc(int zx, int zy, int *zoomx, int *zoomy)
{
	if (zx < 63) {
		int dx = 16 - (zx + 2) / 8, ex = (zx + 2) % 8;
		*zoomx = ((dx << 3) - ex) << 10;
	} else
		*zoomx = 0x10000 - ((zx - 0x3f) * 256);
	if (zy < 127) {
		int dy = 16 - (zy + 2) / 16, ey = (zy + 2) % 16;
		*zoomy = ((dy << 4) - ey) << 9;
	} else
		*zoomy = 0x10000 - ((zy - 0x7f) * 512);
}

void vco_draw_bg0(tc0080vco_t *v, uint16_t *bm, int pitch, int w, int h, int x0, int y0, int opaque)
{
	const uint16_t *sr = v->ram + SCROLL;
	const int flip = vco_flipscreen(v);
	int scrollx = sr[1] & 0x3ff, scrolly = sr[3] & 0x3ff;
	int zx = (sr[6] & 0xff00) >> 8, zy = sr[6] & 0xff;
	if (zx == 0x3f && zy == 0x7f) {
		// MAME tilemap: 512 scroll rows (two lines each), scrollx = -bg0_scrollx -/+ rowscroll
		int width = v->screen_w, height = v->screen_h;
		for (int r = 0; r < h; r++) {
			int y = r + y0;
			if (!flip) {
				int ty = (y + scrolly - v->bg_yoffs) & 1023;
				int rs = v->ram[BGSCROLL + (ty >> 1)];
				layer_row(v, bm + r * pitch, w, BG0_0, BG0_1, x0 - scrollx - rs - v->bg_xoffs, ty, 1, opaque);
			} else {
				int ty = (height - 1 - y - v->bg_flip_yoffs + scrolly) & 1023;
				int rs = v->ram[BGSCROLL + (ty >> 1)];
				layer_row(v, bm + r * pitch, w, BG0_0, BG0_1, width - 1 - x0 - 512 - scrollx + rs, ty, -1, opaque);
			}
		}
		return;
	}
	// MAME bg0_tilemap_draw zoom path (rowscroll + zoom)
	int zoomx, zoomy;
	zoom_calc(zx, zy, &zoomx, &zoomy);
	uint32_t sx, y_index;
	if (!flip) {
		sx = SHL16(-(int)sr[1] - 1);
		y_index = SHL16((int)sr[3] - 1) + (uint32_t)(y0 * zoomy);
	} else {
		sx = SHL16(0x200 + sr[1]) - (uint32_t)((2 * x0 + w - 1) * (zoomx - 0x10000));
		y_index = SHL16(-(int)sr[3] - 2) + (uint32_t)(y0 * zoomy) - (uint32_t)((2 * y0 + h - 1) * (zoomy - 0x10000));
	}
	for (int r = 0; r < h; r++, y_index += (uint32_t)zoomy) {
		int sy = (int)(y_index >> 16) & 0x3ff;
		int ty = flip ? sy ^ 0x3ff : sy;   // (row areas are the same in flipscreen: read in reverse)
		uint32_t x_index = sx - SHL16(v->ram[BGSCROLL + (ty & 0x1ff)]) + (uint32_t)(x0 * zoomx);
		zoom_row(v, bm + r * pitch, w, BG0_0, BG0_1, ty, x_index, (uint32_t)zoomx, flip ? 0x3ff : 0, opaque, 0);
	}
}

void vco_draw_bg1(tc0080vco_t *v, uint16_t *bm, int pitch, int w, int h, int x0, int y0)
{
	const uint16_t *sr = v->ram + SCROLL;
	const int flip = vco_flipscreen(v);
	int scrollx = sr[2] & 0x3ff, scrolly = sr[4] & 0x3ff;
	int zx = (sr[7] & 0xff00) >> 8, zy = sr[7] & 0xff;
	if (zx == 0x3f && zy == 0x7f) {
		int width = v->screen_w, height = v->screen_h;
		for (int r = 0; r < h; r++) {
			int y = r + y0;
			if (!flip)
				layer_row(v, bm + r * pitch, w, BG1_0, BG1_1, x0 - scrollx - v->bg_xoffs,
				          (y + scrolly - v->bg_yoffs) & 1023, 1, 0);
			else
				layer_row(v, bm + r * pitch, w, BG1_0, BG1_1, width - 1 - x0 - 512 - scrollx,
				          (height - 1 - y - v->bg_flip_yoffs + scrolly) & 1023, -1, 0);
		}
		return;
	}
	// MAME copyrozbitmap, no wraparound
	int zoomx, zoomy;
	zoom_calc(zx, zy, &zoomx, &zoomy);
	uint32_t sx, sy;
	if (!flip) {
		sx = SHL16(-(int)sr[2] - 1);
		sy = SHL16((int)sr[4] - 1);
	} else {
		sx = SHL16(0x200 + sr[2]) - (uint32_t)((2 * x0 + w - 1) * (zoomx - 0x10000));
		sy = SHL16(0x3fe - sr[4]) - (uint32_t)((2 * y0 + h - 1) * (zoomy - 0x10000));
	}
	for (int r = 0; r < h; r++) {
		uint32_t syy = (sy + (uint32_t)((r + y0) * zoomy)) >> 16;
		if (syy >= 1024)
			continue;
		zoom_row(v, bm + r * pitch, w, BG1_0, BG1_1, flip ? (int)syy ^ 0x3ff : (int)syy, sx + (uint32_t)(x0 * zoomx),
		         (uint32_t)zoomx, flip ? 0x3ff : 0, 0, 1);
	}
}

// decode the 256 text characters (3 bits per pixel from char RAM) to bytes, with row flags; any[t] = some pixel set
static void tx_decode(tc0080vco_t *v, uint8_t *any)
{
	for (int t = 0; t < 256; t++) {
		uint8_t a = 0;
		for (int py = 0; py < 8; py++) {
			uint16_t w0 = v->ram[CHAIN0 + t * 8 + py], w1 = v->ram[CHAIN1 + t * 8 + py];
			uint8_t p0 = (uint8_t)w0, p1 = (uint8_t)(w0 >> 8), p2 = (uint8_t)w1;
			uint8_t set = (uint8_t)(p0 | p1 | p2);
			v->txrow[t * 8 + py] = set == 0 ? 0 : set == 0xff ? 2 : 1;
			if (set) {
				uint64_t q = s_spread[p0] | s_spread[p1] << 1 | s_spread[p2] << 2;
				memcpy(v->txpix + t * 64 + py * 8, &q, 8);
			}
			a |= set;
		}
		any[t] = a;
	}
}

static inline int tx_tile(const uint16_t *trow, int c, int flip)
{
	uint16_t wd = trow[c >> 1];
	return ((c & 1) ^ flip) ? (wd & 0xff) : (wd >> 8);
}

void vco_draw_tx(tc0080vco_t *v, uint16_t *bm, int pitch, int w, int h, int x0, int y0)
{
	// 64x64 tiles of 8x8 (two tile numbers per word, swapped when flipped), scrolled 48 lines (MAME scrolldy 48,
	// flipped -448). The 512 pixel tilemap wraps, the bitmap is at most 512 wide: each tilemap pixel lands on one
	// bitmap column at most, so only the cells holding a non-empty character are visited
	const int flip = vco_flipscreen(v);
	const int width = v->screen_w, height = v->screen_h;
	uint8_t any[256];
	uint64_t rowmask[64];
	tx_decode(v, any);
	for (int row = 0; row < 64; row++) {
		const uint16_t *trow = v->ram + TX0 + row * 32;
		uint64_t m = 0;
		for (int c = 0; c < 64; c++)
			if (any[tx_tile(trow, c, flip)])
				m |= (uint64_t)1 << c;
		rowmask[row] = m;
	}
	if (w > 512)
		w = 512;
	for (int r = 0; r < h; r++) {
		int y = r + y0;
		int ty = flip ? (height - 1 - y + 448) & 511 : (y - 48) & 511;
		int py = ty & 7;
		uint64_t m = rowmask[ty >> 3];
		if (!m)
			continue;
		const uint16_t *trow = v->ram + TX0 + (ty >> 3) * 32;
		uint16_t *d = bm + r * pitch;
		while (m) {
			int c = __builtin_ctzll(m);
			m &= m - 1;
			int tile = tx_tile(trow, c, flip);
			int f = v->txrow[tile * 8 + py];
			if (!f)
				continue;
			const uint8_t *s = v->txpix + tile * 64 + py * 8;
			if (!flip) {
				int xs = (c * 8 - x0) & 511;
				if (f == 2 && xs + 8 <= w) {
					for (int k = 0; k < 8; k++)
						d[xs + k] = (uint16_t)(512 + s[k]);
				} else {
					for (int k = 0; k < 8; k++) {
						int x = (xs + k) & 511;
						if (x < w && s[k])
							d[x] = (uint16_t)(512 + s[k]);
					}
				}
			} else {
				int xs = width - 1 - x0 - c * 8;
				for (int k = 0; k < 8; k++) {
					int x = (xs - k) & 511;
					if (x < w && s[k])
						d[x] = (uint16_t)(512 + s[k]);
				}
			}
		}
	}
}

// MAME drawgfx zoom_transpen (pen 0 transparent)
static void zoom_tile(const tc0080vco_t *v, uint16_t *bm, int pitch, int w, int h, uint32_t code, int color, int flipx,
                      int flipy, int sx, int sy, int scalex, int scaley)
{
	code = wrap_code(v, code);
	if (!v->opaque[code])
		return;
	int dstw = (scalex * 16 + 0x8000) >> 16, dsth = (scaley * 16 + 0x8000) >> 16;
	if (dstw < 1 || dsth < 1)
		return;
	int dx = (16 << 16) / dstw, dy = (16 << 16) / dsth;
	int ex = sx + dstw, ey = sy + dsth;
	int xbase = 0, yidx = 0;
	if (flipx) { xbase = (dstw - 1) * dx; dx = -dx; }
	if (flipy) { yidx = (dsth - 1) * dy; dy = -dy; }
	if (sx < 0) { xbase += -sx * dx; sx = 0; }
	if (sy < 0) { yidx += -sy * dy; sy = 0; }
	if (ex > w) ex = w;
	if (ey > h) ey = h;
	if (sx >= ex)
		return;
	const uint8_t *src = v->tiles + code * 256;
	const uint8_t *rf = v->rowflags + code * 16;
	uint16_t base = (uint16_t)(color * 16);
	for (int y = sy; y < ey; y++, yidx += dy) {
		int row = yidx >> 16;
		if (!rf[row])
			continue;
		const uint8_t *s = src + row * 16;
		uint16_t *d = bm + y * pitch;
		int xi = xbase;
		for (int x = sx; x < ex; x++, xi += dx) {
			uint8_t c = s[xi >> 16];
			if (c)
				d[x] = (uint16_t)(base + c);
		}
	}
}

static const uint8_t s_zoomy_conv[] = {
	0x00,0x01,0x01,0x02,0x02,0x03,0x04,0x05, 0x06,0x06,0x07,0x08,0x09,0x0a,0x0a,0x0b,
	0x0b,0x0c,0x0c,0x0d,0x0e,0x0e,0x0f,0x10, 0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x16,
	0x17,0x18,0x19,0x1a,0x1b,0x1c,0x1d,0x1e, 0x1f,0x20,0x21,0x22,0x24,0x25,0x26,0x27,
	0x28,0x2a,0x2b,0x2c,0x2e,0x30,0x31,0x32, 0x34,0x36,0x37,0x38,0x3a,0x3c,0x3e,0x3f,
	0x40,0x41,0x42,0x42,0x43,0x43,0x44,0x44, 0x45,0x45,0x46,0x46,0x47,0x47,0x48,0x49,
	0x4a,0x4a,0x4b,0x4b,0x4c,0x4d,0x4e,0x4f, 0x4f,0x50,0x51,0x51,0x52,0x53,0x54,0x55,
	0x56,0x57,0x58,0x59,0x5a,0x5b,0x5c,0x5d, 0x5e,0x5f,0x60,0x61,0x62,0x63,0x64,0x66,
	0x67,0x68,0x6a,0x6b,0x6c,0x6e,0x6f,0x71, 0x72,0x74,0x76,0x78,0x80,0x7b,0x7d,0x7f
};

static void zoom_params(int zoom, int *d, int *e, int *z)
{
	if (zoom < 63) {
		*d = 8 + (zoom + 2) / 8;
		*e = (zoom + 2) % 8;
		*z = ((*d << 1) + *e) << 11;
	} else {
		*d = 16 + (zoom - 63) / 4;
		*e = (zoom - 63) % 4;
		*z = (*d + *e) << 12;
	}
}

uint32_t vco_sprite_params(const tc0080vco_t *v, int offs, int zoomy_enable, vco_sprite_t *s)
{
	const uint16_t *spr = v->ram + SPRITES;
	static const uint8_t size[] = { 1, 2, 4, 4 };
	s->tile_offs = (uint32_t)(spr[offs + 3] & 0x1fff) << 2;
	if (!s->tile_offs)
		return 0;
	s->x0 = spr[offs + 1] & 0x3ff;
	s->y0 = spr[offs] & 0x3ff;
	s->zoomx = (spr[offs + 2] & 0x7f00) >> 8;
	s->zoomy = zoomy_enable ? s_zoomy_conv[spr[offs + 2] & 0x7f] : s->zoomx;
	s->ysize = size[(spr[offs] & 0x0c00) >> 10];
	int ex, ey;
	zoom_params(s->zoomx, &s->dx, &ex, &s->zx);
	(void)ex;
	if (s->zoomx == s->zoomy) {
		s->dy = s->dx;
		s->zy = s->zx;
	} else
		zoom_params(s->zoomy, &s->dy, &ey, &s->zy);
	(void)ey;
	if (s->x0 >= 0x200) s->x0 -= 0x400;
	if (s->y0 >= 0x200) s->y0 -= 0x400;
	if (vco_flipscreen(v)) {
		s->x0 = 497 - s->x0;
		s->y0 = 498 - s->y0;
		s->dx = -s->dx;
		s->dy = -s->dy;
	} else {
		s->x0 += 1;
		s->y0 += 2;
	}
	return s->tile_offs;
}

void vco_sprite_draw(const tc0080vco_t *v, uint16_t *bm, int pitch, int w, int h, int x0, int y0, const vco_sprite_t *s)
{
	const int flip = vco_flipscreen(v);
	uint32_t tile_offs = s->tile_offs;
	int y = s->y0;
	for (int j = 0; j < s->ysize; j++) {
		int x = s->x0;
		for (int k = 0; k < 4; k++) {
			if (tile_offs >= 0x1000) {   // (or dleague pitcher gets blanked)
				uint32_t tile = v->ram[CHAIN0 + tile_offs] & 0x7fff;
				uint16_t a = v->ram[CHAIN1 + tile_offs];
				int fx = a & 0x40, fy = a & 0x80;
				if (flip) {
					fx ^= 0x40;
					fy ^= 0x80;
				}
				zoom_tile(v, bm, pitch, w, h, tile, a & 0x1f, fx, fy, x - x0, y - y0, s->zx, s->zy);
			}
			tile_offs++;
			x += s->dx;
		}
		y += s->dy;
	}
}

int vco_draw_sprites(tc0080vco_t *v, uint16_t *bm, int pitch, int w, int h, int x0, int y0, int offs, int zoomy_enable,
                     int stop_at_terminator)
{
	const uint16_t *spr = v->ram + SPRITES;
	for (; offs >= 0; offs -= 4) {
		if (stop_at_terminator && (spr[offs] == 0xc00 || spr[offs] == 0xcff))
			return offs - 4;
		vco_sprite_t s;
		if (vco_sprite_params(v, offs, zoomy_enable, &s))
			vco_sprite_draw(v, bm, pitch, w, h, x0, y0, &s);
	}
	return 0;
}
