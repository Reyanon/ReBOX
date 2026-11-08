/*  Taito TC0180VCU (B System tilemaps, sprites and pixel frame buffer), ported from MAME
    src/mame/taito/tc0180vcu.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria; sprite/frame buffer
    emulation by Jarek Burczynski) and the screen update of taito_b_v.cpp (GPL-2.0+, Jarek Burczynski).
    Tilemaps are drawn straight from tile RAM every frame (no tilemap cache), with graphics decoded
    once at load into one byte per pixel.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "tc0180vcu.h"

#include <stdlib.h>
#include <string.h>

#define SCREEN_W 320
#define SCREEN_H 224
#define SCREEN_Y0 16      // first visible line of the 256-line screen

#define VRAM(v) ((const uint16_t *)(v)->ram)
#define SPRITERAM_WORD 0x8000
#define SCROLLRAM_WORD 0x9c00

// MAME gfx layouts: planes { 0, 8, half, half + 8 } (first plane = most significant bit)
static void decode(tc0180vcu_t *v, const uint8_t *gfx, uint32_t size)
{
	uint32_t half = size / 2;
	v->nchars = half / 16;
	v->ntiles = half / 64;
	v->chars = (uint8_t *)malloc((size_t)v->nchars * 64);
	v->tiles = (uint8_t *)malloc((size_t)v->ntiles * 256);
	v->char_opaque = (uint8_t *)malloc(v->nchars);
	v->tile_opaque = (uint8_t *)malloc(v->ntiles);
	if (!v->chars || !v->tiles || !v->char_opaque || !v->tile_opaque)
		return;
	for (uint32_t c = 0; c < v->nchars; c++) {
		int set = 0, total = 0;
		for (int y = 0; y < 8; y++) {
			uint32_t o = c * 16 + y * 2;
			uint8_t p0 = gfx[o], p1 = gfx[o + 1], p2 = gfx[half + o], p3 = gfx[half + o + 1];
			for (int x = 0; x < 8; x++) {
				int b = 7 - x;
				uint8_t px = (uint8_t)(((p0 >> b) & 1) << 3 | ((p1 >> b) & 1) << 2 | ((p2 >> b) & 1) << 1 | ((p3 >> b) & 1));
				v->chars[c * 64 + y * 8 + x] = px;
				set += px != 0;
				total++;
			}
		}
		v->char_opaque[c] = set == 0 ? 0 : set == total ? 2 : 1;
	}
	for (uint32_t t = 0; t < v->ntiles; t++) {
		int set = 0;
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				// quarter: x 8..15 at +16 bytes, y 8..15 at +32 bytes
				uint32_t o = t * 64 + (x >> 3) * 16 + (y >> 3) * 32 + (y & 7) * 2;
				int b = 7 - (x & 7);
				uint8_t px = (uint8_t)(((gfx[o] >> b) & 1) << 3 | ((gfx[o + 1] >> b) & 1) << 2 |
				                       ((gfx[half + o] >> b) & 1) << 1 | ((gfx[half + o + 1] >> b) & 1));
				v->tiles[t * 256 + y * 16 + x] = px;
				set += px != 0;
			}
		v->tile_opaque[t] = set == 0 ? 0 : set == 256 ? 2 : 1;
	}
}

int vcu_init(tc0180vcu_t *v, const uint8_t *gfx, uint32_t size, int fb_base, int bg_base, int fg_base, int tx_base)
{
	memset(v, 0, sizeof(*v));
	v->fb_color_base = fb_base * 16;
	v->bg_color_base = bg_base;
	v->fg_color_base = fg_base;
	v->tx_color_base = tx_base;
	decode(v, gfx, size);
	v->fb[0] = (uint16_t *)calloc(2 * VCU_FB_W * VCU_FB_H, 2);
	v->fb[1] = v->fb[0] ? v->fb[0] + VCU_FB_W * VCU_FB_H : NULL;
	if (!v->fb[0] || !v->chars || !v->tiles || !v->char_opaque || !v->tile_opaque || !v->ntiles)
		return -1;
	vcu_reset(v);
	return 0;
}

void vcu_exit(tc0180vcu_t *v)
{
	free(v->chars);
	free(v->tiles);
	free(v->char_opaque);
	free(v->tile_opaque);
	free(v->fb[0]);
	v->fb[0] = v->fb[1] = NULL;
	v->chars = v->tiles = v->char_opaque = v->tile_opaque = NULL;
}

void vcu_reset(tc0180vcu_t *v)
{
	memset(v->ctrl, 0, sizeof(v->ctrl));
	v->bg_bank[0] = v->bg_bank[1] = 0;
	v->fg_bank[0] = v->fg_bank[1] = 0;
	v->tx_bank = 0;
	v->fb_page = 0;
	v->video_control = 0;
}

static void video_control(tc0180vcu_t *v, uint8_t data)
{
	v->video_control = data;
	if (data & 0x80)
		v->fb_page = (~data >> 6) & 1;
}

void vcu_ctrl_w(tc0180vcu_t *v, int offset, uint16_t data, uint16_t mask)
{
	offset &= 15;
	uint16_t old = v->ctrl[offset];
	v->ctrl[offset] = (uint16_t)((old & ~mask) | (data & mask));
	if (old == v->ctrl[offset] || !(mask & 0xff00))
		return;
	uint16_t c = v->ctrl[offset];
	switch (offset) {
	case 0:
		v->fg_bank[0] = ((c >> 8) & 0x0f) << 12;
		v->fg_bank[1] = ((c >> 12) & 0x0f) << 12;
		break;
	case 1:
		v->bg_bank[0] = ((c >> 8) & 0x0f) << 12;
		v->bg_bank[1] = ((c >> 12) & 0x0f) << 12;
		break;
	case 6:
		v->tx_bank = ((c >> 8) & 0x0f) << 11;
		break;
	case 7:
		video_control(v, (uint8_t)(c >> 8));
		break;
	}
}

uint16_t vcu_ctrl_r(tc0180vcu_t *v, int offset) { return v->ctrl[offset & 15]; }

uint16_t vcu_fb_r(tc0180vcu_t *v, uint32_t offset)
{
	int sy = (int)(offset >> 8), sx = 2 * (int)(offset & 0xff);
	const uint16_t *p = &v->fb[(sy >> 8) & 1][(sy & 0xff) * VCU_FB_W + sx];
	return (uint16_t)((p[0] & 0xff) << 8 | (p[1] & 0xff));
}

void vcu_fb_w(tc0180vcu_t *v, uint32_t offset, uint16_t data, uint16_t mask)
{
	int sy = (int)(offset >> 8), sx = 2 * (int)(offset & 0xff);
	uint16_t *p = &v->fb[(sy >> 8) & 1][(sy & 0xff) * VCU_FB_W + sx];
	if (mask & 0xff00)
		p[0] = data >> 8;
	if (mask & 0x00ff)
		p[1] = data & 0xff;
}

// ---- sprites (drawn into the frame buffer at vblank) ----
static void draw_tile_fb(tc0180vcu_t *v, uint16_t *fb, uint32_t code, int color, int flipx, int flipy, int sx, int sy,
                         uint32_t scalex, uint32_t scaley)
{
	const int minx = 0, maxx = SCREEN_W - 1, miny = SCREEN_Y0, maxy = SCREEN_Y0 + SCREEN_H - 1;
	code %= v->ntiles;
	if (!v->tile_opaque[code])
		return;
	const uint8_t *src = v->tiles + code * 256;
	if (scalex == 0x10000 && scaley == 0x10000) {
		for (int y = 0; y < 16; y++) {
			int dy = sy + y;
			if (dy < miny || dy > maxy)
				continue;
			const uint8_t *row = src + (flipy ? 15 - y : y) * 16;
			uint16_t *d = fb + dy * VCU_FB_W;
			for (int x = 0; x < 16; x++) {
				int dx = sx + x;
				if (dx < minx || dx > maxx)
					continue;
				uint8_t c = row[flipx ? 15 - x : x];
				if (c)
					d[dx] = (uint16_t)(color + c);
			}
		}
		return;
	}
	// MAME drawgfx zoom: destination size from the 16.16 scale, source stepped back from it
	int dstw = (int)((scalex * 16 + 0x8000) >> 16), dsth = (int)((scaley * 16 + 0x8000) >> 16);
	if (dstw < 1 || dsth < 1)
		return;
	int dx = (16 << 16) / dstw, dy = (16 << 16) / dsth;
	int ex = sx + dstw, ey = sy + dsth;
	int xbase = 0, yidx = 0;
	if (flipx) { xbase = (dstw - 1) * dx; dx = -dx; }
	if (flipy) { yidx = (dsth - 1) * dy; dy = -dy; }
	if (sx < minx) { xbase += (minx - sx) * dx; sx = minx; }
	if (sy < miny) { yidx += (miny - sy) * dy; sy = miny; }
	if (ex > maxx + 1) ex = maxx + 1;
	if (ey > maxy + 1) ey = maxy + 1;
	for (int y = sy; y < ey; y++, yidx += dy) {
		const uint8_t *row = src + (yidx >> 16) * 16;
		uint16_t *d = fb + y * VCU_FB_W;
		int xi = xbase;
		for (int x = sx; x < ex; x++, xi += dx) {
			uint8_t c = row[xi >> 16];
			if (c)
				d[x] = (uint16_t)(color + c);
		}
	}
}

static void draw_sprites(tc0180vcu_t *v, uint16_t *fb)
{
	const uint16_t *sr = VRAM(v) + SPRITERAM_WORD;
	int xlatch = 0, ylatch = 0, x_no = 0, y_no = 0, x_num = 0, y_num = 0;
	uint32_t zoomxlatch = 0, zoomylatch = 0;
	int big = 0;
	for (int offs = (0x1980 - 16) / 2; offs >= 0; offs -= 8) {
		int code = sr[offs];
		int color = sr[offs + 1];
		int flipx = (color >> 14) & 1, flipy = (color >> 15) & 1;
		color = (color & 0x3f) * 16;
		int x = sr[offs + 2] & 0x3ff, y = sr[offs + 3] & 0x3ff;
		if (x >= 0x200) x -= 0x400;
		if (y >= 0x200) y -= 0x400;
		uint32_t data = sr[offs + 5];
		if (data && !big) {
			x_num = (data >> 8) & 0xff;
			y_num = data & 0xff;
			x_no = y_no = 0;
			xlatch = x;
			ylatch = y;
			data = sr[offs + 4];
			zoomxlatch = (data >> 8) & 0xff;
			zoomylatch = data & 0xff;
			big = 1;
		}
		data = sr[offs + 4];
		uint32_t zoomx = (data >> 8) & 0xff, zoomy = data & 0xff;
		uint32_t zx = (0x100 - zoomx) / 16, zy = (0x100 - zoomy) / 16;
		if (big) {
			zoomx = zoomxlatch;
			zoomy = zoomylatch;
			x = xlatch + (int)((x_no * (0xff - zoomx) + 15) / 16);
			y = ylatch + (int)((y_no * (0xff - zoomy) + 15) / 16);
			zx = (uint32_t)(xlatch + (int)(((x_no + 1) * (0xff - zoomx) + 15) / 16) - x);
			zy = (uint32_t)(ylatch + (int)(((y_no + 1) * (0xff - zoomy) + 15) / 16) - y);
			if (++y_no > y_num) {
				y_no = 0;
				if (++x_no > x_num)
					big = 0;
			}
		}
		if (zoomx || zoomy)
			draw_tile_fb(v, fb, (uint32_t)code, color, flipx, flipy, x, y, (zx << 16) / 16, (zy << 16) / 16);
		else
			draw_tile_fb(v, fb, (uint32_t)code, color, flipx, flipy, x, y, 0x10000, 0x10000);
	}
}

void vcu_vblank_flip(tc0180vcu_t *v, int *draw_page, int *clear)
{
	// MAME vblank_update: clear the shown page, flip, draw into the new one (the clear is done by vcu_draw_sprites)
	*clear = !(v->video_control & 0x01) ? v->fb_page + 1 : 0;   // page + 1, 0 = none
	if (!(v->video_control & 0x80))
		v->fb_page ^= 1;
	*draw_page = v->fb_page;
}

void vcu_draw_sprites(tc0180vcu_t *v, int draw_page, int clear)
{
	if (clear) {
		uint16_t *fb = v->fb[clear - 1];
		for (int y = SCREEN_Y0; y < SCREEN_Y0 + SCREEN_H; y++)
			memset(fb + y * VCU_FB_W, 0, SCREEN_W * 2);
	}
	draw_sprites(v, v->fb[draw_page]);
}

void vcu_vblank(tc0180vcu_t *v)
{
	int page, clear;
	vcu_vblank_flip(v, &page, &clear);
	vcu_draw_sprites(v, page, clear);
}

// ---- screen ----
// flipped screen (video control bit 4): MAME flips the tilemaps around the centre of the visible area (tilemap.cpp
// draw_common: x0 + x1 + 1 = 320, y0 + y1 + 1 = 256; pixel x, y shows the unflipped x' = 319 - x, y' = 255 - y) and the
// frame buffer the same way. A flipped row is the unflipped row from x' = 0, drawn into a buffer and copied back mirrored
// (rambo3 runs flipped: over the whole 512 wide bitmap its title stood 192 pixels to the right)

// one row of a 16x16 layer: 320 pixels of tilemap line ty from tilemap column tx
static void layer16_row(tc0180vcu_t *v, uint16_t *d, int ty, int tx, const int *bank, int color_base, int trans)
{
	const uint16_t *vram = VRAM(v);
	int trow = (ty >> 4) * 64, py = ty & 15;
	int x = 0;
	while (x < SCREEN_W) {
		int px = tx & 15;
		int n = 16 - px;
		if (n > SCREEN_W - x)
			n = SCREEN_W - x;
		int idx = trow + (tx >> 4);
		uint32_t code = vram[bank[0] + idx] % v->ntiles;
		uint16_t attr = vram[bank[1] + idx];
		int op = v->tile_opaque[code];
		if (op || !trans) {
			uint16_t color = (uint16_t)((color_base + (attr & 0x3f)) << 4);
			const uint8_t *src = v->tiles + code * 256 + ((attr & 0x80) ? 15 - py : py) * 16;
			if (attr & 0x40) {
				for (int k = 0; k < n; k++) {
					uint8_t c = src[15 - (px + k)];
					if (c || !trans)
						d[x + k] = color | c;
				}
			} else if (op == 2 || !trans) {
				for (int k = 0; k < n; k++)
					d[x + k] = color | src[px + k];
			} else {
				for (int k = 0; k < n; k++) {
					uint8_t c = src[px + k];
					if (c)
						d[x + k] = color | c;
				}
			}
		}
		x += n;
		tx = (tx + n) & 1023;
	}
}

static void mirror_row(uint16_t *dst, const uint16_t *src)
{
	for (int x = 0; x < SCREEN_W; x++)
		dst[x] = src[SCREEN_W - 1 - x];
}

static void draw_layer16(tc0180vcu_t *v, uint16_t *bitmap, int pitch, int plane, const int *bank, int color_base, int trans)
{
	const uint16_t *scroll = VRAM(v) + SCROLLRAM_WORD + plane * 0x200;
	int lpb = 256 - (v->ctrl[2 + plane] >> 8);
	int nblocks = 256 / lpb;
	int flip = v->video_control & 0x10;
	uint16_t tmp[SCREEN_W];
	for (int r = 0; r < SCREEN_H; r++) {
		int y = flip ? 255 - (r + SCREEN_Y0) : r + SCREEN_Y0;
		int blk = y / lpb;
		if (blk >= nblocks)
			continue;
		int sx = (int16_t)scroll[blk * 2 * lpb], sy = (int16_t)scroll[blk * 2 * lpb + 1];
		int ty = (y - sy) & 1023;
		uint16_t *d = bitmap + r * pitch;
		if (flip) {
			mirror_row(tmp, d);
			layer16_row(v, tmp, ty, (-sx) & 1023, bank, color_base, trans);
			mirror_row(d, tmp);
		} else {
			layer16_row(v, d, ty, (-sx) & 1023, bank, color_base, trans);
		}
	}
}

void vcu_draw_bg(tc0180vcu_t *v, uint16_t *bitmap, int pitch) { draw_layer16(v, bitmap, pitch, 1, v->bg_bank, v->bg_color_base, 0); }
void vcu_draw_fg(tc0180vcu_t *v, uint16_t *bitmap, int pitch) { draw_layer16(v, bitmap, pitch, 0, v->fg_bank, v->fg_color_base, 1); }

// one row of the text layer: 40 characters of line y from column col0
static void tx_row(tc0180vcu_t *v, uint16_t *d, int y, int col0)
{
	const uint16_t *vram = VRAM(v);
	int row = (y >> 3) & 31, py = y & 7;
	for (int col = 0; col < SCREEN_W / 8; col++) {
		uint16_t tile = vram[v->tx_bank + row * 64 + ((col0 + col) & 63)];
		uint32_t code = ((tile & 0x07ff) | ((uint32_t)(v->ctrl[4 + ((tile >> 11) & 1)] >> 8) << 11)) % v->nchars;
		if (!v->char_opaque[code])
			continue;
		uint16_t color = (uint16_t)((v->tx_color_base + ((tile >> 12) & 0x0f)) << 4);
		const uint8_t *src = v->chars + code * 64 + py * 8;
		uint16_t *dd = d + col * 8;
		for (int k = 0; k < 8; k++)
			if (src[k])
				dd[k] = color | src[k];
	}
}

void vcu_draw_tx(tc0180vcu_t *v, uint16_t *bitmap, int pitch)
{
	int flip = v->video_control & 0x10;
	uint16_t tmp[SCREEN_W];
	for (int r = 0; r < SCREEN_H; r++) {
		uint16_t *d = bitmap + r * pitch;
		if (flip) {
			mirror_row(tmp, d);
			tx_row(v, tmp, 255 - (r + SCREEN_Y0), 0);
			mirror_row(d, tmp);
		} else {
			tx_row(v, d, r + SCREEN_Y0, 0);
		}
	}
}

void vcu_draw_fb(tc0180vcu_t *v, uint16_t *bitmap, int pitch, int priority)
{
	priority <<= 4;
	int all = v->video_control & 0x08;
	if (all && priority)
		return;
	const uint16_t *fb = v->fb[v->fb_page];
	if (v->video_control & 0x10) {
		// MAME: frame buffer line y goes to screen line 255 - y, mirrored over the visible width
		for (int r = 0; r < SCREEN_H; r++) {
			const uint16_t *s = fb + (255 - (r + SCREEN_Y0)) * VCU_FB_W;
			uint16_t *d = bitmap + r * pitch;
			for (int x = 0; x < SCREEN_W; x++) {
				uint16_t c = s[SCREEN_W - 1 - x];
				if (c && (all || (c & 0x10) == priority))
					d[x] = (uint16_t)(v->fb_color_base + c);
			}
		}
		return;
	}
	for (int r = 0; r < SCREEN_H; r++) {
		const uint16_t *s = fb + (r + SCREEN_Y0) * VCU_FB_W;
		uint16_t *d = bitmap + r * pitch;
		if (all) {
			for (int x = 0; x < SCREEN_W; x++)
				if (s[x])
					d[x] = (uint16_t)(v->fb_color_base + s[x]);
		} else {
			for (int x = 0; x < SCREEN_W; x++) {
				uint16_t c = s[x];
				if (c && (c & 0x10) == priority)
					d[x] = (uint16_t)(v->fb_color_base + c);
			}
		}
	}
}

void vcu_render(tc0180vcu_t *v, uint16_t *bitmap, int pitch)
{
	vcu_draw_bg(v, bitmap, pitch);
	vcu_draw_fb(v, bitmap, pitch, 1);
	vcu_draw_fg(v, bitmap, pitch);
	vcu_draw_fb(v, bitmap, pitch, 0);
	vcu_draw_tx(v, bitmap, pitch);
}
