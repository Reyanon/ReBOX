/*  Taito TC0480SCP (tc0480scp.h), ported from MAME src/mame/taito/tc0480scp.cpp and taito_helper.cpp
    (BSD-3-Clause, copyright-holders: Nicola Salmoria) with MAME's tilemap placement rules (src/emu/tilemap.cpp).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "tc0480scp.h"

#include <stdlib.h>
#include <string.h>

#define SW 320
#define SH 256

int tc0480scp_init(tc0480scp_t *c, const uint8_t *gfx, uint32_t size)
{
	uint32_t n = size / 128;
	if (!n)
		n = 1;
	uint8_t *t = (uint8_t *)calloc((size_t)n * 257, 1);
	if (!t)
		return -1;
	uint8_t *used = t + (size_t)n * 256;
	for (uint32_t i = 0; i < n && gfx; i++) {
		uint8_t *d = t + (size_t)i * 256, any = 0;
		for (int j = 0; j < 256; j++) {   // gfx_16x16x4_packed_lsb
			uint8_t b = gfx[i * 128 + (j >> 1)];
			d[j] = (j & 1) ? (b >> 4) : (b & 15);
			any |= d[j];
		}
		used[i] = any != 0;
	}
	c->owned = t;
	c->tiles = t;
	c->tile_used = used;
	c->ntiles = n;
	tc0480scp_reset(c);
	return 0;
}

void tc0480scp_exit(tc0480scp_t *c)
{
	free(c->owned);
	c->owned = NULL;
	c->tiles = c->tile_used = NULL;
}

void tc0480scp_reset(tc0480scp_t *c)
{
	memset(c->ram, 0, sizeof(c->ram));
	memset(c->ctrl, 0, sizeof(c->ctrl));
	c->pri_reg = 0;
	memset(c->bgscrollx, 0, sizeof(c->bgscrollx));
	memset(c->bgscrolly, 0, sizeof(c->bgscrolly));
	c->txscrollx = c->txscrolly = 0;
}

void tc0480scp_ctrl_w(tc0480scp_t *c, int offset, uint16_t data, uint16_t mask)
{
	if (offset >= 0x18)
		return;
	int flip = c->pri_reg & 0x40;
	c->ctrl[offset] = (uint16_t)((c->ctrl[offset] & ~mask) | (data & mask));
	int d = c->ctrl[offset];
	switch (offset) {
	// the x offsets of the four bg layers are staggered by intervals of 4 pixels
	case 0x00: case 0x01: case 0x02: case 0x03:
		d += 4 * offset;
		if (!flip) d = -d;
		c->bgscrollx[offset] = d;
		break;
	case 0x04: case 0x05: case 0x06: case 0x07:
		if (flip) d = -d;
		c->bgscrolly[offset - 4] = d;
		break;
	case 0x0c:
		if (!flip) d -= c->text_xoffs;
		else d += c->text_xoffs;
		c->txscrollx = -d;
		break;
	case 0x0d:
		if (!flip) d -= c->text_yoffs;
		else d += c->text_yoffs;
		c->txscrolly = -d;
		break;
	case 0x0f:
		c->pri_reg = d;
		break;
	}
}

void tc0480scp_post_load(tc0480scp_t *c)
{
	int flip = c->ctrl[0xf] & 0x40;
	c->pri_reg = c->ctrl[0xf];
	for (int i = 0; i < 4; i++) {
		int r = c->ctrl[i] + 4 * i;
		c->bgscrollx[i] = flip ? r : -r;
		r = c->ctrl[4 + i];
		c->bgscrolly[i] = flip ? -r : r;
	}
	int r = c->ctrl[0x0c];
	r = flip ? r + c->text_xoffs : r - c->text_xoffs;
	c->txscrollx = -r;
	r = c->ctrl[0x0d];
	r = flip ? r + c->text_yoffs : r - c->text_yoffs;
	c->txscrolly = -r;
}

int tc0480scp_bg_priority(const tc0480scp_t *c)
{
	static const uint16_t lookup[8] = { 0x0123, 0x1230, 0x2301, 0x3012, 0x3210, 0x2103, 0x1032, 0x0321 };
	return lookup[(c->pri_reg & 0x1c) >> 2];
}

void tc0480scp_copy(tc0480scp_t *dst, const tc0480scp_t *src)
{
	memcpy(dst->ram, src->ram, sizeof(src->ram));
	memcpy(dst->ctrl, src->ctrl, sizeof(src->ctrl));
	dst->pri_reg = src->pri_reg;
	memcpy(dst->bgscrollx, src->bgscrollx, sizeof(src->bgscrollx));
	memcpy(dst->bgscrolly, src->bgscrolly, sizeof(src->bgscrolly));
	dst->txscrollx = src->txscrollx;
	dst->txscrolly = src->txscrolly;
}

// standard tilemap row (no zoom): logical row ly, column lx, step +1 / -1
static void bg_row(const tc0480scp_t *c, const uint16_t *map, int cols, int wmask, int ly, int lx, int step,
                   uint16_t *d, uint8_t *p, uint8_t prio)
{
	const uint16_t *row = map + (ly >> 4) * cols * 2;
	int py = ly & 15;
	int x = 0;
	while (x < SW) {
		int px = lx & 15;
		int run = step > 0 ? 16 - px : px + 1;
		if (run > SW - x)
			run = SW - x;
		const uint16_t *e = row + (lx >> 4) * 2;
		uint32_t code = e[1] & 0x7fff;
		if (code >= c->ntiles)
			code %= c->ntiles;
		if (c->tile_used[code]) {
			uint16_t attr = e[0];
			int fx = (attr & 0x4000) ? 15 : 0, fy = (attr & 0x8000) ? 15 : 0;
			const uint8_t *src = c->tiles + (size_t)code * 256 + ((py ^ fy) << 4);
			uint16_t col = (uint16_t)(c->col_base + ((attr & 0xff) << 4));
			for (int i = 0; i < run; i++, px += step) {
				uint8_t pen = src[px ^ fx];
				if (pen) {
					d[x + i] = (uint16_t)(col + pen);
					p[x + i] |= prio;
				}
			}
		}
		x += run;
		lx = (lx + step * run) & wmask;
	}
}

// zoomed layers: MAME bg01_draw / bg23_draw build a scanline from the (flip oriented) pixmap
static void zoom_layer(const tc0480scp_t *c, int layer, uint16_t *bm, uint8_t *pri, int pitch, int sy0, int rows,
                       uint8_t prio, int dbl, const uint16_t *map, int cols, int wmask, int is23)
{
	int flip = c->pri_reg & 0x40;
	int W = wmask + 1;
	const uint16_t *scroll = c->ram + (dbl ? 0x4000 : 0x2000) + layer * 0x200;
	const uint16_t *rowzoom = c->ram + (dbl ? 0x5000 : 0x3000) + (layer - 2) * 0x200;
	const uint16_t *colram = c->ram + (dbl ? 0x5400 : 0x3400) + (layer - 2) * 0x200;
	uint32_t zoomx = 0x10000 - (c->ctrl[0x08 + layer] & 0xff00);
	uint32_t zoomy = 0x10000 - (uint32_t)(((c->ctrl[0x08 + layer] & 0xff) - 0x7f) * 512);
	uint32_t sx, y_index;
	if (!flip) {
		sx = ((uint32_t)(c->bgscrollx[layer] + 15 + layer * 4) << 16) + ((255 - (c->ctrl[0x10 + layer] & 0xff)) << 8);
		sx += (uint32_t)(c->x_offset - 15 - layer * 4) * zoomx;
		y_index = ((uint32_t)c->bgscrolly[layer] << 16) + ((c->ctrl[0x14 + layer] & 0xff) << 8);
		y_index -= (uint32_t)(c->y_offset - sy0) * zoomy;
	} else {
		sx = ((uint32_t)(-c->bgscrollx[layer] + 15 + layer * 4 + c->flip_xoffs) << 16) + ((255 - (c->ctrl[0x10 + layer] & 0xff)) << 8);
		sx += (uint32_t)(c->x_offset - 15 - layer * 4) * zoomx;
		y_index = ((uint32_t)(-c->bgscrolly[layer] + c->flip_yoffs) << 16) + ((c->ctrl[0x14 + layer] & 0xff) << 8);
		y_index -= (uint32_t)(c->y_offset - sy0) * zoomy;
	}
	for (int r = 0; r < rows; r++, y_index += zoomy) {
		int y = sy0 + r;
		int src_y;
		uint32_t x_index, x_step = zoomx;
		if (!is23) {
			src_y = (int)(y_index >> 16) & 0x1ff;
			int row_index = flip ? 0x1ff - src_y : src_y;
			x_index = sx - ((uint32_t)scroll[row_index] << 16) - (((uint32_t)scroll[row_index + 0x800] << 8) & 0xffff);
		} else {
			if (!flip)
				src_y = (int)((y_index >> 16) + colram[(y - c->y_offset) & 0x1ff]) & 0x1ff;
			else
				src_y = (int)((y_index >> 16) + colram[0x1ff - ((y - c->y_offset) & 0x1ff)]) & 0x1ff;
			int row_index = flip ? 0x1ff - src_y : src_y;
			int row_zoom = (c->pri_reg & (layer - 1)) ? rowzoom[row_index] : 0;
			x_index = sx - ((uint32_t)scroll[row_index] << 16) - (((uint32_t)scroll[row_index + 0x800] << 8) & 0xffff);
			x_index -= (uint32_t)(c->x_offset - 0x1f + layer * 4) * (uint32_t)((row_zoom & 0xff) << 8);
			if (row_zoom) {
				if (!(row_zoom & 0xff00))
					x_step -= (uint32_t)((row_zoom * 256) & 0xffff);
				else
					x_step -= (uint32_t)(((row_zoom & 0xff) * 256) & 0xffff);
			}
		}
		int ly = flip ? 511 - src_y : src_y;
		uint16_t *d = bm + r * pitch;
		uint8_t *p = pri + r * pitch;
		// the tile under the pixel only changes every 16 source pixels: cache its row
		const uint16_t *maprow = map + (ly >> 4) * cols * 2;
		int py = ly & 15, last_tc = -1, fx = 0;
		const uint8_t *src = NULL;
		uint16_t col = 0;
		for (int x = 0; x < SW; x++, x_index += x_step) {
			int px = (int)(x_index >> 16) & wmask;
			int lx = flip ? W - 1 - px : px;
			int tc = lx >> 4;
			if (tc != last_tc) {
				last_tc = tc;
				const uint16_t *e = maprow + tc * 2;
				uint32_t code = e[1] & 0x7fff;
				if (code >= c->ntiles)
					code %= c->ntiles;
				src = NULL;
				if (c->tile_used[code]) {
					uint16_t attr = e[0];
					fx = (attr & 0x4000) ? 15 : 0;
					int fy = (attr & 0x8000) ? 15 : 0;
					src = c->tiles + (size_t)code * 256 + ((py ^ fy) << 4);
					col = (uint16_t)(c->col_base + ((attr & 0xff) << 4));
				}
			}
			if (src) {
				uint8_t pen = src[(lx & 15) ^ fx];
				if (pen) {
					d[x] = (uint16_t)(col + pen);
					p[x] |= prio;
				}
			}
		}
	}
}

void tc0480scp_draw(const tc0480scp_t *c, int layer, uint16_t *bm, uint8_t *pri, int pitch, int sy0, int rows,
                    uint8_t priority)
{
	int dbl = (c->pri_reg & 0x80) >> 7;
	int flip = c->pri_reg & 0x40;
	int xd = -c->x_offset, yd = c->y_offset;
	if (layer == 4) {   // text layer: 64x64 8x8, 4bpp characters at RAM 0xe000
		int dx = xd - 3, dxf = (dbl ? 317 : 316) - xd, dy = yd, dyf = 256 - yd;
		const uint16_t *map = c->ram + 0x6000, *chars = c->ram + 0x7000;
		for (int r = 0; r < rows; r++) {
			int sy = sy0 + r, ly, lx, step = flip ? -1 : 1;
			if (!flip) {
				ly = (sy - dy + c->txscrolly) & 511;
				lx = (-dx + c->txscrollx) & 511;
			} else {
				ly = (SH - 1 - sy - dyf + c->txscrolly) & 511;
				lx = (SW - 1 - dxf + c->txscrollx) & 511;
			}
			const uint16_t *row = map + (ly >> 3) * 64;
			uint16_t *d = bm + r * pitch;
			uint8_t *p = pri + r * pitch;
			int x = 0;
			while (x < SW) {
				int px = lx & 7;
				int run = step > 0 ? 8 - px : px + 1;
				if (run > SW - x)
					run = SW - x;
				uint16_t attr = row[lx >> 3];
				int fx = (attr & 0x4000) ? 7 : 0, fy = (attr & 0x8000) ? 7 : 0;
				const uint16_t *ch = chars + (attr & 0xff) * 16 + (((ly & 7) ^ fy) << 1);
				if (ch[0] | ch[1]) {
					uint16_t col = (uint16_t)(c->col_base + (((attr & 0x3f00) >> 8) << 4));
					for (int i = 0; i < run; i++, px += step) {
						int b = px ^ fx;
						int pen = (ch[b >> 2] >> ((b & 3) * 4)) & 15;
						if (pen) {
							d[x + i] = (uint16_t)(col + pen);
							p[x + i] |= priority;
						}
					}
				}
				x += run;
				lx = (lx + step * run) & 511;
			}
		}
		return;
	}
	int cols = dbl ? 64 : 32;
	int wmask = cols * 16 - 1;
	const uint16_t *map = c->ram + (dbl ? 0x1000 : 0x0800) * layer;
	if (layer >= 2) {
		zoom_layer(c, layer, bm, pri, pitch, sy0, rows, priority, dbl, map, cols, wmask, 1);
		return;
	}
	if (c->ctrl[0x08 + layer] != 0x007f) {   // zoomed (zoomx or zoomy != 0x10000)
		zoom_layer(c, layer, bm, pri, pitch, sy0, rows, priority, dbl, map, cols, wmask, 0);
		return;
	}
	// no zoom: standard tilemap with 512 scroll rows (tilemap_update)
	const uint16_t *scroll = c->ram + (dbl ? 0x4000 : 0x2000) + layer * 0x200;
	int dx = xd, dxf = 320 - xd + c->flip_xoffs, dy = yd, dyf = 256 - yd + c->flip_yoffs;
	int scrolly = c->bgscrolly[layer];
	for (int r = 0; r < rows; r++) {
		int sy = sy0 + r, ly, lx;
		if (!flip) {
			ly = (sy - dy + scrolly) & 511;
			lx = (-dx + c->bgscrollx[layer] - scroll[ly]) & wmask;
		} else {
			ly = (SH - 1 - sy - dyf + scrolly) & 511;
			lx = (SW - 1 - dxf + c->bgscrollx[layer] + scroll[ly]) & wmask;
		}
		bg_row(c, map, cols, wmask, ly, lx, flip ? -1 : 1, bm + r * pitch, pri + r * pitch, priority);
	}
}
