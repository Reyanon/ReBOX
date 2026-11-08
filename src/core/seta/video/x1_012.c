/*  Seta (or Allumer) X1-012 (+ X1-011) tile layer generator (x1_012.h). Ported from MAME src/mame/seta/x1_012.cpp
    (BSD-3-Clause, copyright-holders: Luca Elia); the scroll arithmetic of the flipped layer from MAME src/emu/tilemap.cpp
    (BSD-3-Clause, copyright-holders: Nicola Salmoria, Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "x1_012.h"
#include "../../rbx.h"

#include <stdlib.h>
#include <string.h>

// the masks of the drawing, from the decoded pixels
static int masks(x1_012_gfx *g)
{
	g->rows = (uint16_t *)malloc((size_t)g->count * 4);
	if (!g->rows)
		return -1;
	g->full = g->rows + g->count;
	for (uint32_t t = 0; t < g->count; t++) {
		const uint8_t *p = g->pix + (size_t)t * 256;
		uint16_t rows = 0, full = 0;
		for (int y = 0; y < 16; y++, p += 16) {
			int any = 0, all = 1;
			for (int x = 0; x < 16; x++) {
				any |= p[x];
				all &= p[x] != 0;
			}
			rows |= (uint16_t)((any != 0) << y);
			full |= (uint16_t)(all << y);
		}
		g->rows[t] = rows;
		g->full[t] = full;
	}
	return 0;
}

int x1_012_decode(x1_012_gfx *g, const rbx_gfx_layout *l, const uint8_t *rom, uint32_t size, int base, int gran)
{
	rbx_gfx d;
	memset(g, 0, sizeof(*g));
	if (rbx_gfx_decode(&d, l, rom, size))
		return -1;
	g->pix = d.pix;
	g->count = d.count;
	g->base = (uint16_t)base;
	g->gran = (uint16_t)gran;
	return masks(g);
}

int x1_012_decode_packed(x1_012_gfx *g, int planes, const uint8_t *rom, uint32_t size, int base, int gran)
{
	const uint32_t group = (uint32_t)planes / 2, group8 = group * 8, tile = group * 8 * 8;
	memset(g, 0, sizeof(*g));
	g->count = size / tile;
	g->base = (uint16_t)base;
	g->gran = (uint16_t)gran;
	if (!g->count || !(g->pix = (uint8_t *)malloc((size_t)g->count * 256)))
		return -1;
	// a byte of a group: 2 planes of 4 pixels, first pixel in the high bits (one byte of the result per pixel)
	uint32_t lut[256];
	for (int b = 0; b < 256; b++) {
		uint32_t v = 0;
		for (int k = 0; k < 4; k++)
			v |= (uint32_t)((((b >> (7 - k)) & 1) << 1) | ((b >> (3 - k)) & 1)) << (k * 8);
		lut[b] = v;
	}
	for (uint32_t t = 0; t < g->count; t++) {
		const uint8_t *src = rom + t * tile;
		uint8_t *d = g->pix + (size_t)t * 256;
		for (int y = 0; y < 16; y++, d += 16) {
			const uint8_t *row = src + (y >= 8 ? group8 * 4 : 0) + (uint32_t)(y & 7) * group;
			for (int i = 0; i < 4; i++) {
				const uint8_t *p = row + (uint32_t)(3 - i) * group8;
				uint32_t v = 0;
				for (uint32_t k = 0; k < group; k++)
					v = v << 2 | lut[p[k]];
				memcpy(d + i * 4, &v, 4);   // (little-endian hosts)
			}
		}
	}
	return masks(g);
}

void x1_012_share(x1_012_gfx *g, const x1_012_gfx *from, int base)
{
	*g = *from;
	g->base = (uint16_t)base;
}

void x1_012_free(x1_012_t *l)
{
	if (l->ngfx > 0) {
		free(l->gfx[0].pix);
		free(l->gfx[0].rows);
	}
	memset(l->gfx, 0, sizeof(l->gfx));
	l->ngfx = 0;
}

void x1_012_capture(x1_012_t *l)
{
	memcpy(l->snap, l->vram + ((l->vctrl[2] & 8) ? 0x1000 : 0), sizeof(l->snap));   // (only the bank on show)
	memcpy(l->snap_ctrl, l->vctrl, sizeof(l->snap_ctrl));
}

// one line of a tile: every pen
static inline void line_opaque(uint16_t *d, const uint8_t *s, int n, int base)
{
	for (int i = 0; i < n; i++)
		d[i] = (uint16_t)(base + s[i]);
}
static inline void line_opaque_r(uint16_t *d, const uint8_t *s, int n, int base)
{
	for (int i = 0; i < n; i++)
		d[i] = (uint16_t)(base + s[-i]);
}
// pen 0 left out
static inline void line_trans(uint16_t *d, const uint8_t *s, int n, int base)
{
	for (int i = 0; i < n; i++)
		if (s[i])
			d[i] = (uint16_t)(base + s[i]);
}
static inline void line_trans_r(uint16_t *d, const uint8_t *s, int n, int base)
{
	for (int i = 0; i < n; i++)
		if (s[-i])
			d[i] = (uint16_t)(base + s[-i]);
}

// the layer's first tile: the pen of screen (0, y0) is at (ox, oy) in MAME's pixmap
static void origin(const x1_012_t *l, int y0, int vis_h, int flip, int *ox, int *oy)
{
	int x = l->snap_ctrl[0], y = l->snap_ctrl[1];
	x += 0x10 - l->xoffs[flip ? 1 : 0];   // (x1_012_device::update_scroll)
	y -= (256 - vis_h) / 2;
	if (flip)
		y = 256 + vis_h - y;              // (tilemap_t::effective_colscroll, a 512 x 256 screen bitmap)
	*ox = x & 1023;
	*oy = (y + y0) & 511;
}

void x1_012_draw(const x1_012_t *l, uint16_t *bm, int pitch, int w, int h, int y0, int vis_h, int flip, int opaque)
{
	int mode = (l->snap_ctrl[2] >> 4) & 1;
	if (mode >= l->ngfx)
		mode = 0;
	const x1_012_gfx *g = &l->gfx[mode];
	int ox, oy;
	origin(l, y0, vis_h, flip, &ox, &oy);
	const int fx0 = ox & 15, fy0 = oy & 15;
	int row = oy >> 4;
	for (int ty = -fy0; ty < h; ty += 16, row = (row + 1) & 31) {
		const int ya = ty < 0 ? -ty : 0, yb = ty + 16 > h ? h - ty : 16;
		const int mrow = flip ? 31 - row : row;
		const uint16_t *codes = l->snap + mrow * 64, *attrs = codes + 0x800;
		uint16_t *line = bm + (ty + ya) * pitch;
		int col = ox >> 4;
		for (int tx = -fx0; tx < w; tx += 16, col = (col + 1) & 63) {
			const int mcol = flip ? 63 - col : col;
			const int word = codes[mcol];
			const uint32_t code = rbx_wrap(word & 0x3fffu, g->count);
			const uint32_t rows = g->rows[code];
			if (!opaque && !rows)
				continue;
			const uint32_t full = g->full[code];
			const int base = g->base + g->gran * (attrs[mcol] & 0x1f);
			const int flipx = ((word >> 15) & 1) ^ (flip != 0), flipy = ((word >> 14) & 1) ^ (flip != 0);
			const int xa = tx < 0 ? -tx : 0, xb = tx + 16 > w ? w - tx : 16, n = xb - xa;
			const uint8_t *src = g->pix + (size_t)code * 256;
			uint16_t *d = line + tx + xa;
			for (int y = ya; y < yb; y++, d += pitch) {
				const int sy = flipy ? 15 - y : y;
				const uint8_t *s = src + sy * 16;
				if (opaque || (full >> sy) & 1) {
					if (flipx)
						line_opaque_r(d, s + 15 - xa, n, base);
					else
						line_opaque(d, s + xa, n, base);
				} else if ((rows >> sy) & 1) {
					if (flipx)
						line_trans_r(d, s + 15 - xa, n, base);
					else
						line_trans(d, s + xa, n, base);
				}
			}
		}
	}
}

void x1_012_draw_effect(const x1_012_t *l, uint16_t *bm, int pitch, int w, int h, int y0, int vis_h, int flip, int add_pens)
{
	int mode = (l->snap_ctrl[2] >> 4) & 1;
	if (mode >= l->ngfx)
		mode = 0;
	const x1_012_gfx *g = &l->gfx[mode];
	const int effect_mask = g->base + 31 * g->gran;
	int ox, oy;
	origin(l, y0, vis_h, flip, &ox, &oy);
	const int fx0 = ox & 15, fy0 = oy & 15;
	int row = oy >> 4;
	for (int ty = -fy0; ty < h; ty += 16, row = (row + 1) & 31) {
		const int ya = ty < 0 ? -ty : 0, yb = ty + 16 > h ? h - ty : 16;
		const int mrow = flip ? 31 - row : row;
		const uint16_t *codes = l->snap + mrow * 64, *attrs = codes + 0x800;
		uint16_t *line = bm + (ty + ya) * pitch;
		int col = ox >> 4;
		for (int tx = -fx0; tx < w; tx += 16, col = (col + 1) & 63) {
			const int mcol = flip ? 63 - col : col;
			const int word = codes[mcol];
			const uint32_t code = rbx_wrap(word & 0x3fffu, g->count);
			if (!g->rows[code])
				continue;
			const int base = g->base + g->gran * (attrs[mcol] & 0x1f);
			const int effect = (base & effect_mask) == effect_mask;
			const int flipx = ((word >> 15) & 1) ^ (flip != 0), flipy = ((word >> 14) & 1) ^ (flip != 0);
			const int xa = tx < 0 ? -tx : 0, xb = tx + 16 > w ? w - tx : 16, n = xb - xa;
			const uint8_t *src = g->pix + (size_t)code * 256;
			uint16_t *d = line + tx + xa;
			for (int y = ya; y < yb; y++, d += pitch) {
				const uint8_t *s = src + (flipy ? 15 - y : y) * 16 + (flipx ? 15 - xa : xa);
				const int step = flipx ? -1 : 1;
				for (int i = 0; i < n; i++, s += step)
					if (*s)
						d[i] = (uint16_t)(effect ? add_pens + d[i] : base + *s);
			}
		}
	}
}
