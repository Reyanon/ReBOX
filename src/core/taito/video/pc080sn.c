/*  Taito PC080SN tilemaps, ported from MAME src/mame/taito/pc080sn.cpp (BSD-3-Clause, copyright-holders:
    Nicola Salmoria). MAME builds two tilemaps (64x64 or 128x64 tiles, pen 0 transparent) with 512 scroll rows;
    tilemap_update sets row (line + y scroll) to x scroll - line scroll RAM. Here the same mapping is done per
    screen line while drawing: tilemap pixel = screen pixel - scrolldx + scroll (MAME tilemap_t::draw), with
    flip screen mirroring the screen coordinates and using the flipped scroll deltas.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "pc080sn.h"

#include <stdlib.h>
#include <string.h>

int pc080sn_init(pc080sn_t *c, const uint8_t *gfx, uint32_t size, int layout)
{
	memset(c, 0, sizeof(*c));
	c->ntiles = size / 32;
	c->screen_w = 320;
	c->screen_h = 256;
	c->colors = 0x80;
	if (!c->ntiles)
		return -1;
	c->tiles = (uint8_t *)malloc((size_t)c->ntiles * 64);
	c->tflags = (uint8_t *)malloc(c->ntiles);
	if (!c->tiles || !c->tflags) {
		pc080sn_exit(c);
		return -1;
	}
	for (uint32_t t = 0; t < c->ntiles; t++) {
		const uint8_t *src = gfx + t * 32;
		uint8_t *dst = c->tiles + t * 64;
		int all0 = 1, all15 = 1, any0 = 0, any15 = 0;
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				int p;
				if (layout == PC080SN_GFX_PACKED_MSB) {
					uint8_t b = src[y * 4 + (x >> 1)];
					p = (x & 1) ? (b & 15) : (b >> 4);
				} else {   // planes at bit 0, 8, 16, 24 of each 32-bit row, x = bit 0..7 (MSB first)
					p = 0;
					for (int pl = 0; pl < 4; pl++)
						p |= ((src[y * 4 + pl] >> (7 - x)) & 1) << (3 - pl);
				}
				dst[y * 8 + x] = (uint8_t)p;
				if (p != 0) all0 = 0; else any0 = 1;
				if (p != 15) all15 = 0; else any15 = 1;
			}
		c->tflags[t] = (uint8_t)(all0 | all15 << 1 | !any0 << 2 | !any15 << 3);
	}
	pc080sn_reset(c);
	return 0;
}

void pc080sn_exit(pc080sn_t *c)
{
	free(c->tiles);
	free(c->tflags);
	c->tiles = c->tflags = NULL;
}

void pc080sn_reset(pc080sn_t *c)
{
	memset(&c->r, 0, sizeof(c->r));
}

void pc080sn_xscroll_w(pc080sn_t *c, uint32_t offset, uint16_t data, uint16_t mask)
{
	uint16_t *p = &c->r.ctrl[offset & 1];
	*p = (uint16_t)((*p & ~mask) | (data & mask));
}

void pc080sn_yscroll_w(pc080sn_t *c, uint32_t offset, uint16_t data, uint16_t mask)
{
	uint16_t *p = &c->r.ctrl[2 + (offset & 1)];
	*p = (uint16_t)((*p & ~mask) | (data & mask));
}

void pc080sn_ctrl_w(pc080sn_t *c, uint32_t offset, uint16_t data, uint16_t mask)
{
	uint16_t *p = &c->r.ctrl[4 + (offset & 3)];
	*p = (uint16_t)((*p & ~mask) | (data & mask));
}

void pc080sn_draw(const pc080sn_t *c, const pc080sn_regs *s, int layer, uint16_t *bm, int pitch, uint8_t *pri,
                  int w, int y0, int h, int opaque, uint8_t prival)
{
	const int cols = c->dblwidth ? 128 : 64;
	const int wmask = cols * 8 - 1, hmask = 511;
	const int flip = s->ctrl[4] & 1;
	const uint16_t *bg = s->ram + (layer ? 0x4000 : 0);
	const uint16_t *lscroll = s->ram + (layer ? 0x6000 : 0x2000);
	// m_bgscrollx / m_bgscrolly (xscroll_word_w / yscroll_word_w)
	int scrollx = -(int)s->ctrl[layer];
	int scrolly = c->y_invert ? (int)s->ctrl[2 + layer] : -(int)s->ctrl[2 + layer];
	const int ddx = flip ? -16 - c->x_offset : -16 + c->x_offset;
	const int ddy = flip ? -c->y_offset : c->y_offset;
	const int tp = c->trans_pen[layer];
	const int tflag = tp == 15 ? 2 : 1;
	const int solid = tp == 15 ? 8 : 4;   // tile without the transparent pen: no per pixel test
	const uint32_t ntiles = c->ntiles, colors = (uint32_t)c->colors;
	const int dir = flip ? -1 : 1;
	int rowscrolly = scrolly;
	if (c->fixed[layer])
		scrolly = c->fixed_y[layer];

	for (int line = 0; line < h; line++) {
		const int y = y0 + line;
		const int sy = flip ? c->screen_h - 1 - y : y;
		const int ty = (sy - ddy + scrolly) & hmask;
		int vx;
		if (c->dblwidth) {
			vx = scrollx;
		} else {
			int j = (ty - rowscrolly) & 0x1ff;   // the line that set this row in tilemap_update
			vx = j < 256 ? scrollx - (int)(int16_t)lscroll[j] : 0;
			if (c->fixed[layer] && ty == 0)
				vx = c->fixed_x[layer];
		}
		const int sx0 = flip ? c->screen_w - 1 : 0;
		const int tx0 = sx0 - ddx + vx;
		const int trow = (ty >> 3) * cols;
		uint16_t *out = bm + line * pitch;
		uint8_t *po = pri ? pri + line * w : NULL;
		if (po && opaque)
			memset(po, prival, (size_t)w);
		int x = 0;
		while (x < w) {
			const int px = (tx0 + dir * x) & wmask;
			const int sub = px & 7;
			int n = dir > 0 ? 8 - sub : sub + 1;
			if (n > w - x)
				n = w - x;
			const int ti = trow + (px >> 3);
			uint16_t attr, code;
			if (c->dblwidth) {
				code = bg[ti + 0x2000] & 0x3fff;
				attr = bg[ti];
			} else {
				code = bg[2 * ti + 1] & 0x3fff;
				attr = bg[2 * ti];
			}
			if (code >= ntiles)
				code = (uint16_t)(code % ntiles);
			if (!opaque && (c->tflags[code] & tflag)) {
				x += n;
				continue;
			}
			const int fx = (attr >> 14) & 1, fy = (attr >> 15) & 1;
			const uint8_t *row = c->tiles + (uint32_t)code * 64 + (((ty & 7) ^ (fy ? 7 : 0)) << 3);
			uint32_t color = attr & 0x1ff;
			if (color >= colors)
				color %= colors;
			const uint16_t base = (uint16_t)(color * 16);
			int p = fx ? 7 - sub : sub;
			const int step = fx ? -dir : dir;
			if (opaque || (c->tflags[code] & solid)) {
				if (step > 0)
					for (int i = 0; i < n; i++)
						out[x + i] = (uint16_t)(base + row[p + i]);
				else
					for (int i = 0; i < n; i++)
						out[x + i] = (uint16_t)(base + row[p - i]);
				if (po && !opaque)
					for (int i = 0; i < n; i++)
						po[x + i] = prival;
			} else {
				for (int i = 0; i < n; i++, p += step) {
					const int pen = row[p];
					if (pen != tp) {
						out[x + i] = (uint16_t)(base + pen);
						if (po)
							po[x + i] = prival;
					}
				}
			}
			x += n;
		}
	}
}
