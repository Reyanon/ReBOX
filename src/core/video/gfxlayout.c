/*  MAME gfx_layout decoding and prio_transpen (gfxlayout.h).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "gfxlayout.h"

#include <stdlib.h>
#include <string.h>

int rbx_gfx_decode(rbx_gfx *g, const rbx_gfx_layout *l, const uint8_t *rom, uint32_t size)
{
	memset(g, 0, sizeof(*g));
	uint32_t n = (uint32_t)(((uint64_t)size * 8) / l->charincrement);
	if (!n)
		return -1;
	size_t px = (size_t)l->width * l->height;
	g->pix = (uint8_t *)malloc((size_t)n * px + n);
	if (!g->pix)
		return -1;
	g->used = g->pix + (size_t)n * px;
	g->count = n;
	g->width = l->width;
	g->height = l->height;
	for (uint32_t c = 0; c < n; c++) {
		uint8_t *d = g->pix + (size_t)c * px, any = 0;
		uint64_t base = (uint64_t)c * l->charincrement;
		for (int y = 0; y < l->height; y++)
			for (int x = 0; x < l->width; x++) {
				uint8_t v = 0;
				for (int p = 0; p < l->planes; p++) {
					uint64_t b = base + l->planeoffs[p] + l->yoffs[y] + l->xoffs[x];
					if (rom[b >> 3] & (0x80 >> (b & 7)))
						v |= (uint8_t)(1 << (l->planes - 1 - p));
				}
				d[y * l->width + x] = v;
				any |= v;
			}
		g->used[c] = any != 0;
	}
	return 0;
}

void rbx_gfx_free(rbx_gfx *g)
{
	free(g->pix);
	memset(g, 0, sizeof(*g));
}

void rbx_gfx_prio_transpen(const rbx_gfx *g, uint32_t code, uint16_t color_base, int flipx, int flipy, int sx, int sy,
                           uint16_t *bm, uint8_t *pri, int pitch, int bw, int bh, uint32_t pmask)
{
	code %= g->count;
	if (!g->used[code])
		return;
	pmask |= 1u << 31;   // (MAME: the high bit of the mask is implicitly on: a pixel a sprite drew keeps the later ones out)
	const int w = g->width, h = g->height;
	int x0 = sx < 0 ? -sx : 0, x1 = sx + w > bw ? bw - sx : w;
	int y0 = sy < 0 ? -sy : 0, y1 = sy + h > bh ? bh - sy : h;
	if (x0 >= x1 || y0 >= y1)
		return;
	const uint8_t *t = g->pix + (size_t)code * w * h;
	for (int y = y0; y < y1; y++) {
		const uint8_t *row = t + (flipy ? h - 1 - y : y) * w;
		uint16_t *d = bm + (sy + y) * pitch + sx;
		uint8_t *p = pri + (sy + y) * pitch + sx;
		for (int x = x0; x < x1; x++) {
			uint8_t pen = row[flipx ? w - 1 - x : x];
			if (pen) {
				if (!((1u << (p[x] & 0x1f)) & pmask))
					d[x] = (uint16_t)(color_base + pen);
				p[x] = 31;
			}
		}
	}
}
