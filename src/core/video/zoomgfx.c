/*  Zoomed sprite drawing (zoomgfx.h), after MAME src/emu/drawgfxt.ipp drawgfxzoom_core (BSD-3-Clause,
    copyright-holders: Nicola Salmoria, Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "zoomgfx.h"

#include <stdlib.h>
#include <string.h>

int zoomgfx_decode(zoomgfx_t *g, const uint8_t *src, uint32_t size, int layout)
{
	memset(g, 0, sizeof(*g));
	g->w = 16;
	g->h = layout == ZGFX_16x8_PLANAR64 ? 8 : 16;
	uint32_t bytes = layout == ZGFX_16x8_PLANAR64 ? 64 : 128;
	g->count = src ? size / bytes : 0;
	if (!g->count)
		return -1;
	uint32_t n = (uint32_t)(g->w * g->h);
	g->pix = (uint8_t *)malloc((size_t)g->count * n);
	g->opaque = (uint8_t *)malloc(g->count);
	if (!g->pix || !g->opaque) {
		zoomgfx_free(g);
		return -1;
	}
	for (uint32_t c = 0; c < g->count; c++) {
		const uint8_t *s = src + (size_t)c * bytes;
		uint8_t *d = g->pix + (size_t)c * n, any = 0;
		for (int y = 0; y < g->h; y++)
			for (int x = 0; x < 16; x++) {
				uint8_t p;
				if (layout == ZGFX_16x8_PLANAR64) {
					p = 0;
					for (int pl = 0; pl < 4; pl++) {
						int o = y * 64 + pl * 16 + x;   // MAME bit offset, MSB first; plane 0 = pixel bit 3
						p |= (uint8_t)(((s[o >> 3] >> (7 - (o & 7))) & 1) << (3 - pl));
					}
				} else {
					uint8_t b = s[y * 8 + (x >> 1)];
					p = (x & 1) ? (b >> 4) : (b & 15);
				}
				d[y * 16 + x] = p;
				any |= p;
			}
		g->opaque[c] = any != 0;
	}
	return 0;
}

void zoomgfx_free(zoomgfx_t *g)
{
	free(g->pix);
	free(g->opaque);
	g->pix = g->opaque = NULL;
	g->count = 0;
}

void zoomgfx_draw(const zoomgfx_t *g, const zoomgfx_target *t, uint32_t code, uint32_t color, int flipx, int flipy,
                  int destx, int desty, uint32_t scalex, uint32_t scaley, int use_pri, uint32_t pmask)
{
	if (!g->count)
		return;
	code %= g->count;
	if (!g->opaque[code])
		return;
	uint32_t dstw = (scalex * (uint32_t)g->w + 0x8000) >> 16;
	uint32_t dsth = (scaley * (uint32_t)g->h + 0x8000) >> 16;
	if (dstw < 1 || dsth < 1)
		return;
	int32_t dx = (g->w << 16) / (int32_t)dstw;
	int32_t dy = (g->h << 16) / (int32_t)dsth;
	if (scalex == 0x10000 && scaley == 0x10000)   // MAME takes the unscaled path: exact steps
		dx = dy = 0x10000;
	int32_t endx = destx + (int32_t)dstw - 1;
	if (destx > t->x1 || endx < t->x0)
		return;
	int32_t srcx = 0;
	if (destx < t->x0) {
		srcx = (t->x0 - destx) * dx;
		destx = t->x0;
	}
	if (endx > t->x1)
		endx = t->x1;
	int32_t endy = desty + (int32_t)dsth - 1;
	if (desty > t->y1 || endy < t->y0)
		return;
	int32_t srcy = 0;
	if (desty < t->y0) {
		srcy = (t->y0 - desty) * dy;
		desty = t->y0;
	}
	if (endy > t->y1)
		endy = t->y1;
	if (flipx) {
		srcx = ((int32_t)dstw - 1) * dx - srcx;
		dx = -dx;
	}
	if (flipy) {
		srcy = ((int32_t)dsth - 1) * dy - srcy;
		dy = -dy;
	}
	const uint8_t *data = g->pix + (size_t)code * (size_t)(g->w * g->h);
	for (int y = desty; y <= endy; y++, srcy += dy) {
		const uint8_t *row = data + (srcy >> 16) * g->w;
		uint16_t *d = t->bm + (y - t->sy0) * t->pitch;
		uint8_t *p = t->pri ? t->pri + (y - t->sy0) * (t->ppitch ? t->ppitch : t->pitch) : NULL;
		int32_t sx = srcx;
		if (use_pri) {
			for (int x = destx; x <= endx; x++, sx += dx) {
				uint8_t pen = row[sx >> 16];
				if (pen) {
					if (!((1u << (p[x] & 0x1f)) & pmask))
						d[x] = (uint16_t)(color + pen);
					p[x] = 31;
				}
			}
		} else {
			for (int x = destx; x <= endx; x++, sx += dx) {
				uint8_t pen = row[sx >> 16];
				if (pen)
					d[x] = (uint16_t)(color + pen);
			}
		}
	}
}
