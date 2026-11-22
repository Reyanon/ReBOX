/*  Helpers for Capcom's 8-bit boards: MAME's gfx->transpen (src/emu/drawgfx.cpp, BSD-3-Clause, copyright-holders:
    Nicola Salmoria, Aaron Giles) with a transparent pen, clipped to bitmap rows, and the Z80 bus glue.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_CAPCOM_CAP8_H
#define RBX_CAPCOM_CAP8_H

#include "../../rbx.h"
#include "../../video/gfxlayout.h"

// gfx->transpen of a w x h element (code % count) with pens base + pixel, pixel `trans` left out; drawn on the
// screen rows [y0, y1) (bitmap row = y - ybase) and the columns [0, bw) (bitmap column = x - xbase)
static inline void cap8_transpen(const rbx_gfx *g, uint32_t code, uint16_t base, int fx, int fy, int sx, int sy, int trans,
                                 int y0, int y1, int ybase, int xbase, int bw)
{
	code %= g->count;
	const int w = g->width, h = g->height;
	sx -= xbase;
	if (sy >= y1 || sy + h <= y0 || sx >= bw || sx + w <= 0)
		return;
	const uint8_t *src = g->pix + code * (uint32_t)(w * h);
	for (int j = 0; j < h; j++) {
		const int yy = sy + j;
		if (yy < y0 || yy >= y1)
			continue;
		const uint8_t *s = src + (fy ? h - 1 - j : j) * w;
		uint16_t *d = rbx_video.bitmap + (yy - ybase) * rbx_video.pitch;
		for (int i = 0; i < w; i++) {
			const int xx = sx + i;
			if ((unsigned)xx >= (unsigned)bw)
				continue;
			const uint8_t p = s[fx ? w - 1 - i : i];
			if (p != trans)
				d[xx] = (uint16_t)(base + p);
		}
	}
}

// the 0x0e / 0x1f / 0x43 / 0x8f weights of a 4-bit colour PROM (2.2k, 1k, 470, 220 ohms)
static inline int cap8_res4(int v) { return 0x0e * (v & 1) + 0x1f * ((v >> 1) & 1) + 0x43 * ((v >> 2) & 1) + 0x8f * ((v >> 3) & 1); }

#endif
