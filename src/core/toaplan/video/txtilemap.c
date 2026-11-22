/*  Toaplan text tilemap. Ported from MAME src/mame/toaplan/toaplan_txtilemap.cpp (BSD-3-Clause, copyright-holders:
    Quench, David Haywood).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "txtilemap.h"

void txtilemap_draw(const txtilemap_t *t, uint16_t *bm, int pitch, int w, int y0, int y1)
{
	const int flipx = !(t->lineselect[0] & 0x8000);   // (MAME: the whole layer from line 0's bit)
	for (int y = y0; y < y1; y++) {
		// set_scrolly(lineselect[y] - y): the pixmap's line
		const int ly = (t->lineselect[y] - t->dy) & 0xff;
		const int rs = t->linescroll[ly];
		uint16_t *dst = bm + y * pitch;
		for (int x = 0; x < w; x++) {
			int lx;
			if (!flipx)
				lx = (x - t->dx + rs) & 0x1ff;
			else
				lx = 0x1ff - ((x - (t->screen_w - 0x200 - (t->dx_flipped - rs))) & 0x1ff);
			const uint16_t attr = t->vram[(ly >> 3) * 64 + (lx >> 3)];
			const uint8_t pen = t->pix(t->user, attr & 0x3ff, lx & 7, ly & 7);
			if (pen)
				dst[x] = (uint16_t)(t->color_base + ((attr >> 10) & 0x3f) * 16 + pen);
		}
	}
}
