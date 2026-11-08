/*  Data East "Karnov style" sprites, after MAME src/mame/dataeast/deckarn.cpp (BSD-3-Clause, copyright-holders:
    Bryan McPhail). See deckarn.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "deckarn.h"

#include <stddef.h>

// a transparent blit of an element (pen 0 clear), clipped to the bitmap; with pri: MAME prio_transpen (drawn where
// 1 << pri is not in pmask, bit 31 implied; pri = 31 under every opaque pixel)
static void blit(const rbx_gfx *g, uint32_t code, int base, int fx, int fy, int sx, int sy, uint16_t *bm, uint8_t *pri,
                 uint32_t pmask, int pitch, int bw, int bh)
{
	if (!g->count)
		return;
	code %= g->count;
	const int w = g->width, h = g->height;
	const uint8_t *src = g->pix + (size_t)code * (size_t)(w * h);
	for (int y = 0; y < h; y++) {
		const int dy = sy + y;
		if (dy < 0 || dy >= bh)
			continue;
		const uint8_t *row = src + (fy ? h - 1 - y : y) * w;
		uint16_t *d = bm + dy * pitch;
		for (int x = 0; x < w; x++) {
			const int dx = sx + x;
			if (dx < 0 || dx >= bw)
				continue;
			const uint8_t p = row[fx ? w - 1 - x : x];
			if (!p)
				continue;
			if (pri) {
				uint8_t *pr = &pri[dy * bw + dx];
				if (!((1u << (*pr & 0x1f)) & (pmask | 1u << 31)))
					d[dx] = (uint16_t)(base + p);
				*pr = 31;
			} else
				d[dx] = (uint16_t)(base + p);
		}
	}
}

void deckarn_draw_pri(const rbx_gfx *gfx, int color_base, const uint16_t *sr, int size, int flip, uint16_t *bm,
                      uint8_t *pri, void (*colpri)(uint32_t *colour, uint32_t *pri_mask), int pitch, int bw, int bh,
                      int top)
{
	const int prio = colpri != NULL;   // (with the callback: drawn from the list's end)
	for (int offs = prio ? size - 4 : 0; prio ? offs >= 0 : offs < size; offs += prio ? -4 : 4) {
		const uint16_t data0 = sr[offs];
		if (!(data0 & 0x8000))
			continue;
		int y = data0 & 0x1ff;
		const uint16_t data3 = sr[offs + 3];
		uint32_t colour = data3 >> 12, pmask = 0;
		if (prio)
			colpri(&colour, &pmask);
		uint32_t sprite = data3 & 0xfff;
		int x = sr[offs + 2] & 0x1ff;
		const uint16_t data1 = sr[offs + 1];
		if (!(data1 & 1))
			continue;
		const int extra = (data0 >> 11) & 1;
		int fy = (data1 >> 1) & 1, fx = (data1 >> 2) & 1;
		if (extra) {
			y += 16;
			sprite &= 0xffe;   // (taken from the 8-bit version)
		}
		x = (x + 16) & 0x1ff;
		y = (y + 16) & 0x1ff;
		x = 256 - x;
		y = 256 - y;
		if (flip) {
			y = 240 - y;
			x = 240 - x;
			fx = !fx;
			fy = !fy;
			if (extra)
				y -= 16;
		}
		uint32_t sprite2;
		if (extra && fy) {
			sprite2 = sprite;
			sprite |= 1;
		} else
			sprite2 = sprite | 1;
		const int base = color_base + (int)colour * 16;
		blit(gfx, sprite, base, fx, fy, x, y - top, bm, prio ? pri : NULL, pmask, pitch, bw, bh);
		if (extra)
			blit(gfx, sprite2, base, fx, fy, x, y + 16 - top, bm, prio ? pri : NULL, pmask, pitch, bw, bh);
	}
}

void deckarn_draw(const rbx_gfx *gfx, int color_base, const uint16_t *sr, int size, int flip, uint16_t *bm, int pitch,
                  int bw, int bh, int top)
{
	deckarn_draw_pri(gfx, color_base, sr, size, flip, bm, NULL, NULL, pitch, bw, bh, top);
}
