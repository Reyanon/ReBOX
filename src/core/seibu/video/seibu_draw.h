/*  Drawing helpers of the small Seibu boards: MAME's drawgfx transpen / transmask / prio_transpen for one element of a
    decoded gfx set (rbx_gfx, a byte a pixel) into the visible part of an indexed bitmap, and an opaque / transparent
    tile line of a scrolled tilemap.
    After MAME 0.289 src/emu/drawgfxt.ipp and src/emu/drawgfx.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria,
    Aaron Giles) and src/emu/tilemap.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria, Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_SEIBU_DRAW_H
#define RBX_SEIBU_DRAW_H

#include <stdint.h>
#include "../../video/gfxlayout.h"

typedef struct {
	uint16_t *bm;
	int pitch;
	int x0, y0;      // the screen coordinates of the bitmap's first pixel
	int w, h;        // visible size
	uint8_t *pri;    // optional priority buffer (w x h, pitch w)
} sd_target;

// MAME bitswap<n>(v, bits[0] (the result's MSB), ..., bits[n - 1]): the boards' ROM unscrambling
static inline uint32_t sd_bitswap(uint32_t v, int n, const int *bits)
{
	uint32_t o = 0;
	for (int i = 0; i < n; i++)
		o = o << 1 | ((v >> bits[i]) & 1);
	return o;
}

static inline const uint8_t *sd_elem(const rbx_gfx *g, uint32_t code)
{
	const uint32_t n = g->count ? (code < g->count ? code : code % g->count) : 0;
	return g->pix + (size_t)n * (size_t)(g->width * g->height);
}

// one line of a tilemap of power-of-two size (cols << tshift wide, mwmask = that width - 1): the map line py (already
// wrapped), scrolled by sx, w pixels into d, tile by tile (no division per pixel: the Vita's CPU has none). Code
// ((data & 0xfff) | code_or) + code_add, pen cbase + (data >> 12) * 16 + pixel, transpen < 0: opaque; pri (optional) |= pv
static inline void sd_tile_row(uint16_t *d, uint8_t *pri, uint8_t pv, const uint16_t *ram, int cols, int tshift, int mwmask,
                               int py, int sx, const rbx_gfx *g, uint32_t code_or, uint32_t code_add, uint16_t cbase,
                               int transpen, int w)
{
	const int tsz = 1 << tshift;
	const uint16_t *rowram = ram + (py >> tshift) * cols;
	const int fy = (py & (tsz - 1)) << tshift;
	int x = 0, px = sx & mwmask;
	while (x < w) {
		const int fx = px & (tsz - 1);
		int n = tsz - fx;
		if (n > w - x)
			n = w - x;
		const uint16_t data = rowram[px >> tshift];
		const uint8_t *src = sd_elem(g, ((data & 0xfffu) | code_or) + code_add) + fy + fx;
		const uint16_t base = (uint16_t)(cbase + (data >> 12) * 16);
		for (int k = 0; k < n; k++) {
			const uint8_t c = src[k];
			if ((int)c == transpen)
				continue;
			d[x + k] = (uint16_t)(base + c);
			if (pri)
				pri[x + k] |= pv;
		}
		x += n;
		px = (px + n) & mwmask;
	}
}

// drawgfx transpen (transpen < 0: opaque); pens: color_base + pixel
static inline void sd_transpen(const sd_target *t, const rbx_gfx *g, uint32_t code, uint32_t color_base, int flipx,
                               int flipy, int sx, int sy, int transpen)
{
	const int gw = g->width, gh = g->height;
	const uint8_t *src = sd_elem(g, code);
	for (int j = 0; j < gh; j++) {
		const int r = sy + j - t->y0;
		if ((unsigned)r >= (unsigned)t->h)
			continue;
		const uint8_t *row = src + (flipy ? gh - 1 - j : j) * gw;
		uint16_t *d = t->bm + r * t->pitch;
		for (int i = 0; i < gw; i++) {
			const int px = sx + i - t->x0;
			if ((unsigned)px >= (unsigned)t->w)
				continue;
			const uint8_t c = row[flipx ? gw - 1 - i : i];
			if ((int)c == transpen)
				continue;
			d[px] = (uint16_t)(color_base + c);
		}
	}
}

// drawgfx transmask (bit n of mask: pen n transparent)
static inline void sd_transmask(const sd_target *t, const rbx_gfx *g, uint32_t code, uint32_t color_base, int flipx,
                                int flipy, int sx, int sy, uint32_t mask)
{
	const int gw = g->width, gh = g->height;
	const uint8_t *src = sd_elem(g, code);
	for (int j = 0; j < gh; j++) {
		const int r = sy + j - t->y0;
		if ((unsigned)r >= (unsigned)t->h)
			continue;
		const uint8_t *row = src + (flipy ? gh - 1 - j : j) * gw;
		uint16_t *d = t->bm + r * t->pitch;
		for (int i = 0; i < gw; i++) {
			const int px = sx + i - t->x0;
			if ((unsigned)px >= (unsigned)t->w)
				continue;
			const uint8_t c = row[flipx ? gw - 1 - i : i];
			if ((mask >> (c & 31)) & 1)
				continue;
			d[px] = (uint16_t)(color_base + c);
		}
	}
}

// drawgfx prio_transpen: drawn where (1 << pri) & (pmask | 1 << 31) is 0; pri becomes 31
static inline void sd_prio_transpen(const sd_target *t, const rbx_gfx *g, uint32_t code, uint32_t color_base, int flipx,
                                    int flipy, int sx, int sy, uint32_t pmask, int transpen)
{
	const int gw = g->width, gh = g->height;
	const uint8_t *src = sd_elem(g, code);
	pmask |= 0x80000000u;
	for (int j = 0; j < gh; j++) {
		const int r = sy + j - t->y0;
		if ((unsigned)r >= (unsigned)t->h)
			continue;
		const uint8_t *row = src + (flipy ? gh - 1 - j : j) * gw;
		uint16_t *d = t->bm + r * t->pitch;
		uint8_t *p = t->pri + r * t->w;
		for (int i = 0; i < gw; i++) {
			const int px = sx + i - t->x0;
			if ((unsigned)px >= (unsigned)t->w)
				continue;
			const uint8_t c = row[flipx ? gw - 1 - i : i];
			if ((int)c == transpen)
				continue;
			if (!((1u << (p[px] & 0x1f)) & pmask))
				d[px] = (uint16_t)(color_base + c);
			p[px] = 31;
		}
	}
}

#endif
