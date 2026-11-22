/*  Video System boards' drawing pieces (vsystem_gfx.h). From MAME src/emu/digfx.cpp, drawgfx.cpp, drawgfxt.ipp and
    tilemap.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria, Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "vsystem_gfx.h"

#include <stdlib.h>
#include <string.h>

static int vsg_alloc(vsg_t *g, uint32_t count, int w, int h, uint32_t colorbase, uint32_t colors)
{
	memset(g, 0, sizeof(*g));
	if (!count)
		return -1;
	const size_t px = (size_t)w * h;
	g->pix = (uint8_t *)malloc((size_t)count * px + count);
	if (!g->pix)
		return -1;
	g->use = g->pix + (size_t)count * px;
	g->count = count;
	g->w = w;
	g->h = h;
	g->colorbase = colorbase;
	g->colors = colors ? colors : 1;
	return 0;
}

static void vsg_usage(vsg_t *g)
{
	const size_t px = (size_t)g->w * g->h;
	for (uint32_t c = 0; c < g->count; c++) {
		const uint8_t *p = g->pix + c * px;
		uint8_t u = 0;
		for (size_t i = 0; i < px && u != 3; i++)
			u |= p[i] == 15 ? 2 : 1;
		g->use[c] = u;
	}
}

int vsg_packed(vsg_t *g, const uint8_t *rom, uint32_t size, int w, int msb, uint32_t colorbase, uint32_t colors)
{
	const uint32_t bytes = (uint32_t)(w * w / 2);
	if (vsg_alloc(g, size / bytes, w, w, colorbase, colors))
		return -1;
	const uint32_t n = g->count * bytes;
	uint8_t *d = g->pix;
	for (uint32_t i = 0; i < n; i++) {
		const uint8_t v = rom[i];
		*d++ = msb ? (uint8_t)(v >> 4) : (uint8_t)(v & 15);
		*d++ = msb ? (uint8_t)(v & 15) : (uint8_t)(v >> 4);
	}
	vsg_usage(g);
	return 0;
}

int vsg_decode(vsg_t *g, const vsg_layout *l, const uint8_t *rom, uint32_t size, uint32_t start, uint32_t total,
               uint32_t colorbase, uint32_t colors)
{
	if (vsg_alloc(g, total, l->w, l->h, colorbase, colors))
		return -1;
	const uint64_t bits = (uint64_t)size * 8;
	uint8_t *d = g->pix;
	for (uint32_t c = 0; c < total; c++) {
		const uint64_t base = (uint64_t)start * 8 + (uint64_t)c * l->charinc;
		for (int y = 0; y < l->h; y++)
			for (int x = 0; x < l->w; x++) {
				uint8_t v = 0;
				for (int p = 0; p < l->planes; p++) {
					const uint64_t b = base + l->planeoffs[p] + l->yoffs[y] + l->xoffs[x];
					if (b < bits && (rom[b >> 3] & (0x80 >> (b & 7))))
						v |= (uint8_t)(1 << (l->planes - 1 - p));
				}
				*d++ = v;
			}
	}
	vsg_usage(g);
	return 0;
}

void vsg_free(vsg_t *g)
{
	free(g->pix);
	memset(g, 0, sizeof(*g));
}

void vsg_zoom(const vsg_t *g, const vsg_target *t, uint32_t code, uint32_t color, int flipx, int flipy, int sx, int sy,
              uint32_t scalex, uint32_t scaley, int use_pri, uint32_t pmask)
{
	if (!g->count)
		return;
	code %= g->count;
	if (!(g->use[code] & 1))
		return;   // (only the transparent pen)
	const uint16_t col = (uint16_t)(g->colorbase + 16 * (color % g->colors));
	pmask |= 1u << 31;
	const int w = g->w, h = g->h;
	const int32_t dstw = (int32_t)((scalex * (uint32_t)w + 0x8000) >> 16), dsth = (int32_t)((scaley * (uint32_t)h + 0x8000) >> 16);
	if (dstw < 1 || dsth < 1)
		return;
	int32_t dx = (w << 16) / dstw, dy = (h << 16) / dsth;
	int32_t ex = sx + dstw - 1;
	if (sx > t->cx1 || ex < t->cx0)
		return;
	int32_t srcx = 0;
	if (sx < t->cx0) {
		srcx = (t->cx0 - sx) * dx;
		sx = t->cx0;
	}
	if (ex > t->cx1)
		ex = t->cx1;
	int32_t ey = sy + dsth - 1;
	if (sy > t->cy1 || ey < t->cy0)
		return;
	int32_t srcy = 0;
	if (sy < t->cy0) {
		srcy = (t->cy0 - sy) * dy;
		sy = t->cy0;
	}
	if (ey > t->cy1)
		ey = t->cy1;
	if (flipx) {
		srcx = (dstw - 1) * dx - srcx;
		dx = -dx;
	}
	if (flipy) {
		srcy = (dsth - 1) * dy - srcy;
		dy = -dy;
	}
	const uint8_t *src = g->pix + (size_t)code * (size_t)(w * h);
	const int n = ex - sx + 1;
	for (int y = sy; y <= ey; y++, srcy += dy) {
		const uint8_t *row = src + (srcy >> 16) * w;
		uint16_t *d = t->bm + (y - t->y0) * t->pitch + (sx - t->x0);
		int32_t cx = srcx;
		if (use_pri) {
			uint8_t *p = t->pri + (y - t->y0) * t->pitch + (sx - t->x0);
			for (int i = 0; i < n; i++, cx += dx) {
				const uint8_t pen = row[cx >> 16];
				if (pen != 15) {
					if (!((1u << (p[i] & 0x1f)) & pmask))
						d[i] = (uint16_t)(col + pen);
					p[i] = 31;
				}
			}
		} else {
			for (int i = 0; i < n; i++, cx += dx) {
				const uint8_t pen = row[cx >> 16];
				if (pen != 15)
					d[i] = (uint16_t)(col + pen);
			}
		}
	}
}

void vsg_layer_line(const vsg_layer *l, int y, int x0, int x1, uint16_t *bm, uint8_t *pri)
{
	const vsg_t *g = l->gfx;
	if (!g->count)
		return;
	const int wmask = l->cols * 8 - 1, hmask = l->rows * 8 - 1;
	const int my = ((l->flip & 2) ? l->yextent - 1 - y + l->colscroll : y + l->colscroll) & hmask;
	const int32_t rs = l->rowscroll[l->scrollrows <= 1 ? 0 : my / ((hmask + 1) / l->scrollrows)];
	const uint16_t *vrow = l->vram + (my >> 3) * l->cols;
	const int py = my & 7;
	const int flipx = l->flip & 1;
	int mx = (flipx ? l->xextent - 1 - x0 - l->dx_flipped + rs : x0 - l->dx + rs) & wmask;
	const uint8_t prio = l->priority;
	for (int x = x0; x <= x1;) {
		const uint16_t e = vrow[mx >> 3];
		uint32_t tile = (uint32_t)(e & l->code_mask) | l->bank[(e >> l->bank_shift) & l->bank_mask];
		if (tile >= g->count)
			tile %= g->count;
		const uint16_t col = (uint16_t)(g->colorbase + 16 * ((uint32_t)((e >> l->color_shift) + l->color_add) % g->colors));
		const uint8_t *src = g->pix + tile * 64 + py * 8;
		const int px = mx & 7;
		int n = flipx ? px + 1 : 8 - px;
		if (n > x1 - x + 1)
			n = x1 - x + 1;
		uint16_t *d = bm + (x - x0);
		uint8_t *p = pri + (x - x0);
		const uint8_t u = g->use[tile];
		if (l->opaque || !(u & 2)) {
			if (flipx)
				for (int i = 0; i < n; i++) {
					d[i] = (uint16_t)(col + src[px - i]);
					p[i] |= prio;
				}
			else
				for (int i = 0; i < n; i++) {
					d[i] = (uint16_t)(col + src[px + i]);
					p[i] |= prio;
				}
		} else if (u & 1) {
			for (int i = 0; i < n; i++) {
				const uint8_t pen = src[flipx ? px - i : px + i];
				if (pen != 15) {
					d[i] = (uint16_t)(col + pen);
					p[i] |= prio;
				}
			}
		}
		x += n;
		mx = (flipx ? mx - n : mx + n) & wmask;
	}
}
