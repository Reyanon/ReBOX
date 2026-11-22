/*  Namco C169 (ROZ), a line at a time. Ported from MAME 0.289 src/mame/namco/namco_c169roz.cpp (BSD-3-Clause,
    copyright-holders: Phil Stroffolino, David Haywood).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "c169roz.h"

typedef struct {
	uint32_t left, top, size;
	int color, priority, wrap;
	int32_t startx, starty, incxx, incxy, incyx, incyy;
} roz_params;

// unpack_params (System 2: the colour 4 bits)
static void unpack(const uint16_t *src, roz_params *pr)
{
	const int xoffset = 36, yoffset = 3;
	uint16_t t = src[1];
	pr->wrap = !((t >> 11) & 1);
	pr->size = 512u << ((t & 0x0300) >> 8);
	pr->color = (t & 0x000f) * 256;
	pr->priority = (t & 0x00f0) >> 4;
	t = src[2];
	pr->left = (t & 0x7000) >> 3;
	t = (t & 0x8000) ? (uint16_t)(t | 0xf000) : (uint16_t)(t & 0x0fff);
	pr->incxx = (int16_t)t;
	t = src[3];
	pr->top = (t & 0x7000) >> 3;
	t = (t & 0x8000) ? (uint16_t)(t | 0xf000) : (uint16_t)(t & 0x0fff);
	pr->incxy = (int16_t)t;
	t = src[4];
	t = (t & 0x8000) ? (uint16_t)(t | 0xf000) : (uint16_t)(t & 0x0fff);
	pr->incyx = (int16_t)t;
	t = src[5];
	t = (t & 0x8000) ? (uint16_t)(t | 0xf000) : (uint16_t)(t & 0x0fff);
	pr->incyy = (int16_t)t;
	pr->startx = (int16_t)src[6];
	pr->starty = (int16_t)src[7];
	pr->startx *= 16;
	pr->starty *= 16;
	pr->startx += xoffset * pr->incxx + yoffset * pr->incyx;
	pr->starty += xoffset * pr->incxy + yoffset * pr->incyy;
	pr->startx = (int32_t)((uint32_t)pr->startx << 8);
	pr->starty = (int32_t)((uint32_t)pr->starty << 8);
	pr->incxx = (int32_t)((uint32_t)pr->incxx << 8);
	pr->incxy = (int32_t)((uint32_t)pr->incxy << 8);
	pr->incyx = (int32_t)((uint32_t)pr->incyx << 8);
	pr->incyy = (int32_t)((uint32_t)pr->incyy << 8);
}

// draw_helper (non-FL): the line y, x0..x1-1; the start as MAME's after its rows from clip.min_y
static void helper(const c169roz_t *r, const uint16_t *ram, int which, const roz_params *pr, int y, int x0, int x1,
                   uint16_t *dest, uint8_t *pri, uint8_t prival)
{
	const uint32_t size_mask = pr->size - 1;
	uint32_t cx = (uint32_t)pr->startx + (uint32_t)x0 * (uint32_t)pr->incxx + (uint32_t)y * (uint32_t)pr->incyx;
	uint32_t cy = (uint32_t)pr->starty + (uint32_t)x0 * (uint32_t)pr->incxy + (uint32_t)y * (uint32_t)pr->incyy;
	const uint16_t cbase = (uint16_t)(pr->color + r->color_base);
	uint32_t last = 0xffffffffu;
	const uint8_t *tp = 0, *mp = 0;
	if (!pr->incxy) {   // (no vertical step along the line: its row, tile row and mask row fixed)
		const uint32_t ypos = (((cy >> 16) & size_mask) + pr->top) & 0xfff;
		const uint32_t rowbits = ((ypos >> 4) & 0xff) << 7, fy = ypos & 15;
		for (int x = x0; x < x1; x++, cx += (uint32_t)pr->incxx) {
			const uint32_t xpos = (((cx >> 16) & size_mask) + pr->left) & 0xfff;
			const uint32_t col = xpos >> 4;
			const uint32_t cell = ((col & 0x80) << 8) | rowbits | (col & 0x7f);
			if (cell != last) {
				last = cell;
				const uint16_t code = ram[cell & (r->ram_words - 1)] & 0x3fff;
				int tile = code, mask = code;
				if (r->cb)
					r->cb(code, &tile, &mask, which);
				tp = r->gfx + ((uint32_t)tile % r->count) * 256 + fy * 16;
				mp = r->mask + ((uint32_t)mask % r->mask_count) * 32 + fy * 2;
			}
			const uint32_t fx = xpos & 15;
			if (mp[fx >> 3] & (0x80 >> (fx & 7))) {
				dest[x] = (uint16_t)(tp[fx] + cbase);
				pri[x] = prival;
			}
		}
		return;
	}
	for (int x = x0; x < x1; x++, cx += (uint32_t)pr->incxx, cy += (uint32_t)pr->incxy) {
		const uint32_t xpos = (((cx >> 16) & size_mask) + pr->left) & 0xfff;
		const uint32_t ypos = (((cy >> 16) & size_mask) + pr->top) & 0xfff;
		const uint32_t col = xpos >> 4, row = ypos >> 4;
		const uint32_t cell = ((col & 0x80) << 8) | ((row & 0xff) << 7) | (col & 0x7f);
		if (cell != last) {   // (the tile of this cell: once)
			last = cell;
			const uint16_t code = ram[cell & (r->ram_words - 1)] & 0x3fff;
			int tile = code, mask = code;
			if (r->cb)
				r->cb(code, &tile, &mask, which);
			tp = r->gfx + ((uint32_t)tile % r->count) * 256;
			mp = r->mask + ((uint32_t)mask % r->mask_count) * 32;
		}
		const uint32_t fy = ypos & 15, fx = xpos & 15;
		if (mp[fy * 2 + (fx >> 3)] & (0x80 >> (fx & 7))) {
			dest[x] = (uint16_t)(tp[fy * 16 + fx] + cbase);
			pri[x] = prival;
		}
	}
}

void c169roz_draw_line(const c169roz_t *r, const uint16_t *control, const uint16_t *ram, int y, int x0, int x1,
                       uint16_t *dest, uint8_t *pri, int p, uint8_t prival)
{
	if (!r->count || !r->mask_count)
		return;
	const int special = 1;
	const int mode = control[0];   // 0x8000 or 0x1000
	for (int which = 1; which >= 0; which--) {
		const uint16_t *src = &control[which * 8];
		if (src[1] & 0x8000)
			continue;
		roz_params pr;
		if (which == special && mode == 0x8000) {   // draw_scanline: this line's parameters from the RAM
			const int offs = (y >> 3) * 0x100 + (y & 7) * 0x10 + 0xe080;
			const uint16_t *ls = &ram[(offs / 2) & (r->ram_words - 1)];
			if (ls[1] & 0x8000)
				continue;
			unpack(ls, &pr);
			if (pr.priority != p)
				continue;
			// (MAME: the clip of one line from x 0, the rows from this line: the start's y term is y)
			helper(r, ram, which, &pr, y, x0, x1, dest, pri, prival);
			continue;
		}
		unpack(src, &pr);
		if (pr.priority == p)
			helper(r, ram, which, &pr, y, x0, x1, dest, pri, prival);
	}
}

void c169roz_line_params(const c169roz_t *r, const uint16_t *control, const uint16_t *ram, int y, c169roz_layer lp[2])
{
	const int mode = control[0];
	for (int which = 1; which >= 0; which--) {
		const uint16_t *src = &control[which * 8];
		roz_params pr;
		lp[which].on = 0;
		if (src[1] & 0x8000)
			continue;
		if (which == 1 && mode == 0x8000) {
			const int offs = (y >> 3) * 0x100 + (y & 7) * 0x10 + 0xe080;
			const uint16_t *ls = &ram[(offs / 2) & (r->ram_words - 1)];
			if (ls[1] & 0x8000)
				continue;
			unpack(ls, &pr);
		} else
			unpack(src, &pr);
		lp[which].on = 1;
		lp[which].left = pr.left;
		lp[which].top = pr.top;
		lp[which].size = pr.size;
		lp[which].color = pr.color;
		lp[which].priority = pr.priority;
		lp[which].startx = pr.startx;
		lp[which].starty = pr.starty;
		lp[which].incxx = pr.incxx;
		lp[which].incxy = pr.incxy;
		lp[which].incyx = pr.incyx;
		lp[which].incyy = pr.incyy;
	}
}

void c169roz_draw_line_p(const c169roz_t *r, const c169roz_layer lp[2], const uint16_t *ram, int y, int x0, int x1,
                         uint16_t *dest, uint8_t *pri, int p, uint8_t prival)
{
	if (!r->count || !r->mask_count)
		return;
	for (int which = 1; which >= 0; which--) {
		if (!lp[which].on || lp[which].priority != p)
			continue;
		roz_params pr;
		pr.left = lp[which].left;
		pr.top = lp[which].top;
		pr.size = lp[which].size;
		pr.color = lp[which].color;
		pr.priority = lp[which].priority;
		pr.wrap = 1;
		pr.startx = lp[which].startx;
		pr.starty = lp[which].starty;
		pr.incxx = lp[which].incxx;
		pr.incxy = lp[which].incxy;
		pr.incyx = lp[which].incyx;
		pr.incyy = lp[which].incyy;
		helper(r, ram, which, &pr, y, x0, x1, dest, pri, prival);
	}
}
