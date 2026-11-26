/*  Seta X1-001 sprite generator (Taito X System, tnzs family), ported from MAME
    src/devices/video/x1_001.cpp (BSD-3-Clause, copyright-holders: Luca Elia, David Haywood).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "x1_001.h"

#include <stdlib.h>
#include <string.h>

int x1_init(x1_001_t *x, const uint8_t *gfx, uint32_t size, int colors)
{
	memset(x, 0, sizeof(*x));
	x->spritelimit = 0x1ff;
	x->colors = colors;
	x->ntiles = size / 128;
	x->tiles = (uint8_t *)malloc((size_t)x->ntiles * 256);
	x->opaque = (uint8_t *)malloc(x->ntiles);
	if (!x->tiles || !x->opaque || !x->ntiles)
		return -1;
	// planes { 0, 8, 16, 24 } (first = most significant), x: 8 + 8 at +32 bytes, y: 8 rows of 4 bytes, + 8 at +64 bytes
	for (uint32_t t = 0; t < x->ntiles; t++) {
		int set = 0;
		for (int py = 0; py < 16; py++)
			for (int px = 0; px < 16; px++) {
				const uint8_t *b = gfx + t * 128 + (px >> 3) * 32 + (py >> 3) * 64 + (py & 7) * 4;
				int bit = 7 - (px & 7);
				uint8_t v = (uint8_t)(((b[0] >> bit) & 1) << 3 | ((b[1] >> bit) & 1) << 2 | ((b[2] >> bit) & 1) << 1 | ((b[3] >> bit) & 1));
				x->tiles[t * 256 + py * 16 + px] = v;
				set += v != 0;
			}
		x->opaque[t] = set == 0 ? 0 : set == 256 ? 2 : 1;
	}
	x1_reset(x);
	return 0;
}

int x1_init_layout(x1_001_t *x, const uint8_t *gfx, uint32_t size, int colors, const x1_layout *l)
{
	memset(x, 0, sizeof(*x));
	x->spritelimit = 0x1ff;
	x->colors = colors;
	x->ntiles = l->total;
	x->tiles = (uint8_t *)malloc((size_t)x->ntiles * 256);
	x->opaque = (uint8_t *)malloc(x->ntiles ? x->ntiles : 1);
	if (!x->tiles || !x->opaque || !x->ntiles)
		return -1;
	const uint64_t bits = (uint64_t)size * 8;
	for (uint32_t t = 0; t < x->ntiles; t++) {
		int set = 0;
		const uint64_t base = (uint64_t)t * l->charincrement;
		for (int py = 0; py < 16; py++)
			for (int px = 0; px < 16; px++) {
				uint8_t v = 0;
				for (int p = 0; p < 4; p++) {
					uint64_t b = base + l->planes[p] + l->xoffs[px] + l->yoffs[py];
					v = (uint8_t)(v << 1);
					if (b < bits && (gfx[b >> 3] & (0x80 >> (b & 7))))
						v |= 1;
				}
				x->tiles[t * 256 + py * 16 + px] = v;
				set += v != 0;
			}
		x->opaque[t] = set == 0 ? 0 : set == 256 ? 2 : 1;
	}
	x1_reset(x);
	return 0;
}

void x1_exit(x1_001_t *x)
{
	free(x->tiles);
	free(x->opaque);
	x->tiles = x->opaque = NULL;
}

void x1_reset(x1_001_t *x)
{
	// MAME device_start: the RAM starts as 0xff (see the notes there on chukatai / jjsquawk)
	memset(x->ctrl, 0xff, sizeof(x->ctrl));
	memset(x->ylow, 0xff, sizeof(x->ylow));
	memset(x->code, 0xff, sizeof(x->code));
	x->bgflag = 0;
}

typedef struct { uint16_t *bm; int pitch, w, h, x0, y0; } target;

static void tile(x1_001_t *x, const target *t, uint32_t code, int color, int flipx, int flipy, int sx, int sy, int transpen)
{
	sx -= t->x0;
	sy -= t->y0;
	if (sx <= -16 || sy <= -16 || sx >= t->w || sy >= t->h)
		return;
	code %= x->ntiles;   // (only for tiles on screen: tile4 draws each one four times)
	if (transpen >= 0 && !x->opaque[code])
		return;
	const uint8_t *src = x->tiles + code * 256;
	const uint16_t base = (uint16_t)((color + x->gfx_colorbase) * 16);
	// the visible part of the 16x16 tile
	const int y0 = sy < 0 ? -sy : 0, y1 = sy + 16 > t->h ? t->h - sy : 16;
	const int x0 = sx < 0 ? -sx : 0, x1 = sx + 16 > t->w ? t->w - sx : 16;
	for (int yy = y0; yy < y1; yy++) {
		const uint8_t *row = src + (flipy ? 15 - yy : yy) * 16;
		uint16_t *d = t->bm + (sy + yy) * t->pitch + sx;
		if (flipx) {
			for (int xx = x0; xx < x1; xx++) {
				const uint8_t c = row[15 - xx];
				if (c != transpen)
					d[xx] = (uint16_t)(base + c);
			}
		} else {
			for (int xx = x0; xx < x1; xx++) {
				const uint8_t c = row[xx];
				if (c != transpen)
					d[xx] = (uint16_t)(base + c);
			}
		}
	}
}

static void tile4(x1_001_t *x, const target *t, uint32_t code, int color, int flipx, int flipy, int sx, int sy, int transpen)
{
	// MAME draws each tile four times for the x / y wrap
	tile(x, t, code, color, flipx, flipy, sx, sy, transpen);
	tile(x, t, code, color, flipx, flipy, sx - 512, sy, transpen);
	tile(x, t, code, color, flipx, flipy, sx, sy - 256, transpen);
	tile(x, t, code, color, flipx, flipy, sx - 512, sy - 256, transpen);
}

static void draw_background(x1_001_t *x, const target *t, int bank_size)
{
	int ctrl = x->ctrl[0], ctrl2 = x->ctrl[1];
	int flip = (ctrl >> 6) & 1;
	int numcol = ctrl2 & 0x0f;
	const uint8_t *scrollram = &x->ylow[0x200];
	int bank = ((ctrl2 ^ (~ctrl2 << 1)) & 0x40) ? bank_size : 0;
	const int max_y = 0xf0;
	int startcol = 0;
	if (ctrl & 0x01) startcol += 0x4;
	if (ctrl & 0x02) startcol += 0x8;
	int xoffs = flip ? x->bg_flipxoffs : x->bg_noflipxoffs;
	int yoffs = flip ? x->bg_flipyoffs : x->bg_noflipyoffs;
	int transpen = (x->bgflag & 0x80) ? -1 : x->transpen;
	if (numcol == 1)
		numcol = 16;
	uint32_t upper = x->ctrl[2] + x->ctrl[3] * 256;
	for (int col = 0; col < numcol; col++) {
		int scrollx = scrollram[col * 0x10 + 4];
		int scrolly = scrollram[col * 0x10];
		for (int offs = 0; offs < 0x20; offs++) {
			int i = ((col + startcol) & 0xf) * 32 + offs;
			uint16_t code = x->code[(i + 0x400 + bank) & 0x1fff];
			uint16_t color = x->code[(i + 0x600 + bank) & 0x1fff];
			int flipx = code & 0x8000, flipy = code & 0x4000;
			int sx = scrollx + xoffs + (offs & 1) * 16;
			int sy = -(scrolly + yoffs) + (offs / 2) * 16;
			if (upper & (1u << col))
				sx -= 256;
			if (flip) {
				sy = max_y - sy;
				flipx = !flipx;
				flipy = !flipy;
			}
			int c = color >> (16 - 5);
			if (c >= x->colors)   // (wrap without a divide in the usual case)
				c %= x->colors;
			tile4(x, t, code & 0x3fff, c, flipx, flipy, sx & 0x1ff, sy & 0x0ff, transpen);
		}
	}
}

static void draw_foreground(x1_001_t *x, const target *t, int screen_h, int vis_max_y, int bank_size)
{
	int screenflip = (x->ctrl[0] & 0x40) >> 6;
	int ctrl2 = x->ctrl[1];
	const uint16_t *chr = &x->code[0x0000], *xp = &x->code[0x0200];
	int xoffs = screenflip ? x->fg_flipxoffs : x->fg_noflipxoffs;
	int yoffs = screenflip ? x->fg_flipyoffs : x->fg_noflipyoffs;
	if ((ctrl2 ^ (~ctrl2 << 1)) & 0x40) {
		chr += bank_size;
		xp += bank_size;
	}
	int max_y = screen_h;
	for (int i = x->spritelimit; i >= 0; i--) {
		uint32_t code = chr[i] & 0x3fff;
		int color = (xp[i] & 0xf800) >> 11;
		if (x->setac_gfxbank)   // (setac_gfxbank_callback, given x >> 8 as the color)
			code += (uint32_t)(((xp[i] >> 8) & 0x06) >> 1) * 0x4000;
		int sx = (xp[i] & 0x00ff) - (xp[i] & 0x0100);
		int sy = x->ylow[i] & 0xff;
		int flipx = chr[i] & 0x8000, flipy = chr[i] & 0x4000;
		color = (color < x->colors ? color : color % x->colors) + x->colorbase;
		if (screenflip) {
			sy = max_y - sy + (screen_h - (vis_max_y + 1));
			flipx = !flipx;
			flipy = !flipy;
		}
		tile4(x, t, code, color, flipx, flipy, (sx + xoffs) & 0x1ff, max_y - ((sy + yoffs) & 0x0ff), x->transpen);
	}
}

void x1_draw(x1_001_t *x, uint16_t *bitmap, int pitch, int w, int h, int x0, int y0, int screen_h, int vis_max_y, int bank_size)
{
	target t = { bitmap, pitch, w, h, x0, y0 };
	draw_background(x, &t, bank_size);
	draw_foreground(x, &t, screen_h, vis_max_y, bank_size);
}

void x1_setac_eof(x1_001_t *x)
{
	int ctrl2 = x->ctrl[1];
	if (~ctrl2 & 0x20) {
		if (ctrl2 & 0x40)
			memcpy(&x->code[0x0000], &x->code[0x1000], 0x800 * 2);
		else
			memcpy(&x->code[0x1000], &x->code[0x0000], 0x800 * 2);
	}
}

void x1_tnzs_eof(x1_001_t *x)
{
	int ctrl2 = x->ctrl[1];
	if (~ctrl2 & 0x20) {
		if (ctrl2 & 0x40)
			memcpy(&x->code[0x0000], &x->code[0x0800], 0x400 * 2);
		else
			memcpy(&x->code[0x0800], &x->code[0x0000], 0x400 * 2);
		memcpy(&x->code[0x0400], &x->code[0x0c00], 0x400 * 2);
	}
}
