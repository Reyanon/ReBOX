/*  Tecmo's sprite chip (the Ninja Gaiden format). Ported from MAME src/mame/shared/tecmo_spr.cpp (BSD-3-Clause,
    copyright-holders: David Haywood).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "tecmo_spr.h"

#include <string.h>

void tecmo_spr_gaiden_draw(const rbx_gfx *gfx, const uint16_t *src, int sprite_sizey, int spr_offset_y, int flip,
                           int screen_w, uint16_t *bmp, int w, int h, int y0)
{
	static const uint8_t layout[8][8] = {
		{  0,  1,  4,  5, 16, 17, 20, 21 },
		{  2,  3,  6,  7, 18, 19, 22, 23 },
		{  8,  9, 12, 13, 24, 25, 28, 29 },
		{ 10, 11, 14, 15, 26, 27, 30, 31 },
		{ 32, 33, 36, 37, 48, 49, 52, 53 },
		{ 34, 35, 38, 39, 50, 51, 54, 55 },
		{ 40, 41, 44, 45, 56, 57, 60, 61 },
		{ 42, 43, 46, 47, 58, 59, 62, 63 }
	};
	memset(bmp, 0, (size_t)w * (size_t)h * 2);
	const int xmask = screen_w >= 512 ? 512 : 256;
	for (int i = 0; i < 256; i++, src += 8) {   // (NUM_SPRITES, drawn in order: a later one over an earlier one)
		const uint16_t attributes = src[0];
		if (!(attributes & 4))
			continue;
		int flipx = attributes & 1, flipy = (attributes >> 1) & 1;
		uint32_t color = src[2];
		const int sizex = 1 << (color & 3), sizey = 1 << ((color >> sprite_sizey) & 3);
		uint32_t number = src[1];
		if (sizex >= 2) number &= ~0x01u;
		if (sizey >= 2) number &= ~0x02u;
		if (sizex >= 4) number &= ~0x04u;
		if (sizey >= 4) number &= ~0x08u;
		if (sizex >= 8) number &= ~0x10u;
		if (sizey >= 8) number &= ~0x20u;
		int ypos = (src[3] + spr_offset_y) & 0x01ff;
		int xpos = src[4] & ((xmask * 2) - 1);
		color = (color >> 4) & 0x0f;
		if (xpos >= xmask)
			xpos -= xmask * 2;
		if (ypos >= 256)
			ypos -= 512;
		if (flip) {
			flipx = !flipx;
			flipy = !flipy;
			xpos = 256 - (8 * sizex) - xpos;
			ypos = 256 - (8 * sizey) - ypos;
			if (xpos <= -256)
				xpos += 512;
			if (ypos <= -256)
				ypos += 512;
		}
		color |= attributes & 0x03f0;
		const uint16_t c0 = (uint16_t)(color * 16);
		for (int row = 0; row < sizey; row++)
			for (int col = 0; col < sizex; col++) {
				const int sx = xpos + 8 * (flipx ? (sizex - 1 - col) : col);
				const int sy = ypos + 8 * (flipy ? (sizey - 1 - row) : row) - y0;
				if (sx >= w || sx + 8 <= 0 || sy >= h || sy + 8 <= 0)
					continue;
				const uint32_t code = (number + layout[row][col]) % gfx->count;
				if (!gfx->used[code])
					continue;
				const uint8_t *g = gfx->pix + code * 64;
				for (int j = 0; j < 8; j++) {
					const int yy = sy + j;
					if ((unsigned)yy >= (unsigned)h)
						continue;
					const uint8_t *s = g + (flipy ? 7 - j : j) * 8;
					uint16_t *d = bmp + yy * w;
					for (int k = 0; k < 8; k++) {
						const int xx = sx + k;
						if ((unsigned)xx >= (unsigned)w)
							continue;
						const uint8_t p = s[flipx ? 7 - k : k];
						if (p)
							d[xx] = (uint16_t)(c0 + p);
					}
				}
			}
	}
}
