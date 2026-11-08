/*  Data East sprite chip 52, after MAME src/mame/shared/decospr.cpp (BSD-3-Clause, copyright-holders: David Haywood).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "decospr.h"

#include <stddef.h>

// MAME gfx_element::transpen (pen tp transparent), clipped to the bitmap
static void transpen(const rbx_gfx *g, uint32_t code, int color_base, int flipx, int flipy, int sx, int sy, uint16_t *bm,
                     int pitch, int bw, int bh, int tp)
{
	code %= g->count;
	if (!tp && !g->used[code])
		return;
	const int w = g->width, h = g->height;
	const int x0 = sx < 0 ? -sx : 0, x1 = sx + w > bw ? bw - sx : w;
	const int y0 = sy < 0 ? -sy : 0, y1 = sy + h > bh ? bh - sy : h;
	if (x0 >= x1 || y0 >= y1)
		return;
	const uint8_t *t = g->pix + (size_t)code * w * h;
	for (int y = y0; y < y1; y++) {
		const uint8_t *row = t + (flipy ? h - 1 - y : y) * w;
		uint16_t *d = bm + (sy + y) * pitch + sx;
		for (int x = x0; x < x1; x++) {
			const uint8_t pen = row[flipx ? w - 1 - x : x];
			if (pen != tp)
				d[x] = (uint16_t)(color_base + pen);
		}
	}
}

// MAME transpen_raw: the pens plus the colour value as they are (no palette base)
static void transpen_raw(const rbx_gfx *g, uint32_t code, int colour, int flipx, int flipy, int sx, int sy, uint16_t *bm,
                         int pitch, int bw, int bh)
{
	code %= g->count;
	if (!g->used[code])
		return;
	const int w = g->width, h = g->height;
	const int x0 = sx < 0 ? -sx : 0, x1 = sx + w > bw ? bw - sx : w;
	const int y0 = sy < 0 ? -sy : 0, y1 = sy + h > bh ? bh - sy : h;
	if (x0 >= x1 || y0 >= y1)
		return;
	const uint8_t *t = g->pix + (size_t)code * w * h;
	for (int y = y0; y < y1; y++) {
		const uint8_t *row = t + (flipy ? h - 1 - y : y) * w;
		uint16_t *d = bm + (sy + y) * pitch + sx;
		for (int x = x0; x < x1; x++) {
			const uint8_t pen = row[flipx ? w - 1 - x : x];
			if (pen)
				d[x] = (uint16_t)(colour + pen);
		}
	}
}

// one 16x16 piece at screen (x, ypos): MAME's prio_transpen / transpen calls (the cliprect test on ypos is the caller's)
static void piece(const decospr_t *s, uint32_t code, int colour, int fx, int fy, int x, int ypos, uint32_t pmask,
                  uint16_t *bm, uint8_t *pri, int pitch, int bw, int bh, int top)
{
	if (s->raw) {
		transpen_raw(s->gfx, code, colour << s->raw_shift, fx, fy, x, ypos - top, bm, pitch, bw, bh);
		return;
	}
	const int cb = s->color_base + colour * (s->gran ? s->gran : 16);
	if (s->pri_cb)
		rbx_gfx_prio_transpen(s->gfx, code, (uint16_t)cb, fx, fy, x, ypos - top, bm, pri, pitch, bw, bh, pmask);
	else
		transpen(s->gfx, code, cb, fx, fy, x, ypos - top, bm, pitch, bw, bh, s->transpen);
}

static int sext9(int v) { return (v & 0x100) ? (v | ~0x1ff) : (v & 0x1ff); }

void decospr_draw(const decospr_t *s, const uint16_t *spriteram, int sizewords, uint16_t *bm, uint8_t *pri, int pitch,
                  int bw, int bh, int top, int wide, uint32_t frame)
{
	int offs, end, incr;
	const int flipscreen = s->flip_screen;
	const int ctop = top, cbottom = top + bh - 1;   // MAME cliprect.top() / bottom()
	if (s->pri_cb) {
		offs = sizewords - 4;
		end = -4;
		incr = -4;
	} else {
		offs = 0;
		end = sizewords;
		incr = 4;
	}
	for (; offs != end; offs += incr) {
		if (!s->alt_format) {
			int sprite = spriteram[offs + 1];
			int y = spriteram[offs];
			const int flash = (y >> 12) & 1, w = (y >> 11) & 1;
			if (flash && (frame & 1))
				continue;
			int x = spriteram[offs + 2];
			const int colour = s->raw ? ((x >> 9) & 0x7f) | ((y >> 15) & 1 ? 0x80 : 0)   // (fghthist marks priority)
			                          : s->col_cb ? s->col_cb((uint16_t)x, (y >> 15) & 1) : (x >> 9) & 0x1f;
			const uint32_t pmask = s->pri_cb ? s->pri_cb((uint16_t)x, (y >> 15) & 1) : 0;
			int fx = (y >> 13) & 1, fy = (y >> 14) & 1;
			int multi = (1 << (((y >> 10) & 1) << 1 | ((y >> 9) & 1))) - 1;   // (bitswap<2>(y, 10, 9): 1x, 2x, 4x, 8x)
			if (flipscreen)
				x = ((x & 0x1ff) - s->x_offset) & 0x1ff;
			else
				x = ((x & 0x1ff) + s->x_offset) & 0x1ff;
			y = ((y & 0x1ff) + s->y_offset) & 0x1ff;
			if (wide) {
				if (x >= 320) x -= 512;
				if (y >= 256) y -= 512;
				y = 240 - y;
				x = 304 - x;
			} else {
				if (x >= 256) x -= 512;
				if (y >= 256) y -= 512;
				y = 240 - y;
				x = 240 - x;
			}
			if (!s->bootleg)   // (several clone chips do not, see jumpkids)
				sprite &= ~multi;
			int inc, mult;
			if (fy)
				inc = -1;
			else {
				sprite += multi;
				inc = 1;
			}
			if (flipscreen) {
				y = 240 - y;
				fy = !fy;
				mult = 16;
			} else
				mult = -16;
			if (flipscreen ^ s->flipallx) {
				x = wide ? 304 - x : 240 - x;
				fx = !fx;
			}
			const int mult2 = multi + 1;
			while (multi >= 0) {
				const int ypos = y + mult * multi;
				if (ypos <= cbottom && ypos >= ctop - 16) {
					piece(s, (uint32_t)(sprite - multi * inc), colour, fx, fy, x, ypos, pmask, bm, pri, pitch, bw, bh, top);
					if (w)   // (double wing)
						piece(s, (uint32_t)((sprite - multi * inc) - mult2), colour, fx, fy, !flipscreen ? x - 16 : x + 16,
						      ypos, pmask, bm, pri, pitch, bw, bh, top);
				}
				multi--;
			}
		} else {
			int y = spriteram[offs + 0];
			const int sprite = spriteram[offs + 3];
			const uint32_t pmask = s->pri_cb ? s->pri_cb((uint16_t)(spriteram[offs + 2] & 0xff), 0) : 0;
			int x = spriteram[offs + 1];
			if ((y >> 13) & 1 && (frame & 1))
				continue;
			const int colour = spriteram[offs + 2] & (s->raw ? 0xff : 0x1f);   // (raw: every bit, for the mixing)
			const int h = (spriteram[offs + 2] & 0xf000) >> 12, w = (spriteram[offs + 2] & 0x0f00) >> 8;
			int fx = !((spriteram[offs + 0] >> 14) & 1), fy = !((spriteram[offs + 0] >> 15) & 1);
			int mult, mult2;
			if (!flipscreen) {
				x &= 0x1ff;
				y &= 0x1ff;
				if (x > 0x180) x -= 512;
				if (y > 0x180) y -= 512;
				if (fx) { mult = -16; x += 16 * w; } else { mult = 16; x -= 16; }
				if (fy) { mult2 = -16; y += 16 * h; } else { mult2 = 16; y -= 16; }
			} else {
				fx = !fx;
				fy = !fy;
				x = 304 - sext9(x);
				y = 240 - sext9(y);
				if (x >= 432) x -= 512;
				if (y >= 384) y -= 512;
				if (fx) { mult = -16; x += 16; } else { mult = 16; x -= 16 * w; }
				if (fy) { mult2 = -16; y += 16; } else { mult2 = 16; y -= 16 * h; }
			}
			for (int xx = 0; xx < w; xx++)
				for (int yy = 0; yy < h; yy++) {
					int ypos = y + mult2 * (h - yy);
					const int xpos = x + mult * (w - xx);
					const uint32_t code = (uint32_t)(sprite + yy + h * xx);
					if (ypos <= cbottom && ypos >= ctop - 16)
						piece(s, code, colour, fx, fy, xpos, ypos, pmask, bm, pri, pitch, bw, bh, top);
					ypos -= 512;   // (wrap-around y)
					if (ypos <= cbottom && ypos >= ctop - 16)
						piece(s, code, colour, fx, fy, xpos, ypos, pmask, bm, pri, pitch, bw, bh, top);
				}
		}
	}
}
