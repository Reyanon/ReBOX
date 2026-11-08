/*  Data East MXC06 sprite generator, after MAME src/mame/dataeast/decmxc06.cpp (BSD-3-Clause, copyright-holders:
    Bryan McPhail, David Haywood): draw_sprites (the Mad Motor implementation).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "decmxc06.h"

#include <stddef.h>

static void transpen(const rbx_gfx *g, uint32_t code, int color_base, int flipx, int flipy, int sx, int sy, uint16_t *bm,
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
				d[x] = (uint16_t)(color_base + pen);
		}
	}
}

typedef struct {
	int height, flipx, flipy;
	uint32_t colour, pri_mask;
	uint16_t code[8];
	int x[8], y[8];
} mxc_sprite_t;

void decmxc06_draw(const decmxc06_t *s, const uint16_t *spriteram, int size, uint16_t *bm, uint8_t *pri, int pitch,
                   int bw, int bh, int top, uint32_t frame)
{
	static mxc_sprite_t list[0x800];
	const int priority = s->colpri_cb != NULL;
	int n = 0;
	for (int offs = 0; offs < size;) {
		uint32_t pri_mask = 0;
		const uint16_t data0 = spriteram[offs], data2 = spriteram[offs + 2];
		uint32_t colour = data2 >> 12;
		if (priority)
			s->colpri_cb(s->user, &colour, &pri_mask);
		const int flash = data2 & 0x800;
		int flipx = data0 & 0x2000, flipy = data0 & 0x4000;
		const int parent_flipy = flipy;
		const int h = 1 << ((data0 & 0x1800) >> 11), w = 1 << ((data0 & 0x0600) >> 9);
		int sx = data2 & 0x01ff, sy = data0 & 0x01ff;
		if (sx >= 256) sx -= 512;
		if (sy >= 256) sy -= 512;
		sx = 240 - sx;
		sy = 240 - sy;
		int mult;
		if (s->flip_screen) {
			sy = 240 - sy;
			sx = 240 - sx;
			flipx = !flipx;
			flipy = !flipy;
			mult = 16;
		} else
			mult = -16;
		if (!(spriteram[offs] & 0x8000)) {   // (thedeep: the check here)
			offs += 4;
			continue;
		}
		for (int x = 0; x < w; x++) {
			if (offs < size) {
				uint16_t code = spriteram[offs + 1] & 0x1fff;
				code &= (uint16_t)~(h - 1);
				int incy;
				if (parent_flipy)   // (multi-width sprites: the parent's y flip)
					incy = -1;
				else {
					code = (uint16_t)(code + h - 1);
					incy = 1;
				}
				if (!flash || (frame & 1)) {
					mxc_sprite_t sp;
					sp.height = h;
					sp.colour = colour;
					sp.flipx = flipx != 0;
					sp.flipy = flipy != 0;
					sp.pri_mask = pri_mask;
					for (int y = 0; y < h; y++) {
						sp.code[y] = (uint16_t)(code - y * incy);
						sp.x[y] = sx + mult * x;
						sp.y[y] = sy + mult * y;
					}
					if (priority) {
						if (n < (int)(sizeof(list) / sizeof(list[0])))
							list[n++] = sp;
					} else
						for (int y = 0; y < h; y++)
							transpen(s->gfx, sp.code[y], s->color_base + (int)sp.colour * 16, sp.flipx, sp.flipy, sp.x[y],
							         sp.y[y] - top, bm, pitch, bw, bh);
				}
			}
			offs += 4;
		}
	}
	while (n > 0) {   // (priority: the list from its end)
		const mxc_sprite_t *sp = &list[--n];
		for (int y = 0; y < sp->height; y++)
			rbx_gfx_prio_transpen(s->gfx, sp->code[y], (uint16_t)(s->color_base + (int)sp->colour * 16), sp->flipx,
			                      sp->flipy, sp->x[y], sp->y[y] - top, bm, pri, pitch, bw, bh, sp->pri_mask);
	}
}
