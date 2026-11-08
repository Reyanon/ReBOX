/*  Taito PC090OJ sprites, ported from MAME src/mame/taito/pc090oj.cpp (BSD-3-Clause, copyright-holders:
    Nicola Salmoria).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "pc090oj.h"

#include <stdlib.h>
#include <string.h>

int pc090oj_init(pc090oj_t *c, const uint8_t *gfx, uint32_t size, int palette_entries)
{
	memset(c, 0, sizeof(*c));
	c->ntiles = size / 128;
	c->colors = palette_entries / 16;
	if (!c->ntiles)
		return -1;
	c->tiles = (uint8_t *)malloc((size_t)c->ntiles * 256);
	c->empty = (uint8_t *)malloc(c->ntiles);
	if (!c->tiles || !c->empty) {
		pc090oj_exit(c);
		return -1;
	}
	for (uint32_t t = 0; t < c->ntiles; t++) {
		const uint8_t *src = gfx + t * 128;
		uint8_t *dst = c->tiles + t * 256;
		int any = 0;
		for (int i = 0; i < 256; i++) {
			uint8_t b = src[i >> 1];
			dst[i] = (i & 1) ? (b & 15) : (b >> 4);
			any |= dst[i];
		}
		c->empty[t] = !any;
	}
	return 0;
}

void pc090oj_exit(pc090oj_t *c)
{
	free(c->tiles);
	free(c->empty);
	c->tiles = c->empty = NULL;
}

void pc090oj_reset(pc090oj_t *c)
{
	c->ctrl = 0;   // (MAME device_reset; RAM and sprite_ctrl are kept)
}

void pc090oj_word_w(pc090oj_t *c, uint32_t offset, uint16_t data, uint16_t mask)
{
	offset &= 0x1fff;
	uint16_t *p = &c->ram[offset];
	*p = (uint16_t)((*p & ~mask) | (data & mask));
	if (!c->use_buffer && offset < PC090OJ_ACTIVE_WORDS)
		c->buffered[offset] = *p;
	if (offset == 0xdff)
		c->ctrl = data;   // bit 0 is flip control (MAME stores the written data)
}

void pc090oj_eof(pc090oj_t *c)
{
	if (c->use_buffer)
		memcpy(c->buffered, c->ram, sizeof(c->buffered));
}

void pc090oj_snapshot(const pc090oj_t *c, pc090oj_regs *s)
{
	memcpy(s->ram, c->use_buffer ? c->buffered : c->ram, sizeof(s->ram));
	s->ctrl = c->direct ? c->ram[0xdff] : c->ctrl;
	s->sprite_ctrl = c->sprite_ctrl;
}

void pc090oj_draw(const pc090oj_t *c, const pc090oj_regs *s, uint16_t *bm, int pitch, uint8_t *pri,
                  int w, int y0, int h, int use_pri, uint32_t colbank, uint32_t pri_mask)
{
	int start, end, inc;
	if (use_pri) { start = 0; end = PC090OJ_ACTIVE_WORDS; inc = 4; }
	else { start = PC090OJ_ACTIVE_WORDS - 4; end = -4; inc = -4; }
	for (int offs = start; offs != end; offs += inc) {
		const uint16_t data = s->ram[offs];
		int flipy = (data & 0x8000) >> 15;
		int flipx = (data & 0x4000) >> 14;
		const uint32_t color = ((data & 0x000f) | colbank) % (uint32_t)c->colors;
		const uint32_t code = (s->ram[offs + 2] & 0x1fff) % c->ntiles;
		int x = s->ram[offs + 3] & 0x1ff;
		int y = s->ram[offs + 1] & 0x1ff;
		if (x > 0x140) x -= 0x200;
		if (y > 0x140) y -= 0x200;
		if (!(s->ctrl & 1)) {   // sprites flipscreen
			x = 320 - x - 16;
			y = 256 - y - 16;
			flipx = !flipx;
			flipy = !flipy;
		}
		x += c->x_offset;
		y += c->y_offset - y0;
		if (c->empty[code] || x <= -16 || x >= w || y <= -16 || y >= h)
			continue;
		const uint8_t *gfx = c->tiles + code * 256;
		const uint16_t base = (uint16_t)(color * 16);
		int ya = y < 0 ? -y : 0, yb = y + 16 > h ? h - y : 16;
		int xa = x < 0 ? -x : 0, xb = x + 16 > w ? w - x : 16;
		for (int r = ya; r < yb; r++) {
			const uint8_t *src = gfx + ((flipy ? 15 - r : r) << 4);
			uint16_t *dst = bm + (y + r) * pitch + x;
			uint8_t *pd = use_pri ? pri + (y + r) * w + x : NULL;
			for (int col = xa; col < xb; col++) {
				const int pen = src[flipx ? 15 - col : col];
				if (!pen)
					continue;
				if (pd) {
					if (!((1u << (pd[col] & 0x1f)) & pri_mask))
						dst[col] = (uint16_t)(base + pen);
					pd[col] = 31;
				} else {
					dst[col] = (uint16_t)(base + pen);
				}
			}
		}
	}
}
