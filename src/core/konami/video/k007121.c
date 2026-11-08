/*  Konami 007121 video controller. Ported from MAME src/mame/konami/k007121.cpp (BSD-3-Clause, copyright-holders:
    Fabio Priuli, Acho A. Tang, R. Belmont). See k007121.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "k007121.h"

#include <string.h>

void k007121_init(k007121_t *k, int spr_dx, int spr_flip_dx)
{
	memset(k, 0, sizeof(*k));
	k->spr_dx = spr_dx;
	k->spr_flip_dx = spr_flip_dx;
}

void k007121_reset(k007121_t *k)
{
	for (int i = 0; i < 8; i++)
		k007121_ctrl_w(k, i, 0);
}

int k007121_ctrl_w(k007121_t *k, int offset, uint8_t data)
{
	offset &= 7;
	int cleared = 0;
	if (offset == 7) {
		const uint8_t off = (uint8_t)(~data & k->ctrl[7]);
		if (off & 2)
			cleared |= K007121_IRQ;
		if (off & 4)
			cleared |= K007121_FIRQ;
		if (off & 1)
			cleared |= K007121_NMI;
		k->flipscreen = (data >> 3) & 1;
	}
	k->ctrl[offset] = data;
	return cleared;
}

int k007121_scanline(k007121_t *k, int scanline, uint32_t frame, const uint8_t *spriteram)
{
	int lines = 0;
	const int nmi_mask = 0x10 << ((k->ctrl[7] >> 4) & 1);
	if ((k->ctrl[7] & 1) && (scanline & ((nmi_mask << 1) - 1)) == nmi_mask)
		lines |= K007121_NMI;
	if (scanline == 240) {
		if ((k->ctrl[7] & 4) && (frame & 1))
			lines |= K007121_FIRQ;
		if (k->ctrl[7] & 2)
			lines |= K007121_IRQ;
		if (spriteram)   // (sprites_buffer)
			memcpy(k->sprites_buffer, spriteram + ((k->ctrl[3] & 8) ? 0x800 : 0), sizeof(k->sprites_buffer));
	}
	return lines;
}

// an 8x8 element (transmask: gfx's transparent pens of this colour)
static void draw8(const uint8_t *src, int base, uint16_t mask, int flipx, int flipy, int dx, int dy, uint16_t *bm,
                  int pitch, int x0, int y0, int cx0, int cy0, int cx1, int cy1, uint8_t *pri, uint32_t pmask)
{
	for (int y = 0; y < 8; y++) {
		const int sy = dy + y;
		if (sy < cy0 || sy > cy1)
			continue;
		const uint8_t *s = src + (flipy ? 7 - y : y) * 8;
		uint16_t *d = bm + (sy - y0) * pitch - x0;
		uint8_t *pr = pri ? pri + (sy - y0) * pitch - x0 : NULL;
		for (int x = 0; x < 8; x++) {
			const int sx = dx + x;
			if (sx < cx0 || sx > cx1)
				continue;
			const int p = s[flipx ? 7 - x : x];
			if ((mask >> p) & 1)
				continue;
			if (pr) {   // (PIXEL_OP_REMAP_TRANSMASK_PRIORITY: drawn where the priority allows, then 31)
				if (!((1u << (pr[sx] & 0x1f)) & pmask))
					d[sx] = (uint16_t)(base + p);
				pr[sx] = 31;
			} else
				d[sx] = (uint16_t)(base + p);
		}
	}
}

void k007121_sprites_draw(const k007121_t *k, const uint8_t *gfx, uint32_t gfx_count, int gfx_base,
                          const uint16_t *transmask, k007121_sprite_cb_t cb, uint16_t *bm, int pitch, int x0, int y0,
                          int cx0, int cy0, int cx1, int cy1, uint8_t *pri, uint32_t pmask)
{
	enum { MAX_SPRITE_BLOCKS = 264, SIZE = 5 };
	const uint8_t *source = k->sprites_buffer;
	int num_sprites = 0, blocks = 0;
	while (blocks < MAX_SPRITE_BLOCKS && (num_sprites + 1) * SIZE <= (int)sizeof(k->sprites_buffer)) {
		switch (source[num_sprites * SIZE + 4] & 0xe) {
		case 0x02: case 0x04: blocks += 2; break;
		case 0x00: blocks += 4; break;
		case 0x08: blocks += 16; break;
		default: blocks += 1; break;
		}
		num_sprites++;
	}
	static const int x_offset[4] = { 0x0, 0x1, 0x4, 0x5 }, y_offset[4] = { 0x0, 0x2, 0x8, 0xa };
	int inc = SIZE;
	if (pri) {   // (with the priority bitmap: from the last sprite)
		source += (num_sprites - 1) * SIZE;
		inc = -SIZE;
		pmask |= 1u << 31;
	}
	for (int i = 0; i < num_sprites; i++, source += inc) {
		int number = source[0];
		const int sprite_bank = source[1] & 0x0f, attr = source[4];
		int sx = source[3], sy = source[2];
		const int xflip = (attr >> 4) & 1, yflip = (attr >> 5) & 1;
		int color = (source[1] & 0xf0) >> 4;
		if (attr & 0x01)
			sx -= 256;
		if (sy >= 240)
			sy -= 256;
		sx += k->flipscreen ? k->spr_flip_dx : k->spr_dx;
		number += ((sprite_bank & 0x3) << 8) + ((attr & 0xc0) << 4);
		number = number << 2;
		number += (sprite_bank >> 2) & 3;
		if (cb)
			cb(&number, &color, (k->ctrl[6] & 0x30) << 1);
		int width, height;
		switch (attr & 0xe) {
		case 0x06: width = height = 1; break;
		case 0x02: width = 2; height = 1; number &= ~1; break;
		case 0x04: width = 1; height = 2; number &= ~2; break;
		case 0x00: width = height = 2; number &= ~3; break;
		case 0x08: width = height = 4; number &= ~0xf; break;
		default: width = height = 1; break;
		}
		const uint16_t mask = transmask[color & 0xff];
		const int base = gfx_base + color * 16;
		for (int y = 0; y < height; y++)
			for (int x = 0; x < width; x++) {
				const int ex = xflip ? width - 1 - x : x, ey = yflip ? height - 1 - y : y;
				const uint32_t code = (uint32_t)(number + x_offset[ex] + y_offset[ey]) % gfx_count;
				int fx, fy, dx, dy;
				if (k->flipscreen) {
					fx = !xflip;
					fy = !yflip;
					dx = 248 - (sx + x * 8);
					dy = 248 - (sy + y * 8);
				} else {
					fx = xflip;
					fy = yflip;
					dx = sx + x * 8;
					dy = sy + y * 8;
				}
				draw8(gfx + code * 64, base, mask, fx, fy, dx, dy, bm, pitch, x0, y0, cx0, cy0, cx1, cy1, pri, pmask);
			}
	}
}
