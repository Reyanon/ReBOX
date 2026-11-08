/*  Konami 007342 and 007420. Ported from MAME src/mame/konami/k007342.cpp, k007420.cpp (BSD-3-Clause,
    copyright-holders: Fabio Priuli, Acho A. Tang, R. Belmont). See k007342.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "k007342.h"
#include "../../video/zoomgfx.h"

#include <string.h>

void k007342_reset(k007342_t *k)
{
	k->int_enabled = k->flipscreen = k->wrap_y = 0;
	k->scrollx[0] = k->scrollx[1] = 0;
	k->scrolly[0] = k->scrolly[1] = 0;
	memset(k->regs, 0, sizeof(k->regs));
}

void k007342_vreg_w(k007342_t *k, int offset, uint8_t data)
{
	switch (offset & 7) {
	case 0:
		k->int_enabled = (data >> 1) & 1;
		k->flipscreen = (data >> 4) & 1;
		break;
	case 2:
		k->scrollx[0] = (uint16_t)((k->scrollx[0] & 0xff) | ((data & 0x01) << 8));
		k->scrollx[1] = (uint16_t)((k->scrollx[1] & 0xff) | ((data & 0x02) << 7));
		k->wrap_y = (data >> 7) & 1;   // (sprite_wrap_y_cb)
		break;
	case 3: k->scrollx[0] = (uint16_t)((k->scrollx[0] & 0x100) | data); break;
	case 4: k->scrolly[0] = data; break;
	case 5: k->scrollx[1] = (uint16_t)((k->scrollx[1] & 0x100) | data); break;
	case 6: k->scrolly[1] = data; break;
	}
	k->regs[offset & 7] = data;
}

void k007342_draw(const k007342_t *k, int layer, int cat, int opaque, uint8_t priority, const uint8_t *gfx,
                  uint32_t gfx_count, int gfx_base, k007342_tile_cb_t cb, uint16_t *bm, uint8_t *pri, int pitch, int x0,
                  int y0, int cx0, int cy0, int cx1, int cy1)
{
	const uint8_t *cram = k->ram + layer * 0x1000, *vram = cram + 0x800;
	// (tilemap_update: layer 0's modes; 0x0c 32 columns, 0x14 256 rows, else the plain scroll)
	const int mode = layer ? 0 : (k->regs[2] & 0x1c);
	const int sx0 = k->scrollx[layer], sy0 = k->scrolly[layer];
	for (int y = cy0; y <= cy1; y++) {
		uint16_t *d = bm + (y - y0) * pitch - x0;
		uint8_t *pr = pri ? pri + (y - y0) * pitch - x0 : NULL;
		const int scrollx = mode == 0x14 ? k->scroll_ram[2 * (y & 255)] + 256 * k->scroll_ram[2 * (y & 255) + 1] : sx0;
		for (int x = cx0; x <= cx1;) {
			const int tx = (x + scrollx) & 511;
			int n = 8 - (tx & 7);
			if (n > cx1 - x + 1)
				n = cx1 - x + 1;
			if (mode == 0x0c && n > 8 - (x & 7))   // (the column's scroll by the screen's 8-pixel column)
				n = 8 - (x & 7);
			const int scrolly = mode == 0x0c ? k->scroll_ram[2 * ((x & 255) / 8)] + 256 * k->scroll_ram[2 * ((x & 255) / 8) + 1] : sy0;
			const int ty = (y + scrolly) & 255;
			const int col = tx >> 3, row = ty >> 3;
			const int idx = (col & 0x1f) + ((row & 0x1f) << 5) + ((col & 0x20) << 5);   // (scan)
			uint32_t color = cram[idx], code = vram[idx];
			int flags = (int)((color & 0x30) >> 4);
			const int tcat = (int)((color & 0x80) >> 7);
			if (cb)
				cb(layer, k->regs[1], &code, &color, &flags);
			if (cat < 0 || tcat == cat) {
				const int py = (flags & 2) ? 7 - (ty & 7) : (ty & 7);
				const uint8_t *src = gfx + (code % gfx_count) * 64 + py * 8;
				const int pb = gfx_base + (int)color * 16;
				for (int i = 0; i < n; i++) {
					const int px = (tx + i) & 7;
					const int p = src[(flags & 1) ? 7 - px : px];
					if (p || opaque) {
						d[x + i] = (uint16_t)(pb + p);
						if (pr)
							pr[x + i] |= priority;
					}
				}
			}
			x += n;
		}
	}
}

void k007420_draw(const uint8_t *ram, int flipscreen, int wrap_y, uint32_t bank_limit, k007420_cb_t cb, const uint8_t *gfx,
                  const uint8_t *opaque, uint32_t gfx_count, int gfx_base, uint16_t *bm, int pitch, int x0, int y0, int cx0,
                  int cy0, int cx1, int cy1)
{
	const uint32_t codemask = bank_limit, bankmask = ~bank_limit;
	zoomgfx_t g = { (uint8_t *)gfx, (uint8_t *)opaque, gfx_count, 8, 8 };
	zoomgfx_target tg = { bm - x0, NULL, pitch, cx0, cy0, cx1, cy1, y0, 0 };
	static const int xoffset[4] = { 0, 1, 4, 5 }, yoffset[4] = { 0, 2, 8, 10 };
	for (int offs = 0x200 - 8; offs >= 0; offs -= 8) {
		uint32_t code = ram[offs + 1], color = ram[offs + 2];
		int ox = ram[offs + 3] - ((ram[offs + 4] & 0x80) << 1), oy = 256 - ram[offs + 0];
		int flipx = (ram[offs + 4] & 0x04) != 0, flipy = (ram[offs + 4] & 0x08) != 0;
		cb(&code, &color);
		const uint32_t bank = code & bankmask;
		code &= codemask;
		uint32_t zoom = ram[offs + 5] | ((ram[offs + 4] & 0x03) << 8);
		if (!zoom)
			continue;
		zoom = 0x10000 * 128 / zoom;
		int width, height;
		switch (ram[offs + 4] & 0x70) {
		case 0x30: width = height = 1; break;
		case 0x20: width = 2; height = 1; code &= ~1u; break;
		case 0x10: width = 1; height = 2; code &= ~2u; break;
		case 0x00: width = height = 2; code &= ~3u; break;
		case 0x40: width = height = 4; code &= ~0xfu; break;
		default: width = height = 1; break;
		}
		if (flipscreen) {
			ox = 256 - ox - (int)((zoom * (uint32_t)width + (1 << 12)) >> 13);
			oy = 256 - oy - (int)((zoom * (uint32_t)height + (1 << 12)) >> 13);
			flipx = !flipx;
			flipy = !flipy;
		}
		for (int y = 0; y < height; y++) {
			const int sy = oy + (int)((zoom * (uint32_t)y + (1 << 12)) >> 13);
			const int zh = (oy + (int)((zoom * (uint32_t)(y + 1) + (1 << 12)) >> 13)) - sy;
			for (int x = 0; x < width; x++) {
				uint32_t c = code;
				const int sx = ox + (int)((zoom * (uint32_t)x + (1 << 12)) >> 13);
				const int zw = (ox + (int)((zoom * (uint32_t)(x + 1) + (1 << 12)) >> 13)) - sx;
				c += (uint32_t)(flipx ? xoffset[width - 1 - x] : xoffset[x]);
				c += (uint32_t)(flipy ? yoffset[height - 1 - y] : yoffset[y]);
				if (c & bankmask)
					continue;
				c = (c + bank) % gfx_count;
				// (transpen at 1:1, zoom_transpen else: the same at a scale of 0x10000)
				const uint32_t scx = zoom == 0x10000 ? 0x10000 : (uint32_t)((zw << 16) / 8);
				const uint32_t scy = zoom == 0x10000 ? 0x10000 : (uint32_t)((zh << 16) / 8);
				zoomgfx_draw(&g, &tg, c, (uint32_t)(gfx_base + (int)color * 16), flipx, flipy, sx, sy, scx, scy, 0, 0);
				if (wrap_y)
					zoomgfx_draw(&g, &tg, c, (uint32_t)(gfx_base + (int)color * 16), flipx, flipy, sx,
					             sy + (flipscreen ? 256 : -256), scx, scy, 0, 0);
			}
		}
	}
}
