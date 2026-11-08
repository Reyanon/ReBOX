/*  Konami 051316 PSAC (a 32x32 tilemap of 16x16 tiles, rotated and zoomed). Ported from MAME
    src/devices/video/k051316.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria) and tilemap_t::draw_roz of
    src/emu/tilemap.cpp (BSD-3-Clause, copyright-holders: Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "k051316.h"
#include "../../rbx.h"

#include <stdlib.h>
#include <string.h>

int k051316_init(k051316_t *k, int chip, const uint8_t *rom, uint32_t rom_size, int dx, int dy, k051316_cb_t cb)
{
	return k051316_init_bpp(k, chip, 4, rom, rom_size, dx, dy, cb);
}

int k051316_init_bpp(k051316_t *k, int chip, int bpp, const uint8_t *rom, uint32_t rom_size, int dx, int dy, k051316_cb_t cb)
{
	memset(k, 0, sizeof(*k));
	k->bpp = bpp;
	k->rom = rom;
	k->rom_size = rom_size;
	k->chip = chip;
	k->dx = dx;
	k->dy = dy;
	k->cb = cb;
	// (charlayout4: 16x16, 4 bpp packed, the first pixel in the top nibble; charlayout7 / 8: a byte a pixel, 7 bpp its
	// low 7 bits)
	k->ntiles = rom_size / (bpp == 4 ? 128 : 256);
	k->gfx = (uint8_t *)malloc((size_t)k->ntiles * 256 + 1);
	k->pix = (uint16_t *)malloc(512 * 512 * sizeof(uint16_t));
	k->key = (uint32_t *)malloc(1024 * sizeof(uint32_t));
	if (!k->gfx || !k->pix || !k->key)
		return -1;
	if (bpp != 4) {
		const uint8_t mask = bpp == 7 ? 0x7f : 0xff;
		for (uint32_t i = 0; i < k->ntiles * 256; i++)
			k->gfx[i] = rom[i] & mask;
	} else
		for (uint32_t t = 0; t < k->ntiles; t++)
			for (int i = 0; i < 128; i++) {
				const uint8_t b = rom[t * 128 + i];
				k->gfx[t * 256 + i * 2] = b >> 4;
				k->gfx[t * 256 + i * 2 + 1] = b & 15;
			}
	k051316_reset(k);
	return 0;
}

void k051316_exit(k051316_t *k)
{
	free(k->gfx);
	free(k->pix);
	free(k->key);
	k->gfx = NULL;
	k->pix = NULL;
	k->key = NULL;
	k->ntiles = 0;
}

void k051316_invalidate(k051316_t *k)
{
	if (k->key)
		memset(k->key, 0xff, 1024 * sizeof(uint32_t));
}

void k051316_reset(k051316_t *k)   // (device_start's state: MAME has no reset)
{
	memset(k->ram, 0, sizeof(k->ram));
	memset(k->ctrl, 0, sizeof(k->ctrl));
	k->wrap = 0;
	k->readout = 1;
	k->flipx_en = k->flipy_en = 0;
	k051316_invalidate(k);
}

void k051316_ctrl_w(k051316_t *k, int offset, uint8_t data)
{
	offset &= 0x1f;
	if (offset == 0x0e) {
		k->readout = !(data & 1);
		k->flipx_en = (data >> 1) & 1;
		k->flipy_en = (data >> 2) & 1;
	} else if (offset < 0x0e)
		k->ctrl[offset] = data;
}

uint8_t k051316_rom_r(const k051316_t *k, int offset)
{
	if (!k->readout || !k->rom)
		return 0;
	uint32_t addr = (uint32_t)(offset + (k->ctrl[0x0c] << 11) + (k->ctrl[0x0d] << 19));
	if (k->bpp == 4)
		addr /= 2;   // (m_pixels_per_byte: 2 at 4 bpp, else 1)
	return k->rom[addr & (k->rom_size - 1)];
}

// the pixmap's tiles that changed (get_tile_info): a key of code, colour and flips; a pen of colour * 16 + pixel
static void update_pixmap(const k051316_t *k)
{
	for (int i = 0; i < 1024; i++) {
		int code = k->ram[i], color = k->ram[i + 0x400];
		int fl = 0;
		if (k->flipx_en && (color & 0x40))
			fl |= 1;
		if (k->flipy_en && (color & 0x80))
			fl |= 2;
		k->cb(k->chip, &code, &color);
		const uint32_t c = (uint32_t)code % k->ntiles;
		const uint32_t key = c | (uint32_t)(color & 0x3fff) << 16 | (uint32_t)fl << 30;   // (c below 1 << 16)
		if (k->key[i] == key)
			continue;
		k->key[i] = key;
		const uint8_t *src = k->gfx + c * 256;
		const uint16_t pen = (uint16_t)(color << k->bpp);   // (the granularity: 1 << bpp)
		uint16_t *d = k->pix + (i >> 5) * 16 * 512 + (i & 31) * 16;
		for (int y = 0; y < 16; y++) {
			const uint8_t *s = src + ((fl & 2) ? 15 - y : y) * 16;
			for (int x = 0; x < 16; x++) {
				const int p = s[(fl & 1) ? 15 - x : x];
				// (pixel 0 transparent: kept as pen 0 + colour * 16 with bit 15 set)
				d[y * 512 + x] = p ? (uint16_t)(pen + p) : (uint16_t)(0x8000 | pen);
			}
		}
	}
}

void k051316_zoom_draw(const k051316_t *k, uint16_t *frame, uint8_t *pri, int pitch, int fx0, int fy0, int cx0, int cy0,
                       int cx1, int cy1, int flags, uint8_t priority)
{
	if (!k->gfx || !k->pix)
		return;
	update_pixmap(k);
	const uint32_t startx0 = (uint32_t)(256 * (int16_t)(256 * k->ctrl[0x00] + k->ctrl[0x01]));
	const int incxx = (int16_t)(256 * k->ctrl[0x02] + k->ctrl[0x03]);
	const int incyx = (int16_t)(256 * k->ctrl[0x04] + k->ctrl[0x05]);
	const uint32_t starty0 = (uint32_t)(256 * (int16_t)(256 * k->ctrl[0x06] + k->ctrl[0x07]));
	const int incxy = (int16_t)(256 * k->ctrl[0x08] + k->ctrl[0x09]);
	const int incyy = (int16_t)(256 * k->ctrl[0x0a] + k->ctrl[0x0b]);
	uint32_t startx = startx0 - (uint32_t)((16 + k->dy) * incyx) - (uint32_t)((-7 + k->dx) * incxx);
	uint32_t starty = starty0 - (uint32_t)((16 + k->dy) * incyy) - (uint32_t)((-7 + k->dx) * incxy);
	// (draw_roz: << 5 to 16.16, the clip's corner)
	startx <<= 5;
	starty <<= 5;
	const uint32_t ixx = (uint32_t)incxx << 5, ixy = (uint32_t)incxy << 5, iyx = (uint32_t)incyx << 5, iyy = (uint32_t)incyy << 5;
	startx += (uint32_t)cx0 * ixx + (uint32_t)cy0 * iyx;
	starty += (uint32_t)cx0 * ixy + (uint32_t)cy0 * iyy;
	const int opaque = flags & K051316_DRAW_OPAQUE, wrap = k->wrap;
	const uint16_t *pix = k->pix;
	if (k->layermask && (flags & (K051316_DRAW_LAYER0 | K051316_DRAW_LAYER1))) {   // (map_pens_to_layer: all pens to
		// layer 1, those with the mask's bits set to layer 0; the chosen layer's pixels drawn opaque)
		const uint16_t lm = (uint16_t)k->layermask, pm = (uint16_t)((1u << k->bpp) - 1);
		const int want0 = (flags & K051316_DRAW_LAYER0) != 0;
		for (int y = cy0; y <= cy1; y++, startx += iyx, starty += iyy) {
			uint32_t cx = startx, cy = starty;
			uint16_t *d = frame + (y - fy0) * pitch - fx0;
			uint8_t *p = pri + (y - fy0) * pitch - fx0;
			for (int x = cx0; x <= cx1; x++, cx += ixx, cy += ixy) {
				uint32_t px = cx >> 16, py = cy >> 16;
				if (wrap) {
					px &= 511;
					py &= 511;
				} else if (px >= 512 || py >= 512)
					continue;
				const uint16_t v = pix[py * 512 + px] & 0x7fff;
				if ((((v & pm) & lm) == lm) != want0)
					continue;
				d[x] = v;
				p[x] |= priority;
			}
		}
		return;
	}
	for (int y = cy0; y <= cy1; y++, startx += iyx, starty += iyy) {
		uint32_t cx = startx, cy = starty;
		uint16_t *d = frame + (y - fy0) * pitch - fx0;
		uint8_t *p = pri + (y - fy0) * pitch - fx0;
		if (!ixy) {   // (no rotation: one row of the pixmap)
			uint32_t py = cy >> 16;
			if (wrap)
				py &= 511;
			else if (py >= 512)
				continue;
			const uint16_t *row = pix + py * 512;
			if (ixx == 0x10000 && !(cx & 0xffff)) {   // (1:1: the pixmap's row in order)
				uint32_t px = cx >> 16;
				for (int x = cx0; x <= cx1; x++, px = (px + 1) & 0xffff) {   // (cx >> 16 of the 32-bit sum)
					if (wrap)
						px &= 511;
					else if (px >= 512)
						continue;
					const uint16_t v = row[px];
					if (v & 0x8000) {
						if (!opaque)
							continue;
						d[x] = v & 0x7fff;
					} else
						d[x] = v;
					p[x] |= priority;
				}
				continue;
			}
			for (int x = cx0; x <= cx1; x++, cx += ixx) {
				uint32_t px = cx >> 16;
				if (wrap)
					px &= 511;
				else if (px >= 512)
					continue;
				const uint16_t v = row[px];
				if (v & 0x8000) {
					if (!opaque)
						continue;
					d[x] = v & 0x7fff;
				} else
					d[x] = v;
				p[x] |= priority;
			}
			continue;
		}
		for (int x = cx0; x <= cx1; x++, cx += ixx, cy += ixy) {
			uint32_t px = cx >> 16, py = cy >> 16;
			if (wrap) {
				px &= 511;
				py &= 511;
			} else if (px >= 512 || py >= 512)
				continue;
			const uint16_t v = pix[py * 512 + px];
			if (v & 0x8000) {
				if (!opaque)
					continue;
				d[x] = v & 0x7fff;
			} else
				d[x] = v;
			p[x] |= priority;
		}
	}
}

int k051316_state(k051316_t *k, void *ctx, int load, const char *tag)
{
	if (load)
		k051316_invalidate(k);
	return rbx_state_io(ctx, tag, k, (uint32_t)((const uint8_t *)&k->state_end - (const uint8_t *)k));
}
