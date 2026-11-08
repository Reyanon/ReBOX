/*  Taito TC0280GRD / TC0430GRW (tc0280grd.h), ported from MAME src/mame/taito/tc0280grd.cpp (BSD-3-Clause,
    copyright-holders: Nicola Salmoria) and src/emu/tilemap.cpp draw_roz_core.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "tc0280grd.h"

#include <stdlib.h>
#include <string.h>

int tc0280grd_init(tc0280grd_t *c, const uint8_t *gfx, uint32_t size, int xmultiply)
{
	uint32_t n = size / 32;
	if (!n)
		n = 1;
	uint8_t *t = (uint8_t *)calloc((size_t)n * 65, 1);
	if (!t)
		return -1;
	uint8_t *used = t + (size_t)n * 64;
	for (uint32_t i = 0; i < n && gfx; i++) {
		uint8_t *d = t + (size_t)i * 64, any = 0;
		for (int j = 0; j < 64; j++) {   // gfx_8x8x4_packed_msb
			uint8_t b = gfx[i * 32 + (j >> 1)];
			d[j] = (j & 1) ? (b & 15) : (b >> 4);
			any |= d[j];
		}
		used[i] = any != 0;
	}
	c->owned = t;
	c->tiles = t;
	c->tile_used = used;
	c->ntiles = n;
	c->xmultiply = xmultiply;
	tc0280grd_reset(c);
	return 0;
}

void tc0280grd_exit(tc0280grd_t *c)
{
	free(c->owned);
	c->owned = NULL;
	c->tiles = c->tile_used = NULL;
}

void tc0280grd_reset(tc0280grd_t *c)
{
	memset(c->ram, 0, sizeof(c->ram));
	memset(c->ctrl, 0, sizeof(c->ctrl));
}

void tc0280grd_copy(tc0280grd_t *dst, const tc0280grd_t *src)
{
	memcpy(dst->ram, src->ram, sizeof(src->ram));
	memcpy(dst->ctrl, src->ctrl, sizeof(src->ctrl));
}

void tc0280grd_draw(const tc0280grd_t *c, uint16_t *bm, uint8_t *pri, int pitch, int width, int sy0, int rows,
                    int xoffset, int yoffset, int base_color, uint8_t priority)
{
	// 24-bit signed start positions
	int32_t startx = (int32_t)(((c->ctrl[0] & 0xff) << 16) + c->ctrl[1]);
	if (startx & 0x800000)
		startx -= 0x1000000;
	int incxx = (int16_t)c->ctrl[2] * c->xmultiply;
	int incyx = (int16_t)c->ctrl[3];
	int32_t starty = (int32_t)(((c->ctrl[4] & 0xff) << 16) + c->ctrl[5]);
	if (starty & 0x800000)
		starty -= 0x1000000;
	int incxy = (int16_t)c->ctrl[6] * c->xmultiply;
	int incyy = (int16_t)c->ctrl[7];
	startx -= xoffset * incxx + yoffset * incyx;
	starty -= xoffset * incxy + yoffset * incyy;
	// draw_roz with 16.16 coordinates (<< 4), wraparound over the 512x512 map; pre-advanced to the clip origin
	uint32_t sx = (uint32_t)startx << 4, sy = (uint32_t)starty << 4;
	uint32_t ixx = (uint32_t)incxx << 4, ixy = (uint32_t)incxy << 4, iyx = (uint32_t)incyx << 4, iyy = (uint32_t)incyy << 4;
	sx += (uint32_t)sy0 * iyx;
	sy += (uint32_t)sy0 * iyy;
	const uint32_t nt = c->ntiles;
	for (int r = 0; r < rows; r++) {
		uint32_t cx = sx, cy = sy;
		uint16_t *d = bm + r * pitch;
		uint8_t *p = pri + r * pitch;
		for (int x = 0; x < width; x++, cx += ixx, cy += ixy) {
			int tx = (cx >> 16) & 511, ty = (cy >> 16) & 511;
			uint16_t attr = c->ram[(ty >> 3) * 64 + (tx >> 3)];
			uint32_t code = attr & 0x3fff;
			if (code >= nt)
				code %= nt;
			uint8_t pen = c->tiles[(size_t)code * 64 + (ty & 7) * 8 + (tx & 7)];
			if (pen) {
				d[x] = (uint16_t)((((attr >> 14) + base_color) << 4) | pen);
				p[x] |= priority;
			}
		}
		sx += iyx;
		sy += iyy;
	}
}
