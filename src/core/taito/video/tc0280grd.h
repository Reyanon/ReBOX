/*  Taito TC0280GRD / TC0430GRW zooming and rotating tilemap, ported from MAME src/mame/taito/tc0280grd.cpp
    (BSD-3-Clause, copyright-holders: Nicola Salmoria) and MAME tilemap_t::draw_roz (wraparound).
    64x64 tiles of 8x8 4bpp from ROM; the TC0280GRD (used in pairs) doubles incxx / incxy (xmultiply 2),
    the single-chip TC0430GRW does not. Used by Taito F2 (dondokod, cameltry, pulirula, driftout) and Z (bshark ...).
    RAM is host-order words (map it straight onto the 68000 bus); a snapshot is tc0280grd_copy.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_TC0280GRD_H
#define RBX_TC0280GRD_H

#include <stdint.h>

#define TC0280GRD_RAM_BYTES 0x2000

typedef struct {
	uint16_t ram[TC0280GRD_RAM_BYTES / 2];
	uint16_t ctrl[8];
	int xmultiply;                  // 2 = TC0280GRD, 1 = TC0430GRW
	const uint8_t *tiles, *tile_used;
	uint32_t ntiles;
	uint8_t *owned;
} tc0280grd_t;

int tc0280grd_init(tc0280grd_t *c, const uint8_t *gfx, uint32_t size, int xmultiply);
void tc0280grd_exit(tc0280grd_t *c);
void tc0280grd_reset(tc0280grd_t *c);
static inline void tc0280grd_ctrl_w(tc0280grd_t *c, int offset, uint16_t data, uint16_t mask)
{
	offset &= 7;
	c->ctrl[offset] = (uint16_t)((c->ctrl[offset] & ~mask) | (data & mask));
}
void tc0280grd_copy(tc0280grd_t *dst, const tc0280grd_t *src);
// MAME zoom_draw: pen 0 transparent, pri |= priority where drawn; tile colour = (attr >> 14) + base_color.
// bitmap row r is screen line sy0 + r, width pixels per row
void tc0280grd_draw(const tc0280grd_t *c, uint16_t *bm, uint8_t *pri, int pitch, int width, int sy0, int rows,
                    int xoffset, int yoffset, int base_color, uint8_t priority);

#endif
