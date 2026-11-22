/*  Namco C169 (ROZ): two rotate / zoom layers over one 256x256 map of 16x16 tiles (8bpp, a 1bpp mask), the second
    layer's parameters optionally per line from the RAM. Drawing only, a line at a time (the driver keeps the RAM and
    the control registers, and their copy for the drawing).
    Ported from MAME 0.289 src/mame/namco/namco_c169roz.cpp (BSD-3-Clause, copyright-holders: Phil Stroffolino,
    David Haywood). The System FL path (tilemap draw_roz) is not ported.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_C169ROZ_H
#define RBX_C169ROZ_H

#include <stdint.h>

// code (the RAM word & 0x3fff) -> the tile and the mask numbers; which: the layer (MAME's tile callback)
typedef void (*c169roz_tile_cb)(uint16_t code, int *tile, int *mask, int which);

typedef struct {
	const uint8_t *gfx;     // 16x16x8 raw, 256 bytes a tile
	uint32_t count;
	const uint8_t *mask;    // 32 bytes a tile
	uint32_t mask_count;
	uint32_t ram_words;     // (a power of two: 0x8000 on System 2)
	uint16_t color_base;
	c169roz_tile_cb cb;     // NULL: tile = mask = code
} c169roz_t;

// MAME draw(pri) for the line y between x0 and x1 (exclusive): control = the 16 registers, ram = the RAM words
// (host order). Pixels drawn set pri[x] = prival
void c169roz_draw_line(const c169roz_t *r, const uint16_t *control, const uint16_t *ram, int y, int x0, int x1,
                       uint16_t *dest, uint8_t *pri, int p, uint8_t prival);

// the same in two steps: the line's layers unpacked once, then drawn for each priority
typedef struct {
	uint32_t left, top, size;
	int color, priority, on;
	int32_t startx, starty, incxx, incxy, incyx, incyy;
} c169roz_layer;
void c169roz_line_params(const c169roz_t *r, const uint16_t *control, const uint16_t *ram, int y, c169roz_layer lp[2]);
void c169roz_draw_line_p(const c169roz_t *r, const c169roz_layer lp[2], const uint16_t *ram, int y, int x0, int x1,
                         uint16_t *dest, uint8_t *pri, int p, uint8_t prival);

#endif
