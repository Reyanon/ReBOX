/*  Konami 007342 (two 64x32 tilemaps of 8x8 tiles with row / column scroll) and 007420 (sprites, zoomed). Ported from
    MAME src/mame/konami/k007342.cpp, k007420.cpp (BSD-3-Clause, copyright-holders: Fabio Priuli, Acho A. Tang,
    R. Belmont).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_K007342_H
#define RBX_K007342_H

#include <stdint.h>

typedef struct {
	uint8_t ram[0x2000];          // (0000 colour RAM 0, 0800 video RAM 0, 1000 colour 1, 1800 video 1)
	uint8_t scroll_ram[0x200];
	uint8_t regs[8], int_enabled, flipscreen, wrap_y;
	uint16_t scrollx[2];
	uint8_t scrolly[2];
	uint8_t state_end;
} k007342_t;

void k007342_reset(k007342_t *k);
void k007342_vreg_w(k007342_t *k, int offset, uint8_t data);

// the tile callback (layer, regs[1], code, color, flags as MAME's tile callback); flags: bit 0 flip x, bit 1 flip y
typedef void (*k007342_tile_cb_t)(int layer, uint32_t bank, uint32_t *code, uint32_t *color, int *flags);

// tilemap_update + tilemap_draw of a layer: category cat (-1 all), opaque, priority OR-ed into pri (may be NULL),
// pens color * 16 + pixel (+ gfx_base) of the 8x8 elements (one byte per pixel). The bitmap's origin is the screen's
// (x0, y0); the clip in the screen's coordinates
void k007342_draw(const k007342_t *k, int layer, int cat, int opaque, uint8_t priority, const uint8_t *gfx,
                  uint32_t gfx_count, int gfx_base, k007342_tile_cb_t cb, uint16_t *bm, uint8_t *pri, int pitch, int x0,
                  int y0, int cx0, int cy0, int cx1, int cy1);

// K007420: 0x200 bytes of sprite RAM; cb(code, color) as MAME's sprite callback; bank_limit the code mask
typedef void (*k007420_cb_t)(uint32_t *code, uint32_t *color);
void k007420_draw(const uint8_t *ram, int flipscreen, int wrap_y, uint32_t bank_limit, k007420_cb_t cb, const uint8_t *gfx,
                  const uint8_t *opaque, uint32_t gfx_count, int gfx_base, uint16_t *bm, int pitch, int x0, int y0, int cx0,
                  int cy0, int cx1, int cy1);

#endif
