/*  Seta (or Allumer) X1-012 (+ X1-011) tile layer generator, ported from MAME src/mame/seta/x1_012.cpp
    (BSD-3-Clause, copyright-holders: Luca Elia). MAME's tilemap is replaced by a direct per-tile draw: the layer has one
    scroll for the whole screen, so nothing is cached: the VRAM of the frame is read once and the tiles are copied by line.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_X1_012_H
#define RBX_X1_012_H

#include "../../video/gfxlayout.h"

#include <stdint.h>

#define X1_012_VRAM_WORDS 0x2000   // two tilemaps of 0x800 code + 0x800 attribute words

// a gfx element of the layer: 16x16 tiles, one byte per pixel
typedef struct {
	uint8_t *pix;       // 256 bytes per tile (shared between the entries of a layer)
	uint16_t *rows;     // per tile, bit n = row n has a pixel that is not 0
	uint16_t *full;     // per tile, bit n = row n has no pixel 0
	uint32_t count;
	uint16_t base;      // pen of color 0 (GFXDECODE color base)
	uint16_t gran;      // pens per color
} x1_012_gfx;

typedef struct {
	uint16_t vctrl[3];
	uint16_t *vram;     // X1_012_VRAM_WORDS host-order words, on the 68000 bus
	int xoffs[2];       // [0] not flipped, [1] flipped
	x1_012_gfx gfx[2];
	int ngfx;           // 0: no layer on this board
	// the frame's copy (x1_012_capture)
	uint16_t snap[0x1000];
	uint16_t snap_ctrl[3];
} x1_012_t;

// decode the first gfx entry from a MAME layout (size: as RGN_FRAC gives the element count); the second entry shares it
int x1_012_decode(x1_012_gfx *g, const rbx_gfx_layout *l, const uint8_t *rom, uint32_t size, int base, int gran);
// the same for the Seta packed layouts (layout_tilemap, _6bpp, _8bpp: planes = 4, 6, 8), much faster
int x1_012_decode_packed(x1_012_gfx *g, int planes, const uint8_t *rom, uint32_t size, int base, int gran);
void x1_012_share(x1_012_gfx *g, const x1_012_gfx *from, int base);
void x1_012_free(x1_012_t *l);

void x1_012_capture(x1_012_t *l);
// the frame's layer into an indexed bitmap: bitmap line 0 is screen line y0, of a screen vis_h lines high. opaque: MAME
// TILEMAP_DRAW_OPAQUE (pen 0 drawn too)
void x1_012_draw(const x1_012_t *l, uint16_t *bm, int pitch, int w, int h, int y0, int vis_h, int flip, int opaque);
// MAME draw_tilemap_palette_effect: pixels of the last color are not drawn, add_pens goes on the pen under them
void x1_012_draw_effect(const x1_012_t *l, uint16_t *bm, int pitch, int w, int h, int y0, int vis_h, int flip, int add_pens);

#endif
