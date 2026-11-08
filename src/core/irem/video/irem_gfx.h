/*  The planar graphics of the Irem 16-bit boards decoded once to a byte a pixel: the four planes in four equal parts of
    the region, the last part the top bit (the charlayout and spritelayouts of MAME src/mame/irem/m72.cpp, m90.cpp,
    m92.cpp, m107.cpp; BSD-3-Clause, copyright-holders: Nicola Salmoria, Bryan McPhail).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef IREM_GFX_H
#define IREM_GFX_H

#include "../../video/gfxlayout.h"

enum {
	IGFX_CHAR,      // 8x8, 8 bytes a plane
	IGFX_SPRITE,    // 16x16, 32 bytes: the left 16 rows, then the right 16 rows
	IGFX_SPRITE2,   // 16x16, 32 bytes: two bytes a row
	IGFX_SPRITE3,   // 16x16, 32 bytes: two bytes a row, the right half first (M107 spritelayout2)
	IGFX_M107_CHAR  // 8x8, 32 bytes, the planes packed in the region: planes { 8, 0, 24, 16 }
};

// fills g (used too); rows, if not NULL, gets a byte per 8x8 element: a bit per row with a pixel (malloc'ed). 0 = ok
int irem_gfx_decode(rbx_gfx *g, const uint8_t *rom, uint32_t size, int kind, uint8_t **rows);
void irem_gfx_free(rbx_gfx *g);

#endif
