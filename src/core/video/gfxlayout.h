/*  MAME gfx_layout decoding (src/emu/drawgfx.cpp gfx_element, BSD-3-Clause, copyright-holders: Nicola Salmoria,
    Aaron Giles) to one byte per pixel, and the prio_transpen sprite blit (src/emu/drawgfxt.ipp
    PIXEL_OP_REMAP_TRANSPEN_PRIORITY) for 8-bit priority buffers.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_GFXLAYOUT_H
#define RBX_GFXLAYOUT_H

#include <stdint.h>

typedef struct {
	int width, height, planes;
	const uint32_t *planeoffs, *xoffs, *yoffs;   // bit offsets as in MAME (bit 0 = MSB of byte 0)
	uint32_t charincrement;                      // bits per element
} rbx_gfx_layout;

typedef struct {
	uint8_t *pix;      // width * height bytes per element
	uint8_t *used;     // 1 = element has a non-zero pixel
	uint32_t count;
	int width, height;
} rbx_gfx;

// decode every element of rom (RGN_FRAC(1,1)); 0 = ok
int rbx_gfx_decode(rbx_gfx *g, const rbx_gfx_layout *l, const uint8_t *rom, uint32_t size);
void rbx_gfx_free(rbx_gfx *g);

// MAME prio_transpen with transparent pen 0: a pixel is drawn where ((1 << pri) & (pmask | 1 << 31)) == 0 and pri becomes 31.
// Clipped to [0, bw) x [0, bh); bitmap and priority buffer share pitch
void rbx_gfx_prio_transpen(const rbx_gfx *g, uint32_t code, uint16_t color_base, int flipx, int flipy, int sx, int sy,
                           uint16_t *bm, uint8_t *pri, int pitch, int bw, int bh, uint32_t pmask);

#endif
