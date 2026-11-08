/*  Data East BAC06 tilemap chip (one playfield of 8x8 or 16x16 tiles in three shapes, row and column scroll), after
    MAME src/mame/dataeast/decbac06.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail, David Haywood).
    The drawing is MAME's custom_tilemap_draw per pixel over the tilemap's picture (its tiles looked up as they are
    read; the whole picture turned round with the flip screen, as MAME's tilemap flip).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_BAC06_H
#define RBX_BAC06_H
#include <stdint.h>
#include "../../video/gfxlayout.h"

typedef struct {
	uint16_t vram[0x2000];        // (two banks of 0x1000 words: ctrlreg 2 bit 0)
	uint16_t rowscroll[0x1000], colscroll[0x1000];
	uint16_t ctrlreg[4], scrollreg[8];
	int rambank, flip_screen;
	// configuration (MAME set_gfx_region_wide, disable_*, set_thedeep_kludge, set_tile_callback)
	const rbx_gfx *gfx8, *gfx16;
	int base8, base16;            // the gfxdecode entries' palette base
	int gran8, gran16;            // their colour granularity as a shift (0: 4, 16 pens; 3: 8 pens of a 3 bpp layout)
	int wide;
	int no_8x8, no_16x16, no_rc_scroll, thedeep_kludge;
	// MAME draw_bootleg (Pocket Gal's bootleg): 1 + the shape of an 8x8 map drawn without the control and scroll
	// registers (0: off)
	int bootleg8;
	// (flags: bit 0 TILE_FLIPX, BAC06_GROUP1 the tile's transmask group)
	void (*tile_cb)(uint32_t *tile, uint32_t *colour, uint32_t *flags);
	// MAME set_transmask(group, fgmask, bgmask): the pens not in the foreground (LAYER0) / background (LAYER1) of a
	// group's tiles (transmask_on 0: transparent pen 0, every other pen in LAYER0)
	int transmask_on;
	uint16_t fgmask[2], bgmask[2];
} bac06_t;

#define BAC06_GROUP1 0x10
// drawing flags (MAME TILEMAP_DRAW_*): 0 = LAYER0
enum { BAC06_OPAQUE = 1, BAC06_LAYER0 = 2, BAC06_LAYER1 = 4 };

void bac06_reset(bac06_t *b);
// ctrlreg_w / scrollreg_w (16-bit, offsets in words)
void bac06_ctrl_w(bac06_t *b, int offset, uint16_t data, uint16_t mask);
void bac06_scroll_w(bac06_t *b, int offset, uint16_t data, uint16_t mask);
// vram_r / vram_w (the bank applied)
uint16_t bac06_vram_r(const bac06_t *b, int offset);
void bac06_vram_w(bac06_t *b, int offset, uint16_t data, uint16_t mask);
// MAME draw: screen lines y0 .. y1 - 1 of the bitmap (row 0 = screen line top), columns 0 .. width - 1, the pixels of
// the layers in flags (BAC06_*; 0: LAYER0); pri[x] = (pri[x] & pmask) | priority where drawn (pri may be NULL)
void bac06_draw(const bac06_t *b, uint16_t *bm, uint8_t *pri, int pitch, int width, int top, int y0, int y1,
                int flags, uint8_t priority, uint8_t pmask);
#endif
