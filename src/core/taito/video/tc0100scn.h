/*  Taito TC0100SCN tilemap generator, ported from MAME src/mame/taito/tc0100scn.cpp (BSD-3-Clause,
    copyright-holders: Nicola Salmoria). Two 8x8 ROM tile layers with rowscroll (BG1 also columnscroll) and a
    text layer whose 2bpp characters live in RAM; standard (64x64) and double width (128x64) memory layouts.
    Used by Taito F2, Z, asuka, ninjaw, warriorb and others.
    Drawing follows MAME's tilemap placement (scrolldx / scrolldy, flip screen around the visible area) on a
    screen of MAME coordinates: bitmap row r is screen line sy0 + r, 320 pixels wide.
    The chip RAM is plain host-order words and can be mapped straight onto the 68000 bus; drawing reads it
    directly (no tile cache), so a snapshot is a copy of ram + ctrl (tc0100scn_copy).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_TC0100SCN_H
#define RBX_TC0100SCN_H

#include <stdint.h>

#define TC0100SCN_RAM_BYTES 0x14000
#define TC0100SCN_SINGLE_VDU 1024

typedef struct {
	uint16_t ram[TC0100SCN_RAM_BYTES / 2];
	uint16_t ctrl[8];
	uint32_t code_bank;            // OR'ed into BG tile codes after code_mask (mjnquest tile callback)
	uint32_t code_mask;
	// configuration (MAME set_offsets / set_offsets_flip / set_offsets_fliptx / multiscreen)
	int x_offset, y_offset, flip_xoffs, flip_yoffs, flip_text_xoffs, flip_text_yoffs;
	int multiscrn_xoffs, multiscrn_hack;
	int bg_colbank[2], tx_colbank;
	int screen_w, screen_h;
	uint16_t pen_base;             // added to every pixel (palette of this chip's screen), 0 by default        // MAME visible area right+left+1 / bottom+top+1 (flip mirror), 320 x 256 on F2
	// decoded ROM tiles: 8x8, one byte per pixel; tile_used: bit 0 = a non-zero pixel, bit 1 = no pixel 0
	const uint8_t *tiles, *tile_used;
	uint32_t ntiles;
	uint8_t *owned;
} tc0100scn_t;

enum { TC0100SCN_4BPP, TC0100SCN_1BPP };

int tc0100scn_init(tc0100scn_t *c, const uint8_t *gfx, uint32_t size, int layout);
void tc0100scn_exit(tc0100scn_t *c);
void tc0100scn_reset(tc0100scn_t *c);
void tc0100scn_ctrl_w(tc0100scn_t *c, int offset, uint16_t data, uint16_t mask);
static inline uint16_t tc0100scn_ctrl_r(const tc0100scn_t *c, int offset) { return c->ctrl[offset & 7]; }
static inline int tc0100scn_bottomlayer(const tc0100scn_t *c) { return (c->ctrl[6] & 0x8) >> 3; }
// copy the drawing state (RAM, registers, tile bank) into a snapshot that shares the decoded tiles
void tc0100scn_copy(tc0100scn_t *dst, const tc0100scn_t *src);
// draw layer 0 (BG0), 1 (BG1 with colscroll) or 2 (text), pen 0 transparent; pri |= priority where drawn.
// bitmap row r is MAME screen line sy0 + r; 320 pixels per row. Returns 1 if the layer is disabled
int tc0100scn_draw(const tc0100scn_t *c, int layer, uint16_t *bm, uint8_t *pri, int pitch, int sy0, int rows,
                   uint8_t priority);
// tc0100scn_draw with flags: TC0100SCN_OPAQUE = every pixel including pen 0 (MAME TILEMAP_DRAW_OPAQUE)
#define TC0100SCN_OPAQUE 1
int tc0100scn_draw_ex(const tc0100scn_t *c, int layer, uint16_t *bm, uint8_t *pri, int pitch, int sy0, int rows,
                      uint8_t priority, int flags);

#endif
