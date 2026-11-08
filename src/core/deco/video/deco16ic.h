/*  Data East tilemap chips 55 / 56 / 74 / 141 (two playfields of 8x8 or 16x16 tiles, row and column scroll), after
    MAME src/mame/dataeast/deco16ic.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail).
    The drawing is MAME's custom_tilemap_draw (its "reference rasterizer"), which gives the same pixels as its tilemap
    path for row scroll alone and column scroll alone, so one path serves every mode. Graphics bank, colour bank and
    mask, the tile and bank callbacks are the board's (as the MAME device's configuration).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_DECO16IC_H
#define RBX_DECO16IC_H
#include <stdint.h>
#include "../../video/gfxlayout.h"

enum { DECO_32x32 = 0, DECO_64x32 = 1, DECO_32x64 = 2, DECO_64x64 = 3 };

// one screen line's scroll, control and bank of both playfields (deco16ic_latch_line: raster effects)
typedef struct {
	uint16_t scrollx[2], scrolly[2];
	uint8_t control0[2], control1[2];
	int bank[2];
} deco16ic_line_t;

typedef struct {
	uint16_t vram[2][0x1000];     // the playfields' tile words (MAME m_vram[0..1])
	uint16_t control[8];
	struct {
		uint16_t scrollx, scrolly;
		uint8_t control0, control1, bankval;
		int bank;                 // added to the tile code (bank_cb(bankval))
		// configuration
		int size;                 // DECO_*
		int col_bank, col_mask;
		int (*bank_cb)(int bankval);
	} tm[2];
	// configuration: the decoded graphics of the 8x8 and the 16x16 tiles (the board's gfxdecode entries), their
	// colour granularity as a shift (4: 16 pens, 8: 256 pens of an 8 bpp layout; 0 = 4)
	const rbx_gfx *gfx8, *gfx16;
	int gran8, gran16;
	int base8, base16;        // the gfxdecode entries' palette base (pens: base + colour << gran + pen)
	// MAME set_scrolldx (the same dx for both playfields and sizes): taken from the x scroll where MAME draws with its
	// tilemaps (not both row and column scroll on: its custom renderer has none), as MAME's tilemap does (screen x
	// shows tilemap x + scroll - dx); dx: one playfield's one size more ([playfield][0: 16x16, 1: 8x8])
	int scrolldx;
	int dx[2][2];
	// MAME's transmask categories (TILEMAP_DRAW_LAYER0 / 1): the pens a transparent drawing draws (bit n: pen n;
	// 0 = every pen but 0). Set on the drawing's copy of the chip
	uint32_t draw_pens;
	// MAME set_tile_callback (NULL: none): tile and colour of a tile word, before the bank and the colour mask / bank
	void (*tile_cb)(uint32_t *tile, uint32_t *colour, int layer, int is8x8);
	// the lines' scroll and control as latched while the frame ran (lines_on: the board latches them, else the
	// registers as they are at the drawing)
	int lines_on;
	deco16ic_line_t lines[256];
} deco16ic_t;

void deco16ic_reset(deco16ic_t *d);
void deco16ic_control_w(deco16ic_t *d, int offset, uint16_t data, uint16_t mem_mask);
uint16_t deco16ic_control_r(const deco16ic_t *d, int offset);
// MAME update(): the tile banks from the bank callbacks (call once a frame before drawing, as screen_update does)
void deco16ic_update(deco16ic_t *d);
// the registers of screen line y (as MAME's update_partial on a control write draws the lines before it with the old
// ones): call at each line's start; the drawing then takes each line's own (lines_on set)
void deco16ic_latch_line(deco16ic_t *d, int y);
// MAME tilemap_12_combine_draw: the two playfields' 16x16 pixels as one 8 bpp picture (playfield 0's scroll, control
// and rowscroll; pixels where either pen is not 0, or all with opaque), mix as MAME's mix callback (NULL:
// p | (p2 & 0xf) << 4; p with the 16x16 entry's base)
void deco16ic_draw_combine(const deco16ic_t *d, const uint16_t *rowscroll, uint16_t *bm, uint8_t *pri, int pitch,
                           int width, int top, int y0, int y1, int opaque, uint8_t priority,
                           uint16_t (*mix)(uint16_t p, uint16_t p2));
// draw playfield which (0: MAME tilemap_1, 1: tilemap_2) on screen lines y0 .. y1 - 1 (screen coordinates: bitmap row 0
// is screen line top) and columns 0 .. width - 1; rowscroll: the board's rowscroll RAM for it (NULL: none). Pens with
// colour * 16 + pen; pen 0 left alone unless opaque. pri[x] = (pri[x] & pmask) | priority where drawn (pri may be NULL)
void deco16ic_draw(const deco16ic_t *d, int which, const uint16_t *rowscroll, uint16_t *bm, uint8_t *pri, int pitch,
                   int width, int top, int y0, int y1, int opaque, uint8_t priority, uint8_t pmask);
#endif
