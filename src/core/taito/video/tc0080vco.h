/*  Taito TC0080VCO (O / H System and Air System: two 16x16 tilemaps with zoom, 8x8 RAM text layer, chained zoomed
    sprites), ported from MAME src/mame/taito/tc0080vco.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria).
    Screen flip (control register 0, bits 10-11) follows MAME.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_TC0080VCO_H
#define RBX_TC0080VCO_H

#include <stdint.h>

#define VCO_RAM_WORDS (0x21000 / 2)

// word offsets in the chip RAM
#define VCO_SPRITES 0x10200
#define VCO_SCROLL  0x10400

typedef struct {
	uint16_t ram[VCO_RAM_WORDS];   // host-order words (mapped straight on the 68000 bus)
	int bg_xoffs, bg_yoffs, bg_flip_yoffs;
	int screen_w, screen_h;        // MAME screen size (flipped tilemap scroll), 1024 x 1024 by default
	// decoded 16x16 tiles, one byte per pixel; per tile and per tile row: 0 = empty, 1 = some pixels, 2 = all pixels set
	uint8_t *tiles, *opaque, *rowflags;
	uint32_t ntiles;
	int sprite_term;               // first sprite of the second pass (taito_o draw_sprites)
	// drawing scratch: text characters decoded from char RAM at each vco_draw_tx (8 bytes per row, row flags)
	uint8_t txpix[256 * 64];
	uint8_t txrow[256 * 8];
} tc0080vco_t;

// one sprite as MAME tc0080vco_device::get_sprite_params leaves it
typedef struct {
	uint32_t tile_offs;
	int x0, y0, zoomx, zoomy, ysize;
	int dx, dy, zx, zy;
} vco_sprite_t;

int vco_init(tc0080vco_t *v, const uint8_t *gfx, uint32_t size, int bg_xoffs, int bg_yoffs, int bg_flip_yoffs);
void vco_exit(tc0080vco_t *v);
void vco_reset(tc0080vco_t *v);
// a drawing copy (snapshot) of v: configuration and the decoded tiles are shared, RAM is copied by the board
void vco_share(tc0080vco_t *snap, const tc0080vco_t *v);
static inline int vco_flipscreen(const tc0080vco_t *v) { return (v->ram[VCO_SCROLL] & 0x0c00) != 0; }

// layers into an indexed bitmap; (x0, y0) = screen position of bitmap pixel 0, the bitmap (w x h) is the visible
// area. Colours: tiles / sprites colour * 16, text 512 + pen
void vco_draw_bg0(tc0080vco_t *v, uint16_t *bm, int pitch, int w, int h, int x0, int y0, int opaque);
void vco_draw_bg1(tc0080vco_t *v, uint16_t *bm, int pitch, int w, int h, int x0, int y0);
void vco_draw_tx(tc0080vco_t *v, uint16_t *bm, int pitch, int w, int h, int x0, int y0);
// sprites from offs down to 0 (word offset in sprite RAM, 4 words each); stops at a terminator (taito_o / taitoair) and
// returns the next start (0 at the end)
int vco_draw_sprites(tc0080vco_t *v, uint16_t *bm, int pitch, int w, int h, int x0, int y0, int offs, int zoomy_enable,
                     int stop_at_terminator);
// single sprites for the board's own priority loops (MAME get_sprite_params / draw_single_sprite); params returns
// tile_offs (0 = no sprite)
uint32_t vco_sprite_params(const tc0080vco_t *v, int offs, int zoomy_enable, vco_sprite_t *s);
void vco_sprite_draw(const tc0080vco_t *v, uint16_t *bm, int pitch, int w, int h, int x0, int y0, const vco_sprite_t *s);

#endif
