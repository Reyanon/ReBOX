/*  Taito TC0630FDP display processor (F3 system): 4 playfields with line RAM effects (zoom, row/column scroll,
    clip planes, mosaic, palette add, alternate tilemaps), the text (VRAM) and pivot (pixel) layer, zoomed sprites in
    4 priority groups and the TC0650FDA blending; ported from MAME src/mame/taito/taito_f3_v.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail, ywy, 12Me21).
    Output is a palette-index bitmap: the 0x2000 F3 colours, then the blended colours of the frame (allocated per
    frame after index 0x2000).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_TC0630FDP_H
#define RBX_TC0630FDP_H

#include <stdint.h>

#define F3V_PAL_COLORS 0x2000
#define F3V_PALETTE_SIZE 0x10000

typedef struct {
	int code;
	uint8_t color, flip_x, flip_y, pri;
	int32_t x, y, scale_x, scale_y;
} f3v_sprite;

typedef struct {
	// configuration (set before f3v_init)
	int extend;          // 1024x512 playfields (MAME F3config.extend)
	int sprite_lag;      // 0..2
	int pal12;           // 12-bit palette games
	int vis_y0, vis_y1;  // visible lines (screen coordinates, 0..255)
	// decoded graphics: 16x16 tiles of 6-bit pixels (1 byte per pixel)
	uint8_t *spr_gfx, *tile_gfx;
	uint16_t *tile_blank;  // per tile: bit y set = row y has no pixel (any pen mask makes it transparent)
	uint32_t spr_count, tile_count;

	// snapshot (the board copies main CPU state here at vblank; host 16-bit words)
	uint16_t ctrl0[8], ctrl1[8];
	uint16_t spriteram[0x8000];
	uint16_t vram[0x8000];      // 0x610000-0x61ffff: playfields 0x0000, text 0x6000, characters 0x7000 (words)
	uint16_t lineram[0x8000];
	uint16_t pivotram[0x8000];
	uint16_t palram[0x4000];    // 0x2000 32-bit entries, high word first

	// drawing side state
	int flipscreen, sprite_bank, sprite_trails;
	int sflip;                  // flipscreen as the sprite side has it (the lines take it when they are done)
	uint8_t pen_mask;
	f3v_sprite sprites[0x400];
	int nsprites;
	// sprite frame buffers, 256 x 512, and the priorities each row has: the lines read fb / sprite_row_usage, the
	// sprites of a later frame go to fb_draw / sru_draw (the other one: drawn while the lines are)
	uint16_t *fb, *fb_draw, *fbs[2];
	uint8_t *sprite_row_usage, *sru_draw;
	uint8_t sru[2][256];
	uint32_t pal_rgb[F3V_PAL_COLORS];
	// blend colour cache
	uint32_t *hkey;
	uint16_t *hval, *hgen;
	uint16_t gen;
	int next_color;
} f3v_t;

// ROM graphics: sprites / tilemap regions with their "_hi" planes (MAME tile_decode); NULL hi = 4bpp only
int f3v_init(f3v_t *v, const uint8_t *spr, uint32_t spr_size, const uint8_t *spr_hi, uint32_t spr_hi_size,
	const uint8_t *tile, uint32_t tile_size, const uint8_t *tile_hi, uint32_t tile_hi_size);
void f3v_exit(f3v_t *v);
void f3v_reset(f3v_t *v);
// draw the snapshot: bitmap = 320 x (vis_y1 - vis_y0 + 1) indices into palette (F3V_PALETTE_SIZE entries, 0x00RRGGBB);
// returns the number of palette entries in use
int f3v_render(f3v_t *v, uint16_t *bitmap, int pitch, uint32_t *palette);
// another thread draws lines of the frame f3v_render is drawing now (0 = no frame being drawn); same pixels
int f3v_help(int helper);   // helper 1 or 2, | F3V_HELP_ONE: one piece only
#define F3V_HELP_ONE 0x100

#endif
