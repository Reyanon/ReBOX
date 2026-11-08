/*  Data East sprite chip 52 (DECO 52), after MAME src/mame/shared/decospr.cpp (BSD-3-Clause, copyright-holders:
    David Haywood): the standard and the alternate list formats, drawn with a priority callback (MAME pdrawgfx: the
    list from its end) or without (plain transparent drawing from its start). The raw sprite bitmap mode (boards that
    mix sprites themselves) comes with the boards that use it.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_DECOSPR_H
#define RBX_DECOSPR_H
#include <stdint.h>
#include "../../video/gfxlayout.h"

typedef struct {
	const rbx_gfx *gfx;        // 16x16 sprite elements
	int color_base;            // the gfxdecode entry's palette base (pens: color_base + colour * 16 + pen)
	int alt_format;
	int flip_screen;
	int x_offset, y_offset, flipallx;
	// MAME DECOSPR_PRIORITY_CB (NULL: none): the pdrawgfx mask of a sprite from its x word (alternate format: word 2's
	// low byte) and its extra priority bit
	uint32_t (*pri_cb)(uint16_t x, int extpri);
	// MAME DECOSPR_COLOUR_CB (NULL: (x >> 9) & 0x1f)
	int (*col_cb)(uint16_t x, int extpri);
	// MAME alloc_sprite_bitmap (the raw mode: the board mixes): pens | colour << raw_shift into the board's bitmap
	// (bm of decospr_draw, filled with 0 first by the board), colour (x >> 9) & 0x7f | 0x80 with the extra priority bit
	int raw, raw_shift;
	int gran;                  // the colour's pens (the gfxdecode entry's granularity: 64 for 6 bpp; 0 = 16)
	int bootleg;               // MAME set_is_bootleg: the code is not aligned to the height (sprite &= ~multi skipped)
	int transpen;              // MAME set_transpen (plain drawing only; the priority and raw paths use pen 0)
} decospr_t;

// draw the list (sizewords 16-bit words) into bm / pri (pitch shared), bitmap row 0 = screen line top, clip to the
// bitmap (bw x bh; MAME's cliprect: the visible area). wide: the cliprect reaches past x 256 (MAME's 320-wide
// placement). frame: the screen's frame number (flashing sprites)
void decospr_draw(const decospr_t *s, const uint16_t *spriteram, int sizewords, uint16_t *bm, uint8_t *pri, int pitch,
                  int bw, int bh, int top, int wide, uint32_t frame);
#endif
