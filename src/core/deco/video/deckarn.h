/*  Data East "Karnov style" sprites (Karnov, Chelnov, the dec8 boards), after MAME src/mame/dataeast/deckarn.cpp
    (BSD-3-Clause, copyright-holders: Bryan McPhail): four words a sprite, 16x16 or 16x32, drawn from the list's start
    (or, with a colour / priority callback, from its end against a priority buffer).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_DECKARN_H
#define RBX_DECKARN_H
#include <stdint.h>
#include "../../video/gfxlayout.h"

// draw size words of the list into bm (row 0 = screen line top, clipped to bw x bh), pens color_base + colour * 16 + pen
void deckarn_draw(const rbx_gfx *gfx, int color_base, const uint16_t *spriteram, int size, int flip, uint16_t *bm,
                  int pitch, int bw, int bh, int top);
// with MAME's colpri callback (the list drawn from its end, prio_transpen against pri: bw x bh, pitch bw)
void deckarn_draw_pri(const rbx_gfx *gfx, int color_base, const uint16_t *spriteram, int size, int flip, uint16_t *bm,
                      uint8_t *pri, void (*colpri)(uint32_t *colour, uint32_t *pri_mask), int pitch, int bw, int bh,
                      int top);
#endif
