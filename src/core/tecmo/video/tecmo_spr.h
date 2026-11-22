/*  Tecmo's sprite chip (the Ninja Gaiden format). Ported from MAME src/mame/shared/tecmo_spr.cpp (BSD-3-Clause,
    copyright-holders: David Haywood).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_TECMO_SPR_H
#define RBX_TECMO_SPR_H

#include <stdint.h>

#include "../../video/gfxlayout.h"

// gaiden_draw_sprites into a bitmap of the visible area (w x h, its first line the screen's line y0): raw pens with the
// attribute's blend and priority bits (pen + 16 * (color | attributes & 0x3f0)), the bitmap cleared first
void tecmo_spr_gaiden_draw(const rbx_gfx *gfx, const uint16_t *spriteram, int sprite_sizey, int spr_offset_y, int flip,
                           int screen_w, uint16_t *bmp, int w, int h, int y0);

#endif
