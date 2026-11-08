/*  Data East MXC06 sprite generator (Vapor Trail, Mad Motor, the dec0 / dec8 boards ...), after MAME
    src/mame/dataeast/decmxc06.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail, David Haywood).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_DECMXC06_H
#define RBX_DECMXC06_H
#include <stdint.h>
#include "../../video/gfxlayout.h"

typedef struct {
	const rbx_gfx *gfx;        // 16x16 sprite elements
	int color_base;            // pens: color_base + colour * 16 + pen
	int flip_screen;
	// MAME colpri callback (NULL: none, plain drawing in list order): the colour and the pdrawgfx mask of a colour
	void (*colpri_cb)(void *user, uint32_t *colour, uint32_t *pri_mask);
	void *user;
} decmxc06_t;

// draw the list (size 16-bit words) into bm / pri (pitch shared): screen line top at bitmap row 0, clipped to bw x bh
void decmxc06_draw(const decmxc06_t *s, const uint16_t *spriteram, int size, uint16_t *bm, uint8_t *pri, int pitch,
                   int bw, int bh, int top, uint32_t frame);
#endif
