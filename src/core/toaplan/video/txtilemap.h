/*  Toaplan text tilemap (paired with the GP9001). Ported from MAME src/mame/toaplan/toaplan_txtilemap.cpp
    (BSD-3-Clause, copyright-holders: Quench, David Haywood). 64x32 tiles of 8x8, a line select and an x scroll for
    every line.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_TOAPLAN_TXTILEMAP_H
#define RBX_TOAPLAN_TXTILEMAP_H

#include <stdint.h>

typedef struct {
	uint16_t vram[0x1000];
	uint16_t linescroll[0x100];
	uint16_t lineselect[0x800];
	uint8_t state_end;
	// configuration
	int dx, dy, dx_flipped, dy_flipped;
	int color_base;
	int screen_w;               // (the screen's whole width, for the flipped scroll)
	// the pen of pixel x, y (0..7) of tile code
	uint8_t (*pix)(void *user, uint32_t code, int x, int y);
	void *user;
} txtilemap_t;

#define TXTILEMAP_STATE_SIZE ((uint32_t)__builtin_offsetof(txtilemap_t, state_end))

// over lines y0 .. y1 - 1 of bm (pen 0 transparent)
void txtilemap_draw(const txtilemap_t *t, uint16_t *bm, int pitch, int w, int y0, int y1);

#endif
