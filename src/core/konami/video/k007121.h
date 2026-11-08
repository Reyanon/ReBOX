/*  Konami 007121 video controller: its control registers (the interrupts, the flip, the tile code bits), the scroll
    RAM, the sprite list's buffer and its sprite drawing. Ported from MAME src/mame/konami/k007121.cpp (BSD-3-Clause,
    copyright-holders: Fabio Priuli, Acho A. Tang, R. Belmont); the tilemaps stay in the drivers as in MAME.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_K007121_H
#define RBX_K007121_H

#include <stdint.h>

typedef struct {
	uint8_t ctrl[8];
	uint8_t scroll[0x40];
	uint8_t flipscreen;
	uint8_t sprites_buffer[0x800];
	uint8_t state_end;
	int spr_dx, spr_flip_dx;   // (set_sprite_offsets)
} k007121_t;

// the interrupt lines the control register 7 dropped (k007121_ctrl_w's result)
enum { K007121_IRQ = 1, K007121_FIRQ = 2, K007121_NMI = 4 };

void k007121_init(k007121_t *k, int spr_dx, int spr_flip_dx);
void k007121_reset(k007121_t *k);   // (device_reset: ctrl_w(0-7, 0))
// returns the lines cleared (K007121_*)
int k007121_ctrl_w(k007121_t *k, int offset, uint8_t data);
// the scanline timer (every 16 lines from 0): the lines asserted (K007121_*); at 240 the sprite list buffered from
// spriteram (0x1000 bytes: ctrl 3 bit 3 its second half)
int k007121_scanline(k007121_t *k, int scanline, uint32_t frame, const uint8_t *spriteram);

// sprites_draw: the sprite list from the buffer, 8x8 elements of gfx (4 bpp, one byte per pixel), pens gfx_base +
// color * 16 + pixel, a pixel drawn unless its bit in transmask[color] is set; cb(code, color, colbank) as MAME's sprite
// callback (NULL: none). The bitmap's origin is the screen's (x0, y0). pri NULL: pri_mask ~0 (no priority bitmap);
// else prio_transmask over the priority buffer (same pitch) with pmask, the list drawn backwards
typedef void (*k007121_sprite_cb_t)(int *code, int *color, int colbank);
void k007121_sprites_draw(const k007121_t *k, const uint8_t *gfx, uint32_t gfx_count, int gfx_base,
                          const uint16_t *transmask, k007121_sprite_cb_t cb, uint16_t *bm, int pitch, int x0, int y0,
                          int cx0, int cy0, int cx1, int cy1, uint8_t *pri, uint32_t pmask);

#endif
