/*  Toaplan GP9001 VDP. Ported from MAME src/mame/toaplan/gp9001.cpp (BSD-3-Clause, copyright-holders: Quench, David
    Haywood). Three 512x512 tilemaps of 16x16 tiles, 0x100 sprites of 8x8 pieces (the list buffered a frame), 16
    priority levels against a per-pixel priority buffer (the tilemaps' folded to even levels).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_GP9001_H
#define RBX_GP9001_H

#include <stdint.h>

typedef struct {
	uint16_t vram[3][0x800];      // the tilemaps (attribute, code word pairs)
	uint16_t spr[0x400];          // live sprite RAM
	uint16_t spr_buf[0x400];      // the list shown (copied at the vblank: screen_eof)
	uint16_t voffs, scroll_reg;
	uint16_t tm_scrollx[3], tm_scrolly[3], tm_flip[3];   // (flip: 1 x, 2 y)
	uint16_t sp_scrollx, sp_scrolly, sp_flip;
	uint8_t vint;                 // the IRQ line
	uint8_t pad;
	uint8_t state_end;
	// configuration (not saved)
	int no_sprite_buffer;
	int tm_xoff[3][2], tm_yoff[3][2], sp_xoff[2], sp_yoff[2];   // extra offsets: [normal, flipped]
	const uint8_t *gfx;           // 8x8 tiles, 4 bpp, 32 bytes each, the even pixel in the low nibble
	uint32_t tiles;               // 8x8 tiles
	// the tile / sprite code bank callback (batrider's object bank): layer 0..2, 3 sprites; code of 8x8 tiles
	uint32_t (*tile_cb)(void *user, int layer, uint32_t code);
	void *cb_user;
} gp9001_t;

#define GP9001_STATE_SIZE ((uint32_t)__builtin_offsetof(gp9001_t, state_end))

// gfx: the region of the two halves (planes), decoded in place to the format above; 0 = ok
int gp9001_init(gp9001_t *v, uint8_t *region, uint32_t size);
void gp9001_reset(gp9001_t *v);
uint16_t gp9001_read(gp9001_t *v, int offset, int vpos);           // word offset 0 .. 7
void gp9001_write(gp9001_t *v, int offset, uint16_t data, uint16_t mask);
uint16_t gp9001_vdpcount(int vpos, int hpos);                        // (vdpcount_r)
void gp9001_eof(gp9001_t *v);                                        // the vblank's start (screen_eof)
// bootleg access
void gp9001_bootleg_scroll_w(gp9001_t *v, int offset, uint16_t data, uint16_t mask, const int *offs);

// drawing lines y0 .. y1 - 1 of a snapshot: the 0x7ff palette indices into bm (pitch in pixels), pri: the priority
// buffer of those lines (w x (y1 - y0), cleared here); the bitmap's lines filled with 0 first when clear
void gp9001_draw(const gp9001_t *v, uint16_t *bm, int pitch, uint8_t *pri, int w, int y0, int y1, int clear);

#endif
