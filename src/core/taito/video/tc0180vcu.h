/*  Taito TC0180VCU (B System tilemaps, sprites and pixel frame buffer), ported from MAME
    src/mame/taito/tc0180vcu.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria; sprite/frame buffer
    emulation by Jarek Burczynski) and the screen update of taito_b_v.cpp (GPL-2.0+, Jarek Burczynski).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_TC0180VCU_H
#define RBX_TC0180VCU_H

#include <stdint.h>

#define VCU_RAM_BYTES 0x14000          // 0x00000-0x13fff: tile RAM, sprite RAM, scroll RAM
#define VCU_FB_W 512
#define VCU_FB_H 256

typedef struct {
	uint8_t ram[VCU_RAM_BYTES];        // host-order 16-bit words (mapped straight on the 68000 bus)
	uint16_t ctrl[16];
	uint16_t *fb[2];                   // sprite frame buffer pages (shared with snapshots, see vcu_vblank_flip)
	int fb_page;
	uint8_t video_control;
	int bg_bank[2], fg_bank[2], tx_bank;
	int fb_color_base, bg_color_base, fg_color_base, tx_color_base;
	// decoded graphics: 8x8 chars and 16x16 tiles, one byte per pixel
	uint8_t *chars, *tiles;
	uint8_t *char_opaque, *tile_opaque;   // 0 = all transparent, 1 = mixed, 2 = all opaque
	uint32_t nchars, ntiles;
} tc0180vcu_t;

int vcu_init(tc0180vcu_t *v, const uint8_t *gfx, uint32_t size, int fb_base, int bg_base, int fg_base, int tx_base);
void vcu_exit(tc0180vcu_t *v);
void vcu_reset(tc0180vcu_t *v);
void vcu_ctrl_w(tc0180vcu_t *v, int offset, uint16_t data, uint16_t mask);
uint16_t vcu_ctrl_r(tc0180vcu_t *v, int offset);
uint16_t vcu_fb_r(tc0180vcu_t *v, uint32_t offset);                    // word offset in 0x40000-0x7ffff
void vcu_fb_w(tc0180vcu_t *v, uint32_t offset, uint16_t data, uint16_t mask);
void vcu_vblank(tc0180vcu_t *v);          // at vblank start: clear / flip the frame buffer, draw sprites
// the same in two halves, for drawing from a snapshot on another thread: the main side flips the page (returns the page
// to draw and whether to clear it), the drawing side clears it and draws the sprites of its snapshot's sprite RAM
void vcu_vblank_flip(tc0180vcu_t *v, int *draw_page, int *clear);
void vcu_draw_sprites(tc0180vcu_t *v, int draw_page, int clear);
// screen update into indexed bitmap (visible area 320x224 = screen lines 16..239)
void vcu_render(tc0180vcu_t *v, uint16_t *bitmap, int pitch);
// pieces of the screen update for boards with extra layers (hitice)
void vcu_draw_bg(tc0180vcu_t *v, uint16_t *bitmap, int pitch);
void vcu_draw_fb(tc0180vcu_t *v, uint16_t *bitmap, int pitch, int priority);
void vcu_draw_fg(tc0180vcu_t *v, uint16_t *bitmap, int pitch);
void vcu_draw_tx(tc0180vcu_t *v, uint16_t *bitmap, int pitch);

static inline uint16_t vcu_word(const tc0180vcu_t *v, uint32_t word) { return ((const uint16_t *)v->ram)[word]; }

#endif
