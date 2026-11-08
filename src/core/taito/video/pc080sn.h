/*  Taito PC080SN tilemap chip (two 512x512 layers of 8x8 tiles with per-line X scroll), ported from MAME
    src/mame/taito/pc080sn.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria). Used by Rastan, Rainbow Islands,
    Operation Wolf, Top Speed, Asuka & Asuka ... Tiles are decoded at init to one byte per pixel.
    Drawing reads only a pc080sn_regs snapshot, so it can run on the render thread.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_PC080SN_H
#define RBX_PC080SN_H

#include <stdint.h>

typedef struct {
	uint16_t ram[0x8000];   // 64 KB: bg0 tiles, bg0 line scroll, bg1 tiles, bg1 line scroll
	uint16_t ctrl[8];       // 0-1 x scroll, 2-3 y scroll, 4 control (bit 0 flip screen)
} pc080sn_regs;

enum { PC080SN_GFX_PACKED_MSB, PC080SN_GFX_PLANAR };   // gfx_8x8x4_packed_msb / jumping_tilelayout

typedef struct {
	pc080sn_regs r;
	// configuration (MAME set_offsets / set_yinvert / set_dblwidth / set_trans_pen / set_scroll)
	int x_offset, y_offset, y_invert, dblwidth;
	int trans_pen[2];
	int fixed[2], fixed_x[2], fixed_y[2];   // layer scroll forced every frame (jumping)
	int screen_w, screen_h;                 // MAME screen size (flip screen), 320 x 256
	int colors;                             // gfx color count (MAME GFXDECODE_ENTRY), 0x80
	uint8_t *tiles;                         // 64 bytes per tile
	uint8_t *tflags;                        // bit 0: all pixels 0, bit 1: all pixels 15, bit 2: no pixel 0, bit 3: no 15
	uint32_t ntiles;
} pc080sn_t;

int pc080sn_init(pc080sn_t *c, const uint8_t *gfx, uint32_t size, int layout);   // 0 = ok
void pc080sn_exit(pc080sn_t *c);
void pc080sn_reset(pc080sn_t *c);
static inline uint16_t pc080sn_word_r(pc080sn_t *c, uint32_t offset) { return c->r.ram[offset & 0x7fff]; }
static inline void pc080sn_word_w(pc080sn_t *c, uint32_t offset, uint16_t data, uint16_t mask)
{
	uint16_t *p = &c->r.ram[offset & 0x7fff];
	*p = (uint16_t)((*p & ~mask) | (data & mask));
}
void pc080sn_xscroll_w(pc080sn_t *c, uint32_t offset, uint16_t data, uint16_t mask);
void pc080sn_yscroll_w(pc080sn_t *c, uint32_t offset, uint16_t data, uint16_t mask);
void pc080sn_ctrl_w(pc080sn_t *c, uint32_t offset, uint16_t data, uint16_t mask);

// draw a layer from snapshot s into bm (pitch in pixels), lines y0..y0+h-1 of the screen, x 0..w-1.
// opaque: every pixel (MAME TILEMAP_DRAW_OPAQUE); pri: priority buffer (w x h, pitch w) written with prival
// where the layer draws (may be NULL)
void pc080sn_draw(const pc080sn_t *c, const pc080sn_regs *s, int layer, uint16_t *bm, int pitch, uint8_t *pri,
                  int w, int y0, int h, int opaque, uint8_t prival);

#endif
