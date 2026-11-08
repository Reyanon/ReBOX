/*  Seta X1-001 sprite generator (Taito X System, tnzs family), ported from MAME
    src/devices/video/x1_001.cpp (BSD-3-Clause, copyright-holders: Luca Elia, David Haywood).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_X1_001_H
#define RBX_X1_001_H

#include <stdint.h>

typedef struct {
	uint8_t ctrl[4];
	uint8_t ylow[0x300];        // 0x200 sprite Y + 0x100 background column scroll
	uint16_t code[0x2000];      // code / attr words
	uint8_t bgflag;
	// configuration (MAME set_*)
	int fg_flipxoffs, fg_noflipxoffs, fg_flipyoffs, fg_noflipyoffs;
	int bg_flipxoffs, bg_noflipxoffs, bg_flipyoffs, bg_noflipyoffs;
	int colorbase, spritelimit, transpen, colors;
	int gfx_colorbase;          // the gfx entry's color base in color codes (both layers); 0 on most boards
	int setac_gfxbank;          // seta.cpp's setac_gfxbank_callback: bits 9-10 of the X word bank the code
	// decoded 16x16 tiles, one byte per pixel
	uint8_t *tiles, *opaque;
	uint32_t ntiles;
} x1_001_t;

// gfx: MAME "gfx1" region in the Taito X layout (planes 0,8,16,24 of a 32-bit row)
int x1_init(x1_001_t *x, const uint8_t *gfx, uint32_t size, int colors);
// other boards: a MAME gfx_layout of 16x16 4bpp tiles (bit offsets; planes[0] = most significant)
typedef struct {
	uint32_t total;
	uint32_t planes[4];
	uint32_t xoffs[16], yoffs[16];
	uint32_t charincrement;
} x1_layout;
int x1_init_layout(x1_001_t *x, const uint8_t *gfx, uint32_t size, int colors, const x1_layout *l);
void x1_exit(x1_001_t *x);
void x1_reset(x1_001_t *x);
// draw into an indexed bitmap; (x0, y0) = screen position of bitmap pixel 0, screen_h = total screen height (MAME max_y)
void x1_draw(x1_001_t *x, uint16_t *bitmap, int pitch, int w, int h, int x0, int y0, int screen_h, int vis_max_y, int bank_size);
void x1_tnzs_eof(x1_001_t *x);
void x1_setac_eof(x1_001_t *x);

#endif
