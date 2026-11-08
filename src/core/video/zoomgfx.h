/*  Zoomed sprite drawing of MAME gfx_element::zoom_transpen / prio_zoom_transpen (src/emu/drawgfx.cpp, drawgfxt.ipp,
    BSD-3-Clause, copyright-holders: Nicola Salmoria, Aaron Giles) for graphics decoded to one byte per pixel, and the
    decoders of the sprite layouts these boards use. Shared by othunder, topspeed, slapshot and wgp.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_ZOOMGFX_H
#define RBX_ZOOMGFX_H

#include <stdint.h>

typedef struct {
	uint8_t *pix;        // w * h bytes per element
	uint8_t *opaque;     // per element: 0 = all pixels transparent (pen 0)
	uint32_t count;
	int w, h;
} zoomgfx_t;

enum {
	ZGFX_16x8_PLANAR64,     // MAME tile16x8_layout: planes at bits 0/16/32/48 of each 64-bit row (othunder, topspeed)
	ZGFX_16x16_PACKED_LSB,  // gfx_16x16x4_packed_lsb (wgp, slapshot low bits)
};

int zoomgfx_decode(zoomgfx_t *g, const uint8_t *src, uint32_t size, int layout);
void zoomgfx_free(zoomgfx_t *g);

// target: bitmap rows are screen lines y0..y0+h-1 (sx 0..w-1); pri (may be NULL) has the same pitch as the bitmap
typedef struct {
	uint16_t *bm;
	uint8_t *pri;
	int pitch;
	int x0, y0, x1, y1;   // clip, screen coordinates (inclusive)
	int sy0;              // screen line of bitmap row 0
	int ppitch;           // pitch of pri (0 = pitch)
} zoomgfx_target;

// draw element code at (sx, sy) scaled by scalex / scaley (16.16), pen 0 transparent, color = palette base.
// pmask: MAME prio_zoom_transpen (pixel hidden where (1 << pri) & pmask, pri set to 31); use_pri 0 = zoom_transpen
void zoomgfx_draw(const zoomgfx_t *g, const zoomgfx_target *t, uint32_t code, uint32_t color, int flipx, int flipy,
                  int sx, int sy, uint32_t scalex, uint32_t scaley, int use_pri, uint32_t pmask);

#endif
