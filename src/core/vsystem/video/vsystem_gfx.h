/*  Video System boards' drawing pieces: graphics decoded to a byte a pixel (MAME gfx_layout, src/emu/digfx.cpp), the
    zoomed sprite blit of MAME gfx_element::prio_zoom_transpen / zoom_transpen (src/emu/drawgfx.cpp, drawgfxt.ipp,
    BSD-3-Clause, copyright-holders: Nicola Salmoria, Aaron Giles) and the 8x8 tile layers of MAME's tilemap_t
    (src/emu/tilemap.cpp: row scroll, the flip around the visible area's centre) as the boards use them.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_VSYSTEM_GFX_H
#define RBX_VSYSTEM_GFX_H

#include <stdint.h>

// a decoded graphics set (MAME gfx_element): w x h bytes an element; use[code]: bit 0 = a pen other than 15, bit 1 = a
// pen 15 (the transparent pen of every layer and sprite of these boards)
typedef struct {
	uint8_t *pix;
	uint8_t *use;
	uint32_t count;
	int w, h;
	uint32_t colorbase, colors;   // the palette entry of colour 0, the colours (color % colors, 16 pens each)
} vsg_t;

// a layout (bit offsets as MAME's, RGN_FRAC already worked out); total elements from `start` (bytes) of the region
typedef struct {
	int w, h, planes;
	uint32_t planeoffs[4];
	uint32_t xoffs[16], yoffs[16];
	uint32_t charinc;
} vsg_layout;

// MAME's gfx_8x8x4_packed_* / gfx_16x16x4_packed_*: w = 8 or 16, msb = the left pixel in the high nibble
int vsg_packed(vsg_t *g, const uint8_t *rom, uint32_t size, int w, int msb, uint32_t colorbase, uint32_t colors);
int vsg_decode(vsg_t *g, const vsg_layout *l, const uint8_t *rom, uint32_t size, uint32_t start, uint32_t total,
               uint32_t colorbase, uint32_t colors);
void vsg_free(vsg_t *g);

// where the drawing goes: screen lines cy0..cy1 and columns cx0..cx1 (the clip, screen coordinates); bitmap row of
// screen line y at bm + (y - y0) * pitch, column x at [x - x0]; pri (the priority bitmap) the same way
typedef struct {
	uint16_t *bm;
	uint8_t *pri;
	int pitch;
	int x0, y0;
	int cx0, cx1, cy0, cy1;
} vsg_target;

// prio_zoom_transpen (use_pri) or zoom_transpen; scale 16.16 (0x10000: prio_transpen / transpen, the same pixels)
void vsg_zoom(const vsg_t *g, const vsg_target *t, uint32_t code, uint32_t color, int flipx, int flipy, int sx, int sy,
              uint32_t scalex, uint32_t scaley, int use_pri, uint32_t pmask);

// a tile layer (tilemap_t with 8x8 tiles in rows, TILEMAP_SCAN_ROWS): tile = (entry & code_mask) |
// bank[(entry >> bank_shift) & bank_mask], colour = (entry >> color_shift) + color_add
typedef struct {
	const uint16_t *vram;
	const vsg_t *gfx;
	int cols, rows;                  // in tiles (64 x 32 / 64 x 64)
	uint16_t code_mask;
	int bank_shift, bank_mask;
	uint32_t bank[4];
	int color_shift, color_add;
	int opaque;                      // 0: pen 15 transparent
	uint8_t priority;                // ORed into the priority bitmap where a pixel is drawn
	int flip;                        // TILEMAP_FLIPX | TILEMAP_FLIPY
	int dx, dx_flipped;              // set_scrolldx
	int scrollrows;                  // set_scroll_rows (1 = one scroll)
	const int32_t *rowscroll;        // m_rowscroll (scrollrows values)
	int32_t colscroll;               // set_scrolly(0, ...)
	int xextent, yextent;            // visible area right + left + 1, bottom + top + 1
} vsg_layer;

// the layer's pixels of screen line y, columns x0..x1 (bm and pri: column x at [x - x0])
void vsg_layer_line(const vsg_layer *l, int y, int x0, int x1, uint16_t *bm, uint8_t *pri);

#endif
