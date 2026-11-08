/*  Konami 051316 PSAC (a 32x32 tilemap of 16x16 tiles, rotated and zoomed). Ported from MAME
    src/devices/video/k051316.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria) and tilemap_t::draw_roz of
    src/emu/tilemap.cpp (BSD-3-Clause, copyright-holders: Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_K051316_H
#define RBX_K051316_H

#include <stdint.h>

#define K051316_DRAW_OPAQUE 0x10
#define K051316_DRAW_LAYER0 0x20   // (with a layer mask: the pixels whose bits of the mask are all set, opaque)
#define K051316_DRAW_LAYER1 0x40   // (the others)

// the board's zoom callback (K051316_CB_MEMBER): code and colour from the tile's two RAM bytes
typedef void (*k051316_cb_t)(int chip, int *code, int *color);

typedef struct {
	// ---- saved ----
	uint8_t ram[0x800];
	uint8_t ctrl[16];
	uint8_t wrap, readout, flipx_en, flipy_en;
	uint8_t state_end;
	// ---- the configuration ----
	const uint8_t *rom;
	uint32_t rom_size;
	uint8_t *gfx;       // (decoded: a byte a pixel, 256 a tile)
	uint32_t ntiles;
	int chip, dx, dy;   // (set_offsets)
	int bpp;            // (set_bpp: 4, 7 or 8)
	int layermask;      // (set_layermask: 0 none, pixel 0 transparent)
	k051316_cb_t cb;
	// the drawing's cache: the 512x512 pixmap of pens (colour * 16 + pixel, pixel 0 transparent) redrawn by the tiles
	// whose code, colour or flips changed (key); a board drawing from a snapshot copies these pointers to it
	uint16_t *pix;
	uint32_t *key;
} k051316_t;

int k051316_init(k051316_t *k, int chip, const uint8_t *rom, uint32_t rom_size, int dx, int dy, k051316_cb_t cb);   // 4 bpp
int k051316_init_bpp(k051316_t *k, int chip, int bpp, const uint8_t *rom, uint32_t rom_size, int dx, int dy, k051316_cb_t cb);
void k051316_exit(k051316_t *k);
void k051316_reset(k051316_t *k);
void k051316_invalidate(k051316_t *k);   // (the cache redrawn whole: after a state load)
static inline uint8_t k051316_read(const k051316_t *k, int offset) { return k->ram[offset & 0x7ff]; }
static inline void k051316_write(k051316_t *k, int offset, uint8_t data) { k->ram[offset & 0x7ff] = data; }
void k051316_ctrl_w(k051316_t *k, int offset, uint8_t data);
uint8_t k051316_rom_r(const k051316_t *k, int offset);
// zoom_draw from a snapshot of the chip (the callback's state as the board captured it): into an ind16 frame and the
// screen's priority bitmap (frame / pri at the bitmap's (fx0, fy0), pitch in pixels) over the clip [cx0, cx1] x
// [cy0, cy1]; flags K051316_DRAW_OPAQUE, the priority ORed in where drawn
void k051316_zoom_draw(const k051316_t *k, uint16_t *frame, uint8_t *pri, int pitch, int fx0, int fy0, int cx0, int cy0,
                       int cx1, int cy1, int flags, uint8_t priority);
int k051316_state(k051316_t *k, void *ctx, int load, const char *tag);

#endif
