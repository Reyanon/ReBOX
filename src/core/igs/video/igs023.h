/*  IGS023 video (PGM), ported from MAME src/mame/igs/igs023_video.cpp (BSD-3-Clause, copyright-holder: David Haywood):
    a text layer of 8x8x4 tiles, a background of 32x32x5 tiles with a scroll for each line, and sprites drawn from the
    compressed colour (A) and mask (B) data, zoomed by a table in the chip.
    The 68000 sees the video RAM (0x900000) and the registers (0xb00000) through the board driver.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_IGS023_H
#define RBX_IGS023_H

#include <stdint.h>

#define IGS023_W 448
#define IGS023_H 224
#define IGS023_MAX_SPRITES 256
#define IGS023_BANDS 8   // (the drawing in bands: igs023_prepass)

typedef struct {
	int16_t x, y;
	uint8_t xgrow, ygrow, flip, pri;
	uint32_t xzoom, yzoom, color, offs, width, height;
} igs023_sprite;

// what drawing reads (copied at vblank)
typedef struct {
	uint16_t bg[0x800], tx[0x1000], rowscroll[0x800];
	uint16_t bg_yscroll, bg_xscroll, tx_yscroll, tx_xscroll, ctrl, pad;
	uint16_t pal[0xa00];
	igs023_sprite spr[IGS023_MAX_SPRITES];
	int nspr;
	// igs023_prepass: for each unzoomed sprite and band, the first row of the band and the decoding state there, so
	// that a band does not decode the rows above it again
	struct { int32_t ycnt, ab; uint32_t boff, ao; } bst[IGS023_MAX_SPRITES][IGS023_BANDS];
	int pre;
} igs023_snap;

typedef struct {
	// RAM in host-order 16-bit words (the 68000 bus maps bg, tx, rowscroll and spritebuffer directly)
	uint16_t bg[0x800], tx[0x1000], rowscroll[0x800];
	uint16_t spritebuffer[0x800], zoomram[0x20];
	uint16_t bg_yscroll, bg_xscroll, bg_scale, tx_yscroll, tx_xscroll, ctrl;
	igs023_sprite spr[IGS023_MAX_SPRITES];   // the list taken at the last vblank (get_sprites)
	int nspr;
	// ROM data
	const uint8_t *gfx;        // "igs023": text tiles, background tiles
	uint32_t gfx_size;
	const uint16_t *adata;     // "igs023:sprcol"
	uint32_t amask;            // words - 1
	const uint16_t *bdata;     // "igs023:sprmask"
	uint32_t bmask;
	uint8_t *tx_clear, *bg_clear;   // (a tile all transparent: skipped)
} igs023_t;

void igs023_init(igs023_t *v, const uint8_t *gfx, uint32_t gfx_size, const uint8_t *a, uint32_t a_size, const uint8_t *b,
                 uint32_t b_size);
void igs023_exit(igs023_t *v);
void igs023_reset(igs023_t *v);
// the registers at 0xb01000-0xb0ffff (offset from 0xb00000); vpos: the scanline for 0xb07000
uint16_t igs023_reg_r(igs023_t *v, uint32_t offset, int vpos);
void igs023_reg_w(igs023_t *v, uint32_t offset, uint16_t data, uint16_t mask);
// vblank: sprite DMA from main RAM (host-order words) and the sprite list
void igs023_get_sprites(igs023_t *v, const uint16_t *mainram);
void igs023_snapshot(const igs023_t *v, igs023_snap *s, const uint16_t *pal);
// draw the snapshot into an indexed bitmap (IGS023_W x IGS023_H, palette indices 0-0x9ff); palette from s->pal
void igs023_draw(const igs023_t *v, const igs023_snap *s, uint16_t *bitmap, int pitch);
// before drawing in IGS023_BANDS equal bands (band b: lines IGS023_H * b / IGS023_BANDS ...): one pass over the sprites
void igs023_prepass(const igs023_t *v, igs023_snap *s);
// a band of lines [y0, y1): the drawing may be split between threads (pri: IGS023_W bytes a line of the band)
void igs023_draw_lines(const igs023_t *v, const igs023_snap *s, uint16_t *bitmap, int pitch, int y0, int y1, uint8_t *pri);

#endif
