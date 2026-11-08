/*  Konami 052109/051962 tilemaps (three 64x32 layers of 8x8 tiles: FIX, A, B). Ported from MAME
    src/mame/konami/k052109.cpp (BSD-3-Clause, copyright-holders: Fabio Priuli, Acho A. Tang, R. Belmont) and the parts
    of src/emu/tilemap.cpp it uses (BSD-3-Clause, copyright-holders: Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_K052109_H
#define RBX_K052109_H

#include <stdint.h>

#define K052109_TILE_FLIPX 1
#define K052109_TILE_FLIPY 2
// tilemap draw flags (MAME TILEMAP_DRAW_*): the category in the low bits
#define K052109_DRAW_OPAQUE 0x10
#define K052109_DRAW_ALL_CATEGORIES 0x20
#define K052109_DRAW_PRIMASK(m) ((m) << 8)   // (tilemap_draw's priority_mask: the bitmap's bits kept, then | priority)

// the board's tile callback (K052109_CB_MEMBER): layer, bank, code, color, flags (K052109_TILE_*), priority (category)
typedef void (*k052109_cb_t)(int layer, int bank, int *code, int *color, int *flags, int *priority);

typedef struct {
	// ---- saved ----
	uint8_t ram[0x6000];
	uint8_t charrombank[4], charrombank_2[4];
	uint8_t has_extra_video_ram, rmrd_line, irq_control, romsubbank, scrollctrl, addrmap, tileflip_enable;
	uint8_t irq_out, firq_out, nmi_out;   // (the interrupt lines it drives)
	// the scroll of layers A and B as update_scroll set it at the last vblank's end
	uint16_t rows[2], cols[2];
	int16_t rowscroll[2][256], colscroll[2][64];
	uint8_t state_end;
	// ---- the configuration ----
	const uint8_t *rom;
	uint32_t rom_size;
	uint8_t *gfx;          // (the tiles decoded: a byte a pixel, 64 a tile)
	uint32_t ntiles;
	int colors;            // (the palette's entries / 16: the colour modulo)
	k052109_cb_t cb;
} k052109_t;

// the drawing's view of a frame (k052109_prepare from a snapshot of the chip)
typedef struct {
	uint32_t base[3][2048];   // (a tile: its pixels' offset in gfx | flags << 28)
	uint16_t pen[3][2048];    // (its palette base)
	uint8_t cat[3][2048];
	int flip;
	int sx[3][256], sy[3][64];   // (the effective row and column scrolls)
	int rows[3], cols[3];
	int16_t raw_rowscroll[3][256];
} k052109_view_t;

int k052109_init(k052109_t *k, const uint8_t *rom, uint32_t rom_size, int palette_entries, k052109_cb_t cb);
void k052109_exit(k052109_t *k);
// set_char_ram: the tiles from a RAM of host order words (charlayout_ram), decoded now; the board decodes the tiles it
// changes again (k052109_charram_decode: 16 words a tile) while the drawing is idle
int k052109_init_charram(k052109_t *k, const uint16_t *ram, uint32_t size, int palette_entries, k052109_cb_t cb);
void k052109_charram_decode(k052109_t *k, const uint16_t *ram, uint32_t tile);
void k052109_reset(k052109_t *k);
uint8_t k052109_read(const k052109_t *k, int offset);
void k052109_write(k052109_t *k, int offset, uint8_t data);
static inline void k052109_set_rmrd_line(k052109_t *k, int state) { k->rmrd_line = (uint8_t)(state != 0); }
// the vblank: its start raises the IRQ when enabled (irq_out); its end latches the scroll (update_scroll)
void k052109_vblank(k052109_t *k, int state);
// the frame's tiles through the callback (called once a frame from the drawing, the chip a snapshot);
// xextent / yextent: the visible area's x0 + x1 + 1 and y0 + y1 + 1 (the flip around its centre)
void k052109_prepare(const k052109_t *k, k052109_view_t *v, int xextent, int yextent);
// tilemap_draw into an ind16 bitmap and the screen's priority bitmap (frame / pri at the bitmap's (fx0, fy0), pitch in
// pixels) over the clip [cx0, cx1] x [cy0, cy1] of bitmap coordinates
void k052109_draw(const k052109_t *k, const k052109_view_t *v, int layer, int flags, uint8_t priority, uint16_t *frame,
                  uint8_t *pri, int pitch, int fx0, int fy0, int cx0, int cy0, int cx1, int cy1);
int k052109_state(k052109_t *k, void *ctx, int load, const char *tag);

#endif
