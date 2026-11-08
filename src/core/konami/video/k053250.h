/*  Konami 053250 LVC (the line-based road / background layer), ported to C from MAME src/mame/konami/k053250.cpp
    (BSD-3-Clause, copyright-holders: Olivier Galibert). The registers, the RAM with its two line buffers (the DMA at
    the control's bit 1 falling, once a frame), the ROM readback, and the drawing of a line buffer into an RGB frame.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_K053250_H
#define RBX_K053250_H

#include <stdint.h>

typedef struct {
	// ---- state (k053250_state) ----
	uint16_t ram[0x3000];   // (0x2000 / 0x2800: the two line buffers)
	uint8_t regs[8];
	int32_t page, frame;
	// ---- configuration ----
	const uint8_t *rom;     // the ROM (bytes)
	uint32_t rom_size;
	uint8_t *unpacked;      // its nibbles, one a byte
	int offx, offy;
} k053250_t;

int k053250_init(k053250_t *k, const uint8_t *rom, uint32_t rom_size, int offx, int offy);
void k053250_exit(k053250_t *k);
void k053250_reset(k053250_t *k);
uint16_t k053250_reg_r(const k053250_t *k, int offset);
void k053250_reg_w(k053250_t *k, int offset, uint16_t data, uint16_t mem_mask, int frame);
uint16_t k053250_ram_r(const k053250_t *k, int offset);
void k053250_ram_w(k053250_t *k, int offset, uint16_t data, uint16_t mem_mask);
uint16_t k053250_rom_r(const k053250_t *k, int offset);
int k053250_state(k053250_t *k, void *ctx, int load);

// what the drawing needs (taken at vblank): the line buffer shown and the registers
typedef struct { uint16_t lines[0x800]; uint8_t regs[8]; } k053250_snap_t;
void k053250_snap(const k053250_t *k, k053250_snap_t *s);
// k053250_device::draw into a frame of RGB pixels: dst the pixel at (x0, y0) of the clip rectangle, pitch in pixels;
// pens the palette (pens_n entries, a power of 2), colorbase as MAME's
// the same, writing only the rows cy0..cy1 (a band: the geometry is the whole target's)
void k053250_draw_rows(const k053250_t *k, const k053250_snap_t *s, uint32_t *dst, int pitch, int x0, int y0, int x1,
                       int y1, const uint32_t *pens, int pens_n, int colorbase, int vis_max_x, int vis_max_y, uint8_t *pri,
                       uint8_t prival, int cy0, int cy1);
void k053250_draw(const k053250_t *k, const k053250_snap_t *s, uint32_t *dst, int pitch, int x0, int y0, int x1, int y1,
                  const uint32_t *pens, int pens_n, int colorbase, int vis_max_x, int vis_max_y, uint8_t *pri, uint8_t prival);
// (pri: the screen's priority bitmap at (x0, y0), the same pitch; prival written where it draws, 0: none)

#endif
