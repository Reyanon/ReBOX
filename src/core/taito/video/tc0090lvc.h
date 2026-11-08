/*  Taito TC0090LVC (Z80 core + memory banking + tilemaps / sprites / palette / 8bpp bitmap), ported from MAME
    src/devices/machine/tc009xlvc.cpp (LGPL-2.1+, copyright-holders: Angelo Salese; based off the Taito L
    implementation by Olivier Galibert). The Z80 itself is the board's (cpu/z80); this module holds the chip's
    internal registers, the 20-bit "vram space" behind the 0xc000-0xfdff RAM banks and the video.
    Drawing follows the snapshot contract: tc0090lvc_capture() at vblank copies what drawing needs into a
    tc0090lvc_view, tc0090lvc_render() draws from the view only.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_TC0090LVC_H
#define RBX_TC0090LVC_H

#include <stdint.h>

#define LVC_SCREEN_W 320
#define LVC_VIS_Y0   16      // visible lines 16..239 (MAME visarea 0*8..40*8-1, 2*8..30*8-1)
#define LVC_VIS_Y1   239
#define LVC_BLACK    256     // palette index of the black pen (screen disabled)

typedef struct {
	uint8_t vram[0x10000];        // vram space 0x10000-0x1ffff (0x10000-0x13fff read only)
	uint8_t bitmap_ram[0x20000];  // vram space 0x40000-0x5ffff
	uint8_t pal[0x200];           // vram space 0x80000-0x801ff (mirrored to 0x80fff), xBGRBBBBGGGGRRRR_bit0
	uint8_t vregs[0x100];
	uint8_t irq_vector[3], irq_enable, ram_bank[4], rom_bank;
	uint8_t sprram_buffer[0x400];
	uint8_t chr_dirty[256];       // RAM characters (2048 x 32 bytes) written since the last capture
	uint32_t tile_or;             // board tile callback: bits ORed into 8x8 tile codes (horshoes bank << 12)
	const uint8_t *rom;
	uint32_t rom_mask;
} tc0090lvc_t;

typedef struct {
	uint8_t vram[0x10000];
	uint8_t bitmap_ram[0x20000];
	uint8_t pal[0x200];
	uint8_t vregs[8];
	uint8_t spr[0x400];
	uint32_t tile_or;
	uint8_t dirty[256];
	// decoded graphics: 8x8 tiles at one byte per pixel; flags 0 = empty, 1 = mixed, 2 = no transparent pixel
	uint8_t *gfx, *gfx_flags;
	uint32_t ntiles, ntiles_mask;   // (the mask: ntiles - 1 for a power of two, else 0: the % of a tile code)
	uint8_t chr[2048 * 64], chr_flags[2048];
	uint8_t pri[LVC_SCREEN_W * 256];
} tc0090lvc_view;

void tc0090lvc_init(tc0090lvc_t *t, const uint8_t *rom, uint32_t rom_size);   // rom_size: power of two
void tc0090lvc_reset(tc0090lvc_t *t);
// CPU accesses outside 0x8000-0xbfff (the board's external area)
uint8_t tc0090lvc_read(tc0090lvc_t *t, uint16_t a);
void tc0090lvc_write(tc0090lvc_t *t, uint16_t a, uint8_t v);
// irq_enable_w as a plain register (the board adds its line handling)
static inline uint8_t tc0090lvc_irq_vector(const tc0090lvc_t *t, int level) { return t->irq_vector[level]; }

int tc0090lvc_view_init(tc0090lvc_view *v, const uint8_t *gfx, uint32_t gfx_size);   // decodes the gfx ROM
void tc0090lvc_view_exit(tc0090lvc_view *v);
// at vblank start: copy the drawing state into the view, then latch the sprite buffer (MAME screen_eof)
void tc0090lvc_capture(tc0090lvc_t *t, tc0090lvc_view *v);
// draw the visible area (320 x 224) and the palette (257 entries, RGB888)
void tc0090lvc_render(tc0090lvc_view *v, uint16_t *bitmap, int pitch, uint32_t *palette);

#endif
