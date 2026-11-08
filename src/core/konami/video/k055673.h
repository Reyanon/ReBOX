/*  Konami 053246 / 053247 / 055673 sprite generator, ported to C from MAME src/mame/konami/k053246_k053247_k055673.cpp
    (BSD-3-Clause, copyright-holders: David Haywood). This part: the registers (053246 OBJSET1, 055673 OBJSET2),
    the sprite RAM and the ROM readback (the drawing with the GX mixer).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_K055673_H
#define RBX_K055673_H

#include <stdint.h>

// the layouts (MAME K055673_LAYOUT_*: the value is also the bits per pixel but for PS)
#define K055673_LAYOUT_GX 5
#define K055673_LAYOUT_RNG 4
#define K055673_LAYOUT_LE2 8
#define K055673_LAYOUT_GX6 6
#define K055673_LAYOUT_PS 7
#define K055673_LAYOUT_053247 9   // (a K053247: NORMAL_PLANE_ORDER, 4 bpp)
#define K055673_LAYOUT_053245 10  // (a K053244/053245: its spritelayout, 4 bpp)
#define K055673_LAYOUT_051960 11  // (a K051960: its spritelayout, 4 bpp; the MIA one is the K053245's)
#define K055673_LAYOUT_GRADIUS3 12   // (a K051960: spritelayout_gradius3, 4 bpp)

typedef struct k055673_s k055673_t;
struct k055673_s {
	// ---- state (k055673_state) ----
	uint16_t ram[0x2000];
	uint8_t kx46_regs[8];
	uint16_t kx47_regs[16];
	int32_t objcha_line;
	// ---- configuration ----
	const uint8_t *rom;   // the sprite ROM region (MAME m_gfxrom: little-endian words)
	uint32_t rom_size;
	int bpp, dx, dy;
};

void k055673_init(k055673_t *k, const uint8_t *rom, uint32_t rom_size, int layout, int dx, int dy);
void k055673_k053246_w(k055673_t *k, int offset, uint8_t data);   // offset 0..7
uint8_t k055673_k053246_read_register(const k055673_t *k, int offset);
void k055673_reg_word_w(k055673_t *k, int offset, uint16_t data, uint16_t mem_mask);   // OBJSET2 (write only)
uint16_t k055673_rom_word_r(const k055673_t *k, int offset);   // the separate ROM window (offset 0..7 words)
static inline uint16_t k055673_ram_r(const k055673_t *k, int offset) { return k->ram[offset & 0x1fff]; }
static inline void k055673_ram_w(k055673_t *k, int offset, uint16_t data, uint16_t mem_mask)
{
	uint16_t *p = &k->ram[offset & 0x1fff];
	*p = (uint16_t)((data & mem_mask) | (*p & ~mem_mask));
}
int k055673_state(k055673_t *k, void *ctx, int load);

#endif
