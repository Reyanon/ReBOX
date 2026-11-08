/*  Konami 053246 / 053247 / 055673 sprite generator, ported to C from MAME src/mame/konami/k053246_k053247_k055673.cpp
    (BSD-3-Clause, copyright-holders: David Haywood). See k055673.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "k055673.h"
#include "../../rbx.h"

#include <string.h>

void k055673_init(k055673_t *k, const uint8_t *rom, uint32_t rom_size, int layout, int dx, int dy)
{
	memset(k, 0, sizeof(*k));
	k->rom = rom;
	k->rom_size = rom_size;
	k->bpp = layout;
	k->dx = dx;
	k->dy = dy;
}

void k055673_k053246_w(k055673_t *k, int offset, uint8_t data) { k->kx46_regs[offset & 7] = data; }
uint8_t k055673_k053246_read_register(const k055673_t *k, int offset) { return k->kx46_regs[offset & 7]; }

void k055673_reg_word_w(k055673_t *k, int offset, uint16_t data, uint16_t mem_mask)
{
	uint16_t *p = &k->kx47_regs[offset & 15];
	*p = (uint16_t)((data & mem_mask) | (*p & ~mem_mask));
}

static inline uint16_t rom16(const k055673_t *k, uint32_t i)   // (the ROM as little-endian words, as MAME's u16 view)
{
	return 2 * i + 1 < k->rom_size ? (uint16_t)(k->rom[2 * i] | k->rom[2 * i + 1] << 8) : 0;
}
static inline uint8_t rom8(const k055673_t *k, uint32_t i) { return i < k->rom_size ? k->rom[i] : 0; }

// the 5bpp window: +0 32 bits from one set of ROMs, +8 from another (the 5th bit planes after the 4bpp area)
static uint16_t rom_5bpp_word_r(const k055673_t *k, int offset)
{
	const uint32_t size4 = (k->rom_size / (1024 * 1024)) / 5 * 4 * 1024 * 1024;   // (the offset to the 5th bit)
	uint32_t romofs = (uint32_t)k->kx46_regs[6] << 16 | (uint32_t)k->kx46_regs[7] << 8 | k->kx46_regs[4];
	switch (offset) {
	case 0: return rom16(k, romofs + 2);   // 20k / 36u
	case 1: return rom16(k, romofs + 3);   // 17k / 36y
	case 2: case 3: romofs /= 2; return rom8(k, size4 + romofs + 1);   // 10k / 32y
	case 4: return rom16(k, romofs);       // 22k / 34u
	case 5: return rom16(k, romofs + 1);   // 19k / 34y
	case 6: case 7: romofs /= 2; return rom8(k, size4 + romofs);       // 12k / 29y
	}
	return 0;
}

uint16_t k055673_rom_word_r(const k055673_t *k, int offset)
{
	offset &= 7;
	if (k->bpp == 5)
		return rom_5bpp_word_r(k, offset);
	uint32_t romofs = (uint32_t)k->kx46_regs[6] << 16 | (uint32_t)k->kx46_regs[7] << 8 | k->kx46_regs[4];
	romofs = (romofs >> 2) * (uint32_t)k->bpp;
	if ((offset & 0x4) == 0)
		romofs += (uint32_t)k->bpp >> 1;
	return rom16(k, romofs + (uint32_t)(offset & 0x3));
}

int k055673_state(k055673_t *k, void *ctx, int load)
{
	(void)load;
	const uint32_t n = (uint32_t)((const uint8_t *)&k->rom - (const uint8_t *)k);
	return rbx_state_io(ctx, "k055673", k, n);
}
