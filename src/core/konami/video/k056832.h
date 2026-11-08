/*  Konami 056832 tilemap chip, ported to C from MAME src/mame/konami/k054156_k054157_k056832.cpp (BSD-3-Clause,
    copyright-holders: David Haywood). Four layers over a 4x4 grid of 16 pages (64x32 tiles of 8x8, 512x256 pixels),
    a CPU-paged tile RAM, linescroll / rowscroll / xy scroll, the tile ROM readback for the ROM checks.
    This part: the registers, the RAM paging, the page layout and the ROM readback (the drawing: k056832_draw).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_K056832_H
#define RBX_K056832_H

#include <stdint.h>

#define K056832_PAGE_COUNT 16
#define K056832_PAGE_WIDTH 512
#define K056832_PAGE_HEIGHT 256

enum { K056832_BPP_4, K056832_BPP_5, K056832_BPP_6, K056832_BPP_8 };

typedef struct k056832_s k056832_t;
struct k056832_s {
	// ---- state (k056832_state) ----
	uint16_t videoram[0x1000 * (K056832_PAGE_COUNT + 1)];   // 16 pages and the external linescroll page
	uint16_t regs[0x20], regsb[4];
	int32_t layer_assoc_with_page[K056832_PAGE_COUNT];
	int32_t x[8], y[8], w[8], h[8], dx[8], dy[8];
	int32_t layer_offs[8][2], lsram_page[8][2];
	uint8_t layer_tile_mode[8], page_tile_mode[K056832_PAGE_COUNT];
	int32_t default_layer_association, layer_association, active_layer;
	int32_t selected_page, selected_page_x4096, cur_gfx_banks, rom_half;
	int32_t uses_tile_banks, cur_tile_bank;
	// ---- configuration ----
	const uint8_t *rom;          // the tile ROM region (MAME m_rombase)
	uint32_t rom_size;
	int bpp, linemap_enabled, djmain_hack;
	int num_gfx_banks;
};

void k056832_init(k056832_t *k, const uint8_t *rom, uint32_t rom_size, int bpp, int big, int djmain_hack);
// MAME device reset (the control's "reset graphics chips"): nothing but the RAM bank / ROM bank recomputed
void k056832_reset(k056832_t *k);
void k056832_word_w(k056832_t *k, int offset, uint16_t data, uint16_t mem_mask);   // registers (offset 0..0x1f)
uint16_t k056832_word_r(const k056832_t *k, int offset);
void k056832_b_word_w(k056832_t *k, int offset, uint16_t data, uint16_t mem_mask);  // the second bank (0..3)
uint16_t k056832_ram_word_r(k056832_t *k, int offset);   // the CPU page (offset 0..0xfff words)
void k056832_ram_word_w(k056832_t *k, int offset, uint16_t data, uint16_t mem_mask);
// the tile ROM readback: a byte of the 32-bit bus (lane 0 = bits 24-31) at long offset
uint8_t k056832_5bpp_rom_long_r(k056832_t *k, uint32_t offset, int lane);
uint8_t k056832_6bpp_rom_long_r(k056832_t *k, uint32_t offset, int lane);
void k056832_set_tile_bank(k056832_t *k, int bank);
uint16_t k056832_ram_half_word_r(const k056832_t *k, int offset);
void k056832_ram_half_word_w(k056832_t *k, int offset, uint16_t data, uint16_t mem_mask);
uint8_t k056832_ram_code_r(const k056832_t *k, int offset, int hi);   // (the code word's bytes, offset in words / 2)
void k056832_ram_code_w(k056832_t *k, int offset, int hi, uint8_t data);
uint8_t k056832_konmedal_rom_r(const k056832_t *k, uint32_t offset);
void k056832_set_layer_offs(k056832_t *k, int layer, int offsx, int offsy);
void k056832_set_lsram_page(k056832_t *k, int logical_page, int physical_page, int physical_offset);
void k056832_set_layer_association(k056832_t *k, int status);
int k056832_state(k056832_t *k, void *ctx, int load);

#endif
