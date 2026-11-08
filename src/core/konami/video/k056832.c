/*  Konami 056832 tilemap chip, ported to C from MAME src/mame/konami/k054156_k054157_k056832.cpp (BSD-3-Clause,
    copyright-holders: David Haywood). See k056832.h. (MAME's tilemap dirty marking has no counterpart: the pages
    are drawn from the RAM each frame.)
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "k056832.h"
#include "../../rbx.h"

#include <string.h>

static void update_page_layout(k056832_t *k)
{
	k->layer_association = k->default_layer_association;   // (enable layer association by default)
	// disable association if a layer grabs the entire 4x4 map (happens in Twinbee and Dadandarn)
	for (int layer = 0; layer < 4; layer++)
		if (!k->y[layer] && !k->x[layer] && k->h[layer] == 3 && k->w[layer] == 3) {
			k->layer_association = 0;
			break;
		}
	if (k->djmain_hack == 2)   // (winning spike doesn't like layer association)
		k->layer_association = 0;
	for (int p = 0; p < K056832_PAGE_COUNT; p++)
		k->layer_assoc_with_page[p] = -1;
	for (int layer = 0; layer < 4; layer++) {
		const int rowstart = k->y[layer], colstart = k->x[layer], rowspan = k->h[layer] + 1, colspan = k->w[layer] + 1;
		const int setlayer = k->layer_association ? layer : k->active_layer;
		for (int r = 0; r < rowspan; r++)
			for (int c = 0; c < colspan; c++) {
				const int p = (((rowstart + r) & 3) << 2) + ((colstart + c) & 3);
				if (!(k->djmain_hack == 1) || k->layer_assoc_with_page[p] == -1)
					k->layer_assoc_with_page[p] = setlayer;
			}
	}
	for (int p = 0; p < K056832_PAGE_COUNT; p++)   // (mark_all_tilemaps_dirty: the pages take their layer's mode)
		if (k->layer_assoc_with_page[p] != -1)
			k->page_tile_mode[p] = k->layer_tile_mode[k->layer_assoc_with_page[p]];
}

static void mark_plane(k056832_t *k, int layer)   // (mark_plane_dirty: the layer's pages take its tile mode)
{
	for (int p = 0; p < K056832_PAGE_COUNT; p++)
		if (k->layer_assoc_with_page[p] == layer)
			k->page_tile_mode[p] = k->layer_tile_mode[layer];
}

static void change_rambank(k056832_t *k)
{
	// ------xx page col, ---xx--- page row
	const int bank = k->regs[0x19];
	if (k->regs[0] & 0x02)   // (external linescroll enable)
		k->selected_page = K056832_PAGE_COUNT;
	else
		k->selected_page = ((bank >> 1) & 0xc) | (bank & 3);
	k->selected_page_x4096 = k->selected_page << 12;
	for (int p = 0; p < K056832_PAGE_COUNT; p++)
		if (k->layer_assoc_with_page[p] != -1)
			k->page_tile_mode[p] = k->layer_tile_mode[k->layer_assoc_with_page[p]];
}

static void change_rombank(k056832_t *k)
{
	int bank;
	if (k->uses_tile_banks)   // (Asterix)
		bank = (k->regs[0x1a] >> 8) | (k->regs[0x1b] << 4) | (k->cur_tile_bank << 6);
	else
		bank = k->regs[0x1a] | (k->regs[0x1b] << 16);
	k->cur_gfx_banks = k->num_gfx_banks ? bank % k->num_gfx_banks : 0;
}

void k056832_init(k056832_t *k, const uint8_t *rom, uint32_t rom_size, int bpp, int big, int djmain_hack)
{
	memset(k, 0, sizeof(*k));
	k->rom = rom;
	k->rom_size = rom_size;
	k->bpp = bpp;
	k->linemap_enabled = big;
	k->djmain_hack = djmain_hack;
	k->num_gfx_banks = (int)(rom_size / 0x2000);
	for (int i = 0; i < 8; i++) {   // (create_tilemaps)
		k->lsram_page[i][0] = i;
		k->lsram_page[i][1] = i << 11;
		k->layer_tile_mode[i] = 1;
	}
	k->default_layer_association = 1;
	for (int i = 0; i < K056832_PAGE_COUNT; i++)
		k->page_tile_mode[i] = 1;
	update_page_layout(k);   // (finalize_init)
	change_rambank(k);
	change_rombank(k);
}

void k056832_reset(k056832_t *k)
{
	(void)k;   // (MAME: device_reset has nothing; the board's "reset graphics chips" calls it)
}

void k056832_set_layer_offs(k056832_t *k, int layer, int offsx, int offsy)
{
	k->layer_offs[layer][0] = offsx;
	k->layer_offs[layer][1] = offsy;
}

void k056832_set_lsram_page(k056832_t *k, int logical_page, int physical_page, int physical_offset)
{
	k->lsram_page[logical_page][0] = physical_page;
	k->lsram_page[logical_page][1] = physical_offset;
}

void k056832_set_layer_association(k056832_t *k, int status) { k->default_layer_association = status; }

void k056832_set_tile_bank(k056832_t *k, int bank)
{
	k->uses_tile_banks = 1;
	if (k->cur_tile_bank != bank) {
		k->cur_tile_bank = bank;
		for (int l = 0; l < 4; l++)
			mark_plane(k, l);
	}
	change_rombank(k);
}

// the ROM readback for the ROM checks (rom_read_b)
static int rom_read_b(k056832_t *k, int offset, int blksize, int blksize2, int zerosec)
{
	if (k->rom_half && zerosec)
		return 0;
	offset += k->cur_gfx_banks * 0x2000;   // (the bank offset)
	int base = (offset / blksize) * blksize2;   // (the base of the ROM block)
	base += (offset % blksize) * 2;   // (the starting offset of the proper word inside the block)
	int ret;
	if (k->rom_half)
		ret = (uint32_t)(base + 1) < k->rom_size ? k->rom[base + 1] : 0;
	else {
		ret = (uint32_t)base < k->rom_size ? k->rom[base] : 0;
		k->rom_half = 1;
	}
	return ret;
}

uint8_t k056832_5bpp_rom_long_r(k056832_t *k, uint32_t offset, int lane)
{
	return (uint8_t)rom_read_b(k, (int)(offset * 4) + lane, 4, 5, lane == 3);
}

uint8_t k056832_6bpp_rom_long_r(k056832_t *k, uint32_t offset, int lane)
{
	return (uint8_t)rom_read_b(k, (int)(offset * 4) + lane, 4, 6, 0);
}

uint16_t k056832_ram_word_r(k056832_t *k, int offset)
{
	k->rom_half = 0;   // (reading from tile RAM resets the ROM readback "half" offset)
	return k->videoram[k->selected_page_x4096 + (offset & 0xfff)];
}

void k056832_ram_word_w(k056832_t *k, int offset, uint16_t data, uint16_t mem_mask)
{
	uint16_t *t = &k->videoram[k->selected_page_x4096 + (offset & 0xfff)];
	*t = (uint16_t)((data & mem_mask) | (*t & ~mem_mask));
}

// (ram_half_word_r / _w: asterix's 8-bit window, the code words only)
uint16_t k056832_ram_half_word_r(const k056832_t *k, int offset)
{
	return k->videoram[k->selected_page_x4096 + ((((uint32_t)offset << 1) & 0xffe) | (((uint32_t)offset >> 11) ^ 1))];
}

void k056832_ram_half_word_w(k056832_t *k, int offset, uint16_t data, uint16_t mem_mask)
{
	uint16_t *t = &k->videoram[k->selected_page_x4096 + ((((uint32_t)offset << 1) & 0xffe) | 1)];
	*t = (uint16_t)((data & mem_mask) | (*t & ~mem_mask));
}

// (ram_code_lo_r / _hi_r / _lo_w / _hi_w: the medal boards' 8-bit window, the code words only, over two pages)
uint8_t k056832_ram_code_r(const k056832_t *k, int offset, int hi)
{
	const int i = k->selected_page_x4096 + offset * 2 + 1;
	if (i >= 0x1000 * (K056832_PAGE_COUNT + 1))
		return 0;
	return (uint8_t)(hi ? k->videoram[i] >> 8 : k->videoram[i]);
}

void k056832_ram_code_w(k056832_t *k, int offset, int hi, uint8_t data)
{
	const int i = k->selected_page_x4096 + offset * 2 + 1;
	if (i >= 0x1000 * (K056832_PAGE_COUNT + 1))
		return;
	k->videoram[i] = (uint16_t)(hi ? (k->videoram[i] & 0x00ff) | data << 8 : (k->videoram[i] & 0xff00) | data);
}

uint8_t k056832_konmedal_rom_r(const k056832_t *k, uint32_t offset)
{
	const uint32_t addr = (((uint32_t)k->regs[0x1a] << 9) & 0x60000) | (((uint32_t)k->regs[0x1b] << 15) & 0x18000) |
	                      (((uint32_t)k->regs[0x1a] << 3) & 0x06000);
	return k->rom[(addr + offset) % k->rom_size];
}

uint16_t k056832_word_r(const k056832_t *k, int offset) { return k->regs[offset & 0x1f]; }

void k056832_b_word_w(k056832_t *k, int offset, uint16_t data, uint16_t mem_mask)
{
	uint16_t *r = &k->regsb[offset & 3];
	*r = (uint16_t)((data & mem_mask) | (*r & ~mem_mask));
}

void k056832_word_w(k056832_t *k, int offset, uint16_t data, uint16_t mem_mask)
{
	offset &= 0x1f;
	const uint16_t old_data = k->regs[offset];
	const uint16_t new_data = (uint16_t)((data & mem_mask) | (old_data & ~mem_mask));
	k->regs[offset] = new_data;
	if (new_data == old_data)
		return;
	switch (offset) {
	case 0x00 / 2:   // (the screen flips: read at draw time; the external linescroll RAM page enable)
		if ((new_data & 0x02) != (old_data & 0x02))
			change_rambank(k);
		break;
	case 0x08 / 2:   // -------- ----DCBA tile mode: 0=512x1, 1=8x8
		for (int layer = 0; layer < 4; layer++) {
			const int mask = 1 << layer, i = new_data & mask;
			if (i != (old_data & mask)) {
				k->layer_tile_mode[layer] = (uint8_t)(i != 0);
				mark_plane(k, layer);
			}
		}
		break;
	case 0x32 / 2:
		change_rambank(k);
		break;
	case 0x34 / 2:   // (ROM bank select for checksum)
	case 0x36 / 2:   // (secondary ROM bank select for use with tile banking)
		change_rombank(k);
		break;
	default: {
		const int layer = offset & 3;
		if (offset >= 0x10 / 2 && offset <= 0x16 / 2) {
			k->y[layer] = (new_data & 0x18) >> 3;
			k->h[layer] = new_data & 0x3;
			k->active_layer = layer;
			update_page_layout(k);
		} else if (offset >= 0x18 / 2 && offset <= 0x1e / 2) {
			k->x[layer] = (new_data & 0x18) >> 3;
			k->w[layer] = new_data & 0x03;
			k->active_layer = layer;
			update_page_layout(k);
		} else if (offset >= 0x20 / 2 && offset <= 0x26 / 2)
			k->dy[layer] = (int16_t)new_data;
		else if (offset >= 0x28 / 2 && offset <= 0x2e / 2)
			k->dx[layer] = (int16_t)new_data;
		break;
	}
	}
}

int k056832_state(k056832_t *k, void *ctx, int load)
{
	// (the state block: from videoram up to the configuration)
	const uint32_t n = (uint32_t)((const uint8_t *)&k->rom - (const uint8_t *)k);
	(void)load;
	return rbx_state_io(ctx, "k056832", k, n);
}
