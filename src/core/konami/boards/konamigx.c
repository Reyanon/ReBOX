/*  Konami System GX. Ported from MAME src/mame/konami/konamigx.cpp, konamigx_v.cpp, konamigx_m.cpp (BSD-3-Clause,
    copyright-holders: R. Belmont, Acho A. Tang, Phil Stroffolino, Olivier Galibert) and the devices they use
    (src/devices/machine/k053252.cpp: Luca Elia, David Haywood; eepromser.cpp: Aaron Giles).
    A 68EC020 at 24 MHz, the K056832 tilemaps, the K053246/K055673 sprites, the K055555 priority encoder and the
    K054338 alpha blender, the K053252 CCU (the screen timing), a 93C46 EEPROM, the ESC protection (MAME's high-level
    command handlers) and the sound board (machine/gx_sound.c). Type 2 boards (docs/KONAMI.md); the Parodius games
    first (user, 2026-11-04).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../machine/eeprom93c46.h"
#include "../machine/gx_sound.h"
#include "../video/k056832.h"
#include "../video/k055673.h"
#include "konamigx.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MASTER_CLOCK 24000000u
#define VIS_H 224

gx_t gx;

static uint8_t *s_rom;
static uint32_t s_rom_size;
static int s_port_sys, s_port_in, s_port_srv;
static eeprom93c46_t s_eeprom;
static char s_nv_path[256];
static int32_t *s_mix_l, *s_mix_r;
static int s_mix_cap;

// ---- the game's configuration (MAME gameDefs and the machine configurations) ----
enum { SP_NONE, SP_LE2, SP_TKMMPZDM, SP_DRAGOON, SP_SEXYPARO, SP_DAISKISS, SP_SAL2, SP_TYPE4, SP_TBYAHHOO, SP_FANTJOUR };
static const struct { const char *set; int special, readback6; } s_defs[] = {
	{ "le2", SP_LE2, 0 }, { "le2u", SP_LE2, 0 }, { "le2j", SP_LE2, 0 },
	{ "fantjour", SP_FANTJOUR, 0 }, { "fantjoura", SP_FANTJOUR, 0 }, { "gokuparo", SP_NONE, 0 },
	{ "crzcross", SP_NONE, 0 }, { "puzldama", SP_NONE, 0 }, { "tbyahhoo", SP_TBYAHHOO, 0 }, { "mtwinbee", SP_TBYAHHOO, 0 },
	{ "tkmmpzdm", SP_TKMMPZDM, 0 }, { "dragoona", SP_DRAGOON, 0 }, { "dragoonj", SP_DRAGOON, 0 },
	{ "sexyparo", SP_SEXYPARO, 0 }, { "sexyparoa", SP_SEXYPARO, 0 }, { "daiskiss", SP_DAISKISS, 0 }, { "tokkae", SP_NONE, 0 },
	{ "salmndr2", SP_SAL2, 1 }, { "salmndr2a", SP_SAL2, 1 }, { "winspike", SP_TYPE4, 0 }, { "winspikea", SP_TYPE4, 0 },
	{ "winspikej", SP_TYPE4, 0 },
};
// the machines (MAME's machine configurations): the K056832 / K055673 settings and the sprite callback
static const struct { const char *machine; int bpp, big, layout, dx, dy, tilecb, sprcb; } s_machines[] = {
	{ "konamigx", K056832_BPP_5, 0, K055673_LAYOUT_GX, -26, -23, GX_TILECB_TYPE2, GX_SPRCB_TYPE2 },
	{ "gokuparo", K056832_BPP_5, 0, K055673_LAYOUT_GX, -46, -23, GX_TILECB_TYPE2, GX_SPRCB_TYPE2 },
	{ "sexyparo", K056832_BPP_5, 0, K055673_LAYOUT_GX, -42, -23, GX_TILECB_ALPHA, GX_SPRCB_TYPE2 },
	{ "tbyahhoo", K056832_BPP_5, 0, K055673_LAYOUT_GX, -26, -23, GX_TILECB_TYPE2, GX_SPRCB_TYPE2 },
	{ "dragoonj", K056832_BPP_5, 1, K055673_LAYOUT_RNG, -53, -23, GX_TILECB_TYPE2, GX_SPRCB_DRAGOONJ },
	{ "le2", K056832_BPP_8, 1, K055673_LAYOUT_LE2, -46, -23, GX_TILECB_TYPE2, GX_SPRCB_LE2 },
	{ "konamigx_6bpp", K056832_BPP_6, 0, K055673_LAYOUT_GX, -46, -23, GX_TILECB_TYPE2, GX_SPRCB_TYPE2 },
	{ "salmndr2", K056832_BPP_6, 1, K055673_LAYOUT_GX6, -48, -23, GX_TILECB_SALMNDR2, GX_SPRCB_SALMNDR2 },
	{ "winspike", K056832_BPP_8, 0, K055673_LAYOUT_LE2, -53, -23, GX_TILECB_ALPHA, GX_SPRCB_TYPE2 },
};
static int s_special, s_readback6, s_posthack, s_noskip, gx_vis_w = 288;

// ---- interrupts and the object DMA (MAME eeprom_w, control_w, the vblank and DMA callbacks) ----
static void dmastart(void)
{
	gx.rdport1_3 |= 2;   // (the DMA busy flag)
	if (k055673_k053246_read_register(&gx.k55673, 5) & 0x10)   // (DMAEN: the sprite list copied)
		memcpy(gx.spriteram, gx.k55673.ram, 0x1000);
	const int us = (gx.wrport2 & 1) ? 256 + 32 : 342 + 42;
	gx.dma_due = gx.cycles_done + (int64_t)us * (MASTER_CLOCK / 1000000);
}

static void dmaend(void)
{
	gx.dma_due = -1;
	gx.rdport1_3 &= ~2;   // (the DMA busy flag cleared before IRQ 3)
	if ((gx.wrport1_1 & 0x84) == 0x84 || (gx.syncen & 4)) {   // (IRQ 3: the object DMA end)
		gx.syncen &= ~4;
		gx.rdport1_3 &= ~0x80;
		m68k_bus_hold_irq(3);
	}
}

static void vblank_irq(void)   // (konamigx_type2_vblank_irq)
{
	if (gx.syncen & 0x20) {   // (IRQ 1: the main 60 Hz vblank interrupt)
		gx.syncen &= ~0x20;
		if ((gx.wrport1_1 & 0x81) == 0x81 || (gx.syncen & 1)) {
			gx.syncen &= ~1;
			m68k_bus_hold_irq(1);
		}
	}
	dmastart();
}

static void eeprom_w(int hi, uint16_t data, uint16_t mask)
{
	if (hi) {
		if (mask & 0xff00) {   // bits 24-31: the EEPROM lines, the coin counters, the background select
			const uint8_t o = (uint8_t)(data >> 8);
			eeprom_write_lines(&s_eeprom, (o >> 1) & 1, (o >> 2) & 1, o & 1);
			gx.wrport1_0 = o;
		}
		if (mask & 0x00ff) {   // bits 16-23: the IRQ enables (bit 7 masks all)
			gx.wrport1_1 = (uint8_t)data;
			if (gx.wrport1_1 & 0x80)   // (each IRQ triggered at least once after being enabled)
				gx.syncen |= gx.wrport1_1 & 0x1f;
		}
	}
}

static void control_w(int hi, uint16_t data, uint16_t mask)
{
	static const uint32_t pixclock[4] = { 6000000, 8000000, 12000000, 16000000 };
	if (!hi || !(mask & 0x00ff))   // (bits 16-23)
		return;
	const uint8_t v = (uint8_t)data;
	if (v & 0x40)   // bit 22: the sound CPU and the DSP run
		gxs_run_line(1);
	else {
		gxs_run_line(0);
		k056832_reset(&gx.k56832);
	}
	gx.k55673.objcha_line = (v & 0x10) != 0;
	gx.wrport2 = v;
	if (gx.prev_pixel_clock != (v & 3)) {
		gx.ccu_clock = pixclock[v & 3];
		gx.prev_pixel_clock = v & 3;
	}
}

// ---- the K053252 CCU: the screen timing it programs ----
static void ccu_res_change(void)
{
	gx_ccu_t *c = &gx.ccu;
	if (c->hc && c->vc && c->hbp && c->hfp && c->vbp && c->vfp && c->hsw && c->vsw) {
		c->lines = c->vc;
		c->line_clocks = c->hc;
		c->clock = gx.ccu_clock;
		c->vis_y0 = 16;   // (set_offsets(24, 16))
		c->vis_y1 = 16 + c->vc - c->vfp - c->vbp - c->vsw - 1;
	}
}

static void ccu_w(int r, uint8_t data)
{
	gx_ccu_t *c = &gx.ccu;
	c->regs[r] = data;
	switch (r) {
	case 0x00: case 0x01: c->hc = ((c->regs[0] << 8 | c->regs[1]) & 0x3ff) + 1; ccu_res_change(); break;
	case 0x02: case 0x03: c->hfp = (c->regs[2] << 8 | c->regs[3]) & 0x1ff; ccu_res_change(); break;
	case 0x04: case 0x05: c->hbp = (c->regs[4] << 8 | c->regs[5]) & 0x1ff; ccu_res_change(); break;
	case 0x08: case 0x09: c->vc = ((c->regs[8] << 8 | c->regs[9]) & 0x1ff) + 1; ccu_res_change(); break;
	case 0x0a: c->vfp = c->regs[0x0a]; ccu_res_change(); break;
	case 0x0b: c->vbp = c->regs[0x0b] + 1; ccu_res_change(); break;
	case 0x0c: c->vsw = ((c->regs[0x0c] & 0xf0) >> 4) + 1; c->hsw = (c->regs[0x0c] & 0x0f) + 1; ccu_res_change(); break;
	case 0x0e:   // (INT1 ack: vblank_irq_ack_w)
		m68k_bus_clear_irq(1);
		gx.syncen |= 0x20;
		break;
	case 0x0f:   // (INT2 ack: hblank_irq_ack_w)
		m68k_bus_clear_irq(2);
		gx.syncen |= 0x40;
		break;
	}
}

static uint8_t ccu_r(int r)
{
	if (r == 0x0e || r == 0x0f) {   // (VCT read-back)
		const int v = gx.line - gx.ccu.vc;
		return (uint8_t)(r == 0x0e ? (v >> 8) & 1 : v & 0xff);
	}
	return gx.ccu.regs[r];
}

// ---- the ESC (MAME's high-level handlers of its programs) ----
static uint16_t ram_rd16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0xc00000 && a < 0xc20000)
		return m68k_word(gx.workram, a - 0xc00000);
	if (a >= 0xd20000 && a < 0xd24000)
		return gx.k55673.ram[(a - 0xd20000) >> 1];
	if (a < s_rom_size)
		return m68k_word(s_rom, a);
	return 0;
}
static void ram_wr16(uint32_t a, uint16_t v)
{
	a &= 0xffffff;
	if (a >= 0xc00000 && a < 0xc20000)
		*(uint16_t *)(gx.workram + ((a - 0xc00000) & ~1u)) = v;
	else if (a >= 0xd20000 && a < 0xd24000)
		gx.k55673.ram[(a - 0xd20000) >> 1] = v;
}
static uint8_t ram_rd8(uint32_t a) { return (uint8_t)(ram_rd16(a) >> ((a & 1) ? 0 : 8)); }

static void generate_sprites(uint32_t src, uint32_t spr, int count)
{
	static struct { int pri; uint32_t adr; } sprites[0x100];
	int scount = 0, ecount = 0;
	for (int i = 0; i < count; i++) {
		const uint32_t adr = src + 0x100 * (uint32_t)i;
		if (!ram_rd16(adr + 2))
			continue;
		const int pri = ram_rd16(adr + 28);
		if (pri < 256) {
			sprites[ecount].pri = pri;
			sprites[ecount].adr = adr;
			ecount++;
		}
	}
	for (int i = 0; i < ecount; i++) {
		const uint32_t adr = sprites[i].adr;
		if (!adr)
			continue;
		uint32_t set = (uint32_t)ram_rd16(adr) << 16 | ram_rd16(adr + 2);
		const uint16_t glob_x = ram_rd16(adr + 4), glob_y = ram_rd16(adr + 8);
		const uint16_t flip_x = ram_rd16(adr + 12) ? 0x1000 : 0x0000, flip_y = ram_rd16(adr + 14) ? 0x2000 : 0x0000;
		const uint16_t glob_f = (uint16_t)(flip_x | (flip_y ^ 0x2000));
		uint16_t zoom_x = ram_rd16(adr + 20), zoom_y = ram_rd16(adr + 22);
		uint16_t color_val = 0, color_mask = 0xffff, color_set = 0, color_rotate = 0, v;
		v = ram_rd16(adr + 24);
		if (v & 0x8000) {
			color_mask = 0xf3ff;
			color_val |= (uint16_t)((v & 3) << 10);
		}
		v = ram_rd16(adr + 26);
		if (v & 0x8000) {
			color_mask &= 0xfcff;
			color_val |= (uint16_t)((v & 3) << 8);
		}
		v = ram_rd16(adr + 18);
		if (v & 0x8000) {
			color_mask &= 0xff1f;
			color_val |= v & 0xe0;
		}
		v = ram_rd16(adr + 16);
		if (v & 0x8000)
			color_set = v & 0x1f;
		if (v & 0x4000)
			color_rotate = v & 0x1f;
		if (!zoom_x)
			zoom_x = 0x40;
		if (!zoom_y)
			zoom_y = 0x40;
		if (set >= 0x200000 && set < 0xd00000) {
			uint16_t count2 = ram_rd16(set);
			set += 2;
			while (count2) {
				const uint16_t idx = ram_rd16(set), flip = ram_rd16(set + 2);
				uint16_t col = ram_rd16(set + 4);
				short y = (short)ram_rd16(set + 6), x = (short)ram_rd16(set + 8);
				if (idx == 0xffff) {
					set = (uint32_t)flip << 16 | col;
					if (set >= 0x200000 && set < 0xd00000)
						continue;
					break;
				}
				if (zoom_y != 0x40)
					y = (short)(y * 0x40 / zoom_y);
				if (zoom_x != 0x40)
					x = (short)(x * 0x40 / zoom_x);
				x = flip_x ? (short)(glob_x - x) : (short)(glob_x + x);
				if (x < -256 || x > 512 + 32)
					goto next;
				y = flip_y ? (short)(glob_y - y) : (short)(glob_y + y);
				if (y < -256 || y > 512)
					goto next;
				col = (uint16_t)((col & color_mask) | color_val);
				if (color_set)
					col = (uint16_t)((col & 0xffe0) | color_set);
				if (color_rotate)
					col = (uint16_t)((col & 0xffe0) | ((col + color_rotate) & 0x1f));
				ram_wr16(spr, (uint16_t)((flip ^ glob_f) | sprites[i].pri));
				ram_wr16(spr + 2, idx);
				ram_wr16(spr + 4, (uint16_t)y);
				ram_wr16(spr + 6, (uint16_t)x);
				ram_wr16(spr + 8, zoom_y);
				ram_wr16(spr + 10, zoom_x);
				ram_wr16(spr + 12, col);
				spr += 16;
				if (++scount == 256)
					return;
			next:
				count2--;
				set += 10;
			}
		}
	}
	while (scount < 256) {
		ram_wr16(spr, (uint16_t)scount);
		scount++;
		spr += 16;
	}
}

// konamigx_esc_alert (konamigx_m.cpp): the work RAM's object list to the K053247's sprite RAM. srcbase is the work
// RAM as big-endian dwords
static inline uint32_t wram32(int i)
{
	const uint32_t a = (uint32_t)i * 4 & 0x1ffff;
	return (uint32_t)m68k_word(gx.workram, a) << 16 | m68k_word(gx.workram, a + 2);
}
static void esc_alert(int srcoffs, int count, int mode)
{
	static const uint8_t ztable[7][8] = {
		{ 5, 4, 3, 2, 1, 7, 6, 0 }, { 4, 3, 2, 1, 0, 7, 6, 5 }, { 4, 3, 2, 1, 0, 7, 6, 5 }, { 3, 2, 1, 0, 5, 7, 4, 6 },
		{ 6, 5, 1, 4, 3, 7, 0, 2 }, { 5, 4, 3, 2, 1, 7, 6, 0 }, { 5, 4, 3, 2, 1, 7, 6, 0 } };
	static const uint8_t ptable[7][8] = {
		{ 0x00, 0x00, 0x00, 0x10, 0x20, 0x00, 0x00, 0x30 }, { 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x20, 0x20 },
		{ 0x00, 0x00, 0x00, 0x20, 0x20, 0x00, 0x00, 0x00 }, { 0x10, 0x10, 0x10, 0x20, 0x00, 0x00, 0x10, 0x00 },
		{ 0x00, 0x00, 0x20, 0x00, 0x10, 0x00, 0x20, 0x20 }, { 0x00, 0x00, 0x00, 0x10, 0x10, 0x00, 0x00, 0x10 },
		{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10 } };
	uint16_t *dst = gx.k55673.ram;
	if (!count)
		return;
	if (mode == 0) {   // (count x 4 dwords copied as 8 words each, in order)
		for (int k = 0; k < count * 4; k++) {
			const uint32_t d = wram32(srcoffs + k);
			dst[2 * k] = (uint16_t)(d >> 16);
			dst[2 * k + 1] = (uint16_t)d;
		}
		return;
	}
	int i, j = 256, vpos, hpos, voffs, hoffs, vcorr, hcorr, vmask = 0x3ff;
	const uint32_t magicid = wram32(0x71f0 / 4);
	if (magicid != 0x11010111) {
		switch (magicid) {
		case 0x10010801: i = 6; break;
		case 0x11010010: i = 5; vmask = 0x1ff; break;
		case 0x01111018: i = 4; break;
		case 0x10010011: i = 3;
			if ((wram32(0x1c75) & 0xff) == 32)
				gx.k55555[33] = 36;   // (K55_BLEND_ENABLES: MAME's TEMPORARY)
			break;
		case 0x11010811: i = 2; break;
		case 0x10000010: i = 1; break;
		default: i = 0; break;
		}
		vcorr = (int)(wram32(0x26a0 / 4) & 0xffff);
		hcorr = (int)(wram32(0x26a4 / 4) >> 16);
		hcorr -= 10;
	} else
		hcorr = vcorr = i = 0;
	const uint8_t *zcode = ztable[i], *pcode = ptable[i];
#define OBJ(n) wram32(obj + (n))
#define EXTRACT_ODD 	if ((data1 = (int32_t)OBJ(0)) & 0x8000) { 		i = data1 & 7; data1 &= 0xff00; dst[0] = (uint16_t)(data1 | zcode[i]); 		data1 = (int32_t)OBJ(1); dst[1] = (uint16_t)(data1 >> 16); vpos = data1 & 0xffff; 		data1 = (int32_t)OBJ(2); vpos += voffs; dst[4] = (uint16_t)data1; vpos &= vmask; hpos = data1 >> 16; 		data1 = (int32_t)OBJ(3); hpos += hoffs; dst[2] = (uint16_t)vpos; dst[3] = (uint16_t)hpos; 		dst[5] = (uint16_t)(data1 >> 16); i = pcode[i]; dst[6] = (uint16_t)(data1 | i << 4); dst += 8; 		if (!(--j)) return; }
#define EXTRACT_EVEN 	if ((data1 = (int32_t)OBJ(0)) & (int32_t)0x80000000) { 		dst[1] = (uint16_t)data1; data1 >>= 16; i = data1 & 7; data1 &= 0xff00; dst[0] = (uint16_t)(data1 | zcode[i]); 		data1 = (int32_t)OBJ(1); hpos = data1 & 0xffff; vpos = data1 >> 16; hpos += hoffs; vpos += voffs; 		data1 = (int32_t)OBJ(2); vpos &= vmask; dst[3] = (uint16_t)hpos; dst[2] = (uint16_t)vpos; 		dst[5] = (uint16_t)data1; dst[4] = (uint16_t)(data1 >> 16); data1 = (int32_t)(OBJ(3) >> 16); 		i = pcode[i]; dst[6] = (uint16_t)(data1 | i << 4); dst += 8; 		if (!(--j)) return; }
	int32_t data1;
	int obj;
	if (wram32(0x049c / 4) & 0xffff0000) {   // (decode Vic-Viper)
		hoffs = (int)(wram32(0x0502 / 4) & 0xffff) - hcorr;
		voffs = (int)(wram32(0x0506 / 4) & 0xffff) - vcorr;
		obj = 0x049e / 4; EXTRACT_ODD
		obj = 0x04ae / 4; EXTRACT_ODD
		obj = 0x04be / 4; EXTRACT_ODD
	}
	if (wram32(0x0848 / 4) & 0x0000ffff) {   // (decode Lord British)
		hoffs = (int)(wram32(0x08b0 / 4) >> 16) - hcorr;
		voffs = (int)(wram32(0x08b4 / 4) >> 16) - vcorr;
		obj = 0x084c / 4; EXTRACT_EVEN
		obj = 0x085c / 4; EXTRACT_EVEN
		obj = 0x086c / 4; EXTRACT_EVEN
	}
	for (int src = srcoffs; src < srcoffs + count * 0x30; src += 0x30) {   // (the common sprites)
		if (!wram32(src))
			continue;
		i = (int)(wram32(src + 7) & 0xf);
		if (!i)
			continue;   // (retired or zero-element groups)
		i <<= 2;
		hoffs = (int)(wram32(src + 5) >> 16) - hcorr;
		voffs = (int)(wram32(src + 6) >> 16) - vcorr;
		for (obj = src + 8; obj < src + 8 + i; obj += 4) {
			EXTRACT_EVEN
		}
	}
	while (j) {   // (clear the residual data)
		*dst = 0;
		dst += 8;
		j--;
	}
#undef EXTRACT_ODD
#undef EXTRACT_EVEN
#undef OBJ
}

#define ESC_OBJECT_MAGIC_ID 0xfef724fb
static void esc_w(uint32_t data)
{
	if (!data)   // (null writes: keepalives on the real hardware)
		return;
	if (data < 0xc00000 || data > 0xc1ffff)
		return;
	uint32_t opcode = ram_rd16(data + 2) | (uint32_t)ram_rd16(data) << 16;
	if (opcode != ESC_OBJECT_MAGIC_ID)   // (ESC_INIT_CONSTANT and others: nothing)
		return;
	opcode = ram_rd8(data + 8);
	const uint32_t params = (uint32_t)ram_rd16(data + 12) << 16 | ram_rd16(data + 14);
	(void)params;
	if (opcode == 1) {   // (run the program)
		switch (s_special) {
		case SP_SEXYPARO: generate_sprites(0xc00604, 0xd20000, 0xfc); break;
		case SP_TBYAHHOO: case SP_DAISKISS: generate_sprites(0xc00000, 0xd20000, 0x100); break;
		case SP_TKMMPZDM: esc_alert(0x0142, 0x100, 0); break;
		case SP_DRAGOON: esc_alert(0x5c00, 0x100, 0); break;
		case SP_SAL2: esc_alert(0x1c8c, 0x172, 1); break;
		default: break;
		}
	}
	// (the ESTATE_END state, 2, written to the byte at data + 9, then IRQ 4)
	const uint32_t a = data + 9, w = ram_rd16(a & ~1u);
	ram_wr16(a & ~1u, (uint16_t)((a & 1) ? (w & 0xff00) | 2 : (w & 0x00ff) | 0x0200));
	if (gx.wrport1_1 & 0x10) {
		gx.rdport1_3 &= ~8;
		m68k_bus_hold_irq(4);
	}
}

// ---- fantjour's DMA (MAME konamigx_m.cpp fantjour_dma_w) ----
static void fantjour_dma_w(int offset, int hi, uint16_t v, uint16_t mask)
{
	uint32_t *r = &gx.fantjour_dma[offset & 7];
	const uint32_t m32 = hi ? (uint32_t)mask << 16 : mask, d32 = hi ? (uint32_t)v << 16 : v;
	*r = (*r & ~m32) | (d32 & m32);
	if (offset || !(m32 & 0xff000000u))
		return;
	uint32_t sa = gx.fantjour_dma[1];
	uint32_t da = (gx.fantjour_dma[3] & 0xffff) << 16 | (gx.fantjour_dma[4] & 0xffff0000) >> 16;
	const uint32_t db = gx.fantjour_dma[5], x = gx.fantjour_dma[6];
	const uint8_t sz2 = (uint8_t)(gx.fantjour_dma[0] >> 16), mode = (uint8_t)(gx.fantjour_dma[0] >> 24);
	if (mode == 0x93) {
		for (uint32_t i1 = 0; i1 <= sz2; i1++)
			for (uint32_t i2 = 0; i2 < db; i2 += 4) {
				m68k_write_memory_32(da, m68k_read_memory_32(sa) ^ x);
				da += 4;
				sa += 4;
			}
	} else if (mode == 0x8f) {
		for (uint32_t i1 = 0; i1 <= sz2; i1++)
			for (uint32_t i2 = 0; i2 < db; i2 += 4) {
				m68k_write_memory_32(da, x);
				da += 4;
			}
	}
}

// ---- the type 4 protection (MAME type4_prot_w; winspike's commands on a type 2 board) ----
static int s_prot_op = -1, s_prot_clk;
static void type4_prot_w(int offset, uint16_t hi16)
{
	if (offset == 1) {
		s_prot_op = hi16;
		return;
	}
	const int clk = hi16 & 0x200;
	if (clk == 0 && s_prot_clk != 0 && s_prot_op != -1) {
		if (s_prot_op == 0xa56 || s_prot_op == 0xd96 || s_prot_op == 0xd14 || s_prot_op == 0xd1c) {
			for (int i = 0; i < 0x400; i += 2)   // (memcpy c01000 -> c01400: the type 4 startup check)
				ram_wr16(0xc01400 + (uint32_t)i, ram_rd16(0xc01000 + (uint32_t)i));
		} else if (s_prot_op == 0x57a) {   // (winspike: the input buffers)
			static const uint32_t cp[6][2] = { { 0xc10f00, 0xc00f10 }, { 0xc10f04, 0xc00f14 }, { 0xc10f20, 0xc00f20 },
			                                   { 0xc10f24, 0xc00f24 }, { 0xc0fe00, 0xc00f30 }, { 0xc0fe04, 0xc00f34 } };
			for (int i = 0; i < 6; i++) {
				ram_wr16(cp[i][0], ram_rd16(cp[i][1]));
				ram_wr16(cp[i][0] + 2, ram_rd16(cp[i][1] + 2));
			}
		}
		if (gx.wrport1_1 & 0x10) {
			gx.rdport1_3 &= ~8;
			m68k_bus_hold_irq(4);
		}
		s_prot_op = -1;   // (no phony command)
	}
	s_prot_clk = clk;
}

// ---- the main CPU's bus ----
static uint16_t port_hi(int port, int hi) { const uint32_t v = rbx_port_read(port); return (uint16_t)(hi ? v >> 16 : v); }

static int s_light[4];   // (le2: LIGHT0_X, LIGHT0_Y, LIGHT1_X, LIGHT1_Y)
static uint16_t le2_gun_r(uint32_t a, int hi)   // (le2_gun_H_r at d44000, le2_gun_V_r at d44004)
{
	if (!(a & 4)) {
		const int p1x = (int)rbx_port_read(s_light[0]) * 290 / 0xff + 20, p2x = (int)rbx_port_read(s_light[2]) * 290 / 0xff + 20;
		return (uint16_t)(hi ? p1x : p2x);
	}
	int p1y = (int)rbx_port_read(s_light[1]) * 224 / 0xff, p2y = (int)rbx_port_read(s_light[3]) * 224 / 0xff;
	if (p1y >= 0xdf) p1y = 0;   // (off the bottom reloads too)
	if (p2y >= 0xdf) p2y = 0;
	return (uint16_t)(hi ? p1y : p2y);
}

static uint16_t main_r16(uint32_t a)
{
	a &= 0xffffff;
	const int hi = !(a & 2);
	if (s_special == SP_LE2 && a >= 0xd44000 && a < 0xd44008)
		return le2_gun_r(a, hi);
	if (a >= 0xd00000 && a < 0xd02000)   // (the tile ROM readback: byte reads)
		return 0;
	if (a >= 0xd4a000 && a < 0xd4a010)
		return k055673_rom_word_r(&gx.k55673, (int)((a - 0xd4a000) >> 1));
	if (a >= 0xd4c000 && a < 0xd4c020)
		return (uint16_t)(ccu_r((int)(((a - 0xd4c000) >> 2) * 2 + ((a >> 1) & 1))) << 8);
	if (a >= 0xd52000 && a < 0xd52020)
		return (uint16_t)(gxs_host_r((int)(((a - 0xd52000) >> 2) * 2 + ((a >> 1) & 1))) << 8);
	if (a >= 0xd5a000 && a < 0xd5a004) {
		rbx_port_custom(s_port_sys, 0x01, (uint32_t)eeprom_do(&s_eeprom));
		rbx_port_custom(s_port_sys, 0xfe, (uint32_t)(gx.rdport1_3 >> 1) << 1);
		return port_hi(s_port_sys, hi);
	}
	if (a >= 0xd5c000 && a < 0xd5c004)
		return port_hi(s_port_in, hi);
	if (a >= 0xd5e000 && a < 0xd5e004)
		return port_hi(s_port_srv, hi);
	if (a >= 0xda0000 && a < 0xda4000)
		return k056832_ram_word_r(&gx.k56832, (int)((a & 0x1fff) >> 1));
	return 0xffff;
}

static uint8_t main_r8(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0xd00000 && a < 0xd02000) {
		const uint32_t off = (a - 0xd00000) >> 2;
		return s_readback6 ? k056832_6bpp_rom_long_r(&gx.k56832, off, (int)(a & 3))
		                   : k056832_5bpp_rom_long_r(&gx.k56832, off, (int)(a & 3));
	}
	const uint16_t w = main_r16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}

static uint16_t s_esc_hi;
static void main_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	const int hi = !(a & 2);
	if (a >= 0xd40000 && a < 0xd40040)
		k056832_word_w(&gx.k56832, (int)((a - 0xd40000) >> 1), v, mask);
	else if (a >= 0xd44000 && a < 0xd44010) {   // (konamigx_tilebank_w: bytes)
		const int o = (int)(a - 0xd44000) & ~1;
		if (mask & 0xff00) gx.tilebanks[o] = (uint8_t)(v >> 8);
		if (mask & 0x00ff) gx.tilebanks[o + 1] = (uint8_t)v;
	} else if (a >= 0xd48000 && a < 0xd48008) {
		if (mask & 0xff00) k055673_k053246_w(&gx.k55673, (int)(a - 0xd48000), (uint8_t)(v >> 8));
		if (mask & 0x00ff) k055673_k053246_w(&gx.k55673, (int)(a - 0xd48000) + 1, (uint8_t)v);
	} else if (a >= 0xd4a010 && a < 0xd4a020)
		k055673_reg_word_w(&gx.k55673, (int)((a - 0xd4a010) >> 1), v, mask);
	else if (a >= 0xd4c000 && a < 0xd4c020) {   // (umask ff00ff00: the high byte of each word)
		if (mask & 0xff00)
			ccu_w((int)(((a - 0xd4c000) >> 2) * 2 + ((a >> 1) & 1)), (uint8_t)(v >> 8));
	} else if (a >= 0xd50000 && a < 0xd50100) {   // (K055555_long_w: bits 24-31 and 8-15)
		if (mask & 0xff00)
			gx.k55555[(((a - 0xd50000) >> 2) << 1) + ((a >> 1) & 1)] = (uint8_t)(v >> 8);
	} else if (a >= 0xd52000 && a < 0xd52020) {
		if (mask & 0xff00)
			gxs_host_w((int)(((a - 0xd52000) >> 2) * 2 + ((a >> 1) & 1)), (uint8_t)(v >> 8));
	} else if (a >= 0xd56000 && a < 0xd56004)
		eeprom_w(hi, v, mask);
	else if (a >= 0xd58000 && a < 0xd58004)
		control_w(hi, v, mask);
	else if (a >= 0xd80000 && a < 0xd80020) {
		uint16_t *r = &gx.k54338[(a - 0xd80000) >> 1];
		*r = (uint16_t)((v & mask) | (*r & ~mask));
	} else if (a >= 0xda0000 && a < 0xda4000)
		k056832_ram_word_w(&gx.k56832, (int)((a & 0x1fff) >> 1), v, mask);
	else if (s_special == SP_FANTJOUR && a >= 0xdb0000 && a < 0xdb0020)
		fantjour_dma_w((int)((a - 0xdb0000) >> 2), hi, v, mask);
	else if (s_special == SP_TYPE4 && a >= 0xcc0000 && a < 0xcc0008) {   // (offset 1 = cc0004; the high word = data >> 16)
		if (hi)
			type4_prot_w((int)((a - 0xcc0000) >> 2), v);
	} else if (a >= 0xcc0000 && a < 0xcc0004) {   // (esc_w: the 32-bit value, the high word first)
		if (hi)
			s_esc_hi = v;
		else
			esc_w((uint32_t)s_esc_hi << 16 | v);
	}
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v)
{
	main_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00);
}

// ---- the main CPU's wait loop (results unchanged) ----
// The Konami library's wait for the interrupts' flags (sexyparo 0x289ba8, gokuparo 0x2a0a64, tbyahhoo 0x297b98):
//   L: tst.w (An) / beq M / ...   M: tst.w (d16,An) / beq.w L
// The interrupts come only between the run slices (the vblank at a line's start, the DMA's end a slice's end), so
// with both words 0 at L nothing but the time changes until the slice ends: one turn run by Musashi (its cycles: the
// period, back at L with the same registers and flags), then the whole turns left skipped.
static struct { uint32_t l, m; int an; int16_t d16; int ok; } s_wait;

static int wait_loop_at(uint32_t pc)
{
	if (pc == s_wait.l)
		return s_wait.ok;
	s_wait.l = pc;
	s_wait.ok = 0;
	const uint16_t w0 = (uint16_t)m68k_read_memory_16(pc), w1 = (uint16_t)m68k_read_memory_16(pc + 2);
	if ((w0 & 0xfff8) != 0x4a50 || (w1 & 0xff00) != 0x6700)   // (tst.w (An), beq)
		return 0;
	uint32_t m;
	if ((w1 & 0xff) == 0)
		m = pc + 4 + (uint32_t)(int32_t)(int16_t)m68k_read_memory_16(pc + 4);
	else if ((w1 & 0xff) != 0xff)
		m = pc + 4 + (uint32_t)(int32_t)(int8_t)(w1 & 0xff);
	else
		return 0;
	const uint16_t m0 = (uint16_t)m68k_read_memory_16(m), m2 = (uint16_t)m68k_read_memory_16(m + 4);
	if (m0 != (uint16_t)(0x4a68 | (w0 & 7)) || m2 != 0x6700)   // (tst.w (d16,An), beq.w)
		return 0;
	if (m + 6 + (uint32_t)(int32_t)(int16_t)m68k_read_memory_16(m + 6) != pc)
		return 0;
	s_wait.m = m;
	s_wait.an = w0 & 7;
	s_wait.d16 = (int16_t)m68k_read_memory_16(m + 2);
	s_wait.ok = 1;
	return 1;
}

// the cycles taken (0: not at the loop)
static int wait_skip(int run)
{
	const uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC) & 0xffffff;
	if (!wait_loop_at(pc))
		return 0;
	const uint32_t an = m68k_get_reg(NULL, (m68k_register_t)(M68K_REG_A0 + s_wait.an)) & 0xffffff;
	if (m68k_read_memory_16(an) || m68k_read_memory_16(an + (uint32_t)(int32_t)s_wait.d16))
		return 0;
	int used = 0;   // (one turn by Musashi: the period)
	for (int i = 0; i < 4 && used < run; i++)
		used += m68k_execute(1);
	if ((m68k_get_reg(NULL, M68K_REG_PC) & 0xffffff) != pc || used <= 0 || used >= run)
		return used;
	const int k = (run - used) / used;
	return used + k * used;
}

// ---- init / frame ----
static void gx_exit(void);
static void gx_reset(void);

static int gx_init(const rbx_game *g)
{
	uint32_t snd_size, pcm_size, tile_size, spr_size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *snd = rbx_region("soundcpu", &snd_size);
	const uint8_t *pcm = rbx_region("k054539", &pcm_size);
	const uint8_t *tiles = rbx_region("k056832", &tile_size);
	const uint8_t *spr = rbx_region("k055673", &spr_size);
	if (!s_rom || !snd || !pcm || !tiles || !spr) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&gx, 0, sizeof(gx));
	int m = 0;
	for (int i = 0; i < (int)(sizeof(s_machines) / sizeof(s_machines[0])); i++)
		if (!strcmp(s_machines[i].machine, g->machine))
			m = i;
	gx.tilecb = s_machines[m].tilecb;
	gx.sprcb = s_machines[m].sprcb;
	gx.bands = 4;   // (the mixer's frame in four bands: the render thread and the sound worker between its jobs)
	gx.help_worker_only = 1;
	s_noskip = getenv("RBX_GX_NOSKIP") != NULL;   // (to compare)
	s_wait.l = 0xffffffff;
	k056832_init(&gx.k56832, tiles, tile_size, s_machines[m].bpp, s_machines[m].big,
	             !strcmp(g->machine, "winspike") ? 2 : 0);   // (winspike: djmain_hack 2, no layer association)
	k055673_init(&gx.k55673, spr, spr_size, s_machines[m].layout, s_machines[m].dx, s_machines[m].dy);
	s_special = SP_NONE;
	s_readback6 = 0;
	for (int i = 0; i < (int)(sizeof(s_defs) / sizeof(s_defs[0])); i++)
		if (!strcmp(s_defs[i].set, g->set)) {
			s_special = s_defs[i].special;
			s_readback6 = s_defs[i].readback6;
		}
	// (init_posthack: the 68020 at 2/3 of its clock for the POST's first 12 seconds)
	s_posthack = !strcmp(g->set, "crzcross") || !strcmp(g->set, "puzldama") || !strcmp(g->set, "tbyahhoo") ||
	             !strcmp(g->set, "mtwinbee") || !strcmp(g->set, "dragoonj") || !strcmp(g->set, "dragoona");
	{   // (the visible area MAME has after the CCU set it: dragoonj's offsets 24 + 16, 384 wide)
		const int dj = !strcmp(g->machine, "dragoonj"), ws = !strcmp(g->machine, "winspike");
		gx_vis_w = dj || ws ? 384 : 288;
		gx_video_set_area(dj ? 40 : ws ? 39 : 24, gx_vis_w);   // (winspike: set_offsets(24 + 15, 16))
	}
	if (gxs_init(snd, snd_size, pcm, pcm_size)) {
		rbx_fail("%s: sound board", g->set);
		return -1;
	}
	gxs_set_gains(g->set);
	gxs_start();
	rbx_aux_set_idle_help(1);   // (the sound worker draws bands between its jobs)
	if (s_special == SP_TKMMPZDM && s_rom_size > 0x21cbac) {   // (MAME's hack: the checksum fixed, planes B, C, D on)
		s_rom[0x2043c7] &= (uint8_t)~1;   // (rom32[0x810f1] &= ~1: the dword's lowest byte, big-endian bytes here)
		s_rom[0x21cba9] |= 0x0e;          // (rom32[0x872ea] |= 0xe0000)
	}
	// (the main CPU: its ROM swapped to host words after the sound board took its own)
	m68k_init();
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_set_cpu_type(M68K_CPU_TYPE_68EC020);
	m68k_bus_reset_map();
	m68k_bus_map(0x000000, 0x7fffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0xc00000, 0xc1ffff, gx.workram, sizeof(gx.workram), 1, 1);
	m68k_bus_map(0xd20000, 0xd23fff, gx.k55673.ram, sizeof(gx.k55673.ram), 1, 1);
	m68k_bus_map(0xd90000, 0xd97fff, gx.palram, sizeof(gx.palram), 1, 1);
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
	eeprom_init(&s_eeprom);
	{   // (the default contents from the "eeprom" region, big-endian words)
		uint32_t esz;
		const uint8_t *e = rbx_region("eeprom", &esz);
		for (uint32_t i = 0; e && i < 64 && 2 * i + 1 < esz; i++)
			s_eeprom.data[i] = (uint16_t)(e[2 * i] << 8 | e[2 * i + 1]);
	}
	s_nv_path[0] = 0;   // (the contents kept between runs: <set>.nv, MAME's layout)
	{
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			eeprom_load(&s_eeprom, s_nv_path);
		}
	}
	s_port_sys = rbx_port_find("SYSTEM_DSW");
	s_port_in = rbx_port_find("INPUTS");
	s_port_srv = rbx_port_find("SERVICE");
	s_light[0] = rbx_port_find("LIGHT0_X");
	s_light[1] = rbx_port_find("LIGHT0_Y");
	s_light[2] = rbx_port_find("LIGHT1_X");
	s_light[3] = rbx_port_find("LIGHT1_Y");
	s_mix_cap = 2048;
	s_mix_l = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_mix_r = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix_l || !s_mix_r || rbx_audio_setup_stereo(GXS_RATE) || rbx_video_setup(gx_vis_w, VIS_H, 0x10000) ||
	    gx_video_init()) {
		rbx_fail("%s: out of memory", g->set);
		gx_exit();
		return -1;
	}
	for (int i = 0; i < 0x10000; i++) {   // (the bitmap: RGB 565, for the frames the core converts)
		const int r = (i >> 11) & 31, gg = (i >> 5) & 63, b = i & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (gg << 2 | gg >> 4) << 8 | (b << 3 | b >> 2));
	}
	{   // (the machine's video start: konamigx_5bpp / konamigx_6bpp)
		int tilemode = 0, primode = 0;
		if (!strcmp(g->set, "tbyahhoo") || !strcmp(g->set, "mtwinbee"))
			tilemode = 1;
		else if (!strcmp(g->set, "crzcross") || !strcmp(g->set, "puzldama"))
			primode = 5;
		else if (!strcmp(g->set, "daiskiss"))
			primode = 4;
		if (!strcmp(g->machine, "konamigx_6bpp"))
			primode = 5;
		else if (!strcmp(g->machine, "le2"))
			primode = -1;   // (swapped layer B and C priorities)
		gx_video_set_modes(tilemode, primode);
	}
	gx_reset();
	return 0;
}

static void gx_exit(void)
{
	if (s_nv_path[0] && s_eeprom.dirty)
		eeprom_save(&s_eeprom, s_nv_path);
	rbx_aux_set_idle_help(0);
	gxs_stop();
	gx_video_exit();
	gxs_exit();
	free(s_mix_l);
	free(s_mix_r);
	s_mix_l = s_mix_r = NULL;
}

static void gx_reset(void)
{
	memset(gx.workram, 0, sizeof(gx.workram));
	gx.wrport1_0 = gx.wrport1_1 = 0;
	gx.wrport2 = 0;
	gx.rdport1_3 = 0xfc;   // (bit1 DMA busy cleared, bits 2-7 IRQ ready all set)
	gx.syncen = 0;
	gx.prev_pixel_clock = 0xff;
	gx.ccu_clock = MASTER_CLOCK / 4;
	memset(&gx.ccu, 0, sizeof(gx.ccu));
	gx.ccu.lines = 264;   // (set_raw(8000000, 472, 0, 383, 264, 0, 223) until the CCU is set; vblank from line 240)
	gx.ccu.line_clocks = 472;
	gx.ccu.clock = 8000000;
	gx.ccu.vis_y0 = 16;
	gx.ccu.vis_y1 = 239;
	gx.dma_due = -1;
	gx.cycles_done = 0;
	if (s_special == SP_LE2)   // (MAME: the video output on for the test screens)
		gx.k55555[45] = 1;
	gx.frame = 0;
	gxs_sync_reset();
	m68k_bus_irq_reset();
	m68k_pulse_reset();
}

static void gx_frame(void)
{
	const gx_ccu_t *c = &gx.ccu;
	const int lines = c->lines;
	const uint64_t line_cyc_num = (uint64_t)MASTER_CLOCK * (uint64_t)c->line_clocks;   // (cycles a line x clock)
	rbx_audio.frame_mhz = (uint32_t)((uint64_t)c->clock * 1000 / ((uint64_t)c->line_clocks * (uint64_t)lines));
	int n = 0;
	for (int line = 0; line < lines; line++) {
		gx.line = (line + c->vis_y1 + 1) % lines;   // (the frame from the vblank start, as MAME's screen at time 0)
		if (line == 0) {
			gx_video_capture();
			vblank_irq();
		}
		if (s_special == SP_LE2 && gx.line == 48 && (gx.syncen & 0x40)) {   // (konamigx_type2_scanline: the hblank IRQ 2)
			gx.syncen &= ~0x40;
			if ((gx.wrport1_1 & 0x82) == 0x82 || (gx.syncen & 2)) {
				gx.syncen &= ~2;
				m68k_bus_hold_irq(2);
			}
		}
		gx.cyc_frac += line_cyc_num;
		int64_t cyc = (int64_t)(gx.cyc_frac / c->clock);
		gx.cyc_frac -= (uint64_t)cyc * c->clock;
		if (s_posthack && gx.frame < 12u * 60u)   // (the POST hack: 2/3 of the clock for 12 seconds)
			cyc = cyc * 2 / 3;
		while (cyc > 0) {
			int run = (int)cyc;
			if (gx.dma_due >= 0 && gx.dma_due - gx.cycles_done < run)
				run = gx.dma_due - gx.cycles_done > 0 ? (int)(gx.dma_due - gx.cycles_done) : 0;
			if (run > 0) {
				const int skipped = s_noskip ? 0 : wait_skip(run);
				if (skipped) {
					gx.cycles_done += skipped;
					cyc -= skipped;
					continue;
				}
				const int used = m68k_execute(run);
				gx.cycles_done += used;
				cyc -= used;
			}
			if (gx.dma_due >= 0 && gx.cycles_done >= gx.dma_due)
				dmaend();
		}
		gx.snd_frac += (uint64_t)GXS_RATE * (uint64_t)c->line_clocks;
		const int ns = (int)(gx.snd_frac / c->clock);
		gx.snd_frac -= (uint64_t)ns * c->clock;
		gxs_advance(ns);   // (the sound board's time: run a frame behind on the worker)
	}
	n = gxs_frame(s_mix_l, s_mix_r, s_mix_cap);
	for (int i = 0; i < n; i++)
		rbx_audio_put_stereo(s_mix_l[i], s_mix_r[i]);
	if (getenv("GXDBG") && gx.frame % (getenv("GXDBG")[0] == '2' ? 10 : 60) == 0)
		fprintf(stderr, "GX f%u pc %06x sr %04x irq1en %02x syncen %02x ccu %d/%d\n", gx.frame,
		        m68k_get_reg(NULL, M68K_REG_PC), m68k_get_reg(NULL, M68K_REG_SR), gx.wrport1_1, gx.syncen, gx.ccu.lines,
		        gx.ccu.line_clocks);
	if (getenv("GXDBG") && gx.frame % 60 == 0) {
		extern uint32_t gxs_debug_pc(void);
		fprintf(stderr, "   snd %08x\n", gxs_debug_pc());
	}
	gx.frame++;
}

static int gx_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	int rc = 0;
	if ((uint32_t)m68k_context_size() > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	rc |= rbx_state_io(ctx, "gx_cpu", cpu, (uint32_t)m68k_context_size());
	rc |= rbx_state_io(ctx, "gx_bus", &m68k_bus.irq_pending, sizeof(uint32_t) * 2);
	// (gx_t up to state_end: the RAM, the board's registers and counters; the chips through their own)
	rc |= rbx_state_io(ctx, "gx", &gx, (uint32_t)offsetof(gx_t, state_end));
	rc |= k056832_state(&gx.k56832, ctx, load);
	rc |= k055673_state(&gx.k55673, ctx, load);
	rc |= rbx_state_io(ctx, "gx_eeprom", &s_eeprom, sizeof(s_eeprom));
	rc |= gxs_state(ctx, load);
	if (!rc && load)
		m68k_set_context(cpu);
	return rc ? -1 : 0;
}

static int gx_save(void *ctx) { return gx_state(ctx, 0); }
static int gx_load(void *ctx) { return gx_state(ctx, 1); }
static int gx_supported(const char *machine)
{
	// (type 2 boards with the Parodius games first, docs/KONAMI.md)
	return !strcmp(machine, "gokuparo") || !strcmp(machine, "sexyparo") || !strcmp(machine, "tbyahhoo") ||
	       !strcmp(machine, "konamigx") || !strcmp(machine, "konamigx_6bpp") || !strcmp(machine, "dragoonj") ||
	       !strcmp(machine, "salmndr2") || !strcmp(machine, "le2") || !strcmp(machine, "winspike");
}

const rbx_driver rbx_drv_konamigx = { "Konami GX", gx_init, gx_exit, gx_reset, gx_frame, gx_save, gx_load,
                                      gx_supported, gx_render, gx_render_help };
