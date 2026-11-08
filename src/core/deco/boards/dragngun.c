/*  Data East DECO 32 (DE-0360): Dragon Gun. Ported from MAME src/mame/dataeast/dragngun.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail; split from deco32.cpp) with the Namco 186/187 zooming sprites it uses:
    src/mame/shared/namco_c355spr.cpp (BSD-3-Clause, copyright-holders: David Haywood, Phil Stroffolino, Bryan McPhail).
    ARM (Data East 101) at 7 MHz; two tilemap chips 74 (the second's playfields 8 bpp), the 186/187 sprites (formats,
    tile lists and windows in RAM, sprite banks from a control register, 4-bit pixels from 16 MB of expanded ROM),
    146 protection / I/O, the IRQ controller (raster 2 and vblank), a 93C46, a buffered palette (copied on a DMA
    write), two light guns read through a port select; sound: HuC6280 at 2.685 MHz with a YM2151 (its CT port banks
    two OKIs), a third OKI on the main CPU (the gun speaker). The DVI video ROMs are mapped but not decoded (MAME too).
    Not supported: Locked 'n Loaded / Gun Hard (MAME: not working).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/arm/arm.h"
#include "../../cpu/h6280/h6280.h"
#include "../../machine/eeprom93c46.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../machine/decocrpt.h"
#include "../machine/deco146.h"
#include "../machine/deco_irq.h"
#include "../machine/armpoll.h"
#include "../video/deco16ic.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 7000000        // 28 MHz / 4
#define SND_CLOCK 2685000        // 32.22 MHz / 4 / 3
#define YM_CLOCK 3580000         // 32.22 MHz / 9
#define LINES 274                // set_raw(28 MHz / 4, 442, 0, 320, 274, 8, 248)
#define FRAME_MHZ 57800
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 320

static arm_t s_cpu;
static h6280_t s_snd;
static uint8_t *s_rom, *s_snd_rom, *s_dvi, *s_spr;
static uint32_t s_rom_size, s_snd_rom_size, s_dvi_size, s_spr_size;
static uint8_t s_ram[0x20000], s_ram3[0x8000], s_ram4[0x200];   // 0x100000, 0x1000000 (from 0x100), 0x10b0000
static uint8_t s_snd_ram[0x2000];
static uint32_t s_palram[0x800], s_pal[0x800];   // 0x130000 (xBGR_888), the colours taken at the DMA
static uint8_t s_pal_dirty[0x800];
static uint32_t s_spriteram[0x800], s_spriteram_buf[0x800];
static uint32_t s_format[2][0x400], s_spritetile[2][0x2000], s_clip[0x200], s_index[0x100];
static uint32_t s_rowscroll32[4][0x1000];
static uint16_t s_rowscroll[4][0x1000];
static uint32_t s_sprite_ctrl, s_lightgun_port, s_gun_speaker_off, s_rand;
static int s_gun[2][2];
static deco16ic_t s_tile[2];
static deco146_t s_prot;
static deco_irq_t s_irq;
static armpoll_t s_poll;   // the waits for the interrupt's flag (0x11f15c, pc 0x6288) and the vblank (IN2, pc 0x6648)
#define POLL_FLAG 0x11f15c
static eeprom93c46_t s_eeprom;
static char s_nv_path[256];
static int s_irq_raster2, s_irq_vblank;
static rbx_gfx s_gfx8, s_gfx16, s_gfx16_8bpp;
static okim6295_t s_oki[3];
static int s_port_in, s_port_sys, s_port_dsw, s_port_in2;
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac, s_snd_frac, s_ym_frac;
static int32_t *s_mix_l, *s_mix_r, *s_oki_main, *s_oki_gun;
static int s_mix_cap, s_mix_n;
// the two LC7535 electronic volumes on the same serial lines (main: address 8, both sides; gun speaker: 9)
typedef struct { uint8_t addr, count, ce, di, clk, pad[3]; uint16_t data, pad2; int32_t vol256[2]; } lc7535_t;
static lc7535_t s_vol[2];
static void lc7535_clk(lc7535_t *v, int sel, int state);
static uint32_t s_out_rate, s_oki_phase[3];
static int32_t s_oki_last[3];
static uint8_t s_pri_buf[VIS_W * VIS_H];
static uint16_t s_sprbm[VIS_W * VIS_H];
static struct {
	deco16ic_t tile[2];
	uint16_t rowscroll[4][0x800];
	uint16_t spriteram[0x800], format[2][0x400], spritetile[2][0x2000], clip[0x200], index[0x100];
	uint32_t pal[0x800], sprite_ctrl, frame;
} *s_snap;

static uint32_t rd32le(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }
static void wr32m(uint8_t *p, uint32_t data, uint32_t mask)
{
	for (int i = 0; i < 4; i++)
		if (mask & (0xffu << (8 * i)))
			p[i] = (uint8_t)(data >> (8 * i));
}
static inline void combine16(uint16_t *p, uint32_t data, uint32_t mask)
{
	mask &= 0xffff;
	*p = (uint16_t)((*p & ~mask) | (data & mask));
}
static inline void combine32(uint32_t *p, uint32_t data, uint32_t mask) { *p = (*p & ~mask) | (data & mask); }

// ---- interrupts ----
static void arm_irq_update(void) { arm_set_irq(&s_cpu, s_irq_raster2 || s_irq_vblank); }   // (input merger, any high)
static void irq_raster2(void *u, int state) { (void)u; s_irq_raster2 = state; arm_irq_update(); }
static void irq_vblank(void *u, int state) { (void)u; s_irq_vblank = state; arm_irq_update(); }
static void latch_irq(void *u, int state) { (void)u; h6280_set_irq(&s_snd, H6280_IRQ1, state); }
static void ym_irq(int state) { h6280_set_irq(&s_snd, H6280_IRQ2, state); }

static int vblank(void) { return s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H; }

// ---- 146 ports ----
static uint16_t port_a(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_in); }
static uint16_t port_b(void *u)
{
	(void)u;
	rbx_port_custom(s_port_sys, 0x08, vblank() ? 0x08 : 0);
	return (uint16_t)rbx_port_read(s_port_sys);
}
static uint16_t port_c(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_dsw); }

static uint16_t prot_addr(uint32_t o)
{
	const uint32_t real = (o >> 2) * 2;
	return (uint16_t)((((real >> 14) & 0xf) << 11) | (real & 0x7ff));
}

static uint32_t tile_r(deco16ic_t *t, uint32_t o)
{
	if (o < 0x20) return deco16ic_control_r(t, (int)(o >> 2)) | 0xffff0000u;
	if (o >= 0x10000 && o < 0x12000) return t->vram[0][(o - 0x10000) >> 2] | 0xffff0000u;
	if (o >= 0x14000 && o < 0x16000) return t->vram[1][(o - 0x14000) >> 2] | 0xffff0000u;
	return 0;
}
static void tile_w(deco16ic_t *t, uint32_t o, uint32_t data, uint32_t mask)
{
	if (o < 0x20) {
		if (mask & 0xffff)
			deco16ic_control_w(t, (int)(o >> 2), (uint16_t)data, (uint16_t)mask);
	} else if (o >= 0x10000 && o < 0x12000) combine16(&t->vram[0][(o - 0x10000) >> 2], data, mask);
	else if (o >= 0x14000 && o < 0x16000) combine16(&t->vram[1][(o - 0x14000) >> 2], data, mask);
}

static int rowscroll_index(uint32_t a, int *idx)
{
	static const uint32_t base[4] = { 0x1a0000, 0x1a4000, 0x1e0000, 0x1e4000 };
	static const uint32_t size[4] = { 0x4000, 0x2000, 0x4000, 0x2000 };
	for (int i = 0; i < 4; i++)
		if (a >= base[i] && a < base[i] + size[i]) {
			*idx = (int)((a - base[i]) >> 2);
			return i;
		}
	return -1;
}

// the light guns: the crosshairs (0-255) moved by the pointing device's deltas
static uint32_t lightgun_r(void)
{
	switch (s_lightgun_port) {
	case 4: return (uint32_t)s_gun[0][0];
	case 5: return (uint32_t)s_gun[1][0];
	case 6: return (uint32_t)s_gun[0][1];
	case 7: return (uint32_t)s_gun[1][1];
	}
	return 0;
}

static uint32_t *sprite_ram(uint32_t a, uint32_t *n)
{
	if (a >= 0x204800 && a < 0x205000) { *n = (a - 0x204800) >> 2; return s_clip; }
	if (a >= 0x208000 && a < 0x209000) { *n = (a - 0x208000) >> 2; return s_format[0]; }
	if (a >= 0x20c000 && a < 0x20d000) { *n = (a - 0x20c000) >> 2; return s_format[1]; }
	if (a >= 0x210000 && a < 0x218000) { *n = (a - 0x210000) >> 2; return s_spritetile[0]; }
	if (a >= 0x218000 && a < 0x220000) { *n = (a - 0x218000) >> 2; return s_spritetile[1]; }
	if (a >= 0x220000 && a < 0x222000) { *n = (a - 0x220000) >> 2; return s_spriteram; }
	if (a >= 0x228000 && a < 0x228400) { *n = (a - 0x228000) >> 2; return s_index; }
	return NULL;
}

static uint32_t bus_r32(void *ctx, uint32_t a)
{
	(void)ctx;
	a &= 0x03fffffc;
	if (a < 0x100000) return rd32le(s_rom + a);
	if (a < 0x120000) {
		const uint32_t v = rd32le(s_ram + (a & 0x1fffc));
		if (a == (POLL_FLAG & ~3u))
			armpoll_read(&s_poll, &s_cpu, v, s_rom, 0x100000);
		return v;
	}
	if (a < 0x128000) return deco146_read(&s_prot, prot_addr(a - 0x120000), NULL) | 0xffff0000u;
	if (a < 0x128010) return 0xffffff00u | deco_irq_r(&s_irq, (int)((a - 0x128000) >> 2), vblank(), 0);
	if (a >= 0x130000 && a < 0x132000) return s_palram[(a - 0x130000) >> 2];
	if (a >= 0x180000 && a < 0x1a0000) return tile_r(&s_tile[0], a - 0x180000);
	if (a >= 0x1c0000 && a < 0x1e0000) return tile_r(&s_tile[1], a - 0x1c0000);
	int idx;
	const int rs = rowscroll_index(a, &idx);
	if (rs >= 0)
		return s_rowscroll32[rs][idx];
	uint32_t n;
	uint32_t *sp = sprite_ram(a, &n);
	if (sp)
		return sp[n];
	if (a >= 0x300000 && a < 0x400000) return rd32le(s_rom + 0x100000 + (a - 0x300000));
	if (a == 0x400000) return 0xffffff00u | oki_read(&s_oki[2]);
	if (a == 0x420000) return 0xfe | (uint32_t)eeprom_do(&s_eeprom);
	if (a == 0x438000) return lightgun_r();
	if (a == 0x440000) {
		rbx_port_custom(s_port_in2, 0x01, vblank() ? 1 : 0);
		const uint32_t v = rbx_port_read(s_port_in2) | 0xffffff00u;
		armpoll_read(&s_poll, &s_cpu, v, s_rom, 0x100000);
		return v;
	}
	if (a >= 0x1000000 && a < 0x1000008) {   // unk_video_r: machine().rand()
		s_rand = s_rand * 1103515245u + 12345u;
		return s_rand;
	}
	if (a >= 0x1000100 && a < 0x1008000) return rd32le(s_ram3 + (a & 0x7ffc));
	if (a >= 0x10b0000 && a < 0x10b0200) return rd32le(s_ram4 + (a & 0x1fc));
	if (a >= 0x1400000 && a < 0x2000000) {
		const uint32_t o = a - 0x1400000;
		return o + 4 <= s_dvi_size ? rd32le(s_dvi + o) : 0;
	}
	return 0;
}

static void bus_w32m(uint32_t a, uint32_t data, uint32_t mask)
{
	a &= 0x03fffffc;
	if (a >= 0x100000 && a < 0x120000) { wr32m(s_ram + (a & 0x1fffc), data, mask); return; }
	if (a >= 0x120000 && a < 0x128000) {
		if (mask & 0xffff)
			deco146_write(&s_prot, prot_addr(a - 0x120000), (uint16_t)data, (uint16_t)mask, NULL);
		return;
	}
	if (a >= 0x128000 && a < 0x128010) {
		if (mask & 0xff)
			deco_irq_w(&s_irq, (int)((a - 0x128000) >> 2), (uint8_t)data);
		return;
	}
	if (a >= 0x130000 && a < 0x132000) {   // buffered_palette_w
		const uint32_t i = (a - 0x130000) >> 2;
		combine32(&s_palram[i], data, mask);
		s_pal_dirty[i] = 1;
		return;
	}
	if (a == 0x138008) {   // palette_dma_w: the dirty colours taken
		for (int i = 0; i < 0x800; i++)
			if (s_pal_dirty[i]) {
				s_pal_dirty[i] = 0;
				s_pal[i] = s_palram[i];
			}
		return;
	}
	if (a >= 0x180000 && a < 0x1a0000) { tile_w(&s_tile[0], a - 0x180000, data, mask); return; }
	if (a >= 0x1c0000 && a < 0x1e0000) { tile_w(&s_tile[1], a - 0x1c0000, data, mask); return; }
	int idx;
	const int rs = rowscroll_index(a, &idx);
	if (rs >= 0) {
		combine32(&s_rowscroll32[rs][idx], data, mask);
		combine16(&s_rowscroll[rs][idx], data, mask);
		return;
	}
	uint32_t n;
	uint32_t *sp = sprite_ram(a, &n);
	if (sp) {
		combine32(&sp[n], data, mask);
		return;
	}
	if (a == 0x230000) {   // spriteram_dma_w: the list to the chip, the CPU's copy cleared
		memcpy(s_spriteram_buf, s_spriteram, sizeof(s_spriteram));
		memset(s_spriteram, 0, sizeof(s_spriteram));
		return;
	}
	if (a == 0x400000) {
		if (mask & 0xff)
			oki_write(&s_oki[2], (uint8_t)data);
		return;
	}
	if (a == 0x410000) {   // lc7535_volume_w: ce, clk, di to both
		if (mask & 0xff)
			for (int k = 0; k < 2; k++) {
				s_vol[k].ce = (data >> 2) & 1;
				lc7535_clk(&s_vol[k], !k, (data >> 1) & 1);
				s_vol[k].di = data & 1;
			}
		return;
	}
	if (a == 0x418000) { s_gun_speaker_off = data & 1; return; }   // speaker_switch_w
	if (a == 0x420000) {   // eeprom_w
		if (mask & 0xff)
			eeprom_write_lines(&s_eeprom, (data >> 2) & 1, (data >> 1) & 1, data & 1);
		return;
	}
	if (a >= 0x430000 && a < 0x430020) { s_lightgun_port = (a - 0x430000) >> 2; return; }
	if (a == 0x500000) { combine32(&s_sprite_ctrl, data, mask); return; }
	if (a >= 0x1000100 && a < 0x1008000) { wr32m(s_ram3 + (a & 0x7ffc), data, mask); return; }
	if (a >= 0x10b0000 && a < 0x10b0200) { wr32m(s_ram4 + (a & 0x1fc), data, mask); return; }
}

static void bus_w32(void *ctx, uint32_t a, uint32_t data) { (void)ctx; bus_w32m(a, data, 0xffffffffu); }
static uint8_t bus_r8(void *ctx, uint32_t a) { return (uint8_t)(bus_r32(ctx, a) >> (8 * (a & 3))); }
static void bus_w8(void *ctx, uint32_t a, uint8_t v)
{
	(void)ctx;
	const int sh = 8 * (int)(a & 3);
	bus_w32m(a, (uint32_t)v << sh, 0xffu << sh);
}

// ---- sound CPU ----
static uint8_t snd_r(void *ctx, uint32_t pa)
{
	(void)ctx;
	if (pa < 0x10000) return s_snd_rom[pa & (s_snd_rom_size - 1)];
	if ((pa & ~1u) == 0x110000) return rbx_ym_read(0, pa & 1);
	if ((pa & ~1u) == 0x120000) return oki_read(&s_oki[0]);
	if ((pa & ~1u) == 0x130000) return oki_read(&s_oki[1]);
	if (pa == 0x140000) return deco146_soundlatch_r(&s_prot);
	if (pa >= 0x1f0000 && pa < 0x1f2000) return s_snd_ram[pa & 0x1fff];
	return 0;
}
static void snd_w(void *ctx, uint32_t pa, uint8_t v)
{
	(void)ctx;
	if ((pa & ~1u) == 0x110000) rbx_ym_write(0, pa & 1, v);
	else if ((pa & ~1u) == 0x120000) oki_write(&s_oki[0], v);
	else if ((pa & ~1u) == 0x130000) oki_write(&s_oki[1], v);
	else if (pa >= 0x1f0000 && pa < 0x1f2000) s_snd_ram[pa & 0x1fff] = v;
}
static void lc7535_clk(lc7535_t *v, int sel, int state)
{
	static const int s5db[16] = { -75, -70, -65, -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0 };
	static const int s1db[8] = { -98, -98, -4, -3, -2, -1, 0, -98 };
	if (!v->clk && state) {
		if (!v->ce) {
			if (v->count == 0)
				v->addr = 0;
			v->addr = (uint8_t)((v->addr >> 1) | (v->di << 3));
			if (++v->count == 4)
				v->count = 0;
		} else if (v->addr == (8 | (sel ? 0 : 1))) {
			v->data = (uint16_t)((v->data >> 1) | (v->di << 15));
			if (++v->count == 16) {
				int att_l = s1db[(v->data >> 12) & 7], att_r = s1db[(v->data >> 4) & 7];
				if (att_l != -98) att_l += s5db[(v->data >> 8) & 0x0f];
				if (att_r != -98) att_r += s5db[v->data & 0x0f];
				v->vol256[0] = (att_l + 98) * 256 / 98;   // (MAME's linear scale)
				v->vol256[1] = (att_r + 98) * 256 / 98;
				v->addr = 0;
				v->count = 0;
			}
		}
	}
	v->clk = (uint8_t)state;
}

static void ym_port(uint8_t data)   // sound_bankswitch_w
{
	s_oki[0].bank = (data & 1) * 0x40000u;
	s_oki[1].bank = ((data >> 1) & 1) * 0x40000u;
}

static uint32_t s_ym_line_clocks;
static void mix_ym_line(void)
{
	const int n = rbx_ym_advance_lr(0, s_ym_line_clocks, s_mix_l + s_mix_n, s_mix_r + s_mix_n, s_mix_cap - s_mix_n);
	for (int i = 0; i < n; i++) {   // the OKIs (mono to both sides) at their own rates, held; the third to the main
		for (int k = 0; k < 3; k++) {   // or (speaker_switch_w) to the gun speaker
			s_oki_phase[k] += s_oki[k].rate;
			while (s_oki_phase[k] >= s_out_rate) {
				s_oki_phase[k] -= s_out_rate;
				s_oki_last[k] = oki_sample(&s_oki[k]);
			}
		}
		s_oki_main[s_mix_n + i] = s_oki_last[0] + s_oki_last[1] + (s_gun_speaker_off ? s_oki_last[2] : 0);
		s_oki_gun[s_mix_n + i] = s_gun_speaker_off ? 0 : s_oki_last[2];
	}
	s_mix_n += n;
}

// ---- the 186/187 sprites (namco_c355spr: alt precision, transparent pen 15, 32 colours of 16) ----
static int sprite_bank(int sprite, uint32_t ctrl)   // sprite_bank_callback
{
	switch (sprite & 0x3000) {
	default:
	case 0x0000: sprite = (sprite & 0xfff) | (int)((ctrl & 0x000f) << 12); break;
	case 0x1000: sprite = (sprite & 0xfff) | (int)((ctrl & 0x00f0) << 8); break;
	case 0x2000: sprite = (sprite & 0xfff) | (int)((ctrl & 0x0f00) << 4); break;
	case 0x3000: sprite = (sprite & 0xfff) | (int)(ctrl & 0xf000); break;
	}
	switch (sprite & 0xf000) {
	case 0x0000: return 0xc000 | (sprite & 0xfff);
	case 0x1000: return 0xd000 | (sprite & 0xfff);
	case 0x2000: return 0xe000 | (sprite & 0xfff);
	case 0x3000: return 0xf000 | (sprite & 0xfff);
	case 0xc000: return 0x0000 | (sprite & 0xfff);
	case 0xd000: return 0x1000 | (sprite & 0xfff);
	case 0xe000: return 0x2000 | (sprite & 0xfff);
	case 0xf000: return 0x3000 | (sprite & 0xfff);
	}
	return sprite;
}

static inline int sext(int v, int bits) { const int m = 1 << (bits - 1); return ((v & ((1 << bits) - 1)) ^ m) - m; }

static void zdraw(const int clip[4], uint32_t code, uint32_t color, int flipx, int flipy, int hpos, int vpos,
                  int hsize, int vsize, int prival, int sw, int sh)
{
	if (!hsize || !vsize || !sw || !sh)
		return;
	const uint32_t tiles = s_spr_size >> 8;
	const uint8_t *src = s_spr + (size_t)(code % tiles) * 256;
	const int pal = 16 * (int)(color % 32);
	int dx = (16 << 16) / sw, dy = (16 << 16) / sh, ex = hpos + sw, ey = vpos + sh, xb = 0, yi = 0;
	if (flipx) { xb = (sw - 1) * dx; dx = -dx; }
	if (flipy) { yi = (sh - 1) * dy; dy = -dy; }
	if (hpos < clip[0]) { xb += (clip[0] - hpos) * dx; hpos = clip[0]; }
	if (vpos < clip[2]) { yi += (clip[2] - vpos) * dy; vpos = clip[2]; }
	if (ex > clip[1] + 1) ex = clip[1] + 1;
	if (ey > clip[3] + 1) ey = clip[3] + 1;
	if (ex <= hpos)
		return;
	for (int y = vpos; y < ey; y++, yi += dy) {
		const uint8_t *s = src + (yi >> 16) * 16;
		uint16_t *d = s_sprbm + (y - VIS_Y0) * VIS_W;
		int xi = xb;
		for (int x = hpos; x < ex; x++, xi += dx) {
			const uint8_t c = s[xi >> 16];
			if (c != 15)
				d[x] = (uint16_t)(((prival & 0xf) << 12) | ((pal + c) & 0xfff));
		}
	}
}

static void c355_render(void)
{
	for (int i = 0; i < VIS_W * VIS_H; i++)
		s_sprbm[i] = 0xffff;
	const uint32_t ctrl = s_snap->sprite_ctrl;
	for (int i = 0; i < 256; i++) {
		const uint16_t which16 = s_snap->index[i];
		const int which = which16 & 0xff;
		const uint16_t *st = &s_snap->spriteram[which << 3];
		const uint16_t palette = st[6];
		if ((palette & 0x80) && (s_snap->frame & 1))   // (sprite_priority_callback: the flicker)
			goto next;
		{
			const int pri = 7;
			const int fmt_off = st[0] & 0x7ff;
			const int offset = st[1];
			int hpos = st[2], vpos = st[3];
			uint16_t hsize = st[4], vsize = st[5];
			const int clipentry = (palette >> 8) & 0xf;
			const uint16_t *cl = &s_snap->clip[clipentry << 2];
			int clip[4] = { cl[0], cl[1], cl[2], cl[3] };   // (min x, max x, min y, max y; no scroll)
			if (clip[0] < 0) clip[0] = 0;
			if (clip[1] > VIS_W - 1) clip[1] = VIS_W - 1;
			if (clip[2] < VIS_Y0) clip[2] = VIS_Y0;
			if (clip[3] > VIS_Y0 + VIS_H - 1) clip[3] = VIS_Y0 + VIS_H - 1;
			hpos = sext(hpos & 0x7ff, 11);
			vpos = sext(vpos & 0x7ff, 11);
			const uint16_t *fm = &s_snap->format[(fmt_off & 0x400) ? 1 : 0][(fmt_off & 0x1ff) << 2];
			int tile_index = fm[0];
			const uint16_t format = fm[1];
			const int dxo = fm[2] & 0x1ff, dyo = fm[3] & 0x1ff;
			int num_cols = (format >> 4) & 0xf, num_rows = format & 0xf;
			if (num_cols == 0) num_cols = 16;
			const int flipx = (hsize & 0x8000) != 0;
			hsize &= 0x3ff;
			if (hsize == 0)
				goto next;
			uint32_t zoomx = ((uint32_t)hsize << 16) / (uint32_t)(num_cols * 16);
			int32_t dxz = (int32_t)(((uint32_t)(dxo & 0xff) * zoomx + 0x8000) >> 16);
			if (dxo & 0x100) dxz = -dxz;
			hpos += flipx ? dxz : -dxz;
			if (num_rows == 0) num_rows = 16;
			const int flipy = (vsize & 0x8000) != 0;
			vsize &= 0x3ff;
			if (vsize == 0)
				goto next;
			uint32_t zoomy = ((uint32_t)vsize << 16) / (uint32_t)(num_rows * 16);
			int32_t dyz = (int32_t)(((uint32_t)(dyo & 0xff) * zoomy + 0x8000) >> 16);
			if (dyo & 0x100) dyz = -dyz;
			vpos += flipy ? dyz : -dyz;
			const int color = palette & 31;
			uint32_t src_h = (uint32_t)num_rows * 16, scr_h = vsize;
			int y = vpos;
			for (int row = 0; row < num_rows; row++) {
				const int th = (int)(16 * scr_h / src_h);
				zoomy = (scr_h << 16) / src_h;
				if (flipy) y -= th;
				uint32_t src_w = (uint32_t)num_cols * 16, scr_w = hsize;
				int x = hpos;
				for (int col = 0; col < num_cols; col++) {
					const int tw = (int)(16 * scr_w / src_w);
					zoomx = (scr_w << 16) / src_w;
					if (flipx) x -= tw;
					const int ti = tile_index++;
					const uint16_t tile = s_snap->spritetile[(ti & 0x2000) ? 1 : 0][ti & 0x1fff];
					if (!(tile & 0x8000)) {
						// (alt precision: the screen size from the 16.16 positions)
						const int sxw = (int)((((int64_t)x << 16) + ((int64_t)zoomx << 4)) >> 16) - x;
						const int syh = (int)((((int64_t)y << 16) + ((int64_t)zoomy << 4)) >> 16) - y;
						zdraw(clip, (uint32_t)(sprite_bank(tile, ctrl) + offset), (uint32_t)color, flipx, flipy, x, y,
						      (int)zoomx, (int)zoomy, pri, sxw, syh);
					}
					if (!flipx) x += tw;
					scr_w -= (uint32_t)tw;
					src_w -= 16;
				}
				if (!flipy) y += th;
				scr_h -= (uint32_t)th;
				src_h -= 16;
			}
		}
	next:
		if (which16 & 0x100)
			break;
	}
}

// ---- video ----
static int bank_1_cb(int bank) { return (bank & ~0xf) << 8; }
static int bank_2_cb(int bank) { return (bank & ~0x1f) << 7; }

static void dg_render(void)
{
	const uint32_t *p = s_snap->pal;
	for (int i = 0; i < 0x800; i++)   // xBGR_888
		rbx_video.palette[i] = (p[i] & 0xff) << 16 | (p[i] & 0xff00) | ((p[i] >> 16) & 0xff);
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = 0; y < VIS_H; y++)
		for (int x = 0; x < VIS_W; x++)
			bm[y * pitch + x] = 0x400;   // (pen 0x400)
	memset(s_pri_buf, 0, sizeof(s_pri_buf));
	deco16ic_t *t0 = &s_snap->tile[0], *t1 = &s_snap->tile[1];
	deco16ic_draw(t1, 1, s_snap->rowscroll[3], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 1, 0xff);
	deco16ic_draw(t1, 0, s_snap->rowscroll[2], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 2, 0xff);
	deco16ic_draw(t0, 1, s_snap->rowscroll[1], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 4, 0xff);
	deco16ic_draw(t0, 0, s_snap->rowscroll[0], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 8, 0xff);
	c355_render();
	for (int y = 0; y < VIS_H; y++)   // sprite_mix_callback: a sprite pixel over priorities up to its own
		for (int x = 0; x < VIS_W; x++) {
			const uint16_t s = s_sprbm[y * VIS_W + x];
			if (s == 0xffff)
				continue;
			const uint8_t sp = (uint8_t)(s >> 12);
			uint8_t *dp = &s_pri_buf[y * VIS_W + x];
			if (sp >= *dp && (s & 0xf) != 0xf) {
				bm[y * pitch + x] = s & 0xfff;
				*dp = sp;
			}
		}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->tile, s_tile, sizeof(s_tile));
	for (int i = 0; i < 4; i++)
		memcpy(s_snap->rowscroll[i], s_rowscroll[i], sizeof(s_snap->rowscroll[i]));
	for (int i = 0; i < 0x800; i++)
		s_snap->spriteram[i] = (uint16_t)s_spriteram_buf[i];
	for (int b = 0; b < 2; b++) {
		for (int i = 0; i < 0x400; i++)
			s_snap->format[b][i] = (uint16_t)s_format[b][i];
		for (int i = 0; i < 0x2000; i++)
			s_snap->spritetile[b][i] = (uint16_t)s_spritetile[b][i];
	}
	for (int i = 0; i < 0x200; i++)
		s_snap->clip[i] = (uint16_t)s_clip[i];
	for (int i = 0; i < 0x100; i++)
		s_snap->index[i] = (uint16_t)s_index[i];
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	s_snap->sprite_ctrl = s_sprite_ctrl;
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void dg_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_ram3, 0, sizeof(s_ram3));
	memset(s_ram4, 0, sizeof(s_ram4));
	memset(s_snd_ram, 0, sizeof(s_snd_ram));
	memset(s_palram, 0, sizeof(s_palram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_pal_dirty, 0, sizeof(s_pal_dirty));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_spriteram_buf, 0, sizeof(s_spriteram_buf));
	memset(s_format, 0, sizeof(s_format));
	memset(s_spritetile, 0, sizeof(s_spritetile));
	memset(s_clip, 0, sizeof(s_clip));
	memset(s_index, 0, sizeof(s_index));
	memset(s_rowscroll32, 0, sizeof(s_rowscroll32));
	memset(s_rowscroll, 0, sizeof(s_rowscroll));
	s_sprite_ctrl = s_lightgun_port = s_gun_speaker_off = 0;
	memset(s_vol, 0, sizeof(s_vol));
	for (int k = 0; k < 2; k++)   // (MAME leaves the volumes unset until written: full)
		s_vol[k].vol256[0] = s_vol[k].vol256[1] = 256;
	s_rand = 0;
	for (int p = 0; p < 2; p++)
		s_gun[p][0] = s_gun[p][1] = 0x80;
	for (int i = 0; i < 2; i++) {
		memset(s_tile[i].vram, 0, sizeof(s_tile[i].vram));
		deco16ic_reset(&s_tile[i]);
	}
	for (int i = 0; i < 3; i++) {
		oki_reset(&s_oki[i]);
		s_oki[i].bank = 0;
		s_oki_phase[i] = 0;
		s_oki_last[i] = 0;
	}
	deco146_reset(&s_prot);
	deco_irq_reset(&s_irq);
	armpoll_reset(&s_poll);
	s_irq_raster2 = s_irq_vblank = 0;
	rbx_ym_reset(0);
	arm_reset(&s_cpu);
	h6280_reset(&s_snd);
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = s_snd_frac = s_ym_frac = 0;
	s_mix_n = 0;
}

static void dg_exit(void);

static int dg_init(const rbx_game *g)
{
	uint32_t t1s, t2s, t3s, o1s, o2s, o3s;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_snd_rom = rbx_region("audiocpu", &s_snd_rom_size);
	s_spr = rbx_region("c355spr", &s_spr_size);
	s_dvi = rbx_region("dvi", &s_dvi_size);
	uint8_t *t1 = rbx_region("tiles1", &t1s), *t2 = rbx_region("tiles2", &t2s), *t3 = rbx_region("tiles3", &t3s);
	uint8_t *oki1 = rbx_region("oki1", &o1s), *oki2 = rbx_region("oki2", &o2s), *oki3 = rbx_region("oki3", &o3s);
	if (!s_rom || s_rom_size < 0x200000 || !s_snd_rom || !s_spr || !t1 || t1s < 0x20000 || !t2 || t2s < 0x120000 ||
	    !t3 || !oki1 || !oki2 || !oki3) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (!s_dvi)
		s_dvi_size = 0;
	// dragngun_init_common: the 74's graphics decrypted, the characters copied into the tiles' gaps, the sprite
	// ROMs' nibbles expanded (the high ones the first half)
	if (deco74_decrypt_gfx(t1, t1s) || deco74_decrypt_gfx(t2, t2s) || deco74_decrypt_gfx(t3, t3s)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	memcpy(t2 + 0x080000, t1, 0x10000);
	memcpy(t2 + 0x110000, t1 + 0x10000, 0x10000);
	for (uint32_t i = 0; i < s_spr_size / 2; i++) {
		const uint8_t v = s_spr[i];
		s_spr[i + s_spr_size / 2] = v & 0x0f;
		s_spr[i] = (v >> 4) & 0x0f;
	}
	// the test switch lock NOPped (bl $ee000)
	{
		const uint32_t at = !strcmp(g->set, "dragngunj") ? 0x01a1b4 : 0x01b32c;
		s_rom[at] = 0x00;
		s_rom[at + 1] = 0x00;
		s_rom[at + 2] = 0xa0;
		s_rom[at + 3] = 0xe1;
	}
	{	// charlayout / tilelayout (RGN_FRAC(1,2)), tilelayout_8bpp (RGN_FRAC(1,4), 8 planes)
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int k = 0; k < 8; k++) {
			x8[k] = (uint32_t)k;
			y8[k] = (uint32_t)(k * 16);
			x16[k] = (uint32_t)(256 + k);
			x16[8 + k] = (uint32_t)k;
		}
		for (int k = 0; k < 16; k++)
			y16[k] = (uint32_t)(k * 16);
		const uint32_t h1 = t1s * 4, h2 = t2s * 4, q3 = t3s * 2;
		const uint32_t p1[4] = { h1 + 8, h1, 8, 0 }, p2[4] = { h2 + 8, h2, 8, 0 };
		const uint32_t p8[8] = { 3 * q3 + 8, 3 * q3, 2 * q3 + 8, 2 * q3, q3 + 8, q3, 8, 0 };
		const rbx_gfx_layout l8 = { 8, 8, 4, p1, x8, y8, 16 * 8 }, l16 = { 16, 16, 4, p2, x16, y16, 64 * 8 };
		const rbx_gfx_layout l16b = { 16, 16, 8, p8, x16, y16, 64 * 8 };
		if (rbx_gfx_decode(&s_gfx8, &l8, t1, t1s / 2) || rbx_gfx_decode(&s_gfx16, &l16, t2, t2s / 2) ||
		    rbx_gfx_decode(&s_gfx16_8bpp, &l16b, t3, t3s / 4)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			dg_exit();
			return -1;
		}
	}
	memset(s_tile, 0, sizeof(s_tile));
	static const int cbank[2][2] = { { 0x20, 0x30 }, { 0x04, 0x04 } }, cmask[2][2] = { { 0x0f, 0x0f }, { 0x03, 0x03 } };
	for (int c = 0; c < 2; c++)
		for (int i = 0; i < 2; i++) {
			s_tile[c].tm[i].size = DECO_64x32;
			s_tile[c].tm[i].col_bank = cbank[c][i];
			s_tile[c].tm[i].col_mask = cmask[c][i];
		}
	s_tile[0].tm[0].bank_cb = s_tile[0].tm[1].bank_cb = bank_1_cb;
	s_tile[1].tm[0].bank_cb = bank_2_cb;
	s_tile[0].gfx8 = s_tile[1].gfx8 = &s_gfx8;   // (8x8: entry 0)
	s_tile[0].gfx16 = &s_gfx16;                  // 16x16: entry 1
	s_tile[1].gfx16 = &s_gfx16_8bpp;             // entry 2 (8 bpp)
	s_tile[1].gran16 = 8;
	deco146_init(&s_prot, 0);
	static const uint8_t reverse[10] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };   // set_interface_scramble_reverse
	deco146_set_scramble(&s_prot, reverse);
	s_prot.port_a = port_a;
	s_prot.port_b = port_b;
	s_prot.port_c = port_c;
	s_prot.soundlatch_irq = latch_irq;
	memset(&s_irq, 0, sizeof(s_irq));
	s_irq.vis_top = VIS_Y0;
	s_irq.vis_bottom = VIS_Y0 + VIS_H - 1;
	s_irq.raster2 = irq_raster2;
	s_irq.vblank = irq_vblank;
	s_nv_path[0] = 0;
	eeprom_init(&s_eeprom);
	{
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			eeprom_load(&s_eeprom, s_nv_path);
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	// sound: YM2151 0.42 / 0.42, OKI 1 MHz 1.0, OKI 2 MHz 0.35, OKI 3 (1 MHz) 1.0 (both sides)
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 108;
	yc.irq = ym_irq;
	yc.port_a_write = ym_port;
	oki_init(&s_oki[0], 32220000 / 32, 1, oki1, o1s, 256);
	oki_init(&s_oki[1], 32220000 / 16, 1, oki2, o2s, 90);
	oki_init(&s_oki[2], 32220000 / 32, 1, oki3, o3s, 256);
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 2)) {
		rbx_fail("%s: out of memory", g->set);
		dg_exit();
		return -1;
	}
	s_out_rate = rbx_ym_rate(0);
	s_mix_cap = (int)(s_out_rate / 50) + 256;
	s_mix_l = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_mix_r = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_oki_main = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_oki_gun = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix_l || !s_mix_r || !s_oki_main || !s_oki_gun || rbx_audio_setup_stereo(s_out_rate) || rbx_video_setup(VIS_W, VIS_H, 0x800)) {
		dg_exit();
		return -1;
	}
	s_port_in = rbx_port_find("INPUTS");
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_dsw = rbx_port_find("DSW");
	s_port_in2 = rbx_port_find("IN2");
	arm_init(&s_cpu);
	for (uint32_t a = 0; a < 0x100000; a += 0x1000)
		s_cpu.rpage[a >> 12] = s_rom + a;
	for (uint32_t a = 0; a < 0x20000; a += 0x1000)
		s_cpu.rpage[(0x100000 + a) >> 12] = s_cpu.wpage[(0x100000 + a) >> 12] = s_ram + a;
	for (uint32_t a = 0; a < 0x100000; a += 0x1000)
		s_cpu.rpage[(0x300000 + a) >> 12] = s_rom + 0x100000 + a;
	s_cpu.rpage[POLL_FLAG >> 12] = NULL;   // (the page with the polled flag: bus_r32)
	s_cpu.read32 = bus_r32;
	s_cpu.write32 = bus_w32;
	s_cpu.read8 = bus_r8;
	s_cpu.write8 = bus_w8;
	h6280_init(&s_snd);
	for (uint32_t a = 0; a < 0x10000 && a < s_snd_rom_size; a += 0x2000)
		s_snd.rpage[a >> 13] = s_snd_rom + a;
	s_snd.rpage[0x1f0000 >> 13] = s_snd.wpage[0x1f0000 >> 13] = s_snd_ram;
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	dg_reset();
	return 0;
}

static void dg_exit(void)
{
	if (s_nv_path[0] && s_eeprom.dirty)
		eeprom_save(&s_eeprom, s_nv_path);
	s_nv_path[0] = 0;
	rbx_ym_exit(0);
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16);
	rbx_gfx_free(&s_gfx16_8bpp);
	free(s_snap);
	s_snap = NULL;
	free(s_mix_l);
	free(s_mix_r);
	free(s_oki_main);
	free(s_oki_gun);
	s_mix_l = s_mix_r = s_oki_main = s_oki_gun = NULL;
}

static void dg_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int p = 0; p < 2; p++)
		for (int a = 0; a < 2; a++) {
			int v = s_gun[p][a] + rbx_analog(p + 1, a ? RBX_IPT_LIGHTGUN_Y : RBX_IPT_LIGHTGUN_X);
			s_gun[p][a] = v < 0 ? 0 : v > 255 ? 255 : v;
		}
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		deco_irq_line(&s_irq, line);
		if (line >= VIS_Y0 && line < VIS_Y0 + VIS_H)
			for (int i = 0; i < 2; i++)
				deco16ic_latch_line(&s_tile[i], line);
		s_poll.slice_end = s_cpu.cycles + rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		arm_run(&s_cpu, s_poll.slice_end);
		arm_break = 0;
		h6280_run_until(&s_snd, s_snd.cycles + rbx_line_cycles(&s_snd_frac, SND_CLOCK, FRAME_MHZ, LINES));
		h6280_break = 0;
		s_ym_line_clocks = (uint32_t)rbx_line_cycles(&s_ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
		mix_ym_line();
	}
	rbx_ym_async_take_lr(0, s_mix_l, s_mix_r, s_mix_n);
	const int32_t ml = s_vol[0].vol256[0], mr = s_vol[0].vol256[1], gv = s_vol[1].vol256[0];
	for (int i = 0; i < s_mix_n; i++) {   // the main volume on both sides
		const int32_t gun = s_oki_gun[i] * gv / 512;   // (the separate gun speaker: half to each side)
		rbx_audio_put_stereo((s_mix_l[i] + s_oki_main[i]) * ml / 256 + gun, (s_mix_r[i] + s_oki_main[i]) * mr / 256 + gun);
	}
	memset(s_mix_l, 0, sizeof(int32_t) * (size_t)s_mix_n);
	memset(s_mix_r, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	s_frame++;
}

static int dg_state(void *ctx, int load)
{
	uint8_t cpu[ARM_STATE_SIZE];
	uint8_t snd[H6280_STATE_SIZE];
	if (!load) {
		memcpy(cpu, &s_cpu, ARM_STATE_SIZE);
		memcpy(snd, &s_snd, H6280_STATE_SIZE);
	}
	uint64_t fr[3] = { s_cpu_frac, s_snd_frac, s_ym_frac };
	int32_t misc[20] = { (int32_t)s_frame, s_irq_raster2, s_irq_vblank, (int32_t)s_sprite_ctrl, (int32_t)s_lightgun_port,
	                     (int32_t)s_gun_speaker_off, (int32_t)s_rand, s_gun[0][0], s_gun[0][1], s_gun[1][0], s_gun[1][1] };
	for (int k = 0; k < 3; k++) {
		misc[11 + k] = (int32_t)s_oki[k].bank;
		misc[14 + k] = (int32_t)s_oki_phase[k];
		misc[17 + k] = s_oki_last[k];
	}
	int rc = 0;
	rc |= rbx_state_io(ctx, "arm", cpu, ARM_STATE_SIZE);
	rc |= rbx_state_io(ctx, "h6280", snd, H6280_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "ram3", s_ram3, sizeof(s_ram3));
	rc |= rbx_state_io(ctx, "ram4", s_ram4, sizeof(s_ram4));
	rc |= rbx_state_io(ctx, "sram", s_snd_ram, sizeof(s_snd_ram));
	rc |= rbx_state_io(ctx, "palram", s_palram, sizeof(s_palram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "paldirty", s_pal_dirty, sizeof(s_pal_dirty));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "sprb", s_spriteram_buf, sizeof(s_spriteram_buf));
	rc |= rbx_state_io(ctx, "fmt", s_format, sizeof(s_format));
	rc |= rbx_state_io(ctx, "stile", s_spritetile, sizeof(s_spritetile));
	rc |= rbx_state_io(ctx, "clip", s_clip, sizeof(s_clip));
	rc |= rbx_state_io(ctx, "index", s_index, sizeof(s_index));
	rc |= rbx_state_io(ctx, "rows32", s_rowscroll32, sizeof(s_rowscroll32));
	rc |= rbx_state_io(ctx, "rows", s_rowscroll, sizeof(s_rowscroll));
	for (int i = 0; i < 2; i++) {
		rc |= rbx_state_io(ctx, i ? "vram2" : "vram1", s_tile[i].vram, sizeof(s_tile[i].vram));
		rc |= rbx_state_io(ctx, i ? "tctl2" : "tctl1", s_tile[i].control, sizeof(s_tile[i].control));
	}
	rc |= rbx_state_io(ctx, "prot", &s_prot, DECO146_STATE_SIZE);
	rc |= rbx_state_io(ctx, "irqc", &s_irq, DECO_IRQ_STATE_SIZE);
	rc |= rbx_state_io(ctx, "eeprom", &s_eeprom, sizeof(s_eeprom));
	rc |= rbx_state_io(ctx, "lc7535", s_vol, sizeof(s_vol));
	rc |= rbx_state_io(ctx, "oki1", s_oki[0].v, sizeof(s_oki[0].v) + sizeof(int));
	rc |= rbx_state_io(ctx, "oki2", s_oki[1].v, sizeof(s_oki[1].v) + sizeof(int));
	rc |= rbx_state_io(ctx, "oki3", s_oki[2].v, sizeof(s_oki[2].v) + sizeof(int));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (rc)
		return -1;
	if (load) {
		memcpy(&s_cpu, cpu, ARM_STATE_SIZE);
		memcpy(&s_snd, snd, H6280_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_ym_frac = fr[2];
		s_frame = (uint32_t)misc[0];
		s_irq_raster2 = misc[1];
		s_irq_vblank = misc[2];
		s_sprite_ctrl = (uint32_t)misc[3];
		s_lightgun_port = (uint32_t)misc[4];
		s_gun_speaker_off = (uint32_t)misc[5];
		s_rand = (uint32_t)misc[6];
		s_gun[0][0] = misc[7];
		s_gun[0][1] = misc[8];
		s_gun[1][0] = misc[9];
		s_gun[1][1] = misc[10];
		for (int k = 0; k < 3; k++) {
			s_oki[k].bank = (uint32_t)misc[11 + k];
			s_oki_phase[k] = (uint32_t)misc[14 + k];
			s_oki_last[k] = misc[17 + k];
		}
		for (int c = 0; c < 2; c++)
			for (int i = 0; i < 8; i++)
				deco16ic_control_w(&s_tile[c], i, s_tile[c].control[i], 0xffff);
	}
	return 0;
}

static int dg_save(void *ctx) { return dg_state(ctx, 0); }
static int dg_load(void *ctx) { return dg_state(ctx, 1); }
static int dg_supported(const char *machine) { return !strcmp(machine, "dragngun"); }

const rbx_driver rbx_drv_dragngun = { "DECO 32 Dragon Gun", dg_init, dg_exit, dg_reset, dg_frame, dg_save, dg_load,
                                      dg_supported, dg_render, NULL };
