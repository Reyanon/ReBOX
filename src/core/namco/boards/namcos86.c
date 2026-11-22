/*  Namco System 86: two MC6809E, an HD63701 (sound and inputs), the CUS42/CUS43 tilemaps (two pairs), the System 1
    sprites, YM2151 + CUS30, and on Genpei ToumaDen / Rolling Thunder / Wonder Momo the 63701X sample player and the
    banked data ROMs.
    Ported from MAME 0.289 src/mame/namco/namcos86.cpp, namcos86_v.cpp, namco_cus4xtmap.cpp, namcos1_sprite.cpp
    and src/devices/sound/n63701x.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria, Ernesto Corvi, Manuel Abadia,
    David Haywood).
    This file is part of ReBOX. GPL-2.0-or-later.

    Not done: the flip screen (cocktail), the LEDs, the watchdog.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/m6800/m6801.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/namco_wsg.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define VIS_X0 67
#define VIS_W 288
#define VIS_Y0 16
#define VIS_H 224
#define LINES 264
#define FRAME_MHZ 60606         // 6.144 MHz / (384 x 264)
#define CPU_CLOCK 1536000       // 49.152 MHz / 32 (the MCU's E clock: / 8 / 4)
#define SLICES 12               // (MAME: a quantum of the 6809's clock / 4; 12 a line = 8 clocks)
#define YM_CLOCK 3579545

enum { V_HOPMAPPY, V_ROISHTAR, V_GENPEITD, V_RTHUNDER, V_WNDRMOMO };

// ---- the set's configuration ----
static int s_var;
static uint8_t *s_cpu1, *s_cpu2, *s_mcurom, *s_mcusub, *s_banked, *s_samples, *s_tiles[2], *s_sprrom, *s_proms;
static uint32_t s_cpu1_size, s_cpu2_size, s_mcurom_size, s_mcusub_size, s_banked_size, s_samples_size, s_tiles_size[2],
	s_sprrom_size, s_proms_size;
static uint8_t *s_tilepix[2];   // the 8x8 tiles, a byte a pixel
static uint32_t s_tile_count[2];
static uint8_t *s_sprpix;       // the 32x32 sprites, a byte a pixel
static uint32_t s_spr_count;
static uint32_t s_pal[512];     // the indirect colours
static int s_port_in[3], s_port_dsw[2];

static m6809_t s_cpu[2];
static m6801 s_mcu;
static namco_wsg_t s_wsg;

typedef struct {
	uint8_t playing, select, volume, pad;
	uint32_t base_addr, position, silence_counter;
} n63701x_voice;

// ---- the board's state (saved) ----
static struct {
	uint8_t tilebank, backcolor, mainbank, subbank;
	uint8_t copy_sprites, flip, pad[2];
	uint16_t xscroll[2][2];
	uint8_t yscroll[2][2];
	n63701x_voice v63[2];
	uint32_t s63_frac;
	int64_t t[2];
	uint64_t cpu_frac, ym_frac;
	uint8_t vram[2][0x2000];        // the two CUS42/43 pairs
	uint8_t shared[0x1800];         // (0x4000-0x43ff the CUS30's window)
	uint8_t spr[0x800];
	uint8_t mcuram[0xc00];          // 0x1400-0x1fff
} b;

// ---- the drawing's copy ----
typedef struct {
	uint8_t vram[2][0x2000];
	uint8_t spr[0x800];
	uint16_t xscroll[2][2];
	uint8_t yscroll[2][2];
	uint8_t tilebank, backcolor, flip;
} snap_t;
static snap_t *s_snap;
static uint8_t *s_pri;

static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static uint32_t s_ym_rate;

// ---- the 63701X (n63701x.cpp) ----
static const int s_vol63[4] = { 26, 84, 200, 258 };

static void n63_write(int offset, uint8_t data)
{
	const int ch = offset / 2;
	n63701x_voice *v = &b.v63[ch];
	if (offset & 1)
		v->select = data;
	else if ((v->select & 0x1f) && s_samples) {
		v->playing = 1;
		v->base_addr = 0x10000u * (uint32_t)((v->select & 0xe0) >> 5);
		const uint32_t ro = v->base_addr + 2u * (uint32_t)((v->select & 0x1f) - 1);
		v->position = (uint32_t)(s_samples[rbx_wrap(ro, s_samples_size)] << 8) + s_samples[rbx_wrap(ro + 1, s_samples_size)];
		v->volume = data >> 6;
		v->silence_counter = 0;
	}
}

// one sample of the chip's 6 kHz stream (both voices, mono)
static int32_t n63_sample(void)
{
	int32_t out = 0;
	for (int ch = 0; ch < 2; ch++) {
		n63701x_voice *v = &b.v63[ch];
		if (!v->playing)
			continue;
		if (v->silence_counter) {
			v->silence_counter--;
			continue;
		}
		const uint8_t *base = s_samples + v->base_addr;
		(void)base;
		int data = s_samples[rbx_wrap(v->base_addr + (v->position++ & 0xffff), s_samples_size)];
		if (data == 0xff) {
			v->playing = 0;
			continue;
		}
		if (data == 0x00) {
			data = s_samples[rbx_wrap(v->base_addr + (v->position++ & 0xffff), s_samples_size)];
			v->silence_counter = (uint32_t)data;
			continue;
		}
		out += s_vol63[v->volume] * (data - 0x80);
	}
	return out;
}

// ---- the main CPU (cpu1_map) ----
static void main_bank(void)
{
	const uint8_t *src = s_banked ? s_banked + rbx_wrap((uint32_t)b.mainbank * 0x2000, s_banked_size) : s_cpu1 + (uint32_t)(b.mainbank & 3) * 0x2000;
	for (int p = 0; p < 0x20; p++)
		s_cpu[0].rpage[0x60 + p] = src + p * 256;
}

static void sub_bank(void)
{
	for (int p = 0; p < 0x20; p++)
		s_cpu[1].rpage[0x60 + p] = s_cpu2 + (uint32_t)(b.subbank & 3) * 0x2000 + (uint32_t)p * 256;
}

static void scroll_w(int gen, int layer, int off, uint8_t v)
{
	switch (off) {
	case 0: b.xscroll[gen][layer] = (uint16_t)((b.xscroll[gen][layer] & 0xff) | (v << 8)); break;
	case 1: b.xscroll[gen][layer] = (uint16_t)((b.xscroll[gen][layer] & 0xff00) | v); break;
	default: b.yscroll[gen][layer] = v; break;
	}
}

static void spr_w(uint32_t o, uint8_t v)
{
	b.spr[o] = v;
	if (o == 0x7f2)
		b.copy_sprites = 1;
	if (o == 0x7f6 && (v & 1) != b.flip)
		b.flip = v != 0;
}

static uint8_t cpu1_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a >= 0x4000 && a < 0x4400) return namco_cus30_amap_r(&s_wsg, a & 0x3ff);
	return 0;
}

static void cpu1_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x4000) {
		b.vram[a >> 13][a & 0x1fff] = v;
	} else if (a < 0x4400)
		namco_cus30_amap_w(&s_wsg, a & 0x3ff, v);
	else if (a < 0x5800)
		b.shared[a - 0x4000] = v;
	else if (a < 0x6000)
		spr_w(a - 0x5800, v);
	else if (a < 0x8000) {   // cus115_w
		if (!s_banked)
			return;
		const int r = (a & 0x1e00) >> 9;
		if (r < 4)
			n63_write(r, v);
		else if (r == 4) {
			b.mainbank = v & 0x1f;
			main_bank();
		}
	} else if (a == 0x8400)
		m6809_set_irq(&s_cpu[0], 0);
	else if (a >= 0x8800 && a < 0x9000)
		b.tilebank = (uint8_t)(((a - 0x8800) >> 10) & 1);
	else if (a >= 0x9000 && a <= 0x9002)
		scroll_w(0, 0, a - 0x9000, v);
	else if (a == 0x9003) {
		if (!s_banked) {
			b.mainbank = v & 3;
			main_bank();
		}
	} else if (a >= 0x9004 && a <= 0x9006)
		scroll_w(0, 1, a - 0x9004, v);
	else if (a >= 0x9400 && a <= 0x9402)
		scroll_w(1, 0, a - 0x9400, v);
	else if (a >= 0x9404 && a <= 0x9406)
		scroll_w(1, 1, a - 0x9404, v);
	else if (a == 0xa000)
		b.backcolor = v;
}

// ---- the sub CPU (the five maps) ----
static uint8_t cpu2_r(void *ctx, uint16_t a)
{
	(void)ctx;
	(void)a;
	return 0;
}

static void cpu2_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	switch (s_var) {
	case V_HOPMAPPY:
		if (a == 0x9400) m6809_set_irq(&s_cpu[1], 0);
		break;
	case V_ROISHTAR:
		if (a >= 0x1800 && a < 0x2000) spr_w(a - 0x1800, v);
		else if (a == 0xb000) m6809_set_irq(&s_cpu[1], 0);
		break;
	case V_GENPEITD:
		if (a >= 0x5800 && a < 0x6000) spr_w(a - 0x5800, v);
		else if (a == 0x8800) m6809_set_irq(&s_cpu[1], 0);
		break;
	case V_RTHUNDER:
		if (a >= 0x1800 && a < 0x2000) spr_w(a - 0x1800, v);
		else if (a == 0x8800) m6809_set_irq(&s_cpu[1], 0);
		else if (a == 0xd803) {
			b.subbank = v & 3;
			sub_bank();
		}
		break;
	case V_WNDRMOMO:
		if (a >= 0x3800 && a < 0x4000) spr_w(a - 0x3800, v);
		else if (a == 0xc800) m6809_set_irq(&s_cpu[1], 0);
		break;
	}
}

static void map_pages(m6809_t *c, int first, int n, uint8_t *mem, int writable)
{
	for (int p = 0; p < n; p++) {
		c->rpage[first + p] = mem + p * 256;
		c->wpage[first + p] = writable ? mem + p * 256 : NULL;
	}
}

static void sub_map(void)
{
	m6809_t *c = &s_cpu[1];
	for (int p = 0; p < 0x80; p++)
		c->rpage[p] = c->wpage[p] = NULL;
	for (int p = 0x80; p < 0x100; p++) {
		c->rpage[p] = s_cpu2 + (p - 0x80) * 256 + 0x0000 + (0x8000);
		c->wpage[p] = NULL;
	}
	switch (s_var) {
	case V_ROISHTAR:
		map_pages(c, 0x00, 0x18, b.shared, 1);
		map_pages(c, 0x18, 0x08, b.spr, 0);
		map_pages(c, 0x40, 0x20, b.vram[1], 1);
		map_pages(c, 0x60, 0x20, b.vram[0], 1);
		break;
	case V_GENPEITD:
		map_pages(c, 0x00, 0x20, b.vram[0], 1);
		map_pages(c, 0x20, 0x20, b.vram[1], 1);
		map_pages(c, 0x40, 0x18, b.shared, 1);
		map_pages(c, 0x58, 0x08, b.spr, 0);
		break;
	case V_RTHUNDER:
		map_pages(c, 0x00, 0x18, b.shared, 1);
		map_pages(c, 0x18, 0x08, b.spr, 0);
		map_pages(c, 0x20, 0x20, b.vram[0], 1);
		map_pages(c, 0x40, 0x20, b.vram[1], 1);
		sub_bank();
		break;
	case V_WNDRMOMO:
		map_pages(c, 0x20, 0x18, b.shared, 1);
		map_pages(c, 0x38, 0x08, b.spr, 0);
		map_pages(c, 0x40, 0x20, b.vram[0], 1);
		map_pages(c, 0x60, 0x20, b.vram[1], 1);
		break;
	default:
		break;
	}
}

// ---- the MCU (the five maps) ----
static uint16_t s_ym_at, s_in_at, s_rom_lo, s_rom_hi;   // (the variant's YM / inputs addresses, the ROM window)
static uint32_t s_rom_off;

static uint8_t dsw_bits(int sel)
{
	const uint8_t d0 = (uint8_t)rbx_port_read(s_port_dsw[0]), d1 = (uint8_t)rbx_port_read(s_port_dsw[1]);
	if (!sel)   // dsw0_r
		return (uint8_t)(((d0 & 0x01) << 4) | ((d0 & 0x04) << 3) | ((d0 & 0x10) << 2) | ((d0 & 0x40) << 1) |
		                 (d1 & 0x01) | ((d1 & 0x04) >> 1) | ((d1 & 0x10) >> 2) | ((d1 & 0x40) >> 3));
	return (uint8_t)(((d0 & 0x02) << 3) | ((d0 & 0x08) << 2) | ((d0 & 0x20) << 1) | (d0 & 0x80) |
	                 ((d1 & 0x02) >> 1) | ((d1 & 0x08) >> 2) | ((d1 & 0x20) >> 3) | ((d1 & 0x80) >> 4));
}

static uint8_t mcu_r(void *u, uint16_t a)
{
	(void)u;
	if (a >= s_rom_lo && a < s_rom_hi) return s_mcusub[rbx_wrap(a - s_rom_lo + s_rom_off, s_mcusub_size)];
	if (a >= 0x1000 && a < 0x1400) return namco_cus30_amap_r(&s_wsg, a & 0x3ff);
	if (a >= 0x1400 && a < 0x2000) return b.mcuram[a - 0x1400];
	if (a == s_ym_at || a == s_ym_at + 1) return rbx_ym_read(0, a & 1);
	if (a == s_in_at) return (uint8_t)rbx_port_read(s_port_in[0]);
	if (a == s_in_at + 1) return (uint8_t)rbx_port_read(s_port_in[1]);
	if (a == s_in_at + 0x10) return dsw_bits(0);
	if (a == s_in_at + 0x11) return dsw_bits(1);
	if (a >= 0xf000) return s_mcurom[a & 0xfff];
	if (s_var == V_ROISHTAR && a >= 0x2000 && a < 0x4000) return s_mcusub[rbx_wrap(a, s_mcusub_size)];
	return 0;
}

static void mcu_w(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x1000 && a < 0x1400) namco_cus30_amap_w(&s_wsg, a & 0x3ff, v);
	else if (a >= 0x1400 && a < 0x2000) b.mcuram[a - 0x1400] = v;
	else if (a == s_ym_at || a == s_ym_at + 1) rbx_ym_write(0, a & 1, v);
}

static uint8_t mcu_port_r(void *u, int port)
{
	(void)u;
	return port == 1 ? (uint8_t)rbx_port_read(s_port_in[2]) : 0xff;
}

// ---- drawing ----
static void draw_tilemap(const snap_t *sn, int gen, int layer, int vy, uint16_t *d, uint8_t *pri, int p)
{
	static const int xoffs[2] = { 47, 46 }, adj[2] = { 0, 2 };
	const int dx = xoffs[gen] + adj[layer];
	// (flipped: the scroll negated, the tilemap flipped against MAME's extents 422 x 256 and offsets 422 - dx, 9)
	const int flip = sn->flip;
	const int ty = flip ? (230 - vy - sn->yscroll[gen][layer]) & 0xff : (vy + VIS_Y0 + sn->yscroll[gen][layer] + 9) & 0xff;
	const int sx = flip ? dx - 1 - VIS_X0 - sn->xscroll[gen][layer] : VIS_X0 + sn->xscroll[gen][layer] - dx;
	const uint8_t *vram = sn->vram[gen] + (layer << 12);
	const int row = (ty >> 3) * 64, fy = ty & 7;
	const uint8_t *prom = s_proms + 0x1400;   // (m_tile_address_prom: after the palette and the two lookup tables)
	for (int x = 0; x < VIS_W;) {
		const int tx = flip ? (sx - x) & 0x1ff : (x + sx) & 0x1ff;
		const int idx = row + (tx >> 3);
		const uint8_t attr = vram[2 * idx + 1];
		uint32_t code = vram[2 * idx] + ((uint32_t)(attr & 3) << 8);
		if (gen == 0)   // tile_cb_0
			code = (code & 0xff) | ((uint32_t)((prom[((layer & 1) << 4) + (((code >> 8) & 3) << 2)] & 0x0e) >> 1) << 8) |
			       ((uint32_t)sn->tilebank << 11);
		else            // tile_cb_1
			code = (code & 0xff) | ((uint32_t)((prom[((layer & 1) << 4) + ((code >> 8) & 3)] & 0xe0) >> 5) << 8);
		const uint8_t *src = s_tilepix[gen] + rbx_wrap(code, s_tile_count[gen]) * 64 + (uint32_t)fy * 8;
		const uint16_t cbase = (uint16_t)(attr * 8);
		if (flip) {   // (this tile's pixels from fx down to 0)
			for (int fx = tx & 7; fx >= 0 && x < VIS_W; fx--, x++) {
				const uint8_t pen = src[fx];
				if (pen != 7) {
					d[x] = (uint16_t)(cbase + pen);
					pri[x] = (uint8_t)p;
				}
			}
			continue;
		}
		for (int fx = tx & 7; fx < 8 && x < VIS_W; fx++, x++) {
			const uint8_t pen = src[fx];
			if (pen != 7) {
				d[x] = (uint16_t)(cbase + pen);
				pri[x] = (uint8_t)p;
			}
		}
	}
}

static void draw_sprites(const snap_t *sn)
{
	static const int ssz[4] = { 16, 8, 32, 4 };
	const uint8_t *spr = sn->spr;
	const int xoffs = spr[0x7f5] + ((spr[0x7f4] & 1) << 8), yoffs = spr[0x7f7];
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const uint32_t bank_sprites = s_spr_count / 8 ? s_spr_count / 8 : 1;
	for (int o = 0x7e0; o >= 0; o -= 0x10) {
		const uint8_t *s = spr + o;
		const uint8_t a1 = s[10], a2 = s[14];
		int color = s[12];
		int fx = (a1 >> 5) & 1, fy = a2 & 1;
		const int w = ssz[a1 >> 6], h = ssz[(a2 >> 1) & 3];
		const int tx = (a1 & 0x18) & ~(w - 1), ty = (a2 & 0x18) & ~(h - 1);
		int sx = s[13] + ((color & 1) << 8), sy = -s[15] - h;
		const uint32_t code = rbx_wrap(((uint32_t)s[11] & (bank_sprites - 1)) + (uint32_t)(a1 & 7) * bank_sprites, s_spr_count);
		const uint32_t pmask = ((0xffu << (((a2 & 0xe0) >> 5) + 1)) & 0xff) | 0x80000000u;
		color >>= 1;
		sx += xoffs;
		sy -= yoffs;
		if (sn->flip) {
			sx = -sx - w;
			sy = -sy - h;
			fx = !fx;
			fy = !fy;
		}
		sy++;
		const int x0 = sx & 0x1ff, y0 = ((sy + 16) & 0xff) - 16;
		const uint8_t *tile = s_sprpix + code * 1024;
		for (int yy = 0; yy < h; yy++) {
			const int ay = y0 + yy;
			if (ay < VIS_Y0 || ay >= VIS_Y0 + VIS_H)
				continue;
			const uint8_t *src = tile + (uint32_t)(ty + (fy ? h - 1 - yy : yy)) * 32 + (uint32_t)tx;
			uint16_t *d = bm + (ay - VIS_Y0) * pitch - VIS_X0;
			uint8_t *pri = s_pri + (ay - VIS_Y0) * VIS_W - VIS_X0;
			for (int xx = 0; xx < w; xx++) {
				const int ax = x0 + xx;
				if (ax < VIS_X0 || ax >= VIS_X0 + VIS_W)
					continue;
				const uint8_t pen = src[fx ? w - 1 - xx : xx];
				if (pen == 15)
					continue;
				if (!((1u << pri[ax]) & pmask))
					d[ax] = (uint16_t)(2048 + color * 16 + pen);
				pri[ax] = 31;
			}
		}
	}
}

static void ns86_render(void)
{
	const snap_t *sn = s_snap;
	uint32_t *pal = rbx_video.palette;
	for (int i = 0; i < 2048; i++)   // (the pens through the lookup PROMs)
		pal[i] = s_pal[s_proms[0x400 + i]];
	for (int i = 0; i < 2048; i++)
		pal[2048 + i] = s_pal[256 + s_proms[0xc00 + i]];
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const uint16_t bg = (uint16_t)(8 * sn->backcolor + 7);
	memset(s_pri, 0, VIS_W * VIS_H);
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = bm + y * pitch;
		uint8_t *pri = s_pri + y * VIS_W;
		for (int x = 0; x < VIS_W; x++)
			d[x] = bg;
		for (int p = 0; p < 8; p++)
			for (int i = 3; i >= 0; i--)
				if (((sn->xscroll[i >> 1][i & 1] & 0x0e00) >> 9) == p)
					draw_tilemap(sn, i >> 1, i & 1, y, d, pri, p);
	}
	draw_sprites(sn);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->vram, b.vram, sizeof(b.vram));
	memcpy(s_snap->spr, b.spr, sizeof(b.spr));
	memcpy(s_snap->xscroll, b.xscroll, sizeof(b.xscroll));
	memcpy(s_snap->yscroll, b.yscroll, sizeof(b.yscroll));
	s_snap->tilebank = b.tilebank;
	s_snap->backcolor = b.backcolor;
	s_snap->flip = b.flip;
	rbx_video_frame_done();
}

// ---- frame ----
static void ns86_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	s_mix_n = 0;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0 + VIS_H) {   // vblank: the frame drawn, the sprites copied, the IRQs
			vblank_capture();
			if (b.copy_sprites) {
				for (int i = 0; i < 0x800; i += 16)
					for (int j = 10; j < 16; j++)
						b.spr[i + j] = b.spr[i + j - 6];
				b.copy_sprites = 0;
			}
			m6809_set_irq(&s_cpu[0], 1);
			m6809_set_irq(&s_cpu[1], 1);
			m6801_set_irq_line(&s_mcu, M6801_IRQ1_LINE, M6801_HOLD_LINE);
		}
		for (int sl = 0; sl < SLICES; sl++) {
			const int n = rbx_line_cycles(&b.cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES * SLICES);
			b.t[0] += n;
			b.t[1] += n;
			m6809_run_until(&s_cpu[0], b.t[0]);
			m6809_run_until(&s_cpu[1], b.t[1]);
			m6801_execute(&s_mcu, n);
		}
		// the sound of this line: YM2151 (its right output only), CUS30, the 63701X at 6 kHz
		const uint32_t yc = (uint32_t)rbx_line_cycles(&b.ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
		const int k = rbx_ym_advance(0, yc, s_mix + s_mix_n, s_mix_cap - s_mix_n);
		if (k > 0) {
			namco_wsg_render(&s_wsg, s_mix + s_mix_n, NULL, k, s_ym_rate);
			if (s_samples)
				for (int j = 0; j < k; j++) {
					b.s63_frac += 6000;
					static int32_t cur;
					while (b.s63_frac >= s_ym_rate) {
						b.s63_frac -= s_ym_rate;
						cur = n63_sample();
					}
					s_mix[s_mix_n + j] += cur;
				}
			s_mix_n += k;
		}
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++) {
		rbx_audio_put(s_mix[i]);
		s_mix[i] = 0;
	}
}

// ---- init / reset ----
static void ns86_reset(void)
{
	b.tilebank = b.backcolor = 0;
	b.mainbank = b.subbank = 0;
	b.copy_sprites = b.flip = 0;
	memset(b.xscroll, 0, sizeof(b.xscroll));
	memset(b.yscroll, 0, sizeof(b.yscroll));
	memset(b.v63, 0, sizeof(b.v63));
	b.s63_frac = 0;
	b.t[0] = b.t[1] = 0;
	b.cpu_frac = b.ym_frac = 0;
	main_bank();
	sub_map();
	for (int i = 0; i < 2; i++) {
		m6809_reset(&s_cpu[i]);
		s_cpu[i].cycles = 0;
	}
	m6801_reset(&s_mcu);
	rbx_ym_reset(0);
	namco_wsg_init(&s_wsg, NAMCO_WSG_TYPE_CUS30, 49152000 / 2048, 0, NULL, 128);
}

static void ns86_exit(void);

// the 3bpp planar tiles after init_namco86's shuffle: planes at thirds, MSB first
static uint8_t *decode_tiles(uint8_t *gfx, uint32_t size, uint32_t *count)
{
	const uint32_t half = size * 2 / 3;
	{
		uint8_t *buf = (uint8_t *)malloc(half);
		if (!buf)
			return NULL;
		memcpy(buf, gfx, half);
		uint8_t *d1 = gfx, *d2 = gfx + half / 2, *mono = gfx + half;
		for (uint32_t i = 0; i + 1 < half; i += 2) {
			const uint8_t a = buf[i], c = buf[i + 1];
			*d1++ = (uint8_t)((a << 4) | (c & 0xf));
			*d2++ = (uint8_t)((a & 0xf0) | (c >> 4));
			*mono ^= 0xff;
			mono++;
		}
		free(buf);
	}
	const uint32_t third = size / 3;
	*count = third / 8;
	uint8_t *pix = (uint8_t *)malloc((size_t)*count * 64 + 1);
	if (!pix)
		return NULL;
	for (uint32_t t = 0; t < *count; t++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				uint8_t pen = 0;
				for (int pl = 0; pl < 3; pl++)
					if (gfx[pl * third + t * 8 + (uint32_t)y] & (0x80 >> x))
						pen |= (uint8_t)(1 << pl);   // (gfx_8x8x3_planar: the planes at 2/3, 1/3, 0/3, MSB first)
				pix[t * 64 + (uint32_t)(y * 8 + x)] = pen;
			}
	return pix;
}

static int ns86_init(const rbx_game *g)
{
	s_cpu1 = rbx_region("cpu1", &s_cpu1_size);
	s_cpu2 = rbx_region("cpu2", &s_cpu2_size);
	s_mcurom = rbx_region("mcu", &s_mcurom_size);
	s_mcusub = rbx_region("mcusub", &s_mcusub_size);
	s_banked = rbx_region("bankeddata", &s_banked_size);
	s_samples = rbx_region("namco2", &s_samples_size);
	s_tiles[0] = rbx_region("tiles_0", &s_tiles_size[0]);
	s_tiles[1] = rbx_region("tiles_1", &s_tiles_size[1]);
	s_sprrom = rbx_region("sprites", &s_sprrom_size);
	s_proms = rbx_region("proms", &s_proms_size);
	if (!s_cpu1 || s_cpu1_size < 0x10000 || !s_cpu2 || s_cpu2_size < 0x10000 || !s_mcurom || s_mcurom_size < 0x1000 || !s_mcusub ||
	    !s_tiles[0] || !s_tiles[1] || !s_sprrom || !s_proms || s_proms_size < 0x1420) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_var = !strncmp(g->machine, "roishtar:", 9) ? V_ROISHTAR : !strncmp(g->machine, "genpeitd:", 9) ? V_GENPEITD
	      : !strncmp(g->machine, "rthunder:", 9) ? V_RTHUNDER : !strncmp(g->machine, "wndrmomo:", 9) ? V_WNDRMOMO : V_HOPMAPPY;
	switch (s_var) {   // the MCU's YM2151 / inputs and its external ROM window
	case V_HOPMAPPY: s_ym_at = 0x2000; s_in_at = 0x2020; s_rom_lo = 0x8000; s_rom_hi = 0xc000; s_rom_off = 0; break;
	case V_ROISHTAR: s_ym_at = 0x6000; s_in_at = 0x6020; s_rom_lo = 0x8000; s_rom_hi = 0xc000; s_rom_off = 0x4000; break;
	case V_GENPEITD: s_ym_at = 0x2800; s_in_at = 0x2820; s_rom_lo = 0x4000; s_rom_hi = 0xc000; s_rom_off = 0; break;
	case V_RTHUNDER: s_ym_at = 0x2000; s_in_at = 0x2020; s_rom_lo = 0x4000; s_rom_hi = 0xc000; s_rom_off = 0; break;
	default:         s_ym_at = 0x3800; s_in_at = 0x3820; s_rom_lo = 0x4000; s_rom_hi = 0xc000; s_rom_off = 0; break;
	}
	// the palette: 512 colours from the PROMs (2.2k / 1k / 470 / 220 ohm, scaled to 255)
	{
		const double g4[4] = { 1.0 / 2200, 1.0 / 1000, 1.0 / 470, 1.0 / 220 };
		const double sum = g4[0] + g4[1] + g4[2] + g4[3];
		for (int i = 0; i < 512; i++) {
			const uint8_t c0 = s_proms[i], c1 = s_proms[i | 0x200];
			int rgb[3];
			const int bits[3] = { c0 & 0xf, c0 >> 4, c1 & 0xf };
			for (int k = 0; k < 3; k++) {
				double v = 0;
				for (int j = 0; j < 4; j++)
					if (bits[k] & (1 << j))
						v += 255.0 * g4[j] / sum;
				rgb[k] = (int)(v + 0.5);
				if (rgb[k] > 255) rgb[k] = 255;
			}
			s_pal[i] = (uint32_t)(rgb[0] << 16 | rgb[1] << 8 | rgb[2]);
		}
	}
	s_tilepix[0] = decode_tiles(s_tiles[0], s_tiles_size[0], &s_tile_count[0]);
	s_tilepix[1] = decode_tiles(s_tiles[1], s_tiles_size[1], &s_tile_count[1]);
	s_spr_count = s_sprrom_size / 512;
	s_sprpix = (uint8_t *)malloc((size_t)s_spr_count * 1024 + 1);
	s_snap = (snap_t *)calloc(1, sizeof(snap_t));
	s_pri = (uint8_t *)calloc(1, VIS_W * VIS_H);
	if (!s_tilepix[0] || !s_tilepix[1] || !s_sprpix || !s_snap || !s_pri) {
		ns86_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	for (uint32_t t = 0; t < s_spr_count; t++)   // (the System 1 sprite layout, 4bpp)
		for (int y = 0; y < 32; y++)
			for (int x = 0; x < 32; x++) {
				const uint8_t v = s_sprrom[t * 512 + (uint32_t)((y >> 4) * 256 + (y & 15) * 8 + (x >> 4) * 128 + (x & 15) / 2)];
				s_sprpix[t * 1024 + (uint32_t)(y * 32 + x)] = (x & 1) ? (v & 15) : (v >> 4);
			}
	s_port_in[0] = rbx_port_find("IN0");
	s_port_in[1] = rbx_port_find("IN1");
	s_port_in[2] = rbx_port_find("IN2");
	s_port_dsw[0] = rbx_port_find("DSWA");
	s_port_dsw[1] = rbx_port_find("DSWB");

	memset(&b, 0, sizeof(b));
	m6809_init(&s_cpu[0]);
	s_cpu[0].read = cpu1_r;
	s_cpu[0].write = cpu1_w;
	map_pages(&s_cpu[0], 0x00, 0x20, b.vram[0], 0);
	map_pages(&s_cpu[0], 0x20, 0x20, b.vram[1], 0);
	map_pages(&s_cpu[0], 0x44, 0x14, b.shared + 0x400, 1);
	map_pages(&s_cpu[0], 0x58, 0x08, b.spr, 0);
	for (int p = 0x80; p < 0x100; p++)
		s_cpu[0].rpage[p] = s_cpu1 + (p - 0x80) * 256 + 0x8000;
	for (int p = 0; p < 0x40; p++)   // (the tilemaps' RAM: written through cpu1_w, read directly)
		s_cpu[0].wpage[p] = NULL;
	m6809_init(&s_cpu[1]);
	s_cpu[1].read = cpu2_r;
	s_cpu[1].write = cpu2_w;
	s_cpu[1].ctx = &s_cpu[1];
	memset(&s_mcu, 0, sizeof(s_mcu));
	s_mcu.read = mcu_r;
	s_mcu.write = mcu_w;
	s_mcu.port_r = mcu_port_r;
	s_mcu.rom = s_mcurom;
	s_mcu.rom_base = 0xf000;
	s_mcu.rom_size = 0x1000;
	s_mcu.type = M6801_TYPE_HD63701V0;
	m6801_init(&s_mcu);

	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = 0;     // (only its right output is connected, at 0.60)
	yc.gain[1] = 154;
	int rc = rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	s_ym_rate = rc ? 0 : rbx_ym_rate(0);
	s_mix_cap = (int)(s_ym_rate / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	rc = rc || !s_mix || rbx_audio_setup(s_ym_rate) || rbx_video_setup(VIS_W, VIS_H, 4096);
	if (rc) {
		ns86_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	ns86_reset();
	return 0;
}

static void ns86_exit(void)
{
	rbx_ym_exit(0);
	free(s_tilepix[0]);
	free(s_tilepix[1]);
	free(s_sprpix);
	free(s_snap);
	free(s_pri);
	free(s_mix);
	s_tilepix[0] = s_tilepix[1] = NULL;
	s_sprpix = NULL;
	s_snap = NULL;
	s_pri = NULL;
	s_mix = NULL;
}

static int ns86_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "board", &b, sizeof(b));
	rc |= rbx_state_io(ctx, "cpu1", &s_cpu[0], M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "cpu2", &s_cpu[1], M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "mcu", m6801_state_data(&s_mcu), m6801_state_size());
	rc |= rbx_state_io(ctx, "wsg", &s_wsg, (uint32_t)NAMCO_WSG_STATE_SIZE);
	rc |= rbx_ym_state(0, ctx, load);
	if (load && !rc) {
		main_bank();
		sub_map();
	}
	return rc;
}

static int ns86_save(void *ctx) { return ns86_state(ctx, 0); }
static int ns86_load(void *ctx) { return ns86_state(ctx, 1); }
static int ns86_supported(const char *machine)
{
	return !strncmp(machine, "hopmappy:", 9) || !strncmp(machine, "roishtar:", 9) || !strncmp(machine, "genpeitd:", 9) ||
	       !strncmp(machine, "rthunder:", 9) || !strncmp(machine, "wndrmomo:", 9);
}

const rbx_driver rbx_drv_namcos86 = { "Namco System 86", ns86_init, ns86_exit, ns86_reset, ns86_frame, ns86_save, ns86_load,
                                      ns86_supported, ns86_render, NULL };
