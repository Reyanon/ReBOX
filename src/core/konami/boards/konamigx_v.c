/*  Konami System GX video, ported from MAME src/mame/konami/konamigx_v.cpp (BSD-3-Clause, copyright-holders:
    David Haywood, R. Belmont, Acho A. Tang, Phil Stroffolino, Olivier Galibert), with the drawing of
    k054156_k054157_k056832.cpp (tilemap_draw_common: David Haywood), k053246_k053247_k055673.cpp
    (k053247_draw_single_sprite_gxcore, zdrawgfxzoom32GP), k054338.cpp and k055555.cpp, and MAME's palette shadows
    and contrast (src/emu/dipalette.cpp) and tilemap drawing (src/emu/tilemap.cpp: transparency, categories, alpha).
    Type 2 boards (the Parodius games first, docs/KONAMI.md): the K056832 layers A-D and the K053247 sprites mixed by
    the K055555 priorities with the K054338 blending, shadows and brightness, into an RGB frame.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "konamigx.h"
#include "../../rbx.h"

#include <stdlib.h>
#include <string.h>

// the MAME screen: the bitmap's visible area (set_visarea(24, 24+288-1, 16, 16+224-1))
static int s_vis_y0 = 16;   // (the visible area's first line: the CCU's y offset)
#define VIS_Y0 s_vis_y0
#define VIS_H_MAX 256
static int s_vis_h = 224;   // (the visible lines: 224, xexex 256)
#define VIS_H s_vis_h
#define VIS_W_MAX 416   // (rungun 416)
// (the machine's visible area: x from the CCU's offset, MAME's visarea after the CCU set it; dragoonj 384 from 40)
static int s_vis_x0 = 24, s_vis_w = 288;
#define VIS_X0 s_vis_x0
#define VIS_W s_vis_w
#define GX_ZBUFW 512

// K055555 registers (k055555.h)
enum { K55_PALBASE_BG = 0, K55_CONTROL = 1, K55_PRIINP_0 = 7, K55_PRIINP_3 = 10, K55_PRIINP_6 = 13, K55_PRIINP_7 = 14,
       K55_PRIINP_8 = 15, K55_PRIINP_9 = 16, K55_PRIINP_10 = 17, K55_OINPRI_ON = 19, K55_PALBASE_A = 23,
       K55_PALBASE_OBJ = 27, K55_PALBASE_SUB1 = 28, K55_PALBASE_SUB2 = 29, K55_BLEND_ENABLES = 33, K55_VINMIX_ON = 34,
       K55_OSBLEND_ENABLES = 35, K55_OSBLEND_ON = 36, K55_SHAD1_PRI = 37, K55_SHAD2_PRI = 38, K55_SHAD3_PRI = 39,
       K55_SHD_ON = 40, K55_SHD_PRI_SEL = 41, K55_VBRI = 42, K55_INPUT_ENABLES = 45 };
#define K55_INP_OBJ 0x10
// K054338 registers (k054338.h)
enum { K338_REG_BGC_R = 0, K338_REG_BGC_GB = 1, K338_REG_SHAD1R = 2, K338_REG_BRI3 = 11, K338_REG_PBLEND = 13,
       K338_REG_CONTROL = 15 };
#define K338_CTL_KILL 0x01
#define K338_CTL_MIXPRI 0x02
#define K338_CTL_SHDPRI 0x04
#define K338_CTL_CLIPSL 0x20
// the sprite callback's color flags (konamigx.h)
#define K055555_COLORMASK 0x0000ffff
#define K055555_MIXSHIFT 16
#define K055555_SKIPSHADOW 0x40000000
#define K055555_FULLSHADOW 0x80000000
#define GXMIX_BLEND_FORCE 3
#define GXMIX_NOSHADOW 0x10000000
#define GXMIX_NOZBUF 0x20000000

// ---- the decoded graphics (konami_decode_gfx: 8 bits a pixel) ----
static uint8_t *s_tiles, *s_sprites, *s_tiles_raw;
static uint32_t s_ntiles, s_nsprites;
static int s_tile_gran, s_spr_gran;   // (pens a color: the tiles 16, MAME set_granularity(16); the sprites 1 << bpp)

// MAME gfx layout decode: plane[0] the most significant bit, bits counted from the MSB of each byte
static void decode(uint8_t *dst, const uint8_t *src, uint32_t srclen, uint32_t n, int w, int h, int planes,
                   const int *planeoffs, const int *xoffs, const int *yoffs, int charincrement)
{
	for (uint32_t c = 0; c < n; c++) {
		const uint64_t base = (uint64_t)c * (uint32_t)charincrement;
		for (int y = 0; y < h; y++)
			for (int x = 0; x < w; x++) {
				int v = 0;
				for (int p = 0; p < planes; p++) {
					const uint64_t bit = base + (uint64_t)planeoffs[p] + (uint64_t)xoffs[x] + (uint64_t)yoffs[y];
					const uint64_t byte = bit >> 3;
					const int b = byte < srclen ? (src[byte] >> (7 - (int)(bit & 7))) & 1 : 0;
					v = v << 1 | b;
				}
				dst[(size_t)c * (size_t)(w * h) + (size_t)(y * w + x)] = (uint8_t)v;
			}
	}
}

// the two 5bpp layouts byte by byte (the same pixels as decode(): checked with RBX_GX_DECODECHECK)
static void decode_tiles5(uint8_t *dst, const uint8_t *src, uint32_t n)   // charlayout5: planes 32 24 8 16 0
{
	for (uint32_t c = 0; c < n; c++, src += 40)
		for (int y = 0; y < 8; y++) {
			const uint8_t *r = src + y * 5;
			for (int x = 0; x < 8; x++) {
				const int b = 7 - x;
				*dst++ = (uint8_t)(((r[4] >> b) & 1) << 4 | ((r[3] >> b) & 1) << 3 | ((r[1] >> b) & 1) << 2 |
				                   ((r[2] >> b) & 1) << 1 | ((r[0] >> b) & 1));
			}
		}
}
static void decode_sprites5(uint8_t *dst, const uint8_t *src, uint32_t n)   // GX spritelayout: planes 32 24 16 8 0
{
	for (uint32_t c = 0; c < n; c++, src += 160)
		for (int y = 0; y < 16; y++) {
			const uint8_t *row = src + y * 10;
			for (int x = 0; x < 16; x++) {
				const uint8_t *r = row + (x >> 3) * 5;
				const int b = 7 - (x & 7);
				*dst++ = (uint8_t)(((r[4] >> b) & 1) << 4 | ((r[3] >> b) & 1) << 3 | ((r[2] >> b) & 1) << 2 |
				                   ((r[1] >> b) & 1) << 1 | ((r[0] >> b) & 1));
			}
		}
}

static int decode_tiles(const k056832_t *k)
{
	static const int x8[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
	int planes, inc, po[8], yo[8];
	const int *xo = x8;
	static const int x4[8] = { 2 * 4, 3 * 4, 0 * 4, 1 * 4, 6 * 4, 7 * 4, 4 * 4, 5 * 4 };
	switch (k->bpp) {
	case K056832_BPP_4: {   // charlayout4
		static const int p[4] = { 0, 1, 2, 3 };
		planes = 4; memcpy(po, p, sizeof(p)); xo = x4;
		for (int i = 0; i < 8; i++) yo[i] = i * 8 * 4;
		inc = 8 * 8 * 4;
		break;
	}
	case K056832_BPP_5: {   // charlayout5
		static const int p[5] = { 32, 24, 8, 16, 0 };
		planes = 5; memcpy(po, p, sizeof(p));
		for (int i = 0; i < 8; i++) yo[i] = i * 5 * 8;
		inc = 8 * 8 * 5;
		break;
	}
	case K056832_BPP_6: {   // charlayout6
		static const int p[6] = { 40, 32, 24, 8, 16, 0 };
		planes = 6; memcpy(po, p, sizeof(p));
		for (int i = 0; i < 8; i++) yo[i] = i * 6 * 8;
		inc = 8 * 8 * 6;
		break;
	}
	default: {   // charlayout8
		static const int p[8] = { 8 * 7, 8 * 3, 8 * 5, 8 * 1, 8 * 6, 8 * 2, 8 * 4, 8 * 0 };
		planes = 8; memcpy(po, p, sizeof(p));
		for (int i = 0; i < 8; i++) yo[i] = i * 8 * 8;
		inc = 8 * 8 * 8;
		break;
	}
	}
	// (total = ROM bytes / (8 * planes)... MAME: m_rombase.bytes() / (i * bpp) with i = 8)
	s_ntiles = (uint32_t)(k->rom_size / (uint32_t)(8 * planes));
	s_tiles_raw = (uint8_t *)malloc((size_t)s_ntiles * 64 + 63);   // (64-byte aligned: a tile, two cache lines)
	s_tiles = s_tiles_raw ? (uint8_t *)(((uintptr_t)s_tiles_raw + 63) & ~(uintptr_t)63) : NULL;
	if (!s_tiles)
		return -1;
	if (k->bpp == K056832_BPP_5 && gx.mw) {
		// (mystwarr_state::decode_tiles: the ROM's 4 chunky bits and the planar 5th to the GX's 5bpp planar format)
		uint8_t *c = (uint8_t *)malloc(k->rom_size);
		if (!c)
			return -1;
		const uint8_t *s = k->rom;
		uint32_t i = 0;
		for (; i + 5 <= k->rom_size; i += 5, s += 5) {
			const int d0 = ((s[0] & 0x80)) | ((s[0] & 0x08) << 3) | ((s[1] & 0x80) >> 2) | ((s[1] & 0x08) << 1) |
			               ((s[2] & 0x80) >> 4) | ((s[2] & 0x08) >> 1) | ((s[3] & 0x80) >> 6) | ((s[3] & 0x08) >> 3);
			const int d1 = ((s[0] & 0x40) << 1) | ((s[0] & 0x04) << 4) | ((s[1] & 0x40) >> 1) | ((s[1] & 0x04) << 2) |
			               ((s[2] & 0x40) >> 3) | ((s[2] & 0x04)) | ((s[3] & 0x40) >> 5) | ((s[3] & 0x04) >> 2);
			const int d2 = ((s[0] & 0x20) << 2) | ((s[0] & 0x02) << 5) | ((s[1] & 0x20)) | ((s[1] & 0x02) << 3) |
			               ((s[2] & 0x20) >> 2) | ((s[2] & 0x02) << 1) | ((s[3] & 0x20) >> 4) | ((s[3] & 0x02) >> 1);
			const int d3 = ((s[0] & 0x10) << 3) | ((s[0] & 0x01) << 6) | ((s[1] & 0x10) << 1) | ((s[1] & 0x01) << 4) |
			               ((s[2] & 0x10) >> 1) | ((s[2] & 0x01) << 2) | ((s[3] & 0x10) >> 3) | ((s[3] & 0x01));
			c[i] = (uint8_t)d3;
			c[i + 1] = (uint8_t)d1;
			c[i + 2] = (uint8_t)d2;
			c[i + 3] = (uint8_t)d0;
			c[i + 4] = s[4];
		}
		for (; i < k->rom_size; i++)
			c[i] = k->rom[i];
		decode_tiles5(s_tiles, c, s_ntiles);
		free(c);
	} else if (k->bpp == K056832_BPP_5) {
		decode_tiles5(s_tiles, k->rom, s_ntiles);
		if (getenv("RBX_GX_DECODECHECK")) {
			uint8_t *t = (uint8_t *)malloc((size_t)s_ntiles * 64);
			decode(t, k->rom, k->rom_size, s_ntiles, 8, 8, planes, po, xo, yo, inc);
			rbx_log("gx decode check: tiles %s", t && !memcmp(t, s_tiles, (size_t)s_ntiles * 64) ? "same" : "DIFFERENT");
			free(t);
		}
	} else
		decode(s_tiles, k->rom, k->rom_size, s_ntiles, 8, 8, planes, po, xo, yo, inc);
	s_tile_gran = 16;
	return 0;
}

static int decode_sprites(const k055673_t *k)
{
	if (!k->rom || !k->rom_size) {   // (no sprite chip: bishi)
		s_nsprites = 0;
		s_spr_gran = 16;
		return 0;
	}
	if (k->bpp != K055673_LAYOUT_GX) {   // (the other layouts through the generic decoder, MAME's gfx layouts)
		int planes, inc, po[8], xo[16], yo[16];
		uint32_t total;
		if (k->bpp == K055673_LAYOUT_053245 || k->bpp == K055673_LAYOUT_051960) {   // (k053244_device / k051960_device::spritelayout)
			static const int p53245[4] = { 24, 16, 8, 0 }, p51960[4] = { 0, 8, 16, 24 };
			const int *p = k->bpp == K055673_LAYOUT_051960 ? p51960 : p53245;
			static const int x[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 256, 257, 258, 259, 260, 261, 262, 263 };
			static const int yy[16] = { 0, 32, 64, 96, 128, 160, 192, 224, 512, 544, 576, 608, 640, 672, 704, 736 };
			planes = 4; memcpy(po, p, 4 * sizeof(int)); memcpy(xo, x, sizeof(x)); memcpy(yo, yy, sizeof(yy));
			inc = 128 * 8; total = k->rom_size / 128;
		} else if (k->bpp == K055673_LAYOUT_053247) {   // (k053247_device's NORMAL_PLANE_ORDER spritelayout)
			static const int p[4] = { 0, 1, 2, 3 };
			static const int x[16] = { 2 * 4, 3 * 4, 0 * 4, 1 * 4, 6 * 4, 7 * 4, 4 * 4, 5 * 4,
			                           10 * 4, 11 * 4, 8 * 4, 9 * 4, 14 * 4, 15 * 4, 12 * 4, 13 * 4 };
			planes = 4; memcpy(po, p, sizeof(p)); memcpy(xo, x, sizeof(x));
			for (int i = 0; i < 16; i++) yo[i] = i * 64;
			inc = 128 * 8; total = k->rom_size / 128;
		} else if (k->bpp == K055673_LAYOUT_GRADIUS3) {   // (k051960_device::spritelayout_gradius3)
			static const int p[4] = { 0, 1, 2, 3 };
			static const int x[16] = { 2 * 4, 3 * 4, 0 * 4, 1 * 4, 6 * 4, 7 * 4, 4 * 4, 5 * 4, 256 + 2 * 4, 256 + 3 * 4,
			                           256 + 0 * 4, 256 + 1 * 4, 256 + 6 * 4, 256 + 7 * 4, 256 + 4 * 4, 256 + 5 * 4 };
			static const int yy[16] = { 0, 32, 64, 96, 128, 160, 192, 224, 512, 544, 576, 608, 640, 672, 704, 736 };
			planes = 4; memcpy(po, p, sizeof(p)); memcpy(xo, x, sizeof(x)); memcpy(yo, yy, sizeof(yy));
			inc = 128 * 8; total = k->rom_size / 128;
		} else if (k->bpp == K055673_LAYOUT_RNG) {   // spritelayout2 (Run and Gun)
			static const int p[4] = { 24, 16, 8, 0 };
			static const int x[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 32, 33, 34, 35, 36, 37, 38, 39 };
			planes = 4; memcpy(po, p, sizeof(p)); memcpy(xo, x, sizeof(x));
			for (int i = 0; i < 16; i++) yo[i] = i * 64;
			inc = 16 * 16 * 4; total = k->rom_size / (16 * 16 / 2);
		} else if (k->bpp == K055673_LAYOUT_LE2) {   // spritelayout3 (Lethal Enforcers II)
			static const int p[8] = { 56, 48, 40, 32, 24, 16, 8, 0 };
			static const int x[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 64, 65, 66, 67, 68, 69, 70, 71 };
			planes = 8; memcpy(po, p, sizeof(p)); memcpy(xo, x, sizeof(x));
			for (int i = 0; i < 16; i++) yo[i] = i * 128;
			inc = 16 * 16 * 8; total = k->rom_size / (16 * 16);
		} else if (k->bpp == K055673_LAYOUT_GX6) {   // spritelayout4 (GX 6bpp)
			static const int p[6] = { 40, 32, 24, 16, 8, 0 };
			static const int x[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 48, 49, 50, 51, 52, 53, 54, 55 };
			planes = 6; memcpy(po, p, sizeof(p)); memcpy(xo, x, sizeof(x));
			for (int i = 0; i < 16; i++) yo[i] = i * 12 * 8;
			inc = 16 * 16 * 6; total = k->rom_size / (16 * 16 * 6 / 8);
		} else
			return -1;
		s_nsprites = total;
		s_sprites = (uint8_t *)malloc((size_t)s_nsprites * 256);
		if (!s_sprites)
			return -1;
		decode(s_sprites, k->rom, k->rom_size, s_nsprites, 16, 16, planes, po, xo, yo, inc);
		s_spr_gran = 1 << planes;
		return 0;
	}
	// combine the 4bpp area and the 1bpp area to form 5bpp: 4 bytes of one, 1 of the other
	const uint32_t size4 = (k->rom_size / 0x100000) / 5 * 0x400000;
	uint8_t *comb = (uint8_t *)malloc((size_t)size4 * 5 / 4 + 16);
	if (!comb)
		return -1;
	const uint8_t *s1 = k->rom, *s2 = k->rom + size4;
	uint8_t *d = comb;
	for (uint32_t i = 0; i < size4; i += 4) {
		*d++ = *s1++;
		*d++ = *s1++;
		*d++ = *s1++;
		*d++ = *s1++;
		*d++ = *s2++;
	}
	static const int po[5] = { 32, 24, 16, 8, 0 };
	static const int xo[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 40, 41, 42, 43, 44, 45, 46, 47 };
	int yo[16];
	for (int i = 0; i < 16; i++)
		yo[i] = i * 10 * 8;
	s_nsprites = size4 / 128;
	s_sprites = (uint8_t *)malloc((size_t)s_nsprites * 256);
	if (!s_sprites) {
		free(comb);
		return -1;
	}
	decode_sprites5(s_sprites, comb, s_nsprites);
	if (getenv("RBX_GX_DECODECHECK")) {
		uint8_t *t = (uint8_t *)malloc((size_t)s_nsprites * 256);
		decode(t, comb, size4 * 5 / 4, s_nsprites, 16, 16, 5, po, xo, yo, 16 * 16 * 5);
		rbx_log("gx decode check: sprites %s", t && !memcmp(t, s_sprites, (size_t)s_nsprites * 256) ? "same" : "DIFFERENT");
		free(t);
	}
	free(comb);
	s_spr_gran = 32;
	return 0;
}

// ---- the snapshot at vblank (what the drawing reads) ----
typedef struct {
	uint32_t pal[8192];           // xRGB_888
	uint16_t spriteram[0x800];
	uint8_t k55555[0x80];
	uint16_t k54338[0x10];
	uint8_t tilebanks[16];
	uint8_t wrport1_0, wrport2;
	k056832_t k56832;             // (videoram, registers, layout)
	uint16_t kx47_regs[16];
	uint8_t kx46_regs[8];
	k053250_snap_t lvc;           // (the K053250's line buffer and registers)
} snap_t;
static snap_t *s_snap;

// ---- the drawing's state (MAME konamigx_state members kept across frames) ----
static uint32_t *s_frame;                // the bitmap's visible area (VIS_W x VIS_H)
static uint8_t s_objzbuf[VIS_H_MAX * GX_ZBUFW], s_shdzbuf[VIS_H_MAX * GX_ZBUFW * 2];
static int s_pal_n = 8192;               // (its entries: gx.pal_entries)
static uint32_t s_shadow[4][32768];      // MAME's shadow tables (0, 2: shadows, 1, 3: highlights)
static struct { int dr, dg, db, noclip, set; } s_shd_cfg[4];
static int s_shd_rgb[9];
static int s_primode, s_tilemode;
// precache_registers
static int s_vrcbk[4], s_coreg, s_coregshift, s_opset, s_opri, s_oinprion, s_vcblk[6], s_ocblk, s_vinmix, s_vmixon;
static int s_osinmix, s_osmixon;
static uint8_t s_brightness[3];

static inline int pal5bit(int b) { b &= 31; return (b << 3) | (b >> 2); }
static inline int clamp8(int v) { return v < 0 ? 0 : v > 255 ? 255 : v; }

static void set_shadow_drgb32(int mode, int dr, int dg, int db, int noclip)
{
	if (dr < -0xff) dr = -0xff; else if (dr > 0xff) dr = 0xff;
	if (dg < -0xff) dg = -0xff; else if (dg > 0xff) dg = 0xff;
	if (db < -0xff) db = -0xff; else if (db > 0xff) db = 0xff;
	if (s_shd_cfg[mode].set && dr == s_shd_cfg[mode].dr && dg == s_shd_cfg[mode].dg && db == s_shd_cfg[mode].db &&
	    noclip == s_shd_cfg[mode].noclip)
		return;
	s_shd_cfg[mode].dr = dr;
	s_shd_cfg[mode].dg = dg;
	s_shd_cfg[mode].db = db;
	s_shd_cfg[mode].noclip = noclip;
	s_shd_cfg[mode].set = 1;
	for (int i = 0; i < 32768; i++) {
		int r = pal5bit(i >> 10) + dr, g = pal5bit(i >> 5) + dg, b = pal5bit(i) + db;
		if (!noclip) {
			r = clamp8(r);
			g = clamp8(g);
			b = clamp8(b);
		}
		s_shadow[mode][i] = (uint32_t)((r & 0xff) << 16 | (g & 0xff) << 8 | (b & 0xff));
	}
}

// configure_rgb_shadows(mode, factor): the defaults before the K054338 sets them (0.6 shadow, 1/0.6 highlight)
static void configure_rgb_shadows(int mode, float factor)
{
	const int ifactor = (int)(factor * 256.0f);
	for (int i = 0; i < 32768; i++) {
		const int r = clamp8((pal5bit(i >> 10) * ifactor) >> 8), g = clamp8((pal5bit(i >> 5) * ifactor) >> 8),
		          b = clamp8((pal5bit(i) * ifactor) >> 8);
		s_shadow[mode][i] = (uint32_t)(r << 16 | g << 8 | b);
	}
}

static inline uint32_t alpha_blend_r32(uint32_t d, uint32_t s, int level)   // (MAME alpha_blend_r32)
{
	return ((((s & 0xff00ff) * (uint32_t)level + (d & 0xff00ff) * (uint32_t)(256 - level)) >> 8) & 0xff00ff) |
	       ((((s & 0x00ff00) * (uint32_t)level + (d & 0x00ff00) * (uint32_t)(256 - level)) >> 8) & 0x00ff00);
}
static inline uint32_t add_blend_r32(uint32_t d, uint32_t s)   // (MAME add_blend_r32)
{
	uint32_t r = (d & 0xff0000) + (s & 0xff0000), g = (d & 0x00ff00) + (s & 0x00ff00), b = (d & 0x0000ff) + (s & 0x0000ff);
	if (r > 0xff0000) r = 0xff0000;
	if (g > 0x00ff00) g = 0x00ff00;
	if (b > 0x0000ff) b = 0x0000ff;
	return r | g | b;
}

static int set_alpha_level(int pblend)   // (k054338_device::set_alpha_level, alpha inverted: mixer_init)
{
	if (!pblend)
		return 255;
	pblend &= 3;
	const uint16_t *r = s_snap->k54338;
	const uint8_t mixset = (uint8_t)(r[K338_REG_PBLEND + (pblend >> 1 & 1)] >> (~pblend << 3 & 8));
	int mixlv = mixset & 0x1f;
	if (!gx.alpha_noinvert)
		mixlv = 0x1f - mixlv;
	mixlv = (mixlv << 3) | (mixlv >> 2);
	mixlv |= (mixset & 0x20) ? 0x100 : 0;
	mixlv |= (r[K338_REG_CONTROL] & K338_CTL_MIXPRI) ? 0x200 : 0;
	return mixlv;
}

static void precache_registers(void)
{
	static const int coregmasks[5] = { 0xf, 0xe, 0xc, 0x8, 0x0 };
	static const int coregshifts[5] = { 4, 5, 6, 7, 8 };
	const uint16_t *k47 = s_snap->kx47_regs;
	const uint8_t *k55 = s_snap->k55555;
	int i = k47[0x8 / 2];
	s_vrcbk[0] = (i & 0x000f) << 14;
	s_vrcbk[1] = (i & 0x0f00) << 6;
	i = k47[0xa / 2];
	s_vrcbk[2] = (i & 0x000f) << 14;
	s_vrcbk[3] = (i & 0x0f00) << 6;
	s_opset = k47[0xc / 2];
	i = s_opset & 7;
	if (i > 4)
		i = 4;
	s_coreg = (k47[0xc / 2] >> 8) & 0xf;
	s_coreg = (s_coreg & coregmasks[i]) << 12;
	s_coregshift = coregshifts[i];
	s_opri = k55[K55_PRIINP_8];
	s_oinprion = k55[K55_OINPRI_ON];
	for (int l = 0; l < 4; l++)
		s_vcblk[l] = k55[K55_PALBASE_A + l];
	s_vcblk[4] = k55[K55_PALBASE_SUB1];
	s_vcblk[5] = k55[K55_PALBASE_SUB2];
	s_ocblk = k55[K55_PALBASE_OBJ];
	s_vinmix = k55[K55_BLEND_ENABLES];
	s_vmixon = k55[K55_VINMIX_ON];
	s_osinmix = k55[K55_OSBLEND_ENABLES];
	s_osmixon = k55[K55_OSBLEND_ON];
	s_brightness[0] = (uint8_t)s_snap->k54338[K338_REG_BRI3];
	s_brightness[1] = (uint8_t)(s_snap->k54338[K338_REG_BRI3 + 1] >> 8);
	s_brightness[2] = (uint8_t)s_snap->k54338[K338_REG_BRI3 + 1];
}

static int decode_vmixcolor(int layer, int *color)   // (K055555GX_decode_vmixcolor)
{
	const int vcb = s_vcblk[layer] << 6, shift = layer << 1;
	int pal = *color, vmx = s_vinmix >> shift & 3, von = s_vmixon >> shift & 3, pl45, emx;
	emx = pl45 = pal >> 4 & 3;
	pal &= 0xf;
	pl45 &= von;
	vmx &= von;
	pl45 <<= 4;
	emx &= ~von;
	pal |= pl45;
	emx |= vmx;
	pal |= vcb;
	if (von == 3)
		emx = -1;   // (invalidate the external mix code if all bits are from internal)
	*color = pal;
	return emx;
}

// ---- the K053247 sprite callback (type2_sprite_callback) ----
static inline int combine_c18(int attrib)
{
	int c18 = (attrib & 0xff) << s_coregshift | s_coreg;
	if (s_snap->wrport2 & 4)
		c18 &= 0x3fff;
	else if (!(s_snap->wrport2 & 8))
		c18 = (c18 & 0x3fff) | (attrib << 6 & 0xc000);
	return c18;
}
static int decode_objcolor(int c18)
{
	int opon = s_oinprion << 8 | 0xff, ocb = (s_ocblk & 7) << 10;
	c18 &= opon;
	ocb &= ~opon;
	return (ocb | c18) >> s_coregshift;
}
static void sprite_callback(int *code, int *color, int *priority)
{
	if (gx.sprcb >= GX_SPRCB_MW) {   // (mystwarr_v.cpp: the code as it is, the colour base from the K055555)
		const int c = *color,
		          cb = (s_snap->k55555[K55_PALBASE_OBJ] << gx.spr_cbshift) & (gx.spr_cbmask ? gx.spr_cbmask : ~0);
		switch (gx.sprcb) {
		case GX_SPRCB_MW:
			*priority = c & 0xe0;
			*color = cb | (c & 0x1f) | ((c >> 8) & 3) << K055555_MIXSHIFT;
			break;
		case GX_SPRCB_METAMRPH:
			*priority = (c & 0xe0) >> 2;
			*color = (int)((uint32_t)(cb | (c & 0x1f) | ((c >> 8) & 3) << K055555_MIXSHIFT) |
			               (((c >> 10) & 3) == 3 ? K055555_SKIPSHADOW : 0));
			break;
		case GX_SPRCB_GAIAPOLS:
			*priority = c & 0xe0;
			*color = cb | (c >> 4 & 0x20) | (c & 0x1f);
			break;
		default:   // (martchmp)
			*color = (c & 0x3ff) == 0x11f ? (int)K055555_FULLSHADOW : cb | (c & 0x1f);
			*priority = (s_oinprion & 0xf0) ? s_opri : c & 0xf0;
			break;
		}
		return;
	}
	const int num = *code;
	*code = s_vrcbk[num >> 14] | (num & 0x3fff);
	if (gx.sprcb != GX_SPRCB_TYPE2) {   // (dragoonj / salmndr2 / le2_sprite_callback)
		int pri = *color, op = s_opri;
		if (gx.sprcb == GX_SPRCB_DRAGOONJ)
			pri = (pri & 0x200) ? 4 : pri >> 4 & 0xf;
		else if (gx.sprcb == GX_SPRCB_SALMNDR2)
			pri = pri >> 4 & 0x3f;
		else
			pri &= 0xf0;
		op &= s_oinprion;
		pri &= ~s_oinprion;
		*priority = pri | op;
		if (gx.sprcb == GX_SPRCB_LE2)
			*color &= 0x1f;
		else
			*color = decode_objcolor(combine_c18(*color));
		return;
	}
	const int c18 = combine_c18(*color);
	int opon = s_oinprion << 8 | 0xff, ocb = (s_ocblk & 7) << 10, c = c18 & opon;   // (decode_objcolor)
	ocb &= ~opon;
	const int newcolor = (ocb | c) >> s_coregshift;
	int op = s_opri, cc = c18 >> 8;   // (decode_inpri)
	op &= s_oinprion;
	cc &= ~s_oinprion;
	*priority = cc | op;
	*color = newcolor;
}

typedef struct { const uint8_t *base; int penbase; uint8_t fx, fy, cat; } tentry_t;
typedef struct { uint32_t gen; int layer, row; uint8_t ok[64]; tentry_t e[64]; } tcache_t;
static uint32_t s_tc_gen = 1;   // (a new frame: the caches empty)
static int s_tc_side;           // (the tile callback sets the alpha mix code: a draw's own cache only)

// the decoded tile of a code (MAME's gfx element: the code modulo the count; the division only past the end)
static inline uint32_t tile_index(uint32_t code)
{
	if (code < s_ntiles)
		return code;
	return s_ntiles ? rbx_wrap(code, s_ntiles) : 0;
}

// one page drawn into the frame over the bitmap rectangle [x0, x1] x [y0, y1] with its scroll (the tilemap's
// scrollx / scrolly), MAME flags: category (or all), opaque, alpha
// ---- the pre-GX frame: MAME's bitmap_ind16 (pens; the shadows at pen + entries, the black pen past them) and the
// screen's priority bitmap ----
static uint16_t *s_frame16;
static uint8_t *s_pri;
static int s_pri_code;   // (the tilemap's priority: or'ed into the priority bitmap where it draws)
static int s_pre_opaque;   // (TILEMAP_DRAW_OPAQUE: pen 0 drawn too)
static int s_pri_code_w1, s_pri_code_w2, s_pre_opaque_w1, s_pre_opaque_w2;   // (the pre-GX bands' workers)

// ---- the drawing: a copy for each worker (0: the render thread, 1: the emulation thread, 2: the helper core) ----
typedef struct { uint32_t order; int offs, code, color; } gx_obj_t;
static gx_obj_t s_pool[6 + 512];
static struct { int n, draw; } s_job;   // (the frame's pool count, the mixer drawing at all)
#include "konamigx_draw.inc"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#define s_pens s_pens_w1
#define s_current_brightness s_current_brightness_w1
#define s_shadow_mode s_shadow_mode_w1
#define s_last_alpha_tile_mix_code s_last_alpha_tile_mix_code_w1
#define s_mix_key s_mix_key_w1
#define s_cur_key s_cur_key_w1
#define s_draw_seq s_draw_seq_w1
#define s_pt_fy s_pt_fy_w1
#define s_tc s_tc_w1
#define s_by0 s_by0_w1
#define s_by1 s_by1_w1
#define s_pri_code s_pri_code_w1
#define s_pre_opaque s_pre_opaque_w1
#define set_brightness_value set_brightness_value_w1
#define set_brightness set_brightness_w1
#define page_tile page_tile_w1
#define tc_entry tc_entry_w1
#define draw_page16 draw_page16_w1
#define draw_page32p draw_page32p_w1
#define draw_page draw_page_w1
#define tilemap_draw tilemap_draw_w1
#define draw_basic_tilemaps draw_basic_tilemaps_w1
#define draw_lvc draw_lvc_w1
#define draw_roz draw_roz_w1
#define zdrawgfxzoom32gp zdrawgfxzoom32gp_w1
#define draw_single_sprite draw_single_sprite_w1
#define mixer_band mixer_band_w1
#include "konamigx_draw.inc"
#undef s_pens
#undef s_current_brightness
#undef s_shadow_mode
#undef s_last_alpha_tile_mix_code
#undef s_mix_key
#undef s_cur_key
#undef s_draw_seq
#undef s_pt_fy
#undef s_tc
#undef s_by0
#undef s_by1
#undef s_pri_code
#undef s_pre_opaque
#undef set_brightness_value
#undef set_brightness
#undef page_tile
#undef tc_entry
#undef draw_page16
#undef draw_page32p
#undef draw_page
#undef tilemap_draw
#undef draw_basic_tilemaps
#undef draw_lvc
#undef draw_roz
#undef zdrawgfxzoom32gp
#undef draw_single_sprite
#undef mixer_band
#define s_pens s_pens_w2
#define s_current_brightness s_current_brightness_w2
#define s_shadow_mode s_shadow_mode_w2
#define s_last_alpha_tile_mix_code s_last_alpha_tile_mix_code_w2
#define s_mix_key s_mix_key_w2
#define s_cur_key s_cur_key_w2
#define s_draw_seq s_draw_seq_w2
#define s_pt_fy s_pt_fy_w2
#define s_tc s_tc_w2
#define s_by0 s_by0_w2
#define s_by1 s_by1_w2
#define s_pri_code s_pri_code_w2
#define s_pre_opaque s_pre_opaque_w2
#define set_brightness_value set_brightness_value_w2
#define set_brightness set_brightness_w2
#define page_tile page_tile_w2
#define tc_entry tc_entry_w2
#define draw_page16 draw_page16_w2
#define draw_page32p draw_page32p_w2
#define draw_page draw_page_w2
#define tilemap_draw tilemap_draw_w2
#define draw_basic_tilemaps draw_basic_tilemaps_w2
#define draw_lvc draw_lvc_w2
#define draw_roz draw_roz_w2
#define zdrawgfxzoom32gp zdrawgfxzoom32gp_w2
#define draw_single_sprite draw_single_sprite_w2
#define mixer_band mixer_band_w2
#include "konamigx_draw.inc"
#undef s_pens
#undef s_current_brightness
#undef s_shadow_mode
#undef s_last_alpha_tile_mix_code
#undef s_mix_key
#undef s_cur_key
#undef s_draw_seq
#undef s_pt_fy
#undef s_tc
#undef s_by0
#undef s_by1
#undef s_pri_code
#undef s_pre_opaque
#undef set_brightness_value
#undef set_brightness
#undef page_tile
#undef tc_entry
#undef draw_page16
#undef draw_page32p
#undef draw_page
#undef tilemap_draw
#undef draw_basic_tilemaps
#undef draw_lvc
#undef draw_roz
#undef zdrawgfxzoom32gp
#undef draw_single_sprite
#undef mixer_band
#pragma GCC diagnostic pop

// ---- the pre-GX K053247 sprites (k053247_sprites_draw_common, the non-GX side of
// k053247_draw_single_sprite_gxcore and k053247_draw_yxloop_gx, gfx_element::prio_(zoom_)transtable into bitmap_ind16) ----
enum { DM_NONE, DM_SOURCE, DM_SHADOW, DM_SHADOW_PRI };

#include "konamigx_pre.inc"
#define s_pby0 s_pby0_w1
#define s_pby1 s_pby1_w1
#define pre_pixel pre_pixel_w1
#define s_pre_shadow s_pre_shadow_w1
#define pre_pixel32 pre_pixel32_w1
#define s_pclip s_pclip_w1
#define pre_drawgfx pre_drawgfx_w1
#define pre_sprites_core pre_sprites_core_w1
#define rgb15 rgb15_w1
#include "konamigx_pre.inc"
#undef s_pby0
#undef s_pby1
#undef pre_pixel
#undef s_pre_shadow
#undef pre_pixel32
#undef s_pclip
#undef pre_drawgfx
#undef pre_sprites_core
#undef rgb15
#define s_pby0 s_pby0_w2
#define s_pby1 s_pby1_w2
#define pre_pixel pre_pixel_w2
#define s_pre_shadow s_pre_shadow_w2
#define pre_pixel32 pre_pixel32_w2
#define s_pclip s_pclip_w2
#define pre_drawgfx pre_drawgfx_w2
#define pre_sprites_core pre_sprites_core_w2
#define rgb15 rgb15_w2
#include "konamigx_pre.inc"
#undef s_pby0
#undef s_pby1
#undef pre_pixel
#undef s_pre_shadow
#undef pre_pixel32
#undef s_pclip
#undef pre_drawgfx
#undef pre_sprites_core
#undef rgb15


// k053244_device::sprites_draw (bitmap_ind16): the 128 sprites of the buffer by their priority codes
void gx_pre_sprites_053245(const uint16_t *buffer, const uint8_t *regs, int dx, int dy, int z_rejection)
{
	enum { NUM_SPRITES = 128, RAM_WORDS = 0x400 };
	uint8_t drawmode_table[256];
	memset(drawmode_table, DM_SOURCE, sizeof(drawmode_table));
	drawmode_table[0] = DM_NONE;
	const uint8_t shadow_mode = gx.pre_k245_prishadow ? DM_SHADOW_PRI : DM_SHADOW;   // (m_priority_shadows)
	const int flipscreenx = regs[5] & 0x01, flipscreeny = regs[5] & 0x02;
	const int spriteoffsx = regs[0] << 8 | regs[1], spriteoffsy = regs[2] << 8 | regs[3];
	int sortedlist[NUM_SPRITES];
	for (int i = 0; i < NUM_SPRITES; i++)
		sortedlist[i] = -1;
	for (int offs = 0; offs < RAM_WORDS; offs += 8) {
		int pri_code = buffer[offs];
		if (pri_code & 0x8000) {
			pri_code &= 0x7f;
			if (offs && pri_code == z_rejection)
				continue;
			if (sortedlist[pri_code] == -1)
				sortedlist[pri_code] = offs;
		}
	}
	for (int pri_code = NUM_SPRITES - 1; pri_code >= 0; pri_code--) {
		const int offs = sortedlist[pri_code];
		if (offs == -1)
			continue;
		int code = buffer[offs + 1];
		code = (code & 0xffe1) + ((code & 0x0010) >> 2) + ((code & 0x0008) << 1) + ((code & 0x0004) >> 1) +
		       ((code & 0x0002) << 2);
		int color = buffer[offs + 6] & 0x00ff, pri = 0;
		gx.pre_sprcb(&code, &color, &pri);
		const int size = (buffer[offs] & 0x0f00) >> 8, w = 1 << (size & 3), h = 1 << ((size >> 2) & 3);
		int zoomy = buffer[offs + 4], zoomx;
		if (zoomy > 0x2000)
			continue;
		zoomy = zoomy ? (0x400000 + zoomy / 2) / zoomy : 2 * 0x400000;
		if (!(buffer[offs] & 0x4000)) {
			zoomx = buffer[offs + 5];
			if (zoomx > 0x2000)
				continue;
			zoomx = zoomx ? (0x400000 + zoomx / 2) / zoomx : 2 * 0x400000;
		} else
			zoomx = zoomy;
		int ox = buffer[offs + 3] + spriteoffsx, oy = buffer[offs + 2];
		ox -= 96;
		ox += dx;
		oy += dy;
		int flipx = buffer[offs] & 0x1000, flipy = buffer[offs] & 0x2000;
		const int mirrorx = buffer[offs + 6] & 0x0100, mirrory = buffer[offs + 6] & 0x0200;
		const int shadow = buffer[offs + 6] & 0x0080;
		if (mirrorx)
			flipx = 0;
		if (flipscreenx) {
			ox = 320 - ox;
			if (!mirrorx)
				flipx = !flipx;
		}
		if (flipscreeny) {
			oy = -oy;
			if (!mirrory)
				flipy = !flipy;
		}
		ox = (ox + 0x5d) & 0x3ff;
		if (ox >= 768)
			ox -= 1024;
		oy = (-(oy + spriteoffsy + 0x07)) & 0x3ff;
		if (oy >= 640)
			oy -= 1024;
		// (the clip: { ox - 0x200, ox + 0x1ff, oy - 0x100, oy + 0xff } & the visible area)
		s_pclip[0] = ox - 0x200 > VIS_X0 ? ox - 0x200 : VIS_X0;
		s_pclip[1] = ox + 0x1ff < VIS_X0 + VIS_W - 1 ? ox + 0x1ff : VIS_X0 + VIS_W - 1;
		s_pclip[2] = oy - 0x100 > VIS_Y0 ? oy - 0x100 : VIS_Y0;
		s_pclip[3] = oy + 0xff < VIS_Y0 + VIS_H - 1 ? oy + 0xff : VIS_Y0 + VIS_H - 1;
		ox -= (zoomx * w) >> 13;
		oy -= (zoomy * h) >> 13;
		drawmode_table[s_spr_gran - 1] = shadow ? shadow_mode : DM_SOURCE;
		s_pre_shadow = 0;
		for (int y = 0; y < h; y++) {
			const int sy = oy + ((zoomy * y + (1 << 11)) >> 12), zh = (oy + ((zoomy * (y + 1) + (1 << 11)) >> 12)) - sy;
			for (int x = 0; x < w; x++) {
				const int sx = ox + ((zoomx * x + (1 << 11)) >> 12), zw = (ox + ((zoomx * (x + 1) + (1 << 11)) >> 12)) - sx;
				int c = code, fx, fy;
				if (mirrorx) {
					if ((flipx == 0) ^ (2 * x < w)) {
						c += w - x - 1;
						fx = 1;
					} else {
						c += x;
						fx = 0;
					}
				} else {
					c += flipx ? w - 1 - x : x;
					fx = flipx != 0;
				}
				if (mirrory) {
					if ((flipy == 0) ^ (2 * y >= h)) {
						c += 8 * (h - y - 1);
						fy = 1;
					} else {
						c += 8 * y;
						fy = 0;
					}
				} else {
					c += flipy ? 8 * (h - 1 - y) : 8 * y;
					fy = flipy != 0;
				}
				c = (c & 0x3f) | (code & ~0x3f);
				const int z = !(zoomx == 0x10000 && zoomy == 0x10000);
				const uint32_t scx = (uint32_t)zw << 12, scy = (uint32_t)zh << 12;
				// (prio_zoom_transtable: the non-zoom case when its scale is exactly 1 too)
				pre_drawgfx((uint32_t)c, (uint32_t)color, fx, fy, sx, sy, scx, scy, z && !(scx == 0x10000 && scy == 0x10000),
				            (uint32_t)pri, drawmode_table);
			}
		}
	}
}

void gx_pre_fill_pen(int pen)   // (bitmap.fill(pen) of the ind16 frame, the priority bitmap 0)
{
	const uint32_t b2 = (uint32_t)pen * 0x10001u;
	uint32_t *f = (uint32_t *)s_frame16;
	for (int i = 0; i < VIS_W * VIS_H / 2; i++)
		f[i] = b2;
	memset(s_pri, 0, (size_t)(VIS_W * VIS_H));
}

// k051960_device::k051960_sprites_draw (bitmap_ind16, the priority bitmap)
void gx_pre_sprites_051960(const uint8_t *buffer, int control, int shadow_config, int min_priority, int max_priority,
                           void (*cb)(int *code, int *color, int *priority, int *shadow), int priority_shadows)
{
	enum { NUM_SPRITES = 128 };
	static const int xoffset[8] = { 0, 1, 4, 5, 16, 17, 20, 21 };
	static const int yoffset[8] = { 0, 2, 8, 10, 32, 34, 40, 42 };
	static const int width[8] = { 1, 2, 1, 2, 4, 2, 4, 8 };
	static const int height[8] = { 1, 1, 2, 2, 2, 4, 4, 8 };
	static const uint8_t zoomy_table[0x40] = {
		0x00, 0x01, 0x03, 0x05, 0x07, 0x09, 0x0a, 0x0c, 0x0e, 0x0f, 0x11, 0x12, 0x14, 0x15, 0x16, 0x18,
		0x19, 0x1a, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29,
		0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2e, 0x2f, 0x30, 0x31, 0x31, 0x32, 0x33, 0x34, 0x34, 0x35, 0x36,
		0x36, 0x37, 0x38, 0x38, 0x39, 0x39, 0x3a, 0x3b, 0x3b, 0x3c, 0x3c, 0x3d, 0x3d, 0x3e, 0x3e, 0x3f };
	uint8_t drawmode_table[256];
	memset(drawmode_table, DM_SOURCE, sizeof(drawmode_table));
	drawmode_table[0] = DM_NONE;
	const uint8_t shadow_mode = priority_shadows ? DM_SHADOW_PRI : DM_SHADOW;
	int sortedlist[NUM_SPRITES];
	for (int i = 0; i < NUM_SPRITES; i++)
		sortedlist[i] = -1;
	for (int offs = 0; offs < 0x400; offs += 8)   // (prebuild a sorted table)
		if (buffer[offs] & 0x80) {
			int pri_code = buffer[offs] & 0x7f;
			if (max_priority == -1)   // (front to back with the priority bitmap)
				pri_code ^= 0x7f;
			sortedlist[pri_code] = offs;
		}
	s_pclip[0] = VIS_X0;
	s_pclip[1] = VIS_X0 + VIS_W - 1;
	s_pclip[2] = VIS_Y0;
	s_pclip[3] = VIS_Y0 + VIS_H - 1;
	for (int pri_code = 0; pri_code < NUM_SPRITES; pri_code++) {
		const int offs = sortedlist[pri_code];
		if (offs == -1)
			continue;
		int code = buffer[offs + 2] + ((buffer[offs + 1] & 0x1f) << 8);
		int color = buffer[offs + 3] & 0xff;
		int pri = 0;
		int shadow = !(shadow_config & 4) && ((shadow_config & 2) || (color & 0x80));
		cb(&code, &color, &pri, &shadow);
		if (max_priority != -1 && (pri < min_priority || pri > max_priority))
			continue;
		const int size = (buffer[offs + 1] & 0xe0) >> 5, w = width[size], h = height[size];
		if (w >= 2) code &= ~0x01;
		if (h >= 2) code &= ~0x02;
		if (w >= 4) code &= ~0x04;
		if (h >= 4) code &= ~0x08;
		if (w >= 8) code &= ~0x10;
		if (h >= 8) code &= ~0x20;
		int ox = (256 * buffer[offs + 6] + buffer[offs + 7]) & 0x01ff;
		int oy = 256 - ((256 * buffer[offs + 4] + buffer[offs + 5]) & 0x01ff);
		int flipx = buffer[offs + 6] & 0x02, flipy = buffer[offs + 4] & 0x02;
		int zoomx = (buffer[offs + 6] & 0xfc) >> 2;
		zoomx = 0x10000 / 128 * (128 - zoomx);
		int zoomy = (buffer[offs + 4] & 0xfc) >> 2;
		zoomy = 128 - zoomy_table[zoomy];
		for (int i = 0; i < 3; i++)   // (accumulated rounding up for each sprite height)
			if (h <= (1 << i))
				zoomy = (zoomy + 1) / 2;
		zoomy *= 8 / h;
		zoomy = 0x10000 / 128 * zoomy;
		if (control & 8) {
			ox = 512 - (zoomx * w >> 12) - ox;
			oy = 256 - (zoomy * h >> 12) - oy;
			flipx = !flipx;
			flipy = !flipy;
		}
		drawmode_table[s_spr_gran - 1] = shadow ? shadow_mode : DM_SOURCE;
		s_pre_shadow = 0;
		const int nozoom = zoomx == 0x10000 && zoomy == 0x10000;
		for (int y = 0; y < h; y++) {
			const int sy = nozoom ? oy + 16 * y : oy + ((zoomy * y + (1 << 11)) >> 12);
			const int zh = nozoom ? 16 : (oy + ((zoomy * (y + 1) + (1 << 11)) >> 12)) - sy;
			for (int x = 0; x < w; x++) {
				int c = code;
				const int sx = nozoom ? ox + 16 * x : ox + ((zoomx * x + (1 << 11)) >> 12);
				const int zw = nozoom ? 16 : (ox + ((zoomx * (x + 1) + (1 << 11)) >> 12)) - sx;
				c += flipx ? xoffset[w - 1 - x] : xoffset[x];
				c += flipy ? yoffset[h - 1 - y] : yoffset[y];
				// (prio_transtable, or prio_zoom_transtable at (zw << 16) / 16: the plain one when that is 1:1)
				const uint32_t scx = nozoom ? 0x10000 : (uint32_t)((zw << 16) / 16), scy = nozoom ? 0x10000 : (uint32_t)((zh << 16) / 16);
				pre_drawgfx((uint32_t)c, (uint32_t)color, flipx != 0, flipy != 0, (sx & 0x1ff) - 96, sy, scx, scy,
				            !(scx == 0x10000 && scy == 0x10000), max_priority == -1 ? (uint32_t)pri : PRE_NOPRI, drawmode_table);
			}
		}
	}
}

// ---- the pre-GX bands (gx.pre_bands): the board's drawing recorded once, then replayed by the workers, each over its
// band of rows (the RGB frame's fill, the tilemaps, the K053247 sprites) ----
enum { PO_FILL, PO_TILEMAP, PO_SPRITES, PO_LVC, PO_CALL };
typedef struct { int kind, layer, pri, all, cat, alpha, opaque; void (*fn)(int arg, int y0, int y1); } pre_op_t;
static pre_op_t s_pops[32];
static int s_npops, s_prec;

static void pre_record(int kind, int layer, int pri, int all, int cat, int alpha, int opaque)
{
	if (s_npops >= (int)(sizeof(s_pops) / sizeof(s_pops[0])))
		return;
	const pre_op_t op = { kind, layer, pri, all, cat, alpha, opaque, NULL };
	s_pops[s_npops++] = op;
}

// a board's own layer drawn over rows y0..y1 of the frame (from 0): recorded for the bands, else all of them
void gx_pre_call(void (*fn)(int arg, int y0, int y1), int arg)
{
	if (s_prec) {
		pre_record(PO_CALL, arg, 0, 0, 0, 0, 0);
		s_pops[s_npops - 1].fn = fn;
		return;
	}
	fn(arg, 0, VIS_H - 1);
}

void gx_pre_sprites(void)
{
	if (s_prec)
		pre_record(PO_SPRITES, 0, 0, 0, 0, 0, 0);
	else
		pre_sprites_core();
}

void gx_pre_fill(void)
{
	if (s_prec) {
		pre_record(PO_FILL, 0, 0, 0, 0, 0, 0);
		return;
	}
	if (!s_frame16) {   // (the RGB frame: the K054338's fill_solid_bg)
		const uint16_t *k338 = s_snap->k54338;
		const uint32_t bg = (uint32_t)(k338[K338_REG_BGC_R] & 0xff) << 16 | k338[K338_REG_BGC_GB];
		for (int i = 0; i < VIS_W * VIS_H; i++)
			s_frame[i] = bg;
		memset(s_pri, 0, (size_t)(VIS_W * VIS_H));
		return;
	}
	const int n = VIS_W * VIS_H;   // (even: the frame two pens a word)
	const uint32_t b2 = (uint32_t)(2 * s_pal_n) * 0x10001u;   // (the black pen)
	uint32_t *f = (uint32_t *)s_frame16;
	for (int i = 0; i < n / 2; i++)
		f[i] = b2;
	memset(s_pri, 0, (size_t)n);
}

void gx_pre_tilemap_alpha(int layer, int priority, int alpha)
{
	if (s_prec) {
		pre_record(PO_TILEMAP, layer, priority, 0, 0, alpha, 0);
		return;
	}
	s_pri_code = priority;
	tilemap_draw(&s_snap->k56832, layer, 0, 0, 0, alpha);
}

void gx_pre_tilemap_cat(int layer, int priority, int all_categories, int category, int alpha)
{
	if (s_prec) {
		pre_record(PO_TILEMAP, layer, priority, all_categories, category, alpha, 0);
		return;
	}
	s_pri_code = priority;
	tilemap_draw(&s_snap->k56832, layer, 0, all_categories, category, alpha);
}

void gx_pre_lvc(int colorbase, int priority)   // (k053250_device::draw into the RGB frame and the priority bitmap)
{
	if (!gx.k53250)
		return;
	if (s_prec) {   // (layer: the colour base, pri: the priority)
		pre_record(PO_LVC, colorbase, priority, 0, 0, 0, 0);
		return;
	}
	k053250_draw(gx.k53250, &s_snap->lvc, s_frame, VIS_W, VIS_X0, VIS_Y0, VIS_X0 + VIS_W - 1, VIS_Y0 + VIS_H - 1,
	             s_pens, s_pal_n, colorbase, VIS_X0 + VIS_W - 1, VIS_Y0 + VIS_H - 1, s_pri, (uint8_t)priority);
}

void gx_pre_shadows(void)   // (k054338_device::update_all_shadows(0, palette))
{
	const uint16_t *k338 = s_snap->k54338;
	const int noclip = (k338[K338_REG_CONTROL] & K338_CTL_CLIPSL) != 0;
	int d[9];
	for (int i = 0; i < 9; i++) {
		d[i] = k338[K338_REG_SHAD1R + i] & 0x1ff;
		if (d[i] >= 0x100)
			d[i] -= 0x200;
	}
	for (int i = 0; i < 3; i++)
		set_shadow_drgb32(i, d[i * 3], d[i * 3 + 1], d[i * 3 + 2], noclip);
}

int gx_pre_alpha_level(int blend) { return set_alpha_level(blend); }
const uint16_t *gx_pre_k054338(void) { return s_snap->k54338; }

void gx_pre_tilemap_opaque(int layer, int priority)
{
	if (s_prec) {
		pre_record(PO_TILEMAP, layer, priority, 0, 0, 255, 1);
		return;
	}
	s_pre_opaque = 1;
	gx_pre_tilemap(layer, priority);
	s_pre_opaque = 0;
}

uint16_t *gx_pre_frame16(void) { return s_frame16; }
uint8_t *gx_pre_pri(void) { return s_pri; }

void gx_pre_tilemap(int layer, int priority)
{
	if (s_prec) {
		pre_record(PO_TILEMAP, layer, priority, 0, 0, 255, 0);
		return;
	}
	s_pri_code = priority;
	tilemap_draw(&s_snap->k56832, layer, 0, 0, 0, 255);   // (flags 0: category 0)
}

const k056832_t *gx_pre_k056832(void) { return &s_snap->k56832; }

// ---- the mixer (konamigx_mixer, konamigx_mixer_draw) ----

// bishi_state::screen_update: the K054338's solid colour, then the enabled layers by the K055555's priorities
// (konami_sortlayers4), opaque, all categories
static void mixer_layers(void)
{
	const uint8_t *k55 = s_snap->k55555;
	const uint16_t *k338 = s_snap->k54338;
	const uint32_t bg = (uint32_t)(k338[K338_REG_BGC_R] & 0xff) << 16 | k338[K338_REG_BGC_GB];
	for (int i = 0; i < VIS_W * VIS_H; i++)
		s_frame[i] = bg;
	static const int pris[4] = { K55_PRIINP_0, K55_PRIINP_3, K55_PRIINP_6, K55_PRIINP_7 };
	int layers[4], layerpri[4];
	for (int i = 0; i < 4; i++) {
		layers[i] = i;
		layerpri[i] = k55[pris[i]];
	}
	for (int i = 0; i < 3; i++)   // (konami_sortlayers4: swapped when pri[a] <= pri[b], the highest drawn first)
		for (int j = i + 1; j < 4; j++)
			if (layerpri[i] <= layerpri[j]) {
				int t = layerpri[i]; layerpri[i] = layerpri[j]; layerpri[j] = t;
				t = layers[i]; layers[i] = layers[j]; layers[j] = t;
			}
	for (int i = 0; i < 4; i++)
		if ((k55[K55_INPUT_ENABLES] & (1 << layers[i])))   // (K55_INP_VRAM_A..D)
			tilemap_draw(&s_snap->k56832, layers[i], 0, 1, 0, 255);
}

static int mixer_setup(int mixerflags)   // (the pool's count; s_job.draw)
{
	const uint8_t *k55 = s_snap->k55555;
	const uint16_t *k338 = s_snap->k54338;
	const uint8_t disp = k55[K55_INPUT_ENABLES];
	s_job.draw = disp && (k338[K338_REG_CONTROL] & K338_CTL_KILL);
	if (!s_job.draw)
		return 0;
	if (mixerflags & GXMIX_NOZBUF)
		mixerflags |= GXMIX_NOSHADOW;
	precache_registers();
	uint8_t layerid[6] = { 0, 1, 2, 3, 4, 5 }, layerpri[6];
	layerpri[0] = k55[K55_PRIINP_0];
	layerpri[1] = k55[K55_PRIINP_3];
	layerpri[3] = k55[K55_PRIINP_7];
	layerpri[4] = k55[K55_PRIINP_9];
	layerpri[5] = k55[K55_PRIINP_10];
	int shdprisel;
	if (s_primode == -1) {
		layerpri[2] = (uint8_t)(k55[K55_PRIINP_3] + 0x20);
		shdprisel = 0x3f;
	} else {
		layerpri[2] = k55[K55_PRIINP_6];
		shdprisel = k55[K55_SHD_PRI_SEL];
	}
	(void)shdprisel;
	int shadowon[3] = { 0, 0, 0 };
	const uint8_t shdpri[3] = { k55[K55_SHAD1_PRI], k55[K55_SHAD2_PRI], k55[K55_SHAD3_PRI] };
	int spri_min = 0;
	if (!(mixerflags & GXMIX_NOSHADOW)) {
		// (update_all_shadows first: MAME reads the shadow deltas exported the frame before; the same values here
		// as they are refreshed below before the sprites use them)
		for (int i = 0, j = 0; i < 3; j += 3, i++) {
			int kk = s_shd_rgb[j];
			if (kk < -7 || kk > 7) { shadowon[i] = 1; continue; }
			kk = s_shd_rgb[j + 1];
			if (kk < -7 || kk > 7) { shadowon[i] = 1; continue; }
			kk = s_shd_rgb[j + 2];
			if (kk < -7 || kk > 7) shadowon[i] = 1;
		}
		const int temp = k55[K55_SHD_ON];
		for (int i = 0; i < 4; i++)
			if (!((temp >> i) & 1) && spri_min < layerpri[i])
				spri_min = layerpri[i];
		// (k054338 update_all_shadows)
		const int noclip = (k338[K338_REG_CONTROL] & K338_CTL_CLIPSL) != 0;
		for (int i = 0; i < 9; i++) {
			int d = k338[K338_REG_SHAD1R + i] & 0x1ff;
			if (d >= 0x100)
				d -= 0x200;
			s_shd_rgb[i] = d;
		}
		set_shadow_drgb32(0, s_shd_rgb[0], s_shd_rgb[1], s_shd_rgb[2], noclip);
		set_shadow_drgb32(1, s_shd_rgb[3], s_shd_rgb[4], s_shd_rgb[5], noclip);
		set_shadow_drgb32(2, s_shd_rgb[6], s_shd_rgb[7], s_shd_rgb[8], noclip);
	}
	for (int j = 0; j < 5; j++)   // (pre-sort the layers)
		for (int i = j + 1; i < 6; i++)
			if (layerpri[j] <= layerpri[i]) {
				uint8_t t = layerpri[j]; layerpri[j] = layerpri[i]; layerpri[i] = t;
				t = layerid[j]; layerid[j] = layerid[i]; layerid[i] = t;
			}
	int n = 0;
	for (int i = 5; i >= 0; i--) {
		const int code = layerid[i];
		if (code >= 4 && !(code == 4 && (gx.k53250 || gx.sub1_roz)))   // (the sub layers: the sub 1's LVC or ROZ)
			continue;
		s_pool[n].order = (uint32_t)layerpri[i] << 24;
		s_pool[n].offs = -1;
		s_pool[n].code = code;
		s_pool[n].color = 0;
		n++;
	}
	const uint16_t *spr = s_snap->spriteram;
	for (int x = 0; x < 256; x++) {
		const int offs = x * 8;
		if (!(spr[offs] & 0x8000))
			continue;
		uint8_t zcode = (uint8_t)(spr[offs] & 0xff);
		if (s_opset & 0x10)   // (OPSET PRI: the z order inverted)
			zcode = (uint8_t)(0xff - zcode);
		int code = spr[offs + 1], color = spr[offs + 6], pri = 0;
		const int k = color;
		sprite_callback(&code, &color, &pri);
		int shadow_draw_mode = 0, add_shadow = 0, solid_draw_mode = 0, add_solid = 0, spri = 0, shadow = 0;
		if ((uint32_t)color & K055555_FULLSHADOW) {
			shadow = 3;
			spri = pri;
			add_shadow = 1;
			shadow_draw_mode = 5;
		} else {
			shadow = k >> 10 & 3;
			if (shadow) {
				const int objset1 = s_snap->kx46_regs[5];
				if (shadow != 1 || (objset1 & 0x20)) {
					shadow--;
					add_solid = 1;
					solid_draw_mode = 1;   // (partial solid)
					if (shadowon[shadow]) {
						add_shadow = 1;
						shadow_draw_mode = 4;   // (partial shadow)
					}
				} else {   // (the whole sprite to shadow: shadow code 1 and SD0EN off)
					shadow = 0;
					if (!shadowon[0])
						continue;
					add_shadow = 1;
					shadow_draw_mode = 5;
				}
			} else {
				add_solid = 1;
				solid_draw_mode = 0;
			}
			if (add_solid && ((color >> K055555_MIXSHIFT) & 3))   // (tag for alpha blending)
				solid_draw_mode |= 2;
			if (add_shadow)
				spri = (s_opset & 0x20) ? pri : shdpri[shadow];
		}
		switch (s_primode & 0xf) {
		case 1: zcode = 0; break;   // (Dadandarn zcode suppression)
		case 4:   // (Daisukiss bad shadow filter)
			if ((k & 0x3000) || k == 0x0800)
				continue;
			/* fallthrough */
		case 5:   // (Tokkae shadow masking)
			if (spri < spri_min)
				spri = spri_min;
			break;
		}
		if (add_solid) {
			s_pool[n].order = (uint32_t)pri << 24 | (uint32_t)zcode << 16 | (uint32_t)offs << (8 - 3) | (uint32_t)solid_draw_mode << 4;
			s_pool[n].offs = offs;
			s_pool[n].code = code;
			s_pool[n].color = color;
			n++;
		}
		if (add_shadow && !((uint32_t)color & K055555_SKIPSHADOW) && !(mixerflags & GXMIX_NOSHADOW)) {
			s_pool[n].order = (uint32_t)spri << 24 | (uint32_t)zcode << 16 | (uint32_t)offs << (8 - 3) |
			                  (uint32_t)shadow_draw_mode << 4 | (uint32_t)shadow;
			s_pool[n].offs = offs;
			s_pool[n].code = code;
			s_pool[n].color = color;
			n++;
		}
	}
	// reverse, then a stable sort in descending order (ties keep the reversed order)
	for (int i = 0; i < n / 2; i++) {
		const gx_obj_t t = s_pool[i];
		s_pool[i] = s_pool[n - 1 - i];
		s_pool[n - 1 - i] = t;
	}
	for (int i = 1; i < n; i++) {   // (insertion sort: stable)
		const gx_obj_t t = s_pool[i];
		int j = i - 1;
		while (j >= 0 && s_pool[j].order < t.order) {
			s_pool[j + 1] = s_pool[j];
			j--;
		}
		s_pool[j + 1] = t;
	}
	return n;
}

// ---- the mixer's frame in bands (gx.bands: the render thread, the emulation thread waiting for it and the helper
// core each take bands, each worker with its own copy of the drawing's state; the frame, the z-buffers and the pool
// shared). A band starts from the frame's start state; the alpha mix code is the one of the band last in the drawing's
// order, as one drawing of the whole frame leaves it ----
#define GX_BANDS_MAX 8
typedef struct {
	uint32_t *pens;
	int *cur_bright, *shadow_mode, *mix, *by0, *by1;
	uint32_t *mix_key, *draw_seq;
	void (*band)(int mixerflags);
} gx_worker_t;
static const gx_worker_t s_workers[3] = {
	{ s_pens, &s_current_brightness, &s_shadow_mode, &s_last_alpha_tile_mix_code, &s_by0, &s_by1, &s_mix_key, &s_draw_seq,
	  mixer_band },
	{ s_pens_w1, &s_current_brightness_w1, &s_shadow_mode_w1, &s_last_alpha_tile_mix_code_w1, &s_by0_w1, &s_by1_w1,
	  &s_mix_key_w1, &s_draw_seq_w1, mixer_band_w1 },
	{ s_pens_w2, &s_current_brightness_w2, &s_shadow_mode_w2, &s_last_alpha_tile_mix_code_w2, &s_by0_w2, &s_by1_w2,
	  &s_mix_key_w2, &s_draw_seq_w2, mixer_band_w2 },
};
static struct {
	uint32_t pens[8192];   // (the frame's start: the contrast's pens and brightness, the shadow mode)
	int bright, shadow_mode;
	int mixerflags, nb, c565, rotate, pre;
	uint32_t id, wid[3];
	uint32_t key[GX_BANDS_MAX];
	int mix[GX_BANDS_MAX], bright_end[GX_BANDS_MAX], shadow_end[GX_BANDS_MAX];
} s_bj;
static volatile int s_band_next, s_band_left, s_band_active, s_band_helpers;

// the rows y0 to y1 of the frame to the frontend's frame (RGB 565 converted here) or the RGB 565 bitmap
static void out_rows(int y0, int y1)
{
	for (int y = y0 - VIS_Y0; y <= y1 - VIS_Y0; y++) {
		const uint32_t *srow = s_frame + y * VIS_W;
		if (!rbx_video.out) {
			uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
			for (int x = 0; x < VIS_W; x++)
				d[x] = (uint16_t)((srow[x] >> 8 & 0xf800) | (srow[x] >> 5 & 0x07e0) | (srow[x] >> 3 & 0x001f));
			continue;
		}
		uint8_t *o = (uint8_t *)rbx_video.out + (size_t)y * (size_t)rbx_video.out_pitch;
		if (s_bj.c565)
			for (int x = 0; x < VIS_W; x++)
				((uint16_t *)o)[x] = (uint16_t)((srow[x] >> 8 & 0xf800) | (srow[x] >> 5 & 0x07e0) | (srow[x] >> 3 & 0x001f));
		else if (rbx_video.out_bpp == 4)
			for (int x = 0; x < VIS_W; x++)
				((uint32_t *)o)[x] = rbx_video.out_color((int)(srow[x] >> 16) & 0xff, (int)(srow[x] >> 8) & 0xff, (int)srow[x] & 0xff);
		else
			for (int x = 0; x < VIS_W; x++)
				((uint16_t *)o)[x] = (uint16_t)rbx_video.out_color((int)(srow[x] >> 16) & 0xff, (int)(srow[x] >> 8) & 0xff, (int)srow[x] & 0xff);
	}
}

static uint32_t s_band_us[GX_BANDS_MAX];   // (each band's drawing time: help_worker_only splits only heavy frames)
static int s_band_heavy, s_band_force;   // (force: RBX_GXBANDS, the harness's check of the bands)

static void pre_band(int w, int b);

static void run_band(int w, int b)
{
	if (s_bj.pre) {
		pre_band(w, b);
		return;
	}
	const uint64_t t0 = gx.help_worker_only ? rbx_clock_us() : 0;
	const gx_worker_t *k = &s_workers[w];
	if (s_bj.wid[w] != s_bj.id || *k->cur_bright != s_bj.bright) {   // (the frame's start contrast)
		memcpy(k->pens, s_bj.pens, sizeof(s_bj.pens));
		*k->cur_bright = s_bj.bright;
		s_bj.wid[w] = s_bj.id;
	}
	*k->shadow_mode = s_bj.shadow_mode;
	*k->mix_key = 0;
	*k->draw_seq = 0;
	const int y0 = VIS_Y0 + VIS_H * b / s_bj.nb, y1 = VIS_Y0 + VIS_H * (b + 1) / s_bj.nb - 1;
	*k->by0 = y0;
	*k->by1 = y1;
	k->band(s_bj.mixerflags);
	s_bj.key[b] = *k->mix_key;
	s_bj.mix[b] = *k->mix;
	s_bj.bright_end[b] = *k->cur_bright;
	s_bj.shadow_end[b] = *k->shadow_mode;
	out_rows(y0, y1);
	if (gx.help_worker_only)
		s_band_us[b] = (uint32_t)(rbx_clock_us() - t0);
}

// a helper's bands (1: the emulation thread waiting for the drawing, 2: the frontend's helper core; 0x100: one band)
int gx_render_help(int helper)
{
	const int one = helper & 0x100;
	helper &= 0xff;
	if (helper < 1 || helper > 2 || (gx.help_worker_only && !(one && helper == 1)) ||
	    !__atomic_load_n(&s_band_active, __ATOMIC_SEQ_CST))
		return 0;
	__atomic_add_fetch(&s_band_helpers, 1, __ATOMIC_SEQ_CST);
	int did = 0;
	if (__atomic_load_n(&s_band_active, __ATOMIC_SEQ_CST))
		for (;;) {
			const int b = __atomic_fetch_add(&s_band_next, 1, __ATOMIC_ACQ_REL);
			if (b >= s_bj.nb)
				break;
			run_band(helper, b);
			__atomic_sub_fetch(&s_band_left, 1, __ATOMIC_ACQ_REL);
			did = 1;
			if (one)
				break;
		}
	__atomic_sub_fetch(&s_band_helpers, 1, __ATOMIC_SEQ_CST);
	return did;
}

// a pre-GX band: the recorded drawing over the rows of band b by worker w (its own tile cache, pens copy, clip)
static void pre_band(int w, int b)
{
	const int y0 = VIS_Y0 + VIS_H * b / s_bj.nb, y1 = VIS_Y0 + VIS_H * (b + 1) / s_bj.nb - 1;
	if (w && s_bj.wid[w] != s_bj.id) {   // (the frame's pens)
		memcpy(w == 1 ? s_pens_w1 : s_pens_w2, s_pens, sizeof(s_pens));
		s_bj.wid[w] = s_bj.id;
	}
	if (w == 0) { s_by0 = y0; s_by1 = y1; s_pby0 = y0; s_pby1 = y1; }
	else if (w == 1) { s_by0_w1 = y0; s_by1_w1 = y1; s_pby0_w1 = y0; s_pby1_w1 = y1; }
	else { s_by0_w2 = y0; s_by1_w2 = y1; s_pby0_w2 = y0; s_pby1_w2 = y1; }
	for (int i = 0; i < s_npops; i++) {
		const pre_op_t *op = &s_pops[i];
		switch (op->kind) {
		case PO_CALL:
			op->fn(op->layer, y0 - VIS_Y0, y1 - VIS_Y0);
			break;
		case PO_FILL: {   // (fill_solid_bg over the band's rows)
			if (s_frame16) {   // (the ind16 frame: the black pen)
				const uint16_t b = (uint16_t)(2 * s_pal_n);
				for (int j = (y0 - VIS_Y0) * VIS_W; j < (y1 - VIS_Y0 + 1) * VIS_W; j++)
					s_frame16[j] = b;
				memset(s_pri + (y0 - VIS_Y0) * VIS_W, 0, (size_t)((y1 - y0 + 1) * VIS_W));
				break;
			}
			const uint16_t *k338 = s_snap->k54338;
			const uint32_t bg = (uint32_t)(k338[K338_REG_BGC_R] & 0xff) << 16 | k338[K338_REG_BGC_GB];
			for (int j = (y0 - VIS_Y0) * VIS_W; j < (y1 - VIS_Y0 + 1) * VIS_W; j++)
				s_frame[j] = bg;
			memset(s_pri + (y0 - VIS_Y0) * VIS_W, 0, (size_t)((y1 - y0 + 1) * VIS_W));
			break;
		}
		case PO_TILEMAP:
			if (w == 0) {
				s_pri_code = op->pri;
				s_pre_opaque = op->opaque;
				tilemap_draw(&s_snap->k56832, op->layer, 0, op->all, op->cat, op->alpha);
				s_pre_opaque = 0;
			} else if (w == 1) {
				s_pri_code_w1 = op->pri;
				s_pre_opaque_w1 = op->opaque;
				tilemap_draw_w1(&s_snap->k56832, op->layer, 0, op->all, op->cat, op->alpha);
			} else {
				s_pri_code_w2 = op->pri;
				s_pre_opaque_w2 = op->opaque;
				tilemap_draw_w2(&s_snap->k56832, op->layer, 0, op->all, op->cat, op->alpha);
			}
			break;
		case PO_LVC:   // (k053250_device::draw, writing the band's rows)
			k053250_draw_rows(gx.k53250, &s_snap->lvc, s_frame, VIS_W, VIS_X0, VIS_Y0, VIS_X0 + VIS_W - 1, VIS_Y0 + VIS_H - 1,
			                  w == 0 ? s_pens : w == 1 ? s_pens_w1 : s_pens_w2, s_pal_n, op->layer, VIS_X0 + VIS_W - 1,
			                  VIS_Y0 + VIS_H - 1, s_pri, (uint8_t)op->pri, y0, y1);
			break;
		case PO_SPRITES:
			if (w == 0)
				pre_sprites_core();
			else if (w == 1)
				pre_sprites_core_w1();
			else
				pre_sprites_core_w2();
			break;
		}
	}
	if (w == 0) { s_by0 = 0; s_by1 = VIS_H_MAX + 512; s_pby0 = 0; s_pby1 = VIS_H_MAX + 512; }
}

// the recorded pre-GX drawing in gx.pre_bands bands: the render thread and the helpers (gx_render_help) take them
static void pre_bands_run(void)
{
	s_bj.pre = 1;
	s_bj.nb = gx.pre_bands < GX_BANDS_MAX ? gx.pre_bands : GX_BANDS_MAX;
	s_bj.id++;
	const int nb = s_bj.nb;
	__atomic_store_n(&s_band_left, nb, __ATOMIC_RELEASE);
	__atomic_store_n(&s_band_next, 0, __ATOMIC_RELEASE);
	__atomic_store_n(&s_band_active, 1, __ATOMIC_SEQ_CST);
	for (;;) {
		const int b = __atomic_fetch_add(&s_band_next, 1, __ATOMIC_ACQ_REL);
		if (b >= nb)
			break;
		pre_band(s_bj.rotate ? b % 3 : 0, b);
		__atomic_sub_fetch(&s_band_left, 1, __ATOMIC_ACQ_REL);
	}
	while (__atomic_load_n(&s_band_left, __ATOMIC_ACQUIRE) > 0) {
	}
	__atomic_store_n(&s_band_active, 0, __ATOMIC_SEQ_CST);
	while (__atomic_load_n(&s_band_helpers, __ATOMIC_SEQ_CST) > 0) {
	}
	s_bj.pre = 0;
}

// konamigx_mixer for the frame: the pool once, the bands, then the frame's end state as one drawing leaves it
static void mixer_frame(int mixerflags)
{
	s_job.n = mixer_setup(mixerflags);
	memcpy(s_bj.pens, s_pens, sizeof(s_bj.pens));
	s_bj.bright = s_current_brightness;
	s_bj.shadow_mode = s_shadow_mode;
	s_bj.mixerflags = mixerflags;
	s_bj.nb = gx.bands > 1 && !gx.k53250 ? (gx.bands < GX_BANDS_MAX ? gx.bands : GX_BANDS_MAX) : 1;
	// (help_worker_only: the bands while the last frame's drawing took 11 ms or more, back to one under 9 ms. The
	// worker drawing beside a light frame only slowed the emulation (the memory shared): tbyahhoo 15.4 -> 16.1 ms)
	if (gx.help_worker_only && !s_band_heavy && !s_band_force)
		s_bj.nb = 1;
	s_bj.c565 = rbx_video.out && rbx_video.out_bpp == 2 && rbx_video.out_color(0xff, 0, 0) == 0xf800 &&
	            rbx_video.out_color(0, 0xff, 0) == 0x07e0 && rbx_video.out_color(0, 0, 0xff) == 0x001f &&
	            rbx_video.out_color(0x84, 0x82, 0x21) == 0x8404;
	s_bj.id++;
	const int nb = s_bj.nb;
	if (nb == 1)
		run_band(0, 0);
	else {
		__atomic_store_n(&s_band_left, nb, __ATOMIC_RELEASE);
		__atomic_store_n(&s_band_next, 0, __ATOMIC_RELEASE);
		__atomic_store_n(&s_band_active, 1, __ATOMIC_SEQ_CST);
		for (;;) {
			const int b = __atomic_fetch_add(&s_band_next, 1, __ATOMIC_ACQ_REL);
			if (b >= nb)
				break;
			run_band(s_bj.rotate ? b % 3 : 0, b);   // (rotate: the harness checks the copies give the same frame)
			__atomic_sub_fetch(&s_band_left, 1, __ATOMIC_ACQ_REL);
		}
		while (__atomic_load_n(&s_band_left, __ATOMIC_ACQUIRE) > 0) {
		}
		__atomic_store_n(&s_band_active, 0, __ATOMIC_SEQ_CST);
		while (__atomic_load_n(&s_band_helpers, __ATOMIC_SEQ_CST) > 0) {
		}
	}
	if (gx.help_worker_only) {
		uint32_t us = 0;
		for (int b = 0; b < nb; b++)
			us += s_band_us[b];
		s_band_heavy = us >= 11000 ? 1 : us < 9000 ? 0 : s_band_heavy;
	}
	int best = -1;
	for (int b = 0; b < nb; b++)
		if (s_bj.key[b] && (best < 0 || s_bj.key[b] >= s_bj.key[best]))
			best = b;
	if (best >= 0)
		s_last_alpha_tile_mix_code = s_bj.mix[best];
	if (s_current_brightness != s_bj.bright_end[nb - 1])
		set_brightness_value(s_bj.bright_end[nb - 1]);
	s_shadow_mode = s_bj.shadow_end[nb - 1];
	s_by0 = 0;   // (the copy of the render thread: the whole frame for the other drawings)
	s_by1 = VIS_H_MAX + 512;
	if (rbx_video.out)
		rbx_video.out_done = 1;
}

// ---- init / capture / render ----
int gx_video_init(void)
{
	s_pal_n = gx.pal_entries ? gx.pal_entries : 8192;
	if (decode_tiles(&gx.k56832) || decode_sprites(&gx.k55673))
		return -1;
	s_snap = (snap_t *)calloc(1, sizeof(snap_t));
	s_frame = (uint32_t *)calloc((size_t)VIS_W_MAX * VIS_H_MAX, sizeof(uint32_t));
	if (!s_snap || !s_frame)
		return -1;
	if (gx.mixmode == GX_MIXMODE_PRE) {   // (the pre-GX boards: bitmap_ind16 (or rgb32) and the priority bitmap)
		if (!gx.pre_rgb && !(s_frame16 = (uint16_t *)calloc((size_t)VIS_W_MAX * VIS_H_MAX, sizeof(uint16_t))))
			return -1;
		if (!(s_pri = (uint8_t *)calloc((size_t)VIS_W_MAX * VIS_H_MAX, 1)))
			return -1;
	}
	// (common_init: the layer offsets; video start of the machine: the tile mode and the mixer's priority mode)
	k056832_set_layer_offs(&gx.k56832, 0, -2, 0);
	k056832_set_layer_offs(&gx.k56832, 1, 0, 0);
	k056832_set_layer_offs(&gx.k56832, 2, 2, 0);
	k056832_set_layer_offs(&gx.k56832, 3, 3, 0);
	s_tilemode = s_primode = 0;
	s_current_brightness = -1;
	s_last_alpha_tile_mix_code = 0;
	s_bj.rotate = getenv("RBX_GXBANDROT") != NULL;
	{
		const char *e = getenv("RBX_GXBANDS");   // (the harness: the bands without the board's setting)
		if (e)
			gx.bands = atoi(e);
		s_band_force = e != NULL;
		s_band_heavy = 0;
	}
	memset(s_shd_cfg, 0, sizeof(s_shd_cfg));
	memset(s_shd_rgb, 0, sizeof(s_shd_rgb));
	configure_rgb_shadows(0, 0.6f);
	configure_rgb_shadows(1, 1.0f / 0.6f);
	memcpy(s_shadow[2], s_shadow[0], sizeof(s_shadow[0]));
	memcpy(s_shadow[3], s_shadow[1], sizeof(s_shadow[1]));
	if (gx.mixmode != GX_MIXMODE_PRE)
		set_shadow_drgb32(3, -80, -80, -80, 0);   // (mixer_init)
	s_shadow_mode = 0;
	return 0;
}

void gx_video_set_area(int x0, int w)
{
	s_vis_x0 = x0;
	s_vis_w = w;
	s_vis_y0 = 16;
	s_vis_h = 224;
}

void gx_video_set_y0(int y0) { s_vis_y0 = y0; }
void gx_video_set_height(int h) { s_vis_h = h > VIS_H_MAX ? VIS_H_MAX : h; }

void gx_video_set_modes(int tilemode, int primode)
{
	s_tilemode = tilemode;
	s_primode = primode;
	if (gx.sprcb == GX_SPRCB_DRAGOONJ)   // (VIDEO_START dragoonj: the layer offsets + 1)
		for (int l = 0; l < 4; l++)
			k056832_set_layer_offs(&gx.k56832, l, (l == 0 ? -2 : l == 1 ? 0 : l == 2 ? 2 : 3) + 1, 0);
}

void gx_video_exit(void)
{
	free(s_tiles_raw);
	s_tiles_raw = NULL;
	free(s_sprites);
	free(s_snap);
	free(s_frame);
	free(s_frame16);
	free(s_pri);
	s_frame16 = NULL;
	s_pri = NULL;
	s_tiles = s_sprites = NULL;
	s_snap = NULL;
	s_frame = NULL;
	s_ntiles = s_nsprites = 0;
}

void gx_video_capture(void)
{
	rbx_video_wait_idle();
	snap_t *s = s_snap;
	if (gx.pal555)   // (xBGR_555 words: gijoe ...)
		for (int i = 0; i < s_pal_n; i++) {
			const uint16_t w = *(const uint16_t *)(gx.palram + i * 2);
			s->pal[i] = gx.pal555rgb ? (uint32_t)pal5bit(w >> 10) << 16 | (uint32_t)pal5bit(w >> 5) << 8 | (uint32_t)pal5bit(w)
			                         : (uint32_t)pal5bit(w) << 16 | (uint32_t)pal5bit(w >> 5) << 8 | (uint32_t)pal5bit(w >> 10);
		}
	else
		for (int i = 0; i < s_pal_n; i++) {   // (xRGB_888: the high word then the low word of host words)
			const uint16_t hi = *(const uint16_t *)(gx.palram + i * 4), lo = *(const uint16_t *)(gx.palram + i * 4 + 2);
			const uint32_t c = ((uint32_t)hi << 16 | lo) & 0xffffff;
			s->pal[i] = gx.pal_bgr ? (c & 0xff) << 16 | (c & 0xff00) | c >> 16 : c;   // (xBGR_888: bishi)
		}
	memcpy(s->spriteram, gx.k55673.ram, sizeof(s->spriteram));   // (objdma 0: the sprite RAM itself)
	memcpy(s->k55555, gx.k55555, sizeof(s->k55555));
	memcpy(s->k54338, gx.k54338, sizeof(s->k54338));
	memcpy(s->tilebanks, gx.tilebanks, sizeof(s->tilebanks));
	s->wrport1_0 = gx.wrport1_0;
	s->wrport2 = gx.wrport2;
	s->k56832 = gx.k56832;
	memcpy(s->kx47_regs, gx.k55673.kx47_regs, sizeof(s->kx47_regs));
	memcpy(s->kx46_regs, gx.k55673.kx46_regs, sizeof(s->kx46_regs));
	if (gx.k53250)
		k053250_snap(gx.k53250, &s->lvc);
	if (gx.pre_capture)
		gx.pre_capture();
	rbx_video_frame_done();
}

// the pre-GX frame: its pens (the palette, its shadows at x 0.6 as MAME's shadow group, black) to RGB
// (the pre-GX frame's pens: the palette, then its shadows; recomputed when the palette changed)
static uint32_t s_pre_pens[2 * 8192 + 1];
static uint16_t s_pre_pens565[2 * 8192 + 1];
static uint32_t s_pre_pal[8192];
static int s_pre_pens_n = -1;

// out16: the frontend's RGB 565 frame (the pens written there directly), else NULL (s_frame)
static float s_pre_dim[9];   // (the adjustment the pens were made with: the contrasts, the highlight mode)

// adjust_palette_entry: a component x the contrast (float), clamped
static inline uint32_t pre_adjust(uint32_t c, float k)
{
	int r = (int)((float)(c >> 16 & 0xff) * k), g = (int)((float)(c >> 8 & 0xff) * k), b = (int)((float)(c & 0xff) * k);
	r = r < 0 ? 0 : r > 255 ? 255 : r;
	g = g < 0 ? 0 : g > 255 ? 255 : g;
	b = b < 0 ? 0 : b > 255 ? 255 : b;
	return (uint32_t)(r << 16 | g << 8 | b);
}

static void pre_render(uint8_t *out16, int pitch)
{
	const int n = s_pal_n;
	s_tc_gen++;
	s_tc_side = 0;
	if (gx.pre_bands > 1) {   // (recorded, then drawn in bands)
		s_npops = 0;
		s_prec = 1;
		gx.pre_draw();
		s_prec = 0;
		pre_bands_run();
	} else
		gx.pre_draw();
	if (!s_frame16)   // (the RGB frame: drawn as it is)
		return;
	float dim[9] = { 0 };
	if (gx.pre_dim) {
		memcpy(dim, gx.pre_contrast, sizeof(gx.pre_contrast));
		dim[8] = (float)gx.pre_highlight + gx.pre_highlight_factor + 2.0f * gx.pre_shadow_factor;   // (the group's factors too)
	}
	if (gx.pre_dim && (s_pre_pens_n != n || memcmp(s_pre_pal, s_snap->pal, (size_t)n * sizeof(uint32_t)) ||
	                   memcmp(s_pre_dim, dim, sizeof(dim)))) {
		// (MAME's palette: the entry's contrast, the shadow group's 0.6 or the highlight group's 1 / 0.6 with it)
		memcpy(s_pre_pal, s_snap->pal, (size_t)n * sizeof(uint32_t));
		memcpy(s_pre_dim, dim, sizeof(dim));
		s_pre_pens_n = n;
		const float group = gx.pre_highlight ? (gx.pre_highlight_factor > 0 ? gx.pre_highlight_factor : (float)(1.0 / 0.6))
		                  : gx.pre_shadow_factor > 0 ? gx.pre_shadow_factor : 0.6f;
		for (int i = 0; i < n; i++) {
			const float k = 1.0f * dim[(i >> 8) & 7];
			s_pre_pens[i] = pre_adjust(s_pre_pal[i], k);
			s_pre_pens[n + i] = pre_adjust(s_pre_pal[i], k * group);
		}
		s_pre_pens[2 * n] = 0;
		for (int i = 0; i <= 2 * n; i++) {
			const uint32_t c = s_pre_pens[i];
			s_pre_pens565[i] = (uint16_t)((c >> 8 & 0xf800) | (c >> 5 & 0x07e0) | (c >> 3 & 0x001f));
		}
	} else if (!gx.pre_dim && (s_pre_pens_n != n || memcmp(s_pre_pal, s_snap->pal, (size_t)n * sizeof(uint32_t)) ||
	                           memcmp(s_pre_dim, dim, sizeof(dim)))) {
		memcpy(s_pre_dim, dim, sizeof(dim));
		static uint8_t c06[256];   // (adjust_palette_entry: component x contrast 0.6)
		if (!c06[255])
			for (int v = 0; v < 256; v++)
				c06[v] = (uint8_t)(int)((float)v * 0.6f);
		memcpy(s_pre_pal, s_snap->pal, (size_t)n * sizeof(uint32_t));
		s_pre_pens_n = n;
		for (int i = 0; i < n; i++) {
			const uint32_t c = s_pre_pal[i];
			const uint32_t sh = gx.pre_shadow_red ? (uint32_t)(c06[c >> 16 & 0xff] << 16) | (c & 0xffff)
			                                       : (uint32_t)(c06[c >> 16 & 0xff] << 16 | c06[c >> 8 & 0xff] << 8 | c06[c & 0xff]);
			s_pre_pens[i] = c;
			s_pre_pens[n + i] = sh;
		}
		s_pre_pens[2 * n] = 0;
		for (int i = 0; i <= 2 * n; i++) {
			const uint32_t c = s_pre_pens[i];
			s_pre_pens565[i] = (uint16_t)((c >> 8 & 0xf800) | (c >> 5 & 0x07e0) | (c >> 3 & 0x001f));
		}
	}
	if (out16) {
		for (int y = 0; y < VIS_H; y++) {
			uint16_t *o = (uint16_t *)(out16 + (size_t)y * (size_t)pitch);
			const uint16_t *srow = s_frame16 + y * VIS_W;
			for (int x = 0; x < VIS_W; x++)
				o[x] = s_pre_pens565[srow[x]];
		}
		return;
	}
	for (int i = 0; i < VIS_W * VIS_H; i++)
		s_frame[i] = s_pre_pens[s_frame16[i]];
}

void gx_render(void)
{
	// (the pens at this frame's contrast: recomputed when the palette or the contrast changed)
	static uint32_t pal_prev[8192];
	static int pens_brightness = -2;
	const int nb = s_current_brightness < 0 ? 0xff : s_current_brightness;
	const int c565 = rbx_video.out && rbx_video.out_bpp == 2 && rbx_video.out_color(0xff, 0, 0) == 0xf800 &&
	                 rbx_video.out_color(0, 0xff, 0) == 0x07e0 && rbx_video.out_color(0, 0, 0xff) == 0x001f &&
	                 rbx_video.out_color(0x84, 0x82, 0x21) == 0x8404;
	if (gx.mixmode == GX_MIXMODE_PRE && s_frame16) {   // (its own pens; straight to an RGB 565 output)
		if (c565 || !rbx_video.out) {
			if (rbx_video.out) {
				pre_render((uint8_t *)rbx_video.out, rbx_video.out_pitch);
				rbx_video.out_done = 1;
			} else
				pre_render((uint8_t *)rbx_video.bitmap, rbx_video.pitch * (int)sizeof(uint16_t));
			return;
		}
		pre_render(NULL, 0);
	} else if (pens_brightness != nb || memcmp(pal_prev, s_snap->pal, sizeof(pal_prev))) {
		memcpy(pal_prev, s_snap->pal, sizeof(pal_prev));
		s_current_brightness = -1;
		set_brightness_value(nb);
	}
	s_tc_gen++;
	s_tc_side = gx.tilecb == GX_TILECB_ALPHA || gx.tilecb == GX_TILECB_SALMNDR2 || gx.tilecb == GX_TILECB_MW ||
	            gx.tilecb == GX_TILECB_VIOSTORM;
	if (gx.mixmode == GX_MIXMODE_PRE) {
		if (!s_frame16)   // (the RGB frame)
			pre_render(NULL, 0);
	} else if (gx.mixmode == GX_MIXMODE_LAYERS)
		mixer_layers();
	else {   // (the mixer: the rows converted to the output by the bands)
		mixer_frame(gx.mixmode == GX_MIXMODE_MARTCHMP   // (screen_update_martchmp: "not quite right")
		                ? (s_snap->k55555[K55_OINPRI_ON] == 0xef && s_snap->k54338[K338_REG_PBLEND] ? (1 << 16 | GXMIX_BLEND_FORCE) << 2 : 0)
		                : s_last_alpha_tile_mix_code << 30);
		pens_brightness = s_current_brightness;
		return;
	}
	pens_brightness = s_current_brightness;
	// out: the frontend's frame in its format (RGB 565 converted here), else the RGB 565 bitmap (the core's conversion
	// and the harness' hashes)
	if (rbx_video.out) {
		for (int y = 0; y < VIS_H; y++) {
			uint8_t *o = (uint8_t *)rbx_video.out + (size_t)y * (size_t)rbx_video.out_pitch;
			const uint32_t *srow = s_frame + y * VIS_W;
			if (c565)
				for (int x = 0; x < VIS_W; x++)
					((uint16_t *)o)[x] = (uint16_t)((srow[x] >> 8 & 0xf800) | (srow[x] >> 5 & 0x07e0) | (srow[x] >> 3 & 0x001f));
			else if (rbx_video.out_bpp == 4)
				for (int x = 0; x < VIS_W; x++)
					((uint32_t *)o)[x] = rbx_video.out_color((int)(srow[x] >> 16) & 0xff, (int)(srow[x] >> 8) & 0xff, (int)srow[x] & 0xff);
			else
				for (int x = 0; x < VIS_W; x++)
					((uint16_t *)o)[x] = (uint16_t)rbx_video.out_color((int)(srow[x] >> 16) & 0xff, (int)(srow[x] >> 8) & 0xff, (int)srow[x] & 0xff);
		}
		rbx_video.out_done = 1;
		return;
	}
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
		const uint32_t *srow = s_frame + y * VIS_W;
		for (int x = 0; x < VIS_W; x++)
			d[x] = (uint16_t)((srow[x] >> 8 & 0xf800) | (srow[x] >> 5 & 0x07e0) | (srow[x] >> 3 & 0x001f));
	}
}
