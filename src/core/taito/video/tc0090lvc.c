/*  Taito TC0090LVC (tc0090lvc.h), ported from MAME src/devices/machine/tc009xlvc.cpp
    (LGPL-2.1+, copyright-holders: Angelo Salese).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "tc0090lvc.h"

#include <stdlib.h>
#include <string.h>

void tc0090lvc_init(tc0090lvc_t *t, const uint8_t *rom, uint32_t rom_size)
{
	memset(t, 0, sizeof(*t));
	t->rom = rom;
	t->rom_mask = rom_size - 1;
	memset(t->chr_dirty, 0xff, sizeof(t->chr_dirty));
}

void tc0090lvc_reset(tc0090lvc_t *t)
{
	// device_start clears vram / bitmap ram; device_reset the registers
	memset(t->vram, 0, sizeof(t->vram));
	memset(t->bitmap_ram, 0, sizeof(t->bitmap_ram));
	memset(t->pal, 0, sizeof(t->pal));
	memset(t->vregs, 0, sizeof(t->vregs));
	memset(t->sprram_buffer, 0, sizeof(t->sprram_buffer));
	memset(t->irq_vector, 0, sizeof(t->irq_vector));
	t->irq_enable = 0;
	for (int i = 0; i < 4; i++)
		t->ram_bank[i] = 0x80;
	t->rom_bank = 0;
	t->tile_or = 0;
	memset(t->chr_dirty, 0xff, sizeof(t->chr_dirty));
}

// ---- vram space (20 bits): 0x10000-0x1ffff vram, 0x40000-0x5ffff bitmap, 0x80000-0x801ff palette (mirror 0xe00)
static inline uint8_t vspace_r(const tc0090lvc_t *t, uint32_t a)
{
	if (a >= 0x10000 && a < 0x20000) return t->vram[a - 0x10000];
	if (a >= 0x40000 && a < 0x60000) return t->bitmap_ram[a - 0x40000];
	if (a >= 0x80000 && a < 0x81000) return t->pal[a & 0x1ff];
	return 0;
}

static inline void vspace_w(tc0090lvc_t *t, uint32_t a, uint8_t v)
{
	if (a >= 0x14000 && a < 0x20000) {
		uint32_t o = a - 0x10000;
		t->vram[o] = v;
		t->chr_dirty[o >> 8] |= (uint8_t)(1 << ((o >> 5) & 7));
	} else if (a >= 0x40000 && a < 0x60000) {
		t->bitmap_ram[a - 0x40000] = v;
	} else if (a >= 0x80000 && a < 0x81000) {
		t->pal[a & 0x1ff] = v;
	}
}

static inline uint32_t vram_addr(const tc0090lvc_t *t, uint16_t a)
{
	uint32_t o = (uint32_t)(a - 0xc000);
	return ((uint32_t)t->ram_bank[(o >> 12) & 3] << 12) | (o & 0xfff);
}

uint8_t tc0090lvc_read(tc0090lvc_t *t, uint16_t a)
{
	if (a < 0x6000)
		return t->rom[a & t->rom_mask];
	if (a < 0x8000)
		return t->rom[(((uint32_t)t->rom_bank << 13) | (a & 0x1fff)) & t->rom_mask];
	if (a < 0xfe00)
		return a >= 0xc000 ? vspace_r(t, vram_addr(t, a)) : 0;
	if (a < 0xff00)
		return t->vregs[a & 0xff];
	switch (a & 0x0f) {
	case 0: case 1: case 2: return t->irq_vector[a & 3];
	case 3: return t->irq_enable;
	case 4: case 5: case 6: case 7: return t->ram_bank[a & 3];
	case 8: return t->rom_bank;
	}
	return 0;
}

void tc0090lvc_write(tc0090lvc_t *t, uint16_t a, uint8_t v)
{
	if (a < 0xc000)
		return;
	if (a < 0xfe00) {
		vspace_w(t, vram_addr(t, a), v);
		return;
	}
	if (a < 0xff00) {
		t->vregs[a & 0xff] = v;
		return;
	}
	switch (a & 0x0f) {
	case 0: case 1: case 2: t->irq_vector[a & 3] = v; break;
	case 3: t->irq_enable = v; break;
	case 4: case 5: case 6: case 7: t->ram_bank[a & 3] = v; break;
	case 8: t->rom_bank = v; break;
	}
}

// ---- graphics decode: MAME layout_8x8 (planes 8,12,0,4; x 3..0,19..16; rows of 32 bits)
static uint8_t decode_tile(const uint8_t *src, uint8_t *dst)
{
	static const int planes[4] = { 8, 12, 0, 4 };
	static const int xo[8] = { 3, 2, 1, 0, 19, 18, 17, 16 };
	int nz = 0;
	for (int y = 0; y < 8; y++)
		for (int x = 0; x < 8; x++) {
			int p = 0;
			for (int pl = 0; pl < 4; pl++) {
				int b = planes[pl] + xo[x] + y * 32;
				p |= ((src[b >> 3] >> (7 - (b & 7))) & 1) << (3 - pl);
			}
			dst[y * 8 + x] = (uint8_t)p;
			nz += p != 0;
		}
	return (uint8_t)(nz == 0 ? 0 : nz == 64 ? 2 : 1);
}

int tc0090lvc_view_init(tc0090lvc_view *v, const uint8_t *gfx, uint32_t gfx_size)
{
	v->ntiles = gfx_size / 32;
	if (!v->ntiles)
		return -1;
	v->ntiles_mask = (v->ntiles & (v->ntiles - 1)) ? 0 : v->ntiles - 1;
	v->gfx = (uint8_t *)malloc((size_t)v->ntiles * 64);
	v->gfx_flags = (uint8_t *)malloc(v->ntiles);
	if (!v->gfx || !v->gfx_flags)
		return -1;
	for (uint32_t i = 0; i < v->ntiles; i++)
		v->gfx_flags[i] = decode_tile(gfx + i * 32, v->gfx + (size_t)i * 64);
	memset(v->chr, 0, sizeof(v->chr));
	memset(v->chr_flags, 0, sizeof(v->chr_flags));
	memset(v->dirty, 0xff, sizeof(v->dirty));
	return 0;
}

void tc0090lvc_view_exit(tc0090lvc_view *v)
{
	free(v->gfx);
	free(v->gfx_flags);
	v->gfx = v->gfx_flags = NULL;
}

void tc0090lvc_capture(tc0090lvc_t *t, tc0090lvc_view *v)
{
	memcpy(v->vram, t->vram, sizeof(v->vram));
	memcpy(v->pal, t->pal, sizeof(v->pal));
	memcpy(v->vregs, t->vregs, sizeof(v->vregs));
	memcpy(v->spr, t->sprram_buffer, sizeof(v->spr));
	if ((t->vregs[4] & 7) == 7)
		memcpy(v->bitmap_ram, t->bitmap_ram, sizeof(v->bitmap_ram));
	v->tile_or = t->tile_or;
	for (int i = 0; i < 256; i++) {
		v->dirty[i] |= t->chr_dirty[i];
		t->chr_dirty[i] = 0;
	}
	// screen_eof: sprites are buffered for the next frame
	memcpy(t->sprram_buffer, &t->vram[0xb000], 0x400);
}

// ---- drawing
static void update_chars(tc0090lvc_view *v)
{
	for (int i = 0; i < 256; i++) {
		if (!v->dirty[i])
			continue;
		for (int b = 0; b < 8; b++)
			if (v->dirty[i] & (1 << b)) {
				int n = i * 8 + b;
				v->chr_flags[n] = decode_tile(&v->vram[n * 32], &v->chr[n * 64]);
			}
		v->dirty[i] = 0;
	}
}

// one line of a 64x32 tilemap of 8x8 tiles; screen x maps to tilemap x = (xs + dir * x) & 511
static void draw_map_line(const tc0090lvc_view *v, uint16_t *dst, uint8_t *pri, int tmy, int xs, int dir,
                          int layer, int opaque, uint8_t pval)
{
	const uint8_t *map = &v->vram[layer == 2 ? 0xa000 : layer ? 0x9000 : 0x8000] + (tmy >> 3) * 128;
	int fy = (tmy & 7) * 8;
	int x = 0;
	while (x < LVC_SCREEN_W) {
		int tmx = (xs + dir * x) & 511;
		int px = tmx & 7;
		int n = dir > 0 ? 8 - px : px + 1;
		if (n > LVC_SCREEN_W - x)
			n = LVC_SCREEN_W - x;
		const uint8_t *e = map + (tmx >> 3) * 2;
		uint8_t attr = e[1];
		const uint8_t *src;
		uint8_t fl;
		if (layer == 2) {
			uint32_t code = e[0] | ((attr & 7) << 8);
			fl = v->chr_flags[code];
			src = &v->chr[code * 64];
		} else {
			uint32_t code = e[0] | ((attr & 3) << 8) | ((uint32_t)v->vregs[(attr >> 2) & 3] << 10);
			code |= v->tile_or;   // (no divide on the A9 in the common case)
			code = v->ntiles_mask ? code & v->ntiles_mask : code % v->ntiles;
			fl = v->gfx_flags[code];
			src = v->gfx + (size_t)code * 64;
		}
		uint16_t col = (uint16_t)((attr & 0xf0));
		src += fy + px;
		uint16_t *d = dst + x;
		uint8_t *pr = pri + x;
		if (opaque || fl == 2) {
			for (int i = 0; i < n; i++, src += dir) {
				d[i] = col | *src;
				pr[i] = pval;
			}
		} else if (fl) {
			for (int i = 0; i < n; i++, src += dir) {
				uint8_t p = *src;
				if (p) {
					d[i] = col | p;
					pr[i] = pval;
				}
			}
		}
		x += n;
	}
}

static void draw_sprites(tc0090lvc_view *v, uint16_t *bitmap, int pitch, int flip)
{
	uint32_t n16 = v->ntiles / 4;
	if (!n16)
		return;
	for (int count = 0; count < 0x3e7; count += 8) {
		const uint8_t *s = &v->spr[count];
		uint32_t code = s[0] | (s[1] << 8);
		int x = s[4] | ((s[5] & 1) << 8);
		if (x >= LVC_SCREEN_W - 1)
			x -= 512;
		int y = s[6];
		if (y >= LVC_VIS_Y1)
			y -= 256;
		int col = s[2] & 0x0f;
		int fx = s[3] & 1, fy = (s[3] >> 1) & 1;
		code = (((code << 2) | v->tile_or) >> 2) % n16;
		if (flip) {
			x = 304 - x;
			y = 240 - y;
			fx = !fx;
			fy = !fy;
		}
		if (x <= -16 || x >= LVC_SCREEN_W || y <= LVC_VIS_Y0 - 16 || y > LVC_VIS_Y1)
			continue;
		const uint8_t *t4 = v->gfx + (size_t)code * 256;
		const uint8_t *fl = v->gfx_flags + code * 4;
		if (!(fl[0] | fl[1] | fl[2] | fl[3]))
			continue;
		int hide = (col & 8) != 0;   // pmask 0xaa: behind pixels of priority 1 (bg0)
		uint16_t c = (uint16_t)(col << 4);
		for (int dy = 0; dy < 16; dy++) {
			int sy = y + dy;
			if (sy < LVC_VIS_Y0 || sy > LVC_VIS_Y1)
				continue;
			int ty = fy ? 15 - dy : dy;
			const uint8_t *row0 = t4 + ((ty >> 3) * 2) * 64 + (ty & 7) * 8;
			uint16_t *d = bitmap + (sy - LVC_VIS_Y0) * pitch;
			uint8_t *pr = v->pri + (sy - LVC_VIS_Y0) * LVC_SCREEN_W;
			for (int dx = 0; dx < 16; dx++) {
				int sx = x + dx;
				if (sx < 0 || sx >= LVC_SCREEN_W)
					continue;
				int tx = fx ? 15 - dx : dx;
				uint8_t p = row0[(tx >> 3) * 64 + (tx & 7)];
				if (!p)
					continue;
				if (!(hide && pr[sx] == 1))
					d[sx] = c | p;
				pr[sx] = 31;
			}
		}
	}
}

static inline uint8_t pal5(int v) { return (uint8_t)((v << 3) | (v >> 2)); }

void tc0090lvc_render(tc0090lvc_view *v, uint16_t *bitmap, int pitch, uint32_t *palette)
{
	for (int i = 0; i < 256; i++) {
		uint32_t raw = v->pal[i * 2] | (v->pal[i * 2 + 1] << 8);
		int r = (int)(((raw << 1) & 0x1e) | ((raw >> 12) & 1));
		int g = (int)(((raw >> 3) & 0x1e) | ((raw >> 13) & 1));
		int b = (int)(((raw >> 7) & 0x1e) | ((raw >> 14) & 1));
		palette[i] = (uint32_t)(pal5(r) << 16 | pal5(g) << 8 | pal5(b));
	}
	palette[LVC_BLACK] = 0;
	const int h = LVC_VIS_Y1 - LVC_VIS_Y0 + 1;
	uint8_t vr4 = v->vregs[4];
	int flip = (vr4 >> 4) & 1;
	if (!(vr4 & 0x20)) {   // screen disabled
		for (int y = 0; y < h; y++)
			for (int x = 0; x < LVC_SCREEN_W; x++)
				bitmap[y * pitch + x] = LVC_BLACK;
		return;
	}
	if ((vr4 & 7) == 7) {   // 8bpp bitmap
		for (int y = LVC_VIS_Y0; y <= LVC_VIS_Y1; y++) {
			int ry = flip ? 256 - y : y;
			const uint8_t *src = &v->bitmap_ram[(ry & 0xff) * 512];
			uint16_t *d = bitmap + (y - LVC_VIS_Y0) * pitch;
			for (int x = 0; x < LVC_SCREEN_W; x++)
				d[x] = src[(flip ? 320 - x : x) & 0x1ff];
		}
		return;
	}
	update_chars(v);
	const uint8_t *sc = &v->vram[0xb3f4];
	int xs[2], ys[2], dir = flip ? -1 : 1;
	static const int dxn[2] = { 28, 38 }, dxf[2] = { -11, -21 };
	for (int l = 0; l < 2; l++) {
		const uint8_t *r = sc + l * 8;
		int dx = r[0] | (r[1] << 8);
		if (flip)
			dx = ((dx & 0xfffc) | ((dx - 3) & 0x0003)) ^ 0xf;
		int dy = r[2];
		// MAME tilemap: scrollx = -dx with scrolldx (dxn / dxf), scrolly = -dy (effective_rowscroll / colscroll)
		xs[l] = flip ? 319 - dxf[l] - dx : -(dxn[l] + dx);
		ys[l] = dy;
	}
	int bg0pri = (vr4 & 0x08) ? 0 : 1;
	for (int y = LVC_VIS_Y0; y <= LVC_VIS_Y1; y++) {
		uint16_t *d = bitmap + (y - LVC_VIS_Y0) * pitch;
		uint8_t *pr = v->pri + (y - LVC_VIS_Y0) * LVC_SCREEN_W;
		int t1 = flip ? (255 - y - ys[1]) & 255 : (y - ys[1]) & 255;
		int t0 = flip ? (255 - y - ys[0]) & 255 : (y - ys[0]) & 255;
		draw_map_line(v, d, pr, t1, xs[1], dir, 1, 1, 0);
		draw_map_line(v, d, pr, t0, xs[0], dir, 0, 0, (uint8_t)bg0pri);
	}
	draw_sprites(v, bitmap, pitch, flip);
	// text layer: scrolldx -8 / -8 (flipped), no scroll
	int txs = flip ? 319 + 8 : 8;
	for (int y = LVC_VIS_Y0; y <= LVC_VIS_Y1; y++) {
		uint16_t *d = bitmap + (y - LVC_VIS_Y0) * pitch;
		int ty = flip ? 255 - y : y;
		draw_map_line(v, d, v->pri + (y - LVC_VIS_Y0) * LVC_SCREEN_W, ty, txs, dir, 2, 0, 0);
	}
}
