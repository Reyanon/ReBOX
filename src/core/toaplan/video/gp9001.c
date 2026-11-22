/*  Toaplan GP9001 VDP. Ported from MAME src/mame/toaplan/gp9001.cpp (BSD-3-Clause, copyright-holders: Quench, David
    Haywood).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "gp9001.h"

#include <stdlib.h>
#include <string.h>

#define PRIMASK_TMAPS 0x0e

// layout_16x16 / layout (RGN_FRAC(1,2), planes { half + 8, half, 8, 0 }): 8x8 tiles of 16 bytes a half, decoded to
// 4 bpp packed rows
int gp9001_init(gp9001_t *v, uint8_t *region, uint32_t size)
{
	memset(v, 0, sizeof(*v));
	const uint32_t half = size / 2, tiles = half / 16;
	uint8_t *out = (uint8_t *)malloc(tiles * 32 ? tiles * 32 : 1);
	if (!out)
		return -1;
	for (uint32_t t = 0; t < tiles; t++)
		for (int r = 0; r < 8; r++) {
			const uint8_t b0 = region[t * 16 + (uint32_t)r * 2], b1 = region[t * 16 + (uint32_t)r * 2 + 1];
			const uint8_t c0 = region[half + t * 16 + (uint32_t)r * 2], c1 = region[half + t * 16 + (uint32_t)r * 2 + 1];
			for (int x = 0; x < 8; x += 2) {
				uint8_t px[2];
				for (int k = 0; k < 2; k++) {
					const int bit = 7 - (x + k);
					px[k] = (uint8_t)(((c1 >> bit) & 1) << 3 | ((c0 >> bit) & 1) << 2 | ((b1 >> bit) & 1) << 1 | ((b0 >> bit) & 1));
				}
				out[t * 32 + (uint32_t)r * 4 + (uint32_t)(x >> 1)] = (uint8_t)(px[0] | px[1] << 4);
			}
		}
	memcpy(region, out, tiles * 32);   // (the region keeps the decoded tiles: half its size)
	free(out);
	v->gfx = region;
	v->tiles = tiles;
	// default layer offsets used by all original games
	static const int tx[3][2] = { { -0x1d6, -0x229 }, { -0x1d8, -0x227 }, { -0x1da, -0x225 } };
	for (int l = 0; l < 3; l++) {
		v->tm_xoff[l][0] = tx[l][0];
		v->tm_xoff[l][1] = tx[l][1];
		v->tm_yoff[l][0] = -0x1ef;
		v->tm_yoff[l][1] = -0x210;
	}
	v->sp_xoff[0] = -0x1cc;
	v->sp_xoff[1] = -0x17b;
	v->sp_yoff[0] = -0x1ef;
	v->sp_yoff[1] = -0x108;
	return 0;
}

static void sp_scroll(uint16_t *reg, uint16_t *flip, int bit, uint16_t data, uint16_t mask, int f, const int *off)
{
	data = (uint16_t)(data + off[f ? 1 : 0]);
	*reg = (uint16_t)((data & mask) | (*reg & ~mask));
	if (*reg & 0x8000)
		*reg |= 0xfe00;
	else
		*reg &= 0x1ff;
	if (f)
		*flip |= (uint16_t)bit;
	else
		*flip &= (uint16_t)~bit;
}

static void scroll_data_w(gp9001_t *v, uint16_t data, uint16_t mask)
{
	const int f = (v->scroll_reg & 0x80) != 0;
	const int r = v->scroll_reg & 0x7f;
	if (r < 6) {
		const int l = r >> 1, y = r & 1;
		uint16_t *reg = y ? &v->tm_scrolly[l] : &v->tm_scrollx[l];
		*reg = (uint16_t)((data & mask) | (*reg & ~mask));
		if (f)
			v->tm_flip[l] |= (uint16_t)(y ? 2 : 1);
		else
			v->tm_flip[l] &= (uint16_t)~(y ? 2 : 1);
	} else if (r == 6)
		sp_scroll(&v->sp_scrollx, &v->sp_flip, 1, data, mask, f, v->sp_xoff);
	else if (r == 7)
		sp_scroll(&v->sp_scrolly, &v->sp_flip, 2, data, mask, f, v->sp_yoff);
	else if (r == 0x0e || r == 0x0f)
		v->vint = 0;
}

void gp9001_reset(gp9001_t *v)
{
	v->voffs = v->scroll_reg = 0;
	for (int l = 0; l < 3; l++)
		v->tm_scrollx[l] = v->tm_scrolly[l] = v->tm_flip[l] = 0;
	v->sp_scrollx = v->sp_scrolly = v->sp_flip = 0;
	// init_scroll_regs: 0 written to the eight registers, flips off
	for (int r = 0; r < 8; r++) {
		v->scroll_reg = (uint16_t)r;
		scroll_data_w(v, 0, 0xffff);
	}
	v->scroll_reg = 0;
	v->vint = 0;
}

static uint16_t *space_word(gp9001_t *v, uint16_t offs)
{
	const uint32_t a = (uint32_t)(offs * 2) & 0x3fff;
	if (a < 0x3000)
		return &v->vram[a >> 12][(a & 0xfff) >> 1];
	return &v->spr[(a & 0x7ff) >> 1];   // (0x3000 .. 0x37ff mirrored to 0x3fff)
}

uint16_t gp9001_read(gp9001_t *v, int offset, int vpos)
{
	switch (offset & 6) {
	case 2: {
		const uint16_t o = v->voffs++;
		return *space_word(v, o);
	}
	case 6:
		return ((vpos + 15) % 262) >= 245;
	default:
		return 0xffff;
	}
}

void gp9001_write(gp9001_t *v, int offset, uint16_t data, uint16_t mask)
{
	switch (offset & 6) {
	case 0:
		v->voffs = (uint16_t)((data & mask) | (v->voffs & ~mask));
		break;
	case 2: {
		uint16_t *w = space_word(v, v->voffs++);
		*w = (uint16_t)((data & mask) | (*w & ~mask));
		break;
	}
	case 4:
		if (mask & 0xff)
			v->scroll_reg = data & 0x8f;
		break;
	case 6:
		scroll_data_w(v, data, mask);
		break;
	}
}

void gp9001_bootleg_scroll_w(gp9001_t *v, int offset, uint16_t data, uint16_t mask, const int *offs)
{
	if (mask != 0xffff || offset > 7)
		return;
	if (offset == 0 || offset == 2 || offset == 4)
		data = (uint16_t)(data - offs[offset]);
	else
		data = (uint16_t)(data + offs[offset]);
	v->scroll_reg = (uint16_t)offset;
	scroll_data_w(v, data, mask);
}

uint16_t gp9001_vdpcount(int vpos, int hpos)
{
	uint16_t st = 0xff00;
	const int hs = hpos > 325 && hpos < 380, vs = vpos >= 232 && vpos <= 245;
	const int v2 = (vpos + 15) % 262;
	if (hs)
		st &= ~0x8000;
	if (vs)
		st &= ~0x4000;
	if (hs || vs)
		st &= ~0x0100;
	st |= v2 < 256 ? (uint16_t)(v2 & 0xff) : 0xff;
	return st;
}

void gp9001_eof(gp9001_t *v)
{
	if (!v->no_sprite_buffer)
		memcpy(v->spr_buf, v->spr, sizeof(v->spr));
}

static inline uint8_t tile_pix(const gp9001_t *v, uint32_t t, int x, int y)
{
	const uint8_t b = v->gfx[t * 32 + (uint32_t)(y * 4 + (x >> 1))];
	return (x & 1) ? (b >> 4) : (b & 15);
}

static void draw_tilemap(const gp9001_t *v, int layer, uint16_t *bm, int pitch, uint8_t *pri, int w, int y0, int y1)
{
	const int fx = v->tm_flip[layer] & 1, fy = (v->tm_flip[layer] >> 1) & 1;
	const int scrollx = fx ? -(v->tm_scrollx[layer] + v->tm_xoff[layer][1]) : v->tm_scrollx[layer] + v->tm_xoff[layer][0];
	const int scrolly = fy ? -(v->tm_scrolly[layer] + v->tm_yoff[layer][1]) : v->tm_scrolly[layer] + v->tm_yoff[layer][0];
	const uint16_t *vram = v->vram[layer];
	const uint32_t elems = v->tiles / 4;
	if (!elems)
		return;
	for (int y = y0; y < y1; y++) {
		int ly = (y + scrolly) & 0x1ff;
		if (fy)
			ly = 511 - ly;
		uint16_t *dst = bm + y * pitch;
		uint8_t *dp = pri + (y - y0) * w;
		for (int x = 0; x < w;) {
			int lx = (x + scrollx) & 0x1ff;
			if (fx)
				lx = 511 - lx;
			int n = fx ? (lx & 15) + 1 : 16 - (lx & 15);
			if (n > w - x)
				n = w - x;
			const int idx = (ly >> 4) * 32 + (lx >> 4);
			const uint16_t attr = vram[idx * 2];
			uint32_t code = vram[idx * 2 + 1];
			if (v->tile_cb)
				code = v->tile_cb(v->cb_user, layer, code << 2) >> 2;
			code %= elems;
			const uint8_t pp = (uint8_t)((attr >> 8) & PRIMASK_TMAPS);
			const uint16_t cb = (uint16_t)((attr & 0x7f) << 4);
			const uint32_t t = code * 4 + (uint32_t)((ly & 8) ? 2 : 0);
			int px = lx & 15;
			const int step = fx ? -1 : 1;
			for (int i = 0; i < n; i++, px += step) {
				const uint8_t pen = tile_pix(v, t + (uint32_t)(px >> 3), px & 7, ly & 7);
				if (pen && pp >= dp[x + i]) {
					dst[x + i] = (uint16_t)(cb | pen);
					dp[x + i] = pp;
				}
			}
			x += n;
		}
	}
}

static void draw_sprites(const gp9001_t *v, uint16_t *bm, int pitch, uint8_t *pri, int w, int y0, int y1)
{
	int clk = 0;
	const int clk_max = 432 * 262;
	const uint16_t *src = v->no_sprite_buffer ? v->spr : v->spr_buf;
	const uint32_t total = v->tiles;
	if (!total)
		return;
	int old_x = (-(int16_t)v->sp_scrollx) & 0x1ff;
	int old_y = (-(int16_t)v->sp_scrolly) & 0x1ff;
	for (int offs = 0; offs < 0x400; offs += 4) {
		clk += 8;
		if (clk > clk_max)
			return;
		const uint16_t attrib = src[offs];
		const uint8_t priority = (uint8_t)((attrib >> 8) & 0xf);
		if (!(attrib & 0x8000))
			continue;
		uint32_t sprite = (uint32_t)(attrib & 3) << 16 | src[offs + 1];
		if (v->tile_cb)
			sprite = v->tile_cb(v->cb_user, 3, sprite);
		const uint32_t color = (attrib >> 2) & 0x3f;
		const int sizex = ((src[offs + 2] & 0x0f) + 1) * 8, sizey = ((src[offs + 3] & 0x0f) + 1) * 8;
		int sx_base, sy_base;
		if (!(attrib & 0x4000)) {
			sx_base = ((src[offs + 2] >> 7) - (int16_t)v->sp_scrollx) & 0x1ff;
			sy_base = ((src[offs + 3] >> 7) - (int16_t)v->sp_scrolly) & 0x1ff;
		} else {
			sx_base = (old_x + (src[offs + 2] >> 7)) & 0x1ff;
			sy_base = (old_y + (src[offs + 3] >> 7)) & 0x1ff;
		}
		old_x = sx_base;
		old_y = sy_base;
		int flipx = (attrib & 0x1000) != 0, flipy = (attrib & 0x2000) != 0;
		if (flipx) {
			sx_base -= 7;
			if (sx_base >= 0x1c0) sx_base -= 0x200;
		} else if (sx_base >= 0x180)
			sx_base -= 0x200;
		if (flipy) {
			sy_base -= 7;
			if (sy_base >= 0x1c0) sy_base -= 0x200;
		} else if (sy_base >= 0x180)
			sy_base -= 0x200;
		if (v->sp_flip & 1)
			sx_base = 320 - sx_base;
		if (v->sp_flip & 2)
			sy_base = 240 - sy_base;
		flipx ^= v->sp_flip & 1;
		flipy ^= (v->sp_flip >> 1) & 1;
		for (int dim_y = 0; dim_y < sizey; dim_y += 8) {
			const int sy = flipy ? sy_base - dim_y : sy_base + dim_y;
			for (int dim_x = 0; dim_x < sizex; dim_x += 8) {
				clk += 32;
				if (clk > clk_max)
					return;
				const int sx = flipx ? sx_base - dim_x : sx_base + dim_x;
				sprite %= total;
				if (sy + 8 > y0 && sy < y1 && sx + 8 > 0 && sx < w) {
					for (int yy = 0; yy < 8; yy++) {
						const int dy = sy + (flipy ? 7 - yy : yy);
						if (dy < y0 || dy >= y1)
							continue;
						uint16_t *dst = bm + dy * pitch;
						uint8_t *dp = pri + (dy - y0) * w;
						for (int xx = 0; xx < 8; xx++) {
							const int dx = sx + (flipx ? 7 - xx : xx);
							if (dx < 0 || dx >= w)
								continue;
							const uint8_t pix = tile_pix(v, sprite, xx, yy);
							if (pix && priority >= dp[dx]) {
								dst[dx] = (uint16_t)(color * 16 + pix);
								dp[dx] = priority;
							}
						}
					}
				}
				sprite++;
			}
		}
	}
}

void gp9001_draw(const gp9001_t *v, uint16_t *bm, int pitch, uint8_t *pri, int w, int y0, int y1, int clear)
{
	memset(pri, 0, (size_t)w * (size_t)(y1 - y0));
	if (clear)
		for (int y = y0; y < y1; y++)
			memset(bm + y * pitch, 0, sizeof(uint16_t) * (size_t)w);
	for (int l = 0; l < 3; l++)
		draw_tilemap(v, l, bm, pitch, pri, w, y0, y1);
	draw_sprites(v, bm, pitch, pri, w, y0, y1);
}
