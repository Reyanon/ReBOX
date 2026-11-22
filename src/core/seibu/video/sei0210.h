/*  Seibu SEI0210 / SEI0211 sprite generator (with the SEI0220): the sprite list drawn with MAME's prio_transpen
    (transparent pen 15) into a band of lines; and the 16x16 / 8x8 tile line drawing the Seibu boards share.
    Ported from MAME 0.289 src/mame/seibu/sei021x_sei0220_spr.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail,
    hap) and src/emu/drawgfxt.ipp (BSD-3-Clause, copyright-holders: Nicola Salmoria, Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_SEI0210_H
#define RBX_SEI0210_H

#include <stdint.h>

typedef struct {
	int alt;                  // the Blood Bros. format
	int sei0211;              // SEI0211's coordinates (9 bits, >= 0x180 negative); else SEI0210's (bit 15 the sign)
	int xoff, yoff;
	uint32_t (*pri_cb)(int pri, int ext);   // the priority mask (MAME's pri_cb); NULL: 0
	const uint8_t *pix;       // 16x16 elements, a byte a pixel
	uint32_t count;
	uint16_t color_base;
	uint32_t (*bank_cb)(uint32_t code, int ext, int y15);   // MAME's gfxbank_cb (the 4th word's bit 15); NULL: none
} sei0210_t;

// one 16x16 element at (sx, sy) in screen coordinates, the lines [r0, r1) of the bitmap (screen line y0 + r); pri
// buffer (MAME's screen priority) of the same layout, pitch w
static inline void sei0210_elem(const uint8_t *src, uint16_t base, int flipx, int flipy, int sx, int sy, uint32_t pmask,
                                uint16_t *bm, int pitch, uint8_t *pri, int w, int y0, int r0, int r1)
{
	pmask |= 0x80000000u;
	for (int j = 0; j < 16; j++) {
		const int r = sy + j - y0;
		if (r < r0 || r >= r1)
			continue;
		const uint8_t *row = src + (flipy ? 15 - j : j) * 16;
		uint16_t *d = bm + r * pitch;
		uint8_t *p = pri + r * w;
		for (int i = 0; i < 16; i++) {
			const int px = sx + i;
			if ((unsigned)px >= (unsigned)w)
				continue;
			const uint8_t c = row[flipx ? 15 - i : i];
			if (c == 15)
				continue;
			if (!((1u << (p[px] & 0x1f)) & pmask))
				d[px] = (uint16_t)(base + c);
			p[px] = 31;
		}
	}
}

// the list (spriteram in host words, nwords of it) into the lines [r0, r1)
static inline void sei0210_draw(const sei0210_t *s, const uint16_t *spr, int nwords, uint16_t *bm, int pitch,
                                uint8_t *pri, int w, int y0, int r0, int r1)
{
	for (int i = 0; i + 3 < nwords; i += 4) {
		int flipx, flipy, pri_ = 0, ext = 0, sizex, sizey, x, y;
		uint32_t color, code;
		if (s->alt) {
			if (spr[i] & 0x8000)
				continue;
			flipy = (spr[i] >> 14) & 1;
			flipx = (spr[i] >> 13) & 1;
			pri_ = (spr[i] >> 11) & 1;
			sizex = ((spr[i] >> 7) & 7) + 1;
			sizey = ((spr[i] >> 4) & 7) + 1;
			color = spr[i] & 0xf;
			code = spr[i + 1] & 0x1fff;
			x = spr[i + 2] & 0x1ff;
			y = spr[i + 3] & 0x1ff;
			if (x >= 0x180) x -= 0x200;
			if (y >= 0x180) y -= 0x200;
		} else {
			if (!(spr[i] & 0x8000))
				continue;
			flipx = (spr[i] >> 14) & 1;
			flipy = (spr[i] >> 13) & 1;
			sizex = ((spr[i] >> 10) & 7) + 1;
			sizey = ((spr[i] >> 7) & 7) + 1;
			ext = (spr[i] >> 6) & 1;
			color = spr[i] & 0x3f;
			pri_ = (spr[i + 1] >> 14) & 3;
			code = spr[i + 1] & 0x3fff;
			if (s->sei0211) {
				x = spr[i + 2] & 0x1ff;
				y = spr[i + 3] & 0x1ff;
				if (x >= 0x180) x -= 0x200;
				if (y >= 0x180) y -= 0x200;
			} else {
				x = (spr[i + 2] & 0x1ff) - ((spr[i + 2] & 0x8000) ? 0x200 : 0);
				y = (spr[i + 3] & 0x1ff) - ((spr[i + 3] & 0x8000) ? 0x200 : 0);
			}
		}
		x += s->xoff;
		y += s->yoff;
		// (the whole sprite off the band: nothing)
		if (y + 16 * sizey - y0 <= r0 || y - y0 >= r1)
			continue;
		const uint32_t pmask = s->pri_cb ? s->pri_cb(pri_, ext) : 0;
		if (s->bank_cb)
			code = s->bank_cb(code, ext, (spr[i + 3] >> 15) & 1);
		const uint16_t base = (uint16_t)(s->color_base + color * 16);
		for (int ax = 0; ax < sizex; ax++)
			for (int ay = 0; ay < sizey; ay++) {
				const uint32_t c = code++;
				const uint8_t *src = s->pix + (size_t)(c < s->count ? c : s->count ? c % s->count : 0) * 256;
				sei0210_elem(src, base, flipx, flipy, flipx ? x + 16 * (sizex - ax - 1) : x + 16 * ax,
				             flipy ? y + 16 * (sizey - ay - 1) : y + 16 * ay, pmask, bm, pitch, pri, w, y0, r0, r1);
			}
	}
}

// a tilemap line of 16x16 tiles (cols x rows, MAME's TILEMAP_SCAN_ROWS or _COLS) from tile RAM words: tile data &
// 0xfff (| bank) the code, >> 12 the color; scrolled; trans: pen 15 transparent; the drawn pixels' priority |= pv
static inline void seibu_tile16_line(const uint16_t *ram, int cols, int rows, int scan_cols, const uint8_t *pix,
                                     uint32_t count, uint32_t bank, uint16_t cbase, int sx, int sy, int y, int trans,
                                     uint16_t *d, uint8_t *pri, uint8_t pv, int w)
{
	const int mw = cols * 16, mh = rows * 16;
	const int ty = ((y + sy) % mh + mh) % mh;
	int x = 0;
	while (x < w) {
		const int tx = ((x + sx) % mw + mw) % mw;
		const int c = tx >> 4, r = ty >> 4;
		const uint16_t data = ram[scan_cols ? c * rows + r : r * cols + c];
		const uint32_t code = (data & 0xfff) | bank;
		const uint8_t *row = pix + (size_t)(count ? code % count : 0) * 256 + (size_t)(ty & 15) * 16;
		const uint16_t base = (uint16_t)(cbase + (data >> 12) * 16);
		for (int fx = tx & 15; fx < 16 && x < w; fx++, x++) {
			const uint8_t v = row[fx];
			if (trans && v == 15)
				continue;
			d[x] = (uint16_t)(base + v);
			pri[x] |= pv;
		}
	}
}

// the same for 8x8 tiles
static inline void seibu_tile8_line(const uint16_t *ram, int cols, int rows, int scan_cols, const uint8_t *pix,
                                    uint32_t count, uint16_t cbase, int sx, int sy, int y, int trans, uint16_t *d,
                                    uint8_t *pri, uint8_t pv, int w)
{
	const int mw = cols * 8, mh = rows * 8;
	const int ty = ((y + sy) % mh + mh) % mh;
	int x = 0;
	while (x < w) {
		const int tx = ((x + sx) % mw + mw) % mw;
		const int c = tx >> 3, r = ty >> 3;
		const uint16_t data = ram[scan_cols ? c * rows + r : r * cols + c];
		const uint32_t code = data & 0xfff;
		const uint8_t *row = pix + (size_t)(count ? code % count : 0) * 64 + (size_t)(ty & 7) * 8;
		const uint16_t base = (uint16_t)(cbase + (data >> 12) * 16);
		for (int fx = tx & 7; fx < 8 && x < w; fx++, x++) {
			const uint8_t v = row[fx];
			if (trans && v == 15)
				continue;
			d[x] = (uint16_t)(base + v);
			pri[x] |= pv;
		}
	}
}

#endif
