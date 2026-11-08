/*  Data East tilemap chips 55 / 56 / 74 / 141, after MAME src/mame/dataeast/deco16ic.cpp
    (BSD-3-Clause, copyright-holders: Bryan McPhail; updates David Haywood).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "deco16ic.h"

#include <string.h>

void deco16ic_reset(deco16ic_t *d)
{
	for (int i = 0; i < 2; i++) {
		d->tm[i].bank = 0;
		d->tm[i].scrollx = d->tm[i].scrolly = 0;
		d->tm[i].control0 = d->tm[i].control1 = d->tm[i].bankval = 0;
	}
	memset(d->control, 0, sizeof(d->control));
}

void deco16ic_control_w(deco16ic_t *d, int offset, uint16_t data, uint16_t mem_mask)
{
	offset &= 7;
	data = d->control[offset] = (uint16_t)((d->control[offset] & ~mem_mask) | (data & mem_mask));
	switch (offset) {
	case 1: d->tm[0].scrollx = data; break;
	case 2: d->tm[0].scrolly = data; break;
	case 3: d->tm[1].scrollx = data; break;
	case 4: d->tm[1].scrolly = data; break;
	case 5:
		if (mem_mask & 0x00ff) d->tm[0].control0 = (uint8_t)data;
		if (mem_mask & 0xff00) d->tm[1].control0 = (uint8_t)(data >> 8);
		break;
	case 6:
		if (mem_mask & 0x00ff) d->tm[0].control1 = (uint8_t)data;
		if (mem_mask & 0xff00) d->tm[1].control1 = (uint8_t)(data >> 8);
		break;
	case 7:
		if (mem_mask & 0x00ff) d->tm[0].bankval = (uint8_t)data;
		if (mem_mask & 0xff00) d->tm[1].bankval = (uint8_t)(data >> 8);
		break;
	}
}

uint16_t deco16ic_control_r(const deco16ic_t *d, int offset)
{
	return d->control[offset & 7];
}

void deco16ic_update(deco16ic_t *d)
{
	for (int i = 0; i < 2; i++)
		if (d->tm[i].bank_cb)
			d->tm[i].bank = d->tm[i].bank_cb(d->tm[i].bankval);
}

void deco16ic_latch_line(deco16ic_t *d, int y)
{
	deco16ic_line_t *l = &d->lines[y & 255];
	for (int i = 0; i < 2; i++) {
		l->scrollx[i] = d->tm[i].scrollx;
		l->scrolly[i] = d->tm[i].scrolly;
		l->control0[i] = d->tm[i].control0;
		l->control1[i] = d->tm[i].control1;
		l->bank[i] = d->tm[i].bank_cb ? d->tm[i].bank_cb(d->tm[i].bankval) : d->tm[i].bank;
	}
	d->lines_on = 1;
}

// MAME scan_rows (16x16, up to 64x64 tiles): logical (col,row) -> memory offset
static inline uint32_t scan16(uint32_t col, uint32_t row)
{
	return (col & 0x1f) + ((row & 0x1f) << 5) + ((col & 0x20) << 5) + ((row & 0x20) << 6);
}

void deco16ic_draw(const deco16ic_t *d, int which, const uint16_t *rowscroll, uint16_t *bm, uint8_t *pri, int pitch,
                   int width, int top, int y0, int y1, int opaque, uint8_t priority, uint8_t pmask)
{
	const int wt = (d->tm[which].size & DECO_64x32) ? 64 : 32, ht = (d->tm[which].size & DECO_32x64) ? 64 : 32;
	const uint16_t *vram = d->vram[which];
	const int cmask = d->tm[which].col_mask, cbank = d->tm[which].col_bank;
	for (int y = y0; y < y1; y++) {
		// this line's registers (latched as the frame ran, or as they are now)
		int c0, c1, bank;
		uint16_t scrollx, scrolly;
		if (d->lines_on) {
			const deco16ic_line_t *l = &d->lines[(top + y) & 255];
			c0 = l->control0[which];
			c1 = l->control1[which];
			bank = l->bank[which];
			scrollx = l->scrollx[which];
			scrolly = l->scrolly[which];
		} else {
			c0 = d->tm[which].control0;
			c1 = d->tm[which].control1;
			bank = d->tm[which].bank;
			scrollx = d->tm[which].scrollx;
			scrolly = d->tm[which].scrolly;
		}
		if (!(c0 & 0x80))   // playfield disable
			continue;
		const int is8 = (c1 & 0x80) != 0;
		const rbx_gfx *g = is8 ? d->gfx8 : d->gfx16;
		if (!g || !g->count)
			continue;
		const int gran = (is8 ? d->gran8 : d->gran16) ? (is8 ? d->gran8 : d->gran16) : 4;
		const int base = is8 ? d->base8 : d->base16;
		const int ts = is8 ? 8 : 16, tsh = is8 ? 3 : 4;
		const int wmask = wt * ts - 1, hmask = ht * ts - 1;
		const int rows_on = rowscroll && (c1 & 0x40), cols_on = rowscroll && (c1 & 0x20);
		const int row_shift = (c0 >> 3) & 0xf;   // (row_type = 1 << it)
		const int col_shift = 3 + (c0 & 7);      // (col_type = 8 << (control0 & 7))
		const int flipx_bit = c1 & 1, flipy_bit = c1 & 2;
		const uint32_t count = g->count;
		const int src_y = (scrolly + top + y) & hmask;
		int src_x = scrollx;
		if (rows_on)
			src_x += rowscroll[src_y >> row_shift];
		const int sdx = d->scrolldx + d->dx[which][is8];
		if (sdx && !((c1 & 0x60) && !(rowscroll && ((c1 & 0x60) == 0x40 || (c1 & 0x60) == 0x20))))
			src_x -= sdx;
		src_x &= wmask;
		uint16_t *dst = bm + y * pitch;
		uint8_t *pp = pri ? pri + y * pitch : NULL;
		for (int x = 0; x < width;) {
			const int ty = cols_on ? (src_y + rowscroll[0x200 + ((src_x & 0x1ff) >> col_shift)]) & hmask : src_y;
			const int col = src_x >> tsh, row = ty >> tsh;
			const uint16_t t = vram[is8 ? (uint32_t)(row * wt + col) : scan16((uint32_t)col, (uint32_t)row)];
			uint32_t colour = (t >> 12) & 0xf, tile = t & 0xfff;
			int fx = 0, fy = 0;
			if (t & 0x8000) {
				if (flipx_bit) { fx = 1; colour &= 7; }
				if (flipy_bit) { fy = 1; colour &= 7; }
			}
			if (d->tile_cb)
				d->tile_cb(&tile, &colour, which, is8);
			uint32_t code = tile + (uint32_t)bank;
			if (code >= count)
				code %= count;
			const int pen_base = base + (int)(((colour & (uint32_t)cmask) + (uint32_t)cbank) << gran);
			const int py = ty & (ts - 1);
			const uint8_t *trow = g->pix + ((size_t)code * ts + (size_t)(fy ? ts - 1 - py : py)) * ts;
			// the pixels of this tile on this line: up to the tile's edge (column scroll: one column of the scroll
			// grid at most, its rows may differ)
			int run = ts - (src_x & (ts - 1));
			if (cols_on) {
				const int cw = (1 << col_shift) - (src_x & ((1 << col_shift) - 1));
				if (cw < run)
					run = cw;
			}
			if (run > width - x)
				run = width - x;
			// (the run's pixels: the tile row read forwards or backwards from its first one; the cases apart so each
			// loop is a plain one)
			const int px0 = src_x & (ts - 1);
			const uint8_t *sp = fx ? trow + (ts - 1 - px0) : trow + px0;
			const int step = fx ? -1 : 1;
			uint16_t *dp = dst + x;
			if (d->draw_pens && !opaque) {   // (a tilemap category: only these pens)
				const uint32_t dpm = d->draw_pens;
				for (int k = 0; k < run; k++, sp += step) {
					const uint8_t pen = *sp;
					if ((dpm >> pen) & 1) {
						dp[k] = (uint16_t)(pen_base + pen);
						if (pp)
							pp[x + k] = (uint8_t)((pp[x + k] & pmask) | priority);
					}
				}
			} else if (opaque) {
				for (int k = 0; k < run; k++, sp += step)
					dp[k] = (uint16_t)(pen_base + *sp);
				if (pp)
					for (int k = 0; k < run; k++)
						pp[x + k] = (uint8_t)((pp[x + k] & pmask) | priority);
			} else if (pp) {
				uint8_t *q = pp + x;
				for (int k = 0; k < run; k++, sp += step) {
					const uint8_t pen = *sp;
					if (pen) {
						dp[k] = (uint16_t)(pen_base + pen);
						q[k] = (uint8_t)((q[k] & pmask) | priority);
					}
				}
			} else
				for (int k = 0; k < run; k++, sp += step) {
					const uint8_t pen = *sp;
					if (pen)
						dp[k] = (uint16_t)(pen_base + pen);
				}
			x += run;
			src_x = (src_x + run) & wmask;
		}
	}
}

// one playfield's pixel at tilemap (x, y): its palette index (colour << gran | pen) and pen (MAME pixmap / flagsmap)
static inline uint16_t pf_pixel(const deco16ic_t *d, int which, int is8, int c1, int bank, int x, int y, int *pen)
{
	const rbx_gfx *g = is8 ? d->gfx8 : d->gfx16;
	const int ts = is8 ? 8 : 16, tsh = is8 ? 3 : 4;
	const int wt = (d->tm[which].size & DECO_64x32) ? 64 : 32;
	const int col = x >> tsh, row = y >> tsh;
	const uint16_t t = d->vram[which][is8 ? (uint32_t)(row * wt + col) : scan16((uint32_t)col, (uint32_t)row)];
	uint32_t colour = (t >> 12) & 0xf, tile = t & 0xfff;
	int fx = 0, fy = 0;
	if (t & 0x8000) {
		if (c1 & 1) { fx = 1; colour &= 7; }
		if (c1 & 2) { fy = 1; colour &= 7; }
	}
	if (d->tile_cb)
		d->tile_cb(&tile, &colour, which, is8);
	uint32_t code = tile + (uint32_t)bank;
	if (code >= g->count)
		code %= g->count;
	const int gran = (is8 ? d->gran8 : d->gran16) ? (is8 ? d->gran8 : d->gran16) : 4;
	const int px = x & (ts - 1), py = y & (ts - 1);
	const uint8_t p = g->pix[((size_t)code * ts + (size_t)(fy ? ts - 1 - py : py)) * ts + (size_t)(fx ? ts - 1 - px : px)];
	*pen = p;
	return (uint16_t)(d->base16 + ((((colour & (uint32_t)d->tm[which].col_mask) + (uint32_t)d->tm[which].col_bank) << gran) + p));
}

// pf_pixel's 16x16 tile at (x, y) once for its row: the row's pixels (row[fx ? 15 - px : px]) and the pen base
static inline const uint8_t *pf_row16(const deco16ic_t *d, int which, int c1, int bank, int x, int y, int *fx,
                                      uint16_t *base)
{
	const rbx_gfx *g = d->gfx16;
	const uint16_t t = d->vram[which][scan16((uint32_t)(x >> 4), (uint32_t)(y >> 4))];
	uint32_t colour = (t >> 12) & 0xf, tile = t & 0xfff;
	int ffx = 0, fy = 0;
	if (t & 0x8000) {
		if (c1 & 1) { ffx = 1; colour &= 7; }
		if (c1 & 2) { fy = 1; colour &= 7; }
	}
	if (d->tile_cb)
		d->tile_cb(&tile, &colour, which, 0);
	uint32_t code = tile + (uint32_t)bank;
	if (code >= g->count)
		code %= g->count;
	const int gran = d->gran16 ? d->gran16 : 4;
	const int py = y & 15;
	*fx = ffx;
	*base = (uint16_t)(d->base16 + (((colour & (uint32_t)d->tm[which].col_mask) + (uint32_t)d->tm[which].col_bank) << gran));
	return g->pix + ((size_t)code * 16 + (size_t)(fy ? 15 - py : py)) * 16;
}

void deco16ic_draw_combine(const deco16ic_t *d, const uint16_t *rowscroll, uint16_t *bm, uint8_t *pri, int pitch,
                           int width, int top, int y0, int y1, int opaque, uint8_t priority,
                           uint16_t (*mix)(uint16_t p, uint16_t p2))
{
	if (!d->gfx16 || !d->gfx16->count)
		return;
	const int wt = (d->tm[0].size & DECO_64x32) ? 64 : 32, ht = (d->tm[0].size & DECO_32x64) ? 64 : 32;
	const int wmask = wt * 16 - 1, hmask = ht * 16 - 1;
	for (int y = y0; y < y1; y++) {
		int c0, c1, bank0, bank1;
		uint16_t scrollx, scrolly;
		if (d->lines_on) {
			const deco16ic_line_t *l = &d->lines[(top + y) & 255];
			c0 = l->control0[0];
			c1 = l->control1[0];
			bank0 = l->bank[0];
			bank1 = l->bank[1];
			scrollx = l->scrollx[0];
			scrolly = l->scrolly[0];
		} else {
			c0 = d->tm[0].control0;
			c1 = d->tm[0].control1;
			bank0 = d->tm[0].bank;
			bank1 = d->tm[1].bank;
			scrollx = d->tm[0].scrollx;
			scrolly = d->tm[0].scrolly;
		}
		if (!(c0 & 0x80))
			continue;
		const int c1b = d->lines_on ? d->lines[(top + y) & 255].control1[1] : d->tm[1].control1;
		const int rows_on = rowscroll && (c1 & 0x40), cols_on = rowscroll && (c1 & 0x20);
		const int row_shift = (c0 >> 3) & 0xf, col_shift = 3 + (c0 & 7);
		const int src_y = (scrolly + top + y) & hmask;
		int src_x = scrollx;
		if (rows_on)
			src_x += rowscroll[src_y >> row_shift];
		src_x &= wmask;
		uint16_t *dst = bm + y * pitch;
		uint8_t *pp = pri ? pri + y * pitch : NULL;
		if (!cols_on) {   // (both tiles once per run of pixels in them: the same pixels as below)
			for (int x = 0; x < width;) {
				int fx0, fx1;
				uint16_t b0, b1;
				const uint8_t *r0 = pf_row16(d, 0, c1, bank0, src_x, src_y, &fx0, &b0);
				const uint8_t *r1 = pf_row16(d, 1, c1b, bank1, src_x, src_y, &fx1, &b1);
				int n = 16 - (src_x & 15);
				if (n > width - x)
					n = width - x;
				for (int i = 0, px = src_x & 15; i < n; i++, px++, x++) {
					const int pen0 = r0[fx0 ? 15 - px : px], pen1 = r1[fx1 ? 15 - px : px];
					const uint16_t p0 = (uint16_t)(b0 + pen0), p2 = (uint16_t)(b1 + pen1);
					if (opaque || pen0 || pen1) {
						dst[x] = mix ? mix(p0, p2) : (uint16_t)(p0 | (p2 & 0xf) << 4);
						if (pp)
							pp[x] = (uint8_t)(pp[x] | priority);
					}
				}
				src_x = (src_x + n) & wmask;
			}
			continue;
		}
		for (int x = 0; x < width; x++) {
			const int ty = cols_on ? (src_y + rowscroll[0x200 + ((src_x & 0x1ff) >> col_shift)]) & hmask : src_y;
			int pen0, pen1;
			uint16_t p = pf_pixel(d, 0, 0, c1, bank0, src_x, ty, &pen0);
			const uint16_t p2 = pf_pixel(d, 1, 0, c1b, bank1, src_x, ty, &pen1);
			p = mix ? mix(p, p2) : (uint16_t)(p | (p2 & 0xf) << 4);
			src_x = (src_x + 1) & wmask;
			if (opaque || pen0 || pen1) {
				dst[x] = p;
				if (pp)
					pp[x] = (uint8_t)(pp[x] | priority);
			}
		}
	}
}
