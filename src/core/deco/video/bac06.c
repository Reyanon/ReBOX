/*  Data East BAC06 tilemap chip, after MAME src/mame/dataeast/decbac06.cpp (BSD-3-Clause, copyright-holders:
    Bryan McPhail, David Haywood). See bac06.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "bac06.h"

#include <string.h>

void bac06_reset(bac06_t *b)
{
	memset(b->ctrlreg, 0, sizeof(b->ctrlreg));
	memset(b->scrollreg, 0, sizeof(b->scrollreg));
	b->rambank = 0;
	b->flip_screen = 0;
}

void bac06_ctrl_w(bac06_t *b, int offset, uint16_t data, uint16_t mask)
{
	offset &= 3;
	b->ctrlreg[offset] = (uint16_t)((b->ctrlreg[offset] & ~mask) | (data & mask));
	if (offset == 2)
		b->rambank = b->ctrlreg[2] & 1;
}
void bac06_scroll_w(bac06_t *b, int offset, uint16_t data, uint16_t mask)
{
	offset &= 7;
	b->scrollreg[offset] = (uint16_t)((b->scrollreg[offset] & ~mask) | (data & mask));
}
uint16_t bac06_vram_r(const bac06_t *b, int offset) { return b->vram[((b->rambank & 1) ? offset + 0x1000 : offset) & 0x1fff]; }
void bac06_vram_w(bac06_t *b, int offset, uint16_t data, uint16_t mask)
{
	uint16_t *v = &b->vram[((b->rambank & 1) ? offset + 0x1000 : offset) & 0x1fff];
	*v = (uint16_t)((*v & ~mask) | (data & mask));
}

// the tilemap shapes' memory layouts (MAME tile_shape*_scan)
static uint32_t scan16(const bac06_t *b, int shape, uint32_t col, uint32_t row, uint32_t ncols)
{
	switch (shape) {
	case 0:
		if ((b->ctrlreg[0] & 2) == 0) {
			const uint32_t col_mask = ncols - 1;
			return (row & 0xf) + ((col_mask - (col & col_mask)) << 4);
		}
		return (col & 0xf) + ((row & 0xf) << 4) + ((col & 0x1f0) << 4);
	case 1: return (col & 0xf) + ((row & 0x1f) << 4) + ((col & 0xf0) << 5);
	default: return (col & 0xf) + ((row & 0x3f) << 4) + ((col & 0x70) << 6);
	}
}
static uint32_t scan8(int shape, uint32_t col, uint32_t row)
{
	switch (shape) {
	case 0: return (col & 0x1f) + ((row & 0x1f) << 5) + ((col & 0x60) << 5);
	case 1: return (col & 0x1f) + ((row & 0x1f) << 5) + ((row & 0x20) << 5) + ((col & 0x20) << 6);
	default: return (col & 0x1f) + ((row & 0x7f) << 5);
	}
}

typedef struct {
	const bac06_t *b;
	const rbx_gfx *g;
	int is8, shape, ts, tsh, ncols, nrows, w, h, base, gran, flipx_bit;
} map_t;

// the tilemap picture's tile at (x, y) (MAME's tilemap flip: the picture turned round) once for its row: the pixels row[px], row[px + dir], ... for the screen pixels from x
// to the tile's edge (ts - (x & (ts - 1)) of them, flipped or not); *pen0: the pen base, *fl: the tile's flags
static inline const uint8_t *map_row(const map_t *m, int x, int y, int *px, int *dir, uint32_t *pen0, uint32_t *fl)
{
	int d = 1;
	if (m->b->flip_screen) {
		x = m->w - 1 - x;
		y = m->h - 1 - y;
		d = -1;
	}
	const uint32_t col = (uint32_t)x >> m->tsh, row = (uint32_t)y >> m->tsh;
	uint32_t idx = m->is8 ? scan8(m->shape, col, row) : scan16(m->b, m->shape, col, row, (uint32_t)m->ncols);
	if (m->b->rambank & 1)
		idx += 0x1000;
	uint32_t tile = m->b->vram[idx & 0x1fff];
	uint32_t colour = tile >> 12, flags = m->flipx_bit;
	tile &= 0xfff;
	if (m->b->tile_cb)
		m->b->tile_cb(&tile, &colour, &flags);
	int p = x & (m->ts - 1);
	if (flags & 1) {
		p = m->ts - 1 - p;
		d = -d;
	}
	const int py = y & (m->ts - 1);
	const uint32_t code = tile < m->g->count ? tile : m->g->count ? tile % m->g->count : 0;   // (the division only past the end)
	*px = p;
	*dir = d;
	*pen0 = (uint32_t)m->base + (colour << m->gran);
	*fl = flags;
	return m->g->pix + ((size_t)code * (size_t)m->ts + (size_t)py) * (size_t)m->ts;
}

void bac06_draw(const bac06_t *b, uint16_t *bm, uint8_t *pri, int pitch, int width, int top, int y0, int y1,
                int flags, uint8_t priority, uint8_t pmask)
{
	const int opaque = flags & BAC06_OPAQUE;
	const int want = (flags & (BAC06_LAYER0 | BAC06_LAYER1)) ? ((flags & BAC06_LAYER0) ? 1 : 0) | ((flags & BAC06_LAYER1) ? 2 : 0) : 1;
	int shape = b->ctrlreg[3] & 3;
	if (shape == 3)
		shape = 1;   // (3 is invalid / the same as 1?)
	int is8;
	if (b->ctrlreg[0] & 1)   // 8x8 tiles selected
		is8 = !b->no_8x8 ? 1 : (!b->no_16x16 ? 0 : -1);
	else
		is8 = !b->no_16x16 ? 0 : (!b->no_8x8 ? 1 : -1);
	if (b->bootleg8) {
		shape = b->bootleg8 - 1;
		is8 = 1;
	}
	if (is8 < 0)
		return;
	map_t m;
	m.b = b;
	m.g = is8 ? b->gfx8 : b->gfx16;
	if (!m.g || !m.g->count)
		return;
	m.is8 = is8;
	m.shape = shape;
	m.ts = is8 ? 8 : 16;
	m.tsh = is8 ? 3 : 4;
	if (is8) {
		static const int c8[3] = { 128, 64, 32 }, r8[3] = { 32, 64, 128 };
		m.ncols = c8[shape];
		m.nrows = r8[shape];
		m.flipx_bit = (b->ctrlreg[0] & 2) ? 0 : 1;
	} else {
		static const int c16[3] = { 64, 32, 16 }, r16[3] = { 16, 32, 64 };
		const int wide = b->wide > 2 ? 2 : b->wide;
		m.ncols = c16[shape] << wide;
		m.nrows = r16[shape];
		m.flipx_bit = (((b->ctrlreg[0] >> 1) & 1) ^ b->thedeep_kludge) ? 0 : 1;
	}
	m.w = m.ncols * m.ts;
	m.h = m.nrows * m.ts;
	m.base = is8 ? b->base8 : b->base16;
	m.gran = (is8 ? b->gran8 : b->gran16) ? (is8 ? b->gran8 : b->gran16) : 4;
	const uint16_t *ctrl0 = b->ctrlreg, *ctrl1 = b->scrollreg;
	uint32_t scrollx, scrolly;
	if ((ctrl0[0] & 2) == 0)   // (column major mode inverts the scroll direction)
		scrollx = (uint32_t)(-(int)ctrl1[0] - 0x100);
	else
		scrollx = ctrl1[0];
	scrolly = ctrl1[1];
	if (b->bootleg8)   // (no control registers given: no scroll)
		scrollx = scrolly = 0;
	int rows_on = 0, cols_on = 0;
	if (!b->no_rc_scroll && !b->bootleg8) {
		rows_on = (ctrl0[0] >> 2) & 1;
		cols_on = (ctrl0[0] >> 3) & 1;
	}
	const int width_mask = m.w - 1, height_mask = m.h - 1;
	int src_y = b->flip_screen ? (int)((m.h - 256) - scrolly) : (int)scrolly;
	src_y += top + y0;
	for (int y = y0; y < y1; y++, src_y++) {
		uint16_t *d = bm + y * pitch;
		uint8_t *pp = pri ? pri + y * pitch : NULL;
		int src_x;
		if (rows_on)
			src_x = (int)(scrollx + b->rowscroll[(src_y >> (ctrl1[3] & 0xf)) & (0x1ff >> (ctrl1[3] & 0xf))]);
		else
			src_x = (int)scrollx;
		if (b->flip_screen)
			src_x = (m.w - 256) - src_x;
		// (a run of pixels in one tile, and one column scroll group of 8: the tile looked up once; a pixel's
		// layers: bit 0 LAYER0, bit 1 LAYER1)
		for (int x = 0; x < width;) {
			int column_offset = 0;
			if (cols_on)
				column_offset = b->colscroll[((src_x >> 3) >> (ctrl1[2] & 0xf)) & (0x3f >> (ctrl1[2] & 0xf))];
			const int mx = src_x & width_mask;
			int n = m.ts - (mx & (m.ts - 1));
			if (cols_on && n > 8 - (src_x & 7))
				n = 8 - (src_x & 7);
			if (n > width - x)
				n = width - x;
			int px, dir;
			uint32_t pen0, tf;
			const uint8_t *row = map_row(&m, mx, (src_y + column_offset) & height_mask, &px, &dir, &pen0, &tf);
			const int g = (tf & BAC06_GROUP1) ? 1 : 0;
			for (int i = 0; i < n; i++, x++, px += dir) {
				const uint8_t p = row[px];
				int cat;
				if (b->transmask_on)
					cat = (((b->fgmask[g] >> p) & 1) ? 0 : 1) | (((b->bgmask[g] >> p) & 1) ? 0 : 2);
				else
					cat = p != 0;
				if (opaque || (cat & want)) {
					d[x] = (uint16_t)(pen0 + p);
					if (pp)
						pp[x] = (uint8_t)((pp[x] & pmask) | priority);
				}
			}
			src_x += n;
		}
	}
}
