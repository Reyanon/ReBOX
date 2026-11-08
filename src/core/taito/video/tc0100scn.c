/*  Taito TC0100SCN (tc0100scn.h), ported from MAME src/mame/taito/tc0100scn.cpp (BSD-3-Clause,
    copyright-holders: Nicola Salmoria) and the tilemap placement rules of MAME src/emu/tilemap.cpp.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "tc0100scn.h"

#include <stdlib.h>
#include <string.h>

#define TC0100SCN_MAX_TEXT_TILES 64   // character columns of one screen row (up to 504 pixels)

int tc0100scn_init(tc0100scn_t *c, const uint8_t *gfx, uint32_t size, int layout)
{
	uint32_t n = layout == TC0100SCN_1BPP ? size / 8 : size / 32;
	if (!n)
		n = 1;
	uint8_t *t = (uint8_t *)calloc((size_t)n * 65, 1);
	if (!t)
		return -1;
	uint8_t *used = t + (size_t)n * 64;
	for (uint32_t i = 0; i < n && gfx; i++) {
		uint8_t *d = t + (size_t)i * 64, any = 0, all = 1;
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				uint8_t p;
				if (layout == TC0100SCN_1BPP)   // gfx_8x8x1
					p = (gfx[i * 8 + y] >> (7 - x)) & 1;
				else {                           // gfx_8x8x4_packed_msb
					uint8_t b = gfx[i * 32 + y * 4 + (x >> 1)];
					p = (x & 1) ? (b & 15) : (b >> 4);
				}
				d[y * 8 + x] = p;
				any |= p;
				if (!p)
					all = 0;
			}
		used[i] = (uint8_t)((any != 0) | all << 1);
	}
	c->owned = t;
	c->tiles = t;
	c->tile_used = used;
	c->ntiles = n;
	c->code_mask = 0xffff;
	c->code_bank = 0;
	c->screen_w = 320;
	c->screen_h = 256;
	c->pen_base = 0;
	tc0100scn_reset(c);
	return 0;
}

void tc0100scn_exit(tc0100scn_t *c)
{
	free(c->owned);
	c->owned = NULL;
	c->tiles = c->tile_used = NULL;
}

void tc0100scn_reset(tc0100scn_t *c)
{
	memset(c->ram, 0, sizeof(c->ram));
	memset(c->ctrl, 0, sizeof(c->ctrl));
}

void tc0100scn_ctrl_w(tc0100scn_t *c, int offset, uint16_t data, uint16_t mask)
{
	offset &= 7;
	c->ctrl[offset] = (uint16_t)((c->ctrl[offset] & ~mask) | (data & mask));
}

void tc0100scn_copy(tc0100scn_t *dst, const tc0100scn_t *src)
{
	// double width layout uses the extra 0x4000 bytes
	size_t n = (src->ctrl[6] & 0x10) ? sizeof(src->ram) : 0x10000;
	memcpy(dst->ram, src->ram, n);
	memcpy(dst->ctrl, src->ctrl, sizeof(src->ctrl));
	dst->code_bank = src->code_bank;
}

static inline uint32_t tile_code(const tc0100scn_t *c, uint32_t code)
{
	code = (code & c->code_mask) | c->code_bank;
	return code < c->ntiles ? code : code % c->ntiles;
}

// one run of a tile row: full = every pixel (opaque draw or a tile without pen 0), else pen 0 transparent
static inline void put_run(uint16_t *d, uint8_t *p, const uint8_t *src, int px, int step, int fx, uint16_t col, int run,
                           uint8_t prio, int full)
{
	if (full) {
		if (step > 0 && !fx)
			for (int i = 0; i < run; i++)
				d[i] = (uint16_t)(col | src[px + i]);
		else
			for (int i = 0; i < run; i++, px += step)
				d[i] = (uint16_t)(col | src[px ^ fx]);
		if (prio)
			for (int i = 0; i < run; i++)
				p[i] |= prio;
		return;
	}
	for (int i = 0; i < run; i++, px += step) {
		uint8_t pen = src[px ^ fx];
		if (pen) {
			d[i] = (uint16_t)(col | pen);
			p[i] |= prio;
		}
	}
}

// one screen row of a ROM tile layer: logical tilemap row ly, column lx (stepping +1 / -1)
static void bg_row(const tc0100scn_t *c, const uint16_t *map, int cols, int wmask, int ly, int lx, int step, int colbank,
                   uint16_t *d, uint8_t *p, int n, uint8_t prio, int opaque)
{
	const uint16_t *row = map + (ly >> 3) * cols * 2;
	int py = ly & 7;
	int x = 0;
	while (x < n) {
		int px = lx & 7;
		int run = step > 0 ? 8 - px : px + 1;
		if (run > n - x)
			run = n - x;
		const uint16_t *e = row + (lx >> 3) * 2;
		uint32_t code = tile_code(c, e[1]);
		uint8_t used = c->tile_used[code];
		if (opaque || used) {   // opaque: MAME TILEMAP_DRAW_OPAQUE
			uint16_t attr = e[0];
			int fx = (attr & 0x4000) ? 7 : 0, fy = (attr & 0x8000) ? 7 : 0;
			const uint8_t *src = c->tiles + (size_t)code * 64 + ((py ^ fy) << 3);
			uint16_t col = (uint16_t)(((((attr & 0xff) + colbank) & 0xff) << 4) + c->pen_base);
			put_run(d + x, p + x, src, px, step, fx, col, run, prio, opaque || (used & 2));
		}
		x += run;
		lx = (lx + step * run) & wmask;
	}
}

// one screen row of the text layer (2bpp characters in RAM)
static void tx_row(const tc0100scn_t *c, const uint16_t *map, const uint16_t *chars, int cols, int wmask, int ly, int lx,
                   int step, uint16_t *d, uint8_t *p, int n, uint8_t prio, int opaque)
{
	const uint16_t *row = map + (ly >> 3) * cols;
	int py = ly & 7;
	int x = 0;
	while (x < n) {
		int px = lx & 7;
		int run = step > 0 ? 8 - px : px + 1;
		if (run > n - x)
			run = n - x;
		uint16_t attr = row[lx >> 3];
		int fx = (attr & 0x4000) ? 7 : 0, fy = (attr & 0x8000) ? 7 : 0;
		uint16_t w = chars[(attr & 0xff) * 8 + (py ^ fy)];
		if (w || opaque) {
			uint16_t col = (uint16_t)(((((attr & 0x3f00) >> 8) + c->tx_colbank) << 4) + c->pen_base);
			for (int i = 0; i < run; i++, px += step) {
				int b = px ^ fx;
				int pen = ((w >> (15 - b)) & 1) << 1 | ((w >> (7 - b)) & 1);
				if (pen || opaque) {
					d[x + i] = (uint16_t)(col | pen);
					p[x + i] |= prio;
				}
			}
		}
		x += run;
		lx = (lx + step * run) & wmask;
	}
}

int tc0100scn_draw(const tc0100scn_t *c, int layer, uint16_t *bm, uint8_t *pri, int pitch, int sy0, int rows,
                   uint8_t priority)
{
	return tc0100scn_draw_ex(c, layer, bm, pri, pitch, sy0, rows, priority, 0);
}

int tc0100scn_draw_ex(const tc0100scn_t *c, int layer, uint16_t *bm, uint8_t *pri, int pitch, int sy0, int rows,
                      uint8_t priority, int flags)
{
	const int opaque = flags & TC0100SCN_OPAQUE;
	int disable = c->ctrl[6] & 0xf7;
	int dbl = (c->ctrl[6] & 0x10) >> 4;
	int flip = c->ctrl[7] & 1;
	int sw = c->screen_w, sh = c->screen_h;
	int n = sw;
	// MAME scrolldx / scrolldy of the three tilemaps (device_start)
	int xd, yd;
	if (!dbl) {
		xd = c->multiscrn_hack == 0 ? -c->x_offset : -c->x_offset - 2;
		yd = c->multiscrn_hack == 0 ? 8 - c->y_offset : 1 - c->y_offset;
	} else {
		xd = -c->x_offset - c->multiscrn_xoffs;
		yd = 8 - c->y_offset;
	}
	int bg_dx = xd - 16, bg_dxf = -c->flip_xoffs - xd - 16;
	int bg_dy = yd, bg_dyf = -c->flip_yoffs - yd;
	int cols = dbl ? 128 : 64;
	int W = cols * 8, wmask = W - 1;
	const uint16_t *bgscroll = c->ram + (dbl ? 0x10000 / 2 : 0xc000 / 2);
	const uint16_t *fgscroll = c->ram + (dbl ? 0x10400 / 2 : 0xc400 / 2);
	const uint16_t *colscroll = c->ram + (dbl ? 0x10800 / 2 : 0xe000 / 2);

	switch (layer) {
	case 0: {   // BG0: tilemap with 512 scroll rows
		if (disable & 1)
			return 1;
		int scrolly = -(int16_t)c->ctrl[3], scrollx = -(int16_t)c->ctrl[0];
		for (int r = 0; r < rows; r++) {
			int sy = sy0 + r, ly, lx;
			if (!flip) {
				ly = (sy - bg_dy + scrolly) & 511;
				int rs = scrollx - (int16_t)bgscroll[(ly - scrolly) & 0x1ff];
				lx = (-bg_dx + rs) & wmask;
			} else {
				ly = (sh - 1 - sy - bg_dyf + scrolly) & 511;
				int rs = scrollx - (int16_t)bgscroll[(ly - scrolly) & 0x1ff];
				lx = (sw - 1 - bg_dxf + rs) & wmask;
			}
			bg_row(c, c->ram, cols, wmask, ly, lx, flip ? -1 : 1, c->bg_colbank[0], bm + r * pitch, pri + r * pitch, n, priority, opaque);
		}
		return 0;
	}
	case 1: {   // BG1: MAME tilemap_draw_fg (rowscroll in screen space, colscroll per 8 pixel source column)
		if (disable & 2)
			return 1;
		const uint16_t *map = c->ram + 0x4000;
		int H = 512, hmask = H - 1;
		int scrollx_delta = -(flip ? bg_dxf : bg_dx), scrolly_delta = -(flip ? bg_dyf : bg_dy);
		int fgscrollx = -(int16_t)c->ctrl[1], fgscrolly = -(int16_t)c->ctrl[4];
		int src_y = (fgscrolly + scrolly_delta) & hmask;
		if (flip)
			src_y = (256 - src_y) & hmask;
		src_y += sy0;
		for (int r = 0; r < rows; r++, src_y++) {
			int sy = sy0 + r;
			int src_x = (fgscrollx - (int16_t)fgscroll[(sy + scrolly_delta) & 0x1ff] + scrollx_delta) & wmask;
			if (flip)
				src_x = (256 - 64 - src_x) & wmask;
			uint16_t *d = bm + r * pitch;
			uint8_t *p = pri + r * pitch;
			int x = 0;
			while (x < n) {
				int run = 8 - (src_x & 7);
				if (run > n - x)
					run = n - x;
				int py = (src_y - (int16_t)colscroll[(src_x & 0x3ff) >> 3]) & hmask;
				int ly = flip ? hmask - py : py;
				int lx = flip ? wmask - src_x : src_x;
				const uint16_t *e = map + ((ly >> 3) * cols + (lx >> 3)) * 2;
				uint32_t code = tile_code(c, e[1]);
				uint8_t used = c->tile_used[code];
				if (opaque || used) {
					uint16_t attr = e[0];
					int fx = (attr & 0x4000) ? 7 : 0, fy = (attr & 0x8000) ? 7 : 0;
					const uint8_t *src = c->tiles + (size_t)code * 64 + (((ly & 7) ^ fy) << 3);
					uint16_t col = (uint16_t)(((((attr & 0xff) + c->bg_colbank[1]) & 0xff) << 4) + c->pen_base);
					put_run(d + x, p + x, src, lx & 7, flip ? -1 : 1, fx, col, run, priority, opaque || (used & 2));
				}
				x += run;
				src_x = (src_x + run) & wmask;
			}
		}
		return 0;
	}
	case 2: {   // text layer
		if (disable & 4)
			return 1;
		int tx_dx = xd - 16, tx_dxf = -c->flip_text_xoffs - xd - 16 - 7;
		int tx_dy = yd, tx_dyf = -c->flip_text_yoffs - yd;
		int tcols = dbl ? 128 : 64, trows = dbl ? 32 : 64;
		int TW = tcols * 8, TH = trows * 8;
		const uint16_t *map = c->ram + (dbl ? 0x9000 : 0x2000);
		const uint16_t *chars = c->ram + (dbl ? 0x8800 : 0x3000);
		int scrollx = -(int16_t)c->ctrl[2], scrolly = -(int16_t)c->ctrl[5];
		if (!flip && !opaque && n <= (TC0100SCN_MAX_TEXT_TILES - 1) * 8) {
			// mostly empty layer: per row of characters, list the columns whose character has a set pixel
			uint8_t empty[256];
			for (int ch = 0; ch < 256; ch++) {
				const uint16_t *cw = chars + ch * 8;
				empty[ch] = !(cw[0] | cw[1] | cw[2] | cw[3] | cw[4] | cw[5] | cw[6] | cw[7]);
			}
			const int lx = (-tx_dx + scrollx) & (TW - 1);
			const int t0 = lx >> 3, sub = lx & 7, ntiles = (sub + n + 7) >> 3;
			int16_t list[TC0100SCN_MAX_TEXT_TILES];
			int nlist = 0, cur_row = -1;
			for (int r = 0; r < rows; r++) {
				const int ly = (sy0 + r - tx_dy + scrolly) & (TH - 1);
				const uint16_t *mrow = map + (ly >> 3) * tcols;
				if ((ly >> 3) != cur_row) {
					cur_row = ly >> 3;
					nlist = 0;
					for (int j = 0; j < ntiles && j < TC0100SCN_MAX_TEXT_TILES; j++)
						if (!empty[mrow[(t0 + j) & (tcols - 1)] & 0xff])
							list[nlist++] = (int16_t)j;
				}
				uint16_t *d = bm + r * pitch;
				uint8_t *p = pri + r * pitch;
				const int py = ly & 7;
				for (int k = 0; k < nlist; k++) {
					const int j = list[k], xs = j * 8 - sub;
					const uint16_t attr = mrow[(t0 + j) & (tcols - 1)];
					const int fx = (attr & 0x4000) ? 7 : 0, fy = (attr & 0x8000) ? 7 : 0;
					const uint16_t w = chars[(attr & 0xff) * 8 + (py ^ fy)];
					if (!w)
						continue;
					const uint16_t col = (uint16_t)(((((attr & 0x3f00) >> 8) + c->tx_colbank) << 4) + c->pen_base);
					const int xa = xs < 0 ? 0 : xs, xb = xs + 8 > n ? n : xs + 8;
					for (int x = xa; x < xb; x++) {
						const int b = (x - xs) ^ fx;
						const int pen = ((w >> (15 - b)) & 1) << 1 | ((w >> (7 - b)) & 1);
						if (pen) {
							d[x] = (uint16_t)(col | pen);
							p[x] |= priority;
						}
					}
				}
			}
			return 0;
		}
		for (int r = 0; r < rows; r++) {
			int sy = sy0 + r, ly, lx;
			if (!flip) {
				ly = (sy - tx_dy + scrolly) & (TH - 1);
				lx = (-tx_dx + scrollx) & (TW - 1);
			} else {
				ly = (sh - 1 - sy - tx_dyf + scrolly) & (TH - 1);
				lx = (sw - 1 - tx_dxf + scrollx) & (TW - 1);
			}
			tx_row(c, map, chars, tcols, TW - 1, ly, lx, flip ? -1 : 1, bm + r * pitch, pri + r * pitch, n, priority, opaque);
		}
		return 0;
	}
	}
	return 1;
}
