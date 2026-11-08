/*  Konami 052109/051962 tilemaps. Ported from MAME src/mame/konami/k052109.cpp (BSD-3-Clause, copyright-holders:
    Fabio Priuli, Acho A. Tang, R. Belmont) and the parts of src/emu/tilemap.cpp it uses (draw with its row and
    column scrolls, effective_rowscroll / effective_colscroll; BSD-3-Clause, copyright-holders: Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "k052109.h"
#include "../../rbx.h"

#include <stdlib.h>
#include <string.h>

// the chip's RAM: the colour (attribute) RAM, the code RAM and X-Men's extra code RAM of each layer
static const int s_cram[3] = { 0x0000, 0x0800, 0x1000 }, s_vram1[3] = { 0x2000, 0x2800, 0x3000 },
                 s_vram2[3] = { 0x4000, 0x4800, 0x5000 };
static const int s_dx[3] = { -96, -90, -90 };   // (set_scrolldx)

int k052109_init(k052109_t *k, const uint8_t *rom, uint32_t rom_size, int palette_entries, k052109_cb_t cb)
{
	memset(k, 0, sizeof(*k));
	k->rom = rom;
	k->rom_size = rom_size;
	k->cb = cb;
	k->colors = palette_entries / 16;
	k->ntiles = rom_size / 32;   // (charlayout: planes 24 16 8 0, a row 32 bits)
	k->gfx = (uint8_t *)malloc((size_t)k->ntiles * 64 + 1);
	if (!k->gfx)
		return -1;
	for (uint32_t t = 0; t < k->ntiles; t++)
		for (int y = 0; y < 8; y++) {
			const uint8_t *r = rom + t * 32 + y * 4;
			for (int x = 0; x < 8; x++) {
				const int s = 7 - x;
				k->gfx[t * 64 + y * 8 + x] = (uint8_t)((r[3] >> s & 1) << 3 | (r[2] >> s & 1) << 2 | (r[1] >> s & 1) << 1 |
				                                       (r[0] >> s & 1));
			}
		}
	return 0;
}

int k052109_init_charram(k052109_t *k, const uint16_t *ram, uint32_t size, int palette_entries, k052109_cb_t cb)
{
	if (k052109_init(k, NULL, 0, palette_entries, cb))
		return -1;
	free(k->gfx);
	k->ntiles = size / 32;
	k->gfx = (uint8_t *)calloc((size_t)k->ntiles * 64 + 1, 1);
	if (!k->gfx)
		return -1;
	for (uint32_t t = 0; t < k->ntiles; t++)
		k052109_charram_decode(k, ram, t);
	return 0;
}

void k052109_charram_decode(k052109_t *k, const uint16_t *ram, uint32_t tile)
{
	if (tile >= k->ntiles)
		return;
	uint8_t *d = k->gfx + tile * 64;
	for (int y = 0; y < 8; y++) {   // (charlayout_ram: a row of 32 bits, 4 bpp, the first pixel in the top nibble)
		const uint32_t row = (uint32_t)ram[tile * 16 + y * 2] << 16 | ram[tile * 16 + y * 2 + 1];
		for (int x = 0; x < 8; x++)
			d[y * 8 + x] = (uint8_t)(row >> (28 - 4 * x) & 15);
	}
}

void k052109_exit(k052109_t *k)
{
	free(k->gfx);
	k->gfx = NULL;
	k->ntiles = 0;
}

void k052109_reset(k052109_t *k)
{
	memset(k->ram, 0, sizeof(k->ram));   // (make_unique_clear at start; the machine's reset clears it with the start)
	k->rmrd_line = 0;
	k->has_extra_video_ram = 0;
	k->irq_control = 0;
	k->irq_out = k->firq_out = k->nmi_out = 0;
	k->romsubbank = 0;
	k->scrollctrl = 0;
	for (int off = 0x1c00; off <= 0x1f00; off += 0x80)
		k052109_write(k, off, 0);
	memset(k->charrombank, 0, sizeof(k->charrombank));
	memset(k->charrombank_2, 0, sizeof(k->charrombank_2));
	for (int t = 0; t < 2; t++) {
		k->rows[t] = k->cols[t] = 1;
		memset(k->rowscroll[t], 0, sizeof(k->rowscroll[t]));
		memset(k->colscroll[t], 0, sizeof(k->colscroll[t]));
	}
}

uint8_t k052109_read(const k052109_t *k, int offset)
{
	if (!k->rmrd_line || !k->rom)
		return k->ram[offset];
	// (the character ROM through the RAM's window: Punk Shot and TMNT 0000-1fff, Aliens 2000-3fff)
	int code = (offset & 0x1fff) >> 5, color = k->romsubbank, flags = 0, priority = 0;
	int bank = k->charrombank[(color & 0x0c) >> 2] >> 2;
	bank |= k->charrombank_2[(color & 0x0c) >> 2] >> 2;
	if (k->has_extra_video_ram)
		code |= color << 8;   // (kludge for X-Men)
	else if (k->cb)
		k->cb(0, bank, &code, &color, &flags, &priority);
	uint32_t addr = ((uint32_t)code << 5) + (uint32_t)(offset & 0x1f);
	addr &= k->rom_size - 1;
	return k->rom[addr];
}

void k052109_write(k052109_t *k, int offset, uint8_t data)
{
	if ((offset & 0x1fff) < 0x1800) {   // (the tilemap RAM)
		if (offset >= 0x4000)
			k->has_extra_video_ram = 1;   // (kludge for X-Men)
		k->ram[offset] = data;
		return;
	}
	k->ram[offset] = data;
	switch (offset) {
	case 0x1c00: k->addrmap = data; break;
	case 0x1c80: k->scrollctrl = data; break;
	case 0x1d00:   // (the interrupts cleared when their enable goes off)
		if (~data & k->irq_control & 1)
			k->nmi_out = 0;
		if (~data & k->irq_control & 2)
			k->firq_out = 0;
		if (~data & k->irq_control & 4)
			k->irq_out = 0;
		k->irq_control = data;
		break;
	case 0x1d80:
		k->charrombank[0] = data & 0x0f;
		k->charrombank[1] = (data >> 4) & 0x0f;
		break;
	case 0x1e00: case 0x3e00: k->romsubbank = data; break;   // (Surprise Attack: 3e00)
	case 0x1e80: k->tileflip_enable = data & 0x07; break;
	case 0x1f00:
		k->charrombank[2] = data & 0x0f;
		k->charrombank[3] = (data >> 4) & 0x0f;
		break;
	case 0x3d80:   // (Surprise Attack's ROM test)
		k->charrombank_2[0] = data & 0x0f;
		k->charrombank_2[1] = (data >> 4) & 0x0f;
		break;
	case 0x3f00:
		k->charrombank_2[2] = data & 0x0f;
		k->charrombank_2[3] = (data >> 4) & 0x0f;
		break;
	}
}

static void update_scroll(k052109_t *k)
{
	static const int rows_table[4] = { 1, 1, 32, 256 };
	for (int t = 0; t < 2; t++) {
		const int ctrl = k->scrollctrl >> (t * 3) & 7;
		const int rows = rows_table[ctrl & 3], cols = (ctrl & 4) ? 64 : 1;
		const uint8_t *sy = &k->ram[0x1800 | (t ? 0x2000 : 0)], *sx = &k->ram[0x1a00 | (t ? 0x2000 : 0)];
		int16_t *rs = k->rowscroll[t], *cs = k->colscroll[t];
		if (rows == 1 && cols == 1) {   // (standard xy scroll)
			k->rows[t] = k->cols[t] = 1;
			rs[0] = (int16_t)(sx[0] + 256 * sx[1]);
			cs[0] = sy[12];
		} else if (cols == 1) {   // (rowscroll)
			k->rows[t] = 256;
			k->cols[t] = 1;
			const int yscroll = sy[12];
			cs[0] = (int16_t)yscroll;
			const int mask = rows == 256 ? 0xff : 0xf8;
			for (int offs = 0; offs < 256; offs++)
				rs[(offs + yscroll) & 0xff] = (int16_t)(sx[2 * (offs & mask)] + 256 * sx[2 * (offs & mask) + 1]);
		} else if (rows == 1) {   // (colscroll)
			k->rows[t] = 1;
			k->cols[t] = 64;
			int xscroll = sx[0] + 256 * sx[1];
			rs[0] = (int16_t)xscroll;
			xscroll /= 8;
			for (int offs = 0; offs < 64; offs++)
				cs[(offs + xscroll) & 0x3f] = sy[offs];
		} else {   // (mixed: tile_blitter)
			k->rows[t] = (uint16_t)rows;
			k->cols[t] = 64;
			const int step = rows == 256 ? 2 : 16;
			for (int offs = 0; offs < rows; offs++)
				rs[offs] = (int16_t)(sx[step * offs] + 256 * sx[step * offs + 1]);
			for (int offs = 0; offs < 64; offs++)
				cs[offs] = sy[offs];
		}
	}
}

void k052109_vblank(k052109_t *k, int state)
{
	if (state && (k->irq_control & 4))
		k->irq_out = 1;
	if (!state)
		update_scroll(k);
}

static inline int wrap(int v, int n) { return v < 0 ? n - (-v) % n : v % n; }

void k052109_prepare(const k052109_t *k, k052109_view_t *v, int xextent, int yextent)
{
	const int flip = k->tileflip_enable & 1;
	v->flip = flip;
	for (int layer = 0; layer < 3; layer++) {
		const uint8_t *cram = k->ram + s_cram[layer], *v1 = k->ram + s_vram1[layer], *v2 = k->ram + s_vram2[layer];
		for (int i = 0; i < 2048; i++) {   // (get_tile_info)
			int code = v1[i] + 256 * v2[i], color = cram[i], flags = 0, priority = 0;
			int bank = k->charrombank[(color & 0x0c) >> 2];
			if (!(k->addrmap & 0x40))
				color = (color & 0xf3) | ((bank & 0x03) << 2);
			bank >>= 2;
			const int flipy = color & 0x02;
			if (k->cb)
				k->cb(layer, bank, &code, &color, &flags, &priority);
			if (!(k->tileflip_enable & 2))
				flags &= ~K052109_TILE_FLIPX;
			if (flipy && (k->tileflip_enable & 4))
				flags |= K052109_TILE_FLIPY;
			const uint32_t t = k->ntiles ? (uint32_t)code % k->ntiles : 0;
			v->base[layer][i] = t * 64 | (uint32_t)(flags & 3) << 28;
			v->pen[layer][i] = (uint16_t)((uint32_t)color % (uint32_t)k->colors * 16);
			v->cat[layer][i] = (uint8_t)priority;
		}
		// the scroll (tilemap 0 has none), effective as MAME's tilemap computes it with its flip and dx
		const int rows = layer ? k->rows[layer - 1] : 1, cols = layer ? k->cols[layer - 1] : 1;
		v->rows[layer] = rows;
		v->cols[layer] = cols;
		for (int i = 0; i < rows; i++) {
			const int idx = flip ? rows - 1 - i : i;
			const int r = layer ? k->rowscroll[layer - 1][idx] : 0;
			v->sx[layer][i] = wrap(!flip ? s_dx[layer] - r : xextent - 512 - (s_dx[layer] - r), 512);
			v->raw_rowscroll[layer][i] = (int16_t)(layer ? k->rowscroll[layer - 1][i] : 0);
		}
		for (int i = 0; i < cols; i++) {
			const int idx = flip ? cols - 1 - i : i;
			const int c = layer ? k->colscroll[layer - 1][idx] : 0;
			v->sy[layer][i] = wrap(!flip ? -c : yextent - 256 - (0 - c), 256);
		}
	}
}

void k052109_draw(const k052109_t *k, const k052109_view_t *v, int layer, int flags, uint8_t priority, uint16_t *frame,
                  uint8_t *pri, int pitch, int fx0, int fy0, int cx0, int cy0, int cx1, int cy1)
{
	const uint32_t *base = v->base[layer];
	const uint16_t *pen = v->pen[layer];
	const uint8_t *cat = v->cat[layer];
	const uint8_t *gfx = k->gfx;
	if (!gfx)
		return;
	const int all = flags & K052109_DRAW_ALL_CATEGORIES, opaque = flags & K052109_DRAW_OPAQUE, want = flags & 0x0f;
	const uint8_t keep = (flags >> 8) ? (uint8_t)(flags >> 8) : 0xff;   // (K052109_DRAW_PRIMASK: the priority bitmap's kept bits)
	const int flip = v->flip, rows = v->rows[layer], cols = v->cols[layer];
	// (a pixel of the pixmap at (sx, sy): the logical map's at the mirrored place when the tilemap is flipped)
#define PIXEL(SX, SY) do { \
		const int lx = flip ? 511 - (SX) : (SX), ly = flip ? 255 - (SY) : (SY); \
		const int ti = (ly >> 3) * 64 + (lx >> 3); \
		if (all || cat[ti] == want) { \
			const uint32_t b = base[ti]; \
			int px = lx & 7, py = ly & 7; \
			if (b & (1u << 28)) px = 7 - px; \
			if (b & (2u << 28)) py = 7 - py; \
			const int p = gfx[(b & 0x0fffffff) + (uint32_t)(py * 8 + px)]; \
			if (p || opaque) { \
				o[x] = (uint16_t)(pen[ti] + p); \
				pr[x] = (uint8_t)((pr[x] & keep) | priority); \
			} \
		} \
	} while (0)
	for (int y = cy0; y <= cy1; y++) {
		uint16_t *o = frame + (y - fy0) * pitch - fx0;
		uint8_t *pr = pri + (y - fy0) * pitch - fx0;
		if (cols == 1) {   // (xy scroll or rowscroll: the source line the same along the line)
			const int sy = (y - v->sy[layer][0]) & 255;
			const int sxs = v->sx[layer][rows == 1 ? 0 : sy / (256 / rows)];
			int sx = (cx0 - sxs) & 511;
			if (!flip) {   // (by tiles)
				const int ly = sy, row = (ly >> 3) * 64;
				for (int x = cx0; x <= cx1;) {
					const int ti = row + (sx >> 3);
					int n = 8 - (sx & 7);
					if (n > cx1 - x + 1)
						n = cx1 - x + 1;
					if (all || cat[ti] == want) {
						const uint32_t b = base[ti];
						const int py = (b & (2u << 28)) ? 7 - (ly & 7) : (ly & 7);
						const uint8_t *src = gfx + (b & 0x0fffffff) + py * 8;
						const uint16_t pb = pen[ti];
						const int fx = (b & (1u << 28)) != 0;
						for (int i = 0; i < n; i++) {
							const int px = (sx + i) & 7;
							const int p = src[fx ? 7 - px : px];
							if (p || opaque) {
								o[x + i] = (uint16_t)(pb + p);
								pr[x + i] = (uint8_t)((pr[x + i] & keep) | priority);
							}
						}
					}
					x += n;
					sx = (sx + n) & 511;
				}
			} else
				for (int x = cx0; x <= cx1; x++, sx = (sx + 1) & 511)
					PIXEL(sx, sy);
		} else if (rows == 1 && !flip) {   // (colscroll: by tiles, a tile's column one scroll)
			int sx = (cx0 - v->sx[layer][0]) & 511;
			for (int x = cx0; x <= cx1;) {
				int n = 8 - (sx & 7);
				if (n > cx1 - x + 1)
					n = cx1 - x + 1;
				const int ly = (y - v->sy[layer][(sx >> 3) & 63]) & 255;
				const int ti = (ly >> 3) * 64 + (sx >> 3);
				if (all || cat[ti] == want) {
					const uint32_t b = base[ti];
					const int py = (b & (2u << 28)) ? 7 - (ly & 7) : (ly & 7);
					const uint8_t *src = gfx + (b & 0x0fffffff) + py * 8;
					const uint16_t pb = pen[ti];
					const int fx = (b & (1u << 28)) != 0;
					for (int i = 0; i < n; i++) {
						const int px = (sx + i) & 7;
						const int p = src[fx ? 7 - px : px];
						if (p || opaque) {
							o[x + i] = (uint16_t)(pb + p);
							pr[x + i] = (uint8_t)((pr[x + i] & keep) | priority);
						}
					}
				}
				x += n;
				sx = (sx + n) & 511;
			}
		} else if (rows == 1) {   // (colscroll)
			const int sxs = v->sx[layer][0];
			for (int x = cx0; x <= cx1; x++) {
				const int sx = (x - sxs) & 511, sy = (y - v->sy[layer][(sx >> 3) & 63]) & 255;
				PIXEL(sx, sy);
			}
		} else {   // (mixed: k052109's tile_blitter, the row block by the destination's line)
			const int rowheight = 256 / rows, ri = (y & 255) / rowheight;
			const int sxs = v->sx[layer][ri];
			int scrollxi = v->raw_rowscroll[layer][ri] & ~7;
			if (!flip)
				scrollxi = -scrollxi;
			if (!flip) {   // (by tiles: a tile's column one scroll)
				int sx = (cx0 - sxs) & 511;
				for (int x = cx0; x <= cx1;) {
					int n = 8 - (sx & 7);
					if (n > cx1 - x + 1)
						n = cx1 - x + 1;
					const int ly = (y - v->sy[layer][((scrollxi + (sx & ~7)) >> 3) & 0x3f]) & 255;
					const int ti = (ly >> 3) * 64 + (sx >> 3);
					if (all || cat[ti] == want) {
						const uint32_t b = base[ti];
						const int py = (b & (2u << 28)) ? 7 - (ly & 7) : (ly & 7);
						const uint8_t *src = gfx + (b & 0x0fffffff) + py * 8;
						const uint16_t pb = pen[ti];
						const int fx = (b & (1u << 28)) != 0;
						for (int i = 0; i < n; i++) {
							const int px = (sx + i) & 7;
							const int p = src[fx ? 7 - px : px];
							if (p || opaque) {
								o[x + i] = (uint16_t)(pb + p);
								pr[x + i] = (uint8_t)((pr[x + i] & keep) | priority);
							}
						}
					}
					x += n;
					sx = (sx + n) & 511;
				}
				continue;
			}
			for (int x = cx0; x <= cx1; x++) {
				const int sx = (x - sxs) & 511, curcol = sx & ~7;
				const int sy = (y - v->sy[layer][((scrollxi + curcol) >> 3) & 0x3f]) & 255;
				PIXEL(sx, sy);
			}
		}
	}
#undef PIXEL
}

int k052109_state(k052109_t *k, void *ctx, int load, const char *tag)
{
	(void)load;
	return rbx_state_io(ctx, tag, k, (uint32_t)((const uint8_t *)&k->state_end - (const uint8_t *)k));
}
