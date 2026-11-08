/*  Konami 053250 LVC, ported to C from MAME src/mame/konami/k053250.cpp (BSD-3-Clause, copyright-holders:
    Olivier Galibert). See k053250.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "k053250.h"
#include "../../rbx.h"

#include <stdlib.h>
#include <string.h>

#define ORIENTATION_FLIP_X 1
#define ORIENTATION_FLIP_Y 2
#define ORIENTATION_SWAP_XY 4

int k053250_init(k053250_t *k, const uint8_t *rom, uint32_t rom_size, int offx, int offy)
{
	memset(k, 0, sizeof(*k));
	k->rom = rom;
	k->rom_size = rom_size;
	k->offx = offx;
	k->offy = offy;
	k->unpacked = (uint8_t *)malloc((size_t)rom_size * 2 + 1);
	if (!k->unpacked)
		return -1;
	for (uint32_t i = 0; i < rom_size; i++) {   // (unpack_nibbles)
		k->unpacked[2 * i] = rom[i] >> 4;
		k->unpacked[2 * i + 1] = rom[i] & 0xf;
	}
	k053250_reset(k);
	return 0;
}

void k053250_exit(k053250_t *k)
{
	free(k->unpacked);
	k->unpacked = NULL;
}

void k053250_reset(k053250_t *k)
{
	memset(k->ram, 0, sizeof(k->ram));
	k->page = 0;
	k->frame = -1;
	memset(k->regs, 0, sizeof(k->regs));
}

uint16_t k053250_reg_r(const k053250_t *k, int offset) { return k->regs[offset & 7]; }

void k053250_reg_w(k053250_t *k, int offset, uint16_t data, uint16_t mem_mask, int frame)
{
	offset &= 7;
	if (!(mem_mask & 0x00ff))
		return;
	if (offset == 4 && !(data & 2) && (k->regs[4] & 2) && frame != k->frame) {   // (dma(1): once a frame)
		k->frame = frame;
		memcpy(&k->ram[k->page ? 0x2800 : 0x2000], &k->ram[0], 0x1000);
		k->page ^= 1;
	}
	k->regs[offset] = (uint8_t)data;
}

uint16_t k053250_ram_r(const k053250_t *k, int offset) { return k->ram[offset % 0x3000]; }

void k053250_ram_w(k053250_t *k, int offset, uint16_t data, uint16_t mem_mask)
{
	uint16_t *p = &k->ram[offset % 0x3000];
	*p = (uint16_t)((data & mem_mask) | (*p & ~mem_mask));
}

uint16_t k053250_rom_r(const k053250_t *k, int offset)
{
	const uint32_t a = 0x80000u * k->regs[6] + 0x800u * k->regs[7] + (uint32_t)offset / 2;
	return a < k->rom_size ? k->rom[a] : 0;
}

int k053250_state(k053250_t *k, void *ctx, int load)
{
	(void)load;
	int rc = rbx_state_io(ctx, "k053250_ram", k->ram, sizeof(k->ram));
	rc |= rbx_state_io(ctx, "k053250_regs", k->regs, sizeof(k->regs));
	int32_t v[2] = { k->page, k->frame };
	rc |= rbx_state_io(ctx, "k053250_pf", v, sizeof(v));
	k->page = v[0] & 1;
	k->frame = v[1];
	return rc;
}

void k053250_snap(const k053250_t *k, k053250_snap_t *s)
{
	memcpy(s->lines, &k->ram[k->page ? 0x2800 : 0x2000], sizeof(s->lines));   // (m_buffer[m_page])
	memcpy(s->regs, k->regs, sizeof(s->regs));
}

// (pdraw_scanline32, without the priority bitmap: the GX mixer passes priority 0)
typedef struct { uint32_t *dst; int pitch, x0, y0, x1, y1; const uint32_t *pens; uint32_t pm; uint8_t *pri; uint8_t prival; int cy0, cy1; } target_t;

static void scanline(const target_t *t, uint32_t pal_off, const uint8_t *source, uint32_t src_size, int linepos,
                     int scroll, int zoom, uint32_t clipmask, uint32_t wrapmask, int orientation)
{
	enum { FP = 16, FP_HALF = 1 << (FP - 1) };
	int flip, dst_min, dst_max, dst_start, dst_length, src_fx, src_fdx;
	if (!(orientation & ORIENTATION_SWAP_XY)) {
		flip = orientation & ORIENTATION_FLIP_X;
		dst_min = t->x0;
		dst_max = t->x1;
	} else {
		flip = orientation & ORIENTATION_FLIP_Y;
		dst_min = t->y0;
		dst_max = t->y1;
	}
	if (clipmask) {
		dst_start = -scroll;
		if (dst_start > dst_max)
			return;
		dst_length = (int)clipmask + 1;
		if (zoom)
			dst_length = (dst_length << 6) / zoom;
		int end_pixel = dst_start + dst_length - 1;
		if (end_pixel < dst_min)
			return;
		if ((end_pixel -= dst_max) > 0)
			dst_length -= end_pixel;
		if (dst_length <= 0)
			return;
		src_fdx = zoom << (FP - 6);
		end_pixel = dst_min;
		if ((end_pixel -= dst_start) > 0) {
			dst_length -= end_pixel;
			dst_start = dst_min;
			src_fx = end_pixel * src_fdx + FP_HALF;
		} else
			src_fx = FP_HALF;
		if (flip) {
			dst_start = dst_max + dst_min - dst_start - (dst_length - 1);
			src_fx += (dst_length - 1) * src_fdx - 1;
			src_fdx = -src_fdx;
		}
	} else {
		dst_start = dst_min;
		dst_length = dst_max - dst_min + 1;
		src_fdx = zoom << (FP - 6);
		if (!flip)
			src_fx = (scroll + dst_min) * src_fdx + FP_HALF;
		else {
			src_fx = (scroll + dst_max) * src_fdx + FP_HALF - 1;
			src_fdx = -src_fdx;
		}
	}
	const uint32_t src_wrapmask = clipmask ? ~0u : wrapmask;
	int off, adv;
	if (!(orientation & ORIENTATION_SWAP_XY)) {   // (a line at y = linepos)
		if (linepos < t->cy0 || linepos > t->cy1)
			return;
		off = (linepos - t->y0) * t->pitch + (dst_start - t->x0);
		adv = 1;
	} else {   // (a column at x = linepos)
		if (linepos < t->x0 || linepos > t->x1)
			return;
		off = (dst_start - t->y0) * t->pitch + (linepos - t->x0);
		adv = t->pitch;
		// (the rows of a band: the column's geometry stays the whole screen's, only its writes are clipped)
		int lo = t->cy0 - dst_start, hi = t->cy1 - dst_start;
		if (lo > 0) {
			src_fx += lo * src_fdx;
			off += lo * adv;
			dst_length -= lo;
			hi -= lo;
		}
		if (dst_length > hi + 1)
			dst_length = hi + 1;
	}
	for (int n = 0; n < dst_length; n++, off += adv) {
		const uint32_t si = ((uint32_t)(src_fx >> FP)) & src_wrapmask;
		src_fx += src_fdx;
		const int pix = si < src_size ? source[si] : 0;
		if (pix) {
			t->dst[off] = t->pens[(pal_off + (uint32_t)pix) & t->pm];
			if (t->prival)
				t->pri[off] = t->prival;
		}
	}
}

void k053250_draw_rows(const k053250_t *k, const k053250_snap_t *s, uint32_t *dst, int pitch, int x0, int y0, int x1,
                       int y1, const uint32_t *pens, int pens_n, int colorbase, int vis_max_x, int vis_max_y, uint8_t *pri,
                       uint8_t prival, int cy0, int cy1)
{
	if (!k->unpacked)
		return;
	const target_t t = { dst, pitch, x0, y0, x1, y1, pens, (uint32_t)(pens_n - 1), pri, pri ? prival : 0, cy0 < y0 ? y0 : cy0,
	                     cy1 > y1 ? y1 : cy1 };
	const uint16_t *line_ram = s->lines;
	const int map_scrollx = (int16_t)(s->regs[0] << 8 | s->regs[1]) - k->offx;
	const int map_scrolly = (int16_t)(s->regs[2] << 8 | s->regs[3]) - k->offy;
	const uint8_t ctrl = s->regs[4];
	int orientation = 0, dst_height = 512, linedata_adv = 4, wrap500 = 0;
	uint32_t src_wrapmask, src_clipmask, dst_wrapmask;
	int line_start, line_end, scroll_corr, linedata_offs, passes;
	if (!(ctrl & 0x01))
		orientation |= ORIENTATION_SWAP_XY;
	if (ctrl & 0x08)
		orientation |= ORIENTATION_FLIP_X;
	if (ctrl & 0x10)
		orientation |= ORIENTATION_FLIP_Y;
	switch (ctrl >> 5) {
	case 0: src_wrapmask = src_clipmask = 0xff; dst_height = 0x100; break;
	case 1: src_wrapmask = src_clipmask = 0x1ff; break;
	case 4: src_wrapmask = src_clipmask = 0xff; wrap500 = 1; break;
	default: src_wrapmask = src_clipmask = 0x3ff; break;
	}
	if (ctrl & 0x04)
		src_clipmask = 0;
	if (!(orientation & ORIENTATION_SWAP_XY)) {
		line_start = y0;
		line_end = y1;
		scroll_corr = map_scrollx;
		linedata_offs = map_scrolly;
		if (orientation & ORIENTATION_FLIP_X)
			scroll_corr = -scroll_corr;
		if (orientation & ORIENTATION_FLIP_Y) {
			linedata_adv = -linedata_adv;
			linedata_offs += vis_max_y;
		}
		dst_wrapmask = ~0u;
		passes = 1;
	} else {
		line_start = x0;
		line_end = x1;
		scroll_corr = map_scrolly;
		linedata_offs = map_scrollx;
		if (orientation & ORIENTATION_FLIP_Y) {
			scroll_corr = 0x100 - scroll_corr;
			scroll_corr -= 2;
			linedata_offs -= 5;
		}
		if (orientation & ORIENTATION_FLIP_X) {
			linedata_adv = -linedata_adv;
			linedata_offs += vis_max_x;
		}
		if (src_clipmask) {
			dst_wrapmask = (uint32_t)dst_height - 1;
			passes = 2;
		} else {
			dst_wrapmask = ~0u;
			passes = 1;
		}
	}
	linedata_offs *= 4;
	linedata_offs &= 0x7ff;
	linedata_offs += line_start * linedata_adv;
	const uint32_t usize = k->rom_size * 2;
	const uint32_t pal_base = usize ? (uint32_t)(colorbase << 4) % (uint32_t)pens_n : 0;
	for (int line_pos = line_start; line_pos <= line_end; linedata_offs += linedata_adv, line_pos++) {
		linedata_offs &= 0x7ff;
		const int color = line_ram[linedata_offs];
		if (color == 0xffff)
			continue;
		const int offset = line_ram[linedata_offs + 1];
		if (!(color & 0xff) && !offset)
			continue;
		const uint32_t pal_off = pal_base + (uint32_t)((color & 0x1f) << 4);
		const uint32_t pix0 = ((uint32_t)offset << 8) % usize;
		const int zoom = line_ram[linedata_offs + 2];
		int scroll = (int16_t)line_ram[linedata_offs + 3];
		if (wrap500 && scroll >= 0x500)
			scroll -= 0x800;
		scroll += scroll_corr;
		scroll = (int)((uint32_t)scroll & dst_wrapmask);
		int i = passes;
		do {
			scanline(&t, pal_off, k->unpacked + pix0, usize - pix0, line_pos, scroll, zoom, src_clipmask, src_wrapmask,
			         orientation);
			scroll -= dst_height;
		} while (--i);
	}
}

void k053250_draw(const k053250_t *k, const k053250_snap_t *s, uint32_t *dst, int pitch, int x0, int y0, int x1, int y1,
                  const uint32_t *pens, int pens_n, int colorbase, int vis_max_x, int vis_max_y, uint8_t *pri, uint8_t prival)
{
	k053250_draw_rows(k, s, dst, pitch, x0, y0, x1, y1, pens, pens_n, colorbase, vis_max_x, vis_max_y, pri, prival, y0, y1);
}
