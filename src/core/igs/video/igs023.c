/*  IGS023 video (PGM), ported from MAME src/mame/igs/igs023_video.cpp (BSD-3-Clause, copyright-holder: David Haywood).
    See igs023.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "igs023.h"

#include <stdlib.h>
#include <string.h>

static uint32_t pow2_mask(uint32_t words)
{
	uint32_t m = 1;
	while (m * 2 <= words)
		m *= 2;
	return words ? m - 1 : 0;
}

void igs023_init(igs023_t *v, const uint8_t *gfx, uint32_t gfx_size, const uint8_t *a, uint32_t a_size, const uint8_t *b,
                 uint32_t b_size)
{
	memset(v, 0, sizeof(*v));
	v->gfx = gfx;
	v->gfx_size = gfx_size;
	v->adata = (const uint16_t *)a;
	v->amask = pow2_mask(a_size / 2);
	v->bdata = (const uint16_t *)b;
	v->bmask = pow2_mask(b_size / 2);
	const uint32_t ntx = gfx_size / 32, nbg = gfx_size / 640;
	v->tx_clear = (uint8_t *)malloc(ntx ? ntx : 1);
	v->bg_clear = (uint8_t *)malloc(nbg ? nbg : 1);
	if (v->tx_clear)
		for (uint32_t t = 0; t < ntx; t++) {   // (pen 15 everywhere: every byte 0xff)
			const uint8_t *p = gfx + t * 32;
			int clear = 1;
			for (int i = 0; i < 32 && clear; i++)
				clear = p[i] == 0xff;
			v->tx_clear[t] = (uint8_t)clear;
		}
	if (v->bg_clear)
		for (uint32_t t = 0; t < nbg; t++) {   // (pen 31 everywhere: every bit set)
			const uint8_t *p = gfx + t * 640;
			int clear = 1;
			for (int i = 0; i < 640 && clear; i++)
				clear = p[i] == 0xff;
			v->bg_clear[t] = (uint8_t)clear;
		}
}

void igs023_exit(igs023_t *v)
{
	free(v->tx_clear);
	free(v->bg_clear);
	v->tx_clear = v->bg_clear = NULL;
}

void igs023_reset(igs023_t *v)
{
	memset(v->bg, 0, sizeof(v->bg));
	memset(v->tx, 0, sizeof(v->tx));
	memset(v->rowscroll, 0, sizeof(v->rowscroll));
	memset(v->spritebuffer, 0, sizeof(v->spritebuffer));
	memset(v->zoomram, 0, sizeof(v->zoomram));
	v->bg_yscroll = v->bg_xscroll = v->bg_scale = v->tx_yscroll = v->tx_xscroll = v->ctrl = 0;
	v->nspr = 0;
}

uint16_t igs023_reg_r(igs023_t *v, uint32_t offset, int vpos)
{
	offset &= 0xffff;
	if (offset < 0x1000)
		return v->spritebuffer[offset >> 1];
	switch (offset & ~1u) {
	case 0x2000: return v->bg_yscroll;
	case 0x3000: return v->bg_xscroll;
	case 0x4000: return v->bg_scale;
	case 0x5000: return v->tx_yscroll;
	case 0x6000: return v->tx_xscroll;
	case 0x7000: return (uint16_t)vpos;
	case 0xe000: return v->ctrl;
	}
	return 0;
}

#define COMBINE(r) (r) = (uint16_t)(((r) & ~mask) | (data & mask))

void igs023_reg_w(igs023_t *v, uint32_t offset, uint16_t data, uint16_t mask)
{
	offset &= 0xffff;
	if (offset < 0x1000) {
		COMBINE(v->spritebuffer[offset >> 1]);
		return;
	}
	if (offset >= 0x1000 && offset < 0x1040) {
		COMBINE(v->zoomram[(offset - 0x1000) >> 1]);
		return;
	}
	switch (offset & ~1u) {
	case 0x2000: COMBINE(v->bg_yscroll); break;
	case 0x3000: COMBINE(v->bg_xscroll); break;
	case 0x4000: COMBINE(v->bg_scale); break;
	case 0x5000: COMBINE(v->tx_yscroll); break;
	case 0x6000: COMBINE(v->tx_xscroll); break;
	case 0xe000: COMBINE(v->ctrl); break;
	}
}

static int sprite_dma(igs023_t *v, const uint16_t *mainram)
{
	static const uint16_t ram_mask[5] = { 0xffff, 0xfbff, 0x7fff, 0xffff, 0xffff };
	if (!(v->ctrl & 1))
		return 0;
	for (int i = 0, dst = 0, offs = 0; i < 256; i++, dst += 8) {
		for (int src = 0; src < 5; src++)
			v->spritebuffer[dst + src] = mainram[offs++] & ram_mask[src];
		if ((v->spritebuffer[dst + 4] & 0x7fff) == 0)
			return 1;
	}
	return 1;
}

void igs023_get_sprites(igs023_t *v, const uint16_t *mainram)
{
	if (!sprite_dma(v, mainram))
		return;
	int n = 0;
	for (int num = 0; num < 0x1000 / 2 && n < IGS023_MAX_SPRITES; num += 8) {
		const uint16_t spr4 = v->spritebuffer[num + 4];
		if ((spr4 & 0x7fff) == 0)
			break;
		igs023_sprite *p = &v->spr[n++];
		const uint16_t spr0 = v->spritebuffer[num + 0];
		const int xgrow = spr0 >> 15;
		int xzom = (spr0 & 0x7800) >> 11;
		p->x = (int16_t)((spr0 & 0x7ff) ^ 0x400) - 0x400;
		const uint16_t spr1 = v->spritebuffer[num + 1];
		const int ygrow = spr1 >> 15;
		int yzom = (spr1 & 0x7800) >> 11;
		p->y = (int16_t)((spr1 & 0x3ff) ^ 0x200) - 0x200;
		const uint16_t spr2 = v->spritebuffer[num + 2], spr3 = v->spritebuffer[num + 3];
		p->flip = (uint8_t)((spr2 & 0x6000) >> 13);
		p->color = (spr2 & 0x1f00) >> 8;
		p->pri = (uint8_t)((spr2 >> 7) & 1);
		p->offs = ((uint32_t)(spr2 & 0x7f) << 16) | spr3;
		p->width = (spr4 & 0x7e00) >> 9;
		p->height = spr4 & 0x1ff;
		if (xgrow)
			xzom = 0x10 - xzom;
		if (ygrow)
			yzom = 0x10 - yzom;
		p->xzoom = xzom < 0x10 ? (xzom == 0xf ? 1 : ((uint32_t)v->zoomram[xzom * 2] << 16) | v->zoomram[xzom * 2 + 1]) : 0;
		p->yzoom = yzom < 0x10 ? (yzom == 0xf ? 1 : ((uint32_t)v->zoomram[yzom * 2] << 16) | v->zoomram[yzom * 2 + 1]) : 0;
		p->xgrow = (uint8_t)xgrow;
		p->ygrow = (uint8_t)ygrow;
	}
	v->nspr = n;
}

void igs023_snapshot(const igs023_t *v, igs023_snap *s, const uint16_t *pal)
{
	memcpy(s->bg, v->bg, sizeof(s->bg));
	memcpy(s->tx, v->tx, sizeof(s->tx));
	memcpy(s->rowscroll, v->rowscroll, sizeof(s->rowscroll));
	s->bg_yscroll = v->bg_yscroll;
	s->bg_xscroll = v->bg_xscroll;
	s->tx_yscroll = v->tx_yscroll;
	s->tx_xscroll = v->tx_xscroll;
	s->ctrl = v->ctrl;
	memcpy(s->pal, pal, sizeof(s->pal));
	memcpy(s->spr, v->spr, sizeof(s->spr[0]) * (size_t)v->nspr);
	s->nspr = v->nspr;
}

// ---- sprites (MAME's renderer: the data is drawn in the order it is decoded) ----

typedef struct {
	const igs023_t *v;
	uint32_t aoffset, boffset;
	int abit;
	int y0, y1;   // clip lines [y0, y1)
} spr_ctx;

static inline void draw_pix(int x, int pri, uint16_t *dest, uint8_t *destpri, uint16_t srcdat)
{
	if (x >= 0 && x < IGS023_W) {
		if (!(destpri[x] & 1)) {
			if (!pri || !(destpri[x] & 2))
				dest[x] = srcdat;
		}
		destpri[x] |= 1;
	}
}

static inline uint8_t sprite_pix(spr_ctx *c)
{
	const uint8_t d = (uint8_t)((c->v->adata[c->aoffset & c->v->amask] >> c->abit) & 0x1f);
	c->abit += 5;
	if (c->abit >= 15) {
		c->aoffset++;
		c->abit = 0;
	}
	return d;
}

static void sprite_line(spr_ctx *c, int wide, uint16_t *dest, uint8_t *destpri, uint32_t xzoom, int xgrow, int flip, int xpos,
                        int pri, int realxsize, int palt, int draw)
{
	const uint16_t *B = c->v->bdata, *A = c->v->adata;
	const uint32_t bmask = c->v->bmask, amask = c->v->amask;
	uint32_t boff = c->boffset, ao = c->aoffset;
	int ab = c->abit;
	if (!draw) {   // (only the colour data's position moves: 5 bits a pixel the mask lets through)
		int n = 0;
		for (int xcnt = 0; xcnt < wide; xcnt++)
			n += 16 - __builtin_popcount(B[boff++ & bmask]);
		ab += 5 * n;
		c->aoffset = ao + (uint32_t)(ab / 15);
		c->abit = ab % 15;
		c->boffset = boff;
		return;
	}
	int xoffset = 0, xcntdraw = 0;
	const int flipx = flip & 1;
	const uint16_t base = (uint16_t)(palt * 32);
	uint32_t aw = (uint32_t)A[ao & amask] >> ab;
	for (int xcnt = 0; xcnt < wide; xcnt++) {
		uint32_t msk = B[boff++ & bmask];
		for (int x = 0; x < 16; x++, msk >>= 1) {
			const int zb = (xzoom >> (xoffset & 0x1f)) & 1;
			xoffset++;
			if (!(msk & 1)) {
				const uint16_t src = (uint16_t)(base + (aw & 0x1f));
				ab += 5;
				if (ab >= 15) {
					ab = 0;
					aw = A[++ao & amask];
				} else
					aw >>= 5;
				if (xgrow || !zb) {
					const int n = zb ? 2 : 1;
					for (int i = 0; i < n; i++, xcntdraw++) {
						const int xd = flipx ? xpos + realxsize - xcntdraw : xpos + xcntdraw;
						if ((unsigned)xd < IGS023_W) {
							const uint8_t dp = destpri[xd];
							if (!(dp & 1)) {
								if (!pri || !(dp & 2))
									dest[xd] = src;
								destpri[xd] = (uint8_t)(dp | 1);
							}
						}
					}
				}
			} else if (xgrow || !zb)
				xcntdraw += zb ? 2 : 1;
		}
	}
	c->boffset = boff;
	c->aoffset = ao;
	c->abit = ab;
}

static inline int popcount16(uint32_t m) { return __builtin_popcount(m); }

static void sprite_line_basic(spr_ctx *c, int wide, uint16_t *dest, uint8_t *destpri, int flip, int xpos, int pri,
                              int realxsize, int palt, int draw)
{
	const uint16_t *B = c->v->bdata, *A = c->v->adata;
	const uint32_t bmask = c->v->bmask, amask = c->v->amask;
	uint32_t boff = c->boffset, ao = c->aoffset;
	int ab = c->abit;
	if (!draw) {   // (only the colour data's position moves: 5 bits a pixel the mask lets through)
		int n = 0;
		for (int xcnt = 0; xcnt < wide; xcnt++)
			n += 16 - popcount16(B[boff++ & bmask]);
		ab += 5 * n;
		c->aoffset = ao + (uint32_t)(ab / 15);
		c->abit = ab % 15;
		c->boffset = boff;
		return;
	}
	// the colour data a word at a time (3 pixels of 5 bits); the x position steps by one per mask bit
	const int step = (flip & 1) ? -1 : 1;
	int x = (flip & 1) ? xpos + realxsize : xpos;
	const uint16_t base = (uint16_t)(palt * 32);
	uint32_t aw = (uint32_t)A[ao & amask] >> ab;
	for (int xcnt = 0; xcnt < wide; xcnt++) {
		uint32_t msk = B[boff++ & bmask];
		if (msk == 0xffff) {   // (all clear)
			x += 16 * step;
			continue;
		}
		{   // the 16 pixels all off the screen: only the colour data's position moves
			const int xl = step > 0 ? x : x - 15;
			if (xl >= IGS023_W || xl + 15 < 0) {
				ab += 5 * (16 - popcount16(msk));
				if (ab >= 15) {
					ao += (uint32_t)(ab / 15);
					ab %= 15;
				}
				aw = (uint32_t)A[ao & amask] >> ab;
				x += 16 * step;
				continue;
			}
		}
		for (int k = 0; k < 16; k++, x += step, msk >>= 1) {
			if (msk & 1)
				continue;
			const uint16_t src = (uint16_t)(base + (aw & 0x1f));
			ab += 5;
			if (ab >= 15) {
				ab = 0;
				aw = A[++ao & amask];
			} else
				aw >>= 5;
			if ((unsigned)x < IGS023_W) {
				const uint8_t dp = destpri[x];
				if (!(dp & 1)) {
					if (!pri || !(dp & 2))
						dest[x] = src;
					destpri[x] = (uint8_t)(dp | 1);
				}
			}
		}
	}
	c->boffset = boff;
	c->aoffset = ao;
	c->abit = ab;
}

static void sprite_start(spr_ctx *c)
{
	const igs023_t *v = c->v;
	c->aoffset = (((uint32_t)v->bdata[(c->boffset + 1) & v->bmask] << 16) | v->bdata[c->boffset & v->bmask]) >> 2;
	c->abit = 0;
	c->boffset += 2;
}

static void sprite_zoomed(spr_ctx *c, const igs023_sprite *s, uint16_t *bm, int pitch, uint8_t *pri)
{
	const int wide = (int)s->width, high = (int)s->height, xpos = s->x, ypos = s->y, palt = (int)s->color;
	const int flipy = (s->flip >> 1) & 1;
	const uint32_t xzoom = s->xzoom, yzoom = s->yzoom;
	const int xgrow = s->xgrow, ygrow = s->ygrow;
	sprite_start(c);
	int realysize = 0;
	for (int ycnt = 0; ycnt < high; ycnt++) {
		const int zb = (yzoom >> (ycnt & 0x1f)) & 1;
		if (ygrow || !zb)
			realysize += zb ? 2 : 1;
	}
	realysize--;
	int realxsize = 0;
	for (int xcnt = 0; xcnt < wide * 16; xcnt++) {
		const int zb = (xzoom >> (xcnt & 0x1f)) & 1;
		if (xgrow || !zb)
			realxsize += zb ? 2 : 1;
	}
	realxsize--;
	int ycntdraw = 0;
	for (int ycnt = 0; ycnt < high; ycnt++) {
		const int zb = (yzoom >> (ycnt & 0x1f)) & 1;
		if (zb && ygrow) {   // the line twice
			const uint32_t ta = c->aoffset, tb = c->boffset;
			const int tbit = c->abit;
			for (int k = 0; k < 2; k++) {
				const int yd = flipy ? ypos + realysize - ycntdraw : ypos + ycntdraw;
				if (yd >= c->y0 && yd < c->y1)
					sprite_line(c, wide, bm + (yd - c->y0) * pitch, pri + (yd - c->y0) * IGS023_W, xzoom, xgrow, s->flip, xpos,
					            s->pri, realxsize, palt, 1);
				else {
					sprite_line(c, wide, NULL, NULL, xzoom, xgrow, s->flip, xpos, s->pri, realxsize, palt, 0);
					if (k == 1 && (!flipy ? yd >= c->y1 - 1 : yd < c->y0))
						return;
				}
				ycntdraw++;
				if (k == 0) {
					c->aoffset = ta;
					c->boffset = tb;
					c->abit = tbit;
				}
			}
		} else if (zb && !ygrow)
			sprite_line(c, wide, NULL, NULL, xzoom, xgrow, s->flip, xpos, s->pri, realxsize, palt, 0);
		else {
			const int yd = flipy ? ypos + realysize - ycntdraw : ypos + ycntdraw;
			if (yd >= c->y0 && yd < c->y1)
				sprite_line(c, wide, bm + (yd - c->y0) * pitch, pri + (yd - c->y0) * IGS023_W, xzoom, xgrow, s->flip, xpos,
				            s->pri, realxsize, palt, 1);
			else {
				sprite_line(c, wide, NULL, NULL, xzoom, xgrow, s->flip, xpos, s->pri, realxsize, palt, 0);
				if (!flipy ? yd >= c->y1 - 1 : yd < c->y0)
					return;
			}
			ycntdraw++;
		}
	}
}

static void sprite_basic(spr_ctx *c, const igs023_sprite *s, uint16_t *bm, int pitch, uint8_t *pri, int start)
{
	const int wide = (int)s->width, high = (int)s->height, xpos = s->x, ypos = s->y, palt = (int)s->color;
	const int flipy = (s->flip >> 1) & 1;
	if (start < 0)
		sprite_start(c);
	const int realysize = high - 1, realxsize = wide * 16 - 1;
	for (int ycnt = start < 0 ? 0 : start; ycnt < high; ycnt++) {
		const int yd = flipy ? ypos + realysize - ycnt : ypos + ycnt;
		if (yd >= c->y0 && yd < c->y1)
			sprite_line_basic(c, wide, bm + (yd - c->y0) * pitch, pri + (yd - c->y0) * IGS023_W, s->flip, xpos, s->pri,
			                  realxsize, palt, 1);
		else {
			sprite_line_basic(c, wide, NULL, NULL, s->flip, xpos, s->pri, realxsize, palt, 0);
			if (!flipy ? yd >= c->y1 - 1 : yd < c->y0)
				return;
		}
	}
}

// the first row of band [y0, y1) of an unzoomed sprite (rows are decoded in ycnt order)
static int band_start(const igs023_sprite *sp, int y0, int y1)
{
	const int high = (int)sp->height, flipy = (sp->flip >> 1) & 1;
	int st = flipy ? sp->y + high - 1 - (y1 - 1) : y0 - sp->y;
	return st < 0 ? 0 : st > high ? high : st;
}

void igs023_prepass(const igs023_t *v, igs023_snap *s)
{
	s->pre = 0;
	if (!v->adata || !v->bdata)
		return;
	const uint16_t *B = v->bdata;
	for (int i = 0; i < s->nspr; i++) {
		const igs023_sprite *sp = &s->spr[i];
		if (sp->xzoom || sp->yzoom)
			continue;
		int st[IGS023_BANDS], last = 0;
		for (int b = 0; b < IGS023_BANDS; b++) {
			st[b] = band_start(sp, IGS023_H * b / IGS023_BANDS, IGS023_H * (b + 1) / IGS023_BANDS);
			if (st[b] > last)
				last = st[b];
		}
		// sprite_start, then the rows in order: the state at each band's first row
		uint32_t boff = sp->offs;
		uint32_t ao = (((uint32_t)B[(boff + 1) & v->bmask] << 16) | B[boff & v->bmask]) >> 2;
		uint32_t ab = 0;
		boff += 2;
		const int wide = (int)sp->width;
		for (int y = 0; y <= last; y++) {
			for (int b = 0; b < IGS023_BANDS; b++)
				if (st[b] == y) {
					s->bst[i][b].ycnt = y;
					s->bst[i][b].boff = boff;
					s->bst[i][b].ao = ao;
					s->bst[i][b].ab = (int32_t)ab;
				}
			if (y == last)
				break;
			uint32_t n = 0;
			for (int k = 0; k < wide; k++)
				n += 16 - (uint32_t)__builtin_popcount(B[boff++ & v->bmask]);
			ab += 5 * n;
			ao += ab / 15;
			ab %= 15;
		}
	}
	s->pre = 1;
}

// ---- tile layers ----

void igs023_draw_lines(const igs023_t *v, const igs023_snap *s, uint16_t *bitmap, int pitch, int y0, int y1, uint8_t *pri)
{
	const int h = y1 - y0;
	uint16_t *bm = bitmap + y0 * pitch;
	for (int y = 0; y < h; y++) {
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < IGS023_W; x++)
			d[x] = 0x3ff;   // (ddp2's IGS logo needs 0x3ff)
	}
	memset(pri, 0, (size_t)IGS023_W * (size_t)h);
	// background: 64x16 tiles of 32x32, 5 bits a pixel (LSB first), colours 0x400 + 32 x palette, pen 31 clear
	if (!(s->ctrl & 0x1000) && v->gfx_size >= 640) {
		const uint32_t ntiles = v->gfx_size / 640;
		for (int y = y0; y < y1; y++) {
			uint16_t *d = bm + (y - y0) * pitch;
			uint8_t *pp = pri + (y - y0) * IGS023_W;
			const int sy = (y + s->bg_yscroll) & 0x1ff;
			const int sx0 = s->bg_xscroll + s->rowscroll[y];
			for (int x = 0; x < IGS023_W;) {
				const int sx = (sx0 + x) & 0x7ff;
				const int col = sx >> 5, row = sy >> 5;
				const uint16_t *t = &s->bg[(row * 64 + col) * 2];
				uint32_t code = t[0];
				if (code >= ntiles)
					code %= ntiles;
				if (v->bg_clear && v->bg_clear[code]) {
					x += 32 - (sx & 31);
					continue;
				}
				const int color = 0x400 + ((t[1] & 0x3e) >> 1) * 32;
				const int fx = (t[1] >> 6) & 1, fy = (t[1] >> 7) & 1;
				const int ty = fy ? 31 - (sy & 31) : (sy & 31);
				const uint32_t base = code * 5120 + (uint32_t)ty * 160;
				int px = sx & 31;
				for (; px < 32 && x < IGS023_W; px++, x++) {
					const int tx = fx ? 31 - px : px;
					const uint32_t bit = base + (uint32_t)tx * 5;
					const uint8_t *b = v->gfx + (bit >> 3);
					const int pen = ((b[0] | (b[1] << 8)) >> (bit & 7)) & 31;
					if (pen != 31) {
						d[x] = (uint16_t)(color + pen);
						pp[x] = 2;
					}
				}
			}
		}
	}
	// sprites: the list from its end (the first entries end up on top)
	spr_ctx c;
	c.v = v;
	c.y0 = y0;
	c.y1 = y1;
	int band = -1;   // (a band of igs023_prepass: its rows start from the saved state)
	if (s->pre)
		for (int b = 0; b < IGS023_BANDS; b++)
			if (y0 == IGS023_H * b / IGS023_BANDS && y1 == IGS023_H * (b + 1) / IGS023_BANDS)
				band = b;
	if (v->adata && v->bdata) {
		for (int i = s->nspr - 1; i >= 0; i--) {
			const igs023_sprite *sp = &s->spr[i];
			if ((s->ctrl & 0x2000) && !sp->pri)
				continue;
			c.boffset = sp->offs;
			if (!sp->xzoom && !sp->yzoom) {
				int start = -1;
				if (band >= 0) {
					c.boffset = s->bst[i][band].boff;
					c.aoffset = s->bst[i][band].ao;
					c.abit = s->bst[i][band].ab;
					start = s->bst[i][band].ycnt;
				}
				sprite_basic(&c, sp, bm, pitch, pri, start);
			}
			else
				sprite_zoomed(&c, sp, bm, pitch, pri);
		}
	}
	// text: 64x32 tiles of 8x8, 4 bits a pixel, colours 0x800 + 16 x palette, pen 15 clear
	if (!(s->ctrl & 0x0800) && v->gfx_size >= 32) {
		const uint32_t ntiles = v->gfx_size / 32;
		for (int y = y0; y < y1; y++) {
			uint16_t *d = bm + (y - y0) * pitch;
			const int sy = (y + s->tx_yscroll) & 0xff;
			for (int x = 0; x < IGS023_W;) {
				const int sx = (x + s->tx_xscroll) & 0x1ff;
				const uint16_t *t = &s->tx[((sy >> 3) * 64 + (sx >> 3)) * 2];
				uint32_t code = t[0];
				if (code >= ntiles)
					code %= ntiles;
				if (v->tx_clear && v->tx_clear[code]) {
					x += 8 - (sx & 7);
					continue;
				}
				const int color = 0x800 + ((t[1] & 0x3e) >> 1) * 16;
				const int fx = (t[1] >> 6) & 1, fy = (t[1] >> 7) & 1;
				const int ty = fy ? 7 - (sy & 7) : (sy & 7);
				const uint8_t *row = v->gfx + code * 32 + ty * 4;
				int px = sx & 7;
				for (; px < 8 && x < IGS023_W; px++, x++) {
					const int tx = fx ? 7 - px : px;
					const int pen = (row[tx >> 1] >> ((tx & 1) * 4)) & 15;
					if (pen != 15)
						d[x] = (uint16_t)(color + pen);
				}
			}
		}
	}
}

void igs023_draw(const igs023_t *v, const igs023_snap *s, uint16_t *bitmap, int pitch)
{
	static uint8_t pri[IGS023_W * IGS023_H];
	igs023_draw_lines(v, s, bitmap, pitch, 0, IGS023_H, pri);
}
