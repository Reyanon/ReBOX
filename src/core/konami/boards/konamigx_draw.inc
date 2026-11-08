/*  The GX drawing that a worker does over a band of rows (konamigx_v.c includes this once a worker: each copy its own
    state below, the frame, the z-buffers and the snapshot shared). Ported from MAME src/mame/konami/konamigx_v.cpp
    (BSD-3-Clause, copyright-holders: David Haywood, R. Belmont, Acho A. Tang, Phil Stroffolino, Olivier Galibert) and
    the devices it uses (k056832.cpp, k053246_k053247_k055673.cpp, tilemap.cpp).
    This file is part of ReBOX. GPL-2.0-or-later.
*/

// ---- the worker's state ----
static uint32_t s_pens[8192];            // the palette with the current contrast
static int s_current_brightness = -1;
static int s_shadow_mode;
static int s_last_alpha_tile_mix_code;
static uint32_t s_mix_key, s_cur_key, s_draw_seq;   // (the alpha mix code's place in the drawing order: the bands' merge)
static int s_pt_fy;   // (page_tile's tile flipped in y: draw_page's cache of a tile row)
static tcache_t s_tc[K056832_PAGE_COUNT + 1];
static int s_by0 = 0, s_by1 = VIS_H_MAX + 512;   // (the band: its first and last row of the bitmap)

static void set_brightness_value(int nb)
{
	if (s_current_brightness != nb) {
		s_current_brightness = nb;
		const float c = (float)nb / 255.0f;
		for (int i = 0; i < s_pal_n; i++) {
			const uint32_t p = s_snap->pal[i];
			const int r = clamp8((int)((float)(p >> 16 & 0xff) * c)), g = clamp8((int)((float)(p >> 8 & 0xff) * c)),
			          b = clamp8((int)((float)(p & 0xff) * c));
			s_pens[i] = (uint32_t)(r << 16 | g << 8 | b);
		}
	}
}
static void set_brightness(int layer)   // (the palette's contrast for a layer: K55 VBRI)
{
	const int bri_mode = (s_snap->k55555[K55_VBRI] >> (layer * 2)) & 3;
	set_brightness_value(bri_mode ? s_brightness[bri_mode - 1] : 0xff);
}

// ---- the K056832 layers (tilemap_draw_common, the page drawn as MAME's 64x32 tilemap of 8x8 tiles) ----
// the tile at (col, row) of a page: its pixels' row (fx: flipped), pen base, category
static inline const uint8_t *page_tile(const k056832_t *k, int page, int col, int row, int py, int *fx, int *penbase,
                                       int *category, int draw_layer)
{
	static const struct { int flips, palm1, pals2, palm2; } sm[4] = {
		{ 6, 0x3f, 0, 0x00 }, { 4, 0x0f, 2, 0x30 }, { 2, 0x03, 2, 0x3c }, { 0, 0x00, 2, 0x3f } };
	const uint16_t *mem = &k->videoram[(page << 12) + ((row * 64 + col) << 1)];
	int layer;
	if (k->layer_association) {
		layer = k->layer_assoc_with_page[page];
		if (layer == -1)
			layer = 0;   // (layer 0's palette info for unmapped pages)
	} else
		layer = draw_layer;   // (MAME: m_active_layer set to the layer being drawn)
	const int fbits = (k->regs[3] >> 6) & 3;
	int flip = (k->regs[1] >> (layer << 1)) & 0x3;   // (the tile-flip override, REG2)
	const int attr = mem[0];
	int code = mem[1];
	flip &= attr >> sm[fbits].flips & 3;
	int color = (attr & sm[fbits].palm1) | (attr >> sm[fbits].pals2 & sm[fbits].palm2);
	int priority = 0;
	if (gx.tilecb >= GX_TILECB_MW) {   // (mystwarr_v.cpp: the colour base, K055555 palette index << 4)
		const int cb = gx.tilecb == GX_TILECB_BISHI ? layer << 6 : s_snap->k55555[K55_PALBASE_A + layer] << 4;   // (bishi: 0x00 0x40 0x80 0xc0)
		if (gx.tilecb == GX_TILECB_MW || gx.tilecb == GX_TILECB_VIOSTORM) {
			const int mix_code = (gx.tilecb == GX_TILECB_MW ? attr >> 2 : attr) & 3;
			if (mix_code) {
				priority = 1;
				s_last_alpha_tile_mix_code = mix_code;
				s_mix_key = s_cur_key;
			}
		}
		if (gx.tilecb == GX_TILECB_PRE)   // (the board's own callback: gijoe ...)
			gx.pre_tilecb(layer, &code, &color, &flip, &priority);
		else if (gx.tilecb == GX_TILECB_BISHI)
			color = cb + (color & 0xf0);
		else if (gx.tilecb == GX_TILECB_MW)
			color = cb | (color >> 1 & 0x0f);
		else if (gx.tilecb == GX_TILECB_5BPP)
			color = cb | (color >> 1 & 0x1e);
		else
			color = cb | (color >> 2 & 0x0f);
		const uint32_t t = tile_index((uint32_t)code);
		const int fy = (flip >> 1) & 1;
		*fx = flip & 1;
		*penbase = color * s_tile_gran;
		*category = priority;
		s_pt_fy = fy;
		return s_tiles + (size_t)t * 64 + (size_t)((fy ? 7 - (py & 7) : (py & 7)) * 8);
	}
	// the board's tile callback (type2 / alpha / salmndr2)
	if (gx.tilecb == GX_TILECB_ALPHA || gx.tilecb == GX_TILECB_SALMNDR2) {
		const int mix_code = attr >> (gx.tilecb == GX_TILECB_ALPHA ? 6 : 4) & 3;
		if (mix_code) {
			priority = 1;
			s_last_alpha_tile_mix_code = mix_code;
			s_mix_key = s_cur_key;
		}
	}
	code = (s_snap->tilebanks[(code & 0xe000) >> 13] << 13) + (code & 0x1fff);
	decode_vmixcolor(layer, &color);
	// (MAME's tilemap: the screen flip of reg 0 with the tile flip; flipy picks the row)
	const uint32_t t = tile_index((uint32_t)code);
	const int fy = (flip >> 1) & 1;
	*fx = flip & 1;
	*penbase = color * s_tile_gran;
	*category = priority;
	s_pt_fy = fy;
	return s_tiles + (size_t)t * 64 + (size_t)((fy ? 7 - (py & 7) : (py & 7)) * 8);
}

// a tile's information from the page's row cache (the tile's information once for the 8 lines of its row: the same
// tile, the same results; page_tile called in the drawing's order)
static inline tentry_t *tc_entry(tcache_t *tcp, const k056832_t *k, int page, int col, int row, int layer)
{
	if (tcp->row != row) {
		tcp->row = row;
		memset(tcp->ok, 0, sizeof(tcp->ok));
	}
	tentry_t *const te = &tcp->e[col];
	if (!tcp->ok[col]) {
		int f, pb, ct;
		const uint8_t *s0 = page_tile(k, page, col, row, 0, &f, &pb, &ct, layer);
		te->base = s0 - (s_pt_fy ? 56 : 0);
		te->fx = (uint8_t)f;
		te->fy = (uint8_t)s_pt_fy;
		te->penbase = pb;
		te->cat = (uint8_t)ct;
		tcp->ok[col] = 1;
	}
	return te;
}

// draw_page into the pre-GX frame (MAME's tilemap draw into bitmap_ind16: opaque pens, the priority code or'ed)
static void draw_page16(const k056832_t *k, int page, int x0, int x1, int y0, int y1, int scrollx, int scrolly,
                        int all_categories, int category, int layer)
{
	const int scr_flipx = (k->regs[0] & 0x10) != 0, scr_flipy = (k->regs[0] & 0x20) != 0;
	const int xext = 2 * VIS_X0 + VIS_W, yext = 2 * VIS_Y0 + VIS_H;
	tcache_t *const tcp = &s_tc[page % (K056832_PAGE_COUNT + 1)];
	if (s_tc_side || tcp->gen != s_tc_gen || tcp->layer != layer) {
		tcp->gen = s_tc_gen;
		tcp->layer = layer;
		tcp->row = -1;
	}
	const uint32_t pm = (uint32_t)(s_pal_n - 1);
	const uint8_t pc = (uint8_t)s_pri_code;
	if (k->page_tile_mode[page] && !scr_flipx) {
		// (by bands of the lines of a tile row, as draw_page: the same pixels, a tile's information once a band)
		int y = y0 < VIS_Y0 ? VIS_Y0 : y0;
		const int ye = y1 >= VIS_Y0 + VIS_H ? VIS_Y0 + VIS_H - 1 : y1;
		const int xs = x0 < VIS_X0 ? VIS_X0 : x0, xe = x1 >= VIS_X0 + VIS_W ? VIS_X0 + VIS_W - 1 : x1;
		while (y <= ye) {
			const int sy = scr_flipy ? (yext - 1 - y + scrolly) & 255 : (y + scrolly) & 255;
			const int row = sy >> 3;
			int nb = scr_flipy ? (sy & 7) + 1 : 8 - (sy & 7);
			if (nb > ye - y + 1)
				nb = ye - y + 1;
			const int base = (y - VIS_Y0) * VIS_W - VIS_X0;
			for (int x = xs; x <= xe;) {
				const int sx = (x + scrollx) & 511, col = sx >> 3, px0 = sx & 7;
				int n = 8 - px0;
				if (n > xe - x + 1)
					n = xe - x + 1;
				const tentry_t *const te = tc_entry(tcp, k, page, col, row, layer);
				if (all_categories || te->cat == category) {
					const uint32_t pb = (uint32_t)te->penbase;
					uint16_t *o = s_frame16 + base + x;
					uint8_t *pr = s_pri + base + x;
					for (int l = 0; l < nb; l++, o += VIS_W, pr += VIS_W) {
						const int ly = (scr_flipy ? sy - l : sy + l) & 7;
						const uint8_t *src = te->base + (size_t)((te->fy ? 7 - ly : ly) * 8);
						if (te->fx) {
							for (int i = 0; i < n; i++) {
								const int p = src[7 - (px0 + i)];
								if (p || s_pre_opaque) {
									o[i] = (uint16_t)((pb + (uint32_t)p) & pm);
									pr[i] |= pc;
								}
							}
						} else {
							for (int i = 0; i < n; i++) {
								const int p = src[px0 + i];
								if (p || s_pre_opaque) {
									o[i] = (uint16_t)((pb + (uint32_t)p) & pm);
									pr[i] |= pc;
								}
							}
						}
					}
				}
				x += n;
			}
			y += nb;
		}
		return;
	}
	for (int y = y0; y <= y1; y++) {
		if (y < VIS_Y0 || y >= VIS_Y0 + VIS_H)
			continue;
		uint16_t *dst = s_frame16 + (y - VIS_Y0) * VIS_W;
		uint8_t *pri = s_pri + (y - VIS_Y0) * VIS_W;
		const int sy = scr_flipy ? (yext - 1 - y + scrolly) & 255 : (y + scrolly) & 255;
		const int row = sy >> 3;
		int x = x0 < VIS_X0 ? VIS_X0 : x0;
		const int xe = x1 >= VIS_X0 + VIS_W ? VIS_X0 + VIS_W - 1 : x1;
		if (!k->page_tile_mode[page]) {   // (update_linemap: the line a 512x1 tile)
			if (!all_categories && category != 0)
				continue;
			int fx, penbase, cat;
			(void)page_tile(k, page, sy & 63, sy >> 6, 0, &fx, &penbase, &cat, layer);
			const uint16_t *mem = &k->videoram[(page << 12) + (sy << 1)];
			const int code = mem[1];
			for (; x <= xe; x++) {
				const int lx = scr_flipx ? (xext - 1 - x + scrollx) & 511 : (x + scrollx) & 511;
				const uint32_t t = tile_index((uint32_t)((code & ~7) | (lx >> 6)));
				const int p = s_tiles[(size_t)t * 64 + (size_t)(lx & 63)];
				if (p || s_pre_opaque) {
					dst[x - VIS_X0] = (uint16_t)(((uint32_t)penbase + (uint32_t)p) & pm);
					pri[x - VIS_X0] |= pc;
				}
			}
			continue;
		}
		while (x <= xe) {
			const int sx = scr_flipx ? (xext - 1 - x + scrollx) & 511 : (x + scrollx) & 511;
			const int col = sx >> 3;
			const tentry_t *const te = tc_entry(tcp, k, page, col, row, layer);
			const uint8_t *src = te->base + (size_t)((te->fy ? 7 - (sy & 7) : (sy & 7)) * 8);
			int n = scr_flipx ? (sx & 7) + 1 : 8 - (sx & 7);
			if (n > xe - x + 1)
				n = xe - x + 1;
			if (all_categories || te->cat == category) {
				int px = sx & 7;
				const int dir = scr_flipx ? -1 : 1;
				for (int i = 0; i < n; i++, px += dir) {
					const int p = src[te->fx ? 7 - px : px];
					if (p || s_pre_opaque) {
						dst[x - VIS_X0 + i] = (uint16_t)(((uint32_t)te->penbase + (uint32_t)p) & pm);
						pri[x - VIS_X0 + i] |= pc;
					}
				}
			}
			x += n;
		}
	}
}

// draw_page into the RGB pre-GX frame (MAME's tilemap draw into bitmap_rgb32 with the screen's priority bitmap)
static void draw_page32p(const k056832_t *k, int page, int x0, int x1, int y0, int y1, int scrollx, int scrolly,
                         int all_categories, int category, int alpha, int layer)
{
	const int scr_flipx = (k->regs[0] & 0x10) != 0, scr_flipy = (k->regs[0] & 0x20) != 0;
	const int xext = 2 * VIS_X0 + VIS_W, yext = 2 * VIS_Y0 + VIS_H;
	tcache_t *const tcp = &s_tc[page % (K056832_PAGE_COUNT + 1)];
	if (s_tc_side || tcp->gen != s_tc_gen || tcp->layer != layer) {
		tcp->gen = s_tc_gen;
		tcp->layer = layer;
		tcp->row = -1;
	}
	const uint32_t pm = (uint32_t)(s_pal_n - 1);
	const uint8_t pc = (uint8_t)s_pri_code;
	if (k->page_tile_mode[page] && !scr_flipx) {   // (by bands of a tile row's lines, as draw_page16)
		int y = y0 < VIS_Y0 ? VIS_Y0 : y0;
		const int ye = y1 >= VIS_Y0 + VIS_H ? VIS_Y0 + VIS_H - 1 : y1;
		const int xs = x0 < VIS_X0 ? VIS_X0 : x0, xe = x1 >= VIS_X0 + VIS_W ? VIS_X0 + VIS_W - 1 : x1;
		while (y <= ye) {
			const int sy = scr_flipy ? (yext - 1 - y + scrolly) & 255 : (y + scrolly) & 255;
			const int row = sy >> 3;
			int nb = scr_flipy ? (sy & 7) + 1 : 8 - (sy & 7);
			if (nb > ye - y + 1)
				nb = ye - y + 1;
			const int base = (y - VIS_Y0) * VIS_W - VIS_X0;
			for (int x = xs; x <= xe;) {
				const int sx = (x + scrollx) & 511, px0 = sx & 7;
				int n = 8 - px0;
				if (n > xe - x + 1)
					n = xe - x + 1;
				const tentry_t *const te = tc_entry(tcp, k, page, sx >> 3, row, layer);
				if (all_categories || te->cat == category) {
					const uint32_t pb = (uint32_t)te->penbase;
					uint32_t *o = s_frame + base + x;
					uint8_t *pr = s_pri + base + x;
					for (int l = 0; l < nb; l++, o += VIS_W, pr += VIS_W) {
						const int ly = (scr_flipy ? sy - l : sy + l) & 7;
						const uint8_t *src = te->base + (size_t)((te->fy ? 7 - ly : ly) * 8);
						uint64_t row;
						memcpy(&row, src, 8);
						if (!row)   // (a transparent row of the tile)
							continue;
						uint8_t px[8];
						if (te->fx)
							for (int i = 0; i < n; i++)
								px[i] = src[7 - (px0 + i)];
						else
							memcpy(px, src + px0, (size_t)n);
						if (alpha < 255) {
							for (int i = 0; i < n; i++)
								if (px[i]) {
									o[i] = alpha_blend_r32(o[i], s_pens[(pb + px[i]) & pm], alpha);
									pr[i] |= pc;
								}
						} else
							for (int i = 0; i < n; i++)
								if (px[i]) {
									o[i] = s_pens[(pb + px[i]) & pm];
									pr[i] |= pc;
								}
					}
				}
				x += n;
			}
			y += nb;
		}
		return;
	}
	for (int y = y0; y <= y1; y++) {
		if (y < VIS_Y0 || y >= VIS_Y0 + VIS_H)
			continue;
		uint32_t *dst = s_frame + (y - VIS_Y0) * VIS_W;
		uint8_t *pri = s_pri + (y - VIS_Y0) * VIS_W;
		const int sy = scr_flipy ? (yext - 1 - y + scrolly) & 255 : (y + scrolly) & 255;
		const int row = sy >> 3;
		int x = x0 < VIS_X0 ? VIS_X0 : x0;
		const int xe = x1 >= VIS_X0 + VIS_W ? VIS_X0 + VIS_W - 1 : x1;
		if (!k->page_tile_mode[page]) {
			if (!all_categories && category != 0)
				continue;
			int fx, penbase, cat;
			(void)page_tile(k, page, sy & 63, sy >> 6, 0, &fx, &penbase, &cat, layer);
			const int code = k->videoram[(page << 12) + (sy << 1) + 1];
			for (; x <= xe; x++) {
				const int lx = scr_flipx ? (xext - 1 - x + scrollx) & 511 : (x + scrollx) & 511;
				const int p = s_tiles[(size_t)tile_index((uint32_t)((code & ~7) | (lx >> 6))) * 64 + (size_t)(lx & 63)];
				if (p) {
					const uint32_t c = s_pens[((uint32_t)penbase + (uint32_t)p) & pm];
					uint32_t *o = &dst[x - VIS_X0];
					*o = alpha < 255 ? alpha_blend_r32(*o, c, alpha) : c;
					pri[x - VIS_X0] |= pc;
				}
			}
			continue;
		}
		while (x <= xe) {
			const int sx = scr_flipx ? (xext - 1 - x + scrollx) & 511 : (x + scrollx) & 511;
			const tentry_t *const te = tc_entry(tcp, k, page, sx >> 3, row, layer);
			const uint8_t *src = te->base + (size_t)((te->fy ? 7 - (sy & 7) : (sy & 7)) * 8);
			int n = scr_flipx ? (sx & 7) + 1 : 8 - (sx & 7);
			if (n > xe - x + 1)
				n = xe - x + 1;
			if (all_categories || te->cat == category) {
				int px = sx & 7;
				const int dir = scr_flipx ? -1 : 1;
				for (int i = 0; i < n; i++, px += dir) {
					const int p = src[te->fx ? 7 - px : px];
					if (p) {
						const uint32_t c = s_pens[((uint32_t)te->penbase + (uint32_t)p) & pm];
						uint32_t *o = &dst[x - VIS_X0 + i];
						*o = alpha < 255 ? alpha_blend_r32(*o, c, alpha) : c;
						pri[x - VIS_X0 + i] |= pc;
					}
				}
			}
			x += n;
		}
	}
}

static void draw_page(const k056832_t *k, int page, int x0, int x1, int y0, int y1, int scrollx, int scrolly,
                      int all_categories, int category, int alpha, int layer)
{
	if (s_pri && !s_frame16) {
		draw_page32p(k, page, x0, x1, y0, y1, scrollx, scrolly, all_categories, category, alpha, layer);
		return;
	}
	if (s_frame16) {
		draw_page16(k, page, x0, x1, y0, y1, scrollx, scrolly, all_categories, category, layer);
		return;
	}
	const int scr_flipx = (k->regs[0] & 0x10) != 0, scr_flipy = (k->regs[0] & 0x20) != 0;
	// (MAME's tilemap flipped around the visible area's centre: the memory row (y0 + y1 + 1) - 1 - y + scrolly, the
	// column (x0 + x1 + 1) - 1 - x + scrollx)
	const int xext = 2 * VIS_X0 + VIS_W, yext = 2 * VIS_Y0 + VIS_H;
	// (a page's tile row cache: a draw's own, or the frame's when the tile callback has no side effect)
	tcache_t *const tcp = &s_tc[page % (K056832_PAGE_COUNT + 1)];
	if (s_tc_side || tcp->gen != s_tc_gen || tcp->layer != layer) {
		tcp->gen = s_tc_gen;
		tcp->layer = layer;
		tcp->row = -1;
	}
	if (k->page_tile_mode[page] && !scr_flipx && alpha >= 255) {
		// the usual case by bands of the lines of a tile row: a tile's information once for its lines there (the
		// pixels the same in any order: each written once by this draw)
		int y = y0 < VIS_Y0 ? VIS_Y0 : y0;
		const int ye = y1 >= VIS_Y0 + VIS_H ? VIS_Y0 + VIS_H - 1 : y1;
		const int xs = x0 < VIS_X0 ? VIS_X0 : x0, xe = x1 >= VIS_X0 + VIS_W ? VIS_X0 + VIS_W - 1 : x1;
		const uint32_t *const pens = s_pens, pm = (uint32_t)(s_pal_n - 1);
		while (y <= ye) {
			const int sy = scr_flipy ? (yext - 1 - y + scrolly) & 255 : (y + scrolly) & 255;
			const int row = sy >> 3;
			int nb = scr_flipy ? (sy & 7) + 1 : 8 - (sy & 7);   // (the lines left in this tile row)
			if (nb > ye - y + 1)
				nb = ye - y + 1;
			uint32_t *const dst0 = s_frame + (y - VIS_Y0) * VIS_W;
			for (int x = xs; x <= xe;) {
				const int sx = (x + scrollx) & 511, col = sx >> 3, px0 = sx & 7;
				int n = 8 - px0;
				if (n > xe - x + 1)
					n = xe - x + 1;
				const tentry_t *const te = tc_entry(tcp, k, page, col, row, layer);
				if (all_categories || te->cat == category) {
					const uint32_t *const pp = pens;
					const uint32_t pb = (uint32_t)te->penbase;
					uint32_t *o = dst0 + (x - VIS_X0);
					for (int l = 0; l < nb; l++, o += VIS_W) {
						const int ly = (scr_flipy ? sy - l : sy + l) & 7;
						const uint8_t *src = te->base + (size_t)((te->fy ? 7 - ly : ly) * 8);
						if (te->fx) {
							for (int i = 0; i < n; i++) {
								const int p = src[7 - (px0 + i)];
								if (p)
									o[i] = pp[(pb + (uint32_t)p) & pm];
							}
						} else {
							for (int i = 0; i < n; i++) {
								const int p = src[px0 + i];
								if (p)
									o[i] = pp[(pb + (uint32_t)p) & pm];
							}
						}
					}
				}
				x += n;
			}
			y += nb;
		}
		return;
	}
	for (int y = y0; y <= y1; y++) {
		if (y < VIS_Y0 || y >= VIS_Y0 + VIS_H)
			continue;
		uint32_t *dst = s_frame + (y - VIS_Y0) * VIS_W;
		// (MAME tilemap: the source y of a screen line, with the tilemap flip: (height - 1) - ...)
		const int sy = scr_flipy ? (yext - 1 - y + scrolly) & 255 : (y + scrolly) & 255;
		const int row = sy >> 3;
		int x = x0 < VIS_X0 ? VIS_X0 : x0;
		const int xe = x1 >= VIS_X0 + VIS_W ? VIS_X0 + VIS_W - 1 : x1;
		if (!k->page_tile_mode[page]) {   // (update_linemap: the line a 512x1 tile, 8 tiles' data side by side)
			if (!all_categories && category != 0)
				continue;
			int fx, penbase, cat;
			(void)page_tile(k, page, sy & 63, sy >> 6, 0, &fx, &penbase, &cat, layer);
			const uint16_t *mem = &k->videoram[(page << 12) + (sy << 1)];
			int code = mem[1];
			code = (s_snap->tilebanks[(code & 0xe000) >> 13] << 13) + (code & 0x1fff);
			for (; x <= xe; x++) {
				const int lx = scr_flipx ? (xext - 1 - x + scrollx) & 511 : (x + scrollx) & 511;
				const uint32_t t = tile_index((uint32_t)((code & ~7) | (lx >> 6)));
				const int p = s_tiles[(size_t)t * 64 + (size_t)(lx & 63)];
				if (!p)
					continue;
				const uint32_t c = s_pens[(penbase + p) & (s_pal_n - 1)];
				uint32_t *o = &dst[x - VIS_X0];
				*o = alpha < 255 ? alpha_blend_r32(*o, c, alpha) : c;
			}
			continue;
		}
		while (x <= xe) {
			const int sx = scr_flipx ? (xext - 1 - x + scrollx) & 511 : (x + scrollx) & 511;
			const int col = sx >> 3;
			tentry_t *const te = tc_entry(tcp, k, page, col, row, layer);
			const int fx = te->fx, penbase = te->penbase, cat = te->cat;
			const uint8_t *src = te->base + (size_t)((te->fy ? 7 - (sy & 7) : (sy & 7)) * 8);
			int n = scr_flipx ? (sx & 7) + 1 : 8 - (sx & 7);   // (the pixels left in this tile)
			if (n > xe - x + 1)
				n = xe - x + 1;
			if ((all_categories || cat == category) && alpha >= 255 && !scr_flipx) {   // (the usual case)
				const uint32_t *pens = s_pens, pm = (uint32_t)(s_pal_n - 1);
				uint32_t *o = &dst[x - VIS_X0];
				const int px0 = sx & 7;
				if (fx) {
					for (int i = 0; i < n; i++) {
						const int p = src[7 - (px0 + i)];
						if (p)
							o[i] = pens[((uint32_t)penbase + (uint32_t)p) & pm];
					}
				} else {
					for (int i = 0; i < n; i++) {
						const int p = src[px0 + i];
						if (p)
							o[i] = pens[((uint32_t)penbase + (uint32_t)p) & pm];
					}
				}
			} else if (all_categories || cat == category) {
				int px = sx & 7;
				const int dir = scr_flipx ? -1 : 1;
				for (int i = 0; i < n; i++, px += dir) {
					const int p = src[fx ? 7 - px : px];
					if (!p)   // (transparent pen 0)
						continue;
					const uint32_t c = s_pens[(penbase + p) & (s_pal_n - 1)];
					uint32_t *o = &dst[x - VIS_X0 + i];
					*o = alpha < 255 ? alpha_blend_r32(*o, c, alpha) : c;
				}
			}
			x += n;
		}
	}
}

#define DRAW_FLAG_FORCE_XYSCROLL 0x800000

static void tilemap_draw(const k056832_t *k, int layer, int flags_force_xy, int all_categories, int category, int alpha)
{
	const int rowstart = k->y[layer], colstart = k->x[layer], rowspan = k->h[layer] + 1, colspan = k->w[layer] + 1;
	int dy = k->dy[layer], dx = k->dx[layer];
	const int scrollbank = ((k->regs[0x18] >> 1) & 0xc) | (k->regs[0x18] & 3);
	int scrollmode = k->regs[0x05] >> (k->lsram_page[layer][0] << 1) & 3;
	const int height = rowspan * K056832_PAGE_HEIGHT, width = colspan * K056832_PAGE_WIDTH;
	s_draw_seq++;
	const int cminx = VIS_X0, cmaxx = VIS_X0 + VIS_W - 1, cminy = s_by0 > VIS_Y0 ? s_by0 : VIS_Y0,
	          cmaxy = s_by1 < VIS_Y0 + VIS_H - 1 ? s_by1 : VIS_Y0 + VIS_H - 1;
	int corr;
	const int flipy = k->regs[0] & 0x20;
	if (flipy) {
		corr = k->regs[0x3c / 2];
		if (corr & 0x400)
			corr |= (int)0xfffff800;
	} else
		corr = 0;
	dy += corr;
	const int ay = (int)((unsigned)(dy - k->layer_offs[layer][1]) % (unsigned)height);
	const int flipx = k->regs[0] & 0x10;
	if (flipx) {
		corr = k->regs[0x3a / 2];
		if (corr & 0x800)
			corr |= (int)0xfffff000;
	} else
		corr = 0;
	corr -= k->layer_offs[layer][0];
	if (scrollmode == 0 && flags_force_xy)
		scrollmode = 3;
	const uint16_t *p_scroll_data;
	uint16_t ram16[2];
	int line_height, sdat_wrapmask, sdat_adv;
	switch (scrollmode) {
	case 0:   // linescroll
		p_scroll_data = &k->videoram[scrollbank << 12] + (k->lsram_page[layer][1] >> 1);
		line_height = 1;
		sdat_wrapmask = 0x3ff;
		sdat_adv = 2;
		break;
	case 2:   // rowscroll
		p_scroll_data = &k->videoram[scrollbank << 12] + (k->lsram_page[layer][1] >> 1);
		line_height = 8;
		sdat_wrapmask = 0x3ff;
		sdat_adv = 16;
		break;
	default:   // xyscroll
		p_scroll_data = ram16;
		line_height = K056832_PAGE_HEIGHT;
		sdat_wrapmask = 0;
		sdat_adv = 0;
		ram16[0] = 0;
		ram16[1] = (uint16_t)dx;
		break;
	}
	if (flipy)
		sdat_adv = -sdat_adv;
	for (int r = 0; r < rowspan; r++) {
		int cliph, clipy, line_starty, line_endy, sdat_start;
		if (rowspan > 1) {
			int sy = ay, ty = r * K056832_PAGE_HEIGHT;
			if (!flipy) {
				if (r == 0 && sy > height - K056832_PAGE_HEIGHT)
					sy -= height;
				if (sy + K056832_PAGE_HEIGHT <= ty || sy - K056832_PAGE_HEIGHT >= ty)
					continue;
				if ((ty -= sy) >= 0) {
					cliph = K056832_PAGE_HEIGHT - ty;
					clipy = line_starty = ty;
					line_endy = K056832_PAGE_HEIGHT;
					sdat_start = 0;
				} else {
					cliph = K056832_PAGE_HEIGHT + ty;
					ty = -ty;
					clipy = line_starty = 0;
					line_endy = cliph;
					sdat_start = ty;
					if (scrollmode == 2) {
						sdat_start &= ~7;
						line_starty -= ty & 7;
					}
				}
			} else {
				ty += K056832_PAGE_HEIGHT;
				if (r == rowspan - 1 && sy < K056832_PAGE_HEIGHT)
					sy += height;
				if (sy + K056832_PAGE_HEIGHT <= ty || sy - K056832_PAGE_HEIGHT >= ty)
					continue;
				if ((ty -= sy) <= 0) {
					cliph = K056832_PAGE_HEIGHT + ty;
					clipy = line_starty = -ty;
					line_endy = K056832_PAGE_HEIGHT;
					sdat_start = K056832_PAGE_HEIGHT - 1;
					if (scrollmode == 2)
						sdat_start &= ~7;
				} else {
					cliph = K056832_PAGE_HEIGHT - ty;
					clipy = line_starty = 0;
					line_endy = cliph;
					sdat_start = cliph - 1;
					if (scrollmode == 2) {
						sdat_start &= ~7;
						line_starty -= ty & 7;
					}
				}
			}
		} else {
			cliph = line_endy = K056832_PAGE_HEIGHT;
			clipy = line_starty = 0;
			sdat_start = !flipy ? dy : K056832_PAGE_HEIGHT - 1;
			if (scrollmode == 2) {
				sdat_start &= ~7;
				line_starty -= dy & 7;
			}
		}
		sdat_start += r * K056832_PAGE_HEIGHT;
		sdat_start <<= 1;
		const int clipmaxy = clipy + cliph - 1;
		for (int c = 0; c < colspan; c++) {
			const int page = (((rowstart + r) & 3) << 2) + ((colstart + c) & 3);
			if (k->layer_association) {
				if (k->layer_assoc_with_page[page] != layer)
					continue;
			} else if (k->layer_assoc_with_page[page] == -1)
				continue;
			if (!k->page_tile_mode[page] && !k->linemap_enabled)   // (update_linemap: the 512x1 line mode off)
				continue;
			int last_dx = 0x100000, last_visible = 0, x0 = 0, x1 = 0, sdat_walk, line_y;
			s_cur_key = s_draw_seq << 8 | (uint32_t)(r * colspan + c);
			for (sdat_walk = sdat_start, line_y = line_starty; line_y < line_endy;
			     sdat_walk += sdat_adv, line_y += line_height) {
				int dminy = line_y, dmaxy = line_y + line_height - 1;
				if (dminy < clipy) dminy = clipy;
				if (dmaxy > clipmaxy) dmaxy = clipmaxy;
				if (dminy > cmaxy || dmaxy < cminy)
					continue;
				const int sdat_offs = sdat_walk & sdat_wrapmask;
				const int ry0 = dminy < cminy ? cminy : dminy, ry1 = dmaxy > cmaxy ? cmaxy : dmaxy;
				int ldx = ((int)p_scroll_data[sdat_offs] << 16 | (int)p_scroll_data[sdat_offs + 1]) + corr;
				if (last_dx == ldx) {
					if (last_visible)
						draw_page(k, page, x0, x1, ry0, ry1, ldx, ay, all_categories, category, alpha, layer);
					continue;
				}
				last_dx = ldx;
				int clipw, clipx;
				if (colspan > 1) {
					int sx = ldx & (width - 1), tx = c << 9;
					if (!flipx) {
						if (c == 0 && sx > width - K056832_PAGE_WIDTH)
							sx -= width;
						if (sx + K056832_PAGE_WIDTH <= tx || sx - K056832_PAGE_WIDTH >= tx) {
							last_visible = 0;
							continue;
						}
						if ((tx -= sx) <= 0) {
							clipw = K056832_PAGE_WIDTH + tx;
							clipx = 0;
						} else {
							clipw = K056832_PAGE_WIDTH - tx;
							clipx = tx;
						}
					} else {
						tx += K056832_PAGE_WIDTH;
						if (c == colspan - 1 && sx < K056832_PAGE_WIDTH)
							sx += width;
						if (sx + K056832_PAGE_WIDTH <= tx || sx - K056832_PAGE_WIDTH >= tx) {
							last_visible = 0;
							continue;
						}
						if ((tx -= sx) >= 0) {
							clipw = K056832_PAGE_WIDTH - tx;
							clipx = 0;
						} else {
							clipw = K056832_PAGE_WIDTH + tx;
							clipx = -tx;
						}
					}
				} else {
					clipw = K056832_PAGE_WIDTH;
					clipx = 0;
				}
				last_visible = 1;
				const int dminx = clipx, dmaxx = clipx + clipw - 1;
				x0 = dminx < cminx ? cminx : dminx;
				x1 = dmaxx > cmaxx ? cmaxx : dmaxx;
				if (x0 > 0 && x1 == 511)   // (Soccer Superstars: wider than 512)
					x1 = cmaxx;
				draw_page(k, page, x0, x1, ry0, ry1, ldx, ay, all_categories, category, alpha, layer);
			}
		}
	}
}

// gx_draw_basic_tilemaps
static void draw_basic_tilemaps(int mixerflags, int layer)
{
	const uint8_t disp = s_snap->k55555[K55_INPUT_ENABLES];
	if (!(disp & (1 << layer)))
		return;
	set_brightness(layer);
	const int layer2 = layer << 1, j = mixerflags >> layer2 & 3;
	int mix_internal = 0, mix_external = 0;
	if (j == GXMIX_BLEND_FORCE)
		mix_internal = mixerflags >> (layer2 + 16) & 3;
	else {
		const int von = s_vmixon >> layer2 & 3, vmx = s_vinmix >> layer2 & 3, tile_mix_code = (int)((uint32_t)mixerflags >> 30);
		mix_internal = vmx & von;
		mix_external = tile_mix_code & ~von;
	}
	const int force_xy = (mixerflags & (1 << (layer + 12))) != 0;
	int alpha = set_alpha_level(mix_internal) & 0x1ff;
	if (alpha & 0x100) {   // (additive: the alpha inverted, MAME's hack)
		alpha &= 0xff;
		if (alpha)
			alpha = ~alpha & 0xff;
	}
	int alpha2 = set_alpha_level(mix_external) & 0x1ff;
	if (alpha2 & 0x100)
		alpha2 = ~alpha2 & 0xff;
	if (alpha2 < 255) {   // (the category 1 tiles, with a mix code, in their own pass)
		tilemap_draw(&s_snap->k56832, layer, force_xy, 0, 1, alpha2);
		tilemap_draw(&s_snap->k56832, layer, force_xy, 0, 0, alpha);
	} else
		tilemap_draw(&s_snap->k56832, layer, force_xy, 1, 0, alpha);
}

// ---- the K053250 LVC as the sub layer 1 (gx_draw_basic_extended_tilemaps_1: no alpha on it, only its test) ----
static void draw_lvc(int mixerflags)
{
	if (!(s_snap->k55555[K55_INPUT_ENABLES] & 0x20))   // (K55_INP_SUB1)
		return;
	const int j = mixerflags >> 8 & 3;
	int temp1, temp2;
	if (j == 1) {   // (GXMIX_BLEND_NONE)
		temp1 = 0xff;
		temp2 = 0;
	} else if (j == GXMIX_BLEND_FORCE) {
		temp1 = 0;
		temp2 = mixerflags >> 24;
	} else {
		temp1 = s_osinmix;
		temp2 = s_osinmix >> 2 & 3;
	}
	if (temp1 != 0xff && temp2 && (set_alpha_level(temp2) & 0xff) <= 0)
		return;
	// (colorbase: m_vcblk[4] << l, l = GXSUB_4BPP's 4)
	k053250_draw(gx.k53250, &s_snap->lvc, s_frame, VIS_W, VIS_X0, VIS_Y0, VIS_X0 + VIS_W - 1, VIS_Y0 + VIS_H - 1,
	             s_pens, s_pal_n, s_vcblk[4] << 4, VIS_X0 + VIS_W - 1, VIS_Y0 + VIS_H - 1, NULL, 0);
}

// ---- the board's K053936 ROZ as the sub layer 1 (gx_draw_basic_extended_tilemaps_1: the alpha as its test) ----
static void draw_roz(int mixerflags)
{
	if (!(s_snap->k55555[K55_INPUT_ENABLES] & 0x20))   // (K55_INP_SUB1)
		return;
	const int j = mixerflags >> 8 & 3;
	int temp1, temp2, alpha = 255, k = 0;
	if (j == 1) {   // (GXMIX_BLEND_NONE)
		temp1 = 0xff;
		temp2 = 0;
	} else if (j == GXMIX_BLEND_FORCE) {
		temp1 = 0;
		temp2 = mixerflags >> 24;
	} else {
		temp1 = s_osinmix;
		temp2 = s_osinmix >> 2 & 3;
	}
	if (temp1 != 0xff && temp2) {
		alpha = set_alpha_level(temp2) & 0xff;
		if (alpha <= 0)
			return;
		if (alpha < 255)
			k = 1;
	}
	gx.sub1_roz(s_frame, VIS_X0, VIS_Y0, VIS_W, VIS_H, s_by0, s_by1, s_pens, s_vcblk[4], k, alpha);
}

// ---- the K053247 sprites (k053247_draw_single_sprite_gxcore, draw_yxloop_gx, zdrawgfxzoom32GP) ----
static void zdrawgfxzoom32gp(uint32_t code, uint32_t color, int flipx, int flipy, int sx, int sy, int scalex, int scaley,
                             int alpha, int drawmode, int zcode, int pri)
{
	enum { FP = 19 };
	if (!scalex || !scaley)
		return;
	int shdpen = s_spr_gran - 1;
	if (zcode >= 0) {
		if (drawmode == 5) {
			drawmode = 4;
			shdpen = 1;
		}
	} else if (drawmode >= 4)
		return;
	if (drawmode & 2) {
		if (!alpha)
			return;
		if (alpha == 255)
			drawmode &= ~2;
	}
	const int dw = ((scalex << 4) + 0x8000) >> 16, dh = ((scaley << 4) + 0x8000) >> 16;
	const int cl = VIS_X0, cr = VIS_X0 + VIS_W - 1, ct = s_by0 > VIS_Y0 ? s_by0 : VIS_Y0,
	          cb = s_by1 < VIS_Y0 + VIS_H - 1 ? s_by1 : VIS_Y0 + VIS_H - 1;
	int left = sx, top = sy, right = sx + dw - 1, bottom = sy + dh - 1;
	if (left > cr || right < cl || top > cb || bottom < ct || !dw || !dh)
		return;
	const int src_stride_x = (16 << FP) / dw, src_stride_y = (16 << FP) / dh;
	const int src_base_x = (cl - left > 0 ? cl - left : 0) * src_stride_x;
	const int src_base_y = (ct - top > 0 ? ct - top : 0) * src_stride_y;
	if (left < cl) left = cl;
	if (right > cr) right = cr;
	if (top < ct) top = ct;
	if (bottom > cb) bottom = cb;
	const int w = right - left + 1, h = bottom - top + 1;
	int flip_mask = 0;
	if (flipx) flip_mask |= 15;
	if (flipy) flip_mask |= 15 << 4;
	uint32_t *dst = s_frame + (top - VIS_Y0) * VIS_W + (left - VIS_X0);
	const uint8_t *src = s_sprites + (size_t)(s_nsprites ? code % s_nsprites : 0) * 256;
	const uint32_t *pal = s_pens + ((color % ((uint32_t)s_pal_n / (uint32_t)s_spr_gran)) * (uint32_t)s_spr_gran);
	const int zoff = (top - VIS_Y0) * GX_ZBUFW + (left - cl);
	uint8_t *oz = s_objzbuf + zoff;
	const uint8_t z8 = (uint8_t)zcode;
	if (zcode < 0) {   // (no shadow and z-buffering)
		for (int y = 0; y < h; y++)
			for (int x = 0; x < w; x++) {
				const int xo = (src_base_x + x * src_stride_x) >> FP, yo = (src_base_y + y * src_stride_y) >> FP;
				const uint8_t p = src[(xo + yo * 16) ^ flip_mask];
				if (!p || p >= shdpen)
					continue;
				oz[x + y * GX_ZBUFW] = z8;
				dst[x + y * VIS_W] = pal[p];
			}
	} else if (drawmode < 4) {   // (0: all pens solid, 1: solid pens only, 2 / 3: with alpha blending)
		for (int y = 0; y < h; y++)
			for (int x = 0; x < w; x++) {
				const int xo = (src_base_x + x * src_stride_x) >> FP, yo = (src_base_y + y * src_stride_y) >> FP;
				const uint8_t p = src[(xo + yo * 16) ^ flip_mask];
				if (!p || ((drawmode & 3) && p >= shdpen) || oz[x + y * GX_ZBUFW] < z8)
					continue;
				oz[x + y * GX_ZBUFW] = z8;
				uint32_t *o = &dst[x + y * VIS_W];
				if (!(drawmode & 2))
					*o = pal[p];
				else if (alpha & 0x100)   // (additive)
					*o = add_blend_r32(*o, alpha_blend_r32(pal[p], 0, alpha & 0xff));
				else
					*o = alpha_blend_r32(*o, pal[p], alpha & 0xff);
			}
	} else {   // (4: the shadow pens only)
		const uint32_t *shd = s_shadow[s_shadow_mode];
		uint8_t *sz = s_shdzbuf + zoff * 2;
		for (int y = 0; y < h; y++)
			for (int x = 0; x < w; x++) {
				const int xo = (src_base_x + x * src_stride_x) >> FP, yo = (src_base_y + y * src_stride_y) >> FP;
				const uint8_t p = src[(xo + yo * 16) ^ flip_mask];
				const int so = x * 2 + y * GX_ZBUFW * 2;
				if (p < shdpen || sz[so] < z8 || sz[so + 1] <= pri)
					continue;
				sz[so] = z8;
				sz[so + 1] = (uint8_t)pri;
				uint32_t *o = &dst[x + y * VIS_W];
				const uint32_t c = *o;
				*o = shd[(c >> 9 & 0x7c00) | (c >> 6 & 0x03e0) | (c >> 3 & 0x001f)];
			}
	}
}

static void draw_single_sprite(int code, int offs, int color, int alpha, int drawmode, int zcode, int pri)
{
	static const int xoffset[8] = { 0, 1, 4, 5, 16, 17, 20, 21 };
	static const int yoffset[8] = { 0, 2, 8, 10, 32, 34, 40, 42 };
	const uint16_t *sr = s_snap->spriteram;
	const uint8_t *k46 = s_snap->kx46_regs;
	const k055673_t *kk = &gx.k55673;
	int xa = 0, ya = 0;
	if (code & 0x01) xa += 1;
	if (code & 0x02) ya += 1;
	if (code & 0x04) xa += 2;
	if (code & 0x08) ya += 2;
	if (code & 0x10) xa += 4;
	if (code & 0x20) ya += 4;
	code &= ~0x3f;
	const int temp4 = sr[offs];
	int oy = sr[offs + 2] & 0x3ff, ox = sr[offs + 3] & 0x3ff;
	int scalex, scaley, zoomx, zoomy;
	scaley = zoomy = sr[offs + 4] & 0x3ff;
	zoomy = zoomy ? (0x400000 + (zoomy >> 1)) / zoomy : 0x800000;
	if (!(temp4 & 0x4000)) {
		scalex = zoomx = sr[offs + 5] & 0x3ff;
		zoomx = zoomx ? (0x400000 + (zoomx >> 1)) / zoomx : 0x800000;
	} else {
		zoomx = zoomy;
		scalex = scaley;
	}
	int nozoom = scalex == 0x40 && scaley == 0x40;
	int flipx = temp4 & 0x1000, flipy = temp4 & 0x2000;
	const int t6 = sr[offs + 6];
	const int mirrorx = t6 & 0x4000;
	if (mirrorx)
		flipx = 0;
	const int mirrory = t6 & 0x8000;
	const int flipscreenx = k46[5] & 0x01, flipscreeny = k46[5] & 0x02;
	if (k46[5] & 8) {   // (Escape Kids: the sprite width halved)
		zoomx >>= 1;
		ox = (ox >> 1) + 1;
		if (flipscreenx)
			ox += VIS_W;
		nozoom = 0;
	}
	if (flipscreenx) {
		ox = -ox;
		if (!mirrorx)
			flipx = !flipx;
	}
	if (flipscreeny) {
		oy = -oy;
		if (!mirrory)
			flipy = !flipy;
	}
	int wrapsize, xwraplim, ywraplim;
	if (s_snap->kx47_regs[0xc / 2] & 0x40) {
		wrapsize = 512;
		xwraplim = 512 - 64;
		ywraplim = 512 - 128;
	} else {
		wrapsize = 1024;
		xwraplim = 1024 - 384;
		ywraplim = 1024 - 512;
	}
	const int offx = (short)((k46[0] << 8) | k46[1]), offy = (short)((k46[2] << 8) | k46[3]);
	ox += kk->dx;   // (GX: the offsets applied here)
	oy -= kk->dy;
	ox = (ox - offx) & (wrapsize - 1);
	oy = (-oy - offy) & (wrapsize - 1);
	if (ox >= xwraplim) ox -= wrapsize;
	if (oy >= ywraplim) oy -= wrapsize;
	const int sz = temp4 >> 8 & 0x0f, width = 1 << (sz & 3), height = 1 << (sz >> 2 & 3);
	ox -= (zoomx * width) >> 13;   // (the coordinates are the sprite's centre)
	oy -= (zoomy * height) >> 13;
	for (int y = 0; y < height; y++) {
		const int sy = oy + ((zoomy * y + (1 << 11)) >> 12);
		int zh = (oy + ((zoomy * (y + 1) + (1 << 11)) >> 12)) - sy;
		for (int x = 0; x < width; x++) {
			const int sx = ox + ((zoomx * x + (1 << 11)) >> 12);
			int zw = (ox + ((zoomx * (x + 1) + (1 << 11)) >> 12)) - sx;
			int tempcode = code, fx, fy;
			if (mirrorx) {
				if ((!flipx) ^ ((x << 1) < width)) {
					tempcode += xoffset[(width - 1 - x + xa) & 7];
					fx = 1;
				} else {
					tempcode += xoffset[(x + xa) & 7];
					fx = 0;
				}
			} else {
				tempcode += flipx ? xoffset[(width - 1 - x + xa) & 7] : xoffset[(x + xa) & 7];
				fx = flipx != 0;
			}
			if (mirrory) {
				if ((!flipy) ^ ((y << 1) >= height)) {
					tempcode += yoffset[(height - 1 - y + ya) & 7];
					fy = 1;
				} else {
					tempcode += yoffset[(y + ya) & 7];
					fy = 0;
				}
			} else {
				tempcode += flipy ? yoffset[(height - 1 - y + ya) & 7] : yoffset[(y + ya) & 7];
				fy = flipy != 0;
			}
			if (nozoom)
				zw = zh = 0x10;
			zdrawgfxzoom32gp((uint32_t)tempcode, (uint32_t)color, fx, fy, sx, sy, zw << 12, zh << 12, alpha, drawmode,
			                 zcode, pri);
		}
	}
}

// one band of the frame (konamigx_mixer's background, wipezbuf and mixer_draw over the rows s_by0 to s_by1)
static void mixer_band(int mixerflags)
{
	const uint8_t *k55 = s_snap->k55555;
	const uint16_t *k338 = s_snap->k54338;
	// the background: the K055555's gradient (wrport1_0 & 0x20) or the K054338's solid colour
	if (s_snap->wrport1_0 & 0x20) {
		const uint32_t *pp = s_pens + ((uint32_t)k55[0] << 9);
		const int mode = k55[1];
		for (int y = s_by0 - VIS_Y0; y <= s_by1 - VIS_Y0; y++)
			for (int x = 0; x < VIS_W; x++) {
				uint32_t c;
				if (!(mode & 2))
					c = pp[0];
				else if (!(mode & 1))
					c = pp[(VIS_Y0 + y) & 511];
				else
					c = pp[(VIS_X0 + x) & 511];
				s_frame[y * VIS_W + x] = c;
			}
	} else {
		const uint32_t bg = (uint32_t)(k338[K338_REG_BGC_R] & 0xff) << 16 | k338[K338_REG_BGC_GB];
		for (int i = (s_by0 - VIS_Y0) * VIS_W; i < (s_by1 - VIS_Y0 + 1) * VIS_W; i++)
			s_frame[i] = bg;
	}
	if (!s_job.draw)
		return;
	const uint8_t disp = k55[K55_INPUT_ENABLES];
	if (mixerflags & GXMIX_NOZBUF)
		mixerflags |= GXMIX_NOSHADOW;
	if (!(mixerflags & GXMIX_NOZBUF)) {   // (wipezbuf)
		memset(s_objzbuf + (s_by0 - VIS_Y0) * GX_ZBUFW, 0xff, (size_t)(s_by1 - s_by0 + 1) * GX_ZBUFW);
		if (!(mixerflags & GXMIX_NOSHADOW))
			memset(s_shdzbuf + (s_by0 - VIS_Y0) * GX_ZBUFW * 2, 0xff, (size_t)(s_by1 - s_by0 + 1) * GX_ZBUFW * 2);
	}
	// mixer_draw
	for (int c = 0; c < s_job.n; c++) {
		const uint32_t order = s_pool[c].order;
		const int offs = s_pool[c].offs, code = s_pool[c].code;
		int color = s_pool[c].color;
		if (offs >= 0) {
			if (!(disp & K55_INP_OBJ))
				continue;
			const int drawmode = order >> 4 & 0xf;
			int alpha = 255, pri = 0, zcode = -1;
			if (drawmode & 2) {
				alpha = color >> K055555_MIXSHIFT & 3;
				if (alpha)
					alpha = set_alpha_level(alpha);
				if (alpha <= 0)
					continue;
			}
			color &= K055555_COLORMASK;
			if (drawmode >= 4)
				s_shadow_mode = order & 0x03;
			if (!(mixerflags & GXMIX_NOZBUF)) {
				zcode = order >> 16 & 0xff;
				pri = order >> 24 & 0xff;
			}
			draw_single_sprite(code, offs, color, alpha, drawmode, zcode, pri);
		} else if (code == 4 && gx.sub1_roz)
			draw_roz(mixerflags);
		else if (code == 4)
			draw_lvc(mixerflags);
		else
			draw_basic_tilemaps(mixerflags, code);
	}
}
