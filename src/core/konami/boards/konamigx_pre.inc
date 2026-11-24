/*  The pre-GX sprite drawing (k053247 sprites_draw and drawgfx with the priority bitmap), included once for each
    worker of the pre-GX bands (konamigx_v.c: the names of the copies renamed). Part of konamigx_v.c.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef PRE_NOPRI
#define PRE_NOPRI 0xffffffffu   // (pre_drawgfx: drawn without the priority bitmap)
#endif
static int s_pby0 = 0, s_pby1 = VIS_H_MAX + 512;   // (pre_sprites_core's band: its first and last row)

// PIXEL_OP_REBASE_TRANSTABLE16_PRIORITY (the shadow table: pen + entries)
static inline void pre_pixel(uint16_t *d, uint8_t *pri, uint32_t src, const uint8_t *pentable, uint32_t color,
                             uint32_t pmask)
{
	const uint32_t entry = pentable[src];
	if (entry == DM_NONE)
		return;
	const uint8_t pd = *pri;
	if (entry == DM_SOURCE) {
		if (!((1u << (pd & 0x1f)) & pmask))
			*d = (pd & 0xc0) == (DM_SHADOW_PRI << 6) ? (uint16_t)(color + src + (uint32_t)s_pal_n) : (uint16_t)(color + src);
		*pri = 31;
	} else if (!(pd & 0x80) && !((1u << (pd & 0x1f)) & pmask)) {
		if (*d < s_pal_n)   // (shadowtable: the normal pens to their shadows, the rest as they are)
			*d = (uint16_t)(*d + s_pal_n);
		*pri = (uint8_t)(pd | entry << 6);
	}
}

// PIXEL_OP_REMAP_TRANSTABLE32_PRIORITY (the RGB pre-GX frame; shadowtable: the palette's of the shadow mode)
static int s_pre_shadow;
static inline uint32_t rgb15(uint32_t c) { return (c >> 9 & 0x7c00) | (c >> 6 & 0x03e0) | (c >> 3 & 0x001f); }
static inline void pre_pixel32(uint32_t *d, uint8_t *pri, uint32_t src, const uint8_t *pentable, const uint32_t *pal,
                               uint32_t pmask)
{
	const uint32_t entry = pentable[src];
	if (entry == DM_NONE)
		return;
	const uint8_t pd = *pri;
	const uint32_t *shd = s_shadow[s_pre_shadow];
	if (entry == DM_SOURCE) {
		if (!((1u << (pd & 0x1f)) & pmask))
			*d = (pd & 0xc0) == (DM_SHADOW_PRI << 6) ? shd[rgb15(pal[src])] : pal[src];
		*pri = 31;
	} else if (!(pd & 0x80) && !((1u << (pd & 0x1f)) & pmask)) {
		*d = shd[rgb15(*d)];
		*pri = (uint8_t)(pd | entry << 6);
	}
}

// drawgfx_core / drawgfxzoom_core with the priority bitmap (the clip: the visible area)
static int s_pclip[4];   // (pre_drawgfx's clip: left, right, top, bottom)
static void pre_drawgfx(uint32_t code, uint32_t color, int flipx, int flipy, int destx, int desty, uint32_t scalex,
                        uint32_t scaley, int zoom, uint32_t pmask, const uint8_t *pentable)
{
	const int cl = s_pclip[0], cr = s_pclip[1], ct = s_pclip[2], cb = s_pclip[3];
	if (cl > cr || ct > cb)
		return;
	// (PRE_NOPRI: gfx_element::transtable / zoom_transtable, no priority bitmap: k051960_sprites_draw with a priority
	// range draws back to front over everything; with 1 << 31 an earlier, farther sprite hid the nearer ones, issue #2)
	pmask = pmask == PRE_NOPRI ? 0 : pmask | 1u << 31;
	const uint32_t ncolors = (uint32_t)s_pal_n / (uint32_t)s_spr_gran;
	color = (uint32_t)s_spr_gran * (ncolors ? color % ncolors : 0);
	const uint8_t *src = s_sprites + (size_t)(s_nsprites ? code % s_nsprites : 0) * 256;
	if (!zoom) {   // (drawgfx_core)
		int destendx = destx + 15, srcx = 0, srcy = 0;
		if (destx > cr || destendx < cl)
			return;
		if (destx < cl) {
			srcx = cl - destx;
			destx = cl;
		}
		if (destendx > cr)
			destendx = cr;
		int destendy = desty + 15;
		if (desty > cb || destendy < ct)
			return;
		if (desty < ct) {
			srcy = ct - desty;
			desty = ct;
		}
		if (destendy > cb)
			destendy = cb;
		if (flipx)
			srcx = 15 - srcx;
		int dy = 16;
		if (flipy) {
			srcy = 15 - srcy;
			dy = -16;
		}
		const uint8_t *s = src + srcy * 16 + srcx;
		const int dx = flipx ? -1 : 1;
		for (int y = desty; y <= destendy; y++, s += dy) {
			const int o = (y - VIS_Y0) * VIS_W + (destx - VIS_X0);
			uint8_t *p = s_pri + o;
			const uint8_t *ss = s;
			if (!s_frame16) {
				uint32_t *d = s_frame + o;
				for (int x = destx; x <= destendx; x++, d++, p++, ss += dx)
					pre_pixel32(d, p, *ss, pentable, s_pens + color, pmask);
				continue;
			}
			uint16_t *d = s_frame16 + o;
			for (int x = destx; x <= destendx; x++, d++, p++, ss += dx)
				pre_pixel(d, p, *ss, pentable, color, pmask);
		}
		return;
	}
	// (drawgfxzoom_core)
	const uint32_t dstwidth = (scalex * 16 + 0x8000) >> 16, dstheight = (scaley * 16 + 0x8000) >> 16;
	if (dstwidth < 1 || dstheight < 1)
		return;
	int32_t dx = (16 << 16) / (int32_t)dstwidth, dy = (16 << 16) / (int32_t)dstheight;
	int destendx = destx + (int)dstwidth - 1;
	if (destx > cr || destendx < cl)
		return;
	int32_t srcx = 0, srcy = 0;
	if (destx < cl) {
		srcx = (cl - destx) * dx;
		destx = cl;
	}
	if (destendx > cr)
		destendx = cr;
	int destendy = desty + (int)dstheight - 1;
	if (desty > cb || destendy < ct)
		return;
	if (desty < ct) {
		srcy = (ct - desty) * dy;
		desty = ct;
	}
	if (destendy > cb)
		destendy = cb;
	if (flipx) {
		srcx = ((int32_t)dstwidth - 1) * dx - srcx;
		dx = -dx;
	}
	if (flipy) {
		srcy = ((int32_t)dstheight - 1) * dy - srcy;
		dy = -dy;
	}
	for (int y = desty; y <= destendy; y++, srcy += dy) {
		const int o = (y - VIS_Y0) * VIS_W + (destx - VIS_X0);
		uint8_t *p = s_pri + o;
		const uint8_t *row = src + (srcy >> 16) * 16;
		int32_t cx = srcx;
		if (!s_frame16) {
			uint32_t *d = s_frame + o;
			for (int x = destx; x <= destendx; x++, d++, p++, cx += dx)
				pre_pixel32(d, p, row[cx >> 16], pentable, s_pens + color, pmask);
			continue;
		}
		uint16_t *d = s_frame16 + o;
		for (int x = destx; x <= destendx; x++, d++, p++, cx += dx)
			pre_pixel(d, p, row[cx >> 16], pentable, color, pmask);
	}
}

static void pre_sprites_core(void)
{
	const uint16_t *ram = s_snap->spriteram;
	const uint8_t *k46 = s_snap->kx46_regs;
	const uint16_t *k47 = s_snap->kx47_regs;
	uint8_t drawmode_table[256], shadowmode_table[256];
	memset(drawmode_table, DM_SOURCE, sizeof(drawmode_table));
	drawmode_table[0] = DM_NONE;
	memset(shadowmode_table, DM_SHADOW, sizeof(shadowmode_table));
	shadowmode_table[0] = DM_NONE;
	const int shdmask = s_frame16 ? 0 : 3;   // (bitmap_ind16 with shadows: the default ones; rgb32 with highlights: all)
	int sortedlist[256], count = 0;
	for (int offs = 0; offs < 0x800; offs += 8)   // (m_z_rejection -1)
		if (ram[offs] & 0x8000)
			sortedlist[count++] = offs;
	const int w = count, h = count - 1;
	for (int y = 0; y < h; y++) {
		int offs = sortedlist[y], zcode = ram[offs] & 0xff;
		for (int x = y + 1; x < w; x++) {
			const int temp = sortedlist[x], code = ram[temp] & 0xff;
			if (!(k47[0xc / 2] & 0x10) ? zcode <= code : zcode >= code) {
				zcode = code;
				sortedlist[x] = offs;
				sortedlist[y] = offs = temp;
			}
		}
	}
	const int flipscreenx = k46[5] & 0x01, flipscreeny = k46[5] & 0x02;
	s_pclip[0] = VIS_X0;
	s_pclip[1] = VIS_X0 + VIS_W - 1;
	s_pclip[2] = s_pby0 > VIS_Y0 ? s_pby0 : VIS_Y0;   // (the band's rows)
	s_pclip[3] = s_pby1 < VIS_Y0 + VIS_H - 1 ? s_pby1 : VIS_Y0 + VIS_H - 1;
	for (int n = count - 1; n >= 0; n--) {
		const int offs = sortedlist[n];
		int code = ram[offs + 1], color = ram[offs + 6], shadow = color, primask = 0;
		gx.pre_sprcb(&code, &color, &primask);
		// (k053247_draw_single_sprite_gxcore, non-GX)
		int xa = 0, ya = 0;
		if (code & 0x01) xa += 1;
		if (code & 0x02) ya += 1;
		if (code & 0x04) xa += 2;
		if (code & 0x08) ya += 2;
		if (code & 0x10) xa += 4;
		if (code & 0x20) ya += 4;
		code &= ~0x3f;
		const int temp4 = ram[offs];
		int oy = ram[offs + 2] & 0x3ff, ox = ram[offs + 3] & 0x3ff;
		int scaley = ram[offs + 4] & 0x3ff, zoomy = scaley, scalex, zoomx;
		zoomy = zoomy ? (0x400000 + (zoomy >> 1)) / zoomy : 0x800000;
		if (!(temp4 & 0x4000)) {
			scalex = zoomx = ram[offs + 5] & 0x3ff;
			zoomx = zoomx ? (0x400000 + (zoomx >> 1)) / zoomx : 0x800000;
		} else {
			zoomx = zoomy;
			scalex = scaley;
		}
		int nozoom = scalex == 0x40 && scaley == 0x40;
		int flipx = temp4 & 0x1000, flipy = temp4 & 0x2000;
		const int t6 = ram[offs + 6], mirrorx = t6 & 0x4000, mirrory = t6 & 0x8000;
		if (mirrorx)
			flipx = 0;
		if (k46[5] & 8) {   // (objset1 bit 3: half width)
			zoomx >>= 1;
			ox = (ox >> 1) + 1;
			if (flipscreenx)
				ox += gx.pre_screen_w;
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
		if (k47[0xc / 2] & 0x40) {
			wrapsize = 512;
			xwraplim = 512 - 64;
			ywraplim = 512 - 128;
		} else {
			wrapsize = 1024;
			xwraplim = 1024 - 384;
			ywraplim = 1024 - 512;
		}
		const int offx = (int16_t)(k46[0] << 8 | k46[1]), offy = (int16_t)(k46[2] << 8 | k46[3]);
		ox = (ox - offx) & (wrapsize - 1);
		oy = (-oy - offy) & (wrapsize - 1);
		if (ox >= xwraplim)
			ox -= wrapsize;
		if (oy >= ywraplim)
			oy -= wrapsize;
		const int sz = temp4 >> 8 & 0x0f, width = 1 << (sz & 3), height = 1 << (sz >> 2 & 3);
		ox += gx.k55673.dx;   // (the non-GX ones: the offsets here)
		oy -= gx.k55673.dy;
		ox -= (zoomx * width) >> 13;
		oy -= (zoomy * height) >> 13;
		const uint8_t *whichtable = drawmode_table;
		if (color == -1) {
			if (shdmask < 0)
				continue;
			color = 0;
			shadow = -1;
			whichtable = shadowmode_table;
			s_pre_shadow = 0;   // (set_shadow_mode(0))
		} else {
			shadow = (color & 0x20000000) ? color >> 20 : shadow >> 10;   // (K053247_CUSTOMSHADOW, K053247_SHDSHIFT)
			if (shadow &= 3)
				s_pre_shadow = (shadow - 1) & shdmask;
		}
		color &= 0xffff;
		drawmode_table[s_spr_gran - 1] = shadow ? DM_SHADOW : DM_SOURCE;
		// (k053247_draw_yxloop_gx, non-GX)
		static const int xoffset[8] = { 0, 1, 4, 5, 16, 17, 20, 21 };
		static const int yoffset[8] = { 0, 2, 8, 10, 32, 34, 40, 42 };
		for (int y = 0; y < height; y++) {
			const int sy = oy + ((zoomy * y + (1 << 11)) >> 12), zh = (oy + ((zoomy * (y + 1) + (1 << 11)) >> 12)) - sy;
			for (int x = 0; x < width; x++) {
				const int sx = ox + ((zoomx * x + (1 << 11)) >> 12), zw = (ox + ((zoomx * (x + 1) + (1 << 11)) >> 12)) - sx;
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
				// (prio_zoom_transtable: the non-zoom case when the scale is exactly 1)
				const uint32_t scx = (uint32_t)zw << 12, scy = (uint32_t)zh << 12;
				const int z = !nozoom && !(scx == 0x10000 && scy == 0x10000);
				pre_drawgfx((uint32_t)tempcode, (uint32_t)color, fx, fy, sx, sy, scx, scy, z, (uint32_t)primask, whichtable);
				if (mirrory && height == 1)   // (Simpsons shadows)
					pre_drawgfx((uint32_t)tempcode, (uint32_t)color, fx, !fy, sx, sy, scx, scy, z, (uint32_t)primask, whichtable);
			}
		}
	}
}
