/*  Atari motion objects. Ported from MAME src/mame/atari/atarimo.cpp (BSD-3-Clause, copyright-holders: Aaron Giles)
    and src/devices/video/sprite.h (draw_async), src/emu/drawgfx.cpp (transpen_raw).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "atarimo.h"

#include <stdlib.h>
#include <string.h>

static int compute_log(int value)
{
	int log = 0;
	if (value == 0)
		return -1;
	while (!(value & 1))
		log++, value >>= 1;
	if (value != 1)
		return -1;
	return log;
}

static int round_to_powerof2(int value)
{
	int log = 0;
	if (value == 0)
		return 1;
	while ((value >>= 1) != 0)
		log++;
	return 1 << (log + 1);
}

static void param_set(amo_param *p, const uint16_t in[4])
{
	p->word = p->shift = 0;
	p->mask = 0;
	for (int i = 0; i < 4; i++)
		if (in[i]) {
			p->word = (uint8_t)i;
			uint16_t t = in[i];
			while (!(t & 1)) {
				p->shift++;
				t >>= 1;
			}
			p->mask = t;
			return;
		}
}

static inline int ext(const amo_param *p, const uint16_t *e) { return (e[p->word] >> p->shift) & p->mask; }

int amo_init(amo_t *m, const amo_config *c, int tilew, int tileh)
{
	uint32_t *keep = m->codelookup;
	uint8_t *keepg = m->gfxlookup;
	memset(m, 0, sizeof(*m));
	m->c = *c;
	param_set(&m->linkm, c->link);
	param_set(&m->codem, c->code);
	param_set(&m->colorm, c->color);
	param_set(&m->xposm, c->xpos);
	param_set(&m->yposm, c->ypos);
	param_set(&m->widthm, c->width);
	param_set(&m->heightm, c->height);
	param_set(&m->hflipm, c->hflip);
	param_set(&m->vflipm, c->vflip);
	param_set(&m->priom, c->priority);
	param_set(&m->neighm, c->neighbor);
	param_set(&m->absm, c->absolute);
	param_set(&m->specm, c->special);
	m->tilew = tilew;
	m->tileh = tileh;
	m->tilexshift = compute_log(tilew);
	m->tileyshift = compute_log(tileh);
	m->bitmapw = round_to_powerof2(m->xposm.mask);
	m->bitmaph = round_to_powerof2(m->yposm.mask);
	m->bitmapxmask = m->bitmapw - 1;
	m->bitmapymask = m->bitmaph - 1;
	m->entrycount = round_to_powerof2(m->linkm.mask);
	m->entrybits = compute_log(m->entrycount);
	m->slipshift = c->slipheight ? compute_log(c->slipheight) : 0;
	m->sliprammask = (m->bitmaph >> m->slipshift) - 1;
	if (m->c.maxperline == 0)
		m->c.maxperline = AMO_MAX_PER_BANK;
	m->codesize = round_to_powerof2(m->codem.mask);
	m->codelookup = (uint32_t *)realloc(keep, sizeof(uint32_t) * (size_t)m->codesize);
	m->gfxlookup = (uint8_t *)realloc(keepg, (size_t)(m->codesize / 256 > 0 ? m->codesize / 256 : 1));
	if (!m->codelookup || !m->gfxlookup)
		return -1;
	for (int i = 0; i < m->codesize; i++)
		m->codelookup[i] = (uint32_t)i;
	for (int i = 0; i < 256; i++)
		m->colorlookup[i] = (uint8_t)i;
	memset(m->gfxlookup, 0, (size_t)(m->codesize / 256 > 0 ? m->codesize / 256 : 1));
	m->activelast = m->activelist;
	return 0;
}

void amo_exit(amo_t *m)
{
	free(m->codelookup);
	free(m->gfxlookup);
	m->codelookup = NULL;
	m->gfxlookup = NULL;
}

static void build_active_list(amo_t *m, const uint16_t *spriteram, int link)
{
	const uint16_t *bankbase = &spriteram[m->bank << (m->entrybits + 2)];
	uint16_t *current = m->activelist;
	uint8_t visited[AMO_MAX_PER_BANK];
	memset(visited, 0, sizeof(visited));
	for (int i = 0; i < m->c.maxperline && !visited[link]; i++) {
		uint16_t *modata = current;
		if (!m->c.split) {
			const uint16_t *src = &bankbase[link * 4];
			*current++ = src[0];
			*current++ = src[1];
			*current++ = src[2];
			*current++ = src[3];
		} else {
			const uint16_t *src = &bankbase[link];
			*current++ = src[0u << m->entrybits];
			*current++ = src[1u << m->entrybits];
			*current++ = src[2u << m->entrybits];
			*current++ = src[3u << m->entrybits];
		}
		visited[link] = 1;
		if (m->c.linked)
			link = ext(&m->linkm, modata);
		else
			link = (link + 1) & m->linkm.mask;
	}
	m->activelast = current;
}

// gfx_element::transpen_raw, clipped to the cliprect
static void transpen_raw(const amo_t *m, const rbx_gfx *g, uint32_t code, int color, int flipx, int flipy, int sx,
                         int sy, uint16_t *bm, int pitch, int minx, int maxx, int miny, int maxy)
{
	if (!g || !g->count)
		return;
	code %= g->count;
	if (!g->used[code] && m->c.transpen == 0)
		return;
	const int w = g->width, h = g->height;
	int x0 = 0, x1 = w - 1, y0 = 0, y1 = h - 1;
	if (sx < minx) x0 = minx - sx;
	if (sx + w - 1 > maxx) x1 = maxx - sx;
	if (sy < miny) y0 = miny - sy;
	if (sy + h - 1 > maxy) y1 = maxy - sy;
	if (x0 > x1 || y0 > y1)
		return;
	const uint8_t *pix = g->pix + (size_t)code * (size_t)(w * h);
	const int tp = m->c.transpen;
	for (int y = y0; y <= y1; y++) {
		const uint8_t *src = pix + (flipy ? h - 1 - y : y) * w;
		uint16_t *d = bm + (sy + y) * pitch + sx;
		if (!flipx) {
			for (int x = x0; x <= x1; x++)
				if (src[x] != tp)
					d[x] = (uint16_t)(color + src[x]);
		} else {
			for (int x = x0; x <= x1; x++) {
				const uint8_t p = src[w - 1 - x];
				if (p != tp)
					d[x] = (uint16_t)(color + p);
			}
		}
	}
}

static void render_object(amo_t *m, uint16_t *bm, int pitch, int bw, int bh, int minx, int maxx, int miny, int maxy,
                          const uint16_t *entry)
{
	const int rawcode = ext(&m->codem, entry);
	const int gi = m->gfxlookup[rawcode >> 8];
	const rbx_gfx *gfx = m->gfx[gi];
	int code = (int)m->codelookup[rawcode];
	int color = m->colorlookup[ext(&m->colorm, entry)];
	int xpos = ext(&m->xposm, entry) + m->xoffset;
	int ypos = -ext(&m->yposm, entry);
	const int hflip = ext(&m->hflipm, entry);
	const int vflip = ext(&m->vflipm, entry);
	const int width = ext(&m->widthm, entry) + 1;
	const int height = ext(&m->heightm, entry) + 1;
	const int priority = ext(&m->priom, entry);

	color = (color * m->granularity[gi]) | (priority << AMO_PRIORITY_SHIFT);
	color += m->c.palettebase;
	if (!ext(&m->absm, entry)) {
		xpos -= m->xscroll;
		ypos -= m->yscroll;
	}
	ypos -= height << m->tileyshift;
	if (m->next_xpos != 123456)
		xpos = m->next_xpos;
	m->next_xpos = 123456;
	if (ext(&m->neighm, entry) != 0) {
		if (!m->c.nextneighbor)
			xpos = m->last_xpos + m->tilew;
		else
			m->next_xpos = xpos + m->tilew;
	}
	m->last_xpos = xpos;
	xpos &= m->bitmapxmask;
	ypos &= m->bitmapymask;
	if (xpos >= bw)
		xpos -= m->bitmapw;
	if (ypos >= bh)
		ypos -= m->bitmaph;
	if (m->specm.mask != 0 && ext(&m->specm, entry) == m->c.specialvalue)
		return;
	int xadv = m->tilew;
	if (hflip) {
		xpos += (width - 1) << m->tilexshift;
		xadv = -xadv;
	}
	int yadv = m->tileh;
	if (vflip) {
		ypos += (height - 1) << m->tileyshift;
		yadv = -yadv;
	}
	if (!m->c.swapxy) {
		for (int y = 0, sy = ypos; y < height; y++, sy += yadv) {
			if (sy <= miny - m->tileh) {
				code += width;
				continue;
			} else if (sy > maxy)
				break;
			for (int x = 0, sx = xpos; x < width; x++, sx += xadv, code++) {
				if (sx <= -minx - m->tilew || sx > maxx)
					continue;
				transpen_raw(m, gfx, (uint32_t)code, color, hflip, vflip, sx, sy, bm, pitch, minx, maxx, miny, maxy);
			}
		}
	} else {
		for (int x = 0, sx = xpos; x < width; x++, sx += xadv) {
			if (sx <= minx - m->tilew) {
				code += height;
				continue;
			} else if (sx > maxx)
				break;
			for (int y = 0, sy = ypos; y < height; y++, sy += yadv, code++) {
				if (sy <= -miny - m->tileh || sy > maxy)
					continue;
				transpen_raw(m, gfx, (uint32_t)code, color, hflip, vflip, sx, sy, bm, pitch, minx, maxx, miny, maxy);
			}
		}
	}
}

void amo_draw(amo_t *m, const uint16_t *spriteram, const uint16_t *slipram, uint16_t *bm, int pitch, int minx, int maxx,
              int miny, int maxy)
{
	for (int y = miny; y <= maxy; y++)   // (clear the cliprect)
		for (int x = minx; x <= maxx; x++)
			bm[y * pitch + x] = 0xffff;
	const int bw = maxx + 1, bh = maxy + 1;
	int startband = ((miny + m->yscroll - m->c.slipoffset) & m->bitmapymask) >> m->slipshift;
	int stopband = ((maxy + m->yscroll - m->c.slipoffset) & m->bitmapymask) >> m->slipshift;
	if (startband > stopband)
		startband -= m->bitmaph >> m->slipshift;
	if (m->slipshift == 0)
		stopband = startband;
	for (int band = startband; band <= stopband; band++) {
		int cminy = miny, cmaxy = maxy;
		int link = 0;
		if (m->slipshift != 0) {
			link = (slipram[band & m->sliprammask] >> m->linkm.shift) & m->linkm.mask;
			int y0 = ((band << m->slipshift) - m->yscroll + m->c.slipoffset) & m->bitmapymask;
			if (y0 >= bh)
				y0 -= m->bitmaph;
			const int y1 = y0 + (1 << m->slipshift) - 1;
			if (y0 > cminy) cminy = y0;
			if (y1 < cmaxy) cmaxy = y1;
		}
		build_active_list(m, spriteram, link);
		m->next_xpos = 123456;
		if (m->activelist == m->activelast)
			continue;
		const uint16_t *first, *last;
		int step;
		if (m->c.reverse) {
			first = m->activelast - 4;
			last = m->activelist;
			step = -4;
		} else {
			first = m->activelist;
			last = m->activelast - 4;
			step = 4;
		}
		for (const uint16_t *cur = first;; cur += step) {
			render_object(m, bm, pitch, bw, bh, minx, maxx, cminy, cmaxy, cur);
			if (cur == last)
				break;
		}
	}
}
