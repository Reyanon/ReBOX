/*  Tecmo's layer mixer with additive blending. Ported from MAME src/mame/tecmo/tecmo_mix.cpp (BSD-3-Clause,
    copyright-holders: David Haywood).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "tecmo_mix.h"

static inline uint32_t sum_colors(const uint32_t *pal, int c1, int c2)
{
	const uint32_t a = pal[c1], b = pal[c2];
	uint32_t r = ((a >> 16) & 0xff) + ((b >> 16) & 0xff), g = ((a >> 8) & 0xff) + ((b >> 8) & 0xff), bl = (a & 0xff) + (b & 0xff);
	if (r > 0xff) r = 0xff;
	if (g > 0xff) g = 0xff;
	if (bl > 0xff) bl = 0xff;
	return r << 16 | g << 8 | bl;
}

static inline uint32_t mix_rand(tecmo_mix_t *m)
{
	m->rand = m->rand * 1103515245u + 12345u;
	return (m->rand >> 8) & 0xffffff;
}

void tecmo_mix_line(tecmo_mix_t *m, const uint32_t *pal, const uint16_t *bg, const uint16_t *fg, const uint16_t *tx,
                    const uint16_t *sp, uint32_t *dd, int n)
{
	const int rev = m->revspritetile;
	for (int x = 0; x < n; x++) {
		uint16_t sprpixel = sp[x];
		const int sprpri = (sprpixel >> m->sprpri_shift) & 3;
		const int sprbln = (sprpixel >> m->sprbln_shift) & 1;
		const int sprcol = (sprpixel >> m->sprcol_shift) & 0xf;
		sprpixel = (uint16_t)((sprpixel & 0xf) | (sprcol << 4));
		int fgpixel = fg[x];
		const int fgbln = (fgpixel & 0x0100) >> 8;
		fgpixel &= 0xff;
		const int bgpixel = bg[x] & 0xff;
		const int txpixel = tx ? (tx[x] & 0xff) : 0;
		uint32_t c;
		if (sprpixel & 0xf) {
			if (sprpri == (0 ^ rev)) {   // behind all
				if (txpixel & 0xf)
					c = pal[txpixel + m->txregular_comp];
				else if (fgpixel & 0xf)
					c = fgbln ? mix_rand(m) : pal[fgpixel + m->fgregular_comp];
				else if (bgpixel & 0xf)
					c = pal[bgpixel + m->bgregular_comp];
				else
					c = sprbln ? mix_rand(m) : pal[sprpixel + m->spregular_comp];
			} else if (sprpri == (1 ^ rev)) {   // above bg, behind tx, fg
				if (txpixel & 0xf)
					c = pal[txpixel + m->txregular_comp];
				else if (fgpixel & 0xf) {
					if (fgbln) {
						if (sprbln)
							c = sum_colors(pal, bgpixel + m->bgblend_comp, sprpixel + m->spblend_source);
						else
							c = sum_colors(pal, fgpixel + m->fgblend_source, sprpixel + m->spblend_comp);
					} else
						c = pal[fgpixel + m->fgregular_comp];
				} else if (sprbln) {
					if (bgpixel & 0xf)
						c = sum_colors(pal, bgpixel + m->bgblend_comp, sprpixel + m->spblend_source);
					else
						c = sum_colors(pal, m->bgpen_blend, sprpixel + m->spblend_source);
				} else
					c = pal[sprpixel + m->spregular_comp];
			} else if (sprpri == (2 ^ rev)) {   // above bg, fg, behind tx
				if (txpixel & 0xf)
					c = pal[txpixel + m->txregular_comp];
				else if (sprbln) {
					if (fgpixel & 0xf)
						c = fgbln ? mix_rand(m) : sum_colors(pal, fgpixel + m->fgblend_comp, sprpixel + m->spblend_source);
					else if (bgpixel & 0xf)
						c = sum_colors(pal, bgpixel + m->bgblend_comp, sprpixel + m->spblend_source);
					else
						c = sum_colors(pal, m->bgpen_blend, sprpixel + m->spblend_source);
				} else
					c = pal[sprpixel + m->spregular_comp];
			} else {   // above all
				if (sprbln) {
					if (txpixel & 0xf)
						c = sum_colors(pal, txpixel + m->txblend_comp, sprpixel + m->spblend_source);
					else if (fgpixel & 0xf)
						c = fgbln ? mix_rand(m) : sum_colors(pal, fgpixel + m->fgblend_comp, sprpixel + m->spblend_source);
					else if (bgpixel & 0xf)
						c = sum_colors(pal, bgpixel + m->bgblend_comp, sprpixel + m->spblend_source);
					else
						c = sum_colors(pal, m->bgpen_blend, sprpixel + m->spblend_source);
				} else
					c = pal[sprpixel + m->spregular_comp];
			}
		} else {   // no sprite
			if (txpixel & 0xf)
				c = pal[txpixel + m->txregular_comp];
			else if (fgpixel & 0xf) {
				if (fgbln) {
					if (bgpixel & 0xf)
						c = sum_colors(pal, fgpixel + m->fgblend_source, bgpixel + m->bgblend_comp);
					else
						c = sum_colors(pal, fgpixel + m->fgblend_source, m->bgpen_blend);
				} else
					c = pal[fgpixel + m->fgregular_comp];
			} else if (bgpixel & 0xf)
				c = pal[bgpixel + m->bgregular_comp];
			else
				c = pal[m->bgpen];
		}
		dd[x] = c;
	}
}
