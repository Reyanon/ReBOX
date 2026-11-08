/*  Drawing shared by the Irem drivers: MAME's drawgfx transmask (src/emu/drawgfxt.ipp, BSD-3-Clause, copyright-holders:
    Nicola Salmoria, Aaron Giles) on bytes-a-pixel elements, and MAME's compute_resistor_weights / combine_weights
    (src/emu/video/resnet.cpp, BSD-3-Clause, copyright-holders: Couriersud).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_IREM_DRAW_H
#define RBX_IREM_DRAW_H

#include <stdint.h>

// an element of w x h bytes at src to bm, pens base + p, the pens with their bit in transmask left out; the clip's right
// and bottom exclusive
static inline void irem_blit(uint16_t *bm, int pitch, int cx0, int cy0, int cx1, int cy1, const uint8_t *src, int w, int h,
                             uint32_t base, uint32_t transmask, int fx, int fy, int sx, int sy)
{
	int y0 = sy < cy0 ? cy0 - sy : 0, y1 = sy + h > cy1 ? cy1 - sy : h;
	int x0 = sx < cx0 ? cx0 - sx : 0, x1 = sx + w > cx1 ? cx1 - sx : w;
	if (x0 >= x1)
		return;
	for (int j = y0; j < y1; j++) {
		const uint8_t *s = src + (fy ? h - 1 - j : j) * w;
		uint16_t *d = bm + (sy + j) * pitch + sx;
		if (fx) {
			for (int i = x0; i < x1; i++) {
				const uint32_t p = s[w - 1 - i];
				if (!((transmask >> p) & 1))
					d[i] = (uint16_t)(base + p);
			}
		} else {
			for (int i = x0; i < x1; i++) {
				const uint32_t p = s[i];
				if (!((transmask >> p) & 1))
					d[i] = (uint16_t)(base + p);
			}
		}
	}
}

// each resistor's output alone over the given pulldown (0 = none); returns the sum
static inline double irem_res_raw(const int *res, int n, int pulldown, double *w)
{
	double sum = 0;
	for (int k = 0; k < n; k++) {
		double r0 = pulldown ? 1.0 / pulldown : 1.0 / 1e12, r1 = 1.0 / 1e12;
		for (int j = 0; j < n; j++) {
			if (j == k)
				r1 += 1.0 / res[j];
			else
				r0 += 1.0 / res[j];
		}
		r0 = 1.0 / r0;
		r1 = 1.0 / r1;
		w[k] = 255.0 * r0 / (r1 + r0);
		sum += w[k];
	}
	return sum;
}

static inline int irem_res3(const double *w, double scale, int v)   // (combine_weights: bits 0-2)
{
	return (int)(scale * (w[0] * (v & 1) + w[1] * ((v >> 1) & 1) + w[2] * ((v >> 2) & 1)) + 0.5);
}

#endif
