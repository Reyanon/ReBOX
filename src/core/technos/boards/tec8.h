/*  Helpers for the Technos 8-bit boards: MAME's resistor networks (src/lib/util/resnet.cpp compute_resistor_weights
    and resnet.h combine_weights; BSD-3-Clause, copyright-holders: Couriersud) and a transparent tile blit.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_TECHNOS_TEC8_H
#define RBX_TECHNOS_TEC8_H

#include "../../rbx.h"
#include "../../video/gfxlayout.h"

// compute_resistor_weights(0, 255, -1.0, ...) for up to three networks (count 0 = none): all scaled so that the
// network with the greatest output gives 255
static inline void tec8_res_weights(int n1, const int *r1, double *w1, int pd1, int pu1, int n2, const int *r2, double *w2,
                                    int pd2, int pu2, int n3, const int *r3, double *w3, int pd3, int pu3)
{
	const int cnt[3] = { n1, n2, n3 }, pd[3] = { pd1, pd2, pd3 }, pu[3] = { pu1, pu2, pu3 };
	const int *rs[3] = { r1, r2, r3 };
	double *ws[3] = { w1, w2, w3 };
	double max = 0.0;
	for (int i = 0; i < 3; i++) {
		double sum = 0.0;
		for (int n = 0; n < cnt[i]; n++) {
			double R0 = pd[i] == 0 ? 1.0 / 1e12 : 1.0 / pd[i];
			double R1 = pu[i] == 0 ? 1.0 / 1e12 : 1.0 / pu[i];
			for (int j = 0; j < cnt[i]; j++) {
				if (rs[i][j] == 0)
					continue;
				if (j == n)
					R1 += 1.0 / rs[i][j];
				else
					R0 += 1.0 / rs[i][j];
			}
			R0 = 1.0 / R0;
			R1 = 1.0 / R1;
			double v = 255.0 * R0 / (R1 + R0);
			if (v < 0) v = 0;
			if (v > 255) v = 255;
			ws[i][n] = v;
			sum += v;
		}
		if (sum > max)
			max = sum;
	}
	const double scale = 255.0 / max;
	for (int i = 0; i < 3; i++)
		for (int n = 0; n < cnt[i]; n++)
			ws[i][n] *= scale;
}

static inline int tec8_combine3(const double *w, int b0, int b1, int b2) { return (int)(w[0] * b0 + w[1] * b1 + w[2] * b2 + 0.5); }
static inline int tec8_combine2(const double *w, int b0, int b1) { return (int)(w[0] * b0 + w[1] * b1 + 0.5); }

// MAME gfx->transpen (pen 0) of a w x h element onto the bitmap rows [y0, y1) (bitmap row = y - ybase), x in [0, bw)
static inline void tec8_transpen(const rbx_gfx *g, uint32_t code, uint16_t base, int fx, int fy, int sx, int sy, int y0,
                                 int y1, int ybase, int bw)
{
	code %= g->count;
	const int w = g->width, h = g->height;
	if (!g->used[code] || sy >= y1 || sy + h <= y0 || sx >= bw || sx + w <= 0)
		return;
	const uint8_t *src = g->pix + code * (uint32_t)(w * h);
	for (int j = 0; j < h; j++) {
		const int yy = sy + j;
		if (yy < y0 || yy >= y1)
			continue;
		const uint8_t *s = src + (fy ? h - 1 - j : j) * w;
		uint16_t *d = rbx_video.bitmap + (yy - ybase) * rbx_video.pitch;
		for (int i = 0; i < w; i++) {
			const int xx = sx + i;
			if ((unsigned)xx >= (unsigned)bw)
				continue;
			const uint8_t p = s[fx ? w - 1 - i : i];
			if (p)
				d[xx] = (uint16_t)(base + p);
		}
	}
}

#endif
