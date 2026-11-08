/*  The planar graphics of the Irem 16-bit boards (irem_gfx.h). The layouts of MAME src/mame/irem/m72.cpp, m90.cpp,
    m92.cpp, m107.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria, Bryan McPhail).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "irem_gfx.h"

#include <stdlib.h>
#include <string.h>

// 8 pixels of a row from a byte of each plane
static inline void row8(uint8_t *d, const uint8_t *const *p, uint32_t o)
{
	const unsigned a = p[0][o], b = p[1][o], c = p[2][o], e = p[3][o];
	for (int x = 0; x < 8; x++) {
		const int s = 7 - x;
		d[x] = (uint8_t)(((a >> s) & 1) << 3 | ((b >> s) & 1) << 2 | ((c >> s) & 1) << 1 | ((e >> s) & 1));
	}
}

void irem_gfx_free(rbx_gfx *g)
{
	free(g->used);
	free(g->pix);
	memset(g, 0, sizeof(*g));
}

int irem_gfx_decode(rbx_gfx *g, const uint8_t *rom, uint32_t size, int kind, uint8_t **rows)
{
	const uint32_t part = kind == IGFX_M107_CHAR ? size : size / 4;
	const uint8_t *const p[4] = { rom + part * 3, rom + part * 2, rom + part, rom };
	const uint8_t *const q[4] = { rom + 1, rom, rom + 3, rom + 2 };
	const int wh = kind == IGFX_CHAR || kind == IGFX_M107_CHAR ? 8 : 16;
	const uint32_t bytes = kind == IGFX_CHAR ? 8 : 32;
	memset(g, 0, sizeof(*g));
	g->count = part / bytes;
	g->width = g->height = wh;
	if (!g->count)
		return -1;
	g->pix = (uint8_t *)malloc((size_t)g->count * (size_t)(wh * wh));
	g->used = (uint8_t *)calloc(g->count, 1);
	if (rows)
		*rows = (uint8_t *)calloc(g->count, 1);
	if (!g->pix || !g->used || (rows && !*rows))
		return -1;
	for (uint32_t e = 0; e < g->count; e++) {
		uint8_t *d = g->pix + (size_t)e * (size_t)(wh * wh);
		const uint32_t o = e * bytes;
		switch (kind) {
		case IGFX_CHAR:
			for (uint32_t y = 0; y < 8; y++)
				row8(d + y * 8, p, o + y);
			break;
		case IGFX_SPRITE:
			for (uint32_t y = 0; y < 16; y++) {
				row8(d + y * 16, p, o + y);
				row8(d + y * 16 + 8, p, o + 16 + y);
			}
			break;
		case IGFX_SPRITE2:
			for (uint32_t y = 0; y < 16; y++) {
				row8(d + y * 16, p, o + y * 2);
				row8(d + y * 16 + 8, p, o + y * 2 + 1);
			}
			break;
		case IGFX_SPRITE3:
			for (uint32_t y = 0; y < 16; y++) {
				row8(d + y * 16, p, o + y * 2 + 1);
				row8(d + y * 16 + 8, p, o + y * 2);
			}
			break;
		default:
			for (uint32_t y = 0; y < 8; y++)
				row8(d + y * 8, q, o + y * 4);
			break;
		}
		uint8_t any = 0;
		uint32_t mask = 0;
		for (int i = 0; i < wh * wh; i++)
			if (d[i]) {
				any = 1;
				mask |= 1u << (i / wh);
			}
		g->used[e] = any;
		if (rows)
			(*rows)[e] = (uint8_t)mask;
	}
	return 0;
}
