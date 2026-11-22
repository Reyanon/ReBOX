/*  Taito TC0630FDP (F3) video, ported from MAME src/mame/taito/taito_f3_v.cpp (BSD-3-Clause, copyright-holders:
    Bryan McPhail, ywy, 12Me21). Line RAM latching (read_line_ram), scroll registers (get_pf_scroll), clip planes
    (calc_clip), the priority / blend mixer (mix_line, render_line) and the sprite list walker (get_sprite_info,
    f3_drawgfx) follow MAME step by step. MAME draws whole tilemap pixmaps; here each layer line is fetched from the
    tile data straight into a line buffer (only the span the line shows), then mixed.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "tc0630fdp.h"

#include "rbx.h"

static uint16_t s_palprev[F3V_PAL_COLORS * 2];   // the palette RAM the colours were last made from (f3v_render)
static int s_palvalid, s_lutvalid;

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>

#define H_START 46
#define H_VIS 320
#define H_END (H_START + H_VIS)
#define NUM_PF 4
#define NUM_SPG 4
#define FB_W 512

// ---- graphics decoding (MAME tile_decode: low 4 bits | high 2 bits) ----
static uint8_t *decode16(const uint8_t *lo, uint32_t lo_size, const uint8_t *hi, uint32_t hi_size, int sprite, uint32_t *count)
{
	uint32_t n = lo_size / 128;
	uint8_t *out = (uint8_t *)malloc((size_t)n * 256 + 256);
	if (!out)
		return NULL;
	uint32_t nhi = hi ? hi_size / 64 : 0;
	for (uint32_t t = 0; t < n; t++) {
		uint8_t *d = out + (size_t)t * 256;
		const uint8_t *l = lo + t * 128;
		const uint8_t *h = t < nhi ? hi + t * 64 : NULL;
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				uint8_t b = l[y * 8 + (x >> 1)];
				uint8_t p = (x & 1) ? b >> 4 : b & 15;
				if (h) {
					const uint8_t *r = h + y * 4;
					uint8_t hb;
					if (sprite)
						hb = (r[x >> 2] >> ((x & 3) * 2)) & 3;
					else if (x < 8)
						hb = (uint8_t)(((r[0] >> x) & 1) | ((r[1] >> x) & 1) << 1);
					else
						hb = (uint8_t)(((r[2] >> (x - 8)) & 1) | ((r[3] >> (x - 8)) & 1) << 1);
					p |= hb << 4;
				}
				d[y * 16 + x] = p;
			}
	}
	*count = n ? n : 1;
	if (!n)
		memset(out, 0, 256);
	return out;
}

static int workers_init(const f3v_t *v);
static void workers_exit(void);

int f3v_init(f3v_t *v, const uint8_t *spr, uint32_t spr_size, const uint8_t *spr_hi, uint32_t spr_hi_size,
	const uint8_t *tile, uint32_t tile_size, const uint8_t *tile_hi, uint32_t tile_hi_size)
{
	v->spr_gfx = decode16(spr, spr ? spr_size : 0, spr_hi, spr_hi_size, 1, &v->spr_count);
	v->tile_gfx = decode16(tile, tile ? tile_size : 0, tile_hi, tile_hi_size, 0, &v->tile_count);
	v->tile_blank = v->tile_gfx ? (uint16_t *)calloc(v->tile_count, 2) : NULL;
	if (v->tile_blank)
		for (uint32_t t = 0; t < v->tile_count; t++)
			for (int y = 0; y < 16; y++) {
				const uint8_t *r = v->tile_gfx + (size_t)t * 256 + y * 16;
				int any = 0;
				for (int x = 0; x < 16; x++)
					any |= r[x];
				if (!any)
					v->tile_blank[t] |= (uint16_t)(1 << y);
			}
	v->fbs[0] = (uint16_t *)calloc(256 * FB_W, 2);
	v->fbs[1] = (uint16_t *)calloc(256 * FB_W, 2);
	v->fb = v->fb_draw = v->fbs[0];
	v->sprite_row_usage = v->sru_draw = v->sru[0];
	v->hkey = (uint32_t *)calloc(0x10000, 4);
	v->hval = (uint16_t *)calloc(0x10000, 2);
	v->hgen = (uint16_t *)calloc(0x10000, 2);
	if (!v->spr_gfx || !v->tile_gfx || !v->tile_blank || !v->fbs[0] || !v->fbs[1] || !v->hkey || !v->hval || !v->hgen || workers_init(v)) {
		f3v_exit(v);
		return -1;
	}
	f3v_reset(v);
	return 0;
}

void f3v_exit(f3v_t *v)
{
	free(v->spr_gfx);
	free(v->tile_gfx);
	free(v->tile_blank);
	v->tile_blank = NULL;
	free(v->fbs[0]);
	free(v->fbs[1]);
	free(v->hkey);
	free(v->hval);
	free(v->hgen);
	workers_exit();
	v->spr_gfx = v->tile_gfx = NULL;
	v->fb = v->fb_draw = v->fbs[0] = v->fbs[1] = NULL;
	v->hkey = NULL;
	v->hval = v->hgen = NULL;
}

void f3v_reset(f3v_t *v)
{
	v->flipscreen = v->sflip = 0;
	s_palvalid = s_lutvalid = 0;
	v->sprite_bank = 0;
	v->sprite_trails = 0;
	v->pen_mask = 0;
	v->nsprites = 0;
	for (int i = 0; i < 2; i++)
		if (v->fbs[i])
			memset(v->fbs[i], 0, 256 * FB_W * 2);
	memset(v->sru, 0, sizeof(v->sru));
	v->fb = v->fb_draw = v->fbs[0];
	v->sprite_row_usage = v->sru_draw = v->sru[0];
	memset(v->spriteram, 0, sizeof(v->spriteram));
}

// ---- sprites ----
typedef struct {
	int32_t block_scale, pos, block_pos;
	int16_t global, subglobal;
} sprite_axis;

static void axis_update(sprite_axis *a, uint8_t scroll, uint16_t posw, int multi, uint8_t block_ctrl, uint8_t new_zoom)
{
	int16_t new_pos = (int16_t)((posw & 0x800) ? (posw | 0xf000) : (posw & 0x0fff));
	if (scroll & 1)
		a->subglobal = new_pos;
	if (scroll & 2)
		a->global = new_pos;
	if (!(scroll & 8)) {
		new_pos = (int16_t)(new_pos + a->global);
		if (!(scroll & 4))
			new_pos = (int16_t)(new_pos + a->subglobal);
	}
	switch (block_ctrl) {
	case 0:
		if (!multi) {
			a->block_pos = new_pos * 256;
			a->block_scale = 0x100 - new_zoom;
		}
		// fall through
	case 2:
		a->pos = a->block_pos;
		break;
	case 3:
		a->pos += a->block_scale * 16;
		break;
	}
}

static void get_sprite_info(f3v_t *v)
{
	sprite_axis x, y;
	memset(&x, 0, sizeof(x));
	memset(&y, 0, sizeof(y));
	x.block_scale = y.block_scale = 1 << 8;
	uint8_t color = 0;
	int multi = 0;
	f3v_sprite *sp = v->sprites;
	int total = 0;
	for (int offs = 0; offs < 0x400 && total < 0x400; offs++) {
		total++;
		const uint16_t *spr = &v->spriteram[(v->sprite_bank ? 0x4000 : 0) + offs * 8];
		if (spr[3] & 0x8000) {
			uint16_t cntrl = spr[5];
			v->sflip = (cntrl >> 13) & 1;
			int extra = (cntrl >> 8) & 3;
			v->pen_mask = (uint8_t)((extra << 4) | 0x0f);
			v->sprite_trails = (cntrl >> 1) & 1;
			v->sprite_bank = cntrl & 1;
		}
		if (spr[6] & 0x8000) {
			int new_offs = spr[6] & 0x3ff;
			if (new_offs == offs)
				break;
			offs = new_offs - 1;
		}
		uint8_t spritecont = (uint8_t)(spr[4] >> 8);
		if (!(spritecont & 4))
			color = (uint8_t)(spr[4] & 0xff);
		uint8_t scroll_mode = (uint8_t)((spr[2] >> 12) & 0xf);
		uint16_t zooms = spr[1];
		axis_update(&x, scroll_mode, spr[2] & 0xfff, multi, (spritecont >> 6) & 3, (uint8_t)(zooms & 0xff));
		axis_update(&y, scroll_mode, spr[3] & 0xfff, multi, (spritecont >> 4) & 3, (uint8_t)(zooms >> 8));
		multi = (spritecont >> 3) & 1;
		int tile = spr[0] | ((spr[5] & 1) << 16);
		if (!tile)
			continue;
		int32_t tx = v->sflip ? (512 << 8) - x.block_scale * 16 - x.pos : x.pos;
		int32_t ty = v->sflip ? (256 << 8) - y.block_scale * 16 - y.pos : y.pos;
		if (tx + x.block_scale * 16 <= H_START << 8 || tx > (H_END - 1) << 8 ||
		    ty + y.block_scale * 16 <= v->vis_y0 << 8 || ty > v->vis_y1 << 8)
			continue;
		int flip_x = spritecont & 1, flip_y = (spritecont >> 1) & 1;
		sp->x = tx;
		sp->y = ty;
		sp->flip_x = (uint8_t)(v->sflip ? !flip_x : flip_x);
		sp->flip_y = (uint8_t)(v->sflip ? !flip_y : flip_y);
		sp->code = tile;
		sp->color = color;
		sp->scale_x = x.block_scale;
		sp->scale_y = y.block_scale;
		sp->pri = (color >> 6) & 3;
		sp++;
	}
	v->nsprites = (int)(sp - v->sprites);
}

// SIMD types (GCC vector extensions: NEON on the Vita, SSE on the PC)
typedef uint8_t v16u8 __attribute__((vector_size(16)));
typedef int8_t v16s8 __attribute__((vector_size(16)));
// (16-byte vectors at most: a NEON q register; wider ones spill to the stack on the Vita)
typedef uint8_t v8u8 __attribute__((vector_size(8)));
typedef uint16_t v8u16 __attribute__((vector_size(16)));

// the sprite's rows from ylo to yhi
static void draw_sprite(f3v_t *v, const f3v_sprite *s, int ylo, int yhi)
{
	const uint8_t *code_base = v->spr_gfx + (size_t)((uint32_t)s->code < v->spr_count ? (uint32_t)s->code : (uint32_t)s->code % v->spr_count) * 256;
	const uint8_t flipx = s->flip_x ? 0xf : 0, flipy = s->flip_y ? 0xf : 0;
	int32_t dy8 = s->y;
	if (!v->sflip)
		dy8 += 255;
	if ((dy8 + 15 * s->scale_y) >> 8 < ylo || dy8 >> 8 > yhi)
		return;
	if (ylo < v->vis_y0)
		ylo = v->vis_y0;
	if (yhi > v->vis_y1)
		yhi = v->vis_y1;
	const uint16_t cbase = (uint16_t)(0x1000 + (s->color << 4));
	const int x0 = (s->x + 128) >> 8;
	if (s->scale_x == 256 && x0 >= H_START && x0 + 15 <= H_END - 1) {
		// unzoomed across, fully inside the visible columns: a row of 16 pixels at once (same pixels as below)
		const v16u8 pm = (v16u8){ 0 } + v->pen_mask;
		for (int yy = 0; yy < 16; yy++) {
			int dy = dy8 >> 8;
			dy8 += s->scale_y;
			if (dy < ylo || dy > yhi)
				continue;
			uint16_t *dest = v->fb_draw + dy * FB_W + x0;
			v16u8 c;
			memcpy(&c, code_base + (yy ^ flipy) * 16, 16);
			if (flipx)
				c = __builtin_shuffle(c, (v16u8){ 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 });
			c &= pm;
			v8u16 used = { 0 };
			for (int h = 0; h < 2; h++) {
				v8u8 c8;
				memcpy(&c8, (const uint8_t *)&c + h * 8, 8);
				v8u16 d, c16 = __builtin_convertvector(c8, v8u16);
				memcpy(&d, dest + h * 8, 16);
				v8u16 m = (v8u16)((c16 != 0) & (d == 0));
				d = (d & ~m) | ((c16 | cbase) & m);
				memcpy(dest + h * 8, &d, 16);
				used |= m;
			}
			uint64_t any[2];
			memcpy(any, &used, 16);
			if (any[0] | any[1])
				v->sru_draw[dy] |= (uint8_t)(1 << s->pri);
		}
		return;
	}
	for (int yy = 0; yy < 16; yy++) {
		int dy = dy8 >> 8;
		dy8 += s->scale_y;
		if (dy < ylo || dy > yhi)
			continue;
		uint16_t *dest = v->fb_draw + dy * FB_W;
		const uint8_t *src = code_base + (yy ^ flipy) * 16;
		int32_t dx8 = s->x + 128;
		uint8_t used = 0;
		for (int xx = 0; xx < 16; xx++) {
			int dx = dx8 >> 8;
			dx8 += s->scale_x;
			if (dx < H_START || dx > H_END - 1)
				continue;
			if (dx == dx8 >> 8)
				continue;
			uint8_t c = src[xx ^ flipx] & v->pen_mask;
			if (c && !dest[dx]) {
				dest[dx] = (uint16_t)(cbase | c);
				used = 1;
			}
		}
		if (used)
			v->sru_draw[dy] |= (uint8_t)(1 << s->pri);
	}
}

// the sprites of rows ylo..yhi (each row's pixels depend only on that row: the rows can be drawn in bands)
static void draw_sprites_rows(f3v_t *v, int ylo, int yhi)
{
	if (ylo < v->vis_y0)
		ylo = v->vis_y0;
	if (yhi > v->vis_y1)
		yhi = v->vis_y1;
	if (!v->sprite_trails) {
		memset(v->sru_draw + ylo, 0, (size_t)(yhi - ylo + 1));
		for (int y = ylo; y <= yhi; y++)
			memset(v->fb_draw + y * FB_W + H_START, 0, H_VIS * 2);
	} else if (v->fb_draw != v->fb) {   // (trails: on top of the frame the lines have)
		memcpy(v->sru_draw + ylo, v->sprite_row_usage + ylo, (size_t)(yhi - ylo + 1));
		for (int y = ylo; y <= yhi; y++)
			memcpy(v->fb_draw + y * FB_W + H_START, v->fb + y * FB_W + H_START, H_VIS * 2);
	}
	for (int i = v->nsprites; i-- > 0; )
		draw_sprite(v, &v->sprites[i], ylo, yhi);
}

static void draw_sprites(f3v_t *v)
{
	if (!v->sprite_trails)
		memset(v->sru_draw, 0, 256);   // (the rows outside the visible ones too, as before)
	draw_sprites_rows(v, v->vis_y0, v->vis_y1);
}

// ---- line state (MAME f3_line_inf) ----
typedef struct { int16_t l, r; } clip_t;

typedef struct {
	uint16_t mix;
	uint8_t prio, blend_mode, x_sample_enable;
} mixable;

typedef struct {
	mixable m;
	uint16_t colscroll;
	uint8_t alt_tilemap;
	int32_t x_scale, y_scale;
	uint16_t pal_add;
	int32_t rowscroll, reg_sx, reg_sy, reg_fx_y, reg_fx_x;
} pf_inf;

typedef struct {
	mixable m;
	uint8_t blend_select_v;
} sp_inf;

typedef struct {
	mixable m;
	uint8_t pivot_control, blend_select_v;
	uint16_t pivot_enable;
	int32_t reg_sx, reg_sy;
} pv_inf;

typedef struct {
	clip_t clip[4];
	uint8_t blend[4];
	uint8_t x_sample;
	uint16_t bg_palette;
	pv_inf pivot;
	sp_inf sp[NUM_SPG];
	pf_inf pf[NUM_PF];
	// (from here: the drawing side's cache, not part of a line's state)
	uint8_t mosaic_for;      // x_sample the table below was made for (0 = none)
	int16_t mosaic_x[432];   // mosaic(x, x_sample)
} line_inf;
#define LINE_STATE_SIZE offsetof(line_inf, mosaic_for)

static inline void set_mix(mixable *m, uint16_t v) { m->mix = v; m->prio = v & 0xf; m->blend_mode = (uint8_t)((v >> 14) & 3); }
static inline void set_prio(mixable *m, uint8_t p) { m->mix = (uint16_t)((m->mix & 0xfff0) | p); m->prio = p; }
static inline void set_blend(mixable *m, uint8_t b) { m->mix = (uint16_t)((m->mix & 0x3fff) | (b << 14)); m->blend_mode = b; }

static inline void clip_upper(clip_t *c, int left, int right)
{
	c->l = (int16_t)((c->l & 0xff) | left << 8);
	c->r = (int16_t)((c->r & 0xff) | right << 8);
}
static inline void clip_lower(clip_t *c, int left, int right)
{
	c->l = (int16_t)((c->l & 0x100) | left);
	c->r = (int16_t)((c->r & 0x100) | right);
}

static inline uint32_t latched(const uint16_t *lr, int y, int section, int sub)
{
	uint16_t latches = lr[(section * 0x200) / 2 + y];
	uint32_t base = 0x4000 + 0x1000 * section + 0x200 * sub;
	if (latches & (1 << (sub + 4)))
		return (base + 0x800) / 2 + y;
	if (latches & (1 << sub))
		return base / 2 + y;
	return 0;
}

static void read_line_ram(const f3v_t *v, line_inf *line, int y)
{
	const uint16_t *lr = v->lineram;
	uint32_t w;
	for (int i = 2; i <= 3; i++)
		if ((w = latched(lr, y, 0, i))) {
			uint16_t cs = lr[w];
			line->pf[i].colscroll = cs & 0x1ff;
			line->pf[i].alt_tilemap = (uint8_t)(!v->extend && (cs & 0x200));
			clip_upper(&line->clip[2 * (i - 2) + 0], (cs >> 12) & 1, (cs >> 13) & 1);
			clip_upper(&line->clip[2 * (i - 2) + 1], (cs >> 14) & 1, (cs >> 15) & 1);
		}
	for (int i = 0; i < 4; i++)
		if ((w = latched(lr, y, 1, i)))
			clip_lower(&line->clip[i], lr[w] & 0xff, lr[w] >> 8);
	if ((w = latched(lr, y, 2, 0))) {
		uint16_t l6 = lr[w];
		line->pivot.blend_select_v = (l6 >> 9) & 1;
		line->pivot.pivot_control = (uint8_t)(l6 >> 8);
		for (int g = 0; g < NUM_SPG; g++)
			set_blend(&line->sp[g].m, (uint8_t)((l6 >> (g * 2)) & 3));
	}
	if ((w = latched(lr, y, 2, 1))) {
		uint16_t bv = lr[w];
		for (int i = 0; i < 4; i++) {
			int a = 0xf - ((bv >> (4 * i)) & 0xf);
			line->blend[i] = (uint8_t)(a < 8 ? a : 8);
		}
	}
	if ((w = latched(lr, y, 2, 2))) {
		uint16_t xm = lr[w];
		line->x_sample = (uint8_t)(16 - ((xm >> 4) & 0xf));
		for (int i = 0; i < NUM_PF; i++)
			line->pf[i].m.x_sample_enable = (xm >> i) & 1;
		for (int g = 0; g < NUM_SPG; g++)
			line->sp[g].m.x_sample_enable = (xm >> 8) & 1;
		line->pivot.m.x_sample_enable = (xm >> 9) & 1;
	}
	if ((w = latched(lr, y, 2, 3)))
		line->bg_palette = lr[w];
	if ((w = latched(lr, y, 3, 0)))
		line->pivot.pivot_enable = lr[w];
	if ((w = latched(lr, y, 3, 1)))
		set_mix(&line->pivot.m, lr[w]);
	if ((w = latched(lr, y, 3, 2))) {
		uint16_t sm = lr[w];
		for (int g = 0; g < NUM_SPG; g++) {
			set_mix(&line->sp[g].m, (uint16_t)((line->sp[g].m.mix & 0xc00f) | (sm & 0x3ff) << 4));
			line->sp[g].blend_select_v = (sm >> (12 + g)) & 1;
		}
	}
	if ((w = latched(lr, y, 3, 3))) {
		uint16_t sp = lr[w];
		for (int g = 0; g < NUM_SPG; g++)
			set_prio(&line->sp[g].m, (uint8_t)((sp >> (g * 4)) & 0xf));
	}
	static const int FIX_Y[4] = { 0, 3, 2, 1 };
	for (int i = 0; i < 4; i++)
		if ((w = latched(lr, y, 4, i))) {
			uint16_t sc = lr[w];
			line->pf[i].x_scale = 256 - (sc >> 8);
			line->pf[FIX_Y[i]].y_scale = (sc & 0xff) << 1;
		}
	for (int i = 0; i < 4; i++)
		if ((w = latched(lr, y, 5, i)))
			line->pf[i].pal_add = (uint16_t)(lr[w] * 16);
	for (int i = 0; i < 4; i++)
		if ((w = latched(lr, y, 6, i))) {
			int32_t rs = (int32_t)lr[w] << 2;
			line->pf[i].rowscroll = (rs & (int32_t)0xffffff00) - (rs & 0xff);
		}
	for (int i = 0; i < 4; i++)
		if ((w = latched(lr, y, 7, i)))
			set_mix(&line->pf[i].m, lr[w]);
}

static void get_pf_scroll(const f3v_t *v, int pf, int32_t *reg_sx, int32_t *reg_sy)
{
	int16_t sx_raw = (int16_t)v->ctrl0[pf];
	int16_t sy_raw = (int16_t)v->ctrl0[pf + 4];
	sy_raw = (int16_t)(sy_raw + (1 << 7));
	if (v->flipscreen) {
		sx_raw = (int16_t)(sx_raw + (320 << 6));
		sx_raw = (int16_t)(sx_raw + ((512 + 192) << 6));
		sy_raw = (int16_t)-sy_raw;
	}
	sx_raw = (int16_t)(sx_raw + ((40 - 4 * pf) << 6));
	int32_t sx = (int32_t)sx_raw * 4;
	int32_t sy = (int32_t)sy_raw * 2;
	sx ^= 0xfc;
	sx -= H_START << 8;
	if (v->flipscreen)
		sy = -sy;
	*reg_sx = sx;
	*reg_sy = sy;
}

// ---- clip planes (MAME calc_clip) ----
static int calc_clip(const clip_t *clip, uint16_t mix, clip_t *ranges)
{
	const int16_t INF_L = H_START, INF_R = H_END;
	int en = (mix >> 8) & 0xf, inv = (mix >> 4) & 0xf;
	int normal = en & ~inv, invert = en & inv;
	if (!(mix & 0x1000)) {
		int t = normal;
		normal = invert;
		invert = t;
	}
	int n = 1;
	ranges[0].l = INF_L;
	ranges[0].r = INF_R;
	for (int p = 0; p < 4; p++) {
		int16_t cl = (int16_t)(clip[p].l - 1), cr = (int16_t)(clip[p].r - 2);
		if (normal & (1 << p)) {
			int k = 0;
			for (int i = 0; i < n; i++) {
				clip_t c = ranges[i];
				if (cl > cr || c.r < cl || c.l > cr)
					continue;
				if (c.l < cl) c.l = cl;
				if (c.r > cr) c.r = cr;
				ranges[k++] = c;
			}
			n = k;
		} else if ((invert & (1 << p)) && cl <= cr) {
			clip_t nr[16];
			int m = 0;
			for (int i = 0; i < n && m < 16; i++) { nr[m].l = INF_L; nr[m].r = cl; m++; }
			for (int i = 0; i < n && m < 16; i++) { nr[m].l = cr; nr[m].r = INF_R; m++; }
			int k = 0;
			for (int j = 0; j < m; j++) {
				clip_t c = nr[j];
				int erased = 0;
				for (int i = 0; i < n; i++) {
					if (c.l < ranges[i].l) c.l = ranges[i].l;
					if (c.r < ranges[i].l) c.r = ranges[i].l;
					if (c.l >= c.r) {
						erased = 1;
						break;
					}
				}
				if (!erased && k < 8)
					ranges[k++] = c;
			}
			n = k;
		}
	}
	return n;
}

static inline int mosaic(int x, int sample)
{
	int x_count = x - 46 + 114;
	if (x_count >= 432)
		x_count -= 432;
	return x - (x_count % sample);
}

static const int16_t *mosaic_table(line_inf *line)
{
	if (line->mosaic_for != line->x_sample) {
		for (int x = H_START; x < H_END; x++)
			line->mosaic_x[x] = (int16_t)mosaic(x, line->x_sample);
		line->mosaic_for = line->x_sample;
	}
	return line->mosaic_x;
}

// ---- mixer (MAME mix_pix / pri_mode) ----
typedef struct {
	uint16_t src_pal[432], dst_pal[432];
	uint8_t src_blend[432], dst_blend[432];
	uint8_t src_prio[432], dst_prio[432], src_bm[432];
} mixbuf;

// layer line: pal[x] (palette adjusted), f[x] = 0 (nothing) or 0x80 | blend select
typedef struct {
	uint16_t pal[432];
	uint8_t f[432];
} layer_line;

// one opaque layer pixel into the mix (pixel x of the layer, its palette entry and blend select)
static inline void mix_px(mixbuf *restrict z, int x, uint16_t pal, int sel, uint8_t prio, uint8_t bm, const uint8_t *restrict blend)
{
	if (bm == z->src_bm[x])
		return;
	if (prio > z->src_prio[x]) {
		switch (bm) {
		case 1:
			if (!blend[2 + sel]) return;
			z->src_blend[x] = blend[2 + sel];
			break;
		case 2:
			if (!blend[sel]) return;
			z->src_blend[x] = blend[sel];
			break;
		default:
			if (blend[sel] + blend[2 + sel] == 0) return;
			z->src_blend[x] = blend[2 + sel];
			z->dst_blend[x] = blend[sel];
			z->dst_prio[x] = prio;
			z->dst_pal[x] = pal;
			break;
		}
		z->src_pal[x] = pal;
		z->src_bm[x] = bm;
		z->src_prio[x] = prio;
	} else if (prio >= z->dst_prio[x]) {
		z->dst_pal[x] = prio != z->dst_prio[x] ? pal : 0;
		z->dst_prio[x] = prio;
		z->dst_blend[x] = z->src_bm[x] == 1 ? blend[sel] : blend[2 + sel];
	}
}

// ---- 16 pixel vector mixer (GCC vector extensions: SSE on x86, NEON q registers on the Vita) ----
// the same decisions as mix_px, made for 16 lanes at once with masks; bm is a constant in each caller's loop.
// vectors stay at 16 bytes (wider ones spill to the stack) and the per call constants travel as scalars.
typedef uint8_t u8x16 __attribute__((vector_size(16)));
typedef uint16_t u16x8 __attribute__((vector_size(16)));
typedef uint64_t u64x2 __attribute__((vector_size(16)));

#define SEL(m, a, b) (((a) & (m)) | ((b) & ~(m)))
#define BC8(v) ((u8x16){ 0 } + (uint8_t)(v))

// 0x00/0xff byte mask of lanes 0..7 (lo) or 8..15 (hi) as a 16 bit lane mask
static inline u16x8 mask_lo(u8x16 m)
{
	return (u16x8)__builtin_shuffle(m, (u8x16){ 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7 });
}
static inline u16x8 mask_hi(u8x16 m)
{
	return (u16x8)__builtin_shuffle(m, (u8x16){ 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15 });
}
// two 16 bit lane masks narrowed to one byte mask
static inline u8x16 mask_narrow(u16x8 lo, u16x8 hi)
{
	return __builtin_shuffle((u8x16)lo, (u8x16)hi, (u8x16){ 0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30 });
}

static inline int any16(u8x16 m)
{
	u64x2 w = (u64x2)m;
	return (w[0] | w[1]) != 0;
}

static inline void ld16(u8x16 *v, const uint8_t *p) { memcpy(v, p, 16); }
static inline void st16(uint8_t *p, u8x16 v) { memcpy(p, &v, 16); }
static inline void ldp(u16x8 *v, const uint16_t *p) { memcpy(v, p, 16); }
static inline void stp(uint16_t *p, u16x8 v) { memcpy(p, &v, 16); }

// valid: 0xff where the layer pixel is opaque; b_lo / b_hi / okm: blend[sel], blend[2 + sel] and the "blend not 0"
// test of each lane (from its blend select)
static inline __attribute__((always_inline)) void mix16(mixbuf *restrict z, int x, u16x8 pal0, u16x8 pal1, u8x16 valid, u8x16 b_lo, u8x16 b_hi,
                         u8x16 okm, uint8_t prio, uint8_t bm)
{
	u8x16 sbm, sp, dp;
	ld16(&sbm, z->src_bm + x);
	const u8x16 vbm = BC8(bm), vprio = BC8(prio);
	const u8x16 act = valid & (u8x16)(sbm != vbm);
	if (!any16(act))
		return;
	ld16(&sp, z->src_prio + x);
	ld16(&dp, z->dst_prio + x);
	const u8x16 gt = (u8x16)(vprio > sp);
	const u8x16 A = act & gt & okm;
	const u8x16 B = act & ~gt & (u8x16)(vprio >= dp);
	const int adst = bm != 1 && bm != 2;
	const u8x16 AD = adst ? A : (u8x16){ 0 };
	if (any16(A)) {
		u8x16 sbl;
		u16x8 s0, s1;
		ld16(&sbl, z->src_blend + x);
		st16(z->src_blend + x, SEL(A, bm == 2 ? b_lo : b_hi, sbl));
		ldp(&s0, z->src_pal + x);
		ldp(&s1, z->src_pal + x + 8);
		stp(z->src_pal + x, SEL(mask_lo(A), pal0, s0));
		stp(z->src_pal + x + 8, SEL(mask_hi(A), pal1, s1));
		st16(z->src_bm + x, SEL(A, vbm, sbm));
		st16(z->src_prio + x, SEL(A, vprio, sp));
	}
	const u8x16 D = AD | B;
	if (any16(D)) {
		u8x16 dbl;
		u16x8 d0, d1;
		ld16(&dbl, z->dst_blend + x);
		const u8x16 bB = SEL((u8x16)(sbm == BC8(1)), b_lo, b_hi);
		st16(z->dst_blend + x, SEL(AD, b_lo, SEL(B, bB, dbl)));
		const u8x16 keep = AD | (u8x16)(vprio != dp);   // B with equal prio writes 0
		ldp(&d0, z->dst_pal + x);
		ldp(&d1, z->dst_pal + x + 8);
		stp(z->dst_pal + x, SEL(mask_lo(D), pal0 & mask_lo(keep), d0));
		stp(z->dst_pal + x + 8, SEL(mask_hi(D), pal1 & mask_hi(keep), d1));
		st16(z->dst_prio + x, SEL(D, vprio, dp));
	}
}

static inline void mix_ok(uint8_t bm, const uint8_t *blend, uint8_t *ok0, uint8_t *ok1)
{
	int o0, o1;
	if (bm == 1) { o0 = blend[2] != 0; o1 = blend[3] != 0; }
	else if (bm == 2) { o0 = blend[0] != 0; o1 = blend[1] != 0; }
	else { o0 = blend[0] + blend[2] != 0; o1 = blend[1] + blend[3] != 0; }
	*ok0 = o0 ? 0xff : 0;
	*ok1 = o1 ? 0xff : 0;
}

static inline __attribute__((always_inline)) void mix_line_k(mixbuf *restrict z, const layer_line *restrict ll, uint8_t prio, uint8_t bm,
                              const uint8_t *restrict blend, int l, int r)
{
	uint8_t ok0, ok1;
	mix_ok(bm, blend, &ok0, &ok1);
	// lane values from the blend select: v0 ^ (sel1 & (v0 ^ v1))
	const uint8_t lx = blend[0] ^ blend[1], hx = blend[2] ^ blend[3], ox = ok0 ^ ok1;
	int x = l;
	for (; x + 16 <= r; x += 16) {
		u8x16 f;
		ld16(&f, ll->f + x);
		if (!any16(f))
			continue;
		u16x8 p0, p1;
		ldp(&p0, ll->pal + x);
		ldp(&p1, ll->pal + x + 8);
		const u8x16 zero = { 0 };
		const u8x16 sel1 = (u8x16)((f & 1) != zero);
		mix16(z, x, p0, p1, (u8x16)(f != zero), BC8(blend[0]) ^ (sel1 & BC8(lx)), BC8(blend[2]) ^ (sel1 & BC8(hx)),
		      BC8(ok0) ^ (sel1 & BC8(ox)), prio, bm);
	}
	for (; x < r; x++)
		if (ll->f[x])
			mix_px(z, x, ll->pal[x], ll->f[x] & 1, prio, bm, blend);
}

static __attribute__((noinline)) void mix_line(mixbuf *restrict z, const layer_line *restrict ll, uint8_t prio, uint8_t bm, const uint8_t *blend, int l, int r)
{
	// local copy: the uint8_t stores into z could alias blend and force reloads
	const uint8_t b[4] = { blend[0], blend[1], blend[2], blend[3] };
	switch (bm) {
	case 0: mix_line_k(z, ll, prio, 0, b, l, r); break;
	case 1: mix_line_k(z, ll, prio, 1, b, l, r); break;
	case 2: mix_line_k(z, ll, prio, 2, b, l, r); break;
	case 3: mix_line_k(z, ll, prio, 3, b, l, r); break;
	default: mix_line_k(z, ll, prio, bm, b, l, r); break;
	}
}

static inline __attribute__((always_inline)) void mix_sprites_k(mixbuf *restrict z, const uint16_t *restrict row, const int16_t *mx, int group, int sel,
                                 uint8_t prio, uint8_t bm, const uint8_t *restrict blend, int l, int r)
{
	uint8_t ok0, ok1;
	mix_ok(bm, blend, &ok0, &ok1);
	const uint8_t b_lo = blend[sel & 1], b_hi = blend[2 + (sel & 1)], okm = (sel & 1) ? ok1 : ok0;
	const uint16_t gval = (uint16_t)(group << 10);
	int x = l;
	for (; x + 16 <= r; x += 16) {
		u16x8 c0, c1;
		if (mx) {
			uint16_t t[16];
			for (int i = 0; i < 16; i++) {
				int rx = mx[x + i];
				t[i] = rx >= 0 ? row[rx] : 0;
			}
			ldp(&c0, t);
			ldp(&c1, t + 8);
		} else {
			ldp(&c0, row + x);
			ldp(&c1, row + x + 8);
		}
		const u16x8 zero16 = { 0 }, gm = (u16x8){ 0 } + 0x0c00, gv = (u16x8){ 0 } + gval;
		const u16x8 v0 = (u16x8)((c0 != zero16) & ((c0 & gm) == gv));
		const u16x8 v1 = (u16x8)((c1 != zero16) & ((c1 & gm) == gv));
		const u8x16 valid = mask_narrow(v0, v1);
		if (!any16(valid))
			continue;
		mix16(z, x, c0, c1, valid, BC8(b_lo), BC8(b_hi), BC8(okm), prio, bm);
	}
	for (; x < r; x++) {
		int rx = mx ? mx[x] : x;
		uint16_t c = rx >= 0 ? row[rx] : 0;
		if (c && ((c >> 10) & 3) == group)
			mix_px(z, x, c, sel, prio, bm, blend);
	}
}

// a sprite group straight from the sprite frame buffer row (MAME: the sprite layer line of that group)
static __attribute__((noinline)) void mix_sprites(mixbuf *restrict z, const uint16_t *restrict row, const int16_t *mx, int group, int sel, uint8_t prio, uint8_t bm,
                        const uint8_t *blend, int l, int r)
{
	const uint8_t b[4] = { blend[0], blend[1], blend[2], blend[3] };
	switch (bm) {
	case 0: mix_sprites_k(z, row, mx, group, sel, prio, 0, b, l, r); break;
	case 1: mix_sprites_k(z, row, mx, group, sel, prio, 1, b, l, r); break;
	case 2: mix_sprites_k(z, row, mx, group, sel, prio, 2, b, l, r); break;
	case 3: mix_sprites_k(z, row, mx, group, sel, prio, 3, b, l, r); break;
	default: mix_sprites_k(z, row, mx, group, sel, prio, bm, b, l, r); break;
	}
}

// ---- layer line fetch ----
typedef struct {
	const f3v_t *v;
	int tm_flip;
	uint8_t pf_row_usage[32][8];
	uint8_t text_row_usage[64];
	uint8_t pix_row_usage[32];   // pixel layer: a cell row with some pixel set (blank rows draw nothing)
	uint16_t rowpal[1024];
	uint8_t rowf[1024];
} fetch_ctx;

// one whole 16-pixel tile row with SIMD (GCC vector extensions: NEON on the Vita, SSE on the PC)

static inline void tile_row16(const uint8_t *src, int rev, uint8_t pen_mask, int base, uint8_t fsel, uint16_t *rp, uint8_t *rf)
{
	v16u8 p;
	memcpy(&p, src, 16);
	if (rev)
		p = __builtin_shuffle(p, (v16u8){ 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 });
	p &= pen_mask;
	v16u8 f = (v16u8)(p != 0) & fsel;
	for (int h = 0; h < 2; h++) {
		v8u8 p8;
		memcpy(&p8, (const uint8_t *)&p + h * 8, 8);
		v8u16 w = (__builtin_convertvector(p8, v8u16) + (uint16_t)base) & 0x1fff;
		memcpy(rp + h * 8, &w, 16);
	}
	memcpy(rf, &f, 16);
}

static void fetch_pf(fetch_ctx *fc, const pf_inf *pf, int tmap, int y_line, line_inf *line, layer_line *ll)
{
	const f3v_t *v = fc->v;
	const int cols = v->extend ? 64 : 32, W = cols * 16, wmask = W - 1;
	const uint16_t *data = v->vram + (v->extend ? 0x1000 * tmap : 0x800 * tmap);
	int gy = ((pf->reg_fx_y >> 8) + pf->colscroll) & 0x1ff;
	(void)y_line;
	int uy = fc->tm_flip ? 511 - gy : gy;
	const uint16_t *rowp = data + (uy >> 4) * cols * 2;
	int py = uy & 15;
	// span of source columns this line can show (mosaic samples up to 15 pixels to the left)
	int x0 = pf->m.x_sample_enable ? H_START - 16 : H_START;
	int g0 = ((pf->reg_fx_x + (x0 - H_START) * pf->x_scale) >> 8) + H_START;
	int g1 = ((pf->reg_fx_x + (H_END - 1 - H_START) * pf->x_scale) >> 8) + H_START;
	if (g1 - g0 >= W)
		g1 = g0 + W - 1;
	const uint8_t *gfx = v->tile_gfx;
	// unzoomed: source column g lands on screen x = g - g0 + H_START, so write straight into the layer line
	const int direct = !pf->m.x_sample_enable && pf->x_scale == 256;
	for (int g = g0; g <= g1; ) {
		int gx = g & wmask;
		int ux = fc->tm_flip ? wmask - gx : gx;
		const uint16_t *t = rowp + (ux >> 4) * 2;
		uint16_t attr = t[0], code = t[1];
		int pc = attr & 0x1ff, sel = (attr >> 9) & 1, extra = (attr >> 10) & 3;
		uint8_t pen_mask = (uint8_t)((((extra & ~pc) & 3) << 4) | 0x0f);
		int fx = (attr >> 14) & 1, fy = (attr >> 15) & 1;
		uint32_t tc = code < v->tile_count ? code : code % v->tile_count;
		int ty = fy ? 15 - py : py;
		const uint8_t *src = gfx + (size_t)tc * 256 + ty * 16;
		int base = pc * 16 + pf->pal_add;
		// pixels of this tile from gx on; with the tilemap flipped, pixel q of the cell is 15 - q of the tile
		int n = 16 - (gx & 15);
		if (n > g1 - g + 1)
			n = g1 - g + 1;
		int q0 = gx & 15, rev = fx ^ fc->tm_flip;
		uint16_t *rp;
		uint8_t *rf;
		if (direct) {
			int d = g - g0 + H_START - q0;
			rp = ll->pal + d;
			rf = ll->f + d;
		} else {
			rp = fc->rowpal + (gx & ~15);
			rf = fc->rowf + (gx & ~15);
		}
		const uint8_t fsel = (uint8_t)(0x80 | sel);
		if ((v->tile_blank[tc] >> ty) & 1)
			memset(rf + q0, 0, (size_t)n);   // (the colours of transparent pixels are never used)
		else if (n == 16)
			tile_row16(src, rev, pen_mask, base, fsel, rp, rf);
		else if (rev)
			for (int q = q0; q < q0 + n; q++) {
				uint8_t p = src[15 - q] & pen_mask;
				rp[q] = (uint16_t)((base + p) & 0x1fff);
				rf[q] = p ? fsel : 0;
			}
		else
			for (int q = q0; q < q0 + n; q++) {
				uint8_t p = src[q] & pen_mask;
				rp[q] = (uint16_t)((base + p) & 0x1fff);
				rf[q] = p ? fsel : 0;
			}
		g += n;
	}
	if (direct)
		return;
	if (!pf->m.x_sample_enable || line->x_sample == 1) {   // (a sample of 1: mosaic(x) is x)
		int32_t acc = pf->reg_fx_x;
		for (int x = H_START; x < H_END; x++, acc += pf->x_scale) {
			int gx = ((acc >> 8) + H_START) & wmask;
			ll->pal[x] = fc->rowpal[gx];
			ll->f[x] = fc->rowf[gx];
		}
		return;
	}
	const int16_t *mx = mosaic_table(line);
	int lrx = -1, gx = 0;
	for (int x = H_START; x < H_END; x++) {
		const int rx = mx[x];
		if (rx != lrx) {   // (the same sample for a run of pixels)
			gx = (((pf->reg_fx_x + (rx - H_START) * pf->x_scale) >> 8) + H_START) & wmask;
			lrx = rx;
		}
		ll->pal[x] = fc->rowpal[gx];
		ll->f[x] = fc->rowf[gx];
	}
}

// fetch_pf's unzoomed path and mix_line at once, a tile at a time, over the screen x l..r-1 (a clip range of the
// playfield line; pbobble3 has two): a whole tile row whose 16 pixels this layer can change nothing of (every src >= prio and dst > prio there:
// the layers above have them) is neither fetched nor mixed; the others are mixed straight from the tile. The same
// results as fetch_pf + mix_line (pbobble3: its lowest playfield fully opaque, half of it under the others)
static inline __attribute__((always_inline)) void fetch_mix_pf_k(fetch_ctx *restrict fc, const pf_inf *pf, int tmap, mixbuf *restrict z,
                                                                layer_line *restrict ll, uint8_t prio, uint8_t bm, const uint8_t *restrict blend,
                                                                int l, int r)
{
	const f3v_t *v = fc->v;
	const int cols = v->extend ? 64 : 32, W = cols * 16, wmask = W - 1;
	const uint16_t *data = v->vram + (v->extend ? 0x1000 * tmap : 0x800 * tmap);
	const int gy = ((pf->reg_fx_y >> 8) + pf->colscroll) & 0x1ff;
	const int uy = fc->tm_flip ? 511 - gy : gy;
	const uint16_t *rowp = data + (uy >> 4) * cols * 2;
	const int py = uy & 15;
	const int g0 = ((pf->reg_fx_x) >> 8) + H_START;   // (the source column at H_START)
	int g1 = g0 + (r - 1 - H_START);
	if (g1 - g0 >= W)
		g1 = g0 + W - 1;
	const uint8_t *gfx = v->tile_gfx;
	uint8_t ok0, ok1;
	mix_ok(bm, blend, &ok0, &ok1);
	const u8x16 vp = BC8(prio);
	for (int g = g0 + (l - H_START); g <= g1; ) {
		const int gx = g & wmask;
		const int ux = fc->tm_flip ? wmask - gx : gx;
		const int q0 = gx & 15;
		int n = 16 - q0;
		if (n > g1 - g + 1)
			n = g1 - g + 1;
		const int sx = g - g0 + H_START;   // (the screen x of the run's first pixel)
		const uint16_t *t = rowp + (ux >> 4) * 2;
		const uint16_t attr = t[0], code = t[1];
		const uint32_t tc = code < v->tile_count ? code : code % v->tile_count;
		const int fy = (attr >> 15) & 1, ty = fy ? 15 - py : py;
		if ((v->tile_blank[tc] >> ty) & 1) {   // (nothing opaque: nothing to mix)
			g += n;
			continue;
		}
		if (n == 16) {
			u8x16 sp, dp;
			memcpy(&sp, z->src_prio + sx, 16);
			memcpy(&dp, z->dst_prio + sx, 16);
			if (!any16((u8x16)(sp < vp) | (u8x16)(dp <= vp))) {   // (covered: this layer changes none of the 16)
				g += 16;
				continue;
			}
		}
		const int pc = attr & 0x1ff, sel = (attr >> 9) & 1, extra = (attr >> 10) & 3;
		const uint8_t pen_mask = (uint8_t)((((extra & ~pc) & 3) << 4) | 0x0f);
		const int fx = (attr >> 14) & 1, rev = fx ^ fc->tm_flip;
		const uint8_t *src = gfx + (size_t)tc * 256 + ty * 16;
		const int base = pc * 16 + pf->pal_add;
		const uint8_t fsel = (uint8_t)(0x80 | sel);
		if (n == 16) {
			uint16_t rp[16];
			uint8_t rf[16];
			tile_row16(src, rev, pen_mask, base, fsel, rp, rf);
			u16x8 p0, p1;
			u8x16 f;
			memcpy(&p0, rp, 16);
			memcpy(&p1, rp + 8, 16);
			memcpy(&f, rf, 16);
			mix16(z, sx, p0, p1, (u8x16)(f != (u8x16){ 0 }), BC8(blend[sel]), BC8(blend[2 + sel]), BC8(sel ? ok1 : ok0), prio, bm);
		} else {   // (a part of a tile at the line's ends: through the layer line, a pixel at a time)
			for (int q = q0; q < q0 + n; q++) {
				const uint8_t p = src[rev ? 15 - q : q] & pen_mask;
				const int x = sx + q - q0;
				ll->pal[x] = (uint16_t)((base + p) & 0x1fff);
				ll->f[x] = p ? fsel : 0;
			}
			for (int x = sx; x < sx + n; x++)
				if (ll->f[x])
					mix_px(z, x, ll->pal[x], ll->f[x] & 1, prio, bm, blend);
		}
		g += n;
	}
}

static __attribute__((noinline)) void fetch_mix_pf(fetch_ctx *fc, const pf_inf *pf, int tmap, mixbuf *z, layer_line *ll, uint8_t prio,
                                                   uint8_t bm, const uint8_t *blend, int l, int r)
{
	const uint8_t b[4] = { blend[0], blend[1], blend[2], blend[3] };
	switch (bm) {
	case 0: fetch_mix_pf_k(fc, pf, tmap, z, ll, prio, 0, b, l, r); break;
	case 1: fetch_mix_pf_k(fc, pf, tmap, z, ll, prio, 1, b, l, r); break;
	case 2: fetch_mix_pf_k(fc, pf, tmap, z, ll, prio, 2, b, l, r); break;
	case 3: fetch_mix_pf_k(fc, pf, tmap, z, ll, prio, 3, b, l, r); break;
	default: fetch_mix_pf_k(fc, pf, tmap, z, ll, prio, bm, b, l, r); break;
	}
}

static inline uint8_t char_pixel(const uint16_t *tiles, int code, int px, int py)
{
	const uint16_t *r = tiles + code * 16 + py * 2;
	uint32_t row = (uint32_t)r[0] << 16 | r[1];
	return (uint8_t)((row >> (px * 4)) & 0xf);
}

// one 8-pixel cell at a time: the text layer (characters from char RAM) or the pixel layer (pivot RAM, the palette
// from the text layer as MAME get_tile_info_pixel, including its scroll offset hack). use_pix and flip (the tilemap
// flip) are constants in each caller's copy
static inline __attribute__((always_inline)) void fetch_pivot_cells(const f3v_t *v, const pv_inf *pv, int gy, layer_line *restrict ll,
                                                                    const mixbuf *restrict z, uint8_t prio, const int use_pix, const int flip)
{
	const uint16_t *text = v->vram + 0x6000, *chars = v->vram + 0x7000;
	const uint8_t fsel = (uint8_t)(0x80 | pv->blend_select_v);
	const int uy = flip ? (use_pix ? 255 : 511) - gy : gy;
	const int row = uy >> 3, py = uy & 7;
	const uint16_t *trow;
	if (use_pix) {
		int ty = row;
		int y_offs = row * 8 + v->ctrl1[5];
		if (flip)
			y_offs += 0x100;
		if ((y_offs & 0x1ff) >= 256)
			ty += 32;
		trow = text + (ty << 6);
	} else
		trow = text + row * 64;
	const uint16_t *pixrow = v->pivotram + row * 16;   // (cell col: + col * 32 * 16)
	const int sx = pv->reg_sx;
	int x = H_START;
	while (x < H_END) {
		const int gx = (x + sx) & 0x1ff;
		const int ux = flip ? 511 - gx : gx;
		const int col = ux >> 3;
		const uint16_t t = trow[col];
		const int ry = (t & 0x8000) ? 7 - py : py;
		const uint16_t *r = use_pix ? pixrow + col * 32 * 16 + ry * 2 : chars + (t & 0xff) * 16 + ry * 2;
		uint32_t bits = (uint32_t)r[0] << 16 | r[1];
		// screen pixels of this cell: ux moves by +1 (or -1 when flipped) per screen pixel
		int left = flip ? (ux & 7) + 1 : 8 - (ux & 7);
		if (left > H_END - x)
			left = H_END - x;
		if (!bits) {   // (the colours of transparent pixels are never used)
			if (left == 8)
				memset(ll->f + x, 0, 8);
			else
				memset(ll->f + x, 0, (size_t)left);
			x += left;
			continue;
		}
		if (left == 8) {   // (the 8 pixels taken by the layers above: this layer changes none of them)
			v8u8 sp, dp;
			memcpy(&sp, z->src_prio + x, 8);
			memcpy(&dp, z->dst_prio + x, 8);
			const v8u8 need = (v8u8)(sp < prio) | (v8u8)(dp <= prio);
			uint64_t nm;
			memcpy(&nm, &need, 8);
			if (nm == 0) {
				memset(ll->f + x, 0, 8);
				x += 8;
				continue;
			}
		}
		const int color16 = ((t >> 9) & 0x3f) * 16;
		// pixel px of the cell for screen pixel i: (ux & 7) +- i, 7 - that with the cell flipped; with the
		// nibbles reversed when it goes backwards the pixels are the bits from s up
		const int xflip = (t & 0x100) != 0;
		int s0 = xflip ? 7 - (ux & 7) : ux & 7;
		if (flip ^ xflip) {
			bits = __builtin_bswap32(bits);
			bits = ((bits >> 4) & 0x0f0f0f0fu) | ((bits << 4) & 0xf0f0f0f0u);
			s0 = 7 - s0;
		}
		bits >>= s0 * 4;
		if (left == 8) {   // (a whole cell: the 8 nibbles spread to bytes at once)
			const uint32_t lo = bits & 0x0f0f0f0fu, hi = (bits >> 4) & 0x0f0f0f0fu;
			v8u8 a8, p8;
			memcpy(&a8, &lo, 4);
			memcpy((uint8_t *)&a8 + 4, &hi, 4);
			p8 = __builtin_shuffle(a8, (v8u8){ 0, 4, 1, 5, 2, 6, 3, 7 });
			const v8u16 w = __builtin_convertvector(p8, v8u16) + (uint16_t)color16;
			const v8u8 f8 = (v8u8)(p8 != 0) & fsel;
			memcpy(ll->pal + x, &w, 16);
			memcpy(ll->f + x, &f8, 8);
		} else
			for (int i = 0; i < left; i++, bits >>= 4) {
				const uint8_t p = (uint8_t)(bits & 0xf);
				ll->pal[x + i] = (uint16_t)(color16 + p);
				ll->f[x + i] = p ? fsel : 0;
			}
		x += left;
	}
}

// z, prio: the mix so far and this layer's priority: a whole cell this layer can change nothing of is left empty
static void fetch_pivot(fetch_ctx *fc, const pv_inf *pv, int y_line, line_inf *line, layer_line *ll, const mixbuf *z, uint8_t prio)
{
	const f3v_t *v = fc->v;
	const uint16_t *text = v->vram + 0x6000, *chars = v->vram + 0x7000;
	int use_pix = (pv->pivot_control & 0xa0) != 0;
	int gy = (pv->reg_sy + y_line) & (use_pix ? 0xff : 0x1ff);
	int sel = pv->blend_select_v;
	if (!pv->m.x_sample_enable || line->x_sample == 1) {
		switch ((use_pix << 1) | (fc->tm_flip != 0)) {
		case 0: fetch_pivot_cells(v, pv, gy, ll, z, prio, 0, 0); break;
		case 1: fetch_pivot_cells(v, pv, gy, ll, z, prio, 0, 1); break;
		case 2: fetch_pivot_cells(v, pv, gy, ll, z, prio, 1, 0); break;
		default: fetch_pivot_cells(v, pv, gy, ll, z, prio, 1, 1); break;
		}
		return;
	}
	for (int x = H_START; x < H_END; x++) {
		int rx = pv->m.x_sample_enable ? mosaic_table(line)[x] : x;
		int gx = (rx + pv->reg_sx) & 0x1ff;
		uint8_t p;
		int color;
		if (!use_pix) {
			int ux = fc->tm_flip ? 511 - gx : gx, uy = fc->tm_flip ? 511 - gy : gy;
			uint16_t t = text[(uy >> 3) * 64 + (ux >> 3)];
			int px = ux & 7, py = uy & 7;
			if (t & 0x100) px = 7 - px;
			if (t & 0x8000) py = 7 - py;
			p = char_pixel(chars, t & 0xff, px, py);
			color = (t >> 9) & 0x3f;
		} else {
			int ux = fc->tm_flip ? 511 - gx : gx, uy = fc->tm_flip ? 255 - gy : gy;
			int col = ux >> 3, row = uy >> 3;
			// palette from the text layer (MAME get_tile_info_pixel, including its scroll offset hack)
			int ty = row;
			int y_offs = row * 8 + v->ctrl1[5];
			if (fc->tm_flip)
				y_offs += 0x100;
			if ((y_offs & 0x1ff) >= 256)
				ty += 32;
			uint16_t t = text[(ty << 6) | col];
			int px = ux & 7, py = uy & 7;
			if (t & 0x100) px = 7 - px;
			if (t & 0x8000) py = 7 - py;
			p = char_pixel(v->pivotram, col * 32 + row, px, py);
			color = (t >> 9) & 0x3f;
		}
		ll->pal[x] = (uint16_t)(color * 16 + p);
		ll->f[x] = p ? (uint8_t)(0x80 | sel) : 0;
	}
}

// ---- output: blended colours go to palette entries after the F3 palette ----
// blended output per (source, destination, weights) within a frame: the palette and its blended entries are fixed for the
// frame, so the result of a combination can be reused (a direct-mapped table, tagged with the frame's generation)
#define BC_SIZE 2048
typedef struct { uint32_t pals; uint16_t out, gen_w; } blend_entry;   // gen_w = generation << 8 | sb << 4 | db
// each drawing thread has its own blended colour hash and its own range of palette entries
typedef struct {
	uint32_t *hkey;
	uint16_t *hval, *hgen;
	uint16_t gen;
	uint32_t next, end;
	blend_entry bc[BC_SIZE];
} blend_state;

static void blend_begin(blend_state *b, uint32_t start, uint32_t end)
{
	if ((b->gen & 0xff) == 0xff)
		memset(b->bc, 0, sizeof(b->bc));   // (the table's tags hold 8 bits of the generation)
	if (++b->gen == 0) {
		memset(b->hgen, 0, 0x10000 * 2);
		b->gen = 1;
	}
	b->next = start;
	b->end = end;
}

// The frontend's frame written here as the lines are drawn (rbx_video.out: each drawing thread its own lines), with
// the colors converted once: the palette's when they change, a blended one when it gets its entry
static uint32_t s_outlut[F3V_PALETTE_SIZE];
static uint16_t s_outlut16[F3V_PAL_COLORS];   // (the palette's, 16-bit: the RGB565 lines' table, half the cache)

static uint32_t (*s_out_color)(int r, int g, int b);
static int s_out_bpp;
static uint8_t *s_out;
static int s_out_pitch;

static uint16_t blend_index(blend_state *bs, uint32_t *palette, uint32_t rgb, uint16_t fallback)
{
	uint32_t h = (rgb * 2654435761u) >> 16;
	for (int probe = 0; probe < 64; probe++, h = (h + 1) & 0xffff) {
		if (bs->hgen[h] != bs->gen) {
			if (bs->next >= bs->end)
				return fallback;
			bs->hgen[h] = bs->gen;
			bs->hkey[h] = rgb;
			bs->hval[h] = (uint16_t)bs->next;
			palette[bs->next] = rgb;
			if (s_out)
				s_outlut[bs->next] = s_out_color((int)(rgb >> 16) & 0xff, (int)(rgb >> 8) & 0xff, (int)rgb & 0xff);
			return (uint16_t)bs->next++;
		}
		if (bs->hkey[h] == rgb)
			return bs->hval[h];
	}
	return fallback;
}

#ifndef __vita__
static int s_outchk;
static unsigned s_outchk_diff, s_outchk_px;
static void outchk_report(void) { fprintf(stderr, "OUTCHK %u of %u pixels differ\n", s_outchk_diff, s_outchk_px); }
#endif
static void render_line(const f3v_t *v, blend_state *bs, const mixbuf *z, uint16_t *dst, uint32_t *palette)
{
	const uint16_t gtag = (uint16_t)((bs->gen & 0xff) << 8);
	const uint32_t *clut = v->pal_rgb;
	uint16_t lsp = 0xffff, ldp = 0xffff, last_out = 0;
	uint8_t lsb = 0xff, ldb = 0xff;
	for (int x0 = H_START; x0 < H_END; x0 += 8) {
		// 8 pixels at once when each takes one of its two entries as it is (one layer only, or both the same); else
		// the 8 one by one
		v8u16 vs, vd;
		v8u8 bs8, bd8;
		memcpy(&vs, z->src_pal + x0, 16);
		memcpy(&vd, z->dst_pal + x0, 16);
		memcpy(&bs8, z->src_blend + x0, 8);
		memcpy(&bd8, z->dst_blend + x0, 8);
		vs &= 0x1fff;
		vd &= 0x1fff;
		const v8u16 vsb = __builtin_convertvector(bs8, v8u16), vdb = __builtin_convertvector(bd8, v8u16);
		const v8u16 take_s = (v8u16)((vsb == 8) & (vdb == 0)), take_d = (v8u16)((vsb == 0) & (vdb == 8)) & ~take_s;
		const v8u16 same = (v8u16)((vs == vd) & (vsb + vdb == 8));
		const v8u16 simple = take_s | take_d | same;
		uint64_t m[2];
		memcpy(m, &simple, 16);
		if ((m[0] & m[1]) == ~0ull) {
			const v8u16 out8 = (take_d & vd) | (~take_d & vs);
			memcpy(dst + (x0 - H_START), &out8, 16);
			continue;
		}
		for (int x = x0; x < x0 + 8; x++) {
			uint16_t sp = z->src_pal[x] & 0x1fff, dp = z->dst_pal[x] & 0x1fff;
			uint8_t sb = z->src_blend[x], db = z->dst_blend[x];
			uint16_t out;
			if (db == 0 && sb == 8)
				out = sp;
			else if (sb == 0 && db == 8)
				out = dp;
			else if (sp == dp && sb + db == 8)
				out = sp;
			else if (sp == lsp && dp == ldp && sb == lsb && db == ldb)
				out = last_out;
			else {
				const uint32_t pals = 0x80000000u | (uint32_t)sp << 16 | dp;   // (bit 31: a used entry)
				const uint16_t gw = (uint16_t)(gtag | sb << 4 | db);
				blend_entry *be = &bs->bc[(sp * 0x9e5u ^ dp * 0x3b1u ^ (uint32_t)gw * 0x61u) & (BC_SIZE - 1)];
				if (be->pals == pals && be->gen_w == gw) {
					out = be->out;
					lsp = sp; ldp = dp; lsb = sb; ldb = db;
					last_out = out;
					dst[x - H_START] = out;
					continue;
				}
				uint32_t s = clut[sp], d = clut[dp];
				uint32_t r = (((s >> 16) & 0xff) * sb + ((d >> 16) & 0xff) * db) >> 3;
				uint32_t g = (((s >> 8) & 0xff) * sb + ((d >> 8) & 0xff) * db) >> 3;
				uint32_t b = ((s & 0xff) * sb + (d & 0xff) * db) >> 3;
				if (r > 255) r = 255;
				if (g > 255) g = 255;
				if (b > 255) b = 255;
				uint32_t rgb = r << 16 | g << 8 | b;
				if (rgb == s)
					out = sp;
				else if (rgb == d)
					out = dp;
				else
					out = blend_index(bs, palette, rgb, sp);
				lsp = sp; ldp = dp; lsb = sb; ldb = db;
				last_out = out;
				be->pals = pals;
				be->gen_w = gw;
				be->out = out;
			}
			dst[x - H_START] = out;
		}
	}
}

// render_line straight into an RGB565 frame (the frontend's, rbx_video.out): the same colours without the palette
// indices (a blended colour from its two entries as render_line computes it, then to 565 as the frontend's out_color;
// render_line's blend palette, when full, gives the source colour instead: here the blend still)
static int s_out565;   // (out_color is RGB565 by its values: the line goes straight there)
static inline uint16_t rgb565(uint32_t rgb) { return (uint16_t)(((rgb >> 8) & 0xf800) | ((rgb >> 5) & 0x07e0) | ((rgb >> 3) & 0x001f)); }
// the 8 pixels from x0 when some of them blend: all 8 blended at once, then the simple ones from the table
static __attribute__((noinline)) void out565_blend8(const uint32_t *clut, const mixbuf *z, int x0, uint16_t *d)
{
	{
		v8u16 vs, vd;
		v8u8 bs8, bd8;
		memcpy(&vs, z->src_pal + x0, 16);
		memcpy(&vd, z->dst_pal + x0, 16);
		memcpy(&bs8, z->src_blend + x0, 8);
		memcpy(&bd8, z->dst_blend + x0, 8);
		vs &= 0x1fff;
		vd &= 0x1fff;
		const v8u16 vsb = __builtin_convertvector(bs8, v8u16), vdb = __builtin_convertvector(bd8, v8u16);
		const v8u16 take_s = (v8u16)((vsb == 8) & (vdb == 0)), take_d = (v8u16)((vsb == 0) & (vdb == 8)) & ~take_s;
		const v8u16 same = (v8u16)((vs == vd) & (vsb + vdb == 8));
		const v8u16 simple = take_s | take_d | same;
		const v8u16 idx = (take_d & vd) | (~take_d & vs);
		uint16_t sr[8], sg[8], sbl[8], dr[8], dg[8], dbl[8], lut[8];
		for (int i = 0; i < 8; i++) {
			const uint32_t sc = clut[vs[i]], dc = clut[vd[i]];
			sr[i] = (uint16_t)((sc >> 16) & 0xff);
			sg[i] = (uint16_t)((sc >> 8) & 0xff);
			sbl[i] = (uint16_t)(sc & 0xff);
			dr[i] = (uint16_t)((dc >> 16) & 0xff);
			dg[i] = (uint16_t)((dc >> 8) & 0xff);
			dbl[i] = (uint16_t)(dc & 0xff);
			lut[i] = s_outlut16[idx[i]];
		}
		v8u16 vsr, vsg, vsbl, vdr, vdg, vdbl, vlut;
		memcpy(&vsr, sr, 16);
		memcpy(&vsg, sg, 16);
		memcpy(&vsbl, sbl, 16);
		memcpy(&vdr, dr, 16);
		memcpy(&vdg, dg, 16);
		memcpy(&vdbl, dbl, 16);
		memcpy(&vlut, lut, 16);
		v8u16 r = (vsr * vsb + vdr * vdb) >> 3, g = (vsg * vsb + vdg * vdb) >> 3, b = (vsbl * vsb + vdbl * vdb) >> 3;
		r |= (v8u16)(r > 255);   // (over 255: all ones, and 0xff after the masks below)
		g |= (v8u16)(g > 255);
		b |= (v8u16)(b > 255);
		r &= 0xff;
		g &= 0xff;
		b &= 0xff;
		const v8u16 c565 = ((r & 0xf8) << 8) | ((g & 0xfc) << 3) | (b >> 3);
		const v8u16 out = (simple & vlut) | (~simple & c565);
		memcpy(d, &out, 16);
	}
}

// Two passes: the vector work of the whole line into memory first (each pixel's entry, the source's or the
// destination's, and whether it is taken as it is), then the table lookups by the integer pipeline. (One pass read
// each 8-pixel test and the 8 entries out of NEON registers: on the A9 a NEON-to-ARM move waits for the NEON queue.)
static void render_line_565(const f3v_t *v, const mixbuf *z, uint16_t *o)
{
	uint16_t idx[H_VIS] __attribute__((aligned(16)));
	uint8_t simple[H_VIS] __attribute__((aligned(8)));
	for (int x0 = H_START; x0 < H_END; x0 += 8) {
		v8u16 vs, vd;
		v8u8 bs8, bd8;
		memcpy(&vs, z->src_pal + x0, 16);
		memcpy(&vd, z->dst_pal + x0, 16);
		memcpy(&bs8, z->src_blend + x0, 8);
		memcpy(&bd8, z->dst_blend + x0, 8);
		vs &= 0x1fff;
		vd &= 0x1fff;
		const v8u16 vsb = __builtin_convertvector(bs8, v8u16), vdb = __builtin_convertvector(bd8, v8u16);
		const v8u16 take_s = (v8u16)((vsb == 8) & (vdb == 0)), take_d = (v8u16)((vsb == 0) & (vdb == 8)) & ~take_s;
		const v8u16 same = (v8u16)((vs == vd) & (vsb + vdb == 8));
		const v8u16 ix = (take_d & vd) | (~take_d & vs);
		const v8u8 s8 = __builtin_convertvector(take_s | take_d | same, v8u8);
		memcpy(idx + (x0 - H_START), &ix, 16);
		memcpy(simple + (x0 - H_START), &s8, 8);
	}
	const uint16_t *lut = s_outlut16;
	for (int x = 0; x < H_VIS; x += 8) {
		uint32_t m0, m1;
		memcpy(&m0, simple + x, 4);
		memcpy(&m1, simple + x + 4, 4);
		if ((m0 & m1) == 0xffffffffu) {
			const uint16_t *ip = idx + x;
			uint16_t *d = o + x;
			d[0] = lut[ip[0]]; d[1] = lut[ip[1]]; d[2] = lut[ip[2]]; d[3] = lut[ip[3]];
			d[4] = lut[ip[4]]; d[5] = lut[ip[5]]; d[6] = lut[ip[6]]; d[7] = lut[ip[7]];
		} else
			out565_blend8(v->pal_rgb, z, x + H_START, o + x);
	}
}

// ---- frame ----
enum { L_PIVOT, L_SP, L_PF };

// one drawing thread's buffers
typedef struct {
	line_inf line;
	mixbuf z;
	layer_line ll;
	fetch_ctx fc;
	blend_state bs;
	unsigned job;
} f3v_worker;
static f3v_worker s_w[3];   // [0] the render thread, [1] [2] helpers (f3v_help: the waiting emulation thread, a spare core)
static const uint32_t W_RANGE[4] = { F3V_PAL_COLORS, 0x7000, 0xb000, F3V_PALETTE_SIZE };   // blended palette entries of each
static line_inf s_lines[256];   // line states of the frame (LINE_STATE_SIZE bytes of each)

// the frame's job: the visible lines, handed out JOB_CHUNK at a time to the render thread and a helper
#define JOB_CHUNK 8
static struct { f3v_t *v; const fetch_ctx *fc; uint16_t *bitmap; int pitch; uint32_t *palette; int y1; } s_job;
static int s_job_active, s_job_next, s_job_left, s_helpers;
static unsigned s_job_id;

// RBX_F3V_PROF (a device profiling build): the drawing's time by stage, summed over its threads, logged every 120 frames
#ifdef RBX_F3V_PROF
static uint64_t s_pt[6];
static const char *const s_ptn[6] = { "fetch", "mix", "spmix", "out", "spdraw", "line" };
#define PT0() uint64_t pt0_ = rbx_clock_us()
#define PTA(k) do { const uint64_t pt1_ = rbx_clock_us(); __atomic_add_fetch(&s_pt[k], pt1_ - pt0_, __ATOMIC_RELAXED); pt0_ = pt1_; } while (0)
#else
#define PT0() do { } while (0)
#define PTA(k) do { } while (0)
#endif

// a layer of priority p changes a pixel only when p > src or p >= dst there (mix_px): with every src >= p and every
// dst > p it changes nothing on the line. Stops at the first 16 pixels that say otherwise (most lines, at once)
static int changes_nothing(const mixbuf *z, uint8_t p)
{
	const u8x16 vp = BC8(p);
	for (int x = H_START; x < H_END; x += 16) {
		u8x16 s, d;
		memcpy(&s, z->src_prio + x, 16);
		memcpy(&d, z->dst_prio + x, 16);
		if (any16((u8x16)(s < vp) | (u8x16)(d <= vp)))
			return 0;
	}
	return 1;
}

static void draw_line(f3v_t *v, f3v_worker *w, int screen_y, uint16_t *bitmap, int pitch, uint32_t *palette)
{
	PT0();
	static const uint8_t ltype[9] = { L_PIVOT, L_SP, L_PF, L_SP, L_PF, L_SP, L_PF, L_SP, L_PF };
	static const uint8_t lidx[9] = { 0, 0, 0, 3, 3, 2, 2, 1, 1 };
	memcpy(&w->line, &s_lines[screen_y], LINE_STATE_SIZE);
		// layer order, stable sort by priority (high first)
		uint8_t order[9], prios[9];
		for (int i = 0; i < 9; i++) {
			order[i] = (uint8_t)i;
			prios[i] = ltype[i] == L_PIVOT ? w->line.pivot.m.prio : ltype[i] == L_SP ? w->line.sp[lidx[i]].m.prio : w->line.pf[lidx[i]].m.prio;
		}
		for (int i = 1; i < 9; i++) {
			uint8_t o = order[i];
			int j = i - 1;
			while (j >= 0 && prios[order[j]] < prios[o]) {
				order[j + 1] = order[j];
				j--;
			}
			order[j + 1] = o;
		}
		// (mixing and output only touch the visible columns)
		for (int x = H_START; x < H_END; x++)
			w->z.dst_pal[x] = w->line.bg_palette;
		memset(w->z.dst_blend + H_START, 8, H_VIS);
		memset(w->z.src_bm + H_START, 0xff, H_VIS);
		memset(w->z.src_pal + H_START, 0, H_VIS * 2);
		memset(w->z.src_blend + H_START, 0, H_VIS);
		memset(w->z.src_prio + H_START, 0, H_VIS);
		memset(w->z.dst_prio + H_START, 0, H_VIS);
#ifndef __vita__
		static int lmask = -1;   // (PC harness RBX_F3_LAYERS=mask: bit 0 pivot, 1-4 sprites 0-3, 5-8 playfields 0-3)
		if (lmask < 0) { const char *e = getenv("RBX_F3_LAYERS"); lmask = e ? (int)strtol(e, NULL, 0) : 0x1ff; }
#endif
		int mixed = 0;   // (a layer mixed since the lowest priorities were last looked at)
		for (int k = 0; k < 9; k++) {
			int li = order[k], idx = lidx[li];
			const mixable *m;
			int used;
			if (mixed) {   // the layers go high priority first: once one can change nothing, none of the rest can
				mixed = 0;
				if (changes_nothing(&w->z, prios[li]))
					break;
			}
#ifndef __vita__
			{ static const uint8_t bit[9] = { 0, 1, 5, 4, 8, 3, 7, 2, 6 };
			  if (!((lmask >> bit[li]) & 1)) continue; }
#endif
			if (ltype[li] == L_PIVOT) {
				m = &w->line.pivot.m;
				if (!((m->mix & 0x2000) && m->blend_mode != 3))
					continue;
				int use_pix = (w->line.pivot.pivot_control & 0xa0) != 0;
				int yi = (w->line.pivot.reg_sy + screen_y) & (use_pix ? 0xff : 0x1ff);
				int y_adj = v->flipscreen ? 0x1ff - yi : yi;
				if (use_pix) {   // (fetch_pivot's cell row)
					int uy = w->fc.tm_flip ? 255 - yi : yi;
					used = w->fc.pix_row_usage[uy >> 3];
				} else
					used = w->fc.text_row_usage[(y_adj >> 3) & 63] > 0;
			} else if (ltype[li] == L_SP) {
				m = &w->line.sp[idx].m;
				if (!((m->mix & 0x2000) && m->blend_mode != 0))
					continue;
				used = (v->sprite_row_usage[screen_y] >> idx) & 1;
			} else {
				m = &w->line.pf[idx].m;
				if (!((m->mix & 0x2000) && m->blend_mode != 3))
					continue;
				int yi = ((w->line.pf[idx].reg_fx_y >> 8) + w->line.pf[idx].colscroll) & 0x1ff;
				int y_adj = v->flipscreen ? 0x1ff - yi : yi;
				used = w->fc.pf_row_usage[(y_adj >> 4) & 31][idx + 2 * w->line.pf[idx].alt_tilemap] > 0;
			}
			if (!used)
				continue;
			clip_t ranges[8];
			int nr = calc_clip(w->line.clip, m->mix, ranges);
			if (!nr)
				continue;
			if (ltype[li] == L_SP) {
				const sp_inf *sp = &w->line.sp[idx];
				const int16_t *mx = sp->m.x_sample_enable && w->line.x_sample != 1 ? mosaic_table(&w->line) : NULL;
				mixed = 1;
				for (int r = 0; r < nr; r++) {
					int l = ranges[r].l < H_START ? H_START : ranges[r].l;
					int rr = ranges[r].r > H_END ? H_END : ranges[r].r;
					if (l < rr)
						mix_sprites(&w->z, v->fb + screen_y * FB_W, mx, idx, sp->blend_select_v, m->prio, m->blend_mode,
						            w->line.blend, l, rr);
				}
				PTA(2);
				continue;
			}
			if (ltype[li] == L_PF && !w->line.pf[idx].m.x_sample_enable && w->line.pf[idx].x_scale == 256) {
				// (unzoomed: fetched and mixed at once, each clip range)
				int tmap = idx + ((!v->extend && w->line.pf[idx].alt_tilemap) ? 2 : 0);
				for (int r = 0; r < nr; r++) {
					int l = ranges[r].l < H_START ? H_START : ranges[r].l;
					int rr = ranges[r].r > H_END ? H_END : ranges[r].r;
					if (l < rr)
						fetch_mix_pf(&w->fc, &w->line.pf[idx], tmap, &w->z, &w->ll, m->prio, m->blend_mode, w->line.blend, l, rr);
				}
				mixed = 1;
				PTA(1);
				continue;
			}
			if (ltype[li] == L_PIVOT)
				fetch_pivot(&w->fc, &w->line.pivot, screen_y, &w->line, &w->ll, &w->z, m->prio);
			else {
				int tmap = idx + ((!v->extend && w->line.pf[idx].alt_tilemap) ? 2 : 0);
				fetch_pf(&w->fc, &w->line.pf[idx], tmap, screen_y, &w->line, &w->ll);
			}
			PTA(0);
			mixed = 1;
			for (int r = 0; r < nr; r++) {
				int l = ranges[r].l < H_START ? H_START : ranges[r].l;
				int rr = ranges[r].r > H_END ? H_END : ranges[r].r;
				if (l < rr)
					mix_line(&w->z, &w->ll, m->prio, m->blend_mode, w->line.blend, l, rr);
			}
			PTA(1);
		}
	uint16_t *row = bitmap + (screen_y - v->vis_y0) * pitch;
	if (s_out && s_out565) {   // (the frontend's frame only: the bitmap is not used then)
		PTA(5);
		render_line_565(v, &w->z, (uint16_t *)(s_out + (screen_y - v->vis_y0) * s_out_pitch));
		PTA(3);
#ifndef __vita__
		if (s_outchk) {   // (PC harness RBX_OUTCHK: the same as render_line and the conversion?)
			uint16_t ref[H_VIS];
			render_line(v, &w->bs, &w->z, row, palette);
			const uint16_t *o = (const uint16_t *)(s_out + (screen_y - v->vis_y0) * s_out_pitch);
			for (int x = 0; x < H_VIS; x++) {
				ref[x] = (uint16_t)s_outlut[row[x]];
				if (ref[x] != o[x])
					__atomic_add_fetch(&s_outchk_diff, 1, __ATOMIC_RELAXED);
			}
			__atomic_add_fetch(&s_outchk_px, H_VIS, __ATOMIC_RELAXED);
		}
#endif
		return;
	}
	render_line(v, &w->bs, &w->z, row, palette);
	if (s_out) {
		uint8_t *o = s_out + (screen_y - v->vis_y0) * s_out_pitch;
		if (s_out_bpp == 2)
			for (int x = 0; x < H_VIS; x++)
				((uint16_t *)o)[x] = (uint16_t)s_outlut[row[x]];
		else
			for (int x = 0; x < H_VIS; x++)
				((uint32_t *)o)[x] = s_outlut[row[x]];
	}
}

static int workers_init(const f3v_t *v)
{
	memset(s_w, 0, sizeof(s_w));
	s_w[0].bs.hkey = v->hkey;
	s_w[0].bs.hval = v->hval;
	s_w[0].bs.hgen = v->hgen;
	for (int i = 1; i < 3; i++) {
		s_w[i].bs.hkey = (uint32_t *)calloc(0x10000, 4);
		s_w[i].bs.hval = (uint16_t *)calloc(0x10000, 2);
		s_w[i].bs.hgen = (uint16_t *)calloc(0x10000, 2);
		if (!s_w[i].bs.hkey || !s_w[i].bs.hval || !s_w[i].bs.hgen)
			return -1;
	}
	return 0;
}

static void workers_exit(void)
{
	for (int i = 1; i < 3; i++) {
		free(s_w[i].bs.hkey);
		free(s_w[i].bs.hval);
		free(s_w[i].bs.hgen);
	}
	memset(s_w, 0, sizeof(s_w));
}

static void worker_begin(f3v_worker *w, int i)
{
	if (w->job == s_job_id)
		return;
	w->job = s_job_id;
	const fetch_ctx *fc = s_job.fc;
	w->fc.v = fc->v;
	w->fc.tm_flip = fc->tm_flip;
	memcpy(w->fc.pf_row_usage, fc->pf_row_usage, sizeof(fc->pf_row_usage));
	memcpy(w->fc.text_row_usage, fc->text_row_usage, sizeof(fc->text_row_usage));
	memcpy(w->fc.pix_row_usage, fc->pix_row_usage, sizeof(fc->pix_row_usage));
	blend_begin(&w->bs, W_RANGE[i], W_RANGE[i + 1]);
}

static int s_spr_done;   // the last sprite row drawn (sprite lag 0: the render thread draws them in bands as lines go)

static void job_work(f3v_worker *w)
{
	for (;;) {
		int y = __atomic_fetch_add(&s_job_next, JOB_CHUNK, __ATOMIC_ACQ_REL);
		if (y > s_job.y1)
			return;
		int e = y + JOB_CHUNK - 1 < s_job.y1 ? y + JOB_CHUNK - 1 : s_job.y1;
		while (__atomic_load_n(&s_spr_done, __ATOMIC_ACQUIRE) < e) {
		}
		for (int sy = y; sy <= e; sy++)
			draw_line(s_job.v, w, sy, s_job.bitmap, s_job.pitch, s_job.palette);
		__atomic_sub_fetch(&s_job_left, e - y + 1, __ATOMIC_ACQ_REL);
	}
}

// one piece of the frame's lines (a thread with other work between: F3V_HELP_ONE with the helper number)
static void job_some(f3v_worker *w)
{
	const int y = __atomic_fetch_add(&s_job_next, 2, __ATOMIC_ACQ_REL);
	if (y > s_job.y1)
		return;
	const int e = y + 1 < s_job.y1 ? y + 1 : s_job.y1;
	while (__atomic_load_n(&s_spr_done, __ATOMIC_ACQUIRE) < e) {
	}
	for (int sy = y; sy <= e; sy++)
		draw_line(s_job.v, w, sy, s_job.bitmap, s_job.pitch, s_job.palette);
	__atomic_sub_fetch(&s_job_left, e - y + 1, __ATOMIC_ACQ_REL);
}

int f3v_help(int helper)
{
	const int one = helper & F3V_HELP_ONE;
	helper &= ~F3V_HELP_ONE;
	if (helper < 1 || helper > 2)
		return 0;
	if (!__atomic_load_n(&s_job_active, __ATOMIC_SEQ_CST))
		return 0;
	__atomic_add_fetch(&s_helpers, 1, __ATOMIC_SEQ_CST);
	int did = 0;
	if (__atomic_load_n(&s_job_active, __ATOMIC_SEQ_CST)) {
		worker_begin(&s_w[helper], helper);
		if (one)
			job_some(&s_w[helper]);
		else
			job_work(&s_w[helper]);
		did = 1;
	}
	__atomic_sub_fetch(&s_helpers, 1, __ATOMIC_SEQ_CST);
	return did;
}

// side: the render thread's work besides the lines, done while the helpers start on them: 1 the sprite list and
// sprites (sprite lag 1), 2 the sprites (lag 2), into the other frame buffer; it is the lines' one when they are done.
// 3: this frame's sprites (lag 0) in bands from the top, each band's lines once it is drawn
static void scanline_draw(f3v_t *v, fetch_ctx *fc, uint16_t *bitmap, int pitch, uint32_t *palette, int side)
{
	// the line states first (the line RAM latches carry from line to line), then the lines in any order
	static line_inf line;
	memset(&line, 0, LINE_STATE_SIZE);
	line.x_sample = 16;
	for (int pf = 0; pf < NUM_PF; pf++) {
		line.pf[pf].x_scale = 0x80;
		get_pf_scroll(v, pf, &line.pf[pf].reg_sx, &line.pf[pf].reg_sy);
		line.pf[pf].reg_fx_y = line.pf[pf].reg_sy;
	}
	if (v->flipscreen) {
		line.pivot.reg_sx = v->ctrl1[4] - 12;
		line.pivot.reg_sy = v->ctrl1[5];
	} else {
		line.pivot.reg_sx = -(int32_t)v->ctrl1[4] - 5;
		line.pivot.reg_sy = -(int32_t)v->ctrl1[5];
	}
	for (int screen_y = 0; screen_y < 256; screen_y++) {
		int y = v->flipscreen ? 255 - screen_y : screen_y;
		read_line_ram(v, &line, y);
		for (int i = 0; i < NUM_PF; i++) {
			pf_inf *pf = &line.pf[i];
			pf->reg_fx_x = pf->reg_sx + pf->rowscroll;
			pf->reg_fx_x += 10 * (pf->x_scale - (1 << 8));
		}
		if (screen_y >= v->vis_y0 && screen_y <= v->vis_y1)
			memcpy(&s_lines[screen_y], &line, LINE_STATE_SIZE);
		if (screen_y != 0)
			for (int i = 0; i < NUM_PF; i++)
				line.pf[i].reg_fx_y += line.pf[i].y_scale;
	}
	s_job.v = v;
	s_job.fc = fc;
	s_job.bitmap = bitmap;
	s_job.pitch = pitch;
	s_job.palette = palette;
	s_job.y1 = v->vis_y1;
	s_job_next = v->vis_y0;
	s_job_left = v->vis_y1 - v->vis_y0 + 1;
	__atomic_add_fetch(&s_job_id, 1, __ATOMIC_SEQ_CST);
	if (side == 3 && !v->sprite_trails)
		memset(v->sru_draw, 0, 256);
	__atomic_store_n(&s_spr_done, side == 3 ? v->vis_y0 - 1 : 1 << 30, __ATOMIC_RELEASE);
	__atomic_store_n(&s_job_active, 1, __ATOMIC_SEQ_CST);
	if (side == 3)
		for (int y = v->vis_y0; y <= v->vis_y1; y += 16) {
			PT0();
			draw_sprites_rows(v, y, y + 15);
			PTA(4);
			__atomic_store_n(&s_spr_done, y + 15, __ATOMIC_RELEASE);
		}
	else if (side) {
		const int o = v->fb == v->fbs[0];
		v->fb_draw = v->fbs[o];
		v->sru_draw = v->sru[o];
		PT0();
		if (side == 1)
			get_sprite_info(v);
		draw_sprites(v);
		PTA(4);
	}
	worker_begin(&s_w[0], 0);
	job_work(&s_w[0]);
	__atomic_store_n(&s_job_active, 0, __ATOMIC_SEQ_CST);
	// (a helper may still be drawing its last lines)
	while (__atomic_load_n(&s_job_left, __ATOMIC_SEQ_CST) > 0 || __atomic_load_n(&s_helpers, __ATOMIC_SEQ_CST) > 0) {
	}
#ifdef RBX_F3V_PROF
	{
		static int n;
		if (++n == 120) {
			n = 0;
			rbx_log("f3vprof (ms a frame, all threads): %s %.2f %s %.2f %s %.2f %s %.2f %s %.2f %s %.2f", s_ptn[0], s_pt[0] / 120000.0,
			        s_ptn[1], s_pt[1] / 120000.0, s_ptn[2], s_pt[2] / 120000.0, s_ptn[3], s_pt[3] / 120000.0, s_ptn[4],
			        s_pt[4] / 120000.0, s_ptn[5], s_pt[5] / 120000.0);
			memset(s_pt, 0, sizeof(s_pt));
		}
	}
#endif
	if (side == 1 || side == 2) {
		v->fb = v->fb_draw;
		v->sprite_row_usage = v->sru_draw;
		v->flipscreen = v->sflip;
	}
}

int f3v_render(f3v_t *v, uint16_t *bitmap, int pitch, uint32_t *palette)
{
	static fetch_ctx fc;
	fc.v = v;
	fc.tm_flip = v->flipscreen;   // MAME: tilemap flip is set from the previous frame's flipscreen
	// palette: the blocks of 8 entries that changed since the last frame (s_palprev)
	static uint8_t dirty[F3V_PAL_COLORS / 8];
	for (int bl = 0; bl < F3V_PAL_COLORS / 8; bl++) {
		const uint16_t *pw = v->palram + bl * 16;
		dirty[bl] = !s_palvalid || memcmp(pw, s_palprev + bl * 16, 32) != 0;
		if (!dirty[bl])
			continue;
		memcpy(s_palprev + bl * 16, pw, 32);
		for (int i = bl * 8; i < bl * 8 + 8; i++) {
			uint32_t c = (uint32_t)v->palram[i * 2] << 16 | v->palram[i * 2 + 1];
			uint32_t rgb;
			if (v->pal12)
				rgb = (((c >> 12) & 15) * 16) << 16 | (((c >> 8) & 15) * 16) << 8 | ((c >> 4) & 15) * 16;
			else
				rgb = c & 0xffffff;
			v->pal_rgb[i] = rgb;
			palette[i] = rgb;
		}
	}
	s_palvalid = 1;
	// the frame to write (with a new color format: every color converted again)
	s_out = (uint8_t *)rbx_video.out;
#ifndef __vita__
	{	static int once;
		if (!once) {
			once = 1;
			s_outchk = getenv("RBX_OUTCHK") != NULL;
			if (s_outchk)
				atexit(outchk_report);
		}
	}
#endif
	if (s_out) {
		const int again = s_out_color != rbx_video.out_color || s_out_bpp != rbx_video.out_bpp;
		s_out_color = rbx_video.out_color;
		s_out_bpp = rbx_video.out_bpp;
		s_out_pitch = rbx_video.out_pitch;
		s_out565 = s_out_bpp == 2 && s_out_color(0xff, 0, 0) == 0xf800 && s_out_color(0, 0xff, 0) == 0x07e0 &&
			s_out_color(0, 0, 0xff) == 0x001f && s_out_color(0x84, 0x82, 0x21) == rgb565(0x848221);
		for (int i = 0; i < F3V_PAL_COLORS; i++) {
			if (s_lutvalid && !again && !dirty[i >> 3]) {
				i |= 7;
				continue;
			}
			const uint32_t rgb = v->pal_rgb[i];
			s_outlut[i] = s_out_color((int)(rgb >> 16) & 0xff, (int)(rgb >> 8) & 0xff, (int)rgb & 0xff);
			s_outlut16[i] = (uint16_t)s_outlut[i];
		}
		s_lutvalid = 1;
	} else
		s_lutvalid = 0;
	// tile usage per row (MAME m_tilemap_row_usage / m_textram_row_usage)
	memset(fc.pf_row_usage, 0, sizeof(fc.pf_row_usage));
	for (int off = 1; off < 0x4000; off += 2)
		if (v->vram[off]) {
			int row = v->extend ? (off >> 7) & 31 : (off >> 6) & 31;
			int tm = v->extend ? off >> 12 : off >> 11;
			if (fc.pf_row_usage[row][tm] < 255)
				fc.pf_row_usage[row][tm]++;
		}
	// pixel layer cell rows with something set (pivot RAM: cell col * 32 + row, 16 words each)
	memset(fc.pix_row_usage, 0, sizeof(fc.pix_row_usage));
	for (int cell = 0; cell < 64 * 32; cell++) {
		const uint16_t *c = v->pivotram + cell * 16;
		if (!fc.pix_row_usage[cell & 31]) {
			uint16_t any = 0;
			for (int i = 0; i < 16; i++)
				any |= c[i];
			if (any)
				fc.pix_row_usage[cell & 31] = 1;
		}
	}
	memset(fc.text_row_usage, 0, sizeof(fc.text_row_usage));
	for (int off = 0; off < 0x1000; off++)
		if (v->vram[0x6000 + off] & 0xff)
			fc.text_row_usage[(off >> 6) & 63] = 1;

	// (MAME's order: lag 0 the sprites, then the lines; 1 the lines, then the sprite list and sprites; 2 the lines,
	// the sprites, the list: the later sprites go to the other buffer while the lines are drawn)
	if (v->sprite_lag == 0) {
		get_sprite_info(v);
		v->flipscreen = v->sflip;
		v->fb_draw = v->fb;
		v->sru_draw = v->sprite_row_usage;
		scanline_draw(v, &fc, bitmap, pitch, palette, 3);
	} else if (v->sprite_lag == 1)
		scanline_draw(v, &fc, bitmap, pitch, palette, 1);
	else {
		scanline_draw(v, &fc, bitmap, pitch, palette, 2);
		get_sprite_info(v);
		v->flipscreen = v->sflip;
	}
	if (s_out)
		rbx_video.out_done = 1;
	// palette entries used: the render thread's range, or up to the helper's when it drew lines of this frame
	for (int i = 2; i > 0; i--)   // (up to the last range a thread put blended colours in this frame)
		if (s_w[i].job == s_job_id && s_w[i].bs.next > W_RANGE[i])
			return (int)s_w[i].bs.next;
	return (int)s_w[0].bs.next;
}
