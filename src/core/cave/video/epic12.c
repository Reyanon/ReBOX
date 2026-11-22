/*  Cave CV1000 blitter, ported to C from MAME 0.289 src/mame/cave/cv1k_v.cpp, cv1k_v.h, cv1k_v_blit0-8.cpp,
    cv1k_v_in.ipp, cv1k_v_pixel.ipp (BSD-3-Clause, copyright-holders: David Haywood, Luca Elia, MetalliC).
    The blit variants of MAME's include templates are one always-inline function specialised by its constant
    arguments (flip x, tint, transparency, blending, source / destination mode). MAME's quirks are kept
    (add_with_clr_square takes the red of the first colour for all three channels, sources crossing the VRAM's
    right edge are not drawn).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "epic12.h"
#include <stdlib.h>
#include <string.h>

#define VRAM_CLK_NS 13
#define SRAM_CLK_NS 20
#define H_LINE_PERIOD_NS 63600
#define H_LINE_DURATION_NS 2160
#define DRAW_OP_BYTES 20
#define CLIP_OP_BYTES 2
#define CHUNK_BYTES 64
#define CHUNK_INTERVAL_NS 700
#define CLIP_MARGIN 32

#if defined(__GNUC__)
#define E12_INLINE static inline __attribute__((always_inline))
#else
#define E12_INLINE static inline
#endif

typedef struct { int min_x, max_x, min_y, max_y; } rect_t;
typedef struct { uint8_t r, g, b; } clr_t;

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#define E12_NEON 1
#else
#define E12_NEON 0
#endif

static uint8_t colrtable[0x20][0x40];
static uint8_t colrtable_rev[0x20][0x40];
static uint8_t colrtable_add[0x20][0x20];
static uint8_t colrtable_t[0x40][0x20];   // (colrtable transposed: the tint's column as a row, for the NEON lookups)
static int s_tables;
static int s_scalar;   // (RBX_EPIC12_SCALAR=1, testing: no NEON rows)
static int s_pf = 1;    // (RBX_EPIC12_PF: how the NEON rows prefetch, ADR-145)
static int s_list_old;  // (RBX_CV1K_OLDLIST=1: the list read as before ADR-170)

void epic12_init_tables(void)
{
	if (s_tables)
		return;
	for (int y = 0; y < 0x40; y++)
		for (int x = 0; x < 0x20; x++) {
			int v = (x * y) / 0x1f;
			if (v > 0x1f) v = 0x1f;
			colrtable[x][y] = (uint8_t)v;
			colrtable_rev[x ^ 0x1f][y] = (uint8_t)v;
			colrtable_t[y][x] = (uint8_t)v;
		}
	for (int y = 0; y < 0x20; y++)
		for (int x = 0; x < 0x20; x++)
			colrtable_add[x][y] = (uint8_t)((x + y) > 0x1f ? 0x1f : (x + y));
	s_tables = 1;
}

#define ADD  colrtable_add
#define MUL  colrtable
#define REV  colrtable_rev

#if E12_NEON
// MUL[a][b] for a, b < 32: a * b / 31 (rounded down)
static inline uint8x8_t e12_mul31(uint8x8_t a, uint8x8_t b)
{
	const uint16x8_t p = vmull_u8(a, b);
	return vmovn_u16(vshrq_n_u16(vaddq_u16(vaddq_u16(p, vdupq_n_u16(1)), vshrq_n_u16(p, 5)), 5));
}
// a 32-entry table row in four d registers; t looked up by each entry of row (t[row[i]]); the values << 3
static inline uint8x8x4_t e12_tab(const uint8_t *row)
{
	uint8x8x4_t t;
	t.val[0] = vld1_u8(row); t.val[1] = vld1_u8(row + 8); t.val[2] = vld1_u8(row + 16); t.val[3] = vld1_u8(row + 24);
	return t;
}
static inline uint8x8x4_t e12_tab_of(uint8x8x4_t t, const uint8_t *row)
{
	const uint8x8x4_t i = e12_tab(row);
	uint8x8x4_t o;
	for (int k = 0; k < 4; k++)
		o.val[k] = vtbl4_u8(t, i.val[k]);
	return o;
}
static inline uint8x8x4_t e12_tab_shl3(uint8x8x4_t t)
{
	for (int k = 0; k < 4; k++)
		t.val[k] = vshl_n_u8(t.val[k], 3);
	return t;
}
#endif
// [a0, a1] (a0 <= a1 < mod) meets the len values from s on, round the circle of mod (a power of two)
static inline int e12_circ_hit(int a0, int a1, uint32_t s, uint32_t len, uint32_t mod)
{
	const uint32_t d = ((uint32_t)a0 - s) & (mod - 1);
	return d < len || (uint32_t)(a1 - a0) >= mod - d;
}

#ifdef RBX_E12_PROF
// (profiling build only: per blit kind, the calls, rows, pixels and how they go)
#include <stdio.h>
typedef struct { uint64_t calls, rows, px, vpx, tail, ovpx, trpx, t8, o8, w[8]; } e12prof_t;
static e12prof_t s_prof[2048];
static void e12_prof_dump(void)
{
	uint64_t tot = 0;
	for (int k = 0; k < 2048; k++) tot += s_prof[k].px;
	fprintf(stderr, "e12prof: total px %llu\n", (unsigned long long)tot);
	for (;;) {
		int best = -1;
		for (int k = 0; k < 2048; k++)
			if (s_prof[k].px && (best < 0 || s_prof[k].px > s_prof[best].px)) best = k;
		if (best < 0) break;
		e12prof_t *p = &s_prof[best];
		const int k = best;
		fprintf(stderr, "e12prof %s fx%d ti%d tr%d s%d d%d: calls %llu rows %llu px %llu (%.1f%%) vec %llu tail %llu ovl %llu "
		        "transp %llu t8 %llu o8 %llu | w<8 %llu <16 %llu <32 %llu <64 %llu <128 %llu <256 %llu >=256 %llu\n",
		        (k >> 10) & 1 ? "si" : (k >> 3) & 1 ? "bl" : "pl", k & 1, (k >> 1) & 1, (k >> 2) & 1, (k >> 4) & 7, (k >> 7) & 7,
		        (unsigned long long)p->calls, (unsigned long long)p->rows, (unsigned long long)p->px, 100.0 * p->px / (tot ? tot : 1),
		        (unsigned long long)p->vpx, (unsigned long long)p->tail, (unsigned long long)p->ovpx, (unsigned long long)p->trpx,
		        (unsigned long long)p->t8, (unsigned long long)p->o8, (unsigned long long)p->w[0], (unsigned long long)p->w[1],
		        (unsigned long long)p->w[2], (unsigned long long)p->w[3], (unsigned long long)p->w[4], (unsigned long long)p->w[5],
		        (unsigned long long)p->w[6]);
		p->px = 0;
	}
}
// a memory model of the blit rows' accesses (ADR-146), for VRAM layouts: 0 rows 8192 + 16 apart (now), 1 rows 8192
// apart (before), 2..4 bands of 256 / 512 / 1024 columns (each band's rows one after another). Per layout: a 2-way
// 128-entry TLB of 4 KB pages, a 4-way 32 KB L1 and an 8-way 512 KB L2 of 32-byte lines, LRU; virtual = physical
#define E12M_N 5
typedef struct { uint32_t tag[2048][8]; } e12m_cache_t;
static e12m_cache_t s_m_l1[E12M_N], s_m_l2[E12M_N], s_m_tlb[E12M_N];
static uint64_t s_m_acc[E12M_N], s_m_l1m[E12M_N], s_m_l2m[E12M_N], s_m_tlbm[E12M_N];
static int e12m_look(e12m_cache_t *c, uint32_t key, int sets, int ways)
{
	uint32_t *w = c->tag[key & (sets - 1)];
	const uint32_t k = key + 1;
	for (int i = 0; i < ways; i++)
		if (w[i] == k) {
			for (; i > 0; i--) w[i] = w[i - 1];
			w[0] = k;
			return 1;
		}
	for (int i = ways - 1; i > 0; i--) w[i] = w[i - 1];
	w[0] = k;
	return 0;
}
static uint32_t e12m_addr(int v, int x, int y)
{
	if (v == 0) return (uint32_t)(y * (EPIC12_W + 16) + x) * 2;
	if (v == 1) return (uint32_t)(y * EPIC12_W + x) * 2;
	const int bs = v == 2 ? 8 : v == 3 ? 9 : 10, bw = 1 << bs;
	return (uint32_t)((x >> bs) * bw * EPIC12_H + y * bw + (x & (bw - 1))) * 2;
}
static void e12m_run(int x0, int y, int n)
{
	for (int v = 0; v < E12M_N; v++) {
		uint32_t last = 0xffffffffu, lastp = 0xffffffffu;
		for (int i = 0; i < n; i++) {
			const uint32_t a = e12m_addr(v, x0 + i, y), line = a >> 5, page = a >> 12;
			if (page != lastp) {
				lastp = page;
				s_m_tlbm[v] += !e12m_look(&s_m_tlb[v], page, 64, 2);
			}
			if (line != last) {
				last = line;
				s_m_acc[v]++;
				if (!e12m_look(&s_m_l1[v], line, 256, 4)) {
					s_m_l1m[v]++;
					s_m_l2m[v] += !e12m_look(&s_m_l2[v], line, 2048, 8);
				}
			}
		}
	}
}
static void e12m_dump(void)
{
	static const char *const nm[E12M_N] = { "rows 8208", "rows 8192", "band 256", "band 512", "band 1024" };
	for (int v = 0; v < E12M_N; v++)
		fprintf(stderr, "e12mem %-10s: lines %llu, L1 miss %llu, L2 miss %llu, TLB miss %llu\n", nm[v],
		        (unsigned long long)s_m_acc[v], (unsigned long long)s_m_l1m[v], (unsigned long long)s_m_l2m[v],
		        (unsigned long long)s_m_tlbm[v]);
}
#endif

#define BLIT_ARGS uint16_t *vram, const rect_t *clip, int src_x, int src_y, const int dst_x_start, const int dst_y_start, \
	int dimx, int dimy, const int flipy, const uint8_t s_alpha, const uint8_t d_alpha, const clr_t tint
#define BLIT_PASS vram, clip, src_x, src_y, dst_x_start, dst_y_start, dimx, dimy, flipy, s_alpha, d_alpha, tint

#if defined(__GNUC__) && E12_NEON
// (the NEON tables are set wherever they are used: VEC, n >= 8 and not s_scalar, a condition GCC does not follow)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
// cv1k_v_pixel.ipp: one pixel (pen: the source pixel)
E12_INLINE void e12_pixel(uint16_t *bmp, const uint32_t pen, const uint8_t s_alpha, const uint8_t d_alpha, const clr_t tint,
                          const uint8_t *salpha_table, const uint8_t *dalpha_table, const int TINT, const int TRANS,
                          const int BLENDED, const int SMODE, const int DMODE, const int SIMPLE)
{
	if (TRANS && !(pen & 0x8000))
		return;
	if (SIMPLE) {
		*bmp = (uint16_t)pen;
		return;
	}
	clr_t s = { (uint8_t)((pen >> 10) & 0x1f), (uint8_t)((pen >> 5) & 0x1f), (uint8_t)(pen & 0x1f) };
	if (TINT) {
		s.r = MUL[s.r][tint.r];
		s.g = MUL[s.g][tint.g];
		s.b = MUL[s.b][tint.b];
	}
	if (BLENDED) {
		const uint32_t dp = *bmp;
		const clr_t d = { (uint8_t)((dp >> 10) & 0x1f), (uint8_t)((dp >> 5) & 0x1f), (uint8_t)(dp & 0x1f) };
		clr_t c0 = { 0, 0, 0 };
		if (SMODE == 0) {
			if (DMODE == 0) {
				s.r = ADD[salpha_table[s.r]][dalpha_table[d.r]];
				s.g = ADD[salpha_table[s.g]][dalpha_table[d.g]];
				s.b = ADD[salpha_table[s.b]][dalpha_table[d.b]];
			} else if (DMODE == 1) {
				s.r = ADD[salpha_table[s.r]][MUL[s.r][d.r]];
				s.g = ADD[salpha_table[s.g]][MUL[s.g][d.g]];
				s.b = ADD[salpha_table[s.b]][MUL[s.b][d.b]];
			} else if (DMODE == 5) {
				s.r = ADD[salpha_table[s.r]][REV[s.r][d.r]];
				s.g = ADD[salpha_table[s.g]][REV[s.g][d.g]];
				s.b = ADD[salpha_table[s.b]][REV[s.b][d.b]];
			} else {
				c0.r = MUL[s_alpha][s.r]; c0.g = MUL[s_alpha][s.g]; c0.b = MUL[s_alpha][s.b];   // mul_fixed
			}
		} else if (SMODE == 2) {
			if (DMODE == 0) {
				s.r = ADD[MUL[d.r][s.r]][dalpha_table[d.r]];
				s.g = ADD[MUL[d.g][s.g]][dalpha_table[d.g]];
				s.b = ADD[MUL[d.b][s.b]][dalpha_table[d.b]];
			} else {
				c0.r = MUL[d.r][s.r]; c0.g = MUL[d.g][s.g]; c0.b = MUL[d.b][s.b];   // mul_3param(s, d)
			}
		} else if (SMODE == 1) {
			c0.r = MUL[s.r][s.r]; c0.g = MUL[s.g][s.g]; c0.b = MUL[s.b][s.b];       // square
		} else if (SMODE == 3 || SMODE == 7) {
			c0 = s;                                                                  // copy
		} else if (SMODE == 4) {
			c0.r = REV[s_alpha][s.r]; c0.g = REV[s_alpha][s.g]; c0.b = REV[s_alpha][s.b];   // mul_fixed_rev
		} else if (SMODE == 5) {
			c0.r = REV[s.r][s.r]; c0.g = REV[s.g][s.g]; c0.b = REV[s.b][s.b];       // mul_rev_square
		} else {   // SMODE 6
			c0.r = REV[d.r][s.r]; c0.g = REV[d.g][s.g]; c0.b = REV[d.b][s.b];       // mul_rev_3param(s, d)
		}
		// the destination term (SMODE 0 with DMODE 0 / 1 / 5 and SMODE 2 with DMODE 0 are done above)
		const int done = (SMODE == 0 && (DMODE == 0 || DMODE == 1 || DMODE == 5)) || (SMODE == 2 && DMODE == 0);
		if (!done) {
			if (DMODE == 0 && SMODE != 0) {          // add_with_clr_mul_fixed(c0, d_alpha, d)
				s.r = ADD[c0.r][MUL[d.r][d_alpha]];
				s.g = ADD[c0.g][MUL[d.g][d_alpha]];
				s.b = ADD[c0.b][MUL[d.b][d_alpha]];
			} else if (DMODE == 1) {                 // add_with_clr_mul_3param(c0, d, s)
				s.r = ADD[c0.r][MUL[s.r][d.r]];
				s.g = ADD[c0.g][MUL[s.g][d.g]];
				s.b = ADD[c0.b][MUL[s.b][d.b]];
			} else if (DMODE == 2) {                 // add_with_clr_square(c0, d) (MAME: c0.r for all)
				s.r = ADD[c0.r][MUL[d.r][d.r]];
				s.g = ADD[c0.r][MUL[d.g][d.g]];
				s.b = ADD[c0.r][MUL[d.b][d.b]];
			} else if (DMODE == 3 || DMODE == 7) {   // add(c0, d)
				s.r = ADD[c0.r][d.r];
				s.g = ADD[c0.g][d.g];
				s.b = ADD[c0.b][d.b];
			} else if (DMODE == 4) {                 // add_with_clr_mul_fixed_rev(c0, d_alpha, d)
				s.r = ADD[c0.r][REV[d_alpha][d.r]];
				s.g = ADD[c0.g][REV[d_alpha][d.g]];
				s.b = ADD[c0.b][REV[d_alpha][d.b]];
			} else if (DMODE == 5) {                 // add_with_clr_mul_rev_3param(c0, d, s)
				s.r = ADD[c0.r][REV[s.r][d.r]];
				s.g = ADD[c0.g][REV[s.g][d.g]];
				s.b = ADD[c0.b][REV[s.b][d.b]];
			} else if (DMODE == 6) {                 // add_with_clr_mul_rev_square(c0, d)
				s.r = ADD[c0.r][REV[d.r][d.r]];
				s.g = ADD[c0.g][REV[d.g][d.g]];
				s.b = ADD[c0.b][REV[d.b][d.b]];
			}
		}
	}
	*bmp = (uint16_t)((s.r << 10) | (s.g << 5) | s.b | (pen & 0x8000));
}

#if E12_NEON
// the tables of the NEON rows (32 entries each, in four d registers)
typedef struct { uint8x8x4_t s0, s1, s2, d, t0, t1, t2; } e12tabs_t;

// 8 pixels from the source (gfx2: the first one's; FLIPX: going left), their low and high bytes apart
E12_INLINE uint8x8x2_t e12_ld8(const uint16_t *gfx2, const int FLIPX)
{
	uint8x8x2_t sp;
	if (FLIPX) {
		sp = vld2_u8((const uint8_t *)(gfx2 - 7));
		sp.val[0] = vrev64_u8(sp.val[0]); sp.val[1] = vrev64_u8(sp.val[1]);
	} else
		sp = vld2_u8((const uint8_t *)gfx2);
	return sp;
}
E12_INLINE uint16x8_t e12_ld8w(const uint16_t *gfx2, const int FLIPX)
{
	if (FLIPX) {
		const uint16x8_t p = vrev64q_u16(vld1q_u16(gfx2 - 7));
		return vcombine_u16(vget_high_u16(p), vget_low_u16(p));
	}
	return vld1q_u16(gfx2);
}

// 8 pixels drawn (ADR-105, 121, 145): sp the source pixels, dp the destination's (low and high bytes apart), the
// channels 8 to a d register; the same table values (vtbl over the 32 entries of a row; a product of two colours,
// MUL[a][b] = a * b / 31 for a, b < 32, as (p + 1 + (p >> 5)) >> 5, exact for p <= 961); s_mode 0 with d_mode 0 / 4:
// both terms from tables of the values << 3, min(31, a + b) = saturating add >> 3; the tint folded into the source term
// table where the tinted colour is not needed itself (TMIX), else its own tables (TTAB)
E12_INLINE uint8x8x2_t e12_px8(const uint8x8x2_t sp, const uint8x8x2_t dp, const e12tabs_t *t, const int TRANS,
                               const int BLENDED, const int SMODE, const int DMODE, const int Q8, const int TMIX,
                               const int TTAB)
{
	const uint8x8_t c31 = vdup_n_u8(0x1f), c80 = vdup_n_u8(0x80);
	const uint8x8_t lo = sp.val[0], hi = sp.val[1];
	uint8x8_t r = vand_u8(vshr_n_u8(hi, 2), c31);
	uint8x8_t g = vand_u8(vsli_n_u8(vshr_n_u8(lo, 5), hi, 3), c31);
	uint8x8_t bl = vand_u8(lo, c31);
	uint8x8x2_t out;
	if (TTAB) {
		r = vtbl4_u8(t->t0, r); g = vtbl4_u8(t->t1, g); bl = vtbl4_u8(t->t2, bl);
	}
	if (BLENDED) {
		const uint8x8_t dr = vand_u8(vshr_n_u8(dp.val[1], 2), c31);
		const uint8x8_t dg = vand_u8(vsli_n_u8(vshr_n_u8(dp.val[0], 5), dp.val[1], 3), c31);
		const uint8x8_t db = vand_u8(dp.val[0], c31);
		if (Q8) {   // (the values << 3: the sum saturates at 255 from 32 << 3 on)
			r = vqadd_u8(vtbl4_u8(t->s0, r), vtbl4_u8(t->d, dr));
			g = vqadd_u8(vtbl4_u8(TMIX ? t->s1 : t->s0, g), vtbl4_u8(t->d, dg));
			bl = vqadd_u8(vtbl4_u8(TMIX ? t->s2 : t->s0, bl), vtbl4_u8(t->d, db));
		} else {
			uint8x8_t xr, xg, xb, yr, yg, yb;   // (the destination term, the source term)
			if (SMODE == 2) {                // (MUL[d][s], dalpha_table[d])
				yr = e12_mul31(dr, r); yg = e12_mul31(dg, g); yb = e12_mul31(db, bl);
				xr = vtbl4_u8(t->d, dr); xg = vtbl4_u8(t->d, dg); xb = vtbl4_u8(t->d, db);
			} else {
				yr = vtbl4_u8(t->s0, r);
				yg = vtbl4_u8(TMIX ? t->s1 : t->s0, g);
				yb = vtbl4_u8(TMIX ? t->s2 : t->s0, bl);
				if (DMODE == 1) {            // (MUL[s][d])
					xr = e12_mul31(r, dr); xg = e12_mul31(g, dg); xb = e12_mul31(bl, db);
				} else if (DMODE == 5) {     // (REV[s][d] = MUL[s ^ 31][d])
					xr = e12_mul31(veor_u8(r, c31), dr); xg = e12_mul31(veor_u8(g, c31), dg);
					xb = e12_mul31(veor_u8(bl, c31), db);
				} else {                     // (3, 7: d)
					xr = dr; xg = dg; xb = db;
				}
			}
			r = vmin_u8(vadd_u8(yr, xr), c31);
			g = vmin_u8(vadd_u8(yg, xg), c31);
			bl = vmin_u8(vadd_u8(yb, xb), c31);
		}
	}
	if (Q8) {   // (from the values << 3, their low bits not used)
		out.val[0] = vsri_n_u8(vshl_n_u8(g, 2), bl, 3);
		out.val[1] = vbsl_u8(c80, hi, vsri_n_u8(vshr_n_u8(r, 1), g, 6));
	} else {
		out.val[0] = vsli_n_u8(bl, g, 5);
		out.val[1] = vbsl_u8(c80, hi, vsli_n_u8(vshr_n_u8(g, 3), r, 2));
	}
	if (TRANS) {
		const uint8x8_t m = vreinterpret_u8_s8(vshr_n_s8(vreinterpret_s8_u8(hi), 7));
		out.val[0] = vbsl_u8(m, out.val[0], dp.val[0]);
		out.val[1] = vbsl_u8(m, out.val[1], dp.val[1]);
	}
	return out;
}

// (the lanes from k on: e12_tailm + 8 - k)
static const uint8_t e12_tailm[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
static const uint16_t e12_tailm16[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
                                          0xffff };

// a row's pixels from bmp to end, 8 at a time (16 for plain copies); fewer than 8 left: with tail (a row of 8 or more
// whose source and destination do not meet), the last 8 of the row once more with only the pixels not yet drawn
// taken (the others kept as drawn: each pixel from its source and its destination as they were), else left (pbmp, pgfx
// where they are)
E12_INLINE void e12_vrow(uint16_t **pbmp, const uint16_t **pgfx, const uint16_t *end, const e12tabs_t *t, const int tail,
                         const int FLIPX, const int TRANS, const int BLENDED, const int SMODE, const int DMODE,
                         const int SIMPLE, const int Q8, const int TMIX, const int TTAB)
{
	uint16_t *bmp = *pbmp;
	const uint16_t *gfx2 = *pgfx;
	if (SIMPLE) {
		const uint16x8_t top = vdupq_n_u16(0x8000);
		for (; end - bmp >= 16; bmp += 16) {
			uint16x8_t p0 = e12_ld8w(gfx2, FLIPX), p1 = e12_ld8w(FLIPX ? gfx2 - 8 : gfx2 + 8, FLIPX);
			gfx2 += FLIPX ? -16 : 16;
			if (TRANS) {
				p0 = vbslq_u16(vtstq_u16(p0, top), p0, vld1q_u16(bmp));
				p1 = vbslq_u16(vtstq_u16(p1, top), p1, vld1q_u16(bmp + 8));
			}
			vst1q_u16(bmp, p0);
			vst1q_u16(bmp + 8, p1);
		}
		if (end - bmp >= 8) {
			uint16x8_t p0 = e12_ld8w(gfx2, FLIPX);
			gfx2 += FLIPX ? -8 : 8;
			if (TRANS)
				p0 = vbslq_u16(vtstq_u16(p0, top), p0, vld1q_u16(bmp));
			vst1q_u16(bmp, p0);
			bmp += 8;
		}
		if (tail && bmp < end) {
			const int k = 8 - (int)(end - bmp);   // (the pixels of the last 8 already drawn)
			bmp -= k;
			gfx2 += FLIPX ? k : -k;
			const uint16x8_t dp = vld1q_u16(bmp);
			uint16x8_t p0 = e12_ld8w(gfx2, FLIPX);
			if (TRANS)
				p0 = vbslq_u16(vtstq_u16(p0, top), p0, dp);
			vst1q_u16(bmp, vbslq_u16(vld1q_u16(e12_tailm16 + 8 - k), p0, dp));
			bmp += 8;
			gfx2 += FLIPX ? -8 : 8;
		}
	} else {
		for (; end - bmp >= 8; bmp += 8) {
			const uint8x8x2_t sp = e12_ld8(gfx2, FLIPX);
			gfx2 += FLIPX ? -8 : 8;
			uint8x8x2_t dp;
			if (BLENDED || TRANS)
				dp = vld2_u8((const uint8_t *)bmp);
			vst2_u8((uint8_t *)bmp, e12_px8(sp, dp, t, TRANS, BLENDED, SMODE, DMODE, Q8, TMIX, TTAB));
		}
		if (tail && bmp < end) {
			const int k = 8 - (int)(end - bmp);
			bmp -= k;
			gfx2 += FLIPX ? k : -k;
			const uint8x8x2_t sp = e12_ld8(gfx2, FLIPX), dp = vld2_u8((const uint8_t *)bmp);
			uint8x8x2_t out = e12_px8(sp, dp, t, TRANS, BLENDED, SMODE, DMODE, Q8, TMIX, TTAB);
			const uint8x8_t m = vld1_u8(e12_tailm + 8 - k);
			out.val[0] = vbsl_u8(m, out.val[0], dp.val[0]);
			out.val[1] = vbsl_u8(m, out.val[1], dp.val[1]);
			vst2_u8((uint8_t *)bmp, out);
			bmp += 8;
			gfx2 += FLIPX ? -8 : 8;
		}
	}
	*pbmp = bmp;
	*pgfx = gfx2;
}
#endif

// cv1k_v_in.ipp
E12_INLINE void blit(BLIT_ARGS, const int FLIPX, const int TINT, const int TRANS, const int BLENDED, const int SMODE,
                     const int DMODE, const int SIMPLE)
{
	int yf;
	if (FLIPX)
		src_x += dimx - 1;
	if (flipy) { yf = -1; src_y += dimy - 1; }
	else yf = +1;

	int starty = 0;
	const int dst_y_end = dst_y_start + dimy;
	if (dst_y_start < clip->min_y)
		starty = clip->min_y - dst_y_start;
	if (dst_y_end > clip->max_y)
		dimy -= (dst_y_end - 1) - clip->max_y;

	// (a source wrapping round the right edge of the VRAM is not drawn)
	if (FLIPX) {
		if ((src_x & 0x1fff) < ((src_x - (dimx - 1)) & 0x1fff))
			return;
	} else {
		if ((src_x & 0x1fff) > ((src_x + (dimx - 1)) & 0x1fff))
			return;
	}

	int startx = 0;
	const int dst_x_end = dst_x_start + dimx;
	if (dst_x_start < clip->min_x)
		startx = clip->min_x - dst_x_start;
	if (dst_x_end > clip->max_x)
		dimx -= (dst_x_end - 1) - clip->max_x;

	const uint8_t *salpha_table = colrtable[s_alpha];
	const uint8_t *dalpha_table = colrtable[d_alpha];
#ifdef RBX_E12_PROF
	const int PKEY = FLIPX | TINT << 1 | TRANS << 2 | BLENDED << 3 | SMODE << 4 | DMODE << 7 | SIMPLE << 10;
	const int PVEC = SIMPLE || (!BLENDED && TINT) ||
	                 (BLENDED && SMODE == 0 && (DMODE == 0 || DMODE == 1 || DMODE == 3 || DMODE == 4 || DMODE == 5 || DMODE == 7)) ||
	                 (BLENDED && SMODE == 2 && DMODE == 0);
	if (starty < dimy && startx < dimx) {
		const int ww = dimx - startx;
		s_prof[PKEY].calls++;
		s_prof[PKEY].w[ww < 8 ? 0 : ww < 16 ? 1 : ww < 32 ? 2 : ww < 64 ? 3 : ww < 128 ? 4 : ww < 256 ? 5 : 6]++;
	}
#endif
	if (starty >= dimy || startx >= dimx)
		return;
	const int n = dimx - startx;   // (pixels a row)

#if E12_NEON
	// the NEON rows for the common kinds (plain copies, tinted plain, s_mode 0 with d_mode 0, 1, 3, 4, 5, 7, s_mode 2
	// with d_mode 0), the same per-pixel order (a row whose source and destination meet is done one pixel at a time)
	const int VEC = SIMPLE || (!BLENDED && TINT) ||
	                (BLENDED && SMODE == 0 && (DMODE == 0 || DMODE == 1 || DMODE == 3 || DMODE == 4 || DMODE == 5 || DMODE == 7)) ||
	                (BLENDED && SMODE == 2 && DMODE == 0);
	const int Q8 = BLENDED && SMODE == 0 && (DMODE == 0 || DMODE == 4);
	const int TMIX = TINT && BLENDED && SMODE == 0 && DMODE != 1 && DMODE != 5;
	const int TTAB = TINT && !TMIX;   // (the tinted colour itself: plain, s_mode 0 with d_mode 1 / 5, s_mode 2)
	e12tabs_t tb;
	int meet = 1;   // (a row may have its source and destination meet)
	if (VEC && n >= 8 && !s_scalar) {
		if (TTAB) {
			tb.t0 = e12_tab(colrtable_t[tint.r]); tb.t1 = e12_tab(colrtable_t[tint.g]); tb.t2 = e12_tab(colrtable_t[tint.b]);
		}
		if (BLENDED && SMODE == 0) {   // (salpha_table[s] = MUL[s_alpha][s], the mul_fixed of d_mode 3, 4, 7 too)
			tb.s0 = e12_tab(salpha_table);
			if (TMIX) {
				tb.s1 = e12_tab_of(tb.s0, colrtable_t[tint.g]);
				tb.s2 = e12_tab_of(tb.s0, colrtable_t[tint.b]);
				tb.s0 = e12_tab_of(tb.s0, colrtable_t[tint.r]);
			}
			if (Q8) {
				tb.s0 = e12_tab_shl3(tb.s0);
				if (TMIX) { tb.s1 = e12_tab_shl3(tb.s1); tb.s2 = e12_tab_shl3(tb.s2); }
			}
		}
		if (BLENDED && DMODE == 0)
			tb.d = e12_tab(dalpha_table);
		if (BLENDED && DMODE == 4)
			tb.d = e12_tab(REV[d_alpha]);
		if (Q8)
			tb.d = e12_tab_shl3(tb.d);
		// (the columns or the rows apart: no row meets)
		const int dx0 = dst_x_start + startx, sx0 = FLIPX ? src_x - startx - (n - 1) : src_x + startx;
		if (sx0 + n - 1 < dx0 || sx0 > dx0 + n - 1)
			meet = 0;
		else {
			const int cnt = dimy - starty, first = (src_y + yf * starty) & 0x0fff;
			meet = e12_circ_hit(dst_y_start + starty, dst_y_start + dimy - 1,
			                    (uint32_t)(yf > 0 ? first : (first - (cnt - 1)) & 0x0fff), (uint32_t)cnt, EPIC12_H);
		}
		if (!meet) {   // (every row in NEON, its last pixels too: n >= 8)
			uint16_t *drow = vram + (size_t)(dst_y_start + starty) * EPIC12_STRIDE + (dst_x_start + startx);
			const int sxo = FLIPX ? src_x - startx : src_x + startx;
			const int cnt = dimy - starty, first = (src_y + yf * starty) & 0x0fff, lastr = first + yf * (cnt - 1);
			const ptrdiff_t sstep = (ptrdiff_t)yf * EPIC12_STRIDE;
			// (the rows ahead on their way, RBX_EPIC12_PF: 0 the next row's first pixels, 1 the cache lines of its
			// first, middle and last pixels, n >= 2 those of the row n rows on, at most 8)
			const int pfa = s_pf >= 2 ? (s_pf > 8 ? 8 : s_pf) : 1;
			const ptrdiff_t pso = pfa * sstep + (s_pf >= 1 && FLIPX ? -(n - 1) : 0), pdo = pfa * EPIC12_STRIDE;
			const ptrdiff_t pfm = s_pf >= 1 ? n / 2 : 0, pfl = s_pf >= 1 ? n - 1 : 0;
			if (lastr >= 0 && lastr < EPIC12_H) {   // (the source rows not round the bottom of the VRAM)
				const uint16_t *srow = vram + (size_t)first * EPIC12_STRIDE + sxo;
				for (int y = cnt; y > 0; y--, drow += EPIC12_STRIDE, srow += sstep) {
					const uint16_t *ps = srow + pso;
					uint16_t *pd = drow + pdo;
					__builtin_prefetch(ps); __builtin_prefetch(ps + pfm); __builtin_prefetch(ps + pfl);
					__builtin_prefetch(pd, 1); __builtin_prefetch(pd + pfm, 1); __builtin_prefetch(pd + pfl, 1);
					uint16_t *bmp = drow;
					const uint16_t *gfx2 = srow;
					e12_vrow(&bmp, &gfx2, drow + n, &tb, 1, FLIPX, TRANS, BLENDED, SMODE, DMODE, SIMPLE, Q8, TMIX, TTAB);
				}
			} else {
				uint32_t sy = (uint32_t)first;
				for (int y = cnt; y > 0; y--, drow += EPIC12_STRIDE) {
					const uint16_t *gfx2 = vram + (size_t)sy * EPIC12_STRIDE + sxo;
					const uint16_t *ps = vram + (size_t)((sy + (uint32_t)(pfa * yf)) & 0x0fff) * EPIC12_STRIDE + sxo + (pso - pfa * sstep);
					uint16_t *pd = drow + pdo;
					sy = (sy + (uint32_t)yf) & 0x0fff;
					__builtin_prefetch(ps); __builtin_prefetch(ps + pfm); __builtin_prefetch(ps + pfl);
					__builtin_prefetch(pd, 1); __builtin_prefetch(pd + pfm, 1); __builtin_prefetch(pd + pfl, 1);
					uint16_t *bmp = drow;
					e12_vrow(&bmp, &gfx2, drow + n, &tb, 1, FLIPX, TRANS, BLENDED, SMODE, DMODE, SIMPLE, Q8, TMIX, TTAB);
				}
			}
			return;
		}
	}
#endif

	for (int y = starty; y < dimy; y++) {
		uint16_t *bmp = vram + (size_t)(dst_y_start + y) * EPIC12_STRIDE + (dst_x_start + startx);
		const uint16_t *gfx2 = vram + (size_t)((src_y + yf * y) & 0x0fff) * EPIC12_STRIDE;
		gfx2 += FLIPX ? (src_x - startx) : (src_x + startx);
		const uint16_t *end = bmp + n;
#ifdef RBX_E12_PROF
		{
			e12prof_t *P = &s_prof[PKEY];
			P->rows++; P->px += (uint64_t)n;
			const uint16_t *s0 = FLIPX ? gfx2 - (n - 1) : gfx2, *s1 = s0 + n;
			const int vok = PVEC && n >= 8 && (s1 <= bmp || s0 >= end || (!FLIPX && s0 >= bmp));
			if (vok) { P->vpx += (uint64_t)(n & ~7); P->tail += (uint64_t)(n & 7); }
			else if (PVEC && n >= 8) P->ovpx += (uint64_t)n;
			else P->tail += (uint64_t)n;
			for (int i = 0; i < n; i += 8) {
				int tr = 0, m = n - i < 8 ? n - i : 8;
				for (int j = 0; j < m; j++) tr += !(s0[i + j] & 0x8000);
				P->trpx += (uint64_t)tr;
				if (m == 8) { P->t8 += tr == 8; P->o8 += tr == 0; }
			}
			const int sxl = FLIPX ? src_x - startx - (n - 1) : src_x + startx;
			e12m_run(sxl, (src_y + yf * y) & 0x0fff, n);
			e12m_run(dst_x_start + startx, dst_y_start + y, n);
		}
#endif
#if defined(__GNUC__)
		if (y + 1 < dimy) {   // (the next row's first pixels on their way: rows are 16 KB apart, a cache line each)
			const uint16_t *ns = vram + (size_t)((src_y + yf * (y + 1)) & 0x0fff) * EPIC12_STRIDE;
			__builtin_prefetch(ns + (FLIPX ? (src_x - startx) : (src_x + startx)));
			__builtin_prefetch(bmp + EPIC12_STRIDE, 1);
		}
#endif
#if E12_NEON
		if (VEC && !s_scalar && n >= 8) {
			const uint16_t *s0 = FLIPX ? gfx2 - (n - 1) : gfx2, *s1 = s0 + n;
			// (a forward source at or after the destination reads each pixel before it is written over, as one by one)
			if (s1 <= bmp || s0 >= end || (!FLIPX && s0 >= bmp))
				e12_vrow(&bmp, &gfx2, end, &tb, 0, FLIPX, TRANS, BLENDED, SMODE, DMODE, SIMPLE, Q8, TMIX, TTAB);
		}
#endif
		for (; bmp < end; bmp++) {
			e12_pixel(bmp, *gfx2, s_alpha, d_alpha, tint, salpha_table, dalpha_table, TINT, TRANS, BLENDED, SMODE, DMODE,
			          SIMPLE);
			if (FLIPX) gfx2--; else gfx2++;
		}
	}
}

typedef void (*blitfn)(BLIT_ARGS);

#define DEFB(fx, ti, tr, s, d) static void bl_##fx##ti##tr##_##s##d(BLIT_ARGS) { blit(BLIT_PASS, fx, ti, tr, 1, s, d, 0); }
#define DEFB8(fx, ti, tr, d) DEFB(fx, ti, tr, 0, d) DEFB(fx, ti, tr, 1, d) DEFB(fx, ti, tr, 2, d) DEFB(fx, ti, tr, 3, d) \
	DEFB(fx, ti, tr, 4, d) DEFB(fx, ti, tr, 5, d) DEFB(fx, ti, tr, 6, d) DEFB(fx, ti, tr, 7, d)
#define DEFB64(fx, ti, tr) DEFB8(fx, ti, tr, 0) DEFB8(fx, ti, tr, 1) DEFB8(fx, ti, tr, 2) DEFB8(fx, ti, tr, 3) \
	DEFB8(fx, ti, tr, 4) DEFB8(fx, ti, tr, 5) DEFB8(fx, ti, tr, 6) DEFB8(fx, ti, tr, 7)
#define TABB(fx, ti, tr, s, d) bl_##fx##ti##tr##_##s##d,
#define TABB8(fx, ti, tr, d) TABB(fx, ti, tr, 0, d) TABB(fx, ti, tr, 1, d) TABB(fx, ti, tr, 2, d) TABB(fx, ti, tr, 3, d) \
	TABB(fx, ti, tr, 4, d) TABB(fx, ti, tr, 5, d) TABB(fx, ti, tr, 6, d) TABB(fx, ti, tr, 7, d)
#define TABB64(fx, ti, tr) { TABB8(fx, ti, tr, 0) TABB8(fx, ti, tr, 1) TABB8(fx, ti, tr, 2) TABB8(fx, ti, tr, 3) \
	TABB8(fx, ti, tr, 4) TABB8(fx, ti, tr, 5) TABB8(fx, ti, tr, 6) TABB8(fx, ti, tr, 7) }

DEFB64(0, 0, 0) DEFB64(0, 0, 1) DEFB64(0, 1, 0) DEFB64(0, 1, 1)
DEFB64(1, 0, 0) DEFB64(1, 0, 1) DEFB64(1, 1, 0) DEFB64(1, 1, 1)

// [flipx][tint][transparent][s_mode | d_mode << 3]
static const blitfn s_blend[2][2][2][64] = {
	{ { TABB64(0, 0, 0), TABB64(0, 0, 1) }, { TABB64(0, 1, 0), TABB64(0, 1, 1) } },
	{ { TABB64(1, 0, 0), TABB64(1, 0, 1) }, { TABB64(1, 1, 0), TABB64(1, 1, 1) } },
};

// the tinted, unblended ones (MAME draw_sprite_fX_ti1_trX_plain) and the plain copies (draw_sprite_fX_ti0_trX_simple)
#define DEFP(fx, tr) static void pl_##fx##tr(BLIT_ARGS) { blit(BLIT_PASS, fx, 1, tr, 0, 0, 0, 0); } \
	static void si_##fx##tr(BLIT_ARGS) { blit(BLIT_PASS, fx, 0, tr, 0, 0, 0, 1); }
DEFP(0, 0) DEFP(0, 1) DEFP(1, 0) DEFP(1, 1)
static const blitfn s_plain[2][2] = { { pl_00, pl_01 }, { pl_10, pl_11 } };
static const blitfn s_simple[2][2] = { { si_00, si_01 }, { si_10, si_11 } };
#if defined(__GNUC__) && E12_NEON
#pragma GCC diagnostic pop
#endif

/***************************************************************************
    the drawing queue: entries of 32-bit words, a header of 4 words (the kind with the lanes' flags, the entry's
    number, the number of the other lane's entry each lane waits for) then
    Q_DRAW: a qdraw_t; Q_UP: the first pixel's VRAM index, the count, the pixels two to a word (low first);
    Q_WRAP (one word): the rest of the ring unused (the next entry at its start)

    Two lanes (ADR-120): the destination rows are cut in 16-row blocks, lane 1 draws those of the blocks b with
    b % split == split - 1, lane 0 the others. Each lane passes every entry in order and draws its rows of it (a
    blit with its clip cut to the lane's blocks: the same pixels, row by row). A pixel is then drawn by its lane in
    the list's order; what one lane reads (the source rows) the other may draw, so the emulation thread, queuing,
    keeps per block the last entry drawing there and the last entry whose other lane read it, and an entry of lane L
    waits until the other lane has passed the last entry that drew a source block of lane L's part there (read after
    write) or read a block lane L draws (write after read), by tiles of 16 rows x 256 columns (sprites and screens
    share rows; a lane's source rows are those its destination rows read). A blit whose part in one lane reads the
    other lane's destination tiles of the same blit (rows of one blit reading rows of the same blit, but not a blit
    onto itself) is drawn whole by lane 0, after lane 1 has passed all before it, and lane 1 waits for it: "solo". An
    upload over two lanes' blocks too.
    Who draws a lane: whoever holds it (lhold). Its worker; the other lane's worker where it waits for this lane (lane
    0's worker asks lane 1's worker to let go at its next entry) or has passed all its own; the emulation thread at
    the vblank and the state / ring waits (it asks lane 1's worker to let go, draws there and helps lane 0 when free)
***************************************************************************/

enum { Q_WRAP = 1, Q_DRAW, Q_UP, Q_SHOT };
#define Q_UP_MAX 2048   // (pixels an upload entry at most: an upload is split into rows and pieces)
#define QH 4            // (the header words)
// the flags (header word >> 8): lane L draws a part (F_L), lane L waits for the other lane (F_W), drawn whole by lane 0
#define F_L(l) (1u << (l))
#define F_W(l) (4u << (l))
#define F_SOLO 16u
#define REQ_EMU 1   // (lreq: the emulation thread)
#define REQ_W0 2    // (lreq[1]: lane 0's worker, waiting for lane 1)
#define BLK_SHIFT 4
#define NBLK (EPIC12_H >> BLK_SHIFT)
#define COL_SHIFT 8   // (the dependencies by tiles of 16 rows x 256 columns: sprites and screens share rows)
#define NCOL (EPIC12_W >> COL_SHIFT)

typedef struct {
	blitfn f;
	rect_t clip;
	int src_x, src_y, x, y, dimx, dimy, flipy;
	uint8_t s_alpha, d_alpha;
	clr_t tint;
} qdraw_t;
#define QD_WORDS (QH + (uint32_t)((sizeof(qdraw_t) + 3) / 4))
// Q_SHOT (ADR-150): the screen copy, w x h at the scroll position sx, sy into dst, drawn by lane 0 in the list's order
typedef struct {
	uint16_t *dst;
	uint32_t sx, sy;
	int w, h;
} qshot_t;
#define QS_WORDS (QH + (uint32_t)((sizeof(qshot_t) + 3) / 4))

static inline int blk_lane(const epic12_t *e, int blk) { return e->blane[blk]; }   // (blk < NBLK)
static inline uint32_t q_words(const epic12_t *e, const uint32_t *p, uint32_t idx)
{
	const uint32_t kind = p[0] & 0xff;
	return kind == Q_DRAW ? QD_WORDS : kind == Q_UP ? QH + 2 + (p[QH + 1] + 1) / 2 : kind == Q_SHOT ? QS_WORDS :
	       e->qwords - idx;
}

// the w x h window at sx, sy (wrapping round the VRAM) into dst
static void e12_copy(const uint16_t *vram, uint16_t *dst, uint32_t sx, uint32_t sy, int w, int h)
{
	sx &= EPIC12_W - 1;
	const uint32_t n0 = EPIC12_W - sx < (uint32_t)w ? EPIC12_W - sx : (uint32_t)w;   // (then round to column 0)
	for (int y = 0; y < h; y++) {
		const uint16_t *row = vram + (size_t)((y + sy) & (EPIC12_H - 1)) * EPIC12_STRIDE;
		memcpy(dst, row + sx, n0 * 2);
		memcpy(dst + n0, row, ((uint32_t)w - n0) * 2);
		dst += w;
	}
}

int epic12_queue(epic12_t *e, uint32_t words, int split, void (*kick)(int lane), void (*wait)(int lane, int emu))
{
	epic12_queue_free(e);
	if (words < 0x4000 || (words & (words - 1)) || !wait || split < 0 || split == 1)
		return -1;
	e->q = (uint32_t *)calloc(words, 4);
	e->bw = (uint32_t *)calloc(NBLK * NCOL * 2, 4);
	if (!e->q || !e->bw) {
		free(e->q); free(e->bw);
		e->q = e->bw = NULL;
		return -1;
	}
	e->br = e->bw + NBLK * NCOL;
	e->qwords = words;
	e->qw = e->qhead = e->qseq = 0;
	e->qlim = words;
	e->qold = s_list_old = getenv("RBX_CV1K_OLDLIST") && atoi(getenv("RBX_CV1K_OLDLIST"));
	e->split = split;
	for (int b = 0; b < NBLK; b++)
		e->blane[b] = (uint8_t)(split && b % split == split - 1);
	for (int b = NBLK - 1; b >= 0; b--)
		e->bend[b] = (uint8_t)(b < NBLK - 1 && e->blane[b + 1] == e->blane[b] ? e->bend[b + 1] : b);
	for (int l = 0; l < 2; l++)
		e->lpos[l] = e->ldone[l] = 0, e->lhold[l] = e->lreq[l] = e->lwant[l] = 0;
	e->q_kick = kick;
	e->q_wait = wait;
	return 0;
}

void epic12_queue_free(epic12_t *e)
{
	if (e->q)
		epic12_sync(e);
	free(e->q);
	free(e->bw);
	e->q = e->bw = e->br = NULL;
	e->qwords = 0;
}

// the queued entries up to number upto passed by both lanes (drawn by this thread where it can)
static void q_sync_to(epic12_t *e, uint32_t upto);

// the entries written so far given to the lanes
static inline void q_publish(epic12_t *e)
{
	if (e->qhead != e->qw)
		__atomic_store_n(&e->qhead, e->qw, __ATOMIC_RELEASE);
}

// room for n words at the write position (waiting for the lanes when the ring is full)
static uint32_t *q_reserve(epic12_t *e, uint32_t n)
{
	uint32_t idx = e->qw & (e->qwords - 1);
	const uint32_t skip = idx + n > e->qwords ? e->qwords - idx : 0;
	// (ADR-170: the lanes' positions looked at only when the room seen last is short: a lane's position only goes on,
	// so the room seen last is at most the room now, and where it is short the test is the one before)
	if (e->qold || e->qlim - e->qw < n + skip) {
		const uint32_t p0 = __atomic_load_n(&e->lpos[0], __ATOMIC_ACQUIRE);
		const uint32_t p1 = __atomic_load_n(&e->lpos[1], __ATOMIC_ACQUIRE);
		const uint32_t u0 = e->qw - p0, u1 = e->qw - p1;
		e->qlim = (u0 > u1 ? p0 : p1) + e->qwords;
		if (e->qwords - (u0 > u1 ? u0 : u1) < n + skip) {
			e->st_full++;
			q_publish(e);
			q_sync_to(e, e->qseq);   // (then empty)
			const uint32_t v0 = __atomic_load_n(&e->lpos[0], __ATOMIC_ACQUIRE);
			const uint32_t v1 = __atomic_load_n(&e->lpos[1], __ATOMIC_ACQUIRE);
			e->qlim = (e->qw - v0 > e->qw - v1 ? v0 : v1) + e->qwords;
		}
	}
	if (skip) {
		e->q[idx] = Q_WRAP;
		e->qw += skip;
		idx = 0;
	}
	return e->q + idx;
}

// (ADR-170: in a list the entries are given to the lanes every Q_PUB_WORDS words and at its end, not one by one: a
// release store (a barrier on the ARM) less per entry; the lanes are kicked at the list's end anyway)
#define Q_PUB_WORDS 1024u
static inline void q_commit(epic12_t *e, uint32_t n)
{
	e->qw += n;
	if (e->qold || e->qw - e->qhead >= Q_PUB_WORDS)
		__atomic_store_n(&e->qhead, e->qw, __ATOMIC_RELEASE);
}

// v the latest so far (wrap-safe numbers; 0 none, and an entry 2^30 back long passed)
static inline void q_later(uint32_t *best, uint32_t v, uint32_t k)
{
	if (v && (int32_t)(k - v) < (1 << 30) && (!*best || (int32_t)(v - *best) > 0))
		*best = v;
}

static inline int imax(int a, int c) { return a > c ? a : c; }
static inline int imin(int a, int c) { return a < c ? a : c; }

// the header of the entry at p: destination rows y0..y1 by column tiles dc0..dc1; the source rows, read by each
// destination row (src: the first destination row's, flipy: going up; n = 0: none) by column tiles sc0..sc1
// (ADR-170) q_header as below, the same reads, writes and result with less work: a lane's run of blocks from the
// table bend, each run's source blocks worked out once, the destination tiles' reading and marking in one pass (after
// the source tiles' reading, before their marking, as below)
static void q_header_fast(epic12_t *e, uint32_t *p, uint32_t kind, int y0, int y1, int dc0, int dc1, int src, int flipy,
                          int sc0, int sc1)
{
	const uint32_t prev = e->qseq;
	uint32_t k = prev + 1;
	if (!k)
		k = 1;
	e->qseq = k;
	const int db0 = y0 >> BLK_SHIFT, db1 = y1 >> BLK_SHIFT;
	struct { int lane, sb0, nsb; } run[NBLK + 1];
	int nrun = 0, has = 0;
	for (int blk = db0; blk <= db1 && y0 <= y1;) {
		const int own = e->blane[blk];
		const int b2 = imin(e->bend[blk], db1);
		const int ra = imax(y0, blk << BLK_SHIFT), rb = imin(y1, (b2 << BLK_SHIFT) + (1 << BLK_SHIFT) - 1);
		const int sa = (flipy ? src - (rb - y0) : src + (ra - y0)) & 0xfff;
		run[nrun].lane = own;
		run[nrun].sb0 = sa >> BLK_SHIFT;
		run[nrun].nsb = ((sa & 15) + rb - ra + 16) >> 4;   // (= ((sa & 15) + cnt - 1) / 16 + 1)
		nrun++;
		has |= 1 << own;
		blk = b2 + 1;
	}
	if (src < 0)
		sc0 = 0, sc1 = -1;
	int solo = kind == Q_UP && has == 3;
	if (has == 3 && sc0 <= dc1 && dc0 <= sc1)
		for (int r = 0; r < nrun && !solo; r++)
			for (int i = 0; i < run[r].nsb && !solo; i++) {
				const int sb = (run[r].sb0 + i) & (NBLK - 1);
				solo = sb >= db0 && sb <= db1 && e->blane[sb] != run[r].lane;
			}
	uint32_t fl = 0, need[2] = { 0, 0 };
	if (solo) {
		fl = F_SOLO | F_L(0) | F_W(1);
		need[1] = k;
		if (prev)
			fl |= F_W(0), need[0] = prev;
		e->st_solo++;
	} else {
		// (not solo: no source tile of the other lane's blocks is one of the destination tiles (in the same blocks only
		// with other columns), so each source tile's reading (bw) and marking (br) go together, and before the
		// destination tiles' as below; the latest of a set does not depend on the order)
		uint32_t best[2] = { 0, 0 };
		fl = has;   // (= F_L(0) | F_L(1) of the runs)
		if (sc0 <= sc1)
			for (int r = 0; r < nrun; r++) {
				const int l = run[r].lane;
				for (int i = 0; i < run[r].nsb; i++) {
					const int sb = (run[r].sb0 + i) & (NBLK - 1);
					if (e->blane[sb] != l) {
						const uint32_t *w = e->bw + sb * NCOL;
						uint32_t *rr = e->br + sb * NCOL;
						for (int c = sc0; c <= sc1; c++) {
							q_later(&best[l], w[c], k);
							rr[c] = k;
						}
					}
				}
			}
		for (int blk = db0; blk <= db1; blk++) {
			uint32_t *bb = &best[e->blane[blk]];
			const uint32_t *rr = e->br + blk * NCOL;
			uint32_t *w = e->bw + blk * NCOL;
			for (int c = dc0; c <= dc1; c++) {
				q_later(bb, rr[c], k);
				w[c] = k;
			}
		}
		for (int l = 0; l < 2; l++)
			if (best[l])
				fl |= F_W(l), need[l] = best[l];
	}
	p[0] = kind | fl << 8;
	p[1] = k;
	p[2] = need[0];
	p[3] = need[1];
}

static void q_header(epic12_t *e, uint32_t *p, uint32_t kind, int y0, int y1, int dc0, int dc1, int src, int flipy,
                     int sc0, int sc1)
{
	if (!e->qold) {
		q_header_fast(e, p, kind, y0, y1, dc0, dc1, src, flipy, sc0, sc1);
		return;
	}
	const uint32_t prev = e->qseq;
	uint32_t k = prev + 1;
	if (!k)
		k = 1;   // (0 means none)
	e->qseq = k;
	const int db0 = y0 >> BLK_SHIFT, db1 = y1 >> BLK_SHIFT;
	// the runs of one lane's blocks over the destination rows, with their source rows
	struct { int lane, sa, cnt; } run[NBLK + 1];
	int nrun = 0, has[2] = { 0, 0 };
	for (int b = db0; b <= db1 && y0 <= y1;) {
		const int own = blk_lane(e, b);
		int b2 = b;
		while (b2 < db1 && blk_lane(e, b2 + 1) == own)
			b2++;
		const int ra = imax(y0, b << BLK_SHIFT), rb = imin(y1, (b2 << BLK_SHIFT) + (1 << BLK_SHIFT) - 1);
		run[nrun].lane = own;
		run[nrun].sa = flipy ? src - (rb - y0) : src + (ra - y0);
		run[nrun].cnt = rb - ra + 1;
		nrun++;
		has[own] = 1;
		b = b2 + 1;
	}
	if (src < 0)   // (no source)
		sc0 = 0, sc1 = -1;
	// (a lane's part reading the other lane's destination tiles of the same blit, an upload over both lanes: whole)
	int solo = kind == Q_UP && has[0] && has[1];
	if (has[0] && has[1] && sc0 <= dc1 && dc0 <= sc1)
		for (int r = 0; r < nrun && !solo; r++) {
			const int sb0 = (run[r].sa & 0xfff) >> BLK_SHIFT, nsb = (((run[r].sa & 0xfff) & 15) + run[r].cnt - 1) / 16 + 1;
			for (int i = 0; i < nsb && !solo; i++) {
				const int sb = (sb0 + i) & (NBLK - 1);
				solo = sb >= db0 && sb <= db1 && blk_lane(e, sb) != run[r].lane;
			}
		}
	uint32_t fl = 0, need[2] = { 0, 0 };
	if (solo) {   // (lane 0 after all of lane 1 before it, lane 1 after this one)
		fl = F_SOLO | F_L(0) | F_W(1);
		need[1] = k;
		if (prev)
			fl |= F_W(0), need[0] = prev;
		e->st_solo++;
	} else {
		uint32_t best[2] = { 0, 0 };
		for (int r = 0; r < nrun; r++) {
			const int l = run[r].lane;
			fl |= F_L(l);
			if (sc0 <= sc1) {   // (its source tiles the other lane draws: their last drawing)
				const int sb0 = (run[r].sa & 0xfff) >> BLK_SHIFT, nsb = (((run[r].sa & 0xfff) & 15) + run[r].cnt - 1) / 16 + 1;
				for (int i = 0; i < nsb; i++) {
					const int sb = (sb0 + i) & (NBLK - 1);
					if (blk_lane(e, sb) != l)
						for (int c = sc0; c <= sc1; c++)
							q_later(&best[l], e->bw[sb * NCOL + c], k);
				}
			}
		}
		for (int b = db0; b <= db1; b++)   // (its tiles: the other lane's last reading)
			for (int c = dc0; c <= dc1; c++)
				q_later(&best[blk_lane(e, b)], e->br[b * NCOL + c], k);
		for (int l = 0; l < 2; l++)
			if (best[l])
				fl |= F_W(l), need[l] = best[l];
		for (int b = db0; b <= db1; b++)
			for (int c = dc0; c <= dc1; c++)
				e->bw[b * NCOL + c] = k;
		if (sc0 <= sc1)
			for (int r = 0; r < nrun; r++) {
				const int sb0 = (run[r].sa & 0xfff) >> BLK_SHIFT, nsb = (((run[r].sa & 0xfff) & 15) + run[r].cnt - 1) / 16 + 1;
				for (int i = 0; i < nsb; i++) {
					const int sb = (sb0 + i) & (NBLK - 1);
					if (blk_lane(e, sb) != run[r].lane)
						for (int c = sc0; c <= sc1; c++)
							e->br[sb * NCOL + c] = k;
				}
			}
	}
	p[0] = kind | fl << 8;
	p[1] = k;
	p[2] = need[0];
	p[3] = need[1];
}

static void q_draw(epic12_t *e, const qdraw_t *d)
{
	// (the destination rows and columns, inside the clip: gfx_draw drops a blit outside it; the first one's source
	// row, and the source's columns: a source past the right edge is not drawn)
	const int y0 = imax(d->y, d->clip.min_y), y1 = imin(d->y + d->dimy - 1, d->clip.max_y);
	const int x0 = imax(d->x, d->clip.min_x), x1 = imin(d->x + d->dimx - 1, d->clip.max_x);
	const int src = d->flipy ? d->src_y + d->dimy - 1 - (y0 - d->y) : d->src_y + (y0 - d->y);
	const int sx1 = imin(d->src_x + d->dimx - 1, EPIC12_W - 1);
	uint32_t *p = q_reserve(e, QD_WORDS);
	q_header(e, p, Q_DRAW, y0, y1, x0 >> COL_SHIFT, x1 >> COL_SHIFT, src & 0xfff, d->flipy, d->src_x >> COL_SHIFT,
	         sx1 >> COL_SHIFT);
	memcpy(p + QH, d, sizeof(*d));
	q_commit(e, QD_WORDS);
	e->st_draw++;
}

static void up_put(uint16_t *vram, uint32_t start, uint32_t n, const uint32_t *px)
{
	const uint32_t total = (uint32_t)EPIC12_W * EPIC12_H;
	if (start >= total)
		return;
	if (n > total - start)
		n = total - start;
	// (start: a pixel's number, y * EPIC12_W + x; a piece past the right edge goes on in the next row)
	for (uint32_t i = 0; i < n;) {
		const uint32_t at = start + i, x = at & (EPIC12_W - 1);
		const uint32_t m = EPIC12_W - x < n - i ? EPIC12_W - x : n - i;
		uint16_t *dst = vram + (size_t)(at / EPIC12_W) * EPIC12_STRIDE + x;
		for (uint32_t k = 0; k < m; k++, i++)
			dst[k] = (uint16_t)(px[i >> 1] >> ((i & 1) << 4));
	}
}

// lane l's rows of a draw: the blit for each run of the lane's blocks, its clip cut to them
static uint32_t draw_part(epic12_t *e, const qdraw_t *d, int l)
{
	const int y0 = imax(d->y, d->clip.min_y), y1 = imin(d->y + d->dimy - 1, d->clip.max_y);
	const int x0 = imax(d->x, d->clip.min_x), x1 = imin(d->x + d->dimx - 1, d->clip.max_x);
	rect_t c = d->clip;
	uint32_t rows = 0;
	for (int b = y0 >> BLK_SHIFT, bl = y1 >> BLK_SHIFT; b <= bl;) {
		const int own = blk_lane(e, b);
		int b2 = b;
		while (b2 < bl && blk_lane(e, b2 + 1) == own)
			b2++;
		if (own == l) {
			c.min_y = imax(y0, b << BLK_SHIFT);
			c.max_y = imin(y1, (b2 << BLK_SHIFT) + (1 << BLK_SHIFT) - 1);
			d->f(e->vram, &c, d->src_x, d->src_y, d->x, d->y, d->dimx, d->dimy, d->flipy, d->s_alpha, d->d_alpha, d->tint);
			rows += (uint32_t)(c.max_y - c.min_y + 1);
		}
		b = b2 + 1;
	}
	return rows * (uint32_t)(x1 - x0 + 1);
}

enum { R_END, R_UPTO, R_STOP, R_BLOCK };

// lane l drawn by who (holding it) up to entry upto (NULL: all queued), stopping when the lane is asked for (lreq &
// stopmask, or its worker wants it and want); R_BLOCK: waits for the other lane to pass *blk
static int lane_run(epic12_t *e, int l, int who, const uint32_t *upto, int stopmask, int want, uint32_t *blk)
{
	const uint32_t mask = e->qwords - 1;
	uint32_t t = e->lpos[l];
	for (;;) {
		if (upto && (int32_t)(e->ldone[l] - *upto) >= 0)
			return R_UPTO;
		if (t == __atomic_load_n(&e->qhead, __ATOMIC_ACQUIRE))
			return R_END;
		if ((__atomic_load_n(&e->lreq[l], __ATOMIC_ACQUIRE) & stopmask) ||
		    (want && __atomic_load_n(&e->lwant[l], __ATOMIC_ACQUIRE)))
			return R_STOP;
		const uint32_t idx = t & mask;
		const uint32_t *p = e->q + idx;
		const uint32_t kind = p[0] & 0xff, fl = p[0] >> 8;
		if (kind == Q_WRAP) {
			t += e->qwords - idx;
			__atomic_store_n(&e->lpos[l], t, __ATOMIC_RELEASE);
			continue;
		}
		if ((fl & F_W(l)) && (int32_t)(__atomic_load_n(&e->ldone[!l], __ATOMIC_ACQUIRE) - p[2 + l]) < 0) {
			*blk = p[2 + l];
			return R_BLOCK;
		}
		if (kind == Q_SHOT) {
			if (fl & F_L(l)) {
				qshot_t s;
				memcpy(&s, p + QH, sizeof(s));
				e12_copy(e->vram, s.dst, s.sx, s.sy, s.w, s.h);
			}
		} else if (fl & F_L(l)) {
			uint32_t px;
			if (kind == Q_DRAW) {
				qdraw_t d;
				memcpy(&d, p + QH, sizeof(d));
				if (fl & F_SOLO) {
					d.f(e->vram, &d.clip, d.src_x, d.src_y, d.x, d.y, d.dimx, d.dimy, d.flipy, d.s_alpha, d.d_alpha, d.tint);
					px = (uint32_t)d.dimx * (uint32_t)d.dimy;
				} else
					px = draw_part(e, &d, l);
			} else {
				up_put(e->vram, p[QH], p[QH + 1], p + QH + 2);
				px = p[QH + 1];
			}
			e->st_ent[who][l]++;
			e->st_px[who][l] += px;
		}
		t += q_words(e, p, idx);
		__atomic_store_n(&e->lpos[l], t, __ATOMIC_RELEASE);
		__atomic_store_n(&e->ldone[l], p[1], __ATOMIC_RELEASE);
	}
}

static inline int lane_hold(epic12_t *e, int l, int who)
{
	int z = 0;
	return __atomic_compare_exchange_n(&e->lhold[l], &z, 1 + who, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}
static inline void lane_free(epic12_t *e, int l) { __atomic_store_n(&e->lhold[l], 0, __ATOMIC_RELEASE); }
static inline int lane_idle(epic12_t *e, int l)
{
	return __atomic_load_n(&e->lpos[l], __ATOMIC_ACQUIRE) == __atomic_load_n(&e->qhead, __ATOMIC_ACQUIRE);
}
static inline void q_spin(epic12_t *e, int l, int emu, int *spins)
{
	if (++*spins >= 4000)   // (a short wait spins: a sleep is ~100 us on the Vita)
		e->q_wait(l, emu);
}

void epic12_drain_lane(epic12_t *e, int l)
{
	const int who = l ? EPIC12_WHO_W1 : EPIC12_WHO_W0, o = !l;
	int spins = 0, r = R_END;
	uint32_t blk;
	for (;;) {
		// the lane (a helper holding it lets go at its next entry; the emulation thread holding or asking: its)
		__atomic_store_n(&e->lwant[l], 1, __ATOMIC_RELEASE);
		while (!lane_hold(e, l, who)) {
			if ((__atomic_load_n(&e->lreq[l], __ATOMIC_ACQUIRE) & REQ_EMU) || lane_idle(e, l)) {
				__atomic_store_n(&e->lwant[l], 0, __ATOMIC_RELEASE);
				return;
			}
			q_spin(e, l, 0, &spins);
		}
		__atomic_store_n(&e->lwant[l], 0, __ATOMIC_RELEASE);
		for (;;) {
			uint32_t need = 0;
			if (!l)
				__atomic_and_fetch(&e->lreq[1], ~REQ_W0, __ATOMIC_ACQ_REL);
			r = lane_run(e, l, who, NULL, l ? REQ_EMU | REQ_W0 : REQ_EMU, 0, &need);
			if (r != R_BLOCK)
				break;
			// waiting for the other lane: drawn here as far as needed when free (lane 0's worker asks lane 1's
			// worker to let go)
			if (!l)
				__atomic_or_fetch(&e->lreq[1], REQ_W0, __ATOMIC_ACQ_REL);
			if (!(__atomic_load_n(&e->lreq[o], __ATOMIC_ACQUIRE) & REQ_EMU) && lane_hold(e, o, who)) {
				if (!l)
					__atomic_and_fetch(&e->lreq[1], ~REQ_W0, __ATOMIC_ACQ_REL);
				e->st_help[who]++;
				lane_run(e, o, who, &need, REQ_EMU, 0, &blk);
				lane_free(e, o);
				spins = 0;
				continue;
			}
			if (__atomic_load_n(&e->lreq[l], __ATOMIC_ACQUIRE) & REQ_EMU)
				break;
			e->st_block[who]++;
			q_spin(e, l, 0, &spins);
		}
		if (!l)
			__atomic_and_fetch(&e->lreq[1], ~REQ_W0, __ATOMIC_ACQ_REL);
		lane_free(e, l);
		if (r == R_BLOCK || (r == R_STOP && (__atomic_load_n(&e->lreq[l], __ATOMIC_ACQUIRE) & REQ_EMU)))
			return;   // (the emulation thread has it)
		if (r == R_END)
			break;
		// (R_STOP: lane 0's worker drew some of lane 1: taken back)
	}
	// its own passed: the other lane. Lane 0's worker (a whole core) asks lane 1's to let go at its next entry and
	// draws the rest of lane 1; lane 1's draws lane 0 only while free, and lets lane 0's worker have it back
	if (lane_idle(e, o) || (__atomic_load_n(&e->lreq[o], __ATOMIC_ACQUIRE) & REQ_EMU))
		return;
	if (!l) {
		__atomic_or_fetch(&e->lreq[1], REQ_W0, __ATOMIC_ACQ_REL);
		spins = 0;
		while (!lane_hold(e, 1, who)) {
			if (lane_idle(e, 1) || (__atomic_load_n(&e->lreq[1], __ATOMIC_ACQUIRE) & REQ_EMU) ||
			    (__atomic_load_n(&e->lhold[1], __ATOMIC_ACQUIRE) == 1 + EPIC12_WHO_EMU)) {
				__atomic_and_fetch(&e->lreq[1], ~REQ_W0, __ATOMIC_ACQ_REL);
				return;
			}
			q_spin(e, 1, 0, &spins);
		}
		__atomic_and_fetch(&e->lreq[1], ~REQ_W0, __ATOMIC_ACQ_REL);
	} else if (!lane_hold(e, 0, who))
		return;
	e->st_help[who]++;
	lane_run(e, o, who, NULL, REQ_EMU, l, &blk);
	lane_free(e, o);
}

static void q_sync_to(epic12_t *e, uint32_t upto)   // (the emulation thread only)
{
	const int who = EPIC12_WHO_EMU;
	int spins = 0, asked = 0;
	const unsigned pa = rbx_prof_enter(EPIC12_PA_SYNC);
	for (;;) {
		const int d0 = (int32_t)(__atomic_load_n(&e->ldone[0], __ATOMIC_ACQUIRE) - upto) >= 0;
		const int d1 = (int32_t)(__atomic_load_n(&e->ldone[1], __ATOMIC_ACQUIRE) - upto) >= 0;
		if (d0 && d1)
			break;
		int moved = 0;
		uint32_t blk;
		for (int l = 1; l >= 0; l--) {
			if ((l ? d1 : d0) || !lane_hold(e, l, who))
				continue;
			const uint32_t before = e->ldone[l];
			lane_run(e, l, who, &upto, 0, 0, &blk);
			moved |= e->ldone[l] != before;
			lane_free(e, l);
		}
		if (moved) {
			spins = 0;
			continue;
		}
		// lane 1's worker drawing: asked to let go at its next entry (lane 0's, a whole core, goes on)
		if (!d1 && !asked && __atomic_load_n(&e->lhold[1], __ATOMIC_ACQUIRE) == 1 + EPIC12_WHO_W1) {
			__atomic_or_fetch(&e->lreq[1], REQ_EMU, __ATOMIC_ACQ_REL);
			asked = 1;
			e->st_take++;
		}
		q_spin(e, !d1 ? 1 : 0, 1, &spins);
	}
	if (asked)
		__atomic_and_fetch(&e->lreq[1], ~REQ_EMU, __ATOMIC_ACQ_REL);
	for (int l = 0; l < 2; l++)
		if (!lane_idle(e, l) && e->q_kick)
			e->q_kick(l);   // (the rest, drawn on)
	rbx_prof_leave(pa);
}

void epic12_sync(epic12_t *e)
{
	if (e->q && e->qseq)
		q_sync_to(e, e->qseq);
}

static int q_hits(const uint32_t *p, uint32_t sx, uint32_t sy, uint32_t w, uint32_t h)
{
	int x0, x1, y0, y1;
	const uint32_t kind = p[0] & 0xff;
	if (!((p[0] >> 8) & (F_L(0) | F_L(1))))
		return 0;
	if (kind == Q_DRAW) {
		qdraw_t d;
		memcpy(&d, p + QH, sizeof(d));
		x0 = imax(d.x, d.clip.min_x);
		x1 = imin(d.x + d.dimx - 1, d.clip.max_x);
		y0 = imax(d.y, d.clip.min_y);
		y1 = imin(d.y + d.dimy - 1, d.clip.max_y);
		if (x0 > x1 || y0 > y1)
			return 0;
	} else if (kind == Q_UP) {
		const uint32_t total = (uint32_t)EPIC12_W * EPIC12_H, start = p[QH], n = p[QH + 1];
		if (start >= total || !n)
			return 0;
		const uint32_t last = start + n - 1 < total ? start + n - 1 : total - 1;
		y0 = (int)(start / EPIC12_W);
		y1 = (int)(last / EPIC12_W);
		if (y0 == y1) {
			x0 = (int)(start % EPIC12_W);
			x1 = (int)(last % EPIC12_W);
		} else
			x0 = 0, x1 = EPIC12_W - 1;
	} else
		return 0;
	return e12_circ_hit(x0, x1, sx, w, EPIC12_W) && e12_circ_hit(y0, y1, sy, h, EPIC12_H);
}

void epic12_sync_screen(epic12_t *e, int w, int h)
{
	if (!e->q)
		return;
	const uint32_t mask = e->qwords - 1;
	const uint32_t sx = e->scroll_x & (EPIC12_W - 1), sy = e->scroll_y & (EPIC12_H - 1);
	// (from the lane further behind: the last entry not yet passed by both that reaches the screen)
	const uint32_t p0 = __atomic_load_n(&e->lpos[0], __ATOMIC_ACQUIRE), p1 = __atomic_load_n(&e->lpos[1], __ATOMIC_ACQUIRE);
	uint32_t last = 0;
	for (uint32_t t = e->qw - p0 > e->qw - p1 ? p0 : p1; t != e->qw;) {
		const uint32_t idx = t & mask;
		const uint32_t *p = e->q + idx;
		if (q_hits(p, sx, sy, (uint32_t)w, (uint32_t)h))
			last = p[1];
		t += q_words(e, p, idx);
	}
	if (last) {
		e->st_sync++;
		q_sync_to(e, last);
	} else
		e->st_skip++;
}

// The screen copy as an entry (ADR-150): lane 0 copies the window when it gets there, after every entry before it.
// It reads the window like a blit's source with its destination outside the VRAM: lane 0 waits for lane 1's last
// drawing in the window's tiles of lane 1 (read after write), and lane 1's later drawing there waits for it (write
// after read, its br); lane 0's own tiles are in order. Nothing the CPU sees depends on it.
uint32_t epic12_queue_shot(epic12_t *e, uint16_t *dst, int w, int h)
{
	const uint32_t sx = e->scroll_x & (EPIC12_W - 1), sy = e->scroll_y & (EPIC12_H - 1);
	if (!e->q) {
		e12_copy(e->vram, dst, sx, sy, w, h);
		return 0;
	}
	uint32_t *p = q_reserve(e, QS_WORDS);
	uint32_t k = e->qseq + 1;
	if (!k)
		k = 1;
	e->qseq = k;
	uint32_t need = 0;
	const int nb = imin((int)((sy & 15) + (uint32_t)h - 1) / 16 + 1, NBLK);
	const int nc = imin((int)((sx & ((1 << COL_SHIFT) - 1)) + (uint32_t)w - 1) / (1 << COL_SHIFT) + 1, NCOL);
	for (int i = 0; i < nb; i++) {
		const int blk = (int)((sy >> BLK_SHIFT) + (uint32_t)i) & (NBLK - 1);
		if (!blk_lane(e, blk))
			continue;
		for (int j = 0; j < nc; j++) {
			const int c = (int)((sx >> COL_SHIFT) + (uint32_t)j) & (NCOL - 1);
			q_later(&need, e->bw[blk * NCOL + c], k);
			e->br[blk * NCOL + c] = k;
		}
	}
	p[0] = Q_SHOT | (F_L(0) | (need ? F_W(0) : 0)) << 8;
	p[1] = k;
	p[2] = need;
	p[3] = 0;
	const qshot_t s = { dst, sx, sy, w, h };
	memcpy(p + QH, &s, sizeof(s));
	q_commit(e, QS_WORDS);
	q_publish(e);
	if (e->q_kick) {
		e->q_kick(0);
		if (e->split)
			e->q_kick(1);
	}
	return k;
}

void epic12_shot_wait(epic12_t *e, uint32_t k, int emu)
{
	if (!k || !e->q)
		return;
	if (emu) {
		q_sync_to(e, k);
		return;
	}
	// (sleeping at once: on the Vita the render thread shares CPU2 with lane 0's worker, below it)
	while ((int32_t)(__atomic_load_n(&e->ldone[0], __ATOMIC_ACQUIRE) - k) < 0)
		e->q_wait(0, 0);
}

/***************************************************************************
    the operation list
***************************************************************************/

typedef struct {
	epic12_t *e;
	rect_t clip;
	uint64_t delay_ns;
} run_t;

static inline uint16_t next_word(const epic12_t *e, uint32_t *addr)
{
	const uint16_t v = *(const uint16_t *)(e->ram + (((*addr & e->rammask) & ~1u) ^ 2));
	*addr += 2;
	return v;
}

static void clip_set(rect_t *r, int x0, int x1, int y0, int y1) { r->min_x = x0; r->max_x = x1; r->min_y = y0; r->max_y = y1; }

static void clip_screen(run_t *r)
{
	const int cx = (int)r->e->clip_x, cy = (int)r->e->clip_y;
	clip_set(&r->clip, cx - CLIP_MARGIN, cx + 320 - 1 + CLIP_MARGIN, cy - CLIP_MARGIN, cy + 240 - 1 + CLIP_MARGIN);
}

static void idle_blitter(run_t *r, int bytes)
{
	r->e->idle_op_bytes += (uint32_t)bytes;
	if (r->e->idle_op_bytes >= CHUNK_BYTES) {
		r->e->idle_op_bytes -= CHUNK_BYTES;
		r->delay_ns += CHUNK_INTERVAL_NS;
	}
}

// (ADR-170: the loops below counted: per 32 pixels one, and one more where the piece crosses a 32-pixel line)
static inline int vram_pieces(uint32_t start, uint32_t dim)
{
	const uint32_t s = start & 31, nf = dim >> 5, rest = dim & 31;
	return (int)(nf + (rest != 0) + (s ? nf : 0) + (rest && s + rest > 32));
}

static uint16_t vram_accesses(uint16_t start_x, uint16_t start_y, uint16_t dimx, uint16_t dimy)
{
	if (!s_list_old)
		return (uint16_t)(vram_pieces(start_x, dimx) * vram_pieces(start_y, dimy));
	int x_rows = 0, num = 0;
	for (int xp = dimx; xp > 0; xp -= 32) {
		x_rows++;
		if (((start_x & 31) + (xp < 32 ? xp : 32)) > 32)
			x_rows++;
	}
	for (int yp = dimy; yp > 0; yp -= 32) {
		num += x_rows;
		if (((start_y & 31) + (yp < 32 ? yp : 32)) > 32)
			num += x_rows;
	}
	return (uint16_t)num;
}

// (ADR-170) n 16-bit words of the RAM from the even byte address a (not wrapping) two to a word, low first, as
// next_word reads them: the word at a is the high half of the host-order 32-bit word at a & ~3, the one at a + 2 its
// low half
static void up_copy(uint32_t *px, const uint8_t *ram, uint32_t a, uint32_t n)
{
	const uint32_t *w = (const uint32_t *)(ram + (a & ~3u));
	const uint32_t pairs = n >> 1;
	if (!(a & 2)) {
		for (uint32_t i = 0; i < pairs; i++)
			px[i] = w[i] >> 16 | w[i] << 16;
		if (n & 1)
			px[pairs] = w[pairs] >> 16;
	} else {   // (each pair: the low half of one word, the high half of the next)
		for (uint32_t i = 0; i < pairs; i++)
			px[i] = (w[i] & 0xffffu) | (w[i + 1] & 0xffff0000u);
		if (n & 1)
			px[pairs] = w[pairs] & 0xffffu;
	}
}

static void gfx_upload(run_t *r, uint32_t *addr)
{
	epic12_t *e = r->e;
	next_word(e, addr); next_word(e, addr);   // 0x20000000
	next_word(e, addr); next_word(e, addr);   // 0x99999999
	uint32_t dst_x_start = next_word(e, addr) & 0x1fff;
	uint32_t dst_y_start = next_word(e, addr) & 0x0fff;
	const uint32_t dimx = (next_word(e, addr) & 0x1fff) + 1;
	const uint32_t dimy = (next_word(e, addr) & 0x0fff) + 1;
	if (e->q) {   // (queued: the rows in pieces, the pixels copied now)
		e->st_up++;
		for (uint32_t y = 0; y < dimy; y++) {
			const uint32_t row = ((dst_y_start + y) & 0xfff) * EPIC12_W + dst_x_start;
			for (uint32_t x = 0; x < dimx; x += Q_UP_MAX) {
				const uint32_t n = dimx - x < Q_UP_MAX ? dimx - x : Q_UP_MAX, words = QH + 2 + (n + 1) / 2;
				const uint32_t start = row + x, total = (uint32_t)EPIC12_W * EPIC12_H;
				uint32_t *p = q_reserve(e, words);
				// (the rows it reaches: a piece past the right edge runs into the next row; past the end, none)
				const int r0 = start < total ? (int)(start / EPIC12_W) : 1;
				const int r1 = start < total ? (int)((start + n - 1 < total ? start + n - 1 : total - 1) / EPIC12_W) : 0;
				const int c0 = r0 == r1 ? (int)(start % EPIC12_W) >> COL_SHIFT : 0;
				const int c1 = r0 == r1 ? (int)((start + n - 1) % EPIC12_W) >> COL_SHIFT : NCOL - 1;
				q_header(e, p, Q_UP, r0, r1, c0, c1, -1, 0, 0, -1);
				p[QH] = start;
				p[QH + 1] = n;
				uint32_t *px = p + QH + 2;
				uint32_t i = 0;
				const uint32_t a = *addr & e->rammask;
				if (!e->qold && a + 2 * n - 1 <= e->rammask) {   // (ADR-170: not wrapping: two words at a time)
					up_copy(px, e->ram, a & ~1u, n);
					*addr += 2 * n;
					i = n;
				}
				for (; i + 1 < n; i += 2) {
					const uint32_t lo = next_word(e, addr);
					px[i >> 1] = lo | (uint32_t)next_word(e, addr) << 16;
				}
				if (i < n)
					px[i >> 1] = next_word(e, addr);
				q_commit(e, words);
			}
		}
	} else
	for (uint32_t y = 0; y < dimy; y++) {
		// (MAME's bitmap rows: a row past the bottom or the right edge lands in the next row / out of the bitmap;
		// the VRAM is one block, wrapped here)
		const uint32_t row = ((dst_y_start + y) & 0xfff) * EPIC12_W + dst_x_start, total = (uint32_t)EPIC12_W * EPIC12_H;
		for (uint32_t x = 0; x < dimx; x++) {
			const uint16_t pen = next_word(e, addr);
			const uint32_t at = row + x;   // (the pixel's number, y * EPIC12_W + x)
			if (at < total)
				e->vram[(size_t)(at / EPIC12_W) * EPIC12_STRIDE + (at & (EPIC12_W - 1))] = pen;
		}
	}
	// (gfx_upload_shadow_copy)
	const uint32_t num_sram_clk = (16 + dimx * dimy * 2) / 4;
	r->delay_ns += (uint64_t)num_sram_clk * SRAM_CLK_NS;
	e->idle_op_bytes = 0;
}

static void draw_delay(run_t *r, uint16_t src_x_start, uint16_t src_y_start, uint16_t dst_x_start, uint16_t dst_y_start,
                       uint16_t src_dimx, uint16_t src_dimy)
{
	// gfx_draw_shadow_copy
	uint16_t dst_x_end = (uint16_t)(dst_x_start + src_dimx - 1);
	uint16_t dst_y_end = (uint16_t)(dst_y_start + src_dimy - 1);
	const rect_t *c = &r->clip;
	if (dst_x_start > c->max_x || dst_x_end < c->min_x || dst_y_start > c->max_y || dst_y_end < c->min_y) {
		idle_blitter(r, DRAW_OP_BYTES);
		return;
	}
	if (dst_x_start < (uint16_t)c->min_x) dst_x_start = (uint16_t)c->min_x;
	if (dst_y_start < (uint16_t)c->min_y) dst_y_start = (uint16_t)c->min_y;
	if (dst_x_end > (uint16_t)c->max_x) dst_x_end = (uint16_t)c->max_x;
	if (dst_y_end > (uint16_t)c->max_y) dst_y_end = (uint16_t)c->max_y;
	src_dimx = (uint16_t)(dst_x_end - dst_x_start + 1);
	src_dimy = (uint16_t)(dst_y_end - dst_y_start + 1);
	r->e->idle_op_bytes = 0;
	const int src_rows = vram_accesses(src_x_start, src_y_start, src_dimx, src_dimy);
	const int dst_rows = vram_accesses(dst_x_start, dst_y_start, src_dimx, src_dimy);
	dst_x_start = (uint16_t)(dst_x_start - (dst_x_start & 3));
	dst_x_end = (uint16_t)(dst_x_end + ((4 - ((dst_x_end + 1) & 3)) & 3));
	const uint16_t dst_dimx = (uint16_t)(dst_x_end - dst_x_start + 1);
	const uint16_t dst_dimy = (uint16_t)(dst_y_end - dst_y_start + 1);
	const uint32_t num_vram_clk = (uint32_t)(src_dimx * src_dimy / 4 + dst_dimx * dst_dimy / 2 + src_rows * 6 +
	                                         dst_rows * (20 + 11) + 12);
	r->delay_ns += (uint64_t)num_vram_clk * VRAM_CLK_NS;
}

static void gfx_draw(run_t *r, uint32_t *addr)
{
	epic12_t *e = r->e;
	uint16_t op[10];
	const uint32_t a = *addr & e->rammask;
	if (!s_list_old && a + 19 <= e->rammask) {   // (ADR-170: not wrapping, the words without the mask each)
		const uint32_t base = a & ~1u;   // (the word at an even address: the 16 bits at its offset ^ 2)
		for (uint32_t i = 0; i < 10; i++)
			op[i] = *(const uint16_t *)(e->ram + ((base + 2 * i) ^ 2));
		*addr += 20;
	} else
		for (int i = 0; i < 10; i++)
			op[i] = next_word(e, addr);
	const uint16_t attr = op[0];
	const uint16_t alpha = op[1];
	uint16_t src_x = op[2];
	uint16_t src_y = op[3];
	const uint16_t dst_x_start = op[4];
	const uint16_t dst_y_start = op[5];
	const uint16_t w = op[6];
	const uint16_t h = op[7];
	const uint16_t tint_r = op[8];
	const uint16_t tint_gb = op[9];

	draw_delay(r, src_x, src_y, dst_x_start, dst_y_start, (uint16_t)((w & 0x1fff) + 1), (uint16_t)((h & 0x0fff) + 1));

	const int d_mode = attr & 0x0007;
	const int s_mode = (attr & 0x0070) >> 4;
	const int trans = (attr >> 8) & 1;
	int blend = (attr >> 9) & 1;
	const int flipy = (attr >> 10) & 1;
	const int flipx = (attr >> 11) & 1;
	const uint8_t d_alpha = (uint8_t)((alpha & 0x00ff) >> 3);
	const uint8_t s_alpha = (uint8_t)(((alpha & 0xff00) >> 8) >> 3);
	src_x &= 0x1fff;
	src_y &= 0x0fff;
	const int x = (int16_t)dst_x_start;
	const int y = (int16_t)dst_y_start;
	const int dimx = (w & 0x1fff) + 1;
	const int dimy = (h & 0x0fff) + 1;
	clr_t tint;
	tint.r = (uint8_t)((tint_r & 0xff) >> 2);
	tint.g = (uint8_t)(((tint_gb >> 8) & 0xff) >> 2);
	tint.b = (uint8_t)((tint_gb & 0xff) >> 2);
	const int tinted = tint.r != 0x20 || tint.g != 0x20 || tint.b != 0x20;
	if ((s_mode == 0 && s_alpha == 0x1f) && (d_mode == 4 && d_alpha == 0x1f))
		blend = 0;

	// (MAME's clip may reach outside the bitmap: kept inside it here)
	rect_t clip = r->clip;
	if (clip.min_x < 0) clip.min_x = 0;
	if (clip.min_y < 0) clip.min_y = 0;
	if (clip.max_x > EPIC12_W - 1) clip.max_x = EPIC12_W - 1;
	if (clip.max_y > EPIC12_H - 1) clip.max_y = EPIC12_H - 1;
	if (x > clip.max_x || y > clip.max_y || x + dimx - 1 < clip.min_x || y + dimy - 1 < clip.min_y)
		return;

	blitfn f;
	if (tinted)
		f = blend ? s_blend[flipx][1][trans][s_mode | (d_mode << 3)] : s_plain[flipx][trans];
	else if (!blend)
		f = s_simple[flipx][trans];
	else
		f = s_blend[flipx][0][trans][s_mode | (d_mode << 3)];
	if (e->q) {
		const qdraw_t d = { f, clip, src_x, src_y, x, y, dimx, dimy, flipy, s_alpha, d_alpha, tint };
		q_draw(e, &d);
		return;
	}
	f(e->vram,&clip, src_x, src_y, x, y, dimx, dimy, flipy, s_alpha, d_alpha, tint);
}

static void gfx_exec(epic12_t *e, int64_t now)   // (the emulation thread)
{
	const unsigned pa = rbx_prof_enter(EPIC12_PA_LIST);
	run_t r;
	r.e = e;
	r.delay_ns = 0;
	uint32_t addr = e->gfx_addr & 0x1fffffff;
	clip_screen(&r);
	for (;;) {
		const uint16_t data = next_word(e, &addr);
		switch (data & 0xf000) {
		case 0x0000:
		case 0xf000:
			goto done;
		case 0xc000:
			if (next_word(e, &addr))   // cliptype
				clip_screen(&r);
			else
				clip_set(&r.clip, 0, 0x2000 - 1, 0, 0x1000 - 1);
			idle_blitter(&r, CLIP_OP_BYTES);
			break;
		case 0x2000:
			addr -= 2;
			gfx_upload(&r, &addr);
			break;
		case 0x1000:
			addr -= 2;
			gfx_draw(&r, &addr);
			break;
		default:
			goto done;
		}
	}
done:
	r.delay_ns += (r.delay_ns / H_LINE_PERIOD_NS) * H_LINE_DURATION_NS;
	// the timer: delay ns from now, in CPU clocks (rounded up)
	e->busy_until = now + (int64_t)((r.delay_ns * e->cpu_hz + 999999999u) / 1000000000u);
	e->st_exec++;
	if (e->q)
		q_publish(e);
	if (e->q && e->q_kick) {   // (the lanes draw the list while the CPU runs on)
		e->q_kick(0);
		if (e->split)
			e->q_kick(1);
	}
	rbx_prof_leave(pa);
}

void epic12_reset(epic12_t *e)
{
	epic12_sync(e);
	epic12_init_tables();
	s_scalar = getenv("RBX_EPIC12_SCALAR") && atoi(getenv("RBX_EPIC12_SCALAR"));
	s_pf = getenv("RBX_EPIC12_PF") ? atoi(getenv("RBX_EPIC12_PF")) : 1;
	s_list_old = getenv("RBX_CV1K_OLDLIST") && atoi(getenv("RBX_CV1K_OLDLIST"));
#ifdef RBX_E12_PROF
	static int s_prof_at;
	if (!s_prof_at) { s_prof_at = 1; atexit(e12_prof_dump); atexit(e12m_dump); }
#endif
	e->busy_until = 0;
}

uint32_t epic12_r(epic12_t *e, uint32_t offset, uint32_t mask, int64_t now)
{
	(void)mask;
	switch (offset * 4) {
	case 0x10: return now < e->busy_until ? 0x00000000 : 0x00000010;
	case 0x24: return 0xffffffff;
	case 0x28: return 0xffffffff;
	case 0x50: return e->dsw ? e->dsw(e->user) : 0;
	default: return 0;
	}
}

#define COMBINE(reg, d, m) ((reg) = ((reg) & ~(m)) | ((d) & (m)))

void epic12_w(epic12_t *e, uint32_t offset, uint32_t data, uint32_t mask, int64_t now)
{
	switch (offset * 4) {
	case 0x04:
		if ((mask & 0xff) && (data & 1))
			gfx_exec(e, now);
		break;
	case 0x08: COMBINE(e->gfx_addr, data, mask); break;
	case 0x14: COMBINE(e->scroll_x, data, mask); break;
	case 0x18: COMBINE(e->scroll_y, data, mask); break;
	case 0x40: COMBINE(e->clip_x, data, mask); break;
	case 0x44: COMBINE(e->clip_y, data, mask); break;
	default: break;
	}
}

void epic12_screen(const epic12_t *e, uint16_t *dst, int w, int h)
{
	e12_copy(e->vram, dst, e->scroll_x, e->scroll_y, w, h);
}
