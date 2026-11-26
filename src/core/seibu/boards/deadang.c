/*  Seibu Dead Angle / Lead Angle / Gang Hunter: two NEC V30s (the main one with the sprites, the text layer, the
    palette and the scroll registers; the sub one with the top playfield), two ROM-mapped playfields, the Seibu Sound
    System in its Dead Angle variant (SEI80BU-encrypted Z80, two YM2203s, two Seibu ADPCM interfaces with MSM5205s).
    Ported from MAME 0.289 src/mame/seibu/deadang.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail, David Haywood).
    This file is part of ReBOX. GPL-2.0-or-later.

    The CPUs interleaved as MAME's scheduler does it (sched.h; both CPUs' scanline timers, every line). Not done:
    Pop'n Run (not working in MAME), the flip screen, the trackballs' movement (still), the watchdog.
*/
#include "../../rbx.h"
#include "../../cpu/nec/nec.h"
#include "../../sound/rbx_ym.h"
#include "../../video/decoband.h"
#include "../../video/gfxlayout.h"
#include "../machine/sched.h"
#include "../machine/necidle.h"
#include "../machine/seibusnd.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define VIS_W 256
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 2049
#define PAL_BLACK 2048
#define CPU_CLOCK 8000000
#define Z80_CLOCK 3579545

static int s_gh;                  // Gang Hunter (the trackball ports)
static uint8_t *s_main, *s_sub;
static uint32_t s_main_size, s_sub_size;
static const uint8_t *s_bgmap1, *s_bgmap2;
static rbx_gfx s_tx, s_spr, s_t1, s_t2, s_t3;
static int s_port_p, s_port_dsw;
static nec_t s_cpu[2];
static uint8_t *s_pri;

typedef struct {
	uint8_t main_lo[0x4000];       // 0x00000-0x03fff: RAM, the sprites at 0x3800
	uint8_t shared[0x1000];
	uint8_t textram[0x1000];       // (0x800 used: write only)
	uint8_t palette[0x1000];       // (write only)
	uint8_t scroll[0x1000];        // (0x100 used)
	uint8_t sub_lo[0x4000];        // 0x00000-0x03fff: RAM, the top playfield at 0x3800
	uint8_t pal_written[2048];     // (MAME's palette: its initial colors until written)
	uint8_t tilebank, vector[2], pad;
	int32_t line;                  // (the scanline timers' next line)
	uint8_t state_end;
} board_t;
static board_t *s_bp;   // (calloc at the init: ADR-040)
#define b (*s_bp)

typedef struct {
	uint8_t textram[0x800], video_data[0x800], spriteram[0x800];
	uint16_t s3x, s3y, s1x, s1y, s2x, s2y;
	uint8_t ctrl, tilebank;
	uint32_t pal[2048];
} snap_t;
static snap_t *s_snapp;   // (calloc at the init: ADR-040)
#define s_snap (*s_snapp)
static int s_pal_dirty;

static inline uint16_t rd16(const uint8_t *m, uint32_t a) { return (uint16_t)(m[a & ~1u] | m[(a & ~1u) + 1] << 8); }

// ---- the main CPU ----
static uint8_t main_r8(uint32_t a)
{
	a &= 0xfffff;
	if (a >= 0x6000 && a < 0x6010)
		return (a & 1) ? 0 : seibusnd_main_r((int)((a - 0x6000) >> 1));
	if (a >= 0xa000 && a < 0xa004) {
		const uint32_t v = rbx_port_read(a < 0xa002 ? s_port_p : s_port_dsw);
		return (uint8_t)((a & 1) ? v >> 8 : v);
	}
	return 0;   // (the trackballs too: their counts still at 0)
}

static uint16_t main_r16(uint32_t a) { return (uint16_t)(main_r8(a) | main_r8(a + 1) << 8); }

static void main_w8(uint32_t a, uint8_t v)
{
	a &= 0xfffff;
	if (a >= 0x6000 && a < 0x6010) {
		if (!(a & 1))
			seibusnd_main_w((int)((a - 0x6000) >> 1), v);
	} else if (a >= 0x8000 && a < 0x8800) {
		b.textram[a - 0x8000] = v;
	} else if (a >= 0xc000 && a < 0xd000) {
		b.palette[a - 0xc000] = v;
		b.pal_written[(a - 0xc000) >> 1] = 1;
	}
}

static void main_w16(uint32_t a, uint16_t v)
{
	main_w8(a, (uint8_t)v);
	main_w8(a + 1, (uint8_t)(v >> 8));
}

// ---- the sub CPU ----
static uint8_t sub_r8(uint32_t a) { (void)a; return 0; }
static uint16_t sub_r16(uint32_t a) { (void)a; return 0; }
static void sub_w8(uint32_t a, uint8_t v)
{
	if ((a & 0xfffff) == 0x8000)   // bank_w (the low byte)
		b.tilebank = v & 1;
}
static void sub_w16(uint32_t a, uint16_t v) { sub_w8(a, (uint8_t)v); }

static int vector0(void) { return b.vector[0]; }
static int vector1(void) { return b.vector[1]; }

// ---- the drawing ----
static uint16_t sc(int hi, int lo, int hmask)
{
	const uint8_t h = b.scroll[hi * 2], l = b.scroll[lo * 2];
	return (uint16_t)(((h & hmask) << 4) + ((l & 0x7f) << 1) + ((l & 0x80) >> 7));
}

static void capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap.textram, b.textram, sizeof(s_snap.textram));
	memcpy(s_snap.video_data, b.sub_lo + 0x3800, sizeof(s_snap.video_data));
	memcpy(s_snap.spriteram, b.main_lo + 0x3800, sizeof(s_snap.spriteram));
	s_snap.s3y = sc(0x01, 0x02, 0xf0);
	s_snap.s3x = sc(0x09, 0x0a, 0xf0);
	s_snap.s1y = sc(0x11, 0x12, 0x10);
	s_snap.s1x = sc(0x19, 0x1a, 0x10);
	s_snap.s2y = sc(0x21, 0x22, 0xf0);
	s_snap.s2x = sc(0x29, 0x2a, 0xf0);
	s_snap.ctrl = b.scroll[0x34 * 2];
	s_snap.tilebank = b.tilebank;
	for (int i = 0; i < 2048; i++) {   // xBGR_444 (MAME's initial "rainbow" until written)
		const uint16_t w = rd16(b.palette, (uint32_t)i * 2);
		s_snap.pal[i] = !b.pal_written[i] ? (i & 1 ? 0xff0000u : 0) | (i & 2 ? 0xff00u : 0) | (i & 4 ? 0xffu : 0)
		                                  : (uint32_t)(w & 0xf) * 0x110000u | (uint32_t)((w >> 4) & 0xf) * 0x1100u |
		                                        (uint32_t)((w >> 8) & 0xf) * 0x11u;
	}
	s_pal_dirty = 1;
	rbx_video_frame_done();
}

// a ROM-mapped playfield's line (128 x 256 tiles, MAME's bg_scan)
static void pf_rom_line(const uint8_t *map, const rbx_gfx *g, uint16_t cbase, int sx, int sy, int y, int trans,
                        uint16_t *d, uint8_t *pri, uint8_t pv)
{
	const int ty = (y + sy) & 4095;
	for (int x = 0; x < VIS_W;) {
		const int tx = (x + sx) & 2047;
		const int col = tx >> 4, row = ty >> 4;
		const uint32_t idx = (uint32_t)((col & 0xf) | ((row & 0xf) << 4) | ((col & 0x70) << 4) | ((row & 0xf0) << 7));
		const uint16_t code = (uint16_t)(map[idx * 2] << 8 | map[idx * 2 + 1]);   // (ROM_REGION16_BE)
		const uint8_t *rowp = g->pix + (size_t)(rbx_wrap((code & 0x7ff), g->count)) * 256 + (size_t)(ty & 15) * 16;
		const uint16_t base = (uint16_t)(cbase + (code >> 12) * 16);
		for (int fx = tx & 15; fx < 16 && x < VIS_W; fx++, x++) {
			const uint8_t c = rowp[fx];
			if (trans && c == 15)
				continue;
			d[x] = (uint16_t)(base + c);
			pri[x] |= pv;
		}
	}
}

static void draw_band(int band)
{
	const int r0 = VIS_H * band / 8, r1 = VIS_H * (band + 1) / 8;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, ctrl = s_snap.ctrl;
	for (int r = r0; r < r1; r++) {
		const int y = r + VIS_Y0;
		uint16_t *d = bm + r * pitch;
		uint8_t *pri = s_pri + r * VIS_W;
		for (int x = 0; x < VIS_W; x++)
			d[x] = PAL_BLACK;
		memset(pri, 0, VIS_W);
		if (!(ctrl & 1))
			pf_rom_line(s_bgmap1, &s_t3, 0, s_snap.s3x, s_snap.s3y, y, 0, d, pri, 1);
		if (!(ctrl & 2)) {   // the top playfield (TILEMAP_SCAN_COLS 32 x 32)
			const int ty = (y + s_snap.s1y) & 511;
			for (int x = 0; x < VIS_W;) {
				const int tx = (x + s_snap.s1x) & 511;
				const uint16_t data = rd16(s_snap.video_data, (uint32_t)(((tx >> 4) * 32 + (ty >> 4)) * 2));
				const uint32_t code = (data & 0xfff) + s_snap.tilebank * 0x1000u;
				const uint8_t *row = s_t1.pix + (size_t)(rbx_wrap(code, s_t1.count)) * 256 + (size_t)(ty & 15) * 16;
				const uint16_t base = (uint16_t)(1024 + (data >> 12) * 16);
				for (int fx = tx & 15; fx < 16 && x < VIS_W; fx++, x++)
					if (row[fx] != 15) {
						d[x] = (uint16_t)(base + row[fx]);
						pri[x] |= 2;
					}
			}
		}
		if (!(ctrl & 4))
			pf_rom_line(s_bgmap2, &s_t2, 256, s_snap.s2x, s_snap.s2y, y, 1, d, pri, 4);
	}
	if (!(ctrl & 0x10))
		for (int offs = 0; offs < 0x400; offs += 4) {
			const uint16_t w0 = rd16(s_snap.spriteram, (uint32_t)offs * 2), w1 = rd16(s_snap.spriteram, (uint32_t)(offs + 1) * 2),
			               w2 = rd16(s_snap.spriteram, (uint32_t)(offs + 2) * 2), w3 = rd16(s_snap.spriteram, (uint32_t)(offs + 3) * 2);
			if ((w3 & 0xff00) != 0xf00)
				continue;
			const uint32_t pm = ((w2 & 0xc000) == 0x4000 ? 0xf0u : (w2 & 0xc000) == 0 ? 0xfcu : 0) | 0x80000000u;
			const int fx = (w0 & 0x2000) != 0, fy = !(w0 & 0x4000);
			const int y = w0 & 0xff;
			int x = w2 & 0xff;
			if (w2 & 0x100)
				x = 0 - (0xff - x);
			const uint16_t base = (uint16_t)(768 + ((w1 >> 12) & 0xf) * 16);
			const uint8_t *src = s_spr.pix + (size_t)(rbx_wrap((w1 & 0xfff), s_spr.count)) * 256;
			for (int j = 0; j < 16; j++) {
				const int r = y + j - VIS_Y0;
				if (r < r0 || r >= r1)
					continue;
				const uint8_t *row = src + (fy ? 15 - j : j) * 16;
				uint16_t *d = bm + r * pitch;
				uint8_t *pri = s_pri + r * VIS_W;
				for (int i = 0; i < 16; i++) {
					const int px = x + i;
					if ((unsigned)px >= VIS_W)
						continue;
					const uint8_t c = row[fx ? 15 - i : i];
					if (c == 15)
						continue;
					if (!((1u << (pri[px] & 0x1f)) & pm))
						d[px] = (uint16_t)(base + c);
					pri[px] = 31;
				}
			}
		}
	if (!(ctrl & 8))
		for (int r = r0; r < r1; r++) {
			const int y = r + VIS_Y0;
			uint16_t *d = bm + r * pitch;
			for (int x = 0; x < VIS_W; x++) {
				const uint16_t data = rd16(s_snap.textram, (uint32_t)(((y >> 3) * 32 + (x >> 3)) * 2));
				const uint32_t tile = (data & 0xff) | ((data >> 6) & 0x300);
				const uint8_t c = s_tx.pix[(size_t)(rbx_wrap(tile, s_tx.count)) * 64 + (size_t)(y & 7) * 8 + (x & 7)];
				if (c != 15)
					d[x] = (uint16_t)(512 + ((data >> 8) & 0xf) * 16 + c);
			}
		}
}

static decoband_t s_bands = { 0, 0, 0, 0, 8, draw_band };

static void da_render(void)
{
	if (s_pal_dirty) {
		memcpy(rbx_video.palette, s_snap.pal, sizeof(s_snap.pal));
		rbx_video.palette[PAL_BLACK] = 0;
		s_pal_dirty = 0;
	}
	decoband_run(&s_bands);
}

static int da_render_help(int helper) { return decoband_help(&s_bands, helper); }

// ---- the scheduler's devices and timers ----
static int64_t s_run0[2];
static necidle_t s_ni[2];
static int s_ab[2];
static int v30_run(int i, int n)
{
	s_run0[i] = s_cpu[i].total;
	s_ab[i] = 0;
	if (necidle_probe(&s_ni[i], n, s_run0[i], &s_ab[i]))
		return (int)(s_cpu[i].total - s_run0[i]);
	nec_run_until(&s_cpu[i], s_run0[i] + n);
	return (int)(s_cpu[i].total - s_run0[i]);
}
static int run0(int n) { return v30_run(0, n); }
static int run1(int n) { return v30_run(1, n); }
static int64_t done0(void) { return nec_now(&s_cpu[0]) - s_run0[0]; }
static int64_t done1(void) { return nec_now(&s_cpu[1]) - s_run0[1]; }
static void abort0(void) { s_cpu[0].stop = 1; s_ab[0] = 1; }
static void abort1(void) { s_cpu[1].stop = 1; s_ab[1] = 1; }

enum { T_VBLANK = 1, T_VBLANK_END, T_SCANLINE0, T_SOUND, T_LINE };
static int64_t s_frame_as, s_scan_as;
static int64_t s_vbl_start;   // (the frame's vblank start: the scanline timers' base, MAME time_until_pos)
#define VBLANK_AS 2500000000000000LL   // (2500 us)

// the scanline timers' line l (0-255): (l + 16) % 256 lines after the vblank's start
static int64_t line_time(int l) { return s_vbl_start + (int64_t)((l + 16) & 255) * s_scan_as; }

static void fire(int id, int32_t param)
{
	if (seibusnd_fire(id, param))
		return;
	switch (id) {
	case T_VBLANK:
		capture();
		s_vbl_start = sched_now();
		sched_timer_set(T_VBLANK, sched_now() + s_frame_as, 0);
		sched_timer_set(T_VBLANK_END, sched_now() + VBLANK_AS, 0);
		break;
	case T_SCANLINE0:
		sched_timer_set(T_SCANLINE0, sched_now() + s_frame_as, 0);
		break;
	case T_SOUND:
		sched_timer_set(T_SOUND, sched_now() + SCHED_AS_PER_SEC / 50, 0);
		break;
	case T_LINE: {   // both scanline timers: line 240 the vblank-out IRQ, line 0 the vblank-in one (HOLD_LINE)
		const int l = param;
		if (l == 240 || l == 0) {
			b.vector[0] = b.vector[1] = (uint8_t)((l == 240 ? 0xc4 : 0xc8) / 4);
			nec_set_input(&s_cpu[0], NEC_LINE_IRQ, 1);
			nec_set_input(&s_cpu[1], NEC_LINE_IRQ, 1);
		}
		const int next = (l + 1) & 255;
		int64_t t = line_time(next);
		if (t <= sched_now())
			t += s_frame_as;
		sched_timer_set(T_LINE, t, next);
		break;
	}
	default: break;
	}
}

static void da_frame(void)
{
	rbx_audio.frame_mhz = 60000;
	sched_run(T_VBLANK, fire);
	seibusnd_frame_end(s_frame_as);
	sched_rebase(s_frame_as);
	s_vbl_start -= s_frame_as;
}

static void da_reset(void)
{
	rbx_video_wait_idle();
	memset(&b, 0, offsetof(__typeof__(b), state_end));
	for (int i = 0; i < 2; i++) {
		nec_reset(&s_cpu[i]);
		s_cpu[i].total = 0;
	}
	g_sched.base = 0;
	for (int i = 0; i < g_sched.nex; i++) {
		g_sched.ex[i].local = 0;
		g_sched.ex[i].suspended = 0;
	}
	for (int i = 0; i < SCHED_TIMERS; i++)
		g_sched.tm[i].id = -1;
	s_vbl_start = 0;
	sched_timer_set(T_VBLANK, s_frame_as, 0);
	sched_timer_set(T_SCANLINE0, 16 * s_scan_as, 0);
	sched_timer_set(T_SOUND, SCHED_AS_PER_SEC / 50, 0);
	sched_timer_set(T_LINE, line_time(0), 0);
	seibusnd_reset();
}

static void da_exit(void)
{
	seibusnd_exit();
	rbx_gfx_free(&s_tx);
	rbx_gfx_free(&s_spr);
	rbx_gfx_free(&s_t1);
	rbx_gfx_free(&s_t2);
	rbx_gfx_free(&s_t3);
	free(s_pri);
	s_pri = NULL;
	free(s_bp);
	s_bp = NULL;
	free(s_snapp);
	s_snapp = NULL;
}

// MAME's charlayout (RGN_FRAC(1,2)) and spritelayout
static const uint32_t ch_x[8] = { 3, 2, 1, 0, 11, 10, 9, 8 };
static const uint32_t ch_y[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
static const uint32_t sp_planes[4] = { 8, 12, 0, 4 };
static const uint32_t sp_x[16] = { 3, 2, 1, 0, 19, 18, 17, 16, 515, 514, 513, 512, 531, 530, 529, 528 };
static const uint32_t sp_y[16] = { 0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448, 480 };

static int da_init(const rbx_game *g)
{
	s_bp = calloc(1, sizeof(board_t));
	s_snapp = calloc(1, sizeof(snap_t));
	if (!s_bp || !s_snapp) {
		da_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_gh = !strcmp(g->machine, "ghunter");
	uint32_t snd_size, txs, sps, t1s, t2s, t3s, m1s, m2s, a1s = 0, a2s = 0;
	s_main = rbx_region("maincpu", &s_main_size);
	s_sub = rbx_region("sub", &s_sub_size);
	uint8_t *snd = rbx_region("audiocpu", &snd_size);
	const uint8_t *tx = rbx_region("chars", &txs), *sp = rbx_region("sprites", &sps), *t1 = rbx_region("tiles1", &t1s),
	              *t2 = rbx_region("tiles2", &t2s), *t3 = rbx_region("tiles3", &t3s), *a1 = rbx_region("adpcm1", &a1s),
	              *a2 = rbx_region("adpcm2", &a2s);
	s_bgmap1 = rbx_region("bgmap1", &m1s);
	s_bgmap2 = rbx_region("bgmap2", &m2s);
	if (!s_main || !s_sub || !snd || !tx || !sp || !t1 || !t2 || !t3 || !s_bgmap1 || !s_bgmap2 || !a1 || !a2 ||
	    s_main_size < 0x100000 || s_sub_size < 0x100000 || m1s < 0x10000 || m2s < 0x10000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	const uint32_t frac = txs / 2 * 8;
	const uint32_t ch_planes[4] = { frac, frac + 4, 0, 4 };
	const rbx_gfx_layout chl = { 8, 8, 4, ch_planes, ch_x, ch_y, 128 };
	const rbx_gfx_layout spl = { 16, 16, 4, sp_planes, sp_x, sp_y, 1024 };
	s_pri = (uint8_t *)calloc(1, VIS_W * VIS_H);
	if (rbx_gfx_decode(&s_tx, &chl, tx, txs / 2) || rbx_gfx_decode(&s_spr, &spl, sp, sps) ||
	    rbx_gfx_decode(&s_t1, &spl, t1, t1s) || rbx_gfx_decode(&s_t2, &spl, t2, t2s) ||
	    rbx_gfx_decode(&s_t3, &spl, t3, t3s) || !s_pri || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		da_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_port_p = rbx_port_find("P1_P2");
	s_port_dsw = rbx_port_find("DSW");
	s_frame_as = (int64_t)(1e18 / 60.0);   // (HZ_TO_ATTOSECONDS(60))
	s_scan_as = s_frame_as / 256;
	sched_init(SCHED_AS_PER_SEC / 60);   // (no maximum quantum set: MAME's 60 Hz default)
	(void)sched_add_exec(CPU_CLOCK, run0, done0, abort0);
	(void)sched_add_exec(CPU_CLOCK, run1, done1, abort1);
	seibusnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.rom = snd;
	sc.rom_size = snd_size;
	sc.encrypted = 1;
	sc.cabal = 1;   // (the SEI80BU below 0x2000 only, the ADPCM interfaces)
	sc.ym2 = 1;
	sc.ym_type = RBX_YM2203;
	sc.ym_clock = Z80_CLOCK;
	sc.ym_gain = 38;   // (0.15 every output)
	sc.z80_clock = Z80_CLOCK;
	sc.adpcm[0] = a1;
	sc.adpcm[1] = a2;
	sc.adpcm_size[0] = a1s;
	sc.adpcm_size[1] = a2s;
	sc.adpcm_gain = 102;   // (0.40)
	sc.coin_port = rbx_port_find("COIN");
	memset(&b, 0, sizeof(b));
	if (seibusnd_init(&sc)) {
		da_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	nec_init(&s_cpu[0], NEC_V30, NULL);
	nec_init(&s_cpu[1], NEC_V30, NULL);
	s_cpu[0].irq_ack = vector0;
	s_cpu[1].irq_ack = vector1;
	nec_t *m = &s_cpu[0], *su = &s_cpu[1];
	nec_map(m, 0x00000, 0x04000, b.main_lo, b.main_lo);
	nec_map(m, 0x04000, 0x05000, b.shared, b.shared);
	nec_map(m, 0x0e000, 0x0f000, b.scroll, b.scroll);
	nec_map(m, 0xc0000, 0x100000, s_main + 0xc0000, NULL);
	m->read8 = main_r8;
	m->read16 = main_r16;
	m->write8 = main_w8;
	m->write16 = main_w16;
	nec_map(su, 0x00000, 0x04000, b.sub_lo, b.sub_lo);
	nec_map(su, 0x04000, 0x05000, b.shared, b.shared);
	nec_map(su, 0xe0000, 0x100000, s_sub + 0xe0000, NULL);
	su->read8 = sub_r8;
	su->read16 = sub_r16;
	su->write8 = sub_w8;
	su->write16 = sub_w16;
	da_reset();
	necidle_init(&s_ni[0], &s_cpu[0]);
	necidle_init(&s_ni[1], &s_cpu[1]);
	return 0;
}

static int da_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "deadang", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= rbx_state_io(ctx, "deadang_cpu0", &s_cpu[0], NEC_STATE_SIZE);
	rc |= rbx_state_io(ctx, "deadang_cpu1", &s_cpu[1], NEC_STATE_SIZE);
	rc |= rbx_state_io(ctx, "deadang_vbl", &s_vbl_start, sizeof(s_vbl_start));
	rc |= seibusnd_state(ctx, load);
	rc |= sched_state(ctx, load);
	if (load && !rc) {
		nec_after_load(&s_cpu[0]);
		nec_after_load(&s_cpu[1]);
	}
	return rc ? -1 : 0;
}

static int da_save(void *ctx) { return da_state(ctx, 0); }
static int da_load(void *ctx) { return da_state(ctx, 1); }
static int da_supported(const char *machine) { return !strcmp(machine, "deadang") || !strcmp(machine, "ghunter"); }

const rbx_driver rbx_drv_deadang = { "Seibu Dead Angle", da_init, da_exit, da_reset, da_frame, da_save, da_load,
                                     da_supported, da_render, da_render_help };
