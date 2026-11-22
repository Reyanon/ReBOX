/*  Seibu Dynamite Duke / The Double Dynamites: two NEC V30s (the master with the sprites, the text layer and the
    scroll registers; the slave with the two 16x16 tilemaps, the 6 bpp background's priority bit, and the palette) on
    a shared RAM, the Seibu Sound System (encrypted Z80, YM3812, OKI MSM6295).
    Ported from MAME 0.289 src/mame/seibu/dynduke.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail) and the
    buffered sprite RAM of src/devices/video/bufsprite.h (BSD-3-Clause, copyright-holders: Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.

    The CPUs interleaved as MAME's scheduler does it (sched.h: the 1/3600 s quantum; master, slave, Z80 in that
    order; the screen's and the sound manager's timers, the synchronizes). Not done: the flip screen, the coin counters.
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
#define Z80_CLOCK 3579545   // (14.318181 MHz / 4)

static int s_j;                   // The Double Dynamites (Japan): the master's other map
static uint8_t *s_main, *s_slave;
static uint32_t s_main_size, s_slave_size;
static rbx_gfx s_tx, s_bg, s_fg, s_spr;
static int s_port_p, s_port_dsw;
static nec_t s_cpu[2];
static uint8_t *s_pri;

typedef struct {
	uint8_t ram_main[0x7000];
	uint8_t spriteram[0x1000];
	uint8_t scroll[0x1000];        // (0x100 used: a page)
	uint8_t shared[0x1000];
	uint8_t textram[0x1000];       // (0x800 used: a page)
	uint8_t ram_slave[0x6000];
	uint8_t bgfg[0x1000];          // background 0x000-0x7ff, foreground 0x800-0xfff
	uint8_t palette[0x1000];
	uint8_t spritebuf[0x1000];     // (the buffered sprite RAM)
	uint8_t pal_written[2048];     // (MAME's palette: its initial colors until written)
	uint16_t back_bank, fore_bank;
	uint8_t back_en, fore_en, txt_en, spr_en;
	uint8_t state_end;
} board_t;
static board_t *s_bp;   // (calloc at the init: ADR-040)
#define b (*s_bp)

typedef struct {
	uint8_t textram[0x800], bgfg[0x1000], spritebuf[0x1000];
	uint16_t bgx, bgy, fgx, fgy, back_bank, fore_bank;
	uint8_t back_en, fore_en, txt_en, spr_en;
	uint32_t pal[2048];
} snap_t;
static snap_t *s_snapp;   // (calloc at the init: ADR-040)
#define s_snap (*s_snapp)
static int s_pal_dirty;

static inline uint16_t rd16(const uint8_t *m, uint32_t a) { return (uint16_t)(m[a & ~1u] | m[(a & ~1u) + 1] << 8); }

// ---- the master ----
static void control_w(uint8_t v)
{
	b.back_en = !(v & 1);
	b.fore_en = !(v & 2);
	b.txt_en = !(v & 4);
	b.spr_en = !(v & 8);
}

enum { D_NONE, D_SOUND, D_INPUT, D_CONTROL };
static int dev_of(uint32_t a, uint32_t *off)
{
	const uint32_t snd = s_j ? 0x9000 : 0xd000, io = s_j ? 0xf000 : 0xb000;
	if (a >= snd && a < snd + 0xe) { *off = (a - snd) >> 1; return D_SOUND; }
	if (a >= io && a < io + 4) { *off = a - io; return D_INPUT; }
	if (a == io + 6) return D_CONTROL;
	return D_NONE;
}

static uint8_t main_r8(uint32_t a)
{
	uint32_t off = 0;
	switch (dev_of(a & 0xfffff, &off)) {
	case D_SOUND: return (a & 1) ? 0 : seibusnd_main_r((int)off);   // (umask 0x00ff)
	case D_INPUT: {
		const uint32_t v = rbx_port_read(off < 2 ? s_port_p : s_port_dsw);
		return (uint8_t)((a & 1) ? v >> 8 : v);
	}
	default: return 0;
	}
}

static uint16_t main_r16(uint32_t a) { return (uint16_t)(main_r8(a) | main_r8(a + 1) << 8); }

static void main_w8(uint32_t a, uint8_t v)
{
	uint32_t off = 0;
	switch (dev_of(a & 0xfffff, &off)) {
	case D_SOUND: if (!(a & 1)) seibusnd_main_w((int)off, v); break;
	case D_CONTROL: control_w(v); break;
	default: break;
	}
}

static void main_w16(uint32_t a, uint16_t v)
{
	main_w8(a, (uint8_t)v);
	main_w8(a + 1, (uint8_t)(v >> 8));
}

// ---- the slave: the palette's page (its writes noted), the gfx bank ----
static uint8_t slave_r8(uint32_t a) { (void)a; return 0; }
static uint16_t slave_r16(uint32_t a) { (void)a; return 0; }

static void slave_w8(uint32_t a, uint8_t v)
{
	a &= 0xfffff;
	if (a >= 0x7000 && a < 0x8000) {
		b.palette[a - 0x7000] = v;
		b.pal_written[(a - 0x7000) >> 1] = 1;
	} else if (a == 0xa000) {   // gfxbank_w (the low byte)
		b.back_bank = (v & 1) ? 0x1000 : 0;
		b.fore_bank = (v & 0x10) ? 0x1000 : 0;
	}
}

static void slave_w16(uint32_t a, uint16_t v)
{
	slave_w8(a, (uint8_t)v);
	slave_w8(a + 1, (uint8_t)(v >> 8));
}

static int vector_r(void) { return 0xc8 / 4; }

// ---- the drawing ----
static uint16_t scroll_of(int hi, int lo)
{
	const uint8_t h = b.scroll[hi * 2], l = b.scroll[lo * 2];   // (the words' low bytes)
	return (uint16_t)(((h & 0x30) << 4) + ((l & 0x7f) << 1) + ((l & 0x80) >> 7));
}

static void capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap.textram, b.textram, sizeof(s_snap.textram));
	memcpy(s_snap.bgfg, b.bgfg, sizeof(s_snap.bgfg));
	memcpy(s_snap.spritebuf, b.spritebuf, sizeof(s_snap.spritebuf));
	s_snap.bgy = scroll_of(0x01, 0x02);
	s_snap.bgx = scroll_of(0x09, 0x0a);
	s_snap.fgy = scroll_of(0x11, 0x12);
	s_snap.fgx = scroll_of(0x19, 0x1a);
	s_snap.back_bank = b.back_bank;
	s_snap.fore_bank = b.fore_bank;
	s_snap.back_en = b.back_en;
	s_snap.fore_en = b.fore_en;
	s_snap.txt_en = b.txt_en;
	s_snap.spr_en = b.spr_en;
	for (int i = 0; i < 2048; i++) {   // xBGR_444 (MAME's initial "rainbow" until written)
		const uint16_t w = rd16(b.palette, (uint32_t)i * 2);
		s_snap.pal[i] = !b.pal_written[i] ? (i & 1 ? 0xff0000u : 0) | (i & 2 ? 0xff00u : 0) | (i & 4 ? 0xffu : 0)
		                                  : (uint32_t)(w & 0xf) * 0x110000u | (uint32_t)((w >> 4) & 0xf) * 0x1100u |
		                                        (uint32_t)((w >> 8) & 0xf) * 0x11u;
	}
	s_pal_dirty = 1;
	rbx_video_frame_done();
}

static void draw_band(int band)
{
	const int r0 = VIS_H * band / 8, r1 = VIS_H * (band + 1) / 8;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int r = r0; r < r1; r++) {
		const int y = r + VIS_Y0;
		uint16_t *d = bm + r * pitch;
		uint8_t *pri = s_pri + r * VIS_W;
		memset(pri, 0, VIS_W);
		// the background twice (its pixels without, then with, the priority bit 0x20), as MAME's draw_background
		if (!s_snap.back_en) {
			for (int x = 0; x < VIS_W; x++)
				d[x] = PAL_BLACK;
		} else {
			// (every pixel is drawn by one of the passes: one pass in tile spans is the same)
			const int ty = (y + s_snap.bgy) & 0x1ff;
			for (int x = 0; x < VIS_W;) {
				const int tx = (x + s_snap.bgx) & 0x1ff;
				const uint16_t data = rd16(s_snap.bgfg, (uint32_t)(((tx >> 4) * 32 + (ty >> 4)) * 2));
				const uint32_t code = (data & 0xfff) + s_snap.back_bank;
				const uint8_t *row = s_bg.pix + (size_t)rbx_wrap(code, s_bg.count) * 256 + (size_t)(ty & 15) * 16;
				const uint16_t base = (uint16_t)((data >> 12) * 16);
				for (int fx = tx & 15; fx < 16 && x < VIS_W; fx++, x++) {
					const uint8_t p = row[fx];
					d[x] = (uint16_t)(base + (p & 0xf) + ((p & 0x10) ? 0x100 : 0));
					pri[x] = (p & 0x20) ? 2 : 1;
				}
			}
		}
		if (s_snap.fore_en) {
			const int ty = (y + s_snap.fgy) & 0x1ff;
			for (int x = 0; x < VIS_W;) {
				const int tx = (x + s_snap.fgx) & 0x1ff;
				const uint16_t data = rd16(s_snap.bgfg + 0x800, (uint32_t)(((tx >> 4) * 32 + (ty >> 4)) * 2));
				const uint32_t code = (data & 0xfff) + s_snap.fore_bank;
				const uint8_t *row = s_fg.pix + (size_t)rbx_wrap(code, s_fg.count) * 256 + (size_t)(ty & 15) * 16;
				const uint16_t base = (uint16_t)(0x200 + (data >> 12) * 16);
				for (int fx = tx & 15; fx < 16 && x < VIS_W; fx++, x++)
					if (row[fx] != 15) {
						d[x] = (uint16_t)(base + row[fx]);
						pri[x] |= 4;
					}
			}
		}
		if (s_snap.txt_en)
			for (int x = 0; x < VIS_W; x += 8) {
				const uint16_t data = rd16(s_snap.textram, (uint32_t)(((y >> 3) * 32 + (x >> 3)) * 2));
				const uint32_t tile = (data & 0xff) | ((data & 0xc000) >> 6);
				const uint8_t *row = s_tx.pix + (size_t)rbx_wrap(tile, s_tx.count) * 64 + (size_t)(y & 7) * 8;
				const uint16_t base = (uint16_t)(0x500 + ((data >> 8) & 0xf) * 16);
				for (int i = 0; i < 8 && x + i < VIS_W; i++)
					if (row[i] != 15) {
						d[x + i] = (uint16_t)(base + row[i]);
						pri[x + i] |= 8;
					}
			}
	}
	if (!s_snap.spr_en)
		return;
	static const uint32_t pmasks[4] = { 0xff00u | 0xf0f0u | 0xccccu, 0xff00u | 0xf0f0u | 0xccccu, 0xff00u | 0xf0f0u, 0xff00u };
	for (int offs = 0; offs < 0x800; offs += 4) {
		const uint16_t w0 = rd16(s_snap.spritebuf, (uint32_t)offs * 2), w1 = rd16(s_snap.spritebuf, (uint32_t)(offs + 1) * 2),
		               w2 = rd16(s_snap.spritebuf, (uint32_t)(offs + 2) * 2), w3 = rd16(s_snap.spritebuf, (uint32_t)(offs + 3) * 2);
		if ((w3 >> 8) != 0xf)
			continue;
		const uint32_t pmask = pmasks[(w2 >> 13) & 3] | 0x80000000u;
		const int fx = (w0 >> 13) & 1, fy = (w0 >> 14) & 1;
		const int y = w0 & 0xff, x = (w2 & 0x100) ? (int)(w2 & 0x1ff) - 0x200 : (int)(w2 & 0x1ff);
		const uint16_t base = (uint16_t)(0x300 + ((w0 >> 8) & 0x1f) * 16);
		const uint8_t *src = s_spr.pix + (size_t)((w1 & 0x3fff) % s_spr.count) * 256;
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
				if (!((1u << (pri[px] & 0x1f)) & pmask))
					d[px] = (uint16_t)(base + c);
				pri[px] = 31;
			}
		}
	}
}

static decoband_t s_bands = { 0, 0, 0, 0, 8, draw_band };

static void dd_render(void)
{
	if (s_pal_dirty) {
		memcpy(rbx_video.palette, s_snap.pal, sizeof(s_snap.pal));
		rbx_video.palette[PAL_BLACK] = 0;
		s_pal_dirty = 0;
	}
	decoband_run(&s_bands);
}

static int dd_render_help(int helper) { return decoband_help(&s_bands, helper); }

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

enum { T_VBLANK = 1, T_VBLANK_END, T_SCANLINE0, T_SOUND };
static int64_t s_frame_as, s_scan0_as;
#define VBLANK_AS 2500000000000000LL   // (2500 us)

static void fire(int id, int32_t param)
{
	if (seibusnd_fire(id, param))
		return;
	switch (id) {
	case T_VBLANK:   // the screen update, the sprite buffer, both CPUs' interrupt (HOLD_LINE), the timers again
		capture();
		memcpy(b.spritebuf, b.spriteram, sizeof(b.spritebuf));
		nec_set_input(&s_cpu[0], NEC_LINE_IRQ, 1);
		nec_set_input(&s_cpu[1], NEC_LINE_IRQ, 1);
		sched_timer_set(T_VBLANK, sched_now() + s_frame_as, 0);
		sched_timer_set(T_VBLANK_END, sched_now() + VBLANK_AS, 0);
		break;
	case T_SCANLINE0:
		sched_timer_set(T_SCANLINE0, sched_now() + s_frame_as, 0);
		break;
	case T_SOUND:   // (MAME's sound manager update, 50 Hz)
		sched_timer_set(T_SOUND, sched_now() + SCHED_AS_PER_SEC / 50, 0);
		break;
	default: break;
	}
}

static void dd_frame(void)
{
	rbx_audio.frame_mhz = 60000;
	sched_run(T_VBLANK, fire);
	seibusnd_frame_end(s_frame_as);
	sched_rebase(s_frame_as);
}

static void dd_reset(void)
{
	rbx_video_wait_idle();
	memset(&b, 0, offsetof(__typeof__(b), state_end));
	for (int i = 0; i < 2; i++) {
		nec_reset(&s_cpu[i]);
		s_cpu[i].total = 0;
	}
	g_sched.base = 0;
	for (int i = 0; i < g_sched.nex; i++)
		g_sched.ex[i].local = 0;
	for (int i = 0; i < SCHED_TIMERS; i++)
		g_sched.tm[i].id = -1;
	sched_timer_set(T_VBLANK, s_frame_as, 0);
	sched_timer_set(T_SCANLINE0, s_scan0_as, 0);
	sched_timer_set(T_SOUND, SCHED_AS_PER_SEC / 50, 0);
	seibusnd_reset();
}

static void dd_exit(void)
{
	seibusnd_exit();
	rbx_gfx_free(&s_tx);
	rbx_gfx_free(&s_bg);
	rbx_gfx_free(&s_fg);
	rbx_gfx_free(&s_spr);
	free(s_pri);
	s_pri = NULL;
	free(s_bp);
	s_bp = NULL;
	free(s_snapp);
	s_snapp = NULL;
}

// MAME's charlayout, bg_layout (RGN_FRAC(1,3), 6 bpp), fg_layout, spritelayout
static const uint32_t ch_planes[4] = { 4, 0, 0x10000 * 8 + 4, 0x10000 * 8 };
static const uint32_t ch_x[8] = { 0, 1, 2, 3, 8, 9, 10, 11 };
static const uint32_t ch_y[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
static const uint32_t bf_x[16] = { 0, 1, 2, 3, 8, 9, 10, 11, 256, 257, 258, 259, 264, 265, 266, 267 };
static const uint32_t bf_y[16] = { 0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240 };
static const uint32_t fg_planes[4] = { 0x80000 * 8 + 4, 0x80000 * 8, 4, 0 };
static const uint32_t sp_planes[4] = { 12, 8, 4, 0 };
static const uint32_t sp_x[16] = { 0, 1, 2, 3, 16, 17, 18, 19, 512, 513, 514, 515, 528, 529, 530, 531 };
static const uint32_t sp_y[16] = { 0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448, 480 };

static int dd_init(const rbx_game *g)
{
	s_bp = calloc(1, sizeof(board_t));
	s_snapp = calloc(1, sizeof(snap_t));
	if (!s_bp || !s_snapp) {
		dd_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_j = !strcmp(g->machine, "dbldyn");
	uint32_t snd_size, txs, bgs, fgs, sps, okis = 0;
	s_main = rbx_region("maincpu", &s_main_size);
	s_slave = rbx_region("slave", &s_slave_size);
	uint8_t *snd = rbx_region("audiocpu", &snd_size);
	const uint8_t *tx = rbx_region("chars", &txs), *bg = rbx_region("bgtiles", &bgs), *fg = rbx_region("fgtiles", &fgs),
	              *sp = rbx_region("sprites", &sps);
	uint8_t *oki = rbx_region("oki", &okis);
	if (!s_main || !s_slave || !snd || !tx || !bg || !fg || !sp || s_main_size < 0x100000 || s_slave_size < 0x100000 ||
	    txs < 0x20000 || fgs < 0x100000 || sps < 0x200000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (oki && okis) {   // init_dynduke: the OKI ROM's address lines 12-15 swapped
		uint8_t *buf = (uint8_t *)malloc(okis);
		if (!buf) {
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
		memcpy(buf, oki, okis);
		for (uint32_t i = 0; i < okis; i++) {
			const uint32_t sw = ((i >> 13) & 1) << 3 | ((i >> 14) & 1) << 2 | ((i >> 15) & 1) << 1 | ((i >> 12) & 1);
			oki[i] = buf[(i & 0xfff) | sw << 12 | (i & ~0xffffu)];
		}
		free(buf);
	}
	const uint32_t f3 = bgs / 3 * 8;
	const uint32_t bg_planes[6] = { 2 * f3 + 4, 2 * f3, f3 + 4, f3, 4, 0 };
	const rbx_gfx_layout chl = { 8, 8, 4, ch_planes, ch_x, ch_y, 128 };
	const rbx_gfx_layout bgl = { 16, 16, 6, bg_planes, bf_x, bf_y, 512 };
	const rbx_gfx_layout fgl = { 16, 16, 4, fg_planes, bf_x, bf_y, 512 };
	const rbx_gfx_layout spl = { 16, 16, 4, sp_planes, sp_x, sp_y, 1024 };
	s_pri = (uint8_t *)calloc(1, VIS_W * VIS_H);
	if (rbx_gfx_decode(&s_tx, &chl, tx, 1024 * 128 / 8) || rbx_gfx_decode(&s_bg, &bgl, bg, bgs / 3) ||
	    rbx_gfx_decode(&s_fg, &fgl, fg, 0x2000 * 512 / 8) || rbx_gfx_decode(&s_spr, &spl, sp, 0x4000 * 1024 / 8) ||
	    !s_pri || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		dd_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_port_p = rbx_port_find("P1_P2");
	s_port_dsw = rbx_port_find("DSW");
	s_frame_as = (int64_t)(1e18 / 60.0);   // (HZ_TO_ATTOSECONDS(60))
	s_scan0_as = (s_frame_as / 256) * (256 - (VIS_Y0 + VIS_H));
	sched_init(SCHED_AS_PER_SEC / 3600);   // (set_maximum_quantum)
	(void)sched_add_exec(CPU_CLOCK, run0, done0, abort0);
	(void)sched_add_exec(CPU_CLOCK, run1, done1, abort1);
	seibusnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.rom = snd;
	sc.rom_size = snd_size;
	sc.encrypted = 1;
	sc.ym_type = RBX_YM3812;
	sc.ym_clock = Z80_CLOCK;
	sc.ym_gain = 256;
	sc.z80_clock = Z80_CLOCK;
	sc.oki_rom = oki;
	sc.oki_size = okis;
	sc.oki_clock = 1000000;
	sc.oki_pin7 = 1;
	sc.oki_gain = 102;   // (0.40)
	sc.coin_port = rbx_port_find("COIN");
	memset(&b, 0, sizeof(b));
	if (seibusnd_init(&sc)) {
		dd_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	for (int i = 0; i < 2; i++) {
		nec_init(&s_cpu[i], NEC_V30, NULL);
		s_cpu[i].irq_ack = vector_r;
	}
	nec_t *m = &s_cpu[0], *sl = &s_cpu[1];
	nec_map(m, 0x00000, 0x07000, b.ram_main, b.ram_main);
	nec_map(m, 0x07000, 0x08000, b.spriteram, b.spriteram);
	if (s_j) {   // masterj_map
		nec_map(m, 0x08000, 0x09000, b.textram, b.textram);
		nec_map(m, 0x0c000, 0x0d000, b.scroll, b.scroll);
		nec_map(m, 0x0e000, 0x0f000, b.shared, b.shared);
	} else {
		nec_map(m, 0x08000, 0x09000, b.scroll, b.scroll);
		nec_map(m, 0x0a000, 0x0b000, b.shared, b.shared);
		nec_map(m, 0x0c000, 0x0d000, b.textram, b.textram);
	}
	nec_map(m, 0xa0000, 0x100000, s_main + 0xa0000, NULL);
	m->read8 = main_r8;
	m->read16 = main_r16;
	m->write8 = main_w8;
	m->write16 = main_w16;
	nec_map(sl, 0x00000, 0x06000, b.ram_slave, b.ram_slave);
	nec_map(sl, 0x06000, 0x07000, b.bgfg, b.bgfg);
	nec_map(sl, 0x07000, 0x08000, b.palette, NULL);
	nec_map(sl, 0x08000, 0x09000, b.shared, b.shared);
	nec_map(sl, 0xc0000, 0x100000, s_slave + 0xc0000, NULL);
	sl->read8 = slave_r8;
	sl->read16 = slave_r16;
	sl->write8 = slave_w8;
	sl->write16 = slave_w16;
	dd_reset();
	necidle_init(&s_ni[0], &s_cpu[0]);
	necidle_init(&s_ni[1], &s_cpu[1]);
	return 0;
}

static int dd_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "dynduke", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= rbx_state_io(ctx, "dynduke_cpu0", &s_cpu[0], NEC_STATE_SIZE);
	rc |= rbx_state_io(ctx, "dynduke_cpu1", &s_cpu[1], NEC_STATE_SIZE);
	rc |= seibusnd_state(ctx, load);
	rc |= sched_state(ctx, load);
	if (load && !rc) {
		nec_after_load(&s_cpu[0]);
		nec_after_load(&s_cpu[1]);
	}
	return rc ? -1 : 0;
}

static int dd_save(void *ctx) { return dd_state(ctx, 0); }
static int dd_load(void *ctx) { return dd_state(ctx, 1); }
static int dd_supported(const char *machine) { return !strcmp(machine, "dynduke") || !strcmp(machine, "dbldyn"); }

const rbx_driver rbx_drv_dynduke = { "Seibu Dynamite Duke", dd_init, dd_exit, dd_reset, dd_frame, dd_save, dd_load,
                                     dd_supported, dd_render, dd_render_help };
