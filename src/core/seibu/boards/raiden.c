/*  Seibu Raiden: two NEC V30s (the main one with the sprites, the text layer and the scroll registers; the sub one with
    the two 16x16 tilemaps and the palette) on a shared RAM, the Seibu Sound System (Z80, YM3812, OKI MSM6295); the
    newer "raidenb" boards with the Seibu CRTC.
    Ported from MAME 0.289 src/mame/seibu/raiden.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail) and
    src/mame/seibu/seibu_crtc.cpp (BSD-3-Clause, copyright-holders: Angelo Salese), the buffered sprite RAM of
    src/devices/video/bufsprite.h (BSD-3-Clause, copyright-holders: Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.

    The CPUs interleaved as MAME's scheduler does it (sched.h: the 1/12000 s quantum; main, sub, Z80 in that order;
    the screen's and the sound manager's timers, the synchronizes). Not done: the flip screen, the coin counters.
*/
#include "../../cheat_cpu.h"
#include "../../rbx.h"
#include "../../cpu/nec/nec.h"
#include "../../sound/rbx_ym.h"
#include "../../video/decoband.h"
#include "../machine/seibusnd.h"
#include "../machine/sched.h"
#include "../machine/necidle.h"

#include <stdlib.h>
#include <string.h>

#define VIS_W 256
#define VIS_Y0 16
#define VIS_H 224
#define FRAME_MHZ 59600
#define Z80_CLOCK 3579545            // 14.318181 MHz / 4
#define NPENS 2049
#define PAL_BLACK 2048

enum { M_RAIDEN, M_RAIDENU, M_RAIDENB };

static uint8_t *s_main, *s_sub;
static uint32_t s_main_size, s_sub_size;
static uint8_t *s_txpix, *s_bgpix, *s_fgpix, *s_sppix;   // a byte a pixel
static uint32_t s_txn, s_bgn, s_fgn, s_spn;
static int s_m;
static uint32_t s_cpu_clock;
static int s_port_p, s_port_dsw;
static nec_t s_cpu[2];

typedef struct {
	uint8_t ram_main[0x7000];
	uint8_t spriteram[0x1000];
	uint8_t shared[0x1000];
	uint8_t textram[0x1000];
	uint8_t ram_sub[0x6000];
	uint8_t bgfg[0x1000];         // bg 0x000-0x7ff, fg 0x800-0xfff
	uint8_t palette[0x1000];
	uint8_t spritebuf[0x1000];    // (the buffered sprite RAM)
	uint16_t scroll_ram[0x20];    // (raiden: the 8-bit scroll registers; raidenb: the CRTC's six words)
	uint16_t crtc[0x28];
	uint8_t bg_en, fg_en, tx_en, sp_en, flip, pad[3];
	int64_t frames;
	uint8_t state_end;
} board_t;
static board_t *s_bp;   // (calloc at the init: ADR-040)
#define b (*s_bp)

// ---- the drawing's copy ----
typedef struct {
	uint8_t textram[0x800], bgfg[0x1000], spritebuf[0x1000];
	uint16_t scroll[4];
	uint8_t bg_en, fg_en, tx_en, sp_en;
	uint32_t pal[2048];
} snap_t;
static snap_t *s_snapp;   // (calloc at the init: ADR-040)
#define s_snap (*s_snapp)
static int s_pal_dirty;
static uint8_t *s_pri;

static inline uint16_t rd16(const uint8_t *m, uint32_t a) { return (uint16_t)(m[a & ~1u] | m[(a & ~1u) + 1] << 8); }

// ---- the gfx (charlayout / tilelayout) ----
static uint8_t *decode(const uint8_t *rom, uint32_t size, int w, uint32_t *count)
{
	const uint32_t inc = w == 8 ? 256 : 1024;   // (bits an element)
	const uint32_t n = size * 8 / inc;
	uint8_t *p = (uint8_t *)malloc((size_t)n * w * w + 1);
	if (!p)
		return NULL;
	static const int planes[4] = { 12, 8, 4, 0 };
	for (uint32_t e = 0; e < n; e++)
		for (int y = 0; y < w; y++)
			for (int x = 0; x < w; x++) {
				const uint32_t xo = (uint32_t)((x & 3) + ((x >> 2) & 1) * 16 + ((x >> 3) & 1) * 512);
				const uint32_t base = e * inc + (uint32_t)y * 32 + xo;
				int v = 0;
				for (int pl = 0; pl < 4; pl++) {
					const uint32_t bitn = base + (uint32_t)planes[pl];
					v = v << 1 | ((rom[bitn >> 3] >> (7 - (bitn & 7))) & 1);
				}
				p[((size_t)e * w + (size_t)y) * w + (size_t)x] = (uint8_t)v;
			}
	*count = n;
	return p;
}

// ---- the main CPU ----
static uint8_t sound_r(uint32_t off) { return seibusnd_main_r((int)off); }

static void sound_w(uint32_t off, uint8_t v) { seibusnd_main_w((int)off, v); }

static void control_w(uint8_t v)
{
	if (s_m == M_RAIDENB) {
		b.tx_en = !(v & 8);
		b.flip = (v >> 1) & 1;
	} else {
		b.bg_en = !(v & 1);
		b.fg_en = !(v & 2);
		b.tx_en = !(v & 4);
		b.sp_en = !(v & 8);
		b.flip = (v >> 6) & 1;
	}
}

// the main map's devices by address: 0 none; returns the base
enum { D_NONE, D_SOUND, D_INPUT, D_CONTROL, D_SCROLL, D_TEXT, D_CRTC };
static int dev_of(uint32_t a, uint32_t *off)
{
	switch (s_m) {
	case M_RAIDEN:
		if (a >= 0xa000 && a < 0xa00e) { *off = (a - 0xa000) >> 1; return D_SOUND; }
		if (a >= 0xe000 && a < 0xe006) { *off = a - 0xe000; return D_INPUT; }
		if (a == 0xe006) return D_CONTROL;
		if (a >= 0xf000 && a < 0xf040) { *off = (a - 0xf000) >> 1; return D_SCROLL; }
		break;
	default:   // raidenu / raidenb
		if (s_m == M_RAIDENU && a >= 0x8000 && a < 0x8040) { *off = (a - 0x8000) >> 1; return D_SCROLL; }
		if (a >= 0xb000 && a < 0xb006) { *off = a - 0xb000; return D_INPUT; }
		if (a == 0xb006) return D_CONTROL;
		if (a >= 0xd000 && a < 0xd00e) { *off = (a - 0xd000) >> 1; return D_SOUND; }
		if (s_m == M_RAIDENB && a >= 0xd040 && a < 0xd090) { *off = (a - 0xd040) >> 1; return D_CRTC; }
		break;
	}
	if (a >= 0xc000 && a < 0xc800) { *off = a - 0xc000; return D_TEXT; }
	return D_NONE;
}

static uint8_t main_r8(uint32_t a)
{
	uint32_t off = 0;
	switch (dev_of(a & 0xfffff, &off)) {
	case D_SOUND: return (a & 1) ? 0 : sound_r(off);   // (umask 0x00ff)
	case D_INPUT: {
		const uint32_t v = rbx_port_read(off < 2 ? s_port_p : s_port_dsw);
		return (uint8_t)((a & 1) ? v >> 8 : v);
	}
	case D_CRTC: {
		const uint16_t v = off == 0x1a / 2 ? b.crtc[off] : 0;
		return (uint8_t)((a & 1) ? v >> 8 : v);
	}
	default: return 0;
	}
}

static uint16_t main_r16(uint32_t a) { return (uint16_t)(main_r8(a) | main_r8(a + 1) << 8); }

static void crtc_w(uint32_t off, uint16_t v, uint16_t mask)
{
	b.crtc[off] = (uint16_t)((b.crtc[off] & ~mask) | (v & mask));
	if (off == 0x1c / 2) {   // layer_en_w
		const uint16_t d = b.crtc[off];
		b.bg_en = !(d & 1);
		b.fg_en = !(d & 2);
		b.sp_en = !(d & 0x10);
	} else if (off >= 0x20 / 2 && off < 0x2c / 2) {   // layer_scroll_w
		uint16_t *p = &b.scroll_ram[off - 0x20 / 2];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
	}
}

static void main_w8(uint32_t a, uint8_t v)
{
	uint32_t off = 0;
	switch (dev_of(a & 0xfffff, &off)) {
	case D_SOUND: if (!(a & 1)) sound_w(off, v); break;
	case D_CONTROL: control_w(v); break;
	case D_SCROLL: {
		uint16_t *p = &b.scroll_ram[off];
		*p = (a & 1) ? (uint16_t)((*p & 0x00ff) | v << 8) : (uint16_t)((*p & 0xff00) | v);
		break;
	}
	case D_TEXT: b.textram[off] = v; break;
	case D_CRTC: crtc_w(off, (a & 1) ? (uint16_t)(v << 8) : v, (a & 1) ? 0xff00 : 0x00ff); break;
	default: break;
	}
}

static void main_w16(uint32_t a, uint16_t v)
{
	uint32_t off = 0;
	if (dev_of(a & 0xfffff, &off) == D_CRTC) {
		crtc_w(off, v, 0xffff);
		return;
	}
	main_w8(a, (uint8_t)v);
	main_w8(a + 1, (uint8_t)(v >> 8));
}

static uint8_t sub_r8(uint32_t a) { (void)a; return 0; }
static uint16_t sub_r16(uint32_t a) { (void)a; return 0; }
static void sub_w8(uint32_t a, uint8_t v) { (void)a; (void)v; }
static void sub_w16(uint32_t a, uint16_t v) { (void)a; (void)v; }

// vector_r: both CPUs' vector
static int vector_r(void) { return 0xc8 / 4; }

// ---- the drawing ----
static void capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap.textram, b.textram, sizeof(s_snap.textram));
	memcpy(s_snap.bgfg, b.bgfg, sizeof(s_snap.bgfg));
	memcpy(s_snap.spritebuf, b.spritebuf, sizeof(s_snap.spritebuf));
	if (s_m == M_RAIDENB) {
		for (int i = 0; i < 4; i++)
			s_snap.scroll[i] = b.scroll_ram[i];
	} else {   // (the 8-bit scroll registers: 4 bytes a value)
		const uint16_t *r = b.scroll_ram;
		s_snap.scroll[0] = (uint16_t)(((r[0x09] & 0xf0) << 4) | ((r[0x0a] & 0x7f) << 1) | ((r[0x0a] & 0x80) >> 7));
		s_snap.scroll[1] = (uint16_t)(((r[0x01] & 0xf0) << 4) | ((r[0x02] & 0x7f) << 1) | ((r[0x02] & 0x80) >> 7));
		s_snap.scroll[2] = (uint16_t)(((r[0x19] & 0xf0) << 4) | ((r[0x1a] & 0x7f) << 1) | ((r[0x1a] & 0x80) >> 7));
		s_snap.scroll[3] = (uint16_t)(((r[0x11] & 0xf0) << 4) | ((r[0x12] & 0x7f) << 1) | ((r[0x12] & 0x80) >> 7));
	}
	s_snap.bg_en = b.bg_en;
	s_snap.fg_en = b.fg_en;
	s_snap.tx_en = b.tx_en;
	s_snap.sp_en = b.sp_en;
	for (int i = 0; i < 2048; i++) {   // xBGR_444
		const uint16_t w = rd16(b.palette, (uint32_t)i * 2);
		const uint32_t r = (w & 0xf) * 0x11, g = ((w >> 4) & 0xf) * 0x11, bl = ((w >> 8) & 0xf) * 0x11;
		s_snap.pal[i] = r << 16 | g << 8 | bl;
	}
	s_pal_dirty = 1;
	rbx_video_frame_done();
}

// a 16x16 tilemap (32 x 32, TILEMAP_SCAN_COLS) line: x from 0 to VIS_W
static void draw_tile16(const uint8_t *ram, const uint8_t *gfx, uint32_t n, uint16_t cbase, int sx, int sy, int y, int trans,
                        uint16_t *d, uint8_t *pri, uint8_t pv)
{
	const int ty = (y + sy) & 511;
	for (int x = 0; x < VIS_W;) {
		const int tx = (x + sx) & 511;
		const uint16_t data = rd16(ram, (uint32_t)(((tx >> 4) * 32 + (ty >> 4)) * 2));
		const uint8_t *row = gfx + (size_t)rbx_wrap(data & 0xfff, n) * 256 + (size_t)(ty & 15) * 16;
		const uint16_t base = (uint16_t)(cbase + (data >> 12) * 16);
		for (int fx = tx & 15; fx < 16 && x < VIS_W; fx++, x++) {
			const uint8_t c = row[fx];
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
	const int pitch = rbx_video.pitch;
	for (int r = r0; r < r1; r++) {
		const int y = r + VIS_Y0;
		uint16_t *d = bm + r * pitch;
		uint8_t *pri = s_pri + r * VIS_W;
		for (int x = 0; x < VIS_W; x++)
			d[x] = PAL_BLACK;
		memset(pri, 0, VIS_W);
		if (s_snap.bg_en)
			draw_tile16(s_snap.bgfg, s_bgpix, s_bgn, 0, s_snap.scroll[0], s_snap.scroll[1], y, 0, d, pri, 1);
		if (s_snap.fg_en)
			draw_tile16(s_snap.bgfg + 0x800, s_fgpix, s_fgn, 256, s_snap.scroll[2], s_snap.scroll[3], y, 1, d, pri, 2);
		if (s_snap.tx_en) {
			for (int x = 0; x < VIS_W; x += 8) {
				const uint32_t idx = s_m == M_RAIDENB ? (uint32_t)((x >> 3) * 32 + (y >> 3)) : (uint32_t)((y >> 3) * 32 + (x >> 3));
				const uint16_t data = rd16(s_snap.textram, idx * 2);
				const uint32_t tile = (data & 0xff) | ((data >> 6) & 0x300);
				const uint8_t *row = s_txpix + (size_t)rbx_wrap(tile, s_txn) * 64 + (size_t)(y & 7) * 8;
				const uint16_t base = (uint16_t)(768 + ((data >> 8) & 0xf) * 16);
				for (int i = 0; i < 8 && x + i < VIS_W; i++)
					if (row[i] != 15) {
						d[x + i] = (uint16_t)(base + row[i]);
						pri[x + i] |= 4;
					}
			}
		}
	}
	if (!s_snap.sp_en)
		return;
	// the sprites (prio_transpen, pen 15; the first in the RAM on top)
	for (int offs = 0; offs < 0x800; offs += 4) {
		const uint16_t w0 = rd16(s_snap.spritebuf, (uint32_t)offs * 2), w1 = rd16(s_snap.spritebuf, (uint32_t)(offs + 1) * 2),
		               w2 = rd16(s_snap.spritebuf, (uint32_t)(offs + 2) * 2);
		if (!(w0 & 0x8000))
			continue;
		const int priority = (w2 >> 14) & 3;
		if (!priority)
			continue;
		const uint32_t pmask = (priority == 1 ? (0xf0f0u | 0xccccu) : 0xf0f0u) | 0x80000000u;
		const int flipy = (w0 >> 14) & 1, flipx = (w0 >> 13) & 1;
		const uint16_t base = (uint16_t)(512 + ((w0 >> 8) & 0xf) * 16);
		const uint8_t *src = s_sppix + (size_t)rbx_wrap(w1 & 0xfff, s_spn) * 256;
		int y = w0 & 0xff, x = w2 & 0x1ff;
		if (x & 0x100)
			x |= ~0x1ff;
		for (int j = 0; j < 16; j++) {
			const int r = y + j - VIS_Y0;
			if (r < r0 || r >= r1)
				continue;
			const uint8_t *row = src + (flipy ? 15 - j : j) * 16;
			uint16_t *d = bm + r * pitch;
			uint8_t *pri = s_pri + r * VIS_W;
			for (int i = 0; i < 16; i++) {
				const int px = x + i;
				if (px < 0 || px >= VIS_W)
					continue;
				const uint8_t c = row[flipx ? 15 - i : i];
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

static void rd_render(void)
{
	if (s_pal_dirty) {
		memcpy(rbx_video.palette, s_snap.pal, sizeof(s_snap.pal));
		rbx_video.palette[PAL_BLACK] = 0;
		s_pal_dirty = 0;
	}
	decoband_run(&s_bands);
}

static int rd_render_help(int helper) { return decoband_help(&s_bands, helper); }

// ---- the scheduler's devices and timers (MAME's: the screen's, the sound's) ----
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
static int64_t s_frame_as, s_scan_as;   // (MAME: the frame period, a line's)
#define VBLANK_AS 2500000000000000LL    // (2500 us)

static void fire(int id, int32_t param)
{
	if (seibusnd_fire(id, param))
		return;
	switch (id) {
	case T_VBLANK:   // vblank_begin: the screen update, the callbacks (sprite buffer, IRQ), the timers again
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
	case T_SOUND:   // (MAME's sound manager update, 50 Hz: a timer like the others)
		sched_timer_set(T_SOUND, sched_now() + SCHED_AS_PER_SEC / 50, 0);
		break;
	default: break;   // (T_VBLANK_END: nothing here)
	}
}

static void rd_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	sched_run(T_VBLANK, fire);
	seibusnd_frame_end(s_frame_as);
	sched_rebase(s_frame_as);
	b.frames++;
}

static void rd_reset(void)
{
	memset(&b, 0, offsetof(__typeof__(b), state_end));
	b.bg_en = b.fg_en = b.tx_en = b.sp_en = 1;
	for (int i = 0; i < 2; i++) {
		nec_reset(&s_cpu[i]);
		s_cpu[i].total = 0;
	}
	// (the scheduler from time 0: the devices, MAME's screen timers)
	g_sched.base = 0;
	for (int i = 0; i < g_sched.nex; i++)
		g_sched.ex[i].local = 0;
	for (int i = 0; i < SCHED_TIMERS; i++)
		g_sched.tm[i].id = -1;
	sched_timer_set(T_VBLANK, s_frame_as, 0);
	sched_timer_set(T_SCANLINE0, 16 * s_scan_as, 0);
	sched_timer_set(T_SOUND, SCHED_AS_PER_SEC / 50, 0);
	seibusnd_reset();
}

static void rd_exit(void);

static int rd_init(const rbx_game *g)
{
	rbx_cheat_prefer("nec");   // (the cheats' main CPU: another CPU is set up first)
	s_bp = calloc(1, sizeof(board_t));
	s_snapp = calloc(1, sizeof(snap_t));
	if (!s_bp || !s_snapp) {
		rd_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t sz;
	s_main = rbx_region("maincpu", &s_main_size);
	s_sub = rbx_region("sub", &s_sub_size);
	uint8_t *snd = rbx_region("audiocpu", &sz);
	const uint32_t snd_size = sz;
	uint32_t txs, bgs, fgs, sps, okis = 0;
	const uint8_t *tx = rbx_region("text", &txs), *bg = rbx_region("bgtiles", &bgs), *fg = rbx_region("fgtiles", &fgs),
	              *sp = rbx_region("sprites", &sps), *oki = rbx_region("oki", &okis);
	if (!s_main || !s_sub || !snd || !tx || !bg || !fg || !sp) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_m = !strcmp(g->machine, "raidenu") ? M_RAIDENU : !strcmp(g->machine, "raidenb") ? M_RAIDENB : M_RAIDEN;
	s_cpu_clock = !strcmp(g->machine, "raidenkb") ? 8000000 : 10000000;
	if (s_m == M_RAIDEN && s_main_size >= 0x60000 && s_sub_size >= 0x40000) {   // init_decryption
		static const uint16_t xm[16] = { 0x200e, 0x0006, 0x000a, 0x0002, 0x240e, 0x000e, 0x04c2, 0x00c2, 0x008c, 0x0004,
		                                 0x0088, 0x0000, 0x048c, 0x000c, 0x04c0, 0x00c0 };
		static const uint8_t bm_[16] = { 15, 14, 10, 12, 11, 13, 9, 8, 3, 2, 5, 4, 7, 1, 6, 0 };
		static const uint16_t xs[8] = { 0x0080, 0x0080, 0x0244, 0x0288, 0x0288, 0x0288, 0x1041, 0x1009 };
		static const uint8_t bs_[16] = { 15, 14, 13, 9, 11, 10, 12, 8, 2, 0, 5, 4, 7, 3, 1, 6 };
		for (int i = 0; i < 0x20000; i++) {
			uint8_t *p = s_main + 0x20000 + (uint32_t)i * 2;
			uint16_t d = (uint16_t)((p[0] | p[1] << 8) ^ xm[i & 15]), o = 0;
			for (int k = 0; k < 16; k++)
				o |= (uint16_t)(((d >> bm_[k]) & 1) << (15 - k));
			p[0] = (uint8_t)o;
			p[1] = (uint8_t)(o >> 8);
		}
		for (int i = 0; i < 0x20000; i++) {
			uint8_t *p = s_sub + (uint32_t)i * 2;
			uint16_t d = (uint16_t)((p[0] | p[1] << 8) ^ xs[i & 7]), o = 0;
			for (int k = 0; k < 16; k++)
				o |= (uint16_t)(((d >> bs_[k]) & 1) << (15 - k));
			p[0] = (uint8_t)o;
			p[1] = (uint8_t)(o >> 8);
		}
	}
	s_txpix = decode(tx, txs, 8, &s_txn);
	s_bgpix = decode(bg, bgs, 16, &s_bgn);
	s_fgpix = decode(fg, fgs, 16, &s_fgn);
	s_sppix = decode(sp, sps, 16, &s_spn);
	s_pri = (uint8_t *)calloc(1, VIS_W * VIS_H);
	if (!s_txpix || !s_bgpix || !s_fgpix || !s_sppix || !s_pri || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rd_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_port_p = rbx_port_find("P1_P2");
	s_port_dsw = rbx_port_find("DSW");
	seibusnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.rom = snd;
	sc.rom_size = snd_size;
	sc.encrypted = strcmp(g->machine, "raiden") && strcmp(g->machine, "raidenkb") && strcmp(g->machine, "raidenb");
	sc.ym_type = RBX_YM3812;
	sc.ym_clock = Z80_CLOCK;
	sc.ym_gain = 256;
	sc.z80_clock = Z80_CLOCK;
	sc.oki_rom = oki;
	sc.oki_size = okis;
	sc.oki_clock = 1000000;
	sc.oki_pin7 = 1;
	sc.oki_gain = 192;
	sc.coin_port = rbx_port_find("COIN");
	memset(&b, 0, sizeof(b));
	s_frame_as = (int64_t)(1e18 / 59.60);   // (HZ_TO_ATTOSECONDS(59.60))
	s_scan_as = s_frame_as / 256;
	sched_init(SCHED_AS_PER_SEC / 12000);   // (set_maximum_quantum)
	(void)sched_add_exec(s_cpu_clock, run0, done0, abort0);
	(void)sched_add_exec(s_cpu_clock, run1, done1, abort1);
	if (seibusnd_init(&sc)) {
		rd_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	// the V30s
	for (int i = 0; i < 2; i++) {
		nec_t *c = &s_cpu[i];
		nec_init(c, NEC_V30, NULL);
		c->irq_ack = vector_r;
	}
	nec_t *m = &s_cpu[0], *sc2 = &s_cpu[1];
	nec_map(m, 0x00000, 0x07000, b.ram_main, b.ram_main);
	nec_map(m, 0x07000, 0x08000, b.spriteram, b.spriteram);
	if (s_m == M_RAIDEN)
		nec_map(m, 0x08000, 0x09000, b.shared, b.shared);
	else
		nec_map(m, 0x0a000, 0x0b000, b.shared, b.shared);
	nec_map(m, 0xa0000, 0x100000, s_main, NULL);
	m->read8 = main_r8;
	m->read16 = main_r16;
	m->write8 = main_w8;
	m->write16 = main_w16;
	if (s_m == M_RAIDENU) {
		nec_map(sc2, 0x00000, 0x06000, b.ram_sub, b.ram_sub);
		nec_map(sc2, 0x06000, 0x07000, b.bgfg, b.bgfg);
		nec_map(sc2, 0x07000, 0x08000, b.palette, b.palette);
		nec_map(sc2, 0x08000, 0x09000, b.shared, b.shared);
	} else {
		nec_map(sc2, 0x00000, 0x02000, b.ram_sub, b.ram_sub);
		nec_map(sc2, 0x02000, 0x03000, b.bgfg, b.bgfg);
		nec_map(sc2, 0x03000, 0x04000, b.palette, b.palette);
		nec_map(sc2, 0x04000, 0x05000, b.shared, b.shared);
	}
	nec_map(sc2, 0xc0000, 0x100000, s_sub, NULL);
	sc2->read8 = sub_r8;
	sc2->read16 = sub_r16;
	sc2->write8 = sub_w8;
	sc2->write16 = sub_w16;
	necidle_init(&s_ni[0], &s_cpu[0]);
	necidle_init(&s_ni[1], &s_cpu[1]);
	rd_reset();
	return 0;
}

static void rd_exit(void)
{
	seibusnd_exit();
	free(s_txpix);
	free(s_bgpix);
	free(s_fgpix);
	free(s_sppix);
	free(s_pri);
	s_txpix = s_bgpix = s_fgpix = s_sppix = NULL;
	s_pri = NULL;
	free(s_bp);
	s_bp = NULL;
	free(s_snapp);
	s_snapp = NULL;
}

static int rd_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "raiden", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= rbx_state_io(ctx, "raiden_cpu0", &s_cpu[0], NEC_STATE_SIZE);
	rc |= rbx_state_io(ctx, "raiden_cpu1", &s_cpu[1], NEC_STATE_SIZE);
	rc |= seibusnd_state(ctx, load);
	rc |= sched_state(ctx, load);
	if (load && !rc) {
		nec_after_load(&s_cpu[0]);
		nec_after_load(&s_cpu[1]);
	}
	return rc ? -1 : 0;
}

static int rd_save(void *ctx) { return rd_state(ctx, 0); }
static int rd_load(void *ctx) { return rd_state(ctx, 1); }
static int rd_supported(const char *machine)
{
	return !strcmp(machine, "raidene") || !strcmp(machine, "raiden") || !strcmp(machine, "raidenkb") ||
	       !strcmp(machine, "raidenu") || !strcmp(machine, "raidenb");
}

const rbx_driver rbx_drv_raiden = { "Seibu Raiden", rd_init, rd_exit, rd_reset, rd_frame, rd_save, rd_load, rd_supported,
                                    rd_render, rd_render_help };
