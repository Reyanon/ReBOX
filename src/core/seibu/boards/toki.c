/*  TAD / Seibu Toki: a 68000, two 16x16 tilemaps and an 8x8 text layer, 16x16 sprites (buffered), the Seibu Sound
    System (SEI80BU-encrypted Z80, YM3812, OKI MSM6295).
    Ported from MAME 0.289 src/mame/seibu/toki.cpp (BSD-3-Clause, copyright-holders: Jarek Parchanski).
    This file is part of ReBOX. GPL-2.0-or-later.

    The CPUs interleaved by the Seibu scheduler (sched.h) as the other Seibu Sound System boards. Not done: the
    bootlegs (tokib / jujub with their MSM5205 sound board, jujuba's own Z80 decryption), the flip screen, the
    per-line X scroll (the picture is drawn at vblank).
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../machine/sched.h"
#include "../machine/seibusnd.h"
#include "../video/seibu_draw.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define VIS_W 256
#define VIS_H 224
#define VIS_Y0 16
#define NPENS 1024

static uint8_t *s_rom;
static uint32_t s_rom_size;
static rbx_gfx s_tx, s_spr, s_bg, s_fg;
static int s_port_dsw, s_port_inputs, s_port_system;

typedef struct {
	uint8_t ram[0x10000];          // 0x60000-0x6ffff: work RAM, sprites 0xd800, palette 0xe000, bg1 0xe800, bg2 0xf000, text 0xf800
	uint16_t scroll[0x30];         // 0xa0000-0xa005f
	uint16_t spritebuf[0x400];     // (the buffered sprite RAM)
	uint8_t state_end;
} board_t;
static board_t *s_bp;   // (calloc at the init: ADR-040)
#define b (*s_bp)

typedef struct {
	uint16_t bg1[0x400], bg2[0x400], tx[0x400], spr[0x400], scroll[0x30];
	uint32_t pal[NPENS];
} snap_t;
static snap_t *s_snapp;   // (calloc at the init: ADR-040)
#define s_snap (*s_snapp)

#define RAM16(off) ((const uint16_t *)(b.ram + (off)))

// ---- the main CPU's devices ----
static uint16_t r16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x80000 && a < 0x8000e)
		return seibusnd_main_r((int)((a - 0x80000) >> 1));
	if (a == 0xc0000)
		return (uint16_t)rbx_port_read(s_port_dsw);
	if (a == 0xc0002)
		return (uint16_t)rbx_port_read(s_port_inputs);
	if (a == 0xc0004)
		return (uint16_t)rbx_port_read(s_port_system);
	if (a >= 0xa0000 && a < 0xa0060)
		return b.scroll[(a - 0xa0000) >> 1];
	return 0xffff;
}

static uint8_t r8(uint32_t a)
{
	const uint16_t v = r16(a & ~1u);
	return (uint8_t)((a & 1) ? v : v >> 8);
}

static void w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a >= 0xa0000 && a < 0xa0060) {   // control_w (COMBINE_DATA)
		uint16_t *p = &b.scroll[(a - 0xa0000) >> 1];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
		return;
	}
	if (a >= 0x80000 && a < 0x8000e) {   // (umask 0x00ff)
		if (mask & 0x00ff)
			seibusnd_main_w((int)((a - 0x80000) >> 1), (uint8_t)v);
		return;
	}
}

static void w16(uint32_t a, uint16_t v) { w16m(a, v, 0xffff); }
static void w8(uint32_t a, uint8_t v)
{
	if (a & 1)
		w16m(a & ~1u, v, 0x00ff);
	else
		w16m(a, (uint16_t)(v << 8), 0xff00);
}

// ---- the drawing ----
static void capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap.bg1, b.ram + 0xe800, sizeof(s_snap.bg1));
	memcpy(s_snap.bg2, b.ram + 0xf000, sizeof(s_snap.bg2));
	memcpy(s_snap.tx, b.ram + 0xf800, sizeof(s_snap.tx));
	memcpy(s_snap.spr, b.spritebuf, sizeof(s_snap.spr));
	memcpy(s_snap.scroll, b.scroll, sizeof(s_snap.scroll));
	const uint16_t *pal = RAM16(0xe000);
	for (int i = 0; i < NPENS; i++) {   // xBGR_444
		const uint16_t w = pal[i];
		s_snap.pal[i] = (uint32_t)(w & 0xf) * 0x110000u | (uint32_t)((w >> 4) & 0xf) * 0x1100u |
		                (uint32_t)((w >> 8) & 0xf) * 0x11u;
	}
	rbx_video_frame_done();
}

// a 32x32 tilemap of 16x16 (TILEMAP_SCAN_ROWS) line, transparent pen 15 unless opaque
static void tile16_line(const uint16_t *ram, const rbx_gfx *g, uint16_t cbase, int sx, int sy, int y, int opaque, uint16_t *d)
{
	const int ty = (y + sy) & 511;
	for (int x = 0; x < VIS_W;) {
		const int tx = (x + sx) & 511;
		const uint16_t data = ram[(ty >> 4) * 32 + (tx >> 4)];
		const uint8_t *row = sd_elem(g, data & 0xfff) + (ty & 15) * 16;
		const uint16_t base = (uint16_t)(cbase + (data >> 12) * 16);
		for (int fx = tx & 15; fx < 16 && x < VIS_W; fx++, x++) {
			const uint8_t c = row[fx];
			if (opaque || c != 15)
				d[x] = (uint16_t)(base + c);
		}
	}
}

static void tk_render(void)
{
	memcpy(rbx_video.palette, s_snap.pal, sizeof(s_snap.pal));
	const uint16_t *sr = s_snap.scroll;
	const int bgx = ((sr[0x06] & 0x7f) << 1) | ((sr[0x06] & 0x80) >> 7) | ((sr[0x05] & 0x10) << 4);
	const int bgy = ((sr[0x0d] & 0x10) << 4) + ((sr[0x0e] & 0x7f) << 1) + ((sr[0x0e] & 0x80) >> 7);
	const int fgx = ((sr[0x16] & 0x7f) << 1) | ((sr[0x16] & 0x80) >> 7) | ((sr[0x15] & 0x10) << 4);
	const int fgy = ((sr[0x1d] & 0x10) << 4) + ((sr[0x1e] & 0x7f) << 1) + ((sr[0x1e] & 0x80) >> 7);
	const int bg_first = (sr[0x28] & 0x100) != 0;
	for (int r = 0; r < VIS_H; r++) {
		const int y = r + VIS_Y0;
		uint16_t *d = rbx_video.bitmap + r * rbx_video.pitch;
		if (bg_first) {
			tile16_line(s_snap.bg1, &s_bg, 32 * 16, bgx, bgy, y, 1, d);
			tile16_line(s_snap.bg2, &s_fg, 48 * 16, fgx, fgy, y, 0, d);
		} else {
			tile16_line(s_snap.bg2, &s_fg, 48 * 16, fgx, fgy, y, 1, d);
			tile16_line(s_snap.bg1, &s_bg, 32 * 16, bgx, bgy, y, 0, d);
		}
	}
	const sd_target t = { rbx_video.bitmap, rbx_video.pitch, 0, VIS_Y0, VIS_W, VIS_H, NULL };
	for (int offs = 0x400 - 4; offs >= 0; offs -= 4) {   // the sprites, the last entry first
		const uint16_t *w = &s_snap.spr[offs];
		if (w[2] == 0xf000 || w[0] == 0xffff)
			continue;
		int x = (w[2] + (w[0] & 0xf0)) & 0x1ff;
		if (x > 256)
			x -= 512;
		int y = (w[3] + ((w[0] & 0xf) << 4)) & 0x1ff;
		if (y > 256)
			y -= 512;
		const uint32_t tile = (uint32_t)(w[1] & 0xfff) + ((w[2] & 0x8000) >> 3);
		sd_transpen(&t, &s_spr, tile, (uint32_t)(w[1] >> 12) * 16, (w[0] & 0x100) != 0, 0, x, y, 15);
	}
	for (int r = 0; r < VIS_H; r++) {   // the text (pen 15 transparent)
		const int y = r + VIS_Y0;
		uint16_t *d = rbx_video.bitmap + r * rbx_video.pitch;
		for (int x = 0; x < VIS_W; x++) {
			const uint16_t data = s_snap.tx[(y >> 3) * 32 + (x >> 3)];
			const uint8_t c = sd_elem(&s_tx, data & 0xfff)[(y & 7) * 8 + (x & 7)];
			if (c != 15)
				d[x] = (uint16_t)(16 * 16 + (data >> 12) * 16 + c);
		}
	}
}

// ---- the scheduler: the 68000, MAME's timers ----
static int m68k_run(int n) { return m68k_execute(n); }
static int64_t m68k_done(void) { return m68k_cycles_run(); }
static void m68k_abort(void) { m68k_end_timeslice(); }

enum { T_VBLANK = 1, T_SOUND };
static int64_t s_frame_as;

static void fire(int id, int32_t param)
{
	if (seibusnd_fire(id, param))
		return;
	switch (id) {
	case T_VBLANK:   // the sprite buffer, the screen update, the 68000's IRQ 1 (HOLD_LINE)
		memcpy(b.spritebuf, b.ram + 0xd800, sizeof(b.spritebuf));
		capture();
		m68k_bus_hold_irq(1);
		sched_timer_set(T_VBLANK, sched_now() + s_frame_as, 0);
		break;
	case T_SOUND:   // (MAME's sound manager update, 50 Hz)
		sched_timer_set(T_SOUND, sched_now() + SCHED_AS_PER_SEC / 50, 0);
		break;
	default: break;
	}
}

#define FRAME_MHZ 59630   // 6 MHz / (390 x 258)

static void tk_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	sched_run(T_VBLANK, fire);
	seibusnd_frame_end(s_frame_as);
	sched_rebase(s_frame_as);
}

static void tk_reset(void)
{
	rbx_video_wait_idle();
	memset(&b, 0, offsetof(__typeof__(b), state_end));
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	g_sched.base = 0;
	for (int i = 0; i < g_sched.nex; i++) {
		g_sched.ex[i].local = 0;
		g_sched.ex[i].suspended = 0;
	}
	for (int i = 0; i < SCHED_TIMERS; i++)
		g_sched.tm[i].id = -1;
	sched_timer_set(T_VBLANK, s_frame_as, 0);
	sched_timer_set(T_SOUND, SCHED_AS_PER_SEC / 50, 0);
	seibusnd_reset();
}

static void tk_exit(void)
{
	rbx_video_wait_idle();
	seibusnd_exit();
	rbx_gfx_free(&s_tx);
	rbx_gfx_free(&s_spr);
	rbx_gfx_free(&s_bg);
	rbx_gfx_free(&s_fg);
	free(s_bp);
	s_bp = NULL;
	free(s_snapp);
	s_snapp = NULL;
}

// toki_charlayout, toki_tilelayout
static const uint32_t ch_x[8] = { 3, 2, 1, 0, 11, 10, 9, 8 };
static const uint32_t ch_y[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
static const uint32_t tl_planes[4] = { 8, 12, 0, 4 };
static const uint32_t tl_x[16] = { 3, 2, 1, 0, 19, 18, 17, 16, 515, 514, 513, 512, 531, 530, 529, 528 };
static const uint32_t tl_y[16] = { 0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448, 480 };

static int tk_init(const rbx_game *g)
{
	s_bp = calloc(1, sizeof(board_t));
	s_snapp = calloc(1, sizeof(snap_t));
	if (!s_bp || !s_snapp) {
		tk_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t snd_size, txs, sps, bgs, fgs, okis = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *snd = rbx_region("audiocpu", &snd_size);
	const uint8_t *tx = rbx_region("chars", &txs), *sp = rbx_region("sprites", &sps), *bg = rbx_region("bgtiles", &bgs),
	              *fg = rbx_region("fgtiles", &fgs);
	uint8_t *oki = rbx_region("oki", &okis);
	if (!s_rom || !snd || !tx || !sp || !bg || !fg || !oki || okis < 0x20000) {
		tk_exit();
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{   // init_toki: the OKI ROM's address lines 13 and 15 swapped
		uint8_t *buf = (uint8_t *)malloc(0x20000);
		if (!buf) {
			tk_exit();
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
		memcpy(buf, oki, 0x20000);
		for (uint32_t i = 0; i < 0x20000; i++)
			oki[i] = buf[(i & ~0xa000u) | ((i >> 2) & 0x2000u) | ((i << 2) & 0x8000u)];
		free(buf);
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	const uint32_t half = txs / 2 * 8;
	const uint32_t ch_planes[4] = { half, half + 4, 0, 4 };
	const rbx_gfx_layout chl = { 8, 8, 4, ch_planes, ch_x, ch_y, 128 };
	const rbx_gfx_layout tll = { 16, 16, 4, tl_planes, tl_x, tl_y, 1024 };
	if (rbx_gfx_decode(&s_tx, &chl, tx, txs / 2) || rbx_gfx_decode(&s_spr, &tll, sp, sps) ||
	    rbx_gfx_decode(&s_bg, &tll, bg, bgs) || rbx_gfx_decode(&s_fg, &tll, fg, fgs) ||
	    rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		tk_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_port_dsw = rbx_port_find("DSW");
	s_port_inputs = rbx_port_find("INPUTS");
	s_port_system = rbx_port_find("SYSTEM");
	s_frame_as = (int64_t)(1e18 / 59.6303);
	sched_init(SCHED_AS_PER_SEC / 60);
	(void)sched_add_exec(10000000, m68k_run, m68k_done, m68k_abort);
	seibusnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.rom = snd;
	sc.rom_size = snd_size;
	sc.encrypted = 1;
	sc.ym_type = RBX_YM3812;
	sc.ym_clock = 3579545;
	sc.ym_gain = 256;
	sc.z80_clock = 3579545;
	sc.oki_rom = oki;
	sc.oki_size = okis;
	sc.oki_clock = 1000000;
	sc.oki_pin7 = 1;
	sc.oki_gain = 256;
	sc.coin_port = rbx_port_find("COIN");
	if (seibusnd_init(&sc)) {
		tk_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	m68k_bus_reset_map();
	m68k_bus.read8 = r8;
	m68k_bus.read16 = r16;
	m68k_bus.write8 = w8;
	m68k_bus.write16 = w16;
	m68k_bus_map(0x000000, 0x05ffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x060000, 0x06ffff, b.ram, sizeof(b.ram), 1, 1);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);
	tk_reset();
	return 0;
}

static int tk_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (load)
		rbx_video_wait_idle();
	else
		m68k_get_context(cpu);
	uint32_t irq = m68k_bus_irq_pending();
	int rc = rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", &irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "toki", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= seibusnd_state(ctx, load);
	rc |= sched_state(ctx, load);
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int tk_save(void *ctx) { return tk_state(ctx, 0); }
static int tk_load(void *ctx) { return tk_state(ctx, 1); }
static int tk_supported(const char *machine) { return !strcmp(machine, "toki"); }

const rbx_driver rbx_drv_toki = { "Seibu Toki", tk_init, tk_exit, tk_reset, tk_frame, tk_save, tk_load, tk_supported,
                                  tk_render, NULL };
