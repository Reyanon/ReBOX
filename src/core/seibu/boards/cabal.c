/*  Seibu / TAD Cabal: a 68000, a 16x16 background and an 8x8 2 bpp text layer, 16x16 sprites, the Seibu Sound
    System in Cabal's variant (SEI80BU-encrypted Z80, YM2151, two Seibu ADPCM interfaces with MSM5205s); the
    trackball sets with their uPD4701s.
    Ported from MAME 0.289 src/mame/seibu/cabal.cpp (BSD-3-Clause, copyright-holders: Carlos A. Lozano Baides).
    This file is part of ReBOX. GPL-2.0-or-later.

    The CPUs interleaved as MAME's scheduler does it (sched.h), the sound command's 50 us spin of the 68000 included.
    Not done: the bootleg with its own sound hardware (cabalbl), the flip screen, the trackballs' movement (still).
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../sound/rbx_ym.h"
#include "../../video/decoband.h"
#include "../../video/gfxlayout.h"
#include "../machine/sched.h"
#include "../machine/seibusnd.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define VIS_W 256
#define VIS_H 224
#define VIS_Y0 16
#define NPENS 1024

static int s_track;               // the trackball sets (cabalt)
static uint8_t *s_rom;
static uint32_t s_rom_size;
static rbx_gfx s_tx, s_bg, s_spr;
static int s_port_dsw, s_port_in2, s_port_inputs;

typedef struct {
	uint8_t ram[0x10000];          // 0x40000-0x4ffff (the sprites at 0x43800)
	uint8_t text[0x1000];          // 0x60000 (0x800 used)
	uint8_t bg[0x1000];            // 0x80000 (0x400 used)
	uint8_t pal[0x1000];           // 0xe0000 (0x800 used)
	uint8_t pal_written[NPENS];    // (MAME's palette: its initial colors until written)
	uint8_t state_end;
} board_t;
static board_t *s_bp;   // (calloc at the init: ADR-040)
#define b (*s_bp)

typedef struct {
	uint16_t tx[0x400], bg[0x100], spr[0x400];
	uint32_t pal[NPENS];
} snap_t;
static snap_t *s_snapp;   // (calloc at the init: ADR-040)
#define s_snap (*s_snapp)
static int s_pal_dirty;

#define RAM16(p, off) ((const uint16_t *)((p) + (off)))

// ---- the main CPU's devices ----
static uint16_t r16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0xe8000 && a < 0xe800e)
		return seibusnd_main_r((int)((a - 0xe8000) >> 1));
	if (a == 0xa0000)
		return (uint16_t)rbx_port_read(s_port_dsw);
	if (a >= 0xa0008 && a < 0xa0010)   // (the trackballs: uPD4701 read_xy, the counts still at 0)
		return s_track ? 0 : (a == 0xa0008 ? (uint16_t)rbx_port_read(s_port_in2) : 0);
	if (a == 0xa0010)
		return (uint16_t)rbx_port_read(s_port_inputs);
	return 0;
}

static uint8_t r8(uint32_t a)
{
	const uint16_t v = r16(a & ~1u);
	return (uint8_t)((a & 1) ? v : v >> 8);
}

enum { T_VBLANK = 1, T_VBLANK_END, T_SCANLINE0, T_SOUND, T_SPIN };

static void pal_w16(uint32_t a, uint16_t v)
{
	*(uint16_t *)(b.pal + (a - 0xe0000)) = v;
	if (a < 0xe0800)
		b.pal_written[(a - 0xe0000) >> 1] = 1;
}

static void w8(uint32_t a, uint8_t v);

static void w16(uint32_t a, uint16_t v)
{
	a &= 0xffffff;
	if (a >= 0xe0000 && a < 0xe1000) {
		pal_w16(a, v);
		return;
	}
	if (a == 0xe8008) {   // sound_irq_trigger_word_w: the request, then the 68000 spins 50 us
		seibusnd_main_w(4, (uint8_t)v);
		sched_spin(0, T_SPIN, 50 * (SCHED_AS_PER_SEC / 1000000));
		return;
	}
	if (a >= 0xe8000 && a < 0xe800e) {
		seibusnd_main_w((int)((a - 0xe8000) >> 1), (uint8_t)v);
		return;
	}
}

static void w8(uint32_t a, uint8_t v)
{
	a &= 0xffffff;
	if (a >= 0xe0000 && a < 0xe1000) {
		const uint16_t w = *(const uint16_t *)(b.pal + ((a - 0xe0000) & ~1u));
		pal_w16(a & ~1u, (a & 1) ? (uint16_t)((w & 0xff00) | v) : (uint16_t)((w & 0x00ff) | v << 8));
		return;
	}
	if ((a & ~1u) == 0xe8008) {
		if (a & 1)
			seibusnd_main_w(4, v);
		sched_spin(0, T_SPIN, 50 * (SCHED_AS_PER_SEC / 1000000));
		return;
	}
	if (a >= 0xe8000 && a < 0xe800e && (a & 1))
		seibusnd_main_w((int)((a - 0xe8000) >> 1), v);
}

// ---- the drawing ----
static void capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap.tx, b.text, sizeof(s_snap.tx));
	memcpy(s_snap.bg, b.bg, sizeof(s_snap.bg));
	memcpy(s_snap.spr, b.ram + 0x3800, sizeof(s_snap.spr));
	const uint16_t *pal = RAM16(b.pal, 0);
	for (int i = 0; i < NPENS; i++) {   // xBGR_444 (MAME's initial "rainbow" until written)
		const uint16_t w = pal[i];
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
	for (int r = r0; r < r1; r++) {   // the background (opaque)
		const int y = r + VIS_Y0;
		uint16_t *d = bm + r * pitch;
		for (int x = 0; x < VIS_W; x++) {
			const uint16_t data = s_snap.bg[(y >> 4) * 16 + (x >> 4)];
			d[x] = (uint16_t)(512 + (data >> 12) * 16 +
			                  s_bg.pix[(size_t)((data & 0xfff) % s_bg.count) * 256 + (size_t)(y & 15) * 16 + (x & 15)]);
		}
	}
	// the sprites (transpen 15), the last in the RAM drawn first
	for (int offs = 0x400 - 4; offs >= 0; offs -= 4) {
		const uint16_t d0 = s_snap.spr[offs], d1 = s_snap.spr[offs + 1], d2 = s_snap.spr[offs + 2];
		if (!(d0 & 0x100))
			continue;
		const int sy = d0 & 0xff, flipx = (d2 & 0x400) != 0;
		int sx = d2 & 0x1ff;
		if (sx > 256)
			sx -= 512;
		const uint16_t base = (uint16_t)(256 + ((d2 & 0x7800) >> 11) * 16);
		const uint8_t *src = s_spr.pix + (size_t)((d1 & 0xfff) % s_spr.count) * 256;
		for (int j = 0; j < 16; j++) {
			const int r = sy + j - VIS_Y0;
			if (r < r0 || r >= r1)
				continue;
			uint16_t *d = bm + r * pitch;
			for (int i = 0; i < 16; i++) {
				const int px = sx + i;
				if ((unsigned)px >= VIS_W)
					continue;
				const uint8_t c = src[j * 16 + (flipx ? 15 - i : i)];
				if (c != 15)
					d[px] = (uint16_t)(base + c);
			}
		}
	}
	for (int r = r0; r < r1; r++) {   // the text (2 bpp, pen 3 transparent)
		const int y = r + VIS_Y0;
		uint16_t *d = bm + r * pitch;
		for (int x = 0; x < VIS_W; x++) {
			const uint16_t data = s_snap.tx[(y >> 3) * 32 + (x >> 3)];
			const uint8_t c = s_tx.pix[(size_t)((data & 0x3ff) % s_tx.count) * 64 + (size_t)(y & 7) * 8 + (x & 7)];
			if (c != 3)
				d[x] = (uint16_t)((data >> 10) * 4 + c);
		}
	}
}

static decoband_t s_bands = { 0, 0, 0, 0, 8, draw_band };

static void cb_render(void)
{
	if (s_pal_dirty) {
		memcpy(rbx_video.palette, s_snap.pal, sizeof(s_snap.pal));
		s_pal_dirty = 0;
	}
	decoband_run(&s_bands);
}

static int cb_render_help(int helper) { return decoband_help(&s_bands, helper); }

// ---- the scheduler: the 68000, MAME's timers ----
static int m68k_run(int n) { return m68k_execute(n); }
static int64_t m68k_done(void) { return m68k_cycles_run(); }
static void m68k_abort(void) { m68k_end_timeslice(); }

static int64_t s_frame_as, s_scan0_as;
#define VBLANK_AS 2500000000000000LL   // (2500 us)

static void fire(int id, int32_t param)
{
	if (seibusnd_fire(id, param))
		return;
	switch (id) {
	case T_VBLANK:   // the screen update, the 68000's IRQ 1 (HOLD_LINE), the timers again
		capture();
		m68k_bus_hold_irq(1);
		sched_timer_set(T_VBLANK, sched_now() + s_frame_as, 0);
		sched_timer_set(T_VBLANK_END, sched_now() + VBLANK_AS, 0);
		break;
	case T_SCANLINE0:
		sched_timer_set(T_SCANLINE0, sched_now() + s_frame_as, 0);
		break;
	case T_SOUND:   // (MAME's sound manager update, 50 Hz)
		sched_timer_set(T_SOUND, sched_now() + SCHED_AS_PER_SEC / 50, 0);
		break;
	case T_SPIN:   // (the spin's trigger)
		sched_resume(0);
		break;
	default: break;
	}
}

static void cb_frame(void)
{
	rbx_audio.frame_mhz = 59600;
	sched_run(T_VBLANK, fire);
	seibusnd_frame_end(s_frame_as);
	sched_rebase(s_frame_as);
}

static void cb_reset(void)
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
	sched_timer_set(T_SCANLINE0, s_scan0_as, 0);
	sched_timer_set(T_SOUND, SCHED_AS_PER_SEC / 50, 0);
	seibusnd_reset();
}

static void cb_exit(void)
{
	seibusnd_exit();
	rbx_gfx_free(&s_tx);
	rbx_gfx_free(&s_bg);
	rbx_gfx_free(&s_spr);
	free(s_bp);
	s_bp = NULL;
	free(s_snapp);
	s_snapp = NULL;
}

// MAME's text_layout, tile_layout, sprite_layout
static const uint32_t tx_planes[2] = { 0, 4 };
static const uint32_t tx_x[8] = { 3, 2, 1, 0, 11, 10, 9, 8 };
static const uint32_t tx_y[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
static const uint32_t tl_planes[4] = { 8, 12, 0, 4 };
static const uint32_t tl_x[16] = { 3, 2, 1, 0, 19, 18, 17, 16, 515, 514, 513, 512, 531, 530, 529, 528 };
static const uint32_t tl_y[16] = { 0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448, 480 };
static const uint32_t sp_x[16] = { 3, 2, 1, 0, 19, 18, 17, 16, 35, 34, 33, 32, 51, 50, 49, 48 };
static const uint32_t sp_y[16] = { 960, 896, 832, 768, 704, 640, 576, 512, 448, 384, 320, 256, 192, 128, 64, 0 };

static int cb_init(const rbx_game *g)
{
	s_bp = calloc(1, sizeof(board_t));
	s_snapp = calloc(1, sizeof(snap_t));
	if (!s_bp || !s_snapp) {
		cb_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_track = !strcmp(g->machine, "cabalt");
	uint32_t snd_size, txs, bgs, sps, a1s = 0, a2s = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *snd = rbx_region("audiocpu", &snd_size);
	const uint8_t *tx = rbx_region("chars", &txs), *bg = rbx_region("tiles", &bgs), *sp = rbx_region("sprites", &sps),
	              *a1 = rbx_region("adpcm1", &a1s), *a2 = rbx_region("adpcm2", &a2s);
	if (!s_rom || !snd || !tx || !bg || !sp || !a1 || !a2 || snd_size < 0x10000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	const rbx_gfx_layout txl = { 8, 8, 2, tx_planes, tx_x, tx_y, 128 };
	const rbx_gfx_layout tll = { 16, 16, 4, tl_planes, tl_x, tl_y, 1024 };
	const rbx_gfx_layout spl = { 16, 16, 4, tl_planes, sp_x, sp_y, 1024 };
	if (rbx_gfx_decode(&s_tx, &txl, tx, txs) || rbx_gfx_decode(&s_bg, &tll, bg, bgs) ||
	    rbx_gfx_decode(&s_spr, &spl, sp, sps) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		cb_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_port_dsw = rbx_port_find("DSW");
	s_port_in2 = rbx_port_find("IN2");
	s_port_inputs = rbx_port_find("INPUTS");
	s_frame_as = (int64_t)(1e18 / 59.60);   // (HZ_TO_ATTOSECONDS(59.60))
	s_scan0_as = (s_frame_as / 256) * (256 - (VIS_Y0 + VIS_H));
	sched_init(SCHED_AS_PER_SEC / 60);   // (no maximum quantum set: MAME's 60 Hz default)
	(void)sched_add_exec(10000000, m68k_run, m68k_done, m68k_abort);
	seibusnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.rom = snd;
	sc.rom_size = 0x10000;
	sc.cabal = 1;
	sc.predecrypted = !strcmp(g->machine, "cabalbl2");
	sc.encrypted = !sc.predecrypted;
	sc.coin_port = rbx_port_find("COIN");
	sc.ym_type = RBX_YM2151;
	sc.ym_clock = 3579545;
	sc.ym_gain = 205;   // (0.80)
	sc.z80_clock = 3579545;
	sc.adpcm[0] = a1;
	sc.adpcm[1] = a2;
	sc.adpcm_size[0] = a1s;
	sc.adpcm_size[1] = a2s;
	sc.adpcm_gain = 102;   // (0.40)
	if (seibusnd_init(&sc)) {
		cb_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	m68k_bus_reset_map();
	m68k_bus.read8 = r8;
	m68k_bus.read16 = r16;
	m68k_bus.write8 = w8;
	m68k_bus.write16 = w16;
	m68k_bus_map(0x000000, 0x03ffff, s_rom, 0x40000, 1, 0);
	m68k_bus_map(0x040000, 0x04ffff, b.ram, sizeof(b.ram), 1, 1);
	m68k_bus_map(0x060000, 0x060fff, b.text, sizeof(b.text), 1, 1);
	m68k_bus_map(0x080000, 0x080fff, b.bg, sizeof(b.bg), 1, 1);
	m68k_bus_map(0x0e0000, 0x0e0fff, b.pal, sizeof(b.pal), 1, 0);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);
	cb_reset();
	return 0;
}

static int cb_state(void *ctx, int load)
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
	rc |= rbx_state_io(ctx, "cabal", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= seibusnd_state(ctx, load);
	rc |= sched_state(ctx, load);
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int cb_save(void *ctx) { return cb_state(ctx, 0); }
static int cb_load(void *ctx) { return cb_state(ctx, 1); }
static int cb_supported(const char *machine)
{
	return !strcmp(machine, "cabal") || !strcmp(machine, "cabalt") || !strcmp(machine, "cabalbl2");
}

const rbx_driver rbx_drv_cabal = { "Seibu Cabal", cb_init, cb_exit, cb_reset, cb_frame, cb_save, cb_load, cb_supported,
                                   cb_render, cb_render_help };
