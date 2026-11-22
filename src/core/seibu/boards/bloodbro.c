/*  Seibu Blood Bros. / Sky Smasher: a 68000, three tile layers through the Seibu CRTC, the SEI0210 sprites (their
    Blood Bros. format), the Seibu Sound System (Z80, YM3812, OKI MSM6295).
    Ported from MAME 0.289 src/mame/seibu/bloodbro.cpp (BSD-3-Clause, copyright-holders: Carlos A. Lozano Baides) and
    src/mame/seibu/seibu_crtc.cpp (BSD-3-Clause, copyright-holders: Angelo Salese).
    This file is part of ReBOX. GPL-2.0-or-later.

    The CPUs interleaved as MAME's scheduler does it (sched.h). Not done: West Story (the bootleg's own hardware),
    the coin counters.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../sound/rbx_ym.h"
#include "../../video/decoband.h"
#include "../../video/gfxlayout.h"
#include "../machine/sched.h"
#include "../machine/seibusnd.h"
#include "../video/sei0210.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define VIS_W 256
#define VIS_H 224
#define VIS_Y0 16
#define NPENS 2048

static int s_sky;                 // Sky Smasher (the CRTC's registers swapped, IRQ 2)
static uint8_t *s_rom;
static uint32_t s_rom_size;
static rbx_gfx s_tx, s_bg, s_spr;
static int s_port_dsw, s_port_in0, s_port_in1;
static uint8_t *s_pri;

typedef struct {
	uint8_t ram[0x10000];          // 0x80000-0x8ffff: work RAM, the sprites, the layers, the palette
	uint16_t crtc[0x28];
	uint16_t scroll[6];
	uint16_t layer_en, pad;
	uint8_t pal_written[NPENS];    // (MAME's palette: its initial colors until written)
	uint8_t state_end;
} board_t;
static board_t *s_bp;   // (calloc at the init: ADR-040)
#define b (*s_bp)

// (the layers in b.ram, host-order words)
#define RAM16(off) ((const uint16_t *)(b.ram + (off)))
enum { O_SPR = 0xb000, O_BG = 0xc000, O_FG = 0xd000, O_TX = 0xd800, O_PAL = 0xe800 };

typedef struct {
	uint16_t bg[0x200], fg[0x200], tx[0x400], spr[0x800];
	uint16_t scroll[6], layer_en;
	uint32_t pal[NPENS];
} snap_t;
static snap_t *s_snapp;   // (calloc at the init: ADR-040)
#define s_snap (*s_snapp)
static int s_pal_dirty;

// ---- the main CPU's devices ----
// (Sky Smasher's read_alt / write_alt: the word offset's bits 3 and 4 swapped)
static uint32_t crtc_off(uint32_t off) { return s_sky ? (off & ~0x18u) | ((off & 8) << 1) | ((off & 0x10) >> 1) : off; }

static void crtc_w(uint32_t off, uint16_t v)
{
	off = crtc_off(off);
	if (off >= 0x28)
		return;
	b.crtc[off] = v;
	if (off == 0x1c / 2)
		b.layer_en = v;
	else if (off >= 0x20 / 2 && off < 0x2c / 2)
		b.scroll[off - 0x20 / 2] = v;
}

static uint16_t r16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0xa0000 && a < 0xa000e)
		return seibusnd_main_r((int)((a - 0xa0000) >> 1));
	if (a >= 0xc0000 && a < 0xc0050) {
		const uint32_t off = crtc_off((a - 0xc0000) >> 1);
		return off < 0x28 ? b.crtc[off] : 0;
	}
	if (a == 0xe0000)
		return (uint16_t)rbx_port_read(s_port_dsw);
	if (a == 0xe0002)
		return (uint16_t)rbx_port_read(s_port_in0);
	if (a == 0xe0004)
		return (uint16_t)rbx_port_read(s_port_in1);
	return 0;
}

static uint8_t r8(uint32_t a)
{
	const uint16_t v = r16(a & ~1u);
	return (uint8_t)((a & 1) ? v : v >> 8);
}

// the pages of the palette (0x8e000-0x8ffff: with the text, the sprites): their writes here, the palette's noted
static void ram_w16(uint32_t a, uint16_t v)
{
	*(uint16_t *)(b.ram + (a - 0x80000)) = v;
	if (a >= 0x8e800 && a < 0x8f800)
		b.pal_written[(a - 0x8e800) >> 1] = 1;
}

static void w16(uint32_t a, uint16_t v)
{
	a &= 0xffffff;
	if (a >= 0x8e000 && a < 0x90000) {
		ram_w16(a, v);
		return;
	}
	if (a >= 0xa0000 && a < 0xa000e)
		seibusnd_main_w((int)((a - 0xa0000) >> 1), (uint8_t)v);
	else if (a >= 0xc0000 && a < 0xc0050)
		crtc_w((a - 0xc0000) >> 1, v);
}

// (a byte to a word device: MAME's other lane 0)
static void w8(uint32_t a, uint8_t v)
{
	a &= 0xffffff;
	if (a >= 0x8e000 && a < 0x90000) {
		const uint16_t w = *(const uint16_t *)(b.ram + ((a - 0x80000) & ~1u));
		ram_w16(a & ~1u, (a & 1) ? (uint16_t)((w & 0xff00) | v) : (uint16_t)((w & 0x00ff) | v << 8));
		return;
	}
	if (a >= 0xa0000 && a < 0xa000e) {
		if (a & 1)
			seibusnd_main_w((int)((a - 0xa0000) >> 1), v);
		return;
	}
	w16(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8));
}

// ---- the drawing ----
static uint32_t pri_cb(int pri, int ext)
{
	(void)ext;
	return pri ? 0x02 : 0;
}

static void capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap.bg, RAM16(O_BG), sizeof(s_snap.bg));
	memcpy(s_snap.fg, RAM16(O_FG), sizeof(s_snap.fg));
	memcpy(s_snap.tx, RAM16(O_TX), sizeof(s_snap.tx));
	memcpy(s_snap.spr, RAM16(O_SPR), sizeof(s_snap.spr));
	memcpy(s_snap.scroll, b.scroll, sizeof(s_snap.scroll));
	s_snap.layer_en = b.layer_en;
	const uint16_t *pal = RAM16(O_PAL);
	for (int i = 0; i < NPENS; i++) {   // xBGR_444
		if (!b.pal_written[i]) {   // (MAME's initial "rainbow": the index's bits 0-2 as R, G, B)
			s_snap.pal[i] = (i & 1 ? 0xff0000u : 0) | (i & 2 ? 0xff00u : 0) | (i & 4 ? 0xffu : 0);
			continue;
		}
		const uint16_t w = pal[i];
		s_snap.pal[i] = (uint32_t)(w & 0xf) * 0x110000u | (uint32_t)((w >> 4) & 0xf) * 0x1100u |
		                (uint32_t)((w >> 8) & 0xf) * 0x11u;
	}
	s_pal_dirty = 1;
	rbx_video_frame_done();
}

static void draw_band(int band)
{
	const int r0 = VIS_H * band / 8, r1 = VIS_H * (band + 1) / 8;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, en = ~s_snap.layer_en;
	for (int r = r0; r < r1; r++) {
		const int y = r + VIS_Y0;
		uint16_t *d = bm + r * pitch;
		uint8_t *pri = s_pri + r * VIS_W;
		memset(pri, 0, VIS_W);
		if (en & 1)
			seibu_tile16_line(s_snap.bg, 32, 16, 0, s_bg.pix, s_bg.count, 0, 0x40 * 16, s_snap.scroll[0],
			                  s_snap.scroll[1], y, 0, d, pri, 0, VIS_W);
		if (en & 2)
			seibu_tile16_line(s_snap.fg, 32, 16, 0, s_bg.pix, s_bg.count, 0x1000, 0x50 * 16, s_snap.scroll[2],
			                  s_snap.scroll[3], y, 1, d, pri, 1, VIS_W);
	}
	if (en & 0x10) {
		const sei0210_t sp = { 1, 0, 0, 0, pri_cb, s_spr.pix, s_spr.count, 0, NULL };
		sei0210_draw(&sp, s_snap.spr, 0x800, bm, pitch, s_pri, VIS_W, VIS_Y0, r0, r1);
	}
	if (en & 8)
		for (int r = r0; r < r1; r++)
			seibu_tile8_line(s_snap.tx, 32, 32, 0, s_tx.pix, s_tx.count, 0x70 * 16, 0, 0, r + VIS_Y0, 1,
			                 bm + r * pitch, s_pri + r * VIS_W, 0, VIS_W);
}

static decoband_t s_bands = { 0, 0, 0, 0, 8, draw_band };

static void bb_render(void)
{
	if (s_pal_dirty) {
		memcpy(rbx_video.palette, s_snap.pal, sizeof(s_snap.pal));
		s_pal_dirty = 0;
	}
	decoband_run(&s_bands);
}

static int bb_render_help(int helper) { return decoband_help(&s_bands, helper); }

// ---- the scheduler: the 68000, MAME's timers ----
static int m68k_run(int n) { return m68k_execute(n); }
static int64_t m68k_done(void) { return m68k_cycles_run(); }
static void m68k_abort(void) { m68k_end_timeslice(); }

enum { T_VBLANK = 1, T_VBLANK_END, T_SCANLINE0, T_SOUND };
static int64_t s_frame_as, s_scan0_as;
#define VBLANK_AS 2500000000000000LL   // (2500 us)

static void fire(int id, int32_t param)
{
	if (seibusnd_fire(id, param))
		return;
	switch (id) {
	case T_VBLANK:   // the screen update, the 68000's IRQ 4 (Sky Smasher's 2; HOLD_LINE), the timers again
		capture();
		m68k_bus_hold_irq(s_sky ? 2 : 4);
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

static void bb_frame(void)
{
	rbx_audio.frame_mhz = 59390;
	sched_run(T_VBLANK, fire);
	seibusnd_frame_end(s_frame_as);
	sched_rebase(s_frame_as);
}

static void bb_reset(void)
{
	rbx_video_wait_idle();
	memset(&b, 0, offsetof(__typeof__(b), state_end));
	m68k_bus_irq_reset();
	m68k_pulse_reset();
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

static void bb_exit(void)
{
	seibusnd_exit();
	rbx_gfx_free(&s_tx);
	rbx_gfx_free(&s_bg);
	rbx_gfx_free(&s_spr);
	free(s_pri);
	s_pri = NULL;
	free(s_bp);
	s_bp = NULL;
	free(s_snapp);
	s_snapp = NULL;
}

// MAME's textlayout (RGN_FRAC(1,2)) and spritelayout
static const uint32_t ch_x[8] = { 3, 2, 1, 0, 11, 10, 9, 8 };
static const uint32_t ch_y[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
static const uint32_t tl_planes[4] = { 8, 12, 0, 4 };
static const uint32_t tl_x[16] = { 3, 2, 1, 0, 19, 18, 17, 16, 515, 514, 513, 512, 531, 530, 529, 528 };
static const uint32_t tl_y[16] = { 0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448, 480 };

static int bb_init(const rbx_game *g)
{
	s_bp = calloc(1, sizeof(board_t));
	s_snapp = calloc(1, sizeof(snap_t));
	if (!s_bp || !s_snapp) {
		bb_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_sky = !strcmp(g->machine, "skysmash");
	uint32_t snd_size, txs, bgs, sps, okis = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *snd = rbx_region("audiocpu", &snd_size);
	const uint8_t *tx = rbx_region("txtiles", &txs), *bg = rbx_region("bgtiles", &bgs), *sp = rbx_region("sprites", &sps),
	              *oki = rbx_region("oki", &okis);
	if (!s_rom || !snd || !tx || !bg || !sp) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	const uint32_t frac = txs / 2 * 8;
	const uint32_t tx_planes[4] = { 0, 4, frac, frac + 4 };
	const rbx_gfx_layout chl = { 8, 8, 4, tx_planes, ch_x, ch_y, 128 };
	const rbx_gfx_layout tll = { 16, 16, 4, tl_planes, tl_x, tl_y, 1024 };
	s_pri = (uint8_t *)calloc(1, VIS_W * VIS_H);
	if (rbx_gfx_decode(&s_tx, &chl, tx, txs / 2) || rbx_gfx_decode(&s_bg, &tll, bg, bgs) ||
	    rbx_gfx_decode(&s_spr, &tll, sp, sps) || !s_pri || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		bb_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_port_dsw = rbx_port_find("DSW");
	s_port_in0 = rbx_port_find("IN0");
	s_port_in1 = rbx_port_find("IN1");
	s_frame_as = (int64_t)(1e18 / 59.39);   // (HZ_TO_ATTOSECONDS(59.39))
	// (the scanline-0 timer: vpos 0 is (height - visible bottom - 1) lines after the vblank's start)
	s_scan0_as = (s_frame_as / 256) * (256 - (VIS_Y0 + VIS_H));
	sched_init(SCHED_AS_PER_SEC / 60);   // (no maximum quantum set: MAME's 60 Hz default)
	(void)sched_add_exec(10000000, m68k_run, m68k_done, m68k_abort);
	seibusnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.rom = snd;
	sc.rom_size = snd_size;
	sc.coin_port = rbx_port_find("COIN");
	sc.ym_type = RBX_YM3812;
	sc.ym_clock = 3579545;   // (7.15909 MHz / 2)
	sc.ym_gain = 256;
	sc.z80_clock = 3579545;
	sc.oki_rom = oki;
	sc.oki_size = okis;
	sc.oki_clock = 1000000;
	sc.oki_pin7 = 1;
	sc.oki_gain = 256;
	if (seibusnd_init(&sc)) {
		bb_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	m68k_bus_reset_map();
	m68k_bus.read8 = r8;
	m68k_bus.read16 = r16;
	m68k_bus.write8 = w8;
	m68k_bus.write16 = w16;
	m68k_bus_map(0x000000, 0x07ffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x080000, 0x08dfff, b.ram, 0xe000, 1, 1);
	m68k_bus_map(0x08e000, 0x08ffff, b.ram + 0xe000, 0x2000, 1, 0);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);
	bb_reset();
	return 0;
}

static int bb_state(void *ctx, int load)
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
	rc |= rbx_state_io(ctx, "bloodbro", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= seibusnd_state(ctx, load);
	rc |= sched_state(ctx, load);
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int bb_save(void *ctx) { return bb_state(ctx, 0); }
static int bb_load(void *ctx) { return bb_state(ctx, 1); }
static int bb_supported(const char *machine) { return !strcmp(machine, "bloodbro") || !strcmp(machine, "skysmash"); }

const rbx_driver rbx_drv_bloodbro = { "Seibu Blood Bros", bb_init, bb_exit, bb_reset, bb_frame, bb_save, bb_load,
                                      bb_supported, bb_render, bb_render_help };
