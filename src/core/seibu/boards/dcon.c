/*  Seibu D-Con / SD Gundam Psycho Salamander no Kyoui: a 68000, four tile layers through the Seibu CRTC, the SEI0211
    sprites, the Seibu Sound System (Z80 with a YM3812 or a YM2151, an OKI MSM6295).
    Ported from MAME 0.289 src/mame/seibu/dcon.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail) and
    src/mame/seibu/seibu_crtc.cpp (BSD-3-Clause, copyright-holders: Angelo Salese).
    This file is part of ReBOX. GPL-2.0-or-later.

    The CPUs interleaved as MAME's scheduler does it (sched.h). Not done: the coin counters.
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

#define VIS_W 320
#define VIS_H 224
#define NPENS 2048

static int s_sdg;                 // SD Gundam (its map, scroll offsets, sound)
static int s_y0;                  // the visible area's first line
static uint8_t *s_rom;
static uint32_t s_rom_size;
static rbx_gfx s_tx, s_bg, s_fg, s_mg, s_spr;
static int s_port_dsw, s_port_p, s_port_sys;
static uint8_t *s_pri;

typedef struct {
	uint8_t ram[0x10000];          // 0x80000-0x8ffff: work RAM, the layers, the palette, the sprites
	uint16_t crtc[0x28];
	uint16_t scroll[6];
	uint16_t layer_en, gfx_bank;
	uint8_t pal_written[NPENS];    // (MAME's palette: its initial colors until written)
	uint8_t state_end;
} board_t;
static board_t *s_bp;   // (calloc at the init: ADR-040)
#define b (*s_bp)

// (the layers in b.ram, host-order words)
#define RAM16(off) ((const uint16_t *)(b.ram + (off)))
enum { O_BG = 0xc000, O_FG = 0xc800, O_MG = 0xd000, O_TX = 0xd800, O_PAL = 0xe800, O_SPR = 0xf800 };

typedef struct {
	uint16_t bg[0x400], fg[0x400], mg[0x400], tx[0x800], spr[0x400];
	uint16_t scroll[6], layer_en, gfx_bank;
	uint32_t pal[NPENS];
} snap_t;
static snap_t *s_snapp;   // (calloc at the init: ADR-040)
#define s_snap (*s_snapp)
static int s_pal_dirty;

// ---- the main CPU's devices ----
static uint8_t snd_r(uint32_t off)
{
	if (s_sdg && off == 5)   // (sdgndmps_sound_comms_r: no sound commands sent while bit 0 is clear)
		return 1;
	return seibusnd_main_r((int)off);
}

static void crtc_w(uint32_t off, uint16_t v)
{
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
		return snd_r((a - 0xa0000) >> 1);
	if (a >= 0xc0000 && a < 0xc0050)
		return b.crtc[(a - 0xc0000) >> 1];
	if (a == 0xe0000)
		return (uint16_t)rbx_port_read(s_port_dsw);
	if (a == 0xe0002)
		return (uint16_t)rbx_port_read(s_port_p);
	if (a == 0xe0004)
		return (uint16_t)rbx_port_read(s_port_sys);
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
	if (a >= 0x9d000 && a < 0x9d800)
		b.gfx_bank = (v & 1) ? 0x1000 : 0;
	else if (a >= 0xa0000 && a < 0xa000e)
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
	switch (pri) {
	case 0: return 0xf0;   // above the foreground layer
	case 1: return 0xfc;   // above the midground layer
	case 2: return 0xfe;   // above the background layer
	default: return 0;     // above the text layer
	}
}

static void capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap.bg, RAM16(O_BG), sizeof(s_snap.bg));
	memcpy(s_snap.fg, RAM16(O_FG), sizeof(s_snap.fg));
	memcpy(s_snap.mg, RAM16(O_MG), sizeof(s_snap.mg));
	memcpy(s_snap.tx, RAM16(O_TX), sizeof(s_snap.tx));
	memcpy(s_snap.spr, RAM16(O_SPR), sizeof(s_snap.spr));
	memcpy(s_snap.scroll, b.scroll, sizeof(s_snap.scroll));
	s_snap.layer_en = b.layer_en;
	s_snap.gfx_bank = b.gfx_bank;
	const uint16_t *pal = RAM16(O_PAL);
	for (int i = 0; i < NPENS; i++) {   // xBGR_555
		if (!b.pal_written[i]) {   // (MAME's initial "rainbow": the index's bits 0-2 as R, G, B)
			s_snap.pal[i] = (i & 1 ? 0xff0000u : 0) | (i & 2 ? 0xff00u : 0) | (i & 4 ? 0xffu : 0);
			continue;
		}
		const uint16_t w = pal[i];
		const uint32_t r = w & 0x1f, g = (w >> 5) & 0x1f, bl = (w >> 10) & 0x1f;
		s_snap.pal[i] = (r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (bl << 3 | bl >> 2);
	}
	s_pal_dirty = 1;
	rbx_video_frame_done();
}

static void draw_band(int band)
{
	const int r0 = VIS_H * band / 8, r1 = VIS_H * (band + 1) / 8;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, en = ~s_snap.layer_en;
	const int xo = s_sdg ? 128 : 0;
	for (int r = r0; r < r1; r++) {
		const int y = r + s_y0;
		uint16_t *d = bm + r * pitch;
		uint8_t *pri = s_pri + r * VIS_W;
		memset(pri, 0, VIS_W);
		if (en & 1)
			seibu_tile16_line(s_snap.bg, 32, 32, 0, s_bg.pix, s_bg.count, 0, 1024 + 0, s_snap.scroll[0] + xo,
			                  s_snap.scroll[1], y, 0, d, pri, 0, VIS_W);
		else
			for (int x = 0; x < VIS_W; x++)
				d[x] = 15;   // (MAME: pen 15, "should always be black")
		if (en & 2)
			seibu_tile16_line(s_snap.mg, 32, 32, 0, s_mg.pix, s_mg.count, s_snap.gfx_bank, 1024 + 256,
			                  s_snap.scroll[2] + xo, s_snap.scroll[3], y, 1, d, pri, 1, VIS_W);
		if (en & 4)
			seibu_tile16_line(s_snap.fg, 32, 32, 0, s_fg.pix, s_fg.count, 0, 1024 + 512, s_snap.scroll[4] + xo,
			                  s_snap.scroll[5], y, 1, d, pri, 2, VIS_W);
		if (en & 8)
			seibu_tile8_line(s_snap.tx, 64, 32, 0, s_tx.pix, s_tx.count, 1024 + 768, xo, 0, y, 1, d, pri, 4, VIS_W);
	}
	if (en & 0x10) {
		sei0210_t sp = { 0, 1, 0, 0, pri_cb, s_spr.pix, s_spr.count, 0, NULL };
		sei0210_draw(&sp, s_snap.spr, 0x400, bm, pitch, s_pri, VIS_W, s_y0, r0, r1);
	}
}

static decoband_t s_bands = { 0, 0, 0, 0, 8, draw_band };

static void dc_render(void)
{
	if (s_pal_dirty) {
		memcpy(rbx_video.palette, s_snap.pal, sizeof(s_snap.pal));
		s_pal_dirty = 0;
	}
	decoband_run(&s_bands);
}

static int dc_render_help(int helper) { return decoband_help(&s_bands, helper); }

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
	case T_VBLANK:   // the screen update, the 68000's IRQ 4 (HOLD_LINE), the timers again
		capture();
		m68k_bus_hold_irq(4);
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

static void dc_frame(void)
{
	rbx_audio.frame_mhz = 60000;
	sched_run(T_VBLANK, fire);
	seibusnd_frame_end(s_frame_as);
	sched_rebase(s_frame_as);
}

static void dc_reset(void)
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

static void dc_exit(void)
{
	seibusnd_exit();
	rbx_gfx_free(&s_tx);
	rbx_gfx_free(&s_bg);
	rbx_gfx_free(&s_fg);
	rbx_gfx_free(&s_mg);
	rbx_gfx_free(&s_spr);
	free(s_pri);
	s_pri = NULL;
	free(s_bp);
	s_bp = NULL;
	free(s_snapp);
	s_snapp = NULL;
}

// MAME's dcon_charlayout (RGN_FRAC(1,2)) and dcon_tilelayout
static const uint32_t ch_planes[4] = { 0, 4, 0x10000 * 8, 0x10000 * 8 + 4 };
static const uint32_t ch_x[8] = { 3, 2, 1, 0, 11, 10, 9, 8 };
static const uint32_t ch_y[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
static const uint32_t tl_planes[4] = { 8, 12, 0, 4 };
static const uint32_t tl_x[16] = { 3, 2, 1, 0, 19, 18, 17, 16, 515, 514, 513, 512, 531, 530, 529, 528 };
static const uint32_t tl_y[16] = { 0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448, 480 };

static int dc_init(const rbx_game *g)
{
	s_bp = calloc(1, sizeof(board_t));
	s_snapp = calloc(1, sizeof(snap_t));
	if (!s_bp || !s_snapp) {
		dc_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_sdg = !strcmp(g->machine, "sdgndmps");
	s_y0 = s_sdg ? 16 : 0;
	uint32_t snd_size, txs, bgs, fgs, mgs, sps, okis = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *snd = rbx_region("audiocpu", &snd_size);
	const uint8_t *tx = rbx_region("txtiles", &txs), *bg = rbx_region("bgtiles", &bgs), *fg = rbx_region("fgtiles", &fgs),
	              *mg = rbx_region("mgtiles", &mgs), *sp = rbx_region("sprites", &sps), *oki = rbx_region("oki", &okis);
	if (!s_rom || !snd || !tx || !bg || !fg || !mg || !sp) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	const rbx_gfx_layout chl = { 8, 8, 4, ch_planes, ch_x, ch_y, 128 };
	const rbx_gfx_layout tll = { 16, 16, 4, tl_planes, tl_x, tl_y, 1024 };
	s_pri = (uint8_t *)calloc(1, VIS_W * VIS_H);
	if (txs < 0x20000 || rbx_gfx_decode(&s_tx, &chl, tx, txs / 2) || rbx_gfx_decode(&s_bg, &tll, bg, bgs) ||
	    rbx_gfx_decode(&s_fg, &tll, fg, fgs) || rbx_gfx_decode(&s_mg, &tll, mg, mgs) ||
	    rbx_gfx_decode(&s_spr, &tll, sp, sps) || !s_pri || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		dc_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_port_dsw = rbx_port_find("DSW");
	s_port_p = rbx_port_find("P1_P2");
	s_port_sys = rbx_port_find("SYSTEM");
	s_frame_as = (int64_t)(1e18 / 60.0);   // (HZ_TO_ATTOSECONDS(60))
	// (the scanline-0 timer: vpos 0 is (height - visible bottom - 1) lines after the vblank's start)
	s_scan0_as = (s_frame_as / 256) * (256 - (s_y0 + VIS_H));
	sched_init(SCHED_AS_PER_SEC / 60);   // (no maximum quantum set: MAME's 60 Hz default)
	(void)sched_add_exec(10000000, m68k_run, m68k_done, m68k_abort);
	seibusnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.rom = snd;
	sc.rom_size = snd_size;
	sc.coin_port = rbx_port_find("COIN");
	sc.oki_rom = oki;
	sc.oki_size = okis;
	sc.oki_pin7 = 0;
	sc.oki_gain = 102;   // (0.40)
	if (s_sdg) {
		sc.ym_type = RBX_YM2151;
		sc.ym_clock = 3579545;
		sc.ym_gain = 128;   // (0.50 each side)
		sc.z80_clock = 3579545;
		sc.oki_clock = 1250000;
	} else {
		sc.ym_type = RBX_YM3812;
		sc.ym_clock = 4000000;
		sc.ym_gain = 256;
		sc.z80_clock = 4000000;
		sc.oki_clock = 1320000;
	}
	if (seibusnd_init(&sc)) {
		dc_exit();
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
	dc_reset();
	return 0;
}

static int dc_state(void *ctx, int load)
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
	rc |= rbx_state_io(ctx, "dcon", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= seibusnd_state(ctx, load);
	rc |= sched_state(ctx, load);
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int dc_save(void *ctx) { return dc_state(ctx, 0); }
static int dc_load(void *ctx) { return dc_state(ctx, 1); }
static int dc_supported(const char *machine) { return !strcmp(machine, "dcon") || !strcmp(machine, "sdgndmps"); }

const rbx_driver rbx_drv_dcon = { "Seibu D-Con", dc_init, dc_exit, dc_reset, dc_frame, dc_save, dc_load, dc_supported,
                                  dc_render, dc_render_help };
