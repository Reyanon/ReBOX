/*  Seibu Kaihatsu Good E Jong / Tottemo E Jong: a V30 with the Seibu CRTC's three 16x16 tilemaps and 8x8 text layer,
    the SEI0211 (SEI0210) sprites, a mahjong panel; the Seibu Sound System (Z80, YM3812, OKI MSM6295); 2048 colors
    (xBGR_444 RAM).
    Ported from MAME 0.289 src/mame/seibu/goodejan.cpp (BSD-3-Clause, copyright-holders: Angelo Salese, Pierpaolo
    Prazzoli) and src/mame/seibu/seibu_crtc.cpp (BSD-3-Clause, copyright-holders: Angelo Salese).
    This file is part of ReBOX. GPL-2.0-or-later.

    The CPUs interleaved by the Seibu scheduler (sched.h) as the other Seibu Sound System boards.
*/
#include "../../rbx.h"
#include "../../cpu/nec/nec.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../machine/seibusnd.h"
#include "../machine/sched.h"
#include "../video/sei0210.h"
#include "../video/seibu_draw.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define VIS_W 256
#define VIS_Y0 16
#define VIS_H 224
#define FRAME_MHZ 59630
#define NPENS 0x800

static uint8_t *s_rom;
static uint32_t s_rom_size;
static rbx_gfx s_gbg, s_gmd, s_gfg, s_gtx, s_gspr;
static int s_port_dsw1, s_port_dsw2, s_port_key[5];
static int s_totmejan;
static nec_t s_cpu;
static uint8_t *s_pri;

typedef struct {
	uint8_t ram[0x10000];        // 0x00000-0x0ffff (sc0 0xc000, sc3 0xc800, palette 0xd000, sc1 0xe000, sc2 0xe800, sprites 0xf800)
	uint16_t crtc[0x40];
	uint16_t layer_en, scroll[6], sc0bank, mux;
	uint8_t state_end;
} board_t;
static board_t *s_bp;
#define b (*s_bp)

typedef struct {
	uint16_t sc0[0x400], sc1[0x400], sc2[0x400], sc3[0x400], spr[0x400];
	uint16_t layer_en, scroll[6], sc0bank;
	uint32_t pal[NPENS];
} snap_t;
static snap_t *s_snapp;
#define s_snap (*s_snapp)

// ---- the main CPU ----
static uint8_t mem_r8(uint32_t a) { (void)a; return 0xff; }
static uint16_t mem_r16(uint32_t a) { (void)a; return 0xffff; }
static void mem_w8(uint32_t a, uint8_t v) { (void)a; (void)v; }
static void mem_w16(uint32_t a, uint16_t v) { (void)a; (void)v; }

static void crtc_w(int off, uint16_t v, uint16_t mask)
{
	off &= 0x3f;
	b.crtc[off] = (uint16_t)((b.crtc[off] & ~mask) | (v & mask));
	if (off == 0x1c / 2)
		b.layer_en = b.crtc[off];
	else if (off >= 0x20 / 2 && off < 0x2c / 2) {
		uint16_t *p = &b.scroll[off - 0x20 / 2];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
	}
}

static int crtc_off(uint32_t p) { return (int)(((p - 0x8000) >> 1) ^ (s_totmejan ? 0 : 0x20)); }

static uint16_t io_r16(uint32_t p)
{
	p &= 0xffff;
	if (p >= 0x8000 && p < (s_totmejan ? 0x8050u : 0x8080u)) {
		const int off = crtc_off(p) & 0x3f;
		return off == 0x1a / 2 ? b.crtc[off] : 0;
	}
	switch (p) {
	case 0xc000: return (uint16_t)rbx_port_read(s_port_dsw1);
	case 0xc002: {   // mahjong_panel_r
		uint16_t ret = 0x3f;
		for (int i = 0; i < 5; i++)
			if ((b.mux >> i) & 1)
				ret &= (uint16_t)rbx_port_read(s_port_key[i]);
		return (uint16_t)(0xffc0 | ret);
	}
	case 0xc004: return (uint16_t)rbx_port_read(s_port_dsw2);
	default: break;
	}
	if (p >= 0xd000 && p < 0xd010)
		return seibusnd_main_r((int)((p - 0xd000) >> 1));
	return 0xffff;
}

static uint8_t io_r8(uint32_t p)
{
	const uint16_t v = io_r16(p & ~1u);
	return (uint8_t)((p & 1) ? v >> 8 : v);
}

static void io_w16m(uint32_t p, uint16_t v, uint16_t mask)
{
	p &= 0xffff;
	if (p >= 0x8000 && p < (s_totmejan ? 0x8050u : 0x8080u)) {
		crtc_w(crtc_off(p), v, mask);
		return;
	}
	if (p == 0x9000) {   // gfxbank_w (bit 8)
		if (mask & 0xff00)
			b.sc0bank = (v >> 8) & 1;
		return;
	}
	if (p == 0xb004) {
		b.mux = (uint16_t)((b.mux & ~mask) | (v & mask));
		return;
	}
	if (p >= 0xd000 && p < 0xd010 && (mask & 0x00ff))
		seibusnd_main_w((int)((p - 0xd000) >> 1), (uint8_t)v);
}

static uint16_t io_in16(uint32_t p) { return io_r16(p); }
static uint8_t io_in8(uint32_t p) { return io_r8(p); }
static void io_out16(uint32_t p, uint16_t v) { io_w16m(p, v, 0xffff); }
static void io_out8(uint32_t p, uint8_t v)
{
	if (p & 1)
		io_w16m(p & ~1u, (uint16_t)(v << 8), 0xff00);
	else
		io_w16m(p, v, 0x00ff);
}

static int vector_r(void) { return 0x208 / 4; }

// ---- the drawing ----
static uint32_t pri_cb(int pri, int ext)
{
	(void)ext;
	switch (pri) {
	case 0: return 0xff00;
	case 1: return 0xff00 | 0xf0f0;
	case 2: return 0xff00 | 0xf0f0 | 0xcccc;
	default: return 0;
	}
}

static void layer(const uint16_t *ram, const rbx_gfx *g, int tsz, uint32_t bank, uint16_t cbase, int sx, int sy, int opaque,
                  uint8_t pv)
{
	const int mw = 32 * tsz, mh = 32 * tsz, tshift = tsz == 16 ? 4 : 3;
	for (int r = 0; r < VIS_H; r++)
		sd_tile_row(rbx_video.bitmap + r * rbx_video.pitch, s_pri + r * VIS_W, pv, ram, 32, tshift, mw - 1,
		            (r + VIS_Y0 + sy) & (mh - 1), sx, g, 0, bank, cbase, opaque ? -1 : 15, VIS_W);
}

static void gj_render(void)
{
	memcpy(rbx_video.palette, s_snap.pal, sizeof(s_snap.pal));
	memset(s_pri, 0, VIS_W * VIS_H);
	for (int r = 0; r < VIS_H; r++) {
		uint16_t *d = rbx_video.bitmap + r * rbx_video.pitch;
		for (int x = 0; x < VIS_W; x++)
			d[x] = 0x7ff;
	}
	const uint16_t en = s_snap.layer_en, *sc = s_snap.scroll;
	if (!(en & 1)) layer(s_snap.sc0, &s_gbg, 16, (uint32_t)s_snap.sc0bank << 12, 0x000, sc[0] & 0x1ff, sc[1] & 0x1ff, 1, 1);
	if (!(en & 2)) layer(s_snap.sc2, &s_gmd, 16, 0, 0x300, sc[2] & 0x1ff, sc[3] & 0x1ff, 0, 2);
	if (!(en & 4)) layer(s_snap.sc1, &s_gfg, 16, 0, 0x600, sc[4] & 0x1ff, sc[5] & 0x1ff, 0, 4);
	if (!(en & 8)) layer(s_snap.sc3, &s_gtx, 8, 0, 0x100, 0, 0, 0, 8);
	if (!(en & 0x10)) {
		const sei0210_t sg = { 0, !s_totmejan, 0, 0, pri_cb, s_gspr.pix, s_gspr.count, 0x200, NULL };
		sei0210_draw(&sg, s_snap.spr, 0x400, rbx_video.bitmap, rbx_video.pitch, s_pri, VIS_W, VIS_Y0, 0, VIS_H);
	}
}

static void capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap.sc0, b.ram + 0xc000, sizeof(s_snap.sc0));
	memcpy(s_snap.sc3, b.ram + 0xc800, sizeof(s_snap.sc3));
	memcpy(s_snap.sc1, b.ram + 0xe000, sizeof(s_snap.sc1));
	memcpy(s_snap.sc2, b.ram + 0xe800, sizeof(s_snap.sc2));
	memcpy(s_snap.spr, b.ram + 0xf800, sizeof(s_snap.spr));
	s_snap.layer_en = b.layer_en;
	memcpy(s_snap.scroll, b.scroll, sizeof(s_snap.scroll));
	s_snap.sc0bank = b.sc0bank;
	const uint16_t *pal = (const uint16_t *)(b.ram + 0xd000);
	for (int i = 0; i < NPENS; i++) {   // xBGR_444
		const uint16_t w = pal[i];
		s_snap.pal[i] = (uint32_t)(w & 0xf) * 0x110000u | (uint32_t)((w >> 4) & 0xf) * 0x1100u | (uint32_t)((w >> 8) & 0xf) * 0x11u;
	}
	rbx_video_frame_done();
}

// ---- the scheduler ----
static int64_t s_run0;
static int v30_run(int n)
{
	s_run0 = s_cpu.total;
	nec_run_until(&s_cpu, s_run0 + n);
	return (int)(s_cpu.total - s_run0);
}
static int64_t v30_done(void) { return nec_now(&s_cpu) - s_run0; }
static void v30_abort(void) { s_cpu.stop = 1; }

enum { T_VBLANK = 1, T_SOUND };
static int64_t s_frame_as;

static void fire(int id, int32_t param)
{
	if (seibusnd_fire(id, param))
		return;
	switch (id) {
	case T_VBLANK:   // the screen update, IRQ 0 (HOLD_LINE)
		capture();
		nec_set_input(&s_cpu, NEC_LINE_IRQ, 1);
		sched_timer_set(T_VBLANK, sched_now() + s_frame_as, 0);
		break;
	case T_SOUND:
		sched_timer_set(T_SOUND, sched_now() + SCHED_AS_PER_SEC / 50, 0);
		break;
	default: break;
	}
}

static void gj_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	sched_run(T_VBLANK, fire);
	seibusnd_frame_end(s_frame_as);
	sched_rebase(s_frame_as);
}

static void gj_reset(void)
{
	rbx_video_wait_idle();
	memset(&b, 0, offsetof(__typeof__(b), state_end));
	nec_reset(&s_cpu);
	s_cpu.total = 0;
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

static void gj_exit(void)
{
	rbx_video_wait_idle();
	seibusnd_exit();
	rbx_gfx_free(&s_gbg);
	rbx_gfx_free(&s_gmd);
	rbx_gfx_free(&s_gfg);
	rbx_gfx_free(&s_gtx);
	rbx_gfx_free(&s_gspr);
	free(s_pri);
	s_pri = NULL;
	free(s_bp);
	s_bp = NULL;
	free(s_snapp);
	s_snapp = NULL;
}

static const uint32_t tl_planes[4] = { 12, 8, 4, 0 };
static const uint32_t tl_x[16] = { 0, 1, 2, 3, 16, 17, 18, 19, 512, 513, 514, 515, 528, 529, 530, 531 };
static const uint32_t tl_y[16] = { 0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448, 480 };

static int gj_init(const rbx_game *g)
{
	s_totmejan = !strcmp(g->machine, "totmejan");
	uint32_t snd_size, bgs, mds, fgs, txs, sps, okis = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *snd = rbx_region("audiocpu", &snd_size);
	const uint8_t *bg = rbx_region("bg_gfx", &bgs), *md = rbx_region("md_gfx", &mds), *fg = rbx_region("fg_gfx", &fgs),
	              *tx = rbx_region("tx_gfx", &txs), *sp = rbx_region("spr_gfx", &sps), *oki = rbx_region("oki", &okis);
	if (!s_rom || s_rom_size < 0x40000 || !snd || !bg || !md || !fg || !tx || !sp || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_bp = calloc(1, sizeof(board_t));
	s_snapp = calloc(1, sizeof(snap_t));
	s_pri = calloc(1, VIS_W * VIS_H);
	const rbx_gfx_layout tll = { 16, 16, 4, tl_planes, tl_x, tl_y, 1024 };
	const rbx_gfx_layout chl = { 8, 8, 4, tl_planes, tl_x, tl_y, 256 };
	if (!s_bp || !s_snapp || !s_pri || rbx_gfx_decode(&s_gbg, &tll, bg, bgs) || rbx_gfx_decode(&s_gmd, &tll, md, mds) ||
	    rbx_gfx_decode(&s_gfg, &tll, fg, fgs) || rbx_gfx_decode(&s_gtx, &chl, tx, txs) ||
	    rbx_gfx_decode(&s_gspr, &tll, sp, sps) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		gj_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_port_dsw1 = rbx_port_find("DSW1");
	s_port_dsw2 = rbx_port_find("DSW2");
	for (int i = 0; i < 5; i++) {
		char n[8] = "KEY0";
		n[3] = (char)('0' + i);
		s_port_key[i] = rbx_port_find(n);
	}
	s_frame_as = (int64_t)(1e18 / 59.6303);
	sched_init(SCHED_AS_PER_SEC / 60);
	(void)sched_add_exec(8000000, v30_run, v30_done, v30_abort);
	seibusnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.rom = snd;
	sc.rom_size = snd_size;
	sc.ym_type = RBX_YM3812;
	sc.ym_clock = 3579545;
	sc.ym_gain = 256;
	sc.z80_clock = 3579545;
	sc.oki_rom = oki;
	sc.oki_size = okis;
	sc.oki_clock = 1000000;
	sc.oki_pin7 = 1;
	sc.oki_gain = 102;   // (0.40)
	sc.coin_port = rbx_port_find("COIN");
	if (seibusnd_init(&sc)) {
		gj_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	nec_init(&s_cpu, NEC_V30, NULL);
	s_cpu.irq_ack = vector_r;
	nec_map(&s_cpu, 0x00000, 0x10000, b.ram, b.ram);
	nec_map(&s_cpu, 0xc0000, 0x100000, s_rom + (s_rom_size - 0x40000), NULL);
	s_cpu.read8 = mem_r8;
	s_cpu.read16 = mem_r16;
	s_cpu.write8 = mem_w8;
	s_cpu.write16 = mem_w16;
	s_cpu.in8 = io_in8;
	s_cpu.in16 = io_in16;
	s_cpu.out8 = io_out8;
	s_cpu.out16 = io_out16;
	gj_reset();
	return 0;
}

static int gj_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "goodejan", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= rbx_state_io(ctx, "goodejan_cpu", &s_cpu, NEC_STATE_SIZE);
	rc |= seibusnd_state(ctx, load);
	rc |= sched_state(ctx, load);
	if (load && !rc)
		nec_after_load(&s_cpu);
	return rc ? -1 : 0;
}

static int gj_save(void *ctx) { return gj_state(ctx, 0); }
static int gj_load(void *ctx) { return gj_state(ctx, 1); }
static int gj_supported(const char *m) { return !strcmp(m, "goodejan") || !strcmp(m, "totmejan"); }

const rbx_driver rbx_drv_goodejan = { "Seibu Good E Jong", gj_init, gj_exit, gj_reset, gj_frame, gj_save, gj_load,
                                      gj_supported, gj_render, NULL };
