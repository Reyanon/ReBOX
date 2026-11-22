/*  Seibu Legionnaire hardware: Legionnaire, Heated Barrel, Godzilla, SD Gundam Sangokushi Rainbow Tairiku Senki,
    Denjin Makai. A 68000 with the Seibu COP (its DMAs fill the tilemaps and the palette), four tile layers through
    the Seibu CRTC, the SEI0211 sprites, the Seibu Sound System (Z80, YM3812 or YM2151, OKI MSM6295).
    Ported from MAME 0.289 src/mame/seibu/legionna.cpp, legionna.h, legionna_v.cpp (BSD-3-Clause, copyright-holders:
    David Graves, Angelo Salese, David Haywood, Tomasz Slanina) and src/mame/seibu/seibu_crtc.cpp (BSD-3-Clause,
    copyright-holders: Angelo Salese).
    This file is part of ReBOX. GPL-2.0-or-later.

    The CPUs interleaved as MAME's scheduler does it (sched.h). Not done: Seibu Cup Soccer (not working in MAME),
    the flip screen, the coin counters.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../sound/rbx_ym.h"
#include "../../video/decoband.h"
#include "../../video/gfxlayout.h"
#include "../machine/sched.h"
#include "../machine/seibucop.h"
#include "../machine/seibusnd.h"
#include "../video/sei0210.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

extern void (*rbx_m68k_irqack_hook)(int level);   // (cpu/m68k_bus.c)

#define NPENS 2049
#define PAL_BLACK 2048

enum { M_LEGIONNA, M_HEATBRL, M_GODZILLA, M_GRAINBOW, M_DENJINMK };

static int s_m, s_vis_w, s_vis_h, s_y0, s_x0;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static rbx_gfx s_tx, s_bg, s_fg, s_mg, s_spr;
static int s_has_mid;            // (a mid ROM of its own: MAME's split)
static uint32_t s_spr_base;      // (the sprites in b.ram)
static int s_port_dsw1, s_port_dsw2, s_port_p12, s_port_p34, s_port_sys;
static uint8_t *s_pri;
static uint16_t s_spr_pmask[4];
static uint64_t s_cyc_total;     // (the 68000's cycles before the running slice)

typedef struct {
	uint8_t ram[0x20000];          // 0x100000-0x11ffff (the I/O at 0x100400-0x1007ff apart)
	uint16_t back[0x400], fore[0x400], mid[0x400], text[0x800];   // (the COP's tilemap DMA)
	uint16_t pal[0x800];           // (the COP's palette DMA)
	uint16_t crtc[0x28];
	uint16_t scroll[6], layer_disable, text_dy;
	uint16_t back_bank, fore_bank, mid_bank, layer_config[4];
	int32_t reset_left;            // (MAME's 68000 reset: 34 clocks before the first instruction)
	int32_t pad0;
	uint64_t last_vpa;             // (m68000_device::m_last_vpa_time)
	uint8_t state_end;
} board_t;
static board_t *s_bp;   // (calloc at the init: ADR-040)
#define b (*s_bp)

typedef struct {
	uint16_t back[0x400], fore[0x400], mid[0x400], text[0x800], spr[0x800];
	uint16_t scroll[6], layer_disable, text_dy, back_bank, fore_bank, mid_bank;
	uint32_t pal[0x800];
} snap_t;
static snap_t *s_snapp;   // (calloc at the init: ADR-040)
#define s_snap (*s_snapp)
static int s_pal_dirty;

#define RAM16(off) ((uint16_t *)(b.ram + (off)))

// ---- the COP's outputs ----
static void videoram_w(int offset, uint16_t data)   // videowrite_cb_w
{
	if (offset < 0x400)
		b.back[offset] = data;
	else if (offset < 0x800)
		b.fore[offset - 0x400] = data;
	else if (offset < 0xc00)
		b.mid[offset - 0x800] = data;
	else if (offset < 0x1400)
		b.text[offset - 0xc00] = data;
}

static void palette_w(int offset, uint16_t data) { b.pal[offset & 0x7ff] = data; }
static int s_run_pre;            // (the running slice's reset clocks)
static int s_in_exec;
static uint64_t host_cycles(void) { return s_cyc_total + (uint64_t)s_run_pre + (uint64_t)m68k_cycles_run(); }

// ---- the main CPU's devices ----
static void crtc_w(uint32_t off, uint16_t v, uint16_t mask)
{
	if (off >= 0x28)
		return;
	b.crtc[off] = v;   // (the CRTC's register RAM: a whole word)
	if (off == 0x1c / 2)
		b.layer_disable = (uint16_t)((b.layer_disable & ~mask) | (v & mask));
	else if (off >= 0x20 / 2 && off < 0x2c / 2) {
		uint16_t *p = &b.scroll[off - 0x20 / 2];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
	} else if (off == 0x2c / 2 + 7 && s_m == M_GODZILLA)   // (layer_scroll_base offset 7, Godzilla: the text layer's scrolldy)
		b.text_dy = (uint16_t)(0x1ef - v);
}

static uint32_t crtc_base(void) { return s_m == M_HEATBRL ? 0x100640 : 0x100600; }
static uint32_t snd_base(void) { return s_m == M_HEATBRL ? 0x1007c0 : 0x100700; }

static uint16_t io_r16(uint32_t a)
{
	uint16_t v = 0;
	if (a < 0x100400 || a >= 0x100800)
		return *RAM16(a - 0x100000);
	const uint32_t cb = crtc_base(), sb = snd_base();
	if (a >= cb && a < cb + 0x50)
		return b.crtc[(a - cb) >> 1];
	if (a >= sb && a < sb + 0x20) {   // (the 8-bit handler's offset >> 1: a sound register every 4 bytes)
		const uint32_t o8 = (a - sb) >> 1;
		if (s_m == M_DENJINMK && o8 == 10)   // (denjinmk_sound_comms_r)
			return 1;
		return seibusnd_main_r((int)((o8 >> 1) & 7));
	}
	switch (a) {
	case 0x100740: return (uint16_t)rbx_port_read(s_port_dsw1);
	case 0x100744: return (uint16_t)rbx_port_read(s_port_p12);
	case 0x100748: return (uint16_t)rbx_port_read(s_port_p34);
	case 0x10074c: return (uint16_t)rbx_port_read(s_port_sys);
	case 0x10075c: return s_port_dsw2 >= 0 ? (uint16_t)rbx_port_read(s_port_dsw2) : 0;
	default: break;
	}
	if (seibucop_read(a - 0x100400, &v))
		return v;
	return 0;
}

static void io_w16(uint32_t a, uint16_t v, uint16_t mask)
{
	if (a < 0x100400 || a >= 0x100800) {
		uint16_t *p = RAM16(a - 0x100000);
		*p = (uint16_t)((*p & ~mask) | (v & mask));
		return;
	}
	const uint32_t cb = crtc_base(), sb = snd_base();
	if (a >= cb && a < cb + 0x50) {
		crtc_w((a - cb) >> 1, v, mask);
		return;
	}
	if (a >= sb && a < sb + 0x20) {
		if (mask & 0x00ff)
			seibusnd_main_w((int)((((a - sb) >> 1) >> 1) & 7), (uint8_t)v);
		return;
	}
	if (a == 0x100470) {   // (the gfx banks)
		if (s_m == M_HEATBRL || s_m == M_GODZILLA)
			b.back_bank = (v & 0x4000) >> 2;
		else if (s_m == M_DENJINMK) {
			b.back_bank = (v & 0x2000) >> 1;
			b.mid_bank = (v & 0x4000) >> 2;
			b.fore_bank = (v & 0x8000) >> 3;
		}
		return;
	}
	if (s_m == M_GRAINBOW && a >= 0x100480 && a < 0x100488) {
		uint16_t *p = &b.layer_config[(a - 0x100480) >> 1];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
		return;
	}
	(void)seibucop_write(a - 0x100400, v, mask);
}

static uint16_t r16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x100000 && a < 0x101000)
		return io_r16(a);
	return 0;
}

static uint8_t r8(uint32_t a)
{
	const uint16_t v = r16(a & ~1u);
	return (uint8_t)((a & 1) ? v : v >> 8);
}

static void w16(uint32_t a, uint16_t v)
{
	a &= 0xffffff;
	if (a >= 0x100000 && a < 0x101000)
		io_w16(a, v, 0xffff);
	else if (s_m == M_DENJINMK && a >= 0x104000 && a < 0x105000)   // palette_swap_w: A1 swapped
		*RAM16((a ^ 2) - 0x100000) = v;
}

static void w8(uint32_t a, uint8_t v)
{
	a &= 0xffffff;
	const uint16_t w = (a & 1) ? v : (uint16_t)(v << 8), mask = (a & 1) ? 0x00ff : 0xff00;
	if (a >= 0x100000 && a < 0x101000)
		io_w16(a & ~1u, w, mask);
	else if (s_m == M_DENJINMK && a >= 0x104000 && a < 0x105000) {
		uint16_t *p = RAM16(((a & ~1u) ^ 2) - 0x100000);
		*p = (uint16_t)((*p & ~mask) | (w & mask));
	}
}

// ---- the drawing ----
static uint32_t pri_cb(int pri, int ext)
{
	if (s_m == M_GRAINBOW)
		pri = (pri & 2) | (ext & 1);
	return s_spr_pmask[pri & 3];
}

static uint32_t bank_cb(uint32_t code, int ext, int y15)
{
	if (ext)
		code |= 0x4000;
	if (y15)
		code |= 0x8000;
	return code;
}

static void capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap.back, b.back, sizeof(s_snap.back));
	memcpy(s_snap.fore, b.fore, sizeof(s_snap.fore));
	memcpy(s_snap.mid, b.mid, sizeof(s_snap.mid));
	memcpy(s_snap.text, b.text, sizeof(s_snap.text));
	memcpy(s_snap.spr, RAM16(s_spr_base), sizeof(s_snap.spr));
	memcpy(s_snap.scroll, b.scroll, sizeof(s_snap.scroll));
	s_snap.layer_disable = b.layer_disable;
	s_snap.text_dy = b.text_dy;
	s_snap.back_bank = b.back_bank;
	s_snap.fore_bank = b.fore_bank;
	s_snap.mid_bank = b.mid_bank;
	for (int i = 0; i < 0x800; i++) {   // xBGR_555
		const uint16_t w = b.pal[i];
		const uint32_t r = w & 0x1f, g = (w >> 5) & 0x1f, bl = (w >> 10) & 0x1f;
		s_snap.pal[i] = (r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (bl << 3 | bl >> 2);
	}
	s_pal_dirty = 1;
	rbx_video_frame_done();
}

// a 16x16 layer (32 x 32, scan rows; pen 15 transparent) line; mid_share: Legionnaire's midground from the back ROM
static void layer16(const uint16_t *ram, const rbx_gfx *g, uint32_t bank, uint32_t or_code, uint16_t cbase, int sx,
                    int sy, int y, uint16_t *d, uint8_t *pri, uint8_t pv)
{
	const int ty = (y + sy) & 511, w = s_vis_w;
	for (int x = 0; x < w;) {
		const int tx = (x + s_x0 + sx) & 511;
		const uint16_t data = ram[(ty >> 4) * 32 + (tx >> 4)];
		const uint32_t code = rbx_wrap((data & 0xfff) | bank | or_code, g->count);
		const uint8_t *row = g->pix + (size_t)code * 256 + (size_t)(ty & 15) * 16;
		const uint16_t base = (uint16_t)(cbase + (data >> 12) * 16);
		for (int fx = tx & 15; fx < 16 && x < w; fx++, x++)
			if (row[fx] != 15) {
				d[x] = (uint16_t)(base + row[fx]);
				pri[x] |= pv;
			}
	}
}

static void draw_band(int band)
{
	const int r0 = s_vis_h * band / 8, r1 = s_vis_h * (band + 1) / 8;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, en = ~s_snap.layer_disable, w = s_vis_w;
	const uint16_t fill = (s_m == M_GODZILLA || s_m == M_DENJINMK) ? 0xff : PAL_BLACK;
	const uint32_t mid_or = s_has_mid ? 0 : 0x1000;
	const rbx_gfx *mg = s_has_mid ? &s_mg : &s_bg;
	const uint16_t mid_base = s_has_mid ? 16 * 16 : 256;   // (shared: color | 0x10 of the back's)
	const uint8_t txt_trans = s_m == M_DENJINMK ? 7 : 15;
	for (int r = r0; r < r1; r++) {
		const int y = r + s_y0;
		uint16_t *d = bm + r * pitch;
		uint8_t *pri = s_pri + r * w;
		for (int x = 0; x < w; x++)
			d[x] = fill;
		memset(pri, 0, (size_t)w);
#define BACK(pv) layer16(s_snap.back, &s_bg, s_snap.back_bank, 0, 0, s_snap.scroll[0], s_snap.scroll[1], y, d, pri, pv)
#define MID(pv) layer16(s_snap.mid, mg, s_has_mid ? s_snap.mid_bank : 0, mid_or, mid_base, s_snap.scroll[2], s_snap.scroll[3], y, d, pri, pv)
#define FORE(pv) layer16(s_snap.fore, &s_fg, s_snap.fore_bank, 0, 512, s_snap.scroll[4], s_snap.scroll[5], y, d, pri, pv)
		switch (s_m) {
		case M_LEGIONNA:
			if (en & 1) MID(0);
			if (en & 2) BACK(1);
			if (en & 4) FORE(2);
			break;
		case M_HEATBRL:
			if (en & 4) FORE(0);
			if (en & 2) MID(1);
			if (en & 1) BACK(2);
			break;
		default:
			if (en & 1) BACK(0);
			if (en & 2) MID(1);
			if (en & 4) FORE(2);
			break;
		}
#undef BACK
#undef MID
#undef FORE
		if (en & 8) {   // the text (64 x 32 of 8x8, pen 15 transparent, Denjin Makai's 7)
			const int ty = (y - (int16_t)s_snap.text_dy) & 255;   // (MAME tilemap: the source line y - dy)
			for (int x = 0; x < w;) {   // (in tile spans)
				const int tx = (x + s_x0) & 511;
				const uint16_t data = s_snap.text[(ty >> 3) * 64 + (tx >> 3)];
				const uint8_t *row = s_tx.pix + (size_t)rbx_wrap(data & 0xfff, s_tx.count) * 64 + (size_t)(ty & 7) * 8;
				const uint16_t base = (uint16_t)(768 + (data >> 12) * 16);
				for (int fx = tx & 7; fx < 8 && x < w; fx++, x++)
					if (row[fx] != txt_trans) {
						d[x] = (uint16_t)(base + row[fx]);
						pri[x] |= 4;
					}
			}
		}
	}
	if (en & 0x10) {
		const sei0210_t sp = { 0, 1, s_m == M_GRAINBOW ? 16 : 0, s_m == M_GRAINBOW ? 16 : 0, pri_cb, s_spr.pix,
		                       s_spr.count, 1024, (s_m == M_GODZILLA || s_m == M_DENJINMK) ? bank_cb : NULL };
		// (the bitmap's x from the visible area's left edge)
		if (s_x0) {
			// sprite x in screen coordinates: shift by the area's left edge through the offset
			sei0210_t sp2 = sp;
			sp2.xoff -= s_x0;
			sei0210_draw(&sp2, s_snap.spr, 0x800, bm, pitch, s_pri, w, s_y0, r0, r1);
		} else
			sei0210_draw(&sp, s_snap.spr, 0x800, bm, pitch, s_pri, w, s_y0, r0, r1);
	}
}

static decoband_t s_bands = { 0, 0, 0, 0, 8, draw_band };

static void lg_render(void)
{
	if (s_pal_dirty) {
		memcpy(rbx_video.palette, s_snap.pal, sizeof(s_snap.pal));
		rbx_video.palette[PAL_BLACK] = 0;
		s_pal_dirty = 0;
	}
	decoband_run(&s_bands);
}

static int lg_render_help(int helper) { return decoband_help(&s_bands, helper); }

// ---- the scheduler: the 68000, MAME's timers ----
static int m68k_run(int n)
{
	s_run_pre = 0;
	if (b.reset_left > 0) {   // (MAME's reset sequence: the vectors' and the prefetch's reads)
		s_run_pre = b.reset_left < n ? b.reset_left : n;
		b.reset_left -= s_run_pre;
		if (s_run_pre == n) {
			s_cyc_total += (uint64_t)n;
			s_run_pre = 0;
			return n;
		}
	}
	s_in_exec = 1;
	const int ran = s_run_pre + m68k_execute(n - s_run_pre);
	s_in_exec = 0;
	s_cyc_total += (uint64_t)ran;
	s_run_pre = 0;
	return ran;
}
static int64_t m68k_done(void) { return s_run_pre + m68k_cycles_run(); }

// MAME's autovector acknowledge: VPA, synchronized with the E clock (m68000_device::vpa_sync, vpa_after) at the
// vector read, 10 clocks into the interrupt's processing
static void vpa_ack(int level)
{
	(void)level;
	const uint64_t t = s_cyc_total + (s_in_exec ? (uint64_t)(s_run_pre + m68k_cycles_run()) : 0) + 10;
	uint64_t at = b.last_vpa;
	if (at < t) {
		const uint64_t mod = t % 10;
		at = t - mod + (mod < 7 ? 10 : 20);
		b.last_vpa = at;
	}
	m68k_add_int_cycles((int)(at - t) + 1);
}
static void m68k_abort(void) { m68k_end_timeslice(); }

enum { T_VBLANK = 1, T_VBLANK_END, T_SCANLINE0, T_SOUND };
static int64_t s_frame_as, s_scan0_as, s_vblank_as;
static uint32_t s_fps_mhz;

static void fire(int id, int32_t param)
{
	if (seibusnd_fire(id, param))
		return;
	switch (id) {
	case T_VBLANK:   // the screen update, the 68000's IRQ 4 (HOLD_LINE), the timers again
		capture();
		m68k_bus_hold_irq(4);
		sched_timer_set(T_VBLANK, sched_now() + s_frame_as, 0);
		sched_timer_set(T_VBLANK_END, sched_now() + s_vblank_as, 0);
		break;
	case T_SCANLINE0:
		sched_timer_set(T_SCANLINE0, sched_now() + s_frame_as, 0);
		break;
	case T_SOUND:
		sched_timer_set(T_SOUND, sched_now() + SCHED_AS_PER_SEC / 50, 0);
		break;
	default: break;
	}
}

static void lg_frame(void)
{
	rbx_audio.frame_mhz = s_fps_mhz;
	sched_run(T_VBLANK, fire);
	seibusnd_frame_end(s_frame_as);
	sched_rebase(s_frame_as);
}

static void lg_reset(void)
{
	rbx_video_wait_idle();
	memset(&b, 0, offsetof(__typeof__(b), state_end));
	seibucop_reset();
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	b.reset_left = 34;
	s_cyc_total = 0;
	s_run_pre = 0;
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

static void lg_exit(void)
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

// MAME's charlayout and tilelayout
static const uint32_t pl4[4] = { 0, 4, 8, 12 };
static const uint32_t ch_x[8] = { 3, 2, 1, 0, 19, 18, 17, 16 };
static const uint32_t ch_y[8] = { 0, 32, 64, 96, 128, 160, 192, 224 };
static const uint32_t tl_x[16] = { 3, 2, 1, 0, 19, 18, 17, 16, 515, 514, 513, 512, 531, 530, 529, 528 };
static const uint32_t tl_y[16] = { 0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448, 480 };

static int lg_init(const rbx_game *g)
{
	s_bp = calloc(1, sizeof(board_t));
	s_snapp = calloc(1, sizeof(snap_t));
	if (!s_bp || !s_snapp) {
		lg_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_m = !strcmp(g->machine, "heatbrl") ? M_HEATBRL : !strcmp(g->machine, "godzilla") ? M_GODZILLA :
	      !strcmp(g->machine, "grainbow") ? M_GRAINBOW : !strcmp(g->machine, "denjinmk") ? M_DENJINMK : M_LEGIONNA;
	uint32_t snd_size, txs, bgs, fgs, mgs = 0, sps, okis = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *snd = rbx_region("audiocpu", &snd_size);
	const uint8_t *tx = rbx_region("char", &txs), *bg = rbx_region("back", &bgs), *sp = rbx_region("sprite", &sps),
	              *mg = rbx_region("mid", &mgs), *oki = rbx_region("oki", &okis);
	uint8_t *fg = rbx_region("fore", &fgs);
	if (!s_rom || !snd || !tx || !bg || !fg || !sp) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_has_mid = (s_m == M_HEATBRL || s_m == M_DENJINMK) && mg;
	if (s_m == M_LEGIONNA && fgs >= 0x10000) {   // descramble_legionnaire_gfx
		uint8_t *buf = (uint8_t *)malloc(0x10000);
		if (!buf) {
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
		for (uint32_t i = 0; i < 0x10000; i++) {
			const uint32_t a = (i & 0xff0000) | ((i >> 6) & 1) << 15 | ((i >> 5) & 1) << 14 | ((i >> 15) & 1) << 13 |
			                   ((i >> 14) & 1) << 12 | ((i >> 13) & 1) << 11 | ((i >> 12) & 1) << 10 | ((i >> 11) & 1) << 9 |
			                   ((i >> 10) & 1) << 8 | ((i >> 9) & 1) << 7 | ((i >> 8) & 1) << 6 | ((i >> 7) & 1) << 5 |
			                   ((i >> 4) & 1) << 4 | (i & 0xf);
			buf[i] = fg[a];
		}
		memcpy(fg, buf, 0x10000);
		free(buf);
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	switch (s_m) {
	case M_LEGIONNA: s_vis_w = 256; s_vis_h = 224; s_y0 = 16; s_x0 = 0; s_spr_base = 0x5000; break;
	case M_HEATBRL: s_vis_w = 256; s_vis_h = 256; s_y0 = 0; s_x0 = 0; s_spr_base = 0x3000; break;
	case M_GODZILLA: s_vis_w = 320; s_vis_h = 224; s_y0 = 0; s_x0 = 0; s_spr_base = 0x5000; break;
	case M_GRAINBOW: s_vis_w = 320; s_vis_h = 224; s_y0 = 16; s_x0 = 16; s_spr_base = 0x7000; break;
	default: s_vis_w = 320; s_vis_h = 256; s_y0 = 0; s_x0 = 0; s_spr_base = 0x5000; break;
	}
	static const uint16_t pm_leg[4] = { 0x0000, 0xfff0, 0xfffc, 0xfffe }, pm_hb[4] = { 0xfff0, 0xfffc, 0xfffe, 0xffff },
	                      pm_dj[4] = { 0xfff0, 0xfffc, 0xfffe, 0x0000 };
	memcpy(s_spr_pmask, s_m == M_LEGIONNA ? pm_leg : (s_m == M_HEATBRL || s_m == M_GODZILLA) ? pm_hb : pm_dj, sizeof(s_spr_pmask));
	const rbx_gfx_layout chl = { 8, 8, 4, pl4, ch_x, ch_y, 256 };
	const rbx_gfx_layout tll = { 16, 16, 4, pl4, tl_x, tl_y, 1024 };
	s_pri = (uint8_t *)calloc(1, (size_t)s_vis_w * (size_t)s_vis_h);
	if (rbx_gfx_decode(&s_tx, &chl, tx, txs) || rbx_gfx_decode(&s_bg, &tll, bg, bgs) || rbx_gfx_decode(&s_fg, &tll, fg, fgs) ||
	    (s_has_mid && rbx_gfx_decode(&s_mg, &tll, mg, mgs)) || rbx_gfx_decode(&s_spr, &tll, sp, sps) || !s_pri ||
	    rbx_video_setup(s_vis_w, s_vis_h, NPENS)) {
		lg_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_port_dsw1 = rbx_port_find("DSW1");
	s_port_dsw2 = rbx_port_find("DSW2");
	s_port_p12 = rbx_port_find("PLAYERS12");
	s_port_p34 = rbx_port_find("PLAYERS34");
	s_port_sys = rbx_port_find("SYSTEM");
	// the screens: Godzilla's raw (14.31818 MHz / 2, 455 x 258, 320 x 224 shown), the others' refresh and 2500 us
	int height;
	if (s_m == M_GODZILLA) {
		height = 258;
		s_frame_as = (int64_t)(SCHED_AS_PER_SEC / 7159090) * 455 * 258;
		s_vblank_as = (s_frame_as / 258) * (258 - 224);
		s_fps_mhz = 60982;
	} else {
		const double hz = s_m == M_DENJINMK ? 56.0 : 60.0;
		height = 288;
		s_frame_as = (int64_t)(1e18 / hz);
		s_vblank_as = 2500000000000000LL;
		s_fps_mhz = s_m == M_DENJINMK ? 56000 : 60000;
	}
	s_scan0_as = (s_frame_as / height) * (height - (s_y0 + s_vis_h));
	sched_init(SCHED_AS_PER_SEC / 60);   // (no maximum quantum set: MAME's 60 Hz default)
	(void)sched_add_exec(10000000, m68k_run, m68k_done, m68k_abort);
	seibusnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.rom = snd;
	sc.rom_size = snd_size;
	sc.coin_port = rbx_port_find("COIN");
	sc.z80_clock = 3579545;   // (14318180 / 4)
	sc.ym_clock = 3579545;
	if (s_m == M_LEGIONNA || s_m == M_HEATBRL) {
		sc.ym_type = RBX_YM3812;
		sc.ym_gain = 256;
	} else {
		sc.ym_type = RBX_YM2151;
		sc.ym_gain = 128;   // (0.50 each side)
	}
	sc.oki_rom = oki;
	sc.oki_size = okis;
	sc.oki_clock = 1000000;   // (20 MHz / 20)
	sc.oki_pin7 = 1;
	sc.oki_gain = 102;   // (0.40)
	sc.oki_bank_port = s_m == M_GODZILLA;
	if (seibusnd_init(&sc)) {
		lg_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	const seibucop_config cc = { videoram_w, palette_w, host_cycles };
	seibucop_init(&cc);
	m68k_bus_reset_map();
	m68k_bus.read8 = r8;
	m68k_bus.read16 = r16;
	m68k_bus.write8 = w8;
	m68k_bus.write16 = w16;
	m68k_bus_map(0x000000, s_rom_size - 1, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x101000, 0x11ffff, b.ram + 0x1000, 0x1f000, 1, 1);
	if (s_m == M_DENJINMK)   // (the palette swap's page: writes through the handler)
		m68k_bus_cur->wr[0x104000 >> M68K_PAGE_BITS] = NULL;
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);
	rbx_m68k_irqack_hook = vpa_ack;
	lg_reset();
	return 0;
}

static int lg_state(void *ctx, int load)
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
	rc |= rbx_state_io(ctx, "legionna", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= rbx_state_io(ctx, "legionna_cyc", &s_cyc_total, sizeof(s_cyc_total));
	rc |= seibucop_state(ctx, load);
	rc |= seibusnd_state(ctx, load);
	rc |= sched_state(ctx, load);
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int lg_save(void *ctx) { return lg_state(ctx, 0); }
static int lg_load(void *ctx) { return lg_state(ctx, 1); }
static int lg_supported(const char *machine)
{
	return !strcmp(machine, "legionna") || !strcmp(machine, "heatbrl") || !strcmp(machine, "godzilla") ||
	       !strcmp(machine, "grainbow") || !strcmp(machine, "denjinmk");
}

const rbx_driver rbx_drv_legionna = { "Seibu Legionnaire", lg_init, lg_exit, lg_reset, lg_frame, lg_save, lg_load,
                                      lg_supported, lg_render, lg_render_help };
