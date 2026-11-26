/*  Capcom The Speed Rumbler / Rush & Crash. Ported from MAME src/mame/capcom/srumbler.cpp (BSD-3-Clause,
    copyright-holders: Paul Leaman).
    MC6809 at 8 MHz (IRQ at vblank start, FIRQ at its end) with the 0x5000-0xffff space in 4 KB ROM banks chosen
    through two PROMs; Z80 sound CPU at 4 MHz with two YM2203 (the first one's IRQ). Video: a 1024x1024 background
    of 16x16 tiles split in two layers around the sprites, a 512x256 text layer, 16x16 sprites copied at vblank;
    RGBx 444 palette RAM.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../../video/decoband.h"
#include "cap8.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 57445          // set_raw(16 MHz / 2, 512, 80, 432, 272, 8, 248)
#define LINES 272
#define VIS_X0 80
#define VIS_W 352
#define VIS_Y0 8
#define VIS_H 240
#define MAIN_CLOCK 2000000       // (MC6809 at 8 MHz: E = 2 MHz)
#define SND_CLOCK 4000000
#define YM_CLOCK 4000000

static uint8_t *s_rom, *s_zrom, *s_prom;
static uint32_t s_rom_size, s_zrom_size;
static rbx_gfx s_gchar, s_gtile, s_gspr;
static int s_port[5];
static m6809_t s_main;
static z80 s_snd;
static const uint8_t *s_zmap[256];
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;

static struct {
	uint8_t ram[0x2000], bg[0x2000], fg[0x1000], pal[0x400], sprbuf[0x200];
	uint8_t zram[0x800];
	uint8_t bank, flip, latch, scroll[4];
	int ym_irq;
	int64_t main_t;
	uint64_t main_frac, snd_frac, ym_frac;
	uint8_t state_end;
} *t;   // (calloc in init: ADR-040)

static struct {
	uint8_t bg[0x2000], fg[0x1000], pal[0x400], spr[0x200];
	uint16_t sx, sy;
	uint8_t flip;
} *s_snap;

// ---- sound ----
static void ym_irq(int state) { t->ym_irq = state; }

static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_zrom[a];
	if (a >= 0xc000 && a < 0xc800) return t->zram[a - 0xc000];
	if (a == 0xe000) return t->latch;
	return 0;
}
static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xc800) t->zram[a - 0xc000] = v;
	else if (a == 0x8000 || a == 0x8001) rbx_ym_write(0, a & 1, v);
	else if (a == 0xa000 || a == 0xa001) rbx_ym_write(1, a & 1, v);
}
static uint8_t snd_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void snd_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

// ---- main CPU ----
static void map_banks(void)
{
	// (bankswitch_w: bit 2 of prom1, ROM or RAM, not supported in MAME either)
	const uint8_t *p1 = s_prom + (t->bank & 0xf0), *p2 = s_prom + 0x100 + ((t->bank & 0x0f) << 4);
	for (int i = 5; i < 16; i++) {
		const uint32_t b = (uint32_t)((p1[i] & 3) << 4 | (p2[i] & 0x0f));
		for (int p = 0; p < 16; p++)
			s_main.rpage[i * 16 + p] = s_rom + rbx_wrap((b * 0x1000 + p * 256), s_rom_size);
	}
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	switch (a) {
	case 0x4008: return (uint8_t)rbx_port_read(s_port[0]);
	case 0x4009: return (uint8_t)rbx_port_read(s_port[1]);
	case 0x400a: return (uint8_t)rbx_port_read(s_port[2]);
	case 0x400b: return (uint8_t)rbx_port_read(s_port[3]);
	case 0x400c: return (uint8_t)rbx_port_read(s_port[4]);
	}
	return 0;
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	switch (a) {
	case 0x4008: t->bank = v; map_banks(); break;
	case 0x4009: t->flip = v & 1; break;
	case 0x400a: case 0x400b: case 0x400c: case 0x400d: t->scroll[a - 0x400a] = v; break;
	case 0x400e: t->latch = v; break;
	}
}

static void main_pages(void)
{
	for (int p = 0; p < 0x20; p++) {
		s_main.rpage[p] = s_main.wpage[p] = t->ram + p * 256;
		s_main.rpage[0x20 + p] = s_main.wpage[0x20 + p] = t->bg + p * 256;
	}
	for (int p = 0; p < 0x10; p++)
		s_main.wpage[0x50 + p] = t->fg + p * 256;
	for (int p = 0; p < 4; p++)
		s_main.wpage[0x70 + p] = t->pal + p * 256;
	map_banks();
}

// ---- video ----
// (the tile's lookups once per tile: the pixels are the same as per-pixel lookups)
static void draw_bg(int front, int y0, int y1)
{
	// (set_transmask(group, front, back): group 0 all in the back, group 1 pens 11-15 in front)
	static const uint16_t fmask[2] = { 0xffff, 0x07ff }, bmask[2] = { 0x0000, 0xf800 };
	const int flip = s_snap->flip;
	for (int y = y0; y < y1; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y + s_snap->sy : y + s_snap->sy) & 1023;
		uint32_t last = ~0u;
		const uint8_t *src = NULL;
		uint16_t m = 0, base = 0;
		for (int x = 0; x < VIS_W; x++) {
			const int xa = x + VIS_X0;
			const uint32_t tx = (uint32_t)(flip ? 511 - xa + s_snap->sx : xa + s_snap->sx) & 1023;
			if ((tx >> 4) != last) {
				last = tx >> 4;
				const uint32_t ti = (tx >> 4) * 64 + (ty >> 4);
				const uint8_t attr = s_snap->bg[2 * ti];
				const uint32_t code = rbx_wrap((uint32_t)(s_snap->bg[2 * ti + 1] + ((attr & 7) << 8)), s_gtile.count);
				const uint32_t py = (attr & 8) ? 15 - (ty & 15) : ty & 15;
				src = s_gtile.pix + code * 256 + py * 16;
				const int grp = (attr & 0x10) >> 4;
				m = front ? fmask[grp] : bmask[grp];
				base = (uint16_t)(128 + ((attr & 0xe0) >> 5) * 16);
			}
			const uint8_t p = src[tx & 15];
			if (!((m >> p) & 1))
				d[x] = (uint16_t)(base + p);
		}
	}
}

static void draw_fg(int y0, int y1)
{
	const int flip = s_snap->flip;
	for (int y = y0; y < y1; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y : y) & 255;
		uint32_t last = ~0u;
		const uint8_t *src = NULL;
		uint16_t base = 0;
		int opaque = 0;
		for (int x = 0; x < VIS_W; x++) {
			const int xa = x + VIS_X0;
			const uint32_t tx = (uint32_t)(flip ? 511 - xa : xa) & 511;
			if ((tx >> 3) != last) {
				last = tx >> 3;
				const uint32_t ti = (tx >> 3) * 32 + (ty >> 3);
				const uint8_t attr = s_snap->fg[2 * ti];
				const uint32_t code = rbx_wrap((uint32_t)(s_snap->fg[2 * ti + 1] + ((attr & 3) << 8)), s_gchar.count);
				src = s_gchar.pix + code * 64 + (ty & 7) * 8;
				opaque = (attr & 0x40) != 0;   // (TILE_FORCE_LAYER0: opaque)
				base = (uint16_t)(448 + ((attr & 0x3c) >> 2) * 4);
			}
			const uint8_t p = src[tx & 7];
			if (p != 3 || opaque)
				d[x] = (uint16_t)(base + p);
		}
	}
}

// the frame in bands of lines drawn in parallel (decoband.h): the back layer, the sprites once, the front layers
#define BANDS 8
static int s_phase;
static void sr_band(int band)
{
	const int y0 = VIS_Y0 + VIS_H * band / BANDS, y1 = VIS_Y0 + VIS_H * (band + 1) / BANDS;
	if (s_phase == 0)
		draw_bg(0, y0, y1);
	else {
		draw_bg(1, y0, y1);
		draw_fg(y0, y1);
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, sr_band };
static int sr_help(int helper) { return decoband_help(&s_bands, helper); }

static void sr_render(void)
{
	for (int i = 0; i < 512; i++) {   // RGBx_444, big-endian words
		const uint32_t w = (uint32_t)(s_snap->pal[2 * i] << 8 | s_snap->pal[2 * i + 1]);
		rbx_video.palette[i] = ((w >> 12) & 15) * 17 << 16 | ((w >> 8) & 15) * 17 << 8 | ((w >> 4) & 15) * 17;
	}
	const int flip = s_snap->flip;
	s_phase = 0;
	decoband_run(&s_bands);
	for (int o = 0x200 - 4; o >= 0; o -= 4) {
		const uint8_t *s = &s_snap->spr[o];
		const int attr = s[1];
		int sx = s[3] + 0x100 * (attr & 1), sy = s[2], fy = (attr & 2) != 0;
		if (flip) {
			sx = 496 - sx;
			sy = 240 - sy;
			fy = !fy;
		}
		cap8_transpen(&s_gspr, (uint32_t)(s[0] + ((attr & 0xe0) << 3)), (uint16_t)(256 + ((attr & 0x1c) >> 2) * 16), flip, fy, sx, sy,
		              15, VIS_Y0, VIS_Y0 + VIS_H, VIS_Y0, VIS_X0, VIS_W);
	}
	s_phase = 1;
	decoband_run(&s_bands);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->bg, t->bg, sizeof(t->bg));
	memcpy(s_snap->fg, t->fg, sizeof(t->fg));
	memcpy(s_snap->pal, t->pal, sizeof(t->pal));
	memcpy(s_snap->spr, t->sprbuf, sizeof(t->sprbuf));
	s_snap->sx = (uint16_t)(t->scroll[0] | t->scroll[1] << 8);
	s_snap->sy = (uint16_t)(t->scroll[2] | t->scroll[3] << 8);
	s_snap->flip = t->flip;
	rbx_video_frame_done();
	memcpy(t->sprbuf, t->ram + 0x1e00, sizeof(t->sprbuf));   // (vblank_copy_rising, after the screen's update)
}

// ---- init / frame ----
static void snd_start(void)
{
	z80_init(&s_snd);
	s_snd.read_byte = snd_read;
	s_snd.write_byte = snd_write;
	s_snd.port_in = snd_in;
	s_snd.port_out = snd_out;
	s_snd.read_map = s_zmap;
}

static void sr_reset(void)
{
	memset(t, 0, offsetof(__typeof__(*t), state_end));
	m6809_init(&s_main);
	s_main.read = main_read;
	s_main.write = main_write;
	main_pages();
	m6809_reset(&s_main);
	snd_start();
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	s_mix_n = 0;
}

static void sr_exit(void);

static int sr_init(const rbx_game *g)
{
	free(t);
	t = calloc(1, sizeof(*t));
	if (!t) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t s1, s2, s3, sp;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_prom = rbx_region("proms", &sp);
	uint8_t *chars = rbx_region("chars", &s1), *tiles = rbx_region("tiles", &s2), *spr = rbx_region("sprites", &s3);
	if (!s_rom || s_rom_size < 0x10000 || !s_zrom || s_zrom_size < 0x8000 || !s_prom || sp < 0x200 || !chars || !tiles || !spr) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{	// charlayout, tilelayout (RGN_FRAC(1,2)), spritelayout (RGN_FRAC(1,4))
		static const uint32_t pc[2] = { 4, 0 }, xc[8] = { 0, 1, 2, 3, 8, 9, 10, 11 };
		static const uint32_t yc[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
		static const uint32_t xt[16] = { 0, 1, 2, 3, 8, 9, 10, 11, 256, 257, 258, 259, 264, 265, 266, 267 };
		static const uint32_t xs[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 128, 129, 130, 131, 132, 133, 134, 135 };
		static uint32_t yt[16], ys[16];
		for (int i = 0; i < 16; i++) {
			yt[i] = (uint32_t)i * 16;
			ys[i] = (uint32_t)i * 8;
		}
		const uint32_t h2 = s2 / 2 * 8, q3 = s3 / 4 * 8;
		const uint32_t p2[4] = { h2 + 4, h2, 4, 0 }, p3[4] = { 3 * q3, 2 * q3, q3, 0 };
		const rbx_gfx_layout lc = { 8, 8, 2, pc, xc, yc, 128 };
		const rbx_gfx_layout lt = { 16, 16, 4, p2, xt, yt, 512 }, ls = { 16, 16, 4, p3, xs, ys, 256 };
		if (rbx_gfx_decode(&s_gchar, &lc, chars, s1) || rbx_gfx_decode(&s_gtile, &lt, tiles, s2 / 2) ||
		    rbx_gfx_decode(&s_gspr, &ls, spr, s3 / 4)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			sr_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = 26;   // (0.10 SSG, 0.25 FM)
	yc.gain[3] = 64;
	yc.irq = ym_irq;
	int rc = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	yc.irq = NULL;
	rc = rc || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		sr_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 512)) {
		rbx_fail("%s: out of memory", g->set);
		sr_exit();
		return -1;
	}
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x80; p++)
		s_zmap[p] = s_zrom + p * 256;
	static const char *const names[5] = { "SYSTEM", "P1", "P2", "DSW1", "DSW2" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	sr_reset();
	return 0;
}

static void sr_exit(void)
{
	rbx_video_wait_idle();
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gchar);
	rbx_gfx_free(&s_gtile);
	rbx_gfx_free(&s_gspr);
	free(s_snap);
	s_snap = NULL;
	free(t);
	t = NULL;
}

static void sr_frame(void)
{
	static int32_t tmp[1024];
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0 + VIS_H) {   // vblank rising: the screen, the sprite copy, IRQ
			vblank_capture();
			m6809_hold_irq(&s_main);
		} else if (line == VIS_Y0)      // vblank falling: FIRQ
			m6809_hold_firq(&s_main);
		m6809_run_until(&s_main, t->main_t += rbx_line_cycles(&t->main_frac, MAIN_CLOCK, FRAME_MHZ, LINES));
		z80_run(&s_snd, s_snd.cyc + (unsigned long)rbx_line_cycles(&t->snd_frac, SND_CLOCK, FRAME_MHZ, LINES), &t->ym_irq, 0);
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&t->ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
		s_mix_n += rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
		rbx_ym_advance(1, clocks, tmp, 1024);
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	rbx_ym_async_take(1, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int sr_state(void *ctx, int load)
{
	int rc = 0;
	z80 zs = s_snd;
	rc |= rbx_state_io(ctx, "sr", t, (uint32_t)offsetof(__typeof__(*t), state_end));
	rc |= rbx_state_io(ctx, "sr_main", &s_main, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "sr_snd", &zs, sizeof(zs));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	if (!rc && load) {
		zs.read_byte = snd_read;
		zs.write_byte = snd_write;
		zs.port_in = snd_in;
		zs.port_out = snd_out;
		zs.userdata = NULL;
		zs.read_map = s_zmap;
		s_snd = zs;
		map_banks();
	}
	return rc ? -1 : 0;
}

static int sr_save(void *ctx) { return sr_state(ctx, 0); }
static int sr_load(void *ctx) { return sr_state(ctx, 1); }
static int sr_supported(const char *machine) { return !strcmp(machine, "srumbler"); }

const rbx_driver rbx_drv_srumbler = { "Capcom Speed Rumbler", sr_init, sr_exit, sr_reset, sr_frame, sr_save, sr_load,
                                      sr_supported, sr_render, sr_help };
