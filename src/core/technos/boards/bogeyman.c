/*  Technos Bogey Manor. Ported from MAME src/mame/technos/bogeyman.cpp (BSD-3-Clause, copyright-holders: Bryan
    McPhail).
    M6502 at 1.5 MHz (16 IRQs a frame), two YM2149 at 1.5 MHz written through a latch (the first one's port A: the text
    colour bank). Video: a 256x256 tilemap of 16x16 (four tile sets), a 256x256 text layer, 16x16 sprites; 16 RAM
    colours (BGR 233 inverted) and 256 from PROMs.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../sound/ay8910.h"
#include "../../video/gfxlayout.h"
#include "tec8.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 57445          // set_raw(12 MHz / 2, 384, 0, 256, 272, 8, 248)
#define LINES 272
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 256
#define MAIN_CLOCK 1500000
#define AY_CLOCK 1500000

static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint32_t s_ppal[256];
static rbx_gfx s_gchar[2], s_gspr, s_gtile[4];
static int s_port[4];
static m6502_t s_main;
static ay8910_t s_ay[2];
static int32_t s_mix[2048];
static int s_mix_n;
static int s_line;

static struct {
	uint8_t ram[0x1800], fg[0x800], bg[0x200], spr[0x400], pal[0x10];
	uint8_t psg_latch, last_write, colbank, flip;
	int64_t main_t;
	uint64_t main_frac, ay_frac;
	uint8_t state_end;
} t;

static struct {
	uint8_t fg[0x800], bg[0x200], spr[0x400], pal[0x10];
	uint8_t colbank, flip;
} *s_snap;

static void colbank_w(void *u, uint8_t v)
{
	(void)u;
	t.colbank = v & 1;
}

// ---- main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0x4000) return s_rom[a];
	if (a < 0x1800) return t.ram[a];
	if (a < 0x2000) return t.fg[a - 0x1800];
	if (a < 0x2200) return t.bg[a - 0x2000];
	if (a >= 0x2800 && a < 0x2c00) return t.spr[a - 0x2800];
	if (a >= 0x3000 && a < 0x3010) return t.pal[a - 0x3000];
	if (a >= 0x3800 && a < 0x3804) {
		if (a == 0x3801)
			rbx_port_custom(s_port[1], 0x80, (s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H) ? 0x80u : 0);
		return (uint8_t)rbx_port_read(s_port[a - 0x3800]);
	}
	return 0;
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000) return;
	if (a < 0x1800) t.ram[a] = v;
	else if (a < 0x2000) t.fg[a - 0x1800] = v;
	else if (a < 0x2200) t.bg[a - 0x2000] = v;
	else if (a >= 0x2800 && a < 0x2c00) t.spr[a - 0x2800] = v;
	else if (a >= 0x3000 && a < 0x3010) t.pal[a - 0x3000] = v;
	else if (a == 0x3800) {   // ay8910_control_w
		t.flip = ~v & 1;
		if ((t.last_write & 0x20) && !(v & 0x20))
			ay8910_write(&s_ay[0], ~t.last_write >> 4 & 1, t.psg_latch);
		if ((t.last_write & 0x80) && !(v & 0x80))
			ay8910_write(&s_ay[1], ~t.last_write >> 6 & 1, t.psg_latch);
		t.last_write = v;
	} else if (a == 0x3801)
		t.psg_latch = v;
}

// ---- video ----
static inline int pal3(int v) { v &= 7; return (v << 5) | (v << 2) | (v >> 1); }

static void bm_render(void)
{
	for (int i = 0; i < 16; i++) {   // BGR_233_inverted
		const int d = ~s_snap->pal[i] & 0xff;
		rbx_video.palette[i] = (uint32_t)(pal3(d) << 16 | pal3(d >> 3) << 8 | ((d >> 6) & 3) * 0x55);
	}
	memcpy(rbx_video.palette + 16, s_ppal, sizeof(s_ppal));
	const int flip = s_snap->flip;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y : y) & 255;
		uint32_t last = 0xffffffffu, cbase = 0;
		const uint8_t *trow = NULL;
		for (int x = 0; x < VIS_W; x++) {
			const uint32_t tx = (uint32_t)(flip ? 255 - x : x) & 255;
			if ((tx >> 4) != last) {   // (a tile looked up once for its 16 pixels)
				last = tx >> 4;
				const uint32_t idx = (ty >> 4) * 16 + (tx >> 4);
				const int attr = s_snap->bg[0x100 + idx];
				const int bank = ((((attr & 1) << 8) + s_snap->bg[idx]) / 0x80);
				const rbx_gfx *g = &s_gtile[bank & 3];
				const uint32_t code = rbx_wrap((uint32_t)(s_snap->bg[idx] & 0x7f), g->count);
				trow = g->pix + code * 256 + (ty & 15) * 16;
				cbase = 16 + 128 + (uint32_t)((attr >> 1) & 7) * 8;
			}
			d[x] = (uint16_t)(cbase + trow[tx & 15]);
		}
	}
	for (int o = 0; o < 0x400; o += 4) {
		const uint8_t *s = &s_snap->spr[o];
		const int attr = s[0];
		if (!(attr & 1))
			continue;
		const uint32_t code = (uint32_t)(s[1] + ((attr & 0x40) << 2));
		const uint16_t base = (uint16_t)(((attr & 8) >> 3) * 8);
		int fx = !(attr & 4), fy = (attr & 2) != 0;
		int sx = s[3], sy = (240 - s[2]) & 0xff;
		const int multi = attr & 0x10;
		if (multi)
			sy -= 16;
		if (flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			fx = !fx;
			fy = !fy;
		}
		tec8_transpen(&s_gspr, code, base, fx, fy, sx, sy, VIS_Y0, VIS_Y0 + VIS_H, VIS_Y0, VIS_W);
		if (multi)
			tec8_transpen(&s_gspr, code + 1, base, fx, fy, sx, sy + (flip ? -16 : 16), VIS_Y0, VIS_Y0 + VIS_H, VIS_Y0, VIS_W);
	}
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y : y) & 255;
		for (int x = 0; x < VIS_W; x++) {
			const uint32_t tx = (uint32_t)(flip ? 255 - x : x) & 255;
			const uint32_t idx = (ty >> 3) * 32 + (tx >> 3);
			const uint32_t tile = s_snap->fg[idx] | (uint32_t)(s_snap->fg[0x400 + idx] & 3) << 8;
			const rbx_gfx *g = &s_gchar[(tile >> 9) & 1];
			const uint8_t p = g->pix[(rbx_wrap((tile & 0x1ff), g->count)) * 64 + (ty & 7) * 8 + (tx & 7)];
			if (p)
				d[x] = (uint16_t)(16 + s_snap->colbank * 8 + p);
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->fg, t.fg, sizeof(t.fg));
	memcpy(s_snap->bg, t.bg, sizeof(t.bg));
	memcpy(s_snap->spr, t.spr, sizeof(t.spr));
	memcpy(s_snap->pal, t.pal, sizeof(t.pal));
	s_snap->colbank = t.colbank;
	s_snap->flip = t.flip;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void bm_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	m6502_init(&s_main, M6502_NMOS);
	s_main.read = main_read;
	s_main.write = main_write;
	s_main.irq_hold = 1;
	for (int p = 0; p < 0x18; p++)
		s_main.rpage[p] = s_main.wpage[p] = t.ram + p * 256;
	for (int p = 0x40; p < 0x100; p++)
		s_main.rpage[p] = s_rom + p * 256;
	m6502_reset(&s_main);
	for (int i = 0; i < 2; i++)
		ay8910_reset(&s_ay[i]);
	s_mix_n = 0;
	s_line = 0;
}

static void bm_exit(void);

static int bm_init(const rbx_game *g)
{
	uint32_t csize, ssize, tsize, psize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *chars = rbx_region("chars", &csize), *spr = rbx_region("sprites", &ssize), *tiles = rbx_region("tiles", &tsize);
	const uint8_t *prom = rbx_region("proms", &psize);
	if (!s_rom || s_rom_size < 0x10000 || !chars || csize < 0x10000 || !spr || ssize < 0xc000 || !tiles || tsize < 0x10000 ||
	    !prom || psize < 0x200) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	for (int i = 0; i < 256; i++) {
		const uint8_t c = prom[i], c2 = prom[256 + i];
		const int r = 0x21 * (c & 1) + 0x47 * ((c >> 1) & 1) + 0x97 * ((c >> 2) & 1);
		const int gg = 0x21 * ((c >> 3) & 1) + 0x47 * (c2 & 1) + 0x97 * ((c2 >> 1) & 1);
		const int b = 0x47 * ((c2 >> 2) & 1) + 0x97 * ((c2 >> 3) & 1);
		s_ppal[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	{	// charlayout1 / 2, sprites, tiles1a / 1b
		static const uint32_t xc[8] = { 0x2000 * 8 + 3, 0x2000 * 8 + 2, 0x2000 * 8 + 1, 0x2000 * 8 + 0, 3, 2, 1, 0 };
		static const uint32_t yc[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
		static const uint32_t pc1[3] = { 0x8000 * 8 + 4, 0, 4 }, pc2[3] = { 0x8000 * 8, 0x1000 * 8, 0x1000 * 8 + 4 };
		static const uint32_t xt[16] = { 1024 * 64 + 3, 1024 * 64 + 2, 1024 * 64 + 1, 1024 * 64 + 0, 3, 2, 1, 0,
		                                 1024 * 64 + 3 + 64, 1024 * 64 + 2 + 64, 1024 * 64 + 1 + 64, 1024 * 64 + 0 + 64, 67, 66, 65, 64 };
		static const uint32_t yt[16] = { 0, 8, 16, 24, 32, 40, 48, 56, 128, 136, 144, 152, 160, 168, 176, 184 };
		static const uint32_t pta[3] = { 0x8000 * 8 + 4, 0, 4 }, ptb[3] = { 0x8000 * 8, 0x1000 * 8, 0x1000 * 8 + 4 };
		static const uint32_t xs[16] = { 128, 129, 130, 131, 132, 133, 134, 135, 0, 1, 2, 3, 4, 5, 6, 7 };
		static uint32_t ys[16];
		for (int i = 0; i < 16; i++)
			ys[i] = (uint32_t)i * 8;
		static const uint32_t ps[3] = { 0x8000 * 8, 0x4000 * 8, 0 };
		const rbx_gfx_layout l1 = { 8, 8, 3, pc1, xc, yc, 64 }, l2 = { 8, 8, 3, pc2, xc, yc, 64 };
		const rbx_gfx_layout la = { 16, 16, 3, pta, xt, yt, 256 }, lb = { 16, 16, 3, ptb, xt, yt, 256 };
		const rbx_gfx_layout ls = { 16, 16, 3, ps, xs, ys, 256 };
		int rc = rbx_gfx_decode(&s_gchar[0], &l1, chars, 512 * 8) || rbx_gfx_decode(&s_gchar[1], &l2, chars, 512 * 8) ||
		         rbx_gfx_decode(&s_gspr, &ls, spr, 512 * 32);
		for (int i = 0; i < 4 && !rc; i++)
			rc = rbx_gfx_decode(&s_gtile[i], (i & 1) ? &lb : &la, tiles + (i >> 1) * 0x4000, 128 * 32);
		if (rc) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			bm_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	for (int i = 0; i < 2; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_YM2149, AY_CLOCK, AY8910_LEGACY_OUTPUT, 77, 4, 0);   // (0.30)
	s_ay[0].port_a_write = colbank_w;
	if (!s_snap || rbx_audio_setup(ay8910_rate(&s_ay[0])) || rbx_video_setup(VIS_W, VIS_H, 16 + 256)) {
		rbx_fail("%s: out of memory", g->set);
		bm_exit();
		return -1;
	}
	static const char *const names[4] = { "P1", "P2", "DSW1", "DSW2" };
	for (int i = 0; i < 4; i++)
		s_port[i] = rbx_port_find(names[i]);
	bm_reset();
	return 0;
}

static void bm_exit(void)
{
	rbx_video_wait_idle();
	rbx_gfx_free(&s_gchar[0]);
	rbx_gfx_free(&s_gchar[1]);
	rbx_gfx_free(&s_gspr);
	for (int i = 0; i < 4; i++)
		rbx_gfx_free(&s_gtile[i]);
	free(s_snap);
	s_snap = NULL;
}

static void bm_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		if (line % 16 == 8 && line < 256)
			m6502_set_irq(&s_main, 1);
		m6502_run_until(&s_main, t.main_t += rbx_line_cycles(&t.main_frac, MAIN_CLOCK, FRAME_MHZ, LINES));
		const uint32_t ac = (uint32_t)rbx_line_cycles(&t.ay_frac, AY_CLOCK, FRAME_MHZ, LINES);
		const int room = 2048 - s_mix_n;
		ay8910_render(&s_ay[0], ac, s_mix + s_mix_n, room);
		s_mix_n += ay8910_render(&s_ay[1], ac, s_mix + s_mix_n, room);
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int bm_state(void *ctx, int load)
{
	(void)load;
	int rc = 0;
	rc |= rbx_state_io(ctx, "bm", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "bm_main", &s_main, (uint32_t)offsetof(m6502_t, rpage));
	rc |= rbx_state_io(ctx, "bm_ay0", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "bm_ay1", &s_ay[1], ay8910_state_size());
	return rc ? -1 : 0;
}

static int bm_save(void *ctx) { return bm_state(ctx, 0); }
static int bm_load(void *ctx) { return bm_state(ctx, 1); }
static int bm_supported(const char *machine) { return !strcmp(machine, "bogeyman"); }

const rbx_driver rbx_drv_bogeyman = { "Technos Bogey Manor", bm_init, bm_exit, bm_reset, bm_frame, bm_save, bm_load,
                                      bm_supported, bm_render, NULL };
