/*  Technos Dog Fight (Thunderbolt). Ported from MAME src/mame/technos/dogfgt.cpp (BSD-3-Clause, copyright-holders:
    Nicola Salmoria).
    Two M6502 at 1.5 MHz sharing 2 KB (the main CPU's IRQ 16 times a frame, the sub CPU's from the main CPU), two
    YM2149 written through a latch. Video: a 512x512 tilemap of 16x16, 24 sprites of 16x16, a 3-plane 256x256 bitmap
    (its pixels keep the colour bank they were written with, redrawn when the bank or the flip changes); 16 RAM colours
    (BGR 233) and 64 from a PROM.
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
#define CPU_CLOCK 1500000
#define AY_CLOCK 1500000
#define PIXMAP_BASE (16 + 32)
#define BITMAPRAM_SIZE 0x6000

static uint8_t *s_rom, *s_srom;
static uint32_t s_rom_size, s_srom_size;
static uint32_t s_ppal[64];
static rbx_gfx s_gtile, s_gspr;
static int s_port[5];
static m6502_t s_main, s_sub;
static ay8910_t s_ay[2];
static int32_t s_mix[2048];
static int s_mix_n;
static int s_line;

static struct {
	uint8_t shared[0x800], subram[0x800], spr[0x60], bg[0x800], pal[0x10];
	uint8_t bitmapram[BITMAPRAM_SIZE];
	uint8_t pix[256 * 256];
	uint8_t scroll[4], bm_plane, pixcolor, lastpixcolor, flip, lastflip, control, soundcontrol, latch;
	int64_t main_t, sub_t;
	uint64_t main_frac, sub_frac, ay_frac;
	uint8_t state_end;
} *t;   // (calloc in init: ADR-040)

static struct {
	uint8_t spr[0x60], bg[0x800], pal[0x10], pix[256 * 256];
	uint16_t sx, sy;
	uint8_t flip, pixcolor;
} *s_snap;

// ---- the bitmap ----
static void internal_bitmapram_w(uint32_t offset, uint8_t data)
{
	t->bitmapram[offset] = data;
	offset &= BITMAPRAM_SIZE / 3 - 1;
	const int x = 8 * (int)(offset / 256), y = (int)(offset % 256);
	for (int subx = 0; subx < 8; subx++) {
		int color = 0;
		for (int i = 0; i < 3; i++)
			color |= ((t->bitmapram[offset + BITMAPRAM_SIZE / 3 * i] >> subx) & 1) << i;
		const uint8_t v = (uint8_t)(PIXMAP_BASE + 8 * t->pixcolor + color);
		if (t->flip)
			t->pix[(y ^ 0xff) * 256 + ((x + subx) ^ 0xff)] = v;
		else
			t->pix[y * 256 + x + subx] = v;
	}
}

// ---- main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0x8000) return s_rom[a];
	if (a < 0x800) return t->shared[a];
	if (a >= 0x2000 && a < 0x4000) return t->bm_plane > 2 ? 0 : t->bitmapram[(a - 0x2000) + BITMAPRAM_SIZE / 3 * t->bm_plane];
	switch (a) {
	case 0x1800: return (uint8_t)rbx_port_read(s_port[0]);
	case 0x1810: return (uint8_t)rbx_port_read(s_port[1]);
	case 0x1820: return (uint8_t)rbx_port_read(s_port[2]);
	case 0x1830:
		rbx_port_custom(s_port[3], 0x80, (s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H) ? 0x80u : 0);
		return (uint8_t)rbx_port_read(s_port[3]);
	}
	return 0;
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000) return;
	if (a < 0x800) { t->shared[a] = v; return; }
	if (a >= 0xf80 && a < 0xfe0) { t->spr[a - 0xf80] = v; return; }
	if (a >= 0x1000 && a < 0x1800) { t->bg[a - 0x1000] = v; return; }
	if (a >= 0x1870 && a < 0x1880) { t->pal[a - 0x1870] = v; return; }
	if (a >= 0x2000 && a < 0x4000) {
		if (t->bm_plane <= 2)
			internal_bitmapram_w((a - 0x2000u) + BITMAPRAM_SIZE / 3 * t->bm_plane, v);
		return;
	}
	switch (a) {
	case 0x1800:   // control_w
		t->pixcolor = (uint8_t)(((v & 1) << 1) | ((v >> 1) & 1));
		t->flip = (uint8_t)(((v >> 7) & 1) ^ (~rbx_port_read(s_port[4]) & 1));
		t->control = v;
		break;
	case 0x1810: if (v & 4) m6502_set_irq(&s_sub, 1); break;
	case 0x1820: case 0x1821: case 0x1822: case 0x1823: t->scroll[a - 0x1820] = v; break;
	case 0x1824: t->bm_plane = v; break;
	case 0x1830: t->latch = v; break;
	case 0x1840:   // soundcontrol_w (data_address_w: offset 0 the data)
		if ((t->soundcontrol & 0x20) && !(v & 0x20))
			ay8910_write(&s_ay[0], ~t->soundcontrol >> 4 & 1, t->latch);
		if ((t->soundcontrol & 0x80) && !(v & 0x80))
			ay8910_write(&s_ay[1], ~t->soundcontrol >> 6 & 1, t->latch);
		t->soundcontrol = v;
		break;
	}
}

static uint8_t sub_read(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0x8000) return s_srom[a];
	if (a < 0x800) return t->subram[a];
	if (a >= 0x2000 && a < 0x2800) return t->shared[a - 0x2000];
	return 0;
}
static void sub_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x800) t->subram[a] = v;
	else if (a >= 0x2000 && a < 0x2800) t->shared[a - 0x2000] = v;
	else if (a == 0x4000) m6502_set_irq(&s_sub, 0);
}

// ---- video ----
static inline int pal3(int v) { v &= 7; return (v << 5) | (v << 2) | (v >> 1); }

static void dg_render(void)
{
	for (int i = 0; i < 16; i++) {   // BGR_233
		const int d = s_snap->pal[i];
		rbx_video.palette[i] = (uint32_t)(pal3(d) << 16 | pal3(d >> 3) << 8 | ((d >> 6) & 3) * 0x55);
	}
	memcpy(rbx_video.palette + 16, s_ppal, sizeof(s_ppal));
	const int flip = s_snap->flip;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y + s_snap->sy : y + s_snap->sy) & 511;
		for (int x = 0; x < VIS_W; x++) {
			const uint32_t tx = (uint32_t)(flip ? 255 - x + s_snap->sx : x + s_snap->sx) & 511;
			const uint32_t idx = (ty >> 4) * 32 + (tx >> 4);
			const uint32_t code = s_snap->bg[idx] % s_gtile.count;
			d[x] = (uint16_t)(16 + (s_snap->bg[idx + 0x400] & 3) * 8 + s_gtile.pix[code * 256 + (ty & 15) * 16 + (tx & 15)]);
		}
	}
	for (int o = 0; o < 0x60; o += 4) {
		const uint8_t *s = &s_snap->spr[o];
		if (!(s[0] & 1))
			continue;
		int sx = s[3], sy = (241 - s[2]) & 0xff, fx = (s[0] & 4) != 0, fy = (s[0] & 2) != 0;
		if (flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			fx = !fx;
			fy = !fy;
		}
		tec8_transpen(&s_gspr, (uint32_t)(s[1] + ((s[0] & 0x30) << 4)), (uint16_t)(((s[0] & 8) >> 3) * 8), fx, fy, sx, sy,
		              VIS_Y0, VIS_Y0 + VIS_H, VIS_Y0, VIS_W);
	}
	const uint8_t trans = (uint8_t)(PIXMAP_BASE + 8 * s_snap->pixcolor);   // (copybitmap_trans)
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint8_t *p = &s_snap->pix[y * 256];
		for (int x = 0; x < VIS_W; x++)
			if (p[x] != trans)
				d[x] = p[x];
	}
}

static void vblank_capture(void)
{
	if (t->lastflip != t->flip || t->lastpixcolor != t->pixcolor) {   // (screen_update: the bitmap redrawn)
		t->lastflip = t->flip;
		t->lastpixcolor = t->pixcolor;
		for (uint32_t o = 0; o < BITMAPRAM_SIZE; o++)
			internal_bitmapram_w(o, t->bitmapram[o]);
	}
	rbx_video_wait_idle();
	memcpy(s_snap->spr, t->spr, sizeof(t->spr));
	memcpy(s_snap->bg, t->bg, sizeof(t->bg));
	memcpy(s_snap->pal, t->pal, sizeof(t->pal));
	memcpy(s_snap->pix, t->pix, sizeof(t->pix));
	s_snap->sx = (uint16_t)(t->scroll[0] + 256 * t->scroll[1] + 256);
	s_snap->sy = (uint16_t)(t->scroll[2] + 256 * t->scroll[3]);
	s_snap->flip = t->flip;
	s_snap->pixcolor = t->pixcolor;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void dg_reset(void)
{
	memset(t, 0, offsetof(__typeof__(*t), state_end));
	m6502_init(&s_main, M6502_NMOS);
	s_main.read = main_read;
	s_main.write = main_write;
	s_main.irq_hold = 1;
	for (int p = 0; p < 8; p++)
		s_main.rpage[p] = s_main.wpage[p] = t->shared + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		s_main.rpage[p] = s_rom + p * 256;
	m6502_reset(&s_main);
	m6502_init(&s_sub, M6502_NMOS);
	s_sub.read = sub_read;
	s_sub.write = sub_write;
	for (int p = 0; p < 8; p++) {
		s_sub.rpage[p] = s_sub.wpage[p] = t->subram + p * 256;
		s_sub.rpage[0x20 + p] = s_sub.wpage[0x20 + p] = t->shared + p * 256;
	}
	for (int p = 0x80; p < 0x100; p++)
		s_sub.rpage[p] = s_srom + p * 256;
	m6502_reset(&s_sub);
	for (int i = 0; i < 2; i++)
		ay8910_reset(&s_ay[i]);
	s_mix_n = 0;
	s_line = 0;
}

static void dg_exit(void);

static int dg_init(const rbx_game *g)
{
	free(t);
	t = calloc(1, sizeof(*t));
	if (!t) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t tsize, ssize, psize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("subcpu", &s_srom_size);
	uint8_t *tiles = rbx_region("tiles", &tsize), *spr = rbx_region("sprites", &ssize);
	const uint8_t *prom = rbx_region("proms", &psize);
	if (!s_rom || s_rom_size < 0x10000 || !s_srom || s_srom_size < 0x10000 || !tiles || !spr || !prom || psize < 0x40) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	for (int i = 0; i < 64; i++) {
		const uint8_t c = prom[i];
		const int r = 0x21 * (c & 1) + 0x47 * ((c >> 1) & 1) + 0x97 * ((c >> 2) & 1);
		const int gg = 0x21 * ((c >> 3) & 1) + 0x47 * ((c >> 4) & 1) + 0x97 * ((c >> 5) & 1);
		const int b = 0x47 * ((c >> 6) & 1) + 0x97 * ((c >> 7) & 1);
		s_ppal[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	{	// tilelayout, spritelayout (RGN_FRAC(1,3))
		static const uint32_t xt[16] = { 135, 134, 133, 132, 131, 130, 129, 128, 7, 6, 5, 4, 3, 2, 1, 0 };
		static const uint32_t xs[16] = { 7, 6, 5, 4, 3, 2, 1, 0, 135, 134, 133, 132, 131, 130, 129, 128 };
		static uint32_t yt[16];
		for (int i = 0; i < 16; i++)
			yt[i] = (uint32_t)i * 8;
		const uint32_t ft = tsize / 3 * 8, fs = ssize / 3 * 8;
		const uint32_t pt[3] = { 2 * ft, ft, 0 }, ps[3] = { 2 * fs, fs, 0 };
		const rbx_gfx_layout lt = { 16, 16, 3, pt, xt, yt, 256 }, ls = { 16, 16, 3, ps, xs, yt, 256 };
		if (rbx_gfx_decode(&s_gtile, &lt, tiles, tsize / 3) || rbx_gfx_decode(&s_gspr, &ls, spr, ssize / 3)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			dg_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	for (int i = 0; i < 2; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_YM2149, AY_CLOCK, AY8910_LEGACY_OUTPUT, 77, 4, 0);   // (0.30)
	if (!s_snap || rbx_audio_setup(ay8910_rate(&s_ay[0])) || rbx_video_setup(VIS_W, VIS_H, 16 + 64)) {
		rbx_fail("%s: out of memory", g->set);
		dg_exit();
		return -1;
	}
	static const char *const names[5] = { "P1", "P2", "DSW1", "DSW2", "DSW3" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	dg_reset();
	return 0;
}

static void dg_exit(void)
{
	rbx_video_wait_idle();
	rbx_gfx_free(&s_gtile);
	rbx_gfx_free(&s_gspr);
	free(s_snap);
	s_snap = NULL;
	free(t);
	t = NULL;
}

static void dg_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		if (line % 16 == 8 && line < 256)
			m6502_set_irq(&s_main, 1);
		m6502_run_until(&s_main, t->main_t += rbx_line_cycles(&t->main_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		m6502_run_until(&s_sub, t->sub_t += rbx_line_cycles(&t->sub_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		const uint32_t ac = (uint32_t)rbx_line_cycles(&t->ay_frac, AY_CLOCK, FRAME_MHZ, LINES);
		const int room = 2048 - s_mix_n;
		ay8910_render(&s_ay[0], ac, s_mix + s_mix_n, room);
		s_mix_n += ay8910_render(&s_ay[1], ac, s_mix + s_mix_n, room);
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int dg_state(void *ctx, int load)
{
	(void)load;
	int rc = 0;
	rc |= rbx_state_io(ctx, "dg", t, (uint32_t)offsetof(__typeof__(*t), state_end));
	rc |= rbx_state_io(ctx, "dg_main", &s_main, (uint32_t)offsetof(m6502_t, rpage));
	rc |= rbx_state_io(ctx, "dg_sub", &s_sub, (uint32_t)offsetof(m6502_t, rpage));
	rc |= rbx_state_io(ctx, "dg_ay0", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "dg_ay1", &s_ay[1], ay8910_state_size());
	return rc ? -1 : 0;
}

static int dg_save(void *ctx) { return dg_state(ctx, 0); }
static int dg_load(void *ctx) { return dg_state(ctx, 1); }
static int dg_supported(const char *machine) { return !strcmp(machine, "dogfgt"); }

const rbx_driver rbx_drv_dogfgt = { "Technos Dog Fight", dg_init, dg_exit, dg_reset, dg_frame, dg_save, dg_load,
                                    dg_supported, dg_render, NULL };
