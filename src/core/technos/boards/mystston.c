/*  Technos Mysterious Stones. Ported from MAME src/mame/technos/mystston.cpp (BSD-3-Clause, copyright-holders:
    Nicola Salmoria, Couriersud).
    M6502 at 1.5 MHz (IRQ every 16 lines from line 7, at horizontal position 256; the coins on the NMI), two AY-8910 at
    1.5 MHz written through a latch. Video: a 256x512 tilemap of 16x16 (scrolled vertically, two pages), a 256x256
    text layer, 24 sprites of 16x16; 3 bpp, a resistor-network palette of 32 RAM and 32 PROM colours.
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

static uint8_t *s_rom, s_prom[0x20];
static uint32_t s_rom_size;
static rbx_gfx s_gchar, s_gbg, s_gspr;
static double s_wrg[3], s_wb[2];
static int s_port[4];
static m6502_t s_main;
static ay8910_t s_ay[2];
static int32_t s_mix[2048];
static int s_mix_n;
static int s_line;

static struct {
	uint8_t ram[0x1000], fgram[0x800], bgram[0x800], pal[0x20];
	uint8_t vctrl, scroll, ay_data, ay_select;
	uint8_t coins;
	int64_t main_t;
	uint64_t main_frac, ay_frac;
	uint8_t state_end;
} t;

static struct {
	uint8_t fgram[0x800], bgram[0x800], spr[0x60], pal[0x20];
	uint8_t vctrl, scroll, flip;
} *s_snap;

// ---- main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0x4000) return s_rom[a];
	if (a < 0x1000) return t.ram[a];
	if (a < 0x1800) return t.fgram[a - 0x1000];
	if (a < 0x2000) return t.bgram[a - 0x1800];
	switch ((a >> 4) & 7) {
	case 0: return (uint8_t)rbx_port_read(s_port[0]);
	case 1: return (uint8_t)rbx_port_read(s_port[1]);
	case 2: return (uint8_t)rbx_port_read(s_port[2]);
	case 3:
		rbx_port_custom(s_port[3], 0x80, (s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H) ? 0x80u : 0);
		return (uint8_t)rbx_port_read(s_port[3]);
	case 6: case 7: return t.pal[a & 0x1f];
	}
	return 0;
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000) return;
	if (a < 0x1000) { t.ram[a] = v; return; }
	if (a < 0x1800) { t.fgram[a - 0x1000] = v; return; }
	if (a < 0x2000) { t.bgram[a - 0x1800] = v; return; }
	switch ((a >> 4) & 7) {
	case 0: t.vctrl = v; break;
	case 1: m6502_set_irq(&s_main, 0); break;
	case 2: t.scroll = v; break;
	case 3: t.ay_data = v; break;
	case 4:   // ay8910_select_w
		if ((t.ay_select & 0x20) && !(v & 0x20))
			ay8910_write(&s_ay[0], ~t.ay_select >> 4 & 1, t.ay_data);
		if ((t.ay_select & 0x80) && !(v & 0x80))
			ay8910_write(&s_ay[1], ~t.ay_select >> 6 & 1, t.ay_data);
		t.ay_select = v;
		break;
	case 6: case 7: t.pal[a & 0x1f] = v; break;
	}
}

// ---- video ----
static void ms_render(void)
{
	for (int i = 0; i < 0x40; i++) {   // set_palette
		const uint8_t d = (i & 0x20) ? s_prom[i & 0x1f] : s_snap->pal[i];
		const int r = tec8_combine3(s_wrg, d & 1, (d >> 1) & 1, (d >> 2) & 1);
		const int g = tec8_combine3(s_wrg, (d >> 3) & 1, (d >> 4) & 1, (d >> 5) & 1);
		const int b = tec8_combine2(s_wb, (d >> 6) & 1, (d >> 7) & 1);
		rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	const int flip = s_snap->flip;
	const uint32_t page = (uint32_t)(s_snap->vctrl & 4) << 8;
	const int fgcol = ((s_snap->vctrl & 1) << 1) | ((s_snap->vctrl & 2) >> 1);
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y + s_snap->scroll : y + s_snap->scroll) & 511;
		uint32_t last = 0xffffffffu;
		const uint8_t *trow = NULL;
		for (int x = 0; x < VIS_W; x++) {   // bg: TILEMAP_SCAN_COLS_FLIP_X, 16 x 32 of 16x16
			const uint32_t tx = (uint32_t)(flip ? 255 - x : x) & 255;
			if ((tx >> 4) != last) {   // (a tile looked up once for its 16 pixels)
				last = tx >> 4;
				const uint32_t idx = (15 - (tx >> 4)) * 32 + (ty >> 4);
				const uint32_t code = rbx_wrap((uint32_t)(((s_snap->bgram[page | 0x200 | idx] & 1) << 8) | s_snap->bgram[page | idx]), s_gbg.count);
				const uint32_t py = (idx & 0x10) ? 15 - (ty & 15) : ty & 15;
				trow = s_gbg.pix + code * 256 + py * 16;
			}
			d[x] = (uint16_t)(16 + trow[tx & 15]);
		}
	}
	for (int o = 0; o < 0x60; o += 4) {
		const uint8_t *s = &s_snap->spr[o];
		const int attr = s[0];
		if (!(attr & 1))
			continue;
		const uint32_t code = (uint32_t)(((attr & 0x10) << 4) | s[1]);
		const int color = (attr & 8) >> 3;
		int fx = (attr & 4) != 0, fy = (attr & 2) != 0;
		int x = 240 - s[3], y = (240 - s[2]) & 0xff;
		if (flip) {
			x = 240 - x;
			y = 240 - y;
			fx = !fx;
			fy = !fy;
		}
		tec8_transpen(&s_gspr, code, (uint16_t)(color * 8), fx, fy, x, y, VIS_Y0, VIS_Y0 + VIS_H, VIS_Y0, VIS_W);
	}
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y : y) & 255;
		uint32_t last = 0xffffffffu;
		const uint8_t *trow = NULL;
		for (int x = 0; x < VIS_W; x++) {
			const uint32_t tx = (uint32_t)(flip ? 255 - x : x) & 255;
			if ((tx >> 3) != last) {   // (a character looked up once for its 8 pixels)
				last = tx >> 3;
				const uint32_t idx = (31 - (tx >> 3)) * 32 + (ty >> 3);
				const uint32_t code = rbx_wrap((uint32_t)(((s_snap->fgram[0x400 | idx] & 7) << 8) | s_snap->fgram[idx]), s_gchar.count);
				trow = s_gchar.pix + code * 64 + (ty & 7) * 8;
			}
			const uint8_t p = trow[tx & 7];
			if (p)
				d[x] = (uint16_t)(32 + fgcol * 8 + p);
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->fgram, t.fgram, sizeof(t.fgram));
	memcpy(s_snap->bgram, t.bgram, sizeof(t.bgram));
	memcpy(s_snap->spr, t.ram + 0x780, 0x60);
	memcpy(s_snap->pal, t.pal, sizeof(t.pal));
	s_snap->vctrl = t.vctrl;
	s_snap->scroll = t.scroll;
	s_snap->flip = ((t.vctrl & 0x80) ^ ((rbx_port_read(s_port[3]) & 0x20) << 2)) != 0;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void ms_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	t.coins = 0xc0;
	m6502_init(&s_main, M6502_NMOS);
	s_main.read = main_read;
	s_main.write = main_write;
	for (int p = 0; p < 0x10; p++)
		s_main.rpage[p] = s_main.wpage[p] = t.ram + p * 256;
	for (int p = 0x10; p < 0x18; p++)
		s_main.rpage[p] = s_main.wpage[p] = t.fgram + (p - 0x10) * 256;
	for (int p = 0x18; p < 0x20; p++)
		s_main.rpage[p] = s_main.wpage[p] = t.bgram + (p - 0x18) * 256;
	for (int p = 0x40; p < 0x100; p++)
		s_main.rpage[p] = s_rom + p * 256;
	m6502_reset(&s_main);
	ay8910_reset(&s_ay[0]);
	ay8910_reset(&s_ay[1]);
	s_mix_n = 0;
	s_line = 0;
}

static void ms_exit(void);

static int ms_init(const rbx_game *g)
{
	uint32_t fsize, bsize, psize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *fg = rbx_region("fgtiles_sprites", &fsize), *bg = rbx_region("bgtiles", &bsize), *prom = rbx_region("proms", &psize);
	if (!s_rom || s_rom_size < 0x10000 || !fg || !bg || !prom || psize < 0x20) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memcpy(s_prom, prom, 0x20);
	{	// gfx_8x8x3_planar, spritelayout (RGN_FRAC(1,3))
		static const uint32_t x8[8] = { 0, 1, 2, 3, 4, 5, 6, 7 }, y8[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
		static const uint32_t xs[16] = { 128, 129, 130, 131, 132, 133, 134, 135, 0, 1, 2, 3, 4, 5, 6, 7 };
		static uint32_t ys[16];
		for (int i = 0; i < 16; i++)
			ys[i] = (uint32_t)i * 8;
		const uint32_t ft = fsize / 3 * 8, bt = bsize / 3 * 8;
		const uint32_t pf[3] = { 2 * ft, ft, 0 }, pb[3] = { 2 * bt, bt, 0 };
		const rbx_gfx_layout lc = { 8, 8, 3, pf, x8, y8, 64 };
		const rbx_gfx_layout ls = { 16, 16, 3, pf, xs, ys, 256 }, lb = { 16, 16, 3, pb, xs, ys, 256 };
		if (rbx_gfx_decode(&s_gchar, &lc, fg, fsize / 3) || rbx_gfx_decode(&s_gspr, &ls, fg, fsize / 3) ||
		    rbx_gfx_decode(&s_gbg, &lb, bg, bsize / 3)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			ms_exit();
			return -1;
		}
	}
	static const int rrg[3] = { 4700, 3300, 1500 }, rb[2] = { 3300, 1500 };
	tec8_res_weights(3, rrg, s_wrg, 0, 4700, 2, rb, s_wb, 0, 4700, 0, NULL, NULL, 0, 0);
	s_snap = calloc(1, sizeof(*s_snap));
	for (int i = 0; i < 2; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 77, 4, 0);   // (0.30)
	if (!s_snap || rbx_audio_setup(ay8910_rate(&s_ay[0])) || rbx_video_setup(VIS_W, VIS_H, 0x40)) {
		rbx_fail("%s: out of memory", g->set);
		ms_exit();
		return -1;
	}
	static const char *const names[4] = { "IN0", "IN1", "DSW0", "DSW1" };
	for (int i = 0; i < 4; i++)
		s_port[i] = rbx_port_find(names[i]);
	ms_reset();
	return 0;
}

static void ms_exit(void)
{
	rbx_video_wait_idle();
	rbx_gfx_free(&s_gchar);
	rbx_gfx_free(&s_gbg);
	rbx_gfx_free(&s_gspr);
	free(s_snap);
	s_snap = NULL;
}

static void ms_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	{	// coin_inserted: the NMI follows a coin input that changed
		const uint8_t c = (uint8_t)(rbx_port_read(s_port[0]) & 0xc0), ch = c ^ t.coins;
		for (int b = 6; b < 8; b++)
			if (ch & (1 << b))
				m6502_set_nmi(&s_main, (c >> b) & 1 ? 0 : 1);
		t.coins = c;
	}
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		const int c = rbx_line_cycles(&t.main_frac, MAIN_CLOCK, FRAME_MHZ, LINES);
		const int c1 = c * 2 / 3;   // (the interrupt at hpos 256 of lines 7, 23, ... 247)
		m6502_run_until(&s_main, t.main_t += c1);
		if (line % 16 == 7)
			m6502_set_irq(&s_main, 1);
		m6502_run_until(&s_main, t.main_t += c - c1);
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

static int ms_state(void *ctx, int load)
{
	(void)load;
	int rc = 0;
	rc |= rbx_state_io(ctx, "ms", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "ms_main", &s_main, (uint32_t)offsetof(m6502_t, rpage));
	rc |= rbx_state_io(ctx, "ms_ay0", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "ms_ay1", &s_ay[1], ay8910_state_size());
	return rc ? -1 : 0;
}

static int ms_save(void *ctx) { return ms_state(ctx, 0); }
static int ms_load(void *ctx) { return ms_state(ctx, 1); }
static int ms_supported(const char *machine) { return !strcmp(machine, "mystston"); }

const rbx_driver rbx_drv_mystston = { "Technos Mysterious Stones", ms_init, ms_exit, ms_reset, ms_frame, ms_save,
                                      ms_load, ms_supported, ms_render, NULL };
