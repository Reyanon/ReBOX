/*  Technos Syusse Oozumou. Ported from MAME src/mame/technos/ssozumo.cpp (BSD-3-Clause, copyright-holders:
    Takahiro Nogi).
    M6502 at 1.5 MHz (IRQ held at vblank, the coins on the NMI), M6502 sound CPU at 975 kHz with two YM2149 and an 8-bit
    DAC (16 NMIs a frame when enabled). Video: a 256x512 tilemap of 16x16 scrolled vertically, a 256x256 text layer,
    16x16 sprites over both; 3 bpp, 64 PROM colours and 16 RAM colours.
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
#define SND_CLOCK 975000
#define AY_CLOCK 1500000

static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static uint32_t s_ppal[64];
static rbx_gfx s_gchar, s_gtile, s_gspr;
static int s_port[4];
static m6502_t s_main, s_snd;
static ay8910_t s_ay[2];
static int32_t s_mix[2048];
static int s_mix_n;
static int s_line;

static struct {
	uint8_t ram[0x800], fg[0x800], bg[0x800], pal[0x30];
	uint8_t zram[0x200];
	uint8_t scroll, flip, color_bank, latch, nmi_mask, dac, coins;
	int64_t main_t, snd_t;
	uint64_t main_frac, snd_frac, ay_frac;
	uint8_t state_end;
} t;

static struct {
	uint8_t spr[0x80], fg[0x800], bg[0x800], pal[0x30];
	uint8_t scroll, flip, color_bank;
} *s_snap;

// ---- sound ----
static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x200) return t.zram[a];
	if (a == 0x2007) {
		m6502_set_irq(&s_snd, 0);
		return t.latch;
	}
	if (a >= 0x4000) return s_zrom[a];
	return 0;
}
static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x200) t.zram[a] = v;
	else if (a == 0x2000 || a == 0x2001) ay8910_write(&s_ay[0], ~a & 1, v);   // (data_address_w)
	else if (a == 0x2002 || a == 0x2003) ay8910_write(&s_ay[1], ~a & 1, v);
	else if (a == 0x2004) t.dac = v;
	else if (a == 0x2005) t.nmi_mask = v & 1;
}

// ---- main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0x6000) return s_rom[a];
	if (a < 0x800) return t.ram[a];
	if (a >= 0x2000 && a < 0x2800) return t.fg[a - 0x2000];
	if (a >= 0x3000 && a < 0x3800) return t.bg[a - 0x3000];
	if (a >= 0x4050 && a < 0x4080) return t.pal[a - 0x4050];
	switch (a) {
	case 0x4000: return (uint8_t)rbx_port_read(s_port[0]);
	case 0x4010: return (uint8_t)rbx_port_read(s_port[1]);
	case 0x4020: return (uint8_t)rbx_port_read(s_port[2]);
	case 0x4030:
		rbx_port_custom(s_port[3], 0x80, (s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H) ? 0x80u : 0);
		return (uint8_t)rbx_port_read(s_port[3]);
	}
	return 0;
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x6000) return;
	if (a < 0x800) t.ram[a] = v;
	else if (a >= 0x2000 && a < 0x2800) t.fg[a - 0x2000] = v;
	else if (a >= 0x3000 && a < 0x3800) t.bg[a - 0x3000] = v;
	else if (a >= 0x4050 && a < 0x4080) t.pal[a - 0x4050] = v;
	else switch (a) {
	case 0x4000: t.flip = (v & 0x80) != 0; t.color_bank = v & 3; break;
	case 0x4010: t.latch = v; m6502_set_irq(&s_snd, 1); break;
	case 0x4020: t.scroll = v; break;
	}
}

// ---- video ----
static inline int res4(int v) { return 0x0e * (v & 1) + 0x1f * ((v >> 1) & 1) + 0x43 * ((v >> 2) & 1) + 0x8f * ((v >> 3) & 1); }

static void sz_render(void)
{
	memcpy(rbx_video.palette, s_ppal, sizeof(s_ppal));
	for (int i = 0; i < 16; i++)   // (paletteram_w)
		rbx_video.palette[64 + i] = (uint32_t)(res4(s_snap->pal[i]) << 16 | res4(s_snap->pal[i | 0x10]) << 8 | res4(s_snap->pal[i | 0x20]));
	const int flip = s_snap->flip;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y + s_snap->scroll : y + s_snap->scroll) & 511;
		uint32_t last = 0xffffffffu, cbase = 0;
		const uint8_t *trow = NULL;
		for (int x = 0; x < VIS_W; x++) {   // bg: TILEMAP_SCAN_COLS_FLIP_X, 16 x 32 of 16x16, the lower half flipped
			const uint32_t tx = (uint32_t)(flip ? 255 - x : x) & 255;
			if ((tx >> 4) != last) {   // (a tile looked up once for its 16 pixels)
				last = tx >> 4;
				const uint32_t row = ty >> 4, idx = (15 - (tx >> 4)) * 32 + row;
				const uint8_t c = s_snap->bg[0x200 + idx];
				const uint32_t code = rbx_wrap((uint32_t)(s_snap->bg[idx] + ((c & 8) << 5)), s_gtile.count);
				const uint32_t py = row >= 16 ? 15 - (ty & 15) : ty & 15;
				trow = s_gtile.pix + code * 256 + py * 16;
				cbase = 32 + (uint32_t)((c & 0x30) >> 4) * 8;
			}
			d[x] = (uint16_t)(cbase + trow[tx & 15]);
		}
		const uint32_t fy = (uint32_t)(flip ? 255 - y : y) & 255;
		last = 0xffffffffu;
		for (int x = 0; x < VIS_W; x++) {
			const uint32_t tx = (uint32_t)(flip ? 255 - x : x) & 255;
			if ((tx >> 3) != last) {   // (a character looked up once for its 8 pixels)
				last = tx >> 3;
				const uint32_t idx = (31 - (tx >> 3)) * 32 + (fy >> 3);
				const uint32_t code = rbx_wrap((uint32_t)(s_snap->fg[idx] + 256 * (s_snap->fg[0x400 + idx] & 7)), s_gchar.count);
				trow = s_gchar.pix + code * 64 + (fy & 7) * 8;
			}
			const uint8_t p = trow[tx & 7];
			if (p)
				d[x] = (uint16_t)(s_snap->color_bank * 8 + p);
		}
	}
	for (int o = 0; o < 0x80; o += 4) {
		const uint8_t *s = &s_snap->spr[o];
		if (!(s[0] & 1))
			continue;
		int sx = 239 - s[3], sy = (240 - s[2]) & 0xff, fx = (s[0] & 4) != 0, fy = (s[0] & 2) != 0;
		if (flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			fx = !fx;
			fy = !fy;
		}
		tec8_transpen(&s_gspr, (uint32_t)(s[1] + ((s[0] & 0xf0) << 4)), (uint16_t)(64 + ((s[0] & 8) >> 3) * 8), fx, fy, sx,
		              sy, VIS_Y0, VIS_Y0 + VIS_H, VIS_Y0, VIS_W);
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spr, t.ram + 0x780, 0x80);
	memcpy(s_snap->fg, t.fg, sizeof(t.fg));
	memcpy(s_snap->bg, t.bg, sizeof(t.bg));
	memcpy(s_snap->pal, t.pal, sizeof(t.pal));
	s_snap->scroll = t.scroll;
	s_snap->flip = t.flip;
	s_snap->color_bank = t.color_bank;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void sz_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	t.coins = 0xc0;
	m6502_init(&s_main, M6502_NMOS);
	s_main.read = main_read;
	s_main.write = main_write;
	s_main.irq_hold = 1;
	for (int p = 0; p < 8; p++)
		s_main.rpage[p] = s_main.wpage[p] = t.ram + p * 256;
	for (int p = 0x60; p < 0x100; p++)
		s_main.rpage[p] = s_rom + p * 256;
	m6502_reset(&s_main);
	m6502_init(&s_snd, M6502_NMOS);
	s_snd.read = snd_read;
	s_snd.write = snd_write;
	for (int p = 0; p < 2; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = t.zram + p * 256;
	for (int p = 0x40; p < 0x100; p++)
		s_snd.rpage[p] = s_zrom + p * 256;
	m6502_reset(&s_snd);
	for (int i = 0; i < 2; i++)
		ay8910_reset(&s_ay[i]);
	s_mix_n = 0;
	s_line = 0;
}

static void sz_exit(void);

static int sz_init(const rbx_game *g)
{
	uint32_t fsize, bsize, ssize, psize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *fg = rbx_region("fgtiles", &fsize), *bg = rbx_region("bgtiles", &bsize), *spr = rbx_region("sprites", &ssize);
	const uint8_t *prom = rbx_region("proms", &psize);
	if (!s_rom || s_rom_size < 0x10000 || !s_zrom || s_zrom_size < 0x10000 || !fg || !bg || bsize < 0x6000 || !spr ||
	    ssize < 0x1e000 || !prom || psize < 0x80) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	for (int i = 0; i < 64; i++)
		s_ppal[i] = (uint32_t)(res4(prom[i]) << 16 | res4(prom[i] >> 4) << 8 | res4(prom[64 + i]));
	{	// gfx_8x8x3_planar, tilelayout, spritelayout (planes apart)
		static const uint32_t x8[8] = { 0, 1, 2, 3, 4, 5, 6, 7 }, y8[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
		static const uint32_t xs[16] = { 128, 129, 130, 131, 132, 133, 134, 135, 0, 1, 2, 3, 4, 5, 6, 7 };
		static uint32_t ys[16];
		for (int i = 0; i < 16; i++)
			ys[i] = (uint32_t)i * 8;
		const uint32_t f = fsize / 3 * 8;
		const uint32_t pc[3] = { 2 * f, f, 0 };
		static const uint32_t pt[3] = { 2 * 256 * 256, 256 * 256, 0 }, ps[3] = { 2 * 1280 * 256, 1280 * 256, 0 };
		const rbx_gfx_layout lc = { 8, 8, 3, pc, x8, y8, 64 };
		const rbx_gfx_layout lt = { 16, 16, 3, pt, xs, ys, 256 }, ls = { 16, 16, 3, ps, xs, ys, 256 };
		if (rbx_gfx_decode(&s_gchar, &lc, fg, fsize / 3) || rbx_gfx_decode(&s_gtile, &lt, bg, 256 * 32) ||
		    rbx_gfx_decode(&s_gspr, &ls, spr, 1280 * 32)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			sz_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	for (int i = 0; i < 2; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_YM2149, AY_CLOCK, AY8910_LEGACY_OUTPUT, 77, 4, 0);   // (0.3)
	if (!s_snap || rbx_audio_setup(ay8910_rate(&s_ay[0])) || rbx_video_setup(VIS_W, VIS_H, 80)) {
		rbx_fail("%s: out of memory", g->set);
		sz_exit();
		return -1;
	}
	static const char *const names[4] = { "P1", "P2", "DSW2", "DSW1" };
	for (int i = 0; i < 4; i++)
		s_port[i] = rbx_port_find(names[i]);
	sz_reset();
	return 0;
}

static void sz_exit(void)
{
	rbx_video_wait_idle();
	rbx_gfx_free(&s_gchar);
	rbx_gfx_free(&s_gtile);
	rbx_gfx_free(&s_gspr);
	free(s_snap);
	s_snap = NULL;
}

static void sz_frame(void)
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
		if (line == VIS_Y0 + VIS_H) {
			vblank_capture();
			m6502_set_irq(&s_main, 1);
		}
		if (line % 16 == 8 && line < 256 && t.nmi_mask) {
			m6502_set_nmi(&s_snd, 1);
			m6502_set_nmi(&s_snd, 0);
		}
		m6502_run_until(&s_main, t.main_t += rbx_line_cycles(&t.main_frac, MAIN_CLOCK, FRAME_MHZ, LINES));
		m6502_run_until(&s_snd, t.snd_t += rbx_line_cycles(&t.snd_frac, SND_CLOCK, FRAME_MHZ, LINES));
		const uint32_t ac = (uint32_t)rbx_line_cycles(&t.ay_frac, AY_CLOCK, FRAME_MHZ, LINES);
		const int room = 2048 - s_mix_n;
		ay8910_render(&s_ay[0], ac, s_mix + s_mix_n, room);
		const int n = ay8910_render(&s_ay[1], ac, s_mix + s_mix_n, room);
		const int32_t dac = ((int32_t)t.dac - 128) * 77;   // (DAC_8BIT_R2R, 0.3)
		for (int i = 0; i < n; i++)
			s_mix[s_mix_n + i] += dac;
		s_mix_n += n;
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int sz_state(void *ctx, int load)
{
	(void)load;
	int rc = 0;
	rc |= rbx_state_io(ctx, "sz", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "sz_main", &s_main, (uint32_t)offsetof(m6502_t, rpage));
	rc |= rbx_state_io(ctx, "sz_snd", &s_snd, (uint32_t)offsetof(m6502_t, rpage));
	rc |= rbx_state_io(ctx, "sz_ay0", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "sz_ay1", &s_ay[1], ay8910_state_size());
	return rc ? -1 : 0;
}

static int sz_save(void *ctx) { return sz_state(ctx, 0); }
static int sz_load(void *ctx) { return sz_state(ctx, 1); }
static int sz_supported(const char *machine) { return !strcmp(machine, "ssozumo"); }

const rbx_driver rbx_drv_ssozumo = { "Technos Syusse Oozumou", sz_init, sz_exit, sz_reset, sz_frame, sz_save, sz_load,
                                     sz_supported, sz_render, NULL };
