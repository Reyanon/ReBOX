/*  Technos Tag Team Wrestling / The Big Pro Wrestling!. Ported from MAME src/mame/technos/tagteam.cpp
    (BSD-3-Clause, copyright-holders: Steve Ellenoff, Brad Oliver).
    M6502 at 1.5 MHz (IRQ every 16 lines, the coins on the NMI), M6502 sound CPU at 1 MHz with two AY-8910 and an 8-bit
    DAC (its NMI with the main IRQ when enabled). Video: a 256x256 tilemap of 8x8 (also reachable transposed), 8 sprites
    of 16x16 from the tile RAM; 3 bpp, 32 PROM colours through a resistor network, normalized.
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
#define SND_CLOCK 1000000
#define AY_CLOCK 1500000

static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static uint32_t s_ppal[32];
static rbx_gfx s_gchar, s_gspr;
static int s_port[4];
static m6502_t s_main, s_snd;
static ay8910_t s_ay[2];
static int32_t s_mix[2048];
static int s_mix_n;
static int s_line;

static struct {
	uint8_t ram[0x800], vram[0x400], cram[0x400];
	uint8_t zram[0x400];
	uint8_t palbank, flip, latch, nmi_mask, dac, coins;
	int64_t main_t, snd_t;
	uint64_t main_frac, snd_frac, ay_frac;
	uint8_t state_end;
} t;

static struct {
	uint8_t vram[0x400], cram[0x400];
	uint8_t palbank, flip;
} *s_snap;

// ---- sound ----
static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x400) return t.zram[a];
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
	if (a < 0x400) t.zram[a] = v;
	else if (a == 0x2000 || a == 0x2001) ay8910_write(&s_ay[0], ~a & 1, v);   // (data_address_w)
	else if (a == 0x2002 || a == 0x2003) ay8910_write(&s_ay[1], ~a & 1, v);
	else if (a == 0x2004) t.dac = v;
	else if (a == 0x2005) t.nmi_mask = v & 1;
}

// ---- main CPU ----
static inline uint32_t mirror(uint32_t o) { return 32 * (o % 32) + o / 32; }

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0x8000) return s_rom[a];
	if (a < 0x800) return t.ram[a];
	if (a >= 0x4000 && a < 0x4400) return t.vram[mirror(a - 0x4000u)];
	if (a >= 0x4400 && a < 0x4800) return t.cram[mirror(a - 0x4400u)];
	if (a >= 0x4800 && a < 0x4c00) return t.vram[a - 0x4800];
	if (a >= 0x4c00 && a < 0x5000) return t.cram[a - 0x4c00];
	switch (a) {
	case 0x2000: return (uint8_t)rbx_port_read(s_port[1]);
	case 0x2001: return (uint8_t)rbx_port_read(s_port[0]);
	case 0x2002:
		rbx_port_custom(s_port[2], 0x80, (s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H) ? 0x80u : 0);
		return (uint8_t)rbx_port_read(s_port[2]);
	case 0x2003: return (uint8_t)rbx_port_read(s_port[3]);
	}
	return 0;
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000) return;
	if (a < 0x800) t.ram[a] = v;
	else if (a >= 0x4000 && a < 0x4400) t.vram[mirror(a - 0x4000u)] = v;
	else if (a >= 0x4400 && a < 0x4800) t.cram[mirror(a - 0x4400u)] = v;
	else if (a >= 0x4800 && a < 0x4c00) t.vram[a - 0x4800] = v;
	else if (a >= 0x4c00 && a < 0x5000) t.cram[a - 0x4c00] = v;
	else switch (a) {
	case 0x2000: t.flip = v & 1; break;
	case 0x2001: t.palbank = (v & 0x80) >> 7; break;
	case 0x2002: t.latch = v; m6502_set_irq(&s_snd, 1); break;
	case 0x2003: m6502_set_irq(&s_main, 0); break;
	}
}

// ---- video ----
static void tt_render(void)
{
	memcpy(rbx_video.palette, s_ppal, sizeof(s_ppal));
	const int flip = s_snap->flip;
	const uint16_t cbase = (uint16_t)((s_snap->palbank << 1) * 8);
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y : y) & 255;
		for (int x = 0; x < VIS_W; x++) {   // TILEMAP_SCAN_ROWS_FLIP_X
			const uint32_t tx = (uint32_t)(flip ? 255 - x : x) & 255;
			const uint32_t idx = (ty >> 3) * 32 + (31 - (tx >> 3));
			const uint32_t code = rbx_wrap((uint32_t)(s_snap->vram[idx] + 256 * s_snap->cram[idx]), s_gchar.count);
			d[x] = (uint16_t)(cbase + s_gchar.pix[code * 64 + (ty & 7) * 8 + (tx & 7)]);
		}
	}
	for (int o = 0; o < 0x20; o += 4) {
		const uint8_t *v = &s_snap->vram[o];
		const uint32_t bank = (uint32_t)(v[0] & 0x30) << 4;
		int fx = (v[0] & 4) != 0, fy = (v[0] & 2) != 0;
		int sx = 240 - v[3], sy = 240 - v[2];
		if (!(v[0] & 1))
			continue;
		if (flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			fx = !fx;
			fy = !fy;
		}
		tec8_transpen(&s_gspr, v[1] + 256 * bank, (uint16_t)(((s_snap->palbank << 1) | 1) * 8), fx, fy, sx, sy, VIS_Y0,
		              VIS_Y0 + VIS_H, VIS_Y0, VIS_W);
		sy += flip ? -256 : 256;
		tec8_transpen(&s_gspr, v[0x20] + 256 * bank, (uint16_t)(s_snap->palbank * 8), fx, fy, sx, sy, VIS_Y0,
		              VIS_Y0 + VIS_H, VIS_Y0, VIS_W);
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->vram, t.vram, sizeof(t.vram));
	memcpy(s_snap->cram, t.cram, sizeof(t.cram));
	s_snap->palbank = t.palbank;
	s_snap->flip = t.flip;
	rbx_video_frame_done();
}

// ---- the palette: compute_res_net (RES_NET_VCC_5V | RES_NET_VIN_TTL_OUT, no amplifier), normalize_range(0, 31) ----
static int res_net(int inputs, int n, const double *r)
{
	double rtotal = 0, v = 0;
	for (int i = 0; i < n; i++)
		if (!((inputs >> i) & 1)) {
			rtotal += 1.0 / r[i];
			v += 0.05 / r[i];   // (TTL_VOL)
		}
	const int open = rtotal != 0 && v / rtotal > 4.0;
	for (int i = 0; i < n; i++)
		if (((inputs >> i) & 1) && !open) {
			rtotal += 1.0 / (r[i] + 50);
			v += 4.0 / (r[i] + 50);   // (TTL_VOH, the TTL output's 50 ohms)
		}
	rtotal = 1.0 / rtotal;
	v *= rtotal;
	if (v < 0)
		v = 0;
	return (int)(v * 255 / 5.0 + 0.4);
}

static inline int clamp255(int v) { return v < 0 ? 0 : v > 255 ? 255 : v; }

static void make_palette(const uint8_t *prom)
{
	static const double r3[3] = { 4700, 3300, 1500 }, r2[2] = { 3300, 1500 };
	int rr[32], gg[32], bb[32];
	int32_t ymin = 1000 * 255, ymax = 0;
	for (int i = 0; i < 32; i++) {
		rr[i] = res_net(prom[i] & 7, 3, r3);
		gg[i] = res_net((prom[i] >> 3) & 7, 3, r3);
		bb[i] = res_net((prom[i] >> 6) & 3, 2, r2);
		const int32_t y = 299 * rr[i] + 587 * gg[i] + 114 * bb[i];
		if (y < ymin) ymin = y;
		if (y > ymax) ymax = y;
	}
	const int32_t tmin = 0, tmax = 255;
	for (int i = 0; i < 32; i++) {
		const int32_t y = 299 * rr[i] + 587 * gg[i] + 114 * bb[i];
		const int32_t u = (bb[i] - y / 1000) * 492 / 1000, v = (rr[i] - y / 1000) * 877 / 1000;
		const int32_t target = ymax > ymin ? tmin + ((y - ymin) * (tmax - tmin + 1)) / (ymax - ymin) : tmin;
		const int r = clamp255(target + 1140 * v / 1000), g = clamp255(target - 395 * u / 1000 - 581 * v / 1000);
		const int b = clamp255(target + 2032 * u / 1000);
		s_ppal[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
}

// ---- init / frame ----
static void tt_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	t.coins = 0xc0;
	m6502_init(&s_main, M6502_NMOS);
	s_main.read = main_read;
	s_main.write = main_write;
	for (int p = 0; p < 8; p++)
		s_main.rpage[p] = s_main.wpage[p] = t.ram + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		s_main.rpage[p] = s_rom + p * 256;
	m6502_reset(&s_main);
	m6502_init(&s_snd, M6502_NMOS);
	s_snd.read = snd_read;
	s_snd.write = snd_write;
	for (int p = 0; p < 4; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = t.zram + p * 256;
	for (int p = 0x40; p < 0x100; p++)
		s_snd.rpage[p] = s_zrom + p * 256;
	m6502_reset(&s_snd);
	for (int i = 0; i < 2; i++)
		ay8910_reset(&s_ay[i]);
	s_mix_n = 0;
	s_line = 0;
}

static void tt_exit(void);

static int tt_init(const rbx_game *g)
{
	uint32_t gsize, psize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *gfx = rbx_region("gfx", &gsize);
	const uint8_t *prom = rbx_region("proms", &psize);
	if (!s_rom || s_rom_size < 0x10000 || !s_zrom || s_zrom_size < 0x10000 || !gfx || gsize < 0x12000 || !prom || psize < 0x20) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	make_palette(prom);
	{	// gfx_8x8x3_planar, spritelayout (planes apart)
		static const uint32_t x8[8] = { 0, 1, 2, 3, 4, 5, 6, 7 }, y8[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
		static const uint32_t xs[16] = { 128, 129, 130, 131, 132, 133, 134, 135, 0, 1, 2, 3, 4, 5, 6, 7 };
		static uint32_t ys[16];
		for (int i = 0; i < 16; i++)
			ys[i] = (uint32_t)i * 8;
		const uint32_t f = gsize / 3 * 8;
		const uint32_t pc[3] = { 2 * f, f, 0 };
		static const uint32_t ps[3] = { 2 * 768 * 256, 768 * 256, 0 };
		const rbx_gfx_layout lc = { 8, 8, 3, pc, x8, y8, 64 }, ls = { 16, 16, 3, ps, xs, ys, 256 };
		if (rbx_gfx_decode(&s_gchar, &lc, gfx, gsize / 3) || rbx_gfx_decode(&s_gspr, &ls, gfx, 768 * 32)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			tt_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	for (int i = 0; i < 2; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 64, 4, 0);   // (0.25)
	if (!s_snap || rbx_audio_setup(ay8910_rate(&s_ay[0])) || rbx_video_setup(VIS_W, VIS_H, 32)) {
		rbx_fail("%s: out of memory", g->set);
		tt_exit();
		return -1;
	}
	static const char *const names[4] = { "P1", "P2", "DSW1", "DSW2" };
	for (int i = 0; i < 4; i++)
		s_port[i] = rbx_port_find(names[i]);
	tt_reset();
	return 0;
}

static void tt_exit(void)
{
	rbx_video_wait_idle();
	rbx_gfx_free(&s_gchar);
	rbx_gfx_free(&s_gspr);
	free(s_snap);
	s_snap = NULL;
}

static void tt_frame(void)
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
		if (line % 16 == 8 && line <= 256) {   // v8_timer_irq
			m6502_set_irq(&s_main, 1);
			if (t.nmi_mask) {
				m6502_set_nmi(&s_snd, 1);
				m6502_set_nmi(&s_snd, 0);
			}
		}
		m6502_run_until(&s_main, t.main_t += rbx_line_cycles(&t.main_frac, MAIN_CLOCK, FRAME_MHZ, LINES));
		m6502_run_until(&s_snd, t.snd_t += rbx_line_cycles(&t.snd_frac, SND_CLOCK, FRAME_MHZ, LINES));
		const uint32_t ac = (uint32_t)rbx_line_cycles(&t.ay_frac, AY_CLOCK, FRAME_MHZ, LINES);
		const int room = 2048 - s_mix_n;
		ay8910_render(&s_ay[0], ac, s_mix + s_mix_n, room);
		const int n = ay8910_render(&s_ay[1], ac, s_mix + s_mix_n, room);
		const int32_t dac = ((int32_t)t.dac - 128) * 64;   // (DAC_8BIT_R2R, 0.25)
		for (int i = 0; i < n; i++)
			s_mix[s_mix_n + i] += dac;
		s_mix_n += n;
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int tt_state(void *ctx, int load)
{
	(void)load;
	int rc = 0;
	rc |= rbx_state_io(ctx, "tt", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "tt_main", &s_main, (uint32_t)offsetof(m6502_t, rpage));
	rc |= rbx_state_io(ctx, "tt_snd", &s_snd, (uint32_t)offsetof(m6502_t, rpage));
	rc |= rbx_state_io(ctx, "tt_ay0", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "tt_ay1", &s_ay[1], ay8910_state_size());
	return rc ? -1 : 0;
}

static int tt_save(void *ctx) { return tt_state(ctx, 0); }
static int tt_load(void *ctx) { return tt_state(ctx, 1); }
static int tt_supported(const char *machine) { return !strcmp(machine, "tagteam"); }

const rbx_driver rbx_drv_tagteam = { "Technos Tag Team Wrestling", tt_init, tt_exit, tt_reset, tt_frame, tt_save,
                                     tt_load, tt_supported, tt_render, NULL };
