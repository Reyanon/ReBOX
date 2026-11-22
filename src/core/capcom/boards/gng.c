/*  Capcom Ghosts'n Goblins / Makaimura. Ported from MAME src/mame/capcom/gng.cpp (BSD-3-Clause, copyright-holders:
    Roberto Ventura, Mirko Buffoni, Nicola Salmoria).
    MC6809 at 1.5 MHz (IRQ held at vblank), Z80 sound CPU at 3 MHz (IRQ 240 times a second) with two YM2203 at
    1.5 MHz; an LS259 latch for the flip and the sound CPU's reset. Video: a 512x512 tilemap of 16x16 split in two
    layers (sprites between), a 256x256 text layer, 16x16 sprites from a buffered RAM; RGBx 444 palette RAM.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "cap8.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 59637          // set_raw(12 MHz / 2, 384, 0, 256, 262, 16, 240)
#define LINES 262
#define VIS_Y0 16
#define VIS_H 224
#define VIS_W 256
#define MAIN_CLOCK 1500000
#define SND_CLOCK 3000000
#define YM_CLOCK 1500000

static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static rbx_gfx s_gchar, s_gtile, s_gspr;
static int s_port[5];
static m6809_t s_main;
static z80 s_snd;
static const uint8_t *s_zmap[256];
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static int s_line;

static struct {
	uint8_t ram[0x2000], sprbuf[0x200], fg[0x800], bg[0x800], pal[0x100], palext[0x100];
	uint8_t zram[0x800];
	uint8_t scrollx[2], scrolly[2], bank, latch, mainlatch;
	int snd_irq;
	uint32_t snd_irq_acc;
	int64_t main_t;
	uint64_t main_frac, snd_frac, ym_frac;
	uint8_t state_end;
} t;

static struct {
	uint8_t spr[0x200], fg[0x800], bg[0x800], pal[0x100], palext[0x100];
	uint16_t sx, sy;
	uint8_t flip;
} *s_snap;

static uint8_t z_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

// ---- sound ----
static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_zrom[a];
	if (a >= 0xc000 && a < 0xc800) return t.zram[a - 0xc000];
	if (a == 0xc800) return t.latch;
	return 0;
}
static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xc800) t.zram[a - 0xc000] = v;
	else if (a == 0xe000 || a == 0xe001) rbx_ym_write(0, a & 1, v);
	else if (a == 0xe002 || a == 0xe003) rbx_ym_write(1, a & 1, v);
}

static void snd_start(void)
{
	z80_init(&s_snd);
	s_snd.read_byte = snd_read;
	s_snd.write_byte = snd_write;
	s_snd.port_in = z_in;
	s_snd.port_out = z_out;
	s_snd.read_map = s_zmap;
}

static int snd_held(void) { return !(t.mainlatch & 2); }   // (Q1 inverted to the reset line)

// ---- main CPU ----
static const uint8_t *bank_ptr(void) { return t.bank == 4 ? s_rom + 0x4000 : s_rom + 0x10000 + (uint32_t)(t.bank & 3) * 0x2000; }

static void map_bank(void)
{
	const uint8_t *b = bank_ptr();
	for (int p = 0; p < 0x20; p++)
		s_main.rpage[0x40 + p] = b + p * 256;
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0x6000) return s_rom[a];
	if (a >= 0x4000) return bank_ptr()[a - 0x4000];
	if (a < 0x2000) return t.ram[a];
	if (a < 0x2800) return t.fg[a - 0x2000];
	if (a < 0x3000) return t.bg[a - 0x2800];
	if (a >= 0x3000 && a <= 0x3004) return (uint8_t)rbx_port_read(s_port[a - 0x3000]);
	return 0;
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000) return;
	if (a < 0x2000) { t.ram[a] = v; return; }
	if (a < 0x2800) { t.fg[a - 0x2000] = v; return; }
	if (a < 0x3000) { t.bg[a - 0x2800] = v; return; }
	if (a >= 0x3800 && a < 0x3900) { t.palext[a - 0x3800] = v; return; }
	if (a >= 0x3900 && a < 0x3a00) { t.pal[a - 0x3900] = v; return; }
	if (a >= 0x3d00 && a < 0x3d08) {   // mainlatch (LS259)
		const int bit = a & 7, old = t.mainlatch;
		t.mainlatch = (uint8_t)((t.mainlatch & ~(1 << bit)) | ((v & 1) << bit));
		if (bit == 1 && (old & 2) && !(t.mainlatch & 2)) {   // (Q1 low: the sound CPU and the YMs reset)
			snd_start();
			t.snd_irq = 0;
			rbx_ym_reset(0);
			rbx_ym_reset(1);
		}
		return;
	}
	switch (a) {
	case 0x3a00: t.latch = v; break;
	case 0x3b08: case 0x3b09: t.scrollx[a & 1] = v; break;
	case 0x3b0a: case 0x3b0b: t.scrolly[a & 1] = v; break;
	case 0x3c00: memcpy(t.sprbuf, t.ram + 0x1e00, sizeof(t.sprbuf)); break;
	case 0x3e00: t.bank = v == 4 ? 4 : (v & 3); map_bank(); break;
	}
}

// ---- video ----
static void draw_bg(int front)
{
	const int flip = s_snap->flip;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y + s_snap->sy : y + s_snap->sy) & 511;
		for (int x = 0; x < VIS_W; x++) {   // TILEMAP_SCAN_COLS 32 x 32 of 16x16
			const uint32_t tx = (uint32_t)(flip ? 255 - x + s_snap->sx : x + s_snap->sx) & 511;
			const uint32_t idx = (tx >> 4) * 32 + (ty >> 4);
			const uint8_t attr = s_snap->bg[idx + 0x400];
			const uint32_t code = (uint32_t)(s_snap->bg[idx] + ((attr & 0xc0) << 2)) % s_gtile.count;
			const uint32_t px = (attr & 0x10) ? 15 - (tx & 15) : tx & 15, py = (attr & 0x20) ? 15 - (ty & 15) : ty & 15;
			const uint8_t p = s_gtile.pix[code * 256 + py * 16 + px];
			// (set_transmask: group 0 front 0xff / back 0x00, group 1 front 0x41 / back 0xbe)
			const uint32_t mask = (attr & 8) ? (front ? 0x41u : 0xbeu) : (front ? 0xffu : 0x00u);
			if (!((mask >> p) & 1))
				d[x] = (uint16_t)((attr & 7) * 8 + p);
		}
	}
}

static void gng_render(void)
{
	for (int i = 0; i < 256; i++) {   // RGBx_444 (the ext byte high)
		const uint32_t w = (uint32_t)(s_snap->palext[i] << 8 | s_snap->pal[i]);
		rbx_video.palette[i] = ((w >> 12) & 15) * 17 << 16 | ((w >> 8) & 15) * 17 << 8 | ((w >> 4) & 15) * 17;
	}
	const int flip = s_snap->flip;
	draw_bg(0);
	for (int o = 0x200 - 4; o >= 0; o -= 4) {
		const uint8_t *s = &s_snap->spr[o];
		const int attr = s[1];
		int sx = s[3] - 0x100 * (attr & 1), sy = s[2], fx = (attr & 4) != 0, fy = (attr & 8) != 0;
		if (flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			fx = !fx;
			fy = !fy;
		}
		cap8_transpen(&s_gspr, (uint32_t)(s[0] + ((attr << 2) & 0x300)), (uint16_t)(0x40 + ((attr >> 4) & 3) * 16), fx, fy, sx,
		              sy, 15, VIS_Y0, VIS_Y0 + VIS_H, VIS_Y0, 0, VIS_W);
	}
	draw_bg(1);
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y : y) & 255;
		for (int x = 0; x < VIS_W; x++) {
			const uint32_t tx = (uint32_t)(flip ? 255 - x : x) & 255;
			const uint32_t idx = (ty >> 3) * 32 + (tx >> 3);
			const uint8_t attr = s_snap->fg[idx + 0x400];
			const uint32_t code = (uint32_t)(s_snap->fg[idx] + ((attr & 0xc0) << 2)) % s_gchar.count;
			const uint32_t px = (attr & 0x10) ? 7 - (tx & 7) : tx & 7, py = (attr & 0x20) ? 7 - (ty & 7) : ty & 7;
			const uint8_t p = s_gchar.pix[code * 64 + py * 8 + px];
			if (p != 3)
				d[x] = (uint16_t)(0x80 + (attr & 0x0f) * 4 + p);
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spr, t.sprbuf, sizeof(t.sprbuf));
	memcpy(s_snap->fg, t.fg, sizeof(t.fg));
	memcpy(s_snap->bg, t.bg, sizeof(t.bg));
	memcpy(s_snap->pal, t.pal, sizeof(t.pal));
	memcpy(s_snap->palext, t.palext, sizeof(t.palext));
	s_snap->sx = (uint16_t)(t.scrollx[0] + 256 * t.scrollx[1]);
	s_snap->sy = (uint16_t)(t.scrolly[0] + 256 * t.scrolly[1]);
	s_snap->flip = !(t.mainlatch & 1);   // (Q0 inverted)
	rbx_video_frame_done();
}

// ---- init / frame ----
static void gng_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	for (int i = 0; i < 0x100; i += 4) {   // (machine_reset: the palette RAM filled with greys)
		static const uint8_t v[4] = { 0x00, 0x55, 0xaa, 0xff };
		for (int k = 0; k < 4; k++)
			t.pal[i + k] = t.palext[i + k] = v[k];
	}
	m6809_init(&s_main);
	s_main.read = main_read;
	s_main.write = main_write;
	for (int p = 0; p < 0x20; p++)
		s_main.rpage[p] = s_main.wpage[p] = t.ram + p * 256;
	for (int p = 0x60; p < 0x100; p++)
		s_main.rpage[p] = s_rom + p * 256;
	map_bank();
	m6809_reset(&s_main);
	snd_start();
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	s_mix_n = 0;
	s_line = 0;
}

static void gng_exit(void);

static int gng_init(const rbx_game *g)
{
	uint32_t s1, s2, s3;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *chars = rbx_region("chars", &s1), *tiles = rbx_region("tiles", &s2), *spr = rbx_region("sprites", &s3);
	if (!s_rom || s_rom_size < 0x18000 || !s_zrom || s_zrom_size < 0x8000 || !chars || !tiles || !spr) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{	// charlayout, tilelayout (RGN_FRAC(1,3), planes 2/3 1/3 0), spritelayout (RGN_FRAC(1,2))
		static const uint32_t pc[2] = { 4, 0 }, xc[8] = { 0, 1, 2, 3, 8, 9, 10, 11 };
		static const uint32_t yc[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
		static const uint32_t xt[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 128, 129, 130, 131, 132, 133, 134, 135 };
		static const uint32_t xs[16] = { 0, 1, 2, 3, 8, 9, 10, 11, 256, 257, 258, 259, 264, 265, 266, 267 };
		static uint32_t yt[16], ys[16];
		for (int i = 0; i < 16; i++) {
			yt[i] = (uint32_t)i * 8;
			ys[i] = (uint32_t)i * 16;
		}
		const uint32_t f3 = s2 / 3 * 8, f2 = s3 / 2 * 8;
		const uint32_t pt[3] = { 2 * f3, f3, 0 }, ps[4] = { f2 + 4, f2, 4, 0 };
		const rbx_gfx_layout lc = { 8, 8, 2, pc, xc, yc, 128 };
		const rbx_gfx_layout lt = { 16, 16, 3, pt, xt, yt, 256 }, ls = { 16, 16, 4, ps, xs, ys, 512 };
		if (rbx_gfx_decode(&s_gchar, &lc, chars, s1) || rbx_gfx_decode(&s_gtile, &lt, tiles, s2 / 3) ||
		    rbx_gfx_decode(&s_gspr, &ls, spr, s3 / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			gng_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = 102;   // (SSG 0.40, FM 0.20)
	yc.gain[3] = 51;
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1) || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1)) {
		rbx_fail("%s: out of memory", g->set);
		gng_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 256)) {
		rbx_fail("%s: out of memory", g->set);
		gng_exit();
		return -1;
	}
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x80; p++)
		s_zmap[p] = s_zrom + p * 256;
	static const char *const names[5] = { "SYSTEM", "P1", "P2", "DSW1", "DSW2" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	gng_reset();
	return 0;
}

static void gng_exit(void)
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
}

static void gng_frame(void)
{
	static int32_t tmp[1024];
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {
			vblank_capture();
			m6809_hold_irq(&s_main);   // (irq0_line_hold at vblank)
		}
		t.snd_irq_acc += 240 * 1000;   // (set_periodic_int, 240 Hz)
		if (t.snd_irq_acc >= (uint32_t)FRAME_MHZ * LINES) {
			t.snd_irq_acc -= (uint32_t)FRAME_MHZ * LINES;
			if (!snd_held()) {
				s_snd.int_data = 0xff;
				t.snd_irq = 1;
			}
		}
		m6809_run_until(&s_main, t.main_t += rbx_line_cycles(&t.main_frac, MAIN_CLOCK, FRAME_MHZ, LINES));
		const int cs = rbx_line_cycles(&t.snd_frac, SND_CLOCK, FRAME_MHZ, LINES);
		if (snd_held())
			s_snd.cyc += (unsigned long)cs;
		else
			z80_run(&s_snd, s_snd.cyc + (unsigned long)cs, &t.snd_irq, 1);
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&t.ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
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

static int gng_state_io(void *ctx, int load)
{
	int rc = 0;
	z80 c = s_snd;
	rc |= rbx_state_io(ctx, "gng", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "gng_main", &s_main, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "gng_snd", &c, sizeof(c));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	if (!rc && load) {
		c.read_byte = snd_read;
		c.write_byte = snd_write;
		c.port_in = z_in;
		c.port_out = z_out;
		c.userdata = NULL;
		c.read_map = s_zmap;
		s_snd = c;
		map_bank();
	}
	return rc ? -1 : 0;
}

static int gng_save(void *ctx) { return gng_state_io(ctx, 0); }
static int gng_load(void *ctx) { return gng_state_io(ctx, 1); }
static int gng_supported(const char *machine) { return !strcmp(machine, "gng"); }

const rbx_driver rbx_drv_gng = { "Capcom Ghosts'n Goblins", gng_init, gng_exit, gng_reset, gng_frame, gng_save, gng_load,
                                 gng_supported, gng_render, NULL };
