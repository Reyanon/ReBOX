/*  Seibu Kaihatsu Mustache Boy: a Z80 (SEI80BU-encrypted ROM), a 512x256 8x8 tilemap (three row bands scrolled),
    16x16 sprites, the T5182 sound MCU with a YM2151; 256 colors from PROMs.
    Ported from MAME 0.289 src/mame/seibu/mustache.cpp (BSD-3-Clause, copyright-holders: Tomasz Slanina).
    This file is part of ReBOX. GPL-2.0-or-later.

    The CPUs interleaved a scanline at a time. Not done: the flip screen.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../video/gfxlayout.h"
#include "../machine/seibusnd.h"
#include "../machine/smix.h"
#include "../machine/t5182.h"
#include "../video/seibu_draw.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 56747
#define LINES 256
#define VIS_X0 8
#define VIS_W 240
#define VIS_H 248
#define VBLANK_LINES 36   // (2500 us)
#define CPU_CLOCK 6000000

static uint8_t *s_rom, *s_data, *s_op;
static uint32_t s_pens[256];
static rbx_gfx s_gtile, s_gspr;
static int s_port[5];
static z80 s_main;
static smix_t *s_mix;

static struct {
	uint8_t video[0x1000], spr[0x800], ram[0x1000];
	uint8_t scroll, videoctrl;
	int main_irq;
	uint64_t main_frac, snd_frac, ym_frac;
	uint8_t state_end;
} t;

static struct {
	uint8_t video[0x1000], spr[0x800], scroll, videoctrl;
} *s_snap;

static uint32_t bsw(uint32_t v, int n, const int *bits)
{
	uint32_t o = 0;
	for (int i = 0; i < n; i++)
		o = o << 1 | ((v >> bits[i]) & 1);
	return o;
}

// ---- the main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xc000) return s_data[a];
	if (a < 0xd000) return t.video[a - 0xc000];
	if (a == 0xd001) return t5182_semaphore_snd_r();
	if (a >= 0xd400 && a < 0xd500) return t5182_sharedram_r(a & 0xff);
	if (a >= 0xd800 && a <= 0xd804) return (uint8_t)rbx_port_read(s_port[a - 0xd800]);
	if (a >= 0xf000) return t.ram[a - 0xf000];
	return 0xff;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xd000) { t.video[a - 0xc000] = v; return; }
	if (a >= 0xd400 && a < 0xd500) { t5182_sharedram_w(a & 0xff, v); return; }
	if (a >= 0xe800 && a < 0xf000) { t.spr[a - 0xe800] = v; return; }
	if (a >= 0xf000) { t.ram[a - 0xf000] = v; return; }
	switch (a) {
	case 0xd000: t5182_sound_irq_w(); break;
	case 0xd002: t5182_semaphore_main_acquire_w(); break;
	case 0xd003: t5182_semaphore_main_release_w(); break;
	case 0xd806: t.scroll = v; break;
	case 0xd807: t.videoctrl = v; break;
	default: break;
	}
}

static uint8_t z_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

// ---- the drawing ----
static void mu_render(void)
{
	memcpy(rbx_video.palette, s_pens, sizeof(s_pens));
	for (int y = 0; y < VIS_H; y++) {   // TILEMAP_SCAN_ROWS_FLIP_X, 64 x 32; rows 0-191 scrolled, the last band not
		uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
		const int sx = (y >> 6) == 3 ? 0x100 : 0x100 - s_snap->scroll;
		for (int x = 0; x < VIS_W; x++) {
			const int tx = (x + VIS_X0 + sx) & 511;
			const int idx = (y >> 3) * 64 + (63 - (tx >> 3));
			const uint8_t attr = s_snap->video[2 * idx + 1];
			const uint32_t code = (uint32_t)(s_snap->video[2 * idx] + ((attr & 0x60) << 3) + ((s_snap->videoctrl & 0x08) << 7));
			const int px = (attr & 0x10) ? 7 - (tx & 7) : tx & 7, py = (attr & 0x80) ? 7 - (y & 7) : y & 7;
			d[x] = (uint16_t)((attr & 0x0f) * 8 + sd_elem(&s_gtile, code)[py * 8 + px]);
		}
	}
	const int h = (s_snap->videoctrl & 0x0a) ? VIS_H : VIS_H - 56;
	const sd_target tg = { rbx_video.bitmap, rbx_video.pitch, VIS_X0, 0, VIS_W, h, NULL };
	for (int offs = 0; offs < 0x800; offs += 4) {
		const uint8_t *s = &s_snap->spr[offs];
		const int sy = 240 - s[0], sx = 240 - s[3];
		if (sy == 240)
			continue;
		const uint32_t code = (uint32_t)(s[2] + ((s[1] & 0x0c) << 6));
		sd_transpen(&tg, &s_gspr, code, 0x80 + (uint32_t)((s[1] & 0xe0) >> 5) * 16, 0, 0, sx, sy, 0);
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->video, t.video, sizeof(t.video));
	memcpy(s_snap->spr, t.spr, sizeof(t.spr));
	s_snap->scroll = t.scroll;
	s_snap->videoctrl = t.videoctrl;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void mu_reset(void)
{
	rbx_video_wait_idle();
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	z80_init(&s_main);
	s_main.read_byte = main_read;
	s_main.write_byte = main_write;
	s_main.port_in = z_in;
	s_main.port_out = z_out;
	s_main.op_rom = s_op;
	t5182_reset();
	s_mix->n = 0;
	s_mix->frac = 0;
}

static void mu_exit(void);

static int mu_init(const rbx_game *g)
{
	uint32_t rs, ts, ss, ps, cs, es;
	s_rom = rbx_region("maincpu", &rs);
	uint8_t *tiles = rbx_region("tiles", &ts), *spr = rbx_region("sprites", &ss);
	const uint8_t *prom = rbx_region("proms", &ps), *t5cpu = rbx_region("t5182:cpu", &cs),
	              *t5ext = rbx_region("t5182:external", &es);
	if (!s_rom || rs < 0xc000 || !tiles || !spr || !prom || ps < 0x300 || !t5cpu || cs < 0x2000 || !t5ext) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	for (int i = 0; i < 256; i++)   // RGB_444_PROMS
		s_pens[i] = (uint32_t)((prom[i] & 15) * 0x11 << 16 | (prom[i + 0x100] & 15) * 0x11 << 8 | (prom[i + 0x200] & 15) * 0x11);
	s_data = calloc(1, 0x10000);
	s_op = calloc(1, 0x10000);
	uint8_t *buf = calloc(1, ss > ts ? ss : ts);
	s_snap = calloc(1, sizeof(*s_snap));
	s_mix = calloc(1, sizeof(*s_mix));
	if (!s_data || !s_op || !buf || !s_snap || !s_mix) {
		free(buf);
		rbx_fail("%s: out of memory", g->set);
		mu_exit();
		return -1;
	}
	// the SEI80BU below 0x8000, the plain ROM above
	seibusnd_sei80bu_decrypt(s_rom, s_data, s_op, 0x8000);
	memcpy(s_data + 0x8000, s_rom + 0x8000, 0x4000);
	memcpy(s_op + 0x8000, s_rom + 0x8000, 0x4000);
	{   // init_mustache: the gfx data and address lines unscrambled
		static const int b8[8] = { 0, 5, 2, 6, 4, 1, 7, 3 };
		static const int b16[16] = { 14, 1, 13, 5, 9, 2, 10, 6, 3, 8, 4, 15, 0, 11, 12, 7 };
		static const int a16[16] = { 15, 14, 13, 2, 1, 0, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3 };
		static const int s16[16] = { 5, 7, 11, 4, 15, 10, 3, 14, 9, 2, 13, 8, 1, 12, 0, 6 };
		static const int a24[24] = { 23, 22, 21, 20, 19, 18, 17, 16, 15, 12, 11, 10, 9, 8, 7, 6, 5, 4, 13, 14, 3, 2, 1, 0 };
		const uint32_t g1 = ts / 3, g2 = ss / 2;
		for (uint32_t i = 0; i < g1; i++) {
			buf[i] = (uint8_t)bsw(tiles[i], 8, b8);
			const uint32_t w = bsw((uint32_t)(tiles[i + g1] << 8 | tiles[i + g1 * 2]), 16, b16);
			buf[i + g1] = (uint8_t)(w >> 8);
			buf[i + g1 * 2] = (uint8_t)w;
		}
		for (uint32_t i = 0; i < 3 * g1; i++)
			tiles[i] = buf[bsw(i, 16, a16)];
		for (uint32_t i = 0; i < g2; i++) {
			const uint32_t w = bsw((uint32_t)(spr[i] << 8 | spr[i + g2]), 16, s16);
			buf[i] = (uint8_t)(w >> 8);
			buf[i + g2] = (uint8_t)w;
		}
		for (uint32_t i = 0; i < 2 * g2; i++)
			spr[i] = buf[bsw(i, 24, a24)];
		free(buf);
	}
	{   // charlayout (RGN_FRAC(1,3) planes, x STEP8(7,-1)), spritelayout (RGN_FRAC(1,4) planes, x STEP16(15,-1))
		static const uint32_t cx[8] = { 7, 6, 5, 4, 3, 2, 1, 0 }, cy[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
		static uint32_t sx[16], sy[16];
		for (int i = 0; i < 16; i++) {
			sx[i] = (uint32_t)(15 - i);
			sy[i] = (uint32_t)i * 16;
		}
		const uint32_t f3 = ts / 3 * 8, f4 = ss / 4 * 8;
		const uint32_t pt[3] = { 0, f3, 2 * f3 }, psp[4] = { f4, 3 * f4, 0, 2 * f4 };
		const rbx_gfx_layout lt = { 8, 8, 3, pt, cx, cy, 64 }, ls = { 16, 16, 4, psp, sx, sy, 256 };
		if (rbx_gfx_decode(&s_gtile, &lt, tiles, ts / 3) || rbx_gfx_decode(&s_gspr, &ls, spr, ss / 4)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			mu_exit();
			return -1;
		}
	}
	if (t5182_init(t5cpu, t5ext, es, rbx_port_find("T5182_COIN"), 128) || rbx_audio_setup(48000) ||
	    rbx_video_setup(VIS_W, VIS_H, 256)) {
		rbx_fail("%s: out of memory", g->set);
		mu_exit();
		return -1;
	}
	smix_init(s_mix, 48000, FRAME_MHZ, LINES);
	static const char *const names[5] = { "P1", "P2", "START", "DSW1", "DSW2" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	mu_reset();
	return 0;
}

static void mu_exit(void)
{
	rbx_video_wait_idle();
	t5182_exit();
	rbx_gfx_free(&s_gtile);
	rbx_gfx_free(&s_gspr);
	free(s_snap);
	s_snap = NULL;
	free(s_mix);
	s_mix = NULL;
	free(s_data);
	s_data = NULL;
	free(s_op);
	s_op = NULL;
}

static void mu_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_H)
			vblank_capture();
		// (IRQ 0 held by the vblank line: 2500 us from the visible area's end)
		t.main_irq = line >= VIS_H || line < VIS_H + VBLANK_LINES - LINES;
		s_main.int_data = 0xff;
		const int c = rbx_line_cycles(&t.main_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		z80_run(&s_main, s_main.cyc + (unsigned long)c, &t.main_irq, 0);
		t5182_run(rbx_line_cycles(&t.snd_frac, T5182_CLOCK, FRAME_MHZ, LINES));
		smix_begin(s_mix);
		int32_t *tmp = smix_tmp(s_mix);
		const int n = t5182_sound((uint32_t)rbx_line_cycles(&t.ym_frac, T5182_CLOCK, FRAME_MHZ, LINES), tmp, SMIX_TMP);
		smix_add(s_mix, tmp, n);
		smix_end(s_mix);
	}
	smix_flush(s_mix);
}

static int mu_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "mustache", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	z80 c = s_main;
	rc |= rbx_state_io(ctx, "mustache_main", &c, sizeof(c));
	if (!rc && load) {
		c.read_byte = main_read;
		c.write_byte = main_write;
		c.port_in = z_in;
		c.port_out = z_out;
		c.userdata = NULL;
		c.read_map = NULL;
		c.op_rom = s_op;
		s_main = c;
	}
	rc |= t5182_state(ctx, load);
	return rc ? -1 : 0;
}

static int mu_save(void *ctx) { return mu_state(ctx, 0); }
static int mu_load(void *ctx) { return mu_state(ctx, 1); }
static int mu_supported(const char *m) { return !strcmp(m, "mustache"); }

const rbx_driver rbx_drv_mustache = { "Seibu Mustache Boy", mu_init, mu_exit, mu_reset, mu_frame, mu_save, mu_load,
                                      mu_supported, mu_render, NULL };
