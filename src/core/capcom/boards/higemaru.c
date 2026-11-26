/*  Capcom Pirate Ship Higemaru. Ported from MAME src/mame/capcom/higemaru.cpp (BSD-3-Clause, copyright-holders:
    Mirko Buffoni).
    Z80 at 3 MHz (the interrupts from a PROM read by the scanline counter) with two AY-8910 at 1.5 MHz. Video: a
    256x256 tilemap of 8x8 (2 bpp), 16x16 sprites (also drawn 256 pixels to the left); 32 colours through lookup PROMs.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"
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
#define CPU_CLOCK 3000000
#define AY_CLOCK 1500000
#define NPENS (32 * 4 + 16 * 16)

static uint8_t *s_rom, s_irqprom[0x100];
static uint32_t s_rom_size;
static uint32_t s_pens[NPENS];
static rbx_gfx s_gchar, s_gspr;
static int s_port[5];
static z80 s_main;
static ay8910_t s_ay[2];
static const uint8_t *s_mmap[256];
static int32_t s_mix[2048];
static int s_mix_n;

static struct {
	uint8_t ram[0x1000], vram[0x400], cram[0x400], spr[0x180];
	uint8_t flip;
	int irq;
	uint64_t frac, ay_frac;
	uint8_t state_end;
} t;

static struct {
	uint8_t vram[0x400], cram[0x400], spr[0x180];
	uint8_t flip;
} *s_snap;

static uint8_t z_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_rom[a];
	if (a >= 0xc000 && a <= 0xc004) return (uint8_t)rbx_port_read(s_port[a - 0xc000]);
	if (a >= 0xd000 && a < 0xd400) return t.vram[a - 0xd000];
	if (a >= 0xd400 && a < 0xd800) return t.cram[a - 0xd400];
	if (a >= 0xd880 && a < 0xda00) return t.spr[a - 0xd880];
	if (a >= 0xe000 && a < 0xf000) return t.ram[a - 0xe000];
	return 0;
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xe000 && a < 0xf000) t.ram[a - 0xe000] = v;
	else if (a >= 0xd000 && a < 0xd400) t.vram[a - 0xd000] = v;
	else if (a >= 0xd400 && a < 0xd800) t.cram[a - 0xd400] = v;
	else if (a >= 0xd880 && a < 0xda00) t.spr[a - 0xd880] = v;
	else if (a == 0xc800) t.flip = (v & 0x80) != 0;
	else if (a == 0xc801 || a == 0xc802) ay8910_write(&s_ay[0], (a - 0xc801) & 1, v);   // (address_data_w)
	else if (a == 0xc803 || a == 0xc804) ay8910_write(&s_ay[1], (a - 0xc803) & 1, v);
}

static void hg_render(void)
{
	memcpy(rbx_video.palette, s_pens, sizeof(s_pens));
	const int flip = s_snap->flip;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y : y) & 255;
		uint32_t last = 0xffffffffu, cbase = 0, hflip = 0;
		const uint8_t *row = NULL;
		for (int x = 0; x < VIS_W; x++) {
			const uint32_t tx = (uint32_t)(flip ? 255 - x : x) & 255;
			if ((tx >> 3) != last) {   // (a character looked up once for its 8 pixels)
				last = tx >> 3;
				const uint32_t idx = (ty >> 3) * 32 + (tx >> 3);
				const uint8_t attr = s_snap->cram[idx];
				const uint32_t code = rbx_wrap((uint32_t)(s_snap->vram[idx] | ((attr & 0x80) << 1)), s_gchar.count);
				const uint32_t py = (attr & 0x40) ? 7 - (ty & 7) : ty & 7;
				row = s_gchar.pix + code * 64 + py * 8;
				hflip = attr & 0x20;
				cbase = (uint32_t)(attr & 0x1f) * 4;
			}
			d[x] = (uint16_t)(cbase + row[hflip ? 7 - (tx & 7) : tx & 7]);
		}
	}
	for (int o = 0x180 - 16; o >= 0; o -= 16) {
		const uint8_t *s = &s_snap->spr[o];
		const uint32_t code = s[0] & 0x7f;
		const uint16_t base = (uint16_t)(32 * 4 + (s[4] & 0x0f) * 16);
		int sx = s[12], sy = s[8], fx = (s[4] & 0x10) != 0, fy = (s[4] & 0x20) != 0;
		if (flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			fx = !fx;
			fy = !fy;
		}
		cap8_transpen(&s_gspr, code, base, fx, fy, sx, sy, 15, VIS_Y0, VIS_Y0 + VIS_H, VIS_Y0, 0, VIS_W);
		cap8_transpen(&s_gspr, code, base, fx, fy, sx - 256, sy, 15, VIS_Y0, VIS_Y0 + VIS_H, VIS_Y0, 0, VIS_W);
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->vram, t.vram, sizeof(t.vram));
	memcpy(s_snap->cram, t.cram, sizeof(t.cram));
	memcpy(s_snap->spr, t.spr, sizeof(t.spr));
	s_snap->flip = t.flip;
	rbx_video_frame_done();
}

static void hg_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	z80_init(&s_main);
	s_main.read_byte = main_read;
	s_main.write_byte = main_write;
	s_main.port_in = z_in;
	s_main.port_out = z_out;
	s_main.read_map = s_mmap;
	for (int i = 0; i < 2; i++)
		ay8910_reset(&s_ay[i]);
	s_mix_n = 0;
}

static void hg_exit(void);

static int hg_init(const rbx_game *g)
{
	uint32_t s1, s2, sp, si;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *chars = rbx_region("chars", &s1), *spr = rbx_region("sprites", &s2);
	const uint8_t *prom = rbx_region("proms", &sp), *irq = rbx_region("irqprom", &si);
	if (!s_rom || s_rom_size < 0x8000 || !chars || !spr || !prom || sp < 0x320 || !irq || si < 0x100) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memcpy(s_irqprom, irq, 0x100);
	{
		uint32_t rgb[32];
		for (int i = 0; i < 32; i++) {
			const uint8_t c = prom[i];
			const int r = 0x21 * (c & 1) + 0x47 * ((c >> 1) & 1) + 0x97 * ((c >> 2) & 1);
			const int gg = 0x21 * ((c >> 3) & 1) + 0x47 * ((c >> 4) & 1) + 0x97 * ((c >> 5) & 1);
			const int b = 0x47 * ((c >> 6) & 1) + 0x97 * ((c >> 7) & 1);
			rgb[i] = (uint32_t)(r << 16 | gg << 8 | b);
		}
		const uint8_t *cp = prom + 0x20;
		for (int i = 0; i < 0x80; i++)
			s_pens[i] = rgb[cp[i] & 0x0f];
		for (int i = 0x80; i < 0x180; i++)
			s_pens[i] = rgb[(cp[i + 0x80] & 0x0f) | 0x10];
	}
	{	// charlayout, spritelayout (RGN_FRAC(1,2))
		static const uint32_t pc[2] = { 4, 0 }, xc[8] = { 0, 1, 2, 3, 8, 9, 10, 11 };
		static const uint32_t yc[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
		static const uint32_t xs[16] = { 0, 1, 2, 3, 8, 9, 10, 11, 256, 257, 258, 259, 264, 265, 266, 267 };
		static uint32_t ys[16];
		for (int i = 0; i < 16; i++)
			ys[i] = (uint32_t)i * 16;
		const uint32_t h = s2 / 2 * 8;
		const uint32_t ps[4] = { h + 4, h, 4, 0 };
		const rbx_gfx_layout lc = { 8, 8, 2, pc, xc, yc, 128 }, ls = { 16, 16, 4, ps, xs, ys, 512 };
		if (rbx_gfx_decode(&s_gchar, &lc, chars, s1) || rbx_gfx_decode(&s_gspr, &ls, spr, s2 / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			hg_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	for (int i = 0; i < 2; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 64, 4, 0);   // (0.25)
	if (!s_snap || rbx_audio_setup(ay8910_rate(&s_ay[0])) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		hg_exit();
		return -1;
	}
	memset(s_mmap, 0, sizeof(s_mmap));
	for (int p = 0; p < 0x80; p++)
		s_mmap[p] = s_rom + p * 256;
	static const char *const names[5] = { "P1", "P2", "SYSTEM", "DSW1", "DSW2" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	hg_reset();
	return 0;
}

static void hg_exit(void)
{
	rbx_video_wait_idle();
	rbx_gfx_free(&s_gchar);
	rbx_gfx_free(&s_gspr);
	free(s_snap);
	s_snap = NULL;
}

static void hg_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		const uint8_t irq = s_irqprom[line & 0xff];
		if (irq & 8) {
			s_main.int_data = (uint8_t)(0xc7 | ((irq << 3) & 0x18));
			t.irq = 1;
		}
		z80_run(&s_main, s_main.cyc + (unsigned long)rbx_line_cycles(&t.frac, CPU_CLOCK, FRAME_MHZ, LINES), &t.irq, 1);
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

static int hg_state(void *ctx, int load)
{
	int rc = 0;
	z80 c = s_main;
	rc |= rbx_state_io(ctx, "hg", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "hg_main", &c, sizeof(c));
	rc |= rbx_state_io(ctx, "hg_ay0", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "hg_ay1", &s_ay[1], ay8910_state_size());
	if (!rc && load) {
		c.read_byte = main_read;
		c.write_byte = main_write;
		c.port_in = z_in;
		c.port_out = z_out;
		c.userdata = NULL;
		c.read_map = s_mmap;
		s_main = c;
	}
	return rc ? -1 : 0;
}

static int hg_save(void *ctx) { return hg_state(ctx, 0); }
static int hg_load(void *ctx) { return hg_state(ctx, 1); }
static int hg_supported(const char *machine) { return !strcmp(machine, "higemaru"); }

const rbx_driver rbx_drv_higemaru = { "Capcom Pirate Ship Higemaru", hg_init, hg_exit, hg_reset, hg_frame, hg_save,
                                      hg_load, hg_supported, hg_render, NULL };
