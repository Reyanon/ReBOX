/*  Capcom 1942. Ported from MAME src/mame/capcom/1942.cpp, 1942_v.cpp, 1942.h (BSD-3-Clause, copyright-holders:
    Paul Leaman, Nicola Salmoria).
    Z80 at 3 MHz and a Z80 sound CPU at 3 MHz with two AY-8910 at 1.5 MHz; the interrupts from a PROM read by the
    scanline counter (RST vectors for the main CPU). Video: a 512x256 tilemap of 16x16 (3 bpp, four palette banks), a
    256x256 text layer of 8x8 (2 bpp), 16x16 sprites in columns of 1, 2 or 4 found per line as the hardware does;
    256 colours from PROMs through lookup PROMs. The AY outputs go through a netlist in MAME: plain mixing here.
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
#define NPENS (64 * 4 + 4 * 32 * 8 + 16 * 16)

static uint8_t *s_rom, *s_zrom, s_irqprom[0x100];
static uint32_t s_rom_size, s_zrom_size;
static uint32_t s_pens[NPENS];
static rbx_gfx s_gchar, s_gtile, s_gspr;
static int s_port[5];
static z80 s_main, s_snd;
static ay8910_t s_ay[2];
static const uint8_t *s_mmap[256], *s_zmap[256];
static int32_t s_mix[2048];
static int s_mix_n;

static struct {
	uint8_t ram[0x1000], spr[0x80], fg[0x800], bg[0x400];
	uint8_t zram[0x800];
	uint8_t latch[2], scroll[2], palbank, bank, flip, snd_reset;
	int main_irq, snd_irq;
	uint64_t main_frac, snd_frac, ay_frac;
	uint8_t state_end;
} t;

static struct {
	uint8_t spr[0x80], fg[0x800], bg[0x400];
	uint8_t scroll[2], palbank, flip;
} *s_snap;

// ---- sound ----
static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x4000) return s_zrom[a];
	if (a >= 0x4000 && a < 0x4800) return t.zram[a - 0x4000];
	if (a == 0x6000 || a == 0x6001) return t.latch[a & 1];
	return 0;
}
static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000 && a < 0x4800) t.zram[a - 0x4000] = v;
	else if (a == 0x8000 || a == 0x8001) ay8910_write(&s_ay[0], a & 1, v);
	else if (a == 0xc000 || a == 0xc001) ay8910_write(&s_ay[1], a & 1, v);
}

// ---- main CPU ----
static void map_bank(void)
{
	for (int p = 0; p < 0x40; p++)
		s_mmap[0x80 + p] = s_rom + 0x10000 + (uint32_t)t.bank * 0x4000 + p * 256;
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_rom[a];
	if (a < 0xc000) return s_rom[0x10000 + (uint32_t)t.bank * 0x4000 + (a - 0x8000)];
	if (a >= 0xc000 && a <= 0xc004) return (uint8_t)rbx_port_read(s_port[a - 0xc000]);
	if (a >= 0xd000 && a < 0xd800) return t.fg[a - 0xd000];
	if (a >= 0xd800 && a < 0xdc00) return t.bg[a - 0xd800];
	if (a >= 0xe000 && a < 0xf000) return t.ram[a - 0xe000];
	return 0xff;   // (unmap_value_high)
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xe000 && a < 0xf000) { t.ram[a - 0xe000] = v; return; }
	if (a >= 0xd000 && a < 0xd800) { t.fg[a - 0xd000] = v; return; }
	if (a >= 0xd800 && a < 0xdc00) { t.bg[a - 0xd800] = v; return; }
	if (a >= 0xcc00 && a < 0xcc80) { t.spr[a - 0xcc00] = v; return; }
	switch (a) {
	case 0xc800: case 0xc801: t.latch[a & 1] = v; break;
	case 0xc802: case 0xc803: t.scroll[a & 1] = v; break;
	case 0xc804:   // control_w
		if ((v & 0x10) && !t.snd_reset) {
			z80_init(&s_snd);
			s_snd.read_byte = snd_read;
			s_snd.write_byte = snd_write;
			s_snd.read_map = s_zmap;
			t.snd_irq = 0;
		}
		t.snd_reset = (v & 0x10) != 0;
		t.flip = (v & 0x80) != 0;
		break;
	case 0xc805: t.palbank = v & 3; break;
	case 0xc806: t.bank = v & 3; map_bank(); break;
	}
}
static uint8_t z_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

// ---- video ----
static void draw_sprite_line(int y)   // (draw_sprites, one line)
{
	const int flip = s_snap->flip;
	const uint8_t v = (uint8_t)(flip ? ~(y - 1) : y - 1);
	for (int h = 496; h >= 128; h -= 16) {
		const int objcnt4 = ((h >> 8) & 1) != ((~h >> 7) & 1);
		const int objcnt3 = (((v >> 7) & 1) && objcnt4) != ((~h >> 7) & 1);
		uint8_t idx = (uint8_t)((h >> 4) & 7);
		idx |= objcnt3 ? 0x08 : 0;
		idx |= objcnt4 ? 0x10 : 0;
		const uint8_t *o = &s_snap->spr[idx << 2];
		int code = (o[0] & 0x7f) | (((o[1] >> 5) & 1) << 7) | (((o[0] >> 7) & 1) << 8);
		const int col = o[1] & 0x0f;
		int sx = o[3] - (((o[1] >> 4) & 1) << 8), sy = o[2], dir = 1;
		const uint8_t valpha = (uint8_t)sy;
		const uint8_t v2c = (uint8_t)((uint8_t)~v + (flip ? 0x01 : 0xff));
		const uint8_t lvbeta = (uint8_t)(v2c + valpha), vbeta = (uint8_t)~lvbeta;
		const int vleq = vbeta <= (uint8_t)(~valpha & 0xff);
		int vinlen = 1;
		const int vlen = o[1] >> 6;
		switch (vlen & 3) {
		case 0: vinlen = (lvbeta & 0xf0) == 0xf0; break;
		case 1: vinlen = (lvbeta & 0xe0) == 0xe0; break;
		case 2: vinlen = (lvbeta & 0xc0) == 0xc0; break;
		}
		const int vinzone = !(vleq && vinlen);
		if (flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			dir = -1;
		}
		const int row = vlen == 3 ? 16 : (1 << vlen);
		code &= ~(row - 1);
		if (!vinzone)
			for (int i = 0; i < row; i++)
				cap8_transpen(&s_gspr, (uint32_t)(code + i), (uint16_t)(64 * 4 + 4 * 32 * 8 + col * 16), flip, flip, sx,
				              sy + 16 * i * dir, 15, y, y + 1, VIS_Y0, 0, VIS_W);
	}
}

static void c42_render(void)
{
	memcpy(rbx_video.palette, s_pens, sizeof(s_pens));
	const int flip = s_snap->flip;
	const uint32_t scroll = (uint32_t)(s_snap->scroll[0] | s_snap->scroll[1] << 8);
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y : y) & 255;
		for (int x = 0; x < VIS_W; x++) {   // bg: TILEMAP_SCAN_COLS 32 x 16 of 16x16
			const uint32_t tx = (uint32_t)(flip ? 255 - x + scroll : x + scroll) & 511;
			const uint32_t ti0 = (tx >> 4) * 16 + (ty >> 4);
			const uint32_t ti = (ti0 & 0x0f) | ((ti0 & 0x1f0) << 1);
			const uint8_t c = s_snap->bg[ti + 0x10];
			const uint32_t code = rbx_wrap((uint32_t)(s_snap->bg[ti] + ((c & 0x80) << 1)), s_gtile.count);
			const uint32_t px = (c & 0x20) ? 15 - (tx & 15) : tx & 15, py = (c & 0x40) ? 15 - (ty & 15) : ty & 15;
			d[x] = (uint16_t)(64 * 4 + ((c & 0x1f) + 0x20 * s_snap->palbank) * 8 + s_gtile.pix[code * 256 + py * 16 + px]);
		}
		draw_sprite_line(y);
		for (int x = 0; x < VIS_W; x++) {
			const uint32_t tx = (uint32_t)(flip ? 255 - x : x) & 255;
			const uint32_t idx = (ty >> 3) * 32 + (tx >> 3);
			const uint8_t c = s_snap->fg[idx + 0x400];
			const uint32_t code = rbx_wrap((uint32_t)(s_snap->fg[idx] + ((c & 0x80) << 1)), s_gchar.count);
			const uint8_t p = s_gchar.pix[code * 64 + (ty & 7) * 8 + (tx & 7)];
			if (p)
				d[x] = (uint16_t)((c & 0x3f) * 4 + p);
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spr, t.spr, sizeof(t.spr));
	memcpy(s_snap->fg, t.fg, sizeof(t.fg));
	memcpy(s_snap->bg, t.bg, sizeof(t.bg));
	memcpy(s_snap->scroll, t.scroll, 2);
	s_snap->palbank = t.palbank;
	s_snap->flip = t.flip;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void z_setup(z80 *z, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t), const uint8_t **map)
{
	z80_init(z);
	z->read_byte = rd;
	z->write_byte = wr;
	z->port_in = z_in;
	z->port_out = z_out;
	z->read_map = map;
}

static void c42_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	map_bank();
	z_setup(&s_main, main_read, main_write, s_mmap);
	z_setup(&s_snd, snd_read, snd_write, s_zmap);
	for (int i = 0; i < 2; i++)
		ay8910_reset(&s_ay[i]);
	s_mix_n = 0;
}

static void c42_exit(void);

static int c42_init(const rbx_game *g)
{
	uint32_t s1, s2, s3, sp, sc, st, ss, si;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *chars = rbx_region("gfx1", &s1), *tiles = rbx_region("gfx2", &s2), *spr = rbx_region("gfx3", &s3);
	const uint8_t *pal = rbx_region("palproms", &sp), *clut = rbx_region("charprom", &sc), *tlut = rbx_region("tileprom", &st);
	const uint8_t *slut = rbx_region("sprprom", &ss), *irq = rbx_region("irqprom", &si);
	if (!s_rom || s_rom_size < 0x20000 || !s_zrom || s_zrom_size < 0x4000 || !chars || !tiles || !spr || !pal || sp < 0x300 ||
	    !clut || !tlut || !slut || !irq || si < 0x100) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memcpy(s_irqprom, irq, 0x100);
	{	// _1942_palette: 256 colours, the lookup PROMs
		uint32_t rgb[256];
		for (int i = 0; i < 256; i++)
			rgb[i] = (uint32_t)(cap8_res4(pal[i]) << 16 | cap8_res4(pal[256 + i]) << 8 | cap8_res4(pal[512 + i]));
		int n = 0;
		for (int i = 0; i < 64 * 4; i++)
			s_pens[n++] = rgb[0x80 | (clut[i] & 0x0f)];
		for (int b = 0; b < 4; b++)
			for (int i = 0; i < 32 * 8; i++)
				s_pens[n++] = rgb[(b << 4) | (tlut[i] & 0x0f)];
		for (int i = 0; i < 16 * 16; i++)
			s_pens[n++] = rgb[0x40 | (slut[i] & 0x0f)];
	}
	{	// charlayout, tilelayout (RGN_FRAC(1,3)), spritelayout (RGN_FRAC(1,2))
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
		const uint32_t pt[3] = { 0, f3, 2 * f3 }, ps[4] = { f2 + 4, f2, 4, 0 };
		const rbx_gfx_layout lc = { 8, 8, 2, pc, xc, yc, 128 };
		const rbx_gfx_layout lt = { 16, 16, 3, pt, xt, yt, 256 }, ls = { 16, 16, 4, ps, xs, ys, 512 };
		if (rbx_gfx_decode(&s_gchar, &lc, chars, s1) || rbx_gfx_decode(&s_gtile, &lt, tiles, s2 / 3) ||
		    rbx_gfx_decode(&s_gspr, &ls, spr, s3 / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			c42_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	for (int i = 0; i < 2; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 64, 4, 0);
	if (!s_snap || rbx_audio_setup(ay8910_rate(&s_ay[0])) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		c42_exit();
		return -1;
	}
	memset(s_mmap, 0, sizeof(s_mmap));
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x80; p++)
		s_mmap[p] = s_rom + p * 256;
	for (int p = 0; p < 0x40; p++)
		s_zmap[p] = s_zrom + p * 256;
	static const char *const names[5] = { "SYSTEM", "P1", "P2", "DSWA", "DSWB" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	c42_reset();
	return 0;
}

static void c42_exit(void)
{
	rbx_video_wait_idle();
	rbx_gfx_free(&s_gchar);
	rbx_gfx_free(&s_gtile);
	rbx_gfx_free(&s_gspr);
	free(s_snap);
	s_snap = NULL;
}

static void c42_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		const uint8_t irq = s_irqprom[line & 0xff];   // (scanline)
		if (irq & 8) {
			s_main.int_data = (uint8_t)(0xc7 | ((irq << 3) & 0x18));
			t.main_irq = 1;
		}
		if ((irq & 4) && !t.snd_reset) {
			s_snd.int_data = 0xff;
			t.snd_irq = 1;
		}
		const int c = rbx_line_cycles(&t.main_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		z80_run(&s_main, s_main.cyc + (unsigned long)c, &t.main_irq, 1);
		const int cs = rbx_line_cycles(&t.snd_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		if (t.snd_reset)
			s_snd.cyc += (unsigned long)cs;
		else
			z80_run(&s_snd, s_snd.cyc + (unsigned long)cs, &t.snd_irq, 1);
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

static int z_state(void *ctx, const char *tag, z80 *z, int load, uint8_t (*rd)(void *, uint16_t),
                   void (*wr)(void *, uint16_t, uint8_t), const uint8_t **map)
{
	z80 c = *z;
	const int rc = rbx_state_io(ctx, tag, &c, sizeof(c));
	if (!rc && load) {
		c.read_byte = rd;
		c.write_byte = wr;
		c.port_in = z_in;
		c.port_out = z_out;
		c.userdata = NULL;
		c.read_map = map;
		*z = c;
	}
	return rc;
}

static int c42_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "c42", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= z_state(ctx, "c42_main", &s_main, load, main_read, main_write, s_mmap);
	rc |= z_state(ctx, "c42_snd", &s_snd, load, snd_read, snd_write, s_zmap);
	rc |= rbx_state_io(ctx, "c42_ay0", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "c42_ay1", &s_ay[1], ay8910_state_size());
	if (!rc && load)
		map_bank();
	return rc ? -1 : 0;
}

static int c42_save(void *ctx) { return c42_state(ctx, 0); }
static int c42_load(void *ctx) { return c42_state(ctx, 1); }
static int c42_supported(const char *machine) { return !strcmp(machine, "_1942"); }

const rbx_driver rbx_drv_c1942 = { "Capcom 1942", c42_init, c42_exit, c42_reset, c42_frame, c42_save, c42_load,
                                   c42_supported, c42_render, NULL };
