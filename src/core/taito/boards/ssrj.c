/*  Super Speed Race Junior hardware, ported from MAME src/mame/taito/ssrj.cpp (license BSD-3-Clause,
    copyright-holders: Tomasz Slanina). Memory map, the vblank IRQ (HOLD_LINE), the four tilemaps (scrolled
    background, objects drawn from tilemap 2 RAM, fixed text, tilemap 3 shown by the 0x101 == 0x0b hack), the
    vblank-buffered scroll RAM and MAME's fake colour table (the colour PROM is not hooked up in MAME) follow MAME.
    Controls: the wheel (MAME dial, PORT_REVERSE, read as a delta) from left / right or the analog dial, the pedal
    (3 bits) from button 1 (MAME key delta 0x20 per frame). Sound: one AY-3-8910 at 1.6 MHz (sound/ay8910).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../rbx_core.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"

#include <stdlib.h>
#include <string.h>

// MAME: 60 Hz, 320 x 256, visible 0-271 x 8-247, vblank time 0 (256 lines here, 15360 Hz)
#define LINES 256
#define VIS_Y0 8
#define VIS_Y1 247
#define SCREEN_W 272
#define LINE_RATE 15360
#define MAIN_CLOCK 4000000
#define AY_CLOCK 1600000

static const uint8_t s_fakecols[16][8][3] = {
	{ {0x00,0x00,0x00}, {42,87,140}, {0,0,0}, {33,75,160}, {0xff,0xff,0xff}, {37,56,81}, {0x1f,0x1f,0x2f}, {55,123,190} },
	{ {0x00,0x00,0x00}, {0x00,99,41}, {0x00,0x00,0xff}, {0x00,0xff,0}, {255,255,255}, {0xff,0x00,0x00}, {0,45,105}, {0xff,0xff,0} },
	{ {0x00,0x00,0x00}, {0x00,0x20,0x00}, {0x00,0x40,0x00}, {0x00,0x60,0x00}, {0x00,0x80,0x00}, {0x00,0xa0,0x00}, {0x00,0xc0,0x00}, {0x00,0xf0,0x00} },
	{ {0x00,0x00,0x00}, {0x20,0x00,0x20}, {0x40,0x00,0x40}, {0x60,0x00,0x60}, {0x80,0x00,0x80}, {0xa0,0x00,0xa0}, {0xc0,0x00,0xc0}, {0xf0,0x00,0xf0} },
	{ {0x00,0x00,0x00}, {0xff,0x00,0x00}, {0x7f,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0xaf,0x00,0x00}, {0xff,0xff,0xff}, {0xff,0x7f,0x7f} },
	{ {0x00,0x00,0x00}, {0x20,0x20,0x20}, {0x40,0x40,0x40}, {0x60,0x60,0x60}, {0x80,0x80,0x80}, {0xa0,0xa0,0xa0}, {0xc0,0xc0,0xc0}, {0xf0,0xf0,0xf0} },
	{ {0x00,0x00,0x00}, {0x20,0x20,0x20}, {0x40,0x40,0x40}, {0x60,0x60,0x60}, {0x80,0x80,0x80}, {0xa0,0xa0,0xa0}, {0xc0,0xc0,0xc0}, {0xf0,0xf0,0xf0} },
	{ {0x00,0x00,0x00}, {0xff,0x00,0x00}, {0x00,0x00,0x9f}, {0x60,0x60,0x60}, {0x00,0x00,0x00}, {0xff,0xff,0x00}, {0x00,0xff,0x00}, {0xff,0xff,0xff} },
	{ {0x00,0x00,0x00}, {0x00,0x00,0xff}, {0x00,0x00,0x7f}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0xaf}, {0xff,0xff,0xff}, {0x7f,0x7f,0xff} },
	{ {0x00,0x00,0x00}, {0xff,0xff,0x00}, {0x7f,0x7f,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0xaf,0xaf,0x00}, {0xff,0xff,0xff}, {0xff,0xff,0x7f} },
	{ {0x00,0x00,0x00}, {0x00,0xff,0x00}, {0x00,0x7f,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0xaf,0x00}, {0xff,0xff,0xff}, {0x7f,0xff,0x7f} },
	{ {0x00,0x00,0x00}, {0x20,0x20,0x20}, {0x40,0x40,0x40}, {0x60,0x60,0x60}, {0x80,0x80,0x80}, {0xa0,0xa0,0xa0}, {0xc0,0xc0,0xc0}, {0xf0,0xf0,0xf0} },
	{ {0x00,0x00,0x00}, {0x20,0x20,0x20}, {0x40,0x40,0x40}, {0x60,0x60,0x60}, {0x80,0x80,0x80}, {0xa0,0xa0,0xa0}, {0xc0,0xc0,0xc0}, {0xf0,0xf0,0xf0} },
	{ {0x00,0x00,0x00}, {0x20,0x20,0x20}, {0x40,0x40,0x40}, {0x60,0x60,0x60}, {0x80,0x80,0x80}, {0xa0,0xa0,0xa0}, {0xc0,0xc0,0xc0}, {0xf0,0xf0,0xf0} },
	{ {0x00,0x00,0x00}, {0x20,0x20,0x20}, {0x40,0x40,0x40}, {0x60,0x60,0x60}, {0x80,0x80,0x80}, {0xa0,0xa0,0xa0}, {0xc0,0xc0,0xc0}, {0xf0,0xf0,0xf0} },
	{ {0x00,0x00,0x00}, {0xff,0xaf,0xaf}, {0x00,0x00,0xff}, {0xff,0xff,0xff}, {0x00,0x00,0x00}, {0xff,0x50,0x50}, {0xff,0xff,0x00}, {0x00,0xff,0x00} },
};

static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t s_vram[4][0x800], s_ram[0x800], s_scrollram[0x800];
static z80 s_z;
static int s_irq;
static int s_oldport, s_dial, s_pedal;
static ay8910_t s_ay;
static uint64_t s_main_frac, s_ay_frac;
// video
static uint8_t *s_chars;   // 1024 8x8 3bpp
static uint32_t s_pal_rgb[128];
typedef struct {
	uint8_t vram[4][0x800], scroll[0x200];
} snap_t;
static snap_t *s_snap;
// ports
static int s_port_in0, s_port_in2, s_port_in3;
// sound
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static unsigned long s_line_start;
static int s_line_len, s_line_ay;
static uint32_t s_ay_line_done;

static void ay_sync(void)
{
	if (s_line_len <= 0)
		return;
	uint32_t done = (uint32_t)(s_z.cyc - s_line_start);
	if (done > (uint32_t)s_line_len)
		done = (uint32_t)s_line_len;
	uint32_t pos = (uint32_t)s_line_ay * done / (uint32_t)s_line_len;
	if (pos > s_ay_line_done) {
		s_mix_n += ay8910_render(&s_ay, pos - s_ay_line_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
		s_ay_line_done = pos;
	}
}
static uint8_t ay_port_b(void *u) { (void)u; return s_port_in3 >= 0 ? (uint8_t)(rbx_port_read(s_port_in3) & 0x7f) : 0; }

// ---- main CPU ----
static uint8_t wheel_r(void)
{
	int port = (s_dial & 0xff) - 0x80;
	int ret = port - s_oldport;
	s_oldport = port;
	return (uint8_t)ret;
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_rom[a];
	if (a >= 0xc000 && a < 0xe000) return s_vram[(a >> 11) & 3][a & 0x7ff];
	if (a >= 0xe000 && a < 0xe800) return s_ram[a & 0x7ff];
	if (a >= 0xe800 && a < 0xf000) return s_scrollram[a & 0x7ff];
	switch (a) {
	// (bits without a MAME field read 0 here: the game checks IN0 bits 0-1 at boot, "I/O ERROR")
	case 0xf000: return (uint8_t)(((s_port_in0 >= 0 ? rbx_port_read(s_port_in0) : 0) & 0x0d) | s_pedal);
	case 0xf001: return wheel_r();
	case 0xf002: {
		uint8_t v = s_port_in2 >= 0 ? (uint8_t)rbx_port_read(s_port_in2) : 0;
		// IN2 bits 0-3: unused (active low) when IN3 controls = type 1 (the generated table lacks that
		// conditional field), the four start buttons otherwise
		if (s_port_in3 >= 0 && !(rbx_port_read(s_port_in3) & 0x30))
			v |= 0x0f;
		return v;
	}
	case 0xf401: ay_sync(); return ay8910_data_r(&s_ay);
	}
	return 0;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xe000) { s_vram[(a >> 11) & 3][a & 0x7ff] = v; return; }
	if (a >= 0xe000 && a < 0xe800) { s_ram[a & 0x7ff] = v; return; }
	if (a >= 0xe800 && a < 0xf000) { s_scrollram[a & 0x7ff] = v; return; }
	if (a == 0xf400 || a == 0xf401) {
		ay_sync();
		ay8910_write(&s_ay, a & 1, v);
	}
}

static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// ---- video ----
static void draw_tile(uint16_t *bm, int pitch, int code, int color, int fx, int fy, int x, int y)
{
	const uint8_t *src = s_chars + (code & 1023) * 64;
	for (int yy = 0; yy < 8; yy++) {
		int dy = y + yy;
		if (dy < VIS_Y0 || dy > VIS_Y1)
			continue;
		const uint8_t *r = src + (fy ? 7 - yy : yy) * 8;
		uint16_t *d = bm + (dy - VIS_Y0) * pitch;
		for (int xx = 0; xx < 8; xx++) {
			int dx = x + xx;
			uint8_t p = r[fx ? 7 - xx : xx];
			if (p && dx >= 0 && dx < SCREEN_W)
				d[dx] = (uint16_t)(color * 8 + p);
		}
	}
}

// TILEMAP_SCAN_COLS 32 x 32, 2 bytes a tile; transparent = pen 0 skipped
static void draw_layer(uint16_t *bm, int pitch, const uint8_t *vram, int which, int sx0, int sy0, int opaque)
{
	for (int sy = VIS_Y0; sy <= VIS_Y1; sy++) {
		uint16_t *d = bm + (sy - VIS_Y0) * pitch;
		int ty = (sy + sy0) & 255;
		for (int sx = 0; sx < SCREEN_W; sx++) {
			int tx = (sx + sx0) & 255;
			int ti = (tx >> 3) * 32 + (ty >> 3);
			int code = vram[ti * 2] | vram[ti * 2 + 1] << 8;
			int px = (code & 0x4000) ? 7 - (tx & 7) : (tx & 7);
			int py = (code & 0x8000) ? 7 - (ty & 7) : (ty & 7);
			uint8_t p = s_chars[(code & 0x3ff) * 64 + py * 8 + px];
			if (p || opaque)
				d[sx] = (uint16_t)((((code >> 12) & 3) + which * 4) * 8 + p);
		}
	}
}

static void sj_render(void)
{
	const snap_t *s = s_snap;
	memcpy(rbx_video.palette, s_pal_rgb, sizeof(s_pal_rgb));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	draw_layer(bm, pitch, s->vram[0], 0, (0xff - s->scroll[2]) & 0xff, s->scroll[0], 1);
	// objects from tilemap 2 RAM
	for (int i = 0; i < 6; i++) {
		int y = s->scroll[0x80 + 20 * i], x = s->scroll[0x80 + 20 * i + 2];
		if (s->scroll[0x80 + 20 * i + 3])
			continue;
		for (int k = 0; k < 5; k++, x += 8)
			for (int j = 0; j < 0x20; j++) {
				int offs = (i * 5 + k) * 64 + (31 - j) * 2;
				int code = s->vram[2][offs] + 256 * s->vram[2][offs + 1];
				draw_tile(bm, pitch, code, ((code >> 12) & 3) + 8, code & 0x4000, code & 0x8000, x,
				          (247 - (y + (j << 3))) & 0xff);
			}
	}
	draw_layer(bm, pitch, s->vram[1], 1, 0, 0, 0);
	if (s->scroll[0x101] == 0x0b)   // MAME hack to display the 4th tilemap
		draw_layer(bm, pitch, s->vram[3], 3, 0, 0, 0);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->vram, s_vram, sizeof(s_vram));
	memcpy(s_snap->scroll, s_scrollram, sizeof(s_snap->scroll));
	rbx_video_frame_done();
}

// ---- init / frame ----
static void sj_reset(void)
{
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_scrollram, 0, sizeof(s_scrollram));
	z80_init(&s_z);
	s_z.read_byte = main_read;
	s_z.write_byte = main_write;
	s_z.port_in = no_in;
	s_z.port_out = no_out;
	s_z.int_data = 0xff;
	s_irq = 0;
	s_oldport = 0x80;
	s_dial = 0;
	s_pedal = 0;
	s_main_frac = s_ay_frac = 0;
	ay8910_reset(&s_ay);
	s_mix_n = 0;
}

static void sj_exit(void)
{
	free(s_chars);
	free(s_snap);
	free(s_mix);
	s_chars = NULL;
	s_snap = NULL;
	s_mix = NULL;
}

static int sj_init(const rbx_game *g)
{
	uint32_t gs = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *gfx = rbx_region("gfx", &gs);
	if (!s_rom || s_rom_size < 0x8000 || !gfx || gs < 0x6000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_chars = (uint8_t *)malloc(1024 * 64);
	s_snap = (snap_t *)calloc(1, sizeof(snap_t));
	if (!s_chars || !s_snap) {
		sj_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	// charlayout: planes { 0, RGN_FRAC(2,3), RGN_FRAC(1,3) }, the first is the most significant bit
	for (int t = 0; t < 1024; t++)
		for (int y = 0; y < 8; y++) {
			uint8_t p0 = gfx[t * 8 + y], p1 = gfx[0x4000 + t * 8 + y], p2 = gfx[0x2000 + t * 8 + y];
			for (int x = 0; x < 8; x++) {
				int b = 7 - x;
				s_chars[t * 64 + y * 8 + x] = (uint8_t)(((p0 >> b) & 1) << 2 | ((p1 >> b) & 1) << 1 | ((p2 >> b) & 1));
			}
		}
	for (int i = 0; i < 16; i++)
		for (int j = 0; j < 8; j++)
			s_pal_rgb[i * 8 + j] = (uint32_t)(s_fakecols[i][j][0] << 16 | s_fakecols[i][j][1] << 8 | s_fakecols[i][j][2]);
	s_port_in0 = rbx_port_find("IN0");
	s_port_in2 = rbx_port_find("IN2");
	s_port_in3 = rbx_port_find("IN3");
	// MAME route 0.30, legacy output; 1.6 MHz / 8 / 4 = 50 kHz
	ay8910_init(&s_ay, AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 77, 4, 1);
	s_ay.port_b_read = ay_port_b;
	s_mix_cap = (int)(ay8910_rate(&s_ay) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(ay8910_rate(&s_ay)) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 128)) {
		sj_exit();
		return -1;
	}
	sj_reset();
	return 0;
}

static void sj_frame(void)
{
	// controls: wheel (dial, reversed) and pedal (0 - 0xe0 in steps of 0x20)
	uint32_t in = rbx_input_bits(1);
	int dd = rbx_analog(1, RBX_IPT_DIAL);
	if (!dd)
		dd = ((in & RBX_IN_RIGHT) ? 4 : 0) - ((in & RBX_IN_LEFT) ? 4 : 0);
	s_dial = (s_dial - dd) & 0xff;
	if (in & RBX_IN_B1)
		s_pedal = s_pedal + 0x20 > 0xe0 ? 0xe0 : s_pedal + 0x20;
	else
		s_pedal = s_pedal - 0x20 < 0 ? 0 : s_pedal - 0x20;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			s_irq = 1;   // irq0_line_hold
		}
		s_line_len = rbx_line_cycles(&s_main_frac, MAIN_CLOCK, LINE_RATE * 1000, 1);
		s_line_ay = rbx_line_cycles(&s_ay_frac, AY_CLOCK, LINE_RATE * 1000, 1);
		s_ay_line_done = 0;
		s_line_start = s_z.cyc;
		unsigned long end = s_z.cyc + (unsigned long)s_line_len;
		while ((long)(s_z.cyc - end) < 0) {
			s_z.int_pending = s_irq ? 1 : 0;
			z80_step(&s_z);
			if (s_irq && !s_z.int_pending)
				s_irq = 0;
		}
		uint32_t rest = (uint32_t)s_line_ay - s_ay_line_done;
		if (rest)
			s_mix_n += ay8910_render(&s_ay, rest, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int sj_state(void *ctx, int load)
{
	z80 z = s_z;
	int rc = 0;
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "scroll", s_scrollram, sizeof(s_scrollram));
	int32_t misc[4] = { s_irq, s_oldport, s_dial, s_pedal };
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	rc |= rbx_state_io(ctx, "mfrac", &s_main_frac, sizeof(s_main_frac));
	rc |= rbx_state_io(ctx, "ayfrac", &s_ay_frac, sizeof(s_ay_frac));
	rc |= rbx_state_io(ctx, "ay", &s_ay, ay8910_state_size());
	if (rc)
		return -1;
	if (load) {
		z.read_byte = main_read;
		z.write_byte = main_write;
		z.port_in = no_in;
		z.port_out = no_out;
		z.userdata = NULL;
		s_z = z;
		s_irq = misc[0]; s_oldport = misc[1]; s_dial = misc[2]; s_pedal = misc[3];
	}
	return 0;
}

static int sj_save(void *ctx) { return sj_state(ctx, 0); }
static int sj_load(void *ctx) { return sj_state(ctx, 1); }
static int sj_supported(const char *machine) { return !strcmp(machine, "ssrj"); }

const rbx_driver rbx_drv_ssrj = { "Super Speed Race Jr", sj_init, sj_exit, sj_reset, sj_frame, sj_save, sj_load,
                                  sj_supported, sj_render };
