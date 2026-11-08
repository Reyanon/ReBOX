/*  Jolly Jogger / Frog & Spiders hardware, ported from MAME src/mame/taito/jollyjgr.cpp (license BSD-3-Clause,
    copyright-holders: Pierpaolo Prazzoli). Memory maps, the misc latch (flip, bitmap disable, tile bank, priority,
    NMI enable), the vblank NMI, the Galaxian-like column-scrolled tilemap and sprites, the 3bpp bitmap and the
    Frog & Spiders bullets follow MAME. Sound: one AY-3-8910 at 3.579545 MHz / 2 (sound/ay8910).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"

#include <stdlib.h>
#include <string.h>

// MAME set_raw(18 MHz / 3, 384, 0, 256, 264, 16, 240): 15625 Hz lines
#define LINES 264
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256
#define MAIN_LINE 192           // 3 MHz
#define LINE_RATE 15625
#define AY_CLOCK 1789772        // 3.579545 MHz / 2
#define PAL_BM 32               // RGB_3BIT bitmap palette after the 32 PROM colours

static int s_fspider;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t s_ram[0x800], s_videoram[0x400], s_ram98[0x400], s_bitmap[0x6000];
static z80 s_z;
static int s_nmi_line;
static uint8_t s_misc;
static ay8910_t s_ay;
static uint64_t s_ay_frac;
// video
static uint8_t *s_chars, *s_chars2, *s_sprites;
static int s_nchars2;
static uint32_t s_pal_rgb[40];
typedef struct {
	uint8_t videoram[0x400], ram98[0x100], bitmap[0x6000];
	uint8_t misc;
} snap_t;
static snap_t *s_snap;
// ports
static int s_port_dsw1, s_port_dsw2, s_port_inputs, s_port_system;
// sound
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static unsigned long s_line_start;
static int s_line_ay;
static uint32_t s_ay_line_done;

static void ay_sync(void)
{
	uint32_t done = (uint32_t)(s_z.cyc - s_line_start);
	if (done > MAIN_LINE)
		done = MAIN_LINE;
	uint32_t pos = (uint32_t)s_line_ay * done / MAIN_LINE;
	if (pos > s_ay_line_done) {
		s_mix_n += ay8910_render(&s_ay, pos - s_ay_line_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
		s_ay_line_done = pos;
	}
}

static uint8_t port8(int p) { return p >= 0 ? (uint8_t)rbx_port_read(p) : 0; }

static void misc_w(uint8_t data)
{
	s_misc = data;
	if (!(data & 0x80))
		s_nmi_line = 0;   // NMI line cleared
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_rom[a];
	if (a < 0x8800) return s_ram[a & 0x7ff];
	if (a >= 0xa000) return s_bitmap[a - 0xa000];
	switch (a) {
	case 0x8ff8: return port8(s_port_dsw1);
	case 0x8ff9: return port8(s_port_inputs);
	case 0x8ffa: return port8(s_port_system);
	case 0x8fff: return port8(s_port_dsw2);
	}
	if (a >= 0x9000 && a < 0x9400) return s_videoram[a & 0x3ff];
	if (a >= 0x9800 && a < 0x9c00) return s_ram98[a & 0x3ff];
	return 0;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x8000) return;
	if (a < 0x8800) { s_ram[a & 0x7ff] = v; return; }
	if (a >= 0xa000) { s_bitmap[a - 0xa000] = v; return; }
	switch (a) {
	case 0x8ff8: ay_sync(); ay8910_address_w(&s_ay, v); return;
	case 0x8ffa: ay_sync(); ay8910_data_w(&s_ay, v); return;
	case 0x8ffc: misc_w(v); return;
	case 0x8ffd: return;   // coin lockout
	}
	if (a >= 0x9000 && a < 0x9400) { s_videoram[a & 0x3ff] = v; return; }
	if (a >= 0x9800 && a < 0x9c00) s_ram98[a & 0x3ff] = v;
}

static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// ---- video ----
static void draw_tilemap(const snap_t *s, uint16_t *bm, int pitch, int fx, int fy)
{
	const uint8_t *chars = (s->misc & 0x20) ? s_chars2 : s_chars;
	const int nchars = (s->misc & 0x20) ? s_nchars2 : 256;
	for (int sy = VIS_Y0; sy <= VIS_Y1; sy++) {
		uint16_t *d = bm + (sy - VIS_Y0) * pitch;
		int ny = fy ? 255 - sy : sy;
		for (int sx = 0; sx < SCREEN_W; sx++) {
			int nx = fx ? 255 - sx : sx;
			int col = nx >> 3;
			int ty = (ny + s->ram98[col << 1]) & 255;
			int code = s->videoram[(ty >> 3) * 32 + col] % nchars;
			uint8_t p = chars[code * 64 + (ty & 7) * 8 + (nx & 7)];
			if (p)
				d[sx] = (uint16_t)((s->ram98[(col << 1) | 1] & 7) * 4 + p);
		}
	}
}

static void draw_bitmap(const snap_t *s, uint16_t *bm, int pitch, int fx, int fy)
{
	for (int sy = VIS_Y0; sy <= VIS_Y1; sy++) {
		uint16_t *d = bm + (sy - VIS_Y0) * pitch;
		int y = fy ? sy : 255 - sy;
		const uint8_t *row = s->bitmap + y * 32;
		for (int sx = 0; sx < SCREEN_W; sx++) {
			int x = fx ? sx : 255 - sx;
			int i = x & 7, o = x >> 3;
			int c = ((row[o] >> i) & 1) | ((row[o + 0x2000] >> i) & 1) << 1 | ((row[o + 0x4000] >> i) & 1) << 2;
			if (c)
				d[sx] = (uint16_t)(PAL_BM + c);
		}
	}
}

static void jj_render(void)
{
	const snap_t *s = s_snap;
	memcpy(rbx_video.palette, s_pal_rgb, sizeof(s_pal_rgb));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, h = rbx_video.height;
	const int fx = s->misc & 1, fy = (s->misc >> 1) & 1;
	for (int y = 0; y < h; y++)
		for (int x = 0; x < SCREEN_W; x++)
			bm[y * pitch + x] = PAL_BM;
	if (s->misc & 4) {   // priority (Frog & Spiders level 3)
		if (!(s->misc & 0x40))
			draw_bitmap(s, bm, pitch, fx, fy);
		draw_tilemap(s, bm, pitch, fx, fy);
	} else {
		draw_tilemap(s, bm, pitch, fx, fy);
		if (!(s->misc & 0x40))
			draw_bitmap(s, bm, pitch, fx, fy);
	}
	const uint8_t *spr = s->ram98 + 0x40;
	for (int offs = 0; offs < 0x40; offs += 4) {
		int sx = spr[offs + 3] + 1, sy = spr[offs];
		int flipx = spr[offs + 1] & 0x40, flipy = spr[offs + 1] & 0x80;
		int code = spr[offs + 1] & 0x3f, color = spr[offs + 2] & 7;
		if (fx) { sx = 240 - sx; flipx = !flipx; }
		if (fy) flipy = !flipy;
		else sy = 240 - sy;
		if (offs < 3 * 4) sy++;
		const uint8_t *src = s_sprites + code * 256;
		for (int yy = 0; yy < 16; yy++) {
			int dy = sy + yy;
			if (dy < VIS_Y0 || dy > VIS_Y1)
				continue;
			const uint8_t *r = src + (flipy ? 15 - yy : yy) * 16;
			uint16_t *d = bm + (dy - VIS_Y0) * pitch;
			for (int xx = 0; xx < 16; xx++) {
				int dx = sx + xx;
				uint8_t p = r[flipx ? 15 - xx : xx];
				if (p && dx >= 0 && dx < SCREEN_W)
					d[dx] = (uint16_t)(color * 4 + p);
			}
		}
	}
	if (s_fspider) {
		// bullets: 2 bytes each (y, x); 2 player bullets (white), 6 enemy bullets (yellow)
		const uint8_t *b = s->ram98 + 0xa0;
		for (int offs = 0; offs < 0x10; offs += 2) {
			uint8_t by = (uint8_t)~b[offs], bx = (uint8_t)~b[offs | 1];
			uint16_t bc = (uint16_t)(PAL_BM + (offs < 4 ? 7 : 3));
			if (fy) by ^= 0xff;
			if (fx) bx = (uint8_t)(bx + 8);
			if (by < VIS_Y0 || by > VIS_Y1)
				continue;
			for (int x = bx - 4; x < bx; x++)
				if (x >= 0 && x < SCREEN_W)
					bm[(by - VIS_Y0) * pitch + x] = bc;
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->videoram, s_videoram, sizeof(s_videoram));
	memcpy(s_snap->ram98, s_ram98, sizeof(s_snap->ram98));
	memcpy(s_snap->bitmap, s_bitmap, sizeof(s_bitmap));
	s_snap->misc = s_misc;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void decode(const uint8_t *gfx, uint32_t size, int big, uint8_t *out, int count)
{
	// RGN_FRAC(1,2) planes, MSB plane first, pixel bit 7 first
	const int w = big ? 16 : 8, len = big ? 32 : 8;
	const uint32_t half = size / 2;
	for (int t = 0; t < count; t++)
		for (int y = 0; y < w; y++)
			for (int x = 0; x < w; x++) {
				uint32_t o = (uint32_t)(t * len + (y & 7) + (y >= 8 ? 16 : 0) + (x >= 8 ? 8 : 0));
				int b = 7 - (x & 7);
				int v = ((gfx[o] >> b) & 1) << 1 | ((gfx[o + half] >> b) & 1);
				out[t * w * w + y * w + x] = (uint8_t)v;
			}
}

static void jj_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_videoram, 0, sizeof(s_videoram));
	memset(s_ram98, 0, sizeof(s_ram98));
	memset(s_bitmap, 0, sizeof(s_bitmap));
	z80_init(&s_z);
	s_z.read_byte = main_read;
	s_z.write_byte = main_write;
	s_z.port_in = no_in;
	s_z.port_out = no_out;
	s_z.int_data = 0xff;
	s_nmi_line = 0;
	misc_w(0);
	ay8910_reset(&s_ay);
	s_ay_frac = 0;
	s_mix_n = 0;
}

static void jj_exit(void)
{
	free(s_chars);
	free(s_chars2);
	free(s_sprites);
	free(s_snap);
	free(s_mix);
	s_chars = s_chars2 = s_sprites = NULL;
	s_snap = NULL;
	s_mix = NULL;
}

static int jj_init(const rbx_game *g)
{
	uint32_t g1s = 0, g2s = 0, g3s = 0, ps = 0;
	s_fspider = !strcmp(g->machine, "fspider");
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *g1 = rbx_region("gfx1", &g1s), *g2 = rbx_region("gfx2", &g2s), *g3 = rbx_region("gfx3", &g3s);
	uint8_t *prom = rbx_region("proms", &ps);
	if (!s_rom || s_rom_size < 0x8000 || !g1 || g1s < 0x1000 || !g2 || g2s < 0x1000 || !g3 || !prom || ps < 0x20) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_nchars2 = (int)(g3s / 16);
	s_chars = (uint8_t *)malloc(256 * 64);
	s_chars2 = (uint8_t *)malloc((size_t)s_nchars2 * 64);
	s_sprites = (uint8_t *)malloc(64 * 256);
	s_snap = (snap_t *)calloc(1, sizeof(snap_t));
	if (!s_chars || !s_chars2 || !s_sprites || !s_snap) {
		jj_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	decode(g1, 0x1000, 0, s_chars, 256);
	decode(g3, g3s, 0, s_chars2, s_nchars2);
	decode(g2, 0x1000, 1, s_sprites, 64);
	for (int i = 0; i < 32; i++) {
		uint8_t c = prom[i];
		int r = 0x21 * (c & 1) + 0x47 * ((c >> 1) & 1) + 0x97 * ((c >> 2) & 1);
		int gg = 0x21 * ((c >> 3) & 1) + 0x47 * ((c >> 4) & 1) + 0x97 * ((c >> 5) & 1);
		int b = 0x52 * ((c >> 6) & 1) + 0xad * ((c >> 7) & 1);
		s_pal_rgb[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	for (int i = 0; i < 8; i++)   // RGB_3BIT
		s_pal_rgb[PAL_BM + i] = (uint32_t)((i & 1 ? 0xff0000 : 0) | (i & 2 ? 0xff00 : 0) | (i & 4 ? 0xff : 0));
	s_port_dsw1 = rbx_port_find("DSW1");
	s_port_dsw2 = rbx_port_find("DSW2");
	s_port_inputs = rbx_port_find("INPUTS");
	s_port_system = rbx_port_find("SYSTEM");
	// MAME route 0.5, legacy output
	ay8910_init(&s_ay, AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 128, 4, 1);
	s_mix_cap = (int)(ay8910_rate(&s_ay) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(ay8910_rate(&s_ay)) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 40)) {
		jj_exit();
		return -1;
	}
	jj_reset();
	return 0;
}

static void jj_frame(void)
{
	rbx_audio.frame_mhz = LINE_RATE * 1000 / LINES;   // (the board's frame rate: a frame's sound is played in a 60 Hz frame, as its picture)
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			if ((s_misc & 0x80) && !s_nmi_line) {   // vblank_irq: assert NMI (edge)
				s_nmi_line = 1;
				z80_gen_nmi(&s_z);
			}
		}
		s_line_ay = rbx_line_cycles(&s_ay_frac, AY_CLOCK, LINE_RATE * 1000, 1);
		s_ay_line_done = 0;
		s_line_start = s_z.cyc;
		unsigned long end = s_z.cyc + MAIN_LINE;
		while ((long)(s_z.cyc - end) < 0)
			z80_step(&s_z);
		uint32_t rest = (uint32_t)s_line_ay - s_ay_line_done;
		if (rest)
			s_mix_n += ay8910_render(&s_ay, rest, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int jj_state(void *ctx, int load)
{
	z80 z = s_z;
	int rc = 0;
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "videoram", s_videoram, sizeof(s_videoram));
	rc |= rbx_state_io(ctx, "ram98", s_ram98, sizeof(s_ram98));
	rc |= rbx_state_io(ctx, "bitmap", s_bitmap, sizeof(s_bitmap));
	rc |= rbx_state_io(ctx, "nmi", &s_nmi_line, sizeof(s_nmi_line));
	rc |= rbx_state_io(ctx, "misc", &s_misc, 1);
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
	}
	return 0;
}

static int jj_save(void *ctx) { return jj_state(ctx, 0); }
static int jj_load(void *ctx) { return jj_state(ctx, 1); }
static int jj_supported(const char *machine) { return !strcmp(machine, "jollyjgr") || !strcmp(machine, "fspider"); }

const rbx_driver rbx_drv_jollyjgr = { "Jolly Jogger", jj_init, jj_exit, jj_reset, jj_frame, jj_save, jj_load,
                                      jj_supported, jj_render };
