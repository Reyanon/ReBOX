/*  Irem Traverse USA / Zippy Race and Seibu's Shot Rider. Ported from MAME src/mame/irem/travrusa.cpp (BSD-3-Clause,
    copyright-holders: Lee Taylor). A Z80, a 64x32 tilemap of 3 bpp tiles split front and back (the tunnels) around 16x16
    sprites, its last quarter still; the sound board of machine/irem_audio.c.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../video/gfxlayout.h"
#include "../machine/irem_audio.h"
#include "../video/irem_draw.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define PIX_CLOCK 6144000u     // (18.432 MHz / 3)
#define HTOTAL 384u
#define LINES 282
#define FRAME_MHZ ((uint32_t)((uint64_t)PIX_CLOCK * 1000 / (HTOTAL * LINES)))
#define VX0 8
#define VW 240
#define VH 256
#define NPENS 256

static uint8_t *s_rom;
static uint32_t s_rom_size;
static rbx_gfx s_tiles, s_spr;
static z80 s_z;
static const uint8_t *s_map[256];
static int s_port[5], s_line_cyc, s_shtriderb, s_mirror;

static struct {
	uint8_t vram[0x1000], spr[0x200], ram[0x1000];
	uint8_t scroll[2], flip;
	unsigned long main_end;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t vram[0x1000], spr[0x200], scroll[2], flip; } s_vs;

static void capture(void)
{
	memcpy(s_vs.vram, t.vram, sizeof(s_vs.vram));
	memcpy(s_vs.spr, t.spr, sizeof(s_vs.spr));
	memcpy(s_vs.scroll, t.scroll, 2);
	s_vs.flip = t.flip;
}

// (the tilemap, set_scroll_rows(4), the last group still; flipped, MAME's effective scroll over the 384 x 282 screen)
static void draw_tilemap(int front)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, flip = s_vs.flip;
	const int sc = s_vs.scroll[0] + 256 * s_vs.scroll[1];
	const int vy = flip ? LINES - 256 : 0;
	for (int pr = 0; pr < 32; pr++) {
		const int r = flip ? 31 - pr : pr;
		const int rs = r >= 24 ? 0 : sc;
		const int vx = flip ? (int)HTOTAL - 512 + rs : -rs;
		const int y = (pr * 8 + vy) & 255;
		for (int pc = 0; pc < 64; pc++) {
			const int x = (pc * 8 + vx - VX0) & 511;
			if (x >= VW && x < 504)
				continue;
			const int c = flip ? 63 - pc : pc, offs = (r * 64 + c) * 2;
			const int attr = s_vs.vram[offs + 1];
			if (front && (attr & 0x0f) != 0x0f)
				continue;
			const uint32_t code = rbx_wrap(s_vs.vram[offs] + (uint32_t)((attr & 0xc0) << 2), s_tiles.count);
			const int fx = ((attr & 0x20) != 0) ^ flip, fy = ((attr & 0x10) != 0) ^ flip;   // (TILE_FLIPXY)
			const uint8_t *src = s_tiles.pix + code * 64;
			const uint32_t base = (uint32_t)(attr & 0x0f) * 8, tm = front ? 0x3f : 0;
			const int xx = x >= 504 ? x - 512 : x;
			irem_blit(bm, pitch, 0, 0, VW, VH, src, 8, 8, base, tm, fx, fy, xx, y);
			if (y > 248)
				irem_blit(bm, pitch, 0, 0, VW, VH, src, 8, 8, base, tm, fx, fy, xx, y - 256);
		}
	}
}

static void tr_render(void)
{
	uint16_t *bm = rbx_video.bitmap;
	draw_tilemap(0);
	const int y0 = s_vs.flip ? 8 * 8 : 0, y1 = s_vs.flip ? 32 * 8 : 24 * 8;   // (the sprites' visible area, x 1*8 to 31*8)
	for (int offs = 0x200 - 4; offs >= 0; offs -= 4) {
		const uint8_t *sr = s_vs.spr + offs;
		int sx = ((sr[3] + 8) & 0xff) - 8, sy = 240 - sr[0];
		const uint32_t code = rbx_wrap(sr[2], s_spr.count);
		const int attr = sr[1];
		int flipx = attr & 0x40, flipy = attr & 0x80;
		if (s_vs.flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			flipx = !flipx;
			flipy = !flipy;
		}
		if (s_spr.used[code])
			irem_blit(bm, rbx_video.pitch, 8 - VX0, y0, 31 * 8 - VX0, y1, s_spr.pix + code * 256, 16, 16, 128 + (uint32_t)(attr & 0x0f) * 8, 1,
			          flipx, flipy, sx - VX0, sy);
	}
	draw_tilemap(1);
	if (s_mirror) {   // (ROT270 | ORIENTATION_FLIP_X: rbx_game.rot has the rotation only; the rows turned, the picture mirrored)
		uint16_t row[VW];
		for (int y = 0; y < VH / 2; y++) {
			uint16_t *a = bm + y * rbx_video.pitch, *b = bm + (VH - 1 - y) * rbx_video.pitch;
			memcpy(row, a, sizeof(row));
			memcpy(a, b, sizeof(row));
			memcpy(b, row, sizeof(row));
		}
	}
}

// ---- the Z80's bus ----
static uint8_t rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_rom_size ? s_rom[a] : 0;
	if (a < 0x9000)
		return t.vram[a - 0x8000];
	if (a >= 0xd000 && a <= 0xd004)
		return (uint8_t)rbx_port_read(s_port[a - 0xd000]);
	if (a >= 0xe000 && a < 0xf000)
		return t.ram[a - 0xe000];
	return 0;
}

static void wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x9000)
		t.vram[a - 0x8000] = v;
	else if (a == 0x9000)
		t.scroll[0] = v;
	else if (a == 0xa000)
		t.scroll[1] = v;
	else if (a >= 0xc800 && a < 0xca00)
		t.spr[a - 0xc800] = v;
	else if (a == 0xd000)
		irem_audio_cmd_w(v);
	else if (a == 0xd001)   // (the flip is both software and hardware)
		t.flip = (uint8_t)((v & 1) ^ (~rbx_port_read(s_port[4]) & 1));
	else if (a >= 0xe000 && a < 0xf000)
		t.ram[a - 0xe000] = v;
}

static uint8_t pin(z80 *z, uint8_t p)
{
	(void)z;
	return s_shtriderb && p == 0x11 ? 0x88 : 0;   // (shtriderb_port11_r)
}
static void pout(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void z_fix(z80 *z)
{
	z->read_byte = rd;
	z->write_byte = wr;
	z->port_in = pin;
	z->port_out = pout;
	z->userdata = NULL;
	z->read_map = s_map;
	z->op_rom = NULL;
}

// ---- init / frame ----
static void tr_exit(void);
static void tr_reset(void);

static void palette(const uint8_t *prom, int sh)
{
	uint32_t *pal = rbx_video.palette, c[0x90];
	#define B(v, n) (((v) >> (n)) & 1)
	#define C3(b0, b1, b2) (0x21 * (b0) + 0x47 * (b1) + 0x97 * (b2))
	#define C2(b0, b1) (0x52 * (b0) + 0xad * (b1))
	for (int i = 0; i < 0x90; i++) {
		const int v = i < 0x80 ? prom[i] : prom[i - 0x80 + 0x200];
		int r, g, b;
		if (!sh) {
			r = C2(B(v, 6), B(v, 7));
			g = C3(B(v, 3), B(v, 4), B(v, 5));
			b = C3(B(v, 0), B(v, 1), B(v, 2));
		} else if (i < 0x80) {
			const int w = prom[i + 0x100];
			r = C3(0, B(v, 2), B(v, 3));
			g = C3(B(w, 3), B(v, 0), B(v, 1));
			b = C3(B(w, 0), B(w, 1), B(w, 2));
		} else {
			r = C3(0, B(v, 6), B(v, 7));
			g = C3(B(v, 3), B(v, 4), B(v, 5));
			b = C3(B(v, 0), B(v, 1), B(v, 2));
		}
		c[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	#undef C2
	#undef C3
	#undef B
	for (int i = 0; i < 0x80; i++) {
		pal[i] = c[i];
		pal[0x80 + i] = c[(prom[0x220 + i] & 0x0f) | 0x80];
	}
}

static inline uint8_t bits8(uint8_t v, int b7, int b6, int b5, int b4, int b3, int b2, int b1, int b0)
{
	return (uint8_t)(((v >> b7) & 1) << 7 | ((v >> b6) & 1) << 6 | ((v >> b5) & 1) << 5 | ((v >> b4) & 1) << 4 |
	                 ((v >> b3) & 1) << 3 | ((v >> b2) & 1) << 2 | ((v >> b1) & 1) << 1 | ((v >> b0) & 1));
}

static int tr_init(const rbx_game *g)
{
	uint32_t ts, ss, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	const uint8_t *tiles = rbx_region("tiles", &ts), *spr = rbx_region("sprites", &ss), *prom = rbx_region("proms", &ps);
	if (!s_rom || s_rom_size < 0x8000 || !tiles || !spr || !prom || ps < 0x320) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (!strcmp(g->set, "motorace")) {   // (init_motorace: the first ROM's address and data lines scrambled)
		static uint8_t buf[0x2000];
		memcpy(buf, s_rom, sizeof(buf));
		for (int a = 0; a < 0x2000; a++) {
			static const uint8_t sw[13] = { 11, 0, 2, 4, 6, 8, 10, 12, 1, 3, 5, 7, 9 };   // (bit 0 up)
			int j = 0;
			for (int b = 0; b < 13; b++)
				j |= ((a >> sw[b]) & 1) << b;
			s_rom[j] = bits8(buf[a], 2, 7, 4, 1, 6, 3, 0, 5);
		}
	} else if (!strcmp(g->set, "shtridera")) {   // (init_shtridra: D3 / D4 and D5 / D6 swapped)
		for (int a = 0; a < 0x2000; a++)
			s_rom[a] = bits8(s_rom[a], 7, 5, 6, 3, 4, 2, 1, 0);
	}
	memset(&t, 0, sizeof(t));
	const int sh = !strcmp(g->machine, "shtrider");
	s_shtriderb = !strcmp(g->machine, "shtriderb");
	s_mirror = sh || s_shtriderb;
	s_line_cyc = sh ? 375 : (int)HTOTAL / 2;
	uint32_t x8[8], y16[16], p3[3] = { ts / 3 * 16, ts / 3 * 8, 0 }, ps3[3] = { ss / 3 * 16, ss / 3 * 8, 0 }, x16[16], ysh[16], xsh[16];
	for (int i = 0; i < 16; i++) {
		y16[i] = (uint32_t)i * 8;
		x16[i] = (uint32_t)(i < 8 ? i : 16 * 8 + i - 8);
		xsh[i] = (uint32_t)(i < 8 ? i : 8 * 8 + i - 8);
		ysh[i] = (uint32_t)(i < 8 ? i * 8 : (16 + i - 8) * 8);
		if (i < 8)
			x8[i] = (uint32_t)i;
	}
	const rbx_gfx_layout tl = { 8, 8, 3, p3, x8, y16, 64 };
	const rbx_gfx_layout sl = { 16, 16, 3, ps3, x16, y16, 32 * 8 }, s2 = { 16, 16, 3, ps3, xsh, ysh, 32 * 8 };
	const int bad = rbx_gfx_decode(&s_tiles, &tl, tiles, ts / 3) | rbx_gfx_decode(&s_spr, sh || s_shtriderb ? &s2 : &sl, spr, ss / 3);
	static const char *const pn[5] = { "SYSTEM", "P1", "P2", "DSW1", "DSW2" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(pn[i]);
	if (bad || rbx_video_setup(VW, VH, NPENS) || irem_audio_init(IREM_AUDIO_M52_SOUNDC, HTOTAL, PIX_CLOCK)) {
		rbx_fail("%s: out of memory", g->set);
		tr_exit();
		return -1;
	}
	rbx_audio.frame_mhz = FRAME_MHZ;
	palette(prom, sh);
	tr_reset();
	return 0;
}

static void tr_exit(void)
{
	irem_audio_exit();
	rbx_gfx_free(&s_tiles);
	rbx_gfx_free(&s_spr);
}

static void tr_reset(void)
{
	irem_audio_reset();
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	memset(&s_vs, 0, sizeof(s_vs));
	z80_init(&s_z);
	z_fix(&s_z);
	s_z.int_data = 0xff;
	memset(s_map, 0, sizeof(s_map));
	for (int p = 0; p < 0x80; p++)
		s_map[p] = s_rom + p * 256;
	for (int p = 0; p < 0x10; p++) {
		s_map[0x80 + p] = t.vram + p * 256;
		s_map[0xe0 + p] = t.ram + p * 256;
	}
}

static void tr_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		if (line == VH) {   // (the update)
			rbx_video_wait_idle();
			capture();
			rbx_video_frame_done();
		}
		t.main_end += (unsigned long)s_line_cyc;
		int irq = line >= VH;   // (screen_vblank: IRQ0 held through the vblank, taken again and again)
		z80_run(&s_z, t.main_end, &irq, 0);
		irem_audio_line();
	}
	irem_audio_frame();
	t.frame++;
}

static int tr_state(void *ctx, int load)
{
	int rc = 0;
	rc |= irem_audio_state(ctx, load);
	rc |= rbx_state_io(ctx, "tr", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "tr_z80", &z, sizeof(z));
	if (!rc && load) {
		z_fix(&z);
		s_z = z;
	}
	return rc ? -1 : 0;
}

static int tr_save(void *ctx) { return tr_state(ctx, 0); }
static int tr_load(void *ctx) { return tr_state(ctx, 1); }
static int tr_supported(const char *machine)
{
	return !strcmp(machine, "travrusa") || !strcmp(machine, "shtrider") || !strcmp(machine, "shtriderb");
}

const rbx_driver rbx_drv_travrusa = { "Irem Traverse USA", tr_init, tr_exit, tr_reset, tr_frame, tr_save, tr_load, tr_supported,
                                      tr_render, NULL };
