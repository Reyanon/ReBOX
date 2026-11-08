/*  Irem M57: Tropical Angel. Ported from MAME src/mame/irem/m57.cpp (BSD-3-Clause, copyright-holders: Phil Stroffolino).
    A Z80 at 3.07 MHz, a 32x32 tilemap of 3 bpp tiles with a line-by-line water effect in its lower half, 16x32 sprites;
    the sound board of machine/irem_audio.c.
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
#define LINE_CYC (HTOTAL / 2)
#define FRAME_MHZ ((uint32_t)((uint64_t)PIX_CLOCK * 1000 / (HTOTAL * LINES)))
#define VX0 8
#define VY0 8
#define VW 240
#define VH 240
#define NPENS 512              // (the tiles' 256, the sprites' 256)

static uint8_t *s_rom;
static uint32_t s_rom_size;
static rbx_gfx s_tiles, s_spr[4];
static z80 s_z;
static const uint8_t *s_map[256];
static uint32_t s_mask[32];
static int s_port[5];

static struct {
	uint8_t vram[0x800], scroll[0x200], spr[0xe0], ram[0x800];
	uint8_t flip, irq;
	unsigned long main_end;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t vram[0x800], scroll[0x200], spr[0xe0], flip; } s_vs;

static void capture(void)
{
	memcpy(s_vs.vram, t.vram, sizeof(s_vs.vram));
	memcpy(s_vs.scroll, t.scroll, sizeof(s_vs.scroll));
	memcpy(s_vs.spr, t.spr, sizeof(s_vs.spr));
	s_vs.flip = t.flip;
}

static void m57_render(void)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, flip = s_vs.flip;
	for (int y = 0; y < VH; y++) {
		// (the tilemap; 64-127 scrolled by one value; flipped, MAME's effective scroll over the 384 x 282 screen)
		const int py = (y + VY0 - (flip ? 26 : 0)) & 255, ly = flip ? 255 - py : py;
		const int rs = ly >= 64 && ly < 128 ? s_vs.scroll[0x40] : 0;
		const int value = flip ? 128 + rs : -rs;
		const int row = ly >> 3;
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < VW;) {
			const int px = (x + VX0 - value) & 255, lx = flip ? 255 - px : px;
			const int idx = (row * 32 + (lx >> 3)) * 2;
			const int attr = s_vs.vram[idx];
			const uint32_t code = rbx_wrap(s_vs.vram[idx + 1] | (uint32_t)(attr & 0xc0) << 2, s_tiles.count);
			const int fx = attr & 0x20, fy = attr & 0x10;   // (TILE_FLIPXY)
			const uint8_t *s = s_tiles.pix + code * 64 + (fy ? 7 - (ly & 7) : ly & 7) * 8;
			const uint32_t base = (uint32_t)(attr & 0x0f) * 8;
			int n = 8 - (px & 7);
			if (n > VW - x)
				n = VW - x;
			for (int i = 0; i < n; i++) {
				const int lp = (flip ? lx - i : lx + i) & 7;
				d[x + i] = (uint16_t)(base + s[fx ? 7 - lp : lp]);
			}
			x += n;
		}
	}
	for (int y = 128; y < VY0 + VH; y++) {   // (128 on: wrapped, in the bitmap)
		const int16_t sc = (int16_t)(s_vs.scroll[y] | s_vs.scroll[y + 0x100] << 8);
		uint16_t *d = bm + (y - VY0) * pitch;
		if (sc >= 0) {
			for (int x = 0; x < VW; x++)
				d[x] = x + sc < VW ? d[x + sc] : d[VW - 1];
		} else {
			for (int x = VW - 1; x >= 0; x--)
				d[x] = x + sc >= 0 ? d[x + sc] : d[0];
		}
	}
	for (int offs = (int)sizeof(s_vs.spr) - 4; offs >= 0; offs -= 4) {
		const uint8_t *sr = s_vs.spr + offs;
		const int attr = sr[1];
		int sx = sr[3], sy = ((224 - sr[0] - 32) & 0xff) + 32;
		const int code = sr[2], color = attr & 0x1f;
		int flipy = attr & 0x80, flipx = attr & 0x40;
		const int bank = ((code & 0x80) ? 1 : 0) + ((attr & 0x20) ? 2 : 0);
		if (flip) {
			sx = 240 - sx;
			sy = 224 - sy;
			flipx = !flipx;
			flipy = !flipy;
		}
		irem_blit(bm, pitch, 0, 0, VW, VH, s_spr[bank].pix + (code & 0x3f) * (16 * 32), 16, 32, 256 + (uint32_t)color * 8,
		          s_mask[color], flipx, flipy, sx - VX0, sy - VY0);
	}
}

// ---- the Z80's bus ----
static uint8_t rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_rom_size ? s_rom[a] : 0;
	if (a < 0x8800)
		return t.vram[a - 0x8000];
	if (a >= 0x9000 && a < 0x9200)
		return t.scroll[a - 0x9000];
	if (a >= 0xd000 && a <= 0xd004)
		return (uint8_t)rbx_port_read(s_port[a - 0xd000]);
	if (a >= 0xe000 && a < 0xe800)
		return t.ram[a - 0xe000];
	return 0;
}

static void wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8800)
		t.vram[a - 0x8000] = v;
	else if (a >= 0x9000 && a < 0x9200)
		t.scroll[a - 0x9000] = v;
	else if (a >= 0xc820 && a < 0xc900)
		t.spr[a - 0xc820] = v;
	else if (a == 0xd000)
		irem_audio_cmd_w(v);
	else if (a == 0xd001)   // (the flip is both software and hardware)
		t.flip = (uint8_t)((v & 1) ^ (~rbx_port_read(s_port[4]) & 1));
	else if (a >= 0xe000 && a < 0xe800)
		t.ram[a - 0xe000] = v;
}

static uint8_t pin(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
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
static void m57_exit(void);
static void m57_reset(void);

static void palette(const uint8_t *prom)
{
	uint32_t *pal = rbx_video.palette;
	#define C3(b0, b1, b2) (0x21 * (b0) + 0x47 * (b1) + 0x97 * (b2))
	#define B(v, n) (((v) >> (n)) & 1)
	for (int i = 0; i < 256; i++) {
		const int p0 = prom[i], p1 = prom[256 + i];
		const int r = C3(0, B(p1, 2), B(p1, 3));
		const int g = C3(B(p0, 3), B(p1, 0), B(p1, 1));
		const int b = C3(B(p0, 0), B(p0, 1), B(p0, 2));
		pal[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	uint32_t spc[16];
	const uint8_t *sp = prom + 512;
	for (int i = 0; i < 16; i++) {
		const int v = sp[i];
		const int r = C3(0, B(v, 6), B(v, 7));
		const int g = C3(B(v, 3), B(v, 4), B(v, 5));
		const int b = C3(B(v, 0), B(v, 1), B(v, 2));
		spc[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	#undef C3
	#undef B
	memset(s_mask, 0, sizeof(s_mask));
	const uint8_t *lut = sp + 32;
	for (int i = 0; i < 256; i++) {
		const int c = ~lut[i] & 0x0f;
		pal[256 + i] = spc[c];
		if (c == 15)   // (transpen_mask: the pens on 256 + 15)
			s_mask[i >> 3] |= 1u << (i & 7);
	}
}

static int m57_init(const rbx_game *g)
{
	uint32_t ts, ss, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	const uint8_t *tiles = rbx_region("tiles", &ts), *spr = rbx_region("sprites", &ss), *prom = rbx_region("proms", &ps);
	if (!s_rom || !tiles || !spr || !prom || ps < 0x320 || ss < 0x4000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	uint32_t x8[8], y8[8], tp[3] = { ts / 3 * 16, ts / 3 * 8, 0 };
	for (int i = 0; i < 8; i++) {
		x8[i] = (uint32_t)i;
		y8[i] = (uint32_t)i * 8;
	}
	const rbx_gfx_layout tl = { 8, 8, 3, tp, x8, y8, 64 };
	uint32_t sp[3] = { 0, ss / 3 * 8, ss / 3 * 16 }, x16[16], y32[32];
	for (int i = 0; i < 16; i++)
		x16[i] = (uint32_t)(i < 8 ? i : 16 * 8 + i - 8);
	for (int i = 0; i < 32; i++)
		y32[i] = (uint32_t)(i < 16 ? i * 8 : 256 * 64 + (i - 16) * 8);
	const rbx_gfx_layout sl = { 16, 32, 3, sp, x16, y32, 32 * 8 };
	int bad = rbx_gfx_decode(&s_tiles, &tl, tiles, ts / 3);
	for (int i = 0; i < 4; i++)
		bad |= rbx_gfx_decode(&s_spr[i], &sl, spr + i * 0x1000, 64 * 32);
	static const char *const pn[5] = { "IN0", "IN1", "IN2", "DSW1", "DSW2" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(pn[i]);
	if (bad || rbx_video_setup(VW, VH, NPENS) || irem_audio_init(IREM_AUDIO_M52_SOUNDC, HTOTAL, PIX_CLOCK)) {
		rbx_fail("%s: out of memory", g->set);
		m57_exit();
		return -1;
	}
	rbx_audio.frame_mhz = FRAME_MHZ;
	palette(prom);
	m57_reset();
	return 0;
}

static void m57_exit(void)
{
	irem_audio_exit();
	rbx_gfx_free(&s_tiles);
	for (int i = 0; i < 4; i++)
		rbx_gfx_free(&s_spr[i]);
}

static void m57_reset(void)
{
	irem_audio_reset();
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	memset(&s_vs, 0, sizeof(s_vs));
	z80_init(&s_z);
	z_fix(&s_z);
	s_z.int_data = 0xff;
	memset(s_map, 0, sizeof(s_map));
	for (uint32_t p = 0; p < 0x80 && p * 256 < s_rom_size; p++)
		s_map[p] = s_rom + p * 256;
	for (int p = 0; p < 8; p++) {
		s_map[0x80 + p] = t.vram + p * 256;
		s_map[0xe0 + p] = t.ram + p * 256;
	}
}

static void m57_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		if (line == VY0 + VH) {   // (the update; irq0_line_hold)
			rbx_video_wait_idle();
			capture();
			rbx_video_frame_done();
			t.irq = 1;
		}
		t.main_end += LINE_CYC;
		int irq = t.irq;
		z80_run(&s_z, t.main_end, &irq, 1);
		t.irq = (uint8_t)irq;
		irem_audio_line();
	}
	irem_audio_frame();
	t.frame++;
}

static int m57_state(void *ctx, int load)
{
	int rc = 0;
	rc |= irem_audio_state(ctx, load);
	rc |= rbx_state_io(ctx, "m57", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "m57_z80", &z, sizeof(z));
	if (!rc && load) {
		z_fix(&z);
		s_z = z;
	}
	return rc ? -1 : 0;
}

static int m57_save(void *ctx) { return m57_state(ctx, 0); }
static int m57_load(void *ctx) { return m57_state(ctx, 1); }
static int m57_supported(const char *machine) { return !strcmp(machine, "m57"); }

const rbx_driver rbx_drv_m57 = { "Irem M57", m57_init, m57_exit, m57_reset, m57_frame, m57_save, m57_load, m57_supported,
                                 m57_render, NULL };
