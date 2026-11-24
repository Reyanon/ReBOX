/*  Irem M58: 10-Yard Fight and Vs 10-Yard Fight. Ported from MAME src/mame/irem/m58.cpp (BSD-3-Clause, copyright-holders:
    Lee Taylor). A Z80 at 3.07 MHz, a 64x32 tilemap of 3 bpp tiles, pairs of 16x16 sprites and a
    radar panel of bits on the right; the sound board of machine/irem_audio.c.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../cheat_cpu.h"
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
#define VY0 42
#define VW 256
#define VH 224
#define PEN_SP 512             // (the tiles' 256, the radar's 256, the sprites' 256)
#define NPENS (PEN_SP + 256)
#define PANEL_W 56

static uint8_t *s_rom;
static uint32_t s_rom_size;
static rbx_gfx s_tiles, s_spr;
static z80 s_z;
static const uint8_t *s_map[256];
static uint32_t s_mask[32];
static int s_port[5];

static struct {
	uint8_t vram[0x1000], spr[0x60], ram[0x1000];
	uint16_t panel[256][PANEL_W];
	uint8_t sxl, sxh, syl, panel_off, flip, irq;
	unsigned long main_end;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct {
	uint8_t vram[0x1000], spr[0x60];
	uint16_t panel[256][PANEL_W];
	uint8_t sxl, sxh, syl, panel_off, flip;
} s_vs;

static void capture(void)
{
	memcpy(s_vs.vram, t.vram, sizeof(s_vs.vram));
	memcpy(s_vs.spr, t.spr, sizeof(s_vs.spr));
	memcpy(s_vs.panel, t.panel, sizeof(s_vs.panel));
	s_vs.sxl = t.sxl;
	s_vs.sxh = t.sxh;
	s_vs.syl = t.syl;
	s_vs.panel_off = t.panel_off;
	s_vs.flip = t.flip;
}

static inline int tile_offs(int col, int row)   // (tilemap_scan_rows: the second half of the columns after the first)
{
	return col >= 32 ? (row + 32) * 32 + col - 32 : row * 32 + col;
}

static void m58_render(void)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, flip = s_vs.flip;
	const int sx0 = s_vs.sxh * 256 + s_vs.sxl, sy0 = s_vs.syl;
	// (set_scrolldy(26, 26); flipped, MAME's effective scroll over the 384 x 282 screen)
	const int vx = flip ? 384 - 512 + sx0 : -sx0, vy = flip ? 282 - 256 - 26 + sy0 : 26 - sy0;
	for (int y = 0; y < VH; y++) {
		const int py = (y + VY0 - vy) & 255, ly = flip ? 255 - py : py;
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < VW;) {
			const int px = (x - vx) & 511, lx = flip ? 511 - px : px;
			const int offs = tile_offs(lx >> 3, ly >> 3) * 2;
			const int attr = s_vs.vram[offs + 1];
			const uint32_t code = rbx_wrap(s_vs.vram[offs] + (uint32_t)((attr & 0xc0) << 2), s_tiles.count);
			const int fx = attr & 0x20;
			const uint8_t *s = s_tiles.pix + code * 64 + (ly & 7) * 8;
			const uint32_t base = (uint32_t)(attr & 0x1f) * 8;
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
	for (int offs = (int)sizeof(s_vs.spr) - 4; offs >= 0; offs -= 4) {
		const uint8_t *sr = s_vs.spr + offs;
		const int attr = sr[1], bank = (attr & 0x20) >> 5;
		int code1 = sr[2] & 0xbf, code2;
		const int color = attr & 0x1f;
		int flipx = attr & 0x40, flipy = attr & 0x80;
		int sx = sr[3], sy1 = 210 - sr[0], sy2;
		if (flipy) {
			code2 = code1;
			code1 += 0x40;
		} else
			code2 = code1 + 0x40;
		if (flip) {
			sx = 240 - sx;
			sy2 = 192 - sy1;
			sy1 = sy2 + 0x10;
			flipx = !flipx;
			flipy = !flipy;
		} else
			sy2 = sy1 + 0x10;
		const uint32_t c1 = rbx_wrap((uint32_t)code1 + 256 * (uint32_t)bank, s_spr.count);
		const uint32_t c2 = rbx_wrap((uint32_t)code2 + 256 * (uint32_t)bank, s_spr.count);
		const uint32_t base = PEN_SP + (uint32_t)color * 8;
		irem_blit(bm, pitch, 0, 0, VW, VH, s_spr.pix + c1 * 256, 16, 16, base, s_mask[color], flipx, flipy, sx, sy1);
		irem_blit(bm, pitch, 0, 0, VW, VH, s_spr.pix + c2 * 256, 16, 16, base, s_mask[color], flipx, flipy, sx, sy2);
	}
	if (!s_vs.panel_off) {   // (draw_panel; flipped: the panel turned round at the left)
		for (int y = 0; y < VH; y++) {
			const int yp = y + VY0 - (flip ? 2 : 26);
			if (yp < 0 || yp >= 256 || yp < (flip ? 8 : 8) + (flip ? 0 : 0))
				continue;
			uint16_t *d = bm + y * pitch;
			if (flip) {
				for (int x = 0; x < 48; x++)
					d[x] = t.panel[255 - yp][PANEL_W - 9 - x + 1];
			} else {
				for (int x = 208; x < 256; x++)
					d[x] = s_vs.panel[yp][x - 200];
			}
		}
	}
}

// ---- the Z80's bus ----
static uint8_t rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x6000)
		return s_rom[a];
	if (a >= 0x8000 && a < 0x9000)
		return t.vram[a - 0x8000];
	if (a >= 0xc820 && a < 0xc880)
		return t.spr[a - 0xc820];
	if (a >= 0xe000 && a < 0xf000)
		return t.ram[a - 0xe000];
	switch (a) {
	case 0xa000: return t.sxl;
	case 0xa200: return t.sxh;
	case 0xa400: return t.syl;
	case 0xa800: return t.panel_off;
	}
	if (a >= 0xd000 && a <= 0xd004)
		return (uint8_t)rbx_port_read(s_port[a - 0xd000]);
	return 0;
}

static void wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x9000)
		t.vram[a - 0x8000] = v;
	else if (a >= 0x9000 && a < 0xa000) {   // (scroll_panel_w)
		const int offset = a - 0x9000, sy = offset / 16;
		int sx = offset % 16;
		if (sx < 1 || sx > 14)
			return;
		sx = 4 * (sx - 1);
		for (int i = 0; i < 4; i++) {
			int col = (v >> i) & 0x11;
			col = ((col >> 3) | col) & 3;
			t.panel[sy][sx + i] = (uint16_t)(0x100 + (sy & 0xfc) + col);
		}
	} else if (a >= 0xc820 && a < 0xc880)
		t.spr[a - 0xc820] = v;
	else if (a >= 0xe000 && a < 0xf000)
		t.ram[a - 0xe000] = v;
	else if (a == 0xa000)
		t.sxl = v;
	else if (a == 0xa200)
		t.sxh = v;
	else if (a == 0xa400)
		t.syl = v;
	else if (a == 0xa800)
		t.panel_off = v;
	else if (a == 0xd000)
		irem_audio_cmd_w(v);
	else if (a == 0xd001)   // (the flip is both software and hardware)
		t.flip = (uint8_t)((v & 1) ^ (~rbx_port_read(s_port[4]) & 1));
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
static void m58_exit(void);
static void m58_reset(void);

static void palette(const uint8_t *prom)
{
	static const int res3[3] = { 1000, 470, 220 }, res2[2] = { 470, 220 };
	double wr[2], wg[3], wb[3];
	const double s2 = irem_res_raw(res2, 2, 0, wr), s3 = irem_res_raw(res3, 3, 0, wg);
	irem_res_raw(res3, 3, 0, wb);
	const double scale = 255.0 / (s2 > s3 ? s2 : s3);
	uint32_t *pal = rbx_video.palette;
	#define W2(w, v) ((int)(scale * ((w)[0] * ((v) & 1) + (w)[1] * (((v) >> 1) & 1)) + 0.5))
	for (int i = 0; i < 512; i++) {   // (the characters', the radar's)
		const uint8_t *lo = prom + (i < 256 ? 0x000 : 0x320 - 256), *hi = prom + (i < 256 ? 0x100 : 0x420 - 256);
		const int v = (lo[i] & 0x0f) | (hi[i] << 4 & 0xf0);
		pal[i] = (uint32_t)(W2(wr, v >> 6) << 16 | irem_res3(wg, scale, v >> 3) << 8 | irem_res3(wb, scale, v));
	}
	double wr2[2], wg2[3], wb2[3];   // (the sprites: over 470 ohms, the same scale)
	irem_res_raw(res2, 2, 470, wr2);
	irem_res_raw(res3, 3, 470, wg2);
	irem_res_raw(res3, 3, 470, wb2);
	uint32_t spc[16];
	for (int i = 0; i < 16; i++) {
		const int v = prom[0x200 + i];
		spc[i] = (uint32_t)(W2(wr2, v >> 6) << 16 | irem_res3(wg2, scale, v >> 3) << 8 | irem_res3(wb2, scale, v));
	}
	#undef W2
	memset(s_mask, 0, sizeof(s_mask));
	for (int i = 0; i < 256; i++) {
		const int c = prom[0x220 + i] & 0x0f;
		pal[PEN_SP + i] = spc[c];
		if (!c)   // (transpen_mask: the pens on 512)
			s_mask[i >> 3] |= 1u << (i & 7);
	}
}

static int m58_init(const rbx_game *g)
{
	rbx_cheat_prefer("z80");   // (the cheats' main CPU: another CPU is set up first)
	uint32_t ts, ss, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	const uint8_t *tiles = rbx_region("tiles", &ts), *spr = rbx_region("sprites", &ss), *prom = rbx_region("proms", &ps);
	if (!s_rom || s_rom_size < 0x6000 || !tiles || !spr || !prom || ps < 0x520) {
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
	uint32_t sp[3] = { ss / 3 * 16, ss / 3 * 8, 0 }, x16[16], y16[16];
	for (int i = 0; i < 16; i++) {
		x16[i] = (uint32_t)(i < 8 ? i : 16 * 8 + i - 8);
		y16[i] = (uint32_t)i * 8;
	}
	const rbx_gfx_layout sl = { 16, 16, 3, sp, x16, y16, 32 * 8 };
	const int bad = rbx_gfx_decode(&s_tiles, &tl, tiles, ts / 3) | rbx_gfx_decode(&s_spr, &sl, spr, ss / 3);
	static const char *const pn[5] = { "IN0", "IN1", "IN2", "DSW1", "DSW2" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(pn[i]);
	if (bad || rbx_video_setup(VW, VH, NPENS) || irem_audio_init(IREM_AUDIO_M52_LARGE, HTOTAL, PIX_CLOCK)) {
		rbx_fail("%s: out of memory", g->set);
		m58_exit();
		return -1;
	}
	rbx_audio.frame_mhz = FRAME_MHZ;
	palette(prom);
	m58_reset();
	return 0;
}

static void m58_exit(void)
{
	irem_audio_exit();
	rbx_gfx_free(&s_tiles);
	rbx_gfx_free(&s_spr);
}

static void m58_reset(void)
{
	irem_audio_reset();
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	memset(&s_vs, 0, sizeof(s_vs));
	z80_init(&s_z);
	z_fix(&s_z);
	s_z.int_data = 0xff;
	memset(s_map, 0, sizeof(s_map));
	for (int p = 0; p < 0x60; p++)
		s_map[p] = s_rom + p * 256;
	for (int p = 0; p < 0x10; p++) {
		s_map[0x80 + p] = t.vram + p * 256;
		s_map[0xe0 + p] = t.ram + p * 256;
	}
}

static void m58_frame(void)
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

static int m58_state(void *ctx, int load)
{
	int rc = 0;
	rc |= irem_audio_state(ctx, load);
	rc |= rbx_state_io(ctx, "m58", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "m58_z80", &z, sizeof(z));
	if (!rc && load) {
		z_fix(&z);
		s_z = z;
	}
	return rc ? -1 : 0;
}

static int m58_save(void *ctx) { return m58_state(ctx, 0); }
static int m58_load(void *ctx) { return m58_state(ctx, 1); }
static int m58_supported(const char *machine) { return !strcmp(machine, "yard"); }

const rbx_driver rbx_drv_m58 = { "Irem M58", m58_init, m58_exit, m58_reset, m58_frame, m58_save, m58_load, m58_supported,
                                 m58_render, NULL };
