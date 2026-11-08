/*  Data East Competition Golf Final Round, from MAME src/mame/dataeast/compgolf.cpp (BSD-3-Clause,
    copyright-holders: Angelo Salese, Pierpaolo Prazzoli, Bryan McPhail).
    A 6809 at 2 MHz (NMI the vblank) with a YM2203 (its IRQ the CPU's IRQ; its ports: the background's scroll LSBs),
    a 512x512 background of 16x16 tiles in 3 bpp, a 3 bpp 8x8 text layer, 24 byte-coded sprites over both, the PROM
    palette. MAME's screen: 60 Hz, 256x256 with a 2.5 ms vblank (here 288 lines: 8 .. 247 shown, 248 .. 287 the
    vblank), 248x240 shown.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 2000000        // HD68B09EP
#define YM_CLOCK 1500000
#define FRAME_MHZ 60000          // (60 Hz)
#define LINES 288
#define VIS_X0 8
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 248

static uint8_t *s_rom, *s_bank_rom;
static uint32_t s_rom_size, s_bank_size;
static uint8_t s_ram[0x2100];              // 0x0000 .. 0x20ff (RAM, text, background, sprites)
static int s_bank, s_scrollx_lo, s_scrollx_hi, s_scrolly_lo, s_scrolly_hi;
static m6809_t s_cpu;
static rbx_gfx s_gfx8, s_gfx16, s_gfxspr;
static uint32_t s_palette[256];
static int s_port_p[2], s_port_dsw[2];
static int s_line;
static uint64_t s_cpu_frac, s_ym_frac;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static struct {
	uint8_t txt[0x800], bg[0x800], spr[0x60];
	int scrollx, scrolly;
} *s_snap;

#define TXT 0x1000   // (s_ram offsets)
#define BG 0x1800
#define SPR 0x2000

// ---- bus ----
static int vblank(void) { return s_line >= VIS_Y0 + VIS_H; }
static void set_bank(int b)
{
	s_bank = b & 1;
	for (int p = 0; p < 0x40; p++)
		s_cpu.rpage[0x40 + p] = s_bank_rom + (uint32_t)s_bank * 0x4000 + (uint32_t)p * 256;
}
static void ym_irq(int state) { m6809_set_irq(&s_cpu, state); }
static void ym_porta_w(uint8_t v) { s_scrollx_lo = v; }
static void ym_portb_w(uint8_t v) { s_scrolly_lo = v; }
static uint8_t cpu_r(void *ctx, uint16_t a)
{
	(void)ctx;
	switch (a) {
	case 0x3000: return (uint8_t)rbx_port_read(s_port_p[0]);
	case 0x3001:
		rbx_port_custom(s_port_p[1], 0x80, vblank() ? 0x80 : 0);
		return (uint8_t)rbx_port_read(s_port_p[1]);
	case 0x3002: return (uint8_t)rbx_port_read(s_port_dsw[0]);
	case 0x3003: return (uint8_t)rbx_port_read(s_port_dsw[1]);
	case 0x3800: case 0x3801: return rbx_ym_read(0, a & 1);
	}
	return 0;
}
static void cpu_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a == 0x3001) {   // ctrl_w: the bank, the scroll MSBs
		set_bank((v & 4) >> 2);
		s_scrollx_hi = (v & 1) << 8;
		s_scrolly_hi = (v & 2) << 7;
	} else if (a == 0x3800 || a == 0x3801)
		rbx_ym_write(0, a & 1, v);
}

// ---- video ----
static void spr_blit(uint32_t code, int colour, int fx, int sx, int sy, uint16_t *bm, int pitch)
{
	if (!s_gfxspr.count)
		return;
	code %= s_gfxspr.count;
	const uint8_t *src = s_gfxspr.pix + code * 256;
	for (int y = 0; y < 16; y++) {
		const int dy = sy + y - VIS_Y0;
		if (dy < 0 || dy >= VIS_H)
			continue;
		const uint8_t *row = src + y * 16;
		for (int x = 0; x < 16; x++) {
			const int dx = sx + x - VIS_X0;
			if (dx < 0 || dx >= VIS_W)
				continue;
			const uint8_t p = row[fx ? 15 - x : x];
			if (p)
				bm[dy * pitch + dx] = (uint16_t)(colour * 8 + p);
		}
	}
}

static void cg_render(void)
{
	memcpy(rbx_video.palette, s_palette, sizeof(s_palette));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const uint8_t *bg = s_snap->bg, *txt = s_snap->txt;
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < VIS_W; x++) {
			// the background (32x32 of 16x16, back_scan, opaque)
			const int px = (x + VIS_X0 + s_snap->scrollx) & 0x1ff, py = (y + VIS_Y0 + s_snap->scrolly) & 0x1ff;
			const int col = px >> 4, row = py >> 4;
			const int idx = (col & 0x0f) + ((row & 0x0f) << 4) + ((col & 0x10) << 4) + ((row & 0x10) << 5);
			const int attr = bg[idx * 2];
			const uint32_t code = s_gfx16.count ? (uint32_t)(bg[idx * 2 + 1] + ((attr & 1) << 8)) % s_gfx16.count : 0;
			const uint8_t p = s_gfx16.pix[(code * 16 + (uint32_t)(py & 15)) * 16 + (uint32_t)(px & 15)];
			d[x] = (uint16_t)(((attr & 0x3e) >> 1) * 8 + p);
			// the text (32x32 of 8x8, pen 0 clear; the colour modulo 16)
			const int qx = x + VIS_X0, qy = y + VIS_Y0;
			const int o = ((qy >> 3) * 32 + (qx >> 3)) * 2;
			const uint32_t c8 = s_gfx8.count ? (uint32_t)(txt[o + 1] | (txt[o] << 8)) % s_gfx8.count : 0;
			const uint8_t q = s_gfx8.pix[(c8 * 8 + (uint32_t)(qy & 7)) * 8 + (uint32_t)(qx & 7)];
			if (q)
				d[x] = (uint16_t)(((txt[o] >> 2) & 15) * 8 + q);
		}
	}
	const uint8_t *sr = s_snap->spr;
	for (int offs = 0; offs < 0x60; offs += 4) {   // draw_sprites (in the list's order)
		const uint32_t code = (uint32_t)(sr[offs + 1] + ((sr[offs] & 0xc0) >> 6) * 0x100);
		const int x = 240 - sr[offs + 3], y = sr[offs + 2];
		const int colour = (sr[offs] & 8) >> 3, fx = (sr[offs] & 4) != 0;
		spr_blit(code, colour, fx, x, y, bm, pitch);
		if (sr[offs] & 0x10)   // double height
			spr_blit(code + 1, colour, fx, x, y + 16, bm, pitch);
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->txt, s_ram + TXT, sizeof(s_snap->txt));
	memcpy(s_snap->bg, s_ram + BG, sizeof(s_snap->bg));
	memcpy(s_snap->spr, s_ram + SPR, sizeof(s_snap->spr));
	s_snap->scrollx = s_scrollx_hi + s_scrollx_lo;
	s_snap->scrolly = s_scrolly_hi + s_scrolly_lo;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void cg_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	s_scrollx_lo = s_scrollx_hi = s_scrolly_lo = s_scrolly_hi = 0;
	set_bank(0);
	m6809_reset(&s_cpu);
	m6809_set_irq(&s_cpu, 0);
	m6809_set_nmi(&s_cpu, 0);
	rbx_ym_reset(0);
	s_mix_n = 0;
	s_line = 0;
	s_cpu_frac = s_ym_frac = 0;
}

static void cg_exit(void);

static int cg_init(const rbx_game *g)
{
	uint32_t csize, tsize, t2size, ssize, psize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_bank_rom = rbx_region("bgdata", &s_bank_size);
	uint8_t *chars = rbx_region("chars", &csize), *tiles = rbx_region("bgtiles", &tsize),
	        *tiles2 = rbx_region("bgtiles2", &t2size), *spr = rbx_region("sprites", &ssize), *prom = rbx_region("proms", &psize);
	if (!s_rom || s_rom_size < 0x10000 || !s_bank_rom || s_bank_size < 0x8000 || !chars || csize < 0x8000 || !tiles ||
	    tsize < 0x10000 || !tiles2 || t2size < 0x4000 || !spr || ssize < 0x18000 || !prom || psize < 0x100) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	for (int x = 0; x < 0x4000; x++) {   // expand_bg: cv04's nibbles as the background's third plane
		tiles[0x8000 + x] = (uint8_t)((tiles2[x] & 0x0f) << 4);
		tiles[0xc000 + x] = tiles2[x] & 0xf0;
	}
	{	// spritelayout (1024, the thirds), tilelayoutbg (512, 3 bpp: the half, 0, 4; 4 columns of 4), tilelayout8
		// (1024, 3 bpp: the half + 4, 0, 4)
		static uint32_t xs[16], ys[16], xt[16], y16[16], x8[8], y8[8];
		for (int i = 0; i < 8; i++) {
			xs[i] = (uint32_t)(8 * 8 * 2 + i);
			xs[8 + i] = (uint32_t)i;
			ys[i] = (uint32_t)(i * 8);
			ys[8 + i] = (uint32_t)(8 * 8 + i * 8);
			y8[i] = (uint32_t)(i * 8);
		}
		for (int i = 0; i < 4; i++) {
			xt[i] = (uint32_t)i;
			xt[4 + i] = (uint32_t)(16 * 8 + i);
			xt[8 + i] = (uint32_t)(2 * 16 * 8 + i);
			xt[12 + i] = (uint32_t)(3 * 16 * 8 + i);
			x8[i] = (uint32_t)i;
			x8[4 + i] = (uint32_t)(8 * 8 + i);
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 8);
		static const uint32_t ps[3] = { 0, 0x8000 * 8, 0x10000 * 8 }, pt[3] = { 0x8000 * 8, 0, 4 },
		                      pc[3] = { 0x4000 * 8 + 4, 0, 4 };
		const rbx_gfx_layout ls = { 16, 16, 3, ps, xs, ys, 16 * 16 };
		const rbx_gfx_layout lt = { 16, 16, 3, pt, xt, y16, 32 * 16 };
		const rbx_gfx_layout l8 = { 8, 8, 3, pc, x8, y8, 16 * 8 };
		if (rbx_gfx_decode(&s_gfxspr, &ls, spr, 1024 * 32) || rbx_gfx_decode(&s_gfx16, &lt, tiles, 512 * 64) ||
		    rbx_gfx_decode(&s_gfx8, &l8, chars, 1024 * 16)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			cg_exit();
			return -1;
		}
	}
	for (int i = 0; i < 256; i++) {   // palette_init: 3-3-2 with the weights 0x21, 0x47, 0x97 (blue's low bit 0)
		const int c = prom[i];
		const int r = 0x21 * (c & 1) + 0x47 * ((c >> 1) & 1) + 0x97 * ((c >> 2) & 1);
		const int gg = 0x21 * ((c >> 3) & 1) + 0x47 * ((c >> 4) & 1) + 0x97 * ((c >> 5) & 1);
		const int b = 0x47 * ((c >> 6) & 1) + 0x97 * ((c >> 7) & 1);
		s_palette[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;   // (YM2203 1.5 MHz, all outputs 1.0)
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 256;
	yc.irq = ym_irq;
	yc.port_a_write = ym_porta_w;
	yc.port_b_write = ym_portb_w;
	int rc = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	if (!rc) {
		s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		rc = !s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 256);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		cg_exit();
		return -1;
	}
	m6809_init(&s_cpu);
	for (int p = 0; p < 0x20; p++)   // (RAM, text, background; 0x0800 .. 0x0fff open)
		if (p < 0x08 || p >= 0x10)
			s_cpu.rpage[p] = s_cpu.wpage[p] = s_ram + p * 256;
	s_cpu.rpage[0x20] = s_cpu.wpage[0x20] = s_ram + SPR;   // (the sprites: 0x2000 .. 0x2060, the rest of the page RAM too)
	for (int p = 0x80; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + p * 256;
	s_cpu.read = cpu_r;
	s_cpu.write = cpu_w;
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	cg_reset();
	return 0;
}

static void cg_exit(void)
{
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
}

static void cg_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // screen_vblank on the NMI
			vblank_capture();
			m6809_set_nmi(&s_cpu, 1);
		} else if (line == 0)
			m6809_set_nmi(&s_cpu, 0);
		s_cpu_frac += (uint64_t)CPU_CLOCK * 1000;
		const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
		const int64_t cyc = (int64_t)(s_cpu_frac / den);
		s_cpu_frac -= (uint64_t)cyc * den;
		m6809_run_until(&s_cpu, s_cpu.cycles + cyc);
		s_mix_n += rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&s_ym_frac, YM_CLOCK, FRAME_MHZ, LINES), s_mix + s_mix_n,
		                          s_mix_cap - s_mix_n);
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int cg_state(void *ctx, int load)
{
	uint8_t cpu[M6809_STATE_SIZE];
	if (!load)
		memcpy(cpu, &s_cpu, M6809_STATE_SIZE);
	uint64_t fr[2] = { s_cpu_frac, s_ym_frac };
	int32_t misc[5] = { s_bank, s_scrollx_lo, s_scrollx_hi, s_scrolly_lo, s_scrolly_hi };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m6809", cpu, M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		memcpy(&s_cpu, cpu, M6809_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_ym_frac = fr[1];
		set_bank(misc[0]);
		s_scrollx_lo = misc[1];
		s_scrollx_hi = misc[2];
		s_scrolly_lo = misc[3];
		s_scrolly_hi = misc[4];
	}
	return rc ? -1 : 0;
}

static int cg_save(void *ctx) { return cg_state(ctx, 0); }
static int cg_load(void *ctx) { return cg_state(ctx, 1); }
static int cg_supported(const char *machine) { return !strcmp(machine, "compgolf"); }

const rbx_driver rbx_drv_compgolf = { "DECO Competition Golf", cg_init, cg_exit, cg_reset, cg_frame, cg_save, cg_load,
                                      cg_supported, cg_render, NULL };
