/*  Irem M52: Moon Patrol, Moon Ranger and Alpha One. Ported from MAME src/mame/irem/m52.cpp (BSD-3-Clause,
    copyright-holders: Nicola Salmoria). A Z80 at 3.07 MHz, a 32x32 tilemap of 2 bpp tiles whose lowest quarter scrolls,
    three 256x128 background pictures under it, 16x16 sprites over it; the sound board of machine/irem_audio.c.
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
#define LINES 282u
#define LINE_CYC (HTOTAL / 2)  // (the Z80 at half the pixel clock)
#define FRAME_MHZ ((uint32_t)((uint64_t)PIX_CLOCK * 1000 / (HTOTAL * LINES)))
#define VX0 136
#define VW 240
#define PEN_SP 512             // (the tilemap's 512, the sprites' 256, the backgrounds' 12)
#define PEN_BG (PEN_SP + 256)
#define NPENS (PEN_BG + 12)

static uint8_t *s_rom;
static uint32_t s_rom_size;
static rbx_gfx s_tx, s_sp, s_bg[3];
static z80 s_z;
static const uint8_t *s_map[256];
static uint32_t s_spmask[16];
static int s_alpha, s_vy0, s_vh;
static int s_port[5];

static struct {
	uint8_t vram[0x400], cram[0x400], spr[0x400], ram[0x1000];
	uint8_t bgx[2], bgy[2], bgctl, scroll, flip, irq;
	unsigned long main_end;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t vram[0x400], cram[0x400], spr[0x400], bgx[2], bgy[2], bgctl, scroll, flip; } s_vs;

static void capture(void)
{
	memcpy(s_vs.vram, t.vram, sizeof(s_vs.vram));
	memcpy(s_vs.cram, t.cram, sizeof(s_vs.cram));
	memcpy(s_vs.spr, t.spr, s_alpha ? 0x200 : 0x100);
	memcpy(s_vs.bgx, t.bgx, 2);
	memcpy(s_vs.bgy, t.bgy, 2);
	s_vs.bgctl = t.bgctl;
	s_vs.scroll = t.scroll;
	s_vs.flip = t.flip;
}

static void draw_background(uint16_t *bm, int xpos, int ypos, int image)
{
	const int pitch = rbx_video.pitch, flip = s_vs.flip;
	if (flip) {
		xpos = 264 - xpos;
		ypos = 264 - ypos - 128;
	}
	xpos += 124 - VX0;
	ypos += 16 - s_vy0;
	const uint32_t base = PEN_BG + (uint32_t)image * 4;
	irem_blit(bm, pitch, 0, 0, VW, s_vh, s_bg[image].pix, 256, 128, base, 1, flip, flip, xpos, ypos);
	irem_blit(bm, pitch, 0, 0, VW, s_vh, s_bg[image].pix, 256, 128, base, 1, flip, flip, xpos - 256, ypos);
	if (!s_alpha) {   // (a solid fill below the pictures)
		const int y0 = flip ? ypos - 128 : ypos + 128;
		for (int y = y0 < 0 ? 0 : y0; y < y0 + 128 && y < s_vh; y++) {
			uint16_t *d = bm + y * pitch;
			for (int x = 0; x < VW; x++)
				d[x] = (uint16_t)(base + 3);
		}
	}
}

static void draw_tilemap(uint16_t *bm)
{
	const int pitch = rbx_video.pitch, flip = s_vs.flip;
	const int sc = -(s_vs.scroll + 1);
	for (int y = 0; y < s_vh; y++) {
		// (set_scrolldx(127, 127), set_scrolldy(16, 16), the scroll_rows groups of 64 lines; flipped, MAME's effective
		// scroll over the 384 x 282 screen)
		const int py = (y + s_vy0 - (flip ? 10 : 16)) & 255, ly = flip ? 255 - py : py, g = ly >> 6;
		const int rs = s_alpha ? (g ? sc : 255) : (g == 3 ? sc : 255);
		const int value = flip ? 1 + rs : 127 - rs;
		const int row = ly >> 3, opaque = row <= 6;
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < VW;) {
			const int px = (x + VX0 - value) & 255, lx = flip ? 255 - px : px;
			const int idx = row * 32 + (lx >> 3);
			const int color = s_vs.cram[idx];
			const uint32_t code = s_vs.vram[idx] | (color & 0x80) << 1;
			int n = 8 - (px & 7);
			if (n > VW - x)
				n = VW - x;
			if (!opaque && !s_tx.used[code]) {
				x += n;
				continue;
			}
			const uint8_t *s = s_tx.pix + code * 64 + (ly & 7) * 8;
			const uint32_t base = (uint32_t)(color & 0x7f) * 4;
			for (int i = 0; i < n; i++) {
				const int p = s[(flip ? lx - i : lx + i) & 7];
				if (p || opaque)
					d[x + i] = (uint16_t)(base + (uint32_t)p);
			}
			x += n;
		}
	}
}

static void draw_sprites(uint16_t *bm, int initoffs)
{
	for (int offs = initoffs; offs >= (initoffs & 0xc0); offs -= 4) {
		int sy = 257 - s_vs.spr[offs];
		const int color = s_vs.spr[offs + 1] & 0x0f;
		int flipx = s_vs.spr[offs + 1] & 0x40, flipy = s_vs.spr[offs + 1] & 0x80;
		const uint32_t code = rbx_wrap(s_vs.spr[offs + 2], s_sp.count);
		int sx = s_vs.spr[offs + 3];
		if (s_vs.flip) {
			flipx = !flipx;
			flipy = !flipy;
			sx = 238 - sx;
			sy = 282 - sy;
		}
		sx += 129;
		irem_blit(bm, rbx_video.pitch, 0, 0, VW, s_vh, s_sp.pix + code * 256, 16, 16, PEN_SP + (uint32_t)color * 8, s_spmask[color],
		          flipx, flipy, sx - VX0, sy - s_vy0);
	}
}

static void m52_render(void)
{
	uint16_t *bm = rbx_video.bitmap;
	for (int y = 0; y < s_vh; y++) {
		uint16_t *d = bm + y * rbx_video.pitch;
		for (int x = 0; x < VW; x++)
			d[x] = PEN_SP;
	}
	if (!(s_vs.bgctl & 0x20)) {
		if (!(s_vs.bgctl & 0x10))
			draw_background(bm, s_vs.bgx[1], s_vs.bgy[1], 0);   // (distant mountains)
		if (!(s_vs.bgctl & 0x02))
			draw_background(bm, s_vs.bgx[0], s_vs.bgy[0], 1);   // (hills)
		else if (!(s_vs.bgctl & 0x04))
			draw_background(bm, s_vs.bgx[0], s_vs.bgy[0], 2);   // (cityscape)
	}
	draw_tilemap(bm);
	const int limit = s_alpha ? 0x200 - 4 : 0x100 - 4;
	for (int offs = 0x3c; offs <= limit; offs += 0x40)
		draw_sprites(bm, offs);
}

// ---- the Z80's bus ----
static uint8_t port(int p) { return (uint8_t)rbx_port_read(s_port[p]); }

static uint8_t rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_rom_size ? s_rom[a] : 0;
	if (a < 0x8400)
		return t.vram[a - 0x8000];
	if (a < 0x8800)
		return t.cram[a - 0x8400];
	if (a < 0x9000) {   // (protection_r: a custom chip on the scroll board)
		int pop = 0;
		for (int v = t.bgx[0] & 0x7f; v; v >>= 1)
			pop += v & 1;
		return (uint8_t)(pop ^ (t.bgx[0] >> 7));
	}
	if (a >= 0xd000 && a < 0xd800) {
		const int p = s_alpha ? (a < 0xd005 ? a & 7 : 7) : a & 7;
		return p < 5 ? port(p) : 0;
	}
	if (a >= 0xe000 && a < (s_alpha ? 0xf000 : 0xe800))
		return t.ram[a - 0xe000];
	return 0;
}

static void wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8400)
		t.vram[a - 0x8000] = v;
	else if (a >= 0x8400 && a < 0x8800)
		t.cram[a - 0x8400] = v;
	else if (a >= 0xc800 && a < 0xd000) {
		if (!s_alpha)
			t.spr[a & 0x3ff] = v;
		else if (a < 0xca00)
			t.spr[a - 0xc800] = v;
	} else if (a >= 0xd000 && a < 0xd800) {
		const int r = s_alpha ? (a < 0xd002 ? a & 3 : 3) : a & 3;
		if (r == 0)
			irem_audio_cmd_w(v);
		else if (r == 1)   // (the flip is both software and hardware)
			t.flip = (uint8_t)(s_alpha ? v & 1 : (v & 1) ^ (~port(4) & 1));
	} else if (a >= 0xe000 && a < (s_alpha ? 0xf000 : 0xe800))
		t.ram[a - 0xe000] = v;
}

static uint8_t pin(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }

static void pout(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	switch (p & 0xe0) {
	case 0x00: t.scroll = v; break;
	case 0x40: t.bgx[0] = v; break;
	case 0x60: t.bgy[0] = v; break;
	case 0x80: t.bgx[1] = v; break;
	case 0xa0: t.bgy[1] = v; break;
	case 0xc0: t.bgctl = v; break;
	}
}

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
static void m52_exit(void);
static void m52_reset(void);

static void palette(const uint8_t *tx, const uint8_t *bg, const uint8_t *sp, const uint8_t *clut)
{
	static const int res3[3] = { 1000, 470, 220 }, res2[2] = { 470, 220 };
	double wr[3], wg[3], wb[3];
	const double s3 = irem_res_raw(res3, 3, 0, wr);
	irem_res_raw(res3, 3, 0, wg);
	const double s2 = irem_res_raw(res2, 2, 0, wb);
	const double scale = 255.0 / (s3 > s2 ? s3 : s2);
	uint32_t *pal = rbx_video.palette, bgc[32], spc[32];
	#define W2(w, v) ((int)(scale * ((w)[0] * ((v) & 1) + (w)[1] * (((v) >> 1) & 1)) + 0.5))
	for (int i = 0; i < 512; i++) {
		const int v = tx[i];
		pal[i] = (uint32_t)(irem_res3(wr, scale, v) << 16 | irem_res3(wg, scale, v >> 3) << 8 | W2(wb, v >> 6));
	}
	for (int i = 0; i < 32; i++) {
		const int v = bg[i];
		bgc[i] = (uint32_t)(irem_res3(wr, scale, v) << 16 | irem_res3(wg, scale, v >> 3) << 8 | W2(wb, v >> 6));
	}
	static const uint8_t bgi[12] = { 0, 4, 8, 12, 0, 1, 2, 3, 0, 17, 18, 19 };
	for (int i = 0; i < 12; i++)
		pal[PEN_BG + i] = bgc[bgi[i]];
	double wr2[2], wg2[3], wb2[3];   // (the sprites: over 470 ohms, the same scale)
	irem_res_raw(res2, 2, 470, wr2);
	irem_res_raw(res3, 3, 470, wg2);
	irem_res_raw(res3, 3, 470, wb2);
	for (int i = 0; i < 32; i++) {
		const int v = sp[i];
		spc[i] = (uint32_t)(W2(wr2, v >> 6) << 16 | irem_res3(wg2, scale, v >> 3) << 8 | irem_res3(wb2, scale, v));
	}
	#undef W2
	memset(s_spmask, 0, sizeof(s_spmask));
	for (int i = 0; i < 256; i++) {
		pal[PEN_SP + i] = spc[clut[i] & 31];
		if (i < 128 && !clut[i])   // (transpen_mask: the pens on colour 0)
			s_spmask[i >> 3] |= 1u << (i & 7);
	}
}

static int m52_init(const rbx_game *g)
{
	uint32_t txs, sps, bgs[3], tps, bps, sps2, cps;
	static const char *const bgn[3] = { "bg0", "bg1", "bg2" };
	s_rom = rbx_region("maincpu", &s_rom_size);
	const uint8_t *tx = rbx_region("tx", &txs), *sp = rbx_region("sp", &sps);
	const uint8_t *txp = rbx_region("tx_pal", &tps), *bgp = rbx_region("bg_pal", &bps);
	const uint8_t *spp = rbx_region("spr_pal", &sps2), *clut = rbx_region("spr_clut", &cps);
	const uint8_t *bg[3];
	s_alpha = !strcmp(g->machine, "alpha1v");
	int ok = s_rom && tx && sp && txp && bgp && spp && clut && tps >= 512 && bps >= 32 && sps2 >= 32 && cps >= 256;
	for (int i = 0; i < 3; i++) {
		bg[i] = rbx_region(bgn[i], &bgs[i]);
		ok = ok && bg[i] && bgs[i] >= 0x2000;
	}
	if (!ok) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	s_vy0 = s_alpha ? 16 : 22;
	s_vh = s_alpha ? 256 : 252;
	uint32_t txp_o[2] = { txs * 4, 0 }, x8[8], y8[8];
	for (int i = 0; i < 8; i++) {
		x8[i] = (uint32_t)i;
		y8[i] = (uint32_t)i * 8;
	}
	const rbx_gfx_layout txl = { 8, 8, 2, txp_o, x8, y8, 64 };
	uint32_t spp_o[3] = { sps / 3 * 8 * 2, 0, sps / 3 * 8 }, x16[16], y16[16];
	for (int i = 0; i < 16; i++) {
		x16[i] = (uint32_t)(i < 8 ? i : 16 * 8 + i - 8);
		y16[i] = (uint32_t)i * 8;
	}
	const rbx_gfx_layout spl = { 16, 16, 3, spp_o, x16, y16, 32 * 8 };
	static uint32_t bgx[256], bgy[128];
	static const uint32_t bgp_o[2] = { 4, 0 };
	for (int i = 0; i < 256; i++)
		bgx[i] = (uint32_t)(i >> 2) * 8 + (uint32_t)(i & 3);
	for (int i = 0; i < 128; i++)
		bgy[i] = (uint32_t)(i >> 5) * 0x4000 + (uint32_t)(i & 31) * 0x200;
	const rbx_gfx_layout bgl = { 256, 128, 2, bgp_o, bgx, bgy, 0x10000 };
	if (rbx_gfx_decode(&s_tx, &txl, tx, txs) || rbx_gfx_decode(&s_sp, &spl, sp, sps)) {
		rbx_fail("%s: out of memory", g->set);
		m52_exit();
		return -1;
	}
	for (int i = 0; i < 3; i++)
		if (rbx_gfx_decode(&s_bg[i], &bgl, bg[i], 0x2000)) {
			rbx_fail("%s: out of memory", g->set);
			m52_exit();
			return -1;
		}
	static const char *const pn[5] = { "IN0", "IN1", "IN2", "DSW1", "DSW2" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(pn[i]);
	if (rbx_video_setup(VW, s_vh, NPENS) || irem_audio_init(IREM_AUDIO_M52_SOUNDC, HTOTAL, PIX_CLOCK)) {
		rbx_fail("%s: out of memory", g->set);
		m52_exit();
		return -1;
	}
	rbx_audio.frame_mhz = FRAME_MHZ;
	palette(txp, bgp, spp, clut);
	m52_reset();
	return 0;
}

static void m52_exit(void)
{
	irem_audio_exit();
	rbx_gfx_free(&s_tx);
	rbx_gfx_free(&s_sp);
	for (int i = 0; i < 3; i++)
		rbx_gfx_free(&s_bg[i]);
}

static void m52_reset(void)
{
	irem_audio_reset();
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	memset(&s_vs, 0, sizeof(s_vs));
	z80_init(&s_z);
	z_fix(&s_z);
	s_z.int_data = 0xff;
	memset(s_map, 0, sizeof(s_map));
	for (uint32_t p = 0; p < 0x70 && p * 256 < s_rom_size; p++)
		s_map[p] = s_rom + p * 256;
	for (int p = 0; p < (s_alpha ? 0x10 : 0x08); p++)
		s_map[0xe0 + p] = t.ram + p * 256;
}

static void m52_frame(void)
{
	const int vb = s_vy0 + s_vh;
	for (int line = 0; line < (int)LINES; line++) {
		if (line == vb) {   // (the update; irq0_line_hold)
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

static int m52_state(void *ctx, int load)
{
	int rc = 0;
	rc |= irem_audio_state(ctx, load);
	rc |= rbx_state_io(ctx, "m52", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "m52_z80", &z, sizeof(z));
	if (!rc && load) {
		z_fix(&z);
		s_z = z;
	}
	return rc ? -1 : 0;
}

static int m52_save(void *ctx) { return m52_state(ctx, 0); }
static int m52_load(void *ctx) { return m52_state(ctx, 1); }
static int m52_supported(const char *machine) { return !strcmp(machine, "m52") || !strcmp(machine, "alpha1v"); }

const rbx_driver rbx_drv_m52 = { "Irem M52", m52_init, m52_exit, m52_reset, m52_frame, m52_save, m52_load, m52_supported,
                                 m52_render, NULL };
