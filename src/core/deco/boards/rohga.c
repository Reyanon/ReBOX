/*  Data East DE-0353-3 / DE-0375-x and kin: Rohga Armor Force, Wolf Fang, Wizard Fire / Dark Seal 2, Nitro Ball /
    Gun Ball, and Hot-B's Schmeiser Robo and Hangzo. Ported from MAME src/mame/dataeast/rohga.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail).
    68000 at 14 MHz (IRQ 6 at vblank until its acknowledge), the 104 protection / I/O chip (Nitro Ball: the 146), two
    tilemap chips (56 / 74 encrypted graphics), a buffered palette (taken by its DMA write), the sound board
    (machine/deco16snd.c: HuC6280, YM2151 banking two OKIs, stereo).
    Rohga and the Hot-B boards: sprite chip 52 (Rohga 6 bpp) with priorities, drawn as pens. Wizard Fire and Nitro Ball:
    two sprite chips drawn raw and mixed with the playfields per pixel in RGB (the second chip's and a playfield's
    alpha), in bands with the render helpers.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../video/gfxlayout.h"
#include "../machine/decocrpt.h"
#include "../machine/deco146.h"
#include "../machine/deco16snd.h"
#include "../video/deco16ic.h"
#include "../video/decoband.h"
#include "../video/decospr.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 14000000       // 28 MHz / 2
#define FRAME_MHZ 58000          // set_refresh_hz(58)
#define LINES 256                // set_size(40*8, 32*8)
#define VIS_Y0 8                 // set_visarea(0, 319, 8, 247)
#define VIS_H 240
#define VIS_W 320
#define VBL_LINES 8              // vblank 529 us
#define BANDS 4
#define NONE 0xffff              // (a playfield's pixel not drawn)

enum { V_ROHGA, V_WIZD, V_NITRO, V_SCHM, V_HANGZO };
static int s_var;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint16_t s_ram[0x10000];           // the work RAM (Rohga 0x3f0000; the others: their areas end to end)
static uint16_t s_spriteram[2][0x800], s_spriteram_buf[2][0x400];
static uint16_t s_palram[0x1000];         // 2048 entries of two words
static uint8_t s_pal_dirty[0x800];
static uint32_t s_pal[0x800];             // the palette its DMA took (0x00RRGGBB)
static uint16_t s_rowscroll[4][0x1000];
static uint16_t s_priority;
static deco16ic_t s_tile[2];
static decospr_t s_spr[2];
static deco146_t s_prot;
static rbx_gfx s_gfx8, s_gfx16a, s_gfx16b, s_gfxspr[2];
static int s_port_in, s_port_dsw, s_port_sys, s_port_dsw3;
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac;
static uint8_t s_pri_buf[VIS_W * VIS_H];
static uint16_t s_l[4][VIS_W * VIS_H];    // (two-chip boards) the playfields' pens; the sprites raw
static uint16_t s_sbm[2][VIS_W * VIS_H];
static int s_565;
static struct {
	deco16ic_t tile[2];
	uint16_t rowscroll[4][0x1000], spriteram[2][0x400], priority;
	uint32_t pal[0x800];
	uint32_t frame;
} *s_snap;

// ---- the 104's ports and latch ----
static uint16_t port_a(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_in); }
static uint16_t port_b(void *u)
{
	(void)u;
	const int vbl = s_line >= VIS_Y0 + VIS_H && s_line < VIS_Y0 + VIS_H + VBL_LINES;
	rbx_port_custom(s_port_sys, 0x0008, vbl ? 0x0008 : 0);
	return (uint16_t)rbx_port_read(s_port_sys);
}
static uint16_t port_c(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_dsw); }
static void latch_irq(void *u, int state) { (void)u; deco16snd_latch_irq(state); }
static uint8_t latch_r(void) { return deco146_soundlatch_r(&s_prot); }
static void ym_port(uint8_t data)   // sound_bankswitch_w
{
	deco16snd_oki(0)->bank = (data & 1) * 0x40000u;
	deco16snd_oki(1)->bank = ((data >> 1) & 1) * 0x40000u;
}

// ---- bus ----
static void palette_dma(void)
{
	for (int i = 0; i < 0x800; i++)
		if (s_pal_dirty[i]) {
			s_pal_dirty[i] = 0;
			const uint16_t w0 = s_palram[2 * i], w1 = s_palram[2 * i + 1];
			s_pal[i] = (uint32_t)(w1 & 0xff) << 16 | (w1 & 0xff00) | (w0 & 0xff);
		}
}
static void palette_w(uint32_t o, uint16_t v, uint16_t mask)   // buffered_palette_w (o: the word)
{
	uint16_t *p = &s_palram[o & 0xfff];
	*p = (uint16_t)((*p & ~mask) | (v & mask));
	s_pal_dirty[(o & 0xfff) >> 1] = 1;
}
static int is_two_chip(void) { return s_var == V_WIZD || s_var == V_NITRO; }
static uint32_t prot_base(void) { return s_var == V_WIZD ? 0xfe4000 : s_var == V_NITRO ? 0xff4000 : 0x280000; }

static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	const uint32_t pb = prot_base();
	if (a >= pb && a < pb + 0x4000) return deco146_read(&s_prot, (uint16_t)((a - pb) & 0x7fe), NULL);
	if (is_two_chip()) {
		if (a >= 0x380000 && a < 0x382000) return s_palram[(a - 0x380000) >> 1];
		if (a == 0x320000 && s_var == V_NITRO) return (uint16_t)rbx_port_read(s_port_dsw3);
		return 0;
	}
	if (a == 0x2c0000) return (uint16_t)rbx_port_read(s_port_dsw3);
	if (a == 0x321100 && s_var == V_ROHGA) { m68k_bus_set_line(6, 0); return 0; }   // irq_ack_r
	if (s_var != V_ROHGA) {   // (hotb_base_map)
		if (a == 0x300000) return (uint16_t)rbx_port_read(s_port_dsw3);
		if (a == 0x310002) return port_b(NULL);
		if (a >= 0x3e0000 && a < 0x3e4000) return s_palram[((a - 0x3e0000) >> 1) & 0xfff];
	} else if (a >= 0x3e0000 && a < 0x3e2000)
		return s_palram[(a - 0x3e0000) >> 1];
	return 0;
}
static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	const uint32_t pb = prot_base();
	if (a >= pb && a < pb + 0x4000) {
		deco146_write(&s_prot, (uint16_t)((a - pb) & 0x7fe), v, mask, NULL);
		return;
	}
	if (is_two_chip()) {
		if (a >= 0x380000 && a < 0x382000) palette_w((a - 0x380000) >> 1, v, mask);
		else if (a >= 0x300000 && a < 0x300010) deco16ic_control_w(&s_tile[0], (int)((a >> 1) & 7), v, mask);
		else if (a >= 0x310000 && a < 0x310010) deco16ic_control_w(&s_tile[1], (int)((a >> 1) & 7), v, mask);
		else if (a == 0x320000) s_priority = (uint16_t)((s_priority & ~mask) | (v & mask));
		else if (a == 0x320004) m68k_bus_set_line(6, 0);   // irq_ack_w
		else if (a == 0x350000) memcpy(s_spriteram_buf[0], s_spriteram[0], sizeof(s_spriteram_buf[0]));
		else if (a == 0x370000) memcpy(s_spriteram_buf[1], s_spriteram[1], sizeof(s_spriteram_buf[1]));
		else if (a == 0x390008) palette_dma();
		return;
	}
	if (a >= 0x3e0000 && a < (s_var == V_ROHGA ? 0x3e2000u : 0x3e4000u))   // (Hot-B: mirrored at 0x2000)
		palette_w((a - 0x3e0000) >> 1, v, mask);
	else if (a >= 0x200000 && a < 0x200010)
		deco16ic_control_w(&s_tile[0], (int)((a >> 1) & 7), v, mask);
	else if (a >= 0x240000 && a < 0x240010)
		deco16ic_control_w(&s_tile[1], (int)((a >> 1) & 7), v, mask);
	else if (a == 0x300000)   // the sprite DMA
		memcpy(s_spriteram_buf[0], s_spriteram[0], sizeof(s_spriteram_buf[0]));
	else if (a == 0x31000a)
		palette_dma();
	else if (a == 0x322000)
		s_priority = (uint16_t)((s_priority & ~mask) | (v & mask));
	else if (a == 0x321100)   // (Rohga's CLR reads first: Musashi does not; Hot-B: irq_ack_w)
		m68k_bus_set_line(6, 0);
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static int bank_cb(int bank) { return (bank & 0x30) << 8; }
static uint32_t pri_cb(uint16_t x, int extpri)
{
	(void)extpri;
	switch (x & 0x6000) {
	case 0x4000: return 0xf0;
	case 0x6000: return 0xf0 | 0xcc;
	default: return 0;
	}
}
static int col_cb(uint16_t x, int extpri) { (void)extpri; return (x >> 9) & 0xf; }
static int schmeisr_col_cb(uint16_t x, int extpri)   // (wire mods on the PCB)
{
	(void)extpri;
	return (((x >> 9) & 0xf) << 2) + ((x & 0x8000) ? 1 : 0);
}

// Rohga and the Hot-B boards
static void rg_draw(void)
{
	memcpy(rbx_video.palette, s_snap->pal, sizeof(s_snap->pal));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	deco16ic_t *t0 = &s_snap->tile[0], *t1 = &s_snap->tile[1];
	decospr_t spr = s_spr[0];
	spr.flip_screen = !((deco16ic_control_r(t0, 0) >> 7) & 1);   // (sprites are flipped relative to the tilemaps)
	memset(s_pri_buf, 0, sizeof(s_pri_buf));
	for (int y = 0; y < VIS_H; y++)
		for (int x = 0; x < VIS_W; x++)
			bm[y * pitch + x] = 768;
	const uint16_t *rs0 = s_snap->rowscroll[0], *rs1 = s_snap->rowscroll[1], *rs2 = s_snap->rowscroll[2],
	               *rs3 = s_snap->rowscroll[3];
	switch (s_snap->priority & 3) {
	case 0:
		if (s_snap->priority & 4)   // as one 8 bpp layer
			deco16ic_draw_combine(t1, rs2, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 1, 3, NULL);
		else {
			deco16ic_draw(t1, 1, rs3, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 1, 1, 0xff);
			deco16ic_draw(t1, 0, rs2, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 2, 0xff);
		}
		deco16ic_draw(t0, 1, rs1, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 4, 0xff);
		break;
	case 1:
		deco16ic_draw(t1, 1, rs3, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 1, 1, 0xff);
		deco16ic_draw(t0, 1, rs1, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 2, 0xff);
		deco16ic_draw(t1, 0, rs2, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 4, 0xff);
		break;
	case 2:
		deco16ic_draw(t0, 1, rs1, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 1, 0xff);
		deco16ic_draw(t1, 1, rs3, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 2, 0xff);
		deco16ic_draw(t1, 0, rs2, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 4, 0xff);
		break;
	}
	decospr_draw(&spr, s_snap->spriteram[0], 0x400, bm, s_pri_buf, pitch, VIS_W, VIS_H, VIS_Y0, 1, s_snap->frame);
	deco16ic_draw(t0, 0, rs0, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
}

static inline uint32_t blend(uint32_t d, uint32_t s, int level)   // MAME alpha_blend_r32
{
	return ((((s & 0xff00ff) * (uint32_t)level + (d & 0xff00ff) * (uint32_t)(256 - level)) >> 8) & 0xff00ff) |
	       ((((s & 0x00ff00) * (uint32_t)level + (d & 0x00ff00) * (uint32_t)(256 - level)) >> 8) & 0x00ff00);
}

// Nitro Ball's mixnitroballlayer: the first chip's pixel priority (from its bits 9 .. 11) against the playfields
static int nitro_pri1(uint16_t prio, uint16_t pix)
{
	static const uint16_t p0[8] = { 0x200, 0x020, 0x008, 0x002, 0x100, 0x040, 0x004, 0x001 };
	static const uint16_t p20[8] = { 0x080, 0x004, 0x002, 0x001, 0x100, 0x020, 0x008, 0x200 };
	return (prio == 0x20 ? p20 : p0)[(pix >> 9) & 7];
}

// Wizard Fire / Nitro Ball: a band of lines (the playfields' pens and the raw sprites, mixed per pixel in RGB)
static void wz_band(int band)
{
	const int r0 = VIS_H * band / BANDS, r1 = VIS_H * (band + 1) / BANDS, n = (r1 - r0) * VIS_W, o = r0 * VIS_W;
	deco16ic_t *t0 = &s_snap->tile[0], *t1 = &s_snap->tile[1];
	const uint32_t *pal = s_snap->pal;
	const int nitro = s_var == V_NITRO, flip = (deco16ic_control_r(t0, 0) >> 7) & 1;
	for (int c = 1; c >= 0; c--) {
		decospr_t spr = s_spr[c];
		spr.flip_screen = nitro ? flip : !flip;
		memset(s_sbm[c] + o, 0, (size_t)n * 2);
		decospr_draw(&spr, s_snap->spriteram[c], 0x400, s_sbm[c] + o, NULL, VIS_W, VIS_W, r1 - r0, VIS_Y0 + r0, 1,
		             s_snap->frame);
	}
	for (int k = 0; k < 4; k++)
		memset(s_l[k] + o, 0xff, (size_t)n * 2);   // (NONE)
	const uint16_t *rs = NULL;
	if (nitro) {   // pf3 and pf4 as one 8 bpp picture (opaque); pf2; pf1 on top
		deco16ic_draw_combine(t1, s_snap->rowscroll[2], s_l[3], NULL, VIS_W, VIS_W, VIS_Y0, r0, r1, 1, 0, NULL);
		deco16ic_draw(t0, 1, s_snap->rowscroll[1], s_l[1], NULL, VIS_W, VIS_W, VIS_Y0, r0, r1, 0, 0, 0xff);
		rs = s_snap->rowscroll[0];
	} else {
		deco16ic_draw(t1, 1, s_snap->rowscroll[3], s_l[3], NULL, VIS_W, VIS_W, VIS_Y0, r0, r1, 1, 0, 0xff);
		deco16ic_draw(t0, 1, NULL, s_l[1], NULL, VIS_W, VIS_W, VIS_Y0, r0, r1, 0, 0, 0xff);
		deco16ic_draw(t1, 0, s_snap->rowscroll[2], s_l[2], NULL, VIS_W, VIS_W, VIS_Y0, r0, r1, 0, 0, 0xff);
	}
	deco16ic_draw(t0, 0, rs, s_l[0], NULL, VIS_W, VIS_W, VIS_Y0, r0, r1, 0, 0, 0xff);
	const uint16_t prio = s_snap->priority;
	const int alpha2 = (prio & 0x1f) == 0x1f;   // (Wizard Fire's pf3 at alpha 0x80)
	for (int y = r0; y < r1; y++) {
		uint32_t d[VIS_W];
		for (int x = 0; x < VIS_W; x++) {
			const int i = y * VIS_W + x;
			const uint16_t p1 = s_sbm[0][i], p2 = s_sbm[1][i];
			uint32_t c = s_l[3][i] != NONE ? pal[s_l[3][i] & 0x7ff] : pal[512];
			if (nitro) {
				int bgpri = 0;
				if (s_l[1][i] != NONE) {
					c = pal[s_l[1][i] & 0x7ff];
					bgpri = prio == 0x20 ? 0x040 : 0x008;
				}
				const int pri1 = nitro_pri1(prio, p1), pri2 = prio == 0x20 ? 0x010 : 0x080;
				int drawn1 = 0;
				if ((p1 & 0xf) && pri1 > bgpri) {
					c = pal[((p1 & 0x1ff) + 0x400) & 0x7ff];
					drawn1 = 1;
				}
				if ((p2 & 0xf) && pri2 > bgpri && (!drawn1 || pri2 > pri1)) {
					const uint32_t sc = pal[((p2 & 0xff) + 0x600) & 0x7ff];
					c = (p2 & 0x100) ? blend(c, sc, 0x80) : sc;
				}
			} else {
				// the first chip by its priority bits 9-10: 0x600 under pf2, 0x400 under pf3, the rest over it
				const int s1 = (p1 & 0xf) ? ((p1 & 0x600) == 0x600 ? 0 : (p1 & 0x600) == 0x400 ? 1 : 2) : -1;
				const uint32_t c1 = pal[((p1 & 0x1ff) + 0x400) & 0x7ff];
				if (s1 == 0) c = c1;
				if (s_l[1][i] != NONE) c = pal[s_l[1][i] & 0x7ff];
				if (s1 == 1) c = c1;
				if (s_l[2][i] != NONE) c = alpha2 ? blend(c, pal[s_l[2][i] & 0x7ff], 0x80) : pal[s_l[2][i] & 0x7ff];
				if (s1 == 2) c = c1;
				if (p2 & 0xf) {   // mixwizdfirelayer
					const uint32_t sc = pal[((p2 & 0xff) + 0x600) & 0x7ff];
					c = (p2 & 0x100) ? blend(c, sc, 0x80) : sc;
				}
			}
			if (s_l[0][i] != NONE)
				c = pal[s_l[0][i] & 0x7ff];
			d[x] = c;
		}
		uint8_t *op = (uint8_t *)rbx_video.out + (size_t)y * rbx_video.out_pitch;
		if (s_565)
			for (int x = 0; x < VIS_W; x++)
				((uint16_t *)op)[x] = (uint16_t)((d[x] >> 8 & 0xf800) | (d[x] >> 5 & 0x07e0) | (d[x] >> 3 & 0x001f));
		else if (rbx_video.out_bpp == 4)
			for (int x = 0; x < VIS_W; x++)
				((uint32_t *)op)[x] = rbx_video.out_color((int)(d[x] >> 16) & 0xff, (int)(d[x] >> 8) & 0xff, (int)d[x] & 0xff);
		else
			for (int x = 0; x < VIS_W; x++)
				((uint16_t *)op)[x] = (uint16_t)rbx_video.out_color((int)(d[x] >> 16) & 0xff, (int)(d[x] >> 8) & 0xff, (int)d[x] & 0xff);
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, wz_band };
static int rg_help(int helper) { return decoband_help(&s_bands, helper); }

static void rg_render(void)
{
	deco16ic_update(&s_snap->tile[0]);
	deco16ic_update(&s_snap->tile[1]);
	if (!is_two_chip()) {
		rg_draw();
		return;
	}
	if (!rbx_video.out)
		return;
	s_565 = rbx_video.out_bpp == 2 && rbx_video.out_color(0xff, 0, 0) == 0xf800 && rbx_video.out_color(0, 0xff, 0) == 0x07e0 &&
	        rbx_video.out_color(0, 0, 0xff) == 0x001f && rbx_video.out_color(0x84, 0x82, 0x21) == 0x8404;
	decoband_run(&s_bands);
	rbx_video.out_done = 1;
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->tile, s_tile, sizeof(s_tile));
	memcpy(s_snap->rowscroll, s_rowscroll, sizeof(s_rowscroll));
	memcpy(s_snap->spriteram, s_spriteram_buf, sizeof(s_spriteram_buf));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	s_snap->priority = s_priority;
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void rg_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_spriteram_buf, 0, sizeof(s_spriteram_buf));
	memset(s_palram, 0, sizeof(s_palram));
	memset(s_pal_dirty, 0, sizeof(s_pal_dirty));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_rowscroll, 0, sizeof(s_rowscroll));
	s_priority = 0;
	for (int i = 0; i < 2; i++) {
		memset(s_tile[i].vram, 0, sizeof(s_tile[i].vram));
		deco16ic_reset(&s_tile[i]);
	}
	deco146_reset(&s_prot);
	deco16snd_reset();
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = 0;
}

static void rg_exit(void);

static int rg_init(const rbx_game *g)
{
	s_var = !strcmp(g->machine, "wizdfire") ? V_WIZD : !strcmp(g->machine, "nitrobal") ? V_NITRO
	      : !strcmp(g->machine, "schmeisr") ? V_SCHM : !strcmp(g->machine, "hangzo") ? V_HANGZO : V_ROHGA;
	uint32_t snd_size, csize, t1s, t2s, ssize, s2s = 0, o1s, o2s;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *snd = rbx_region("audiocpu", &snd_size), *chars = rbx_region("chars", &csize),
	        *t1 = rbx_region("tiles1", &t1s), *t2 = rbx_region("tiles2", &t2s), *spr = rbx_region("sprites1", &ssize),
	        *spr2 = is_two_chip() ? rbx_region("sprites2", &s2s) : NULL,
	        *oki1 = rbx_region("oki1", &o1s), *oki2 = rbx_region("oki2", &o2s);
	if (!s_rom || !snd || !chars || !t1 || !t2 || !spr || !oki1 || !oki2 || (is_two_chip() && !spr2)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (s_var == V_SCHM || s_var == V_HANGZO) {   // init_hangzo: the chars from tiles1's first quarters
		if (csize >= 0x40000 && t1s >= 0xa0000) {
			memcpy(chars, t1, 0x20000);
			memcpy(chars + 0x20000, t1 + 0x80000, 0x20000);
		}
	}
	int rc = 0;
	switch (s_var) {
	case V_ROHGA: rc = deco56_decrypt_gfx(chars, csize) || deco56_decrypt_gfx(t1, t1s); break;
	case V_WIZD: rc = deco74_decrypt_gfx(chars, csize) || deco74_decrypt_gfx(t1, t1s) || deco74_decrypt_gfx(t2, t2s); break;
	case V_NITRO: rc = deco56_decrypt_gfx(chars, csize) || deco56_decrypt_gfx(t1, t1s) || deco74_decrypt_gfx(t2, t2s); break;
	case V_SCHM: rc = deco74_decrypt_gfx(chars, csize) || deco74_decrypt_gfx(t1, t1s); break;
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	{	// charlayout, tilelayout (RGN_FRAC(1,2)), tilelayout_6bpp (RGN_FRAC(1,3))
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 16);
			x16[i] = (uint32_t)(256 + i);
			x16[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 16);
		const uint32_t ch = csize * 4, h1 = t1s * 4, h2 = t2s * 4, st = ssize / 3 * 8, hs = ssize * 4, hs2 = s2s * 4;
		const uint32_t pc[4] = { ch + 8, ch, 8, 0 }, p1[4] = { h1 + 8, h1, 8, 0 }, p2[4] = { h2 + 8, h2, 8, 0 };
		const uint32_t ps6[6] = { 2 * st + 8, 2 * st, st + 8, st, 8, 0 };
		const uint32_t ps[4] = { hs + 8, hs, 8, 0 }, ps2[4] = { hs2 + 8, hs2, 8, 0 };
		const rbx_gfx_layout l8 = { 8, 8, 4, pc, x8, y8, 16 * 8 };
		const rbx_gfx_layout la = { 16, 16, 4, p1, x16, y16, 32 * 16 }, lb = { 16, 16, 4, p2, x16, y16, 32 * 16 };
		const rbx_gfx_layout ls6 = { 16, 16, 6, ps6, x16, y16, 32 * 16 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, x16, y16, 32 * 16 }, ls2 = { 16, 16, 4, ps2, x16, y16, 32 * 16 };
		if (rbx_gfx_decode(&s_gfx8, &l8, chars, csize / 2) || rbx_gfx_decode(&s_gfx16a, &la, t1, t1s / 2) ||
		    rbx_gfx_decode(&s_gfx16b, &lb, t2, t2s / 2) ||
		    (s_var == V_ROHGA ? rbx_gfx_decode(&s_gfxspr[0], &ls6, spr, ssize / 3)
		                      : rbx_gfx_decode(&s_gfxspr[0], &ls, spr, ssize / 2)) ||
		    (spr2 && rbx_gfx_decode(&s_gfxspr[1], &ls2, spr2, s2s / 2))) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			rg_exit();
			return -1;
		}
	}
	memset(s_tile, 0, sizeof(s_tile));
	for (int c = 0; c < 2; c++) {
		for (int i = 0; i < 2; i++) {
			s_tile[c].tm[i].size = DECO_64x32;
			s_tile[c].tm[i].col_bank = i ? 0x10 : 0x00;
			s_tile[c].tm[i].col_mask = 0x0f;
			s_tile[c].tm[i].bank_cb = bank_cb;
		}
		s_tile[c].gfx8 = &s_gfx8;
	}
	if (s_var != V_WIZD && s_var != V_NITRO)
		s_tile[0].tm[0].size = DECO_64x64;
	if (s_var == V_NITRO) {
		s_tile[0].tm[1].size = DECO_32x32;
		for (int i = 0; i < 2; i++) {
			s_tile[1].tm[i].size = DECO_32x32;
			s_tile[1].tm[i].col_bank = 0;
			s_tile[1].tm[i].col_mask = 0;
		}
	}
	s_tile[0].gfx16 = &s_gfx16a;
	s_tile[1].gfx16 = &s_gfx16b;
	s_tile[1].base16 = 512;   // (gfx_rohga: tiles2 from colour 512)
	memset(s_spr, 0, sizeof(s_spr));
	s_spr[0].gfx = &s_gfxspr[0];
	if (is_two_chip()) {   // (alloc_sprite_bitmap: raw shift 4; Nitro Ball the alternate list format)
		s_spr[1].gfx = &s_gfxspr[1];
		for (int c = 0; c < 2; c++) {
			s_spr[c].raw = 1;
			s_spr[c].raw_shift = 4;
			s_spr[c].alt_format = s_var == V_NITRO;
		}
	} else {
		s_spr[0].color_base = 1024;
		s_spr[0].gran = s_var == V_ROHGA ? 64 : 16;
		s_spr[0].pri_cb = pri_cb;
		s_spr[0].col_cb = s_var == V_ROHGA ? col_cb : schmeisr_col_cb;
	}
	deco146_init(&s_prot, s_var != V_NITRO);
	if (is_two_chip()) {
		static const uint8_t reverse[10] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };   // set_interface_scramble_reverse
		deco146_set_scramble(&s_prot, reverse);
		s_prot.magic_read_xor_on = s_var == V_NITRO;
	}
	s_prot.port_a = port_a;
	s_prot.port_b = port_b;
	s_prot.port_c = port_c;
	s_prot.soundlatch_irq = latch_irq;
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_snap) {
		rbx_fail("%s: out of memory", g->set);
		rg_exit();
		return -1;
	}
	// sound: HuC6280 2.685 MHz, YM2151 3.58 MHz 0.36, OKI 1 MHz 0.46, OKI 2 MHz 0.18 (both banked), stereo
	deco16snd_cfg sc;
	memset(&sc, 0, sizeof(sc));
	sc.cpu_clock = 32220000 / 4 / 3;
	sc.ym_clock = 32220000 / 9;
	sc.ym_gain[0] = sc.ym_gain[1] = 92;
	sc.oki_rom[0] = oki1;
	sc.oki_size[0] = o1s;
	sc.oki_clock[0] = 32220000 / 32;
	sc.oki_gain[0] = 118;
	sc.oki_rom[1] = oki2;
	sc.oki_size[1] = o2s;
	sc.oki_clock[1] = 32220000 / 16;
	sc.oki_gain[1] = 46;
	sc.stereo = 1;
	sc.ym_port = ym_port;
	sc.latch_r = latch_r;
	if (deco16snd_init(snd, snd_size, &sc, FRAME_MHZ, LINES) || rbx_video_setup(VIS_W, VIS_H, 0x800)) {
		rbx_fail("%s: out of memory", g->set);
		rg_exit();
		return -1;
	}
	s_port_in = rbx_port_find("INPUTS");
	s_port_dsw = rbx_port_find("DSW");
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_dsw3 = rbx_port_find("DSW3");
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, s_var == V_SCHM || s_var == V_HANGZO ? 0xfffff : 0x1fffff, s_rom, s_rom_size, 1, 0);
	if (is_two_chip()) {
		const uint32_t mir = s_var == V_NITRO;   // (Nitro Ball: the playfields' RAM mirrored)
		m68k_bus_map(0x200000, 0x200fff, s_tile[0].vram[0], 0x1000, 1, 1);
		m68k_bus_map(0x202000, mir ? 0x2027ff : 0x202fff, s_tile[0].vram[1], mir ? 0x800 : 0x1000, 1, 1);
		m68k_bus_map(0x208000, mir ? 0x2087ff : 0x208fff, s_tile[1].vram[0], mir ? 0x800 : 0x1000, 1, 1);
		m68k_bus_map(0x20a000, mir ? 0x20a7ff : 0x20afff, s_tile[1].vram[1], mir ? 0x800 : 0x1000, 1, 1);
		if (mir) {
			m68k_bus_map(0x201000, 0x201fff, s_tile[0].vram[0], 0x1000, 1, 1);
			m68k_bus_map(0x202800, 0x202fff, s_tile[0].vram[1], 0x800, 1, 1);
			m68k_bus_map(0x208800, 0x208fff, s_tile[1].vram[0], 0x800, 1, 1);
			m68k_bus_map(0x20a800, 0x20afff, s_tile[1].vram[1], 0x800, 1, 1);
			m68k_bus_map(0x204000, 0x2047ff, s_rowscroll[0], 0x800, 1, 1);
			m68k_bus_map(0x206000, 0x2067ff, s_rowscroll[1], 0x800, 1, 1);
		}
		m68k_bus_map(0x20c000, 0x20c7ff, s_rowscroll[2], 0x800, 1, 1);
		m68k_bus_map(0x20e000, 0x20e7ff, s_rowscroll[3], 0x800, 1, 1);
		m68k_bus_map(0x340000, 0x3407ff, s_spriteram[0], 0x800, 1, 1);
		m68k_bus_map(0x360000, 0x3607ff, s_spriteram[1], 0x800, 1, 1);
		if (s_var == V_WIZD) {
			m68k_bus_map(0xfdc000, 0xfe3fff, s_ram, 0x8000, 1, 1);
			m68k_bus_map(0xfe8000, 0xffffff, s_ram + 0x4000, 0x18000, 1, 1);
		} else {
			m68k_bus_map(0xfec000, 0xff3fff, s_ram, 0x8000, 1, 1);
			m68k_bus_map(0xff8000, 0xffffff, s_ram + 0x4000, 0x8000, 1, 1);
		}
	} else {
		m68k_bus_map(0x3c0000, 0x3c1fff, s_tile[0].vram[0], sizeof(s_tile[0].vram[0]), 1, 1);
		m68k_bus_map(0x3c2000, 0x3c2fff, s_tile[0].vram[1], 0x1000, 1, 1);
		m68k_bus_map(0x3c4000, 0x3c4fff, s_tile[1].vram[0], 0x1000, 1, 1);
		m68k_bus_map(0x3c6000, 0x3c6fff, s_tile[1].vram[1], 0x1000, 1, 1);
		m68k_bus_map(0x3c8000, 0x3c9fff, s_rowscroll[0], 0x2000, 1, 1);
		m68k_bus_map(0x3ca000, 0x3cbfff, s_rowscroll[1], 0x1000, 1, 1);   // (mirror 0x1000)
		m68k_bus_map(0x3cc000, 0x3cdfff, s_rowscroll[2], 0x1000, 1, 1);
		m68k_bus_map(0x3ce000, 0x3cffff, s_rowscroll[3], 0x1000, 1, 1);
		m68k_bus_map(0x3d0000, 0x3d0fff, s_spriteram[0], sizeof(s_spriteram[0]), 1, 1);
		if (s_var == V_SCHM)
			m68k_bus_map(0xff0000, 0xff7fff, s_ram, 0x8000, 1, 1);
		else
			m68k_bus_map(0x3f0000, 0x3f3fff, s_ram, 0x4000, 1, 1);
	}
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	rg_reset();
	return 0;
}

static void rg_exit(void)
{
	deco16snd_exit();
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16a);
	rbx_gfx_free(&s_gfx16b);
	rbx_gfx_free(&s_gfxspr[0]);
	rbx_gfx_free(&s_gfxspr[1]);
	free(s_snap);
	s_snap = NULL;
}

static void rg_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // irq6_line_assert at vblank (until its acknowledge)
			vblank_capture();
			m68k_bus_set_line(6, 1);
		}
		if (line >= VIS_Y0 && line < VIS_Y0 + VIS_H)
			for (int i = 0; i < 2; i++)
				deco16ic_latch_line(&s_tile[i], line);
		m68k_execute(rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		deco16snd_line();
	}
	deco16snd_frame_end();
	s_frame++;
}

static int rg_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr = s_cpu_frac;
	uint32_t misc[2] = { s_frame, s_priority };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, s_var == V_ROHGA ? 0x4000 : sizeof(s_ram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram[0], sizeof(s_spriteram[0]));
	rc |= rbx_state_io(ctx, "sprb", s_spriteram_buf[0], sizeof(s_spriteram_buf[0]));
	if (is_two_chip()) {
		rc |= rbx_state_io(ctx, "spr2", s_spriteram[1], sizeof(s_spriteram[1]));
		rc |= rbx_state_io(ctx, "sprb2", s_spriteram_buf[1], sizeof(s_spriteram_buf[1]));
	}
	rc |= rbx_state_io(ctx, "palram", s_palram, sizeof(s_palram));
	rc |= rbx_state_io(ctx, "paldirty", s_pal_dirty, sizeof(s_pal_dirty));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "rows", s_rowscroll, sizeof(s_rowscroll));
	for (int i = 0; i < 2; i++) {
		rc |= rbx_state_io(ctx, i ? "vram2" : "vram1", s_tile[i].vram, sizeof(s_tile[i].vram));
		rc |= rbx_state_io(ctx, i ? "tctl2" : "tctl1", s_tile[i].control, sizeof(s_tile[i].control));
	}
	rc |= rbx_state_io(ctx, "prot", &s_prot, DECO146_STATE_SIZE);
	rc |= deco16snd_state(ctx, load);
	rc |= rbx_state_io(ctx, "frac", &fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		s_cpu_frac = fr;
		s_frame = misc[0];
		s_priority = (uint16_t)misc[1];
		for (int c = 0; c < 2; c++)
			for (int i = 0; i < 8; i++)
				deco16ic_control_w(&s_tile[c], i, s_tile[c].control[i], 0xffff);
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int rg_save(void *ctx) { return rg_state(ctx, 0); }
static int rg_load(void *ctx) { return rg_state(ctx, 1); }
static int rg_supported(const char *machine)
{
	return !strcmp(machine, "rohga") || !strcmp(machine, "wizdfire") || !strcmp(machine, "nitrobal") ||
	       !strcmp(machine, "schmeisr") || !strcmp(machine, "hangzo");
}

const rbx_driver rbx_drv_rohga = { "DECO 16 Rohga", rg_init, rg_exit, rg_reset, rg_frame, rg_save, rg_load,
                                   rg_supported, rg_render, rg_help };
