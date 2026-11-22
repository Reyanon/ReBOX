/*  Data East DE-0379: Boogie Wings / The Great Ragtime Show. Ported from MAME src/mame/dataeast/boogwing.cpp
    (BSD-3-Clause, copyright-holders: Bryan McPhail, David Haywood).
    The 102 encrypted 68000 at 14 MHz (IRQ 6 at vblank, HOLD_LINE), the 104 protection / I/O chip, two tilemap chips
    (the first one's 16x16 tiles 5 bpp, all graphics encrypted), two sprite chips 52 drawn raw and mixed with the
    playfields by MAME's mix (priorities, the second chip's alpha from the DECO ACE), the DECO ACE (video/deco_ace.h:
    faded and raw palettes), the sound board (machine/deco16snd.c: HuC6280, YM2151 banking both OKIs, stereo). The
    output is RGB.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../video/gfxlayout.h"
#include "../machine/decocrpt.h"
#include "../machine/deco102.h"
#include "../machine/deco146.h"
#include "../machine/deco16snd.h"
#include "../video/deco16ic.h"
#include "../video/decospr.h"
#include "../../video/decoband.h"
#include "../video/deco_ace.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 14000000       // 28 MHz / 2
#define FRAME_MHZ 57800          // set_raw(28 MHz / 4, 442, 0, 320, 274, 8, 248)
#define LINES 274
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 320
#define BANDS 4

static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint16_t *s_opcodes;
static uint16_t (*s_ram_hp)[0x8000];            // 0x200000 (64 KB)
#define s_ram (*s_ram_hp)
static uint16_t s_spriteram[2][0x800], s_spriteram_buf[2][0x400];
static uint16_t (*s_rowscroll_hp)[4][0x800];
#define s_rowscroll (*s_rowscroll_hp)
static uint16_t s_priority;
static deco_ace_t s_ace;
static deco16ic_t (*s_tile_hp)[2];
#define s_tile (*s_tile_hp)
static decospr_t s_spr[2];
static deco146_t s_prot;
static rbx_gfx s_gfx8, s_gfx16a, s_gfx16b, s_gfxspr1, s_gfxspr2;
static int s_port_in, s_port_dsw, s_port_sys;
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac;
static int s_565;
static uint16_t s_tmap[VIS_W * VIS_H], s_alpha[VIS_W * VIS_H], s_top[VIS_W * VIS_H], s_sbm[2][VIS_W * VIS_H];
static uint8_t (*s_pri_hp)[VIS_W * VIS_H];
#define s_pri (*s_pri_hp)
static struct {
	deco16ic_t tile[2];
	uint16_t rowscroll[4][0x800], spriteram[2][0x400], priority, ace[0x28];
	uint32_t pens[0x1000];
	uint32_t frame;
} *s_snap;

// ---- the 104's ports and latch ----
static uint16_t port_a(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_in); }
static uint16_t port_b(void *u)
{
	(void)u;
	const int vbl = s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H;
	rbx_port_custom(s_port_sys, 0x0008, vbl ? 0x0008 : 0);
	return (uint16_t)rbx_port_read(s_port_sys);
}
static uint16_t port_c(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_dsw); }
static void latch_irq(void *u, int state) { (void)u; deco16snd_latch_irq(state); }
static uint8_t latch_r(void) { return deco146_soundlatch_r(&s_prot); }
static void ym_port(uint8_t data)   // sound_bankswitch_w
{
	deco16snd_oki(1)->bank = ((data & 2) >> 1) * 0x40000u;
	deco16snd_oki(0)->bank = (data & 1) * 0x40000u;
}

// ---- bus ----
static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x24e000 && a < 0x24f000) return deco146_read(&s_prot, (uint16_t)((a - 0x24e000) & 0x7fe), NULL);
	if (a >= 0x284000 && a < 0x286000) return deco_ace_pal16_r(&s_ace, (int)((a - 0x284000) >> 1));
	if (a >= 0x3c0000 && a < 0x3c0050) return s_ace.ram[(a - 0x3c0000) >> 1];
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
	if (a >= 0x260000 && a < 0x260010)
		deco16ic_control_w(&s_tile[0], (int)((a >> 1) & 7), v, mask);
	else if (a >= 0x270000 && a < 0x270010)
		deco16ic_control_w(&s_tile[1], (int)((a >> 1) & 7), v, mask);
	else if (a >= 0x24e000 && a < 0x24f000)
		deco146_write(&s_prot, (uint16_t)((a - 0x24e000) & 0x7fe), v, mask, NULL);
	else if (a >= 0x284000 && a < 0x286000)
		deco_ace_pal16_w(&s_ace, (int)((a - 0x284000) >> 1), v, mask);
	else if (a >= 0x3c0000 && a < 0x3c0050)
		deco_ace_w(&s_ace, (int)((a - 0x3c0000) >> 1), v, mask);
	else if (a == 0x282008)
		deco_ace_dma(&s_ace);
	else if (a == 0x240000)   // the sprite DMAs
		memcpy(s_spriteram_buf[0], s_spriteram[0], sizeof(s_spriteram_buf[0]));
	else if (a == 0x244000)
		memcpy(s_spriteram_buf[1], s_spriteram[1], sizeof(s_spriteram_buf[1]));
	else if (a == 0x220000)
		s_priority = (uint16_t)((s_priority & ~mask) | (v & mask));
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static int bank_cb(int bank) { return (bank & 0x70) << 8; }
static int bank_cb2(int bank)
{
	int offset = (bank & 0x70) << 8;
	if ((bank & 0xf) == 0xa)
		offset += 0x800;   // (strange: the transporter level)
	return offset;
}

static inline uint32_t blend(uint32_t d, uint32_t s, int level)   // MAME alpha_blend_r32
{
	return ((((s & 0xff00ff) * (uint32_t)level + (d & 0xff00ff) * (uint32_t)(256 - level)) >> 8) & 0xff00ff) |
	       ((((s & 0x00ff00) * (uint32_t)level + (d & 0x00ff00) * (uint32_t)(256 - level)) >> 8) & 0x00ff00);
}
static inline uint32_t sub_blend(uint32_t d, uint32_t s, int level)   // MAME sub_blend_r32
{
	int r = (int)((d >> 16) & 0xff) - (int)(((s >> 16) & 0xff) * (uint32_t)level >> 8);
	int g = (int)((d >> 8) & 0xff) - (int)(((s >> 8) & 0xff) * (uint32_t)level >> 8);
	int b = (int)(d & 0xff) - (int)((s & 0xff) * (uint32_t)level >> 8);
	if (r < 0) r = 0;
	if (g < 0) g = 0;
	if (b < 0) b = 0;
	return (uint32_t)(r << 16 | g << 8 | b);
}

// MAME mix (one line)
static void mix_line(int y, uint32_t *d)
{
	const uint32_t *pal = s_snap->pens;
	const uint16_t *ace = s_snap->ace;
	const int priority = s_snap->priority & 7;
	const uint16_t coloffs = (s_snap->priority & 8) ? 0x800 : 0;
	const uint16_t *src1 = s_sbm[0] + y * VIS_W, *src2 = s_sbm[1] + y * VIS_W, *src3 = s_alpha + y * VIS_W,
	               *tm = s_tmap + y * VIS_W;
	const uint8_t *bp = s_pri + y * VIS_W;
	uint8_t al[32];   // (the ACE's alpha levels)
	for (int i = 0; i < 32; i++)
		al[i] = deco_ace_alpha(ace, (uint8_t)i);
	const int alpha3 = al[0x1f];
	for (int x = 0; x < VIS_W; x++) {
		const uint16_t pix1 = src1[x], pix2 = src2[x], pix3 = src3[x], tmappix = tm[x];
		if (!((pix1 | pix2) & 0xf) && !((pix3 & 0xf) && priority == 3)) {   // (no sprite: the playfields as drawn)
			if (tmappix != 0xffff)
				d[x] = pal[tmappix & 0xfff];
			continue;
		}
		int pri1, pri2, pri3 = 0, spri1, spri2;
		int alpha2 = al[(pix2 >> 4) & 0xf];
		if (priority == 0)
			spri1 = (pix1 & 0x600) == 0x600 ? 2 : (pix1 & 0x600) == 0x400 ? 8 : 32;
		else
			spri1 = (pix1 & 0x400) ? 8 : 32;
		if (priority == 1)
			pri1 = (pix1 & 0x600) ? 16 : 64;
		else if (priority == 0)
			pri1 = (pix1 & 0x400) == 0x400 ? 16 : 64;
		else
			pri1 = (pix1 & 0x600) == 0x600 ? 4 : (pix1 & 0x600) == 0x400 ? 16 : 64;
		spri2 = (pix2 & 0x600) == 0x600 ? 4 : (pix2 & 0x600) ? 16 : 64;
		if (pix2 & 0x100) {
			if (pix2 & 0x800)   // (the LUT: explosions)
				alpha2 = (pix2 & 8) ? 0xff : al[0x14 + ((pix2 - 1) & 0x7)];
			else
				alpha2 = al[0x10 + ((pix2 & 0x80) >> 7)];
		} else if (pix2 & 0x800)
			alpha2 = al[0x12 + ((pix2 & 0x80) >> 7)];
		if (priority == 2)
			pri2 = (pix2 & 0x600) == 0x600 ? 4 : (pix2 & 0x600) == 0x400 ? 16 : 64;
		else {
			if (priority == 3)
				pri3 = 32;
			pri2 = (pix2 & 0x400) == 0x400 ? 16 : 64;
		}
		const int bgpri = bp[x];
		int drawn = 0;
		if ((pix1 & 0xf) && pri1 > bgpri) {
			d[x] = pal[coloffs | ((pix1 & 0x1ff) + 0x500)];
			drawn |= 1;
		}
		if (pix2 & 0xf) {
			if (!drawn && tmappix != 0xffff)
				d[x] = pal[coloffs | tmappix];
			if (pri2 > bgpri && (!drawn || spri2 > spri1)) {
				const uint32_t c = pal[coloffs | ((pix2 & 0xff) + 0x700)];
				d[x] = alpha2 >= 0xff ? c : blend(d[x], c, alpha2);
				drawn |= 2;
			}
		}
		if (!drawn && tmappix != 0xffff)
			d[x] = pal[tmappix & 0xfff];
		if ((pix3 & 0xf) && priority == 3) {   // the alpha playfield (its shadowing: as MAME, approximate)
			const int bg2 = bgpri == 8 && !drawn;
			const int s1 = (drawn & 1) && pri1 <= pri3, s2 = (drawn & 2) && pri2 <= pri3;
			if (bg2 || (s1 && !(drawn & 2)) || (s2 && !(drawn & 1)) || (s1 && s2))
				if ((pix2 & 0x900) != 0x900 || (spri2 <= spri1 && s1)) {
					const uint32_t c = pal[(((drawn & 3) ? coloffs : 0) | pix3) & 0xfff];
					d[x] = ace[0x1f] == 0x22 ? sub_blend(d[x], c, alpha3) : blend(d[x], c, alpha3);
				}
		}
	}
}

static void bw_band(int band)
{
	const int r0 = VIS_H * band / BANDS, r1 = VIS_H * (band + 1) / BANDS, n = (r1 - r0) * VIS_W, o = r0 * VIS_W;
	deco16ic_t *t0 = &s_snap->tile[0], *t1 = &s_snap->tile[1];
	memset(s_sbm[0] + o, 0, (size_t)n * 2);
	memset(s_sbm[1] + o, 0, (size_t)n * 2);
	memset(s_alpha + o, 0, (size_t)n * 2);
	memset(s_pri + o, 0, (size_t)n);
	for (int i = 0; i < n; i++)
		s_tmap[o + i] = s_top[o + i] = 0xffff;
	const int flip = !((deco16ic_control_r(t0, 0) >> 7) & 1);   // (sprites are flipped relative to the tilemaps)
	for (int c = 1; c >= 0; c--) {
		decospr_t spr = s_spr[c];
		spr.flip_screen = flip;
		decospr_draw(&spr, s_snap->spriteram[c], 0x400, s_sbm[c] + o, NULL, VIS_W, VIS_W, r1 - r0, VIS_Y0 + r0, 1,
		             s_snap->frame);
	}
	const uint16_t *rs0 = s_snap->rowscroll[0], *rs1 = s_snap->rowscroll[1], *rs2 = s_snap->rowscroll[2],
	               *rs3 = s_snap->rowscroll[3];
	uint16_t *tb = s_tmap;
	uint8_t *pb = s_pri;
	switch (s_snap->priority & 7) {
	case 5:
		deco16ic_draw(t0, 1, rs1, tb, pb, VIS_W, VIS_W, VIS_Y0, r0, r1, 1, 0, 0xff);
		deco16ic_draw_combine(t1, rs2, tb, pb, VIS_W, VIS_W, VIS_Y0, r0, r1, 0, 32, NULL);
		break;
	case 4:
		deco16ic_draw_combine(t1, rs2, tb, pb, VIS_W, VIS_W, VIS_Y0, r0, r1, 1, 0, NULL);
		deco16ic_draw(t0, 1, rs1, tb, pb, VIS_W, VIS_W, VIS_Y0, r0, r1, 0, 32, 0xff);
		break;
	case 1: case 2:
		deco16ic_draw(t1, 1, rs3, tb, pb, VIS_W, VIS_W, VIS_Y0, r0, r1, 1, 0, 0xff);
		deco16ic_draw(t0, 1, rs1, tb, pb, VIS_W, VIS_W, VIS_Y0, r0, r1, 0, 8, 0xff);
		deco16ic_draw(t1, 0, rs2, tb, pb, VIS_W, VIS_W, VIS_Y0, r0, r1, 0, 32, 0xff);
		break;
	case 3:   // (the third playfield shadows: drawn apart)
		deco16ic_draw(t1, 1, rs3, tb, pb, VIS_W, VIS_W, VIS_Y0, r0, r1, 1, 0, 0xff);
		deco16ic_draw(t0, 1, rs1, tb, pb, VIS_W, VIS_W, VIS_Y0, r0, r1, 0, 8, 0xff);
		deco16ic_draw(t1, 0, rs2, s_alpha, pb, VIS_W, VIS_W, VIS_Y0, r0, r1, 0, 0, 0xff);
		break;
	default:
		deco16ic_draw(t1, 1, rs3, tb, pb, VIS_W, VIS_W, VIS_Y0, r0, r1, 1, 0, 0xff);
		deco16ic_draw(t1, 0, rs2, tb, pb, VIS_W, VIS_W, VIS_Y0, r0, r1, 0, 8, 0xff);
		deco16ic_draw(t0, 1, rs1, tb, pb, VIS_W, VIS_W, VIS_Y0, r0, r1, 0, 32, 0xff);
		break;
	}
	deco16ic_draw(t0, 0, rs0, s_top, NULL, VIS_W, VIS_W, VIS_Y0, r0, r1, 0, 0, 0xff);
	const uint32_t *pens = s_snap->pens;
	const uint32_t bg = pens[0x400];   // (pen not confirmed in MAME)
	for (int y = r0; y < r1; y++) {
		uint32_t d[VIS_W];
		for (int x = 0; x < VIS_W; x++)
			d[x] = bg;
		mix_line(y, d);
		const uint16_t *top = s_top + y * VIS_W;
		for (int x = 0; x < VIS_W; x++)
			if (top[x] != 0xffff)
				d[x] = pens[top[x] & 0xfff];
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
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, bw_band };
static int bw_help(int helper) { return decoband_help(&s_bands, helper); }

static void bw_render(void)
{
	if (!rbx_video.out)
		return;
	s_565 = rbx_video.out_bpp == 2 && rbx_video.out_color(0xff, 0, 0) == 0xf800 && rbx_video.out_color(0, 0xff, 0) == 0x07e0 &&
	        rbx_video.out_color(0, 0, 0xff) == 0x001f && rbx_video.out_color(0x84, 0x82, 0x21) == 0x8404;
	deco16ic_update(&s_snap->tile[0]);
	deco16ic_update(&s_snap->tile[1]);
	decoband_run(&s_bands);
	rbx_video.out_done = 1;
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->tile, s_tile, sizeof(s_tile));
	memcpy(s_snap->rowscroll, s_rowscroll, sizeof(s_rowscroll));
	memcpy(s_snap->spriteram, s_spriteram_buf, sizeof(s_spriteram_buf));
	deco_ace_pens(&s_ace);
	memcpy(s_snap->pens, s_ace.pens, sizeof(s_ace.pens));
	memcpy(s_snap->ace, s_ace.ram, sizeof(s_ace.ram));
	s_snap->priority = s_priority;
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void bw_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_spriteram_buf, 0, sizeof(s_spriteram_buf));
	memset(s_rowscroll, 0, sizeof(s_rowscroll));
	s_priority = 0;
	deco_ace_reset(&s_ace);
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

static void bw_exit(void);

static int bw_init(const rbx_game *g)
{
	if (!s_pri_hp && !(s_pri_hp = calloc(1, sizeof(*s_pri_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ram_hp && !(s_ram_hp = calloc(1, sizeof(*s_ram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_rowscroll_hp && !(s_rowscroll_hp = calloc(1, sizeof(*s_rowscroll_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_tile_hp && !(s_tile_hp = calloc(1, sizeof(*s_tile_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t snd_size, t1s, t2s, t3s, s1s, s2s, hs, o1s, o2s;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *snd = rbx_region("audiocpu", &snd_size), *t1 = rbx_region("tiles1", &t1s), *t2 = rbx_region("tiles2", &t2s),
	        *t3 = rbx_region("tiles3", &t3s), *spr1 = rbx_region("sprites1", &s1s), *spr2 = rbx_region("sprites2", &s2s),
	        *hi = rbx_region("tiles2_hi", &hs), *oki1 = rbx_region("oki1", &o1s), *oki2 = rbx_region("oki2", &o2s);
	if (!s_rom || s_rom_size < 0x100000 || !snd || !t1 || !t2 || t2s < 0x300000 || !t3 || !spr1 || !spr2 || !hi ||
	    hs < 0x100000 || !oki1 || !oki2) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	if (deco56_decrypt_gfx(t1, t1s) || deco56_decrypt_gfx(t2, t2s) || deco56_decrypt_gfx(t3, t3s) ||
	    deco56_remap_gfx(hi, hs) || !(s_opcodes = (uint16_t *)malloc(0x100000)) ||
	    deco102_decrypt_cpu((uint16_t *)s_rom, s_opcodes, 0x100000, 0x42ba, 0x00, 0x18)) {
		rbx_fail("%s: out of memory", g->set);
		bw_exit();
		return -1;
	}
	memcpy(t2 + 0x200000, hi, 0x100000);   // (the 5th plane)
	{	// tile_8x8_layout, tile_16x16_layout (RGN_FRAC(1,2)), tile_16x16_layout_5bpp (RGN_FRAC(1,3))
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 16);
			x16[i] = (uint32_t)(256 + i);
			x16[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 16);
		const uint32_t h1 = t1s * 4, h3 = t3s * 4, th = t2s / 3 * 8, hs1 = s1s * 4, hs2 = s2s * 4;
		const uint32_t p1[4] = { h1 + 8, h1, 8, 0 }, p3[4] = { h3 + 8, h3, 8, 0 }, p5[5] = { 2 * th, th + 8, th, 8, 0 };
		const uint32_t ps1[4] = { hs1 + 8, hs1, 8, 0 }, ps2[4] = { hs2 + 8, hs2, 8, 0 };
		const rbx_gfx_layout l8 = { 8, 8, 4, p1, x8, y8, 8 * 16 };
		const rbx_gfx_layout l5 = { 16, 16, 5, p5, x16, y16, 32 * 16 }, l3 = { 16, 16, 4, p3, x16, y16, 32 * 16 };
		const rbx_gfx_layout ls1 = { 16, 16, 4, ps1, x16, y16, 32 * 16 }, ls2 = { 16, 16, 4, ps2, x16, y16, 32 * 16 };
		if (rbx_gfx_decode(&s_gfx8, &l8, t1, t1s / 2) || rbx_gfx_decode(&s_gfx16a, &l5, t2, t2s / 3) ||
		    rbx_gfx_decode(&s_gfx16b, &l3, t3, t3s / 2) || rbx_gfx_decode(&s_gfxspr1, &ls1, spr1, s1s / 2) ||
		    rbx_gfx_decode(&s_gfxspr2, &ls2, spr2, s2s / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			bw_exit();
			return -1;
		}
	}
	memset(s_tile, 0, sizeof(s_tile));
	for (int c = 0; c < 2; c++) {
		for (int i = 0; i < 2; i++) {
			s_tile[c].tm[i].size = DECO_64x32;
			s_tile[c].tm[i].col_mask = 0x0f;
		}
		s_tile[c].gfx8 = &s_gfx8;
		s_tile[c].base8 = 0x800;   // (gfx_boogwing: the chars from 0x800, tiles 2 from 0x100, tiles 3 from 0x300)
	}
	s_tile[1].tm[1].col_bank = 16;
	s_tile[0].tm[1].bank_cb = bank_cb;
	s_tile[1].tm[0].bank_cb = s_tile[1].tm[1].bank_cb = bank_cb2;
	s_tile[0].gfx16 = &s_gfx16a;
	s_tile[0].gran16 = 5;
	s_tile[0].base16 = 0x100;
	s_tile[1].gfx16 = &s_gfx16b;
	s_tile[1].base16 = 0x300;
	for (int c = 0; c < 2; c++) {   // (alloc_sprite_bitmap: raw shift 4)
		memset(&s_spr[c], 0, sizeof(s_spr[c]));
		s_spr[c].gfx = c ? &s_gfxspr2 : &s_gfxspr1;
		s_spr[c].raw = 1;
		s_spr[c].raw_shift = 4;
	}
	deco146_init(&s_prot, 1);
	static const uint8_t reverse[10] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };   // set_interface_scramble_reverse
	deco146_set_scramble(&s_prot, reverse);
	s_prot.magic_read_xor_on = 1;
	s_prot.port_a = port_a;
	s_prot.port_b = port_b;
	s_prot.port_c = port_c;
	s_prot.soundlatch_irq = latch_irq;
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_snap) {
		rbx_fail("%s: out of memory", g->set);
		bw_exit();
		return -1;
	}
	// sound: HuC6280 8.05 MHz, YM2151 3.58 MHz 0.32 (banking both OKIs), OKI 1 MHz 0.56, OKI 2 MHz 0.12, stereo
	deco16snd_cfg sc;
	memset(&sc, 0, sizeof(sc));
	sc.cpu_clock = 32220000 / 4;
	sc.ym_clock = 32220000 / 9;
	sc.ym_gain[0] = sc.ym_gain[1] = 82;
	sc.oki_rom[0] = oki1;
	sc.oki_size[0] = o1s;
	sc.oki_clock[0] = 32220000 / 32;
	sc.oki_gain[0] = 143;
	sc.oki_rom[1] = oki2;
	sc.oki_size[1] = o2s;
	sc.oki_clock[1] = 32220000 / 16;
	sc.oki_gain[1] = 31;
	sc.stereo = 1;
	sc.ym_port = ym_port;
	sc.latch_r = latch_r;
	if (deco16snd_init(snd, snd_size, &sc, FRAME_MHZ, LINES) || rbx_video_setup(VIS_W, VIS_H, 0x800)) {
		rbx_fail("%s: out of memory", g->set);
		bw_exit();
		return -1;
	}
	s_port_in = rbx_port_find("INPUTS");
	s_port_dsw = rbx_port_find("DSW");
	s_port_sys = rbx_port_find("SYSTEM");
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0xfffff, s_rom, 0x100000, 1, 0);
	m68k_bus.op = (const uint8_t *)s_opcodes;
	m68k_bus.op_size = 0x100000;
	m68k_bus_map(0x200000, 0x20ffff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x242000, 0x242fff, s_spriteram[0], sizeof(s_spriteram[0]), 1, 1);
	m68k_bus_map(0x246000, 0x246fff, s_spriteram[1], sizeof(s_spriteram[1]), 1, 1);
	m68k_bus_map(0x264000, 0x265fff, s_tile[0].vram[0], sizeof(s_tile[0].vram[0]), 1, 1);
	m68k_bus_map(0x266000, 0x267fff, s_tile[0].vram[1], sizeof(s_tile[0].vram[1]), 1, 1);
	m68k_bus_map(0x268000, 0x268fff, s_rowscroll[0], 0x1000, 1, 1);
	m68k_bus_map(0x26a000, 0x26afff, s_rowscroll[1], 0x1000, 1, 1);
	m68k_bus_map(0x274000, 0x275fff, s_tile[1].vram[0], sizeof(s_tile[1].vram[0]), 1, 1);
	m68k_bus_map(0x276000, 0x277fff, s_tile[1].vram[1], sizeof(s_tile[1].vram[1]), 1, 1);
	m68k_bus_map(0x278000, 0x278fff, s_rowscroll[2], 0x1000, 1, 1);
	m68k_bus_map(0x27a000, 0x27afff, s_rowscroll[3], 0x1000, 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	bw_reset();
	return 0;
}

static void bw_exit(void)
{
	deco16snd_exit();
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16a);
	rbx_gfx_free(&s_gfx16b);
	rbx_gfx_free(&s_gfxspr1);
	rbx_gfx_free(&s_gfxspr2);
	free(s_snap);
	s_snap = NULL;
	m68k_bus.op = NULL;
	m68k_bus.op_size = 0;
	free(s_opcodes);
	s_opcodes = NULL;
	free(s_pri_hp);
	s_pri_hp = NULL;
	free(s_ram_hp);
	s_ram_hp = NULL;
	free(s_rowscroll_hp);
	s_rowscroll_hp = NULL;
	free(s_tile_hp);
	s_tile_hp = NULL;
}

static void bw_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // irq6_line_hold at vblank
			vblank_capture();
			m68k_bus_hold_irq(6);
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

static int bw_state(void *ctx, int load)
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
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "sprb", s_spriteram_buf, sizeof(s_spriteram_buf));
	rc |= rbx_state_io(ctx, "aceram", s_ace.ram, sizeof(s_ace.ram));
	rc |= rbx_state_io(ctx, "acepal", s_ace.palram, sizeof(s_ace.palram));
	rc |= rbx_state_io(ctx, "acebuf", s_ace.palbuf, sizeof(s_ace.palbuf));
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
		deco_ace_update(&s_ace);   // (MAME device_post_load)
		for (int c = 0; c < 2; c++)
			for (int i = 0; i < 8; i++)
				deco16ic_control_w(&s_tile[c], i, s_tile[c].control[i], 0xffff);
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int bw_save(void *ctx) { return bw_state(ctx, 0); }
static int bw_load(void *ctx) { return bw_state(ctx, 1); }
static int bw_supported(const char *machine) { return !strcmp(machine, "boogwing"); }

const rbx_driver rbx_drv_boogwing = { "DECO 16 Boogie Wings", bw_init, bw_exit, bw_reset, bw_frame, bw_save, bw_load,
                                      bw_supported, bw_render, bw_help };
