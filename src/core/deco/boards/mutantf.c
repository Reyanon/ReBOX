/*  Data East DE-0364: Mutant Fighter / Death Brade. Ported from MAME src/mame/dataeast/cninja.cpp, cninja_v.cpp
    (BSD-3-Clause, copyright-holders: Bryan McPhail).
    68000 at 14 MHz (IRQ 6 at vblank, HOLD_LINE), the 146 protection / I/O chip, two tilemap chips (chars and tiles 1
    encrypted, the chars partly in tiles 1), two sprite chips 52 in the alternate format drawn raw (the second one mixed
    at half alpha, their order from a priority word), the sound board (machine/deco16snd.c: HuC6280, YM2151 banking the
    second OKI). The output is RGB (the blending).
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
#include "../video/decospr.h"
#include "../video/decoband.h"

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
static uint16_t s_ram[0x2000];            // 0x100000 (16 KB)
static uint16_t s_spriteram[2][0x800], s_spriteram_buf[2][0x400];
static uint16_t s_palram[0x1000];         // 0x160000: xBGR_888, 2048 entries of two words
static uint16_t s_rowscroll[4][0x800];
static uint16_t s_priority;
static deco16ic_t s_tile[2];
static decospr_t s_spr[2];
static deco146_t s_prot;
static rbx_gfx s_gfx8, s_gfx16a, s_gfx16b, s_gfxspr1, s_gfxspr2;
static int s_port_in, s_port_dsw, s_port_sys;
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac;
static int s_565;
static uint16_t s_bm[VIS_W * VIS_H], s_top[VIS_W * VIS_H], s_sbm[2][VIS_W * VIS_H];
static struct {
	deco16ic_t tile[2];
	uint16_t rowscroll[4][0x800], spriteram[2][0x400], priority;
	uint32_t pens[0x800];
	uint32_t frame;
} *s_snap;

// ---- the 146's ports and latch ----
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
static void ym_port(uint8_t data) { deco16snd_oki(1)->bank = (data & 1) * 0x40000u; }   // sound_bankswitch_w

// ---- bus ----
static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x1a0000 && a < 0x1a4000) return deco146_read(&s_prot, (uint16_t)((a - 0x1a0000) & 0x7fe), NULL);
	if (a == 0x1c0000) return 0xffff;   // mutantf_71_r
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
	if (a >= 0x300000 && a < 0x300010)
		deco16ic_control_w(&s_tile[0], (int)((a >> 1) & 7), v, mask);
	else if (a >= 0x310000 && a < 0x310010)
		deco16ic_control_w(&s_tile[1], (int)((a >> 1) & 7), v, mask);
	else if (a >= 0x1a0000 && a < 0x1a4000)
		deco146_write(&s_prot, (uint16_t)((a - 0x1a0000) & 0x7fe), v, mask, NULL);
	else if (a == 0x1c0000)   // the sprite DMAs
		memcpy(s_spriteram_buf[0], s_spriteram[0], sizeof(s_spriteram_buf[0]));
	else if (a == 0x1e0000)
		memcpy(s_spriteram_buf[1], s_spriteram[1], sizeof(s_spriteram_buf[1]));
	else if (a == 0x180000)   // robocop2_priority_w
		s_priority = (uint16_t)((s_priority & ~mask) | (v & mask));
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static int robocop2_bank_cb(int bank) { return (bank & 0x30) << 8; }
static int mutantf_2_bank_cb(int bank) { return (bank & 0x20) << 9; }

static inline uint32_t blend(uint32_t d, uint32_t s, int level)   // MAME alpha_blend_r32
{
	return ((((s & 0xff00ff) * (uint32_t)level + (d & 0xff00ff) * (uint32_t)(256 - level)) >> 8) & 0xff00ff) |
	       ((((s & 0x00ff00) * (uint32_t)level + (d & 0x00ff00) * (uint32_t)(256 - level)) >> 8) & 0x00ff00);
}

static void mf_band(int band)
{
	const int r0 = VIS_H * band / BANDS, r1 = VIS_H * (band + 1) / BANDS, n = (r1 - r0) * VIS_W;
	const uint32_t *pens = s_snap->pens;
	deco16ic_t *t0 = &s_snap->tile[0], *t1 = &s_snap->tile[1];
	memset(s_sbm[0] + r0 * VIS_W, 0, (size_t)n * 2);
	memset(s_sbm[1] + r0 * VIS_W, 0, (size_t)n * 2);
	for (int i = 0; i < n; i++)
		s_top[r0 * VIS_W + i] = 0xffff;
	const int flip = !((deco16ic_control_r(t0, 0) >> 7) & 1);   // (sprites are flipped relative to the tilemaps)
	for (int c = 1; c >= 0; c--) {
		decospr_t spr = s_spr[c];
		spr.flip_screen = flip;
		decospr_draw(&spr, s_snap->spriteram[c], 0x400, s_sbm[c] + r0 * VIS_W, NULL, VIS_W, VIS_W, r1 - r0, VIS_Y0 + r0, 1,
		             s_snap->frame);
	}
	deco16ic_draw(t1, 1, s_snap->rowscroll[3], s_bm, NULL, VIS_W, VIS_W, VIS_Y0, r0, r1, 1, 0, 0xff);
	deco16ic_draw(t0, 1, s_snap->rowscroll[1], s_bm, NULL, VIS_W, VIS_W, VIS_Y0, r0, r1, 0, 0, 0xff);
	deco16ic_draw(t1, 0, s_snap->rowscroll[2], s_bm, NULL, VIS_W, VIS_W, VIS_Y0, r0, r1, 0, 0, 0xff);
	deco16ic_draw(t0, 0, s_snap->rowscroll[0], s_top, NULL, VIS_W, VIS_W, VIS_Y0, r0, r1, 0, 0, 0xff);
	const int pri1 = s_snap->priority & 1;
	for (int y = r0; y < r1; y++) {
		uint32_t d[VIS_W];
		const uint16_t *bm = s_bm + y * VIS_W, *s0 = s_sbm[0] + y * VIS_W, *s1 = s_sbm[1] + y * VIS_W,
		               *top = s_top + y * VIS_W;
		for (int x = 0; x < VIS_W; x++) {
			uint32_t c = pens[bm[x] & 0x7ff];
			const uint16_t p0 = s0[x], p1 = s1[x];
			// inefficient_copy_sprite_bitmap: chip 1 at 0x100 (0x1ff), chip 2 at 0x700 (0xff) blended 0x80
			if (pri1) {
				if (p0 & 0xf) c = pens[((p0 & 0x1ff) + 0x100) & 0x7ff];
				if (p1 & 0xf) c = blend(c, pens[((p1 & 0xff) + 0x700) & 0x7ff], 0x80);
			} else {
				if (p1 & 0xf) c = blend(c, pens[((p1 & 0xff) + 0x700) & 0x7ff], 0x80);
				if (p0 & 0xf) c = pens[((p0 & 0x1ff) + 0x100) & 0x7ff];
			}
			if (top[x] != 0xffff)
				c = pens[top[x] & 0x7ff];
			d[x] = c;
		}
		uint8_t *o = (uint8_t *)rbx_video.out + (size_t)y * rbx_video.out_pitch;
		if (s_565)
			for (int x = 0; x < VIS_W; x++)
				((uint16_t *)o)[x] = (uint16_t)((d[x] >> 8 & 0xf800) | (d[x] >> 5 & 0x07e0) | (d[x] >> 3 & 0x001f));
		else if (rbx_video.out_bpp == 4)
			for (int x = 0; x < VIS_W; x++)
				((uint32_t *)o)[x] = rbx_video.out_color((int)(d[x] >> 16) & 0xff, (int)(d[x] >> 8) & 0xff, (int)d[x] & 0xff);
		else
			for (int x = 0; x < VIS_W; x++)
				((uint16_t *)o)[x] = (uint16_t)rbx_video.out_color((int)(d[x] >> 16) & 0xff, (int)(d[x] >> 8) & 0xff, (int)d[x] & 0xff);
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, mf_band };
static int mf_help(int helper) { return decoband_help(&s_bands, helper); }

static void mf_render(void)
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
	for (int i = 0; i < 0x800; i++) {   // xBGR_888 (the entry's two words, big-endian)
		const uint32_t p = (uint32_t)s_palram[2 * i] << 16 | s_palram[2 * i + 1];
		s_snap->pens[i] = (p & 0xff) << 16 | (p & 0xff00) | ((p >> 16) & 0xff);
	}
	s_snap->priority = s_priority;
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void mf_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_spriteram_buf, 0, sizeof(s_spriteram_buf));
	memset(s_palram, 0, sizeof(s_palram));
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

static void mf_exit(void);

static int mf_init(const rbx_game *g)
{
	uint32_t snd_size, csize, t1s, t2s, s1s, s2s, o1s, o2s;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *snd = rbx_region("audiocpu", &snd_size), *chars = rbx_region("chars", &csize),
	        *t1 = rbx_region("tiles1", &t1s), *t2 = rbx_region("tiles2", &t2s), *spr1 = rbx_region("sprites1", &s1s),
	        *spr2 = rbx_region("sprites2", &s2s), *oki1 = rbx_region("oki1", &o1s), *oki2 = rbx_region("oki2", &o2s);
	if (!s_rom || !snd || !chars || csize < 0xa0000 || !t1 || t1s < 0x80000 || !t2 || !spr1 || !spr2 || !oki1 || !oki2) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	// init_mutantf: the 16x16 graphics' 8x8 chars into the chars region, then both decrypted
	memcpy(chars + 0x50000, chars + 0x10000, 0x10000);
	memcpy(chars + 0x10000, t1, 0x40000);
	memcpy(chars + 0x60000, t1 + 0x40000, 0x40000);
	if (deco56_decrypt_gfx(chars, csize) || deco56_decrypt_gfx(t1, t1s)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	{	// charlayout, tilelayout (RGN_FRAC(1,2))
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 16);
			x16[i] = (uint32_t)(256 + i);
			x16[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 16);
		const uint32_t ch = csize * 4, h1 = t1s * 4, h2 = t2s * 4, hs1 = s1s * 4, hs2 = s2s * 4;
		const uint32_t pc[4] = { ch + 8, ch, 8, 0 }, p1[4] = { h1 + 8, h1, 8, 0 }, p2[4] = { h2 + 8, h2, 8, 0 };
		const uint32_t ps1[4] = { hs1 + 8, hs1, 8, 0 }, ps2[4] = { hs2 + 8, hs2, 8, 0 };
		const rbx_gfx_layout l8 = { 8, 8, 4, pc, x8, y8, 16 * 8 };
		const rbx_gfx_layout la = { 16, 16, 4, p1, x16, y16, 64 * 8 }, lb = { 16, 16, 4, p2, x16, y16, 64 * 8 };
		const rbx_gfx_layout ls1 = { 16, 16, 4, ps1, x16, y16, 64 * 8 }, ls2 = { 16, 16, 4, ps2, x16, y16, 64 * 8 };
		if (rbx_gfx_decode(&s_gfx8, &l8, chars, csize / 2) || rbx_gfx_decode(&s_gfx16a, &la, t1, t1s / 2) ||
		    rbx_gfx_decode(&s_gfx16b, &lb, t2, t2s / 2) || rbx_gfx_decode(&s_gfxspr1, &ls1, spr1, s1s / 2) ||
		    rbx_gfx_decode(&s_gfxspr2, &ls2, spr2, s2s / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			mf_exit();
			return -1;
		}
	}
	memset(s_tile, 0, sizeof(s_tile));
	static const int banks[2][2] = { { 0x00, 0x30 }, { 0x20, 0x40 } };
	for (int c = 0; c < 2; c++) {
		for (int i = 0; i < 2; i++) {
			s_tile[c].tm[i].size = DECO_64x32;
			s_tile[c].tm[i].col_bank = banks[c][i];
			s_tile[c].tm[i].col_mask = 0x0f;
			s_tile[c].tm[i].bank_cb = robocop2_bank_cb;
		}
		s_tile[c].gfx8 = &s_gfx8;
	}
	s_tile[0].tm[1].bank_cb = mutantf_2_bank_cb;
	s_tile[0].gfx16 = &s_gfx16a;
	s_tile[1].gfx16 = &s_gfx16b;
	for (int c = 0; c < 2; c++) {   // (alloc_sprite_bitmap: raw shift 4)
		memset(&s_spr[c], 0, sizeof(s_spr[c]));
		s_spr[c].gfx = c ? &s_gfxspr2 : &s_gfxspr1;
		s_spr[c].alt_format = 1;
		s_spr[c].raw = 1;
		s_spr[c].raw_shift = 4;
	}
	deco146_init(&s_prot, 0);
	s_prot.port_a = port_a;
	s_prot.port_b = port_b;
	s_prot.port_c = port_c;
	s_prot.soundlatch_irq = latch_irq;
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_snap) {
		rbx_fail("%s: out of memory", g->set);
		mf_exit();
		return -1;
	}
	// sound: HuC6280 4.03 MHz, YM2151 3.58 MHz 0.45 (banking the second OKI), OKI 1 MHz 0.75, OKI 2 MHz 0.60, one
	// speaker
	deco16snd_cfg sc;
	memset(&sc, 0, sizeof(sc));
	sc.cpu_clock = 32220000 / 8;
	sc.ym_clock = 32220000 / 9;
	sc.ym_gain[0] = sc.ym_gain[1] = 115;
	sc.oki_rom[0] = oki1;
	sc.oki_size[0] = o1s;
	sc.oki_clock[0] = 32220000 / 32;
	sc.oki_gain[0] = 192;
	sc.oki_rom[1] = oki2;
	sc.oki_size[1] = o2s;
	sc.oki_clock[1] = 32220000 / 16;
	sc.oki_gain[1] = 154;
	sc.ym_port = ym_port;
	sc.latch_r = latch_r;
	if (deco16snd_init(snd, snd_size, &sc, FRAME_MHZ, LINES) || rbx_video_setup(VIS_W, VIS_H, 0x800)) {
		rbx_fail("%s: out of memory", g->set);
		mf_exit();
		return -1;
	}
	s_port_in = rbx_port_find("INPUTS");
	s_port_dsw = rbx_port_find("DSW");
	s_port_sys = rbx_port_find("SYSTEM");
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x7ffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x100000, 0x103fff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x120000, 0x120fff, s_spriteram[0], sizeof(s_spriteram[0]), 1, 1);
	m68k_bus_map(0x140000, 0x140fff, s_spriteram[1], sizeof(s_spriteram[1]), 1, 1);
	m68k_bus_map(0x160000, 0x161fff, s_palram, sizeof(s_palram), 1, 1);
	m68k_bus_map(0x304000, 0x305fff, s_tile[0].vram[0], sizeof(s_tile[0].vram[0]), 1, 1);
	m68k_bus_map(0x306000, 0x307fff, s_tile[0].vram[1], sizeof(s_tile[0].vram[1]), 1, 1);
	m68k_bus_map(0x308000, 0x308fff, s_rowscroll[0], 0x1000, 1, 1);
	m68k_bus_map(0x30a000, 0x30afff, s_rowscroll[1], 0x1000, 1, 1);
	m68k_bus_map(0x314000, 0x315fff, s_tile[1].vram[0], sizeof(s_tile[1].vram[0]), 1, 1);
	m68k_bus_map(0x316000, 0x317fff, s_tile[1].vram[1], sizeof(s_tile[1].vram[1]), 1, 1);
	m68k_bus_map(0x318000, 0x318fff, s_rowscroll[2], 0x1000, 1, 1);
	m68k_bus_map(0x31a000, 0x31afff, s_rowscroll[3], 0x1000, 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	mf_reset();
	return 0;
}

static void mf_exit(void)
{
	deco16snd_exit();
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16a);
	rbx_gfx_free(&s_gfx16b);
	rbx_gfx_free(&s_gfxspr1);
	rbx_gfx_free(&s_gfxspr2);
	free(s_snap);
	s_snap = NULL;
}

static void mf_frame(void)
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

static int mf_state(void *ctx, int load)
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
	rc |= rbx_state_io(ctx, "palram", s_palram, sizeof(s_palram));
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

static int mf_save(void *ctx) { return mf_state(ctx, 0); }
static int mf_load(void *ctx) { return mf_state(ctx, 1); }
static int mf_supported(const char *machine) { return !strcmp(machine, "mutantf"); }

// (the cninja board's driver hands this machine here: cninja.c)
const rbx_driver rbx_drv_mutantf = { "DECO 16 Caveman Ninja", mf_init, mf_exit, mf_reset, mf_frame, mf_save, mf_load,
                                     mf_supported, mf_render, mf_help };
