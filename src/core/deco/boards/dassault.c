/*  Data East DE-0359-2: Thunder Zone / Desert Assault. Ported from MAME src/mame/dataeast/dassault.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail).
    Two 68000s at 14 MHz (cpu/m68k_multi.c; IRQ 4 / 5 at vblank until acknowledged) talking through an MB8421 mailbox
    RAM (its last words raise the other CPU's interrupt), two tilemap chips, two sprite chips 52 drawn raw and mixed
    between the playfields by their priority bits (the second chip at half alpha), the sound board
    (machine/deco16snd.c: HuC6280, YM2203, YM2151 banking the second OKI, stereo). The output is RGB.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k_multi.h"
#include "../../cpu/m68k/m68k.h"
#include "../../video/gfxlayout.h"
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
#define BANDS 6
#define SLICES 4                 // the CPUs interleaved a quarter line (MAME: a quantum of clock / 4)

static uint8_t *s_rom[2];
static uint32_t s_rom_size[2];
static uint16_t s_ram[2][0x2000];         // 0x3f8000 of each CPU (16 KB)
static uint16_t s_spriteram[2][0x800], s_spriteram_buf[2][0x400];
static uint16_t s_shared[0x800];          // the MB8421 (11-bit addresses)
static uint16_t s_palram[0x2000];         // 0x100000: xBGR_888, 4096 entries of two words
static uint16_t s_rowscroll[2][0x800];    // rowscroll_2, rowscroll_4
static uint16_t s_priority;
static deco16ic_t s_tile[2];
static decospr_t s_spr[2];
static rbx_gfx s_gfx8, s_gfx16a, s_gfx16b, s_gfxspr1, s_gfxspr2;
static int s_port[5], s_port_vbl;
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac[2];
static int s_565;
static uint16_t s_l[4][VIS_W * VIS_H];    // the back, middle, front-middle and top playfields' pens (0xffff: none)
static uint16_t s_sbm[2][VIS_W * VIS_H];
static struct {
	deco16ic_t tile[2];
	uint16_t rowscroll[2][0x800], spriteram[2][0x400], priority;
	uint32_t pens[0x1000];
	uint32_t frame;
	uint16_t pens565[0x1000];   // (the pens as the 565 output has them: at the render's start)
} *s_snap;

static void ym_port(uint8_t data) { deco16snd_oki(1)->bank = (data & 1) * 0x40000u; }   // sound_bankswitch_w

// the waits of both programs: BTST #n, (abs).L then BNE back to it (main 0x114c on its RAM, 0xb98 on the vblank
// bit; sub 0xfb0, 0xfc2). What they read changes only between the CPUs' slices (the interrupts, the other CPU, the
// lines), so when the same pc reads the same value at the same interval, the whole iterations up to the slice's end
// pass at once (the same reads, flags and cycles as without)
static struct { uint32_t pc, val, slice; int t, period; } s_poll[2];
static uint32_t s_slice;
static void poll_read(int cpu, uint32_t v)
{
	const uint32_t pc = (uint32_t)m68k_get_reg(NULL, M68K_REG_PPC) & 0xfffffe;
	const int now = m68k_cycles_run();
	const int same = pc == s_poll[cpu].pc && v == s_poll[cpu].val && s_poll[cpu].slice == s_slice;
	if (same && s_poll[cpu].period > 0 && now - s_poll[cpu].t == s_poll[cpu].period && pc + 10 <= s_rom_size[cpu]) {
		const uint16_t *w = (const uint16_t *)(s_rom[cpu] + pc);   // (host-order words)
		if (w[0] == 0x0839 && w[4] == 0x66f6) {
			const int per = s_poll[cpu].period, k = (m68k_cycles_remaining() - 1) / per;
			if (k > 0)
				m68k_modify_timeslice(-k * per);
		}
	}
	s_poll[cpu].period = same ? now - s_poll[cpu].t : 0;
	s_poll[cpu].pc = pc;
	s_poll[cpu].val = v;
	s_poll[cpu].t = now;
	s_poll[cpu].slice = s_slice;
}

static int vblank(void) { return s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H; }

// ---- the mailbox (MAME mb8421: left = the main CPU, right = the sub) ----
static uint16_t shared_r(int right, uint32_t o)
{
	o &= 0x7ff;
	if (!right && o == 0x7fe) m68k_multi_set_line(0, 5, 0);   // (INTL acknowledged)
	if (right && o == 0x7ff) m68k_multi_set_line(1, 6, 0);    // (INTR acknowledged)
	return s_shared[o];
}
static void shared_w(int right, uint32_t o, uint16_t v, uint16_t mask)
{
	o &= 0x7ff;
	s_shared[o] = (uint16_t)((s_shared[o] & ~mask) | (v & mask));
	if (!right && o == 0x7ff) m68k_multi_set_line(1, 6, 1);   // INTR to the sub
	if (right && o == 0x7fe) m68k_multi_set_line(0, 5, 1);    // INTL to the main
}

// ---- main CPU ----
static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x3fe000 && a < 0x3ff000) return shared_r(0, (a - 0x3fe000) >> 1);
	if (a >= 0x3f8000 && a < 0x3f9000) {   // (the RAM page with the wait's flag)
		const uint16_t v = s_ram[0][(a - 0x3f8000) >> 1];
		poll_read(0, v);
		return v;
	}
	if (a >= 0x1c0000 && a < 0x1c0010) {   // control_r
		const int o = (int)((a - 0x1c0000) >> 1);
		if (o == 4) {
			rbx_port_custom(s_port[4], 0x08, vblank() ? 0x08 : 0);
			const uint16_t v = (uint16_t)rbx_port_read(s_port[4]);
			poll_read(0, v);
			return v;
		}
		return o < 4 ? (uint16_t)rbx_port_read(s_port[o]) : 0xffff;
	}
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
	if (a >= 0x3fe000 && a < 0x3ff000)
		shared_w(0, (a - 0x3fe000) >> 1, v, mask);
	else if (a >= 0x220000 && a < 0x220010)
		deco16ic_control_w(&s_tile[0], (int)((a >> 1) & 7), v, mask);
	else if (a >= 0x260000 && a < 0x260010)
		deco16ic_control_w(&s_tile[1], (int)((a >> 1) & 7), v, mask);
	else if (a == 0x140004)   // main_irq_ack_w
		m68k_multi_set_line(0, 4, 0);
	else if (a == 0x180000) { if (mask & 0xff) deco16snd_latch_w((uint8_t)v); }
	else if (a == 0x1c000a)   // priority_w
		s_priority = v;
	else if (a == 0x1c000c)   // the second sprite chip's DMA
		memcpy(s_spriteram_buf[1], s_spriteram[1], sizeof(s_spriteram_buf[1]));
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- sub CPU ----
static uint16_t sub_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x3fe000 && a < 0x3ff000) return shared_r(1, (a - 0x3fe000) >> 1);
	if (a >= 0x3fa000 && a < 0x3fb000) {   // (the RAM page with the waits' flags)
		const uint16_t v = s_ram[1][(a - 0x3f8000) >> 1];
		poll_read(1, v);
		return v;
	}
	if (a == 0x100004) {   // VBLANK1
		rbx_port_custom(s_port_vbl, 0xff, vblank() ? 0xff : 0);
		return (uint16_t)rbx_port_read(s_port_vbl);
	}
	return 0;
}
static uint8_t sub_read8(uint32_t a)
{
	const uint16_t v = sub_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static void sub_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a >= 0x3fe000 && a < 0x3ff000)
		shared_w(1, (a - 0x3fe000) >> 1, v, mask);
	else if (a == 0x100000)   // the first sprite chip's DMA
		memcpy(s_spriteram_buf[0], s_spriteram[0], sizeof(s_spriteram_buf[0]));
	else if (a == 0x100002)   // sub_irq_ack_w
		m68k_multi_set_line(1, 5, 0);
}
static void sub_write16(uint32_t a, uint16_t v) { sub_write16m(a, v, 0xffff); }
static void sub_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	sub_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static int bank_cb(int bank) { return (bank & 0xf0) << 8; }

static inline uint32_t blend(uint32_t d, uint32_t s, int level)   // MAME alpha_blend_r32
{
	return ((((s & 0xff00ff) * (uint32_t)level + (d & 0xff00ff) * (uint32_t)(256 - level)) >> 8) & 0xff00ff) |
	       ((((s & 0x00ff00) * (uint32_t)level + (d & 0x00ff00) * (uint32_t)(256 - level)) >> 8) & 0x00ff00);
}

// the drawing order of screen_update: playfield layers (L) and mix_layer calls of the first chip (S1 with its
// priority) and the second (S2, alpha 0x80)
enum { OP_L1 = 1, OP_L2, OP_S1, OP_S2 };
typedef struct { uint8_t op; uint16_t pri; } dz_op;
static const dz_op s_ops[4][7] = {
	// 0: tg0 tm1 (layer 1), tg1 tm0 (layer 2)
	{ { OP_S1, 0x600 }, { OP_L1, 0 }, { OP_S1, 0x400 }, { OP_L2, 0 }, { OP_S1, 0x200 }, { OP_S2, 0 }, { OP_S1, 0 } },
	// 1: tg1 tm0, tg0 tm1
	{ { OP_S1, 0x600 }, { OP_L2, 0 }, { OP_S1, 0x400 }, { OP_S2, 0 }, { OP_S1, 0x200 }, { OP_L1, 0 }, { OP_S1, 0 } },
	{ { 0, 0 } },   // (2: unused)
	// 3: tg1 tm0, tg0 tm1
	{ { OP_S1, 0x600 }, { OP_L2, 0 }, { OP_S1, 0x400 }, { OP_L1, 0 }, { OP_S1, 0x200 }, { OP_S2, 0 }, { OP_S1, 0 } },
};

static void dz_band(int band)
{
	const int r0 = VIS_H * band / BANDS, r1 = VIS_H * (band + 1) / BANDS, n = (r1 - r0) * VIS_W, o = r0 * VIS_W;
	deco16ic_t *t0 = &s_snap->tile[0], *t1 = &s_snap->tile[1];
	const uint32_t *pens = s_snap->pens;
	for (int k = 0; k < 2; k++)
		memset(s_sbm[k] + o, 0, (size_t)n * 2);
	for (int k = 0; k < 4; k++)
		memset(s_l[k] + o, 0xff, (size_t)n * 2);   // (0xffff: none)
	const int flip = (deco16ic_control_r(t0, 0) >> 7) & 1;
	for (int c = 1; c >= 0; c--) {
		decospr_t spr = s_spr[c];
		spr.flip_screen = flip;
		decospr_draw(&spr, s_snap->spriteram[c], 0x400, s_sbm[c] + o, NULL, VIS_W, VIS_W, r1 - r0, VIS_Y0 + r0, 1,
		             s_snap->frame);
	}
	// (update(nullptr, rowscroll): only the second playfields have row scroll)
	deco16ic_draw(t1, 1, s_snap->rowscroll[1], s_l[0], NULL, VIS_W, VIS_W, VIS_Y0, r0, r1, 1, 0, 0xff);
	deco16ic_draw(t0, 1, s_snap->rowscroll[0], s_l[1], NULL, VIS_W, VIS_W, VIS_Y0, r0, r1, 0, 0, 0xff);
	deco16ic_draw(t1, 0, NULL, s_l[2], NULL, VIS_W, VIS_W, VIS_Y0, r0, r1, 0, 0, 0xff);
	deco16ic_draw(t0, 0, NULL, s_l[3], NULL, VIS_W, VIS_W, VIS_Y0, r0, r1, 0, 0, 0xff);
	const int mode = s_snap->priority & 3;
	const dz_op *ops = s_ops[mode];
	for (int y = r0; y < r1; y++) {
		// each pixel's pen (0x1000 + pen: the second chip's sprite at alpha 0x80 over the pen in u[])
		uint16_t d[VIS_W], u[VIS_W];
		const int i0 = y * VIS_W;
		for (int x = 0; x < VIS_W; x++) {
			const int i = i0 + x;
			unsigned c = s_l[0][i] != 0xffff ? s_l[0][i] & 0xfffu : 3072u;
			if (mode != 2) {
				const uint16_t p1 = s_sbm[0][i], p2 = s_sbm[1][i], a = s_l[1][i], b = s_l[2][i];
				if (!((p1 | p2) & 0xf)) {   // (no sprite: the two playfields in this mode's order)
					const int l1_first = mode == 0;
					const uint16_t f = l1_first ? a : b, s2 = l1_first ? b : a;
					if (f != 0xffff) c = f & 0xfffu;
					if (s2 != 0xffff) c = s2 & 0xfffu;
				} else {
					// the first chip's pixel goes in at one place of the order (its priority bits: 0x600 first ..
					// 0x000 last, the slots 0 / 2 / 4 / 6 of s_ops); the rest as the order has them
					const int slot1 = (p1 & 0xf) ? 6 - (((p1 >> 9) & 3) << 1) : -1;
					const unsigned c1 = ((p1 & 0x1ffu) + (p1 & 0x800 ? 0x200u : 0) + 0x400u) & 0xfffu;
					for (int k = 0; k < 7; k++) {
						const uint8_t o = ops[k].op;
						if (o == OP_S1) { if (k == slot1) c = c1; }
						else if (o == OP_L1) { if (a != 0xffff) c = a & 0xfffu; }
						else if (o == OP_L2) { if (b != 0xffff) c = b & 0xfffu; }
						else if (p2 & 0xf) {   // mix_layer(sprite 2, 0, 0, 0x800, 0x80)
							const unsigned sc = ((p2 & 0x1ffu) + (p2 & 0x800 ? 0x200u : 0) + 0x800u) & 0xfffu;
							if (p2 & 0x600) {   // (one S2 step in an order: c a plain pen here)
								u[x] = (uint16_t)c;
								c = 0x1000u | sc;
							} else
								c = sc;
						}
					}
				}
			}
			if (s_l[3][i] != 0xffff)
				c = s_l[3][i] & 0xfffu;
			d[x] = (uint16_t)c;
		}
		uint8_t *op = (uint8_t *)rbx_video.out + (size_t)y * rbx_video.out_pitch;
		if (s_565) {
			const uint16_t *p5 = s_snap->pens565;
			for (int x = 0; x < VIS_W; x++) {
				const unsigned c = d[x];
				if (c & 0x1000) {
					const uint32_t v = blend(pens[u[x]], pens[c & 0xfff], 0x80);
					((uint16_t *)op)[x] = (uint16_t)((v >> 8 & 0xf800) | (v >> 5 & 0x07e0) | (v >> 3 & 0x001f));
				} else
					((uint16_t *)op)[x] = p5[c];
			}
		} else
			for (int x = 0; x < VIS_W; x++) {
				const unsigned c = d[x];
				const uint32_t v = c & 0x1000 ? blend(pens[u[x]], pens[c & 0xfff], 0x80) : pens[c];
				const int r = (int)(v >> 16) & 0xff, g = (int)(v >> 8) & 0xff, bl = (int)v & 0xff;
				if (rbx_video.out_bpp == 4)
					((uint32_t *)op)[x] = rbx_video.out_color(r, g, bl);
				else
					((uint16_t *)op)[x] = (uint16_t)rbx_video.out_color(r, g, bl);
			}
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, dz_band };
static int dz_help(int helper) { return decoband_help(&s_bands, helper); }

static void dz_render(void)
{
	if (!rbx_video.out)
		return;
	s_565 = rbx_video.out_bpp == 2 && rbx_video.out_color(0xff, 0, 0) == 0xf800 && rbx_video.out_color(0, 0xff, 0) == 0x07e0 &&
	        rbx_video.out_color(0, 0, 0xff) == 0x001f && rbx_video.out_color(0x84, 0x82, 0x21) == 0x8404;
	deco16ic_update(&s_snap->tile[0]);
	deco16ic_update(&s_snap->tile[1]);
	if (s_565)
		for (int i = 0; i < 0x1000; i++) {
			const uint32_t v = s_snap->pens[i];
			s_snap->pens565[i] = (uint16_t)((v >> 8 & 0xf800) | (v >> 5 & 0x07e0) | (v >> 3 & 0x001f));
		}
	decoband_run(&s_bands);
	rbx_video.out_done = 1;
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->tile, s_tile, sizeof(s_tile));
	memcpy(s_snap->rowscroll, s_rowscroll, sizeof(s_rowscroll));
	memcpy(s_snap->spriteram, s_spriteram_buf, sizeof(s_spriteram_buf));
	for (int i = 0; i < 0x1000; i++) {   // xBGR_888 (the entry's two words, big-endian)
		const uint32_t p = (uint32_t)s_palram[2 * i] << 16 | s_palram[2 * i + 1];
		s_snap->pens[i] = (p & 0xff) << 16 | (p & 0xff00) | ((p >> 16) & 0xff);
	}
	s_snap->priority = s_priority;
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void dz_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_spriteram_buf, 0, sizeof(s_spriteram_buf));
	memset(s_shared, 0, sizeof(s_shared));
	memset(s_palram, 0, sizeof(s_palram));
	memset(s_rowscroll, 0, sizeof(s_rowscroll));
	s_priority = 0;
	for (int i = 0; i < 2; i++) {
		memset(s_tile[i].vram, 0, sizeof(s_tile[i].vram));
		deco16ic_reset(&s_tile[i]);
	}
	deco16snd_reset();
	m68k_multi_reset();
	for (int c = 0; c < 2; c++)
		m68k_multi_set_asserted(c, 0);
	s_line = 0;
	s_frame = 0;
	s_cpu_frac[0] = s_cpu_frac[1] = 0;
}

static void dz_exit(void);

static int dz_init(const rbx_game *g)
{
	uint32_t snd_size, csize, t1s, t2s, s1s, s2s, o1s, o2s;
	s_rom[0] = rbx_region("maincpu", &s_rom_size[0]);
	s_rom[1] = rbx_region("sub", &s_rom_size[1]);
	uint8_t *snd = rbx_region("audiocpu", &snd_size), *chars = rbx_region("chars", &csize),
	        *t1 = rbx_region("tiles1", &t1s), *t2 = rbx_region("tiles2", &t2s), *spr1 = rbx_region("sprites1", &s1s),
	        *spr2 = rbx_region("sprites2", &s2s), *oki1 = rbx_region("oki1", &o1s), *oki2 = rbx_region("oki2", &o2s);
	if (!s_rom[0] || !s_rom[1] || !snd || !chars || csize < 0x20000 || !t1 || t1s < 0x120000 || !t2 || !spr1 || !spr2 ||
	    !oki1 || !oki2) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{	// init_dassault: the chars into both halves of tiles 1 (playfield 4 reaches them too)
		uint8_t *tmp = (uint8_t *)malloc(0x80000);
		if (!tmp) {
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
		memcpy(tmp, t1 + 0x80000, 0x80000);
		memcpy(t1 + 0x90000, tmp, 0x80000);
		memcpy(t1 + 0x80000, chars, 0x10000);
		memcpy(t1 + 0x110000, chars + 0x10000, 0x10000);
		free(tmp);
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
		const uint32_t h1 = t1s * 4, h2 = t2s * 4, hs1 = s1s * 4, hs2 = s2s * 4;
		const uint32_t p1[4] = { h1 + 8, h1, 8, 0 }, p2[4] = { h2 + 8, h2, 8, 0 };
		const uint32_t ps1[4] = { hs1 + 8, hs1, 8, 0 }, ps2[4] = { hs2 + 8, hs2, 8, 0 };
		const rbx_gfx_layout l8 = { 8, 8, 4, p1, x8, y8, 16 * 8 };
		const rbx_gfx_layout la = { 16, 16, 4, p1, x16, y16, 64 * 8 }, lb = { 16, 16, 4, p2, x16, y16, 64 * 8 };
		const rbx_gfx_layout ls1 = { 16, 16, 4, ps1, x16, y16, 64 * 8 }, ls2 = { 16, 16, 4, ps2, x16, y16, 64 * 8 };
		if (rbx_gfx_decode(&s_gfx8, &l8, t1, t1s / 2) || rbx_gfx_decode(&s_gfx16a, &la, t1, t1s / 2) ||
		    rbx_gfx_decode(&s_gfx16b, &lb, t2, t2s / 2) || rbx_gfx_decode(&s_gfxspr1, &ls1, spr1, s1s / 2) ||
		    rbx_gfx_decode(&s_gfxspr2, &ls2, spr2, s2s / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			dz_exit();
			return -1;
		}
	}
	memset(s_tile, 0, sizeof(s_tile));
	for (int c = 0; c < 2; c++) {
		for (int i = 0; i < 2; i++) {
			s_tile[c].tm[i].size = DECO_64x32;
			s_tile[c].tm[i].col_bank = i ? 16 : 0;
			s_tile[c].tm[i].col_mask = 0x0f;
			s_tile[c].tm[i].bank_cb = bank_cb;
		}
		s_tile[c].gfx8 = &s_gfx8;
	}
	s_tile[0].gfx16 = &s_gfx16a;
	s_tile[1].gfx16 = &s_gfx16b;
	s_tile[1].base16 = 512;   // (gfx_dassault: tiles 2 from 512)
	for (int c = 0; c < 2; c++) {   // (alloc_sprite_bitmap: raw shift 4)
		memset(&s_spr[c], 0, sizeof(s_spr[c]));
		s_spr[c].gfx = c ? &s_gfxspr2 : &s_gfxspr1;
		s_spr[c].raw = 1;
		s_spr[c].raw_shift = 4;
	}
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_snap || m68k_multi_init(2)) {
		rbx_fail("%s: out of memory", g->set);
		dz_exit();
		return -1;
	}
	// sound: HuC6280 4.03 MHz, YM2203 4.03 MHz 0.40, YM2151 3.58 MHz 0.45 (banking the second OKI), OKI 1 MHz 0.50,
	// OKI 2 MHz 0.25, stereo
	deco16snd_cfg sc;
	memset(&sc, 0, sizeof(sc));
	sc.cpu_clock = 32220000 / 8;
	sc.ym_clock = 32220000 / 9;
	sc.ym_gain[0] = sc.ym_gain[1] = 115;
	sc.ym2203_clock = 32220000 / 8;
	sc.ym2203_gain[0] = sc.ym2203_gain[1] = sc.ym2203_gain[2] = sc.ym2203_gain[3] = 102;
	sc.oki_rom[0] = oki1;
	sc.oki_size[0] = o1s;
	sc.oki_clock[0] = 32220000 / 32;
	sc.oki_gain[0] = 128;
	sc.oki_rom[1] = oki2;
	sc.oki_size[1] = o2s;
	sc.oki_clock[1] = 32220000 / 16;
	sc.oki_gain[1] = 64;
	sc.stereo = 1;
	sc.ym_port = ym_port;
	if (deco16snd_init(snd, snd_size, &sc, FRAME_MHZ, LINES) || rbx_video_setup(VIS_W, VIS_H, 0x800)) {
		rbx_fail("%s: out of memory", g->set);
		dz_exit();
		return -1;
	}
	static const char *const ports[5] = { "P1_P2", "P3_P4", "DSW1", "DSW2", "SYSTEM" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(ports[i]);
	s_port_vbl = rbx_port_find("VBLANK1");
	for (int c = 0; c < 2; c++)
		m68k_bus_swap16(s_rom[c], s_rom_size[c]);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	for (int c = 0; c < 2; c++) {
		m68k_bus_reset_map();
		m68k_bus_map(0, 0x7ffff, s_rom[c], s_rom_size[c], 1, 0);
		m68k_bus_map(0x3f8000, 0x3fbfff, s_ram[c], sizeof(s_ram[c]), 1, 1);
		m68k_bus_map(0x3fc000, 0x3fcfff, s_spriteram[c ? 0 : 1], sizeof(s_spriteram[0]), 1, 1);   // (main: spriteram2)
		if (!c) {
			m68k_bus_map(0x100000, 0x103fff, s_palram, sizeof(s_palram), 1, 1);
			m68k_bus_map(0x200000, 0x201fff, s_tile[0].vram[0], sizeof(s_tile[0].vram[0]), 1, 1);
			m68k_bus_map(0x202000, 0x203fff, s_tile[0].vram[1], sizeof(s_tile[0].vram[1]), 1, 1);
			m68k_bus_map(0x212000, 0x212fff, s_rowscroll[0], sizeof(s_rowscroll[0]), 0, 1);
			m68k_bus_map(0x240000, 0x240fff, s_tile[1].vram[0], 0x1000, 1, 1);
			m68k_bus_map(0x242000, 0x242fff, s_tile[1].vram[1], 0x1000, 1, 1);
			m68k_bus_map(0x252000, 0x252fff, s_rowscroll[1], sizeof(s_rowscroll[1]), 0, 1);
		}
		m68k_bus.rd[(c ? 0x3fa000 : 0x3f8000) >> M68K_PAGE_BITS] = NULL;   // (reads of the polled page: the handler)
		m68k_bus.read8 = c ? sub_read8 : main_read8;
		m68k_bus.read16 = c ? sub_read16 : main_read16;
		m68k_bus.write8 = c ? sub_write8 : main_write8;
		m68k_bus.write16 = c ? sub_write16 : main_write16;
		m68k_multi_capture(c);
	}
	m68k_multi_finish();
	m68k_bus_set_line(7, 0);   // (the bus's interrupt hook in place for the lines set from the other CPU)
	dz_reset();
	return 0;
}

static void dz_exit(void)
{
	deco16snd_exit();
	m68k_multi_exit();
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16a);
	rbx_gfx_free(&s_gfx16b);
	rbx_gfx_free(&s_gfxspr1);
	rbx_gfx_free(&s_gfxspr2);
	free(s_snap);
	s_snap = NULL;
}

static void dz_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // irq4_line_assert / irq5_line_assert at vblank
			vblank_capture();
			m68k_multi_set_line(0, 4, 1);
			m68k_multi_set_line(1, 5, 1);
		}
		if (line >= VIS_Y0 && line < VIS_Y0 + VIS_H)
			for (int i = 0; i < 2; i++)
				deco16ic_latch_line(&s_tile[i], line);
		int cyc[2];
		for (int c = 0; c < 2; c++)
			cyc[c] = rbx_line_cycles(&s_cpu_frac[c], CPU_CLOCK, FRAME_MHZ, LINES);
		for (int k = 0; k < SLICES; k++)
			for (int c = 0; c < 2; c++) {
				const int n = cyc[c] * (k + 1) / SLICES - cyc[c] * k / SLICES;
				m68k_multi_switch(c);
				s_slice++;
				m68k_execute(n);
			}
		m68k_multi_switch(0);
		deco16snd_line();
	}
	deco16snd_frame_end();
	s_frame++;
}

static int dz_state(void *ctx, int load)
{
	uint64_t fr[2] = { s_cpu_frac[0], s_cpu_frac[1] };
	uint32_t misc[4] = { s_frame, s_priority, m68k_multi_asserted(0), m68k_multi_asserted(1) };
	int rc = 0;
	rc |= m68k_multi_state(ctx, load);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "shared", s_shared, sizeof(s_shared));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "sprb", s_spriteram_buf, sizeof(s_spriteram_buf));
	rc |= rbx_state_io(ctx, "palram", s_palram, sizeof(s_palram));
	rc |= rbx_state_io(ctx, "rows", s_rowscroll, sizeof(s_rowscroll));
	for (int i = 0; i < 2; i++) {
		rc |= rbx_state_io(ctx, i ? "vram2" : "vram1", s_tile[i].vram, sizeof(s_tile[i].vram));
		rc |= rbx_state_io(ctx, i ? "tctl2" : "tctl1", s_tile[i].control, sizeof(s_tile[i].control));
	}
	rc |= deco16snd_state(ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		s_cpu_frac[0] = fr[0];
		s_cpu_frac[1] = fr[1];
		s_frame = misc[0];
		s_priority = (uint16_t)misc[1];
		m68k_multi_set_asserted(0, misc[2]);
		m68k_multi_set_asserted(1, misc[3]);
		for (int c = 0; c < 2; c++)
			for (int i = 0; i < 8; i++)
				deco16ic_control_w(&s_tile[c], i, s_tile[c].control[i], 0xffff);
	}
	return rc ? -1 : 0;
}

static int dz_save(void *ctx) { return dz_state(ctx, 0); }
static int dz_load(void *ctx) { return dz_state(ctx, 1); }
static int dz_supported(const char *machine) { return !strcmp(machine, "dassault"); }

const rbx_driver rbx_drv_dassault = { "DECO 16 Thunder Zone", dz_init, dz_exit, dz_reset, dz_frame, dz_save, dz_load,
                                      dz_supported, dz_render, dz_help };
