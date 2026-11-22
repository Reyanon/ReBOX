/*  Data East DECO 32 (DE-0351): Captain America and The Avengers. Ported from MAME src/mame/dataeast/captaven.cpp
    (BSD-3-Clause, copyright-holders: Bryan McPhail; split from deco32.cpp).
    ARM (Data East 101) at 7 MHz; two tilemap chips 56 (the second's first playfield 8 bpp), sprite chip 52
    (alternate list format, buffered), 146 protection / I/O, the IRQ controller (raster 2 and vblank to the ARM);
    sound: HuC6280 at 2.685 MHz with a YM2151 (its CT port banks the OKIs) and two OKI M6295.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/arm/arm.h"
#include "../../cpu/h6280/h6280.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../machine/decocrpt.h"
#include "../machine/deco146.h"
#include "../machine/deco_irq.h"
#include "../video/deco16ic.h"
#include "../video/decospr.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 7000000        // 28 MHz / 4
#define SND_CLOCK 2685000        // 32.22 MHz / 4 / 3
#define YM_CLOCK 3580000         // 32.22 MHz / 9
#define LINES 274                // set_raw(28 MHz / 4, 442, 0, 320, 274, 8, 248)
#define FRAME_MHZ 57800          // 7 MHz / (442 x 274)
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 320

static arm_t s_cpu;
static h6280_t s_snd;
static uint8_t *s_rom, *s_snd_rom;
static uint32_t s_rom_size, s_snd_rom_size;
static uint8_t s_ram[0x8000], s_ram2[0x8000];   // 0x120000 main, 0x160000 extra (32-bit)
static uint8_t s_snd_ram[0x2000];
static uint32_t s_pal[0x800];                    // 0x130000 (xBGR_888)
static uint16_t s_spriteram[0x800], s_spriteram_buf[0x800];
static uint32_t (*s_rowscroll32_hp)[4][0x1000];
#define s_rowscroll32 (*s_rowscroll32_hp)
static uint16_t (*s_rowscroll_hp)[4][0x1000];
#define s_rowscroll (*s_rowscroll_hp)
static uint32_t s_pri;
static deco16ic_t (*s_tile_hp)[2];
#define s_tile (*s_tile_hp)
static decospr_t s_spr;
static deco146_t s_prot;
static deco_irq_t s_irq;
static int s_irq_raster2, s_irq_vblank;
static rbx_gfx s_gfx8, s_gfx16, s_gfx16_8bpp, s_gfxspr;
static okim6295_t s_oki[2];
static int s_port_dsw[3], s_port_in0, s_port_in1, s_port_sys;
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac, s_snd_frac, s_ym_frac;
// sound: the YM2151's rate out (stereo), the OKIs' samples held between their own
static int32_t *s_mix_l, *s_mix_r;
static int s_mix_cap, s_mix_n;
static uint32_t s_out_rate, s_oki_phase[2];
static int32_t s_oki_last[2];
static uint8_t (*s_pri_buf_hp)[VIS_W * VIS_H];
#define s_pri_buf (*s_pri_buf_hp)
static struct {
	deco16ic_t tile[2];
	uint16_t rowscroll[4][0x800], spriteram[0x800];
	uint32_t pal[0x800], pri, frame;
} *s_snap;

static uint32_t rd32le(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }
static void wr32m(uint8_t *p, uint32_t data, uint32_t mask)
{
	for (int i = 0; i < 4; i++)
		if (mask & (0xffu << (8 * i)))
			p[i] = (uint8_t)(data >> (8 * i));
}

// ---- interrupts ----
static void arm_irq_update(void) { arm_set_irq(&s_cpu, s_irq_raster2 || s_irq_vblank); }   // (input merger, any high)
static void irq_raster2(void *u, int state) { (void)u; s_irq_raster2 = state; arm_irq_update(); }
static void irq_vblank(void *u, int state) { (void)u; s_irq_vblank = state; arm_irq_update(); }
static void latch_irq(void *u, int state) { (void)u; h6280_set_irq(&s_snd, H6280_IRQ1, state); }
static void ym_irq(int state) { h6280_set_irq(&s_snd, H6280_IRQ2, state); }

// ---- 146 ports ----
static uint16_t port_a(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_in0); }
static uint16_t port_b(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_sys); }
static uint16_t port_c(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_in1); }

// the 146's address: (real address bits 14-17 << 11) | bits 0-10 (NC 31-18, 13-11)
static uint16_t prot_addr(uint32_t o)
{
	const uint32_t real = (o >> 2) * 2;
	return (uint16_t)((((real >> 14) & 0xf) << 11) | (real & 0x7ff));
}

static inline void combine16(uint16_t *p, uint32_t data, uint32_t mask)
{
	mask &= 0xffff;
	*p = (uint16_t)((*p & ~mask) | (data & mask));
}

static uint32_t tile_r(deco16ic_t *t, uint32_t o)
{
	if (o < 0x20) return deco16ic_control_r(t, (int)(o >> 2)) | 0xffff0000u;
	if (o >= 0x10000 && o < 0x14000) return t->vram[0][((o - 0x10000) >> 2) & 0x7ff] | 0xffff0000u;   // (mirror 0x2000)
	if (o >= 0x14000 && o < 0x16000) return t->vram[1][(o - 0x14000) >> 2] | 0xffff0000u;
	return 0;
}
static void tile_w(deco16ic_t *t, uint32_t o, uint32_t data, uint32_t mask)
{
	if (o < 0x20) {
		if (mask & 0xffff)
			deco16ic_control_w(t, (int)(o >> 2), (uint16_t)data, (uint16_t)mask);
	} else if (o >= 0x10000 && o < 0x14000) combine16(&t->vram[0][((o - 0x10000) >> 2) & 0x7ff], data, mask);
	else if (o >= 0x14000 && o < 0x16000) combine16(&t->vram[1][(o - 0x14000) >> 2], data, mask);
}

static int rowscroll_index(uint32_t a, int *idx)
{
	static const uint32_t base[4] = { 0x1a0000, 0x1a4000, 0x1e0000, 0x1e4000 };
	static const uint32_t size[4] = { 0x4000, 0x2000, 0x4000, 0x2000 };
	for (int i = 0; i < 4; i++)
		if (a >= base[i] && a < base[i] + size[i]) {
			*idx = (int)((a - base[i]) >> 2);
			return i;
		}
	return -1;
}

// The main loop's wait for the interrupt (pc 0x39d0 .. 0x39f0): while the flag byte at 0x12748c is not 0 it sums the
// work RAM into r8 (r4 += 12; r9 = [sl], sl += 4; r8 += r9 + r7; sl past r5: back to r6). The flag only changes in
// the interrupt, which comes at a line's start, so the iterations up to the line's end are done here at once with the
// same registers, flags and cycles (the read at 0x39e8 again at the same interval: one iteration's time)
#define IDLE_FLAG 0x12748c
static uint32_t s_idle_val;
static int64_t s_idle_cyc, s_idle_period, s_slice_end;
static int s_idle_off;   // (tests: 1 = the loop run as it is)
static void idle_loop(uint32_t v)
{
	const int64_t now = s_cpu.cycles, period = now - s_idle_cyc;
	if (!s_idle_off && v == s_idle_val && period > 0 && period == s_idle_period && arm_pc(&s_cpu) == 0x39e8 && v != 0) {
		const int64_t left = s_slice_end - now;
		if (left > period) {
			int64_t k = (left - 1) / period;
			uint32_t r4 = s_cpu.r[4], r8 = s_cpu.r[8], sl = s_cpu.r[10], cmp = 0;
			const uint32_t r5 = s_cpu.r[5], r6 = s_cpu.r[6], r7 = s_cpu.r[7];
			for (int64_t i = 0; i < k; i++) {
				r4 += 12;
				const uint32_t w = rd32le(s_ram + (sl & 0x7ffc));   // (sl stays in 0x120000 .. r5: the work RAM)
				sl += 4;
				r8 += w + r7;
				cmp = sl;
				if ((int32_t)sl >= (int32_t)r5)
					sl = r6;
			}
			s_cpu.r[4] = r4;
			s_cpu.r[8] = r8;
			s_cpu.r[10] = sl;
			// the flags of the last "cmp sl, r5" (NZCV in R15)
			const uint32_t d = cmp - r5;
			const uint32_t nzcv = (d & 0x80000000u) | (d == 0 ? 0x40000000u : 0) | (cmp >= r5 ? 0x20000000u : 0) |
			                      ((((cmp ^ r5) & (cmp ^ d)) >> 31) << 28);
			s_cpu.r[15] = (s_cpu.r[15] & 0x0fffffffu) | nzcv;
			s_cpu.cycles += k * period;
		}
	}
	s_idle_period = v == s_idle_val ? period : 0;
	s_idle_val = v;
	s_idle_cyc = s_cpu.cycles;
}

static uint32_t bus_r32(void *ctx, uint32_t a)
{
	(void)ctx;
	a &= 0x03fffffc;
	if (a < 0x100000) return rd32le(s_rom + (a & (s_rom_size - 1)));
	if (a < 0x100008) return 0xffffffffu;   // _71_r
	if (a >= 0x110000 && a < 0x112000) return s_spriteram[(a - 0x110000) >> 2] ^ 0xffff0000u;
	if (a >= 0x120000 && a < 0x128000) {
		const uint32_t v = rd32le(s_ram + (a & 0x7ffc));
		if (a == (IDLE_FLAG & ~3u))
			idle_loop((v >> (8 * (IDLE_FLAG & 3))) & 0xff);
		return v;
	}
	if (a >= 0x128000 && a < 0x130000) return deco146_read(&s_prot, prot_addr(a - 0x128000), NULL) | 0xffff0000u;
	if (a >= 0x130000 && a < 0x132000) return s_pal[(a - 0x130000) >> 2];
	if (a >= 0x148000 && a < 0x148010) {
		const int vbl = s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H;
		return 0xffffff00u | deco_irq_r(&s_irq, (int)((a - 0x148000) >> 2), vbl, 0);
	}
	if (a >= 0x160000 && a < 0x168000) return rd32le(s_ram2 + (a & 0x7ffc));
	if (a == 0x168000)
		return (rbx_port_read(s_port_dsw[0]) & 0xff) | (rbx_port_read(s_port_dsw[1]) & 0xff) << 8 |
		       (rbx_port_read(s_port_dsw[2]) & 0xff) << 16 | (uint32_t)(0x7f | (s_prot.soundlatch_pending ? 0 : 0x80)) << 24;
	if (a >= 0x180000 && a < 0x1a0000) return tile_r(&s_tile[0], a - 0x180000);
	if (a >= 0x1c0000 && a < 0x1e0000) return tile_r(&s_tile[1], a - 0x1c0000);
	int idx;
	const int rs = rowscroll_index(a, &idx);
	if (rs >= 0)
		return s_rowscroll32[rs][idx];
	return 0;
}

static void bus_w32m(uint32_t a, uint32_t data, uint32_t mask)
{
	a &= 0x03fffffc;
	if (a < 0x100004 && a >= 0x100000) {   // buffer_spriteram_w
		memcpy(s_spriteram_buf, s_spriteram, sizeof(s_spriteram));
		return;
	}
	if (a >= 0x110000 && a < 0x112000) combine16(&s_spriteram[(a - 0x110000) >> 2], data, mask);
	else if (a >= 0x120000 && a < 0x128000) wr32m(s_ram + (a & 0x7ffc), data, mask);
	else if (a >= 0x128000 && a < 0x130000) {
		if (mask & 0xffff)
			deco146_write(&s_prot, prot_addr(a - 0x128000), (uint16_t)data, (uint16_t)mask, NULL);
	} else if (a >= 0x130000 && a < 0x132000) {
		uint32_t *p = &s_pal[(a - 0x130000) >> 2];
		*p = (*p & ~mask) | (data & mask);
	} else if (a >= 0x148000 && a < 0x148010) {
		if (mask & 0xff)
			deco_irq_w(&s_irq, (int)((a - 0x148000) >> 2), (uint8_t)data);
	} else if (a >= 0x160000 && a < 0x168000) wr32m(s_ram2 + (a & 0x7ffc), data, mask);
	else if (a == 0x178000) s_pri = (s_pri & ~mask) | (data & mask);
	else if (a >= 0x180000 && a < 0x1a0000) tile_w(&s_tile[0], a - 0x180000, data, mask);
	else if (a >= 0x1c0000 && a < 0x1e0000) tile_w(&s_tile[1], a - 0x1c0000, data, mask);
	else {
		int idx;
		const int rs = rowscroll_index(a, &idx);
		if (rs >= 0) {   // rowscroll_w: the 32-bit RAM and its low halves for the tilemaps
			s_rowscroll32[rs][idx] = (s_rowscroll32[rs][idx] & ~mask) | (data & mask);
			combine16(&s_rowscroll[rs][idx], data, mask);
		}
	}
}

static void bus_w32(void *ctx, uint32_t a, uint32_t data) { (void)ctx; bus_w32m(a, data, 0xffffffffu); }
static uint8_t bus_r8(void *ctx, uint32_t a) { return (uint8_t)(bus_r32(ctx, a) >> (8 * (a & 3))); }
static void bus_w8(void *ctx, uint32_t a, uint8_t v)
{
	(void)ctx;
	const int sh = 8 * (int)(a & 3);
	bus_w32m(a, (uint32_t)v << sh, 0xffu << sh);
}

// ---- sound CPU (21-bit physical addresses) ----
static uint8_t snd_r(void *ctx, uint32_t pa)
{
	(void)ctx;
	if (pa < 0x10000) return s_snd_rom[pa & (s_snd_rom_size - 1)];
	if ((pa & ~1u) == 0x110000) return rbx_ym_read(0, pa & 1);   // (the YM runs line by line)
	if ((pa & ~1u) == 0x120000) return oki_read(&s_oki[0]);
	if ((pa & ~1u) == 0x130000) return oki_read(&s_oki[1]);
	if (pa == 0x140000) return deco146_soundlatch_r(&s_prot);
	if (pa >= 0x1f0000 && pa < 0x1f2000) return s_snd_ram[pa & 0x1fff];
	return 0;
}
static void snd_w(void *ctx, uint32_t pa, uint8_t v)
{
	(void)ctx;
	if ((pa & ~1u) == 0x110000) rbx_ym_write(0, pa & 1, v);
	else if ((pa & ~1u) == 0x120000) oki_write(&s_oki[0], v);
	else if ((pa & ~1u) == 0x130000) oki_write(&s_oki[1], v);
	else if (pa >= 0x1f0000 && pa < 0x1f2000) s_snd_ram[pa & 0x1fff] = v;
}
static void ym_port(uint8_t data)   // sound_bankswitch_w
{
	s_oki[0].bank = (data & 1) * 0x40000u;
	s_oki[1].bank = ((data >> 1) & 1) * 0x40000u;
}

static uint32_t s_ym_line_clocks;
static void mix_ym_line(void)
{
	const int n = rbx_ym_advance_lr(0, s_ym_line_clocks, s_mix_l + s_mix_n, s_mix_r + s_mix_n, s_mix_cap - s_mix_n);
	for (int i = 0; i < n; i++) {   // the OKIs (mono to both sides) at their own rates, held
		int32_t o = 0;
		for (int k = 0; k < 2; k++) {
			s_oki_phase[k] += s_oki[k].rate;
			while (s_oki_phase[k] >= s_out_rate) {
				s_oki_phase[k] -= s_out_rate;
				s_oki_last[k] = oki_sample(&s_oki[k]);
			}
			o += s_oki_last[k];
		}
		s_mix_l[s_mix_n + i] += o;
		s_mix_r[s_mix_n + i] += o;
	}
	s_mix_n += n;
}

// ---- video ----
static uint32_t pri_cb(uint16_t pri, int extpri)
{
	(void)extpri;
	switch (pri & 0x60) {
	case 0x00: return 0;        // above everything
	case 0x20: return 0xfff0;   // above the 2nd playfield
	case 0x40: return 0xfffc;   // above the 1st playfield
	default: return 0xfffe;     // under everything
	}
}

static void tile_cb(uint32_t *tile, uint32_t *colour, int layer, int is8x8)
{
	(void)is8x8;
	if (layer == 0) {
		*tile |= (*colour & 3) << 12;
		*colour >>= 2;
	}
}

static int bank_cb(int bank) { return (bank & 0x20) << 9; }

static void cv_render(void)
{
	const uint32_t *p = s_snap->pal;
	for (int i = 0; i < 0x800; i++)   // xBGR_888
		rbx_video.palette[i] = (p[i] & 0xff) << 16 | (p[i] & 0xff00) | ((p[i] >> 16) & 0xff);
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = 0; y < VIS_H; y++)
		memset(bm + y * pitch, 0, VIS_W * 2);   // (palette index 0)
	memset(s_pri_buf, 0, sizeof(s_pri_buf));
	deco16ic_t *t0 = &s_snap->tile[0], *t1 = &s_snap->tile[1];
	if ((s_snap->pri & 1) == 0) {
		deco16ic_draw(t1, 0, s_snap->rowscroll[2], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 1, 0xff);
		deco16ic_draw(t0, 1, s_snap->rowscroll[1], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 2, 0xff);
	} else {
		deco16ic_draw(t0, 1, s_snap->rowscroll[1], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 1, 0xff);
		deco16ic_draw(t1, 0, s_snap->rowscroll[2], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 2, 0xff);
	}
	deco16ic_draw(t0, 0, s_snap->rowscroll[0], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 4, 0xff);
	decospr_draw(&s_spr, s_snap->spriteram, 0x400, bm, s_pri_buf, pitch, VIS_W, VIS_H, VIS_Y0, 1, s_snap->frame);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->tile, s_tile, sizeof(s_tile));
	for (int i = 0; i < 4; i++)
		memcpy(s_snap->rowscroll[i], s_rowscroll[i], sizeof(s_snap->rowscroll[i]));
	memcpy(s_snap->spriteram, s_spriteram_buf, sizeof(s_spriteram_buf));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	s_snap->pri = s_pri;
	s_snap->frame = s_frame;
	s_spr.flip_screen = (s_tile[0].control[0] >> 7) & 1;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void cv_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_ram2, 0, sizeof(s_ram2));
	memset(s_snd_ram, 0, sizeof(s_snd_ram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_spriteram_buf, 0, sizeof(s_spriteram_buf));
	memset(s_rowscroll32, 0, sizeof(s_rowscroll32));
	memset(s_rowscroll, 0, sizeof(s_rowscroll));
	s_pri = 0;
	for (int i = 0; i < 2; i++) {
		memset(s_tile[i].vram, 0, sizeof(s_tile[i].vram));
		deco16ic_reset(&s_tile[i]);
		oki_reset(&s_oki[i]);
		s_oki[i].bank = 0;
		s_oki_phase[i] = 0;
		s_oki_last[i] = 0;
	}
	deco146_reset(&s_prot);
	deco_irq_reset(&s_irq);
	s_irq_raster2 = s_irq_vblank = 0;
	rbx_ym_reset(0);
	arm_reset(&s_cpu);
	h6280_reset(&s_snd);
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = s_snd_frac = s_ym_frac = 0;
	s_mix_n = 0;
	s_idle_val = 0;
	s_idle_cyc = s_idle_period = 0;
}

static void cv_exit(void);

static int cv_init(const rbx_game *g)
{
	if (!s_pri_buf_hp && !(s_pri_buf_hp = calloc(1, sizeof(*s_pri_buf_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_rowscroll_hp && !(s_rowscroll_hp = calloc(1, sizeof(*s_rowscroll_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_rowscroll32_hp && !(s_rowscroll32_hp = calloc(1, sizeof(*s_rowscroll32_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_tile_hp && !(s_tile_hp = calloc(1, sizeof(*s_tile_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t t1s, t2s, ss, o1s, o2s;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_snd_rom = rbx_region("audiocpu", &s_snd_rom_size);
	uint8_t *t1 = rbx_region("tiles1", &t1s), *t2 = rbx_region("tiles2", &t2s), *spr = rbx_region("sprites", &ss);
	uint8_t *oki1 = rbx_region("oki1", &o1s), *oki2 = rbx_region("oki2", &o2s);
	if (!s_rom || !s_snd_rom || !t1 || !t2 || !spr || !oki1 || !oki2) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (deco56_decrypt_gfx(t1, t1s) || deco56_decrypt_gfx(t2, t2s)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	{	// charlayout / tilelayout (RGN_FRAC(1,2)), tilelayout_8bpp (RGN_FRAC(1,4), 8 planes)
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int k = 0; k < 8; k++) {
			x8[k] = (uint32_t)k;
			y8[k] = (uint32_t)(k * 16);
			x16[k] = (uint32_t)(256 + k);
			x16[8 + k] = (uint32_t)k;
		}
		for (int k = 0; k < 16; k++)
			y16[k] = (uint32_t)(k * 16);
		const uint32_t h1 = t1s * 4, hs = ss * 4, q2 = t2s * 2;   // (half / quarter of a region, in bits)
		const uint32_t p1[4] = { h1 + 8, h1, 8, 0 }, ps[4] = { hs + 8, hs, 8, 0 };
		const uint32_t p8[8] = { 3 * q2 + 8, 3 * q2, 2 * q2 + 8, 2 * q2, q2 + 8, q2, 8, 0 };
		const rbx_gfx_layout l8 = { 8, 8, 4, p1, x8, y8, 16 * 8 }, l16 = { 16, 16, 4, p1, x16, y16, 64 * 8 };
		const rbx_gfx_layout l16b = { 16, 16, 8, p8, x16, y16, 64 * 8 }, lsp = { 16, 16, 4, ps, x16, y16, 64 * 8 };
		if (rbx_gfx_decode(&s_gfx8, &l8, t1, t1s / 2) || rbx_gfx_decode(&s_gfx16, &l16, t1, t1s / 2) ||
		    rbx_gfx_decode(&s_gfx16_8bpp, &l16b, t2, t2s / 4) || rbx_gfx_decode(&s_gfxspr, &lsp, spr, ss / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			cv_exit();
			return -1;
		}
	}
	memset(s_tile, 0, sizeof(s_tile));
	static const int cbank[2][2] = { { 0x20, 0x30 }, { 0x04, 0x00 } }, cmask[2][2] = { { 0x0f, 0x0f }, { 0x03, 0x00 } };
	for (int c = 0; c < 2; c++)
		for (int i = 0; i < 2; i++) {
			s_tile[c].tm[i].size = c ? DECO_32x32 : DECO_64x32;
			s_tile[c].tm[i].col_bank = cbank[c][i];
			s_tile[c].tm[i].col_mask = cmask[c][i];
		}
	s_tile[0].gfx8 = s_tile[1].gfx8 = &s_gfx8;   // (8x8: entry 0)
	s_tile[0].gfx16 = &s_gfx16;                  // 16x16: entry 1
	s_tile[1].gfx16 = &s_gfx16_8bpp;             // entry 2 (8 bpp)
	s_tile[1].gran16 = 8;
	s_tile[1].tile_cb = tile_cb;
	s_tile[1].tm[0].bank_cb = bank_cb;
	memset(&s_spr, 0, sizeof(s_spr));
	s_spr.gfx = &s_gfxspr;
	s_spr.color_base = 0;
	s_spr.alt_format = 1;
	s_spr.pri_cb = pri_cb;
	deco146_init(&s_prot, 0);
	s_prot.port_a = port_a;
	s_prot.port_b = port_b;
	s_prot.port_c = port_c;
	s_prot.soundlatch_irq = latch_irq;
	memset(&s_irq, 0, sizeof(s_irq));
	s_irq.vis_top = VIS_Y0;
	s_irq.vis_bottom = VIS_Y0 + VIS_H - 1;
	s_irq.raster2 = irq_raster2;
	s_irq.vblank = irq_vblank;
	s_snap = calloc(1, sizeof(*s_snap));
	// sound: YM2151 0.42 / 0.42, OKI 1 MHz 1.0, OKI 2 MHz 0.35 (both sides)
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 108;
	yc.irq = ym_irq;
	yc.port_a_write = ym_port;
	oki_init(&s_oki[0], 32220000 / 32, 1, oki1, o1s, 256);
	oki_init(&s_oki[1], 32220000 / 16, 1, oki2, o2s, 90);
	// (the YM2151's samples made on the worker: rbx_ym_async_take_lr at the frame's end)
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 2)) {
		rbx_fail("%s: out of memory", g->set);
		cv_exit();
		return -1;
	}
	s_out_rate = rbx_ym_rate(0);
	s_mix_cap = (int)(s_out_rate / 50) + 256;
	s_mix_l = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_mix_r = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix_l || !s_mix_r || rbx_audio_setup_stereo(s_out_rate) || rbx_video_setup(VIS_W, VIS_H, 0x800)) {
		cv_exit();
		return -1;
	}
	static const char *const dsw[3] = { "DSW1", "DSW2", "DSW3" };
	for (int i = 0; i < 3; i++)
		s_port_dsw[i] = rbx_port_find(dsw[i]);
	s_port_in0 = rbx_port_find("IN0");
	s_port_in1 = rbx_port_find("IN1");
	s_port_sys = rbx_port_find("SYSTEM");
	arm_init(&s_cpu);
	for (uint32_t a = 0; a < 0x100000 && a < s_rom_size; a += 0x1000)
		s_cpu.rpage[a >> 12] = s_rom + a;
	for (uint32_t a = 0; a < 0x8000; a += 0x1000) {
		s_cpu.rpage[(0x120000 + a) >> 12] = s_cpu.wpage[(0x120000 + a) >> 12] = s_ram + a;
		s_cpu.rpage[(0x160000 + a) >> 12] = s_cpu.wpage[(0x160000 + a) >> 12] = s_ram2 + a;
	}
	s_cpu.rpage[IDLE_FLAG >> 12] = NULL;   // (the page with the idle loop's flag: bus_r32)
	s_idle_off = getenv("RBX_NOIDLE") != NULL;
	s_cpu.read32 = bus_r32;
	s_cpu.write32 = bus_w32;
	s_cpu.read8 = bus_r8;
	s_cpu.write8 = bus_w8;
	h6280_init(&s_snd);
	for (uint32_t a = 0; a < 0x10000 && a < s_snd_rom_size; a += 0x2000)
		s_snd.rpage[a >> 13] = s_snd_rom + a;
	s_snd.rpage[0x1f0000 >> 13] = s_snd.wpage[0x1f0000 >> 13] = s_snd_ram;
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	cv_reset();
	return 0;
}

static void cv_exit(void)
{
	rbx_ym_exit(0);
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16);
	rbx_gfx_free(&s_gfx16_8bpp);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
	free(s_mix_l);
	free(s_mix_r);
	s_mix_l = s_mix_r = NULL;
	free(s_pri_buf_hp);
	s_pri_buf_hp = NULL;
	free(s_rowscroll_hp);
	s_rowscroll_hp = NULL;
	free(s_rowscroll32_hp);
	s_rowscroll32_hp = NULL;
	free(s_tile_hp);
	s_tile_hp = NULL;
}

static void cv_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		deco_irq_line(&s_irq, line);
		if (line >= VIS_Y0 && line < VIS_Y0 + VIS_H)
			for (int i = 0; i < 2; i++)
				deco16ic_latch_line(&s_tile[i], line);
		s_slice_end = s_cpu.cycles + rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		arm_run(&s_cpu, s_slice_end);
		arm_break = 0;
		h6280_run_until(&s_snd, s_snd.cycles + rbx_line_cycles(&s_snd_frac, SND_CLOCK, FRAME_MHZ, LINES));
		h6280_break = 0;
		s_ym_line_clocks = (uint32_t)rbx_line_cycles(&s_ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
		mix_ym_line();
	}
	rbx_ym_async_take_lr(0, s_mix_l, s_mix_r, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put_stereo(s_mix_l[i], s_mix_r[i]);
	memset(s_mix_l, 0, sizeof(int32_t) * (size_t)s_mix_n);
	memset(s_mix_r, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	s_frame++;
}

static int cv_state(void *ctx, int load)
{
	uint8_t cpu[ARM_STATE_SIZE];
	uint8_t snd[H6280_STATE_SIZE];
	if (!load) {
		memcpy(cpu, &s_cpu, ARM_STATE_SIZE);
		memcpy(snd, &s_snd, H6280_STATE_SIZE);
	}
	uint64_t fr[3] = { s_cpu_frac, s_snd_frac, s_ym_frac };
	int32_t misc[10] = { (int32_t)s_frame, (int32_t)s_pri, s_irq_raster2, s_irq_vblank, (int32_t)s_oki[0].bank,
	                     (int32_t)s_oki[1].bank, (int32_t)s_oki_phase[0], (int32_t)s_oki_phase[1], s_oki_last[0], s_oki_last[1] };
	int rc = 0;
	rc |= rbx_state_io(ctx, "arm", cpu, ARM_STATE_SIZE);
	rc |= rbx_state_io(ctx, "h6280", snd, H6280_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "ram2", s_ram2, sizeof(s_ram2));
	rc |= rbx_state_io(ctx, "sram", s_snd_ram, sizeof(s_snd_ram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "sprb", s_spriteram_buf, sizeof(s_spriteram_buf));
	rc |= rbx_state_io(ctx, "rows32", s_rowscroll32, sizeof(s_rowscroll32));
	rc |= rbx_state_io(ctx, "rows", s_rowscroll, sizeof(s_rowscroll));
	for (int i = 0; i < 2; i++) {
		rc |= rbx_state_io(ctx, i ? "vram2" : "vram1", s_tile[i].vram, sizeof(s_tile[i].vram));
		rc |= rbx_state_io(ctx, i ? "tctl2" : "tctl1", s_tile[i].control, sizeof(s_tile[i].control));
	}
	rc |= rbx_state_io(ctx, "prot", &s_prot, DECO146_STATE_SIZE);
	rc |= rbx_state_io(ctx, "irqc", &s_irq, DECO_IRQ_STATE_SIZE);
	rc |= rbx_state_io(ctx, "oki1", s_oki[0].v, sizeof(s_oki[0].v) + sizeof(int));
	rc |= rbx_state_io(ctx, "oki2", s_oki[1].v, sizeof(s_oki[1].v) + sizeof(int));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (rc)
		return -1;
	if (load) {
		memcpy(&s_cpu, cpu, ARM_STATE_SIZE);
		memcpy(&s_snd, snd, H6280_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_ym_frac = fr[2];
		s_frame = (uint32_t)misc[0];
		s_pri = (uint32_t)misc[1];
		s_irq_raster2 = misc[2];
		s_irq_vblank = misc[3];
		s_oki[0].bank = (uint32_t)misc[4];
		s_oki[1].bank = (uint32_t)misc[5];
		s_oki_phase[0] = (uint32_t)misc[6];
		s_oki_phase[1] = (uint32_t)misc[7];
		s_oki_last[0] = misc[8];
		s_oki_last[1] = misc[9];
		for (int c = 0; c < 2; c++)
			for (int i = 0; i < 8; i++)
				deco16ic_control_w(&s_tile[c], i, s_tile[c].control[i], 0xffff);
	}
	return 0;
}

static int cv_save(void *ctx) { return cv_state(ctx, 0); }
static int cv_load(void *ctx) { return cv_state(ctx, 1); }
static int cv_supported(const char *machine) { return !strcmp(machine, "captaven"); }

const rbx_driver rbx_drv_captaven = { "DECO 32 Captain America", cv_init, cv_exit, cv_reset, cv_frame, cv_save, cv_load,
                                      cv_supported, cv_render, NULL };
