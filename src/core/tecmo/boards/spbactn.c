/*  Tecmo Super Pinball Action. Ported from MAME src/mame/tecmo/spbactn.cpp (BSD-3-Clause, copyright-holders: David
    Haywood).
    68000 at 12 MHz (IRQ 3 at the vblank, held until acknowledged); two 1024x1024 layers of 16x8 tiles and the Ninja
    Gaiden sprites, mixed by the Tecmo mixer (RGB) on a 512-wide screen. Sound: Z80 at 4 MHz, YM3812 (its IRQ),
    OKI6295 at 1 MHz, the latch on the NMI. (The dual-screen prototype is not here.)
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/decoband.h"
#include "../../video/gfxlayout.h"
#include "../video/tecmo_mix.h"
#include "../video/tecmo_spr.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 12000000
#define Z80_CLOCK 4000000
#define YM_CLOCK 4000000
#define OKI_CLOCK 1000000
#define FRAME_MHZ 60051          // set_raw(22.656 MHz / 2, 720, 0, 512, 262, 16, 240)
#define LINES 262
#define VIS_Y0 16
#define VIS_H 224
#define VIS_W 512
#define PAL_N 0x1400

static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static uint16_t (*s_ram_hp)[0x2000];          // 0x40000 .. 0x43fff
#define s_ram (*s_ram_hp)
static uint16_t s_spram[0x800];         // 0x50000 .. 0x50fff
static uint16_t (*s_fgram_hp)[0x4000];        // 0x60000 .. 0x67fff (attributes, then codes at 0x2000)
#define s_fgram (*s_fgram_hp)
static uint16_t (*s_bgram_hp)[0x4000];        // 0x70000 .. 0x77fff
#define s_bgram (*s_bgram_hp)
static uint16_t s_pal[PAL_N];           // 0x80000 .. 0x827ff
static rbx_gfx s_gfxfg, s_gfxbg, s_gfxspr;
static int s_port[5];
static uint64_t s_cpu_frac, s_z80_frac, s_ym_frac;
static tecmo_mix_t s_mix_cfg;
static uint16_t (*s_sprbmp_hp)[VIS_W * VIS_H];
#define s_sprbmp (*s_sprbmp_hp)
static struct {
	uint16_t spr[0x800], fg[0x4000], bg[0x4000], pal[PAL_N];
} *s_snap;

// ---- sound ----
static z80 s_z80;
static uint8_t s_zram[0x800];
static uint8_t s_latch;
static int s_latch_pending;
static okim6295_t s_oki;
static int s_ym_irq;
static uint32_t s_oki_acc;
static int32_t s_oki_hold;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static const uint8_t *s_zmap[256];

static void ym_irq(int state) { s_ym_irq = state; }

static uint8_t z80_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xf000) return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0xf800) return s_zram[a - 0xf000];
	if (a == 0xf800) return oki_read(&s_oki);
	if (a == 0xfc20) {
		s_latch_pending = 0;
		return s_latch;
	}
	return 0;
}
static void z80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xf000 && a < 0xf800) s_zram[a - 0xf000] = v;
	else if (a == 0xf800) oki_write(&s_oki, v);
	else if (a == 0xf810 || a == 0xf811) rbx_ym_write(0, a & 1, v);
}
static uint8_t z80_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z80_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

static void z80_start(void)
{
	z80_init(&s_z80);
	s_z80.read_byte = z80_read;
	s_z80.write_byte = z80_write;
	s_z80.port_in = z80_in;
	s_z80.port_out = z80_out;
	s_z80.read_map = s_zmap;
}

static void sound_line(void)
{
	const int cycles = rbx_line_cycles(&s_z80_frac, Z80_CLOCK, FRAME_MHZ, LINES);
	z80_run(&s_z80, s_z80.cyc + (unsigned long)cycles, &s_ym_irq, 0);
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&s_ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
	const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	const uint32_t yr = rbx_ym_rate(0);
	for (int i = 0; i < n; i++) {
		s_oki_acc += s_oki.rate;
		while (s_oki_acc >= yr) {
			s_oki_acc -= yr;
			s_oki_hold = oki_sample(&s_oki);
		}
		s_mix[s_mix_n + i] += s_oki_hold;
	}
	s_mix_n += n;
}

static void sound_frame_end(void)
{
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

// ---- bus ----
static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x90000 && a < 0x90050 && !(a & 0xe))   // IN0, IN1, SYSTEM, DSW1, DSW2
		return (uint16_t)rbx_port_read(s_port[(a - 0x90000) >> 4]);
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
	if (a == 0x90010 && (mask & 0x00ff)) {
		s_latch = (uint8_t)v;
		if (!s_latch_pending)
			z80_gen_nmi(&s_z80);
		s_latch_pending = 1;
	} else if (a == 0x90020)
		m68k_bus_set_line(3, 0);   // main_irq_ack_w
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
// one line of a layer (64x128 of 16x8, no scroll, pen 0 transparent): fg 0 (32 colors from 0x200), bg 1 (from 0x300)
static void layer_line(int which, int y, uint16_t *dst)
{
	const uint16_t *vram = which ? s_snap->bg : s_snap->fg;
	const rbx_gfx *gfx = which ? &s_gfxbg : &s_gfxfg;
	const uint32_t ty = (uint32_t)y & 1023, row = (ty >> 3) * 64, fy = ty & 7;
	for (int col = 0; col < VIS_W / 16; col++) {
		const uint32_t i = row + (uint32_t)col;
		const uint32_t code = rbx_wrap(vram[0x2000 + i], gfx->count);
		uint16_t *d = dst + col * 16;
		if (!gfx->used[code]) {
			memset(d, 0, 32);
			continue;
		}
		const uint16_t attr = vram[i];
		uint32_t color = (attr & 0xf0u) >> 4;
		if (!which && (attr & 0x08))
			color += 0x10;
		const uint16_t c0 = (uint16_t)((which ? 0x300 : 0x200) + color * 16);
		const uint8_t *src = gfx->pix + code * 128 + fy * 16;
		for (int k = 0; k < 16; k++)
			d[k] = src[k] ? (uint16_t)(c0 + src[k]) : 0;
	}
}

#define BANDS 8
static uint32_t (*s_pal_rgb_hp)[PAL_N];
#define s_pal_rgb (*s_pal_rgb_hp)
static int s_565;

static void sb_band(int band)
{
	const int r0 = VIS_H * band / BANDS, r1 = VIS_H * (band + 1) / BANDS;
	tecmo_mix_t mix = s_mix_cfg;
	mix.rand = (uint32_t)band * 0x9e3779b9u;
	for (int y = r0; y < r1; y++) {
		uint16_t bg[VIS_W], fg[VIS_W];
		uint32_t d[VIS_W];
		layer_line(1, y + VIS_Y0, bg);
		layer_line(0, y + VIS_Y0, fg);
		tecmo_mix_line(&mix, s_pal_rgb, bg, fg, NULL, s_sprbmp + y * VIS_W, d, VIS_W);
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
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, sb_band };
static int sb_help(int helper) { return decoband_help(&s_bands, helper); }

static void sb_render(void)
{
	if (!rbx_video.out)
		return;
	for (int i = 0; i < PAL_N; i++) {   // xBGR_444
		const uint16_t p = s_snap->pal[i];
		s_pal_rgb[i] = (uint32_t)(((p & 15) * 17) << 16 | (((p >> 4) & 15) * 17) << 8 | (((p >> 8) & 15) * 17));
	}
	s_565 = rbx_video.out_bpp == 2 && rbx_video.out_color(0xff, 0, 0) == 0xf800 && rbx_video.out_color(0, 0xff, 0) == 0x07e0 &&
	        rbx_video.out_color(0, 0, 0xff) == 0x001f;
	tecmo_spr_gaiden_draw(&s_gfxspr, s_snap->spr, 0, 0, 0, VIS_W, s_sprbmp, VIS_W, VIS_H, VIS_Y0);
	decoband_run(&s_bands);
	rbx_video.out_done = 1;
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spr, s_spram, sizeof(s_spram));
	memcpy(s_snap->fg, s_fgram, sizeof(s_fgram));
	memcpy(s_snap->bg, s_bgram, sizeof(s_bgram));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	rbx_video_frame_done();
}

// ---- init / frame ----
static void sb_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_spram, 0, sizeof(s_spram));
	memset(s_fgram, 0, sizeof(s_fgram));
	memset(s_bgram, 0, sizeof(s_bgram));
	memset(s_pal, 0, sizeof(s_pal));
	s_latch = 0;
	s_latch_pending = 0;
	s_ym_irq = 0;
	memset(s_zram, 0, sizeof(s_zram));
	z80_start();
	rbx_ym_reset(0);
	oki_reset(&s_oki);
	s_oki_acc = 0;
	s_oki_hold = 0;
	s_mix_n = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_cpu_frac = s_z80_frac = s_ym_frac = 0;
}

static void sb_exit(void);

static int sb_init(const rbx_game *g)
{
	if (!s_bgram_hp && !(s_bgram_hp = calloc(1, sizeof(*s_bgram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_fgram_hp && !(s_fgram_hp = calloc(1, sizeof(*s_fgram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_pal_rgb_hp && !(s_pal_rgb_hp = calloc(1, sizeof(*s_pal_rgb_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ram_hp && !(s_ram_hp = calloc(1, sizeof(*s_ram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_sprbmp_hp && !(s_sprbmp_hp = calloc(1, sizeof(*s_sprbmp_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t fgs, bgs, sps, osize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *fgt = rbx_region("fgtiles", &fgs), *bgt = rbx_region("bgtiles", &bgs), *spr = rbx_region("sprites", &sps);
	uint8_t *oki = rbx_region("oki", &osize);
	if (!s_rom || s_rom_size < 0x40000 || !s_zrom || !fgt || !bgt || !spr || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	{	// fgtilelayout, bgtilelayout (RGN_FRAC(1,2)), gfx_8x8x4_packed_msb
		static uint32_t xf[16], yf[8], xb[16], yb[8], x8[8], y8[8];
		for (int i = 0; i < 8; i++) {
			xf[i] = (uint32_t)(i * 4);
			xf[8 + i] = (uint32_t)(4 * 8 * 8 + i * 4);
			yf[i] = (uint32_t)(i * 4 * 8);
			yb[i] = (uint32_t)(i * 16);
			x8[i] = (uint32_t)(i * 4);
			y8[i] = (uint32_t)(i * 32);
		}
		const uint32_t h = bgs * 4;   // (RGN_FRAC(1,2), in bits)
		const uint32_t xbv[16] = { h + 1 * 4, h + 0 * 4, 1 * 4, 0 * 4, h + 3 * 4, h + 2 * 4, 3 * 4, 2 * 4,
		                           16 * 8 + h + 1 * 4, 16 * 8 + h + 0 * 4, 16 * 8 + 1 * 4, 16 * 8 + 0 * 4,
		                           16 * 8 + h + 3 * 4, 16 * 8 + h + 2 * 4, 16 * 8 + 3 * 4, 16 * 8 + 2 * 4 };
		memcpy(xb, xbv, sizeof(xb));
		static const uint32_t pf[4] = { 0, 1, 2, 3 }, pb[4] = { 3, 2, 1, 0 };
		const rbx_gfx_layout lf = { 16, 8, 4, pf, xf, yf, 16 * 8 * 4 };
		const rbx_gfx_layout lb = { 16, 8, 4, pb, xb, yb, 32 * 8 };
		const rbx_gfx_layout l8 = { 8, 8, 4, pf, x8, y8, 32 * 8 };
		if (rbx_gfx_decode(&s_gfxfg, &lf, fgt, fgs) || rbx_gfx_decode(&s_gfxbg, &lb, bgt, bgs / 2) ||
		    rbx_gfx_decode(&s_gfxspr, &l8, spr, sps)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			sb_exit();
			return -1;
		}
	}
	memset(&s_mix_cfg, 0, sizeof(s_mix_cfg));
	s_mix_cfg.sprpri_shift = 8;
	s_mix_cfg.sprbln_shift = 10;
	s_mix_cfg.sprcol_shift = 4;
	s_mix_cfg.bgblend_comp = 0x0000 + 0x300;
	s_mix_cfg.fgblend_comp = 0x0000 + 0x200;
	s_mix_cfg.txblend_comp = 0x0000 + 0x100;
	s_mix_cfg.spblend_comp = 0x0000 + 0x000;
	s_mix_cfg.bgregular_comp = 0x0800 + 0x300;
	s_mix_cfg.fgregular_comp = 0x0800 + 0x200;
	s_mix_cfg.txregular_comp = 0x0800 + 0x100;
	s_mix_cfg.spregular_comp = 0x0800 + 0x000;
	s_mix_cfg.spblend_source = 0x1000 + 0x000;
	s_mix_cfg.fgblend_source = 0x1000 + 0x100;
	s_mix_cfg.bgpen = 0x800 + 0x300;
	s_mix_cfg.bgpen_blend = 0x000 + 0x300;
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;   // YM3812 4 MHz 1.0, OKI 0.50
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM3812;
	yc.clock = YM_CLOCK;
	yc.gain[0] = 256;
	yc.irq = ym_irq;
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		sb_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 1)) {
		rbx_fail("%s: out of memory", g->set);
		sb_exit();
		return -1;
	}
	oki_init(&s_oki, OKI_CLOCK, 1, oki, osize, 128);
	memset(s_zmap, 0, sizeof(s_zmap));
	for (uint32_t p = 0; p < 0xf0 && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	static const char *const names[5] = { "IN0", "IN1", "SYSTEM", "DSW1", "DSW2" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);   // (MAME's DIVx / MULx / ADDQ.W An times)
	m68k_bus_reset_map();
	m68k_bus_map(0x00000, 0x3ffff, s_rom, 0x40000, 1, 0);
	m68k_bus_map(0x40000, 0x43fff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x50000, 0x50fff, s_spram, sizeof(s_spram), 1, 1);
	m68k_bus_map(0x60000, 0x67fff, s_fgram, sizeof(s_fgram), 1, 1);
	m68k_bus_map(0x70000, 0x77fff, s_bgram, sizeof(s_bgram), 1, 1);
	m68k_bus_map(0x80000, 0x827ff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	sb_reset();
	return 0;
}

static void sb_exit(void)
{
	m68k_set_mame_timing(0);
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gfxfg);
	rbx_gfx_free(&s_gfxbg);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
	free(s_bgram_hp);
	s_bgram_hp = NULL;
	free(s_fgram_hp);
	s_fgram_hp = NULL;
	free(s_pal_rgb_hp);
	s_pal_rgb_hp = NULL;
	free(s_ram_hp);
	s_ram_hp = NULL;
	free(s_sprbmp_hp);
	s_sprbmp_hp = NULL;
}

static void sb_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0 + VIS_H) {   // the vblank: the frame drawn, irq3_line_assert
			vblank_capture();
			m68k_bus_set_line(3, 1);
		}
		m68k_execute(rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		sound_line();
	}
	sound_frame_end();
}

static int sb_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[3] = { s_cpu_frac, s_z80_frac, s_ym_frac };
	int32_t misc[5] = { s_latch, s_latch_pending, s_ym_irq, (int32_t)s_oki_acc, s_oki_hold };
	z80 z = s_z80;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "spr", s_spram, sizeof(s_spram));
	rc |= rbx_state_io(ctx, "fg", s_fgram, sizeof(s_fgram));
	rc |= rbx_state_io(ctx, "bg", s_bgram, sizeof(s_bgram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= rbx_state_io(ctx, "oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		z.read_byte = z80_read;
		z.write_byte = z80_write;
		z.port_in = z80_in;
		z.port_out = z80_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		s_z80 = z;
		s_cpu_frac = fr[0];
		s_z80_frac = fr[1];
		s_ym_frac = fr[2];
		s_latch = (uint8_t)misc[0];
		s_latch_pending = misc[1];
		s_ym_irq = misc[2];
		s_oki_acc = (uint32_t)misc[3];
		s_oki_hold = misc[4];
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int sb_save(void *ctx) { return sb_state(ctx, 0); }
static int sb_load(void *ctx) { return sb_state(ctx, 1); }
static int sb_supported(const char *machine) { return !strcmp(machine, "spbactn"); }

const rbx_driver rbx_drv_spbactn = { "Tecmo Super Pinball Action", sb_init, sb_exit, sb_reset, sb_frame, sb_save, sb_load,
                                     sb_supported, sb_render, sb_help };
