/*  Konami's boards with an HD6309 or a 6809 and the K052109 tilemaps and K051960 sprites: The Main Event,
    Devastators, Bottom of the Ninth, S.P.Y. Ported from MAME src/mame/konami/mainevt.cpp, bottom9.cpp, spy.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail; Nicola Salmoria) and the devices they use (k052109.cpp, k051960.cpp, k051316.cpp,
    k007232.cpp, upd7759.cpp, k051733.cpp; spy's 052591 PMC: its programs as MAME's pmc_run). The HD6309E / 6809E at 3 MHz
    (cpu/hd6309, cpu/m6809), the sprites through konamigx_v.c's pre-GX mode (MAME's bitmap_ind16 and priority bitmap);
    the sound: a Z80 at 3.58 MHz (its NMI from a scanline timer or spy's YM3812), K007232s and a uPD7759, a frame
    behind on the worker.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/hd6309/hd6309.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/k007232.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/upd7759.h"
#include "../video/k051316.h"
#include "../video/k052109.h"
#include "../video/k055673.h"
#include "../machine/k051733.h"
#include "konamigx.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 3000000u     // (24 MHz / 8)
#define Z80_CLOCK 3579545u
#define CHIP_CLOCK 3579545u    // (the K007232s)
#define UPD_CLOCK 640000u
#define SND_RATE 48000
#define PIX_CLOCK 6000000u     // (set_raw(24 MHz / 4, 384, x0, x1, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_Y0 16
#define VIS_H 224

enum { M_MAINEVT, M_BOTTOM9, M_SPY, M_DEVSTORS };
static int s_mach, s_vis_x0, s_vis_w;
static uint8_t *s_rom, *s_zrom, *s_sprrom;
static uint32_t s_rom_size, s_zrom_size, s_sprrom_size;
static const uint8_t *s_upd_rom;
static uint32_t s_upd_size;
static hd6309_t s_cpu;
static m6809_t s_cpu9;   // (spy)
// the main CPU, the HD6309 or spy's 6809
#define CPU_CYCLES (s_mach == M_SPY ? s_cpu9.cycles : s_cpu.cycles)
static void cpu_irq(int state)
{
	if (s_mach == M_SPY)
		m6809_set_irq(&s_cpu9, state);
	else
		hd6309_set_irq(&s_cpu, state);
}
static z80 s_z;
static const uint8_t *s_zmap[256];
static k052109_t s_k109;
static k051316_t s_k316;      // (bottom9)
static k007232_t s_k232[2];
static upd7759_t s_upd;       // (mainevt)
static k051733_t s_k733;      // (devstors)
static int s_port_p[4], s_port_dsw[3], s_port_system;

static struct {
	uint8_t pal8[0x800];   // (the palette: xBGR_555, the high byte first)
	uint8_t ram[0x2000];   // 4000-5fff (mainevt 5e00-5fff the palette)
	uint8_t k51960_ram[0x400], k51960_buffer[0x400];
	uint8_t k51960_control, k51960_shadow, k51960_rombank[3], k51960_romoffset;
	int64_t k51960_busy_until;
	uint8_t mainbank, video_enable, pal_view, k316_view;   // (bottom9 _1f90_w: the views, 0 off 1 RAM 2 ROM)
	uint8_t zram[0x800], soundlatch, z80_irq, nmi_enable, nmi_line, upd_bank;
	uint8_t pmcram[0x800], rambank, pmcbank, pmcpc, old_3f90;   // (spy)
	uint64_t k232_frac;
	int32_t k232_out[2][2];   // (the K007232s' last samples, held between theirs)
	int64_t cycles_done;
	int64_t cpu_end, snd_end;   // (the CPUs' line targets: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, z_frac, z_end, snd_frac, ym_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot (screen_update) ----
static struct { uint8_t k51960_buffer[0x400], k51960_control, k51960_shadow, video_enable; k052109_t k109; k051316_t k316; } s_vs;
static k052109_view_t s_view;

static void tile_cb(int layer, int bank, int *code, int *color, int *flags, int *priority)
{
	(void)bank;
	const int c = *color;
	if (s_mach == M_MAINEVT) {   // (layer colour bases 0, 128, 64; layer 2's category from bit 5)
		static const int base[3] = { 0, 8, 4 };
		*flags = (c & 0x02) ? K052109_TILE_FLIPX : 0;
		*priority = layer == 2 ? (c & 0x20) >> 5 : 0;
		*code |= ((c & 0x01) << 8) | ((c & 0x1c) << 7);
		*color = base[layer] + ((c & 0xc0) >> 6);
		return;
	}
	if (s_mach == M_DEVSTORS) {   // (layer colour bases 0, 0, 64)
		static const int dbase[3] = { 0, 0, 4 };
		*code |= ((c & 0x01) << 8) | ((c & 0x3c) << 7);
		*color = dbase[layer] + ((c & 0xc0) >> 6);
		return;
	}
	if (s_mach == M_SPY) {   // (layer colour bases 768, 0, 256)
		static const int sbase[3] = { 48, 0, 16 };
		*flags = (c & 0x20) ? K052109_TILE_FLIPX : 0;
		*code |= ((c & 0x03) << 8) | ((c & 0x10) << 6) | ((c & 0x0c) << 9) | (bank << 13);
		*color = sbase[layer] + ((c & 0xc0) >> 6);
		return;
	}
	static const int base[3] = { 0, 0, 16 };   // (bottom9)
	*code |= (c & 0x3f) << 8;
	*color = base[layer] + ((c & 0xc0) >> 6);
}

static void sprite_cb(int *code, int *color, int *priority, int *shadow)
{
	(void)code;
	(void)shadow;
	const int c = *color;
	if (s_mach == M_MAINEVT) {
		*priority = (c & 0x20) ? 0xff00 : (c & 0x40) ? 0xff00 | 0xf0f0 : 0xff00 | 0xf0f0 | 0xcccc;
		*color = 192 / 16 + (c & 0x03);
		return;
	}
	if (s_mach == M_DEVSTORS) {
		*color = 128 / 16 + (c & 0x07);
		return;
	}
	*priority = (s_mach == M_SPY ? 0 : 0xf0) | ((c & 0x10) ? 0xaa : 0) | ((~c & 0x20) ? 0xcc : 0);   // (GFX_PMASK_4, 1, 2)
	*color = 512 / 16 + (c & 0x0f);
}

static void zoom_cb(int chip, int *code, int *color)   // (bottom9 zoom_callback)
{
	(void)chip;
	*code |= (*color & 0x03) << 8;
	*color = 768 / 16 + ((*color & 0x3c) >> 2);
}

static void capture(void)
{
	for (int i = 0; i < gx.pal_entries; i++)   // (the palette: xBGR_555 of two bytes, the high one first)
		((uint16_t *)gx.palram)[i] = (uint16_t)(t.pal8[2 * i] << 8 | t.pal8[2 * i + 1]);
	memcpy(s_vs.k51960_buffer, t.k51960_buffer, sizeof(s_vs.k51960_buffer));
	s_vs.k51960_control = t.k51960_control;
	s_vs.k51960_shadow = t.k51960_shadow;
	s_vs.video_enable = t.video_enable;
	memcpy(&s_vs.k109, &s_k109, offsetof(k052109_t, state_end));
	if (s_mach == M_BOTTOM9)
		memcpy(&s_vs.k316, &s_k316, offsetof(k051316_t, state_end));
}

static void draw(void)   // (screen_update)
{
	s_vs.k109.rom = s_k109.rom;
	s_vs.k109.rom_size = s_k109.rom_size;
	s_vs.k109.gfx = s_k109.gfx;
	s_vs.k109.ntiles = s_k109.ntiles;
	s_vs.k109.colors = s_k109.colors;
	s_vs.k109.cb = tile_cb;
	k052109_prepare(&s_vs.k109, &s_view, s_vis_x0 + (s_vis_x0 + s_vis_w - 1) + 1, VIS_Y0 + (VIS_Y0 + VIS_H - 1) + 1);
	uint16_t *f = gx_pre_frame16();
	uint8_t *p = gx_pre_pri();
	const int x0 = s_vis_x0, x1 = s_vis_x0 + s_vis_w - 1, y1 = VIS_Y0 + VIS_H - 1;
#define LAYER(l, fl, pr) k052109_draw(&s_vs.k109, &s_view, l, fl, pr, f, p, s_vis_w, x0, VIS_Y0, x0, VIS_Y0, x1, y1)
	if (s_mach == M_SPY) {
		if (!s_vs.video_enable) {
			gx_pre_fill_pen(48 * 16);   // (layer_colorbase[0] * 16)
			return;
		}
		memset(p, 0, (size_t)(s_vis_w * VIS_H));
		LAYER(1, K052109_DRAW_OPAQUE, 1);
		LAYER(2, 0, 2);
		gx_pre_sprites_051960(s_vs.k51960_buffer, s_vs.k51960_control, s_vs.k51960_shadow, -1, -1, sprite_cb, 0);
		LAYER(0, 0, 0);
		return;
	}
	if (s_mach == M_DEVSTORS) {   // (the sprites without the priority bitmap: it cleared so that none are masked)
		memset(p, 0, (size_t)(s_vis_w * VIS_H));
		LAYER(1, K052109_DRAW_OPAQUE, 0);
		LAYER(2, 0, 0);
		gx_pre_sprites_051960(s_vs.k51960_buffer, s_vs.k51960_control, s_vs.k51960_shadow, 0, 0, sprite_cb, 0);
		LAYER(0, 0, 0);
		return;
	}
	if (s_mach == M_MAINEVT) {
		memset(p, 0, (size_t)(s_vis_w * VIS_H));
		LAYER(1, K052109_DRAW_OPAQUE, 1);
		LAYER(2, 1, 2);   // (the low priority part of the layer)
		LAYER(2, 0, 4);   // (the high priority part)
		LAYER(0, 0, 8);
		gx_pre_sprites_051960(s_vs.k51960_buffer, s_vs.k51960_control, s_vs.k51960_shadow, -1, -1, sprite_cb, 0);
		return;
	}
	gx_pre_fill_pen(0);   // (bitmap.fill(layer_colorbase[1] * 16))
	if (!s_vs.video_enable)
		return;
	k051316_t *z = &s_vs.k316;
	z->rom = s_k316.rom;
	z->rom_size = s_k316.rom_size;
	z->gfx = s_k316.gfx;
	z->ntiles = s_k316.ntiles;
	z->pix = s_k316.pix;   // (the drawing's cache, the render thread's own)
	z->key = s_k316.key;
	z->cb = s_k316.cb;
	z->bpp = s_k316.bpp;
	k051316_zoom_draw(z, f, p, s_vis_w, x0, VIS_Y0, x0, VIS_Y0, x1, y1, 0, 1);
	LAYER(2, K052109_DRAW_PRIMASK(2), 2);
	LAYER(1, K052109_DRAW_PRIMASK(4), 4);
	gx_pre_sprites_051960(s_vs.k51960_buffer, s_vs.k51960_control, s_vs.k51960_shadow, -1, -1, sprite_cb, 0);
#undef LAYER
}

// ---- the K051960 / K051937 (as konami8b.c's) ----
static inline int64_t now_cycles(void) { return t.cycles_done + CPU_CYCLES; }
static void irq_lines(void) { cpu_irq(s_k109.irq_out); }   // (the K052109's IRQ)

static uint8_t k051960_fetchromdata(int offset)
{
	int addr = t.k51960_romoffset + (t.k51960_rombank[0] << 8) + ((t.k51960_rombank[1] & 0x03) << 16);
	int code = (addr & 0x3ffe0) >> 5, color = ((t.k51960_rombank[1] & 0xfc) >> 2) + ((t.k51960_rombank[2] & 0x03) << 6);
	const int off1 = addr & 0x1f;
	int pri = 0, shadow = 0;
	sprite_cb(&code, &color, &pri, &shadow);
	addr = (code << 7) | (off1 << 2) | offset;
	return s_sprrom[(uint32_t)addr & (s_sprrom_size - 1)];
}
static uint8_t k051960_r(int offset)
{
	if (t.k51960_control & 0x20) {
		t.k51960_romoffset = (uint8_t)((offset & 0x3fc) >> 2);
		return k051960_fetchromdata(offset & 3);
	}
	return t.k51960_ram[offset & 0x3ff];
}
static uint8_t k051937_r(int offset)
{
	offset &= 7;
	if ((t.k51960_control & 0x20) && (offset & 4))
		return k051960_fetchromdata(offset & 3);
	if (offset == 0)
		return now_cycles() < t.k51960_busy_until;
	return 0;
}
static void k051937_w(int offset, uint8_t v)
{
	offset &= 7;
	if (offset == 0)
		t.k51960_control = v;
	else if (offset == 1)
		t.k51960_shadow = v & 0x07;
	else if (offset < 5)
		t.k51960_rombank[offset - 2] = v;
}
static void k051960_vblank(void)
{
	if (t.k51960_control & 0x10)
		return;
	memcpy(t.k51960_buffer, t.k51960_ram, sizeof(t.k51960_buffer));
	int active = 0;
	for (int i = 0; i < 0x400; i += 8)
		active += t.k51960_buffer[i] >> 7;
	const uint32_t ticks = (uint32_t)(active * 32 + (128 - active) * 18);
	t.k51960_busy_until = now_cycles() + (int64_t)((uint64_t)ticks * 4 * CPU_CLOCK / 24000000u);
}

static uint8_t vid_r(int o)   // (k052109_051960_r)
{
	if (!s_k109.rmrd_line) {
		if (o >= 0x3800 && o < 0x3808)
			return k051937_r(o - 0x3800);
		if (o >= 0x3c00)
			return k051960_r(o - 0x3c00);
	}
	return k052109_read(&s_k109, o);
}
static void vid_w(int o, uint8_t v)
{
	if (o >= 0x3800 && o < 0x3808)
		k051937_w(o - 0x3800, v);
	else if (o < 0x3c00) {
		k052109_write(&s_k109, o, v);
		irq_lines();
	} else
		t.k51960_ram[o - 0x3c00] = v;
}

// ---- the sound board a frame behind on the worker (rbx_aux): the latch, the IRQ and each line's samples queued ----
#define Q_MAX 2048
enum { Q_ADV, Q_LATCH, Q_IRQ };
typedef struct { uint16_t kind, arg; uint32_t v; } q_ev_t;
static q_ev_t (*s_q_hp)[2][Q_MAX];
#define s_q (*s_q_hp)
static int s_qn[2], s_qcur;
static int32_t s_out[2048];
static int s_out_n, s_out_pos;
static volatile int s_job_frame;

static void q_put(int kind, int arg, uint32_t v)
{
	const int c = s_qcur;
	if (s_qn[c] < Q_MAX) {
		s_q[c][s_qn[c]].kind = (uint16_t)kind;
		s_q[c][s_qn[c]].arg = (uint16_t)arg;
		s_q[c][s_qn[c]].v = v;
		s_qn[c]++;
	}
}

// ---- the main CPU's bus ----
static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }

static uint8_t me_rd(void *ctx, uint16_t a)   // (mainevt main_map; devstors: no P3 / P4, the K051733)
{
	(void)ctx;
	if (a < 0x4000) {
		if (s_mach == M_DEVSTORS) {
			if (a >= 0x1fa0 && a < 0x1fc0)
				return k051733_read(&s_k733, a - 0x1fa0);
			if (a == 0x1f99 || a == 0x1f9a)
				return vid_r(a);
		}
		switch (a) {
		case 0x1f8c: return 0;   // (watchdog)
		case 0x1f94: return port(s_port_system);
		case 0x1f95: return port(s_port_p[0]);
		case 0x1f96: return port(s_port_p[1]);
		case 0x1f97: return port(s_port_dsw[0]);
		case 0x1f98: return port(s_port_dsw[2]);
		case 0x1f99: return port(s_port_p[2]);
		case 0x1f9a: return port(s_port_p[3]);
		case 0x1f9b: return port(s_port_dsw[1]);
		}
		return vid_r(a);
	}
	if (a < 0x6000)
		return a >= 0x5e00 ? t.pal8[a - 0x5e00] : t.ram[a - 0x4000];
	if (a < 0x8000)
		return s_rom[((uint32_t)t.mainbank * 0x2000 + (a - 0x6000)) % s_rom_size];
	return s_rom[a];
}
static void me_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x4000) {
		switch (a) {
		case 0x1f80:   // (bankswitch_w)
			t.mainbank = v & 3;
			k052109_set_rmrd_line(&s_k109, (v >> 6) & 1);
			return;
		case 0x1f84: q_put(Q_LATCH, 0, v); return;
		case 0x1f88: q_put(Q_IRQ, 0, 0); return;   // (sh_irqtrigger_w: HOLD_LINE)
		case 0x1f8c: case 0x1f90: return;          // (watchdog, coin counters and lamps)
		}
		if (s_mach == M_DEVSTORS && a >= 0x1fa0 && a < 0x1fc0) {
			k051733_write(&s_k733, a - 0x1fa0, v);
			return;
		}
		vid_w(a, v);
		return;
	}
	if (a < 0x5e00)
		t.ram[a - 0x4000] = v;
	else if (a < 0x6000)
		t.pal8[a - 0x5e00] = v;
}

static uint8_t b9_rd(void *ctx, uint16_t a)   // (bottom9 main_map)
{
	(void)ctx;
	if (a < 0x4000) {
		if (a < 0x0800 && t.k316_view)   // (the K051316's RAM or ROM)
			return t.k316_view == 2 ? k051316_rom_r(&s_k316, a) : s_k316.ram[a];
		if (a >= 0x2000 && a < 0x2800 && t.pal_view)
			return t.pal8[a - 0x2000];
		switch (a) {
		case 0x1fd0: return port(s_port_system);
		case 0x1fd1: return port(s_port_p[0]);
		case 0x1fd2: return port(s_port_p[1]);
		case 0x1fd3: return port(s_port_dsw[0]);
		case 0x1fe0: return port(s_port_dsw[1]);
		}
		return vid_r(a);
	}
	if (a < 0x6000)
		return t.ram[a - 0x4000];
	if (a < 0x8000)
		return s_rom[((uint32_t)t.mainbank * 0x2000 + (a - 0x6000)) % s_rom_size];
	return s_rom[0x18000 + (a - 0x8000)];
}
static void b9_1f90_w(uint8_t v)   // (_1f90_w)
{
	k052109_set_rmrd_line(&s_k109, (v >> 2) & 1);
	t.video_enable = !((v >> 3) & 1);
	if (v & 0x20)
		t.pal_view = t.k316_view = 0;
	else {
		t.pal_view = 1;
		t.k316_view = (uint8_t)(1 + ((v >> 4) & 1));
	}
}
static void b9_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x4000) {
		if (a < 0x0800 && t.k316_view) {
			k051316_write(&s_k316, a, v);
			return;
		}
		if (a >= 0x2000 && a < 0x2800 && t.pal_view) {
			t.pal8[a - 0x2000] = v;
			return;
		}
		if (a >= 0x1ff0 && a < 0x2000) {
			k051316_ctrl_w(&s_k316, a - 0x1ff0, v);
			return;
		}
		switch (a) {
		case 0x1f80:   // (bankswitch_w: bit 4 the ROM's second half)
			t.mainbank = (v & 0x10) ? (uint8_t)(8 + ((v & 0x06) >> 1)) : (uint8_t)((v & 0x0e) >> 1);
			return;
		case 0x1f90: b9_1f90_w(v); return;
		case 0x1fa0: return;   // (watchdog)
		case 0x1fb0: q_put(Q_LATCH, 0, v); return;
		case 0x1fc0: q_put(Q_IRQ, 0, 0); return;   // (sh_irqtrigger_w: HOLD_LINE)
		}
		vid_w(a, v);
		return;
	}
	if (a < 0x6000)
		t.ram[a - 0x4000] = v;
}

// -- spy (main_map: 0000-07ff the palette, the PMC's program counter or its RAM with ram_view) --
static void pmc_run(void)   // (MAME's pmc_run: the projection or the collision program)
{
	uint8_t *m = t.pmcram;
	if (t.pmcpc == 0x00) {
		uint16_t loopend = (uint16_t)(m[0] << 8 | m[1]), nearplane = (uint16_t)(m[2] << 8 | m[3]);
		if (loopend > 64)
			loopend = 64;
		if (!nearplane)
			nearplane = 0x6400;
		loopend = (uint16_t)((loopend << 1) + 4);
		for (int i = 4; i < loopend; i += 2) {
			const uint32_t op = ((uint32_t)(m[i] << 8 | m[i + 1]) << 8) / nearplane;
			m[i] = (uint8_t)(op >> 8);
			m[i + 1] = (uint8_t)op;
		}
		memset(m + loopend, 0, (size_t)(0x800 - loopend));   // (clean up for next frame)
		return;
	}
	if (!m[0x2])
		return;
	const int count = m[0] << 8 | m[1];
	for (int i = 16; i < 16 + 14 * count && i + 13 < 0x800; i += 14) {
		if (!m[i])
			continue;
		uint8_t tests_failed = 3;
		for (int j = 0; j < 3 * 4; j += 4) {
			const int16_t a_pos = (int16_t)(m[j + 3] << 8 | m[j + 4]), a_size = (int16_t)(m[j + 5] << 8 | m[j + 6]);
			const int16_t b_pos = (int16_t)(m[i + j + 1] << 8 | m[i + j + 2]), b_size = (int16_t)(m[i + j + 3] << 8 | m[i + j + 4]);
			const int16_t a_max = (int16_t)(a_pos + a_size), a_min = (int16_t)(a_pos - a_size);
			const int16_t b_max = (int16_t)(b_pos + b_size), b_min = (int16_t)(b_pos - b_size);
			if (b_min > a_min) {
				if (a_max >= b_min)
					tests_failed--;
			} else if (b_max >= a_min)
				tests_failed--;
		}
		if (!tests_failed)
			m[0xf] = 0;
		m[i + 0xd] = tests_failed;
	}
}

static uint8_t spy_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x0800 && t.rambank) {   // (ram_view: 1 the palette, else the PMC's program counter or RAM)
		if (t.rambank & 1)
			return t.pal8[a];
		return t.pmcbank ? t.pmcram[a] : 0;
	}
	if (a < 0x1b00)
		return t.ram[a];
	if (a < 0x2000)
		return 0;
	if (a < 0x6000) {
		switch (a) {
		case 0x3fd0: return port(s_port_system);
		case 0x3fd1: return port(s_port_p[0]);
		case 0x3fd2: return port(s_port_p[1]);
		case 0x3fd3: return port(s_port_dsw[0]);
		case 0x3fe0: return port(s_port_dsw[1]);
		}
		return vid_r(a - 0x2000);
	}
	if (a < 0x8000)
		return s_rom[(0x10000u + (uint32_t)t.mainbank * 0x2000 + (a - 0x6000)) % s_rom_size];
	return s_rom[a];
}
static void spy_3f90_w(uint8_t v)   // (_3f90_w)
{
	k052109_set_rmrd_line(&s_k109, (v >> 2) & 1);
	t.video_enable = !((v >> 3) & 1);
	t.rambank = (v & 0x30) >> 4;
	t.pmcbank = (v >> 7) & 1;
	if ((v & 0x40) && !(t.old_3f90 & 0x40)) {   // (the PMC's program, then its FIRQ)
		pmc_run();
		m6809_hold_firq(&s_cpu9);
	}
	t.old_3f90 = v;
}
static void spy_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0800 && t.rambank) {
		if (t.rambank & 1)
			t.pal8[a] = v;
		else if (t.pmcbank)
			t.pmcram[a] = v;
		else
			t.pmcpc = v & 0x3f;
		return;
	}
	if (a < 0x1b00) {
		t.ram[a] = v;
		return;
	}
	if (a < 0x2000 || a >= 0x6000)
		return;
	switch (a) {
	case 0x3f80:   // (bankswitch_w: bit 4 the ROM's second half)
		t.mainbank = (v & 0x10) ? (uint8_t)(8 + ((v & 0x06) >> 1)) : (uint8_t)((v & 0x0e) >> 1);
		return;
	case 0x3f90: spy_3f90_w(v); return;
	case 0x3fa0: return;   // (watchdog)
	case 0x3fb0: q_put(Q_LATCH, 0, v); return;
	case 0x3fc0: q_put(Q_IRQ, 0, 0); return;   // (sh_irqtrigger_w: HOLD_LINE)
	}
	vid_w(a - 0x2000, v);
}

// ---- the sound board (mainevt sound_map, bottom9 audio_map) ----
static void k007232_port0(uint8_t data)   // (volume_callback)
{
	k007232_set_volume(&s_k232[0], 0, (data >> 4) * 0x11, 0);
	k007232_set_volume(&s_k232[0], 1, 0, (data & 0x0f) * 0x11);
}
static void k007232_port1(uint8_t data)
{
	k007232_set_volume(&s_k232[1], 0, (data >> 4) * 0x11, 0);
	k007232_set_volume(&s_k232[1], 1, 0, (data & 0x0f) * 0x11);
}
static void upd_bank(void)   // (set_rom_bank: 128 KB pages)
{
	const uint32_t off = (uint32_t)t.upd_bank * 0x20000;
	s_upd.rom = s_upd_rom && off < s_upd_size ? s_upd_rom + off : s_upd_rom;
	s_upd.rom_size = s_upd_rom && off < s_upd_size ? (s_upd_size - off < 0x20000 ? s_upd_size - off : 0x20000) : 0;
}

static void spy_ym_irq(int state)   // (the YM3812's IRQ to the Z80's NMI)
{
	if (state && !t.nmi_line)
		z80_gen_nmi(&s_z);
	t.nmi_line = (uint8_t)(state != 0);
}

static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	if ((s_mach == M_SPY || s_mach == M_DEVSTORS) && (a == 0xc000 || a == 0xc001))
		return rbx_ym_read(0, a & 1);
	if (s_mach == M_MAINEVT || s_mach == M_DEVSTORS) {
		if (a < 0x8400)
			return t.zram[a - 0x8000];
		if (a == 0xa000)
			return t.soundlatch;
		if (a >= 0xb000 && a < 0xb00e)
			return k007232_read(&s_k232[0], a - 0xb000);
		if (a == 0xd000 && s_mach == M_MAINEVT)
			return (uint8_t)upd7759_busy_r(&s_upd);
		return 0;
	}
	if (a < 0x8800)
		return t.zram[a - 0x8000];
	if (a >= 0xa000 && a < 0xa00e)
		return k007232_read(&s_k232[0], a - 0xa000);
	if (a >= 0xb000 && a < 0xb00e)
		return k007232_read(&s_k232[1], a - 0xb000);
	if (a == 0xd000)
		return t.soundlatch;
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_mach == M_DEVSTORS) {
		if (a >= 0x8000 && a < 0x8400)
			t.zram[a - 0x8000] = v;
		else if (a >= 0xb000 && a < 0xb00e)
			k007232_write(&s_k232[0], a - 0xb000, v);
		else if (a == 0xc000 || a == 0xc001)
			rbx_ym_write(0, a & 1, v);
		else if (a == 0xe000)   // (sh_irqcontrol_w: the scanline timer's IRQ mask)
			t.nmi_enable = (v >> 2) & 1;
		else if (a == 0xf000)   // (sh_bankswitch_w)
			k007232_set_bank(&s_k232[0], v & 3, (v >> 2) & 3);
		return;
	}
	if (s_mach == M_MAINEVT) {
		if (a >= 0x8000 && a < 0x8400)
			t.zram[a - 0x8000] = v;
		else if (a == 0x9000)
			upd7759_port_w(&s_upd, v);
		else if (a >= 0xb000 && a < 0xb00e)
			k007232_write(&s_k232[0], a - 0xb000, v);
		else if (a == 0xe000) {   // (sh_irqcontrol_w)
			upd7759_reset_w(&s_upd, (v >> 1) & 1);
			upd7759_start_w(&s_upd, !(v & 1));
			t.nmi_enable = (v >> 2) & 1;
		} else if (a == 0xf000) {   // (sh_bankswitch_w)
			k007232_set_bank(&s_k232[0], v & 3, (v >> 2) & 3);
			t.upd_bank = (v >> 4) & 3;
			upd_bank();
		}
		return;
	}
	if (s_mach == M_SPY && (a == 0xc000 || a == 0xc001))
		rbx_ym_write(0, a & 1, v);
	else if (a >= 0x8000 && a < 0x8800)
		t.zram[a - 0x8000] = v;
	else if (a == 0x9000) {   // (sound_bank_w)
		k007232_set_bank(&s_k232[0], v & 3, (v >> 2) & 3);
		k007232_set_bank(&s_k232[1], (v >> 4) & 3, (v >> 6) & 3);
	} else if (a >= 0xa000 && a < 0xa00e)
		k007232_write(&s_k232[0], a - 0xa000, v);
	else if (a >= 0xb000 && a < 0xb00e)
		k007232_write(&s_k232[1], a - 0xb000, v);
	else if (a == 0xf000) {   // (nmi_enable_w)
		t.nmi_enable = v & 1;
		if (!t.nmi_enable)
			t.nmi_line = 0;
	}
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// a line's n samples: the scanline timer's NMI first (at vpos & 0x1f == 0x10; devstors its IRQ at vpos & 0x3f == 0x20),
// the Z80 for their cycles, the K007232s at their rate (held between) and the uPD7759 or the YM; mono
static void sound_run(int n, int vpos)
{
	if (s_mach == M_DEVSTORS) {
		if ((vpos & 0x3f) == 0x20 && t.nmi_enable)
			t.z80_irq = 1;   // (HOLD_LINE)
	} else if ((vpos & 0x1f) == 0x10 && t.nmi_enable && s_mach != M_SPY) {
		if (s_mach == M_MAINEVT)   // (pulse_input_line)
			z80_gen_nmi(&s_z);
		else if (!t.nmi_line) {   // (ASSERT_LINE until nmi_enable_w(0))
			t.nmi_line = 1;
			z80_gen_nmi(&s_z);
		}
	}
	if (n <= 0)
		return;
	t.z_frac += (uint64_t)Z80_CLOCK * (uint64_t)n;
	const unsigned long cyc = (unsigned long)(t.z_frac / SND_RATE);
	t.z_frac -= (uint64_t)cyc * SND_RATE;
	int line = t.z80_irq;
	t.z_end += cyc;
	z80_run(&s_z, (unsigned long)t.z_end, &line, 1);
	t.z80_irq = (uint8_t)line;
	if (n > 64)
		n = 64;
	int32_t up[64], ym[96];
	if (s_mach == M_MAINEVT)
		upd7759_render(&s_upd, up, n);
	int yn = 0;
	if (s_mach == M_SPY || s_mach == M_DEVSTORS) {   // (the YM3812 1.0; the YM2151 0.30 each side, mono)
		memset(ym, 0, sizeof(ym));
		t.ym_frac += (uint64_t)CHIP_CLOCK * (uint64_t)n;
		const uint32_t yc = (uint32_t)(t.ym_frac / SND_RATE);
		t.ym_frac -= (uint64_t)yc * SND_RATE;
		yn = rbx_ym_advance(0, yc, ym, 96);
	}
	const int nk = s_mach == M_MAINEVT || s_mach == M_DEVSTORS ? 1 : 2;
	for (int i = 0; i < n; i++) {
		t.k232_frac += CHIP_CLOCK / 128;   // (their rate: clock / 128)
		while (t.k232_frac >= SND_RATE) {
			t.k232_frac -= SND_RATE;
			for (int k = 0; k < nk; k++)
				k007232_sample(&s_k232[k], &t.k232_out[k][0], &t.k232_out[k][1]);
		}
		int32_t m;
		if (s_mach == M_MAINEVT)   // (the K007232 0.20, the uPD7759 0.50)
			m = (t.k232_out[0][0] + t.k232_out[0][1]) / 5 + up[i] / 2;
		else if (s_mach == M_DEVSTORS)   // (the K007232 0.20)
			m = (yn > 0 ? ym[i * yn / n] : 0) + (t.k232_out[0][0] + t.k232_out[0][1]) / 5;
		else if (s_mach == M_SPY)   // (the K007232s 0.20)
			m = (yn > 0 ? ym[i * yn / n] : 0) + (t.k232_out[0][0] + t.k232_out[0][1] + t.k232_out[1][0] + t.k232_out[1][1]) / 5;
		else   // (the K007232s 0.40)
			m = (t.k232_out[0][0] + t.k232_out[0][1] + t.k232_out[1][0] + t.k232_out[1][1]) * 2 / 5;
		if (s_out_pos < 2048)
			s_out[s_out_pos++] = m;
	}
}

static void job(void)
{
	if (!__atomic_load_n(&s_job_frame, __ATOMIC_ACQUIRE))
		return;
	const int b = s_qcur ^ 1;
	s_out_pos = 0;
	for (int i = 0; i < s_qn[b]; i++) {
		const q_ev_t *e = &s_q[b][i];
		switch (e->kind) {
		case Q_ADV: sound_run((int)e->v, e->arg); break;
		case Q_LATCH: t.soundlatch = (uint8_t)e->v; break;
		case Q_IRQ: t.z80_irq = 1; break;
		}
	}
	s_qn[b] = 0;
	s_out_n = s_out_pos;
	__atomic_store_n(&s_job_frame, 0, __ATOMIC_RELEASE);
}

static void sound_frame(void)
{
	rbx_aux_sync();
	for (int i = 0; i < s_out_n; i++)
		rbx_audio_put_stereo(s_out[i], s_out[i]);
	s_out_n = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void k63_exit(void);
static void k63_reset(void);

static int k63_init(const rbx_game *g)
{
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_mach = !strcmp(g->machine, "bottom9") ? M_BOTTOM9 : !strcmp(g->machine, "spy") ? M_SPY
	       : !strcmp(g->machine, "devstors") ? M_DEVSTORS : M_MAINEVT;
	const int b9 = s_mach == M_BOTTOM9, spy = s_mach == M_SPY, two = b9 || spy;   // (two K007232s)
	const int dv = s_mach == M_DEVSTORS;
	uint32_t tile_size, pcm0_size = 0, pcm1_size = 0, zsize = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *tiles = rbx_region("k052109", &tile_size);
	s_sprrom = rbx_region("k051960", &s_sprrom_size);
	const uint8_t *pcm0 = rbx_region(two ? "k007232_1" : "k007232", &pcm0_size);
	const uint8_t *pcm1 = two ? rbx_region("k007232_2", &pcm1_size) : NULL;
	const uint8_t *zrom = b9 ? rbx_region("k051316", &zsize) : NULL;
	s_upd_size = 0;
	s_upd_rom = two || dv ? NULL : rbx_region("upd", &s_upd_size);
	if (!s_rom || !s_zrom || !tiles || !s_sprrom || !pcm0 || (two && !pcm1) || (b9 && !zrom) || (!two && !dv && !s_upd_rom)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_vis_x0 = spy ? 8 : dv ? 24 : 16;   // (set_raw(..., 0+16, 320-16, ...); spy 0+8, 320-8; devstors 0+24, 320-8)
	s_vis_w = spy ? 304 : 288;
	memset(&gx, 0, sizeof(gx));
	memset(&t, 0, sizeof(t));
	gx.pal_entries = two ? 1024 : 256;
	gx.pal555 = 1;
	gx.tilecb = GX_TILECB_PRE;
	gx.mixmode = GX_MIXMODE_PRE;
	gx.pre_capture = capture;
	gx.pre_draw = draw;
	gx.pre_screen_w = HTOTAL;
	gx.pre_dim = 1;
	for (int i = 0; i < 8; i++)
		gx.pre_contrast[i] = 1.0f;
	k055673_init(&gx.k55673, s_sprrom, s_sprrom_size, K055673_LAYOUT_051960, 0, 0);
	gx_video_set_area(s_vis_x0, s_vis_w);
	gx_video_set_y0(VIS_Y0);
	gx_video_set_height(VIS_H);
	if (k052109_init(&s_k109, tiles, tile_size, gx.pal_entries, tile_cb) ||
	    (b9 && k051316_init(&s_k316, 0, zrom, zsize, 0, 0, zoom_cb))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	k007232_init(&s_k232[0], pcm0, pcm0_size, k007232_port0);
	k007232_init(&s_k232[1], pcm1, pcm1_size, k007232_port1);
	if (!two && !dv)
		upd7759_init(&s_upd, s_upd_rom, s_upd_size, UPD_CLOCK, SND_RATE);
	if (spy || dv) {
		rbx_ym_config yc;
		memset(&yc, 0, sizeof(yc));
		yc.type = dv ? RBX_YM2151 : RBX_YM3812;
		yc.clock = CHIP_CLOCK;
		yc.gain[0] = dv ? 77 : 256;   // (devstors 0.30 each side)
		yc.gain[1] = 77;
		yc.irq = dv ? NULL : spy_ym_irq;
		if (rbx_ym_init(0, &yc)) {
			rbx_fail("%s: YM", g->set);
			return -1;
		}
	}
	static const char *const pn[4] = { "P1", "P2", "P3", "P4" }, *const dn[3] = { "DSW1", "DSW2", "DSW3" };
	for (int i = 0; i < 4; i++)
		s_port_p[i] = rbx_port_find(pn[i]);
	for (int i = 0; i < 3; i++)
		s_port_dsw[i] = rbx_port_find(dn[i]);
	s_port_system = rbx_port_find("SYSTEM");
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(s_vis_w, VIS_H, 0x10000) || gx_video_init()) {
		rbx_fail("%s: out of memory", g->set);
		k63_exit();
		return -1;
	}
	for (int i = 0; i < 0x10000; i++) {   // (the bitmap: RGB 565)
		const int r = (i >> 11) & 31, gg = (i >> 5) & 63, b = i & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (gg << 2 | gg >> 4) << 8 | (b << 3 | b >> 2));
	}
	k63_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void k63_exit(void)
{
	rbx_aux_set_job(NULL);
	gx_video_exit();
	k052109_exit(&s_k109);
	if (s_mach == M_BOTTOM9)
		k051316_exit(&s_k316);
	if (s_mach == M_SPY || s_mach == M_DEVSTORS)
		rbx_ym_exit(0);
	free(s_q_hp);
	s_q_hp = NULL;
}

static void map_bank(void) {}   // (the banked ROM read through the handlers)

static void k63_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	k052109_reset(&s_k109);
	for (int i = 0; i < 2; i++)
		k007232_reset(&s_k232[i]);
	if (s_mach == M_BOTTOM9) {
		k051316_reset(&s_k316);
		b9_1f90_w(0);   // (machine_reset: nmi_enable_w(0), _1f90_w(0))
	} else if (s_mach == M_SPY) {
		t.old_3f90 = 0xff;   // (machine_reset: m_old_3f90 = -1, the views off, the video off)
		rbx_ym_reset(0);
	} else if (s_mach == M_DEVSTORS) {
		k051733_reset(&s_k733);
		rbx_ym_reset(0);
	} else {
		upd7759_reset(&s_upd);   // (machine_reset: sh_irqcontrol_w(0): /RESET 0, /ST 1)
		upd7759_reset_w(&s_upd, 0);
		upd7759_start_w(&s_upd, 1);
		upd_bank();
	}
	if (s_mach == M_SPY) {
		m6809_init(&s_cpu9);
		s_cpu9.read = spy_rd;
		s_cpu9.write = spy_wr;
		s_cpu9.ctx = NULL;
		for (int p = 0x80; p < 0x100; p++)   // (8000-ffff the ROM's own, 0800-1aff the RAM)
			s_cpu9.rpage[p] = s_rom + (uint32_t)p * 256;
		for (int p = 0x08; p < 0x1b; p++)
			s_cpu9.rpage[p] = s_cpu9.wpage[p] = t.ram + p * 256;
		m6809_reset(&s_cpu9);
	} else {
		hd6309_init(&s_cpu);
		s_cpu.read = s_mach == M_BOTTOM9 ? b9_rd : me_rd;
		s_cpu.write = s_mach == M_BOTTOM9 ? b9_wr : me_wr;
		s_cpu.ctx = NULL;
		const uint32_t fix = s_mach == M_BOTTOM9 ? 0x18000 : 0x8000;   // (8000-ffff: the fixed ROM)
		for (int p = 0x80; p < 0x100; p++)
			s_cpu.rpage[p] = s_rom + fix + (uint32_t)(p - 0x80) * 256;
		for (int p = 0x40; p < (s_mach == M_BOTTOM9 ? 0x60 : 0x5e); p++)
			s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + (p - 0x40) * 256;
		map_bank();
		hd6309_reset(&s_cpu);
	}
	z80_init(&s_z);
	s_z.read_byte = z_rd;
	s_z.write_byte = z_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x80; p++)
		s_zmap[p] = (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
	for (int p = 0; p < (s_mach == M_MAINEVT || s_mach == M_DEVSTORS ? 4 : 8); p++)
		s_zmap[0x80 + p] = t.zram + p * 256;
	s_z.read_map = s_zmap;
}

static void k63_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, the K051960's sprite DMA, the K052109's IRQ)
			gx_video_capture();
			k051960_vblank();
			k052109_vblank(&s_k109, 1);
			irq_lines();
			if (s_mach == M_DEVSTORS && k051733_nmiclock(&s_k733, 1)) {   // (the K051733's nmiclock_w)
				hd6309_set_nmi(&s_cpu, 1);
				hd6309_set_nmi(&s_cpu, 0);
			}
		}
		if (vpos == VIS_Y0) {
			k052109_vblank(&s_k109, 0);
			irq_lines();
			if (s_mach == M_DEVSTORS)
				k051733_nmiclock(&s_k733, 0);
		}
		t.cyc_frac += (uint64_t)CPU_CLOCK * HTOTAL;
		const int64_t cyc = (int64_t)(t.cyc_frac / PIX_CLOCK);
		t.cyc_frac -= (uint64_t)cyc * PIX_CLOCK;
		if (s_mach == M_SPY)
			m6809_run_until(&s_cpu9, t.cpu_end += cyc);
		else
			hd6309_run_until(&s_cpu, t.cpu_end += cyc);
		t.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(t.snd_frac / PIX_CLOCK);
		t.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		q_put(Q_ADV, vpos, (uint32_t)ns);
	}
	sound_frame();
	t.frame++;
	gx.frame = t.frame;
}

static int k63_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	if (s_mach == M_SPY)
		rc |= rbx_state_io(ctx, "k63_cpu9", &s_cpu9, (uint32_t)offsetof(m6809_t, rpage));
	else
		rc |= rbx_state_io(ctx, "k63_cpu", &s_cpu, (uint32_t)offsetof(hd6309_t, rpage));
	rc |= rbx_state_io(ctx, "gx", &gx, (uint32_t)offsetof(gx_t, state_end));
	rc |= k052109_state(&s_k109, ctx, load, "k63_k109");
	rc |= rbx_state_io(ctx, "k63", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= k007232_state(&s_k232[0], ctx, load, "k63_k232a");
	rc |= k007232_state(&s_k232[1], ctx, load, "k63_k232b");
	if (s_mach == M_BOTTOM9)
		rc |= k051316_state(&s_k316, ctx, load, "k63_k316");
	else if (s_mach == M_SPY)
		rc |= rbx_ym_state(0, ctx, load);
	else if (s_mach == M_DEVSTORS) {
		rc |= rbx_ym_state(0, ctx, load);
		rc |= rbx_state_io(ctx, "k63_k733", &s_k733, sizeof(s_k733));
	} else
		rc |= upd7759_state(&s_upd, ctx, load, "k63_upd");
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "k63_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "k63_out", s_out, sizeof(s_out));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "k63_z80", &z, sizeof(z));
	if (!rc && load) {
		s_out_n = qn >= 0 && qn <= 2048 ? qn : 0;
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
		z.read_byte = z_rd;
		z.write_byte = z_wr;
		z.port_in = z_in;
		z.port_out = z_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		z.op_rom = NULL;
		s_z = z;
		if (s_mach == M_MAINEVT)
			upd_bank();
	}
	return rc ? -1 : 0;
}

static int k63_save(void *ctx) { return k63_state(ctx, 0); }
static int k63_load(void *ctx) { return k63_state(ctx, 1); }
static int k63_supported(const char *machine)
{
	return !strcmp(machine, "mainevt") || !strcmp(machine, "bottom9") || !strcmp(machine, "spy") ||
	       !strcmp(machine, "devstors");
}

const rbx_driver rbx_drv_mainevt = { "Konami Main Event", k63_init, k63_exit, k63_reset, k63_frame, k63_save, k63_load,
                                     k63_supported, gx_render, NULL };
