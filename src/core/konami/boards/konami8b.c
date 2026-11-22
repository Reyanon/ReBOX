/*  Konami's 8-bit boards with the Konami CPU, the K052109 tilemaps and the K051960 sprites: Aliens, Crime Fighters,
    Block Hole, Super Contra, Thunder Cross, Gang Busters, '88 Games. Ported from MAME src/mame/konami/aliens.cpp,
    crimfght.cpp, blockhl.cpp, thunderx.cpp, 88games.cpp (BSD-3-Clause, copyright-holders: Manuel Abadia; Nicola
    Salmoria) and the devices they use (k052109.cpp, k051960.cpp, k051316.cpp, k007232.cpp, upd7759.cpp; thunderx's 052591 PMC: its collision program as MAME's pmc_run). The Konami CPU at
    3 MHz (cpu/konami), the K051960 sprites through konamigx_v.c's pre-GX mode (MAME's bitmap_ind16 and priority
    bitmap); the sound: a Z80 at 3.58 MHz, a YM2151 (its CT lines the K007232's banks) and a K007232, a frame behind on
    the worker.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/konami/konami.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/k007232.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/upd7759.h"
#include "../video/k051316.h"
#include "../video/k052109.h"
#include "../video/k055673.h"
#include "konamigx.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 3000000u     // (24 MHz / 2, the CPU's internal divider of 4)
#define Z80_CLOCK 3579545u
#define CHIP_CLOCK 3579545u    // (the YM2151 and the K007232)
#define SND_RATE 48000
#define PIX_CLOCK 6000000u     // (set_raw(24 MHz / 4, 384, x0, x1, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_Y0 16
#define VIS_H 224
#define PMASK_1 0xaa           // (GFX_PMASK_1, 2, 4 for the priority values below 8)
#define PMASK_2 0xcc
#define PMASK_4 0xf0

enum { M_ALIENS, M_CRIMFGHT, M_BLOCKHL, M_SCONTRA, M_THUNDERX, M_GBUSTERS, M_88GAMES };
#define TX (s_mach >= M_SCONTRA && s_mach <= M_GBUSTERS)   // (the thunderx.cpp family)
static int s_mach, s_vis_x0, s_vis_w;
static uint8_t *s_rom, *s_zrom, *s_sprrom;
static uint32_t s_rom_size, s_zrom_size, s_sprrom_size;
static konami_t s_cpu;
static z80 s_z;
static const uint8_t *s_zmap[256];
static k052109_t s_k109;
static k007232_t s_k232;
static k051316_t s_k316;      // ('88 Games)
static upd7759_t s_upd[2];
static char s_nv_path[256];  // ('88 Games: 3000-37ff battery RAM)
#define UPD_CLOCK 640000u
static int s_port_p[4], s_port_dsw[3], s_port_system;

static struct {
	uint8_t pal8[0x1000];  // (the palette's view: xBGR_555, the high byte first; 88games 2048 entries)
	uint8_t ram[0x2000];   // aliens, crimfght 0000-1fff, blockhl 4000-5fff
	uint8_t k51960_ram[0x400], k51960_buffer[0x400];
	uint8_t k51960_control, k51960_shadow, k51960_rombank[3], k51960_romoffset;
	int64_t k51960_busy_until;
	uint8_t mainbank, pal_view, woco, rmrd, init;
	uint8_t pmcram[0x800], view5800, latch1f98, priority;   // (thunderx.cpp: 5800-5fff 0 palette, 1 PMC, 2 RAM)
	int64_t firq_at;   // (thunderx: the PMC's FIRQ after its program, -1 none)
	uint8_t videobank, zoomreadroms, speech_chip;   // ('88 Games)
	int32_t nmi_lines;  // (thunderx: lines to the K052109's next NMI, every 32 lines from vpos 0 at power on)
	uint8_t zram[0x800], soundlatch, z80_irq;
	uint64_t k232_frac;
	int32_t k232_a, k232_b;   // (the K007232's last sample, held between its own)
	int64_t cycles_done;
	int64_t cpu_end;   // (the CPU's line target: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, z_frac, z_end, ym_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot (screen_update) ----
static struct { uint8_t k51960_buffer[0x400], k51960_control, k51960_shadow, priority; k052109_t k109; k051316_t k316; } s_vs;
static k052109_view_t s_view;

static void tile_cb(int layer, int bank, int *code, int *color, int *flags, int *priority)
{
	(void)priority;
	const int c = *color;
	switch (s_mach) {
	case M_ALIENS:
		*code |= ((c & 0x3f) << 8) | (bank << 14);
		*color = layer * 4 + ((c & 0xc0) >> 6);
		break;
	case M_CRIMFGHT:
		*flags = (c & 0x20) ? K052109_TILE_FLIPX : 0;
		*code |= ((c & 0x1f) << 8) | (bank << 13);
		*color = layer * 4 + ((c & 0xc0) >> 6);
		break;
	case M_BLOCKHL: {   // (layer colour bases 0, 16, 32)
		static const int base[3] = { 0, 16, 32 };
		*code |= (c & 0x0f) << 8;
		*color = base[layer] + ((c & 0xe0) >> 5);
		break;
	}
	case M_88GAMES: {   // (layer colour bases 1024, 0, 256)
		static const int base[3] = { 64, 0, 16 };
		*code |= ((c & 0x0f) << 8) | (bank << 12);
		*color = base[layer] + ((c & 0xf0) >> 4);
		break;
	}
	default: {   // (thunderx.cpp: layer colour bases 768, 0, 256; gbusters_tile_callback)
		static const int base[3] = { 48, 0, 16 };
		if (s_mach == M_GBUSTERS)
			*code |= ((c & 0x0d) << 8) | ((c & 0x10) << 5) | (bank << 12);
		else
			*code |= ((c & 0x1f) << 8) | (bank << 13);
		*color = base[layer] + ((c & 0xe0) >> 5);
		break;
	}
	}
}

static void sprite_cb(int *code, int *color, int *priority, int *shadow)
{
	const int c = *color;
	if (s_mach == M_88GAMES) {   // (the priority range of k051960_sprites_draw: 0 or 1)
		*priority = (c & 0x20) >> 5;
		*color = 512 / 16 + (c & 0x0f);
		return;
	}
	if (TX) {   // (1 behind the background, 3 not drawn)
		static const int pm[4] = { 0, PMASK_2 | PMASK_1, PMASK_2, 0xff };
		*priority = pm[(c & 0x30) >> 4];
		*color = 512 / 16 + (c & 0x0f);
		return;
	}
	if (s_mach == M_BLOCKHL) {
		*priority = (c & 0x10) ? PMASK_1 : 0;
		*color = 768 / 16 + (c & 0x0f);
		return;
	}
	switch (c & 0x70) {   // (the PROM's mixed priorities)
	case 0x10: *priority = 0; break;                                // over ABF
	case 0x00: *priority = PMASK_4; break;                          // over AB, not F
	case 0x40: *priority = PMASK_4 | PMASK_2; break;                // over A, not BF
	case 0x20: case 0x60: *priority = PMASK_4 | PMASK_2 | PMASK_1; break;
	case 0x50: *priority = PMASK_2; break;                          // over AF, not B
	default: *priority = PMASK_2 | PMASK_1; break;                  // (0x30, 0x70) over F, not AB
	}
	if (s_mach == M_ALIENS) {
		*code |= (c & 0x80) << 6;
		*shadow = 0;   // (shadows not used)
	}
	*color = 256 / 16 + (c & 0x0f);
}

static void capture(void)
{
	for (int i = 0; i < gx.pal_entries; i++)   // (the palette: xBGR_555 of two bytes, the high one first)
		((uint16_t *)gx.palram)[i] = (uint16_t)(t.pal8[2 * i] << 8 | t.pal8[2 * i + 1]);
	memcpy(s_vs.k51960_buffer, t.k51960_buffer, sizeof(s_vs.k51960_buffer));
	s_vs.k51960_control = t.k51960_control;
	s_vs.k51960_shadow = t.k51960_shadow;
	s_vs.priority = t.priority;
	memcpy(&s_vs.k109, &s_k109, offsetof(k052109_t, state_end));
	if (s_mach == M_88GAMES)
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
	memset(p, 0, (size_t)(s_vis_w * VIS_H));
#define LAYER(l, fl, pr) k052109_draw(&s_vs.k109, &s_view, l, fl, pr, f, p, s_vis_w, x0, VIS_Y0, x0, VIS_Y0, x1, y1)
	if (s_mach == M_88GAMES) {
		k051316_t *z = &s_vs.k316;
		z->rom = s_k316.rom;
		z->rom_size = s_k316.rom_size;
		z->gfx = s_k316.gfx;
		z->ntiles = s_k316.ntiles;
		z->pix = s_k316.pix;   // (the drawing's cache, the render thread's own)
		z->key = s_k316.key;
		z->cb = s_k316.cb;
		z->bpp = s_k316.bpp;
#define ZOOM() k051316_zoom_draw(z, f, p, s_vis_w, x0, VIS_Y0, x0, VIS_Y0, x1, y1, 0, 0)
#define SPR(n) gx_pre_sprites_051960(s_vs.k51960_buffer, s_vs.k51960_control, s_vs.k51960_shadow, n, n, sprite_cb, 0)
		if (s_vs.priority) {
			LAYER(0, K052109_DRAW_OPAQUE, 0);
			SPR(1);
			LAYER(2, 0, 0);
			LAYER(1, 0, 0);
			SPR(0);
			ZOOM();
		} else {
			LAYER(2, K052109_DRAW_OPAQUE, 0);
			ZOOM();
			SPR(0);
			LAYER(1, 0, 0);
			SPR(1);
			LAYER(0, 0, 0);
		}
#undef ZOOM
#undef SPR
		return;
	}
	if (TX) {
		LAYER(1, K052109_DRAW_OPAQUE, 0);
		LAYER(s_vs.priority ? 2 : 1, 0, 1);
		LAYER(s_vs.priority ? 1 : 2, 0, 2);
		gx_pre_sprites_051960(s_vs.k51960_buffer, s_vs.k51960_control, s_vs.k51960_shadow, -1, -1, sprite_cb, 0);
		LAYER(0, 0, 0);
		return;
	}
	if (s_mach == M_BLOCKHL) {
		LAYER(2, K052109_DRAW_OPAQUE, 0);
		LAYER(1, 0, 1);
		gx_pre_sprites_051960(s_vs.k51960_buffer, s_vs.k51960_control, s_vs.k51960_shadow, -1, -1, sprite_cb, 0);
		LAYER(0, 0, 0);
		return;
	}
	LAYER(1, K052109_DRAW_OPAQUE, 0);
	LAYER(1, 0, 1);
	LAYER(2, 0, 2);
	LAYER(0, 0, 4);
	gx_pre_sprites_051960(s_vs.k51960_buffer, s_vs.k51960_control, s_vs.k51960_shadow, -1, -1, sprite_cb, 0);
#undef LAYER
}

// ---- the K051960 / K051937 ----
static inline int64_t now_cycles(void) { return t.cycles_done + s_cpu.cycles; }

static void irq_lines(void)   // (blockhl, thunderx.cpp, 88games: the K052109's IRQ (thunderx its NMI); the others the K051960's)
{
	if (s_mach == M_BLOCKHL || TX || s_mach == M_88GAMES)
		konami_set_irq(&s_cpu, s_k109.irq_out);
	if (s_mach == M_THUNDERX)
		konami_set_nmi(&s_cpu, s_k109.nmi_out);
}

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
	if (offset == 0) {   // (the interrupts cleared when their enable goes off; the IRQ the only one connected here)
		if (~v & t.k51960_control & 1 && s_mach != M_BLOCKHL && !TX && s_mach != M_88GAMES)
			konami_set_irq(&s_cpu, 0);
		t.k51960_control = v;
	} else if (offset == 1)
		t.k51960_shadow = v & 0x07;
	else if (offset < 5)
		t.k51960_rombank[offset - 2] = v;
}
static void k051960_vblank(void)
{
	if ((t.k51960_control & 1) && s_mach != M_BLOCKHL && !TX && s_mach != M_88GAMES)   // (the vblank IRQ)
		konami_set_irq(&s_cpu, 1);
	if (t.k51960_control & 0x10)
		return;
	memcpy(t.k51960_buffer, t.k51960_ram, sizeof(t.k51960_buffer));
	int active = 0;
	for (int i = 0; i < 0x400; i += 8)
		active += t.k51960_buffer[i] >> 7;
	const uint32_t ticks = (uint32_t)(active * 32 + (128 - active) * 18);
	t.k51960_busy_until = now_cycles() + (int64_t)((uint64_t)ticks * 4 * CPU_CLOCK / 24000000u);
}

// the K052109 / K051960 window (k052109_051960_r / _w)
static uint8_t vid_r(int o)
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

// ---- the sound board a frame behind on the worker (rbx_aux): the latch and the IRQ queued with the samples ----
#define Q_MAX 2048
enum { Q_ADV, Q_LATCH, Q_IRQ };
typedef struct { uint16_t kind, arg; uint32_t v; } q_ev_t;
static q_ev_t (*s_q_hp)[2][Q_MAX];
#define s_q (*s_q_hp)
static int s_qn[2], s_qcur;
static int32_t s_out_l[2048], s_out_r[2048];
static int s_out_n, s_out_pos;
static volatile int s_job_frame;

static void q_put(int kind, uint32_t v)
{
	const int c = s_qcur;
	if (s_qn[c] < Q_MAX) {
		s_q[c][s_qn[c]].kind = (uint16_t)kind;
		s_q[c][s_qn[c]].v = v;
		s_qn[c]++;
	}
}

// ---- the main CPU's bus ----
static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }

static uint8_t al_rd(void *ctx, uint16_t a)   // (aliens main_map)
{
	(void)ctx;
	if (a < 0x2000)
		return (a < 0x400 && t.pal_view) ? t.pal8[a] : t.ram[a];
	if (a < 0x4000)
		return s_rom[((uint32_t)t.mainbank * 0x2000 + (a - 0x2000)) % s_rom_size];
	if (a < 0x8000) {
		switch (a) {
		case 0x5f80: return port(s_port_dsw[2]);
		case 0x5f81: return port(s_port_p[0]);
		case 0x5f82: return port(s_port_p[1]);
		case 0x5f83: return port(s_port_dsw[1]);
		case 0x5f84: return port(s_port_dsw[0]);
		case 0x5f88: return 0;   // (watchdog)
		}
		return vid_r(a - 0x4000);
	}
	return s_rom[0x28000 + (a - 0x8000)];
}
static void al_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x2000) {
		if (a < 0x400 && t.pal_view)
			t.pal8[a] = v;
		else
			t.ram[a] = v;
		return;
	}
	if (a < 0x4000 || a >= 0x8000)
		return;
	if (a == 0x5f88) {   // (coin_counter_w: bit 5 the palette's view, bit 6 the char ROM's reading)
		t.pal_view = (v >> 5) & 1;
		k052109_set_rmrd_line(&s_k109, (v >> 6) & 1);
		return;
	}
	if (a == 0x5f8c) {   // (the sound latch: its data pending the Z80's IRQ)
		q_put(Q_LATCH, v);
		q_put(Q_IRQ, 0);
		return;
	}
	vid_w(a - 0x4000, v);
}

static uint8_t cf_rd(void *ctx, uint16_t a)   // (crimfght main_map)
{
	(void)ctx;
	if (a < 0x2000)
		return (a < 0x400 && t.woco) ? t.pal8[a] : t.ram[a];
	if (a < 0x6000) {
		if (a >= 0x3f80 && a < 0x3f90) {
			switch (a) {
			case 0x3f80: return port(s_port_system);
			case 0x3f81: return port(s_port_p[0]);
			case 0x3f82: return port(s_port_p[1]);
			case 0x3f83: return port(s_port_dsw[1]);
			case 0x3f84:   // (DSW3: bits 4-7 system_r: VCC, WOCO, RMRD, INIT)
				return (uint8_t)((port(s_port_dsw[2]) & 0x0f) | 0x10 | t.woco << 5 | t.rmrd << 6 | t.init << 7);
			case 0x3f85: return port(s_port_p[2]);
			case 0x3f86: return port(s_port_p[3]);
			case 0x3f87: return port(s_port_dsw[0]);
			}
			if (a < 0x3f8c)
				return 0;   // (watchdog)
		}
		return vid_r(a - 0x2000);
	}
	if (a < 0x8000)
		return s_rom[((uint32_t)t.mainbank * 0x2000 + (a - 0x6000)) % s_rom_size];
	return s_rom[0x18000 + (a - 0x8000)];
}
static void cf_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x2000) {
		if (a < 0x400 && t.woco)
			t.pal8[a] = v;
		else
			t.ram[a] = v;
		return;
	}
	if (a >= 0x6000)
		return;
	if (a >= 0x3f88 && a < 0x3f8c)   // (coin_w)
		return;
	if (a >= 0x3f8c && a < 0x3f90) {   // (sound_w: the latch, the Z80's IRQ until acknowledged)
		q_put(Q_LATCH, v);
		q_put(Q_IRQ, 0);
		return;
	}
	vid_w(a - 0x2000, v);
}

static uint8_t bh_rd(void *ctx, uint16_t a)   // (blockhl main_map)
{
	(void)ctx;
	if (a < 0x4000) {
		switch (a) {
		case 0x1f94: return port(s_port_dsw[2]);
		case 0x1f95: return port(s_port_p[0]);
		case 0x1f96: return port(s_port_p[1]);
		case 0x1f97: return port(s_port_dsw[0]);
		case 0x1f98: return port(s_port_dsw[1]);
		}
		return vid_r(a);
	}
	if (a < 0x6000)
		return (a >= 0x5800 && t.pal_view) ? t.pal8[a - 0x5800] : t.ram[a - 0x4000];
	if (a < 0x8000)
		return s_rom[((uint32_t)t.mainbank * 0x2000 + (a - 0x6000)) % s_rom_size];
	return s_rom[0x8000 + (a - 0x8000)];
}
static void bh_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x4000) {
		switch (a) {
		case 0x1f84: q_put(Q_LATCH, v); return;
		case 0x1f88: q_put(Q_IRQ, 0); return;   // (sound_irq_w: HOLD_LINE)
		case 0x1f8c: return;                    // (watchdog)
		}
		vid_w(a, v);
		return;
	}
	if (a < 0x6000) {
		if (a >= 0x5800 && t.pal_view)
			t.pal8[a - 0x5800] = v;
		else
			t.ram[a - 0x4000] = v;
	}
}

// -- 88games.cpp (main_map) --
static void zoom_cb(int chip, int *code, int *color)   // (zoom_callback)
{
	(void)chip;
	const int c = *color;
	*code |= (c & 0x07) << 8;
	*color = 768 / 16 + ((c & 0x38) >> 3) + ((c & 0x80) >> 4);
}

static uint8_t g88_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x2000) {   // (the banked ROM; 1000-1fff the palette with its view)
		if (a >= 0x1000 && t.pal_view)
			return t.pal8[a - 0x1000];
		return s_rom[(0x10000u + (uint32_t)t.mainbank * 0x2000 + a) % s_rom_size];
	}
	if (a < 0x4000) {
		if (a >= 0x3800 && !t.videobank)   // (the K051316's RAM, or its ROM through rom_r)
			return t.zoomreadroms ? k051316_rom_r(&s_k316, a - 0x3800) : s_k316.ram[a - 0x3800];
		return t.ram[a - 0x2000];
	}
	if (a < 0x8000) {
		switch (a) {
		case 0x5f94: return port(s_port_p[0]);   // (IN0, IN1, IN2)
		case 0x5f95: return port(s_port_p[1]);
		case 0x5f96: return port(s_port_p[2]);
		case 0x5f97: return port(s_port_dsw[0]);
		case 0x5f9b: return port(s_port_dsw[1]);
		}
		return vid_r(a - 0x4000);
	}
	return s_rom[a];
}
static void g88_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x2000) {
		if (a >= 0x1000)   // (the palette's writes whatever the view)
			t.pal8[a - 0x1000] = v;
		return;
	}
	if (a < 0x4000) {
		if (a >= 0x3800 && !t.videobank)
			k051316_write(&s_k316, a - 0x3800, v);
		else
			t.ram[a - 0x2000] = v;
		return;
	}
	if (a >= 0x8000)
		return;
	if (a >= 0x5fc0 && a < 0x5fd0) {
		k051316_ctrl_w(&s_k316, a - 0x5fc0, v);
		return;
	}
	switch (a) {
	case 0x5f84: t.zoomreadroms = (v >> 2) & 1; return;   // (_5f84_w)
	case 0x5f88: return;                                 // (watchdog)
	case 0x5f8c: q_put(Q_LATCH, v); return;
	case 0x5f90: q_put(Q_IRQ, 0); return;                // (sh_irqtrigger_w: HOLD_LINE)
	}
	vid_w(a - 0x4000, v);
}

// -- thunderx.cpp (scontra_map, thunderx_map, gbusters_map) --
static void map_bank(void);

static void pmc_run(void)   // (the 052591's collision program: MAME's pmc_run)
{
	uint8_t *m = t.pmcram;
	const int e0 = m[0] << 8 | m[1], e1 = m[2];
	int s0, s1, p0_lim;
	if (m[5] < 16) {
		s0 = m[5] << 8 | m[6];
		s1 = m[7];
		p0_lim = 0x136;
	} else {
		s0 = m[5];
		s1 = m[6];
		p0_lim = 0xe6;
	}
	const uint8_t cm = m[3], hm = m[4];
	for (int a0 = s0; a0 < e0 && a0 + 4 < 0x800; a0 += 5) {
		uint8_t *p0 = m + a0;
		if (!(p0[0] & cm))
			continue;
		for (int a1 = s1; a1 < e1; a1 += 5) {
			uint8_t *p1 = m + a1;
			if (!(p1[0] & hm))
				continue;
			if (p1[1] + p0[1] < abs(p1[3] - p0[3]))
				continue;
			if (p1[2] + p0[2] < abs(p1[4] - p0[4]))
				continue;
			p1[0] = (uint8_t)((p1[0] & 0x8f) | 0x10);
			if (a0 + 4 >= p0_lim)   // (hardcoded in the PMC program)
				p0[0] = (uint8_t)((p0[0] & 0x9b) | (p1[0] & 0x04) | 0x10);
			break;
		}
	}
	t.firq_at = now_cycles() + 100;   // (thunderx_firq_cb: HOLD_LINE 100 cycles on)
}

static uint8_t tx_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x4000) {
		switch (a) {
		case 0x1f90: return port(s_port_system);
		case 0x1f91: return port(s_port_p[0]);
		case 0x1f92: return port(s_port_p[1]);
		case 0x1f93: return port(s_port_dsw[2]);
		case 0x1f94: return port(s_port_dsw[0]);
		case 0x1f95: return port(s_port_dsw[1]);
		case 0x1f98: return t.latch1f98;
		}
		return vid_r(a);
	}
	if (a < 0x6000) {
		if (a >= 0x5800 && t.view5800 == 0)
			return t.pal8[a - 0x5800];
		if (a >= 0x5800 && t.view5800 == 1)   // (pmc_r: readable with pmc_bk, 1f98 bit 1)
			return (t.latch1f98 & 2) ? t.pmcram[a - 0x5800] : 0;
		return t.ram[a - 0x4000];
	}
	if (a < 0x8000) {   // (entries 0-7 the ROM's first 8 KB pages again and again, 8-15 its 10000-1ffff)
		const uint32_t e = t.mainbank;
		return s_rom[((e < 8 ? (e & 3) * 0x2000u : 0x10000u + (e - 8) * 0x2000u) + (a - 0x6000)) % s_rom_size];
	}
	return s_rom[a];
}
static void tx_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x4000) {
		switch (a) {
		case 0x1f80:   // (scontra_bankswitch_w, thunderx_videobank_w, gbusters_videobank_w)
			if (s_mach == M_SCONTRA) {
				t.mainbank = v & 0x0f;
				t.view5800 = (v & 0x10) ? 2 : 0;
				t.priority = (v >> 7) & 1;
			} else {
				t.view5800 = (s_mach == M_THUNDERX && (v & 0x10)) ? 1 : (v & 1) ? 2 : 0;
				t.priority = (v >> 3) & 1;
			}
			map_bank();
			return;
		case 0x1f84: q_put(Q_LATCH, v); return;
		case 0x1f88: q_put(Q_IRQ, 0); return;   // (sh_irqtrigger_w: HOLD_LINE)
		case 0x1f8c: return;                    // (watchdog)
		case 0x1f98:   // (1f98_w: bit 0 the char ROM's reading; thunderx bit 2 rising runs the PMC)
			if (s_mach == M_THUNDERX && (v & 4) && !(t.latch1f98 & 4))
				pmc_run();
			k052109_set_rmrd_line(&s_k109, v & 1);
			t.latch1f98 = v;
			return;
		}
		vid_w(a, v);
		return;
	}
	if (a < 0x6000) {
		if (a >= 0x5800 && t.view5800 == 0)
			t.pal8[a - 0x5800] = v;
		else if (a >= 0x5800 && t.view5800 == 1) {
			if (t.latch1f98 & 2)   // (pmc_w: pmc_bk; else the PMC's start address)
				t.pmcram[a - 0x5800] = v;
		} else
			t.ram[a - 0x4000] = v;
	}
}

static void map_bank(void)   // (the banked ROM's direct pages)
{
	if (s_mach == M_88GAMES) {   // (0000-1fff: 1000-1fff the palette's reads with its view)
		const uint8_t *b = s_rom + 0x10000 + (uint32_t)t.mainbank * 0x2000;
		for (int p = 0; p < 0x20; p++)
			s_cpu.rpage[p] = (p < 0x10 || !t.pal_view) ? b + p * 256 : NULL;
		return;
	}
	const uint16_t at = s_mach == M_ALIENS ? 0x2000 : 0x6000;
	uint32_t base = ((uint32_t)t.mainbank * 0x2000) % s_rom_size;
	if (TX)
		base = t.mainbank < 8 ? (t.mainbank & 3u) * 0x2000u : 0x10000u + (t.mainbank - 8u) * 0x2000u;
	for (uint32_t p = 0; p < 0x20; p++)
		s_cpu.rpage[(at >> 8) + p] = base + p * 256 + 256 <= s_rom_size ? s_rom + base + p * 256 : NULL;
}

static void set_lines(void *ctx, uint8_t data)   // (the banking callbacks)
{
	(void)ctx;
	switch (s_mach) {
	case M_ALIENS:
		t.mainbank = (uint8_t)((data & 0x1f) < 24 ? data & 0x1f : 0);   // (set_membank(...).mask(0x1f): 24 entries)
		break;
	case M_CRIMFGHT:
		t.mainbank = data & 0x0f;
		t.woco = (data >> 5) & 1;
		t.rmrd = (data >> 6) & 1;
		k052109_set_rmrd_line(&s_k109, t.rmrd);
		t.init = (data >> 7) & 1;
		break;
	case M_SCONTRA:
		break;   // (the line not connected: the bank by 1f80)
	case M_88GAMES:
		t.mainbank = data & 0x07;
		t.pal_view = (data >> 3) & 1;
		t.videobank = (data >> 4) & 1;
		k052109_set_rmrd_line(&s_k109, (data >> 5) & 1);
		t.priority = (data >> 7) & 1;
		break;
	case M_THUNDERX: case M_GBUSTERS:
		t.mainbank = data & 0x0f;   // (set_membank(...).mask(0x0f))
		break;
	default:
		t.mainbank = data & 0x03;
		t.pal_view = !((data >> 5) & 1);
		k052109_set_rmrd_line(&s_k109, (data >> 6) & 1);
		break;
	}
	map_bank();
}

// ---- the sound board (sound_map) ----
static void k007232_port(uint8_t data)   // (volume_callback; scontra's channels the other way round)
{
	const int lo = (data & 0x0f) * 0x11, hi = (data >> 4) * 0x11;
	k007232_set_volume(&s_k232, 0, TX ? hi : lo, 0);
	k007232_set_volume(&s_k232, 1, 0, TX ? lo : hi);
}
static void ym_ct(uint8_t data) { k007232_set_bank(&s_k232, (data >> 1) & 1, data & 1); }   // (snd_bankswitch_w)

static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	if (s_mach == M_88GAMES) {
		if (a < 0x8800)
			return t.zram[a - 0x8000];
		if (a == 0xa000)
			return t.soundlatch;
		if (a == 0xc000 || a == 0xc001)
			return rbx_ym_read(0, a & 1);
		return 0;
	}
	if (s_mach == M_BLOCKHL || TX) {   // (thunderx_sound_map; scontra_sound_map the K007232 at b000)
		if (a < 0x8800)
			return t.zram[a - 0x8000];
		if (a == 0xa000)
			return t.soundlatch;
		if (a >= 0xb000 && a < 0xb00e && TX && s_mach != M_THUNDERX)
			return k007232_read(&s_k232, a - 0xb000);
		if (a == 0xc000 || a == 0xc001)
			return rbx_ym_read(0, a & 1);
		return 0;
	}
	switch (a >> 13) {   // (crimfght's mirrors: 8000-9fff RAM, a000 YM2151, c000 latch, e000 K007232)
	case 4: return t.zram[a & 0x7ff];
	case 5: return rbx_ym_read(0, a & 1);
	case 6: return t.soundlatch;
	default: return k007232_read(&s_k232, a & 0x0f);
	}
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x8000)
		return;
	if (s_mach == M_88GAMES) {
		if (a < 0x8800)
			t.zram[a - 0x8000] = v;
		else if (a == 0x9000)   // (speech_msg_w)
			upd7759_port_w(&s_upd[t.speech_chip], v);
		else if (a == 0xc000 || a == 0xc001)
			rbx_ym_write(0, a & 1, v);
		else if (a == 0xe000) {   // (speech_control_w)
			t.speech_chip = (v >> 2) & 1;
			upd7759_reset_w(&s_upd[t.speech_chip], (v >> 1) & 1);
			upd7759_start_w(&s_upd[t.speech_chip], !(v & 1));
		}
		return;
	}
	if (s_mach == M_BLOCKHL || TX) {
		if (a < 0x8800)
			t.zram[a - 0x8000] = v;
		else if (a == 0xc000 || a == 0xc001)
			rbx_ym_write(0, a & 1, v);
		else if (TX && s_mach != M_THUNDERX && a >= 0xb000 && a < 0xb00e)
			k007232_write(&s_k232, a - 0xb000, v);
		else if (TX && s_mach != M_THUNDERX && a == 0xf000)   // (k007232_bankswitch_w)
			k007232_set_bank(&s_k232, v & 3, (v >> 2) & 3);
		return;
	}
	switch (a >> 13) {
	case 4: t.zram[a & 0x7ff] = v; break;
	case 5: rbx_ym_write(0, a & 1, v); break;
	case 6: break;
	default: k007232_write(&s_k232, a & 0x0f, v); break;
	}
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// n samples (a line's): the Z80 for their cycles, the YM2151 (its samples spread over them), the K007232 at its rate
// (held between). aliens, blockhl mono (YM2151 0.60, K007232 0.20); crimfght YM2151 1.0 stereo, K007232 0.20 to both
static void sound_run(int n)
{
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
	int32_t yl[96], yr[96];
	memset(yl, 0, sizeof(yl));
	memset(yr, 0, sizeof(yr));
	t.ym_frac += (uint64_t)CHIP_CLOCK * (uint64_t)n;
	const uint32_t yc = (uint32_t)(t.ym_frac / SND_RATE);
	t.ym_frac -= (uint64_t)yc * SND_RATE;
	const int yn = rbx_ym_advance_lr(0, yc, yl, yr, 96);
	int32_t up0[64], up1[64];
	if (s_mach == M_88GAMES) {
		upd7759_render(&s_upd[0], up0, n);
		upd7759_render(&s_upd[1], up1, n);
	}
	for (int i = 0; i < n; i++) {
		if (s_mach == M_88GAMES) {   // (YM2151 0.75 (its gain), the uPD7759s 0.30)
			const int k = yn > 0 ? i * yn / n : -1;
			if (s_out_pos < 2048) {
				s_out_l[s_out_pos] = s_out_r[s_out_pos] = (k >= 0 ? yl[k] + yr[k] : 0) + (up0[i] + up1[i]) * 30 / 100;
				s_out_pos++;
			}
			continue;
		}
		if (s_mach != M_BLOCKHL && s_mach != M_THUNDERX) {
			t.k232_frac += CHIP_CLOCK / 128;   // (its rate: clock / 128)
			while (t.k232_frac >= SND_RATE) {
				t.k232_frac -= SND_RATE;
				k007232_sample(&s_k232, &t.k232_a, &t.k232_b);
			}
		}
		const int k = yn > 0 ? i * yn / n : -1;
		const int32_t l = k >= 0 ? yl[k] : 0, r = k >= 0 ? yr[k] : 0, pcm = (t.k232_a + t.k232_b) / (TX ? 10 : 5);
		if (s_out_pos >= 2048)
			continue;
		if (s_mach == M_CRIMFGHT) {
			s_out_l[s_out_pos] = l + pcm;
			s_out_r[s_out_pos] = r + pcm;
		} else
			s_out_l[s_out_pos] = s_out_r[s_out_pos] = l + r + pcm;   // (the YM2151's gains set at init)
		s_out_pos++;
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
		case Q_ADV: sound_run((int)e->v); break;
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
		rbx_audio_put_stereo(s_out_l[i], s_out_r[i]);
	s_out_n = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void k8b_exit(void);
static void k8b_reset(void);

static int k8b_init(const rbx_game *g)
{
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	static const char *const names[] = { "aliens", "crimfght", "blockhl", "scontra", "thunderx", "gbusters", "_88games" };
	s_mach = M_ALIENS;
	for (int i = 0; i < 7; i++)
		if (!strcmp(g->machine, names[i]))
			s_mach = i;
	uint32_t tile_size, pcm_size = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *tiles = rbx_region("k052109", &tile_size);
	s_sprrom = rbx_region("k051960", &s_sprrom_size);
	const int has232 = s_mach != M_BLOCKHL && s_mach != M_THUNDERX && s_mach != M_88GAMES;
	const uint8_t *pcm = has232 ? rbx_region("k007232", &pcm_size) : NULL;
	uint32_t zsize = 0, u1size = 0, u2size = 0;
	const int g88 = s_mach == M_88GAMES;
	const uint8_t *zrom = g88 ? rbx_region("k051316", &zsize) : NULL;
	const uint8_t *u1 = g88 ? rbx_region("upd1", &u1size) : NULL, *u2 = g88 ? rbx_region("upd2", &u2size) : NULL;
	if (!s_rom || !s_zrom || !tiles || !s_sprrom || (has232 && !pcm) || (g88 && (!zrom || !u1 || !u2))) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	const int wide = s_mach == M_CRIMFGHT || TX || g88;   // (set_raw(..., 0, 320, ...); the others 0+16, 320-16)
	s_vis_x0 = wide ? 0 : 16;
	s_vis_w = wide ? 320 : 288;
	memset(&gx, 0, sizeof(gx));
	memset(&t, 0, sizeof(t));
	gx.pal_entries = g88 ? 2048 : s_mach == M_BLOCKHL || TX ? 1024 : 512;
	gx.pal555 = 1;
	gx.tilecb = GX_TILECB_PRE;
	gx.mixmode = GX_MIXMODE_PRE;
	gx.pre_capture = capture;
	gx.pre_draw = draw;
	gx.pre_screen_w = HTOTAL;
	gx.pre_dim = 1;
	for (int i = 0; i < 8; i++)
		gx.pre_contrast[i] = 1.0f;
	gx.pre_shadow_factor = TX ? 7.0f / 8.0f : 0;   // (thunderx.cpp machine_start: set_shadow_factor(7/8))
	k055673_init(&gx.k55673, s_sprrom, s_sprrom_size, K055673_LAYOUT_051960, 0, 0);
	gx_video_set_area(s_vis_x0, s_vis_w);
	gx_video_set_y0(VIS_Y0);
	gx_video_set_height(VIS_H);
	if (k052109_init(&s_k109, tiles, tile_size, gx.pal_entries, tile_cb)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	k007232_init(&s_k232, pcm, pcm_size, k007232_port);
	if (g88) {
		if (k051316_init(&s_k316, 0, zrom, zsize, 0, 0, zoom_cb)) {
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
		upd7759_init(&s_upd[0], u1, u1size, UPD_CLOCK, SND_RATE);
		upd7759_init(&s_upd[1], u2, u2size, UPD_CLOCK, SND_RATE);
	}
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = CHIP_CLOCK;
	yc.gain[0] = yc.gain[1] = s_mach == M_CRIMFGHT ? 256 : TX ? 128 : g88 ? 192 : 154;   // (crimfght 1.0, thunderx.cpp 0.5, 88games 0.75, the others 0.60)
	if (s_mach == M_ALIENS || s_mach == M_CRIMFGHT)
		yc.port_a_write = ym_ct;   // (the CT lines: the K007232's banks)
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: YM2151", g->set);
		return -1;
	}
	static const char *const pn[4] = { "P1", "P2", "P3", "P4" }, *const dn[3] = { "DSW1", "DSW2", "DSW3" };
	for (int i = 0; i < 4; i++)
		s_port_p[i] = rbx_port_find(pn[i]);
	for (int i = 0; i < 3; i++)
		s_port_dsw[i] = rbx_port_find(dn[i]);
	s_port_system = rbx_port_find("SYSTEM");
	if (g88) {   // (IN0-IN2 at 5f94)
		for (int i = 0; i < 3; i++) {
			static const char *const in[3] = { "IN0", "IN1", "IN2" };
			s_port_p[i] = rbx_port_find(in[i]);
		}
	}
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(s_vis_w, VIS_H, 0x10000) || gx_video_init()) {
		rbx_fail("%s: out of memory", g->set);
		k8b_exit();
		return -1;
	}
	for (int i = 0; i < 0x10000; i++) {   // (the bitmap: RGB 565)
		const int r = (i >> 11) & 31, gg = (i >> 5) & 63, b = i & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (gg << 2 | gg >> 4) << 8 | (b << 3 | b >> 2));
	}
	k8b_reset();
	s_nv_path[0] = 0;
	if (g88) {   // (the battery RAM 3000-37ff: DEFAULT_ALL_0, kept between runs)
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			FILE *f = fopen(s_nv_path, "rb");
			if (f) {
				if (fread(t.ram + 0x1000, 1, 0x800, f) != 0x800)
					memset(t.ram + 0x1000, 0, 0x800);
				fclose(f);
			}
		}
	}
	rbx_aux_set_job(job);
	return 0;
}

static void k8b_exit(void)
{
	if (s_nv_path[0]) {
		FILE *f = fopen(s_nv_path, "wb");
		if (f) {
			fwrite(t.ram + 0x1000, 1, 0x800, f);
			fclose(f);
		}
		s_nv_path[0] = 0;
	}
	if (s_mach == M_88GAMES)
		k051316_exit(&s_k316);
	rbx_aux_set_job(NULL);
	gx_video_exit();
	k052109_exit(&s_k109);
	rbx_ym_exit(0);
	free(s_q_hp);
	s_q_hp = NULL;
}

static void map_pages(void)   // (the fixed ROM and the RAM as direct pages; the windows with handlers left out)
{
	memset(s_cpu.rpage, 0, sizeof(s_cpu.rpage));
	memset(s_cpu.wpage, 0, sizeof(s_cpu.wpage));
	const uint32_t fix = s_mach == M_ALIENS ? 0x28000 : s_mach == M_CRIMFGHT ? 0x18000 : 0x8000;
	for (int p = 0x80; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + fix + (uint32_t)(p - 0x80) * 256;
	if (s_mach == M_88GAMES)
		for (int p = 0x20; p < 0x38; p++)   // (2000-37ff: 3800-3fff the K051316's view)
			s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + (p - 0x20) * 256;
	else if (s_mach == M_BLOCKHL || TX)
		for (int p = 0x40; p < 0x58; p++)   // (4000-57ff: 5800-5fff the palette's view)
			s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + (p - 0x40) * 256;
	else
		for (int p = 0x04; p < 0x20; p++)   // (0400-1fff: 0000-03ff the palette's view)
			s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + p * 256;
	map_bank();
}

static void k8b_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	t.pal_view = s_mach == M_BLOCKHL;   // (blockhl machine_start: the palette's view selected)
	t.firq_at = -1;
	t.nmi_lines = LINES - (VIS_Y0 + VIS_H);   // (the frame starts at the vblank: vpos 0 that many lines on)
	k052109_reset(&s_k109);
	k007232_reset(&s_k232);
	if (s_mach == M_88GAMES) {
		k051316_reset(&s_k316);
		upd7759_reset(&s_upd[0]);
		upd7759_reset(&s_upd[1]);
	}
	rbx_ym_reset(0);
	konami_init(&s_cpu);
	static uint8_t (*const rd[7])(void *, uint16_t) = { al_rd, cf_rd, bh_rd, tx_rd, tx_rd, tx_rd, g88_rd };
	static void (*const wr[7])(void *, uint16_t, uint8_t) = { al_wr, cf_wr, bh_wr, tx_wr, tx_wr, tx_wr, g88_wr };
	s_cpu.read = rd[s_mach];
	s_cpu.write = wr[s_mach];
	s_cpu.set_lines = set_lines;
	s_cpu.ctx = NULL;
	map_pages();
	konami_reset(&s_cpu);
	z80_init(&s_z);
	s_z.read_byte = z_rd;
	s_z.write_byte = z_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x80; p++)
		s_zmap[p] = (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
	for (int p = 0; p < 8; p++)
		s_zmap[0x80 + p] = t.zram + p * 256;
	s_z.read_map = s_zmap;
}

static void k8b_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, the K051960's IRQ and sprite DMA, the K052109's IRQ)
			gx_video_capture();
			k051960_vblank();
			k052109_vblank(&s_k109, 1);
			irq_lines();
		}
		if (vpos == VIS_Y0) {
			k052109_vblank(&s_k109, 0);
			irq_lines();
		}
		if (s_mach == M_THUNDERX && --t.nmi_lines < 0) {   // (nmi_scanline: not on 16V, every 32 lines)
			t.nmi_lines = 31;
			if (s_k109.irq_control & 1) {
				s_k109.nmi_out = 1;
				irq_lines();
			}
		}
		t.cyc_frac += (uint64_t)CPU_CLOCK * HTOTAL;
		const int64_t cyc = (int64_t)(t.cyc_frac / PIX_CLOCK);
		t.cyc_frac -= (uint64_t)cyc * PIX_CLOCK;
		const int64_t end = t.cpu_end += cyc;
		while (t.firq_at >= 0 && t.firq_at < end) {   // (thunderx: the PMC's FIRQ inside the line)
			konami_run_until(&s_cpu, t.firq_at);
			t.firq_at = -1;
			konami_hold_firq(&s_cpu);
		}
		konami_run_until(&s_cpu, end);
		t.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(t.snd_frac / PIX_CLOCK);
		t.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		if (ns > 0)
			q_put(Q_ADV, (uint32_t)ns);
	}
	sound_frame();
	t.frame++;
	gx.frame = t.frame;
}

static int k8b_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "k8b_cpu", &s_cpu, KONAMI_STATE_SIZE);
	rc |= rbx_state_io(ctx, "gx", &gx, (uint32_t)offsetof(gx_t, state_end));
	rc |= k052109_state(&s_k109, ctx, load, "k8b_k109");
	rc |= rbx_state_io(ctx, "k8b", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= k007232_state(&s_k232, ctx, load, "k8b_k232");
	if (s_mach == M_88GAMES) {
		rc |= k051316_state(&s_k316, ctx, load, "k8b_k316");
		rc |= upd7759_state(&s_upd[0], ctx, load, "k8b_upd0");
		rc |= upd7759_state(&s_upd[1], ctx, load, "k8b_upd1");
	}
	rc |= rbx_ym_state(0, ctx, load);
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "k8b_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "k8b_outl", s_out_l, sizeof(s_out_l));
	rc |= rbx_state_io(ctx, "k8b_outr", s_out_r, sizeof(s_out_r));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "k8b_z80", &z, sizeof(z));
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
		map_bank();
	}
	return rc ? -1 : 0;
}

static int k8b_save(void *ctx) { return k8b_state(ctx, 0); }
static int k8b_load(void *ctx) { return k8b_state(ctx, 1); }
static int k8b_supported(const char *machine)
{
	return !strcmp(machine, "aliens") || !strcmp(machine, "crimfght") || !strcmp(machine, "blockhl") ||
	       !strcmp(machine, "scontra") || !strcmp(machine, "thunderx") || !strcmp(machine, "gbusters") ||
	       !strcmp(machine, "_88games");
}

const rbx_driver rbx_drv_aliens = { "Konami Aliens", k8b_init, k8b_exit, k8b_reset, k8b_frame, k8b_save, k8b_load,
                                    k8b_supported, gx_render, NULL };
