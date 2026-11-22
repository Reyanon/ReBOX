/*  Konami Teenage Mutant Ninja Turtles and M.I.A. - Missing in Action. Ported from MAME src/mame/konami/tmnt.cpp
    (BSD-3-Clause, copyright-holders: Nicola Salmoria) and the devices it uses (k052109.cpp, k051960.cpp: Fabio Priuli,
    Acho A. Tang, R. Belmont; k007232.cpp: Nicola Salmoria, Hiromitsu Shioya; upd7759.cpp: Juergen Buchmueller, Mike
    Balfour, Howie Cohen, Olivier Galibert, Aaron Giles). A 68000 at 8 MHz, the K052109 tilemaps, the K051960 sprites
    (drawn by konamigx_v.c's pre-GX mode: MAME's bitmap_ind16); the sound: a Z80 at 3.58 MHz, a YM2151, a K007232, and on
    TMNT a uPD7759 and the title music (MAME's samples from the title ROM's floating point data), a frame behind on the
    worker.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/k007232.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/upd7759.h"
#include "../video/k052109.h"
#include "../video/k055673.h"
#include "konamigx.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAIN_CLOCK 8000000u    // (24 MHz / 3)
#define Z80_CLOCK 3579545u
#define CHIP_CLOCK 3579545u    // (the YM2151 and the K007232)
#define UPD_CLOCK 640000u
#define SND_RATE 48000
#define PIX_CLOCK 6000000u     // (set_raw(24 MHz / 4, 384, x0, x1, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_Y0 16
#define VIS_H 224

static int s_tmnt;   // (tmnt_state's tmnt machine; else mia or cuebrick)
static int s_cue;    // (cuebrick: no sound CPU, the YM2151 on the 68000's bus, its IRQ 6; the NVRAM)
static uint8_t (*s_nv_hp)[0x8000];   // (cuebrick: 32 KB of NVRAM paged in a 1 KB window at 0b0000)
#define s_nv (*s_nv_hp)
static char s_nv_path[256];
static int s_vis_x0, s_vis_w;
static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static z80 s_z;
static const uint8_t *s_zmap[256];
static k052109_t s_k109;
static k007232_t s_k232;
static upd7759_t s_upd;
static int16_t *s_title;   // (the title music: 0x40000 samples at 640 kHz / 32)
static uint8_t *s_sprrom;
static uint32_t s_sprrom_size;
static int s_port_coins, s_port_p[4], s_port_dsw[3];

static struct {
	uint8_t pal8[0x800];   // 080000 (umask 00ff): the palette's bytes
	uint8_t k51960_ram[0x400], k51960_buffer[0x400];
	uint8_t k51960_control, k51960_shadow, k51960_rombank[3], k51960_romoffset;
	int64_t k51960_busy_until;
	uint8_t last, irq5_mask, priority, nvbank;
	uint8_t zram[0x800];
	uint8_t soundlatch, sres, z80_irq;
	uint32_t title_pos, title_frac;   // (the title music: playing while title_pos < 0x40000)
	uint8_t title_on;
	uint64_t k232_frac;
	int32_t k232_a, k232_b;           // (the K007232's last sample, held between its own)
	int64_t cycles_done;
	uint64_t cyc_frac, z_frac, z_end, ym_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot (screen_update) ----
static struct { uint8_t k51960_buffer[0x400], k51960_control, k51960_shadow, priority; k052109_t k109; } s_vs;
static k052109_view_t s_view;

// ---- the video callbacks (layer colour bases 0, 32, 40, sprites 16: VIDEO_START) ----
static const int s_layer_colorbase[3] = { 0, 32, 40 };
#define SPRITE_COLORBASE 16

static void tile_cb(int layer, int bank, int *code, int *color, int *flags, int *priority)
{
	(void)priority;
	const int c = *color;
	if (s_tmnt) {   // (tmnt_tile_callback)
		*code |= ((c & 0x03) << 8) | ((c & 0x10) << 6) | ((c & 0x0c) << 9) | (bank << 13);
		*color = s_layer_colorbase[layer] + ((c & 0xe0) >> 5);
		return;
	}
	if (s_cue) {   // (cuebrick_tile_callback)
		if (!s_vs.k109.rmrd_line && layer == 0) {
			*code |= (c & 0x01) << 8;
			*color = s_layer_colorbase[layer] + ((c & 0x0e) >> 1);
		} else {
			*code |= (c & 0x0f) << 8;
			*color = s_layer_colorbase[layer] + ((c & 0xe0) >> 5);
		}
		return;
	}
	*flags = (c & 0x04) ? K052109_TILE_FLIPX : 0;   // (mia_tile_callback)
	if (layer == 0) {
		*code |= (c & 0x01) << 8;
		*color = s_layer_colorbase[layer] + ((c & 0x80) >> 5) + ((c & 0x10) >> 1);
	} else {
		*code |= ((c & 0x01) << 8) | ((c & 0x18) << 6) | (bank << 11);
		*color = s_layer_colorbase[layer] + ((c & 0xe0) >> 5);
	}
}

static void sprite_cb(int *code, int *color, int *priority, int *shadow)   // (tmnt_ / mia_sprite_callback)
{
	(void)priority;
	(void)shadow;
	if (s_tmnt)
		*code |= (*color & 0x10) << 9;
	*color = SPRITE_COLORBASE + (*color & 0x0f);
}

static void capture(void)
{
	for (int i = 0; i < 0x400; i++)   // (the palette: xBGR_555 of two bytes, the high one first)
		((uint16_t *)gx.palram)[i] = (uint16_t)(t.pal8[2 * i] << 8 | t.pal8[2 * i + 1]);
	memcpy(s_vs.k51960_buffer, t.k51960_buffer, sizeof(s_vs.k51960_buffer));
	s_vs.k51960_control = t.k51960_control;
	s_vs.k51960_shadow = t.k51960_shadow;
	s_vs.priority = t.priority;
	memcpy(&s_vs.k109, &s_k109, offsetof(k052109_t, state_end));
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
	const int x1 = s_vis_x0 + s_vis_w - 1, y1 = VIS_Y0 + VIS_H - 1;
	memset(p, 0, (size_t)(s_vis_w * VIS_H));
	k052109_draw(&s_vs.k109, &s_view, 2, K052109_DRAW_OPAQUE, 0, f, p, s_vis_w, s_vis_x0, VIS_Y0, s_vis_x0, VIS_Y0, x1, y1);
	if (s_vs.priority & 1)
		gx_pre_sprites_051960(s_vs.k51960_buffer, s_vs.k51960_control, s_vs.k51960_shadow, 0, 0, sprite_cb, 0);
	k052109_draw(&s_vs.k109, &s_view, 1, 0, 0, f, p, s_vis_w, s_vis_x0, VIS_Y0, s_vis_x0, VIS_Y0, x1, y1);
	if (!(s_vs.priority & 1))
		gx_pre_sprites_051960(s_vs.k51960_buffer, s_vs.k51960_control, s_vs.k51960_shadow, 0, 0, sprite_cb, 0);
	k052109_draw(&s_vs.k109, &s_view, 0, 0, 0, f, p, s_vis_w, s_vis_x0, VIS_Y0, s_vis_x0, VIS_Y0, x1, y1);
}

// ---- the K051960 / K051937 ----
static inline int64_t now_cycles(void) { return t.cycles_done + m68k_cycles_run(); }

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
	t.k51960_busy_until = now_cycles() + (int64_t)((uint64_t)ticks * 4 * MAIN_CLOCK / 24000000u);
}

// ---- the sound board a frame behind on the worker (rbx_aux): the latch and the IRQ queued with the samples ----
#define Q_MAX 2048
enum { Q_ADV, Q_LATCH, Q_IRQ };
typedef struct { uint16_t kind, arg; uint32_t v; } q_ev_t;
static q_ev_t (*s_q_hp)[2][Q_MAX];
#define s_q (*s_q_hp)
static int s_qn[2], s_qcur;
static int32_t s_out[2048];
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
static inline int noA12(int o) { return ((o & 0x3000) >> 1) | (o & 0x07ff); }

static uint16_t main_r16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x080000 && a < 0x081000)   // (the palette: umask 00ff)
		return t.pal8[(a - 0x080000) >> 1];
	if (s_cue) {
		if (a >= 0x0b0000 && a < 0x0b0400) {   // (the NVRAM's bank)
			const uint8_t *p = s_nv + t.nvbank * 0x400 + (a - 0x0b0000);
			return (uint16_t)(p[0] << 8 | p[1]);
		}
		if (a == 0x0c0000 || a == 0x0c0002)   // (the YM2151: umask ff00)
			return (uint16_t)(rbx_ym_read(0, (a >> 1) & 1) << 8);
	}
	switch (a) {
	case 0x0a0000: return (uint16_t)rbx_port_read(s_port_coins);
	case 0x0a0002: return (uint16_t)rbx_port_read(s_port_p[0]);
	case 0x0a0004: return (uint16_t)rbx_port_read(s_port_p[1]);
	case 0x0a0006: return (uint16_t)rbx_port_read(s_port_p[2]);
	case 0x0a0010: return (uint16_t)rbx_port_read(s_port_dsw[0]);
	case 0x0a0012: return (uint16_t)rbx_port_read(s_port_dsw[1]);
	case 0x0a0014: return (uint16_t)rbx_port_read(s_port_p[3]);
	case 0x0a0018: return (uint16_t)rbx_port_read(s_port_dsw[2]);
	}
	if (a >= 0x100000 && a < 0x108000)   // (k052109_word_noA12_r: a word read the high byte's)
		return (uint16_t)(k052109_read(&s_k109, noA12((int)((a - 0x100000) >> 1))) << 8);
	if (a >= 0x140000 && a < 0x140008)
		return (uint16_t)(k051937_r((int)(a - 0x140000)) << 8 | k051937_r((int)(a - 0x140000) + 1));
	if (a >= 0x140400 && a < 0x140800)
		return (uint16_t)(k051960_r((int)(a - 0x140400)) << 8 | k051960_r((int)(a - 0x140400) + 1));
	return 0;
}
static uint8_t main_r8(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x100000 && a < 0x108000)
		return k052109_read(&s_k109, noA12((int)((a - 0x100000) >> 1)) + ((a & 1) ? 0x2000 : 0));
	if (a >= 0x140000 && a < 0x140008)
		return k051937_r((int)(a - 0x140000));
	if (a >= 0x140400 && a < 0x140800)
		return k051960_r((int)(a - 0x140400));
	const uint16_t w = main_r16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}

static void main_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a >= 0x080000 && a < 0x081000) {
		if (mask & 0x00ff)
			t.pal8[(a - 0x080000) >> 1] = (uint8_t)v;
	} else if (a == 0x0a0000) {   // (_0a0000_w)
		if (t.last == 0x08 && !(v & 0x08))   // (bit 3 high then low: the sound CPU's IRQ)
			q_put(Q_IRQ, 0);
		t.last = (uint8_t)(v & 0x08);
		t.irq5_mask = (uint8_t)(v & 0x20);
		if (!t.irq5_mask)
			m68k_bus_set_line(5, 0);
		k052109_set_rmrd_line(&s_k109, (v & 0x80) != 0);
	} else if (s_cue && a >= 0x0b0000 && a < 0x0b0400) {
		uint8_t *p = s_nv + t.nvbank * 0x400 + (a - 0x0b0000);
		if (mask & 0xff00) p[0] = (uint8_t)(v >> 8);
		if (mask & 0x00ff) p[1] = (uint8_t)v;
	} else if (s_cue && a == 0x0b0400 && (mask & 0xff00))   // (cuebrick_nvbank_w)
		t.nvbank = (uint8_t)((v >> 8) & 0x1f);
	else if (s_cue && (a == 0x0c0000 || a == 0x0c0002) && (mask & 0xff00))
		rbx_ym_write(0, (a >> 1) & 1, (uint8_t)(v >> 8));
	else if (a == 0x0a0008 && (mask & 0x00ff))   // (0a0009: the sound latch)
		q_put(Q_LATCH, (uint8_t)v);
	else if (a == 0x0c0000 && s_tmnt)   // (priority_w)
		t.priority = (uint8_t)((v & 0x0c) >> 2);
	else if (a >= 0x100000 && a < 0x108000) {   // (k052109_word_noA12_w: the MSB alone on word writes)
		const int o = noA12((int)((a - 0x100000) >> 1));
		if (mask & 0xff00)
			k052109_write(&s_k109, o, (uint8_t)(v >> 8));
		else
			k052109_write(&s_k109, o + 0x2000, (uint8_t)v);
	} else if (a >= 0x140000 && a < 0x140008) {
		if (mask & 0xff00) k051937_w((int)(a - 0x140000), (uint8_t)(v >> 8));
		if (mask & 0x00ff) k051937_w((int)(a - 0x140000) + 1, (uint8_t)v);
	} else if (a >= 0x140400 && a < 0x140800) {
		if (mask & 0xff00) t.k51960_ram[a - 0x140400] = (uint8_t)(v >> 8);
		if (mask & 0x00ff) t.k51960_ram[a - 0x140400 + 1] = (uint8_t)v;
	}
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v)
{
	main_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00);
}

// ---- the sound board (tmnt_ / mia_audio_map) ----
static void k007232_port(uint8_t data)   // (volume_callback)
{
	k007232_set_volume(&s_k232, 0, (data >> 4) * 0x11, 0);
	k007232_set_volume(&s_k232, 1, 0, (data & 0x0f) * 0x11);
}

static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0x8800)
		return t.zram[a - 0x8000];
	if (a == 0x9000 && s_tmnt)
		return t.sres;
	if (a == 0xa000)
		return t.soundlatch;
	if (a >= 0xb000 && a < 0xb00e)
		return k007232_read(&s_k232, a - 0xb000);
	if (a == 0xc000 || a == 0xc001)
		return rbx_ym_read(0, a & 1);
	if (a == 0xf000 && s_tmnt)
		return (uint8_t)upd7759_busy_r(&s_upd);
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8800)
		t.zram[a - 0x8000] = v;
	else if (a >= 0xb000 && a < 0xb00e)
		k007232_write(&s_k232, a - 0xb000, v);
	else if (a == 0xc000 || a == 0xc001)
		rbx_ym_write(0, a & 1, v);
	else if (!s_tmnt)
		return;
	else if (a == 0x9000) {   // (sres_w: bit 1 the uPD7759's reset, bit 2 the title music)
		upd7759_reset_w(&s_upd, (v >> 1) & 1);
		if (v & 4) {
			if (!t.title_on) {
				t.title_on = 1;
				t.title_pos = 0;
				t.title_frac = 0;
			}
		} else
			t.title_on = 0;
		t.sres = v;
	} else if (a == 0xd000)
		upd7759_port_w(&s_upd, v);
	else if (a == 0xe000)
		upd7759_start_w(&s_upd, !(v & 1));
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// n samples (a line's): the Z80 for their cycles, the YM2151 (its samples spread over them), the K007232 at its rate
// (held between), the uPD7759 and the title music; mono
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
	int32_t yl[96], yr[96], up[64];
	memset(yl, 0, sizeof(yl));
	memset(yr, 0, sizeof(yr));
	t.ym_frac += (uint64_t)CHIP_CLOCK * (uint64_t)n;
	const uint32_t yc = (uint32_t)(t.ym_frac / SND_RATE);
	t.ym_frac -= (uint64_t)yc * SND_RATE;
	const int yn = rbx_ym_advance_lr(0, yc, yl, yr, 96);
	if (s_tmnt)
		upd7759_render(&s_upd, up, n);
	for (int i = 0; i < n; i++) {
		t.k232_frac += CHIP_CLOCK / 128;   // (its rate: clock / 128)
		while (t.k232_frac >= SND_RATE) {
			t.k232_frac -= SND_RATE;
			k007232_sample(&s_k232, &t.k232_a, &t.k232_b);
		}
		const int k = yn > 0 ? i * yn / n : -1;
		int32_t m = (k >= 0 ? yl[k] + yr[k] : 0) + (t.k232_a + t.k232_b) * 15 / 100;
		if (s_tmnt) {
			m += up[i] * 30 / 100;
			if (t.title_on && t.title_pos < 0x40000 && s_title) {   // (samples: 640 kHz / 32, at 0.25)
				m += s_title[t.title_pos] / 4;
				t.title_frac += UPD_CLOCK / 32;
				while (t.title_frac >= SND_RATE) {
					t.title_frac -= SND_RATE;
					t.title_pos++;
				}
			}
		}
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
	if (s_cue) {   // (made on this thread: cue_sound)
		for (int i = 0; i < s_out_pos; i++)
			rbx_audio_put_stereo(s_out[i], s_out[i]);
		s_out_pos = 0;
		s_qn[0] = s_qn[1] = 0;
		return;
	}
	rbx_aux_sync();
	for (int i = 0; i < s_out_n; i++)
		rbx_audio_put_stereo(s_out[i], s_out[i]);
	s_out_n = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// cuebrick: the YM2151 with the main CPU, a line's n samples (its IRQ to the 68000 at once)
static void cue_sound(int n)
{
	if (n > 64)
		n = 64;
	int32_t ym[96];
	memset(ym, 0, sizeof(ym));
	t.ym_frac += (uint64_t)CHIP_CLOCK * (uint64_t)n;
	const uint32_t yc = (uint32_t)(t.ym_frac / SND_RATE);
	t.ym_frac -= (uint64_t)yc * SND_RATE;
	const int yn = rbx_ym_advance(0, yc, ym, 96);
	for (int i = 0; i < n && s_out_pos < 2048; i++)
		s_out[s_out_pos++] = yn > 0 ? ym[i * yn / n] : 0;
}
static void cue_ym_irq(int state) { m68k_bus_set_line(6, state); }

// ---- init / frame ----
static void tm_exit(void);
static void tm_reset(void);

static uint32_t bitswap32(uint32_t d, const int *b)   // (bitswap<32>: the result's bit 31 - i from bit b[i])
{
	uint32_t r = 0;
	for (int i = 0; i < 32; i++)
		r |= ((d >> b[i]) & 1u) << (31 - i);
	return r;
}

static void chunky_to_planar(uint8_t *rom, uint32_t size)
{
	static const int b[32] = { 31, 27, 23, 19, 15, 11, 7, 3, 30, 26, 22, 18, 14, 10, 6, 2,
	                           29, 25, 21, 17, 13, 9, 5, 1, 28, 24, 20, 16, 12, 8, 4, 0 };
	for (uint32_t i = 0; i + 4 <= size; i += 4) {
		uint32_t d = (uint32_t)rom[i] | (uint32_t)rom[i + 1] << 8 | (uint32_t)rom[i + 2] << 16 | (uint32_t)rom[i + 3] << 24;
		d = bitswap32(d, b);
		rom[i] = (uint8_t)d;
		rom[i + 1] = (uint8_t)(d >> 8);
		rom[i + 2] = (uint8_t)(d >> 16);
		rom[i + 3] = (uint8_t)(d >> 24);
	}
}

// the sprite ROM's address lines unscrambled (init_tmnt: by the PROM's entry; init_mia: a fixed swap), a word of 4 bytes
static int unscramble_sprites(uint8_t *rom, uint32_t size, const uint8_t *prom)
{
	const int len = (int)(size / 4);
	uint32_t *tmp = (uint32_t *)malloc((size_t)len * 4);
	if (!tmp)
		return -1;
	memcpy(tmp, rom, (size_t)len * 4);
	static const uint8_t pick[10][8] = {
		{ 3, 3, 3, 3, 3, 3, 3, 3 }, { 0, 0, 5, 5, 5, 5, 5, 5 }, { 1, 1, 0, 0, 0, 7, 7, 7 }, { 2, 2, 1, 1, 1, 0, 0, 9 },
		{ 4, 4, 2, 2, 2, 1, 1, 0 }, { 5, 6, 4, 4, 4, 2, 2, 1 }, { 6, 5, 6, 6, 6, 4, 4, 2 }, { 7, 7, 7, 7, 8, 6, 6, 4 },
		{ 8, 8, 8, 8, 7, 8, 8, 6 }, { 9, 9, 9, 9, 9, 9, 9, 8 } };
	for (int A = 0; A < len; A++) {
		int B;
		if (prom) {
			const int entry = prom[(A & 0x7f800) >> 11] & 7;
			B = A & 0x7fc00;
			for (int i = 0; i < 10; i++)
				B |= ((A >> pick[i][entry]) & 1) << i;
		} else {
			// (bitswap<8>(A, 7,6,4,2,1,0,5,3) or (A, 6,4,2,1,0,7,5,3): the result's bit 7 - i from the listed bit)
			static const int s1[8] = { 7, 6, 4, 2, 1, 0, 5, 3 }, s2[8] = { 6, 4, 2, 1, 0, 7, 5, 3 };
			const int *sw = (A & 0x3c000) == 0x3c000 ? s1 : s2;
			B = A & 0x3ff00;
			for (int i = 0; i < 8; i++)
				B |= ((A >> sw[i]) & 1) << (7 - i);
		}
		memcpy(rom + (size_t)A * 4, &tmp[B < len ? B : 0], 4);
	}
	free(tmp);
	return 0;
}

static int tm_init(const rbx_game *g)
{
	if (!s_nv_hp && !(s_nv_hp = calloc(1, sizeof(*s_nv_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t tile_size, pcm_size, upd_size = 0, title_size = 0, prom_size = 0;
	s_tmnt = !strcmp(g->machine, "tmnt");
	s_cue = !strcmp(g->machine, "cuebrick");
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *tiles = rbx_region("k052109", &tile_size);
	s_sprrom = rbx_region("k051960", &s_sprrom_size);
	const uint8_t *pcm = rbx_region("k007232", &pcm_size);
	const uint8_t *prom = rbx_region("proms", &prom_size);
	const uint8_t *upd = s_tmnt ? rbx_region("upd", &upd_size) : NULL;
	const uint8_t *title = s_tmnt ? rbx_region("title", &title_size) : NULL;
	if (!s_rom || !tiles || !s_sprrom || (!s_cue && (!s_zrom || !pcm)) || (s_tmnt && (!prom || !upd || !title))) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (s_cue) {
		s_zrom = NULL;
		s_zrom_size = 0;
		pcm = NULL;
		pcm_size = 0;
	}
	// (init_tmnt / init_mia: the gfx ROMs to the planar format, the sprite ROM unscrambled, the title music decoded;
	// init_cuebrick none)
	if (!s_cue)
		chunky_to_planar(tiles, tile_size);
	if (!s_cue)
		chunky_to_planar(s_sprrom, s_sprrom_size);
	if (!s_cue && unscramble_sprites(s_sprrom, s_sprrom_size, s_tmnt ? prom : NULL)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (s_tmnt) {
		if (!s_title && !(s_title = (int16_t *)malloc(0x40000 * sizeof(int16_t))))
			return -1;
		for (int i = 0; i < 0x40000 && (uint32_t)(2 * i + 1) < title_size; i++) {   // (ymfm::decode_fp of the 3.10 value)
			int16_t v = (int16_t)((title[2 * i] + title[2 * i + 1] * 256) >> 3);
			v ^= 0x1e00;
			s_title[i] = (int16_t)((int16_t)(v << 6) >> ((v >> 10) & 7));
		}
	}
	s_vis_x0 = s_tmnt ? 0 : 8;   // (tmnt: set_raw(..., 0, 320, ...); mia 0+8, 320-8)
	s_vis_w = s_tmnt ? 320 : 304;
	memset(&gx, 0, sizeof(gx));
	memset(&t, 0, sizeof(t));
	gx.pal_entries = 1024;
	gx.pal555 = 1;
	gx.tilecb = GX_TILECB_PRE;
	gx.mixmode = GX_MIXMODE_PRE;
	gx.pre_capture = capture;
	gx.pre_draw = draw;
	gx.pre_screen_w = HTOTAL;
	gx.pre_dim = 1;
	for (int i = 0; i < 8; i++)
		gx.pre_contrast[i] = 1.0f;
	gx.pre_shadow_factor = s_tmnt ? 0.75f : 0;   // (VIDEO_START tmnt: set_shadow_factor(0.75))
	k055673_init(&gx.k55673, s_sprrom, s_sprrom_size, K055673_LAYOUT_053245, 0, 0);   // (K051960_PLANEORDER_MIA)
	gx_video_set_area(s_vis_x0, s_vis_w);
	gx_video_set_y0(VIS_Y0);
	gx_video_set_height(VIS_H);
	if (k052109_init(&s_k109, tiles, tile_size, 1024, tile_cb)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	m68k_init();
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0x000000, (s_tmnt ? 0x60000 : s_cue ? 0x20000 : 0x40000) - 1, s_rom, s_rom_size, 1, 0);
	if (!s_tmnt)
		m68k_bus_map(0x040000, 0x043fff, gx.workram + 0x4000, 0x4000, 1, 1);
	m68k_bus_map(0x060000, 0x063fff, gx.workram, 0x4000, 1, 1);
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
	k007232_init(&s_k232, pcm, pcm_size, k007232_port);
	if (s_tmnt)
		upd7759_init(&s_upd, upd, upd_size, UPD_CLOCK, SND_RATE);
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = CHIP_CLOCK;
	yc.gain[0] = yc.gain[1] = s_tmnt ? 102 : s_cue ? 256 : 192;   // (tmnt 0.40, mia 0.75, cuebrick 1.0: both to mono)
	if (s_cue)
		yc.irq = cue_ym_irq;   // (cuebrick: M68K_IRQ_6)
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: YM2151", g->set);
		return -1;
	}
	s_port_coins = rbx_port_find("COINS");
	static const char *const pn[4] = { "P1", "P2", "P3", "P4" };
	for (int i = 0; i < 4; i++)
		s_port_p[i] = rbx_port_find(pn[i]);
	s_port_dsw[0] = rbx_port_find(s_cue ? "DSW2" : "DSW1");   // (cuebrick: DSW2 at 0a0010, DSW1 at 0a0012)
	s_port_dsw[1] = rbx_port_find(s_cue ? "DSW1" : "DSW2");
	s_port_dsw[2] = rbx_port_find("DSW3");
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(s_vis_w, VIS_H, 0x10000) || gx_video_init()) {
		rbx_fail("%s: out of memory", g->set);
		tm_exit();
		return -1;
	}
	for (int i = 0; i < 0x10000; i++) {   // (the bitmap: RGB 565)
		const int r = (i >> 11) & 31, gg = (i >> 5) & 63, b = i & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (gg << 2 | gg >> 4) << 8 | (b << 3 | b >> 2));
	}
	memset(s_nv, 0, sizeof(s_nv));   // (NVRAM DEFAULT_ALL_0)
	s_nv_path[0] = 0;
	if (s_cue) {
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			FILE *f = fopen(s_nv_path, "rb");
			if (f) {
				if (fread(s_nv, 1, sizeof(s_nv), f) != sizeof(s_nv))
					memset(s_nv, 0, sizeof(s_nv));
				fclose(f);
			}
		}
	}
	tm_reset();
	rbx_aux_set_job(s_cue ? NULL : job);
	return 0;
}

static void tm_exit(void)
{
	if (s_nv_path[0]) {
		FILE *f = fopen(s_nv_path, "wb");
		if (f) {
			fwrite(s_nv, 1, sizeof(s_nv), f);
			fclose(f);
		}
		s_nv_path[0] = 0;
	}
	rbx_aux_set_job(NULL);
	gx_video_exit();
	k052109_exit(&s_k109);
	rbx_ym_exit(0);
	free(s_title);
	s_title = NULL;
	free(s_nv_hp);
	s_nv_hp = NULL;
	free(s_q_hp);
	s_q_hp = NULL;
}

static void tm_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(gx.workram, 0, sizeof(gx.workram));
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	k052109_reset(&s_k109);
	for (int i = 0; i < 5; i++)
		k051937_w(i, 0);
	k007232_reset(&s_k232);
	if (s_tmnt) {   // (MACHINE_RESET tmnt: the uPD7759's /ST 1, /RESET 0)
		upd7759_reset(&s_upd);
		upd7759_start_w(&s_upd, 1);
		upd7759_reset_w(&s_upd, 0);
	}
	rbx_ym_reset(0);
	z80_init(&s_z);
	s_z.read_byte = z_rd;
	s_z.write_byte = z_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	for (int p = 0; p < 256; p++)
		s_zmap[p] = p < 0x80 && (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
	for (int p = 0; p < 8; p++)
		s_zmap[0x80 + p] = t.zram + p * 256;
	s_z.read_map = s_zmap;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
}

static void tm_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, the sprite DMA, vblank_w's IRQ 5)
			gx_video_capture();
			k051960_vblank();
			k052109_vblank(&s_k109, 1);
			if (t.irq5_mask)
				m68k_bus_set_line(5, 1);
		}
		if (vpos == VIS_Y0)
			k052109_vblank(&s_k109, 0);
		t.cyc_frac += (uint64_t)MAIN_CLOCK * HTOTAL;
		int64_t cyc = (int64_t)(t.cyc_frac / PIX_CLOCK);
		t.cyc_frac -= (uint64_t)cyc * PIX_CLOCK;
		while (cyc > 0) {
			const int used = m68k_execute((int)cyc);
			t.cycles_done += used;
			cyc -= used;
		}
		t.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(t.snd_frac / PIX_CLOCK);
		t.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		if (ns > 0 && s_cue)
			cue_sound(ns);
		else if (ns > 0)
			q_put(Q_ADV, (uint32_t)ns);
	}
	sound_frame();
	t.frame++;
	gx.frame = t.frame;
}

static int tm_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	int rc = 0;
	rbx_aux_sync();
	if ((uint32_t)m68k_context_size() > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	rc |= rbx_state_io(ctx, "tn_cpu", cpu, (uint32_t)m68k_context_size());
	rc |= rbx_state_io(ctx, "tn_bus", &m68k_bus.irq_pending, sizeof(uint32_t) * 2);
	rc |= rbx_state_io(ctx, "gx", &gx, (uint32_t)offsetof(gx_t, state_end));
	rc |= k052109_state(&s_k109, ctx, load, "tn_k109");
	rc |= rbx_state_io(ctx, "tn", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= k007232_state(&s_k232, ctx, load, "tn_k232");
	if (s_tmnt)
		rc |= upd7759_state(&s_upd, ctx, load, "tn_upd");
	if (s_cue)
		rc |= rbx_state_io(ctx, "tn_nv", s_nv, sizeof(s_nv));
	rc |= rbx_ym_state(0, ctx, load);
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "tn_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "tn_out", s_out, sizeof(s_out));
	if (load && !rc) {
		s_out_n = qn >= 0 && qn <= 2048 ? qn : 0;
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
	}
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "tn_z80", &z, sizeof(z));
	if (!rc && load) {
		m68k_set_context(cpu);
		z.read_byte = z_rd;
		z.write_byte = z_wr;
		z.port_in = z_in;
		z.port_out = z_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		z.op_rom = NULL;
		s_z = z;
	}
	return rc ? -1 : 0;
}

static int tm_save(void *ctx) { return tm_state(ctx, 0); }
static int tm_load(void *ctx) { return tm_state(ctx, 1); }
static int tm_supported(const char *machine)
{
	return !strcmp(machine, "tmnt") || !strcmp(machine, "mia") || !strcmp(machine, "cuebrick");
}

const rbx_driver rbx_drv_tmnt = { "Konami TMNT", tm_init, tm_exit, tm_reset, tm_frame, tm_save, tm_load, tm_supported,
                                  gx_render, NULL };
