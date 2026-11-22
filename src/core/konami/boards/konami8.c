/*  Konami's 8-bit boards with the Konami CPU (052001 / 053248) and the K052109 tilemaps: The Simpsons, Vendetta, Escape
    Kids, Parodius, Surprise Attack; Rollergames (its K051316 and K053245 alone, a YM3812). Ported from MAME
    src/mame/konami/simpsons.cpp, vendetta.cpp, parodius.cpp, surpratk.cpp, rollerg.cpp (BSD-3-Clause,
    copyright-holders: Ernesto Corvi; Bryan McPhail) and the devices they use (k052109.cpp, k053246_k053247_k055673.cpp,
    k053244_k053245.cpp, k053251.cpp, k054000.cpp, k053252.cpp, k053260.cpp, eepromser.cpp). The Konami CPU at 3 MHz
    (cpu/konami), the K053247 / K053245 sprites through konamigx_v.c's pre-GX mode (MAME's bitmap_ind16 and priority
    bitmap); the sound: a Z80 at 3.58 MHz (its NMI armed by the program, raised by the K053260's SH1), a YM2151 and the
    K053260, a frame behind on the worker.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/konami/konami.h"
#include "../../cpu/z80/z80.h"
#include "../../machine/eepromser.h"
#include "../../sound/k053260.h"
#include "../../sound/rbx_ym.h"
#include "../video/k051316.h"
#include "../video/k052109.h"
#include "../video/k055673.h"
#include "konamigx.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 3000000u     // (24 MHz / 2, the CPU's internal divider of 4)
#define Z80_CLOCK 3579545u
#define CHIP_CLOCK 3579545u
#define SND_RATE 48000
#define PIX_CLOCK 6000000u     // (set_raw(24 MHz / 4, 384, ..., 264, ...))
#define HTOTAL 384
#define LINES 264

enum { M_SIMPSONS, M_VENDETTA, M_ESCKIDS, M_PARODIUS, M_SURPRATK, M_ROLLERG };
#define K245 (s_mach == M_PARODIUS || s_mach == M_SURPRATK || s_mach == M_ROLLERG)   // (the K053244 / K053245 sprites)
static int s_mach, s_vis_x0, s_vis_w, s_vis_y0, s_vis_h;
#define VIS_X0 s_vis_x0
#define VIS_W s_vis_w
#define VIS_Y0 s_vis_y0
#define VIS_H s_vis_h

static uint8_t *s_rom, *s_zrom, *s_sprrom;
static uint32_t s_rom_size, s_zrom_size, s_sprrom_size;
static konami_t s_cpu;
static z80 s_z;
static const uint8_t *s_zmap[256];
static k052109_t s_k109;
static k053260_t s_k260;
static k051316_t s_k316;   // (rollerg)
static eepser_t s_eeprom;
static int s_port_coin, s_port_test, s_port_p[4], s_port_dsw[3], s_port_eep, s_port_service;
static int s_ym_gain, s_k260_gain;   // (the mix, 256 = 1.0)

static struct {
	uint8_t pal8[0x1000];      // (the palette's view: xBGR_555, the high byte first)
	uint8_t ram[0x2000];       // simpsons 4000-5fff, vendetta 2000-3fff, esckids 0000-1fff, parodius 0800-1fff
	uint8_t vram3[0x1000];     // (simpsons 3000-3fff with the video view 1)
	uint16_t spriteram[0x800]; // (the sprites' RAM in the video view: the K053247's words, the high byte first)
	uint8_t bank0_ram[0x800];  // (parodius 0000-07ff, bank0000's bank 0)
	uint8_t k54000[0x20];
	uint8_t k53252[16];
	uint16_t k245_ram[0x400], k245_buffer[0x400];
	uint8_t k244_regs[16];
	uint8_t k53251[16];
	uint8_t mainbank, video_bank, firq_enabled, irq_enabled, bank0000;
	uint8_t zoomroms;   // (rollerg ext_enable_w bit 2: the K051316's ROM at 0800-0fff)
	int32_t firq_on_line, firq_off_line;   // (the object DMA's FIRQ: dma_start / dma_end in lines from the vblank)
	int32_t line, obj_busy_end;            // (the line from the vblank start; vendetta's obj_busy until that line)
	uint8_t zram[0x800], zbank, z80_irq, nmi_line;
	int64_t nmi_due;
	int64_t cycles_done;
	int64_t cpu_end, snd_end;   // (the CPUs' line targets: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, z_frac, z_end, ym_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t k53251[16]; uint16_t k245_buffer[0x400]; uint8_t k244_regs[16]; k052109_t k109; k051316_t k316; } s_vs;
static k052109_view_t s_view;
static int s_layer_colorbase[3], s_layerpri[3], s_sprite_colorbase;

static int k53251_palette_index(const uint8_t *r, int ci)   // (k053251_device::reset_indexes)
{
	switch (ci) {
	case 0: return 32 * (r[9] & 3);
	case 1: return 32 * (r[9] >> 2 & 3);
	case 2: return 32 * (r[9] >> 4 & 3);
	case 3: return 16 * (r[10] & 7);
	default: return 16 * (r[10] >> 3 & 7);
	}
}

static void tile_cb(int layer, int bank, int *code, int *color, int *flags, int *priority)
{
	(void)flags;
	(void)priority;
	const int c = *color;
	switch (s_mach) {
	case M_SIMPSONS:   // (simpsons tile_callback)
		*code |= ((c & 0x3f) << 8) | (bank << 14);
		*color = s_layer_colorbase[layer] + ((c & 0xc0) >> 6);
		break;
	case M_VENDETTA:   // (vendetta_tile_callback)
		*code |= ((c & 0x03) << 8) | ((c & 0x30) << 6) | ((c & 0x0c) << 10) | (bank << 14);
		*color = s_layer_colorbase[layer] + ((c & 0xc0) >> 6);
		break;
	case M_SURPRATK:
		*flags = (c & 0x80) ? K052109_TILE_FLIPX : 0;
		*code |= ((c & 0x03) << 8) | ((c & 0x10) << 6) | ((c & 0x0c) << 9) | (bank << 13);
		*color = s_layer_colorbase[layer] + ((c & 0x60) >> 5);
		break;
	default:   // (esckids_tile_callback, parodius tile_callback)
		*code |= ((c & 0x03) << 8) | ((c & 0x10) << 6) | ((c & 0x0c) << 9) | (bank << 13);
		*color = s_layer_colorbase[layer] + ((c & 0xe0) >> 5);
		break;
	}
}

static void sprite_cb(int *code, int *color, int *primask)   // (the drivers' sprite_callback)
{
	(void)code;
	if (s_mach == M_ROLLERG) {   // (behind the K051316's priority 1 with bit 4 clear)
		*primask = (*color & 0x10) ? 0 : 0x02;
		*color = 256 / 16 + (*color & 0x0f);
		return;
	}
	const int pri = s_mach == M_SIMPSONS ? (*color & 0x0f80) >> 6 : K245 ? 0x20 | ((*color & 0x60) >> 2)
	                                                                                    : (*color & 0x03e0) >> 4;
	if (pri <= s_layerpri[2])
		*primask = 0;
	else if (pri <= s_layerpri[1])
		*primask = 0xf0;
	else if (pri <= s_layerpri[0])
		*primask = 0xf0 | 0xcc;
	else
		*primask = 0xf0 | 0xcc | 0xaa;
	*color = s_sprite_colorbase + (*color & 0x001f);
}

static void capture(void)
{
	for (int i = 0; i < 2048; i++)   // (the palette: xBGR_555 of two bytes, the high one first)
		((uint16_t *)gx.palram)[i] = (uint16_t)(t.pal8[2 * i] << 8 | t.pal8[2 * i + 1]);
	memcpy(s_vs.k53251, t.k53251, sizeof(s_vs.k53251));
	if (K245) {
		memcpy(s_vs.k245_buffer, t.k245_buffer, sizeof(s_vs.k245_buffer));
		memcpy(s_vs.k244_regs, t.k244_regs, sizeof(s_vs.k244_regs));
	}
	memcpy(&s_vs.k109, &s_k109, offsetof(k052109_t, state_end));
	if (s_mach == M_ROLLERG)
		memcpy(&s_vs.k316, &s_k316, offsetof(k051316_t, state_end));
}

static void zoom_cb(int chip, int *code, int *color)   // (rollerg zoom_callback)
{
	(void)chip;
	*code |= (*color & 0x0f) << 8;
	*color = (*color & 0x30) >> 4;
}

static void draw(void)   // (screen_update)
{
	if (s_mach == M_ROLLERG) {   // (the background pen 256, the K051316 at priority 1, the sprites)
		k051316_t *z = &s_vs.k316;
		z->rom = s_k316.rom;
		z->rom_size = s_k316.rom_size;
		z->gfx = s_k316.gfx;
		z->ntiles = s_k316.ntiles;
		z->pix = s_k316.pix;   // (the drawing's cache, the render thread's own)
		z->key = s_k316.key;
		z->cb = s_k316.cb;
		z->bpp = s_k316.bpp;
		gx_pre_fill_pen(16 * 16);
		k051316_zoom_draw(z, gx_pre_frame16(), gx_pre_pri(), VIS_W, VIS_X0, VIS_Y0, VIS_X0, VIS_Y0, VIS_X0 + VIS_W - 1,
		                  VIS_Y0 + VIS_H - 1, 0, 1);
		gx_pre_sprites_053245(s_vs.k245_buffer, s_vs.k244_regs, -3, -1, -1);   // (set_offsets(-3, -1))
		return;
	}
	static const int ci_234[3] = { 2, 3, 4 }, ci_243[3] = { 2, 4, 3 };
	const int *ci = K245 ? ci_243 : ci_234;
	const uint8_t *r = s_vs.k53251;
	const int bg_colorbase = k53251_palette_index(r, 0);
	s_sprite_colorbase = k53251_palette_index(r, 1);
	int layer[3];
	for (int i = 0; i < 3; i++) {
		s_layer_colorbase[i] = k53251_palette_index(r, ci[i]);
		layer[i] = i;
		s_layerpri[i] = r[ci[i]] & 0x3f;
	}
	for (int a = 0; a < 2; a++)   // (konami_sortlayers3: swapped when pri[a] < pri[b])
		for (int b = a + 1; b < 3; b++)
			if (s_layerpri[a] < s_layerpri[b]) {
				int x = s_layerpri[a]; s_layerpri[a] = s_layerpri[b]; s_layerpri[b] = x;
				x = layer[a]; layer[a] = layer[b]; layer[b] = x;
			}
	s_vs.k109.rom = s_k109.rom;
	s_vs.k109.rom_size = s_k109.rom_size;
	s_vs.k109.gfx = s_k109.gfx;
	s_vs.k109.ntiles = s_k109.ntiles;
	s_vs.k109.colors = s_k109.colors;
	s_vs.k109.cb = tile_cb;
	k052109_prepare(&s_vs.k109, &s_view, VIS_X0 + (VIS_X0 + VIS_W - 1) + 1, VIS_Y0 + (VIS_Y0 + VIS_H - 1) + 1);
	uint16_t *f = gx_pre_frame16();
	uint8_t *p = gx_pre_pri();
	const int vend = s_mach == M_VENDETTA || s_mach == M_ESCKIDS;   // (vendetta: the first layer OPAQUE, no fill)
	if (vend)
		memset(p, 0, (size_t)(VIS_W * VIS_H));
	else
		gx_pre_fill_pen(16 * bg_colorbase);
	static const uint8_t prio[3] = { 1, 2, 4 };
	for (int i = 0; i < 3; i++)
		k052109_draw(&s_vs.k109, &s_view, layer[i], (vend && !i) ? K052109_DRAW_OPAQUE : 0, prio[i], f, p, VIS_W, VIS_X0,
		             VIS_Y0, VIS_X0, VIS_Y0, VIS_X0 + VIS_W - 1, VIS_Y0 + VIS_H - 1);
	if (K245)
		gx_pre_sprites_053245(s_vs.k245_buffer, s_vs.k244_regs, 0, 0, -1);
	else
		gx_pre_sprites();
}

// ---- the sound board a frame behind on the worker: the K053260's latches and the Z80's IRQ queued ----
#define Q_MAX 2048
enum { Q_ADV, Q_LATCH, Q_IRQ };
typedef struct { uint16_t kind, arg; uint32_t v; } q_ev_t;
static q_ev_t (*s_q_hp)[2][Q_MAX];
#define s_q (*s_q_hp)
static int s_qn[2], s_qcur;
static int32_t s_out_l[2048], s_out_r[2048];
static int s_out_n, s_out_pos;
static volatile int s_job_frame;
static uint8_t s_pub[2];   // (the K053260's replies to the main CPU, a frame late)

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
static void irq_lines(void)   // (simpsons, parodius: the K052109's IRQ)
{
	if ((s_mach == M_SIMPSONS || K245) && s_mach != M_ROLLERG)
		konami_set_irq(&s_cpu, s_k109.irq_out);
}

static uint8_t k053246_r(int offset)   // (k053247_device::k053246_r)
{
	if (!gx.k55673.objcha_line)
		return 0;
	const uint8_t *r = gx.k55673.kx46_regs;
	uint32_t addr = ((uint32_t)r[6] << 17) | ((uint32_t)r[7] << 9) | ((uint32_t)r[4] << 1) | ((offset & 1) ^ 1);
	return s_sprrom[addr & (s_sprrom_size - 1)];
}

static uint8_t sprite_byte_r(int o)
{
	const uint16_t w = t.spriteram[(o >> 1) & 0x7ff];
	return (uint8_t)((o & 1) ? w : w >> 8);
}
static void sprite_byte_w(int o, uint8_t v)
{
	uint16_t *w = &t.spriteram[(o >> 1) & 0x7ff];
	*w = (o & 1) ? (uint16_t)((*w & 0xff00) | v) : (uint16_t)((*w & 0x00ff) | v << 8);
}

static inline int64_t now_cycles(void) { return t.cycles_done + s_cpu.cycles; }

// the EEPROM's lines (EEPROMOUT: CS bit 3, CLK bit 4, DI the simpsons' bit 7, the others' bit 5)
static void eeprom_lines(uint8_t v, int di_bit)
{
	const int64_t now = now_cycles();
	eepser_di_write(&s_eeprom, (v >> di_bit) & 1);
	eepser_cs_write(&s_eeprom, (v >> 3) & 1, now);
	eepser_clk_write(&s_eeprom, (v >> 4) & 1, now);
}

// -- simpsons --
static uint8_t simp_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a >= 0x1f80 && a < 0x1fd0) {
		switch (a) {
		case 0x1f80: return (uint8_t)rbx_port_read(s_port_coin);
		case 0x1f81: return (uint8_t)((rbx_port_read(s_port_test) & ~0x30u) |
		                              (uint32_t)eepser_do_read(&s_eeprom) << 4 |
		                              (uint32_t)eepser_ready_read(&s_eeprom, now_cycles()) << 5);
		case 0x1f90: case 0x1f91: case 0x1f92: case 0x1f93: return (uint8_t)rbx_port_read(s_port_p[a - 0x1f90]);
		case 0x1fc4: q_put(Q_IRQ, 0, 0); return 0;   // (sound_interrupt_r)
		case 0x1fc6: case 0x1fc7: return s_pub[a & 1];
		case 0x1fc8: case 0x1fc9: return k053246_r(a & 1);
		}
		return 0;
	}
	if (a < 0x1000 && (t.video_bank & 1))
		return t.pal8[a];
	if (a < 0x2000)
		return k052109_read(&s_k109, a);
	if (a < 0x4000) {
		if (!(t.video_bank & 2))
			return k052109_read(&s_k109, a);   // (k052109_r: offset + 0x2000)
		if (a < 0x3000)
			return sprite_byte_r(a - 0x2000);
		return t.vram3[a - 0x3000];
	}
	if (a < 0x6000)
		return t.ram[a - 0x4000];
	if (a < 0x8000)
		return s_rom[((uint32_t)t.mainbank * 0x2000 + (a - 0x6000)) & (s_rom_size - 1)];
	return s_rom[0x78000 + (a - 0x8000)];
}

static void simp_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a >= 0x1f80 && a < 0x1fd0) {
		if (a >= 0x1fa0 && a < 0x1fa8)
			k055673_k053246_w(&gx.k55673, a - 0x1fa0, v);
		else if (a >= 0x1fb0 && a < 0x1fc0)
			t.k53251[a - 0x1fb0] = v & 0x3f;
		else if (a == 0x1fc0) {   // (coin_counter_w: bit 3 the char ROM's reading, bit 5 the sprite ROM's)
			k052109_set_rmrd_line(&s_k109, (v >> 3) & 1);
			gx.k55673.objcha_line = !((v >> 5) & 1);
		} else if (a == 0x1fc2) {   // (eeprom_w)
			if (v == 0xff)
				return;
			eeprom_lines(v, 7);
			t.video_bank = v & 3;
			t.firq_enabled = (v >> 2) & 1;
			if (!t.firq_enabled)
				konami_set_firq(&s_cpu, 0);
		} else if (a == 0x1fc6 || a == 0x1fc7)
			q_put(Q_LATCH, a & 1, v);
		return;
	}
	if (a < 0x1000 && (t.video_bank & 1)) {
		t.pal8[a] = v;
		return;
	}
	if (a < 0x2000 || (a < 0x4000 && !(t.video_bank & 2))) {
		k052109_write(&s_k109, a, v);
		irq_lines();
		return;
	}
	if (a < 0x3000)
		sprite_byte_w(a - 0x2000, v);
	else if (a < 0x4000)
		t.vram3[a - 0x3000] = v;
	else if (a < 0x6000)
		t.ram[a - 0x4000] = v;
}

// -- vendetta, esckids: the K052109 at 4000-7fff (esckids 2000-5fff), its first 4 KB the sprites' RAM and its third
// the palette with the video view --

// the K054000 (k054000_device: the collision of two boxes; as tmnt2.c's)
static int k54000_delta(const uint8_t *b)
{
	int res = b[0] << 16 | b[1] << 8 | b[2];
	return (b[3] & 0x80) ? res - (0x100 - b[3]) : res + b[3];
}
static uint8_t k54000_axis(uint32_t ac, uint32_t bc, uint32_t aa, uint32_t ba)
{
	uint8_t res = 0;
	const int32_t sub = (int32_t)(ac - bc);
	if (sub > 511 || sub <= -1024)
		res |= 1;
	if (((uint32_t)abs(sub) & 0x1ff) > ((aa + ba) & 0x1ff))
		res |= 1;
	return res;
}
static uint8_t k54000_r(int r)
{
	if (r != 0x18)
		return 0;
	const uint8_t *k = t.k54000;
	const uint32_t acx = (uint32_t)k54000_delta(k + 1), acy = (uint32_t)k54000_delta(k + 9);
	const uint32_t bcx = (uint32_t)(k[0x15] << 16 | k[0x16] << 8 | k[0x17]), bcy = (uint32_t)(k[0x11] << 16 | k[0x12] << 8 | k[0x13]);
	return (uint8_t)(k54000_axis(acx, bcx, k[6], k[0x0e]) | k54000_axis(acy, bcy, k[7], k[0x0f]));
}

static int vpos(void) { return (VIS_Y0 + VIS_H + t.line) % LINES; }

static uint8_t vend_eeprom_port(void)   // (EEPROM: DO bit 0, READY bit 1, the object DMA's busy bit 3)
{
	return (uint8_t)((rbx_port_read(s_port_eep) & ~0x0bu) | (uint32_t)eepser_do_read(&s_eeprom) |
	                 (uint32_t)eepser_ready_read(&s_eeprom, now_cycles()) << 1 | (uint32_t)(t.line < t.obj_busy_end) << 3);
}

static void vend_5fe0_w(uint8_t v)   // (_5fe0_w: bit 3 the char ROM's reading, bit 5 the sprite ROM's)
{
	k052109_set_rmrd_line(&s_k109, (v >> 3) & 1);
	gx.k55673.objcha_line = (v >> 5) & 1;
}

static void vend_eeprom_w(uint8_t v)   // (eeprom_w)
{
	if (v == 0xff)
		return;
	eeprom_lines(v, 5);
	t.irq_enabled = (v >> 6) & 1;
	if (!t.irq_enabled)
		konami_set_irq(&s_cpu, 0);
	t.video_bank = v & 1;
}

// the I/O at 5f80 (esckids 3f80): 1 handled
static int vend_io_rd(int o, uint8_t *v)
{
	if (s_mach == M_VENDETTA) {
		if (o < 0x20) { *v = k54000_r(o); return 1; }
		switch (o) {
		case 0x40: case 0x41: case 0x42: case 0x43: *v = (uint8_t)rbx_port_read(s_port_p[o - 0x40]); return 1;
		case 0x50: *v = vend_eeprom_port(); return 1;
		case 0x51: *v = (uint8_t)rbx_port_read(s_port_service); return 1;
		case 0x64: q_put(Q_IRQ, 0, 0); *v = 0; return 1;   // (z80_irq_r)
		case 0x66: case 0x67: *v = s_pub[o & 1]; return 1;
		case 0x68: case 0x69: *v = k053246_r(o & 1); return 1;
		case 0x6a: *v = 0; return 1;   // (watchdog)
		}
		return 0;
	}
	if (o >= 0x40 && o < 0x50) {   // (the K053252: VCT read back)
		const int vc = ((t.k53252[8] << 8 | t.k53252[9]) & 0x1ff) + 1;
		*v = o == 0x4e ? (uint8_t)(((vpos() - vc) >> 8) & 1) : o == 0x4f ? (uint8_t)(vpos() - vc) : t.k53252[o - 0x40];
		return 1;
	}
	switch (o) {
	case 0x00: case 0x01: case 0x02: case 0x03: *v = (uint8_t)rbx_port_read(s_port_p[o]); return 1;
	case 0x12: *v = vend_eeprom_port(); return 1;
	case 0x13: *v = (uint8_t)rbx_port_read(s_port_service); return 1;
	case 0x54: q_put(Q_IRQ, 0, 0); *v = 0; return 1;
	case 0x56: case 0x57: *v = s_pub[o & 1]; return 1;
	case 0x58: case 0x59: *v = k053246_r(o & 1); return 1;
	}
	return 0;
}

static int vend_io_wr(int o, uint8_t v)
{
	if (s_mach == M_VENDETTA) {
		if (o < 0x20) {
			t.k54000[o] = v;
			return 1;
		}
		if (o >= 0x20 && o < 0x30) { t.k53251[o - 0x20] = v & 0x3f; return 1; }
		if (o >= 0x30 && o < 0x38) { k055673_k053246_w(&gx.k55673, o - 0x30, v); return 1; }
		switch (o) {
		case 0x60: vend_5fe0_w(v); return 1;
		case 0x62: vend_eeprom_w(v); return 1;
		case 0x64: q_put(Q_IRQ, 0, 0); return 1;   // (z80_irq_w)
		case 0x66: case 0x67: q_put(Q_LATCH, o & 1, v); return 1;
		}
		return 0;
	}
	if (o >= 0x20 && o < 0x28) { k055673_k053246_w(&gx.k55673, o - 0x20, v); return 1; }
	if (o >= 0x30 && o < 0x40) { t.k53251[o - 0x30] = v & 0x3f; return 1; }
	if (o >= 0x40 && o < 0x50) { t.k53252[o - 0x40] = v; return 1; }
	switch (o) {
	case 0x50: vend_5fe0_w(v); return 1;
	case 0x52: vend_eeprom_w(v); return 1;
	case 0x54: q_put(Q_IRQ, 0, 0); return 1;
	case 0x56: case 0x57: q_put(Q_LATCH, o & 1, v); return 1;
	case 0x5a: return 1;
	}
	return 0;
}

static uint8_t vend_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	const int esc = s_mach == M_ESCKIDS;
	const uint16_t io = esc ? 0x3f80 : 0x5f80, kb = esc ? 0x2000 : 0x4000;
	if (a >= io && a < io + 0x80) {
		uint8_t v;
		if (vend_io_rd(a - io, &v))
			return v;
	}
	if (a >= kb && a < kb + 0x4000) {
		const int o = a - kb;
		if (t.video_bank & 1) {
			if (o < 0x1000)
				return sprite_byte_r(o);
			if (o >= 0x2000 && o < 0x3000)
				return t.pal8[o - 0x2000];
		}
		return k052109_read(&s_k109, o);
	}
	if (a >= 0x8000)
		return s_rom[(esc ? 0x18000 : 0x38000) + (a - 0x8000)];
	if (esc ? a < 0x2000 : (a >= 0x2000 && a < 0x4000))
		return t.ram[a & 0x1fff];
	return s_rom[((uint32_t)t.mainbank * 0x2000 + (a & 0x1fff)) & (s_rom_size - 1)];   // (vendetta 0000, esckids 6000)
}

static void vend_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	const int esc = s_mach == M_ESCKIDS;
	const uint16_t io = esc ? 0x3f80 : 0x5f80, kb = esc ? 0x2000 : 0x4000;
	if (a >= io && a < io + 0x80 && vend_io_wr(a - io, v))
		return;
	if (a >= kb && a < kb + 0x4000) {
		const int o = a - kb;
		if (t.video_bank & 1) {
			if (o < 0x1000) {
				sprite_byte_w(o, v);
				return;
			}
			if (o >= 0x2000 && o < 0x3000) {
				t.pal8[o - 0x2000] = v;
				return;
			}
		}
		k052109_write(&s_k109, o, v);
		return;
	}
	if (esc ? a < 0x2000 : (a >= 0x2000 && a < 0x4000))
		t.ram[a & 0x1fff] = v;
}

// -- parodius --

// the K053244 / K053245 (as tmnt2.c's; the ROM bank 0)
static uint8_t k053244_r(int offset)
{
	offset &= 0xf;
	if (offset == 6)
		memcpy(t.k245_buffer, t.k245_ram, sizeof(t.k245_buffer));   // (update_buffer)
	else if (offset == 7)
		for (int i = 0; i < 0x400; i += 8)   // (clear_buffer)
			t.k245_buffer[i] = 0;
	else if (offset >= 0x0c) {
		uint32_t addr = (uint32_t)(t.k244_regs[11] & 7) << 18 | (uint32_t)t.k244_regs[8] << 10 |
		                (uint32_t)t.k244_regs[9] << 2 | (uint32_t)((offset & 3) ^ 1);
		return s_sprrom[addr & (s_sprrom_size - 1)];
	}
	return 0;
}
static void k053244_w(int offset, uint8_t v)
{
	offset &= 0xf;
	t.k244_regs[offset] = v;
	if (offset == 6)
		memcpy(t.k245_buffer, t.k245_ram, sizeof(t.k245_buffer));
	else if (offset == 7)
		for (int i = 0; i < 0x400; i += 8)
			t.k245_buffer[i] = 0;
}

static uint8_t par_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x0800) {   // (bank0000: bank 0 the RAM, 2 and 3 the palette's halves)
		if (t.bank0000 == 0)
			return t.bank0_ram[a];
		return t.bank0000 >= 2 ? t.pal8[(t.bank0000 - 2) * 0x800 + a] : 0;
	}
	if (a < 0x2000)
		return t.ram[a];
	if (a < 0x6000) {
		switch (a) {
		case 0x3f8c: case 0x3f8d: return (uint8_t)rbx_port_read(s_port_p[a - 0x3f8c]);
		case 0x3f8e: return (uint8_t)rbx_port_read(s_port_dsw[2]);
		case 0x3f8f: return (uint8_t)rbx_port_read(s_port_dsw[0]);
		case 0x3f90: return (uint8_t)rbx_port_read(s_port_dsw[1]);
		case 0x3fc0: return 0;   // (watchdog)
		case 0x3fcc: case 0x3fcd: return s_pub[a & 1];
		}
		if (a >= 0x3fa0 && a < 0x3fb0)
			return k053244_r(a - 0x3fa0);
		if (a < 0x2800 && (t.video_bank & 2)) {   // (k053245_r)
			const uint16_t w = t.k245_ram[((a - 0x2000) >> 1) & 0x3ff];
			return (uint8_t)((a & 1) ? w : w >> 8);
		}
		return k052109_read(&s_k109, a - 0x2000);
	}
	if (a < 0xa000)
		return s_rom[((uint32_t)t.mainbank * 0x4000 + (a - 0x6000)) & (s_rom_size - 1)];
	return s_rom[0x3a000 + (a - 0xa000)];
}

static void par_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0800) {
		if (t.bank0000 == 0)
			t.bank0_ram[a] = v;
		else if (t.bank0000 >= 2)
			t.pal8[(t.bank0000 - 2) * 0x800 + a] = v;
		return;
	}
	if (a < 0x2000) {
		t.ram[a] = v;
		return;
	}
	if (a >= 0x6000)
		return;
	if (a >= 0x3fa0 && a < 0x3fb0) {
		k053244_w(a - 0x3fa0, v);
		return;
	}
	if (a >= 0x3fb0 && a < 0x3fc0) {
		t.k53251[a - 0x3fb0] = v & 0x3f;
		return;
	}
	switch (a) {
	case 0x3fc0: k052109_set_rmrd_line(&s_k109, (v >> 3) & 1); return;   // (_3fc0_w)
	case 0x3fc4:   // (videobank_w)
		t.bank0000 = (v & 1) ? (uint8_t)(2 + ((v & 4) >> 2)) : 0;
		t.video_bank = v;
		return;
	case 0x3fc8: q_put(Q_IRQ, 0, 0); return;   // (sh_irqtrigger_w)
	case 0x3fcc: case 0x3fcd: q_put(Q_LATCH, a & 1, v); return;
	}
	if (a < 0x2800 && (t.video_bank & 2)) {   // (k053245_w)
		uint16_t *w = &t.k245_ram[((a - 0x2000) >> 1) & 0x3ff];
		*w = (a & 1) ? (uint16_t)((*w & 0xff00) | v) : (uint16_t)((*w & 0x00ff) | v << 8);
		return;
	}
	k052109_write(&s_k109, a - 0x2000, v);
	irq_lines();
}

// -- surpratk (main_map: bank0000 0 RAM, 1 the K053245's, 2 and 3 the palette's halves; the YM2151 on this CPU) --
static uint8_t sur_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x0800) {
		switch (t.bank0000) {
		case 0: return t.bank0_ram[a];
		case 1: {   // (k053245_r)
			const uint16_t w = t.k245_ram[(a >> 1) & 0x3ff];
			return (uint8_t)((a & 1) ? w : w >> 8);
		}
		default: return t.pal8[(t.bank0000 - 2) * 0x800 + a];
		}
	}
	if (a < 0x2000)
		return t.ram[a];
	if (a < 0x4000)
		return s_rom[((uint32_t)t.mainbank * 0x2000 + (a - 0x2000)) % s_rom_size];
	if (a < 0x8000) {
		switch (a) {
		case 0x5f8c: case 0x5f8d: return (uint8_t)rbx_port_read(s_port_p[a - 0x5f8c]);
		case 0x5f8e: return (uint8_t)rbx_port_read(s_port_dsw[2]);
		case 0x5f8f: return (uint8_t)rbx_port_read(s_port_dsw[0]);
		case 0x5f90: return (uint8_t)rbx_port_read(s_port_dsw[1]);
		case 0x5fc0: return 0;   // (watchdog)
		}
		if (a >= 0x5fa0 && a < 0x5fb0)
			return k053244_r(a - 0x5fa0);
		return k052109_read(&s_k109, a - 0x4000);
	}
	return s_rom[0x38000 + (a - 0x8000)];
}
static void sur_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0800) {
		if (t.bank0000 == 0)
			t.bank0_ram[a] = v;
		else if (t.bank0000 == 1) {   // (k053245_w)
			uint16_t *w = &t.k245_ram[(a >> 1) & 0x3ff];
			*w = (a & 1) ? (uint16_t)((*w & 0xff00) | v) : (uint16_t)((*w & 0x00ff) | v << 8);
		} else
			t.pal8[(t.bank0000 - 2) * 0x800 + a] = v;
		return;
	}
	if (a < 0x2000) {
		t.ram[a] = v;
		return;
	}
	if (a < 0x4000 || a >= 0x8000)
		return;
	if (a >= 0x5fa0 && a < 0x5fb0) {
		k053244_w(a - 0x5fa0, v);
		return;
	}
	if (a >= 0x5fb0 && a < 0x5fc0) {
		t.k53251[a - 0x5fb0] = v & 0x3f;
		return;
	}
	switch (a) {
	case 0x5fc0: k052109_set_rmrd_line(&s_k109, (v >> 3) & 1); return;   // (control_w)
	case 0x5fc4: t.bank0000 = (v & 2) ? (uint8_t)(2 + ((v >> 2) & 1)) : (uint8_t)(v & 1); return;   // (videobank_w)
	case 0x5fd0: case 0x5fd1: rbx_ym_write(0, a & 1, v); return;
	}
	k052109_write(&s_k109, a - 0x4000, v);
	irq_lines();
}
static void sur_ym_irq(int state) { konami_set_firq(&s_cpu, state); }   // (KONAMI_FIRQ_LINE)

// -- rollerg (main_map) --
static uint8_t rg_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x0800) {
		switch (a) {
		case 0x0020: return 0;   // (watchdog)
		case 0x0030: case 0x0031: return s_pub[a & 1];
		case 0x0050: case 0x0051: return (uint8_t)rbx_port_read(s_port_p[a - 0x0050]);
		case 0x0052: return (uint8_t)rbx_port_read(s_port_dsw[2]);
		case 0x0053: return (uint8_t)rbx_port_read(s_port_dsw[0]);
		case 0x0060: return (uint8_t)rbx_port_read(s_port_dsw[1]);
		case 0x0061: return 0x7f;   // (pip_r)
		}
		if (a >= 0x0100 && a < 0x0110)   // (the K053252: its registers read back)
			return t.k53252[a - 0x0100];
		if (a >= 0x0300 && a < 0x0310)
			return k053244_r(a - 0x0300);
		return 0;
	}
	if (a < 0x1000)
		return t.zoomroms ? k051316_rom_r(&s_k316, a - 0x0800) : s_k316.ram[a - 0x0800];
	if (a < 0x1800) {   // (k053245_r)
		const uint16_t w = t.k245_ram[((a - 0x1000) >> 1) & 0x3ff];
		return (uint8_t)((a & 1) ? w : w >> 8);
	}
	if (a < 0x2000)
		return t.pal8[a - 0x1800];
	if (a < 0x3b00)
		return t.ram[a - 0x2000];
	if (a < 0x4000)
		return 0;
	if (a < 0x8000) {   // (entries 0-5 the ROM's 16 KB pages, 6 and 7 its first two again)
		const uint32_t e = t.mainbank < 6 ? t.mainbank : t.mainbank - 6u;
		return s_rom[(e * 0x4000 + (a - 0x4000)) % s_rom_size];
	}
	return s_rom[0x18000 + (a - 0x8000)];
}
static void rg_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0800) {
		switch (a) {
		case 0x0010:   // (ext_enable_w: bit 2 the zoom ROM's view, bit 5 the wraparound)
			t.zoomroms = (v >> 2) & 1;
			s_k316.wrap = (v >> 5) & 1;
			return;
		case 0x0030: case 0x0031: q_put(Q_LATCH, a & 1, v); return;
		case 0x0040: q_put(Q_IRQ, 0, 0); return;   // (soundirq_w: HOLD_LINE)
		}
		if (a >= 0x0100 && a < 0x0110) {   // (the K053252: int1_ack (0e) clears the vblank IRQ)
			t.k53252[a - 0x0100] = v;
			if (a == 0x010e)
				konami_set_irq(&s_cpu, 0);
		} else if (a >= 0x0200 && a < 0x0210)
			k051316_ctrl_w(&s_k316, a - 0x0200, v);
		else if (a >= 0x0300 && a < 0x0310)
			k053244_w(a - 0x0300, v);
		return;
	}
	if (a < 0x1000)
		k051316_write(&s_k316, a - 0x0800, v);
	else if (a < 0x1800) {   // (k053245_w)
		uint16_t *w = &t.k245_ram[((a - 0x1000) >> 1) & 0x3ff];
		*w = (a & 1) ? (uint16_t)((*w & 0xff00) | v) : (uint16_t)((*w & 0x00ff) | v << 8);
	} else if (a < 0x2000)
		t.pal8[a - 0x1800] = v;
	else if (a < 0x3b00)
		t.ram[a - 0x2000] = v;
}

// the banked ROM's pages for the CPU's direct reads
static void map_bank(void)
{
	uint32_t base, size;
	uint16_t at;
	switch (s_mach) {
	case M_VENDETTA: at = 0x0000; size = 0x2000; base = (uint32_t)t.mainbank * 0x2000; break;
	case M_ESCKIDS: at = 0x6000; size = 0x2000; base = (uint32_t)t.mainbank * 0x2000; break;
	case M_PARODIUS: at = 0x6000; size = 0x4000; base = (uint32_t)t.mainbank * 0x4000; break;
	case M_SURPRATK: at = 0x2000; size = 0x2000; base = (uint32_t)t.mainbank * 0x2000; break;
	case M_ROLLERG: at = 0x4000; size = 0x4000; base = (uint32_t)(t.mainbank < 6 ? t.mainbank : t.mainbank - 6) * 0x4000; break;
	default: return;   // (simpsons: through main_rd)
	}
	base &= s_rom_size - 1;
	for (uint32_t p = 0; p < size / 256; p++)
		s_cpu.rpage[(at >> 8) + p] = base + p * 256 + 256 <= s_rom_size ? s_rom + base + p * 256 : NULL;
}

static void set_lines(void *ctx, uint8_t data)   // (banking_callback)
{
	(void)ctx;
	switch (s_mach) {
	case M_SIMPSONS: t.mainbank = data & 0x3f; break;
	case M_PARODIUS: t.mainbank = (data & 0x0f) ^ 0x0f; break;
	case M_SURPRATK: t.mainbank = data & 0x1f; break;
	case M_ROLLERG: t.mainbank = data & 0x07; break;
	default:
		if (data < 0x1c)   // (vendetta: 28 banks, others ignored)
			t.mainbank = data;
		break;
	}
	map_bank();
}

static void object_dma(void)   // (simpsons object_dma: the active sprites to the K053247's RAM)
{
	uint16_t *dst = gx.k55673.ram;
	const uint16_t *src = t.spriteram;
	int num_inactive = 256;
	for (int counter = 256; counter; counter--, src += 8)
		if ((*src & 0x8000) && (*src & 0xff)) {
			memcpy(dst, src, 16);
			dst += 8;
			num_inactive--;
		}
	for (; num_inactive; num_inactive--, dst += 8)
		*dst = 0;
}

// ---- the sound board (z80_map; vendetta, parodius: 0000-efff the ROM) ----
static void z_bank(void)
{
	if (s_mach != M_SIMPSONS)
		return;
	// (audiobank: entries 0 and 1 the region's 0x10000, 2-7 0x10000 + 0x4000 each after the first)
	const uint32_t off = 0x10000 + (t.zbank < 2 ? 0 : (uint32_t)(t.zbank - 2) * 0x4000);
	for (int p = 0; p < 0x40; p++)
		s_zmap[0x80 + p] = off + (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + off + p * 256 : NULL;
}

static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (s_mach == M_ROLLERG) {   // (rollerg sound_map)
		if (a < 0x8000)
			return a < s_zrom_size ? s_zrom[a] : 0;
		if (a < 0x8800)
			return t.zram[a - 0x8000];
		if (a >= 0xa000 && a < 0xa030)
			return k053260_read(&s_k260, a - 0xa000);
		if (a == 0xc000 || a == 0xc001)
			return rbx_ym_read(0, a & 1);
		return 0;
	}
	if (a < 0x8000 || (a < 0xf000 && s_mach != M_SIMPSONS))
		return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0xc000 && s_mach == M_SIMPSONS) {
		const uint32_t off = 0x10000 + (t.zbank < 2 ? 0 : (uint32_t)(t.zbank - 2) * 0x4000) + (a - 0x8000);
		return off < s_zrom_size ? s_zrom[off] : 0;
	}
	if (a >= 0xf000 && a < 0xf800)
		return t.zram[a - 0xf000];
	if (a == 0xf800 || a == 0xf801)
		return rbx_ym_read(0, a & 1);
	if (a >= 0xfc00 && a < 0xfc30)
		return k053260_read(&s_k260, a - 0xfc00);
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_mach == M_ROLLERG) {   // (rollerg: 8000 RAM, a000 the K053260, c000 the YM3812, fc00 the NMI's arming)
		if (a >= 0x8000 && a < 0x8800)
			t.zram[a - 0x8000] = v;
		else if (a >= 0xa000 && a < 0xa030)
			k053260_write(&s_k260, a - 0xa000, v);
		else if (a == 0xc000 || a == 0xc001)
			rbx_ym_write(0, a & 1, v);
		else if (a == 0xfc00)
			a = 0xfa00;   // (the same arming below)
		if (a != 0xfa00)
			return;
	}
	if (a >= 0xf000 && a < 0xf800 && s_mach != M_ROLLERG)
		t.zram[a - 0xf000] = v;
	else if (a == 0xf800 || a == 0xf801)
		rbx_ym_write(0, a & 1, v);
	else if (a >= 0xfc00 && a < 0xfc30)
		k053260_write(&s_k260, a - 0xfc00, v);
	else if (a == 0xfa00) {   // (z80_arm_nmi_w: NMI cleared, blocked 4 cycles; the next SH1 rise asserts it)
		t.nmi_line = 0;
		const uint64_t tb = ((uint64_t)s_z.cyc + 4) * CHIP_CLOCK / Z80_CLOCK + 1;
		const uint64_t rise = tb + k053260_sh1_next_rise(tb) - 1;
		t.nmi_due = (int64_t)((rise * Z80_CLOCK + CHIP_CLOCK - 1) / CHIP_CLOCK);
		z80_break = 1;
	} else if (a == 0xfe00 && s_mach == M_SIMPSONS) {
		t.zbank = v & 7;
		z_bank();
	}
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void sound_run(int n)
{
	if (n <= 0)
		return;
	t.z_frac += (uint64_t)Z80_CLOCK * (uint64_t)n;
	const unsigned long cyc = (unsigned long)(t.z_frac / SND_RATE);
	t.z_frac -= (uint64_t)cyc * SND_RATE;
	t.z_end += cyc;
	for (;;) {
		unsigned long end = (unsigned long)t.z_end;
		if (t.nmi_due >= 0 && (unsigned long)t.nmi_due < end)
			end = (unsigned long)t.nmi_due;
		int line = t.z80_irq;
		z80_break = 0;
		if ((long)(s_z.cyc - end) < 0)
			z80_run(&s_z, end, &line, 1);
		t.z80_irq = (uint8_t)line;
		z80_break = 0;
		if (t.nmi_due >= 0 && (long)(s_z.cyc - (unsigned long)t.nmi_due) >= 0) {   // (z80_nmi_w: SH1's rise)
			t.nmi_due = -1;
			if (!t.nmi_line) {
				t.nmi_line = 1;
				z80_gen_nmi(&s_z);
			}
			continue;
		}
		if ((long)(s_z.cyc - (unsigned long)t.z_end) >= 0)
			break;
	}
	int32_t l[64], r[64], yl[96], yr[96];
	if (n > 64)
		n = 64;
	memset(l, 0, sizeof(int32_t) * (size_t)n);
	memset(r, 0, sizeof(int32_t) * (size_t)n);
	k053260_render(&s_k260, l, r, n);
	t.ym_frac += (uint64_t)CHIP_CLOCK * (uint64_t)n;
	const uint32_t yc = (uint32_t)(t.ym_frac / SND_RATE);
	t.ym_frac -= (uint64_t)yc * SND_RATE;
	memset(yl, 0, sizeof(yl));
	memset(yr, 0, sizeof(yr));
	const int rg = s_mach == M_ROLLERG;   // (rollerg: the YM3812 1.0 and the K053260 0.70, mono)
	const int yn = rg ? rbx_ym_advance(0, yc, yl, 96) : rbx_ym_advance_lr(0, yc, yl, yr, 96);
	const int mono = s_mach == M_SIMPSONS;   // (simpsons: only the YM2151's left output, to both sides)
	for (int i = 0; i < n; i++) {
		const int k = yn > 0 ? i * yn / n : -1;
		if (s_out_pos < 2048) {
			const int32_t y0 = k >= 0 ? yl[k] : 0, y1 = k >= 0 ? (mono ? yl[k] : yr[k]) : 0;
			if (rg) {
				s_out_l[s_out_pos] = s_out_r[s_out_pos] = y0 + (((l[i] + r[i]) * 179) >> 8);
			} else if (mono) {   // (0.5 and 0.5)
				s_out_l[s_out_pos] = y0 / 2 + l[i] / 2;
				s_out_r[s_out_pos] = y1 / 2 + r[i] / 2;
			} else {
				s_out_l[s_out_pos] = (y0 * s_ym_gain + l[i] * s_k260_gain) >> 8;
				s_out_r[s_out_pos] = (y1 * s_ym_gain + r[i] * s_k260_gain) >> 8;
			}
			s_out_pos++;
		}
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
		case Q_LATCH: k053260_main_write(&s_k260, e->arg, (uint8_t)e->v); break;
		case Q_IRQ: t.z80_irq = 1; break;
		}
	}
	s_qn[b] = 0;
	s_out_n = s_out_pos;
	__atomic_store_n(&s_job_frame, 0, __ATOMIC_RELEASE);
}

// surpratk: the YM2151 with the main CPU, a line's n samples (its IRQ to the CPU at once)
static void sur_sound(int n)
{
	if (n > 64)
		n = 64;
	int32_t yl[96], yr[96];
	memset(yl, 0, sizeof(yl));
	memset(yr, 0, sizeof(yr));
	t.ym_frac += (uint64_t)CHIP_CLOCK * (uint64_t)n;
	const uint32_t yc = (uint32_t)(t.ym_frac / SND_RATE);
	t.ym_frac -= (uint64_t)yc * SND_RATE;
	const int yn = rbx_ym_advance_lr(0, yc, yl, yr, 96);
	for (int i = 0; i < n && s_out_pos < 2048; i++, s_out_pos++) {
		const int k = yn > 0 ? i * yn / n : -1;
		s_out_l[s_out_pos] = k >= 0 ? yl[k] : 0;
		s_out_r[s_out_pos] = k >= 0 ? yr[k] : 0;
	}
}

static void sound_frame(void)
{
	if (s_mach == M_SURPRATK) {   // (made on this thread: sur_sound)
		for (int i = 0; i < s_out_pos; i++)
			rbx_audio_put_stereo(s_out_l[i], s_out_r[i]);
		s_out_pos = 0;
		s_qn[0] = s_qn[1] = 0;
		return;
	}
	rbx_aux_sync();
	for (int i = 0; i < s_out_n; i++)
		rbx_audio_put_stereo(s_out_l[i], s_out_r[i]);
	s_out_n = 0;
	s_pub[0] = k053260_main_read(&s_k260, 0);
	s_pub[1] = k053260_main_read(&s_k260, 1);
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void k8_exit(void);
static void k8_reset(void);

static int k8_init(const rbx_game *g)
{
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_mach = !strcmp(g->machine, "vendetta") ? M_VENDETTA : !strcmp(g->machine, "esckids") ? M_ESCKIDS
	       : !strcmp(g->machine, "parodius") ? M_PARODIUS : !strcmp(g->machine, "surpratk") ? M_SURPRATK
	       : !strcmp(g->machine, "rollerg") ? M_ROLLERG : M_SIMPSONS;
	const int rg = s_mach == M_ROLLERG;   // (no K052109)
	const int sur = s_mach == M_SURPRATK;   // (no sound CPU: the YM2151 on the main CPU's bus)
	uint32_t tile_size, pcm_size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *tiles = rbx_region("k052109", &tile_size);
	s_sprrom = rbx_region(s_mach == M_PARODIUS ? "k053245" : sur || rg ? "k053244" : "k053246", &s_sprrom_size);
	uint32_t zsize = 0;
	const uint8_t *zrom = rg ? rbx_region("k051316", &zsize) : NULL;
	pcm_size = 0;
	const uint8_t *pcm = sur ? NULL : rbx_region("k053260", &pcm_size);
	if (!s_rom || (!sur && (!s_zrom || !pcm)) || (!tiles && !rg) || !s_sprrom || (rg && !zrom)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	// (the screens: set_raw(24 MHz / 4, 384, x0, x1, 264, y0, y1); the sprites' set_config offsets)
	int sdx = -43, sdy = 23;
	switch (s_mach) {
	case M_VENDETTA: s_vis_x0 = 8; s_vis_w = 304; s_vis_y0 = 16; s_vis_h = 224; sdy = 6; break;
	case M_ESCKIDS: s_vis_x0 = 0; s_vis_w = 321; s_vis_y0 = 8; s_vis_h = 240; sdx = 5; sdy = 6; break;
	case M_PARODIUS: s_vis_x0 = 16; s_vis_w = 288; s_vis_y0 = 16; s_vis_h = 224; sdx = sdy = 0; break;
	case M_SURPRATK: s_vis_x0 = 0; s_vis_w = 320; s_vis_y0 = 16; s_vis_h = 224; sdx = sdy = 0; break;
	case M_ROLLERG: s_vis_x0 = 16; s_vis_w = 288; s_vis_y0 = 16; s_vis_h = 224; sdx = sdy = 0; break;
	default: s_vis_x0 = 0; s_vis_w = 320; s_vis_y0 = 16; s_vis_h = 224; break;
	}
	s_ym_gain = s_mach == M_PARODIUS ? 256 : 128;     // (parodius 1.0, vendetta 0.5)
	s_k260_gain = s_mach == M_PARODIUS ? 179 : 192;   // (parodius 0.70, vendetta 0.75)
	memset(&gx, 0, sizeof(gx));
	memset(&t, 0, sizeof(t));
	gx.pal_entries = 2048;
	gx.pal555 = 1;
	gx.tilecb = GX_TILECB_PRE;
	gx.mixmode = GX_MIXMODE_PRE;
	gx.pre_sprcb = sprite_cb;
	gx.pre_capture = capture;
	gx.pre_draw = draw;
	gx.pre_screen_w = HTOTAL;
	gx.pre_shadow_red = sur;       // (surpratk palette_w: the shadow pens with red at 0.6)
	gx.pre_k245_prishadow = sur;   // (set_priority_shadows(true))
	k055673_init(&gx.k55673, s_sprrom, s_sprrom_size, K245 ? K055673_LAYOUT_053245 : K055673_LAYOUT_053247,
	             sdx, sdy);
	gx_video_set_area(VIS_X0, VIS_W);
	gx_video_set_y0(VIS_Y0);
	gx_video_set_height(VIS_H);
	if (rg && k051316_init(&s_k316, 0, zrom, zsize, 6 + 16, 1, zoom_cb)) {   // (set_offsets(6 + 16, 1))
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!rg && k052109_init(&s_k109, tiles, tile_size, 2048, tile_cb)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	k053260_init(&s_k260, pcm, pcm_size, CHIP_CLOCK);
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = rg ? RBX_YM3812 : RBX_YM2151;
	yc.clock = CHIP_CLOCK;
	yc.gain[0] = yc.gain[1] = 256;
	if (sur)
		yc.irq = sur_ym_irq;
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: YM2151", g->set);
		return -1;
	}
	eepser_init(&s_eeprom, EEPSER_ER5911, 128, 8, CPU_CLOCK);
	{
		uint32_t esz;
		const uint8_t *e = rbx_region("eeprom", &esz);
		if (e)
			eepser_default(&s_eeprom, e, esz);
	}
	eepser_nv_load(&s_eeprom, g->set);   // (the contents kept between runs)
	s_port_coin = rbx_port_find("COIN");
	s_port_test = rbx_port_find("TEST");
	s_port_eep = rbx_port_find("EEPROM");
	s_port_service = rbx_port_find("SERVICE");
	static const char *const pn[4] = { "P1", "P2", "P3", "P4" }, *const dn[3] = { "DSW1", "DSW2", "DSW3" };
	for (int i = 0; i < 4; i++)
		s_port_p[i] = rbx_port_find(pn[i]);
	for (int i = 0; i < 3; i++)
		s_port_dsw[i] = rbx_port_find(dn[i]);
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, 0x10000) || gx_video_init()) {
		rbx_fail("%s: out of memory", g->set);
		k8_exit();
		return -1;
	}
	for (int i = 0; i < 0x10000; i++) {   // (the bitmap: RGB 565)
		const int r = (i >> 11) & 31, gg = (i >> 5) & 63, b = i & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (gg << 2 | gg >> 4) << 8 | (b << 3 | b >> 2));
	}
	k8_reset();
	rbx_aux_set_job(s_mach == M_SURPRATK ? NULL : job);
	return 0;
}

static void k8_exit(void)
{
	eepser_nv_save(&s_eeprom);
	rbx_aux_set_job(NULL);
	gx_video_exit();
	k052109_exit(&s_k109);
	if (s_mach == M_ROLLERG)
		k051316_exit(&s_k316);
	rbx_ym_exit(0);
	free(s_q_hp);
	s_q_hp = NULL;
}

// the CPU's direct pages: the fixed ROM and the RAM (the I/O windows left to the handlers)
static void map_pages(void)
{
	memset(s_cpu.rpage, 0, sizeof(s_cpu.rpage));
	memset(s_cpu.wpage, 0, sizeof(s_cpu.wpage));
	switch (s_mach) {
	case M_SIMPSONS:
		for (int p = 0x80; p < 0x100; p++)   // (8000-ffff: the ROM's last 32 KB)
			s_cpu.rpage[p] = s_rom + 0x78000 + (p - 0x80) * 256;
		for (int p = 0x40; p < 0x60; p++)
			s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + (p - 0x40) * 256;
		break;
	case M_VENDETTA:
		for (int p = 0x80; p < 0x100; p++)
			s_cpu.rpage[p] = s_rom + 0x38000 + (p - 0x80) * 256;
		for (int p = 0x20; p < 0x40; p++)
			s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + (p - 0x20) * 256;
		break;
	case M_ESCKIDS:
		for (int p = 0x80; p < 0x100; p++)
			s_cpu.rpage[p] = s_rom + 0x18000 + (p - 0x80) * 256;
		for (int p = 0x00; p < 0x20; p++)
			s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + p * 256;
		break;
	case M_ROLLERG:
		for (int p = 0x80; p < 0x100; p++)
			s_cpu.rpage[p] = s_rom + 0x18000 + (p - 0x80) * 256;
		for (int p = 0x20; p < 0x3b; p++)
			s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + (p - 0x20) * 256;
		break;
	case M_SURPRATK:
		for (int p = 0x80; p < 0x100; p++)
			s_cpu.rpage[p] = s_rom + 0x38000 + (p - 0x80) * 256;
		for (int p = 0x08; p < 0x20; p++)
			s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + p * 256;
		break;
	case M_PARODIUS:
		for (int p = 0xa0; p < 0x100; p++)
			s_cpu.rpage[p] = s_rom + 0x3a000 + (p - 0xa0) * 256;
		for (int p = 0x08; p < 0x20; p++)
			s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + p * 256;
		break;
	}
	map_bank();
}

static void k8_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	s_pub[0] = s_pub[1] = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	t.nmi_due = -1;
	t.nmi_line = 1;   // (machine_reset: the sound CPU's NMI asserted)
	t.firq_on_line = t.firq_off_line = -1;
	memset(gx.k55673.ram, 0, sizeof(gx.k55673.ram));
	k052109_reset(&s_k109);
	k053260_reset(&s_k260);
	rbx_ym_reset(0);
	konami_init(&s_cpu);
	static uint8_t (*const rd[6])(void *, uint16_t) = { simp_rd, vend_rd, vend_rd, par_rd, sur_rd, rg_rd };
	static void (*const wr[6])(void *, uint16_t, uint8_t) = { simp_wr, vend_wr, vend_wr, par_wr, sur_wr, rg_wr };
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
	const int zfix = s_mach == M_SIMPSONS || s_mach == M_ROLLERG ? 0x80 : 0xf0;   // (the ROM's direct pages)
	for (int p = 0; p < zfix; p++)
		s_zmap[p] = (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
	z_bank();
	for (int p = 0; p < 8; p++)   // (rollerg's RAM at 8000)
		s_zmap[(s_mach == M_ROLLERG ? 0x80 : 0xf0) + p] = t.zram + p * 256;
	s_z.read_map = s_zmap;
}

static void k8_frame(void)
{
	const int vend = s_mach == M_VENDETTA || s_mach == M_ESCKIDS;
	for (int line = 0; line < LINES; line++) {
		t.line = line;
		const int vp = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vp == VIS_Y0 + VIS_H) {   // (vblank: the update, the IRQ, the object DMA)
			gx_video_capture();
			k052109_vblank(&s_k109, 1);
			irq_lines();
			if (s_mach == M_ROLLERG)   // (irq0_line_assert: until the K053252's int1_ack)
				konami_set_irq(&s_cpu, 1);
			if (vend) {   // (vblank_irq)
				if (t.irq_enabled)
					konami_set_irq(&s_cpu, 1);
				if (gx.k55673.kx46_regs[5] & 0x10) {   // (the sprites' RAM to the K053247's, obj_busy 250 us)
					memcpy(gx.k55673.ram, t.spriteram, sizeof(t.spriteram));
					t.obj_busy_end = line + 4;
				}
			} else if (s_mach == M_SIMPSONS && (gx.k55673.kx46_regs[5] & 0x10)) {   // (periodic_irq's object DMA)
				object_dma();
				t.firq_on_line = line + 1;   // (dma_start: 256 ticks of 6 MHz, inside the next line)
				t.firq_off_line = line + 7;  // (dma_end: 256 + 2048 ticks: 6 lines on)
			}
		}
		if (vp == VIS_Y0) {
			k052109_vblank(&s_k109, 0);
			irq_lines();
		}
		if (line == t.firq_on_line) {
			t.firq_on_line = -1;
			if (t.firq_enabled)
				konami_set_firq(&s_cpu, 1);
		}
		if (line == t.firq_off_line) {
			t.firq_off_line = -1;
			konami_set_firq(&s_cpu, 0);
		}
		t.cyc_frac += (uint64_t)CPU_CLOCK * HTOTAL;
		const int64_t cyc = (int64_t)(t.cyc_frac / PIX_CLOCK);
		t.cyc_frac -= (uint64_t)cyc * PIX_CLOCK;
		konami_run_until(&s_cpu, t.cpu_end += cyc);
		t.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(t.snd_frac / PIX_CLOCK);
		t.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		if (ns > 0 && s_mach == M_SURPRATK)
			sur_sound(ns);
		else if (ns > 0)
			q_put(Q_ADV, 0, (uint32_t)ns);
	}
	t.obj_busy_end -= LINES;
	sound_frame();
	t.frame++;
	gx.frame = t.frame;
}

static int k8_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "k8_cpu", &s_cpu, KONAMI_STATE_SIZE);
	rc |= rbx_state_io(ctx, "gx", &gx, (uint32_t)offsetof(gx_t, state_end));
	rc |= k055673_state(&gx.k55673, ctx, load);
	rc |= k052109_state(&s_k109, ctx, load, "k8_k109");
	rc |= rbx_state_io(ctx, "k8", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= k053260_state(&s_k260, ctx, load, "k8_k260");
	if (s_mach == M_ROLLERG)
		rc |= k051316_state(&s_k316, ctx, load, "k8_k316");
	rc |= rbx_ym_state(0, ctx, load);
	rc |= eepser_state(&s_eeprom, ctx, load, "k8_eeprom");
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "k8_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "k8_outl", s_out_l, sizeof(s_out_l));
	rc |= rbx_state_io(ctx, "k8_outr", s_out_r, sizeof(s_out_r));
	rc |= rbx_state_io(ctx, "k8_pub", s_pub, sizeof(s_pub));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "k8_z80", &z, sizeof(z));
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
		z_bank();
		map_bank();
	}
	return rc ? -1 : 0;
}

static int k8_save(void *ctx) { return k8_state(ctx, 0); }
static int k8_load(void *ctx) { return k8_state(ctx, 1); }
static int k8_supported(const char *machine)
{
	return !strcmp(machine, "simpsons") || !strcmp(machine, "vendetta") || !strcmp(machine, "esckids") ||
	       !strcmp(machine, "parodius") || !strcmp(machine, "surpratk") || !strcmp(machine, "rollerg");
}

const rbx_driver rbx_drv_simpsons = { "Konami Simpsons", k8_init, k8_exit, k8_reset, k8_frame, k8_save, k8_load,
                                      k8_supported, gx_render, NULL };
