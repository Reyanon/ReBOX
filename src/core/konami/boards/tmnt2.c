/*  Konami Teenage Mutant Ninja Turtles: Turtles in Time, Sunset Riders, Lightning Fighters, Bells & Whistles, Punk Shot,
    Thunder Cross II (the K051960 / K051937 sprites: k051960.cpp, Fabio Priuli, Acho A. Tang, R. Belmont), Premier Soccer
    (a K053936 ROZ layer from ROM: k053936.cpp, David Haywood; tilemap.cpp's draw_roz; a K054539 and the K054321 for
    its sound: k054539.cpp, k054321.cpp, Olivier Galibert). Ported from MAME src/mame/konami/tmnt2.cpp
    (BSD-3-Clause, copyright-holders: Nicola Salmoria) and the devices it uses (k053244_k053245.cpp: Nicola Salmoria;
    k053251.cpp: Nicola Salmoria; eepromser.cpp: Aaron Giles; k053260.cpp: Ernesto Corvi, Alex W. Jackson).
    A 68000 at 16 MHz, the K052109 tilemaps (video/k052109.c), the K053244/K053245 sprites, the K053251 priorities and
    colour bases, the palette dimming (drawn by konamigx_v.c's pre-GX mode: MAME's bitmap_ind16 with the priority
    bitmap), the 053990 protection (TMNT2: its sprite builder; Sunset Riders: MAME's simulation from a bootleg), an ER5911
    EEPROM; the sound: a Z80 at 8 MHz, a YM2151, a K053260 whose SH1 gives the Z80's NMI (a frame behind on the worker).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../machine/eepromser.h"
#include "../../sound/k053260.h"
#include "../../sound/k054539.h"
#include "../../sound/rbx_ym.h"
#include "../video/k052109.h"
#include "../video/k055673.h"
#include "konamigx.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CHIP_CLOCK 3579545u   // (the YM2151 and the K053260)
#define SND_RATE 48000
#define PIX_CLOCK 6000000u    // (set_raw(24 MHz / 4, 384, x0, x1, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_Y0 16
#define VIS_H 224

// the machines: tmnt2_state (the 053990's sprite builder), ssriders_state (its protection), lgtnfght_state (DIP
// switches, its own sound map without the NMI), blswhstl_state (the K054000, a tile ROM bank)
enum { M_TMNT2, M_SSRIDERS, M_LGTNFGHT, M_BLSWHSTL, M_PUNKSHOT, M_THNDRX2, M_PRMRSOCR };
static int s_mach, s_tmnt2, s_irq_level;
static uint32_t MAIN_CLOCK, Z80_CLOCK;
static int s_port_dsw[3];
static k054539_t s_k539;              // (prmrsocr)
static const uint8_t *s_zoom, *s_zoomtmap;
static uint32_t s_zoom_size, s_zoomtmap_size;
static uint8_t *s_roz_gfx;            // (the zoom tiles: gfx_16x16x4_packed_lsb, a byte a pixel)
static uint32_t *s_roz_tmap;          // (the tilemap's 512 x 256 tiles: the tile's pixels' offset | its pen base << 20)
static uint32_t s_roz_n;
static int32_t s_gain_l, s_gain_r;   // (the K054321's volume, x 4096)
static int s_vis_x0, s_vis_w;
static uint8_t *s_rom, *s_zrom, *s_sprrom;
static uint32_t s_rom_size, s_zrom_size, s_sprrom_size;
static z80 s_z;
static const uint8_t *s_zmap[256];
static k053260_t s_k260;
static k052109_t s_k109;
static eepser_t s_eeprom;
static int s_port_p[4], s_port_coins, s_port_eep;

static struct {
	uint16_t spriteram[0x2000];   // 180000: the RAM around the K053245's
	uint16_t k245_ram[0x400], k245_buffer[0x400];
	uint8_t k244_regs[16];
	uint8_t k53251[16];
	uint16_t protram[0x10];
	uint8_t ram1c05[0x80];
	uint8_t zram[0x2000];   // (prmrsocr c000-dfff; the others f000-f7ff)
	uint8_t z80_irq, nmi_line, dim_c, dim_v, lastirq, tile_rombank;
	uint8_t k54000[0x20];
	uint8_t k51960_ram[0x400], k51960_buffer[0x400];   // (punkshot, thndrx2: the K051960)
	uint8_t k51960_control, k51960_shadow, k51960_rombank[3], k51960_romoffset;
	int64_t k51960_busy_until;
	uint32_t rand_state;
	uint16_t roz_ctrl[16], roz_line[0x800];   // (prmrsocr: the K053936)
	uint8_t latch[3], k54321_active, k54321_volume, sound_bank, sprite_bank, roz_char_bank, nmi_prev;
	uint32_t rombank;
	int64_t nmi_due;              // (the Z80 cycle of the next NMI: SH1 rising after the arm, -1 none)
	int64_t cycles_done;
	uint64_t cyc_frac, z_frac, z_end, ym_frac, snd_frac;
	uint32_t frame;
	int vpos;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot (screen_update) ----
static struct {
	uint8_t k53251[16], k244_regs[16], dim_c, dim_v, tile_rombank;
	uint16_t buffer[0x400];
	uint8_t k51960_buffer[0x400], k51960_control, k51960_shadow, sprite_bank;
	uint16_t roz_ctrl[16], roz_line[0x800];
	k052109_t k109;
} s_vs;
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

// ---- the video callbacks (tmnt2_base_state, tmnt2_k053245_base_state) ----
static void tile_cb(int layer, int bank, int *code, int *color, int *flags, int *priority)
{
	(void)flags;
	(void)priority;
	const int c = *color;
	if (s_mach == M_BLSWHSTL)   // (blswhstl_tile_callback)
		*code |= ((c & 0x01) << 8) | ((c & 0x10) << 5) | ((c & 0x0c) << 8) | (bank << 12) | s_vs.tile_rombank << 14;
	else
		*code |= ((c & 0x03) << 8) | ((c & 0x10) << 6) | ((c & 0x0c) << 9) | (bank << 13);
	*color = s_layer_colorbase[layer] + ((c & 0xe0) >> 5);
}

static void sprite_cb(int *code, int *color, int *primask)   // (lgtnfght_sprite_callback)
{
	(void)code;
	const int pri = 0x20 | ((*color & 0x60) >> 2);
	if (pri <= s_layerpri[2])
		*primask = 0;
	else if (pri <= s_layerpri[1])
		*primask = 0xf0;
	else if (pri <= s_layerpri[0])
		*primask = 0xf0 | 0xcc;
	else
		*primask = 0xf0 | 0xcc | 0xaa;
	*color = s_sprite_colorbase + (*color & 0x1f);
	if (s_mach == M_PRMRSOCR)   // (prmrsocr_sprite_callback)
		*code |= s_vs.sprite_bank << 14;
}

// (punkshot_ / thndrx2_sprite_callback: the K051960's)
static void sprite_cb51(int *code, int *color, int *priority, int *shadow)
{
	(void)shadow;
	const int pri = 0x20 | ((*color & 0x60) >> 2);
	if (pri <= s_layerpri[2])
		*priority = 0;
	else if (pri <= s_layerpri[1])
		*priority = 0xf0;
	else if (pri <= s_layerpri[0])
		*priority = 0xf0 | 0xcc;
	else
		*priority = 0xf0 | 0xcc | 0xaa;
	if (s_mach == M_PUNKSHOT)
		*code |= (*color & 0x10) << 9;
	*color = s_sprite_colorbase + (*color & 0x0f);
}

// ---- prmrsocr's K053936 (zoom_draw: flags 0, priority 1, glfgreat_hack 1, offsets -11 1, no wrap) over its ROM
// tilemap of 512 x 256 16 x 16 tiles (prmrsocr_get_roz_tile_info: the gfx's colours from 0x400) ----
#define ROZ_XOFF (-11)
#define ROZ_YOFF 1
static void roz_line(uint16_t *frame, uint8_t *pri, int y, int x0, int x1, uint32_t cx, uint32_t cy, int incxx, int incxy)
{
	uint16_t *o = frame + (y - VIS_Y0) * s_vis_w - s_vis_x0;
	uint8_t *pr = pri + (y - VIS_Y0) * s_vis_w - s_vis_x0;
	const uint32_t *tm = s_roz_tmap;
	const uint8_t *gfx = s_roz_gfx;
	if (incxx == 0x10000 && incxy == 0) {   // (a pixel a pixel along a map row)
		if (cy >= (4096u << 16))
			return;
		const uint32_t py = cy >> 16;
		const uint32_t *trow = tm + (py >> 4) * 512;
		for (int x = x0; x <= x1; x++, cx += 0x10000u) {
			if (cx >= (8192u << 16))
				continue;
			const uint32_t px = cx >> 16, e = trow[px >> 4];
			const int p = gfx[(e & 0xfffff) + (py & 15) * 16 + (px & 15)];
			if (p) {
				o[x] = (uint16_t)((e >> 20) + (uint32_t)p);
				pr[x] |= 1;
			}
		}
		return;
	}
	for (int x = x0; x <= x1; x++, cx += (uint32_t)incxx, cy += (uint32_t)incxy) {
		if (cx >= (8192u << 16) || cy >= (4096u << 16))
			continue;
		const uint32_t px = cx >> 16, py = cy >> 16, e = tm[(py >> 4) * 512 + (px >> 4)];
		const int p = gfx[(e & 0xfffff) + (py & 15) * 16 + (px & 15)];
		if (p) {
			o[x] = (uint16_t)((e >> 20) + (uint32_t)p);
			pr[x] |= 1;
		}
	}
}

static void roz_draw(void)
{
	if (!s_roz_gfx || !s_roz_tmap)
		return;
	const uint16_t *ctrl = s_vs.roz_ctrl;
	uint16_t *frame = gx_pre_frame16();
	uint8_t *pri = gx_pre_pri();
	const int cl = s_vis_x0, cr = s_vis_x0 + s_vis_w - 1, ct = VIS_Y0, cb = VIS_Y0 + VIS_H - 1;
	if (ctrl[0x07] & 0x0040) {   // (per line)
		int minx = cl, maxx = cr, y = ct, maxy = cb;
		if ((ctrl[0x07] & 0x0002) && ctrl[0x09]) {   // (the glfgreat hack's clip)
			minx = ctrl[0x08] + ROZ_XOFF + 2;
			maxx = ctrl[0x09] + ROZ_XOFF + 2 - 1;
			if (minx < cl) minx = cl;
			if (maxx > cr) maxx = cr;
			y = ctrl[0x0a] + ROZ_YOFF - 2;
			if (y < ct) y = ct;
			maxy = ctrl[0x0b] + ROZ_YOFF - 2 - 1;
			if (maxy > cb) maxy = cb;
		}
		if (minx > maxx)
			return;
		for (; y <= maxy; y++) {
			const uint16_t *la = s_vs.roz_line + 4 * ((y - ROZ_YOFF) & 0x1ff);
			uint32_t startx = (uint32_t)(256 * (int16_t)(la[0] + ctrl[0x00]));
			uint32_t starty = (uint32_t)(256 * (int16_t)(la[1] + ctrl[0x01]));
			int incxx = (int16_t)la[2], incxy = (int16_t)la[3];
			if (ctrl[0x06] & 0x8000) incxx *= 256;
			if (ctrl[0x06] & 0x0080) incxy *= 256;
			startx -= (uint32_t)(ROZ_XOFF * incxx);
			starty -= (uint32_t)(ROZ_XOFF * incxy);
			roz_line(frame, pri, y, minx, maxx, (startx << 5) + (uint32_t)minx * (uint32_t)(incxx << 5),
			         (starty << 5) + (uint32_t)minx * (uint32_t)(incxy << 5), incxx << 5, incxy << 5);
		}
	} else {   // ("simple" mode)
		uint32_t startx = (uint32_t)(256 * (int16_t)ctrl[0x00]), starty = (uint32_t)(256 * (int16_t)ctrl[0x01]);
		int incyx = (int16_t)ctrl[0x02], incyy = (int16_t)ctrl[0x03], incxx = (int16_t)ctrl[0x04], incxy = (int16_t)ctrl[0x05];
		if (ctrl[0x06] & 0x4000) { incyx *= 256; incyy *= 256; }
		if (ctrl[0x06] & 0x0040) { incxx *= 256; incxy *= 256; }
		startx -= (uint32_t)(ROZ_YOFF * incyx);
		starty -= (uint32_t)(ROZ_YOFF * incyy);
		startx -= (uint32_t)(ROZ_XOFF * incxx);
		starty -= (uint32_t)(ROZ_XOFF * incxy);
		startx <<= 5; starty <<= 5; incxx <<= 5; incxy <<= 5; incyx <<= 5; incyy <<= 5;
		startx += (uint32_t)cl * (uint32_t)incxx + (uint32_t)ct * (uint32_t)incyx;
		starty += (uint32_t)cl * (uint32_t)incxy + (uint32_t)ct * (uint32_t)incyy;
		for (int y = ct; y <= cb; y++, startx += (uint32_t)incyx, starty += (uint32_t)incyy)
			roz_line(frame, pri, y, cl, cr, startx, starty, incxx, incxy);
	}
}

static void capture(void)
{
	memcpy(s_vs.k53251, t.k53251, sizeof(s_vs.k53251));
	memcpy(s_vs.k244_regs, t.k244_regs, sizeof(s_vs.k244_regs));
	memcpy(s_vs.buffer, t.k245_buffer, sizeof(s_vs.buffer));
	s_vs.dim_c = t.dim_c;
	s_vs.dim_v = t.dim_v;
	s_vs.tile_rombank = t.tile_rombank;
	memcpy(s_vs.k51960_buffer, t.k51960_buffer, sizeof(s_vs.k51960_buffer));
	s_vs.k51960_control = t.k51960_control;
	s_vs.k51960_shadow = t.k51960_shadow;
	s_vs.sprite_bank = t.sprite_bank;
	if (s_mach == M_PRMRSOCR) {
		memcpy(s_vs.roz_ctrl, t.roz_ctrl, sizeof(s_vs.roz_ctrl));
		memcpy(s_vs.roz_line, t.roz_line, sizeof(s_vs.roz_line));
	}
	memcpy(&s_vs.k109, &s_k109, offsetof(k052109_t, state_end));
}

static void draw(void)   // (ssriders_state::screen_update: screen_update_lgtnfght, then the palette's dimming)
{
	static const int ci_lg[3] = { 2, 4, 3 }, ci_roz[3] = { 2, 3, 4 };   // (K053251 CI2, CI4, CI3; the ROZ boards CI2, CI3, CI4)
	const int *ci = s_mach == M_PRMRSOCR ? ci_roz : ci_lg;
	const uint8_t *r = s_vs.k53251;
	const int dims = s_mach == M_TMNT2 || s_mach == M_SSRIDERS;   // (ssriders_state::screen_update)
	gx.pre_highlight = dims && !(s_vs.dim_c & 0x10);   // (set_shadow_mode(~dim_c bit 4))
	const int bg_colorbase = k53251_palette_index(r, 0);
	s_sprite_colorbase = k53251_palette_index(r, 1);
	int layer[3];
	for (int i = 0; i < 3; i++) {
		s_layer_colorbase[i] = k53251_palette_index(r, ci[i]);
		if (s_mach == M_PRMRSOCR && i == 1)
			s_layer_colorbase[i] += 8;   // ("weird")
		layer[i] = i;
		s_layerpri[i] = r[ci[i]];
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
	k052109_prepare(&s_vs.k109, &s_view, s_vis_x0 + (s_vis_x0 + s_vis_w - 1) + 1, VIS_Y0 + (VIS_Y0 + VIS_H - 1) + 1);
	uint16_t *f = gx_pre_frame16();
	uint8_t *p = gx_pre_pri();
	static const uint8_t prio[3] = { 1, 2, 4 };
	if (s_mach == M_PUNKSHOT)   // (screen_update_punkshot: the first layer opaque, no background)
		memset(p, 0, (size_t)(s_vis_w * VIS_H));
	else
		gx_pre_fill_pen(16 * bg_colorbase);
	for (int i = 0; i < 3; i++) {
		k052109_draw(&s_vs.k109, &s_view, layer[i], (s_mach == M_PUNKSHOT && !i) ? K052109_DRAW_OPAQUE : 0, prio[i], f, p,
		             s_vis_w, s_vis_x0, VIS_Y0, s_vis_x0, VIS_Y0, s_vis_x0 + s_vis_w - 1, VIS_Y0 + VIS_H - 1);
		// (the ROZ after the last layer of priority 0x30 or more)
		if (s_mach == M_PRMRSOCR && s_layerpri[i] >= 0x30 && (i == 2 || s_layerpri[i + 1] < 0x30))
			roz_draw();
	}
	if (s_mach == M_PRMRSOCR)
		gx_pre_sprites_053245(s_vs.buffer, s_vs.k244_regs, 0, 0, -1);
	else if (s_mach >= M_PUNKSHOT)
		gx_pre_sprites_051960(s_vs.k51960_buffer, s_vs.k51960_control, s_vs.k51960_shadow, -1, -1, sprite_cb51, 0);
	else   // (lgtnfght_state's video_start: set_z_rejection(0); blswhstl keeps the -1)
		gx_pre_sprites_053245(s_vs.buffer, s_vs.k244_regs, 0, 0, s_mach == M_BLSWHSTL ? -1 : 0);
	if (!dims)
		return;
	// the dimming: layer 1 (CI4); with the sprites (CI1) and layer 2 (CI3) unless the K053251's priority 5 bit 4
	const int enable = r[5];
	int dimpal = 0;
	if (enable != 0x3e) {
		dimpal |= 1 << (s_layer_colorbase[1] >> 4 & 7);
		if (!(enable & 0x10)) {
			for (int i = 0; i < 2; i++)
				dimpal |= 1 << ((s_sprite_colorbase >> 4 | i) & 7);
			dimpal |= 1 << (s_layer_colorbase[2] >> 4 & 7);
		}
	}
	const int factor = s_vs.dim_v | ((~s_vs.dim_c & 0x10) >> 1);
	const double brt = 1.0 - (1.0 - 0.6) * factor / 8;
	for (int i = 0; i < 8; i++)
		gx.pre_contrast[i] = (float)((dimpal >> i & 1) ? brt : 1.0);
}

// ---- the K053244 / K053245 ----
static uint8_t k053244_r(int offset)
{
	offset &= 0xf;
	if (offset == 6)
		memcpy(t.k245_buffer, t.k245_ram, sizeof(t.k245_buffer));   // (update_buffer)
	else if (offset == 7)
		for (int i = 0; i < 0x400; i += 8)   // (clear_buffer)
			t.k245_buffer[i] = 0;
	else if (offset >= 0x0c) {
		uint32_t addr = t.rombank << 19 | (uint32_t)(t.k244_regs[11] & 7) << 18 | (uint32_t)t.k244_regs[8] << 10 |
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

// the K053245's RAM inside the larger one: A1, A5 and A6 not wired to it
static void spriteram_w(uint32_t o, uint16_t v, uint16_t mask)
{
	t.spriteram[o & 0x1fff] = (uint16_t)((v & mask) | (t.spriteram[o & 0x1fff] & ~mask));
	if (!(o & 0x0031)) {
		uint16_t *k = &t.k245_ram[(((o & 0x000e) >> 1) | ((o & 0x1fc0) >> 3)) & 0x3ff];
		*k = (uint16_t)((v & mask) | (*k & ~mask));
	}
}
static uint16_t spriteram_r(uint32_t o)
{
	if (s_tmnt2 || (o & 0x0031))   // (tmnt2: the RAM itself)
		return t.spriteram[o & 0x1fff];
	return t.k245_ram[(((o & 0x000e) >> 1) | ((o & 0x1fc0) >> 3)) & 0x3ff];
}

// ---- the sound board a frame behind on the worker (rbx_aux), as asterix.c's ----
#define Q_MAX 2048
enum { Q_ADV, Q_LATCH, Q_IRQ };
typedef struct { uint16_t kind, arg; uint32_t v; } q_ev_t;
static q_ev_t s_q[2][Q_MAX];
static int s_qn[2], s_qcur;
static int32_t s_out_l[2048], s_out_r[2048];
static int s_out_n, s_out_pos;
static uint8_t s_pub[2];   // (the K053260's ports 2 and 3 as of the last job)
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

// ---- the protection ----
static inline uint16_t rd16(uint32_t a) { return (uint16_t)m68k_read_memory_16(a & 0xffffff); }

static uint16_t ssriders_protection_r(void)   // (simulation derived from a bootleg)
{
	int data = rd16(0x105a0a);
	const int cmd = rd16(0x1058fc);
	switch (cmd) {
	case 0x100b: return 0x0064;
	case 0x6003: return (uint16_t)(data & 0x000f);
	case 0x6004: return (uint16_t)(data & 0x001f);
	case 0x6000: return (uint16_t)(data & 0x0001);
	case 0x0000: return (uint16_t)(data & 0x00ff);
	case 0x6007: return (uint16_t)(data & 0x00ff);
	case 0x8abc:   // (collision table)
		data = -(int)rd16(0x105818);
		data = ((data / 8 - 4) & 0x1f) * 0x40;
		data += (((int)rd16(0x105cb0) + (int)rd16(0x1040c8) - 6) / 8 + 12) & 0x3f;
		return (uint16_t)data;
	default: return 0xffff;
	}
}

static void ssriders_protection_w(int offset)
{
	if (offset != 1)
		return;
	int hardware_pri = 1;   // (the sprites' priority attributes)
	for (int logical_pri = 1; logical_pri < 0x100; logical_pri <<= 1)
		for (int i = 0; i < 128; i++)
			if ((rd16(0x180006 + 128 * i) >> 8) == logical_pri) {
				uint16_t *k = &t.k245_ram[(8 * i) & 0x3ff];
				*k = (uint16_t)((hardware_pri & 0x00ff) | (*k & 0xff00));
				hardware_pri++;
			}
}

static uint32_t tmnt2_get_word(uint32_t addr)
{
	if (addr <= 0x07ffff / 2)
		return rd16(addr * 2);
	if (addr >= 0x104000 / 2 && addr <= 0x107fff / 2)
		return rd16(addr * 2);
	if (addr >= 0x180000 / 2 && addr <= 0x183fff / 2)
		return t.spriteram[addr - 0x180000 / 2];
	return 0;
}
static void tmnt2_put_word(uint32_t addr, uint16_t data)
{
	if (addr >= 0x180000 / 2 && addr <= 0x183fff / 2) {
		const uint32_t offs = addr - 0x180000 / 2;
		t.spriteram[offs] = data;
		if (!(offs & 0x0031))
			t.k245_ram[(((offs & 0x000e) >> 1) | ((offs & 0x1fc0) >> 3)) & 0x3ff] = data;
	} else if (addr >= 0x104000 / 2 && addr <= 0x107fff / 2)
		m68k_write_memory_16(addr * 2, data);
}

static void tmnt2_prot_w(int offset, uint16_t data, uint16_t mask)   // (the 053990's sprite builder)
{
	t.protram[offset & 0xf] = (uint16_t)((data & mask) | (t.protram[offset & 0xf] & ~mask));
	if (offset != 0x18 / 2 || !(mask & 0xff00))
		return;
	const uint16_t *pr = t.protram;
	if ((pr[8] & 0xff00) != 0x8200)
		return;
	const uint32_t src_addr = (pr[0] | (uint32_t)(pr[1] & 0xff) << 16) >> 1;
	const uint32_t dst_addr = (pr[2] | (uint32_t)(pr[3] & 0xff) << 16) >> 1;
	const uint32_t mod_addr = (pr[4] | (uint32_t)(pr[5] & 0xff) << 16) >> 1;
	const int zlock = (pr[8] & 0xff) == 0x0001;
	uint16_t src[4], mod[24];
	for (int i = 0; i < 4; i++)
		src[i] = (uint16_t)tmnt2_get_word(src_addr + (uint32_t)i);
	for (int i = 0; i < 24; i++)
		mod[i] = (uint16_t)tmnt2_get_word(mod_addr + (uint32_t)i);
	const uint32_t code = src[0];
	const int f1 = src[1];
	uint32_t attr1 = (uint32_t)(f1 >> 2 & 0x3f00);
	uint32_t attr2 = (uint32_t)(f1 & 0x380);
	const uint32_t cbase = (uint32_t)(f1 & 0x01f), cmod = mod[0x2a / 2] >> 8;
	const uint32_t color = (cbase != 0x0f && cmod <= 0x1f && !zlock) ? cmod : cbase;
	int xoffs = (int16_t)src[2], yoffs = (int16_t)src[3];
	const int f2 = mod[0];
	attr2 |= (uint32_t)(f2 & 0x0060);
	const int keepaspect = (f2 & 0x0014) == 0x0014;
	if (f2 & 0x8000) attr1 |= 0x8000;
	if (keepaspect) attr1 |= 0x4000;
	if (f2 & 0x4000) { attr1 ^= 0x1000; xoffs = -xoffs; }
	const int xmod = (int16_t)mod[6], ymod = (int16_t)mod[7], zmod = (int16_t)mod[8];
	const int xzoom = mod[0x1c / 2], yzoom = keepaspect ? xzoom : mod[0x1e / 2];
	const int xlock = (f2 & 0x003b) == 0x0020, ylock = xlock;
	if (!xlock) {
		int z = xzoom - 0x4f00;
		if (z > 0) {
			z >>= 8;
			xoffs += (int)(pow(z, 1.891292) * xoffs / 599.250121);
		} else if (z < 0) {
			z = (z >> 3) + (z >> 4) + (z >> 5) + (z >> 6) + xzoom;
			xoffs = (z > 0) ? (xoffs * z / 0x4f00) : 0;
		}
	}
	if (!ylock) {
		int z = yzoom - 0x4f00;
		if (z > 0) {
			z >>= 8;
			yoffs += (int)(pow(z, 1.891292) * yoffs / 599.250121);
		} else if (z < 0) {
			z = (z >> 3) + (z >> 4) + (z >> 5) + (z >> 6) + yzoom;
			yoffs = (z > 0) ? (yoffs * z / 0x4f00) : 0;
		}
	}
	if (!zlock)
		yoffs += zmod;
	xoffs += xmod;
	yoffs += ymod;
	tmnt2_put_word(dst_addr + 0, (uint16_t)attr1);
	tmnt2_put_word(dst_addr + 2, (uint16_t)code);
	tmnt2_put_word(dst_addr + 4, (uint16_t)yoffs);
	tmnt2_put_word(dst_addr + 6, (uint16_t)xoffs);
	tmnt2_put_word(dst_addr + 12, (uint16_t)(attr2 | color));
}

// the K054000 (k054000_device: the collision of two boxes; blswhstl)
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

// ---- the main CPU's bus ----
static inline int64_t now_cycles(void) { return t.cycles_done + m68k_cycles_run(); }

static void irq_lines(void) { m68k_bus_set_line(s_irq_level, s_k109.irq_out); }   // (the K052109's IRQ)

// k052109_word_r / _w (and the noA12 games' mirror): the high byte the offset, the low one + 0x2000
static uint16_t k109_word_r(int o) { return (uint16_t)(k052109_read(&s_k109, o) << 8); }
static uint8_t k109_byte_r(int o, int lo) { return k052109_read(&s_k109, o + (lo ? 0x2000 : 0)); }
static void k109_word_w(int o, uint16_t v, uint16_t mask)
{
	if (mask & 0xff00)
		k052109_write(&s_k109, o, (uint8_t)(v >> 8));
	else
		k052109_write(&s_k109, o + 0x2000, (uint8_t)v);
	irq_lines();
}
static inline int noA12(int o) { return ((o & 0x3000) >> 1) | (o & 0x07ff); }

static uint16_t k244_pair_r(int o)   // (k053244_word_noA1: the register pair, mirrored)
{
	o &= ~1;
	return (uint16_t)(k053244_r(o + 1) | k053244_r(o) << 8);
}
static void k244_pair_w(int o, uint16_t v, uint16_t mask)
{
	o &= ~1;
	if (mask & 0xff00) k053244_w(o, (uint8_t)(v >> 8));
	if (mask & 0x00ff) k053244_w(o + 1, (uint8_t)v);
}

static void eeprom_lines(uint16_t v)   // (EEPROMOUT: bit 0 data, bit 1 cs, bit 2 clock)
{
	const int64_t tc = now_cycles();
	eepser_di_write(&s_eeprom, v & 1);
	eepser_cs_write(&s_eeprom, v >> 1 & 1, tc);
	eepser_clk_write(&s_eeprom, v >> 2 & 1, tc);
}

// ---- the K051960 / K051937 ----
static uint8_t k051960_fetchromdata(int offset)
{
	int addr = t.k51960_romoffset + (t.k51960_rombank[0] << 8) + ((t.k51960_rombank[1] & 0x03) << 16);
	int code = (addr & 0x3ffe0) >> 5, color = ((t.k51960_rombank[1] & 0xfc) >> 2) + ((t.k51960_rombank[2] & 0x03) << 6);
	const int off1 = addr & 0x1f;
	int pri = 0, shadow = 0;
	sprite_cb51(&code, &color, &pri, &shadow);
	addr = (code << 7) | (off1 << 2) | offset;
	return s_sprrom[(uint32_t)addr & (s_sprrom_size - 1)];
}
static uint8_t k051960_r(int offset)
{
	if (t.k51960_control & 0x20) {   // (the last address read used for the sprite ROMs)
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
	if (offset == 0)   // (the sprite DMA's busy flag)
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
static void k051960_vblank(void)   // (vblank_callback: the sprite DMA, its busy time)
{
	if (t.k51960_control & 0x10)
		return;
	memcpy(t.k51960_buffer, t.k51960_ram, sizeof(t.k51960_buffer));
	int active = 0;
	for (int i = 0; i < 0x400; i += 8)
		active += t.k51960_buffer[i] >> 7;
	const uint32_t ticks = (uint32_t)(active * 32 + (128 - active) * 18);   // (24 MHz ticks x 4)
	t.k51960_busy_until = now_cycles() + (int64_t)((uint64_t)ticks * 4 * MAIN_CLOCK / 24000000u);
}

// ---- prmrsocr: the K054321 (its volume, the latches) ----
static void k54321_volume(void)
{
	const double vol = pow(2.0, ((double)t.k54321_volume - 40.0) / 10.0);
	s_gain_l = (t.k54321_active & 2) ? (int32_t)(vol * 4096.0 + 0.5) : 0;
	s_gain_r = (t.k54321_active & 1) ? (int32_t)(vol * 4096.0 + 0.5) : 0;
}
static void k54321_main_w(int r, uint8_t v)
{
	switch (r) {
	case 0x0: t.k54321_active = v; k54321_volume(); break;
	case 0x2: t.k54321_volume = 0; k54321_volume(); break;
	case 0x3:
		if (v && t.k54321_volume < 64) {
			t.k54321_volume++;
			k54321_volume();
		}
		break;
	case 0x6: t.latch[0] = v; break;
	case 0x7: t.latch[1] = v; break;
	}
}

// prmrsocr_main_map
static uint16_t pr_r16(uint32_t a)
{
	if (a >= 0x104000 && a < 0x108000)
		return spriteram_r((a - 0x104000) >> 1);
	if (a >= 0x10c000 && a < 0x10d000)
		return t.roz_line[(a - 0x10c000) >> 1];
	if (a >= 0x114000 && a < 0x114020)   // (the K053244: umask 00ff)
		return k053244_r((int)((a - 0x114000) >> 1));
	if (a == 0x120000)
		return (uint16_t)rbx_port_read(s_port_p[0]);
	if (a == 0x120002) {
		rbx_port_custom(s_port_eep, 0x300,
		                (uint32_t)(eepser_do_read(&s_eeprom) | eepser_ready_read(&s_eeprom, now_cycles()) << 1) << 8);
		return (uint16_t)rbx_port_read(s_port_eep);
	}
	if (a >= 0x121000 && a < 0x121020) {   // (the K054321: busy 0, latch 2 as of the last job)
		const int r = (int)((a - 0x121000) >> 1);
		return r == 0xa ? s_pub[0] : r == 0x8 ? 0 : 0xff;
	}
	if (a >= 0x200000 && a < 0x208000)
		return k109_word_r(noA12((int)((a - 0x200000) >> 1)));
	if (a >= 0x300000 && a < 0x340000) {   // (prmrsocr_rom_r)
		const uint32_t o = (a - 0x300000) >> 1;
		if (t.roz_char_bank)
			return o < s_zoom_size ? s_zoom[o] : 0;
		return (uint16_t)(256 * (o < s_zoomtmap_size ? s_zoomtmap[o] : 0) +
		                  (o + 0x20000 < s_zoomtmap_size ? s_zoomtmap[o + 0x20000] : 0));
	}
	return 0;
}
static void pr_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if (a >= 0x104000 && a < 0x108000)
		spriteram_w((a - 0x104000) >> 1, v, mask);
	else if (a >= 0x10c000 && a < 0x10d000) {
		uint16_t *q = &t.roz_line[(a - 0x10c000) >> 1];
		*q = (uint16_t)((v & mask) | (*q & ~mask));
	} else if (a >= 0x110000 && a < 0x110020)
		k244_pair_w((int)((a - 0x110000) >> 1), v, mask);
	else if (a >= 0x114000 && a < 0x114020) {
		if (mask & 0x00ff)
			k053244_w((int)((a - 0x114000) >> 1), (uint8_t)v);
	} else if (a >= 0x118000 && a < 0x118020) {
		uint16_t *q = &t.roz_ctrl[(a - 0x118000) >> 1];
		*q = (uint16_t)((v & mask) | (*q & ~mask));
	} else if (a >= 0x11c000 && a < 0x11c020) {   // (the K053251: umask ff00)
		if (mask & 0xff00)
			t.k53251[((a - 0x11c000) >> 1) & 0xf] = (uint8_t)((v >> 8) & 0x3f);
	} else if (a >= 0x121000 && a < 0x121020) {
		if (mask & 0x00ff)
			q_put(Q_LATCH, (int)((a - 0x121000) >> 1), (uint8_t)v);
	} else if (a == 0x122000) {   // (prmrsocr_eeprom_w: the low byte 122000_w, the high one the EEPROM)
		if (mask & 0x00ff) {
			k052109_set_rmrd_line(&s_k109, (v >> 4) & 1);
			t.sprite_bank = (v >> 6) & 1;
			t.rombank = (uint32_t)t.sprite_bank << 2;   // (bankselect)
			t.roz_char_bank = (v >> 7) & 1;
		}
		if (mask & 0xff00)
			eeprom_lines(v >> 8);
	} else if (a == 0x123000)
		q_put(Q_IRQ, 0, 0);   // (prmrsocr_sound_irq_w)
	else if (a >= 0x200000 && a < 0x208000)
		k109_word_w(noA12((int)((a - 0x200000) >> 1)), v, mask);
}

// punkshot_main_map
static uint16_t ps_r16(uint32_t a)
{
	if (a == 0x0a0000)
		return (uint16_t)rbx_port_read(s_port_dsw[0]);
	if (a == 0x0a0002)
		return (uint16_t)rbx_port_read(s_port_coins);
	if (a == 0x0a0004)
		return (uint16_t)rbx_port_read(s_port_p[1]);
	if (a == 0x0a0006)
		return (uint16_t)rbx_port_read(s_port_p[0]);
	if (a >= 0x0a0040 && a < 0x0a0044)
		return (((a - 0x0a0040) >> 1) & 1) ? s_pub[1] : s_pub[0];
	if (a >= 0x100000 && a < 0x108000)
		return k109_word_r(noA12((int)((a - 0x100000) >> 1)));
	if (a >= 0x110000 && a < 0x110008)   // (the K051937: byte wide)
		return (uint16_t)(k051937_r((int)(a - 0x110000)) << 8 | k051937_r((int)(a - 0x110000) + 1));
	if (a >= 0x110400 && a < 0x110800)
		return (uint16_t)(k051960_r((int)(a - 0x110400)) << 8 | k051960_r((int)(a - 0x110400) + 1));
	if (a >= 0xfffffc) {   // (punkshot_kludge_r: a random number)
		t.rand_state = t.rand_state * 1103515245u + 12345u;
		return (uint16_t)(t.rand_state >> 16);
	}
	return 0;
}
static void ps_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if (a == 0x0a0020 && (mask & 0x00ff)) {   // (punkshot_0a0020_w: the sound IRQ on bit 2's fall, RMRD)
		if (t.lastirq && !(v & 4))
			q_put(Q_IRQ, 0, 0);
		t.lastirq = (v >> 2) & 1;
		k052109_set_rmrd_line(&s_k109, (v >> 3) & 1);
	} else if (a >= 0x0a0040 && a < 0x0a0044) {
		if (mask & 0x00ff)
			q_put(Q_LATCH, (int)((a - 0x0a0040) >> 1), (uint8_t)v);
	} else if (a >= 0x0a0060 && a < 0x0a0080) {
		if (mask & 0x00ff)
			t.k53251[((a - 0x0a0060) >> 1) & 0xf] = (uint8_t)(v & 0x3f);
	} else if (a >= 0x100000 && a < 0x108000)
		k109_word_w(noA12((int)((a - 0x100000) >> 1)), v, mask);
	else if (a >= 0x110000 && a < 0x110008) {
		if (mask & 0xff00) k051937_w((int)(a - 0x110000), (uint8_t)(v >> 8));
		if (mask & 0x00ff) k051937_w((int)(a - 0x110000) + 1, (uint8_t)v);
	} else if (a >= 0x110400 && a < 0x110800) {
		if (mask & 0xff00) t.k51960_ram[a - 0x110400] = (uint8_t)(v >> 8);
		if (mask & 0x00ff) t.k51960_ram[a - 0x110400 + 1] = (uint8_t)v;
	}
}

// thndrx2_main_map
static uint16_t tx_r16(uint32_t a)
{
	if (a >= 0x400000 && a < 0x400004)
		return (((a - 0x400000) >> 1) & 1) ? s_pub[1] : s_pub[0];
	if (a >= 0x500000 && a < 0x500040)
		return k54000_r((int)((a - 0x500000) >> 1) & 0x1f);
	if (a == 0x500200)
		return (uint16_t)rbx_port_read(s_port_p[0]);
	if (a == 0x500202) {
		rbx_port_custom(s_port_eep, 0x300,
		                (uint32_t)(eepser_do_read(&s_eeprom) | eepser_ready_read(&s_eeprom, now_cycles()) << 1) << 8);
		return (uint16_t)rbx_port_read(s_port_eep);
	}
	if (a >= 0x600000 && a < 0x608000)
		return k109_word_r(noA12((int)((a - 0x600000) >> 1)));
	if (a >= 0x700000 && a < 0x700008)
		return (uint16_t)(k051937_r((int)(a - 0x700000)) << 8 | k051937_r((int)(a - 0x700000) + 1));
	if (a >= 0x700400 && a < 0x700800)
		return (uint16_t)(k051960_r((int)(a - 0x700400)) << 8 | k051960_r((int)(a - 0x700400) + 1));
	return 0;
}
static void tx_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if (a >= 0x300000 && a < 0x300020) {
		if (mask & 0x00ff)
			t.k53251[((a - 0x300000) >> 1) & 0xf] = (uint8_t)(v & 0x3f);
	} else if (a >= 0x400000 && a < 0x400004) {
		if (mask & 0x00ff)
			q_put(Q_LATCH, (int)((a - 0x400000) >> 1), (uint8_t)v);
	} else if (a >= 0x500000 && a < 0x500040) {
		if (mask & 0x00ff)
			t.k54000[((a - 0x500000) >> 1) & 0x1f] = (uint8_t)v;
	} else if (a == 0x500100 && (mask & 0x00ff)) {   // (thndrx2_eeprom_w: the EEPROM, the sound IRQ on bit 5's rise, RMRD)
		eeprom_lines(v);
		if (!t.lastirq && (v & 0x20))
			q_put(Q_IRQ, 0, 0);
		t.lastirq = (v >> 5) & 1;
		k052109_set_rmrd_line(&s_k109, (v >> 6) & 1);
	} else if (a >= 0x600000 && a < 0x608000)
		k109_word_w(noA12((int)((a - 0x600000) >> 1)), v, mask);
	else if (a >= 0x700000 && a < 0x700008) {
		if (mask & 0xff00) k051937_w((int)(a - 0x700000), (uint8_t)(v >> 8));
		if (mask & 0x00ff) k051937_w((int)(a - 0x700000) + 1, (uint8_t)v);
	} else if (a >= 0x700400 && a < 0x700800) {
		if (mask & 0xff00) t.k51960_ram[a - 0x700400] = (uint8_t)(v >> 8);
		if (mask & 0x00ff) t.k51960_ram[a - 0x700400 + 1] = (uint8_t)v;
	}
}

// lgtnfght_main_map
static uint16_t lg_r16(uint32_t a)
{
	if (a >= 0x0a0000 && a < 0x0a000a) {
		static const int idx[5] = { -1, 0, 1, -2, -3 };   // (COINS, P1, P2, DSW1, DSW2)
		const int i = idx[(a - 0x0a0000) >> 1];
		return (uint16_t)rbx_port_read(i == -1 ? s_port_coins : i >= 0 ? s_port_p[i] : s_port_dsw[-i - 2]);
	}
	if (a == 0x0a0010)
		return (uint16_t)rbx_port_read(s_port_dsw[2]);
	if (a >= 0x0a0020 && a < 0x0a0024)
		return (((a - 0x0a0020) >> 1) & 1) ? s_pub[1] : s_pub[0];
	if (a >= 0x0b0000 && a < 0x0b4000)
		return spriteram_r((a - 0x0b0000) >> 1);
	if (a >= 0x0c0000 && a < 0x0c0020)
		return k244_pair_r((int)((a - 0x0c0000) >> 1));
	if (a >= 0x100000 && a < 0x108000)
		return k109_word_r(noA12((int)((a - 0x100000) >> 1)));
	return 0;
}
static void lg_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if (a == 0x0a0018 && (mask & 0x00ff)) {   // (lgtnfght_0a0018_w: the sound IRQ on bit 2's rise, RMRD)
		if (!t.lastirq && (v & 4))
			q_put(Q_IRQ, 0, 0);
		t.lastirq = (v >> 2) & 1;
		k052109_set_rmrd_line(&s_k109, (v >> 3) & 1);
	} else if (a >= 0x0a0020 && a < 0x0a0024) {
		if (mask & 0x00ff)
			q_put(Q_LATCH, (int)((a - 0x0a0020) >> 1), (uint8_t)v);
	} else if (a >= 0x0b0000 && a < 0x0b4000)
		spriteram_w((a - 0x0b0000) >> 1, v, mask);
	else if (a >= 0x0c0000 && a < 0x0c0020)
		k244_pair_w((int)((a - 0x0c0000) >> 1), v, mask);
	else if (a >= 0x0e0000 && a < 0x0e0020) {
		if (mask & 0x00ff)
			t.k53251[((a - 0x0e0000) >> 1) & 0xf] = (uint8_t)(v & 0x3f);
	} else if (a >= 0x100000 && a < 0x108000)
		k109_word_w(noA12((int)((a - 0x100000) >> 1)), v, mask);
}

// blswhstl_main_map
static uint16_t bw_r16(uint32_t a)
{
	if (a >= 0x180000 && a < 0x184000)
		return k109_word_r((int)((a - 0x180000) >> 1));
	if (a >= 0x300000 && a < 0x304000)
		return spriteram_r((a - 0x300000) >> 1);
	if (a >= 0x500000 && a < 0x500040)
		return k54000_r((int)((a - 0x500000) >> 1) & 0x1f);
	if (a >= 0x680000 && a < 0x680020)
		return k244_pair_r((int)((a - 0x680000) >> 1));
	if (a == 0x700000 || a == 0x700002)
		return (uint16_t)rbx_port_read(s_port_p[(a - 0x700000) >> 1]);
	if (a == 0x700004)
		return (uint16_t)rbx_port_read(s_port_coins);
	if (a == 0x700006) {
		rbx_port_custom(s_port_eep, 0x03,
		                (uint32_t)(eepser_do_read(&s_eeprom) | eepser_ready_read(&s_eeprom, now_cycles()) << 1));
		return (uint16_t)rbx_port_read(s_port_eep);
	}
	if (a >= 0x780600 && a < 0x780604)
		return (((a - 0x780600) >> 1) & 1) ? s_pub[1] : s_pub[0];
	return 0;
}
static void bw_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if (a >= 0x180000 && a < 0x184000)
		k109_word_w((int)((a - 0x180000) >> 1), v, mask);
	else if (a >= 0x300000 && a < 0x304000)
		spriteram_w((a - 0x300000) >> 1, v, mask);
	else if (a >= 0x500000 && a < 0x500040) {
		if (mask & 0x00ff)
			t.k54000[((a - 0x500000) >> 1) & 0x1f] = (uint8_t)v;
	} else if (a >= 0x680000 && a < 0x680020)
		k244_pair_w((int)((a - 0x680000) >> 1), v, mask);
	else if (a == 0x700200 && (mask & 0x00ff))
		eeprom_lines(v);
	else if (a == 0x700300 && (mask & 0x00ff)) {   // (blswhstl_700300_w: RMRD, the tile ROM bank)
		k052109_set_rmrd_line(&s_k109, (v >> 3) & 1);
		t.tile_rombank = (v >> 7) & 1;
	} else if (a >= 0x780600 && a < 0x780604) {
		if (mask & 0x00ff)
			q_put(Q_LATCH, (int)((a - 0x780600) >> 1), (uint8_t)v);
	} else if (a == 0x780604)
		q_put(Q_IRQ, 0, 0);   // (ssriders_soundkludge_w)
	else if (a >= 0x780700 && a < 0x780720) {
		if (mask & 0x00ff)
			t.k53251[((a - 0x780700) >> 1) & 0xf] = (uint8_t)(v & 0x3f);
	}
}

static uint16_t main_r16(uint32_t a)
{
	a &= 0xffffff;
	if (s_mach == M_LGTNFGHT)
		return lg_r16(a);
	if (s_mach == M_BLSWHSTL)
		return bw_r16(a);
	if (s_mach == M_PUNKSHOT)
		return ps_r16(a);
	if (s_mach == M_THNDRX2)
		return tx_r16(a);
	if (s_mach == M_PRMRSOCR)
		return pr_r16(a);
	if (a >= 0x180000 && a < 0x184000)
		return spriteram_r((a - 0x180000) >> 1);
	if (a >= 0x1c0000 && a < 0x1c0008)
		return (uint16_t)rbx_port_read(s_port_p[(a - 0x1c0000) >> 1]);
	if (a == 0x1c0100)
		return (uint16_t)rbx_port_read(s_port_coins);
	if (a == 0x1c0102) {   // (the EEPROM's lines, the vblank active low)
		const int vblank = t.vpos < VIS_Y0 || t.vpos >= VIS_Y0 + VIS_H;
		rbx_port_custom(s_port_eep, 0x0b,
		                (uint32_t)(eepser_do_read(&s_eeprom) | eepser_ready_read(&s_eeprom, now_cycles()) << 1 | !vblank << 3));
		return (uint16_t)rbx_port_read(s_port_eep);
	}
	if (a >= 0x1c0500 && a < 0x1c0580)
		return (uint16_t)(t.ram1c05[a - 0x1c0500] << 8 | t.ram1c05[a - 0x1c0500 + 1]);
	if (a == 0x1c0800 && !s_tmnt2)
		return ssriders_protection_r();
	if (a >= 0x5a0000 && a < 0x5a0020) {   // (k053244_word_noA1: the register pair, mirrored)
		const int o = (int)((a - 0x5a0000) >> 1) & ~1;
		return (uint16_t)(k053244_r(o + 1) | k053244_r(o) << 8);
	}
	if (a >= 0x5c0600 && a < 0x5c0604)   // (the K053260's main side: umask 00ff)
		return (((a - 0x5c0600) >> 1) & 1) ? s_pub[1] : s_pub[0];
	if (a >= 0x600000 && a < 0x604000)   // (k052109_word_r: a word read the high byte's)
		return (uint16_t)(k052109_read(&s_k109, (int)((a - 0x600000) >> 1)) << 8);
	return 0;
}
static uint8_t main_r8(uint32_t a)
{
	a &= 0xffffff;
	if (s_mach == M_LGTNFGHT && a >= 0x100000 && a < 0x108000)
		return k109_byte_r(noA12((int)((a - 0x100000) >> 1)), a & 1);
	if (s_mach == M_BLSWHSTL && a >= 0x180000 && a < 0x184000)
		return k109_byte_r((int)((a - 0x180000) >> 1), a & 1);
	if (s_mach == M_PUNKSHOT && a >= 0x100000 && a < 0x108000)
		return k109_byte_r(noA12((int)((a - 0x100000) >> 1)), a & 1);
	if (s_mach == M_THNDRX2 && a >= 0x600000 && a < 0x608000)
		return k109_byte_r(noA12((int)((a - 0x600000) >> 1)), a & 1);
	if (s_mach == M_PUNKSHOT && a >= 0x110000 && a < 0x110800)   // (the K051937 / K051960: byte wide)
		return a < 0x110008 ? k051937_r((int)(a - 0x110000)) : a >= 0x110400 ? k051960_r((int)(a - 0x110400)) : 0;
	if (s_mach == M_PRMRSOCR && a >= 0x200000 && a < 0x208000)
		return k109_byte_r(noA12((int)((a - 0x200000) >> 1)), a & 1);
	if (s_mach == M_THNDRX2 && a >= 0x700000 && a < 0x700800)
		return a < 0x700008 ? k051937_r((int)(a - 0x700000)) : a >= 0x700400 ? k051960_r((int)(a - 0x700400)) : 0;
	if (s_mach <= M_SSRIDERS && a >= 0x600000 && a < 0x604000)   // (k052109_word_r: the low byte + 0x2000)
		return k109_byte_r((int)((a - 0x600000) >> 1), a & 1);
	const uint16_t w = main_r16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}

static void main_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (s_mach == M_LGTNFGHT) {
		lg_w16m(a, v, mask);
		return;
	}
	if (s_mach == M_BLSWHSTL) {
		bw_w16m(a, v, mask);
		return;
	}
	if (s_mach == M_PUNKSHOT) {
		ps_w16m(a, v, mask);
		return;
	}
	if (s_mach == M_THNDRX2) {
		tx_w16m(a, v, mask);
		return;
	}
	if (s_mach == M_PRMRSOCR) {
		pr_w16m(a, v, mask);
		return;
	}
	if (a >= 0x180000 && a < 0x184000)
		spriteram_w((a - 0x180000) >> 1, v, mask);
	else if (a == 0x1c0200 && (mask & 0x00ff)) {   // (ssriders_eeprom_w: the EEPROM, the dimming's mode, the ROM bank)
		const int64_t tc = now_cycles();
		eepser_di_write(&s_eeprom, v & 1);
		eepser_cs_write(&s_eeprom, v >> 1 & 1, tc);
		eepser_clk_write(&s_eeprom, v >> 2 & 1, tc);
		t.dim_c = (uint8_t)(v & 0x18);
		t.rombank = (uint32_t)((v >> 5) & 1) << 2;   // (bankselect)
	} else if (a == 0x1c0300 && (mask & 0x00ff)) {   // (ssriders_1c0300_w: RMRD, the dimming)
		k052109_set_rmrd_line(&s_k109, (v >> 3) & 1);
		t.dim_v = (uint8_t)((v & 0x70) >> 4);
	} else if (a >= 0x1c0500 && a < 0x1c0580) {
		if (mask & 0xff00) t.ram1c05[a - 0x1c0500] = (uint8_t)(v >> 8);
		if (mask & 0x00ff) t.ram1c05[a - 0x1c0500 + 1] = (uint8_t)v;
	} else if (a >= 0x1c0800 && a < 0x1c0820) {
		if (s_tmnt2)
			tmnt2_prot_w((int)((a - 0x1c0800) >> 1), v, mask);
		else if (a < 0x1c0804)
			ssriders_protection_w((int)((a - 0x1c0800) >> 1));
	} else if (a >= 0x5a0000 && a < 0x5a0020) {
		const int o = (int)((a - 0x5a0000) >> 1) & ~1;
		if (mask & 0xff00) k053244_w(o, (uint8_t)(v >> 8));
		if (mask & 0x00ff) k053244_w(o + 1, (uint8_t)v);
	} else if (a >= 0x5c0600 && a < 0x5c0604) {
		if (mask & 0x00ff)
			q_put(Q_LATCH, (int)((a - 0x5c0600) >> 1), (uint8_t)v);
	} else if (a == 0x5c0604)
		q_put(Q_IRQ, 0, 0);   // (ssriders_soundkludge_w)
	else if (a >= 0x5c0700 && a < 0x5c0720) {   // (the K053251: umask 00ff)
		if (mask & 0x00ff)
			t.k53251[((a - 0x5c0700) >> 1) & 0xf] = (uint8_t)(v & 0x3f);
	} else if (a >= 0x600000 && a < 0x604000)   // (k052109_word_w: the MSB alone on word writes)
		k109_word_w((int)((a - 0x600000) >> 1), v, mask);
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v)
{
	main_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00);
}

// ---- the sound board (ssriders_audio_map) ----
static void pr_z_bank(void)   // (audiobank: 8 entries of 0x4000 from the ROM's start)
{
	const uint32_t base = (uint32_t)(t.sound_bank & 7) * 0x4000;
	for (int p = 0; p < 0x40; p++)
		s_zmap[0x80 + p] = base + (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + base + (uint32_t)p * 256 : NULL;
}

static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (s_mach == M_PRMRSOCR) {
		if (a < 0x8000)
			return a < s_zrom_size ? s_zrom[a] : 0;
		if (a < 0xc000) {
			const uint32_t o = (uint32_t)(t.sound_bank & 7) * 0x4000 + (a - 0x8000);
			return o < s_zrom_size ? s_zrom[o] : 0;
		}
		if (a < 0xe000)
			return t.zram[a - 0xc000];
		if (a < 0xe130)
			return k054539_read(&s_k539, (uint32_t)(((a & 0x100) << 1) | (a & 0xff)));
		if (a == 0xf002)
			return t.latch[0];
		if (a == 0xf003)
			return t.latch[1];
		return 0;
	}
	if (s_mach == M_LGTNFGHT) {   // (lgtnfght_audio_map)
		if (a < 0x8000)
			return a < s_zrom_size ? s_zrom[a] : 0xff;
		if (a < 0x8800)
			return t.zram[a - 0x8000];
		if (a == 0xa000 || a == 0xa001)
			return rbx_ym_read(0, a & 1);
		if (a >= 0xc000 && a < 0xc030)
			return k053260_read(&s_k260, a - 0xc000);
		return 0;   // (unmapped: MAME's 0, which the stack at ffff relies on: the reset's NMI returns through it)
	}
	if (s_mach >= M_PUNKSHOT) {   // (punkshot / thndrx2_audio_map: ROM to 7fff / efff, the YM2151 mirrored at f810)
		if (a < (s_mach == M_PUNKSHOT ? 0x8000 : 0xf000))
			return a < s_zrom_size ? s_zrom[a] : 0;
		if (a >= 0xf000 && a < 0xf800)
			return t.zram[a - 0xf000];
		if ((a & ~(s_mach == M_THNDRX2 ? 0x11 : 0x01)) == 0xf800)
			return rbx_ym_read(0, a & 1);
		if (a >= 0xfc00 && a < 0xfc30)
			return k053260_read(&s_k260, a - 0xfc00);
		return 0;
	}
	if (a < 0xf000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0xf800)
		return t.zram[a - 0xf000];
	if (a == 0xf800 || a == 0xf801)
		return rbx_ym_read(0, a & 1);
	if (a >= 0xfa00 && a < 0xfa30)
		return k053260_read(&s_k260, a - 0xfa00);
	return 0;   // (unmapped: MAME's 0)
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_mach == M_PRMRSOCR) {
		if (a >= 0xc000 && a < 0xe000)
			t.zram[a - 0xc000] = v;
		else if (a >= 0xe000 && a < 0xe130)
			k054539_write(&s_k539, (uint32_t)(((a & 0x100) << 1) | (a & 0xff)), v);
		else if (a == 0xf000)
			t.latch[2] = v;
		else if (a == 0xf800) {
			t.sound_bank = v;
			pr_z_bank();
		}
		return;
	}
	if (s_mach == M_LGTNFGHT) {
		if (a >= 0x8000 && a < 0x8800)
			t.zram[a - 0x8000] = v;
		else if (a == 0xa000 || a == 0xa001)
			rbx_ym_write(0, a & 1, v);
		else if (a >= 0xc000 && a < 0xc030)
			k053260_write(&s_k260, a - 0xc000, v);
		return;
	}
	const int pk = s_mach >= M_PUNKSHOT;   // (punkshot / thndrx2: the K053260 at fc00, the NMI's arm at fa00)
	if (a >= 0xf000 && a < 0xf800)
		t.zram[a - 0xf000] = v;
	else if (a == 0xf800 || a == 0xf801 || (s_mach == M_THNDRX2 && (a == 0xf810 || a == 0xf811)))
		rbx_ym_write(0, a & 1, v);
	else if (!pk && a >= 0xfa00 && a < 0xfa30)
		k053260_write(&s_k260, a - 0xfa00, v);
	else if (pk && a >= 0xfc00 && a < 0xfc30)
		k053260_write(&s_k260, a - 0xfc00, v);
	else if (a == (pk ? 0xfa00 : 0xfc00)) {   // (sound_arm_nmi_w: NMI cleared, blocked 4 cycles; then the next SH1 rise asserts it)
		t.nmi_line = 0;
		const uint64_t tb = ((uint64_t)s_z.cyc + 4) * CHIP_CLOCK / Z80_CLOCK + 1;
		const uint64_t rise = tb + k053260_sh1_next_rise(tb) - 1;
		t.nmi_due = (int64_t)((rise * Z80_CLOCK + CHIP_CLOCK - 1) / CHIP_CLOCK);
		z80_break = 1;
	}
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void k054539_nmi(void *user, int state)   // (prmrsocr: the timer the Z80's NMI line)
{
	(void)user;
	if (state && !t.nmi_prev)
		z80_gen_nmi(&s_z);
	t.nmi_prev = (uint8_t)state;
}

// prmrsocr: n samples, the Z80 a sample's cycles at a time (the K054539's timer between), the K054539 at 1.0
static void pr_sound_run(int n)
{
	for (int i = 0; i < n; i++) {
		t.z_frac += Z80_CLOCK;
		const unsigned long cyc = (unsigned long)(t.z_frac / SND_RATE);
		t.z_frac -= (uint64_t)cyc * SND_RATE;
		int line = t.z80_irq;
		t.z_end += cyc;
		z80_run(&s_z, (unsigned long)t.z_end, &line, 1);
		t.z80_irq = (uint8_t)line;
		int32_t l = 0, r = 0;
		k054539_render(&s_k539, &l, &r, 1);
		if (s_out_pos < 2048) {
			s_out_l[s_out_pos] = (int32_t)((int64_t)l * s_gain_l >> 12);
			s_out_r[s_out_pos] = (int32_t)((int64_t)r * s_gain_r >> 12);
			s_out_pos++;
		}
	}
}

// n samples (a line's): the Z80 (up to its NMIs), the K053260 and the YM2151 (its samples spread over the line's)
static void sound_run(int n)
{
	if (n <= 0)
		return;
	if (s_mach == M_PRMRSOCR) {
		pr_sound_run(n);
		return;
	}
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
	const int yn = rbx_ym_advance_lr(0, yc, yl, yr, 96);
	for (int i = 0; i < n; i++) {
		const int k = yn > 0 ? i * yn / n : -1;
		if (s_out_pos < 2048) {
			if (s_mach == M_PUNKSHOT) {   // (mono: the YM2151 at 1.0, the K053260 at 0.70, all their outputs)
				const int32_t m = (k >= 0 ? yl[k] + yr[k] : 0) + (l[i] + r[i]) * 7 / 10;
				s_out_l[s_out_pos] = s_out_r[s_out_pos] = m;
			} else if (s_mach == M_THNDRX2) {   // (the YM2151 at 0.25, the K053260 at 0.50)
				s_out_l[s_out_pos] = (k >= 0 ? yl[k] : 0) / 4 + l[i] / 2;
				s_out_r[s_out_pos] = (k >= 0 ? yr[k] : 0) / 4 + r[i] / 2;
			} else if (s_mach == M_BLSWHSTL) {   // (the YM2151 at 0.70, the K053260 at 0.50 with its channels inverted)
				s_out_l[s_out_pos] = (k >= 0 ? yl[k] : 0) * 7 / 10 + r[i] / 2;
				s_out_r[s_out_pos] = (k >= 0 ? yr[k] : 0) * 7 / 10 + l[i] / 2;
			} else {   // (the YM2151 at 1.0, the K053260 at 0.70)
				s_out_l[s_out_pos] = (k >= 0 ? yl[k] : 0) + l[i] * 7 / 10;
				s_out_r[s_out_pos] = (k >= 0 ? yr[k] : 0) + r[i] * 7 / 10;
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
		case Q_LATCH:
			if (s_mach == M_PRMRSOCR)
				k54321_main_w(e->arg, (uint8_t)e->v);
			else
				k053260_main_write(&s_k260, e->arg, (uint8_t)e->v);
			break;
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
	if (s_mach == M_PRMRSOCR)
		s_pub[0] = t.latch[2];
	else {
		s_pub[0] = k053260_main_read(&s_k260, 0);
		s_pub[1] = k053260_main_read(&s_k260, 1);
	}
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void tm_exit(void);
static void tm_reset(void);

static int tm_init(const rbx_game *g)
{
	uint32_t pcm_size, tile_size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	const uint8_t *pcm = rbx_region(!strcmp(g->machine, "prmrsocr") ? "k054539" : "k053260", &pcm_size);
	const uint8_t *tiles = rbx_region("k052109", &tile_size);
	s_sprrom = rbx_region("k053245", &s_sprrom_size);
	if (!s_sprrom)
		s_sprrom = rbx_region("k051960", &s_sprrom_size);   // (punkshot, thndrx2)
	if (!s_rom || !s_zrom || !pcm || !tiles || !s_sprrom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_mach = !strcmp(g->machine, "tmnt2") ? M_TMNT2 : !strcmp(g->machine, "lgtnfght") ? M_LGTNFGHT
	         : !strcmp(g->machine, "blswhstl") ? M_BLSWHSTL : !strcmp(g->machine, "punkshot") ? M_PUNKSHOT
	         : !strcmp(g->machine, "thndrx2") ? M_THNDRX2 : !strcmp(g->machine, "prmrsocr") ? M_PRMRSOCR : M_SSRIDERS;
	s_tmnt2 = s_mach == M_TMNT2;
	// (set_raw: tmnt2 0+8 to 320-8, ssriders 0+16 to 320-16, lgtnfght and blswhstl 0 to 320)
	// (punkshot, thndrx2: 0+16 to 320-16)
	s_vis_x0 = s_mach == M_TMNT2 ? 8 : s_mach == M_LGTNFGHT || s_mach == M_BLSWHSTL ? 0 : 16;
	s_vis_w = s_mach == M_TMNT2 ? 304 : s_mach == M_LGTNFGHT || s_mach == M_BLSWHSTL ? 320 : 288;
	MAIN_CLOCK = s_mach == M_LGTNFGHT || s_mach == M_PUNKSHOT || s_mach == M_THNDRX2 ? 12000000u : 16000000u;
	Z80_CLOCK = s_mach <= M_SSRIDERS || s_mach == M_PRMRSOCR ? 8000000u : CHIP_CLOCK;
	s_irq_level = s_mach == M_LGTNFGHT || s_mach == M_PRMRSOCR ? 5 : 4;
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
	gx.pre_dim = 1;
	for (int i = 0; i < 8; i++)
		gx.pre_contrast[i] = 1.0f;
	k055673_init(&gx.k55673, s_sprrom, s_sprrom_size,
	             s_mach == M_PUNKSHOT || s_mach == M_THNDRX2 ? K055673_LAYOUT_051960 : K055673_LAYOUT_053245, 0, 0);
	if (s_mach == M_PRMRSOCR) {   // (zoom: gfx_16x16x4_packed_lsb; zoomtmap: the ROZ's tilemap)
		s_zoom = rbx_region("zoom", &s_zoom_size);
		s_zoomtmap = rbx_region("zoomtmap", &s_zoomtmap_size);
		if (!s_zoom || !s_zoomtmap || s_zoomtmap_size < 0x40000) {
			rbx_fail("%s: ROM regions missing", g->set);
			return -1;
		}
		s_roz_n = s_zoom_size / 128;
		if (!(s_roz_gfx = (uint8_t *)malloc((size_t)s_roz_n * 256 + 1))) {
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
		for (uint32_t i = 0; i < s_roz_n * 128; i++) {
			s_roz_gfx[i * 2] = s_zoom[i] & 15;
			s_roz_gfx[i * 2 + 1] = s_zoom[i] >> 4;
		}
		if (!(s_roz_tmap = (uint32_t *)malloc(512 * 256 * sizeof(uint32_t)))) {
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
		for (uint32_t ti = 0; ti < 512 * 256; ti++) {   // (prmrsocr_get_roz_tile_info: the colours from 0x400)
			const uint32_t code = (uint32_t)(s_zoomtmap[ti + 0x20000] + 256 * s_zoomtmap[ti]);
			const uint32_t tile = s_roz_n ? (code & 0x1fff) % s_roz_n : 0;
			s_roz_tmap[ti] = tile * 256 | (0x400 + (code >> 13) * 16) << 20;
		}
	}
	gx_video_set_area(s_vis_x0, s_vis_w);
	gx_video_set_y0(VIS_Y0);
	gx_video_set_height(VIS_H);
	if (k052109_init(&s_k109, tiles, tile_size, 2048, tile_cb)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	m68k_init();
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	// (the program: tmnt2 000000-0fffff, ssriders -0bffff, lgtnfght -03ffff, blswhstl -07ffff; not mirrored past it)
	m68k_bus_map(0x000000, (s_mach == M_TMNT2 ? 0x100000 : s_mach == M_SSRIDERS ? 0xc0000
	                        : s_mach == M_BLSWHSTL || s_mach == M_PRMRSOCR ? 0x80000 : 0x40000) - 1, s_rom, s_rom_size, 1, 0);
	if (s_mach == M_PRMRSOCR) {
		m68k_bus_map(0x100000, 0x103fff, gx.workram, 0x4000, 1, 1);
		m68k_bus_map(0x108000, 0x108fff, gx.palram, 0x1000, 1, 1);
	} else if (s_mach == M_PUNKSHOT) {
		m68k_bus_map(0x080000, 0x083fff, gx.workram, 0x4000, 1, 1);
		m68k_bus_map(0x090000, 0x090fff, gx.palram, 0x1000, 1, 1);
	} else if (s_mach == M_THNDRX2) {
		m68k_bus_map(0x100000, 0x103fff, gx.workram, 0x4000, 1, 1);
		m68k_bus_map(0x200000, 0x200fff, gx.palram, 0x1000, 1, 1);
	} else if (s_mach == M_LGTNFGHT) {
		m68k_bus_map(0x080000, 0x080fff, gx.palram, 0x1000, 1, 1);
		m68k_bus_map(0x090000, 0x093fff, gx.workram, 0x4000, 1, 1);
	} else if (s_mach == M_BLSWHSTL) {
		m68k_bus_map(0x204000, 0x207fff, gx.workram, 0x4000, 1, 1);
		m68k_bus_map(0x400000, 0x400fff, gx.palram, 0x1000, 1, 1);
	} else {
		m68k_bus_map(0x104000, 0x107fff, gx.workram, 0x4000, 1, 1);
		m68k_bus_map(0x140000, 0x140fff, gx.palram, 0x1000, 1, 1);
	}
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
	k053260_init(&s_k260, pcm, pcm_size, CHIP_CLOCK);
	if (s_mach == M_PRMRSOCR) {
		k054539_init(&s_k539, pcm, pcm_size);
		s_k539.timer_cb = k054539_nmi;
	}
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = CHIP_CLOCK;
	yc.gain[0] = yc.gain[1] = 256;
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: YM2151", g->set);
		return -1;
	}
	eepser_init(&s_eeprom, EEPSER_ER5911, 128, 8, MAIN_CLOCK);
	{
		uint32_t esz;
		const uint8_t *e = rbx_region("eeprom", &esz);
		if (e)
			eepser_default(&s_eeprom, e, esz);
	}
	eepser_nv_load(&s_eeprom, g->set);   // (the contents kept between runs)
	static const char *const pn[4] = { "P1", "P2", "P3", "P4" };
	for (int i = 0; i < 4; i++)
		s_port_p[i] = rbx_port_find(pn[i]);
	s_port_coins = rbx_port_find("COINS");
	s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	s_port_dsw[2] = rbx_port_find("DSW3");
	s_port_eep = rbx_port_find("EEPROM");
	if (s_mach == M_PUNKSHOT) {   // (DSW1_DSW2, COINS_DSW3, P1_P2, P3_P4)
		s_port_dsw[0] = rbx_port_find("DSW1_DSW2");
		s_port_coins = rbx_port_find("COINS_DSW3");
		s_port_p[0] = rbx_port_find("P1_P2");
		s_port_p[1] = rbx_port_find("P3_P4");
	} else if (s_mach == M_THNDRX2 || s_mach == M_PRMRSOCR) {
		s_port_p[0] = rbx_port_find("P1_COINS");
		s_port_eep = rbx_port_find("P2_EEPROM");
	}
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(s_vis_w, VIS_H, 0x10000) || gx_video_init()) {
		rbx_fail("%s: out of memory", g->set);
		tm_exit();
		return -1;
	}
	for (int i = 0; i < 0x10000; i++) {   // (the bitmap: RGB 565)
		const int r = (i >> 11) & 31, gg = (i >> 5) & 63, b = i & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (gg << 2 | gg >> 4) << 8 | (b << 3 | b >> 2));
	}
	tm_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void tm_exit(void)
{
	eepser_nv_save(&s_eeprom);
	rbx_aux_set_job(NULL);
	gx_video_exit();
	k052109_exit(&s_k109);
	rbx_ym_exit(0);
	free(s_roz_gfx);
	free(s_roz_tmap);
	s_roz_gfx = NULL;
	s_roz_tmap = NULL;
	s_roz_n = 0;
}

static void tm_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_pub[0] = s_pub[1] = 0;
	s_job_frame = 0;
	memset(gx.workram, 0, sizeof(gx.workram));
	memset(gx.palram, 0, sizeof(gx.palram));
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	k052109_reset(&s_k109);
	for (int i = 0; i < 5; i++)   // (the K051960's reset)
		k051937_w(i, 0);
	t.k51960_romoffset = 0;
	k053260_reset(&s_k260);
	if (s_mach == M_PRMRSOCR)
		k054539_reset(&s_k539);
	k54321_volume();
	rbx_ym_reset(0);
	z80_init(&s_z);
	s_z.read_byte = z_rd;
	s_z.write_byte = z_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	const int zrom_pages = s_mach == M_LGTNFGHT || s_mach == M_PUNKSHOT ? 0x80 : 0xf0,
	          zram_page = s_mach == M_LGTNFGHT ? 0x80 : 0xf0;
	for (int p = 0; p < 256; p++)
		s_zmap[p] = p < zrom_pages && (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
	for (int p = 0; p < 8; p++)
		s_zmap[zram_page + p] = t.zram + p * 256;
	s_z.read_map = s_zmap;
	if (s_mach == M_PRMRSOCR) {   // (prmrsocr_audio_map: RAM c000-dfff, the bank)
		for (int pg = 0xc0; pg < 0x100; pg++)
			s_zmap[pg] = pg < 0xe0 ? t.zram + (pg - 0xc0) * 256 : NULL;
		pr_z_bank();
	}
	t.nmi_due = -1;
	t.nmi_line = 1;   // (machine_reset: the Z80's NMI goes low with the reset; prmrsocr has no K053260)
	if (s_mach != M_PRMRSOCR)
		z80_gen_nmi(&s_z);
	m68k_bus_irq_reset();
	m68k_pulse_reset();
}

static void tm_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		t.vpos = vpos;
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, the K052109's IRQ, the K051960's sprite DMA)
			gx_video_capture();
			if (s_mach >= M_PUNKSHOT)
				k051960_vblank();
			k052109_vblank(&s_k109, 1);
			irq_lines();
		}
		if (vpos == VIS_Y0)   // (the vblank's end: the K052109 latches its scroll)
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
		if (ns > 0)
			q_put(Q_ADV, 0, (uint32_t)ns);
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
	rc |= rbx_state_io(ctx, "tm_cpu", cpu, (uint32_t)m68k_context_size());
	rc |= rbx_state_io(ctx, "tm_bus", &m68k_bus.irq_pending, sizeof(uint32_t) * 2);
	rc |= rbx_state_io(ctx, "gx", &gx, (uint32_t)offsetof(gx_t, state_end));
	rc |= k052109_state(&s_k109, ctx, load, "tm_k109");
	rc |= rbx_state_io(ctx, "tm", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= eepser_state(&s_eeprom, ctx, load, "tm_eeprom");
	rc |= k053260_state(&s_k260, ctx, load, "tm_k260_");
	rc |= rbx_ym_state(0, ctx, load);
	if (s_mach == M_PRMRSOCR)
		rc |= k054539_state(&s_k539, ctx, load, "tm_k539_");
	int32_t qv[3] = { s_out_n, s_pub[0], s_pub[1] };
	rc |= rbx_state_io(ctx, "tm_q", qv, sizeof(qv));
	rc |= rbx_state_io(ctx, "tm_outl", s_out_l, sizeof(s_out_l));
	rc |= rbx_state_io(ctx, "tm_outr", s_out_r, sizeof(s_out_r));
	if (load && !rc) {
		s_out_n = qv[0] >= 0 && qv[0] <= 2048 ? qv[0] : 0;
		s_pub[0] = (uint8_t)qv[1];
		s_pub[1] = (uint8_t)qv[2];
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
	}
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "tm_z80", &z, sizeof(z));
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
		if (s_mach == M_PRMRSOCR)
			pr_z_bank();
		k54321_volume();
	}
	return rc ? -1 : 0;
}

static int tm_save(void *ctx) { return tm_state(ctx, 0); }
static int tm_load(void *ctx) { return tm_state(ctx, 1); }
static int tm_supported(const char *machine)
{
	return !strcmp(machine, "tmnt2") || !strcmp(machine, "ssriders") || !strcmp(machine, "lgtnfght") ||
	       !strcmp(machine, "blswhstl") || !strcmp(machine, "punkshot") || !strcmp(machine, "thndrx2") ||
	       !strcmp(machine, "prmrsocr");
}

const rbx_driver rbx_drv_tmnt2 = { "Konami TMNT 2", tm_init, tm_exit, tm_reset, tm_frame, tm_save, tm_load,
                                   tm_supported, gx_render, NULL };
