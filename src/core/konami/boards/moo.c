/*  Konami Wild West C.O.W.-Boys of Moo Mesa (and Bucky O'Hare; Xexex: src/mame/konami/xexex.cpp, BSD-3-Clause,
    copyright-holders: Olivier Galibert, with its K053250 road layer). Ported from MAME src/mame/konami/moo.cpp (BSD-3-Clause,
    copyright-holders: R. Belmont, Acho A. Tang) and the devices it uses (k053251.cpp: Nicola Salmoria; k054321.cpp:
    Olivier Galibert; eepromser.cpp: Aaron Giles). A 68000 at 16 MHz, the K056832 tilemaps (4 bpp), the K053246/K053247
    sprites, the K053251 priorities and colour bases, the K054338 background colour, shadows and alpha (drawn by
    konamigx_v.c's pre-GX mode, MAME's bitmap_rgb32), the protection's helper, an ER5911 EEPROM; the sound: a Z80 at
    8 MHz with a banked ROM, a YM2151, a K054539, the K054321 latches.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../machine/eepromser.h"
#include "../../sound/k054539.h"
#include "../../sound/rbx_ym.h"
#include "../video/k056832.h"
#include "../video/k055673.h"
#include "../video/k053250.h"
#include "konamigx.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define MAIN_CLOCK 16000000u
#define Z80_CLOCK 8000000u
#define YM_CLOCK 4000000u
#define SND_RATE 48000
#define LINES 256          // (set_refresh_hz(60), set_size(64*8, 32*8))
#define VIS_H 224
#define DMA_CYCLES (MAIN_CLOCK / 1000000 * 100)   // (MOO_DMADELAY: 100 us)

static int s_bucky;   // (bucky_map, VIDEO_START bucky, the K054000)
static int s_xexex;   // (xexex_state: its map, timing, screen update, K053250, YM2151 filters)
static k053250_t s_lvc;
static int32_t s_ym_gl = 256, s_ym_gr = 256;   // (xexex: the YM2151's filters, ym_set_mixing x 256)
static uint8_t *s_extra;   // (bucky: the extra RAM at 0a0000 and 184000)
static uint8_t *s_rom, *s_zrom, *s_sprrom;
static uint32_t s_rom_size, s_zrom_size, s_sprrom_size;
static z80 s_z;
static const uint8_t *s_zmap[256];
static k054539_t s_k539;
static eepser_t s_eeprom;
static int s_port_in0, s_port_in1, s_port_p13, s_port_p24;
static int32_t s_gain_l, s_gain_r;

static struct {
	uint16_t spriteram[0x8000];   // 190000: the object DMA's source
	uint16_t protram[16];
	uint8_t k53251[16];
	uint8_t zram[0x2000];
	uint8_t latch[3], k54321_active, k54321_volume, z80_irq, sound_bank;
	uint16_t control2;
	uint8_t k54000[0x20];          // (bucky: the K054000's registers)
	uint8_t cur_alpha, suspended;  // (xexex: the alpha of plane 1; the main CPU spinning until an interrupt)
	uint8_t ym_pan;                // (xexex: the K054539's analog pan, register 0x13f)
	int64_t cycles_done, dma_due;
	uint64_t cyc_frac, z_frac, z_end, snd_frac, ym_frac;
	uint32_t frame;
	uint8_t state_end;
} m;

// ---- the drawing's snapshot and state (screen_update) ----
static struct { uint8_t k53251[16]; } s_vs;
static int s_layer_colorbase[4], s_layerpri[3], s_sprite_colorbase;

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

// ---- the K054321 ----
static void k54321_volume(void)
{
	const double vol = pow(2.0, ((double)m.k54321_volume - 40.0) / 10.0);
	s_gain_l = (m.k54321_active & 2) ? (int32_t)(vol * 4096.0 + 0.5) : 0;
	s_gain_r = (m.k54321_active & 1) ? (int32_t)(vol * 4096.0 + 0.5) : 0;
}
static void k54321_main_w(int r, uint8_t v)
{
	switch (r) {
	case 0x0: m.k54321_active = v; k54321_volume(); break;
	case 0x2: m.k54321_volume = 0; k54321_volume(); break;
	case 0x3:
		if (v && m.k54321_volume < 64) {
			m.k54321_volume++;
			k54321_volume();
		}
		break;
	case 0x6: m.latch[0] = v; break;
	case 0x7: m.latch[1] = v; break;
	}
}
// ---- the sound board a frame behind on the worker (rbx_aux), as the GX's (machine/gx_sound.c): the host's writes
// queued with the samples before them, the job runs the frame through them; the host reads the board's reply as of
// the last finished frame (the same with or without the worker thread) ----
#define Q_MAX 2048
enum { Q_ADV, Q_K54321, Q_IRQ };
typedef struct { uint16_t kind, arg; uint32_t v; } q_ev_t;
static q_ev_t (*s_q_hp)[2][Q_MAX];
#define s_q (*s_q_hp)
static int s_qn[2], s_qcur;
static int32_t s_out_l[2048], s_out_r[2048];
static int s_out_n, s_out_pos;
static uint8_t s_pub;   // (latch 2 as of the last job)
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

static uint8_t k54321_main_r(int r) { return r == 0xa ? s_pub : r == 0x8 ? 0 : 0xff; }

// ---- the video callbacks (moo_base_state) ----
static void tile_cb(int layer, int *code, int *color, int *flip, int *category)
{
	(void)code;
	(void)flip;
	(void)category;
	*color = s_layer_colorbase[layer] | (*color >> 2 & 0x0f);
}

static void sprite_cb(int *code, int *color, int *primask)
{
	(void)code;
	const int pri = (*color & 0x03e0) >> 4;
	if (pri <= s_layerpri[2])
		*primask = 0;
	else if (pri <= s_layerpri[1])
		*primask = 0xf0;
	else if (pri <= s_layerpri[0])
		*primask = 0xf0 | 0xcc;
	else
		*primask = 0xf0 | 0xcc | 0xaa;
	*color = s_sprite_colorbase | (*color & 0x001f);
}

static uint8_t s_vs_alpha;
static void capture(void)
{
	memcpy(s_vs.k53251, m.k53251, sizeof(s_vs.k53251));
	s_vs_alpha = m.cur_alpha;
}

static void tile_cb_xexex(int layer, int *code, int *color, int *flip, int *category)   // (the alpha flag: category)
{
	(void)code;
	(void)flip;
	*category = *color & 1;   // (priority = color & 1: the alpha flag)
	*color = s_layer_colorbase[layer] | (*color >> 2 & 0x0f);
}

static int s_layerpri4[4];
static void sprite_cb_xexex(int *code, int *color, int *primask)
{
	(void)code;
	const int pri = (*color & 0x3e0) >> 4;
	if (pri <= s_layerpri4[3])
		*primask = 0;
	else if (pri <= s_layerpri4[2])
		*primask = 0xff00;
	else if (pri <= s_layerpri4[1])
		*primask = 0xff00 | 0xf0f0;
	else if (pri <= s_layerpri4[0])
		*primask = 0xff00 | 0xf0f0 | 0xcccc;
	else
		*primask = 0xff00 | 0xf0f0 | 0xcccc | 0xaaaa;
	*color = s_sprite_colorbase | (*color & 0x001f);
}

static void draw_xexex(void)   // (xexex_state::screen_update)
{
	s_sprite_colorbase = k53251_palette_index(s_vs.k53251, 0);
	const int bg_colorbase = k53251_palette_index(s_vs.k53251, 1);
	s_layer_colorbase[0] = 0x70;
	for (int plane = 1; plane < 4; plane++)
		s_layer_colorbase[plane] = k53251_palette_index(s_vs.k53251, plane + 1);
	int layer[4] = { 1, 2, 3, -1 };
	s_layerpri4[0] = s_vs.k53251[2];
	s_layerpri4[1] = s_vs.k53251[3];
	s_layerpri4[2] = s_vs.k53251[4];
	s_layerpri4[3] = s_vs.k53251[1];
	for (int a = 0; a < 3; a++)   // (konami_sortlayers4: swapped when pri[a] <= pri[b])
		for (int b = a + 1; b < 4; b++)
			if (s_layerpri4[a] <= s_layerpri4[b]) {
				int t = s_layerpri4[a]; s_layerpri4[a] = s_layerpri4[b]; s_layerpri4[b] = t;
				t = layer[a]; layer[a] = layer[b]; layer[b] = t;
			}
	gx_pre_shadows();
	gx_pre_fill();
	for (int plane = 0; plane < 4; plane++) {
		if (layer[plane] < 0)
			gx_pre_lvc(bg_colorbase, 1 << plane);
		else if (!s_vs_alpha || layer[plane] != 1)
			gx_pre_tilemap_cat(layer[plane], 1 << plane, 1, 0, 255);
		else
			gx_pre_tilemap_cat(layer[plane], 1 << plane, 0, 0, 255);
	}
	gx_pre_sprites();
	if (s_vs_alpha) {
		const int alpha = gx_pre_alpha_level(1) & 0xff;
		if (alpha > 0)
			gx_pre_tilemap_cat(1, 0, 0, 1, alpha);
	}
	gx_pre_tilemap_cat(0, 0, 1, 0, 255);
}

static void draw(void)   // (screen_update)
{
	s_sprite_colorbase = k53251_palette_index(s_vs.k53251, 0);
	s_layer_colorbase[0] = 0x70;
	for (int plane = 1; plane < 4; plane++)
		s_layer_colorbase[plane] = k53251_palette_index(s_vs.k53251, plane + 1);
	int layer[3];
	for (int i = 0; i < 3; i++) {
		layer[i] = i + 1;
		s_layerpri[i] = s_vs.k53251[i + 2];   // (get_priority: CI2, CI3, CI4)
	}
	for (int a = 0; a < 2; a++)   // (konami_sortlayers3: swapped when pri[a] < pri[b])
		for (int b = a + 1; b < 3; b++)
			if (s_layerpri[a] < s_layerpri[b]) {
				int t = s_layerpri[a]; s_layerpri[a] = s_layerpri[b]; s_layerpri[b] = t;
				t = layer[a]; layer[a] = layer[b]; layer[b] = t;
			}
	gx_pre_shadows();
	gx_pre_fill();
	if (s_layerpri[0] < s_vs.k53251[1])   // (bucky hides the back layer behind the background: CI1's priority)
		gx_pre_tilemap(layer[0], 1);
	gx_pre_tilemap(layer[1], 2);
	const uint16_t *k338 = gx_pre_k054338();
	const int alpha = (k338[15] & 0x02) ? gx_pre_alpha_level(1) & 0xff : 255;   // (K338_REG_CONTROL & K338_CTL_MIXPRI)
	if (alpha > 0)
		gx_pre_tilemap_alpha(layer[2], 4, alpha);
	gx_pre_sprites();
	gx_pre_tilemap(0, 0);
}

// ---- the main CPU's bus ----
static inline int64_t now_cycles(void) { return m.cycles_done + m68k_cycles_run(); }

static void object_dma(void)   // (moo_base_state::object_dma: zmask 0xffff, bucky 0x00ff)
{
	if (s_xexex) {   // (xexex_state::object_dma: every other word of a 0x40-word slot)
		uint16_t *dst = gx.k55673.ram;
		const uint16_t *src = m.spriteram;
		int num_inactive = 256;
		for (int counter = 256; counter; counter--, src += 0x40)
			if (*src & 0x8000) {
				for (int i = 0; i < 8; i++)
					dst[i] = src[i * 2];
				dst += 8;
				num_inactive--;
			}
		for (; num_inactive; num_inactive--, dst += 8)
			*dst = 0;
		return;
	}
	uint16_t *dst = gx.k55673.ram;
	const uint16_t *src = m.spriteram;
	const uint16_t zmask = s_bucky ? 0x00ff : 0xffff;
	int num_inactive = 256;
	for (int counter = 256; counter; counter--, src += 0x80)
		if ((*src & 0x8000) && (*src & zmask)) {
			memcpy(dst, src, 0x10);
			dst += 8;
			num_inactive--;
		}
	for (; num_inactive; num_inactive--, dst += 8)
		*dst = 0;
}

static uint8_t k053246_r(int offset)
{
	if (!gx.k55673.objcha_line)
		return 0;
	const uint8_t *r = gx.k55673.kx46_regs;
	uint32_t addr = (uint32_t)r[6] << 17 | (uint32_t)r[7] << 9 | (uint32_t)r[4] << 1 | (uint32_t)((offset & 1) ^ 1);
	return s_sprrom[addr & (s_sprrom_size - 1)];
}

static void moo_prot_w(int offset, uint16_t v, uint16_t mask)
{
	m.protram[offset & 15] = (uint16_t)((v & mask) | (m.protram[offset & 15] & ~mask));
	if ((offset & 15) != 0xc)
		return;
	uint32_t src1 = (uint32_t)(m.protram[1] & 0xff) << 16 | m.protram[0];
	uint32_t src2 = (uint32_t)(m.protram[3] & 0xff) << 16 | m.protram[2];
	uint32_t dst = (uint32_t)(m.protram[5] & 0xff) << 16 | m.protram[4];
	for (uint32_t length = m.protram[0xf]; length; length--, src1 += 2, src2 += 2, dst += 2) {
		const uint32_t a = (uint16_t)m68k_read_memory_16(src1), b = (uint16_t)m68k_read_memory_16(src2);
		m68k_write_memory_16(dst, (uint16_t)(a + 2 * b));
	}
}

// the K054000 (k054000_device: the collision of two boxes)
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
	const uint8_t *k = m.k54000;
	const uint32_t acx = (uint32_t)k54000_delta(k + 1), acy = (uint32_t)k54000_delta(k + 9);
	const uint32_t bcx = (uint32_t)(k[0x15] << 16 | k[0x16] << 8 | k[0x17]), bcy = (uint32_t)(k[0x11] << 16 | k[0x12] << 8 | k[0x13]);
	return (uint8_t)(k54000_axis(acx, bcx, k[6], k[0x0e]) | k54000_axis(acy, bcy, k[7], k[0x0f]));
}

static uint16_t main_r16(uint32_t a)
{
	a &= 0xffffff;
	if (s_xexex) {   // (xexex_state::main_map: what differs from moo_map)
		if (a == 0x080014 && (m68k_get_reg(NULL, M68K_REG_PC) & 0xffffff) == 0x1158) {   // (waitskip_r: spin until an IRQ)
			m.suspended = 1;
			m68k_end_timeslice();
		}
		if (a >= 0x090000 && a < 0x0a0000)
			return m.spriteram[((a - 0x090000) >> 1) & 0x3fff];
		if (a >= 0x0c6000 && a < 0x0c8000)
			return k053250_ram_r(&s_lvc, (int)((a - 0x0c6000) >> 1));
		if (a >= 0x0c8000 && a < 0x0c8010)
			return k053250_reg_r(&s_lvc, (int)((a - 0x0c8000) >> 1));
		if (a >= 0x180000 && a < 0x184000)
			return k056832_ram_word_r(&gx.k56832, (int)((a & 0x1fff) >> 1));
		if (a >= 0x190000 && a < 0x192000) {
			const k056832_t *k = &gx.k56832;
			const uint32_t addr = 0x2000u * (uint32_t)k->cur_gfx_banks + (a - 0x190000);
			return (uint16_t)((addr + 1 < k->rom_size ? k->rom[addr + 1] : 0) | (addr < k->rom_size ? k->rom[addr] : 0) << 8);
		}
		if (a >= 0x1a0000 && a < 0x1a2000)
			return k053250_rom_r(&s_lvc, (int)((a - 0x1a0000) >> 1));
		if (a >= 0x180000)
			return 0;
	}
	if (s_bucky) {   // (bucky_map: what differs from moo_map)
		if (a >= 0x090000 && a < 0x0a0000)
			return m.spriteram[(a - 0x090000) >> 1];
		if (a >= 0x0a0000 && a < 0x0b0000)
			return *(const uint16_t *)(s_extra + (a - 0x0a0000));
		if (a >= 0x0d2000 && a < 0x0d2040)
			return k54000_r((int)((a - 0x0d2000) >> 1));
		if (a >= 0x180000 && a < 0x184000)
			return k056832_ram_word_r(&gx.k56832, (int)((a & 0x1fff) >> 1));
		if (a >= 0x184000 && a < 0x188000)
			return *(const uint16_t *)(s_extra + 0x10000 + (a - 0x184000));
		if (a >= 0x190000 && a < 0x192000) {
			const k056832_t *k = &gx.k56832;
			const uint32_t addr = 0x2000u * (uint32_t)k->cur_gfx_banks + (a - 0x190000);
			return (uint16_t)((addr + 1 < k->rom_size ? k->rom[addr + 1] : 0) | (addr < k->rom_size ? k->rom[addr] : 0) << 8);
		}
		if (a >= 0x190000)
			return 0;
	}
	if (a == 0x0c4000)
		return (uint16_t)(k053246_r(0) << 8 | k053246_r(1));
	if (a >= 0x0d6000 && a < 0x0d6020)
		return k54321_main_r((int)((a - 0x0d6000) >> 1));   // (the reply as of the last frame)
	if (a == 0x0da000)
		return (uint16_t)rbx_port_read(s_port_p13);
	if (a == 0x0da002)
		return (uint16_t)rbx_port_read(s_port_p24);
	if (a == 0x0dc000)
		return (uint16_t)rbx_port_read(s_port_in0);
	if (a == 0x0dc002) {
		rbx_port_custom(s_port_in1, 0x03, (uint32_t)(eepser_do_read(&s_eeprom) | eepser_ready_read(&s_eeprom, now_cycles()) << 1));
		return (uint16_t)rbx_port_read(s_port_in1);
	}
	if (a == 0x0de000)
		return m.control2;
	if (a >= 0x190000 && a < 0x1a0000)
		return m.spriteram[(a - 0x190000) >> 1];
	if (a >= 0x1a0000 && a < 0x1a4000)
		return k056832_ram_word_r(&gx.k56832, (int)((a & 0x1fff) >> 1));
	if (a >= 0x1b0000 && a < 0x1b2000) {   // (rom_word_r)
		const k056832_t *k = &gx.k56832;
		const uint32_t addr = 0x2000u * (uint32_t)k->cur_gfx_banks + (a - 0x1b0000);
		return (uint16_t)((addr + 1 < k->rom_size ? k->rom[addr + 1] : 0) | (addr < k->rom_size ? k->rom[addr] : 0) << 8);
	}
	return 0;
}
static uint8_t main_r8(uint32_t a)
{
	const uint16_t w = main_r16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}

static void main_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (s_xexex) {
		if (a >= 0x090000 && a < 0x0a0000) {
			uint16_t *p = &m.spriteram[((a - 0x090000) >> 1) & 0x3fff];
			*p = (uint16_t)((v & mask) | (*p & ~mask));
			return;
		}
		if (a >= 0x0c6000 && a < 0x0c8000) {
			k053250_ram_w(&s_lvc, (int)((a - 0x0c6000) >> 1), v, mask);
			return;
		}
		if (a >= 0x0c8000 && a < 0x0c8010) {
			k053250_reg_w(&s_lvc, (int)((a - 0x0c8000) >> 1), v, mask, (int)m.frame);
			return;
		}
		if (a >= 0x180000 && a < 0x184000) {
			k056832_ram_word_w(&gx.k56832, (int)((a & 0x1fff) >> 1), v, mask);
			return;
		}
		if (a >= 0x180000)
			return;
		if (a == 0x0de000) {   // (control2_w: parse_control2)
			m.control2 = (uint16_t)((v & mask) | (m.control2 & ~mask));
			const int64_t t = now_cycles();
			eepser_di_write(&s_eeprom, m.control2 & 1);
			eepser_cs_write(&s_eeprom, m.control2 >> 1 & 1, t);
			eepser_clk_write(&s_eeprom, m.control2 >> 2 & 1, t);
			gx.k55673.objcha_line = (m.control2 >> 8) & 1;
			m.cur_alpha = !(m.control2 & 0x200);
			return;
		}
	}
	if (s_bucky) {
		uint16_t *p = NULL;
		if (a >= 0x090000 && a < 0x0a0000)
			p = &m.spriteram[(a - 0x090000) >> 1];
		else if (a >= 0x0a0000 && a < 0x0b0000)
			p = (uint16_t *)(s_extra + (a - 0x0a0000));
		else if (a >= 0x184000 && a < 0x188000)
			p = (uint16_t *)(s_extra + 0x10000 + (a - 0x184000));
		else if (a >= 0x0d2000 && a < 0x0d2040) {
			if (mask & 0x00ff)
				m.k54000[((a - 0x0d2000) >> 1) & 0x1f] = (uint8_t)v;
			return;
		} else if (a >= 0x180000 && a < 0x184000) {
			k056832_ram_word_w(&gx.k56832, (int)((a & 0x1fff) >> 1), v, mask);
			return;
		} else if (a >= 0x190000)
			return;
		if (p) {
			*p = (uint16_t)((v & mask) | (*p & ~mask));
			return;
		}
	}
	if (a >= 0x0c0000 && a < 0x0c0040)
		k056832_word_w(&gx.k56832, (int)((a - 0x0c0000) >> 1), v, mask);
	else if (a >= 0x0c2000 && a < 0x0c2008) {
		if (mask & 0xff00) k055673_k053246_w(&gx.k55673, (int)(a - 0x0c2000), (uint8_t)(v >> 8));
		if (mask & 0x00ff) k055673_k053246_w(&gx.k55673, (int)(a - 0x0c2000) + 1, (uint8_t)v);
	} else if (a >= 0x0ca000 && a < 0x0ca020) {
		uint16_t *r = &gx.k54338[(a - 0x0ca000) >> 1];
		*r = (uint16_t)((v & mask) | (*r & ~mask));
	} else if (a >= 0x0cc000 && a < 0x0cc020) {   // (K053251: umask 00ff)
		if (mask & 0x00ff)
			m.k53251[((a - 0x0cc000) >> 1) & 0xf] = (uint8_t)(v & 0x3f);
	} else if (a >= 0x0ce000 && a < 0x0ce020)
		moo_prot_w((int)((a - 0x0ce000) >> 1), v, mask);
	else if (a == 0x0d4000)
		q_put(Q_IRQ, 0, 0);   // (sound_irq_w)
	else if (a >= 0x0d6000 && a < 0x0d6020) {
		if (mask & 0x00ff)
			q_put(Q_K54321, (int)((a - 0x0d6000) >> 1), (uint8_t)v);
	} else if (a >= 0x0d8000 && a < 0x0d8008)
		k056832_b_word_w(&gx.k56832, (int)((a - 0x0d8000) >> 1), v, mask);
	else if (a == 0x0de000) {   // (control2_w: the EEPROM lines, OBJCHA bit 8, the IRQ enables)
		m.control2 = (uint16_t)((v & mask) | (m.control2 & ~mask));
		const int64_t t = now_cycles();
		eepser_di_write(&s_eeprom, m.control2 & 1);
		eepser_cs_write(&s_eeprom, m.control2 >> 1 & 1, t);
		eepser_clk_write(&s_eeprom, m.control2 >> 2 & 1, t);
		gx.k55673.objcha_line = (m.control2 >> 8) & 1;
	} else if (a >= 0x190000 && a < 0x1a0000) {
		uint16_t *p = &m.spriteram[(a - 0x190000) >> 1];
		*p = (uint16_t)((v & mask) | (*p & ~mask));
	} else if (a >= 0x1a0000 && a < 0x1a4000)
		k056832_ram_word_w(&gx.k56832, (int)((a & 0x1fff) >> 1), v, mask);
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v)
{
	main_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00);
}

// ---- the sound board (sound_map) ----
static void z_bank(void)
{
	const uint32_t base = (uint32_t)(m.sound_bank & (s_xexex ? 7 : 0xf)) * 0x4000;
	for (int p = 0; p < 0x40; p++)
		s_zmap[0x80 + p] = base + (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + base + (uint32_t)p * 256 : NULL;
}
static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_zrom_size ? s_zrom[a] : 0xff;
	if (a < 0xc000) {
		const uint32_t o = (uint32_t)(m.sound_bank & (s_xexex ? 7 : 0xf)) * 0x4000 + (a - 0x8000);
		return o < s_zrom_size ? s_zrom[o] : 0xff;
	}
	if (a < 0xe000)
		return m.zram[a - 0xc000];
	if (a < 0xe230)
		return k054539_read(&s_k539, a - 0xe000);
	if (a == 0xec00 || a == 0xec01)
		return rbx_ym_read(0, a & 1);
	if (a == 0xf002)
		return m.latch[0];
	if (a == 0xf003)
		return m.latch[1];
	return 0xff;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xe000)
		m.zram[a - 0xc000] = v;
	else if (a >= 0xe000 && a < 0xe230)
		k054539_write(&s_k539, a - 0xe000, v);
	else if (a == 0xec00 || a == 0xec01)
		rbx_ym_write(0, a & 1, v);
	if (s_xexex && a == 0xe13f) {   // (the K054539's analog pan: ym_set_mixing, 71 x pan / 55)
		const int pan = v >= 0x11 && v <= 0x1f ? v - 0x11 : 0x18 - 0x11;
		m.ym_pan = (uint8_t)pan;
		s_ym_gl = (int32_t)(71.0 * s_k539.pantab[pan] / 55.0 * 256.0);
		s_ym_gr = (int32_t)(71.0 * s_k539.pantab[0xe - pan] / 55.0 * 256.0);
	}
	else if (a == 0xf000)
		m.latch[2] = v;
	else if (a == 0xf800) {
		m.sound_bank = v;
		z_bank();
	}
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// n samples (a line's): the Z80 for their cycles (no timer of the K054539 drives it here), the K054539, the YM2151
// for its clocks (its samples spread over the line's, the nearest earlier)
static void sound_run(int n)
{
	if (n <= 0)
		return;
	m.z_frac += (uint64_t)Z80_CLOCK * (uint64_t)n;
	const unsigned long cyc = (unsigned long)(m.z_frac / SND_RATE);
	m.z_frac -= (uint64_t)cyc * SND_RATE;
	int line = m.z80_irq;
	m.z_end += cyc;
	z80_run(&s_z, (unsigned long)m.z_end, &line, 1);
	m.z80_irq = (uint8_t)line;
	int32_t l[64], r[64], yl[96], yr[96];
	if (n > 64)
		n = 64;
	memset(l, 0, sizeof(int32_t) * (size_t)n);
	memset(r, 0, sizeof(int32_t) * (size_t)n);
	k054539_render(&s_k539, l, r, n);
	m.ym_frac += (uint64_t)YM_CLOCK * (uint64_t)n;
	const uint32_t yc = (uint32_t)(m.ym_frac / SND_RATE);
	m.ym_frac -= (uint64_t)yc * SND_RATE;
	memset(yl, 0, sizeof(yl));
	memset(yr, 0, sizeof(yr));
	const int yn = rbx_ym_advance_lr(0, yc, yl, yr, 96);
	for (int i = 0; i < n; i++) {
		const int k = yn > 0 ? i * yn / n : -1;
		const int32_t hl = k >= 0 ? yl[k] : 0, hr = k >= 0 ? yr[k] : 0;
		// (the K054539 at 0.5 inverted, the YM2151 at 0.30 (its gains), at the K054321's volume)
		if (s_out_pos < 2048) {
			if (s_xexex) {   // (the K054539 at 0.4 to both sides; the YM2151 at 0.2 through its filters)
				const int64_t k = (int64_t)(l[i] + r[i]) * 2 / 5;
				// (both YM2151 outputs to both sides' filters: filter1 / filter2 _l at the left gain, _r the right)
				s_out_l[s_out_pos] = (int32_t)((k + (int64_t)(hl + hr) * s_ym_gl / 256) * s_gain_l >> 12);
				s_out_r[s_out_pos] = (int32_t)((k + (int64_t)(hl + hr) * s_ym_gr / 256) * s_gain_r >> 12);
			} else {
				s_out_l[s_out_pos] = (int32_t)(((int64_t)(r[i] / 2) + hl) * s_gain_l >> 12);
				s_out_r[s_out_pos] = (int32_t)(((int64_t)(l[i] / 2) + hr) * s_gain_r >> 12);
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
		case Q_K54321: k54321_main_w(e->arg, (uint8_t)e->v); break;
		case Q_IRQ: m.z80_irq = 1; break;
		}
	}
	s_qn[b] = 0;
	s_out_n = s_out_pos;
	__atomic_store_n(&s_job_frame, 0, __ATOMIC_RELEASE);
}

// the frame's end: the last frame's samples out, its reply seen, this frame's queue to the worker
static void sound_frame(void)
{
	rbx_aux_sync();
	for (int i = 0; i < s_out_n; i++)
		rbx_audio_put_stereo(s_out_l[i], s_out_r[i]);
	s_out_n = 0;
	s_pub = m.latch[2];
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void moo_exit(void);
static void moo_reset(void);

static int moo_init(const rbx_game *g)
{
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t pcm_size, tile_size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("soundcpu", &s_zrom_size);
	if (!s_zrom)
		s_zrom = rbx_region("audiocpu", &s_zrom_size);   // (xexex)
	const uint8_t *pcm = rbx_region("k054539", &pcm_size);
	const uint8_t *tiles = rbx_region("k056832", &tile_size);
	s_sprrom = rbx_region("k053246", &s_sprrom_size);
	if (!s_rom || !s_zrom || !pcm || !tiles || !s_sprrom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&gx, 0, sizeof(gx));
	memset(&m, 0, sizeof(m));
	s_bucky = !strcmp(g->machine, "bucky");
	s_xexex = !strcmp(g->machine, "xexex");
	s_ym_gl = s_ym_gr = 256;   // (xexex: the YM2151's filters at 1 until the K054539's analog pan sets them)
	if (s_bucky && !s_extra && !(s_extra = (uint8_t *)calloc(1, 0x14000)))
		return -1;
	gx.pal_entries = s_bucky ? 4096 : 2048;
	gx.alpha_noinvert = 1;   // (K054338 without set_alpha_invert)
	gx.tilecb = GX_TILECB_PRE;
	gx.mixmode = GX_MIXMODE_PRE;
	gx.pre_rgb = 1;
	gx.pre_bands = 4;   // (the drawing in four bands: gx_render_help; xexex's K053250 too, clipped to each band)
	gx.pre_tilecb = tile_cb;
	gx.pre_sprcb = sprite_cb;
	gx.pre_capture = capture;
	gx.pre_draw = draw;
	gx.pre_screen_w = 512;
	k056832_init(&gx.k56832, tiles, tile_size, K056832_BPP_4, 1, 0);
	k055673_init(&gx.k55673, s_sprrom, s_sprrom_size, K055673_LAYOUT_053247, s_bucky || s_xexex ? -48 : -48 + 1,
	             s_xexex ? 32 : 23);
	gx_video_set_area(40, 384);   // (set_visarea(40, 40+384-1, 16, 16+224-1))
	if (s_xexex) {   // (set_raw(32 MHz / 4, 512, 40, 424, 289, 0, 256))
		gx_video_set_y0(0);
		gx_video_set_height(256);
	}
	m68k_init();
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0x000000, 0x07ffff, s_rom, s_rom_size < 0x80000 ? s_rom_size : 0x80000, 1, 0);
	if (s_xexex) {   // (xexex: the work RAM, the program's second half, the palette)
		m68k_bus_map(0x080000, 0x08ffff, gx.workram, 0x10000, 1, 1);
		if (s_rom_size > 0x100000)
			m68k_bus_map(0x100000, 0x17ffff, s_rom + 0x100000, s_rom_size - 0x100000, 1, 0);
		m68k_bus_map(0x1b0000, 0x1b1fff, gx.palram, 0x2000, 1, 1);
		uint32_t lsize;
		const uint8_t *lrom = rbx_region("k053250", &lsize);
		if (!lrom || k053250_init(&s_lvc, lrom, lsize, -5, -16)) {
			rbx_fail("%s: K053250", g->set);
			return -1;
		}
		gx.k53250 = &s_lvc;
		gx.pre_tilecb = tile_cb_xexex;
		gx.pre_sprcb = sprite_cb_xexex;
		gx.pre_draw = draw_xexex;
	} else if (s_bucky) {   // (bucky_map: the work RAM, the palette, the data ROM)
		m68k_bus_map(0x080000, 0x08ffff, gx.workram, 0x10000, 1, 1);
		m68k_bus_map(0x1b0000, 0x1b3fff, gx.palram, 0x4000, 1, 1);
		if (s_rom_size > 0x200000)
			m68k_bus_map(0x200000, 0x23ffff, s_rom + 0x200000, s_rom_size - 0x200000, 1, 0);
	} else {
		if (s_rom_size > 0x100000)
			m68k_bus_map(0x100000, 0x17ffff, s_rom + 0x100000, s_rom_size - 0x100000, 1, 0);
		m68k_bus_map(0x180000, 0x18ffff, gx.workram, 0x10000, 1, 1);
		m68k_bus_map(0x1c0000, 0x1c1fff, gx.palram, 0x2000, 1, 1);
	}
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
	k054539_init(&s_k539, pcm, pcm_size);
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = s_xexex ? 51 : 77;   // (0.30; xexex 0.2 into its filters)
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
	s_port_in0 = rbx_port_find(s_xexex ? "SYSTEM" : "IN0");   // (xexex: SYSTEM, EEPROM, P1, P2)
	s_port_in1 = rbx_port_find(s_xexex ? "EEPROM" : "IN1");
	s_port_p13 = rbx_port_find(s_xexex ? "P1" : "P1_P3");
	s_port_p24 = rbx_port_find(s_xexex ? "P2" : "P2_P4");
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(384, s_xexex ? 256 : VIS_H, 0x10000) || gx_video_init()) {
		rbx_fail("%s: out of memory", g->set);
		moo_exit();
		return -1;
	}
	for (int i = 0; i < 0x10000; i++) {   // (the bitmap: RGB 565)
		const int r = (i >> 11) & 31, gg = (i >> 5) & 63, b = i & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (gg << 2 | gg >> 4) << 8 | (b << 3 | b >> 2));
	}
	static const int offs[4] = { -2 + 1, 2 + 1, 4 + 1, 6 + 1 };   // (VIDEO_START moo; bucky: one less, no association)
	for (int l = 0; l < 4; l++)   // (xexex: -2 2 4 6, y 16)
		k056832_set_layer_offs(&gx.k56832, l, offs[l] - (s_bucky || s_xexex), s_xexex ? 16 : 0);
	if (s_bucky)
		k056832_set_layer_association(&gx.k56832, 0);
	moo_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void moo_exit(void)
{
	eepser_nv_save(&s_eeprom);
	rbx_aux_set_job(NULL);
	gx_video_exit();
	k053250_exit(&s_lvc);
	rbx_ym_exit(0);
	free(s_q_hp);
	s_q_hp = NULL;
}

static void moo_reset(void)
{
	rbx_aux_sync();   // (the job done before the board changes)
	memset(gx.workram, 0, sizeof(gx.workram));
	memset(m.zram, 0, sizeof(m.zram));
	memset(m.latch, 0, sizeof(m.latch));
	memset(m.k53251, 0, sizeof(m.k53251));
	memset(m.protram, 0, sizeof(m.protram));
	memset(m.k54000, 0, sizeof(m.k54000));
	m.cur_alpha = m.suspended = 0;
	m.ym_pan = 0;
	if (gx.k53250)
		k053250_reset(gx.k53250);
	if (s_extra)
		memset(s_extra, 0, 0x14000);
	m.control2 = 0;
	m.z80_irq = 0;
	m.sound_bank = 0;
	m.cycles_done = 0;
	m.dma_due = -1;
	m.cyc_frac = m.z_frac = m.z_end = m.snd_frac = m.ym_frac = 0;
	m.frame = 0;
	m.k54321_active = m.k54321_volume = 0;
	k54321_volume();
	k054539_reset(&s_k539);
	rbx_ym_reset(0);
	z80_init(&s_z);
	s_z.read_byte = z_rd;
	s_z.write_byte = z_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	for (int p = 0; p < 256; p++)
		s_zmap[p] = p < 0x80 && (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
	for (int p = 0; p < 0x20; p++)
		s_zmap[0xc0 + p] = m.zram + p * 256;
	z_bank();
	s_z.read_map = s_zmap;
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_pub = 0;
	s_job_frame = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
}

#define XE_LINES 289            // (xexex: set_raw(32 MHz / 4, 512, ..., 289, 0, 256))
#define XE_DMA_CYCLES (MAIN_CLOCK / 1000000 * 256)   // (XE_DMADELAY: 256 us)
static void xexex_frame(void)
{
	rbx_audio.frame_mhz = (uint32_t)(8000000ull * 1000 / (512ull * XE_LINES));
	for (int line = 0; line < XE_LINES; line++) {
		const int vpos = (line + 256) % XE_LINES;   // (the frame from the vblank start)
		if (vpos == 256) {   // (interrupt, scanline 256: the screen drawn, the object DMA, IRQ 4)
			m.suspended = 0;
			gx_video_capture();
			if (gx.k55673.kx46_regs[5] & 0x10) {
				object_dma();
				m.dma_due = m.cycles_done + XE_DMA_CYCLES;
			}
			if (m.control2 & 0x0800)
				m68k_bus_hold_irq(4);
		}
		if (vpos == 0) {   // (scanline 0: IRQ 6)
			m.suspended = 0;
			if (m.control2 & 0x0020)
				m68k_bus_hold_irq(6);
		}
		m.cyc_frac += (uint64_t)MAIN_CLOCK * 512;
		int64_t cyc = (int64_t)(m.cyc_frac / 8000000);
		m.cyc_frac -= (uint64_t)cyc * 8000000;
		while (cyc > 0) {
			int run = (int)cyc;
			if (m.dma_due >= 0 && m.dma_due - m.cycles_done < run)
				run = m.dma_due - m.cycles_done > 0 ? (int)(m.dma_due - m.cycles_done) : 0;
			if (run > 0) {
				// (waitskip_r: the CPU spinning until an interrupt resumes it, its cycles passing)
				const int used = m.suspended ? run : m68k_execute(run);
				m.cycles_done += used;
				cyc -= used;
			}
			if (m.dma_due >= 0 && m.cycles_done >= m.dma_due) {   // (dmaend_callback: IRQ 5)
				m.dma_due = -1;
				if (m.control2 & 0x0040) {
					m.suspended = 0;
					m68k_bus_hold_irq(5);
				}
			}
		}
		m.snd_frac += (uint64_t)SND_RATE * 512;
		const int ns = (int)(m.snd_frac / 8000000);
		m.snd_frac -= (uint64_t)ns * 8000000;
		if (ns > 0)
			q_put(Q_ADV, 0, (uint32_t)ns);
	}
	sound_frame();
	m.frame++;
	gx.frame = m.frame;
}

static void moo_frame(void)
{
	if (s_xexex) {
		xexex_frame();
		return;
	}
	for (int line = 0; line < LINES; line++) {
		if (line == 0) {   // (the vblank start: moo_interrupt)
			gx_video_capture();
			if (gx.k55673.kx46_regs[5] & 0x10) {   // (k053246_is_irq_enabled: the object DMA, IRQ 4 at its end)
				object_dma();
				m.dma_due = m.cycles_done + DMA_CYCLES;
			}
			if (m.control2 & 0x20)
				m68k_bus_hold_irq(5);
		}
		m.cyc_frac += MAIN_CLOCK;
		int64_t cyc = (int64_t)(m.cyc_frac / (60 * LINES));
		m.cyc_frac -= (uint64_t)cyc * (60 * LINES);
		while (cyc > 0) {
			int run = (int)cyc;
			if (m.dma_due >= 0 && m.dma_due - m.cycles_done < run)
				run = m.dma_due - m.cycles_done > 0 ? (int)(m.dma_due - m.cycles_done) : 0;
			if (run > 0) {
				const int used = m68k_execute(run);
				m.cycles_done += used;
				cyc -= used;
			}
			if (m.dma_due >= 0 && m.cycles_done >= m.dma_due) {   // (dmaend_callback)
				m.dma_due = -1;
				if (m.control2 & 0x800)
					m68k_bus_hold_irq(4);
			}
		}
		m.snd_frac += SND_RATE;
		const int ns = (int)(m.snd_frac / (60 * LINES));
		m.snd_frac -= (uint64_t)ns * (60 * LINES);
		if (ns > 0)
			q_put(Q_ADV, 0, (uint32_t)ns);
	}
	sound_frame();
	m.frame++;
	gx.frame = m.frame;
}

static int moo_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	int rc = 0;
	rbx_aux_sync();   // (the job done: the board at the frame's end, its samples waiting)
	if ((uint32_t)m68k_context_size() > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	rc |= rbx_state_io(ctx, "moo_cpu", cpu, (uint32_t)m68k_context_size());
	rc |= rbx_state_io(ctx, "moo_bus", &m68k_bus.irq_pending, sizeof(uint32_t) * 2);
	rc |= rbx_state_io(ctx, "gx", &gx, (uint32_t)offsetof(gx_t, state_end));
	rc |= k056832_state(&gx.k56832, ctx, load);
	rc |= k055673_state(&gx.k55673, ctx, load);
	rc |= rbx_state_io(ctx, "moo", &m, (uint32_t)offsetof(__typeof__(m), state_end));
	rc |= eepser_state(&s_eeprom, ctx, load, "moo_eeprom");
	rc |= k054539_state(&s_k539, ctx, load, "moo_k539_");
	rc |= rbx_ym_state(0, ctx, load);
	if (s_extra)
		rc |= rbx_state_io(ctx, "moo_extra", s_extra, 0x14000);
	if (gx.k53250)
		rc |= k053250_state(gx.k53250, ctx, load);
	int32_t qv[2] = { s_out_n, s_pub };   // (the last frame's samples not out yet, the reply)
	rc |= rbx_state_io(ctx, "moo_q", qv, sizeof(qv));
	rc |= rbx_state_io(ctx, "moo_outl", s_out_l, sizeof(s_out_l));
	rc |= rbx_state_io(ctx, "moo_outr", s_out_r, sizeof(s_out_r));
	if (load && !rc) {
		s_out_n = qv[0] >= 0 && qv[0] <= 2048 ? qv[0] : 0;
		s_pub = (uint8_t)qv[1];
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
	}
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "moo_z80", &z, sizeof(z));
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
		z_bank();
		k54321_volume();
		if (s_xexex) {
			s_ym_gl = (int32_t)(71.0 * s_k539.pantab[m.ym_pan] / 55.0 * 256.0);
			s_ym_gr = (int32_t)(71.0 * s_k539.pantab[0xe - m.ym_pan] / 55.0 * 256.0);
		}
	}
	return rc ? -1 : 0;
}

static int moo_save(void *ctx) { return moo_state(ctx, 0); }
static int moo_load(void *ctx) { return moo_state(ctx, 1); }
static int moo_supported(const char *machine)
{
	return !strcmp(machine, "moo") || !strcmp(machine, "bucky") || !strcmp(machine, "xexex");
}

const rbx_driver rbx_drv_moo = { "Konami Moo Mesa", moo_init, moo_exit, moo_reset, moo_frame, moo_save, moo_load,
                                 moo_supported, gx_render, gx_render_help };
