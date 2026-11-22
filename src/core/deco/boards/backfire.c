/*  Data East Backfire! (156 ARM, two screens side by side), ported from MAME src/mame/dataeast/backfire.cpp
    (BSD-3-Clause, copyright-holders: David Haywood).
    156 = encrypted ARM, 7 MHz with MAME's clock scale 4 (28 MHz of instructions); two tilemap chips 141 and two sprite
    chips 52, one per screen (the screens are drawn side by side, 640 x 240); YMZ280B (left / right speaker = the
    two screens'), 93C46 EEPROM, ADC0808 for the two paddles.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/arm/arm.h"
#include "../../sound/ymz280b.h"
#include "../../machine/eeprom93c46.h"
#include "../../machine/adc0808.h"
#include "../../video/gfxlayout.h"
#include "../machine/deco156.h"
#include "../machine/decocrpt.h"
#include "../machine/armpoll.h"
#include "../video/deco16ic.h"
#include "../video/decospr.h"
#include "../../video/decoband.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 28000000   // 28 MHz / 4 x set_clock_scale(4.0)
#define LINES 256            // set_size(40*8, 32*8), 60 Hz
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 320
#define VBL_LINES 38         // vblank 2500 us at 60 Hz x 256 lines (65.1 us a line)
#define ADC_CLOCK 1000000

static arm_t s_cpu;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t (*s_ram_hp)[0x8000];             // 0x170000 main RAM (32-bit)
#define s_ram (*s_ram_hp)
static uint16_t s_pal[0x800];             // 0x160000 (xBGR_555, 16-bit)
static uint16_t s_spriteram[2][0x800];
static uint16_t s_rowscroll[4][0x400];
static uint32_t s_pri_reg[2];             // left / right priority (0x1a8000 / 0x1ac000)
static deco16ic_t (*s_tile_hp)[2];
#define s_tile (*s_tile_hp)
static decospr_t s_spr[2];
static rbx_gfx s_gfx8[2], s_gfx16[2], s_gfxspr[2];
static ymz280b_t s_ymz;
static eeprom93c46_t s_eeprom;
static adc0808_t s_adc;
static char s_nv_path[256];
static int s_port_in0, s_port_in1;
static int s_line, s_irq;
static uint32_t s_frame;
static int s_paddle[2];                   // 0x20 .. 0xe0 (PORT_MINMAX), start 0x80
static uint64_t s_cpu_frac, s_snd_frac, s_adc_frac;
static int64_t s_spin_until;
static uint32_t s_spd_pc;
static armpoll_t s_poll;
static int32_t *s_ymz_l, *s_ymz_r;
static int s_ymz_cap, s_ymz_n;
static uint32_t s_out_rate, s_ymz_line0, s_ymz_line_n;
static int64_t s_line_c0, s_line_cn;
static uint8_t (*s_pri_hp)[2 * VIS_W * VIS_H];
#define s_pri (*s_pri_hp)
static struct {
	deco16ic_t tile[2];
	uint16_t rowscroll[4][0x400], spriteram[2][0x800], pal[0x800];
	uint32_t pri_reg[2], frame;
} *s_snap;

static uint32_t rd32le(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

// ---- sound: the YMZ rendered up to the CPU's time before each access ----
static void ymz_to(int n)
{
	if (n > s_ymz_cap)
		n = s_ymz_cap;
	if (n > s_ymz_n) {
		ymz280b_render(&s_ymz, s_ymz_l + s_ymz_n, s_ymz_r + s_ymz_n, n - s_ymz_n);
		s_ymz_n = n;
	}
}
static void ymz_sync(void)
{
	int64_t d = s_cpu.cycles - s_line_c0;
	if (d < 0) d = 0;
	if (d > s_line_cn) d = s_line_cn;
	ymz_to((int)(s_ymz_line0 + (s_line_cn ? (uint32_t)(d * s_ymz_line_n / s_line_cn) : 0)));
}

// ---- bus ----
static uint32_t in0_r(void)
{
	const int vbl = s_line >= VIS_Y0 + VIS_H || s_line < VIS_Y0 + VIS_H + VBL_LINES - LINES;
	rbx_port_custom(s_port_in0, 0x05500000u, (vbl ? 0x00500000u : 0) | (eeprom_do(&s_eeprom) ? 0x01000000u : 0) |
	                                         (s_adc.eoc ? 0x04000000u : 0));
	return rbx_port_read(s_port_in0);
}

static inline void combine16(uint16_t *p, uint32_t data, uint32_t mask)
{
	mask &= 0xffff;
	*p = (uint16_t)((*p & ~mask) | (data & mask));
}

// a tilemap chip's block: control 0x00, vram 0x10000 / 0x14000; rowscroll blocks are separate
static uint32_t tile_r(deco16ic_t *t, uint32_t o)
{
	if (o < 0x20) return deco16ic_control_r(t, (int)(o >> 2)) | 0xffff0000u;
	if (o >= 0x10000 && o < 0x12000) return t->vram[0][(o - 0x10000) >> 2] | 0xffff0000u;
	if (o >= 0x14000 && o < 0x16000) return t->vram[1][(o - 0x14000) >> 2] | 0xffff0000u;
	return 0;
}
static void tile_w(deco16ic_t *t, uint32_t o, uint32_t data, uint32_t mask)
{
	if (o < 0x20) {
		if (mask & 0xffff)
			deco16ic_control_w(t, (int)(o >> 2), (uint16_t)data, (uint16_t)mask);
	} else if (o >= 0x10000 && o < 0x12000) combine16(&t->vram[0][(o - 0x10000) >> 2], data, mask);
	else if (o >= 0x14000 && o < 0x16000) combine16(&t->vram[1][(o - 0x14000) >> 2], data, mask);
}

static uint32_t bus_r32(void *ctx, uint32_t a)
{
	(void)ctx;
	a &= 0x03fffffc;
	if (a < 0x100000)
		return rd32le(s_rom + (a & (s_rom_size - 1)));
	if (a >= 0x170000 && a < 0x178000) {
		const uint32_t v = rd32le(s_ram + (a & 0x7ffc));
		if (a == 0x170018 && s_spd_pc && arm_pc(&s_cpu) == s_spd_pc && s_spin_until <= s_cpu.cycles) {
			s_spin_until = s_cpu.cycles + CPU_CLOCK / 2500;   // backfire_speedup_r: spin_until_time(400 us)
			arm_break = 1;
		}
		return v;
	}
	if (a >= 0x100000 && a < 0x120000) return tile_r(&s_tile[0], a - 0x100000);
	if (a >= 0x120000 && a < 0x121000) return s_rowscroll[0][(a - 0x120000) >> 2] | 0xffff0000u;
	if (a >= 0x124000 && a < 0x125000) return s_rowscroll[1][(a - 0x124000) >> 2] | 0xffff0000u;
	if (a >= 0x130000 && a < 0x150000) return tile_r(&s_tile[1], a - 0x130000);
	if (a >= 0x150000 && a < 0x151000) return s_rowscroll[2][(a - 0x150000) >> 2] | 0xffff0000u;
	if (a >= 0x154000 && a < 0x155000) return s_rowscroll[3][(a - 0x154000) >> 2] | 0xffff0000u;
	if (a >= 0x160000 && a < 0x162000) return s_pal[(a - 0x160000) >> 2];   // (palette read16, umask32 0x0000ffff)
	if (a >= 0x184000 && a < 0x186000) return s_spriteram[0][(a - 0x184000) >> 2] | 0xffff0000u;
	if (a >= 0x18c000 && a < 0x18e000) return s_spriteram[1][(a - 0x18c000) >> 2] | 0xffff0000u;
	if (a == 0x190000) {
		const uint32_t v = in0_r();
		armpoll_read(&s_poll, &s_cpu, v, s_rom, s_rom_size);
		return v;
	}
	if (a == 0x194000) return rbx_port_read(s_port_in1);
	if (a == 0x1a8000) return s_pri_reg[0];
	if (a == 0x1ac000) return s_pri_reg[1];
	if (a >= 0x1c0000 && a < 0x1c0008) {
		ymz_sync();
		return ymz280b_read(&s_ymz, (int)((a - 0x1c0000) >> 2));
	}
	if (a == 0x1e4000) return adc0808_data_r(&s_adc);
	if (a >= 0x1e8000 && a < 0x1e8008) {   // pot_select_r: a conversion of channel offset
		adc0808_start(&s_adc, (int)((a - 0x1e8000) >> 2));
		return 0;
	}
	return 0;
}

static void bus_w32m(uint32_t a, uint32_t data, uint32_t mask)
{
	a &= 0x03fffffc;
	if (a >= 0x170000 && a < 0x178000) {
		uint8_t *p = s_ram + (a & 0x7ffc);
		for (int i = 0; i < 4; i++)
			if (mask & (0xffu << (8 * i)))
				p[i] = (uint8_t)(data >> (8 * i));
	} else if (a >= 0x100000 && a < 0x120000) tile_w(&s_tile[0], a - 0x100000, data, mask);
	else if (a >= 0x120000 && a < 0x121000) combine16(&s_rowscroll[0][(a - 0x120000) >> 2], data, mask);
	else if (a >= 0x124000 && a < 0x125000) combine16(&s_rowscroll[1][(a - 0x124000) >> 2], data, mask);
	else if (a >= 0x130000 && a < 0x150000) tile_w(&s_tile[1], a - 0x130000, data, mask);
	else if (a >= 0x150000 && a < 0x151000) combine16(&s_rowscroll[2][(a - 0x150000) >> 2], data, mask);
	else if (a >= 0x154000 && a < 0x155000) combine16(&s_rowscroll[3][(a - 0x154000) >> 2], data, mask);
	else if (a >= 0x160000 && a < 0x162000) combine16(&s_pal[(a - 0x160000) >> 2], data, mask);
	else if (a >= 0x184000 && a < 0x186000) combine16(&s_spriteram[0][(a - 0x184000) >> 2], data, mask);
	else if (a >= 0x18c000 && a < 0x18e000) combine16(&s_spriteram[1][(a - 0x18c000) >> 2], data, mask);
	else if (a == 0x1a4000) {   // eeprom_w (the byte at 0x1a4000)
		if (mask & 0xff)
			eeprom_write_lines(&s_eeprom, (data >> 2) & 1, (data >> 1) & 1, data & 1);
	} else if (a == 0x1a8000) s_pri_reg[0] = (s_pri_reg[0] & ~mask) | (data & mask);
	else if (a == 0x1ac000) s_pri_reg[1] = (s_pri_reg[1] & ~mask) | (data & mask);
	else if (a == 0x1b0000) {   // irq_ack_w
		s_irq = 0;
		arm_set_irq(&s_cpu, 0);
	} else if (a >= 0x1c0000 && a < 0x1c0008) {
		if (mask & 0xff) {
			ymz_sync();
			ymz280b_write(&s_ymz, (int)((a - 0x1c0000) >> 2), (uint8_t)data);
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

// ---- video ----
static uint32_t pri_cb(uint16_t x, int extpri)
{
	(void)extpri;
	return (x & 0x4000) ? 0xf0 : 0;   // 0x4000 / 0xc000: behind the front playfield; 0x0000 / 0x8000: over
}

// bitswap<3>(bank, 5, 6, 4) << 12
static int bank_cb(int bank) { return (((bank >> 5) & 1) << 2 | ((bank >> 6) & 1) << 1 | ((bank >> 4) & 1)) << 12; }

// one screen (0 left: the chips' playfield 1, 1 right: playfield 2) into its half of the bitmap
static void draw_screen(int scr, uint16_t *bm, uint8_t *pri, int pitch)
{
	const int pf = scr;   // tilemap_1_draw: tmap[0], tilemap_2_draw: tmap[1]
	const uint16_t fill = scr ? 0x500 : 0x100;
	for (int y = 0; y < VIS_H; y++) {
		for (int x = 0; x < VIS_W; x++)
			bm[y * pitch + x] = fill;
		memset(pri + y * pitch, 0, VIS_W);
	}
	const uint32_t p = s_snap->pri_reg[scr];
	const uint16_t *rs0 = s_snap->rowscroll[pf], *rs1 = s_snap->rowscroll[2 + pf];
	if (p == 0) {
		deco16ic_draw(&s_snap->tile[1], pf, rs1, bm, pri, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 1, 0xff);
		deco16ic_draw(&s_snap->tile[0], pf, rs0, bm, pri, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 2, 0xff);
	} else if (p == 2) {
		deco16ic_draw(&s_snap->tile[0], pf, rs0, bm, pri, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 2, 0xff);
		deco16ic_draw(&s_snap->tile[1], pf, rs1, bm, pri, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 4, 0xff);
	} else
		return;   // (MAME: popmessage, nothing drawn but the fill)
	decospr_draw(&s_spr[scr], s_snap->spriteram[scr], 0x2000 / 4, bm, pri, pitch, VIS_W, VIS_H, VIS_Y0, 1, s_snap->frame);
}

static void draw_job(int scr)
{
	draw_screen(scr, rbx_video.bitmap + scr * VIS_W, s_pri + scr * VIS_W, rbx_video.pitch);
}
static decoband_t s_band = { 0, 0, 0, 0, 2, draw_job };   // (a job per screen: the helper draws the other)
static int bf_help(int helper) { return decoband_help(&s_band, helper); }

static void bf_render(void)
{
	const uint16_t *p = s_snap->pal;
	for (int i = 0; i < 0x800; i++) {   // xBGR_555
		int r = p[i] & 31, g = (p[i] >> 5) & 31, b = (p[i] >> 10) & 31;
		r = (r << 3) | (r >> 2); g = (g << 3) | (g >> 2); b = (b << 3) | (b >> 2);
		rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	for (int i = 0; i < 2; i++)
		deco16ic_update(&s_snap->tile[i]);
	decoband_run(&s_band);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->tile, s_tile, sizeof(s_tile));
	memcpy(s_snap->rowscroll, s_rowscroll, sizeof(s_rowscroll));
	memcpy(s_snap->spriteram, s_spriteram, sizeof(s_spriteram));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	memcpy(s_snap->pri_reg, s_pri_reg, sizeof(s_pri_reg));
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void bf_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_rowscroll, 0, sizeof(s_rowscroll));
	memset(s_pri_reg, 0, sizeof(s_pri_reg));
	for (int i = 0; i < 2; i++) {
		memset(s_tile[i].vram, 0, sizeof(s_tile[i].vram));
		deco16ic_reset(&s_tile[i]);
	}
	ymz280b_reset(&s_ymz);
	adc0808_reset(&s_adc);
	arm_reset(&s_cpu);
	armpoll_reset(&s_poll);
	s_paddle[0] = s_paddle[1] = 0x80;
	s_line = s_irq = 0;
	s_frame = 0;
	s_cpu_frac = s_snd_frac = s_adc_frac = 0;
	s_spin_until = 0;
	s_ymz_n = 0;
}

static void bf_exit(void);

static int decode_tiles(int i, uint8_t *rom, uint32_t size)
{
	const uint32_t h = size * 4;
	static uint32_t x8[8], y8[8], x16[16], y16[16];
	for (int k = 0; k < 8; k++) {
		x8[k] = (uint32_t)k;
		y8[k] = (uint32_t)(k * 16);
		x16[k] = (uint32_t)(256 + k);
		x16[8 + k] = (uint32_t)k;
	}
	for (int k = 0; k < 16; k++)
		y16[k] = (uint32_t)(k * 16);
	const uint32_t pl[4] = { h + 8, h, 8, 0 };
	const rbx_gfx_layout l8 = { 8, 8, 4, pl, x8, y8, 8 * 8 * 2 };
	const rbx_gfx_layout l16 = { 16, 16, 4, pl, x16, y16, 16 * 16 * 2 };
	if (i < 2)
		return rbx_gfx_decode(&s_gfx8[i], &l8, rom, size / 2) || rbx_gfx_decode(&s_gfx16[i], &l16, rom, size / 2);
	return rbx_gfx_decode(&s_gfxspr[i - 2], &l16, rom, size / 2);
}

static int bf_init(const rbx_game *g)
{
	if (!s_pri_hp && !(s_pri_hp = calloc(1, sizeof(*s_pri_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ram_hp && !(s_ram_hp = calloc(1, sizeof(*s_ram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_tile_hp && !(s_tile_hp = calloc(1, sizeof(*s_tile_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t ts[2], ss[2], ys;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *tiles[2] = { rbx_region("tiles1", &ts[0]), rbx_region("tiles2", &ts[1]) };
	uint8_t *spr[2] = { rbx_region("sprites1", &ss[0]), rbx_region("sprites2", &ss[1]) };
	uint8_t *ymz = rbx_region("ymz", &ys);
	if (!s_rom || !tiles[0] || !tiles[1] || !spr[0] || !spr[1] || !ymz) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (deco56_decrypt_gfx(tiles[0], ts[0]) || deco56_decrypt_gfx(tiles[1], ts[1]) || deco156_decrypt(s_rom, s_rom_size)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	{	// descramble_sound: only the first 2 MB (the first ROM)
		const uint32_t len = ys < 0x200000 ? ys : 0x200000;
		uint8_t *buf = (uint8_t *)malloc(len);
		if (!buf) {
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
		for (uint32_t x = 0; x < len; x++) {
			const uint32_t addr = (x & 0xe00000) | ((x & 1) << 20) | ((x >> 1) & 0xfffff);
			if (addr < len)
				buf[addr] = ymz[x];
		}
		memcpy(ymz, buf, len);
		free(buf);
	}
	for (int i = 0; i < 4; i++)
		if (decode_tiles(i, i < 2 ? tiles[i] : spr[i - 2], i < 2 ? ts[i] : ss[i - 2])) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			bf_exit();
			return -1;
		}
	static const int colbank[2][2] = { { 0x00, 0x40 }, { 0x10, 0x50 } };
	for (int c = 0; c < 2; c++) {
		memset(&s_tile[c], 0, sizeof(s_tile[c]));
		for (int i = 0; i < 2; i++) {
			s_tile[c].tm[i].size = DECO_64x32;
			s_tile[c].tm[i].col_bank = colbank[c][i];
			s_tile[c].tm[i].col_mask = 0x0f;
			s_tile[c].tm[i].bank_cb = bank_cb;
		}
		s_tile[c].gfx8 = &s_gfx8[c];
		s_tile[c].gfx16 = &s_gfx16[c];
		memset(&s_spr[c], 0, sizeof(s_spr[c]));
		s_spr[c].gfx = &s_gfxspr[c];
		s_spr[c].color_base = c ? 0x600 : 0x200;
		s_spr[c].pri_cb = pri_cb;
		s_spr[c].flip_screen = 1;   // (sprites are flipped relative to the tilemaps)
	}
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_snap) {
		rbx_fail("%s: out of memory", g->set);
		bf_exit();
		return -1;
	}
	ymz280b_init(&s_ymz, 14000000, ymz, ys, NULL, NULL);
	s_out_rate = ymz280b_rate(&s_ymz);
	s_ymz_cap = (int)(s_out_rate / 50) + 64;
	s_ymz_l = (int32_t *)calloc((size_t)s_ymz_cap, sizeof(int32_t));
	s_ymz_r = (int32_t *)calloc((size_t)s_ymz_cap, sizeof(int32_t));
	if (!s_ymz_l || !s_ymz_r || rbx_audio_setup_stereo(s_out_rate) || rbx_video_setup(2 * VIS_W, VIS_H, 0x800)) {
		bf_exit();
		return -1;
	}
	s_nv_path[0] = 0;
	eeprom_init(&s_eeprom);
	{
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			eeprom_load(&s_eeprom, s_nv_path);
		}
	}
	s_spd_pc = !strcmp(g->set, "backfire") ? 0xce44 : !strcmp(g->set, "backfirea") ? 0xcee4 : 0;
	s_port_in0 = rbx_port_find("IN0");
	s_port_in1 = rbx_port_find("IN1");
	arm_init(&s_cpu);
	for (uint32_t a = 0; a < 0x100000 && a < s_rom_size; a += 0x1000)
		s_cpu.rpage[a >> 12] = s_rom + a;
	for (uint32_t a = 0; a < 0x8000; a += 0x1000)
		s_cpu.rpage[(0x170000 + a) >> 12] = s_cpu.wpage[(0x170000 + a) >> 12] = s_ram + a;
	s_cpu.rpage[0x170000 >> 12] = NULL;   // (the page with the speedup word 0x170018: the handler)
	s_cpu.read32 = bus_r32;
	s_cpu.write32 = bus_w32;
	s_cpu.read8 = bus_r8;
	s_cpu.write8 = bus_w8;
	s_cpu.ctx = NULL;
	bf_reset();
	return 0;
}

static void bf_exit(void)
{
	if (s_nv_path[0] && s_eeprom.dirty)
		eeprom_save(&s_eeprom, s_nv_path);
	s_nv_path[0] = 0;
	for (int i = 0; i < 2; i++) {
		rbx_gfx_free(&s_gfx8[i]);
		rbx_gfx_free(&s_gfx16[i]);
		rbx_gfx_free(&s_gfxspr[i]);
	}
	free(s_snap);
	s_snap = NULL;
	free(s_ymz_l);
	free(s_ymz_r);
	s_ymz_l = s_ymz_r = NULL;
	free(s_pri_hp);
	s_pri_hp = NULL;
	free(s_ram_hp);
	s_ram_hp = NULL;
	free(s_tile_hp);
	s_tile_hp = NULL;
}

static void run_cpu(int64_t end)
{
	s_poll.slice_end = end;
	while (s_cpu.cycles < end) {
		if (s_spin_until > s_cpu.cycles) {
			s_cpu.cycles = s_spin_until < end ? s_spin_until : end;
			continue;
		}
		arm_run(&s_cpu, end);
		arm_break = 0;
	}
}

static void bf_frame(void)
{
	// paddles: absolute 0x20 .. 0xe0, key delta 4 (d-pad left / right without one)
	for (int p = 0; p < 2; p++) {
		s_paddle[p] += rbx_analog_dpad(p + 1, RBX_IPT_PADDLE, 4);
		s_paddle[p] = s_paddle[p] < 0x20 ? 0x20 : s_paddle[p] > 0xe0 ? 0xe0 : s_paddle[p];
	}
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // vbl_interrupt: assert (irq_ack_w clears)
			vblank_capture();
			s_irq = 1;
			arm_set_irq(&s_cpu, 1);
		}
		const int lc = rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, 60000, LINES);
		const uint64_t den = 60000ull * LINES;
		s_snd_frac += (uint64_t)s_out_rate * 1000;
		s_ymz_line0 = (uint32_t)s_ymz_n;
		s_ymz_line_n = (uint32_t)rbx_div64_small(s_snd_frac, den);
		s_snd_frac -= (uint64_t)s_ymz_line_n * den;
		s_line_c0 = s_cpu.cycles;
		s_line_cn = lc;
		run_cpu(s_cpu.cycles + lc);
		ymz_to((int)(s_ymz_line0 + s_ymz_line_n));
		if (adc0808_advance(&s_adc, rbx_line_cycles(&s_adc_frac, ADC_CLOCK, 60000, LINES)))   // PORT_REVERSE
			s_adc.sar = (uint8_t)(s_adc.addr < 2 ? 0x20 + 0xe0 - s_paddle[s_adc.addr] : 0xff);
	}
	for (int i = 0; i < s_ymz_n; i++)
		rbx_audio_put_stereo(s_ymz_l[i], s_ymz_r[i]);
	memset(s_ymz_l, 0, sizeof(int32_t) * (size_t)s_ymz_n);
	memset(s_ymz_r, 0, sizeof(int32_t) * (size_t)s_ymz_n);
	s_ymz_n = 0;
	s_frame++;
}

static int bf_state(void *ctx, int load)
{
	uint8_t cpu[ARM_STATE_SIZE];
	if (!load)
		memcpy(cpu, &s_cpu, ARM_STATE_SIZE);
	uint64_t fr[3] = { s_cpu_frac, s_snd_frac, s_adc_frac };
	int32_t misc[6] = { (int32_t)s_frame, s_irq, s_paddle[0], s_paddle[1], 0, 0 };
	int64_t spin = s_spin_until;
	int rc = 0;
	rc |= rbx_state_io(ctx, "arm", cpu, ARM_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "rows", s_rowscroll, sizeof(s_rowscroll));
	rc |= rbx_state_io(ctx, "prio", s_pri_reg, sizeof(s_pri_reg));
	for (int i = 0; i < 2; i++) {
		rc |= rbx_state_io(ctx, i ? "vram2" : "vram1", s_tile[i].vram, sizeof(s_tile[i].vram));
		rc |= rbx_state_io(ctx, i ? "tctl2" : "tctl1", s_tile[i].control, sizeof(s_tile[i].control));
	}
	rc |= rbx_state_io(ctx, "ymz", &s_ymz, YMZ280B_STATE_SIZE);
	rc |= rbx_state_io(ctx, "eeprom", &s_eeprom, sizeof(s_eeprom));
	rc |= rbx_state_io(ctx, "adc", &s_adc, sizeof(s_adc));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	rc |= rbx_state_io(ctx, "spin", &spin, sizeof(spin));
	if (rc)
		return -1;
	if (load) {
		memcpy(&s_cpu, cpu, ARM_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_adc_frac = fr[2];
		s_frame = (uint32_t)misc[0];
		s_irq = misc[1];
		s_paddle[0] = misc[2];
		s_paddle[1] = misc[3];
		s_spin_until = spin;
		for (int c = 0; c < 2; c++)
			for (int i = 0; i < 8; i++)
				deco16ic_control_w(&s_tile[c], i, s_tile[c].control[i], 0xffff);
		armpoll_reset(&s_poll);
	}
	return 0;
}

static int bf_save(void *ctx) { return bf_state(ctx, 0); }
static int bf_load(void *ctx) { return bf_state(ctx, 1); }
static int bf_supported(const char *machine) { return !strcmp(machine, "backfire"); }

const rbx_driver rbx_drv_backfire = { "DECO 156 Backfire", bf_init, bf_exit, bf_reset, bf_frame, bf_save, bf_load,
                                      bf_supported, bf_render, bf_help };
