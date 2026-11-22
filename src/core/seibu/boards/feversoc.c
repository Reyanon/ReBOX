/*  Seibu Kaihatsu Fever Soccer: an SH-2 (SH7604) with the RISE11 sprite generator (6 bpp sprites, encrypted ROMs),
    an OKI MSM6295 (banked), a 93C56 EEPROM, battery-backed RAM; 4096 colors (xBGR_555 RAM).
    Ported from MAME 0.289 src/mame/seibu/feversoc.cpp (BSD-3-Clause, copyright-holders: Angelo Salese), the sprite
    drawing of src/mame/seibu/sei25x_rise1x_spr.cpp (BSD-3-Clause, copyright-holders: Olivier Galibert, Angelo Salese,
    David Haywood, Tomasz Slanina, Nicola Salmoria, Ville Linde, hap), the RISE11 sprite decryption of
    src/mame/seibu/seibuspi_m.cpp (BSD-3-Clause, copyright-holders: Ville Linde, Nicola Salmoria) and
    src/mame/seibu/seibu_helper.cpp (BSD-3-Clause, copyright-holders: Ville Linde, Nicola Salmoria).
    This file is part of ReBOX. GPL-2.0-or-later.

    Not done: the RTC (JRC6355E: its data line reads 0), the hopper (never empty), the lamps and counters, the NVRAM
    and EEPROM kept across runs.
*/
#include "../../rbx.h"
#include "../../cpu/sh2/sh2.h"
#include "../../cpu/sh2/sh2idle.h"
#include "../../machine/eepromser.h"
#include "../../sound/okim6295.h"
#include "../../video/gfxlayout.h"
#include "../machine/smix.h"
#include "../video/seibu_draw.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 60000
#define LINES 256
#define VIS_W 320
#define VIS_H 240
#define CPU_CLOCK 28636363
#define OKI_CLOCK (28636363 / 16)
#define NPENS 0x1000

static uint8_t *s_rom;
static uint32_t s_rom_size;
static rbx_gfx s_gspr;
static int s_port_in0, s_port_in1;
static sh2_t *s_cpu;   // (calloc at the init: its page tables)
static sh2_idle_t s_idle;   // (cpu/sh2/sh2idle.c: the vblank wait; RBX_SEIBU_NOIDLE=1 off, to compare)
static okim6295_t s_oki;
static eepser_t s_eeprom;
static smix_t *s_mix;

typedef struct {
	uint8_t ram[0x40000];        // 0x02000000-0x0203ffff (work RAM, NVRAM 0x30000, sprites 0x3e000), host-order longs
	uint8_t pal[0x2000];         // 0x06018000
	uint8_t irq, oki_bank;
	uint64_t main_frac, oki_frac;
	uint8_t state_end;
} board_t;
static board_t *s_bp;
#define b (*s_bp)

typedef struct {
	uint16_t spr[0x1000];
	uint32_t pal[NPENS];
} snap_t;
static snap_t *s_snapp;
#define s_snap (*s_snapp)

static inline uint16_t host_word(const uint8_t *base, uint32_t off) { return *(const uint16_t *)(base + (off ^ 2)); }

// ---- the RISE11 sprite decryption ----
static uint32_t partial_carry_sum(uint32_t add1, uint32_t add2, uint32_t carry_mask, int bits)
{
	uint32_t res = 0;
	int carry = 0;
	for (int i = 0; i < bits; i++) {
		const int bit = (int)((add1 >> i) & 1) + (int)((add2 >> i) & 1) + carry;
		res += (uint32_t)(bit & 1) << i;
		carry = ((carry_mask >> i) & 1) ? bit >> 1 : 0;
	}
	if (carry)
		res ^= 1;
	return res;
}

static void sprite_reorder(uint8_t *buffer)
{
	uint8_t temp[64];
	for (int j = 0; j < 16; j++) {
		temp[2 * (j * 2) + 0] = buffer[2 * j + 0];
		temp[2 * (j * 2) + 1] = buffer[2 * j + 1];
		temp[2 * (j * 2) + 2] = buffer[2 * j + 32];
		temp[2 * (j * 2) + 3] = buffer[2 * j + 33];
	}
	memcpy(buffer, temp, 64);
}

#define BIT(v, n) (((v) >> (n)) & 1u)
static void rise11_decrypt(uint8_t *rom, uint32_t size, uint32_t k1, uint32_t k2, uint32_t k3, uint32_t k4, uint32_t k5,
                           int feversoc_kludge)
{
	for (uint32_t i = 0; i < size / 2; i++) {
		const uint32_t b1 = (uint32_t)(rom[2 * i] + (rom[2 * i + 1] << 8));
		const uint32_t b2 = (uint32_t)(rom[size + 2 * i] + (rom[size + 2 * i + 1] << 8));
		const uint32_t b3 = (uint32_t)(rom[2 * size + 2 * i] + (rom[2 * size + 2 * i + 1] << 8));
		uint32_t p543 = BIT(b2, 11) << 0 | BIT(b1, 6) << 1 | BIT(b3, 12) << 2 | BIT(b3, 3) << 3 | BIT(b2, 12) << 4 |
		                BIT(b3, 14) << 5 | BIT(b3, 4) << 6 | BIT(b1, 11) << 7 | BIT(b1, 12) << 8 | BIT(b1, 2) << 9 |
		                BIT(b2, 5) << 10 | BIT(b1, 9) << 11 | BIT(b3, 1) << 12 | BIT(b2, 2) << 13 | BIT(b2, 10) << 14 |
		                BIT(b3, 5) << 15 | BIT(b1, 3) << 16 | BIT(b2, 7) << 17 | BIT(b1, 15) << 18 | BIT(b3, 9) << 19 |
		                BIT(b2, 13) << 20 | BIT(b1, 4) << 21 | BIT(b3, 2) << 22 | BIT(b2, 0) << 23;
		uint32_t p210 = BIT(b1, 14) << 0 | BIT(b1, 1) << 1 | BIT(b1, 13) << 2 | BIT(b3, 0) << 3 | BIT(b1, 7) << 4 |
		                BIT(b2, 14) << 5 | BIT(b2, 4) << 6 | BIT(b2, 9) << 7 | BIT(b3, 8) << 8 | BIT(b2, 1) << 9 |
		                BIT(b3, 7) << 10 | BIT(b2, 6) << 11 | BIT(b1, 0) << 12 | BIT(b3, 11) << 13 | BIT(b2, 8) << 14 |
		                BIT(b3, 13) << 15 | BIT(b1, 8) << 16 | BIT(b3, 10) << 17 | BIT(b3, 6) << 18 | BIT(b1, 10) << 19 |
		                BIT(b2, 15) << 20 | BIT(b2, 3) << 21 | BIT(b1, 5) << 22 | BIT(b3, 15) << 23;
		p543 = partial_carry_sum(p543, k1, k2, 32) ^ k3;
		p210 = partial_carry_sum(p210, i, k4, 24) ^ k5;
		if (feversoc_kludge)
			p210 = partial_carry_sum(p210, 1, 0x000001, 24);
		rom[2 * i] = (uint8_t)(p543 >> 16);
		rom[2 * i + 1] = (uint8_t)(p543 >> 8);
		rom[size + 2 * i] = (uint8_t)p543;
		rom[size + 2 * i + 1] = (uint8_t)(p210 >> 16);
		rom[2 * size + 2 * i] = (uint8_t)(p210 >> 8);
		rom[2 * size + 2 * i + 1] = (uint8_t)p210;
	}
	for (uint32_t i = 0; i < size / 2; i += 32) {
		sprite_reorder(&rom[2 * i]);
		sprite_reorder(&rom[size + 2 * i]);
		sprite_reorder(&rom[2 * size + 2 * i]);
	}
}
#undef BIT

// ---- the main CPU's devices ----
static uint16_t in_r(uint32_t a)
{
	if (a == 0x06000008)
		return (uint16_t)rbx_port_read(s_port_in1);
	if (a == 0x0600000a) {   // IN0: the EEPROM's DO (bit 6); the hopper and the RTC lines low
		const int64_t now = sh2_total_cycles(s_cpu);
		return (uint16_t)((rbx_port_read(s_port_in0) & ~0xe0u) | (eepser_do_read_93cxx(&s_eeprom, now) ? 0x40 : 0));
	}
	return 0xffff;
}

static void output_w(uint16_t v)
{
	const int64_t now = sh2_total_cycles(s_cpu);
	b.oki_bank = (v >> 5) & 1;
	s_oki.bank = (uint32_t)b.oki_bank * 0x40000;
	eepser_di_write(&s_eeprom, (v >> 15) & 1);
	eepser_clk_write(&s_eeprom, (v >> 14) & 1, now);
	eepser_cs_write(&s_eeprom, (v >> 13) & 1, now);
}

static void reg_w16(uint32_t a, uint16_t v)
{
	switch (a) {
	case 0x06000000: output_w(v); break;
	case 0x06000006: b.irq = 0; sh2_set_irq_line(s_cpu, 8, SH2_CLEAR); break;   // irq_ack_w
	default: break;
	}
}

static uint8_t rd8(void *u, uint32_t a)
{
	(void)u;
	if (a == 0x0600000d)
		return oki_read(&s_oki);
	const uint16_t w = in_r(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}
static uint16_t rd16(void *u, uint32_t a) { (void)u; return in_r(a); }
static uint32_t rd32(void *u, uint32_t a) { (void)u; return (uint32_t)in_r(a) << 16 | in_r(a + 2); }
static void wr8(void *u, uint32_t a, uint8_t v)
{
	(void)u;
	if (a == 0x0600000d)
		oki_write(&s_oki, v);
}
static void wr16(void *u, uint32_t a, uint16_t v) { (void)u; reg_w16(a, v); }
static void wr32(void *u, uint32_t a, uint32_t v)
{
	(void)u;
	reg_w16(a, (uint16_t)(v >> 16));
	reg_w16(a + 2, (uint16_t)v);
}

// ---- the drawing ----
static void fs_render(void)
{
	memcpy(rbx_video.palette, s_snap.pal, sizeof(s_snap.pal));
	for (int r = 0; r < VIS_H; r++) {
		uint16_t *d = rbx_video.bitmap + r * rbx_video.pitch;
		for (int x = 0; x < VIS_W; x++)
			d[x] = 0;
	}
	const sd_target tg = { rbx_video.bitmap, rbx_video.pitch, 0, 0, VIS_W, VIS_H, NULL };
	const uint16_t *spr = s_snap.spr;
	for (int i = 0x1000 - 4; i != -4; i -= 4) {   // sei25x_rise1x_device::draw
		uint32_t code = spr[i + 1];
		if (!s_gspr.count || code % s_gspr.count == 0)
			continue;
		const int flipy = (spr[i] >> 15) & 1, sizey = ((spr[i] >> 12) & 7) + 1;
		const int flipx = (spr[i] >> 11) & 1, sizex = ((spr[i] >> 8) & 7) + 1;
		const uint32_t color = spr[i] & 0x3f;
		int x = spr[i + 2] & 0x1ff, y = spr[i + 3] & 0x1ff;
		if (x >= 0x180) x -= 0x200;
		if (y >= 0x180) y -= 0x200;
		for (int ax = 0; ax < sizex; ax++) {
			const int sx = flipx ? x + 16 * (sizex - ax - 1) : x + 16 * ax;
			for (int ay = 0; ay < sizey; ay++) {
				const int sy = flipy ? y + 16 * (sizey - ay - 1) : y + 16 * ay;
				sd_transpen(&tg, &s_gspr, code++, color * 64, flipx, flipy, sx, sy, 63);
			}
		}
	}
}

static void capture(void)
{
	rbx_video_wait_idle();
	for (uint32_t i = 0; i < 0x1000; i++)
		s_snap.spr[i] = host_word(b.ram, 0x3e000 + i * 2);
	for (uint32_t i = 0; i < NPENS; i++) {   // xBGR_555
		const uint16_t w = host_word(b.pal, i * 2);
		const uint32_t r = w & 31, g = (w >> 5) & 31, bl = (w >> 10) & 31;
		s_snap.pal[i] = (r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (bl << 3 | bl >> 2);
	}
	rbx_video_frame_done();
}

// ---- init / frame ----
static void fs_reset(void)
{
	rbx_video_wait_idle();
	memset(&b, 0, offsetof(__typeof__(b), state_end));
	sh2_reset(s_cpu);
	sh2_idle_reset(&s_idle, s_cpu, getenv("RBX_SEIBU_NOIDLE") != NULL);
	oki_reset(&s_oki);
	s_oki.bank = 0;
	s_mix->n = 0;
	s_mix->frac = 0;
}

static void fs_exit(void)
{
	rbx_video_wait_idle();
	rbx_gfx_free(&s_gspr);
	free(s_cpu);
	s_cpu = NULL;
	free(s_bp);
	s_bp = NULL;
	free(s_snapp);
	s_snapp = NULL;
	free(s_mix);
	s_mix = NULL;
}

static int fs_init(const rbx_game *g)
{
	uint32_t ss, os;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *spr = rbx_region("sprites", &ss);
	const uint8_t *oki = rbx_region("oki", &os);
	if (!s_rom || s_rom_size < 0x40000 || !spr || ss < 0x600000 || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_cpu = calloc(1, sizeof(sh2_t));
	s_bp = calloc(1, sizeof(board_t));
	s_snapp = calloc(1, sizeof(snap_t));
	s_mix = calloc(1, sizeof(*s_mix));
	if (!s_cpu || !s_bp || !s_snapp || !s_mix) {
		fs_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	rise11_decrypt(spr, ss / 3, 0x9df5b2, 0x9ae999, 0x4a32e9, 0x968bd5, 0x1d97ac, 1);
	{   // spritelayout: 6 planes, RGN_FRAC(1,3)
		static const uint32_t sx[16] = { 7, 6, 5, 4, 3, 2, 1, 0, 23, 22, 21, 20, 19, 18, 17, 16 };
		static uint32_t sy[16];
		for (int i = 0; i < 16; i++)
			sy[i] = (uint32_t)i * 32;
		const uint32_t f = ss / 3 * 8;
		const uint32_t pl[6] = { 0, 8, f, f + 8, 2 * f, 2 * f + 8 };
		const rbx_gfx_layout ls = { 16, 16, 6, pl, sx, sy, 512 };
		if (rbx_gfx_decode(&s_gspr, &ls, spr, ss / 3) || rbx_video_setup(VIS_W, VIS_H, NPENS) || rbx_audio_setup(48000)) {
			fs_exit();
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
	}
	smix_init(s_mix, 48000, FRAME_MHZ, LINES);
	oki_init(&s_oki, OKI_CLOCK, 0, oki, os, 154);   // (0.60)
	eepser_init(&s_eeprom, EEPSER_93CXX, 128, 16, CPU_CLOCK);
	eepser_default(&s_eeprom, NULL, 0);
	s_port_in0 = rbx_port_find("IN0");
	s_port_in1 = rbx_port_find("IN1");
	sh2_swap32(s_rom, s_rom_size);
	sh2_init(s_cpu);
	s_cpu->read8 = rd8;
	s_cpu->read16 = rd16;
	s_cpu->read32 = rd32;
	s_cpu->write8 = wr8;
	s_cpu->write16 = wr16;
	s_cpu->write32 = wr32;
	sh2_map(s_cpu, 0x00000000, 0x0003ffff, s_rom, 0x40000, 1, 0);
	sh2_map(s_cpu, 0x02000000, 0x0203ffff, b.ram, sizeof(b.ram), 1, 1);
	sh2_map(s_cpu, 0x06018000, 0x06019fff, b.pal, sizeof(b.pal), 1, 1);
	fs_reset();
	return 0;
}


static void fs_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_H) {   // vblank: IRQ 8 asserted until acknowledged
			capture();
			b.irq = 1;
			sh2_set_irq_line(s_cpu, 8, SH2_ASSERT);
		}
		sh2_execute(s_cpu, rbx_line_cycles(&b.main_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		if (!(line & 15))
			sh2_idle_learn(&s_idle, s_cpu);
		smix_begin(s_mix);
		int32_t *tmp = smix_tmp(s_mix);
		const int n = rbx_line_cycles(&b.oki_frac, s_oki.rate, FRAME_MHZ, LINES);
		for (int i = 0; i < n && i < SMIX_TMP; i++)
			tmp[i] = oki_sample(&s_oki);
		smix_add(s_mix, tmp, n < SMIX_TMP ? n : SMIX_TMP);
		smix_end(s_mix);
	}
	smix_flush(s_mix);
}

static int fs_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "feversoc", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= rbx_state_io(ctx, "feversoc_cpu", s_cpu, SH2_STATE_SIZE);
	rc |= rbx_state_io(ctx, "feversoc_oki", s_oki.v, sizeof(s_oki.v));
	rc |= eepser_state(&s_eeprom, ctx, load, "feversoc_eeprom");
	if (load && !rc) {
		sh2_post_load(s_cpu);
		s_oki.bank = (uint32_t)b.oki_bank * 0x40000;
	}
	return rc ? -1 : 0;
}

static int fs_save(void *ctx) { return fs_state(ctx, 0); }
static int fs_load(void *ctx) { return fs_state(ctx, 1); }
static int fs_supported(const char *m) { return !strcmp(m, "feversoc"); }

const rbx_driver rbx_drv_feversoc = { "Seibu Fever Soccer", fs_init, fs_exit, fs_reset, fs_frame, fs_save, fs_load,
                                      fs_supported, fs_render, NULL };
