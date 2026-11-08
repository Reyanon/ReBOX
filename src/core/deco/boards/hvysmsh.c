/*  Data East DE-0385-2 (156 ARM): Heavy Smash; World Cup Volleyball '95 (the same board with a YMZ280B). Ported from
    MAME src/mame/dataeast/hvysmsh.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail, David Haywood).
    156 = encrypted ARM at 28 MHz (machine/deco156.c), tilemap chip 141 (video/deco16ic.c), sprite chip 52
    (video/decospr.c), 93C46 EEPROM; Heavy Smash: two OKI M6295 (both banked) with a volume latch.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/arm/arm.h"
#include "../../sound/okim6295.h"
#include "../../sound/ymz280b.h"
#include "../../machine/eeprom93c46.h"
#include "../../video/gfxlayout.h"
#include "../machine/deco156.h"
#include "../machine/decocrpt.h"
#include "../machine/armpoll.h"
#include "../video/deco16ic.h"
#include "../video/decospr.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 28000000
#define FRAME_MHZ 58000      // set_refresh_hz(58)
#define LINES 256            // set_size(40*8, 32*8)
#define VIS_Y0 8             // set_visarea(0, 319, 8, 247)
#define VIS_H 240
#define VIS_W 320
#define VBL_LINES 8          // vblank 529 us at 58 Hz x 256 lines (67.3 us a line)

static int s_wc;                         // World Cup Volleyball '95 (wcvol95 machine): its map, YMZ280B, xBGR_555
static arm_t s_cpu;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t s_ram[0x8000];             // 0x100000 (32-bit)
static uint32_t s_pal[0x400];             // 0x1c0000 (xBGR_888, 32-bit)
static uint16_t s_spriteram[0x800];
static uint16_t s_rowscroll[2][0x400];
static deco16ic_t s_tile;
static decospr_t s_spr;
static rbx_gfx s_gfx8, s_gfx16, s_gfxspr;
static okim6295_t s_oki[2];
static eeprom93c46_t s_eeprom;
static char s_nv_path[256];
static int s_port_in;
static int s_line;
static uint32_t s_frame;
static uint8_t s_volume;                  // volume_w: 0x00 max .. 0xff min
static uint64_t s_cpu_frac, s_snd_frac;
static uint32_t s_out_rate, s_sfx_rate, s_sfx_phase;
static int32_t s_sfx_last;
static armpoll_t s_poll;
// wcvol95: the YMZ280B (stereo, its own rate), rendered up to the CPU's time before each access
static ymz280b_t s_ymz;
static int32_t *s_ymz_l, *s_ymz_r;
static int s_ymz_cap, s_ymz_n;            // samples made this frame
static uint32_t s_ymz_line0, s_ymz_line_n; // samples at the line's start, in the line
static int64_t s_line_c0, s_line_cn;      // the line's first cycle, its cycles
static uint8_t s_pri[VIS_W * VIS_H];
static struct {
	deco16ic_t tile;
	uint16_t rowscroll[2][0x400], spriteram[0x800];
	uint32_t pal[0x400];
	uint32_t frame;
} *s_snap;

static uint32_t rd32le(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

// ---- bus ----
static uint32_t in_r(void)
{
	const int vbl = s_line >= VIS_Y0 + VIS_H && s_line < VIS_Y0 + VIS_H + VBL_LINES;
	rbx_port_custom(s_port_in, 0x01100000u, (vbl ? 0x00100000u : 0) | (eeprom_do(&s_eeprom) ? 0x01000000u : 0));
	return rbx_port_read(s_port_in);
}

static void oki_gain(void)
{
	// volume_w: linear (MAME: (0xff - data) / 255 on both); routes 1.0 and 0.35
	const int raw = 0xff - s_volume;
	s_oki[0].gain = 256 * raw / 255;
	s_oki[1].gain = 90 * raw / 255;
}

static uint32_t tilegen_r(uint32_t o)
{
	if (o < 0x20) return deco16ic_control_r(&s_tile, (int)(o >> 2)) | 0xffff0000u;
	if (o >= 0x10000 && o < 0x12000) return s_tile.vram[0][(o - 0x10000) >> 2] | 0xffff0000u;
	if (o >= 0x14000 && o < 0x16000) return s_tile.vram[1][(o - 0x14000) >> 2] | 0xffff0000u;
	if (o >= 0x20000 && o < 0x21000) return s_rowscroll[0][(o - 0x20000) >> 2] | 0xffff0000u;
	if (o >= 0x24000 && o < 0x25000) return s_rowscroll[1][(o - 0x24000) >> 2] | 0xffff0000u;
	return 0;
}

static inline void combine16(uint16_t *p, uint32_t data, uint32_t mask)
{
	mask &= 0xffff;
	*p = (uint16_t)((*p & ~mask) | (data & mask));
}

static void tilegen_w(uint32_t o, uint32_t data, uint32_t mask)
{
	if (o < 0x20) {
		if (mask & 0xffff)
			deco16ic_control_w(&s_tile, (int)(o >> 2), (uint16_t)data, (uint16_t)mask);
	} else if (o >= 0x10000 && o < 0x12000) combine16(&s_tile.vram[0][(o - 0x10000) >> 2], data, mask);
	else if (o >= 0x14000 && o < 0x16000) combine16(&s_tile.vram[1][(o - 0x14000) >> 2], data, mask);
	else if (o >= 0x20000 && o < 0x21000) combine16(&s_rowscroll[0][(o - 0x20000) >> 2], data, mask);
	else if (o >= 0x24000 && o < 0x25000) combine16(&s_rowscroll[1][(o - 0x24000) >> 2], data, mask);
}

static uint32_t bus_r32(void *ctx, uint32_t a)
{
	(void)ctx;
	a &= 0x03fffffc;
	if (a < 0x100000)
		return rd32le(s_rom + (a & (s_rom_size - 1)));
	if (a < 0x108000 && a >= 0x100000) {
		const uint32_t v = rd32le(s_ram + (a & 0x7ffc));
		if (a == 0x100104)   // (the main loop's wait for the interrupt's flag: the RAM page comes here for it)
			armpoll_read(&s_poll, &s_cpu, v, s_rom, s_rom_size);
		return v;
	}
	if (a == 0x120000) {
		const uint32_t v = in_r();
		armpoll_read(&s_poll, &s_cpu, v, s_rom, s_rom_size);
		return v;
	}
	if (a == 0x140000) return 0xffffff00u | oki_read(&s_oki[0]);
	if (a == 0x160000) return 0xffffff00u | oki_read(&s_oki[1]);
	if (a >= 0x180000 && a < 0x1b0000) return tilegen_r(a - 0x180000);
	if (a >= 0x1c0000 && a < 0x1c1000) return s_pal[(a - 0x1c0000) >> 2];
	if (a >= 0x1d0010 && a < 0x1d0030) return 0;   // (nopr)
	if (a >= 0x1e0000 && a < 0x1e2000) return s_spriteram[(a - 0x1e0000) >> 2] | 0xffff0000u;
	return 0;
}

static void bus_w32m(uint32_t a, uint32_t data, uint32_t mask)
{
	a &= 0x03fffffc;
	if (a >= 0x100000 && a < 0x108000) {
		uint8_t *p = s_ram + (a & 0x7ffc);
		for (int i = 0; i < 4; i++)
			if (mask & (0xffu << (8 * i)))
				p[i] = (uint8_t)(data >> (8 * i));
		return;
	}
	switch (a) {
	case 0x120000:   // volume_w (the byte at 0x120000)
		if (mask & 0xff) {
			s_volume = (uint8_t)data;
			oki_gain();
		}
		return;
	case 0x120004:   // eeprom_w: the music OKI's bank, the EEPROM lines (EEPROMOUT bits 4-6)
		if (mask & 0xff) {
			s_oki[1].bank = (data & 7) * 0x40000u;
			eeprom_write_lines(&s_eeprom, (data >> 6) & 1, (data >> 5) & 1, (data >> 4) & 1);
		}
		return;
	case 0x12000c:   // oki_0_bank_w
		s_oki[0].bank = (data & 1) * 0x40000u;
		return;
	case 0x140000:
		if (mask & 0xff)
			oki_write(&s_oki[0], (uint8_t)data);
		return;
	case 0x160000:
		if (mask & 0xff)
			oki_write(&s_oki[1], (uint8_t)data);
		return;
	}
	if (a >= 0x180000 && a < 0x1b0000)
		tilegen_w(a - 0x180000, data, mask);
	else if (a >= 0x1c0000 && a < 0x1c1000) {
		uint32_t *p = &s_pal[(a - 0x1c0000) >> 2];
		*p = (*p & ~mask) | (data & mask);
	} else if (a >= 0x1e0000 && a < 0x1e2000)
		combine16(&s_spriteram[(a - 0x1e0000) >> 2], data, mask);
}

// ---- wcvol95 bus ----
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

static uint32_t wc_r32(void *ctx, uint32_t a)
{
	(void)ctx;
	a &= 0x03fffffc;
	if (a < 0x100000)
		return rd32le(s_rom + (a & (s_rom_size - 1)));
	if (a >= 0x100000 && a < 0x130000) return tilegen_r(a - 0x100000);
	if (a >= 0x130000 && a < 0x138000) {
		const uint32_t v = rd32le(s_ram + (a & 0x7ffc));
		if (a == 0x137100)   // (the wait for the interrupt's counter, 0x38cc0: the RAM page comes here for it)
			armpoll_read(&s_poll, &s_cpu, v, s_rom, s_rom_size);
		return v;
	}
	if (a == 0x140000) {
		const uint32_t v = in_r();
		armpoll_read(&s_poll, &s_cpu, v, s_rom, s_rom_size);
		return v;
	}
	if (a >= 0x160000 && a < 0x162000) return s_spriteram[(a - 0x160000) >> 2] | 0xffff0000u;
	if (a >= 0x180000 && a < 0x181000) return s_pal[(a - 0x180000) >> 2];   // (readonly: the RAM as written)
	if (a >= 0x1a0000 && a < 0x1a0008) {
		ymz_sync();
		return ymz280b_read(&s_ymz, (int)((a - 0x1a0000) >> 2));
	}
	return 0;
}

static void wc_w32m(uint32_t a, uint32_t data, uint32_t mask)
{
	a &= 0x03fffffc;
	if (a >= 0x100000 && a < 0x130000)
		tilegen_w(a - 0x100000, data, mask);
	else if (a >= 0x130000 && a < 0x138000) {
		uint8_t *p = s_ram + (a & 0x7ffc);
		for (int i = 0; i < 4; i++)
			if (mask & (0xffu << (8 * i)))
				p[i] = (uint8_t)(data >> (8 * i));
	} else if (a == 0x150000)   // EEPROMOUT: di bit 0, clk bit 1, cs bit 2
		eeprom_write_lines(&s_eeprom, (data >> 2) & 1, (data >> 1) & 1, data & 1);
	else if (a >= 0x160000 && a < 0x162000)
		combine16(&s_spriteram[(a - 0x160000) >> 2], data, mask);
	else if (a >= 0x180000 && a < 0x181000) {   // palette write16, umask32 0x0000ffff
		uint32_t *p = &s_pal[(a - 0x180000) >> 2];
		const uint32_t m = mask & 0xffff;
		*p = (*p & ~m) | (data & m);
	} else if (a >= 0x1a0000 && a < 0x1a0008) {
		if (mask & 0xff) {
			ymz_sync();
			ymz280b_write(&s_ymz, (int)((a - 0x1a0000) >> 2), (uint8_t)data);
		}
	}
}

static void wc_w32(void *ctx, uint32_t a, uint32_t data) { (void)ctx; wc_w32m(a, data, 0xffffffffu); }
static uint8_t wc_r8(void *ctx, uint32_t a) { return (uint8_t)(wc_r32(ctx, a) >> (8 * (a & 3))); }
static void wc_w8(void *ctx, uint32_t a, uint8_t v)
{
	(void)ctx;
	const int sh = 8 * (int)(a & 3);
	wc_w32m(a, (uint32_t)v << sh, 0xffu << sh);
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
	switch (x & 0xc000) {
	case 0x0000: return 0;
	case 0x4000: return 0xf0;
	default: return 0xf0 | 0xcc;
	}
}

static int bank_cb(int bank) { return (bank & 0x70) << 8; }

static void hs_render(void)
{
	const uint32_t *p = s_snap->pal;
	if (s_wc)
		for (int i = 0; i < 0x400; i++) {   // xBGR_555
			int r = p[i] & 31, g = (p[i] >> 5) & 31, b = (p[i] >> 10) & 31;
			r = (r << 3) | (r >> 2); g = (g << 3) | (g >> 2); b = (b << 3) | (b >> 2);
			rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | b);
		}
	else
		for (int i = 0; i < 0x400; i++)   // xBGR_888
			rbx_video.palette[i] = (p[i] & 0xff) << 16 | (p[i] & 0xff00) | ((p[i] >> 16) & 0xff);
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	memset(s_pri, 0, sizeof(s_pri));
	deco16ic_update(&s_snap->tile);
	// the back playfield opaque, the sprites, the front playfield over them (priority 0 throughout)
	deco16ic_draw(&s_snap->tile, 1, s_snap->rowscroll[1], bm, s_pri, pitch, VIS_W, VIS_Y0, 0, VIS_H, 1, 0, 0xff);
	decospr_draw(&s_spr, s_snap->spriteram, 0x800, bm, s_pri, pitch, VIS_W, VIS_H, VIS_Y0, 1, s_snap->frame);
	deco16ic_draw(&s_snap->tile, 0, s_snap->rowscroll[0], bm, s_pri, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	s_snap->tile = s_tile;
	memcpy(s_snap->rowscroll, s_rowscroll, sizeof(s_rowscroll));
	memcpy(s_snap->spriteram, s_spriteram, sizeof(s_spriteram));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- sound: the 2 MHz OKI's rate out, the 1 MHz one's samples held ----
static void sound_line(void)
{
	s_snd_frac += (uint64_t)s_out_rate * 1000;
	const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
	while (s_snd_frac >= den) {
		s_snd_frac -= den;
		s_sfx_phase += s_sfx_rate;
		if (s_sfx_phase >= s_out_rate) {
			s_sfx_phase -= s_out_rate;
			s_sfx_last = oki_sample(&s_oki[0]);
		}
		rbx_audio_put(s_sfx_last + oki_sample(&s_oki[1]));
	}
}

// ---- init / frame ----
static void irq_ack(void *ctx, int line)
{
	(void)ctx;
	if (line == ARM_IRQ_LINE)   // (HOLD_LINE)
		arm_set_irq(&s_cpu, 0);
}

static void hs_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_rowscroll, 0, sizeof(s_rowscroll));
	memset(s_tile.vram, 0, sizeof(s_tile.vram));
	deco16ic_reset(&s_tile);
	for (int i = 0; i < 2; i++) {
		oki_reset(&s_oki[i]);
		s_oki[i].bank = 0;
	}
	s_volume = 0;
	if (!s_wc)
		oki_gain();
	else
		ymz280b_reset(&s_ymz);
	s_ymz_n = 0;
	arm_reset(&s_cpu);
	armpoll_reset(&s_poll);
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = s_snd_frac = 0;
	s_sfx_phase = 0;
	s_sfx_last = 0;
}

static void hs_exit(void);

// MAME descramble_sound: the low address line goes to the banking (bitswap<24>(x, 23, 22, 21, 0, 20, ... 1))
static int descramble(uint8_t *rom, uint32_t size)
{
	uint8_t *buf = (uint8_t *)malloc(size);
	if (!buf)
		return -1;
	for (uint32_t x = 0; x < size; x++) {
		const uint32_t addr = (x & 0xe00000) | ((x & 1) << 20) | ((x >> 1) & 0xfffff);
		if (addr < size)
			buf[addr] = rom[x];
	}
	memcpy(rom, buf, size);
	free(buf);
	return 0;
}

static int hs_init(const rbx_game *g)
{
	uint32_t tsize, ssize, o1s, o2s;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *tiles = rbx_region("tiles", &tsize), *spr = rbx_region("sprites", &ssize);
	s_wc = !strcmp(g->machine, "wcvol95");
	uint8_t *oki1 = NULL, *oki2 = NULL, *ymz = NULL;
	if (s_wc)
		ymz = rbx_region("ymz", &o2s);
	else {
		oki1 = rbx_region("oki1", &o1s);
		oki2 = rbx_region("oki2", &o2s);
	}
	if (!s_rom || !tiles || !spr || (s_wc ? !ymz : (!oki1 || !oki2))) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (deco56_decrypt_gfx(tiles, tsize) || deco156_decrypt(s_rom, s_rom_size) || descramble(s_wc ? ymz : oki2, o2s)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	{	// RGN_FRAC(1,2) layouts as simpl156's
		const uint32_t th = tsize * 4, sh = ssize * 4;
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 16);
			x16[i] = (uint32_t)(256 + i);
			x16[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 16);
		const uint32_t pt[4] = { th + 8, th, 8, 0 }, ps[4] = { sh + 8, sh, 8, 0 };
		const rbx_gfx_layout l8 = { 8, 8, 4, pt, x8, y8, 16 * 8 };
		const rbx_gfx_layout l16 = { 16, 16, 4, pt, x16, y16, 64 * 8 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, x16, y16, 64 * 8 };
		if (rbx_gfx_decode(&s_gfx8, &l8, tiles, tsize / 2) || rbx_gfx_decode(&s_gfx16, &l16, tiles, tsize / 2) ||
		    rbx_gfx_decode(&s_gfxspr, &ls, spr, ssize / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			hs_exit();
			return -1;
		}
	}
	memset(&s_tile, 0, sizeof(s_tile));
	for (int i = 0; i < 2; i++) {
		s_tile.tm[i].size = DECO_64x32;
		s_tile.tm[i].col_bank = i ? 0x10 : 0x00;
		s_tile.tm[i].col_mask = 0x0f;
		s_tile.tm[i].bank_cb = bank_cb;
	}
	s_tile.gfx8 = &s_gfx8;
	s_tile.gfx16 = &s_gfx16;
	memset(&s_spr, 0, sizeof(s_spr));
	s_spr.gfx = &s_gfxspr;
	s_spr.color_base = 512;
	s_spr.pri_cb = pri_cb;
	s_spr.flip_screen = 1;   // (sprites are flipped relative to the tilemaps)
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_snap) {
		rbx_fail("%s: out of memory", g->set);
		hs_exit();
		return -1;
	}
	if (s_wc) {   // YMZ280B at 14 MHz, both outputs at 1.0
		ymz280b_init(&s_ymz, CPU_CLOCK / 2, ymz, o2s, NULL, NULL);
		s_out_rate = ymz280b_rate(&s_ymz);
		s_ymz_cap = (int)(s_out_rate / 50) + 64;
		s_ymz_l = (int32_t *)calloc((size_t)s_ymz_cap, sizeof(int32_t));
		s_ymz_r = (int32_t *)calloc((size_t)s_ymz_cap, sizeof(int32_t));
		if (!s_ymz_l || !s_ymz_r || rbx_audio_setup_stereo(s_out_rate) || rbx_video_setup(VIS_W, VIS_H, 0x400)) {
			hs_exit();
			return -1;
		}
	} else {
		oki_init(&s_oki[0], CPU_CLOCK / 28, 1, oki1, o1s, 256);
		oki_init(&s_oki[1], CPU_CLOCK / 14, 1, oki2, o2s, 90);
		s_out_rate = s_oki[1].rate;
		s_sfx_rate = s_oki[0].rate;
	}
	if ((!s_wc && rbx_audio_setup(s_out_rate)) || rbx_video_setup(VIS_W, VIS_H, 0x400)) {
		hs_exit();
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
	s_port_in = rbx_port_find("INPUTS");
	arm_init(&s_cpu);
	for (uint32_t a = 0; a < 0x100000 && a < s_rom_size; a += 0x1000)
		s_cpu.rpage[a >> 12] = s_rom + a;
	const uint32_t ram = s_wc ? 0x130000 : 0x100000;
	for (uint32_t a = 0; a < 0x8000; a += 0x1000)
		s_cpu.rpage[(ram + a) >> 12] = s_cpu.wpage[(ram + a) >> 12] = s_ram + a;
	s_cpu.rpage[(s_wc ? 0x137000 : 0x100000) >> 12] = NULL;   // (reads of the page with the polled flag: the handler)
	s_cpu.read32 = s_wc ? wc_r32 : bus_r32;
	s_cpu.write32 = s_wc ? wc_w32 : bus_w32;
	s_cpu.read8 = s_wc ? wc_r8 : bus_r8;
	s_cpu.write8 = s_wc ? wc_w8 : bus_w8;
	s_cpu.ctx = NULL;
	s_cpu.irq_ack = irq_ack;
	hs_reset();
	return 0;
}

static void hs_exit(void)
{
	if (s_nv_path[0] && s_eeprom.dirty)
		eeprom_save(&s_eeprom, s_nv_path);
	s_nv_path[0] = 0;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
	free(s_ymz_l);
	free(s_ymz_r);
	s_ymz_l = s_ymz_r = NULL;
}

static void hs_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {
			vblank_capture();
			arm_set_irq(&s_cpu, 1);
		}
		const int lc = rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		const int64_t end = s_cpu.cycles + lc;
		s_poll.slice_end = end;
		if (s_wc) {   // (the YMZ's samples of this line, for ymz_sync)
			s_snd_frac += (uint64_t)s_out_rate * 1000;
			const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
			s_ymz_line0 = (uint32_t)s_ymz_n;
			s_ymz_line_n = (uint32_t)rbx_div64_small(s_snd_frac, den);
			s_snd_frac -= (uint64_t)s_ymz_line_n * den;
			s_line_c0 = s_cpu.cycles;
			s_line_cn = lc;
		}
		arm_run(&s_cpu, end);
		if (s_wc)
			ymz_to((int)(s_ymz_line0 + s_ymz_line_n));
		else
			sound_line();
	}
	if (s_wc) {
		for (int i = 0; i < s_ymz_n; i++)
			rbx_audio_put_stereo(s_ymz_l[i], s_ymz_r[i]);
		memset(s_ymz_l, 0, sizeof(int32_t) * (size_t)s_ymz_n);
		memset(s_ymz_r, 0, sizeof(int32_t) * (size_t)s_ymz_n);
		s_ymz_n = 0;
	}
	arm_set_irq(&s_cpu, 0);
	s_frame++;
}

static int hs_state(void *ctx, int load)
{
	uint8_t cpu[ARM_STATE_SIZE];
	if (!load)
		memcpy(cpu, &s_cpu, ARM_STATE_SIZE);
	uint64_t fr[2] = { s_cpu_frac, s_snd_frac };
	uint32_t misc[6] = { s_frame, s_sfx_phase, (uint32_t)s_sfx_last, s_oki[0].bank, s_oki[1].bank, s_volume };
	int rc = 0;
	rc |= rbx_state_io(ctx, "arm", cpu, ARM_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "rows", s_rowscroll, sizeof(s_rowscroll));
	rc |= rbx_state_io(ctx, "vram", s_tile.vram, sizeof(s_tile.vram));
	rc |= rbx_state_io(ctx, "tctl", s_tile.control, sizeof(s_tile.control));
	rc |= rbx_state_io(ctx, "oki1", s_oki[0].v, sizeof(s_oki[0].v) + sizeof(int));
	rc |= rbx_state_io(ctx, "oki2", s_oki[1].v, sizeof(s_oki[1].v) + sizeof(int));
	rc |= rbx_state_io(ctx, "eeprom", &s_eeprom, sizeof(s_eeprom));
	if (s_wc)
		rc |= rbx_state_io(ctx, "ymz", &s_ymz, YMZ280B_STATE_SIZE);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (rc)
		return -1;
	if (load) {
		memcpy(&s_cpu, cpu, ARM_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_frame = misc[0];
		s_sfx_phase = misc[1];
		s_sfx_last = (int32_t)misc[2];
		s_oki[0].bank = misc[3];
		s_oki[1].bank = misc[4];
		s_volume = (uint8_t)misc[5];
		if (!s_wc)
			oki_gain();
		for (int i = 0; i < 8; i++)
			deco16ic_control_w(&s_tile, i, s_tile.control[i], 0xffff);
		armpoll_reset(&s_poll);
	}
	return 0;
}

static int hs_save(void *ctx) { return hs_state(ctx, 0); }
static int hs_load(void *ctx) { return hs_state(ctx, 1); }
static int hs_supported(const char *machine) { return !strcmp(machine, "hvysmsh") || !strcmp(machine, "wcvol95"); }

const rbx_driver rbx_drv_hvysmsh = { "DECO 156 Heavy Smash", hs_init, hs_exit, hs_reset, hs_frame, hs_save, hs_load,
                                     hs_supported, hs_render, NULL };
