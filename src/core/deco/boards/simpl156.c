/*  Data East "Simple" 156 hardware (DE-0409-1 / DE-0491-1, Mitchell MT5601-0): Joe & Mac Returns, Chain Reaction /
    Magical Drop, Charlie Ninja, Party Time, Osman. Ported from MAME src/mame/dataeast/simpl156.cpp
    (BSD-3-Clause, copyright-holders: David Haywood).
    156 = encrypted ARM (machine/deco156.c) at 28 MHz, tilemap chip 141 (video/deco16ic.c, encrypted tiles:
    machine/decocrpt.c), sprite chip 52 (video/decospr.c), two OKI M6295 (the music one banked), 93C46 EEPROM.
    The devices sit on the 32-bit bus as 16-bit parts: reads give 0xffff in the upper half, writes keep the lower.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/arm/arm.h"
#include "../../sound/okim6295.h"
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
#define FRAME_MHZ 58000      // screen.set_refresh_hz(58)
#define LINES 256            // set_size(64*8, 32*8)
#define VIS_Y0 8             // set_visarea(0, 319, 8, 247)
#define VIS_H 240
#define VIS_W 320
#define VBL_LINES 12         // vblank 800 us at 58 Hz x 256 lines (67.3 us a line)

// the address map of each machine: where the common block (RAM, sprites, palette, I/O, tilemaps) and the OKIs are
typedef struct { const char *machine; uint32_t common, oki_sfx, oki_music; int music_1mhz; } sp_map;
static const sp_map s_maps[] = {
	{ "joemacr", 0x100000, 0x180000, 0x1c0000, 0 },
	{ "chainrec", 0x400000, 0x480000, 0x3c0000, 0 },
	{ "magdrop", 0x380000, 0x400000, 0x340000, 0 },
	{ "magdropp", 0x680000, 0x780000, 0x4c0000, 0 },
	{ "mitchell156", 0x180000, 0x100000, 0x140000, 1 },
};
// MAME's idle skips (speedup read handlers): the work RAM word read at that pc spins 400 us
typedef struct { const char *set; uint32_t addr, pc; } sp_speedup;
static const sp_speedup s_speedups[] = {
	{ "joemacr", 0x018, 0x284 }, { "chainrec", 0x018, 0x2d4 }, { "prtytime", 0xae0, 0x4f0 },
	{ "charlien", 0x010, 0xc8c8 }, { "osman", 0x010, 0x5974 },
};

static const sp_map *s_m;
static arm_t s_cpu;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t s_sysram[0x1000];          // 0x201000 work RAM (32-bit), mirrored at 0x203000
static uint16_t s_mainram[0x2000];        // common + 0x00000 (16-bit on the 32-bit bus)
static uint16_t s_spriteram[0x800];
static uint16_t s_pal[0x400];
static uint16_t s_rowscroll[2][0x800];
static deco16ic_t s_tile;
static decospr_t s_spr;
static rbx_gfx s_gfx8, s_gfx16, s_gfxspr;
static okim6295_t s_oki_sfx, s_oki_music;
static eeprom93c46_t s_eeprom;
static char s_nv_path[256];
static int s_port_in0, s_port_in1;
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac;
static uint32_t s_spd_addr, s_spd_pc;     // idle skip (0: none)
static int64_t s_spin_until;              // the CPU spins (MAME spin_until_time) until this cycle count
static armpoll_t s_poll;                 // the polling-loop skip (machine/armpoll.h)
// MAME's speedup reads (spin_until_time(400 us) at that pc) as a polling-loop skip instead: the spin went past the
// interrupt the loop waits for, so a frame whose work ended in its last 400 us waited a whole frame more (Joe & Mac
// Returns' bonus stage at 30 Hz; docs/PERF.md 2026-11-03)
static armpoll_t s_poll_spd;
// sound: the music OKI's rate is the output rate; the sfx OKI's samples are held between its own
static uint32_t s_out_rate, s_sfx_rate, s_sfx_phase;
static int32_t s_sfx_last;
static uint64_t s_snd_frac;
static struct {
	deco16ic_t tile;
	uint16_t rowscroll[2][0x800], spriteram[0x800], pal[0x400];
	uint32_t frame;
} *s_snap;

// ---- bus ----
static void eeprom_w(uint32_t data)
{
	s_oki_music.bank = (data & 7) * 0x40000u;   // set_rom_bank
	eeprom_write_lines(&s_eeprom, (data >> 6) & 1, (data >> 5) & 1, (data >> 4) & 1);
}

static uint32_t in0_r(void)
{
	const int vbl = s_line >= VIS_Y0 + VIS_H && s_line < VIS_Y0 + VIS_H + VBL_LINES;
	rbx_port_custom(s_port_in0, 0x180, (vbl ? 0x80u : 0) | (eeprom_do(&s_eeprom) ? 0x100u : 0));
	return rbx_port_read(s_port_in0);
}

// the common block (offset from its base), 32-bit accesses
static uint32_t common_r(uint32_t o)
{
	if (o < 0x8000) return s_mainram[(o >> 2) & 0x1fff] | 0xffff0000u;
	if (o >= 0x10000 && o < 0x12000) return s_spriteram[(o - 0x10000) >> 2] | 0xffff0000u;
	if (o >= 0x20000 && o < 0x21000) return s_pal[(o - 0x20000) >> 2] | 0xffff0000u;
	if (o >= 0x30000 && o < 0x30004) return rbx_port_read(s_port_in1);
	if (o >= 0x40000 && o < 0x40020) return deco16ic_control_r(&s_tile, (int)((o - 0x40000) >> 2)) | 0xffff0000u;
	if (o >= 0x50000 && o < 0x54000) return s_tile.vram[0][((o - 0x50000) >> 2) & 0x7ff] | 0xffff0000u;
	if (o >= 0x54000 && o < 0x56000) return s_tile.vram[1][(o - 0x54000) >> 2] | 0xffff0000u;
	if (o >= 0x60000 && o < 0x62000) return s_rowscroll[0][(o - 0x60000) >> 2] | 0xffff0000u;
	if (o >= 0x64000 && o < 0x66000) return s_rowscroll[1][(o - 0x64000) >> 2] | 0xffff0000u;
	if (o >= 0x70000 && o < 0x70004) return 0;   // (readonly RAM nothing writes)
	return 0xffffffffu;
}

static inline void combine16(uint16_t *p, uint32_t data, uint32_t mask)
{
	mask &= 0xffff;   // (the 16-bit parts: the upper half of the bus is not connected)
	*p = (uint16_t)((*p & ~mask) | (data & mask));
}

static void common_w(uint32_t o, uint32_t data, uint32_t mask)
{
	if (o < 0x8000) combine16(&s_mainram[(o >> 2) & 0x1fff], data, mask);
	else if (o >= 0x10000 && o < 0x12000) combine16(&s_spriteram[(o - 0x10000) >> 2], data, mask);
	else if (o >= 0x20000 && o < 0x21000) combine16(&s_pal[(o - 0x20000) >> 2], data, mask);
	else if (o >= 0x30000 && o < 0x30004) eeprom_w(data);
	else if (o >= 0x40000 && o < 0x40020) {
		if (mask & 0xffff)
			deco16ic_control_w(&s_tile, (int)((o - 0x40000) >> 2), (uint16_t)data, (uint16_t)mask);
	} else if (o >= 0x50000 && o < 0x54000) combine16(&s_tile.vram[0][((o - 0x50000) >> 2) & 0x7ff], data, mask);
	else if (o >= 0x54000 && o < 0x56000) combine16(&s_tile.vram[1][(o - 0x54000) >> 2], data, mask);
	else if (o >= 0x60000 && o < 0x62000) combine16(&s_rowscroll[0][(o - 0x60000) >> 2], data, mask);
	else if (o >= 0x64000 && o < 0x66000) combine16(&s_rowscroll[1][(o - 0x64000) >> 2], data, mask);
}

static uint32_t bus_r32(void *ctx, uint32_t a)
{
	(void)ctx;
	a &= 0x03fffffc;
	if (a < 0x80000) {
		const uint8_t *p = s_rom + (a & (s_rom_size - 1));
		return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
	}
	if (a == 0x200000) {
		const uint32_t v = in0_r();
		armpoll_read(&s_poll, &s_cpu, v, s_rom, s_rom_size < 0x80000 ? s_rom_size : 0x80000);
		return v;
	}
	if ((a & ~0x2fffu) == 0x201000) {   // 0x201000-0x201fff, mirror 0x203000
		const uint32_t o = a & 0xffc;
		const uint8_t *p = s_sysram + o;
		const uint32_t v = (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
		if (o == s_spd_addr && s_spd_addr)   // (MAME's speedup location: its wait for the interrupt, skipped exactly)
			armpoll_read(&s_poll_spd, &s_cpu, v, s_rom, s_rom_size < 0x80000 ? s_rom_size : 0x80000);
		return v;
	}
	if (a == s_m->oki_sfx)
		return 0xffffff00u | oki_read(&s_oki_sfx);
	if (a == s_m->oki_music)
		return 0xffffff00u | oki_read(&s_oki_music);
	if (a >= s_m->common && a < s_m->common + 0x80000)
		return common_r(a - s_m->common);
	return 0xffffffffu;   // (unmap_value_high)
}

static void bus_w32m(uint32_t a, uint32_t data, uint32_t mask)
{
	a &= 0x03fffffc;
	if ((a & ~0x2fffu) == 0x201000) {
		uint8_t *p = s_sysram + (a & 0xffc);
		for (int i = 0; i < 4; i++)
			if (mask & (0xffu << (8 * i)))
				p[i] = (uint8_t)(data >> (8 * i));
		return;
	}
	if (a == s_m->oki_sfx) {
		if (mask & 0xff)
			oki_write(&s_oki_sfx, (uint8_t)data);
		return;
	}
	if (a == s_m->oki_music) {
		if (mask & 0xff)
			oki_write(&s_oki_music, (uint8_t)data);
		return;
	}
	if (a >= s_m->common && a < s_m->common + 0x80000)
		common_w(a - s_m->common, data, mask);
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
	default: return 0xf0 | 0xcc;   // (0x8000, 0xc000)
	}
}

static int bank_cb(int bank) { return (bank & 0x70) << 8; }

static uint8_t s_pri[VIS_W * VIS_H];

static void sp_render(void)
{
	const uint16_t *p = s_snap->pal;
	for (int i = 0; i < 0x400; i++) {   // xBGR_555
		int r = p[i] & 31, g = (p[i] >> 5) & 31, b = (p[i] >> 10) & 31;
		r = (r << 3) | (r >> 2); g = (g << 3) | (g >> 2); b = (b << 3) | (b >> 2);
		rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	memset(s_pri, 0, sizeof(s_pri));
	for (int y = 0; y < VIS_H; y++)   // bitmap.fill(256)
		for (int x = 0; x < VIS_W; x++)
			bm[y * pitch + x] = 256;
	deco16ic_update(&s_snap->tile);
	// (pri shares the bitmap's pitch in the draws: the bitmap is VIS_W wide here)
	deco16ic_draw(&s_snap->tile, 1, s_snap->rowscroll[1], bm, s_pri, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 2, 0xff);
	deco16ic_draw(&s_snap->tile, 0, s_snap->rowscroll[0], bm, s_pri, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 4, 0xff);
	decospr_draw(&s_spr, s_snap->spriteram, 0x1400 / 4, bm, s_pri, pitch, VIS_W, VIS_H, VIS_Y0, 1, s_snap->frame);
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

// ---- sound ----
static void sound_to_line(int line)
{
	// (samples up to the end of this line: the OKI commands of the line take effect at its end)
	s_snd_frac += (uint64_t)s_out_rate * 1000;
	const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
	while (s_snd_frac >= den) {
		s_snd_frac -= den;
		s_sfx_phase += s_sfx_rate;
		if (s_sfx_phase >= s_out_rate) {
			s_sfx_phase -= s_out_rate;
			s_sfx_last = oki_sample(&s_oki_sfx);
		}
		rbx_audio_put(s_sfx_last + oki_sample(&s_oki_music));
	}
	(void)line;
}

// ---- init / frame ----
static void irq_ack(void *ctx, int line)
{
	(void)ctx;
	if (line == ARM_IRQ_LINE)   // (HOLD_LINE: the IRQ line goes down when the CPU takes it)
		arm_set_irq(&s_cpu, 0);
}

static void sp_reset(void)
{
	memset(s_sysram, 0, sizeof(s_sysram));
	memset(s_mainram, 0, sizeof(s_mainram));
	memset(s_spriteram, 0xff, sizeof(s_spriteram));   // video_start: fill with 0xffff
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_rowscroll, 0, sizeof(s_rowscroll));
	memset(s_tile.vram, 0, sizeof(s_tile.vram));
	deco16ic_reset(&s_tile);
	oki_reset(&s_oki_sfx);
	oki_reset(&s_oki_music);
	s_oki_music.bank = 0;
	arm_reset(&s_cpu);
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = s_snd_frac = 0;
	s_sfx_phase = 0;
	s_sfx_last = 0;
	s_spin_until = 0;
	armpoll_reset(&s_poll);
	armpoll_reset(&s_poll_spd);
}

static void sp_exit(void);

static int sp_init(const rbx_game *g)
{
	s_m = NULL;
	for (size_t i = 0; i < sizeof(s_maps) / sizeof(s_maps[0]); i++)
		if (!strcmp(s_maps[i].machine, g->machine))
			s_m = &s_maps[i];
	uint32_t tsize, ssize, sfx_size, mus_size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *tiles = rbx_region("tiles", &tsize), *spr = rbx_region("sprites", &ssize);
	uint8_t *sfx = rbx_region("okisfx", &sfx_size), *mus = rbx_region("okimusic", &mus_size);
	if (!s_m || !s_rom || !tiles || !spr || !sfx || !mus) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	// init_simpl156: the music ROM's low address line goes to the banking (bitswap<24>(x, 23, 22, 21, 0, 20, ... 1))
	{
		uint8_t *buf = (uint8_t *)malloc(mus_size);
		if (!buf) {
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
		for (uint32_t x = 0; x < mus_size; x++) {
			const uint32_t addr = (x & 0xe00000) | ((x & 1) << 20) | ((x >> 1) & 0xfffff);
			if (addr < mus_size)
				buf[addr] = mus[x];
		}
		memcpy(mus, buf, mus_size);
		free(buf);
	}
	if (deco56_decrypt_gfx(tiles, tsize) || deco156_decrypt(s_rom, s_rom_size)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	// graphics: RGN_FRAC(1,2) layouts (planes at the second half + 8 / + 0, the first half + 8 / + 0)
	{
		const uint32_t th = tsize * 4, sh = ssize * 4;   // (the half, in bits)
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 16);
			x16[i] = (uint32_t)(256 + i);
			x16[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 16);
		const uint32_t pt[4] = { th + 8, th + 0, 8, 0 }, ps[4] = { sh + 8, sh + 0, 8, 0 };
		const rbx_gfx_layout l8 = { 8, 8, 4, pt, x8, y8, 8 * 16 };
		const rbx_gfx_layout l16 = { 16, 16, 4, pt, x16, y16, 32 * 16 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, x16, y16, 32 * 16 };
		if (rbx_gfx_decode(&s_gfx8, &l8, tiles, tsize / 2) || rbx_gfx_decode(&s_gfx16, &l16, tiles, tsize / 2) ||
		    rbx_gfx_decode(&s_gfxspr, &ls, spr, ssize / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			sp_exit();
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
	s_spr.color_base = 0x200;
	s_spr.pri_cb = pri_cb;
	s_spr.flip_screen = 1;   // (sprites are flipped relative to the tilemaps)
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_snap) {
		rbx_fail("%s: out of memory", g->set);
		sp_exit();
		return -1;
	}
	// idle skip of the set (or its parent)
	s_spd_addr = s_spd_pc = 0;
	for (size_t i = 0; i < sizeof(s_speedups) / sizeof(s_speedups[0]); i++)
		if (!strcmp(s_speedups[i].set, g->set) || (g->parent && !strcmp(s_speedups[i].set, g->parent))) {
			s_spd_addr = s_speedups[i].addr;
			s_spd_pc = s_speedups[i].pc;
		}
	if (!strcmp(g->set, "magdrop") || !strcmp(g->set, "magdropp") || !strcmp(g->set, "gangonta") ||
	    !strcmp(g->set, "candance"))
		;   // (MAME: init_chainrec / init_prtytime / init_osman by parent: kept above)
	// sound: OKI sfx 1 MHz (0.6), music 2 MHz (Mitchell 1 MHz, 0.2), pin 7 high
	const uint32_t mus_clock = s_m->music_1mhz ? CPU_CLOCK / 28 : CPU_CLOCK / 14;
	oki_init(&s_oki_sfx, CPU_CLOCK / 28, 1, sfx, sfx_size, 154);
	oki_init(&s_oki_music, mus_clock, 1, mus, mus_size, 51);
	s_out_rate = s_oki_music.rate;
	s_sfx_rate = s_oki_sfx.rate;
	if (rbx_audio_setup(s_out_rate) || rbx_video_setup(VIS_W, VIS_H, 0x400)) {
		sp_exit();
		return -1;
	}
	// EEPROM (kept per set)
	s_nv_path[0] = 0;
	eeprom_init(&s_eeprom);
	{	// (MAME: a region named as the device gives its first contents: big-endian 16-bit words)
		uint32_t es;
		const uint8_t *e = rbx_region("eeprom", &es);
		for (uint32_t i = 0; e && i < 64 && 2 * i + 1 < es; i++)
			s_eeprom.data[i] = (uint16_t)(e[2 * i] << 8 | e[2 * i + 1]);
	}
	{
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			eeprom_load(&s_eeprom, s_nv_path);
		}
	}
	s_port_in0 = rbx_port_find("IN0");
	s_port_in1 = rbx_port_find("IN1");
	// CPU: the ROM and the work RAM as direct pages, the rest through the handlers
	arm_init(&s_cpu);
	for (uint32_t a = 0; a < 0x80000 && a < s_rom_size; a += 0x1000)
		s_cpu.rpage[a >> 12] = s_rom + a;
	if (!s_spd_addr) {   // (an idle skip reads the work RAM through the handler)
		s_cpu.rpage[0x201] = s_cpu.rpage[0x203] = s_sysram;
	}
	s_cpu.wpage[0x201] = s_cpu.wpage[0x203] = s_sysram;
	s_cpu.read32 = bus_r32;
	s_cpu.write32 = bus_w32;
	s_cpu.read8 = bus_r8;
	s_cpu.write8 = bus_w8;
	s_cpu.ctx = NULL;
	s_cpu.irq_ack = irq_ack;
	sp_reset();
	return 0;
}

static void sp_exit(void)
{
	if (s_nv_path[0] && s_eeprom.dirty)
		eeprom_save(&s_eeprom, s_nv_path);
	s_nv_path[0] = 0;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
}

static void run_cpu(int cycles)
{
	const int64_t end = s_cpu.cycles + cycles;
	s_poll.slice_end = s_poll_spd.slice_end = end;
	while (s_cpu.cycles < end) {
		if (s_spin_until > s_cpu.cycles) {   // (spinning: the time passes without instructions)
			s_cpu.cycles = s_spin_until < end ? s_spin_until : end;
			continue;
		}
		arm_run(&s_cpu, end);
		arm_break = 0;
	}
}

static void sp_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // vblank start: the frame's picture, the IRQ (HOLD_LINE: down once taken)
			vblank_capture();
			arm_set_irq(&s_cpu, 1);
		}
		run_cpu(rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		sound_to_line(line);
	}
	arm_set_irq(&s_cpu, 0);   // (vblank end: CLEAR_LINE)
	s_frame++;
}

static int sp_state(void *ctx, int load)
{
	uint8_t cpu[ARM_STATE_SIZE];
	if (!load)
		memcpy(cpu, &s_cpu, ARM_STATE_SIZE);
	uint64_t fr[2] = { s_cpu_frac, s_snd_frac };
	uint32_t misc[5] = { s_frame, s_sfx_phase, (uint32_t)s_sfx_last, s_oki_music.bank, 0 };
	int64_t spin = s_spin_until;
	int rc = 0;
	rc |= rbx_state_io(ctx, "arm", cpu, ARM_STATE_SIZE);
	rc |= rbx_state_io(ctx, "sysram", s_sysram, sizeof(s_sysram));
	rc |= rbx_state_io(ctx, "mainram", s_mainram, sizeof(s_mainram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "rows", s_rowscroll, sizeof(s_rowscroll));
	rc |= rbx_state_io(ctx, "vram", s_tile.vram, sizeof(s_tile.vram));
	rc |= rbx_state_io(ctx, "tctl", s_tile.control, sizeof(s_tile.control));
	// (the voices and the command latch: the ROM pointers stay this run's)
	rc |= rbx_state_io(ctx, "oki1", s_oki_sfx.v, sizeof(s_oki_sfx.v) + sizeof(int));
	rc |= rbx_state_io(ctx, "oki2", s_oki_music.v, sizeof(s_oki_music.v) + sizeof(int));
	rc |= rbx_state_io(ctx, "eeprom", &s_eeprom, sizeof(s_eeprom));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	rc |= rbx_state_io(ctx, "spin", &spin, sizeof(spin));
	if (rc)
		return -1;
	if (load) {
		memcpy(&s_cpu, cpu, ARM_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_frame = misc[0];
		s_sfx_phase = misc[1];
		s_sfx_last = (int32_t)misc[2];
		s_oki_music.bank = misc[3];
		s_spin_until = spin;
		for (int i = 0; i < 8; i++)   // (the scroll and control values from the registers)
			deco16ic_control_w(&s_tile, i, s_tile.control[i], 0xffff);
	}
	return 0;
}

static int sp_save(void *ctx) { return sp_state(ctx, 0); }
static int sp_load(void *ctx) { return sp_state(ctx, 1); }
static int sp_supported(const char *machine)
{
	for (size_t i = 0; i < sizeof(s_maps) / sizeof(s_maps[0]); i++)
		if (!strcmp(s_maps[i].machine, machine))
			return 1;
	return 0;
}

const rbx_driver rbx_drv_simpl156 = { "DECO Simple 156", sp_init, sp_exit, sp_reset, sp_frame, sp_save, sp_load,
                                      sp_supported, sp_render, NULL };
