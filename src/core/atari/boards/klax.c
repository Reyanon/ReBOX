/*  Atari Klax hardware: Klax (the original sets; the MSM5205 bootlegs are not supported). Ported from MAME
    src/mame/atari/klax.cpp (BSD-3-Clause, copyright-holders: Aaron Giles).
    68000 at 7.16 MHz; a 64x32 playfield of 8x8 tiles (scanned by columns), the Atari motion objects (atarimo.c);
    an OKI6295 at 894.9 kHz (pin 7 high); a 2816 parallel EEPROM. IRQ 4 from the 32V timer and the VBLANK.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../sound/okim6295.h"
#include "../../video/gfxlayout.h"
#include "../video/atarimo.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAIN_CLOCK 7159090         // 14.318181 MHz / 2
#define LINE_CYC 456               // set_raw(14.318181 MHz / 2, 456, 0, 336, 262, 0, 240)
#define LINES 262
#define VIS_W 336
#define VIS_H 240
#define FRAME_MHZ 59923
#define SLICES 4

static uint8_t *s_rom;
static uint32_t s_rom_size;
static rbx_gfx s_gfx_pf, s_gfx_mo;
static amo_t s_mo;
static okim6295_t s_oki;

// main memory (host-order words)
static uint16_t s_vram[0x2000];      // 3f0000-3f3fff: playfield, SLIP, playfield ext, MO, RAM
static uint16_t s_pal[0x400];        // 3e0000-3e07ff (the high bytes)
static uint8_t s_eeprom[0x800];
static int s_eep_unlocked;
static char s_nv_path[512];
static int s_port[2];
static int s_line;
static int64_t s_mdebt;
static uint32_t s_oki_acc;

#define PF (s_vram)
#define SLIP (s_vram + 0x7c0)
#define PFEXT (s_vram + 0x800)
#define MOB (s_vram + 0x1000)

// video
static uint16_t *s_frame;            // VIS_W x VIS_H
static uint16_t *s_mobm;             // the MO bitmap
static struct { uint16_t frame[VIS_W * VIS_H]; uint32_t pal[0x200]; } *s_snap;

static inline uint8_t pal6bit(uint8_t x) { return (uint8_t)((x << 2) | (x >> 4)); }

static inline uint32_t irgb_1555(uint16_t raw)   // IRRRRRGGGGGBBBBB_decoder
{
	const uint8_t i = (raw >> 15) & 1;
	const uint32_t r = pal6bit((uint8_t)(((raw >> 9) & 0x3e) | i));
	const uint32_t g = pal6bit((uint8_t)(((raw >> 4) & 0x3e) | i));
	const uint32_t b = pal6bit((uint8_t)(((raw << 1) & 0x3e) | i));
	return r << 16 | g << 8 | b;
}

static void draw_frame(void)   // screen_update
{
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = s_frame + y * VIS_W;
		const int row = y >> 3, py = y & 7;
		for (int col = 0; col < VIS_W / 8; col++) {
			const int idx = col * 32 + row;
			const uint16_t data1 = PF[idx];
			const uint16_t data2 = PFEXT[idx] >> 8;
			const uint32_t code = rbx_wrap(data1 & 0x1fff, s_gfx_pf.count);
			const uint16_t color = (uint16_t)(256 + (data2 & 0x0f) * 16);
			const uint8_t *src = s_gfx_pf.pix + code * 64 + py * 8;
			uint16_t *dd = d + col * 8;
			if (data1 & 0x8000)
				for (int k = 0; k < 8; k++) dd[k] = (uint16_t)(color + src[7 - k]);
			else
				for (int k = 0; k < 8; k++) dd[k] = (uint16_t)(color + src[k]);
		}
	}
	amo_draw(&s_mo, MOB, SLIP, s_mobm, VIS_W, 0, VIS_W - 1, 0, VIS_H - 1);
	for (int i = 0; i < VIS_W * VIS_H; i++) {
		const uint16_t mo = s_mobm[i];
		if (mo != 0xffff && (s_frame[i] & 0xf0) != 0xf0)   // PFPRI if (PFS7-4 == 0 || LBPIX3-0 == 0)
			s_frame[i] = mo;
	}
}

// ---- main bus ----
static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a < 0x40000)
		return *(const uint16_t *)(s_rom + a);
	if (a >= 0x0e0000 && a < 0x0e1000)
		return (uint16_t)(0xff00 | s_eeprom[(a & 0xfff) >> 1]);
	switch (a & ~1u) {
	case 0x260000: {
		uint16_t v = (uint16_t)(rbx_port_read(s_port[0]) & ~0x800u);
		if (s_line >= VIS_H) v |= 0x800;   // VBLANK, active high
		return v;
	}
	case 0x260002: return (uint16_t)rbx_port_read(s_port[1]);
	case 0x270000: return (uint16_t)(0xff00 | oki_read(&s_oki));
	}
	if (a >= 0x3e0000 && a < 0x3e0800)
		return s_pal[(a & 0x7ff) >> 1];
	if (a >= 0x3f0000 && a < 0x3f4000)
		return s_vram[(a & 0x3fff) >> 1];
	return 0xffff;
}

static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a >= 0x0e0000 && a < 0x0e1000) {
		if ((mask & 0x00ff) && s_eep_unlocked) {
			s_eeprom[(a & 0xfff) >> 1] = (uint8_t)v;
			s_eep_unlocked = 0;   // (lock_after_write)
		}
		return;
	}
	if (a >= 0x1f0000 && a < 0x200000) {
		s_eep_unlocked = 1;
		return;
	}
	if (a >= 0x3e0000 && a < 0x3e0800) {
		uint16_t *p = &s_pal[(a & 0x7ff) >> 1];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
		return;
	}
	if (a >= 0x3f0000 && a < 0x3f4000) {
		uint16_t *p = &s_vram[(a & 0x3fff) >> 1];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
		return;
	}
	switch (a & ~1u) {
	case 0x270000:
		if (mask & 0x00ff)
			oki_write(&s_oki, (uint8_t)v);
		break;
	case 0x360000: m68k_bus_set_line(4, 0); break;   // interrupt_ack_w
	default: break;   // the latch, the watchdog
	}
}

static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	main_write16m(a & ~1u, (uint16_t)(v << 8 | v), a & 1 ? 0x00ff : 0xff00);
}

static void map_main(void)
{
	m68k_bus_reset_map();
	m68k_bus_map(0x000000, 0x03ffff, s_rom, 0x40000, 1, 0);
	m68k_bus_map(0x3e0000, 0x3e07ff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus_map(0x3f0000, 0x3f3fff, s_vram, sizeof(s_vram), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
}

// ---- frame ----
static void capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->frame, s_frame, sizeof(s_snap->frame));
	for (int i = 0; i < 0x200; i++)
		s_snap->pal[i] = irgb_1555((uint16_t)((s_pal[i * 2] & 0xff00) | (s_pal[i * 2 + 1] >> 8)));
	rbx_video_frame_done();
}

static void kx_render(void)
{
	for (int y = 0; y < VIS_H; y++)
		memcpy(rbx_video.bitmap + y * rbx_video.pitch, s_snap->frame + y * VIS_W, VIS_W * 2);
	memcpy(rbx_video.palette, s_snap->pal, sizeof(s_snap->pal));
}

static void kx_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line < VIS_H && !(line & 63))   // the 32V timer: (scanline & 32) == 0 and not in the VBLANK
			m68k_bus_set_line(4, 1);
		if (line == VIS_H) {   // VIDEO_UPDATE_BEFORE_VBLANK, then the VBLANK IRQ
			draw_frame();
			capture();
			m68k_bus_set_line(4, 1);
		}
		for (int left = LINE_CYC; left > 0;) {
			int step = LINE_CYC / SLICES;
			if (step > left)
				step = left;
			left -= step;
			s_mdebt += step;
			if (s_mdebt > 0)
				s_mdebt -= m68k_execute((int)s_mdebt);
		}
		s_oki_acc += s_oki.rate * LINE_CYC;
		while (s_oki_acc >= MAIN_CLOCK) {
			s_oki_acc -= MAIN_CLOCK;
			rbx_audio_put(oki_sample(&s_oki));
		}
	}
}

// ---- init / reset ----
static void kx_reset(void)
{
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_pal, 0, sizeof(s_pal));
	s_mo.bank = 0;
	amo_set_xscroll(&s_mo, 0);
	amo_set_yscroll(&s_mo, 0);
	s_eep_unlocked = 0;
	m68k_bus_irq_reset();
	oki_reset(&s_oki);
	s_oki_acc = 0;
	s_mdebt = 0;
	s_line = 0;
	m68k_pulse_reset();
}

static void kx_exit(void);

static int kx_init(const rbx_game *g)
{
	uint32_t ts, ss, os;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *tiles = rbx_region("tiles", &ts), *spr = rbx_region("sprites", &ss), *oki = rbx_region("oki", &os);
	if (!s_rom || s_rom_size < 0x40000 || !tiles || !spr || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	{   // gfx_8x8x4_packed_msb
		static const uint32_t pp[4] = { 0, 1, 2, 3 };
		static const uint32_t px[8] = { 0, 4, 8, 12, 16, 20, 24, 28 };
		static const uint32_t py[8] = { 0, 32, 64, 96, 128, 160, 192, 224 };
		const rbx_gfx_layout l = { 8, 8, 4, pp, px, py, 256 };
		if (rbx_gfx_decode(&s_gfx_pf, &l, tiles, ts) || rbx_gfx_decode(&s_gfx_mo, &l, spr, ss)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			kx_exit();
			return -1;
		}
	}
	static const amo_config moc = {
		1, 1, 0, 0, 0, 0,   // banks, linked, split, reverse, swapxy, nextneighbor
		8, 0, 0, 0x000, 0,  // slipheight, slipoffset, maxperline, palettebase, transpen
		{ 0x00ff, 0, 0, 0 }, { 0, 0x0fff, 0, 0 }, { 0, 0, 0x000f, 0 }, { 0, 0, 0xff80, 0 }, { 0, 0, 0, 0xff80 },
		{ 0, 0, 0, 0x0070 }, { 0, 0, 0, 0x0007 }, { 0, 0, 0, 0x0008 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, 0
	};
	if (amo_init(&s_mo, &moc, 8, 8)) {
		rbx_fail("%s: out of memory", g->set);
		kx_exit();
		return -1;
	}
	s_mo.gfx[0] = &s_gfx_mo;
	s_mo.granularity[0] = 16;
	s_frame = (uint16_t *)calloc(VIS_W * VIS_H, sizeof(uint16_t));
	s_mobm = (uint16_t *)calloc(VIS_W * VIS_H, sizeof(uint16_t));
	s_snap = calloc(1, sizeof(*s_snap));
	oki_init(&s_oki, 14318181u / 4 / 4, 1, oki, os, 256);
	if (!s_frame || !s_mobm || !s_snap || rbx_audio_setup(s_oki.rate) || rbx_video_setup(VIS_W, VIS_H, 0x200)) {
		rbx_fail("%s: out of memory", g->set);
		kx_exit();
		return -1;
	}
	s_port[0] = rbx_port_find("P1");
	s_port[1] = rbx_port_find("P2");
	// the EEPROM: blank (0xff), then the saved file
	memset(s_eeprom, 0xff, sizeof(s_eeprom));
	s_nv_path[0] = 0;
	{
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			FILE *f = fopen(s_nv_path, "rb");
			if (f) {
				if (fread(s_eeprom, 1, sizeof(s_eeprom), f) != sizeof(s_eeprom))
					memset(s_eeprom, 0xff, sizeof(s_eeprom));
				fclose(f);
			}
		}
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);
	map_main();
	kx_reset();
	return 0;
}

static void kx_exit(void)
{
	if (s_nv_path[0]) {
		FILE *f = fopen(s_nv_path, "wb");
		if (f) {
			fwrite(s_eeprom, 1, sizeof(s_eeprom), f);
			fclose(f);
		}
		s_nv_path[0] = 0;
	}
	m68k_set_mame_timing(0);
	amo_exit(&s_mo);
	rbx_gfx_free(&s_gfx_pf);
	rbx_gfx_free(&s_gfx_mo);
	free(s_frame);
	free(s_mobm);
	free(s_snap);
	s_frame = s_mobm = NULL;
	s_snap = NULL;
}

// ---- save state ----
static int kx_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	int32_t regs[3] = { s_eep_unlocked, (int32_t)s_mdebt, (int32_t)s_oki_acc };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "eeprom", s_eeprom, sizeof(s_eeprom));
	rc |= rbx_state_io(ctx, "regs", regs, sizeof(regs));
	rc |= rbx_state_io(ctx, "oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		s_eep_unlocked = regs[0];
		s_mdebt = regs[1];
		s_oki_acc = (uint32_t)regs[2];
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int kx_save(void *ctx) { return kx_state(ctx, 0); }
static int kx_load(void *ctx) { return kx_state(ctx, 1); }
static int kx_supported(const char *machine) { return !strcmp(machine, "klax"); }

const rbx_driver rbx_drv_klax = { "Atari Klax", kx_init, kx_exit, kx_reset, kx_frame, kx_save, kx_load,
                                  kx_supported, kx_render, NULL };
