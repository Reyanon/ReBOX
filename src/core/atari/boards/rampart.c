/*  Atari Rampart hardware: Rampart (trackball, joystick, Japan). Ported from MAME src/mame/atari/rampart.cpp
    (BSD-3-Clause, copyright-holders: Aaron Giles).
    68000 at 7.16 MHz; a 512x256 8-bit bitmap playfield, the Atari motion objects (atarimo.c); the slapstic 118 on
    0x140000-0x147fff; an OKI6295 at 1.19 MHz (pin 7 low) and a YM2413 at 3.58 MHz; a 2816 parallel EEPROM.
    IRQ 4 from the 32V timer only.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../video/atarimo.h"
#include "../machine/slapstic.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAIN_CLOCK 7159090         // 14.318181 MHz / 2
#define YM_LINE_CLK 228            // 3.58 MHz: half the 68000's clocks of a line
#define LINE_CYC 456               // set_raw(14.318181 MHz / 2, 456, 0+12, 336+12, 262, 0, 240)
#define LINES 262
#define VIS_X0 12
#define VIS_W 336
#define VIS_H 240
#define BM_W (VIS_X0 + VIS_W)      // the MO bitmap: screen columns 0 .. 347
#define FRAME_MHZ 59923
#define SLICES 4

static uint8_t *s_rom;
static uint32_t s_rom_size;
static rbx_gfx s_gfx;
static amo_t s_mo;
static slapstic_t s_slap;
static okim6295_t s_oki;

// main memory (host-order words)
static uint16_t *s_bitmap;           // 200000-21ffff (0x10000 words)
static uint16_t s_ram[0x8000];       // 3e0000-3effff: MO at 0, the SLIP RAM at 0x3f40
static uint16_t s_pal[0x400];        // 3c0000-3c07ff (the high bytes)
static uint8_t s_eeprom[0x800];
static int s_eep_unlocked;
static char s_nv_path[512];
static int s_port[7];
static int s_line;
static int64_t s_mdebt;
static uint16_t s_latch;
static uint32_t s_oki_acc;
static int32_t s_oki_hold;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;

#define MOB (s_ram)
#define SLIP (s_ram + 0x3f40 / 2)

// video
static uint16_t *s_frame;            // VIS_W x VIS_H
static uint16_t *s_mobm;             // the MO bitmap, BM_W x VIS_H
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

static void draw_frame(void)   // screen_update: the bitmap, the MO over it
{
	amo_draw(&s_mo, MOB, SLIP, s_mobm, BM_W, VIS_X0, BM_W - 1, 0, VIS_H - 1);
	for (int y = 0; y < VIS_H; y++) {
		const uint16_t *src = s_bitmap + 256 * y;
		const uint16_t *mo = s_mobm + y * BM_W + VIS_X0;
		uint16_t *d = s_frame + y * VIS_W;
		for (int x = 0; x < VIS_W; x += 2) {   // screen x = VIS_X0 + x: src[(x - 8) / 2]
			const uint16_t bits = src[(VIS_X0 + x - 8) / 2];
			d[x] = bits >> 8;
			d[x + 1] = bits & 0xff;
		}
		for (int x = 0; x < VIS_W; x++)
			if (mo[x] != 0xffff)
				d[x] = mo[x];
	}
}

// ---- sound ----
static void latch_w(uint16_t v, uint16_t mask)
{
	if (!(mask & 0x00ff))
		return;
	s_latch = (uint16_t)((s_latch & ~mask) | (v & mask));
	s_oki.gain = v & 0x20 ? 154 : 0;   // 0.60 route x the PMIX0 gain
	if (!(v & 0x10))
		oki_reset(&s_oki);
	const int g = (int)(((v >> 1) & 7) * 256 / 7);
	rbx_ym_set_gain(0, 0, g);
	rbx_ym_set_gain(0, 1, g);
	if (!(v & 0x01))
		rbx_ym_reset(0);
}

// ---- main bus ----
static uint32_t s_slices;
static void slap_tap(uint32_t a)
{
	const uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC) & 0x7fffff;
	const uint32_t inst = s_slices * 4096u + (uint32_t)m68k_cycles_remaining();
	slapstic_tap(&s_slap, a & 0x7fffff, pc, (pc & 0x3c0000) == 0x140000, inst);
}

static uint16_t main_read16(uint32_t a)
{
	a &= 0x7fffff;
	if (a < 0x100000)
		return *(const uint16_t *)(s_rom + a);
	if ((a & 0x3c0000) == 0x140000) {   // the slapstic's bank (mirrored by 0x43e000)
		const uint16_t v = *(const uint16_t *)(s_rom + 0x40000 + s_slap.current_bank * 0x2000 + (a & 0x1ffe));
		slap_tap(a);
		return v;
	}
	if ((a & 0x7e0000) == 0x200000)
		return s_bitmap[(a & 0x1ffff) >> 1];
	if ((a & 0x7e0000) == 0x3e0000)
		return s_ram[(a & 0xffff) >> 1];
	switch (a & 0x7e6000) {
	case 0x3c0000:
		return s_pal[(a & 0x7ff) >> 1];
	case 0x460000: return (uint16_t)(oki_read(&s_oki) << 8 | 0xff);
	case 0x500000: return (uint16_t)(0xff00 | s_eeprom[(a & 0xfff) >> 1]);
	case 0x640000: {
		if (a & 2) {
			uint16_t v = (uint16_t)(rbx_port_read(s_port[1]) & ~0x400u);
			if (s_port[2] < 0 || !(rbx_port_read(s_port[2]) & 2)) v |= 0x400;   // (custom, active low: inverted)
			return v;
		}
		uint16_t v = (uint16_t)(rbx_port_read(s_port[0]) & ~0x0c03u);
		{   // the P3 buttons (custom, active low: inverted)
			const uint32_t p3 = s_port[2] >= 0 ? ~rbx_port_read(s_port[2]) : ~0u;
			v |= (uint16_t)((p3 & 3) | (p3 & 1) << 10);
		}
		if (s_line >= VIS_H) v |= 0x800;   // VBLANK, active high
		return v;
	}
	case 0x6c0000: case 0x6c2000: case 0x6c4000: case 0x6c6000:
		return (uint16_t)rbx_port_read(s_port[3 + ((a >> 1) & 3)]);
	}
	return 0xffff;
}

static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0x7fffff;
	if ((a & 0x3c0000) == 0x140000) {   // (the slapstic's range, mirrored by 0x438000)
		slap_tap(a);
		return;
	}
	if ((a & 0x7e0000) == 0x200000) {
		uint16_t *p = &s_bitmap[(a & 0x1ffff) >> 1];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
		return;
	}
	if ((a & 0x7e0000) == 0x3e0000) {
		uint16_t *p = &s_ram[(a & 0xffff) >> 1];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
		return;
	}
	switch (a & 0x7e6000) {
	case 0x3c0000:
		{
			uint16_t *p = &s_pal[(a & 0x7ff) >> 1];
			*p = (uint16_t)((*p & ~mask) | (v & mask));
		}
		break;
	case 0x460000:
		if (mask & 0xff00)
			oki_write(&s_oki, (uint8_t)(v >> 8));
		break;
	case 0x480000:
		if (mask & 0xff00)
			rbx_ym_write(0, (a >> 1) & 1, (uint8_t)(v >> 8));
		break;
	case 0x500000:
		if ((mask & 0x00ff) && s_eep_unlocked) {
			s_eeprom[(a & 0xfff) >> 1] = (uint8_t)v;
			s_eep_unlocked = 0;   // (lock_after_write)
		}
		break;
	case 0x5a6000: s_eep_unlocked = 1; break;
	case 0x640000: latch_w(v, mask); break;
	case 0x7e6000: m68k_bus_set_line(4, 0); break;   // scanline_int_ack_w
	default: break;   // the watchdog
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
	m68k_bus_map(0x000000, 0x0fffff, s_rom, 0x100000, 1, 0);
	m68k_bus_map(0x200000, 0x21ffff, s_bitmap, 0x20000, 1, 1);
	m68k_bus_map(0x3e0000, 0x3effff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x3f0000, 0x3fffff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x800000, 0x8fffff, s_rom, 0x100000, 1, 0);   // (the global mask 0x7fffff)
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

static void rp_render(void)
{
	for (int y = 0; y < VIS_H; y++)
		memcpy(rbx_video.bitmap + y * rbx_video.pitch, s_snap->frame + y * VIS_W, VIS_W * 2);
	memcpy(rbx_video.palette, s_snap->pal, sizeof(s_snap->pal));
}

static void rp_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	const uint32_t yr = rbx_ym_rate(0);
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (!(line & 63))   // the 32V timer: (scanline & 32) == 0
			m68k_bus_set_line(4, 1);
		if (line == VIS_H) {   // VIDEO_UPDATE_BEFORE_VBLANK
			draw_frame();
			capture();
		}
		for (int left = LINE_CYC; left > 0;) {
			int step = LINE_CYC / SLICES;
			if (step > left)
				step = left;
			left -= step;
			s_mdebt += step;
			if (s_mdebt > 0) {
				s_slices++;
				s_mdebt -= m68k_execute((int)s_mdebt);
			}
		}
		const int n = rbx_ym_advance(0, YM_LINE_CLK, s_mix + s_mix_n, s_mix_cap - s_mix_n);
		for (int i = 0; i < n; i++) {
			s_oki_acc += s_oki.rate;
			while (s_oki_acc >= yr) {
				s_oki_acc -= yr;
				s_oki_hold = oki_sample(&s_oki);
			}
			s_mix[s_mix_n + i] += s_oki_hold;
		}
		s_mix_n += n;
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

// ---- init / reset ----
static void rp_reset(void)
{
	memset(s_bitmap, 0, 0x20000);
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_pal, 0, sizeof(s_pal));
	s_mo.bank = 0;
	amo_set_xscroll(&s_mo, -12);   // video_start: the initial scroll offset
	amo_set_yscroll(&s_mo, 0);
	s_eep_unlocked = 0;
	slapstic_reset(&s_slap);
	m68k_bus_irq_reset();
	oki_reset(&s_oki);
	s_oki.gain = 154;
	rbx_ym_reset(0);
	rbx_ym_set_gain(0, 0, 256);
	rbx_ym_set_gain(0, 1, 256);
	s_latch = 0xffff;
	s_oki_acc = 0;
	s_oki_hold = 0;
	s_mix_n = 0;
	s_mdebt = 0;
	s_line = 0;
	m68k_pulse_reset();
}

static void rp_exit(void);

static int rp_init(const rbx_game *g)
{
	uint32_t gs, os, es;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *gfx = rbx_region("gfx", &gs), *oki = rbx_region("oki", &os), *eep = rbx_region("eeprom", &es);
	if (!s_rom || s_rom_size < 0x100000 || !gfx || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	{   // gfx_8x8x4_packed_msb
		static const uint32_t pp[4] = { 0, 1, 2, 3 };
		static const uint32_t px[8] = { 0, 4, 8, 12, 16, 20, 24, 28 };
		static const uint32_t py[8] = { 0, 32, 64, 96, 128, 160, 192, 224 };
		const rbx_gfx_layout l = { 8, 8, 4, pp, px, py, 256 };
		if (rbx_gfx_decode(&s_gfx, &l, gfx, gs)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			return -1;
		}
	}
	static const amo_config moc = {
		1, 1, 0, 0, 0, 0,   // banks, linked, split, reverse, swapxy, nextneighbor
		8, 0, 0, 0x100, 0,  // slipheight, slipoffset, maxperline, palettebase, transpen
		{ 0x00ff, 0, 0, 0 }, { 0, 0x7fff, 0, 0 }, { 0, 0, 0x000f, 0 }, { 0, 0, 0xff80, 0 }, { 0, 0, 0, 0xff80 },
		{ 0, 0, 0, 0x0070 }, { 0, 0, 0, 0x0007 }, { 0, 0x8000, 0, 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, 0
	};
	if (amo_init(&s_mo, &moc, 8, 8) || slapstic_init(&s_slap, 118, 0x140000, 0x147fff, 0x438000, 16)) {
		rbx_fail("%s: out of memory", g->set);
		rp_exit();
		return -1;
	}
	s_mo.gfx[0] = &s_gfx;
	s_mo.granularity[0] = 16;
	s_bitmap = (uint16_t *)calloc(0x10000, sizeof(uint16_t));
	s_frame = (uint16_t *)calloc(VIS_W * VIS_H, sizeof(uint16_t));
	s_mobm = (uint16_t *)calloc(BM_W * VIS_H, sizeof(uint16_t));
	s_snap = calloc(1, sizeof(*s_snap));
	oki_init(&s_oki, 14318181u / 4 / 3, 0, oki, os, 154);
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2413;
	yc.clock = 14318181u / 4;
	yc.gain[0] = yc.gain[1] = 256;
	int rc = !s_bitmap || !s_frame || !s_mobm || !s_snap || rbx_ym_init(0, &yc);
	if (!rc) {
		s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		rc = !s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 0x200);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		rp_exit();
		return -1;
	}
	static const char *const names[7] = { "IN0", "IN1", "P3", "TRACK0", "TRACK1", "TRACK2", "TRACK3" };
	for (int i = 0; i < 7; i++)
		s_port[i] = rbx_port_find(names[i]);
	// the EEPROM: the region's data (or blank), then the saved file
	memset(s_eeprom, 0xff, sizeof(s_eeprom));
	if (eep && es >= sizeof(s_eeprom))
		memcpy(s_eeprom, eep, sizeof(s_eeprom));
	s_nv_path[0] = 0;
	{
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			FILE *f = fopen(s_nv_path, "rb");
			if (f) {
				uint8_t tmp[sizeof(s_eeprom)];
				if (fread(tmp, 1, sizeof(tmp), f) == sizeof(tmp))
					memcpy(s_eeprom, tmp, sizeof(tmp));
				fclose(f);
			}
		}
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);
	map_main();
	rp_reset();
	return 0;
}

static void rp_exit(void)
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
	rbx_ym_exit(0);
	amo_exit(&s_mo);
	rbx_gfx_free(&s_gfx);
	free(s_bitmap);
	free(s_frame);
	free(s_mobm);
	free(s_snap);
	free(s_mix);
	s_bitmap = s_frame = s_mobm = NULL;
	s_snap = NULL;
	s_mix = NULL;
}

// ---- save state ----
static int rp_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	int32_t regs[6] = { s_eep_unlocked, (int32_t)s_mdebt, (int32_t)s_oki_acc, s_oki_hold, s_latch, s_oki.gain };
	uint8_t slap[3] = { s_slap.state, s_slap.current_bank, s_slap.loaded_bank };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "bitmap", s_bitmap, 0x20000);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "eeprom", s_eeprom, sizeof(s_eeprom));
	rc |= rbx_state_io(ctx, "regs", regs, sizeof(regs));
	rc |= rbx_state_io(ctx, "slap", slap, sizeof(slap));
	rc |= rbx_state_io(ctx, "oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	rc |= rbx_ym_state(0, ctx, load);
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		s_eep_unlocked = regs[0];
		s_mdebt = regs[1];
		s_oki_acc = (uint32_t)regs[2];
		s_oki_hold = regs[3];
		s_latch = (uint16_t)regs[4];
		s_oki.gain = regs[5];
		const int g = s_latch == 0xffff ? 256 : (int)(((s_latch >> 1) & 7) * 256 / 7);
		rbx_ym_set_gain(0, 0, g);
		rbx_ym_set_gain(0, 1, g);
		s_slap.state = slap[0];
		s_slap.current_bank = slap[1];
		s_slap.loaded_bank = slap[2];
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int rp_save(void *ctx) { return rp_state(ctx, 0); }
static int rp_load(void *ctx) { return rp_state(ctx, 1); }
static int rp_supported(const char *machine) { return !strcmp(machine, "rampart"); }

const rbx_driver rbx_drv_rampart = { "Atari Rampart", rp_init, rp_exit, rp_reset, rp_frame, rp_save, rp_load,
                                     rp_supported, rp_render, NULL };
