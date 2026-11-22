/*  Atari Blasteroids hardware. Ported from MAME src/mame/atari/blstroid.cpp and atarigen.cpp (halt_until_hblank_0)
    (BSD-3-Clause, copyright-holders: Aaron Giles).
    68000 at 7.16 MHz; a 64x32 playfield of 16x8 tiles (8x8 doubled), the Atari motion objects (atarimo.c, 16x8, linked)
    through a priority RAM; a 2804 parallel EEPROM. Sound: the JSA I without its POKEY and TMS5220 (atarijsa.c).
    IRQ 2 at the VBLANK, IRQ 4 from the sound response (MAME keeps the scanline IRQ 1 off).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../video/gfxlayout.h"
#include "../video/atarimo.h"
#include "../machine/atarijsa.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LINE_CYC 456               // set_raw(14.318181 MHz, 456*2, 0, 320*2, 262, 0, 240): in 68000 clocks
#define HBLANK_CYC 320             // the visible 640 pixels
#define HALT_CYC 410               // halt_until_hblank_0: width * 9 / 10
#define LINES 262
#define VIS_W 640
#define VIS_H 240
#define FRAME_MHZ 59923
#define SLICES 4

static uint8_t *s_rom;
static uint32_t s_rom_size;
static rbx_gfx s_gfx_pf, s_gfx_mo;
static amo_t s_mo;

// main memory (host-order words)
static uint16_t s_ram[0x2000];       // 804000-807fff: playfield, MO at 0x1000, RAM
static uint16_t s_pal[0x200];        // 802000-8023ff
static uint16_t s_prio[0x100];       // 800800-8009ff
static uint8_t s_eeprom[0x200];
static int s_eep_unlocked;
static char s_nv_path[512];
static int s_port[5];
static int s_line;
static int s_slice_pos;              // the line position (68000 clocks) where the running m68k_execute started
static int s_halt;                   // clocks the 68000 is halted for (halt_until_hblank_0)
static int64_t s_mdebt;
static int s_sndacc;

#define PF (s_ram)
#define MOB (s_ram + 0x1000 / 2)

// video
static uint16_t *s_frame;            // VIS_W x VIS_H
static uint16_t *s_mobm;             // the MO bitmap
static struct { uint16_t frame[VIS_W * VIS_H]; uint32_t pal[0x200]; } *s_snap;

static inline uint32_t xrgb_555(uint16_t raw)
{
	const uint32_t r = (raw >> 10) & 31, g = (raw >> 5) & 31, b = raw & 31;
	return (r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (b << 3 | b >> 2);
}

static void draw_frame(void)   // screen_update
{
	for (int y = 0; y < VIS_H; y++) {   // the playfield (64x32 tiles of 16x8, by rows)
		uint16_t *d = s_frame + y * VIS_W;
		const int row = y >> 3, py = y & 7;
		for (int col = 0; col < VIS_W / 16; col++) {
			const uint16_t data = PF[row * 64 + col];
			const uint32_t code = rbx_wrap(data & 0x1fff, s_gfx_pf.count);
			const uint16_t color = (uint16_t)(256 + ((data >> 13) & 7) * 16);
			const uint8_t *src = s_gfx_pf.pix + code * 64 + py * 8;
			uint16_t *dd = d + col * 16;
			for (int k = 0; k < 8; k++)
				dd[2 * k] = dd[2 * k + 1] = (uint16_t)(color + src[k]);
		}
	}
	amo_draw(&s_mo, MOB, NULL, s_mobm, VIS_W, 0, VIS_W - 1, 0, VIS_H - 1);
	for (int i = 0; i < VIS_W * VIS_H; i++) {
		const uint16_t mo = s_mobm[i];
		if (mo == 0xffff)
			continue;
		const uint16_t pf = s_frame[i];   // priority address = HPPPMMMM
		const int priaddr = ((pf & 8) << 4) | (pf & 0x70) | ((mo & 0xf0) >> 4);
		if (s_prio[priaddr] & 1)
			s_frame[i] = mo;
	}
}

static int line_pos(void) { return s_slice_pos + m68k_cycles_run(); }

// ---- main bus ----
static void main_irq4(int state) { m68k_bus_set_line(4, state); }

static uint16_t in_r(int n)
{
	uint16_t v = (uint16_t)(rbx_port_read(s_port[2 + n]) & ~0x70u);
	if (line_pos() >= HBLANK_CYC) v |= 0x10;   // HBLANK
	if (s_line >= VIS_H) v |= 0x20;            // VBLANK
	if (!jsa_main_to_sound_ready()) v |= 0x40;  // (custom, active low)
	return v;
}

static uint16_t main_read16(uint32_t a)
{
	a &= 0x83ffff;
	if (!(a & 0x800000))
		return a < 0x40000 ? *(const uint16_t *)(s_rom + a) : 0xffff;
	const uint32_t o = a & 0x7fff;
	if (o >= 0x4000) return s_ram[(o - 0x4000) >> 1];
	if (o >= 0x3000) return (uint16_t)(0xff00 | s_eeprom[(o & 0x3ff) >> 1]);
	if (o >= 0x2000) return s_pal[(o & 0x3ff) >> 1];
	switch (o & 0x1c00) {
	case 0x1400: return (uint16_t)(0xff00 | jsa_main_response_r());
	case 0x1800: return (uint16_t)rbx_port_read(s_port[(o >> 2) & 1]);
	case 0x1c00: return in_r((o >> 1) & 1);
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
	a &= 0x83ffff;
	if (!(a & 0x800000))
		return;
	const uint32_t o = a & 0x7fff;
	if (o >= 0x4000) {
		uint16_t *p = &s_ram[(o - 0x4000) >> 1];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
		return;
	}
	if (o >= 0x3000) {
		if ((mask & 0x00ff) && s_eep_unlocked) {
			s_eeprom[(o & 0x3ff) >> 1] = (uint8_t)v;
			s_eep_unlocked = 0;   // (lock_after_write)
		}
		return;
	}
	if (o >= 0x2000) {
		uint16_t *p = &s_pal[(o & 0x3ff) >> 1];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
		return;
	}
	if (o >= 0x1000)
		return;
	switch (o & 0xe00) {
	case 0x200: m68k_bus_set_line(1, 0); break;   // scanline_int_ack_w
	case 0x400: m68k_bus_set_line(2, 0); break;   // video_int_ack_w
	case 0x600: s_eep_unlocked = 1; break;
	case 0x800: {
		uint16_t *p = &s_prio[(o & 0x1ff) >> 1];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
		break;
	}
	case 0xa00:
		if (mask & 0x00ff) {
			jsa_main_command_w((uint8_t)v);
			m68k_end_timeslice();
		}
		break;
	case 0xc00: jsa_sound_reset_w(); m68k_end_timeslice(); break;
	case 0xe00: {   // halt_until_hblank_0
		const int pos = line_pos();
		s_halt = (pos >= HALT_CYC ? HALT_CYC + LINE_CYC : HALT_CYC) - pos;
		m68k_end_timeslice();
		break;
	}
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
	m68k_bus_map(0x000000, 0x03ffff, s_rom, 0x40000, 1, 0);
	m68k_bus_map(0x804000, 0x807fff, s_ram, sizeof(s_ram), 1, 1);
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
		s_snap->pal[i] = xrgb_555(s_pal[i]);
	rbx_video_frame_done();
}

static void bl_render(void)
{
	for (int y = 0; y < VIS_H; y++)
		memcpy(rbx_video.bitmap + y * rbx_video.pitch, s_snap->frame + y * VIS_W, VIS_W * 2);
	memcpy(rbx_video.palette, s_snap->pal, sizeof(s_snap->pal));
}

static void bl_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	jsa_set_test((rbx_port_read(s_port[2]) >> 7) & 1);
	jsa_frame_begin();
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_H) {   // VIDEO_UPDATE_BEFORE_VBLANK, then the VBLANK IRQ
			draw_frame();
			capture();
			m68k_bus_set_line(2, 1);
		}
		for (int left = LINE_CYC; left > 0;) {
			int step = jsa_boosting() ? 36 : LINE_CYC / SLICES;
			if (step > left)
				step = left;
			s_slice_pos = LINE_CYC - left - (int)s_mdebt;   // (the 68000 is ahead of the slice by its overrun)
			left -= step;
			s_mdebt += step;
			if (s_mdebt > 0) {
				if (s_halt > 0) {   // (halted: the clocks pass)
					const int h = s_halt < s_mdebt ? s_halt : (int)s_mdebt;
					s_halt -= h;
					s_mdebt -= h;
					s_slice_pos += h;
				}
				if (s_mdebt > 0) {
					s_mdebt -= m68k_execute((int)s_mdebt);
					if (s_halt > 0) {
						const int h = s_halt < s_mdebt ? s_halt : (s_mdebt > 0 ? (int)s_mdebt : 0);
						s_halt -= h;
						s_mdebt -= h;
					}
				}
			}
			s_sndacc += step;   // (the 6502 at a quarter of the 68000's clock)
			jsa_run(s_sndacc >> 2);
			s_sndacc &= 3;
		}
		jsa_line();
	}
	jsa_frame_end();
}

// ---- init / reset ----
static void bl_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_prio, 0, sizeof(s_prio));
	s_mo.bank = 0;
	amo_set_xscroll(&s_mo, 0);
	amo_set_yscroll(&s_mo, 0);
	s_eep_unlocked = 0;
	m68k_bus_irq_reset();
	jsa_reset();
	s_mdebt = 0;
	s_sndacc = 0;
	s_halt = 0;
	s_slice_pos = 0;
	s_line = 0;
	m68k_pulse_reset();
}

static void bl_exit(void);

static int bl_init(const rbx_game *g)
{
	uint32_t ts, ss, js;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *tiles = rbx_region("tiles", &ts), *spr = rbx_region("sprites", &ss), *jrom = rbx_region("jsa:cpu", &js);
	if (!s_rom || s_rom_size < 0x40000 || !tiles || !spr || !jrom || js < 0x10000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	{   // gfx_8x8x4_packed_msb (shown doubled), molayout
		static const uint32_t pp[4] = { 0, 1, 2, 3 };
		static const uint32_t px[8] = { 0, 4, 8, 12, 16, 20, 24, 28 };
		static const uint32_t py[8] = { 0, 32, 64, 96, 128, 160, 192, 224 };
		const rbx_gfx_layout l = { 8, 8, 4, pp, px, py, 256 };
		const uint32_t h = ss / 2 * 8;
		const uint32_t mx[16] = { h + 0, h + 4, 0, 4, h + 8, h + 12, 8, 12, h + 16, h + 20, 16, 20, h + 24, h + 28, 24, 28 };
		const rbx_gfx_layout lm = { 16, 8, 4, pp, mx, py, 256 };
		if (rbx_gfx_decode(&s_gfx_pf, &l, tiles, ts) || rbx_gfx_decode(&s_gfx_mo, &lm, spr, ss / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			bl_exit();
			return -1;
		}
	}
	static const amo_config moc = {
		1, 1, 0, 0, 0, 0,   // banks, linked, split, reverse, swapxy, nextneighbor
		0, 0, 0, 0x000, 0,  // slipheight, slipoffset, maxperline, palettebase, transpen
		{ 0, 0, 0x0ff8, 0 }, { 0, 0x3fff, 0, 0 }, { 0, 0, 0, 0x000f }, { 0, 0, 0, 0xffc0 }, { 0xff80, 0, 0, 0 },
		{ 0 }, { 0x000f, 0, 0, 0 }, { 0, 0x8000, 0, 0 }, { 0, 0x4000, 0, 0 }, { 0 }, { 0 }, { 0 }, { 0 }, 0
	};
	if (amo_init(&s_mo, &moc, 16, 8)) {
		rbx_fail("%s: out of memory", g->set);
		bl_exit();
		return -1;
	}
	s_mo.gfx[0] = &s_gfx_mo;
	s_mo.granularity[0] = 16;
	s_frame = (uint16_t *)calloc(VIS_W * VIS_H, sizeof(uint16_t));
	s_mobm = (uint16_t *)calloc(VIS_W * VIS_H, sizeof(uint16_t));
	s_snap = calloc(1, sizeof(*s_snap));
	static const char *const names[5] = { "DIAL0", "DIAL1", "IN0", "IN1", "JSA" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	jsa_config jc;
	memset(&jc, 0, sizeof(jc));
	jc.type = 1;
	jc.out = JSA_OUT_STEREO;
	jc.gain = 256;
	jc.rom = jrom;
	jc.port = s_port[4];
	jc.main_int = main_irq4;
	if (!s_frame || !s_mobm || !s_snap || jsa_init(&jc) || rbx_video_setup(VIS_W, VIS_H, 0x200)) {
		rbx_fail("%s: out of memory", g->set);
		bl_exit();
		return -1;
	}
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
	bl_reset();
	return 0;
}

static void bl_exit(void)
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
	jsa_exit();
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
static int bl_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	int32_t regs[4] = { s_eep_unlocked, (int32_t)s_mdebt, s_sndacc, s_halt };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "prio", s_prio, sizeof(s_prio));
	rc |= rbx_state_io(ctx, "eeprom", s_eeprom, sizeof(s_eeprom));
	rc |= rbx_state_io(ctx, "regs", regs, sizeof(regs));
	rc |= jsa_state(ctx, load);
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		s_eep_unlocked = regs[0];
		s_mdebt = regs[1];
		s_sndacc = regs[2];
		s_halt = regs[3];
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int bl_save(void *ctx) { return bl_state(ctx, 0); }
static int bl_load(void *ctx) { return bl_state(ctx, 1); }
static int bl_supported(const char *machine) { return !strcmp(machine, "blstroid"); }

const rbx_driver rbx_drv_blstroid = { "Atari Blasteroids", bl_init, bl_exit, bl_reset, bl_frame, bl_save, bl_load,
                                      bl_supported, bl_render, NULL };
