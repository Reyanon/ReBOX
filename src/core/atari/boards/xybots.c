/*  Atari Xybots hardware. Ported from MAME src/mame/atari/xybots.cpp (BSD-3-Clause, copyright-holders: Aaron Giles).
    68000 at 7.16 MHz; a 64x32 playfield of 8x8 tiles, the Atari motion objects (atarimo.c, not linked), a 64x32
    alphanumerics layer; the slapstic 107 on 0x8000-0xffff; a 2804 parallel EEPROM. Sound: the JSA I without its POKEY
    (atarijsa.c), the outputs swapped. IRQ 1 at the VBLANK, IRQ 2 from the sound response.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../video/gfxlayout.h"
#include "../video/atarimo.h"
#include "../machine/slapstic.h"
#include "../machine/atarijsa.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LINE_CYC 456               // set_raw(14.318181 MHz / 2, 456, 0, 336, 262, 0, 240)
#define LINES 262
#define VIS_W 336
#define VIS_H 240
#define FRAME_MHZ 59923
#define SLICES 4

static uint8_t *s_rom;
static uint32_t s_rom_size;
static rbx_gfx s_gfx_pf, s_gfx_mo, s_gfx_an;
static amo_t s_mo;
static slapstic_t s_slap;

// main memory (host-order words)
static uint16_t s_ram[0x2000];       // 800000-803fff: alpha, RAM, MO at 0x2e00, playfield at 0x3000
static uint16_t s_pal[0x400];        // 804000-8047ff
static uint8_t s_eeprom[0x200];
static int s_eep_unlocked;
static char s_nv_path[512];
static int s_port[3];
static int s_line;
static int64_t s_mdebt;
static int s_sndacc;
static uint16_t s_h256;

#define ALPHA (s_ram)
#define MOB (s_ram + 0x2e00 / 2)
#define PF (s_ram + 0x3000 / 2)

// video
static uint16_t *s_frame;            // VIS_W x VIS_H
static uint16_t *s_mobm;             // the MO bitmap
static struct { uint16_t frame[VIS_W * VIS_H]; uint32_t pal[0x400]; } *s_snap;

static inline uint32_t irgb_4444(uint16_t raw)   // IRGB_4444 (standard_irgb_decoder<4,4,4,4, 12,8,4,0>)
{
	const uint32_t i = ((raw >> 12) & 15) * 0x11;
	const uint32_t r = (i * (((raw >> 8) & 15) * 0x11)) >> 8;
	const uint32_t g = (i * (((raw >> 4) & 15) * 0x11)) >> 8;
	const uint32_t b = (i * ((raw & 15) * 0x11)) >> 8;
	return r << 16 | g << 8 | b;
}

static void draw_frame(void)   // screen_update
{
	for (int y = 0; y < VIS_H; y++) {   // the playfield (64x32, by rows)
		uint16_t *d = s_frame + y * VIS_W;
		const int row = y >> 3, py = y & 7;
		for (int col = 0; col < VIS_W / 8; col++) {
			const uint16_t data = PF[row * 64 + col];
			const uint32_t code = rbx_wrap(data & 0x1fff, s_gfx_pf.count);
			const uint16_t color = (uint16_t)(512 + ((data >> 11) & 0x0f) * 16);
			const uint8_t *src = s_gfx_pf.pix + code * 64 + py * 8;
			uint16_t *dd = d + col * 8;
			if (data & 0x8000)
				for (int k = 0; k < 8; k++) dd[k] = (uint16_t)(color + src[7 - k]);
			else
				for (int k = 0; k < 8; k++) dd[k] = (uint16_t)(color + src[k]);
		}
	}
	amo_draw(&s_mo, MOB, NULL, s_mobm, VIS_W, 0, VIS_W - 1, 0, VIS_H - 1);
	for (int i = 0; i < VIS_W * VIS_H; i++) {
		const uint16_t mo = s_mobm[i];
		if (mo == 0xffff)
			continue;
		const int mopriority = (mo >> AMO_PRIORITY_SHIFT) ^ 15;
		const int pfcolor = (s_frame[i] >> 4) & 0x0f;
		if ((mo & 0x0f) > 1) {   // PRIEN
			if (mopriority <= pfcolor)
				s_frame[i] = (mo & 0x80) ? (uint16_t)((mo ^ 0x2f0) & 0x0fff) : (uint16_t)(mo & 0x0fff);
		} else if (mopriority < pfcolor)
			s_frame[i] = (uint16_t)(mo & 0x0fff);
	}
	for (int y = 0; y < VIS_H; y++) {   // the alphanumerics (64x32, by rows; pen 0 transparent unless opaque)
		uint16_t *d = s_frame + y * VIS_W;
		const int row = y >> 3, py = y & 7;
		for (int col = 0; col < VIS_W / 8; col++) {
			const uint16_t data = ALPHA[row * 64 + col];
			const uint32_t code = rbx_wrap(data & 0x3ff, s_gfx_an.count);
			const int opaque = data & 0x8000;
			if (!opaque && !s_gfx_an.used[code])
				continue;
			const uint16_t color = (uint16_t)(((data >> 12) & 7) * 4);
			const uint8_t *src = s_gfx_an.pix + code * 64 + py * 8;
			uint16_t *dd = d + col * 8;
			for (int k = 0; k < 8; k++)
				if (src[k] || opaque)
					dd[k] = (uint16_t)(color + src[k]);
		}
	}
}

// ---- main bus ----
static uint32_t s_slices;
static void slap_tap(uint32_t a)
{
	const uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC) & 0xffffff;
	const uint32_t inst = s_slices * 4096u + (uint32_t)m68k_cycles_remaining();
	slapstic_tap(&s_slap, a & 0xffffff, pc, !(pc & 0x800000) && (pc & 0x38000) == 0x8000, inst);
}

static void main_irq2(int state) { m68k_bus_set_line(2, state); }

static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (!(a & 0x800000)) {
		const uint32_t o = a & 0x3ffff;
		if ((o & 0x38000) == 0x8000) {   // the slapstic's bank (mirrored by 0x7c6000)
			const uint16_t v = *(const uint16_t *)(s_rom + 0x8000 + s_slap.current_bank * 0x2000 + (o & 0x1ffe));
			slap_tap(a);
			return v;
		}
		return *(const uint16_t *)(s_rom + o);
	}
	const uint32_t o = a & 0x7fff;
	if (o < 0x4000) return s_ram[o >> 1];
	if (o < 0x5000) return s_pal[(o & 0x7ff) >> 1];
	if (o < 0x6000) return (uint16_t)(0xff00 | s_eeprom[(o & 0x3ff) >> 1]);
	switch (o & 0x7f00) {
	case 0x6000: return (uint16_t)(0xff00 | jsa_main_response_r());
	case 0x6100: return (uint16_t)rbx_port_read(s_port[0]);
	case 0x6200: {   // special_port1_r
		uint16_t v = (uint16_t)(rbx_port_read(s_port[1]) & ~0x0e00u);
		if (!jsa_main_to_sound_ready()) v |= 0x200;   // /AUDBUSY (custom, active low)
		if (s_line >= VIS_H) v |= 0x800;              // VBLANK
		v ^= (uint16_t)(s_h256 ^ 0x0400);
		s_h256 ^= 0x0400;
		return v;
	}
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
	a &= 0xffffff;
	if (!(a & 0x800000)) {
		if ((a & 0x38000) == 0x8000)
			slap_tap(a);
		return;
	}
	const uint32_t o = a & 0x7fff;
	if (o < 0x4000) {
		uint16_t *p = &s_ram[o >> 1];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
		return;
	}
	if (o < 0x5000) {
		uint16_t *p = &s_pal[(o & 0x7ff) >> 1];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
		return;
	}
	if (o < 0x6000) {
		if ((mask & 0x00ff) && s_eep_unlocked) {
			s_eeprom[(o & 0x3ff) >> 1] = (uint8_t)v;
			s_eep_unlocked = 0;   // (lock_after_write)
		}
		return;
	}
	switch (o & 0x7f00) {
	case 0x6800: s_eep_unlocked = 1; break;
	case 0x6900:
		if (mask & 0x00ff) {
			jsa_main_command_w((uint8_t)v);
			m68k_end_timeslice();
		}
		break;
	case 0x6b00: m68k_bus_set_line(1, 0); break;   // video_int_ack_w
	case 0x6e00: jsa_sound_reset_w(); m68k_end_timeslice(); break;
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
	m68k_bus_map(0x000000, 0x007fff, s_rom, 0x8000, 1, 0);
	m68k_bus_map(0x010000, 0x03ffff, s_rom + 0x10000, 0x30000, 1, 0);
	m68k_bus_map(0x800000, 0x803fff, s_ram, sizeof(s_ram), 1, 1);
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
	for (int i = 0; i < 0x400; i++)
		s_snap->pal[i] = irgb_4444(s_pal[i]);
	rbx_video_frame_done();
}

static void xb_render(void)
{
	for (int y = 0; y < VIS_H; y++)
		memcpy(rbx_video.bitmap + y * rbx_video.pitch, s_snap->frame + y * VIS_W, VIS_W * 2);
	memcpy(rbx_video.palette, s_snap->pal, sizeof(s_snap->pal));
}

static void xb_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	jsa_set_test((rbx_port_read(s_port[1]) >> 8) & 1);
	jsa_frame_begin();
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_H) {   // VIDEO_UPDATE_BEFORE_VBLANK, then the VBLANK IRQ
			draw_frame();
			capture();
			m68k_bus_set_line(1, 1);
		}
		// the CPUs in slices; after a sound command (the comm's quantum of 5 us) in steps of 36 clocks
		for (int left = LINE_CYC; left > 0;) {
			int step = jsa_boosting() ? 36 : LINE_CYC / SLICES;
			if (step > left)
				step = left;
			left -= step;
			s_mdebt += step;
			if (s_mdebt > 0) {
				s_slices++;
				s_mdebt -= m68k_execute((int)s_mdebt);
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
static void xb_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_pal, 0, sizeof(s_pal));
	s_mo.bank = 0;
	amo_set_xscroll(&s_mo, 0);
	amo_set_yscroll(&s_mo, 0);
	s_eep_unlocked = 0;
	s_h256 = 0x0400;
	slapstic_reset(&s_slap);
	m68k_bus_irq_reset();
	jsa_reset();
	s_mdebt = 0;
	s_sndacc = 0;
	s_line = 0;
	m68k_pulse_reset();
}

static void xb_exit(void);

static int xb_init(const rbx_game *g)
{
	uint32_t ts, ss, cs, js;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *tiles = rbx_region("tiles", &ts), *spr = rbx_region("sprites", &ss), *chars = rbx_region("chars", &cs);
	uint8_t *jrom = rbx_region("jsa:cpu", &js);
	if (!s_rom || s_rom_size < 0x40000 || !tiles || !spr || !chars || !jrom || js < 0x10000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	{   // gfx_8x8x4_packed_msb, anlayout
		static const uint32_t pp[4] = { 0, 1, 2, 3 };
		static const uint32_t px[8] = { 0, 4, 8, 12, 16, 20, 24, 28 };
		static const uint32_t py[8] = { 0, 32, 64, 96, 128, 160, 192, 224 };
		static const uint32_t ap[2] = { 0, 4 }, ax[8] = { 0, 1, 2, 3, 8, 9, 10, 11 };
		static const uint32_t ay[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
		const rbx_gfx_layout l = { 8, 8, 4, pp, px, py, 256 };
		const rbx_gfx_layout la = { 8, 8, 2, ap, ax, ay, 128 };
		if (rbx_gfx_decode(&s_gfx_pf, &l, tiles, ts) || rbx_gfx_decode(&s_gfx_mo, &l, spr, ss) ||
		    rbx_gfx_decode(&s_gfx_an, &la, chars, cs)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			xb_exit();
			return -1;
		}
	}
	static const amo_config moc = {
		1, 0, 0, 0, 0, 0,   // banks, linked, split, reverse, swapxy, nextneighbor
		0, 0, 0, 0x100, 0,  // slipheight, slipoffset, maxperline, palettebase, transpen
		{ 0x3f, 0, 0, 0 }, { 0x3fff, 0, 0, 0 }, { 0, 0, 0, 0x000f }, { 0, 0, 0, 0xff80 }, { 0, 0, 0xff80, 0 },
		{ 0 }, { 0, 0, 0x0007, 0 }, { 0x8000, 0, 0, 0 }, { 0 }, { 0, 0x000f, 0, 0 }, { 0 }, { 0 }, { 0 }, 0
	};
	if (amo_init(&s_mo, &moc, 8, 8) || slapstic_init(&s_slap, 107, 0x8000, 0xffff, 0x7c0000, 16)) {
		rbx_fail("%s: out of memory", g->set);
		xb_exit();
		return -1;
	}
	s_mo.gfx[0] = &s_gfx_mo;
	s_mo.granularity[0] = 16;
	s_frame = (uint16_t *)calloc(VIS_W * VIS_H, sizeof(uint16_t));
	s_mobm = (uint16_t *)calloc(VIS_W * VIS_H, sizeof(uint16_t));
	s_snap = calloc(1, sizeof(*s_snap));
	static const char *const names[3] = { "FFE100", "FFE200", "JSA" };
	for (int i = 0; i < 3; i++)
		s_port[i] = rbx_port_find(names[i]);
	jsa_config jc;
	memset(&jc, 0, sizeof(jc));
	jc.type = 1;
	jc.tms = 1;
	jc.out = JSA_OUT_SWAPPED;
	jc.gain = 256;
	jc.rom = jrom;
	jc.port = s_port[2];
	jc.main_int = main_irq2;
	if (!s_frame || !s_mobm || !s_snap || jsa_init(&jc) || rbx_video_setup(VIS_W, VIS_H, 0x400)) {
		rbx_fail("%s: out of memory", g->set);
		xb_exit();
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
	xb_reset();
	return 0;
}

static void xb_exit(void)
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
	rbx_gfx_free(&s_gfx_an);
	free(s_frame);
	free(s_mobm);
	free(s_snap);
	s_frame = s_mobm = NULL;
	s_snap = NULL;
}

// ---- save state ----
static int xb_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	int32_t regs[4] = { s_eep_unlocked, (int32_t)s_mdebt, s_sndacc, s_h256 };
	uint8_t slap[3] = { s_slap.state, s_slap.current_bank, s_slap.loaded_bank };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "eeprom", s_eeprom, sizeof(s_eeprom));
	rc |= rbx_state_io(ctx, "regs", regs, sizeof(regs));
	rc |= rbx_state_io(ctx, "slap", slap, sizeof(slap));
	rc |= jsa_state(ctx, load);
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		s_eep_unlocked = regs[0];
		s_mdebt = regs[1];
		s_sndacc = regs[2];
		s_h256 = (uint16_t)regs[3];
		s_slap.state = slap[0];
		s_slap.current_bank = slap[1];
		s_slap.loaded_bank = slap[2];
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int xb_save(void *ctx) { return xb_state(ctx, 0); }
static int xb_load(void *ctx) { return xb_state(ctx, 1); }
static int xb_supported(const char *machine) { return !strcmp(machine, "xybots"); }

const rbx_driver rbx_drv_xybots = { "Atari Xybots", xb_init, xb_exit, xb_reset, xb_frame, xb_save, xb_load,
                                    xb_supported, xb_render, NULL };
