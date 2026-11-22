/*  Atari Cyberball 2072 (2 player) hardware. Ported from MAME src/mame/atari/cyberbal.cpp (BSD-3-Clause,
    copyright-holders: Aaron Giles).
    68000 at 7.16 MHz; a 64x64 playfield of 16x8 tiles (8x8 shown doubled), the Atari motion objects (atarimo.c, linked,
    one SLIP for the screen), a 64x32 alphanumerics layer of 16x8 tiles whose last columns hold the per-band palette
    bank, scrolls and SLIP (scanline_update every 8 lines); a 2816 parallel EEPROM. Sound: the JSA II (atarijsa.c).
    IRQ 1 at the VBLANK, IRQ 3 from the sound response.
    The cyberbal machine (two screens, the second 68000 and the SAC sound board) is not here yet.
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

#define LINE_CYC 456               // set_raw(14.318181 MHz, 456*2, 0, 336*2, 262, 0, 240): in 68000 clocks
#define LINES 262
#define VIS_W 672
#define VIS_H 240
#define FRAME_MHZ 59923
#define SLICES 4

static uint8_t *s_rom;
static uint32_t s_rom_size;
static rbx_gfx s_gfx_pf, s_gfx_mo, s_gfx_an;
static amo_t s_mo;

// main memory (host-order words)
static uint16_t s_vram[0x8000];      // ff0000-ffffff: playfield, alpha at 0x2000, MO at 0x3000, RAM from 0x3800
static uint16_t s_pal[0x800];        // fca000-fcafff
static uint8_t s_eeprom[0x800];
static int s_eep_unlocked;
static char s_nv_path[512];
static int s_port[4];
static int s_line, s_last_drawn;
static int64_t s_mdebt;
static int s_sndacc;
static uint16_t s_slip;
static int s_pf_bank, s_pf_xscroll, s_pf_yscroll;

#define PF (s_vram)
#define ALPHA (s_vram + 0x2000 / 2)
#define MOB (s_vram + 0x3000 / 2)

// video
static uint16_t *s_frame;            // VIS_W x VIS_H
static uint16_t *s_mobm;             // the MO bitmap
static struct { uint16_t frame[VIS_W * VIS_H]; uint32_t pal[0x800]; } *s_snap;

static inline uint8_t pal6bit(uint8_t x) { return (uint8_t)((x << 2) | (x >> 4)); }

static inline uint32_t irgb_1555(uint16_t raw)   // IRRRRRGGGGGBBBBB_decoder
{
	const uint8_t i = (raw >> 15) & 1;
	const uint32_t r = pal6bit((uint8_t)(((raw >> 9) & 0x3e) | i));
	const uint32_t g = pal6bit((uint8_t)(((raw >> 4) & 0x3e) | i));
	const uint32_t b = pal6bit((uint8_t)(((raw << 1) & 0x3e) | i));
	return r << 16 | g << 8 | b;
}

static void draw_lines(int y0, int y1)   // update_one_screen on [y0, y1]
{
	const uint16_t bank = (uint16_t)(s_pf_bank << 8);
	for (int y = y0; y <= y1; y++) {   // the playfield (64x64 tiles of 16x8, by rows)
		uint16_t *d = s_frame + y * VIS_W;
		const int ty = (y + s_pf_yscroll) & 511;
		const int row = ty >> 3, py = ty & 7;
		int x = 0, tx = s_pf_xscroll & 1023;
		while (x < VIS_W) {
			const uint16_t data = PF[row * 64 + (tx >> 4)];
			const uint32_t code = rbx_wrap(data & 0x1fff, s_gfx_pf.count);
			const uint16_t color = (uint16_t)(bank + ((data >> 11) & 0x0f) * 16);
			const uint8_t *src = s_gfx_pf.pix + code * 64 + py * 8;
			const int flip = data >> 15;
			for (int px = tx & 15; px < 16 && x < VIS_W; px++, x++, tx++)
				d[x] = (uint16_t)(color + src[flip ? 7 - (px >> 1) : px >> 1]);
			tx &= 1023;
		}
	}
	amo_draw(&s_mo, MOB, &s_slip, s_mobm, VIS_W, 0, VIS_W - 1, y0, y1);
	for (int y = y0; y <= y1; y++) {
		const uint16_t *mo = s_mobm + y * VIS_W;
		uint16_t *pf = s_frame + y * VIS_W;
		for (int x = 0; x < VIS_W; x++)
			if (mo[x] != 0xffff)
				pf[x] = mo[x];
	}
	for (int y = y0; y <= y1; y++) {   // the alphanumerics (64x32 tiles of 16x8, by rows; pen 0 transparent)
		uint16_t *d = s_frame + y * VIS_W;
		const int row = y >> 3, py = y & 7;
		for (int col = 0; col < VIS_W / 16; col++) {
			const uint16_t data = ALPHA[row * 64 + col];
			const uint32_t code = rbx_wrap(data & 0xfff, s_gfx_an.count);
			if (!s_gfx_an.used[code])
				continue;
			const uint16_t color = (uint16_t)(0x780 + ((data >> 12) & 7) * 16);
			const uint8_t *src = s_gfx_an.pix + code * 64 + py * 8;
			uint16_t *dd = d + col * 16;
			const int flip = data >> 15;
			for (int k = 0; k < 8; k++) {
				const uint8_t p = src[flip ? 7 - k : k];
				if (p)
					dd[2 * k] = dd[2 * k + 1] = (uint16_t)(color + p);
			}
		}
	}
}

static void update_partial(int vpos)   // screen().update_partial(vpos)
{
	if (vpos > VIS_H - 1)
		vpos = VIS_H - 1;
	if (vpos <= s_last_drawn)
		return;
	draw_lines(s_last_drawn + 1, vpos);
	s_last_drawn = vpos;
}

static void scanline_update(int scanline)   // scanline_update_one: the parameters in the alphanumerics' last columns
{
	int offset = ((scanline - 8) / 8) * 64 + 47;
	if (offset < 0)
		offset += 0x800;
	else if (offset >= 0x800)
		return;
	uint16_t word = ALPHA[offset + 3];
	if (!(word & 1) && ((word >> 1) & 7) != s_pf_bank) {
		if (scanline > 0)
			update_partial(scanline - 1);
		s_pf_bank = (word >> 1) & 7;
	}
	word = ALPHA[offset + 4];
	if (!(word & 1)) {
		const int ns = 2 * (((word >> 7) + 4) & 0x1ff);
		if (ns != s_pf_xscroll) {
			if (scanline > 0)
				update_partial(scanline - 1);
			s_pf_xscroll = ns;
		}
	}
	word = ALPHA[offset + 5];
	if (!(word & 1)) {   // (a new vscroll latches the offset into a counter)
		const int ns = ((word >> 7) - scanline) & 0x1ff;
		if (ns != s_pf_yscroll) {
			if (scanline > 0)
				update_partial(scanline - 1);
			s_pf_yscroll = ns;
		}
	}
	word = ALPHA[offset + 7];
	if (!(word & 1) && word != s_slip) {
		if (scanline > 0)
			update_partial(scanline - 1);
		s_slip = word;
	}
}

// ---- main bus ----
static void main_irq3(int state) { m68k_bus_set_line(3, state); }

static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a < 0x80000)
		return *(const uint16_t *)(s_rom + a);
	if (a >= 0xff0000)
		return s_vram[(a & 0xffff) >> 1];
	switch (a & 0xfff000) {
	case 0xfc0000: return a < 0xfc0004 ? (uint16_t)rbx_port_read(s_port[0]) : 0;
	case 0xfc2000: return a < 0xfc2004 ? (uint16_t)rbx_port_read(s_port[1]) : 0;
	case 0xfc4000:
		if (a < 0xfc4004) {
			uint16_t v = (uint16_t)(rbx_port_read(s_port[2]) & ~0x6000u);
			if (!jsa_main_to_sound_ready()) v |= 0x2000;   // (custom, active low)
			if (s_line >= VIS_H) v |= 0x4000;            // VBLANK
			return v;
		}
		return 0;
	case 0xfc6000: return a < 0xfc6004 ? (uint16_t)(jsa_main_response_r() << 8 | 0xff) : 0;
	case 0xfc8000: return (uint16_t)(0xff00 | s_eeprom[(a & 0xfff) >> 1]);
	case 0xfca000: return s_pal[(a & 0xfff) >> 1];
	case 0xfe0000: return a < 0xfe0004 ? (jsa_main_to_sound_ready() ? 0 : 0xffff) : 0;   // sound_state_r
	}
	return 0;
}

static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a >= 0xff0000) {
		uint16_t *p = &s_vram[(a & 0xffff) >> 1];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
		return;
	}
	if ((a & 0xfff000) == 0xfca000) {
		uint16_t *p = &s_pal[(a & 0xfff) >> 1];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
		return;
	}
	if ((a & 0xfff000) == 0xfc8000) {
		if ((mask & 0x00ff) && s_eep_unlocked) {
			s_eeprom[(a & 0xfff) >> 1] = (uint8_t)v;
			s_eep_unlocked = 0;   // (lock_after_write)
		}
		return;
	}
	if ((a & 0xfff000) >= 0xfd0000 && (a & 0xfff000) <= 0xfd8000 && (a & 0xfff) < 4) {
		switch (a & 0xfff000) {
		case 0xfd0000: s_eep_unlocked = 1; break;
		case 0xfd2000: jsa_sound_reset_w(); m68k_end_timeslice(); break;
		case 0xfd6000: m68k_bus_set_line(1, 0); break;   // video_int_ack_w
		case 0xfd8000:
			if (mask & 0xff00) {
				jsa_main_command_w((uint8_t)(v >> 8));
				m68k_end_timeslice();
			}
			break;
		default: break;   // the watchdog
		}
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
	m68k_bus_map(0x000000, 0x07ffff, s_rom, 0x80000, 1, 0);
	m68k_bus_map(0xfca000, 0xfcafff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus_map(0xff0000, 0xffffff, s_vram, sizeof(s_vram), 1, 1);
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
	for (int i = 0; i < 0x800; i++)
		s_snap->pal[i] = irgb_1555(s_pal[i]);
	rbx_video_frame_done();
}

static void cb_render(void)
{
	for (int y = 0; y < VIS_H; y++)
		memcpy(rbx_video.bitmap + y * rbx_video.pitch, s_snap->frame + y * VIS_W, VIS_W * 2);
	memcpy(rbx_video.palette, s_snap->pal, sizeof(s_snap->pal));
}

static void cb_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	jsa_set_test((rbx_port_read(s_port[2]) >> 15) & 1);
	jsa_frame_begin();
	s_last_drawn = -1;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (!(line & 7))   // the scanline timer (0, 8 .. 256)
			scanline_update(line);
		if (line == VIS_H) {   // VIDEO_UPDATE_BEFORE_VBLANK, then the VBLANK IRQ
			update_partial(VIS_H - 1);
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
			if (s_mdebt > 0)
				s_mdebt -= m68k_execute((int)s_mdebt);
			s_sndacc += step;   // (the 6502 at a quarter of the 68000's clock)
			jsa_run(s_sndacc >> 2);
			s_sndacc &= 3;
		}
		jsa_line();
	}
	jsa_frame_end();
}

// ---- init / reset ----
static void cb_reset(void)
{
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_pal, 0, sizeof(s_pal));
	s_slip = 0;
	s_pf_bank = s_pf_xscroll = s_pf_yscroll = 0;
	s_mo.bank = 0;
	amo_set_xscroll(&s_mo, 5);   // (video_start: the sprite positions adjusted)
	amo_set_yscroll(&s_mo, 0);
	s_eep_unlocked = 0;
	m68k_bus_irq_reset();
	jsa_reset();
	s_mdebt = 0;
	s_sndacc = 0;
	s_line = 0;
	s_last_drawn = -1;
	m68k_pulse_reset();
}

static void cb_exit(void);

static int cb_init(const rbx_game *g)
{
	uint32_t ts, ss, cs, js, os, es;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *tiles = rbx_region("tiles", &ts), *spr = rbx_region("sprites", &ss), *chars = rbx_region("chars", &cs);
	uint8_t *jrom = rbx_region("jsa:cpu", &js), *oki = rbx_region("jsa:oki1", &os), *eep = rbx_region("eeprom", &es);
	if (!s_rom || s_rom_size < 0x80000 || !tiles || !spr || !chars || !jrom || js < 0x10000 || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	{   // gfx_8x8x4_packed_msb (the tiles and chars, shown doubled), molayout (16x8, RGN_FRAC(1,4))
		static const uint32_t pp[4] = { 0, 1, 2, 3 };
		static const uint32_t px[8] = { 0, 4, 8, 12, 16, 20, 24, 28 };
		static const uint32_t py[8] = { 0, 32, 64, 96, 128, 160, 192, 224 };
		static const uint32_t my[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
		const uint32_t q = ss / 4 * 8;
		const uint32_t mx[16] = { 3 * q, 3 * q + 4, 2 * q, 2 * q + 4, q, q + 4, 0, 4,
		                          3 * q + 8, 3 * q + 12, 2 * q + 8, 2 * q + 12, q + 8, q + 12, 8, 12 };
		const rbx_gfx_layout l = { 8, 8, 4, pp, px, py, 256 };
		const rbx_gfx_layout lm = { 16, 8, 4, pp, mx, my, 128 };
		if (rbx_gfx_decode(&s_gfx_pf, &l, tiles, ts) || rbx_gfx_decode(&s_gfx_mo, &lm, spr, ss / 4) ||
		    rbx_gfx_decode(&s_gfx_an, &l, chars, cs)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			cb_exit();
			return -1;
		}
	}
	static const amo_config moc = {
		1, 1, 0, 0, 0, 1,      // banks, linked, split, reverse, swapxy, nextneighbor
		1024, 0, 0, 0x600, 0,  // slipheight, slipoffset, maxperline, palettebase, transpen
		{ 0, 0, 0x07f8, 0 }, { 0x7fff, 0, 0, 0 }, { 0, 0, 0, 0x000f }, { 0, 0, 0, 0xffc0 }, { 0, 0xff80, 0, 0 },
		{ 0 }, { 0, 0x000f, 0, 0 }, { 0x8000, 0, 0, 0 }, { 0 }, { 0 }, { 0, 0, 0, 0x0010 }, { 0 }, { 0 }, 0
	};
	if (amo_init(&s_mo, &moc, 16, 8)) {
		rbx_fail("%s: out of memory", g->set);
		cb_exit();
		return -1;
	}
	s_mo.gfx[0] = &s_gfx_mo;
	s_mo.granularity[0] = 16;
	s_frame = (uint16_t *)calloc(VIS_W * VIS_H, sizeof(uint16_t));
	s_mobm = (uint16_t *)calloc(VIS_W * VIS_H, sizeof(uint16_t));
	s_snap = calloc(1, sizeof(*s_snap));
	static const char *const names[4] = { "IN0", "IN1", "IN2", "JSA" };
	for (int i = 0; i < 4; i++)
		s_port[i] = rbx_port_find(names[i]);
	jsa_config jc;
	memset(&jc, 0, sizeof(jc));
	jc.type = 2;
	jc.out = JSA_OUT_MONO;
	jc.gain = 256;
	jc.rom = jrom;
	jc.oki = oki;
	jc.oki_size = os;
	jc.port = s_port[3];
	jc.main_int = main_irq3;
	if (!s_frame || !s_mobm || !s_snap || jsa_init(&jc) || rbx_video_setup(VIS_W, VIS_H, 0x800)) {
		rbx_fail("%s: out of memory", g->set);
		cb_exit();
		return -1;
	}
	// the EEPROM: its default contents (else blank), then the saved file
	memset(s_eeprom, 0xff, sizeof(s_eeprom));
	if (eep)
		memcpy(s_eeprom, eep, es < sizeof(s_eeprom) ? es : sizeof(s_eeprom));
	s_nv_path[0] = 0;
	{
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			FILE *f = fopen(s_nv_path, "rb");
			if (f) {
				if (fread(s_eeprom, 1, sizeof(s_eeprom), f) != sizeof(s_eeprom) && eep)
					memcpy(s_eeprom, eep, es < sizeof(s_eeprom) ? es : sizeof(s_eeprom));
				fclose(f);
			}
		}
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);
	map_main();
	cb_reset();
	return 0;
}

static void cb_exit(void)
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
static int cb_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	int32_t regs[8] = { s_eep_unlocked, (int32_t)s_mdebt, s_sndacc, s_slip, s_pf_bank, s_pf_xscroll, s_pf_yscroll, 0 };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
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
		s_slip = (uint16_t)regs[3];
		s_pf_bank = regs[4];
		s_pf_xscroll = regs[5];
		s_pf_yscroll = regs[6];
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int cb_save(void *ctx) { return cb_state(ctx, 0); }
static int cb_load(void *ctx) { return cb_state(ctx, 1); }
static int cb_supported(const char *machine) { return !strcmp(machine, "cyberbal2p"); }

const rbx_driver rbx_drv_cyberbal = { "Atari Cyberball", cb_init, cb_exit, cb_reset, cb_frame, cb_save, cb_load,
                                      cb_supported, cb_render, NULL };
