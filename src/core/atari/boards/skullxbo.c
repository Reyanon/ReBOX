/*  Atari Skull & Crossbones hardware. Ported from MAME src/mame/atari/skullxbo.cpp and atarigen.cpp
    (halt_until_hblank_0) (BSD-3-Clause, copyright-holders: Aaron Giles).
    68000 at 7.16 MHz; a 64x64 playfield of 16x8 tiles (8x8 doubled, by columns), the Atari motion objects
    (atarimo.c, 16x8 of 5 bits, two banks), a 64x31 alphanumerics layer of 16x8 tiles whose last columns hold the
    scroll and scanline interrupt commands; a 2816 parallel EEPROM. Sound: the JSA II (atarijsa.c).
    IRQ 1 from the alphanumerics' scanline interrupts, IRQ 2 at the VBLANK, IRQ 4 from the sound response.
    The screen is drawn as it goes (MAME's partial updates at the scroll and MO bank writes).
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
#define HBLANK_CYC 336             // the visible 672 pixels
#define HALT_CYC 410               // width * 9 / 10 (halt_until_hblank_0, the scanline IRQ)
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
static uint16_t s_pal[0x800];        // ff2000-ff2fff
static uint16_t s_pf[0x1000];        // ff8000-ff9fff
static uint16_t s_pfext[0x1000];     // ffa000-ffbfff
static uint16_t s_alpha[0x800];      // ffc000-ffcfff (the SLIP RAM at 0xf80)
static uint16_t s_mob[0x800];        // ffd000-ffdfff
static uint16_t s_ram[0x1000];       // ffe000-ffffff
static uint8_t s_eeprom[0x800];
static int s_eep_unlocked;
static char s_nv_path[512];
static int s_port[3];
static int s_line;
static int s_slice_pos, s_halt;
static int s_cpu_pos0;               // the line position (68000 clocks) where the running m68k_execute started
static int64_t s_mdebt;
static int s_sndacc;
static uint16_t s_xscroll, s_yscroll;
static int32_t s_pf_latch;
static int s_pf_scrollx, s_pf_scrolly;
static int s_irq1_line;              // the line of a pending scanline interrupt (at HALT_CYC), -1 none
static int s_last_drawn;

// video
static uint16_t *s_frame;            // VIS_W x VIS_H being drawn
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

static void draw_lines(int y0, int y1)   // screen_update on [y0, y1]
{
	for (int y = y0; y <= y1; y++) {   // the playfield (64x64 tiles of 16x8, by columns)
		uint16_t *d = s_frame + y * VIS_W;
		const int ty = (y + s_pf_scrolly) & 511;
		const int row = ty >> 3, py = ty & 7;
		int x = 0, tx = s_pf_scrollx & 1023;
		while (x < VIS_W) {
			const int idx = (tx >> 4) * 64 + row;
			const uint16_t data1 = s_pf[idx];
			const uint32_t code = rbx_wrap(data1 & 0x7fff, s_gfx_pf.count);
			const uint16_t color = (uint16_t)(0x200 + (s_pfext[idx] & 0x0f) * 16);
			const uint8_t *src = s_gfx_pf.pix + code * 64 + py * 8;
			const int flip = data1 >> 15;
			for (int px = tx & 15; px < 16 && x < VIS_W; px++, x++, tx++)
				d[x] = (uint16_t)(color + src[flip ? 7 - (px >> 1) : px >> 1]);
			tx &= 1023;
		}
	}
	amo_draw(&s_mo, s_mob, &s_alpha[0xf80 / 2], s_mobm, VIS_W, 0, VIS_W - 1, y0, y1);
	for (int y = y0; y <= y1; y++) {
		const uint16_t *mo = s_mobm + y * VIS_W;
		uint16_t *pf = s_frame + y * VIS_W;
		for (int x = 0; x < VIS_W; x++) {
			if (mo[x] == 0xffff)
				continue;
			const int mopriority = mo[x] >> AMO_PRIORITY_SHIFT;
			const int mopix = mo[x] & 0x1f;
			const int pfcolor = (pf[x] >> 4) & 0x0f;
			const int pfpix = pf[x] & 0x0f;
			const int o17 = (pf[x] & 0xc8) == 0xc8;
			if ((mopriority == 0 && !o17 && mopix >= 2) || (mopriority == 1 && mopix >= 2 && !(pfcolor & 0x08)) ||
			    ((mopriority & 2) && mopix >= 2 && !(pfcolor & 0x0c)) || (!(pfpix & 8) && mopix >= 2))
				pf[x] = mo[x] & 0x0fff;
			if ((mopriority == 0 && !o17 && mopix == 1) || (mopriority == 1 && mopix == 1 && !(pfcolor & 0x08)) ||
			    ((mopriority & 2) && mopix == 1 && !(pfcolor & 0x0c)) || (!(pfpix & 8) && mopix == 1))
				pf[x] |= 0x400;
		}
	}
	for (int y = y0; y <= y1; y++) {   // the alphanumerics (64x31 tiles of 16x8, by rows)
		uint16_t *d = s_frame + y * VIS_W;
		const int row = y >> 3, py = y & 7;
		if (row >= 31)
			continue;
		for (int col = 0; col < VIS_W / 16; col++) {
			const uint16_t data = s_alpha[row * 64 + col];
			const uint32_t code = rbx_wrap((data ^ 0x400) & 0x7ff, s_gfx_an.count);
			const int opaque = data & 0x8000;
			if (!opaque && !s_gfx_an.used[code])
				continue;
			const uint16_t color = (uint16_t)(0x300 + ((data >> 11) & 0x0f) * 4);
			const uint8_t *src = s_gfx_an.pix + code * 64 + py * 8;
			uint16_t *dd = d + col * 16;
			for (int k = 0; k < 8; k++)
				if (src[k] || opaque)
					dd[2 * k] = dd[2 * k + 1] = (uint16_t)(color + src[k]);
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

static int line_pos(void) { return s_cpu_pos0 + m68k_cycles_run(); }

static void xscroll_w(uint16_t v, uint16_t mask)
{
	const uint16_t n = (uint16_t)((s_xscroll & ~mask) | (v & mask));
	if (n != s_xscroll)
		update_partial(s_line);
	s_pf_scrollx = 2 * (n >> 7);
	amo_set_xscroll(&s_mo, 2 * (n >> 7));
	s_xscroll = n;
}

static void yscroll_w(uint16_t v, uint16_t mask)
{
	int scanline = s_line;
	const uint16_t n = (uint16_t)((s_yscroll & ~mask) | (v & mask));
	if (n != s_yscroll)
		update_partial(scanline);
	if (scanline > VIS_H - 1)
		scanline = 0;
	const uint16_t eff = (uint16_t)((n >> 7) - scanline);
	s_pf_scrolly = eff;
	amo_set_yscroll(&s_mo, eff & 0x1ff);
	s_yscroll = n;
}

static void scanline_timer(int scanline)   // the alphanumerics' scanline interrupt and scroll commands
{
	int offset = (scanline / 8) * 64 + 42;
	if (offset >= 0x7c0)
		return;
	if (s_alpha[offset] & 0x8000)   // the IRQ at the HBLANK of the 6th scanline following
		s_irq1_line = scanline + 6;
	if (scanline == 0) {
		const int ns = (s_yscroll >> 7) & 0x1ff;
		s_pf_scrolly = ns;
		amo_set_yscroll(&s_mo, ns);
	}
	for (int x = 42; x < 64; x++) {
		const uint16_t data = s_alpha[offset++];
		if ((data & 0x0f) == 0x0d) {
			const int ns = ((data >> 7) - scanline) & 0x1ff;
			if (scanline > 0)
				update_partial(scanline - 1);
			s_pf_scrolly = ns;
			amo_set_yscroll(&s_mo, ns);
			s_yscroll = data;
		}
	}
}

// ---- main bus ----
static void main_irq4(int state) { m68k_bus_set_line(4, state); }

static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a < 0x80000)
		return *(const uint16_t *)(s_rom + a);
	if (a < 0xff0000)
		return 0xffff;
	const uint32_t o = a & 0xffff;
	if (o >= 0xe000) return s_ram[(o - 0xe000) >> 1];
	if (o >= 0xd000) return s_mob[(o - 0xd000) >> 1];
	if (o >= 0xc000) return s_alpha[(o - 0xc000) >> 1];
	if (o >= 0xa000) return s_pfext[(o - 0xa000) >> 1];
	if (o >= 0x8000) return s_pf[(o - 0x8000) >> 1];
	if (o >= 0x6000 && o < 0x7000) return (uint16_t)(0xff00 | s_eeprom[(o & 0xfff) >> 1]);
	if (o >= 0x2000 && o < 0x3000) return s_pal[(o & 0xfff) >> 1];
	switch (o & 0xfffe) {
	case 0x4000: return s_yscroll;
	case 0x1c80: return s_xscroll;
	case 0x5000: return (uint16_t)(0xff00 | jsa_main_response_r());
	case 0x5800: return (uint16_t)rbx_port_read(s_port[0]);
	case 0x5802: {
		uint16_t v = (uint16_t)(rbx_port_read(s_port[1]) & ~0x70u);
		if (line_pos() >= HBLANK_CYC) v |= 0x10;   // HBLANK
		if (s_line >= VIS_H) v |= 0x20;            // VBLANK
		if (!jsa_main_to_sound_ready()) v |= 0x40;  // /AUDBUSY (custom, active low)
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

static inline void wr(uint16_t *p, uint16_t v, uint16_t mask) { *p = (uint16_t)((*p & ~mask) | (v & mask)); }

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a < 0xff0000)
		return;
	const uint32_t o = a & 0xffff;
	if (o >= 0xe000) { wr(&s_ram[(o - 0xe000) >> 1], v, mask); return; }
	if (o >= 0xd000) { wr(&s_mob[(o - 0xd000) >> 1], v, mask); return; }
	if (o >= 0xc000) { wr(&s_alpha[(o - 0xc000) >> 1], v, mask); return; }
	if (o >= 0xa000) { wr(&s_pfext[(o - 0xa000) >> 1], v, mask); return; }
	if (o >= 0x8000) {   // playfield_latched_w
		const uint32_t i = (o - 0x8000) >> 1;
		wr(&s_pf[i], v, mask);
		if (s_pf_latch != -1)
			s_pfext[i] = (uint16_t)((s_pfext[i] & ~0x00ff) | (s_pf_latch & 0x00ff));
		return;
	}
	if (o >= 0x6000 && o < 0x7000) {
		if ((mask & 0x00ff) && s_eep_unlocked) {
			s_eeprom[(o & 0xfff) >> 1] = (uint8_t)v;
			s_eep_unlocked = 0;   // (lock_after_write)
		}
		return;
	}
	if (o >= 0x4000 && o < 0x4800) { yscroll_w(v, mask); return; }
	if (o >= 0x2000 && o < 0x3000) { wr(&s_pal[(o & 0xfff) >> 1], v, mask); return; }
	if (o < 0x0800) {   // mobmsb_w
		update_partial(s_line);
		s_mo.bank = (int)((o >> 10) & 1);
		return;
	}
	if (o < 0x0c00) {   // halt_until_hblank_0
		const int pos = line_pos();
		s_halt = (pos >= HALT_CYC ? HALT_CYC + LINE_CYC : HALT_CYC) - pos;
		m68k_end_timeslice();
		return;
	}
	if (o < 0x1000) { s_eep_unlocked = 1; return; }
	if (o < 0x1400) { m68k_bus_set_line(2, 0); return; }   // video_int_ack_w
	if (o < 0x1800) {
		if (mask & 0x00ff) {
			jsa_main_command_w((uint8_t)v);
			m68k_end_timeslice();
		}
		return;
	}
	if (o < 0x1c00) { jsa_sound_reset_w(); m68k_end_timeslice(); return; }
	if (o < 0x2000) {
		switch ((o >> 7) & 3) {
		case 0: s_pf_latch = v; break;   // playfield_latch_w (COMBINE into an int32: the full word)
		case 1: xscroll_w(v, mask); break;
		case 2: m68k_bus_set_line(1, 0); break;   // scanline_int_ack_w
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
	m68k_bus_map(0xff2000, 0xff2fff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus_map(0xffa000, 0xffbfff, s_pfext, sizeof(s_pfext), 1, 1);
	m68k_bus_map(0xffc000, 0xffcfff, s_alpha, sizeof(s_alpha), 1, 1);
	m68k_bus_map(0xffd000, 0xffdfff, s_mob, sizeof(s_mob), 1, 1);
	m68k_bus_map(0xffe000, 0xffffff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0xff8000, 0xff9fff, s_pf, sizeof(s_pf), 1, 0);   // (writes: the latch)
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

static void sx_render(void)
{
	for (int y = 0; y < VIS_H; y++)
		memcpy(rbx_video.bitmap + y * rbx_video.pitch, s_snap->frame + y * VIS_W, VIS_W * 2);
	memcpy(rbx_video.palette, s_snap->pal, sizeof(s_snap->pal));
}

static void run_main(int step)
{
	s_cpu_pos0 = s_slice_pos - (int)s_mdebt;   // (the 68000 is ahead of the slice by its overrun)
	s_mdebt += step;
	if (s_mdebt <= 0)
		return;
	if (s_halt > 0) {   // (halted: the clocks pass)
		const int h = s_halt < s_mdebt ? s_halt : (int)s_mdebt;
		s_halt -= h;
		s_mdebt -= h;
		s_cpu_pos0 += h;
	}
	if (s_mdebt > 0) {
		s_mdebt -= m68k_execute((int)s_mdebt);
		if (s_halt > 0 && s_mdebt > 0) {
			const int h = s_halt < s_mdebt ? s_halt : (int)s_mdebt;
			s_halt -= h;
			s_mdebt -= h;
		}
	}
}

static void sx_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	jsa_set_test((rbx_port_read(s_port[1]) >> 7) & 1);
	jsa_frame_begin();
	s_last_drawn = -1;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (!(line & 7))
			scanline_timer(line);
		if (line == VIS_H) {   // VIDEO_UPDATE_BEFORE_VBLANK, then the VBLANK IRQ
			update_partial(VIS_H - 1);
			capture();
			m68k_bus_set_line(2, 1);
		}
		for (int pos = 0; pos < LINE_CYC;) {
			int step = jsa_boosting() ? 36 : LINE_CYC / SLICES;
			if (pos + step > LINE_CYC)
				step = LINE_CYC - pos;
			if (s_irq1_line == line) {
				if (pos >= HALT_CYC) {   // the scanline interrupt
					s_irq1_line = -1;
					m68k_bus_set_line(1, 1);
				} else if (pos + step > HALT_CYC)
					step = HALT_CYC - pos;
			}
			s_slice_pos = pos;
			run_main(step);
			pos += step;
			s_sndacc += step;   // (the 6502 at a quarter of the 68000's clock)
			jsa_run(s_sndacc >> 2);
			s_sndacc &= 3;
		}
		jsa_line();
	}
	jsa_frame_end();
}

// ---- init / reset ----
static void sx_reset(void)
{
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_pf, 0, sizeof(s_pf));
	memset(s_pfext, 0, sizeof(s_pfext));
	memset(s_alpha, 0, sizeof(s_alpha));
	memset(s_mob, 0, sizeof(s_mob));
	memset(s_ram, 0, sizeof(s_ram));
	s_mo.bank = 0;
	amo_set_xscroll(&s_mo, 0);
	amo_set_yscroll(&s_mo, 0);
	s_xscroll = s_yscroll = 0;
	s_pf_scrollx = s_pf_scrolly = 0;
	s_pf_latch = -1;
	s_irq1_line = -1;
	s_eep_unlocked = 0;
	m68k_bus_irq_reset();
	jsa_reset();
	s_mdebt = 0;
	s_sndacc = 0;
	s_halt = 0;
	s_slice_pos = 0;
	s_line = 0;
	s_last_drawn = -1;
	m68k_pulse_reset();
}

static void sx_exit(void);

static int sx_init(const rbx_game *g)
{
	uint32_t ps, ss, cs, js, os;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *pf = rbx_region("playfield", &ps), *spr = rbx_region("sprites", &ss), *chars = rbx_region("chars", &cs);
	uint8_t *jrom = rbx_region("jsa:cpu", &js), *oki = rbx_region("jsa:oki1", &os);
	if (!s_rom || s_rom_size < 0x80000 || !pf || !spr || !chars || !jrom || js < 0x10000 || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	{   // molayout (5 planes, RGN_FRAC(1,5)), pflayout (RGN_FRAC(1,2)), anlayout; the last two shown doubled
		const uint32_t f = ss / 5 * 8, h = ps / 2 * 8;
		const uint32_t mp[5] = { 4 * f, 3 * f, 2 * f, f, 0 };
		static const uint32_t mx[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
		static const uint32_t y16[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
		static const uint32_t pp[4] = { 0, 1, 2, 3 };
		const uint32_t px[8] = { h + 0, h + 4, 0, 4, h + 8, h + 12, 8, 12 };
		static const uint32_t ap[2] = { 0, 1 }, ax[8] = { 0, 2, 4, 6, 8, 10, 12, 14 };
		const rbx_gfx_layout lm = { 16, 8, 5, mp, mx, y16, 128 };
		const rbx_gfx_layout lp = { 8, 8, 4, pp, px, y16, 128 };
		const rbx_gfx_layout la = { 8, 8, 2, ap, ax, y16, 128 };
		if (rbx_gfx_decode(&s_gfx_mo, &lm, spr, ss / 5) || rbx_gfx_decode(&s_gfx_pf, &lp, pf, ps / 2) ||
		    rbx_gfx_decode(&s_gfx_an, &la, chars, cs)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			sx_exit();
			return -1;
		}
	}
	static const amo_config moc = {
		2, 1, 0, 0, 0, 0,   // banks, linked, split, reverse, swapxy, nextneighbor
		8, 0, 0, 0x000, 0,  // slipheight, slipoffset, maxperline, palettebase, transpen
		{ 0x00ff, 0, 0, 0 }, { 0, 0x7fff, 0, 0 }, { 0, 0, 0x000f, 0 }, { 0, 0, 0xffc0, 0 }, { 0, 0, 0, 0xff80 },
		{ 0, 0, 0, 0x0070 }, { 0, 0, 0, 0x000f }, { 0, 0x8000, 0, 0 }, { 0 }, { 0, 0, 0x0030, 0 }, { 0 }, { 0 }, { 0 }, 0
	};
	if (amo_init(&s_mo, &moc, 16, 8)) {
		rbx_fail("%s: out of memory", g->set);
		sx_exit();
		return -1;
	}
	s_mo.gfx[0] = &s_gfx_mo;
	s_mo.granularity[0] = 32;
	s_frame = (uint16_t *)calloc(VIS_W * VIS_H, sizeof(uint16_t));
	s_mobm = (uint16_t *)calloc(VIS_W * VIS_H, sizeof(uint16_t));
	s_snap = calloc(1, sizeof(*s_snap));
	static const char *const names[3] = { "FF5800", "FF5802", "JSA" };
	for (int i = 0; i < 3; i++)
		s_port[i] = rbx_port_find(names[i]);
	jsa_config jc;
	memset(&jc, 0, sizeof(jc));
	jc.type = 2;
	jc.out = JSA_OUT_MONO;
	jc.gain = 205;   // add_route(ALL_OUTPUTS, "mono", 0.8)
	jc.rom = jrom;
	jc.oki = oki;
	jc.oki_size = os;
	jc.port = s_port[2];
	jc.main_int = main_irq4;
	if (!s_frame || !s_mobm || !s_snap || jsa_init(&jc) || rbx_video_setup(VIS_W, VIS_H, 0x800)) {
		rbx_fail("%s: out of memory", g->set);
		sx_exit();
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
	sx_reset();
	return 0;
}

static void sx_exit(void)
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
static int sx_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	int32_t regs[13] = { s_eep_unlocked, (int32_t)s_mdebt, s_sndacc, s_halt, s_xscroll, s_yscroll, s_pf_latch,
	                     s_pf_scrollx, s_pf_scrolly, s_irq1_line, s_mo.bank, s_mo.xscroll, s_mo.yscroll };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "pf", s_pf, sizeof(s_pf));
	rc |= rbx_state_io(ctx, "pfext", s_pfext, sizeof(s_pfext));
	rc |= rbx_state_io(ctx, "alpha", s_alpha, sizeof(s_alpha));
	rc |= rbx_state_io(ctx, "mob", s_mob, sizeof(s_mob));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
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
		s_xscroll = (uint16_t)regs[4];
		s_yscroll = (uint16_t)regs[5];
		s_pf_latch = regs[6];
		s_pf_scrollx = regs[7];
		s_pf_scrolly = regs[8];
		s_irq1_line = regs[9];
		s_mo.bank = regs[10];
		s_mo.xscroll = regs[11];
		s_mo.yscroll = regs[12];
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int sx_save(void *ctx) { return sx_state(ctx, 0); }
static int sx_load(void *ctx) { return sx_state(ctx, 1); }
static int sx_supported(const char *machine) { return !strcmp(machine, "skullxbo"); }

const rbx_driver rbx_drv_skullxbo = { "Atari Skull & Crossbones", sx_init, sx_exit, sx_reset, sx_frame, sx_save,
                                      sx_load, sx_supported, sx_render, NULL };
