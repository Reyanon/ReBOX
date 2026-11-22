/*  Atari Escape from the Planet of the Robot Monsters hardware: Escape (two 68000s, the JSA I without its POKEY), the
    Klax prototypes and Guts n' Glory (one 68000, the JSA II). Ported from MAME src/mame/atari/eprom.cpp
    (BSD-3-Clause, copyright-holders: Aaron Giles) and the apply_stain of atarimo.cpp.
    68000s at 7.16 MHz (the extra CPU shares 0x160000-0x16ffff, held in reset by the latch); a 64x64 playfield of 8x8
    tiles (by columns), the Atari motion objects (atarimo.c), a 64x31 alphanumerics layer; an ADC0809 on the
    analog sticks (Escape, Guts); a 2804 parallel EEPROM. IRQ 4 at the VBLANK, IRQ 6 from the sound response.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k_multi.h"
#include "../../cpu/m68k/m68k.h"
#include "../../machine/adc0808.h"
#include "../../video/gfxlayout.h"
#include "../video/atarimo.h"
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

enum { K_EPROM, K_KLAXP, K_GUTS };
static int s_kind, s_ncpu;
static uint8_t *s_rom, *s_xrom;
static uint32_t s_rom_size, s_xrom_size;
static rbx_gfx s_gfx_pf, s_gfx_mo, s_gfx_an;
static amo_t s_mo;
static adc0808_t s_adc;

// memory (host-order words)
static uint16_t s_shared[0x8000];    // 160000-16ffff
static uint16_t s_pal[0x800];        // 3e0000-3e0fff
static uint16_t s_pf[0x1000];        // 3f0000-3f1fff (guts ff8000)
static uint16_t s_pfext[0x1000];     // 3f8000-3f9fff (guts ff0000)
static uint16_t s_mob[0x1000];       // 3f2000-3f3fff (guts ffa000)
static uint16_t s_alpha[0x800];      // 3f4000-3f4fff, the SLIP RAM at 0xf80 (guts ffc000)
static uint16_t s_ram[0x1800];       // 3f5000-3f7fff (guts ffd000)
static uint8_t s_eeprom[0x200];
static int s_eep_unlocked;
static char s_nv_path[512];
static int s_port[7];
static int s_line;
static int s_intensity, s_video_disable;
static int s_extra_held, s_extra_release;
static int64_t s_mdebt[2];
static int s_sndacc, s_adcacc;

// video
static uint16_t *s_frame;            // VIS_W x VIS_H
static uint16_t *s_mobm;             // the MO bitmap
static struct { uint16_t frame[VIS_W * VIS_H]; uint32_t pal[0x800]; } *s_snap;
static int s_xscroll, s_yscroll;

static void draw_frame(void)
{
	if (s_video_disable) {
		memset(s_frame, 0, VIS_W * VIS_H * sizeof(uint16_t));
		return;
	}
	for (int y = 0; y < VIS_H; y++) {   // the playfield (64x64 tiles, by columns)
		uint16_t *d = s_frame + y * VIS_W;
		const int ty = (y + s_yscroll) & 511;
		const int row = ty >> 3, py = ty & 7;
		int x = 0, tx = s_xscroll & 511;
		while (x < VIS_W) {
			const int idx = (tx >> 3) * 64 + row;
			const uint16_t data1 = s_pf[idx];
			const uint32_t code = rbx_wrap(data1 & 0x7fff, s_gfx_pf.count);
			const uint16_t color = (uint16_t)(256 + (0x10 + ((s_pfext[idx] >> 8) & 0x0f)) * 16);
			const uint8_t *src = s_gfx_pf.pix + code * 64 + py * 8;
			const int flip = data1 >> 15;
			for (int px = tx & 7; px < 8 && x < VIS_W; px++, x++, tx++)
				d[x] = (uint16_t)(color + src[flip ? 7 - px : px]);
			tx &= 511;
		}
	}
	amo_draw(&s_mo, s_mob, &s_alpha[0xf80 / 2], s_mobm, VIS_W, 0, VIS_W - 1, 0, VIS_H - 1);
	for (int i = 0; i < VIS_W * VIS_H; i++) {
		const uint16_t mo = s_mobm[i];
		if (mo == 0xffff)
			continue;
		uint16_t *pf = &s_frame[i];
		const int mopriority = (mo >> AMO_PRIORITY_SHIFT) & 7;
		if (mopriority & 4)   // (special rendering: drawn below)
			continue;
		if (s_kind == K_GUTS) {
			const int pfpriority = (*pf >> 5) & 3;
			if (!(*pf & 8) || mopriority >= pfpriority)
				*pf = mo & 0x0fff;
			continue;
		}
		const int pfpriority = (*pf >> 4) & 3;
		int forcemc0 = 0;
		if (!(*pf & 8)) {
			if (((pfpriority == 3) && !(mopriority & 1)) || ((pfpriority & 2) && !(mopriority & 2)) ||
			    ((pfpriority & 1) && (mopriority == 0)))
				forcemc0 = 1;
		}
		int shade = 1;
		if (((mo & 0x0f) != 1) || ((mo & 0xf0) == 0) || forcemc0)
			shade = 0;
		int pfm = 1;
		if ((mopriority == 3) || (*pf & 8) || (!(pfpriority & 1) && (mopriority & 2)) ||
		    (!(pfpriority & 2) && (mopriority & 2)) || (!(pfpriority & 2) && (mopriority & 1)) ||
		    ((pfpriority == 0) && (mopriority == 0)))
			pfm = 0;
		const int m7 = (mo & 0x0f) == 1;
		if (!pfm && !m7)
			*pf = forcemc0 ? (uint16_t)(mo & 0x0fff & ~0x70) : (uint16_t)(mo & 0x0fff);
		else {
			if (shade) *pf |= 0x100;
			if (m7) *pf |= 0x080;
		}
	}
	for (int y = 0; y < VIS_H; y++) {   // the alphanumerics (64x31, by rows; pen 0 transparent unless opaque)
		uint16_t *d = s_frame + y * VIS_W;
		const int row = y >> 3, py = y & 7;
		if (row >= 31)
			continue;
		for (int col = 0; col < VIS_W / 8; col++) {
			const uint16_t data = s_alpha[row * 64 + col];
			const uint32_t code = rbx_wrap(data & 0x3ff, s_gfx_an.count);
			const int opaque = data & 0x8000;
			if (!opaque && !s_gfx_an.used[code])
				continue;
			const uint16_t color = (uint16_t)((((data >> 10) & 0x0f) | ((data >> 9) & 0x20)) * 4);
			const uint8_t *src = s_gfx_an.pix + code * 64 + py * 8;
			uint16_t *dd = d + col * 8;
			for (int k = 0; k < 8; k++)
				if (src[k] || opaque)
					dd[k] = (uint16_t)(color + src[k]);
		}
	}
	for (int y = 0; y < VIS_H; y++) {   // the upper bit of the MO priority: apply_stain
		const uint16_t *mo = s_mobm + y * VIS_W;
		uint16_t *pf = s_frame + y * VIS_W;
		for (int x = 0; x < VIS_W; x++) {
			if (mo[x] == 0xffff || !((mo[x] >> AMO_PRIORITY_SHIFT) & 4) || !(mo[x] & 2))
				continue;
			const uint16_t START_MARKER = (4 << AMO_PRIORITY_SHIFT) | 2, END_MARKER = (4 << AMO_PRIORITY_SHIFT) | 4;
			int offnext = 0;
			for (int sx = x; sx < VIS_W; sx++) {
				pf[sx] |= 0x400;
				if (offnext && (mo[sx] == 0xffff || (mo[sx] & START_MARKER) != START_MARKER))
					break;
				offnext = mo[sx] != 0xffff && (mo[sx] & END_MARKER) == END_MARKER;
			}
		}
	}
}

// ---- the buses ----
static void main_irq6(int state) { m68k_multi_set_line(0, 6, state); }

static void int_ack(void)
{
	m68k_multi_set_line(0, 4, 0);
	if (s_ncpu > 1)
		m68k_multi_set_line(1, 4, 0);
}

static void latch_w(uint8_t data)
{
	if (s_ncpu < 2)
		return;
	if (data & 1) {   // the extra CPU's reset released
		if (s_extra_held)
			s_extra_release = 1;
		s_extra_held = 0;
	} else {
		s_extra_held = 1;
		if (m68k_multi_current() == 1)
			m68k_end_timeslice();
	}
	s_intensity = (data & 0x1e) >> 1;
	s_video_disable = (data >> 5) & 1;
}

static uint16_t io_r(uint32_t a)
{
	switch (a & 0x30) {
	case 0x00: return (uint16_t)rbx_port_read(s_port[0]);
	case 0x10: {
		uint16_t v = (uint16_t)(rbx_port_read(s_port[1]) & ~0x1du);
		if (s_line < VIS_H) v |= 0x01;               // VBLANK (custom, active low)
		if (!jsa_sound_to_main_ready()) v |= 0x04;   // (custom, active low)
		if (!jsa_main_to_sound_ready()) v |= 0x08;
		if (s_kind == K_KLAXP) v |= 0x10;            // (unused, active low)
		else if (s_adc.eoc) v |= 0x10;               // the ADC's EOC
		return v;
	}
	case 0x20: {   // adc_r
		if (s_kind == K_KLAXP)
			return 0xffff;
		const uint8_t r = adc0808_data_r(&s_adc);
		adc0808_start(&s_adc, (int)((a >> 1) & 3));
		return (uint16_t)(0xff00 | r);
	}
	default: return (uint16_t)(0xff00 | jsa_main_response_r());
	}
}

static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	const int cpu = m68k_multi_current();
	if (a < 0x100000) {
		if (cpu)
			return a < 0x80000 ? *(const uint16_t *)(s_xrom + a) : 0xffff;
		if (a < 0xa0000)
			return *(const uint16_t *)(s_rom + a);
		if (a >= 0x0e0000 && a < 0x0e0400)
			return (uint16_t)(0xff00 | s_eeprom[(a & 0x3ff) >> 1]);
		return 0xffff;
	}
	if ((a & 0xff0000) == 0x160000)
		return s_shared[(a & 0xffff) >> 1];
	if ((a & 0xffffc0) == 0x260000)
		return io_r(a);
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
	const int cpu = m68k_multi_current();
	if ((a & 0xff0000) == 0x160000) {
		uint16_t *p = &s_shared[(a & 0xffff) >> 1];
		const uint16_t old = *p;
		*p = (uint16_t)((*p & ~mask) | (v & mask));
		if ((a & 0xfffffe) == 0x16cc00 && (old & 0xff00) != (*p & 0xff00))
			m68k_end_timeslice();   // sync_w: yield
		return;
	}
	if (!cpu) {
		if (a >= 0x0e0000 && a < 0x0e0400) {
			if ((mask & 0x00ff) && s_eep_unlocked) {
				s_eeprom[(a & 0x3ff) >> 1] = (uint8_t)v;
				s_eep_unlocked = 0;   // (lock_after_write)
			}
			return;
		}
		if ((a & 0xff0000) == 0x1f0000) {
			s_eep_unlocked = 1;
			return;
		}
	}
	switch (a & 0xfffffe) {
	case 0x360000: int_ack(); break;
	case 0x360010:
		if ((mask & 0x00ff) && s_kind != K_GUTS)
			latch_w((uint8_t)v);
		break;
	case 0x360020: jsa_sound_reset_w(); m68k_end_timeslice(); break;
	case 0x360030:
		if (mask & 0x00ff) {
			jsa_main_command_w((uint8_t)v);
			m68k_end_timeslice();
		}
		break;
	default: break;   // the watchdog
	}
}

static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	main_write16m(a & ~1u, (uint16_t)(v << 8 | v), a & 1 ? 0x00ff : 0xff00);
}

static void map_cpus(void)
{
	for (int c = 0; c < s_ncpu; c++) {
		m68k_bus_reset_map();
		if (!c) {
			m68k_bus_map(0x000000, 0x09ffff, s_rom, 0xa0000, 1, 0);
			m68k_bus_map(0x3e0000, 0x3e0fff, s_pal, sizeof(s_pal), 1, 1);
			const uint32_t vb = s_kind == K_GUTS ? 0xff0000 : 0x3f0000;
			if (s_kind == K_GUTS) {
				m68k_bus_map(vb + 0x0000, vb + 0x1fff, s_pfext, sizeof(s_pfext), 1, 1);
				m68k_bus_map(vb + 0x8000, vb + 0x9fff, s_pf, sizeof(s_pf), 1, 1);
				m68k_bus_map(vb + 0xa000, vb + 0xbfff, s_mob, sizeof(s_mob), 1, 1);
				m68k_bus_map(vb + 0xc000, vb + 0xcfff, s_alpha, sizeof(s_alpha), 1, 1);
				m68k_bus_map(vb + 0xd000, vb + 0xffff, s_ram, sizeof(s_ram), 1, 1);
			} else {
				m68k_bus_map(vb + 0x0000, vb + 0x1fff, s_pf, sizeof(s_pf), 1, 1);
				m68k_bus_map(vb + 0x2000, vb + 0x3fff, s_mob, sizeof(s_mob), 1, 1);
				m68k_bus_map(vb + 0x4000, vb + 0x4fff, s_alpha, sizeof(s_alpha), 1, 1);
				m68k_bus_map(vb + 0x5000, vb + 0x7fff, s_ram, sizeof(s_ram), 1, 1);
				m68k_bus_map(vb + 0x8000, vb + 0x9fff, s_pfext, sizeof(s_pfext), 1, 1);
			}
		} else
			m68k_bus_map(0x000000, 0x07ffff, s_xrom, 0x80000, 1, 0);
		// the shared RAM but its page with the sync word
		m68k_bus_map(0x160000, 0x16bfff, s_shared, 0xc000, 1, 1);
		m68k_bus_map(0x16c000, 0x16cfff, s_shared + 0x6000, 0x1000, 1, 0);
		m68k_bus_map(0x16d000, 0x16ffff, s_shared + 0x6800, 0x3000, 1, 1);
		m68k_bus.read8 = main_read8;
		m68k_bus.read16 = main_read16;
		m68k_bus.write8 = main_write8;
		m68k_bus.write16 = main_write16;
		m68k_multi_capture(c);
	}
	m68k_multi_finish();
}

// ---- frame ----
static void capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->frame, s_frame, sizeof(s_snap->frame));
	for (int c = 0; c < 0x800; c++) {   // update_palette
		const uint16_t data = s_pal[c];
		int i = (((data >> 12) & 15) + 1) * (4 - s_intensity);
		if (i < 0)
			i = 0;
		const uint32_t r = (uint32_t)(((data >> 8) & 15) * i / 4), g = (uint32_t)(((data >> 4) & 15) * i / 4);
		const uint32_t b = (uint32_t)((data & 15) * i / 4);
		s_snap->pal[c] = s_video_disable ? 0 : r << 16 | g << 8 | b;
	}
	rbx_video_frame_done();
}

static void ep_render(void)
{
	for (int y = 0; y < VIS_H; y++)
		memcpy(rbx_video.bitmap + y * rbx_video.pitch, s_snap->frame + y * VIS_W, VIS_W * 2);
	memcpy(rbx_video.palette, s_snap->pal, sizeof(s_snap->pal));
}

static void ep_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	jsa_set_test((rbx_port_read(s_port[1]) >> 1) & 1);
	jsa_frame_begin();
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == 0) {   // scanline_update(0): the scroll
			s_xscroll = (s_alpha[0x780] >> 7) & 0x1ff;
			s_yscroll = (s_alpha[0x781] >> 7) & 0x1ff;
			amo_set_xscroll(&s_mo, s_xscroll);
			amo_set_yscroll(&s_mo, s_yscroll);
		}
		if (line == VIS_H) {   // VIDEO_UPDATE_BEFORE_VBLANK, then the VBLANK IRQ
			draw_frame();
			capture();
			m68k_multi_set_line(0, 4, 1);
			if (s_ncpu > 1)
				m68k_multi_set_line(1, 4, 1);
		}
		for (int left = LINE_CYC; left > 0;) {
			int step = jsa_boosting() ? 36 : LINE_CYC / SLICES;
			if (step > left)
				step = left;
			left -= step;
			for (int c = 0; c < s_ncpu; c++) {
				s_mdebt[c] += step;
				if (c) {
					if (s_extra_release) {
						s_extra_release = 0;
						m68k_multi_reset_cpu(1);
					}
					if (s_extra_held) {
						s_mdebt[1] = 0;
						continue;
					}
				}
				if (s_mdebt[c] > 0) {
					m68k_multi_switch(c);
					s_mdebt[c] -= m68k_execute((int)s_mdebt[c]);
				}
			}
			m68k_multi_switch(0);
			s_sndacc += step;   // (the 6502 at a quarter of the 68000's clock)
			jsa_run(s_sndacc >> 2);
			s_sndacc &= 3;
			s_adcacc += step;   // (the ADC at 14.318181 MHz / 16)
			if (adc0808_advance(&s_adc, s_adcacc >> 3)) {
				const int p = s_port[3 + (s_adc.addr & 3)];
				s_adc.sar = (uint8_t)(p >= 0 ? rbx_port_read(p) : 0x80);
			}
			s_adcacc &= 7;
		}
		jsa_line();
	}
	jsa_frame_end();
}

// ---- init / reset ----
static void ep_reset(void)
{
	memset(s_shared, 0, sizeof(s_shared));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_pf, 0, sizeof(s_pf));
	memset(s_pfext, 0, sizeof(s_pfext));
	memset(s_mob, 0, sizeof(s_mob));
	memset(s_alpha, 0, sizeof(s_alpha));
	memset(s_ram, 0, sizeof(s_ram));
	s_mo.bank = 0;
	s_xscroll = s_yscroll = 0;
	amo_set_xscroll(&s_mo, 0);
	amo_set_yscroll(&s_mo, 0);
	s_eep_unlocked = 0;
	s_intensity = 0;
	s_video_disable = 0;
	s_extra_held = s_extra_release = 0;
	adc0808_reset(&s_adc);
	jsa_reset();
	s_mdebt[0] = s_mdebt[1] = 0;
	s_sndacc = s_adcacc = 0;
	s_line = 0;
	m68k_multi_reset();
	for (int c = 0; c < s_ncpu; c++)
		m68k_multi_set_asserted(c, 0);
}

static void ep_exit(void);

static int ep_init(const rbx_game *g)
{
	const char *m = g->machine;
	s_kind = !strcmp(m, "guts") ? K_GUTS : !strcmp(m, "klaxp") ? K_KLAXP : K_EPROM;
	s_ncpu = s_kind == K_EPROM ? 2 : 1;
	uint32_t ts, ms, cs, js, os = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_xrom = rbx_region("extra", &s_xrom_size);
	uint8_t *tiles = rbx_region(s_kind == K_GUTS ? "tiles" : "spr_tiles", &ts);
	uint8_t *spr = s_kind == K_GUTS ? rbx_region("sprites", &ms) : (ms = ts, tiles);
	uint8_t *chars = rbx_region("chars", &cs), *jrom = rbx_region("jsa:cpu", &js);
	uint8_t *oki = rbx_region("jsa:oki1", &os);
	if (!s_rom || s_rom_size < 0xa0000 || !tiles || !spr || !chars || !jrom || js < 0x10000 ||
	    (s_ncpu > 1 && (!s_xrom || s_xrom_size < 0x80000)) || (s_kind != K_EPROM && !oki)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	if (s_xrom)
		m68k_bus_swap16(s_xrom, s_xrom_size);
	{   // pfmolayout (RGN_FRAC(1,4) planes, the lowest first), anlayout
		static const uint32_t px[8] = { 0, 1, 2, 3, 4, 5, 6, 7 }, py[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
		static const uint32_t ap[2] = { 0, 4 }, ax[8] = { 0, 1, 2, 3, 8, 9, 10, 11 };
		static const uint32_t ay[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
		const uint32_t q = ts / 4 * 8, qm = ms / 4 * 8;
		const uint32_t pp[4] = { 0, q, 2 * q, 3 * q }, pm[4] = { 0, qm, 2 * qm, 3 * qm };
		const rbx_gfx_layout lp = { 8, 8, 4, pp, px, py, 64 };
		const rbx_gfx_layout lm = { 8, 8, 4, pm, px, py, 64 };
		const rbx_gfx_layout la = { 8, 8, 2, ap, ax, ay, 128 };
		if (rbx_gfx_decode(&s_gfx_pf, &lp, tiles, ts / 4) || rbx_gfx_decode(&s_gfx_an, &la, chars, cs) ||
		    (s_kind == K_GUTS && rbx_gfx_decode(&s_gfx_mo, &lm, spr, ms / 4))) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			ep_exit();
			return -1;
		}
	}
	static const amo_config moc = {
		1, 1, 0, 1, 0, 0,   // banks, linked, split, reverse, swapxy, nextneighbor
		8, 0, 0, 0x100, 0,  // slipheight, slipoffset, maxperline, palettebase, transpen
		{ 0x03ff, 0, 0, 0 }, { 0, 0x7fff, 0, 0 }, { 0, 0, 0x000f, 0 }, { 0, 0, 0xff80, 0 }, { 0, 0, 0, 0xff80 },
		{ 0, 0, 0, 0x0070 }, { 0, 0, 0, 0x0007 }, { 0, 0, 0, 0x0008 }, { 0 }, { 0, 0, 0x0070, 0 }, { 0 }, { 0 }, { 0 }, 0
	};
	static const amo_config moc_guts = {
		1, 1, 0, 0, 0, 0,
		8, 0, 0, 0x100, 0,
		{ 0x03ff, 0, 0, 0 }, { 0, 0x7fff, 0, 0 }, { 0, 0, 0x000f, 0 }, { 0, 0, 0xff80, 0 }, { 0, 0, 0, 0xff80 },
		{ 0, 0, 0, 0x0070 }, { 0, 0, 0, 0x000f }, { 0, 0x8000, 0, 0 }, { 0 }, { 0, 0, 0x0070, 0 }, { 0 }, { 0 }, { 0 }, 0
	};
	if (amo_init(&s_mo, s_kind == K_GUTS ? &moc_guts : &moc, 8, 8) || m68k_multi_init(s_ncpu)) {
		rbx_fail("%s: out of memory", g->set);
		ep_exit();
		return -1;
	}
	s_mo.gfx[0] = s_kind == K_GUTS ? &s_gfx_mo : &s_gfx_pf;
	s_mo.granularity[0] = 16;
	s_frame = (uint16_t *)calloc(VIS_W * VIS_H, sizeof(uint16_t));
	s_mobm = (uint16_t *)calloc(VIS_W * VIS_H, sizeof(uint16_t));
	s_snap = calloc(1, sizeof(*s_snap));
	static const char *const names[7] = { "260000", "260010", "JSA", "ADC0", "ADC1", "ADC2", "ADC3" };
	for (int i = 0; i < 7; i++)
		s_port[i] = rbx_port_find(names[i]);
	jsa_config jc;
	memset(&jc, 0, sizeof(jc));
	jc.type = s_kind == K_EPROM ? 1 : 2;
	jc.tms = s_kind == K_EPROM;
	jc.out = JSA_OUT_MONO;
	jc.gain = s_kind == K_EPROM ? 102 : 256;   // add_route(ALL_OUTPUTS, "mono", 0.4 / 1.0)
	jc.rom = jrom;
	jc.oki = oki;
	jc.oki_size = os;
	jc.port = s_port[2];
	jc.main_int = main_irq6;
	if (!s_frame || !s_mobm || !s_snap || jsa_init(&jc) || rbx_video_setup(VIS_W, VIS_H, 0x800)) {
		rbx_fail("%s: out of memory", g->set);
		ep_exit();
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
	map_cpus();
	ep_reset();
	return 0;
}

static void ep_exit(void)
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
	m68k_multi_exit();
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
static int ep_state(void *ctx, int load)
{
	uint32_t asserted[2] = { m68k_multi_asserted(0), s_ncpu > 1 ? m68k_multi_asserted(1) : 0 };
	int32_t regs[12] = { s_eep_unlocked, s_intensity, s_video_disable, s_extra_held, s_extra_release,
	                     (int32_t)s_mdebt[0], (int32_t)s_mdebt[1], s_sndacc, s_adcacc, s_xscroll, s_yscroll, 0 };
	int rc = 0;
	rc |= m68k_multi_state(ctx, load);
	rc |= rbx_state_io(ctx, "asserted", asserted, sizeof(asserted));
	rc |= rbx_state_io(ctx, "shared", s_shared, sizeof(s_shared));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "pf", s_pf, sizeof(s_pf));
	rc |= rbx_state_io(ctx, "pfext", s_pfext, sizeof(s_pfext));
	rc |= rbx_state_io(ctx, "mob", s_mob, sizeof(s_mob));
	rc |= rbx_state_io(ctx, "alpha", s_alpha, sizeof(s_alpha));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "eeprom", s_eeprom, sizeof(s_eeprom));
	rc |= rbx_state_io(ctx, "regs", regs, sizeof(regs));
	rc |= rbx_state_io(ctx, "adc", &s_adc, sizeof(s_adc));
	rc |= jsa_state(ctx, load);
	if (!rc && load) {
		for (int c = 0; c < s_ncpu; c++)
			m68k_multi_set_asserted(c, asserted[c]);
		s_eep_unlocked = regs[0];
		s_intensity = regs[1];
		s_video_disable = regs[2];
		s_extra_held = regs[3];
		s_extra_release = regs[4];
		s_mdebt[0] = regs[5];
		s_mdebt[1] = regs[6];
		s_sndacc = regs[7];
		s_adcacc = regs[8];
		s_xscroll = regs[9];
		s_yscroll = regs[10];
		amo_set_xscroll(&s_mo, s_xscroll);
		amo_set_yscroll(&s_mo, s_yscroll);
	}
	return rc ? -1 : 0;
}

static int ep_save(void *ctx) { return ep_state(ctx, 0); }
static int ep_load(void *ctx) { return ep_state(ctx, 1); }
static int ep_supported(const char *machine)
{
	return !strcmp(machine, "eprom") || !strcmp(machine, "klaxp") || !strcmp(machine, "guts");
}

const rbx_driver rbx_drv_eprom = { "Atari Escape", ep_init, ep_exit, ep_reset, ep_frame, ep_save, ep_load,
                                   ep_supported, ep_render, NULL };
