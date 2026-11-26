/*  Bally Midway MCR/68k hardware, ported to C from MAME src/mame/bally/mcr68.cpp (BSD-3-Clause, copyright-holders:
    Bryan McPhail, Aaron Giles) and the ADC0844 of src/devices/machine/adc0844.cpp (BSD-3-Clause, copyright-holders:
    Dirk Best).
    Main CPU: 68000 at 7.7238 MHz (Musashi) with a 6840 PTM on its E clock (machine/ptm6840: counter 1 clocked by
    VSYNC, counter 3 by HSYNC; its interrupt on level 2) and the 493 interrupt (level 1) a fixed time before each
    VBLANK. Sound: Sounds Good (Xenophobe, Blasted), Sounds Good + Turbo Cheap Squeak (Spy
    Hunter II), the Williams D-11581 background board (Arch Rivals, Pigskin 621AD, Tri-Sports): midway/machine/
    midway_snd.
    Video: 32x32 tiles of 16x16 (8x8 doubled) with a priority bit, 32x32 sprites in two priority passes over and under
    the priority tiles (pens 0 and 8 clear, pen 8 masking the sprites under it), 512x480.
    Timing: MAME's 30 Hz frame of 480 lines run as two 240-line fields shown at 60 Hz (the games run at 30 Hz); the
    main CPU in slices up to the PTM's next expiry and the 493 interrupt's edges.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../rbx_core.h"
#include "../../rbx_prof.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../machine/ptm6840.h"
#include "../machine/midway_snd.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { MAP_MCR68, MAP_PIGSKIN, MAP_TRISPORT };
enum { G_XENOPHOB, G_BLASTED, G_INTLASER, G_SPYHUNT2, G_ARCHRIVL, G_ARCHRIVLB, G_PIGSKIN, G_TRISPORT };

typedef struct {
	const char *machine;   // "config:init"
	uint8_t g, map, snd;   // snd: MSND_B_*
	uint8_t bg_type;
	int8_t sprite_clip, sprite_xoffset;
	uint16_t timing_e;     // MAME's m_timing_factor in E clocks (the 493 comes that long before VBLANK)
	uint16_t watchdog;     // vblanks (30 Hz)
	float snd_gain;
} m68cfg_t;

static const m68cfg_t s_cfgs[] = {
	{ "xenophob:xenophob", G_XENOPHOB, MAP_MCR68, MSND_B_68K, 0, 0, -4, 256 + 16, 8, 1.0f },
	{ "xenophob:blasted",  G_BLASTED,  MAP_MCR68, MSND_B_68K, 0, 0, 0, 256 + 16, 8, 1.0f },
	// (International Team Laser, a prototype: waits forever for its task flag after the power-up; left out)
	{ "spyhunt2:spyhunt2", G_SPYHUNT2, MAP_MCR68, MSND_B_68K | MSND_B_TCS, 0, 0, -6, 256 + 16, 8, 0.5f },
	{ "archrivl:archrivl", G_ARCHRIVL, MAP_MCR68, MSND_B_BG, MSND_BG_S11, 16, 0, 256 + 16, 8, 1.0f },
	{ "archrivl:archrivlb", G_ARCHRIVLB, MAP_MCR68, MSND_B_BG, MSND_BG_S11, 16, 0, 256 + 16, 8, 1.0f },
	{ "pigskin:pigskin",   G_PIGSKIN,  MAP_PIGSKIN, MSND_B_BG, MSND_BG_S11C, 16, 0, 115, 8, 1.0f },
	{ "trisport:trisport", G_TRISPORT, MAP_TRISPORT, MSND_B_BG, MSND_BG_S11C, 0, 0, 115, 8, 1.0f },
};

#define MAIN_CLOCK 7723800
#define SCR_W 512
#define SCR_H 480
#define FRAME_LINES 480
#define FIELD_LINES 240
#define FRAME_CYC (MAIN_CLOCK / 30)   // (257460: the 30 Hz frame in CPU clocks)

static const m68cfg_t *s_c;

// ROM / decoded graphics
static uint8_t *s_rom;          // host-order words
static uint32_t s_rom_size;
static uint8_t *s_gfx1, *s_gfx2;
static uint32_t s_gfx1_size, s_gfx2_size;
static uint8_t *s_tiles;        // 8x8 pens a tile
static int s_ntiles;
static uint8_t *s_sprites;      // 32x32 pens a sprite
static int s_nsprites;
static uint8_t (*s_spr_rows)[32][2];

// memory (16-bit words in host order)
static uint16_t s_ram[0x2000], s_ram2[0x800], s_vram[0x800], s_spram[0x800], s_pal[0x40];
static uint32_t s_pal_rgb[64];
static int s_spram_words;

static ptm6840_t s_ptm;

// the MAME ports
enum { P_IN0, P_IN1, P_DSW, P_AN1, P_AN2, P_AN3, P_AN4, P_49X1, P_49Y1, P_49X2, P_49Y2, P_COUNT };
static const char *const s_port_tags[P_COUNT] = { "IN0", "IN1", "DSW", "AN1", "AN2", "AN3", "AN4", "49WAYX1",
                                                  "49WAYY1", "49WAYX2", "49WAYY2" };
static int s_port[P_COUNT];

typedef struct {
	int64_t now;              // main CPU clocks executed
	int64_t ptm_time;         // the PTM advanced up to here (CPU clocks)
	int64_t on493, off493;    // the 493's edges (CPU clocks, -1: none)
	int64_t frame0;           // the current 30 Hz frame's VSYNC
	int64_t snd_tgt[3];       // SG / TCS / BG targets (their clocks)
	uint64_t snd_frac[3];
	uint64_t frac;
	int32_t ptm_rem;          // CPU clocks not yet a whole E clock
	int32_t watchdog;
	uint16_t control_word, field, line, pad0_;
	uint8_t prot[5], adc_channel, adc_result, ptm_irq;
	uint8_t an[4], pad1_[4];
	int16_t x49[2], y49[2];
} m68state_t;
static m68state_t S;

static int s_snd;
static uint32_t s_snd_clock[3];
static int s_in_exec;
static char s_nv_path[512];

// render snapshot
typedef struct {
	uint16_t vram[0x800], spram[0x800];
	uint32_t pal[64];
	int valid;
} snap_t;
static snap_t *s_snap[2];
static int s_work;
static uint8_t *s_pri;

enum { M68_PA_MAIN = RBX_PA_DRV, M68_PA_SND68, M68_PA_SND2, M68_PA_MIX, M68_PA_END };
static const char *const s_pa_names[] = { "mcr68 main 68000", "mcr68 sg 68000", "mcr68 tcs/bg 6809", "mcr68 mix" };

static inline uint16_t rport16(int p) { return s_port[p] >= 0 ? (uint16_t)rbx_port_read(s_port[p]) : 0xffff; }
static inline int64_t main_time(void) { return S.now + (s_in_exec ? m68k_cycles_run() : 0); }

// ---- the PTM, on the E clock (CPU / 10) ----
static void ptm_sync(void)
{
	const int64_t now = main_time();
	if (now <= S.ptm_time)
		return;
	const int64_t total = now - S.ptm_time + S.ptm_rem;
	const int32_t e = (int32_t)(total / 10);
	S.ptm_rem = (int32_t)(total - (int64_t)e * 10);
	S.ptm_time = now;
	if (e)
		ptm6840_advance(&s_ptm, e);
}

static void ptm_irq(void *u, int state)
{
	(void)u;
	S.ptm_irq = (uint8_t)state;
	m68k_bus_set_line(2, state);
	if (s_in_exec)
		m68k_end_timeslice();
}

// ---- sound boards: brought up to the main CPU's time at an access ----
static int64_t s_main0, s_main1, s_snd0[3];

static void snd_sync_all(void)
{
	if (!s_snd)
		return;
	int64_t t = main_time();
	const int64_t span = s_main1 - s_main0;
	if (t > s_main1)
		t = s_main1;
	if (span <= 0 || t <= s_main0)
		return;
	const double f = (double)(t - s_main0) / (double)span;
	const unsigned pa = rbx_prof_enter(M68_PA_SND68);
	if (s_snd & MSND_B_68K)
		msnd_68k_run(s_snd0[0] + (int64_t)(f * (double)(S.snd_tgt[0] - s_snd0[0])));
	if (s_snd & MSND_B_TCS)
		msnd_tcs_run(s_snd0[1] + (int64_t)(f * (double)(S.snd_tgt[1] - s_snd0[1])));
	if (s_snd & MSND_B_BG)
		msnd_bg_run(s_snd0[2] + (int64_t)(f * (double)(S.snd_tgt[2] - s_snd0[2])));
	rbx_prof_leave(pa);
}

static void snd_line_begin(void)
{
	for (int i = 0; i < 3; i++)
		s_snd0[i] = S.snd_tgt[i];
	if (s_snd & MSND_B_68K)
		S.snd_tgt[0] += rbx_line_cycles(&S.snd_frac[0], s_snd_clock[0], 30000, FRAME_LINES);
	if (s_snd & MSND_B_TCS)
		S.snd_tgt[1] += rbx_line_cycles(&S.snd_frac[1], s_snd_clock[1], 30000, FRAME_LINES);
	if (s_snd & MSND_B_BG)
		S.snd_tgt[2] += rbx_line_cycles(&S.snd_frac[2], s_snd_clock[2], 30000, FRAME_LINES);
}

static void snd_line(void)
{
	if (s_snd & MSND_B_68K) {
		rbx_prof_set(M68_PA_SND68);
		msnd_68k_run(S.snd_tgt[0]);
	}
	if (s_snd & MSND_B_TCS) {
		rbx_prof_set(M68_PA_SND2);
		msnd_tcs_run(S.snd_tgt[1]);
	}
	if (s_snd & MSND_B_BG) {
		rbx_prof_set(M68_PA_SND2);
		msnd_bg_run(S.snd_tgt[2]);
	}
	rbx_prof_set(M68_PA_MIX);
	msnd_mix(s_snd);
}

// ---- inputs ----
static const uint8_t translate49[7] = { 0x7, 0x3, 0x1, 0x0, 0xc, 0xe, 0xf };

static uint8_t adc_clamp(int v) { return (uint8_t)(v < 0 ? 0 : v > 0xff ? 0xff : v); }
static uint8_t adc0844_convert(const uint8_t *an, int channel)
{
	switch (channel & 0x0f) {
	case 0x00: case 0x08: return adc_clamp(0xff - (an[1] - an[0]));
	case 0x01: case 0x09: return adc_clamp(0xff - (an[0] - an[1]));
	case 0x02: case 0x0a: return adc_clamp(0xff - (an[3] - an[2]));
	case 0x03: case 0x0b: return adc_clamp(0xff - (an[2] - an[3]));
	case 0x04: return an[0];
	case 0x05: return an[1];
	case 0x06: return an[2];
	case 0x07: return an[3];
	case 0x0c: return adc_clamp(0xff - (an[3] - an[0]));
	case 0x0d: return adc_clamp(0xff - (an[3] - an[1]));
	case 0x0e: return adc_clamp(0xff - (an[3] - an[2]));
	default: return 0;
	}
}

// the 49-way joystick (0x00..0x6f, centre 0x38): the stick's deflection in thirds, else the d-pad at full
static int way49(int v, int neg, int pos)
{
	int off;
	if (v) {
		const int m = v < 0 ? -v : v;
		off = m >= 7 ? 3 : m >= 4 ? 2 : 1;
		if (v < 0) off = -off;
	} else
		off = pos ? 3 : neg ? -3 : 0;
	return (3 + off) << 4;   // (the port's >> 4 picks the position: 0x00, 0x10 .. 0x60)
}

static void inputs_field(void)
{
	if (s_c->g == G_ARCHRIVL || s_c->g == G_PIGSKIN)
		for (int p = 0; p < 2; p++) {
			const uint32_t in = rbx_input(p + 1);
			S.x49[p] = (int16_t)way49(rbx_analog(p + 1, RBX_IPT_AD_STICK_X), (in & RBX_IN_LEFT) != 0,
			                          (in & RBX_IN_RIGHT) != 0);
			S.y49[p] = (int16_t)way49(rbx_analog(p + 1, RBX_IPT_AD_STICK_Y), (in & RBX_IN_UP) != 0,
			                          (in & RBX_IN_DOWN) != 0);
		}
	if (s_c->g == G_SPYHUNT2)
		// AN1 / AN2 = P2 / P1 pedal (0x30..0xff, reversed: released 0xff), AN3 / AN4 = P2 / P1 wheel (0x10..0xf0,
		// reversed, centre 0x80): the wheel on the stick / d-pad, the gas on the stick's up or button 6... (B5-B7
		// are the gears): the d-pad's up
		for (int p = 0; p < 2; p++) {
			const uint32_t in = rbx_input(p + 1);
			int ax = rbx_analog(p + 1, RBX_IPT_PADDLE);
			if (!ax) ax = ((in & RBX_IN_RIGHT) ? 8 : 0) - ((in & RBX_IN_LEFT) ? 8 : 0);
			if (ax > 8) ax = 8;
			if (ax < -8) ax = -8;
			int ped = -rbx_analog(p + 1, RBX_IPT_AD_STICK_Y);
			if (in & RBX_IN_UP) ped = 8;
			if (ped < 0) ped = 0;
			if (ped > 8) ped = 8;
			S.an[p ? 0 : 1] = (uint8_t)(0xff - ped * (0xff - 0x30) / 8);
			S.an[p ? 2 : 3] = (uint8_t)(0x80 - ax * 0x70 / 8);
		}
	if (s_c->g == G_TRISPORT)
		// AN1 / AN2: the trackball (relative, signed)
		for (int i = 0; i < 2; i++)
			S.an[i] = (uint8_t)(S.an[i] + rbx_analog_dpad(1, i ? RBX_IPT_TRACKBALL_Y : RBX_IPT_TRACKBALL_X, 4));
}

static uint16_t in_read(int which)   // which: 0 = IN0, 1 = IN1 / port 1, 2 = DSW / port 2
{
	switch (s_c->g) {
	case G_SPYHUNT2:
		if (which == 0) {
			snd_sync_all();
			return (uint16_t)((rport16(P_IN0) & 0x00df) | ((msnd_68k_read() & 1) << 5) | (S.adc_result << 8));
		}
		if (which == 1) {
			snd_sync_all();
			return (uint16_t)((rport16(P_IN1) & 0xff7f) | ((msnd_tcs_read() & 1) << 7));
		}
		break;
	case G_ARCHRIVL:
		if (which == 1)
			return (uint16_t)(translate49[S.y49[1] >> 4] << 12 | translate49[S.x49[1] >> 4] << 8 |
			                  translate49[S.y49[0] >> 4] << 4 | translate49[S.x49[0] >> 4]);
		break;
	case G_PIGSKIN:
		if (which == 1)
			return (uint16_t)((rport16(P_IN1) & 0x00ff) | translate49[S.x49[0] >> 4] << 12 |
			                  translate49[S.y49[0] >> 4] << 8);
		if (which == 2)
			return (uint16_t)((rport16(P_DSW) & 0x00ff) | translate49[S.x49[1] >> 4] << 12 |
			                  translate49[S.y49[1] >> 4] << 8);
		break;
	case G_TRISPORT:
		if (which == 1) {
			const int x = (int8_t)S.an[0], y = (int8_t)S.an[1];
			return (uint16_t)((rport16(P_IN1) & ~0xff00) | ((x & 0x3c) << 6) | ((y & 0x3c) << 10));
		}
		break;
	default: break;
	}
	return rport16(which == 0 ? P_IN0 : which == 1 ? P_IN1 : P_DSW);
}

// ---- control writes ----
static void control_w(uint16_t data, uint16_t mask)
{
	S.control_word = (uint16_t)((S.control_word & ~mask) | (data & mask));
	const uint16_t cw = S.control_word;
	snd_sync_all();
	switch (s_c->g) {
	case G_XENOPHOB:
		msnd_68k_write((uint8_t)(((cw & 0x000f) << 1) | ((cw & 0x0010) >> 4)));
		break;
	case G_BLASTED: case G_INTLASER:
		msnd_68k_write((uint8_t)((cw >> 8) & 0x1f));
		break;
	case G_SPYHUNT2:
		msnd_tcs_write((uint8_t)((cw >> 8) & 0x1f));
		msnd_68k_reset_write(!(cw & 0x2000));
		msnd_68k_write((uint8_t)((cw >> 8) & 0x1f));
		S.adc_channel = (uint8_t)((cw >> 3) & 0x0f);
		S.adc_result = adc0844_convert(S.an, S.adc_channel);
		break;
	default:   // archrivl_control_w (Arch Rivals, Pigskin, Tri-Sports)
		msnd_bg_resetq_w((cw >> 10) & 1);
		msnd_bg_data_w((uint8_t)cw);
		msnd_bg_ctrl_w((cw >> 8) & 1);
		msnd_bg_extra_w((cw >> 9) & 1);
		break;
	}
}

static uint16_t pigskin_protection_r(void)
{
	const uint8_t *p = S.prot;
	if (p[4] == 0xe3 && p[3] == 0x94) return 0x00;
	if (p[4] == 0xc7 && p[3] == 0x7b && p[2] == 0x36) return 0x00;
	if (p[4] == 0xc7 && p[3] == 0x7b) return 0x07;
	if (p[4] == 0xc7 && p[3] == 0x1f && p[2] == 0x03 && p[1] == 0x25 && p[0] == 0x36) return 0x00;
	return 0x00;
}

static void pigskin_protection_w(uint16_t data, uint16_t mask)
{
	if (mask & 0x00ff) {
		memmove(S.prot, S.prot + 1, 4);
		S.prot[4] = (uint8_t)data;
	}
}

// ---- palette (xRBG_333) ----
static void pal_w(int index, uint16_t data)
{
	static const uint8_t pal3[8] = { 0x00, 0x24, 0x49, 0x6d, 0x92, 0xb6, 0xdb, 0xff };
	s_pal[index] = data;
	s_pal_rgb[index] = (uint32_t)pal3[(data >> 6) & 7] << 16 | (uint32_t)pal3[data & 7] << 8 | pal3[(data >> 3) & 7];
}

// ---- main CPU bus: the handlers (the page table holds the ROM and the RAMs) ----
static void watchdog_kick(void) { S.watchdog = 0; }

static uint16_t ptm_rd16(uint32_t a, int hi)
{
	ptm_sync();
	const uint8_t v = ptm6840_read(&s_ptm, (a >> 1) & 7);
	return hi ? (uint16_t)(v << 8 | 0xff) : (uint16_t)(v << 8 | v);   // (MCR-68: cswidth 16, the byte on both lanes)
}

static uint16_t main_read16(uint32_t a)
{
	a &= 0x1fffff;
	switch (s_c->map) {
	case MAP_MCR68:
		if (a >= 0x0a0000 && a < 0x0a0010) return ptm_rd16(a, 0);
		if ((a >> 16) == 0x0d) return in_read(0);
		if ((a >> 16) == 0x0e) return in_read(1);
		if ((a >> 16) == 0x0f) return in_read(2);
		break;
	case MAP_PIGSKIN:
		if ((a >> 16) == 0x08) return in_read(1);
		if ((a >> 16) == 0x0a) return in_read(2);
		if (a >= 0x120000 && a < 0x120002) return pigskin_protection_r();
		if (a >= 0x180000 && a < 0x180010) return ptm_rd16(a, 1);
		if ((a >> 16) == 0x1e) return in_read(0);
		break;
	default:   // MAP_TRISPORT
		if ((a >> 16) == 0x08) return in_read(1);
		if ((a >> 16) == 0x0a) return in_read(2);
		if (a >= 0x180000 && a < 0x180010) return ptm_rd16(a, 1);
		if ((a >> 16) == 0x1e) return in_read(0);
		break;
	}
	return 0xffff;
}

static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (a & 1) ? (uint8_t)v : (uint8_t)(v >> 8);
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0x1fffff;
	switch (s_c->map) {
	case MAP_MCR68:
		if (a >= 0x090000 && a < 0x090080) {
			const int i = (a >> 1) & 0x3f;
			pal_w(i, (uint16_t)((s_pal[i] & ~mask) | (v & mask)));
			return;
		}
		if (a >= 0x0a0000 && a < 0x0a0010) {
			// (umask 0x00ff with cswidth 16: the chip is selected by either byte; the 68000 puts a byte on both lanes)
			ptm_sync();
			ptm6840_write(&s_ptm, (a >> 1) & 7, (uint8_t)(mask == 0xff00 ? v >> 8 : v));
			return;
		}
		if ((a >> 16) == 0x0b) { watchdog_kick(); return; }
		if ((a >> 16) == 0x0c) { control_w(v, mask); return; }
		return;
	case MAP_PIGSKIN:
		if (a >= 0x0c0000 && a < 0x0c0080) {
			const int i = (a >> 1) & 0x3f;
			pal_w(i, (uint16_t)((s_pal[i] & ~mask) | (v & mask)));
			return;
		}
		if ((a >> 16) == 0x0e) { watchdog_kick(); return; }
		if (a >= 0x120000 && a < 0x120002) { pigskin_protection_w(v, mask); return; }
		if (a >= 0x180000 && a < 0x180010) {
			if (mask & 0xff00) {
				ptm_sync();
				ptm6840_write(&s_ptm, (a >> 1) & 7, (uint8_t)(v >> 8));
			}
			return;
		}
		if ((a >> 16) == 0x1a) { control_w(v, mask); return; }
		return;
	default:   // MAP_TRISPORT
		if (a >= 0x120000 && a < 0x120080) {
			const int i = (a >> 1) & 0x3f;
			pal_w(i, (uint16_t)((s_pal[i] & ~mask) | (v & mask)));
			return;
		}
		if (a >= 0x180000 && a < 0x180010) {
			if (mask & 0xff00) {
				ptm_sync();
				ptm6840_write(&s_ptm, (a >> 1) & 7, (uint8_t)(v >> 8));
			}
			return;
		}
		if ((a >> 16) == 0x1a) { control_w(v, mask); return; }
		if ((a >> 16) == 0x1c) { watchdog_kick(); return; }
		return;
	}
}

static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	if (a & 1)
		main_write16m(a & ~1u, v, 0x00ff);
	else
		main_write16m(a, (uint16_t)(v << 8), 0xff00);
}

static void map_main(void)
{
	m68k_bus_reset_map();
	// (global mask 0x1fffff: the 2 MB mirrored up to 16 MB)
	for (uint32_t base = 0; base < 0x1000000; base += 0x200000) {
		m68k_bus_map(base, base + 0x03ffff, s_rom, s_rom_size, 1, 0);
		switch (s_c->map) {
		case MAP_MCR68:
			m68k_bus_map(base + 0x060000, base + 0x063fff, s_ram, 0x4000, 1, 1);
			m68k_bus_map(base + 0x070000, base + 0x070fff, s_vram, 0x1000, 1, 1);
			m68k_bus_map(base + 0x071000, base + 0x071fff, s_ram2, 0x1000, 1, 1);
			m68k_bus_map(base + 0x080000, base + 0x080fff, s_spram, 0x1000, 1, 1);
			break;
		case MAP_PIGSKIN:
			m68k_bus_map(base + 0x100000, base + 0x100fff, s_vram, 0x1000, 1, 1);
			m68k_bus_map(base + 0x140000, base + 0x143fff, s_ram, 0x4000, 1, 1);
			m68k_bus_map(base + 0x160000, base + 0x1607ff, s_spram, 0x800, 1, 1);
			break;
		default:
			m68k_bus_map(base + 0x100000, base + 0x103fff, s_ram, 0x4000, 1, 1);   // (NVRAM)
			m68k_bus_map(base + 0x140000, base + 0x1407ff, s_spram, 0x800, 1, 1);
			m68k_bus_map(base + 0x160000, base + 0x160fff, s_vram, 0x1000, 1, 1);
			break;
		}
	}
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
}

// ---- graphics ----
static void decode_gfx(void)
{
	const uint32_t half = s_gfx1_size / 2;
	s_ntiles = (int)(half / 16);
	for (int t = 0; t < s_ntiles; t++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				const uint32_t bit = (uint32_t)t * 128 + y * 16 + x * 2;
				const uint8_t hi = s_gfx1[half + (bit >> 3)], lo = s_gfx1[bit >> 3];
				const int sh = 6 - (bit & 7);
				s_tiles[t * 64 + y * 8 + x] = (uint8_t)((((hi >> sh) & 3) << 2) | ((lo >> sh) & 3));
			}
	const uint32_t q = s_gfx2_size / 4;
	s_nsprites = (int)(q / 128);
	for (int s = 0; s < s_nsprites; s++)
		for (int y = 0; y < 32; y++) {
			int first = 32, last = 0;
			for (int x = 0; x < 32; x++) {
				const uint8_t b = s_gfx2[((x & 7) >> 1) * q + (uint32_t)s * 128 + y * 4 + (x >> 3)];
				const uint8_t pen = (x & 1) ? (b & 15) : (b >> 4);
				s_sprites[s * 1024 + y * 32 + x] = pen;
				if (pen) {
					if (x < first) first = x;
					last = x + 1;
				}
			}
			s_spr_rows[s][y][0] = (uint8_t)(first < 32 ? first : 0);
			s_spr_rows[s][y][1] = (uint8_t)last;
		}
}

// the tilemap: all tiles opaque (pass 0), or the priority tiles' opaque pens (pass 1)
static void draw_tiles(const snap_t *sn, uint16_t *bm, int pitch, int pass)
{
	const int has_cat = s_ntiles < 0x1000;
	for (int ty = 0; ty < 30; ty++)
		for (int tx = 0; tx < 32; tx++) {
			const int idx = ty * 32 + tx;
			const int data = (sn->vram[idx * 2] & 0xff) | ((sn->vram[idx * 2 + 1] & 0xff) << 8);
			if (pass && !(has_cat && (data >> 15)))
				continue;
			const int code = (int)rbx_wrap((uint32_t)((data & 0x3ff) | ((data >> 4) & 0xc00)), (uint32_t)s_ntiles);
			const int color = (~data >> 12) & 3;
			const int fx = (data >> 10) & 1, fy = (data >> 11) & 1;
			const uint8_t *t = s_tiles + code * 64;
			const uint16_t base = (uint16_t)(color * 16);
			for (int y = 0; y < 8; y++) {
				const uint8_t *row = t + (fy ? 7 - y : y) * 8;
				uint16_t *d = bm + (ty * 16 + y * 2) * pitch + tx * 16;
				if (!pass) {
					for (int x = 0; x < 8; x++) {
						const uint16_t c = (uint16_t)(base + row[fx ? 7 - x : x]);
						d[2 * x] = c;
						d[2 * x + 1] = c;
					}
					memcpy(d + pitch, d, 16 * sizeof(uint16_t));
				} else
					for (int x = 0; x < 8; x++) {
						const uint8_t p = row[fx ? 7 - x : x];
						if (p) {
							const uint16_t c = (uint16_t)(base + p);
							d[2 * x] = d[2 * x + 1] = d[pitch + 2 * x] = d[pitch + 2 * x + 1] = c;
						}
					}
			}
		}
}

static void draw_sprites(const snap_t *sn, uint16_t *bm, int pitch, int priority)
{
	const int x0c = s_c->sprite_clip, x1c = SCR_W - s_c->sprite_clip;
	for (int y = 0; y < SCR_H; y++)
		memset(s_pri + y * SCR_W, 1, SCR_W);
	for (int offs = s_spram_words - 4; offs >= 0; offs -= 4) {
		const int flags = sn->spram[offs + 1] & 0xff;
		const int raw = (sn->spram[offs + 2] & 0xff) + 256 * ((flags >> 3) & 1) + 512 * ((flags >> 6) & 3);
		if (raw == 0 || ((flags >> 2) & 1) != priority)
			continue;
		const int code = (int)rbx_wrap((uint32_t)raw, (uint32_t)s_nsprites);
		const int base = (~flags & 3) * 16;
		const int hflip = (flags & 0x10) ? 31 : 0, vflip = (flags & 0x20) ? 31 : 0;
		int sx = (sn->spram[offs + 3] & 0xff) * 2 + s_c->sprite_xoffset;
		const int sy = (241 - (sn->spram[offs] & 0xff)) * 2;
		if (sx > 0x1f0)
			sx -= 0x200;
		const uint8_t *gfx = s_sprites + code * 1024;
		for (int y = 0; y < 32; y++) {
			const int py = sy + y;
			if (py < 0 || py >= SCR_H)
				continue;
			const int r = y ^ vflip;
			const uint8_t first = s_spr_rows[code][r][0], last = s_spr_rows[code][r][1];
			if (last <= first)
				continue;
			const uint8_t *src = gfx + r * 32;
			uint16_t *dst = bm + py * pitch;
			uint8_t *pri = s_pri + py * SCR_W;
			const int a = hflip ? 31 - (last - 1) : first, b = hflip ? 31 - first + 1 : last;
			for (int x = a; x < b; x++) {
				const int px = sx + x;
				if (px < x0c || px >= x1c)
					continue;
				const uint8_t p = src[x ^ hflip];
				if (p & 7) {
					dst[px] = (uint16_t)(base + p);
					pri[px] = 31;
				}
			}
			for (int x = a; x < b; x++) {
				const int px = sx + x;
				if (px < x0c || px >= x1c)
					continue;
				if (src[x ^ hflip] == 8) {
					if (pri[px] != 1)
						dst[px] = (uint16_t)(base + 8);
					pri[px] = 31;
				}
			}
		}
	}
}

static void m68_render(void)
{
	const snap_t *sn = s_snap[s_work ^ 1];
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	if (!sn->valid) {
		for (int y = 0; y < SCR_H; y++)
			memset(bm + y * pitch, 0, SCR_W * sizeof(uint16_t));
		return;
	}
	memcpy(rbx_video.palette, sn->pal, sizeof(sn->pal));
	draw_tiles(sn, bm, pitch, 0);
	draw_sprites(sn, bm, pitch, 0);
	draw_tiles(sn, bm, pitch, 1);
	draw_sprites(sn, bm, pitch, 1);
}

static void present(void)
{
	rbx_video_wait_idle();
	snap_t *sn = s_snap[s_work];
	memcpy(sn->vram, s_vram, sizeof(s_vram));
	memcpy(sn->spram, s_spram, sizeof(s_spram));
	memcpy(sn->pal, s_pal_rgb, sizeof(sn->pal));
	sn->valid = 1;
	s_work ^= 1;
	rbx_video_frame_done();
}

// ---- the main CPU's run: slices up to the PTM's next expiry and the 493's edges ----
static void run_main(int64_t end)
{
	while (S.now < end) {
		ptm_sync();
		int64_t stop = end;
		const int32_t nx = ptm6840_next(&s_ptm);
		if (nx != INT32_MAX) {
			const int64_t t = S.ptm_time + (int64_t)nx * 10 - S.ptm_rem;
			if (t < stop)
				stop = t;
		}
		if (S.on493 >= 0 && S.on493 < stop)
			stop = S.on493;
		if (S.off493 >= 0 && S.off493 < stop)
			stop = S.off493;
		if (stop <= S.now)
			stop = S.now + 1;
		s_in_exec = 1;
#ifdef RBX_M68K_TRACE
		{
			extern long long rbx_m68k_trace_base;
			rbx_m68k_trace_base = S.now;
		}
#endif
		const int used = m68k_execute((int)(stop - S.now));
		s_in_exec = 0;
		S.now += used;
		ptm_sync();
		if (S.on493 >= 0 && S.now >= S.on493) {   // (mcr68_493_callback: IRQ 1 for a scanline)
			S.on493 = -1;
			m68k_bus_set_line(1, 1);
			S.off493 = S.now + MAIN_CLOCK / (30 * FRAME_LINES);
		}
		if (S.off493 >= 0 && S.now >= S.off493) {
			S.off493 = -1;
			m68k_bus_set_line(1, 0);
		}
	}
}

// ---- init / reset / frame ----
static const m68cfg_t *find_cfg(const char *machine)
{
	for (size_t i = 0; i < sizeof(s_cfgs) / sizeof(s_cfgs[0]); i++)
		if (!strcmp(s_cfgs[i].machine, machine))
			return &s_cfgs[i];
	return NULL;
}

static void soft_reset(void)
{
	m68k_bus_irq_reset();
	m68k_bus_set_line(1, 0);
	m68k_bus_set_line(2, 0);
	ptm6840_reset(&s_ptm);
	S.on493 = S.off493 = -1;
	S.watchdog = 0;
	m68k_pulse_reset();
	if (s_snd & MSND_B_68K) {
		msnd_68k_reset_write(1);
		msnd_68k_reset_write(0);
	}
	if (s_snd & MSND_B_TCS) {
		msnd_tcs_reset_write(1);
		msnd_tcs_reset_write(0);
	}
	if (s_snd & MSND_B_BG) {
		msnd_bg_resetq_w(1);
		msnd_bg_resetq_w(0);
	}
}

static void m68_reset(void)
{
	rbx_video_wait_idle();
	memset(&S, 0, sizeof(S));
	if (s_c->map != MAP_TRISPORT)
		memset(s_ram, 0, sizeof(s_ram));
	memset(s_ram2, 0, sizeof(s_ram2));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_spram, 0, sizeof(s_spram));
	for (int i = 0; i < 64; i++)
		pal_w(i, 0);
	S.on493 = S.off493 = -1;
	S.an[0] = S.an[1] = 0xff;
	S.an[2] = S.an[3] = 0x80;
	if (s_c->g == G_TRISPORT)
		S.an[0] = S.an[1] = 0;
	for (int p = 0; p < 2; p++)
		S.x49[p] = S.y49[p] = 0x30;
	memset(&s_ptm, 0, PTM6840_STATE_OFFSET);
	s_ptm.irq = ptm_irq;
	ptm6840_reset(&s_ptm);
	m68k_bus_irq_reset();
	map_main();
	m68k_pulse_reset();
	if (s_snd & MSND_B_68K)
		msnd_68k_reset();
	if (s_snd & MSND_B_TCS)
		msnd_tcs_reset();
	if (s_snd & MSND_B_BG)
		msnd_bg_reset();
	msnd_mix_reset();
	for (int i = 0; i < 2; i++)
		s_snap[i]->valid = 0;
}

static void m68_exit(void);

static int m68_init(const rbx_game *g)
{
	s_c = find_cfg(g->machine);
	if (!s_c) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	uint32_t rsize = 0, ssize = 0;
	uint8_t *rom = rbx_region("maincpu", &rsize);
	s_gfx1 = rbx_region("gfx1", &s_gfx1_size);
	s_gfx2 = rbx_region("gfx2", &s_gfx2_size);
	if (!rom || rsize < 0x40000 || !s_gfx1 || !s_gfx2) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_rom = rom;
	s_rom_size = 0x40000;
	m68k_bus_swap16(s_rom, s_rom_size);
	s_snd = s_c->snd;
	if (s_snd & MSND_B_68K) {
		uint8_t *r = rbx_region("sg:cpu", &ssize);
		if (!r || msnd_68k_init(MSND_68K_SG, r, ssize, s_c->snd_gain)) {
			rbx_fail("%s: Sounds Good ROM missing", g->set);
			s_snd = 0;
			return -1;
		}
		s_snd_clock[0] = 8000000;
	}
	if (s_snd & MSND_B_TCS) {
		uint8_t *r = rbx_region("tcs:cpu", &ssize);
		if (!r || msnd_tcs_init(r, ssize, s_c->snd_gain)) {
			rbx_fail("%s: Turbo Cheap Squeak ROM missing", g->set);
			m68_exit();
			return -1;
		}
		s_snd_clock[1] = 2000000;
	}
	if (s_snd & MSND_B_BG) {
		uint8_t *r = rbx_region("bg:cpu", &ssize);
		if (!r || msnd_bg_init(s_c->bg_type, r, ssize, s_c->snd_gain)) {
			rbx_fail("%s: sound board ROM missing", g->set);
			s_snd = 0;
			return -1;
		}
		s_snd_clock[2] = 2000000;
	}
	s_tiles = (uint8_t *)malloc(s_gfx1_size / 32 * 64);
	s_sprites = (uint8_t *)malloc(s_gfx2_size / 512 * 1024);
	s_spr_rows = calloc(s_gfx2_size / 512, sizeof(*s_spr_rows));
	s_snap[0] = (snap_t *)calloc(1, sizeof(snap_t));
	s_snap[1] = (snap_t *)calloc(1, sizeof(snap_t));
	s_pri = (uint8_t *)calloc(1, SCR_W * SCR_H);
	if (!s_tiles || !s_sprites || !s_spr_rows || !s_snap[0] || !s_snap[1] || !s_pri) {
		rbx_fail("%s: out of memory", g->set);
		m68_exit();
		return -1;
	}
	decode_gfx();
	s_spram_words = s_c->map == MAP_MCR68 ? 0x800 : 0x400;
	s_work = 0;
	for (int i = 0; i < P_COUNT; i++)
		s_port[i] = rbx_port_find(s_port_tags[i]);
	// Tri-Sports' NVRAM (MAME default all 0)
	memset(s_ram, 0, sizeof(s_ram));
	s_nv_path[0] = 0;
	if (s_c->map == MAP_TRISPORT) {
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			FILE *f = fopen(s_nv_path, "rb");
			if (f) {
				if (fread(s_ram, 1, sizeof(s_ram), f) != sizeof(s_ram))
					memset(s_ram, 0, sizeof(s_ram));
				fclose(f);
			}
		}
	}
	if (rbx_audio_setup_stereo(MSND_RATE) || rbx_video_setup(SCR_W, SCR_H, 64)) {
		rbx_fail("%s: out of memory", g->set);
		m68_exit();
		return -1;
	}
	rbx_prof_name(RBX_PA_DRV, s_pa_names, M68_PA_END - RBX_PA_DRV);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);
	m68_reset();
	return 0;
}

static void m68_exit(void)
{
	rbx_video_wait_idle();
	if (s_nv_path[0]) {
		rbx_file_put_now(s_nv_path, s_ram, sizeof(s_ram));
		s_nv_path[0] = 0;
	}
	m68k_set_mame_timing(0);
	if (s_snd & MSND_B_68K)
		msnd_68k_exit();
	if (s_snd & MSND_B_BG)
		msnd_bg_exit();
	s_snd = 0;
	free(s_tiles);
	free(s_sprites);
	free(s_spr_rows);
	free(s_snap[0]);
	free(s_snap[1]);
	free(s_pri);
	s_tiles = s_sprites = s_pri = NULL;
	s_spr_rows = NULL;
	s_snap[0] = s_snap[1] = NULL;
	s_c = NULL;
}

static void m68_frame(void)
{
	rbx_audio.frame_mhz = 60000;
	inputs_field();
	for (int line = 0; line < FIELD_LINES; line++) {
		const int fline = (S.field & 1) * FIELD_LINES + line;   // (the line of the 30 Hz frame)
		S.line = (uint16_t)fline;
		ptm_sync();
		if (fline == 0) {   // VSYNC: PTM C1, the 493 a timing factor before the next VBLANK, the watchdog
			ptm6840_set_clock(&s_ptm, 0, 0);
			ptm6840_set_clock(&s_ptm, 0, 1);
			S.frame0 = S.now;
			S.on493 = S.now + FRAME_CYC - (int64_t)s_c->timing_e * 10;
			if (++S.watchdog >= s_c->watchdog)
				soft_reset();
		}
		ptm6840_set_clock(&s_ptm, 2, 0);   // HSYNC: PTM C3
		ptm6840_set_clock(&s_ptm, 2, 1);
		rbx_prof_set(M68_PA_MAIN);
		s_main0 = S.now;
		const int64_t end = S.now + rbx_line_cycles(&S.frac, MAIN_CLOCK, 30000, FRAME_LINES);
		s_main1 = end;
		snd_line_begin();
		run_main(end);
		snd_line();
	}
	S.field++;
	present();
	rbx_prof_set(RBX_PA_DRIVER);
}

// ---- save states ----
static int m68_state(void *ctx, int load)
{
	int rc = 0;
	uint8_t cpu[1024];
	const uint32_t csize = m68k_context_size();
	if (csize > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	rc |= rbx_state_io(ctx, "m68_cpu", cpu, csize);
	rc |= rbx_state_io(ctx, "m68_irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "m68", &S, sizeof(S));
	rc |= rbx_state_io(ctx, "m68_ptm", (uint8_t *)&s_ptm + PTM6840_STATE_OFFSET, (uint32_t)PTM6840_STATE_SIZE);
	rc |= rbx_state_io(ctx, "m68_ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "m68_ram2", s_ram2, sizeof(s_ram2));
	rc |= rbx_state_io(ctx, "m68_vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "m68_spram", s_spram, sizeof(s_spram));
	rc |= rbx_state_io(ctx, "m68_pal", s_pal, sizeof(s_pal));
	rc |= msnd_state(s_snd, ctx, load);
	if (rc)
		return -1;
	if (load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		for (int i = 0; i < 64; i++)
			pal_w(i, s_pal[i]);
	}
	return 0;
}

static int m68_save(void *ctx) { return m68_state(ctx, 0); }
static int m68_load(void *ctx) { return m68_state(ctx, 1); }
static int m68_supported(const char *machine) { return find_cfg(machine) != NULL; }

const rbx_driver rbx_drv_mcr68 = { "Midway MCR-68k", m68_init, m68_exit, m68_reset, m68_frame, m68_save, m68_load,
                                   m68_supported, m68_render, NULL };
