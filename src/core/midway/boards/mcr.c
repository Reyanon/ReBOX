/*  Bally Midway MCR (MCR-I / MCR-II) and MCR-3 hardware, ported to C from MAME src/mame/bally/mcr.cpp, mcr.h,
    mcr_m.cpp, mcr_v.cpp, mcr3.cpp, mcr3.h, mcr3_v.cpp (BSD-3-Clause, copyright-holders: Aaron Giles; driver by
    Christopher Kirmse, Aaron Giles), the watchdog's vblank count of src/devices/machine/watchdog.cpp and the ADC0804 /
    ADC0844 of src/devices/machine/adc0804.cpp, adc0844.cpp (BSD-3-Clause, copyright-holders: Aaron Giles, Dirk Best).
    Main CPU: Z80 (2.496 MHz on the 90009 / 90010 / 91475 boards, 5 MHz on the 91490 board and the MCR-3 boards) with
    a Z80 CTC on its daisy chain (machine/z80ctc; channel 0's ZC/TO clocks channel 1, VBLANK channel 2, the 30 Hz
    frame channel 3). Sound: the boards of midway/machine/midway_snd (SSIO, Turbo Cheap Squeak, Sounds Good, Cheap
    Squeak Deluxe, Squawk & Talk).
    Video: 32x30 tiles of 16x16 (8x8 doubled), 32x32 sprites on the 91399 (ORed line buffer) or 91464 (first sprite
    wins) video boards, 512x480. MCR-3 scrolling boards (Spy Hunter, Crater Raider): a 64x32 scrolled tile layer of
    32x32 tiles, the sprites and a 16x16 alpha layer.
    Timing: MAME's 30 Hz interlaced frame of 480 lines is run as two 240-line fields (VBLANK = CTC trigger 2 at each
    field's start, the 30 Hz trigger 3 at every other one), each shown when it ends: 60 frames a second, the CPUs a
    scanline at a time, the main CPU in slices up to its CTC's next timer expiry.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../rbx_core.h"
#include "../../rbx_prof.h"
#include "../../cpu/z80/z80.h"
#include "../../machine/z80ctc.h"
#include "../machine/midway_snd.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ---- machine configurations ----
enum { CPU_90009, CPU_90010, CPU_91475, CPU_91490, CPU_MONO, CPU_SCROLL };
enum { SPR_91399, SPR_91464 };
// the init function (MAME's init_*): each game's custom I/O
enum { G_PLAIN, G_SOLARFOX, G_KICK, G_DPOKER, G_WACKO, G_TWOTIGER, G_KROOZR, G_JOURNEY, G_DOTRONE, G_DEMODERB,
       G_DOTRON, G_TRON, G_TWOTIGERC,
       G_DEMODERM, G_SARGE, G_MAXRPM, G_RAMPAGE, G_POWERDRV, G_STARGRDS, G_SPYHUNT, G_CRATER, G_TURBOTAG };

typedef struct {
	const char *machine;
	uint8_t cpu, spr, snd;   // snd: MSND_B_* boards
	uint8_t pal;             // palette entries / 64
	uint32_t clock;
	uint8_t nvram_fill;
} mcfg_t;

static const mcfg_t s_cfgs[] = {
	{ "mcr_90009",     CPU_90009,  SPR_91399, MSND_B_SSIO,              1, 2496000, 0xff },
	{ "mcr_90009_dp",  CPU_90009,  SPR_91399, MSND_B_SSIO,              1, 2496000, 0xff },
	{ "mcr_90010",     CPU_90010,  SPR_91399, MSND_B_SSIO,              1, 2496000, 0xff },
	{ "mcr_90010_tt",  CPU_90010,  SPR_91399, MSND_B_SSIO,              1, 2496000, 0xff },
	{ "mcr_91475",     CPU_91475,  SPR_91464, MSND_B_SSIO,              2, 2496000, 0xff },
	{ "mcr_91490",     CPU_91490,  SPR_91464, MSND_B_SSIO,              1, 5000000, 0xff },
	{ "mcr_91490_snt", CPU_91490,  SPR_91464, MSND_B_SSIO | MSND_B_SNT, 1, 5000000, 0xff },
	{ "mcr_91490_tcs", CPU_91490,  SPR_91464, MSND_B_SSIO | MSND_B_TCS, 1, 5000000, 0xff },
	// MCR-3
	{ "mono_tcs",      CPU_MONO,   SPR_91464, MSND_B_TCS,               1, 5000000, 0x00 },
	{ "maxrpm",        CPU_MONO,   SPR_91464, MSND_B_TCS,               1, 5000000, 0x00 },
	{ "mono_sg",       CPU_MONO,   SPR_91464, MSND_B_68K,               1, 5000000, 0x00 },
	{ "mcrscroll",     CPU_SCROLL, SPR_91464, MSND_B_SSIO,              2, 5000000, 0x00 },
	{ "spyhunt",       CPU_SCROLL, SPR_91464, MSND_B_SSIO | MSND_B_68K, 2, 5000000, 0x00 },
	{ "turbotag",      CPU_SCROLL, SPR_91464, MSND_B_SSIO | MSND_B_68K, 2, 5000000, 0x00 },
};

static const struct { const char *name; uint8_t id; } s_inits[] = {
	{ "solarfox", G_SOLARFOX }, { "kick", G_KICK }, { "dpoker", G_DPOKER }, { "mcr_90010", G_PLAIN },
	{ "wacko", G_WACKO }, { "twotiger", G_TWOTIGER }, { "kroozr", G_KROOZR }, { "journey", G_JOURNEY },
	{ "mcr_91490", G_PLAIN }, { "dotrone", G_DOTRONE }, { "demoderb", G_DEMODERB },
	{ "demoderm", G_DEMODERM }, { "sarge", G_SARGE }, { "maxrpm", G_MAXRPM }, { "rampage", G_RAMPAGE },
	{ "powerdrv", G_POWERDRV }, { "stargrds", G_STARGRDS }, { "spyhunt", G_SPYHUNT }, { "crater", G_CRATER },
	{ "turbotag", G_TURBOTAG },
};

#define SCR_W 512
#define SCR_H 480
#define FIELD_LINES 240
#define MAX_PAL 128

static const mcfg_t *s_c;
static int s_g;              // G_*
static int s_mirror;         // 1: mirror x, 2: mirror y (the orientations that are not a rotation)
static int s_vis_w, s_vis_h;
static int s_mcr3;

// ROM
static uint8_t *s_mrom, *s_gfx1, *s_gfx2, *s_gfx3;
static uint32_t s_mrom_size, s_gfx1_size, s_gfx2_size, s_gfx3_size;

// decoded graphics
static uint8_t *s_tiles;     // 8x8 (MCR) or 32x32 (scrolling MCR-3) pens a tile
static int s_ntiles;
static uint8_t *s_sprites;   // 32x32 pens a sprite
static int s_nsprites;
static uint8_t (*s_spr_rows)[32][2];   // each sprite row's first and last+1 opaque column (0, 0: empty)
static uint8_t *s_alpha;     // Spy Hunter's alpha characters: 8x8 pens (drawn doubled)
static int s_nalpha;
static uint8_t *s_alpha_blank;   // 1: the character has no opaque pen (skipped)

// CPUs and chips
static z80 s_main;
static z80ctc_t s_ctc;

// memory
static uint8_t s_nvram[0x800], s_spriteram[0x200], s_videoram[0x800], s_paletteram[0x80], s_alpharam[0x800];
static uint8_t s_hiram[0x200];   // (Draw Poker's meter RAM; the monoboard's 0xea00-0xebff)
static uint16_t s_pal_raw[MAX_PAL];   // 9-bit palette entries (xRBG 444 on the 90009)
static uint32_t s_pal_rgb[MAX_PAL];
static const uint8_t *s_rmap[256];

// the MAME ports
enum { P_IP0, P_IP1, P_IP2, P_IP3, P_IP4, P_DIP, P_IP1ALT, P_IP2ALT, P_IP1A1, P_IP1A2, P_IP2A1, P_IP2A2, P_DIAL,
       P_DIAL2, P_P24, P_P28, P_P2C, P_IP1A3, P_SHIFT, P_IP0ALT, P_COUNT };
static int s_port[P_COUNT];
static const char *const s_tags_mcr[P_COUNT] = {
	"ssio:IP0", "ssio:IP1", "ssio:IP2", "ssio:IP3", "ssio:IP4", "ssio:DIP", "ssio:IP1.ALT", "ssio:IP2.ALT",
	"ssio:IP1.ALT1", "ssio:IP1.ALT2", "ssio:IP2.ALT1", "ssio:IP2.ALT2", "DIAL", "DIAL2", "P24", "P28", "P2C",
	"ssio:IP1.ALT3", "SHIFT", "ssio:IP0.ALT" };
static const char *const s_tags_mono[P_COUNT] = {
	"MONO.IP0", "MONO.IP1", "MONO.IP2", "MONO.IP3", "MONO.IP4", "ssio:DIP", "MONO.IP1.ALT", "MONO.IP2.ALT",
	"MONO.IP1.ALT1", "MONO.IP1.ALT2", "MONO.IP2.ALT1", "MONO.IP2.ALT2", "DIAL", "DIAL2", "P24", "P28", "P2C",
	"MONO.IP1.ALT3", "SHIFT", "MONO.IP0.ALT" };

// machine state (saved)
typedef struct {
	uint8_t cocktail, input_mux, last_op4, irq_line, pad_run_, latched_input, pad0_[2];
	uint16_t line, field;
	int64_t main_tgt;            // main CPU clocks since reset (absolute)
	int64_t cyc_base;            // (the Z80's count is 32 bits on the Vita: time = base + cyc)
	int64_t ctc_time;            // the CTC advanced up to here
	uint64_t frac;               // the main clock's line fraction (rbx_line_cycles)
	int64_t snd_tgt[3];          // SSIO, TCS / SNT, 68K targets (their clocks)
	uint64_t snd_frac[3];
	int32_t watchdog;            // fields since the last kick
	// inputs
	uint8_t dial[4], an[4];
	// Draw Poker
	uint8_t coin_status, dp_output, dp_prev, pad1_;
	int32_t coin_timer, hopper_timer;   // fields (0: off)
	// Max RPM
	uint8_t adc_control, last_shift, pad2_[2];
	int8_t p1_shift, p2_shift, pad3_[2];
	uint8_t adc_mux, pad4_[3];
	// Spy Hunter
	int16_t scrollx, scrolly;
	uint8_t adc_value, pad5_[3];
} mstate_t;
static mstate_t S;

static int s_snd;               // MSND_B_* in use
static uint32_t s_snd_clock[3]; // their CPU clocks
static char s_nv_path[512];
static uint32_t s_nv_size;
static int s_sprite_xoffs, s_sprite_xoffs_flip;
static int s_spr_colormask;      // 91464: 0x40 on Journey
static uint8_t s_spy_sprite_color_mask;
static int16_t s_spy_scroll_offset;

// render snapshot
typedef struct {
	uint8_t videoram[0x800], spriteram[0x200], alpharam[0x800];
	uint32_t pal[MAX_PAL];
	uint8_t cocktail, valid;
	int16_t scrollx, scrolly;
} snap_t;
static snap_t *s_snap[2];
static int s_work;
static uint8_t *s_pri;           // the priority bitmap (SCR_W x SCR_H)
static uint16_t *s_frame;        // the unrotated, unmirrored frame (SCR_W x SCR_H) when the game is mirrored

enum { MC_PA_MAIN = RBX_PA_DRV, MC_PA_SSIO, MC_PA_SND2, MC_PA_SND68, MC_PA_MIX, MC_PA_END };
static const char *const s_pa_names[] = { "mcr main z80", "mcr ssio z80", "mcr tcs/snt", "mcr 68k sound",
                                          "mcr sound mix" };

static inline uint8_t rport(int p) { return s_port[p] >= 0 ? (uint8_t)rbx_port_read(s_port[p]) : 0xff; }
static inline int64_t main_now(void) { return S.cyc_base + (int64_t)s_main.cyc; }

// ---- palette ----
static inline uint32_t pal3(int v) { v &= 7; return (uint32_t)((v << 5) | (v << 2) | (v >> 1)); }
static inline uint32_t pal4(int v) { v &= 15; return (uint32_t)((v << 4) | v); }

static void set_color9(int index, int data)   // mcr_set_color: RRRBBBGGG
{
	s_pal_rgb[index] = pal3(data >> 6) << 16 | pal3(data >> 0) << 8 | pal3(data >> 3);
}

static void journey_set_color(int index, int data)
{
	int r = (data >> 6) & 7, g = (data >> 0) & 7, b = (data >> 3) & 7;
	r = (r << 5) | (r << 1);
	g = (g << 5) | (g << 1);
	b = (b << 5) | (b << 1);
	s_pal_rgb[index] = (uint32_t)(r << 16 | g << 8 | b);
	if ((index & 0x31) == 0x31) {   // (the upper bank's odd entries: a hard-wired low bit, finer grays)
		r |= 0x11;
		g |= 0x11;
		b |= 0x11;
	}
	s_pal_rgb[index + 64] = (uint32_t)(r << 16 | g << 8 | b);
}

static void pal90009_w(int index)   // xRBG_444
{
	const int v = s_pal_raw[index];
	s_pal_rgb[index] = pal4(v >> 8) << 16 | pal4(v) << 8 | pal4(v >> 4);
}

static void rebuild_palette(void)
{
	if (s_c->cpu == CPU_90009) {
		for (int i = 0; i < 32; i++)
			pal90009_w(i);
	} else if (s_c->cpu == CPU_90010 || s_c->cpu == CPU_91475) {
		for (int i = 0; i < 64; i++) {
			const int off = 0x780 + i * 2;   // (palette RAM is the videoram's upper 0x80 bytes)
			// the 9th bit is A0 of the bus: the last write of the pair decides; the pair is kept in s_pal_raw
			if (s_c->cpu == CPU_91475)
				journey_set_color(i, s_pal_raw[i]);
			else
				set_color9(i, s_pal_raw[i]);
			(void)off;
		}
	} else
		for (int i = 0; i < 64; i++)
			set_color9(i, s_pal_raw[i]);
	if (s_c->cpu == CPU_SCROLL) {   // (the alpha colours are wired: black, green, blue, white)
		s_pal_rgb[64] = 0x000000;
		s_pal_rgb[65] = 0x00ff00;
		s_pal_rgb[66] = 0x0000ff;
		s_pal_rgb[67] = 0xffffff;
	}
}

static void palette9_w(int offset, uint8_t data)   // mcr_paletteram9_w
{
	s_paletteram[offset & 0x7f] = data;
	const int i = (offset & 0x7f) >> 1;
	s_pal_raw[i] = (uint16_t)(data | ((offset & 1) << 8));
	set_color9(i, s_pal_raw[i]);
}

// ---- video RAM ----
static void videoram90010_w(int offset, uint8_t data)
{
	offset &= 0x7ff;
	s_videoram[offset] = data;
	if ((offset & 0x780) == 0x780) {
		const int i = (offset / 2) & 0x3f;
		s_pal_raw[i] = (uint16_t)(data | ((offset & 1) << 8));
		if (s_c->cpu == CPU_91475)
			journey_set_color(i, s_pal_raw[i]);
		else
			set_color9(i, s_pal_raw[i]);
	}
}

static inline int twotiger_eff(int offset) { return ((offset << 1) & 0x7fe) | ((offset >> 10) & 1); }

static void twotiger_videoram_w(int offset, uint8_t data)
{
	offset &= 0x7ff;
	const int eff = twotiger_eff(offset);
	s_videoram[eff] = data;
	if ((eff & 0x780) == 0x780) {
		const int i = ((offset & 0x400) >> 5) | ((offset >> 1) & 0x1f);
		s_pal_raw[i] = (uint16_t)(data | ((offset & 1) << 8));
		set_color9(i, s_pal_raw[i]);
	}
}

// ---- CTC and the daisy chain ----
static void ctc_sync(void)
{
	const int64_t now = main_now();
	if (now > S.ctc_time) {
		z80ctc_advance(&s_ctc, (int32_t)(now - S.ctc_time));
		S.ctc_time = now;
	}
}

static int s_live_line;   // the INT line as z80_run sees it (its HOLD clears it when the interrupt is taken)

static void ctc_intr(void *u, int state)
{
	(void)u;
	S.irq_line = (uint8_t)state;
	s_live_line = state;
	// (z80_run's HOLD clears its line when the instruction leaves no request pending: a change made by the instruction
	// itself (a RETI, an OUT to the CTC) is the request's own state)
	s_main.int_pending = state != 0;
	s_main.int_data = z80ctc_irq_vector(&s_ctc);
}

// the interrupt taken by the CPU (z80_run's HOLD cleared the copy it was given): the daisy chain's acknowledge
static void check_ack(void)
{
	if (S.irq_line && !s_live_line) {
		z80ctc_irq_ack(&s_ctc);
		s_main.int_data = z80ctc_irq_vector(&s_ctc);
	}
}

static void main_reti(z80 *z)
{
	if (z != &s_main)
		return;
	check_ack();
	ctc_sync();
	z80ctc_irq_reti(&s_ctc);
	z80_break = 1;
}

static void ctc_zc0(void *u, int state) { (void)u; z80ctc_trg(&s_ctc, 1, state); }

// ---- sound boards: brought up to the main CPU's time before a write ----
static void snd_sync_all(void);

// ---- the SSIO's I/O ports (MAME ioport_read with the games' custom inputs) ----
static uint8_t dpoker_ip0(void)
{
	uint8_t p0 = rport(P_IP0);
	p0 |= (S.coin_status >> 1) & 1;
	p0 ^= (uint8_t)((p0 << 1 & 0x80) | S.coin_status);
	return p0;
}

static uint8_t ssio_in(int offset)
{
	uint8_t v = offset == 5 ? 0xff : rport(P_IP0 + offset);
	switch (s_g) {
	case G_SOLARFOX:
		if (offset == 0) {
			uint8_t c;
			const uint8_t ip0 = rport(P_IP0), ip2 = rport(P_IP2);
			if (S.cocktail)
				c = ip0 | 0x08;
			else
				c = (uint8_t)(((ip0 & ~0x14) | 0x08) | ((ip0 & 0x08) >> 1) | ((ip2 & 0x01) << 4));
			v = (uint8_t)((v & ~0x1c) | (c & 0x1c));
		} else if (offset == 1)
			v = S.cocktail ? (uint8_t)(rport(P_IP1) | 0xf0) : (uint8_t)((rport(P_IP1) >> 4) | 0xf0);
		break;
	case G_KICK:
		if (offset == 1)
			v = (uint8_t)((v & 0x0f) | ((S.dial[1] << 4) & 0xf0));
		break;
	case G_DPOKER:
		if (offset == 0)
			v = (uint8_t)((v & ~0x8e) | (dpoker_ip0() & 0x8e));
		break;
	case G_WACKO:
		if (offset == 1) v = S.input_mux ? S.an[2] : S.an[0];
		else if (offset == 2) v = S.input_mux ? S.an[3] : S.an[1];
		break;
	case G_KROOZR:
		if (offset == 1) {
			const int dial = S.dial[0];
			v = (uint8_t)((v & ~0x47) | (((dial & 0x80) >> 1) | ((dial & 0x70) >> 4)));
		}
		break;
	case G_DEMODERB:
		if (offset == 1)
			v = (uint8_t)((v & 0x03) | ((S.dial[S.input_mux ? 2 : 0] & 0x3f) << 2));
		else if (offset == 2)
			v = (uint8_t)((v & 0x03) | ((S.dial[S.input_mux ? 3 : 1] & 0x3f) << 2));
		break;
	case G_SPYHUNT: case G_TURBOTAG:
		if (offset == 1) {
			snd_sync_all();
			v = (uint8_t)((v & ~0x60) | ((msnd_68k_read() << 5) & 0x60));
		} else if (offset == 2)
			v = S.adc_value;
		break;
	default:
		break;
	}
	return v;
}

static void dotron_op4(uint8_t data)
{
	S.last_op4 = data;
	msnd_snt_select(data & 0x0f);
	msnd_snt_int((data >> 4) & 1);
}

static void spyhunt_adc_value(void)
{
	// the ADC0804 converts the multiplexed wheel / pedal (ssio:IP2 / IP2.ALT)
	S.adc_value = S.input_mux ? S.an[1] : S.an[0];
}

static void ssio_out(int offset, uint8_t data)
{
	const int which = offset >> 2;
	if (which == 0) {   // mcr_control_port_w
		S.cocktail = (data >> 6) & 1;
		return;
	}
	switch (s_g) {
	case G_WACKO: S.input_mux = data & 1; break;
	case G_DOTRONE: snd_sync_all(); dotron_op4(data); break;
	case G_DEMODERB:
		if (data & 0x40) S.input_mux = 1;
		if (data & 0x80) S.input_mux = 0;
		snd_sync_all();
		msnd_tcs_write(data);
		break;
	case G_SPYHUNT: case G_TURBOTAG:
		S.input_mux = (data >> 7) & 1;
		if (!(S.last_op4 & 0x40) && (data & 0x40))   // (ADC0804 /WR rising: the conversion)
			spyhunt_adc_value();
		S.last_op4 = data;
		snd_sync_all();
		msnd_68k_sr_w(data & 0x0f);
		msnd_68k_sirq_w((data >> 4) & 1);
		break;
	default: break;
	}
}

// ---- the monoboard's I/O (MCR-3) ----
static uint8_t maxrpm_ip2(void)
{
	static const uint8_t shift_bits[5] = { 0x00, 0x05, 0x06, 0x01, 0x02 };
	const uint8_t start = rport(P_IP0), shift = rport(P_SHIFT);
	if (!(start & 0x08)) S.p1_shift = 0;
	if (!(start & 0x04)) S.p2_shift = 0;
	if (!(shift & 0x01) && (S.last_shift & 0x01) && ++S.p1_shift > 4) S.p1_shift = 4;
	if (!(shift & 0x02) && (S.last_shift & 0x02) && --S.p1_shift < 0) S.p1_shift = 0;
	if (!(shift & 0x04) && (S.last_shift & 0x04) && ++S.p2_shift > 4) S.p2_shift = 4;
	if (!(shift & 0x08) && (S.last_shift & 0x08) && --S.p2_shift < 0) S.p2_shift = 0;
	S.last_shift = shift;
	return (uint8_t)~((shift_bits[S.p1_shift] << 4) + shift_bits[S.p2_shift]);
}

static uint8_t mono_in(int offset)
{
	uint8_t v = rport(P_IP0 + offset);
	switch (s_g) {
	case G_DEMODERM:
		if (offset == 1)
			v = (uint8_t)((v & 0x03) | ((S.dial[S.input_mux ? 2 : 0] & 0x3f) << 2));
		else if (offset == 2)
			v = (uint8_t)((v & 0x03) | ((S.dial[S.input_mux ? 3 : 1] & 0x3f) << 2));
		break;
	case G_MAXRPM:
		if (offset == 1) v = S.latched_input;
		else if (offset == 2) v = maxrpm_ip2();
		break;
	case G_RAMPAGE:
		if (offset == 4) { snd_sync_all(); v = (uint8_t)((v & 0x7f) | (msnd_68k_read() << 7)); }
		break;
	case G_POWERDRV:
		if (offset == 2) { snd_sync_all(); v = (uint8_t)((v & 0x7f) | (msnd_68k_read() << 7)); }
		break;
	case G_STARGRDS:
		if (offset == 0) {
			uint8_t r = v;
			if (S.input_mux)
				r = (uint8_t)((r & ~0x0a) | (rport(P_IP0ALT) & 0x0a));
			snd_sync_all();
			v = (uint8_t)((r & ~0x10) | ((msnd_68k_read() << 4) & 0x10));
		}
		break;
	default: break;
	}
	return v;
}

// the ADC0844's conversion (adc0844.cpp conversion_complete) of channels ch1..ch4 = an[0..3]; done at the write
// (MAME: 40 us later)
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

static void maxrpm_adc_read(void) { S.latched_input = S.adc_value; }

static void mono_out(int port, uint8_t data)
{
	if (port == 5) {   // mcrmono_control_port_w (+ the games' extra bits)
		if (s_g == G_MAXRPM)
			S.adc_control = (data >> 1) & 0x0f;
		if (s_g == G_STARGRDS)
			S.input_mux = (data >> 1) & 1;
		S.cocktail = (data >> 6) & 1;
		return;
	}
	if (port != 6)
		return;
	snd_sync_all();
	switch (s_g) {
	case G_DEMODERM:
		if (data & 0x80) S.input_mux = 0;
		if (data & 0x40) S.input_mux = 1;
		msnd_tcs_write(data);
		break;
	case G_SARGE: msnd_tcs_write(data); break;
	case G_MAXRPM:
		if (!(data & 0x80))
			maxrpm_adc_read();
		if (!(data & 0x40) && !(data & 0x20)) {   // write: MUX bits = bitswap<4>(control, 2, 3, 1, 0)
			const uint8_t c = S.adc_control;
			S.adc_mux = (uint8_t)(((c >> 2) & 1) << 3 | ((c >> 3) & 1) << 2 | ((c >> 1) & 1) << 1 | (c & 1));
			S.adc_value = adc0844_convert(S.an, S.adc_mux);
		}
		msnd_tcs_write(data);
		break;
	case G_RAMPAGE: case G_POWERDRV:
		msnd_68k_reset_write((~data >> 5) & 1);
		msnd_68k_write(data);
		break;
	case G_STARGRDS:
		msnd_68k_reset_write((~data >> 6) & 1);
		msnd_68k_write((uint8_t)((data << 1) | (data >> 7)));
		break;
	default: break;
	}
}

// ---- main CPU bus ----
static void watchdog_kick(void) { S.watchdog = 0; }

static uint8_t main_rd(void *u, uint16_t a)
{
	(void)u;
	switch (s_c->cpu) {
	case CPU_90009:
		if (a >= 0x7000 && a < 0x8000) return s_nvram[a & 0x7ff];
		if (a >= 0x8000 && a < 0x8200 && s_g == G_DPOKER) return s_hiram[a & 0x1ff];
		if (a >= 0xf000 && a < 0xf400) return s_spriteram[a & 0x1ff];
		if (a >= 0xfc00) return s_videoram[a & 0x3ff];
		return a < 0x7000 && a < s_mrom_size ? s_mrom[a] : 0xff;
	case CPU_90010: case CPU_91475:
		if (a < 0xc000) return a < s_mrom_size ? s_mrom[a] : 0xff;
		if (a < 0xe000) return s_nvram[a & 0x7ff];
		if (a & 0x0800) {
			if (s_g == G_TWOTIGER)
				return s_videoram[twotiger_eff(a & 0x7ff)];
			return s_videoram[a & 0x7ff];
		}
		return s_spriteram[a & 0x1ff];
	case CPU_91490:
		if (a < 0xe000) return s_mrom[a];
		if (a < 0xe800) return s_nvram[a & 0x7ff];
		if (a < 0xec00) return s_spriteram[a & 0x1ff];
		if (a >= 0xf000 && a < 0xf800) return s_videoram[a & 0x7ff];
		return 0xff;
	case CPU_MONO:
		if (a < 0xe000 || a >= 0xf800) return s_mrom[a];
		if (a < 0xe800) return s_nvram[a & 0x7ff];
		if (a < 0xea00) return s_spriteram[a & 0x1ff];
		if (a < 0xec00) return s_hiram[a & 0x1ff];
		if (a >= 0xf000) return s_videoram[a & 0x7ff];
		return 0xff;
	default:   // CPU_SCROLL
		if (a == 0x0b53 && s_g == G_TURBOTAG)   // (the instruction at 0x0b29 reads the checksum byte: 0x82)
			return s_main.pc >= 0x0b29 && s_main.pc < 0x0b2d ? 0x82 : 0x92;
		if (a < 0xe000) return s_mrom[a];
		if (a < 0xe800) return s_videoram[a & 0x7ff];
		if (a < 0xf000) return s_alpharam[a & 0x3ff];
		if (a < 0xf800) return s_nvram[a & 0x7ff];
		if (a < 0xfa00) return s_spriteram[a & 0x1ff];
		return 0xff;
	}
}

static void main_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	switch (s_c->cpu) {
	case CPU_90009:
		if (a >= 0x7000 && a < 0x8000) { s_nvram[a & 0x7ff] = v; return; }
		if (a >= 0x8000 && a < 0x8200 && s_g == G_DPOKER) { s_hiram[a & 0x1ff] = v; return; }
		if (a >= 0xf000 && a < 0xf400) { s_spriteram[a & 0x1ff] = v; return; }
		if (a >= 0xf400 && a < 0xf800) {   // palette (low byte)
			const int i = a & 0x1f;
			s_pal_raw[i] = (uint16_t)((s_pal_raw[i] & 0xff00) | v);
			pal90009_w(i);
			return;
		}
		if (a >= 0xf800 && a < 0xfc00) {   // palette (high byte)
			const int i = a & 0x1f;
			s_pal_raw[i] = (uint16_t)((s_pal_raw[i] & 0x00ff) | (v << 8));
			pal90009_w(i);
			return;
		}
		if (a >= 0xfc00) s_videoram[a & 0x3ff] = v;
		return;
	case CPU_90010: case CPU_91475:
		if (a < 0xc000) return;
		if (a < 0xe000) { s_nvram[a & 0x7ff] = v; return; }
		if (a & 0x0800) {
			if (s_g == G_TWOTIGER)
				twotiger_videoram_w(a & 0x7ff, v);
			else
				videoram90010_w(a & 0x7ff, v);
			return;
		}
		s_spriteram[a & 0x1ff] = v;
		return;
	case CPU_91490:
		if (a < 0xe000) return;
		if (a < 0xe800) { s_nvram[a & 0x7ff] = v; return; }
		if (a < 0xec00) { s_spriteram[a & 0x1ff] = v; return; }
		if (a < 0xf000) return;
		if (a < 0xf800) { s_videoram[a & 0x7ff] = v; return; }
		palette9_w(a & 0x7f, v);
		return;
	case CPU_MONO:
		if (a < 0xe000 || a >= 0xf800) return;
		if (a < 0xe800) { s_nvram[a & 0x7ff] = v; return; }
		if (a < 0xea00) { s_spriteram[a & 0x1ff] = v; return; }
		if (a < 0xec00) { s_hiram[a & 0x1ff] = v; return; }
		if (a < 0xf000) { palette9_w(a & 0x7f, v); return; }
		s_videoram[a & 0x7ff] = v;
		return;
	default:   // CPU_SCROLL
		if (a < 0xe000) return;
		if (a < 0xe800) { s_videoram[a & 0x7ff] = v; return; }
		if (a < 0xf000) { s_alpharam[a & 0x3ff] = v; return; }
		if (a < 0xf800) { s_nvram[a & 0x7ff] = v; return; }
		if (a < 0xfa00) { s_spriteram[a & 0x1ff] = v; return; }
		if (a < 0xfc00) palette9_w(a & 0x7f, v);
		return;
	}
}

static uint8_t main_in(z80 *z, uint8_t port)
{
	(void)z;
	if (s_c->cpu == CPU_MONO) {
		if ((port & 0xf0) == 0xf0) {   // the CTC (f0-f3 mirror 0c)
			check_ack();
			ctc_sync();
			return z80ctc_read(&s_ctc, port & 3);
		}
		if (!(port & 0x80) && (port & 7) <= 4)
			return mono_in(port & 7);
		return 0xff;
	}
	if (port >= 0xf0 && port <= 0xf3) {
		check_ack();
		ctc_sync();
		return z80ctc_read(&s_ctc, port & 3);
	}
	if (port < 0x20) {
		const int o = port & 7;
		if (o <= 4)
			return ssio_in(o);
		if (o == 7) {
			snd_sync_all();
			return msnd_ssio_status();
		}
		return 0xff;
	}
	if (s_g == G_DPOKER) {
		if (port == 0x24) return rport(P_P24);
		if (port == 0x28) return rport(P_P28);
		if (port == 0x2c) return rport(P_P2C);
	}
	return 0xff;
}

static void main_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	if (s_c->cpu == CPU_MONO) {
		if ((port & 0xf0) == 0xf0) {
			check_ack();
			ctc_sync();
			z80ctc_write(&s_ctc, port & 3, v);
			z80_break = 1;
			return;
		}
		if (port & 0x80)
			return;
		if ((port & 7) == 7) { watchdog_kick(); return; }
		mono_out(port & 7, v);
		return;
	}
	if (port >= 0xf0 && port <= 0xf3) {
		check_ack();
		ctc_sync();
		z80ctc_write(&s_ctc, port & 3, v);
		z80_break = 1;
		return;
	}
	if (port < 0x08) { ssio_out(port, v); return; }
	if (port >= 0x1c && port < 0x20) {
		snd_sync_all();
		msnd_ssio_write(port & 3, v);
		return;
	}
	if (port == 0xe0) { watchdog_kick(); return; }
	if (s_c->cpu == CPU_SCROLL && port >= 0x84 && port <= 0x86) {   // spyhunt_scroll_value_w
		switch (port - 0x84) {
		case 0: S.scrollx = (int16_t)((S.scrollx & ~0xff) | v); break;
		case 1:
			S.scrollx = (int16_t)((S.scrollx & 0xff) | ((v & 0x07) << 8));
			S.scrolly = (int16_t)((S.scrolly & 0xff) | ((v & 0x80) << 1));
			break;
		case 2: S.scrolly = (int16_t)((S.scrolly & ~0xff) | v); break;
		}
		return;
	}
	if (s_g == G_DPOKER && port == 0x34) {   // output_w (the hopper: d6)
		if (v & 0x40 & ~S.dp_output)
			S.hopper_timer = 30;   // 500 ms
		S.dp_output = v;
	}
}

static void map_main(void)
{
	for (int p = 0; p < 256; p++)
		s_rmap[p] = NULL;
	const uint32_t rom_end = s_c->cpu == CPU_90009 ? 0x7000 : (s_c->cpu == CPU_90010 || s_c->cpu == CPU_91475) ?
	                         0xc000 : 0xe000;
	for (uint32_t p = 0; p < rom_end >> 8; p++)
		if ((p + 1) * 256 <= s_mrom_size)
			s_rmap[p] = s_mrom + p * 256;
	if (s_g == G_TURBOTAG)
		s_rmap[0x0b] = NULL;   // (turbotag_kludge_r)
	s_main.read_map = s_rmap;
	s_main.read_byte = main_rd;
	s_main.write_byte = main_wr;
	s_main.port_in = main_in;
	s_main.port_out = main_out;
	s_main.userdata = NULL;
}

// ---- graphics decoding ----
static void decode_tiles(void)
{
	// mcr_bg_layout: 8x8, planes { half + 0, half + 1, 0, 1 }, 2 bits a pixel in each half, 16 bytes a tile half
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
}

static void decode_scroll_tiles(void)
{
	// spyhunt_charlayout: 64x32 drawn from 32x16 source pixels (each doubled both ways), planes
	// { RGN_FRAC(1,2), RGN_FRAC(1,2)+1, 0, 1 }, 2 bits a pixel in each half, 64-bit rows, 128 bytes a tile half
	const uint32_t half = s_gfx1_size / 2;
	s_ntiles = (int)(half / 128);
	for (int t = 0; t < s_ntiles; t++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 32; x++) {
				const uint32_t bit = (uint32_t)t * 1024 + (uint32_t)y * 64 + (uint32_t)x * 2;
				const uint8_t hi = s_gfx1[half + (bit >> 3)], lo = s_gfx1[bit >> 3];
				const int sh = 6 - (bit & 7);
				s_tiles[t * 512 + y * 32 + x] = (uint8_t)((((hi >> sh) & 3) << 2) | ((lo >> sh) & 3));
			}
}

static void decode_sprites(void)
{
	// mcr_sprite_layout: 32x32, 4 bpp nibbles, x: quarter (j / 2) * RGN_FRAC(1,4) + group (x / 8) + nibble (j & 1)
	const uint32_t q = s_gfx2_size / 4;
	s_nsprites = (int)(q / 128);
	for (int s = 0; s < s_nsprites; s++)
		for (int y = 0; y < 32; y++) {
			int first = 32, last = 0;
			for (int x = 0; x < 32; x++) {
				const int g = x >> 3, j = x & 7;
				const uint8_t b = s_gfx2[(j >> 1) * q + (uint32_t)s * 128 + y * 4 + g];
				const uint8_t pen = (j & 1) ? (b & 15) : (b >> 4);
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

static void decode_alpha(void)
{
	// spyhunt_alphalayout: 16x16 from 8x8 doubled, 2 bpp, planes { 0, 1 }, 16 bits a row
	s_nalpha = (int)(s_gfx3_size / 16);
	for (int t = 0; t < s_nalpha; t++) {
		int any = 0;
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				const uint32_t bit = (uint32_t)t * 128 + y * 16 + x * 2;
				const uint8_t b = s_gfx3[bit >> 3];
				const uint8_t p = (uint8_t)((b >> (6 - (bit & 7))) & 3);
				s_alpha[t * 64 + y * 8 + x] = p;
				any |= p;
			}
		s_alpha_blank[t] = (uint8_t)!any;
	}
}

// ---- rendering ----
static void draw_tiles(const snap_t *sn, uint16_t *bm, int pitch)
{
	const int flip = sn->cocktail;
	for (int ty = 0; ty < 30; ty++)
		for (int tx = 0; tx < 32; tx++) {
			const int idx = ty * 32 + tx;
			int code, color = 0, cat, fx = 0, fy = 0;
			if (s_c->cpu == CPU_90009) {
				code = sn->videoram[idx];
				cat = 1;
			} else {
				const int data = sn->videoram[idx * 2] | (sn->videoram[idx * 2 + 1] << 8);
				if (s_c->cpu == CPU_91490 || s_c->cpu == CPU_MONO) {
					code = data & 0x3ff;
					color = (data >> 12) & 3;
					fx = (data >> 10) & 1;
					fy = (data >> 11) & 1;
				} else {
					code = data & 0x1ff;
					color = (data >> 11) & 3;
					fx = (data >> 9) & 1;
					fy = (data >> 10) & 1;
				}
				cat = (data >> 14) & 3;
			}
			code = (int)rbx_wrap((uint32_t)code, (uint32_t)s_ntiles);
			const uint8_t *t = s_tiles + code * 64;
			// the screen position (cocktail: the whole map flipped)
			int px = tx * 16, py = ty * 16;
			if (flip) {
				px = SCR_W - 16 - px;
				py = SCR_H - 16 - py;
				fx ^= 1;
				fy ^= 1;
			}
			const uint16_t base = (uint16_t)(color * 16);
			const uint8_t pri = (uint8_t)(cat << 4);
			for (int y = 0; y < 8; y++) {
				const uint8_t *row = t + (fy ? 7 - y : y) * 8;
				uint16_t *d = bm + (py + y * 2) * pitch + px;
				for (int x = 0; x < 8; x++) {
					const uint16_t c = (uint16_t)(base + row[fx ? 7 - x : x]);
					d[2 * x] = c;
					d[2 * x + 1] = c;
				}
				memcpy(d + pitch, d, 16 * sizeof(uint16_t));
				uint8_t *pr = s_pri + (py + y * 2) * SCR_W + px;
				memset(pr, pri, 16);
				memset(pr + SCR_W, pri, 16);
			}
		}
}

static void sprites_91399(const snap_t *sn, uint16_t *bm, int pitch)
{
	for (int offs = 0; offs < 0x200; offs += 4) {
		const uint8_t *sr = sn->spriteram + offs;
		const int code = (int)rbx_wrap(sr[1] & 0x3f, (uint32_t)s_nsprites);
		int hflip = (sr[1] & 0x40) ? 31 : 0, vflip = (sr[1] & 0x80) ? 31 : 0;
		int sx = (sr[2] - 4) * 2, sy = (240 - sr[0]) * 2;
		if (sn->cocktail) {
			hflip ^= 31;
			vflip ^= 31;
			sx = 466 - sx + s_sprite_xoffs_flip;
			sy = 450 - sy;
		} else
			sx += s_sprite_xoffs;
		sx &= 0x1ff;
		sy &= 0x1ff;
		const uint8_t *gfx = s_sprites + code * 1024;
		for (int y = 0; y < 32; y++, sy = (sy + 1) & 0x1ff) {
			if (sy >= SCR_H)
				continue;
			const int r = y ^ vflip;
			const uint8_t first = s_spr_rows[code][r][0], last = s_spr_rows[code][r][1];
			if (last <= first)
				continue;
			const uint8_t *src = gfx + r * 32;
			uint16_t *dst = bm + sy * pitch;
			uint8_t *pri = s_pri + sy * SCR_W;
			// (a transparent pen changes nothing: only the source row's opaque span)
			const int x0 = hflip ? 31 - (last - 1) : first, x1 = hflip ? 31 - first + 1 : last;
			for (int x = x0; x < x1; x++) {
				const uint8_t p = src[x ^ hflip];
				if (!p)
					continue;
				const int tx = (sx + x) & 0x1ff;
				const int pix = pri[tx] | p;
				pri[tx] = (uint8_t)pix;
				if (pix & 0x07)
					dst[tx] = (uint16_t)pix;
			}
		}
	}
}

static void sprites_91464(const snap_t *sn, uint16_t *bm, int pitch, int sprmask, int colormask)
{
	for (int offs = 0x200 - 4; offs >= 0; offs -= 4) {
		const uint8_t *sr = sn->spriteram + offs;
		const int code = (int)rbx_wrap((uint32_t)(sr[2] + 256 * ((sr[1] >> 3) & 1)), (uint32_t)s_nsprites);
		const int color = (((~sr[1] & 3) << 4) & sprmask) | colormask;
		int hflip = (sr[1] & 0x10) ? 31 : 0, vflip = (sr[1] & 0x20) ? 31 : 0;
		int sx = (sr[3] - 3) * 2, sy = (241 - sr[0]) * 2;
		if (sn->cocktail) {
			hflip ^= 31;
			vflip ^= 31;
			sx = 480 - sx;
			sy = 452 - sy;
		}
		sx &= 0x1ff;
		sy &= 0x1ff;
		const uint8_t *gfx = s_sprites + code * 1024;
		for (int y = 0; y < 32; y++, sy = (sy + 1) & 0x1ff) {
			if (sy < 2 || sy >= s_vis_h)
				continue;
			const int r = y ^ vflip;
			const uint8_t first = s_spr_rows[code][r][0], last = s_spr_rows[code][r][1];
			if (last <= first)
				continue;
			const uint8_t *src = gfx + r * 32;
			uint16_t *dst = bm + sy * pitch;
			uint8_t *pri = s_pri + sy * SCR_W;
			const int x0 = hflip ? 31 - (last - 1) : first, x1 = hflip ? 31 - first + 1 : last;
			for (int x = x0; x < x1; x++) {
				const uint8_t p = src[x ^ hflip];
				if (!p)
					continue;
				const int tx = (sx + x) & 0x1ff;
				if (tx >= s_vis_w || pri[tx] == 0xff)
					continue;
				pri[tx] = 0xff;
				const int pix = color | p;
				if (pix & 0x07)
					dst[tx] = (uint16_t)pix;
			}
		}
	}
}

// MCR-3 sprites (mcr3_update_sprites): pens 0 and 8 transparent, then pen 8 drawn only over earlier sprites
static void sprites_mcr3(const snap_t *sn, uint16_t *bm, int pitch, int color_mask, int dx, int dy)
{
	for (int y = 0; y < s_vis_h; y++)
		memset(s_pri + y * SCR_W, 1, (size_t)s_vis_w);
	for (int offs = 0x200 - 4; offs >= 0; offs -= 4) {
		const uint8_t *sr = sn->spriteram + offs;
		if (sr[0] == 0)
			continue;
		const int flags = sr[1];
		const int code = (int)rbx_wrap((uint32_t)(sr[2] + 256 * ((flags >> 3) & 1)), (uint32_t)s_nsprites);
		const int base = (~flags & color_mask) * 16;
		int hflip = (flags & 0x10) ? 31 : 0, vflip = (flags & 0x20) ? 31 : 0;
		int sx = (sr[3] - 3) * 2 + dx, sy = (241 - sr[0]) * 2 + dy;
		if (sn->cocktail) {
			hflip ^= 31;
			vflip ^= 31;
			sx = 480 - sx;
			sy = 452 - sy;
		}
		const uint8_t *gfx = s_sprites + code * 1024;
		for (int y = 0; y < 32; y++) {
			const int py = sy + y;
			if (py < 0 || py >= s_vis_h)
				continue;
			const int r = y ^ vflip;
			const uint8_t first = s_spr_rows[code][r][0], last = s_spr_rows[code][r][1];
			if (last <= first)
				continue;
			const uint8_t *src = gfx + r * 32;
			uint16_t *dst = bm + py * pitch;
			uint8_t *pri = s_pri + py * SCR_W;
			const int x0 = hflip ? 31 - (last - 1) : first, x1 = hflip ? 31 - first + 1 : last;
			for (int x = x0; x < x1; x++) {
				const int px = sx + x;
				if (px < 0 || px >= s_vis_w)
					continue;
				const uint8_t p = src[x ^ hflip];
				if (p & 7) {   // (the visible pass: every pen but 0 and 8)
					dst[px] = (uint16_t)(base + p);
					pri[px] = 31;
				}
			}
			for (int x = x0; x < x1; x++) {   // (the mask pass: pen 8 over what earlier sprites drew)
				const int px = sx + x;
				if (px < 0 || px >= s_vis_w)
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

// the monoboard's tilemap (mcrmono_get_bg_tile_info): 32x30 tiles of 16x16, colour ^ 3, code bit 10 from bit 14
static void draw_mono_tiles(const snap_t *sn, uint16_t *bm, int pitch)
{
	const int flip = sn->cocktail;
	for (int ty = 0; ty < 30; ty++)
		for (int tx = 0; tx < 32; tx++) {
			const int idx = ty * 32 + tx;
			const int data = sn->videoram[idx * 2] | (sn->videoram[idx * 2 + 1] << 8);
			const int code = (int)rbx_wrap((uint32_t)((data & 0x3ff) | ((data >> 4) & 0x400)), (uint32_t)s_ntiles);
			const int color = ((data >> 12) & 3) ^ 3;
			int fx = (data >> 10) & 1, fy = (data >> 11) & 1;
			int px = tx * 16, py = ty * 16;
			if (flip) {
				px = SCR_W - 16 - px;
				py = SCR_H - 16 - py;
				fx ^= 1;
				fy ^= 1;
			}
			const uint8_t *t = s_tiles + code * 64;
			const uint16_t base = (uint16_t)(color * 16);
			for (int y = 0; y < 8; y++) {
				const uint8_t *row = t + (fy ? 7 - y : y) * 8;
				uint16_t *d = bm + (py + y * 2) * pitch + px;
				for (int x = 0; x < 8; x++) {
					const uint16_t c = (uint16_t)(base + row[fx ? 7 - x : x]);
					d[2 * x] = c;
					d[2 * x + 1] = c;
				}
				memcpy(d + pitch, d, 16 * sizeof(uint16_t));
			}
		}
}

// MCR-3 scrolling: the 64x32 map of 64x32 tiles (spyhunt_bg_scan), scrolled; pens 48-63
static void draw_scroll(const snap_t *sn, uint16_t *bm, int pitch)
{
	const int sx0 = sn->scrollx * 2 + s_spy_scroll_offset, sy0 = sn->scrolly * 2;
	for (int y = 0; y < s_vis_h; y++) {
		const int my = (y + sy0) & 0x3ff;     // (32 rows of 32 = 1024)
		const int row = my >> 5, py = my & 31;
		uint16_t *d = bm + y * pitch;
		int x = 0;
		int mx = sx0 & 0xfff;                 // (64 columns of 64 = 4096)
		while (x < s_vis_w) {
			const int col = mx >> 6, px = mx & 63;
			const int idx = (row & 0x0f) | ((col & 0x3f) << 4) | ((row & 0x10) << 6);
			const int data = sn->videoram[idx & 0x7ff];
			const int code = (int)rbx_wrap((uint32_t)((data & 0x3f) | ((data >> 1) & 0x40)), (uint32_t)s_ntiles);
			const int sy = ((data & 0x40) ? 31 - py : py) >> 1;   // (TILE_FLIPY)
			const uint8_t *t = s_tiles + code * 512 + sy * 32;
			int n = 64 - px;
			if (n > s_vis_w - x)
				n = s_vis_w - x;
			for (int i = 0; i < n; i++)
				d[x + i] = (uint16_t)(3 * 16 + t[(px + i) >> 1]);
			x += n;
			mx = (mx + n) & 0xfff;
		}
	}
}

// the alpha layer: 32x32 characters of 16x16 (8x8 doubled), TILEMAP_SCAN_COLS, scrolled 16 left, pen 0 clear
static void draw_alpha(const snap_t *sn, uint16_t *bm, int pitch)
{
	for (int tx = 0; tx < 32; tx++)
		for (int ty = 0; ty < 30; ty++) {
			const int data = sn->alpharam[tx * 32 + ty];
			const int code = (int)rbx_wrap((uint32_t)data, (uint32_t)s_nalpha);
			if (s_alpha_blank[code])
				continue;
			const uint8_t *t = s_alpha + code * 64;
			const int px = tx * 16 - 16, py = ty * 16;
			for (int y = 0; y < 16; y++) {
				uint16_t *d = bm + (py + y) * pitch;
				const uint8_t *row = t + (y >> 1) * 8;
				for (int x = 0; x < 16; x++) {
					const int sx = px + x;
					if (sx < 0 || sx >= s_vis_w)
						continue;
					const uint8_t p = row[x >> 1];
					if (p)
						d[sx] = (uint16_t)(4 * 16 + p);
				}
			}
		}
}

static void mcr_render(void)
{
	const snap_t *sn = s_snap[s_work ^ 1];
	const int W = s_vis_w, H = s_vis_h;
	uint16_t *out = rbx_video.bitmap;
	const int opitch = rbx_video.pitch;
	if (!sn->valid) {
		for (int y = 0; y < H; y++)
			memset(out + y * opitch, 0, (size_t)W * sizeof(uint16_t));
		return;
	}
	memcpy(rbx_video.palette, sn->pal, sizeof(sn->pal));
	uint16_t *bm = s_mirror ? s_frame : out;
	const int pitch = s_mirror ? SCR_W : opitch;
	if (s_c->cpu == CPU_SCROLL) {
		draw_scroll(sn, bm, pitch);
		sprites_mcr3(sn, bm, pitch, s_spy_sprite_color_mask, -12, 0);
		draw_alpha(sn, bm, pitch);
	} else if (s_c->cpu == CPU_MONO) {
		draw_mono_tiles(sn, bm, pitch);
		sprites_mcr3(sn, bm, pitch, 0x03, 0, 0);
	} else {
		draw_tiles(sn, bm, pitch);
		if (s_c->spr == SPR_91399)
			sprites_91399(sn, bm, pitch);
		else
			sprites_91464(sn, bm, pitch, 0x30, s_spr_colormask);
	}
	if (s_mirror == 1)
		for (int y = 0; y < H; y++) {
			const uint16_t *s = s_frame + y * SCR_W;
			uint16_t *d = out + y * opitch;
			for (int x = 0; x < W; x++)
				d[x] = s[W - 1 - x];
		}
	else if (s_mirror == 2)
		for (int y = 0; y < H; y++)
			memcpy(out + y * opitch, s_frame + (H - 1 - y) * SCR_W, (size_t)W * sizeof(uint16_t));
}

static void present(void)
{
	rbx_video_wait_idle();
	snap_t *sn = s_snap[s_work];
	memcpy(sn->videoram, s_videoram, sizeof(s_videoram));
	memcpy(sn->spriteram, s_spriteram, sizeof(s_spriteram));
	if (s_c->cpu == CPU_SCROLL)
		memcpy(sn->alpharam, s_alpharam, sizeof(s_alpharam));
	memcpy(sn->pal, s_pal_rgb, sizeof(sn->pal));
	sn->cocktail = S.cocktail;
	sn->scrollx = S.scrollx;
	sn->scrolly = S.scrolly;
	sn->valid = 1;
	s_work ^= 1;
	rbx_video_frame_done();
}

// ---- inputs, a field at a time ----
static void inputs_field(void)
{
	switch (s_g) {
	case G_TRON: case G_DOTRON: case G_DOTRONE: {
		// the aiming dial (PORT_REVERSE) on the right stick's left / right; the joystick moves
		const uint32_t in = rbx_input(1);
		const int step = 5;
		const int d = ((in & RBX_IN_RRIGHT) ? step : 0) - ((in & RBX_IN_RLEFT) ? step : 0);
		S.dial[0] = (uint8_t)(S.dial[0] - d);
		const uint32_t in2 = rbx_input(2);
		const int d2 = ((in2 & RBX_IN_RRIGHT) ? step : 0) - ((in2 & RBX_IN_RLEFT) ? step : 0);
		S.dial[1] = (uint8_t)(S.dial[1] - d2);
		break;
	}
	case G_KICK:
		S.dial[0] = (uint8_t)(S.dial[0] - rbx_analog_dpad(1, RBX_IPT_DIAL, 3));
		S.dial[1] = (uint8_t)(S.dial[1] - rbx_analog_dpad(2, RBX_IPT_DIAL, 3));
		break;
	case G_TWOTIGERC:
		S.dial[0] = (uint8_t)(S.dial[0] - rbx_analog_dpad(1, RBX_IPT_DIAL, 3));
		S.dial[1] = (uint8_t)(S.dial[1] - rbx_analog_dpad(2, RBX_IPT_DIAL, 3));
		break;
	case G_CRATER:
		S.dial[0] = (uint8_t)(S.dial[0] - rbx_analog_dpad(1, RBX_IPT_DIAL, 4));
		break;
	case G_KROOZR: {
		const uint32_t in = rbx_input(1);
		const int d = ((in & RBX_IN_RRIGHT) ? 4 : 0) - ((in & RBX_IN_RLEFT) ? 4 : 0);
		S.dial[0] = (uint8_t)(S.dial[0] - d);
		// the analog stick (48..152, centre 100): the stick's deflection, or the d-pad at full
		int ax = rbx_analog(1, RBX_IPT_AD_STICK_X), ay = rbx_analog(1, RBX_IPT_AD_STICK_Y);
		if (!ax) ax = ((in & RBX_IN_RIGHT) ? 8 : 0) - ((in & RBX_IN_LEFT) ? 8 : 0);
		if (!ay) ay = ((in & RBX_IN_DOWN) ? 8 : 0) - ((in & RBX_IN_UP) ? 8 : 0);
		if (ax > 8) ax = 8;
		if (ax < -8) ax = -8;
		if (ay > 8) ay = 8;
		if (ay < -8) ay = -8;
		S.an[0] = (uint8_t)(100 + ax * 52 / 8);
		S.an[1] = (uint8_t)(100 + ay * 52 / 8);
		break;
	}
	case G_WACKO:
		// the trackball on the d-pad / left stick (IP1 x, IP2 y reversed); the fire joystick on the right stick
		for (int p = 0; p < 2; p++) {
			S.an[p * 2] = (uint8_t)(S.an[p * 2] + rbx_analog_dpad(p + 1, RBX_IPT_TRACKBALL_X, 5));
			S.an[p * 2 + 1] = (uint8_t)(S.an[p * 2 + 1] - rbx_analog_dpad(p + 1, RBX_IPT_TRACKBALL_Y, 5));
		}
		break;
	case G_TWOTIGER: {
		for (int p = 0; p < 2; p++) {   // the yoke (0..206, centre 0x67): the stick's deflection or the d-pad
			const uint32_t in = rbx_input(p + 1);
			int ax = rbx_analog(p + 1, RBX_IPT_AD_STICK_X);
			if (!ax) ax = ((in & RBX_IN_RIGHT) ? 8 : 0) - ((in & RBX_IN_LEFT) ? 8 : 0);
			if (ax > 8) ax = 8;
			if (ax < -8) ax = -8;
			S.an[p] = (uint8_t)(0x67 + ax * 0x67 / 8);
		}
		break;
	}
	case G_DEMODERB: case G_DEMODERM:
		for (int p = 0; p < 4; p++)   // the four steering wheels: each player's left / right
			S.dial[p] = (uint8_t)(S.dial[p] + rbx_analog_dpad(p + 1, RBX_IPT_DIAL, 3));
		break;
	case G_MAXRPM: {
		// ADC0844 channels: 1 = P2 pedal, 2 = P1 pedal (0x30..0xff), 3 = P2 wheel, 4 = P1 wheel (0x34..0xb4)
		for (int p = 0; p < 2; p++) {
			const uint32_t in = rbx_input(p + 1);
			const int pedal = (in & RBX_IN_B1) ? 0xff : 0x30;
			int ax = rbx_analog(p + 1, RBX_IPT_PADDLE);
			if (!ax) ax = ((in & RBX_IN_RIGHT) ? 8 : 0) - ((in & RBX_IN_LEFT) ? 8 : 0);
			if (ax > 8) ax = 8;
			if (ax < -8) ax = -8;
			const int wheel = 0x74 - ax * 0x40 / 8;   // (PORT_REVERSE)
			S.an[p ? 0 : 1] = (uint8_t)(0xff - pedal + 0x30);   // (PORT_REVERSE)
			S.an[p ? 2 : 3] = (uint8_t)wheel;
		}
		break;
	}
	case G_SPYHUNT: case G_TURBOTAG: {
		// the wheel (ssio:IP2, 0x34..0xb4 centre 0x74) and the pedal (ssio:IP2.ALT, 0x30..0xff)
		const uint32_t in = rbx_input(1);
		int ax = rbx_analog(1, RBX_IPT_PADDLE);
		if (!ax) ax = ((in & RBX_IN_RIGHT) ? 8 : 0) - ((in & RBX_IN_LEFT) ? 8 : 0);
		if (ax > 8) ax = 8;
		if (ax < -8) ax = -8;
		S.an[1] = (uint8_t)(0x74 + ax * 0x40 / 8);
		int ped = -rbx_analog(1, RBX_IPT_AD_STICK_Y);   // (stick up = gas; the d-pad's up or button 1 = full)
		if (in & (RBX_IN_UP | RBX_IN_B1)) ped = 8;
		if (ped < 0) ped = 0;
		if (ped > 8) ped = 8;
		S.an[0] = (uint8_t)(0x30 + ped * (0xff - 0x30) / 8);
		break;
	}
	default: break;
	}
	// the analog bits the ports' defaults stand in for
	switch (s_g) {
	case G_TRON: case G_TWOTIGERC:
		rbx_port_custom(s_port[P_IP1], 0xff, S.dial[0]);
		rbx_port_custom(s_port[P_IP4], 0xff, S.dial[1]);
		break;
	case G_DOTRON: case G_DOTRONE: {
		rbx_port_custom(s_port[P_IP1], 0x7f, S.dial[0] & 0x7f);
		// aim up / down (buttons 4 / 3) also on the right stick's up / down
		const uint32_t in = rbx_input(1);
		rbx_port_custom(s_port[P_IP2], 0x30, ((in & (RBX_IN_RDOWN | RBX_IN_B3)) ? 0x10 : 0) |
		                                      ((in & (RBX_IN_RUP | RBX_IN_B4)) ? 0x20 : 0));
		break;
	}
	case G_KICK: case G_CRATER:
		rbx_port_custom(s_port[P_IP1], 0xff, S.dial[0]);
		break;
	case G_KROOZR:
		rbx_port_custom(s_port[P_IP2], 0xff, S.an[0]);
		rbx_port_custom(s_port[P_IP4], 0xff, S.an[1]);
		break;
	case G_TWOTIGER:
		rbx_port_custom(s_port[P_IP2], 0xff, S.an[0]);
		rbx_port_custom(s_port[P_IP1], 0xff, S.an[1]);
		break;
	case G_WACKO: {
		const uint32_t in = rbx_input(1);
		uint8_t v = 0;
		if (in & RBX_IN_RRIGHT) v |= 0x01;
		if (in & RBX_IN_RLEFT) v |= 0x02;
		if (in & RBX_IN_RDOWN) v |= 0x04;
		if (in & RBX_IN_RUP) v |= 0x08;
		const uint32_t in2 = rbx_input(2);
		if (in2 & RBX_IN_RRIGHT) v |= 0x10;
		if (in2 & RBX_IN_RLEFT) v |= 0x20;
		if (in2 & RBX_IN_RDOWN) v |= 0x40;
		if (in2 & RBX_IN_RUP) v |= 0x80;
		rbx_port_custom(s_port[P_IP4], 0xff, v);   // (active low: the core inverts)
		break;
	}
	default: break;
	}
	if (s_g == G_DPOKER) {
		// coin_in_hit: PORT_CHANGED's newval is the bit as read (active low): the coin's release starts the 100 ms
		const uint8_t c = rport(P_IP0) & 1;
		if (c && !S.dp_prev) {
			S.coin_status |= 2;
			S.coin_timer = 6;
		}
		S.dp_prev = c;
		if (S.coin_timer && --S.coin_timer == 0)
			S.coin_status &= ~2;
		if (S.hopper_timer && --S.hopper_timer == 0) {
			if (S.dp_output & 0x40) {
				S.coin_status ^= 8;
				S.hopper_timer = (S.coin_status & 8) ? 6 : 15;
			} else
				S.coin_status &= ~8;
		}
	}
}

// ---- sound board scheduling ----
// each line: the main CPU's span [s_main0, S.main_tgt] and the boards' [s_snd0[i], S.snd_tgt[i]]; a main CPU access to a
// board's latch or status first brings the board up to the main CPU's time (MAME's synchronize / perfect_quantum)
static int64_t s_main0, s_snd0[3];

static void snd_sync_all(void)
{
	if (!s_snd)
		return;
	int64_t t = main_now();
	const int64_t span = S.main_tgt - s_main0;
	if (t > S.main_tgt)
		t = S.main_tgt;
	if (span <= 0 || t <= s_main0)
		return;
	const double f = (double)(t - s_main0) / (double)span;
	const unsigned pa = rbx_prof_enter(MC_PA_SSIO);
	if (s_snd & MSND_B_SSIO)
		msnd_ssio_run(s_snd0[0] + (int64_t)(f * (double)(S.snd_tgt[0] - s_snd0[0])));
	if (s_snd & MSND_B_TCS)
		msnd_tcs_run(s_snd0[1] + (int64_t)(f * (double)(S.snd_tgt[1] - s_snd0[1])));
	if (s_snd & MSND_B_SNT)
		msnd_snt_run(s_snd0[1] + (int64_t)(f * (double)(S.snd_tgt[1] - s_snd0[1])));
	if (s_snd & MSND_B_68K)
		msnd_68k_run(s_snd0[2] + (int64_t)(f * (double)(S.snd_tgt[2] - s_snd0[2])));
	rbx_prof_leave(pa);
}

static void snd_line_begin(void)
{
	static const uint32_t rate_mhz = 60000;
	for (int i = 0; i < 3; i++)
		s_snd0[i] = S.snd_tgt[i];
	if (s_snd & MSND_B_SSIO)
		S.snd_tgt[0] += rbx_line_cycles(&S.snd_frac[0], s_snd_clock[0], rate_mhz, FIELD_LINES);
	if (s_snd & (MSND_B_TCS | MSND_B_SNT))
		S.snd_tgt[1] += rbx_line_cycles(&S.snd_frac[1], s_snd_clock[1], rate_mhz, FIELD_LINES);
	if (s_snd & MSND_B_68K)
		S.snd_tgt[2] += rbx_line_cycles(&S.snd_frac[2], s_snd_clock[2], rate_mhz, FIELD_LINES);
}

static void snd_line(void)
{
	if (s_snd & MSND_B_SSIO) {
		rbx_prof_set(MC_PA_SSIO);
		msnd_ssio_run(S.snd_tgt[0]);
	}
	if (s_snd & (MSND_B_TCS | MSND_B_SNT)) {
		rbx_prof_set(MC_PA_SND2);
		if (s_snd & MSND_B_TCS)
			msnd_tcs_run(S.snd_tgt[1]);
		else
			msnd_snt_run(S.snd_tgt[1]);
	}
	if (s_snd & MSND_B_68K) {
		rbx_prof_set(MC_PA_SND68);
		msnd_68k_run(S.snd_tgt[2]);
	}
	rbx_prof_set(MC_PA_MIX);
	msnd_mix(s_snd);
}

// ---- main CPU ----
static void run_main(int64_t end)
{
	if (s_main.cyc > 0x40000000ul) {
		s_main.cyc -= 0x40000000ul;
		S.cyc_base += 0x40000000;
	}
	while (main_now() < end) {
		ctc_sync();
		int64_t stop = end;
		const int32_t nx = z80ctc_next(&s_ctc);
		if (nx != INT32_MAX && main_now() + nx < stop)
			stop = main_now() + nx;
		if (stop <= main_now())
			stop = main_now() + 1;
		s_live_line = S.irq_line;
		s_main.int_data = z80ctc_irq_vector(&s_ctc);
		z80_break = 0;
		// (a CTC change during the run updates s_live_line through ctc_intr)
		z80_run(&s_main, (unsigned long)(stop - S.cyc_base), &s_live_line, 1);
		z80_break = 0;
		check_ack();
	}
	ctc_sync();
}

// ---- init / reset / frame ----
static const mcfg_t *find_cfg(const char *machine, int *g)
{
	char name[64];
	snprintf(name, sizeof(name), "%s", machine);
	char *colon = strchr(name, ':');
	const char *init = colon ? colon + 1 : "";
	if (colon)
		*colon = 0;
	const mcfg_t *c = NULL;
	for (size_t i = 0; i < sizeof(s_cfgs) / sizeof(s_cfgs[0]); i++)
		if (!strcmp(s_cfgs[i].machine, name))
			c = &s_cfgs[i];
	if (!c)
		return NULL;
	int id = -1;
	for (size_t i = 0; i < sizeof(s_inits) / sizeof(s_inits[0]); i++)
		if (!strcmp(s_inits[i].name, init))
			id = s_inits[i].id;
	if (id < 0)
		return NULL;
	if (g)
		*g = id;
	return c;
}

static void soft_reset(void)
{
	const unsigned long cyc = s_main.cyc;
	z80_init(&s_main);
	s_main.ix = s_main.iy = 0xffff;   // (MAME z80: IX and IY are FFFF after a reset)
	map_main();
	s_main.cyc = cyc;
	z80ctc_reset(&s_ctc);
	S.cocktail = 0;
	S.watchdog = 0;
	if (s_snd & MSND_B_SSIO) {
		msnd_ssio_reset_write(1);
		msnd_ssio_reset_write(0);
	}
	if (s_snd & MSND_B_TCS) {
		msnd_tcs_reset_write(1);
		msnd_tcs_reset_write(0);
	}
	if (s_snd & MSND_B_68K) {
		msnd_68k_reset_write(1);
		msnd_68k_reset_write(0);
	}
}

static void mcr_reset(void)
{
	rbx_video_wait_idle();
	memset(&S, 0, sizeof(S));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_videoram, 0, sizeof(s_videoram));
	memset(s_alpharam, 0, sizeof(s_alpharam));
	memset(s_paletteram, 0, sizeof(s_paletteram));
	memset(s_pal_raw, 0, sizeof(s_pal_raw));
	memset(s_hiram, 0, sizeof(s_hiram));
	memset(s_pal_rgb, 0, sizeof(s_pal_rgb));
	rebuild_palette();
	S.an[0] = S.an[1] = 0x74;
	if (s_g == G_KROOZR) S.an[0] = S.an[1] = 100;
	if (s_g == G_TWOTIGER) S.an[0] = S.an[1] = 0x67;
	if (s_g == G_MAXRPM) { S.an[0] = S.an[1] = 0x30; S.an[2] = S.an[3] = 0x74; }
	if (s_g == G_SPYHUNT || s_g == G_TURBOTAG) { S.an[0] = 0x30; S.an[1] = 0x74; }
	S.dp_prev = 1;
	z80_init(&s_main);
	s_main.ix = s_main.iy = 0xffff;   // (MAME z80: IX and IY are FFFF after a reset)
	map_main();
	memset(&s_ctc, 0, Z80CTC_STATE_OFFSET);
	s_ctc.user = NULL;
	s_ctc.intr = ctc_intr;
	s_ctc.zc[0] = ctc_zc0;
	z80ctc_reset(&s_ctc);
	if (s_snd & MSND_B_SSIO) {
		msnd_ssio_reset();
		msnd_ssio_set_dip(rport(P_DIP));
		msnd_ssio_suspend(s_g == G_DEMODERB);
	}
	if (s_snd & MSND_B_TCS)
		msnd_tcs_reset();
	if (s_snd & MSND_B_SNT)
		msnd_snt_reset();
	if (s_snd & MSND_B_68K) {
		msnd_68k_reset();
		msnd_68k_suspend(s_g == G_TURBOTAG);
	}
	msnd_mix_reset();
	for (int i = 0; i < 2; i++)
		s_snap[i]->valid = 0;
}

static void mcr_exit(void);

static int mcr_init_common(const rbx_game *g, int mcr3)
{
	s_mcr3 = mcr3;
	s_c = find_cfg(g->machine, &s_g);
	if (!s_c) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	if (s_g == G_PLAIN) {   // (games whose inputs are the shared init's: told apart by their ports)
		if (!strncmp(g->set, "tron", 4)) s_g = G_TRON;
		else if (!strncmp(g->set, "dotron", 6)) s_g = G_DOTRON;
		else if (!strcmp(g->set, "twotigerc")) s_g = G_TWOTIGERC;
	}
	s_mirror = 0;
	if (s_g == G_SOLARFOX) s_mirror = 1;                                     // (ROT90 ^ ORIENTATION_FLIP_Y)
	if (s_g == G_KICK && strcmp(g->set, "kickc")) s_mirror = 2;             // (ORIENTATION_SWAP_XY)
	if (!strncmp(g->set, "dotron", 6) || s_g == G_CRATER) s_mirror = 1;     // (ORIENTATION_FLIP_X)
	uint32_t ssize = 0;
	s_mrom = rbx_region("maincpu", &s_mrom_size);
	s_gfx1 = rbx_region("gfx1", &s_gfx1_size);
	s_gfx2 = rbx_region("gfx2", &s_gfx2_size);
	s_gfx3 = NULL;
	s_gfx3_size = 0;
	if (s_c->cpu == CPU_SCROLL)
		s_gfx3 = rbx_region("gfx3", &s_gfx3_size);
	if (!s_mrom || s_mrom_size < 0x10000 || !s_gfx1 || !s_gfx2 || (s_c->cpu == CPU_SCROLL && !s_gfx3)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_snd = s_c->snd;
	if (s_snd & MSND_B_SSIO) {
		uint8_t *srom = rbx_region("ssio:cpu", &ssize);
		uint32_t psize = 0;
		const uint8_t *prom = rbx_region_opt("ssio:proms", &psize);
		if (!srom || msnd_ssio_init(srom, ssize, prom && psize >= 0x20 ? prom : NULL, 1.0f)) {
			rbx_fail("%s: SSIO ROM missing", g->set);
			return -1;
		}
		s_snd_clock[0] = 2000000;
	}
	if (s_snd & MSND_B_TCS) {
		uint8_t *r = rbx_region("tcs:cpu", &ssize);
		if (!r || msnd_tcs_init(r, ssize, mcr3 ? 0.8f : 1.0f)) {
			rbx_fail("%s: Turbo Cheap Squeak ROM missing", g->set);
			return -1;
		}
		s_snd_clock[1] = 2000000;
	}
	if (s_snd & MSND_B_SNT) {
		uint8_t *r = rbx_region("snt:cpu", &ssize);
		if (!r || msnd_snt_init(r, ssize, 1.0f)) {
			rbx_fail("%s: Squawk & Talk ROM missing", g->set);
			return -1;
		}
		s_snd_clock[1] = 894886;
	}
	if (s_snd & MSND_B_68K) {
		const int csd = s_c->cpu == CPU_SCROLL;
		uint8_t *r = rbx_region(csd ? "csd:cpu" : "sg:cpu", &ssize);
		if (!r || msnd_68k_init(csd ? MSND_68K_CSD : MSND_68K_SG, r, ssize, csd ? 0.8f : 0.75f)) {
			rbx_fail("%s: sound board ROM missing", g->set);
			mcr_exit();
			return -1;
		}
		s_snd_clock[2] = 8000000;
	}
	s_tiles = (uint8_t *)malloc(s_c->cpu == CPU_SCROLL ? s_gfx1_size / 256 * 512 : s_gfx1_size / 32 * 64);
	s_sprites = (uint8_t *)malloc(s_gfx2_size / 512 * 1024);
	s_spr_rows = calloc(s_gfx2_size / 512, sizeof(*s_spr_rows));
	s_snap[0] = (snap_t *)calloc(1, sizeof(snap_t));
	s_snap[1] = (snap_t *)calloc(1, sizeof(snap_t));
	s_pri = (uint8_t *)calloc(1, SCR_W * SCR_H);
	s_frame = s_mirror ? (uint16_t *)calloc(1, SCR_W * SCR_H * sizeof(uint16_t)) : NULL;
	s_alpha = s_gfx3 ? (uint8_t *)malloc(s_gfx3_size / 16 * 64) : NULL;
	s_alpha_blank = s_gfx3 ? (uint8_t *)malloc(s_gfx3_size / 16) : NULL;
	if (!s_tiles || !s_sprites || !s_spr_rows || !s_snap[0] || !s_snap[1] || !s_pri || (s_mirror && !s_frame) ||
	    (s_gfx3 && (!s_alpha || !s_alpha_blank))) {
		rbx_fail("%s: out of memory", g->set);
		mcr_exit();
		return -1;
	}
	if (s_c->cpu == CPU_SCROLL) {
		decode_scroll_tiles();
		decode_alpha();
	} else
		decode_tiles();
	decode_sprites();
	s_work = 0;
	const char *const *tags = s_c->cpu == CPU_MONO ? s_tags_mono : s_tags_mcr;
	for (int i = 0; i < P_COUNT; i++)
		s_port[i] = rbx_port_find(tags[i]);
	s_sprite_xoffs = s_g == G_SOLARFOX ? 16 : 0;
	s_sprite_xoffs_flip = (s_g == G_KICK || s_g == G_DPOKER) ? 16 : 0;
	s_spr_colormask = s_c->cpu == CPU_91475 ? 0x40 : 0;
	s_spy_sprite_color_mask = s_g == G_CRATER ? 0x03 : 0;
	s_spy_scroll_offset = s_g == G_SPYHUNT ? 16 : s_g == G_CRATER ? 96 : s_g == G_TURBOTAG ? 88 : 0;
	// video
	s_vis_w = s_c->cpu == CPU_SCROLL ? 30 * 16 : SCR_W;
	s_vis_h = SCR_H;
	// NVRAM
	s_nv_size = 0x800;
	memset(s_nvram, s_c->nvram_fill, sizeof(s_nvram));
	s_nv_path[0] = 0;
	extern const char *rbx_nvram_dir(void);
	const char *dir = rbx_nvram_dir();
	if (dir) {
		snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
		FILE *f = fopen(s_nv_path, "rb");
		if (f) {
			if (fread(s_nvram, 1, s_nv_size, f) != s_nv_size)
				memset(s_nvram, s_c->nvram_fill, sizeof(s_nvram));
			fclose(f);
		}
	}
	if (rbx_audio_setup_stereo(MSND_RATE) || rbx_video_setup(s_vis_w, s_vis_h, MAX_PAL)) {
		rbx_fail("%s: out of memory", g->set);
		mcr_exit();
		return -1;
	}
	rbx_prof_name(RBX_PA_DRV, s_pa_names, MC_PA_END - RBX_PA_DRV);
	z80_reti_hook = main_reti;
	// the aiming dials and the fire joystick on the right stick
	rbx_set_rstick_dial(s_g == G_TRON || s_g == G_DOTRON || s_g == G_DOTRONE || s_g == G_KROOZR || s_g == G_WACKO);
	mcr_reset();
	return 0;
}

static int mcr_init(const rbx_game *g) { return mcr_init_common(g, 0); }
static int mcr3_init(const rbx_game *g) { return mcr_init_common(g, 1); }

static void mcr_exit(void)
{
	rbx_video_wait_idle();
	if (s_nv_path[0]) {
		rbx_file_put_now(s_nv_path, s_nvram, s_nv_size);
		s_nv_path[0] = 0;
	}
	if (s_snd & MSND_B_68K)
		msnd_68k_exit();
	s_snd = 0;
	z80_reti_hook = NULL;
	free(s_tiles);
	free(s_sprites);
	free(s_spr_rows);
	free(s_snap[0]);
	free(s_snap[1]);
	free(s_pri);
	free(s_frame);
	free(s_alpha);
	free(s_alpha_blank);
	s_alpha_blank = NULL;
	s_tiles = s_sprites = s_pri = s_alpha = NULL;
	s_spr_rows = NULL;
	s_snap[0] = s_snap[1] = NULL;
	s_frame = NULL;
	s_c = NULL;
}

static void mcr_frame(void)
{
	rbx_audio.frame_mhz = 60000;
	inputs_field();
	// VBLANK (CTC trigger 2) every field, the 30 Hz frame (trigger 3) every other one
	ctc_sync();
	z80ctc_trg(&s_ctc, 2, 1);
	z80ctc_trg(&s_ctc, 2, 0);
	if (!(S.field & 1)) {
		z80ctc_trg(&s_ctc, 3, 1);
		z80ctc_trg(&s_ctc, 3, 0);
		if (++S.watchdog >= 16)   // (16 vblanks of the 30 Hz screen without a kick)
			soft_reset();
	}
	for (int line = 0; line < FIELD_LINES; line++) {
		S.line = (uint16_t)line;
		rbx_prof_set(MC_PA_MAIN);
		s_main0 = S.main_tgt;
		S.main_tgt += rbx_line_cycles(&S.frac, s_c->clock, 60000, FIELD_LINES);
		snd_line_begin();
		run_main(S.main_tgt);
		snd_line();
	}
	S.field++;
	present();
	rbx_prof_set(RBX_PA_DRIVER);
}

// ---- save states ----
static int mcr_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "mcr", &S, sizeof(S));
	rc |= rbx_state_io(ctx, "mcr_main", &s_main, sizeof(s_main));
	rc |= rbx_state_io(ctx, "mcr_ctc", (uint8_t *)&s_ctc + Z80CTC_STATE_OFFSET, (uint32_t)Z80CTC_STATE_SIZE);
	rc |= rbx_state_io(ctx, "mcr_nv", s_nvram, sizeof(s_nvram));
	rc |= rbx_state_io(ctx, "mcr_spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "mcr_vram", s_videoram, sizeof(s_videoram));
	rc |= rbx_state_io(ctx, "mcr_pal", s_paletteram, sizeof(s_paletteram));
	rc |= rbx_state_io(ctx, "mcr_palraw", s_pal_raw, sizeof(s_pal_raw));
	rc |= rbx_state_io(ctx, "mcr_hiram", s_hiram, sizeof(s_hiram));
	if (s_c->cpu == CPU_SCROLL)
		rc |= rbx_state_io(ctx, "mcr_alpha", s_alpharam, sizeof(s_alpharam));
	rc |= msnd_state(s_snd, ctx, load);
	if (rc)
		return -1;
	if (load) {
		map_main();
		rebuild_palette();
	}
	return 0;
}

static int mcr_save(void *ctx) { return mcr_state(ctx, 0); }
static int mcr_load(void *ctx) { return mcr_state(ctx, 1); }
static int mcr_supported(const char *machine) { return find_cfg(machine, NULL) != NULL; }

const rbx_driver rbx_drv_mcr = { "Midway MCR", mcr_init, mcr_exit, mcr_reset, mcr_frame, mcr_save, mcr_load,
                                 mcr_supported, mcr_render, NULL };
const rbx_driver rbx_drv_mcr3 = { "Midway MCR-3", mcr3_init, mcr_exit, mcr_reset, mcr_frame, mcr_save, mcr_load,
                                  mcr_supported, mcr_render, NULL };
