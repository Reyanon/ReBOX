/*  Gladiator / Ping-Pong King hardware, ported from MAME src/mame/taito/gladiatr.cpp and gladiatr_v.cpp
    (BSD-3-Clause, copyright-holders: Victor Trucco, Steve Ellenoff, Phil Stroffolino, Tatsuyuki Satoh,
    Tomasz Slanina, Nicola Salmoria, Vas Crabb).
    Z80 main (6 MHz) + Z80 sub (3 MHz) + MC6809 ADPCM CPU (12 MHz / 4, cpu/m6809) + YM2203 (1.5 MHz) + MSM5205 in VCLK
    slave mode (the 6809 toggles VCLK; a nibble is decoded on each falling edge, MAME msm5205 update_adpcm).
    Gladiator: the four UPI-41 MCUs run their dumped ROMs through cpu/mcs48 (cctl / ccpu / ucpu I8741A, csnd I8742,
    I8741A on greatgur), with MAME's wiring: the serial link between ucpu and csnd (P1 / T1 cross-connected, T0 from
    the 12 MHz / 2048 TCLK), the button latches of cctl P1 / P2, the YM2203 port A reset of cctl / ccpu, the bootleg
    MCU ROM patch and the main CPU write that pulses the sub CPU IRQ (MAME "irq patch").
    Ping-Pong King: MAME's MCU simulation (ppking_qx*), sub CPU IRQ at 60 Hz acknowledged by port 0x40.
    The ucpu / csnd pair runs in lockstep, one instruction at a time (MAME uses a perfect quantum for them).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/mcs48/mcs48.h"
#include "../../sound/rbx_ym.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

enum { M_PPKING, M_GLADIATR, M_GREATGUR };

#define SCREEN_W 256
#define VIS_Y0 16
#define VIS_Y1 239
#define SLICES 4
#define MAIN_CLOCK 6000000u
#define SUB_CLOCK 3000000u
#define ADPCM_CLOCK 750000u    // MC6809 at 12 MHz / 4, internal divider 4
#define MCU_CLOCK 400000u      // I8741 at 6 MHz, 15 clocks a machine cycle
#define BLACK_PEN 1024

typedef struct {
	z80 z;
	int irq;        // HOLD_LINE (acknowledge clears)
	int irq_level;  // ASSERT_LINE (ppking sub)
} zcpu;

typedef struct {
	int32_t mainbank, adpcmbank, sprite_buffer, sprite_bank, flip;
	int32_t video_attributes, fg_scrollx, fg_scrolly, bg_scrollx, bg_scrolly, fg_tile_bank, bg_tile_bank;
	int32_t ppk_scrolly[16];
	uint8_t soundlatch, soundlatch2, adpcm_nmi, ym_nmi;
	uint8_t cctl_p1, cctl_p2, ucpu_p1, csnd_p1;
	int32_t mcu_reset;            // cctl / ccpu held in reset (YM2203 port A bit 7 low)
	int32_t in_prev[2];
	// MSM5205
	int32_t msm_signal, msm_step, msm_data, msm_reset, msm_vck;
	// ppking MCU simulation
	uint8_t mcu_rxd[2], mcu_txd[2], mcu_rst[2], mcu_state[2], mcu_packet[2];
	uint8_t pad[2];
	int64_t mcu_t[4], mcu_debt[2];
	int64_t adpcm_target;
	uint64_t frac_main, frac_sub, frac_adpcm, frac_mcu;
} misc_t;

static const struct { const char *name; int kind; } s_machines[] = {
	{ "ppking", M_PPKING }, { "gladiatr", M_GLADIATR }, { "greatgur", M_GREATGUR },
};

static int s_kind, s_lines;
static uint32_t s_rate_mhz;
static uint8_t *s_main_rom, *s_sub_rom, *s_audio_rom;
static uint32_t s_main_size, s_sub_size, s_audio_size;
static uint8_t s_spriteram[0xc00], s_pal[0x400], s_pal_ext[0x400], s_videoram[0x800], s_colorram[0x800];
static uint8_t s_textram[0x800], s_nvram[0x800], s_sub_ram[0x400];
static misc_t s_st;
static zcpu s_main, s_sub;
static m6809_t s_adpcm;
static mcs48 s_mcu[4];   // cctl, ccpu, ucpu, csnd
enum { CCTL, CCPU, UCPU, CSND };
// video
static uint8_t *s_tx, *s_bg, *s_spr;   // decoded 1 byte / pixel
static uint32_t s_tx_n, s_bg_n, s_spr_n;
static uint8_t *s_spr_opaque;
typedef struct {
	uint8_t spriteram[0xc00], videoram[0x800], colorram[0x800], textram[0x800];
	uint32_t rgb[1024];
	misc_t st;
} snap_t;
static snap_t s_snap;
// sound
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static uint32_t s_ym_done;
static unsigned long s_sub_frame_start;
static int64_t s_adpcm_frame_start;
static int s_msm_diff[49 * 16];
// ports
static int s_p_dsw1, s_p_dsw2, s_p_dsw3, s_p_in0, s_p_in1, s_p_in2, s_p_coins, s_p_p1, s_p_p2, s_p_system;

static uint8_t port8(int p) { return (uint8_t)rbx_port_read(p); }
static uint8_t bitswap8(uint8_t v, int b7, int b6, int b5, int b4, int b3, int b2, int b1, int b0)
{
	return (uint8_t)(((v >> b7) & 1) << 7 | ((v >> b6) & 1) << 6 | ((v >> b5) & 1) << 5 | ((v >> b4) & 1) << 4 |
	                 ((v >> b3) & 1) << 3 | ((v >> b2) & 1) << 2 | ((v >> b1) & 1) << 1 | ((v >> b0) & 1));
}

// ---- sound: YM2203 (driven by the sub CPU clock) + MSM5205 sample-and-hold ----
static void ym_run_to(uint32_t pos)
{
	if (pos <= s_ym_done)
		return;
	int n0 = s_mix_n;
	s_mix_n += rbx_ym_advance(0, pos - s_ym_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	int32_t msm = (s_st.msm_signal & ~3) * 8 * 154 / 256;   // route 0.60
	for (int i = n0; i < s_mix_n; i++)
		s_mix[i] += msm;
	s_ym_done = pos;
}
static void ym_sync_sub(void) { ym_run_to((uint32_t)((s_sub.z.cyc - s_sub_frame_start) / 2)); }
static void ym_sync_adpcm(void) { ym_run_to((uint32_t)((uint64_t)(s_adpcm.cycles - s_adpcm_frame_start) * 2)); }

static void ym_irq(int state)
{
	// gladiatr_state_base::ym_irq: NMI on the sub CPU
	if (state && !s_st.ym_nmi)
		s_sub.z.nmi_pending = 1;
	s_st.ym_nmi = (uint8_t)(state != 0);
}

static void mcu_reset_line(int state)
{
	if (state && !s_st.mcu_reset) {
		mcs48_reset(&s_mcu[CCPU]);
		mcs48_reset(&s_mcu[CCTL]);
	}
	s_st.mcu_reset = state;
}

static void ym_port_a_w(uint8_t data)
{
	if (s_kind != M_PPKING)
		mcu_reset_line(!(data & 0x80));   // gladiator_int_control_w
}
static uint8_t ym_port_a_r(void) { return 0xff; }   // ppking_f1_r
static uint8_t ym_port_b_r(void) { return port8(s_p_dsw3); }

static void msm_init_tables(void)
{
	static const int nbl2bit[16][4] = {
		{ 1, 0, 0, 0}, { 1, 0, 0, 1}, { 1, 0, 1, 0}, { 1, 0, 1, 1}, { 1, 1, 0, 0}, { 1, 1, 0, 1}, { 1, 1, 1, 0}, { 1, 1, 1, 1},
		{-1, 0, 0, 0}, {-1, 0, 0, 1}, {-1, 0, 1, 0}, {-1, 0, 1, 1}, {-1, 1, 0, 0}, {-1, 1, 0, 1}, {-1, 1, 1, 0}, {-1, 1, 1, 1}
	};
	for (int step = 0; step <= 48; step++) {
		int stepval = (int)floor(16.0 * pow(11.0 / 10.0, (double)step));
		for (int nib = 0; nib < 16; nib++)
			s_msm_diff[step * 16 + nib] = nbl2bit[nib][0] *
				(stepval * nbl2bit[nib][1] + stepval / 2 * nbl2bit[nib][2] + stepval / 4 * nbl2bit[nib][3] + stepval / 8);
	}
}

// MAME msm5205 update_adpcm (4-bit)
static void msm_capture(void)
{
	static const int index_shift[8] = { -1, -1, -1, -1, 2, 4, 6, 8 };
	if (s_st.msm_reset) {
		s_st.msm_signal = 0;
		s_st.msm_step = 0;
		return;
	}
	int val = s_st.msm_data & 15;
	int s = s_st.msm_signal + s_msm_diff[s_st.msm_step * 16 + val];
	if (s > 2047) s = 2047;
	else if (s < -2048) s = -2048;
	s_st.msm_signal = s;
	s_st.msm_step += index_shift[val & 7];
	if (s_st.msm_step > 48) s_st.msm_step = 48;
	else if (s_st.msm_step < 0) s_st.msm_step = 0;
}

static void adpcm_w(uint8_t data)
{
	if (s_kind != M_PPKING)
		s_st.adpcmbank = (data >> 6) & 1;
	s_st.msm_data = data & 0x0f;
	s_st.msm_reset = (data >> 5) & 1;
	int vck = (data >> 4) & 1;
	if (s_st.msm_vck && !vck) {   // capture on the falling VCLK edge
		ym_sync_adpcm();
		msm_capture();
	}
	s_st.msm_vck = vck;
}

static void adpcm_command_w(uint8_t data)
{
	s_st.soundlatch = data;
	if (!s_st.adpcm_nmi)
		m6809_set_nmi(&s_adpcm, 1);
	s_st.adpcm_nmi = 1;
}

// ---- 6809 (ADPCM) ----
static uint8_t adpcm_read(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0x2000 && a < 0x3000) {   // adpcm_command_r
		m6809_set_nmi(&s_adpcm, 0);
		s_st.adpcm_nmi = 0;
		return s_st.soundlatch;
	}
	if (s_kind == M_PPKING)
		return a >= 0x8000 ? s_audio_rom[a] : 0;
	if (a >= 0x4000) {
		uint32_t o = 0x10000u + (uint32_t)s_st.adpcmbank * 0xc000u + (a - 0x4000u);
		return o < s_audio_size ? s_audio_rom[o] : 0;
	}
	return 0;
}
static void adpcm_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x1000 && a < 0x2000)
		adpcm_w(v);
}
static void adpcm_cpu_reset(void)
{
	m6809_init(&s_adpcm);
	s_adpcm.read = adpcm_read;
	s_adpcm.write = adpcm_write;
	m6809_reset(&s_adpcm);
	s_st.adpcm_target = 0;
}

// ---- MCUs (gladiatr) ----
static int tclk_at(int64_t t) { return (int)(((uint64_t)t * 15u / 512u) & 1); }   // 12 MHz / 8 / 128 / 2 square wave

static uint8_t mcu_port_r(void *user, int port)
{
	int id = (int)(intptr_t)user;
	switch (id) {
	case CCTL: return port == 1 ? (uint8_t)(s_st.cctl_p1 & port8(s_p_in2)) : s_st.cctl_p2;
	case CCPU: return port == 1 ? port8(s_p_in0) : port8(s_p_in1);
	case UCPU:
		if (port == 1) return s_st.csnd_p1 |= 0xfe;
		return bitswap8(port8(s_p_dsw1), 0, 1, 2, 3, 4, 5, 6, 7);
	default:
		if (port == 1) return s_st.ucpu_p1 |= 0xfe;
		return bitswap8(port8(s_p_dsw2), 2, 3, 4, 5, 6, 7, 1, 0);
	}
}
static void mcu_port_w(void *user, int port, uint8_t v)
{
	int id = (int)(intptr_t)user;
	if (id == UCPU && port == 1) s_st.ucpu_p1 = v;
	else if (id == CSND && port == 1) s_st.csnd_p1 = v;
	// ccpu P2: coin counters (not emulated)
}
static int mcu_test_r(void *user, int t)
{
	int id = (int)(intptr_t)user;
	uint8_t coins = port8(s_p_coins);
	switch (id) {
	case CCTL: return t == 0 ? (coins >> 3) & 1 : (coins >> 2) & 1;
	case CCPU: return t == 0 ? (coins >> 1) & 1 : coins & 1;
	case UCPU: return t == 0 ? tclk_at(s_st.mcu_t[UCPU]) : (s_st.csnd_p1 >> 1) & 1;
	default: return t == 0 ? tclk_at(s_st.mcu_t[CSND]) : (s_st.ucpu_p1 >> 1) & 1;
	}
}

// button latches (MAME p1_s1 / p1_s2 / p2_s1 / p2_s2 input-changed handlers)
static void poll_buttons(void)
{
	for (int p = 0; p < 2; p++) {
		int v = port8(p ? s_p_in1 : s_p_in0);
		int old = s_st.in_prev[p];
		uint8_t *q = p ? &s_st.cctl_p2 : &s_st.cctl_p1;
		if ((v ^ old) & 0x20)   // S2: bit 0 is high while pressed
			*q = (uint8_t)((*q & 0xfe) | ((v & 0x20) ? 0 : 1));
		if ((old & 0x10) && !(v & 0x10))   // S1 pressed: bit 1 = bit 0
			*q = (uint8_t)((*q & 0xfd) | ((*q & 1) << 1));
		s_st.in_prev[p] = v;
	}
}

static void mcus_run(int cycles)
{
	if (s_kind == M_PPKING)
		return;
	// ucpu / csnd: lockstep (polled serial link)
	int64_t target = s_st.mcu_t[CCTL] + cycles;   // mcu_t[CCTL] / [CCPU] follow the frame clock exactly
	while (s_st.mcu_t[UCPU] < target || s_st.mcu_t[CSND] < target) {
		int id = s_st.mcu_t[UCPU] <= s_st.mcu_t[CSND] ? UCPU : CSND;
		s_st.mcu_t[id] += mcs48_execute(&s_mcu[id], 1);
	}
	for (int id = CCTL; id <= CCPU; id++) {
		if (s_st.mcu_reset) {
			s_st.mcu_t[id] += cycles;
			continue;
		}
		int n = cycles - (int)s_st.mcu_debt[id];
		if (n > 0) {
			int ran = mcs48_execute(&s_mcu[id], n);
			s_st.mcu_debt[id] = ran - n;
		} else
			s_st.mcu_debt[id] = -n;
		s_st.mcu_t[id] += cycles;
	}
}

// ---- Ping-Pong King MCU simulation (MAME ppking_qx*) ----
static void ppk_input_check(void)
{
	if (s_st.mcu_txd[0] == 0x41) s_st.mcu_state[0] = 1;
	else if (s_st.mcu_txd[0] == 0x42) s_st.mcu_state[0] = 2;
	else if (s_st.mcu_txd[0] == 0x44) s_st.mcu_state[0] = 3;
}

static uint8_t ppk_qx0_r(int offset)
{
	if (offset == 1)
		return 1;
	if (s_st.mcu_rst[0]) {
		switch (s_st.mcu_state[0]) {
		case 1:
			s_st.mcu_packet[0] ^= 1;
			if (s_st.mcu_packet[0] & 1) s_st.mcu_rxd[0] = (uint8_t)((port8(s_p_dsw3) & 0x9f) | 0x20);
			else s_st.mcu_rxd[0] = (uint8_t)(port8(s_p_system) & 0x9f);
			break;
		case 2:
		case 3:
			s_st.mcu_packet[0] ^= 1;
			if (s_st.mcu_packet[0] & 1) s_st.mcu_rxd[0] = 0x17;
			else s_st.mcu_rxd[0] = (uint8_t)((port8(s_st.mcu_state[0] == 2 ? s_p_p1 : s_p_p2) & 0x3f) |
			                                 (port8(s_p_system) & 0x80));
			break;
		}
		int bits = 0;
		for (int i = 0; i < 8; i++) bits += (s_st.mcu_rxd[0] >> i) & 1;
		if (bits % 2) s_st.mcu_rxd[0] |= 0x40;
	}
	return s_st.mcu_rxd[0];
}

static void ppk_qx0_w(int offset, uint8_t data)
{
	if (offset == 1) {
		switch (data) {
		case 0: s_st.mcu_rxd[0] = 0x40; s_st.mcu_rst[0] = 0; s_st.mcu_state[0] = 0; break;
		case 1: s_st.mcu_rxd[0] = 0x40; s_st.mcu_rst[0] = 0; break;
		case 2: s_st.mcu_rxd[0] = (uint8_t)((port8(s_p_dsw2) & 0x1f) << 2); s_st.mcu_rst[0] = 0; break;
		case 3: ppk_input_check(); s_st.mcu_rst[0] = 1; break;
		}
	} else {
		s_st.mcu_txd[0] = data;
		s_st.mcu_rst[1] = 0;
		s_st.soundlatch2 = data;
		ppk_input_check();
	}
}

// ---- main CPU ----
static void mainlatch_w(int bit, int state)
{
	switch (bit) {
	case 0: s_st.sprite_buffer = state; break;
	case 1: if (s_kind != M_PPKING) s_st.sprite_bank = state ? 4 : 2; break;
	case 2: if (s_kind != M_PPKING) s_st.mainbank = state; break;
	case 7: s_st.flip = state; break;
	}
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (s_kind == M_PPKING) {
		if (a < 0xc000) return a < s_main_size ? s_main_rom[a] : 0;
	} else {
		if (a < 0x6000) return s_main_rom[a];
		if (a < 0xc000) {
			uint32_t o = 0x10000u + (uint32_t)s_st.mainbank * 0x6000u + (a - 0x6000u);
			return o < s_main_size ? s_main_rom[o] : 0;
		}
	}
	if (a < 0xcc00) return s_spriteram[a - 0xc000];
	if (a < 0xd000) return 0;
	if (a < 0xd400) return s_pal[a - 0xd000];
	if (a < 0xd800) return s_pal_ext[a - 0xd400];
	if (a < 0xe000) return s_videoram[a - 0xd800];
	if (a < 0xe800) return s_colorram[a - 0xe000];
	if (a < 0xf000) return s_textram[a - 0xe800];
	if (a < 0xf800) return s_nvram[a - 0xf000];
	return 0;
}

static void video_registers_w(int offset, uint8_t data)
{
	if (s_kind == M_PPKING) {
		switch (offset & 0x300) {
		case 0x000: s_st.ppk_scrolly[offset & 0x0f] = 0x100 - data; break;
		case 0x200: s_st.fg_scrolly = (data & 0x80) ? data + 0x100 : data; break;
		case 0x300: s_st.fg_tile_bank = data & 0x03; s_st.video_attributes = data; break;
		}
		return;
	}
	switch (offset) {
	case 0x000: s_st.fg_scrolly = data; break;
	case 0x080:
		s_st.fg_tile_bank = data & 0x03;
		s_st.bg_tile_bank = (data & 0x10) >> 4;
		s_st.video_attributes = data;
		break;
	case 0x100: s_st.fg_scrollx = data; break;
	case 0x200: s_st.bg_scrolly = data; break;
	case 0x300: s_st.bg_scrollx = data; break;
	}
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xc000) return;
	if (a < 0xcc00) { s_spriteram[a - 0xc000] = v; return; }
	if (a < 0xd000) { video_registers_w(a - 0xcc00, v); return; }
	if (a < 0xd400) { s_pal[a - 0xd000] = v; return; }
	if (a < 0xd800) { s_pal_ext[a - 0xd400] = v; return; }
	if (a < 0xe000) { s_videoram[a - 0xd800] = v; return; }
	if (a < 0xe800) { s_colorram[a - 0xe000] = v; return; }
	if (a < 0xf000) { s_textram[a - 0xe800] = v; return; }
	if (a < 0xf800) { s_nvram[a - 0xf000] = v; return; }
}

// main I/O: the boards decode c0xx; the Z80 core passes the low address byte only
static uint8_t main_in(z80 *z, uint8_t p)
{
	(void)z;
	if (p == 0x9e || p == 0x9f) {
		if (s_kind == M_PPKING) return ppk_qx0_r(p & 1);
		return upi41_master_r(&s_mcu[UCPU], p & 1);
	}
	if (s_kind == M_PPKING && (p == 0xc0 || p == 0xc1))
		return (p & 1) ? 1 : 0;   // ppking_qxcomu_r
	return 0;
}

static void main_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (p < 0x08) {
		if (p == 4 && s_kind != M_PPKING) {
			s_sub.irq = 1;   // gladiatr_irq_patch_w: HOLD_LINE on the sub CPU
			return;
		}
		mainlatch_w(p, v & 1);
		return;
	}
	if (p == 0x9e || p == 0x9f) {
		if (s_kind == M_PPKING) ppk_qx0_w(p & 1, v);
		else upi41_master_w(&s_mcu[UCPU], p & 1, v);
	}
}

// ---- sub CPU ----
static uint8_t sub_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_sub_size ? s_sub_rom[a] : 0;
	if (a < 0x8400) return s_sub_ram[a - 0x8000];
	return 0;
}
static void sub_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8400) s_sub_ram[a - 0x8000] = v;
}

static uint8_t sub_in(z80 *z, uint8_t p)
{
	(void)z;
	if (p <= 0x01) { ym_sync_sub(); return rbx_ym_read(0, p & 1); }
	if (s_kind == M_PPKING) {
		if (p == 0x20 || p == 0x21) {   // ppking_qx1_r
			if (p & 1) return 1;
			return s_st.mcu_rst[1] == 1 ? 0x40 : s_st.soundlatch2;
		}
		if (p == 0x80 || p == 0x81) return (p & 1) ? 1 : 0;   // ppking_qx3_r
		return 0;
	}
	if (p == 0x20 || p == 0x21) return upi41_master_r(&s_mcu[CSND], p & 1);
	if (p == 0x60 || p == 0x61) return upi41_master_r(&s_mcu[CCTL], p & 1);
	if (p == 0x80 || p == 0x81) return upi41_master_r(&s_mcu[CCPU], p & 1);
	return 0;
}

static void sub_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (p <= 0x01) { ym_sync_sub(); rbx_ym_write(0, p & 1, v); return; }
	if (p == 0xe0) { adpcm_command_w(v); return; }
	if (s_kind == M_PPKING) {
		if (p == 0x21 && v == 0xf0) s_st.mcu_rst[1] = 1;   // ppking_qx1_w
		else if (p == 0x40) s_sub.irq_level = 0;           // cpu2_irq_ack_w
		return;
	}
	if (p == 0x20 || p == 0x21) upi41_master_w(&s_mcu[CSND], p & 1, v);
	else if (p == 0x60 || p == 0x61) upi41_master_w(&s_mcu[CCTL], p & 1, v);
	else if (p == 0x80 || p == 0x81) upi41_master_w(&s_mcu[CCPU], p & 1, v);
	// 0xa0-0xa7: filter latch (not emulated)
}

static void zcpu_init(zcpu *c, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t),
                      uint8_t (*in)(z80 *, uint8_t), void (*out)(z80 *, uint8_t, uint8_t))
{
	unsigned long cyc = c->z.cyc;
	z80_init(&c->z);
	c->z.read_byte = rd;
	c->z.write_byte = wr;
	c->z.port_in = in;
	c->z.port_out = out;
	c->z.int_data = 0xff;
	c->z.cyc = cyc;
	c->irq = c->irq_level = 0;
}

static void zcpu_run(zcpu *c, int cycles)
{
	unsigned long end = c->z.cyc + (unsigned long)cycles;
	while ((long)(c->z.cyc - end) < 0) {
		c->z.int_pending = (c->irq || c->irq_level) ? 1 : 0;
		z80_step(&c->z);
		if (c->irq && !c->z.int_pending)
			c->irq = 0;   // HOLD_LINE acknowledged
	}
}

// ---- video ----
static uint32_t pal5(int v) { return (uint32_t)((v << 3) | (v >> 2)); }

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap.spriteram, s_spriteram, sizeof(s_spriteram));
	memcpy(s_snap.videoram, s_videoram, sizeof(s_videoram));
	memcpy(s_snap.colorram, s_colorram, sizeof(s_colorram));
	memcpy(s_snap.textram, s_textram, sizeof(s_textram));
	for (int i = 0; i < 1024; i++) {   // xBGRBBBBGGGGRRRR_bit0
		int raw = s_pal[i] | s_pal_ext[i] << 8;
		int r = ((raw << 1) & 0x1e) | ((raw >> 12) & 1);
		int g = ((raw >> 3) & 0x1e) | ((raw >> 13) & 1);
		int b = ((raw >> 7) & 0x1e) | ((raw >> 14) & 1);
		s_snap.rgb[i] = pal5(r) << 16 | pal5(g) << 8 | pal5(b);
	}
	s_snap.st = s_st;
	rbx_video_frame_done();
}

static int wrap(int v, int m) { v %= m; return v < 0 ? v + m : v; }

// MAME tilemap mapping of a screen coordinate to a tilemap coordinate (effective_rowscroll / colscroll)
static int map_x(int sx, int scroll, int dx, int dxf, int w, int flip)
{
	if (!flip)
		return wrap(sx - (dx - scroll), w);
	return w - 1 - wrap(sx - (SCREEN_W - w - (dxf - scroll)), w);
}
static int map_y(int sy, int scroll, int h, int flip)
{
	if (!flip)
		return wrap(sy + scroll, h);
	return h - 1 - wrap(sy - (256 - h + scroll), h);
}

static void draw_sprites(uint16_t *bm, int pitch, const snap_t *s)
{
	static const int tile_offset[2][2] = { { 0, 1 }, { 2, 3 } };
	const misc_t *st = &s->st;
	for (int offs = 0; offs < 0x80; offs += 2) {
		const uint8_t *src = &s->spriteram[offs + (st->sprite_buffer << 7)];
		int attributes = src[0x800];
		int size = (attributes >> 4) & 1;
		int bank = (attributes & 1) + ((attributes & 2) ? st->sprite_bank : 0);
		int tile_number = src[0] + 256 * bank;
		int sx = src[0x401] + 256 * (src[0x801] & 1) - 0x38;
		int sy = 240 - src[0x400] - (size ? 16 : 0);
		int xflip = (attributes >> 2) & 1, yflip = (attributes >> 3) & 1;
		int color = 0x100 + (src[1] & 0x1f) * 8;
		if (st->flip) { xflip = !xflip; yflip = !yflip; }
		for (int y = 0; y <= size; y++)
			for (int x = 0; x <= size; x++) {
				int ex = xflip ? size - x : x, ey = yflip ? size - y : y;
				uint32_t t = (uint32_t)(tile_offset[ey][ex] + tile_number) % s_spr_n;
				if (!s_spr_opaque[t])
					continue;
				const uint8_t *g = s_spr + t * 256;
				for (int pass = 0; pass < 2; pass++) {   // second pass: wraparound (ppking)
					int x0 = sx + x * 16, y0 = sy + y * 16 + pass * 256;
					for (int py = 0; py < 16; py++) {
						int dy = y0 + py;
						if (dy < VIS_Y0 || dy > VIS_Y1)
							continue;
						const uint8_t *row = g + (yflip ? 15 - py : py) * 16;
						uint16_t *d = bm + (dy - VIS_Y0) * pitch;
						for (int px = 0; px < 16; px++) {
							int dx = x0 + px;
							if (dx < 0 || dx >= SCREEN_W)
								continue;
							int c = row[xflip ? 15 - px : px];
							if (c)
								d[dx] = (uint16_t)(color + c);
						}
					}
				}
			}
	}
}

static void gl_render(void)
{
	const snap_t *s = &s_snap;
	const misc_t *st = &s->st;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, h = VIS_Y1 - VIS_Y0 + 1;
	memcpy(rbx_video.palette, s->rgb, sizeof(s->rgb));
	rbx_video.palette[BLACK_PEN] = 0;
	if (s_kind == M_PPKING) {
		// bg: 32x64 tilemap, 16 scroll columns
		for (int y = 0; y < h; y++) {
			uint16_t *d = bm + y * pitch;
			int sy = y + VIS_Y0;
			for (int x = 0; x < SCREEN_W; x++) {
				int tx = st->flip ? SCREEN_W - 1 - x : x;
				int ty = map_y(sy, st->ppk_scrolly[tx >> 4], 512, st->flip);
				int idx = (ty >> 3) * 32 + (tx >> 3);
				int attr = s->colorram[idx];
				uint32_t code = (uint32_t)(s->videoram[idx] + ((attr & 7) << 8)) % s_bg_n;
				d[x] = (uint16_t)(((attr >> 3) ^ 0x1f) * 8 + s_bg[code * 64 + (ty & 7) * 8 + (tx & 7)]);
			}
		}
		draw_sprites(bm, pitch, s);
		// the fg layer selects the upper palette bank on the pixels below it
		for (int y = 0; y < h; y++) {
			uint16_t *d = bm + y * pitch;
			int ty = (y + VIS_Y0 + st->fg_scrolly) & 0x1ff;
			for (int x = 0; x < SCREEN_W; x++) {
				int idx = (ty >> 3) * 32 + (x >> 3);
				uint32_t code = (uint32_t)(s->textram[idx] + (st->fg_tile_bank << 8)) % s_tx_n;
				if (s_tx[code * 64 + (ty & 7) * 8 + (x & 7)])
					d[x] += 512;
			}
		}
		return;
	}
	if (!(st->video_attributes & 0x20)) {
		for (int y = 0; y < h; y++)
			for (int x = 0; x < SCREEN_W; x++)
				bm[y * pitch + x] = BLACK_PEN;
		return;
	}
	int bgsx = (st->bg_scrollx + ((st->video_attributes & 0x04) << 6)) ^ (st->flip ? 0x0f : 0);
	int fgsx = (st->fg_scrollx + ((st->video_attributes & 0x08) << 5)) ^ (st->flip ? 0x0f : 0);
	int bgbank = st->bg_tile_bank << 11, fgbank = st->fg_tile_bank << 8;
	for (int y = 0; y < h; y++) {
		uint16_t *d = bm + y * pitch;
		int ty = map_y(y + VIS_Y0, st->bg_scrolly, 256, st->flip);
		for (int x = 0; x < SCREEN_W; x++) {
			int tx = map_x(x, bgsx, -0x30, 0x12f, 512, st->flip);
			int idx = (ty >> 3) * 64 + (tx >> 3);
			int attr = s->colorram[idx];
			uint32_t code = (uint32_t)(s->videoram[idx] + ((attr & 7) << 8) + bgbank) % s_bg_n;
			d[x] = (uint16_t)(((attr >> 3) ^ 0x1f) * 8 + s_bg[code * 64 + (ty & 7) * 8 + (tx & 7)]);
		}
	}
	draw_sprites(bm, pitch, s);
	for (int y = 0; y < h; y++) {
		uint16_t *d = bm + y * pitch;
		int ty = map_y(y + VIS_Y0, st->fg_scrolly, 256, st->flip);
		for (int x = 0; x < SCREEN_W; x++) {
			int tx = map_x(x, fgsx, -0x30, 0x12f, 512, st->flip);
			int idx = (ty >> 3) * 64 + (tx >> 3);
			uint32_t code = (uint32_t)(s->textram[idx] + fgbank) % s_tx_n;
			int c = s_tx[code * 64 + (ty & 7) * 8 + (tx & 7)];
			if (c)
				d[x] = (uint16_t)(0x200 + c);
		}
	}
}

// ---- graphics decode ----
static int gbit(const uint8_t *rom, uint32_t size, uint32_t bit)
{
	return (bit >> 3) < size && (rom[bit >> 3] & (0x80 >> (bit & 7))) ? 1 : 0;
}

static void swap_ranges(uint8_t *a, uint8_t *a_end, uint8_t *b)
{
	for (; a < a_end; a++, b++) { uint8_t t = *a; *a = *b; *b = t; }
}

static int decode_gfx(void)
{
	uint32_t txs, bgs, sps;
	uint8_t *tx = rbx_region("tx_tiles", &txs), *bg = rbx_region("bg_tiles", &bgs), *sp = rbx_region("sprites", &sps);
	if (!tx || !bg || !sp)
		return -1;
	if (s_kind == M_PPKING) {   // init_ppking
		for (int i = 0; i < 0x2000; i++)
			bg[i + 0x2000] = bg[i] >> 4;
		for (int j = 1; j >= 0; j--)
			for (int i = 0; i < 0x2000; i++) {
				sp[i + (2 * j + 1) * 0x2000] = sp[i + j * 0x2000] >> 4;
				sp[i + 2 * j * 0x2000] = sp[i + j * 0x2000];
			}
	} else {   // init_gladiatr
		for (int j = 3; j >= 0; j--)
			for (int i = 0; i < 0x2000; i++) {
				bg[i + (2 * j + 1) * 0x2000] = bg[i + j * 0x2000] >> 4;
				bg[i + 2 * j * 0x2000] = bg[i + j * 0x2000];
			}
		swap_ranges(bg + 0x14000, bg + 0x18000, bg + 0x18000);
		for (int j = 5; j >= 0; j--)
			for (int i = 0; i < 0x2000; i++) {
				sp[i + (2 * j + 1) * 0x2000] = sp[i + j * 0x2000] >> 4;
				sp[i + 2 * j * 0x2000] = sp[i + j * 0x2000];
			}
		swap_ranges(sp + 0x1a000, sp + 0x1c000, sp + 0x1c000);
		swap_ranges(sp + 0x22000, sp + 0x24000, sp + 0x28000);
		swap_ranges(sp + 0x26000, sp + 0x28000, sp + 0x2c000);
		swap_ranges(sp + 0x24000, sp + 0x28000, sp + 0x28000);
	}
	s_tx_n = txs / 8;
	s_bg_n = bgs / 2 * 8 / 128;
	s_spr_n = sps / 2 * 8 / 512;
	s_tx = (uint8_t *)malloc((size_t)s_tx_n * 64);
	s_bg = (uint8_t *)malloc((size_t)s_bg_n * 64);
	s_spr = (uint8_t *)malloc((size_t)s_spr_n * 256);
	s_spr_opaque = (uint8_t *)calloc(s_spr_n, 1);
	if (!s_tx || !s_bg || !s_spr || !s_spr_opaque)
		return -1;
	for (uint32_t t = 0; t < s_tx_n; t++)   // gfx_8x8x1
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++)
				s_tx[t * 64 + y * 8 + x] = (uint8_t)gbit(tx, txs, t * 64 + y * 8 + x);
	static const int xo[16] = { 0, 1, 2, 3, 64, 65, 66, 67, 128, 129, 130, 131, 192, 193, 194, 195 };
	uint32_t half = bgs / 2 * 8;
	for (uint32_t t = 0; t < s_bg_n; t++)   // tilelayout: planes { 4, half, half + 4 }
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				uint32_t b = t * 128 + (uint32_t)y * 8 + (uint32_t)xo[x];
				s_bg[t * 64 + y * 8 + x] = (uint8_t)(gbit(bg, bgs, b + 4) << 2 | gbit(bg, bgs, b + half) << 1 |
				                                     gbit(bg, bgs, b + half + 4));
			}
	half = sps / 2 * 8;
	for (uint32_t t = 0; t < s_spr_n; t++) {   // spritelayout
		int any = 0;
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				uint32_t b = t * 512 + (uint32_t)(y < 8 ? y * 8 : 256 + (y - 8) * 8) + (uint32_t)xo[x];
				int v = gbit(sp, sps, b + 4) << 2 | gbit(sp, sps, b + half) << 1 | gbit(sp, sps, b + half + 4);
				s_spr[t * 256 + y * 16 + x] = (uint8_t)v;
				any |= v;
			}
		s_spr_opaque[t] = any != 0;
	}
	return 0;
}

// ---- init / reset / frame ----
static void gl_exit(void)
{
	free(s_tx); free(s_bg); free(s_spr); free(s_spr_opaque); free(s_mix);
	s_tx = s_bg = s_spr = s_spr_opaque = NULL;
	s_mix = NULL;
	rbx_ym_exit(0);
}

static void gl_reset(void)
{
	memset(&s_st, 0, sizeof(s_st));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_pal_ext, 0, sizeof(s_pal_ext));
	memset(s_videoram, 0, sizeof(s_videoram));
	memset(s_colorram, 0, sizeof(s_colorram));
	memset(s_textram, 0, sizeof(s_textram));
	memset(s_nvram, 0, sizeof(s_nvram));
	memset(s_sub_ram, 0, sizeof(s_sub_ram));
	memset(&s_main, 0, sizeof(s_main));
	memset(&s_sub, 0, sizeof(s_sub));
	zcpu_init(&s_main, main_read, main_write, main_in, main_out);
	zcpu_init(&s_sub, sub_read, sub_write, sub_in, sub_out);
	s_st.sprite_bank = s_kind == M_PPKING ? 1 : 2;
	s_st.cctl_p1 = s_st.cctl_p2 = s_st.ucpu_p1 = s_st.csnd_p1 = 0xff;
	s_st.in_prev[0] = s_st.in_prev[1] = 0xff;
	adpcm_cpu_reset();
	if (s_kind == M_PPKING)   // ppking_state::machine_reset
		s_st.mcu_rxd[0] = (uint8_t)((port8(s_p_dsw1) & 0x1f) << 2);
	else
		for (int i = 0; i < 4; i++)
			mcs48_reset(&s_mcu[i]);
	rbx_ym_reset(0);
	s_ym_done = 0;
	s_mix_n = 0;
	s_sub_frame_start = s_sub.z.cyc;
	s_adpcm_frame_start = s_adpcm.cycles;
}

static int setup_mcu(int id, const char *region, int type, const rbx_game *g)
{
	uint32_t size;
	uint8_t *rom = rbx_region(region, &size);
	if (!rom || size < 0x400) {
		rbx_fail("%s: MCU ROM %s missing", g->set, region);
		return -1;
	}
	if (id == CCTL || id == CCPU)
		rom[0] = 0x22;   // MAME init_gladiatr: suspected bad bootleg dumps
	mcs48_init(&s_mcu[id], type);
	s_mcu[id].rom = rom;
	s_mcu[id].userdata = (void *)(intptr_t)id;
	s_mcu[id].port_r = mcu_port_r;
	s_mcu[id].port_w = mcu_port_w;
	s_mcu[id].test_r = mcu_test_r;
	return 0;
}

static int gl_init(const rbx_game *g)
{
	s_kind = -1;
	for (size_t i = 0; i < sizeof(s_machines) / sizeof(s_machines[0]); i++)
		if (!strcmp(s_machines[i].name, g->machine))
			s_kind = s_machines[i].kind;
	if (s_kind < 0) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_main_rom = rbx_region("maincpu", &s_main_size);
	s_sub_rom = rbx_region("sub", &s_sub_size);
	s_audio_rom = rbx_region("audiocpu", &s_audio_size);
	if (!s_main_rom || !s_sub_rom || !s_audio_rom || s_audio_size < 0x10000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (s_kind == M_PPKING) {
		// patch audio CPU crash + ROM checksums (MAME init_ppking)
		static const uint16_t patch[] = { 0x1b9, 0x1ba, 0x1bb, 0x839, 0x83a, 0x83b, 0x845, 0x846, 0x847 };
		for (size_t i = 0; i < sizeof(patch) / sizeof(patch[0]); i++)
			s_sub_rom[patch[i]] = 0;
		// MAME: screen.set_raw(12 MHz / 2, 384, 0, 256, 264, 16, 240)
		s_lines = 264;
		s_rate_mhz = (uint32_t)(6000000ull * 1000 / (384 * 264));
	} else {
		if (setup_mcu(CCTL, "cctl", MCS48_I8041A, g) || setup_mcu(CCPU, "ccpu", MCS48_I8041A, g) ||
		    setup_mcu(UCPU, "ucpu", MCS48_I8041A, g) ||
		    setup_mcu(CSND, "csnd", s_kind == M_GREATGUR ? MCS48_I8041A : MCS48_I8742, g))
			return -1;
		s_lines = 256;   // set_refresh_hz(60), 256 lines
		s_rate_mhz = 60000;
	}
	if (decode_gfx()) {
		rbx_fail("%s: graphics ROMs missing or out of memory", g->set);
		gl_exit();
		return -1;
	}
	msm_init_tables();
	s_p_dsw1 = rbx_port_find("DSW1");
	s_p_dsw2 = rbx_port_find("DSW2");
	s_p_dsw3 = rbx_port_find("DSW3");
	s_p_in0 = rbx_port_find("IN0");
	s_p_in1 = rbx_port_find("IN1");
	s_p_in2 = rbx_port_find("IN2");
	s_p_coins = rbx_port_find("COINS");
	s_p_p1 = rbx_port_find("P1");
	s_p_p2 = rbx_port_find("P2");
	s_p_system = rbx_port_find("SYSTEM");

	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = 1500000;
	yc.irq = ym_irq;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = 154;   // 0.60
	yc.gain[3] = 128;                             // 0.50
	yc.port_b_read = ym_port_b_r;
	if (s_kind == M_PPKING)
		yc.port_a_read = ym_port_a_r;
	else
		yc.port_a_write = ym_port_a_w;
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: sound init failed", g->set);
		gl_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 2048)) {
		gl_exit();
		return -1;
	}
	gl_reset();
	return 0;
}

static void gl_frame(void)
{
	rbx_audio.frame_mhz = s_rate_mhz;   // (the board's frame rate: a frame's sound is played in a 60 Hz frame, as its picture)
	for (int line = 0; line < s_lines; line++) {
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			s_main.irq = 1;   // irq0_line_hold at vblank
			if (s_kind == M_PPKING)
				s_sub.irq_level = 1;   // periodic irq0_line_assert, 60 Hz
		}
		if (s_kind != M_PPKING)
			poll_buttons();
		int nm = rbx_line_cycles(&s_st.frac_main, MAIN_CLOCK, s_rate_mhz, s_lines);
		int ns = rbx_line_cycles(&s_st.frac_sub, SUB_CLOCK, s_rate_mhz, s_lines);
		int na = rbx_line_cycles(&s_st.frac_adpcm, ADPCM_CLOCK, s_rate_mhz, s_lines);
		int nu = rbx_line_cycles(&s_st.frac_mcu, MCU_CLOCK, s_rate_mhz, s_lines);
		for (int sl = 0; sl < SLICES; sl++) {
			int part = (sl + 1 == SLICES);
			zcpu_run(&s_main, part ? nm - nm / SLICES * (SLICES - 1) : nm / SLICES);
			zcpu_run(&s_sub, part ? ns - ns / SLICES * (SLICES - 1) : ns / SLICES);
			s_st.adpcm_target += part ? na - na / SLICES * (SLICES - 1) : na / SLICES;
			m6809_run_until(&s_adpcm, s_st.adpcm_target);
			mcus_run(part ? nu - nu / SLICES * (SLICES - 1) : nu / SLICES);
		}
		ym_sync_sub();
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	s_ym_done = 0;
	s_sub_frame_start = s_sub.z.cyc;
	s_adpcm_frame_start = s_adpcm.cycles;
}

static int zcpu_state(void *ctx, const char *tag, zcpu *c, int load)
{
	zcpu t = *c;
	if (rbx_state_io(ctx, tag, &t, sizeof(t)))
		return -1;
	if (load) {
		t.z.read_byte = c->z.read_byte;
		t.z.write_byte = c->z.write_byte;
		t.z.port_in = c->z.port_in;
		t.z.port_out = c->z.port_out;
		t.z.userdata = c->z.userdata;
		*c = t;
	}
	return 0;
}

static int gl_state(void *ctx, int load)
{
	int rc = 0;
	rc |= zcpu_state(ctx, "main", &s_main, load);
	rc |= zcpu_state(ctx, "sub", &s_sub, load);
	m6809_t m = s_adpcm;
	rc |= rbx_state_io(ctx, "m6809", &m, M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "misc", &s_st, sizeof(s_st));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "palx", s_pal_ext, sizeof(s_pal_ext));
	rc |= rbx_state_io(ctx, "vram", s_videoram, sizeof(s_videoram));
	rc |= rbx_state_io(ctx, "cram", s_colorram, sizeof(s_colorram));
	rc |= rbx_state_io(ctx, "tram", s_textram, sizeof(s_textram));
	rc |= rbx_state_io(ctx, "nvram", s_nvram, sizeof(s_nvram));
	rc |= rbx_state_io(ctx, "subram", s_sub_ram, sizeof(s_sub_ram));
	if (s_kind != M_PPKING) {
		static const char *tags[4] = { "cctl", "ccpu", "ucpu", "csnd" };
		for (int i = 0; i < 4; i++)
			rc |= rbx_state_io(ctx, tags[i], (uint8_t *)&s_mcu[i] + MCS48_STATE_OFFSET, (uint32_t)MCS48_STATE_SIZE);
	}
	rc |= rbx_ym_state(0, ctx, load);
	if (rc)
		return -1;
	if (load) {
		memcpy(&s_adpcm, &m, M6809_STATE_SIZE);
		s_sub_frame_start = s_sub.z.cyc;
		s_adpcm_frame_start = s_adpcm.cycles;
		s_ym_done = 0;
		s_mix_n = 0;
	}
	return 0;
}

static int gl_save(void *ctx) { return gl_state(ctx, 0); }
static int gl_load(void *ctx) { return gl_state(ctx, 1); }
static int gl_supported(const char *machine)
{
	for (size_t i = 0; i < sizeof(s_machines) / sizeof(s_machines[0]); i++)
		if (!strcmp(s_machines[i].name, machine))
			return 1;
	return 0;
}

const rbx_driver rbx_drv_gladiatr = { "Gladiator", gl_init, gl_exit, gl_reset, gl_frame, gl_save, gl_load, gl_supported, gl_render };
