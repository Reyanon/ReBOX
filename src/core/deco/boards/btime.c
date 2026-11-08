/*  Data East Burger Time hardware and its relatives: Burger Time, Cook Race, Lock'n'Chase, Pro Tennis, World Tennis,
    Minky Monkey, Burnin' Rubber / Bump 'n' Jump, Zoar, Disco No.1, Super Doubles Tennis, Scrambled Egg / Eggs, Dommy.
    Ported from MAME src/mame/dataeast/btime.cpp (BSD-3-Clause, copyright-holders: Zsolt Vasvari, Couriersud).
    The main CPU: a DECO CPU-7 (an opcode after a write at an address with bits 8 and 2 set is bit-swapped), a DECO
    C10707 (opcode bits 5 and 6 swapped) or a plain 6502; no vblank interrupt (the game polls it), the coins raise an
    IRQ or an NMI. The sound: a 6502 at 500 kHz (its IRQ the latch's pending data, its NMI a 16-line clock gated by an
    enable), two AY-3-8910, and Burger Time's discrete stage (channel 2A through an op-amp band-pass, the mix through
    an op-amp and two CR filters). Eggs / Dommy: the 6502 alone, its IRQ from a scanline timer, the AYs on its bus.
    Not supported (MAME: not working): tisland.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../sound/ay8910.h"
#include "../../video/gfxlayout.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 57444          // set_raw(12 MHz / 2, 384, 8, 248, 272, 8, 248)
#define LINES 272
#define VIS_Y0 8
#define VIS_H 240

enum { K_BTIME, K_COOKRACE, K_LNC, K_WTENNIS, K_MMONKEY, K_BNJ, K_SDTENNIS, K_ZOAR, K_DISCO, K_PROTENN, K_SCREGG,
       K_DOMMY };
enum { CPU_DECO7, CPU_C10707, CPU_6502 };
enum { NMI_NONE, NMI_DIRECT, NMI_AY };       // m_audio_nmi_enable_type
enum { COIN_IRQ_HI, COIN_IRQ_LO, COIN_NMI_LO, COIN_NONE };

static int s_k, s_nmi_type, s_coin_kind, s_vis_x0, s_vis_w;
static uint32_t s_cpu_clock;
static uint8_t *s_rom, *s_srom;
static uint32_t s_rom_size, s_srom_size;
static uint8_t s_ram[0x4000];               // main RAM (lnc: 0x0000-0x3bff; disco: the low 0x500)
static uint8_t s_ram_b[0x400];              // lnc 0xb000-0xb1ff, cookrace 0xd000-0xd3ff, bnj 0x5200-0x53ff
static uint8_t s_vram[0x400], s_cram[0x400], s_bgram[0x400];
static uint8_t s_charram[0x6000];           // disco
static uint8_t s_spriteram[0x20];           // disco
static uint8_t s_palram[16];
static uint8_t s_scroll[2], s_zoar_scroll[4], s_palbank, s_flip, s_charbank;
static uint8_t s_prot_cmd, s_prot_status, s_prot_value, s_prot_ret;   // (mmonkey)
static uint8_t s_mm_table[0x1000];          // mmonkey: the protection's tables in the ROM space at 0xb000
static const uint8_t *s_proms, *s_bgmap;
static uint32_t s_proms_size, s_bgmap_size;
static int s_port_p1, s_port_p2, s_port_sys, s_port_dsw1, s_port_dsw2, s_port_vbl;
static int s_line;
static uint32_t s_frame;
static uint8_t s_coin_last;
static int64_t s_cpu_target, s_snd_target;
static uint64_t s_cpu_frac, s_snd_frac, s_ay_frac;
static m6502_t s_cpu, s_snd;
static uint32_t s_rgb[64], s_rgb_prom[64];   // (the live palette; the PROM's)
static rbx_gfx s_gfxchar, s_gfxspr, s_gfxbg;
static int s_irq6502;                       // scregg: the scanline IRQ (level)

typedef struct {
	uint8_t vram[0x400], cram[0x400], bgram[0x400], charram[0x6000], spriteram[0x20], palram[16];
	uint8_t scroll[2], zoar_scroll[4], palbank, flip;
	uint32_t rgb[64];
	uint32_t frame;
} snap_t;
static snap_t *s_snap;

// ---- sound ----
static ay8910_t s_ay[2];
static uint8_t s_sram[0x400];
static uint8_t s_latch;
static int s_latch_pending, s_nmi_enable, s_nmi_clock;
// the discrete stage (btime_sound_discrete) at the AYs' rate / 4
typedef struct {
	double bp_x1, bp_x2, bp_y1, bp_y2;      // DST_OP_AMP_FILT band pass (m_fc)
	double mix_cf;                          // DST_MIXER's cF
	double cr1, cr2;                        // DST_CRFILTER caps
	int32_t acc[6], acc_n;
} disc_t;
static disc_t s_disc;
static double s_bp_a1, s_bp_a2, s_bp_b0, s_bp_b1, s_bp_b2, s_exp_cf, s_exp_cr1, s_exp_cr2, s_disc_rate;
static int16_t *s_chbuf[2];
static int32_t *s_mix;
static int s_mix_cap, s_ch_cap;
static int s_discrete;                      // btime's network (else the AYs summed)
static uint32_t s_ay_clock;

static void update_snd_nmi(void)   // INPUT_MERGER_ALL_HIGH (the enable, the 8 vertical clock)
{
	m6502_set_nmi(&s_snd, s_nmi_enable && s_nmi_clock);
}
static void ay_port_a(void *u, uint8_t data)   // ay_audio_nmi_enable_w: port A bit 0, when 1, inhibits the NMI
{
	(void)u;
	if (s_nmi_type == NMI_AY) {
		s_nmi_enable = !(data & 1);
		update_snd_nmi();
	}
}

static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (s_k == K_DISCO || s_k == K_PROTENN) {   // disco_audio_map
		if (a < 0x0400) return s_sram[a];
		if (a >= 0x8000 && a < 0x9000) return s_latch;
		if (a >= 0xf000) return a < s_srom_size ? s_srom[a] : 0;
		return 0;
	}
	if ((s_k == K_COOKRACE || s_k == K_WTENNIS) && a >= 0x0200 && a < 0x1000)   // (install_rom from 0xe200)
		return s_srom[0xe200 + (a - 0x0200)];
	if (a < 0x2000) return s_sram[a & 0x3ff];
	if (a >= 0xa000 && a < 0xc000) {   // (the IRQ: the latch's pending data until read)
		s_latch_pending = 0;
		m6502_set_irq(&s_snd, 0);
		return s_latch;
	}
	if (a >= 0xe000) return s_srom[0xe000 + (a & 0x0fff)];
	return 0;
}
static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (s_k == K_DISCO || s_k == K_PROTENN) {
		if (a < 0x0400) s_sram[a] = v;
		else if (a >= 0x4000 && a < 0x5000) ay8910_data_w(&s_ay[0], v);
		else if (a >= 0x5000 && a < 0x6000) ay8910_address_w(&s_ay[0], v);
		else if (a >= 0x6000 && a < 0x7000) ay8910_data_w(&s_ay[1], v);
		else if (a >= 0x7000 && a < 0x8000) ay8910_address_w(&s_ay[1], v);
		else if (a >= 0x8000 && a < 0x9000) {   // acknowledge_w (separate acknowledge)
			s_latch_pending = 0;
			m6502_set_irq(&s_snd, 0);
		}
		return;
	}
	if (a < 0x2000) s_sram[a & 0x3ff] = v;
	else if (a < 0x4000) ay8910_data_w(&s_ay[0], v);
	else if (a < 0x6000) ay8910_address_w(&s_ay[0], v);
	else if (a < 0x8000) ay8910_data_w(&s_ay[1], v);
	else if (a < 0xa000) ay8910_address_w(&s_ay[1], v);
	else if (a >= 0xc000 && a < 0xe000 && s_nmi_type == NMI_DIRECT) {   // audio_nmi_enable_w
		s_nmi_enable = v & 1;
		update_snd_nmi();
	}
}

static void latch_w(uint8_t v)
{
	s_latch = v;
	s_latch_pending = 1;
	m6502_set_irq(&s_snd, 1);
}

// DISCRETE_OP_AMP_FILTER band pass 1M (btime_opamp_desc), DISCRETE_MIXER2 (btime_sound_mixer_desc), the CR filters
static void disc_setup(double rate)
{
	const double r1 = 5000, r3 = 10000, rf = 47000, c1 = 0.068e-6, c2 = 0.068e-6;
	const double rtot = 1.0 / (1.0 / r1 + 1.0 / r3);
	const double fc = 1.0 / (2 * M_PI * sqrt(rtot * rf * c1 * c2));
	const double d = (c1 + c2) / sqrt(rf / rtot * c1 * c2);
	const double gain = -rf / rtot * c2 / (c1 + c2);
	const double two_over_t = 2 * rate, tt2 = two_over_t * two_over_t;
	const double wc = rate * 2.0 * tan(M_PI * fc / rate), wc2 = wc * wc;   // (calculate_filter2_coefficients)
	const double den = tt2 + d * wc * two_over_t + wc2;
	s_bp_a1 = 2.0 * (-tt2 + wc2) / den;
	s_bp_a2 = (tt2 - d * wc * two_over_t + wc2) / den;
	s_bp_b0 = d * wc * two_over_t / den * gain;
	s_bp_b1 = 0;
	s_bp_b2 = -(d * wc * two_over_t / den) * gain;
	const double t = 1.0 / rate;   // (RC_CHARGE_EXP)
	s_exp_cf = 1.0 - exp(-t / (10000 * 150e-12));
	s_exp_cr1 = 1.0 - exp(-t / (10000 * 10e-6));
	s_exp_cr2 = 1.0 - exp(-t / (3.0 * 100e-6));
	s_disc_rate = rate;
}

static int32_t disc_step(const double *v)   // v: the six channels in volts (1A, 1B, 1C, 2A, 2B, 2C)
{
	disc_t *d = &s_disc;
	const double n22 = (v[0] + v[1] + v[2] + v[4] + v[5]) * 0.2;
	// NODE_30: the op amp: Millman of the inputs (r1, r3 to vRef 0), then the band pass, clipped to the rails
	const double vin = v[3] / 5000.0 * (1.0 / (1.0 / 5000 + 1.0 / 10000));
	double n30 = -s_bp_a1 * d->bp_y1 - s_bp_a2 * d->bp_y2 + s_bp_b0 * vin + s_bp_b1 * d->bp_x1 + s_bp_b2 * d->bp_x2;
	d->bp_x2 = d->bp_x1;
	d->bp_x1 = vin;
	d->bp_y2 = d->bp_y1;
	if (n30 > 5.0 - 1.5) n30 = 5.0 - 1.5;   // (vP - OP_AMP_VP_RAIL_OFFSET)
	if (n30 < -5.0) n30 = -5.0;
	d->bp_y1 = n30;
	// NODE_40: the op-amp mixer (rF 10k, cF 150p)
	const double i = (0 - n22) / 100000.0 + (0 - n30) / 100000.0;
	d->mix_cf += (i * 10000.0 - d->mix_cf) * s_exp_cf;
	const double n40 = d->mix_cf;
	// NODE_41, NODE_43: CR filters
	const double n41 = n40 - d->cr1;
	d->cr1 += n41 * s_exp_cr1;
	const double n43 = n41 - d->cr2;
	d->cr2 += n43 * s_exp_cr2;
	double out = n43 * 32767.0 / 5.0 * 35.0;
	if (out > 32767) out = 32767;
	if (out < -32768) out = -32768;
	return (int32_t)out;
}

static void sound_line(void)
{
	s_snd_target += rbx_line_cycles(&s_snd_frac, 500000, FRAME_MHZ, LINES);
	if (s_k != K_SCREGG && s_k != K_DOMMY)
		m6502_run_until(&s_snd, s_snd_target);
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&s_ay_frac, s_ay_clock, FRAME_MHZ, LINES);
	if (s_discrete) {   // the channels at the native rate, 4 of them averaged into a discrete step
		const int n0 = ay8910_render_ch(&s_ay[0], clocks, s_chbuf[0], s_ch_cap);
		const int n1 = ay8910_render_ch(&s_ay[1], clocks, s_chbuf[1], s_ch_cap);
		const int n = n0 < n1 ? n0 : n1;
		for (int k = 0; k < n; k++) {
			for (int c = 0; c < 3; c++) {
				s_disc.acc[c] += s_chbuf[0][3 * k + c];
				s_disc.acc[3 + c] += s_chbuf[1][3 * k + c];
			}
			if (++s_disc.acc_n == 4) {
				double v[6];
				for (int c = 0; c < 6; c++) {
					v[c] = s_disc.acc[c] / 4.0 * 5.0 / 32767.0;   // (DISCRETE_INPUTX_STREAM 5.0/32767.0)
					s_disc.acc[c] = 0;
				}
				s_disc.acc_n = 0;
				rbx_audio_put(disc_step(v));
			}
		}
		return;
	}
	const int n0 = ay8910_render(&s_ay[0], clocks, s_mix, s_mix_cap);
	const int n1 = ay8910_render(&s_ay[1], clocks, s_mix, s_mix_cap);
	const int n = n0 > n1 ? n0 : n1;
	for (int k = 0; k < n; k++) {
		rbx_audio_put(s_mix[k]);
		s_mix[k] = 0;
	}
}

// ---- main bus ----
static int vblank(void) { return s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H; }
static uint8_t dsw1_r(void)
{
	if (s_k != K_DISCO && s_k != K_PROTENN)   // (bit 7: the vblank)
		rbx_port_custom(s_port_dsw1, 0x80, vblank() ? 0x80 : 0);
	return (uint8_t)rbx_port_read(s_port_dsw1);
}
static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }

// btime_mirror*: x and y swapped
static uint16_t mirror(uint16_t o) { return (uint16_t)(32 * (o % 32) + o / 32); }

static uint32_t pal3(int v) { return (uint32_t)((v << 5) | (v << 2) | (v >> 1)); }
static void pal_w(int i, uint8_t v)   // palette_device::write8, BGR_233_inverted
{
	const uint8_t d = (uint8_t)~v;
	s_palram[i] = v;
	s_rgb[i] = pal3(d & 7) << 16 | pal3((d >> 3) & 7) << 8 | (uint32_t)(((d >> 6) & 3) * 0x55);
}

static void video_control_w(uint8_t v) { s_flip = v & 1; }                  // btime_video_control_w
static void bnj_video_control_w(uint8_t v)
{
	if (port(s_port_dsw1) & 0x40)   // (cocktail)
		video_control_w(v);
}

static uint8_t mm_prot_r(uint16_t off)   // mmonkey_state::protection_r
{
	if (off == 0x0000) return s_prot_status;
	if (off == 0x0e00) return s_prot_ret;
	if (off >= 0x0d00 && off <= 0x0d02) return s_mm_table[off];
	return 0;
}
static void mm_prot_w(uint16_t off, uint8_t v)
{
	if (off == 0) {
		if (v == 0) {
			if (s_prot_cmd == 0) {   // score addition
				const uint8_t *t = &s_mm_table[0x0d00];
				int s1 = (t[0] & 15) + 10 * (t[0] >> 4) + 100 * (t[1] & 15) + 1000 * (t[1] >> 4) + 10000 * (t[2] & 15) +
				         100000 * (t[2] >> 4);
				int s2 = (t[3] & 15) + 10 * (t[3] >> 4) + 100 * (t[4] & 15) + 1000 * (t[4] >> 4) + 10000 * (t[5] & 15) +
				         100000 * (t[5] >> 4);
				int r = s1 + s2;
				uint8_t *w = &s_mm_table[0x0d00];
				w[0] = (uint8_t)(r % 10); r /= 10;
				w[0] |= (uint8_t)((r % 10) << 4); r /= 10;
				w[1] = (uint8_t)(r % 10); r /= 10;
				w[1] |= (uint8_t)((r % 10) << 4); r /= 10;
				w[2] = (uint8_t)(r % 10); r /= 10;
				w[2] |= (uint8_t)((r % 10) << 4);
			} else if (s_prot_cmd == 1) {   // decryption: the value's index in the table
				for (int i = 0; i < 0x100; i++)
					if (s_mm_table[0x0f00 + i] == s_prot_value) {
						s_prot_ret = (uint8_t)i;
						break;
					}
			}
			s_prot_status = 0;
		}
	} else if (off == 0x0c00)
		s_prot_cmd = v;
	else if (off == 0x0e00)
		s_prot_value = v;
	else if (off >= 0x0f00 || (off >= 0x0d00 && off <= 0x0d05))
		s_mm_table[off] = v;
}

// the program ROM's places in each map
static int rom_mapped(uint16_t a)
{
	switch (s_k) {
	case K_BTIME: return a >= 0xb000;
	case K_COOKRACE: return (a >= 0x0500 && a < 0x4000) || a >= 0xfff9;
	case K_LNC: case K_WTENNIS: case K_MMONKEY: return a >= 0xc000;
	case K_BNJ: case K_SDTENNIS: case K_DISCO: case K_PROTENN: case K_DOMMY: return a >= 0xa000;
	case K_ZOAR: return a >= 0xd000;
	case K_SCREGG: return (a >= 0x3000 && a < 0x8000) || a >= 0xf000;
	}
	return 0;
}

static uint8_t main_r(void *ctx, uint16_t a)
{
	(void)ctx;
	switch (s_k) {
	case K_BTIME:   // btime_map
		if (a < 0x0800) return s_ram[a];
		if (a >= 0x0c00 && a < 0x0c10) return s_palram[a & 15];
		if (a >= 0x1000 && a < 0x1400) return s_vram[a & 0x3ff];
		if (a >= 0x1400 && a < 0x1800) return s_cram[a & 0x3ff];
		if (a >= 0x1800 && a < 0x1c00) return s_vram[mirror(a & 0x3ff)];
		if (a >= 0x1c00 && a < 0x2000) return s_cram[mirror(a & 0x3ff)];
		switch (a) {
		case 0x4000: return port(s_port_p1);
		case 0x4001: return port(s_port_p2);
		case 0x4002: return port(s_port_sys);
		case 0x4003: return dsw1_r();
		case 0x4004: return port(s_port_dsw2);
		}
		break;
	case K_COOKRACE:
		if (a < 0x0400) return s_ram[a];
		if (a >= 0xc000 && a < 0xc400) return s_vram[a & 0x3ff];
		if (a >= 0xc400 && a < 0xc800) return s_cram[a & 0x3ff];
		if (a >= 0xc800 && a < 0xcc00) return s_vram[mirror(a & 0x3ff)];
		if (a >= 0xcc00 && a < 0xd000) return s_cram[mirror(a & 0x3ff)];
		if (a >= 0xd000 && a < 0xd400) return s_ram_b[a & 0x3ff];
		if (a >= 0xd400 && a < 0xd800) return s_bgram[a & 0x3ff];
		switch (a) {
		case 0xe000: case 0xe300: return dsw1_r();
		case 0xe001: return port(s_port_dsw2);
		case 0xe002: return port(s_port_p1);
		case 0xe003: return port(s_port_p2);
		case 0xe004: return port(s_port_sys);
		}
		break;
	case K_LNC: case K_WTENNIS: case K_MMONKEY:   // lnc_map / mmonkey_state::program_map
		if (a < 0x3c00) return s_ram[a];
		if (a >= 0x3c00 && a < 0x4000) return s_vram[a & 0x3ff];
		if (a >= 0x7c00 && a < 0x8000) return s_vram[mirror(a & 0x3ff)];
		if (s_k == K_MMONKEY && a >= 0xb000 && a < 0xc000) return mm_prot_r((uint16_t)(a - 0xb000));
		if (s_k != K_MMONKEY && a >= 0xb000 && a < 0xb200) return s_ram_b[a & 0x1ff];
		if (s_k == K_WTENNIS && a == 0xc15f) {   // wtennis_reset_hack_r
			s_rom[0xfc30] = 0;
			return s_rom[0xc15f];
		}
		switch (a) {
		case 0x8000: return dsw1_r();
		case 0x8001: return port(s_port_dsw2);
		case 0x9000: return port(s_port_p1);
		case 0x9001: return port(s_port_p2);
		case 0x9002: return port(s_port_sys);
		}
		break;
	case K_BNJ: case K_SDTENNIS:
		if (a < 0x0800) return s_ram[a];
		if (a >= 0x4000 && a < 0x4400) return s_vram[a & 0x3ff];
		if (a >= 0x4400 && a < 0x4800) return s_cram[a & 0x3ff];
		if (a >= 0x4800 && a < 0x4c00) return s_vram[mirror(a & 0x3ff)];
		if (a >= 0x4c00 && a < 0x5000) return s_cram[mirror(a & 0x3ff)];
		if (a >= 0x5000 && a < 0x5200) return s_bgram[a & 0x1ff];
		if (a >= 0x5200 && a < 0x5400) return s_ram_b[a & 0x1ff];
		if (a >= 0x5c00 && a < 0x5c10) return s_palram[a & 15];
		switch (a) {
		case 0x1000: return dsw1_r();
		case 0x1001: return port(s_port_dsw2);
		case 0x1002: return port(s_port_p1);
		case 0x1003: return port(s_port_p2);
		case 0x1004: return port(s_port_sys);
		}
		break;
	case K_ZOAR:
		if (a < 0x0800) return s_ram[a];
		switch (a) {
		case 0x9800:   // zoar_dsw1_read: !vblank in bit 7
			return (uint8_t)((!vblank() << 7) | (port(s_port_dsw1) & 0x7f));
		case 0x9801: return port(s_port_dsw2);
		case 0x9802: return port(s_port_p1);
		case 0x9803: return port(s_port_p2);
		case 0x9804: return port(s_port_sys);
		}
		break;
	case K_DISCO: case K_PROTENN:
		if (a < 0x0500) return s_ram[a];
		if (a >= 0x2000 && a < 0x8000) return s_charram[a - 0x2000];
		if (a >= 0x8000 && a < 0x8400) return s_vram[a & 0x3ff];
		if (a >= 0x8400 && a < 0x8800) return s_cram[a & 0x3ff];
		if (a >= 0x8800 && a < 0x8820) return s_spriteram[a & 0x1f];
		switch (a) {
		case 0x9000: return port(s_port_sys);
		case 0x9200: return port(s_port_p1);
		case 0x9400: return port(s_port_p2);
		case 0x9800: return port(s_port_dsw1);
		case 0x9a00: return s_k == K_DISCO ? port(s_port_dsw2) : 0;
		case 0x9a01: return s_k == K_PROTENN ? port(s_port_dsw2) : 0;
		case 0x9c00:
			rbx_port_custom(s_port_vbl, 0x80, vblank() ? 0x80 : 0);
			return port(s_port_vbl);
		}
		break;
	case K_SCREGG:   // eggs_map
		if (a < 0x0800) return s_ram[a];
		if (a >= 0x1000 && a < 0x1400) return s_vram[a & 0x3ff];
		if (a >= 0x1400 && a < 0x1800) return s_cram[a & 0x3ff];
		if (a >= 0x1800 && a < 0x1c00) return s_vram[mirror(a & 0x3ff)];
		if (a >= 0x1c00 && a < 0x2000) return s_cram[mirror(a & 0x3ff)];
		switch (a) {
		case 0x2000: return dsw1_r();
		case 0x2001: return port(s_port_dsw2);
		case 0x2002: return port(s_port_p1);
		case 0x2003: return port(s_port_p2);
		case 0x2004: case 0x2005: s_irq6502 = 0; m6502_set_irq(&s_cpu, 0); return 0;   // irqack_r
		}
		break;
	case K_DOMMY:
		if (a < 0x0800) return s_ram[a];
		if (a >= 0x2000 && a < 0x2400) return s_vram[a & 0x3ff];
		if (a >= 0x2400 && a < 0x2800) return s_cram[a & 0x3ff];
		if (a >= 0x2800 && a < 0x2c00) return s_vram[mirror(a & 0x3ff)];
		switch (a) {
		case 0x4000: return dsw1_r();
		case 0x4001: return port(s_port_dsw2);
		case 0x4002: return port(s_port_p1);
		case 0x4003: return port(s_port_p2);
		case 0x4004: case 0x4005: s_irq6502 = 0; m6502_set_irq(&s_cpu, 0); return 0;
		}
		break;
	}
	return rom_mapped(a) ? s_rom[a] : 0;   // (unmapped: 0)
}

static void main_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	switch (s_k) {
	case K_BTIME:
		if (a < 0x0800) s_ram[a] = v;
		else if (a >= 0x0c00 && a < 0x0c10) pal_w(a & 15, v);
		else if (a >= 0x1000 && a < 0x1400) s_vram[a & 0x3ff] = v;
		else if (a >= 0x1400 && a < 0x1800) s_cram[a & 0x3ff] = v;
		else if (a >= 0x1800 && a < 0x1c00) s_vram[mirror(a & 0x3ff)] = v;
		else if (a >= 0x1c00 && a < 0x2000) s_cram[mirror(a & 0x3ff)] = v;
		else if (a == 0x4002) video_control_w(v);
		else if (a == 0x4003) latch_w(v);
		else if (a == 0x4004) s_scroll[0] = v;
		return;
	case K_COOKRACE:
		if (a < 0x0400) s_ram[a] = v;
		else if (a >= 0xc000 && a < 0xc400) s_vram[a & 0x3ff] = v;
		else if (a >= 0xc400 && a < 0xc800) s_cram[a & 0x3ff] = v;
		else if (a >= 0xc800 && a < 0xcc00) s_vram[mirror(a & 0x3ff)] = v;
		else if (a >= 0xcc00 && a < 0xd000) s_cram[mirror(a & 0x3ff)] = v;
		else if (a >= 0xd000 && a < 0xd400) s_ram_b[a & 0x3ff] = v;
		else if (a >= 0xd400 && a < 0xd800) s_bgram[a & 0x3ff] = v;
		else if (a == 0xe000) bnj_video_control_w(v);
		else if (a == 0xe001) latch_w(v);
		return;
	case K_LNC: case K_WTENNIS: case K_MMONKEY:
		if (a < 0x3c00) s_ram[a] = v;
		else if (a < 0x4000) {   // lnc_videoram_w: the colour RAM gets the char bank
			s_vram[a & 0x3ff] = v;
			s_cram[a & 0x3ff] = s_charbank;
		} else if (a >= 0x7800 && a < 0x7c00) s_cram[a & 0x3ff] = v;
		else if (a >= 0x7c00 && a < 0x8000) {
			const uint16_t o = mirror(a & 0x3ff);
			s_vram[o] = v;
			s_cram[o] = s_charbank;
		} else if (a == 0x8001) bnj_video_control_w(v);
		else if (a == 0x8003) s_charbank = v;
		else if (a == 0x9002) latch_w(v);
		else if (s_k == K_MMONKEY && a >= 0xb000 && a < 0xc000) mm_prot_w((uint16_t)(a - 0xb000), v);
		else if (s_k != K_MMONKEY && a >= 0xb000 && a < 0xb200) s_ram_b[a & 0x1ff] = v;
		return;
	case K_BNJ: case K_SDTENNIS:
		if (a < 0x0800) s_ram[a] = v;
		else if (a == 0x1001) bnj_video_control_w(v);
		else if (a == 0x1002) latch_w(v);
		else if (a >= 0x4000 && a < 0x4400) s_vram[a & 0x3ff] = v;
		else if (a >= 0x4400 && a < 0x4800) s_cram[a & 0x3ff] = v;
		else if (a >= 0x4800 && a < 0x4c00) s_vram[mirror(a & 0x3ff)] = v;
		else if (a >= 0x4c00 && a < 0x5000) s_cram[mirror(a & 0x3ff)] = v;
		else if (a >= 0x5000 && a < 0x5200) s_bgram[a & 0x1ff] = v;
		else if (a >= 0x5200 && a < 0x5400) s_ram_b[a & 0x1ff] = v;
		else if (a == 0x5400) s_scroll[0] = v;
		else if (a == 0x5800) s_scroll[1] = v;
		else if (a >= 0x5c00 && a < 0x5c10) pal_w(a & 15, v);
		return;
	case K_ZOAR:
		if (a < 0x0800) s_ram[a] = v;
		else if (a >= 0x8000 && a < 0x8400) s_vram[a & 0x3ff] = v;
		else if (a >= 0x8400 && a < 0x8800) s_cram[a & 0x3ff] = v;
		else if (a >= 0x8800 && a < 0x8c00) s_vram[mirror(a & 0x3ff)] = v;
		else if (a >= 0x8c00 && a < 0x9000) s_cram[mirror(a & 0x3ff)] = v;
		else if (a == 0x9000) {   // zoar_video_control_w
			s_palbank = (uint8_t)((v & 0x30) >> 3);
			if (port(s_port_dsw1) & 0x40)
				s_flip = (v & 0x80) != 0;
		} else if (a >= 0x9800 && a < 0x9804) s_zoar_scroll[a & 3] = v;
		else if (a == 0x9804) s_scroll[1] = v;
		else if (a == 0x9805) s_scroll[0] = v;
		else if (a == 0x9806) latch_w(v);
		return;
	case K_DISCO: case K_PROTENN:
		if (a < 0x0500) s_ram[a] = v;
		else if (a >= 0x2000 && a < 0x8000) s_charram[a - 0x2000] = v;
		else if (a >= 0x8000 && a < 0x8400) s_vram[a & 0x3ff] = v;
		else if (a >= 0x8400 && a < 0x8800) s_cram[a & 0x3ff] = v;
		else if (a >= 0x8800 && a < 0x8820) s_spriteram[a & 0x1f] = v;
		else if (a == 0x9a00) latch_w(v);
		else if (a == 0x9c00) {   // disco_video_control_w
			s_palbank = (v >> 2) & 3;
			if (!(port(s_port_dsw1) & 0x40))
				s_flip = v & 1;
		}
		return;
	case K_SCREGG:
		if (a < 0x0800) s_ram[a] = v;
		else if (a >= 0x1000 && a < 0x1400) s_vram[a & 0x3ff] = v;
		else if (a >= 0x1400 && a < 0x1800) s_cram[a & 0x3ff] = v;
		else if (a >= 0x1800 && a < 0x1c00) s_vram[mirror(a & 0x3ff)] = v;
		else if (a >= 0x1c00 && a < 0x2000) s_cram[mirror(a & 0x3ff)] = v;
		else if (a == 0x2000) video_control_w(v);
		else if (a == 0x2001) { s_irq6502 = 0; m6502_set_irq(&s_cpu, 0); }   // irqack_w
		else if (a == 0x2004 || a == 0x2005) ay8910_write(&s_ay[0], a & 1, v);
		else if (a == 0x2006 || a == 0x2007) ay8910_write(&s_ay[1], a & 1, v);
		return;
	case K_DOMMY:
		if (a < 0x0800) s_ram[a] = v;
		else if (a >= 0x2000 && a < 0x2400) s_vram[a & 0x3ff] = v;
		else if (a >= 0x2400 && a < 0x2800) s_cram[a & 0x3ff] = v;
		else if (a >= 0x2800 && a < 0x2c00) s_vram[mirror(a & 0x3ff)] = v;
		else if (a == 0x4000) { s_irq6502 = 0; m6502_set_irq(&s_cpu, 0); }
		else if (a == 0x4001) video_control_w(v);
		else if (a == 0x4004 || a == 0x4005) ay8910_write(&s_ay[0], a & 1, v);
		else if (a == 0x4006 || a == 0x4007) ay8910_write(&s_ay[1], a & 1, v);
		return;
	}
}

// ---- video ----
static uint16_t s_bnjbg[256 * 512];          // bnj's background bitmap (512 x 256)
static uint8_t s_disco8[1024 * 64], s_disco16[256 * 256];   // disco's RAM graphics (0x6000 / 3 / 8, / 32)

// a gfx element drawn into the 256 x 256 screen (MAME transpen / opaque, clipped to the bitmap)
static void blit(const uint8_t *pix, int w, int h, int base, int fx, int fy, int sx, int sy, int opaque, uint16_t *dst,
                 int dw, int dh)
{
	for (int y = 0; y < h; y++) {
		const int dy = sy + y;
		if (dy < 0 || dy >= dh)
			continue;
		const uint8_t *row = pix + (fy ? h - 1 - y : y) * w;
		for (int x = 0; x < w; x++) {
			const int dx = sx + x;
			if (dx < 0 || dx >= dw)
				continue;
			const uint8_t p = row[fx ? w - 1 - x : x];
			if (opaque || p)
				dst[dy * dw + dx] = (uint16_t)(base + p);
		}
	}
}
static uint16_t s_scr[256 * 256];

static const uint8_t *elem(const rbx_gfx *g, uint32_t code) { return g->pix + (size_t)rbx_wrap(code, g->count) * g->width * g->height; }

static void draw_chars(const snap_t *s, int transparency, int color, int priority)
{
	for (int offs = 0; offs < 0x400; offs++) {
		int x = 31 - offs / 32, y = offs % 32;
		const int code = s->vram[offs] + 256 * (s->cram[offs] & 3);
		if (priority != -1 && priority != ((code >> 7) & 1))
			continue;
		if (s->flip) {
			x = 31 - x;
			y = 31 - y;
		}
		const uint8_t *pix = (s_k == K_DISCO || s_k == K_PROTENN) ? s_disco8 + (code & 0x3ff) * 64 : elem(&s_gfxchar, (uint32_t)code);
		blit(pix, 8, 8, color * 8, s->flip, s->flip, 8 * x, 8 * y, !transparency, s_scr, 256, 256);
	}
}

static void draw_sprites(const snap_t *s, int color, int yadj, int yadj_flip, const uint8_t *ram, int il)
{
	const int disco = s_k == K_DISCO || s_k == K_PROTENN;
	for (int i = 0, offs = 0; i < 8; i++, offs += 4 * il) {
		if (!(ram[offs] & 1))
			continue;
		int x = 240 - ram[offs + 3 * il], y = 240 - ram[offs + 2 * il];
		int fx = (ram[offs] & 4) != 0, fy = (ram[offs] & 2) != 0;
		if (s->flip) {
			x = 240 - x;
			y = 240 - y + yadj_flip;
			fx = !fx;
			fy = !fy;
		}
		y -= yadj;
		const uint32_t code = ram[offs + il];
		const uint8_t *pix = disco ? s_disco16 + (code & 0xff) * 256 : elem(&s_gfxspr, code);
		blit(pix, 16, 16, color * 8, fx, fy, x, y, 0, s_scr, 256, 256);
		y += s->flip ? -256 : 256;   // (wrap around)
		blit(pix, 16, 16, color * 8, fx, fy, x, y, 0, s_scr, 256, 256);
	}
}

static void draw_background(const snap_t *s, const uint8_t *tmap, int color)
{
	int scroll = -(s->scroll[1] | ((s->scroll[0] & 3) << 8));
	for (int i = 0; i < 5; i++, scroll += 256) {   // (one extra for the wrap)
		const uint32_t tileoffset = (uint32_t)tmap[i & 3] * 0x100;
		if (scroll > 256)
			break;
		if (scroll < -256)
			continue;
		for (int offs = 0; offs < 0x100; offs++) {
			int x = 240 - (16 * (offs / 16) + scroll) - 1, y = 16 * (offs % 16);
			if (s->flip) {
				x = 240 - x;
				y = 240 - y;
			}
			const uint32_t t = tileoffset + (uint32_t)offs;
			blit(elem(&s_gfxbg, t < s_bgmap_size ? s_bgmap[t] : 0), 16, 16, 8 + color * 8, s->flip, s->flip, x, y, 1,
			     s_scr, 256, 256);
		}
	}
}

static void bt_render(void)
{
	const snap_t *s = s_snap;
	for (int i = 0; i < 64; i++)
		rbx_video.palette[i] = s->rgb[i];
	if (s_k == K_DISCO || s_k == K_PROTENN) {   // disco_tile8layout / disco_tile16layout from the RAM
		const uint8_t *r = s->charram;
		for (int c = 0; c < 1024; c++)
			for (int y = 0; y < 8; y++) {
				const int o = c * 8 + y;
				for (int x = 0; x < 8; x++) {
					const int b = 7 - x;
					s_disco8[c * 64 + y * 8 + x] = (uint8_t)(((r[0x4000 + o] >> b) & 1) << 2 | ((r[0x2000 + o] >> b) & 1) << 1 |
					                                         ((r[o] >> b) & 1));
				}
			}
		for (int c = 0; c < 256; c++)
			for (int y = 0; y < 16; y++)
				for (int x = 0; x < 16; x++) {
					const int o = c * 32 + (x < 8 ? 16 : 0) + y, b = 7 - (x & 7);
					s_disco16[c * 256 + y * 16 + x] = (uint8_t)(((r[0x4000 + o] >> b) & 1) << 2 | ((r[0x2000 + o] >> b) & 1) << 1 |
					                                            ((r[o] >> b) & 1));
				}
	}
	switch (s_k) {
	case K_BTIME:   // screen_update_btime
		if (s->scroll[0] & 0x10) {
			uint8_t tmap[4];
			int start = s->flip ? 0 : 1;
			for (int i = 0; i < 4; i++) {
				tmap[i] = (uint8_t)(start | (s->scroll[0] & 0x04));
				start = (start + 1) & 3;
			}
			draw_background(s, tmap, 0);
			draw_chars(s, 1, 0, -1);
		} else
			draw_chars(s, 0, 0, -1);
		draw_sprites(s, 0, 1, 0, s->vram, 0x20);
		break;
	case K_LNC: case K_MMONKEY:
		draw_chars(s, 0, 0, -1);
		draw_sprites(s, 0, 1, 2, s->vram, 0x20);
		break;
	case K_WTENNIS: case K_SCREGG: case K_DOMMY:   // screen_update_eggs
		draw_chars(s, 0, 0, -1);
		draw_sprites(s, 0, 0, 0, s->vram, 0x20);
		break;
	case K_ZOAR:
		if (s->scroll[0] & 0x04) {
			draw_background(s, s->zoar_scroll, s->palbank - 1);   // (gfx_zoar: background colour base 0)
			draw_chars(s, 1, s->palbank + 1, -1);
		} else
			draw_chars(s, 0, s->palbank + 1, -1);
		draw_sprites(s, s->palbank + 1, 1, 2, s->vram + 0x1f, 0x20);   // (the order: the priorities)
		draw_sprites(s, s->palbank + 1, 1, 2, s->vram, 0x20);
		break;
	case K_BNJ: case K_SDTENNIS:
		if (s->scroll[0]) {
			for (int offs = 0x1ff; offs >= 0; offs--) {
				int sx = 16 * ((offs < 0x100) ? ((offs % 0x80) / 8) : ((offs % 0x80) / 8) + 16);
				int sy = 16 * (((offs % 0x100) < 0x80) ? offs % 8 : (offs % 8) + 8);
				sx = 496 - sx;
				if (s->flip) {
					sx = 496 - sx;
					sy = 240 - sy;
				}
				blit(elem(&s_gfxbg, (uint32_t)((s->bgram[offs] >> 4) + ((offs & 0x80) >> 3) + 32)), 16, 16, 8, s->flip, s->flip,
				     sx, sy, 1, s_bnjbg, 512, 256);
			}
			int scroll = (s->scroll[0] & 0x02) * 128 + 511 - s->scroll[1];
			if (!s->flip)
				scroll = 767 - scroll;
			for (int y = 0; y < 256; y++)   // copyscrollbitmap
				for (int x = 0; x < 256; x++)
					s_scr[y * 256 + x] = s_bnjbg[y * 512 + ((x - scroll) & 511)];
			draw_chars(s, 1, 0, 1);
			draw_sprites(s, 0, 0, 0, s->vram, 0x20);
			draw_chars(s, 1, 0, 0);
		} else {
			draw_chars(s, 0, 0, -1);
			draw_sprites(s, 0, 0, 0, s->vram, 0x20);
		}
		break;
	case K_COOKRACE:
		for (int offs = 0x3ff; offs >= 0; offs--) {
			int sx = 31 - offs / 32, sy = offs % 32;
			if (s->flip) {
				sx = 31 - sx;
				sy = 31 - sy;
			}
			blit(elem(&s_gfxbg, s->bgram[offs]), 8, 8, 8, s->flip, s->flip, 8 * sx, 8 * sy, 1, s_scr, 256, 256);
		}
		draw_chars(s, 1, 0, -1);
		draw_sprites(s, 0, 1, 0, s->vram, 0x20);
		break;
	case K_DISCO: case K_PROTENN:
		draw_chars(s, 0, s->palbank, -1);
		draw_sprites(s, s->palbank, 0, 0, s->spriteram, 1);
		break;
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = 0; y < VIS_H; y++)
		memcpy(bm + y * pitch, s_scr + (VIS_Y0 + y) * 256 + s_vis_x0, (size_t)s_vis_w * 2);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	snap_t *s = s_snap;
	memcpy(s->vram, s_vram, sizeof(s_vram));
	memcpy(s->cram, s_cram, sizeof(s_cram));
	memcpy(s->bgram, s_bgram, sizeof(s_bgram));
	if (s_k == K_DISCO || s_k == K_PROTENN) {
		memcpy(s->charram, s_charram, sizeof(s_charram));
		memcpy(s->spriteram, s_spriteram, sizeof(s_spriteram));
	}
	memcpy(s->palram, s_palram, sizeof(s_palram));
	memcpy(s->rgb, s_rgb, sizeof(s_rgb));
	memcpy(s->scroll, s_scroll, sizeof(s_scroll));
	memcpy(s->zoar_scroll, s_zoar_scroll, sizeof(s_zoar_scroll));
	s->palbank = s_palbank;
	s->flip = s_flip;
	s->frame = s_frame;
	rbx_video_frame_done();
}

// ---- the coins: MAME's PORT_CHANGED_MEMBER on SYSTEM bits 6 and 7 ----
static void coins(void)
{
	const uint8_t now = (uint8_t)(rbx_port_read(s_port_sys) & 0xc0);
	for (int b = 6; b < 8; b++) {
		const int was = (s_coin_last >> b) & 1, is = (now >> b) & 1;
		if (was == is)
			continue;
		switch (s_coin_kind) {
		case COIN_IRQ_HI: if (is) m6502_set_irq(&s_cpu, 1); break;   // (HOLD_LINE: irq_hold)
		case COIN_IRQ_LO: if (!is) m6502_set_irq(&s_cpu, 1); break;
		case COIN_NMI_LO: m6502_set_nmi(&s_cpu, !is); break;
		}
	}
	s_coin_last = now;
}

// ---- init / frame ----
static void bt_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_ram_b, 0, sizeof(s_ram_b));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_cram, 0, sizeof(s_cram));
	memset(s_bgram, 0, sizeof(s_bgram));
	memset(s_charram, 0, sizeof(s_charram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_palram, 0, sizeof(s_palram));
	memcpy(s_rgb, s_rgb_prom, sizeof(s_rgb));
	memset(s_scroll, 0, sizeof(s_scroll));
	memset(s_zoar_scroll, 0, sizeof(s_zoar_scroll));
	s_palbank = s_flip = 0;
	s_charbank = 1;   // (m_lnc_charbank = 1)
	s_prot_cmd = s_prot_status = s_prot_value = s_prot_ret = 0;
	if (s_k == K_MMONKEY)   // (the protection's tables live in the ROM space)
		memcpy(s_mm_table, s_rom + 0xb000, sizeof(s_mm_table));
	memset(s_sram, 0, sizeof(s_sram));
	s_latch = 0;
	s_latch_pending = 0;
	s_nmi_enable = s_nmi_type == NMI_NONE;   // (by default the NMI is disabled, except without an enable)
	s_nmi_clock = 0;
	s_irq6502 = 0;
	memset(&s_disc, 0, sizeof(s_disc));
	ay8910_reset(&s_ay[0]);
	ay8910_reset(&s_ay[1]);
	m6502_reset(&s_cpu);
	m6502_set_irq(&s_cpu, 0);
	m6502_set_nmi(&s_cpu, 0);
	s_cpu.cpu7_written = 0;
	m6502_reset(&s_snd);
	m6502_set_irq(&s_snd, 0);
	m6502_set_nmi(&s_snd, 0);
	s_snd.cpu7_written = 0;
	s_coin_last = (uint8_t)(rbx_port_read(s_port_sys) & 0xc0);
	s_line = 0;
	s_frame = 0;
	s_cpu_target = s_snd_target = 0;
	s_cpu_frac = s_snd_frac = s_ay_frac = 0;
}

static void bt_exit(void);

static int decode(rbx_gfx *g, int w, int h, const uint8_t *rom, uint32_t size, int kind)
{
	// kind 0: gfx_8x8x3_planar, 1: tile16layout (RGN_FRAC(1,3)), 2: bnj_tile16layout (RGN_FRAC(1,2))
	static uint32_t xo[16], yo[16];
	uint32_t po[3];
	rbx_gfx_layout l;
	memset(&l, 0, sizeof(l));
	l.width = w;
	l.height = h;
	l.planes = 3;
	l.planeoffs = po;
	l.xoffs = xo;
	l.yoffs = yo;
	if (kind == 2) {
		const uint32_t half = size / 2 * 8;
		po[0] = half + 4;
		po[1] = 0;
		po[2] = 4;
		for (int i = 0; i < 4; i++) {
			xo[i] = (uint32_t)(3 * 16 * 8 + i);
			xo[4 + i] = (uint32_t)(2 * 16 * 8 + i);
			xo[8 + i] = (uint32_t)(1 * 16 * 8 + i);
			xo[12 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			yo[i] = (uint32_t)(i * 8);
		l.charincrement = 64 * 8;
		return rbx_gfx_decode(g, &l, rom, size / 2);
	}
	const uint32_t third = size / 3 * 8;
	po[0] = 2 * third;
	po[1] = third;
	po[2] = 0;
	if (kind == 0) {
		for (int i = 0; i < 8; i++) {
			xo[i] = (uint32_t)i;
			yo[i] = (uint32_t)(i * 8);
		}
		l.charincrement = 8 * 8;
	} else {
		for (int i = 0; i < 8; i++) {
			xo[i] = (uint32_t)(16 * 8 + i);
			xo[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			yo[i] = (uint32_t)(i * 8);
		l.charincrement = 32 * 8;
	}
	return rbx_gfx_decode(g, &l, rom, size / 3);
}

static void palette_from_proms(int n, int lnc)   // btime_palette / lnc_palette
{
	for (int i = 0; i < n && (uint32_t)i < s_proms_size; i++) {
		const uint8_t p = s_proms[i];
		int r, g, b;
		if (lnc) {
			r = 0x21 * ((p >> 7) & 1) + 0x47 * ((p >> 6) & 1) + 0x97 * ((p >> 5) & 1);
			g = 0x21 * ((p >> 4) & 1) + 0x47 * ((p >> 3) & 1) + 0x97 * ((p >> 2) & 1);
			b = 0x52 * ((p >> 1) & 1) + 0xad * (p & 1);
		} else {
			r = 0x21 * (p & 1) + 0x47 * ((p >> 1) & 1) + 0x97 * ((p >> 2) & 1);
			g = 0x21 * ((p >> 3) & 1) + 0x47 * ((p >> 4) & 1) + 0x97 * ((p >> 5) & 1);
			b = 0x52 * ((p >> 6) & 1) + 0xad * ((p >> 7) & 1);
		}
		s_rgb[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
}

static int bt_init(const rbx_game *g)
{
	const char *m = g->machine;
	static const struct { const char *machine; int k, cpu, nmi, coin; uint32_t clock; } cfg[] = {
		{ "btime", K_BTIME, CPU_DECO7, NMI_DIRECT, COIN_IRQ_HI, 1500000 },
		{ "cookrace", K_COOKRACE, CPU_C10707, NMI_DIRECT, COIN_NMI_LO, 1500000 },
		{ "lnc", K_LNC, CPU_C10707, NMI_AY, COIN_NMI_LO, 1500000 },
		{ "wtennis", K_WTENNIS, CPU_C10707, NMI_AY, COIN_NMI_LO, 1500000 },
		{ "mmonkey", K_MMONKEY, CPU_C10707, NMI_AY, COIN_NMI_LO, 1500000 },
		{ "bnj", K_BNJ, CPU_C10707, NMI_DIRECT, COIN_NMI_LO, 750000 },
		{ "sdtennis", K_SDTENNIS, CPU_C10707, NMI_DIRECT, COIN_NMI_LO, 750000 },
		{ "zoar", K_ZOAR, CPU_DECO7, NMI_AY, COIN_IRQ_LO, 1500000 },
		{ "disco", K_DISCO, CPU_DECO7, NMI_AY, COIN_IRQ_HI, 750000 },
		{ "protenn", K_PROTENN, CPU_DECO7, NMI_AY, COIN_IRQ_HI, 750000 },
		{ "scregg", K_SCREGG, CPU_6502, NMI_NONE, COIN_NONE, 1500000 },
		{ "dommy", K_DOMMY, CPU_6502, NMI_NONE, COIN_NONE, 1500000 },
	};
	int ci = -1;
	for (int i = 0; i < (int)(sizeof(cfg) / sizeof(cfg[0])); i++)
		if (!strcmp(cfg[i].machine, m))
			ci = i;
	if (ci < 0) {
		rbx_fail("%s: machine not supported", g->set);
		return -1;
	}
	s_k = cfg[ci].k;
	s_nmi_type = cfg[ci].nmi;
	s_coin_kind = cfg[ci].coin;
	s_cpu_clock = cfg[ci].clock;
	const int wide = s_k == K_BNJ || s_k == K_SDTENNIS || s_k == K_ZOAR;   // (set_visarea(0, 255, 8, 247))
	s_vis_x0 = wide ? 0 : 8;
	s_vis_w = wide ? 256 : 240;
	uint32_t g1s = 0, g2s = 0, g3s = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("audiocpu", &s_srom_size);
	uint8_t *gfx1 = rbx_region("gfx1", &g1s), *gfx2 = rbx_region("gfx2", &g2s), *gfx3 = rbx_region("gfx3", &g3s);
	s_proms = rbx_region("proms", &s_proms_size);
	s_bgmap = rbx_region("bg_map", &s_bgmap_size);
	const int has_snd = s_k != K_SCREGG && s_k != K_DOMMY, disco = s_k == K_DISCO || s_k == K_PROTENN;
	if (!s_rom || s_rom_size < 0x10000 || (has_snd && (!s_srom || s_srom_size < 0x10000)) || (!disco && !gfx1)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (!strcmp(g->set, "zoar"))   // init_zoar: an undocumented opcode NOPed
		memset(s_rom + 0xd50a, 0xea, 8);
	if (!strcmp(g->set, "rockduck"))   // init_rockduck: rd2 / rd1 bitswapped
		for (uint32_t x = 0x2000; x < 0x6000 && x < g1s; x++) {
			const uint8_t v = gfx1[x];   // bitswap<8>(v, 2, 0, 3, 6, 1, 4, 7, 5)
			gfx1[x] = (uint8_t)(((v >> 2) & 1) << 7 | (v & 1) << 6 | ((v >> 3) & 1) << 5 | ((v >> 6) & 1) << 4 |
			                    ((v >> 1) & 1) << 3 | ((v >> 4) & 1) << 2 | ((v >> 7) & 1) << 1 | ((v >> 5) & 1));
		}
	int rc = 0;
	if (!disco) {
		rc = decode(&s_gfxchar, 8, 8, gfx1, g1s, 0) ||
		     decode(&s_gfxspr, 16, 16, s_k == K_ZOAR && gfx3 ? gfx3 : gfx1, s_k == K_ZOAR && gfx3 ? g3s : g1s, 1);
		if (!rc && gfx2)
			rc = decode(&s_gfxbg, s_k == K_COOKRACE ? 8 : 16, s_k == K_COOKRACE ? 8 : 16, gfx2, g2s,
			            s_k == K_COOKRACE ? 0 : (s_k == K_BNJ || s_k == K_SDTENNIS) ? 2 : 1);
	}
	s_snap = (snap_t *)calloc(1, sizeof(*s_snap));
	if (rc || !s_snap) {
		rbx_fail("%s: out of memory", g->set);
		bt_exit();
		return -1;
	}
	// the palette's fixed part: from the PROM (btime_palette / lnc_palette) where there is one
	memset(s_rgb, 0, sizeof(s_rgb));
	if (s_proms) {
		const int n = s_k == K_ZOAR ? 64 : (disco ? 32 : (s_k == K_LNC || s_k == K_WTENNIS || s_k == K_MMONKEY ||
		                                                  s_k == K_SCREGG || s_k == K_DOMMY) ? 8 : 16);
		palette_from_proms(n, s_k == K_LNC || s_k == K_WTENNIS || s_k == K_MMONKEY);
	}
	memcpy(s_rgb_prom, s_rgb, sizeof(s_rgb));
	// sound
	memset(&s_ay, 0, sizeof(s_ay));
	s_discrete = 0;
	if (s_k == K_SCREGG || s_k == K_DOMMY) {   // (1.5 MHz, 0.23, MAME's default legacy output)
		s_ay_clock = 1500000;
		for (int i = 0; i < 2; i++)
			ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, s_ay_clock, AY8910_LEGACY_OUTPUT, 59, 4, 0);
	} else if (s_k == K_ZOAR) {   // (3 MHz, 0.23; the first with the discrete output's levels)
		s_ay_clock = 3000000;
		ay8910_init(&s_ay[0], AY8910_TYPE_AY8910, s_ay_clock, 0, 59, 8, 0);
		ay8910_set_res_loads(&s_ay[0], 5000, 5000, 5000);
		ay8910_init(&s_ay[1], AY8910_TYPE_AY8910, s_ay_clock, AY8910_LEGACY_OUTPUT, 59, 8, 0);
	} else {   // btime's discrete stage
		s_ay_clock = 1500000;
		s_discrete = 1;
		ay8910_init(&s_ay[0], AY8910_TYPE_AY8910, s_ay_clock, 0, 256, 1, 0);
		ay8910_set_res_loads(&s_ay[0], 5000, 5000, 5000);
		ay8910_init(&s_ay[1], AY8910_TYPE_AY8910, s_ay_clock, 0, 256, 1, 0);
		ay8910_set_res_loads(&s_ay[1], 1000, 5000, 5000);
	}
	s_ay[0].port_a_write = ay_port_a;
	const uint32_t rate = s_discrete ? s_ay_clock / 8 / 4 : ay8910_rate(&s_ay[0]);
	s_ch_cap = (int)(s_ay_clock / 8 / 50) + 64;
	s_mix_cap = (int)(rate / 50) + 64;
	s_chbuf[0] = (int16_t *)calloc((size_t)s_ch_cap * 3, sizeof(int16_t));
	s_chbuf[1] = (int16_t *)calloc((size_t)s_ch_cap * 3, sizeof(int16_t));
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_chbuf[0] || !s_chbuf[1] || !s_mix || rbx_audio_setup(rate) || rbx_video_setup(s_vis_w, VIS_H, 64)) {
		rbx_fail("%s: out of memory", g->set);
		bt_exit();
		return -1;
	}
	if (s_discrete)
		disc_setup((double)rate);
	// CPUs
	m6502_init(&s_cpu, M6502_NMOS);
	s_cpu.deco222 = cfg[ci].cpu == CPU_C10707;
	s_cpu.cpu7 = cfg[ci].cpu == CPU_DECO7;
	s_cpu.irq_hold = s_coin_kind == COIN_IRQ_HI || s_coin_kind == COIN_IRQ_LO;   // (the coin IRQ: HOLD_LINE)
	s_cpu.read = main_r;
	s_cpu.write = main_w;
	for (int p = 0; p < 0x100; p++)   // (the ROM's whole pages read directly; wtennis' hack at 0xc15f excepted)
		if (rom_mapped((uint16_t)(p << 8)) && rom_mapped((uint16_t)(p << 8 | 0xff)) && !(s_k == K_WTENNIS && p == 0xc1))
			s_cpu.rpage[p] = s_rom + p * 256;
	m6502_init(&s_snd, M6502_NMOS);
	s_snd.deco222 = s_k == K_SDTENNIS;   // (sdtennis: a C10707 for the sound too)
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	s_port_p1 = rbx_port_find("P1");
	s_port_p2 = rbx_port_find("P2");
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_dsw1 = rbx_port_find("DSW1");
	s_port_dsw2 = rbx_port_find("DSW2");
	s_port_vbl = disco ? rbx_port_find("VBLANK") : -1;
	bt_reset();
	return 0;
}

static void bt_exit(void)
{
	rbx_gfx_free(&s_gfxchar);
	rbx_gfx_free(&s_gfxspr);
	rbx_gfx_free(&s_gfxbg);
	free(s_snap);
	s_snap = NULL;
	free(s_chbuf[0]);
	free(s_chbuf[1]);
	free(s_mix);
	s_chbuf[0] = s_chbuf[1] = NULL;
	s_mix = NULL;
}

static void bt_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	if (s_coin_kind != COIN_NONE)
		coins();
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		if ((line & 7) == 0) {   // the 8-line timers: the sound NMI clock (8vck), scregg's IRQ
			s_nmi_clock = (line & 8) >> 3;
			update_snd_nmi();
			if (s_k == K_SCREGG || s_k == K_DOMMY) {   // scregg_interrupt: (param & 8) ? ASSERT : CLEAR
				s_irq6502 = (line & 8) != 0;
				m6502_set_irq(&s_cpu, s_irq6502);
			}
		}
		s_cpu_target += rbx_line_cycles(&s_cpu_frac, s_cpu_clock, FRAME_MHZ, LINES);
		m6502_run_until(&s_cpu, s_cpu_target);
		sound_line();
	}
	s_frame++;
}

static int bt_state(void *ctx, int load)
{
	uint8_t cpu[M6502_STATE_SIZE], snd[M6502_STATE_SIZE];
	if (!load) {
		memcpy(cpu, &s_cpu, M6502_STATE_SIZE);
		memcpy(snd, &s_snd, M6502_STATE_SIZE);
	}
	int64_t tg[2] = { s_cpu_target, s_snd_target };
	uint64_t fr[3] = { s_cpu_frac, s_snd_frac, s_ay_frac };
	int32_t misc[20] = { (int32_t)s_frame, s_palbank, s_flip, s_charbank, s_prot_cmd, s_prot_status, s_prot_value,
	                     s_prot_ret, s_latch, s_latch_pending, s_nmi_enable, s_nmi_clock, s_irq6502, s_coin_last,
	                     s_cpu.cpu7_written, s_snd.cpu7_written, 0, 0, 0, 0 };
	uint8_t ay[2][256];
	const uint32_t ays = ay8910_state_size();
	if (!load)
		for (int i = 0; i < 2; i++)
			memcpy(ay[i], &s_ay[i], ays);
	int rc = 0;
	rc |= rbx_state_io(ctx, "cpu", cpu, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "snd", snd, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "ramb", s_ram_b, sizeof(s_ram_b));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "cram", s_cram, sizeof(s_cram));
	rc |= rbx_state_io(ctx, "bgram", s_bgram, sizeof(s_bgram));
	rc |= rbx_state_io(ctx, "chram", s_charram, sizeof(s_charram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "pal", s_palram, sizeof(s_palram));
	rc |= rbx_state_io(ctx, "rgb", s_rgb, sizeof(s_rgb));
	rc |= rbx_state_io(ctx, "scr", s_scroll, sizeof(s_scroll));
	rc |= rbx_state_io(ctx, "zscr", s_zoar_scroll, sizeof(s_zoar_scroll));
	rc |= rbx_state_io(ctx, "mm", s_mm_table, sizeof(s_mm_table));
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_state_io(ctx, "ay", ay, sizeof(ay));
	rc |= rbx_state_io(ctx, "disc", &s_disc, sizeof(s_disc));
	rc |= rbx_state_io(ctx, "tg", tg, sizeof(tg));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (s_k == K_WTENNIS)
		rc |= rbx_state_io(ctx, "fc30", &s_rom[0xfc30], 1);
	if (!rc && load) {
		memcpy(&s_cpu, cpu, M6502_STATE_SIZE);
		memcpy(&s_snd, snd, M6502_STATE_SIZE);
		for (int i = 0; i < 2; i++)
			memcpy(&s_ay[i], ay[i], ays);
		s_cpu_target = tg[0];
		s_snd_target = tg[1];
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_ay_frac = fr[2];
		s_frame = (uint32_t)misc[0];
		s_palbank = (uint8_t)misc[1];
		s_flip = (uint8_t)misc[2];
		s_charbank = (uint8_t)misc[3];
		s_prot_cmd = (uint8_t)misc[4];
		s_prot_status = (uint8_t)misc[5];
		s_prot_value = (uint8_t)misc[6];
		s_prot_ret = (uint8_t)misc[7];
		s_latch = (uint8_t)misc[8];
		s_latch_pending = misc[9];
		s_nmi_enable = misc[10];
		s_nmi_clock = misc[11];
		s_irq6502 = misc[12];
		s_coin_last = (uint8_t)misc[13];
		s_cpu.cpu7_written = (uint8_t)misc[14];
		s_snd.cpu7_written = (uint8_t)misc[15];
	}
	return rc ? -1 : 0;
}

static int bt_save(void *ctx) { return bt_state(ctx, 0); }
static int bt_load(void *ctx) { return bt_state(ctx, 1); }
static int bt_supported(const char *machine)
{
	static const char *const ok[] = { "btime", "cookrace", "lnc", "wtennis", "mmonkey", "bnj", "sdtennis", "zoar", "disco",
	                                  "protenn", "scregg", "dommy", NULL };
	for (int i = 0; ok[i]; i++)
		if (!strcmp(machine, ok[i]))
			return 1;
	return 0;
}

const rbx_driver rbx_drv_btime = { "DECO Burger Time", bt_init, bt_exit, bt_reset, bt_frame, bt_save, bt_load,
                                   bt_supported, bt_render, NULL };
