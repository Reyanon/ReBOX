/*  Data East Mad Alien. Ported from MAME src/mame/dataeast/madalien.cpp, madalien_v.cpp (GPL-2.0+, copyright-holders:
    Norbert Kehrer) and madalien_a.cpp (BSD-3-Clause, copyright-holders: Derrick Renaud), with the discrete modules it
    uses from src/devices/sound/disc_dev.hxx, disc_flt.hxx, disc_mth.hxx, disc_wav.hxx (BSD-3-Clause, copyright-holders:
    K.Wilkins, Derrick Renaud).
    A 6502 at 1.32 MHz (the coin on its NMI, a shifter PROM read through two registers), RAM characters over two 16x16
    road edge tilemaps (four scroll modes over two map sections), the highlight and the headlight lighting pens up;
    an MC6845 (here only its timing: the vsync to the sound NMI), a 6502 at 500 kHz with an AY-3-8910 whose ports
    switch the filters and two 555 motor sounds of the discrete netlist (run at 48 kHz as MAME's).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../sound/ay8910.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define XTAL 10595000
#define FRAME_MHZ 54744          // set_raw(10.595 MHz / 2, 336, 0, 256, 288, 0, 256)
#define LINES 288
#define VBSTART 256
#define DISC_RATE 48000
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static uint8_t *s_rom, *s_srom, *s_map, *s_shift;
static uint32_t s_rom_size, s_srom_size, s_map_size, s_shift_size;
static uint8_t s_ram[0x400], s_ram2[0x400], s_vram[0x400], s_chr[0x1800], s_sram[0x400], s_sram2[4];
static uint8_t s_video_control, s_shift_hi, s_shift_lo, s_video_flags, s_headlight_pos, s_edge1_pos, s_edge2_pos;
static uint8_t s_scroll, s_latch, s_latch2, s_porta, s_portb;
static int s_latch_pending, s_coin_nmi;
static int s_port_p1, s_port_p2, s_port_dsw;
static int s_line;               // (from the vblank start, as MAME's screen at time 0)
static uint32_t s_frame;
static int64_t s_cpu_target, s_snd_target;
static uint64_t s_cpu_frac, s_snd_frac, s_ay_frac, s_disc_frac;
static m6502_t s_cpu, s_snd;
static ay8910_t s_ay;
static int16_t *s_chbuf;
static int s_ch_cap;
static uint8_t s_tiles[32][16 * 16];   // the edge tiles (gfx1, 16x16x3)
static uint8_t (*s_headlight_hp)[128][128];
#define s_headlight (*s_headlight_hp)
static int s_tile_count;
static uint32_t s_rgb[0x30];
static uint8_t (*s_scr_hp)[256 * 256];
#define s_scr (*s_scr_hp)
static struct {
	uint8_t vram[0x400], chr[0x1800], video_control, video_flags, headlight_pos, edge1_pos, edge2_pos, scroll, flip_dsw;
} *s_snap;

static int vblank(void) { return s_frame && s_line < LINES - VBSTART; }

// ---- the discrete netlist (madalien_discrete) ----
typedef struct { double cap, out, last_rc[3], exp[3]; int ff; } d555;
typedef struct { double a1, a2, b0, b1, b2, x1, x2, y1, y2; } dfilt2;
static struct {
	d555 n50, n52, n65;
	uint32_t n54, n67;
	double n56_last, n68_last, n57, n60_cap, n90_amp;
	dfilt2 f21, f41;
} s_d;

// RC_CHARGE_EXP for the sample time
static double rc_exp(double rc) { return 1.0 - exp(-1.0 / DISC_RATE / rc); }

// DSD_555_ASTBL: reset, R1, R2, C, the control voltage (< 0: none), v_pos (v_charge too); out_cap: DISC_555_OUT_CAP
// (else DISC_555_OUT_COUNT_R_X)
static void step_555(d555 *s, int reset, double r1, double r2, double c, double ctrlv, double v_pos, int out_cap)
{
	if (reset) {
		s->out = 0;
		s->ff = 1;
		s->cap = 0;
		return;
	}
	int count_r = 0, ff = s->ff, update_exponent = 0;
	double threshold = v_pos * 2.0 / 3.0, trigger = v_pos / 3.0, v_cap = s->cap, v_cap_next = 0, x_time = 0, exponent;
	if (ctrlv >= 0) {
		if (ctrlv < .25)
			return;
		threshold = ctrlv;
		trigger = ctrlv / 2.0;
		if (v_cap >= threshold)
			ff = 0;
		else if (v_cap <= trigger) {
			ff = 1;
			count_r++;
		}
	}
	const double v_charge = v_pos;
	const double t_bleed = 10e6 * c, t_charge = (r1 + r2) * c, t_discharge = r2 * c;
	if (s->last_rc[0] != t_bleed || s->last_rc[1] != t_charge || s->last_rc[2] != t_discharge) {
		s->last_rc[0] = t_bleed;
		s->last_rc[1] = t_charge;
		s->last_rc[2] = t_discharge;
		s->exp[0] = rc_exp(t_bleed);
		s->exp[1] = rc_exp(t_charge);
		s->exp[2] = rc_exp(t_discharge);
	}
	double dt = 1.0 / DISC_RATE;
	do {
		if (ff) {
			if (r1 == 0) {   // (no charge resistor: the cap bleeds)
				exponent = update_exponent ? 1.0 - exp(-dt / t_bleed) : s->exp[0];
				v_cap_next = v_cap - v_cap * exponent;
				dt = 0;
			} else {
				exponent = update_exponent ? 1.0 - exp(-dt / t_charge) : s->exp[1];
				v_cap_next = v_cap + (v_charge - v_cap) * exponent;
				dt = 0;
				if (v_cap_next >= threshold) {
					dt = t_charge * log(1.0 / (1.0 - ((v_cap_next - threshold) / (v_charge - v_cap))));
					x_time = dt;
					v_cap_next = threshold;
					ff = 0;
					update_exponent = 1;
				}
			}
		} else {
			exponent = update_exponent ? 1.0 - exp(-dt / t_discharge) : s->exp[2];
			v_cap_next = v_cap - v_cap * exponent;
			dt = 0;
			if (v_cap_next <= trigger) {
				if (v_cap_next < trigger)
					dt = t_discharge * log(1.0 / (1.0 - ((trigger - v_cap_next) / v_cap)));
				x_time = dt;
				v_cap_next = trigger;
				ff = 1;
				count_r++;
				update_exponent = 1;
			}
		}
		v_cap = v_cap_next;
	} while (dt);
	s->cap = v_cap;
	x_time *= DISC_RATE;
	s->out = out_cap ? v_cap : count_r ? count_r + x_time : 0;
	s->ff = ff;
}

// DISC_OP_AMP_FILTER_IS_BAND_PASS_1M: r1 || r3 into rF, c1, c2 (calculate_filter2_coefficients, band pass)
static void filt2_init(dfilt2 *f, double r1, double r3, double rf, double c1, double c2)
{
	const double rt = 1.0 / (1.0 / r1 + 1.0 / r3);
	const double fc = 1.0 / (2 * M_PI * sqrt(rt * rf * c1 * c2));
	const double d = (c1 + c2) / sqrt(rf / rt * c1 * c2);
	const double gain = -rf / rt * c2 / (c1 + c2);
	const double two_t = 2.0 * DISC_RATE, two_t2 = two_t * two_t;
	const double wc = DISC_RATE * 2.0 * tan(M_PI * fc / DISC_RATE), wc2 = wc * wc;
	const double den = two_t2 + d * wc * two_t + wc2;
	memset(f, 0, sizeof(*f));
	f->a1 = 2.0 * (-two_t2 + wc2) / den;
	f->a2 = (two_t2 - d * wc * two_t + wc2) / den;
	f->b0 = d * wc * two_t / den * gain;
	f->b1 = 0;
	f->b2 = -d * wc * two_t / den * gain;
}
// (the input Millman'd through r1 || r3 with vRef 0; the output clipped to vP - 1.5 / vN)
static double filt2_step(dfilt2 *f, double in, double r1, double r3)
{
	const double rt = 1.0 / (1.0 / r1 + 1.0 / r3);
	const double v = in / r1 * rt;
	double out = -f->a1 * f->y1 - f->a2 * f->y2 + f->b0 * v + f->b1 * f->x1 + f->b2 * f->x2;
	f->x2 = f->x1;
	f->x1 = v;
	f->y2 = f->y1;
	if (out > 5.0 - 1.5) out = 5.0 - 1.5;
	if (out < -5.0) out = -5.0;
	f->y1 = out;
	return out;
}

// DST_DAC_R1 without a cap: the Millman voltage of the set bits (a step past the ladder's: 0, as MAME's table)
static double dac_r1(uint32_t data, const double *r, int len)
{
	if (data >= (1u << len))
		return 0;
	double rt = 0, i = 0;
	for (int b = 0; b < len; b++)
		if (r[b] > 0) {
			rt += 1.0 / r[b];
			if ((data >> b) & 1)
				i += 3.4 / r[b];
		}
	return i / rt;
}

static void disc_reset(void)
{
	memset(&s_d, 0, sizeof(s_d));
	s_d.n50.ff = s_d.n52.ff = s_d.n65.ff = 1;
	filt2_init(&s_d.f21, 1270, 10000, 100000, 0.1e-6, 0.1e-6);
	filt2_init(&s_d.f41, 1270, 10000, 100000, 0.068e-6, 0.068e-6);
}

// one 48 kHz step from the three PSG voltages; returns the output sample (NODE_90 x 1.1)
static int32_t disc_step(double psg_a, double psg_b, double psg_c)
{
	static const double r_e1b[2] = { 22000, 22000 }, r_e2[4] = { 0, 33000, 22000, 33000 };
	const uint8_t pa = s_porta, pb = s_portb;
	const double pa1 = (pa & 0x01) * 270.0, pa5 = (pa & 0x10) * 270.0, pa8 = (pa & 0x80) * 270.0;
	const int pa2 = pa & 0x02, pa3 = pa & 0x04, pa4 = pa & 0x08, pa6 = pa & 0x20;
	const double pb1 = (pb & 0x01) * 270.0;
	const int pb23 = (pb & 0x06) / 2, pb45 = (pb & 0x18) / 8, pb6 = pb & 0x20;
	const double pb7 = (pb & 0x40) * ((3.4 - 1) / 0x40);
	// the PSG filtering: the top, middle and bottom op-amps, mixed
	const double n21 = filt2_step(&s_d.f21, pa2 ? psg_a : 0, 1270, 10000);
	const double n30 = psg_b * (3300.0 / (3300.0 + 3300.0));
	const double n31 = pa4 ? n30 : 0;
	const double n32 = (psg_a - n30) * (-10000.0 / 3300.0) + n30;
	const double n33 = pa3 ? n32 : n31;
	const double n41 = filt2_step(&s_d.f41, pa6 ? psg_b : 0, 1270, 10000);
	const double n48 = (n21 / 10000 + n33 / 10000 + n41 / 10000) / (3.0 / 10000);
	// the player motor: 1L (its cap voltage) the control voltage of 1F, clocking a 74161
	step_555(&s_d.n50, 0, 4700, 22000, 2.2e-6, -1, 5.0, 1);
	const double n51 = pb45 == 0 ? 0 : pb45 == 1 ? 23000 : pb45 == 2 ? 16000 : 1.0 / (1.0 / 23000 + 1.0 / 16000);
	step_555(&s_d.n52, pb45 == 0, n51, 10000, 0.22e-6, s_d.n50.out, 5.0 - .5, 0);
	if (pb45 == 0)
		s_d.n54 = 0;
	else
		s_d.n54 = (s_d.n54 + (uint32_t)(int)s_d.n52.out) & 15;
	const double n56 = dac_r1(s_d.n54, r_e1b, 2);
	s_d.n57 += (n56 - s_d.n57) * rc_exp((22000.0 / 2 + 22000) * 0.033e-6);
	const double n59 = pb23 == 0 ? 0 : pb23 == 1 ? 270 : pb23 == 2 ? 22270 : 1.0 / (1.0 / 270 + 1.0 / 22270);
	// the enemy motor: 1C with its control voltage from bit 7 through a CR filter
	const double n60 = pb7 - s_d.n60_cap;
	s_d.n60_cap += n60 * rc_exp(100000 * 4.7e-6);
	const double n62 = n60 < 0 ? 0 : n60 > 12 ? 12 : n60;
	const double n64 = (n62 / 1500 + 5.0 / 5000) / (1.0 / 1500 + 1.0 / 5000 + 1.0 / 10000);
	step_555(&s_d.n65, !pb6, 47000, 22000, 0.033e-6, n64, 5.0, 0);
	if (!pb6)
		s_d.n67 = 0;
	else
		s_d.n67 = (s_d.n67 + (uint32_t)(int)s_d.n65.out) & 15;
	const double n68 = dac_r1(s_d.n67, r_e2, 4);
	// the final mixer (resistors, some switched in by the 4066s: a node of 0 = open), rF 100k, cAmp 1 uF
	const double r[7] = { 10000, 10000.0 / 3, 10000, 10000, 22000 + 22000.0 / 3, 22000 + 22000 + 11000,
	                      33000 + 1.0 / (1.0 / 33000 + 1.0 / 22000 + 1.0 / 33000) };
	const double node[7] = { pa1, -1, pa5, pa8, pb1, n59, -1 };
	const double in[7] = { psg_a, n48, psg_b, psg_c, n56, s_d.n57, n68 };
	double rt = 1.0 / 100000, i = 0;
	for (int k = 0; k < 7; k++) {
		double rr = r[k];
		if (node[k] >= 0) {
			if (node[k] == 0)
				continue;
			rr += node[k];
		}
		rt += 1.0 / rr;
		i += in[k] / rr;
	}
	double v = i / rt;
	s_d.n90_amp += (v - s_d.n90_amp) * rc_exp(100000 * 1e-6);
	v -= s_d.n90_amp;
	return (int32_t)(v * (32768.0 / 3.4) * 1.1);
}

static void port_a_w(void *u, uint8_t v) { (void)u; s_porta = v; }
static void port_b_w(void *u, uint8_t v) { (void)u; s_portb = v; }

// ---- sound CPU ----
static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x2000) return s_sram[a & 0x3ff];
	if (a >= 0x6000 && a < 0x8000) return s_sram2[a & 3];
	if (a >= 0x8000 && a < 0xa000 && (a & 3) == 0) {   // the latch (its read acknowledges)
		s_latch_pending = 0;
		m6502_set_irq(&s_snd, 0);
		return s_latch;
	}
	if (a >= 0xf800) return s_srom[a];
	return 0;
}
static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x2000) s_sram[a & 0x3ff] = v;
	else if (a >= 0x6000 && a < 0x8000) s_sram2[a & 3] = v;
	else if (a >= 0x8000 && a < 0xa000) {
		if ((a & 3) < 2) ay8910_write(&s_ay, a & 1, v);
		else if ((a & 3) == 2) s_latch2 = v;
	}
}

// ---- main ----
static uint8_t main_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x400) return s_ram[a];
	if (a >= 0x6000 && a < 0x6400) return s_vram[a - 0x6000];
	if (a >= 0x6400 && a < 0x6800) return s_ram2[a - 0x6400];
	if (a >= 0x6800 && a < 0x8000) return s_chr[a - 0x6800];
	if (a >= 0xa000) return s_rom[a];
	if (a >= 0x8000 && a < 0x9000) {
		switch (a & 0xf) {
		case 0x6: return s_latch2;
		case 0x8:
			if (a < 0x8800)   // shift_r
				return s_shift[((s_shift_hi & 7) << 8) | s_shift_lo];
			break;
		case 0x9:
			if (a < 0x8800) {   // shift_rev_r: the operands and the result reversed
				uint8_t lo = 0, r = 0;
				for (int i = 0; i < 8; i++)
					lo |= (uint8_t)(((s_shift_lo >> i) & 1) << (7 - i));
				const uint8_t t = s_shift[(((s_shift_hi ^ 7) & 7) << 8) | lo];
				r = (uint8_t)(((t >> 7) & 1) << 7 | (t & 1) << 6 | ((t >> 1) & 1) << 5 | ((t >> 2) & 1) << 4 |
				              ((t >> 3) & 1) << 3 | ((t >> 4) & 1) << 2 | ((t >> 5) & 1) << 1 | ((t >> 6) & 1));
				return r & 0x7f;
			}
			break;
		}
		return 0;
	}
	if (a >= 0x9000 && a < 0xa000)
		switch (a & 0xf) {
		case 0: return (uint8_t)rbx_port_read(s_port_p1);
		case 1:
			rbx_port_custom(s_port_dsw, 0x80, vblank() ? 0x80 : 0);
			return (uint8_t)rbx_port_read(s_port_dsw);
		case 2: return (uint8_t)rbx_port_read(s_port_p2);
		}
	return 0;
}
static void main_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x400) s_ram[a] = v;
	else if (a >= 0x6000 && a < 0x6400) s_vram[a - 0x6000] = v;
	else if (a >= 0x6400 && a < 0x6800) s_ram2[a - 0x6400] = v;
	else if (a >= 0x6800 && a < 0x8000) s_chr[a - 0x6800] = v;
	else if (a >= 0x8000 && a < 0x9000) {
		const int lo = a & 0xf, near = a < 0x8800;
		if (lo == 0x4) s_video_control = v;
		else if (lo == 0x6) {   // the sound latch (its pending data: the sound IRQ)
			s_latch = v;
			s_latch_pending = 1;
			m6502_set_irq(&s_snd, 1);
		} else if (near) {
			switch (lo) {
			case 0x8: s_shift_hi = v; break;
			case 0x9: s_shift_lo = v; break;
			case 0xb: s_video_flags = v; break;
			case 0xc: s_headlight_pos = v; break;
			case 0xd: s_edge1_pos = v; break;
			case 0xe: s_edge2_pos = v; break;
			case 0xf: s_scroll = v; break;
			}
		}
	}
}

// the coin on the NMI (coin_inserted: newval ? CLEAR : ASSERT, the bit active low)
static void coins(void)
{
	const int nmi = !(rbx_port_read(s_port_p2) & 0x80);
	if (nmi != s_coin_nmi)
		m6502_set_nmi(&s_cpu, nmi);
	s_coin_nmi = nmi;
}

// ---- video ----
// a tilemap pixel's logical coordinate for screen x (MAME effective_rowscroll / colscroll and the flip): size the
// tilemap's, scroll the set value, screen 256
static int tm_coord(int x, int size, int scroll, int flip)
{
	int value = flip ? 256 - size + scroll : -scroll;
	value = value < 0 ? size - (-value) % size : value % size;
	int p = (x - value) % size;
	if (p < 0) p += size;
	return flip ? size - 1 - p : p;
}

static void draw_edge(int second, int mode, int flipx, int flipy, int y0, int y1)
{
	const int cols = mode < 2 ? 16 : 32, w = cols * 16;
	const uint8_t *map = s_map + ((s_snap->video_flags & 0x08) << 6) + (second ? 0x80 : 0);
	const int color = (s_snap->video_flags & 4) ? 16 : 0;
	const int sx = -(s_snap->scroll & 0xfc), sy = (second ? s_snap->edge2_pos : s_snap->edge1_pos) & 0x7f;
	if (y0 < 0) y0 = 0;
	if (y1 > 255) y1 = 255;
	for (int y = y0; y <= y1; y++) {
		const int ty = tm_coord(y, 128, sy, flipy), row = ty >> 4;
		uint8_t *d = s_scr + y * 256;
		for (int x = 0; x < 256; x++) {
			const int tx = tm_coord(x, w, sx, flipx), col = tx >> 4;
			const int section = mode == 0 ? 0 : mode == 1 ? 1 : mode == 2 ? (col >> 4) & 1 : !((col >> 4) & 1);
			const int idx = (section << 8) | ((~col & 0x0f) << 3) | row;
			const int code = map[idx] % s_tile_count;
			d[x] = (uint8_t)(color + s_tiles[code][(ty & 15) * 16 + (tx & 15)]);
		}
	}
}

static void ma_render(void)
{
	for (int i = 0; i < 0x30; i++)
		rbx_video.palette[i] = s_rgb[i];
	const int flip = s_snap->flip_dsw && (s_snap->video_control & 1);
	const int mode = s_snap->scroll & 3;
	memset(s_scr, 0, sizeof(s_scr));
	// the road edges
	if (flip) {
		draw_edge(0, mode, 1, 1, s_snap->edge1_pos | 0x80, 255);
		draw_edge(1, mode, 1, 0, 0, (s_snap->edge2_pos & 0x7f) ^ 0x7f);
	} else {
		draw_edge(0, mode, 0, 0, 0, (s_snap->edge1_pos & 0x7f) ^ 0x7f);
		draw_edge(1, mode, 0, 1, s_snap->edge2_pos | 0x80, 255);
	}
	// the foreground: RAM characters (3 planes 0x800 apart), TILEMAP_SCAN_COLS_FLIP_X, pen 0 transparent
	const uint8_t *chr = s_snap->chr;
	for (int y = 0; y < 256; y++) {
		const int ty = flip ? 255 - y : y;
		for (int x = 0; x < 256; x++) {
			const int tx = flip ? 255 - x : x;
			const int code = s_snap->vram[(31 - (tx >> 3)) * 32 + (ty >> 3)];
			const int o = code * 8 + (ty & 7), b = 7 - (tx & 7);
			const int p = ((chr[0x1000 + o] >> b) & 1) << 2 | ((chr[0x800 + o] >> b) & 1) << 1 | ((chr[o] >> b) & 1);
			if (p)
				s_scr[y * 256 + x] = (uint8_t)(0x20 + p);
		}
	}
	// the highlight of section A (and the tunnel lit by bit 1)
	if (mode != 1 || (s_snap->video_flags & 2)) {
		int min_x = 0, max_x = 0xff;
		if (!(s_snap->video_flags & 2)) {
			if (mode == 2) min_x = s_snap->scroll & 0xfc;
			else if (mode == 3) max_x = (s_snap->scroll & 0xfc) - 1;
		}
		if (flip) {
			const int m = max_x;
			max_x = 0xff - min_x;
			min_x = 0xff - m;
		}
		for (int y = 0; y < 256; y++)
			for (int x = min_x < 0 ? 0 : min_x; x <= max_x && x < 256; x++)
				s_scr[y * 256 + x] |= 8;
	}
	// the headlight
	if (s_snap->video_flags & 1)
		for (int y = 0; y < 0x80; y++) {
			uint8_t hy = (uint8_t)(y - s_snap->headlight_pos);
			if (flip) hy = (uint8_t)~hy;
			for (int x = 0; x < 0x80; x++) {
				const uint8_t hx = (uint8_t)(flip ? ~x : x);
				if (s_headlight[y][x])
					s_scr[hy * 256 + hx] |= 8;
			}
		}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = 0; y < 256; y++)
		for (int x = 0; x < 256; x++)
			bm[y * pitch + x] = s_scr[y * 256 + x];
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->vram, s_vram, sizeof(s_vram));
	memcpy(s_snap->chr, s_chr, sizeof(s_chr));
	s_snap->video_control = s_video_control;
	s_snap->video_flags = s_video_flags;
	s_snap->headlight_pos = s_headlight_pos;
	s_snap->edge1_pos = s_edge1_pos;
	s_snap->edge2_pos = s_edge2_pos;
	s_snap->scroll = s_scroll;
	s_snap->flip_dsw = (rbx_port_read(s_port_dsw) >> 6) & 1;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void ma_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_ram2, 0, sizeof(s_ram2));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_chr, 0, sizeof(s_chr));
	memset(s_sram, 0, sizeof(s_sram));
	memset(s_sram2, 0, sizeof(s_sram2));
	s_video_control = s_shift_hi = s_shift_lo = s_video_flags = s_headlight_pos = s_edge1_pos = s_edge2_pos = 0;
	s_scroll = s_latch = s_latch2 = s_porta = s_portb = 0;
	s_latch_pending = s_coin_nmi = 0;
	ay8910_reset(&s_ay);
	disc_reset();
	m6502_reset(&s_cpu);
	m6502_set_irq(&s_cpu, 0);
	m6502_set_nmi(&s_cpu, 0);
	m6502_reset(&s_snd);
	m6502_set_irq(&s_snd, 0);
	m6502_set_nmi(&s_snd, 0);
	s_line = 0;
	s_frame = 0;
	s_cpu_target = s_snd_target = 0;
	s_cpu_frac = s_snd_frac = s_ay_frac = s_disc_frac = 0;
}

static void ma_exit(void);

static int ma_init(const rbx_game *g)
{
	if (!s_headlight_hp && !(s_headlight_hp = calloc(1, sizeof(*s_headlight_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_scr_hp && !(s_scr_hp = calloc(1, sizeof(*s_scr_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t gs, hs, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("audiocpu", &s_srom_size);
	s_map = rbx_region("user1", &s_map_size);
	s_shift = rbx_region("user2", &s_shift_size);
	const uint8_t *gfx = rbx_region("gfx1", &gs), *hl = rbx_region("gfx2", &hs), *prom = rbx_region("proms", &ps);
	if (!s_rom || s_rom_size < 0x10000 || !s_srom || s_srom_size < 0x10000 || !s_map || s_map_size < 0x400 ||
	    !s_shift || s_shift_size < 0x800 || !gfx || gs < 0xc00 || !hl || hs < 0x400 || !prom || ps < 0x20) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	// the edge tiles: 16x16x3, the planes RGN_FRAC thirds, the columns in four byte groups (bits 3-0 of each)
	const uint32_t third = gs / 3;
	s_tile_count = (int)(third / 64);
	if (s_tile_count > 32) s_tile_count = 32;
	for (int t = 0; t < s_tile_count; t++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				const uint32_t o = (uint32_t)t * 64 + (uint32_t)(3 - (x >> 2)) * 16 + (uint32_t)y;
				const int b = 3 - (x & 3);
				s_tiles[t][y * 16 + x] = (uint8_t)(((gfx[2 * third + o] >> b) & 1) << 2 | ((gfx[third + o] >> b) & 1) << 1 |
				                                   ((gfx[o] >> b) & 1));
			}
	// the headlight: 128x64 1bpp, drawn twice (the second half flipped vertically)
	for (int y = 0; y < 64; y++)
		for (int x = 0; x < 128; x++) {
			const uint32_t bit = (uint32_t)(0x78 - (x >> 3) * 8 + (x & 7)) + (y < 32 ? (uint32_t)y * 0x80 : 0x1000 + (uint32_t)(y - 32) * 0x80);
			const uint8_t p = (hl[bit >> 3] >> (7 - (bit & 7))) & 1;
			s_headlight[y][x] = p;
			s_headlight[127 - y][x] = p;
		}
	// the palette: PROM colours (bits 0-1 red 0x3f / 0xc0 ...), pens 0x10-0x1f the edge colour 2 swap, 0x20- the chars
	uint32_t ind[0x20];
	for (int i = 0; i < 0x20; i++) {
		const uint8_t p = prom[i];
		const int r = (p & 1 ? 0x3f : 0) + (p & 2 ? 0xc0 : 0), gg = (p & 4 ? 0x3f : 0) + (p & 8 ? 0xc0 : 0);
		const int b = (p & 0x10 ? 0x3f : 0) + (p & 0x20 ? 0xc0 : 0);
		ind[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	for (int i = 0; i < 0x10; i++) {
		s_rgb[i] = ind[i];
		int c = i;
		if ((i >> 1) & 1) c ^= 6;
		if ((i >> 2) & 1) c ^= 6;
		s_rgb[0x10 + i] = ind[c];
		s_rgb[0x20 + i] = ind[i | 0x10];
	}
	s_snap = calloc(1, sizeof(*s_snap));
	// AY-3-8910 1 MHz, its three channels into the netlist (MAME's default legacy tables)
	ay8910_init(&s_ay, AY8910_TYPE_AY8910, 4000000 / 4, AY8910_LEGACY_OUTPUT, 256, 1, 0);
	s_ay.port_a_write = port_a_w;
	s_ay.port_b_write = port_b_w;
	s_ch_cap = (int)(ay8910_rate(&s_ay) / 50) + 64;
	s_chbuf = (int16_t *)calloc((size_t)s_ch_cap * 3, sizeof(int16_t));
	if (!s_snap || !s_chbuf || rbx_audio_setup(DISC_RATE) || rbx_video_setup(256, 256, 0x30)) {
		rbx_fail("%s: out of memory", g->set);
		ma_exit();
		return -1;
	}
	m6502_init(&s_cpu, M6502_NMOS);
	s_cpu.read = main_r;
	s_cpu.write = main_w;
	for (int p = 0xa0; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + p * 256;
	for (int p = 0; p < 4; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = s_ram + p * 256;
	for (int p = 0; p < 4; p++) {
		s_cpu.rpage[0x60 + p] = s_cpu.wpage[0x60 + p] = s_vram + p * 256;
		s_cpu.rpage[0x64 + p] = s_cpu.wpage[0x64 + p] = s_ram2 + p * 256;
	}
	for (int p = 0; p < 0x18; p++)
		s_cpu.rpage[0x68 + p] = s_cpu.wpage[0x68 + p] = s_chr + p * 256;
	m6502_init(&s_snd, M6502_NMOS);
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	for (int p = 0xf8; p < 0x100; p++)
		s_snd.rpage[p] = s_srom + p * 256;
	s_port_p1 = rbx_port_find("PLAYER1");
	s_port_p2 = rbx_port_find("PLAYER2");
	s_port_dsw = rbx_port_find("DSW");
	ma_reset();
	return 0;
}

static void ma_exit(void)
{
	free(s_snap);
	s_snap = NULL;
	free(s_chbuf);
	s_chbuf = NULL;
	free(s_headlight_hp);
	s_headlight_hp = NULL;
	free(s_scr_hp);
	s_scr_hp = NULL;
}

static void sound_line(void)
{
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&s_ay_frac, 4000000 / 4, FRAME_MHZ, LINES);
	const int n = ay8910_render_ch(&s_ay, clocks, s_chbuf, s_ch_cap);
	const int steps = rbx_line_cycles(&s_disc_frac, DISC_RATE, FRAME_MHZ, LINES);
	for (int k = 0; k < steps; k++) {   // (the native samples of the step averaged: MAME's resampling)
		int a = n * k / steps, b = n * (k + 1) / steps;
		if (b <= a) b = a + 1;
		if (b > n) b = n;
		if (a >= b) a = b - 1;
		double v[3] = { 0, 0, 0 };
		if (a >= 0)
			for (int j = a; j < b; j++)
				for (int c = 0; c < 3; c++)
					v[c] += s_chbuf[3 * j + c];
		for (int c = 0; c < 3; c++)   // (DISCRETE_INPUTX_STREAM 2.0 / 32768, 0.25)
			v[c] = (a >= 0 ? v[c] / (b - a) : 0) * 2.0 / 32768.0 + .25;
		rbx_audio_put(disc_step(v[0], v[1], v[2]));
	}
}

static void ma_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	coins();
	for (int i = 0; i < LINES; i++) {   // (from line 256, the vblank start)
		s_line = i;
		if (i == 0 && s_frame)   // the MC6845's vsync: the sound NMI
			m6502_set_nmi(&s_snd, 1);
		else if (i == 1)
			m6502_set_nmi(&s_snd, 0);
		s_cpu_target += rbx_line_cycles(&s_cpu_frac, XTAL / 8, FRAME_MHZ, LINES);
		m6502_run_until(&s_cpu, s_cpu_target);
		s_snd_target += rbx_line_cycles(&s_snd_frac, 4000000 / 8, FRAME_MHZ, LINES);
		m6502_run_until(&s_snd, s_snd_target);
		sound_line();
	}
	vblank_capture();
	s_frame++;
}

static int ma_state(void *ctx, int load)
{
	uint8_t cpu[M6502_STATE_SIZE], snd[M6502_STATE_SIZE], ay[256];
	const uint32_t ays = ay8910_state_size();
	if (!load) {
		memcpy(cpu, &s_cpu, M6502_STATE_SIZE);
		memcpy(snd, &s_snd, M6502_STATE_SIZE);
		memcpy(ay, &s_ay, ays);
	}
	int64_t tg[2] = { s_cpu_target, s_snd_target };
	uint64_t fr[4] = { s_cpu_frac, s_snd_frac, s_ay_frac, s_disc_frac };
	uint8_t regs[16] = { s_video_control, s_shift_hi, s_shift_lo, s_video_flags, s_headlight_pos, s_edge1_pos,
	                     s_edge2_pos, s_scroll, s_latch, s_latch2, s_porta, s_portb };
	int32_t misc[4] = { (int32_t)s_frame, s_latch_pending, s_coin_nmi, 0 };
	int rc = 0;
	rc |= rbx_state_io(ctx, "cpu", cpu, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "snd", snd, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "ram2", s_ram2, sizeof(s_ram2));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "chr", s_chr, sizeof(s_chr));
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_state_io(ctx, "sram2", s_sram2, sizeof(s_sram2));
	rc |= rbx_state_io(ctx, "ay", ay, sizeof(ay));
	rc |= rbx_state_io(ctx, "disc", &s_d, sizeof(s_d));
	rc |= rbx_state_io(ctx, "tg", tg, sizeof(tg));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "regs", regs, sizeof(regs));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		memcpy(&s_cpu, cpu, M6502_STATE_SIZE);
		memcpy(&s_snd, snd, M6502_STATE_SIZE);
		memcpy(&s_ay, ay, ays);
		s_cpu_target = tg[0];
		s_snd_target = tg[1];
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_ay_frac = fr[2];
		s_disc_frac = fr[3];
		s_video_control = regs[0];
		s_shift_hi = regs[1];
		s_shift_lo = regs[2];
		s_video_flags = regs[3];
		s_headlight_pos = regs[4];
		s_edge1_pos = regs[5];
		s_edge2_pos = regs[6];
		s_scroll = regs[7];
		s_latch = regs[8];
		s_latch2 = regs[9];
		s_porta = regs[10];
		s_portb = regs[11];
		s_frame = (uint32_t)misc[0];
		s_latch_pending = misc[1];
		s_coin_nmi = misc[2];
	}
	return rc ? -1 : 0;
}

static int ma_save(void *ctx) { return ma_state(ctx, 0); }
static int ma_load(void *ctx) { return ma_state(ctx, 1); }
static int ma_supported(const char *machine) { return !strcmp(machine, "madalien"); }

const rbx_driver rbx_drv_madalien = { "DECO Mad Alien", ma_init, ma_exit, ma_reset, ma_frame, ma_save, ma_load,
                                      ma_supported, ma_render, NULL };
