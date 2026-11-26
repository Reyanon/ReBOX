/*  The Galaxian / Moon Cresta discrete sound, after MAME 0.289 src/mame/galaxian/galaxian_a.cpp (BSD-3-Clause,
    copyright-holders: Couriersud) and the discrete nodes it uses: DISCRETE_DAC_R1, DISCRETE_555_CC,
    DISCRETE_555_ASTABLE_CV, DISCRETE_MIXER, DISCRETE_NOTE, DISCRETE_LFSR_NOISE, DISCRETE_SQUAREWFIX,
    DISCRETE_LOGIC_DFLIPFLOP, DISCRETE_RCDISC5, DISCRETE_OP_AMP_FILTER (band pass 1M), DISCRETE_RCFILTER
    (src/devices/sound/disc_wav.hxx, disc_dev.hxx, disc_flt.hxx, disc_mth.hxx; BSD-3-Clause, copyright-holders:
    K.Wilkins, Derrick Renaud, Couriersud).
    The node graph is MAME's, stepped once a sample in single precision; the exponents of the fixed RC networks are
    computed once (the 555s' overshoot times at a threshold crossing still take a log, as MAME's: a few thousand a
    second at most). The LFSR's exact taps are kept (17 bits, F0 = bit 4 xor !bit 16), clocked at 122.88 kHz and latched
    at the 2V edges (7920 Hz).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "galaxian_a.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TTL_OUT 4.0f
// resistors and capacitors of the schematics (galaxian_a.cpp)
#define R15 100e3
#define R16 220e3
#define R17 470e3
#define R18 1000e3
#define R19 330e3
#define R20 15e3
#define R21 100e3
#define R22 100e3
#define R23 470e3
#define R24 10e3
#define R25 100e3
#define R26 330e3
#define R27 10e3
#define R28 100e3
#define R29 220e3
#define R30 10e3
#define R31 47e3
#define R32 47e3
#define R33 10e3
#define R34 5.1e3
#define MCRST_R34 15e3
#define R35 150e3
#define R36 22e3
#define R37 470e3
#define R38 33e3
#define R39 22e3
#define R40 (2.2e3 * 0.6)
#define R41 100e3
#define R43 2.2e3
#define R44 10e3
#define R45 22e3
#define R46 10e3
#define R47 2.2e3
#define R48 2.2e3
#define R49 10e3
#define R50 22e3
#define R51 33e3
#define R52 15e3
#define R91 10e3
#define C15 1e-6
#define C17 0.01e-6
#define C18 0.01e-6
#define C19 0.01e-6
#define C20 0.1e-6
#define C21 2.2e-6
#define C22 0.01e-6
#define C23 0.01e-6
#define C25 1e-6
#define C26 0.01e-6
#define C27 0.01e-6
#define C28 47e-6
#define C46 0.1e-6

static float rc_exp(double rc, double dt) { return (float)(1.0 - exp(-dt / rc)); }

void galsnd_init(galsnd_t *g, int type, double rate, float gain)
{
	memset(g, 0, sizeof(*g));
	g->type = type;
	const double dt = 1.0 / rate;
	g->dt = (float)dt;
	g->inv_dt = (float)rate;
	g->out_gain = gain;
	// DISCRETE_NOTE: SOUND_CLOCK (18.432 MHz / 12, bongo / 8 more) clocks a sample
	const double clk = 18432000.0 / 12.0 / (type == GALSND_BONGO ? 8.0 : 1.0);
	g->note_clocks = (int)(clk / rate * 65536.0 + 0.5);
	g->lfsr_step = (uint32_t)(18432000.0 / 3.0 * 2.0 / 100.0 / rate * 65536.0 + 0.5);   // (RNG_RATE / 100)
	g->sq_step = (uint32_t)(60.0 * 264.0 / 2.0 / rate * 65536.0 + 0.5);                 // (the 2V signal)
	// DISCRETE_DAC_R1 galaxian_bck_dac: 1M, 470k, 220k, 100k from 4 V, 15k from 4.4 V, 330k to ground
	static const double ladder[4] = { R18, R17, R16, R15 };
	double gt = 1.0 / R20 + 1.0 / R19;
	for (int b = 0; b < 4; b++)
		gt += 1.0 / ladder[b];
	for (int i = 0; i < 16; i++) {
		double cur = 4.4 / R20;
		for (int b = 0; b < 4; b++)
			if ((i >> b) & 1)
				cur += TTL_OUT / ladder[b];
		g->dac_v[i] = (float)(cur / gt);
	}
	// the three background 555s
	static const double r1[3] = { R22, R25, R28 }, r2[3] = { R23, R26, R29 }, c[3] = { C17, C18, C19 };
	for (int i = 0; i < 3; i++) {
		g->bck_t_chg[i] = (float)((r1[i] + r2[i]) * c[i]);
		g->bck_t_dis[i] = (float)(r2[i] * c[i]);
		g->bck_e_chg[i] = rc_exp(g->bck_t_chg[i], dt);
		g->bck_e_dis[i] = rc_exp(g->bck_t_dis[i], dt);
	}
	g->fire_t_chg = (float)((R44 + R45) * C27);
	g->fire_t_dis = (float)(R45 * C27);
	g->fire_e_chg = rc_exp(g->fire_t_chg, dt);
	g->fire_e_dis = rc_exp(g->fire_t_dis, dt);
	g->e_bck_cf = rc_exp(1.0 / (3.0 / R24) * C20, dt);   // (galaxian_bck_mixer_desc: cF on the three 10k in parallel)
	g->e_hit = rc_exp((R35 + R36) * C21, dt);
	g->e_fire_rc = rc_exp(R47 * C28, dt);
	g->e_fire = rc_exp(R41 * C25, dt);
	// the mixer's capacitor on the fire input: with rF (galaxian: R91) its resistor in parallel with rF
	g->e_mix_c = type == GALSND_GALAXIAN ? rc_exp(1.0 / (1.0 / R43 + 1.0 / R91) * C26, dt) : rc_exp(R43 * C26, dt);
	g->e_amp = rc_exp(100e3 * C46, dt);   // (cAmp: 100k as the final stage's impedance)
	// DISCRETE_OP_AMP_FILTER band pass 1M (galaxian_bandpass_desc)
	const double rtot = 1.0 / (1.0 / R35 + 1.0 / R36);
	const double fc = 1.0 / (2.0 * M_PI * sqrt(rtot * R37 * C22 * C23));
	const double d = (C22 + C23) / sqrt(R37 / rtot * C22 * C23);
	const double bgain = -R37 / rtot * C23 / (C22 + C23);
	const double two_t = 2.0 * rate, two_t2 = two_t * two_t;
	const double wc = rate * 2.0 * tan(M_PI * fc / rate);
	const double den = two_t2 + d * wc * two_t + wc * wc;
	g->bp_a1 = (float)(2.0 * (-two_t2 + wc * wc) / den);
	g->bp_a2 = (float)((two_t2 - d * wc * two_t + wc * wc) / den);
	g->bp_b0 = (float)(d * wc * two_t / den * bgain);
	g->bp_b2 = -g->bp_b0;
	g->bp_rtot = (float)rtot;
	galsnd_reset(g);
}

void galsnd_reset(galsnd_t *g)
{
	memset(&g->s, 0, sizeof(g->s));
	for (int i = 0; i < 3; i++)
		g->s.bck[i].ff = 1;
	g->s.fire555.ff = 1;
	g->s.lfo_vcap = 0;
	g->s.note_c1 = 0;   // (dss_note reset: the data, 0 until a pitch is written)
	g->s.pitch = 0;
	// the circuit settled first (half a second: the capacitors charged to their idle levels, no thump at the start)
	int32_t sink[256];
	memset(sink, 0, sizeof(sink));
	for (int k = 0; k < (int)(0.5f / g->dt) / 256; k++)
		galsnd_render(g, sink, 256);
}

void galsnd_sound_w(galsnd_t *g, int offset, uint8_t data)
{
	data &= 1;
	switch (offset & 7) {
	case 0: case 1: case 2: g->s.fs[offset & 3] = data; break;   // FS1-FS3 (the 555s at 8R, 8S, 8T)
	case 3: g->s.hit = data; break;
	case 5: g->s.fire = data; break;
	case 6: case 7: g->s.vol[offset & 1] = data; break;
	default: break;
	}
}

void galsnd_lfo_w(galsnd_t *g, int offset, uint8_t data)
{
	g->s.lfo_val = (uint8_t)((g->s.lfo_val & ~(1 << offset)) | ((data & 1) << offset));
}

void galsnd_pitch_w(galsnd_t *g, uint8_t data) { g->s.pitch = data; }

// DISCRETE_555_ASTABLE_CV (DISC_555_OUT_ENERGY when vhigh_energy, else the square wave), v_charge 5 V, enabled
static float astable(galsnd_555_t *s, float cv, float t_chg, float t_dis, float e_chg, float e_dis, float vhigh, int energy,
                     float dtime, float inv_dt)
{
	if (cv < 0.25f)
		return s->out;   // (MAME ignores the step: the output stays)
	const float thr = cv, trg = cv * 0.5f, vchg = 5.0f;
	float v = s->vcap;
	uint8_t ff = s->ff;
	const uint8_t ff0 = ff;
	int toggles = 0;
	if (v >= thr) {
		ff = 0;
		toggles++;
	} else if (v <= trg) {
		ff = 1;
		toggles++;
	}
	float dt = dtime, x_time = 0;
	int upd = 0;
	do {
		float vn;
		if (ff) {
			const float e = upd ? 1.0f - expf(-dt / t_chg) : e_chg;
			vn = v + (vchg - v) * e;
			dt = 0;
			if (vn >= thr) {
				dt = t_chg * logf(1.0f / (1.0f - (vn - thr) / (vchg - v)));
				x_time = dt;
				vn = thr;
				ff = 0;
				toggles++;
				upd = 1;
			}
		} else {
			const float e = upd ? 1.0f - expf(-dt / t_dis) : e_dis;
			vn = v - v * e;
			dt = 0;
			if (vn <= trg) {
				if (vn < trg)
					dt = t_dis * logf(1.0f / (1.0f - (trg - vn) / v));
				x_time = dt;
				vn = trg;
				ff = 1;
				toggles++;
				upd = 1;
			}
		}
		v = vn;
		if (!(dt > 0))   // (also ends a NaN)
			break;
	} while (1);
	s->vcap = v;
	s->ff = ff;
	float out;
	if (energy) {
		x_time *= inv_dt;
		if (x_time == 0)
			x_time = 1.0f;
		out = vhigh * (ff ? x_time : 1.0f - x_time);
	} else
		out = toggles >= 2 ? (ff0 ? 0 : vhigh) : ff * vhigh;
	s->out = out;
	return out;
}

void galsnd_render(galsnd_t *g, int32_t *out, int n)
{
	galsnd_state_t *s = &g->s;
	const float dt = g->dt;
	for (int k = 0; k < n; k++) {
		// ---- NOISE: the LFSR, latched at the 2V edges
		s->lfsr_acc += g->lfsr_step;
		while (s->lfsr_acc >= 0x10000) {
			s->lfsr_acc -= 0x10000;
			s->lfsr = ((s->lfsr << 1) | s->lfsr_fb) & 0x1ffff;
			s->lfsr_fb = (uint8_t)(((s->lfsr >> 4) ^ ~(s->lfsr >> 16)) & 1);
		}
		s->sq_acc += g->sq_step;
		if (s->sq_acc >= 0x10000) {   // (a low to high edge of the square wave a period)
			s->sq_acc -= 0x10000;
			s->noise = s->lfsr_fb;
		}
		const float noise = (float)s->noise;
		// ---- BACKGROUND: the DAC, the 555 CC's ramp, the three 555s
		{
			const float vin = g->dac_v[s->lfo_val & 15];
			const float vlim = vin + 0.7f;
			float i = (5.0f - vlim) * (float)(1.0 / R21);
			if (i < 0)
				i = 0;
			float v = s->lfo_vcap;
			if (i > 0) {
				float rem = dt;
				for (int guard = 0; guard < 8 && rem > 0; guard++) {
					float vn = v + i * rem * (float)(1.0 / C15);
					if (vn > vlim)
						vn = vlim;
					rem = 0;
					if (vn >= 5.0f * 2.0f / 3.0f) {   // (threshold reached: discharged at once to the trigger)
						rem = (float)C15 * (vn - 5.0f * 2.0f / 3.0f) / i;
						vn = 5.0f / 3.0f;
					}
					v = vn;
				}
			} else
				v -= v * (1.0f - expf(-dt / 1.0f));   // (no current: the bleed; not reached with this DAC)
			s->lfo_vcap = v;
		}
		float cv = s->lfo_vcap * (float)(R33 / (1.0 / (1.0 / R31 + 1.0 / R32 + 1.0 / R33))) - (float)(5.0 * R33 / R31);
		if (cv < 0) cv = 0;
		if (cv > 5) cv = 5;
		float bsum = 0;
		for (int j = 0; j < 3; j++) {
			if (!s->fs[j]) {   // (reset)
				s->bck[j].ff = 1;
				s->bck[j].vcap = 0;
				s->bck[j].out = 0;
				continue;
			}
			bsum += astable(&s->bck[j], cv, g->bck_t_chg[j], g->bck_t_dis[j], g->bck_e_chg[j], g->bck_e_dis[j], 4.5f, 1,
			                dt, g->inv_dt);
		}
		s->bck_cf += (bsum * (1.0f / 3.0f) - s->bck_cf) * g->e_bck_cf;   // (NODE_120: the average, through cF)
		const float v120 = s->bck_cf;
		// ---- PITCH: the counters (DISCRETE_NOTE, max 255 and 15)
		if (s->pitch != 255) {
			s->note_frac += (uint32_t)g->note_clocks;
			int clocks = (int)(s->note_frac >> 16);
			s->note_frac &= 0xffff;
			while (clocks > 0) {
				const int left = 256 - s->note_c1;
				if (clocks >= left) {
					clocks -= left;
					s->note_c1 = s->pitch;
					s->note_c2 = (s->note_c2 + 1) & 15;
				} else {
					s->note_c1 += clocks;
					clocks = 0;
				}
			}
		} else
			s->note_frac = (s->note_frac + (uint32_t)g->note_clocks) & 0xffff;
		const float b0 = (s->note_c2 & 1) ? TTL_OUT : 0, b2 = (s->note_c2 & 4) ? TTL_OUT : 0,
		            b3 = (s->note_c2 & 8) ? TTL_OUT : 0;
		// ---- HIT: RCDISC5 gated by the noise, the band pass
		float v155;
		{
			float u = (s->hit ? TTL_OUT : 0) - 0.7f;
			if (u < 0) u = 0;
			float diff = u - s->hit_vcap;
			if (s->noise) {
				if (diff < 0)
					diff *= g->e_hit;
				s->hit_vcap += diff;
				v155 = s->hit_vcap;
			} else {
				if (diff > 0)
					s->hit_vcap = u;
				v155 = 0;
			}
		}
		float v157;
		{
			const float vref = 2.0f;   // (5 V x R39 / (R38 + R39))
			const float v = ((v155 - vref) * (float)(1.0 / R35) + (0 - vref) * (float)(1.0 / R36)) * g->bp_rtot;
			float vo = -g->bp_a1 * s->bp_y1 - g->bp_a2 * s->bp_y2 + g->bp_b0 * v + g->bp_b2 * s->bp_x2 + vref;
			s->bp_x2 = s->bp_x1;
			s->bp_x1 = v;
			s->bp_y2 = s->bp_y1;
			if (vo > 5.0f - 1.5f) vo = 5.0f - 1.5f;
			if (vo < 0) vo = 0;
			s->bp_y1 = vo - vref;
			v157 = vo;
		}
		// ---- FIRE
		float v182;
		{
			const float v171 = s->fire ? TTL_OUT : 0, v172 = s->fire ? 0 : TTL_OUT;
			s->fire_rc += (v172 - s->fire_rc) * g->e_fire_rc;
			const float v178 = (float)(1.0 / (1.0 / R46 + 1.0 / R48)) *
			                   (noise * TTL_OUT * (float)(1.0 / R46) + s->fire_rc * (float)(1.0 / R48));
			const float v181 = astable(&s->fire555, v178, g->fire_t_chg, g->fire_t_dis, g->fire_e_chg, g->fire_e_dis,
			                           1.0f, 0, dt, g->inv_dt);
			float u = v171 - 0.7f;
			if (u < 0) u = 0;
			float diff = u - s->fire_vcap;
			if (v181 != 0) {
				if (diff < 0)
					diff *= g->e_fire;
				s->fire_vcap += diff;
				v182 = s->fire_vcap;
			} else {
				if (diff > 0)
					s->fire_vcap = u;
				v182 = 0;
			}
		}
		// ---- the mix
		float v;
		const int vs = s->vol[0] | s->vol[1] << 1;
		if (g->type == GALSND_GALAXIAN) {
			// galaxian_mixerpre_desc (VOL1 / VOL2 switch R49 / R52 in), then galaxian_mixer_desc (rF R91)
			static const float inv_g0[4] = {
				(float)(1.0 / (1.0 / R51 + 1.0 / R50 + 1.0 / R34)),
				(float)(1.0 / (1.0 / R51 + 1.0 / R50 + 1.0 / R34 + 1.0 / R49)),
				(float)(1.0 / (1.0 / R51 + 1.0 / R50 + 1.0 / R34 + 1.0 / R52)),
				(float)(1.0 / (1.0 / R51 + 1.0 / R50 + 1.0 / R34 + 1.0 / R49 + 1.0 / R52)) };
			float i = b0 * (float)(1.0 / R51) + b2 * (float)(1.0 / R50) + v120 * (float)(1.0 / R34);
			if (vs & 1) i += b2 * (float)(1.0 / R49);
			if (vs & 2) i += b3 * (float)(1.0 / R52);
			const float v279 = i * inv_g0[vs];
			s->mix_c += (v182 - s->mix_c) * g->e_mix_c;
			const float vf = v182 - s->mix_c;
			const float i2 = v279 * (float)(1.0 / R34) + v157 * (float)(1.0 / R40) + vf * (float)(1.0 / R43);
			v = i2 * (float)(1.0 / (1.0 / R34 + 1.0 / R40 + 1.0 / R43 + 1.0 / R91));
		} else {
			// mooncrst_mixer_desc: the seven inputs in one resistor network
#define MC_G (1.0 / R51 + 1.0 / R50 + 1.0 / MCRST_R34 + 1.0 / R40 + 1.0 / R43)
			static const float inv_g0[4] = { (float)(1.0 / MC_G), (float)(1.0 / (MC_G + 1.0 / R49)),
			                                 (float)(1.0 / (MC_G + 1.0 / R52)), (float)(1.0 / (MC_G + 1.0 / R49 + 1.0 / R52)) };
#undef MC_G
			float i = b0 * (float)(1.0 / R51) + b2 * (float)(1.0 / R50) + v120 * (float)(1.0 / MCRST_R34) +
			          v157 * (float)(1.0 / R40);
			if (vs & 1) i += b2 * (float)(1.0 / R49);
			if (vs & 2) i += b3 * (float)(1.0 / R52);
			s->mix_c += (v182 - s->mix_c) * g->e_mix_c;
			i += (v182 - s->mix_c) * (float)(1.0 / R43);
			v = i * inv_g0[vs];
		}
		s->amp_c += (v - s->amp_c) * g->e_amp;   // (cAmp: the high pass)
		v -= s->amp_c;
		out[k] += (int32_t)(v * g->out_gain);
	}
}
