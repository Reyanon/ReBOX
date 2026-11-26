/*  The Galaxian / Moon Cresta discrete sound (the background 555s and their LFO, the pitch counter, the noise, the
    hit and the fire circuits), after MAME src/mame/galaxian/galaxian_a.cpp (BSD-3-Clause, copyright-holders:
    Couriersud) and the discrete nodes it uses (src/devices/sound/disc_*.hxx, BSD-3-Clause, copyright-holders: K.Wilkins,
    Derrick Renaud, Couriersud).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_GALAXIAN_A_H
#define RBX_GALAXIAN_A_H

#include <stdint.h>

enum { GALSND_GALAXIAN, GALSND_MOONCRST, GALSND_BONGO };

typedef struct {
	float vcap;
	uint8_t ff;
	float out;   // (the last output: kept when the control voltage is below 0.25 V)
} galsnd_555_t;

typedef struct {
	// ---- inputs (the latches)
	uint8_t lfo_val, fs[3], hit, fire, vol[2], pitch;
	// ---- state
	float lfo_vcap;         // (NODE_105, the 555 CC)
	galsnd_555_t bck[3];       // (NODE_115..117)
	float bck_cf;           // (NODE_120's output capacitor)
	int32_t note_c1, note_c2;
	uint32_t lfsr;
	uint8_t lfsr_fb, noise;
	uint32_t lfsr_acc, sq_acc;   // (16.16 clocks of the LFSR and of the 2V square wave)
	uint32_t note_frac;        // (16.16, the pitch counter's clock)
	float hit_vcap;         // (NODE_155)
	float bp_x1, bp_x2, bp_y1, bp_y2;   // (NODE_157)
	float fire_rc;          // (NODE_173)
	galsnd_555_t fire555;      // (NODE_181)
	float fire_vcap;        // (NODE_182)
	float mix_c;            // (the mixer's capacitor on the fire input)
	float amp_c;            // (cAmp)
} galsnd_state_t;

typedef struct {
	galsnd_state_t s;          // (saved)
	// ---- configuration
	int type;
	float dt, inv_dt;
	int note_clocks;        // (the pitch counter's clocks a sample, 16.16)
	uint32_t lfsr_step, sq_step;
	float dac_v[16];
	float bck_t_chg[3], bck_t_dis[3], bck_e_chg[3], bck_e_dis[3];
	float fire_t_chg, fire_t_dis, fire_e_chg, fire_e_dis;
	float e_bck_cf, e_hit, e_fire_rc, e_fire, e_mix_c, e_amp;
	float bp_a1, bp_a2, bp_b0, bp_b2, bp_rtot;
	float out_gain;
} galsnd_t;

#define GALSND_STATE_SIZE ((uint32_t)sizeof(galsnd_state_t))

void galsnd_init(galsnd_t *g, int type, double rate, float gain);
void galsnd_reset(galsnd_t *g);
// MAME's write handlers: sound_w (offset 0-7 of the 6800 latch), lfo_freq_w (offset 0-3), pitch_w
void galsnd_sound_w(galsnd_t *g, int offset, uint8_t data);
void galsnd_lfo_w(galsnd_t *g, int offset, uint8_t data);
void galsnd_pitch_w(galsnd_t *g, uint8_t data);
// n samples added to out[]
void galsnd_render(galsnd_t *g, int32_t *out, int n);

#endif
