/*  AY-3-8910 / YM2149 PSG, ported from MAME src/devices/sound/ay8910.cpp
    (BSD-3-Clause, copyright-holders: Couriersud; based on code by Ville Hallik, Michael Cuddy,
    Tatsuyuki Satoh, Fabrice Frances, Nicola Salmoria).
    Tone / noise / envelope stepping, register semantics and the resistor output models follow MAME;
    the single-output 3D table of MAME (8*32*32*32 entries) is evaluated on demand and cached by index.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "ay8910.h"

#include <string.h>

enum {
	AY_AFINE, AY_ACOARSE, AY_BFINE, AY_BCOARSE, AY_CFINE, AY_CCOARSE, AY_NOISEPER, AY_ENABLE,
	AY_AVOL, AY_BVOL, AY_CVOL, AY_EAFINE, AY_EACOARSE, AY_EASHAPE, AY_PORTA, AY_PORTB
};

typedef struct { double r_up, r_down; int res_count; double res[32]; } ay_ym_param;

static const ay_ym_param ym2149_param = {
	630, 801, 16,
	{ 73770, 37586, 27458, 21451, 15864, 12371, 8922,  6796,
	  4763,  3521,  2403,  1737,  1123,   762,  438,   251 },
};

static const ay_ym_param ym2149_param_env = {
	630, 801, 32,
	{ 103350, 73770, 52657, 37586, 32125, 27458, 24269, 21451,
	  18447, 15864, 14009, 12371, 10506,  8922,  7787,  6796,
	  5689,  4763,  4095,  3521,  2909,  2403,  2043,  1737,
	  1397,  1123,   925,   762,   578,   438,   332,   251 },
};

// RL = 2000, based on Matthew Westcott's measurements (see MAME)
static const ay_ym_param ay8910_param = {
	800000, 8000000, 16,
	{ 15950, 15350, 15090, 14760, 14275, 13620, 12890, 11370,
	  10600,  8590,  7190,  5985,  4820,  3945,  3017,  2345 },
};

#define RES_LOAD 1000.0   // MAME default m_res_load

static void build_single_table(double rl, const ay_ym_param *par, int normalize, int32_t *tab, int zero_is_off)
{
	double temp[32], min = 10.0, max = 0.0;
	for (int j = 0; j < par->res_count; j++) {
		double rt = 1.0 / par->r_down + 1.0 / rl;
		double rw = 1.0 / par->res[j];
		rt += 1.0 / par->res[j];
		if (!(zero_is_off && j == 0)) {
			rw += 1.0 / par->r_up;
			rt += 1.0 / par->r_up;
		}
		temp[j] = rw / rt;
		if (temp[j] < min) min = temp[j];
		if (temp[j] > max) max = temp[j];
	}
	for (int j = 0; j < 32; j++) {
		double v = 0;
		if (j < par->res_count)
			v = normalize ? (((temp[j] - min) / (max - min)) - 0.25) * 0.5 : temp[j];
		tab[j] = (int32_t)(v * 32767.0);
	}
}

// MAME build_3D_table, one entry
static double mix3d_raw(const ay8910_t *ay, uint32_t indx)
{
	int e = (int)(indx >> 15);
	int j[3] = { (int)(indx & 31), (int)((indx >> 5) & 31), (int)((indx >> 10) & 31) };
	double n;
	if (ay->zero_is_off) {
		n = 0;
		for (int c = 0; c < 3; c++)
			n += (j[c] != 0 || (e & (1 << c))) ? 1 : 0;
	} else
		n = 3.0;
	double rt = n / ay->r_up + 3.0 / ay->r_down + 1.0 / ay->rl;
	double rw = n / ay->r_up;
	for (int c = 0; c < 3; c++) {
		double r = (e & (1 << c)) ? ay->res_env_inv[j[c]] : ay->res_inv[j[c]];
		rw += r;
		rt += r;
	}
	return rw / rt;
}

static void build_mixer(ay8910_t *ay)
{
	int normalize = (ay->flags & AY8910_LEGACY_OUTPUT) != 0;
	const ay_ym_param *par = ay->type == AY8910_TYPE_AY8910 ? &ay8910_param : &ym2149_param;
	const ay_ym_param *par_env = ay->type == AY8910_TYPE_AY8910 ? &ay8910_param : &ym2149_param_env;
	if (!ay->single) {
		for (int c = 0; c < 3; c++) {
			build_single_table(ay->res_load[c], par, normalize, ay->vol_table[c], ay->zero_is_off);
			build_single_table(ay->res_load[c], par_env, normalize, ay->env_table[c], 0);
		}
		return;
	}
	ay->r_up = par->r_up;
	ay->r_down = par->r_down;
	ay->rl = ay->res_load[0];
	for (int i = 0; i < 32; i++) {
		// entries past res_count are 0 in MAME's tables (never indexed by valid states)
		ay->res_inv[i] = i < par->res_count ? 1.0 / par->res[i] : 0;
		ay->res_env_inv[i] = i < par_env->res_count ? 1.0 / par_env->res[i] : 0;
	}
	ay->norm_min = 0;
	ay->norm_scale = 1.0;
	if (normalize) {
		// the previous implementation added all three channels up: factor 3 (MAME)
		double min = 10.0, max = 0.0;
		for (int e = 0; e < 8; e++) {
			int c1 = (e & 1) ? par_env->res_count : par->res_count;
			int c2 = (e & 2) ? par_env->res_count : par->res_count;
			int c3 = (e & 4) ? par_env->res_count : par->res_count;
			for (int j1 = 0; j1 < c1; j1++)
				for (int j2 = 0; j2 < c2; j2++)
					for (int j3 = 0; j3 < c3; j3++) {
						double t = mix3d_raw(ay, (uint32_t)(e << 15 | j3 << 10 | j2 << 5 | j1));
						if (t < min) min = t;
						if (t > max) max = t;
					}
		}
		ay->norm_min = min;
		ay->norm_scale = 3.0 / (max - min);
	}
	ay->last_index = 0xffffffffu;
}

static int32_t mix3d(ay8910_t *ay, uint32_t indx)
{
	if (indx != ay->last_index) {
		ay->last_index = indx;
		ay->last_value = (int32_t)((mix3d_raw(ay, indx) - ay->norm_min) * ay->norm_scale * 32767.0);
	}
	return ay->last_value;
}

static void env_set_shape(ay8910_env_t *e, uint8_t shape, uint8_t mask)
{
	e->attack = (shape & 0x04) ? mask : 0x00;
	if ((shape & 0x08) == 0) {
		// Continue = 0: map to the equivalent shape with Continue = 1
		e->hold = 1;
		e->alternate = e->attack;
	} else {
		e->hold = shape & 0x01;
		e->alternate = shape & 0x02;
	}
	e->step = (int8_t)mask;
	e->holding = 0;
	e->volume = (uint32_t)(e->step ^ e->attack);
}

static void write_reg(ay8910_t *ay, int r, uint8_t v)
{
	ay->regs[r] = v;
	switch (r) {
	case AY_AFINE: case AY_ACOARSE:
		ay->tone[0].period = ay->regs[AY_AFINE] | (ay->regs[AY_ACOARSE] & 0x0f) << 8;
		break;
	case AY_BFINE: case AY_BCOARSE:
		ay->tone[1].period = ay->regs[AY_BFINE] | (ay->regs[AY_BCOARSE] & 0x0f) << 8;
		break;
	case AY_CFINE: case AY_CCOARSE:
		ay->tone[2].period = ay->regs[AY_CFINE] | (ay->regs[AY_CCOARSE] & 0x0f) << 8;
		break;
	case AY_AVOL: ay->tone[0].volume = v; break;
	case AY_BVOL: ay->tone[1].volume = v; break;
	case AY_CVOL: ay->tone[2].volume = v; break;
	case AY_EAFINE: case AY_EACOARSE:
		ay->env.period = ay->regs[AY_EAFINE] | ay->regs[AY_EACOARSE] << 8;
		break;
	case AY_ENABLE:
		// output is high-impedance while a port is set to input
		if ((v & 0x40) != (ay->last_enable & 0x40) && ay->port_a_write)
			ay->port_a_write(ay->user, (v & 0x40) ? ay->regs[AY_PORTA] : 0xff);
		if ((v & 0x80) != (ay->last_enable & 0x80) && ay->port_b_write)
			ay->port_b_write(ay->user, (v & 0x80) ? ay->regs[AY_PORTB] : 0xff);
		ay->last_enable = v;
		break;
	case AY_EASHAPE:
		env_set_shape(&ay->env, v, ay->env_step_mask);
		break;
	case AY_PORTA:
		if ((ay->regs[AY_ENABLE] & 0x40) && ay->port_a_write)
			ay->port_a_write(ay->user, v);
		break;
	case AY_PORTB:
		if ((ay->regs[AY_ENABLE] & 0x80) && ay->port_b_write)
			ay->port_b_write(ay->user, v);
		break;
	default:
		break;
	}
}

void ay8910_init(ay8910_t *ay, int type, uint32_t clock, int flags, int gain, int divider, int dc_block)
{
	memset(ay, 0, sizeof(*ay));   // (callbacks are set by the caller after init)
	ay->type = type;
	ay->flags = flags;
	ay->clock = clock;
	ay->gain = gain;
	ay->divider = divider < 1 ? 1 : (uint32_t)divider;
	ay->dc_block = dc_block;
	ay->single = (flags & AY8910_SINGLE_OUTPUT) != 0;
	ay->clk_per_sample = (type == AY8910_TYPE_YM2149 && (flags & YM2149_PIN26_LOW)) ? 16 : 8;
	if (type == AY8910_TYPE_AY8910) {
		ay->env_step_mask = 0x0f;
		ay->step = 2;
		ay->zero_is_off = 1;
	} else {
		ay->env_step_mask = 0x1f;
		ay->step = 1;
		ay->zero_is_off = 0;
	}
	ay->res_load[0] = ay->res_load[1] = ay->res_load[2] = RES_LOAD;
	build_mixer(ay);
	ay8910_reset(ay);
}

void ay8910_reset(ay8910_t *ay)
{
	ay->active = 0;
	ay->register_latch = 0;
	ay->rng = 1;
	ay->noise_out = 0;
	memset(ay->tone, 0, sizeof(ay->tone));
	memset(&ay->env, 0, sizeof(ay->env));
	ay->count_noise = 0;
	ay->prescale_noise = 0;
	ay->last_enable = 0xc0;   // force a write
	for (int i = 0; i < AY_PORTA; i++)
		write_reg(ay, i, 0);
	ay->frac = 0;
	ay->div_count = 0;
	ay->div_acc = 0;
	ay->last_index = 0xffffffffu;
}

uint32_t ay8910_rate(const ay8910_t *ay)
{
	return ay->clock / ay->clk_per_sample / ay->divider;
}

void ay8910_address_w(ay8910_t *ay, uint8_t data)
{
	ay->active = (data >> 4) == 0;   // mask programmed 4-bit code
	if (ay->active)
		ay->register_latch = data & 0x0f;
}

void ay8910_data_w(ay8910_t *ay, uint8_t data)
{
	if (ay->active)
		write_reg(ay, ay->register_latch, data);
}

void ay8910_write(ay8910_t *ay, int offset, uint8_t data)
{
	if (offset & 1)
		ay8910_data_w(ay, data);
	else
		ay8910_address_w(ay, data);
}

uint8_t ay8910_data_r(ay8910_t *ay)
{
	int r = ay->register_latch;
	if (!ay->active)
		return 0xff;   // high impedance
	// even with the port set as output the external data is returned (MAME, kidniki)
	if (r == AY_PORTA && ay->port_a_read)
		ay->regs[AY_PORTA] = ay->port_a_read(ay->user);
	else if (r == AY_PORTB && ay->port_b_read)
		ay->regs[AY_PORTB] = ay->port_b_read(ay->user);
	if (ay->type == AY8910_TYPE_AY8910) {
		// inaccessible bits read back as 0
		static const uint8_t mask[16] = {
			0xff, 0x0f, 0xff, 0x0f, 0xff, 0x0f, 0x1f, 0xff, 0x1f, 0x1f, 0x1f, 0xff, 0xff, 0x0f, 0xff, 0xff
		};
		return ay->regs[r] & mask[r];
	}
	return ay->regs[r];
}

// one sample at the native rate (MAME sound_stream_update loop body), 1.0 = 32767; ch (separate outputs): the
// channels' levels
static int32_t sample_ch(ay8910_t *ay, int32_t *ch)
{
	uint8_t enable = ay->regs[AY_ENABLE];
	for (int c = 0; c < 3; c++) {
		ay8910_tone_t *t = &ay->tone[c];
		int32_t period = t->period ? (int32_t)t->period : 1;
		t->count++;
		while (t->count >= period) {
			t->duty_cycle = (t->duty_cycle - 1) & 0x1f;
			t->output = t->duty_cycle & 1;
			t->count -= period;
		}
	}
	if (++ay->count_noise >= (ay->regs[AY_NOISEPER] & 0x1f)) {
		// toggle the prescaler output; the 17-bit LFSR input is bit0 ^ bit3
		ay->count_noise = 0;
		ay->prescale_noise ^= 1;
		if (!ay->prescale_noise)
			ay->rng = (ay->rng >> 1) | (((ay->rng ^ (ay->rng >> 3)) & 1) << 16);
	}
	uint8_t noise = ay->rng & 1;
	for (int c = 0; c < 3; c++)
		ay->vol_enabled[c] = (ay->tone[c].output | ((enable >> c) & 1)) & (noise | ((enable >> (3 + c)) & 1));
	// envelope
	ay8910_env_t *e = &ay->env;
	if (e->holding == 0) {
		uint32_t period = e->period * ay->step;
		if ((uint32_t)(++e->count) >= period) {
			e->count = 0;
			e->step--;
			if (e->step < 0) {
				if (e->hold) {
					if (e->alternate)
						e->attack ^= ay->env_step_mask;
					e->holding = 1;
					e->step = 0;
				} else {
					// if the envelope has looped an odd number of times (usually 1), invert the output
					if (e->alternate && (e->step & (ay->env_step_mask + 1)))
						e->attack ^= ay->env_step_mask;
					e->step &= ay->env_step_mask;
				}
			}
		}
	}
	e->volume = (uint32_t)(e->step ^ e->attack);
	if (ay->single) {
		uint32_t indx = 0;
		for (int c = 0; c < 3; c++) {
			const ay8910_tone_t *t = &ay->tone[c];
			if (t->volume & 0x10)
				indx |= (1u << (c + 15)) | (ay->vol_enabled[c] ? e->volume << (c * 5) : 0);
			else
				indx |= ay->vol_enabled[c] ? (uint32_t)(t->volume & 0x0f) << (c * 5) : 0;
		}
		return mix3d(ay, indx);
	}
	int32_t s = 0;
	for (int c = 0; c < 3; c++) {
		const ay8910_tone_t *t = &ay->tone[c];
		int32_t v;
		if (t->volume & 0x10)
			v = ay->env_table[c][ay->vol_enabled[c] ? e->volume : 0];
		else
			v = ay->vol_table[c][ay->vol_enabled[c] ? (t->volume & 0x0f) : 0];
		if (ch)
			ch[c] = v;
		s += v;
	}
	return s;
}
static int32_t sample(ay8910_t *ay) { return sample_ch(ay, NULL); }

void ay8910_set_res_loads(ay8910_t *ay, double r0, double r1, double r2)
{
	ay->res_load[0] = r0;
	ay->res_load[1] = r1;
	ay->res_load[2] = r2;
	build_mixer(ay);
}

int ay8910_render_ch(ay8910_t *ay, uint32_t clocks, int16_t *out, int max)
{
	int n = 0;
	uint32_t total = ay->frac + clocks;
	int32_t ch[3] = { 0, 0, 0 };
	while (total >= ay->clk_per_sample) {
		total -= ay->clk_per_sample;
		sample_ch(ay, ch);
		if (n < max) {
			out[3 * n] = (int16_t)ch[0];
			out[3 * n + 1] = (int16_t)ch[1];
			out[3 * n + 2] = (int16_t)ch[2];
			n++;
		}
	}
	ay->frac = total;
	return n;
}

int ay8910_render(ay8910_t *ay, uint32_t clocks, int32_t *out, int max)
{
	int n = 0;
	uint32_t total = ay->frac + clocks;
	while (total >= ay->clk_per_sample) {
		total -= ay->clk_per_sample;
		ay->div_acc += sample(ay);
		if (++ay->div_count < ay->divider)
			continue;
		int32_t v = ay->div_acc / (int32_t)ay->divider;
		ay->div_acc = 0;
		ay->div_count = 0;
		v = (int32_t)(((int64_t)v * ay->gain) >> 8);
		if (ay->dc_block) {
			// one-pole high pass (time constant 2048 output samples), the level kept x256
			int32_t x = v << 8;
			ay->dc += (x - ay->dc) >> 11;
			v = (x - ay->dc) >> 8;
		}
		if (n < max)
			out[n++] += v;
	}
	ay->frac = total;
	return n;
}

uint32_t ay8910_state_size(void)
{
	return (uint32_t)AY8910_STATE_END;
}
