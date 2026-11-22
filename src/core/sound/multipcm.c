/*  Yamaha YMW-258-F 'GEW8' (Sega 315-5560, MultiPCM). Ported from MAME 0.289 src/devices/sound/multipcm.cpp and
    gew.cpp (BSD-3-Clause, copyright-holders: Miguel Angel Horna): MAME's tables (float as MAME) and stream update.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "multipcm.h"

#include <math.h>
#include <string.h>

#define TL_SHIFT 12
#define EG_SHIFT 16
#define LFO_SHIFT 8
#define CLOCK_DIVIDER 224

enum { ST_ATTACK, ST_DECAY1, ST_DECAY2, ST_RELEASE };

// ---- the tables (gew_pcm_device::device_start, lfo_init: the same for every chip of a clock) ----
static uint32_t s_tab_clock;
static uint32_t s_attack_step[0x40], s_decay_release_step[0x40], s_freq_step_table[0x400];
static int32_t s_left_pan_table[0x800], s_right_pan_table[0x800], s_linear_to_exp_volume[0x400];
static int32_t s_total_level_steps[2];
static int32_t s_pitch_table[256], s_amplitude_table[256];
static int32_t s_pitch_scale_tables[8][256], s_amplitude_scale_tables[8][256];

static const double BASE_TIMES[64] = {
	0,          0,          0,          0,
	6222.95,    4978.37,    4148.66,    3556.01,
	3111.47,    2489.21,    2074.33,    1778.00,
	1555.74,    1244.63,    1037.19,    889.02,
	777.87,     622.31,     518.59,     444.54,
	388.93,     311.16,     259.32,     222.27,
	194.47,     155.60,     129.66,     111.16,
	97.23,      77.82,      64.85,      55.60,
	48.62,      38.91,      32.43,      27.80,
	24.31,      19.46,      16.24,      13.92,
	12.15,      9.75,       8.12,       6.98,
	6.08,       4.90,       4.08,       3.49,
	3.04,       2.49,       2.13,       1.90,
	1.72,       1.41,       1.18,       1.04,
	0.91,       0.73,       0.59,       0.50,
	0.45,       0.45,       0.45,       0.45
};
static const float LFO_FREQ[8] = { 0.168f, 2.019f, 3.196f, 4.206f, 5.215f, 5.888f, 6.224f, 7.066f };
static const float PHASE_SCALE_LIMIT[8] = { 0.0f, 3.378f, 5.065f, 6.750f, 10.114f, 20.170f, 40.180f, 79.307f };
static const float AMPLITUDE_SCALE_LIMIT[8] = { 0.0f, 0.4f, 0.8f, 1.5f, 3.0f, 6.0f, 12.0f, 24.0f };
static const int32_t VALUE_TO_CHANNEL[32] = {
	0, 1, 2, 3, 4, 5, 6, -1,
	7, 8, 9, 10, 11, 12, 13, -1,
	14, 15, 16, 17, 18, 19, 20, -1,
	21, 22, 23, 24, 25, 26, 27, -1,
};

static uint32_t value_to_fixed(uint32_t bits, float value)
{
	const float float_shift = (float)(1 << bits);
	return (uint32_t)(float_shift * value);
}

static void tables_init(uint32_t clock, float rate)
{
	if (s_tab_clock == clock)
		return;
	s_tab_clock = clock;
	for (int32_t level = 0; level < 0x80; ++level) {
		const float vol_db = (float)level * (-24.0f) / 64.0f;
		const float total_level = powf(10.0f, vol_db / 20.0f) / 4.0f;
		for (int32_t pan = 0; pan < 0x10; ++pan) {
			float pan_left, pan_right;
			if (pan == 0x8)
				pan_left = 0.0, pan_right = 0.0;
			else if (pan == 0x0)
				pan_left = 1.0, pan_right = 1.0;
			else if (pan & 0x8) {
				pan_left = 1.0;
				const int32_t inverted_pan = 0x10 - pan;
				const float pan_vol_db = (float)inverted_pan * (-12.0f) / 4.0f;
				pan_right = (float)pow(10.0f, pan_vol_db / 20.0f);
				if ((inverted_pan & 0x7) == 7)
					pan_right = 0.0;
			} else {
				pan_right = 1.0;
				const float pan_vol_db = (float)pan * (-12.0f) / 4.0f;
				pan_left = (float)pow(10.0f, pan_vol_db / 20.0f);
				if ((pan & 0x7) == 7)
					pan_left = 0.0;
			}
			s_left_pan_table[(pan << 7) | level] = (int32_t)value_to_fixed(TL_SHIFT, pan_left * total_level);
			s_right_pan_table[(pan << 7) | level] = (int32_t)value_to_fixed(TL_SHIFT, pan_right * total_level);
		}
	}
	for (int32_t i = 0; i < 0x400; ++i) {
		const float fcent = rate * (1024.0f + (float)i) / 1024.0f;
		s_freq_step_table[i] = value_to_fixed(TL_SHIFT, fcent);
	}
	// envelope_generator_init(BASE_TIMES, 14.32833)
	for (int32_t i = 4; i < 0x40; ++i) {
		s_attack_step[i] = (uint32_t)((float)(0x400 << EG_SHIFT) / (float)(BASE_TIMES[i] * 44100.0 / 1000.0));
		s_decay_release_step[i] = (uint32_t)((float)(0x400 << EG_SHIFT) / (float)(BASE_TIMES[i] * 14.32833 * 44100.0 / 1000.0));
	}
	s_attack_step[0] = s_attack_step[1] = s_attack_step[2] = s_attack_step[3] = 0;
	s_attack_step[0x3f] = 0x400 << EG_SHIFT;
	s_decay_release_step[0] = s_decay_release_step[1] = s_decay_release_step[2] = s_decay_release_step[3] = 0;
	s_total_level_steps[0] = (int32_t)(-(float)(0x80 << TL_SHIFT) / (78.2f * 44100.0f / 1000.0f));
	s_total_level_steps[1] = (int32_t)((float)(0x80 << TL_SHIFT) / (78.2f * 2 * 44100.0f / 1000.0f));
	for (int32_t i = 0; i < 0x400; ++i) {
		const float db = -(96.0f - (96.0f * (float)i / (float)0x400));
		const float exp_volume = powf(10.0f, db / 20.0f);
		s_linear_to_exp_volume[i] = (int32_t)value_to_fixed(TL_SHIFT, exp_volume);
	}
	// lfo_init
	for (int32_t i = 0; i < 256; ++i) {
		if (i < 64) s_pitch_table[i] = i * 2 + 128;
		else if (i < 128) s_pitch_table[i] = 383 - i * 2;
		else if (i < 192) s_pitch_table[i] = 384 - i * 2;
		else s_pitch_table[i] = i * 2 - 383;
		s_amplitude_table[i] = i < 128 ? 255 - (i * 2) : (i * 2) - 256;
	}
	for (int32_t table = 0; table < 8; ++table) {
		float limit = PHASE_SCALE_LIMIT[table];
		for (int32_t i = -128; i < 128; ++i) {
			const float value = (limit * (float)i) / 128.0f;
			const float converted = powf(2.0f, value / 1200.0f);
			s_pitch_scale_tables[table][i + 128] = (int32_t)value_to_fixed(LFO_SHIFT, converted);
		}
		limit = -AMPLITUDE_SCALE_LIMIT[table];
		for (int32_t i = 0; i < 256; ++i) {
			const float value = (limit * (float)i) / 256.0f;
			const float converted = powf(10.0f, value / 20.0f);
			s_amplitude_scale_tables[table][i] = (int32_t)value_to_fixed(LFO_SHIFT, converted);
		}
	}
}

// ---- the ROM space (multipcm_map) ----
static inline uint8_t read_byte(const multipcm_t *c, uint32_t a)
{
	a &= 0x3fffff;
	uint32_t o;
	if (a < 0x100000) o = a;
	else if (a < 0x180000) o = c->bank_lo + (a - 0x100000);
	else if (a < 0x200000) o = c->bank_hi + (a - 0x180000);
	else return 0;
	return o < c->rom_size ? c->rom[o] : 0;
}

// ---- envelopes ----
static void envelope_generator_calc(multipcm_slot_t *s);
static void retrigger_sample(multipcm_slot_t *s)
{
	s->offset = 0;
	s->prev_sample = 0;
	s->total_level = s->dest_total_level << TL_SHIFT;
	envelope_generator_calc(s);
	s->eg_state = ST_ATTACK;
	s->eg_volume = 0;
}
static void update_step(const multipcm_t *c, multipcm_slot_t *s)
{
	const uint8_t oct = (uint8_t)((s->octave - 1) & 0xf);
	uint32_t pitch = s_freq_step_table[s->pitch];
	if (oct & 0x8)
		pitch >>= (16 - oct);
	else
		pitch <<= oct;
	s->step = (uint32_t)(pitch / c->rate);
}
static int32_t envelope_generator_update(multipcm_slot_t *s)
{
	switch (s->eg_state) {
	case ST_ATTACK:
		s->eg_volume += s->eg_attack_rate;
		if (s->eg_volume >= (0x3ff << EG_SHIFT)) {
			s->eg_state = ST_DECAY1;
			if (s->eg_decay1_rate >= (0x400 << EG_SHIFT))
				s->eg_state = ST_DECAY2;
			s->eg_volume = 0x3ff << EG_SHIFT;
		}
		break;
	case ST_DECAY1:
		s->eg_volume -= s->eg_decay1_rate;
		if (s->eg_volume <= 0)
			s->eg_volume = 0;
		if (s->eg_volume >> (EG_SHIFT + 6) <= s->eg_decay_level)
			s->eg_state = ST_DECAY2;
		break;
	case ST_DECAY2:
		s->eg_volume -= s->eg_decay2_rate;
		if (s->eg_volume <= 0)
			s->eg_volume = 0;
		break;
	case ST_RELEASE:
		s->eg_volume -= s->eg_release_rate;
		if (s->eg_volume <= 0) {
			s->eg_volume = 0;
			s->playing = 0;
		}
		break;
	default:
		return 1 << TL_SHIFT;
	}
	if (s->eg_reverb && s->eg_state != ST_ATTACK && (s->eg_volume >> EG_SHIFT) <= 0x300) {
		s->eg_decay1_rate = (int32_t)s_decay_release_step[17];
		s->eg_decay2_rate = (int32_t)s_decay_release_step[17];
		s->eg_release_rate = (int32_t)s_decay_release_step[17];
	}
	return s_linear_to_exp_volume[s->eg_volume >> EG_SHIFT];
}
static uint32_t get_rate(const uint32_t *steps, int32_t rate, uint32_t val)
{
	if (val == 0)
		return steps[0];
	if (val == 0xf)
		return steps[0x3f];
	int r = 4 * (int)val + rate;
	r = r < 0 ? 0 : r > 0x3f ? 0x3f : r;
	return steps[r];
}
static void envelope_generator_calc(multipcm_slot_t *s)
{
	int32_t octave = s->octave;
	if (octave & 8)
		octave = octave - 16;
	int32_t rate;
	if (s->sample.key_rate_scale != 0xf)
		rate = (octave + s->sample.key_rate_scale) * 2 + ((s->pitch >> 9) & 1);
	else
		rate = 0;
	s->eg_attack_rate = (int32_t)get_rate(s_attack_step, rate, s->sample.attack_reg);
	s->eg_decay1_rate = (int32_t)get_rate(s_decay_release_step, rate, s->sample.decay1_reg);
	s->eg_decay2_rate = (int32_t)get_rate(s_decay_release_step, rate, s->sample.decay2_reg);
	s->eg_release_rate = (int32_t)get_rate(s_decay_release_step, rate, s->sample.release_reg);
	s->eg_decay_level = 0xf - s->sample.decay_level;
	s->eg_reverb = 0;
}

// ---- LFOs ----
static inline int32_t lfo_step(multipcm_lfo_t *lfo)
{
	lfo->phase = (uint16_t)(lfo->phase + lfo->phase_step);
	int32_t p = (lfo->amplitude ? s_amplitude_table : s_pitch_table)[(lfo->phase >> LFO_SHIFT) & 0xff];
	p = (lfo->amplitude ? s_amplitude_scale_tables : s_pitch_scale_tables)[lfo->scale][p];
	return p << (TL_SHIFT - LFO_SHIFT);
}
static void lfo_compute_step(const multipcm_t *c, multipcm_lfo_t *lfo, uint32_t lfo_frequency, uint32_t lfo_scale, int amplitude_lfo)
{
	const float step = (float)LFO_FREQ[lfo_frequency] * 256.0f / (float)c->rate;
	lfo->phase_step = (uint32_t)((float)(1 << LFO_SHIFT) * step);
	lfo->amplitude = (uint8_t)(amplitude_lfo != 0);
	lfo->scale = (uint8_t)lfo_scale;
}

// ---- registers (multipcm_device) ----
static void init_sample(const multipcm_t *c, multipcm_sample_t *sm, uint32_t index)
{
	const uint32_t address = index * 12;
	sm->start = ((uint32_t)read_byte(c, address) << 16) | ((uint32_t)read_byte(c, address + 1) << 8) | read_byte(c, address + 2);
	sm->format = (uint8_t)((sm->start >> 20) & 0xfe);
	sm->start &= 0x3fffff;
	sm->loop = ((uint32_t)read_byte(c, address + 3) << 8) | read_byte(c, address + 4);
	sm->end = 0x10000 - (((uint32_t)read_byte(c, address + 5) << 8) | read_byte(c, address + 6));
	sm->attack_reg = (read_byte(c, address + 8) >> 4) & 0xf;
	sm->decay1_reg = read_byte(c, address + 8) & 0xf;
	sm->decay2_reg = read_byte(c, address + 9) & 0xf;
	sm->decay_level = (read_byte(c, address + 9) >> 4) & 0xf;
	sm->release_reg = read_byte(c, address + 10) & 0xf;
	sm->key_rate_scale = (read_byte(c, address + 10) >> 4) & 0xf;
	sm->lfo_vibrato_reg = read_byte(c, address + 7);
	sm->lfo_amplitude_reg = read_byte(c, address + 11) & 0xf;
}

static void write_slot(multipcm_t *c, multipcm_slot_t *s, int reg, uint8_t data)
{
	s->regs[reg] = data;
	switch (reg) {
	case 0:
		s->pan = (data >> 4) & 0xf;
		break;
	case 1:
		init_sample(c, &s->sample, s->regs[1] | ((s->regs[2] & 1) << 8));
		write_slot(c, s, 6, s->sample.lfo_vibrato_reg);
		write_slot(c, s, 7, s->sample.lfo_amplitude_reg);
		if (s->playing)
			retrigger_sample(s);
		break;
	case 2:
	case 3:
		s->octave = s->regs[3] >> 4;
		s->pitch = (uint16_t)(((s->regs[3] & 0xf) << 6) | (s->regs[2] >> 2));
		update_step(c, s);
		break;
	case 4:
		if (data & 0x80) {
			s->playing = 1;
			retrigger_sample(s);
		} else if (s->playing) {
			if (s->sample.release_reg != 0xf)
				s->eg_state = ST_RELEASE;
			else
				s->playing = 0;
		}
		break;
	case 5:
		s->dest_total_level = (data >> 1) & 0x7f;
		if (!(data & 1)) {
			if ((s->total_level >> TL_SHIFT) > s->dest_total_level)
				s->total_level_step = s_total_level_steps[0];
			else
				s->total_level_step = s_total_level_steps[1];
		} else
			s->total_level = s->dest_total_level << TL_SHIFT;
		break;
	case 6:
	case 7:
		s->lfo_frequency = (s->regs[6] >> 3) & 7;
		s->vibrato = s->regs[6] & 7;
		s->tremolo = s->regs[7] & 7;
		if (data) {
			lfo_compute_step(c, &s->pitch_lfo, s->lfo_frequency, s->vibrato, 0);
			lfo_compute_step(c, &s->amplitude_lfo, s->lfo_frequency, s->tremolo, 1);
		}
		break;
	}
}

void multipcm_write(multipcm_t *c, int offset, uint8_t data)
{
	switch (offset) {
	case 0:
		if (c->cur_slot < MULTIPCM_VOICES)
			write_slot(c, &c->slots[c->cur_slot], (int)c->address, data);
		break;
	case 1:
		c->cur_slot = (uint32_t)VALUE_TO_CHANNEL[data & 0x1f];   // (-1: no slot)
		break;
	case 2:
		c->address = data > 7 ? 7 : data;
		break;
	}
}

void multipcm_set_banks(multipcm_t *c, uint32_t lo, uint32_t hi)
{
	const uint32_t n = c->rom_size / 0x80000;
	// (memory_bank set_entry: an entry past the configured ones is not taken)
	if (lo < n) c->bank_lo = lo * 0x80000;
	if (hi < n) c->bank_hi = hi * 0x80000;
}

void multipcm_reset(multipcm_t *c)
{
	for (int i = 0; i < MULTIPCM_VOICES; i++)
		c->slots[i].playing = 0;
}

void multipcm_init(multipcm_t *c, uint32_t clock, const uint8_t *rom, uint32_t rom_size)
{
	memset(c, 0, sizeof(*c));
	c->clock = clock;
	c->rate = (float)clock / CLOCK_DIVIDER;
	c->rom = rom;
	c->rom_size = rom_size;
	tables_init(clock, c->rate);
}

// ---- the stream (sound_stream_update: one sample) ----
static void one_sample(multipcm_t *c)
{
	int32_t smpl = 0, smpr = 0;
	for (int sl = 0; sl < MULTIPCM_VOICES; ++sl) {
		multipcm_slot_t *s = &c->slots[sl];
		if (!s->playing)
			continue;
		const uint32_t vol = (s->total_level >> TL_SHIFT) | (s->pan << 7);
		uint32_t spos = s->offset >> TL_SHIFT;
		uint32_t step = s->step;
		int32_t csample;
		const int32_t fpart = (int32_t)(s->offset & ((1 << TL_SHIFT) - 1));
		if (s->reverse)
			spos = s->sample.end - spos - 1;
		if (s->sample.format & 4) {
			const uint32_t adr = s->sample.start + (spos >> 1) * 3;
			if (!(spos & 1))
				csample = (int16_t)(read_byte(c, adr) << 8 | ((read_byte(c, adr + 1) & 0xf) << 4));
			else
				csample = (int16_t)((read_byte(c, adr + 2) << 8) | (read_byte(c, adr + 1) & 0xf0));
		} else
			csample = (int16_t)(read_byte(c, s->sample.start + spos) << 8);
		int32_t sample = (csample * fpart + s->prev_sample * ((1 << TL_SHIFT) - fpart)) >> TL_SHIFT;
		if (s->vibrato) {
			step = step * (uint32_t)lfo_step(&s->pitch_lfo);
			step >>= TL_SHIFT;
		}
		s->offset += step;
		if (spos ^ (s->offset >> TL_SHIFT))
			s->prev_sample = csample;
		if (s->offset >= (s->sample.end << TL_SHIFT)) {
			s->offset -= (s->sample.end - s->sample.loop) << TL_SHIFT;
			s->reverse = 0;
		}
		if ((s->total_level >> TL_SHIFT) != s->dest_total_level)
			s->total_level += (uint32_t)s->total_level_step;
		if (s->tremolo) {
			sample = sample * lfo_step(&s->amplitude_lfo);
			sample >>= TL_SHIFT;
		}
		sample = (sample * envelope_generator_update(s)) >> 10;
		smpl += (s_left_pan_table[vol] * sample) >> TL_SHIFT;
		smpr += (s_right_pan_table[vol] * sample) >> TL_SHIFT;
	}
	c->cur_l = smpl < -32768 ? -32768 : smpl > 32767 ? 32767 : smpl;
	c->cur_r = smpr < -32768 ? -32768 : smpr > 32767 ? 32767 : smpr;
}

void multipcm_render_lr(multipcm_t *c, int32_t *l, int32_t *r, int n, uint32_t out_rate, int gain)
{
	const uint64_t div = (uint64_t)out_rate * CLOCK_DIVIDER;
	for (int i = 0; i < n; i++) {
		c->frac += c->clock;
		while (c->frac >= div) {
			c->frac -= div;
			one_sample(c);
		}
		l[i] += c->cur_l * gain / 256;
		r[i] += c->cur_r * gain / 256;
	}
}
