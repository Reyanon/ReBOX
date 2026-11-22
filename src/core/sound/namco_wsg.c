/*  Namco wavetable sound generators (WSG, 15XX, CUS30). Ported from MAME 0.289 src/devices/sound/namco.cpp
    (BSD-3-Clause, copyright-holders: Nicola Salmoria, Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.

    The voice math (counters, waveform fetch, volumes, the CUS30 noise LFSR and hold) is MAME's. Differences:
    - MAME updates its stream before every register write; here registers change at render-call boundaries, so the
      caller renders in slices if it wants finer timing. cus30_r register 5 (the wave position, Baraduke) reads the
      counter as of the last render.
    - MAME resamples its 192 kHz stream to the output with its own resampler; here each output sample is the mean
      of the chip samples that fall in its period (a box filter, fractional accumulator), the previous output held
      when none does.
*/
#include "namco_wsg.h"
#include <string.h>

// quality parameter: internal sample rate is 192 KHz
#define INTERNAL_RATE 192000u

#define BUF_SIZE 1024
#define OUT_CHUNK 256
static int32_t s_buf_l[BUF_SIZE], s_buf_r[BUF_SIZE];
static uint16_t s_cnt[OUT_CHUNK];

void namco_wsg_init(namco_wsg_t *t, int type, uint32_t clock, int stereo, const uint8_t *prom, int gain256)
{
	memset(t, 0, sizeof(*t));
	t->type = type;
	t->voices = type == NAMCO_WSG_TYPE_WSG ? 3 : 8;
	t->packed = type == NAMCO_WSG_TYPE_CUS30;
	t->stereo = type == NAMCO_WSG_TYPE_CUS30 && stereo;
	t->prom = prom;
	t->mix_res = 128 * t->voices;
	t->gain = gain256;

	// device_clock_changed
	uint32_t namco_clock = clock;
	int clock_multiple;
	for (clock_multiple = 0; namco_clock < INTERNAL_RATE; clock_multiple++)
		namco_clock *= 2;
	t->f_fracbits = clock_multiple + 15;
	t->sample_rate = namco_clock;

	// device_start: sound enabled, voices reset
	t->sound_enable = 1;
	for (int i = 0; i < 8; i++)
		t->v[i].noise_seed = 1;
}

void namco_wsg_sound_enable_w(namco_wsg_t *t, int state)
{
	t->sound_enable = state != 0;
}

static inline int waveform_position(const namco_wsg_t *t, uint32_t n)
{
	return (n >> t->f_fracbits) & 0x1f;
}

static inline int waveform_r(const namco_wsg_t *t, uint16_t pos)
{
	if (t->packed) // use full byte, first 4 high bits, then low 4 bits
		return ((t->ram[(pos >> 1) & 0xff] >> ((~pos & 1) << 2)) & 0x0f) - 8;
	else // use only low 4 bits
		return t->prom ? (t->prom[pos & 0xff] & 0x0f) - 8 : -8;
}

static uint32_t update_one(const namco_wsg_t *t, int32_t *buf, int m, uint16_t select, int volume, uint32_t counter,
                           uint32_t freq)
{
	select <<= 5;
	for (int i = 0; i < m; i++) {
		buf[i] += waveform_r(t, (uint16_t)(select + waveform_position(t, counter))) * volume;
		counter += freq;
	}
	return counter;
}

/******************************************************************** WSG */

void namco_wsg_pacman_sound_w(namco_wsg_t *t, int offset, uint8_t data)
{
	int ch;
	offset &= 0x1f;
	data &= 0x0f;
	if (t->soundregs[offset] == data)
		return;
	t->soundregs[offset] = data;

	if (offset < 0x10)
		ch = (offset - 5) / 5;
	else if (offset == 0x10)
		ch = 0;
	else
		ch = (offset - 0x11) / 5;
	if (ch < 0 || ch >= t->voices)   // (offsets 0-4: C's division truncates to 0 as MAME's does, then no case matches)
		return;

	namco_wsg_voice_t *voice = &t->v[ch];
	const uint8_t *r = t->soundregs;
	switch (offset - ch * 5) {
	case 0x05:
		voice->waveform_select = data & 7;
		break;
	case 0x10: case 0x11: case 0x12: case 0x13: case 0x14:
		// the frequency has 20 bits; the first voice has extra frequency bits
		voice->frequency = (ch == 0) ? r[0x10] : 0;
		voice->frequency += (uint32_t)r[ch * 5 + 0x11] << 4;
		voice->frequency += (uint32_t)r[ch * 5 + 0x12] << 8;
		voice->frequency += (uint32_t)r[ch * 5 + 0x13] << 12;
		voice->frequency += (uint32_t)r[ch * 5 + 0x14] << 16; // always 0
		break;
	case 0x15:
		voice->volume[0] = data;
		break;
	}
}

/******************************************************************** 15XX */

void namco_15xx_w(namco_wsg_t *t, int offset, uint8_t data)
{
	offset &= 0x3f;
	if (t->soundregs[offset] == data)
		return;
	t->soundregs[offset] = data;

	const int ch = offset >> 3;
	namco_wsg_voice_t *voice = &t->v[ch];
	const uint8_t *r = t->soundregs;
	switch (offset & 7) {
	case 0x02:
		voice->counter &= (1u << t->f_fracbits) - 1;
		voice->counter |= (uint32_t)(data & 0x1f) << t->f_fracbits;
		break;
	case 0x03:
		voice->volume[0] = data & 0x0f;
		break;
	case 0x06:
		voice->waveform_select = (data >> 4) & 7;
		/* fall through */
	case 0x04:
	case 0x05:
		// the frequency has 20 bits
		voice->frequency = r[ch * 8 + 0x04];
		voice->frequency += (uint32_t)r[ch * 8 + 0x05] << 8;
		voice->frequency += (uint32_t)(r[ch * 8 + 0x06] & 15) << 16; // high bits are from here
		break;
	}
}

uint8_t namco_15xx_r(namco_wsg_t *t, int offset)
{
	return t->soundregs[offset & 0x3f];
}

uint8_t namco_15xx_amap_r(namco_wsg_t *t, int offset)
{
	offset &= 0x3ff;
	return offset < 0x040 ? namco_15xx_r(t, offset) : t->ram[offset];
}

void namco_15xx_amap_w(namco_wsg_t *t, int offset, uint8_t data)
{
	offset &= 0x3ff;
	if (offset < 0x040)
		namco_15xx_w(t, offset, data);
	else
		t->ram[offset] = data;
}

/******************************************************************** CUS30 */

void namco_cus30_w(namco_wsg_t *t, int offset, uint8_t data)
{
	if (offset < 0 || offset > 63)
		return;
	if (t->soundregs[offset] == data)
		return;
	t->soundregs[offset] = data;

	const int ch = offset >> 3;
	namco_wsg_voice_t *voice = &t->v[ch];
	const uint8_t *r = t->soundregs;
	switch (offset & 7) {
	case 0x00:
		voice->volume[0] = data & 0x0f;
		break;
	case 0x01:
		voice->waveform_select = (data >> 4) & 15;
		/* fall through */
	case 0x02:
	case 0x03:
		// the frequency has 20 bits
		voice->frequency = (uint32_t)(r[ch * 8 + 0x01] & 15) << 16; // high bits are from here
		voice->frequency += (uint32_t)r[ch * 8 + 0x02] << 8;
		voice->frequency += r[ch * 8 + 0x03];
		break;
	case 0x04:
		voice->volume[1] = data & 0x0f;
		t->v[(ch + 1) & 7].noise_sw = (data >> 7) & 1;
		break;
	}
}

uint8_t namco_cus30_r(namco_wsg_t *t, int offset)
{
	offset &= 0x3f;
	// reading from register 5 returns the counter (used by baraduke)
	if ((offset & 7) == 5)
		return (uint8_t)waveform_position(t, t->v[offset >> 3].counter);
	return t->soundregs[offset];
}

uint8_t namco_cus30_amap_r(namco_wsg_t *t, int offset)
{
	offset &= 0x3ff;
	if (offset >= 0x100 && offset < 0x140)
		return namco_cus30_r(t, offset - 0x100);
	return t->ram[offset];
}

void namco_cus30_amap_w(namco_wsg_t *t, int offset, uint8_t data)
{
	offset &= 0x3ff;
	if (offset >= 0x100 && offset < 0x140)
		namco_cus30_w(t, offset - 0x100, data);
	else
		t->ram[offset] = data;
}

/******************************************************************** stream */

// MAME's sound_stream_update for m chip samples, raw (x MIX_RES) into s_buf_l / s_buf_r
static void stream_update(namco_wsg_t *t, int m)
{
	memset(s_buf_l, 0, sizeof(s_buf_l[0]) * (size_t)m);
	memset(s_buf_r, 0, sizeof(s_buf_r[0]) * (size_t)m);
	if (!t->sound_enable)
		return;

	for (int vi = 0; vi < t->voices; vi++) {
		namco_wsg_voice_t *voice = &t->v[vi];
		if (t->type != NAMCO_WSG_TYPE_CUS30) {
			const int v = voice->volume[0];
			if (v)
				voice->counter = update_one(t, s_buf_l, m, voice->waveform_select, v, voice->counter, voice->frequency);
			continue;
		}

		const int lv = voice->volume[0];
		const int rv = voice->volume[1];
		if (voice->noise_sw) {
			const int f = voice->frequency & 0xff;
			// only update if we have non-zero volume
			if (lv || rv) {
				const int hold_time = 1 << (t->f_fracbits - 16);
				int hold = voice->noise_hold;
				const uint32_t delta = (uint32_t)f << 4;
				uint32_t c = voice->noise_counter;
				const int16_t l_noise_data = (int16_t)(0x07 * (lv >> 1));
				const int16_t r_noise_data = (int16_t)(t->stereo ? (0x07 * (rv >> 1)) : 0);

				for (int i = 0; i < m; i++) {
					if (voice->noise_state) {
						s_buf_l[i] += l_noise_data;
						s_buf_r[i] += r_noise_data;
					} else {
						s_buf_l[i] -= l_noise_data;
						s_buf_r[i] -= r_noise_data;
					}

					if (hold) {
						hold--;
						continue;
					}
					hold = hold_time;

					c += delta;
					int cnt = (int)(c >> 12);
					c &= (1u << 12) - 1;
					for (; cnt > 0; cnt--) {
						if ((voice->noise_seed + 1) & 2) voice->noise_state ^= 1;
						if (voice->noise_seed & 1) voice->noise_seed ^= 0x28000;
						voice->noise_seed >>= 1;
					}
				}
				voice->noise_counter = c;
				voice->noise_hold = hold;
			}
		} else {
			uint32_t c = voice->counter;
			if (lv)
				c = update_one(t, s_buf_l, m, voice->waveform_select, lv, voice->counter, voice->frequency);
			if (rv && t->stereo)
				c = update_one(t, s_buf_r, m, voice->waveform_select, rv, voice->counter, voice->frequency);
			voice->counter = c;
		}
	}
}

// n / d truncated toward zero, with a 32-bit division when both fit (the same result as the 64-bit one)
static inline int32_t div_fit(int64_t n, int64_t d)
{
	if (n >= -0x7fffffffLL && n <= 0x7fffffffLL && d > 0 && d <= 0x7fffffffLL)
		return (int32_t)n / (int32_t)d;
	return (int32_t)(n / d);
}

void namco_wsg_render(namco_wsg_t *t, int32_t *left, int32_t *right, int n, uint32_t out_rate)
{
	const uint64_t chip_rate = t->sample_rate;
	// value / MIX_RES x 32768 x gain / 256
	const int64_t mul = 128 * (int64_t)t->gain;
	const int mono = !t->stereo;

	while (n > 0) {
		// how many chip samples fall in each output period of this chunk
		int j = 0, m = 0;
		uint64_t frac = t->frac;
		while (j < n && j < OUT_CHUNK) {
			const uint64_t a = frac + chip_rate;
			// (a 32-bit division when it fits: no libgcc call on ARM, the same quotient)
			int k = (int)(a <= 0xffffffffu ? (uint32_t)a / out_rate : a / out_rate);
			if (k > BUF_SIZE) k = BUF_SIZE;
			if (m + k > BUF_SIZE) break;
			frac = a - (uint64_t)k * out_rate;
			if (frac >= out_rate) frac = 0;   // (clamped: out_rate below the documented minimum)
			s_cnt[j++] = (uint16_t)k;
			m += k;
		}
		t->frac = frac;

		stream_update(t, m);

		int p = 0;
		for (int i = 0; i < j; i++) {
			const int k = s_cnt[i];
			if (k) {
				int64_t sl = 0, sr = 0;
				for (int q = 0; q < k; q++) {
					sl += s_buf_l[p + q];
					sr += s_buf_r[p + q];
				}
				p += k;
				const int64_t d = (int64_t)t->mix_res * k;
				t->cur_l = div_fit(sl * mul, d);
				t->cur_r = mono ? t->cur_l : div_fit(sr * mul, d);
			}
			left[i] += t->cur_l;
			if (right)
				right[i] += t->cur_r;
		}
		left += j;
		if (right)
			right += j;
		n -= j;
	}
}
