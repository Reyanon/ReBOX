/*  Hudson HuC6280 PSG (C6280), ported from MAME src/devices/sound/c6280.cpp (BSD-3-Clause, copyright-holders:
    Charles MacDonald). See c6280.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include <stddef.h>
#include "c6280.h"

#include <math.h>
#include <string.h>

void c6280_init(c6280_t *c, uint32_t clock)
{
	memset(c, 0, sizeof(*c));
	c->clock = clock;
	double level = 65535.0 / 6.0 / 32.0;   // (the PSG's 48 dB over 32 steps)
	const double step = 48.0 / 32.0;
	for (int i = 0; i < 31; i++) {
		c->volume_table[i] = (int16_t)(uint16_t)level;
		level /= pow(10.0, step / 20.0);
	}
	c->volume_table[31] = 0;
}

void c6280_reset(c6280_t *c)
{
	// (MAME: the registers cleared at device_start, device_reset only the indexes and the noise seeds)
	memset(c, 0, C6280_STATE_SIZE);
	for (int ch = 0; ch < 6; ch++)
		if (ch >= 4)
			c->ch[ch].noise_seed = 1;
}

void c6280_w(c6280_t *c, int offset, uint8_t data)
{
	c6280_channel *ch = &c->ch[c->select];
	switch (offset & 0x0f) {
	case 0x00: c->select = data & 0x07; break;
	case 0x01: c->balance = data; break;
	case 0x02: ch->frequency = (uint16_t)((ch->frequency & 0x0f00) | data); break;
	case 0x03: ch->frequency = (uint16_t)((ch->frequency & 0x00ff) | ((data << 8) & 0x0f00)); break;
	case 0x04:
		if ((ch->control & 0x40) && !(data & 0x40))   // (the DDA bit 1 -> 0: the waveform index reset)
			ch->index = 0;
		if (!(ch->control & 0x80) && (data & 0x80))
			ch->tick = ch->frequency;
		ch->control = data;
		c->enabled = (uint8_t)((c->enabled & ~(1 << c->select)) | (((data >> 7) & 1) << c->select));
		break;
	case 0x05: ch->balance = data; break;
	case 0x06:
		if (!(ch->control & 0x40)) {
			ch->waveform[ch->index & 0x1f] = data & 0x1f;
			if (!(ch->control & 0x80))
				ch->index = (ch->index + 1) & 0x1f;
		} else
			ch->dda = data & 0x1f;
		break;
	case 0x07: ch->noise_control = data; break;
	case 0x08: c->lfo_frequency = data; break;
	case 0x09: c->lfo_control = data; break;
	}
}

static const uint8_t scale_tab[16] = { 0x00, 0x03, 0x05, 0x07, 0x09, 0x0b, 0x0d, 0x0f,
                                       0x10, 0x13, 0x15, 0x17, 0x19, 0x1b, 0x1d, 0x1f };

static inline int32_t run_of(int32_t tick) { return tick > 0 ? tick : 1; }

// the sum of (data - 16) over d clocks of a plain waveform channel
static int64_t wave_sum(c6280_channel *ch, int32_t d)
{
	const int32_t step = ch->frequency ? ch->frequency : 0x1000;
	int64_t s = 0;
	while (d > 0) {
		int32_t r = run_of(ch->tick);
		if (r > d) r = d;
		s += (int64_t)(ch->waveform[ch->index] - 16) * r;
		ch->tick -= r;
		d -= r;
		if (ch->tick <= 0) {
			ch->tick = step;
			ch->index = (ch->index + 1) & 0x1f;
		}
	}
	return s;
}

static int64_t noise_sum(c6280_channel *ch, int32_t d)
{
	const int32_t step = (int32_t)((ch->noise_control & 0x1f) ^ 0x1f);
	int64_t s = 0;
	while (d > 0) {
		int32_t r = run_of(ch->noise_counter);
		if (r > d) r = d;
		s += (int64_t)(((ch->noise_seed & 1) ? 0x1f : 0) - 16) * r;
		ch->noise_counter -= r;
		d -= r;
		if (ch->noise_counter <= 0) {
			ch->noise_counter = step << 6;
			const uint32_t seed = ch->noise_seed;
			ch->noise_seed = (seed >> 1) |
			                 ((((seed >> 0) ^ (seed >> 1) ^ (seed >> 11) ^ (seed >> 12) ^ (seed >> 17)) & 1) << 17);
		}
	}
	return s;
}

// channel 0 frequency-modulated by channel 1 (the LFO); sums of both over d clocks (channel 1 outputs the
// index the LFO stepping left it at, after channel 0 in the same clock)
static void lfo_sums(c6280_t *c, int32_t d, int64_t *s0, int64_t *s1)
{
	c6280_channel *src = &c->ch[1], *dst = &c->ch[0];
	const int32_t lfo_step = src->frequency ? src->frequency : 0x1000;
	const int32_t dstf = dst->frequency ? dst->frequency : 0x1000;
	const int shift = ((c->lfo_control & 3) - 1) << 1;
	while (d > 0) {
		int32_t step, r;
		if (c->lfo_control & 0x80) {   // (the LFO held in reset)
			src->tick = lfo_step * c->lfo_frequency;
			src->index = 0;
			step = dstf;
			r = run_of(dst->tick);
			if (r > d) r = d;
			*s0 += (int64_t)(dst->waveform[dst->index] - 16) * r;
			*s1 += (int64_t)(src->waveform[0] - 16) * r;
		} else {
			const int32_t lfo_data = src->waveform[src->index];
			step = dstf + ((lfo_data - 16) << shift);
			r = run_of(src->tick);
			if (run_of(dst->tick) < r) r = run_of(dst->tick);
			if (r > d) r = d;
			*s0 += (int64_t)(dst->waveform[dst->index] - 16) * r;
			src->tick -= r;
			if (src->tick <= 0) {
				*s1 += (int64_t)(src->waveform[src->index] - 16) * (r - 1);
				src->tick = lfo_step * c->lfo_frequency;
				src->index = (src->index + 1) & 0x1f;
				*s1 += src->waveform[src->index] - 16;
			} else
				*s1 += (int64_t)(src->waveform[src->index] - 16) * r;
		}
		dst->tick -= r;
		d -= r;
		if (dst->tick <= 0) {
			dst->tick = step;
			dst->index = (dst->index + 1) & 0x1f;
		}
	}
}

void c6280_render(c6280_t *c, int32_t *out, int n, uint32_t rate, int gain)
{
	const int lmal = scale_tab[(c->balance >> 4) & 0x0f], rmal = scale_tab[c->balance & 0x0f];
	int32_t vl[6], vr[6];
	for (int ch = 0; ch < 6; ch++) {
		const c6280_channel *h = &c->ch[ch];
		const int lal = scale_tab[(h->balance >> 4) & 0x0f], ral = scale_tab[h->balance & 0x0f], al = h->control & 0x1f;
		int l = (0x1f - lmal) + (0x1f - al) + (0x1f - lal), r = (0x1f - rmal) + (0x1f - al) + (0x1f - ral);
		vl[ch] = c->volume_table[l > 0x1f ? 0x1f : l];
		vr[ch] = c->volume_table[r > 0x1f ? 0x1f : r];
	}
	const int lfo = (c->lfo_control & 3) != 0;
	for (int i = 0; i < n; i++) {
		c->frac += c->clock;
		const int32_t d = (int32_t)(c->frac / rate);
		c->frac -= (uint64_t)d * rate;
		if (d <= 0)
			continue;
		int64_t acc = 0, s1 = 0;
		int s1_done = 0;
		for (int ch = 0; ch < 6; ch++) {
			if (!((c->enabled >> ch) & 1))
				continue;
			c6280_channel *h = &c->ch[ch];
			int64_t s;
			if (ch >= 4 && (h->noise_control & 0x80))
				s = noise_sum(h, d);
			else if (h->control & 0x40)
				s = (int64_t)(h->dda - 16) * d;
			else if (lfo && ch < 2) {
				if (ch == 0) {
					s = 0;
					lfo_sums(c, d, &s, &s1);
					s1_done = 1;
				} else
					s = s1_done ? s1 : (int64_t)(h->waveform[h->index] - 16) * d;
			} else
				s = wave_sum(h, d);
			acc += s * (vl[ch] + vr[ch]);
		}
		out[i] += (int32_t)(acc / d * gain / 256);
	}
}
