/*  OKI MSM5232 8-channel tone generator, ported from MAME src/devices/sound/msm5232.cpp
    (GPL-2.0+, copyright-holders: Jarek Burczynski, Hiromitsu Shioya).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "msm5232.h"

#include <string.h>

#define CLOCK_RATE_DIVIDER 16
#define STEP_SH 16
#define VMIN 0
#define VMAX 32768

#define ROM(counter, bindiv) ((counter) | ((bindiv) << 9))
static const uint16_t MSM5232_ROM[88] = {
	ROM(506, 7),
	ROM(478, 7), ROM(451, 7), ROM(426, 7), ROM(402, 7), ROM(379, 7), ROM(358, 7), ROM(338, 7), ROM(319, 7),
	ROM(301, 7), ROM(284, 7), ROM(268, 7), ROM(253, 7),
	ROM(478, 6), ROM(451, 6), ROM(426, 6), ROM(402, 6), ROM(379, 6), ROM(358, 6), ROM(338, 6), ROM(319, 6),
	ROM(301, 6), ROM(284, 6), ROM(268, 6), ROM(253, 6),
	ROM(478, 5), ROM(451, 5), ROM(426, 5), ROM(402, 5), ROM(379, 5), ROM(358, 5), ROM(338, 5), ROM(319, 5),
	ROM(301, 5), ROM(284, 5), ROM(268, 5), ROM(253, 5),
	ROM(478, 4), ROM(451, 4), ROM(426, 4), ROM(402, 4), ROM(379, 4), ROM(358, 4), ROM(338, 4), ROM(319, 4),
	ROM(301, 4), ROM(284, 4), ROM(268, 4), ROM(253, 4),
	ROM(478, 3), ROM(451, 3), ROM(426, 3), ROM(402, 3), ROM(379, 3), ROM(358, 3), ROM(338, 3), ROM(319, 3),
	ROM(301, 3), ROM(284, 3), ROM(268, 3), ROM(253, 3),
	ROM(478, 2), ROM(451, 2), ROM(426, 2), ROM(402, 2), ROM(379, 2), ROM(358, 2), ROM(338, 2), ROM(319, 2),
	ROM(301, 2), ROM(284, 2), ROM(268, 2), ROM(253, 2),
	ROM(478, 1), ROM(451, 1), ROM(426, 1), ROM(402, 1), ROM(379, 1), ROM(358, 1), ROM(338, 1), ROM(319, 1),
	ROM(301, 1), ROM(284, 1), ROM(268, 1), ROM(253, 1),
	ROM(253, 1), ROM(253, 1),
	ROM(13, 7)
};
#undef ROM

// attack / decay 1 / decay 2 resistances (MAME: guesswork from the datasheet timings)
static const double R51 = 870, R52 = 17400, R53 = 101000;

static void init_tables(msm5232_t *m)
{
	const uint32_t rate = m->clock / CLOCK_RATE_DIVIDER;
	m->update_step = (uint32_t)(int)((double)(1 << STEP_SH) * (double)rate / (double)m->clock);
	const double scale = (double)m->clock / (double)rate;
	m->noise_step = (int32_t)(((1 << STEP_SH) / 128.0) * scale);
	const double clockscale = (double)m->clock / 2119040.0;
	for (int i = 0; i < 8; i++) {
		int rcp = 1 << ((i & 4) ? (i & ~2) : i);
		m->ar_tbl[i] = (rcp / clockscale) * R51;
		m->dr_tbl[i] = (rcp / clockscale) * R52;
		m->dr_tbl[i + 8] = (rcp / clockscale) * R53;
	}
}

static void init_voice(msm5232_t *m, int i)
{
	msm5232_voice *v = &m->voi[i];
	memset(v, 0, sizeof(*v));
	v->ar_rate = m->ar_tbl[0] * m->cap[i];
	v->dr_rate = m->dr_tbl[0] * m->cap[i];
	v->rr_rate = m->dr_tbl[0] * m->cap[i];
	v->eg_sect = -1;
	v->eg = 0;
	v->eg_arm = 0;
	v->pitch = -1;
}

static void gate_update(msm5232_t *m)
{
	int st = (m->control2 & 0x20) ? m->voi[7].GF : 0;
	if (m->gate != st) {
		m->gate = st;
		if (m->gate_cb)
			m->gate_cb(m->user, st);
	}
}

void msm5232_init(msm5232_t *m, uint32_t clock, const double caps[8], int divider)
{
	memset(m, 0, sizeof(*m));
	m->clock = clock;
	m->divider = divider < 1 ? 1 : (uint32_t)divider;
	for (int i = 0; i < 8; i++)
		m->cap[i] = caps[i];
	init_tables(m);
	for (int i = 0; i < 8; i++)
		init_voice(m, i);
}

uint32_t msm5232_rate(const msm5232_t *m) { return m->clock / CLOCK_RATE_DIVIDER / m->divider; }
uint32_t msm5232_state_size(void) { return (uint32_t)MSM5232_STATE_END; }

void msm5232_write(msm5232_t *m, int offset, uint8_t data)
{
	if (offset > 0x0d)
		return;
	if (offset < 0x08) {
		const int ch = offset & 7;
		msm5232_voice *v = &m->voi[ch];
		v->GF = (data >> 7) & 1;
		if (ch == 7)
			gate_update(m);
		if (data & 0x80) {
			if (data >= 0xd8) {
				v->mode = 1;      // noise
				v->eg_sect = 0;   // key on
			} else {
				if (v->pitch != (data & 0x7f)) {
					v->pitch = data & 0x7f;
					const uint32_t pg = MSM5232_ROM[data & 0x7f];
					v->TG_count_period = (int32_t)((pg & 0x1ff) * m->update_step / 2);
					int n = (pg >> 9) & 7;
					v->TG_out16 = (uint8_t)(1 << n);
					n = n > 0 ? n - 1 : 0;
					v->TG_out8 = (uint8_t)(1 << n);
					n = n > 0 ? n - 1 : 0;
					v->TG_out4 = (uint8_t)(1 << n);
					n = n > 0 ? n - 1 : 0;
					v->TG_out2 = (uint8_t)(1 << n);
				}
				v->mode = 0;
				v->eg_sect = 0;
			}
		} else
			v->eg_sect = v->eg_arm ? 1 : 2;   // key off: decay (ARM = 1) or release
		return;
	}
	switch (offset) {
	case 0x08:
		for (int i = 0; i < 4; i++) m->voi[i].ar_rate = m->ar_tbl[data & 7] * m->cap[i];
		break;
	case 0x09:
		for (int i = 0; i < 4; i++) m->voi[i + 4].ar_rate = m->ar_tbl[data & 7] * m->cap[i + 4];
		break;
	case 0x0a:
		for (int i = 0; i < 4; i++) m->voi[i].dr_rate = m->dr_tbl[data & 0xf] * m->cap[i];
		break;
	case 0x0b:
		for (int i = 0; i < 4; i++) m->voi[i + 4].dr_rate = m->dr_tbl[data & 0xf] * m->cap[i + 4];
		break;
	case 0x0c:
	case 0x0d: {
		const int g = offset - 0x0c;
		if (g)
			m->control2 = data;
		else
			m->control1 = data;
		if (g)
			gate_update(m);
		for (int i = 0; i < 4; i++) {
			msm5232_voice *v = &m->voi[g * 4 + i];
			if ((data & 0x10) && v->eg_sect == 1)
				v->eg_sect = 0;
			v->eg_arm = data & 0x10;
		}
		m->EN_out16[g] = (data & 1) ? ~0u : 0;
		m->EN_out8[g] = (data & 2) ? ~0u : 0;
		m->EN_out4[g] = (data & 4) ? ~0u : 0;
		m->EN_out2[g] = (data & 8) ? ~0u : 0;
		break;
	}
	}
}

void msm5232_reset(msm5232_t *m)
{
	for (int i = 0; i < 8; i++) {
		msm5232_write(m, i, 0x80);
		msm5232_write(m, i, 0x00);
	}
	m->noise_cnt = 0;
	m->noise_rng = 1;
	m->noise_clocks = 0;
	m->control1 = m->control2 = 0;
	for (int g = 0; g < 2; g++)
		m->EN_out16[g] = m->EN_out8[g] = m->EN_out4[g] = m->EN_out2[g] = 0;
	gate_update(m);
	m->frac = 0;
	m->div_count = 0;
	m->div_acc = 0;
}

static void eg_advance(msm5232_t *m)
{
	const int samplerate = (int)(m->clock / CLOCK_RATE_DIVIDER);
	for (int i = 0; i < 8; i++) {
		msm5232_voice *v = &m->voi[i];
		switch (v->eg_sect) {
		case 0:   // attack: capacitor charge
			if (v->eg < VMAX) {
				v->counter -= (int)((VMAX - v->eg) / v->ar_rate);
				if (v->counter <= 0) {
					int n = -v->counter / samplerate + 1;
					v->counter += n * samplerate;
					if ((v->eg += n) > VMAX)
						v->eg = VMAX;
				}
			}
			// ARM = 0: decay as soon as the capacitor reaches about 80 %
			if (!v->eg_arm && v->eg >= VMAX * 80 / 100)
				v->eg_sect = 1;
			v->egvol = v->eg / 16;
			break;
		case 1:   // decay
		case 2: { // release
			double r = v->eg_sect == 1 ? v->dr_rate : v->rr_rate;
			if (v->eg > VMIN) {
				v->counter -= (int)((v->eg - VMIN) / r);
				if (v->counter <= 0) {
					int n = -v->counter / samplerate + 1;
					v->counter += n * samplerate;
					if ((v->eg -= n) < VMIN)
						v->eg = VMIN;
				}
			} else
				v->eg_sect = -1;
			v->egvol = v->eg / 16;
			break;
		}
		default:
			break;
		}
	}
}

// one group: o[0..3] = 2', 4', 8', 16'; solo[0..1] = solo 8', 16' (group 2 only)
static void tg_group(msm5232_t *m, int g, int32_t *o, int32_t *solo)
{
	int o2 = 0, o4 = 0, o8 = 0, o16 = 0;
	for (int i = 0; i < 4; i++) {
		msm5232_voice *v = &m->voi[g * 4 + i];
		int out2 = 0, out4 = 0, out8 = 0, out16 = 0;
		if (v->mode == 0) {
			int left = 1 << STEP_SH;
			do {
				int nextevent = left;
				if (v->TG_cnt & v->TG_out16) out16 += v->TG_count;
				if (v->TG_cnt & v->TG_out8) out8 += v->TG_count;
				if (v->TG_cnt & v->TG_out4) out4 += v->TG_count;
				if (v->TG_cnt & v->TG_out2) out2 += v->TG_count;
				v->TG_count -= nextevent;
				while (v->TG_count <= 0) {
					v->TG_count += v->TG_count_period;
					v->TG_cnt++;
					if (v->TG_cnt & v->TG_out16) out16 += v->TG_count_period;
					if (v->TG_cnt & v->TG_out8) out8 += v->TG_count_period;
					if (v->TG_cnt & v->TG_out4) out4 += v->TG_count_period;
					if (v->TG_cnt & v->TG_out2) out2 += v->TG_count_period;
					if (v->TG_count > 0)
						break;
					v->TG_count += v->TG_count_period;
					v->TG_cnt++;
					if (v->TG_cnt & v->TG_out16) out16 += v->TG_count_period;
					if (v->TG_cnt & v->TG_out8) out8 += v->TG_count_period;
					if (v->TG_cnt & v->TG_out4) out4 += v->TG_count_period;
					if (v->TG_cnt & v->TG_out2) out2 += v->TG_count_period;
					if (v->TG_count_period <= 0)
						break;   // (period 0 before the first key on: guard against an endless loop)
				}
				if (v->TG_cnt & v->TG_out16) out16 -= v->TG_count;
				if (v->TG_cnt & v->TG_out8) out8 -= v->TG_count;
				if (v->TG_cnt & v->TG_out4) out4 -= v->TG_count;
				if (v->TG_cnt & v->TG_out2) out2 -= v->TG_count;
				left -= nextevent;
			} while (left > 0);
		} else {
			if (m->noise_clocks & 8) out16 += 1 << STEP_SH;
			if (m->noise_clocks & 4) out8 += 1 << STEP_SH;
			if (m->noise_clocks & 2) out4 += 1 << STEP_SH;
			if (m->noise_clocks & 1) out2 += 1 << STEP_SH;
		}
		o16 += ((out16 - (1 << (STEP_SH - 1))) * v->egvol) >> STEP_SH;
		o8 += ((out8 - (1 << (STEP_SH - 1))) * v->egvol) >> STEP_SH;
		o4 += ((out4 - (1 << (STEP_SH - 1))) * v->egvol) >> STEP_SH;
		o2 += ((out2 - (1 << (STEP_SH - 1))) * v->egvol) >> STEP_SH;
		if (i == 3 && g == 1) {
			solo[1] += ((out16 - (1 << (STEP_SH - 1))) << 11) >> STEP_SH;
			solo[0] += ((out8 - (1 << (STEP_SH - 1))) << 11) >> STEP_SH;
		}
	}
	o[0] = (int32_t)((uint32_t)o2 & m->EN_out2[g]);
	o[1] = (int32_t)((uint32_t)o4 & m->EN_out4[g]);
	o[2] = (int32_t)((uint32_t)o8 & m->EN_out8[g]);
	o[3] = (int32_t)((uint32_t)o16 & m->EN_out16[g]);
}

int msm5232_render(msm5232_t *m, uint32_t clocks, int32_t *out, int max)
{
	int n = 0;
	m->frac += clocks;
	while (m->frac >= CLOCK_RATE_DIVIDER) {
		m->frac -= CLOCK_RATE_DIVIDER;
		int32_t o[MSM5232_OUTPUTS] = { 0 };
		eg_advance(m);
		tg_group(m, 0, o, o + 8);
		tg_group(m, 1, o + 4, o + 8);
		o[MSM5232_OUT_NOISE] = (m->noise_rng & (1 << 16)) ? 32768 : 0;
		// noise generator
		int cnt = (m->noise_cnt += m->noise_step) >> STEP_SH;
		m->noise_cnt &= (1 << STEP_SH) - 1;
		while (cnt > 0) {
			int tmp = m->noise_rng & (1 << 16);
			if (m->noise_rng & 1)
				m->noise_rng ^= 0x24000;
			m->noise_rng >>= 1;
			if ((m->noise_rng & (1 << 16)) != tmp)
				m->noise_clocks++;
			cnt--;
		}
		int64_t s = 0;
		for (int c = 0; c < MSM5232_OUTPUTS; c++)
			s += (int64_t)o[c] * m->gain[c];
		m->div_acc += s;
		if (++m->div_count < m->divider)
			continue;
		if (n < max)
			out[n++] += (int32_t)(m->div_acc / ((int64_t)m->divider * 256));
		m->div_count = 0;
		m->div_acc = 0;
	}
	return n;
}
