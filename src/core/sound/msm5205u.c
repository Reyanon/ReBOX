/*  OKI MSM5205 ADPCM, ported from MAME src/devices/sound/msm5205.cpp (BSD-3-Clause, copyright-holders: Aaron Giles;
    ADPCM by Buffoni Mirko). The VCK timer runs at clock / prescaler; the ADPCM capture (update_adpcm) follows the
    falling VCK edge by 6 master clocks, as in MAME.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "msm5205u.h"

#include <math.h>
#include <string.h>

static int s_diff_lookup[49 * 16];
static int s_tables;

static const int index_shift[8] = { -1, -1, -1, -1, 2, 4, 6, 8 };

static void compute_tables(void)
{
	static const int nbl2bit[16][4] = {
		{ 1, 0, 0, 0}, { 1, 0, 0, 1}, { 1, 0, 1, 0}, { 1, 0, 1, 1},
		{ 1, 1, 0, 0}, { 1, 1, 0, 1}, { 1, 1, 1, 0}, { 1, 1, 1, 1},
		{-1, 0, 0, 0}, {-1, 0, 0, 1}, {-1, 0, 1, 0}, {-1, 0, 1, 1},
		{-1, 1, 0, 0}, {-1, 1, 0, 1}, {-1, 1, 1, 0}, {-1, 1, 1, 1}
	};
	for (int step = 0; step <= 48; step++) {
		int stepval = (int)floor(16.0 * pow(11.0 / 10.0, (double)step));
		for (int nib = 0; nib < 16; nib++)
			s_diff_lookup[step * 16 + nib] = nbl2bit[nib][0] *
				(stepval * nbl2bit[nib][1] + stepval / 2 * nbl2bit[nib][2] + stepval / 4 * nbl2bit[nib][3] + stepval / 8);
	}
	s_tables = 1;
}

void msm5205u_init(msm5205u_t *m, uint32_t clock, int select, void (*vck)(msm5205u_t *, void *), void *user)
{
	if (!s_tables)
		compute_tables();
	memset(m, 0, sizeof(*m));
	m->clock = clock;
	int s1 = select & 1, s2 = (select >> 1) & 1;
	m->prescaler = s1 ? (s2 ? 0 : 48) : (s2 ? 64 : 96);
	m->bitwidth = (select & 4) ? 4 : 3;
	m->vck = vck;
	m->user = user;
	msm5205u_reset(m);
}

void msm5205u_reset(msm5205u_t *m)
{
	m->data = 0;
	m->reset = 0;
	m->signal = 0;
	m->step = 0;
	m->frac = 0;
	// VCK timer started at device start: falling edge after a full period, capture 6 clocks later
	m->countdown = m->prescaler + 6;
}

void msm5205u_reset_w(msm5205u_t *m, int state) { m->reset = (uint8_t)(state ? 1 : 0); }

void msm5205u_data_w(msm5205u_t *m, uint8_t data)
{
	if (m->bitwidth == 4)
		m->data = data & 0x0f;
	else
		m->data = (uint8_t)((data & 0x07) << 1);   // unknown (MAME)
}

// MAME update_adpcm
static void update_adpcm(msm5205u_t *m)
{
	int new_signal;
	if (m->vck)
		m->vck(m, m->user);
	if (m->reset) {
		new_signal = 0;
		m->step = 0;
	} else {
		int val = m->data;
		new_signal = m->signal + s_diff_lookup[m->step * 16 + (val & 15)];
		if (new_signal > 2047)
			new_signal = 2047;
		else if (new_signal < -2048)
			new_signal = -2048;
		m->step += index_shift[val & 7];
		if (m->step > 48)
			m->step = 48;
		else if (m->step < 0)
			m->step = 0;
	}
	m->signal = new_signal;
}

void msm5205u_advance(msm5205u_t *m, uint32_t clocks)
{
	if (!m->prescaler)
		return;
	m->countdown -= (int32_t)clocks;
	while (m->countdown <= 0) {
		update_adpcm(m);
		m->countdown += m->prescaler;
	}
}

void msm5205u_render(msm5205u_t *m, int32_t *out, int n, uint32_t rate, int gain)
{
	for (int i = 0; i < n; i++) {
		m->frac += m->clock;
		uint32_t t = m->frac / rate;
		m->frac -= t * rate;
		if (t)
			msm5205u_advance(m, t);
		out[i] += msm5205u_output(m) * gain / 256;
	}
}
