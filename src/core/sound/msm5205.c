/*  OKI MSM5205 ADPCM (msm5205.h), ported from MAME src/devices/sound/msm5205.cpp
    (BSD-3-Clause, copyright-holders: Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "msm5205.h"

#include <math.h>
#include <string.h>

static int s_diff[49 * 16];
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
			s_diff[step * 16 + nib] = nbl2bit[nib][0] *
				(stepval * nbl2bit[nib][1] + stepval / 2 * nbl2bit[nib][2] + stepval / 4 * nbl2bit[nib][3] + stepval / 8);
	}
	s_tables = 1;
}

void msm5205_set_selector(msm5205_t *m, int selector)
{
	int s1 = selector & 1, s2 = (selector >> 1) & 1;
	m->prescaler = s1 ? (s2 ? 0 : 48) : (s2 ? 64 : 96);
	m->bitwidth = (selector & 4) ? 4 : 3;
}

void msm5205_init(msm5205_t *m, uint32_t clock, int selector, void (*vck)(void))
{
	if (!s_tables)
		compute_tables();
	memset(m, 0, sizeof(*m));
	m->clock = clock;
	m->vck = vck;
	msm5205_set_selector(m, selector);
}

void msm5205_reset(msm5205_t *m)
{
	m->data = 0;
	m->reset = 0;
	m->signal = 0;
	m->step = 0;
	m->acc = 0;
}

// one sample period: MAME update_adpcm (VCK low edge + capture delay)
static void update_adpcm(msm5205_t *m)
{
	if (m->vck)
		m->vck();
	if (m->reset) {
		m->signal = 0;
		m->step = 0;
		return;
	}
	int val = m->data;
	int s = m->signal + s_diff[m->step * 16 + (val & 15)];
	if (s > 2047) s = 2047;
	else if (s < -2048) s = -2048;
	m->signal = s;
	m->step += index_shift[val & 7];
	if (m->step > 48) m->step = 48;
	else if (m->step < 0) m->step = 0;
}

void msm5205_clock(msm5205_t *m) { update_adpcm(m); }
// VCK slave mode (selector SEX_3B / SEX_4B): the board drives VCK; a capture happens on its falling edge
void msm5205_vclk_capture(msm5205_t *m)
{
	update_adpcm(m);
}

void msm5205_advance(msm5205_t *m, uint32_t clocks)
{
	if (!m->prescaler)
		return;
	m->acc += clocks;
	while (m->acc >= (uint32_t)m->prescaler) {
		m->acc -= (uint32_t)m->prescaler;
		update_adpcm(m);
	}
}
