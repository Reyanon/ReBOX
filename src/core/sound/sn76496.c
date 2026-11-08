/*  TI SN76496 family PSG, ported from MAME src/devices/sound/sn76496.cpp (BSD-3-Clause, copyright-holders:
    Nicola Salmoria).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "sn76496.h"

#include <string.h>

void sn76496_init(sn76496_t *p, uint32_t clock, int feedback_mask, int tap1, int tap2, int negate, int clock_divider,
                  int gain, int divider, int dc_block)
{
	memset(p, 0, sizeof(*p));
	p->clock = clock;
	p->feedback_mask = feedback_mask;
	p->tap1 = tap1;
	p->tap2 = tap2;
	p->negate = negate;
	p->clock_divider = clock_divider;
	p->gain = gain;
	p->divider = divider < 1 ? 1 : (uint32_t)divider;
	p->dc_block = dc_block;
	sn76496_reset(p);
}

void sn76496_reset(sn76496_t *p)
{
	// (MAME device_start: the chip state is set up once)
	const int MAX_OUTPUT = 0x7fff;
	p->last_register = 0;
	for (int i = 0; i < 8; i += 2) {
		p->reg[i] = 0;
		p->reg[i + 1] = 0;
	}
	for (int i = 0; i < 4; i++) {
		p->output[i] = 0;
		p->count[i] = 0;
		p->period[i] = i == 3 ? 0 : 0x400;
	}
	p->rng = (uint32_t)p->feedback_mask;
	p->output[3] = p->rng & 1;
	p->current_clock = p->clock_divider - 1;
	double out = MAX_OUTPUT / 4;
	for (int i = 0; i < 15; i++) {
		p->vol_table[i] = out > MAX_OUTPUT / 4 ? MAX_OUTPUT / 4 : (int32_t)out;
		out /= 1.258925412;   // 2 dB
	}
	p->vol_table[15] = 0;
	for (int i = 0; i < 4; i++)
		p->volume[i] = p->vol_table[p->reg[i * 2 + 1]];
	p->frac = 0;
	p->div_count = 0;
	p->div_acc = 0;
	p->dc = 0;
}

uint32_t sn76496_rate(const sn76496_t *p) { return p->clock / 2 / p->divider; }
uint32_t sn76496_state_size(void) { return (uint32_t)SN76496_STATE_END; }

void sn76496_write(sn76496_t *p, uint8_t data)
{
	int r;
	if (data & 0x80) {
		r = (data & 0x70) >> 4;
		p->last_register = r;
		p->reg[r] = (p->reg[r] & 0x3f0) | (data & 0x0f);
	} else
		r = p->last_register;
	int c = r >> 1;
	switch (r) {
	case 0: case 2: case 4:
		if (!(data & 0x80))
			p->reg[r] = (p->reg[r] & 0x0f) | ((data & 0x3f) << 4);
		p->period[c] = p->reg[r] ? p->reg[r] : 0x400;
		if (r == 4 && (p->reg[6] & 3) == 3)
			p->period[3] = p->period[2] << 1;
		break;
	case 1: case 3: case 5: case 7:
		p->volume[c] = p->vol_table[data & 0x0f];
		if (!(data & 0x80))
			p->reg[r] = (p->reg[r] & 0x3f0) | (data & 0x0f);
		break;
	case 6: {
		if (!(data & 0x80))
			p->reg[r] = (p->reg[r] & 0x3f0) | (data & 0x0f);
		int n = p->reg[6];
		p->period[3] = ((n & 3) == 3) ? (p->period[2] << 1) : (1 << (5 + (n & 3)));
		p->rng = (uint32_t)p->feedback_mask;
		break;
	}
	}
}

int sn76496_render(sn76496_t *p, uint32_t clocks, int32_t *out, int max)
{
	int n = 0;
	p->frac += clocks;
	while (p->frac >= 2) {
		p->frac -= 2;
		if (p->current_clock > 0)
			p->current_clock--;
		else {
			p->current_clock = p->clock_divider - 1;
			for (int i = 0; i < 3; i++) {
				if (--p->count[i] <= 0) {
					p->output[i] ^= 1;
					p->count[i] = p->period[i];
				}
			}
			if (--p->count[3] <= 0) {
				if (((p->rng & (uint32_t)p->tap1) != 0) != (((p->rng & (uint32_t)p->tap2) != 0) && (p->reg[6] & 4))) {
					p->rng >>= 1;
					p->rng |= (uint32_t)p->feedback_mask;
				} else
					p->rng >>= 1;
				p->output[3] = p->rng & 1;
				p->count[3] = p->period[3];
			}
		}
		int32_t o = (p->output[0] ? p->volume[0] : 0) + (p->output[1] ? p->volume[1] : 0) +
		            (p->output[2] ? p->volume[2] : 0) + (p->output[3] ? p->volume[3] : 0);
		if (p->negate)
			o = -o;
		p->div_acc += o;
		if (++p->div_count < p->divider)
			continue;
		int32_t v = p->div_acc / (int32_t)p->divider * p->gain / 256;
		p->div_count = 0;
		p->div_acc = 0;
		if (p->dc_block) {
			// one-pole DC blocker (the PSG output is unipolar)
			p->dc += ((v << 8) - p->dc) >> 10;
			v -= p->dc >> 8;
		}
		if (n < max)
			out[n++] += v;
	}
	return n;
}
