/*  Ricoh RF5C68 PCM. Ported from MAME src/devices/sound/rf5c68.cpp (BSD-3-Clause, copyright-holders: Olivier Galibert,
    Aaron Giles): 8 channels stepping through the wave RAM in 11-bit fractions, 0xff the loop mark, 10-bit output.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "rf5c68.h"

#include <string.h>

void rf5c68_init(rf5c68_t *c, uint32_t clock)
{
	memset(c, 0, sizeof(*c));
	c->clock = clock;
}

void rf5c68_w(rf5c68_t *c, int offset, uint8_t data)
{
	rf5c68_chan_t *chan = &c->chan[c->cbank];
	switch (offset) {
	case 0x00: chan->env = data; break;
	case 0x01: chan->pan = data; break;
	case 0x02: chan->step = (uint16_t)((chan->step & 0xff00) | data); break;
	case 0x03: chan->step = (uint16_t)((chan->step & 0x00ff) | (data << 8)); break;
	case 0x04: chan->loopst = (uint16_t)((chan->loopst & 0xff00) | data); break;
	case 0x05: chan->loopst = (uint16_t)((chan->loopst & 0x00ff) | (data << 8)); break;
	case 0x06:
		chan->start = data;
		if (!chan->enable)
			chan->addr = (uint32_t)chan->start << (8 + 11);
		break;
	case 0x07:
		c->enable = (data >> 7) & 1;
		if (data & 0x40)
			c->cbank = data & 7;
		else
			c->wbank = (uint16_t)((data & 0xf) << 12);
		break;
	case 0x08:
		for (int i = 0; i < 8; i++) {
			c->chan[i].enable = (~data >> i) & 1;
			if (!c->chan[i].enable)
				c->chan[i].addr = (uint32_t)c->chan[i].start << (8 + 11);
		}
		break;
	}
}

uint8_t rf5c68_mem_r(const rf5c68_t *c, int offset) { return c->ram[(c->wbank | (offset & 0xfff)) & 0xffff]; }
void rf5c68_mem_w(rf5c68_t *c, int offset, uint8_t data) { c->ram[(c->wbank | (offset & 0xfff)) & 0xffff] = data; }

static int32_t one_sample(rf5c68_t *c)
{
	c->cur_l = c->cur_r = 0;
	if (!c->enable)
		return 0;
	int32_t l = 0, r = 0;
	for (int i = 0; i < 8; i++) {
		rf5c68_chan_t *chan = &c->chan[i];
		if (!chan->enable)
			continue;
		const int lv = (chan->pan & 0x0f) * chan->env, rv = ((chan->pan >> 4) & 0x0f) * chan->env;
		int sample = c->ram[(chan->addr >> 11) & 0xffff];
		if (sample == 0xff) {
			chan->addr = (uint32_t)chan->loopst << 11;
			sample = c->ram[(chan->addr >> 11) & 0xffff];
			if (sample == 0xff)   // (a loop to a loop mark: silent)
				continue;
		}
		chan->addr += chan->step;
		if (sample & 0x80) {
			sample &= 0x7f;
			l += (sample * lv) >> 5;
			r += (sample * rv) >> 5;
		} else {
			l -= (sample * lv) >> 5;
			r -= (sample * rv) >> 5;
		}
	}
	// (10 bits out: the 6 low bits dropped, clamped)
	l &= ~0x3f;
	r &= ~0x3f;
	if (l > 32767) l = 32767;
	if (l < -32768) l = -32768;
	if (r > 32767) r = 32767;
	if (r < -32768) r = -32768;
	c->cur_l = l;
	c->cur_r = r;
	return l + r;
}

void rf5c68_render(rf5c68_t *c, int32_t *out, int n, uint32_t out_rate)
{
	const uint64_t chip_rate = c->clock / 384;   // (samples a second)
	for (int i = 0; i < n; i++) {
		c->frac += chip_rate;
		while (c->frac >= out_rate) {
			c->frac -= out_rate;
			c->cur = one_sample(c);
		}
		out[i] += c->cur;
	}
}

void rf5c68_render_lr(rf5c68_t *c, int32_t *l, int32_t *r, int n, uint32_t out_rate, int gain)
{
	const uint64_t chip_rate = c->clock / 384;
	for (int i = 0; i < n; i++) {
		c->frac += chip_rate;
		while (c->frac >= out_rate) {
			c->frac -= out_rate;
			c->cur = one_sample(c);
		}
		l[i] += c->cur_l * gain / 256;
		r[i] += c->cur_r * gain / 256;
	}
}
