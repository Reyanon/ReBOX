/*  ICS2115 WaveFront synthesizer, ported from MAME src/devices/sound/ics2115.cpp (BSD-3-Clause, copyright-holders:
    Alex Marshall, nimitz, austere). See ics2115.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "ics2115.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

// osc_conf bits
#define OC_ULAW    0x01
#define OC_STOP    0x02
#define OC_8BIT    0x04
#define OC_LOOP    0x08
#define OC_BIDIR   0x10
#define OC_IRQ     0x20
#define OC_INVERT  0x40
#define OC_PENDING 0x80
// vol_ctrl bits
#define VC_DONE    0x01
#define VC_STOP    0x02
#define VC_LOOP    0x08
#define VC_BIDIR   0x10
#define VC_IRQ     0x20
#define VC_INVERT  0x40
#define VC_PENDING 0x80

static int16_t s_ulaw[256];
static uint16_t s_volume[4096];
static uint16_t s_panlaw[256];
static uint32_t s_volinc_frac[32];
static int s_tables;

static void make_tables(void)
{
	if (s_tables)
		return;
	s_tables = 1;
	for (int i = 0; i < 4096; i++) {   // hardware measured formula
		const int exponent = i >> 8, mantissa = i & 0xff;
		s_volume[i] = exponent == 0 ? (uint16_t)(mantissa >> 7)
		                            : (uint16_t)((((0x100 | mantissa) << (exponent - 1)) + 0xff) >> 8);
	}
	uint16_t lut[8];
	const uint16_t lut_initial = 33 << 2;
	for (int i = 0; i < 8; i++)
		lut[i] = (uint16_t)((lut_initial << i) - lut_initial);
	for (unsigned i = 0; i < 256; i++) {
		const int exponent = (~i >> 4) & 0x07, mantissa = ~i & 0x0f;
		const int16_t value = (int16_t)(lut[exponent] + (mantissa << (exponent + 3)));
		s_ulaw[i] = (i & 0x80) ? (int16_t)-value : value;
		int bw = 0;   // std::bit_width(i)
		for (unsigned v = i; v; v >>= 1)
			bw++;
		s_panlaw[i] = (uint16_t)(16 - (bw - 1));
	}
	s_panlaw[0] = 0xfff;
	for (int i = 0; i < 32; i++)
		s_volinc_frac[i] = (uint16_t)lround(1024.0 * pow(2.0, i / 32.0));
}

int ics2115_init(ics2115_t *c, const uint8_t *rom, uint32_t rom_size, void (*irq_cb)(int), int out_cap)
{
	make_tables();
	memset(c, 0, sizeof(*c));
	c->rom = rom;
	c->rom_size = rom_size > 0x1000000 ? 0x1000000 : rom_size;
	c->irq_cb = irq_cb;
	c->out = (int32_t *)malloc(sizeof(int32_t) * (size_t)out_cap);
	c->out_cap = out_cap;
	if (!c->out)
		return -1;
	ics2115_reset(c);
	return 0;
}

void ics2115_exit(ics2115_t *c)
{
	free(c->out);
	c->out = NULL;
}

static void recalc_irq(ics2115_t *c)
{
	int irq = (c->irq_pending & c->irq_enabled) != 0;
	for (int i = 0; !irq && i < 32; i++) {
		const ics2115_voice *v = &c->voice[i];
		irq |= (v->osc_conf & OC_IRQ) && (v->osc_conf & OC_PENDING);
		irq |= (v->vol_ctrl & VC_IRQ) && (v->vol_ctrl & VC_PENDING);
	}
	c->irq_on = (uint8_t)irq;
	if (c->irq_cb)
		c->irq_cb(irq);
}

void ics2115_reset(ics2115_t *c)
{
	c->irq_enabled = 0;
	c->irq_pending = 0;
	c->active_osc = 31;
	c->osc_select = 0;
	c->reg_select = 0;
	c->irq_on = 0;
	memset(c->voice, 0, sizeof(c->voice));
	for (int i = 0; i < 2; i++) {
		c->timer[i].period = 0;
		c->timer[i].left = 0;
		c->timer[i].scale = 0;
		c->timer[i].preset = 0;
	}
	for (int i = 0; i < 32; i++) {
		ics2115_voice *v = &c->voice[i];
		v->osc_conf = 2;
		v->vol.pan = 0x7f;
		v->vol_ctrl = 1;
	}
	c->tick_frac = 0;
	c->out_n = 0;
}

static int update_volume_envelope(ics2115_voice *v)
{
	int bc = 1;   // (MAME: acc >= end || acc <= end, always)
	int ret = 0;
	if (v->vol_ctrl & (VC_DONE | VC_STOP))
		return ret;
	if (v->vol_ctrl & VC_INVERT) {
		v->vol.acc -= v->vol.add;
		v->vol.left = v->vol.acc - v->vol.start;
	} else {
		v->vol.acc += v->vol.add;
		v->vol.left = v->vol.end - v->vol.acc;
	}
	if (v->vol.left > 0)
		return ret;
	if (v->vol_ctrl & VC_IRQ) {
		v->vol_ctrl |= VC_PENDING;
		ret = 1;
	}
	if (v->osc_conf & OC_8BIT)
		return ret;
	if (v->vol_ctrl & VC_LOOP) {
		if (bc) {
			if (!(v->vol_ctrl & VC_BIDIR)) {
				if (!(v->vol_ctrl & VC_INVERT))
					v->vol.acc = v->vol.start - (v->vol.end - (v->vol.acc + v->vol.incr));
				else
					v->vol.acc = v->vol.end + ((v->vol.acc - v->vol.incr) - v->vol.start);
			} else {
				if (!(v->vol_ctrl & VC_INVERT))
					v->vol.acc = v->vol.end + (v->vol.end - (v->vol.acc + v->vol.incr));
				else
					v->vol.acc = v->vol.start - ((v->vol.acc - v->vol.incr) - v->vol.start);
			}
		}
	} else
		v->vol_ctrl |= VC_DONE;
	return ret;
}

static int update_oscillator(ics2115_voice *v)
{
	int ret = 0;
	if (v->osc_conf & OC_STOP)
		return ret;
	if (v->osc_conf & OC_INVERT) {
		v->osc.acc -= (uint32_t)v->osc.fc << 2;
		v->osc.left = (int32_t)(v->osc.acc - v->osc.start);
	} else {
		v->osc.acc += (uint32_t)v->osc.fc << 2;
		v->osc.left = (int32_t)(v->osc.end - v->osc.acc);
	}
	if (v->osc.left > 0)
		return ret;
	if (v->osc_conf & OC_IRQ) {
		v->osc_conf |= OC_PENDING;
		ret = 1;
	}
	if (v->osc_conf & OC_LOOP) {
		if (v->osc_conf & OC_BIDIR)
			v->osc_conf ^= OC_INVERT;
		if (v->osc_conf & OC_INVERT) {
			v->osc.acc = v->osc.end + (uint32_t)v->osc.left;
			v->osc.left = (int32_t)(v->osc.acc - v->osc.start);
		} else {
			v->osc.acc = v->osc.start - (uint32_t)v->osc.left;
			v->osc.left = (int32_t)(v->osc.end - v->osc.acc);
		}
	} else {
		v->on = 0;
		v->osc_conf |= OC_STOP;
		v->osc.acc = (v->osc_conf & OC_INVERT) ? v->osc.start : v->osc.end;
	}
	return ret;
}

static inline uint8_t read_sample(const ics2115_t *c, const ics2115_voice *v, uint32_t addr)
{
	const uint32_t a = (((uint32_t)v->osc.saddr << 20) | (addr & 0xfffff)) & 0xffffff;
	return a < c->rom_size ? c->rom[a] : 0;
}

static inline int32_t get_sample(const ics2115_t *c, const ics2115_voice *v)
{
	const uint32_t curaddr = v->osc.acc >> 12;
	uint32_t nextaddr;
	if (v->on && (v->osc_conf & OC_LOOP) && !(v->osc_conf & OC_BIDIR) && (v->osc.left < (int32_t)((uint32_t)v->osc.fc << 2)))
		nextaddr = v->osc.start >> 12;
	else
		nextaddr = curaddr + 2;
	int32_t s1, s2;
	if (v->osc_conf & OC_ULAW) {
		s1 = s_ulaw[read_sample(c, v, curaddr)];
		s2 = s_ulaw[read_sample(c, v, curaddr + 1)];
	} else if (v->osc_conf & OC_8BIT) {
		s1 = (int8_t)read_sample(c, v, curaddr) * 256;
		s2 = (int8_t)read_sample(c, v, curaddr + 1) * 256;
	} else {
		s1 = read_sample(c, v, curaddr) | ((int8_t)read_sample(c, v, curaddr + 1) * 256);
		s2 = read_sample(c, v, nextaddr) | ((int8_t)read_sample(c, v, nextaddr + 1) * 256);
	}
	const int32_t diff = s2 - s1;
	const int32_t fract = (int32_t)((v->osc.acc & 0xff8) >> 3);
	return ((s1 * 512) + (diff * fract)) >> 9;
}

static inline int playing(const ics2115_voice *v) { return v->on && !(v->osc_conf & OC_STOP); }

// n samples of every voice, a voice at a time as MAME's sound_stream_update
static void make_samples(ics2115_t *c, int n)
{
	if (c->out_n + n > c->out_cap)
		n = c->out_cap - c->out_n;
	if (n <= 0)
		return;
	int32_t *out = c->out + c->out_n;
	memset(out, 0, sizeof(int32_t) * (size_t)n);
	c->out_n += n;
	int irq_invalid = 0;
	for (int osc = 0; osc <= c->active_osc; osc++) {
		ics2115_voice *v = &c->voice[osc];
		switch (v->vol.mode & 3) {
		case 2:
			v->vol.add = v->vol.incr << 10;
			break;
		default: {
			const unsigned e = v->vol.incr + ((v->vol.mode & 1) ? 256 : 0);
			v->vol.add = (int32_t)((s_volinc_frac[e & 0x1f] << (e >> 5)) >> 10);
			break;
		}
		}
		for (int i = 0; i < n && playing(v); i++) {
			const int32_t volacc = (v->vol.acc >> 14) & 0xfff;
			const int32_t vl = volacc - s_panlaw[255 - v->vol.pan];
			const int32_t vr = volacc - s_panlaw[v->vol.pan];
			const int32_t vleft = vl > 0 ? s_volume[vl] : 0;
			const int32_t vright = vr > 0 ? s_volume[vr] : 0;
			const int32_t smp = get_sample(c, v);
			out[i] += ((smp * vleft) >> 15) + ((smp * vright) >> 15);
			if (update_oscillator(v))
				irq_invalid = 1;
			if (update_volume_envelope(v))
				irq_invalid = 1;
		}
	}
	if (irq_invalid)
		recalc_irq(c);
}

static void timer_fire(ics2115_t *c, int t)
{
	if (!(c->irq_pending & (1 << t))) {
		c->irq_pending |= (uint8_t)(1 << t);
		recalc_irq(c);
	}
}

uint64_t ics2115_next_event(const ics2115_t *c)
{
	uint64_t n = UINT64_MAX;
	for (int t = 0; t < 2; t++)
		if (c->timer[t].period && c->timer[t].left < n)
			n = c->timer[t].left;
	return n;
}

void ics2115_advance(ics2115_t *c, uint32_t ticks)
{
	while (ticks) {
		uint32_t step = ticks;
		for (int t = 0; t < 2; t++)
			if (c->timer[t].period && c->timer[t].left < step)
				step = (uint32_t)c->timer[t].left;
		if (step) {
			c->tick_frac += step;
			const uint32_t st = ics2115_sample_ticks(c);
			if (c->tick_frac >= st) {
				const int n = (int)(c->tick_frac / st);
				c->tick_frac -= (uint32_t)n * st;
				make_samples(c, n);
			}
		}
		for (int t = 0; t < 2; t++) {
			if (!c->timer[t].period)
				continue;
			c->timer[t].left -= step;
			if (!c->timer[t].left) {
				c->timer[t].left = c->timer[t].period;
				timer_fire(c, t);
			}
		}
		ticks -= step;
	}
}

static void recalc_timer(ics2115_t *c, int t)
{
	uint64_t period = (uint64_t)((c->timer[t].scale & 0x1f) + 1) * (c->timer[t].preset + 1);
	period <<= 4 + (c->timer[t].scale >> 5);
	if (c->timer[t].period != period) {
		c->timer[t].period = period;
		c->timer[t].left = period;   // (MAME: adjust(tp, timer, tp), from now)
	}
}

static uint16_t reg_read(ics2115_t *c)
{
	uint16_t ret = 0;
	ics2115_voice *v = &c->voice[c->osc_select];
	switch (c->reg_select) {
	case 0x00:
		ret = v->osc_conf;
		if (v->on)
			ret |= 8;
		else
			ret &= ~8;
		ret <<= 8;
		break;
	case 0x01: ret = v->osc.fc; break;
	case 0x02: ret = (uint16_t)(v->osc.start >> 16); break;
	case 0x03: ret = (uint16_t)(v->osc.start & 0xff00); break;
	case 0x04: ret = (uint16_t)(v->osc.end >> 16); break;
	case 0x05: ret = (uint16_t)(v->osc.end & 0xff00); break;
	case 0x06: ret = v->vol.incr; break;
	case 0x07: ret = (uint16_t)(v->vol.start >> 18); break;
	case 0x08: ret = (uint16_t)(v->vol.end >> 18); break;
	case 0x09: ret = (uint16_t)(v->vol.acc >> 10); break;
	case 0x0a: ret = (uint16_t)(v->osc.acc >> 16); break;
	case 0x0b: ret = (uint16_t)(v->osc.acc & 0xfff8); break;
	case 0x0c: ret = (uint16_t)(v->vol.pan << 8); break;
	case 0x0d:
		ret |= 0x01;
		if (v->vol_ctrl & VC_IRQ)
			ret |= 0x80;
		ret <<= 8;
		break;
	case 0x0e: ret = c->active_osc; break;
	case 0x0f:
		ret = 0xff;
		for (int i = 0; i <= c->active_osc; i++) {
			ics2115_voice *w = &c->voice[i];
			if ((w->osc_conf & OC_PENDING) || (w->vol_ctrl & VC_PENDING)) {
				ret = (uint16_t)(i | 0xe0);
				if (w->osc_conf & OC_PENDING) {
					w->osc_conf &= ~OC_PENDING;
					ret &= ~0x80;
				}
				if (w->vol_ctrl & VC_PENDING) {
					w->vol_ctrl &= ~VC_PENDING;
					ret &= ~0x40;
				}
				recalc_irq(c);
				break;
			}
		}
		ret <<= 8;
		break;
	case 0x10: ret = (uint16_t)(v->osc.ctl << 8); break;
	case 0x11: ret = (uint16_t)(v->osc.saddr << 8); break;
	case 0x40:
	case 0x41:
		ret = c->timer[c->reg_select & 1].preset;
		c->irq_pending &= (uint8_t)~(1 << (c->reg_select & 1));
		recalc_irq(c);
		break;
	case 0x43: ret = c->irq_pending & 3; break;
	case 0x4a: ret = c->irq_pending; break;
	case 0x4b: ret = 0x80; break;
	case 0x4c: ret = 1; break;   // revision
	default: ret = 0; break;
	}
	return ret;
}

static void reg_write(ics2115_t *c, uint16_t data, uint16_t mask)
{
	ics2115_voice *v = &c->voice[c->osc_select];
	if (c->reg_select < 0x20)
		v->regs[c->reg_select] = (uint16_t)((v->regs[c->reg_select] & ~mask) | (data & mask));
	else if (c->reg_select >= 0x40 && c->reg_select < 0x80)
		c->regs[c->reg_select - 0x40] = (uint16_t)((c->regs[c->reg_select - 0x40] & ~mask) | (data & mask));
	const int hi = (mask & 0xff00) != 0, lo = (mask & 0x00ff) != 0;
	switch (c->reg_select) {
	case 0x00:
		if (hi) {
			v->osc_conf &= 0x80;
			v->osc_conf |= (data >> 8) & 0x7f;
			recalc_irq(c);
		}
		break;
	case 0x01:
		if (hi) v->osc.fc = (uint16_t)((v->osc.fc & 0x00fe) | (data & 0xff00));
		if (lo) v->osc.fc = (uint16_t)((v->osc.fc & 0xff00) | (data & 0x00fe));
		break;
	case 0x02:
		if (hi) v->osc.start = (v->osc.start & 0x00ffffff) | ((uint32_t)(data & 0xff00) << 16);
		if (lo) v->osc.start = (v->osc.start & 0xff00ffff) | ((uint32_t)(data & 0x00ff) << 16);
		break;
	case 0x03:
		if (hi) v->osc.start = (v->osc.start & 0xffff00ff) | (data & 0xff00);
		break;
	case 0x04:
		if (hi) v->osc.end = (v->osc.end & 0x00ffffff) | ((uint32_t)(data & 0xff00) << 16);
		if (lo) v->osc.end = (v->osc.end & 0xff00ffff) | ((uint32_t)(data & 0x00ff) << 16);
		break;
	case 0x05:
		if (hi) v->osc.end = (v->osc.end & 0xffff00ff) | (data & 0xff00);
		break;
	case 0x06:
		if (hi) v->vol.incr = (uint8_t)(data >> 8);
		break;
	case 0x07:
		if (lo) v->vol.start = (data & 0xff) << 18;
		break;
	case 0x08:
		if (lo) v->vol.end = (data & 0xff) << 18;
		break;
	case 0x09:
		if (hi) v->vol.regacc = (uint16_t)((v->vol.regacc & 0x00ff) | (data & 0xff00));
		if (lo) v->vol.regacc = (uint16_t)((v->vol.regacc & 0xff00) | (data & 0x00ff));
		v->vol.acc = v->vol.regacc << 10;
		break;
	case 0x0a:
		if (hi) v->osc.acc = (v->osc.acc & 0x00ffffff) | ((uint32_t)(data & 0xff00) << 16);
		if (lo) v->osc.acc = (v->osc.acc & 0xff00ffff) | ((uint32_t)(data & 0x00ff) << 16);
		break;
	case 0x0b:
		if (hi) v->osc.acc = (v->osc.acc & 0xffff00ff) | (data & 0xff00);
		if (lo) v->osc.acc = (v->osc.acc & 0xffffff00) | (data & 0x00f8);
		break;
	case 0x0c:
		if (hi) v->vol.pan = (uint8_t)(data >> 8);
		break;
	case 0x0d:
		if (hi) {
			v->vol_ctrl &= 0x80;
			v->vol_ctrl |= (data >> 8) & 0x7f;
			recalc_irq(c);
		}
		break;
	case 0x0e:
		if (hi)
			c->active_osc = (data >> 8) & 0x1f;
		break;
	case 0x10:
		if (hi) {
			data >>= 8;
			v->osc.ctl = (uint8_t)data;
			v->on = !v->osc.ctl;
			if (data == 0xf) {
				v->osc_conf |= OC_STOP;
				v->vol_ctrl |= VC_STOP;
			}
		}
		break;
	case 0x11:
		if (hi) v->osc.saddr = (uint8_t)(data >> 8);
		break;
	case 0x12:
		if (hi) v->vol.mode = (uint8_t)(data >> 8);
		break;
	case 0x40:
	case 0x41:
		if (lo) {
			c->timer[c->reg_select & 1].preset = (uint8_t)data;
			recalc_timer(c, c->reg_select & 1);
		}
		break;
	case 0x42:
	case 0x43:
		if (lo) {
			c->timer[c->reg_select & 1].scale = (uint8_t)data;
			recalc_timer(c, c->reg_select & 1);
		}
		break;
	case 0x4a:
		if (lo) {
			c->irq_enabled = (uint8_t)data;
			recalc_irq(c);
		}
		break;
	case 0x4f:
		if (lo)
			c->osc_select = (uint8_t)((data & 0xff) % (1 + c->active_osc));
		break;
	default:
		break;
	}
}

uint8_t ics2115_read(ics2115_t *c, int offset)
{
	uint8_t ret = 0;
	switch (offset) {
	case 0:
		if (c->irq_on) {
			ret |= 0x80;
			if (c->irq_enabled && (c->irq_pending & 3))
				ret |= 1;
			for (int i = 0; i <= c->active_osc; i++)
				if (c->voice[i].osc_conf & OC_PENDING) {
					ret |= 2;
					break;
				}
		}
		break;
	case 1: ret = c->reg_select; break;
	case 2: ret = (uint8_t)reg_read(c); break;
	case 3: ret = (uint8_t)(reg_read(c) >> 8); break;
	}
	return ret;
}

void ics2115_write(ics2115_t *c, int offset, uint8_t data)
{
	switch (offset) {
	case 1: c->reg_select = data; break;
	case 2: reg_write(c, data, 0x00ff); break;
	case 3: reg_write(c, (uint16_t)(data << 8), 0xff00); break;
	}
}
