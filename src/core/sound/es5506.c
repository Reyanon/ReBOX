/*  Ensoniq ES5506 (OTTO), ported from MAME src/devices/sound/es5506.cpp (BSD-3-Clause, copyright-holders: Aaron Giles).
    The arithmetic (accumulator, interpolation, filters, envelopes, volume and u-law tables) follows MAME's
    es550x_device / es5506_device. The ES5506's accumulator is 21.11 bits with no shift: MAME's 64-bit fields are
    masked to 32 bits, kept here in 32-bit ones (the same values).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "es5506.h"

#include <string.h>

#define CONTROL_BS1   0x8000
#define CONTROL_BS0   0x4000
#define CONTROL_CMPD  0x2000
#define CONTROL_IRQ   0x0080
#define CONTROL_DIR   0x0040
#define CONTROL_IRQE  0x0020
#define CONTROL_BLE   0x0010
#define CONTROL_LPE   0x0008
#define CONTROL_LEI   0x0004
#define CONTROL_STOP1 0x0002
#define CONTROL_STOP0 0x0001
#define CONTROL_LOOPMASK (CONTROL_BLE | CONTROL_LPE)
#define CONTROL_STOPMASK (CONTROL_STOP1 | CONTROL_STOP0)

#define LP3 1
#define LP4 2

#define ADDRESS_FRAC_BIT 11
#define FILTER_SHIFT 4
#define FILTER_BIT 12
#define VOLUME_BIT 16          // VOLUME_BIT_ES5506
#define VOLUME_SHIFT 4         // (total 16 - (4 exponent + 8 mantissa))
#define VOLUME_ACC_SHIFT 11    // ((16 + 15) - VOLUME_ACC_BIT 20)
#define ULAW_MAXBITS 8

static int32_t s_volume_lookup[4096];
static int32_t s_ulaw_lookup[256];
static int s_tables;

static void compute_tables(void)
{
	// compute_tables(16, 4, 8)
	for (int i = 0; i < (1 << ULAW_MAXBITS); i++) {
		const uint16_t rawval = (uint16_t)((i << (16 - ULAW_MAXBITS)) | (1 << (15 - ULAW_MAXBITS)));
		const uint8_t exponent = rawval >> 13;
		uint32_t mantissa = (uint32_t)(rawval << 3) & 0xffff;
		if (exponent == 0)
			s_ulaw_lookup[i] = (int16_t)mantissa >> 7;
		else {
			mantissa = (mantissa >> 1) | (~mantissa & 0x8000);
			s_ulaw_lookup[i] = (int16_t)mantissa >> (7 - exponent);
		}
	}
	for (int i = 0; i < 4096; i++) {
		const uint32_t exponent = (i >> 8) & 15;
		const uint32_t mantissa = (i & 255) | 256;
		s_volume_lookup[i] = (int32_t)((mantissa << 7) >> (16 - exponent));
	}
	s_tables = 1;
}

void es5506_init(es5506_t *c, int channels)
{
	if (!s_tables)
		compute_tables();
	memset(c, 0, sizeof(*c));
	c->channels = (uint8_t)(channels >= 1 && channels <= 6 ? channels : 1);
	for (int j = 0; j < 32; j++) {
		c->voice[j].control = CONTROL_STOPMASK;
		c->voice[j].lvol = 1u << (VOLUME_BIT - 1);
		c->voice[j].rvol = 1u << (VOLUME_BIT - 1);
	}
	c->irqv = 0x80;
	c->active_voices = 31;
	es5506_reset(c);
}

void es5506_reset(es5506_t *c)
{
	c->active_voices = 0x1f;
	c->mode = 0x17;
}

void es5506_set_bank(es5506_t *c, int bank, const uint8_t *base, uint32_t bytes)
{
	uint32_t w = bytes / 2;
	if (w > (1u << 21))
		w = 1u << 21;
	c->base[bank & 3] = base;
	c->words[bank & 3] = base ? w : 0;
}

static inline uint32_t integer_addr(uint32_t accum, int bias)
{
	return (accum + ((uint32_t)bias << ADDRESS_FRAC_BIT)) >> ADDRESS_FRAC_BIT;
}

static inline uint16_t read_sample(const es5506_t *c, const es5506_voice *v, uint32_t addr)
{
	const int b = (v->control >> 14) & 3;
	if (addr >= c->words[b])
		return 0;
	const uint8_t *p = c->base[b] + addr * 2;
	return (uint16_t)(p[0] << 8 | p[1]);
}

static inline int32_t interpolate(int32_t s1, int32_t s2, uint32_t accum)
{
	const uint32_t shifted = 1 << ADDRESS_FRAC_BIT;
	accum &= shifted - 1;
	return (s1 * (int32_t)(shifted - accum) + s2 * (int32_t)accum) >> ADDRESS_FRAC_BIT;
}

static inline int32_t lowpass(int32_t out, int32_t cutoff, int32_t in)
{
	return ((int32_t)(cutoff >> FILTER_SHIFT) * (out - in) / (1 << FILTER_BIT)) + in;
}

static inline int32_t highpass(int32_t out, int32_t cutoff, int32_t in, int32_t prev)
{
	return out - prev + ((int32_t)(cutoff >> FILTER_SHIFT) * in) / (1 << (FILTER_BIT + 1)) + in / 2;
}

static inline int32_t apply_filters(es5506_voice *v, int32_t s)
{
	s = lowpass(s, (int32_t)v->k1, v->o1n1);
	v->o1n1 = s;
	s = lowpass(s, (int32_t)v->k1, v->o2n1);
	v->o2n2 = v->o2n1;
	v->o2n1 = s;
	switch ((v->control >> 8) & 3) {
	case 0:
		s = highpass(s, (int32_t)v->k2, v->o3n1, v->o2n2);
		v->o3n2 = v->o3n1;
		v->o3n1 = s;
		s = highpass(s, (int32_t)v->k2, v->o4n1, v->o3n2);
		v->o4n1 = s;
		break;
	case LP3:
		s = lowpass(s, (int32_t)v->k1, v->o3n1);
		v->o3n2 = v->o3n1;
		v->o3n1 = s;
		s = highpass(s, (int32_t)v->k2, v->o4n1, v->o3n2);
		v->o4n1 = s;
		break;
	case LP4:
		s = lowpass(s, (int32_t)v->k2, v->o3n1);
		v->o3n2 = v->o3n1;
		v->o3n1 = s;
		s = lowpass(s, (int32_t)v->k2, v->o4n1);
		v->o4n1 = s;
		break;
	default:
		s = lowpass(s, (int32_t)v->k1, v->o3n1);
		v->o3n2 = v->o3n1;
		v->o3n1 = s;
		s = lowpass(s, (int32_t)v->k2, v->o4n1);
		v->o4n1 = s;
		break;
	}
	return s;
}

static inline void update_envelopes(es5506_voice *v)
{
	const uint32_t volume_max = (1u << VOLUME_BIT) - 1;
	v->ecount--;
	if (v->lvramp) {
		v->lvol += (uint32_t)(int32_t)(int8_t)v->lvramp;
		if ((int32_t)v->lvol < 0) v->lvol = 0;
		else if (v->lvol > volume_max) v->lvol = volume_max;
	}
	if (v->rvramp) {
		v->rvol += (uint32_t)(int32_t)(int8_t)v->rvramp;
		if ((int32_t)v->rvol < 0) v->rvol = 0;
		else if (v->rvol > volume_max) v->rvol = volume_max;
	}
	if (v->k1ramp && ((int32_t)v->k1ramp >= 0 || !(v->filtcount & 7))) {
		v->k1 += (uint32_t)(int32_t)(int8_t)v->k1ramp;
		if ((int32_t)v->k1 < 0) v->k1 = 0;
		else if (v->k1 > 0xffff) v->k1 = 0xffff;
	}
	if (v->k2ramp && ((int32_t)v->k2ramp >= 0 || !(v->filtcount & 7))) {
		v->k2 += (uint32_t)(int32_t)(int8_t)v->k2ramp;
		if ((int32_t)v->k2 < 0) v->k2 = 0;
		else if (v->k2 > 0xffff) v->k2 = 0xffff;
	}
	v->filtcount++;
}

static inline void check_end_forward(es5506_voice *v, uint32_t *accum)
{
	if (*accum > v->end && !(v->control & CONTROL_LEI)) {
		if (v->control & CONTROL_IRQE)
			v->control |= CONTROL_IRQ;
		switch (v->control & CONTROL_LOOPMASK) {
		case 0:
			v->control |= CONTROL_STOP0;
			break;
		case CONTROL_LPE:
			*accum = v->start + (*accum - v->end);
			break;
		case CONTROL_BLE:
			*accum = v->start + (*accum - v->end);
			v->control = (v->control & ~CONTROL_LOOPMASK) | CONTROL_LEI;
			break;
		case CONTROL_LPE | CONTROL_BLE:
			*accum = v->end - (*accum - v->end);
			v->control ^= CONTROL_DIR;
			break;
		}
	}
}

static inline void check_end_reverse(es5506_voice *v, uint32_t *accum)
{
	if (*accum < v->start && !(v->control & CONTROL_LEI)) {
		if (v->control & CONTROL_IRQE)
			v->control |= CONTROL_IRQ;
		switch (v->control & CONTROL_LOOPMASK) {
		case 0:
			v->control |= CONTROL_STOP0;
			break;
		case CONTROL_LPE:
			*accum = v->end - (v->start - *accum);
			break;
		case CONTROL_BLE:
			*accum = v->end - (v->start - *accum);
			v->control = (v->control & ~CONTROL_LOOPMASK) | CONTROL_LEI;
			break;
		case CONTROL_LPE | CONTROL_BLE:
			*accum = v->start + (v->start - *accum);
			v->control ^= CONTROL_DIR;
			break;
		}
	}
}

static inline int32_t get_sample(int32_t sample, uint32_t volume)
{
	return (int32_t)(((int64_t)sample * s_volume_lookup[volume >> VOLUME_SHIFT]) >> VOLUME_ACC_SHIFT);
}

// generate_pcm / generate_ulaw
static inline void generate(const es5506_t *c, es5506_voice *v, int32_t *dest)
{
	uint32_t accum = v->accum;
	if (!(v->control & CONTROL_STOPMASK)) {
		int32_t val1, val2;
		const uint16_t r1 = read_sample(c, v, integer_addr(accum, 0));
		const uint16_t r2 = read_sample(c, v, integer_addr(accum, 1));
		if (v->control & CONTROL_CMPD) {
			val1 = s_ulaw_lookup[r1 >> (16 - ULAW_MAXBITS)];
			val2 = s_ulaw_lookup[r2 >> (16 - ULAW_MAXBITS)];
		} else {
			val1 = (int16_t)r1;
			val2 = (int16_t)r2;
		}
		val1 = interpolate(val1, val2, accum);
		const int reverse = (v->control & CONTROL_DIR) != 0;
		accum = reverse ? accum - v->freqcount : accum + v->freqcount;
		val1 = apply_filters(v, val1);
		if (v->ecount != 0)
			update_envelopes(v);
		dest[0] += get_sample(val1, v->lvol);
		dest[1] += get_sample(val1, v->rvol);
		if (reverse)
			check_end_reverse(v, &accum);
		else
			check_end_forward(v, &accum);
	} else if (v->ecount != 0)
		update_envelopes(v);
	v->accum = accum;
}

void es5506_sample(es5506_t *c, int32_t *out)
{
	int32_t cur[12] = { 0 };
	const int chans = c->channels;
	for (int vi = 0; vi <= c->active_voices; vi++) {
		es5506_voice *v = &c->voice[vi];
		if (v->start == v->end)
			v->control |= CONTROL_STOP0;
		if ((v->control & CONTROL_STOPMASK) && !v->ecount && !(v->control & CONTROL_IRQ))
			continue;   // (nothing to do: no sound, no envelope, no interrupt waiting)
		int ch = (v->control >> 10) & 7;
		while (ch >= chans)   // (voice_channel % m_channels)
			ch -= chans;
		generate(c, v, &cur[ch << 1]);
		// generate_irq
		if ((v->control & CONTROL_IRQ) && (c->irqv & 0x80)) {
			c->irqv = (uint8_t)(vi & 0x1f);
			v->control &= ~CONTROL_IRQ;
		}
	}
	for (int i = 0; i < chans * 2; i++) {
		int32_t s = cur[i];
		out[i] = s < -(1 << 19) ? -(1 << 19) : s > (1 << 19) - 1 ? (1 << 19) - 1 : s;
	}
}

// ---- registers
static void reg_write_low(es5506_t *c, es5506_voice *v, int offset, uint32_t data)
{
	switch (offset) {
	case 0: v->control = data & 0xffff; break;
	case 1: v->freqcount = data & 0x1ffff; break;
	case 2: v->lvol = data & 0xffff; break;
	case 3: v->lvramp = (data & 0xff00) >> 8; break;
	case 4: v->rvol = data & 0xffff; break;
	case 5: v->rvramp = (data & 0xff00) >> 8; break;
	case 6: v->ecount = data & 0x1ff; v->filtcount = 0; break;
	case 7: v->k2 = data & 0xffff; break;
	case 8: v->k2ramp = ((data & 0xff00) >> 8) | ((data & 0x0001) << 31); break;
	case 9: v->k1 = data & 0xffff; break;
	case 10: v->k1ramp = ((data & 0xff00) >> 8) | ((data & 0x0001) << 31); break;
	case 11: c->active_voices = (uint8_t)(data & 0x1f); break;
	case 12: c->mode = (uint8_t)(data & 0x1f); break;
	case 15: c->current_page = (uint8_t)(data & 0x7f); break;
	}
}

static inline int32_t sext18(uint32_t d) { return (int32_t)(d << 14) >> 14; }

static void reg_write_high(es5506_t *c, es5506_voice *v, int offset, uint32_t data)
{
	switch (offset) {
	case 0: v->control = data & 0xffff; break;
	case 1: v->start = data & 0xfffff800; break;
	case 2: v->end = data & 0xffffff80; break;
	case 3: v->accum = data; break;
	case 4: v->o4n1 = sext18(data); break;
	case 5: v->o3n1 = sext18(data); break;
	case 6: v->o3n2 = sext18(data); break;
	case 7: v->o2n1 = sext18(data); break;
	case 8: v->o2n2 = sext18(data); break;
	case 9: v->o1n1 = sext18(data); break;
	case 10: c->wst = (uint8_t)(data & 0x7f); break;
	case 11: c->wend = (uint8_t)(data & 0x7f); break;
	case 12: c->lrend = (uint8_t)(data & 0x7f); break;
	case 15: c->current_page = (uint8_t)(data & 0x7f); break;
	}
}

void es5506_write(es5506_t *c, int offset, uint8_t data)
{
	es5506_voice *v = &c->voice[c->current_page & 0x1f];
	const int shift = 8 * (offset & 3);
	c->write_latch = (c->write_latch & ~(0xff000000u >> shift)) | ((uint32_t)data << (24 - shift));
	if (shift != 24)
		return;
	const int reg = (offset >> 2) & 15;
	if (c->current_page < 0x20)
		reg_write_low(c, v, reg, c->write_latch);
	else if (c->current_page < 0x40)
		reg_write_high(c, v, reg, c->write_latch);
	else if (reg == 15)   // (reg_write_test: the test registers only log)
		c->current_page = (uint8_t)(c->write_latch & 0x7f);
	c->write_latch = 0;
}

static uint32_t reg_read_low(es5506_t *c, const es5506_voice *v, int offset)
{
	switch (offset) {
	case 0: return v->control;
	case 1: return v->freqcount;
	case 2: return v->lvol;
	case 3: return v->lvramp << 8;
	case 4: return v->rvol;
	case 5: return v->rvramp << 8;
	case 6: return v->ecount;
	case 7: return v->k2;
	case 8: return (v->k2ramp << 8) | (v->k2ramp >> 31);
	case 9: return v->k1;
	case 10: return (v->k1ramp << 8) | (v->k1ramp >> 31);
	case 11: return c->active_voices;
	case 12: return c->mode;
	case 13: return 0;   // (PAR: no read_port callback here)
	case 14: { const uint32_t r = c->irqv; c->irqv = 0x80; return r; }
	case 15: return c->current_page;
	}
	return 0;
}

static uint32_t reg_read_high(es5506_t *c, const es5506_voice *v, int offset)
{
	switch (offset) {
	case 0: return v->control;
	case 1: return v->start;
	case 2: return v->end;
	case 3: return v->accum;
	case 4: return (uint32_t)v->o4n1 & 0x3ffff;
	case 5: return (uint32_t)v->o3n1 & 0x3ffff;
	case 6: return (uint32_t)v->o3n2 & 0x3ffff;
	case 7: return (uint32_t)v->o2n1 & 0x3ffff;
	case 8: return (uint32_t)v->o2n2 & 0x3ffff;
	case 9: return (uint32_t)v->o1n1 & 0x3ffff;
	case 10: return c->wst;
	case 11: return c->wend;
	case 12: return c->lrend;
	case 13: return 0;
	case 14: { const uint32_t r = c->irqv; c->irqv = 0x80; return r; }
	case 15: return c->current_page;
	}
	return 0;
}

uint8_t es5506_read(es5506_t *c, int offset)
{
	const es5506_voice *v = &c->voice[c->current_page & 0x1f];
	const int shift = 8 * (offset & 3);
	if (shift != 0)
		return (uint8_t)(c->read_latch >> (24 - shift));
	const int reg = (offset >> 2) & 15;
	if (c->current_page < 0x20)
		c->read_latch = reg_read_low(c, v, reg);
	else if (c->current_page < 0x40)
		c->read_latch = reg_read_high(c, v, reg);
	else
		c->read_latch = reg == 14 ? c->irqv : reg == 15 ? c->current_page : 0;
	return (uint8_t)(c->read_latch >> 24);
}
