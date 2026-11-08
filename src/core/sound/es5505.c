/*  Ensoniq ES5505 (OTIS), ported from MAME src/devices/sound/es5506.cpp (BSD-3-Clause, copyright-holders: Aaron Giles).
    The arithmetic (accumulator, interpolation, filters, volume table) follows the MAME es550x_device / es5505_device.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "es5505.h"

#include <string.h>

#define CONTROL_IRQ      0x0080
#define CONTROL_DIR      0x0040
#define CONTROL_IRQE     0x0020
#define CONTROL_BLE      0x0010
#define CONTROL_LPE      0x0008
#define CONTROL_STOPMASK 0x0003
#define CONTROL_LOOPMASK (CONTROL_BLE | CONTROL_LPE)
#define CONTROL_STOP0    0x0001

#define LP3 1
#define LP4 2

// MAME: ADDRESS_FRAC_BIT 11, ES5505 20 integer + 9 fraction bits -> accumulator shift 2, mask 31 bits
#define ADDRESS_FRAC_BIT 11
#define ACC_SHIFT 2
#define ACC_MASK 0x7fffffffull
#define FILTER_SHIFT 4
#define FILTER_BIT 12
#define VOLUME_ACC_SHIFT 11

static uint32_t s_volume_lookup[256];
static int s_tables;

static void compute_tables(void)
{
	// compute_tables(8, 4, 4): 4 bit exponent, 4 bit mantissa
	for (int i = 0; i < 256; i++) {
		uint32_t exponent = (i >> 4) & 15;
		uint32_t mantissa = (i & 15) | 16;
		s_volume_lookup[i] = (mantissa << 11) >> (16 - exponent);
	}
	s_tables = 1;
}

void es5505_init(es5505_t *c, const uint8_t *rom, uint32_t rom_bytes)
{
	if (!s_tables)
		compute_tables();
	memset(c, 0, sizeof(*c));
	c->rom = rom;
	uint32_t words = rom_bytes / 2, m = 1;
	while (m < words)
		m <<= 1;
	c->rom_mask = m - 1;
	es5505_reset(c);
}

void es5505_reset(es5505_t *c)
{
	// MAME: device_start state (the ES5505 has no reset handler there)
	for (int j = 0; j < 32; j++) {
		memset(&c->voice[j], 0, sizeof(c->voice[j]));
		c->voice[j].control = CONTROL_STOPMASK;
		c->voice[j].lvol = 1 << 7;
		c->voice[j].rvol = 1 << 7;
	}
	c->current_page = 0;
	c->active_voices = 0x1f;
	c->irqv = 0x80;
	c->mode = 0;
	c->irq = 0;
	memset(c->bank, 0, sizeof(c->bank));
	if (c->ctl)
		es5505_ctl_begin(c);
}

static inline uint16_t read_sample(const es5505_t *c, int v, uint32_t addr)
{
	uint32_t a = (c->bank[v] + addr) & c->rom_mask;
	const uint8_t *p = c->rom + a * 2;
	return (uint16_t)(p[0] << 8 | p[1]);
}

// (the accumulator and the addresses are 31 bits wide, ACC_MASK: 32-bit arithmetic gives the same low 31 bits as the
// 64-bit fields they are kept in, at a fraction of the cost on 32-bit ARM)
static inline uint32_t integer_addr(uint32_t accum, int bias)
{
	return ((accum + ((uint32_t)bias << ADDRESS_FRAC_BIT)) & (uint32_t)ACC_MASK) >> ADDRESS_FRAC_BIT;
}

static inline int32_t interpolate(int32_t s1, int32_t s2, uint32_t accum)
{
	const uint32_t shifted = 1 << ADDRESS_FRAC_BIT;
	accum &= (shifted - 1) & ACC_MASK;
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

static inline void apply_filters(es5505_voice *v, int32_t *sample)
{
	int32_t s = *sample;
	s = lowpass(s, (int32_t)v->k1, v->o1n1);
	v->o1n1 = s;
	s = lowpass(s, (int32_t)v->k1, v->o2n1);
	v->o2n2 = v->o2n1;
	v->o2n1 = s;
	switch ((v->control >> 10) & 3) {
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
	*sample = s;
}

static inline int64_t get_sample(int32_t sample, uint32_t volume)
{
	return ((int64_t)sample * s_volume_lookup[volume & 0xff]) >> VOLUME_ACC_SHIFT;
}

// ES5505: BLE is ignored when LPE = 0
static inline void check_end_forward(es5505_voice *v, uint32_t *accum)
{
	const uint32_t start = (uint32_t)v->start, end = (uint32_t)v->end;
	if (*accum > end) {
		if (v->control & CONTROL_IRQE)
			v->control |= CONTROL_IRQ;
		switch (v->control & CONTROL_LOOPMASK) {
		case 0:
		case CONTROL_BLE:
			v->control |= CONTROL_STOP0;
			break;
		case CONTROL_LPE:
			*accum = (start + (*accum - end)) & (uint32_t)ACC_MASK;
			break;
		case CONTROL_LPE | CONTROL_BLE:
			*accum = (end - (*accum - end)) & (uint32_t)ACC_MASK;
			v->control ^= CONTROL_DIR;
			break;
		}
	}
}

static inline void check_end_reverse(es5505_voice *v, uint32_t *accum)
{
	const uint32_t start = (uint32_t)v->start, end = (uint32_t)v->end;
	if (*accum < start) {
		if (v->control & CONTROL_IRQE)
			v->control |= CONTROL_IRQ;
		switch (v->control & CONTROL_LOOPMASK) {
		case 0:
		case CONTROL_BLE:
			v->control |= CONTROL_STOP0;
			break;
		case CONTROL_LPE:
			*accum = (end - (start - *accum)) & (uint32_t)ACC_MASK;
			break;
		case CONTROL_LPE | CONTROL_BLE:
			*accum = (start + (start - *accum)) & (uint32_t)ACC_MASK;
			v->control ^= CONTROL_DIR;
			break;
		}
	}
}

static inline void generate_pcm(es5505_t *c, int vi, int32_t *dest)
{
	es5505_voice *v = &c->voice[vi];
	uint32_t accum = (uint32_t)v->accum & (uint32_t)ACC_MASK;
	const uint32_t control = v->control;
	if (!(control & CONTROL_STOPMASK)) {
		const uint32_t bank = c->bank[vi], mask = c->rom_mask, freq = (uint32_t)v->freqcount;
		const uint32_t a1 = integer_addr(accum, 0);
		// (integer_addr(accum, 1) is the next word: the 20-bit integer part plus one, wrapped)
		const uint8_t *p1 = c->rom + (((bank + a1) & mask) << 1), *p2 = c->rom + (((bank + ((a1 + 1) & 0xfffff)) & mask) << 1);
		int32_t val1 = (int16_t)(p1[0] << 8 | p1[1]);
		int32_t val2 = (int16_t)(p2[0] << 8 | p2[1]);
		val1 = interpolate(val1, val2, accum);
		if (!(control & CONTROL_DIR)) {
			accum = (accum + freq) & (uint32_t)ACC_MASK;
			// (the samples this voice reads 16 steps on, into the cache meanwhile: the sample ROM is mostly cache misses)
			__builtin_prefetch(c->rom + (((bank + integer_addr(accum + freq * 16, 0)) & mask) << 1));
		} else {
			accum = (accum - freq) & (uint32_t)ACC_MASK;
			__builtin_prefetch(c->rom + (((bank + integer_addr(accum - freq * 16, 0)) & mask) << 1));
		}
		apply_filters(v, &val1);
		dest[0] += (int32_t)get_sample(val1, v->lvol);
		dest[1] += (int32_t)get_sample(val1, v->rvol);
		if (!(control & CONTROL_DIR))
			check_end_forward(v, &accum);
		else
			check_end_reverse(v, &accum);
	}
	v->accum = accum;
}

#if defined(__arm__)
__attribute__((target("arm")))   // (ARM rather than Thumb-2 code, as the ES5510 loop)
#endif
void es5505_sample(es5505_t *c, int32_t out[8])
{
	int32_t cur[8] = { 0 };
	for (int v = 0; v <= c->active_voices; v++) {
		es5505_voice *voice = &c->voice[v];
		int l = ((voice->control >> 8) & 3) << 1;
		generate_pcm(c, v, &cur[l]);
		if (voice->control & CONTROL_IRQ) {
			if (c->irqv & 0x80) {
				c->irqv = (uint8_t)(v & 0x1f);
				voice->control &= ~CONTROL_IRQ;
				c->irq = 1;
			}
		}
	}
	for (int i = 0; i < 8; i++) {
		int32_t s = cur[i];
		if (s > (1 << 19) - 1) s = (1 << 19) - 1;
		if (s < -(1 << 19)) s = -(1 << 19);
		out[i] = s;
	}
}

// es5505_sample without the sound: only what the voices' controls, accumulators and the interrupt vector do (the
// same steps as generate_pcm and es5505_sample; neither depends on the samples or the filters). Between its ends a
// playing voice's accumulator only moves by its frequency each sample, so it is brought up to date only when the host
// reads or writes the voice (ctl_sync) and at the sample of its next end (ctl_ev: the first sample its accumulator is
// past the end, or before the start backwards; a wrap of the 31 bits can only come at that sample too), where the
// sample's step is done as before. The interrupt vector takes the first voice with IRQ in order, as the loop did.
#define CTL_NEVER (~0ull)
static void ctl_sync(es5505_t *c, int vi, uint64_t to)
{
	es5505_voice *v = &c->voice[vi];
	if (vi <= c->active_voices && !(v->control & CONTROL_STOPMASK)) {
		const uint32_t d = (uint32_t)((to - c->ctl_t[vi]) * (uint64_t)(uint32_t)v->freqcount);
		const uint32_t a = (uint32_t)v->accum;
		v->accum = (v->control & CONTROL_DIR ? a - d : a + d) & (uint32_t)ACC_MASK;
	}
	c->ctl_t[vi] = to;
}

static void ctl_event(es5505_t *c, int vi)   // (the voice synced to ctl_now: its next end)
{
	es5505_voice *v = &c->voice[vi];
	uint64_t ev = CTL_NEVER;
	if (vi <= c->active_voices && !(v->control & CONTROL_STOPMASK)) {
		const uint32_t a = (uint32_t)v->accum & (uint32_t)ACC_MASK, f = (uint32_t)v->freqcount;
		const uint32_t start = (uint32_t)v->start, end = (uint32_t)v->end;
		if (!(v->control & CONTROL_DIR))
			ev = a > end ? 1 : f ? (uint64_t)((end - a) / f) + 1 : CTL_NEVER;
		else
			ev = a < start ? 1 : f ? (uint64_t)((a - start) / f) + 1 : CTL_NEVER;
		if (ev != CTL_NEVER)
			ev += c->ctl_now;
	}
	c->ctl_ev[vi] = ev;
	if (ev < c->ctl_next)
		c->ctl_next = ev;
}

void es5505_ctl_begin(es5505_t *c)
{
	c->ctl = 1;
	c->ctl_now = 0;
	c->ctl_next = CTL_NEVER;
	c->ctl_irq = 1;
	c->ctl_dirty = 0;
	for (int vi = 0; vi < 32; vi++) {
		c->voice[vi].accum = (uint32_t)c->voice[vi].accum & (uint32_t)ACC_MASK;
		c->ctl_t[vi] = 0;
		ctl_event(c, vi);
	}
}

void es5505_sample_ctl(es5505_t *c)
{
	while (c->ctl_dirty) {   // (the voices the host wrote: their ends, from where they are now)
		const int vi = __builtin_ctz(c->ctl_dirty);
		c->ctl_dirty &= c->ctl_dirty - 1;
		ctl_event(c, vi);
	}
	const uint64_t now = ++c->ctl_now;
	if (now < c->ctl_next && !(c->ctl_irq && (c->irqv & 0x80)))
		return;
	uint64_t next = CTL_NEVER;
	int irq = 0;
	for (int vi = 0; vi <= c->active_voices; vi++) {
		es5505_voice *v = &c->voice[vi];
		if (c->ctl_ev[vi] <= now) {   // (the step of this sample)
			ctl_sync(c, vi, now - 1);
			uint32_t accum = (uint32_t)v->accum & (uint32_t)ACC_MASK;
			if (!(v->control & CONTROL_STOPMASK)) {
				if (!(v->control & CONTROL_DIR)) {
					accum = (accum + (uint32_t)v->freqcount) & (uint32_t)ACC_MASK;
					check_end_forward(v, &accum);
				} else {
					accum = (accum - (uint32_t)v->freqcount) & (uint32_t)ACC_MASK;
					check_end_reverse(v, &accum);
				}
			}
			v->accum = accum;
			c->ctl_t[vi] = now;
			c->ctl_next = CTL_NEVER;
			ctl_event(c, vi);
		}
		if (c->ctl_ev[vi] < next)
			next = c->ctl_ev[vi];
		if (v->control & CONTROL_IRQ) {
			if (c->irqv & 0x80) {
				c->irqv = (uint8_t)(vi & 0x1f);
				v->control &= ~CONTROL_IRQ;
				c->irq = 1;
			} else
				irq = 1;
		}
	}
	c->ctl_next = next;
	c->ctl_irq = irq;
}

// the registers a read gives from the filters (O4(n-1) .. O2(n-2), O1(n-1) of a playing voice): not kept by
// es5505_sample_ctl
int es5505_read_needs_sound(const es5505_t *c, int offset)
{
	offset &= 0x0f;
	if (c->current_page < 0x20 || c->current_page >= 0x40 || offset < 1 || offset > 6)
		return 0;
	return offset < 6 || !(c->voice[c->current_page & 0x1f].control & CONTROL_STOPMASK);
}

// ---- registers ----
#define SHV(val, bias) ((uint64_t)(val) << (ACC_SHIFT - (bias)))
#define RES(val, bias) ((uint64_t)(val) >> (ACC_SHIFT - (bias)))
#define LO (mem_mask & 0x00ff)
#define HI (mem_mask & 0xff00)

static void set_active(es5505_t *c, uint16_t data)
{
	c->active_voices = data & 0x1f;
}

static void write_low(es5505_t *c, es5505_voice *v, int offset, uint16_t data, uint16_t mem_mask)
{
	switch (offset) {
	case 0x00:
		v->control |= 0xf000;
		if (LO) {
			v->control &= ~0x00ffu;
			v->control |= data & 0x00ff;
		}
		if (HI)
			v->control = (v->control & ~0x0f00u) | (data & 0x0f00);
		break;
	case 0x01:
		if (LO) v->freqcount = (v->freqcount & ~SHV(0x00fe, 1)) | SHV(data & 0x00fe, 1);
		if (HI) v->freqcount = (v->freqcount & ~SHV(0xff00, 1)) | SHV(data & 0xff00, 1);
		break;
	case 0x02:
		if (LO) v->start = (v->start & ~SHV(0x00ff0000, 0)) | SHV((uint32_t)(data & 0x00ff) << 16, 0);
		if (HI) v->start = (v->start & ~SHV(0x1f000000, 0)) | SHV((uint32_t)(data & 0x1f00) << 16, 0);
		break;
	case 0x03:
		if (LO) v->start = (v->start & ~SHV(0x000000e0, 0)) | SHV(data & 0x00e0, 0);
		if (HI) v->start = (v->start & ~SHV(0x0000ff00, 0)) | SHV(data & 0xff00, 0);
		break;
	case 0x04:
		if (LO) v->end = (v->end & ~SHV(0x00ff0000, 0)) | SHV((uint32_t)(data & 0x00ff) << 16, 0);
		if (HI) v->end = (v->end & ~SHV(0x1f000000, 0)) | SHV((uint32_t)(data & 0x1f00) << 16, 0);
		break;
	case 0x05:
		if (LO) v->end = (v->end & ~SHV(0x000000e0, 0)) | SHV(data & 0x00e0, 0);
		if (HI) v->end = (v->end & ~SHV(0x0000ff00, 0)) | SHV(data & 0xff00, 0);
		break;
	case 0x06:
		if (LO) v->k2 = (v->k2 & ~0x00f0u) | (data & 0x00f0);
		if (HI) v->k2 = (v->k2 & ~0xff00u) | (data & 0xff00);
		break;
	case 0x07:
		if (LO) v->k1 = (v->k1 & ~0x00f0u) | (data & 0x00f0);
		if (HI) v->k1 = (v->k1 & ~0xff00u) | (data & 0xff00);
		break;
	case 0x08:
		if (HI) v->lvol = (v->lvol & ~0xffu) | ((data & 0xff00) >> 8);
		break;
	case 0x09:
		if (HI) v->rvol = (v->rvol & ~0xffu) | ((data & 0xff00) >> 8);
		break;
	case 0x0a:
		if (LO) v->accum = (v->accum & ~SHV(0x00ff0000, 0)) | SHV((uint32_t)(data & 0x00ff) << 16, 0);
		if (HI) v->accum = (v->accum & ~SHV(0x1f000000, 0)) | SHV((uint32_t)(data & 0x1f00) << 16, 0);
		break;
	case 0x0b:
		if (LO) v->accum = (v->accum & ~SHV(0x000000ff, 0)) | SHV(data & 0x00ff, 0);
		if (HI) v->accum = (v->accum & ~SHV(0x0000ff00, 0)) | SHV(data & 0xff00, 0);
		break;
	case 0x0d:
		if (LO) set_active(c, data);
		break;
	case 0x0f:
		if (LO) c->current_page = data & 0x7f;
		break;
	}
}

static void write_s16(int32_t *r, uint16_t data, uint16_t mem_mask)
{
	if (LO) *r = (*r & ~0x00ff) | (data & 0x00ff);
	if (HI) *r = (int16_t)((*r & ~0xff00) | (data & 0xff00));
}

static void write_high(es5505_t *c, es5505_voice *v, int offset, uint16_t data, uint16_t mem_mask)
{
	switch (offset) {
	case 0x00:
		v->control |= 0xf000;
		if (LO) v->control = (v->control & ~0x00ffu) | (data & 0x00ff);
		if (HI) v->control = (v->control & ~0x0f00u) | (data & 0x0f00);
		break;
	case 0x01: write_s16(&v->o4n1, data, mem_mask); break;
	case 0x02: write_s16(&v->o3n1, data, mem_mask); break;
	case 0x03: write_s16(&v->o3n2, data, mem_mask); break;
	case 0x04: write_s16(&v->o2n1, data, mem_mask); break;
	case 0x05: write_s16(&v->o2n2, data, mem_mask); break;
	case 0x06: write_s16(&v->o1n1, data, mem_mask); break;
	case 0x0d:
		if (LO) set_active(c, data);
		break;
	case 0x0f:
		if (LO) c->current_page = data & 0x7f;
		break;
	}
}

static void write_test(es5505_t *c, int offset, uint16_t data, uint16_t mem_mask)
{
	switch (offset) {
	case 0x08:
		c->mode |= 0x7f8;
		if (HI) c->mode = (c->mode & ~0xf800u) | (data & 0xf800);
		if (LO) c->mode = (c->mode & ~0x0007u) | (data & 0x0007);
		break;
	case 0x0d:
		if (LO) set_active(c, data);
		break;
	case 0x0f:
		if (LO) c->current_page = data & 0x7f;
		break;
	}
}

// the registers of a voice its accumulator's path depends on, or that read it: low page control, frequency, start,
// end, accumulator; high page control and O1(n-1) (a stopped voice's sample at the accumulator)
static int ctl_reg(const es5505_t *c, int offset)
{
	offset &= 0x0f;
	if (c->current_page < 0x20)
		return offset <= 0x05 || offset == 0x0a || offset == 0x0b;
	return c->current_page < 0x40 && (offset == 0x00 || offset == 0x06);
}

// before a host access: the voices it may see or change, up to date (1: a voice register of the path)
static int ctl_host(es5505_t *c, int offset)
{
	if ((offset & 0x0f) == 0x0d) {
		for (int vi = 0; vi < 32; vi++)
			ctl_sync(c, vi, c->ctl_now);
		return 0;
	}
	if (!ctl_reg(c, offset))
		return 0;
	ctl_sync(c, c->current_page & 0x1f, c->ctl_now);
	return 1;
}

// after a write: the ends again (of a voice: at the next sample, once for all the writes before it)
static void ctl_host_done(es5505_t *c, int offset, int voice)
{
	if ((offset & 0x0f) == 0x0d) {
		c->ctl_next = CTL_NEVER;
		c->ctl_dirty = 0;
		for (int vi = 0; vi < 32; vi++)
			ctl_event(c, vi);
	} else if (voice) {
		const int vi = c->current_page & 0x1f;
		c->ctl_dirty |= 1u << vi;
		c->ctl_ev[vi] = 0;   // (until then: no end, not stepped)
		if ((offset & 0x0f) == 0x00)
			c->ctl_irq = 1;   // (a control write may set IRQ)
	}
}

static void write_reg(es5505_t *c, int offset, uint16_t data, uint16_t mem_mask);
void es5505_write(es5505_t *c, int offset, uint16_t data, uint16_t mem_mask)
{
	if (!c->ctl) {
		write_reg(c, offset, data, mem_mask);
		return;
	}
	const int voice = ctl_host(c, offset);
	write_reg(c, offset, data, mem_mask);
	ctl_host_done(c, offset, voice);
}

static void write_reg(es5505_t *c, int offset, uint16_t data, uint16_t mem_mask)
{
	es5505_voice *v = &c->voice[c->current_page & 0x1f];
	offset &= 0x0f;
	if (c->current_page < 0x20)
		write_low(c, v, offset, data, mem_mask);
	else if (c->current_page < 0x40)
		write_high(c, v, offset, data, mem_mask);
	else
		write_test(c, offset, data, mem_mask);
}

static uint16_t read_irqv(es5505_t *c)
{
	uint16_t r = c->irqv;
	c->irqv = 0x80;
	c->irq = 0;
	return r;
}

static uint16_t read_reg(es5505_t *c, int offset);
uint16_t es5505_read(es5505_t *c, int offset)
{
	if (!c->ctl)
		return read_reg(c, offset);
	ctl_host(c, offset);
	return read_reg(c, offset);   // (a vector read frees it: ctl_irq still says whether a voice waits)
}

static uint16_t read_reg(es5505_t *c, int offset)
{
	int vi = c->current_page & 0x1f;
	es5505_voice *v = &c->voice[vi];
	offset &= 0x0f;
	if (offset == 0x0d) return c->active_voices;
	if (offset == 0x0e) return read_irqv(c);
	if (offset == 0x0f) return c->current_page;
	if (c->current_page < 0x20) {
		switch (offset) {
		case 0x00: return (uint16_t)(v->control | 0xf000);
		case 0x01: return (uint16_t)RES(v->freqcount, 1);
		case 0x02: return (uint16_t)(RES(v->start, 0) >> 16);
		case 0x03: return (uint16_t)RES(v->start, 0);
		case 0x04: return (uint16_t)(RES(v->end, 0) >> 16);
		case 0x05: return (uint16_t)RES(v->end, 0);
		case 0x06: return (uint16_t)v->k2;
		case 0x07: return (uint16_t)v->k1;
		case 0x08: return (uint16_t)(v->lvol << 8);
		case 0x09: return (uint16_t)(v->rvol << 8);
		case 0x0a: return (uint16_t)(RES(v->accum, 0) >> 16);
		case 0x0b: return (uint16_t)RES(v->accum, 0);
		}
		return 0;
	}
	if (c->current_page < 0x40) {
		switch (offset) {
		case 0x00: return (uint16_t)(v->control | 0xf000);
		case 0x01: return (uint16_t)v->o4n1;
		case 0x02: return (uint16_t)v->o3n1;
		case 0x03: return (uint16_t)v->o3n2;
		case 0x04: return (uint16_t)v->o2n1;
		case 0x05: return (uint16_t)v->o2n2;
		case 0x06:
			// MAME: the Taito F3 games read raw sample data through O1(n-1) of a stopped voice
			if (v->control & CONTROL_STOPMASK)
				v->o1n1 = read_sample(c, vi, integer_addr((uint32_t)v->accum, 0));
			return (uint16_t)v->o1n1;
		}
		return 0;
	}
	if (offset == 0x08)
		return (uint16_t)(c->mode | 0x7f8);
	return 0;
}
