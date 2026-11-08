/*  Konami 051649 SCC1 (5 channel wavetable). Ported from MAME src/devices/sound/k051649.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail).
    This file is part of ReBOX. GPL-2.0-or-later.

    Only the 051649 handlers (channel 5 shares the waveram of channel 4); MAME's 052539 variant is left out.
*/
#include "k051649.h"
#include "../rbx.h"

#include <string.h>

void k051649_init(k051649_t *k, uint32_t clock)
{
	memset(k, 0, sizeof(*k));   // (the constructor: counters, volume 0, waveram 0)
	k->clock = clock;
	k->rate = clock / 16 ? clock / 16 : 1;
	k051649_reset(k);
}

void k051649_set_rate(k051649_t *k, uint32_t rate)
{
	k->rate = rate ? rate : 1;
	k->acc = 0;
}

uint32_t k051649_rate(const k051649_t *k)
{
	return k->rate;
}

void k051649_reset(k051649_t *k)   // (device_reset: the waveram and the latched samples are kept)
{
	for (int i = 0; i < 5; i++) {
		k->ch[i].frequency = 0;
		k->ch[i].volume = 0xf;
		k->ch[i].counter = 0;
		k->ch[i].clock = 0;
		k->ch[i].key = 0;
	}
	k->test = 0;
	k->acc = 0;
}

void k051649_waveform_w(k051649_t *k, uint32_t offset, uint8_t data)
{
	offset &= 0x7f;
	// waveram is read-only?
	if ((k->test & 0x40) || ((k->test & 0x80) && offset >= 0x60))
		return;
	if (offset >= 0x60) {
		// channel 5 shares waveram with channel 4
		k->ch[3].waveram[offset & 0x1f] = (int8_t)data;
		k->ch[4].waveram[offset & 0x1f] = (int8_t)data;
	} else
		k->ch[offset >> 5].waveram[offset & 0x1f] = (int8_t)data;
}

uint8_t k051649_waveform_r(k051649_t *k, uint32_t offset)
{
	uint8_t counter = 0;
	offset &= 0x7f;
	// test register bits 6/7 expose the internal counter
	if (k->test & 0xc0) {
		if (offset >= 0x60 && (k->test & 0xc0) != 0xc0)
			counter = k->ch[3 + (k->test >> 6 & 1)].counter;
		else if (k->test & 0x40)
			counter = k->ch[offset >> 5].counter;
	}
	return (uint8_t)k->ch[offset >> 5].waveram[(offset + counter) & 0x1f];
}

void k051649_volume_w(k051649_t *k, uint32_t offset, uint8_t data)
{
	if (offset < 5)
		k->ch[offset].volume = data & 0xf;
}

void k051649_frequency_w(k051649_t *k, uint32_t offset, uint8_t data)
{
	const int freq_hi = offset & 1;
	offset >>= 1;
	if (offset >= 5)
		return;
	if (freq_hi)
		k->ch[offset].frequency = (uint16_t)((k->ch[offset].frequency & 0x0ff) | (data << 8 & 0xf00));
	else
		k->ch[offset].frequency = (uint16_t)((k->ch[offset].frequency & 0xf00) | data);
	// test register bit 5 resets the internal counter
	if (k->test & 0x20)
		k->ch[offset].counter = 0;
	// sample reload pending
	k->ch[offset].clock = 0xffff;
}

void k051649_keyonoff_w(k051649_t *k, uint8_t data)
{
	for (int i = 0; i < 5; i++)
		k->ch[i].key = (data >> i) & 1;
}

void k051649_test_w(k051649_t *k, uint8_t data)
{
	k->test = data;
}

uint8_t k051649_test_r(k051649_t *k, uint8_t unmap)
{
	// reading the test register triggers a write
	k051649_test_w(k, unmap);
	return unmap;
}

void k051649_scc_w(k051649_t *k, uint32_t offset, uint8_t data)
{
	offset &= 0xff;
	if (offset < 0x80)
		k051649_waveform_w(k, offset, data);
	else if (offset < 0xa0) {
		uint32_t r = offset & 0x0f;   // (0x80-0x8f, mirrored at 0x90-0x9f)
		if (r < 0x0a)
			k051649_frequency_w(k, r, data);
		else if (r < 0x0f)
			k051649_volume_w(k, r - 0x0a, data);
		else
			k051649_keyonoff_w(k, data);
	} else if (offset >= 0xe0)
		k051649_test_w(k, data);
}

uint8_t k051649_scc_r(k051649_t *k, uint32_t offset, uint8_t unmap)
{
	offset &= 0xff;
	if (offset < 0x80)
		return k051649_waveform_r(k, offset);
	if (offset >= 0xe0)
		return k051649_test_r(k, unmap);
	return unmap;
}

// t chip clocks of MAME's per sample step in one go:
//   if (frequency > 8) { if (++clock > frequency) { counter++; clock = 0; } if (clock == 0) latch the sample; }
// clock is 0..frequency, or 0xffff after a frequency write (the u16 wraps to 0: a reload without a step).
// The latch inputs (waveram, key, volume) do not change inside a render call, so the last latch decides.
static inline void advance(k051649_t *k, int i, uint32_t t)
{
	const uint32_t f = k->ch[i].frequency;
	if (f <= 8 || !t)   // (channel is halted for freq < 9)
		return;
	int latch = 0;
	uint32_t c = k->ch[i].clock;
	if (c == 0xffff) {
		c = 0;
		latch = 1;
		t--;
	} else if (c > f)   // (not reachable from the handlers; a loaded state)
		c = f;
	uint32_t need = f + 1 - c;   // clocks to the next step
	if (t >= need) {
		uint32_t rem = t - need;
		uint32_t n = 1 + rem / (f + 1);
		k->ch[i].counter = (uint8_t)((k->ch[i].counter + n) & 0x1f);
		c = rem % (f + 1);
		latch = 1;   // (a step happened: the sample was latched at its clock 0)
	} else
		c += t;
	k->ch[i].clock = (uint16_t)c;
	if (latch)
		k->ch[i].sample = (int16_t)((k->ch[i].key ? k->ch[i].waveram[k->ch[i].counter] : 0) * k->ch[i].volume);
}

void k051649_render(k051649_t *k, int32_t *out, int n)
{
	const uint32_t clock = k->clock, rate = k->rate;
	for (int s = 0; s < n; s++) {
		k->acc += clock;
		uint32_t t = k->acc / rate;
		k->acc -= t * rate;
		int32_t sum = 0;
		for (int i = 0; i < 5; i++) {
			advance(k, i, t);
			// scale to 11 bit digital output on chip
			sum += k->ch[i].sample >> 4;
		}
		out[s] = sum * 32;   // (add_int(.., 1024) -> x 32768 / 1024)
	}
}

int k051649_state(k051649_t *k, void *ctx, int load, const char *tag)
{
	(void)load;
	return rbx_state_io(ctx, tag, k, (uint32_t)((const uint8_t *)&k->state_end - (const uint8_t *)k));
}
