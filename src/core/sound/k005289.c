/*  Konami 005289 wave address generator. Ported from MAME src/devices/sound/k005289.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail).
    This file is part of ReBOX. GPL-2.0-or-later.

    The chip is only an address generator into the wave PROMs; MAME does the DAC, the volume and the mix of the
    two channels here for convenience, and so do we.
*/
#include "k005289.h"
#include "../rbx.h"

#include <string.h>

void k005289_init(k005289_t *k, uint32_t clock, const uint8_t *prom)
{
	memset(k, 0, sizeof(*k));
	k->prom = prom;
	k->clock = clock;
	k->rate = clock / 16 ? clock / 16 : 1;
	k005289_reset(k);
}

void k005289_set_rate(k005289_t *k, uint32_t rate)
{
	k->rate = rate ? rate : 1;
	k->acc = 0;
}

uint32_t k005289_rate(const k005289_t *k)
{
	return k->rate;
}

void k005289_reset(k005289_t *k)   // (device_start's voice reset: MAME has no device_reset)
{
	memset(k->voice, 0, sizeof(k->voice));
	k->acc = 0;
}

void k005289_control_a_w(k005289_t *k, uint8_t data)
{
	k->voice[0].volume = data & 0xf;
	k->voice[0].waveform = (uint16_t)((k->voice[0].waveform & ~0xe0) | (data & 0xe0));
}

void k005289_control_b_w(k005289_t *k, uint8_t data)
{
	k->voice[1].volume = data & 0xf;
	k->voice[1].waveform = (uint16_t)((k->voice[1].waveform & ~0xe0) | (data & 0xe0));
}

void k005289_ld1_w(k005289_t *k, uint32_t offset)
{
	k->voice[0].pitch = (uint16_t)(0xfff - (offset & 0xfff));
}

void k005289_ld2_w(k005289_t *k, uint32_t offset)
{
	k->voice[1].pitch = (uint16_t)(0xfff - (offset & 0xfff));
}

void k005289_tg1_w(k005289_t *k)
{
	k->voice[0].frequency = k->voice[0].pitch;
}

void k005289_tg2_w(k005289_t *k)
{
	k->voice[1].frequency = k->voice[1].pitch;
}

// t chip clocks of MAME's per sample step (if (--counter < 0) { waveform++ (low 5 bits); counter = frequency; })
// in one go: the counter only ever holds 0..frequency between clocks
static inline void advance(k005289_t *k, int i, uint32_t t)
{
	int32_t c = k->voice[i].counter;
	if (c < 0)   // (not reachable from the handlers; a loaded state)
		c = 0;
	uint32_t need = (uint32_t)c + 1;   // clocks to the next wrap below 0
	if (t >= need) {
		uint32_t period = (uint32_t)k->voice[i].frequency + 1;
		uint32_t rem = t - need;
		uint32_t n = 1 + rem / period;
		uint16_t w = k->voice[i].waveform;
		k->voice[i].waveform = (uint16_t)((w & 0xffe0u) | ((w + n) & 0x1fu));
		k->voice[i].counter = (int16_t)(k->voice[i].frequency - rem % period);
	} else
		k->voice[i].counter = (int16_t)(c - (int32_t)t);
}

void k005289_render(k005289_t *k, int32_t *out, int n)
{
	const uint32_t clock = k->clock, rate = k->rate;
	for (int s = 0; s < n; s++) {
		k->acc += clock;
		uint32_t t = k->acc / rate;
		k->acc -= t * rate;
		int32_t sum = 0;
		for (int i = 0; i < 2; i++) {
			if (t)
				advance(k, i, t);
			int v = k->prom ? (k->prom[((i & 1) << 8) | (k->voice[i].waveform & 0xff)] & 0xf) - 8 : 0;
			sum += v * k->voice[i].volume;
		}
		out[s] = sum * 64;   // (add_int(.., 512) -> x 32768 / 512)
	}
}

int k005289_state(k005289_t *k, void *ctx, int load, const char *tag)
{
	(void)load;
	return rbx_state_io(ctx, tag, k, (uint32_t)((const uint8_t *)&k->state_end - (const uint8_t *)k));
}
