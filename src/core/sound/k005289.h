/*  Konami 005289 wave address generator (the "SCC" of the Bubble System / Nemesis boards). Ported from MAME
    src/devices/sound/k005289.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_K005289_H
#define RBX_K005289_H

#include <stdint.h>

typedef struct {
	// ---- saved ----
	struct { int16_t counter; uint16_t frequency, pitch, waveform; uint8_t volume; } voice[2];
	uint32_t acc;          // (the fraction of a chip clock carried between output samples, in 1/rate units)
	uint8_t state_end;
	// ---- the configuration ----
	const uint8_t *prom;   // the "k005289" region: 0x200 bytes (2 x 256, 4 bits each)
	uint32_t clock, rate;
} k005289_t;

// MAME runs the stream at the chip clock; here the counters advance clock / rate chip clocks per output sample
// (an integer accumulator, exact) and each output sample is the chip's value at the last of those clocks
// (point sampling, no averaging). The default output rate is clock / 16.
void k005289_init(k005289_t *k, uint32_t clock, const uint8_t *prom);
void k005289_set_rate(k005289_t *k, uint32_t rate);
uint32_t k005289_rate(const k005289_t *k);
void k005289_reset(k005289_t *k);
// the AY-3-8910 #2 port A / B: bits 0-3 the volume, bits 5-7 the waveform select
void k005289_control_a_w(k005289_t *k, uint8_t data);
void k005289_control_b_w(k005289_t *k, uint8_t data);
// the address bus is latched (offset 0..0xfff within the 0x1000 window), the data is ignored
void k005289_ld1_w(k005289_t *k, uint32_t offset);
void k005289_ld2_w(k005289_t *k, uint32_t offset);
void k005289_tg1_w(k005289_t *k);
void k005289_tg2_w(k005289_t *k);
// n mono samples at the output rate, MAME's int16 scale (add_int(.., 512): x 64 per voice, before the route gain)
void k005289_render(k005289_t *k, int32_t *out, int n);
int k005289_state(k005289_t *k, void *ctx, int load, const char *tag);

#endif
