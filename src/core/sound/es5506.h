/*  Ensoniq ES5506 (OTTO) wavetable synthesizer, ported from MAME src/devices/sound/es5506.cpp
    (BSD-3-Clause, copyright-holders: Aaron Giles). ES5506 part only (the ES5505 of the Taito F3 is es5505.c, left
    as it is): 32 voices, 4-pole filters, envelopes (volume / filter ramps), u-law samples, 4 sample banks, 6 stereo
    outputs (voices to output "channel % channels"), the 8-bit host bus with its 32-bit latches.
    Sample memory: 4 banks of 16-bit words (big-endian bytes), the board sets them (MAME's 4 address spaces).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_ES5506_H
#define RBX_ES5506_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
	uint32_t control, freqcount, start, end, accum;   // (the 64-bit fields of MAME: 21.11 bits, shift 0, 32 bits)
	uint32_t lvol, rvol, lvramp, rvramp, ecount;
	uint32_t k2, k2ramp, k1, k1ramp;
	int32_t o4n1, o3n1, o3n2, o2n1, o2n2, o1n1;
	uint32_t filtcount;
} es5506_voice;

typedef struct {
	es5506_voice voice[32];
	uint32_t write_latch, read_latch;
	uint8_t current_page, active_voices, mode, irqv;
	uint8_t wst, wend, lrend, channels;
	// sample memory (not saved): bank b's words, word w = base[2w] << 8 | base[2w + 1]; reads past words give 0
	const uint8_t *base[4];
	uint32_t words[4];
} es5506_t;
#define ES5506_STATE_SIZE offsetof(es5506_t, base)

void es5506_init(es5506_t *c, int channels);   // (device_start; banks cleared: es5506_set_bank)
void es5506_reset(es5506_t *c);                // (device_reset: RESB)
void es5506_set_bank(es5506_t *c, int bank, const uint8_t *base, uint32_t bytes);   // bytes: up to 2^21 words used
uint8_t es5506_read(es5506_t *c, int offset);     // offset 0..0x3f (the byte lanes of the 16 registers)
void es5506_write(es5506_t *c, int offset, uint8_t data);
// one output sample: out[0..2 * channels - 1] = ch0 L, ch0 R, ch1 L ... (20-bit range, clamped)
void es5506_sample(es5506_t *c, int32_t *out);
// a sample's master clocks: 16 * (active voices + 1)
static inline int es5506_sample_clocks(const es5506_t *c) { return 16 * (c->active_voices + 1); }

#endif
