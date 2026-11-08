/*  Ensoniq ES5505 (OTIS) wavetable synthesizer, ported from MAME src/devices/sound/es5506.cpp
    (BSD-3-Clause, copyright-holders: Aaron Giles). ES5505 part only: 32 voices, 4-pole filters, 4 stereo outputs,
    no envelopes and no u-law (the ES5505 has neither).
    Sample memory: 16-bit words through a per-voice bank (the board decides the mapping, as MAME's address map does).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_ES5505_H
#define RBX_ES5505_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
	uint32_t control;
	uint64_t freqcount, start, end, accum;
	uint32_t lvol, rvol;
	uint32_t k2, k1;
	int32_t o4n1, o3n1, o3n2, o2n1, o2n2, o1n1;
} es5505_voice;

typedef struct {
	es5505_voice voice[32];
	uint8_t current_page, active_voices, irqv, pad;
	uint16_t mode;
	int irq;                       // IRQB output (1 = a voice interrupt is latched)
	// sample memory: word w of voice v = rom[(bank[v] + w) & rom_mask] (big-endian 16-bit words)
	const uint8_t *rom;
	uint32_t rom_mask;             // in words
	uint32_t bank[32];             // in words, set by the board
	// es5505_sample_ctl's copy (es5505_ctl_begin): the accumulators move only when read, written or at their ends.
	// (not chip state: a save state keeps the chip up to ES5505_STATE_SIZE)
	int ctl, ctl_irq;              // (ctl_irq: a voice interrupt may wait for the vector)
	uint32_t ctl_dirty;            // voices written since the last sample: their ends to find
	uint64_t ctl_now, ctl_next;    // samples done, the next sample a voice ends at
	uint64_t ctl_t[32], ctl_ev[32];   // each voice: the sample its accum is at, the sample of its next end
} es5505_t;
#define ES5505_STATE_SIZE offsetof(es5505_t, ctl)

void es5505_init(es5505_t *c, const uint8_t *rom, uint32_t rom_bytes);
void es5505_reset(es5505_t *c);
uint16_t es5505_read(es5505_t *c, int offset);
void es5505_write(es5505_t *c, int offset, uint16_t data, uint16_t mem_mask);
// one output sample of the 4 stereo channels: out[0..7] = ch0 L, ch0 R, ch1 L ... (20-bit range, clamped)
void es5505_sample(es5505_t *c, int32_t out[8]);
// a sample's steps without the sound (controls, accumulators, interrupt vector: all that reads give but the filters),
// on a chip made such a copy by es5505_ctl_begin (after it is set or loaded; a reset keeps it one)
void es5505_ctl_begin(es5505_t *c);
void es5505_sample_ctl(es5505_t *c);
// a read at offset (in the current page) gives filter state, which es5505_sample_ctl does not keep
int es5505_read_needs_sound(const es5505_t *c, int offset);
// clock divider of one output sample: 16 * (active voices + 1) master clocks
static inline int es5505_sample_clocks(const es5505_t *c) { return 16 * (c->active_voices + 1); }

#endif
