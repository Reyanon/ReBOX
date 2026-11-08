/*  Konami 051649 SCC1 (5 channel wavetable). Ported from MAME src/devices/sound/k051649.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_K051649_H
#define RBX_K051649_H

#include <stdint.h>

typedef struct {
	// ---- saved ----
	struct {
		uint8_t counter;     // address counter for the wavetable
		uint16_t clock;      // internal clock (0xffff: a sample reload pending)
		uint16_t frequency;  // (input clock / (32 * (frequency + 1)))
		uint8_t volume;
		int16_t sample;      // latched sample data
		uint8_t key;
		int8_t waveram[32];
	} ch[5];
	uint8_t test;
	uint32_t acc;            // (the fraction of a chip clock carried between output samples, in 1/rate units)
	uint8_t state_end;
	// ---- the configuration ----
	uint32_t clock, rate;
} k051649_t;

// MAME 0.289 runs the stream at the chip clock (not clock / 16); here the channels advance clock / rate chip
// clocks per output sample (an integer accumulator, exact) and each output sample is the chip's value at the
// last of those clocks (point sampling). The default output rate is clock / 16 (16 chip clocks per sample).
void k051649_init(k051649_t *k, uint32_t clock);
void k051649_set_rate(k051649_t *k, uint32_t rate);
uint32_t k051649_rate(const k051649_t *k);
void k051649_reset(k051649_t *k);
// scc_map (City Bomber / Kitten Kaboodle map it at 0x9800-0x98ff), with MAME's offsets:
//   0x00-0x7f waveform r/w (offset 0..0x7f), 0x80-0x89 (+0x10 mirror) frequency_w (offset 0..9),
//   0x8a-0x8e (+0x10) volume_w (offset 0..4), 0x8f (+0x10) keyonoff_w, 0xe0-0xff test r/w
void k051649_waveform_w(k051649_t *k, uint32_t offset, uint8_t data);
uint8_t k051649_waveform_r(k051649_t *k, uint32_t offset);
void k051649_volume_w(k051649_t *k, uint32_t offset, uint8_t data);
void k051649_frequency_w(k051649_t *k, uint32_t offset, uint8_t data);
void k051649_keyonoff_w(k051649_t *k, uint8_t data);
void k051649_test_w(k051649_t *k, uint8_t data);
// reading the test register writes the open bus value (MAME: space.unmap(), 0 on the Z80) into it
uint8_t k051649_test_r(k051649_t *k, uint8_t unmap);
// the whole scc_map in one place (offset 0..0xff of the 0x100 window)
void k051649_scc_w(k051649_t *k, uint32_t offset, uint8_t data);
uint8_t k051649_scc_r(k051649_t *k, uint32_t offset, uint8_t unmap);
// n mono samples at the output rate, MAME's int16 scale (add_int(sample >> 4, 1024): x 32 per channel)
void k051649_render(k051649_t *k, int32_t *out, int n);
int k051649_state(k051649_t *k, void *ctx, int load, const char *tag);

#endif
