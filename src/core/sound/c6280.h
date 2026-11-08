/*  Hudson HuC6280 PSG (C6280), ported from MAME src/devices/sound/c6280.cpp (BSD-3-Clause, copyright-holders:
    Charles MacDonald).
    MAME runs it one sample per input clock (the CPU clock); here an output sample is the average of its clocks,
    stepped from one waveform / noise / LFO change to the next (the same values as MAME's clock by clock).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_C6280_H
#define RBX_C6280_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint16_t frequency;
	uint8_t control, balance, waveform[32], index, noise_control;
	int16_t dda;
	int32_t noise_counter, tick;
	uint32_t noise_seed;
} c6280_channel;

typedef struct {
	// state (saved: the whole struct up to clock)
	uint8_t select, enabled, balance, lfo_frequency, lfo_control, pad[3];
	c6280_channel ch[8];
	uint64_t frac;           // input clocks of the output rate not yet consumed (x rate)
	// configuration
	uint32_t clock;
	int16_t volume_table[32];
} c6280_t;

#define C6280_STATE_SIZE ((uint32_t)offsetof(c6280_t, clock))

void c6280_init(c6280_t *c, uint32_t clock);
void c6280_reset(c6280_t *c);
void c6280_w(c6280_t *c, int offset, uint8_t data);
// n samples at rate: out[i] += (left + right) * gain / 256 (MAME's stream values x 32768)
void c6280_render(c6280_t *c, int32_t *out, int n, uint32_t rate, int gain);

#ifdef __cplusplus
}
#endif

#endif
