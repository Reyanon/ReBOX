/*  Data East Pinball BSMT2000 sound board, after MAME src/mame/shared/decobsmt.cpp (BSD-3-Clause, copyright-holders:
    R. Belmont) and src/devices/sound/bsmt2000.cpp (BSD-3-Clause, copyright-holders: Aaron Giles).
    A 6809E at 2 MHz (its FIRQ at 489 Hz, its IRQ when the BSMT takes a word) feeds the BSMT2000: a TMS320C15 at
    24 MHz running its mask ROM, which reads the sample ROM through its ports and writes the left / right outputs
    (MAME's stream at 24 MHz / 128). Here every 4 of those samples averaged: out at 46875 Hz, stereo.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_DECOBSMT_H
#define RBX_DECOBSMT_H

#include <stdint.h>
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/tms320c1x/tms320c1x.h"

#define DECOBSMT_RATE 46875

typedef struct {
	m6809_t cpu;
	tms320c1x_t tms;
	// ---- state (saved: decobsmt_state) ----
	struct {
		uint8_t latch, reset, comms, cpu_reset, firq, irq, write_pending, acc_n;
		uint16_t reg_select, write_data, rom_address, rom_bank;
		int16_t left, right;
		int32_t acc_l, acc_r;
		uint32_t firq_frac, carry;   // carry: the clocks given and not run yet (under a slice)
		int64_t clock;          // master clocks run
	} s;
	uint8_t ram[0x2000];
	// ---- configuration ----
	const uint8_t *rom;         // the 6809's 64K (0x2000-0xffff used)
	const uint8_t *samples;     // the BSMT's sample ROM
	uint32_t samples_size;
	uint16_t prog[0x1000];      // the BSMT2000's mask ROM (words)
} decobsmt_t;

// prog: the mask ROM bytes (big-endian words, 0x2000 bytes)
void decobsmt_init(decobsmt_t *d, const uint8_t *rom, const uint8_t *prog, const uint8_t *samples, uint32_t samples_size);
void decobsmt_reset(decobsmt_t *d);
void decobsmt_comms_w(decobsmt_t *d, uint8_t data);
void decobsmt_reset_line(decobsmt_t *d, int state);   // the 6809's reset (1 = held)
// run master clocks (24 MHz); the samples made go to out_l / out_r (added), at most max; returns how many
int decobsmt_run(decobsmt_t *d, uint32_t clocks, int32_t *out_l, int32_t *out_r, int max);
int decobsmt_state(decobsmt_t *d, void *ctx, int load);

#endif
