/*  Ricoh RF5C68 PCM (8 channels from 64 KB of wave RAM). Ported from MAME src/devices/sound/rf5c68.cpp (BSD-3-Clause,
    copyright-holders: Olivier Galibert, Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_RF5C68_H
#define RBX_RF5C68_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
	uint8_t enable, env, pan, start;
	uint32_t addr;
	uint16_t step, loopst;
} rf5c68_chan_t;

typedef struct {
	// state (saved: from here to state_end)
	rf5c68_chan_t chan[8];
	uint8_t cbank, enable;
	uint16_t wbank;
	uint64_t frac;               // (the chip's samples against the output's)
	int32_t cur;                 // (the last chip sample, L + R)
	uint8_t ram[0x10000];
	uint8_t state_end;
	// configuration
	uint32_t clock;
	int32_t cur_l, cur_r;        // (the last chip sample's sides, rf5c68_render_lr: not saved, held one sample)
} rf5c68_t;

#define RF5C68_STATE_SIZE offsetof(rf5c68_t, state_end)

void rf5c68_init(rf5c68_t *c, uint32_t clock);
void rf5c68_w(rf5c68_t *c, int offset, uint8_t data);
uint8_t rf5c68_mem_r(const rf5c68_t *c, int offset);
void rf5c68_mem_w(rf5c68_t *c, int offset, uint8_t data);
// add n samples at out_rate (mono: left + right) to out
void rf5c68_render(rf5c68_t *c, int32_t *out, int n, uint32_t out_rate);
// the same in stereo, each side scaled by gain / 256 (MAME's routes) and added to l[] / r[]
void rf5c68_render_lr(rf5c68_t *c, int32_t *l, int32_t *r, int n, uint32_t out_rate, int gain);

#endif
