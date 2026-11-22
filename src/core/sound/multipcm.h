/*  Yamaha YMW-258-F 'GEW8' (Sega 315-5560, MultiPCM): 28 voices of 8 / 12-bit PCM from a ROM, envelopes and LFOs.
    Ported from MAME 0.289 src/devices/sound/multipcm.cpp and gew.cpp (BSD-3-Clause, copyright-holders: Miguel Angel
    Horna). The ROM space (22 bits) is the board's: here a region with two 512 KB banks at 0x100000 / 0x180000
    (Sega Multi 32's multipcm_map), the first 1 MB fixed.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_MULTIPCM_H
#define RBX_MULTIPCM_H

#include <stddef.h>
#include <stdint.h>

#define MULTIPCM_VOICES 28

typedef struct {
	uint32_t start, loop, end;
	uint8_t attack_reg, decay1_reg, decay2_reg, decay_level, release_reg, key_rate_scale;
	uint8_t lfo_vibrato_reg, lfo_amplitude_reg, format;
} multipcm_sample_t;

typedef struct {
	uint16_t phase;
	uint32_t phase_step;
	uint8_t amplitude;   // the table (0 pitch, 1 amplitude)
	uint8_t scale;       // the scale table's index
} multipcm_lfo_t;

typedef struct {
	uint8_t regs[8];
	uint8_t playing;
	multipcm_sample_t sample;
	uint32_t offset;
	uint8_t octave;
	uint16_t pitch;
	uint32_t step;
	uint8_t reverse;
	uint32_t pan;
	uint32_t total_level, dest_total_level;
	int32_t total_level_step;
	int32_t prev_sample;
	// envelope_gen_t
	int32_t eg_volume;
	uint8_t eg_state, eg_reverb;
	int32_t eg_attack_rate, eg_decay1_rate, eg_decay2_rate, eg_release_rate, eg_decay_level;
	uint8_t lfo_frequency;
	multipcm_lfo_t pitch_lfo;
	uint8_t vibrato;
	multipcm_lfo_t amplitude_lfo;
	uint8_t tremolo;
} multipcm_slot_t;

typedef struct {
	// state (saved: from here to state_end)
	multipcm_slot_t slots[MULTIPCM_VOICES];
	uint32_t cur_slot, address;
	uint32_t bank_lo, bank_hi;   // the banks' offsets in the ROM
	uint64_t frac;               // (the chip's samples against the output's)
	int32_t cur_l, cur_r;        // (the last chip sample)
	uint8_t state_end;
	// configuration
	uint32_t clock;
	float rate;
	const uint8_t *rom;
	uint32_t rom_size;
} multipcm_t;

#define MULTIPCM_STATE_SIZE offsetof(multipcm_t, state_end)

void multipcm_init(multipcm_t *c, uint32_t clock, const uint8_t *rom, uint32_t rom_size);   // device_start + reset
void multipcm_reset(multipcm_t *c);
void multipcm_write(multipcm_t *c, int offset, uint8_t data);
void multipcm_set_banks(multipcm_t *c, uint32_t lo, uint32_t hi);   // (entries of 512 KB)
// add n samples at out_rate to l[] / r[], each side scaled by gain / 256 (held between the chip's samples)
void multipcm_render_lr(multipcm_t *c, int32_t *l, int32_t *r, int n, uint32_t out_rate, int gain);

#endif
