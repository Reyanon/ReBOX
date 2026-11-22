/*  Namco C352 PCM sound chip (32 voices, 8-bit linear / mu-law samples, noise, four outputs; Namco NB-1 / NB-2,
    System 22 / 23, ND-1, FL).
    Ported from MAME 0.289 src/devices/sound/c352.cpp / c352.h (BSD-3-Clause, copyright-holders: R. Belmont,
    superctr).
    This file is part of ReBOX. GPL-2.0-or-later.

    The sample rate is the clock / the divider (MAME sample_rate_base); the board renders at that rate, up to the
    time of each register access (MAME updates its stream there).
*/
#ifndef RBX_C352_H
#define RBX_C352_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
	uint32_t pos;
	uint32_t counter;
	int16_t sample;
	int16_t last_sample;
	uint16_t vol_f;
	uint16_t vol_r;
	uint8_t curr_vol[4];
	uint16_t freq;
	uint16_t flags;
	uint16_t wave_bank;
	uint16_t wave_start;
	uint16_t wave_end;
	uint16_t wave_loop;
} c352_voice_t;

typedef struct {
	// state (saved: up to state_end)
	c352_voice_t v[32];
	uint16_t random;
	uint16_t control;
	uint8_t state_end;
	// configuration
	const uint8_t *rom;       // the "c352" region (device_rom_interface<24>: bytes)
	uint32_t rom_mask;        // (the region's size - 1, a power of two: mirrored)
	int16_t mulawtab[256];
} c352_t;

#define C352_STATE_SIZE offsetof(c352_t, state_end)

void c352_init(c352_t *c, const uint8_t *rom, uint32_t rom_size);
void c352_reset(c352_t *c);
// word offsets (MAME read / write); mem_mask as MAME's (0xffff a word access)
uint16_t c352_read(c352_t *c, uint32_t offset);
void c352_write(c352_t *c, uint32_t offset, uint16_t data, uint16_t mem_mask);
// n samples of outputs 0 (front left) and 1 (front right) added to l / r (16-bit scale)
void c352_render(c352_t *c, int32_t *l, int32_t *r, int n);

#endif
