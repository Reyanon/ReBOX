/*  Seta X1-010 (16 channels, PCM samples or RAM wave forms with envelopes). Ported from MAME
    src/devices/sound/x1_010.cpp (BSD-3-Clause, copyright-holders: Luca Elia).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_X1_010_H
#define RBX_X1_010_H

#include <stdint.h>

#define X1_010_CHANNELS 16

typedef struct {
	uint8_t reg[0x2000];
	uint8_t hi_word[0x2000];
	uint32_t smp_offset[X1_010_CHANNELS], env_offset[X1_010_CHANNELS];
	// ---- configuration (not saved) ----
	const uint8_t *rom;
	uint32_t rom_mask;
} x1_010_t;

#define X1_010_STATE_SIZE ((uint32_t)(sizeof(((x1_010_t *)0)->reg) + sizeof(((x1_010_t *)0)->hi_word) + \
	sizeof(((x1_010_t *)0)->smp_offset) + sizeof(((x1_010_t *)0)->env_offset)))

void x1_010_init(x1_010_t *x, const uint8_t *rom, uint32_t rom_size);   // (rom_size: a power of 2)
void x1_010_reset(x1_010_t *x);
uint8_t x1_010_read(const x1_010_t *x, uint32_t offset);
void x1_010_write(x1_010_t *x, uint32_t offset, uint8_t data);
uint16_t x1_010_word_r(const x1_010_t *x, uint32_t offset);
void x1_010_word_w(x1_010_t *x, uint32_t offset, uint16_t data);
// n samples at clock / 512 added to l[] and r[] (16-bit scale, no gain)
void x1_010_render(x1_010_t *x, int32_t *l, int32_t *r, int n);

#endif
