/*  OKI MSM6295 ADPCM, ported from MAME src/devices/sound/okim6295.cpp and okiadpcm.cpp
    (BSD-3-Clause, copyright-holders: Mirko Buffoni, Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_OKIM6295_H
#define RBX_OKIM6295_H

#include <stdint.h>

typedef struct {
	int playing;
	uint32_t base, sample, count;
	int volume;              // 0..32
	int signal, step;
} oki_voice;

typedef struct {
	oki_voice v[4];
	int command;
	const uint8_t *rom;
	uint32_t rom_size;
	uint32_t bank;           // added to sample addresses (boards with banked sample ROM)
	uint32_t rate;           // samples per second (clock / 132 or 165)
	int gain;                // 1/256
	// optional: the sample ROM through the board's banking (NMK112, a sound CPU's bank latches); a = 0 .. 0x3ffff
	uint8_t (*read)(void *user, uint32_t a);
	void *user;
} okim6295_t;

void oki_init(okim6295_t *o, uint32_t clock, int pin7_high, const uint8_t *rom, uint32_t size, int gain256);
void oki_reset(okim6295_t *o);
void oki_write(okim6295_t *o, uint8_t cmd);
uint8_t oki_read(okim6295_t *o);
int32_t oki_sample(okim6295_t *o);   // next output sample, 16-bit scale, gain applied

#endif
