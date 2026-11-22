/*  Sega 315-5218 SegaPCM: 16 voices of 8-bit samples from ROM, stereo (MAME src/devices/sound/segapcm.cpp,
    BSD-3-Clause, copyright-holders: Hiromitsu Shioya, Olivier Galibert). One sample a tick at clock / 128; the output
    in MAME's 1/32768 units (voice sample * volume).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_SEGAPCM_H
#define RBX_SEGAPCM_H

#include <stdint.h>

typedef struct {
	uint32_t addr;
	uint16_t loop;
	uint8_t end, freq, lvol, rvol, ctrl, pad;
} pcm_voice;
typedef struct {
	pcm_voice v[16];   // (saved)
	uint8_t ram[0x100];   // (saved; 0xff at start)
	const uint8_t *rom;
	uint32_t mask;        // the ROM's size - 1 (a power of two)
	uint8_t bankshift, bankmask;   // MAME set_bank: BANK_512 12 / 0x70, BANK_12M | BANK_MASKF8 13 / 0xf8
} segapcm_t;

uint8_t segapcm_r(const segapcm_t *p, uint8_t offset);   // the 0x100-byte window
void segapcm_w(segapcm_t *p, uint8_t offset, uint8_t data);
void segapcm_tick(segapcm_t *p, int32_t *l, int32_t *r);

#endif
