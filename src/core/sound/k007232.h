/*  Konami 007232 PCM controller. Ported from MAME src/devices/sound/k007232.cpp (BSD-3-Clause, copyright-holders:
    Nicola Salmoria, Hiromitsu Shioya).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_K007232_H
#define RBX_K007232_H

#include <stdint.h>

typedef struct {
	// ---- saved ----
	struct { uint8_t vol[2]; uint32_t addr; int32_t counter; uint32_t start, step, bank; uint8_t play; } ch[2];
	uint8_t wreg[0x10];
	uint8_t state_end;
	// ---- the configuration ----
	const uint8_t *rom;
	uint32_t rom_size, pcmlimit;
	void (*port_write)(uint8_t data);   // (register 12: the external port, usually the volume)
} k007232_t;

// the rate of its samples: clock / 128
void k007232_init(k007232_t *k, const uint8_t *rom, uint32_t rom_size, void (*port_write)(uint8_t data));
void k007232_reset(k007232_t *k);
void k007232_write(k007232_t *k, int offset, uint8_t data);
uint8_t k007232_read(k007232_t *k, int offset);
void k007232_set_volume(k007232_t *k, int channel, int vol_a, int vol_b);
void k007232_set_bank(k007232_t *k, int chan_a_bank, int chan_b_bank);
// one sample at its rate: the two outputs (MAME's put_int(.., 32768): x 1, the sum of out x vol x 2)
void k007232_sample(k007232_t *k, int32_t *out_a, int32_t *out_b);
int k007232_state(k007232_t *k, void *ctx, int load, const char *tag);

#endif
