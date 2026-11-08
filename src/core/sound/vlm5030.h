/*  Sanyo VLM5030 LPC speech synthesizer. Ported from MAME src/devices/sound/vlm5030.cpp (BSD-3-Clause,
    copyright-holders: Tatsuyuki Satoh) and its coefficient table from src/devices/sound/tms5110r.hxx (BSD-3-Clause,
    copyright-holders: Frank Palazzolo, Couriersud, Jonathan Gevaryahu).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_VLM5030_H
#define RBX_VLM5030_H

#include <stdint.h>

typedef struct {
	// ---- saved ----
	uint16_t address;
	uint8_t pin_bsy, pin_st, pin_vcu, pin_rst;
	uint8_t latch_data;
	uint16_t vcu_addr_h;
	uint8_t parameter;
	uint8_t phase;
	// (the option parameter: MAME rebuilds these from parameter after a load, saved here as they are)
	int32_t frame_size, pitch_offset;
	uint8_t interp_step;
	uint8_t interp_count, sample_count, pitch_count;
	uint16_t old_energy;
	uint8_t old_pitch;
	int16_t old_k[10];
	uint16_t target_energy;
	uint8_t target_pitch;
	int16_t target_k[10];
	uint16_t new_energy;
	uint8_t new_pitch;
	int16_t new_k[10];
	uint32_t current_energy, current_pitch;
	int32_t current_k[10];
	int32_t x[10];
	uint32_t rand_seed;   // (MAME's machine().rand() for the unvoiced frames)
	uint8_t state_end;
	// ---- the configuration ----
	const uint8_t *rom;   // the speech data (a power of 2 in size, mirrored over the 16-bit space); may be RAM and
	uint32_t rom_size;    // may be changed at any time
	uint32_t clock;
} vlm5030_t;

void vlm5030_init(vlm5030_t *v, uint32_t clock, const uint8_t *rom, uint32_t rom_size);   // (device_start)
void vlm5030_reset(vlm5030_t *v);   // (device_reset)
// The pins. MAME brings its stream up to the current time before rst / st / bsy: render up to now before calling them.
void vlm5030_data_w(vlm5030_t *v, uint8_t data);
void vlm5030_rst(vlm5030_t *v, int state);
void vlm5030_st(vlm5030_t *v, int state);
void vlm5030_vcu(vlm5030_t *v, int state);
int vlm5030_bsy(vlm5030_t *v);
uint32_t vlm5030_rate(const vlm5030_t *v);   // (clock / 440)
// n samples at that rate: MAME's put_int_clamp(.., u, 512) as an int16 (clamp(u, -512, 511) x 64)
void vlm5030_render(vlm5030_t *v, int16_t *out, int n);
int vlm5030_state(vlm5030_t *v, void *ctx, int load, const char *tag);

#endif
