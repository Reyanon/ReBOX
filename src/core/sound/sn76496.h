/*  TI SN76496 family PSG, ported from MAME src/devices/sound/sn76496.cpp (BSD-3-Clause, copyright-holders:
    Nicola Salmoria). Mono variants (feedback mask, white noise taps, negate, clock divider as the MAME device
    constructors); the Game Gear stereo and the Sega / NCR quirks are not included.
    The native rate is clock / 2 (one sample per input clock pair, the chip is clocked every `clock_divider`).
    Save state: the struct up to SN76496_STATE_END is plain data.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_SN76496_H
#define RBX_SN76496_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	// ---- state ----
	int32_t vol_table[16];
	int32_t reg[8];
	int32_t last_register;
	int32_t volume[4];
	uint32_t rng;
	int32_t current_clock;
	int32_t period[4], count[4], output[4];
	uint32_t frac;           // input clocks not yet turned into a native sample
	uint32_t div_count;
	int32_t div_acc;
	int32_t dc;              // DC blocker state (x256)
	// ---- configuration ----
	uint32_t clock, divider;
	int32_t feedback_mask, tap1, tap2, negate, clock_divider;
	int gain, dc_block;
} sn76496_t;
#define SN76496_STATE_END offsetof(sn76496_t, clock)

// MAME device constructor values
#define SN76489A_CONFIG 0x10000, 0x04, 0x08, 0, 8
#define SN76489_CONFIG  0x4000, 0x01, 0x02, 1, 8
#define SN76496_CONFIG  0x10000, 0x04, 0x08, 0, 8

// gain = route gain in 1/256 (output 1.0 = 32768); divider = native samples averaged into one output sample
void sn76496_init(sn76496_t *p, uint32_t clock, int feedback_mask, int tap1, int tap2, int negate, int clock_divider,
                  int gain, int divider, int dc_block);
void sn76496_reset(sn76496_t *p);
uint32_t sn76496_rate(const sn76496_t *p);
void sn76496_write(sn76496_t *p, uint8_t data);
int sn76496_render(sn76496_t *p, uint32_t clocks, int32_t *out, int max);   // samples added to out[]
uint32_t sn76496_state_size(void);

#ifdef __cplusplus
}
#endif

#endif
