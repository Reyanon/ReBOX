/*  OKI MSM5205 ADPCM, ported from MAME src/devices/sound/msm5205.cpp (BSD-3-Clause, copyright-holders: Aaron Giles).
    Master mode only (prescaler 1/48, 1/64, 1/96): every sample period the board's VCK callback is called (MAME
    vck_legacy_callback, where the board feeds the next nibble with msm5205_data_w), then the nibble is decoded.
    Time advances through msm5205_advance() in master clocks.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_MSM5205_H
#define RBX_MSM5205_H

#include <stdint.h>

enum { MSM5205_S96_3B, MSM5205_S48_3B, MSM5205_S64_3B, MSM5205_SEX_3B,
       MSM5205_S96_4B, MSM5205_S48_4B, MSM5205_S64_4B, MSM5205_SEX_4B };

typedef struct {
	uint32_t clock;
	int prescaler, bitwidth;
	int data, reset, signal, step;
	uint32_t acc;                 // master clocks into the current sample period
	void (*vck)(void);            // legacy VCK callback (data latch)
} msm5205_t;

void msm5205_init(msm5205_t *m, uint32_t clock, int selector, void (*vck)(void));
void msm5205_reset(msm5205_t *m);
void msm5205_set_selector(msm5205_t *m, int selector);
static inline void msm5205_reset_w(msm5205_t *m, int state) { m->reset = state; }
static inline void msm5205_data_w(msm5205_t *m, uint8_t data)
{
	m->data = m->bitwidth == 4 ? (data & 0x0f) : ((data & 0x07) << 1);
}
void msm5205_advance(msm5205_t *m, uint32_t clocks);
// one ADPCM sample now (VCK slave mode: the board calls it on the falling edge of VCK, MAME vclk_w)
void msm5205_clock(msm5205_t *m);
// VCK slave mode: one ADPCM capture (MAME vclk_w falling edge); the board tracks the VCK level itself
void msm5205_vclk_capture(msm5205_t *m);
// current output (10-bit DAC), full scale +-16384 (MAME 1.0 = 32768, signal / 4096)
static inline int msm5205_output(const msm5205_t *m) { return (m->signal & ~3) * 8; }
// sample period in master clocks (0 = slave mode, not emulated)
static inline int msm5205_period(const msm5205_t *m) { return m->prescaler; }

#endif
