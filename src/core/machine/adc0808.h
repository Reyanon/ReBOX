/*  ADC0808 / ADC0809 8-channel A/D converter, timing after MAME src/devices/machine/adc0808.cpp (BSD-3-Clause,
    copyright-holders: Dirk Best): a start (address_offset_start_w) begins a conversion that ends 66 converter clocks
    later (start, ready, 64 clocks of conversion); the input is sampled at the end, EOC rises and the EOC flip-flop is
    set (cleared by a data read). The board advances it in converter clocks and samples its inputs on completion.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_ADC0808_H
#define RBX_ADC0808_H

#include <stdint.h>

typedef struct {
	int32_t remain;      // converter clocks to the end of the running conversion (0 = idle)
	uint8_t addr, sar, eoc, eoc_ff;
} adc0808_t;

static inline void adc0808_reset(adc0808_t *a) { a->remain = 0; a->addr = 0; a->sar = 0xff; a->eoc = 1; a->eoc_ff = 0; }
static inline void adc0808_start(adc0808_t *a, int channel)
{
	a->addr = (uint8_t)(channel & 7);
	a->remain = 66;
	a->eoc = 0;
}
static inline uint8_t adc0808_data_r(adc0808_t *a) { a->eoc_ff = 0; return a->sar; }
// advance; returns 1 when a conversion ended (EOC rising edge): the board then stores its input in sar
static inline int adc0808_advance(adc0808_t *a, int32_t clocks)
{
	if (a->remain <= 0)
		return 0;
	a->remain -= clocks;
	if (a->remain > 0)
		return 0;
	a->remain = 0;
	a->eoc = 1;
	a->eoc_ff = 1;
	return 1;
}

#endif
