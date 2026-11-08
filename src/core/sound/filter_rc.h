/*  An RC filter as MAME's filter_rc_device (src/devices/sound/flt_rc.cpp, BSD-3-Clause, copyright-holders: Derrick
    Renaud, Couriersud): the low pass of LOWPASS_3R (the board's three resistors and capacitor, as the AY outputs of
    Konami's 8-bit sound boards), one sample at a time at the stream's rate.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_FILTER_RC_H
#define RBX_FILTER_RC_H

#include <math.h>
#include <stdint.h>

typedef struct {
	int32_t k;     // 16.16: 0x10000 = no filtering (C = 0)
	int32_t mem;
} filter_rc_t;

// filter_rc_set_RC(LOWPASS_3R, r1, r2, r3, c): c in farads, rate the samples per second
static inline void filter_rc_lowpass_3r(filter_rc_t *f, double r1, double r2, double r3, double c, double rate)
{
	if (c == 0.0) {
		f->k = 0x10000;
		return;
	}
	const double req = (r1 * (r2 + r3)) / (r1 + r2 + r3);
	f->k = (int32_t)(0x10000 - 0x10000 * exp(-1.0 / (req * c) / rate));
}

static inline int32_t filter_rc_step(filter_rc_t *f, int32_t in)
{
	f->mem += (int32_t)(((int64_t)(in - f->mem) * f->k) >> 16);
	return f->mem;
}

#endif
