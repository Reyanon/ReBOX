/*  Harris HC-55516 CVSD decoder with a software clock, ported to C from MAME src/devices/sound/hc55516.cpp
    (BSD-3-Clause, copyright-holders: Aaron Giles, Jonathan Gevaryahu; thanks-to: Zonn Moore). The output is the
    sample made at the last clock edge (MAME's stream holds it between edges).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_HC55516_H
#define RBX_HC55516_H
#include <stdint.h>
typedef struct {
	uint8_t last_clock_state, buffered_bit, shiftreg, agc, buffered_fzq, pad_[3];
	int32_t sylfilter, intfilter, next_sample;
} hc55516_t;
void hc55516_reset(hc55516_t *c);
void hc55516_digit_w(hc55516_t *c, int digit);
void hc55516_clock_w(hc55516_t *c, int state);
static inline int32_t hc55516_out(const hc55516_t *c) { return c->next_sample; }   // 16-bit scale
#endif
