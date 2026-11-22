/*  Harris HC-55516 CVSD decoder, ported to C from MAME src/devices/sound/hc55516.cpp (BSD-3-Clause,
    copyright-holders: Aaron Giles, Jonathan Gevaryahu; thanks-to: Zonn Moore).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "hc55516.h"
#include <string.h>

#define SYLMASK  0xfc0
#define SYLSHIFT 6
#define SYLADD   0xfc1
#define INTSHIFT 4
#define SHIFTREG_MASK 0x7

static inline int32_t sext10(int32_t v) { return (int32_t)((uint32_t)v << 22) >> 22; }

void hc55516_reset(hc55516_t *c)
{
	uint8_t sr = c->shiftreg, bit = c->buffered_bit;   // (not reset by MAME)
	int32_t ns = c->next_sample;
	memset(c, 0, sizeof(*c));
	c->shiftreg = sr;
	c->buffered_bit = bit;
	c->next_sample = ns;
	c->last_clock_state = 0;
	c->sylfilter = 0x3f;
	c->intfilter = 0;
	c->agc = 1;
	c->buffered_fzq = 1;
}

void hc55516_digit_w(hc55516_t *c, int digit) { c->buffered_bit = digit ? 1 : 0; }

static void process_bit(hc55516_t *c, int bit, int clock_state)
{
	const int frozen = ((c->intfilter >= 0x180) && !bit) || ((c->intfilter <= -0x180) && bit);
	int32_t sum;
	if (clock_state != c->last_clock_state && clock_state == 1) {   // (RISING: the active edge)
		if (!c->buffered_fzq)
			bit = !(c->shiftreg & 1);
		c->shiftreg = (uint8_t)((c->shiftreg << 1) | (bit ? 1 : 0));
		if (((c->shiftreg & SHIFTREG_MASK) == 0) || ((c->shiftreg & SHIFTREG_MASK) == SHIFTREG_MASK)) {
			if (!frozen) c->sylfilter += (((~c->sylfilter) & SYLMASK) >> SYLSHIFT);
		} else {
			if (!frozen) c->sylfilter += (((~c->sylfilter) & SYLMASK) >> SYLSHIFT) + SYLADD;
		}
		c->sylfilter &= 0xfff;
		sum = sext10(((~c->intfilter) >> INTSHIFT) + 1);
	} else {
		int32_t m = c->sylfilter >> 6;
		if (m < 2) m = 2;
		sum = (c->shiftreg & 1) ? sext10((~m) + 1) : sext10(m);
	}
	if (!frozen)
		c->intfilter = sext10(c->intfilter + sum);
	c->next_sample = (int32_t)(int16_t)((c->intfilter << 6) | (((c->intfilter & 0x3ff) ^ 0x200) >> 4));
	c->agc = ((c->intfilter >= 0x100) || (c->intfilter <= -0x100)) ? 0 : 1;
}

void hc55516_clock_w(hc55516_t *c, int state)
{
	int clock_state = state ? 1 : 0;
	if (clock_state != c->last_clock_state)
		process_bit(c, c->buffered_bit, clock_state);
	c->last_clock_state = (uint8_t)clock_state;
}
