/*  Sega 315-5248 multiplier and 315-5249 divider (segaic16m.h). Ported from MAME src/mame/sega/segaic16_m.cpp
    (BSD-3-Clause, copyright-holders: Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "segaic16m.h"

uint16_t sega_mult_r(const uint16_t r[2], uint32_t off)
{
	const int32_t p = (int32_t)(int16_t)r[0] * (int16_t)r[1];
	switch (off & 3) {
	case 0: return r[0];
	case 1: return r[1];
	case 2: return (uint16_t)(p >> 16);
	default: return (uint16_t)p;
	}
}
void sega_mult_w(uint16_t r[2], uint32_t off, uint16_t v, uint16_t mask)
{
	uint16_t *w = &r[off & 1];
	*w = (uint16_t)((*w & ~mask) | (v & mask));
}

static void div_execute(uint16_t *r, int mode)
{
	r[6] = 0;
	if (mode == 0) {
		const int32_t dividend = (int32_t)((uint32_t)r[0] << 16 | r[1]);
		const int32_t divisor = (int16_t)r[2];
		int32_t q;
		if (divisor == 0) {
			q = dividend;
			r[6] |= 0x4000;
		} else
			q = dividend / divisor;
		if (q < -32768) {
			q = -32768;
			r[6] |= 0x8000;
		} else if (q > 32767) {
			q = 32767;
			r[6] |= 0x8000;
		}
		r[4] = (uint16_t)(int16_t)q;
		r[5] = (uint16_t)(int16_t)(dividend - q * divisor);
	} else {
		const uint32_t dividend = (uint32_t)r[0] << 16 | r[1];
		const uint32_t divisor = r[2];
		uint32_t q;
		if (divisor == 0) {
			q = dividend;
			r[6] |= 0x4000;
		} else
			q = dividend / divisor;
		r[4] = (uint16_t)(q >> 16);
		r[5] = (uint16_t)q;
	}
}
uint16_t sega_div_r(const uint16_t r[8], uint32_t off)
{
	switch (off & 7) {
	case 0: case 1: case 2: case 4: case 5: case 6: return r[off & 7];
	}
	return 0xffff;
}
void sega_div_w(uint16_t r[8], uint32_t off, uint16_t v, uint16_t mask)
{
	if ((off & 3) < 3) {
		uint16_t *w = &r[off & 3];
		*w = (uint16_t)((*w & ~mask) | (v & mask));
	}
	if (off & 8)
		div_execute(r, (off & 4) != 0);
}

