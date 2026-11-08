/*  Konami 051733. Ported from MAME src/mame/konami/k051733.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria).
    See k051733.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "k051733.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

void k051733_reset(k051733_t *k)
{
	memset(k->ram, 0, sizeof(k->ram));
	k->lfsr = 0xff;
	k->nmi_timer = 0;
	k->nmi_clock = 0;
}

static void clock_lfsr(k051733_t *k)
{
	const int feedback = ((k->lfsr >> 1) ^ (k->lfsr >> 8) ^ (k->lfsr >> 12)) & 1;
	k->lfsr = (uint16_t)(k->lfsr << 1 | feedback);
}

static uint32_t u32_sqrt(uint32_t op) { return op ? (uint32_t)sqrt((double)op) & ~1u : 0; }

uint8_t k051733_read(k051733_t *k, int offset)
{
	offset &= 0x07;
	const uint8_t lfsr = (uint8_t)k->lfsr;
	clock_lfsr(k);
	const uint8_t *r = k->ram;
	const int op1 = (int16_t)(r[0x00] << 8 | r[0x01]), op2 = (int16_t)(r[0x02] << 8 | r[0x03]);
	const uint32_t op3 = (uint32_t)(r[0x04] << 8 | r[0x05]);
	const int rad = r[0x06] << 8 | r[0x07], y1 = r[0x08] << 8 | r[0x09], x1 = r[0x0a] << 8 | r[0x0b];
	const int y2 = r[0x0c] << 8 | r[0x0d], x2 = r[0x0e] << 8 | r[0x0f];
	switch (offset) {
	case 0x00: return op2 ? (uint8_t)((op1 / op2) >> 8) : 0;
	case 0x01: return op2 ? (uint8_t)(op1 / op2) : 0;
	case 0x02: return op2 ? (uint8_t)((op1 % op2) >> 8) : (uint8_t)(op1 >> 8);
	case 0x03: return op2 ? (uint8_t)(op1 % op2) : (uint8_t)op1;
	case 0x04: return (uint8_t)(u32_sqrt(op3 << 16) >> 8);
	case 0x05: return (uint8_t)u32_sqrt(op3 << 16);
	case 0x06: return lfsr;
	default: {   // (the direction from (x1, y1) to (x2, y2) within the radius)
		const int dx = x2 - x1, dy = y2 - y1;
		if (abs(dx) > rad || abs(dy) > rad)
			return 0xff;
		if (dy <= 0) {
			if (dx <= 0)
				return (dy >= dx) ? 0x00 : 0x06;
			return (dy >= -dx) ? 0x04 : 0x07;
		}
		if (dx <= 0)
			return (dy > -dx) ? 0x02 : 0x01;
		return (dy > dx) ? 0x03 : 0x05;
	}
	}
}

void k051733_write(k051733_t *k, int offset, uint8_t data)
{
	k->ram[offset & 0x1f] = data;
	clock_lfsr(k);
}

int k051733_nmiclock(k051733_t *k, int state)
{
	int fire = 0;
	if (!k->nmi_clock && state && !k->nmi_timer--) {
		fire = k->ram[0x12] & 1;
		k->nmi_timer = (uint16_t)(k->ram[0x10] << 8 | k->ram[0x11]);
	}
	k->nmi_clock = (uint8_t)(state != 0);
	return fire;
}
