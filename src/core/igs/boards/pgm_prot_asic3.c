/*  PGM protection: ASIC3 of Oriental Legend, ported from MAME src/mame/igs/pgmprot_orlegend.cpp (BSD-3-Clause,
    copyright-holders: Olivier Galibert, iq_132).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "pgm.h"

#include <string.h>

static struct {
	uint8_t reg, latch[3], x, pad;
	uint16_t hilo, hold;
} s_a3;

#define BIT(v, n) (((v) >> (n)) & 1)

static void compute_hold(int y, int z)
{
	const uint16_t old = s_a3.hold;
	uint16_t h = (uint16_t)((old << 1) | (old >> 15));
	h ^= 0x2bad;
	h ^= (uint16_t)BIT(z, y);
	h ^= (uint16_t)(BIT(s_a3.x, 2) << 10);
	h ^= (uint16_t)BIT(old, 5);
	const int x = s_a3.x;
	switch (pgm_region_port()) {
	case 0:
	case 1: h ^= (uint16_t)(BIT(old, 10) ^ BIT(old, 8) ^ (BIT(x, 0) << 1) ^ (BIT(x, 1) << 6) ^ (BIT(x, 3) << 14)); break;
	case 2: h ^= (uint16_t)(BIT(old, 10) ^ BIT(old, 8) ^ (BIT(x, 0) << 4) ^ (BIT(x, 1) << 6) ^ (BIT(x, 3) << 12)); break;
	case 3: h ^= (uint16_t)(BIT(old, 7) ^ BIT(old, 6) ^ (BIT(x, 0) << 4) ^ (BIT(x, 1) << 6) ^ (BIT(x, 3) << 12)); break;
	case 4: h ^= (uint16_t)(BIT(old, 7) ^ BIT(old, 6) ^ (BIT(x, 0) << 3) ^ (BIT(x, 1) << 8) ^ (BIT(x, 3) << 14)); break;
	}
	s_a3.hold = h;
}

static uint16_t asic3_r(void)
{
	const uint32_t region = pgm_region_port();
	switch (s_a3.reg) {
	case 0x00: return (uint16_t)((s_a3.latch[0] & 0xf7) | ((region << 3) & 0x08));
	case 0x01: return s_a3.latch[1];
	case 0x02: return (uint16_t)((s_a3.latch[2] & 0x7f) | ((region << 6) & 0x80));
	case 0x03: {   // bitswap<8>(hold, 5,2,9,7,10,13,12,15)
		static const int b[8] = { 5, 2, 9, 7, 10, 13, 12, 15 };
		uint16_t r = 0;
		for (int i = 0; i < 8; i++)
			r |= (uint16_t)(BIT(s_a3.hold, b[i]) << (7 - i));
		return r;
	}
	case 0x20: return 0x49;
	case 0x21: return 0x47;
	case 0x22: return 0x53;
	case 0x24: return 0x41;
	case 0x25: return 0x41;
	case 0x26: return 0x7f;
	case 0x27: return 0x41;
	case 0x28: return 0x41;
	case 0x2a: return 0x3e;
	case 0x2b: return 0x41;
	case 0x2c: return 0x49;
	case 0x2d: return 0xf9;
	case 0x2e: return 0x0a;
	case 0x30: return 0x26;
	case 0x31: return 0x49;
	case 0x32: return 0x49;
	case 0x33: return 0x49;
	case 0x34: return 0x32;
	}
	return 0;
}

static void asic3_w(int offset, uint16_t data)
{
	if (offset == 0) {
		s_a3.reg = (uint8_t)data;
		return;
	}
	switch (s_a3.reg) {
	case 0x00: case 0x01: case 0x02:
		s_a3.latch[s_a3.reg] = (uint8_t)(data << 1);
		break;
	case 0x40:
		s_a3.hilo = (uint16_t)((s_a3.hilo << 8) | data);
		break;
	case 0x48:
		s_a3.x = 0;
		if ((s_a3.hilo & 0x0090) == 0) s_a3.x |= 0x01;
		if ((s_a3.hilo & 0x0006) == 0) s_a3.x |= 0x02;
		if ((s_a3.hilo & 0x9000) == 0) s_a3.x |= 0x04;
		if ((s_a3.hilo & 0x0a00) == 0) s_a3.x |= 0x08;
		break;
	case 0x80: case 0x81: case 0x82: case 0x83: case 0x84: case 0x85: case 0x86: case 0x87:
		compute_hold(s_a3.reg & 7, data);
		break;
	case 0xa0:
		s_a3.hold = 0;
		break;
	}
}

static int a3_read16(uint32_t a, uint16_t *v)
{
	if (a < 0xc04000 || a > 0xc0400f)
		return 0;
	*v = asic3_r();
	return 1;
}

static int a3_write16(uint32_t a, uint16_t v, uint16_t mask)
{
	(void)mask;
	if (a < 0xc04000 || a > 0xc0400f)
		return 0;
	asic3_w((int)((a - 0xc04000) >> 1), v);
	return 1;
}

static int a3_init(const char *init)
{
	(void)init;
	memset(&s_a3, 0, sizeof(s_a3));
	return 0;
}

static int a3_state(void *ctx, int load)
{
	(void)load;
	return rbx_state_io(ctx, "asic3", &s_a3, sizeof(s_a3));
}

// (the state is set in init only, as MAME: not on reset)
const pgm_prot pgm_prot_asic3 = { a3_init, NULL, NULL, a3_read16, a3_write16, NULL, NULL, a3_state, 0 };
