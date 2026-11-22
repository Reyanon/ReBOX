/*  Capcom Kabuki Z80 decryption, ported from MAME src/mame/capcom/kabuki.cpp (BSD-3-Clause, copyright-holders:
    Nicola Salmoria).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "kabuki.h"

static int bitswap1(int src, int key, int select)
{
	if (select & (1 << ((key >> 0) & 7))) src = (src & 0xfc) | ((src & 0x01) << 1) | ((src & 0x02) >> 1);
	if (select & (1 << ((key >> 4) & 7))) src = (src & 0xf3) | ((src & 0x04) << 1) | ((src & 0x08) >> 1);
	if (select & (1 << ((key >> 8) & 7))) src = (src & 0xcf) | ((src & 0x10) << 1) | ((src & 0x20) >> 1);
	if (select & (1 << ((key >> 12) & 7))) src = (src & 0x3f) | ((src & 0x40) << 1) | ((src & 0x80) >> 1);
	return src;
}
static int bitswap2(int src, int key, int select)
{
	if (select & (1 << ((key >> 12) & 7))) src = (src & 0xfc) | ((src & 0x01) << 1) | ((src & 0x02) >> 1);
	if (select & (1 << ((key >> 8) & 7))) src = (src & 0xf3) | ((src & 0x04) << 1) | ((src & 0x08) >> 1);
	if (select & (1 << ((key >> 4) & 7))) src = (src & 0xcf) | ((src & 0x10) << 1) | ((src & 0x20) >> 1);
	if (select & (1 << ((key >> 0) & 7))) src = (src & 0x3f) | ((src & 0x40) << 1) | ((src & 0x80) >> 1);
	return src;
}
static int bytedecode(int src, uint32_t k1, uint32_t k2, int xor_key, int select)
{
	src = bitswap1(src, (int)(k1 & 0xffff), select & 0xff);
	src = ((src & 0x7f) << 1) | ((src & 0x80) >> 7);
	src = bitswap2(src, (int)(k1 >> 16), select & 0xff);
	src ^= xor_key;
	src = ((src & 0x7f) << 1) | ((src & 0x80) >> 7);
	src = bitswap2(src, (int)(k2 & 0xffff), select >> 8);
	src = ((src & 0x7f) << 1) | ((src & 0x80) >> 7);
	src = bitswap1(src, (int)(k2 >> 16), select >> 8);
	return src;
}
static void kabuki_decode(uint8_t *src, uint8_t *dop, int base, int len, uint32_t k1, uint32_t k2, int ak, int xk)
{
	for (int a = 0; a < len; a++) {
		dop[a] = (uint8_t)bytedecode(src[a], k1, k2, xk, (a + base) + ak);
		src[a] = (uint8_t)bytedecode(src[a], k1, k2, xk, ((a + base) ^ 0x1fc0) + ak + 1);
	}
}
void kabuki_mitchell_decode(uint8_t *src, uint8_t *dst, uint32_t size, uint32_t k1, uint32_t k2, int ak, int xk)
{
	const int numbanks = (int)(size - 0x10000) / 0x4000;
	kabuki_decode(src, dst, 0, 0x8000, k1, k2, ak, xk);
	for (int i = 0; i < numbanks; i++)
		kabuki_decode(src + 0x10000 + i * 0x4000, dst + 0x10000 + i * 0x4000, 0x8000, 0x4000, k1, k2, ak, xk);
}

