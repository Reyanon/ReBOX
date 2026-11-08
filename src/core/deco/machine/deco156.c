/*  Data East 156: the encrypted ARM's program decryption, ported to C from MAME src/mame/dataeast/deco156.cpp
    (BSD-3-Clause, copyright-holders: Bryan McPhail).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "deco156.h"

#include <stdlib.h>
#include <string.h>

// MAME bitswap<32>: result bit 31 - k is source bit b[k]
static uint32_t bitswap32(uint32_t x, const uint8_t *b)
{
	uint32_t r = 0;
	for (int k = 0; k < 32; k++)
		r |= ((x >> b[k]) & 1u) << (31 - k);
	return r;
}

static const uint32_t s_xor[4] = { 0xec63197a, 0x58a5a55f, 0xe3a65f16, 0x28d93783 };
static const uint8_t s_swap[4][32] = {
	{ 1, 4, 7, 28, 22, 18, 20, 9, 16, 10, 30, 2, 31, 24, 19, 29, 6, 21, 23, 11, 12, 13, 5, 0, 8, 26, 27, 15, 14, 17, 25, 3 },
	{ 14, 23, 28, 29, 6, 24, 10, 1, 5, 16, 7, 2, 30, 8, 18, 3, 31, 22, 25, 20, 17, 0, 19, 27, 9, 12, 21, 15, 26, 13, 4, 11 },
	{ 19, 30, 21, 4, 2, 18, 15, 1, 12, 25, 8, 0, 24, 20, 17, 23, 22, 26, 28, 16, 9, 27, 6, 11, 31, 10, 3, 13, 14, 7, 29, 5 },
	{ 30, 6, 15, 0, 31, 18, 26, 22, 14, 23, 19, 17, 10, 8, 11, 20, 1, 28, 2, 4, 9, 24, 25, 27, 7, 21, 13, 29, 5, 3, 16, 12 },
};

static uint32_t rd32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

int deco156_decrypt(uint8_t *rom, uint32_t size)
{
	uint8_t *buf = (uint8_t *)malloc(size);
	if (!buf)
		return -1;
	memcpy(buf, rom, size);
	const uint32_t n = size / 4;
	for (uint32_t a = 0; a < n; a++) {
		uint32_t addr = (a & 0xff0000) | 0x92c6;
		if (a & 0x0001) addr ^= 0xce4a;
		if (a & 0x0002) addr ^= 0x4db2;
		if (a & 0x0004) addr ^= 0xef60;
		if (a & 0x0008) addr ^= 0x5737;
		if (a & 0x0010) addr ^= 0x13dc;
		if (a & 0x0020) addr ^= 0x4bd9;
		if (a & 0x0040) addr ^= 0xa209;
		if (a & 0x0080) addr ^= 0xd996;
		if (a & 0x0100) addr ^= 0xa700;
		if (a & 0x0200) addr ^= 0xeca0;
		if (a & 0x0400) addr ^= 0x7529;
		if (a & 0x0800) addr ^= 0x3100;
		if (a & 0x1000) addr ^= 0x33b4;
		if (a & 0x2000) addr ^= 0x6161;
		if (a & 0x4000) addr ^= 0x1eef;
		if (a & 0x8000) addr ^= 0xf5a5;
		// (MAME reads src[addr] of the whole region: a smaller region than 64K dwords wraps as the hardware's address lines)
		uint32_t dword = addr < n ? rd32(buf + 4 * addr) : 0;
		// each of these lines changes exactly two bits
		if (a & 0x00004) dword ^= 0x04400000;
		if (a & 0x00008) dword ^= 0x40000004;
		if (a & 0x00010) dword ^= 0x00048000;
		if (a & 0x00020) dword ^= 0x00000280;
		if (a & 0x00040) dword ^= 0x00200040;
		if (a & 0x00080) dword ^= 0x09000000;
		if (a & 0x00100) dword ^= 0x00001100;
		if (a & 0x00200) dword ^= 0x20002000;
		if (a & 0x00400) dword ^= 0x00000022;
		if (a & 0x00800) dword ^= 0x000a0000;
		if (a & 0x01000) dword ^= 0x10004000;
		if (a & 0x02000) dword ^= 0x00010400;
		if (a & 0x04000) dword ^= 0x80000010;
		if (a & 0x08000) dword ^= 0x00000009;
		if (a & 0x10000) dword ^= 0x02100000;
		if (a & 0x20000) dword ^= 0x00800800;
		dword = bitswap32(dword ^ s_xor[a & 3], s_swap[a & 3]);
		rom[4 * a] = (uint8_t)dword;
		rom[4 * a + 1] = (uint8_t)(dword >> 8);
		rom[4 * a + 2] = (uint8_t)(dword >> 16);
		rom[4 * a + 3] = (uint8_t)(dword >> 24);
	}
	free(buf);
	return 0;
}
