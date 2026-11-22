/*  NEC MC-8123 (a Z80 decrypting its ROM by 12 bits of the address and a 8 KB key, opcodes and data apart).
    Ported from MAME src/devices/cpu/z80/mc8123.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria, David Widel).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "mc8123.h"

#define BIT(x, n) (((x) >> (n)) & 1)
static inline uint32_t bsn(uint32_t v, int n, const int *b)
{
	uint32_t r = 0;
	for (int i = 0; i < n; i++)
		r |= (uint32_t)(BIT(v, b[i]) << (n - 1 - i));
	return r;
}
#define BS8(v, ...) ((uint8_t)bsn((uint32_t)(v), 8, (const int[]){ __VA_ARGS__ }))
#define BS12(v, ...) bsn((uint32_t)(v), 12, (const int[]){ __VA_ARGS__ })
static inline uint8_t bits_(const int *b, int n)
{
	uint8_t r = 0;
	for (int i = 0; i < n; i++)
		r |= (uint8_t)(1u << b[i]);
	return r;
}
#define BITS(...) bits_((const int[]){ __VA_ARGS__ }, (int)(sizeof((const int[]){ __VA_ARGS__ }) / sizeof(int)))

static uint8_t decrypt_type0(uint8_t val, uint8_t param, unsigned swap)
{
	if (swap == 0) val = BS8(val,7,5,3,1,2,0,6,4);
	if (swap == 1) val = BS8(val,5,3,7,2,1,0,4,6);
	if (swap == 2) val = BS8(val,0,3,4,6,7,1,5,2);
	if (swap == 3) val = BS8(val,0,7,3,2,6,4,1,5);

	if (BIT(param,3) && BIT(val,7))
		val ^= BITS(5,3,0);

	if (BIT(param,2) && BIT(val,6))
		val ^= BITS(7,2,1);

	if (BIT(val,6)) val ^= BITS(7);

	if (BIT(param,1) && BIT(val,7))
		val ^= BITS(6);

	if (BIT(val,2)) val ^= BITS(5,0);

	val ^= BITS(4,3,1);

	if (BIT(param,2)) val ^= BITS(5,2,0);
	if (BIT(param,1)) val ^= BITS(7,6);
	if (BIT(param,0)) val ^= BITS(5,0);

	if (BIT(param,0)) val = BS8(val,7,6,5,1,4,3,2,0);

	return val;
}

static uint8_t decrypt_type1a(uint8_t val, uint8_t param, unsigned swap)
{
	if (swap == 0) val = BS8(val,4,2,6,5,3,7,1,0);
	if (swap == 1) val = BS8(val,6,0,5,4,3,2,1,7);
	if (swap == 2) val = BS8(val,2,3,6,1,4,0,7,5);
	if (swap == 3) val = BS8(val,6,5,1,3,2,7,0,4);

	if (BIT(param,2)) val = BS8(val,7,6,1,5,3,2,4,0);

	if (BIT(val,1)) val ^= BITS(0);
	if (BIT(val,6)) val ^= BITS(3);
	if (BIT(val,7)) val ^= BITS(6,3);
	if (BIT(val,2)) val ^= BITS(6,3,1);
	if (BIT(val,4)) val ^= BITS(7,6,2);

	if (BIT(val,7) ^ BIT(val,2))
		val ^= BITS(4);

	val ^= BITS(6,3,1,0);

	if (BIT(param,3)) val ^= BITS(7,2);
	if (BIT(param,1)) val ^= BITS(6,3);

	if (BIT(param,0)) val = BS8(val,7,6,1,4,3,2,5,0);

	return val;
}

static uint8_t decrypt_type1b(uint8_t val, uint8_t param, unsigned swap)
{
	if (swap == 0) val = BS8(val,1,0,3,2,5,6,4,7);
	if (swap == 1) val = BS8(val,2,0,5,1,7,4,6,3);
	if (swap == 2) val = BS8(val,6,4,7,2,0,5,1,3);
	if (swap == 3) val = BS8(val,7,1,3,6,0,2,5,4);

	if (BIT(val,2) && BIT(val,0))
		val ^= BITS(7,4);

	if (BIT(val,7)) val ^= BITS(2);
	if (BIT(val,5)) val ^= BITS(7,2);
	if (BIT(val,1)) val ^= BITS(5);
	if (BIT(val,6)) val ^= BITS(1);
	if (BIT(val,4)) val ^= BITS(6,5);
	if (BIT(val,0)) val ^= BITS(6,2,1);
	if (BIT(val,3)) val ^= BITS(7,6,2,1,0);

	val ^= BITS(6,4,0);

	if (BIT(param,3)) val ^= BITS(4,1);
	if (BIT(param,2)) val ^= BITS(7,6,3,0);
	if (BIT(param,1)) val ^= BITS(4,3);
	if (BIT(param,0)) val ^= BITS(6,2,1,0);

	return val;
}

static uint8_t decrypt_type2a(uint8_t val, uint8_t param, unsigned swap)
{
	if (swap == 0) val = BS8(val,0,1,4,3,5,6,2,7);
	if (swap == 1) val = BS8(val,6,3,0,5,7,4,1,2);
	if (swap == 2) val = BS8(val,1,6,4,5,0,3,7,2);
	if (swap == 3) val = BS8(val,4,6,7,5,2,3,1,0);

	if (BIT(val,3) || (BIT(param,1) && BIT(val,2)))
		val = BS8(val,6,0,7,4,3,2,1,5);

	if (BIT(val,5)) val ^= BITS(7);
	if (BIT(val,6)) val ^= BITS(5);
	if (BIT(val,0)) val ^= BITS(6);
	if (BIT(val,4)) val ^= BITS(3,0);
	if (BIT(val,1)) val ^= BITS(2);

	val ^= BITS(7,6,5,4,1);

	if (BIT(param,2)) val ^= BITS(4,3,2,1,0);

	if (BIT(param,3))
	{
		if (BIT(param,0))
			val = BS8(val,7,6,5,3,4,1,2,0);
		else
			val = BS8(val,7,6,5,1,2,4,3,0);
	}
	else if (BIT(param,0))
	{
		val = BS8(val,7,6,5,2,1,3,4,0);
	}

	return val;
}

static uint8_t decrypt_type2b(uint8_t val, uint8_t param, unsigned swap)
{
	// only 0x20 possible encryptions for this method - all others have 0x40
	// this happens because BIT(param,2) cancels the other three

	if (swap == 0) val = BS8(val,1,3,4,6,5,7,0,2);
	if (swap == 1) val = BS8(val,0,1,5,4,7,3,2,6);
	if (swap == 2) val = BS8(val,3,5,4,1,6,2,0,7);
	if (swap == 3) val = BS8(val,5,2,3,0,4,7,6,1);

	if (BIT(val,7) && BIT(val,3))
		val ^= BITS(6,4,0);

	if (BIT(val,7)) val ^= BITS(2);
	if (BIT(val,5)) val ^= BITS(7,3);
	if (BIT(val,1)) val ^= BITS(5);
	if (BIT(val,4)) val ^= BITS(7,5,3,1);

	if (BIT(val,7) && BIT(val,5))
		val ^= BITS(4,0);

	if (BIT(val,5) && BIT(val,1))
		val ^= BITS(4,0);

	if (BIT(val,6)) val ^= BITS(7,5);
	if (BIT(val,3)) val ^= BITS(7,6,5,1);
	if (BIT(val,2)) val ^= BITS(3,1);

	val ^= BITS(7,3,2,1);

	if (BIT(param,3)) val ^= BITS(6,3,1);
	if (BIT(param,2)) val ^= BITS(7,6,5,3,2,1); // same as the other three combined
	if (BIT(param,1)) val ^= BITS(7);
	if (BIT(param,0)) val ^= BITS(5,2);

	return val;
}

static uint8_t decrypt_type3a(uint8_t val, uint8_t param, unsigned swap)
{
	if (swap == 0) val = BS8(val,5,3,1,7,0,2,6,4);
	if (swap == 1) val = BS8(val,3,1,2,5,4,7,0,6);
	if (swap == 2) val = BS8(val,5,6,1,2,7,0,4,3);
	if (swap == 3) val = BS8(val,5,6,7,0,4,2,1,3);

	if (BIT(val,2)) val ^= BITS(7,5,4);
	if (BIT(val,3)) val ^= BITS(0);

	if (BIT(param,0)) val = BS8(val,7,2,5,4,3,1,0,6);

	if (BIT(val,1)) val ^= BITS(6,0);
	if (BIT(val,3)) val ^= BITS(4,2,1);

	if (BIT(param,3)) val ^= BITS(4,3);

	if (BIT(val,3)) val = BS8(val,5,6,7,4,3,2,1,0);

	if (BIT(val,5)) val ^= BITS(2,1);

	val ^= BITS(6,5,4,3);

	if (BIT(param,2)) val ^= BITS(7);
	if (BIT(param,1)) val ^= BITS(4);
	if (BIT(param,0)) val ^= BITS(0);

	return val;
}

static uint8_t decrypt_type3b(uint8_t val, uint8_t param, unsigned swap)
{
	if (swap == 0) val = BS8(val,3,7,5,4,0,6,2,1);
	if (swap == 1) val = BS8(val,7,5,4,6,1,2,0,3);
	if (swap == 2) val = BS8(val,7,4,3,0,5,1,6,2);
	if (swap == 3) val = BS8(val,2,6,4,1,3,7,0,5);

	if (BIT(val,2)) val ^= BITS(7);

	if (BIT(val,7)) val = BS8(val,7,6,3,4,5,2,1,0);

	if (BIT(param,3)) val ^= BITS(7);

	if (BIT(val,4)) val ^= BITS(6);
	if (BIT(val,1)) val ^= BITS(6,4,2);

	if (BIT(val,7) && BIT(val,6))
		val ^= BITS(1);

	if (BIT(val,7)) val ^= BITS(1);

	if (BIT(param,3)) val ^= BITS(7);
	if (BIT(param,2)) val ^= BITS(0);

	if (BIT(param,3)) val = BS8(val,4,6,3,2,5,0,1,7);

	if (BIT(val,4)) val ^= BITS(1);
	if (BIT(val,5)) val ^= BITS(4);
	if (BIT(val,7)) val ^= BITS(2);

	val ^= BITS(5,3,2);

	if (BIT(param,1)) val ^= BITS(7);
	if (BIT(param,0)) val ^= BITS(3);

	return val;
}

static uint8_t decrypt_internal(uint8_t val, uint8_t key, int opcode)
{
	unsigned type = 0;
	unsigned swap = 0;
	uint8_t param = 0;

	key ^= 0xff;

	// no encryption
	if (key == 0x00)
		return val;

	type ^= BIT(key,0) << 0;
	type ^= BIT(key,2) << 0;
	type ^= BIT(key,0) << 1;
	type ^= BIT(key,1) << 1;
	type ^= BIT(key,2) << 1;
	type ^= BIT(key,4) << 1;
	type ^= BIT(key,4) << 2;
	type ^= BIT(key,5) << 2;

	swap ^= BIT(key,0) << 0;
	swap ^= BIT(key,1) << 0;
	swap ^= BIT(key,2) << 1;
	swap ^= BIT(key,3) << 1;

	param ^= BIT(key,0) << 0;
	param ^= BIT(key,0) << 1;
	param ^= BIT(key,2) << 1;
	param ^= BIT(key,3) << 1;
	param ^= BIT(key,0) << 2;
	param ^= BIT(key,1) << 2;
	param ^= BIT(key,6) << 2;
	param ^= BIT(key,1) << 3;
	param ^= BIT(key,6) << 3;
	param ^= BIT(key,7) << 3;

	if (!opcode)
	{
		param ^= BITS(0);
		type ^= BITS(0);
	}

	switch (type)
	{
		default:
		case 0: return decrypt_type0(val, param, swap);
		case 1: return decrypt_type0(val, param, swap);
		case 2: return decrypt_type1a(val, param, swap);
		case 3: return decrypt_type1b(val, param, swap);
		case 4: return decrypt_type2a(val, param, swap);
		case 5: return decrypt_type2b(val, param, swap);
		case 6: return decrypt_type3a(val, param, swap);
		case 7: return decrypt_type3b(val, param, swap);
	}
}

static uint8_t decrypt(const uint8_t *m_key, uint32_t addr, uint8_t val, int opcode)
{
	// pick the translation table from bits fd57 of the address
	const uint32_t tbl_num = BS12(addr,15,14,13,12,11,10,8,6,4,2,1,0);

	return decrypt_internal(val, m_key[tbl_num | (opcode ? 0x0000 : 0x1000)], opcode);
}

void mc8123_decode(uint8_t *rom, uint8_t *opcodes, unsigned length, const uint8_t *key)
{
	for (unsigned i = 0; i < length; i++)
	{
		const unsigned adr = (i >= 0xc000) ? ((i & 0x3fff) | 0x8000) : i;
		const uint8_t src = rom[i];

		// decode the opcodes
		opcodes[i] = decrypt(key, adr, src, 1);

		// decode the data
		rom[i] = decrypt(key, adr, src, 0);
	}
}
