/*  IGS036 program ROM decryption, ported from MAME src/mame/igs/igs036crypt.cpp (BSD-3-Clause, copyright-holders:
    Andreas Naive, David Haywood): igs036_decryptor::decrypter_rom / decrypt / deobfuscate / rotation / rot_enabled /
    rot_group and their tables, the same arithmetic. ReBOX: the final bitswap through two byte tables.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "igs036crypt.h"

#define BIT(x, n) (((x) >> (n)) & 1)

static int unknown(int a) { (void)a; return 0; }
static int cZero(int a) { (void)a; return 0; }
static int cOne(int a) { (void)a; return 1; }
static int bit_3(int a) { return BIT(a, 3); }
static int bit_4(int a) { return BIT(a, 4); }
static int bit_7(int a) { return BIT(a, 7); }
static int not_3(int a) { return BIT(a, 3) ^ 1; }
static int not_4(int a) { return BIT(a, 4) ^ 1; }
static int not_7(int a) { return BIT(a, 7) ^ 1; }
static int xor_37(int a) { return BIT(a, 3) ^ BIT(a, 7); }
static int xnor_37(int a) { return BIT(a, 3) ^ BIT(a, 7) ^ 1; }
static int xor_47(int a) { return BIT(a, 4) ^ BIT(a, 7); }
static int xnor_47(int a) { return BIT(a, 4) ^ BIT(a, 7) ^ 1; }
static int nor_34(int a) { return (BIT(a, 3) | BIT(a, 4)) ^ 1; }
static int impl_43(int a) { return BIT(a, 3) || (BIT(a, 4) ^ 1); }

static int (*const rot_enabling[16][4])(int) = {
	{ bit_3, not_3, bit_3, not_3 },
	{ bit_3, not_3, bit_3, not_3 },
	{ bit_4, bit_4, bit_4, bit_4 },
	{ bit_4, not_4, bit_4, not_4 },
	{ bit_3, bit_3, bit_3, bit_3 },
	{ nor_34, bit_7, bit_7, cZero },
	{ cZero, cOne, cZero, cOne },
	{ impl_43, xor_37, xnor_37, not_3 },
	{ bit_3, bit_3, not_3, not_3 },
	{ bit_4, bit_4, not_4, not_4 },
	{ cZero, cZero, cZero, cZero },
	{ nor_34, bit_7, not_7, cOne },
	{ bit_3, not_3, bit_3, not_3 },
	{ cZero, cOne, cOne, cZero },
	{ bit_4, not_4, bit_4, not_4 },
	{ unknown, unknown, unknown, unknown },
};

static int (*const rot_direction[4][8])(int) = {
	{ bit_3, xor_37, xnor_37, not_3, bit_3, xor_37, xnor_37, not_3 },
	{ cZero, not_7, not_7, cZero, cZero, not_7, not_7, cZero },
	{ bit_4, xor_47, xnor_47, not_4, bit_4, xor_47, xnor_47, not_4 },
	{ bit_3, not_7, bit_7, cZero, cOne, not_7, bit_7, cZero },
};

// the triggers describe under what conditions are every one of the 16 XORs activated
static const uint32_t triggers[16][2] = {
	{ 0x000101, 0x000001 }, { 0x000802, 0x000800 }, { 0x000204, 0x000004 }, { 0x000408, 0x000408 },
	{ 0x010010, 0x000010 }, { 0x020020, 0x000020 }, { 0x040040, 0x000040 }, { 0x080080, 0x080080 },
	{ 0x100100, 0x000100 }, { 0x200200, 0x200000 }, { 0x400400, 0x400000 }, { 0x800801, 0x000001 },
	{ 0x001004, 0x001000 }, { 0x002010, 0x002000 }, { 0x004040, 0x000040 }, { 0x008100, 0x008100 }
};

static int rot_enabled(int address, const int *group)
{
	int enabled = 0;
	for (int j = 0; j < 4; ++j) {
		if (BIT(address, 8 + group[j])) {
			const int aux = address ^ (0x1b * BIT(address, 2));
			enabled = rot_enabling[group[j]][aux & 3](aux);
			break;
		}
	}
	return enabled;
}

static int rot_group(int address, const int *group)
{
	const int aux = rot_direction[group[0] & 3][address & 7](address);
	return (aux * 2) - 1;
}

static int rotation(int address)
{
	static const int group15[] = { 15, 11, 7, 5 };   // 15 is a guess
	static const int group14[] = { 14, 9, 3, 2 };
	static const int group13[] = { 13, 10, 6, 1 };
	static const int group12[] = { 12, 8, 4, 0 };

	// rotation depending on all the address bits
	const int enabled0 = rot_enabled(address, group15);
	int rot = enabled0 * rot_group(address, group15) * 9;
	const int enabled1 = enabled0 ^ rot_enabled(address, group14);
	rot += enabled1 * rot_group(address, group14) * 1;
	const int enabled2 = enabled0 ^ rot_enabled(address, group13);
	rot += enabled2 * rot_group(address, group13) * 2;
	const int enabled3 = enabled0 ^ rot_enabled(address, group12);
	rot += enabled3 * rot_group(address, group12) * 4;

	// block-independent rotation (just depending on the lowest 8 bits)
	int rot2 = 4 * BIT(address, 0);
	rot2 += 1 * BIT(address, 4) * (BIT(address, 0) * 2 - 1);
	rot2 += 4 * BIT(address, 3) * (BIT(address, 0) * 2 - 1);
	rot2 *= (BIT(address, 7) | (BIT(address, 0) ^ BIT(address, 1) ^ 1)) * 2 - 1;
	rot2 += 2 * ((BIT(address, 0) ^ BIT(address, 1)) & (BIT(address, 7) ^ 1));

	return (rot + rot2) & 0xf;
}

// bitswap<16>(aux, 10,9,8,7,0,15,6,5, 14,13,4,3,12,11,2,1): result bit 15-k = aux bit src[k]
static uint16_t s_swap_lo[256], s_swap_hi[256];
static int s_swap_ready;
static void swap_init(void)
{
	static const int src[16] = { 10, 9, 8, 7, 0, 15, 6, 5, 14, 13, 4, 3, 12, 11, 2, 1 };
	for (int v = 0; v < 256; v++) {
		uint16_t lo = 0, hi = 0;
		for (int k = 0; k < 16; k++) {
			const int b = src[k], out = 15 - k;
			if (b < 8 && BIT(v, b))
				lo |= (uint16_t)(1u << out);
			if (b >= 8 && BIT(v, b - 8))
				hi |= (uint16_t)(1u << out);
		}
		s_swap_lo[v] = lo;
		s_swap_hi[v] = hi;
	}
	s_swap_ready = 1;
}

void igs036_decrypt_rom(const uint8_t key[0x100], uint16_t *rom, int size, int offset)
{
	if (!s_swap_ready)
		swap_init();
	for (int i = 0; i < size / 2; i++) {
		const int word_address = i + offset;
		// key-independent manipulation (deobfuscate)
		const int shift = rotation(word_address);
		const uint16_t cw = rom[i];
		const uint16_t r = (uint16_t)(cw << shift), l = (uint16_t)(shift ? cw >> (16 - shift) : 0);
		const uint16_t rl = (uint16_t)(r | l);
		int aux = s_swap_lo[rl & 0xff] | s_swap_hi[rl >> 8];
		// key-dependent manipulation
		for (int b = 0; b < 16; ++b) {
			if (((uint32_t)word_address & triggers[b][0]) == triggers[b][1])
				aux ^= BIT(key[word_address & 0xff], b & 7) << b;
			else
				aux ^= BIT(0x1a3a, b) << b;
		}
		rom[i] = (uint16_t)aux;
	}
}

// ReBOX: the inverse of igs036_decrypt_rom (MAME keeps an encrypted copy of the ROM for its reset instead)
void igs036_encrypt_rom(const uint8_t key[0x100], uint16_t *rom, int size, int offset)
{
	static uint16_t inv_lo[256], inv_hi[256];
	static int inv_ready;
	if (!s_swap_ready)
		swap_init();
	if (!inv_ready) {
		// the swap's inverse: output bit o came from input bit src -> input bit src from output bit o
		for (int v = 0; v < 256; v++) {
			uint16_t lo = 0, hi = 0;
			for (int o = 0; o < 16; o++) {
				// find the input bit i with swap(1 << i) == 1 << o
				for (int i = 0; i < 16; i++) {
					const uint16_t m = i < 8 ? s_swap_lo[1u << i] : s_swap_hi[1u << (i - 8)];
					if (m == (uint16_t)(1u << o)) {
						if (o < 8 && ((v >> o) & 1))
							lo |= (uint16_t)(1u << i);
						if (o >= 8 && ((v >> (o - 8)) & 1))
							hi |= (uint16_t)(1u << i);
					}
				}
			}
			inv_lo[v] = lo;
			inv_hi[v] = hi;
		}
		inv_ready = 1;
	}
	for (int i = 0; i < size / 2; i++) {
		const int word_address = i + offset;
		int aux = rom[i];
		for (int b = 0; b < 16; ++b) {
			if (((uint32_t)word_address & triggers[b][0]) == triggers[b][1])
				aux ^= BIT(key[word_address & 0xff], b & 7) << b;
			else
				aux ^= BIT(0x1a3a, b) << b;
		}
		const uint16_t rl = (uint16_t)(inv_lo[aux & 0xff] | inv_hi[(aux >> 8) & 0xff]);
		const int shift = rotation(word_address);
		rom[i] = (uint16_t)(shift ? (uint16_t)(rl >> shift) | (uint16_t)(rl << (16 - shift)) : rl);
	}
}
