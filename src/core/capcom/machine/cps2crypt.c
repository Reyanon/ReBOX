/*  CPS2 opcode decryption. Ported from MAME src/mame/capcom/cps2crypt.cpp (BSD-3-Clause, copyright-holders: Paul Leaman,
    Andreas Naive, Nicola Salmoria, Charles MacDonald; the algorithm broken by Andreas Naive): two 4-round Feistel
    networks over the opcode and the low 16 bits of its address, keyed by the 64-bit key of the game's "key" ROM.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "cps2crypt.h"

#define BIT(x, n) (((x) >> (n)) & 1)

static const int fn1_groupA[8] = { 10, 4, 6, 7, 2, 13, 15, 14 };
static const int fn1_groupB[8] = {  0, 1, 3, 5, 8,  9, 11, 12 };

static const int fn2_groupA[8] = { 6, 0, 2, 13, 1,  4, 14,  7 };
static const int fn2_groupB[8] = { 3, 5, 9, 10, 8, 15, 12, 11 };

typedef struct {
	uint8_t table[64];
	int inputs[6];    // positions of the input bits, -1: none (the key only)
	int outputs[2];   // positions of the output bits
} sbox;

typedef struct { uint8_t input_lookup[256]; uint8_t output[64]; } optimised_sbox;

static const sbox fn1_r1_boxes[4] =
{
	{   // subkey bits  0- 5
		{
			0,2,2,0,1,0,1,1,3,2,0,3,0,3,1,2,1,1,1,2,1,3,2,2,2,3,3,2,1,1,1,2,
			2,2,0,0,3,1,3,1,1,1,3,0,0,1,0,0,1,2,2,1,2,3,2,2,2,3,1,3,2,0,1,3,
		},
		{ 3, 4, 5, 6, -1, -1 },
		{ 3, 6 }
	},
	{   // subkey bits  6-11
		{
			3,0,2,2,2,1,1,1,1,2,1,0,0,0,2,3,2,3,1,3,0,0,0,2,1,2,2,3,0,3,3,3,
			0,1,3,2,3,3,3,1,1,1,1,2,0,1,2,1,3,2,3,1,1,3,2,2,2,3,1,3,2,3,0,0,
		},
		{ 0, 1, 2, 4, 7, -1 },
		{ 2, 7 }
	},
	{   // subkey bits 12-17
		{
			3,0,3,1,1,0,2,2,3,1,2,0,3,3,2,3,0,1,0,1,2,3,0,2,0,2,0,1,0,0,1,0,
			2,3,1,2,1,0,2,0,2,1,0,1,0,2,1,0,3,1,2,3,1,3,1,1,1,2,0,2,2,0,0,0,
		},
		{ 0, 1, 2, 3, 6, 7 },
		{ 0, 1 }
	},
	{   // subkey bits 18-23
		{
			3,2,0,3,0,2,2,1,1,2,3,2,1,3,2,1,2,2,1,3,3,2,1,0,1,0,1,3,0,0,0,2,
			2,1,0,1,0,1,0,1,3,1,1,2,2,3,2,0,3,3,2,0,2,1,3,3,0,0,3,0,1,1,3,3,
		},
		{ 0, 1, 3, 5, 6, 7 },
		{ 4, 5 }
	},
};

static const sbox fn1_r2_boxes[4] =
{
	{   // subkey bits 24-29
		{
			3,3,2,0,3,0,3,1,0,3,0,1,0,2,1,3,1,3,0,3,3,1,3,3,3,2,3,2,2,3,1,2,
			0,2,2,1,0,1,2,0,3,3,0,1,3,2,1,2,3,0,1,3,0,1,2,2,1,2,1,2,0,1,3,0,
		},
		{ 0, 1, 2, 3, 6, -1 },
		{ 1, 6 }
	},
	{   // subkey bits 30-35
		{
			1,2,3,2,1,3,0,1,1,0,2,0,0,2,3,2,3,3,0,1,2,2,1,0,1,0,1,2,3,2,1,3,
			2,2,2,0,1,0,2,3,2,1,2,1,2,1,0,3,0,1,2,3,1,2,1,3,2,0,3,2,3,0,2,0,
		},
		{ 2, 4, 5, 6, 7, -1 },
		{ 5, 7 }
	},
	{   // subkey bits 36-41
		{
			0,1,0,2,1,1,0,1,0,2,2,2,1,3,0,0,1,1,3,1,2,2,2,3,1,0,3,3,3,2,2,2,
			1,1,3,0,3,1,3,0,1,3,3,2,1,1,0,0,1,2,2,2,1,1,1,2,2,0,0,3,2,3,1,3,
		},
		{ 1, 2, 3, 4, 5, 7 },
		{ 0, 3 }
	},
	{   // subkey bits 42-47
		{
			2,1,0,3,3,3,2,0,1,2,1,1,1,0,3,1,1,3,3,0,1,2,1,0,0,0,3,0,3,0,3,0,
			1,3,3,3,0,3,2,0,2,1,2,2,2,1,1,3,0,1,0,1,0,1,1,1,1,3,1,0,1,2,3,3,
		},
		{ 0, 1, 3, 4, 6, 7 },
		{ 2, 4 }
	},
};

static const sbox fn1_r3_boxes[4] =
{
	{   // subkey bits 48-53
		{
			0,0,0,3,3,1,1,0,2,0,2,0,0,0,3,2,0,1,2,3,2,2,1,0,3,0,0,0,0,0,2,3,
			3,0,0,1,1,2,3,3,0,1,3,2,0,1,3,3,2,0,0,1,0,2,0,0,0,3,1,3,3,3,3,3,
		},
		{ 0, 1, 5, 6, 7, -1 },
		{ 0, 5 }
	},
	{   // subkey bits 54-59
		{
			2,3,2,3,0,2,3,0,2,2,3,0,3,2,0,2,1,0,2,3,1,1,1,0,0,1,0,2,1,2,2,1,
			3,0,2,1,2,3,3,0,3,2,3,1,0,2,1,0,1,2,2,3,0,2,1,3,1,3,0,2,1,1,1,3,
		},
		{ 2, 3, 4, 6, 7, -1 },
		{ 6, 7 }
	},
	{   // subkey bits 60-65
		{
			3,0,2,1,1,3,1,2,2,1,2,2,2,0,0,1,2,3,1,0,2,0,0,2,3,1,2,0,0,0,3,0,
			2,1,1,2,0,0,1,2,3,1,1,2,0,1,3,0,3,1,1,0,0,2,3,0,0,0,0,3,2,0,0,0,
		},
		{ 0, 2, 3, 4, 5, 6 },
		{ 1, 4 }
	},
	{   // subkey bits 66-71
		{
			0,1,0,0,2,1,3,2,3,3,2,1,0,1,1,1,1,1,0,3,3,1,1,0,0,2,2,1,0,3,3,2,
			1,3,3,0,3,0,2,1,1,2,3,2,2,2,1,0,0,3,3,3,2,2,3,1,0,2,3,0,3,1,1,0,
		},
		{ 0, 1, 2, 3, 5, 7 },
		{ 2, 3 }
	},
};

static const sbox fn1_r4_boxes[4] =
{
	{   // subkey bits 72-77
		{
			1,1,1,1,1,0,1,3,3,2,3,0,1,2,0,2,3,3,0,1,2,1,2,3,0,3,2,3,2,0,1,2,
			0,1,0,3,2,1,3,2,3,1,2,3,2,0,1,2,2,0,0,0,2,1,3,0,3,1,3,0,1,3,3,0,
		},
		{ 1, 2, 3, 4, 5, 7 },
		{ 0, 4 }
	},
	{   // subkey bits 78-83
		{
			3,0,0,0,0,1,0,2,3,3,1,3,0,3,1,2,2,2,3,1,0,0,2,0,1,0,2,2,3,3,0,0,
			1,1,3,0,2,3,0,3,0,3,0,2,0,2,0,1,0,3,0,1,3,1,1,0,0,1,3,3,2,2,1,0,
		},
		{ 0, 1, 2, 3, 5, 6 },
		{ 1, 3 }
	},
	{   // subkey bits 84-89
		{
			0,1,1,2,0,1,3,1,2,0,3,2,0,0,3,0,3,0,1,2,2,3,3,2,3,2,0,1,0,0,1,0,
			3,0,2,3,0,2,2,2,1,1,0,2,2,0,0,1,2,1,1,1,2,3,0,3,1,2,3,3,1,1,3,0,
		},
		{ 0, 2, 4, 5, 6, 7 },
		{ 2, 6 }
	},
	{   // subkey bits 90-95
		{
			0,1,2,2,0,1,0,3,2,2,1,1,3,2,0,2,0,1,3,3,0,2,2,3,3,2,0,0,2,1,3,3,
			1,1,1,3,1,2,1,1,0,3,3,2,3,2,3,0,3,1,0,0,3,0,0,0,2,2,2,1,2,3,0,0,
		},
		{ 0, 1, 3, 4, 6, 7 },
		{ 5, 7 }
	},
};

/******************************************************************************/

static const sbox fn2_r1_boxes[4] =
{
	{   // subkey bits  0- 5
		{
			2,0,2,0,3,0,0,3,1,1,0,1,3,2,0,1,2,0,1,2,0,2,0,2,2,2,3,0,2,1,3,0,
			0,1,0,1,2,2,3,3,0,3,0,2,3,0,1,2,1,1,0,2,0,3,1,1,2,2,1,3,1,1,3,1,
		},
		{ 0, 3, 4, 5, 7, -1 },
		{ 6, 7 }
	},
	{   // subkey bits  6-11
		{
			1,1,0,3,0,2,0,1,3,0,2,0,1,1,0,0,1,3,2,2,0,2,2,2,2,0,1,3,3,3,1,1,
			1,3,1,3,2,2,2,2,2,2,0,1,0,1,1,2,3,1,1,2,0,3,3,3,2,2,3,1,1,1,3,0,
		},
		{ 1, 2, 3, 4, 6, -1 },
		{ 3, 5 }
	},
	{   // subkey bits 12-17
		{
			1,0,2,2,3,3,3,3,1,2,2,1,0,1,2,1,1,2,3,1,2,0,0,1,2,3,1,2,0,0,0,2,
			2,0,1,1,0,0,2,0,0,0,2,3,2,3,0,1,3,0,0,0,2,3,2,0,1,3,2,1,3,1,1,3,
		},
		{ 1, 2, 4, 5, 6, 7 },
		{ 1, 4 }
	},
	{   // subkey bits 18-23
		{
			1,3,3,0,3,2,3,1,3,2,1,1,3,3,2,1,2,3,0,3,1,0,0,2,3,0,0,0,3,3,0,1,
			2,3,0,0,0,1,2,1,3,0,0,1,0,2,2,2,3,3,1,2,1,3,0,0,0,3,0,1,3,2,2,0,
		},
		{ 0, 2, 3, 5, 6, 7 },
		{ 0, 2 }
	},
};

static const sbox fn2_r2_boxes[4] =
{
	{   // subkey bits 24-29
		{
			3,1,3,0,3,0,3,1,3,0,0,1,1,3,0,3,1,1,0,1,2,3,2,3,3,1,2,2,2,0,2,3,
			2,2,2,1,1,3,3,0,3,1,2,1,1,1,0,2,0,3,3,0,0,2,0,0,1,1,2,1,2,1,1,0,
		},
		{ 0, 2, 4, 6, -1, -1 },
		{ 4, 6 }
	},
	{   // subkey bits 30-35
		{
			0,3,0,3,3,2,1,2,3,1,1,1,2,0,2,3,0,3,1,2,2,1,3,3,3,2,1,2,2,0,1,0,
			2,3,0,1,2,0,1,1,2,0,2,1,2,0,2,3,3,1,0,2,3,3,0,3,1,1,3,0,0,1,2,0,
		},
		{ 1, 3, 4, 5, 6, 7 },
		{ 0, 3 }
	},
	{   // subkey bits 36-41
		{
			0,0,2,1,3,2,1,0,1,2,2,2,1,1,0,3,1,2,2,3,2,1,1,0,3,0,0,1,1,2,3,1,
			3,3,2,2,1,0,1,1,1,2,0,1,2,3,0,3,3,0,3,2,2,0,2,2,1,2,3,2,1,0,2,1,
		},
		{ 0, 1, 3, 4, 5, 7 },
		{ 1, 7 }
	},
	{   // subkey bits 42-47
		{
			0,2,1,2,0,2,2,0,1,3,2,0,3,2,3,0,3,3,2,3,1,2,3,1,2,2,0,0,2,2,1,2,
			2,3,3,3,1,1,0,0,0,3,2,0,3,2,3,1,1,1,1,0,1,0,1,3,0,0,1,2,2,3,2,0,
		},
		{ 1, 2, 3, 5, 6, 7 },
		{ 2, 5 }
	},
};

static const sbox fn2_r3_boxes[4] =
{
	{   // subkey bits 48-53
		{
			2,1,2,1,2,3,1,3,2,2,1,3,3,0,0,1,0,2,0,3,3,1,0,0,1,1,0,2,3,2,1,2,
			1,1,2,1,1,3,2,2,0,2,2,3,3,3,2,0,0,0,0,0,3,3,3,0,1,2,1,0,2,3,3,1,
		},
		{ 2, 3, 4, 6, -1, -1 },
		{ 3, 5 }
	},
	{   // subkey bits 54-59
		{
			3,2,3,3,1,0,3,0,2,0,1,1,1,0,3,0,3,1,3,1,0,1,2,3,2,2,3,2,0,1,1,2,
			3,0,0,2,1,0,0,2,2,0,1,0,0,2,0,0,1,3,1,3,2,0,3,3,1,0,2,2,2,3,0,0,
		},
		{ 0, 1, 3, 5, 7, -1 },
		{ 0, 2 }
	},
	{   // subkey bits 60-65
		{
			2,2,1,0,2,3,3,0,0,0,1,3,1,2,3,2,2,3,1,3,0,3,0,3,3,2,2,1,0,0,0,2,
			1,2,2,2,0,0,1,2,0,1,3,0,2,3,2,1,3,2,2,2,3,1,3,0,2,0,2,1,0,3,3,1,
		},
		{ 0, 1, 2, 3, 5, 7 },
		{ 1, 6 }
	},
	{   // subkey bits 66-71
		{
			1,2,3,2,0,2,1,3,3,1,0,1,1,2,2,0,0,1,1,1,2,1,1,2,0,1,3,3,1,1,1,2,
			3,3,1,0,2,1,1,1,2,1,0,0,2,2,3,2,3,2,2,0,2,2,3,3,0,2,3,0,2,2,1,1,
		},
		{ 0, 2, 4, 5, 6, 7 },
		{ 4, 7 }
	},
};

static const sbox fn2_r4_boxes[4] =
{
	{   // subkey bits 72-77
		{
			2,0,1,1,2,1,3,3,1,1,1,2,0,1,0,2,0,1,2,0,2,3,0,2,3,3,2,2,3,2,0,1,
			3,0,2,0,2,3,1,3,2,0,0,1,1,2,3,1,1,1,0,1,2,0,3,3,1,1,1,3,3,1,1,0,
		},
		{ 0, 1, 3, 6, 7, -1 },
		{ 0, 3 }
	},
	{   // subkey bits 78-83
		{
			1,2,2,1,0,3,3,1,0,2,2,2,1,0,1,0,1,1,0,1,0,2,1,0,2,1,0,2,3,2,3,3,
			2,2,1,2,2,3,1,3,3,3,0,1,0,1,3,0,0,0,1,2,0,3,3,2,3,2,1,3,2,1,0,2,
		},
		{ 0, 1, 2, 4, 5, 6 },
		{ 4, 7 }
	},
	{   // subkey bits 84-89
		{
			2,3,2,1,3,2,3,0,0,2,1,1,0,0,3,2,3,1,0,1,2,2,2,1,3,2,2,1,0,2,1,2,
			0,3,1,0,0,3,1,1,3,3,2,0,1,0,1,3,0,0,1,2,1,2,3,2,1,0,0,3,2,1,1,3,
		},
		{ 0, 2, 3, 4, 5, 7 },
		{ 1, 2 }
	},
	{   // subkey bits 90-95
		{
			2,0,0,3,2,2,2,1,3,3,1,1,2,0,0,3,1,0,3,2,1,0,2,0,3,2,2,3,2,0,3,0,
			1,3,0,2,2,1,3,3,0,1,0,3,1,1,3,2,0,3,0,2,3,2,1,3,2,3,0,0,1,3,2,1,
		},
		{ 2, 3, 4, 5, 6, 7 },
		{ 5, 6 }
	},
};

static int extract_inputs(const sbox *s, uint32_t val)
{
	int res = 0;
	for (int i = 0; i < 6; ++i)
		if (s->inputs[i] >= 0)
			res |= (int)BIT(val, s->inputs[i]) << i;
	return res;
}

static void optimise(optimised_sbox *o, const sbox *in)
{
	for (int i = 0; i < 256; ++i)
		o->input_lookup[i] = (uint8_t)extract_inputs(in, (uint32_t)i);
	for (int i = 0; i < 64; ++i) {
		const int v = in->table[i];
		o->output[i] = 0;
		if (v & 1)
			o->output[i] |= (uint8_t)(1 << in->outputs[0]);
		if (v & 2)
			o->output[i] |= (uint8_t)(1 << in->outputs[1]);
	}
}

static inline uint8_t sfn(const optimised_sbox *o, uint8_t in, uint32_t key) { return o->output[o->input_lookup[in] ^ (key & 0x3f)]; }

static uint8_t fn(uint8_t in, const optimised_sbox *sboxes, uint32_t key)
{
	return (uint8_t)(sfn(&sboxes[0], in, key >> 0) | sfn(&sboxes[1], in, key >> 6) | sfn(&sboxes[2], in, key >> 12) |
	                 sfn(&sboxes[3], in, key >> 18));
}

// the 64-bit master key (2x32 bits) to the 96-bit key of the first FN (4x24 bits)
static void expand_1st_key(uint32_t *dstkey, const uint32_t *srckey)
{
	static const int bits[96] = {
		33, 58, 49, 36,  0, 31,
		22, 30,  3, 16,  5, 53,
		10, 41, 23, 19, 27, 39,
		43,  6, 34, 12, 61, 21,
		48, 13, 32, 35,  6, 42,
		43, 14, 21, 41, 52, 25,
		18, 47, 46, 37, 57, 53,
		20,  8, 55, 54, 59, 60,
		27, 33, 35, 18,  8, 15,
		63,  1, 50, 44, 16, 46,
		 5,  4, 45, 51, 38, 25,
		13, 11, 62, 29, 48,  2,
		59, 61, 62, 56, 51, 57,
		54,  9, 24, 63, 22,  7,
		26, 42, 45, 40, 23, 14,
		 2, 31, 52, 28, 44, 17,
	};
	dstkey[0] = dstkey[1] = dstkey[2] = dstkey[3] = 0;
	for (int i = 0; i < 96; ++i)
		dstkey[i / 24] |= BIT(srckey[bits[i] / 32], bits[i] % 32) << (i % 24);
}

// the master key XOR the subkey to the 96-bit key of the second FN
static void expand_2nd_key(uint32_t *dstkey, const uint32_t *srckey)
{
	static const int bits[96] = {
		34,  9, 32, 24, 44, 54,
		38, 61, 47, 13, 28,  7,
		29, 58, 18,  1, 20, 60,
		15,  6, 11, 43, 39, 19,
		63, 23, 16, 62, 54, 40,
		31,  3, 56, 61, 17, 25,
		47, 38, 55, 57,  5,  4,
		15, 42, 22,  7,  2, 19,
		46, 37, 29, 39, 12, 30,
		49, 57, 31, 41, 26, 27,
		24, 36, 11, 63, 33, 16,
		56, 62, 48, 60, 59, 32,
		12, 30, 53, 48, 10,  0,
		50, 35,  3, 59, 14, 49,
		51, 45, 44,  2, 21, 33,
		55, 52, 23, 28,  8, 26,
	};
	dstkey[0] = dstkey[1] = dstkey[2] = dstkey[3] = 0;
	for (int i = 0; i < 96; ++i)
		dstkey[i / 24] |= BIT(srckey[bits[i] / 32], bits[i] % 32) << (i % 24);
}

// the 16-bit seed of the first FN to the 64-bit subkey (2x32 bits)
static void expand_subkey(uint32_t *subkey, uint16_t seed)
{
	static const int bits[64] = {
		 5, 10, 14,  9,  4,  0, 15,  6,  1,  8,  3,  2, 12,  7, 13, 11,
		 5, 12,  7,  2, 13, 11,  9, 14,  4,  1,  6, 10,  8,  0, 15,  3,
		 4, 10,  2,  0,  6,  9, 12,  1, 11,  7, 15,  8, 13,  5, 14,  3,
		14, 11, 12,  7,  4,  5,  2, 10,  1, 15,  0,  9,  8,  6, 13,  3,
	};
	subkey[0] = subkey[1] = 0;
	for (int i = 0; i < 64; ++i)
		subkey[i / 32] |= (uint32_t)BIT(seed, bits[i]) << (i % 32);
}

static uint16_t feistel(uint16_t val, const int *bitsA, const int *bitsB, const optimised_sbox *boxes1,
                        const optimised_sbox *boxes2, const optimised_sbox *boxes3, const optimised_sbox *boxes4,
                        uint32_t key1, uint32_t key2, uint32_t key3, uint32_t key4)
{
	uint8_t l = 0, r = 0;
	for (int i = 0; i < 8; i++) {
		l |= (uint8_t)(BIT(val, bitsB[i]) << i);
		r |= (uint8_t)(BIT(val, bitsA[i]) << i);
	}
	l ^= fn(r, boxes1, key1);
	r ^= fn(l, boxes2, key2);
	l ^= fn(r, boxes3, key3);
	r ^= fn(l, boxes4, key4);
	uint16_t res = 0;
	for (int i = 0; i < 8; i++)
		res |= (uint16_t)(BIT(l, i) << bitsA[i] | BIT(r, i) << bitsB[i]);
	return res;
}

static void optimise_sboxes(optimised_sbox *out, const sbox *in)
{
	for (int box = 0; box < 4; ++box)
		optimise(&out[box], &in[box]);
}

void cps2_decrypt(const uint16_t *rom, uint16_t *dec, int length, const uint32_t *master_key, uint32_t lower_limit,
                  uint32_t upper_limit)
{
	optimised_sbox sboxes1[4 * 4], sboxes2[4 * 4];
	optimise_sboxes(&sboxes1[0 * 4], fn1_r1_boxes);
	optimise_sboxes(&sboxes1[1 * 4], fn1_r2_boxes);
	optimise_sboxes(&sboxes1[2 * 4], fn1_r3_boxes);
	optimise_sboxes(&sboxes1[3 * 4], fn1_r4_boxes);
	optimise_sboxes(&sboxes2[0 * 4], fn2_r1_boxes);
	optimise_sboxes(&sboxes2[1 * 4], fn2_r2_boxes);
	optimise_sboxes(&sboxes2[2 * 4], fn2_r3_boxes);
	optimise_sboxes(&sboxes2[3 * 4], fn2_r4_boxes);

	uint32_t key1[4];
	expand_1st_key(key1, master_key);
	// extra bits for the s-boxes with less than 6 inputs
	key1[0] ^= BIT(key1[0], 1) << 4;
	key1[0] ^= BIT(key1[0], 2) << 5;
	key1[0] ^= BIT(key1[0], 8) << 11;
	key1[1] ^= BIT(key1[1], 0) << 5;
	key1[1] ^= BIT(key1[1], 8) << 11;
	key1[2] ^= BIT(key1[2], 1) << 5;
	key1[2] ^= BIT(key1[2], 8) << 11;

	for (int i = 0; i < 0x10000; ++i) {
		const uint16_t seed = feistel((uint16_t)i, fn1_groupA, fn1_groupB, &sboxes1[0 * 4], &sboxes1[1 * 4],
		                              &sboxes1[2 * 4], &sboxes1[3 * 4], key1[0], key1[1], key1[2], key1[3]);
		uint32_t subkey[2];
		expand_subkey(subkey, seed);
		subkey[0] ^= master_key[0];
		subkey[1] ^= master_key[1];
		uint32_t key2[4];
		expand_2nd_key(key2, subkey);
		key2[0] ^= BIT(key2[0], 0) << 5;
		key2[0] ^= BIT(key2[0], 6) << 11;
		key2[1] ^= BIT(key2[1], 0) << 5;
		key2[1] ^= BIT(key2[1], 1) << 4;
		key2[2] ^= BIT(key2[2], 2) << 5;
		key2[2] ^= BIT(key2[2], 3) << 4;
		key2[2] ^= BIT(key2[2], 7) << 11;
		key2[3] ^= BIT(key2[3], 1) << 5;
		for (int a = i; a < length / 2; a += 0x10000) {
			if ((uint32_t)a >= lower_limit && (uint32_t)a <= upper_limit)
				dec[a] = feistel(rom[a], fn2_groupA, fn2_groupB, &sboxes2[0 * 4], &sboxes2[1 * 4], &sboxes2[2 * 4],
				                 &sboxes2[3 * 4], key2[0], key2[1], key2[2], key2[3]);
			else
				dec[a] = rom[a];
		}
	}
}
