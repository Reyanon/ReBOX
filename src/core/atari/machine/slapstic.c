/*  Atari Slapstic. Ported from MAME src/mame/atari/slapstic.cpp (BSD-3-Clause, copyright-holders: Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "slapstic.h"

#include <string.h>

enum { S_IDLE, S_ACTIVE, S_ALT_VALID, S_ALT_SELECT, S_ALT_COMMIT, S_BIT_LOAD, S_BIT_SET_ODD, S_BIT_SET_EVEN, S_ADD_LOAD,
       S_ADD_SET };

typedef struct { uint16_t mask, value; } mv;
typedef struct {
	uint8_t bankstart;
	uint16_t bank[4];
	mv alt1, alt2, alt3, alt4;
	int altshift;
	mv bit1, bit2, bit3c0, bit3s0, bit3c1, bit3s1, bit4;
	mv add1, add2, addplus1, addplus2, add3;
} slapstic_data;

#define U { 0xffff, 0xffff }
#define NO_BITWISE U, U, U, U, U, U, U
#define NO_ADDITIVE U, U, U, U, U

static const slapstic_data s_data[18] = {
	// 101: Empire Strikes Back / Tetris
	{ 3, { 0x0080, 0x0090, 0x00a0, 0x00b0 }, { 0x1f00, 0x1e00 }, { 0x1fff, 0x1fff }, { 0x1ffc, 0x1b5c }, { 0x1fcf, 0x0080 }, 0,
	  { 0x1ff0, 0x1540 }, { 0x1fcf, 0x0080 }, { 0x1ff3, 0x1540 }, { 0x1ff3, 0x1541 }, { 0x1ff3, 0x1542 }, { 0x1ff3, 0x1543 },
	  { 0x1ff8, 0x1550 }, NO_ADDITIVE },
	// 102: never seen
	{ 0, { 0, 0, 0, 0 }, U, U, U, U, 0, NO_BITWISE, NO_ADDITIVE },
	// 103: Marble Madness
	{ 3, { 0x0040, 0x0050, 0x0060, 0x0070 }, { 0x007f, 0x002d }, { 0x3fff, 0x3d14 }, { 0x3ffc, 0x3d24 }, { 0x3fcf, 0x0040 }, 0,
	  { 0x3ff0, 0x34c0 }, { 0x3fcf, 0x0040 }, { 0x3ff3, 0x34c0 }, { 0x3ff3, 0x34c1 }, { 0x3ff3, 0x34c2 }, { 0x3ff3, 0x34c3 },
	  { 0x3ff8, 0x34d0 }, NO_ADDITIVE },
	// 104: Gauntlet
	{ 3, { 0x0020, 0x0028, 0x0030, 0x0038 }, { 0x007f, 0x0069 }, { 0x3fff, 0x3735 }, { 0x3ffc, 0x3764 }, { 0x3fe7, 0x0020 }, 0,
	  { 0x3ff0, 0x3d90 }, { 0x3fe7, 0x0020 }, { 0x3ff3, 0x3d90 }, { 0x3ff3, 0x3d91 }, { 0x3ff3, 0x3d92 }, { 0x3ff3, 0x3d93 },
	  { 0x3ff8, 0x3da0 }, NO_ADDITIVE },
	// 105: Indiana Jones / Paperboy
	{ 3, { 0x0010, 0x0014, 0x0018, 0x001c }, { 0x007f, 0x003d }, { 0x3fff, 0x0092 }, { 0x3ffc, 0x00a4 }, { 0x3ff3, 0x0010 }, 0,
	  { 0x3ff0, 0x35b0 }, { 0x3ff3, 0x0010 }, { 0x3ff3, 0x35b0 }, { 0x3ff3, 0x35b1 }, { 0x3ff3, 0x35b2 }, { 0x3ff3, 0x35b3 },
	  { 0x3ff8, 0x35c0 }, NO_ADDITIVE },
	// 106: Gauntlet II
	{ 3, { 0x0008, 0x000a, 0x000c, 0x000e }, { 0x007f, 0x002b }, { 0x3fff, 0x0052 }, { 0x3ffc, 0x0064 }, { 0x3ff9, 0x0008 }, 0,
	  { 0x3ff0, 0x3da0 }, { 0x3ff9, 0x0008 }, { 0x3ff3, 0x3da0 }, { 0x3ff3, 0x3da1 }, { 0x3ff3, 0x3da2 }, { 0x3ff3, 0x3da3 },
	  { 0x3ff8, 0x3db0 }, NO_ADDITIVE },
	// 107: Peter Packrat / Xybots / 2p Gauntlet / 720
	{ 3, { 0x0018, 0x001a, 0x001c, 0x001e }, { 0x007f, 0x006b }, { 0x3fff, 0x3d52 }, { 0x3ffc, 0x3d64 }, { 0x3ff9, 0x0018 }, 0,
	  { 0x3ff0, 0x00a0 }, { 0x3ff9, 0x0018 }, { 0x3ff3, 0x00a0 }, { 0x3ff3, 0x00a1 }, { 0x3ff3, 0x00a2 }, { 0x3ff3, 0x00a3 },
	  { 0x3ff8, 0x00b0 }, NO_ADDITIVE },
	// 108: Road Runner / Super Sprint
	{ 3, { 0x0028, 0x002a, 0x002c, 0x002e }, { 0x007f, 0x001f }, { 0x3fff, 0x3772 }, { 0x3ffc, 0x3764 }, { 0x3ff9, 0x0028 }, 0,
	  { 0x3ff0, 0x0060 }, { 0x3ff9, 0x0028 }, { 0x3ff3, 0x0060 }, { 0x3ff3, 0x0061 }, { 0x3ff3, 0x0062 }, { 0x3ff3, 0x0063 },
	  { 0x3ff8, 0x0070 }, NO_ADDITIVE },
	// 109: Championship Sprint / Road Blasters
	{ 3, { 0x0008, 0x000a, 0x000c, 0x000e }, { 0x007f, 0x002b }, { 0x3fff, 0x0052 }, { 0x3ffc, 0x0064 }, { 0x3ff9, 0x0008 }, 0,
	  { 0x3ff0, 0x3da0 }, { 0x3ff9, 0x0008 }, { 0x3ff3, 0x3da0 }, { 0x3ff3, 0x3da1 }, { 0x3ff3, 0x3da2 }, { 0x3ff3, 0x3da3 },
	  { 0x3ff8, 0x3db0 }, NO_ADDITIVE },
	// 110: Road Blasters / APB
	{ 3, { 0x0040, 0x0050, 0x0060, 0x0070 }, { 0x007f, 0x002d }, { 0x3fff, 0x3d14 }, { 0x3ffc, 0x3d24 }, { 0x3fcf, 0x0040 }, 0,
	  { 0x3ff0, 0x34c0 }, { 0x3fcf, 0x0040 }, { 0x3ff3, 0x34c0 }, { 0x3ff3, 0x34c1 }, { 0x3ff3, 0x34c2 }, { 0x3ff3, 0x34c3 },
	  { 0x3ff8, 0x34d0 }, NO_ADDITIVE },
	// 111: Pit Fighter
	{ 0, { 0x0042, 0x0052, 0x0062, 0x0072 }, { 0x007f, 0x000a }, { 0x3fff, 0x28a4 }, { 0x0784, 0x0080 }, { 0x3fcf, 0x0042 }, 0,
	  NO_BITWISE, { 0x3fff, 0x00a1 }, { 0x3fff, 0x00a2 }, { 0x3c4f, 0x284d }, { 0x3a5f, 0x285d }, { 0x3ff8, 0x2800 } },
	// 112: Pit Fighter
	{ 0, { 0x002c, 0x003c, 0x006c, 0x007c }, { 0x007f, 0x0014 }, { 0x3fff, 0x29a0 }, { 0x0073, 0x0010 }, { 0x3faf, 0x002c }, 2,
	  NO_BITWISE, { 0x3fff, 0x2dce }, { 0x3fff, 0x2dcf }, { 0x3def, 0x15e2 }, { 0x3fbf, 0x15a2 }, { 0x3ffc, 0x1450 } },
	// 113: Pit Fighter
	{ 0, { 0x0008, 0x0018, 0x0028, 0x0038 }, { 0x007f, 0x0059 }, { 0x3fff, 0x11a5 }, { 0x0860, 0x0800 }, { 0x3fcf, 0x0008 }, 3,
	  NO_BITWISE, { 0x3fff, 0x049b }, { 0x3fff, 0x049c }, { 0x3fcf, 0x3ec7 }, { 0x3edf, 0x3ed7 }, { 0x3fff, 0x3fb2 } },
	// 114: Pit Fighter
	{ 0, { 0x0040, 0x0048, 0x0050, 0x0058 }, { 0x007f, 0x0016 }, { 0x3fff, 0x24de }, { 0x3871, 0x0000 }, { 0x3fe7, 0x0040 }, 1,
	  NO_BITWISE, { 0x3fff, 0x0ab7 }, { 0x3fff, 0x0ab8 }, { 0x3f63, 0x0d40 }, { 0x3fd9, 0x0dc8 }, { 0x3fff, 0x0ab0 } },
	// 115: Race Drivin' DSK board
	{ 0, { 0x0020, 0x0022, 0x0024, 0x0026 }, { 0x007f, 0x0054 }, { 0x3fff, 0x3e01 }, { 0x3879, 0x0029 }, { 0x3ff9, 0x0020 }, 1,
	  NO_BITWISE, { 0x3fff, 0x2591 }, { 0x3fff, 0x2592 }, { 0x3fe6, 0x3402 }, { 0x3fb4, 0x3410 }, { 0x3fff, 0x34a2 } },
	// 116: Hydra
	{ 0, { 0x0044, 0x004c, 0x0054, 0x005c }, { 0x007f, 0x0069 }, { 0x3fff, 0x2bab }, { 0x387c, 0x0808 }, { 0x3fe7, 0x0044 }, 0,
	  NO_BITWISE, { 0x3fff, 0x3f7c }, { 0x3fff, 0x3f7d }, { 0x3db2, 0x3c12 }, { 0x3fe3, 0x3e43 }, { 0x3fff, 0x2ba8 } },
	// 117: Race Drivin' main board
	{ 0, { 0x0008, 0x001a, 0x002c, 0x003e }, { 0x007f, 0x007d }, { 0x3fff, 0x3580 }, { 0x0079, 0x0020 }, { 0x3fc9, 0x0008 }, 1,
	  NO_BITWISE, { 0x3fff, 0x0676 }, { 0x3fff, 0x0677 }, { 0x3e62, 0x1a42 }, { 0x3e35, 0x1a11 }, { 0x3fff, 0x1a42 } },
	// 118: Rampart / Vindicators II
	{ 0, { 0x0014, 0x0034, 0x0054, 0x0074 }, { 0x007f, 0x0002 }, { 0x3fff, 0x1950 }, { 0x0067, 0x0020 }, { 0x3f9f, 0x0014 }, 3,
	  NO_BITWISE, { 0x3fff, 0x1958 }, { 0x3fff, 0x1959 }, { 0x3f73, 0x3052 }, { 0x3f67, 0x3042 }, { 0x3ff8, 0x30e0 } },
};

typedef struct { uint32_t range_mask, range_value, shift, input_mask; } checker;

static slap_test t_in(const checker *c, mv x)
{
	slap_test t = { c->range_mask | ((uint32_t)x.mask << c->shift), c->range_value | ((uint32_t)x.value << c->shift) };
	return t;
}
static slap_test t_any(const checker *c, mv x)
{
	slap_test t = { (uint32_t)x.mask << c->shift, (uint32_t)x.value << c->shift };
	return t;
}
static inline int hit(slap_test t, uint32_t a) { return (a & t.m) == t.v; }

int slapstic_init(slapstic_t *s, int chipnum, uint32_t start, uint32_t end, uint32_t mirror, int data_width)
{
	memset(s, 0, sizeof(*s));
	if (chipnum < 101 || chipnum > 118 || chipnum == 102)
		return -1;
	const slapstic_data *d = &s_data[chipnum - 101];
	checker c;
	c.range_mask = ~((end - start) | mirror);
	c.range_value = start;
	c.shift = data_width == 16 ? 1 : 0;
	c.input_mask = ((1u << (chipnum == 101 ? 13 : 14)) - 1) << c.shift;
	s->chipnum = chipnum;
	s->bankstart = d->bankstart;
	s->reset.m = c.range_mask | c.input_mask;
	s->reset.v = c.range_value;
	s->inside.m = c.range_mask;
	s->inside.v = c.range_value;
	for (int i = 0; i < 4; i++) {
		s->bank[i].m = c.range_mask | c.input_mask;
		s->bank[i].v = c.range_value | ((uint32_t)d->bank[i] << c.shift);
	}
	const int fam = chipnum <= 102 ? 0 : chipnum <= 110 ? 1 : 2;
	s->alt1 = fam == 0 ? t_in(&c, d->alt1) : t_any(&c, d->alt1);
	s->alt2 = fam == 0 ? t_any(&c, d->alt2) : t_in(&c, d->alt2);
	s->alt3 = fam <= 1 ? t_in(&c, d->alt3) : t_any(&c, d->alt3);
	s->alt4 = t_in(&c, d->alt4);
	s->altshift = (int)c.shift + d->altshift;
	s->bit1 = t_in(&c, d->bit1);
	s->bit2 = t_in(&c, d->bit2);
	s->bit4 = t_in(&c, d->bit4);
	// bit_set: [1] odd (clear0 c0, set0 s0, clear1 c1, set1 s1), [0] even (s1, c1, s0, c0)
	s->bset[1][0] = t_in(&c, d->bit3c0);
	s->bset[1][1] = t_in(&c, d->bit3s0);
	s->bset[1][2] = t_in(&c, d->bit3c1);
	s->bset[1][3] = t_in(&c, d->bit3s1);
	s->bset[0][0] = t_in(&c, d->bit3s1);
	s->bset[0][1] = t_in(&c, d->bit3c1);
	s->bset[0][2] = t_in(&c, d->bit3s0);
	s->bset[0][3] = t_in(&c, d->bit3c0);
	s->add1 = t_in(&c, d->add1);
	s->add2 = t_in(&c, d->add2);
	s->addp1 = t_in(&c, d->addplus1);
	s->addp2 = t_in(&c, d->addplus2);
	s->add3 = t_in(&c, d->add3);
	slapstic_reset(s);
	return 0;
}

void slapstic_reset(slapstic_t *s)
{
	s->state = S_IDLE;
	s->current_bank = s->bankstart;
	s->loaded_bank = 0;
	s->pf_inst = 0xffffffffu;
	s->pf_skip = 0xffffffffu;
	s->pf_done = 0;
}

void slapstic_tap(slapstic_t *s, uint32_t a, uint32_t pc, int pc_inside, uint32_t inst)
{
	if (inst != s->pf_inst) {
		s->pf_inst = inst;
		s->pf_done = 0;
	}
	const int fetch = a == pc - 2;   // (Musashi has moved the PC past a word it fetches)
	if (fetch) {
		const int skip = a == s->pf_skip;
		s->pf_skip = 0xffffffffu;
		if (skip)
			return;
	} else if (!s->pf_done) {   // the instruction's first operand access: the fetch at the PC before it
		s->pf_done = 1;
		if (pc_inside) {
			slapstic_access(s, pc);
			s->pf_skip = pc;
		} else if (s->state == S_ACTIVE && s->chipnum > 102 && hit(s->alt1, pc))
			s->state = S_ALT_VALID;
	}
	slapstic_access(s, a);
}

int slapstic_access(slapstic_t *s, uint32_t a)
{
	const int fam = s->chipnum <= 102 ? 0 : s->chipnum <= 110 ? 1 : 2;
	switch (s->state) {
	case S_IDLE:
		if (hit(s->reset, a))
			s->state = S_ACTIVE;
		break;
	case S_ACTIVE:
		for (int i = 0; i < 4; i++)
			if (hit(s->bank[i], a)) {   // direct switch
				s->current_bank = (uint8_t)i;
				s->state = S_IDLE;
				return s->current_bank;
			}
		if (hit(s->alt1, a))
			s->state = S_ALT_VALID;
		else if (fam <= 1 && hit(s->bit1, a))
			s->state = S_BIT_LOAD;
		else if (fam == 2 && hit(s->add1, a))
			s->state = S_ADD_LOAD;
		break;
	case S_ALT_VALID:
		if (hit(s->reset, a))
			s->state = S_ACTIVE;
		else if (fam == 0 ? (!hit(s->inside, a) && hit(s->alt2, a)) : hit(s->alt2, a))
			s->state = S_ALT_SELECT;
		else if (fam == 2 && hit(s->add1, a))
			s->state = S_ADD_LOAD;
		else
			s->state = S_ACTIVE;
		break;
	case S_ALT_SELECT:
		if (hit(s->reset, a))
			s->state = S_ACTIVE;
		else if (hit(s->alt3, a)) {
			s->loaded_bank = (uint8_t)((a >> s->altshift) & 3);
			s->state = S_ALT_COMMIT;
		} else
			s->state = S_ACTIVE;
		break;
	case S_ALT_COMMIT:
		if (hit(s->reset, a))
			s->state = S_ACTIVE;
		else if (hit(s->alt4, a)) {
			s->current_bank = s->loaded_bank;
			s->state = S_IDLE;
		}
		break;
	case S_BIT_LOAD:
		if (hit(s->reset, a))
			s->state = S_ACTIVE;
		else if (hit(s->bit2, a)) {
			s->loaded_bank = s->current_bank;
			s->state = S_BIT_SET_ODD;
		}
		break;
	case S_BIT_SET_ODD:
	case S_BIT_SET_EVEN: {
		const int odd = s->state == S_BIT_SET_ODD;
		const uint8_t next = odd ? S_BIT_SET_EVEN : S_BIT_SET_ODD;
		if (hit(s->reset, a))
			s->state = S_ACTIVE;
		else if (hit(s->bset[odd][0], a)) {
			s->loaded_bank &= (uint8_t)~1;
			s->state = next;
		} else if (hit(s->bset[odd][1], a)) {
			s->loaded_bank |= 1;
			s->state = next;
		} else if (hit(s->bset[odd][2], a)) {
			s->loaded_bank &= (uint8_t)~2;
			s->state = next;
		} else if (hit(s->bset[odd][3], a)) {
			s->loaded_bank |= 2;
			s->state = next;
		} else if (hit(s->bit4, a)) {
			s->current_bank = s->loaded_bank;
			s->state = S_IDLE;
		}
		break;
	}
	case S_ADD_LOAD:
		if (hit(s->reset, a))
			s->state = S_ACTIVE;
		else if (hit(s->add2, a)) {
			s->loaded_bank = s->current_bank;
			s->state = S_ADD_SET;
		} else
			s->state = S_ACTIVE;
		break;
	case S_ADD_SET:
		if (hit(s->reset, a))
			s->state = S_ACTIVE;
		else if (hit(s->addp1, a))
			s->loaded_bank = (uint8_t)((s->loaded_bank + 1) & 3);
		else if (hit(s->addp2, a))
			s->loaded_bank = (uint8_t)((s->loaded_bank + 2) & 3);
		else if (hit(s->add3, a))
			s->state = S_ALT_COMMIT;
		break;
	}
	return s->current_bank;
}
