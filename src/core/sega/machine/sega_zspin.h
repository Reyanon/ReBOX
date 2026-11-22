/*  The sound Z80's polling loops skipped within a run (hashes unchanged): Sega X-Board / Y-Board.
    A Z80 in a short loop that only reads mapped memory (z->read_map) or ports the board says read the same every
    time, with no interrupt it would take (no NMI pending, no EI pending, the IRQ line clear or masked), is run to the
    loop's head and one turn an instruction at a time (z80_run's own steps); if that turn comes back to the head with
    every register as it was (R and the cycles aside), the next turns are the same, so as many as fit before the run's
    end are counted (cycles and R) and z80_run does the rest, ending where running every turn ends.
    Recognised: ld a,(nn) / in a,(n) / or a / and a / cp n / and n / bit b,r / nop and jr / jp forward (out), ending
    in a jr / jp back to the head.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_SEGA_ZSPIN_H
#define RBX_SEGA_ZSPIN_H

#include <string.h>
#include "../../cpu/z80/z80.h"

static inline int sega_zspin_rd(const z80 *z, uint16_t a, int op)
{
	if (op && z->op_rom)
		return z->op_rom[a];
	const uint8_t *p = z->read_map ? z->read_map[a >> 8] : NULL;
	return p ? p[a & 0xff] : -1;
}

// one of those instructions at a: its length (0: not one), *t its branch's target (-1: none), *jmp an unconditional one
static inline int sega_zspin_ins(const z80 *z, uint16_t a, int (*port_fixed)(const z80 *, uint8_t), int *t, int *jmp)
{
	const int op = sega_zspin_rd(z, a, 1), p1 = sega_zspin_rd(z, (uint16_t)(a + 1), 0);
	*t = -1;
	*jmp = 0;
	if (op < 0 || p1 < 0)
		return 0;
	switch (op) {
	case 0x00: case 0xb7: case 0xa7: return 1;   // nop / or a / and a
	case 0xfe: case 0xe6: return 2;              // cp n / and n
	case 0x3a: {                                 // ld a,(nn)
		const int p2 = sega_zspin_rd(z, (uint16_t)(a + 2), 0);
		return p2 >= 0 && sega_zspin_rd(z, (uint16_t)(p1 | p2 << 8), 0) >= 0 ? 3 : 0;
	}
	case 0xdb: return port_fixed && port_fixed(z, (uint8_t)p1) ? 2 : 0;   // in a,(n)
	case 0xcb: {                                 // bit b,r
		const int o = sega_zspin_rd(z, (uint16_t)(a + 1), 1);
		return o >= 0x40 && o <= 0x7f && (o & 7) != 6 ? 2 : 0;
	}
	case 0x18: case 0x20: case 0x28: case 0x30: case 0x38:   // jr (cc)
		*t = (uint16_t)(a + 2 + (int8_t)p1);
		*jmp = op == 0x18;
		return 2;
	case 0xc3: case 0xc2: case 0xca: case 0xd2: case 0xda: {   // jp (cc)
		const int p2 = sega_zspin_rd(z, (uint16_t)(a + 2), 0);
		if (p2 < 0)
			return 0;
		*t = p1 | p2 << 8;
		*jmp = op == 0xc3;
		return 3;
	}
	default: return 0;
	}
}

// the head of such a loop that z->pc is in (-1: none): from the pc to a branch back to the pc or a little before,
// then from that head round to it, the branches on the way only forward. *bound: its instructions (bit n: head + n)
static inline int sega_zspin_head(const z80 *z, int (*port_fixed)(const z80 *, uint8_t), uint32_t *bound)
{
	uint16_t a = z->pc;
	int head = -1, t, jmp;
	for (int n = 0; n < 8 && head < 0; n++) {
		const int len = sega_zspin_ins(z, a, port_fixed, &t, &jmp);
		if (!len || (jmp && (uint16_t)(z->pc - t) > 12))
			return -1;
		if (t >= 0 && (uint16_t)(z->pc - t) <= 12)
			head = t;
		a = (uint16_t)(a + len);
	}
	if (head < 0)
		return -1;
	a = (uint16_t)head;
	*bound = 0;
	for (int n = 0; n < 8; n++) {
		const int len = sega_zspin_ins(z, a, port_fixed, &t, &jmp);
		if (!len || (uint16_t)(a - head) >= 32)
			return -1;
		*bound |= 1u << (uint16_t)(a - head);
		if (t == head)
			return head;
		if (jmp || (t >= 0 && (uint16_t)(t - head) <= (uint16_t)(a - head)))
			return -1;
		a = (uint16_t)(a + len);
	}
	return -1;
}

// before z80_run(z, end, line, 0): the turns of a polling loop at z->pc that surely fit, counted
static inline void sega_zspin(z80 *z, unsigned long end, int *line, int (*port_fixed)(const z80 *, uint8_t))
{
	if (z->halted || z->nmi_pending || z->iff_delay || (*line && z->iff1) || (long)(end - z->cyc) < 96)
		return;
	uint32_t bound;
	const int head = sega_zspin_head(z, port_fixed, &bound);
	if (head < 0)
		return;
	for (int n = 0; z->pc != head; n++) {   // to the head, then one turn (z80_run's own steps: an instruction each)
		if (n == 8 || (long)(end - z->cyc) <= 0)
			return;
		z80_run(z, z->cyc + 1, line, 0);
	}
	z80 z0 = *z;
	for (int n = 0;; n++) {   // (only the loop's own instructions: an exit taken ends it)
		if (n == 16 || (long)(end - z->cyc) <= 0)
			return;
		z80_run(z, z->cyc + 1, line, 0);
		if (z->pc == z0.pc)
			break;
		const uint16_t o = (uint16_t)(z->pc - head);
		if (o >= 32 || !((bound >> o) & 1))
			return;
	}
	const unsigned long turn = z->cyc - z0.cyc;
	const unsigned rinc = (unsigned)(z->r - z0.r) & 0x7f;
	z80 z1 = *z;
	z1.cyc = z0.cyc;
	z1.r = z0.r;
	if (memcmp(&z0, &z1, sizeof(z80)) || (long)(end - z->cyc) <= (long)turn)
		return;
	const unsigned long k = (end - z->cyc - 1) / turn;
	z->cyc += k * turn;
	z->r = (uint8_t)((z->r & 0x80) | ((z->r + k * rinc) & 0x7f));
}

#endif
