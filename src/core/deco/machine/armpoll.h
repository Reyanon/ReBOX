/*  The DECO ARM boards' polling-loop skip (results unchanged). A loop that only reads a location and compares it reads
    the same value at the same interval until the location can change, at a line boundary (an input port; or work
    RAM only the CPU's interrupt writes, the interrupt coming at a line's start):
      [ADD rc, rc, #k]                       (optional: a counter of the time waited, wcvol95's 0x38cc0)
      or [MOV rn, #imm]                      (optional: the address set in the loop, rd may be rn; nslasher's 0x9c4)
      LDR / LDRB rd, [rn, #imm]              (no writeback; or post-indexed by 0)
      TST / TEQ / CMP rd, #imm or rd, rm, or rn, rd   (registers unshifted; fghthist's 0x9cf0: cmp r1, r3)
      or MOVS rd, rd                         (osman's 0x5978)
      [ADD rc, rc, #k]                       (optional: the counter after the compare, skullfng's 0x6e88)
      B<cond> back to the loop's first instruction
    When the same pc reads the same value at the same interval again and the instructions are that loop, the whole
    intervals up to the line's end are passed at once (the counter given the iterations' k): the same reads, the same
    registers and cycle count as without (docs/PERF.md 2026-11-03: Joe & Mac Returns' vblank wait, 67000 reads a frame).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_ARMPOLL_H
#define RBX_ARMPOLL_H
#include <stdint.h>
#include "../../cpu/arm/arm.h"

typedef struct {
	uint32_t pc, val;
	int64_t cyc, period;
	int64_t slice_end;          // the cycle count the current line's run ends at (the board sets it)
	uint32_t loop_pc;           // the last pc looked at: whether it is such a loop, its counter
	int loop_ok, counter_reg;   // counter_reg -1: none
	uint32_t counter_add;
	uint32_t w_ld, w_cmp, w_br;  // the loop's read, compare and branch
} armpoll_t;

static inline void armpoll_reset(armpoll_t *p)
{
	p->pc = p->val = 0;
	p->cyc = p->period = 0;
	p->loop_pc = 0xffffffffu;
	p->loop_ok = 0;
	p->counter_reg = -1;
	p->counter_add = 0;
}

static inline uint32_t armpoll_word(const uint8_t *rom, uint32_t a)
{
	const uint8_t *b = rom + a;
	return (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24;
}

// the instructions at pc, the read (in rom, little-endian words): see above
static inline int armpoll_is_loop(armpoll_t *p, uint32_t pc, const uint8_t *rom, uint32_t rom_size)
{
	if (pc == p->loop_pc)
		return p->loop_ok;
	p->loop_pc = pc;
	p->loop_ok = 0;
	p->w_ld = armpoll_word(rom, pc);
	p->w_cmp = armpoll_word(rom, pc + 4);
	p->w_br = armpoll_word(rom, pc + 8);
	p->counter_reg = -1;
	if (pc < 4 || pc + 12 > rom_size)
		return 0;
	const uint32_t wa = armpoll_word(rom, pc - 4), w0 = armpoll_word(rom, pc), w1 = armpoll_word(rom, pc + 4),
	               w2 = armpoll_word(rom, pc + 8);
	const uint32_t rd = (w0 >> 12) & 15, rn = (w0 >> 16) & 15;
	// MOV rn, #imm before the read, the branch back to it: rn set again each time (rd may be rn)
	const int mov = (wa & 0xffff0000u) == 0xe3a00000u && ((wa >> 12) & 15) == rn && rn != 15 &&
	                (w2 & 0x00ffffffu) == 0x00fffffbu;
	// LDR(B) rd, [rn, #imm], or LDR(B) rd, [rn], #0 (post-indexed by 0: rn unchanged; dragngun's 0x6288)
	const int post0 = (w0 & 0x0f300fffu) == 0x04100000u;
	if (((w0 & 0x0f300000u) != 0x05100000u && !post0) || rd == 15 || (rn == rd && !mov))
		return 0;
	// the compare: TST 0x11, TEQ 0x13, CMP 0x15 with S; immediate, or a register unshifted
	const uint32_t op = (w1 >> 20) & 0xff;
	// MOVS rd, rd (unshifted: rd unchanged, the flags from it) as a compare of rd with itself
	const int movs = op == 0x1b && !(w1 & 0x000f0ff0u) && ((w1 >> 12) & 15) == rd && (w1 & 15) == rd;
	const int imm = (op == 0x31 || op == 0x33 || op == 0x35) || movs, reg = (op == 0x11 || op == 0x13 || op == 0x15);
	const uint32_t c_rn = movs ? rd : (w1 >> 16) & 15, c_rm = w1 & 15;
	if ((w1 & 0xf0000000u) != 0xe0000000u || !(imm || reg) || (reg && (w1 & 0xff0u)))
		return 0;
	if (c_rn != rd && !(reg && c_rm == rd))   // (the value read compared, as either operand)
		return 0;
	const uint32_t rm = c_rn == rd ? c_rm : c_rn;   // (the other register: not written in the loop)
	if ((w2 & 0xfff00000u) == 0xe2800000u && pc + 16 <= rom_size && !mov) {
		// ADD rc, rc, #k after the compare (no flags), then B<cond> back to the read: skullfng's 0x6e88
		const uint32_t w3 = armpoll_word(rom, pc + 12), rc2 = (w2 >> 12) & 15;
		if ((w3 & 0x0f000000u) == 0x0a000000u && (w3 & 0x00ffffffu) == 0x00fffffbu && ((w2 >> 16) & 15) == rc2 &&
		    rc2 < 8 && rc2 != rd && rc2 != rn && !(reg && (rm == rc2 || rm == 15))) {
			const uint32_t k = w2 & 0xff, rot = ((w2 >> 8) & 15) * 2;
			p->counter_reg = (int)rc2;
			p->counter_add = rot ? (k >> rot) | (k << (32 - rot)) : k;
			p->w_br = w3;
			p->loop_ok = 1;
		}
		return p->loop_ok;
	}
	if ((w2 & 0x0f000000u) != 0x0a000000u)
		return 0;
	if ((w2 & 0x00ffffffu) == 0x00fffffcu) {   // B<cond> pc: the read starts the loop
		p->loop_ok = !(reg && rm == 15);
		return p->loop_ok;
	}
	if (mov) {
		p->loop_ok = !(reg && (rm == 15 || rm == rn));
		return p->loop_ok;
	}
	// B<cond> pc - 4: ADD rc, rc, #k before the read (rc a register of every mode's bank: r0-r7)
	const uint32_t rc = (wa >> 12) & 15;
	if ((w2 & 0x00ffffffu) == 0x00fffffbu && (wa & 0xfff00000u) == 0xe2800000u && ((wa >> 16) & 15) == rc && rc < 8 &&
	    rc != rd && rc != rn && !(reg && (rm == rc || rm == 15))) {
		const uint32_t k = wa & 0xff, rot = ((wa >> 8) & 15) * 2;
		p->counter_reg = (int)rc;
		p->counter_add = rot ? (k >> rot) | (k << (32 - rot)) : k;
		p->loop_ok = 1;
	}
	return p->loop_ok;
}

// whether the loop's branch is taken for the word v read (the loop goes round again): an exit (the value it waits
// for, or one an outer loop also reads at the same interval: dragngun's 0x6648) is not skipped
static inline int armpoll_continues(const armpoll_t *p, const arm_t *cpu, uint32_t v)
{
	const uint32_t w0 = p->w_ld, w1 = p->w_cmp, rd = (w0 >> 12) & 15, rn = (w0 >> 16) & 15;
	if (rn == 15)
		return 0;
	uint32_t ea = cpu->r[rn];
	if (w0 & (1u << 24))
		ea = (w0 & (1u << 23)) ? ea + (w0 & 0xfff) : ea - (w0 & 0xfff);
	uint32_t val;
	if (w0 & (1u << 22))
		val = (v >> (8 * ((cpu->big_endian ? 3 - (ea & 3) : ea & 3)))) & 0xff;
	else if (ea & 3)
		return 0;
	else
		val = v;
	uint32_t flags = cpu->r[15] >> 28;   // NZCV
	int c = (flags >> 1) & 1, vf = flags & 1;
	const uint32_t op = (w1 >> 20) & 0xff;
	uint32_t res;
	if (op == 0x1b)   // MOVS rd, rd
		res = val;
	else {
		uint32_t a, b;
		const uint32_t c_rn = (w1 >> 16) & 15;
		if (op & 0x20) {   // immediate
			const uint32_t k = w1 & 0xff, rot = ((w1 >> 8) & 15) * 2;
			a = val;
			b = rot ? (k >> rot) | (k << (32 - rot)) : k;
			if (rot && op != 0x35)
				c = b >> 31;
		} else {
			const uint32_t c_rm = w1 & 15;
			a = c_rn == rd ? val : cpu->r[c_rn];
			b = c_rm == rd ? val : cpu->r[c_rm];
		}
		switch (op & 0x1f) {
		case 0x11: res = a & b; break;
		case 0x13: res = a ^ b; break;
		default:
			res = a - b;
			c = a >= b;
			vf = (int)(((a ^ b) & (a ^ res)) >> 31);
			break;
		}
	}
	const int n = (int)(res >> 31), z = res == 0;
	int t;
	switch (p->w_br >> 28) {
	case 0x0: t = z; break;
	case 0x1: t = !z; break;
	case 0x2: t = c; break;
	case 0x3: t = !c; break;
	case 0x4: t = n; break;
	case 0x5: t = !n; break;
	case 0x6: t = vf; break;
	case 0x7: t = !vf; break;
	case 0x8: t = c && !z; break;
	case 0x9: t = !c || z; break;
	case 0xa: t = n == vf; break;
	case 0xb: t = n != vf; break;
	case 0xc: t = !z && n == vf; break;
	case 0xd: t = z || n != vf; break;
	case 0xe: t = 1; break;
	default: t = 0; break;
	}
	return t;
}

// call from the location's read handler with the value read (the program in rom: pcs below rom_size)
static inline void armpoll_read(armpoll_t *p, arm_t *cpu, uint32_t v, const uint8_t *rom, uint32_t rom_size)
{
	const uint32_t pc = arm_pc(cpu);
	const int64_t now = cpu->cycles, period = now - p->cyc;
	const int same = pc == p->pc && v == p->val;
	if (same && period > 0 && period == p->period && armpoll_is_loop(p, pc, rom, rom_size) &&
	    armpoll_continues(p, cpu, v)) {
		const int64_t left = p->slice_end - now;
		if (left > period) {
			const int64_t k = (left - 1) / period;   // (the reads up to the line's end, at the same phase)
			cpu->cycles += k * period;
			if (p->counter_reg >= 0)
				cpu->r[p->counter_reg] += (uint32_t)k * p->counter_add;
		}
	}
	p->period = same ? period : 0;
	p->pc = pc;
	p->val = v;
	p->cyc = cpu->cycles;
}
#endif
