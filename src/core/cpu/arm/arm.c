/*  Acorn ARM2 / ARM3 CPU (26-bit address bus, PSR held in R15), ported to C from MAME src/devices/cpu/arm
    (arm.cpp, arm.h; MAME 0.289; BSD-3-Clause, copyright-holders: Bryan McPhail, Phil Stroffolino).
    The handlers (HandleALU, HandleMul, HandleMemSingle, HandleMemBlock, HandleBranch, HandleCoPro,
    HandleCoProVL86C020, decodeShift, arm_check_irq_state, execute_run) keep MAME's semantics one to one, including
    its quirks (R15 + 8 / + 12 pipeline offsets, the double read of a word loaded into R15, block writeback rules) and
    its cycle counts (S = N = I = 1 clock), with the same register results, bus accesses (and their order) and clocks.
    ReBOX: the registers of the current mode are kept in r[0..15] (MAME looks every access up through the mode's
    register table); the bank is switched where R15's mode bits change. The copro's BCD packing and divider are done
    without hardware division (the Vita's Cortex-A9 has none); the results are the same as MAME's / and %.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../cheat_cpu.h"
#include "arm.h"
#include <stdio.h>

#include <string.h>

int arm_break;

#if defined(__GNUC__)
#define ARM_INLINE static inline __attribute__((always_inline))
#define LIKELY(x) __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)
#else
#define ARM_INLINE static inline
#define LIKELY(x) (x)
#define UNLIKELY(x) (x)
#endif

enum { MODE_USER = 0, MODE_FIQ = 1, MODE_IRQ = 2, MODE_SVC = 3 };

enum {
	eR0 = 0, eR1, eR2, eR3, eR4, eR5, eR6, eR7, eR8, eR9, eR10, eR11, eR12, eR13, eR14, eR15,
	eR8_FIQ, eR9_FIQ, eR10_FIQ, eR11_FIQ, eR12_FIQ, eR13_FIQ, eR14_FIQ,
	eR13_IRQ, eR14_IRQ,
	eR13_SVC, eR14_SVC
};

// MAME sRegisterTable: the m_sArmRegister index of each visible register in each mode
static const uint8_t regtab[4][16] = {
	{ eR0, eR1, eR2, eR3, eR4, eR5, eR6, eR7, eR8, eR9, eR10, eR11, eR12, eR13, eR14, eR15 },
	{ eR0, eR1, eR2, eR3, eR4, eR5, eR6, eR7, eR8_FIQ, eR9_FIQ, eR10_FIQ, eR11_FIQ, eR12_FIQ, eR13_FIQ, eR14_FIQ, eR15 },
	{ eR0, eR1, eR2, eR3, eR4, eR5, eR6, eR7, eR8, eR9, eR10, eR11, eR12, eR13_IRQ, eR14_IRQ, eR15 },
	{ eR0, eR1, eR2, eR3, eR4, eR5, eR6, eR7, eR8, eR9, eR10, eR11, eR12, eR13_SVC, eR14_SVC, eR15 },
};

#define Z_BIT 30
#define C_BIT 29
#define V_BIT 28
#define N_MASK 0x80000000u
#define Z_MASK 0x40000000u
#define C_MASK 0x20000000u
#define V_MASK 0x10000000u
#define I_MASK 0x08000000u
#define F_MASK 0x04000000u

#define PSR_MASK     0xf0000000u
#define IRQ_MASK     0x0c000000u
#define ADDRESS_MASK 0x03fffffcu
#define MODE_MASK    0x00000003u
#define SIGN_BIT     0x80000000u

#define INSN_SDT_L 0x00100000u
#define INSN_SDT_W 0x00200000u
#define INSN_SDT_B 0x00400000u
#define INSN_SDT_U 0x00800000u
#define INSN_SDT_P 0x01000000u
#define INSN_BDT_L 0x00100000u
#define INSN_BDT_W 0x00200000u
#define INSN_BDT_S 0x00400000u
#define INSN_BDT_U 0x00800000u
#define INSN_BDT_P 0x01000000u
#define INSN_MUL_A 0x00200000u
#define INSN_I     0x02000000u
#define INSN_S     0x00100000u
#define INSN_BL    0x01000000u

#define S_CYCLE 1
#define N_CYCLE 1
#define I_CYCLE 1

#define R15 (c->r[15])
#define MODE (R15 & 3)

// cond_ok[cond] bit f = condition passes for flags nibble f (N=8 Z=4 C=2 V=1)
static uint16_t cond_ok[16];

static void build_cond_table(void)
{
	for (int cond = 0; cond < 16; cond++) {
		uint16_t m = 0;
		for (int f = 0; f < 16; f++) {
			int n = (f >> 3) & 1, z = (f >> 2) & 1, cf = (f >> 1) & 1, v = f & 1, ok;
			switch (cond) {
			case 0x0: ok = z; break;
			case 0x1: ok = !z; break;
			case 0x2: ok = cf; break;
			case 0x3: ok = !cf; break;
			case 0x4: ok = n; break;
			case 0x5: ok = !n; break;
			case 0x6: ok = v; break;
			case 0x7: ok = !v; break;
			case 0x8: ok = cf && !z; break;
			case 0x9: ok = !cf || z; break;
			case 0xa: ok = n == v; break;
			case 0xb: ok = n != v; break;
			case 0xc: ok = !z && n == v; break;
			case 0xd: ok = z || n != v; break;
			case 0xe: ok = 1; break;
			default:  ok = 0; break;   // NV
			}
			if (ok)
				m |= (uint16_t)(1u << f);
		}
		cond_ok[cond] = m;
	}
}

/* ---- register bank ---- */

// r[8..14] of bank_mode go back to their MAME slots, those of mode m come in
static void rebank(arm_t *c, uint32_t m)
{
	const uint32_t o = c->bank_mode;
	const int lo = (o == MODE_FIQ || m == MODE_FIQ) ? 8 : 13;
	for (int i = lo; i < 15; i++)
		c->bank[regtab[o][i] - 8] = c->r[i];
	for (int i = lo; i < 15; i++)
		c->r[i] = c->bank[regtab[m][i] - 8];
	c->bank_mode = (uint8_t)m;
}

// R15 written with mode bits that may change: MAME's later accesses use the new mode's registers
ARM_INLINE void set_r15(arm_t *c, uint32_t v)
{
	c->r[15] = v;
	if (UNLIKELY((v & 3) != c->bank_mode))
		rebank(c, v & 3);
}

// MAME SetRegister (the register may be R15)
ARM_INLINE void SR(arm_t *c, uint32_t i, uint32_t v)
{
	if (i == 15)
		set_r15(c, v);
	else
		c->r[i] = v;
}

// MAME Get/SetModeRegister: register i of the given mode
ARM_INLINE uint32_t *mode_reg(arm_t *c, uint32_t mode, uint32_t i)
{
	const uint32_t idx = regtab[mode][i];
	return idx == regtab[c->bank_mode][i] ? &c->r[i] : &c->bank[idx - 8];
}

/* ---- memory (MAME cpu_read32 / cpu_write32 / cpu_read8 / cpu_write8) ---- */

ARM_INLINE uint32_t rotr32(uint32_t v, uint32_t n) { return (v >> n) | (v << ((32 - n) & 31)); }

ARM_INLINE uint32_t RD32(arm_t *c, uint32_t addr)
{
	uint32_t a = addr & ADDRESS_MASK, v;
	const uint8_t *p = c->rpage[a >> ARM_PAGE_SHIFT];
	if (LIKELY(p != NULL))
		memcpy(&v, p + (a & 0xfff), 4);
	else
		v = c->read32(c->ctx, a);
	// unaligned reads rotate the word, they never combine words
	if (UNLIKELY(addr & 3))
		v = rotr32(v, (addr & 3) << 3);
	return v;
}

ARM_INLINE void WR32(arm_t *c, uint32_t addr, uint32_t v)
{
	// unaligned writes are treated as normal writes
	uint32_t a = addr & ADDRESS_MASK;
	uint8_t *p = c->wpage[a >> ARM_PAGE_SHIFT];
	if (LIKELY(p != NULL))
		memcpy(p + (a & 0xfff), &v, 4);
	else
		c->write32(c->ctx, a, v);
}

ARM_INLINE uint32_t RD8(arm_t *c, uint32_t addr)
{
	uint32_t a = addr & 0x03ffffffu;
	const uint8_t *p = c->rpage[a >> ARM_PAGE_SHIFT];
	if (LIKELY(p != NULL))
		return p[(a & 0xfff) ^ (uint32_t)(-c->big_endian & 3)];
	return c->read8(c->ctx, a);
}

ARM_INLINE void WR8(arm_t *c, uint32_t addr, uint8_t v)
{
	uint32_t a = addr & 0x03ffffffu;
	uint8_t *p = c->wpage[a >> ARM_PAGE_SHIFT];
	if (LIKELY(p != NULL))
		p[(a & 0xfff) ^ (uint32_t)(-c->big_endian & 3)] = v;
	else
		c->write8(c->ctx, a, v);
}

ARM_INLINE uint32_t FETCH(arm_t *c, uint32_t pc)
{
	uint32_t a = pc & ADDRESS_MASK, v;
	const uint8_t *p = c->rpage[a >> ARM_PAGE_SHIFT];
	if (LIKELY(p != NULL)) {
		memcpy(&v, p + (a & 0xfff), 4);
		return v;
	}
	return c->read32(c->ctx, a);
}

/* ---- interrupts ---- */

static void check_irq_state(arm_t *c)
{
	uint32_t pc = R15 + 4;   // save old pc (already incremented in pipeline)

	// exception priorities for ARM2/3: reset, data abort / address exception, FIRQ, IRQ, prefetch abort, undefined
	if (c->pending_fiq && (pc & F_MASK) == 0) {
		if (c->irq_ack)
			c->irq_ack(c->ctx, ARM_FIRQ_LINE);   // MAME standard_irq_callback
		set_r15(c, MODE_FIQ);    // set FIQ mode so PC is saved to the correct R14 bank
		c->r[14] = pc;
		R15 = (pc & PSR_MASK) | (pc & IRQ_MASK) | 0x1c | MODE_FIQ | I_MASK | F_MASK;
		return;
	}
	if (c->pending_irq && (pc & I_MASK) == 0) {
		if (c->irq_ack)
			c->irq_ack(c->ctx, ARM_IRQ_LINE);
		set_r15(c, MODE_IRQ);
		c->r[14] = pc;
		R15 = (pc & PSR_MASK) | (pc & IRQ_MASK) | 0x18 | MODE_IRQ | I_MASK | (pc & F_MASK);
		return;
	}
}

/* ---- barrel shifter (MAME decodeShift), rm already read (+ 8 for R15); the carry out is *pcarry != 0 ---- */

ARM_INLINE uint32_t decode_shift(arm_t *c, uint32_t insn, uint32_t rm, uint32_t *pcarry, int *cyc)
{
	uint32_t k = (insn & 0xf80u) >> 7;
	const uint32_t t = (insn & 0x70u) >> 4;

	if (t & 1) {
		// only the least significant byte of Rs is used (Rs = R15: the whole R15, as MAME)
		k = c->r[k >> 1] & 0xff;
		*cyc += S_CYCLE;
		if (k == 0) {   // register shift by 0 is a no-op
			*pcarry = R15 & C_MASK;
			return rm;
		}
	}

	switch (t >> 1) {
	case 0:   // LSL
		if (k >= 32) {
			*pcarry = (k == 32) ? rm & 1 : 0;
			return 0;
		}
		*pcarry = k ? (rm & (1u << (32 - k))) : (R15 & C_MASK);
		return rm << k;
	case 1:   // LSR
		if (k == 0 || k == 32) {
			*pcarry = rm & SIGN_BIT;
			return 0;
		}
		if (k > 32) {
			*pcarry = 0;
			return 0;
		}
		*pcarry = rm & (1u << (k - 1));
		return rm >> k;
	case 2:   // ASR
		if (k == 0 || k > 32)
			k = 32;
		*pcarry = rm & (1u << (k - 1));
		if (k >= 32)
			return (rm & SIGN_BIT) ? 0xffffffffu : 0;
		if (rm & SIGN_BIT)
			return (rm >> k) | (0xffffffffu << (32 - k));
		return rm >> k;
	default:  // ROR and RRX
		if (k) {
			k = ((k - 1) & 31) + 1;   // MAME: while (k > 32) k -= 32;
			*pcarry = rm & (1u << (k - 1));
			return rotr32(rm, k & 31);
		}
		*pcarry = rm & 1;
		return (rm >> 1) | ((R15 & C_MASK) << 2);
	}
}

// Op2 / the SDT offset from a register (MAME decodeShift): Rm = R15 reads R15 + 8 (MAME, for both shift kinds)
ARM_INLINE uint32_t reg_operand(arm_t *c, uint32_t insn, uint32_t *pcarry, int *cyc)
{
	const uint32_t m = insn & 0xf;
	uint32_t rm = c->r[m];
	if (UNLIKELY(m == 15))
		rm += 8;
	if (LIKELY(!(insn & 0xff0u))) {   // LSL #0
		*pcarry = R15 & C_MASK;
		return rm;
	}
	return decode_shift(c, insn, rm, pcarry, cyc);
}

/* ---- data processing (MAME HandleALU) ---- */

#define NZ_FLAGS(rd) (((rd) & SIGN_BIT) | ((uint32_t)((rd) == 0) << Z_BIT))
// the carry / overflow of a + b (+ carry in) = rd and of a - b (- borrow) = rd: MAME's sign-bit formulas
#define ADD_CV(rd, a, b) ((((((a) & (b)) | (((a) | (b)) & ~(rd))) >> 31) << C_BIT) | \
                          (((((a) ^ (rd)) & ~((a) ^ (b))) >> 31) << V_BIT))
#define SUB_CV(rd, a, b) ((((((a) & ~(b)) | (((a) | ~(b)) & ~(rd))) >> 31) << C_BIT) | \
                          (((((a) ^ (b)) & ((a) ^ (rd))) >> 31) << V_BIT))

// op2 and the shifter carry (sc != 0) are given; cyc holds the clocks so far
ARM_INLINE int alu_exec(arm_t *c, uint32_t insn, uint32_t op2, uint32_t sc, int cyc)
{
	const uint32_t opcode = (insn >> 21) & 0xf;
	const uint32_t r15 = R15;
	uint32_t rn = 0, rd, nr15;

	if ((opcode & 0xd) != 0xd) {   // no Rn in MOV / MVN
		const uint32_t n = (insn >> 16) & 0xf;
		// docs suggest the mode bits should be included, but that breaks Captain America (MAME)
		rn = LIKELY(n != 15) ? c->r[n] : (r15 + 8) & ADDRESS_MASK;
	}

	switch (opcode) {
	case 0x6:  // SBC
		rd = rn - op2 - ((r15 & C_MASK) ? 0 : 1);
		nr15 = SUB_CV(rd, rn, op2);
		goto arith;
	case 0xa:  // CMP
	case 0x2:  // SUB
		rd = rn - op2;
		nr15 = SUB_CV(rd, rn, op2);
		goto arith;
	case 0x7:  // RSC
		rd = op2 - rn - ((r15 & C_MASK) ? 0 : 1);
		nr15 = SUB_CV(rd, op2, rn);
		goto arith;
	case 0x3:  // RSB
		rd = op2 - rn;
		nr15 = SUB_CV(rd, op2, rn);
		goto arith;
	case 0x5:  // ADC
		rd = rn + op2 + ((r15 & C_MASK) >> C_BIT);
		nr15 = ADD_CV(rd, rn, op2);
		goto arith;
	case 0xb:  // CMN
	case 0x4:  // ADD
		rd = rn + op2;
		nr15 = ADD_CV(rd, rn, op2);
		goto arith;
	case 0x8:  // TST
	case 0x0:  // AND
		rd = rn & op2;
		goto logic;
	case 0xe:  // BIC
		rd = rn & ~op2;
		goto logic;
	case 0x9:  // TEQ
	case 0x1:  // EOR
		rd = rn ^ op2;
		goto logic;
	case 0xc:  // ORR
		rd = rn | op2;
		goto logic;
	case 0xd:  // MOV
		rd = op2;
		goto logic;
	default:   // MVN
		rd = ~op2;
		goto logic;
	}
arith:
	if (insn & INSN_S)
		R15 = ((r15 & ~(N_MASK | Z_MASK | V_MASK | C_MASK)) | nr15 | NZ_FLAGS(rd)) + 4;
	else
		R15 = r15 + 4;
	goto store;
logic:
	if (insn & INSN_S)
		R15 = ((r15 & ~(N_MASK | Z_MASK | C_MASK)) | NZ_FLAGS(rd) | ((uint32_t)(sc != 0) << C_BIT)) + 4;
	else
		R15 = r15 + 4;
store:;
	const uint32_t rdn = (insn >> 12) & 0xf;
	if ((opcode & 0xc) != 0x8) {
		if (LIKELY(rdn != 15)) {
			c->r[rdn] = rd;
		} else if (!(insn & INSN_S)) {
			// merge the old NZCV flags (and I/F, mode) into the new PC value
			R15 = (rd & ADDRESS_MASK) | (R15 & ~ADDRESS_MASK);
			cyc += S_CYCLE + N_CYCLE;
		} else {
			// S set: PSR and mode only in a non-user mode
			if (R15 & MODE_MASK)
				set_r15(c, rd);
			else
				R15 = (rd & (ADDRESS_MASK | PSR_MASK)) | (R15 & (IRQ_MASK | MODE_MASK));
			cyc += S_CYCLE + N_CYCLE;
		}
	} else if (rdn == 15 && (insn & INSN_S)) {
		// TSTP / TEQP / CMPP / CMNP: update only the flags (and I/F, mode outside user mode)
		rd = (rd & ~ADDRESS_MASK) | (R15 & ADDRESS_MASK);
		if (R15 & MODE_MASK)
			set_r15(c, rd);
		else
			R15 = (rd & (ADDRESS_MASK | PSR_MASK)) | (R15 & (IRQ_MASK | MODE_MASK));
		cyc += S_CYCLE + N_CYCLE;
	}
	return cyc;
}

/* ---- multiply (MAME HandleMul) ---- */

static void handle_mul(arm_t *c, uint32_t insn)
{
	uint32_t r = c->r[insn & 0xf] * c->r[(insn >> 8) & 0xf];
	if (insn & INSN_MUL_A)
		r += c->r[(insn >> 12) & 0xf];
	SR(c, (insn >> 16) & 0xf, r);
	if (insn & INSN_S)
		R15 = (R15 & ~(N_MASK | Z_MASK)) | NZ_FLAGS(r);
}

/* ---- single data transfer (MAME HandleMemSingle) ---- */

ARM_INLINE int handle_mem_single(arm_t *c, uint32_t insn)
{
	uint32_t rnv, off;
	int cyc = 0;

	if (insn & INSN_I) {
		uint32_t sc_unused;
		off = reg_operand(c, insn, &sc_unused, &cyc);
	} else {
		off = insn & 0xfff;
	}

	const uint32_t rn = (insn >> 16) & 0xf;
	const uint32_t base = LIKELY(rn != 15) ? c->r[rn] : (R15 & ADDRESS_MASK) + 8;
	if (insn & INSN_SDT_P)   // pre-indexed (R15: (pc ± off) + 8 = (pc + 8) ± off)
		rnv = (insn & INSN_SDT_U) ? base + off : base - off;
	else                     // post-indexed
		rnv = base;

	const uint32_t rd = (insn >> 12) & 0xf;
	if (insn & INSN_SDT_L) {
		cyc += S_CYCLE + I_CYCLE + N_CYCLE;
		if (insn & INSN_SDT_B) {
			SR(c, rd, RD8(c, rnv));
		} else if (LIKELY(rd != 15)) {
			c->r[rd] = RD32(c, rnv);
		} else {
			R15 = (RD32(c, rnv) & ADDRESS_MASK) | (R15 & ~ADDRESS_MASK);
			// MAME: the bottom bits should be masked off, but World Cup Volleyball 95 makes an unaligned jump that
			// must continue from the rounded-up address; otherwise 4 is taken off for the pipeline. MAME reads the
			// word a second time here (kept: the bus sees both reads).
			if (c->copro_type == ARM_COPRO_VL86C020 || (RD32(c, rnv) & 3) == 0)
				R15 -= 4;
			cyc += S_CYCLE + N_CYCLE;
		}
	} else {
		cyc += 2 * N_CYCLE;
		if (insn & INSN_SDT_B)
			WR8(c, rnv, (uint8_t)(c->r[rd] & 0xff));
		else
			WR32(c, rnv, rd == 15 ? R15 + 8 : c->r[rd]);
	}

	// writeback. MAME's "SetRegister(rn, GetRegister(rd))" when rd == rn (a load's value wins) changes nothing
	if (insn & INSN_SDT_P) {
		if ((insn & INSN_SDT_W) && !((insn & INSN_SDT_L) && rd == rn))
			SR(c, rn, rnv);
	} else if (rd != rn) {   // post-indexed: always (W is ignored)
		SR(c, rn, (insn & INSN_SDT_U) ? rnv + off : rnv - off);
	}
	return cyc;
}

/* ---- block data transfer (MAME HandleMemBlock, loadInc / loadDec / storeInc / storeDec) ---- */

static int load_inc(arm_t *c, uint32_t pat, uint32_t rbv, uint32_t s)
{
	int result = 0;
	for (uint32_t p = pat & 0x7fff; p; p &= p - 1) {
		rbv += 4;
		c->r[__builtin_ctz(p)] = RD32(c, rbv);
		result++;
	}
	if (pat & 0x8000) {
		rbv += 4;
		if (s)   // pull full contents from the stack
			set_r15(c, RD32(c, rbv));
		else     // pull only the address, preserve mode and status flags
			R15 = (R15 & ~ADDRESS_MASK) | (RD32(c, rbv) & ADDRESS_MASK);
		result++;
	}
	return result;
}

static int load_dec(arm_t *c, uint32_t pat, uint32_t rbv, uint32_t s, uint32_t *deferred_r15, int *defer)
{
	int result = 0;
	if (pat & 0x8000) {
		*defer = 1;
		rbv -= 4;
		if (s)
			*deferred_r15 = RD32(c, rbv);
		else
			*deferred_r15 = (R15 & ~ADDRESS_MASK) | (RD32(c, rbv) & ADDRESS_MASK);
		result++;
	}
	for (uint32_t p = pat & 0x7fff; p; ) {
		const int i = 31 - __builtin_clz(p);
		p &= ~(1u << i);
		rbv -= 4;
		c->r[i] = RD32(c, rbv);
		result++;
	}
	return result;
}

static int store_inc(arm_t *c, uint32_t pat, uint32_t rbv)
{
	int result = 0;
	for (uint32_t p = pat & 0xffff; p; p &= p - 1) {
		rbv += 4;
		WR32(c, rbv, c->r[__builtin_ctz(p)]);
		result++;
	}
	return result;
}

static int store_dec(arm_t *c, uint32_t pat, uint32_t rbv)
{
	int result = 0;
	for (uint32_t p = pat & 0xffff; p; ) {
		const int i = 31 - __builtin_clz(p);
		p &= ~(1u << i);
		rbv -= 4;
		WR32(c, rbv, c->r[i]);
		result++;
	}
	return result;
}

static int handle_mem_block(arm_t *c, uint32_t insn)
{
	const uint32_t rb = (insn >> 16) & 0xf;
	uint32_t rbp = c->r[rb];
	const uint32_t pat = insn & 0xffff;
	int result, cyc = 0;

	if (insn & INSN_BDT_L) {
		if (insn & INSN_BDT_U) {
			const uint32_t mode = MODE;
			if (!(insn & INSN_BDT_P))
				rbp -= 4;
			// S set, R15 not in the list: transfer to the user bank
			if ((insn & INSN_BDT_S) && !(insn & 0x8000)) {
				const uint32_t curmode = MODE;
				set_r15(c, R15 & ~MODE_MASK);
				result = load_inc(c, pat, rbp, insn & INSN_BDT_S);
				set_r15(c, R15 | curmode);
			} else {
				result = load_inc(c, pat, rbp, insn & INSN_BDT_S);
			}
			if (insn & 0x8000) {
				R15 -= 4;
				cyc += S_CYCLE + N_CYCLE;
			}
			if (insn & INSN_BDT_W) {
				// MAME skips the writeback when the base is in the list (Fighter's History LDMUW [R13], {R13-R15});
				// it goes to the register of the mode before the transfer
				if ((insn & (1u << rb)) == 0)
					*mode_reg(c, mode, rb) += (uint32_t)result * 4;
			}
		} else {
			uint32_t deferred_r15 = 0;
			int defer = 0;
			if (!(insn & INSN_BDT_P))
				rbp += 4;
			if ((insn & INSN_BDT_S) && !(insn & 0x8000)) {
				const uint32_t curmode = MODE;
				set_r15(c, R15 & ~MODE_MASK);
				result = load_dec(c, pat, rbp, insn & INSN_BDT_S, &deferred_r15, &defer);
				set_r15(c, R15 | curmode);
			} else {
				result = load_dec(c, pat, rbp, insn & INSN_BDT_S, &deferred_r15, &defer);
			}
			if (insn & INSN_BDT_W)
				SR(c, rb, c->r[rb] - (uint32_t)result * 4);
			// R15 pulled from memory is set after the writeback (which must use the old mode's bank)
			if (defer)
				set_r15(c, deferred_r15);
			if (insn & 0x8000) {
				cyc += S_CYCLE + N_CYCLE;
				R15 -= 4;
			}
		}
		cyc += result * S_CYCLE + N_CYCLE + I_CYCLE;
	} else {
		if (insn & 0x8000)
			R15 += 12;   // stored R15 is the STM address + 12
		if (insn & INSN_BDT_U) {
			if (!(insn & INSN_BDT_P))
				rbp -= 4;
			if (insn & INSN_BDT_S) {   // S set: transfer from the user bank
				const uint32_t curmode = MODE;
				set_r15(c, R15 & ~MODE_MASK);
				result = store_inc(c, pat, rbp);
				set_r15(c, R15 | curmode);
			} else {
				result = store_inc(c, pat, rbp);
			}
			if (insn & INSN_BDT_W)
				SR(c, rb, c->r[rb] + (uint32_t)result * 4);
		} else {
			if (!(insn & INSN_BDT_P))
				rbp += 4;
			if (insn & INSN_BDT_S) {
				const uint32_t curmode = MODE;
				set_r15(c, R15 & ~MODE_MASK);
				result = store_dec(c, pat, rbp);
				set_r15(c, R15 | curmode);
			} else {
				result = store_dec(c, pat, rbp);
			}
			if (insn & INSN_BDT_W)
				SR(c, rb, c->r[rb] - (uint32_t)result * 4);
		}
		if (insn & 0x8000)
			R15 -= 12;
		cyc += (result - 1) * S_CYCLE + 2 * N_CYCLE;
	}
	return cyc;
}

/* ---- coprocessors (MAME HandleCoPro: Data East 156; HandleCoProVL86C020) ---- */

static uint32_t bcd_to_decimal(uint32_t value)
{
	uint32_t acc = 0, mul = 1;
	for (int i = 0; i < 8; i++) {
		acc += (value & 0xf) * mul;
		mul *= 10;
		value >>= 4;
	}
	return acc;
}

// MAME DecimalToBCD: the low 8 decimal digits of value, packed (v / 10 by multiply: exact for any 32-bit v)
static uint32_t decimal_to_bcd(uint32_t value)
{
	uint32_t acc = 0;
	for (int i = 0; i < 8; i++) {
		uint32_t q = (uint32_t)(((uint64_t)value * 0xcccccccdu) >> 35);
		acc += (value - q * 10) << (i * 4);
		value = q;
	}
	return acc;
}

// unsigned 32-bit divide by shift and subtract (n / d, n % d), d != 0
static void udivmod32(uint32_t n, uint32_t d, uint32_t *q, uint32_t *r)
{
	uint32_t quo = 0, rem = 0;
	for (int i = 31; i >= 0; i--) {
		uint64_t t = ((uint64_t)rem << 1) | ((n >> i) & 1);
		if (t >= d) {
			t -= d;
			quo |= 1u << i;
		}
		rem = (uint32_t)t;
	}
	*q = quo;
	*r = rem;
}

static void handle_copro_vl86c020(arm_t *c, uint32_t insn)
{
	uint32_t rn = (insn >> 12) & 0xf, crn = (insn >> 16) & 0xf;
	if ((insn & 0x0f100010u) == 0x0e100010u) {          // MRC
		// crn 0: ID, read only (Acorn designer 0x41, VLSI 0x56, part VL86C020 0x03, revision 0)
		SR(c, rn, crn == 0 ? 0x41560300u : c->copro[crn]);
	} else if ((insn & 0x0f100010u) == 0x0e000010u) {   // MCR
		if (crn != 0)
			c->copro[crn] = c->r[rn];
	}
	// else: unimplemented (MAME prints and breaks into the debugger)
}

static void handle_copro(arm_t *c, uint32_t insn)
{
	uint32_t rn = (insn >> 12) & 0xf, crn = (insn >> 16) & 0xf;
	if ((insn & 0x0f100010u) == 0x0e100010u) {          // MRC
		SR(c, rn, c->copro[crn]);
	} else if ((insn & 0x0f100010u) == 0x0e000010u) {   // MCR
		c->copro[crn] = c->r[rn];
		// Data East 156 copro: writing CR2 starts a BCD operation, result in CR5
		if (crn == 2) {
			uint32_t v0 = bcd_to_decimal(c->copro[0]), v1 = bcd_to_decimal(c->copro[1]);
			if (c->copro[2] == 0)
				c->copro[5] = decimal_to_bcd(v0 + v1);
			else if (c->copro[2] == 1)
				c->copro[5] = decimal_to_bcd(v0 * v1);
			else if (c->copro[2] == 3)
				c->copro[5] = decimal_to_bcd(v0 - v1);
		}
	} else if ((insn & 0x0f000010u) == 0x0e000000u) {   // CDP
		// Data East 156 copro divider: CR3 = CR0 / CR1, CR4 = CR0 % CR1
		if (c->copro[1]) {
			udivmod32(c->copro[0], c->copro[1], &c->copro[3], &c->copro[4]);
		} else {
			c->copro[3] = 0xffffffffu;   // unverified (MAME)
			c->copro[4] = 0xffffffffu;
		}
	}
}

/* ---- execute (MAME execute_run, one pass of its loop) ---- */

ARM_INLINE int exec_one(arm_t *c)
{
	if (UNLIKELY(c->pending_irq | c->pending_fiq)) {
		// (only when one can be taken: a line held while masked (tattass' IRQ through its handler) called it for
		// nothing at every instruction, a quarter of them)
		const uint32_t pc4 = R15 + 4;
		if ((c->pending_fiq && !(pc4 & F_MASK)) || (c->pending_irq && !(pc4 & I_MASK)))
			check_irq_state(c);
	}

	const uint32_t pc = R15;
	const uint32_t insn = FETCH(c, pc);
	const uint32_t cond = insn >> 28;

	if (cond != 0xe && !((cond_ok[cond] >> (pc >> 28)) & 1)) {
		R15 = pc + 4;
		return S_CYCLE;
	}

	switch ((insn >> 25) & 7) {
	case 0:
		if (UNLIKELY((insn & 0x0fc000f0u) == 0x00000090u)) {   // multiply
			handle_mul(c, insn);
			R15 += 4;
			return S_CYCLE + I_CYCLE;
		} else {
			int cyc = S_CYCLE;
			uint32_t sc;
			const uint32_t op2 = reg_operand(c, insn, &sc, &cyc);
			return alu_exec(c, insn, op2, sc, cyc);
		}
	case 1: {   // data processing, immediate: imm8 ror 2 * rot; the carry is bit 31 if rotated, else C
		const uint32_t rot = (insn >> 7) & 0x1e;
		const uint32_t op2 = rotr32(insn & 0xff, rot);
		const uint32_t sc = rot ? op2 & SIGN_BIT : pc & C_MASK;
		return alu_exec(c, insn, op2, sc, S_CYCLE);
	}
	case 2:
	case 3: {   // single data transfer
		int cyc = handle_mem_single(c, insn);
		R15 += 4;
		return cyc;
	}
	case 4: {   // block data transfer
		int cyc = handle_mem_block(c, insn);
		R15 += 4;
		return cyc;
	}
	case 5: {   // branch
		if (insn & INSN_BL)
			c->r[14] = pc + 4;
		// the 24-bit word offset, sign-extended (bit 23 -> bit 25 of the byte offset)
		const uint32_t soff = (uint32_t)((int32_t)(insn << 8) >> 6);
		R15 = ((pc + soff + 8) & ADDRESS_MASK) | (pc & ~ADDRESS_MASK);
		return 2 * S_CYCLE + N_CYCLE;
	}
	case 6:     // coprocessor data transfer (LDC / STC): undefined on this core
		R15 = pc + 4;
		return S_CYCLE;
	default:
		if (!(insn & 0x01000000u)) {   // coprocessor data operation / register transfer
			if (c->copro_type == ARM_COPRO_VL86C020)
				handle_copro_vl86c020(c, insn);
			else
				handle_copro(c, insn);
			R15 += 4;
			return S_CYCLE;
		} else {                       // software interrupt
			const uint32_t rpc = pc + 4;
			set_r15(c, MODE_SVC);   // set SVC mode so PC is saved to the correct R14 bank
			c->r[14] = rpc;
			R15 = (rpc & PSR_MASK) | (rpc & IRQ_MASK) | 0x8 | MODE_SVC | I_MASK | (rpc & MODE_MASK);
			return 2 * S_CYCLE + N_CYCLE;
		}
	}
}

/* ---- API ---- */

// (the cheats' access to this CPU's program space: cheat_cpu.h)
static uint8_t cheat_rd(void *x, uint32_t a)   // (as the CPU: its pages, the byte lane of its bus, else the callbacks)
{
	arm_t *c = (arm_t *)x;
	a &= 0x3ffffff;
	const uint8_t *p = c->rpage[a >> ARM_PAGE_SHIFT];
	return p ? p[(a & 0xfff) ^ (c->big_endian ? 3u : 0u)] : c->read8(c->ctx, a);
}
static void cheat_wr(void *x, uint32_t a, uint8_t v)
{
	arm_t *c = (arm_t *)x;
	a &= 0x3ffffff;
	uint8_t *p = c->wpage[a >> ARM_PAGE_SHIFT];
	if (p)
		p[(a & 0xfff) ^ (c->big_endian ? 3u : 0u)] = v;
	else
		c->write8(c->ctx, a, v);
}

void arm_init(arm_t *c)
{
	rbx_cheat_offer("arm", cheat_rd, cheat_wr, c, 0, 1);
	build_cond_table();
	memset(c, 0, sizeof(*c));
	c->bank_mode = MODE_SVC;
}

void arm_reset(arm_t *c)
{
	memset(c->r, 0, sizeof(c->r));
	memset(c->bank, 0, sizeof(c->bank));
	memset(c->copro, 0, sizeof(c->copro));
	c->pending_irq = 0;
	c->pending_fiq = 0;
	c->cycles = 0;
	// start up in SVC mode with interrupts disabled
	c->r[15] = MODE_SVC | I_MASK | F_MASK;
	c->bank_mode = MODE_SVC;
}

uint32_t *arm_reg(arm_t *c, int index)
{
	for (int i = 0; i < 16; i++)
		if (regtab[c->bank_mode][i] == index)
			return &c->r[i];
	return &c->bank[index - 8];
}

void arm_set_r15(arm_t *c, uint32_t v) { set_r15(c, v); }

int arm_step(arm_t *c)
{
	if (UNLIKELY((R15 & 3) != c->bank_mode))
		rebank(c, R15 & 3);
	int n = exec_one(c);
	c->cycles += n;
	return n;
}

void arm_run(arm_t *c, int64_t until)
{
	if (UNLIKELY((R15 & 3) != c->bank_mode))
		rebank(c, R15 & 3);
	while (c->cycles < until) {
#ifdef RBX_ARM_PCPROF
		{	// (profiling builds: the main program's PCs counted, dumped every 0x400000 instructions)
			static uint32_t hist[0x40000];
			static uint64_t n;
			hist[((R15 & 0x03fffffc) >> 2) & 0x3ffff]++;
			if (!(++n & 0x3fffff)) {
				FILE *f = fopen("/tmp/armpc.txt", "w");
				for (int i = 0; f && i < 0x40000; i++)
					if (hist[i])
						fprintf(f, "%x %u\n", i << 2, hist[i]);
				if (f)
					fclose(f);
			}
		}
#endif
		c->cycles += exec_one(c);   // kept current: memory handlers read (and move) the CPU time
		if (UNLIKELY(arm_break))
			break;
	}
}

void arm_set_irq(arm_t *c, int state) { c->pending_irq = state ? 1 : 0; }
void arm_set_fiq(arm_t *c, int state) { c->pending_fiq = state ? 1 : 0; }
