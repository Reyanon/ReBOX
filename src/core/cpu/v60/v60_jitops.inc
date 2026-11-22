/*  V60 JIT (ADR-046): the native instructions (included by v60_jit.inc). Each one is MAME's handler (v60 op*.hxx as in
    v60_ops.inc, BSD-3-Clause, copyright-holders: Farfetch'd, R. Belmont) in host code: its operands decoded in the
    interpreter's order (register reads and updates, then the accesses), the same accesses, the same flags.
    This file is part of ReBOX. GPL-2.0-or-later. */

// a forward branch in the hot code, patched to where the code is later
static unsigned e_bfwd(uint32_t cond) { E(cond << 28 | 0x0a000000u); return s_nb - 1; }
static void e_here(unsigned at) { s_vb[at] |= (s_nb - at - 2) & 0x00ffffffu; }

// the shift that puts a dim's value at the top of a word (0 byte: 24, 1 half: 16, 2 word: 0)
static inline uint32_t vsh(int dim) { return dim == 0 ? 24u : dim == 1 ? 16u : 0u; }

// SP update: SP += d (d may be negative); r0 = the new SP
static void e_sp_add(int32_t d)
{
	e_ldr(0, V_REG(31));
	e_addi(0, 0, (uint32_t)d);
	e_str(0, V_REG(31));
}

// NORMALIZEFLAGS: each flag byte to 0 / 1 (its bits or'ed into bit 0: the shifts' spill into the byte above is masked)
static void e_normflags(void)
{
	e_ldr(2, V_FL);
	E(0xe1822222u);   // orr r2, r2, r2, lsr #4
	E(0xe1822122u);   // orr r2, r2, r2, lsr #2
	E(0xe18220a2u);   // orr r2, r2, r2, lsr #1
	e_imm(3, 0x01010101u);
	E(0xe0022003u);   // and r2, r2, r3
	e_str(2, V_FL);
}

// the flags word (CY OV S Z bytes) in r2, tested for the branch condition c (as the low 4 bits of opcodes 0x60-0x6f):
// the host condition under which the branch is taken (MAME's flags are tested for != 0, normalized first by the
// handlers of LT GE LE GT)
static uint32_t e_cond(int c)
{
	if (c >= 0xc && !s_fnorm) {
		e_normflags();
		s_fnorm = 1;
	}
	e_ldr(2, V_FL);
	switch (c) {
	case 0x0: case 0x1: E(0xe3120cffu); break;   // tst r2, #0xff00 (OV)
	case 0x2: case 0x3: E(0xe31200ffu); break;   // tst r2, #0xff (CY)
	case 0x4: case 0x5: E(0xe31204ffu); break;   // tst r2, #0xff000000 (Z)
	case 0x6: case 0x7:
		E(0xe1823c22u);   // orr r3, r2, r2, lsr #24 (CY | Z)
		E(0xe31300ffu);   // tst r3, #0xff
		break;
	case 0x8: case 0x9: E(0xe31208ffu); break;   // tst r2, #0xff0000 (S)
	case 0xc: case 0xd:
		E(0xe0223422u);   // eor r3, r2, r2, lsr #8 (byte 1: S ^ OV)
		E(0xe3130cffu);   // tst r3, #0xff00
		break;
	case 0xe: case 0xf:
		E(0xe0223422u);   // eor r3, r2, r2, lsr #8
		E(0xe1833822u);   // orr r3, r3, r2, lsr #16 (byte 1: (S ^ OV) | Z)
		E(0xe3130cffu);
		break;
	}
	return (c & 1) ? 0x0u : 0x1u;   // (eq / ne)
}

// a taken conditional transfer from 'from' to 'to' (cond: the host condition)
static void e_bcc_exit(uint32_t cond, uint32_t from, uint32_t to)
{
	e_tocold(cond);
	e_cold();
	e_exit_to(from, to);
	e_hot_nob();
}

// a format 1 / 2 read-modify-write op (F12DecodeOperands(ReadAM, d1, ReadAMAddress, d2), F12LOADOP2, F12STOREOP2):
// r_dec decodes, r_load emits the operands (the first's value -> r8 unless it is an immediate and want1 is 0, the
// second's address -> r9, its value -> r0), r_store writes r0 back
static vf12 s_rf;
static int s_rd2;
static int r_dec(uint32_t at, int d1, int d2)
{
	if (!vj_f12(at, 1, 1, d1, 2, d2, &s_rf))
		return 0;
	if (s_rf.a1.k == K_IMM && d1 < 2 && s_rf.a1.c >> (8u << d1))
		return 0;
	s_rd2 = d2;
	return 1;
}
static void r_load(int d1, int want1)
{
	if (want1 || s_rf.a1.k != K_IMM)
		e_value(&s_rf.a1, d1, 8);
	if (s_rf.a2.k == K_REG)
		e_ldreg(0, s_rf.a2.breg, s_rd2);
	else {
		e_ea(&s_rf.a2);
		e_mov(9, 0);
		e_read(s_rd2);
	}
}
static void r_store(void)
{
	if (s_rf.a2.k == K_REG)
		e_streg(0, s_rf.a2.breg, s_rd2);
	else {
		e_mov(1, 0);
		e_mov(0, 9);
		e_write(s_rd2);
	}
}
// the flags word from CY in r1 (0 / 1), OV in r3 (0 / 1; ov0: OV = 0) and S, Z of the result in r0 (bits wide,
// zero-extended) (r2, r12 clobbered)
static void e_flagword(int ov0, unsigned bits)
{
	if (ov0)
		e_mov(2, 1);
	else
		e_dpr(12, 0, 2, 1, 3, LSL(8));   // orr r2, r1, r3, lsl #8
	E(ubfx(12, 0, (int)bits - 1, 1));
	e_dpr(12, 0, 2, 2, 12, LSL(16));      // orr r2, r2, r12, lsl #16
	E(0xe16fcf10u);                       // clz r12, r0
	E(0xe1a0c2acu);                       // lsr r12, r12, #5
	e_dpr(12, 0, 2, 2, 12, LSL(24));      // orr r2, r2, r12, lsl #24
	e_str(2, V_FL);
}

// SHL / SHA with a constant count (op1 an immediate; MAME's opSHL* / opSHA*), counts the C code has defined: within
// the width
// the other read-modify-write ops: the operands native, the operation the handler's C code (v60_jitsh.h)
#include "v60_jitsh.h"
static int vj_rmw(uint32_t at, uint8_t op)
{
	if (!s_vj_rmw[op].fn || !r_dec(at, s_vj_rmw[op].d1, s_vj_rmw[op].d2))
		return -1;
	r_load(s_vj_rmw[op].d1, 1);
	e_mov(1, 8);
	e_call((const void *)s_vj_rmw[op].fn);
	r_store();
	return s_rf.len;
}

static int vj_shift(uint32_t at, int d2, int arith)
{
	if (!r_dec(at, 0, d2))
		return -1;
	const int count = s_rf.a1.k == K_IMM ? (int8_t)(s_rf.a1.c & 0xff) : 0;
	const unsigned bits = 8u << d2;
	if (s_rf.a1.k != K_IMM || count >= (int)bits || -count >= (int)bits)
		return vj_rmw(at, vrb(at));   // (a count from a register or memory)
	r_load(0, 0);
	if (count == 0) {   // (CY = OV = 0, S Z of the value, the value written back)
		E(0xe3a01000u);   // mov r1, #0
		e_flagword(1, bits);
	} else if (count > 0) {
		E(ubfx(1, 0, (int)bits - count, 1));   // CY: the last bit out
		if (arith) {   // OV: the bits shifted out (and the sign) not all the sign
			const uint32_t mask = (uint32_t)(((1ull << count) - 1) << (bits - (unsigned)count));
			e_imm(12, mask);
			E(0xe000200cu);                 // and r2, r0, r12
			E(0xe1a03000u | LSL(32 - bits)); // lsl r3, r0, #(32 - bits)
			E(0xe1a03fc3u);                 // asr r3, r3, #31
			E(0xe003300cu);                 // and r3, r3, r12
			E(0xe1520003u);                 // cmp r2, r3
			E(0x13a03001u);                 // movne r3, #1
			E(0x03a03000u);                 // moveq r3, #0
		}
		E(0xe1a00000u | LSL(count));      // lsl r0, r0, #count
		if (bits < 32)
			E(ubfx(0, 0, 0, (int)bits));
		e_flagword(!arith, bits);
	} else {
		const int n = -count;
		E(ubfx(1, 0, n - 1, 1));          // CY: the last bit out
		if (arith) {
			if (bits < 32)
				E((bits == 8 ? 0xe6af0070u : 0xe6bf0070u));   // sxtb / sxth r0, r0
			E(0xe1a00000u | ASR(n));      // asr r0, r0, #n
			if (bits < 32)
				E(ubfx(0, 0, 0, (int)bits));
		} else
			E(0xe1a00000u | LSR(n));      // lsr r0, r0, #n
		e_flagword(1, bits);
	}
	r_store();
	return s_rf.len;
}

// ROT with a constant count (MAME's opROT*: count times by one bit)
static int vj_rot(uint32_t at, int d2)
{
	if (!r_dec(at, 0, d2) || s_rf.a1.k != K_IMM)
		return -1;
	const int count = (int8_t)(s_rf.a1.c & 0xff);
	const unsigned bits = 8u << d2;
	if (count >= (int)bits || -count >= (int)bits)
		return -1;
	r_load(0, 0);
	const unsigned l = count >= 0 ? (unsigned)count : bits - (unsigned)-count;   // (as a left rotation)
	if (l) {
		if (bits == 32)
			E(0xe1a00060u | (32u - l) << 7);   // ror r0, r0, #(32 - l)
		else {
			E(0xe1a01000u | LSL(l));          // lsl r1, r0, #l
			E(0xe1810000u | LSR(bits - l));   // orr r0, r1, r0, lsr #(bits - l)
			E(ubfx(0, 0, 0, (int)bits));
		}
	}
	if (count > 0)
		E(ubfx(1, 0, 0, 1));                 // CY: bit 0
	else if (count < 0)
		E(ubfx(1, 0, (int)bits - 1, 1));     // CY: the top bit
	else
		E(0xe3a01000u);
	e_flagword(1, bits);
	r_store();
	return s_rf.len;
}

// MULW / MULUW (CY kept, OV: the high word not 0)
static int vj_mul(uint32_t at, int sign)
{
	if (!r_dec(at, 2, 2))
		return -1;
	r_load(2, 1);
	E((sign ? 0xe0c00090u : 0xe0800090u) | 1u << 16 | 0u << 12 | 8u << 8 | 0u);   // s/umull r0, r1, r0, r8
	E(0xe3510000u);   // cmp r1, #0
	E(0x13a03001u);   // movne r3, #1
	E(0x03a03000u);   // moveq r3, #0
	E(0xe5d41000u | V_FL);   // ldrb r1, [r4, #CY]
	e_flagword(0, 32);
	r_store();
	return s_rf.len;
}

// ADDCW / SUBCW: with _CY ? 1 : 0
static int vj_addcw(uint32_t at, int sub)
{
	if (!r_dec(at, 2, 2))
		return -1;
	r_load(2, 1);
	E(0xe5d43000u | V_FL);   // ldrb r3, [r4, #CY]
	if (sub)
		E(0xe2733000u);      // rsbs r3, r3, #0 (C = CY == 0)
	else
		E(0xe2533001u);      // subs r3, r3, #1 (C = CY != 0)
	e_dpr(sub ? 6 : 5, 1, 0, 0, 8, 0);   // sbcs / adcs r0, r0, r8
	e_flags(sub ? 1 : 0);
	r_store();
	return s_rf.len;
}

// TEST1 / SET1 / CLR1 with a constant bit (0-31): CY = the bit, Z = !CY
static int vj_bit(uint32_t at, int kind)
{
	vf12 f;
	if (kind == 0) {
		if (!vj_f12(at, 1, 1, 2, 1, 2, &f) || f.a1.k != K_IMM || f.a1.c > 31 || f.first == 2)
			return -1;
		e_value(&f.a2, 2, 0);
	} else {
		if (!r_dec(at, 2, 2) || s_rf.a1.k != K_IMM || s_rf.a1.c > 31)
			return -1;
		f = s_rf;
		r_load(2, 0);
	}
	const unsigned bit = f.a1.c;
	E(ubfx(1, 0, (int)bit, 1));
	E(0xe5c41000u | V_FL);         // strb r1, [r4, #CY]
	E(0xe2212001u);                // eor r2, r1, #1
	E(0xe5c42000u | (V_FL + 3));   // strb r2, [r4, #Z]
	if (kind == 0)
		return f.len;
	e_dpi(kind == 1 ? 12 : 14, 0, 0, 0, 1u << bit);   // orr / bic r0, r0, #bit
	r_store();
	return s_rf.len;
}

// NOT / NEG (F12DecodeFirstOperand(ReadAM, d), F12WriteSecondOperand(d))
static int vj_notneg(uint32_t at, int d, int neg)
{
	vf12 f;
	if (!vj_f12(at, 2, 1, d, 3, d, &f))
		return -1;
	if (f.a1.k == K_IMM && d < 2 && f.a1.c >> (8u << d))
		return -1;
	e_value(&f.a1, d, 8);
	const uint32_t sh = vsh(d);
	if (neg) {
		E(0xe1a00008u | LSL(sh));   // lsl r0, r8, #sh
		E(0xe2700000u);             // rsbs r0, r0, #0
		e_flags(1);
		E(0xe3c220ffu);             // bic r2, r2, #0xff
		E(0xe3500000u);             // cmp r0, #0
		E(0x13822001u);             // orrne r2, r2, #1 (CY = the result not 0)
		e_str(2, V_FL);
	} else {
		E(0xe1e00008u);             // mvn r0, r8
		E(0xe1b00000u | LSL(sh));   // movs r0, r0, lsl #sh
		e_flags(2);
	}
	if (sh)
		E(0xe1a08020u | LSR(sh) | 0u);   // lsr r8, r0, #sh
	else
		e_mov(8, 0);
	e_wvalue(&f.a2, d, 8);
	return f.len;
}

// format 1 / 2 ALU ops: op2 = op2 OP op1 (kind 0 add, 1 sub, 2 cmp, 3 and, 4 or, 5 xor)
static int vj_alu(uint32_t at, int dim, int kind)
{
	vf12 f;
	const int cmp = kind == 2;
	if (!vj_f12(at, 1, 1, dim, cmp ? 1 : 2, dim, &f))
		return -1;
	if (f.a1.k == K_IMM && dim < 2 && f.a1.c >> (8u << dim))
		return -1;   // (not possible: an immediate of the dim)
	if (f.first == 2) {   // (cmp: the second operand's register read first)
		e_value(&f.a2, dim, 9);
		e_value(&f.a1, dim, 8);
	} else {
		e_value(&f.a1, dim, 8);
		if (cmp)
			e_value(&f.a2, dim, 9);
		else if (f.a2.k == K_MEM) {
			e_ea(&f.a2);
			e_mov(9, 0);
		}
	}
	// the second operand's value -> r0
	if (cmp)
		e_mov(0, 9);
	else if (f.a2.k == K_REG)
		e_ldreg(0, f.a2.breg, dim);
	else {
		e_mov(0, 9);
		e_read(dim);
	}
	const uint32_t sh = vsh(dim);
	if (sh) {
		E(0xe1a00000u | LSL(sh));   // lsl r0, r0, #sh
		E(0xe1a01008u | LSL(sh));   // lsl r1, r8, #sh
	} else
		e_mov(1, 8);
	static const uint32_t opc[6] = { 4, 2, 2, 0, 12, 1 };   // adds, subs, subs, ands, orrs, eors
	e_dpr(opc[kind], 1, 0, 0, 1, 0);
	if (kind <= 2) {
		e_flags(kind ? 1 : 0);
		s_fnorm = 1;
	} else
		e_flags(2);
	if (cmp)
		return f.len;
	if (sh)
		E(0xe1a00000u | LSR(sh));   // lsr r0, r0, #sh
	if (f.a2.k == K_REG)
		e_streg(0, f.a2.breg, dim);
	else {
		e_mov(1, 0);
		e_mov(0, 9);
		e_write(dim);
	}
	return f.len;
}

// MOV / MOVS / MOVZ / MOVEA (F12DecodeFirstOperand, F12WriteSecondOperand): ext 0 none, 1 sign-extended, 2 the address
static int vj_mov(uint32_t at, int d1, int d2, int ext)
{
	vf12 f;
	if (!vj_f12(at, 2, ext == 2 ? 2 : 1, d1, 3, d2, &f))
		return -1;
	if (f.a1.k == K_IMM && d1 < 2 && f.a1.c >> (8u << d1))
		return -1;
	if (ext == 2) {
		if (f.a1.k != K_MEM)
			return -1;   // (MOVEA of a register: MAME writes the register's number)
		e_ea(&f.a1);
		e_mov(8, 0);
	} else {
		e_value(&f.a1, d1, 8);
		if (ext == 1)
			E((d1 == 0 ? 0xe6af0070u : 0xe6bf0070u) | 8u << 12 | 8u);   // sxtb / sxth r8, r8
	}
	e_wvalue(&f.a2, d2, 8);
	return f.len;
}

// TESTB / TESTH / TESTW
static int vj_test(uint32_t at, uint8_t op, int dim)
{
	vam a;
	const int l = vj_am(at + 1, op & 1, dim, 1, at, &a);
	if (!l || (a.k == K_IMM && dim < 2 && a.c >> (8u << dim)))
		return -1;
	e_value(&a, dim, 0);
	if (vsh(dim))
		E(0xe1a00000u | LSL(vsh(dim)));
	e_flags_test(0);
	s_fnorm = 1;
	return l + 1;
}

// INC / DEC
static int vj_incdec(uint32_t at, uint8_t op)
{
	const int dim = (op >> 1) & 3, inc = op & 8;
	vam a;
	const int l = vj_am(at + 1, op & 1, dim, 2, at, &a);
	if (!l || a.k == K_IMM)
		return -1;
	if (a.k == K_REG)
		e_ldreg(0, a.breg, dim);
	else {
		e_ea(&a);
		e_mov(9, 0);
		e_read(dim);
	}
	const uint32_t sh = vsh(dim);
	if (sh)
		E(0xe1a00000u | LSL(sh));
	e_imm(1, 1u << sh);
	e_dpr(inc ? 4 : 2, 1, 0, 0, 1, 0);   // adds / subs r0, r0, r1
	e_flags(inc ? 0 : 1);
	s_fnorm = 1;
	if (sh)
		E(0xe1a00000u | LSR(sh));
	if (a.k == K_REG)
		e_streg(0, a.breg, dim);
	else {
		e_mov(1, 0);
		e_mov(0, 9);
		e_write(dim);
	}
	return l + 1;
}

// the native translation of the instruction at 'at': its length, or -1 (its handler); *open = 0 when the block ends
static int vj_native1(uint32_t at, uint8_t op, int *open)
{
	switch (op) {
	// ---- moves ----
	case 0x09: return vj_mov(at, 0, 0, 0);   // MOVB
	case 0x1b: return vj_mov(at, 1, 1, 0);   // MOVH
	case 0x2d: return vj_mov(at, 2, 2, 0);   // MOVW
	case 0x0a: return vj_mov(at, 0, 1, 1);   // MOVSBH
	case 0x0b: return vj_mov(at, 0, 1, 0);   // MOVZBH
	case 0x0c: return vj_mov(at, 0, 2, 1);   // MOVSBW
	case 0x0d: return vj_mov(at, 0, 2, 0);   // MOVZBW
	case 0x1c: return vj_mov(at, 1, 2, 1);   // MOVSHW
	case 0x1d: return vj_mov(at, 1, 2, 0);   // MOVZHW
	case 0x40: return vj_mov(at, 0, 2, 2);   // MOVEAB
	case 0x42: return vj_mov(at, 1, 2, 2);   // MOVEAH
	case 0x44: return vj_mov(at, 2, 2, 2);   // MOVEAW
	// ---- ALU ----
	case 0x80: return vj_alu(at, 0, 0);   // ADDB
	case 0x82: return vj_alu(at, 1, 0);
	case 0x84: return vj_alu(at, 2, 0);
	case 0xa8: return vj_alu(at, 0, 1);   // SUBB
	case 0xaa: return vj_alu(at, 1, 1);
	case 0xac: return vj_alu(at, 2, 1);
	case 0xb8: return vj_alu(at, 0, 2);   // CMPB
	case 0xba: return vj_alu(at, 1, 2);
	case 0xbc: return vj_alu(at, 2, 2);
	case 0xa0: return vj_alu(at, 0, 3);   // ANDB
	case 0xa2: return vj_alu(at, 1, 3);
	case 0xa4: return vj_alu(at, 2, 3);
	case 0x88: return vj_alu(at, 0, 4);   // ORB
	case 0x8a: return vj_alu(at, 1, 4);
	case 0x8c: return vj_alu(at, 2, 4);
	case 0xb0: return vj_alu(at, 0, 5);   // XORB
	case 0xb2: return vj_alu(at, 1, 5);
	case 0xb4: return vj_alu(at, 2, 5);
	case 0xa9: return vj_shift(at, 0, 0);   // SHLB
	case 0xab: return vj_shift(at, 1, 0);
	case 0xad: return vj_shift(at, 2, 0);
	case 0xb9: return vj_shift(at, 0, 1);   // SHAB
	case 0xbb: return vj_shift(at, 1, 1);
	case 0xbd: return vj_shift(at, 2, 1);
	case 0x89: return vj_rot(at, 0);        // ROTB
	case 0x8b: return vj_rot(at, 1);
	case 0x8d: return vj_rot(at, 2);
	case 0x85: return vj_mul(at, 1);        // MULW
	case 0x95: return vj_mul(at, 0);        // MULUW
	case 0x94: return vj_addcw(at, 0);      // ADDCW
	case 0x9c: return vj_addcw(at, 1);      // SUBCW
	case 0x87: return vj_bit(at, 0);        // TEST1
	case 0x97: return vj_bit(at, 1);        // SET1
	case 0xa7: return vj_bit(at, 2);        // CLR1
	case 0xcd: return 1;                    // NOP
	case 0x38: return vj_notneg(at, 0, 0);  // NOTB
	case 0x3a: return vj_notneg(at, 1, 0);
	case 0x3c: return vj_notneg(at, 2, 0);
	case 0x39: return vj_notneg(at, 0, 1);  // NEGB
	case 0x3b: return vj_notneg(at, 1, 1);
	case 0x3d: return vj_notneg(at, 2, 1);
	// ---- format 3 ----
	case 0xf0: case 0xf1: return vj_test(at, op, 0);
	case 0xf2: case 0xf3: return vj_test(at, op, 1);
	case 0xf4: case 0xf5: return vj_test(at, op, 2);
	case 0xd0: case 0xd1: case 0xd2: case 0xd3: case 0xd4: case 0xd5:
	case 0xd8: case 0xd9: case 0xda: case 0xdb: case 0xdc: case 0xdd:
		return vj_incdec(at, op);
	case 0xee: case 0xef: {   // PUSH
		vam a;
		const int l = vj_am(at + 1, op & 1, 2, 1, at, &a);
		if (!l)
			return -1;
		e_value(&a, 2, 8);
		e_sp_add(-4);
		e_mov(1, 8);
		e_write(2);
		return l + 1;
	}
	case 0xe6: case 0xe7: {   // POP
		vam a;
		const int l = vj_am(at + 1, op & 1, 2, 3, at, &a);
		if (!l)
			return -1;
		e_ldr(0, V_REG(31));
		e_read(2);
		e_mov(8, 0);
		e_sp_add(4);
		e_wvalue(&a, 2, 8);
		return l + 1;
	}
	// ---- transfers ----
	case 0x6a: *open = 0; e_exit_to(at, at + (uint32_t)(int8_t)vrb(at + 1)); return 2;    // BR8
	case 0x7a: *open = 0; e_exit_to(at, at + (uint32_t)(int16_t)vrw(at + 1)); return 3;   // BR16
	case 0x60: case 0x61: case 0x62: case 0x63: case 0x64: case 0x65: case 0x66: case 0x67:
	case 0x68: case 0x69: case 0x6c: case 0x6d: case 0x6e: case 0x6f:
		e_bcc_exit(e_cond(op & 15), at, at + (uint32_t)(int8_t)vrb(at + 1));
		return 2;
	case 0x70: case 0x71: case 0x72: case 0x73: case 0x74: case 0x75: case 0x76: case 0x77:
	case 0x78: case 0x79: case 0x7c: case 0x7d: case 0x7e: case 0x7f:
		e_bcc_exit(e_cond(op & 15), at, at + (uint32_t)(int16_t)vrw(at + 1));
		return 3;
	case 0xc6: case 0xc7: {   // DBcc / TB
		const uint8_t b = vrb(at + 1);
		const int idx = b >> 5, reg = b & 0x1f;
		const uint32_t to = at + (uint32_t)(int16_t)vrw(at + 2);
		if (op == 0xc7 && idx == 5) {   // TB: taken when the register is 0
			e_ldr(0, V_REG(reg));
			E(0xe3500000u);   // cmp r0, #0
			e_bcc_exit(0x0, at, to);
			return 4;
		}
		const int c = idx * 2 + (op & 1);   // (DBV DBL DBE DBNH DBN DBR DBLT DBLE / DBNV DBNL DBNE DBH DBP - DBGE DBGT)
		if (c >= 0xc && !s_fnorm) {
			e_normflags();
			s_fnorm = 1;
		}
		e_ldr(0, V_REG(reg));
		E(0xe2500001u);   // subs r0, r0, #1
		e_str(0, V_REG(reg));
		if (c == 0xa) {   // DBR
			e_bcc_exit(0x1, at, to);
			return 4;
		}
		const unsigned skip = e_bfwd(0x0);   // (the register 0: not taken)
		const uint32_t cond = e_cond(c);
		e_bcc_exit(cond, at, to);
		e_here(skip);
		return 4;
	}
	case 0x48: {   // BSR
		e_sp_add(-4);
		e_imm(1, at + 3);
		e_write(2);
		*open = 0;
		e_exit_to(at, at + (uint32_t)(int16_t)vrw(at + 1));
		return 3;
	}
	case 0xca:   // RSR
		e_ldr(0, V_REG(31));
		e_read(2);
		e_str(0, V_PC);
		e_sp_add(4);
		*open = 0;
		e_exit_dyn(at);
		return 1;
	case 0xd6: case 0xd7: {   // JMP
		vam a;
		const int l = vj_am(at + 1, op & 1, 0, 2, at, &a);
		if (!l || a.k != K_MEM)
			return -1;
		e_ea(&a);
		e_str(0, V_PC);
		*open = 0;
		e_exit_dyn(at);
		return l + 1;
	}
	case 0xe8: case 0xe9: {   // JSR
		vam a;
		const int l = vj_am(at + 1, op & 1, 0, 2, at, &a);
		if (!l || a.k != K_MEM)
			return -1;
		e_ea(&a);
		e_mov(9, 0);
		e_sp_add(-4);
		e_imm(1, at + (uint32_t)l + 1);
		e_write(2);
		e_str(9, V_PC);
		*open = 0;
		e_exit_dyn(at);
		return l + 1;
	}
	case 0xe2: case 0xe3: {   // RET
		vam a;
		const int l = vj_am(at + 1, op & 1, 2, 1, at, &a);
		if (!l)
			return -1;
		e_value(&a, 2, 8);
		e_ldr(0, V_REG(31));
		e_read(2);
		e_str(0, V_PC);
		e_sp_add(4);
		e_read(2);
		e_str(0, V_REG(29));   // AP
		e_ldr(0, V_REG(31));
		E(0xe2800004u);        // add r0, r0, #4
		e_dpr(4, 0, 0, 0, 8, 0);   // add r0, r0, r8
		e_str(0, V_REG(31));
		*open = 0;
		e_exit_dyn(at);
		return l + 1;
	}
	}
	return vj_rmw(at, op);
}
static int vj_native(uint32_t at, uint8_t op, int *open)
{
	const int len = vj_native1(at, op, open);
	return len >= 0 ? len : vj_rmw(at, op);
}
