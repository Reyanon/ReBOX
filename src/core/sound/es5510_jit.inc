/*  ES5510 program compiled to ARM code (included by es5510.c; ARM builds with executable memory from the frontend).
    This file is part of ReBOX. GPL-2.0-or-later.

    In the steady state each sample runs the program from pc 0 to its END, with that END in flight from the sample
    before; every step's work is then known from the decoded program. jit_build turns one such pass into ARM code:
    the RAM pipeline, the DIL loads, the delay line addresses, the operand reads and the delay line writes inline (with
    the registers, constants and offsets of each step as immediates; the RAM address of each step in its own slot, so
    nothing shifts), the multiplier result, the ALU add / sub / cmp / move, most special register reads and the delay
    line out inline too; the other ALU operations, special register writes, MACH / MACL reads and END as calls of C
    helpers specialized by kind and operation (fixed targets: predicted). The steps and results are those of exec_noskip (es5510_run_once runs that
    when the state is not a steady pass: other programs, the first pass). Skippable instructions: their skip state is
    computed by a helper and the next step's results check it (exec_noskip's pf). */

#include <stddef.h>
#if !defined(__vita__)
#include <stdio.h>
#endif

// the pass state: exec_noskip's locals (all 32-bit but MACHL, for the generated code's loads and stores)
typedef struct {
	int32_t cV, dV, aV, bV;
	int32_t dil, dol0, dol1, dol_count;
	int32_t ccr, mac_overflow;
	int32_t dbase, dlength, dmod, memmask, memincrement, memshift, mulshift;
	int32_t alu_result;
	int32_t pf, npf;   // the skip state of the instruction before / of this one (exec_noskip pf, npf)
	es5510_t *e;
	int64_t machl, mac_product, mac_result;
	uint32_t addr[0xa0];   // the RAM address each step computed
} jit_state;
// (its own cache lines: the worker writes it all the time, and false sharing with the emulation thread's data costs
// that thread dearly)
static union { jit_state s; uint8_t pad[(sizeof(jit_state) + 63) & ~63u]; } s_jit_state __attribute__((aligned(64)));
#define J (s_jit_state.s)

// The code is made by the thread that runs the ESP (it decodes the program) in ordinary memory and put into the
// executable memory by the emulation thread at a frame start (es5510_jit_install): on the Vita only the thread that
// opened the VM domain may write there, and the worker threads are made again for each game (ReST-V ADR-110).
// The code memory is only ever added to (ReST-V ADR-110: code rewritten in place can run stale from the other core's
// instruction cache); when it is used up the ESP interprets.
#define JIT_SIZE 0x10000
#define JIT_VM_SIZE 0x400000   // (a part of the frontend's block: rbx_jit_alloc)
static uint32_t s_jit_stage[JIT_SIZE / 4];
static unsigned s_jit_stage_len, s_jit_stage_ver;
enum { JS_NONE, JS_BUILDING, JS_READY, JS_COPYING };
static int s_jit_stage_state;
static uint32_t *s_jit_vm;                     // JIT_VM_SIZE executable (NULL: none yet / none)
static unsigned s_jit_vm_used;                 // bytes
static int s_jit_vm_none;
static uint32_t *volatile s_jit_live;          // the installed code
static volatile unsigned s_jit_live_ver;       // the build it is
static unsigned s_jit_ver;                     // the build for the decoded program (0 none)
// s_jit_ok (es5510.c): 1 compiled for the decoded program, 0 not yet, -1 not as a pass
static int s_jit_end;

// ---- helpers (the generated code calls these; they work on J) ----
static void jit_spill(void)
{
	es5510_t *e = J.e;
	e->dil = (int16_t)J.dil; e->dol[0] = (int16_t)J.dol0; e->dol[1] = (int16_t)J.dol1; e->dol_count = J.dol_count;
	e->ccr = (int8_t)J.ccr; e->machl = J.machl; e->mac_overflow = (uint8_t)J.mac_overflow; e->dbase = J.dbase;
	e->dlength = J.dlength;
}

static void jit_fill(void)
{
	es5510_t *e = J.e;
	J.ccr = e->ccr; J.machl = e->machl; J.mac_overflow = e->mac_overflow; J.dbase = e->dbase; J.dlength = e->dlength;
	J.memmask = e->memmask; J.memincrement = e->memincrement; J.memshift = e->memshift; J.mulshift = e->mulshift;
	J.dmod = J.dlength + J.memincrement;
}

static int32_t jit_rreg(uint32_t r)
{
	return read_spr_fast(J.e, (uint8_t)r, (int16_t)J.dil, (int8_t)J.ccr, J.machl, (uint8_t)J.mac_overflow, J.dbase, J.dlength);
}

static void jit_wspr(uint32_t r, int32_t v)
{
	v &= 0x00ffffff;
	switch (r) {
	case 234: case 235: case 236: case 237: case 238: case 239: case 240: case 241: write_ser(J.e, (uint8_t)r, v); break;
	case 242: J.machl = sext48((J.machl & ((int64_t)0x00ffffff << 24)) | v); break;
	case 243: J.machl = sext48((J.machl & 0x00ffffff) | ((int64_t)v << 24)); J.mac_overflow = 0; break;
	case 245: J.dlength = v; J.dmod = J.dlength + J.memincrement; break;
	case 246: J.e->abase = v; break;
	case 247: J.e->bbase = v; break;
	case 248: J.dbase = v; break;
	case 250: J.ccr = (int8_t)((v >> 16) & FLAG_MASK); break;
	case 251: J.e->cmr = (int8_t)((v >> 16) & (FLAG_MASK | FLAG_NOT)); break;
	default: jit_spill(); write_reg(J.e, (uint8_t)r, v); jit_fill(); break;
	}
}

static void jit_dol(int32_t v)
{
	const int16_t d = (int16_t)(v >> 8);
	if (J.dol_count >= 2) {
		J.dol0 = J.dol1;
		J.dol1 = d;
	} else if (J.dol_count++ == 0)
		J.dol0 = d;
	else
		J.dol1 = d;
}

static int32_t jit_delay_mod(int32_t x, int32_t m) { return m ? x % m : 0; }

// multiplier result (of the operands read the step before): the saturated 24-bit value
static inline int32_t jit_mac(int acc)
{
	J.mac_product = ((int64_t)sext24(J.cV) * (int64_t)sext24(J.dV)) * ((int64_t)1 << J.mulshift);
	J.mac_result = acc ? J.mac_product + J.machl : J.mac_product;
	const int32_t hi = (int32_t)((uint64_t)J.mac_result >> 32);
	J.mac_overflow = (int32_t)((uint32_t)hi << 16) >> 16 != hi;
	J.machl = J.mac_result;
	return J.mac_overflow ? (hi < 0 ? 0x00800000 : 0x007fffff) :
		(int32_t)((((uint32_t)hi << 8) | ((uint32_t)J.mac_result >> 24)) & 0x00ffffff);
}
// arg: the result register (G), the write kind << 8 | register (X)
#define JIT_MAC(ACC, SUF) \
	static void jit_mac_g##SUF(uint32_t r) { J.e->gpr[r] = jit_mac(ACC) & 0x00ffffff; } \
	static void jit_mac_gd##SUF(uint32_t r) { const int32_t t = jit_mac(ACC); J.e->gpr[r] = t & 0x00ffffff; jit_dol(t); } \
	static void jit_mac_d##SUF(uint32_t r) { (void)r; jit_dol(jit_mac(ACC)); } \
	static void jit_mac_n##SUF(uint32_t r) { (void)r; (void)jit_mac(ACC); } \
	static void jit_mac_x##SUF(uint32_t a) { const int32_t t = jit_mac(ACC); const uint32_t k = a >> 8, r = a & 0xff; \
		if (k & WK_GPR) J.e->gpr[r] = t & 0x00ffffff; else if (k & WK_SPR) jit_wspr(r, t); if (k & WK_DOL) jit_dol(t); }
JIT_MAC(0, 0)
JIT_MAC(1, 1)
#undef JIT_MAC

#define JIT_ALU(OP) \
	static void jit_alu_g##OP(uint32_t r) { uint8_t f = (uint8_t)J.ccr; const int32_t v = alu_operation(OP, J.aV, J.bV, &f); \
		J.e->gpr[r] = v & 0x00ffffff; J.alu_result = v; J.ccr = (int8_t)f; } \
	static void jit_alu_x##OP(uint32_t a) { uint8_t f = (uint8_t)J.ccr; const int32_t v = alu_operation(OP, J.aV, J.bV, &f); \
		const uint32_t k = a >> 8, r = a & 0xff; \
		if (k & WK_GPR) J.e->gpr[r] = v & 0x00ffffff; else if (k & WK_SPR) jit_wspr(r, v); if (k & WK_DOL) jit_dol(v); \
		J.alu_result = v; J.ccr = (int8_t)f; }
JIT_ALU(0) JIT_ALU(1) JIT_ALU(2) JIT_ALU(3) JIT_ALU(4) JIT_ALU(5) JIT_ALU(6) JIT_ALU(7)
JIT_ALU(8) JIT_ALU(9) JIT_ALU(10) JIT_ALU(11) JIT_ALU(12) JIT_ALU(13) JIT_ALU(14) JIT_ALU(15)
#undef JIT_ALU
static void (*const JIT_ALU_G[16])(uint32_t) = { jit_alu_g0, jit_alu_g1, jit_alu_g2, jit_alu_g3, jit_alu_g4, jit_alu_g5,
	jit_alu_g6, jit_alu_g7, jit_alu_g8, jit_alu_g9, jit_alu_g10, jit_alu_g11, jit_alu_g12, jit_alu_g13, jit_alu_g14, jit_alu_g15 };
static void (*const JIT_ALU_X[16])(uint32_t) = { jit_alu_x0, jit_alu_x1, jit_alu_x2, jit_alu_x3, jit_alu_x4, jit_alu_x5,
	jit_alu_x6, jit_alu_x7, jit_alu_x8, jit_alu_x9, jit_alu_x10, jit_alu_x11, jit_alu_x12, jit_alu_x13, jit_alu_x14, jit_alu_x15 };

enum { JPF_MSKIP = 1, JPF_ASKIP = 2, JPF_NOCCR = 4 };
// skip state of a skippable instruction (on ccr before the ALU result of the one before, as exec_noskip)
static void jit_skip(uint32_t is_cmp)
{
	const int8_t cmr = J.e->cmr;
	int sat = (J.ccr & cmr & FLAG_MASK) != 0;
	if (cmr & FLAG_NOT)
		sat = !sat;
	J.npf = !is_cmp ? (sat ? JPF_MSKIP | JPF_ASKIP | JPF_NOCCR : JPF_NOCCR) : (sat ? JPF_MSKIP : 0);
}

// ALU result of a skippable instruction: arg = op | write kind << 8 | register << 16
static void jit_alu_s(uint32_t a)
{
	if (J.pf & JPF_ASKIP)
		return;
	uint8_t f = (uint8_t)J.ccr;
	const int32_t v = alu_operation((uint8_t)(a & 15), J.aV, J.bV, &f);
	const uint32_t k = (a >> 8) & 0xff, r = a >> 16;
	if (k & WK_GPR)
		J.e->gpr[r] = v & 0x00ffffff;
	else if (k & WK_SPR)
		jit_wspr(r, v);
	if (k & WK_DOL)
		jit_dol(v);
	J.alu_result = v;
	if (!(J.pf & JPF_NOCCR))
		J.ccr = (int8_t)f;
}

static void jit_end(uint32_t unused)
{
	(void)unused;
	// (halt asserted: the pass ends here, as exec_noskip's END)
	J.e->state = ST_HALTED;
	J.e->host_control &= ~0x04;
	J.dbase -= J.memincrement;
	if (J.dbase < 0)
		J.dbase = J.dlength;
}

// ---- ARM code ----
// registers: r11 = &J, r10 = gpr, r9 = dram; r0-r3, r12 scratch
enum { R0, R1, R2, R3, R4, R5, R6, R7, R8, R9, R10, R11, R12, SP = 13, LR = 14, PC = 15 };
#define JOFF(f) ((uint32_t)offsetof(jit_state, f))
static uint32_t *s_jp, *s_jlim;
static int s_jfail;

static void jw(uint32_t op)
{
	if (s_jp < s_jlim)
		*s_jp++ = op;
	else
		s_jfail = 1;
}
static void j_ldr(int rt, int rn, uint32_t off) { jw(0xe5900000u | rn << 16 | rt << 12 | (off & 0xfff)); if (off > 0xfff) s_jfail = 1; }
static void j_str(int rt, int rn, uint32_t off) { jw(0xe5800000u | rn << 16 | rt << 12 | (off & 0xfff)); if (off > 0xfff) s_jfail = 1; }
static void j_movw(int rd, uint32_t v) { jw(0xe3000000u | (v >> 12 & 0xf) << 16 | rd << 12 | (v & 0xfff)); }
static void j_movt(int rd, uint32_t v) { jw(0xe3400000u | (v >> 12 & 0xf) << 16 | rd << 12 | (v & 0xfff)); }
static void j_mov32(int rd, uint32_t v) { j_movw(rd, v & 0xffff); if (v >> 16) j_movt(rd, v >> 16); }
static void j_call(void (*f)(uint32_t), uint32_t arg)
{
	j_mov32(R0, arg);
	j_mov32(R12, (uint32_t)(uintptr_t)f);
	jw(0xe12fff3cu);   // blx r12
}

// r0 = the DRAM word at the slot's address (ram_pp: the step two before), into J.dil
static void j_dil_load(int slot)
{
	j_ldr(R0, R11, JOFF(addr) + (uint32_t)slot * 4);
	jw(0xe7f30050u | R0 << 12 | R0);        // ubfx r0, r0, #0, #20
	jw(0xe0800080u | R9 << 16 | R0 << 12 | R0);   // add r0, r9, r0, lsl #1
	jw(0xe1d000f0u | R0 << 16 | R0 << 12);  // ldrsh r0, [r0]
	j_str(R0, R11, JOFF(dil));
}

// the delay line address of step pc (exec_noskip RA_DELAY) into its slot
static void j_delay_addr(int pc)
{
	j_ldr(R0, R10, (uint32_t)pc * 4);
	j_ldr(R1, R11, JOFF(dbase));
	jw(0xe0800001u);                        // add r0, r0, r1
	j_ldr(R2, R11, JOFF(dmod));
	jw(0xe3500000u);                        // cmp r0, #0
	uint32_t *b_slow1 = s_jp; jw(0);        // blt slow
	jw(0xe1500002u);                        // cmp r0, r2
	uint32_t *b_ok1 = s_jp; jw(0);          // blt ok
	jw(0xe0403002u);                        // sub r3, r0, r2
	jw(0xe1530002u);                        // cmp r3, r2
	jw(0xb1a00003u);                        // movlt r0, r3
	uint32_t *b_ok2 = s_jp; jw(0);          // blt ok
	uint32_t *slow = s_jp;
	jw(0xe1a01002u);                        // mov r1, r2
	j_mov32(R12, (uint32_t)(uintptr_t)jit_delay_mod);
	jw(0xe12fff3cu);                        // blx r12
	uint32_t *ok = s_jp;
	j_ldr(R1, R11, JOFF(memmask));
	jw(0xe0000001u);                        // and r0, r0, r1
	j_ldr(R1, R11, JOFF(memshift));
	jw(0xe1a00150u);                        // asr r0, r0, r1
	j_str(R0, R11, JOFF(addr) + (uint32_t)pc * 4);
	if (s_jfail)
		return;
#define JB(at, to, cond) (*(at) = (uint32_t)(cond) << 28 | 0x0a000000u | ((uint32_t)(((to) - (at)) - 2) & 0x00ffffff))
	JB(b_slow1, slow, 0xb);
	JB(b_ok1, ok, 0xb);
	JB(b_ok2, ok, 0xb);
#undef JB
}

static es5510_t *s_je;   // the chip the code is built for

// r0 = operand register r: a general register, a constant, or a special register (as read_spr_fast; MACH / MACL by
// the helper)
static int j_operand(uint8_t r)
{
	int16_t *const ser[8] = { &s_je->ser0r, &s_je->ser0l, &s_je->ser1r, &s_je->ser1l, &s_je->ser2r, &s_je->ser2l,
		&s_je->ser3r, &s_je->ser3l };
	if (r < 0xc0)
		j_ldr(R0, R10, (uint32_t)r * 4);
	else if (r >= 252)
		j_mov32(R0, (uint32_t)SPR_CONST[r - 252]);
	else if (r >= 234 && r <= 241) {
		j_mov32(R0, (uint32_t)(uintptr_t)ser[r - 234]);
		jw(0xe1d000f0u);                    // ldrsh r0, [r0]
		jw(0xe1a00400u);                    // lsl r0, r0, #8
	} else if (r == 244) {
		j_ldr(R0, R11, JOFF(dil));
		jw(0xe1a00400u);                    // lsl r0, r0, #8
	} else if (r == 245)
		j_ldr(R0, R11, JOFF(dlength));
	else if (r == 248)
		j_ldr(R0, R11, JOFF(dbase));
	else if (r == 246 || r == 247 || r == 249) {
		j_mov32(R0, (uint32_t)(uintptr_t)(r == 246 ? &s_je->abase : r == 247 ? &s_je->bbase : &s_je->sigreg));
		j_ldr(R0, R0, 0);
	} else if (r == 250) {
		j_ldr(R0, R11, JOFF(ccr));
		jw(0xe1a00800u);                    // lsl r0, r0, #16
	} else if (r == 251) {
		j_mov32(R0, (uint32_t)(uintptr_t)&s_je->cmr);
		jw(0xe1d000d0u);                    // ldrsb r0, [r0]
		jw(0xe1a00800u);                    // lsl r0, r0, #16
	} else {
		j_mov32(R0, r);
		j_mov32(R12, (uint32_t)(uintptr_t)jit_rreg);
		jw(0xe12fff3cu);                    // blx r12
	}
	return 0;
}
static void j_operand_dil(void)
{
	j_ldr(R0, R11, JOFF(dil));
	jw(0xe1a00400u);                        // lsl r0, r0, #8
}

// the delay line out shifts after a write / dump cycle (exec_noskip RAM cycle N-1)
static void j_dol_shift(void)
{
	j_ldr(R1, R11, JOFF(dol1));
	j_str(R1, R11, JOFF(dol0));
	j_ldr(R1, R11, JOFF(dol_count));
	jw(0xe3510000u);                        // cmp r1, #0
	jw(0xc2411001u);                        // subgt r1, r1, #1
	jw(0xc5801000u | R11 << 16 | (JOFF(dol_count) & 0xfff));   // strgt r1, [r11, #dol_count]
}

// the delay line out written inline from r0 (as jit_dol)
static void j_dol(void)
{
	const uint32_t c = JOFF(dol_count), d0 = JOFF(dol0), d1 = JOFF(dol1);
	jw(0xe7af0450u);                        // sbfx r0, r0, #8, #16
	j_ldr(R1, R11, c);
	jw(0xe3510001u);                        // cmp r1, #1
	jw(0xb5800000u | R11 << 16 | d0);       // strlt r0, [r11, #dol0]
	jw(0xb3a02001u);                        // movlt r2, #1
	jw(0xb5802000u | R11 << 16 | c);        // strlt r2, [r11, #dol_count]
	jw(0x05800000u | R11 << 16 | d1);       // streq r0, [r11, #dol1]
	jw(0x03a02002u);                        // moveq r2, #2
	jw(0x05802000u | R11 << 16 | c);        // streq r2, [r11, #dol_count]
	jw(0xc5902000u | R11 << 16 | d1);       // ldrgt r2, [r11, #dol1]
	jw(0xc5802000u | R11 << 16 | d0);       // strgt r2, [r11, #dol0]
	jw(0xc5800000u | R11 << 16 | d1);       // strgt r0, [r11, #dol1]
}

// the multiplier result inline (as jit_mac): product, MACHL, the overflow, and the saturated 24-bit value in r0
static void j_mac(int acc)
{
	j_ldr(R0, R11, JOFF(cV));
	j_ldr(R1, R11, JOFF(dV));
	jw(0xe7b70050u | R0 << 12 | R0);              // sbfx r0, r0, #0, #24
	jw(0xe7b70050u | R1 << 12 | R1);              // sbfx r1, r1, #0, #24
	jw(0xe0c00090u | R3 << 16 | R2 << 12 | R1 << 8 | R0);   // smull r2, r3, r0, r1
	j_ldr(R12, R11, JOFF(mulshift));
	jw(0xe2600000u | R12 << 16 | LR << 12 | 32);  // rsb lr, r12, #32
	jw(0xe1a00010u | R3 << 12 | R12 << 8 | R3);   // lsl r3, r3, r12
	jw(0xe1800030u | R3 << 16 | R3 << 12 | LR << 8 | R2);   // orr r3, r3, r2, lsr lr
	jw(0xe1a00010u | R2 << 12 | R12 << 8 | R2);   // lsl r2, r2, r12
#define STRD(rt, off) jw(0xe1c000f0u | R11 << 16 | (rt) << 12 | ((off) & 0xf0) << 4 | ((off) & 0xf))
#define LDRD(rt, off) jw(0xe1c000d0u | R11 << 16 | (rt) << 12 | ((off) & 0xf0) << 4 | ((off) & 0xf))
	STRD(R2, JOFF(mac_product));
	if (acc) {
		LDRD(R0, JOFF(machl));
		jw(0xe0922000u);                          // adds r2, r2, r0
		jw(0xe0a33001u);                          // adc r3, r3, r1
	}
	STRD(R2, JOFF(mac_result));
	STRD(R2, JOFF(machl));
#undef STRD
#undef LDRD
	jw(0xe6bf0073u);                              // sxth r0, r3
	jw(0xe1500003u);                              // cmp r0, r3
	jw(0x13a01001u);                              // movne r1, #1
	jw(0x03a01000u);                              // moveq r1, #0
	j_str(R1, R11, JOFF(mac_overflow));
	jw(0xe1a00403u);                              // lsl r0, r3, #8
	jw(0xe1800c22u);                              // orr r0, r0, r2, lsr #24
	jw(0xe7f70050u);                              // ubfx r0, r0, #0, #24
	jw(0xe3510000u);                              // cmp r1, #0
	jw(0x0a000003u);                              // beq over (no overflow)
	jw(0xe3530000u);                              // cmp r3, #0
	jw(0xb3a00880u);                              // movlt r0, #0x800000
	jw(0xa30f0fffu);                              // movwge r0, #0xffff
	jw(0xa340007fu);                              // movtge r0, #0x7f
}

// ARM data processing: register (shifted by an immediate) and immediate forms
enum { DP_AND = 0, DP_EOR = 1, DP_SUB = 2, DP_RSB = 3, DP_ADD = 4, DP_ORR = 12, DP_MOV = 13, DP_BIC = 14 };
static void j_dp(int opc, int s, int rd, int rn, int rm, int lsl)
{
	jw(0xe0000000u | (uint32_t)opc << 21 | (uint32_t)s << 20 | (uint32_t)rn << 16 | (uint32_t)rd << 12 | (uint32_t)lsl << 7 | (uint32_t)rm);
}
static void j_dpi(uint32_t cond, int opc, int rd, int rn, uint32_t imm12)
{
	jw(cond << 28 | 0x02000000u | (uint32_t)opc << 21 | (uint32_t)rn << 16 | (uint32_t)rd << 12 | imm12);
}
static void j_ubfx(int rd, int rn, int lsb, int width) { jw(0xe7e00050u | (uint32_t)(width - 1) << 16 | (uint32_t)rd << 12 | (uint32_t)lsb << 7 | (uint32_t)rn); }

// the ALU result inline (ops add, sub, their saturating forms, cmp, move: as alu_operation) from J.aV / J.bV: the
// value in r2, J.alu_result and the flags in J.ccr (r4-r7 work registers: the helpers keep them)
static int j_alu_inline(uint8_t op) { return op <= 4 || op == 9; }
static void j_alu(uint8_t op)
{
	j_ldr(R1, R11, JOFF(bV));
	if (op == 9) {
		j_dp(DP_MOV, 0, R2, 0, R1, 0);
		j_str(R2, R11, JOFF(alu_result));
		return;
	}
	j_ldr(R0, R11, JOFF(aV));
	if (op == 1 || op == 3 || op == 4) {   // negate(b)
		j_dpi(0xe, DP_RSB, R1, R1, 0);
		j_ubfx(R1, R1, 0, 24);
	}
	j_dp(DP_ADD, 0, R2, R0, R1, 0);
	j_dp(DP_EOR, 0, R3, R0, R2, 0);
	j_dp(DP_EOR, 0, R4, R0, R1, 0);
	j_dp(DP_BIC, 0, R3, R3, R4, 0);
	j_ubfx(R3, R3, 23, 1);                  // V
	j_ubfx(R4, R2, 23, 1);                  // N
	j_ubfx(R5, R2, 24, 1);                  // C
	j_dp(DP_EOR, 0, R6, R3, R4, 0);         // LT
	j_dp(DP_MOV, 0, R7, 0, R4, 7);
	jw(0xe1877305u);                        // orr r7, r7, r5, lsl #6
	jw(0xe1877283u);                        // orr r7, r7, r3, lsl #5
	jw(0xe1877206u);                        // orr r7, r7, r6, lsl #4
	jw(0xe1b06402u);                        // lsls r6, r2, #8
	j_dpi(0x0, DP_ORR, R7, R7, 8);          // orreq r7, r7, #FLAG_Z
	j_ubfx(R2, R2, 0, 24);
	if (op == 0 || op == 1) {   // saturate: on overflow N from a, Z clear, the limit by a's sign
		jw(0xe3530000u);                    // cmp r3, #0
		jw(0x0a000006u);                    // beq over (6)
		j_ubfx(R4, R0, 23, 1);
		j_dpi(0xe, DP_BIC, R7, R7, 0x88);
		jw(0xe1877384u);                    // orr r7, r7, r4, lsl #7
		jw(0xe30f2fffu);                    // movw r2, #0xffff
		jw(0xe340207fu);                    // movt r2, #0x7f
		jw(0xe3540000u);                    // cmp r4, #0
		jw(0x13a02880u);                    // movne r2, #0x800000
	} else if (op == 4)
		j_dp(DP_MOV, 0, R2, 0, R0, 0);
	j_ldr(R6, R11, JOFF(ccr));
	j_dpi(0xe, DP_BIC, R6, R6, 0xf8);
	j_dp(DP_ORR, 0, R6, R6, R7, 0);
	jw(0xe6af6076u);                        // sxtb r6, r6
	j_str(R6, R11, JOFF(ccr));
	j_str(R2, R11, JOFF(alu_result));
}

static int jit_build(es5510_t *e)
{
	int end = -1;
	for (int i = 0; i < 0xa0 && end < 0; i++)
		if (s_dec[i].op == 0xf)
			end = i;
	if (end < 2 || s_jit_vm_none || s_dec[end].skippable)
		return 0;
	s_jit_end = end;
	s_jp = s_jit_stage;
	s_jlim = s_jit_stage + JIT_SIZE / 4 - 16;
	s_jfail = 0;
	jw(0xe92d5ff0u);                        // push {r4-r12, lr} (8-byte aligned stack for the helpers)
	jw(0xe1a0b000u);                        // mov r11, r0 (&J)
	j_mov32(R10, (uint32_t)(uintptr_t)e->gpr);
	j_mov32(R9, (uint32_t)(uintptr_t)e->dram);
	s_je = e;
#define SLOT(k) (pc >= (k) ? pc - (k) : end + 1 + pc - (k))
	int nop_steps = 0;
	for (int pc = 0; pc <= end; pc++) {
		if (nop_steps >= 2 && s_dec[pc].nop_run > 2) {
			pc += s_dec[pc].nop_run - 2;
			j_delay_addr(pc - 1);   // (the two before as exec_noskip's skip sets them up: delay line reads)
			j_delay_addr(pc - 2);
		}
		const es5510_dec *d = &s_dec[pc], *pd = &s_dec[SLOT(1)], *p2 = &s_dec[SLOT(2)];
		// RAM pipeline: DIL from the step two before
		if (p2->cycle != RC_WRITE) {
			if (p2->access == RA_IO) {
				jw(0xe3a00000u);            // mov r0, #0
				j_str(R0, R11, JOFF(dil));
			} else
				j_dil_load(SLOT(2));
		}
		// this step's RAM address
		switch (d->access) {
		case RA_DELAY:
			j_delay_addr(pc);
			break;
		case RA_TABLE_A:
		case RA_TABLE_B:
			j_ldr(R0, R10, (uint32_t)pc * 4);
			j_ldr(R1, R11, JOFF(e));
			j_ldr(R1, R1, d->access == RA_TABLE_A ? (uint32_t)offsetof(es5510_t, abase) : (uint32_t)offsetof(es5510_t, bbase));
			jw(0xe0800001u);                // add r0, r0, r1
			j_ldr(R1, R11, JOFF(memmask));
			jw(0xe0000001u);                // and r0, r0, r1
			j_ldr(R1, R11, JOFF(memshift));
			jw(0xe1a00150u);                // asr r0, r0, r1
			j_str(R0, R11, JOFF(addr) + (uint32_t)pc * 4);
			break;
		default:
			j_ldr(R0, R10, (uint32_t)pc * 4);
			j_mov32(R1, 0x00fffff0);
			jw(0xe0000001u);                // and r0, r0, r1
			j_str(R0, R11, JOFF(addr) + (uint32_t)pc * 4);
			break;
		}
		// skip state of this instruction (before the results of the one before)
		if (d->skippable)
			j_call(jit_skip, d->op == OP_CMP);
		// multiplier result of the step before
		if (!pd->mac_nop) {
			uint32_t *b_skip = NULL;
			if (pd->skippable) {
				j_ldr(R0, R11, JOFF(pf));
				jw(0xe3100000u | JPF_MSKIP);    // tst r0, #MSKIP
				b_skip = s_jp;
				jw(0);                          // bne over
			}
			static void (*const MAC[2][5])(uint32_t) = {
				{ jit_mac_g0, jit_mac_gd0, jit_mac_d0, jit_mac_n0, jit_mac_x0 },
				{ jit_mac_g1, jit_mac_gd1, jit_mac_d1, jit_mac_n1, jit_mac_x1 } };
			const uint8_t k = pd->mwk;
			const int v = k == WK_GPR ? 0 : k == (WK_GPR | WK_DOL) ? 1 : k == WK_DOL ? 2 : k == 0 ? 3 : 4;
			if (v == 4)
				j_call(MAC[pd->accumulate ? 1 : 0][v], (uint32_t)k << 8 | pd->cReg);
			else {   // (inline: the result to a general register and / or the delay line out)
				j_mac(pd->accumulate);
				if (k & WK_GPR)
					j_str(R0, R10, (uint32_t)pd->cReg * 4);
				if (k & WK_DOL)
					j_dol();
			}
			if (b_skip && !s_jfail)
				*b_skip = 0x1a000000u | ((uint32_t)((s_jp - b_skip) - 2) & 0x00ffffff);   // bne over
		}
		// multiplier operands
		if (!d->mac_nop) {
			if (d->msrc == SD_REG)
				j_operand(d->cReg);
			else
				j_operand_dil();
			j_str(R0, R11, JOFF(cV));
			j_operand(d->dReg);
			j_str(R0, R11, JOFF(dV));
		}
		// ALU result of the step before
		if (!pd->alu_nop) {
			if (pd->skippable)
				j_call(jit_alu_s, (uint32_t)(pd->op & 15) | (uint32_t)pd->awk << 8 | (uint32_t)pd->aReg << 16);
			else if (j_alu_inline(pd->op & 15) && !(pd->awk & WK_SPR)) {
				j_alu(pd->op & 15);
				if (pd->awk & WK_GPR) {
					j_ubfx(R0, R2, 0, 24);
					j_str(R0, R10, (uint32_t)pd->aReg * 4);
				}
				if (pd->awk & WK_DOL) {
					j_dp(DP_MOV, 0, R0, 0, R2, 0);
					j_dol();
				}
			} else if (pd->awk == WK_GPR)
				j_call(JIT_ALU_G[pd->op & 15], pd->aReg);
			else
				j_call(JIT_ALU_X[pd->op & 15], (uint32_t)pd->awk << 8 | pd->aReg);
		}
		// ALU operands, or END
		if (!d->alu_nop) {
			if (d->op == 0xf)
				j_call(jit_end, 0);
			else if (d->operands == 1) {
				if (d->asrc == SD_REG)
					j_operand(d->bReg);
				else
					j_operand_dil();
				j_str(R0, R11, JOFF(bV));
			} else {
				if (d->asrc == SD_REG)
					j_operand(d->aReg);
				else
					j_operand_dil();
				j_str(R0, R11, JOFF(aV));
				j_operand(d->bReg);
				j_str(R0, R11, JOFF(bV));
			}
		}
		// RAM cycle of the step before
		if (pd->cycle != RC_READ) {
			if (pd->cycle == RC_WRITE && pd->access != RA_IO) {
				j_ldr(R0, R11, JOFF(addr) + (uint32_t)SLOT(1) * 4);
				jw(0xe7f30050u | R0 << 12 | R0);        // ubfx r0, r0, #0, #20
				jw(0xe0800080u | R9 << 16 | R0 << 12 | R0);   // add r0, r9, r0, lsl #1
				j_ldr(R1, R11, JOFF(dol0));
				jw(0xe1c010b0u);                    // strh r1, [r0]
			}
			j_dol_shift();
		}
		if (d->skippable) {   // (pf for the next step)
			j_ldr(R0, R11, JOFF(npf));
			j_str(R0, R11, JOFF(pf));
		}
		nop_steps = d->nop ? nop_steps + 1 : 0;
	}
#undef SLOT
	jw(0xe8bd9ff0u);                        // pop {r4-r12, pc}
	s_jit_stage_len = (unsigned)((uint8_t *)s_jp - (uint8_t *)s_jit_stage);
#if !defined(__vita__)   // (PC harness: RBX_JITSTAT prints the code's size, calls and ALU operations)
	if (getenv("RBX_JITSTAT")) { unsigned nb = 0, h[16] = { 0 }; for (uint32_t *q = s_jit_stage; q < s_jp; q++) nb += *q == 0xe12fff3cu;
	  for (int i = 0; i <= end; i++) if (!s_dec[i].alu_nop) h[s_dec[i].op & 15]++;
	  fprintf(stderr, "JITSTAT end %d words %u blx %u ops", end, s_jit_stage_len / 4, nb);
	  for (int i = 0; i < 16; i++) fprintf(stderr, " %u", h[i]); fprintf(stderr, "\n");
	  const void *fn[] = { (void *)jit_rreg, (void *)jit_dol, (void *)jit_delay_mod, (void *)jit_skip, (void *)jit_alu_s, (void *)jit_end };
	  unsigned fc[8] = { 0 };
	  for (uint32_t *q = s_jit_stage + 2; q < s_jp; q++) if (*q == 0xe12fff3cu) {
		uint32_t w = q[-2], t = q[-1], v = ((w >> 4) & 0xf000) | (w & 0xfff);
		v |= (((t >> 4) & 0xf000) | (t & 0xfff)) << 16;
		int k = 6; for (int j = 0; j < 6; j++) if ((uint32_t)(uintptr_t)fn[j] == v) k = j;
		fc[k]++; }
	  fprintf(stderr, "JITCALLS rreg %u dol %u dmod %u skip %u alus %u end %u other %u\n", fc[0], fc[1], fc[2], fc[3], fc[4], fc[5], fc[6]); }
#endif
	return !s_jfail;
}

// one pass by the generated code: exec_noskip's state in and out
static void jit_pass(es5510_t *e)
{
	J.e = e;
	J.dil = e->dil; J.dol0 = e->dol[0]; J.dol1 = e->dol[1]; J.dol_count = e->dol_count;
	jit_fill();
	J.cV = e->mulacc.cValue; J.dV = e->mulacc.dValue; J.aV = e->alu.aValue; J.bV = e->alu.bValue;
	J.mac_product = e->mulacc.product; J.mac_result = e->mulacc.result; J.alu_result = e->alu.result;
	J.addr[s_jit_end] = (uint32_t)e->ram.address;
	J.addr[s_jit_end - 1] = (uint32_t)e->ram_p.address;
	((void (*)(jit_state *))(uintptr_t)s_jit_live)(&J);
	jit_spill();
	e->pc = (uint8_t)(s_jit_end + 1);
	const es5510_dec *pd = &s_dec[s_jit_end], *p1 = &s_dec[s_jit_end - 1], *p2 = &s_dec[s_jit_end - 2];
	e->mulacc.cReg = pd->cReg; e->mulacc.dReg = pd->dReg; e->mulacc.src = pd->msrc; e->mulacc.dst = pd->mdst;
	e->mulacc.accumulate = pd->accumulate; e->mulacc.write_result = 1;
	e->alu.aReg = pd->aReg; e->alu.bReg = pd->bReg; e->alu.op = pd->op; e->alu.src = pd->asrc; e->alu.dst = pd->adst;
	e->alu.write_result = 1; e->alu.update_ccr = 1;
	e->mulacc.cValue = J.cV; e->mulacc.dValue = J.dV; e->mulacc.product = J.mac_product; e->mulacc.result = J.mac_result;
	e->alu.aValue = J.aV; e->alu.bValue = J.bV; e->alu.result = J.alu_result;
	e->ram.address = (int32_t)(J.addr[s_jit_end] & 0x00ffffff); e->ram.cycle = pd->cycle; e->ram.io = pd->access == RA_IO;
	e->ram_p.address = (int32_t)(J.addr[s_jit_end - 1] & 0x00ffffff); e->ram_p.cycle = p1->cycle; e->ram_p.io = p1->access == RA_IO;
	e->ram_pp.address = (int32_t)(J.addr[s_jit_end - 2] & 0x00ffffff); e->ram_pp.cycle = p2->cycle; e->ram_pp.io = p2->access == RA_IO;
}

// a steady pass: pc 0 with the END in flight (not skipped) and the RAM stages of END and the step before
static int jit_ready(const es5510_t *e)
{
	const es5510_dec *d = &s_dec[s_jit_end], *d1 = &s_dec[s_jit_end - 1];
	const es5510_mac *m = &e->mulacc;
	const es5510_alu *a = &e->alu;
	return e->pc == 0 && e->state == ST_RUNNING && e->halt_asserted &&
		m->cReg == d->cReg && m->dReg == d->dReg && m->src == d->msrc && m->dst == d->mdst &&
		m->accumulate == d->accumulate && m->write_result &&
		a->aReg == d->aReg && a->bReg == d->bReg && a->op == d->op && a->src == d->asrc && a->dst == d->adst &&
		a->write_result && a->update_ccr &&
		e->ram.cycle == d->cycle && (e->ram.io != 0) == (d->access == RA_IO) &&
		e->ram_p.cycle == d1->cycle && (e->ram_p.io != 0) == (d1->access == RA_IO);
}

// the pass by the generated code when it can (1 = done)
static int jit_run(es5510_t *e)
{
	if (!s_jit_ok) {
		// (staging: not while the emulation thread copies the code before)
		int st = __atomic_load_n(&s_jit_stage_state, __ATOMIC_ACQUIRE);
		if (st == JS_COPYING || !__atomic_compare_exchange_n(&s_jit_stage_state, &st, JS_BUILDING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
			return 0;
		if (jit_build(e)) {
			s_jit_ok = 1;
			s_jit_ver = s_jit_stage_ver = s_jit_ver + 1 + (s_jit_ver + 1 == 0);
			__atomic_store_n(&s_jit_stage_state, JS_READY, __ATOMIC_RELEASE);
		} else {
			s_jit_ok = -1;
			__atomic_store_n(&s_jit_stage_state, JS_NONE, __ATOMIC_RELEASE);
		}
	}
	if (s_jit_ok <= 0 || __atomic_load_n(&s_jit_live_ver, __ATOMIC_ACQUIRE) != s_jit_ver || !jit_ready(e))
		return 0;
	jit_pass(e);
	return 1;
}

void es5510_jit_install(void)
{
	int st = JS_READY;
	if (!__atomic_compare_exchange_n(&s_jit_stage_state, &st, JS_COPYING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
		return;
	if (!s_jit_vm && !s_jit_vm_none) {
		s_jit_vm = (uint32_t *)rbx_jit_alloc(JIT_VM_SIZE);
		s_jit_vm_none = !s_jit_vm;
	}
	const unsigned len = (s_jit_stage_len + 63) & ~63u;
	if (s_jit_vm && s_jit_vm_used + len <= JIT_VM_SIZE) {
		uint32_t *dst = s_jit_vm + s_jit_vm_used / 4;
		s_jit_vm_used += len;
		rbx_jit_write_begin();
		memcpy(dst, s_jit_stage, s_jit_stage_len);
		rbx_jit_write_end(dst, s_jit_stage_len);
		s_jit_live = dst;
		__atomic_store_n(&s_jit_live_ver, s_jit_stage_ver, __ATOMIC_RELEASE);
	}
	__atomic_store_n(&s_jit_stage_state, JS_NONE, __ATOMIC_RELEASE);
}
