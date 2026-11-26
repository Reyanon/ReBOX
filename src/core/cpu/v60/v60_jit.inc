/*  V60 JIT (ADR-046): guest V60 code translated to host ARM code (included by v60.c; ARM hosts with executable memory
    from the frontend, rbx_jit_alloc). The instruction semantics are MAME's (v60 op*.hxx / am*.hxx, BSD-3-Clause,
    copyright-holders: Farfetch'd, R. Belmont) as converted into v60_ops.inc: the same flags, the same accesses in the
    same order through the same bus paths, MAME's 8 cycles an instruction; the interpreter's handlers for the rest.
    This file is part of ReBOX. GPL-2.0-or-later.

    A block is the guest code from a pc up to a transfer, an instruction left to the interpreter that may leave the
    flow, or VJ_MAXI instructions; only code in read-only mapped pages (ROM) is translated (no write can change it; a
    map change makes the blocks again). Every guest instruction is done as the run loop does it: the slice ends before
    it when the cycles ran out, 8 cycles off, then the instruction; the run loop's interrupt test after it is the
    interpreter's when the block returns to the run loop after a handler (inside a slice the interrupt line does not
    change: the driver syncs set_input_line, and no native instruction changes PSW), its polling loop test
    (spin_check) at a taken branch back within 256 bytes. The guest registers and flags live in s_v (memory), m_PPC
    and PC are set at every exit. An instruction left to the interpreter is its handler called with PC at it; the
    block goes on after it when the handler left PC and returned the length the translator found, else it leaves
    with the interpreter's PC += length (and the run loop's tests).
    Host registers in a block: r4 = &s_v, r5 = icount (stored around calls), r6 = v60_bus.rd, r7 = v60_bus.wr,
    r8 / r9 values kept over an access. */

#if defined(__arm__)
#define V60_JIT 1
#include <stddef.h>
#include <stdlib.h>
#include "v60_jitf12.h"

void *rbx_jit_alloc(unsigned size);
void rbx_jit_write_begin(void);
void rbx_jit_write_end(void *code, unsigned size);

#define VJ_TAB_BITS 13
#define VJ_MAXI 48           // (instructions in a block)
#define VJ_BUF 12288         // (words: a block's hot code)
#define VJ_COLD 12288        // (words: its cold code)
#define VJ_NONE ((uint32_t *)(uintptr_t)1)   // (a table entry: no block can be made at that pc)

typedef int (*vj_enter_fn)(uint32_t *body);
static uint32_t *s_vj_code;   // executable (NULL: none yet)
static int s_vj_nocode;
static unsigned s_vj_size, s_vj_used, s_vj_srw;   // words (the first s_vj_srw: the shared routines)
static struct vj_ent { uint32_t pc; uint32_t *body; } (*s_vj_tab)[4];   // (the tables and buffers: on the heap, ADR-040)
static uint8_t s_vj_next[1u << VJ_TAB_BITS];
#define VJ_MID_BITS 13
static struct vj_ent (*s_vj_mid)[4];   // (entries inside the blocks: vj_mid_find)
static int s_vj_on = -1;      // (-1: RBX_V60_NOJIT not read yet)
static int s_vj_dbg;          // RBX_V60_JITDBG: 1 no native instructions (handlers only), 2 no chaining, 4 no native accesses, 8 128 KB of code memory
static int s_vj_stat;         // RBX_V60_JITSTAT=1: counts printed at exit
static uint32_t *s_vj_slot;   // (a chain slot to patch with a jump to the block at PC)
static uint64_t s_vj_nblocks, s_vj_ninsns, s_vj_nflush, s_vj_runs, s_vj_isteps, s_vj_nhot, s_vj_ncold;
static uint32_t s_vj_dump;
static uint32_t s_vj_fb[256];  // (RBX_V60_JITSTAT: the instructions made as handler calls, by opcode)

// the shared routines
enum { SR_ENTER, SR_RET0, SR_RET1, SR_CHAIN, SR_ICX, SR_RD8, SR_RD16, SR_RD32, SR_WR8, SR_WR16, SR_WR32, SR_SPIN,
       SR_DYN, SR_N };
static unsigned s_sr[SR_N];   // (word offsets in s_vj_code)

#define VOFS(f) ((uint32_t)offsetof(struct v60_core, f))
#define V_FL VOFS(flags)
#define V_IC VOFS(icount)
#define V_PPC VOFS(ppc)
#define V_TOUCH VOFS(touch)
#define V_REG(n) (4u * (uint32_t)(n))
#define V_PC V_REG(32)

// ---- the C side of the accesses (the interpreter's functions) ----
static uint64_t s_vj_cc[4];   // (RBX_V60_JITSTAT: slow reads, slow writes, polling tests, read-modify-write helpers)
#define VJ_CC(k) ((void)s_vj_cc[k]++)
static uint32_t vj_rb(uint32_t a) { VJ_CC(0); return V60_RB(a); }
static uint32_t vj_rw(uint32_t a) { VJ_CC(0); return V60_RW(a); }
static uint32_t vj_rd(uint32_t a) { VJ_CC(0); return V60_RD(a); }
static void vj_wb(uint32_t a, uint32_t v) { VJ_CC(1); V60_WB(a, (uint8_t)v); }
static void vj_ww(uint32_t a, uint32_t v) { VJ_CC(1); V60_WW(a, (uint16_t)v); }
static void vj_wd(uint32_t a, uint32_t v) { VJ_CC(1); V60_WD(a, v); }
static int spin_check(void);
// the run loop's polling loop test at a taken branch back (spin_check), gated per branch: a branch whose test found
// nothing 16 times in a row is not tested for its next 512 times (any choice of where to test gives the same
// results: a skip only skips iterations that are the same)
#define VJ_NSPIN 32768
static struct vj_spin_site { uint16_t cool; uint8_t fails, pad; } *s_vj_sp;   // (VJ_NSPIN + 1: the last one not gated)
static unsigned s_vj_nsp;
static void vj_spin(struct vj_spin_site *p)
{
	VJ_CC(2);
	if (spin_check())
		p->fails = 0;
	else if (p != &s_vj_sp[VJ_NSPIN] && ++p->fails >= 16) {
		p->fails = 0;
		p->cool = 512;
	}
}

static void vj_flush(void)
{
	memset(s_vj_tab, 0, (1u << VJ_TAB_BITS) * sizeof(*s_vj_tab));
	memset(s_vj_next, 0, sizeof(s_vj_next));
	memset(s_vj_mid, 0, (1u << VJ_MID_BITS) * sizeof(*s_vj_mid));
	memset(s_vj_sp, 0, (VJ_NSPIN + 1) * sizeof(*s_vj_sp));
	s_vj_nsp = 0;
	s_vj_used = s_vj_srw;
	s_vj_slot = NULL;
	s_vj_nflush++;
}

/***************************************************************************
    the emitter: a block's hot code in s_vb, its cold code (slow paths, exits) in s_vc, put after the hot code at
    the end; branches between them and to the shared routines patched then
***************************************************************************/

static uint32_t *s_vb, *s_vc;
static unsigned s_nb, s_ncd;
static uint32_t *s_cb;
static unsigned *s_cn = &s_nb, s_ccap = VJ_BUF;
static int s_full;
static void E(uint32_t w)
{
	if (*s_cn < s_ccap)
		s_cb[(*s_cn)++] = w;
	else
		s_full = 1;
}
#define VJ_XREF 2048
// fixups: kind 0 hot -> cold position, 1 cold -> hot position, 2 hot -> shared routine, 3 cold -> shared routine
static struct { uint8_t kind; unsigned at, to; } *s_fx;
static unsigned s_nfx;
static void fx_add(int kind, unsigned at, unsigned to)
{
	if (s_nfx < VJ_XREF * 2) {
		s_fx[s_nfx].kind = (uint8_t)kind;
		s_fx[s_nfx].at = at;
		s_fx[s_nfx++].to = to;
	} else
		s_full = 1;
}
static int in_cold(void) { return s_cn == &s_ncd; }
// branch / bl (link) to a shared routine
static void e_bsr(uint32_t cond, int sr, int link)
{
	fx_add(in_cold() ? 3 : 2, *s_cn, (unsigned)sr);
	E(cond << 28 | (link ? 0x0b000000u : 0x0a000000u));
}
// hot branches to the next cold segment
static unsigned s_hot_sites[8], s_nhot;
static void e_tocold(uint32_t cond)
{
	if (s_nhot < 8) {
		s_hot_sites[s_nhot++] = s_nb;
		E(cond << 28 | 0x0a000000u);
	} else
		s_full = 1;
}
static void e_cold(void)
{
	for (unsigned k = 0; k < s_nhot; k++)
		fx_add(0, s_hot_sites[k], s_ncd);
	s_nhot = 0;
	s_cb = s_vc;
	s_cn = &s_ncd;
	s_ccap = VJ_COLD;
}
static void e_hot_nob(void)   // (back to the hot buffer, the cold segment ended by its own branch)
{
	s_cb = s_vb;
	s_cn = &s_nb;
	s_ccap = VJ_BUF;
}
static void e_hot(void)   // (back to the hot code where it is now)
{
	fx_add(1, s_ncd, s_nb);
	E(0xea000000u);
	e_hot_nob();
}

static void e_ldr(int rt, uint32_t off) { E(0xe5940000u | (uint32_t)rt << 12 | off); }    // ldr rt, [r4, #off]
static void e_str(int rt, uint32_t off) { E(0xe5840000u | (uint32_t)rt << 12 | off); }    // str rt, [r4, #off]
static void e_mov(int rd, int rm) { if (rd != rm) E(0xe1a00000u | (uint32_t)rd << 12 | (uint32_t)rm); }
static uint32_t ubfx(int rd, int rn, int lsb, int w) { return 0xe7e00050u | (uint32_t)(w - 1) << 16 | (uint32_t)rd << 12 | (uint32_t)lsb << 7 | (uint32_t)rn; }
static void e_imm(int rd, uint32_t v)   // movw / movt
{
	E(0xe3000000u | (v & 0xf000u) << 4 | (uint32_t)rd << 12 | (v & 0xfffu));
	if (v >> 16)
		E(0xe3400000u | (v >> 16 & 0xf000u) << 4 | (uint32_t)rd << 12 | (v >> 16 & 0xfffu));
}
static int arm_imm(uint32_t v)   // the 12-bit operand for v, -1: none
{
	for (int r = 0; r < 16; r++) {
		const uint32_t x = r ? (v << (2 * r)) | (v >> (32 - 2 * r)) : v;
		if (x < 256)
			return r << 8 | (int)x;
	}
	return -1;
}
// a data processing op with an immediate (opc: 0 and, 1 eor, 2 sub, 4 add, 8 tst, 10 cmp, 12 orr, 14 bic ...),
// r12 when the immediate has no ARM form (rn must not be r12 then)
static void e_dpi(uint32_t opc, int s, int rd, int rn, uint32_t v)
{
	const int k = arm_imm(v);
	if (k >= 0)
		E(0xe2000000u | opc << 21 | (uint32_t)s << 20 | (uint32_t)rn << 16 | (uint32_t)rd << 12 | (uint32_t)k);
	else {
		e_imm(12, v);
		E(0xe0000000u | opc << 21 | (uint32_t)s << 20 | (uint32_t)rn << 16 | (uint32_t)rd << 12 | 12u);
	}
}
static void e_addi(int rd, int rn, uint32_t v)   // rd = rn + v
{
	if (!v)
		e_mov(rd, rn);
	else if (arm_imm(v) >= 0)
		e_dpi(4, 0, rd, rn, v);
	else if (arm_imm(0u - v) >= 0)
		e_dpi(2, 0, rd, rn, 0u - v);
	else
		e_dpi(4, 0, rd, rn, v);
}
// a data processing op with a register (shift: type << 5 | amount << 7)
static void e_dpr(uint32_t opc, int s, int rd, int rn, int rm, uint32_t shift)
{
	E(0xe0000000u | opc << 21 | (uint32_t)s << 20 | (uint32_t)rn << 16 | (uint32_t)rd << 12 | shift | (uint32_t)rm);
}
#define LSL(n) ((uint32_t)(n) << 7)
#define LSR(n) ((uint32_t)(n) << 7 | 0x20u)
#define ASR(n) ((uint32_t)(n) << 7 | 0x40u)
static void e_call(const void *fn)   // blx fn (r0-r3 the arguments)
{
	e_imm(12, (uint32_t)(uintptr_t)fn);
	E(0xe12fff3cu);
}
// the guest register n -> rt (dim 0 byte, 1 half, 2 word: zero-extended)
static void e_ldreg(int rt, int n, int dim)
{
	const uint32_t o = V_REG(n);
	if (dim == 0)
		E(0xe5d40000u | (uint32_t)rt << 12 | o);
	else if (dim == 1)
		E(0xe1d400b0u | (uint32_t)rt << 12 | (o >> 4) << 8 | (o & 15));
	else
		e_ldr(rt, o);
}
// rt -> the guest register n (SETREG8 / SETREG16: the low byte / half)
static void e_streg(int rt, int n, int dim)
{
	const uint32_t o = V_REG(n);
	if (dim == 0)
		E(0xe5c40000u | (uint32_t)rt << 12 | o);
	else if (dim == 1)
		E(0xe1c400b0u | (uint32_t)rt << 12 | (o >> 4) << 8 | (o & 15));
	else
		e_str(rt, o);
}

/***************************************************************************
    the accesses: the page's memory inline when it is mapped and the access is inside the page (the interpreter's
    V60_R* / V60_W* do the same there), else the interpreter's function (handlers, split accesses)
***************************************************************************/

// r0 = the value read at the address in r0 (r1-r3, r12, lr clobbered)
static void e_read(int dim)
{
	if (s_vj_dbg & 4) {
		e_bsr(0xe, SR_RD8 + dim, 1);
		return;
	}
	E(v60_bus.amask == 0xffffff ? 0xe3c004ffu : 0xe3c004fcu);   // bic r0, r0, #0xff000000 / #0xfc000000 (26 bits)
	E(0xe1a0c620u);   // lsr r12, r0, #12
	E(0xe796c10cu);   // ldr r12, [r6, r12, lsl #2]
	E(0xe35c0000u);   // cmp r12, #0
	e_tocold(0x0);    // beq
	E(ubfx(1, 0, 0, 12));
	if (dim) {
		E(0xe2812000u | (dim == 2 ? 3u : 1u));   // add r2, r1, #3 / #1
		E(0xe3120a01u);   // tst r2, #0x1000
		e_tocold(0x1);    // bne
	}
	E(dim == 0 ? 0xe7dc0001u : dim == 1 ? 0xe19c00b1u : 0xe79c0001u);   // ldrb / ldrh / ldr r0, [r12, r1]
	e_cold();
	e_bsr(0xe, SR_RD8 + dim, 1);
	e_hot();
}
// the value in r1 written at the address in r0 (r0-r3, r12, lr clobbered)
static void e_write(int dim)
{
	if (s_vj_dbg & 4) {
		e_bsr(0xe, SR_WR8 + dim, 1);
		return;
	}
	E(v60_bus.amask == 0xffffff ? 0xe3c004ffu : 0xe3c004fcu);   // bic r0, r0, #0xff000000 / #0xfc000000 (26 bits)
	E(0xe1a0c620u);   // lsr r12, r0, #12
	E(0xe797c10cu);   // ldr r12, [r7, r12, lsl #2]
	E(0xe35c0000u);   // cmp r12, #0
	e_tocold(0x0);
	E(ubfx(2, 0, 0, 12));
	if (dim) {
		E(0xe2823000u | (dim == 2 ? 3u : 1u));   // add r3, r2, #3 / #1
		E(0xe3130a01u);   // tst r3, #0x1000
		e_tocold(0x1);
	}
	e_ldr(3, V_TOUCH);
	E(0xe2833001u);   // add r3, r3, #1
	e_str(3, V_TOUCH);
	E(dim == 0 ? 0xe7cc1002u : dim == 1 ? 0xe18c10b2u : 0xe78c1002u);   // strb / strh / str r1, [r12, r2]
	e_cold();
	e_bsr(0xe, SR_WR8 + dim, 1);
	e_hot();
}

// the flags from the host NZCV after an add / a sub (CY the borrow) / a logic op (CY kept: its byte or'ed back)
static void e_flags(int kind)
{
	// (conditional ors rather than mrs: qemu reads the CPSR through a helper call)
	if (kind == 2)
		E(0xe5d42000u | V_FL);   // ldrb r2, [r4, #CY] (kept)
	else {
		E(0xe3a02000u);          // mov r2, #0
		E((kind ? 0x30000000u : 0x20000000u) | 0x03822001u);   // orrcc / orrcs r2, r2, #1 (CY)
		E(0x63822c01u);          // orrvs r2, r2, #0x100 (OV)
	}
	E(0x43822801u);              // orrmi r2, r2, #0x10000 (S)
	E(0x03822401u);              // orreq r2, r2, #0x1000000 (Z)
	e_str(2, V_FL);
}
// TEST's flags: Z and S of the value in rv (top aligned: S its bit 31), CY = OV = 0
static void e_flags_test(int rv)
{
	E(0xe3500000u | (uint32_t)rv << 16);   // cmp rv, #0 (N, Z of rv)
	E(0xe3a02000u);   // mov r2, #0
	E(0x43822801u);   // orrmi r2, r2, #0x10000
	E(0x03822401u);   // orreq r2, r2, #0x1000000
	e_str(2, V_FL);
}

/***************************************************************************
    the addressing modes (am1 read, am2 address, am3 write: MAME's tables), decoded when translating
***************************************************************************/

enum { K_REG, K_IMM, K_MEM };
typedef struct {
	uint8_t k;
	int8_t breg;      // K_REG: the register; K_MEM: the base register (-1: the constant c)
	int8_t ireg;      // K_MEM: the index register (-1: none), shifted by ish
	uint8_t ish, ind; // ind: the address is the 32-bit word at base + c, plus d2 (then the index)
	int8_t inc;       // 1: post-increment of breg by isz, -1: pre-decrement
	uint8_t isz;
	uint32_t c, d2;   // K_IMM: c the value
} vam;

static inline uint8_t vrb(uint32_t a) { return V60_RB(a); }
static inline uint16_t vrw(uint32_t a) { return V60_RW(a); }
static inline uint32_t vrd(uint32_t a) { return V60_RD(a); }

// the mode at modadd (m the mode bit, kind 1 am1, 2 am2, 3 am3) of an instruction at pc: its length (0 not done here)
static int vj_am(uint32_t modadd, int m, int dim, int kind, uint32_t pc, vam *a)
{
	const uint8_t mv = vrb(modadd);
	const int r = mv & 0x1f;
	memset(a, 0, sizeof(*a));
	a->k = K_MEM;
	a->breg = -1;
	a->ireg = -1;
	if (dim > 2)
		return 0;
	if (!m) {
		switch (mv >> 5) {
		case 0: a->breg = (int8_t)r; a->c = (uint32_t)(int8_t)vrb(modadd + 1); return 2;
		case 1: a->breg = (int8_t)r; a->c = (uint32_t)(int16_t)vrw(modadd + 1); return 3;
		case 2: a->breg = (int8_t)r; a->c = vrd(modadd + 1); return 5;
		case 3: a->breg = (int8_t)r; return 1;
		case 4: a->breg = (int8_t)r; a->c = (uint32_t)(int8_t)vrb(modadd + 1); a->ind = 1; return 2;
		case 5: a->breg = (int8_t)r; a->c = (uint32_t)(int16_t)vrw(modadd + 1); a->ind = 1; return 3;
		case 6: a->breg = (int8_t)r; a->c = vrd(modadd + 1); a->ind = 1; return 5;
		default:   // group 7
			if (r < 0x10) {   // immediate quick
				if (kind != 1)
					return 0;
				a->k = K_IMM;
				a->c = mv & 0xf;
				return 1;
			}
			switch (r) {
			case 0x10: a->c = pc + (uint32_t)(int8_t)vrb(modadd + 1); return 2;
			case 0x11: a->c = pc + (uint32_t)(int16_t)vrw(modadd + 1); return 3;
			case 0x12: a->c = pc + vrd(modadd + 1); return 5;
			case 0x13: a->c = vrd(modadd + 1); return 5;
			case 0x14:   // immediate
				if (kind != 1)
					return 0;
				a->k = K_IMM;
				if (dim == 0) { a->c = vrb(modadd + 1); return 2; }
				if (dim == 1) { a->c = vrw(modadd + 1); return 3; }
				a->c = vrd(modadd + 1);
				return 5;
			case 0x18: a->c = pc + (uint32_t)(int8_t)vrb(modadd + 1); a->ind = 1; return 2;
			case 0x19: a->c = pc + (uint32_t)(int16_t)vrw(modadd + 1); a->ind = 1; return 3;
			case 0x1a: a->c = pc + vrd(modadd + 1); a->ind = 1; return 5;
			case 0x1b: a->c = vrd(modadd + 1); a->ind = 1; return 5;
			case 0x1c: a->c = pc + (uint32_t)(int8_t)vrb(modadd + 1); a->ind = 1; a->d2 = (uint32_t)(int8_t)vrb(modadd + 2); return 3;
			case 0x1d: a->c = pc + (uint32_t)(int16_t)vrw(modadd + 1); a->ind = 1; a->d2 = (uint32_t)(int16_t)vrw(modadd + 3); return 5;
			case 0x1e: a->c = pc + vrd(modadd + 1); a->ind = 1; a->d2 = vrd(modadd + 5); return 9;
			default: return 0;
			}
		}
	}
	switch (mv >> 5) {
	case 0: a->breg = (int8_t)r; a->c = (uint32_t)(int8_t)vrb(modadd + 1); a->ind = 1; a->d2 = (uint32_t)(int8_t)vrb(modadd + 2); return 3;
	case 1: a->breg = (int8_t)r; a->c = (uint32_t)(int16_t)vrw(modadd + 1); a->ind = 1; a->d2 = (uint32_t)(int16_t)vrw(modadd + 3); return 5;
	case 2: a->breg = (int8_t)r; a->c = vrd(modadd + 1); a->ind = 1; a->d2 = vrd(modadd + 5); return 9;
	case 3: a->k = K_REG; a->breg = (int8_t)r; return 1;
	case 4: a->breg = (int8_t)r; a->inc = 1; a->isz = (uint8_t)(1u << dim); return 1;
	case 5: a->breg = (int8_t)r; a->inc = -1; a->isz = (uint8_t)(1u << dim); return 1;
	case 6: {   // group 6: the second byte
		const uint8_t mv2 = vrb(modadd + 1);
		const int r2 = mv2 & 0x1f;
		a->ireg = (int8_t)r;
		a->ish = (uint8_t)dim;
		switch (mv2 >> 5) {
		case 0: a->breg = (int8_t)r2; a->c = (uint32_t)(int8_t)vrb(modadd + 2); return 3;
		case 1: a->breg = (int8_t)r2; a->c = (uint32_t)(int16_t)vrw(modadd + 2); return 4;
		case 2: a->breg = (int8_t)r2; a->c = vrd(modadd + 2); return 6;
		case 3: a->breg = (int8_t)r2; return 2;
		case 4: a->breg = (int8_t)r2; a->c = (uint32_t)(int8_t)vrb(modadd + 2); a->ind = 1; return 3;
		case 5: a->breg = (int8_t)r2; a->c = (uint32_t)(int16_t)vrw(modadd + 2); a->ind = 1; return 4;
		case 6: a->breg = (int8_t)r2; a->c = vrd(modadd + 2); a->ind = 1; return 6;
		default:   // group 7a
			if (!(mv2 & 0x10))
				return 0;
			switch (mv2 & 0xf) {
			case 0: a->c = pc + (uint32_t)(int8_t)vrb(modadd + 2); return 3;
			case 1: a->c = pc + (uint32_t)(int16_t)vrw(modadd + 2); return 4;
			case 2: a->c = pc + vrd(modadd + 2); return 6;
			case 3: a->c = vrd(modadd + 2); return 6;
			case 8: a->c = pc + (uint32_t)(int8_t)vrb(modadd + 2); a->ind = 1; return 3;
			case 9: a->c = pc + (uint32_t)(int16_t)vrw(modadd + 2); a->ind = 1; return 4;
			case 10: a->c = pc + vrd(modadd + 2); a->ind = 1; return 6;
			case 11:
				a->c = vrd(modadd + 2);
				a->ind = 1;
				if (kind == 3)
					a->ish = 0;   // (MAME's am3DirectAddressDeferredIndexed: the index not scaled)
				return 6;
			default: return 0;
			}
		}
	}
	default: return 0;   // (error 1)
	}
}

// the address of a K_MEM mode -> r0, with its register update (r1-r3, r12, lr clobbered)
static void e_ea(const vam *a)
{
	if (a->breg >= 0) {
		e_ldr(0, V_REG(a->breg));
		if (a->inc < 0) {
			e_dpi(2, 0, 0, 0, a->isz);   // sub r0, r0, #isz
			e_str(0, V_REG(a->breg));
		} else if (a->inc > 0) {
			e_dpi(4, 0, 1, 0, a->isz);   // add r1, r0, #isz
			e_str(1, V_REG(a->breg));
		}
		e_addi(0, 0, a->c);
	} else
		e_imm(0, a->c);
	if (a->ind) {
		e_read(2);
		e_addi(0, 0, a->d2);
	}
	if (a->ireg >= 0) {
		e_ldr(1, V_REG(a->ireg));
		e_dpr(4, 0, 0, 0, 1, LSL(a->ish));   // add r0, r0, r1, lsl #ish
	}
}
// the value of a read mode (am1) -> rd (zero-extended to its dim)
static void e_value(const vam *a, int dim, int rd)
{
	if (a->k == K_REG)
		e_ldreg(rd, a->breg, dim);
	else if (a->k == K_IMM)
		e_imm(rd, a->c);
	else {
		e_ea(a);
		e_read(dim);
		e_mov(rd, 0);
	}
}
// the value in rv written by a write mode (am3) (rv not r0-r3)
static void e_wvalue(const vam *a, int dim, int rv)
{
	if (a->k == K_REG)
		e_streg(rv, a->breg, dim);
	else {
		e_ea(a);
		e_mov(1, rv);
		e_write(dim);
	}
}

/***************************************************************************
    the block's compile state, exits
***************************************************************************/

static uint32_t s_ppc;     // the pc of the instruction translated last (m_PPC at an exit before the next)
static int s_first;        // the instruction is the block's first (m_PPC at its exit: already set)
static int s_fnorm;        // the flags are known to be 0 / 1 (NORMALIZEFLAGS has nothing to do)

// leave before the instruction at pc (the cycles ran out): PC = pc, m_PPC = the last one's
static void e_icount_exit(uint32_t pc)
{
	E(0xe3550000u);   // cmp r5, #0
	e_tocold(0xd);    // ble
	e_cold();
	e_bsr(0xe, SR_ICX, 1);
	E(pc);
	E(s_first ? 0xffffffffu : s_ppc);
	e_hot_nob();
	E(0xe2455008u);   // sub r5, r5, #8
}
// the polling loop test of a branch back (vj_spin with the branch's gate, the gate's cool count inline)
static void e_spin(void)
{
	struct vj_spin_site *p = &s_vj_sp[s_vj_nsp < VJ_NSPIN ? s_vj_nsp++ : VJ_NSPIN];
	e_imm(1, (uint32_t)(uintptr_t)p);
	E(0xe1d120b0u);   // ldrh r2, [r1]
	E(0xe3520000u);   // cmp r2, #0
	E(0x12422001u);   // subne r2, r2, #1
	E(0x11c120b0u);   // strhne r2, [r1]
	E(0x1a000001u);   // bne (over the next two)
	E(0xe1a00001u);   // mov r0, r1
	e_bsr(0xe, SR_SPIN, 1);
}
// a static transfer from the instruction at from to 'to' (the run loop's polling loop test first when it goes back)
static void e_exit_to(uint32_t from, uint32_t to)
{
	e_imm(0, from);
	e_str(0, V_PPC);
	e_imm(0, to);
	e_str(0, V_PC);
	if (to <= from && from - to < 256)
		e_spin();
	if (s_vj_dbg & 2)
		e_bsr(0xe, SR_RET0, 0);
	else
		e_bsr(0xe, SR_CHAIN, 1);   // (the slot: patched to a branch to the block at 'to')
}
// leave with PC set (in memory): the run loop's tests after the instruction at from
static void e_exit_dyn(uint32_t from)
{
	e_imm(0, from);
	e_str(0, V_PPC);
	e_bsr(0xe, (s_vj_dbg & 2) ? SR_RET1 : SR_DYN, 0);
}

/***************************************************************************
    the instructions
***************************************************************************/

// the format 1 / 2 operands (F12DecodeOperands / F12DecodeFirstOperand + F12WriteSecondOperand), with the order in
// which the interpreter reads them: first = 2 when the second operand's register is read before the first's mode
typedef struct { vam a1, a2; int len, first; } vf12;
static int vj_f12(uint32_t pc, int form, int k1, int d1, int k2, int d2, vf12 *f)
{
	const uint8_t b = vrb(pc + 1);
	f->first = 1;
	if (b & 0x80) {
		const int l1 = vj_am(pc + 2, b & 0x40, d1, k1, pc, &f->a1);
		if (!l1)
			return 0;
		const int l2 = vj_am(pc + 2 + l1, b & 0x20, d2, k2, pc, &f->a2);
		if (!l2)
			return 0;
		f->len = 2 + l1 + l2;
		return 1;
	}
	if (b & 0x20) {
		const int l1 = vj_am(pc + 2, b & 0x40, d1, k1, pc, &f->a1);
		if (!l1)
			return 0;
		memset(&f->a2, 0, sizeof(f->a2));
		f->a2.k = K_REG;
		f->a2.breg = (int8_t)(b & 0x1f);
		f->first = form == 1 ? 2 : 1;
		f->len = 2 + l1;
		return 1;
	}
	memset(&f->a1, 0, sizeof(f->a1));
	f->a1.k = K_REG;
	f->a1.breg = (int8_t)(b & 0x1f);
	const int l2 = vj_am(pc + 2, b & 0x40, d2, k2, pc, &f->a2);
	if (!l2)
		return 0;
	f->len = 2 + l2;
	return 1;
}

// the length of a bit addressing mode (BitReadAM / BitReadAMAddress, MAME am1/am2 bam tables): that of the same
// address mode, which the bit modes repeat; register, immediate and auto-increment / decrement ones are errors there
// (0: not known here)
static int vj_bam_len(uint32_t modadd, int m, uint32_t pc)
{
	vam a;
	const int l = vj_am(modadd, m, 2, 2, pc, &a);
	return l && a.k == K_MEM && !a.inc ? l : 0;
}

// the length of an instruction made as a handler call (0: not known here, the block ends after it)
static int vj_len(uint32_t pc, uint8_t op)
{
	const uint8_t *t = s_vj_f12[op];
	vf12 f;
	vam a;
	int l;
	switch (op) {
	case 0x01: case 0x12: case 0x13: case 0x4a: case 0x4d: case 0x4e: case 0x4f:   // (PSW, tasks, traps)
		return 0;
	case 0xd0: case 0xd1: case 0xd2: case 0xd3: case 0xd4: case 0xd5:
	case 0xd8: case 0xd9: case 0xda: case 0xdb: case 0xdc: case 0xdd:   // INC / DEC
		l = vj_am(pc + 1, op & 1, (op >> 1) & 3, 2, pc, &a);
		return l ? l + 1 : 0;
	case 0xe6: case 0xe7:   // POP
		l = vj_am(pc + 1, op & 1, 2, 3, pc, &a);
		return l ? l + 1 : 0;
	case 0xee: case 0xef:   // PUSH
	case 0xf4: case 0xf5:   // TESTW
		l = vj_am(pc + 1, op & 1, 2, 1, pc, &a);
		return l ? l + 1 : 0;
	case 0xf0: case 0xf1: case 0xf2: case 0xf3:   // TESTB / TESTH
		l = vj_am(pc + 1, op & 1, (op >> 1) & 1, 1, pc, &a);
		return l ? l + 1 : 0;
	case 0x5d: {   // EXTBFS / EXTBFZ / EXTBFL (F7b), INSBFR / INSBFL (F7c): a bit mode, a mode, the length byte
		const uint8_t sub = vrb(pc + 1);
		int l1, l2;
		const int s = sub & 0x1f;
		if (s >= 0x08 && s <= 0x0a) {   // (s_Op5DTable)
			l1 = vj_bam_len(pc + 2, sub & 0x40, pc);
			l2 = l1 ? vj_am(pc + 3 + l1, sub & 0x20, 2, 3, pc, &a) : 0;
		} else if (s == 0x18 || s == 0x19) {
			l1 = vj_am(pc + 2, sub & 0x40, 2, 1, pc, &a);
			l2 = l1 ? vj_bam_len(pc + 2 + l1, sub & 0x20, pc) : 0;
		} else
			return 0;
		return l1 && l2 ? l1 + l2 + 3 : 0;
	}
	}
	if (!t[0])
		return 0;
	// (dimension 3, MOVD: an address mode's length is that of dimension 2 but for an immediate, read by kind 1 only)
	const int d1 = t[1] != 1 && t[2] > 2 ? 2 : t[2], d2 = t[3] != 1 && t[4] > 2 ? 2 : t[4];
	return vj_f12(pc, t[0], t[1], d1, t[3], d2, &f) ? f.len : 0;
}

// an instruction done by its handler (PC = pc as the run loop has it); the block goes on at pc + len when the
// handler left PC and returned len, else it leaves (PC += what it returned; the run loop's tests)
static uint32_t s_vj_fbrun[256];   // (RBX_V60_JITSTAT: the handler calls run, by opcode)
static void e_handler(uint32_t pc, uint8_t op, int len)
{
	if (s_vj_stat) {
		e_imm(0, (uint32_t)(uintptr_t)&s_vj_fbrun[op]);
		E(0xe5901000u);   // ldr r1, [r0]
		E(0xe2811001u);   // add r1, r1, #1
		E(0xe5801000u);   // str r1, [r0]
	}
	e_imm(0, pc);
	e_str(0, V_PC);
	e_str(0, V_PPC);
	e_str(5, V_IC);
	e_call((const void *)s_OpCodeTable[op]);
	e_ldr(5, V_IC);
	s_fnorm = 0;
	if (len) {
		e_ldr(1, V_PC);
		e_imm(2, pc);
		E(0xe1510002u);   // cmp r1, r2
		e_tocold(0x1);
		E(0xe3500000u | (uint32_t)len);   // cmp r0, #len (len < 256)
		e_tocold(0x1);
		e_cold();
		E(0xe0811000u);   // add r1, r1, r0
		e_str(1, V_PC);
		e_bsr(0xe, SR_RET1, 0);
		e_hot_nob();
	} else {
		e_ldr(1, V_PC);
		E(0xe0811000u);   // add r1, r1, r0
		e_str(1, V_PC);
		e_bsr(0xe, SR_RET1, 0);
	}
}

#include "v60_jitops.inc"

/***************************************************************************
    the blocks
***************************************************************************/

static inline unsigned vj_hash(uint32_t pc) { return (pc ^ (pc >> VJ_TAB_BITS) ^ (pc >> 21)) & ((1u << VJ_TAB_BITS) - 1); }
static uint32_t *vj_find(uint32_t pc)
{
	struct vj_ent *e = s_vj_tab[vj_hash(pc)];
	for (int k = 0; k < 4; k++)
		if (e[k].body && e[k].pc == pc)
			return e[k].body;
	return NULL;
}
static void vj_insert(uint32_t pc, uint32_t *body)
{
	const unsigned h = vj_hash(pc);
	struct vj_ent *e = &s_vj_tab[h][s_vj_next[h]++ & 3];
	e->pc = pc;
	e->body = body;
}

// the bytes of an instruction at pc (up to 24) in pages mapped read-only (ROM)
static int vj_rom(uint32_t pc)
{
	for (uint32_t a = pc & ~0xfffu; a < pc + 24; a += 0x1000) {
		const uint32_t p = (a & v60_bus.amask) >> V60_PAGE_SHIFT;
		if (!v60_bus.rd[p] || v60_bus.wr[p])
			return 0;
	}
	return 1;
}

// (ADR-200) the tables and the translation buffers (784 KB) live while a game runs: made at its first run, given back
// at its end (v60_release). Kept for the app's life they sat above the first game's buffers and split the heap's free
// space (newlib's fixed-size heap: CV1000's 64 MB VRAM after a System 32 game). The code memory and the shared
// routines stay (code is only added to, ADR-012); SR_DYN's address of s_vj_tab (its movw / movt) is written again.
static unsigned s_vj_tabimm;   // (word offset in s_vj_code of SR_DYN's movw r3 / movt r3 of s_vj_tab)
static void vj_tables_free(void)
{
	free(s_vj_tab);
	free(s_vj_mid);
	free(s_vj_sp);
	free(s_vb);
	free(s_vc);
	free(s_fx);
	s_vj_tab = NULL;
	s_vj_mid = NULL;
	s_vj_sp = NULL;
	s_vb = s_vc = NULL;
	s_fx = NULL;
}
static int vj_tables(void)
{
	s_vj_tab = calloc(1u << VJ_TAB_BITS, sizeof(*s_vj_tab));
	s_vj_mid = calloc(1u << VJ_MID_BITS, sizeof(*s_vj_mid));
	s_vj_sp = calloc(VJ_NSPIN + 1, sizeof(*s_vj_sp));
	s_vb = calloc(VJ_BUF, 4);
	s_vc = calloc(VJ_COLD, 4);
	s_fx = calloc(VJ_XREF * 2, sizeof(*s_fx));
	if (s_vj_tab && s_vj_mid && s_vj_sp && s_vb && s_vc && s_fx)
		return 1;
	vj_tables_free();
	return 0;
}
static uint32_t vj_movw(int rd, uint32_t v) { return 0xe3000000u | (v & 0xf000u) << 4 | (uint32_t)rd << 12 | (v & 0xfffu); }
static uint32_t vj_movt(int rd, uint32_t v) { return 0xe3400000u | (v >> 16 & 0xf000u) << 4 | (uint32_t)rd << 12 | (v >> 16 & 0xfffu); }
static void vj_release(void)
{
	if (!getenv("RBX_V60_KEEPTAB"))   // (=1: kept as before ADR-200, for the heap measurements)
		vj_tables_free();
}

static int vj_alloc(void)
{
	if (s_vj_code && s_vj_tab)
		return 1;
	if (s_vj_nocode)
		return 0;
	if (s_vj_code) {   // (the tables of the last game were given back: made again, empty)
		if (!vj_tables())
			return 0;   // (the interpreter, this game)
		const uint32_t v = (uint32_t)(uintptr_t)s_vj_tab;
		rbx_jit_write_begin();
		s_vj_code[s_vj_tabimm] = vj_movw(3, v);
		s_vj_code[s_vj_tabimm + 1] = vj_movt(3, v);
		rbx_jit_write_end(s_vj_code + s_vj_tabimm, 8);
		s_cb = s_vb;
		s_cn = &s_nb;
		s_ccap = VJ_BUF;
		vj_flush();
		return 1;
	}
	for (unsigned sz = 4u << 20; sz >= (1u << 20); sz >>= 1)
		if ((s_vj_code = (uint32_t *)rbx_jit_alloc(sz)) != NULL) {
			s_vj_size = (s_vj_dbg & 8) ? 32768u : sz / 4;   // (8: a small code memory, flushed often)
			break;
		}
	if (!s_vj_code || !vj_tables()) {
		s_vj_code = NULL;
		s_vj_nocode = 1;
		return 0;
	}
	// the shared routines
	rbx_jit_write_begin();
	s_cb = s_vj_code;
	s_nb = 0;
	s_cn = &s_nb;
	s_ccap = 1024;
	s_sr[SR_ENTER] = s_nb;
	E(0xe92d4ff8u);   // push {r3-r11, lr}
	e_imm(4, (uint32_t)(uintptr_t)&s_v);
	e_ldr(5, V_IC);
	e_imm(6, (uint32_t)(uintptr_t)v60_bus.rd);
	e_imm(7, (uint32_t)(uintptr_t)v60_bus.wr);
	E(0xe12fff10u);   // bx r0
	s_sr[SR_RET0] = s_nb;
	e_str(5, V_IC);
	E(0xe3a00000u);   // mov r0, #0
	E(0xe8bd8ff8u);   // pop {r3-r11, pc}
	s_sr[SR_RET1] = s_nb;
	e_str(5, V_IC);
	E(0xe3a00001u);   // mov r0, #1
	E(0xe8bd8ff8u);
	s_sr[SR_CHAIN] = s_nb;   // (bl from a slot: the slot = lr - 4)
	E(0xe24e0004u);   // sub r0, lr, #4
	e_imm(1, (uint32_t)(uintptr_t)&s_vj_slot);
	E(0xe5810000u);   // str r0, [r1]
	E(0xea000000u | ((s_sr[SR_RET0] - (s_nb + 2)) & 0x00ffffffu));   // b RET0
	s_sr[SR_ICX] = s_nb;     // (bl, then the data words: PC, m_PPC (~0: unchanged))
	E(0xe59e0000u);   // ldr r0, [lr]
	e_str(0, V_PC);
	E(0xe59e0004u);   // ldr r0, [lr, #4]
	E(0xe3700001u);   // cmn r0, #1
	E(0x15840000u | V_PPC);   // strne r0, [r4, #ppc]
	E(0xea000000u | ((s_sr[SR_RET0] - (s_nb + 2)) & 0x00ffffffu));
	static const void *const fns[6] = { (const void *)vj_rb, (const void *)vj_rw, (const void *)vj_rd,
	                                    (const void *)vj_wb, (const void *)vj_ww, (const void *)vj_wd };
	for (int k = 0; k < 6; k++) {
		s_sr[SR_RD8 + k] = s_nb;
		E(0xe92d4008u);   // push {r3, lr}
		e_str(5, V_IC);
		e_call(fns[k]);
		e_ldr(5, V_IC);
		E(0xe8bd8008u);   // pop {r3, pc}
	}
	s_sr[SR_SPIN] = s_nb;
	E(0xe92d4008u);
	e_str(5, V_IC);
	e_call((const void *)vj_spin);
	e_ldr(5, V_IC);
	E(0xe8bd8008u);
	// SR_DYN: a transfer to PC (in s_v, m_PPC set) by a native instruction: on into the block at PC when there is one
	// and the run loop's tests have nothing to do (no polling loop test: not back within 256 bytes; the interrupt
	// line and PSW unchanged in the slice), else the run loop (RET1)
	s_sr[SR_DYN] = s_nb;
	e_ldr(0, V_PC);
	e_ldr(1, V_PPC);
	E(0xe0412000u);   // sub r2, r1, r0
	E(0xe3520c01u);   // cmp r2, #256
	E(0x3a000000u | ((s_sr[SR_RET1] - (s_nb + 2)) & 0x00ffffffu));   // blo RET1
	E(0xe02026a0u);   // eor r2, r0, r0, lsr #13
	E(0xe0222aa0u);   // eor r2, r2, r0, lsr #21
	E(ubfx(2, 2, 0, VJ_TAB_BITS));
	s_vj_tabimm = s_nb;   // (always both words: written again with the next game's tables, ADR-200)
	E(vj_movw(3, (uint32_t)(uintptr_t)s_vj_tab));
	E(vj_movt(3, (uint32_t)(uintptr_t)s_vj_tab));
	E(0xe0833282u);   // add r3, r3, r2, lsl #5
	for (uint32_t k = 0; k < 4; k++) {
		E(0xe5931000u | 8 * k);         // ldr r1, [r3, #8k]
		E(0xe1510000u);                 // cmp r1, r0
		E(0x0593c000u | (8 * k + 4));   // ldreq r12, [r3, #8k + 4]
		E(0x13a0c000u);                 // movne r12, #0
		E(0xe35c0001u);                 // cmp r12, #1 (NULL, VJ_NONE: no block)
		E(0x812fff1cu);                 // bxhi r12
	}
	E(0xea000000u | ((s_sr[SR_RET1] - (s_nb + 2)) & 0x00ffffffu));
	s_vj_srw = (s_nb + 15) & ~15u;
	rbx_jit_write_end(s_vj_code, s_vj_srw * 4);
	s_vj_used = s_vj_srw;
	s_cb = s_vb;
	s_ccap = VJ_BUF;
	return 1;
}

// the entries inside the blocks (where a slice starts: the instructions after the first, with no flags state assumed)
static uint8_t s_vj_midn[1u << VJ_MID_BITS];
static uint32_t s_mid_pc[VJ_MAXI], s_nmid;
static unsigned s_mid_at[VJ_MAXI];
static void vj_mid_insert(uint32_t pc, uint32_t *at)
{
	const unsigned h = vj_hash(pc) & ((1u << VJ_MID_BITS) - 1);
	struct vj_ent *e = s_vj_mid[h];
	for (int k = 0; k < 4; k++)
		if (e[k].body && e[k].pc == pc)
			return;   // (the older one kept)
	e = &s_vj_mid[h][s_vj_midn[h]++ & 3];
	e->pc = pc;
	e->body = at;
}
static uint32_t *vj_mid_find(uint32_t pc)
{
	struct vj_ent *e = s_vj_mid[vj_hash(pc) & ((1u << VJ_MID_BITS) - 1)];
	for (int k = 0; k < 4; k++)
		if (e[k].body && e[k].pc == pc)
			return e[k].body;
	return NULL;
}

// translate the block at pc (NULL: none can be made there)
static uint32_t *vj_build(uint32_t pc)
{
	if (!vj_rom(pc))
		return NULL;
	s_nb = s_ncd = s_nfx = s_nhot = 0;
	s_cb = s_vb;
	s_cn = &s_nb;
	s_ccap = VJ_BUF;
	s_full = 0;
	s_first = 1;
	s_fnorm = 0;
	s_ppc = pc;
	s_nmid = 0;
	uint32_t at = pc;
	int n = 0, open = 1;
	while (open && !s_full) {
		if (n >= VJ_MAXI || !vj_rom(at) || s_nb > VJ_BUF - 600 || s_ncd > VJ_COLD - 600) {
			// (the block's end: on at 'at' through a chained exit)
			e_exit_to(s_ppc, at);
			break;
		}
		s_fnorm = 0;   // (no state of the code before an instruction assumed: a slice may start at it)
		if (n) {   // (an entry for a slice that starts here)
			s_mid_pc[s_nmid] = at;
			s_mid_at[s_nmid++] = s_nb;
		}
		e_icount_exit(at);
		const uint8_t op = vrb(at);
		int len = (s_vj_dbg & 1) ? -1 : vj_native(at, op, &open);
		if (len < 0) {   // (the handler)
			len = vj_len(at, op);
			if (s_vj_stat)
				s_vj_fb[op]++;
			e_handler(at, op, len);
			if (!len)
				open = 0;
		}
		s_ppc = at;
		at += (uint32_t)len;
		s_first = 0;
		n++;
	}
	if (s_full)
		return NULL;
	// install: hot, then cold
	const unsigned total = s_nb + s_ncd;
	if (s_vj_used + total > s_vj_size) {
		vj_flush();
		if (s_vj_used + total > s_vj_size)
			return NULL;
	}
	uint32_t *dst = s_vj_code + s_vj_used;
	const unsigned base = s_vj_used;
	rbx_jit_write_begin();
	memcpy(dst, s_vb, s_nb * 4);
	memcpy(dst + s_nb, s_vc, s_ncd * 4);
	for (unsigned k = 0; k < s_nfx; k++) {
		const unsigned at_w = s_fx[k].kind == 0 || s_fx[k].kind == 2 ? s_fx[k].at : s_nb + s_fx[k].at;
		unsigned to_w;
		switch (s_fx[k].kind) {
		case 0: to_w = s_nb + s_fx[k].to; break;
		case 1: to_w = s_fx[k].to; break;
		default: to_w = s_sr[s_fx[k].to] - base; break;
		}
		dst[at_w] |= (uint32_t)((int)to_w - (int)at_w - 2) & 0x00ffffffu;
	}
	rbx_jit_write_end(dst, total * 4);
	s_vj_used += total;
	for (unsigned k = 0; k < s_nmid; k++)
		vj_mid_insert(s_mid_pc[k], dst + s_mid_at[k]);
	s_vj_nblocks++;
	s_vj_ninsns += (unsigned)n;
	s_vj_nhot += s_nb;
	s_vj_ncold += s_ncd;
	if (s_vj_dump == pc) {   // RBX_V60_JITDUMP=pc: the block's code (hot, then cold) to /tmp/vj_dump.bin
		FILE *fp = fopen("/tmp/vj_dump.bin", "wb");
		if (fp) {
			fwrite(dst, 4, total, fp);
			fclose(fp);
		}
		fprintf(stderr, "v60 jit: block %06x-%06x (%d instructions, %u + %u words):", pc, at, n, s_nb, s_ncd);
		for (uint32_t a = pc; a < at && a < pc + 64; a++)
			fprintf(stderr, " %02x", vrb(a));
		fprintf(stderr, "\n");
	}
	return dst;
}

static void vj_stat_print(void)
{
	fprintf(stderr, "v60 jit: %llu blocks, %llu instructions made, %llu flushes, %llu runs, %llu interpreted steps\n",
	        (unsigned long long)s_vj_nblocks, (unsigned long long)s_vj_ninsns, (unsigned long long)s_vj_nflush,
	        (unsigned long long)s_vj_runs, (unsigned long long)s_vj_isteps);
	fprintf(stderr, "v60 jit: slow reads %llu, slow writes %llu, polling tests %llu, returns with the run loop's tests %llu\n",
	        (unsigned long long)s_vj_cc[0],
	        (unsigned long long)s_vj_cc[1], (unsigned long long)s_vj_cc[2], (unsigned long long)s_vj_cc[3]);
	fprintf(stderr, "v60 jit: code words hot %llu, cold %llu (memory %u words)\n", (unsigned long long)s_vj_nhot,
	        (unsigned long long)s_vj_ncold, s_vj_size);
	fprintf(stderr, "v60 jit: handler calls made by opcode:");
	for (int k = 0; k < 256; k++)
		if (s_vj_fb[k])
			fprintf(stderr, " %02x:%u", k, s_vj_fb[k]);
	fprintf(stderr, "\nv60 jit: handler calls run by opcode:");
	for (int k = 0; k < 256; k++)
		if (s_vj_fbrun[k])
			fprintf(stderr, " %02x:%u", k, s_vj_fbrun[k]);
	fprintf(stderr, "\n");
}

static int vj_on(void)
{
	if (s_vj_on < 0) {
		s_vj_on = getenv("RBX_V60_NOJIT") == NULL;
		s_vj_dbg = getenv("RBX_V60_JITDBG") ? atoi(getenv("RBX_V60_JITDBG")) : 0;
		s_vj_stat = getenv("RBX_V60_JITSTAT") != NULL;
		s_vj_dump = getenv("RBX_V60_JITDUMP") ? (uint32_t)strtoul(getenv("RBX_V60_JITDUMP"), NULL, 16) : ~0u;
		if (s_vj_stat)
			atexit(vj_stat_print);
	}
	if (!s_vj_on || !vj_alloc())
		return 0;
	if (s_maps_changed) {
		s_maps_changed = 0;
		vj_flush();
	}
	return 1;
}

// the hot counter: a block is made at a pc the flow came to by a transfer the second time
#define VJ_HOT_BITS 12
static struct { uint32_t pc; uint32_t n; } s_vj_hot[1u << VJ_HOT_BITS];
static int vj_hot(uint32_t pc, unsigned need)
{
	const unsigned h = (pc ^ (pc >> 12)) & ((1u << VJ_HOT_BITS) - 1);
	if (s_vj_hot[h].pc != pc) {
		s_vj_hot[h].pc = pc;
		s_vj_hot[h].n = 1;
		return 0;
	}
	return ++s_vj_hot[h].n >= need;
}

static void vj_insert_none(uint32_t pc) { vj_insert(pc, VJ_NONE); }

// the run loop with blocks (the interpreter's loop where there is none)
static void vj_run(void)
{
	const vj_enter_fn enter = (vj_enter_fn)(uintptr_t)(s_vj_code + s_sr[SR_ENTER]);
	// (blocks are made only where the flow came by a transfer or a block's end; the slice starts anywhere: there an
	// entry inside a block)
	int look = 1, make = 0;
	while (m_icount > 0) {
		if (look) {
			uint32_t *b = vj_find(PC);
			if (!b && !make)
				b = vj_mid_find(PC);
			if (!b && (s_vj_slot || (make && vj_hot(PC, 2)))) {
				b = vj_build(PC);
				if (b)
					vj_insert(PC, b);
				else
					vj_insert_none(PC);
			}
			if (s_vj_slot) {   // (the exit that came here: chained to the block)
				if (b && b != VJ_NONE) {
					rbx_jit_write_begin();
					*s_vj_slot = 0xea000000u | ((uint32_t)(b - (s_vj_slot + 2)) & 0x00ffffffu);
					rbx_jit_write_end(s_vj_slot, 4);
				}
				s_vj_slot = NULL;
			}
			if (b && b != VJ_NONE) {
				s_vj_runs++;
				make = 1;
				const int post = enter(b);
				if (post) {
					VJ_CC(3);
					if (m_irq_line != 0)
						v60_try_irq();
					if (PC <= m_PPC && m_PPC - PC < 256)
						spin_check();
				}
				continue;
			}
		}
		// the interpreter's step
		const uint32_t at = PC;
		m_PPC = PC;
		m_icount -= 8;
		const uint32_t inc = s_OpCodeTable[OpRead8(PC)]();
		PC += inc;
		if (m_irq_line != 0)
			v60_try_irq();
		if (PC <= m_PPC && m_PPC - PC < 256)
			spin_check();
		look = !inc || PC != at + inc;
		make |= look;
		s_vj_isteps++;
	}
	s_vj_slot = NULL;
}

#endif
