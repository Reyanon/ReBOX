/*  ARM7 core: guest ARM code translated to host ARM code (included by arm7.c; ARM hosts with executable memory from the
    frontend, rbx_jit_alloc).
    This file is part of ReBOX. GPL-2.0-or-later.

    A block is the guest ARM code from a pc up to a branch, a write of R15, an instruction it does not translate, the
    end of its 1 KB page, or 32 instructions; only code in read-only pages (ROM) is translated, so no write can change
    it. Each guest instruction becomes:
      - data processing with an immediate or a register shifted by an immediate (no R15 operand or result): the same
        instruction on host registers r0-r2, with the guest flags put into the host's first and taken back after;
      - LDR / STR (byte or word) with an immediate offset, pre-indexed, no write-back (no R15): a call of RD32 / RD8 /
        WR32 / WR8;
      - B / BL: the new pc, and the block ends;
      - the rest: a call of the interpreter for that one instruction (arm_exec), ending the block when it changed the
        pc's flow.
    The cycle count, the condition test, the speedup hook, the interrupt test and the end of the slice are done after
    every instruction exactly as arm7_run does them, so a run gives the same results with or without translation. */

#if defined(__arm__) && !defined(__thumb__) || defined(__arm__)
#define ARM7_JIT_HOST 1
#endif

#ifdef ARM7_JIT_HOST

void *rbx_jit_alloc(unsigned size);
void rbx_jit_write_begin(void);
void rbx_jit_write_end(void *code, unsigned size);

#define JIT_CODE_SIZE (8u << 20)
#define JIT_TRAMP 64   // words at the start of the code memory: the memory access slow paths (jit_code_init)
#define JIT_TAB_BITS 14
#define JIT_MAX_INSNS 32

// a block returns the pc of its last instruction for the idle check after it (arm7_run's prev_pc), or the next pc
// when there is nothing to check (the check needs R15 below it)
typedef uint32_t (*jit_block_fn)(arm7_t *c);
static uint32_t *s_jcode;            // JIT_CODE_SIZE executable (NULL: none yet)
static int s_jnone;
static unsigned s_jused;             // words
static struct { uint32_t pc; jit_block_fn fn; } s_jtab[1u << JIT_TAB_BITS][4];   // (4 ways a set)
static uint8_t s_jnext[1u << JIT_TAB_BITS];
static uint32_t s_jbuf[JIT_MAX_INSNS * 48 + 128];   // a block being made
static unsigned s_jn;
// (in a block icount lives in r5, spilled around calls; the exits branch to stubs and two tails at the block's end)
static unsigned s_stub_at[JIT_MAX_INSNS * 4], s_nstub;
static uint32_t s_stub_pc[JIT_MAX_INSNS * 4];
static unsigned s_tpc_at[JIT_MAX_INSNS * 4], s_ntpc;   // -> tail_pc: R15 = r0, return r0
static unsigned s_tret_at[JIT_MAX_INSNS * 4], s_ntret; // -> tail_ret: return r0 (R15 set)
static unsigned s_bl_at[JIT_MAX_INSNS * 4], s_nbl;     // bl to a trampoline, patched at install
static uint8_t s_bl_k[JIT_MAX_INSNS * 4];
static unsigned s_tramp_at[4];                          // ld32, ld8, st32, st8 (words into the code memory)

static void jit_flush_all(void)
{
	memset(s_jtab, 0, sizeof(s_jtab));
	memset(s_jnext, 0, sizeof(s_jnext));
	s_jused = JIT_TRAMP;   // (the trampolines stay)
}

// ---- helpers called by the generated code (c in r0) ----
// one instruction by the interpreter (its condition passed): 1 = the flow went on to pc + 4
static int jit_interp(arm7_t *c, uint32_t insn, uint32_t pc)
{
	R15 = pc;
	if (UNLIKELY((insn >> 28) == 0xf))
		arm9_exec_nv(c, insn);   // (archRev 5)
	else
		arm_exec(c, insn);
	if (c->pending_interrupt)
		check_irq_state(c);
	ICOUNT -= 3;
	return R15 == pc + 4 && !c->pending_interrupt;
}
// the slow path of the generated access: the access, and the page into the cache when it is mapped
static void tlb_fill_r(arm7_t *c, uint32_t a)
{
	uint8_t *p = (uint8_t *)page_r(c, a);
	if (p) {
		const unsigned k = (a >> ARM7_PAGE_SHIFT) & 3;
		c->tlb_r[k].tag = a >> ARM7_PAGE_SHIFT;
		c->tlb_r[k].mem = p;
	}
}
static void tlb_fill_w(arm7_t *c, uint32_t a)
{
	uint8_t *p = page_w(c, a);
	if (p) {
		const unsigned k = (a >> ARM7_PAGE_SHIFT) & 3;
		c->tlb_w[k].tag = a >> ARM7_PAGE_SHIFT;
		c->tlb_w[k].mem = p;
	}
}
static uint32_t jit_ld32(arm7_t *c, uint32_t a) { tlb_fill_r(c, a); return RD32(c, a); }
// one Thumb instruction by the interpreter: 1 = the flow went on to pc + 2 in Thumb state
static int jit_tinterp(arm7_t *c, uint32_t op, uint32_t pc)
{
	R15 = pc;
	thumb_exec(c, op);
	if (c->pending_interrupt)
		check_irq_state(c);
	ICOUNT -= 3;
	return R15 == pc + 2 && (CPSR & T_MASK) && !c->pending_interrupt;
}
// BX Rm (arm7ops_0123)
static void jit_bx(arm7_t *c, uint32_t rm)
{
	R15 = c->r[rm];
	if (R15 & 1) {
		R15--;
		set_cpsr(c, CPSR | T_MASK);
	}
}
static uint32_t jit_ld8(arm7_t *c, uint32_t a) { tlb_fill_r(c, a); return RD8(c, a); }
static void jit_st32(arm7_t *c, uint32_t a, uint32_t v) { tlb_fill_w(c, a); WR32(c, a, v); }
static void jit_st8(arm7_t *c, uint32_t a, uint32_t v) { tlb_fill_w(c, a); WR8(c, a, v & 0xff); }
// after a memory access that raised an interrupt line: the interpreter's test after the instruction
static void jit_irq(arm7_t *c, uint32_t next)
{
	R15 = next;
	check_irq_state(c);
}

// ---- emitter ----
static void E(uint32_t w) { s_jbuf[s_jn++] = w; }
#define OFS_R(n) ((uint32_t)offsetof(arm7_t, r) + 4u * (n))
#define OFS_CPSR ((uint32_t)offsetof(arm7_t, cpsr))
#define OFS_ICOUNT ((uint32_t)offsetof(arm7_t, icount))
#define OFS_PEND ((uint32_t)offsetof(arm7_t, pending_interrupt))
static void e_ldr(int rt, uint32_t off) { E(0xe5940000u | (uint32_t)rt << 12 | off); }    // ldr rt, [r4, #off]
static void e_str(int rt, uint32_t off) { E(0xe5840000u | (uint32_t)rt << 12 | off); }    // str rt, [r4, #off]
static void e_ldrb(int rt, uint32_t off) { E(0xe5d40000u | (uint32_t)rt << 12 | off); }   // ldrb rt, [r4, #off]
static void e_imm(int rd, uint32_t v)   // movw / movt
{
	E(0xe3000000u | (v & 0xf000u) << 4 | (uint32_t)rd << 12 | (v & 0xfffu));
	if (v >> 16)
		E(0xe3400000u | (v >> 16 & 0xf000u) << 4 | (uint32_t)rd << 12 | (v >> 16 & 0xfffu));
}
static void e_call(const void *fn)   // r12 = fn; blx r12 (r0-r3 arguments); icount (r5) spilled around it
{
	e_str(5, OFS_ICOUNT);
	e_imm(12, (uint32_t)(uintptr_t)fn);
	E(0xe12fff3cu);
	e_ldr(5, OFS_ICOUNT);
}
static void e_mov_r0_c(void) { E(0xe1a00004u); }   // mov r0, r4
// a forward branch to patch: returns its index
static unsigned e_bcond(uint32_t cond) { E(cond << 28 | 0x0a000000u); return s_jn - 1; }
static void e_patch(unsigned at) { s_jbuf[at] |= ((s_jn - at - 2) & 0x00ffffffu); }
// leave the block with the guest pc = pc (returned too: nothing for the idle check)
static void e_exit(uint32_t pc)
{
	e_imm(0, pc);
	s_tpc_at[s_ntpc++] = s_jn;
	E(0xea000000u);   // b tail_pc
}
// leave the block, R15 set, returning r0
static void e_ret(void)
{
	s_tret_at[s_ntret++] = s_jn;
	E(0xea000000u);   // b tail_ret
}
// a conditional exit to the stub that leaves with the guest pc = pc
static void e_stub(uint32_t cond, uint32_t pc)
{
	s_stub_at[s_nstub] = s_jn;
	s_stub_pc[s_nstub++] = pc;
	E(cond << 28 | 0x0a000000u);
}
static void e_sub_icount(uint32_t n) { E(0xe2455000u | n); }   // sub r5, r5, #n
// icount -= n, and the slice's end after this instruction (the guest pc then next)
static void e_cycles(int n, uint32_t next)
{
	E(0xe2555000u | (uint32_t)n);   // subs r5, r5, #n
	e_stub(0xd, next);              // ble: the slice's end
}
static void e_begin(void)
{
	s_jn = s_nstub = s_ntpc = s_ntret = s_nbl = 0;
	E(0xe92d4070u);   // push {r4, r5, r6, lr}
	E(0xe1a04000u);   // mov r4, r0
	e_ldr(5, OFS_ICOUNT);
}
// the stubs and the tails after the block's code, the branches to them patched
static void e_finish(void)
{
	for (unsigned k = 0; k < s_nstub; k++) {
		s_jbuf[s_stub_at[k]] |= (s_jn - s_stub_at[k] - 2) & 0x00ffffffu;
		e_imm(0, s_stub_pc[k]);
		s_tpc_at[s_ntpc++] = s_jn;
		E(0xea000000u);
	}
	for (unsigned k = 0; k < s_ntpc; k++)
		s_jbuf[s_tpc_at[k]] |= (s_jn - s_tpc_at[k] - 2) & 0x00ffffffu;
	e_str(0, OFS_R(15));   // tail_pc
	for (unsigned k = 0; k < s_ntret; k++)
		s_jbuf[s_tret_at[k]] |= (s_jn - s_tret_at[k] - 2) & 0x00ffffffu;
	e_str(5, OFS_ICOUNT);  // tail_ret
	E(0xe8bd8000u | 0x70u); // pop {r4, r5, r6, pc}
}
static void e_flags_in(void)   // the guest NZCV into the host
{
	e_ldr(3, OFS_CPSR);
	E(0xe128f003u);   // msr APSR_nzcvq, r3
}
static void e_flags_out(void)   // the host NZCV into the guest cpsr
{
	E(0xe10f3000u);   // mrs r3, APSR
	e_ldr(12, OFS_CPSR);
	E(0xe3ccc20fu);   // bic r12, r12, #0xf0000000
	E(0xe203320fu);   // and r3, r3, #0xf0000000
	E(0xe18cc003u);   // orr r12, r12, r3
	e_str(12, OFS_CPSR);
}

#define OFS_TLBR ((uint32_t)offsetof(arm7_t, tlb_r))
#define OFS_TLBW ((uint32_t)offsetof(arm7_t, tlb_w))
#define OFS_WRITES ((uint32_t)offsetof(arm7_t, writes))
// an access through the page cache, the helper on a miss (or an unaligned word): load -> r0; store of r2
static void e_access(int load, int byte)
{
	E(0xe1a03521u);   // lsr r3, r1, #10
	E(0xe203c003u);   // and r12, r3, #3
	E(0xe084c18cu);   // add r12, r4, r12, lsl #3
	const uint32_t tofs = load ? OFS_TLBR : OFS_TLBW;
	E(0xe59c0000u | tofs);               // ldr r0, [r12, #tag]
	E(0xe1500003u);                      // cmp r0, r3
	const unsigned miss = e_bcond(0x1);  // bne
	unsigned unal = 0;
	if (!byte) {
		E(0xe3110003u);                  // tst r1, #3
		unal = e_bcond(0x1);             // bne
	}
	E(0xe59c3000u | (tofs + 4));         // ldr r3, [r12, #mem]
	E(0xe7e9c051u);                      // ubfx r12, r1, #0, #10
	if (load)
		E(byte ? 0xe7d3000cu : 0xe793000cu);   // ldrb / ldr r0, [r3, r12]
	else {
		// (the idle check's count of writes that change a value)
		E(byte ? 0xe7d3000cu : 0xe793000cu);   // ldrb / ldr r0, [r3, r12] (the old value)
		E(byte ? 0xe7c3200cu : 0xe783200cu);   // strb / str r2, [r3, r12]
		if (byte)
			E(0xe20220ffu);              // and r2, r2, #0xff
		E(0xe1500002u);                  // cmp r0, r2
		e_ldr(0, OFS_WRITES);
		E(0x12800001u);                  // addne r0, r0, #1
		e_str(0, OFS_WRITES);
	}
	const unsigned done = e_bcond(0xe);
	e_patch(miss);
	if (unal)
		e_patch(unal);
	s_bl_at[s_nbl] = s_jn;   // bl trampoline (c, the address in r1, the value in r2)
	s_bl_k[s_nbl++] = (uint8_t)((load ? 0 : 2) + (byte ? 1 : 0));
	E(0xeb000000u);
	e_patch(done);
}

// the instruction's kind for the translator
enum { JK_DP, JK_MEM, JK_B, JK_BX, JK_LIT, JK_OTHER };
static int jit_kind(uint32_t insn)
{
	const uint32_t rd = (insn >> 12) & 0xf, rn = (insn >> 16) & 0xf;
	if ((insn & 0x0e000000u) == 0x02000000u && rd != 15 && (insn & 0x01900000u) != 0x01000000u &&
	    ((insn >> 21 & 0xd) == 0xd || rn != 15))
		return JK_DP;   // immediate operand
	if ((insn & 0x0e000010u) == 0 && rd != 15 && (insn & 0xf) != 15 && (insn & 0x01900000u) != 0x01000000u &&
	    ((insn >> 21 & 0xd) == 0xd || rn != 15))
		return JK_DP;   // register shifted by an immediate
	// LDR / STR (B) with an immediate offset, pre-indexed (write-back or not) or post-indexed, Rd and Rn not R15
	if ((insn & 0x0e000000u) == 0x04000000u && rd != 15 && rn != 15 && ((insn & INSN_SDT_P) || !(insn & INSN_SDT_W)))
		return JK_MEM;
	if ((insn & 0x0f300000u) == 0x05100000u && rn == 15 && rd != 15)
		return JK_LIT;   // LDR (B) Rd, [pc, #+-imm12]
	if ((insn & 0x0ffffff0u) == 0x012fff10u)
		return JK_BX;
	if ((insn & 0x0e000000u) == 0x0a000000u)
		return JK_B;
	return JK_OTHER;
}

// the block made in s_jbuf into the code memory
static jit_block_fn jit_install(void)
{
	if (s_jused + s_jn > JIT_CODE_SIZE / 4) {   // (full: all of it again)
		extern void rbx_log(const char *fmt, ...);
		rbx_log("arm7 jit: code memory full, cleared");
		jit_flush_all();
		if (s_jn > JIT_CODE_SIZE / 4)
			return NULL;
	}
	uint32_t *dst = s_jcode + s_jused;
	for (unsigned k = 0; k < s_nbl; k++)
		s_jbuf[s_bl_at[k]] |= (s_tramp_at[s_bl_k[k]] - (s_jused + s_bl_at[k]) - 2) & 0x00ffffffu;
	rbx_jit_write_begin();
	memcpy(dst, s_jbuf, s_jn * 4);
	rbx_jit_write_end(dst, s_jn * 4);
	s_jused += s_jn;
	return (jit_block_fn)(void *)dst;
}

// a literal load whose word is in a read-only mapped page (its value cannot change before the code is thrown away)
static int lit_ok(arm7_t *c, uint32_t p, uint32_t insn)
{
	const uint32_t off = insn & 0xfff;
	const uint32_t a = ((insn & INSN_SDT_U) ? p + off : p - off) + 8;
	return page_r(c, a & ~3u) && !page_w(c, a & ~3u);
}

// the code memory, once, its trampolines first: push {r12, lr}; icount out; fn (c, r1, r2); icount in; pop {r12, pc}
static int jit_code_init(void)
{
	if (s_jcode)
		return 1;
	if (s_jnone)
		return 0;
	s_jcode = (uint32_t *)rbx_jit_alloc(JIT_CODE_SIZE);
	if (!s_jcode) {
		s_jnone = 1;
		return 0;
	}
	static const void *const fns[4] = { (const void *)jit_ld32, (const void *)jit_ld8, (const void *)jit_st32,
	                                    (const void *)jit_st8 };
	s_jn = 0;
	for (int k = 0; k < 4; k++) {
		s_tramp_at[k] = s_jn;
		E(0xe92d5000u);   // push {r12, lr}
		e_mov_r0_c();
		e_call(fns[k]);
		E(0xe8bd9000u);   // pop {r12, pc}
	}
	rbx_jit_write_begin();
	memcpy(s_jcode, s_jbuf, s_jn * 4);
	rbx_jit_write_end(s_jcode, s_jn * 4);
	s_jused = JIT_TRAMP;
	return 1;
}

// translate the block at pc (ARM state, a read-only page): NULL = not now (no room, not translatable)
static jit_block_fn jit_build(arm7_t *c, uint32_t pc)
{
	if (!jit_code_init())
		return NULL;
	const uint8_t *page = page_r(c, pc);
	if (!page || page_w(c, pc))
		return NULL;
	e_begin();
	uint32_t p = pc;
	for (int i = 0; i < JIT_MAX_INSNS; i++, p += 4) {
		if ((p >> ARM7_PAGE_SHIFT) != (pc >> ARM7_PAGE_SHIFT)) {   // (the page's end)
			e_exit(p);
			break;
		}
		uint32_t insn;
		memcpy(&insn, page + POFS(p), 4);
		const uint32_t cond = insn >> 28;
		if (cond == 0xf && !c->arch5) {   // (never: archRev 4; archRev 5 runs it: jit_interp)
			e_cycles(1, p + 4);
			if (i == JIT_MAX_INSNS - 1)
				e_exit(p + 4);
			continue;
		}
		if (c->hook_eat && p == c->hook_pc) {   // (arm7_run: before the instruction)
			e_imm(1, (uint32_t)c->hook_eat);
			E(0xe0455001u);   // sub r5, r5, r1
		}
		unsigned skip = 0;
		if (cond != 0xe && cond != 0xf) {   // the condition: not passed -> one cycle (icount += 2 - 3)
			e_flags_in();
			skip = e_bcond(cond ^ 1);
		}
		const int kind = cond == 0xf ? JK_OTHER : jit_kind(insn);
		int end = 0;
		if (kind == JK_DP) {
			const uint32_t op = (insn >> 21) & 0xf;
			const int has_rn = (op & 0xd) != 0xd, s = (insn & INSN_S) != 0;
			const int reg = !(insn & INSN_I);
			if (cond == 0xe)
				e_flags_in();   // (carry in, shifter carry, untouched V)
			if (has_rn)
				e_ldr(1, OFS_R((insn >> 16) & 0xf));
			if (reg)
				e_ldr(2, OFS_R(insn & 0xf));
			uint32_t h = 0xe0000000u | (insn & 0x0ff00fffu);
			h &= ~(0xfu << 16 | 0xfu << 12);
			h |= (has_rn ? 1u : 0u) << 16;
			if (reg)
				h = (h & ~0xfu) | 2u;
			E(h);   // <op>{s} r0, r1, op2
			if (s)
				e_flags_out();
			if ((op & 0xc) != 0x8)
				e_str(0, OFS_R((insn >> 12) & 0xf));
			e_cycles(reg ? 2 : 1, p + 4);   // (HandleALU: += 2, register operand -= 1; then -= 3)
		} else if (kind == JK_MEM) {
			// (HandleMemSingle: pre-indexed with write-back sets Rn before the access; post-indexed takes Rn as the
			// address and adds the offset after, unless a load wrote Rn)
			const uint32_t off = insn & 0xfff, rn = (insn >> 16) & 0xf, rd = (insn >> 12) & 0xf;
			const int pre = (insn & INSN_SDT_P) != 0, wb = pre && (insn & INSN_SDT_W);
			e_ldr(1, OFS_R(rn));
			if (off)
				e_imm(2, off);
			if (pre && off)
				E((insn & INSN_SDT_U) ? 0xe0811002u : 0xe0411002u);   // add / sub r1, r1, r2
			if (wb)
				e_str(1, OFS_R(rn));
			if (!pre)
				E(0xe1a06001u);   // mov r6, r1 (the address, kept over the call)
			e_mov_r0_c();
			if (insn & INSN_SDT_L) {
				e_access(1, (insn & INSN_SDT_B) != 0);
				e_str(0, OFS_R(rd));
			} else {
				e_ldr(2, OFS_R(rd));
				e_access(0, (insn & INSN_SDT_B) != 0);
			}
			if (!pre && !((insn & INSN_SDT_L) && rd == rn)) {   // post-indexed write-back
				if (off) {
					e_imm(2, off);
					E((insn & INSN_SDT_U) ? 0xe0866002u : 0xe0466002u);   // add / sub r6, r6, r2
				}
				e_str(6, OFS_R(rn));
			}
			// (an interrupt line raised by the access: the interpreter's test after the instruction, then the cycles)
			e_ldrb(0, OFS_PEND);
			E(0xe3500000u);   // cmp r0, #0
			const unsigned b = e_bcond(0x0);   // beq: none
			e_mov_r0_c();
			e_imm(1, p + 4);
			e_call((const void *)jit_irq);
			e_sub_icount((insn & INSN_SDT_L) ? 3 : 2);
			e_imm(0, p);
			e_ret();
			e_patch(b);
			e_cycles((insn & INSN_SDT_L) ? 3 : 2, p + 4);
		} else if (kind == JK_B) {
			uint32_t off = (insn & 0x00ffffffu) << 2;
			off = (off & 0x02000000u) ? (off | 0xfc000000u) : off;
			if (insn & INSN_BL) {
				e_imm(0, p + 4);
				e_str(0, OFS_R(14));
			}
			e_sub_icount(3);
			e_imm(0, p + 8 + off);
			e_str(0, OFS_R(15));
			e_imm(0, p);      // (returned: the branch's pc, for the idle check)
			e_ret();
			end = 1;   // (a conditional one too, after its not-taken path: one basic block a block)
		} else if (kind == JK_BX) {
			e_mov_r0_c();
			e_imm(1, insn & 0xf);
			e_call((const void *)jit_bx);
			e_sub_icount(3);
			e_imm(0, p);      // (returned: for the idle check)
			e_ret();
			end = cond == 0xe;
		} else if (kind == JK_LIT && lit_ok(c, p, insn)) {
			// a literal from the read-only page: its value now (RD32 / RD8 of a mapped page: no side effect)
			const uint32_t off = insn & 0xfff;
			const uint32_t a = ((insn & INSN_SDT_U) ? p + off : p - off) + 8;
			e_imm(0, (insn & INSN_SDT_B) ? RD8(c, a) : RD32(c, a));
			e_str(0, OFS_R((insn >> 12) & 0xf));
			e_cycles(3, p + 4);
		} else {
			e_mov_r0_c();
			e_imm(1, insn);
			e_imm(2, p);
			e_call((const void *)jit_interp);
			E(0xe3500000u);   // cmp r0, #0
			const unsigned b = e_bcond(0x1);   // bne: on to pc + 4
			e_imm(0, p);      // (returned: this instruction's pc, for the idle check)
			e_ret();
			e_patch(b);
			E(0xe3550000u);   // cmp r5, #0
			e_stub(0xd, p + 4);
		}
		if (skip) {   // (the condition failed: one cycle, on to the next)
			const unsigned over = e_bcond(0xe);
			e_patch(skip);
			e_cycles(1, p + 4);
			e_patch(over);
		}
		if (end) {
			if (cond != 0xe)
				e_exit(p + 4);   // (the condition failed: on after the branch, in another block)
			break;
		}
		if (i == JIT_MAX_INSNS - 1)
			e_exit(p + 4);
	}
	e_finish();
	return jit_install();
}

// ---- Thumb blocks ----
static void e_thumb_dp_out(int rd, int setflags)   // the result r0 to Rd, the flags back
{
	if (setflags)
		e_flags_out();
	if (rd >= 0)
		e_str(0, OFS_R(rd));
}

// a literal word of a read-only mapped page (its value cannot change before the code is thrown away)
static int ro_word(arm7_t *c, uint32_t a) { return page_r(c, a & ~3u) && !page_w(c, a & ~3u); }

// a Thumb block (the same rules as jit_build; every Thumb instruction costs 3 cycles)
static jit_block_fn jit_build_thumb(arm7_t *c, uint32_t pc)
{
	const uint8_t *page = page_r(c, pc);
	if (!page || page_w(c, pc))
		return NULL;
	e_begin();
	// (the slice's end: a block runs only when icount covers all of it but its last instruction, else 1 is returned and
	// the interpreter goes on; inside, the count is checked only after what can take more cycles: memory, the
	// interpreter, the hook. The checks' immediates (cycles left after) patched at the end)
	unsigned chk_at[JIT_MAX_INSNS + 1], chk_i[JIT_MAX_INSNS + 1], nchk = 0, n = 0;
	chk_at[nchk] = s_jn;
	chk_i[nchk++] = (unsigned)-1;
	E(0xe3550000u);   // cmp r5, #left
	const unsigned entry_b = e_bcond(0xd);   // ble: return 1
	int hf = 0;   // the host flags hold the guest NZCV
	uint32_t p = pc;
	for (int i = 0; i < JIT_MAX_INSNS; i++, p += 2) {
		if ((p >> ARM7_PAGE_SHIFT) != (pc >> ARM7_PAGE_SHIFT)) {
			e_exit(p);
			break;
		}
		uint16_t op16;
		memcpy(&op16, page + POFS(p), 2);
		const uint32_t op = op16;
		n = (unsigned)i + 1;
		int chk = 0;
		if (c->hook_eat && p == c->hook_pc) {
			e_imm(1, (uint32_t)c->hook_eat);
			E(0xe0455001u);   // sub r5, r5, r1
			chk = 1;
		}
		const uint32_t top = op >> 11;   // (5 bits)
		int done = 1, end = 0;
		if (top <= 2) {   // LSL / LSR / ASR #imm (thumb_exec tg00 / tg01_0)
			const uint32_t n = (op >> 6) & 0x1f;
			if (!hf)
				if (!hf)
					e_flags_in();
			e_ldr(1, OFS_R((op >> 3) & 7));
			E(0xe1b00001u | n << 7 | top << 5);   // movs r0, r1, lsl/lsr/asr #n (#0: none / 32 / 32)
			e_thumb_dp_out((int)(op & 7), 1);
			hf = 1;
		} else if (top == 3) {   // ADD / SUB Rd, Rs, Rn / #imm3
			const uint32_t n = (op >> 6) & 7;
			e_ldr(1, OFS_R((op >> 3) & 7));
			if (op & 0x0400)
				E(((op & 0x0200) ? 0xe2500000u : 0xe2900000u) | 1u << 16 | n);   // subs / adds r0, r1, #n
			else {
				e_ldr(2, OFS_R(n));
				E((op & 0x0200) ? 0xe0510002u : 0xe0910002u);   // subs / adds r0, r1, r2
			}
			e_thumb_dp_out((int)(op & 7), 1);
			hf = 1;
		} else if (top >= 4 && top <= 7) {   // MOV / CMP / ADD / SUB Rd, #imm8
			const uint32_t rd = (op >> 8) & 7, imm = op & 0xff;
			if (top == 4) {
				if (!hf)
				if (!hf)
					e_flags_in();
				E(0xe3b00000u | imm);   // movs r0, #imm (C, V kept)
				e_thumb_dp_out((int)rd, 1);
				hf = 1;
			} else {
				e_ldr(1, OFS_R(rd));
				E((top == 6 ? 0xe2910000u : 0xe2510000u) | imm);   // adds / subs r0, r1, #imm (CMP: subs)
				e_thumb_dp_out(top == 5 ? -1 : (int)rd, 1);
				hf = 1;
			}
		} else if ((op & 0xfc00) == 0x4000 && ((op >> 6) & 0xf) != 0x2 && ((op >> 6) & 0xf) != 0x3 &&
		           ((op >> 6) & 0xf) != 0x4 && ((op >> 6) & 0xf) != 0x5 && ((op >> 6) & 0xf) != 0x6 &&
		           ((op >> 6) & 0xf) != 0x7) {   // ALU Rd, Rs: the ones the hardware does as thumb_alu does
			const uint32_t a = (op >> 6) & 0xf, rd = op & 7, rs = (op >> 3) & 7;
			static const uint32_t host[16] = {
				0xe0110002u, 0xe0310002u, 0, 0, 0, 0, 0, 0,          // ands, eors (r0, r1, r2)
				0xe0110002u, 0xe2720000u, 0xe0510002u, 0xe0910002u,   // tst, negs (rsbs r0, r2, #0), cmp (subs), cmn (adds)
				0xe1910002u, 0xe0100291u, 0xe1d10002u, 0xe1f00002u }; // orrs, muls r0, r1, r2, bics, mvns r0, r2
			if (!hf)
				if (!hf)
					e_flags_in();
			e_ldr(1, OFS_R(rd));
			e_ldr(2, OFS_R(rs));
			E(host[a]);
			const int keep = a != 0x8 && a != 0xa && a != 0xb;   // (TST, CMP, CMN: the flags only)
			e_thumb_dp_out(keep ? (int)rd : -1, 1);
			hf = 1;
		} else if ((op & 0xf800) == 0x4800 && ro_word(c, (p & ~2u) + 4 + ((op & 0xff) << 2))) {
			// LDR Rd, [PC, #imm]: the ROM word now
			e_imm(0, RD32(c, (p & ~2u) + 4 + ((op & 0xff) << 2)));
			e_str(0, OFS_R((op >> 8) & 7));
		} else if ((op & 0xf000) == 0x5000 && (((op >> 9) & 7) == 0 || ((op >> 9) & 7) == 2 || ((op >> 9) & 7) == 4 ||
		           ((op >> 9) & 7) == 6)) {   // STR / STRB / LDR / LDRB Rd, [Rn, Rm]
			const uint32_t k = (op >> 9) & 7;
			e_ldr(1, OFS_R((op >> 3) & 7));
			e_ldr(2, OFS_R((op >> 6) & 7));
			E(0xe0811002u);   // add r1, r1, r2
			if (k >= 4) {
				e_access(1, k == 6);
				chk = 1;
				e_str(0, OFS_R(op & 7));
			} else {
				e_ldr(2, OFS_R(op & 7));
				e_access(0, k == 2);
				chk = 1;
			}
		} else if ((op & 0xe000) == 0x6000) {   // STR / LDR (word: imm5 * 4, byte: imm5) Rd, [Rn, #imm]
			const int byte = (op & 0x1000) != 0;
			const uint32_t off = ((op >> 6) & 0x1f) << (byte ? 0 : 2);
			e_ldr(1, OFS_R((op >> 3) & 7));
			if (off)
				E(0xe2811000u | off);   // add r1, r1, #off (< 128)
			if (op & 0x0800) {
				e_access(1, byte);
				chk = 1;
				e_str(0, OFS_R(op & 7));
			} else {
				e_ldr(2, OFS_R(op & 7));
				e_access(0, byte);
				chk = 1;
			}
		} else if ((op & 0xf000) == 0x9000) {   // STR / LDR Rd, [SP, #imm8 * 4] (the load aligned)
			const uint32_t off = (op & 0xff) << 2;
			e_ldr(1, OFS_R(13));
			if (off) {
				e_imm(2, off);
				E(0xe0811002u);   // add r1, r1, r2
			}
			if (op & 0x0800) {
				E(0xe3c11003u);   // bic r1, r1, #3
				e_access(1, 0);
				chk = 1;
				e_str(0, OFS_R((op >> 8) & 7));
			} else {
				e_ldr(2, OFS_R((op >> 8) & 7));
				e_access(0, 0);
				chk = 1;
			}
		} else if ((op & 0xf000) == 0xa000) {   // ADD Rd, PC / SP, #imm8 * 4
			const uint32_t off = (op & 0xff) << 2;
			if (!(op & 0x0800))
				e_imm(0, ((p + 4) & ~2u) + off);
			else {
				e_ldr(0, OFS_R(13));
				e_imm(2, off);
				E(0xe0800002u);   // add r0, r0, r2
			}
			e_str(0, OFS_R((op >> 8) & 7));
		} else if ((op & 0xff00) == 0xb000) {   // ADD SP, #+-imm7 * 4
			e_ldr(0, OFS_R(13));
			E(((op & 0x80) ? 0xe2400f00u : 0xe2800f00u) | (op & 0x7f));   // sub / add r0, r0, #imm7 << 2
			e_str(0, OFS_R(13));
		} else if ((op & 0xf000) == 0xd000 && ((op >> 8) & 0xf) < 0xe) {   // Bcond
			const uint32_t cond = (op >> 8) & 0xf;
			const int32_t offs = (int8_t)(op & 0xff);
			if (!hf)
				if (!hf)
					e_flags_in();
			const unsigned no = e_bcond(cond ^ 1);
			e_sub_icount(3);
			e_imm(0, p + 4 + (uint32_t)(offs * 2));
			e_str(0, OFS_R(15));
			e_imm(0, p);
			e_ret();
			e_patch(no);
			e_sub_icount(3);
			e_exit(p + 2);   // (a block is one basic block: no copies of the code after a jump target)
			end = 1;
		} else if ((op & 0xf800) == 0xe000) {   // B
			uint32_t offs = (op & 0x07ff) << 1;
			if (offs & 0x800)
				offs |= 0xfffff000u;
			e_sub_icount(3);
			e_imm(0, p + 4 + offs);
			e_str(0, OFS_R(15));
			e_imm(0, p);
			e_ret();
			end = 1;
		} else if ((op & 0xf800) == 0xf000) {   // BL (hi): LR = pc + 4 + offset << 12
			uint32_t addr = (op & 0x07ff) << 12;
			if (addr & 0x400000)
				addr |= 0xff800000u;
			e_imm(0, addr + p + 4);
			e_str(0, OFS_R(14));
		} else if ((op & 0xf800) == 0xf800) {   // BL (lo): pc = (LR & ~1) + offset << 1, LR = pc + 2 | 1
			e_ldr(0, OFS_R(14));
			E(0xe3c00001u);   // bic r0, r0, #1
			e_imm(2, (op & 0x07ff) << 1);
			E(0xe0800002u);   // add r0, r0, r2
			e_str(0, OFS_R(15));
			e_imm(0, (p + 2) | 1);
			e_str(0, OFS_R(14));
			e_sub_icount(3);
			e_imm(0, p);
			e_ret();
			end = 1;
		} else
			done = 0;
		if (end)
			break;
		if (!done) {   // through the interpreter
			e_mov_r0_c();
			e_imm(1, op);
			e_imm(2, p);
			e_call((const void *)jit_tinterp);
			E(0xe3500000u);   // cmp r0, #0
			const unsigned b = e_bcond(0x1);   // bne: on to pc + 2
			e_imm(0, p);
			e_ret();
			e_patch(b);
			chk_at[nchk] = s_jn;
			chk_i[nchk++] = (unsigned)i;
			E(0xe3550000u);   // cmp r5, #left
			e_stub(0xd, p + 2);
			hf = 0;
		} else if (chk) {
			e_sub_icount(3);
			chk_at[nchk] = s_jn;
			chk_i[nchk++] = (unsigned)i;
			E(0xe3550000u);   // cmp r5, #left
			e_stub(0xd, p + 2);
			hf = 0;
		} else
			e_sub_icount(3);
		if (i == JIT_MAX_INSNS - 1)
			e_exit(p + 2);
	}
	for (unsigned k = 0; k < nchk; k++) {   // (r5 > 3 * (left - 1): none of the rest can end the slice before the last)
		const unsigned left = n - 1 - chk_i[k];   // (chk_i -1: all n)
		s_jbuf[chk_at[k]] |= left ? 3 * (left - 1) : 0;
	}
	e_patch(entry_b);
	E(0xe3a00001u);   // mov r0, #1 (not run)
	e_ret();
	e_finish();
	return jit_install();
}

// the block for pc (made the first time); NULL: interpret
// (pc | 1: a Thumb block). A block is made only where the flow came by a jump or the end of a block (build): a slice
// starts wherever the last one stopped, and blocks made there would fill the code memory with copies of the same
// code; from there the interpreter runs up to the next jump
static jit_block_fn jit_lookup(arm7_t *c, uint32_t pc, int build)
{
	const unsigned i = ((pc >> 1) ^ (pc >> 15) ^ (pc >> 27)) & ((1u << JIT_TAB_BITS) - 1);
	for (int w = 0; w < 4; w++)
		if (s_jtab[i][w].pc == pc && s_jtab[i][w].fn)
			return s_jtab[i][w].fn;
	if (!build || !jit_code_init())
		return NULL;
	jit_block_fn f = (pc & 1) ? jit_build_thumb(c, pc & ~1u) : jit_build(c, pc);
	if (f) {   // (into the set's next way)
		const unsigned w = s_jnext[i]++ & 3;
		s_jtab[i][w].pc = pc;
		s_jtab[i][w].fn = f;
	}
	return f;
}

void arm7_jit_reset(void) { jit_flush_all(); }

#else
void arm7_jit_reset(void) {}
#endif
