/*  SH-3 JIT (ADR-043): guest SH-3 code translated to host ARM code (included by sh3.c; ARM hosts with executable
    memory from the frontend, rbx_jit_alloc). The instruction semantics and the cycle, cache and bus timing are the
    interpreter's (sh3.c: MAME 0.289 src/devices/cpu/sh sh.cpp, sh4.cpp, sh3comn.cpp, sh7709s.cpp, BSD-3-Clause,
    copyright-holders: David Haywood, R. Belmont, Jesus Ramos): the same C functions for the timing model, inline only
    its common cases that are the same by construction (a cache hit on the line's way used last; the bus part of a
    not cacheable access to areas 1, 3, 4-6 with no write-back pending).
    This file is part of ReBOX. GPL-2.0-or-later.

    A block is the guest code from a pc up to an unconditional delayed transfer (and its slot), an instruction left to
    the interpreter (RTE, TRAPA, SLEEP, a transfer or LDC SR in a slot), the end of its 4 KB page or SJ_MAX_INSNS
    instructions. A conditional branch not taken goes on in the block, a taken one leaves it. The CV1000 runs its
    program from SDRAM, so code in RAM is translated: the page of a block loses its write table entry (writes reach
    sj_write through slow_w, the interpreter's and the DMA's too) and a write over translated code (4-byte granules)
    throws all blocks away; the block that wrote leaves after that instruction. A page without a read entry (the
    CV1000's idle variable page: reads through the board) is translated from its write entry's memory when the driver
    sets fetch_wr.
    Every instruction is done in the interpreter's order: the pc flush (drc_pc), arg0 of the C helpers, the
    instruction fetch penalty (icache_penalty), the operation with its accesses (the page tables directly, else
    slow_r / slow_w with c->pc and c->icount as the interpreter's SYNC leaves them) each followed by its data_penalty,
    then the interpreter's NEXT: the cycles gathered (pend) counted off at the DRC's update points, the slice's end
    there when icount < 0 and no interrupt is to be tested, the interrupt test after a helper or bus access left one
    (the block leaves, the interpreter's irq_check takes it). The run loop (sj_run) runs a block when the pc has one,
    else the interpreter until its next taken transfer (run(c, 1): JX). pend lives in r6 / c->jpend between them.
    Host registers in a block: r4 = c, r5 = icount, r6 = pend from before the block (not yet counted off), r7 = c->rd,
    r8 = c->wr, r10 = test_irq | flushed << 1 after a call that may change them, r11 = an access's address
    (OPT_RC, ADR-140: the flag in s_sj_irqf, r10 / lr / r6 hold a block's most used guest registers). */

#ifdef SH3_JIT
#include "../../rbx.h"

#define SJ_TAB_BITS 14
#define SJ_MAX_INSNS 64
#define SJ_MAXP 4096          // (pages with translated code)
#define SJ_BUF 8192
#define SJ_BUF_LIMIT 4400     // (no new instruction past this many words)
#define SJ_MAXX 400           // (exits of a block)

typedef uint32_t (*sj_fn)(void);
#define SJ_NONE ((sj_fn)(uintptr_t)1)   // (a table entry: no block at that pc)

static uint32_t *s_sj_code;             // executable (NULL: none yet)
static int s_sj_nocode;
static unsigned s_sj_used, s_sj_size, s_sj_srw;   // words (the first s_sj_srw: the shared routines)
static struct sj_ent { uint32_t pc; sj_fn fn; } (*s_sj_tab)[4];
static uint8_t *s_sj_next;
static uint32_t *s_sj_buf;
static int s_sj_on = -1;                // (-1: RBX_SH3_NOJIT not read yet)
static sh3_t *s_sj_cpu;                 // (the CPU the blocks are made for)
static uint32_t s_sj_flushpc = 1;       // (c->flush_pc the blocks are made with)
static uint32_t s_sj_flushed;           // (set by sj_flush: a running block leaves)
static uint32_t s_sj_xlr;               // (SR_EXIT's lr: the exit's data words)
static uint32_t *s_sj_slot;             // (a chain slot to patch with a jump to the block at c->pc)
static uint32_t s_sj_npg, s_sj_pg[SJ_MAXP];   // (the hooked pages, slot k + 1)
static uint8_t *s_sj_host[SJ_MAXP];          // (their memory: the write entry they had)
static uint8_t (*s_sj_bits)[128];       // (their translated 4-byte granules: data next to code does not throw the blocks away)
static uint32_t s_sj_dump;
static int s_sj_dbg;                    // RBX_SH3_JITDBG: 1 no native operations, 2 no chaining, 4 no native accesses, 8 the timing model by its C functions only
static uint64_t s_sj_us, s_sj_nb, s_sj_ni, s_sj_nflush, s_sj_nwflush, s_sj_runs, s_sj_interp;
static uint64_t s_sj_wh, s_sj_wx, s_sj_wc, s_sj_nfull;   // (RBX_SH3_JITSTAT: words made hot, exit stubs, cold; times full)
static uint32_t s_sj_cnt[H_COUNT];

// the code shape (ADR-130), RBX_SH3_JITOPT = a mask of these (default all; 0 = the code of ADR-043..077; the Vita:
// data folder file "sh3jitold"). Read once, before the first block
#define OPT_UPD   1u    // the count-off and the interrupt test after an access in one test (subs; cmppl r10; bne)
#define OPT_TBASE 2u    // r9 = the calibrated model's tag array: its check of the way used last in 4 instructions
#define OPT_A11   4u    // an access's address computed straight into r11
#define OPT_DYN   8u    // a transfer to a register's address: the next block found in the code (SR_DLOOK)
#define OPT_LIT   16u   // MOV.L / MOV.W @(disp,PC) from the block's page: the value known (its granule marked translated)
#define OPT_TF    32u   // a BT / BF / BT/S / BF/S right after the T bit was set: on the host flags
#define OPT_KREG  128u  // guest registers known at translation (MOV #imm, the literals of OPT_LIT): a JSR / JMP / BRAF /
                        // BSRF to a known address chained as BRA, a load from a known address read from its page directly
#define OPT_PCHK  256u  // an access's two tests before its page in one (rsbs; cmpcs; bcc)
#define OPT_SENT  64u   // an access's slow path that left r10 set makes icount a sentinel: the count-off's test only
#define OPT_RC    512u  // (ADR-140) a block's most used guest registers (R0-R15, SR) in host registers r10, lr, r6
                        // (r6 once the pend from before the block is counted off): loaded at first use, written back
                        // at the exits and before every C call; the interrupt flag (r10 before) in s_sj_irqf
static unsigned s_sj_opt = ~0u;
static int32_t s_sj_stash;   // (OPT_SENT: icount while r5 is the sentinel)
static int s_rc;             // (OPT_RC: the block being made has the register cache)
static uint32_t s_rcl, s_rcd;   // (OPT_RC: its guest registers loaded, dirty)
static uint32_t s_kmask, s_kv[16];   // (OPT_KREG: the known guest registers, below)
#define SJ_OPT(k) ((s_sj_opt & (k)) != 0)
#define SJ_R9 SJ_OPT(OPT_TBASE)
#define SJ_RCF (SJ_OPT(OPT_RC) && SJ_OPT(OPT_A11))   // (OPT_RC: every block has the flag in s_sj_irqf, r10 is free)
#define OPT_FUSE  1024u // (ADR-140) the count-off after an access before its tag check, one branch for both
                        // (subs r5; ...; cmpge tag; bne): the calibrated model, with OPT_RC, TBASE, UPD, SENT
#define OPT_DSITE 2048u // (ADR-140) an OPT_DYN exit's own last target {pc, entry} checked in the stub before SR_DLOOK
#define OPT_SHR   4096u // (ADR-166) the cold paths shared: an access's slow path, the code page store, the calibrated
                        // model's penalty and the update points' exit stubs as routines at the start of the code
                        // memory (the Vita: data folder file "sh3coldold" clears it)
static int s_shr;   // (OPT_SHR and its routines made)
#define SJ_DS_N 16384
static uint32_t (*s_sj_ds)[2];   // (OPT_DSITE: the exits' slots, s_sj_nds used; {1, 0} = none)
static unsigned s_sj_nds;
static int s_fuse_ok, s_fused;   // (OPT_FUSE: the instruction's update point follows its access; it was made there)
static uint32_t s_fcost, s_fnext;   // (its cost and its next pc)
static int s_fdyn;

// RBX_SH3_JITPROF=1 (testing): every block counts its entries; at exit the hottest blocks by entries x guest
// instructions, their host words (hot part / all)
#define SJ_PROF_N 65536
static int s_sj_prof;
static unsigned s_sj_nprof;
static struct sj_profr { uint32_t pc, n, ni, hot, words; uint8_t h[2 * SJ_MAX_INSNS + 4]; } *s_sj_profr;
static uint64_t s_sj_xc[8];   // (sj_run's exits: 0 slice end, 1 on (chainable), 2 on (dynamic), 3 on (other), 4 interrupt; 5: of 2, a block at the target)

// (OPT_DYN) the blocks by pc for the code's own lookup: direct mapped, {pc, where a chained jump enters}; an empty
// entry is {1, SR_EXITD} (no block at an odd pc; a match there still leaves to sj_run)
#define SJ_DT_BITS 12
static struct sj_dent { uint32_t pc, entry; } *s_sj_dtab;
static uint32_t s_sj_dt_empty;
static inline unsigned sj_dt_idx(uint32_t pc) { return ((pc ^ (pc >> 13)) >> 1) & ((1u << SJ_DT_BITS) - 1); }

#define SJ_HOT_BITS 12
#define SJ_HOT_N 4
static uint32_t s_sj_hotn = SJ_HOT_N;   // (RBX_SH3_HOTN: testing)
static struct sj_hot { uint32_t pc, n; } s_sj_hot[1u << SJ_HOT_BITS];
static int s_sj_a7 = -1;
static void sj_tables_clear(void)
{
	memset(s_sj_hot, 0, sizeof(s_sj_hot));
	s_sj_a7 = -1;
	if (s_sj_dtab)
		for (unsigned i = 0; i < (1u << SJ_DT_BITS); i++) {
			s_sj_dtab[i].pc = 1;
			s_sj_dtab[i].entry = s_sj_dt_empty;
		}
	if (s_sj_tab) {
		memset(s_sj_tab, 0, sizeof(*s_sj_tab) << SJ_TAB_BITS);
		memset(s_sj_next, 0, 1u << SJ_TAB_BITS);
	}
	s_sj_used = s_sj_srw;
	s_sj_slot = NULL;
	s_sj_nds = 0;   // (OPT_DSITE: the exits' slots go with their code)
}

// nothing of the CPU touched (sh3_init: it may be gone)
static void sj_forget(void)
{
	if (s_sj_pslot)
		for (uint32_t k = 0; k < s_sj_npg; k++)
			s_sj_pslot[s_sj_pg[k]] = 0;
	if (s_sj_bits)
		memset(s_sj_bits, 0, (size_t)s_sj_npg * sizeof(*s_sj_bits));
	s_sj_npg = 0;
	s_sj_cpu = NULL;
	sj_tables_clear();
}

// all blocks thrown away, the write table as the map made it
static void sj_flush(sh3_t *c)
{
	const unsigned pa = rbx_prof_enter(SH3_PA_INVAL);
	idle_stop(c, 0);   // (the write table as recording found it first)
	if (s_sj_cpu == c && s_sj_pslot)
		for (uint32_t k = 0; k < s_sj_npg; k++)
			c->wr[s_sj_pg[k]] = s_sj_host[k];
	sj_forget();
	s_sj_cpu = c;
	s_sj_flushed = 1;
	s_sj_nflush++;
	rbx_prof_leave(pa);
}

// a write to a page with translated code (a: masked, aligned to size): over a translated granule, the blocks go
static void sj_write(sh3_t *c, uint32_t a, uint32_t v, int size)
{
	const uint32_t pg = PAGE(a), g = (a & 0xfff) >> 2;
	uint8_t *p = s_sj_host[s_sj_pslot[pg] - 1];
	rbx_prof_set(SH3_PA_INVAL);   // (slow_w puts back the caller's)
	if ((s_sj_bits[s_sj_pslot[pg] - 1][g >> 3] >> (g & 7)) & 1) {
		s_sj_nwflush++;
		sj_flush(c);
	}
	if (size == 4) *(uint32_t *)(p + (a & 0xffc)) = v;
	else if (size == 2) *(uint16_t *)(p + ((a & 0xffe) ^ 2)) = (uint16_t)v;
	else p[(a & 0xfff) ^ 3] = (uint8_t)v;
}

/***************************************************************************
    helpers called by the blocks
***************************************************************************/

static uint64_t s_sj_calls[8];   // (RBX_SH3_JITSTAT: icache, dpen, apen, rd, wr, div1, exec)
#define SJ_CNT(k) (s_sj_calls[k]++)
static uint32_t sj_icache(sh3_t *c)
{
	SJ_CNT(0);
	const unsigned pa = rbx_prof_enter(SH3_PA_PENALTY);
	const uint32_t v = icache_penalty(c);
	rbx_prof_leave(pa);
	return v;
}
static uint64_t s_sj_dcls[5];
static uint32_t s_sj_unc[4096];   // (RBX_SH3_JITSTAT: the not cacheable accesses by MB)   // (RBX_SH3_JITSTAT: the C data penalties: on-chip, not cacheable, other way, miss, MRU)
static uint32_t sj_dpen(sh3_t *c, uint32_t a, uint32_t w)
{
	SJ_CNT(1);
	if (s_sj_stat) {
		const uint32_t ad = a < 0xe0000000u ? a : c->arg0, ca = (ad & SH34_AM) / 16;
		int k = a >= 0xe0000000u ? 0 : !is_cacheable(ad) ? 1 : 3;
		for (int i = 0; k == 3 && i < 4; i++)
			if (c->cache[ca & 255][i].tag == ca)
				k = c->cache[ca & 255][i].lru == 3 ? 4 : 2;
		s_sj_dcls[k]++;
		if (k == 1)
			s_sj_unc[(ad >> 20) & 0xfff]++;
	}
	const unsigned pa = rbx_prof_enter(SH3_PA_PENALTY);
	const uint32_t v = data_penalty(c, a, (int)w);
	rbx_prof_leave(pa);
	return v;
}
// the slow accesses (the interpreter's RD*N / WR*N slow paths); a write returns test_irq | flushed << 1
static uint32_t s_sj_rdmb[4096];
#define SJ_RDMB(a) (s_sj_stat ? (void)s_sj_rdmb[(a) >> 20]++ : (void)0)
static uint32_t s_sj_wrmb[4096];
#define SJ_WRMB(a) (s_sj_stat ? (void)s_sj_wrmb[(a) >> 20]++ : (void)0)
// (pc: bit 0 set by a load in a delay slot, ADR-175: c->rnorm for sh3.c's polling loop proof)
static uint32_t sj_rdn(sh3_t *c, uint32_t a, uint32_t pc, int size)
{
	c->pc = pc & ~1u;
	c->rnorm = !(pc & 1);
	const uint32_t v = slow_r(c, a, size);
	c->rnorm = 0;
	return v;
}
static uint32_t sj_rd8(sh3_t *c, uint32_t a, uint32_t pc) { SJ_CNT(3); SJ_RDMB(a); return sj_rdn(c, a, pc, 1); }
static uint32_t sj_rd16(sh3_t *c, uint32_t a, uint32_t pc) { SJ_CNT(3); SJ_RDMB(a); return sj_rdn(c, a & ~1u, pc, 2); }
static uint32_t sj_rd32(sh3_t *c, uint32_t a, uint32_t pc) { SJ_CNT(3); SJ_RDMB(a); return sj_rdn(c, a & ~3u, pc, 4); }
static uint32_t sj_wr8(sh3_t *c, uint32_t a, uint32_t v, uint32_t pc)
{
	SJ_CNT(4);
	SJ_WRMB(a);
	c->pc = pc;
	slow_w(c, a, v & 0xff, 1);
	return c->test_irq | s_sj_flushed << 1;
}
static uint32_t sj_wr16(sh3_t *c, uint32_t a, uint32_t v, uint32_t pc)
{
	SJ_CNT(4);
	SJ_WRMB(a);
	c->pc = pc;
	slow_w(c, a & ~1u, v & 0xffff, 2);
	return c->test_irq | s_sj_flushed << 1;
}
static uint32_t sj_wr32(sh3_t *c, uint32_t a, uint32_t v, uint32_t pc)
{
	SJ_CNT(4);
	SJ_WRMB(a);
	c->pc = pc;
	slow_w(c, a & ~3u, v, 4);
	return c->test_irq | s_sj_flushed << 1;
}
static void sj_div1(sh3_t *c, uint32_t op)
{
	SJ_CNT(5);
	const unsigned pa = rbx_prof_enter(SH3_PA_HELPER);
	op_div1(c, (op >> 4) & 15, (op >> 8) & 15);
	rbx_prof_leave(pa);
}

// an instruction's operation as the interpreter's label does it (c->pc = pc, c->icount as SYNC leaves them; the
// timing before it and the cycles after it are the block's). Returns test_irq | flushed << 1
#define XN ((op >> 8) & 15)
#define XM ((op >> 4) & 15)
#define XRN c->r[XN]
#define XRM c->r[XM]
#define XSETT(cond) do { if (cond) c->sr |= SH_T; else c->sr &= ~SH_T; } while (0)
#define XRD8N(v, addr) do { uint32_t a_ = (addr); const uint8_t *p_ = FASTP(c->rd, a_); \
	(v) = p_ ? p_[(a_ & 0xfff) ^ 3] : slow_r(c, a_, 1); } while (0)
#define XRD16N(v, addr) do { uint32_t a_ = (addr) & ~1u; const uint8_t *p_ = FASTP(c->rd, a_); \
	(v) = p_ ? *(const uint16_t *)(p_ + ((a_ & 0xffe) ^ 2)) : slow_r(c, a_, 2); } while (0)
#define XRD32N(v, addr) do { uint32_t a_ = (addr) & ~3u; const uint8_t *p_ = FASTP(c->rd, a_); \
	(v) = p_ ? *(const uint32_t *)(p_ + (a_ & 0xffc)) : slow_r(c, a_, 4); } while (0)
#define XWR8N(addr, v) do { uint32_t a_ = (addr), v_ = (v); uint8_t *p_ = FASTP(c->wr, a_); \
	if (p_) p_[(a_ & 0xfff) ^ 3] = (uint8_t)v_; else slow_w(c, a_, v_ & 0xff, 1); } while (0)
#define XWR16N(addr, v) do { uint32_t a_ = (addr) & ~1u, v_ = (v); uint8_t *p_ = FASTP(c->wr, a_); \
	if (p_) *(uint16_t *)(p_ + ((a_ & 0xffe) ^ 2)) = (uint16_t)v_; else slow_w(c, a_, v_ & 0xffff, 2); } while (0)
#define XWR32N(addr, v) do { uint32_t a_ = (addr) & ~3u, v_ = (v); uint8_t *p_ = FASTP(c->wr, a_); \
	if (p_) *(uint32_t *)(p_ + (a_ & 0xffc)) = v_; else slow_w(c, a_, v_, 4); } while (0)
#define XRD8(v, addr) do { uint32_t pa_ = (addr); XRD8N(v, pa_); c->icount -= (int32_t)data_penalty(c, pa_, 0); } while (0)
#define XRD16(v, addr) do { uint32_t pa_ = (addr); XRD16N(v, pa_); c->icount -= (int32_t)data_penalty(c, pa_, 0); } while (0)
#define XRD32(v, addr) do { uint32_t pa_ = (addr); XRD32N(v, pa_); c->icount -= (int32_t)data_penalty(c, pa_, 0); } while (0)
#define XWR8(addr, v) do { uint32_t pa_ = (addr); XWR8N(pa_, v); c->icount -= (int32_t)data_penalty(c, pa_, 1); } while (0)
#define XWR16(addr, v) do { uint32_t pa_ = (addr); XWR16N(pa_, v); c->icount -= (int32_t)data_penalty(c, pa_, 1); } while (0)
#define XWR32(addr, v) do { uint32_t pa_ = (addr); XWR32N(pa_, v); c->icount -= (int32_t)data_penalty(c, pa_, 1); } while (0)

SH3_INLINE uint32_t sj_exec_body(sh3_t *c, uint32_t op, uint32_t h, uint32_t pc)
{
	c->pc = pc;
	SJ_CNT(6);
	if (s_sj_stat)
		s_sj_cnt[h]++;
	switch (h) {
	case H_NOP: break;
	case H_STC_SR: XRN = c->sr; break;
	case H_STC_GBR: XRN = c->gbr; break;
	case H_STC_VBR: XRN = c->vbr; break;
	case H_STC_SSR: XRN = c->ssr; break;
	case H_STC_SPC: XRN = c->spc; break;
	case H_STC_SGR: XRN = c->sgr; break;
	case H_STC_DBR: XRN = c->dbr; break;
	case H_STC_RBANK: XRN = c->rbnk[(c->sr & SH_RB) ? 0 : 1][XM & 7]; break;
	case H_STS_FPUL: XRN = c->fpul; break;
	case H_STS_FPSCR: XRN = c->fpscr & 0x003fffff; break;
	case H_MOVB_S0: XWR8(XRN + c->r[0], XRM); break;
	case H_MOVW_S0: XWR16(XRN + c->r[0], XRM); break;
	case H_MOVL_S0: XWR32(XRN + c->r[0], XRM); break;
	case H_MULL: c->macl = XRN * XRM; break;
	case H_CLRT: c->sr &= ~SH_T; break;
	case H_SETT: c->sr |= SH_T; break;
	case H_CLRS: c->sr &= ~SH_S; break;
	case H_SETS: c->sr |= SH_S; break;
	case H_CLRMAC: c->mach = 0; c->macl = 0; break;
	case H_DIV0U: c->sr &= ~(SH_M | SH_Q | SH_T); break;
	case H_MOVT: XRN = c->sr & SH_T; break;
	case H_STS_MACH: XRN = c->mach; break;
	case H_STS_MACL: XRN = c->macl; break;
	case H_STS_PR: XRN = c->pr; break;
	case H_MOVCAL: XWR32(XRN, c->r[0]); break;
	case H_MOVB_L0: { uint32_t v; XRD8(v, XRM + c->r[0]); XRN = SEXT8(v); } break;
	case H_MOVW_L0: { uint32_t v; XRD16(v, XRM + c->r[0]); XRN = SEXT16(v); } break;
	case H_MOVL_L0: { uint32_t v; XRD32(v, XRM + c->r[0]); XRN = v; } break;
	case H_MACL: op_mac_l(c, XM, XN); break;
	case H_MOVL_S4: XWR32(XRN + (op & 15) * 4, XRM); break;
	case H_MOVB_S: XWR8(XRN, XRM); break;
	case H_MOVW_S: XWR16(XRN, XRM); break;
	case H_MOVL_S: XWR32(XRN, XRM); break;
	case H_MOVB_M: { uint32_t d = XRM; XRN -= 1; XWR8(XRN, d); } break;
	case H_MOVW_M: { uint32_t d = XRM; XRN -= 2; XWR16(XRN, d); } break;
	case H_MOVL_M: { uint32_t d = XRM; XRN -= 4; XWR32(XRN, d); } break;
	case H_DIV0S: {
		uint32_t sr = c->sr & ~(SH_Q | SH_M | SH_T);
		if (XRN & 0x80000000) sr |= SH_Q;
		if (XRM & 0x80000000) sr |= SH_M;
		if ((XRM ^ XRN) & 0x80000000) sr |= SH_T;
		c->sr = sr;
		break;
	}
	case H_TST: XSETT((XRN & XRM) == 0); break;
	case H_AND: XRN &= XRM; break;
	case H_XOR: XRN ^= XRM; break;
	case H_OR: XRN |= XRM; break;
	case H_CMPSTR: {
		uint32_t t = XRN ^ XRM;
		if ((t >> 24) && ((t >> 16) & 0xff) && ((t >> 8) & 0xff) && (t & 0xff)) c->sr &= ~SH_T;
		else c->sr |= SH_T;
		break;
	}
	case H_XTRCT: XRN = (XRN >> 16) | (XRM << 16); break;
	case H_MULU: c->macl = (uint32_t)(uint16_t)XRN * (uint16_t)XRM; break;
	case H_MULS: c->macl = (uint32_t)((int32_t)(int16_t)XRN * (int16_t)XRM); break;
	case H_CMPEQ: XSETT(XRN == XRM); break;
	case H_CMPHS: XSETT(XRN >= XRM); break;
	case H_CMPGE: XSETT((int32_t)XRN >= (int32_t)XRM); break;
	case H_DIV1: op_div1(c, XM, XN); break;
	case H_DMULU: { uint64_t p = (uint64_t)XRN * XRM; c->mach = (uint32_t)(p >> 32); c->macl = (uint32_t)p; } break;
	case H_CMPHI: XSETT(XRN > XRM); break;
	case H_CMPGT: XSETT((int32_t)XRN > (int32_t)XRM); break;
	case H_SUB: XRN -= XRM; break;
	case H_SUBC: {
		uint32_t tmp1 = XRN - XRM;
		uint32_t tmp0 = XRN;
		XRN = tmp1 - (c->sr & SH_T);
		XSETT(tmp0 < tmp1);
		if (tmp1 < XRN) c->sr |= SH_T;
		break;
	}
	case H_SUBV: {
		int32_t dest = (XRN >> 31) & 1;
		int32_t src = ((XRM >> 31) & 1) + dest;
		XRN -= XRM;
		int32_t ans = ((XRN >> 31) & 1) + dest;
		XSETT(src == 1 && ans == 1);
		break;
	}
	case H_ADD: XRN += XRM; break;
	case H_DMULS: {
		int64_t p = (int64_t)(int32_t)XRN * (int32_t)XRM;
		c->mach = (uint32_t)((uint64_t)p >> 32);
		c->macl = (uint32_t)p;
		break;
	}
	case H_ADDC: {
		uint32_t tmp1 = XRN + XRM;
		uint32_t tmp0 = XRN;
		XRN = tmp1 + (c->sr & SH_T);
		XSETT(tmp0 > tmp1);
		if (tmp1 > XRN) c->sr |= SH_T;
		break;
	}
	case H_ADDV: {
		int32_t dest = (XRN >> 31) & 1;
		int32_t src = ((XRM >> 31) & 1) + dest;
		XRN += XRM;
		int32_t ans = ((XRN >> 31) & 1) + dest;
		if (src != 1) XSETT(ans == 1); else c->sr &= ~SH_T;
		break;
	}
	case H_SHLL: c->sr = (c->sr & ~SH_T) | ((XRN >> 31) & SH_T); XRN <<= 1; break;
	case H_SHLR: c->sr = (c->sr & ~SH_T) | (XRN & SH_T); XRN >>= 1; break;
	case H_SHAR: c->sr = (c->sr & ~SH_T) | (XRN & SH_T); XRN = (uint32_t)((int32_t)XRN >> 1); break;
	case H_STSM_MACH: XRN -= 4; XWR32(XRN, c->mach); break;
	case H_STSM_MACL: XRN -= 4; XWR32(XRN, c->macl); break;
	case H_STSM_PR: XRN -= 4; XWR32(XRN, c->pr); break;
	case H_STCM_SR: XRN -= 4; XWR32(XRN, c->sr); break;
	case H_STCM_GBR: XRN -= 4; XWR32(XRN, c->gbr); break;
	case H_STCM_VBR: XRN -= 4; XWR32(XRN, c->vbr); break;
	case H_ROTL: c->sr = (c->sr & ~SH_T) | ((XRN >> 31) & SH_T); XRN = (XRN << 1) | (XRN >> 31); break;
	case H_ROTR: c->sr = (c->sr & ~SH_T) | (XRN & SH_T); XRN = (XRN >> 1) | (XRN << 31); break;
	case H_ROTCL: {
		uint32_t t = (XRN >> 31) & SH_T;
		XRN = (XRN << 1) | (c->sr & SH_T);
		c->sr = (c->sr & ~SH_T) | t;
		break;
	}
	case H_ROTCR: {
		uint32_t t = (c->sr & SH_T) << 31;
		XSETT(XRN & SH_T);
		XRN = (XRN >> 1) | t;
		break;
	}
	case H_LDSM_MACH: { uint32_t v; XRD32(v, XRN); c->mach = v; XRN += 4; } break;
	case H_LDSM_MACL: { uint32_t v; XRD32(v, XRN); c->macl = v; XRN += 4; } break;
	case H_LDSM_PR: { uint32_t v; XRD32(v, XRN); c->pr = v; XRN += 4; } break;
	case H_LDCM_GBR: { uint32_t v; XRD32(v, XRN); c->gbr = v; XRN += 4; } break;
	case H_LDCM_VBR: { uint32_t v; XRD32(v, XRN); c->vbr = v; XRN += 4; } break;
	case H_SHLL2: XRN <<= 2; break;
	case H_SHLL8: XRN <<= 8; break;
	case H_SHLL16: XRN <<= 16; break;
	case H_SHLR2: XRN >>= 2; break;
	case H_SHLR8: XRN >>= 8; break;
	case H_SHLR16: XRN >>= 16; break;
	case H_LDS_MACH: c->mach = XRN; break;
	case H_LDS_MACL: c->macl = XRN; break;
	case H_LDS_PR: c->pr = XRN; break;
	case H_TAS: {
		uint32_t ea = XRN, t;
		XRD8(t, ea);
		XSETT(t == 0);
		XWR8(ea, t | 0x80);
		break;
	}
	case H_LDC_GBR: c->gbr = XRN; break;
	case H_LDC_VBR: c->vbr = XRN; break;
	case H_MACW: op_mac_w(c, XM, XN); break;
	case H_DT: XRN--; XSETT(XRN == 0); break;
	case H_CMPPZ: XSETT((int32_t)XRN >= 0); break;
	case H_CMPPL: XSETT((int32_t)XRN > 0); break;
	case H_SHAD: {
		uint32_t m = c->r[XM];
		if (!(m & 0x80000000)) XRN = XRN << (m & 0x1f);
		else if (!(m & 0x1f)) XRN = (XRN & 0x80000000) ? 0xffffffff : 0;
		else XRN = (uint32_t)((int32_t)XRN >> ((~m & 0x1f) + 1));
		break;
	}
	case H_SHLD: {
		uint32_t m = c->r[XM];
		if (!(m & 0x80000000)) XRN = XRN << (m & 0x1f);
		else if (!(m & 0x1f)) XRN = 0;
		else XRN = XRN >> ((~m & 0x1f) + 1);
		break;
	}
	case H_LDC_RBANK: c->rbnk[(c->sr & SH_RB) ? 0 : 1][XM & 7] = XRN; break;
	case H_STCM_RBANK: { uint32_t d = c->rbnk[(c->sr & SH_RB) ? 0 : 1][XM & 7]; XRN -= 4; XWR32N(XRN, d); } break;
	case H_LDCM_RBANK: { uint32_t v, m = XM; XRD32N(v, XRN); c->rbnk[(c->sr & SH_RB) ? 0 : 1][m & 7] = v; XRN += 4; } break;
	case H_STCM_SGR: XRN -= 4; XWR32N(XRN, c->sgr); break;
	case H_STCM_SSR: XRN -= 4; XWR32N(XRN, c->ssr); break;
	case H_LDCM_SSR: { uint32_t v; XRD32N(v, XRN); c->ssr = v; XRN += 4; } break;
	case H_LDC_SSR: c->ssr = XRN; break;
	case H_STCM_SPC: XRN -= 4; XWR32N(XRN, c->spc); break;
	case H_LDCM_SPC: { uint32_t v; XRD32N(v, XRN); c->spc = v; XRN += 4; } break;
	case H_LDC_SPC: c->spc = XRN; break;
	case H_STSM_FPUL: XRN -= 4; XWR32N(XRN, c->fpul); break;
	case H_LDSM_FPUL: { uint32_t v; XRD32N(v, XRN); c->fpul = v; XRN += 4; } break;
	case H_LDS_FPUL: c->fpul = XRN; break;
	case H_STSM_FPSCR: XRN -= 4; XWR32N(XRN, c->fpscr & 0x003fffff); break;
	case H_LDSM_FPSCR: { uint32_t v; XRD32N(v, XRN); c->fpscr = v & 0x003fffff; XRN += 4; } break;
	case H_LDS_FPSCR: c->fpscr = XRN & 0x003fffff; break;
	case H_STCM_DBR: XRN -= 4; XWR32N(XRN, c->dbr); break;
	case H_LDCM_DBR: { uint32_t v; XRD32N(v, XRN); c->dbr = v; XRN += 4; } break;
	case H_LDC_DBR: c->dbr = XRN; break;
	case H_MOVL_L4: { uint32_t v; XRD32(v, XRM + (op & 15) * 4); XRN = v; } break;
	case H_MOVB_L: { uint32_t v; XRD8(v, XRM); XRN = SEXT8(v); } break;
	case H_MOVW_L: { uint32_t v; XRD16(v, XRM); XRN = SEXT16(v); } break;
	case H_MOVL_L: { uint32_t v; XRD32(v, XRM); XRN = v; } break;
	case H_MOV: XRN = XRM; break;
	case H_MOVB_P: { uint32_t m = XM, n = XN, v; XRD8(v, c->r[m]); c->r[n] = SEXT8(v); if (n != m) c->r[m] += 1; } break;
	case H_MOVW_P: { uint32_t m = XM, n = XN, v; XRD16(v, c->r[m]); c->r[n] = SEXT16(v); if (n != m) c->r[m] += 2; } break;
	case H_MOVL_P: { uint32_t m = XM, n = XN, v; XRD32(v, c->r[m]); c->r[n] = v; if (n != m) c->r[m] += 4; } break;
	case H_NOT: XRN = ~XRM; break;
	case H_SWAPB: { uint32_t v = XRM; XRN = (v & 0xffff0000) | ((v & 0xff) << 8) | ((v >> 8) & 0xff); } break;
	case H_SWAPW: { uint32_t v = XRM; XRN = (v << 16) | (v >> 16); } break;
	case H_NEGC: {
		uint32_t t = XRM;
		XRN = 0u - t - (c->sr & SH_T);
		XSETT(t || (c->sr & SH_T));
		break;
	}
	case H_NEG: XRN = 0u - XRM; break;
	case H_EXTUB: XRN = XRM & 0xff; break;
	case H_EXTUW: XRN = XRM & 0xffff; break;
	case H_EXTSB: XRN = SEXT8(XRM); break;
	case H_EXTSW: XRN = SEXT16(XRM); break;
	case H_ADDI: XRN += SEXT8(op); break;
	case H_MOVB_S4R0: XWR8(XRM + (op & 15), c->r[0]); break;
	case H_MOVW_S4R0: XWR16(XRM + (op & 15) * 2, c->r[0]); break;
	case H_MOVB_L4R0: { uint32_t v; XRD8(v, XRM + (op & 15)); c->r[0] = SEXT8(v); } break;
	case H_MOVW_L4R0: { uint32_t v; XRD16(v, XRM + (op & 15) * 2); c->r[0] = SEXT16(v); } break;
	case H_CMPIM: XSETT(c->r[0] == SEXT8(op)); break;
	case H_MOVW_I: { uint32_t v; XRD16(v, pc + (op & 0xff) * 2 + 2); XRN = SEXT16(v); } break;
	case H_MOVB_SG: XWR8(c->gbr + (op & 0xff), c->r[0]); break;
	case H_MOVW_SG: XWR16(c->gbr + (op & 0xff) * 2, c->r[0]); break;
	case H_MOVL_SG: XWR32(c->gbr + (op & 0xff) * 4, c->r[0]); break;
	case H_MOVB_LG: { uint32_t v; XRD8(v, c->gbr + (op & 0xff)); c->r[0] = SEXT8(v); } break;
	case H_MOVW_LG: { uint32_t v; XRD16(v, c->gbr + (op & 0xff) * 2); c->r[0] = SEXT16(v); } break;
	case H_MOVL_LG: { uint32_t v; XRD32(v, c->gbr + (op & 0xff) * 4); c->r[0] = v; } break;
	case H_MOVA: c->r[0] = ((pc + 2) & ~3u) + (op & 0xff) * 4; break;
	case H_TSTI: XSETT(((op & 0xff) & c->r[0]) == 0); break;
	case H_ANDI: c->r[0] &= op & 0xff; break;
	case H_XORI: c->r[0] ^= op & 0xff; break;
	case H_ORI: c->r[0] |= op & 0xff; break;
	case H_TSTB: { uint32_t v; XRD8(v, c->gbr + c->r[0]); XSETT(((op & 0xff) & v) == 0); } break;
	case H_ANDB: { uint32_t ea = c->gbr + c->r[0], v; XRD8(v, ea); XWR8(ea, (op & 0xff) & v); } break;
	case H_XORB: { uint32_t ea = c->gbr + c->r[0], v; XRD8(v, ea); XWR8(ea, v ^ (op & 0xff)); } break;
	case H_ORB: { uint32_t ea = c->gbr + c->r[0], v; XRD8(v, ea); XWR8(ea, v | (op & 0xff)); } break;
	case H_MOVL_I: { uint32_t v; XRD32(v, ((pc + 2) & ~3u) + (op & 0xff) * 4); XRN = v; } break;
	case H_MOVI: XRN = SEXT8(op); break;
	default: break;   // (the transfers and the instructions left to the interpreter: never here)
	}
	return c->test_irq | s_sj_flushed << 1;
}
static uint32_t sj_exec(sh3_t *c, uint32_t op, uint32_t h, uint32_t pc)
{
	const unsigned pa = rbx_prof_enter(SH3_PA_HELPER);
	const uint32_t r = sj_exec_body(c, op, h, pc);
	rbx_prof_leave(pa);
	return r;
}

// the instructions left to the interpreter (they change the flow and test the interrupts as the DRC's helpers do, or
// sleep): a block ends before them. LDC SR / LDC.L SR: in a block, not in a delay slot
static int sj_interp_only(int h)
{
	return h == H_RTE || h == H_TRAPA || h == H_SLEEP;
}
static int sj_slot_ok(int h)
{
	return !sj_interp_only(h) && h != H_LDC_SR && h != H_LDCM_SR && h != H_BT && h != H_BF;
}
static int sj_delayed(int h)
{
	return h == H_BRA || h == H_BSR || h == H_BTS || h == H_BFS || h == H_RTS || h == H_JMP || h == H_JSR ||
	       h == H_BRAF || h == H_BSRF;
}
// the operations that may access memory or change test_irq (c->icount synced around them, r10 after)
static int sj_touches(int h)
{
	return op_mem(h) || h == H_TSTB || h == H_ANDB || h == H_XORB || h == H_ORB || h == H_MACL || h == H_MACW ||
	       h == H_STCM_RBANK || h == H_LDCM_RBANK || h == H_STCM_SGR || h == H_STCM_SSR || h == H_LDCM_SSR ||
	       h == H_STCM_SPC || h == H_LDCM_SPC || h == H_STSM_FPUL || h == H_LDSM_FPUL || h == H_STSM_FPSCR ||
	       h == H_LDSM_FPSCR || h == H_STCM_DBR || h == H_LDCM_DBR;
}

/***************************************************************************
    emitter
***************************************************************************/

// two buffers: the block's code (hot) and its slow paths (cold, put after the exits at the end); E writes to the
// current one, e_bcond / e_patch work inside it
#define SJ_COLD 4096
static uint32_t s_cold[SJ_COLD];
static unsigned s_n, s_nc;
static uint32_t *s_cb = NULL;
static unsigned *s_cn = &s_n, s_ccap = SJ_BUF;
static int s_full;
static void E(uint32_t w)
{
	if (*s_cn < s_ccap)
		s_cb[(*s_cn)++] = w;
	else
		s_full = 1;   // (the block is thrown away)
}
#define SJ_XREF 1024
static unsigned s_hx_at[SJ_XREF], s_hx_to[SJ_XREF], s_nhx;   // hot branches to cold positions
static unsigned s_cx_at[SJ_XREF], s_cx_to[SJ_XREF], s_ncx;   // cold branches back to hot positions
static unsigned s_hot_sites[8], s_nhot;
// a hot branch to the next cold path (cond)
static void e_tocold(uint32_t cond)
{
	if (s_nhot < 8) {
		s_hot_sites[s_nhot++] = s_n;
		E(cond << 28 | 0x0a000000u);
	} else
		s_full = 1;
}
// the cold path starts: the branches e_tocold made since the last one go here
static void e_cold(void)
{
	for (unsigned k = 0; k < s_nhot; k++)
		if (s_nhx < SJ_XREF) {
			s_hx_at[s_nhx] = s_hot_sites[k];
			s_hx_to[s_nhx++] = s_nc;
		} else
			s_full = 1;
	s_nhot = 0;
	s_cb = s_cold;
	s_cn = &s_nc;
	s_ccap = SJ_COLD;
}
// the cold path ends: back to the hot code where it is now
static void e_hot(void)
{
	if (s_ncx < SJ_XREF) {
		s_cx_at[s_ncx] = s_nc;
		s_cx_to[s_ncx++] = s_n;
	} else
		s_full = 1;
	E(0xea000000u);
	s_cb = s_sj_buf;
	s_cn = &s_n;
	s_ccap = SJ_BUF;
}
// (in the cold code) a conditional branch back to the hot code where it is now; the cold path goes on
static void e_hot_if(uint32_t cond)
{
	if (s_ncx < SJ_XREF) {
		s_cx_at[s_ncx] = s_nc;
		s_cx_to[s_ncx++] = s_n;
	} else
		s_full = 1;
	E(cond << 28 | 0x0a000000u);
}
// (e_finish's end: the cold code after the hot, the cross branches patched)
static unsigned s_cold_base;   // (where the cold code went: e_finish moves the cold bl sites)
static void e_cold_append(void)
{
	const unsigned h = s_n;
	s_cold_base = h;
	if (h + s_nc > SJ_BUF) {
		s_full = 1;
		return;
	}
	memcpy(s_sj_buf + h, s_cold, s_nc * 4);
	s_n += s_nc;
	for (unsigned k = 0; k < s_nhx; k++)
		s_sj_buf[s_hx_at[k]] |= (h + s_hx_to[k] - s_hx_at[k] - 2) & 0x00ffffffu;
	for (unsigned k = 0; k < s_ncx; k++)
		s_sj_buf[h + s_cx_at[k]] |= (s_cx_to[k] - (h + s_cx_at[k]) - 2) & 0x00ffffffu;
}
#define COFS(f) ((uint32_t)offsetof(sh3_t, f))
#define RO(n) (4u * (uint32_t)(n))
static void e_ldr(int rt, uint32_t off) { E(0xe5940000u | (uint32_t)rt << 12 | off); }    // ldr rt, [r4, #off]
static void e_str(int rt, uint32_t off) { E(0xe5840000u | (uint32_t)rt << 12 | off); }    // str rt, [r4, #off]
// a field past 4 KB of c: r12 = r4 + the 4 KB part (an ARM immediate below 64 KB)
static void e_strf(int rt, uint32_t off)
{
	if (off < 4096) { e_str(rt, off); return; }
	E(0xe284ca00u | ((off >> 12) & 0xf));   // add r12, r4, #(off & 0xf000)
	E(0xe58c0000u | (uint32_t)rt << 12 | (off & 0xfff));
}
static void e_ldrf(int rt, uint32_t off)
{
	if (off < 4096) { e_ldr(rt, off); return; }
	E(0xe284ca00u | ((off >> 12) & 0xf));
	E(0xe59c0000u | (uint32_t)rt << 12 | (off & 0xfff));
}
static void e_mov(int rd, int rm) { E(0xe1a00000u | (uint32_t)rd << 12 | (uint32_t)rm); }
static uint32_t ubfx(int rd, int rn, int lsb, int w) { return 0xe7e00050u | (uint32_t)(w - 1) << 16 | (uint32_t)rd << 12 | (uint32_t)lsb << 7 | (uint32_t)rn; }
static uint32_t bfi(int rd, int rn, int lsb, int w) { return 0xe7c00010u | (uint32_t)(lsb + w - 1) << 16 | (uint32_t)rd << 12 | (uint32_t)lsb << 7 | (uint32_t)rn; }
static void e_imm(int rd, uint32_t v)   // movw / movt
{
	E(0xe3000000u | (v & 0xf000u) << 4 | (uint32_t)rd << 12 | (v & 0xfffu));
	if (v >> 16)
		E(0xe3400000u | (v >> 16 & 0xf000u) << 4 | (uint32_t)rd << 12 | (v >> 16 & 0xfffu));
}
static void e_imm2(int rd, uint32_t v)  // movw + movt (a fixed length)
{
	E(0xe3000000u | (v & 0xf000u) << 4 | (uint32_t)rd << 12 | (v & 0xfffu));
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
static void e_addi(int rd, int rn, uint32_t v)   // rd = rn + v (r12 for a large v)
{
	int k;
	if (!v) {
		if (rd != rn)
			e_mov(rd, rn);
	} else if ((k = arm_imm(v)) >= 0)
		E(0xe2800000u | (uint32_t)rn << 16 | (uint32_t)rd << 12 | (uint32_t)k);
	else if ((k = arm_imm(0u - v)) >= 0)
		E(0xe2400000u | (uint32_t)rn << 16 | (uint32_t)rd << 12 | (uint32_t)k);
	else {
		e_imm(12, v);
		E(0xe080000cu | (uint32_t)rn << 16 | (uint32_t)rd << 12);
	}
}
// a data processing op with an immediate operand (opc: 0 and, 1 eor, 12 orr, 14 bic, 10 cmp, 8 tst ...), r12 if the
// immediate has no ARM form
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
static void e_rc_pre(void);
static void e_rc_post(void);
static void e_call(const void *fn)   // blx fn (r0-r3 the arguments; OPT_RC: the dirty guest registers written back first)
{
	e_rc_pre();
	e_imm(12, (uint32_t)(uintptr_t)fn);
	E(0xe12fff3cu);
	e_rc_post();
}
static unsigned e_bcond(uint32_t cond) { E(cond << 28 | 0x0a000000u); return *s_cn - 1; }
static void e_patch(unsigned at) { s_cb[at] |= ((*s_cn - at - 2) & 0x00ffffffu); }
static void e_bufs_reset(void)
{
	s_cb = s_sj_buf;
	s_cn = &s_n;
	s_ccap = SJ_BUF;
	s_n = s_nc = s_nhx = s_ncx = s_nhot = 0;
}

// the exits (stubs after the block's code)
// (XK_SPLIT, OPT_UPD: icount < 0 -> exit next, else exit pend; XK_SPLIT2, OPT_SENT: r10 0 -> exit next, else icount
// back from the sentinel, then as XK_SPLIT)
// (OPT_SHR: one stub for an update point's exits, its routine decides: XK_G2 = XK_SPLIT2 and its exits, XK_G1 =
// XK_SPLIT and its, XK_SQ = an XK_SLICE with the interrupt's exit, XK_EQ = an XK_EXIT code 1 with it)
enum { XK_SLICE, XK_EXIT, XK_SPLIT, XK_SPLIT2, XK_G2, XK_G1, XK_SQ, XK_EQ };
static struct { uint8_t kind, dyn, pdyn, chain, code, q, look; uint16_t qto; uint32_t next, pend, wb; } s_x[SJ_MAXX];
static unsigned s_nx;
static unsigned s_site_at[SJ_MAXX * 2], s_site_to[SJ_MAXX * 2], s_nsite;
static unsigned e_newx(int kind, uint32_t next, int dyn, uint32_t pend, int pdyn, int code, int chain)
{
	if (s_nx >= SJ_MAXX - 1) {
		s_full = 1;
		return s_nx - 1;
	}
	s_x[s_nx].kind = (uint8_t)kind;
	s_x[s_nx].next = next;
	s_x[s_nx].dyn = (uint8_t)dyn;
	s_x[s_nx].pend = pend;
	s_x[s_nx].pdyn = (uint8_t)pdyn;
	s_x[s_nx].code = (uint8_t)code;
	s_x[s_nx].chain = (uint8_t)(chain && !dyn && !(s_sj_dbg & 2));
	// (OPT_DYN: the end of a transfer to a register's address, on at c->pc: the next block looked up in the code)
	s_x[s_nx].look = (uint8_t)(chain && dyn && kind == XK_EXIT && code == 1 && SJ_OPT(OPT_DYN) && !(s_sj_dbg & 2));
	s_x[s_nx].q = 0;
	s_x[s_nx].wb = s_rc ? s_rcl & s_rcd : 0;   // (OPT_RC: the dirty guest registers its stub writes back)
	return s_nx++;
}
static void e_bto(uint32_t cond, unsigned to)
{
	if (s_nsite >= SJ_MAXX * 2) {
		s_full = 1;
		return;
	}
	s_site_at[s_nsite] = s_cn == &s_nc ? s_nc | 0x80000000u : s_n;   // (a cold site: patched after e_cold_append)
	s_site_to[s_nsite++] = to;
	E(cond << 28 | 0x0a000000u);
}

// the shared routines at the start of the code memory; bl to one patched at install
// SR_DPEN0/1: data_penalty(c, r11, write), r5 -= it; SR_UNC0/1: its not cacheable cases inline (else SR_DPEN*);
// SR_APEN: access_penalty(c, r0, 0), r5 -= it (r0-r3, r12 clobbered)
// SR_DP0/1: data_penalty inline (else SR_UNC* / SR_DPEN*); SR_IC: icache_penalty inline (else SR_APEN / the C function)
// (OPT_SHR, ADR-166) SR_RD0 + 2 * (0 byte, 1 word, 2 long) + sign: an access's slow load (bl; r11 the address, r2 the
// pc, the value in r0); SR_WF0 + size index: the store to a page with translated code outside its granules (r12
// link: r11 the address, r1 the value; Z set when done, else clear: SR_WC0 + size index, bl, r3 the pc); SR_TP0/1:
// the calibrated model's cold path (r12 link, r5 -= the cost, flags as subs r5 / ge on a hit); SR_G2, G1, SQ, EQ (+1:
// the dynamic one, no next word): the update points' exits (bl, then next)
enum { SR_EXIT, SR_EXITD, SR_DPEN0, SR_DPEN1, SR_UNC0, SR_UNC1, SR_APEN, SR_DP0, SR_DP1, SR_IC, SR_DLOOK, SR_DLOOK2,
       SR_RD0, SR_WF0 = SR_RD0 + 6, SR_WC0 = SR_WF0 + 3, SR_TP0 = SR_WC0 + 3, SR_G2 = SR_TP0 + 2, SR_G2D, SR_G1, SR_G1D,
       SR_SQ, SR_SQD, SR_EQ, SR_EQD, SR_N };
#define SJ_MAXBL 2048
static unsigned s_sr_at[SR_N];
static unsigned s_bl_at[SJ_MAXBL], s_nbl;
static uint8_t s_bl_to[SJ_MAXBL];
static void e_bl_raw(int sr)
{
	if (s_nbl >= SJ_MAXBL) {
		s_full = 1;
		s_nbl = 0;
	}
	s_bl_at[s_nbl] = s_cn == &s_nc ? s_nc | 0x80000000u : s_n;   // (a cold site: moved by e_finish)
	s_bl_to[s_nbl++] = (uint8_t)sr;
	E(0xeb000000u);
}
static void e_b_r12(int sr)   // (OPT_SHR: a shared routine with the return address in r12; lr kept)
{
	E(0xe28fc000u);   // add r12, pc, #0 (the word after the b)
	if (s_nbl >= SJ_MAXBL) {
		s_full = 1;
		s_nbl = 0;
	}
	s_bl_at[s_nbl] = s_cn == &s_nc ? s_nc | 0x80000000u : s_n;
	s_bl_to[s_nbl++] = (uint8_t)sr;
	E(0xea000000u);
}
static void e_bl(int sr)   // (a shared routine: lr and the routine's C call as e_call)
{
	e_rc_pre();
	e_bl_raw(sr);
	e_rc_post();
}

// the block's compile state: cycles gathered and not counted off (s_pend, + r6 when s_pdyn), r10 maybe set, the pc
// in c->pc (a slot of a transfer to a register's address)
static uint32_t s_pend;
static int s_pdyn, s_r10, s_dyn;
static int s_sj_inslot;   // (ADR-175: a delay slot being translated)

// OPT_RC (ADR-140): the register cache of a block. Guest g (R0-R15, 16 = SR) in host register s_rch[g] (r10, lr or
// r6; r6 only once s_pdyn is 0: until then g is used in c), loaded at its first use (s_rcl), dirty when written
// (s_rcd). Written back at every exit (the stub, before r6 is set), before every C call and shared routine (lr is
// reloaded after one in the cold code, forgotten in the hot), forgotten where a helper may change it. The hot code
// is one straight path (its cold paths come back to the point they left), so the state at each point is known
#define RC_SR 16
// (s_rc, s_rcl, s_rcd: at the options)
static int8_t s_rch[17];

static uint32_t s_sj_irqf;       // (OPT_RC: r10's test_irq | flushed << 1; 0 at a block's entry: sj_run clears it)
static uint32_t rc_off(int g) { return g == RC_SR ? COFS(sr) : RO(g); }
static int rc_h(int g)   // (the host register of g now, -1: in c)
{
	if (!s_rc || s_rch[g] < 0 || (s_rch[g] == 6 && s_pdyn))
		return -1;
	return s_rch[g];
}
// a register with g's value: its host register (loaded now at the first use) or s loaded from c
static int e_rg(int g, int s)
{
	const int h = rc_h(g);
	if (h < 0) {
		e_ldr(s, rc_off(g));
		return s;
	}
	if (!((s_rcl >> g) & 1)) {
		e_ldr(h, rc_off(g));
		s_rcl |= 1u << g;
	}
	return h;
}
// the register to compute g's new value into (its host register or s), then e_rw(g, it)
static int e_rd(int g, int s) { const int h = rc_h(g); return h < 0 ? s : h; }
static void e_rw(int g, int r)
{
	const int h = rc_h(g);
	if (h < 0) {
		e_str(r, rc_off(g));
		return;
	}
	if (r != h)
		e_mov(h, r);
	s_rcl |= 1u << g;
	s_rcd |= 1u << g;
}
// the dirty ones of mask to c (in the hot code: clean after it)
static void e_rc_wb(uint32_t mask)
{
	if (!s_rc)
		return;
	const uint32_t m = s_rcl & s_rcd & mask;
	for (int g = 0; g < 17; g++)
		if ((m >> g) & 1)
			e_str(s_rch[g], rc_off(g));
	if (s_cn == &s_n)
		s_rcd &= ~m;
}
// (hot code, after e_rc_wb: c has them) the host registers of mask not holding their guests any more
static void e_rc_kill(uint32_t mask)
{
	s_rcl &= ~mask;
	s_rcd &= ~mask;
}
static void e_rc_pre(void) { e_rc_wb(~0u); }
static void e_rc_post(void)
{
	if (!s_rc)
		return;
	for (int g = 0; g < 17; g++)
		if (s_rch[g] == 14 && ((s_rcl >> g) & 1)) {
			if (s_cn == &s_n)
				e_rc_kill(1u << g);
			else
				e_ldr(14, rc_off(g));   // (the cold code goes back to a hot point that has it)
		}
}
// the flag r10 held, for a test: r10, or (OPT_RC) s_sj_irqf in r12
static int e_flag(void)
{
	if (!SJ_RCF)
		return 10;
	e_imm(12, (uint32_t)(uintptr_t)&s_sj_irqf);
	E(0xe59cc000u);   // ldr r12, [r12]
	return 12;
}
// the flag computed in r to where e_flag finds it
static void e_flag_set(int r)
{
	if (!SJ_RCF) {
		if (r != 10)
			e_mov(10, r);
		return;
	}
	e_imm(12, (uint32_t)(uintptr_t)&s_sj_irqf);
	E(0xe58c0000u | (uint32_t)r << 12);   // str r, [r12]
}

static unsigned s_sj_entry;   // (words of e_begin: where a chained jump from another block enters)
static uint32_t s_tpos, s_tcond;   // (OPT_TF: the host flags hold T as condition s_tcond while the hot code is at s_tpos)
static void e_begin(sh3_t *c)
{
	e_bufs_reset();
	s_nx = s_nsite = s_nbl = 0;
	s_full = 0;
	s_tpos = ~0u;
	s_kmask = 0;
	s_rc = 0;
	s_rcl = s_rcd = 0;
	E(SJ_R9 ? 0xe92d4ff8u : 0xe92d4df0u);   // push {r3-r11, lr} / push {r4-r8, r10, r11, lr}
	e_imm2(4, (uint32_t)(uintptr_t)c);
	e_ldr(5, COFS(icount));
	E(0xe284ca00u | ((COFS(jpend) >> 12) & 0xf));               // add r12, r4, #hi
	E(0xe59c6000u | (COFS(jpend) & 0xfff));                     // ldr r6, [r12, #lo]
	e_imm2(7, (uint32_t)(uintptr_t)c->rd);
	e_imm2(8, (uint32_t)(uintptr_t)c->wr);
	if (SJ_R9)
		e_imm2(9, (uint32_t)(uintptr_t)&c->cache[0][0]);   // (OPT_TBASE: r9 = the tag array)
	E(0xe3a0a000u);   // mov r10, #0
	s_sj_entry = s_n;
	if (s_sj_prof && s_sj_nprof < SJ_PROF_N) {   // (RBX_SH3_JITPROF: the block's entries, chained ones too)
		e_imm2(12, (uint32_t)(uintptr_t)&s_sj_profr[s_sj_nprof].n);
		E(0xe59c0000u);   // ldr r0, [r12]
		E(0xe2800001u);   // add r0, r0, #1
		E(0xe58c0000u);   // str r0, [r12]
	}
	s_pend = 0;
	s_pdyn = 1;
	s_r10 = 0;
	s_dyn = 0;
	s_sj_inslot = 0;
}

static void e_finish(void)
{
	unsigned at[SJ_MAXX];
	for (unsigned k = 0; k < s_nx; k++) {
		at[k] = s_n;
		if (s_x[k].kind >= XK_G2) {   // (OPT_SHR: the dirty registers, [r6 as XK_EXIT's], the routine, [next])
			for (int g = 0; g < 17; g++)
				if ((s_x[k].wb >> g) & 1)
					e_str(s_rch[g], rc_off(g));
			if (s_x[k].kind == XK_EQ) {
				if (s_x[k].pdyn)
					e_addi(6, 6, s_x[k].pend);
				else
					e_imm(6, s_x[k].pend);
			}
			e_bl_raw((s_x[k].kind == XK_G2 ? SR_G2 : s_x[k].kind == XK_G1 ? SR_G1 : s_x[k].kind == XK_SQ ? SR_SQ : SR_EQ) +
			         (s_x[k].dyn ? 1 : 0));
			if (!s_x[k].dyn)
				E(s_x[k].next);
			continue;
		}
		if (s_x[k].kind == XK_SPLIT2) {
			E(0xe3500000u | (uint32_t)e_flag() << 16);   // cmp r10, #0
			e_bto(0x0, s_x[k].next);
			e_imm(12, (uint32_t)(uintptr_t)&s_sj_stash);
			E(0xe59cc000u);   // ldr r12, [r12]
			E(0xe2455103u);   // sub r5, r5, #0xc0000000
			E(0xe085500cu);   // add r5, r5, r12
		}
		if (s_x[k].kind == XK_SPLIT || s_x[k].kind == XK_SPLIT2) {
			E(0xe3550000u);   // cmp r5, #0
			e_bto(0xb, s_x[k].next);
			e_bto(0xe, s_x[k].pend);
			continue;
		}
		if (s_x[k].q) {   // (the interrupt's exit when r10 bit 0 is set)
			E(0xe3100001u | (uint32_t)e_flag() << 16);   // tst r10, #1
			e_bto(0x1, s_x[k].qto);
		}
		for (int g = 0; g < 17; g++)   // (OPT_RC: before r6 is set)
			if ((s_x[k].wb >> g) & 1)
				e_str(s_rch[g], rc_off(g));
		if (s_x[k].kind == XK_EXIT) {
			const uint32_t p = s_x[k].pend;
			if (s_x[k].pdyn)
				e_addi(6, 6, p);
			else
				e_imm(6, p);
		}
		if (s_x[k].chain)
			E(0xe1a00000u);   // the slot (nop)
		const uint32_t code = (s_x[k].kind == XK_EXIT ? s_x[k].code : 0) | (s_x[k].chain ? 0x100u : 0) |
		                      (s_x[k].dyn ? 0x200u : 0);   // (0x200: RBX_SH3_JITPROF's count only)
		if (s_x[k].dyn && s_x[k].look && SJ_OPT(OPT_DSITE) && s_sj_ds && s_sj_nds < SJ_DS_N) {
			// (OPT_DSITE: ldr r0, pc; r12 = the slot; ldm r12, {r1, r2}; cmp r0, r1; bxeq r2; else SR_DLOOK2)
			uint32_t *const ds = s_sj_ds[s_sj_nds++];
			ds[0] = 1;
			ds[1] = 0;
			e_ldr(0, COFS(pc));
			e_imm2(12, (uint32_t)(uintptr_t)ds);
			E(0xe89c0006u);   // ldm r12, {r1, r2}
			E(0xe1500001u);   // cmp r0, r1
			E(0x012fff12u);   // bxeq r2
			e_bl_raw(SR_DLOOK2);
			E(code);
		} else if (s_x[k].dyn) {
			e_bl_raw(s_x[k].look ? SR_DLOOK : SR_EXITD);
			E(code);
		} else {
			e_bl_raw(SR_EXIT);
			E(s_x[k].next);
			E(code);
		}
	}
	for (unsigned k = 0; k < s_nsite; k++)
		if (!(s_site_at[k] >> 31))
			s_sj_buf[s_site_at[k]] |= (at[s_site_to[k]] - s_site_at[k] - 2) & 0x00ffffffu;
	e_cold_append();
	for (unsigned k = 0; k < s_nsite; k++)
		if (s_site_at[k] >> 31) {
			const unsigned w = s_cold_base + (s_site_at[k] & 0x7fffffffu);
			s_sj_buf[w] |= (at[s_site_to[k]] - w - 2) & 0x00ffffffu;
		}
	for (unsigned k = 0; k < s_nbl; k++)
		if (s_bl_at[k] >> 31)
			s_bl_at[k] = s_cold_base + (s_bl_at[k] & 0x7fffffffu);
}

// leave for next (code 1: on at next, 2: the interrupt test first) with the cycles gathered
static void e_exit(uint32_t next, int dyn, int code, int chain)
{
	e_bto(0xe, e_newx(XK_EXIT, next, dyn, s_pend, s_pdyn, code, chain));
}
// MAME's update point: the gathered cycles counted off; the slice ends when icount < 0 and no interrupt is to be
// tested (with one: the interrupt test, which ends the slice after it)
// (r5 -= the gathered cycles; OPT_UPD: the last subtraction sets the flags as cmp r5, #0 would: icount is far from
// overflowing, so N is lt)
static void e_countoff(void)
{
	const int s = SJ_OPT(OPT_UPD);
	if (s_pdyn)
		E((s && !s_pend ? 0xe0555006u : 0xe0455006u));   // sub(s) r5, r5, r6
	if (s_pend) {
		const int k = arm_imm(s_pend);
		if (k >= 0)
			E((s ? 0xe2555000u : 0xe2455000u) | (uint32_t)k);   // sub(s) r5, r5, #k
		else {
			e_imm(12, s_pend);
			E(s ? 0xe055500cu : 0xe045500cu);
		}
	}
	if (!s || (!s_pend && !s_pdyn))
		E(0xe3550000u);   // cmp r5, #0
	s_pend = 0;
	s_pdyn = 0;
}
// (XK_SPLIT2 / XK_SPLIT and their exits: the slice's end, the interrupt's, on; OPT_SHR: one XK_G2 / XK_G1)
static unsigned e_split(int kind, uint32_t next, int dyn)
{
	if (s_shr)
		return e_newx(kind == XK_SPLIT2 ? XK_G2 : XK_G1, next, dyn, 0, 0, 0, 0);
	const unsigned z = e_newx(kind, 0, dyn, 0, 0, 0, 0);
	const unsigned x = e_newx(XK_SLICE, next, dyn, 0, 0, 0, 0);
	s_x[x].q = 1;
	s_x[x].qto = (uint16_t)e_newx(XK_EXIT, next, dyn, 0, 0, 2, 0);
	const unsigned y = e_newx(XK_EXIT, next, dyn, 0, 0, 1, 0);
	s_x[y].q = 1;
	s_x[y].qto = (uint16_t)e_newx(XK_EXIT, next, dyn, 0, 0, 2, 0);
	s_x[z].next = x;   // (the split: icount < 0 -> x, else y)
	s_x[z].pend = y;
	return z;
}
static void e_update(uint32_t next, int dyn)
{
	e_countoff();
	unsigned x;
	if (s_r10 && s_shr)
		x = e_newx(XK_SQ, next, dyn, 0, 0, 0, 0);
	else {
		x = e_newx(XK_SLICE, next, dyn, 0, 0, 0, 0);
		if (s_r10) {
			s_x[x].q = 1;
			s_x[x].qto = (uint16_t)e_newx(XK_EXIT, next, dyn, 0, 0, 2, 0);
		}
	}
	e_bto(0xb, x);    // blt
}
// the interrupt test after a call that may have set test_irq (or flushed the blocks: leave, unchained)
static void e_irqtest(uint32_t next, int dyn)
{
	if (!s_r10)
		return;
	E(0xe3500000u | (uint32_t)e_flag() << 16);   // cmp r10, #0
	unsigned x;
	if (s_shr)
		x = e_newx(XK_EQ, next, dyn, s_pend, s_pdyn, 1, 0);
	else {
		x = e_newx(XK_EXIT, next, dyn, s_pend, s_pdyn, 1, 0);
		s_x[x].q = 1;
		s_x[x].qto = (uint16_t)e_newx(XK_EXIT, next, dyn, s_pend, s_pdyn, 2, 0);
	}
	e_bto(0x1, x);    // bne
	s_r10 = 0;
}
// the count-off (upd) and the interrupt test after an instruction: OPT_UPD with r10 maybe set, one test in the hot
// code (subs r5 ...; cmppl r10, #0; bne) to a stub that takes the same exits as e_update then e_irqtest
static void e_update_irq(uint32_t next, int dyn, int upd)
{
	if (upd && s_r10 == 2) {   // (OPT_SENT: r10 set -> r5 below 0)
		e_countoff();
		e_bto(0xb, e_split(XK_SPLIT2, next, dyn));    // blt
		s_r10 = 0;
		return;
	}
	if (!upd || !s_r10 || !SJ_OPT(OPT_UPD)) {
		if (upd)
			e_update(next, dyn);
		e_irqtest(next, dyn);
		return;
	}
	e_countoff();
	E(0x53500000u | (uint32_t)e_flag() << 16);   // cmppl r10, #0
	e_bto(0x1, e_split(XK_SPLIT, next, dyn));    // bne
	s_r10 = 0;
}

// (OPT_FUSE) the exits e_update_irq's test takes (r5 < 0 after its count-off), made where the test is
static unsigned e_upd_exits(uint32_t next, int dyn)
{
	if (s_r10 == 2)
		return e_split(XK_SPLIT2, next, dyn);
	return e_newx(XK_SLICE, next, dyn, 0, 0, 0, 0);
}

// the T bit from the host condition cond (after a compare)
static void e_tflags(uint32_t cond)   // (OPT_TF: the host flags hold T as cond here)
{
	if (s_cn == &s_n) {
		s_tpos = s_n;
		s_tcond = cond;
	}
}
static void e_sett(uint32_t cond)
{
	const int r = e_rg(RC_SR, 2), d = e_rd(RC_SR, 2);
	E(0xe3c00001u | (uint32_t)r << 16 | (uint32_t)d << 12);   // bic r2, r2, #1
	E(cond << 28 | 0x03800001u | (uint32_t)d << 16 | (uint32_t)d << 12);   // orr<cond> r2, r2, #1
	e_rw(RC_SR, d);
	e_tflags(cond);
}
// a conditional transfer's branch when it is not taken (bt: taken on T set): on the host flags when nothing was
// emitted since the T bit was set (OPT_TF), else on c->sr
static unsigned e_tbranch(int bt)
{
	if (SJ_OPT(OPT_TF) && s_cn == &s_n && s_tpos == s_n)
		return e_bcond(bt ? s_tcond ^ 1 : s_tcond);
	E(0xe3100001u | (uint32_t)e_rg(RC_SR, 0) << 16);   // tst r0, #1
	return e_bcond(bt ? 0x0 : 0x1);
}
// the pc of the instruction (its address + 2, or the transfer's target in a slot) -> rd
static void e_pcv(int rd, uint32_t pcv)
{
	if (s_dyn)
		e_ldr(rd, COFS(pc));
	else
		e_imm(rd, pcv);
}
// (ADR-175) a load's pc for sj_rd*: bit 0 set in a delay slot (s_sj_inslot; s_dyn is a slot's)
static void e_pcv_ld(uint32_t pcv)
{
	if (s_dyn) {
		e_ldr(2, COFS(pc));
		E(0xe3822001u);   // orr r2, r2, #1
	} else
		e_imm(2, pcv | (uint32_t)s_sj_inslot);
}

/***************************************************************************
    the timing model's common case inline (sh3.c cache_access: a hit on the way used last, lru 3, changes nothing but
    the dirty bit of a write; the interpreter's hint is only its shortcut to the same answer), else its C functions
***************************************************************************/

#define SJ_CACHE COFS(cache)
// the cache line of the address in ra (not r1-r3): falls through at a hit on the line's way used last (r3 = that
// way's entry), else e_tocold to the next cold path (not cacheable, not in the cache, or the LRU to update: the C
// function decides). Clobbers r1-r3
static void e_cacheable(int ra)   // Z clear when the address in ra is cacheable. Clobbers r1
{
	E(0xe1a01ea0u | (uint32_t)ra);           // lsr r1, ra, #29
	E(0xe3510005u);                          // cmp r1, #5
	E(0x13510007u);                          // cmpne r1, #7
}
static void e_mru2(int ra, int test);
static void e_mru(int ra) { e_mru2(ra, 1); }
static void e_mru2(int ra, int test)
{
	if (test) {
		e_cacheable(ra);
		e_tocold(0x0);                       // beq: not cacheable
	}
	E(ubfx(1, ra, 4, 25));                   // r1 = (a & 0x1fffffff) / 16
	E(0xe20120ffu);                          // and r2, r1, #255
	E(0xe0843282u);                          // add r3, r4, r2, lsl #5
	e_addi(3, 3, SJ_CACHE);
	E(0xe5932000u);                          // ldr r2, [r3]
	E(0xe1520001u);                          // cmp r2, r1
	for (int w = 1; w < 4; w++) {
		E(0x15b32008u);                      // ldrne r2, [r3, #8]!
		E(0x11520001u);                      // cmpne r2, r1
	}
	e_tocold(0x1);                           // bne: not in the cache
	E(0xe5d32004u);                          // ldrb r2, [r3, #4] (lru)
	E(0xe3520003u);                          // cmp r2, #3
	e_tocold(0x1);                           // bne: the C function updates the LRU
}
static uint32_t sj_apen(sh3_t *c, uint32_t a, uint32_t w)
{
	SJ_CNT(2);
	const unsigned pa = rbx_prof_enter(SH3_PA_PENALTY);
	const uint32_t v = access_penalty(c, a, (int)w);
	rbx_prof_leave(pa);
	return v;
}
// (the fields access_penalty's bus part uses, in one 4 KB part of c past the cache)
static int sj_bus_fields_ok(void)
{
	const uint32_t hi = COFS(wb_address) & ~0xfffu;
	return (COFS(last_area) & ~0xfffu) == hi && (COFS(wb_active_cycles) & ~0xfffu) == hi &&
	       (COFS(last_sdram_page) & ~0xfffu) == hi && (COFS(precharge_remaining) & ~0xfffu) == hi &&
	       (COFS(wcr1) & ~0xfffu) == hi && (COFS(wcr2) & ~0xfffu) == hi && (COFS(mcr) & ~0xfffu) == hi;
}
#define FLD(f) (COFS(f) & 0xfff)
// access_penalty of a not cacheable access at r11 (cache_access: 0, nothing changed) for areas 1, 3 (SDRAM) and
// 4-6 with no write-back pending (wb_address 0): its bus part inline, r5 -= it. The other cases branch to the sites
// put in toc[] (3 of them: the C function)
static void e_uncached(int write, unsigned *toc)
{
	E(0xe284ca00u | ((COFS(wb_address) >> 12) & 0xf));   // add r12, r4, #hi
	E(0xe59c0000u | FLD(wb_address));                     // ldr r0, [r12, #wb_address]
	E(0xe3500000u);                                       // cmp r0, #0
	toc[0] = e_bcond(0x1);                                // bne -> C
	E(ubfx(1, 11, 26, 3));                                // r1 = area
	E(0xe3510003u);                                       // cmp r1, #3
	const unsigned sd = e_bcond(0x0);                     // beq
	E(0xe3510001u);                                       // cmp r1, #1
	const unsigned a1 = e_bcond(0x0);                     // beq
	E(0xe3510004u);                                       // cmp r1, #4
	toc[1] = e_bcond(0x3);                                // blo -> C (areas 0, 2)
	E(0xe3510006u);                                       // cmp r1, #6
	toc[2] = e_bcond(0x8);                                // bhi -> C (area 7)
	// areas 4-6: bus = 3 + single[(wcr2 >> (7 + (area - 4) * 3)) & 7], + wcr1 when the area changed
	E(0xe59c0000u | FLD(wcr2));                           // ldr r0, [r12, #wcr2]
	E(0xe6ff0070u);                                       // uxth r0, r0
	E(0xe2412004u);                                       // sub r2, r1, #4
	E(0xe0822082u);                                       // add r2, r2, r2, lsl #1
	E(0xe2822007u);                                       // add r2, r2, #7
	E(0xe1a00230u);                                       // lsr r0, r0, r2
	E(0xe2000007u);                                       // and r0, r0, #7
	E(0xe3500004u);                                       // cmp r0, #4
	E(0xc1a00080u);                                       // lslgt r0, r0, #1
	E(0xc2400004u);                                       // subgt r0, r0, #4 (single[]: 0 1 2 3 4 6 8 10)
	E(0xe2802003u);                                       // add r2, r0, #3
	E(0xe59c3000u | FLD(last_area));                      // ldr r3, [r12, #last_area]
	E(0xe1530001u);                                       // cmp r3, r1
	const unsigned same = e_bcond(0x0);
	E(0xe59c0000u | FLD(wcr1));                           // ldr r0, [r12, #wcr1]
	E(0xe1a03081u);                                       // lsl r3, r1, #1
	E(0xe1a00330u);                                       // lsr r0, r0, r3
	E(0xe2000003u);                                       // and r0, r0, #3
	E(0xe3500000u);                                       // cmp r0, #0
	E(0x03a00001u);                                       // moveq r0, #1
	E(0xe0822000u);                                       // add r2, r2, r0
	const unsigned nosd = e_bcond(0xe);
	e_patch(a1);
	E(0xe3a02003u);                                       // mov r2, #3 (area 1: no wait states)
	e_patch(same);
	e_patch(nosd);
	E(0xe58c1000u | FLD(last_area));                      // str r1, [r12, #last_area]
	E(0xe3a03000u);                                       // mov r3, #0 (cpu)
	E(0xe58c3000u | FLD(precharge_remaining));            // str r3, [r12, #precharge_remaining]
	const unsigned common = e_bcond(0xe);
	// SDRAM: bus = 3 + rcd + (write ? trwl : wcr2), + wcr1 when the area changed; the page's precharge
	e_patch(sd);
	E(0xe59c0000u | FLD(mcr));                            // ldr r0, [r12, #mcr]
	E(ubfx(2, 0, 12, 2));
	E(0xe2822004u);                                       // add r2, r2, #4 (3 + rcd)
	if (write) {
		E(ubfx(3, 0, 10, 2));
		E(0xe0822003u);                                   // add r2, r2, r3
		E(0xe2822001u);                                   // add r2, r2, #1 (trwl)
	} else {
		E(0xe59c3000u | FLD(wcr2));                       // ldr r3, [r12, #wcr2]
		E(ubfx(3, 3, 5, 2));
		E(0xe3530000u);                                   // cmp r3, #0
		E(0x03a03001u);                                   // moveq r3, #1
		E(0xe0822003u);                                   // add r2, r2, r3
	}
	E(0xe59c3000u | FLD(last_area));                      // ldr r3, [r12, #last_area]
	E(0xe1530001u);                                       // cmp r3, r1
	const unsigned same3 = e_bcond(0x0);
	E(0xe59c3000u | FLD(wcr1));                           // ldr r3, [r12, #wcr1]
	E(ubfx(3, 3, 6, 2));
	E(0xe3530000u);
	E(0x03a03001u);
	E(0xe0822003u);
	e_patch(same3);
	E(0xe58c1000u | FLD(last_area));                      // str r1, [r12, #last_area]
	E(0xe1a0152bu);                                       // lsr r1, r11, #10 (the page)
	E(0xe3a03000u);                                       // mov r3, #0
	E(0xe59c0000u | FLD(last_sdram_page));                // ldr r0, [r12, #last_sdram_page]
	E(0xe1500001u);                                       // cmp r0, r1
	E(0x059c3000u | FLD(precharge_remaining));            // ldreq r3, [r12, #precharge_remaining]
	E(0xe58c1000u | FLD(last_sdram_page));                // str r1, [r12, #last_sdram_page]
	E(0xe59c0000u | FLD(mcr));                            // ldr r0, [r12, #mcr]
	E(ubfx(0, 0, 14, 2));
	E(0xe2800001u);                                       // add r0, r0, #1
	E(0xe1a00080u);                                       // lsl r0, r0, #1 (mcr_tpc * 2)
	E(0xe58c0000u | FLD(precharge_remaining));            // str r0, [r12, #precharge_remaining]
	e_patch(common);
	E(0xe59c0000u | FLD(wb_active_cycles));               // ldr r0, [r12, #wb_active_cycles]
	E(0xe0833000u);                                       // add r3, r3, r0
	E(0xe3a00000u);                                       // mov r0, #0
	E(0xe58c0000u | FLD(wb_active_cycles));               // str r0, [r12, #wb_active_cycles]
	E(0xe0830082u);                                       // add r0, r3, r2, lsl #1
	E(0xe0455000u);                                       // sub r5, r5, r0
}
// data_penalty for the access at r11: r5 -= it (bl SR_DP*, the shared routine made by e_dpen_sr)
static void e_sites_to(unsigned target);
static void e_dpen_sr(int write);
// the calibrated model (sh3.c tpenalty, c->exact 0) for the access at r11: inline the check of the set's way used
// last (a hit costs nothing; a not cacheable address is never a tag); in the cold code the other way (a hit: the two
// swap), a miss (the line in, the older out) or a not cacheable access: r5 -= the class's cost
// tcost[write][r11 >> 26]. Clobbers r0-r3
typedef char sj_tag_ofs_ok[offsetof(sh3_t, cache) < 4096 - 16 && sizeof(((sh3_t *)0)->cache[0][0]) == 8 ? 1 : -1];
static void e_tpen(int write)
{
	uint32_t co = COFS(cache);               // (r3 + co: the set's way used last)
	const int fuse = s_fuse_ok && SJ_OPT(OPT_FUSE) && SJ_RCF && SJ_R9 && SJ_OPT(OPT_SENT) && SJ_OPT(OPT_UPD) &&
	                 s_r10 != 1;
	unsigned fneg = 0, fz = 0;
	if (fuse) {   // (the update point's count-off first: subs r5 ...)
		s_pend += s_fcost;
		e_countoff();
		s_fused = 1;
	}
	if (SJ_R9) {
		E(ubfx(2, 11, 4, 9));                // r2 = the set
		E(0xe7992202u);                      // ldr r2, [r9, r2, lsl #4] (the way used last)
		E(fuse ? 0xa152022bu : 0xe152022bu); // cmp(ge) r2, r11, lsr #4 (the line)
		e_tocold(0x1);                       // bne (OPT_FUSE: or icount < 0)
		e_cold();
		if (fuse) {   // (icount < 0: below; else a miss of the way used last)
			fz = e_upd_exits(s_fnext, s_fdyn);
			E(0xe3550000u);                  // cmp r5, #0
			fneg = e_bcond(0xb);             // blt
		}
		if (!s_shr) {   // (SR_TP* finds them)
			E(0xe1a0122bu);                  // lsr r1, r11, #4
			E(ubfx(2, 11, 4, 9));
			E(0xe0893202u);                  // add r3, r9, r2, lsl #4
		}
		co = 0;
	} else {
		E(0xe1a0122bu);                      // lsr r1, r11, #4 (the line)
		E(ubfx(2, 11, 4, 9));                // r2 = its set
		E(0xe0843202u);                      // add r3, r4, r2, lsl #4
		E(0xe5932000u | COFS(cache));        // ldr r2, [r3, #cache] (the way used last)
		E(0xe1520001u);                      // cmp r2, r1
		e_tocold(0x1);                       // bne
		e_cold();
	}
	if (s_shr) {   // (ADR-166: the same in SR_TP*, r12 link)
		e_b_r12(SR_TP0 + write);
		if (fuse)
			e_bto(0xb, fz);                      // blt: the update point's exits
	} else {
		E(0xe5930000u | co);                     // ldr r0, [r3, #co]
		E(0xe5932000u | (co + 8));               // ldr r2, [r3, #co + 8] (the other way)
		E(0xe1520001u);                          // cmp r2, r1
		const unsigned miss = e_bcond(0x1);      // bne
		E(0xe5831000u | co);                     // str r1, [r3, #co]
		E(0xe5830000u | (co + 8));               // str r0, [r3, #co + 8]
		e_imm(2, (uint32_t)(uintptr_t)&s_sj_cpu->cache_gen);
		E(0xe5921000u);                          // ldr r1, [r2]
		E(0xe2811001u);                          // add r1, r1, #1
		E(0xe5821000u);                          // str r1, [r2]
		const unsigned done = e_bcond(0xe);
		e_patch(miss);
		e_cacheable(11);
		const unsigned unc = e_bcond(0x0);       // beq: not cacheable
		E(0xe1a0122bu);                          // lsr r1, r11, #4
		E(0xe5831000u | co);                     // str r1, [r3, #co]
		E(0xe5830000u | (co + 8));               // str r0, [r3, #co + 8]
		e_imm(2, (uint32_t)(uintptr_t)&s_sj_cpu->cache_gen);
		E(0xe5921000u);                          // ldr r1, [r2]
		E(0xe2811001u);                          // add r1, r1, #1
		E(0xe5821000u);                          // str r1, [r2]
		e_patch(unc);
		e_imm(3, (uint32_t)(uintptr_t)s_sj_cpu->tcost[write]);
		E(0xe1a00d2bu);                          // lsr r0, r11, #26
		E(0xe0833080u);                          // add r3, r3, r0, lsl #1
		E(0xe1d300b0u);                          // ldrh r0, [r3]
		if (fuse) {
			E(0xe0555000u);                      // subs r5, r5, r0
			e_bto(0xb, fz);                      // blt: the update point's exits
		} else
			E(0xe0455000u);                      // sub r5, r5, r0
		e_patch(done);
	}
	e_hot();
	if (fuse) {   // (icount < 0 after the count-off: the penalty in C, then the exits)
		s_cb = s_cold;
		s_cn = &s_nc;
		s_ccap = SJ_COLD;
		e_patch(fneg);
		e_bl(SR_DPEN0 + write);
		e_bto(0xe, fz);
		s_cb = s_sj_buf;
		s_cn = &s_n;
		s_ccap = SJ_BUF;
		s_r10 = 0;
	}
}
static void e_dpen(int write)
{
	if (!s_sj_cpu->exact) {
		e_tpen(write);
		return;
	}
	if (s_sj_dbg & 8) {
		e_bl(SR_DPEN0 + write);
		return;
	}
	e_dpen_sr(write);
}
// (sr: SR_DP*, a shared routine; else inline in a block, its other cases in the cold code)
static int s_sr_gen;
static void e_dpen_sr(int write)
{
	if (s_sr_gen)
		s_nhot = 0;
	E(0xe35b020eu);                          // cmp r11, #0xe0000000
	e_tocold(0x2);                           // bhs: arg0 stays (the C function)
	e_strf(11, COFS(arg0));
	e_cacheable(11);
	const unsigned ub = e_bcond(0x0);        // beq: not cacheable
	e_mru2(11, 0);
	if (write) {
		E(0xe3a02001u);                      // mov r2, #1
		E(0xe5c32005u);                      // strb r2, [r3, #5] (dirty)
	}
	if (s_sr_gen) {
		E(0xe12fff1eu);                      // bx lr (a hit: 0)
		e_sites_to(s_sr_at[SR_DPEN0 + write]);
		s_sj_buf[ub] |= (s_sr_at[SR_UNC0 + write] - ub - 2) & 0x00ffffffu;
		return;
	}
	e_cold();                                // (the C function: the sites of e_tocold above)
	e_bl(SR_DPEN0 + write);
	e_hot();
	if (s_nhx < SJ_XREF) {                   // (not cacheable: bl SR_UNC*, back to the same place)
		s_hx_at[s_nhx] = ub;
		s_hx_to[s_nhx++] = s_nc;
	} else
		s_full = 1;
	s_cb = s_cold;
	s_cn = &s_nc;
	s_ccap = SJ_COLD;
	e_bl(SR_UNC0 + write);
	e_hot();
}
// icache_penalty: r5 -= it (in a block: bl SR_IC, the shared routine made by e_ipen_sr)
static int s_sr_ic_ok;
static void e_ipen_sr(void);
static void e_ipen(void)
{
	if (!s_sj_cpu->exact)   // (the calibrated model: no instruction fetch cost)
		return;
	if (s_sr_ic_ok && !(s_sj_dbg & 8)) {
		e_ipen_sr();   // (inline: s_sr_gen 0)
		return;
	}
	e_mov(0, 4);
	e_call((const void *)sj_icache);
	E(0xe0455000u);
}
static void e_sites_to(unsigned target)   // (the e_tocold sites, in a shared routine: branches to target)
{
	for (unsigned k = 0; k < s_nhot; k++)
		s_sj_buf[s_hot_sites[k]] |= (target - s_hot_sites[k] - 2) & 0x00ffffffu;
	s_nhot = 0;
}
static void e_ipen_sr(void)
{
	if (s_sr_gen)
		s_nhot = 0;
	const uint32_t hi = COFS(wb_active_cycles) & ~0xfffu;
	if ((COFS(precharge_remaining) & ~0xfffu) != hi || (COFS(drc_pc) & ~0xfffu) != hi || (COFS(mcr) & ~0xfffu) != hi)
		return;   // (e_ipen calls the C function)
	E(0xe284ca00u | ((hi >> 12) & 0xf));                      // add r12, r4, #hi
	E(0xe59c0000u | (COFS(wb_active_cycles) & 0xfff));        // ldr r0, [r12, #wb_active_cycles]
	E(0xe3500000u);                                           // cmp r0, #0
	const unsigned nowb = e_bcond(0x0);                       // beq
	E(0xe2500001u);                                           // subs r0, r0, #1
	E(0xe58c0000u | (COFS(wb_active_cycles) & 0xfff));        // str r0, [r12, #wb_active_cycles]
	const unsigned mru1 = e_bcond(0x1);                       // bne
	E(0xe59c1000u | (COFS(mcr) & 0xfff));                     // ldr r1, [r12, #mcr]
	E(ubfx(1, 1, 14, 2));
	E(0xe2811001u);                                           // add r1, r1, #1
	E(0xe1a01081u);                                           // lsl r1, r1, #1 (mcr_tpc * 2)
	E(0xe58c1000u | (COFS(precharge_remaining) & 0xfff));     // str r1, [r12, #precharge_remaining]
	const unsigned mru2 = e_bcond(0xe);
	e_patch(nowb);
	E(0xe59c1000u | (COFS(precharge_remaining) & 0xfff));     // ldr r1, [r12, #precharge_remaining]
	E(0xe3510000u);                                           // cmp r1, #0
	E(0x12411001u);                                           // subne r1, r1, #1
	E(0x158c1000u | (COFS(precharge_remaining) & 0xfff));     // strne r1, [r12, #precharge_remaining]
	e_patch(mru1);
	e_patch(mru2);
	E(0xe59c0000u | (COFS(drc_pc) & 0xfff));                  // ldr r0, [r12, #drc_pc]
	e_mru(0);
	if (s_sr_gen) {
		E(0xe12fff1eu);                                       // bx lr (a hit: 0)
		e_sites_to(s_sr_at[SR_APEN]);                         // access_penalty(c, drc_pc, 0)
		s_sr_ic_ok = 1;
		return;
	}
	e_cold();
	e_bl(SR_APEN);                                            // access_penalty(c, drc_pc, 0)
	e_hot();
}

/***************************************************************************
    memory accesses (the interpreter's RD* / WR*: the page table, else slow_r / slow_w; then the data penalty)
***************************************************************************/

// (OPT_PCHK) an access's two tests (on-chip area, no page) in one: a > 0xdf000000 or no page -> the slow path. The
// same paths as the two tests when no page of 0x1f000000-0x1fffffff (area 7) has a read or write entry or translated
// code: then the addresses 0xdf000001-0xdfffffff find no page either. Known again after every flush (s_sj_a7 -1)
static int e_pchk(void)
{
	if (!SJ_OPT(OPT_PCHK))
		return 0;
	if (s_sj_a7 < 0) {
		const sh3_t *const c = s_sj_cpu;
		s_sj_a7 = 1;
		for (uint32_t pg = PAGE(0x1f000000u); pg <= PAGE(0x1fffffffu); pg++)
			if (c->rd[pg] || c->wr[pg] || s_sj_pslot[pg]) {
				s_sj_a7 = 0;
				break;
			}
	}
	return s_sj_a7;
}
// (OPT_SENT, the cold path of an access, r10 just set) r10 != 0: icount to s_sj_stash, r5 = SJ_SENT, so that the
// count-off after the access (r5 stays far below 0) finds it in its one test and its stub puts icount back

static void e_sentinel(int f)   // (f: the register with the flag)
{
	if (!SJ_OPT(OPT_SENT))
		return;
	E(0xe3500000u | (uint32_t)f << 16);   // cmp r10, #0
	e_imm(12, (uint32_t)(uintptr_t)&s_sj_stash);
	E(0x158c5000u);                    // strne r5, [r12]
	E(0x13a05103u);                    // movne r5, #0xc0000000
}
// a load of size bytes at ra (r0, or r11 with OPT_A11; sx: sign-extended), to guest register dg (OPT_RC: maybe its
// host register) or (dg < 0) the field of c at dst (MACH ...); then guest register pg (>= 0, ra 11) += psz (the
// post-increment), then the penalty. Clobbers r0-r3, r12, r11 (= the address after it)
static void e_load(int size, int sx, int dg, uint32_t dst, uint32_t pcv, int ra, int pg, uint32_t psz)
{
	const uint32_t A = (uint32_t)ra;
	const int v = dg >= 0 ? e_rd(dg, 0) : 0;   // (the value's register)
	const uint32_t V = (uint32_t)v << 12;
	if (ra != 11)
		e_mov(11, ra);
	if (e_pchk()) {
		E(ubfx(1, ra, 12, 17));        // r1 = (a & 0x1fffffff) >> 12
		E(0xe7973101u);                // ldr r3, [r7, r1, lsl #2]
		E(0xe270c4dfu | A << 16);      // rsbs r12, ra, #0xdf000000
		E(0x23530001u);                // cmpcs r3, #1
		e_tocold(0x3);                 // bcc
	} else {
		E(0xe350020eu | A << 16);      // cmp ra, #0xe0000000
		e_tocold(0x2);                 // bhs
		E(ubfx(1, ra, 12, 17));        // r1 = (a & 0x1fffffff) >> 12
		E(0xe7973101u);                // ldr r3, [r7, r1, lsl #2]
		E(0xe3530000u);                // cmp r3, #0
		e_tocold(0x0);                 // beq
	}
	if (size == 4) {
		E(ubfx(1, ra, 2, 10));
		E(0xe7930101u | V);            // ldr r0, [r3, r1, lsl #2]
	} else if (size == 2) {
		E(ubfx(1, ra, 1, 11));
		E(0xe2211001u);                // eor r1, r1, #1
		E(0xe0833081u);                // add r3, r3, r1, lsl #1
		E((sx ? 0xe1d300f0u : 0xe1d300b0u) | V);   // ldrsh / ldrh r0, [r3]
	} else {
		E(ubfx(1, ra, 0, 12));
		E(0xe2211003u);                // eor r1, r1, #3
		E((sx ? 0xe19300d1u : 0xe7d30001u) | V);   // ldrsb / ldrb r0, [r3, r1]
	}
	e_cold();
	if (s_shr) {   // (ADR-166: the same in SR_RD*)
		e_pcv_ld(pcv);
		e_bl(SR_RD0 + 2 * (size == 4 ? 2 : size == 2 ? 1 : 0) + (sx && size < 4));
		if (v)
			e_mov(v, 0);
	} else {
		e_str(5, COFS(icount));
		e_mov(1, ra);
		e_mov(0, 4);
		e_pcv_ld(pcv);
		e_call(size == 4 ? (const void *)sj_rd32 : size == 2 ? (const void *)sj_rd16 : (const void *)sj_rd8);
		e_ldr(5, COFS(icount));
		const int f = SJ_RCF ? 2 : 10;     // (the flag's register)
		e_ldr(f, COFS(test_irq));
		e_imm(12, (uint32_t)(uintptr_t)&s_sj_flushed);   // (a handler that changed the map: the blocks went)
		E(0xe59cc000u);                    // ldr r12, [r12]
		E(0xe180008cu | (uint32_t)f << 16 | (uint32_t)f << 12);   // orr r10, r10, r12, lsl #1
		if (SJ_RCF)
			e_flag_set(f);
		e_sentinel(f);
		if (sx && size < 4)
			E(size == 2 ? 0xe6bf0070u : 0xe6af0070u);   // sxth / sxtb r0, r0
		if (v)
			e_mov(v, 0);
	}
	e_hot();
	if (dg >= 0)
		e_rw(dg, v);
	else
		e_str(0, dst);
	if (pg >= 0) {
		const int d = e_rd(pg, 0);
		E(0xe28b0000u | (uint32_t)d << 12 | psz);   // add r0, r11, #psz
		e_rw(pg, d);
	}
	s_r10 = SJ_OPT(OPT_SENT) ? 2 : 1;
	e_dpen(0);
}
// a store of rv (r1, or a guest's host register; size bytes) at ra (r0, or r11 with OPT_A11), then the penalty.
// Clobbers r0-r3, r12, r11 (= the address after it)
static void e_store(int size, uint32_t pcv, int ra, int rv)
{
	const uint32_t A = (uint32_t)ra, RV = (uint32_t)rv << 12;
	if (ra != 11)
		e_mov(11, ra);
	if (e_pchk()) {
		E(ubfx(2, ra, 12, 17));
		E(0xe7983102u);                // ldr r3, [r8, r2, lsl #2]
		E(0xe270c4dfu | A << 16);      // rsbs r12, ra, #0xdf000000
		E(0x23530001u);                // cmpcs r3, #1
		e_tocold(0x3);                 // bcc
	} else {
		E(0xe350020eu | A << 16);      // cmp ra, #0xe0000000
		e_tocold(0x2);
		E(ubfx(2, ra, 12, 17));
		E(0xe7983102u);                // ldr r3, [r8, r2, lsl #2]
		E(0xe3530000u);
		e_tocold(0x0);
	}
	if (size == 4) {
		E(ubfx(2, ra, 2, 10));
		E(0xe7830102u | RV);           // str r1, [r3, r2, lsl #2]
	} else if (size == 2) {
		E(ubfx(2, ra, 1, 11));
		E(0xe2222001u);                // eor r2, r2, #1
		E(0xe0833082u);                // add r3, r3, r2, lsl #1
		E(0xe1c300b0u | RV);           // strh r1, [r3]
	} else {
		E(ubfx(2, ra, 0, 12));
		E(0xe2222003u);                // eor r2, r2, #3
		E(0xe7c30002u | RV);           // strb r1, [r3, r2]
	}
	e_cold();
	if (s_shr) {   // (ADR-166: the same in SR_WF* (r12 link: Z set when done) and SR_WC*)
		const int si = size == 4 ? 2 : size == 2 ? 1 : 0;
		if (rv != 1)
			e_mov(1, rv);
		e_b_r12(SR_WF0 + si);
		e_hot_if(0x0);                       // beq: back
		e_pcv(3, pcv);
		e_bl(SR_WC0 + si);
	} else {
		// a page with translated code, outside its translated granules and not while idle rounds record: the write to
		// its memory here, as slow_w / sj_write do it (nslow counted), r10 = test_irq as sj_wr* return it
		unsigned toc[5];
		E(0xe35b020eu);                          // cmp r11, #0xe0000000
		toc[0] = e_bcond(0x2);                   // bhs -> C
		E(ubfx(2, 11, 12, 17));                  // r2 = the page
		e_imm(3, (uint32_t)(uintptr_t)s_sj_pslot);
		E(0xe0833082u);                          // add r3, r3, r2, lsl #1
		E(0xe1d330b0u);                          // ldrh r3, [r3] (its slot + 1)
		E(0xe3530000u);                          // cmp r3, #0
		toc[1] = e_bcond(0x0);                   // beq -> C
		e_imm(12, (uint32_t)(uintptr_t)&s_sj_cpu->idle.rec);
		E(0xe59cc000u);                          // ldr r12, [r12]
		E(0xe35c0000u);                          // cmp r12, #0
		toc[2] = e_bcond(0x1);                   // bne -> C
		E(0xe2433001u);                          // sub r3, r3, #1
		E(ubfx(12, 11, 2, 10));                  // r12 = the 4-byte granule
		e_imm(0, (uint32_t)(uintptr_t)s_sj_bits);
		E(0xe0800383u);                          // add r0, r0, r3, lsl #7
		E(0xe7d001acu);                          // ldrb r0, [r0, r12, lsr #3]
		E(0xe20cc007u);                          // and r12, r12, #7
		E(0xe1a00c30u);                          // lsr r0, r0, r12
		E(0xe3100001u);                          // tst r0, #1
		toc[3] = e_bcond(0x1);                   // bne: translated -> C (the blocks go)
		e_imm(0, (uint32_t)(uintptr_t)s_sj_host);
		E(0xe7900103u);                          // ldr r0, [r0, r3, lsl #2] (its memory)
		if (size == 4) {
			E(ubfx(2, 11, 2, 10));
			E(0xe7800102u | RV);                 // str r1, [r0, r2, lsl #2]
		} else if (size == 2) {
			E(ubfx(2, 11, 1, 11));
			E(0xe2222001u);                      // eor r2, r2, #1
			E(0xe0800082u);                      // add r0, r0, r2, lsl #1
			E(0xe1c000b0u | RV);                 // strh r1, [r0]
		} else {
			E(ubfx(2, 11, 0, 12));
			E(0xe2222003u);                      // eor r2, r2, #3
			E(0xe7c00002u | RV);                 // strb r1, [r0, r2]
		}
		e_imm(0, (uint32_t)(uintptr_t)&s_sj_cpu->nslow);
		E(0xe5902000u);                          // ldr r2, [r0]
		E(0xe2822001u);                          // add r2, r2, #1
		E(0xe5802000u);                          // str r2, [r0]
		const int f = SJ_RCF ? 0 : 10;           // (the flag's register)
		e_ldr(f, COFS(test_irq));
		toc[4] = e_bcond(0xe);                   // b done
		for (int k = 0; k < 4; k++)
			e_patch(toc[k]);
		e_mov(0, 11);                            // (the C path: r0 = the address again)
		e_str(5, COFS(icount));
		e_mov(2, rv);
		e_mov(1, 0);
		e_mov(0, 4);
		e_pcv(3, pcv);
		e_call(size == 4 ? (const void *)sj_wr32 : size == 2 ? (const void *)sj_wr16 : (const void *)sj_wr8);
		if (f)
			e_mov(10, 0);
		e_ldr(5, COFS(icount));
		e_patch(toc[4]);
		if (SJ_RCF)
			e_flag_set(f);
		e_sentinel(f);
	}
	e_hot();
	s_r10 = SJ_OPT(OPT_SENT) ? 2 : 1;
	e_dpen(1);
}

/***************************************************************************
    operations
***************************************************************************/

#define EN ((int)((op >> 8) & 15))
#define EM ((int)((op >> 4) & 15))

// (OPT_KREG) the guest registers whose value the translation knows (bit k: R k = s_kv[k]); none at a block's start, an
// instruction's writes (sj_wmask: all when not listed) take them away, MOV #imm / a literal / MOVA / MOV / ADD #imm
// give them

static uint32_t sj_wmask(int h, uint32_t op)
{
	const uint32_t n = 1u << ((op >> 8) & 15), m = 1u << ((op >> 4) & 15);
	switch (h) {
	case H_NOP: case H_CMPEQ: case H_CMPHS: case H_CMPGE: case H_CMPHI: case H_CMPGT: case H_CMPPZ: case H_CMPPL:
	case H_CMPIM: case H_CMPSTR: case H_TST: case H_TSTI: case H_CLRT: case H_SETT: case H_CLRS: case H_SETS:
	case H_CLRMAC: case H_DIV0U: case H_DIV0S: case H_MULL: case H_MULU: case H_MULS: case H_DMULU: case H_DMULS:
	case H_LDS_MACH: case H_LDS_MACL: case H_LDS_PR: case H_LDC_GBR: case H_LDC_VBR: case H_LDC_SSR: case H_LDC_SPC:
	case H_LDC_DBR: case H_LDS_FPUL: case H_LDS_FPSCR: case H_LDC_RBANK:
	case H_MOVB_S: case H_MOVW_S: case H_MOVL_S: case H_MOVB_S0: case H_MOVW_S0: case H_MOVL_S0: case H_MOVL_S4:
	case H_MOVB_S4R0: case H_MOVW_S4R0: case H_MOVB_SG: case H_MOVW_SG: case H_MOVL_SG: case H_MOVCAL:
	case H_TSTB: case H_ANDB: case H_XORB: case H_ORB: case H_TAS:
		return 0;
	case H_MOV: case H_MOVI: case H_ADD: case H_ADDI: case H_SUB: case H_AND: case H_OR: case H_XOR: case H_NOT:
	case H_NEG: case H_NEGC: case H_EXTUB: case H_EXTUW: case H_EXTSB: case H_EXTSW: case H_SWAPB: case H_SWAPW:
	case H_XTRCT: case H_SHLL: case H_SHLR: case H_SHAR: case H_SHLL2: case H_SHLL8: case H_SHLL16: case H_SHLR2:
	case H_SHLR8: case H_SHLR16: case H_ROTL: case H_ROTR: case H_ROTCL: case H_ROTCR: case H_SHAD: case H_SHLD:
	case H_ADDC: case H_SUBC: case H_ADDV: case H_SUBV: case H_DT: case H_MOVT: case H_DIV1:
	case H_STS_MACH: case H_STS_MACL: case H_STS_PR: case H_STC_SR: case H_STC_GBR: case H_STC_VBR: case H_STC_SSR:
	case H_STC_SPC: case H_STC_SGR: case H_STC_DBR: case H_STC_RBANK: case H_STS_FPUL: case H_STS_FPSCR:
	case H_MOVB_L: case H_MOVW_L: case H_MOVL_L: case H_MOVB_L0: case H_MOVW_L0: case H_MOVL_L0: case H_MOVL_L4:
	case H_MOVW_I: case H_MOVL_I:
	case H_MOVB_M: case H_MOVW_M: case H_MOVL_M: case H_STSM_MACH: case H_STSM_MACL: case H_STSM_PR: case H_STCM_SR:
	case H_STCM_GBR: case H_STCM_VBR: case H_STCM_RBANK: case H_STCM_SGR: case H_STCM_SSR: case H_STCM_SPC:
	case H_STSM_FPUL: case H_STSM_FPSCR: case H_STCM_DBR:
	case H_LDSM_MACH: case H_LDSM_MACL: case H_LDSM_PR: case H_LDCM_GBR: case H_LDCM_VBR: case H_LDCM_RBANK:
	case H_LDCM_SSR: case H_LDCM_SPC: case H_LDSM_FPUL: case H_LDSM_FPSCR: case H_LDCM_DBR:
		return n;
	case H_MOVB_P: case H_MOVW_P: case H_MOVL_P: case H_MACL: case H_MACW:
		return n | m;
	case H_ANDI: case H_ORI: case H_XORI: case H_MOVA: case H_MOVB_L4R0: case H_MOVW_L4R0: case H_MOVB_LG:
	case H_MOVW_LG: case H_MOVL_LG:
		return 1;
	default:
		return 0xffff;
	}
}
// after the instruction (h, op): what it wrote is not known any more, unless it is one of these (lit: a literal was
// folded, s_litv its value)
static uint32_t s_litv;
static void sj_kafter(int h, uint32_t op, uint32_t pcv, int lit)
{
	if (!SJ_OPT(OPT_KREG))
		return;
	const int n = (int)((op >> 8) & 15), m = (int)((op >> 4) & 15);
	const uint32_t was = s_kmask, vm = s_kv[m], vn = s_kv[n];
	s_kmask &= ~sj_wmask(h, op);
	switch (h) {
	case H_MOVI: s_kv[n] = SEXT8(op); s_kmask |= 1u << n; break;
	case H_MOVW_I: case H_MOVL_I: if (lit) { s_kv[n] = s_litv; s_kmask |= 1u << n; } break;
	case H_MOVA: if (!s_dyn) { s_kv[0] = ((pcv + 2) & ~3u) + (op & 0xff) * 4; s_kmask |= 1; } break;
	case H_MOV: if ((was >> m) & 1) { s_kv[n] = vm; s_kmask |= 1u << n; } break;
	case H_ADDI: if ((was >> n) & 1) { s_kv[n] = vn + SEXT8(op); s_kmask |= 1u << n; } break;
	default: break;
	}
}
// (OPT_KREG) a load of size bytes from the known address a (the unmasked address: the penalty's), when its page has
// a read entry (only sh3_map changes those, and it throws the blocks away): read from it, the access's timing as
// e_load's. 0: not here
static int e_kload(uint32_t a, int size, int sx, int dg)
{
	const sh3_t *const c = s_sj_cpu;
	if (a >= 0xe0000000u)
		return 0;
	const uint8_t *const p = c->rd[PAGE(a & SH34_AM)];
	if (!p)
		return 0;
	const int v = e_rd(dg, 0);
	const uint32_t V = (uint32_t)v << 12;
	if (size == 4) {
		e_imm(3, (uint32_t)(uintptr_t)(p + (a & 0xffc)));
		E(0xe5930000u | V);                      // ldr r0, [r3]
	} else if (size == 2) {
		e_imm(3, (uint32_t)(uintptr_t)(p + ((a & 0xffe) ^ 2)));
		E((sx ? 0xe1d300f0u : 0xe1d300b0u) | V);   // ldrsh / ldrh r0, [r3]
	} else {
		e_imm(3, (uint32_t)(uintptr_t)(p + ((a & 0xfff) ^ 3)));
		E((sx ? 0xe1d300d0u : 0xe5d30000u) | V);   // ldrsb / ldrb r0, [r3]
	}
	e_rw(dg, v);
	e_imm(11, a);
	e_dpen(0);
	return 1;
}
#define SJ_K(r) (SJ_OPT(OPT_KREG) && ((s_kmask >> (r)) & 1))

// (OPT_LIT) a MOV.W / MOV.L @(disp,PC) whose literal at la is in the block's page, read through its read entry (the
// interpreter's fast path): the value as it is now, its 4-byte granule marked translated (a write to it throws the
// blocks away as one over code does; a ROM page: no write reaches it). The access's timing as e_load's
static uint32_t s_bpg;           // (the block's page, its memory, RAM)
static const uint8_t *s_bpage;
static int s_bram;
static uint32_t s_lit[2 * SJ_MAX_INSNS];
static unsigned s_nlit;
static int s_lit_done;   // (the instruction's literal was folded)
static int e_literal(uint32_t la, int size, int dg)
{
	const sh3_t *const c = s_sj_cpu;
	if (!SJ_OPT(OPT_LIT) || la >= 0xe0000000u || PAGE(la & SH34_AM) != s_bpg || !s_bpage || c->rd[s_bpg] != s_bpage ||
	    s_nlit >= sizeof(s_lit) / sizeof(s_lit[0]))
		return 0;
	const uint32_t v = size == 4 ? *(const uint32_t *)(s_bpage + (la & 0xffc)) :
	                   SEXT16(*(const uint16_t *)(s_bpage + ((la & 0xffe) ^ 2)));
	const int r = e_rd(dg, 0);
	e_imm(r, v);
	e_rw(dg, r);
	e_imm(11, la);
	e_dpen(0);
	s_litv = v;
	s_lit_done = 1;
	if (s_bram)
		s_lit[s_nlit++] = la;
	return 1;
}

// (OPT_RC) the address g + disp of an access in R (r11, or r0 without OPT_A11), or g's host register itself (disp 0)
static int e_addr(int g, uint32_t disp, int R)
{
	const int a = e_rg(g, R);
	if (!disp)
		return a;
	e_addi(R, a, disp);
	return R;
}
#define RN_ (1u << 16)
#define RD_ (1u << 12)
// d(n) = insn(a, b): an ALU operation with Rd at bit 12, its first operand at 16, its second at 0
static void e_op(int n, int a, int b, uint32_t insn)
{
	const int d = e_rd(n, 0);
	E(insn | (uint32_t)a << 16 | (uint32_t)d << 12 | (uint32_t)b);
	e_rw(n, d);
}

// the operation of h natively: 1 done, 0 not (sj_exec does it)
static int e_native(uint32_t op, int h, uint32_t pcv)
{
	if (s_sj_dbg & 1)
		return 0;
	const int n = EN, m = EM;
	const uint32_t i8 = op & 0xff, s8 = SEXT8(op);
	const int mem = !(s_sj_dbg & 4);
	switch (h) {
	case H_NOP: return 1;
	case H_MOV: e_rw(n, e_rg(m, 0)); return 1;
	case H_MOVI: { const int d = e_rd(n, 0); e_imm(d, s8); e_rw(n, d); return 1; }
	case H_MOVA: {
		if (s_dyn) return 0;
		const int d = e_rd(0, 0);
		e_imm(d, ((pcv + 2) & ~3u) + i8 * 4); e_rw(0, d); return 1;
	}
	case H_MOVT: { const int t = e_rg(RC_SR, 0); e_op(n, t, 0, 0xe2000001u); return 1; }   // and r0, r0, #1
	case H_ADD: { const int a = e_rg(n, 0), b = e_rg(m, 1); e_op(n, a, b, 0xe0800000u); return 1; }
	case H_ADDI: {
		const int a = e_rg(n, 0), d = e_rd(n, 0);
		e_addi(d, a, s8); e_rw(n, d); return 1;
	}
	case H_SUB: { const int a = e_rg(n, 0), b = e_rg(m, 1); e_op(n, a, b, 0xe0400000u); return 1; }
	case H_AND: { const int a = e_rg(n, 0), b = e_rg(m, 1); e_op(n, a, b, 0xe0000000u); return 1; }
	case H_OR: { const int a = e_rg(n, 0), b = e_rg(m, 1); e_op(n, a, b, 0xe1800000u); return 1; }
	case H_XOR: { const int a = e_rg(n, 0), b = e_rg(m, 1); e_op(n, a, b, 0xe0200000u); return 1; }
	case H_NOT: e_op(n, 0, e_rg(m, 0), 0xe1e00000u); return 1;        // mvn r0, r0
	case H_NEG: e_op(n, e_rg(m, 0), 0, 0xe2600000u); return 1;        // rsb r0, r0, #0
	case H_EXTUB: e_op(n, 0, e_rg(m, 0), 0xe6ef0070u); return 1;
	case H_EXTUW: e_op(n, 0, e_rg(m, 0), 0xe6ff0070u); return 1;
	case H_EXTSB: e_op(n, 0, e_rg(m, 0), 0xe6af0070u); return 1;
	case H_EXTSW: e_op(n, 0, e_rg(m, 0), 0xe6bf0070u); return 1;
	case H_SWAPB: {
		const int a = e_rg(m, 0), d = e_rd(n, 0);
		E(0xe6bf1fb0u | (uint32_t)a);   // rev16 r1, r0
		if (d != a) e_mov(d, a);
		E(bfi(d, 1, 0, 16));            // r0[15:0] = r1[15:0]
		e_rw(n, d);
		return 1;
	}
	case H_SWAPW: e_op(n, 0, e_rg(m, 0), 0xe1a00860u); return 1;      // ror r0, r0, #16
	case H_XTRCT: {
		const int a = e_rg(n, 0), b = e_rg(m, 1);
		int d = e_rd(n, 0);
		if (d == b) d = 0;   // (n == m)
		E(0xe1a00820u | (uint32_t)d << 12 | (uint32_t)a);                      // lsr r0, r0, #16
		E(0xe1800800u | (uint32_t)d << 16 | (uint32_t)d << 12 | (uint32_t)b);  // orr r0, r0, r1, lsl #16
		e_rw(n, d);
		return 1;
	}
	case H_SHLL2: case H_SHLL8: case H_SHLL16: case H_SHLR2: case H_SHLR8: case H_SHLR16: {
		const uint32_t sh = (h == H_SHLL2 || h == H_SHLR2) ? 2 : (h == H_SHLL8 || h == H_SHLR8) ? 8 : 16;
		const int right = h == H_SHLR2 || h == H_SHLR8 || h == H_SHLR16;
		e_op(n, 0, e_rg(n, 0), 0xe1a00000u | sh << 7 | (right ? 0x20u : 0));
		return 1;
	}
	case H_SHLL: case H_SHLR: case H_SHAR: {
		const int a = e_rg(n, 0), t = e_rg(RC_SR, 2), d = e_rd(n, 0), dt = e_rd(RC_SR, 2);
		E((h == H_SHLL ? 0xe1b00080u : h == H_SHLR ? 0xe1b000a0u : 0xe1b000c0u) | (uint32_t)d << 12 | (uint32_t)a);   // lsls / lsrs / asrs r0, r0, #1
		E(0xe3c00001u | (uint32_t)t << 16 | (uint32_t)dt << 12);   // bic r2, r2, #1
		E(0x23800001u | (uint32_t)dt << 16 | (uint32_t)dt << 12);  // orrcs r2, r2, #1
		e_rw(n, d);
		e_rw(RC_SR, dt);
		e_tflags(0x2);
		return 1;
	}
	case H_ROTL: {
		const int a = e_rg(n, 0), t = e_rg(RC_SR, 2), d = e_rd(n, 0), dt = e_rd(RC_SR, 2);
		E(0xe1a00fe0u | (uint32_t)d << 12 | (uint32_t)a);   // ror r0, r0, #31
		if (dt != t) e_mov(dt, t);
		E(bfi(dt, d, 0, 1));
		e_rw(n, d); e_rw(RC_SR, dt);
		return 1;
	}
	case H_ROTR: {
		const int a = e_rg(n, 0), t = e_rg(RC_SR, 2), d = e_rd(n, 0), dt = e_rd(RC_SR, 2);
		if (dt != t) e_mov(dt, t);
		E(bfi(dt, a, 0, 1));
		E(0xe1a000e0u | (uint32_t)d << 12 | (uint32_t)a);   // ror r0, r0, #1
		e_rw(n, d); e_rw(RC_SR, dt);
		return 1;
	}
	case H_ROTCL: {
		const int a = e_rg(n, 0), t = e_rg(RC_SR, 2), d = e_rd(n, 0), dt = e_rd(RC_SR, 2);
		E(0xe1a01fa0u | (uint32_t)a);        // lsr r1, r0, #31
		E(0xe2003001u | (uint32_t)t << 16);  // and r3, r2, #1
		E(0xe1830080u | (uint32_t)d << 12 | (uint32_t)a);   // orr r0, r3, r0, lsl #1
		if (dt != t) e_mov(dt, t);
		E(bfi(dt, 1, 0, 1));
		e_rw(n, d); e_rw(RC_SR, dt);
		return 1;
	}
	case H_ROTCR: {
		const int a = e_rg(n, 0), t = e_rg(RC_SR, 2), d = e_rd(n, 0), dt = e_rd(RC_SR, 2);
		E(0xe2003001u | (uint32_t)t << 16);  // and r3, r2, #1
		if (dt != t) e_mov(dt, t);
		E(bfi(dt, a, 0, 1));
		E(0xe1a000a0u | (uint32_t)d << 12 | (uint32_t)a);   // lsr r0, r0, #1
		E(0xe1800f83u | (uint32_t)d << 16 | (uint32_t)d << 12);   // orr r0, r0, r3, lsl #31
		e_rw(n, d); e_rw(RC_SR, dt);
		return 1;
	}
	case H_SHAD: case H_SHLD: {
		const int a = e_rg(n, 0), b = e_rg(m, 1), d = e_rd(n, 0);
		E(0xe3100102u | (uint32_t)b << 16);  // tst r1, #0x80000000
		E(0xe200201fu | (uint32_t)b << 16);  // and r2, r1, #0x1f
		E(0x12622020u);        // rsbne r2, r2, #32
		E(0x01a00210u | (uint32_t)d << 12 | (uint32_t)a);   // lsleq r0, r0, r2
		E((h == H_SHAD ? 0x11a00250u : 0x11a00230u) | (uint32_t)d << 12 | (uint32_t)a);   // asrne / lsrne r0, r0, r2
		e_rw(n, d);
		return 1;
	}
	case H_CMPEQ: case H_CMPHS: case H_CMPGE: case H_CMPHI: case H_CMPGT: case H_TST: {
		const int a = e_rg(n, 0), b = e_rg(m, 1);
		E((h == H_TST ? 0xe1100000u : 0xe1500000u) | (uint32_t)a << 16 | (uint32_t)b);   // cmp / tst r0, r1
		e_sett(h == H_CMPEQ || h == H_TST ? 0x0 : h == H_CMPHS ? 0x2 : h == H_CMPGE ? 0xa : h == H_CMPHI ? 0x8 : 0xc);
		return 1;
	}
	case H_CMPPZ: case H_CMPPL:
		E(0xe3500000u | (uint32_t)e_rg(n, 0) << 16);   // cmp r0, #0
		e_sett(h == H_CMPPZ ? 0xa : 0xc);
		return 1;
	case H_CMPIM: { const int a = e_rg(0, 0); e_imm(1, s8); E(0xe1500001u | (uint32_t)a << 16); e_sett(0x0); return 1; }
	case H_CMPSTR: {
		const int a = e_rg(n, 0), b = e_rg(m, 1);
		E(0xe0200000u | (uint32_t)a << 16 | (uint32_t)b);   // eor r0, r0, r1
		E(0xe31004ffu);        // tst r0, #0xff000000
		E(0x131008ffu);        // tstne r0, #0xff0000
		E(0x13100cffu);        // tstne r0, #0xff00
		E(0x131000ffu);        // tstne r0, #0xff
		e_sett(0x0);
		return 1;
	}
	case H_TSTI: E(0xe3100000u | (uint32_t)e_rg(0, 0) << 16 | i8); e_sett(0x0); return 1;
	case H_ANDI: e_op(0, e_rg(0, 0), 0, 0xe2000000u | i8); return 1;
	case H_ORI: e_op(0, e_rg(0, 0), 0, 0xe3800000u | i8); return 1;
	case H_XORI: e_op(0, e_rg(0, 0), 0, 0xe2200000u | i8); return 1;
	case H_DT: e_op(n, e_rg(n, 0), 0, 0xe2500001u); e_sett(0x0); return 1;   // subs r0, r0, #1
	case H_CLRT: case H_SETT: case H_DIV0U: {
		const int t = e_rg(RC_SR, 2), dt = e_rd(RC_SR, 2);
		if (h == H_DIV0U) {
			E(0xe3c00c03u | (uint32_t)t << 16 | (uint32_t)dt << 12);   // bic r2, r2, #0x300
			E(0xe3c00001u | (uint32_t)dt << 16 | (uint32_t)dt << 12);  // bic r2, r2, #1
		} else
			E((h == H_SETT ? 0xe3800001u : 0xe3c00001u) | (uint32_t)t << 16 | (uint32_t)dt << 12);   // orr / bic r2, r2, #1
		e_rw(RC_SR, dt);
		return 1;
	}
	case H_DIV0S: {
		const int a = e_rg(n, 0), b = e_rg(m, 1), t = e_rg(RC_SR, 2), dt = e_rd(RC_SR, 2);
		const uint32_t D = (uint32_t)dt << 16 | (uint32_t)dt << 12;
		E(0xe3c00c03u | (uint32_t)t << 16 | (uint32_t)dt << 12); E(0xe3c00001u | D);   // bic r2, #0x301
		E(0xe1a03fa0u | (uint32_t)a);        // lsr r3, r0, #31
		E(0xe1800403u | D);                  // orr r2, r2, r3, lsl #8
		E(0xe1a03fa0u | (uint32_t)b);        // lsr r3, r1, #31
		E(0xe1800483u | D);                  // orr r2, r2, r3, lsl #9
		E(0xe0203000u | (uint32_t)a << 16 | (uint32_t)b);   // eor r3, r0, r1
		E(0xe1800fa3u | D);                  // orr r2, r2, r3, lsr #31
		e_rw(RC_SR, dt);
		return 1;
	}
	case H_DIV1:
		e_mov(0, 4);
		e_imm(1, op);
		e_call((const void *)sj_div1);   // (OPT_RC: the guest registers written back first)
		e_rc_kill(1u << n | 1u << RC_SR);
		return 1;
	case H_ADDC: case H_SUBC: {
		const int a = e_rg(n, 0), b = e_rg(m, 1), t = e_rg(RC_SR, 2), d = e_rd(n, 0), dt = e_rd(RC_SR, 2);
		const uint32_t ab = (uint32_t)a << 16 | (uint32_t)d << 12 | (uint32_t)b;
		if (h == H_ADDC) {
			E(0xe1b030a0u | (uint32_t)t);   // lsrs r3, r2, #1 (C = T)
			E(0xe0b00000u | ab);            // adcs r0, r0, r1
			E(0xe3c00001u | (uint32_t)t << 16 | (uint32_t)dt << 12);
			E(0x23800001u | (uint32_t)dt << 16 | (uint32_t)dt << 12);   // orrcs
		} else {
			E(0xe2003001u | (uint32_t)t << 16);   // and r3, r2, #1
			E(0xe2733000u);    // rsbs r3, r3, #0 (C = !T)
			E(0xe0d00000u | ab);            // sbcs r0, r0, r1
			E(0xe3c00001u | (uint32_t)t << 16 | (uint32_t)dt << 12);
			E(0x33800001u | (uint32_t)dt << 16 | (uint32_t)dt << 12);   // orrcc
		}
		e_rw(n, d);
		e_rw(RC_SR, dt);
		return 1;
	}
	case H_NEGC: {
		const int b = e_rg(m, 1), t = e_rg(RC_SR, 2), d = e_rd(n, 0), dt = e_rd(RC_SR, 2);
		E(0xe2003001u | (uint32_t)t << 16);  // and r3, r2, #1
		E(0xe2733000u);        // rsbs r3, r3, #0 (C = !T)
		E(0xe2f00000u | (uint32_t)b << 16 | (uint32_t)d << 12);   // rscs r0, r1, #0
		E(0xe3c00001u | (uint32_t)t << 16 | (uint32_t)dt << 12);
		E(0x33800001u | (uint32_t)dt << 16 | (uint32_t)dt << 12);   // orrcc
		e_rw(n, d);
		e_rw(RC_SR, dt);
		return 1;
	}
	case H_MULL: { const int a = e_rg(n, 0), b = e_rg(m, 1); E(0xe0000090u | (uint32_t)b << 8 | (uint32_t)a); e_str(0, COFS(macl)); return 1; }   // mul r0, r0, r1
	case H_MULU: case H_MULS: {
		const int a = e_rg(n, 0), b = e_rg(m, 1);
		E((h == H_MULU ? 0xe6ff0070u : 0xe6bf0070u) | (uint32_t)a);          // uxth / sxth r0, r0
		E((h == H_MULU ? 0xe6ff1070u : 0xe6bf1070u) | (uint32_t)b);          // uxth / sxth r1, r1
		E(0xe0000190u); e_str(0, COFS(macl));
		return 1;
	}
	case H_DMULU: case H_DMULS: {
		const int a = e_rg(n, 0), b = e_rg(m, 1);
		E((h == H_DMULU ? 0xe0832090u : 0xe0c32090u) | (uint32_t)b << 8 | (uint32_t)a);   // umull / smull r2, r3, r0, r1
		e_str(3, COFS(mach));
		e_str(2, COFS(macl));
		return 1;
	}
	case H_STS_MACH: case H_STS_MACL: case H_STS_PR: case H_STC_GBR: {
		const int d = e_rd(n, 0);
		e_ldr(d, h == H_STS_MACH ? COFS(mach) : h == H_STS_MACL ? COFS(macl) : h == H_STS_PR ? COFS(pr) : COFS(gbr));
		e_rw(n, d);
		return 1;
	}
	case H_STC_SR: e_rw(n, e_rg(RC_SR, 0)); return 1;
	case H_LDS_MACH: e_str(e_rg(n, 0), COFS(mach)); return 1;
	case H_LDS_MACL: e_str(e_rg(n, 0), COFS(macl)); return 1;
	case H_LDS_PR: e_str(e_rg(n, 0), COFS(pr)); return 1;
	case H_LDC_GBR: e_str(e_rg(n, 0), COFS(gbr)); return 1;
	case H_CLRMAC: E(0xe3a00000u); e_str(0, COFS(mach)); e_str(0, COFS(macl)); return 1;
	default: break;
	}
	if (!mem)
		return 0;
	const int R = SJ_OPT(OPT_A11) ? 11 : 0;   // (the address's register)
	switch (h) {
	// ---- loads
	case H_MOVB_L: case H_MOVW_L: case H_MOVL_L:
		if (SJ_K(m) && e_kload(s_kv[m], h == H_MOVB_L ? 1 : h == H_MOVW_L ? 2 : 4, 1, n))
			return 1;
		e_load(h == H_MOVB_L ? 1 : h == H_MOVW_L ? 2 : 4, 1, n, 0, pcv, e_addr(m, 0, R), -1, 0);
		return 1;
	case H_MOVB_L0: case H_MOVW_L0: case H_MOVL_L0: {
		const int a = e_rg(m, 0), b = e_rg(0, 1);
		E(0xe0800000u | (uint32_t)a << 16 | (uint32_t)R << 12 | (uint32_t)b);   // add R, r0, r1
		e_load(h == H_MOVB_L0 ? 1 : h == H_MOVW_L0 ? 2 : 4, 1, n, 0, pcv, R, -1, 0);
		return 1;
	}
	case H_MOVL_L4:
		if (SJ_K(m) && e_kload(s_kv[m] + (op & 15) * 4, 4, 0, n))
			return 1;
		e_load(4, 0, n, 0, pcv, e_addr(m, (op & 15) * 4, R), -1, 0);
		return 1;
	case H_MOVB_L4R0: case H_MOVW_L4R0:
		if (SJ_K(m) && e_kload(s_kv[m] + (op & 15) * (h == H_MOVB_L4R0 ? 1u : 2u), h == H_MOVB_L4R0 ? 1 : 2, 1, 0))
			return 1;
		e_load(h == H_MOVB_L4R0 ? 1 : 2, 1, 0, 0, pcv, e_addr(m, (op & 15) * (h == H_MOVB_L4R0 ? 1u : 2u), R), -1, 0);
		return 1;
	case H_MOVB_LG: case H_MOVW_LG: case H_MOVL_LG: {
		const uint32_t sz = h == H_MOVB_LG ? 1 : h == H_MOVW_LG ? 2 : 4;
		e_ldr(R, COFS(gbr)); e_addi(R, R, i8 * sz);
		e_load((int)sz, 1, 0, 0, pcv, R, -1, 0);
		return 1;
	}
	case H_MOVW_I: case H_MOVL_I: {
		if (s_dyn) return 0;
		const uint32_t la = h == H_MOVW_I ? pcv + i8 * 2 + 2 : ((pcv + 2) & ~3u) + i8 * 4;
		if (e_literal(la, h == H_MOVW_I ? 2 : 4, n))
			return 1;
		e_imm(R, la);
		e_load(h == H_MOVW_I ? 2 : 4, h == H_MOVW_I, n, 0, pcv, R, -1, 0);
		return 1;
	}
	case H_MOVB_P: case H_MOVW_P: case H_MOVL_P: {
		const int sz = h == H_MOVB_P ? 1 : h == H_MOVW_P ? 2 : 4;
		const int inc = SJ_RCF && n != m;   // (OPT_RC: the post-increment in e_load, before the penalty)
		e_load(sz, 1, n, 0, pcv, e_addr(m, 0, R), inc ? m : -1, (uint32_t)sz);
		if (n != m && !inc) {
			if (R == 11) E(0xe28b0000u | (uint32_t)sz);   // add r0, r11, #sz (r11: the address, Rm before)
			else { e_ldr(0, RO(m)); E(0xe2800000u | (uint32_t)sz); }
			e_str(0, RO(m));
		}
		return 1;
	}
	case H_LDSM_MACH: case H_LDSM_MACL: case H_LDSM_PR: case H_LDCM_GBR: {
		const uint32_t f = h == H_LDSM_MACH ? COFS(mach) : h == H_LDSM_MACL ? COFS(macl) : h == H_LDSM_PR ? COFS(pr) : COFS(gbr);
		e_load(4, 0, -1, f, pcv, e_addr(n, 0, R), SJ_RCF ? n : -1, 4);
		if (!SJ_RCF) {
			if (R == 11) E(0xe28b0004u);   // add r0, r11, #4
			else { e_ldr(0, RO(n)); E(0xe2800004u); }   // add r0, r0, #4
			e_str(0, RO(n));
		}
		return 1;
	}
	// ---- stores
	case H_MOVB_S: case H_MOVW_S: case H_MOVL_S: {
		const int a = e_addr(n, 0, R);
		e_store(h == H_MOVB_S ? 1 : h == H_MOVW_S ? 2 : 4, pcv, a, e_rg(m, 1));
		return 1;
	}
	case H_MOVB_S0: case H_MOVW_S0: case H_MOVL_S0: {
		const int a = e_rg(n, 0), b = e_rg(0, 1);
		E(0xe0800000u | (uint32_t)a << 16 | (uint32_t)R << 12 | (uint32_t)b);   // add R, r0, r1
		e_store(h == H_MOVB_S0 ? 1 : h == H_MOVW_S0 ? 2 : 4, pcv, R, e_rg(m, 1));
		return 1;
	}
	case H_MOVL_S4: {
		const int a = e_addr(n, (op & 15) * 4, R);
		e_store(4, pcv, a, e_rg(m, 1));
		return 1;
	}
	case H_MOVB_S4R0: case H_MOVW_S4R0: {
		const int a = e_addr(m, (op & 15) * (h == H_MOVB_S4R0 ? 1u : 2u), R);
		e_store(h == H_MOVB_S4R0 ? 1 : 2, pcv, a, e_rg(0, 1));
		return 1;
	}
	case H_MOVB_SG: case H_MOVW_SG: case H_MOVL_SG: {
		const uint32_t sz = h == H_MOVB_SG ? 1 : h == H_MOVW_SG ? 2 : 4;
		e_ldr(R, COFS(gbr)); e_addi(R, R, i8 * sz);
		e_store((int)sz, pcv, R, e_rg(0, 1));
		return 1;
	}
	case H_MOVB_M: case H_MOVW_M: case H_MOVL_M: {
		const uint32_t sz = h == H_MOVB_M ? 1 : h == H_MOVW_M ? 2 : 4;
		int v = e_rg(m, 1);
		if (m == n && v != 1) {   // (the value before the decrement)
			e_mov(1, v);
			v = 1;
		}
		const int a = e_rg(n, 0);
		E(0xe2400000u | (uint32_t)a << 16 | (uint32_t)R << 12 | sz);   // sub R, r0, #sz
		e_rw(n, R);
		e_store((int)sz, pcv, R, v);
		return 1;
	}
	case H_MOVCAL: {
		const int a = e_addr(n, 0, R);
		e_store(4, pcv, a, e_rg(0, 1));
		return 1;
	}
	case H_STSM_MACH: case H_STSM_MACL: case H_STSM_PR: case H_STCM_GBR: case H_STCM_SR: {
		const uint32_t f = h == H_STSM_MACH ? COFS(mach) : h == H_STSM_MACL ? COFS(macl) : h == H_STSM_PR ? COFS(pr) :
		                   COFS(gbr);
		const int a = e_rg(n, 0);
		E(0xe2400004u | (uint32_t)a << 16 | (uint32_t)R << 12);       // sub R, r0, #4
		e_rw(n, R);
		int v = 1;
		if (h == H_STCM_SR)
			v = e_rg(RC_SR, 1);
		else
			e_ldr(1, f);
		e_store(4, pcv, R, v);
		return 1;
	}
	default:
		return 0;
	}
}

/***************************************************************************
    instructions and blocks
***************************************************************************/

// the interpreter's GO (the pc flush, arg0, the instruction fetch penalty), the operation (native or sj_exec);
// pcv: the instruction's pc (its address + 2, a slot's: the target; c->pc when s_dyn)
static void e_go(sh3_t *c, uint32_t op, int h, uint32_t pcv)
{
	if (c->flush_pc != 1) {   // (MAME sh2drc_add_pcflush: pc == flush_pc + 2)
		if (!s_dyn) {
			if (pcv == c->flush_pc + 2) {
				e_imm(0, pcv - 2);
				e_strf(0, COFS(drc_pc));
			}
		} else {
			e_ldr(0, COFS(pc));
			e_imm(1, c->flush_pc + 2);
			E(0xe1500001u);                 // cmp r0, r1
			const unsigned b = e_bcond(0x1);
			E(0xe2400002u);                 // sub r0, r0, #2
			e_strf(0, COFS(drc_pc));
			e_patch(b);
		}
	}
	if (op_helper(h)) {
		e_imm(0, op);
		e_strf(0, COFS(arg0));
	}
	if ((0x8011u >> (op >> 12)) & 1)
		e_ipen();
}
static void e_insn(sh3_t *c, uint32_t op, int h, uint32_t pcv)
{
	e_go(c, op, h, pcv);
	if (e_native(op, h, pcv))
		return;
	const int t = sj_touches(h);
	if (t)
		e_str(5, COFS(icount));
	e_mov(0, 4);
	e_imm(1, op);
	e_imm(2, (uint32_t)h);
	e_pcv(3, pcv);
	e_call((const void *)sj_exec);   // (OPT_RC: the guest registers written back first)
	e_rc_kill(~0u);                  // (any of them may have changed)
	if (t) {
		e_flag_set(0);
		e_ldr(5, COFS(icount));
		s_r10 = 1;
	}
}

static inline uint16_t sj_op(const uint8_t *page, uint32_t a) { return *(const uint16_t *)(page + ((a & 0xffe) ^ 2)); }

static unsigned s_nmark;
static uint32_t s_mark[2 * SJ_MAX_INSNS + 4];

// r0 = a delayed transfer's target computed at run time: 0 becomes a + 4. The interpreter keeps the pending target
// in delay with 0 for none (MAME's sh_common_execution m_delay), so a transfer to address 0 does not happen there:
// the slot runs with the pc after it and the code goes on after the slot (only pr changed by JSR / BSRF)
static void e_dyn0(uint32_t a)
{
	E(0xe3500000u);   // cmp r0, #0
	if (s_nhot) {     // (other cold branches pending: in line)
		const unsigned nz = e_bcond(0x1);   // bne
		e_imm(0, a + 4);
		e_patch(nz);
		return;
	}
	e_tocold(0x0);    // beq
	e_cold();
	e_imm(0, a + 4);
	e_hot();
}

// a delayed transfer at a (h, op) with its slot (hs, sop) at a + 2: the slot run with the target as its pc, the
// count-off, the exit to the target (a target of 0: a + 4, see e_dyn0)
static void e_delayed(sh3_t *c, uint32_t a, uint32_t op, int h, uint32_t sop, int hs)
{
	const int n = EN;
	e_go(c, op, h, a + 2);   // (the icache of groups 0 and 4; the transfer's operation below)
	uint32_t t = 0;
	int dyn = 0;
	unsigned nt = 0;
	switch (h) {
	case H_BRA: t = a + 4 + SEXT12(op & 0xfff) * 2; break;
	case H_BSR: t = a + 4 + SEXT12(op & 0xfff) * 2; e_imm(0, a + 4); e_str(0, COFS(pr)); break;
	case H_BTS: case H_BFS:
		t = a + 4 + SEXT8(op) * 2;
		nt = e_tbranch(h == H_BTS);         // (not taken)
		break;
	case H_RTS: e_ldr(0, COFS(pr)); e_dyn0(a); e_str(0, COFS(pc)); dyn = 1; break;
	case H_JMP: case H_JSR:
		if (h == H_JSR) { e_imm(1, a + 4); e_str(1, COFS(pr)); }
		if (SJ_K(n)) { t = s_kv[n]; break; }   // (OPT_KREG: as BRA)
		{   // (the register cache's copy into r0, then a target of 0 falls through: ADR-135 with ADR-140)
			const int rr = e_rg(n, 0);
			if (rr != 0)
				e_mov(0, rr);
		}
		e_dyn0(a); e_str(0, COFS(pc)); dyn = 1;
		break;
	case H_BRAF: case H_BSRF:
		if (h == H_BSRF) { e_imm(1, a + 4); e_str(1, COFS(pr)); }
		if (SJ_K(n)) { t = s_kv[n] + a + 4; break; }
		e_addi(0, e_rg(n, 0), a + 4); e_dyn0(a); e_str(0, COFS(pc)); dyn = 1;
		break;
	default: break;
	}
	if (!dyn && t == 0)   // (a target of 0: no transfer, as for e_dyn0)
		t = a + 4;
	const uint32_t pend0 = s_pend;
	const int pdyn0 = s_pdyn;
	s_pend += (uint32_t)op_cost(h);
	s_dyn = dyn;
	const uint32_t kmask0 = s_kmask;   // (not taken: the slot is translated again as an instruction)
	const uint32_t rcl0 = s_rcl, rcd0 = s_rcd;
	s_fuse_ok = 1; s_fcost = (uint32_t)op_cost(hs); s_fnext = t; s_fdyn = dyn; s_fused = 0;
	s_sj_inslot = 1;
	e_insn(c, sop, hs, t);
	s_sj_inslot = 0;
	s_fuse_ok = 0;
	if (s_fused)
		s_fused = 0;   // (OPT_FUSE: counted off and tested in its access)
	else {
		s_pend += (uint32_t)op_cost(hs);
		e_update_irq(t, dyn, 1);
	}
	e_exit(t, dyn, 1, 1);
	s_dyn = 0;
	s_kmask = kmask0;
	s_rcl = rcl0;
	s_rcd = rcd0;
	if (h == H_BTS || h == H_BFS) {   // not taken: the cycles gathered are dropped, on with the slot as an instruction
		e_patch(nt);
		(void)pend0; (void)pdyn0;
		s_pend = 0;
		s_pdyn = 0;
		s_r10 = 0;
	}
}

// translate the block at pc: NULL = none now, SJ_NONE = none at that pc
// (OPT_RC) the guest registers a block uses most get the host registers: its instructions as sj_build will take
// them, each reading or writing one counted
static uint32_t sj_rc_uses(int h, uint32_t op)
{
	const uint32_t n = 1u << ((op >> 8) & 15), m = 1u << ((op >> 4) & 15), T = 1u << RC_SR;
	uint32_t u = 0;
	switch (op >> 12) {
	case 0x0: u = ((op & 15) >= 4 && (op & 15) <= 7) || (op & 15) >= 12 ? n | m | 1 : n; break;
	case 0x1: case 0x2: case 0x3: case 0x6: u = n | m; break;
	case 0x4: case 0x7: case 0x9: case 0xd: case 0xe: u = n; break;
	case 0x5: u = n | m; break;
	case 0x8: u = (op & 0x0f00) <= 0x0500 ? 1 | m : 1; break;
	case 0xc: u = 1; break;
	default: break;
	}
	switch (h) {
	case H_CMPEQ: case H_CMPHS: case H_CMPGE: case H_CMPHI: case H_CMPGT: case H_CMPPZ: case H_CMPPL: case H_CMPIM:
	case H_CMPSTR: case H_TST: case H_TSTI: case H_DT: case H_BT: case H_BF: case H_BTS: case H_BFS: case H_MOVT:
	case H_SHLL: case H_SHLR: case H_SHAR: case H_ROTL: case H_ROTR: case H_ROTCL: case H_ROTCR: case H_ADDC:
	case H_SUBC: case H_NEGC: case H_CLRT: case H_SETT:
		u |= T;
		break;
	default: break;
	}
	return u;
}
static int s_sj_rcdbg = 7, s_sj_rcmax = -1, s_sj_rcmin = 3;   // (RBX_SH3_RCDBG: bits 0-2 the host registers r10, lr, r6, 8 not SR;
                                               // RBX_SH3_RCMAX: only the first n blocks)
static void sj_rc_alloc(const uint8_t *page, uint32_t pc, uint32_t pg)
{
	if (!SJ_OPT(OPT_RC) || !SJ_OPT(OPT_A11))
		return;
	unsigned cnt[17] = { 0 };
	uint32_t a = pc;
	for (int k = 0; k < SJ_MAX_INSNS && PAGE(a & SH34_AM) == pg; k++, a += 2) {
		const uint32_t op = sj_op(page, a);
		const int h = decode(op);
		if (sj_interp_only(h) || h == H_LDC_SR || h == H_LDCM_SR)
			break;
		const uint32_t u = sj_rc_uses(h, op);
		for (int g = 0; g < 17; g++)
			cnt[g] += (u >> g) & 1;
		if (sj_delayed(h) && h != H_BTS && h != H_BFS) {
			if (PAGE((a + 2) & SH34_AM) == pg) {
				const uint32_t so = sj_op(page, a + 2), us = sj_rc_uses(decode(so), so);
				for (int g = 0; g < 17; g++)
					cnt[g] += (us >> g) & 1;
			}
			break;
		}
	}
	static const int8_t host[3] = { 10, 14, 6 };
	if (s_sj_rcdbg & 8)
		cnt[RC_SR] = 0;
	if (s_sj_rcmax >= 0 && s_sj_nb >= (uint64_t)s_sj_rcmax)
		return;
	for (int g = 0; g < 17; g++)
		s_rch[g] = -1;
	for (int k = 0; k < 3; k++) {
		int best = -1;
		for (int g = 0; g < 17; g++)
			if (s_rch[g] < 0 && cnt[g] >= (unsigned)s_sj_rcmin && (best < 0 || cnt[g] > cnt[best]))
				best = g;
		if (best < 0)
			break;
		if (!((s_sj_rcdbg >> k) & 1)) {
			cnt[best] = 0;
			continue;
		}
		s_rch[best] = (s_sj_rcdbg & 16) ? host[k ^ 1 ^ (k >> 1)] : host[k];
		cnt[best] = 0;
		s_rc = 1;
	}
}

static sj_fn sj_install(void);
static int sj_hook(sh3_t *c, uint32_t pg);
static sj_fn sj_build(sh3_t *c, uint32_t pc)
{
	if (pc >= 0xe0000000u || (pc & 1))
		return SJ_NONE;
	if (c->idle.rec)   // (the write entries back before a page is looked at or hooked)
		idle_stop(c, 0);
	const uint32_t pg = PAGE(pc & SH34_AM);
	// the code's memory: the read entry, or the write entry's (fetch_wr); RAM (a write entry) must be the same memory
	uint8_t *const host = s_sj_pslot[pg] ? s_sj_host[s_sj_pslot[pg] - 1] : c->wr[pg];
	const uint8_t *page = c->rd[pg];
	if (!page) {
		if (!c->fetch_wr || !host)
			return SJ_NONE;
		page = host;
	} else if (host && host != page)
		return SJ_NONE;
	e_begin(c);
	s_nmark = 0;
	s_nlit = 0;
	s_bpg = pg;
	s_bpage = page;
	s_bram = host != NULL;
	sj_rc_alloc(page, pc, pg);
	uint32_t a = pc;
	int n = 0;
	for (;;) {
		if (PAGE(a & SH34_AM) != pg || n >= SJ_MAX_INSNS || s_n + s_nc > SJ_BUF_LIMIT || s_nx > SJ_MAXX - 24) {
			e_exit(a, 0, 1, 1);
			break;
		}
		const uint32_t op = sj_op(page, a);
		const int h = decode(op);
		if (sj_interp_only(h)) {
			if (!n)
				return SJ_NONE;
			e_exit(a, 0, 1, 1);
			break;
		}
		if (sj_delayed(h)) {
			const uint32_t sa = a + 2;
			const uint32_t sop = PAGE(sa & SH34_AM) == pg ? sj_op(page, sa) : 0;
			const int hs = decode(sop);
			if (PAGE(sa & SH34_AM) != pg || !sj_slot_ok(hs) || sj_delayed(hs)) {
				if (!n)
					return SJ_NONE;
				e_exit(a, 0, 1, 1);
				break;
			}
			e_delayed(c, a, op, h, sop, hs);
			s_mark[s_nmark++] = a;
			s_mark[s_nmark++] = sa;
			n++;
			if (h == H_BTS || h == H_BFS) {
				a += 2;
				continue;
			}
			n++;
			break;
		}
		if (h == H_BT || h == H_BF) {
			e_go(c, op, h, a + 2);
			const uint32_t t = a + 4 + SEXT8(op) * 2;
			const unsigned nt = e_tbranch(h == H_BT);   // (not taken)
			s_pend += (uint32_t)op_cost(h);
			e_update(t, 0);
			e_exit(t, 0, 1, 1);
			e_patch(nt);
			s_pend = 0;
			s_pdyn = 0;
			s_mark[s_nmark++] = a;
			n++;
			a += 2;
			continue;
		}
		if (h == H_LDC_SR || h == H_LDCM_SR) {   // (drc_pc = pc, the helper, the cycles dropped, the interrupt test)
			e_go(c, op, h, a + 2);
			e_imm(0, a + 2);
			e_strf(0, COFS(drc_pc));
			e_rc_wb(~0u);   // (OPT_RC: the bank may change)
			if (h == H_LDC_SR) {
				e_mov(0, 4);
				e_ldr(1, RO(EN));
				e_call((const void *)op_ldcsr);
			} else {
				e_str(5, COFS(icount));
				e_imm(0, a + 2);
				e_str(0, COFS(pc));
				e_mov(0, 4);
				e_imm(1, (op >> 8) & 15);
				e_call((const void *)op_ldcmsr);
				e_ldr(5, COFS(icount));
			}
			if (SJ_RCF) {
				e_ldr(0, COFS(test_irq));
				e_flag_set(0);
			} else
				e_ldr(10, COFS(test_irq));
			e_rc_kill(~0u);
			s_r10 = 1;
			s_pend = 0;
			s_pdyn = 0;
			s_kmask = 0;   // (the register bank may have changed)
			e_irqtest(a + 2, 0);
			s_mark[s_nmark++] = a;
			n++;
			a += 2;
			continue;
		}
		s_lit_done = 0;
		s_fuse_ok = op_mem(h); s_fcost = (uint32_t)op_cost(h); s_fnext = a + 2; s_fdyn = 0; s_fused = 0;
		e_insn(c, op, h, a + 2);
		s_fuse_ok = 0;
		sj_kafter(h, op, a + 2, s_lit_done);
		if (s_fused)
			s_fused = 0;   // (OPT_FUSE: counted off and tested in its access)
		else {
			s_pend += (uint32_t)op_cost(h);
			e_update_irq(a + 2, 0, op_mem(h));
		}
		s_mark[s_nmark++] = a;
		n++;
		a += 2;
	}
	if (s_full)
		return NULL;
	const int ram = host != NULL;   // (a ROM page: no write reaches its memory)
	if (ram && !sj_hook(c, pg))
		return NULL;
	const unsigned nhot = s_n;
	e_finish();
	if (s_full)
		return NULL;
	const sj_fn f = sj_install();
	if (!f)
		return NULL;
	if (s_sj_dump && pc == s_sj_dump) {   // (RBX_SH3_JITDUMP=pc: that block's code to /tmp/sj_dump.bin)
		FILE *df = fopen("/tmp/sj_dump.bin", "wb");
		if (df) {
			fwrite((const void *)f, 4, s_n, df);
			fclose(df);
		}
		rbx_log("sh3 jit: block %08x: %d instructions, %u words at %p", pc, n, s_n, (const void *)f);
	}
	if (s_sj_dump == 1) {   // (RBX_SH3_JITDUMP=1: every block to /tmp/sj_all.bin, a header of 8 words before each)
		FILE *df = fopen("/tmp/sj_all.bin", "ab");
		if (df) {
			const uint32_t hd[8] = { 0x5348334au, pc, (uint32_t)n, nhot, s_cold_base, s_n,
			                         (uint32_t)(uintptr_t)f, (uint32_t)(uintptr_t)s_sj_code };
			fwrite(hd, 4, 8, df);
			fwrite((const void *)f, 4, s_n, df);
			fclose(df);
		}
	}
	if (s_sj_prof && s_sj_nprof < SJ_PROF_N) {
		struct sj_profr *const pr = &s_sj_profr[s_sj_nprof++];
		pr->pc = pc;
		pr->ni = (uint32_t)n;
		pr->hot = s_cold_base;
		pr->words = s_n;
		for (unsigned k = 0; k < s_nmark && k < sizeof(pr->h); k++)
			pr->h[k] = (uint8_t)(decode(sj_op(page, s_mark[k])) + 1);
	}
	for (unsigned k = 0; ram && k < s_nmark + s_nlit; k++) {   // (the translated granules, the literals' too)
		const uint32_t g = ((k < s_nmark ? s_mark[k] : s_lit[k - s_nmark]) & 0xfff) >> 2;
		s_sj_bits[s_sj_pslot[pg] - 1][g >> 3] |= (uint8_t)(1u << (g & 7));
	}
	s_sj_nb++;
	s_sj_ni += (unsigned)n;
	s_sj_wh += nhot;
	s_sj_wx += s_cold_base - nhot;
	s_sj_wc += s_n - s_cold_base;
	return f;
}

// the page's writes to sj_write from now on (RAM: rd == wr)
static int sj_hook(sh3_t *c, uint32_t pg)
{
	if (s_sj_pslot[pg])
		return 1;
	if (s_sj_npg >= SJ_MAXP) {
		sj_flush(c);
		return 0;
	}
	s_sj_host[s_sj_npg] = c->wr[pg];
	s_sj_pg[s_sj_npg++] = pg;
	s_sj_pslot[pg] = (uint16_t)s_sj_npg;
	c->wr[pg] = NULL;
	return 1;
}

// (OPT_SHR, ADR-166) the shared cold paths, made once with the options (the CPU through r4 only: a CPU after this
// one uses them as well). tail: SR_EXIT's end (r1 = the code)
static void e_btail(unsigned tail) { E(0xea000000u | ((tail - s_n - 2) & 0x00ffffffu)); }
static void e_r4ofs(int rd, uint32_t off)   // rd = r4 + off (no r12)
{
	e_imm(rd, off);
	E(0xe0840000u | (uint32_t)rd << 12 | (uint32_t)rd);   // add rd, r4, rd
}
static void e_inc_r4(uint32_t off)   // c->(off)++ (r1, r2)
{
	e_imm(2, off);
	E(0xe7941002u);   // ldr r1, [r4, r2]
	E(0xe2811001u);   // add r1, r1, #1
	E(0xe7841002u);   // str r1, [r4, r2]
}
static void sj_gen_shared(unsigned tail)
{
	s_rc = 0;
	s_rcl = s_rcd = 0;
	// SR_RD*: as e_load's slow path
	for (int k = 0; k < 6; k++) {
		const int size = k >> 1 == 2 ? 4 : k >> 1 == 1 ? 2 : 1, sx = k & 1;
		s_sr_at[SR_RD0 + k] = s_n;
		E(0xe92d4008u);   // push {r3, lr}
		e_str(5, COFS(icount));
		e_mov(1, 11);
		e_mov(0, 4);
		e_call(size == 4 ? (const void *)sj_rd32 : size == 2 ? (const void *)sj_rd16 : (const void *)sj_rd8);
		e_ldr(5, COFS(icount));
		const int f = SJ_RCF ? 2 : 10;
		e_ldr(f, COFS(test_irq));
		e_imm(12, (uint32_t)(uintptr_t)&s_sj_flushed);
		E(0xe59cc000u);                    // ldr r12, [r12]
		E(0xe180008cu | (uint32_t)f << 16 | (uint32_t)f << 12);   // orr r10, r10, r12, lsl #1
		if (SJ_RCF)
			e_flag_set(f);
		e_sentinel(f);
		if (sx && size < 4)
			E(size == 2 ? 0xe6bf0070u : 0xe6af0070u);   // sxth / sxtb r0, r0
		E(0xe8bd8008u);   // pop {r3, pc}
	}
	// SR_WF*: e_store's write to a code page (r12 the return; r0, r2, r3 clobbered, r1 kept)
	for (int k = 0; k < 3; k++) {
		const int size = k == 2 ? 4 : k == 1 ? 2 : 1;
		unsigned toc[4];
		s_sr_at[SR_WF0 + k] = s_n;
		E(0xe35b020eu);                          // cmp r11, #0xe0000000
		toc[0] = e_bcond(0x2);                   // bhs -> no
		E(ubfx(2, 11, 12, 17));                  // r2 = the page
		e_imm(3, (uint32_t)(uintptr_t)s_sj_pslot);
		E(0xe0833082u);                          // add r3, r3, r2, lsl #1
		E(0xe1d330b0u);                          // ldrh r3, [r3] (its slot + 1)
		E(0xe3530000u);                          // cmp r3, #0
		toc[1] = e_bcond(0x0);                   // beq -> no
		e_r4ofs(0, COFS(idle.rec));
		E(0xe5900000u);                          // ldr r0, [r0]
		E(0xe3500000u);                          // cmp r0, #0
		toc[2] = e_bcond(0x1);                   // bne -> no
		E(0xe2433001u);                          // sub r3, r3, #1
		E(ubfx(2, 11, 2, 10));                   // r2 = the 4-byte granule
		e_imm(0, (uint32_t)(uintptr_t)s_sj_bits);
		E(0xe0800383u);                          // add r0, r0, r3, lsl #7
		E(0xe7d001a2u);                          // ldrb r0, [r0, r2, lsr #3]
		E(0xe2022007u);                          // and r2, r2, #7
		E(0xe1a00230u);                          // lsr r0, r0, r2
		E(0xe3100001u);                          // tst r0, #1
		toc[3] = e_bcond(0x1);                   // bne: translated -> no
		e_imm(0, (uint32_t)(uintptr_t)s_sj_host);
		E(0xe7900103u);                          // ldr r0, [r0, r3, lsl #2] (its memory)
		if (size == 4) {
			E(ubfx(2, 11, 2, 10));
			E(0xe7801102u);                      // str r1, [r0, r2, lsl #2]
		} else if (size == 2) {
			E(ubfx(2, 11, 1, 11));
			E(0xe2222001u);                      // eor r2, r2, #1
			E(0xe0800082u);                      // add r0, r0, r2, lsl #1
			E(0xe1c010b0u);                      // strh r1, [r0]
		} else {
			E(ubfx(2, 11, 0, 12));
			E(0xe2222003u);                      // eor r2, r2, #3
			E(0xe7c01002u);                      // strb r1, [r0, r2]
		}
		e_r4ofs(0, COFS(nslow));
		E(0xe5902000u);                          // ldr r2, [r0]
		E(0xe2822001u);                          // add r2, r2, #1
		E(0xe5802000u);                          // str r2, [r0]
		const int f = SJ_RCF ? 0 : 10;
		e_ldr(f, COFS(test_irq));
		if (SJ_RCF) {   // (e_flag_set, e_sentinel with r2 for r12)
			e_imm(2, (uint32_t)(uintptr_t)&s_sj_irqf);
			E(0xe5820000u | (uint32_t)f << 12);   // str f, [r2]
		}
		if (SJ_OPT(OPT_SENT)) {
			E(0xe3500000u | (uint32_t)f << 16);   // cmp f, #0
			e_imm(2, (uint32_t)(uintptr_t)&s_sj_stash);
			E(0x15825000u);                      // strne r5, [r2]
			E(0x13a05103u);                      // movne r5, #0xc0000000
		}
		E(0xe1520002u);                          // cmp r2, r2 (Z: done)
		E(0xe12fff1cu);                          // bx r12
		for (int j = 0; j < 4; j++)
			e_patch(toc[j]);
		E(0xe35c0000u);                          // cmp r12, #0 (Z clear: the C path)
		E(0xe12fff1cu);                          // bx r12
	}
	// SR_WC*: e_store's C path (bl; r1 the value, r3 the pc)
	for (int k = 0; k < 3; k++) {
		const int size = k == 2 ? 4 : k == 1 ? 2 : 1;
		s_sr_at[SR_WC0 + k] = s_n;
		E(0xe92d4008u);   // push {r3, lr}
		e_str(5, COFS(icount));
		e_mov(2, 1);
		e_mov(1, 11);
		e_mov(0, 4);
		e_call(size == 4 ? (const void *)sj_wr32 : size == 2 ? (const void *)sj_wr16 : (const void *)sj_wr8);
		const int f = SJ_RCF ? 0 : 10;
		if (f)
			e_mov(10, 0);
		e_ldr(5, COFS(icount));
		if (SJ_RCF)
			e_flag_set(f);
		e_sentinel(f);
		E(0xe8bd8008u);   // pop {r3, pc}
	}
	// SR_TP*: e_tpen's cold path (r12 the return; r0-r3 clobbered; a hit of the other way: cmp r5, #0)
	for (int w = 0; w < 2; w++) {
		uint32_t co = 0;
		s_sr_at[SR_TP0 + w] = s_n;
		E(0xe1a0122bu);                          // lsr r1, r11, #4
		E(ubfx(2, 11, 4, 9));
		if (SJ_R9)
			E(0xe0893202u);                      // add r3, r9, r2, lsl #4
		else {
			E(0xe0843202u);                      // add r3, r4, r2, lsl #4
			co = COFS(cache);
		}
		E(0xe5930000u | co);                     // ldr r0, [r3, #co]
		E(0xe5932000u | (co + 8));               // ldr r2, [r3, #co + 8] (the other way)
		E(0xe1520001u);                          // cmp r2, r1
		const unsigned miss = e_bcond(0x1);      // bne
		E(0xe5831000u | co);                     // str r1, [r3, #co]
		E(0xe5830000u | (co + 8));               // str r0, [r3, #co + 8]
		e_inc_r4(COFS(cache_gen));
		E(0xe3550000u);                          // cmp r5, #0
		E(0xe12fff1cu);                          // bx r12
		e_patch(miss);
		e_cacheable(11);
		const unsigned unc = e_bcond(0x0);       // beq: not cacheable
		E(0xe1a0122bu);                          // lsr r1, r11, #4
		E(0xe5831000u | co);                     // str r1, [r3, #co]
		E(0xe5830000u | (co + 8));               // str r0, [r3, #co + 8]
		e_inc_r4(COFS(cache_gen));
		e_patch(unc);
		e_r4ofs(3, COFS(tcost[w]));
		E(0xe1a00d2bu);                          // lsr r0, r11, #26
		E(0xe0833080u);                          // add r3, r3, r0, lsl #1
		E(0xe1d300b0u);                          // ldrh r0, [r3]
		E(0xe0555000u);                          // subs r5, r5, r0
		E(0xe12fff1cu);                          // bx r12
	}
	// the update points' exits (bl, [next]; the dirty registers written back): r0 = the flag; X: the slice's end (r6 as
	// it is, code 0) or the interrupt's exit (r6 = 0, code 2); Y: on (r6 = 0, code 1) or the interrupt's
	unsigned toX[4], toY[2], nX = 0, nY = 0;
	for (int g = 0; g < 4; g++) {
		s_sr_at[SR_G2 + 2 * g] = s_n;
		E(0xe59e0000u);                          // ldr r0, [lr] (next)
		e_str(0, COFS(pc));
		s_sr_at[SR_G2 + 2 * g + 1] = s_n;
		if (SJ_RCF) {
			e_imm(12, (uint32_t)(uintptr_t)&s_sj_irqf);
			E(0xe59c0000u);                      // ldr r0, [r12]
		} else
			e_mov(0, 10);
		if (g == 0) {   // (G2: XK_SPLIT2's stub)
			E(0xe3500000u);                      // cmp r0, #0
			toX[nX++] = e_bcond(0x0);            // beq X
			e_imm(12, (uint32_t)(uintptr_t)&s_sj_stash);
			E(0xe59cc000u);                      // ldr r12, [r12]
			E(0xe2455103u);                      // sub r5, r5, #0xc0000000
			E(0xe085500cu);                      // add r5, r5, r12
		}
		if (g <= 1) {   // (G1: XK_SPLIT's)
			E(0xe3550000u);                      // cmp r5, #0
			toX[nX++] = e_bcond(0xb);            // blt X
			toY[nY++] = e_bcond(0xe);            // b Y
		} else if (g == 2)   // (SQ)
			toX[nX++] = e_bcond(0xe);
		else {   // (EQ: r6 as the stub set it)
			E(0xe3100001u);                      // tst r0, #1
			E(0x13a01002u);                      // movne r1, #2
			E(0x03a01001u);                      // moveq r1, #1
			e_btail(tail);
		}
	}
	for (unsigned j = 0; j < nY; j++)
		e_patch(toY[j]);
	E(0xe3100001u);                              // tst r0, #1
	E(0xe3a06000u);                              // mov r6, #0
	E(0x13a01002u);                              // movne r1, #2
	E(0x03a01001u);                              // moveq r1, #1
	e_btail(tail);
	for (unsigned j = 0; j < nX; j++)
		e_patch(toX[j]);
	E(0xe3100001u);                              // tst r0, #1
	E(0x13a06000u);                              // movne r6, #0
	E(0x13a01002u);                              // movne r1, #2
	E(0x03a01000u);                              // moveq r1, #0
	e_btail(tail);
}

/* (ADR-185) the tables given back at the game's end (sh3_release): made at the first run, they sat above the board's
   buffers for the app's life, and the next game's 64 MB VRAM block did not fit in the hole those left (ddpdfk after
   espgal2: out of memory on the Vita). The code memory stays (the frontend's one block); its shared routines are made
   again at the next first run, as they are the first time */
static void sj_release(void)
{
	sj_forget();
	free(s_sj_tab); s_sj_tab = NULL;
	free(s_sj_next); s_sj_next = NULL;
	free(s_sj_buf); s_sj_buf = NULL;
	free(s_sj_bits); s_sj_bits = NULL;
	free(s_sj_dtab); s_sj_dtab = NULL;
	free(s_sj_ds); s_sj_ds = NULL;
	free(s_sj_pslot); s_sj_pslot = NULL;
}

static int sj_code_init(void)
{
	if (s_sj_code && s_sj_tab)
		return 1;
	if (s_sj_nocode)
		return 0;
	for (unsigned sz = 8u << 20; sz >= (2u << 20) && !s_sj_code; sz -= 2u << 20) {   // (8, 6, 4 or 2 MB)
		s_sj_code = (uint32_t *)rbx_jit_alloc(sz);
		s_sj_size = sz / 4;
	}
	s_sj_tab = calloc(1u << SJ_TAB_BITS, sizeof(*s_sj_tab));
	s_sj_next = calloc(1u << SJ_TAB_BITS, 1);
	s_sj_buf = calloc(SJ_BUF, 4);
	s_sj_bits = calloc(SJ_MAXP, sizeof(*s_sj_bits));
	s_sj_dtab = calloc(1u << SJ_DT_BITS, sizeof(*s_sj_dtab));
	s_sj_ds = calloc(SJ_DS_N, sizeof(*s_sj_ds));   // (none: OPT_DSITE off)
	if (!s_sj_pslot)
		s_sj_pslot = calloc(SH3_PAGES, sizeof(*s_sj_pslot));
	if (!s_sj_code || !s_sj_tab || !s_sj_next || !s_sj_buf || !s_sj_bits || !s_sj_pslot || !s_sj_dtab) {
		s_sj_nocode = 1;
		s_sj_code = NULL;
		return 0;
	}
	rbx_log("sh3 jit: %u KB of code memory at %p", s_sj_size / 256, (void *)s_sj_code);
	// (ADR-185: made again after a game, the routines start from the translator's state of a fresh start, not from
	// what the last block left: its register cache (s_rc, dirty registers) made SR_EXIT store them, and the next game
	// ran wrong)
	s_rc = 0;
	s_rcl = s_rcd = 0;
	s_kmask = 0;
	memset(s_kv, 0, sizeof(s_kv));
	memset(s_rch, 0, sizeof(s_rch));
	s_tpos = s_tcond = 0;
	s_fuse_ok = s_fused = s_fdyn = 0;
	s_fcost = s_fnext = 0;
	s_pend = 0;
	s_pdyn = s_r10 = s_dyn = 0;
	s_sj_inslot = 0;
	s_litv = s_bpg = 0;
	s_bram = s_lit_done = 0;
	s_nlit = s_nmark = 0;
	s_nx = s_nsite = s_nbl = 0;
	s_full = 0;
	s_cold_base = 0;
	s_sj_entry = 0;
	// SR_EXIT (bl, then the words next, code): c->pc = next; SR_EXITD (bl, then code): c->pc as it is. Both:
	// c->icount = r5, c->jpend = r6, s_sj_xlr = lr (the chain slot before the bl), return code
	e_bufs_reset();
	s_sr_at[SR_EXIT] = s_n;
	E(0xe89e0003u);   // ldm lr, {r0, r1}
	e_str(0, COFS(pc));
	const unsigned b = e_bcond(0xe);
	s_sr_at[SR_EXITD] = s_n;
	E(0xe59e1000u);   // ldr r1, [lr]
	e_patch(b);
	const unsigned tail = s_n;   // (OPT_SHR's routines end here, r1 = the code)
	e_str(5, COFS(icount));
	E(0xe284ca00u | ((COFS(jpend) >> 12) & 0xf));   // add r12, r4, #hi
	E(0xe58c6000u | (COFS(jpend) & 0xfff));         // str r6, [r12, #lo]
	e_imm(2, (uint32_t)(uintptr_t)&s_sj_xlr);
	E(0xe582e000u);   // str lr, [r2]
	E(0xe1a00001u);   // mov r0, r1
	E(SJ_R9 ? 0xe8bd8ff8u : 0xe8bd8df0u);   // pop {r3-r11, pc} / pop {r4-r8, r10, r11, pc}
	// SR_DLOOK (bl, then code; OPT_DYN): the block at c->pc from s_sj_dtab entered where a chained jump enters (what
	// sj_run would do next: no interrupt to test, no slot, the blocks not flushed), else SR_EXITD
	s_sr_at[SR_DLOOK] = s_n;
	e_ldr(0, COFS(pc));
	e_imm2(1, (uint32_t)(uintptr_t)s_sj_dtab);
	E(0xe02026a0u);           // eor r2, r0, r0, lsr #13
	E(ubfx(2, 2, 1, SJ_DT_BITS));
	E(0xe0811182u);           // add r1, r1, r2, lsl #3
	E(0xe891000cu);           // ldm r1, {r2, r3}
	E(0xe1520000u);           // cmp r2, r0
	E(0x012fff13u);           // bxeq r3
	E(0xea000000u | ((s_sr_at[SR_EXITD] - s_n - 2) & 0x00ffffffu));   // b SR_EXITD
	// SR_DLOOK2 (OPT_DSITE: r0 = c->pc, r12 = the exit's slot): as SR_DLOOK, the block found kept in the slot
	s_sr_at[SR_DLOOK2] = s_n;
	e_imm2(1, (uint32_t)(uintptr_t)s_sj_dtab);
	E(0xe02026a0u);           // eor r2, r0, r0, lsr #13
	E(ubfx(2, 2, 1, SJ_DT_BITS));
	E(0xe0811182u);           // add r1, r1, r2, lsl #3
	E(0xe891000cu);           // ldm r1, {r2, r3}
	E(0xe1520000u);           // cmp r2, r0
	E(0x088c000cu);           // stmeq r12, {r2, r3}
	E(0x012fff13u);           // bxeq r3
	E(0xea000000u | ((s_sr_at[SR_EXITD] - s_n - 2) & 0x00ffffffu));   // b SR_EXITD
	// SR_DPEN*, SR_UNC*, SR_APEN
	for (int w = 0; w < 2; w++) {
		s_sr_at[SR_DPEN0 + w] = s_n;
		E(0xe92d4008u);   // push {r3, lr}
		e_mov(0, 4);
		e_mov(1, 11);
		E(0xe3a02000u | (uint32_t)w);
		e_call((const void *)sj_dpen);
		E(0xe0455000u);   // sub r5, r5, r0
		E(0xe8bd8008u);   // pop {r3, pc}
		s_sr_at[SR_UNC0 + w] = s_n;
		if (sj_bus_fields_ok()) {
			unsigned toc[3];
			e_uncached(w, toc);
			E(0xe12fff1eu);   // bx lr
			for (int k = 0; k < 3; k++)   // (to SR_DPEN: a tail call)
				s_sj_buf[toc[k]] |= (s_sr_at[SR_DPEN0 + w] - toc[k] - 2) & 0x00ffffffu;
		} else
			E(0xea000000u | ((s_sr_at[SR_DPEN0 + w] - s_n - 2) & 0x00ffffffu));   // b SR_DPEN
	}
	s_sr_at[SR_APEN] = s_n;
	E(0xe92d4008u);
	e_mov(1, 0);
	e_mov(0, 4);
	E(0xe3a02000u);
	e_call((const void *)sj_apen);
	E(0xe0455000u);
	E(0xe8bd8008u);
	s_sr_gen = 1;   // (SR_DP* / SR_IC: the code the blocks have inline, made once to check it can be made)
	for (int w = 0; w < 2; w++) {
		s_sr_at[SR_DP0 + w] = s_n;
		e_dpen_sr(w);
	}
	s_sr_at[SR_IC] = s_n;
	s_sr_ic_ok = 0;
	e_ipen_sr();
	s_sr_gen = 0;
	s_shr = 0;
	if (SJ_OPT(OPT_SHR)) {
		sj_gen_shared(tail);
		s_shr = 1;
	}
	s_sj_srw = s_n;
	rbx_jit_write_begin();
	memcpy(s_sj_code, s_sj_buf, s_n * 4);
	rbx_jit_write_end(s_sj_code, s_n * 4);
	s_sj_used = s_sj_srw;
	s_sj_dt_empty = (uint32_t)(uintptr_t)(s_sj_code + s_sr_at[SR_EXITD]);
	sj_tables_clear();
	return 1;
}
static inline void sj_dt_set(uint32_t pc, sj_fn f)
{
	struct sj_dent *const d = &s_sj_dtab[sj_dt_idx(pc)];
	d->pc = pc;
	d->entry = (uint32_t)(uintptr_t)((const uint32_t *)(void *)f + s_sj_entry);
}

static sj_fn sj_install(void)
{
	if (s_sj_used + s_n > s_sj_size) {   // (full: everything again)
		rbx_log("sh3 jit: code memory full, cleared (%u blocks)", (unsigned)s_sj_nb);
		s_sj_nfull++;
		sj_flush(s_sj_cpu);
		return NULL;
	}
	uint32_t *dst = s_sj_code + s_sj_used;
	for (unsigned k = 0; k < s_nbl; k++)
		s_sj_buf[s_bl_at[k]] |= (s_sr_at[s_bl_to[k]] - (s_sj_used + s_bl_at[k]) - 2) & 0x00ffffffu;
	rbx_jit_write_begin();
	memcpy(dst, s_sj_buf, s_n * 4);
	rbx_jit_write_end(dst, s_n * 4);
	s_sj_used += s_n;
	return (sj_fn)(void *)dst;
}

static inline sj_fn sj_find(sh3_t *c, uint32_t pc)
{
	const unsigned i = ((pc >> 1) ^ (pc >> 15)) & ((1u << SJ_TAB_BITS) - 1);
	for (int w = 0; w < 4; w++)
		if (s_sj_tab[i][w].pc == pc && s_sj_tab[i][w].fn) {
			const sj_fn f = s_sj_tab[i][w].fn;
			if (f == SJ_NONE)
				return NULL;
			if (SJ_OPT(OPT_DYN) && s_sj_dtab[sj_dt_idx(pc)].pc != pc)
				sj_dt_set(pc, f);
			return f;
		}
	// (a block only where the flow came a few times: interrupts return to any pc, one block each would fill the
	// code memory)
	struct sj_hot *const hc = &s_sj_hot[((pc >> 1) * 2654435761u) >> (32 - SJ_HOT_BITS)];
	if (hc->pc != pc) {
		hc->pc = pc;
		hc->n = 0;
	}
	if (++hc->n < s_sj_hotn)
		return NULL;
	rbx_prof_set(SH3_PA_COMPILE);   // (sj_run's mark again after)
	const sj_fn f = sj_build(c, pc);
	rbx_prof_set(SH3_PA_RUN);
	if (!f)
		return NULL;
	const unsigned w = s_sj_next[i]++ & 3;
	s_sj_tab[i][w].pc = pc;
	s_sj_tab[i][w].fn = f;
	if (f == SJ_NONE)
		return NULL;
	if (SJ_OPT(OPT_DYN))
		sj_dt_set(pc, f);
	return f;
}

// the slot of the last exit made a jump into f (after its e_begin: the registers are the same in every block)
static void sj_chain(sj_fn f)
{
	uint32_t *slot = s_sj_slot;
	s_sj_slot = NULL;
	if (*slot != 0xe1a00000u)
		return;
	const uint32_t *t = (const uint32_t *)(void *)f + s_sj_entry;
	rbx_jit_write_begin();
	*slot = 0xea000000u | ((uint32_t)(t - slot - 2) & 0x00ffffffu);
	rbx_jit_write_end(slot, 4);
}

// (RBX_SH3_JITSTAT: where the interpreter runs from, and why)
static struct { uint32_t pc, n, why, op; uint64_t cyc; } s_sj_ipc[1024];
static unsigned sj_stat_pc(sh3_t *c, int from_block)   // (the entry's index, 1024 = none; cyc: the run's cycles)
{
	const uint32_t pc = c->pc, why = from_block ? 3 : c->delay ? 1 : c->test_irq ? 2 : 0;
	const unsigned i = ((pc >> 1) * 2654435761u) >> 22;
	if (s_sj_ipc[i].pc != pc || s_sj_ipc[i].why != why) {
		if (s_sj_ipc[i].n > 16) { s_sj_ipc[i].n--; return 1024; }
		s_sj_ipc[i].pc = pc; s_sj_ipc[i].why = why; s_sj_ipc[i].n = 0; s_sj_ipc[i].cyc = 0;
		const uint32_t pg = PAGE(pc & SH34_AM);
		const uint8_t *p = pc < 0xe0000000u ? c->rd[pg] : NULL;
		if (!p && pc < 0xe0000000u)   // (fetch_wr: the write entry's memory)
			p = s_sj_pslot && s_sj_pslot[pg] ? s_sj_host[s_sj_pslot[pg] - 1] : c->wr[pg];
		s_sj_ipc[i].op = p ? sj_op(p, pc) : 0;
	}
	s_sj_ipc[i].n++;
	return i;
}
static void sj_report(void)
{
	for (int k = 0; k < 16; k++) {
		int best = -1;
		for (int i = 0; i < 1024; i++)
			if (s_sj_ipc[i].n && (best < 0 || s_sj_ipc[i].n > s_sj_ipc[best].n))
				best = i;
		if (best < 0)
			break;
		const uint32_t pc = s_sj_ipc[best].pc;

		rbx_log("sh3 jit: interpreter from %08x (%s) %u times, op %04x, %llu cycles", pc, s_sj_ipc[best].why == 1 ? "slot" :
		        s_sj_ipc[best].why == 2 ? "irq" : s_sj_ipc[best].why == 3 ? "block exit" : "no block", s_sj_ipc[best].n,
		        s_sj_ipc[best].op, (unsigned long long)s_sj_ipc[best].cyc);
		s_sj_ipc[best].n = 0;
	}
	rbx_log("sh3 jit: %u ms in sh3_execute; %llu blocks, %llu instructions made; %llu flushes (%llu by writes); "
	        "%llu blocks run, %llu interpreter runs", (unsigned)(s_sj_us / 1000), (unsigned long long)s_sj_nb,
	        (unsigned long long)s_sj_ni, (unsigned long long)s_sj_nflush, (unsigned long long)s_sj_nwflush,
	        (unsigned long long)s_sj_runs, (unsigned long long)s_sj_interp);
	rbx_log("sh3 jit: words made: %llu hot, %llu exit stubs, %llu cold (per instruction %.1f, %.1f, %.1f); full %llu "
	        "times; %u KB used of %u", (unsigned long long)s_sj_wh, (unsigned long long)s_sj_wx,
	        (unsigned long long)s_sj_wc, s_sj_ni ? (double)s_sj_wh / (double)s_sj_ni : 0.0,
	        s_sj_ni ? (double)s_sj_wx / (double)s_sj_ni : 0.0, s_sj_ni ? (double)s_sj_wc / (double)s_sj_ni : 0.0,
	        (unsigned long long)s_sj_nfull, s_sj_used / 256, s_sj_size / 256);
	rbx_log("sh3 jit: calls: icache %llu, dpen %llu, apen %llu, rd %llu, wr %llu, div1 %llu, exec %llu",
	        (unsigned long long)s_sj_calls[0], (unsigned long long)s_sj_calls[1], (unsigned long long)s_sj_calls[2],
	        (unsigned long long)s_sj_calls[3], (unsigned long long)s_sj_calls[4], (unsigned long long)s_sj_calls[5],
	        (unsigned long long)s_sj_calls[6]);
	rbx_log("sh3 jit: C data penalties: on-chip %llu, not cacheable %llu, other way %llu, miss %llu, MRU %llu",
	        (unsigned long long)s_sj_dcls[0], (unsigned long long)s_sj_dcls[1], (unsigned long long)s_sj_dcls[2],
	        (unsigned long long)s_sj_dcls[3], (unsigned long long)s_sj_dcls[4]);
	for (int i = 0; i < 4096; i++) {
		if (s_sj_rdmb[i] > 100000)
			rbx_log("sh3 jit: slow reads at %03x00000: %u", i, s_sj_rdmb[i]);
		if (s_sj_wrmb[i] > 100000)
			rbx_log("sh3 jit: slow writes at %03x00000: %u", i, s_sj_wrmb[i]);
	}
	for (int i = 0; i < 4096; i++)
		if (s_sj_unc[i] > 100000)
			rbx_log("sh3 jit: not cacheable at %03x00000: %u", i, s_sj_unc[i]);
	rbx_log("sh3 jit: %u ms in the board's write handlers (%llu calls, timed)", (unsigned)(s_sj_cbus / 1000),
	        (unsigned long long)s_sj_cbn);
	for (int k = 0; k < 30; k++) {   // (the operations sj_exec did most)
		int best = -1;
		for (int i = 0; i < H_COUNT; i++)
			if (s_sj_cnt[i] && (best < 0 || s_sj_cnt[i] > s_sj_cnt[best]))
				best = i;
		if (best < 0)
			break;
		rbx_log("sh3 jit: sj_exec h %d: %u", best, s_sj_cnt[best]);
		s_sj_cnt[best] = 0;
	}
}

static int sj_prof_cmp(const void *x, const void *y)
{
	const struct sj_profr *a = x, *b = y;
	if (a->pc != b->pc)
		return a->pc < b->pc ? -1 : 1;
	return 0;
}
static void sj_prof_report(void)
{
	// (the same pc made again after a flush: one line)
	qsort(s_sj_profr, s_sj_nprof, sizeof(*s_sj_profr), sj_prof_cmp);
	unsigned m = 0;
	for (unsigned i = 0; i < s_sj_nprof; i++) {
		if (m && s_sj_profr[m - 1].pc == s_sj_profr[i].pc) {
			s_sj_profr[m - 1].n += s_sj_profr[i].n;
			continue;
		}
		s_sj_profr[m++] = s_sj_profr[i];
	}
	uint64_t tn = 0, ti = 0, th = 0;
	for (unsigned i = 0; i < m; i++) {
		tn += s_sj_profr[i].n;
		ti += (uint64_t)s_sj_profr[i].n * s_sj_profr[i].ni;
		th += (uint64_t)s_sj_profr[i].n * s_sj_profr[i].hot;
	}
	static uint64_t hn[H_COUNT];
	for (unsigned i = 0; i < m; i++)
		for (unsigned k = 0; k < sizeof(s_sj_profr[i].h) && s_sj_profr[i].h[k]; k++)
			hn[s_sj_profr[i].h[k] - 1] += s_sj_profr[i].n;
	for (int k = 0; k < 40; k++) {
		int best = -1;
		for (int i = 0; i < H_COUNT; i++)
			if (hn[i] && (best < 0 || hn[i] > hn[best]))
				best = i;
		if (best < 0)
			break;
		rbx_log("sh3 jitprof: h %d: %llu", best, (unsigned long long)hn[best]);
		hn[best] = 0;
	}
	rbx_log("sh3 jitprof: sj_run exits: slice %llu, chainable %llu, dynamic %llu (a block there %llu), other %llu, irq %llu",
	        (unsigned long long)s_sj_xc[0], (unsigned long long)s_sj_xc[1], (unsigned long long)s_sj_xc[2],
	        (unsigned long long)s_sj_xc[5], (unsigned long long)s_sj_xc[3], (unsigned long long)s_sj_xc[4]);
	rbx_log("sh3 jitprof: %u blocks, %llu entries, %llu guest instructions (full blocks), %llu hot host words", m,
	        (unsigned long long)tn, (unsigned long long)ti, (unsigned long long)th);
	for (int k = 0; k < 60; k++) {
		int best = -1;
		for (unsigned i = 0; i < m; i++)
			if (s_sj_profr[i].n && (best < 0 || (uint64_t)s_sj_profr[i].n * s_sj_profr[i].hot >
			                                    (uint64_t)s_sj_profr[best].n * s_sj_profr[best].hot))
				best = (int)i;
		if (best < 0)
			break;
		const struct sj_profr *const r = &s_sj_profr[best];
		rbx_log("sh3 jitprof: %08x entries %u insns %u hot %u words %u (%.2f%% of hot words)", r->pc, r->n, r->ni,
		        r->hot, r->words, 100.0 * (double)r->n * r->hot / (double)(th ? th : 1));
		s_sj_profr[best].n = 0;
	}
}

static int s_sj_noblk;   // (debugging, RBX_SH3_NOBLK=1: no blocks, the interpreter through sj_run's loop)
static long long s_sj_blog0 = 1, s_sj_blog1 = 0;
static void sj_env(void)
{
	if (s_sj_on >= 0)
		return;
	if (getenv("RBX_SH3_JITOPT"))
		s_sj_opt = (unsigned)strtoul(getenv("RBX_SH3_JITOPT"), NULL, 0);
	if (getenv("RBX_SH3_SHR") && !atoi(getenv("RBX_SH3_SHR")))   // (ADR-166 off: the Vita's "sh3coldold")
		s_sj_opt &= ~OPT_SHR;
	if (s_sj_opt != ~0u)
		rbx_log("sh3 jit: code shape options %#x", s_sj_opt);
	s_sj_prof = getenv("RBX_SH3_JITPROF") && atoi(getenv("RBX_SH3_JITPROF"));
	if (s_sj_prof) {
		s_sj_profr = calloc(SJ_PROF_N, sizeof(*s_sj_profr));
		if (s_sj_profr)
			atexit(sj_prof_report);
		else
			s_sj_prof = 0;
	}
	if (getenv("RBX_SH3_HOTN"))
		s_sj_hotn = (uint32_t)atoi(getenv("RBX_SH3_HOTN"));
	s_sj_on = !(getenv("RBX_SH3_NOJIT") && atoi(getenv("RBX_SH3_NOJIT")));
	s_sj_stat = getenv("RBX_SH3_JITSTAT") && atoi(getenv("RBX_SH3_JITSTAT"));
	if (getenv("RBX_SH3_JITDUMP"))
		s_sj_dump = (uint32_t)strtoul(getenv("RBX_SH3_JITDUMP"), NULL, 16);
	if (getenv("RBX_SH3_RCDBG"))
		s_sj_rcdbg = atoi(getenv("RBX_SH3_RCDBG"));
	if (getenv("RBX_SH3_RCMIN"))
		s_sj_rcmin = atoi(getenv("RBX_SH3_RCMIN"));
	if (getenv("RBX_SH3_RCMAX"))
		s_sj_rcmax = atoi(getenv("RBX_SH3_RCMAX"));
	if (getenv("RBX_SH3_JITDBG"))
		s_sj_dbg = atoi(getenv("RBX_SH3_JITDBG"));
	if (s_sj_stat)
		atexit(sj_report);
	s_sj_noblk = getenv("RBX_SH3_NOBLK") && atoi(getenv("RBX_SH3_NOBLK"));
	if (getenv("RBX_SH3_BLOG"))
		sscanf(getenv("RBX_SH3_BLOG"), "%lld,%lld", &s_sj_blog0, &s_sj_blog1);
}

// (debugging, RBX_SH3_BLOG=clock0,clock1: the guest state at every turn of sj_run's loop in that clock range, on stderr)
static void sj_blog(sh3_t *c)
{
	const long long t = (long long)now_cycles(c);
	if (t < s_sj_blog0 || t > s_sj_blog1)
		return;
	fprintf(stderr, "B %08x %lld %u d%u i%u sr%08x pr%08x", c->pc, t, c->jpend, c->delay, c->test_irq, c->sr, c->pr);
	for (int k = 0; k < 16; k++)
		fprintf(stderr, " %08x", c->r[k]);
	fprintf(stderr, "\n");
}

static int sj_enabled(sh3_t *c)
{
	sj_env();
	return s_sj_on && !c->trace && sj_code_init();
}

// the run loop with the blocks: a block where the pc has one, else the interpreter up to its next taken transfer
static void sj_run(sh3_t *c)
{
	if (s_sj_cpu != c) {
		if (s_sj_cpu)
			sj_forget();
		s_sj_cpu = c;
	}
	rbx_prof_set(SH3_PA_RUN);   // (sh3_execute's mark again after)
	if (c->flush_pc != s_sj_flushpc) {
		sj_flush(c);
		s_sj_flushpc = c->flush_pc;
	}
	// (the interpreter's entry: an interrupt raised between slices is taken before the first instruction)
	if (c->test_irq && !c->delay)
		check_pending_irq(c);
	c->jpend = 0;
	s_sj_slot = NULL;
	for (;;) {
		if (s_sj_blog1)
			sj_blog(c);
		if (!c->delay && !c->test_irq && !s_sj_noblk) {
			const sj_fn f = sj_find(c, c->pc);
			if (f) {
				if (s_sj_slot)
					sj_chain(f);
				s_sj_flushed = 0;
				s_sj_runs++;
				rbx_prof_set(SH3_PA_JIT);
				const uint32_t r = f();
				rbx_prof_set(SH3_PA_RUN);
				s_sj_irqf = 0;   // (OPT_RC: the flag a block left)
				if (s_sj_prof) {
					const unsigned k = (r & 0xff) == 0 ? 0 : (r & 0xff) == 2 ? 4 : (r & 0x100) ? 1 : (r & 0x200) ? 2 : 3;
					s_sj_xc[k]++;
					if (k == 2) {
						const unsigned i = ((c->pc >> 1) ^ (c->pc >> 15)) & ((1u << SJ_TAB_BITS) - 1);
						for (int w = 0; w < 4; w++)
							if (s_sj_tab[i][w].pc == c->pc && s_sj_tab[i][w].fn && s_sj_tab[i][w].fn != SJ_NONE)
								s_sj_xc[5]++;
					}
				}
				s_sj_slot = (r & 0x100) ? (uint32_t *)(uintptr_t)s_sj_xlr - 2 : NULL;
				if ((r & 0xff) == 0)
					return;
				if ((r & 0xff) == 2) {
					s_sj_slot = NULL;
					s_sj_interp++;
					const unsigned si = s_sj_stat ? sj_stat_pc(c, 1) : 1024;
					const int32_t ic0 = c->icount;
					rbx_prof_set(SH3_PA_INTERP);
					const int more = run(c, 2);
					rbx_prof_set(SH3_PA_RUN);
					if (si < 1024)
						s_sj_ipc[si].cyc += (uint32_t)(ic0 - c->icount);
					if (!more)
						return;
				}
				continue;
			}
		}
		s_sj_slot = NULL;
		s_sj_interp++;
		const unsigned si = s_sj_stat ? sj_stat_pc(c, 0) : 1024;
		const int32_t ic0 = c->icount;
		rbx_prof_set(SH3_PA_INTERP);
		const int more = run(c, 1);
		rbx_prof_set(SH3_PA_RUN);
		if (si < 1024)
			s_sj_ipc[si].cyc += (uint32_t)(ic0 - c->icount);
		if (!more)
			return;
	}
}

#endif
