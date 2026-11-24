/*  Hitachi SH-3 CPU, ported to C from MAME src/devices/cpu/sh: sh.cpp, sh.h (BSD-3-Clause, copyright-holders:
    David Haywood), sh4.cpp, sh4.h, sh4comn.cpp, sh4comn.h, sh3comn.cpp, sh3comn.h, sh4tmu.cpp (BSD-3-Clause,
    copyright-holders: R. Belmont). MAME's sh2 / sh4 cores are based on the C/C++ SH-2 core by
    <tiraniddo@hotmail.com>, with help from Chuck Mason and Olivier Galibert (SEMU).
    See sh3.h for the scope.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../cheat_cpu.h"
#include "sh3.h"
#include "../../rbx_prof.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#endif

// the sampled activity (rbx_prof.h, ADR-155): the CPU's ids, from RBX_PA_CPU
enum {
	SH3_PA_EXEC = RBX_PA_CPU,   // sh3_execute's slice loop
	SH3_PA_RUN,                 // sj_run: the block lookup, chaining, the dispatch between blocks
	SH3_PA_JIT,                 // inside the translated code
	SH3_PA_COMPILE,             // a block translated (sj_build)
	SH3_PA_INVAL,               // a write to a page with translated code (sj_write), the blocks thrown away (sj_flush)
	SH3_PA_INTERP,              // the interpreter (run)
	SH3_PA_SLOW,                // the C slow path of a memory access (slow_r / slow_w: on-chip registers, page lookups)
	SH3_PA_BOARD,               // the board's read / write handlers, where the board marks nothing
	SH3_PA_HELPER,              // an instruction the translated code leaves to C (sj_exec, sj_div1)
	SH3_PA_PENALTY,             // the timing model's C cases from the translated code (sj_dpen, sj_apen, sj_icache)
	SH3_PA_IRQ,                 // an interrupt or exception taken (check_pending_irq)
	SH3_PA_EVENTS,              // the timer and DMA end events (run_events)
	SH3_PA_DMA,                 // a DMA transfer
	SH3_PA_SPIN,                // the polling loop skips (sh3_spin_round, the idle rounds' reads)
	SH3_PA_END
};
static const char *const s_sh3_pa_names[SH3_PA_END - RBX_PA_CPU] = {
	"sh3 exec", "sh3 run", "sh3 jit", "sh3 compile", "sh3 inval", "sh3 interp", "sh3 slow", "sh3 board", "sh3 helper",
	"sh3 penalty", "sh3 irq", "sh3 events", "sh3 dma", "sh3 spin",
};

#define SH_T     0x00000001u
#define SH_S     0x00000002u
#define SH_I     0x000000f0u
#define SH_Q     0x00000100u
#define SH_M     0x00000200u
#define SH_FD    0x00008000u
#define SH_BL    0x10000000u
#define SH_RB    0x20000000u
#define SH_MD    0x40000000u
#define SH34_FLAGS (SH_MD | SH_RB | SH_BL | SH_FD | SH_M | SH_Q | SH_I | SH_S | SH_T)
#define SH34_AM  0x1fffffffu

#if defined(__GNUC__)
#define LIKELY(x)   __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)
#define SH3_INLINE  static inline __attribute__((always_inline))
#define SH3_NOINLINE static __attribute__((noinline))
#else
#define LIKELY(x)   (x)
#define UNLIKELY(x) (x)
#define SH3_INLINE  static inline
#define SH3_NOINLINE static
#endif

// the JIT (sh3_jit.inc): ARM hosts only
#if defined(__arm__)
#define SH3_JIT 1
static uint16_t *s_sj_pslot;   // (the pages holding translated code: their slot, 0 = none; the writes come to sj_write)
static void sj_write(sh3_t *c, uint32_t a, uint32_t v, int size);
static void sj_flush(sh3_t *c);
static int s_sj_stat;                // RBX_SH3_JITSTAT=1: counts and times (the harness), printed at exit
static uint64_t s_sj_cbus, s_sj_cbn; // (the board's write handlers: time, calls)
#if defined(__linux__)
#include <time.h>
static uint64_t sj_now_us(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return (uint64_t)t.tv_sec * 1000000u + (uint64_t)(t.tv_nsec / 1000);
}
#else
static uint64_t sj_now_us(void) { return 0; }
#endif
#endif

// MAME sh4.h: the exceptions
enum {
	INTC_TRAP = 21, INTC_NMI = 23, INTC_IRLn0, INTC_IRL0 = INTC_IRLn0 + 15, INTC_IRL1, INTC_IRL2, INTC_IRL3,
	INTC_HUDI, INTC_GPOI, INTC_DMTE0, INTC_DMTE1, INTC_DMTE2, INTC_DMTE3, INTC_DMTE4, INTC_DMTE5, INTC_DMTE6,
	INTC_DMTE7, INTC_DMAE, INTC_TUNI3, INTC_TUNI4, INTC_TUNI0, INTC_TUNI1, INTC_TUNI2, INTC_TICPI2, INTC_ATI,
	INTC_PRI, INTC_CUI, INTC_SCI1ERI, INTC_SCI1RXI, INTC_SCI1TXI, INTC_SCI1TEI, INTC_SCIFERI, INTC_SCIFRXI,
	INTC_SCIFBRI, INTC_SCIFTXI, INTC_ITI, INTC_RCMI, INTC_ROVI
};

// MAME sh4comn.h
#define EXPPRI(pl, po, p, n) ((uint32_t)(((4 - (pl)) << 24) | ((15 - (po)) << 16) | ((p) << 8) | (255 - (n))))
#define INTPRI(p, n)         EXPPRI(4, 2, p, n)

// MAME sh4comn.cpp
static const uint32_t exception_priority_default[] = {
	EXPPRI(1, 1, 0, 0), EXPPRI(1, 2, 0, 1), EXPPRI(1, 1, 0, 2), EXPPRI(1, 3, 0, 3), EXPPRI(1, 4, 0, 4),
	EXPPRI(2, 0, 0, 5), EXPPRI(2, 1, 0, 6), EXPPRI(2, 2, 0, 7), EXPPRI(2, 3, 0, 8), EXPPRI(2, 4, 0, 9),
	EXPPRI(2, 4, 0, 10), EXPPRI(2, 4, 0, 11), EXPPRI(2, 4, 0, 12), EXPPRI(2, 5, 0, 13), EXPPRI(2, 5, 0, 14),
	EXPPRI(2, 6, 0, 15), EXPPRI(2, 6, 0, 16), EXPPRI(2, 7, 0, 17), EXPPRI(2, 7, 0, 18), EXPPRI(2, 8, 0, 19),
	EXPPRI(2, 9, 0, 20), EXPPRI(2, 4, 0, 21), EXPPRI(2, 10, 0, 22), EXPPRI(3, 0, 16, INTC_NMI)
};

static const int exception_codes[SH3_EXC_COUNT] = {
	0x000, 0x020, 0x000, 0x140, 0x140, 0x1E0, 0x0E0, 0x040, 0x0A0, 0x180, 0x1A0, 0x800, 0x820, 0x0E0, 0x100,
	0x040, 0x060, 0x0A0, 0x0C0, 0x120, 0x080, 0x160, 0x1E0, 0x1C0,
	0x200, 0x220, 0x240, 0x260, 0x280, 0x2A0, 0x2C0, 0x2E0, 0x300, 0x320, 0x340, 0x360, 0x380, 0x3A0, 0x3C0,
	0x240, 0x2A0, 0x300, 0x360,
	0x600, 0x620, 0x640, 0x660, 0x680, 0x6A0, 0x780, 0x7A0, 0x7C0, 0x7E0, 0x6C0, 0xB00, 0xB80,
	0x400, 0x420, 0x440, 0x460, 0x480, 0x4A0, 0x4C0, 0x4E0, 0x500, 0x520, 0x540, 0x700, 0x720, 0x740, 0x760,
	0x560, 0x580, 0x5A0
};

static const int sh3_intevt2_exception_codes[SH3_EXC_COUNT] = {
	0x000, 0x020, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0x160, -1, 0x1C0,
	0x200, 0x220, 0x240, 0x260, 0x280, 0x2A0, 0x2C0, 0x2E0, 0x300, 0x320, 0x340, 0x360, 0x380, 0x3A0, 0x3C0,
	0x600, 0x620, 0x640, 0x660,
	-1, -1, 0x800, 0x820, 0x840, 0x860, -1, -1, -1, -1, -1, -1, -1,
	0x400, 0x420, 0x440, 0x460, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
};

static const int tcnt_div[8] = { 4, 16, 64, 256, 1024, 1, 1, 1 };

// on-chip register offsets (MAME sh3comn.h)
#define UP(a)  (((a) - 0xffffd000u) / 4)
#define LO(a)  (((a) - 0x04000000u) / 4)
#define INTEVT2      LO(0x4000000)
#define IRR0_IRR1    LO(0x4000004)
#define PINTER_IPRC  LO(0x4000014)
#define PADR_PBDR    LO(0x4000120)
#define PCDR_PDDR    LO(0x4000124)
#define PEDR_PFDR    LO(0x4000128)
#define PGDR_PHDR    LO(0x400012c)
#define PJDR_PKDR    LO(0x4000130)
#define PLDR_SCPDR   LO(0x4000134)
#define ICR0_IPRA    UP(0xfffffee0)
#define IPRB         UP(0xfffffee4)
#define SCSSR        UP(0xfffffe88)
#define TOCR_TSTR    UP(0xfffffe90)
#define TCOR0        UP(0xfffffe94)
#define TCNT0        UP(0xfffffe98)
#define TCR0         UP(0xfffffe9c)
#define TCOR1        UP(0xfffffea0)
#define TCNT1        UP(0xfffffea4)
#define TCR1         UP(0xfffffea8)
#define TCOR2        UP(0xfffffeac)
#define TCNT2        UP(0xfffffeb0)
#define TCR2         UP(0xfffffeb4)
#define TCPR2        UP(0xfffffeb8)
#define TRA          UP(0xffffffd0)
#define EXPEVT       UP(0xffffffd4)
#define INTEVT       UP(0xffffffd8)

static int s_log = -1;
static void logu(const char *what, uint32_t a, uint32_t d, uint32_t m)
{
	if (s_log < 0) s_log = getenv("RBX_SH3_LOG") != NULL;
	if (s_log) fprintf(stderr, "sh3: %s %08x = %08x & %08x\n", what, a, d, m);
}

/***************************************************************************
    clock
***************************************************************************/

SH3_INLINE int64_t now_cycles(const sh3_t *c)
{
	return c->running ? c->slice_end - c->icount : c->total_cycles;
}

int64_t sh3_total_cycles(const sh3_t *c) { return now_cycles(c); }

static int64_t next_event(const sh3_t *c)
{
	int64_t e = c->tmu_event[0];
	if (c->tmu_event[1] < e) e = c->tmu_event[1];
	if (c->tmu_event[2] < e) e = c->tmu_event[2];
	for (int i = 0; i < 4; i++)
		if (c->dma_event[i] < e) e = c->dma_event[i];
	return e;
}

static void resched(sh3_t *c)
{
	if (!c->running) return;
	int64_t e = next_event(c);
	if (e < c->slice_end) {
		c->icount -= (int32_t)(c->slice_end - e);
		c->slice_end = e;
	}
}

/***************************************************************************
    exceptions (sh4comn.cpp)
***************************************************************************/

static void change_register_bank(sh3_t *c, int to)
{
	if (to) {   // 0 -> 1
		for (int s = 0; s < 8; s++) {
			c->rbnk[0][s] = c->r[s];
			c->r[s] = c->rbnk[1][s];
		}
	} else {    // 1 -> 0
		for (int s = 0; s < 8; s++) {
			c->rbnk[1][s] = c->r[s];
			c->r[s] = c->rbnk[0][s];
		}
	}
}

static void default_exception_priorities(sh3_t *c)
{
	for (int a = 0; a <= INTC_NMI; a++)
		c->exc_pri[a] = exception_priority_default[a];
	for (int a = INTC_IRLn0; a <= INTC_IRLn0 + 14; a++)
		c->exc_pri[a] = INTPRI(15 - (a - INTC_IRLn0), a);
	c->exc_pri[INTC_IRL0] = INTPRI(13, INTC_IRL0);
	c->exc_pri[INTC_IRL1] = INTPRI(10, INTC_IRL1);
	c->exc_pri[INTC_IRL2] = INTPRI(7, INTC_IRL2);
	c->exc_pri[INTC_IRL3] = INTPRI(4, INTC_IRL3);
	for (int a = INTC_HUDI; a <= INTC_ROVI; a++)
		c->exc_pri[a] = INTPRI(0, a);
}

static void exception_recompute(sh3_t *c)
{
	c->test_irq = 0;
	if (!c->pending_irq || ((c->sr & SH_BL) && c->exc_req[INTC_NMI] == 0))
		return;
	int z = (c->sr >> 4) & 15;
	for (int a = 0; a <= INTC_ROVI; a++)
		if (c->exc_req[a]) {
			int pri = (int)(c->exc_pri[a] >> 8) & 255;
			if (pri > z) {
				c->test_irq = 1;
				break;
			}
		}
}

static void exception_request(sh3_t *c, int e)
{
	if (!c->exc_req[e]) {
		c->exc_req[e] = 1;
		c->pending_irq++;
		exception_recompute(c);
	}
}

static void exception_unrequest(sh3_t *c, int e)
{
	if (c->exc_req[e]) {
		c->exc_req[e] = 0;
		c->pending_irq--;
		exception_recompute(c);
	}
}

static void line_set(sh3_t *c, int line, int state)
{
	// MAME execute_set_input for the SH-3 (lines 0-3); a held line asserts
	int st = state == SH3_CLEAR ? SH3_CLEAR : SH3_ASSERT;
	int cur = c->line_state[line] == SH3_CLEAR ? SH3_CLEAR : SH3_ASSERT;
	if (st == SH3_CLEAR && cur == SH3_CLEAR) { c->line_state[line] = SH3_CLEAR; return; }
	if (st == cur) { c->line_state[line] = (uint8_t)state; return; }
	c->line_state[line] = (uint8_t)state;
	if (st == SH3_CLEAR)
		exception_unrequest(c, INTC_IRL0 + line);
	else
		exception_request(c, INTC_IRL0 + line);
}

void sh3_set_irq_line(sh3_t *c, int line, int state)
{
	if (line < 0 || line > 3) return;
	line_set(c, line, state);
}

static void exception_process(sh3_t *c, int e, uint32_t vector)
{
	// (sh4_exception_checkunrequest)
	if (e == INTC_NMI || e == INTC_DMTE0 || e == INTC_DMTE1 || e == INTC_DMTE2 || e == INTC_DMTE3)
		exception_unrequest(c, e);
	c->spc = c->pc;
	c->ssr = c->sr;
	c->sgr = c->r[15];
	c->sr |= SH_MD;
	if (!(c->sr & SH_RB))
		change_register_bank(c, 1);
	c->sr |= SH_RB;
	c->sr |= SH_BL;
	exception_recompute(c);
	c->pc = c->vbr + vector;
	c->drc_pc = c->pc;
	c->fetch_page = ~0u;
	if (c->sleep_mode == 1)
		c->sleep_mode = 2;
}

static void exception(sh3_t *c, int e)
{
	// SH-3 path of sh4_exception
	if (e <= INTC_NMI)
		return;   // (not supported by MAME for the SH-3)
	if (c->sr & SH_BL)
		return;
	if ((int)((c->exc_pri[e] >> 8) & 255) <= (int)((c->sr >> 4) & 15))
		return;
	// standard_irq_callback: a held line is dropped
	if (e >= INTC_IRL0 && e <= INTC_IRL3) {
		int line = e - INTC_IRL0;
		if (c->line_state[line] == SH3_HOLD)
			line_set(c, line, SH3_CLEAR);
	}
	int code = sh3_intevt2_exception_codes[e];
	c->lower[INTEVT2] = (uint32_t)code;
	c->upper[EXPEVT] = (uint32_t)exception_codes[e];
	if (code >= 0x600)
		c->upper[INTEVT] = 0x3e0 - ((c->exc_pri[e] >> 8) & 255) * 0x20;
	else
		c->upper[INTEVT] = (uint32_t)code;
	exception_process(c, e, 0x600);
}

static void check_pending_irq(sh3_t *c)
{
	int irq = 0;
	int z = -1;
	for (int a = 0; a <= INTC_ROVI; a++)
		if (c->exc_req[a] && (int)c->exc_pri[a] > z) {
			z = (int)c->exc_pri[a];
			irq = a;
		}
	if (z >= 0) {
		const unsigned pa = rbx_prof_enter(SH3_PA_IRQ);
		exception(c, irq);
		rbx_prof_leave(pa);
	}
}

/***************************************************************************
    TMU (sh4tmu.cpp)
***************************************************************************/

SH3_INLINE int64_t tick_cycles(const sh3_t *c, int which)
{
	return 4 * (int64_t)tcnt_div[c->tcr[which] & 7];   // (peripheral clock = CPU clock / 4)
}

static uint32_t compute_ticks(const sh3_t *c, int which, int div)
{
	// MAME compute_ticks_timer: remaining * hertz / divisor - 1, as a double
	int64_t rem = c->tmu_event[which] - now_cycles(c);
	double ret = ((double)rem / 4.0) / (double)div - 1;
	return (uint32_t)(int64_t)ret;
}

static void timer_recompute(sh3_t *c, int which)
{
	c->tmu_event[which] = now_cycles(c) + tick_cycles(c, which) * ((int64_t)c->tcnt[which] + 1);
	resched(c);
}

static void timer_callback(sh3_t *c, int which)
{
	// (the clock is the timer's expiry here)
	int64_t t = c->tmu_event[which];
	c->tcnt[which] = c->tcor[which];
	c->tmu_event[which] = t + tick_cycles(c, which) * ((int64_t)c->tcnt[which] + 1);
	c->tcr[which] |= 0x100;
	if (c->tcr[which] & 0x20)
		exception_request(c, INTC_TUNI0 + which);
}

static uint32_t tcnt_r(sh3_t *c, int which)
{
	if (c->tstr & (1u << which))
		return compute_ticks(c, which, tcnt_div[c->tcr[which] & 7]);
	return c->tcnt[which];
}

#define COMBINE(reg, d, m) ((reg) = ((reg) & ~(m)) | ((d) & (m)))

static void tstr_w(sh3_t *c, uint32_t data, uint32_t mask)
{
	uint32_t chg = c->tstr;
	COMBINE(c->tstr, data, mask);
	chg ^= c->tstr;
	for (int i = 0; i < 3; i++)
		if (chg & (1u << i)) {
			if (!(c->tstr & (1u << i))) {
				c->tcnt[i] = compute_ticks(c, i, tcnt_div[c->tcr[i] & 7]);
				c->tmu_event[i] = SH3_NEVER;
			} else
				timer_recompute(c, i);
		}
}

static void tcr_w(sh3_t *c, int i, uint32_t data, uint32_t mask)
{
	uint32_t old2 = c->tcr[i];
	COMBINE(c->tcr[i], data, mask);
	if (c->tstr & (1u << i)) {
		c->tcnt[i] = compute_ticks(c, i, tcnt_div[old2 & 7]);
		timer_recompute(c, i);
	}
	if (!(c->tcr[i] & 0x20) || !(c->tcr[i] & 0x100))
		exception_unrequest(c, INTC_TUNI0 + i);
}

static void tcnt_w(sh3_t *c, int i, uint32_t data, uint32_t mask)
{
	COMBINE(c->tcnt[i], data, mask);
	if (c->tstr & (1u << i))
		timer_recompute(c, i);
}

static void ipra_w(sh3_t *c, uint32_t data, uint32_t mask)
{
	COMBINE(c->ipra, data, mask);
	c->exc_pri[INTC_ATI] = INTPRI(c->ipra & 0x000f, INTC_ATI);
	c->exc_pri[INTC_PRI] = INTPRI(c->ipra & 0x000f, INTC_PRI);
	c->exc_pri[INTC_CUI] = INTPRI(c->ipra & 0x000f, INTC_CUI);
	c->exc_pri[INTC_TUNI2] = INTPRI((c->ipra & 0x00f0) >> 4, INTC_TUNI2);
	c->exc_pri[INTC_TICPI2] = INTPRI((c->ipra & 0x00f0) >> 4, INTC_TICPI2);
	c->exc_pri[INTC_TUNI1] = INTPRI((c->ipra & 0x0f00) >> 8, INTC_TUNI1);
	c->exc_pri[INTC_TUNI0] = INTPRI((c->ipra & 0xf000) >> 12, INTC_TUNI0);
	exception_recompute(c);
}

/***************************************************************************
    SH7709S cache and bus timing (sh7709s.cpp, "experimental cache/memory timing", hardcoded for cv1k)
***************************************************************************/

#define CACHE_BLOCKS 256
#define CACHE_WAYS 4
#define CACHE_LINE 16
#define SDRAM_PAGE_SIZE 1024
#define CACHE_MISS_STALL 1
#define BUS_ACCESS_PENALTY 3
#define BURST_READ_WORD_PENALTY 3
#define BURST_WRITE_WORD_PENALTY 4

static inline int is_cacheable(uint32_t a) { uint32_t r = a >> 29; return r != 5 && r != 7; }
static inline uint32_t get_area(uint32_t a) { return (a & SH34_AM) >> 26; }
static inline int is_sdram_region(uint32_t a) { return get_area(a) == 3; }

static int cache_access(sh3_t *c, uint32_t address, int write)
{
	if (!is_cacheable(address))
		return 0;
	address &= SH34_AM;
	const uint32_t cache_address = address / CACHE_LINE;
	const uint32_t blk = cache_address % CACHE_BLOCKS;
	// (the two lines hit last, looked at first: a hit on the most recent way changes no LRU, the same as the loop;
	// the entry itself is checked, so a hint never decides wrongly)
	static unsigned s_hint[2];
	for (int h = 0; h < 2; h++) {
		const unsigned bi = s_hint[h];
		if (bi >> 2 == blk && c->cache[blk][bi & 3].tag == cache_address && c->cache[blk][bi & 3].lru == CACHE_WAYS - 1) {
			if (write) {
				c->cache[blk][bi & 3].dirty = 1;
				c->cache_gen++;
			}
			return 1;
		}
	}
	for (int i = 0; i < CACHE_WAYS; i++) {
		if (c->cache[blk][i].tag == cache_address) {
			s_hint[1] = s_hint[0];
			s_hint[0] = blk << 2 | (unsigned)i;
			c->cache_gen += (uint32_t)write;
			if (write)
				c->cache[blk][i].dirty = 1;
			if (c->cache[blk][i].lru != CACHE_WAYS - 1) {
				c->cache_gen++;
				const uint8_t l = c->cache[blk][i].lru;
				for (int j = 0; j < CACHE_WAYS; j++)
					if (c->cache[blk][j].lru > l)
						c->cache[blk][j].lru--;
				c->cache[blk][i].lru = CACHE_WAYS - 1;
			}
			return 1;
		}
	}
	c->cache_gen++;
	for (int i = 0; i < CACHE_WAYS; i++) {
		if (c->cache[blk][i].lru == 0) {
			if (c->cache[blk][i].dirty)
				c->wb_address = c->cache[blk][i].tag * CACHE_LINE;
			c->cache[blk][i].tag = cache_address;
			c->cache[blk][i].lru = CACHE_WAYS - 1;
			c->cache[blk][i].dirty = (uint8_t)write;
			for (int j = 0; j < CACHE_WAYS; j++)
				if (i != j && c->cache[blk][j].lru != 0)
					c->cache[blk][j].lru--;
			break;
		}
	}
	return 0;
}

static uint32_t get_wcr1_timing(const sh3_t *c, uint32_t a)
{
	const uint32_t area = get_area(a);
	if (area > 6 || area == 1)
		return 0;
	const uint32_t v = (c->wcr1 >> (area * 2)) & 3;
	return v ? v : 1;
}

static uint32_t get_wcr2_timing(const sh3_t *c, uint32_t a)
{
	static const uint8_t burst[8] = { 2, 2, 3, 4, 4, 6, 8, 10 }, single[8] = { 0, 1, 2, 3, 4, 6, 8, 10 };
	const uint32_t area = get_area(a);
	uint32_t wcr2 = c->wcr2 & 0xffff;
	if (area > 6 || area == 1)
		return 0;
	uint32_t v;
	if (area == 0)
		v = wcr2 & 7;
	else if (area == 2 || area == 3) {
		wcr2 >>= 3;
		if (area == 3)
			wcr2 >>= 2;
		v = wcr2 & 3;
		return v ? v : 1;
	} else {
		wcr2 >>= 7 + ((area - 4) * 3);
		v = wcr2 & 7;
	}
	return is_sdram_region(a) ? burst[v] : single[v];
}

static inline uint32_t mcr_tpc(const sh3_t *c) { return ((c->mcr >> 14) & 3) + 1; }
static inline uint32_t mcr_rcd(const sh3_t *c) { return ((c->mcr >> 12) & 3) + 1; }
static inline uint32_t mcr_trwl(const sh3_t *c) { return ((c->mcr >> 10) & 3) + 1; }

static uint32_t cache_line_fetch_count(const sh3_t *c, uint32_t a)
{
	const uint32_t area = get_area(a);
	if (area == 0) return 8;
	if (area == 1) return 0;
	return CACHE_LINE >> ((c->bcr2 >> (area * 2)) & 3);
}

static uint32_t access_penalty(sh3_t *c, uint32_t address, int write)
{
	const uint32_t area = get_area(address);
	if (cache_access(c, address, write))
		return 0;
	const int sdram = is_sdram_region(address), cacheable = is_cacheable(address);
	uint32_t bus_penalty = BUS_ACCESS_PENALTY + BURST_READ_WORD_PENALTY + mcr_rcd(c) + get_wcr2_timing(c, address);
	uint32_t cpu_penalty = cacheable ? CACHE_MISS_STALL : 0;
	const uint32_t page_read = address / SDRAM_PAGE_SIZE;
	if (!sdram) {
		if (!cacheable)
			bus_penalty = BUS_ACCESS_PENALTY + get_wcr2_timing(c, address);
		else
			bus_penalty = (BUS_ACCESS_PENALTY + get_wcr2_timing(c, address)) * cache_line_fetch_count(c, address);
	} else if (!cacheable) {
		if (!write)
			bus_penalty = BUS_ACCESS_PENALTY + mcr_rcd(c) + get_wcr2_timing(c, address);
		else
			bus_penalty = BUS_ACCESS_PENALTY + mcr_rcd(c) + mcr_trwl(c);
	}
	if (area != c->last_area)
		bus_penalty += get_wcr1_timing(c, address);
	c->last_area = area;
	if (sdram && page_read == c->last_sdram_page && c->precharge_remaining > 0)
		cpu_penalty += c->precharge_remaining;
	c->precharge_remaining = 0;
	if (c->wb_active_cycles > 0) {
		cpu_penalty += c->wb_active_cycles;
		c->wb_active_cycles = 0;
	}
	if (sdram)
		c->last_sdram_page = page_read;
	if (c->wb_address != 0) {
		if (sdram) {
			const uint32_t page_write = c->wb_address / SDRAM_PAGE_SIZE;
			if (page_read == page_write)
				c->wb_active_cycles += mcr_tpc(c) * 2;
			c->wb_active_cycles += (BUS_ACCESS_PENALTY + get_wcr1_timing(c, address) + BURST_WRITE_WORD_PENALTY + mcr_rcd(c) +
			                        mcr_trwl(c)) * 2;
			c->last_sdram_page = page_write;
		} else {
			c->wb_active_cycles += get_wcr1_timing(c, address) * 2;
			c->wb_active_cycles += ((BUS_ACCESS_PENALTY + get_wcr2_timing(c, address)) * cache_line_fetch_count(c, address)) * 2;
		}
		c->wb_address = 0;
	} else if (sdram)
		c->precharge_remaining = mcr_tpc(c) * 2;
	return cpu_penalty + bus_penalty * 2;
}

// RBX_SH3_TSTAT=file (the interpreter, testing): the exact model's penalties by access class (write, address >> 26)
// and of the instruction fetch, one line per sh3_tstat_frame
static int s_tstat = -1;
static FILE *s_tstat_f;
static uint64_t s_ts_n[2][64], s_ts_sum[2][64], s_ts_m[2][64], s_ts_in, s_ts_isum;
static uint32_t s_ts_tag[512][2];   // (tpenalty's tag array alongside: its misses by class)
static void tstat_open(void)
{
	const char *p = getenv("RBX_SH3_TSTAT");
	s_tstat = 0;
	if (p && *p && (s_tstat_f = fopen(p, "w")) != NULL)
		s_tstat = 1;
}
void sh3_tstat_frame(sh3_t *c, int frame)
{
	(void)c;
	if (s_tstat < 0)
		tstat_open();
	if (!s_tstat)
		return;
	fprintf(s_tstat_f, "f %d i %llu %llu", frame, (unsigned long long)s_ts_in, (unsigned long long)s_ts_isum);
	for (int w = 0; w < 2; w++)
		for (int k = 0; k < 64; k++)
			if (s_ts_n[w][k]) {
				fprintf(s_tstat_f, " %c%02x %llu %llu", w ? 'w' : 'r', k, (unsigned long long)s_ts_n[w][k],
				        (unsigned long long)s_ts_sum[w][k]);
				fprintf(s_tstat_f, " %llu", (unsigned long long)s_ts_m[w][k]);
			}
	fputc('\n', s_tstat_f);
	memset(s_ts_n, 0, sizeof(s_ts_n));
	memset(s_ts_sum, 0, sizeof(s_ts_sum));
	memset(s_ts_m, 0, sizeof(s_ts_m));
	s_ts_in = s_ts_isum = 0;
}

/* The calibrated timing model (ADR-076, c->exact 0). MAME's model above (a 4-way LRU cache search on every data
   access and on the instruction fetch of groups 0, 4 and 15, the bus, precharge and write-back timing on a miss)
   was most of the SH-3's time in the JIT's code. Here a data access looks its line (address >> 4, the region kept)
   up in a 16 KB 2-way LRU tag array (512 sets in the 1024 tag fields of cache[][]; the way used last first, the only
   check the JIT inlines): a hit costs nothing, a miss tcost[write][address >> 26] clocks, the average MAME's model
   charges that class (read / write, the region and bus area) per miss of this array, measured over play (cv1k.c's
   tables). A not cacheable address (P2, the on-chip area) is never a tag: its class's average per access. No
   instruction fetch cost (under 0.7% of MAME's penalties). A frame's work keeps about MAME's clocks and follows its
   locality (the per-frame error of the penalty sum: sd 0.8-1.4% of a frame), so the game slows down where it does. */
SH3_INLINE uint32_t tpenalty(sh3_t *c, uint32_t a, int write)
{
	const uint32_t ln = a >> 4;
	sh3_ctag_t *const e = &(&c->cache[0][0])[(ln & 511) * 2];
	if (e[0].tag == ln)
		return 0;
	const uint32_t r = a >> 29, t0 = e[0].tag;
	if (e[1].tag == ln || (r != 5 && r != 7)) {   // (the other way: the two swap; a miss: the line in, the older out)
		const int hit = e[1].tag == ln;
		e[0].tag = ln;
		e[1].tag = t0;
		c->cache_gen++;
		if (hit)
			return 0;
	}
	return c->tcost[write][a >> 26];
}

void sh3_set_timing(sh3_t *c, const uint16_t *rd, const uint16_t *wr)
{
	c->exact = rd == NULL || wr == NULL;
	if (!c->exact) {
		memcpy(c->tcost[0], rd, sizeof(c->tcost[0]));
		memcpy(c->tcost[1], wr, sizeof(c->tcost[1]));
	}
}

// drc_update_icache (before the instructions of groups 0, 4 and 15)
static uint32_t icache_penalty_x(sh3_t *c);
static inline uint32_t icache_penalty(sh3_t *c)
{
	if (!c->exact)
		return 0;
	const uint32_t p = icache_penalty_x(c);
	if (UNLIKELY(s_tstat > 0)) {
		s_ts_in++;
		s_ts_isum += p;
	}
	return p;
}
static uint32_t icache_penalty_x(sh3_t *c)
{
	if (c->wb_active_cycles > 0) {
		c->wb_active_cycles--;
		if (c->wb_active_cycles == 0)
			c->precharge_remaining = mcr_tpc(c) * 2;
	} else if (c->precharge_remaining > 0)
		c->precharge_remaining--;
	return access_penalty(c, c->drc_pc, 0);
}

// a data access's penalty (the DRC's memory accessor: below 0xe0000000 the address is recorded in arg0, above it
// the last one is used)
static inline uint32_t data_penalty(sh3_t *c, uint32_t a, int write)
{
	if (!c->exact)
		return tpenalty(c, a, write);
	if (a < 0xe0000000u)
		c->arg0 = a;
	const uint32_t p = access_penalty(c, c->arg0, write);
	if (UNLIKELY(s_tstat > 0)) {
		s_ts_n[write][a >> 26]++;
		s_ts_sum[write][a >> 26] += p;
		if (is_cacheable(a)) {   // (as tpenalty)
			const uint32_t ln = a >> 4;
			uint32_t *const e = s_ts_tag[ln & 511];
			if (e[0] != ln + 1) {
				s_ts_m[write][a >> 26] += e[1] != ln + 1;
				e[1] = e[0];
				e[0] = ln + 1;
			}
		}
	}
	return p;
}

// the memory mapped cache address array (0xf0000000-0xf0ffffff, ahead of the board's map)
static uint32_t cache_array_r(sh3_t *c, uint32_t a)
{
	const uint32_t idx = (a - 0xf0000000u) >> 4;
	const uint32_t blk = idx % CACHE_BLOCKS, way = (idx / CACHE_BLOCKS) & 3;   // (MAME: no bound on the way)
	return (c->cache[blk][way].tag << 10) | ((uint32_t)c->cache[blk][way].dirty << 1) | (c->cache[blk][way].tag != 0);
}

static void cache_array_w(sh3_t *c, uint32_t a, uint32_t data)
{
	const uint32_t idx = (a - 0xf0000000u) >> 4;
	if (!(data & 2)) {
		const uint32_t blk = idx % CACHE_BLOCKS, way = (idx / CACHE_BLOCKS) & 3;
		if (c->cache[blk][way].dirty) {
			const uint32_t wb = c->cache[blk][way].tag * CACHE_LINE;
			c->cache[blk][way].dirty = 0;
			c->cache_gen++;
			c->icount -= (int32_t)((BUS_ACCESS_PENALTY + BURST_WRITE_WORD_PENALTY + get_wcr1_timing(c, wb) + mcr_rcd(c) +
			                        mcr_trwl(c)) * 2);
			c->last_area = get_area(wb);
		}
	}
}

/***************************************************************************
    DMAC (sh4dmac.cpp, the SH-3 paths)
***************************************************************************/

#define CHCR_DM 0x0000c000u
#define CHCR_SM 0x00003000u
#define CHCR_RS 0x00000f00u
#define CHCR_IE 0x00000004u
#define CHCR_TE 0x00000002u
#define DMAOR_DDT  0x8000u
#define DMAOR_AE   0x0004u
#define DMAOR_NMIF 0x0002u
#define DMAOR_DME  0x0001u

static uint32_t slow_r(sh3_t *c, uint32_t a, int size);
static void slow_w(sh3_t *c, uint32_t a, uint32_t v, int size);
static void idle_stop(sh3_t *c, int fail);

static void dmac_callback(sh3_t *c, int ch)
{
	c->dma_active[ch] = 0;
	c->dma_event[ch] = SH3_NEVER;
	c->dmatcr[ch] = 0;
	c->chcr[ch] |= CHCR_TE;
	if (c->chcr[ch] & CHCR_IE)
		exception_request(c, INTC_DMTE0 + ch);
}

// ADR-171: RBX_SH3_OLDDMA=1 = the byte port's copies a byte at a time in 256-byte reads, as before
static int s_dma_old;
#define DMA_BUF 2048

// RBX_SH3_DMASTAT=1: the transfers by kind (size, source / destination steps, their 64 MB areas), printed at exit
static int s_dma_stat, s_ds_at;
static uint64_t s_ds_n[256], s_ds_bytes[256];
static void dma_stat_report(void)
{
	for (int k = 0; k < 256; k++)
		if (s_ds_n[k])
			fprintf(stderr, "sh3 dma: size %d incs %d incd %d src area %d dst area %d: %llu transfers, %llu units\n",
			        1 << ((k >> 6) & 3), (k >> 4) & 3, (k >> 2) & 3, (k >> 1) & 1 ? 1 : 0, k & 1,
			        (unsigned long long)s_ds_n[k], (unsigned long long)s_ds_bytes[k]);
}
static void dma_stat(uint32_t chcr, uint32_t src, uint32_t dst, uint32_t count)
{
	// (areas: 0 = the bus areas 0..2 (ROM, NAND, blitter), 1 = 3 and up (RAM))
	const int k = (int)(((chcr >> 3) & 3) << 6 | ((chcr & CHCR_SM) >> 12) << 4 | ((chcr & CHCR_DM) >> 14) << 2 |
	                    (src >> 26 >= 3) << 1 | (dst >> 26 >= 3));
	if (!s_ds_at) {
		s_ds_at = 1;
		atexit(dma_stat_report);
	}
	s_ds_n[k]++;
	s_ds_bytes[k] += count;
}

// bytes a .. a + n - 1 of a plain page (host-order 32-bit words: byte a at (a & 0xfff) ^ 3) from src
static void dma_page_put(uint8_t *p, uint32_t off, const uint8_t *src, uint32_t n)
{
	while (n && (off & 3)) {
		p[off ^ 3] = *src++;
		off++, n--;
	}
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
	for (; n >= 16; n -= 16, off += 16, src += 16)
		vst1q_u8(p + off, vrev32q_u8(vld1q_u8(src)));
#endif
	for (; n >= 4; n -= 4, off += 4, src += 4) {
		uint32_t w;
		memcpy(&w, src, 4);
		w = (w >> 24) | ((w >> 8) & 0xff00u) | ((w << 8) & 0xff0000u) | (w << 24);   // (a byte swap: rev)
		memcpy(p + off, &w, 4);
	}
	for (; n; n--, off++)
		p[off ^ 3] = *src++;
}

// a byte port's n bytes to dst going up: each plain page's run copied at once, the rest each as slow_w does it (as
// the byte loop: a page's state looked at again after every slow write, the same nslow)
static void dma_put_up(sh3_t *c, uint32_t dst, const uint8_t *buf, uint32_t n)
{
	for (uint32_t i = 0; i < n;) {
		uint8_t *p = dst - 0x04000000u >= 0x04000000u ? c->wr[dst >> SH3_PAGE_SHIFT] : NULL;
#ifdef SH3_JIT
		if (p && s_sj_pslot && s_sj_pslot[dst >> SH3_PAGE_SHIFT])
			p = NULL;
#endif
		if (p) {
			uint32_t m = 0x1000u - (dst & 0xfffu);
			if (m > n - i)
				m = n - i;
			dma_page_put(p, dst & 0xfffu, buf + i, m);
			c->nslow += m;
			dst += m;
			i += m;
		} else {
			slow_w(c, dst, buf[i], 1);
			dst++;
			i++;
		}
	}
}

static void dma_transfer(sh3_t *c, int ch, uint32_t chcr)
{
	static const int sh3_dmasize[4] = { 1, 2, 4, 16 };
	int incd = (int)((chcr & CHCR_DM) >> 14);
	int incs = (int)((chcr & CHCR_SM) >> 12);
	int size = sh3_dmasize[(chcr >> 3) & 3];
	if (incd == 3 || incs == 3)
		return;
	uint32_t src = c->sar[ch], dst = c->dar[ch], count = c->dmatcr[ch];
	if (!count)
		count = 0x1000000;
	// timer mode 1: the end after 2 x count + 1 clocks
	c->dma_active[ch] = 1;
	c->dma_event[ch] = now_cycles(c) + 2 * (int64_t)count + 1;
	resched(c);
	src &= SH34_AM;
	dst &= SH34_AM;
	if (UNLIKELY(s_dma_stat))
		dma_stat(chcr, src, dst, count);
	// (MAME's program space accesses: physical addresses; the registers keep their values)
	if (size == 1 && incs == 0 && c->read_block8 && src - 0x04000000u >= 0x04000000u && !c->rd[src >> SH3_PAGE_SHIFT] &&
	    !(c->read_until && src - c->ru_lo <= c->ru_hi - c->ru_lo)) {
		// a byte port to memory (the CV1000's NAND): the bytes read in blocks, the same reads; each write as slow_w
		// does it, directly to a plain page
		uint8_t buf[DMA_BUF];
		if (c->idle.rec)
			idle_stop(c, 1);
		const uint32_t bn = s_dma_old ? 256u : (uint32_t)sizeof(buf);
		while (count > 0) {
			const uint32_t n = count < bn ? count : bn;
			if (c->read_block8(c->user, src, buf, n) != n)
				break;
			c->nslow += n;
			if (incd == 1 && !s_dma_old) {   // (ADR-171: the run on each plain page at once, the same bytes)
				dma_put_up(c, dst, buf, n);
				dst += n;
				count -= n;
				continue;
			}
			for (uint32_t i = 0; i < n; i++) {
				if (incd == 2) dst--;
				uint8_t *p = dst - 0x04000000u >= 0x04000000u ? c->wr[dst >> SH3_PAGE_SHIFT] : NULL;
#ifdef SH3_JIT
				if (p && s_sj_pslot && s_sj_pslot[dst >> SH3_PAGE_SHIFT])
					p = NULL;
#endif
				if (p) {
					c->nslow++;
					p[(dst & 0xfff) ^ 3] = buf[i];
				} else
					slow_w(c, dst, buf[i], 1);
				if (incd == 1) dst++;
			}
			count -= n;
		}
	}
	if (size == 1) {
		for (; count > 0; count--) {
			if (incs == 2) src--;
			if (incd == 2) dst--;
			slow_w(c, dst, slow_r(c, src, 1), 1);
			if (incs == 1) src++;
			if (incd == 1) dst++;
		}
	} else if (size == 2) {
		src &= ~1u; dst &= ~1u;
		for (; count > 0; count--) {
			if (incs == 2) src -= 2;
			if (incd == 2) dst -= 2;
			slow_w(c, dst, slow_r(c, src, 2), 2);
			if (incs == 1) src += 2;
			if (incd == 1) dst += 2;
		}
	} else if (size == 4) {
		src &= ~3u; dst &= ~3u;
		for (; count > 0; count--) {
			if (incs == 2) src -= 4;
			if (incd == 2) dst -= 4;
			slow_w(c, dst, slow_r(c, src, 4), 4);
			if (incs == 1) src += 4;
			if (incd == 1) dst += 4;
		}
	}
	// (16-byte units: MAME moves nothing)
}

static void dmac_check(sh3_t *c, int ch)
{
	const uint32_t chcr = c->chcr[ch];
	if (chcr & c->dmaor & DMAOR_DME) {
		if (((chcr & CHCR_RS) >> 8) < 2 || ((chcr & CHCR_RS) >> 8) > 6)
			return;
		if (!c->dma_active[ch] && !(chcr & CHCR_TE) && !(c->dmaor & (DMAOR_AE | DMAOR_NMIF))) {
			if (((chcr & CHCR_RS) >> 8) > 3) {
				const unsigned pa = rbx_prof_enter(SH3_PA_DMA);
				dma_transfer(c, ch, chcr);
				rbx_prof_leave(pa);
			} else if (!(c->dmaor & DMAOR_DDT))
				logu("dma to device", ch, chcr, 0);
		}
	} else if (c->dma_active[ch]) {
		c->dma_event[ch] = SH3_NEVER;
		c->dma_active[ch] = 0;
	}
}

static void dmaor_w(sh3_t *c, uint32_t data, uint32_t mask)
{
	uint32_t old = c->dmaor;
	COMBINE(c->dmaor, data, mask);
	if ((c->dmaor & DMAOR_AE) && (~old & DMAOR_AE)) c->dmaor &= ~DMAOR_AE;
	if ((c->dmaor & DMAOR_NMIF) && (~old & DMAOR_NMIF)) c->dmaor &= ~DMAOR_NMIF;
	for (int i = 0; i < 4; i++)
		dmac_check(c, i);
}

/***************************************************************************
    on-chip registers (sh3comn.cpp): a is 4-aligned, mask the big-endian lanes
***************************************************************************/

static uint32_t upper_r(sh3_t *c, uint32_t off, uint32_t mask)
{
	uint32_t ret = 0;
	switch (off) {
	case ICR0_IPRA: return (c->upper[off] & 0xffff0000) | (c->ipra & 0xffff);
	case TOCR_TSTR:
		if (mask & 0xff00000) ret |= (c->tocr & 0xff) << 24;   // (MAME tests 0x0ff00000)
		if (mask & 0x0000ff00) ret |= (c->tstr & 0xff) << 8;
		return ret;
	case TCOR0: return c->tcor[0];
	case TCOR1: return c->tcor[1];
	case TCOR2: return c->tcor[2];
	case TCNT0: return tcnt_r(c, 0);
	case TCNT1: return tcnt_r(c, 1);
	case TCNT2: return tcnt_r(c, 2);
	case TCR0: return c->tcr[0] << 16;
	case TCR1: return c->tcr[1] << 16;
	case TCR2: return c->tcr[2] << 16;
	case TCPR2: return c->tcpr2;
	case SCSSR: return 0x84u << 24;
	case UP(0xffffffec): return c->ccr;
	default: return c->upper[off];
	}
}

static void upper_w(sh3_t *c, uint32_t off, uint32_t data, uint32_t mask)
{
	COMBINE(c->upper[off], data, mask);
	switch (off) {
	case ICR0_IPRA:
		if (mask & 0x0000ffff) ipra_w(c, data & 0xffff, mask & 0xffff);
		break;
	case TOCR_TSTR:
		if (mask & 0xff000000) COMBINE(c->tocr, (data >> 24) & 0xffff, (mask >> 24) & 0xff);
		if (mask & 0x0000ff00) tstr_w(c, (data >> 8) & 0xff, (mask >> 8) & 0xff);
		break;
	case TCOR0: COMBINE(c->tcor[0], data, mask); break;
	case TCOR1: COMBINE(c->tcor[1], data, mask); break;
	case TCOR2: COMBINE(c->tcor[2], data, mask); break;
	case TCNT0: tcnt_w(c, 0, data, mask); break;
	case TCNT1: tcnt_w(c, 1, data, mask); break;
	case TCNT2: tcnt_w(c, 2, data, mask); break;
	case TCR0: tcr_w(c, 0, data >> 16, mask >> 16); break;
	case TCR1: tcr_w(c, 1, data >> 16, mask >> 16); break;
	case TCR2: tcr_w(c, 2, data >> 16, mask >> 16); break;
	case TCPR2: COMBINE(c->tcpr2, data, mask); break;
	case IPRB: break;
	case UP(0xffffff60):   // BCR1 / BCR2
		if (mask & 0xffff) COMBINE(c->bcr2, data & 0xffff, mask & 0xffff);
		break;
	case UP(0xffffff64):   // WCR1 / WCR2
		if (mask & 0xffff0000) COMBINE(c->wcr1, data >> 16, mask >> 16);
		if (mask & 0xffff) COMBINE(c->wcr2, data & 0xffff, mask & 0xffff);
		break;
	case UP(0xffffff68):   // MCR
		if (mask & 0xffff0000) COMBINE(c->mcr, data >> 16, mask >> 16);
		break;
	case UP(0xffffffec):   // CCR (sh7709s ccr_w: CF invalidates the cache and is not kept)
		if (data & 8) {
			memset(c->cache, 0, sizeof(c->cache));
			c->cache_gen++;
		}
		COMBINE(c->ccr, data, mask & ~8u);
		break;
	default:
		logu("upper w", 0xffffd000u + off * 4, data, mask);
		break;
	}
}

static uint8_t port_rd(sh3_t *c, int p) { return c->port_r ? c->port_r(c->user, p) : 0; }
static void port_wr(sh3_t *c, int p, uint8_t d) { if (c->port_w) c->port_w(c->user, p, d); }

static uint32_t lower_r(sh3_t *c, uint32_t off, uint32_t mask)
{
	if (off >= 0x1000) {
		logu("lower r", 0x04000000u + off * 4, 0, mask);
		return 0;
	}
	if (off >= LO(0x4000020) && off < LO(0x4000060)) {
		const int ch = (int)(off - LO(0x4000020)) / 4;
		switch ((off - LO(0x4000020)) & 3) {
		case 0: return c->sar[ch];
		case 1: return c->dar[ch];
		case 2: return c->dmatcr[ch];
		default: return c->chcr[ch];
		}
	}
	switch (off) {
	case LO(0x4000060): return c->dmaor << 16;
	case INTEVT2: case IRR0_IRR1:
		return c->lower[off];
	case PADR_PBDR:
		if (mask & 0xffff0000) return (uint32_t)port_rd(c, SH3_PORT_A) << 24;
		if (mask & 0x0000ffff) return (uint32_t)port_rd(c, SH3_PORT_B) << 8;
		break;
	case PCDR_PDDR:
		if (mask & 0xffff0000) return (uint32_t)port_rd(c, SH3_PORT_C) << 24;
		if (mask & 0x0000ffff) return (uint32_t)port_rd(c, SH3_PORT_D) << 8;
		break;
	case PEDR_PFDR:
		if (mask & 0xffff0000) return (uint32_t)port_rd(c, SH3_PORT_E) << 24;
		if (mask & 0x0000ffff) return (uint32_t)port_rd(c, SH3_PORT_F) << 8;
		break;
	case PGDR_PHDR:
		if (mask & 0xffff0000) return (uint32_t)port_rd(c, SH3_PORT_G) << 24;
		if (mask & 0x0000ffff) return (uint32_t)port_rd(c, SH3_PORT_H) << 8;
		break;
	case PJDR_PKDR:
		if (mask & 0xffff0000) return (uint32_t)port_rd(c, SH3_PORT_J) << 24;
		if (mask & 0x0000ffff) return (uint32_t)port_rd(c, SH3_PORT_K) << 8;
		break;
	case PLDR_SCPDR:
		if (mask & 0xffff0000) return (uint32_t)port_rd(c, SH3_PORT_L) << 24;
		break;
	case LO(0x4000150): case LO(0x4000154): case LO(0x4000158): case LO(0x400015c):   // SCIF: latched
		if (mask & 0xffffff00) return c->lower[off];
		break;
	default:
		logu("lower r", 0x04000000u + off * 4, 0, mask);
		break;
	}
	return 0;
}

static void lower_w(sh3_t *c, uint32_t off, uint32_t data, uint32_t mask)
{
	if (off >= 0x1000) {
		logu("lower w", 0x04000000u + off * 4, data, mask);
		return;
	}
	COMBINE(c->lower[off], data, mask);
	if (off >= LO(0x4000020) && off < LO(0x4000060)) {
		const int ch = (int)(off - LO(0x4000020)) / 4;
		switch ((off - LO(0x4000020)) & 3) {
		case 0: COMBINE(c->sar[ch], data, mask); break;
		case 1: COMBINE(c->dar[ch], data, mask); break;
		case 2: COMBINE(c->dmatcr[ch], data, mask); break;
		default: COMBINE(c->chcr[ch], data, mask); dmac_check(c, ch); break;
		}
		return;
	}
	switch (off) {
	case LO(0x4000060): dmaor_w(c, data >> 16, mask >> 16); break;
	case IRR0_IRR1:
		if (mask & 0xff000000)
			for (int i = 0; i < 4; i++)
				if (!(data & (0x01000000u << i))) line_set(c, i, SH3_CLEAR);
		break;
	case PINTER_IPRC:
		if (mask & 0x0000ffff) {
			COMBINE(c->iprc, data & 0xffff, mask & 0xffff);
			c->exc_pri[INTC_IRL0] = INTPRI((c->iprc & 0x000f) >> 0, INTC_IRL0);
			c->exc_pri[INTC_IRL1] = INTPRI((c->iprc & 0x00f0) >> 4, INTC_IRL1);
			c->exc_pri[INTC_IRL2] = INTPRI((c->iprc & 0x0f00) >> 8, INTC_IRL2);
			c->exc_pri[INTC_IRL3] = INTPRI((c->iprc & 0xf000) >> 12, INTC_IRL3);
			exception_recompute(c);
		}
		break;
	case PADR_PBDR:
		if (mask & 0xffff0000) port_wr(c, SH3_PORT_A, (uint8_t)(data >> 24));
		if (mask & 0x0000ffff) port_wr(c, SH3_PORT_B, (uint8_t)(data >> 8));
		break;
	case PCDR_PDDR:
		if (mask & 0xffff0000) port_wr(c, SH3_PORT_C, (uint8_t)(data >> 24));
		if (mask & 0x0000ffff) port_wr(c, SH3_PORT_D, (uint8_t)(data >> 8));
		break;
	case PEDR_PFDR:
		if (mask & 0xffff0000) port_wr(c, SH3_PORT_E, (uint8_t)(data >> 24));
		if (mask & 0x0000ffff) port_wr(c, SH3_PORT_F, (uint8_t)(data >> 8));
		break;
	case PGDR_PHDR:
		if (mask & 0xffff0000) port_wr(c, SH3_PORT_G, (uint8_t)(data >> 24));
		if (mask & 0x0000ffff) port_wr(c, SH3_PORT_H, (uint8_t)(data >> 8));
		break;
	case PJDR_PKDR:
		if (mask & 0xffff0000) port_wr(c, SH3_PORT_J, (uint8_t)(data >> 24));
		if (mask & 0x0000ffff) port_wr(c, SH3_PORT_K, (uint8_t)(data >> 8));
		break;
	case LO(0x4000104): case LO(0x4000108): case LO(0x400010c): case LO(0x4000110): case LO(0x4000114):
	case LO(0x4000150): case LO(0x4000154): case LO(0x4000158): case LO(0x400015c):
		break;   // (port control, SCIF: latched)
	default:
		logu("lower w", 0x04000000u + off * 4, data, mask);
		break;
	}
}

/***************************************************************************
    bus
***************************************************************************/

#define PAGE(a) ((a) >> SH3_PAGE_SHIFT)

/* ---- idle rounds (a speed-up with exactly the results of running them) ----
   A polling loop: the round from a read of a board address the driver declares a function of the clock (read_until)
   back to the same read. Three rounds in a row from the same CPU state (the registers, the delay slot, the interrupt
   state, the timing model: cache tags / LRU / dirty, write-back, precharge, last area, arg0, drc_pc ...), the same
   value read, nothing else through slow_r / slow_w and, in the last two, the write table emptied so that any write
   would come to slow_w (none came), make the round a fixed point: it changes nothing but icount, by the same D cycles
   each time. The rounds up to the slice's end (icount stays >= 0 at every update point) or up to the clock where the
   read's value changes are then left out by icount alone; the read that follows them is done as it comes.
   Why three: the cycles gathered and not yet counted off (pend) are not in sh3_t. Every round has a taken transfer
   (it comes back to the same pc), which counts them off or drops them, so two equal rounds (the same path: the same
   state, value and memory) leave the same pend at their ends, and the second one's D is every later round's.
   Nothing else can change within a slice: the timer and DMA ends are its end, the interrupt lines change between
   slices, memory only by the CPU's writes (and the DMA they start). The on-chip register files (upper / lower)
   change only through slow_w. */
static struct { uint32_t pg; uint8_t *p; } *s_idle_wr;   // (the write table entries taken away while recording)
static uint32_t s_idle_wrn, s_idle_wrcap;

/* (ADR-165) The cache array (8 KB of the 8.8 KB) is compared by cache_gen instead of its bytes: every change of it
   raises cache_gen (the C timing code, the JIT's inline paths of the calibrated model, CCR, reset / load) except the
   dirty bit of a write (the exact model's inline write hit, cache_array_w). The recorded rounds R1 -> R2 -> R3 write
   nothing (a write ends the recording), so an unchanged cache_gen from R1 on means the same cache array in R1, R2 and
   R3: the two rounds that set D start from the same state, as with the bytes compared. exc_pri (296 bytes) is left out
   too: it is a function of IPRA / IPRC (compared) set by their writes (upper_w / lower_w: through slow_w, counted in
   nslow, which a round must not have but for its read) and by reset / load (they end the recording).
   RBX_SH3_SPINOLD=1: the bytes as before (also for sh3_spin_round's). */
typedef struct { uint16_t lo, hi; } sh3_part_t;   // (a part of sh3_t: offsets, both multiples of 4)
static const sh3_part_t s_idle_parts[2][4] = {
	{ { offsetof(sh3_t, r), offsetof(sh3_t, icount) }, { offsetof(sh3_t, sleep_mode), offsetof(sh3_t, exc_pri) },
	  { offsetof(sh3_t, total_cycles), offsetof(sh3_t, cache) }, { offsetof(sh3_t, wb_address), offsetof(sh3_t, upper) } },
	{ { offsetof(sh3_t, r), offsetof(sh3_t, icount) }, { offsetof(sh3_t, sleep_mode), offsetof(sh3_t, upper) },
	  { 0, 0 }, { 0, 0 } },   // (RBX_SH3_SPINOLD)
};
typedef char sh3_idle_snap_fits[(offsetof(sh3_t, icount) - offsetof(sh3_t, r)) +
                                (offsetof(sh3_t, upper) - offsetof(sh3_t, sleep_mode)) <= sizeof(((sh3_t *)0)->idle.snap) ? 1 : -1];
typedef char sh3_part_align[(offsetof(sh3_t, sleep_mode) | offsetof(sh3_t, exc_pri) | offsetof(sh3_t, total_cycles) |
                             offsetof(sh3_t, cache) | offsetof(sh3_t, wb_address) | offsetof(sh3_t, upper) |
                             offsetof(sh3_t, tstr) | offsetof(sh3_t, icount) | offsetof(sh3_t, idle.snap) |
                             offsetof(sh3_t, spin.snap)) % 4 == 0 ? 1 : -1];
static int s_idle_bytes;   // (RBX_SH3_SPINOLD: the cache array's and exc_pri's bytes compared, as before ADR-165)

static void parts_put(const sh3_t *c, const sh3_part_t *pt, int n, uint8_t *o)
{
	for (int i = 0; i < n; i++) {
		memcpy(o, (const char *)c + pt[i].lo, pt[i].hi - pt[i].lo);
		o += pt[i].hi - pt[i].lo;
	}
}
// (the same as memcmp == 0; words, 4 at a time: no early exit inside a group of 16 bytes)
static int parts_eq(const sh3_t *c, const sh3_part_t *pt, int n, const uint8_t *o)
{
	for (int i = 0; i < n; i++) {
		const uint32_t *a = (const uint32_t *)(const void *)((const char *)c + pt[i].lo);
		const uint32_t *s = (const uint32_t *)(const void *)o;
		uint32_t k = (uint32_t)(pt[i].hi - pt[i].lo) >> 2;
		o += pt[i].hi - pt[i].lo;
		for (; k >= 4; k -= 4, a += 4, s += 4)
			if ((a[0] ^ s[0]) | (a[1] ^ s[1]) | (a[2] ^ s[2]) | (a[3] ^ s[3]))
				return 0;
		for (; k; k--)
			if (*a++ != *s++)
				return 0;
	}
	return 1;
}

static void idle_snap(sh3_t *c)
{
	c->idle.nsnap++;
	c->idle.cg = c->cache_gen;
	parts_put(c, s_idle_parts[s_idle_bytes], s_idle_bytes ? 2 : 4, c->idle.snap);
}

static int idle_same(const sh3_t *c)
{
	return (s_idle_bytes || c->cache_gen == c->idle.cg) &&
	       parts_eq(c, s_idle_parts[s_idle_bytes], s_idle_bytes ? 2 : 4, c->idle.snap);
}

// the writable pages' write entries away (every write comes to slow_w); 0 = no memory for the list
static int idle_unmap(sh3_t *c)
{
	uint32_t n = 0;
	if (c->idle.nwr > 8)   // (more writable ranges than kept)
		return 0;
	for (uint32_t i = 0; i < c->idle.nwr; i++)
		n +=c->idle.wrp[i][1] - c->idle.wrp[i][0] + 1;
	if (n > s_idle_wrcap) {
		void *p = realloc(s_idle_wr, n * sizeof(*s_idle_wr));
		if (!p)
			return 0;
		s_idle_wr = p;
		s_idle_wrcap = n;
	}
	s_idle_wrn = 0;
	for (uint32_t i = 0; i < c->idle.nwr; i++)
		for (uint32_t pg = c->idle.wrp[i][0]; pg <= c->idle.wrp[i][1]; pg++)
			if (c->wr[pg]) {
				s_idle_wr[s_idle_wrn].pg = pg;
				s_idle_wr[s_idle_wrn++].p = c->wr[pg];
				c->wr[pg] = NULL;
			}
	c->idle.rec = 1;
	return 1;
}

/* (ADR-175) A tight polling loop proven to write nothing, so its rounds are recorded without taking the write table
   away (idle_unmap walks every writable page twice: about 12 thousand host instructions a recording, 15 a frame in a
   busy scene). The read is made by a load outside a delay slot (c->rnorm: the interpreter's and the JIT's loads say
   so; then it is at c->pc - 2) and the instruction before it is not a delayed branch, no
   interrupt can be taken (test_irq 0: in the same state it stays 0, the instructions below do not change SR's mask
   or the INTC), and from it the code goes through register instructions only (no memory access, no system register)
   to a branch back to pc that this state takes (the instructions evaluated with the value read). Every round from
   this state is then this path: no write, so the memory does not change and the next round (the state compared) is
   the same path again. The code is read from the page table (a page without one: not proven). Anything else is
   recorded as before (RBX_SH3_IDLEPURE=0: always). */
static int s_idle_pure = 1;

static int idle_op(const sh3_t *c, uint32_t pc, uint32_t *op)
{
	if (pc >= 0xe0000000u)
		return 0;
	const uint32_t a = pc & SH34_AM;
	const uint8_t *p = c->rd[PAGE(a)];
	if (!p)
		return 0;
	*op = *(const uint16_t *)(p + ((a & 0xffe) ^ 2));
	return 1;
}

static int op_delayed(uint32_t op)   // (BRA BSR BT/S BF/S BRAF BSRF JMP JSR RTS RTE)
{
	return (op & 0xe000) == 0xa000 || (op & 0xfd00) == 0x8d00 || (op & 0xf0df) == 0x0003 || (op & 0xf0df) == 0x400b ||
	       (op & 0xffdf) == 0x000b;
}

// a register instruction of the list: done on r / t (1), else 0
static int idle_alu(uint32_t op, uint32_t *r, uint32_t *t)
{
	const uint32_t n = (op >> 8) & 15, m = (op >> 4) & 15, i = op & 0xff;
	switch (op >> 12) {
	case 0x0:
		return op == 0x0009;   // NOP
	case 0x2:
		switch (op & 15) {
		case 8: *t = (r[n] & r[m]) == 0; return 1;   // TST Rm,Rn
		case 9: r[n] &= r[m]; return 1;              // AND
		case 10: r[n] ^= r[m]; return 1;             // XOR
		case 11: r[n] |= r[m]; return 1;             // OR
		default: return 0;
		}
	case 0x3:
		if ((op & 15) == 0) { *t = r[n] == r[m]; return 1; }   // CMP/EQ Rm,Rn
		return 0;
	case 0x6:
		switch (op & 15) {
		case 3: r[n] = r[m]; return 1;                           // MOV Rm,Rn
		case 12: r[n] = r[m] & 0xff; return 1;                   // EXTU.B
		case 13: r[n] = r[m] & 0xffff; return 1;                 // EXTU.W
		case 14: r[n] = (uint32_t)(int32_t)(int8_t)r[m]; return 1;    // EXTS.B
		case 15: r[n] = (uint32_t)(int32_t)(int16_t)r[m]; return 1;   // EXTS.W
		default: return 0;
		}
	case 0x8:
		if (n == 8) { *t = r[0] == (uint32_t)(int32_t)(int8_t)i; return 1; }   // CMP/EQ #imm,R0
		return 0;
	case 0xc:
		switch (n) {
		case 8: *t = (r[0] & i) == 0; return 1;   // TST #imm,R0
		case 9: r[0] &= i; return 1;              // AND #imm,R0
		case 10: r[0] ^= i; return 1;             // XOR #imm,R0
		case 11: r[0] |= i; return 1;             // OR #imm,R0
		default: return 0;
		}
	case 0xe:
		r[n] = (uint32_t)(int32_t)(int8_t)i;   // MOV #imm,Rn
		return 1;
	default:
		return 0;
	}
}

static int idle_pure(const sh3_t *c, int size, uint32_t v)
{
	const uint32_t pc = c->pc - 2;   // (c->pc: the pc after the load, a slot's the target; rnorm: not a slot)
	uint32_t op, prev;
	if (!s_idle_pure || !c->rnorm || c->test_irq || c->delay || !idle_op(c, pc, &op) || !idle_op(c, pc - 2, &prev) || op_delayed(prev))
		return 0;
	uint32_t r[16], t = c->sr & 1;
	memcpy(r, c->r, sizeof(r));
	// the read: its register gets the value
	const uint32_t n = (op >> 8) & 15;
	const uint32_t sx = size == 1 ? (uint32_t)(int32_t)(int8_t)v : size == 2 ? (uint32_t)(int32_t)(int16_t)v : v;
	if ((op & 0xf00c) == 0x6000 && (op & 3) != 3 && (int)(1u << (op & 3)) == size)          // MOV.x @Rm,Rn
		r[n] = sx;
	else if ((op & 0xf000) == 0x5000 && size == 4)                                         // MOV.L @(disp,Rm),Rn
		r[n] = sx;
	else if ((op & 0xfe00) == 0x8400 && (int)(1u << ((op >> 8) & 1)) == size)             // MOV.B / W @(disp,Rm),R0
		r[0] = sx;
	else
		return 0;
	uint32_t at = pc + 2;
	for (int k = 0; k < 8; k++, at += 2) {
		if (!idle_op(c, at, &op))
			return 0;
		if ((op & 0xf900) == 0x8900) {   // BT BF BT/S BF/S
			const int taken = ((op >> 9) & 1) ? !t : t;
			if (!taken)
				continue;
			if (at + 4 + (uint32_t)((int32_t)(int8_t)(op & 0xff) * 2) != pc)
				return 0;
			uint32_t slot;
			return !((op >> 10) & 1) || (idle_op(c, at + 2, &slot) && idle_alu(slot, r, &t));
		}
		if ((op & 0xf000) == 0xa000) {   // BRA
			uint32_t slot;
			const int32_t d = (int32_t)((op & 0xfff) ^ 0x800) - 0x800;
			return at + 4 + (uint32_t)(d * 2) == pc && idle_op(c, at + 2, &slot) && idle_alu(slot, r, &t);
		}
		if (!idle_alu(op, r, &t))
			return 0;
	}
	return 0;
}

// the recording (if any) ends, the write table back; fail: a loop that is not one, tried again later and later
static void idle_stop(sh3_t *c, int fail)
{
	struct sh3_idle *s = &c->idle;
	if (s->rec) {
		for (uint32_t i = 0; i < s_idle_wrn; i++)
			c->wr[s_idle_wr[i].pg] = s_idle_wr[i].p;
		s_idle_wrn = 0;
		s->rec = 0;
	}
	if (fail) {
		s->nfail++;
		if (s->fails < 10)
			s->fails++;
		s->wait = 1u << s->fails;
		s->miss = 0;
	}
	s->n = 0;
}

static uint32_t idle_board_r(sh3_t *c, uint32_t a, int size)
{
	return size == 4 ? c->read32(c->user, a) : size == 2 ? c->read16(c->user, a) : c->read8(c->user, a);
}

// a board read in [ru_lo, ru_hi] (a masked, aligned)
SH3_NOINLINE uint32_t idle_read(sh3_t *c, uint32_t a, int size)
{
	struct sh3_idle *s = &c->idle;
	s->ncall++;
	const int64_t now = c->slice_end - c->icount;
	const int64_t until = c->running && !c->trace ? c->read_until(c->user, a, size) : 0;
	uint32_t v = idle_board_r(c, a, size);
	if (until <= now) {   // (not a value that stays: the loop ends)
		if (s->n)
			idle_stop(c, 0);
		return v;
	}
	if (s->wait) {
		s->wait--;
		return v;
	}
	const int chain = s->n && c->pc == s->pc && a == s->a && size == s->size && v == s->v && c->slice_end == s->se &&
	                  c->nslow == s->ns + 1 && c->icount < s->ic;
	s->nsame += (uint32_t)chain;
	if (!chain || !idle_same(c)) {
		if (s->n >= 2 || (s->n && ++s->miss > 16))   // (a recorded round that did not come back; a loop that does not settle)
			idle_stop(c, 1);
		else if (s->rec)
			idle_stop(c, 0);
		// round 0 from here
		s->pc = c->pc;
		s->a = a;
		s->size = size;
		s->v = v;
		s->se = c->slice_end;
		s->ns = c->nslow;
		s->ic = c->icount;
		s->n = 1;
		idle_snap(c);
		return v;
	}
	if (s->n < 3) {   // (round 1: record the next two)
		if (s->n == 1 && idle_pure(c, size, v)) {   // (ADR-175: no write possible, the write table stays)
			s->npure++;
			s_idle_wrn = 0;
			s->rec = 1;
		} else if (s->n == 1 && !idle_unmap(c)) {
			s->n = 0;
			return v;
		}
		if (s->n == 1)
			s->nrec++;
		s->n++;
		s->ns = c->nslow;
		s->ic = c->icount;
		return v;
	}
	// two rounds came back unchanged: the rest of them left out
	const int32_t d = s->ic - c->icount;
	idle_stop(c, 0);
	s->fails = 0;
	s->miss = 0;
	int64_t k = c->icount >= 0 ? c->icount / d : 0;
	const int64_t kt = (until - 1 - now) / d + 1;
	if (kt < k)
		k = kt;
	if (k > 0) {
		c->icount -= (int32_t)(k * d);
		s->skipped += k * d;
		s->nskip++;
		v = idle_board_r(c, a, size);
	}
	return v;
}

// accesses that are not in the page table. a: the CPU's address
SH3_INLINE uint32_t slow_r_body(sh3_t *c, uint32_t a, int size)
{
	c->nslow++;
	if (a >= 0xe0000000u) {
		if (a >= 0xffffd000u) {
			uint32_t mask = size == 4 ? 0xffffffffu : size == 2 ? ((a & 2) ? 0xffffu : 0xffff0000u)
			                                                   : 0xffu << ((3 - (a & 3)) * 8);
			uint32_t v = upper_r(c, (a - 0xffffd000u) >> 2, mask) & mask;
			return size == 4 ? v : size == 2 ? ((a & 2) ? v : v >> 16) : v >> ((3 - (a & 3)) * 8);
		}
		if ((a >> 24) == 0xf0) {
			uint32_t v = cache_array_r(c, a);
			return size == 4 ? v : size == 2 ? ((a & 2) ? v & 0xffff : v >> 16) : (v >> ((3 - (a & 3)) * 8)) & 0xff;
		}
	} else {
		a &= SH34_AM;
		if (a >= 0x04000000u && a < 0x08000000u) {
			uint32_t mask = size == 4 ? 0xffffffffu : size == 2 ? ((a & 2) ? 0xffffu : 0xffff0000u)
			                                                   : 0xffu << ((3 - (a & 3)) * 8);
			uint32_t v = lower_r(c, (a - 0x04000000u) >> 2, mask) & mask;
			return size == 4 ? v : size == 2 ? ((a & 2) ? v : v >> 16) : v >> ((3 - (a & 3)) * 8);
		}
		const uint8_t *p = c->rd[PAGE(a)];
		if (p)
			return size == 4 ? *(const uint32_t *)(p + (a & 0xffc)) : size == 2 ? *(const uint16_t *)(p + ((a & 0xffe) ^ 2))
			                                                                    : p[(a & 0xfff) ^ 3];
		if (UNLIKELY(c->read_until != NULL) && a - c->ru_lo <= c->ru_hi - c->ru_lo) {
			rbx_prof_set(SH3_PA_SPIN);
			return idle_read(c, a, size);
		}
	}
	rbx_prof_set(SH3_PA_BOARD);
	return size == 4 ? c->read32(c->user, a) : size == 2 ? c->read16(c->user, a) : c->read8(c->user, a);
}

// (the marks: slow, then what it went to; the caller's put back after)
SH3_NOINLINE uint32_t slow_r(sh3_t *c, uint32_t a, int size)
{
	const unsigned pa = rbx_prof_enter(SH3_PA_SLOW);
	const uint32_t v = slow_r_body(c, a, size);
	rbx_prof_leave(pa);
	return v;
}

SH3_INLINE void slow_w_body(sh3_t *c, uint32_t a, uint32_t v, int size)
{
	c->nslow++;
	if (UNLIKELY(c->idle.rec))   // (a write in a recorded round: not a polling loop; the write table back first)
		idle_stop(c, 1);
	if (a >= 0xe0000000u) {
		if (a >= 0xffffd000u) {
			int sh = size == 4 ? 0 : size == 2 ? ((a & 2) ? 0 : 16) : (3 - (a & 3)) * 8;
			uint32_t mask = (size == 4 ? 0xffffffffu : size == 2 ? 0xffffu : 0xffu) << sh;
			upper_w(c, (a - 0xffffd000u) >> 2, v << sh, mask);
			return;
		}
		if ((a >> 24) == 0xf0) {
			int sh = size == 4 ? 0 : size == 2 ? ((a & 2) ? 0 : 16) : (3 - (a & 3)) * 8;
			cache_array_w(c, a, v << sh);
			return;
		}
	} else {
		a &= SH34_AM;
		if (a >= 0x04000000u && a < 0x08000000u) {
			int sh = size == 4 ? 0 : size == 2 ? ((a & 2) ? 0 : 16) : (3 - (a & 3)) * 8;
			uint32_t mask = (size == 4 ? 0xffffffffu : size == 2 ? 0xffffu : 0xffu) << sh;
			lower_w(c, (a - 0x04000000u) >> 2, v << sh, mask);
			return;
		}
#ifdef SH3_JIT
		if (s_sj_pslot && s_sj_pslot[PAGE(a)]) {   // (a page with translated code: its write table entry is taken away)
			sj_write(c, a, v, size);
			return;
		}
#endif
		uint8_t *p = c->wr[PAGE(a)];
		if (p) {
			if (size == 4) *(uint32_t *)(p + (a & 0xffc)) = v;
			else if (size == 2) *(uint16_t *)(p + ((a & 0xffe) ^ 2)) = (uint16_t)v;
			else p[(a & 0xfff) ^ 3] = (uint8_t)v;
			return;
		}
	}
	rbx_prof_set(SH3_PA_BOARD);
#ifdef SH3_JIT
	if (s_sj_stat) {   // (RBX_SH3_JITSTAT: the time in the board's write handlers)
		const uint64_t t0 = sj_now_us();
		if (size == 4) c->write32(c->user, a, v);
		else if (size == 2) c->write16(c->user, a, (uint16_t)v);
		else c->write8(c->user, a, (uint8_t)v);
		s_sj_cbus += sj_now_us() - t0;
		s_sj_cbn++;
		return;
	}
#endif
	if (size == 4) c->write32(c->user, a, v);
	else if (size == 2) c->write16(c->user, a, (uint16_t)v);
	else c->write8(c->user, a, (uint8_t)v);
}

SH3_NOINLINE void slow_w(sh3_t *c, uint32_t a, uint32_t v, int size)
{
	const unsigned pa = rbx_prof_enter(SH3_PA_SLOW);
	slow_w_body(c, a, v, size);
	rbx_prof_leave(pa);
}

// the fast path: below 0xe0000000, a page in the table (the on-chip area is never mapped)
#define FASTP(tab, a) ((a) < 0xe0000000u ? (tab)[PAGE((a) & SH34_AM)] : NULL)

SH3_INLINE uint32_t RL(sh3_t *c, uint32_t a)
{
	a &= ~3u;
	const uint8_t *p = FASTP(c->rd, a);
	if (LIKELY(p != NULL)) return *(const uint32_t *)(p + (a & 0xffc));
	return slow_r(c, a, 4);
}
SH3_INLINE void WL(sh3_t *c, uint32_t a, uint32_t v)
{
	a &= ~3u;
	uint8_t *p = FASTP(c->wr, a);
	if (LIKELY(p != NULL)) { *(uint32_t *)(p + (a & 0xffc)) = v; return; }
	slow_w(c, a, v, 4);
}

uint32_t sh3_read32(sh3_t *c, uint32_t a) { return RL(c, a); }
uint16_t sh3_read16(sh3_t *c, uint32_t a)
{
	a &= ~1u;
	const uint8_t *p = FASTP(c->rd, a);
	if (p) return *(const uint16_t *)(p + ((a & 0xffe) ^ 2));
	return (uint16_t)slow_r(c, a, 2);
}

// instruction fetch (MAME m_pr16(pc & AM)): the page becomes the fetch page
SH3_NOINLINE uint16_t fetch_slow(sh3_t *c, uint32_t pc)
{
	uint32_t a = pc & SH34_AM;
	const uint8_t *p = c->rd[PAGE(a)];
	if (p) {
		c->fetch_page = pc >> SH3_PAGE_SHIFT;
		c->fetch_base = p;
		return *(const uint16_t *)(p + ((a & 0xffe) ^ 2));
	}
	return (uint16_t)slow_r(c, a & ~1u, 2);
}

/***************************************************************************
    map
***************************************************************************/

int sh3_map(sh3_t *c, uint32_t start, uint32_t end, void *mem, uint32_t size, int readable, int writable)
{
	if ((start & 0xfff) || (end & 0xfff) != 0xfff || end < start || end > 0x1fffffff) return -1;
	idle_stop(c, 0);
	c->spin.n = 0;
	if (mem && writable) {
		if (c->idle.nwr < 8) {
			c->idle.wrp[c->idle.nwr][0] = PAGE(start);
			c->idle.wrp[c->idle.nwr++][1] = PAGE(end);
		} else
			c->idle.nwr = 9;
	}
#ifdef SH3_JIT
	sj_flush(c);   // (the write table as the map made it, the blocks thrown away)
#endif
	if (size == 0) size = end - start + 1;
	if (size & 0xfff) return -1;
	for (uint32_t a = start, off = 0; ; a += 0x1000) {
		uint8_t *p = mem ? (uint8_t *)mem + off : NULL;
		if (a >= 0x04000000u && a < 0x08000000u) p = NULL;   // (the on-chip registers)
		if (!mem || readable) c->rd[PAGE(a)] = p;
		if (!mem || writable) c->wr[PAGE(a)] = p;
		off += 0x1000;
		if (off >= size) off = 0;
		if (a >= end - 0xfff) break;
	}
	c->fetch_page = ~0u;
	c->fetch_base = NULL;
	return 0;
}

void sh3_swap32(void *p, uint32_t size)
{
	uint8_t *b = (uint8_t *)p;
	for (uint32_t i = 0; i + 3 < size; i += 4) {
		uint8_t t0 = b[i], t1 = b[i + 1];
		b[i] = b[i + 3]; b[i + 1] = b[i + 2]; b[i + 2] = t1; b[i + 3] = t0;
	}
}

/***************************************************************************
    instructions
***************************************************************************/

#define N  ((op >> 8) & 15)
#define M  ((op >> 4) & 15)
#define RN c->r[N]
#define RM c->r[M]
#define SETT_IF(cond) do { if (cond) c->sr |= SH_T; else c->sr &= ~SH_T; } while (0)
#define SEXT8(x)  ((uint32_t)(int32_t)(int8_t)(x))
#define SEXT12(x) ((uint32_t)(((int32_t)((x) << 20)) >> 20))
#define SEXT16(x) ((uint32_t)(int32_t)(int16_t)(x))

SH3_NOINLINE void op_trapa(sh3_t *c, uint32_t i)
{
	uint32_t imm = i & 0xff;
	c->upper[TRA] = imm << 2;
	c->ssr = c->sr;
	c->spc = c->pc;
	c->sgr = c->r[15];
	c->sr |= SH_MD;
	if (!(c->sr & SH_RB))
		change_register_bank(c, 1);
	c->sr |= SH_RB;
	c->sr |= SH_BL;
	exception_recompute(c);
	c->upper[EXPEVT] = 0x00000160;
	c->pc = c->vbr + 0x00000100;
	c->drc_pc = c->pc;
}

SH3_NOINLINE void op_div1(sh3_t *c, uint32_t m, uint32_t n)
{
	uint32_t old_q = c->sr & SH_Q;
	if (0x80000000 & c->r[n]) c->sr |= SH_Q; else c->sr &= ~SH_Q;
	c->r[n] = (c->r[n] << 1) | (c->sr & SH_T);
	uint32_t tmp = c->r[n];
	if (!old_q) {
		if (!(c->sr & SH_M)) {
			c->r[n] -= c->r[m];
			if (!(c->sr & SH_Q)) { if (c->r[n] > tmp) c->sr |= SH_Q; else c->sr &= ~SH_Q; }
			else { if (c->r[n] > tmp) c->sr &= ~SH_Q; else c->sr |= SH_Q; }
		} else {
			c->r[n] += c->r[m];
			if (!(c->sr & SH_Q)) { if (c->r[n] < tmp) c->sr &= ~SH_Q; else c->sr |= SH_Q; }
			else { if (c->r[n] < tmp) c->sr |= SH_Q; else c->sr &= ~SH_Q; }
		}
	} else {
		if (!(c->sr & SH_M)) {
			c->r[n] += c->r[m];
			if (!(c->sr & SH_Q)) { if (c->r[n] < tmp) c->sr |= SH_Q; else c->sr &= ~SH_Q; }
			else { if (c->r[n] < tmp) c->sr &= ~SH_Q; else c->sr |= SH_Q; }
		} else {
			c->r[n] -= c->r[m];
			if (!(c->sr & SH_Q)) { if (c->r[n] > tmp) c->sr &= ~SH_Q; else c->sr |= SH_Q; }
			else { if (c->r[n] > tmp) c->sr |= SH_Q; else c->sr &= ~SH_Q; }
		}
	}
	uint32_t qm = c->sr & (SH_Q | SH_M);
	if (qm == 0 || qm == 0x300) c->sr |= SH_T; else c->sr &= ~SH_T;
}

SH3_NOINLINE void op_mac_l(sh3_t *c, uint32_t m, uint32_t n)
{
	int32_t tempn = (int32_t)RL(c, c->r[n]);
	c->r[n] += 4;
	int32_t tempm = (int32_t)RL(c, c->r[m]);
	c->r[m] += 4;
	int64_t p = (int64_t)tempn * tempm;
	uint32_t res2 = (uint32_t)((uint64_t)p >> 32);
	uint32_t res0 = (uint32_t)p;
	if (c->sr & SH_S) {
		res0 = c->macl + res0;
		if (c->macl > res0) res2++;
		res2 += c->mach & 0x0000ffff;
		if ((int32_t)res2 < 0 && res2 < 0xffff8000) { res2 = 0x00008000; res0 = 0x00000000; }
		else if ((int32_t)res2 > 0 && res2 > 0x00007fff) { res2 = 0x00007fff; res0 = 0xffffffff; }
		c->mach = res2;
		c->macl = res0;
	} else {
		res0 = c->macl + res0;
		if (c->macl > res0) res2++;
		res2 += c->mach;
		c->mach = res2;
		c->macl = res0;
	}
}

SH3_NOINLINE void op_mac_w(sh3_t *c, uint32_t m, uint32_t n)
{
	int32_t tempn = (int32_t)(int16_t)sh3_read16(c, c->r[n]);
	c->r[n] += 2;
	int32_t tempm = (int32_t)(int16_t)sh3_read16(c, c->r[m]);
	c->r[m] += 2;
	uint32_t templ = c->macl;
	tempm = (int32_t)((uint32_t)tempm * (uint32_t)tempn);
	int32_t dest = (c->macl >> 31) & 1;
	int32_t src = (((uint32_t)tempm >> 31) & 1) + dest;
	tempn = ((uint32_t)tempm >> 31) ? -1 : 0;
	c->macl += (uint32_t)tempm;
	int32_t ans = ((c->macl >> 31) & 1) + dest;
	if (c->sr & SH_S) {
		if (ans == 1) {
			if (src == 0) c->macl = 0x7fffffff;
			else if (src == 2) c->macl = 0x80000000;
		}
	} else {
		c->mach += (uint32_t)tempn;
		if (templ > c->macl) c->mach += 1;
	}
}

// LDC Rm,SR (sh34: the value is taken before the bank change)
SH3_NOINLINE void op_ldcsr(sh3_t *c, uint32_t v)
{
	if ((v & SH_RB) != (c->sr & SH_RB))
		change_register_bank(c, (v & SH_RB) ? 1 : 0);
	c->sr = v & SH34_FLAGS;
	exception_recompute(c);
}

SH3_NOINLINE void op_ldcmsr(sh3_t *c, uint32_t n)
{
	uint32_t old = c->sr;
	c->sr = RL(c, c->r[n]) & SH34_FLAGS;
	if ((old & SH_RB) != (c->sr & SH_RB))
		change_register_bank(c, (c->sr & SH_RB) ? 1 : 0);
	c->r[n] += 4;   // (the register of the new bank, as MAME)
	exception_recompute(c);
}

SH3_NOINLINE void op_rte(sh3_t *c)
{
	c->delay = c->spc;
	c->drc_pc = c->spc;
	if ((c->ssr & SH_RB) != (c->sr & SH_RB))
		change_register_bank(c, (c->ssr & SH_RB) ? 1 : 0);
	c->sr = c->ssr;
	exception_recompute(c);
}

SH3_NOINLINE void op_sleep(sh3_t *c)
{
	if (c->sleep_mode == 1 && !c->test_irq && c->icount > 3) {
		int32_t rounds = (c->icount + 2) / 3;   // (nothing wakes the CPU inside the slice: run the rounds at once)
		c->icount -= 3 * (rounds - 1);
	}
	if (c->sleep_mode != 2)
		c->pc -= 2;
	if (c->sleep_mode == 0) c->sleep_mode = 1;
	else if (c->sleep_mode == 2) c->sleep_mode = 0;
}

#define SH3_OPLIST(X) \
	X(NOP) X(STC_SR) X(STC_GBR) X(STC_VBR) X(STC_SSR) X(STC_SPC) X(STC_SGR) X(STC_DBR) X(STC_RBANK) X(STS_FPUL) \
	X(STS_FPSCR) X(BSRF) X(BRAF) X(MOVB_S0) X(MOVW_S0) X(MOVL_S0) X(MULL) X(CLRT) X(SETT) X(CLRS) X(SETS) \
	X(CLRMAC) X(DIV0U) X(MOVT) X(STS_MACH) X(STS_MACL) X(STS_PR) X(RTS) X(SLEEP) X(RTE) X(MOVCAL) \
	X(MOVB_L0) X(MOVW_L0) X(MOVL_L0) X(MACL) X(MOVL_S4) \
	X(MOVB_S) X(MOVW_S) X(MOVL_S) X(MOVB_M) X(MOVW_M) X(MOVL_M) X(DIV0S) X(TST) X(AND) X(XOR) X(OR) X(CMPSTR) \
	X(XTRCT) X(MULU) X(MULS) \
	X(CMPEQ) X(CMPHS) X(CMPGE) X(DIV1) X(DMULU) X(CMPHI) X(CMPGT) X(SUB) X(SUBC) X(SUBV) X(ADD) X(DMULS) X(ADDC) \
	X(ADDV) \
	X(SHLL) X(SHLR) X(SHAR) X(STSM_MACH) X(STSM_MACL) X(STSM_PR) X(STCM_SR) X(STCM_GBR) X(STCM_VBR) X(ROTL) \
	X(ROTR) X(ROTCL) X(ROTCR) X(LDSM_MACH) X(LDSM_MACL) X(LDSM_PR) X(LDCM_SR) X(LDCM_GBR) X(LDCM_VBR) X(SHLL2) \
	X(SHLL8) X(SHLL16) X(SHLR2) X(SHLR8) X(SHLR16) X(LDS_MACH) X(LDS_MACL) X(LDS_PR) X(JSR) X(JMP) X(TAS) \
	X(LDC_SR) X(LDC_GBR) X(LDC_VBR) X(MACW) X(DT) X(CMPPZ) X(CMPPL) \
	X(SHAD) X(SHLD) X(LDC_RBANK) X(STCM_RBANK) X(LDCM_RBANK) X(STCM_SGR) X(STCM_SSR) X(LDCM_SSR) X(LDC_SSR) \
	X(STCM_SPC) X(LDCM_SPC) X(LDC_SPC) X(STSM_FPUL) X(LDSM_FPUL) X(LDS_FPUL) X(STSM_FPSCR) X(LDSM_FPSCR) \
	X(LDS_FPSCR) X(STCM_DBR) X(LDCM_DBR) X(LDC_DBR) \
	X(MOVL_L4) \
	X(MOVB_L) X(MOVW_L) X(MOVL_L) X(MOV) X(MOVB_P) X(MOVW_P) X(MOVL_P) X(NOT) X(SWAPB) X(SWAPW) X(NEGC) X(NEG) \
	X(EXTUB) X(EXTUW) X(EXTSB) X(EXTSW) \
	X(ADDI) \
	X(MOVB_S4R0) X(MOVW_S4R0) X(MOVB_L4R0) X(MOVW_L4R0) X(CMPIM) X(BT) X(BF) X(BTS) X(BFS) \
	X(MOVW_I) X(BRA) X(BSR) \
	X(MOVB_SG) X(MOVW_SG) X(MOVL_SG) X(TRAPA) X(MOVB_LG) X(MOVW_LG) X(MOVL_LG) X(MOVA) X(TSTI) X(ANDI) X(XORI) \
	X(ORI) X(TSTB) X(ANDB) X(XORB) X(ORB) \
	X(MOVL_I) X(MOVI)

#define SH3_ENUM(n) H_##n,
enum { SH3_OPLIST(SH3_ENUM) H_COUNT };

static struct { uint8_t tab[65536]; uint8_t cost[65536]; uint8_t flags[65536]; const void *lab[H_COUNT]; } s_disp;
static int s_disp_ok;

// sh_common_execution::execute_one (the SH-2 decode, illegal = NOP for the sh34)
static int decode_common(uint32_t op)
{
	switch (op >> 12) {
	case 0x0:
		switch (op & 0x3f) {
		case 0x02: return H_STC_SR;
		case 0x12: return H_STC_GBR;
		case 0x22: return H_STC_VBR;
		case 0x03: return H_BSRF;
		case 0x23: return H_BRAF;
		case 0x04: case 0x14: case 0x24: case 0x34: return H_MOVB_S0;
		case 0x05: case 0x15: case 0x25: case 0x35: return H_MOVW_S0;
		case 0x06: case 0x16: case 0x26: case 0x36: return H_MOVL_S0;
		case 0x07: case 0x17: case 0x27: case 0x37: return H_MULL;
		case 0x08: return H_CLRT;
		case 0x18: return H_SETT;
		case 0x28: return H_CLRMAC;
		case 0x09: return H_NOP;
		case 0x19: return H_DIV0U;
		case 0x29: return H_MOVT;
		case 0x0a: return H_STS_MACH;
		case 0x1a: return H_STS_MACL;
		case 0x2a: return H_STS_PR;
		case 0x0b: return H_RTS;
		case 0x1b: return H_SLEEP;
		case 0x2b: return H_RTE;
		case 0x0c: case 0x1c: case 0x2c: case 0x3c: return H_MOVB_L0;
		case 0x0d: case 0x1d: case 0x2d: case 0x3d: return H_MOVW_L0;
		case 0x0e: case 0x1e: case 0x2e: case 0x3e: return H_MOVL_L0;
		case 0x0f: case 0x1f: case 0x2f: case 0x3f: return H_MACL;
		default: return H_NOP;
		}
	case 0x1: return H_MOVL_S4;
	case 0x2:
		switch (op & 15) {
		case 0: return H_MOVB_S;
		case 1: return H_MOVW_S;
		case 2: return H_MOVL_S;
		case 4: return H_MOVB_M;
		case 5: return H_MOVW_M;
		case 6: return H_MOVL_M;
		case 7: return H_DIV0S;
		case 8: return H_TST;
		case 9: return H_AND;
		case 10: return H_XOR;
		case 11: return H_OR;
		case 12: return H_CMPSTR;
		case 13: return H_XTRCT;
		case 14: return H_MULU;
		case 15: return H_MULS;
		default: return H_NOP;
		}
	case 0x3:
		switch (op & 15) {
		case 0: return H_CMPEQ;
		case 2: return H_CMPHS;
		case 3: return H_CMPGE;
		case 4: return H_DIV1;
		case 5: return H_DMULU;
		case 6: return H_CMPHI;
		case 7: return H_CMPGT;
		case 8: return H_SUB;
		case 10: return H_SUBC;
		case 11: return H_SUBV;
		case 12: return H_ADD;
		case 13: return H_DMULS;
		case 14: return H_ADDC;
		case 15: return H_ADDV;
		default: return H_NOP;
		}
	case 0x4:
		switch (op & 0x3f) {
		case 0x00: case 0x20: return H_SHLL;
		case 0x01: return H_SHLR;
		case 0x21: return H_SHAR;
		case 0x02: return H_STSM_MACH;
		case 0x12: return H_STSM_MACL;
		case 0x22: return H_STSM_PR;
		case 0x03: return H_STCM_SR;
		case 0x13: return H_STCM_GBR;
		case 0x23: return H_STCM_VBR;
		case 0x04: return H_ROTL;
		case 0x05: return H_ROTR;
		case 0x24: return H_ROTCL;
		case 0x25: return H_ROTCR;
		case 0x06: return H_LDSM_MACH;
		case 0x16: return H_LDSM_MACL;
		case 0x26: return H_LDSM_PR;
		case 0x07: return H_LDCM_SR;
		case 0x17: return H_LDCM_GBR;
		case 0x27: return H_LDCM_VBR;
		case 0x08: return H_SHLL2;
		case 0x18: return H_SHLL8;
		case 0x28: return H_SHLL16;
		case 0x09: return H_SHLR2;
		case 0x19: return H_SHLR8;
		case 0x29: return H_SHLR16;
		case 0x0a: return H_LDS_MACH;
		case 0x1a: return H_LDS_MACL;
		case 0x2a: return H_LDS_PR;
		case 0x0b: return H_JSR;
		case 0x2b: return H_JMP;
		case 0x1b: return H_TAS;
		case 0x0e: return H_LDC_SR;
		case 0x1e: return H_LDC_GBR;
		case 0x2e: return H_LDC_VBR;
		case 0x0f: case 0x1f: case 0x2f: case 0x3f: return H_MACW;
		case 0x10: return H_DT;
		case 0x11: return H_CMPPZ;
		case 0x15: return H_CMPPL;
		default: return H_NOP;
		}
	case 0x5: return H_MOVL_L4;
	case 0x6:
		switch (op & 15) {
		case 0: return H_MOVB_L;
		case 1: return H_MOVW_L;
		case 2: return H_MOVL_L;
		case 3: return H_MOV;
		case 4: return H_MOVB_P;
		case 5: return H_MOVW_P;
		case 6: return H_MOVL_P;
		case 7: return H_NOT;
		case 8: return H_SWAPB;
		case 9: return H_SWAPW;
		case 10: return H_NEGC;
		case 11: return H_NEG;
		case 12: return H_EXTUB;
		case 13: return H_EXTUW;
		case 14: return H_EXTSB;
		default: return H_EXTSW;
		}
	case 0x7: return H_ADDI;
	case 0x8:
		switch ((op >> 8) & 15) {
		case 0: return H_MOVB_S4R0;
		case 1: return H_MOVW_S4R0;
		case 4: return H_MOVB_L4R0;
		case 5: return H_MOVW_L4R0;
		case 8: return H_CMPIM;
		case 9: return H_BT;
		case 11: return H_BF;
		case 13: return H_BTS;
		case 15: return H_BFS;
		default: return H_NOP;
		}
	case 0x9: return H_MOVW_I;
	case 0xa: return H_BRA;
	case 0xb: return H_BSR;
	case 0xc:
		switch ((op >> 8) & 15) {
		case 0: return H_MOVB_SG;
		case 1: return H_MOVW_SG;
		case 2: return H_MOVL_SG;
		case 3: return H_TRAPA;
		case 4: return H_MOVB_LG;
		case 5: return H_MOVW_LG;
		case 6: return H_MOVL_LG;
		case 7: return H_MOVA;
		case 8: return H_TSTI;
		case 9: return H_ANDI;
		case 10: return H_XORI;
		case 11: return H_ORI;
		case 12: return H_TSTB;
		case 13: return H_ANDB;
		case 14: return H_XORB;
		default: return H_ORB;
		}
	case 0xd: return H_MOVL_I;
	case 0xe: return H_MOVI;
	default: return H_NOP;   // (the FPU group: the SH-3 has none)
	}
}

// sh34_base_device::execute_one_0000 / execute_one_4000, then the common decode
static int decode(uint32_t op)
{
	uint32_t lo = op & 0xff;
	if ((op >> 12) == 0x0) {
		switch (lo) {
		case 0x52: case 0x62: case 0x43: case 0x63: case 0xe3: case 0x68: case 0xe8: case 0x4a: case 0xca:
		case 0x93: case 0xa3: case 0xb3: case 0x38: case 0xb8: case 0x83:
			return H_NOP;   // (illegal, TODO, LDTLB, PREF: no-ops)
		case 0x82: case 0x92: case 0xa2: case 0xb2: case 0xc2: case 0xd2: case 0xe2: case 0xf2: return H_STC_RBANK;
		case 0x32: return H_STC_SSR;
		case 0x42: return H_STC_SPC;
		case 0xc3: return H_MOVCAL;
		case 0x48: case 0xc8: return H_CLRS;
		case 0x58: case 0xd8: return H_SETS;
		case 0x3a: case 0xba: return H_STC_SGR;
		case 0x5a: case 0xda: return H_STS_FPUL;
		case 0x6a: case 0xea: return H_STS_FPSCR;
		case 0x7a: case 0xfa: return H_STC_DBR;
		default: break;
		}
	} else if ((op >> 12) == 0x4) {
		switch (lo) {
		case 0x42: case 0x46: case 0x4a: case 0x53: case 0x57: case 0x5e: case 0x63: case 0x67: case 0x6e:
		case 0x82: case 0x86: case 0x8a: case 0x92: case 0x96: case 0x9a: case 0xa2: case 0xa6: case 0xaa:
		case 0xc2: case 0xc6: case 0xca: case 0xd2: case 0xd6: case 0xda: case 0xe2: case 0xe6: case 0xea:
			return H_NOP;
		case 0x32: return H_STCM_SGR;
		case 0x33: return H_STCM_SSR;
		case 0x37: return H_LDCM_SSR;
		case 0x3e: return H_LDC_SSR;
		case 0x43: return H_STCM_SPC;
		case 0x47: return H_LDCM_SPC;
		case 0x4e: return H_LDC_SPC;
		case 0x52: return H_STSM_FPUL;
		case 0x56: return H_LDSM_FPUL;
		case 0x5a: return H_LDS_FPUL;
		case 0x62: return H_STSM_FPSCR;
		case 0x66: return H_LDSM_FPSCR;
		case 0x6a: return H_LDS_FPSCR;
		case 0xf2: return H_STCM_DBR;
		case 0xf6: return H_LDCM_DBR;
		case 0xfa: return H_LDC_DBR;
		default: break;
		}
		if ((lo & 0x0f) == 0x0c) return H_SHAD;
		if ((lo & 0x0f) == 0x0d) return H_SHLD;
		if (lo >= 0x80) {
			if ((lo & 0x0f) == 0x0e) return H_LDC_RBANK;
			if ((lo & 0x0f) == 0x03) return H_STCM_RBANK;
			if ((lo & 0x0f) == 0x07) return H_LDCM_RBANK;
		}
	}
	return decode_common(op);
}

// the cycles of an instruction: MAME's DRC (sh_fe.cpp / sh4fe.cpp desc.cycles), which MAME runs; a branch costs
// the same taken or not (the interpreter's counts differ: BT taken 3, not taken 1 ...)
static int op_cost(int h)
{
	switch (h) {
	case H_BRA: case H_BSR: case H_BTS: case H_BFS: case H_RTS: case H_BRAF:
	case H_MULU: case H_MULS: case H_DMULU: case H_DMULS: case H_MULL: case H_STCM_SR:
		return 2;
	case H_BT: case H_BF: case H_MACL: case H_MACW: case H_SLEEP: case H_LDCM_SR:
		return 3;
	case H_RTE: case H_TAS:
		return 4;
	case H_TRAPA:
		return 8;
	default:
		return 1;
	}
}

// the instructions the DRC runs through a C helper with arg0 = the opcode
static int op_helper(int h)
{
	switch (h) {
	case H_STC_RBANK: case H_LDC_RBANK: case H_STCM_RBANK: case H_LDCM_RBANK: case H_CLRS: case H_SETS: case H_TRAPA:
	case H_LDC_SR: case H_LDCM_SR: case H_SHAD: case H_SHLD: case H_STCM_SGR: case H_STCM_SSR: case H_LDCM_SSR:
	case H_LDC_SSR: case H_STCM_SPC: case H_LDCM_SPC: case H_LDC_SPC: case H_STSM_FPUL: case H_LDSM_FPUL: case H_LDS_FPUL:
	case H_STSM_FPSCR: case H_LDSM_FPSCR: case H_LDS_FPSCR: case H_STCM_DBR: case H_LDCM_DBR: case H_LDC_DBR:
	case H_DIV1: case H_SUBV: case H_ADDV: case H_MACL: case H_MACW:
		return 1;
	default:
		return 0;
	}
}
#define F_ICACHE 1   // drc_update_icache before it (groups 0, 4, 15)
#define F_ARG0   2   // a C helper with arg0 = the opcode
#define F_MEM    4   // the DRC counts the cycles off after it (a memory access, outside a delay slot)
#define F_DISC   8   // the DRC drops the cycles counted since the last count-off (interrupt check, TRAPA)

// the instructions after which MAME's DRC calls generate_update_cycles (if not in a delay slot)
static int op_mem(int h)
{
	switch (h) {
	case H_MOVL_S4: case H_MOVL_L4: case H_MOVW_I: case H_MOVL_I:
	case H_MOVB_S: case H_MOVW_S: case H_MOVL_S: case H_MOVB_M: case H_MOVW_M: case H_MOVL_M:
	case H_MOVB_L: case H_MOVW_L: case H_MOVL_L: case H_MOVB_P: case H_MOVW_P: case H_MOVL_P:
	case H_MOVB_S4R0: case H_MOVW_S4R0: case H_MOVB_L4R0: case H_MOVW_L4R0:
	case H_MOVB_SG: case H_MOVW_SG: case H_MOVL_SG: case H_MOVB_LG: case H_MOVW_LG: case H_MOVL_LG:
	case H_MOVB_S0: case H_MOVW_S0: case H_MOVL_S0: case H_MOVB_L0: case H_MOVW_L0: case H_MOVL_L0:
	case H_STSM_MACH: case H_STCM_SR: case H_LDSM_MACH: case H_STSM_MACL: case H_STCM_GBR: case H_LDSM_MACL:
	case H_LDCM_GBR: case H_TAS: case H_STSM_PR: case H_STCM_VBR: case H_LDSM_PR: case H_LDCM_VBR:
	case H_SLEEP: case H_MOVCAL:
		return 1;
	default:
		return 0;
	}
}

static void build_disp(const void *const *lab)
{
	for (uint32_t op = 0; op < 65536; op++) {
		s_disp.tab[op] = (uint8_t)decode(op);
		s_disp.cost[op] = (uint8_t)op_cost(s_disp.tab[op]);
		const int h = s_disp.tab[op];
		s_disp.flags[op] = (uint8_t)((((0x8011u >> (op >> 12)) & 1) ? F_ICACHE : 0) | (op_helper(h) ? F_ARG0 : 0) |
		                             (op_mem(h) ? F_MEM : 0) | ((h == H_LDC_SR || h == H_LDCM_SR || h == H_TRAPA) ? F_DISC : 0));
	}
	for (int i = 0; i < H_COUNT; i++)
		s_disp.lab[i] = lab[i];
	s_disp_ok = 1;
}

#define SYNC()   do { c->pc = pc; c->delay = delay; c->icount = icount; } while (0)
#define RELOAD() do { pc = c->pc; delay = c->delay; icount = c->icount; ftag = c->fetch_page; \
	fptr = (uintptr_t)c->fetch_base - (uintptr_t)(uint32_t)(ftag << SH3_PAGE_SHIFT); tirq = c->test_irq; } while (0)
#define CALL(stmt) do { SYNC(); stmt; RELOAD(); } while (0)

// the accesses of the DRC's memory accessor (with the penalty) and of its C helpers (N: without)
#define RD8N(v, addr) do { uint32_t a_ = (addr); const uint8_t *p_ = FASTP(c->rd, a_); \
	if (LIKELY(p_ != NULL)) (v) = p_[(a_ & 0xfff) ^ 3]; \
	else { SYNC(); c->rnorm = !delay_was; (v) = slow_r(c, a_, 1); c->rnorm = 0; RELOAD(); } } while (0)
#define RD16N(v, addr) do { uint32_t a_ = (addr) & ~1u; const uint8_t *p_ = FASTP(c->rd, a_); \
	if (LIKELY(p_ != NULL)) (v) = *(const uint16_t *)(p_ + ((a_ & 0xffe) ^ 2)); \
	else { SYNC(); c->rnorm = !delay_was; (v) = slow_r(c, a_, 2); c->rnorm = 0; RELOAD(); } } while (0)
#define RD32N(v, addr) do { uint32_t a_ = (addr) & ~3u; const uint8_t *p_ = FASTP(c->rd, a_); \
	if (LIKELY(p_ != NULL)) (v) = *(const uint32_t *)(p_ + (a_ & 0xffc)); \
	else { SYNC(); c->rnorm = !delay_was; (v) = slow_r(c, a_, 4); c->rnorm = 0; RELOAD(); } } while (0)
#define WR8N(addr, v) do { uint32_t a_ = (addr), v_ = (v); uint8_t *p_ = FASTP(c->wr, a_); \
	if (LIKELY(p_ != NULL)) p_[(a_ & 0xfff) ^ 3] = (uint8_t)v_; \
	else { SYNC(); slow_w(c, a_, v_ & 0xff, 1); RELOAD(); } } while (0)
#define WR16N(addr, v) do { uint32_t a_ = (addr) & ~1u, v_ = (v); uint8_t *p_ = FASTP(c->wr, a_); \
	if (LIKELY(p_ != NULL)) *(uint16_t *)(p_ + ((a_ & 0xffe) ^ 2)) = (uint16_t)v_; \
	else { SYNC(); slow_w(c, a_, v_ & 0xffff, 2); RELOAD(); } } while (0)
#define WR32N(addr, v) do { uint32_t a_ = (addr) & ~3u, v_ = (v); uint8_t *p_ = FASTP(c->wr, a_); \
	if (LIKELY(p_ != NULL)) *(uint32_t *)(p_ + (a_ & 0xffc)) = v_; \
	else { SYNC(); slow_w(c, a_, v_, 4); RELOAD(); } } while (0)

#define RD8(v, addr) do { uint32_t pa_ = (addr); RD8N(v, pa_); icount -= (int32_t)data_penalty(c, pa_, 0); } while (0)
#define RD16(v, addr) do { uint32_t pa_ = (addr); RD16N(v, pa_); icount -= (int32_t)data_penalty(c, pa_, 0); } while (0)
#define RD32(v, addr) do { uint32_t pa_ = (addr); RD32N(v, pa_); icount -= (int32_t)data_penalty(c, pa_, 0); } while (0)
#define WR8(addr, v) do { uint32_t pa_ = (addr); WR8N(pa_, v); icount -= (int32_t)data_penalty(c, pa_, 1); } while (0)
#define WR16(addr, v) do { uint32_t pa_ = (addr); WR16N(pa_, v); icount -= (int32_t)data_penalty(c, pa_, 1); } while (0)
#define WR32(addr, v) do { uint32_t pa_ = (addr); WR32N(pa_, v); icount -= (int32_t)data_penalty(c, pa_, 1); } while (0)

// fetch: ftag = the page of the unmasked pc whose host page is fetch_base
#define FETCH() do { \
	if (LIKELY((pc >> SH3_PAGE_SHIFT) == ftag)) op = *(const uint16_t *)(fptr + ((pc & ~1u) ^ 2)); \
	else CALL(op = fetch_slow(c, pc)); } while (0)
#define GO() do { \
	if (UNLIKELY(c->trace != NULL)) { SYNC(); c->trace(c, delay_was ? delay_was : pc - 2); } \
	if (UNLIKELY(pc == c->flush_pc + 2)) c->drc_pc = pc - 2; \
	if (s_disp.flags[op]) { \
		if (s_disp.flags[op] & F_ARG0) c->arg0 = op; \
		if (s_disp.flags[op] & F_ICACHE) icount -= (int32_t)icache_penalty(c); \
	} \
	goto *s_disp.lab[s_disp.tab[op]]; } while (0)
#define DISPATCH() do { FETCH(); delay_was = 0; \
	if (UNLIKELY(delay)) { delay_was = pc; pc = delay; delay = 0; } else pc += 2; \
	GO(); } while (0)
// MAME's DRC counts an instruction's cycles (pend) off at its update points: after a memory access outside a delay
// slot, after a taken branch (its delay slot included), at the end of a sequence; the timeslice ends there when
// icount goes negative. A conditional branch not taken drops the cycles gathered since the last update (the update is
// compiled into the taken path only), as do the interrupt checks after LDC SR / RTE and TRAPA.
#define NEXT() do { \
	pend += s_disp.cost[op]; \
	if (UNLIKELY(in_slot)) { \
		if (in_slot == 2) pend = 0; \
		else { icount -= (int32_t)pend; pend = 0; if (icount < 0 && !tirq) { in_slot = 0; goto out; } } \
		in_slot = 0; \
		if (!tirq) JX(); \
	} else if (s_disp.flags[op] & (F_MEM | F_DISC)) { \
		if (s_disp.flags[op] & F_DISC) pend = 0; \
		else { icount -= (int32_t)pend; pend = 0; if (icount < 0 && !tirq) goto out; } \
	} \
	if (UNLIKELY(tirq)) goto irq_check; \
	FETCH(); pc += 2; delay_was = 0; GO(); } while (0)
// a taken delayed branch: its slot next, then the update (slot 2: the update drops the cycles)
#define NEXT_BR() do { pend += s_disp.cost[op]; in_slot = 1; DISPATCH(); } while (0)
#define NEXT_BR_DISC() do { pend += s_disp.cost[op]; in_slot = 2; DISPATCH(); } while (0)
// a conditional branch not taken: the cycles gathered so far and its own are dropped
#define NEXT_NOT_TAKEN() do { pend = 0; \
	if (UNLIKELY(tirq)) goto irq_check; \
	FETCH(); pc += 2; delay_was = 0; GO(); } while (0)
// a taken BT / BF: counted off at once
#define NEXT_TAKEN() do { pend += s_disp.cost[op]; icount -= (int32_t)pend; pend = 0; \
	if (icount < 0 && !tirq) goto out; \
	if (UNLIKELY(tirq)) goto irq_check; \
	JX(); \
	FETCH(); pc += 2; delay_was = 0; GO(); } while (0)
// after a handler that may change test_irq (the exception recompute)
#define NEXT_T() do { tirq = c->test_irq; NEXT(); } while (0)

// the JIT's run loop (sh3_jit.inc): the interpreter leaves at a taken transfer (after its delay slot, the count-off,
// no interrupt to test), pend 0, for the block at the target
#ifdef SH3_JIT
#define JX() do { if (UNLIKELY(jx)) { SYNC(); c->jpend = pend; return 1; } } while (0)
#else
#define JX() do { } while (0)
#endif

#define SH3_LAB(n) &&L_##n,

// entry 0: a slice's start; 1: on at c->pc with c->jpend gathered (sh3_jit.inc); 2: the same, the interrupt test
// first. Returns 0 at the slice's end, 1 when it left for the JIT (JX)
static int run(sh3_t *c, int entry)
{
	static const void *const lab[H_COUNT] = { SH3_OPLIST(SH3_LAB) };
	uint32_t pc, delay, tirq, op, delay_was = 0;
	uint32_t pend = 0, in_slot = 0;
	int32_t icount;
	uint32_t ftag;
	uintptr_t fptr;

	if (UNLIKELY(!s_disp_ok))
		build_disp(lab);
	RELOAD();
#ifdef SH3_JIT
	const int jx = entry != 0;
#endif
	if (entry) {
		pend = c->jpend;
		if (entry == 2)
			goto irq_check;
	} else if (c->test_irq && !delay)   // an interrupt raised between slices is taken before the first instruction (as MAME's DRC entry)
		CALL(check_pending_irq(c));
	tirq = c->test_irq;
	DISPATCH();

irq_check:
	CALL(check_pending_irq(c));
	if (UNLIKELY(icount < 0)) goto out;
	if (!tirq) JX();
	DISPATCH();

out:
	SYNC();
	return 0;

	// ---- 0xxx
L_NOP:      NEXT();
L_STC_SR:   RN = c->sr; NEXT();
L_STC_GBR:  RN = c->gbr; NEXT();
L_STC_VBR:  RN = c->vbr; NEXT();
L_STC_SSR:  RN = c->ssr; NEXT();
L_STC_SPC:  RN = c->spc; NEXT();
L_STC_SGR:  RN = c->sgr; NEXT();
L_STC_DBR:  RN = c->dbr; NEXT();
L_STC_RBANK: RN = c->rbnk[(c->sr & SH_RB) ? 0 : 1][M & 7]; NEXT();                 // STC Rm_BANK,Rn
L_STS_FPUL: RN = c->fpul; NEXT();
L_STS_FPSCR: RN = c->fpscr & 0x003fffff; NEXT();
L_BSRF:     c->pr = pc + 2; delay = pc + RN + 2; NEXT_BR();
L_BRAF:     delay = pc + RN + 2; NEXT_BR();
L_MOVB_S0:  WR8(RN + c->r[0], RM); NEXT();
L_MOVW_S0:  WR16(RN + c->r[0], RM); NEXT();
L_MOVL_S0:  WR32(RN + c->r[0], RM); NEXT();
L_MULL:     c->macl = RN * RM; NEXT();
L_CLRT:     c->sr &= ~SH_T; NEXT();
L_SETT:     c->sr |= SH_T; NEXT();
L_CLRS:     c->sr &= ~SH_S; NEXT();
L_SETS:     c->sr |= SH_S; NEXT();
L_CLRMAC:   c->mach = 0; c->macl = 0; NEXT();
L_DIV0U:    c->sr &= ~(SH_M | SH_Q | SH_T); NEXT();
L_MOVT:     RN = c->sr & SH_T; NEXT();
L_STS_MACH: RN = c->mach; NEXT();
L_STS_MACL: RN = c->macl; NEXT();
L_STS_PR:   RN = c->pr; NEXT();
L_RTS:      delay = c->pr; NEXT_BR();
L_SLEEP:    CALL(op_sleep(c)); NEXT();
L_RTE:      CALL(op_rte(c)); tirq = c->test_irq; NEXT_BR_DISC();
L_MOVCAL:   WR32(RN, c->r[0]); NEXT();                                              // MOVCA.L R0,@Rn
L_MOVB_L0:  { uint32_t v; RD8(v, RM + c->r[0]); RN = SEXT8(v); } NEXT();
L_MOVW_L0:  { uint32_t v; RD16(v, RM + c->r[0]); RN = SEXT16(v); } NEXT();
L_MOVL_L0:  { uint32_t v; RD32(v, RM + c->r[0]); RN = v; } NEXT();
L_MACL:     CALL(op_mac_l(c, M, N)); NEXT();

	// ---- 1xxx
L_MOVL_S4:  WR32(RN + (op & 15) * 4, RM); NEXT();

	// ---- 2xxx
L_MOVB_S:   WR8(RN, RM); NEXT();
L_MOVW_S:   WR16(RN, RM); NEXT();
L_MOVL_S:   WR32(RN, RM); NEXT();
L_MOVB_M:   { uint32_t d = RM; RN -= 1; WR8(RN, d); } NEXT();
L_MOVW_M:   { uint32_t d = RM; RN -= 2; WR16(RN, d); } NEXT();
L_MOVL_M:   { uint32_t d = RM; RN -= 4; WR32(RN, d); } NEXT();
L_DIV0S: {
		uint32_t sr = c->sr & ~(SH_Q | SH_M | SH_T);
		if (RN & 0x80000000) sr |= SH_Q;
		if (RM & 0x80000000) sr |= SH_M;
		if ((RM ^ RN) & 0x80000000) sr |= SH_T;
		c->sr = sr;
	}
	NEXT();
L_TST:      SETT_IF((RN & RM) == 0); NEXT();
L_AND:      RN &= RM; NEXT();
L_XOR:      RN ^= RM; NEXT();
L_OR:       RN |= RM; NEXT();
L_CMPSTR: {
		uint32_t t = RN ^ RM;
		if ((t >> 24) && ((t >> 16) & 0xff) && ((t >> 8) & 0xff) && (t & 0xff)) c->sr &= ~SH_T;
		else c->sr |= SH_T;
	}
	NEXT();
L_XTRCT:    RN = (RN >> 16) | (RM << 16); NEXT();
L_MULU:     c->macl = (uint32_t)(uint16_t)RN * (uint16_t)RM; NEXT();
L_MULS:     c->macl = (uint32_t)((int32_t)(int16_t)RN * (int16_t)RM); NEXT();

	// ---- 3xxx
L_CMPEQ:    SETT_IF(RN == RM); NEXT();
L_CMPHS:    SETT_IF(RN >= RM); NEXT();
L_CMPGE:    SETT_IF((int32_t)RN >= (int32_t)RM); NEXT();
L_DIV1:     op_div1(c, M, N); NEXT();
L_DMULU:    { uint64_t p = (uint64_t)RN * RM; c->mach = (uint32_t)(p >> 32); c->macl = (uint32_t)p; } NEXT();
L_CMPHI:    SETT_IF(RN > RM); NEXT();
L_CMPGT:    SETT_IF((int32_t)RN > (int32_t)RM); NEXT();
L_SUB:      RN -= RM; NEXT();
L_SUBC: {
		uint32_t tmp1 = RN - RM;
		uint32_t tmp0 = RN;
		RN = tmp1 - (c->sr & SH_T);
		SETT_IF(tmp0 < tmp1);
		if (tmp1 < RN) c->sr |= SH_T;
	}
	NEXT();
L_SUBV: {
		int32_t dest = (RN >> 31) & 1;
		int32_t src = ((RM >> 31) & 1) + dest;
		RN -= RM;
		int32_t ans = ((RN >> 31) & 1) + dest;
		SETT_IF(src == 1 && ans == 1);
	}
	NEXT();
L_ADD:      RN += RM; NEXT();
L_DMULS: {
		int64_t p = (int64_t)(int32_t)RN * (int32_t)RM;
		c->mach = (uint32_t)((uint64_t)p >> 32);
		c->macl = (uint32_t)p;
	}
	NEXT();
L_ADDC: {
		uint32_t tmp1 = RN + RM;
		uint32_t tmp0 = RN;
		RN = tmp1 + (c->sr & SH_T);
		SETT_IF(tmp0 > tmp1);
		if (tmp1 > RN) c->sr |= SH_T;
	}
	NEXT();
L_ADDV: {
		int32_t dest = (RN >> 31) & 1;
		int32_t src = ((RM >> 31) & 1) + dest;
		RN += RM;
		int32_t ans = ((RN >> 31) & 1) + dest;
		if (src != 1) SETT_IF(ans == 1); else c->sr &= ~SH_T;
	}
	NEXT();

	// ---- 4xxx
L_SHLL:     c->sr = (c->sr & ~SH_T) | ((RN >> 31) & SH_T); RN <<= 1; NEXT();
L_SHLR:     c->sr = (c->sr & ~SH_T) | (RN & SH_T); RN >>= 1; NEXT();
L_SHAR:     c->sr = (c->sr & ~SH_T) | (RN & SH_T); RN = (uint32_t)((int32_t)RN >> 1); NEXT();
L_STSM_MACH: RN -= 4; WR32(RN, c->mach); NEXT();
L_STSM_MACL: RN -= 4; WR32(RN, c->macl); NEXT();
L_STSM_PR:  RN -= 4; WR32(RN, c->pr); NEXT();
L_STCM_SR:  RN -= 4; WR32(RN, c->sr); NEXT();
L_STCM_GBR: RN -= 4; WR32(RN, c->gbr); NEXT();
L_STCM_VBR: RN -= 4; WR32(RN, c->vbr); NEXT();
L_ROTL:     c->sr = (c->sr & ~SH_T) | ((RN >> 31) & SH_T); RN = (RN << 1) | (RN >> 31); NEXT();
L_ROTR:     c->sr = (c->sr & ~SH_T) | (RN & SH_T); RN = (RN >> 1) | (RN << 31); NEXT();
L_ROTCL: {
		uint32_t t = (RN >> 31) & SH_T;
		RN = (RN << 1) | (c->sr & SH_T);
		c->sr = (c->sr & ~SH_T) | t;
	}
	NEXT();
L_ROTCR: {
		uint32_t t = (c->sr & SH_T) << 31;
		SETT_IF(RN & SH_T);
		RN = (RN >> 1) | t;
	}
	NEXT();
L_LDSM_MACH: { uint32_t v; RD32(v, RN); c->mach = v; RN += 4; } NEXT();
L_LDSM_MACL: { uint32_t v; RD32(v, RN); c->macl = v; RN += 4; } NEXT();
L_LDSM_PR:  { uint32_t v; RD32(v, RN); c->pr = v; RN += 4; } NEXT();
L_LDCM_SR:  c->drc_pc = pc; CALL(op_ldcmsr(c, N)); NEXT_T();
L_LDCM_GBR: { uint32_t v; RD32(v, RN); c->gbr = v; RN += 4; } NEXT();
L_LDCM_VBR: { uint32_t v; RD32(v, RN); c->vbr = v; RN += 4; } NEXT();
L_SHLL2:    RN <<= 2; NEXT();
L_SHLL8:    RN <<= 8; NEXT();
L_SHLL16:   RN <<= 16; NEXT();
L_SHLR2:    RN >>= 2; NEXT();
L_SHLR8:    RN >>= 8; NEXT();
L_SHLR16:   RN >>= 16; NEXT();
L_LDS_MACH: c->mach = RN; NEXT();
L_LDS_MACL: c->macl = RN; NEXT();
L_LDS_PR:   c->pr = RN; NEXT();
L_JSR:      c->pr = pc + 2; delay = RN; NEXT_BR();
L_JMP:      delay = RN; NEXT_BR();
L_TAS: {
		uint32_t ea = RN, t;
		RD8(t, ea);
		SETT_IF(t == 0);
		WR8(ea, t | 0x80);
	}
	NEXT();
L_LDC_SR:   c->drc_pc = pc; CALL(op_ldcsr(c, RN)); NEXT_T();
L_LDC_GBR:  c->gbr = RN; NEXT();
L_LDC_VBR:  c->vbr = RN; NEXT();
L_MACW:     CALL(op_mac_w(c, M, N)); NEXT();
L_DT:       RN--; SETT_IF(RN == 0); NEXT();
L_CMPPZ:    SETT_IF((int32_t)RN >= 0); NEXT();
L_CMPPL:    SETT_IF((int32_t)RN > 0); NEXT();
L_SHAD: {                                                                          // SHAD Rm,Rn
		uint32_t m = c->r[M];
		if (!(m & 0x80000000)) RN = RN << (m & 0x1f);
		else if (!(m & 0x1f)) RN = (RN & 0x80000000) ? 0xffffffff : 0;
		else RN = (uint32_t)((int32_t)RN >> ((~m & 0x1f) + 1));
	}
	NEXT();
L_SHLD: {                                                                          // SHLD Rm,Rn
		uint32_t m = c->r[M];
		if (!(m & 0x80000000)) RN = RN << (m & 0x1f);
		else if (!(m & 0x1f)) RN = 0;
		else RN = RN >> ((~m & 0x1f) + 1);
	}
	NEXT();
L_LDC_RBANK: c->rbnk[(c->sr & SH_RB) ? 0 : 1][M & 7] = RN; NEXT();                 // LDC Rn,Rm_BANK
L_STCM_RBANK: { uint32_t d = c->rbnk[(c->sr & SH_RB) ? 0 : 1][M & 7]; RN -= 4; WR32N(RN, d); } NEXT();
L_LDCM_RBANK: { uint32_t v, m = M; RD32N(v, RN); c->rbnk[(c->sr & SH_RB) ? 0 : 1][m & 7] = v; RN += 4; } NEXT();
L_STCM_SGR: RN -= 4; WR32N(RN, c->sgr); NEXT();
L_STCM_SSR: RN -= 4; WR32N(RN, c->ssr); NEXT();
L_LDCM_SSR: { uint32_t v; RD32N(v, RN); c->ssr = v; RN += 4; } NEXT();
L_LDC_SSR:  c->ssr = RN; NEXT();
L_STCM_SPC: RN -= 4; WR32N(RN, c->spc); NEXT();
L_LDCM_SPC: { uint32_t v; RD32N(v, RN); c->spc = v; RN += 4; } NEXT();
L_LDC_SPC:  c->spc = RN; NEXT();
L_STSM_FPUL: RN -= 4; WR32N(RN, c->fpul); NEXT();
L_LDSM_FPUL: { uint32_t v; RD32N(v, RN); c->fpul = v; RN += 4; } NEXT();
L_LDS_FPUL: c->fpul = RN; NEXT();
L_STSM_FPSCR: RN -= 4; WR32N(RN, c->fpscr & 0x003fffff); NEXT();
L_LDSM_FPSCR: { uint32_t v; RD32N(v, RN); c->fpscr = v & 0x003fffff; RN += 4; } NEXT();
L_LDS_FPSCR: c->fpscr = RN & 0x003fffff; NEXT();
L_STCM_DBR: RN -= 4; WR32N(RN, c->dbr); NEXT();
L_LDCM_DBR: { uint32_t v; RD32N(v, RN); c->dbr = v; RN += 4; } NEXT();
L_LDC_DBR:  c->dbr = RN; NEXT();

	// ---- 5xxx
L_MOVL_L4:  { uint32_t v; RD32(v, RM + (op & 15) * 4); RN = v; } NEXT();

	// ---- 6xxx
L_MOVB_L:   { uint32_t v; RD8(v, RM); RN = SEXT8(v); } NEXT();
L_MOVW_L:   { uint32_t v; RD16(v, RM); RN = SEXT16(v); } NEXT();
L_MOVL_L:   { uint32_t v; RD32(v, RM); RN = v; } NEXT();
L_MOV:      RN = RM; NEXT();
L_MOVB_P:   { uint32_t m = M, n = N, v; RD8(v, c->r[m]); c->r[n] = SEXT8(v); if (n != m) c->r[m] += 1; } NEXT();
L_MOVW_P:   { uint32_t m = M, n = N, v; RD16(v, c->r[m]); c->r[n] = SEXT16(v); if (n != m) c->r[m] += 2; } NEXT();
L_MOVL_P:   { uint32_t m = M, n = N, v; RD32(v, c->r[m]); c->r[n] = v; if (n != m) c->r[m] += 4; } NEXT();
L_NOT:      RN = ~RM; NEXT();
L_SWAPB:    { uint32_t v = RM; RN = (v & 0xffff0000) | ((v & 0xff) << 8) | ((v >> 8) & 0xff); } NEXT();
L_SWAPW:    { uint32_t v = RM; RN = (v << 16) | (v >> 16); } NEXT();
L_NEGC: {
		uint32_t t = RM;
		RN = 0u - t - (c->sr & SH_T);
		SETT_IF(t || (c->sr & SH_T));
	}
	NEXT();
L_NEG:      RN = 0u - RM; NEXT();
L_EXTUB:    RN = RM & 0xff; NEXT();
L_EXTUW:    RN = RM & 0xffff; NEXT();
L_EXTSB:    RN = SEXT8(RM); NEXT();
L_EXTSW:    RN = SEXT16(RM); NEXT();

	// ---- 7xxx
L_ADDI:     RN += SEXT8(op); NEXT();

	// ---- 8xxx
L_MOVB_S4R0: WR8(RM + (op & 15), c->r[0]); NEXT();
L_MOVW_S4R0: WR16(RM + (op & 15) * 2, c->r[0]); NEXT();
L_MOVB_L4R0: { uint32_t v; RD8(v, RM + (op & 15)); c->r[0] = SEXT8(v); } NEXT();
L_MOVW_L4R0: { uint32_t v; RD16(v, RM + (op & 15) * 2); c->r[0] = SEXT16(v); } NEXT();
L_CMPIM:    SETT_IF(c->r[0] == SEXT8(op)); NEXT();
L_BT:       if (c->sr & SH_T) { pc = pc + SEXT8(op) * 2 + 2; NEXT_TAKEN(); } NEXT_NOT_TAKEN();
L_BF:       if (!(c->sr & SH_T)) { pc = pc + SEXT8(op) * 2 + 2; NEXT_TAKEN(); } NEXT_NOT_TAKEN();
L_BTS:      if (c->sr & SH_T) { delay = pc + SEXT8(op) * 2 + 2; NEXT_BR(); } NEXT_NOT_TAKEN();
L_BFS:      if (!(c->sr & SH_T)) { delay = pc + SEXT8(op) * 2 + 2; NEXT_BR(); } NEXT_NOT_TAKEN();

	// ---- 9xxx
L_MOVW_I:   { uint32_t v; RD16(v, pc + (op & 0xff) * 2 + 2); RN = SEXT16(v); } NEXT();

	// ---- Axxx / Bxxx
L_BRA:      delay = pc + SEXT12(op & 0xfff) * 2 + 2; NEXT_BR();
L_BSR:      c->pr = pc + 2; delay = pc + SEXT12(op & 0xfff) * 2 + 2; NEXT_BR();

	// ---- Cxxx
L_MOVB_SG:  WR8(c->gbr + (op & 0xff), c->r[0]); NEXT();
L_MOVW_SG:  WR16(c->gbr + (op & 0xff) * 2, c->r[0]); NEXT();
L_MOVL_SG:  WR32(c->gbr + (op & 0xff) * 4, c->r[0]); NEXT();
L_TRAPA:    CALL(op_trapa(c, op & 0xff)); NEXT_T();
L_MOVB_LG:  { uint32_t v; RD8(v, c->gbr + (op & 0xff)); c->r[0] = SEXT8(v); } NEXT();
L_MOVW_LG:  { uint32_t v; RD16(v, c->gbr + (op & 0xff) * 2); c->r[0] = SEXT16(v); } NEXT();
L_MOVL_LG:  { uint32_t v; RD32(v, c->gbr + (op & 0xff) * 4); c->r[0] = v; } NEXT();
L_MOVA:     c->r[0] = ((pc + 2) & ~3u) + (op & 0xff) * 4; NEXT();
L_TSTI:     SETT_IF(((op & 0xff) & c->r[0]) == 0); NEXT();
L_ANDI:     c->r[0] &= op & 0xff; NEXT();
L_XORI:     c->r[0] ^= op & 0xff; NEXT();
L_ORI:      c->r[0] |= op & 0xff; NEXT();
L_TSTB:     { uint32_t v; RD8(v, c->gbr + c->r[0]); SETT_IF(((op & 0xff) & v) == 0); } NEXT();
L_ANDB:     { uint32_t ea = c->gbr + c->r[0], v; RD8(v, ea); WR8(ea, (op & 0xff) & v); } NEXT();
L_XORB:     { uint32_t ea = c->gbr + c->r[0], v; RD8(v, ea); WR8(ea, v ^ (op & 0xff)); } NEXT();
L_ORB:      { uint32_t ea = c->gbr + c->r[0], v; RD8(v, ea); WR8(ea, v | (op & 0xff)); } NEXT();

	// ---- Dxxx / Exxx
L_MOVL_I:   { uint32_t v; RD32(v, ((pc + 2) & ~3u) + (op & 0xff) * 4); RN = v; } NEXT();
L_MOVI:     RN = SEXT8(op); NEXT();
}

#include "sh3_jit.inc"

/***************************************************************************
    execute
***************************************************************************/

static void run_events(sh3_t *c)
{
	for (;;) {
		int64_t t = c->total_cycles;
		int w = 0;
		int64_t e = c->tmu_event[0];
		if (c->tmu_event[1] < e) { e = c->tmu_event[1]; w = 1; }
		if (c->tmu_event[2] < e) { e = c->tmu_event[2]; w = 2; }
		for (int i = 0; i < 4; i++)
			if (c->dma_event[i] < e) { e = c->dma_event[i]; w = 3 + i; }
		if (e > t) break;
		const unsigned pa = rbx_prof_enter(SH3_PA_EVENTS);
		if (w < 3) timer_callback(c, w);
		else dmac_callback(c, w - 3);
		rbx_prof_leave(pa);
	}
}

int sh3_execute(sh3_t *c, int cycles)
{
	int64_t start = c->total_cycles;
	if (cycles > 0)
		c->target += cycles;
	c->abort_slice = 0;
	const unsigned pa = rbx_prof_enter(SH3_PA_EXEC);
#ifdef SH3_JIT
	const int jit = sj_enabled(c);
	const uint64_t t0 = s_sj_stat ? sj_now_us() : 0;
#endif

	while (c->total_cycles < c->target && !c->abort_slice) {
		int64_t stop = next_event(c);
		if (stop > c->target) stop = c->target;
		if (stop <= c->total_cycles) {
			run_events(c);
			stop = next_event(c);
			if (stop > c->target) stop = c->target;
			if (stop <= c->total_cycles) stop = c->total_cycles + 1;
		}
		if (stop - c->total_cycles > 0x40000000) stop = c->total_cycles + 0x40000000;

		if (c->resume_at > c->total_cycles) {   // suspended (spin_until_time): the clock moves on
			c->total_cycles = c->resume_at < stop ? c->resume_at : stop;
		} else {
			c->slice_end = stop;
			c->icount = (int32_t)(stop - c->total_cycles);
			c->drc_pc = c->delay ? c->delay : c->pc;   // (MAME: the DRC's exit stores the next pc)
			c->running = 1;
#ifdef SH3_JIT
			if (jit)
				sj_run(c);
			else
#endif
			{
				rbx_prof_set(SH3_PA_INTERP);
				run(c, 0);
			}
			rbx_prof_set(SH3_PA_EXEC);
			if (c->idle.n)   // (a polling loop is found again in each slice)
				idle_stop(c, 0);
			c->running = 0;
			c->total_cycles = c->slice_end - c->icount;
		}
		run_events(c);
	}
#ifdef SH3_JIT
	if (s_sj_stat)   // (RBX_SH3_JITSTAT=1: the time in sh3_execute, with or without the JIT)
		s_sj_us += sj_now_us() - t0;
#endif
	rbx_prof_leave(pa);
	return (int)(c->total_cycles - start);
}

void sh3_end_timeslice(sh3_t *c)
{
	if (c->running) {
		c->abort_slice = 1;
		c->slice_end -= c->icount;
		c->icount = 0;
	} else
		c->abort_slice = 1;
}

void sh3_spin_until(sh3_t *c, int64_t t)
{
	c->resume_at = t;
	if (c->running) {   // (the slice ends after the current instruction; the clock resumes at t)
		c->slice_end -= c->icount;
		c->icount = 0;
	}
}

/* The spinning polling loop (the CV1000's idle loop under MAME's speed-up): a read spins the CPU for period clocks,
   the slice ends after its instruction and the next one starts at the clock it resumes at, the loop's instructions
   up to the next read count no cycles off before it (the driver: a read in a delay slot, the loop's other
   instructions without memory accesses), so in a steady loop the reads are exactly period clocks apart. A read
   period clocks after the last one, at the same pc, the same value read, nothing through slow_r / slow_w between
   (no interrupt handler, no other device), the cache array unchanged (cache_gen: the round writes nothing, the
   JIT's inline cache paths change only dirty bits on writes) and the same state otherwise (the registers, the
   interrupt and timer / DMA state, the timing model; not the clock) makes the round a fixed point: the next ones
   would do the same, each a period later. The reads up to the next timer / DMA end or sh3_execute's target (no
   event between them) are then left out: the CPU resumes where the last of them would have left it. */
// (ADR-165: exc_pri left out as for the idle rounds — the round has no slow access but its read, reset / load clear
// spin.n; [1] = RBX_SH3_SPINOLD, as before)
static const sh3_part_t s_spin_part[2][4] = {
	{ { offsetof(sh3_t, r), offsetof(sh3_t, icount) }, { offsetof(sh3_t, sleep_mode), offsetof(sh3_t, exc_pri) },
	  { offsetof(sh3_t, tstr), offsetof(sh3_t, cache) }, { offsetof(sh3_t, wb_address), offsetof(sh3_t, upper) } },
	{ { offsetof(sh3_t, r), offsetof(sh3_t, icount) }, { offsetof(sh3_t, sleep_mode), offsetof(sh3_t, total_cycles) },
	  { offsetof(sh3_t, tstr), offsetof(sh3_t, cache) }, { offsetof(sh3_t, wb_address), offsetof(sh3_t, upper) } },
};
typedef char sh3_spin_snap_fits[(offsetof(sh3_t, icount) - offsetof(sh3_t, r)) +
                                (offsetof(sh3_t, total_cycles) - offsetof(sh3_t, sleep_mode)) +
                                (offsetof(sh3_t, cache) - offsetof(sh3_t, tstr)) +
                                (offsetof(sh3_t, upper) - offsetof(sh3_t, wb_address)) <= sizeof(((sh3_t *)0)->spin.snap) ? 1 : -1];

static int spin_snap(sh3_t *c, int cmp)
{
	if (cmp)
		return parts_eq(c, s_spin_part[s_idle_bytes], 4, c->spin.snap);
	parts_put(c, s_spin_part[s_idle_bytes], 4, c->spin.snap);
	return 1;
}

void sh3_spin_round(sh3_t *c, uint32_t period, uint32_t v)
{
	const unsigned pa = rbx_prof_enter(SH3_PA_SPIN);
	struct sh3_spinr *s = &c->spin;
	s->ncall++;
	const int64_t now = now_cycles(c);
	int64_t resume = now + period;
	if (c->running && !c->trace && !c->abort_slice && period) {
		if (s->n && c->pc == s->pc && v == s->v && now == s->t + period && c->nslow == s->ns + 1 && c->cache_gen == s->cg &&
		    spin_snap(c, 1)) {
			int64_t stop = next_event(c);
			if (c->target < stop)
				stop = c->target;
			const int64_t m = stop > now ? (stop - 1 - now) / period : 0;   // (the reads left out: before stop)
			if (m > 0) {
				resume += m * period;
				s->skipped += m * period;
				s->nskip++;
			}
			s->t = resume - period;
		} else {
			s->nfirst++;
			spin_snap(c, 0);
			s->pc = c->pc;
			s->v = v;
			s->t = now;
			s->cg = c->cache_gen;
			s->n = 1;
		}
		s->ns = c->nslow;
	} else
		s->n = 0;
	sh3_spin_until(c, resume);
	rbx_prof_leave(pa);
}

/***************************************************************************
    init / reset / state
***************************************************************************/

static uint8_t dummy_r8(void *u, uint32_t a) { (void)u; (void)a; return 0; }
static uint16_t dummy_r16(void *u, uint32_t a) { (void)u; (void)a; return 0; }
static uint32_t dummy_r32(void *u, uint32_t a) { (void)u; (void)a; return 0; }
static void dummy_w8(void *u, uint32_t a, uint8_t d) { (void)u; (void)a; (void)d; }
static void dummy_w16(void *u, uint32_t a, uint16_t d) { (void)u; (void)a; (void)d; }
static void dummy_w32(void *u, uint32_t a, uint32_t d) { (void)u; (void)a; (void)d; }

// (the cheats' access to this CPU's program space: cheat_cpu.h)
// (as the CPU: the physical address's page, byte lane ^ 3, else the callbacks)
static uint8_t cheat_rd(void *x, uint32_t a)
{
	sh3_t *c = (sh3_t *)x;
	const uint32_t pa = a & 0x1fffffff;
	const uint8_t *p = c->rd[pa >> SH3_PAGE_SHIFT];
	return p ? p[(pa & 0xfff) ^ 3] : c->read8(c->user, a);
}
static void cheat_wr(void *x, uint32_t a, uint8_t v)
{
	sh3_t *c = (sh3_t *)x;
	const uint32_t pa = a & 0x1fffffff;
	uint8_t *p = c->wr[pa >> SH3_PAGE_SHIFT];
	if (p)
		p[(pa & 0xfff) ^ 3] = v;
	else
		c->write8(c->user, a, v);
}

void sh3_init(sh3_t *c)
{
	rbx_cheat_offer("sh3", cheat_rd, cheat_wr, c, 1, 1);
	memset(c, 0, sizeof(*c));
	rbx_prof_name(RBX_PA_CPU, s_sh3_pa_names, SH3_PA_END - RBX_PA_CPU);
	s_idle_bytes = getenv("RBX_SH3_SPINOLD") && atoi(getenv("RBX_SH3_SPINOLD"));
	s_idle_pure = !(getenv("RBX_SH3_IDLEPURE") && !atoi(getenv("RBX_SH3_IDLEPURE")));
#ifdef SH3_JIT
	sj_forget();   // (a CPU before this one is gone: nothing of it touched)
#endif
	c->read8 = dummy_r8; c->read16 = dummy_r16; c->read32 = dummy_r32;
	c->write8 = dummy_w8; c->write16 = dummy_w16; c->write32 = dummy_w32;
	c->fetch_page = ~0u;
	c->tmu_event[0] = c->tmu_event[1] = c->tmu_event[2] = SH3_NEVER;
	for (int i = 0; i < 4; i++)
		c->dma_event[i] = SH3_NEVER;
	c->exact = 1;
	c->flush_pc = 1;
	// (sh3_base_device::device_start: the BSC)
	c->bcr2 = 0x3ff0;
	c->wcr1 = 0x3ff3;
	c->wcr2 = 0xffff;
	c->mcr = 0;
}

void sh3_release(void)
{
#ifdef SH3_JIT
	sj_release();
#endif
}

void sh3_reset(sh3_t *c)
{
#ifdef SH3_JIT
	sj_flush(c);
#endif
	idle_stop(c, 0);
	c->idle.fails = c->idle.wait = 0;
	c->idle.miss = 0;
	c->spin.n = 0;
	c->cache_gen++;
	c->running = 0;
	c->abort_slice = 0;
	c->fetch_page = ~0u;
	c->fetch_base = NULL;
	c->total_cycles = c->target = 0;
	c->resume_at = 0;

	// sh34_base_device::device_reset
	c->spc = c->pr = c->sr = c->ssr = c->gbr = c->vbr = c->mach = c->macl = 0;
	memset(c->r, 0, sizeof(c->r));
	memset(c->rbnk, 0, sizeof(c->rbnk));
	c->sgr = 0;
	c->delay = 0;
	c->pending_irq = 0;
	c->test_irq = 0;
	memset(c->exc_pri, 0, sizeof(c->exc_pri));
	memset(c->exc_req, 0, sizeof(c->exc_req));
	memset(c->upper, 0, sizeof(c->upper));
	memset(c->lower, 0, sizeof(c->lower));
	memset(c->line_state, 0, sizeof(c->line_state));
	c->tstr = c->tocr = c->tcpr2 = 0;
	for (int i = 0; i < 3; i++) {
		c->tcnt[i] = c->tcor[i] = c->tcr[i] = 0;
		c->tmu_event[i] = SH3_NEVER;
	}
	c->ipra = c->iprc = 0;
	for (int i = 0; i < 4; i++) {
		c->sar[i] = c->dar[i] = c->dmatcr[i] = c->chcr[i] = 0;
		c->dma_active[i] = 0;
		c->dma_event[i] = SH3_NEVER;
	}
	c->dmaor = 0;
	// sh7709s_device::device_reset
	memset(c->cache, 0, sizeof(c->cache));
	c->wb_address = 0;
	c->last_area = 0;
	c->wb_active_cycles = 0;
	c->last_sdram_page = 0;
	c->precharge_remaining = 0;
	c->arg0 = 0;
	c->ccr = 0;
	default_exception_priorities(c);
	s_dma_old = getenv("RBX_SH3_OLDDMA") && atoi(getenv("RBX_SH3_OLDDMA"));
	s_dma_stat = getenv("RBX_SH3_DMASTAT") && atoi(getenv("RBX_SH3_DMASTAT"));
	c->pc = 0xa0000000;
	c->r[15] = RL(c, 4);
	c->sr = 0x700000f0;
	c->fpscr = 0x00040001;
	c->fpul = 0;
	c->dbr = 0;
	c->sleep_mode = 0;
	// sh3_base_device::device_reset
	for (int i = 0; i < 3; i++)
		c->tcor[i] = c->tcnt[i] = 0xffffffff;
}

void sh3_post_load(sh3_t *c)
{
#ifdef SH3_JIT
	sj_flush(c);   // (the memory was loaded under the blocks)
#endif
	idle_stop(c, 0);
	c->idle.fails = c->idle.wait = 0;
	c->idle.miss = 0;
	c->spin.n = 0;
	c->cache_gen++;
	c->running = 0;
	c->abort_slice = 0;
	c->fetch_page = ~0u;
	c->fetch_base = NULL;
}
