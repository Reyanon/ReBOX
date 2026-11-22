/*  Hitachi SH-3 (SH7709S, big-endian, as Cave's CV1000 uses it), ported to C from MAME 0.289 src/devices/cpu/sh:
    sh.cpp, sh.h, sh_fe.cpp (BSD-3-Clause, copyright-holders: David Haywood), sh4.cpp, sh4.h, sh4comn.cpp,
    sh4comn.h, sh3comn.cpp, sh3comn.h, sh4tmu.cpp, sh4dmac.cpp (BSD-3-Clause, copyright-holders: R. Belmont),
    sh7709s.cpp, sh7709s.h (BSD-3-Clause, copyright-holders: Jesus Ramos).
    The instructions of sh_common_execution with the sh34_base_device overrides: the register banks (SR.RB),
    SSR / SPC / SGR / DBR, SHAD / SHLD, LDC / STC Rm_BANK, CLRS / SETS, PREF / MOVCA.L / LDTLB (no-ops, as for MAME's
    SH-3), TRAPA through VBR + 0x100, the interrupts through VBR + 0x600 with MAME's SH-3 INTEVT / INTEVT2 / EXPEVT
    values, MAME's exception priorities (IRL0-3 lines, IPRA / IPRC levels). Illegal opcodes are NOPs, the FPU group
    (0xfxxx) does nothing (the SH-3 has none).
    Timing: MAME runs this CPU through its DRC, and the CV1000 screens depend on it, so the interpreter counts cycles
    the DRC's way: the frontend's cycles of each instruction (sh_fe.cpp, a branch costs the same taken or not), counted
    off at the DRC's update points (after a memory access outside a delay slot, after a taken branch and its slot);
    the cycles gathered since the last update are dropped by a conditional branch not taken and by the interrupt
    checks after LDC SR / RTE (the DRC compiles the update into the taken path only); the timeslice ends at an update
    point once icount is negative. On top, the SH7709S cache and bus penalties of sh7709s.cpp: a 16 KB 4-way cache,
    SDRAM precharge and write-back stalls, the BSC wait states (WCR1 / WCR2 / MCR / BCR2) on every data access of
    the DRC's memory accessor (the C helpers' accesses: none) and on the instruction fetch of groups 0, 4 and 15 at
    the pc the DRC last stored (drc_pc: the slice's start, an exception, RTE, LDC SR, the driver's pc flush), the CCR
    cache invalidate, the memory mapped cache address array (0xf0000000-0xf0ffffff) with its flush cost.
    Memory: addresses below 0xe0000000 are masked with 0x1fffffff (MMU off); on-chip registers at 0x04000000-
    0x07ffffff (INTC, DMAC (auto-request transfers done at once, the end flag after 2 x count + 1 clocks, as MAME),
    the I/O ports through callbacks, SCIF latched) and 0xffffd000-0xffffffff (TMU, IPRA / IPRB, BSC, CCR, TRA /
    EXPEVT / INTEVT). The TMU and DMA ends are cycle stamps on the CPU clock (peripheral clock = CPU clock / 4).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_SH3_H
#define RBX_SH3_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// input lines: 0-3 = IRL0-3 (MAME SH4_IRL0..3 on an SH-3: independent lines)
enum { SH3_CLEAR = 0, SH3_ASSERT = 1, SH3_HOLD = 2 };

#define SH3_PAGE_SHIFT 12
#define SH3_PAGES      (1u << (29 - SH3_PAGE_SHIFT))   // the 512 MB physical space in 4 KB pages
#define SH3_NEVER      INT64_MAX
#define SH3_EXC_COUNT  74                              // MAME SH4_INTC_ROVI + 1

// the I/O ports (MAME SH3_PORT_A ... as indices)
enum { SH3_PORT_A, SH3_PORT_B, SH3_PORT_C, SH3_PORT_D, SH3_PORT_E, SH3_PORT_F, SH3_PORT_G, SH3_PORT_H,
       SH3_PORT_J, SH3_PORT_K, SH3_PORT_L };

typedef struct { uint32_t tag; uint8_t lru, dirty, pad[2]; } sh3_ctag_t;   // (a cache line's tag)
typedef struct sh3_s sh3_t;
struct sh3_s {
	/* ---- saved state (POD, from r up to `running`) ---- */
	uint32_t r[16];
	uint32_t rbnk[2][8];            // MAME m_rbnk: the bank not in r[0-7] (r[] holds the current one)
	uint32_t pc, pr, sr, gbr, vbr, mach, macl;
	uint32_t ssr, spc, sgr, dbr, fpul, fpscr;
	uint32_t delay;                 // MAME m_delay
	uint32_t test_irq;
	int32_t pending_irq;            // the number of exceptions requesting (MAME m_pending_irq)
	int32_t icount;
	uint8_t sleep_mode;
	uint8_t line_state[4];          // SH3_CLEAR / SH3_ASSERT / SH3_HOLD
	uint8_t exc_req[SH3_EXC_COUNT]; // MAME m_exception_requesting
	uint32_t exc_pri[SH3_EXC_COUNT];// MAME m_exception_priority
	int64_t total_cycles;           // the CPU clock
	int64_t target;
	int64_t resume_at;              // suspended until this clock (MAME spin_until_time), SH3_NEVER-free: 0 = not
	// TMU
	uint32_t tstr, tocr, tcnt[3], tcor[3], tcr[3], tcpr2;
	int64_t tmu_event[3];
	// INTC
	uint32_t ipra, iprc;
	// DMAC (sh4dmac.cpp)
	uint32_t sar[4], dar[4], dmatcr[4], chcr[4], dmaor;
	uint32_t dma_active[4];
	int64_t dma_event[4];
	// SH7709S cache / bus timing (sh7709s.cpp: MAME's DRC charges these)
	sh3_ctag_t cache[256][4];
	uint32_t wb_address, last_area, wb_active_cycles, last_sdram_page, precharge_remaining;
	uint32_t arg0;                  // MAME m_sh2_state->arg0: the last data address (or a helper's opcode)
	uint32_t drc_pc;                // MAME m_sh2_state->pc as the DRC leaves it (the instruction fetch penalty's address)
	uint32_t bcr2, wcr1, wcr2, mcr, ccr;
	// the on-chip register files (MAME m_sh3internal_upper / lower)
	uint32_t upper[0x3000 / 4];
	uint32_t lower[0x1000];

	/* ---- not saved ---- */
	int running;
	// the timing model (ADR-076): 1 = MAME's (the cache and bus simulation above, the default); 0 = the calibrated
	// one (sh3.c tpenalty): the cache[][] tags as a 2-way 16 KB array, a miss or a not cacheable access of class
	// (write, address >> 26) costs tcost[write][class] clocks. Set before the first sh3_execute (sh3_set_timing)
	int exact;
	uint16_t tcost[2][64];
	uint32_t jpend;                 // (sh3_jit.inc: the cycles gathered since the last count-off, between the interpreter and the blocks)
	// the JIT may translate code in a page without a read entry from its write entry's memory: the driver's read
	// callback returns that memory unchanged for instruction fetches there (no side effect at a code address)
	int fetch_wr;
	// polling loops (sh3.c "idle rounds"): a board read in [ru_lo, ru_hi] (masked addresses) the driver declares a
	// function of the clock: read_until returns the clock up to which (not included) a read of a, size bytes, returns
	// what it returns now with no side effect (<= now: not such a read). NULL = off
	int64_t (*read_until)(void *user, uint32_t a, int size);
	uint32_t ru_lo, ru_hi;
	uint32_t nslow;                 // (the accesses through slow_r / slow_w)
	struct sh3_idle {
		uint32_t pc, a, v, ns;
		int size, n, rec, miss;
		int32_t ic;
		uint32_t fails, wait;
		int64_t se;
		uint32_t nwr, wrp[8][2];    // (the pages mapped writable: first, last)
		int64_t skipped;            // (statistics: cycles skipped, skips, recordings, failed recordings)
		uint32_t nskip, nrec, nfail, ncall, nsnap, nsame;
		uint32_t cg;                // (cache_gen at the snapshot, ADR-165)
		uint32_t npure;             // (statistics: recordings with the loop proven to write nothing, ADR-175)
		uint8_t snap[10240];
	} idle;
	uint32_t cache_gen;             // (raised by every change of the cache array the C code makes)
	struct sh3_spinr {              // (sh3_spin_round)
		uint32_t pc, v, ns, cg;
		int n;
		int64_t t;
		int64_t skipped;            // (statistics: cycles, skips)
		uint32_t nskip, ncall, nfirst;
		uint8_t snap[1536];
	} spin;
	int64_t slice_end;
	int abort_slice;
	uint32_t fetch_page;            // the masked pc >> 12 whose host page is fetch_base (~0 = none)
	const uint8_t *fetch_base;
	// accesses that miss the page table: a is the address as MAME's space sees it (masked below 0xe0000000),
	// aligned; the on-chip registers never reach them
	uint8_t (*read8)(void *user, uint32_t a);
	uint16_t (*read16)(void *user, uint32_t a);
	uint32_t (*read32)(void *user, uint32_t a);
	void (*write8)(void *user, uint32_t a, uint8_t d);
	void (*write16)(void *user, uint32_t a, uint16_t d);
	void (*write32)(void *user, uint32_t a, uint32_t d);
	// (optional) n byte reads at a (a board address, as read8 gets it) one after another into buf, the same as n
	// read8 calls; returns 0 when it does not do it (then read8 each). The DMA's byte transfers from a fixed source
	uint32_t (*read_block8)(void *user, uint32_t a, uint8_t *buf, uint32_t n);
	uint8_t (*port_r)(void *user, int port);
	void (*port_w)(void *user, int port, uint8_t d);
	void *user;
	// MAME sh2drc_add_pcflush: the DRC stores the pc before this instruction (1 = none)
	uint32_t flush_pc;
	// debugging: called before each instruction with its address (NULL = off)
	void (*trace)(sh3_t *c, uint32_t pc);
	// page table: host memory of each 4 KB page of the physical space (NULL = callbacks); host-order 32-bit words:
	// the guest byte at a is at base[(a & 0xfff) ^ 3], the word at base + ((a & 0xffe) ^ 2), the long at
	// base + (a & 0xffc)
	uint8_t *rd[SH3_PAGES];
	uint8_t *wr[SH3_PAGES];
	// (ADR-175) 1 while a slow read is made by a load known to be outside a delay slot (the interpreter's loads and the
	// JIT's, set around their slow_r); 0 otherwise (unknown). Not state
	int rnorm;
};

#define SH3_STATE_SIZE ((uint32_t)offsetof(sh3_t, running))

// clear everything (registers, callbacks, map). The struct is large (the page table): allocate it on the heap.
void sh3_init(sh3_t *c);
// the game is over (the CPU about to be freed): the JIT's tables given back (ADR-185; none without the JIT)
void sh3_release(void);
// MAME sh34_base_device::device_reset + sh3_base_device::device_reset: PC = 0xa0000000, R15 = long at 4,
// SR = 0x700000f0, the on-chip registers cleared, TCOR / TCNT = 0xffffffff, the lines cleared
void sh3_reset(sh3_t *c);
// run until the CPU clock reaches target + cycles (an instruction is never split). Returns the cycles run.
int sh3_execute(sh3_t *c, int cycles);
// MAME execute_set_input (lines 0-3). The interrupt is taken after the next instruction.
void sh3_set_irq_line(sh3_t *c, int line, int state);
// map [start, end] (4 KB aligned, physical, below 0x20000000) to host memory (layout above), size = the block
// repeated over the range (0 = the whole range). mem == NULL unmaps. Returns 0 or -1.
int sh3_map(sh3_t *c, uint32_t start, uint32_t end, void *mem, uint32_t size, int readable, int writable);
// stop sh3_execute after the current instruction
void sh3_end_timeslice(sh3_t *c);
// MAME spin_until_time: the CPU stops after the current instruction and runs again at clock t
void sh3_spin_until(sh3_t *c, int64_t t);
// sh3_spin_until(now + period) from a board read (value v) of a polling loop the driver has checked: the loop's round
// from this read back to the next one at the same pc writes nothing and reads nothing else through the board or the
// on-chip registers. When the previous round came back to the same state exactly period clocks later, the rounds
// up to the next timer / DMA end or the end of sh3_execute are left out with the results of running them
void sh3_spin_round(sh3_t *c, uint32_t period, uint32_t v);
int64_t sh3_total_cycles(const sh3_t *c);
// swap a big-endian byte image into host-order 32-bit words in place (size multiple of 4)
void sh3_swap32(void *p, uint32_t size);
uint32_t sh3_read32(sh3_t *c, uint32_t a);
uint16_t sh3_read16(sh3_t *c, uint32_t a);
// save state: SH3_STATE_SIZE bytes from r; then sh3_post_load
void sh3_post_load(sh3_t *c);
// the timing model: rd / wr = the 64 classes' costs (clocks, by address >> 26) or NULL = MAME's exact model (ADR-076)
void sh3_set_timing(sh3_t *c, const uint16_t *rd, const uint16_t *wr);
// (testing, RBX_SH3_TSTAT=file: the timing model's penalties by access class since the last call, one line)
void sh3_tstat_frame(sh3_t *c, int frame);

#ifdef __cplusplus
}
#endif

#endif
