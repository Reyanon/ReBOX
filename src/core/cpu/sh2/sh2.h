/*  Hitachi SH-2 (SH7604) CPU with its on-chip peripherals, ported to C from MAME 0.289
    src/devices/cpu/sh: sh.cpp, sh.h (BSD-3-Clause, copyright-holders: David Haywood), sh2.cpp, sh2.h,
    sh7604.cpp, sh7604.h (BSD-3-Clause, copyright-holders: Juergen Buchmueller, R. Belmont).
    The interpreter of sh_common_execution / sh2_device (MAME's non-DRC path) with MAME's cycle counts (one cycle per
    instruction plus the extra icount deductions of each handler, quirks included: OR #imm,R0 costs 3, JMP 1) and
    MAME's delay slots (the slot instruction is fetched at the branch's PC + 2, then PC becomes the target before the
    slot runs, and no interrupt is taken between a branch and its slot). The SH7604 on-chip modules are the ones
    sh7604_device maps at 0xfffffe00-0xffffffff: SCI (registers latched only, as in MAME), FRT (free-running timer
    with OCRA / OCRB compare, overflow, input capture), INTC (IPRA / IPRB levels, VCRx vectors, ICR VECMD / NMIE),
    DIVU (32/32 and 64/32 signed division, overflow flag), WDT (interval timer mode; watchdog mode only sets WOVF,
    as in MAME), DMAC (two channels, auto request, byte / word / long / 16-byte units, end interrupt, cycle steal),
    UBC (BARA / BARB latched), BSC and cache control registers (latched). 0x40000000-0xbfffffff reads 0xa5a5a5a5.
    Differences from MAME (see sh2.c for details): sh7604_device::sh2_recalc_irq sets a shadowed m_test_irq that the
    interpreter never reads (MAME's interpreter would only notice an internal interrupt at the next LDC SR / RTE /
    external line change; the DRC notices it at the next timeslice); here it is checked after the current
    instruction. The emu_timers (FRT, WDT, DMA) are cycle stamps on the CPU's own clock (MAME schedules them from the
    start of the current timeslice); DMA units are moved in batches at slice boundaries (MAME: one unit every 2
    cycles), the end of a transfer (TE, the interrupt) comes at MAME's cycle. The DMA FIFO / kludge / FTCSR read
    callbacks (Saturn, 32X) and the DRC are not ported. Unaligned word / long accesses are aligned down.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_SH2_H
#define RBX_SH2_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// input lines: 0-15 = the IRQ levels (external interrupt at level n, autovector 64 + n/2 unless ICR VECMD is set),
// SH2_INPUT_NMI = the NMI (edge triggered, vector 11)
#define SH2_INPUT_NMI 16
// line states (MAME CLEAR_LINE / ASSERT_LINE / HOLD_LINE: a held line is cleared when the CPU takes it)
enum { SH2_CLEAR = 0, SH2_ASSERT = 1, SH2_HOLD = 2 };

// memory map: guest addresses 0x00000000-0x07ffffff (after MAME's address mask 0xc7ffffff, so 0x2xxxxxxx and the
// 0x08000000 mirrors land here) in 4 KB pages
#define SH2_PAGE_SHIFT 12
#define SH2_PAGES      (1u << (27 - SH2_PAGE_SHIFT))   // 32768

#define SH2_NEVER INT64_MAX

typedef struct sh2_s sh2_t;
struct sh2_s {
	/* ---- saved state (POD, from r up to `running`) ---- */
	uint32_t r[16];
	uint32_t pc, pr, sr, gbr, vbr, mach, macl;
	uint32_t delay;                 // MAME m_delay: target of a delayed branch whose slot is next (0 = none)
	uint32_t pending_irq;           // bit n = external line n asserted
	uint32_t pending_nmi;
	int32_t internal_irq_level;     // on-chip interrupt level (-1 = none)
	int32_t internal_irq_vector;
	uint32_t test_irq;              // check interrupts after the current instruction
	int32_t icount;                 // MAME m_icount (valid inside sh2_execute)
	uint8_t sleep_mode;             // 0 normal, 1 in SLEEP, 2 left by an exception
	uint8_t nmi_line_state;
	uint8_t irq_line_state[16];     // SH2_CLEAR / SH2_ASSERT / SH2_HOLD
	uint8_t halted;                 // cycle-steal DMA in progress (MAME suspend SUSPEND_REASON_HALT)
	uint8_t is_slave;               // BCR1 bit 15 (MAME set_is_slave)
	int64_t total_cycles;           // clock of the CPU (cycles run since sh2_reset)
	int64_t target;                 // the clock sh2_execute runs up to (overshoot carried to the next call)

	// SCI
	uint8_t smr, brr, scr, tdr, ssr;
	// FRT
	uint8_t tier, ftcsr, frc_tcr, tocr;
	uint8_t frt_input;
	uint16_t frc, ocra, ocrb, frc_icr;
	int64_t frc_base;               // MAME m_frc_base (CPU clock of the last resync)
	int64_t frt_event;              // MAME m_timer expiry (SH2_NEVER = off)
	// INTC
	uint8_t lvl_frc, lvl_sci, lvl_divu, lvl_dmac, lvl_wdt;
	uint8_t vec_fic, vec_foc, vec_fov, vec_divu, vec_dmac[2];
	uint8_t vecmd, nmie;
	uint16_t ipra, iprb, vcra, vcrb, vcrc, vcrd, vcrwdt, intc_icr;
	uint32_t vcrdiv, vcrdma[2];
	// DIVU
	uint8_t divu_ovf, divu_ovfie;
	uint32_t dvsr, dvdntl, dvdnth;
	// WDT
	uint8_t wtcnt, wtcsr, rstcsr;
	uint16_t wtcw[2];
	int64_t wdt_event;
	// UBC
	uint16_t barah, baral, barbh, barbl;
	// DMAC
	struct { uint32_t sar, dar, tcr, chcr; uint8_t drcr; } dmac[2];
	uint8_t dmaor;
	uint8_t dma_active[2];          // MAME m_dma_timer_active (1 = running)
	uint8_t dma_irq[2];
	uint8_t dma_incs[2], dma_incd[2], dma_size[2], dma_steal[2];
	uint32_t dma_src[2], dma_dst[2], dma_count[2];
	int64_t dma_tick[2];            // clock of the channel's next 2-cycle step (SH2_NEVER = idle)
	// misc, BSC
	uint8_t sbycr, ccr;
	uint32_t bcr1, bcr2, wcr, mcr, rtcsr, rtcor, rtcnt;

	/* ---- not saved (run state, bus) ---- */
	int running;
	int64_t slice_end;              // clock at which the running slice ends (now = slice_end - icount)
	int abort_slice;
	uint32_t onchip_n;               // on-chip register accesses counted (a driver's idle-round recorder sees them)
	// a driver's trace of the instructions fetched (their addresses, before each runs); NULL = none (the usual)
	void (*fetch_trace)(void *user, uint32_t pc);
	// the instruction fetch page (the unmasked pc >> 12 whose host page is fetch_base, ~0 = none)
	uint32_t fetch_page;
	const uint8_t *fetch_base;
	// accesses that miss the page table. Addresses are MAME's: masked with 0xc7ffffff when bits 31-28 are 0 or 2,
	// word / long aligned. The on-chip area (0x40000000-0xbfffffff, 0xfffffe00-0xffffffff) never reaches them.
	uint8_t (*read8)(void *user, uint32_t addr);
	uint16_t (*read16)(void *user, uint32_t addr);
	uint32_t (*read32)(void *user, uint32_t addr);
	void (*write8)(void *user, uint32_t addr, uint8_t data);
	void (*write16)(void *user, uint32_t addr, uint16_t data);
	void (*write32)(void *user, uint32_t addr, uint32_t data);
	void *user;
	// a fetch-only area above 0x40000000 (MAME's AS_OPCODES for CPS3's cache RAM at 0xc0000000: the opcodes decrypted
	// while data reads see the plain RAM through the callbacks): op_hi_size bytes at op_hi_base, host-order 32-bit
	// words like the page table; NULL = none
	const uint8_t *op_hi;
	uint32_t op_hi_base, op_hi_size;
	// a second one anywhere (4 KB aligned base and size, host-order words): the instruction fetches of
	// [op_lo_base, op_lo_base + op_lo_size) read it before the page table (CPS3's decrypted flash, data reads seeing the
	// plain one); NULL = none
	const uint8_t *op_lo;
	uint32_t op_lo_base, op_lo_size;
	// MAME's SH2 DMA kludge callback (CPS3: the DMA bypasses the encryption): a DMA unit's data between its read and
	// its write; size = MAME's m_active_dma_size (0 byte, 1 word, 2 long, 3 16-byte: src is the first long's for all
	// four, as in MAME); NULL = none
	uint32_t (*dma_kludge)(void *user, uint32_t src, uint32_t dst, uint32_t data, int size);
	// MAME standard_irq_callback: called when an external interrupt (line 0-15) is taken, after a held line has
	// been cleared; returns the vector used when ICR VECMD is set. NULL = vector 0
	int (*irq_ack)(void *user, int line);
	// the driver's idle loops (a speed-up, MAME's eat_cycles in effect): a BT / BF taken to an idle_pc (1 = none, set
	// by sh2_init) calls idle with c->pc = that idle_pc and c->icount the cycles left in the slice (the branch's base cycle not
	// yet counted). It may skip whole rounds of the loop: their effects on registers and memory, c->icount lowered by
	// their cycles (left above one round). No interrupt can come inside a slice except from the driver's handlers
	// A BRA to an idle_pc calls it too, before its slot: c->delay = the target, c->pc = the slot's address.
	uint32_t idle_pc[4];
	void (*idle)(sh2_t *c);
	void *idle_user;
	// page table: host memory of each 4 KB page (NULL = callbacks). Layout: host-order 32-bit words (MAME's layout
	// of a big-endian 32-bit space on a little-endian host): the guest byte at a is at base[(a & 0xfff) ^ 3], the
	// word at base + ((a & 0xffe) ^ 2), the long at base + (a & 0xffc)
	uint8_t *rd[SH2_PAGES];
	uint8_t *wr[SH2_PAGES];
};

#define SH2_STATE_SIZE ((uint32_t)offsetof(sh2_t, running))

// clear everything (registers, callbacks, map). Call once, then set the callbacks, map memory and sh2_reset.
void sh2_init(sh2_t *c);
// MAME device_reset (sh2_device + sh7604_device): registers cleared, SR = 0xf0, PC = long at 0, R15 = long at 4,
// lines cleared. The on-chip registers keep their values as in MAME (only FRC / OCR / ICR, the WDT counter, the
// UBC and the DMA run state are cleared), except at the first reset after sh2_init (all zero). Map and callbacks kept.
void sh2_reset(sh2_t *c);
// run until the CPU clock reaches target + cycles (MAME execute_run: an instruction is never split, the overshoot
// is taken off the next call). Returns the cycles run in this call.
int sh2_execute(sh2_t *c, int cycles);
// MAME execute_set_input. line 0-15 or SH2_INPUT_NMI; state SH2_CLEAR / SH2_ASSERT / SH2_HOLD. Outside sh2_execute
// an interrupt is taken at once (unless a delay slot is next); from a handler inside it, after the current instruction.
void sh2_set_irq_line(sh2_t *c, int line, int state);
// MAME set_frt_input (FRT input capture pin)
void sh2_set_frt_input(sh2_t *c, int state);
// map [start, end] (4 KB aligned, inside 0x00000000-0x07ffffff) to host memory (layout above). size = the size of
// the block at mem, repeated over the range (mirrors); 0 = the whole range. readable / writable choose the
// directions served from it (the others go to the callbacks). mem == NULL unmaps. Returns 0, or -1 for a bad range.
int sh2_map(sh2_t *c, uint32_t start, uint32_t end, void *mem, uint32_t size, int readable, int writable);
// stop sh2_execute after the current instruction (MAME abort_timeslice); the cycles not run stay owed
void sh2_end_timeslice(sh2_t *c);
// MAME eat_cycles: n clocks pass without instructions (a driver's idle-loop speedup); inside sh2_execute the slice
// ends sooner, outside it the CPU clock moves on
void sh2_eat_cycles(sh2_t *c, int n);
// MAME total_cycles (valid inside handlers too)
int64_t sh2_total_cycles(const sh2_t *c);
// swap a big-endian byte image into host-order 32-bit words in place (size multiple of 4)
void sh2_swap32(void *p, uint32_t size);

// bus accesses as the CPU makes them (address masking, page table, on-chip registers, callbacks)
uint8_t sh2_read8(sh2_t *c, uint32_t a);
uint16_t sh2_read16(sh2_t *c, uint32_t a);
uint32_t sh2_read32(sh2_t *c, uint32_t a);
void sh2_write8(sh2_t *c, uint32_t a, uint8_t v);
void sh2_write16(sh2_t *c, uint32_t a, uint16_t v);
void sh2_write32(sh2_t *c, uint32_t a, uint32_t v);

// save state: the block from r up to `running` (SH2_STATE_SIZE bytes); after loading it call sh2_post_load
void sh2_post_load(sh2_t *c);

#ifdef __cplusplus
}
#endif

#endif
