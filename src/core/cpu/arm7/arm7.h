/*  ARM7TDMI CPU (ARMv4T, little endian), ported to C from MAME src/devices/cpu/arm7
    (arm7.cpp, arm7.h, arm7core.h, arm7core.hxx, arm7help.h, arm7ops.cpp, arm7thmb.cpp; MAME 0.289; BSD-3-Clause,
    copyright-holders: Steve Ellenoff, R. Belmont, Ryan Holtz).
    Scope: MAME's ARM7 device (archRev 4, ARCHFLAG_T, ENDIANNESS_LITTLE) as used by IGS PGM (IGS027A / 55857x).
    Instruction semantics, flag rules, register banking, exception entry, the R15 pipeline offsets and the icount
    deductions follow arm7_cpu_device one to one (including its quirks; see arm7.c). The MMU (translation, aborts),
    the DRC, the disassembler, the debugger hooks and the ARMv5+/ARM9/XScale-only paths are not ported.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_ARM7_H
#define RBX_ARM7_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { ARM7_IRQ_LINE = 0, ARM7_FIRQ_LINE = 1 };

// CPSR mode values (bits 3-0; bit 4 is always set)
enum {
	ARM7_MODE_USER = 0x0, ARM7_MODE_FIQ = 0x1, ARM7_MODE_IRQ = 0x2, ARM7_MODE_SVC = 0x3,
	ARM7_MODE_ABT = 0x7, ARM7_MODE_UND = 0xb, ARM7_MODE_SYS = 0xf
};

// MAME's m_r[] indices (the layout of arm7_t.bank[])
enum {
	ARM7_R8 = 8, ARM7_R13 = 13, ARM7_R14 = 14,
	ARM7_R8_FIQ = 17, ARM7_R13_FIQ = 22, ARM7_R14_FIQ = 23, ARM7_SPSR_FIQ = 24,
	ARM7_R13_IRQ = 25, ARM7_R14_IRQ = 26, ARM7_SPSR_IRQ = 27,
	ARM7_R13_SVC = 28, ARM7_R14_SVC = 29, ARM7_SPSR_SVC = 30,
	ARM7_R13_ABT = 31, ARM7_R14_ABT = 32, ARM7_SPSR_ABT = 33,
	ARM7_R13_UND = 34, ARM7_R14_UND = 35, ARM7_SPSR_UND = 36,
	ARM7_NUM_REGS = 37
};

// memory map: 1 KB pages, two levels (1 MB blocks -> a pool of second-level tables)
#define ARM7_PAGE_SHIFT 10
#define ARM7_L1_SHIFT   20
#define ARM7_L1_SIZE    (1u << (32 - ARM7_L1_SHIFT))               // 4096 blocks of 1 MB
#define ARM7_L2_SIZE    (1u << (ARM7_L1_SHIFT - ARM7_PAGE_SHIFT))  // 1024 pages of 1 KB per block
#define ARM7_L2_POOL    32                                         // 1 MB blocks that can hold mapped pages

typedef struct arm7_s arm7_t;
typedef struct arm7_idle_head arm7_idle_head;
#define ARM7_IDLE_HEADS 4
struct arm7_s {
	/* ---- saved state (POD, from r up to total_cycles) ---- */
	// r[0..15]: the registers seen in the current mode (cpsr & 0xf); r[15] is MAME's R15 (the address of the
	// executing instruction; reads of PC add the pipeline offset where MAME adds it). bank[]: MAME's m_r[] slots
	// (ARM7_R8.. ARM7_SPSR_UND) for the banked registers not seen in the current mode, and every SPSR. The core
	// switches r[8..14] when the CPSR mode changes (use arm7_set_cpsr from outside).
	uint32_t r[16];
	uint32_t cpsr;
	uint32_t bank[ARM7_NUM_REGS];
	// CP15 (MAME m_control, m_tlbBase, m_domainAccessControl, m_faultStatus, m_faultAddress, m_fcsePID): stored and
	// read back, no translation is done
	uint32_t cp15_control, cp15_tlb_base, cp15_dac, cp15_fsr[2], cp15_far, cp15_pid;
	uint8_t pending_irq, pending_fiq, pending_und, pending_swi, pending_interrupt;
	uint8_t pad_[3];
	int32_t icount;            // MAME m_icount (valid inside arm7_run)
	int64_t total_cycles;      // cycles run since reset (sum of arm7_run results)

	/* ---- not saved (run state, bus) ---- */
	int32_t slice;             // the cycles asked for by the arm7_run in progress (less what end_timeslice cut)
	int running;
	// ReBOX idle loop skip (arm7_run): on when idle_skip is set. A loop head (the target of a short backward jump)
	// reached with the same registers and flags, no memory changed in between (writes counts the writes that change
	// a value) and no interrupt waiting: the CPU is going round a cycle that only something outside can end, and
	// the slice ends there. idle_hits counts the slices ended
	int idle_skip;
	// archRev 5 (ARM946E-S, MAME arm946es_cpu_device: IGS036): BLX / PLD (cond NV), loads into R15 with bit 0 enter
	// Thumb, Thumb POP {PC} interworks, LDRSH odd addresses not shifted, the 946's CP15 (ID, control, TCM registers;
	// set before arm7_reset). The TCMs are not emulated: tcm_use notes that the code turned one on
	int arch5, tcm_use;
	uint32_t cp15_itcm_reg, cp15_dtcm_reg;   // (archRev 5; saved by the driver)
	int jit;   // (ReBOX: ARM code in read-only pages translated to host code: arm7_jit.inc; ARM hosts only)
	// (the translated code's last data pages, 4 for reads and 4 for writes: tag = address >> 10, host page; cleared
	// by arm7_map)
	struct { uint32_t tag; uint8_t *mem; } tlb_r[4], tlb_w[4];
	uint32_t writes, idle_hits;
	struct arm7_idle_head { uint32_t pc, cnt, valid, writes, cpsr, regs[16]; int32_t stamp; } idle[4];
	int idle_next;
	// ReBOX: MAME's speedup read handlers (a read by the instruction at hook_pc eats hook_eat cycles): the cycles are
	// eaten when that instruction runs, the same count in the same instruction; hook_eat 0 = none
	uint32_t hook_pc;
	int32_t hook_eat;
	// unmapped accesses. Addresses are passed as MAME's arm7_cpu_* give them to the bus: read32/write32 word aligned,
	// read16/write16 halfword aligned, read8/write8 unchanged. An unaligned 32-bit read rotates the word and an odd
	// 16-bit read rotates the halfword in the core, as MAME does. Instruction fetches from unmapped addresses use
	// read32 (a Thumb fetch takes the halfword of the word at pc & ~3). All six must be set before arm7_run.
	uint32_t (*read32)(void *user, uint32_t addr);
	uint16_t (*read16)(void *user, uint32_t addr);
	uint8_t (*read8)(void *user, uint32_t addr);
	void (*write32)(void *user, uint32_t addr, uint32_t data);
	void (*write16)(void *user, uint32_t addr, uint16_t data);
	void (*write8)(void *user, uint32_t addr, uint8_t data);
	void *user;
	// called when an IRQ / FIQ is taken (MAME standard_irq_callback), with line = ARM7_IRQ_LINE / ARM7_FIRQ_LINE,
	// before the vector instruction runs. Lets a driver drop a HOLD_LINE input. NULL = not called
	void (*irq_ack)(void *user, int line);
	// page tables (arm7_map): l1[addr >> 20] = 1 + index into l2[], 0 = no mapped page in that 1 MB block
	uint8_t l1[ARM7_L1_SIZE];
	int l2_used;
	struct { uint8_t *r[ARM7_L2_SIZE]; uint8_t *w[ARM7_L2_SIZE]; } l2[ARM7_L2_POOL];
};

// save state: the register block from r up to total_cycles as one blob (the bank layout follows cpsr in it)
#define ARM7_STATE_SIZE ((uint32_t)offsetof(arm7_t, slice))

// clear everything (registers, callbacks, map). Call once, then set the callbacks, map memory and arm7_reset.
void arm7_init(arm7_t *c);
// MAME device_reset: registers, CP15 and lines cleared, SVC mode with IRQ and FIQ masked (CPSR 0xd3), R15 = 0.
// The memory map and callbacks are kept.
void arm7_reset(arm7_t *c);
// run for about `cycles` clocks (MAME execute_run: instructions until icount <= 0, so the last one may overrun).
// Returns the clocks actually used. cycles <= 0 runs nothing and returns 0.
int arm7_run(arm7_t *c, int cycles);
// MAME execute_set_input for ARM7_IRQ_LINE / ARM7_FIRQ_LINE (state != 0 = asserted). Outside arm7_run the interrupt
// is checked (and taken) at once, as MAME does; from a handler inside arm7_run it is taken after the current
// instruction (MAME would take it at the next scheduler sync).
void arm7_set_irq_line(arm7_t *c, int line, int state);
// map [start, end] (1 KB aligned: start & 0x3ff == 0, (end & 0x3ff) == 0x3ff) to host memory at mem, guest byte
// order (little endian 32-bit words = host order on a little-endian host). Reads always use it, writes only when
// writable (otherwise they go to the write callbacks). mem == NULL unmaps the range (both directions).
// Returns 0, or -1 for a bad range / a full second-level pool.
int arm7_map(arm7_t *c, uint32_t start, uint32_t end, void *mem, int writable);
// stop arm7_run after the current instruction (Musashi m68k_end_timeslice / MAME abort_timeslice); the return value
// of arm7_run then counts only the clocks used up to here
void arm7_end_timeslice(arm7_t *c);
// the translated code thrown away (a game start, ROM contents changed): arm7_jit.inc
void arm7_jit_reset(void);
// MAME eat_cycles: remove n clocks from the current slice (they count as used)
void arm7_eat_cycles(arm7_t *c, int n);
// MAME pc() (STATE_GENPC = R15): inside a memory handler, the address of the executing instruction
static inline uint32_t arm7_pc(const arm7_t *c) { return c->r[15]; }
// CPSR write from outside (MAME set_cpsr: forces bit 4, switches the register bank)
void arm7_set_cpsr(arm7_t *c, uint32_t v);
// register i (0-15) of the given mode, wherever it lives now (MAME GetModeRegister)
uint32_t *arm7_mode_reg(arm7_t *c, int mode, int i);
// the SPSR of the current mode (MAME GetRegister(SPSR): R0 in USR / SYS, as MAME's register table gives)
uint32_t *arm7_spsr(arm7_t *c);

#ifdef __cplusplus
}
#endif

#endif
