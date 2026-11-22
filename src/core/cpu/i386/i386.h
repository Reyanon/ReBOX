// license:BSD-3-Clause
// copyright-holders:Ville Linde, Barry Rodewald, Carl, Philip Bennett
/*  Intel 386 (DX) CPU: MAME's i386_device as plain C, the 386 only (build_opcode_table(OP_I386): no FPU, the x87
    escapes are MAME's i386_escape: the operand skipped), one instance.
    Ported from MAME 0.289 src/devices/cpu/i386/i386.cpp, i386.h, i386priv.h, cycles.h, i386ops.h, i386ops.hxx,
    i386op16.hxx, i386op32.hxx, i386segs.hxx (BSD-3-Clause, copyright-holders: Ville Linde, Barry Rodewald, Carl,
    Philip Bennett).
    This file is part of ReBOX. GPL-2.0-or-later.

    Left out: the debugger hooks, the data breakpoints of DR0-DR3 (the registers are kept, the taps are not made),
    the paging TLB (with CR0.PG set every access walks the page tables, as MAME does on a TLB miss), SMM (the 386 has
    no SMI). MAME's C++ exceptions (the faults thrown from deep in an access) are setjmp / longjmp here.
    The bus is MAME's 32-bit little-endian program space: the handlers get a dword address and the byte lanes'
    mask. Pages mapped with i386_map are read and written directly (64 KB granularity).
*/
#ifndef RBX_I386_H
#define RBX_I386_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	// program space: addr is dword aligned, mask has the byte lanes accessed (as MAME's read_dword(addr, mem_mask))
	uint32_t (*read32)(uint32_t addr, uint32_t mask);
	void (*write32)(uint32_t addr, uint32_t data, uint32_t mask);
	// I/O space (NULL: reads 0, writes ignored)
	uint32_t (*io_read32)(uint32_t addr, uint32_t mask);
	void (*io_write32)(uint32_t addr, uint32_t data, uint32_t mask);
	// the IRQ acknowledge: the vector (MAME's irq_acknowledge_callback; a HOLD_LINE driver clears its line here)
	int (*irq_ack)(void);
} i386_bus;

int i386_init(const i386_bus *bus);   // 0 = ok (the page maps are allocated)
void i386_exit(void);
// pages [start, end) (64 KB aligned) read from mem directly; writable: written too; mem NULL: the handlers again
void i386_map(uint32_t start, uint32_t end, uint8_t *mem, int writable);
// 4 KB parts of plain memory (no side effects) inside 64 KB pages that go through the handlers (left unmapped by
// i386_map): read and written straight, as the handlers would (up to 4 such pages; mem NULL: the handlers again)
void i386_map_split(uint32_t start, uint32_t end, uint8_t *mem, int writable);
void i386_reset(void);
// runs about `cycles` (MAME's execute_run with an icount); returns the cycles ran
int i386_run(int cycles);
int i386_done(void);   // inside i386_run: cycles done so far
void i386_abort(void); // inside i386_run: stop after this instruction (MAME abort_timeslice)
// the INT line (MAME execute_set_input(0, state)); state != 0 also ends a halt or a spin
void i386_set_irq(int state);
void i386_set_nmi(int state);
// MAME spin_until_interrupt: the rest of the slice and the slices after it are eaten until an input line is asserted
void i386_spin_until_interrupt(void);
uint32_t i386_pc(void);   // MAME pc(): m_pc, the linear address of the next fetch
// Exact skipping of polling loops inside i386_run (after i386_map; i386_init turns it off). A taken backward transfer
// to the same PC a third time in a row runs the loop's next iteration with the writes and the handler accesses caught;
// when it comes back to that PC with the CPU's state as before, every byte it wrote back to its old value, no write
// through a handler but of the bytes `pure` accepts with write = 1 (writes without any effect), no I/O port, no handler
// read but of the bytes `pure` accepts with write = 0 (side-effect free, their value changed by nothing but this CPU
// within a run; NULL: none), no paging, the iterations left are the same: the whole ones before the run's end are
// counted without being run. The board's rule: nothing else changes the direct pages or the pure bytes inside i386_run.
void i386_idle_enable(int (*pure)(uint32_t addr, int write));
int i386_state(void *ctx, int load);

#ifdef __cplusplus
}
#endif

#endif
