/*  Several 68000s on the one global Musashi core (darius, ninjaw ...): each CPU keeps its Musashi context, its
    HOLD_LINE interrupt lines and its own m68k_bus page table / handlers. Switching copies only the page table
    ranges where the CPUs' maps differ (computed once after the maps are built), so a per-scanline interleave
    stays cheap.
    Usage: for each CPU build its map in m68k_bus (m68k_bus_reset_map / m68k_bus_map / handlers) and call
    m68k_multi_capture(i); then m68k_multi_finish(); then m68k_multi_switch(i) before running CPU i.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_M68K_MULTI_H
#define RBX_M68K_MULTI_H

#include <stdint.h>

#define M68K_MULTI_MAX 2

int m68k_multi_init(int ncpu);           // 0 = ok (allocates the page tables)
void m68k_multi_exit(void);
void m68k_multi_capture(int cpu);        // store the map now in m68k_bus as CPU cpu's map
void m68k_multi_finish(void);            // all maps captured: compute the differing ranges, CPU 0 current
// make cpu the running CPU (Musashi context, interrupt lines, page table, handlers)
void m68k_multi_switch(int cpu);
int m68k_multi_current(void);
// pulse reset on every CPU (contexts start from their vectors), interrupt lines cleared; CPU 0 current
void m68k_multi_reset(void);
void m68k_multi_reset_cpu(int cpu);      // reset one CPU (MAME INPUT_LINE_RESET assert)
// HOLD_LINE interrupt for any CPU (applied when it runs)
void m68k_multi_hold_irq(int cpu, int level);
// ASSERT_LINE / CLEAR_LINE interrupt for any CPU (dassault's mailbox and vblank lines)
void m68k_multi_set_line(int cpu, int level, int state);
// a CPU's ASSERT_LINE levels (not in m68k_multi_state: the boards that use them save them)
uint32_t m68k_multi_asserted(int cpu);
void m68k_multi_set_asserted(int cpu, uint32_t mask);
// save state of all CPUs (contexts + interrupt lines)
int m68k_multi_state(void *ctx, int load);

#endif
