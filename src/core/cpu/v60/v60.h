/*  NEC V60 (uPD70615) CPU, ported to C from MAME 0.289 src/devices/cpu/v60: v60.cpp, v60.h and the *.hxx
    instruction / addressing mode files (BSD-3-Clause, copyright-holders: Farfetch'd, R. Belmont).
    MAME's interpreter as it is (the .hxx files converted by tools/v60conv.py into v60_ops.inc), with MAME's timing:
    8 cycles an instruction (MAME's "average"), interrupts checked before the slice and after every instruction.
    One V60 (the single-CPU boards: Sega System 32). The bus is little-endian, 16 bits wide, 24-bit addresses, as
    MAME's 16-bit memory_access cache: unaligned words / longs split into MAME's native 16-bit accesses (with their
    byte masks) for the handlers; 4 KB pages of host memory (little-endian bytes) for RAM / ROM.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_V60_H
#define RBX_V60_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define V60_PAGE_SHIFT 12
// (the page tables cover 26 address bits: the V70 boards' 64 MB mirror, v60_bus_set_bits; the V60 boards use 24)
#define V60_PAGES (1u << (26 - V60_PAGE_SHIFT))

typedef struct {
	uint8_t *rd[V60_PAGES];   // host memory of the page (little-endian bytes), NULL = the handlers
	uint8_t *wr[V60_PAGES];
	// native 16-bit accesses (a even): mask = the byte lanes (0x00ff = the even byte)
	uint16_t (*read16)(uint32_t a, uint16_t mask);
	void (*write16)(uint32_t a, uint16_t data, uint16_t mask);
	// MAME irq_cycle_callback: the vector of the interrupt being taken (0x40 is added)
	uint8_t (*irq_cycle)(void);
	uint32_t amask;   // the address bits decoded (v60_bus_clear: 24, the V60; v60_bus_set_bits)
} v60_bus_t;

extern v60_bus_t v60_bus;

void v60_bus_clear(void);   // all pages to the handlers, 24 address bits
// after v60_bus_clear: the address bits decoded (24 or 26: the V70 boards' 64 MB mirrored image of the 32-bit space,
// Jaleco Mega System 32); the maps and the handlers see the addresses masked to them
void v60_bus_set_bits(int bits);
// map [start, end] (page aligned, inclusive) to mem, repeating every size bytes (a power of two, >= 4 KB)
void v60_bus_map(uint32_t start, uint32_t end, void *mem, uint32_t size, int readable, int writable);

void v60_init(void);        // device_start (all registers but SP, PC, PSW, SBR, SYCW, TKCW, PIR, PSW2 cleared)
void v60_reset(void);       // device_reset
void v60_release(void);     // at the game's end: the JIT's tables given back (made again at the next run, ADR-200)
void v60_set_pir(uint32_t pir);   // after v60_init: the V70's PIR (0x7000; its 32-bit bus is taken as the V60's)
int v60_execute(int cycles);   // returns the cycles run (MAME's ran: overshoot included)
void v60_abort_timeslice(void);   // from a handler: end the slice after the current instruction
int v60_executing(void);
int v60_cycles_left(void);  // inside v60_execute: the cycles left of the slice (MAME m_icount)
int v60_cycles_run(void);   // inside v60_execute: the cycles run so far (MAME's local time)
void v60_set_irq_line(int state);   // 0 clear, 1 assert (taken now if PSW.IE)
void v60_set_nmi_line(int state);
uint32_t v60_pc(void);
uint32_t v60_ppc(void);

// the saved state (registers, flags, lines)
typedef struct {
	uint32_t reg[68];
	uint8_t cy, ov, s, z;
	uint8_t irq_line, nmi_line, stall_io, pad;
	uint32_t ppc;
} v60_state_t;
void v60_get_state(v60_state_t *st);
void v60_set_state(const v60_state_t *st);

#ifdef __cplusplus
}
#endif

#endif
