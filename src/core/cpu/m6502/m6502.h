/*  MOS 6502 (NMOS) and Rockwell R65C02, ported to C from MAME src/devices/cpu/m6502 (m6502.cpp, om6502.lst, dm6502.lst; BSD-3-Clause,
    copyright-holders: Olivier Galibert; w65c02.cpp, ow65c02.lst, r65c02.cpp, dr65c02.lst). The instructions are MAME's microcode as tools/gen_m6502.py translates it:
    every bus access costs one clock, the next opcode is fetched at the end of each instruction (where interrupts are
    taken, as MAME's prefetch), undocumented opcodes included. An instruction runs as one step.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_M6502_H
#define RBX_M6502_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { M6502_NMOS = 0, M6502_R65C02 = 1, M6502_DECO16 = 2 };

typedef struct m6502_s m6502_t;
struct m6502_s {
	// registers and state (saved: up to cycles)
	uint16_t pc, sp, tmp;
	uint8_t a, x, y, p, tmp2, ir;
	uint8_t irq_state, nmi_state, nmi_pending, irq_taken, halted, waiting;
	int32_t inst_state;        // (MAME m_inst_state: -1 normal; the reset sequence pending)
	int64_t cycles;            // clocks executed since init
	// bus: 256-byte pages read / written directly when the pointer is set, else through the callbacks
	const uint8_t *rpage[256];
	uint8_t *wpage[256];
	uint8_t (*read)(void *ctx, uint16_t a);
	void (*write)(void *ctx, uint16_t a, uint8_t v);
	void *ctx;
	int type;                  // M6502_* (configuration)
	int deco222;               // (configuration) the DECO 222 / C10707: opcodes with bits 5 and 6 swapped
	int irq_hold;              // (configuration) MAME HOLD_LINE: the IRQ line dropped when the CPU takes it
	// (configuration) MAME's AS_OPCODES space: the opcode fetches (read_sync, not the arguments) at op_start and up
	// from op_rom[a - op_start]; NULL: none
	const uint8_t *op_rom;
	uint16_t op_start;
	// (configuration) the DECO CPU-7 (decocpu7.cpp): the first opcode fetched after a write, at an address with bits 8
	// and 2 set, has its bits swapped; cpu7_written (state, saved by the board) is that pending write
	int cpu7;
	uint8_t cpu7_written;
	// the DECO CPU-16's I/O space (its opcodes 0x4b, 0x67, 0x8f): NULL = 0xff / dropped
	uint8_t (*io_read)(void *ctx, uint8_t a);
	void (*io_write)(void *ctx, uint8_t a, uint8_t v);
};

// clear registers and the bus tables (call once, then set the bus and m6502_reset)
void m6502_init(m6502_t *c, int type);
// MAME device_reset: the reset sequence (vector 0xfffc) runs at the next step
void m6502_reset(m6502_t *c);
// one instruction (or the reset / interrupt sequence); returns the clocks used
int m6502_step(m6502_t *c);
// run until c->cycles >= target
void m6502_run_until(m6502_t *c, int64_t target);
void m6502_set_irq(m6502_t *c, int state);
void m6502_set_nmi(m6502_t *c, int state);   // (edge-triggered)

// save state: the register block (from pc up to cycles) as one blob
#define M6502_STATE_SIZE ((uint32_t)(offsetof(m6502_t, rpage)))

#ifdef __cplusplus
}
#endif
#endif
