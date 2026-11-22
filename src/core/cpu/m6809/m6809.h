/*  Motorola 6809 / 6809E CPU, ported to C from MAME src/devices/cpu/m6809 (m6809.cpp, m6809.lst, base6x09.lst,
    m6809inl.h; BSD-3-Clause, copyright-holders: Nathan Woods; "Portable 6809 emulator", John Butler).
    Instruction semantics, flag rules, undocumented opcodes and cycle counts follow MAME: every bus access and every
    dummy (VMA) cycle costs one clock, so the totals match MAME's cycle-exact core. An instruction runs as one step
    (interrupt lines are sampled before each instruction, as MAME does at its MAIN label).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_M6809_H
#define RBX_M6809_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct m6809_s m6809_t;
struct m6809_s {
	// registers
	uint16_t pc, x, y, u, s;
	uint8_t a, b, dp, cc;
	// interrupt lines / state
	uint8_t irq_line, firq_line, nmi_line, nmi_asserted, lds_encountered;
	uint8_t wait;              // 0 = running, 1 = CWAI (state pushed), 2 = SYNC
	uint8_t firq_hold;   // (m6809_hold_firq: the FIRQ line dropped when taken, MAME's HOLD_LINE)
	uint8_t irq_hold;    // (m6809_hold_irq: the same for IRQ; in what was the padding)
	int64_t cycles;            // clocks executed since reset
	// bus: 256-byte pages read / written directly when the pointer is set, else through the callbacks
	const uint8_t *rpage[256];
	uint8_t *wpage[256];
	uint8_t (*read)(void *ctx, uint16_t a);
	void (*write)(void *ctx, uint16_t a, uint8_t v);
	// first opcode byte fetch (6809E LIC high): NULL = normal read
	uint8_t (*read_op)(void *ctx, uint16_t a);
	void *ctx;
	// KONAMI1 (konami1.cpp): the opcode bytes (the prefixes' too) XOR-ed by the address's bits 1 and 3
	uint8_t konami1;
	// wait loops: idle_hook runs before an opcode is fetched at idle_pc[0] or [1] (NULL: none; a driver that skips
	// loops it knows exactly)
	uint16_t idle_pc[2];
	void (*idle_hook)(m6809_t *c);
};

// clear registers and the bus tables (call once, then set the bus and m6809_reset)
void m6809_init(m6809_t *c);
void m6809_reset(m6809_t *c);
// one instruction (or interrupt entry); returns the clocks used
int m6809_step(m6809_t *c);
// run until c->cycles >= target (a waiting CPU jumps to the target)
void m6809_run_until(m6809_t *c, int64_t target);
void m6809_set_irq(m6809_t *c, int state);
void m6809_set_firq(m6809_t *c, int state);
void m6809_hold_firq(m6809_t *c);   // (set_input_line(FIRQ, HOLD_LINE))
void m6809_hold_irq(m6809_t *c);    // (set_input_line(IRQ, HOLD_LINE))
void m6809_set_nmi(m6809_t *c, int state);

// save state: the register block (from pc up to cycles) as one blob
#define M6809_STATE_SIZE ((uint32_t)(offsetof(m6809_t, rpage)))

#ifdef __cplusplus
}
#endif

#endif
