/*  Konami 052001 custom CPU (6809 derivative: Aliens, Simpsons, Vendetta, Parodius, Thunder Cross, 88 Games ...),
    ported to C from MAME src/devices/cpu/m6809 (konami.cpp, konami.h, konami.lst, base6x09.lst, m6809.h,
    m6809inl.h; BSD-3-Clause, copyright-holders: Nathan Woods; based on the M6809 core by John Butler).
    Instruction semantics, flag rules and cycle counts follow MAME: every bus access and every dummy (VMA) cycle
    costs one clock (MAME runs this core with hd6309_native_mode() = true, so the 6809 "extra" dummy cycles are
    absent). An instruction runs as one step; interrupt lines are sampled before each instruction, as MAME does at
    its MAIN label. The Konami CPU has no SWI / CWAI / SYNC.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_KONAMI_H
#define RBX_KONAMI_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct konami_s konami_t;
struct konami_s {
	// registers
	uint16_t pc, x, y, u, s;
	uint8_t a, b, dp, cc;
	// interrupt lines / state
	uint8_t irq_line, firq_line, nmi_line, nmi_asserted, lds_encountered;
	// MAME m_temp_im / m_bcount (shift count latch of LSRD/ASLD ...)
	uint8_t temp_im, bcount;
	uint8_t firq_hold;   // (konami_hold_firq: the FIRQ line dropped when taken, MAME's HOLD_LINE)
	uint8_t pad_[2];
	int64_t cycles;            // clocks executed since reset
	// bus: 256-byte pages read / written directly when the pointer is set, else through the callbacks
	const uint8_t *rpage[256];
	uint8_t *wpage[256];
	uint8_t (*read)(void *ctx, uint16_t a);
	void (*write)(void *ctx, uint16_t a, uint8_t v);
	// SETLINES instruction output (konami_cpu_device::line(), boards bank ROM through it); NULL = ignored
	void (*set_lines)(void *ctx, uint8_t data);
	void *ctx;
};

// clear registers and the bus tables (call once, then set the bus and konami_reset)
void konami_init(konami_t *c);
void konami_reset(konami_t *c);
// one instruction (or interrupt entry); returns the clocks used
int konami_step(konami_t *c);
// run until c->cycles >= target (block instructions may overshoot)
void konami_run_until(konami_t *c, int64_t target);
void konami_set_irq(konami_t *c, int state);
void konami_set_firq(konami_t *c, int state);
void konami_set_nmi(konami_t *c, int state);
void konami_hold_firq(konami_t *c);   // (set_input_line(FIRQ, HOLD_LINE))

// save state: the register block (from pc up to cycles) as one blob
#define KONAMI_STATE_SIZE ((uint32_t)(offsetof(konami_t, rpage)))

#ifdef __cplusplus
}
#endif

#endif
