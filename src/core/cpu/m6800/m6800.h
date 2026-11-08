/*  Motorola 6800 / 6802 / 6808 CPU, ported to C from MAME src/devices/cpu/m6800 (m6800.cpp, 6800ops.hxx;
    BSD-3-Clause, copyright-holders: Aaron Giles; "m6800: Portable 6800 class emulator", John Butler, Haruki Ikeda).
    Only the 6800 instruction set (MAME m6800_insn / cycles_6800 tables). The 6802 internal RAM (0x0000-0x007f) is
    left to the board's memory map. The 6802 divides its input clock by 4 (MAME execute_clocks_to_cycles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_M6800_H
#define RBX_M6800_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct m6800_s m6800_t;
struct m6800_s {
	uint16_t pc, s, x;
	uint8_t a, b, cc;
	uint8_t wai, irq_line, nmi_line, nmi_pending, irq_hold;   // irq_hold: IRQ not taken before the next instruction
	uint8_t pad_[3];
	int64_t cycles;
	const uint8_t *rpage[256];
	uint8_t *wpage[256];
	uint8_t (*read)(void *ctx, uint16_t a);
	void (*write)(void *ctx, uint16_t a, uint8_t v);
	void *ctx;
};

void m6800_init(m6800_t *c);
void m6800_reset(m6800_t *c);
int m6800_step(m6800_t *c);
void m6800_run_until(m6800_t *c, int64_t target);
void m6800_set_irq(m6800_t *c, int state);
void m6800_set_nmi(m6800_t *c, int state);

#define M6800_STATE_SIZE ((uint32_t)(offsetof(m6800_t, rpage)))

#ifdef __cplusplus
}
#endif

#endif
