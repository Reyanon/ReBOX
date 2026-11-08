/*  Intel 8259 / NEC uPD71059C interrupt controller. Ported from MAME src/devices/machine/pic8259.cpp (BSD-3-Clause,
    copyright-holders: Wilbert Pol), without the cascade. MAME's zero timer after a change is a call here.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef PIC8259_H
#define PIC8259_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
	uint8_t state, isr, irr, prio, imr, irq_lines, ocw3;
	uint8_t level_trig_mode, vector_size, cascade, icw4_needed, vector_addr_low, base, vector_addr_high;
	uint8_t slave, nested, mode, auto_eoi, is_x86, inta_sequence;
	int8_t current_level;
	uint8_t state_end;
	void (*out_int)(int state);
} pic8259_t;

#define PIC8259_STATE_SIZE ((uint32_t)offsetof(pic8259_t, state_end))

void pic8259_reset(pic8259_t *p);
void pic8259_set_irq(pic8259_t *p, int irq, int state);   // the IRn lines
uint8_t pic8259_ack(pic8259_t *p);                        // the vector of an INTA
uint8_t pic8259_read(pic8259_t *p, int offset);
void pic8259_write(pic8259_t *p, int offset, uint8_t data);

#endif
