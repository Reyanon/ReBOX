/*  Motorola 6821 PIA, ported to C from MAME src/devices/machine/6821pia.cpp (BSD-3-Clause,
    copyright-holders: Aaron Giles, Nathan Woods). Same register model, CA/CB edge and strobe logic, IRQ outputs and
    "input overrides output" mask on port A. Unset handlers behave as MAME's unconnected ones.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_PIA6821_H
#define RBX_PIA6821_H

#include <stdint.h>
#include <stddef.h>

typedef struct pia6821_s pia6821_t;
struct pia6821_s {
	// state (saved)
	uint8_t in_a, in_ca1, in_ca2, out_a, out_ca2, ddr_a, ctl_a, irq_a1, irq_a2, irq_a_state;
	uint8_t in_b, in_cb1, in_cb2, out_b, out_cb2, last_out_cb2_z, ddr_b, ctl_b, irq_b1, irq_b2, irq_b_state;
	uint8_t in_a_pushed, in_b_pushed;
	uint8_t pad_[3];
	// configuration
	uint8_t a_input_overrides_output_mask;
	int ts_b;                                        // three-state port B value, -1 = no handler
	uint8_t (*in_a_cb)(void *ctx);                   // NULL = pushed value (porta_w) or 0xff
	uint8_t (*in_b_cb)(void *ctx);                   // NULL = pushed value (portb_w) or 0x00
	void (*out_a_cb)(void *ctx, uint8_t data);
	void (*out_b_cb)(void *ctx, uint8_t data);
	void (*ca2_cb)(void *ctx, int state);
	void (*cb2_cb)(void *ctx, int state);
	void (*irqa_cb)(void *ctx, int state);
	void (*irqb_cb)(void *ctx, int state);
	void *ctx;
};

#define PIA6821_STATE_SIZE ((uint32_t)offsetof(pia6821_t, a_input_overrides_output_mask))   // saved bytes

void pia6821_config(pia6821_t *p);   // clear everything (call before setting handlers)
void pia6821_reset(pia6821_t *p);
uint8_t pia6821_read(pia6821_t *p, int offset);
void pia6821_write(pia6821_t *p, int offset, uint8_t data);
void pia6821_porta_w(pia6821_t *p, uint8_t data);   // drive the port A inputs
void pia6821_portb_w(pia6821_t *p, uint8_t data);
void pia6821_ca1_w(pia6821_t *p, int state);
void pia6821_ca2_w(pia6821_t *p, int state);
void pia6821_cb1_w(pia6821_t *p, int state);
void pia6821_cb2_w(pia6821_t *p, int state);
uint8_t pia6821_b_output(pia6821_t *p);

#endif
