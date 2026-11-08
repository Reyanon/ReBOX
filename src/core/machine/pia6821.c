/*  Motorola 6821 PIA, ported to C from MAME src/devices/machine/6821pia.cpp (BSD-3-Clause,
    copyright-holders: Aaron Giles, Nathan Woods).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "pia6821.h"

#include <string.h>

#define PIA_IRQ1 0x80
#define PIA_IRQ2 0x40

static inline int irq1_enabled(uint8_t c)    { return c & 0x01; }
static inline int c1_low_to_high(uint8_t c)  { return (c >> 1) & 1; }
static inline int c1_high_to_low(uint8_t c)  { return !((c >> 1) & 1); }
static inline int output_selected(uint8_t c) { return (c >> 2) & 1; }
static inline int irq2_enabled(uint8_t c)    { return (c >> 3) & 1; }
static inline int strobe_e_reset(uint8_t c)  { return (c >> 3) & 1; }
static inline int strobe_c1_reset(uint8_t c) { return !((c >> 3) & 1); }
static inline int c2_set(uint8_t c)          { return (c >> 3) & 1; }
static inline int c2_low_to_high(uint8_t c)  { return (c >> 4) & 1; }
static inline int c2_high_to_low(uint8_t c)  { return !((c >> 4) & 1); }
static inline int c2_set_mode(uint8_t c)     { return (c >> 4) & 1; }
static inline int c2_strobe_mode(uint8_t c)  { return !((c >> 4) & 1); }
static inline int c2_output(uint8_t c)       { return (c >> 5) & 1; }
static inline int c2_input(uint8_t c)        { return !((c >> 5) & 1); }

void pia6821_config(pia6821_t *p)
{
	memset(p, 0, sizeof(*p));
	p->in_a = 0xff;
	p->in_ca1 = 1;
	p->in_ca2 = 1;
	p->ts_b = -1;
}

static void update_interrupts(pia6821_t *p)
{
	int s = (p->irq_a1 && irq1_enabled(p->ctl_a)) || (p->irq_a2 && irq2_enabled(p->ctl_a));
	if (s != p->irq_a_state) {
		p->irq_a_state = (uint8_t)s;
		if (p->irqa_cb)
			p->irqa_cb(p->ctx, s);
	}
	s = (p->irq_b1 && irq1_enabled(p->ctl_b)) || (p->irq_b2 && irq2_enabled(p->ctl_b));
	if (s != p->irq_b_state) {
		p->irq_b_state = (uint8_t)s;
		if (p->irqb_cb)
			p->irqb_cb(p->ctx, s);
	}
}

void pia6821_reset(pia6821_t *p)
{
	p->out_a = 0;
	p->out_ca2 = 0;
	p->ddr_a = 0;
	p->ctl_a = 0;
	p->irq_a1 = p->irq_a2 = 0;
	p->irq_a_state = 0;
	p->out_b = 0;
	p->out_cb2 = 0;
	p->last_out_cb2_z = 0;
	p->ddr_b = 0;
	p->ctl_b = 0;
	p->irq_b1 = p->irq_b2 = 0;
	p->irq_b_state = 0;
	if (p->irqa_cb) p->irqa_cb(p->ctx, 0);
	if (p->irqb_cb) p->irqb_cb(p->ctx, 0);
	if (p->out_a_cb) p->out_a_cb(p->ctx, 0xff);
	if (p->ca2_cb) p->ca2_cb(p->ctx, 1);
	if (p->out_b_cb && p->ts_b >= 0) p->out_b_cb(p->ctx, (uint8_t)p->ts_b);
}

static uint8_t get_in_a_value(pia6821_t *p)
{
	uint8_t d;
	if (p->in_a_cb)
		d = p->in_a_cb(p->ctx);
	else
		d = p->in_a_pushed ? p->in_a : 0xff;
	uint8_t m = p->a_input_overrides_output_mask;
	return (uint8_t)((~p->ddr_a & d) | (p->ddr_a & p->out_a & ~m) | (p->ddr_a & d & m));
}

static uint8_t get_in_b_value(pia6821_t *p)
{
	if (p->ddr_b == 0xff)
		return p->out_b;
	uint8_t d;
	if (p->in_b_cb)
		d = p->in_b_cb(p->ctx);
	else
		d = p->in_b_pushed ? p->in_b : 0x00;
	return (uint8_t)((p->out_b & p->ddr_b) | (d & ~p->ddr_b));
}

static uint8_t get_out_a_value(pia6821_t *p)
{
	if (p->ddr_a == 0xff)
		return p->out_a;
	return (uint8_t)((p->out_a & p->ddr_a) | (get_in_a_value(p) & ~p->ddr_a));
}

static uint8_t get_out_b_value(pia6821_t *p)
{
	uint8_t r = p->out_b & p->ddr_b;
	if (p->ddr_b != 0xff && p->ts_b >= 0)
		r |= (uint8_t)(p->ts_b & ~p->ddr_b);
	return r;
}

uint8_t pia6821_b_output(pia6821_t *p) { return get_out_b_value(p); }

static void set_out_ca2(pia6821_t *p, int data)
{
	p->out_ca2 = (uint8_t)data;
	if (p->ca2_cb)
		p->ca2_cb(p->ctx, data);
}

static void set_out_cb2(pia6821_t *p, int data)
{
	int z = !c2_output(p->ctl_b);
	if (data != p->out_cb2 || z != p->last_out_cb2_z) {
		p->out_cb2 = (uint8_t)data;
		p->last_out_cb2_z = (uint8_t)z;
		if (p->cb2_cb)
			p->cb2_cb(p->ctx, data);
	}
}

static void send_out_a(pia6821_t *p)
{
	if (p->out_a_cb)
		p->out_a_cb(p->ctx, get_out_a_value(p));
}

static void send_out_b(pia6821_t *p)
{
	if (p->out_b_cb)
		p->out_b_cb(p->ctx, get_out_b_value(p));
}

static uint8_t port_a_r(pia6821_t *p)
{
	uint8_t r = get_in_a_value(p);
	p->irq_a1 = p->irq_a2 = 0;
	update_interrupts(p);
	if (c2_output(p->ctl_a) && c2_strobe_mode(p->ctl_a)) {
		if (p->out_ca2)
			set_out_ca2(p, 0);
		if (strobe_e_reset(p->ctl_a))
			set_out_ca2(p, 1);
	}
	return r;
}

static uint8_t port_b_r(pia6821_t *p)
{
	uint8_t r = get_in_b_value(p);
	if (p->irq_b1 && c2_strobe_mode(p->ctl_b) && strobe_c1_reset(p->ctl_b))
		set_out_cb2(p, 1);
	p->irq_b1 = p->irq_b2 = 0;
	update_interrupts(p);
	return r;
}

uint8_t pia6821_read(pia6821_t *p, int offset)
{
	uint8_t r;
	switch (offset & 3) {
	case 0:
		return output_selected(p->ctl_a) ? port_a_r(p) : p->ddr_a;
	case 1:
		r = p->ctl_a;
		if (p->irq_a1) r |= PIA_IRQ1;
		if (p->irq_a2 && c2_input(p->ctl_a)) r |= PIA_IRQ2;
		return r;
	case 2:
		return output_selected(p->ctl_b) ? port_b_r(p) : p->ddr_b;
	default:
		r = p->ctl_b;
		if (p->irq_b1) r |= PIA_IRQ1;
		if (p->irq_b2 && c2_input(p->ctl_b)) r |= PIA_IRQ2;
		return r;
	}
}

static void control_a_w(pia6821_t *p, uint8_t data)
{
	data &= 0x3f;
	int was_output = c2_output(p->ctl_a);
	p->ctl_a = data;
	if (c2_output(data)) {
		if (c2_set_mode(data)) {
			int set = c2_set(data);
			if (!was_output || p->out_ca2 != set)
				set_out_ca2(p, set);
		} else if (!was_output || !p->out_ca2) {
			set_out_ca2(p, 1);
		}
	} else if (was_output) {
		if (p->ca2_cb)
			p->ca2_cb(p->ctx, 1);
	}
	update_interrupts(p);
}

static void control_b_w(pia6821_t *p, uint8_t data)
{
	data &= 0x3f;
	p->ctl_b = data;
	set_out_cb2(p, c2_set_mode(data) ? c2_set(data) : 1);
	update_interrupts(p);
}

void pia6821_write(pia6821_t *p, int offset, uint8_t data)
{
	switch (offset & 3) {
	case 0:
		if (output_selected(p->ctl_a)) {
			p->out_a = data;
			send_out_a(p);
		} else if (p->ddr_a != data) {
			p->ddr_a = data;
			send_out_a(p);
		}
		break;
	case 1:
		control_a_w(p, data);
		break;
	case 2:
		if (output_selected(p->ctl_b)) {
			p->out_b = data;
			send_out_b(p);
			if (c2_strobe_mode(p->ctl_b)) {
				set_out_cb2(p, 0);
				if (strobe_e_reset(p->ctl_b))
					set_out_cb2(p, 1);
			}
		} else if (p->ddr_b != data) {
			p->ddr_b = data;
			send_out_b(p);
		}
		break;
	default:
		control_b_w(p, data);
		break;
	}
}

void pia6821_porta_w(pia6821_t *p, uint8_t data)
{
	p->in_a = data;
	p->in_a_pushed = 1;
}

void pia6821_portb_w(pia6821_t *p, uint8_t data)
{
	p->in_b = data;
	p->in_b_pushed = 1;
}

void pia6821_ca1_w(pia6821_t *p, int state)
{
	state = state != 0;
	if (p->in_ca1 != state && ((state && c1_low_to_high(p->ctl_a)) || (!state && c1_high_to_low(p->ctl_a)))) {
		p->irq_a1 = 1;
		update_interrupts(p);
		if (c2_output(p->ctl_a) && c2_strobe_mode(p->ctl_a) && strobe_c1_reset(p->ctl_a) && !p->out_ca2)
			set_out_ca2(p, 1);
	}
	p->in_ca1 = (uint8_t)state;
}

void pia6821_ca2_w(pia6821_t *p, int state)
{
	state = state != 0;
	if (c2_input(p->ctl_a) && p->in_ca2 != state &&
	    ((state && c2_low_to_high(p->ctl_a)) || (!state && c2_high_to_low(p->ctl_a)))) {
		p->irq_a2 = 1;
		update_interrupts(p);
	}
	p->in_ca2 = (uint8_t)state;
}

void pia6821_cb1_w(pia6821_t *p, int state)
{
	state = state != 0;
	if (p->in_cb1 != state && ((state && c1_low_to_high(p->ctl_b)) || (!state && c1_high_to_low(p->ctl_b)))) {
		p->irq_b1 = 1;
		update_interrupts(p);
	}
	p->in_cb1 = (uint8_t)state;
}

void pia6821_cb2_w(pia6821_t *p, int state)
{
	state = state != 0;
	if (c2_input(p->ctl_b) && p->in_cb2 != state &&
	    ((state && c2_low_to_high(p->ctl_b)) || (!state && c2_high_to_low(p->ctl_b)))) {
		p->irq_b2 = 1;
		update_interrupts(p);
	}
	p->in_cb2 = (uint8_t)state;
}
