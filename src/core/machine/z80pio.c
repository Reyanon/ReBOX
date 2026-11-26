/*  Zilog Z80 PIO. Ported from MAME src/devices/machine/z80pio.cpp (BSD-3-Clause, copyright-holders: Curt Coder).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "z80pio.h"

#include <string.h>

enum { MODE_OUTPUT = 0, MODE_INPUT, MODE_BIDIRECTIONAL, MODE_BIT_CONTROL };
enum { ANY = 0, IOR, MASK };
#define ICW_ENABLE_INT 0x80
#define ICW_MASK_FOLLOWS 0x10

static uint8_t in_port(z80pio_t *p, int index) { return p->in_p[index] ? p->in_p[index](p->user) : 0; }
static void out_port(z80pio_t *p, int index, uint8_t v) { if (p->out_p[index]) p->out_p[index](p->user, v); }

static void check_interrupts(z80pio_t *p)
{
	int state = 0;
	const int ius = p->port[0].ius || p->port[1].ius;
	for (int i = 0; i < 2; i++)
		if (!ius && p->port[i].ie && p->port[i].ip)
			state = 1;
	if (p->out_int)
		p->out_int(p->user, state);
}

int z80pio_int_state(const z80pio_t *p)
{
	const int ius = p->port[0].ius || p->port[1].ius;
	for (int i = 0; i < 2; i++)
		if (!ius && p->port[i].ie && p->port[i].ip)
			return 1;
	return 0;
}

static void set_rdy(z80pio_t *p, int index, int state)
{
	z80pio_port_t *q = &p->port[index];
	if (q->rdy == state)
		return;
	q->rdy = (uint8_t)state;
	if (p->out_rdy[index])
		p->out_rdy[index](p->user, state);
}

static void trigger_interrupt(z80pio_t *p, int index)
{
	p->port[index].ip = 1;
	check_interrupts(p);
}

static void set_mode(z80pio_t *p, int index, int mode)
{
	z80pio_port_t *q = &p->port[index];
	switch (mode) {
	case MODE_OUTPUT:
		out_port(p, index, q->output);
		set_rdy(p, index, 1);
		q->mode = (uint8_t)mode;
		break;
	case MODE_INPUT:
		q->mode = (uint8_t)mode;
		break;
	case MODE_BIDIRECTIONAL:
		if (index == Z80PIO_PORT_A)
			q->mode = (uint8_t)mode;
		break;
	case MODE_BIT_CONTROL:
		if (index == Z80PIO_PORT_A || p->port[0].mode != MODE_BIDIRECTIONAL)
			set_rdy(p, index, 0);
		q->ie = 0;
		check_interrupts(p);
		q->match = 0;
		q->next_control_word = IOR;
		q->mode = (uint8_t)mode;
		break;
	}
}

static void port_reset(z80pio_t *p, int index)
{
	z80pio_port_t *q = &p->port[index];
	set_mode(p, index, MODE_INPUT);
	q->icw &= (uint8_t)~ICW_ENABLE_INT;
	q->ie = 0;
	q->ip = 0;
	q->ius = 0;
	q->match = 0;
	q->ior = 0;
	q->mask = 0xff;
	q->output = 0;
	set_rdy(p, index, 0);
}

void z80pio_reset(z80pio_t *p)
{
	memset(p->port, 0, sizeof(p->port));   // (the constructor's values)
	port_reset(p, 0);
	port_reset(p, 1);
}

void z80pio_strobe(z80pio_t *p, int index, int state)
{
	z80pio_port_t *q = &p->port[index];
	if (p->port[0].mode == MODE_BIDIRECTIONAL) {
		if (q->rdy) {
			if (q->stb && !state) {   // falling edge
				if (index == Z80PIO_PORT_A)
					out_port(p, 0, q->output);
				else
					p->port[0].input = in_port(p, 0);
			} else if (!q->stb && state) {   // rising edge
				trigger_interrupt(p, index);
				set_rdy(p, index, 0);
			}
		}
	} else {
		switch (q->mode) {
		case MODE_OUTPUT:
			if (q->rdy && !q->stb && state) {   // rising edge
				trigger_interrupt(p, index);
				set_rdy(p, index, 0);
			}
			break;
		case MODE_INPUT:
			if (!state)
				q->input = in_port(p, index);
			else if (!q->stb && state) {   // rising edge
				trigger_interrupt(p, index);
				set_rdy(p, index, 0);
			}
			break;
		}
	}
	q->stb = (uint8_t)(state ? 1 : 0);
}

uint8_t z80pio_port_read(z80pio_t *p, int index)
{
	const z80pio_port_t *q = &p->port[index];
	switch (q->mode) {
	case MODE_OUTPUT: return q->output;
	case MODE_BIDIRECTIONAL: return index == Z80PIO_PORT_A ? q->output : 0xff;
	case MODE_BIT_CONTROL: return (uint8_t)(q->ior | (q->output & (q->ior ^ 0xff)));
	}
	return 0xff;
}

static void control_write(z80pio_t *p, int index, uint8_t data)
{
	z80pio_port_t *q = &p->port[index];
	switch (q->next_control_word) {
	case ANY:
		if (!(data & 1))
			q->vector = data;
		else
			switch (data & 0x0f) {
			case 0x0f:   // select operating mode
				set_mode(p, index, data >> 6);
				break;
			case 0x07:   // set interrupt control word
				q->icw = data;
				if (q->icw & ICW_MASK_FOLLOWS) {
					q->ie = 0;
					q->ip = 0;
					check_interrupts(p);
					q->match = 0;
					q->next_control_word = MASK;
				} else {
					q->ie = (q->icw >> 7) & 1;
					check_interrupts(p);
				}
				break;
			case 0x03:   // set interrupt enable flip-flop
				q->icw = (uint8_t)((data & 0x80) | (q->icw & 0x7f));
				q->ie = (q->icw >> 7) & 1;
				check_interrupts(p);
				break;
			}
		break;
	case IOR:   // data direction register
		q->ior = data;
		q->ie = (q->icw >> 7) & 1;
		check_interrupts(p);
		q->next_control_word = ANY;
		break;
	case MASK:   // interrupt mask
		q->mask = data;
		q->ie = (q->icw >> 7) & 1;
		check_interrupts(p);
		q->next_control_word = ANY;
		break;
	}
}

static uint8_t data_read(z80pio_t *p, int index)
{
	z80pio_port_t *q = &p->port[index];
	uint8_t data = 0;
	switch (q->mode) {
	case MODE_OUTPUT:
		data = q->output;
		break;
	case MODE_INPUT:
		if (!q->stb)
			q->input = in_port(p, index);
		data = q->input;
		set_rdy(p, index, 0);
		set_rdy(p, index, 1);
		break;
	case MODE_BIDIRECTIONAL:
		data = q->input;
		set_rdy(p, 1, 0);
		set_rdy(p, 1, 1);
		break;
	case MODE_BIT_CONTROL:
		q->input = in_port(p, index);
		data = (uint8_t)((q->input & q->ior) | (q->output & (q->ior ^ 0xff)));
		break;
	}
	return data;
}

static void data_write(z80pio_t *p, int index, uint8_t data)
{
	z80pio_port_t *q = &p->port[index];
	switch (q->mode) {
	case MODE_OUTPUT:
		set_rdy(p, index, 0);
		q->output = data;
		out_port(p, index, q->output);
		set_rdy(p, index, 1);
		break;
	case MODE_INPUT:
		q->output = data;
		break;
	case MODE_BIDIRECTIONAL:
		set_rdy(p, index, 0);
		q->output = data;
		if (!q->stb)
			out_port(p, index, data);
		set_rdy(p, index, 1);
		break;
	case MODE_BIT_CONTROL:
		q->output = data;
		out_port(p, index, (uint8_t)(q->ior | (q->output & (q->ior ^ 0xff))));
		break;
	}
}

uint8_t z80pio_read(z80pio_t *p, int offset)
{
	const int index = offset & 1;
	return (offset & 2) ? (uint8_t)((p->port[0].icw & 0xc0) | (p->port[1].icw >> 4)) : data_read(p, index);
}

void z80pio_write(z80pio_t *p, int offset, uint8_t data)
{
	const int index = offset & 1;
	if (offset & 2)
		control_write(p, index, data);
	else
		data_write(p, index, data);
}
