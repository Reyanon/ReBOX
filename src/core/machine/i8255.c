/*  Intel 8255 PPI. Ported from MAME src/devices/machine/i8255.cpp (BSD-3-Clause, copyright-holders: Curt Coder).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "i8255.h"

#include <stddef.h>

enum { PORT_A, PORT_B, PORT_C, CONTROL };
enum { GROUP_A, GROUP_B };
enum { MODE_0, MODE_1, MODE_2 };
enum { MODE_OUTPUT, MODE_INPUT };

#define CONTROL_PORT_C_LOWER_INPUT 0x01
#define CONTROL_PORT_B_INPUT 0x02
#define CONTROL_GROUP_B_MODE_1 0x04
#define CONTROL_PORT_C_UPPER_INPUT 0x08
#define CONTROL_PORT_A_INPUT 0x10
#define CONTROL_GROUP_A_MODE_MASK 0x60
#define CONTROL_MODE_SET 0x80

static int group_mode(const i8255_t *p, int group)
{
	if (group == GROUP_A) {
		switch ((p->control & CONTROL_GROUP_A_MODE_MASK) >> 5) {
		case 0: return MODE_0;
		case 1: return MODE_1;
		default: return MODE_2;
		}
	}
	return (p->control & CONTROL_GROUP_B_MODE_1) ? MODE_1 : MODE_0;
}
static int port_mode(const i8255_t *p, int port)
{
	if (port == PORT_A)
		return (p->control & CONTROL_PORT_A_INPUT) ? MODE_INPUT : MODE_OUTPUT;
	return (p->control & CONTROL_PORT_B_INPUT) ? MODE_INPUT : MODE_OUTPUT;
}
static int port_c_lower_mode(const i8255_t *p) { return (p->control & CONTROL_PORT_C_LOWER_INPUT) ? MODE_INPUT : MODE_OUTPUT; }
static int port_c_upper_mode(const i8255_t *p) { return (p->control & CONTROL_PORT_C_UPPER_INPUT) ? MODE_INPUT : MODE_OUTPUT; }

static void out_pa(i8255_t *p, uint8_t v) { if (p->out_pa) p->out_pa(p->user, v); }
static void out_pb(i8255_t *p, uint8_t v) { if (p->out_pb) p->out_pb(p->user, v); }

static void output_pc(i8255_t *p)
{
	uint8_t data = 0, mask = 0, b_mask = 0x0f;
	switch (group_mode(p, GROUP_A)) {
	case MODE_0:
		if (port_c_upper_mode(p) == MODE_OUTPUT)
			mask |= 0xf0;
		else
			data |= p->tri_pc & 0xf0;
		break;
	case MODE_1:
		data |= p->intr[PORT_A] ? 0x08 : 0x00;
		if (port_mode(p, PORT_A) == MODE_OUTPUT) {
			data |= p->obf[PORT_A] ? 0x80 : 0x00;
			mask |= 0x30;
		} else {
			data |= p->ibf[PORT_A] ? 0x20 : 0x00;
			mask |= 0xc0;
		}
		break;
	case MODE_2:
		b_mask = 0x07;
		data |= p->intr[PORT_A] ? 0x08 : 0x00;
		data |= p->ibf[PORT_A] ? 0x20 : 0x00;
		data |= p->obf[PORT_A] ? 0x80 : 0x00;
		break;
	}
	switch (group_mode(p, GROUP_B)) {
	case MODE_0:
		if (port_c_lower_mode(p) == MODE_OUTPUT)
			mask |= b_mask;
		else
			data |= p->tri_pc & b_mask;
		break;
	case MODE_1:
		data |= p->intr[PORT_B] ? 0x01 : 0x00;
		if (port_mode(p, PORT_B) == MODE_OUTPUT)
			data |= p->obf[PORT_B] ? 0x02 : 0x00;
		else
			data |= p->ibf[PORT_B] ? 0x02 : 0x00;
		break;
	}
	data |= p->output[PORT_C] & mask;
	if (p->out_pc)
		p->out_pc(p->user, data);
}

static void set_intr(i8255_t *p, int port, int state)
{
	p->intr[port] = (uint8_t)state;
	output_pc(p);
}
static void check_interrupt(i8255_t *p, int port)
{
	switch (group_mode(p, port)) {
	case MODE_1:
		switch (port_mode(p, port)) {
		case MODE_INPUT: set_intr(p, port, p->inte[port] && p->ibf[port]); break;
		case MODE_OUTPUT: set_intr(p, port, p->inte[port] && p->obf[port]); break;
		}
		break;
	case MODE_2:
		set_intr(p, port, (p->inte1 && p->obf[port]) || (p->inte2 && p->ibf[port]));
		break;
	}
}
static void set_ibf(i8255_t *p, int port, int state) { p->ibf[port] = (uint8_t)state; check_interrupt(p, port); }
static void set_obf(i8255_t *p, int port, int state) { p->obf[port] = (uint8_t)state; check_interrupt(p, port); }
static void set_inte(i8255_t *p, int port, int state) { p->inte[port] = (uint8_t)state; check_interrupt(p, port); }
static void set_inte1(i8255_t *p, int state) { p->inte1 = (uint8_t)state; check_interrupt(p, PORT_A); }
static void set_inte2(i8255_t *p, int state) { p->inte2 = (uint8_t)state; check_interrupt(p, PORT_A); }

static uint8_t in_port(i8255_t *p, int port)
{
	uint8_t (*f)(void *) = port == PORT_A ? p->in_pa : port == PORT_B ? p->in_pb : p->in_pc;
	return f ? f(p->user) : 0;
}

static uint8_t read_mode0(i8255_t *p, int port)
{
	return port_mode(p, port) == MODE_OUTPUT ? p->output[port] : in_port(p, port);
}
static uint8_t read_mode1(i8255_t *p, int port)
{
	if (port_mode(p, port) == MODE_OUTPUT)
		return p->output[port];
	const uint8_t data = p->input[port];
	set_ibf(p, port, 0);
	set_intr(p, port, 0);
	p->input[port] = 0;
	return data;
}
static uint8_t read_mode2(i8255_t *p)
{
	const uint8_t data = p->input[PORT_A];
	set_ibf(p, PORT_A, 0);
	set_intr(p, PORT_A, 0);
	p->input[PORT_A] = 0;
	return data;
}
static uint8_t read_pc(i8255_t *p)
{
	uint8_t data = 0, mask = 0, b_mask = 0x0f;
	switch (group_mode(p, GROUP_A)) {
	case MODE_0:
		if (port_c_upper_mode(p) == MODE_OUTPUT)
			data |= p->output[PORT_C] & 0xf0;
		else
			mask |= 0xf0;
		break;
	case MODE_1:
		data |= p->intr[PORT_A] ? 0x08 : 0x00;
		if (port_mode(p, PORT_A) == MODE_OUTPUT) {
			data |= p->obf[PORT_A] ? 0x80 : 0x00;
			data |= p->inte[PORT_A] ? 0x40 : 0x00;
			mask |= 0x30;
		} else {
			data |= p->ibf[PORT_A] ? 0x20 : 0x00;
			data |= p->inte[PORT_A] ? 0x10 : 0x00;
			mask |= 0xc0;
		}
		if (port_c_upper_mode(p) == MODE_OUTPUT) {
			data |= p->output[PORT_C] & mask;
			mask = 0;
		}
		break;
	case MODE_2:
		b_mask = 0x07;
		data |= p->intr[PORT_A] ? 0x08 : 0x00;
		data |= p->inte2 ? 0x10 : 0x00;
		data |= p->ibf[PORT_A] ? 0x20 : 0x00;
		data |= p->inte1 ? 0x40 : 0x00;
		data |= p->obf[PORT_A] ? 0x80 : 0x00;
		break;
	}
	switch (group_mode(p, GROUP_B)) {
	case MODE_0:
		if (port_c_lower_mode(p) == MODE_OUTPUT)
			data |= p->output[PORT_C] & b_mask;
		else
			mask |= b_mask;
		break;
	case MODE_1:
		data |= p->inte[PORT_B] ? 0x04 : 0x00;
		data |= p->intr[PORT_B] ? 0x01 : 0x00;
		if (port_mode(p, PORT_B) == MODE_OUTPUT)
			data |= p->obf[PORT_B] ? 0x02 : 0x00;
		else
			data |= p->ibf[PORT_B] ? 0x02 : 0x00;
		break;
	}
	if (mask)
		data |= in_port(p, PORT_C) & mask;
	return data;
}

static void write_mode01(i8255_t *p, int port, uint8_t data, int mode1)
{
	if (port_mode(p, port) != MODE_OUTPUT)
		return;
	p->output[port] = data;
	if (port == PORT_A)
		out_pa(p, data);
	else
		out_pb(p, data);
	if (mode1) {
		set_obf(p, port, 0);
		set_intr(p, port, 0);
	}
}
static void write_mode2(i8255_t *p, uint8_t data)
{
	p->output[PORT_A] = data;
	out_pa(p, data);
	set_obf(p, PORT_A, 0);
	set_intr(p, PORT_A, 0);
}

static void set_mode(i8255_t *p, uint8_t data)
{
	p->control = data;
	p->output[PORT_A] = 0;
	p->input[PORT_A] = 0;
	p->ibf[PORT_A] = 0;
	p->obf[PORT_A] = 1;
	p->inte[PORT_A] = 0;
	p->inte1 = 0;
	p->inte2 = 0;
	if (port_mode(p, PORT_A) == MODE_OUTPUT) {
		out_pa(p, p->output[PORT_A]);
		p->ibf[PORT_A] = 1;
	} else
		out_pa(p, p->tri_pa);
	p->output[PORT_B] = 0;
	p->input[PORT_B] = 0;
	p->ibf[PORT_B] = 0;
	p->obf[PORT_B] = 1;
	p->inte[PORT_B] = 0;
	out_pb(p, port_mode(p, PORT_B) == MODE_OUTPUT ? p->output[PORT_B] : p->tri_pb);
	p->output[PORT_C] = 0;
	p->input[PORT_C] = 0;
	output_pc(p);
}

static void set_pc_bit(i8255_t *p, int bit, int state)
{
	p->output[PORT_C] = (uint8_t)((p->output[PORT_C] & ~(1 << bit)) | (state << bit));
	switch (group_mode(p, GROUP_A)) {
	case MODE_1:
		if (port_mode(p, PORT_A) == MODE_OUTPUT) {
			switch (bit) {
			case 3: set_intr(p, PORT_A, state); break;
			case 6: set_inte(p, PORT_A, state); break;
			case 7: set_obf(p, PORT_A, state); break;
			}
		} else {
			switch (bit) {
			case 3: set_intr(p, PORT_A, state); break;
			case 4: set_inte(p, PORT_A, state); break;
			case 5: set_ibf(p, PORT_A, state); break;
			}
		}
		break;
	case MODE_2:
		switch (bit) {
		case 3: set_intr(p, PORT_A, state); break;
		case 4: set_inte2(p, state); break;
		case 5: set_ibf(p, PORT_A, state); break;
		case 6: set_inte1(p, state); break;
		case 7: set_obf(p, PORT_A, state); break;
		}
		break;
	}
	if (group_mode(p, GROUP_B) == MODE_1) {
		switch (bit) {
		case 0: set_intr(p, PORT_B, state); break;
		case 1:
			if (port_mode(p, PORT_B) == MODE_OUTPUT)
				set_obf(p, PORT_B, state);
			else
				set_ibf(p, PORT_B, state);
			break;
		case 2: set_inte(p, PORT_B, state); break;
		}
	}
	output_pc(p);
}

void i8255_reset(i8255_t *p)
{
	p->intr[0] = p->intr[1] = 0;
	set_mode(p, 0x9b);
}

uint8_t i8255_read(i8255_t *p, int offset)
{
	switch (offset & 3) {
	case PORT_A:
		switch (group_mode(p, GROUP_A)) {
		case MODE_0: return read_mode0(p, PORT_A);
		case MODE_1: return read_mode1(p, PORT_A);
		default: return read_mode2(p);
		}
	case PORT_B:
		return group_mode(p, GROUP_B) == MODE_0 ? read_mode0(p, PORT_B) : read_mode1(p, PORT_B);
	case PORT_C:
		return read_pc(p);
	default:
		return p->control;
	}
}

void i8255_write(i8255_t *p, int offset, uint8_t data)
{
	switch (offset & 3) {
	case PORT_A:
		switch (group_mode(p, GROUP_A)) {
		case MODE_0: write_mode01(p, PORT_A, data, 0); break;
		case MODE_1: write_mode01(p, PORT_A, data, 1); break;
		default: write_mode2(p, data); break;
		}
		break;
	case PORT_B:
		write_mode01(p, PORT_B, data, group_mode(p, GROUP_B) == MODE_1);
		break;
	case PORT_C:
		p->output[PORT_C] = data;
		output_pc(p);
		break;
	default:
		if (data & CONTROL_MODE_SET)
			set_mode(p, data);
		else
			set_pc_bit(p, (data >> 1) & 7, data & 1);
		break;
	}
}

void i8255_pc2_w(i8255_t *p, int state)
{
	if (group_mode(p, GROUP_B) != MODE_1)
		return;
	if (port_mode(p, PORT_B) == MODE_OUTPUT) {
		if (!p->obf[PORT_B] && !state)
			set_obf(p, PORT_B, 1);
	} else if (!state) {
		p->input[PORT_B] = in_port(p, PORT_B);
		set_ibf(p, PORT_B, 1);
	}
}

void i8255_pc4_w(i8255_t *p, int state)
{
	if ((group_mode(p, GROUP_A) == MODE_2 || (group_mode(p, GROUP_A) == MODE_1 && port_mode(p, PORT_A) == MODE_INPUT)) && !state) {
		p->input[PORT_A] = in_port(p, PORT_A);
		set_ibf(p, PORT_A, 1);
	}
}

void i8255_pc6_w(i8255_t *p, int state)
{
	if ((group_mode(p, GROUP_A) == MODE_2 || (group_mode(p, GROUP_A) == MODE_1 && port_mode(p, PORT_A) == MODE_OUTPUT)) &&
	    !p->obf[PORT_A] && !state)
		set_obf(p, PORT_A, 1);
}
