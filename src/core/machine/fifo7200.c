/*  IDT7200-series FIFO, ported from MAME 0.289 src/devices/machine/7200fifo.cpp (BSD-3-Clause,
    copyright-holders: hap). Not done (as MAME): retransmit (RT pin), cascaded width expansion.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "fifo7200.h"

#include <string.h>

void fifo7200_init(fifo7200_t *f, int ram_size)
{
	memset(f, 0, sizeof(*f));
	f->ram_size = ram_size > FIFO7200_MAX ? FIFO7200_MAX : ram_size;
	fifo7200_reset(f);
}

// device_reset: the master reset
void fifo7200_reset(fifo7200_t *f)
{
	memset(f->buffer, 0, sizeof(f->buffer));
	f->read_ptr = 0;
	f->write_ptr = 0;
	f->ef = 1;
	f->ff = 0;
	f->hf = 0;
}

static int fifo_used(const fifo7200_t *f)
{
	int ret = f->ram_size;
	if (!f->ff) {
		if (f->write_ptr >= f->read_ptr)
			ret = f->write_ptr - f->read_ptr;
		else
			ret = f->ram_size + f->write_ptr - f->read_ptr;
	}
	return ret;
}

void fifo7200_write(fifo7200_t *f, uint16_t data)
{
	if (f->ff)   // (overflow)
		return;
	f->buffer[f->write_ptr] = data & 0x1ff;
	f->write_ptr = (f->write_ptr + 1) % f->ram_size;
	// update flags
	if (f->ef)
		f->ef = 0;
	else if (f->read_ptr == f->write_ptr)
		f->ff = 1;
	else if (!f->hf && (fifo_used(f) >= f->ram_size / 2))
		f->hf = 1;
}

uint16_t fifo7200_read(fifo7200_t *f)
{
	if (f->ef)   // (underflow)
		return 0x1ff;
	const uint16_t ret = f->buffer[f->read_ptr];
	f->read_ptr = (f->read_ptr + 1) % f->ram_size;
	// update flags
	if (f->ff)
		f->ff = 0;
	else if (f->read_ptr == f->write_ptr)
		f->ef = 1;
	else if (f->hf && (fifo_used(f) < f->ram_size / 2))
		f->hf = 0;
	return ret;
}
