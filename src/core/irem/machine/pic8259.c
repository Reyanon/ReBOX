/*  Intel 8259 / NEC uPD71059C (pic8259.h). Ported from MAME src/devices/machine/pic8259.cpp (BSD-3-Clause,
    copyright-holders: Wilbert Pol).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "pic8259.h"

enum { ST_ICW1, ST_ICW2, ST_ICW3, ST_ICW4, ST_READY };

static void tick(pic8259_t *p)   // (irq_timer_tick)
{
	for (int n = 0, irq = p->prio; n < 8; n++, irq = (irq + 1) & 7) {
		const uint8_t mask = (uint8_t)(1 << irq);
		if (p->isr & mask)   // (in service; no cascade here)
			break;
		if (p->state == ST_READY && (p->irr & mask) && !(p->imr & mask)) {
			p->current_level = (int8_t)irq;
			p->out_int(1);
			return;
		}
	}
	p->current_level = -1;
	p->out_int(0);
}

void pic8259_reset(pic8259_t *p)
{
	void (*f)(int) = p->out_int;
	for (size_t i = 0; i < PIC8259_STATE_SIZE; i++)
		((uint8_t *)p)[i] = 0;
	p->out_int = f;
	p->state = ST_READY;
	p->current_level = -1;
}

void pic8259_set_irq(pic8259_t *p, int irq, int state)
{
	const uint8_t mask = (uint8_t)(1 << irq);
	if (state && !(p->irq_lines & mask)) {
		p->irr |= mask;
		p->irq_lines |= mask;
	} else if (!state && (p->irq_lines & mask)) {
		p->irq_lines &= (uint8_t)~mask;
		p->irr &= (uint8_t)~mask;
	}
	if (p->inta_sequence == 0)
		tick(p);
}

uint8_t pic8259_ack(pic8259_t *p)
{
	if (p->is_x86) {
		if (p->current_level != -1) {
			const uint8_t mask = (uint8_t)(1 << p->current_level);
			const uint8_t v = (uint8_t)(p->current_level + p->base);
			if (!p->level_trig_mode)
				p->irr &= (uint8_t)~mask;
			if (!p->auto_eoi)
				p->isr |= mask;
			tick(p);
			return v;
		}
		return (uint8_t)(p->base + 7);
	}
	// (the 8080 / 85's three bytes; a NEC takes one: not reached by the Irem programs, which set x86 mode)
	if (p->inta_sequence == 0) {
		if (p->current_level != -1) {
			const uint8_t mask = (uint8_t)(1 << p->current_level);
			if (!p->level_trig_mode)
				p->irr &= (uint8_t)~mask;
			p->isr |= mask;
		}
		p->inta_sequence = 1;
		return 0xcd;
	}
	if (p->inta_sequence == 1) {
		p->inta_sequence = 2;
		return (uint8_t)(p->vector_addr_low + ((p->current_level & 7) << (3 - p->vector_size)));
	}
	p->inta_sequence = 0;
	if (p->auto_eoi && p->current_level != -1)
		p->isr &= (uint8_t)~(1 << p->current_level);
	tick(p);
	return p->vector_addr_high;
}

uint8_t pic8259_read(pic8259_t *p, int offset)
{
	uint8_t data = 0;
	if (offset == 0) {
		if (p->ocw3 & 0x04) {   // (polling mode)
			if (p->current_level != -1) {
				data = (uint8_t)(0x80 | p->current_level);
				if (!p->level_trig_mode)
					p->irr &= (uint8_t)~(1 << p->current_level);
				if (!p->auto_eoi)
					p->isr |= (uint8_t)(1 << p->current_level);
				tick(p);
			}
			p->ocw3 &= 0xfb;
		} else
			data = (p->ocw3 & 1) ? (uint8_t)(p->isr & ~p->imr) : p->irr;
	} else
		data = p->imr;
	return data;
}

void pic8259_write(pic8259_t *p, int offset, uint8_t data)
{
	if (offset == 0) {
		if (data & 0x10) {   // (ICW1)
			p->imr = 0;
			p->isr = 0;
			p->slave = 0;
			p->level_trig_mode = (data & 0x08) ? 1 : 0;
			p->vector_size = (data & 0x04) ? 1 : 0;
			p->cascade = (data & 0x02) ? 0 : 1;
			p->icw4_needed = (data & 0x01) ? 1 : 0;
			p->vector_addr_low = data & 0xe0;
			p->state = ST_ICW2;
			p->current_level = -1;
			p->inta_sequence = 0;
			p->irr = p->level_trig_mode ? p->irq_lines : 0;
			p->out_int(0);
		} else if (p->state == ST_READY) {
			if ((data & 0x98) == 0x08) {   // (OCW3)
				if (data & 2)
					p->ocw3 = (uint8_t)((p->ocw3 & 0xfe) | (data & 1));
				if (data & 4)
					p->ocw3 |= 0x04;
				if (data & 0x40)
					p->ocw3 = (uint8_t)((p->ocw3 & 0xdf) | (data & 0x20));
			} else if ((data & 0x18) == 0) {   // (OCW2)
				int n = data & 7;
				uint8_t mask = (uint8_t)(1 << n);
				switch (data & 0xe0) {
				case 0x00:
					p->prio = 0;
					break;
				case 0x20:
					for (n = 0, mask = (uint8_t)(1 << p->prio); n < 8; n++, mask = (uint8_t)(mask << 1 | mask >> 7))
						if (p->isr & mask) {
							p->isr &= (uint8_t)~mask;
							break;
						}
					break;
				case 0x60:
					if (p->isr & mask)
						p->isr &= (uint8_t)~mask;
					break;
				case 0x80:
					p->prio = (p->prio + 1) & 7;
					break;
				case 0xa0:
					for (n = 0, mask = (uint8_t)(1 << p->prio); n < 8; n++, mask = (uint8_t)(mask << 1 | mask >> 7))
						if (p->isr & mask) {
							p->isr &= (uint8_t)~mask;
							p->prio = (p->prio + 1) & 7;
							break;
						}
					break;
				case 0xc0:
					p->prio = (uint8_t)((n + 1) & 7);
					break;
				case 0xe0:
					if (p->isr & mask) {
						p->isr &= (uint8_t)~mask;
						p->prio = (uint8_t)((n + 1) & 7);
					}
					break;
				}
			}
		}
	} else {
		switch (p->state) {
		case ST_ICW2:
			p->base = data & 0xf8;
			p->vector_addr_high = data;
			p->state = p->cascade ? ST_ICW3 : p->icw4_needed ? ST_ICW4 : ST_READY;
			break;
		case ST_ICW3:
			p->slave = data;
			p->state = p->icw4_needed ? ST_ICW4 : ST_READY;
			break;
		case ST_ICW4:
			p->nested = (data & 0x10) ? 1 : 0;
			p->mode = (data >> 2) & 3;
			p->auto_eoi = (data & 0x02) ? 1 : 0;
			p->is_x86 = data & 1;
			p->state = ST_READY;
			break;
		case ST_READY:
			p->imr = data;
			break;
		}
	}
	tick(p);
}
