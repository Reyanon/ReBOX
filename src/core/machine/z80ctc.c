/*  Zilog Z80 CTC (Z8430), ported from MAME src/devices/machine/z80ctc.cpp (BSD-3-Clause, copyright-holders: Wilbert
    Pol; based on the original version by Tatsuyuki Satoh). See z80ctc.h for the time base.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "z80ctc.h"

#include <stdint.h>
#include <string.h>

// the bits of the incoming commands
#define INTERRUPT         0x80
#define INTERRUPT_ON      0x80
#define MODE              0x40
#define MODE_COUNTER      0x40
#define PRESCALER         0x20
#define EDGE              0x10
#define TRIGGER           0x08
#define CONSTANT          0x04
#define RESET             0x02
#define CONTROL           0x01
#define WAITING_FOR_TRIG  0x100   // (extra bit kept in mode)

static void interrupt_check(z80ctc_t *c)
{
	if (c->intr)
		c->intr(c->user, (z80ctc_irq_state(c) & Z80CTC_DAISY_INT) ? 1 : 0);
}

int z80ctc_irq_state(const z80ctc_t *c)
{
	int state = 0;
	for (int ch = 0; ch < 4; ch++) {
		// a request being serviced blocks the ones after it
		if (c->ch[ch].int_state & Z80CTC_DAISY_IEO) {
			state |= Z80CTC_DAISY_IEO;
			break;
		}
		state |= c->ch[ch].int_state;
	}
	return state;
}

uint8_t z80ctc_irq_vector(const z80ctc_t *c)
{
	for (int ch = 0; ch < 4; ch++)
		if (c->ch[ch].int_state & Z80CTC_DAISY_INT)
			return (uint8_t)(c->vector + ch * 2);
	return c->vector;
}

int z80ctc_irq_ack(z80ctc_t *c)
{
	for (int ch = 0; ch < 4; ch++) {
		z80ctc_channel_t *h = &c->ch[ch];
		if (h->int_state & Z80CTC_DAISY_INT) {
			h->int_state = Z80CTC_DAISY_IEO;
			interrupt_check(c);
			return c->vector + ch * 2;
		}
	}
	return c->vector;
}

void z80ctc_irq_reti(z80ctc_t *c)
{
	for (int ch = 0; ch < 4; ch++) {
		z80ctc_channel_t *h = &c->ch[ch];
		if (h->int_state & Z80CTC_DAISY_IEO) {
			h->int_state &= ~Z80CTC_DAISY_IEO;
			interrupt_check(c);
			return;
		}
	}
}

void z80ctc_reset(z80ctc_t *c)
{
	c->vector = 0;
	for (int ch = 0; ch < 4; ch++) {
		z80ctc_channel_t *h = &c->ch[ch];
		h->mode = RESET;
		h->tconst = 0x100;
		h->timer_on = 0;
		h->int_state = 0;
	}
	interrupt_check(c);
}

// the channel's period in clocks (timer mode); counter mode and reset: none (MAME's channel clock is 0)
static int32_t period(const z80ctc_channel_t *h)
{
	if ((h->mode & RESET) || (h->mode & MODE) == MODE_COUNTER)
		return 0;
	return (int32_t)(((h->mode & PRESCALER) ? 256 : 16) * h->tconst);
}

static void timer_start(z80ctc_channel_t *h)
{
	const int32_t p = period(h);
	h->timer_on = p != 0;
	h->remaining = p;
	h->period = p;
}

static uint8_t ch_read(const z80ctc_channel_t *h)
{
	if (!h->timer_on || (h->mode & WAITING_FOR_TRIG))
		return (uint8_t)h->down;
	const int32_t unit = (h->mode & PRESCALER) ? 256 : 16;
	// (MAME: remaining / period + 1; the remaining time is never 0 here)
	return (uint8_t)(h->remaining / unit + 1);
}

uint8_t z80ctc_read(z80ctc_t *c, int offset)
{
	return ch_read(&c->ch[offset & 3]);
}

static void zero_count(z80ctc_t *c, int ch)
{
	z80ctc_channel_t *h = &c->ch[ch];
	if ((h->mode & INTERRUPT) == INTERRUPT_ON) {
		h->int_state |= Z80CTC_DAISY_INT;
		interrupt_check(c);
	}
	if (c->zc[ch]) {   // (the ZC/TO pulse)
		c->zc[ch](c->user, 1);
		c->zc[ch](c->user, 0);
	}
	h->down = h->tconst;
}

void z80ctc_write(z80ctc_t *c, int offset, uint8_t data)
{
	const int index = offset & 3;
	z80ctc_channel_t *h = &c->ch[index];
	if (h->mode & CONSTANT) {   // the time constant
		h->tconst = data ? data : 0x100;
		h->mode &= ~(CONSTANT | RESET);
		if ((h->mode & MODE) == MODE_COUNTER || !(h->mode & TRIGGER))
			timer_start(h);
		else {   // waiting for the trigger (MAME's 1-clock timer of a clockless channel never fires)
			h->mode |= WAITING_FOR_TRIG;
			h->timer_on = 0;
		}
		h->down = h->tconst;
	} else if (!(data & CONTROL) && index == 0) {
		c->vector = data & 0xf8;
	} else if (data & CONTROL) {
		// (mode change without reset?)
		if ((h->mode & MODE) != MODE_COUNTER && (data & MODE) == MODE_COUNTER && !(data & RESET))
			h->timer_on = 0;
		if (data & RESET) {
			h->down = ch_read(h);
			h->timer_on = 0;
		}
		h->mode = data;
		// clearing this bit resets the interrupt state regardless of M1 activity
		if (!(data & INTERRUPT) && (h->int_state & Z80CTC_DAISY_INT)) {
			h->int_state &= ~Z80CTC_DAISY_INT;
			interrupt_check(c);
		}
	}
}

void z80ctc_trg(z80ctc_t *c, int ch, int state)
{
	z80ctc_channel_t *h = &c->ch[ch & 3];
	state = state != 0;
	if (state == h->extclk)
		return;
	h->extclk = (uint8_t)state;
	if (((h->mode & EDGE) && state) || (!(h->mode & EDGE) && !state)) {
		if ((h->mode & WAITING_FOR_TRIG) && (h->mode & MODE) != MODE_COUNTER)
			timer_start(h);
		h->mode &= ~WAITING_FOR_TRIG;
		if ((h->mode & MODE) == MODE_COUNTER) {
			h->down = (uint16_t)(h->down - 1);
			if (h->down == 0)
				zero_count(c, ch & 3);
		}
	}
}

int32_t z80ctc_next(const z80ctc_t *c)
{
	int32_t n = INT32_MAX;
	for (int ch = 0; ch < 4; ch++)
		if (c->ch[ch].timer_on && c->ch[ch].remaining < n)
			n = c->ch[ch].remaining;
	return n;
}

void z80ctc_advance(z80ctc_t *c, int32_t clocks)
{
	while (clocks > 0) {
		// the next expiry within the span (the earliest channel first, as MAME's scheduler)
		int first = -1;
		int32_t t = clocks;
		for (int ch = 0; ch < 4; ch++)
			if (c->ch[ch].timer_on && c->ch[ch].remaining <= t) {
				t = c->ch[ch].remaining;
				first = ch;
			}
		for (int ch = 0; ch < 4; ch++)
			if (c->ch[ch].timer_on)
				c->ch[ch].remaining -= t;
		clocks -= t;
		if (first < 0)
			break;
		z80ctc_channel_t *h = &c->ch[first];
		h->remaining = h->period;
		h->timer_on = h->period != 0;
		zero_count(c, first);
	}
}
