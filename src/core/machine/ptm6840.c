/*  Motorola 6840 PTM, ported to C from MAME src/devices/machine/6840ptm.cpp (BSD-3-Clause, copyright-holders: James
    Wallace). See ptm6840.h for the time base.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "ptm6840.h"

#include <stdint.h>
#include <string.h>

enum { CTRL1 = 0, CTRL2 = 1, STATUS = 1, MSBBUF1 = 2, LSB1 = 3, MSBBUF2 = 4, LSB2 = 5, MSBBUF3 = 6, LSB3 = 7 };
#define RESET_TIMERS    0x01
#define CR1_SELECT      0x01
#define T3_PRESCALE_EN  0x01
#define INTERNAL_CLK_EN 0x02
#define COUNT_MODE_8BIT 0x04
#define INTERRUPT_EN    0x40
#define COUNT_OUT_EN    0x80
#define TIMER1_IRQ      0x01
#define TIMER2_IRQ      0x02
#define TIMER3_IRQ      0x04
#define ANY_IRQ         0x80

static inline int one_shot(const ptm6840_t *p, int idx) { return p->mode[idx] == 4 || p->mode[idx] == 6; }
static inline void out_w(ptm6840_t *p, int idx, int state) { if (p->out[idx]) p->out[idx](p->user, state); }

static void update_interrupts(ptm6840_t *p)
{
	const int new_state = ((p->status & TIMER1_IRQ) && (p->control[0] & INTERRUPT_EN)) ||
	                      ((p->status & TIMER2_IRQ) && (p->control[1] & INTERRUPT_EN)) ||
	                      ((p->status & TIMER3_IRQ) && (p->control[2] & INTERRUPT_EN));
	if (new_state != p->irq_state) {
		p->irq_state = (uint8_t)new_state;
		if (new_state)
			p->status |= ANY_IRQ;
		else
			p->status &= (uint8_t)~ANY_IRQ;
		if (p->irq)
			p->irq(p->user, new_state);
	}
}

void ptm6840_reset(ptm6840_t *p)
{
	p->t3_divisor = 1;
	p->t3_scaler = 0;
	p->status = 0;
	p->status_read_since_int = 0;
	for (int i = 0; i < 3; i++) {
		p->control[i] = 0;
		p->counter[i] = 0xffff;
		p->latch[i] = 0xffff;
		p->disable_on[i] = 0;
		p->timer_on[i] = 0;
		p->output[i] = 0;
		p->clk[i] = 0;
		p->single_fired[i] = 0;
		p->enabled[i] = 0;
		p->mode[i] = 0;
	}
	p->control[0] = 1;
	p->irq_state = 0;
	if (p->irq)
		p->irq(p->user, 0);
	for (int i = 0; i < 3; i++)
		out_w(p, i, 0);
}

// the counter's value now (compute_counter): an internal clock's from its timer
static int compute_counter(const ptm6840_t *p, int idx)
{
	if (!p->enabled[idx])
		return p->counter[idx];
	if (p->control[0] & RESET_TIMERS) {
		if (p->control[idx] & COUNT_MODE_8BIT)
			return (p->latch[idx] >> 8) * ((p->latch[idx] & 0xff) + 1);
		return p->latch[idx];
	}
	int32_t remaining;
	if (p->timer_on[idx])
		remaining = p->remaining[idx];
	else if (p->disable_on[idx])
		remaining = p->disable_time[idx];
	else
		return p->counter[idx];
	// (as_ticks of the channel's clock: E, or E / 8 for a prescaled timer 3)
	return idx == 2 && p->t3_divisor == 8 ? remaining >> 3 : remaining;
}

static void reload_counter(ptm6840_t *p, int idx)
{
	const int os = one_shot(p, idx);
	p->counter[idx] = p->latch[idx];
	int32_t count = (int32_t)p->counter[idx] + 1;
	if (p->control[idx] & COUNT_MODE_8BIT) {
		const int lsb = p->latch[idx] & 0xff, msb = p->latch[idx] >> 8;
		count = !p->output[idx] ? (lsb + 1) * msb : lsb + 1;
	} else if (os)
		count = !p->output[idx] ? 1 : p->counter[idx];
	if (!(p->control[idx] & INTERNAL_CLK_EN)) {   // (an external clock: its edges count, no timer)
		p->enabled[idx] = 0;
		p->timer_on[idx] = 0;
		return;
	}
	p->enabled[idx] = 1;
	int32_t duration = count;
	if (idx == 2)
		duration *= p->t3_divisor;
	if (duration < 1)   // (a zero count expires at once: one E clock here)
		duration = 1;
	const int gated = (!os && p->gate[idx]) || (p->control[0] & RESET_TIMERS);
	if (gated) {
		p->disable_time[idx] = duration;
		p->disable_on[idx] = 1;
		p->timer_on[idx] = 0;
	} else {
		p->remaining[idx] = duration;
		p->timer_on[idx] = 1;
	}
}

static void state_changed(ptm6840_t *p, int idx)
{
	const int os = one_shot(p, idx);
	const int dual_8bit = (p->control[idx] & COUNT_MODE_8BIT) != 0;
	const int end_of_cycle = (!dual_8bit && !os) || p->output[idx];
	if (end_of_cycle) {
		p->status |= (uint8_t)(1 << idx);
		p->status_read_since_int &= (uint8_t)~(1 << idx);
		update_interrupts(p);
	}
	const int enable_output = (p->control[idx] & COUNT_OUT_EN) != 0;
	switch (p->mode[idx]) {
	case 0: case 2:
		p->output[idx] = !p->output[idx];
		out_w(p, idx, enable_output ? p->output[idx] : 0);
		break;
	case 4: case 6:
		p->output[idx] = !p->output[idx];
		if (!p->single_fired[idx]) {
			if (enable_output)
				out_w(p, idx, p->output[idx]);
			if (!p->output[idx])
				p->single_fired[idx] = 1;
		} else
			out_w(p, idx, 0);
		break;
	}
	p->enabled[idx] = 0;
	p->timer_on[idx] = 0;
	reload_counter(p, idx);
}

static void deduct_from_counter(ptm6840_t *p, int idx)
{
	if (p->control[idx] & COUNT_MODE_8BIT) {
		uint16_t msb = p->counter[idx] >> 8, lsb = p->counter[idx] & 0xff;
		lsb--;
		int timed_out = 0;
		if (lsb == 0xffff) {
			lsb = (uint16_t)((p->latch[idx] & 0xff) + 1);
			msb--;
			if (msb == 0xffff) {
				timed_out = 1;
				state_changed(p, idx);
			} else if (msb == 0) {
				msb = (uint16_t)((p->latch[idx] >> 8) + 1);
				state_changed(p, idx);
			}
		}
		if (!timed_out)
			p->counter[idx] = (uint16_t)((msb << 8) + lsb);
	} else {
		p->counter[idx]--;
		if ((one_shot(p, idx) && !p->output[idx]) || p->counter[idx] == 0xffff)
			state_changed(p, idx);
	}
}

// the clock source changed (update_expiration_for_clock_source; external clocks have no frequency here)
static void clock_source_changed(ptm6840_t *p, int idx, int changed_to_external)
{
	if (p->control[0] & RESET_TIMERS)
		return;
	if (changed_to_external)
		p->control[idx] |= INTERNAL_CLK_EN;
	const int updated = compute_counter(p, idx);
	if (changed_to_external)
		p->control[idx] &= (uint8_t)~INTERNAL_CLK_EN;
	if (!(p->control[idx] & INTERNAL_CLK_EN)) {
		if ((p->control[idx] & COUNT_MODE_8BIT) && !p->output[idx]) {
			const int lsb = p->latch[idx] & 0xff, msb = p->latch[idx] >> 8;
			const uint8_t count_lsb = (uint8_t)(msb ? updated % msb : 0);
			const uint8_t count_msb = (uint8_t)((updated - count_lsb) / (lsb + 1));
			p->counter[idx] = (uint16_t)((count_msb << 8) | count_lsb);
		} else
			p->counter[idx] = (uint16_t)updated;
		p->enabled[idx] = 0;
		p->timer_on[idx] = 0;
		return;
	}
	int32_t duration = updated;
	if (idx == 2)
		duration *= p->t3_divisor;
	if (duration < 1)
		duration = 1;
	p->enabled[idx] = 1;
	if (!one_shot(p, idx) && p->gate[idx]) {
		p->timer_on[idx] = 0;
		p->disable_time[idx] = duration;
		p->disable_on[idx] = 1;
	} else {
		p->remaining[idx] = duration;
		p->timer_on[idx] = 1;
	}
}

uint8_t ptm6840_read(ptm6840_t *p, int offset)
{
	switch (offset & 7) {
	case CTRL1:
		return 0;
	case STATUS:
		p->status_read_since_int |= p->status & 0x07;
		return p->status;
	case LSB1: case LSB2: case LSB3:
		return p->lsb_buffer;
	default: {   // MSBBUF1..3
		const int idx = ((offset & 7) - 2) / 2;
		const uint16_t result = (uint16_t)compute_counter(p, idx);
		if (p->status_read_since_int & (1 << idx)) {
			p->status &= (uint8_t)~(1 << idx);
			update_interrupts(p);
		}
		p->lsb_buffer = (uint8_t)result;
		return (uint8_t)(result >> 8);
	}
	}
}

void ptm6840_write(ptm6840_t *p, int offset, uint8_t data)
{
	switch (offset & 7) {
	case CTRL1: case CTRL2: {
		const int idx = (offset & 7) == 1 ? 1 : (p->control[1] & CR1_SELECT) ? 0 : 2;
		const uint8_t diffs = data ^ p->control[idx];
		p->mode[idx] = (data >> 3) & 0x07;
		p->control[idx] = data;
		p->t3_divisor = (p->control[2] & T3_PRESCALE_EN) ? 8 : 1;
		if (diffs & INTERRUPT_EN)
			update_interrupts(p);
		if (!(p->control[idx] & COUNT_OUT_EN))
			out_w(p, idx, 0);
		if (idx == 0 && (diffs & RESET_TIMERS)) {
			if (data & RESET_TIMERS) {   // (holding the timers in reset)
				p->status = 0;
				p->status_read_since_int = 0;
				update_interrupts(p);
				for (int i = 0; i < 3; i++) {
					p->timer_on[i] = 0;
					p->enabled[i] = 0;
					reload_counter(p, i);
					p->output[i] = 0;
					out_w(p, i, 0);
				}
			} else {   // (releasing them)
				for (int i = 0; i < 3; i++) {
					p->single_fired[i] = 0;
					reload_counter(p, i);
					if (p->disable_on[i] && !p->timer_on[i] && (p->control[i] & INTERNAL_CLK_EN)) {
						p->remaining[i] = p->disable_time[i];
						p->timer_on[i] = 1;
						p->disable_on[i] = 0;
					}
				}
			}
			p->status = 0;
			update_interrupts(p);
		}
		if (diffs & INTERNAL_CLK_EN)
			clock_source_changed(p, idx, !(p->control[idx] & INTERNAL_CLK_EN));
		break;
	}
	case MSBBUF1: case MSBBUF2: case MSBBUF3:
		p->msb_buffer = data;
		break;
	default: {   // LSB1..3: the latch, and a reload unless the counter waits for its gate
		const int idx = ((offset & 7) - 3) / 2;
		p->latch[idx] = (uint16_t)((p->msb_buffer << 8) | data);
		p->status &= (uint8_t)~(1 << idx);
		update_interrupts(p);
		if (!(p->control[idx] & 0x10) || (p->control[0] & RESET_TIMERS))
			reload_counter(p, idx);
		break;
	}
	}
}

void ptm6840_set_gate(ptm6840_t *p, int idx, int state)
{
	state = state != 0;
	if (state == 0 && p->gate[idx]) {
		if (!(p->control[0] & RESET_TIMERS)) {
			p->single_fired[idx] = 0;
			p->output[idx] = 0;
			reload_counter(p, idx);
		}
		if (p->disable_on[idx] && (p->control[idx] & INTERNAL_CLK_EN)) {
			p->remaining[idx] = p->disable_time[idx];
			p->timer_on[idx] = 1;
			p->disable_on[idx] = 0;
		}
	} else if (state == 1 && !p->gate[idx] && !one_shot(p, idx)) {
		p->disable_time[idx] = p->remaining[idx];
		p->disable_on[idx] = p->timer_on[idx];   // (a stopped timer: never)
		p->timer_on[idx] = 0;
	}
	p->gate[idx] = (uint8_t)state;
}

void ptm6840_set_clock(ptm6840_t *p, int idx, int state)
{
	state = state != 0;
	if (p->clk[idx] == state)
		return;
	const int rising = !p->clk[idx] && state;
	p->clk[idx] = (uint8_t)state;
	if (rising)
		return;
	const int use_external = !(p->control[idx] & INTERNAL_CLK_EN);
	const int running = !(p->control[0] & RESET_TIMERS);
	const int gated = !one_shot(p, idx) && p->gate[idx];
	if (use_external && running && !gated) {
		if (idx == 2) {
			if (++p->t3_scaler >= p->t3_divisor) {
				deduct_from_counter(p, idx);
				p->t3_scaler = 0;
			}
		} else
			deduct_from_counter(p, idx);
	}
}

int32_t ptm6840_next(const ptm6840_t *p)
{
	int32_t n = INT32_MAX;
	for (int i = 0; i < 3; i++)
		if (p->timer_on[i] && p->remaining[i] < n)
			n = p->remaining[i];
	return n;
}

void ptm6840_advance(ptm6840_t *p, int32_t eclocks)
{
	while (eclocks > 0) {
		int first = -1;
		int32_t t = eclocks;
		for (int i = 0; i < 3; i++)
			if (p->timer_on[i] && p->remaining[i] <= t) {
				t = p->remaining[i];
				first = i;
			}
		for (int i = 0; i < 3; i++)
			if (p->timer_on[i])
				p->remaining[i] -= t;
		eclocks -= t;
		if (first < 0)
			break;
		p->timer_on[first] = 0;
		state_changed(p, first);
	}
}
