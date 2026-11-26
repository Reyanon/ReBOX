/*  Motorola 6840 PTM (programmable timer module), ported to C from MAME src/devices/machine/6840ptm.cpp (BSD-3-Clause,
    copyright-holders: James Wallace). Three 16-bit counters (or dual 8-bit), internal (E) or external clocks, gates,
    the continuous and one-shot modes, outputs, the status register and its interrupt.
    Time base: the E clock. MAME's timers are kept as E clocks to the next expiry; the board advances the chip
    (ptm6840_advance) and asks for the next event (ptm6840_next). External clocks only by their edges (set_clock):
    MAME's fixed external frequencies (set_ext_clock) are not modelled (no board here has one).
    Save state: the struct from PTM6840_STATE_OFFSET is plain data.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_PTM6840_H
#define RBX_PTM6840_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
	// configuration (not saved)
	void *user;
	void (*irq)(void *user, int state);
	void (*out[3])(void *user, int state);
	// state (saved: from here)
	uint8_t control[3], mode[3], output[3], gate[3], clk[3], single_fired[3], enabled[3], timer_on[3], disable_on[3];
	uint8_t status, status_read_since_int, lsb_buffer, msb_buffer, irq_state, t3_divisor, t3_scaler, pad_;
	uint16_t counter[3], latch[3];
	int32_t remaining[3];      // E clocks to the timer's expiry (timer_on)
	int32_t disable_time[3];   // E clocks the timer had left when it was gated (disable_on)
} ptm6840_t;

#define PTM6840_STATE_OFFSET offsetof(ptm6840_t, control)
#define PTM6840_STATE_SIZE (sizeof(ptm6840_t) - offsetof(ptm6840_t, control))

void ptm6840_reset(ptm6840_t *p);
uint8_t ptm6840_read(ptm6840_t *p, int offset);
void ptm6840_write(ptm6840_t *p, int offset, uint8_t data);
void ptm6840_set_gate(ptm6840_t *p, int idx, int state);
void ptm6840_set_clock(ptm6840_t *p, int idx, int state);   // (MAME set_c1 .. set_c3: idx 0..2)
void ptm6840_advance(ptm6840_t *p, int32_t eclocks);
int32_t ptm6840_next(const ptm6840_t *p);   // E clocks to the next expiry (INT32_MAX: none)

#endif
