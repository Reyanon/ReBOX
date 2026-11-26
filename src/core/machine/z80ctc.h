/*  Zilog Z80 CTC (Z8430): four counter / timer channels, their interrupt logic and the daisy chain's view of them.
    Ported from MAME src/devices/machine/z80ctc.cpp (BSD-3-Clause, copyright-holders: Wilbert Pol; based on the
    original version by Tatsuyuki Satoh).
    Time base: the CTC's clock (the CPU's on every board so far). MAME's per-channel timers are kept as clocks to the
    next zero count; the board advances the chip (z80ctc_advance) and asks for the next event (z80ctc_next) so that a
    CPU runs no further than the next timer interrupt. The ZC/TO pulse is one call with 1 and one with 0 at once (MAME:
    1 clock apart). The daisy chain is left to the board (z80ctc_irq_state / ack / reti, MAME's z80daisy_* calls).
    Save state: the struct from Z80CTC_STATE_OFFSET is plain data.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_Z80CTC_H
#define RBX_Z80CTC_H

#include <stddef.h>
#include <stdint.h>

#define Z80CTC_DAISY_INT 0x01   // (MAME Z80_DAISY_INT: an interrupt is pending)
#define Z80CTC_DAISY_IEO 0x02   // (MAME Z80_DAISY_IEO: the interrupt is being serviced)

typedef struct {
	uint16_t mode, tconst, down;
	uint8_t extclk, int_state;
	uint8_t timer_on, pad_;
	int32_t remaining;   // clocks to the timer's next expiry (timer_on)
	int32_t period;      // its reload (0: one shot)
} z80ctc_channel_t;

typedef struct {
	// configuration (not saved)
	void *user;
	void (*intr)(void *user, int state);            // the INT output (NULL: none)
	void (*zc[4])(void *user, int state);           // ZC/TO outputs (NULL: none); channel 3 has none on the chip
	// state (saved: from here)
	uint8_t vector, pad_[3];
	z80ctc_channel_t ch[4];
} z80ctc_t;

#define Z80CTC_STATE_OFFSET offsetof(z80ctc_t, vector)
#define Z80CTC_STATE_SIZE (sizeof(z80ctc_t) - offsetof(z80ctc_t, vector))

void z80ctc_reset(z80ctc_t *c);
uint8_t z80ctc_read(z80ctc_t *c, int offset);
void z80ctc_write(z80ctc_t *c, int offset, uint8_t data);
void z80ctc_trg(z80ctc_t *c, int ch, int state);   // the CLK/TRG inputs (MAME trg0..trg3)
// run the timers for that many clocks (their expiries in order); clocks to the next expiry (INT32_MAX: none)
void z80ctc_advance(z80ctc_t *c, int32_t clocks);
int32_t z80ctc_next(const z80ctc_t *c);
// the daisy chain (MAME z80daisy_irq_state / irq_ack / irq_reti)
int z80ctc_irq_state(const z80ctc_t *c);
int z80ctc_irq_ack(z80ctc_t *c);       // the vector; the channel goes to IEO
uint8_t z80ctc_irq_vector(const z80ctc_t *c);   // the vector an acknowledge now would give (no state change)
void z80ctc_irq_reti(z80ctc_t *c);

#endif
