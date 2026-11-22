/*  Namco 56XX / 58XX / 59XX custom I/O chips. Ported from MAME 0.289 src/mame/namco/namcoio.cpp (BSD-3-Clause,
    copyright-holders: Nicola Salmoria, Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "namcoio.h"

#include <string.h>

#define IORAM_READ(o) (io->ram[o] & 0x0f)
#define IORAM_WRITE(o, d) (io->ram[o] = (uint8_t)((d) & 0x0f))

static inline uint8_t in_r(namcoio_t *io, int n) { return io->in ? io->in(io->ctx, n) : 0; }
static inline void out_w(namcoio_t *io, int n, uint8_t d)
{
	if (io->out)
		io->out(io->ctx, n, d);
}

void namcoio_init(namcoio_t *io, int type)
{
	memset(io, 0, sizeof(*io));
	io->type = type;
}

void namcoio_set_reset_line(namcoio_t *io, int state)
{
	io->reset = state ? 1 : 0;
	if (state) {
		// reset internal registers
		io->credits = 0;
		io->coins[0] = 0;
		io->coins_per_cred[0] = 1;
		io->creds_per_coin[0] = 1;
		io->coins[1] = 0;
		io->coins_per_cred[1] = 1;
		io->creds_per_coin[1] = 1;
		io->in_count = 0;
	}
}

int namcoio_read_reset_line(const namcoio_t *io) { return io->reset; }

void namcoio_reset(namcoio_t *io)
{
	memset(io->ram, 0, sizeof(io->ram));
	if (io->reset != 1) {
		namcoio_set_reset_line(io, 1);
		namcoio_set_reset_line(io, 0);
	}
}

uint8_t namcoio_read(namcoio_t *io, int offset)
{
	// RAM is 4-bit wide; Pac & Pal requires the | 0xf0 otherwise Easter egg doesn't work
	return (uint8_t)(0xf0 | io->ram[offset & 0x0f]);
}

void namcoio_write(namcoio_t *io, int offset, uint8_t data)
{
	io->ram[offset & 0x0f] = data & 0x0f;   // RAM is 4-bit wide
}

static void handle_coins(namcoio_t *io, int swap)
{
	int credit_add = 0, credit_sub = 0;
	int val = ~in_r(io, 0);   // pins 38-41
	int toggled = val ^ io->lastcoins;
	io->lastcoins = val;

	// check coin insertion
	if (val & toggled & 0x01) {
		io->coins[0]++;
		if (io->coins[0] >= (io->coins_per_cred[0] & 7)) {
			credit_add = io->creds_per_coin[0] - (io->coins_per_cred[0] >> 3);
			io->coins[0] -= io->coins_per_cred[0] & 7;
		} else if (io->coins_per_cred[0] & 8)
			credit_add = 1;
	}
	if (val & toggled & 0x02) {
		io->coins[1]++;
		if (io->coins[1] >= (io->coins_per_cred[1] & 7)) {
			credit_add = io->creds_per_coin[1] - (io->coins_per_cred[1] >> 3);
			io->coins[1] -= io->coins_per_cred[1] & 7;
		} else if (io->coins_per_cred[1] & 8)
			credit_add = 1;
	}
	if (val & toggled & 0x08)
		credit_add = 1;

	val = ~in_r(io, 3);   // pins 30-33
	toggled = val ^ io->lastbuttons;
	io->lastbuttons = val;

	// check start buttons, only if the game allows
	if (IORAM_READ(9) == 0) {   // (the other argument is IORAM_READ(10) = 1, meaning unknown)
		if (val & toggled & 0x04) {
			if (io->credits >= 1)
				credit_sub = 1;
		} else if (val & toggled & 0x08) {
			if (io->credits >= 2)
				credit_sub = 2;
		}
	}

	io->credits += credit_add - credit_sub;

	IORAM_WRITE(0 ^ swap, io->credits / 10);   // BCD credits
	IORAM_WRITE(1 ^ swap, io->credits % 10);   // BCD credits
	// these two are cleared by the host CPU in handshake fashion
	if (credit_add)
		IORAM_WRITE(2 ^ swap, credit_add);   // credit increment (coin inputs)
	if (credit_sub)
		IORAM_WRITE(3 ^ swap, credit_sub);   // credit decrement (start buttons)
	IORAM_WRITE(4, ~in_r(io, 1));   // pins 22-25
	int button = ((val & 0x05) << 1) | (val & toggled & 0x05);
	IORAM_WRITE(5, button);   // pins 30 & 32 normal and impulse
	IORAM_WRITE(6, ~in_r(io, 2));   // pins 26-29
	button = (val & 0x0a) | ((val & toggled & 0x0a) >> 1);
	IORAM_WRITE(7, button);   // pins 31 & 33 normal and impulse
}

// read the four inputs twice, with pin 13 (out 0) low and then high
static void read_dips(namcoio_t *io)
{
	out_w(io, 0, 0);   // set pin 13 = 0
	IORAM_WRITE(0, ~in_r(io, 0));   // pins 38-41, pin 13 = 0
	IORAM_WRITE(2, ~in_r(io, 1));   // pins 22-25, pin 13 = 0
	IORAM_WRITE(4, ~in_r(io, 2));   // pins 26-29, pin 13 = 0
	IORAM_WRITE(6, ~in_r(io, 3));   // pins 30-33, pin 13 = 0
	out_w(io, 0, 1);   // set pin 13 = 1
	IORAM_WRITE(1, ~in_r(io, 0));   // pins 38-41, pin 13 = 1
	IORAM_WRITE(3, ~in_r(io, 1));   // pins 22-25, pin 13 = 1
	IORAM_WRITE(5, ~in_r(io, 2));   // pins 26-29, pin 13 = 1
	IORAM_WRITE(7, ~in_r(io, 3));   // pins 30-33, pin 13 = 1
}

static void run_56xx(namcoio_t *io)
{
	switch (IORAM_READ(8)) {
	case 0:   // nop?
		break;
	case 1:   // read switch inputs
		IORAM_WRITE(0, ~in_r(io, 0));   // pins 38-41
		IORAM_WRITE(1, ~in_r(io, 1));   // pins 22-25
		IORAM_WRITE(2, ~in_r(io, 2));   // pins 26-29
		IORAM_WRITE(3, ~in_r(io, 3));   // pins 30-33
		out_w(io, 0, (uint8_t)IORAM_READ(9));    // output to pins 13-16 (motos, pacnpal, gaplus)
		out_w(io, 1, (uint8_t)IORAM_READ(10));   // output to pins 17-20 (gaplus)
		break;
	case 2:   // initialize coinage settings
		io->coins_per_cred[0] = IORAM_READ(9);
		io->creds_per_coin[0] = IORAM_READ(10);
		io->coins_per_cred[1] = IORAM_READ(11);
		io->creds_per_coin[1] = IORAM_READ(12);
		break;
	case 4:   // druaga, digdug chip #1: read dip switches and inputs; superpac chip #0: process coin and start inputs
		handle_coins(io, 0);
		break;
	case 7:   // bootup check (liblrabl only)
		IORAM_WRITE(2, 0xe);
		IORAM_WRITE(7, 0x6);
		break;
	case 8: {   // bootup check: the sum of 9-15
		int sum = 0;
		for (int i = 9; i < 16; i++)
			sum += IORAM_READ(i);
		IORAM_WRITE(0, sum >> 4);
		IORAM_WRITE(1, sum & 0xf);
		break;
	}
	case 9:   // read dip switches and inputs
		read_dips(io);
		break;
	default:
		break;
	}
}

static void run_59xx(namcoio_t *io)
{
	switch (IORAM_READ(8)) {
	case 3:   // pacnpal chip #1: read dip switches and inputs
		IORAM_WRITE(4, ~in_r(io, 0));   // pins 38-41, pin 13 = 0 ?
		IORAM_WRITE(5, ~in_r(io, 2));   // pins 26-29 ?
		IORAM_WRITE(6, ~in_r(io, 1));   // pins 22-25 ?
		IORAM_WRITE(7, ~in_r(io, 3));   // pins 30-33
		break;
	default:
		break;
	}
}

#define NEXT(n) ((((n) & 1) ? (n) ^ 0x90 : (n)) >> 1)

static void run_58xx(namcoio_t *io)
{
	switch (IORAM_READ(8)) {
	case 1:   // read switch inputs
		IORAM_WRITE(4, ~in_r(io, 0));   // pins 38-41
		IORAM_WRITE(5, ~in_r(io, 1));   // pins 22-25
		IORAM_WRITE(6, ~in_r(io, 2));   // pins 26-29
		IORAM_WRITE(7, ~in_r(io, 3));   // pins 30-33
		out_w(io, 0, (uint8_t)IORAM_READ(9));    // output to pins 13-16 (toypop)
		out_w(io, 1, (uint8_t)IORAM_READ(10));   // output to pins 17-20 (toypop)
		break;
	case 2:   // initialize coinage settings
		io->coins_per_cred[0] = IORAM_READ(9);
		io->creds_per_coin[0] = IORAM_READ(10);
		io->coins_per_cred[1] = IORAM_READ(11);
		io->creds_per_coin[1] = IORAM_READ(12);
		break;
	case 3:   // process coin and start inputs, read switch inputs
		handle_coins(io, 2);
		break;
	case 4:   // read dip switches and inputs
		read_dips(io);
		break;
	case 5: {   // bootup check: repeated XORs controlled by a 7-bit LFSR
		int n = (IORAM_READ(9) * 16 + IORAM_READ(10)) & 0x7f;
		int seed = 0x22;
		for (int i = 0; i < n; i++)
			seed = NEXT(seed);
		for (int i = 1; i < 8; i++) {
			n = 0;
			int rng = seed;
			if (rng & 1) n ^= ~IORAM_READ(11);
			rng = NEXT(rng);
			seed = rng;   // save state for next loop
			if (rng & 1) n ^= ~IORAM_READ(10);
			rng = NEXT(rng);
			if (rng & 1) n ^= ~IORAM_READ(9);
			rng = NEXT(rng);
			if (rng & 1) n ^= ~IORAM_READ(15);
			rng = NEXT(rng);
			if (rng & 1) n ^= ~IORAM_READ(14);
			rng = NEXT(rng);
			if (rng & 1) n ^= ~IORAM_READ(13);
			rng = NEXT(rng);
			if (rng & 1) n ^= ~IORAM_READ(12);
			IORAM_WRITE(i, ~n);
		}
		IORAM_WRITE(0, 0x0);
		if (IORAM_READ(9) == 0xf)   // kludge for gaplus
			IORAM_WRITE(0, 0xf);
		break;
	}
	default:
		break;
	}
}

void namcoio_run(namcoio_t *io)
{
	switch (io->type) {
	case NAMCOIO_56XX: run_56xx(io); break;
	case NAMCOIO_58XX: run_58xx(io); break;
	default: run_59xx(io); break;
	}
}
