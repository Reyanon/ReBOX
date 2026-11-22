/*  Namco 56XX / 58XX / 59XX custom I/O chips (the 4-bit MCUs of Super Pac-Man, Mappy, Phozon, Gaplus ...), simulated
    by their modes as MAME does. Ported from MAME 0.289 src/mame/namco/namcoio.cpp / namcoio.h (BSD-3-Clause,
    copyright-holders: Nicola Salmoria, Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_NAMCOIO_H
#define RBX_NAMCOIO_H

#include <stddef.h>
#include <stdint.h>

enum { NAMCOIO_56XX = 0, NAMCOIO_58XX, NAMCOIO_59XX };

typedef struct namcoio_s namcoio_t;
struct namcoio_s {
	// state (saved: from here to state_end)
	uint8_t ram[16];
	int32_t reset;
	int32_t lastcoins, lastbuttons;
	int32_t credits;
	int32_t coins[2];
	int32_t coins_per_cred[2];
	int32_t creds_per_coin[2];
	int32_t in_count;
	uint8_t state_end;
	// configuration
	int type;
	uint8_t (*in)(void *ctx, int n);              // in_callback<n> (NULL: 0)
	void (*out)(void *ctx, int n, uint8_t data);  // out_callback<n> (NULL: none)
	void *ctx;
};

#define NAMCOIO_STATE_SIZE offsetof(namcoio_t, state_end)

// clear the state and set the type (the callbacks are the caller's, set after)
void namcoio_init(namcoio_t *io, int type);
void namcoio_reset(namcoio_t *io);              // device_reset
uint8_t namcoio_read(namcoio_t *io, int offset);
void namcoio_write(namcoio_t *io, int offset, uint8_t data);
void namcoio_set_reset_line(namcoio_t *io, int state);
int namcoio_read_reset_line(const namcoio_t *io);
void namcoio_run(namcoio_t *io);                // customio_run

#endif
