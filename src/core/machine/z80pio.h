/*  Zilog Z80 PIO (parallel I/O): the two ports' modes, handshake (ready / strobe) and the interrupt logic.
    Ported from MAME src/devices/machine/z80pio.cpp (BSD-3-Clause, copyright-holders: Curt Coder). The daisy chain
    is left to the board (z80pio_int_state / ack / reti).
    Save state: the struct from Z80PIO_STATE_OFFSET is plain data.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_Z80PIO_H
#define RBX_Z80PIO_H

#include <stddef.h>
#include <stdint.h>

enum { Z80PIO_PORT_A = 0, Z80PIO_PORT_B };

typedef struct {
	uint8_t mode, next_control_word, input, output, ior;
	uint8_t rdy, stb, ie, ip, ius, icw, vector, mask, match;
} z80pio_port_t;

typedef struct {
	// configuration (not saved)
	void *user;
	uint8_t (*in_p[2])(void *user);                // NULL: 0
	void (*out_p[2])(void *user, uint8_t v);
	void (*out_rdy[2])(void *user, int state);
	void (*out_int)(void *user, int state);
	// state (saved: from here)
	z80pio_port_t port[2];
} z80pio_t;

#define Z80PIO_STATE_OFFSET offsetof(z80pio_t, port)
#define Z80PIO_STATE_SIZE (sizeof(z80pio_t) - offsetof(z80pio_t, port))

void z80pio_reset(z80pio_t *p);
uint8_t z80pio_read(z80pio_t *p, int offset);              // (MAME read(): A0 = port B, A1 = control)
void z80pio_write(z80pio_t *p, int offset, uint8_t data);
uint8_t z80pio_port_read(z80pio_t *p, int index);          // (the port's output pins)
void z80pio_strobe(z80pio_t *p, int index, int state);
int z80pio_int_state(const z80pio_t *p);                   // (the INT output)

#endif
