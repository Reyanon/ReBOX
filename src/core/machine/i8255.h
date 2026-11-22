/*  Intel 8255 PPI. Ported from MAME src/devices/machine/i8255.cpp (BSD-3-Clause, copyright-holders: Curt Coder).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_I8255_H
#define RBX_I8255_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
	// configuration (not saved)
	void *user;
	uint8_t (*in_pa)(void *user);   // NULL: 0
	uint8_t (*in_pb)(void *user);
	uint8_t (*in_pc)(void *user);
	void (*out_pa)(void *user, uint8_t v);
	void (*out_pb)(void *user, uint8_t v);
	void (*out_pc)(void *user, uint8_t v);
	uint8_t tri_pa, tri_pb, tri_pc;   // (the floating inputs: MAME's tri_*_callback, 0xff by default)
	// state (saved: from here)
	uint8_t control;
	uint8_t output[3], input[3];
	uint8_t ibf[2], obf[2], inte[2], intr[2];
	uint8_t inte1, inte2;
} i8255_t;

#define I8255_STATE_OFFSET offsetof(i8255_t, control)
#define I8255_STATE_SIZE (sizeof(i8255_t) - offsetof(i8255_t, control))

void i8255_reset(i8255_t *p);   // (set_mode(0x9b))
uint8_t i8255_read(i8255_t *p, int offset);
void i8255_write(i8255_t *p, int offset, uint8_t data);
void i8255_pc2_w(i8255_t *p, int state);
void i8255_pc4_w(i8255_t *p, int state);
void i8255_pc6_w(i8255_t *p, int state);   // (port A's /ACK in mode 1 output)

#endif
