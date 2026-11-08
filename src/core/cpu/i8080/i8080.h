/*  Intel 8080 / 8085A CPU core, ported to C from MAME src/devices/cpu/i8085/i8085.cpp
    (license BSD-3-Clause, copyright-holders: Juergen Buchmueller, Roberto Fresca, Grull Osgo;
    thanks-to: Marcel De Kogel). Opcode behaviour, flags and cycle tables follow MAME.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_I8080_H
#define RBX_I8080_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// input lines (MAME I8085_*_LINE)
enum { I8080_INTR_LINE = 0, I8085_RST55_LINE = 1, I8085_RST65_LINE = 2, I8085_RST75_LINE = 3, I8085_TRAP_LINE = 4 };

// CPU state (plain data: saved and restored as a block)
typedef struct {
	uint16_t pc, sp, wz;
	uint8_t a, f, b, c, d, e, h, l;
	uint8_t halt;
	uint8_t im;            // interrupt mask (8085A), IE bit also used on the 8080
	uint8_t status;        // status word
	uint8_t after_ei;      // post-EI processing; starts at 2, check for ints at 0
	uint8_t nmi_state;     // raw TRAP line state
	uint8_t irq_state[4];  // raw IRQ line states
	uint8_t trap_pending;
	uint8_t trap_im_copy;
	uint8_t sod_state;
	uint8_t in_acknowledge;
	uint8_t is_8085;
	uint8_t pad[3];
	int32_t icount;
} i8080_state;

typedef struct i8080_t {
	i8080_state s;
	void *user;
	uint8_t (*read)(void *user, uint16_t addr);
	void (*write)(void *user, uint16_t addr, uint8_t data);
	uint8_t (*in)(void *user, uint8_t port);
	void (*out)(void *user, uint8_t port, uint8_t data);
	// interrupt acknowledge (MAME standard_irq_callback): returns the byte read on the data bus (an RST opcode)
	uint8_t (*irq_ack)(void *user, int line);
	void (*inte)(void *user, int state);     // INTE output changed (may be NULL)
	int (*sid)(void *user);                  // 8085 SID input (may be NULL)
	void (*sod)(void *user, int state);      // 8085 SOD output (may be NULL)
	// optional fast paths: 256-byte pages read directly (NULL = use read callback)
	const uint8_t *rpage[256];
} i8080_t;

void i8080_init(i8080_t *cpu, int is_8085);   // after the callbacks are set
void i8080_reset(i8080_t *cpu);
void i8080_set_input(i8080_t *cpu, int line, int state);
// run for the given number of clocks (carried over between calls); returns clocks used by this call
int i8080_run(i8080_t *cpu, int clocks);

#ifdef __cplusplus
}
#endif

#endif
