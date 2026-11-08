/*  Texas Instruments TMS320C10 / C15 DSP, ported to C from MAME src/devices/cpu/tms320c1x/tms320c1x.cpp
    (BSD-3-Clause, copyright-holders: Tony La Porta).
    A cycle is 4 input clocks. Program space: 12-bit word addresses (the board's ROM, as big-endian words); data
    space: 256 words of RAM on the C15 (128 + 16 on the C10); I/O: 8 word ports. The instruction set, flags and cycle
    counts as MAME's.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_TMS320C1X_H
#define RBX_TMS320C1X_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct tms320c1x_s tms320c1x_t;
struct tms320c1x_s {
	// ---- state (saved: up to ram, included) ----
	uint16_t pc, prevpc, str, treg, ar[2], stack[4], opcode, memaccess;
	uint32_t acc, alu, preg, oldacc;
	int32_t intf, icount;
	int64_t cycles;            // cycles executed since init
	uint16_t ram[256];
	// ---- configuration ----
	const uint16_t *rom;       // program words (host order), rom_mask + 1 of them
	uint16_t rom_mask;
	uint16_t ram_mask;         // 0xff (C15)
	uint16_t (*in)(void *user, int port);
	void (*out)(void *user, int port, uint16_t data);
	int (*bio)(void *user);    // the BIO pin (1 = asserted: BIOZ branches)
	void *user;
};

#define TMS320C1X_STATE_SIZE ((uint32_t)offsetof(tms320c1x_t, rom))

void tms320c1x_init(tms320c1x_t *c);   // clears everything; set rom / callbacks, then reset
void tms320c1x_reset(tms320c1x_t *c);
// run whole instructions until at least 'cycles' cycles are used; returns the cycles used
int tms320c1x_execute(tms320c1x_t *c, int cycles);
void tms320c1x_set_irq(tms320c1x_t *c, int state);   // (a pending interrupt cannot be cleared)

#ifdef __cplusplus
}
#endif

#endif
