/*  Data East 146 / 104 protection and I/O chips, after MAME src/mame/dataeast/deco146.cpp / deco104.cpp
    (BSD-3-Clause, copyright-holders: David Haywood, Charles MacDonald).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_DECO146_H
#define RBX_DECO146_H
#include <stdint.h>
#include <stddef.h>

typedef struct {
	// ---- state (saved: from rambank up to the configuration) ----
	uint16_t rambank[2][0x80];
	uint16_t nand, xor_, latchaddr, latchdata;
	uint8_t current_rambank, latchflag, soundlatch, soundlatch_pending;
	uint8_t region_selects[6];
	uint8_t pad_[2];
	// ---- configuration ----
	uint8_t bankswap_read_addr, xor_port, mask_port, sound_port;
	uint16_t magic_read_xor;
	int magic_read_xor_on;      // set_use_magic_read_address_xor
	uint8_t addrswap[10];       // a0 .. a9 (set_interface_scramble)
	const void *table;
	uint16_t (*port_a)(void *user), (*port_b)(void *user), (*port_c)(void *user);   // NULL: 0xffff
	void (*soundlatch_irq)(void *user, int state);
	void *user;
} deco146_t;
#define DECO146_STATE_SIZE ((uint32_t)offsetof(deco146_t, bankswap_read_addr))

void deco146_init(deco146_t *d, int is104);      // the 146 or the 104 (their tables and ports)
// MAME set_interface_scramble(a9, a8, ... a0)
void deco146_set_scramble(deco146_t *d, const uint8_t a9_to_a0[10]);
void deco146_reset(deco146_t *d);
// MAME write_data / read_data: address in bytes (the board's offset << 1 as MAME passes it), csflags may be NULL
void deco146_write(deco146_t *d, uint16_t address, uint16_t data, uint16_t mem_mask, uint8_t *csflags);
uint16_t deco146_read(deco146_t *d, uint16_t address, uint8_t *csflags);
uint8_t deco146_soundlatch_r(deco146_t *d);
#endif
