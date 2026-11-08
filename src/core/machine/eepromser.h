/*  Serial EEPROMs (93Cxx, ER5911), ported to C from MAME src/devices/machine/eepromser.cpp and eeprom.cpp
    (BSD-3-Clause, copyright-holders: Aaron Giles): the CS / CLK / DI state machine, the commands, the operation times
    (the ready line). The times are in the board's ticks (ticks_per_sec at init: its main CPU clock).
    (machine/eeprom93c46.c stays for the boards that use it.)
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_EEPROMSER_H
#define RBX_EEPROMSER_H

#include <stdint.h>

enum { EEPSER_93CXX, EEPSER_ER5911 };

typedef struct {
	// ---- state (eepser_state) ----
	uint16_t data[256];
	int32_t state, cs, clk, di, bits_accum, command, address, locked;
	uint32_t shift, cmd_accum;
	int64_t completion_time, last_cs_rising;
	// ---- configuration ----
	int type, data_bits, address_bits, cmd_address_bits, streaming;
	int64_t op_time[4];   // write, write all, erase, erase all (ticks)
	int dirty;            // (written since loaded or saved)
	char nv_path[256];    // (eepser_nv_load's file: the set's .nv in the NVRAM folder)
} eepser_t;

// cells: 64 / 128 ..., data_bits: 8 or 16
void eepser_init(eepser_t *e, int type, int cells, int data_bits, int64_t ticks_per_sec);
void eepser_cs_write(eepser_t *e, int state, int64_t now);
void eepser_clk_write(eepser_t *e, int state, int64_t now);
void eepser_di_write(eepser_t *e, int state);
int eepser_do_read(const eepser_t *e);
int eepser_ready_read(const eepser_t *e, int64_t now);
// the default contents: a ROM region (big-endian words for 16-bit cells), else 0xff..
void eepser_default(eepser_t *e, const uint8_t *data, uint32_t size);
int eepser_state(eepser_t *e, void *ctx, int load, const char *tag);
// the contents kept between runs as MAME's nvram (<set>.nv in the NVRAM folder, eepser_default's layout): read at
// init after the default (none without a folder: the frontend's measurements), written at exit when changed
void eepser_nv_load(eepser_t *e, const char *set);
void eepser_nv_save(eepser_t *e);

#endif
