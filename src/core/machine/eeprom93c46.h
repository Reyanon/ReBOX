/*  Serial EEPROM 93C46 in 16-bit mode (64 words), behaviour after MAME
    src/devices/machine/eepromser.cpp (BSD-3-Clause, copyright-holders: Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_EEPROM93C46_H
#define RBX_EEPROM93C46_H

#include <stdint.h>

typedef struct {
	uint16_t data[64];
	int cs, clk, di, dout;
	int state, bits, count;
	uint32_t shift;
	int addr, write_enabled;
	int dirty;
} eeprom93c46_t;

void eeprom_init(eeprom93c46_t *e);         // contents 0xffff
void eeprom_write_lines(eeprom93c46_t *e, int cs, int clk, int di);
int eeprom_do(const eeprom93c46_t *e);
int eeprom_load(eeprom93c46_t *e, const char *path);
int eeprom_save(eeprom93c46_t *e, const char *path);

#endif
