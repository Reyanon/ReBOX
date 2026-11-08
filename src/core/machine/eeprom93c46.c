/*  Serial EEPROM 93C46 in 16-bit mode (64 words), behaviour after MAME
    src/devices/machine/eepromser.cpp (BSD-3-Clause, copyright-holders: Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "eeprom93c46.h"

#include <stdio.h>
#include <string.h>

enum { ST_IDLE, ST_WAIT_START, ST_CMD, ST_READ, ST_DATA };

void eeprom_init(eeprom93c46_t *e)
{
	memset(e, 0, sizeof(*e));
	for (int i = 0; i < 64; i++)
		e->data[i] = 0xffff;
	e->dout = 1;
}

static void command(eeprom93c46_t *e)
{
	int op = (e->shift >> 6) & 3, addr = e->shift & 0x3f;
	switch (op) {
	case 2:                 // READ
		e->addr = addr;
		e->shift = e->data[addr];
		e->count = 16;
		e->dout = 0;        // dummy zero before the data
		e->state = ST_READ;
		break;
	case 1:                 // WRITE
		e->addr = addr;
		e->shift = 0;
		e->count = 16;
		e->state = ST_DATA;
		break;
	case 3:                 // ERASE
		if (e->write_enabled) {
			e->data[addr] = 0xffff;
			e->dirty = 1;
		}
		e->dout = 1;
		e->state = ST_WAIT_START;
		break;
	default:
		switch (addr >> 4) {
		case 0: e->write_enabled = 0; break;           // EWDS
		case 3: e->write_enabled = 1; break;           // EWEN
		case 2:                                         // ERAL
			if (e->write_enabled) {
				for (int i = 0; i < 64; i++) e->data[i] = 0xffff;
				e->dirty = 1;
			}
			break;
		case 1:                                         // WRAL
			e->addr = -1;
			e->shift = 0;
			e->count = 16;
			e->state = ST_DATA;
			return;
		}
		e->dout = 1;
		e->state = ST_WAIT_START;
		break;
	}
}

void eeprom_write_lines(eeprom93c46_t *e, int cs, int clk, int di)
{
	cs = !!cs; clk = !!clk; di = !!di;
	if (!cs) {
		e->state = ST_IDLE;
		e->cs = 0;
		e->clk = clk;
		e->di = di;
		return;
	}
	if (!e->cs) {
		e->state = ST_WAIT_START;
		e->dout = 1;
	}
	e->cs = 1;
	if (clk && !e->clk) {
		switch (e->state) {
		case ST_WAIT_START:
			if (di) {
				e->state = ST_CMD;
				e->shift = 0;
				e->bits = 0;
			}
			break;
		case ST_CMD:
			e->shift = (e->shift << 1) | di;
			if (++e->bits == 8)
				command(e);
			break;
		case ST_READ:   // (MAME: reading until CS falls, ones after the word)
			e->dout = (e->shift >> 15) & 1;
			e->shift = ((e->shift << 1) | 1) & 0xffff;
			break;
		case ST_DATA:
			e->shift = (e->shift << 1) | di;
			if (--e->count == 0) {
				if (e->write_enabled) {
					if (e->addr < 0)
						for (int i = 0; i < 64; i++) e->data[i] = (uint16_t)e->shift;
					else
						e->data[e->addr] = (uint16_t)e->shift;
					e->dirty = 1;
				}
				e->dout = 1;
				e->state = ST_WAIT_START;
			}
			break;
		}
	}
	e->clk = clk;
	e->di = di;
}

// MAME do_read: the data while reading; otherwise high (the pull-up, or ready while waiting for a start bit)
int eeprom_do(const eeprom93c46_t *e) { return e->state == ST_READ ? e->dout : 1; }

int eeprom_load(eeprom93c46_t *e, const char *path)
{
	FILE *fp = fopen(path, "rb");
	if (!fp)
		return -1;
	uint8_t b[128];
	size_t n = fread(b, 1, 128, fp);
	fclose(fp);
	if (n != 128)
		return -1;
	for (int i = 0; i < 64; i++)
		e->data[i] = (uint16_t)(b[2 * i] << 8 | b[2 * i + 1]);   // MAME .nv layout: big-endian words
	return 0;
}

int eeprom_save(eeprom93c46_t *e, const char *path)
{
	uint8_t b[128];
	for (int i = 0; i < 64; i++) {
		b[2 * i] = (uint8_t)(e->data[i] >> 8);
		b[2 * i + 1] = (uint8_t)e->data[i];
	}
	FILE *fp = fopen(path, "wb");
	if (!fp)
		return -1;
	size_t n = fwrite(b, 1, 128, fp);
	fclose(fp);
	e->dirty = 0;
	return n == 128 ? 0 : -1;
}
