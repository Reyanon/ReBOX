/*  Dallas DS2404 through the SEI600's parallel interface, ported from MAME 0.289 src/devices/machine/ds2404.cpp
    (BSD-3-Clause, copyright-holders: Aaron Giles). See ds2404.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "ds2404.h"

#include <string.h>

enum {
	STATE_IDLE = 1,              /* waiting for ROM command, in 1-wire mode */
	STATE_COMMAND,               /* waiting for memory command */
	STATE_ADDRESS1,              /* waiting for address bits 0-7 */
	STATE_ADDRESS2,              /* waiting for address bits 8-15 */
	STATE_OFFSET,                /* waiting for ending offset */
	STATE_INIT_COMMAND,
	STATE_READ_MEMORY,           /* Read Memory command active */
	STATE_WRITE_SCRATCHPAD,      /* Write Scratchpad command active */
	STATE_READ_SCRATCHPAD,       /* Read Scratchpad command active */
	STATE_COPY_SCRATCHPAD        /* Copy Scratchpad command active */
};

void ds2404_init(ds2404_t *d)
{
	memset(d, 0, sizeof(*d));
	// (the RTC: 0 seconds after the reference date, see ds2404.h)
	for (int i = 0; i < 8; i++)
		d->state[i] = STATE_IDLE;
}

static void rom_cmd(ds2404_t *d, uint8_t cmd)
{
	switch (cmd) {
	case 0xcc:      /* Skip ROM */
		d->state[0] = STATE_COMMAND;
		d->state_ptr = 0;
		break;
	default:        /* (MAME: fatal, unknown ROM command) */
		break;
	}
}

static void cmd(ds2404_t *d, uint8_t c)
{
	switch (c) {
	case 0x0f:      /* Write scratchpad */
		d->state[0] = STATE_ADDRESS1;
		d->state[1] = STATE_ADDRESS2;
		d->state[2] = STATE_INIT_COMMAND;
		d->state[3] = STATE_WRITE_SCRATCHPAD;
		d->state_ptr = 0;
		break;
	case 0x55:      /* Copy scratchpad */
		d->state[0] = STATE_ADDRESS1;
		d->state[1] = STATE_ADDRESS2;
		d->state[2] = STATE_OFFSET;
		d->state[3] = STATE_INIT_COMMAND;
		d->state[4] = STATE_COPY_SCRATCHPAD;
		d->state_ptr = 0;
		break;
	case 0xf0:      /* Read memory */
		d->state[0] = STATE_ADDRESS1;
		d->state[1] = STATE_ADDRESS2;
		d->state[2] = STATE_INIT_COMMAND;
		d->state[3] = STATE_READ_MEMORY;
		d->state_ptr = 0;
		break;
	default:        /* (MAME: fatal, unknown command) */
		break;
	}
}

static uint8_t readmem(ds2404_t *d)
{
	if (d->address < 0x200)
		return d->sram[d->address];
	else if (d->address >= 0x202 && d->address <= 0x206)
		return d->rtc[d->address - 0x202];
	return 0;
}

static void writemem(ds2404_t *d, uint8_t value)
{
	if (d->address < 0x200)
		d->sram[d->address] = value;
	else if (d->address >= 0x202 && d->address <= 0x206)
		d->rtc[d->address - 0x202] = value;
}

void ds2404_1w_reset_w(ds2404_t *d)
{
	d->state[0] = STATE_IDLE;
	d->state_ptr = 0;
}

void ds2404_3w_reset_w(ds2404_t *d)
{
	d->state[0] = STATE_COMMAND;
	d->state_ptr = 0;
}

uint8_t ds2404_data_r(ds2404_t *d)
{
	uint8_t value = 0;
	switch (d->state[d->state_ptr]) {
	case STATE_READ_MEMORY:
		value = readmem(d);
		break;
	case STATE_READ_SCRATCHPAD:
		if (d->offset < 0x20) {
			value = d->ram[d->offset];
			d->offset++;
		}
		break;
	default:
		break;
	}
	return value;
}

void ds2404_data_w(ds2404_t *d, uint8_t data)
{
	switch (d->state[d->state_ptr]) {
	case STATE_IDLE:
		rom_cmd(d, data & 0xff);
		break;
	case STATE_COMMAND:
		cmd(d, data & 0xff);
		break;
	case STATE_ADDRESS1:
		d->a1 = data & 0xff;
		d->state_ptr++;
		break;
	case STATE_ADDRESS2:
		d->a2 = data & 0xff;
		d->state_ptr++;
		break;
	case STATE_OFFSET:
		d->end_offset = data & 0xff;
		d->state_ptr++;
		break;
	case STATE_WRITE_SCRATCHPAD:
		if (d->offset < 0x20) {
			d->ram[d->offset] = data & 0xff;
			d->offset++;
		}
		break;
	default:
		break;
	}

	if (d->state[d->state_ptr] == STATE_INIT_COMMAND) {
		switch (d->state[d->state_ptr + 1]) {
		case STATE_READ_MEMORY:
			d->address = (uint16_t)((d->a2 << 8) | d->a1);
			d->address -= 1;
			break;
		case STATE_WRITE_SCRATCHPAD:
		case STATE_READ_SCRATCHPAD:
			d->address = (uint16_t)((d->a2 << 8) | d->a1);
			d->offset = d->address & 0x1f;
			break;
		case STATE_COPY_SCRATCHPAD:
			d->address = (uint16_t)((d->a2 << 8) | d->a1);
			for (int i = 0; i <= d->end_offset; i++) {
				writemem(d, d->ram[i & 0x1f]);
				d->address++;
			}
			break;
		default:
			break;
		}
		d->state_ptr++;
	}
}

void ds2404_clk_w(ds2404_t *d)
{
	if (d->state[d->state_ptr] == STATE_READ_MEMORY)
		d->address++;
}

// timer_tick
void ds2404_tick(ds2404_t *d)
{
	for (int i = 0; i < 5; i++) {
		d->rtc[i]++;
		if (d->rtc[i] != 0)
			break;
	}
}
