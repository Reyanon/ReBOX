/*  Serial EEPROMs (93Cxx, ER5911), ported to C from MAME src/devices/machine/eepromser.cpp and eeprom.cpp
    (BSD-3-Clause, copyright-holders: Aaron Giles). See eepromser.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "eepromser.h"
#include "../rbx.h"

#include <stdio.h>
#include <string.h>

enum { ST_IN_RESET, ST_WAIT_FOR_START_BIT, ST_WAIT_FOR_COMMAND, ST_READING_DATA, ST_WAIT_FOR_DATA, ST_WAIT_FOR_COMPLETION };
enum { CMD_INVALID, CMD_READ, CMD_WRITE, CMD_ERASE, CMD_LOCK, CMD_UNLOCK, CMD_WRITEALL, CMD_ERASEALL };
enum { EV_CS_RISING, EV_CS_FALLING, EV_CLK_RISING, EV_CLK_FALLING };
enum { T_WRITE, T_WRITE_ALL, T_ERASE, T_ERASE_ALL };

// cell a (0..511: the 512 cells of a 93C66 in 8-bit mode go on in data_hi)
#define CELL(e, a) (*((a) < 256 ? &(e)->data[(a)] : &(e)->data_hi[((a) - 256) & 0xff]))

void eepser_init(eepser_t *e, int type, int cells, int data_bits, int64_t ticks_per_sec)
{
	memset(e, 0, sizeof(*e));
	e->type = type;
	e->data_bits = data_bits;
	e->address_bits = 0;
	while ((1 << e->address_bits) < cells)
		e->address_bits++;
	// (the command's address bits: MAME's DEFINE_SERIAL_EEPROM_DEVICE address bits)
	if (type == EEPSER_ER5911)
		e->cmd_address_bits = data_bits == 8 ? 9 : 8;
	else
		e->cmd_address_bits = e->address_bits;
	e->op_time[T_WRITE] = ticks_per_sec * 1750 / 1000000;
	e->op_time[T_WRITE_ALL] = ticks_per_sec * 8000 / 1000000;
	e->op_time[T_ERASE] = ticks_per_sec * 1000 / 1000000;
	e->op_time[T_ERASE_ALL] = ticks_per_sec * 8000 / 1000000;
	for (int i = 0; i < 512; i++)
		CELL(e, i) = (uint16_t)((1u << data_bits) - 1);
	e->state = ST_IN_RESET;
	e->completion_time = 0;
	e->last_cs_rising = -1;
}

void eepser_default(eepser_t *e, const uint8_t *data, uint32_t size)
{
	const int cells = 1 << e->address_bits;
	for (int i = 0; i < cells && i < 512; i++) {
		if (e->data_bits == 16)
			CELL(e, i) = 2u * (uint32_t)i + 1 < size ? (uint16_t)(data[2 * i] << 8 | data[2 * i + 1]) : 0xffff;
		else
			CELL(e, i) = (uint32_t)i < size ? data[i] : 0xff;
	}
}

static inline int ready(const eepser_t *e, int64_t now) { return now >= e->completion_time; }
static inline uint32_t mask_data(const eepser_t *e) { return (1u << e->data_bits) - 1; }
static void ewrite(eepser_t *e, int a, uint32_t v, int64_t now)
{
	CELL(e, a & ((1 << e->address_bits) - 1)) = (uint16_t)(v & mask_data(e));
	e->dirty = 1;
	e->completion_time = now + e->op_time[T_WRITE];
}

static void parse_command(eepser_t *e)
{
	e->command = CMD_INVALID;
	e->address = (int)(e->cmd_accum & ((1u << e->cmd_address_bits) - 1));
	const int sub = e->address >> (e->cmd_address_bits - 2);
	switch (e->cmd_accum >> e->cmd_address_bits) {
	case 0:
		if (e->type == EEPSER_ER5911)
			e->command = sub == 0 ? CMD_LOCK : sub == 2 ? CMD_ERASEALL : sub == 3 ? CMD_UNLOCK : CMD_INVALID;
		else
			e->command = sub == 0 ? CMD_LOCK : sub == 1 ? CMD_WRITEALL : sub == 2 ? CMD_ERASEALL : CMD_UNLOCK;
		e->address = 0;
		break;
	case 1: e->command = CMD_WRITE; break;
	case 2: e->command = CMD_READ; break;
	case 3: e->command = e->type == EEPSER_ER5911 ? CMD_WRITE : CMD_ERASE; break;   // (WRITE instead of ERASE on the ER5911)
	}
}

static void execute_command(eepser_t *e, int64_t now)
{
	parse_command(e);
	e->bits_accum = 0;
	switch (e->command) {
	case CMD_READ:
		e->shift = 0;
		e->state = ST_READING_DATA;
		break;
	case CMD_WRITE:
	case CMD_WRITEALL:
		e->shift = 0;
		e->state = ST_WAIT_FOR_DATA;
		break;
	case CMD_ERASE:
		if (e->locked) {
			e->state = ST_IN_RESET;
			break;
		}
		CELL(e, e->address & ((1 << e->address_bits) - 1)) = (uint16_t)mask_data(e);
		e->dirty = 1;
		e->completion_time = now + e->op_time[T_ERASE];
		e->state = ST_WAIT_FOR_COMPLETION;
		break;
	case CMD_LOCK: e->locked = 1; e->state = ST_IN_RESET; break;
	case CMD_UNLOCK: e->locked = 0; e->state = ST_IN_RESET; break;
	case CMD_ERASEALL:
		if (e->locked) {
			e->state = ST_IN_RESET;
			break;
		}
		for (int a = 0; a < (1 << e->address_bits); a++)
			CELL(e, a) = (uint16_t)mask_data(e);
		e->dirty = 1;
		e->completion_time = now + e->op_time[T_ERASE_ALL];
		e->state = ST_WAIT_FOR_COMPLETION;
		break;
	default:
		e->state = ST_IN_RESET;
		break;
	}
}

static void execute_write_command(eepser_t *e, int64_t now)
{
	if (e->locked) {
		e->state = ST_IN_RESET;
		return;
	}
	if (e->command == CMD_WRITE)
		ewrite(e, e->address, e->shift, now);
	else {   // (WRITEALL: ANDed with the data after an ERASEALL)
		for (int a = 0; a < (1 << e->address_bits); a++)
			CELL(e, a) = (uint16_t)(CELL(e, a) & e->shift & mask_data(e));
		e->dirty = 1;
		e->completion_time = now + e->op_time[T_WRITE_ALL];
	}
	e->state = ST_WAIT_FOR_COMPLETION;
}

static void handle_event(eepser_t *e, int ev, int64_t now)
{
	switch (e->state) {
	case ST_IN_RESET:
		if (ev == EV_CS_RISING)
			e->state = ST_WAIT_FOR_START_BIT;
		break;
	case ST_WAIT_FOR_START_BIT:
		if (ev == EV_CLK_RISING && e->di && ready(e, now) && now > e->last_cs_rising) {
			e->cmd_accum = 0;
			e->bits_accum = 0;
			e->state = ST_WAIT_FOR_COMMAND;
		} else if (ev == EV_CS_FALLING)
			e->state = ST_IN_RESET;
		break;
	case ST_WAIT_FOR_COMMAND:
		if (ev == EV_CLK_RISING) {
			e->cmd_accum = (e->cmd_accum << 1) | (uint32_t)e->di;
			if (++e->bits_accum == 2 + e->cmd_address_bits)
				execute_command(e, now);
		} else if (ev == EV_CS_FALLING)
			e->state = ST_IN_RESET;
		break;
	case ST_READING_DATA:
		if (ev == EV_CLK_RISING) {   // (output on the rising clock)
			const int bit_index = e->bits_accum++;
			if (bit_index % e->data_bits == 0 && (bit_index == 0 || e->streaming))
				e->shift = (uint32_t)CELL(e, (e->address + e->bits_accum / e->data_bits) & ((1 << e->address_bits) - 1))
				           << (32 - e->data_bits);
			else
				e->shift = (e->shift << 1) | 1;
		} else if (ev == EV_CS_FALLING)
			e->state = ST_IN_RESET;
		break;
	case ST_WAIT_FOR_DATA:
		if (ev == EV_CLK_RISING) {
			e->shift = (e->shift << 1) | (uint32_t)e->di;
			if (++e->bits_accum == e->data_bits)
				execute_write_command(e, now);
		} else if (ev == EV_CS_FALLING)
			e->state = ST_IN_RESET;
		break;
	case ST_WAIT_FOR_COMPLETION:
		if (ev == EV_CS_FALLING)
			e->state = ST_IN_RESET;
		break;
	}
}

void eepser_cs_write(eepser_t *e, int state, int64_t now)
{
	state &= 1;
	if (state == e->cs)
		return;
	e->cs = state;
	if (state)
		e->last_cs_rising = now;
	handle_event(e, state ? EV_CS_RISING : EV_CS_FALLING, now);
}

void eepser_clk_write(eepser_t *e, int state, int64_t now)
{
	state &= 1;
	if (state == e->clk)
		return;
	e->clk = state;
	handle_event(e, state ? EV_CLK_RISING : EV_CLK_FALLING, now);
}

void eepser_di_write(eepser_t *e, int state) { e->di = state & 1; }

int eepser_do_read(const eepser_t *e)
{
	if (e->state == ST_READING_DATA)
		return (e->shift & 0x80000000u) != 0;
	return 1;   // (the tri-state value: ASSERT_LINE)
}

int eepser_ready_read(const eepser_t *e, int64_t now) { return ready(e, now); }

// eeprom_serial_93cxx_device::do_read: the ready state while waiting for the start bit
int eepser_do_read_93cxx(const eepser_t *e, int64_t now)
{
	return e->state == ST_WAIT_FOR_START_BIT ? ready(e, now) : eepser_do_read(e);
}

int eepser_state(eepser_t *e, void *ctx, int load, const char *tag)
{
	(void)load;
	const uint32_t n = (uint32_t)((const uint8_t *)&e->type - (const uint8_t *)e);
	if (e->address_bits > 8) {   // (the cells past 256: a block of their own, the others' states unchanged)
		char t2[64];
		snprintf(t2, sizeof(t2), "%s+", tag);
		if (rbx_state_io(ctx, t2, e->data_hi, sizeof(e->data_hi)))
			return -1;
	}
	return rbx_state_io(ctx, tag, e, n);
}

void eepser_nv_load(eepser_t *e, const char *set)
{
	extern const char *rbx_nvram_dir(void);
	const char *dir = rbx_nvram_dir();
	e->nv_path[0] = 0;
	e->dirty = 0;
	if (!dir || !set)
		return;
	snprintf(e->nv_path, sizeof(e->nv_path), "%s%s.nv", dir, set);
	FILE *f = fopen(e->nv_path, "rb");
	if (!f)
		return;
	uint8_t b[512];
	const size_t n = fread(b, 1, sizeof(b), f);
	fclose(f);
	const uint32_t want = (uint32_t)(1 << e->address_bits) * (e->data_bits > 8 ? 2u : 1u);
	if (n >= want)
		eepser_default(e, b, (uint32_t)n);
}

void eepser_nv_save(eepser_t *e)
{
	if (!e->nv_path[0] || !e->dirty)
		return;
	const int cells = 1 << e->address_bits;
	uint8_t b[512];
	uint32_t n = 0;
	for (int i = 0; i < cells && i < 512; i++) {
		if (e->data_bits > 8)
			b[n++] = (uint8_t)(CELL(e, i) >> 8);
		b[n++] = (uint8_t)CELL(e, i);
	}
	FILE *f = fopen(e->nv_path, "wb");
	if (!f)
		return;
	fwrite(b, 1, n, f);
	fclose(f);
	e->dirty = 0;
}
