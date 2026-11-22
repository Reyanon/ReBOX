/*  Intel-command 8-bit flash, ported from MAME 0.289 src/devices/machine/intelfsh.cpp (BSD-3-Clause,
    copyright-holders: R. Belmont): the paths an 8-bit Intel part (E28F008SA) takes. See intelfsh.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include <stddef.h>
#include <string.h>

#include "intelfsh.h"

enum {
	FM_NORMAL,       // normal read/write
	FM_READID,       // read ID
	FM_READSTATUS,   // read status
	FM_WRITEPART1,   // first half of programming, awaiting second
	FM_CLEARPART1,   // first half of clear, awaiting second
	FM_SETMASTER     // first half of set master lock, awaiting on/off
};

void intelfsh_init(intelfsh_t *f, uint8_t *data, uint32_t size, uint8_t maker_id, uint8_t device_id,
	void (*erase_delay)(void *user, int seconds), void *user)
{
	memset(f, 0, sizeof(*f));
	f->data = data;
	f->size = size;
	f->maker_id = maker_id;
	f->device_id = device_id;
	f->erase_delay = erase_delay;
	f->user = user;
	intelfsh_reset(f);
}

// (the device's constructor values)
void intelfsh_reset(intelfsh_t *f)
{
	f->status = 0x80;
	f->flash_mode = FM_NORMAL;
	f->flash_master_lock = 0;
}

// delay_tick: the end of an erase
void intelfsh_delay_tick(intelfsh_t *f)
{
	if (f->flash_mode == FM_READSTATUS)
		f->status = 0x80;
}

// read_full (8 bits)
int intelfsh_reads_data(const intelfsh_t *f) { return f->flash_mode == FM_NORMAL; }

uint8_t intelfsh_read(intelfsh_t *f, uint32_t address)
{
	uint32_t data = 0;
	address = (f->size & (f->size - 1)) ? address % f->size : address & (f->size - 1);   // (no division: a power of two)
	switch (f->flash_mode) {
	default:
	case FM_NORMAL:
		data = f->data[address];
		break;
	case FM_READSTATUS:
		data = f->status;
		break;
	case FM_READID:
		switch (address & 0xff) {
		case 0: // maker ID
			data = f->maker_id;
			break;
		case 1: // chip ID
			data = f->device_id;
			break;
		case 2: // block lock config
			data = 0; // we don't support this yet
			break;
		case 3: // master lock config
			data = f->flash_master_lock ? 1 : 0;
			break;
		}
		break;
	}
	return (uint8_t)data;
}

// write_full (8 bits)
void intelfsh_write(intelfsh_t *f, uint32_t address, uint8_t data)
{
	address = (f->size & (f->size - 1)) ? address % f->size : address & (f->size - 1);
	switch (f->flash_mode) {
	case FM_NORMAL:
	case FM_READSTATUS:
	case FM_READID:
		switch (data & 0xff) {
		case 0xf0:
		case 0xff:  // reset chip mode
			f->flash_mode = FM_NORMAL;
			break;
		case 0x90:  // read ID
			f->flash_mode = FM_READID;
			break;
		case 0x40:
		case 0x10:  // program
			f->flash_mode = FM_WRITEPART1;
			break;
		case 0x50:  // clear status reg
			f->status = 0x80;
			f->flash_mode = FM_READSTATUS;
			break;
		case 0x20:  // block erase
			f->flash_mode = FM_CLEARPART1;
			break;
		case 0x60:  // set master lock
			f->flash_mode = FM_SETMASTER;
			break;
		case 0x70:  // read status
			f->flash_mode = FM_READSTATUS;
			break;
		default:    // (unknown flash mode byte; the AMD ID sequence of other parts is not emulated)
			break;
		}
		break;
	case FM_WRITEPART1:
		f->data[address] = data;
		f->status = 0x80;
		f->flash_mode = FM_READSTATUS;
		break;
	case FM_CLEARPART1:
		if ((data & 0xff) == 0xd0) {
			// clear the 64k block containing the current address to all 0xffs
			memset(&f->data[address & ~0xffffu], 0xff, 64 * 1024);
			if (f->erase_delay)
				f->erase_delay(f->user, 1);
			f->status = 0x00;
			f->flash_mode = FM_READSTATUS;
		}
		break;
	case FM_SETMASTER:
		switch (data & 0xff) {
		case 0xf1:
			f->flash_master_lock = 1;
			break;
		case 0xd0:
			f->flash_master_lock = 0;
			break;
		default:
			break;
		}
		f->flash_mode = FM_READSTATUS;
		break;
	}
}
