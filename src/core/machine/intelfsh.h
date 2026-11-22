/*  Intel-command 8-bit flash (the Intel E28F008SA: 1 MB, maker 0x89, device 0xa2), ported from MAME 0.289
    src/devices/machine/intelfsh.cpp, intelfsh.h (BSD-3-Clause, copyright-holders: R. Belmont): read_full /
    write_full for an 8-bit Intel part (read array, read ID, read / clear status, program, 64 KB block erase, master
    lock). The other makers' sequences (AMD, Atmel, Winbond, SST, the write buffer) are left out.
    The erase's delay is the owner's timer: flash_erase_delay is set (in seconds) and the owner calls
    intelfsh_delay_tick after it (MAME's delay_tick).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_INTELFSH_H
#define RBX_INTELFSH_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
	// ---- state (saved) ----
	uint8_t status;
	int32_t flash_mode;
	uint8_t flash_master_lock;
	uint8_t pad[3];
	// ---- configuration ----
	uint8_t *data;       // size bytes (the owner's, saved with the state by the owner)
	uint32_t size;
	uint8_t maker_id, device_id;
	void (*erase_delay)(void *user, int seconds);   // starts the timer that calls intelfsh_delay_tick
	void *user;
} intelfsh_t;

#define INTELFSH_STATE_SIZE offsetof(intelfsh_t, data)

void intelfsh_init(intelfsh_t *f, uint8_t *data, uint32_t size, uint8_t maker_id, uint8_t device_id,
	void (*erase_delay)(void *user, int seconds), void *user);
void intelfsh_reset(intelfsh_t *f);   // (the device's start state: read array mode)
// 1 while a read gives the array's byte (normal mode: not the status or the IDs)
int intelfsh_reads_data(const intelfsh_t *f);
uint8_t intelfsh_read(intelfsh_t *f, uint32_t address);
void intelfsh_write(intelfsh_t *f, uint32_t address, uint8_t data);
void intelfsh_delay_tick(intelfsh_t *f);

#endif
