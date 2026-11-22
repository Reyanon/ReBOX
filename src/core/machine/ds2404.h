/*  Dallas DS2404 EconoRAM Time Chip (RTC + 512 bytes of SRAM) through the parallel interface of the Seibu SEI600
    (MAME's), ported from MAME 0.289 src/devices/machine/ds2404.cpp, ds2404.h (BSD-3-Clause, copyright-holders:
    Aaron Giles).
    The RTC counts 1/256 s ticks: the owner calls ds2404_tick 256 times a second (MAME's timer). It starts at 0
    seconds after the reference date rather than at the host's time (MAME reads the host clock): the runs are
    repeatable. The SRAM starts cleared (MAME's nvram default) and is not kept between runs.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_DS2404_H
#define RBX_DS2404_H

#include <stdint.h>

typedef struct {
	uint16_t address;
	uint16_t offset;
	uint16_t end_offset;
	uint8_t a1, a2;
	uint8_t sram[512];   // 4096 bits
	uint8_t ram[32];     // scratchpad ram, 256 bits
	uint8_t rtc[5];      // 40-bit RTC counter
	uint8_t state[8];
	int32_t state_ptr;
} ds2404_t;

void ds2404_init(ds2404_t *d);   // (device_start, nvram_default)
void ds2404_1w_reset_w(ds2404_t *d);
void ds2404_3w_reset_w(ds2404_t *d);
uint8_t ds2404_data_r(ds2404_t *d);
void ds2404_data_w(ds2404_t *d, uint8_t data);
void ds2404_clk_w(ds2404_t *d);
void ds2404_tick(ds2404_t *d);

#endif
