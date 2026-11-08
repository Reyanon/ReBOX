/*  Irem GA20 PCM. Ported from MAME src/devices/sound/iremga20.cpp (BSD-3-Clause, copyright-holders: Acho A. Tang,
    R. Belmont, Valley Bell).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef IREMGA20_H
#define IREMGA20_H

#include <stdint.h>

typedef struct {
	uint32_t rate, pos, counter, end, volume, play;
} ga20_channel;

typedef struct {
	uint8_t regs[0x20];
	ga20_channel ch[4];
	uint8_t state_end;
	const uint8_t *rom;
	uint32_t mask;
} ga20_t;

#define GA20_STATE_SIZE ((uint32_t)offsetof(ga20_t, state_end))

void ga20_init(ga20_t *c, const uint8_t *rom, uint32_t size);
void ga20_reset(ga20_t *c);
void ga20_write(ga20_t *c, uint32_t offset, uint8_t data);
uint8_t ga20_read(ga20_t *c, uint32_t offset);
// add n output samples to out; an output sample is the average of `over` samples of the chip (clock / 4)
void ga20_render(ga20_t *c, int32_t *out, int n, int over);

#endif
