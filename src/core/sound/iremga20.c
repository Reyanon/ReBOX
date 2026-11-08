/*  Irem GA20 PCM (iremga20.h). Ported from MAME src/devices/sound/iremga20.cpp (BSD-3-Clause, copyright-holders:
    Acho A. Tang, R. Belmont, Valley Bell). The chip's 900 kHz loop walked by whole steps of a channel (a byte stays for
    0x100 - rate samples), down to the board's output rate by averaging.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include <stddef.h>
#include <string.h>

#include "iremga20.h"

void ga20_init(ga20_t *c, const uint8_t *rom, uint32_t size)
{
	memset(c, 0, sizeof(*c));
	c->rom = rom;
	uint32_t m = 1;
	while (m < size)
		m <<= 1;
	c->mask = m - 1;
}

void ga20_reset(ga20_t *c)
{
	memset(c->regs, 0, sizeof(c->regs));
	memset(c->ch, 0, sizeof(c->ch));
}

void ga20_write(ga20_t *c, uint32_t offset, uint8_t data)
{
	offset &= 0x1f;
	c->regs[offset] = data;
	const int n = (int)(offset >> 3);
	ga20_channel *ch = &c->ch[n];
	switch (offset & 7) {
	case 4:
		ch->rate = data;
		break;
	case 5:
		ch->volume = (data * 256u) / (data + 10u);
		break;
	case 6:
		if (data & 2) {   // (d1: key on / off)
			ch->play = 1;
			ch->pos = (uint32_t)(c->regs[n << 3 | 0] | c->regs[n << 3 | 1] << 8) << 4;
			ch->end = (uint32_t)(c->regs[n << 3 | 2] | c->regs[n << 3 | 3] << 8) << 4;
			ch->counter = 0x100;
		} else
			ch->play = 0;
		break;
	}
}

uint8_t ga20_read(ga20_t *c, uint32_t offset)
{
	offset &= 0x1f;
	return (offset & 7) == 7 ? (uint8_t)c->ch[offset >> 3].play : 0;   // (voice status)
}

void ga20_render(ga20_t *c, int32_t *out, int n, int over)
{
	const uint8_t *rom = c->rom;
	const uint32_t mask = c->mask;
	for (int k = 0; k < 4; k++) {
		ga20_channel *ch = &c->ch[k];
		if (!ch->play)
			continue;
		uint32_t pos = ch->pos, counter = ch->counter;
		const uint32_t rate = ch->rate;
		const int32_t vol = (int32_t)ch->volume;
		int left = over, i = 0;
		int32_t acc = 0;
		while (i < n) {
			const int s = rom ? rom[pos & mask] : 0;
			if (s == 0) {   // (sample end marker)
				ch->play = 0;
				break;
			}
			int to_step = counter > rate ? (int)(counter - rate) : 1;
			const int take = to_step < left ? to_step : left;
			acc += (s - 0x80) * vol * take;
			counter -= (uint32_t)take;
			if (take == to_step) {
				pos++;
				counter = 0x100;
			}
			left -= take;
			if (left == 0) {
				out[i++] += acc / (over * 4);   // (put_int with 32768 * 4)
				acc = 0;
				left = over;
			}
		}
		if (left != over && i < n)
			out[i] += acc / (over * 4);
		ch->pos = pos;
		ch->counter = counter;
	}
}
