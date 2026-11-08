/*  OKI MSM6295 ADPCM, ported from MAME src/devices/sound/okim6295.cpp and okiadpcm.cpp
    (BSD-3-Clause, copyright-holders: Mirko Buffoni, Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "okim6295.h"

#include <math.h>
#include <string.h>

static const int8_t s_index_shift[8] = { -1, -1, -1, -1, 2, 4, 6, 8 };
static int s_diff[49 * 16];
static int s_tables;
static const int s_volume[16] = { 0x20, 0x16, 0x10, 0x0b, 0x08, 0x06, 0x04, 0x03, 0x02, 0, 0, 0, 0, 0, 0, 0 };

static void tables(void)
{
	static const int8_t nbl2bit[16][4] = {
		{ 1, 0, 0, 0 }, { 1, 0, 0, 1 }, { 1, 0, 1, 0 }, { 1, 0, 1, 1 },
		{ 1, 1, 0, 0 }, { 1, 1, 0, 1 }, { 1, 1, 1, 0 }, { 1, 1, 1, 1 },
		{ -1, 0, 0, 0 }, { -1, 0, 0, 1 }, { -1, 0, 1, 0 }, { -1, 0, 1, 1 },
		{ -1, 1, 0, 0 }, { -1, 1, 0, 1 }, { -1, 1, 1, 0 }, { -1, 1, 1, 1 }
	};
	if (s_tables)
		return;
	s_tables = 1;
	for (int step = 0; step <= 48; step++) {
		int stepval = (int)floor(16.0 * pow(11.0 / 10.0, (double)step));
		for (int nib = 0; nib < 16; nib++)
			s_diff[step * 16 + nib] = nbl2bit[nib][0] *
				(stepval * nbl2bit[nib][1] + stepval / 2 * nbl2bit[nib][2] + stepval / 4 * nbl2bit[nib][3] + stepval / 8);
	}
}

static uint8_t rd(okim6295_t *o, uint32_t a)
{
	a = (a & 0x3ffff) + o->bank;
	return o->rom && a < o->rom_size ? o->rom[a] : 0;
}

void oki_init(okim6295_t *o, uint32_t clock, int pin7_high, const uint8_t *rom, uint32_t size, int gain256)
{
	tables();
	memset(o, 0, sizeof(*o));
	o->rom = rom;
	o->rom_size = size;
	o->rate = clock / (pin7_high ? 132 : 165);
	o->gain = gain256;
	oki_reset(o);
}

void oki_reset(okim6295_t *o)
{
	o->command = -1;
	for (int i = 0; i < 4; i++)
		o->v[i].playing = 0;
}

void oki_write(okim6295_t *o, uint8_t command)
{
	if (o->command != -1) {
		int mask = command >> 4;
		for (int n = 0; n < 4; n++, mask >>= 1) {
			if (!(mask & 1))
				continue;
			oki_voice *v = &o->v[n];
			if (v->playing)
				continue;
			uint32_t base = o->command * 8;
			uint32_t start = (rd(o, base) << 16 | rd(o, base + 1) << 8 | rd(o, base + 2)) & 0x3ffff;
			uint32_t stop = (rd(o, base + 3) << 16 | rd(o, base + 4) << 8 | rd(o, base + 5)) & 0x3ffff;
			if (start < stop) {
				v->playing = 1;
				v->base = start;
				v->sample = 0;
				v->count = 2 * (stop - start + 1);
				v->signal = -2;
				v->step = 0;
				v->volume = s_volume[command & 0x0f];
			}
		}
		o->command = -1;
	} else if (command & 0x80) {
		o->command = command & 0x7f;
	} else {
		int mask = command >> 3;
		for (int n = 0; n < 4; n++, mask >>= 1)
			if (mask & 1)
				o->v[n].playing = 0;
	}
}

uint8_t oki_read(okim6295_t *o)
{
	uint8_t r = 0xf0;
	for (int n = 0; n < 4; n++)
		if (o->v[n].playing)
			r |= 1 << n;
	return r;
}

int32_t oki_sample(okim6295_t *o)
{
	int32_t out = 0;
	for (int n = 0; n < 4; n++) {
		oki_voice *v = &o->v[n];
		if (!v->playing)
			continue;
		int nibble = rd(o, v->base + v->sample / 2) >> (((v->sample & 1) << 2) ^ 4);
		v->signal += s_diff[v->step * 16 + (nibble & 15)];
		if (v->signal > 2047) v->signal = 2047;
		else if (v->signal < -2048) v->signal = -2048;
		v->step += s_index_shift[nibble & 7];
		if (v->step > 48) v->step = 48;
		else if (v->step < 0) v->step = 0;
		// MAME: signal * volume / 2048 of full scale -> 16-bit: signal * volume / 2
		out += v->signal * v->volume / 2;
		if (++v->sample >= v->count)
			v->playing = 0;
	}
	return (out * o->gain) >> 8;
}
