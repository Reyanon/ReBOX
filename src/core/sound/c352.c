/*  Namco C352 PCM sound chip. Ported from MAME 0.289 src/devices/sound/c352.cpp (BSD-3-Clause, copyright-holders:
    R. Belmont, superctr). See c352.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "c352.h"
#include <string.h>

enum {
	FLG_BUSY = 0x8000, FLG_KEYON = 0x4000, FLG_KEYOFF = 0x2000, FLG_LOOPTRG = 0x1000, FLG_LOOPHIST = 0x0800,
	FLG_FM = 0x0400, FLG_PHASERL = 0x0200, FLG_PHASEFL = 0x0100, FLG_PHASEFR = 0x0080, FLG_LDIR = 0x0040,
	FLG_LINK = 0x0020, FLG_NOISE = 0x0010, FLG_MULAW = 0x0008, FLG_FILTER = 0x0004, FLG_REVLOOP = 0x0003,
	FLG_LOOP = 0x0002, FLG_REVERSE = 0x0001
};

void c352_init(c352_t *c, const uint8_t *rom, uint32_t rom_size)
{
	memset(c, 0, sizeof(*c));
	c->rom = rom;
	uint32_t m = 1;
	while (m < rom_size)
		m <<= 1;
	c->rom_mask = rom && rom_size ? m - 1 : 0;
	// the mu-law table (output similar to Namco's VC emulator)
	int j = 0;
	for (int i = 0; i < 128; i++) {
		c->mulawtab[i] = (int16_t)(j << 5);
		if (i < 16)
			j += 1;
		else if (i < 24)
			j += 2;
		else if (i < 48)
			j += 4;
		else if (i < 100)
			j += 8;
		else
			j += 16;
	}
	for (int i = 0; i < 128; i++)
		c->mulawtab[i + 128] = (int16_t)((~c->mulawtab[i]) & 0xffe0);
	c352_reset(c);
}

void c352_reset(c352_t *c)
{
	memset(c->v, 0, sizeof(c->v));
	c->random = 0x1234;
	c->control = 0;
}

static inline uint8_t rom_byte(const c352_t *c, uint32_t a)
{
	if (!c->rom)
		return 0;
	a &= 0xffffff & c->rom_mask;
	return c->rom[a];
}

static void fetch_sample(c352_t *c, c352_voice_t *v)
{
	v->last_sample = v->sample;
	if (v->flags & FLG_NOISE) {
		c->random = (uint16_t)((c->random >> 1) ^ ((-(c->random & 1)) & 0xfff6));
		v->sample = (int16_t)c->random;
	} else {
		const int8_t s = (int8_t)rom_byte(c, v->pos);
		if (v->flags & FLG_MULAW)
			v->sample = c->mulawtab[s & 0xff];
		else
			v->sample = (int16_t)(s * 256);
		const uint16_t pos = (uint16_t)(v->pos & 0xffff);
		if ((v->flags & FLG_LOOP) && (v->flags & FLG_REVERSE)) {
			if ((v->flags & FLG_LDIR) && pos == v->wave_loop)   // backwards > forwards
				v->flags &= (uint16_t)~FLG_LDIR;
			else if (!(v->flags & FLG_LDIR) && pos == v->wave_end)   // forwards > backwards
				v->flags |= FLG_LDIR;
			v->pos += (v->flags & FLG_LDIR) ? (uint32_t)-1 : 1;
		} else if (pos == v->wave_end) {
			if ((v->flags & FLG_LINK) && (v->flags & FLG_LOOP)) {
				v->pos = ((uint32_t)v->wave_start << 16) | v->wave_loop;
				v->flags |= FLG_LOOPHIST;
			} else if (v->flags & FLG_LOOP) {
				v->pos = (v->pos & 0xff0000) | v->wave_loop;
				v->flags |= FLG_LOOPHIST;
			} else {
				v->flags |= FLG_KEYOFF;
				v->flags &= (uint16_t)~FLG_BUSY;
				v->sample = 0;
			}
		} else {
			v->pos += (v->flags & FLG_REVERSE) ? (uint32_t)-1 : 1;
		}
	}
}

static inline void ramp_volume(c352_voice_t *v, int ch, uint8_t val)
{
	const int vol_delta = v->curr_vol[ch] - val;
	if (vol_delta != 0)
		v->curr_vol[ch] = (uint8_t)(v->curr_vol[ch] + ((vol_delta > 0) ? -1 : 1));
}

void c352_render(c352_t *c, int32_t *l, int32_t *r, int n)
{
	for (int i = 0; i < n; i++) {
		int out0 = 0, out1 = 0;
		for (int j = 0; j < 32; j++) {
			c352_voice_t *v = &c->v[j];
			int s = 0;
			if (v->flags & FLG_BUSY) {
				const int32_t next_counter = (int32_t)(v->counter + v->freq);
				if (next_counter & 0x10000)
					fetch_sample(c, v);
				if ((next_counter ^ (int32_t)v->counter) & 0x18000) {
					ramp_volume(v, 0, (uint8_t)(v->vol_f >> 8));
					ramp_volume(v, 1, (uint8_t)(v->vol_f & 0xff));
					ramp_volume(v, 2, (uint8_t)(v->vol_r >> 8));
					ramp_volume(v, 3, (uint8_t)(v->vol_r & 0xff));
				}
				v->counter = (uint32_t)next_counter & 0xffff;
				s = v->sample;
				if ((v->flags & FLG_FILTER) == 0)   // interpolate
					s = (int16_t)(v->last_sample + (int)((v->counter * (uint32_t)(v->sample - v->last_sample)) >> 16));   // (MAME: u32 arithmetic)
			}
			out0 += (((v->flags & FLG_PHASEFL) ? -s : s) * v->curr_vol[0]) >> 8;
			out1 += (((v->flags & FLG_PHASEFR) ? -s : s) * v->curr_vol[1]) >> 8;
			// (outputs 2 and 3, the rear, are not connected on the boards here: their volume ramps still run above)
		}
		l[i] += (int16_t)(out0 >> 3);
		r[i] += (int16_t)(out1 >> 3);
	}
}

static uint16_t *reg_ptr(c352_t *c, uint32_t offset)
{
	c352_voice_t *v = &c->v[offset / 8];
	switch (offset % 8) {
	case 0: return &v->vol_f;
	case 1: return &v->vol_r;
	case 2: return &v->freq;
	case 3: return &v->flags;
	case 4: return &v->wave_bank;
	case 5: return &v->wave_start;
	case 6: return &v->wave_end;
	default: return &v->wave_loop;
	}
}

uint16_t c352_read(c352_t *c, uint32_t offset)
{
	if (offset < 0x100)
		return *reg_ptr(c, offset);
	if (offset == 0x200)
		return c->control;
	return 0;
}

void c352_write(c352_t *c, uint32_t offset, uint16_t data, uint16_t mem_mask)
{
	if (offset < 0x100) {
		uint16_t *p = reg_ptr(c, offset);
		*p = (uint16_t)((*p & ~mem_mask) | (data & mem_mask));
	} else if (offset == 0x200) {
		c->control = (uint16_t)((c->control & ~mem_mask) | (data & mem_mask));
	} else if (offset == 0x202) {   // execute key-ons / key-offs (16-bit accesses only)
		if (mem_mask != 0xffff)
			return;
		for (int i = 0; i < 32; i++) {
			c352_voice_t *v = &c->v[i];
			if (v->flags & FLG_KEYON) {
				v->pos = ((uint32_t)v->wave_bank << 16) | v->wave_start;
				v->sample = 0;
				v->last_sample = 0;
				v->counter = 0xffff;
				v->flags |= FLG_BUSY;
				v->flags &= (uint16_t)~(FLG_KEYON | FLG_LOOPHIST);
				v->curr_vol[0] = v->curr_vol[1] = 0;
				v->curr_vol[2] = v->curr_vol[3] = 0;
			}
			if (v->flags & FLG_KEYOFF) {
				v->flags &= (uint16_t)~(FLG_BUSY | FLG_KEYOFF);
				v->counter = 0xffff;
			}
		}
	}
}
