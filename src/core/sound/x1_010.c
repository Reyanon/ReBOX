/*  Seta X1-010. Ported from MAME src/devices/sound/x1_010.cpp (BSD-3-Clause, copyright-holders: Luca Elia).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "x1_010.h"

#include <string.h>

#define VOL_BASE (2 * 32 * 256 / 30)

// a channel's registers (X1_010_CHANNEL)
enum { R_STATUS, R_VOLUME, R_FREQUENCY, R_PITCH_HI, R_START, R_END, R_SIZE = 8 };

void x1_010_init(x1_010_t *x, const uint8_t *rom, uint32_t rom_size)
{
	memset(x, 0, sizeof(*x));
	x->rom = rom;
	x->rom_mask = rom_size - 1;
}

void x1_010_reset(x1_010_t *x)
{
	memset(x->reg, 0, sizeof(x->reg));
	memset(x->hi_word, 0, sizeof(x->hi_word));
	memset(x->smp_offset, 0, sizeof(x->smp_offset));
	memset(x->env_offset, 0, sizeof(x->env_offset));
}

uint8_t x1_010_read(const x1_010_t *x, uint32_t offset) { return x->reg[offset & 0x1fff]; }

void x1_010_write(x1_010_t *x, uint32_t offset, uint8_t data)
{
	offset &= 0x1fff;
	const uint32_t channel = offset / R_SIZE, reg = offset % R_SIZE;
	if (channel < X1_010_CHANNELS && reg == R_STATUS && !(x->reg[offset] & 1) && (data & 1)) {   // (key on)
		x->smp_offset[channel] = 0;
		x->env_offset[channel] = 0;
	}
	x->reg[offset] = data;
}

uint16_t x1_010_word_r(const x1_010_t *x, uint32_t offset)
{
	offset &= 0x1fff;
	return (uint16_t)(x->hi_word[offset] << 8 | x->reg[offset]);
}

void x1_010_word_w(x1_010_t *x, uint32_t offset, uint16_t data)
{
	offset &= 0x1fff;
	x->hi_word[offset] = (uint8_t)(data >> 8);
	x1_010_write(x, offset, (uint8_t)data);
}

void x1_010_render(x1_010_t *x, int32_t *l, int32_t *r, int n)
{
	for (int ch = 0; ch < X1_010_CHANNELS; ch++) {
		uint8_t *reg = &x->reg[ch * R_SIZE];
		const int status = reg[R_STATUS];
		if (!(status & 1))   // (key off)
			continue;
		const int div = (status & 0x80) ? 1 : 0;
		if (!(status & 2)) {   // PCM sampling
			const uint32_t start = (uint32_t)reg[R_START] << 12, end = (uint32_t)(0x100 - reg[R_END]) << 12;
			const int vol_l = ((reg[R_VOLUME] >> 4) & 0xf) * VOL_BASE, vol_r = (reg[R_VOLUME] & 0xf) * VOL_BASE;
			uint32_t smp_offs = x->smp_offset[ch];
			int freq = reg[R_FREQUENCY] >> div;
			if (freq == 0)   // (as MAME)
				freq = 4;
			for (int i = 0; i < n; i++) {
				const uint32_t delta = smp_offs >> 4;
				if (start + delta >= end) {
					reg[R_STATUS] &= 0xfe;
					break;
				}
				const int data = (int8_t)x->rom[(start + delta) & x->rom_mask];
				l[i] += (data * vol_l) >> 8;
				r[i] += (data * vol_r) >> 8;
				smp_offs += (uint32_t)freq;
			}
			x->smp_offset[ch] = smp_offs;
		} else {   // wave form
			const uint32_t start = ((uint32_t)reg[R_VOLUME] << 7) + 0x1000;
			uint32_t smp_offs = x->smp_offset[ch], env_offs = x->env_offset[ch];
			const uint32_t smp_step = (uint32_t)(((reg[R_PITCH_HI] << 8) + reg[R_FREQUENCY]) >> div);
			const uint32_t env = (uint32_t)reg[R_END] << 7, env_step = reg[R_START];
			for (int i = 0; i < n; i++) {
				const uint32_t delta = env_offs >> 10;
				if ((status & 4) && delta >= 0x80) {   // (the envelope's one shot mode)
					reg[R_STATUS] &= 0xfe;
					break;
				}
				const int vol = x->reg[(env + (delta & 0x7f)) & 0x1fff];
				const int vol_l = ((vol >> 4) & 0xf) * VOL_BASE, vol_r = (vol & 0xf) * VOL_BASE;
				const int data = (int8_t)x->reg[(start + ((smp_offs >> 10) & 0x7f)) & 0x1fff];
				l[i] += (data * vol_l) >> 8;
				r[i] += (data * vol_r) >> 8;
				smp_offs += smp_step;
				env_offs += env_step;
			}
			x->smp_offset[ch] = smp_offs;
			x->env_offset[ch] = env_offs;
		}
	}
}
