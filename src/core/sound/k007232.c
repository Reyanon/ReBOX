/*  Konami 007232 PCM controller. Ported from MAME src/devices/sound/k007232.cpp (BSD-3-Clause, copyright-holders:
    Nicola Salmoria, Hiromitsu Shioya).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "k007232.h"
#include "../rbx.h"

#include <string.h>

void k007232_init(k007232_t *k, const uint8_t *rom, uint32_t rom_size, void (*port_write)(uint8_t data))
{
	memset(k, 0, sizeof(*k));
	k->rom = rom;
	k->rom_size = rom_size;
	k->port_write = port_write;
	// (the default mapping: a bank switched ROM when larger than the 17 bits, else the ROM itself and its end the limit)
	k->pcmlimit = rom_size > 0x20000 ? 1u << 17 : rom_size;
	k007232_reset(k);
}

void k007232_reset(k007232_t *k)   // (device_start's set up: MAME has no reset)
{
	for (int i = 0; i < 2; i++) {
		k->ch[i].addr = 0;
		k->ch[i].start = 0;
		k->ch[i].counter = 0x1000;
		k->ch[i].step = 0;
		k->ch[i].play = 0;
		k->ch[i].bank = 0;
	}
	k->ch[0].vol[0] = 255;   // (channel A to output A, B to B)
	k->ch[0].vol[1] = 0;
	k->ch[1].vol[0] = 0;
	k->ch[1].vol[1] = 255;
	memset(k->wreg, 0, sizeof(k->wreg));
}

static inline uint8_t read_sample(const k007232_t *k, int ch, uint32_t addr)
{
	addr &= 0x1ffff;
	if (k->rom_size > 0x20000)   // (read_rom_default)
		return k->rom[(k->ch[ch].bank + addr) & (k->rom_size - 1)];
	return addr < k->rom_size ? k->rom[addr] : 0;
}

static void start(k007232_t *k, int ch)
{
	if (k->ch[ch].start < k->pcmlimit) {
		k->ch[ch].play = 1;
		k->ch[ch].addr = k->ch[ch].start;
		k->ch[ch].counter = 0x1000;
	}
}

void k007232_write(k007232_t *k, int offset, uint8_t data)
{
	offset &= 0x0f;
	k->wreg[offset] = data;
	if (offset == 12) {
		if (k->port_write)
			k->port_write(data);
	} else if (offset < 12) {
		const int c = offset >= 6, ri = c ? 6 : 0;
		switch (offset - ri) {
		case 0: case 1:   // (the address step)
			k->ch[c].step = (uint32_t)((k->wreg[ri + 1] & 0x0f) << 8 | k->wreg[ri]);
			break;
		case 2: case 3: case 4:   // (the start address)
			k->ch[c].start = (uint32_t)((k->wreg[ri + 4] & 1) << 16 | k->wreg[ri + 3] << 8 | k->wreg[ri + 2]);
			break;
		case 5:
			start(k, c);
			break;
		}
	}
}

uint8_t k007232_read(k007232_t *k, int offset)
{
	offset &= 0x0f;
	if (offset == 5 || offset == 11)
		start(k, offset == 11);
	return 0;
}

void k007232_set_volume(k007232_t *k, int channel, int vol_a, int vol_b)
{
	k->ch[channel].vol[0] = (uint8_t)vol_a;
	k->ch[channel].vol[1] = (uint8_t)vol_b;
}

void k007232_set_bank(k007232_t *k, int chan_a_bank, int chan_b_bank)
{
	k->ch[0].bank = (uint32_t)chan_a_bank << 17;
	k->ch[1].bank = (uint32_t)chan_b_bank << 17;
}

void k007232_sample(k007232_t *k, int32_t *out_a, int32_t *out_b)
{
	int32_t lsum = 0, rsum = 0;
	for (int i = 0; i < 2; i++) {
		if (!k->ch[i].play)
			continue;
		const int vol_a = k->ch[i].vol[0] * 2, vol_b = k->ch[i].vol[1] * 2;
		uint32_t addr = k->ch[i].addr & 0x1ffff;
		while ((uint32_t)k->ch[i].counter <= k->ch[i].step) {   // (MAME: int <= u32, an unsigned compare)
			if (addr >= k->pcmlimit || (read_sample(k, i, addr++) & 0x80)) {   // (the end)
				if (k->wreg[13] >> i & 1)
					addr = k->ch[i].start;   // (looped)
				else {
					k->ch[i].play = 0;
					break;
				}
			}
			k->ch[i].counter += (int32_t)(0x1000 - k->ch[i].step);
		}
		k->ch[i].addr = addr;
		if (!k->ch[i].play)
			break;   // (MAME: the second channel not mixed in this sample when the first stopped)
		k->ch[i].counter -= 32;
		const int out = (read_sample(k, i, addr) & 0x7f) - 0x40;
		lsum += out * vol_a;
		rsum += out * vol_b;
	}
	*out_a = lsum;
	*out_b = rsum;
}

int k007232_state(k007232_t *k, void *ctx, int load, const char *tag)
{
	(void)load;
	return rbx_state_io(ctx, tag, k, (uint32_t)((const uint8_t *)&k->state_end - (const uint8_t *)k));
}
