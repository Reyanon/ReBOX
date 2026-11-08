/*  Konami 053260 KDSC PCM sound chip, ported to C from MAME 0.289 src/devices/sound/k053260.cpp (BSD-3-Clause,
    copyright-holders: Ernesto Corvi, Alex W. Jackson). See k053260.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "k053260.h"
#include "../rbx.h"

#include <stdio.h>
#include <string.h>

#define CLOCKS_PER_SAMPLE 64

// Pan multipliers (1.16), MAME pan_mul
static const int pan_mul[8][2] = {
	{ 0, 0 }, { 65536, 0 }, { 59870, 26656 }, { 53684, 37950 },
	{ 46341, 46341 }, { 37950, 53684 }, { 26656, 59870 }, { 0, 65536 }
};

static const int8_t kadpcm_table[16] = { 0, 1, 2, 4, 8, 16, 32, 64, -128, -64, -32, -16, -8, -4, -2, -1 };

static uint32_t gcd32(uint32_t a, uint32_t b)
{
	while (b) { uint32_t t = a % b; a = b; b = t; }
	return a;
}

// device_rom_interface<21>: the address space mirrors the ROM when its size is a power of two
static inline uint8_t rom_byte(const k053260_t *k, uint32_t a)
{
	a &= 0x1fffff;
	if (!k->rom_size) return 0;
	if (!(k->rom_size & (k->rom_size - 1))) return k->rom[a & (k->rom_size - 1)];
	return a < k->rom_size ? k->rom[a] : 0;
}

static void update_pan_volume(k053260_t *k, int i)
{
	k->v[i].pan_volume[0] = k->v[i].volume * pan_mul[k->v[i].pan][0];
	k->v[i].pan_volume[1] = k->v[i].volume * pan_mul[k->v[i].pan][1];
}

void k053260_init(k053260_t *k, const uint8_t *rom, uint32_t rom_size, uint32_t clock)
{
	memset(k, 0, sizeof(*k));
	k->rom = rom;
	k->rom_size = rom_size;
	// time unit 1 / (clock * 48000): a native sample = 64 * 48000, a 48 kHz sample = clock
	uint32_t un = CLOCKS_PER_SAMPLE * 48000u, uo = clock ? clock : 1;
	uint32_t g = gcd32(un, uo);
	k->unit_native = un / g;
	k->unit_out = uo / g;
	k->inv_out = 1.0f / (float)k->unit_out;
	k053260_reset(k);
}

void k053260_reset(k053260_t *k)
{
	// MAME device_reset resets the voices only (ports, keyon, mode keep their values)
	for (int i = 0; i < 4; i++) {
		memset(&k->v[i], 0, sizeof(k->v[i]));
		update_pan_volume(k, i);
	}
	k->hold_l = k->hold_r = 0.0f;
	k->rem = 0;
}

uint8_t k053260_main_read(k053260_t *k, uint32_t offset) { return k->portdata[2 + (offset & 1)]; }
void k053260_main_write(k053260_t *k, uint32_t offset, uint8_t data) { k->portdata[offset & 1] = data; }

uint8_t k053260_read(k053260_t *k, uint32_t offset)
{
	offset &= 0x3f;
	uint8_t ret = 0;
	switch (offset) {
	case 0x00: case 0x01:
		ret = k->portdata[offset];
		break;
	case 0x29:
		for (int i = 0; i < 4; i++) ret |= (uint8_t)(k->v[i].playing << i);
		break;
	case 0x2e:   // ROM readback through voice 0
		if (k->mode & 1) {
			ret = rom_byte(k, k->v[0].start + k->v[0].position);
			k->v[0].position = (k->v[0].position + 1) & 0xffff;
		}
		break;
	default: break;
	}
	return ret;
}

void k053260_write(k053260_t *k, uint32_t offset, uint8_t data)
{
	offset &= 0x3f;
	if (offset >= 0x08 && offset <= 0x27) {
		int i = (int)(offset - 8) / 8;
		switch (offset & 7) {
		case 0: k->v[i].pitch = (uint16_t)((k->v[i].pitch & 0x0f00) | data); break;
		case 1: k->v[i].pitch = (uint16_t)((k->v[i].pitch & 0x00ff) | ((data << 8) & 0x0f00)); break;
		case 2: k->v[i].length = (uint16_t)((k->v[i].length & 0xff00) | data); break;
		case 3: k->v[i].length = (uint16_t)((k->v[i].length & 0x00ff) | (data << 8)); break;
		case 4: k->v[i].start = (k->v[i].start & 0x1fff00) | data; break;
		case 5: k->v[i].start = (k->v[i].start & 0x1f00ff) | ((uint32_t)data << 8); break;
		case 6: k->v[i].start = (k->v[i].start & 0x00ffff) | (((uint32_t)data << 16) & 0x1f0000); break;
		case 7: k->v[i].volume = data & 0x7f; update_pan_volume(k, i); break;
		}
		return;
	}
	switch (offset) {
	case 0x02: case 0x03:
		k->portdata[offset] = data;
		break;
	case 0x28: {   // key on/off and reverse
		uint8_t rising = (uint8_t)(data & ~k->keyon);
		for (int i = 0; i < 4; i++) {
			k->v[i].reverse = (data >> (i | 4)) & 1;
			if ((rising >> i) & 1) {
				k->v[i].position = k->v[i].kadpcm ? 1 : 0;
				k->v[i].counter = 0x1000 - CLOCKS_PER_SAMPLE;
				k->v[i].output = 0;
				k->v[i].playing = 1;
			} else if (!((data >> i) & 1)) {
				k->v[i].position = 0;
				k->v[i].output = 0;
				k->v[i].playing = 0;
			}
		}
		k->keyon = data;
		break;
	}
	case 0x2a:
		for (int i = 0; i < 4; i++) {
			k->v[i].loop = (data >> i) & 1;
			k->v[i].kadpcm = (data >> (i | 4)) & 1;
		}
		break;
	case 0x2c:
		k->v[0].pan = data & 7; update_pan_volume(k, 0);
		k->v[1].pan = (data >> 3) & 7; update_pan_volume(k, 1);
		break;
	case 0x2d:
		k->v[2].pan = data & 7; update_pan_volume(k, 2);
		k->v[3].pan = (data >> 3) & 7; update_pan_volume(k, 3);
		break;
	case 0x2f:
		k->mode = data;
		break;
	default: break;
	}
}

// one native sample (MAME sound_stream_update for one sample), clamped like put_int_clamp
static void step(k053260_t *k)
{
	int32_t out[2] = { 0, 0 };
	if (!(k->mode & 2)) {
		// MAME leaves the stream at zero, and the voices do not advance
		k->hold_l = k->hold_r = 0.0f;
		return;
	}
	for (int i = 0; i < 4; i++) {
		if (!k->v[i].playing) continue;
		struct k053260_voice *v = &k->v[i];
		v->counter = (uint16_t)(v->counter + CLOCKS_PER_SAMPLE);
		int stop = 0;
		while (v->counter >= 0x1000) {
			v->counter = (uint16_t)(v->counter - 0x1000 + v->pitch);
			uint32_t bytepos = ++v->position >> (v->kadpcm ? 1 : 0);
			if (bytepos > v->length) {
				if (v->loop) {
					v->position = 0; v->output = 0; bytepos = 0;
				} else {
					v->playing = 0;
					stop = 1;
					break;
				}
			}
			uint8_t romdata = rom_byte(k, v->start + (v->reverse ? (uint32_t)-bytepos : bytepos));
			if (v->kadpcm) {
				if (v->position & 1) romdata >>= 4;
				v->output = (int8_t)(v->output + kadpcm_table[romdata & 0xf]);
			} else {
				v->output = (int8_t)romdata;
			}
		}
		if (stop) continue;
		out[0] += (v->output * v->pan_volume[0]) >> 15;
		out[1] += (v->output * v->pan_volume[1]) >> 15;
	}
	for (int c = 0; c < 2; c++) {
		if (out[c] < -32768) out[c] = -32768;
		if (out[c] > 32767) out[c] = 32767;
	}
	k->hold_l = (float)out[0];
	k->hold_r = (float)out[1];
}

void k053260_render(k053260_t *k, int32_t *l, int32_t *r, int n)
{
	for (int j = 0; j < n; j++) {
		uint32_t need = k->unit_out;
		float al = 0.0f, ar = 0.0f;
		while (need) {
			if (!k->rem) { step(k); k->rem = k->unit_native; }
			uint32_t take = k->rem < need ? k->rem : need;
			al += k->hold_l * (float)take;
			ar += k->hold_r * (float)take;
			need -= take;
			k->rem -= take;
		}
		l[j] += (int32_t)(al * k->inv_out);
		r[j] += (int32_t)(ar * k->inv_out);
	}
}

int k053260_state(k053260_t *k, void *ctx, int load, const char *prefix)
{
	char t[48];
	int rc = 0;
#define IO(name, p, n) (snprintf(t, sizeof(t), "%s%s", prefix, name), rbx_state_io(ctx, t, p, n))
	rc |= IO("ports", k->portdata, sizeof(k->portdata));
	rc |= IO("km", &k->keyon, 2);
	rc |= IO("v", k->v, sizeof(k->v));
	rc |= IO("hold", &k->hold_l, sizeof(float) * 2);
	rc |= IO("rem", &k->rem, sizeof(k->rem));
#undef IO
	(void)load;
	return rc;
}
