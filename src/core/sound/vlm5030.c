/*  Sanyo VLM5030 LPC speech synthesizer. Ported from MAME src/devices/sound/vlm5030.cpp (BSD-3-Clause,
    copyright-holders: Tatsuyuki Satoh) and its coefficient table from src/devices/sound/tms5110r.hxx (BSD-3-Clause,
    copyright-holders: Frank Palazzolo, Couriersud, Jonathan Gevaryahu).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "vlm5030.h"
#include "../rbx.h"

#include <string.h>

// interpolator per frame
#define FR_SIZE 4
// samples per interpolator
#define IP_SIZE_SLOWER (240 / FR_SIZE)
#define IP_SIZE_SLOW   (200 / FR_SIZE)
#define IP_SIZE_NORMAL (160 / FR_SIZE)
#define IP_SIZE_FAST   (120 / FR_SIZE)
#define IP_SIZE_FASTER (80 / FR_SIZE)

enum { PH_RESET, PH_IDLE, PH_SETUP, PH_WAIT, PH_RUN, PH_STOP, PH_END };

static const int speed_table[8] = {
	IP_SIZE_NORMAL, IP_SIZE_FAST, IP_SIZE_FASTER, IP_SIZE_FASTER,
	IP_SIZE_NORMAL, IP_SIZE_SLOWER, IP_SIZE_SLOW, IP_SIZE_SLOW
};

// vlm5030_coeff of tms5110r.hxx (the chirp and interpolation tables are not used by the VLM5030)
#define PITCH_BITS  5
#define ENERGY_BITS 5
static const int kbits[10] = { 6, 5, 4, 4, 3, 3, 3, 3, 3, 3 };
static const uint16_t energytable[32] = {
	0,  1,  2,  3,  5,  6,  7,  9,
	11, 13, 15, 17, 19, 22, 24, 27,
	31, 34, 38, 42, 47, 51, 57, 62,
	68, 75, 82, 89, 98, 107, 116, 127
};
static const uint16_t pitchtable[32] = {
	0,  21, 22, 23, 24, 25, 26, 27,
	28, 29, 31, 33, 35, 37, 39, 41,
	43, 45, 49, 53, 57, 61, 65, 69,
	73, 77, 85, 93, 101, 109, 117, 125
};
static const int16_t ktable[10][64] = {
	// K1
	{ 390, 403, 414, 425, 434, 443, 450, 457,
	  463, 469, 474, 478, 482, 485, 488, 491,
	  494, 496, 498, 499, 501, 502, 503, 504,
	  505, 506, 507, 507, 508, 508, 509, 509,
	  -390, -376, -360, -344, -325, -305, -284, -261,
	  -237, -211, -183, -155, -125, -95, -64, -32,
	  0, 32, 64, 95, 125, 155, 183, 211,
	  237, 261, 284, 305, 325, 344, 360, 376 },
	// K2
	{ 0, 50, 100, 149, 196, 241, 284, 325,
	  362, 396, 426, 452, 473, 490, 502, 510,
	  0, -510, -502, -490, -473, -452, -426, -396,
	  -362, -325, -284, -241, -196, -149, -100, -50 },
	// K3
	{ 0, 64, 128, 192, 256, 320, 384, 448,
	  -512, -448, -384, -320, -256, -192, -128, -64 },
	// K4
	{ 0, 64, 128, 192, 256, 320, 384, 448,
	  -512, -448, -384, -320, -256, -192, -128, -64 },
	// K5 .. K10
	{ 0, 128, 256, 384, -512, -384, -256, -128 },
	{ 0, 128, 256, 384, -512, -384, -256, -128 },
	{ 0, 128, 256, 384, -512, -384, -256, -128 },
	{ 0, 128, 256, 384, -512, -384, -256, -128 },
	{ 0, 128, 256, 384, -512, -384, -256, -128 },
	{ 0, 128, 256, 384, -512, -384, -256, -128 },
};

// (device_rom_interface<16>: the 16-bit space, the data mirrored over it)
static inline int read_byte(const vlm5030_t *v, int offset)
{
	if (!v->rom || !v->rom_size)
		return 0;
	return v->rom[(uint32_t)offset & 0xffff & (v->rom_size - 1)];
}

static uint32_t machine_rand(vlm5030_t *v)
{
	v->rand_seed = 1664525u * v->rand_seed + 1013904223u;
	return (v->rand_seed >> 16) | (v->rand_seed << 16);
}

static void setup_parameter(vlm5030_t *v, uint8_t param)
{
	v->parameter = param;
	// bit 0,1 : 4800bps / 9600bps , interpolator step
	if (param & 2)
		v->interp_step = 4;   // 9600bps : no interpolator
	else if (param & 1)
		v->interp_step = 2;   // 4800bps : 2 interpolator
	else
		v->interp_step = 1;   // 2400bps : 4 interpolator
	// bit 3,4,5 : speed (frame size)
	v->frame_size = speed_table[(param >> 3) & 7];
	// bit 6,7 : low / high pitch
	if (param & 0x80)
		v->pitch_offset = -8;
	else if (param & 0x40)
		v->pitch_offset = 8;
	else
		v->pitch_offset = 0;
}

void vlm5030_reset(vlm5030_t *v)
{
	v->phase = PH_RESET;
	v->address = 0;
	v->vcu_addr_h = 0;
	v->pin_bsy = 0;
	v->old_energy = v->old_pitch = 0;
	v->new_energy = v->new_pitch = 0;
	v->current_energy = v->current_pitch = 0;
	v->target_energy = v->target_pitch = 0;
	memset(v->old_k, 0, sizeof(v->old_k));
	memset(v->new_k, 0, sizeof(v->new_k));
	memset(v->current_k, 0, sizeof(v->current_k));
	memset(v->target_k, 0, sizeof(v->target_k));
	v->interp_count = v->sample_count = v->pitch_count = 0;
	memset(v->x, 0, sizeof(v->x));
	setup_parameter(v, 0x00);
}

void vlm5030_init(vlm5030_t *v, uint32_t clock, const uint8_t *rom, uint32_t rom_size)
{
	memset(v, 0, sizeof(*v));
	v->clock = clock;
	v->rom = rom;
	v->rom_size = rom_size;
	v->pin_rst = v->pin_st = v->pin_vcu = 0;
	v->latch_data = 0;
	v->rand_seed = 0x9d14abd7;   // (MAME's default seed of running_machine)
	vlm5030_reset(v);
	v->phase = PH_IDLE;
}

uint32_t vlm5030_rate(const vlm5030_t *v)
{
	return v->clock / 440;
}

static int get_bits(const vlm5030_t *v, int sbit, int bits)
{
	const int offset = v->address + (sbit >> 3);
	int data = read_byte(v, offset) | (read_byte(v, offset + 1) << 8);
	data >>= (sbit & 7);
	data &= (0xff >> (8 - bits));
	return data;
}

// get next frame
static int parse_frame(vlm5030_t *v)
{
	// remember previous frame
	v->old_energy = v->new_energy;
	v->old_pitch = v->new_pitch;
	for (int i = 0; i <= 9; i++)
		v->old_k[i] = v->new_k[i];

	// command byte check
	uint8_t cmd = (uint8_t)read_byte(v, v->address);
	if (cmd & 0x01) {
		// extend frame
		v->new_energy = v->new_pitch = 0;
		for (int i = 0; i <= 9; i++)
			v->new_k[i] = 0;
		v->address++;
		if (cmd & 0x02)
			return 0;   // end of speech
		// silent frame
		int nums = ((cmd >> 2) + 1) * 2;
		return nums * FR_SIZE;
	}

	v->new_pitch = (uint8_t)pitchtable[get_bits(v, 1, PITCH_BITS)];
	if (v->new_pitch > 0)
		v->new_pitch = (uint8_t)(v->new_pitch + v->pitch_offset);
	v->new_energy = energytable[get_bits(v, 6, ENERGY_BITS)];

	// 10 K's
	v->new_k[9] = ktable[9][get_bits(v, 11, kbits[9])];
	v->new_k[8] = ktable[8][get_bits(v, 14, kbits[8])];
	v->new_k[7] = ktable[7][get_bits(v, 17, kbits[7])];
	v->new_k[6] = ktable[6][get_bits(v, 20, kbits[6])];
	v->new_k[5] = ktable[5][get_bits(v, 23, kbits[5])];
	v->new_k[4] = ktable[4][get_bits(v, 26, kbits[4])];
	v->new_k[3] = ktable[3][get_bits(v, 29, kbits[3])];
	v->new_k[2] = ktable[2][get_bits(v, 33, kbits[2])];
	v->new_k[1] = ktable[1][get_bits(v, 37, kbits[1])];
	v->new_k[0] = ktable[0][get_bits(v, 42, kbits[0])];

	v->address += 6;
	return FR_SIZE;
}

int vlm5030_bsy(vlm5030_t *v)
{
	return v->pin_bsy;
}

void vlm5030_data_w(vlm5030_t *v, uint8_t data)
{
	v->latch_data = data;
}

// RST pin : reset / set table address A8-A15
void vlm5030_rst(vlm5030_t *v, int state)
{
	state = state ? 1 : 0;
	if (v->pin_rst != state) {
		v->pin_rst = (uint8_t)state;
		if (!state)
			setup_parameter(v, v->latch_data);   // H -> L : latch parameters
		else if (v->pin_bsy)
			vlm5030_reset(v);                    // L -> H : reset chip
	}
}

// VCU pin : direct mode / indirect mode
void vlm5030_vcu(vlm5030_t *v, int state)
{
	v->pin_vcu = state ? 1 : 0;
}

// ST pin : set table address A0-A7 / start speech
void vlm5030_st(vlm5030_t *v, int state)
{
	state = state ? 1 : 0;
	if (v->pin_st == state)
		return;
	v->pin_st = (uint8_t)state;
	if (!state) {
		// H -> L
		if (v->pin_vcu) {
			// direct access mode & address High
			v->vcu_addr_h = (uint16_t)(((int)v->latch_data << 8) + 0x01);
		} else {
			// start speech
			if (v->vcu_addr_h) {
				// direct access mode
				v->address = (uint16_t)((v->vcu_addr_h & 0xff00) + v->latch_data);
				v->vcu_addr_h = 0;
			} else {
				// indirect access mode
				int table = (v->latch_data & 0xfe) + (((int)v->latch_data & 1) << 8);
				v->address = (uint16_t)((read_byte(v, table) << 8) | read_byte(v, table + 1));
			}
			// reset process status
			v->sample_count = (uint8_t)v->frame_size;
			v->interp_count = FR_SIZE;
			v->phase = PH_RUN;
		}
	} else {
		// L -> H : setup speech , BSY on after 30ms?
		v->phase = PH_SETUP;
		v->sample_count = 1;   // wait time for busy on
		v->pin_bsy = 1;
	}
}

static inline int16_t put_int_clamp(int sample)
{
	// (sample / 512 as a float of [-1, 1) in MAME, clamped to [-512, 511])
	if (sample < -512) sample = -512;
	if (sample > 511) sample = 511;
	return (int16_t)(sample * 64);
}

// (sound_stream_update: n is stream.samples(), including its use in the setup and end phases)
void vlm5030_render(vlm5030_t *v, int16_t *out, int n)
{
	int sampindex = 0;

	if (v->phase == PH_RUN || v->phase == PH_STOP) {
		int u[11];
		for (; sampindex < n; sampindex++) {
			int current_val;

			// check new interpolator or new frame
			if (v->sample_count == 0) {
				if (v->phase == PH_STOP) {
					v->phase = PH_END;
					v->sample_count = 1;
					goto phase_stop;   // continue to end phase
				}
				v->sample_count = (uint8_t)v->frame_size;

				// interpolator changes
				if (v->interp_count == 0) {
					// change to new frame
					v->interp_count = (uint8_t)parse_frame(v);
					if (v->interp_count == 0) {
						// end mark found
						v->interp_count = FR_SIZE;
						v->sample_count = (uint8_t)v->frame_size;   // end -> stop time
						v->phase = PH_STOP;
					}

					// set old target as new start of frame
					v->current_energy = v->old_energy;
					v->current_pitch = v->old_pitch;
					for (int i = 0; i <= 9; i++)
						v->current_k[i] = v->old_k[i];

					if (v->current_energy == 0) {
						// zero energy frame
						v->target_energy = 0;
						v->target_pitch = (uint8_t)v->current_pitch;
						for (int i = 0; i <= 9; i++)
							v->target_k[i] = (int16_t)v->current_k[i];
					} else {
						v->target_energy = v->new_energy;
						v->target_pitch = v->new_pitch;
						for (int i = 0; i <= 9; i++)
							v->target_k[i] = v->new_k[i];
					}
				}

				// next interpolator: 25% , 50% , 75% , 100%
				v->interp_count = (uint8_t)(v->interp_count - v->interp_step);
				int interp_effect = FR_SIZE - (v->interp_count % FR_SIZE);
				v->current_energy = (uint32_t)(v->old_energy + (v->target_energy - v->old_energy) * interp_effect / FR_SIZE);
				if (v->old_pitch > 1)
					v->current_pitch = (uint32_t)(v->old_pitch + (v->target_pitch - v->old_pitch) * interp_effect / FR_SIZE);
				for (int i = 0; i <= 9; i++)
					v->current_k[i] = v->old_k[i] + (v->target_k[i] - v->old_k[i]) * interp_effect / FR_SIZE;
			}

			// calculate digital filter
			if (v->old_energy == 0)
				current_val = 0x00;   // silent
			else if (v->old_pitch <= 1)
				current_val = (int)((machine_rand(v) & 1) ? v->current_energy : 0u - v->current_energy);   // unvoiced
			else
				current_val = (v->pitch_count == 0) ? (int)v->current_energy : 0;   // voiced

			// lattice filter
			u[10] = current_val;
			for (int i = 9; i >= 0; i--)
				u[i] = u[i + 1] - ((-v->current_k[i] * v->x[i]) / 512);
			for (int i = 9; i >= 1; i--)
				v->x[i] = v->x[i - 1] + ((-v->current_k[i - 1] * u[i - 1]) / 512);
			v->x[0] = u[0];

			out[sampindex] = put_int_clamp(u[0]);

			v->sample_count--;
			v->pitch_count++;
			if (v->pitch_count >= v->current_pitch)
				v->pitch_count = 0;
		}
	}

phase_stop:
	// (the rest of the buffer: MAME's stream output is cleared before the update)
	for (int i = sampindex; i < n; i++)
		out[i] = 0;

	switch (v->phase) {
	case PH_SETUP:
		if (v->sample_count <= n) {
			v->sample_count = 0;
			v->phase = PH_WAIT;
		} else {
			v->sample_count = (uint8_t)(v->sample_count - n);
		}
		break;
	case PH_END:
		if (v->sample_count <= n) {
			v->sample_count = 0;
			v->pin_bsy = 0;
			v->phase = PH_IDLE;
		} else {
			v->sample_count = (uint8_t)(v->sample_count - n);
		}
		break;
	}
}

int vlm5030_state(vlm5030_t *v, void *ctx, int load, const char *tag)
{
	// (all of the working state is saved, so MAME's device_post_load rebuild is not needed)
	(void)load;
	return rbx_state_io(ctx, tag, v, (uint32_t)((const uint8_t *)&v->state_end - (const uint8_t *)v));
}
