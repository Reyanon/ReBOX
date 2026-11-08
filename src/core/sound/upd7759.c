/*  NEC uPD7759 ADPCM speech processor (the stand alone mode: its own ROM). Ported from MAME
    src/devices/sound/upd7759.cpp (BSD-3-Clause, copyright-holders: Juergen Buchmueller, Mike Balfour, Howie Cohen,
    Olivier Galibert, Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "upd7759.h"
#include "../rbx.h"

#include <string.h>

#define FRAC_BITS 20
#define FRAC_ONE (1u << FRAC_BITS)

enum { STATE_IDLE, STATE_DROP_DRQ, STATE_START, STATE_FIRST_REQ, STATE_LAST_SAMPLE, STATE_DUMMY1, STATE_ADDR_MSB,
       STATE_ADDR_LSB, STATE_DUMMY2, STATE_BLOCK_HEADER, STATE_NIBBLE_COUNT, STATE_NIBBLE_MSN, STATE_NIBBLE_LSN };

static const int s_step[16][16] = {
	{ 0,  0,  1,  2,  3,   5,   7,  10,  0,   0,  -1,  -2,  -3,   -5,   -7,  -10 },
	{ 0,  1,  2,  3,  4,   6,   8,  13,  0,  -1,  -2,  -3,  -4,   -6,   -8,  -13 },
	{ 0,  1,  2,  4,  5,   7,  10,  15,  0,  -1,  -2,  -4,  -5,   -7,  -10,  -15 },
	{ 0,  1,  3,  4,  6,   9,  13,  19,  0,  -1,  -3,  -4,  -6,   -9,  -13,  -19 },
	{ 0,  2,  3,  5,  8,  11,  15,  23,  0,  -2,  -3,  -5,  -8,  -11,  -15,  -23 },
	{ 0,  2,  4,  7, 10,  14,  19,  29,  0,  -2,  -4,  -7, -10,  -14,  -19,  -29 },
	{ 0,  3,  5,  8, 12,  16,  22,  33,  0,  -3,  -5,  -8, -12,  -16,  -22,  -33 },
	{ 1,  4,  7, 10, 15,  20,  29,  43, -1,  -4,  -7, -10, -15,  -20,  -29,  -43 },
	{ 1,  4,  8, 13, 18,  25,  35,  53, -1,  -4,  -8, -13, -18,  -25,  -35,  -53 },
	{ 1,  6, 10, 16, 22,  31,  43,  64, -1,  -6, -10, -16, -22,  -31,  -43,  -64 },
	{ 2,  7, 12, 19, 27,  37,  51,  76, -2,  -7, -12, -19, -27,  -37,  -51,  -76 },
	{ 2,  9, 16, 24, 34,  46,  64,  96, -2,  -9, -16, -24, -34,  -46,  -64,  -96 },
	{ 3, 11, 19, 29, 41,  57,  79, 117, -3, -11, -19, -29, -41,  -57,  -79, -117 },
	{ 4, 13, 24, 36, 50,  69,  96, 143, -4, -13, -24, -36, -50,  -69,  -96, -143 },
	{ 4, 16, 29, 44, 62,  85, 118, 175, -4, -16, -29, -44, -62,  -85, -118, -175 },
	{ 6, 20, 36, 54, 76, 104, 144, 214, -6, -20, -36, -54, -76, -104, -144, -214 },
};
static const int s_state_table[16] = { -1, -1, 0, 0, 1, 2, 2, 3, -1, -1, 0, 0, 1, 2, 2, 3 };

static inline uint8_t read_byte(const upd7759_t *u, uint32_t a)
{
	a &= 0x1ffff;   // (the 17 address bits)
	return a < u->rom_size ? u->rom[a] : 0;
}

void upd7759_init(upd7759_t *u, const uint8_t *rom, uint32_t rom_size, uint32_t clock, uint32_t rate)
{
	memset(u, 0, sizeof(*u));
	u->rom = rom;
	u->rom_size = rom_size;
	u->clock = clock;
	u->step = (uint32_t)(((uint64_t)clock << FRAC_BITS) / rate);   // (the chip's clocks a sample, MAME's 4 at clock / 4)
	u->reset = 1;
	u->start = 1;
	upd7759_reset(u);
}

void upd7759_reset(upd7759_t *u)
{
	u->pos = 0;
	u->state = STATE_IDLE;
	u->clocks_left = 0;
	u->nibbles_left = 0;
	u->repeat_count = 0;
	u->post_drq_state = STATE_IDLE;
	u->post_drq_clocks = 0;
	u->req_sample = 0;
	u->last_sample = 0;
	u->block_header = 0;
	u->sample_rate = 0;
	u->first_valid_header = 0;
	u->offset = 0;
	u->repeat_offset = 0;
	u->adpcm_state = 0;
	u->adpcm_data = 0;
	u->sample = 0;
	u->drq = 0;
}

static void update_adpcm(upd7759_t *u, int data)
{
	u->sample = (int16_t)(u->sample + s_step[u->adpcm_state][data]);
	int st = u->adpcm_state + s_state_table[data];
	u->adpcm_state = (int8_t)(st < 0 ? 0 : st > 15 ? 15 : st);
}

static void advance_state(upd7759_t *u)   // (the stand alone mode: the sample offset shift 1)
{
	switch (u->state) {
	case STATE_IDLE:
		u->clocks_left = 4;
		break;
	case STATE_DROP_DRQ:
		u->drq = 0;
		u->clocks_left = u->post_drq_clocks;
		u->state = u->post_drq_state;
		break;
	case STATE_START:
		u->req_sample = u->fifo_in;
		u->clocks_left = 70;   // (+ the start delay: 0)
		u->state = STATE_FIRST_REQ;
		break;
	case STATE_FIRST_REQ:
		u->drq = 1;
		u->clocks_left = 44;
		u->state = STATE_LAST_SAMPLE;
		break;
	case STATE_LAST_SAMPLE:
		u->last_sample = read_byte(u, 0);
		u->drq = 1;
		u->clocks_left = 28;
		u->state = u->req_sample > u->last_sample ? STATE_IDLE : STATE_DUMMY1;
		break;
	case STATE_DUMMY1:
		u->drq = 1;
		u->clocks_left = 32;
		u->state = STATE_ADDR_MSB;
		break;
	case STATE_ADDR_MSB:
		u->offset = (uint32_t)read_byte(u, (uint32_t)u->req_sample * 2 + 5) << (8 + 1);
		u->drq = 1;
		u->clocks_left = 44;
		u->state = STATE_ADDR_LSB;
		break;
	case STATE_ADDR_LSB:
		u->offset |= (uint32_t)read_byte(u, (uint32_t)u->req_sample * 2 + 6) << 1;
		u->drq = 1;
		u->clocks_left = 36;
		u->state = STATE_DUMMY2;
		break;
	case STATE_DUMMY2:
		u->offset++;
		u->first_valid_header = 0;
		u->drq = 1;
		u->clocks_left = 36;
		u->state = STATE_BLOCK_HEADER;
		break;
	case STATE_BLOCK_HEADER:
		if (u->repeat_count) {
			u->repeat_count--;
			u->offset = u->repeat_offset;
		}
		u->block_header = read_byte(u, u->offset++);
		u->drq = 1;
		switch (u->block_header & 0xc0) {
		case 0x00:   // (silence)
			u->clocks_left = 1024 * ((u->block_header & 0x3f) + 1);
			u->state = (u->block_header == 0 && u->first_valid_header) ? STATE_IDLE : STATE_BLOCK_HEADER;
			u->sample = 0;
			u->adpcm_state = 0;
			break;
		case 0x40:   // (256 nibbles)
			u->sample_rate = (uint8_t)((u->block_header & 0x3f) + 1);
			u->nibbles_left = 256;
			u->clocks_left = 36;
			u->state = STATE_NIBBLE_MSN;
			break;
		case 0x80:   // (n nibbles)
			u->sample_rate = (uint8_t)((u->block_header & 0x3f) + 1);
			u->clocks_left = 36;
			u->state = STATE_NIBBLE_COUNT;
			break;
		case 0xc0:   // (repeat loop)
			u->repeat_count = (uint8_t)((u->block_header & 7) + 1);
			u->repeat_offset = u->offset;
			u->clocks_left = 36;
			u->state = STATE_BLOCK_HEADER;
			break;
		}
		if (u->block_header != 0)
			u->first_valid_header = 1;
		break;
	case STATE_NIBBLE_COUNT:
		u->nibbles_left = (uint16_t)(read_byte(u, u->offset++) + 1);
		u->drq = 1;
		u->clocks_left = 36;
		u->state = STATE_NIBBLE_MSN;
		break;
	case STATE_NIBBLE_MSN:
		u->adpcm_data = read_byte(u, u->offset++);
		update_adpcm(u, u->adpcm_data >> 4);
		u->drq = 1;
		u->clocks_left = u->sample_rate * 4;
		u->state = --u->nibbles_left == 0 ? STATE_BLOCK_HEADER : STATE_NIBBLE_LSN;
		break;
	case STATE_NIBBLE_LSN:
		update_adpcm(u, u->adpcm_data & 15);
		u->clocks_left = u->sample_rate * 4;
		u->state = --u->nibbles_left == 0 ? STATE_BLOCK_HEADER : STATE_NIBBLE_MSN;
		break;
	}
	if (u->drq) {
		u->post_drq_state = u->state;
		u->post_drq_clocks = u->clocks_left - 21;
		u->state = STATE_DROP_DRQ;
		u->clocks_left = 21;
	}
}

void upd7759_reset_w(upd7759_t *u, int state)
{
	const uint8_t old = u->reset;
	u->reset = state != 0;
	if (old && !u->reset)
		upd7759_reset(u);
}

void upd7759_start_w(upd7759_t *u, int state)
{
	const uint8_t old = u->start;
	u->start = state != 0;
	if (u->state == STATE_IDLE && old && !u->start && u->reset)
		u->state = STATE_START;
}

void upd7759_port_w(upd7759_t *u, uint8_t data) { u->fifo_in = data; }

int upd7759_busy_r(const upd7759_t *u) { return u->state == STATE_IDLE; }

void upd7759_render(upd7759_t *u, int32_t *out, int n)   // (sound_stream_update: the sample held between states)
{
	int32_t clocks_left = u->clocks_left;
	uint32_t pos = u->pos;
	int32_t sample = u->sample * 128;
	int i = 0;
	if (u->state != STATE_IDLE)
		for (; i < n; i++) {
			out[i] = sample;
			pos += u->step;
			while (pos >= FRAC_ONE) {
				int this_time = (int)(pos >> FRAC_BITS);
				if (this_time > clocks_left)
					this_time = clocks_left;
				pos -= (uint32_t)this_time * FRAC_ONE;
				clocks_left -= this_time;
				if (clocks_left == 0) {
					u->clocks_left = 0;
					advance_state(u);
					if (u->state == STATE_IDLE)
						break;
					clocks_left = u->clocks_left;
					sample = u->sample * 128;
				}
			}
			if (u->state == STATE_IDLE) {
				i++;
				break;
			}
		}
	for (; i < n; i++)
		out[i] = 0;
	u->clocks_left = clocks_left;
	u->pos = pos;
}

int upd7759_state(upd7759_t *u, void *ctx, int load, const char *tag)
{
	(void)load;
	return rbx_state_io(ctx, tag, u, (uint32_t)((const uint8_t *)&u->state_end - (const uint8_t *)u));
}
