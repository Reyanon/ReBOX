/*  Capcom QSound DL-1425, high level. Ported from MAME src/devices/sound/qsoundhle.cpp (BSD-3-Clause,
    copyright-holders: superctr, Valley Bell).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "qsound.h"
#include <string.h>
#include <stdlib.h>

enum {
	DATA_PAN_TAB = 0x110, DATA_ADPCM_TAB = 0x9dc, DATA_FILTER_TAB = 0xd53, DATA_FILTER_TAB2 = 0xf2e,
	STATE_BOOT = 0x000, STATE_INIT1 = 0x288, STATE_INIT2 = 0x61a, STATE_REFRESH1 = 0x039, STATE_REFRESH2 = 0x04f,
	STATE_NORMAL1 = 0x314, STATE_NORMAL2 = 0x6b2
};
#define PAN_TABLE_DRY 0
#define PAN_TABLE_WET 98
#define PAN_TABLE_CH_OFFSET 196
#define FILTER_ENTRY_SIZE 95
#define DELAY_BASE_OFFSET 0x554
#define DELAY_BASE_OFFSET2 0x53c

static inline int32_t clamp32(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : v > hi ? hi : v; }
static inline uint16_t dsp_rd(const qsound_t *q, uint16_t a) { return q->dsp[a & 0xfff]; }

static inline int16_t read_sample(const qsound_t *q, uint16_t bank, uint16_t address)
{
	uint32_t a = (((uint32_t)(bank & 0x7fff) << 16) | address) & 0xffffff;
	if (q->rom_mask)
		a &= q->rom_mask;
	else if (a >= q->rom_size)
		return 0;
	return (int16_t)(q->rom[a] << 8);
}

void qsound_init(qsound_t *q, const uint8_t *dsp_rom, const uint8_t *samples, uint32_t size)
{
	memset(q, 0, sizeof(*q));
	q->dsp = (const uint16_t *)dsp_rom;   // little-endian words; the hosts are little-endian
	q->rom = samples;
	q->rom_size = size;
	q->rom_mask = (size && !(size & (size - 1))) ? size - 1 : 0;
	for (int i = 0; i < 16; i++) {
		q->reg[(i << 3) + 0] = &q->voice[(i + 1) % 16].bank;   // the bank applies to the next channel
		q->reg[(i << 3) + 1] = (uint16_t *)&q->voice[i].addr;
		q->reg[(i << 3) + 2] = &q->voice[i].rate;
		q->reg[(i << 3) + 3] = &q->voice[i].phase;
		q->reg[(i << 3) + 4] = (uint16_t *)&q->voice[i].loop_len;
		q->reg[(i << 3) + 5] = (uint16_t *)&q->voice[i].end_addr;
		q->reg[(i << 3) + 6] = (uint16_t *)&q->voice[i].volume;
		q->reg[i + 0x80] = &q->voice_pan[i];
		q->reg[i + 0xba] = (uint16_t *)&q->voice[i].echo;
	}
	for (int i = 0; i < 3; i++) {
		q->reg[(i << 2) + 0xca] = &q->adpcm[i].start_addr;
		q->reg[(i << 2) + 0xcb] = &q->adpcm[i].end_addr;
		q->reg[(i << 2) + 0xcc] = &q->adpcm[i].bank;
		q->reg[(i << 2) + 0xcd] = (uint16_t *)&q->adpcm[i].volume;
		q->reg[i + 0xd6] = &q->adpcm[i].flag;
		q->reg[i + 0x90] = &q->voice_pan[16 + i];
	}
	q->reg[0x93] = (uint16_t *)&q->echo.feedback;
	q->reg[0xd9] = &q->echo.end_pos;
	q->reg[0xe2] = &q->delay_update;
	q->reg[0xe3] = &q->next_state;
	for (int i = 0; i < 2; i++) {
		q->reg[(i << 1) + 0xda] = &q->filter[i].table_pos;
		q->reg[(i << 1) + 0xde] = (uint16_t *)&q->wet[i].delay;
		q->reg[(i << 1) + 0xe4] = (uint16_t *)&q->wet[i].volume;
		q->reg[(i << 1) + 0xdb] = &q->alt_filter[i].table_pos;
		q->reg[(i << 1) + 0xdf] = (uint16_t *)&q->dry[i].delay;
		q->reg[(i << 1) + 0xe5] = (uint16_t *)&q->dry[i].volume;
	}
	qsound_reset(q);
}

void qsound_reset(qsound_t *q)
{
	// the whole state as at power on (device_start's zero state, then device_reset)
	memset(q, 0, QSOUND_STATE_SIZE);
	q->ready_flag = 0;
	q->out[0] = q->out[1] = 0;
	q->state = STATE_BOOT;
	q->state_counter = 0;
}

void qsound_w(qsound_t *q, int offset, uint8_t data)
{
	switch (offset) {
	case 0: q->data_latch = (q->data_latch & 0x00ff) | (data << 8); break;
	case 1: q->data_latch = (q->data_latch & 0xff00) | data; break;
	case 2:
		if (q->reg[data])
			*q->reg[data] = q->data_latch;
		q->ready_flag = 0;
		break;
	}
}

uint8_t qsound_r(qsound_t *q) { return (uint8_t)q->ready_flag; }

static void state_init(qsound_t *q)
{
	const int mode = (q->state == STATE_INIT2) ? 1 : 0;
	if (q->state_counter >= 2) {
		q->state_counter = 0;
		q->state = q->next_state;
		return;
	} else if (q->state_counter == 1) {
		q->state_counter++;
		return;
	}
	memset(q->voice, 0, sizeof(q->voice));
	memset(q->adpcm, 0, sizeof(q->adpcm));
	memset(q->filter, 0, sizeof(q->filter));
	memset(q->alt_filter, 0, sizeof(q->alt_filter));
	memset(q->wet, 0, sizeof(q->wet));
	memset(q->dry, 0, sizeof(q->dry));
	memset(&q->echo, 0, sizeof(q->echo));
	for (int i = 0; i < 19; i++) {
		q->voice_pan[i] = DATA_PAN_TAB + 0x10;
		q->voice_output[i] = 0;
	}
	for (int i = 0; i < 16; i++)
		q->voice[i].bank = 0x8000;
	for (int i = 0; i < 3; i++)
		q->adpcm[i].bank = 0x8000;
	if (mode == 0) {
		q->wet[0].delay = 0; q->dry[0].delay = 46;
		q->wet[1].delay = 0; q->dry[1].delay = 48;
		q->filter[0].table_pos = DATA_FILTER_TAB + (FILTER_ENTRY_SIZE * 1);
		q->filter[1].table_pos = DATA_FILTER_TAB + (FILTER_ENTRY_SIZE * 2);
		q->echo.end_pos = DELAY_BASE_OFFSET + 6;
		q->next_state = STATE_REFRESH1;
	} else {
		q->wet[0].delay = 1; q->dry[0].delay = 0;
		q->wet[1].delay = 0; q->dry[1].delay = 0;
		q->filter[0].table_pos = 0xf73;
		q->filter[1].table_pos = 0xfa4;
		q->alt_filter[0].table_pos = 0xf73;
		q->alt_filter[1].table_pos = 0xfa4;
		q->echo.end_pos = DELAY_BASE_OFFSET2 + 6;
		q->next_state = STATE_REFRESH2;
	}
	q->wet[0].volume = q->dry[0].volume = q->wet[1].volume = q->dry[1].volume = 0x3fff;
	q->delay_update = 1;
	q->ready_flag = 0;
	q->state_counter = 1;
}

static void state_refresh_filter_1(qsound_t *q)
{
	for (int ch = 0; ch < 2; ch++) {
		q->filter[ch].delay_pos = 0;
		q->filter[ch].tap_count = 95;
		for (int i = 0; i < 95; i++)
			q->filter[ch].taps[i] = (int16_t)dsp_rd(q, q->filter[ch].table_pos + i);
	}
	q->state = q->next_state = STATE_NORMAL1;
}

static void state_refresh_filter_2(qsound_t *q)
{
	for (int ch = 0; ch < 2; ch++) {
		q->filter[ch].delay_pos = 0;
		q->filter[ch].tap_count = 45;
		for (int i = 0; i < 45; i++)
			q->filter[ch].taps[i] = (int16_t)dsp_rd(q, q->filter[ch].table_pos + i);
		q->alt_filter[ch].delay_pos = 0;
		q->alt_filter[ch].tap_count = 44;
		for (int i = 0; i < 44; i++)
			q->alt_filter[ch].taps[i] = (int16_t)dsp_rd(q, q->alt_filter[ch].table_pos + i);
	}
	q->state = q->next_state = STATE_NORMAL2;
}

static inline int16_t voice_update(const qsound_t *q, qs_voice *v, int32_t *echo_out)
{
	const int16_t output = (int16_t)((v->volume * read_sample(q, v->bank, v->addr)) >> 14);
	*echo_out += (output * v->echo) << 2;
	int32_t new_phase = v->rate + (((int32_t)v->addr << 12) | (v->phase >> 4));
	if ((new_phase >> 12) >= v->end_addr)
		new_phase -= ((int32_t)v->loop_len << 12);
	new_phase = clamp32(new_phase, -0x8000000, 0x7ffffff);
	v->addr = (int16_t)(new_phase >> 12);
	v->phase = (uint16_t)((new_phase << 4) & 0xffff);
	return output;
}

static int16_t adpcm_update(const qsound_t *q, qs_adpcm *a, int16_t curr_sample, int nibble)
{
	int8_t step;
	if (!nibble) {
		if (a->cur_addr == a->end_addr)
			a->cur_vol = 0;
		if (a->flag) {
			curr_sample = 0;
			a->flag = 0;
			a->step_size = 10;
			a->cur_vol = a->volume;
			a->cur_addr = a->start_addr;
		}
		step = (int8_t)(read_sample(q, a->bank, a->cur_addr) >> 8);
	} else
		step = (int8_t)(read_sample(q, a->bank, a->cur_addr++) >> 4);
	step >>= 4;
	int32_t delta = ((1 + abs(step << 1)) * a->step_size) >> 1;
	if (step <= 0)
		delta = -delta;
	delta += curr_sample;
	delta = clamp32(delta, -32768, 32767);
	a->step_size = (int16_t)((dsp_rd(q, DATA_ADPCM_TAB + 8 + step) * a->step_size) >> 6);
	a->step_size = (int16_t)clamp32(a->step_size, 1, 2000);
	return (int16_t)((delta * a->cur_vol) >> 16);
}

static inline int16_t echo_apply(qs_echo *e, int32_t input)
{
	int32_t old_sample = e->delay_line[e->delay_pos];
	const int32_t last_sample = e->last_sample;
	e->last_sample = (int16_t)old_sample;
	old_sample = (old_sample + last_sample) >> 1;
	const int32_t new_sample = input + ((old_sample * e->feedback) << 2);
	e->delay_line[e->delay_pos++] = (int16_t)(new_sample >> 16);
	if (e->delay_pos >= e->length)
		e->delay_pos = 0;
	return (int16_t)old_sample;
}

static inline int32_t fir_apply(qs_fir *f, int16_t input)
{
	int32_t output = 0, tap = 0;
	const int32_t n = f->tap_count - 1;
	int32_t pos = f->delay_pos;
	for (; tap < n; tap++) {
		output -= (f->taps[tap] * f->delay_line[pos++]) << 2;
		if (pos >= n)
			pos = 0;
	}
	output -= (f->taps[tap] * input) << 2;
	f->delay_line[pos++] = input;
	if (pos >= n)
		pos = 0;
	f->delay_pos = pos;
	return output;
}

static inline int32_t delay_apply(qs_delay *d, const int32_t input)
{
	d->delay_line[d->write_pos++] = (int16_t)(input >> 16);
	if (d->write_pos >= 51)
		d->write_pos = 0;
	const int32_t output = d->delay_line[d->read_pos++] * d->volume;
	if (d->read_pos >= 51)
		d->read_pos = 0;
	return output;
}

static inline void delay_update(qs_delay *d)
{
	const int16_t p = (int16_t)((d->write_pos - d->delay) % 51);
	d->read_pos = p < 0 ? p + 51 : p;
}

static void state_normal_update(qsound_t *q)
{
	q->ready_flag = 0x80;
	if (q->state == STATE_NORMAL2)
		q->echo.length = (int16_t)(q->echo.end_pos - DELAY_BASE_OFFSET2);
	else
		q->echo.length = (int16_t)(q->echo.end_pos - DELAY_BASE_OFFSET);
	q->echo.length = (int16_t)clamp32(q->echo.length, 0, 1024);

	int32_t echo_input = 0;
	for (int i = 0; i < 16; i++)
		q->voice_output[i] = voice_update(q, &q->voice[i], &echo_input);
	const int av = q->state_counter % 3;
	q->voice_output[16 + av] = adpcm_update(q, &q->adpcm[av], q->voice_output[16 + av], q->state_counter / 3);
	const int16_t echo_output = echo_apply(&q->echo, echo_input);

	for (int ch = 0; ch < 2; ch++) {
		int32_t wet = (ch == 1) ? echo_output << 14 : 0;
		int32_t dry = (ch == 0) ? echo_output << 14 : 0;
		for (int i = 0; i < 19; i++) {
			const uint16_t pan_index = q->voice_pan[i] + (ch * PAN_TABLE_CH_OFFSET);
			dry -= (q->voice_output[i] * (int16_t)dsp_rd(q, pan_index + PAN_TABLE_DRY));
			wet -= (q->voice_output[i] * (int16_t)dsp_rd(q, pan_index + PAN_TABLE_WET));
		}
		dry = (int32_t)((uint32_t)clamp32(dry, -0x1fffffff, 0x1fffffff) << 2);
		wet = (int32_t)((uint32_t)clamp32(wet, -0x1fffffff, 0x1fffffff) << 2);
		wet = fir_apply(&q->filter[ch], (int16_t)(wet >> 16));
		if (q->state == STATE_NORMAL2)
			dry = fir_apply(&q->alt_filter[ch], (int16_t)(dry >> 16));
		int32_t output = delay_apply(&q->wet[ch], wet) + delay_apply(&q->dry[ch], dry);
		output = (output + 0x2000) & ~0x3fff;
		q->out[ch] = (int16_t)clamp32(output >> 14, -0x7fff, 0x7fff);
		if (q->delay_update) {
			delay_update(&q->wet[ch]);
			delay_update(&q->dry[ch]);
		}
	}
	q->delay_update = 0;
	if (++q->state_counter > 5) {
		q->state_counter = 0;
		q->state = q->next_state;
	}
}

static inline void update_sample(qsound_t *q)
{
	switch (q->state) {
	default:
	case STATE_INIT1:
	case STATE_INIT2: state_init(q); break;
	case STATE_REFRESH1: state_refresh_filter_1(q); break;
	case STATE_REFRESH2: state_refresh_filter_2(q); break;
	case STATE_NORMAL1:
	case STATE_NORMAL2: state_normal_update(q); break;
	}
}

void qsound_render(qsound_t *q, int32_t *l, int32_t *r, int n)
{
	for (int i = 0; i < n; i++) {
		update_sample(q);
		l[i] += q->out[0];
		r[i] += q->out[1];
	}
}
