/*  Capcom QSound DL-1425, high level. Ported from MAME src/devices/sound/qsoundhle.cpp (BSD-3-Clause,
    copyright-holders: superctr, Valley Bell; based on the disassembled DSP program): 16 PCM voices, 3 ADPCM voices,
    the echo, the FIR filters and delays, the tables from the DSP's ROM. One stereo sample per clock / 2 / 1248.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_QSOUND_H
#define RBX_QSOUND_H

#include <stdint.h>

typedef struct { uint16_t bank; int16_t addr; uint16_t phase, rate; int16_t loop_len, end_addr, volume, echo; } qs_voice;
typedef struct { uint16_t start_addr, end_addr, bank; int16_t volume; uint16_t flag; int16_t cur_vol, step_size; uint16_t cur_addr; } qs_adpcm;
typedef struct { int32_t tap_count, delay_pos; uint16_t table_pos; int16_t taps[95], delay_line[95]; } qs_fir;
typedef struct { int16_t delay, volume, write_pos, read_pos; int16_t delay_line[51]; } qs_delay;
typedef struct { uint16_t end_pos; int16_t feedback, length, last_sample; int16_t delay_line[1024]; int16_t delay_pos; } qs_echo;

typedef struct {
	uint16_t data_latch;
	int16_t out[2];
	qs_voice voice[16];
	qs_adpcm adpcm[3];
	uint16_t voice_pan[19];
	int16_t voice_output[19];
	qs_echo echo;
	qs_fir filter[2], alt_filter[2];
	qs_delay wet[2], dry[2];
	uint16_t state, next_state, delay_update;
	int32_t state_counter, ready_flag;
	uint8_t state_end;
	// configuration
	const uint16_t *dsp;     // the DSP ROM: 0x1000 little-endian words
	const uint8_t *rom;      // the samples
	uint32_t rom_size, rom_mask;
	uint16_t *reg[256];
} qsound_t;

#define QSOUND_STATE_SIZE ((uint32_t)__builtin_offsetof(qsound_t, state_end))

void qsound_init(qsound_t *q, const uint8_t *dsp_rom, const uint8_t *samples, uint32_t size);
void qsound_reset(qsound_t *q);
void qsound_w(qsound_t *q, int offset, uint8_t data);   // offsets 0, 1: the data latch; 2: the register
uint8_t qsound_r(qsound_t *q);                          // the ready flag
void qsound_render(qsound_t *q, int32_t *l, int32_t *r, int n);   // n samples added

#endif
