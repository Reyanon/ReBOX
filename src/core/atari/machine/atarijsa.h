/*  The Atari JSA I / JSA II sound boards (6502 at 1.79 MHz, a YM2151, a POKEY and a TMS5220C on the JSA I, an OKI6295
    on the JSA II) and their sound command latches. Ported from MAME src/mame/atari/atarijsa.cpp, atarijsa.h and
    atariscom.cpp, atariscom.h (BSD-3-Clause, copyright-holders: Aaron Giles).
    One board at a time (the driver's): its state is here. The board runs the 6502 with jsa_run (its cycles), the YM2151
    with jsa_line (once a scanline: the YM time of the 6502 cycles run since), and mixes a frame with jsa_frame_end.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_ATARIJSA_H
#define RBX_ATARIJSA_H

#include <stdint.h>

#define JSA_CPU_CLOCK 1789772      // JSA_MASTER_CLOCK / 2

enum { JSA_OUT_STEREO, JSA_OUT_SWAPPED, JSA_OUT_MONO };

typedef struct {
	int type;              // 1 (JSA I) or 2 (JSA II)
	int pokey, tms;        // JSA I: the chips populated (config.device_remove)
	int out;               // JSA_OUT_*: the board's routes of the two outputs
	int gain;              // the board's route gain of the outputs (1/256)
	const uint8_t *rom;    // the "jsa:cpu" region (0x10000)
	const uint8_t *oki;    // JSA II: the "jsa:oki1" region
	uint32_t oki_size;
	int port;              // the coin port ("JSA")
	void (*main_int)(int state);   // the sound response interrupt of the main CPU
} jsa_config;

int jsa_init(const jsa_config *c);   // 0 = ok (sets up the audio stream)
void jsa_exit(void);
void jsa_reset(void);                // device_reset of the board and its comm device; the 6502 reset
void jsa_set_test(int state);        // test_read_cb: the main board's self test input (1 = off)

// the main CPU's side (atari_sound_comm_device)
void jsa_main_command_w(uint8_t data);
uint8_t jsa_main_response_r(void);
void jsa_sound_reset_w(void);
int jsa_main_to_sound_ready(void);
int jsa_sound_to_main_ready(void);
int jsa_boosting(void);              // the comm's add_quantum (5 us for 1 ms after a command / reset) running

void jsa_run(int cycles);            // the 6502 for that many of its cycles
void jsa_line(void);                 // the YM2151 up to the 6502's time
void jsa_frame_begin(void);
void jsa_frame_end(void);            // the frame's samples to the audio stream

int jsa_state(void *ctx, int load);

#endif
