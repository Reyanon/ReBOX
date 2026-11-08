/*  Taito Ensoniq sound board (68000 + ES5505 + MC68681 DUART + ES5510 ESP + MB87078 + MB8421 dual-port RAM),
    ported from MAME src/mame/taito/taito_en.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail, Aaron Giles,
    R. Belmont, Philip Bennett), with the MC68681 counter/timer after src/devices/machine/mc68681.cpp
    (BSD-3-Clause, Mariusz Wojcieszek, R. Belmont, Joseph Zatarski) and the MB87078 after src/devices/sound/mb87077.cpp.
    The ES5510 effects DSP (sound/es5510.c) runs once per output sample as MAME's ES5505/5510 pump does
    (sound/esqpump.cpp; DUART OP6 = ESPHALT); its output and the dry ES5505 channel 0 are the stereo output.
    The sound 68000 shares the Musashi core with the main CPU: ten_line() switches contexts and the clashing pages
    of the m68k_bus page table.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_TAITO_EN_H
#define RBX_TAITO_EN_H

#include <stdint.h>

typedef struct {
	uint32_t frame_rate_mhz;   // host frame rate (milli-Hz)
	int lines;                 // scanlines per frame (ten_line is called once per line)
} ten_config;

int ten_init(const ten_config *cfg);   // regions "taito_en:audiocpu", "taito_en:ensoniq"; call after the main CPU map
void ten_exit(void);
void ten_reset(int hard);              // machine reset (sound CPU held in reset, as MAME's taito_f3 machine_reset);
                                       // hard = power on (clears RAM), else soft reset (watchdog)
void ten_set_reset_line(int asserted); // main CPU sound reset ports
void ten_set_bank(int bank, int entry);
// main CPU side of the dual-port RAM (0x800 bytes used, 0x1000 allocated to fill a 4KB page; host word order): read
// there, written by ten_dpram_w (68000 byte offset, word data, byte mask)
uint8_t *ten_dpram(void);
void ten_dpram_w(uint32_t a, uint16_t v, uint16_t mask);
void ten_frame_begin(void);
void ten_line(void);                   // after each main CPU scanline (the sound board runs TEN_LAG lines behind)
void ten_frame_end(void);              // hand the frame's samples to rbx_audio (after the ES5510 job)
int ten_state(void *ctx, int load);

#endif
