/*  Space Invaders family (Intel 8080, Taito sets of MAME src/mame/midw8080/8080bw.cpp): internals shared by
    the machine part (si8080.c) and the sound part (si8080_a.c).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_SI8080_H
#define RBX_SI8080_H

#include <stdint.h>

// sound boards (MAME machine configuration -> sound hardware)
enum {
	SIA_INVADERS,    // invaders_audio (mw8080bw_a.cpp) driven like the samples boards: SN76477 saucer + samples
	SIA_INVADPT2,    // invaders_samples_audio: SN76477 + "invaders" samples
	SIA_SPCEWARS,    // invaders samples + 1-bit speaker
	SIA_LRESCUE,     // "lrescue" samples + 1-bit speaker
	SIA_SCHASER,     // schaser discrete (SN76477 effect / explosion, dot tone, 1-bit music)
	SIA_SCHASERCV,   // invaders samples + 1-bit speaker
	SIA_CRASHRD,     // schaser discrete, Crash Road port layout
	SIA_LUPIN3,      // SN76477 + "lupin3" samples + note music
	SIA_POLARIS,     // polaris discrete
	SIA_BALLBOMB,    // invaders samples + note music
	SIA_INDIANBT,    // invaders samples + note music
	SIA_INDIANBTBR,  // invaders samples
	SIA_NONE
};

#define SIA_RATE 31440   // two samples per scanline (262 lines, 60 frames)

// state the sound handlers change in the machine (video latches)
typedef struct {
	uint8_t flip_screen, color_map, screen_red;
	uint8_t schaser_bg_disable, schaser_bg_select;
	uint8_t cab;          // cocktail cabinet input (CAB bit 0 / DIP), read by the flip screen logic
	uint8_t cab_invaders; // invaders_state: flip line only connected on the cocktail PCB
} sia_latch;

int sia_init(int board, const char *set);   // 0 = ok
void sia_exit(void);
void sia_reset(void);
// port writes of the sound boards, MAME handler names: which = 1 (sh_port_1), 2 (sh_port_2), 3 (sh_port_3 / extra)
void sia_write(int which, uint8_t data, sia_latch *l);
int16_t sia_sample(void);   // one output sample at SIA_RATE
int sia_state(void *ctx, int load);

#endif
