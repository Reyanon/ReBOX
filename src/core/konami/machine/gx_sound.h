/*  The Konami GX sound board, after MAME src/mame/konami/konamigx.cpp (BSD-3-Clause, copyright-holders: R. Belmont,
    Acho A. Tang, Phil Stroffolino, Olivier Galibert): a 68000 at 8 MHz, two K054539 (48 kHz), the K056800 host
    interface, the TMS57002 DASP (cpu/tms57002: one program pass a sample, the two chips its inputs).
    The 68000 is the second Musashi instance (cpu/m68k_snd).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_GX_SOUND_H
#define RBX_GX_SOUND_H

#include <stdint.h>

#define GXS_RATE 48000

// rom: the 68000's program (big-endian bytes, swapped in place), pcm: the K054539s' ROM. 0 = ok
int gxs_init(uint8_t *rom, uint32_t rom_size, const uint8_t *pcm, uint32_t pcm_size);
void gxs_exit(void);
void gxs_reset(void);   // the board as at power on: the 68000 held (MAME machine reset)
// (the board runs a frame behind on the rbx_aux worker: the calls below are queued, gxs_frame runs them)
void gxs_start(void);   // after gxs_init (the worker's job)
void gxs_stop(void);
void gxs_sync_reset(void);   // the host's reset: the job finished, the queue dropped, gxs_reset
// the host's control bit 22 (SOUNDRESET): 1 the 68000 runs, 0 held in reset (and the DASP)
void gxs_run_line(int run);
// the K056800's host side (offset 0..7): the reads see the board's replies as of the last finished frame
uint8_t gxs_host_r(int offset);
void gxs_host_w(int offset, uint8_t data);
// MAME's sound gain hacks of a set (tkmmpzdm, dragoonj / dragoona)
void gxs_set_gains(const char *set);
void gxs_advance(int n);   // n samples of the host's time (48 kHz), after the writes before them
// the frame's end: the last frame's samples to l[] and r[] (at most max), this frame's queue to the worker
int gxs_frame(int32_t *l, int32_t *r, int max);
int gxs_state(void *ctx, int load);

#endif
