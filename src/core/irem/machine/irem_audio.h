/*  The Irem M52 / M62 sound board, ported from MAME src/mame/irem/irem.cpp (BSD-3-Clause, copyright-holders: Couriersud).
    A 6803 at 3.58 MHz, two AY-3-8910s and one or two MSM5205s whose VCK is the 6803's NMI; a frame behind on the worker
    (rbx_aux). Not done: the M62's netlist (its analog drums on the second AY's port A are silent; the AYs and the
    MSM5205s are mixed plainly) and the M52-SOUND-C filters.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_IREM_AUDIO_H
#define RBX_IREM_AUDIO_H

#include <stdint.h>

enum { IREM_AUDIO_M62, IREM_AUDIO_M52_SOUNDC, IREM_AUDIO_M52_LARGE };

// the region "irem_audio:iremsound" loaded; the board's line is htotal / pix_clock seconds; sets up rbx_audio and
// rbx_aux's job. 0 = ok
int irem_audio_init(int type, uint32_t htotal, uint32_t pix_clock);
void irem_audio_exit(void);
void irem_audio_reset(void);
void irem_audio_cmd_w(uint8_t v);    // the main CPU's write to the command latch
void irem_audio_line(void);          // a line has gone by
void irem_audio_frame(void);         // the end of a frame: the last frame's samples out, this frame's work to the worker
int irem_audio_state(void *ctx, int load);

#endif
