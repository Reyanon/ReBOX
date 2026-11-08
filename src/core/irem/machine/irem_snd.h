/*  The sound board of Irem M92 / M107: a V35, a YM2151 and a GA20, with two latches to the main board. Ported from the
    sound maps of MAME src/mame/irem/m92.cpp and m107.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail).
    A frame behind on the workers: rbx_aux2 runs the V35 and the GA20 from a queue of the main board's frame, the YM2151's
    samples are made on rbx_aux (rbx_ym_set_async). What the V35 answers reaches the main board at the same line a frame
    later.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef IREM_SND_H
#define IREM_SND_H

#include <stdint.h>

// answer: the data pending line of the second latch (the main board's interrupt)
int isnd_init(const uint8_t *dec, int stereo, uint32_t frame_mhz, int lines, void (*answer)(int state));
void isnd_exit(void);
void isnd_reset(void);
void isnd_latch_w(uint8_t v);         // soundlatch
uint8_t isnd_latch2_r(void);          // soundlatch2: the pending line falls
void isnd_reset_line(int asserted);   // sound_reset_w
void isnd_line(int line);             // every line of the frame, in order
void isnd_frame(void);                // the frame's end: the sound of the last one out, this one's queue to the worker
int isnd_state(void *ctx, int load);

#endif
