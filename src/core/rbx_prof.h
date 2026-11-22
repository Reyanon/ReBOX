/*  ReBOX sampled activity profile (ADR-155): the emulation thread stores what it is doing now in rbx_prof_cur, a plain
    volatile byte; a sampler thread (the Vita frontend's, the harness's RBX_PROF) reads it every 250 us and counts the
    values. No clock calls on the emulation thread: a mark is one byte store (rbx_prof_set), or a load and a store to
    put back what was there (rbx_prof_enter / rbx_prof_leave) around a nested activity.
    Only the emulation thread writes it: code that may run on a worker thread (a blitter lane, the sound worker, the
    render thread) has no marks.
    Ids: below RBX_PA_CPU the core's own (named here), RBX_PA_CPU.. a CPU core's (sh3.h), RBX_PA_DRV.. the driver's;
    their names are given with rbx_prof_name (init) and read with rbx_prof_get_name.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_PROF_H
#define RBX_PROF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
	RBX_PA_OTHER,       // nothing marked
	RBX_PA_FRONT,       // the frontend's own work between frames (menus, input, logs)
	RBX_PA_DRAW,        // the frontend's drawing lists (vita2d)
	RBX_PA_SWAP,        // the frontend's swap (the vblank wait)
	RBX_PA_INPUT,       // core: the input ports at the frame start
	RBX_PA_DRIVER,      // core: the driver's frame, where the driver marks nothing
	RBX_PA_VIDEO_OUT,   // core: the driver's render or the conversion on the emulation thread
	RBX_PA_AUDIO_OUT,   // core: the frame's sound resampled
	RBX_PA_WAIT,        // core: asleep waiting for another thread (wait_site)
	RBX_PA_ROMREAD,     // core: rbx_region_read (a region kept in a file: memory card reads)
	RBX_PA_IOWAIT,      // core: waiting for the io worker's read ahead (rbx_io_finish / rbx_io_drop, ADR-160)
	RBX_PA_FE_PAD,      // the frontend: the pads, touch and power tick at the frame start (ADR-161)
	RBX_PA_FE_AUDIO,    // the frontend: the frame's sound resampled into the output ring (rbx_audio_push)
	RBX_PA_FE_TICKS,    // the frontend: the per-frame ticks after the swap (fps box, threads, medal, osd)
	RBX_PA_CPU = 16,    // 16..31: the CPU core's (sh3.h SH3_PA_*)
	RBX_PA_DRV = 32,    // 32..47: the driver's
	RBX_PROF_IDS = 48
};

extern volatile uint8_t rbx_prof_cur;

#ifdef RBX_PROF_CNT   // (testing, -DRBX_PROF_CNT: the marks counted, the harness prints them a frame at the exit)
extern unsigned long long rbx_prof_cnt;
#define RBX_PROF_COUNT_ONE() (rbx_prof_cnt++)
#else
#define RBX_PROF_COUNT_ONE() ((void)0)
#endif
#ifndef RBX_PROF_OFF
static inline void rbx_prof_set(unsigned id) { RBX_PROF_COUNT_ONE(); rbx_prof_cur = (uint8_t)id; }
// a nested activity: returns what to put back with rbx_prof_leave
static inline unsigned rbx_prof_enter(unsigned id)
{
	RBX_PROF_COUNT_ONE();
	const unsigned prev = rbx_prof_cur;
	rbx_prof_cur = (uint8_t)id;
	return prev;
}
static inline void rbx_prof_leave(unsigned prev) { RBX_PROF_COUNT_ONE(); rbx_prof_cur = (uint8_t)prev; }
#else   // (-DRBX_PROF_OFF: no marks, for measuring their cost)
static inline void rbx_prof_set(unsigned id) { (void)id; }
static inline unsigned rbx_prof_enter(unsigned id) { (void)id; return 0; }
static inline void rbx_prof_leave(unsigned prev) { (void)prev; }
#endif

// the names (static strings): ids first..first+n-1 from names[] (a NULL entry leaves that id unnamed)
void rbx_prof_name(unsigned first, const char *const *names, unsigned n);
const char *rbx_prof_get_name(unsigned id);   // ("" for an unnamed id)

#ifdef __cplusplus
}
#endif

#endif
