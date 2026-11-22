// The frame buffer (ADR-195): the emulation may run one frame ahead of the display. A present thread (CPU0, above the
// emulation thread) draws and shows each frame the emulation posts, once and in order; the emulation thread only waits
// when one finished frame is already queued behind the one being shown. Input is read when the frame is emulated, so
// the picture comes one frame (about 17 ms) later than without it
#ifndef RBX_FRONTEND_PRESENT_H
#define RBX_FRONTEND_PRESENT_H

#include <stdint.h>

int rbx_present_start(void);    // 0 = the thread runs (the emulation thread then calls only rbx_present_post to show)
void rbx_present_stop(void);    // (drains first; nothing if not started)
int rbx_present_running(void);
// the emulation thread: show the render thread's frame n (rbx_video_draw_rendered's numbering) with the overlay layers.
// Waits while a posted frame has not been taken yet (the queue holds one)
void rbx_present_post(uint64_t frame_n);
// the emulation thread: wait until every posted frame has been shown; the present thread then touches no vita2d state
// until the next post (the menu, a state load, a reset or the game's end draw on the emulation thread)
void rbx_present_drain(void);

// the frame buffer setting in effect for the running game: Display > Frame buffer (Auto: on for the Cave CV1000), or
// the data folder's runahead0 / runahead1 (testing)
int rbx_present_wanted(void);
void rbx_present_set_board(const char *board);   // (the running game's board, at its start)
void rbx_present_check_files(void);              // (the data folder's switches, once at the app's start)
const char *rbx_present_auto_text(void);         // what Auto means for the running game ("1 frame" / "Off")

#endif
