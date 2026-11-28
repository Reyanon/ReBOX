// Test input (log builds only): ux0:data/rebox/keys holds a script of button presses and touches that the app plays
// as if they came from the pad and the touch screen (the device tests drive the menus with it; the vitacompanion
// "press" command does not reach the app's pad reads). Tokens, separated by spaces or new lines:
//   circle | down*5            a tap (4 frames down, 8 up), optionally repeated
//   select+start               keys pressed together
//   hold:right:90              held for that many frames
//   wait:60                    nothing for that many frames
//   touch:480:272:10           a touch at x, y for that many frames
//   drag:100:100:300:200:30    a touch moving from x0, y0 to x1, y1 over that many frames
// The file is removed when it is read; a new one can be sent at any time.
#ifndef RBX_FRONTEND_TESTPAD_H
#define RBX_FRONTEND_TESTPAD_H

#include <stdint.h>

#ifdef RBX_ENABLE_LOG
// once per frame from the loop that reads the pad: the buttons the script presses now
uint32_t rbx_testpad_step(void);
// the buttons the script pressed at its last step (for code that reads the devices itself, as the key capture)
uint32_t rbx_testpad_last(void);
// the touch the script makes now (1, x, y in screen pixels) or 0
int rbx_testpad_touch(int *x, int *y);
// (ADR-161) the test files the frame loop looked for on the memory card (keys, shotgame, shotfb), looked for instead by a
// low-priority thread every 250 ms: a card stat on the emulation thread could stall it for seconds while the card is
// being written. rbx_testfile_seen: the watcher saw that file since the last call (then look for it once), or the
// watcher is not running (then look as before, at the caller's pace)
void rbx_testfiles_start(void);
void rbx_testfiles_stop(void);
int rbx_testfile_seen(const char *path);
#else
static inline uint32_t rbx_testpad_step(void) { return 0; }
static inline uint32_t rbx_testpad_last(void) { return 0; }
static inline int rbx_testpad_touch(int *x, int *y) { (void)x; (void)y; return 0; }
static inline void rbx_testfiles_start(void) {}
static inline void rbx_testfiles_stop(void) {}
#endif

#endif
