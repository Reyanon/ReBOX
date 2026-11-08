#ifndef RBX_FRONTEND_INPUT_H
#define RBX_FRONTEND_INPUT_H

#include <stdint.h>

// Controllers (re_pad: 1P and 2P, common/re_pad.h; the touch buttons are 1P's) -> core input: button mapping, turbo, combo keys, analog stick, Rotate (settings.h).
// SELECT+START opens the options, SELECT+L / SELECT+R quick save / load (not passed to the game)

void rbx_input_init(void);

// Read the controllers, pass them to the core, return every controller's buttons together (no touch buttons; for the
// menus and combos)
uint32_t rbx_input_poll(void);
// every game input released (before a game ends: the next game starts with nothing held)
void rbx_input_release(void);

#endif
