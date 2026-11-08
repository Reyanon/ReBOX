// Front touch screen: on-screen key areas (any number, made in the menu's touch layout editor). Ported from ReSys12
#ifndef RBX_FRONTEND_TOUCH_H
#define RBX_FRONTEND_TOUCH_H

#include <stdint.h>

// keys an area can press: RBX_KEY_* (Cross .. Start), then the d-pad
enum { RBX_TKEY_UP = 8, RBX_TKEY_DOWN, RBX_TKEY_LEFT, RBX_TKEY_RIGHT, RBX_TKEY_COUNT };
// a touch button can also press two keys together (its own, apart from Controls > Combo keys):
// key = RBX_TKEY_COMBO + bit mask (1 << RBX_KEY_*) of the two (as in ReST-V)
#define RBX_TKEY_COMBO 256
#define RBX_TKEY_IS_COMBO(k) ((k) >= RBX_TKEY_COMBO)

typedef struct { int16_t x, y, w, h, key; } ReiTouchArea; // screen pixels (960x544)

void rbx_touch_init(void);                 // start sampling, read the layout (ux0:data/rebox/touch.cfg)
int rbx_touch_count(void);
ReiTouchArea *rbx_touch_area(int i);
int rbx_touch_add(int x, int y, int w, int h, int key); // index of the new area (-1: no memory)
void rbx_touch_remove(int i);
void rbx_touch_clear(void);
void rbx_touch_save(void);                 // to the layout file in use
// the layout file in use: the game's (per-game settings on; game_path) or the global one (NULL). keep: the layout now
// is saved to it (per-game settings made); else it is read (a game without its own yet: the global layout)
void rbx_touch_select(const char *game_path, int keep);
int rbx_touch_on_area(int x, int y);       // a touch point on a button
int rbx_touch_gen(void);                   // changes whenever the layout changes
void rbx_touch_changed(void);              // an area was edited in place
uint32_t rbx_touch_key_code(int key);      // RBX_TKEY / RBX_KEY -> SCE_CTRL_* bits
const char *rbx_touch_key_name(int key);

// the touches now (screen pixels, up to 6); returns the count
int rbx_touch_read(int *x, int *y);
// SCE_CTRL_* bits of the areas touched now
uint32_t rbx_touch_buttons(void);

#endif
