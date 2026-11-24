#ifndef RBX_FRONTEND_MENU_H
#define RBX_FRONTEND_MENU_H

#include <stdint.h>

#include "rbx_games.h"

// In-game menu, game list, loading screen, notices, FPS box, touch buttons and the PS button. Ported from ReSys12
// vita/src/frontend/menu.c (common UI of the Re series of Vita emulators: same layout, controls, animations); drawn
// with ui.c on overlay layers (video.h). The in-game menu opens with SELECT+START or PS; the game pauses while it is open.

void rbx_menu_init(void);
void rbx_menu_fini(void);
// the running game (after rbx_core_init): set name, title, game buttons (2..6)
void rbx_menu_set_game(const char *set, const char *title, int buttons);

// Game list (all supported games, ROM-less ones grey; board tabs, SELECT filter, Square favorite (star), START or PS
// options (Display, Controls, Refresh ROM list; global settings), Triangle About; a newer release is offered once:
// Update / Close / Skip). Scans dirs for the ROM zips; the cursor starts on g_settings.last_game. notice: shown in a box
// first (NULL = none). Returns the picked game (a playable one; the pointer stays valid until the next call)
int rbx_rom_dirs_refresh(void);   // (main.c: the ROM folders again after Settings changed one; returns how many)
const rbx_game_found *rbx_game_select(const char *const *dirs, int ndirs, const char *rom_dir, const char *notice);

// Loading screen: the app's background, game name, stage, moving bar. The loader thread sets the progress
// (rbx_loading_set, 0-1000, any thread); the main thread draws one frame per vblank (rbx_loading_frame)
void rbx_loading_title(const char *title);
void rbx_loading_set(int permille);
void rbx_loading_frame(void);
void rbx_loading_progress(int permille); // set and draw at once
void rbx_loading_done(void);

// 1 at the moment SELECT+START or PS is pressed (called every frame in the game)
int rbx_menu_should_open(uint32_t buttons);

typedef enum {
	RBX_MENU_STAY,        // stay in menu
	RBX_MENU_CLOSE,       // return to game
	RBX_MENU_EXIT,        // exit app
	RBX_MENU_SELECT_GAME, // to the game list (same process)
	RBX_MENU_RESET,       // reset game, then return
	RBX_MENU_SAVE,        // save to rbx_menu_state_slot() slot (stay in menu)
	RBX_MENU_LOAD,        // load from rbx_menu_state_slot() slot, then return
	RBX_MENU_REFRESH,     // game list options: read the ROM folders again
} rbx_menu_result;

// Called every frame while the menu is open: input handling (buttons = SceCtrlData.buttons)
rbx_menu_result rbx_menu_update(uint32_t buttons);
// draws the menu canvas when something changed (the overlay layers are drawn by rbx_video_draw_layers)
void rbx_menu_draw(void);
void rbx_menu_hide(void);
// the theme's accent colour (0xAABBGGRR): "iREM" in the header, the app's blank artwork is tinted with it
uint32_t rbx_menu_accent(void);
// Slot chosen on the Save/Load page (0~9)
int rbx_menu_state_slot(void);
void rbx_menu_states_changed(void); // a state was saved: read the slots again

// Status message at bottom center (~2s). Shown on top even while the menu is open
void rbx_osd_show(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void rbx_osd_tick(void); // every frame (hides it after 2 s)

// FPS box in the game picture's top-right corner (when the setting is on): once per presented game frame
// once per display frame: ran = a new game frame in it, target_hz = the game's own rate (ADR-211)
void rbx_fps_tick(int ran, float target_hz);
void rbx_menu_shot_poll(void); // test: screen capture while a game runs (log builds)
void rbx_fps_reset_frames(void); // frame number shown in test builds back to 0 (new game)
void rbx_fps_hide(void); // while the menu is open

// touch buttons over the game (redrawn only when the layout or the settings change); hidden = 1 while the menu is open
void rbx_touch_overlay_update(int hidden);

// kernel helper for the PS button and the update install (module/rbx_kctrl.skprx; may restart the app once after boot:
// restarted = the app was started again for it, argument "kctrl")
void rbx_kctrl_init(int restarted);
int rbx_kctrl_ready(void);
void rbx_ps_release(void); // before exit: the PS button back to the system (and the CPU3 unlock)
void rbx_kctrl_cpu3_undo(void);   // (the unlock given back when CPU3 stayed closed)
void rbx_kctrl_cpu3(int hook);  // after rbx_kctrl_init: CPU3 unlocked on a verified firmware (hook 0: already open)

void rbx_menu_bench(void);   // (log builds: ux0:data/rebox/menubench)

void rbx_loading_prepare(void);   // the loading screen's background made ahead (app start)

#endif
