#ifndef RBX_FRONTEND_SETTINGS_H
#define RBX_FRONTEND_SETTINGS_H

#include <stdint.h>

// User settings (changed in the options screen, saved to ux0:data/rebox/settings.cfg; per-game DIP in <set>.cfg)

enum {
	RBX_ASPECT_4_3,      // 4:3 fitted to 544 height (default)
	RBX_ASPECT_FULL,     // stretched to 960x544
	RBX_ASPECT_INT2X,    // integer 2x (640x480)
	RBX_ASPECT_COUNT
};

enum {
	RBX_FILTER_LINEAR,   // smooth (default)
	RBX_FILTER_POINT,    // sharp
	RBX_FILTER_COUNT
};

// Scanline types. CRT types follow game lines (240), PX types draw evenly at screen-pixel spacing
enum {
	RBX_SCAN_OFF,
	RBX_SCAN_CRT_SOFT,   // darken game line borders with a soft curve (linear filter)
	RBX_SCAN_CRT_SHARP,  // darken the lower half of each game line
	RBX_SCAN_PX2,        // 1 line every 2 screen pixels
	RBX_SCAN_PX3,        // 1 line every 3 pixels
	RBX_SCAN_PX4,        // 1 line every 4 pixels
	RBX_SCAN_COUNT
};

enum { RBX_SCAN_LIGHT, RBX_SCAN_MEDIUM, RBX_SCAN_HEAVY, RBX_SCAN_STRENGTH_COUNT };

// Margin outside the game screen (4:3 / Integer 2x). Artwork: the user's artwork/<set>.png, else the app's blank artwork
enum { RBX_BORDER_BLACK, RBX_BORDER_ARTWORK, RBX_BORDER_COLOR, RBX_BORDER_MARQUEE, RBX_BORDER_COUNT };   // (ARTWORK: "Graphic")
// Screen rotation: Auto = vertical games upright on the landscape screen; Off = the game's buffer as it is; Vertical L / R =
// upright with the Vita turned left (right edge up) / right (left edge up), directions turned with it
enum { RBX_ROT_AUTO, RBX_ROT_OFF, RBX_ROT_VL, RBX_ROT_VR, RBX_ROT_COUNT };
// Display > Frame buffer (ADR-195): the emulation may run one frame ahead of the display (input about 17 ms later).
// Auto = on for the Cave CV1000 games, off for the others (present.c rbx_present_wanted)
enum { RBX_FRAMEBUF_AUTO, RBX_FRAMEBUF_OFF, RBX_FRAMEBUF_ONE, RBX_FRAMEBUF_COUNT };
// Menu colour theme (global; Graphite is the ReBOX default, Red ReSys12's, Blue ReST-V's)
enum { RBX_THEME_RED, RBX_THEME_BLUE, RBX_THEME_GREEN, RBX_THEME_PURPLE, RBX_THEME_ORANGE, RBX_THEME_GRAPHITE, RBX_THEME_PINK, RBX_THEME_COUNT };
// A stick direction, and what it presses: nothing, a lever direction (order of RBX_SDIR_*), game buttons 1-6, coin, start
enum { RBX_SDIR_UP, RBX_SDIR_DOWN, RBX_SDIR_LEFT, RBX_SDIR_RIGHT, RBX_SDIR_COUNT };
enum { RBX_STGT_NONE, RBX_STGT_UP, RBX_STGT_DOWN, RBX_STGT_LEFT, RBX_STGT_RIGHT, RBX_STGT_B1, RBX_STGT_B6 = RBX_STGT_B1 + 5,
	RBX_STGT_COIN, RBX_STGT_START, RBX_STGT_COUNT };
enum { RBX_BCOL_NAVY, RBX_BCOL_GRAY, RBX_BCOL_MAROON, RBX_BCOL_GREEN, RBX_BCOL_PURPLE, RBX_BCOL_BRICK,
       RBX_BCOL_BLACK, RBX_BCOL_CHARCOAL, RBX_BCOL_SILVER, RBX_BCOL_WHITE, RBX_BCOL_BLUE, RBX_BCOL_SKY, RBX_BCOL_TEAL,
       RBX_BCOL_OLIVE, RBX_BCOL_FOREST, RBX_BCOL_BROWN, RBX_BCOL_ORANGE, RBX_BCOL_GOLD, RBX_BCOL_RED, RBX_BCOL_WINE,
       RBX_BCOL_PINK, RBX_BCOL_INDIGO, RBX_BCOL_COUNT };   // (new ones at the end: the saved numbers stay)

// Options screen confirm button (the other one is back)
enum { RBX_CONFIRM_CROSS, RBX_CONFIRM_CIRCLE, RBX_CONFIRM_COUNT };

// Mappable game buttons
enum {
	RBX_ACT_ATTACK,      // game button 1 (nbbatman attack)
	RBX_ACT_JUMP,        // game button 2 (nbbatman jump)
	RBX_ACT_COIN,
	RBX_ACT_START,
	RBX_ACT_B3, RBX_ACT_B4, RBX_ACT_B5, RBX_ACT_B6, // game buttons 3-6 (shown in the menu only if the game uses them, RBX_KEY_NONE allowed)
	RBX_ACT_COUNT
};

// Selectable Vita buttons (cycled left/right in the options screen)
enum {
	RBX_KEY_CROSS, RBX_KEY_CIRCLE, RBX_KEY_SQUARE, RBX_KEY_TRIANGLE,
	RBX_KEY_L, RBX_KEY_R, RBX_KEY_SELECT, RBX_KEY_START,
	RBX_KEY_L2, RBX_KEY_R2, RBX_KEY_L3, RBX_KEY_R3, // (a controller's; settings before them: keyver < 2, None was 8)
	// the sticks' directions (settings before them: keyver < 4, None was 12)
	RBX_KEY_LUP, RBX_KEY_LDOWN, RBX_KEY_LLEFT, RBX_KEY_LRIGHT, RBX_KEY_RUP, RBX_KEY_RDOWN, RBX_KEY_RLEFT, RBX_KEY_RRIGHT,
	// a whole stick, any direction: no single press gives it, so it is only stepped to (settings before: keyver < 5, None was 20)
	RBX_KEY_LANY, RBX_KEY_RANY,
	RBX_KEY_COUNT
};
// their bits among the buttons (bits SCE_CTRL_* and RE_PAD_* leave free; masked out of what a pad reports)
#define RBX_STICK_BITS_L 0x00C60000u
#define RBX_STICK_BITS_R 0xF0000000u
#define RBX_STICK_BITS (RBX_STICK_BITS_L | RBX_STICK_BITS_R)
#define RBX_KEY_EXTRA 3 // keys an action can have besides its first

// Turbo: one Vita button (RBX_KEY_NONE = off) fires one game button (1..RBX_TURBO_BUTTONS)
#define RBX_KEY_NONE RBX_KEY_COUNT
// Combo keys: a Vita button (Cross, Circle, Square, Triangle, L, R = RBX_KEY_CROSS..RBX_KEY_R) can act as two other Vita
// buttons pressed together. combo[key] = bit mask (1 << RBX_KEY_*) of those two, 0 = normal
#define RBX_COMBO_KEYS 6
// keys that can be combo keys: those six, then a controller's L2 / R2 / L3 / R3 (the pairs stay of the six)
#define RBX_COMBO_SRC 10
#define RBX_TURBO_BUTTONS 6
enum { RBX_TURBO_FAST, RBX_TURBO_MEDIUM, RBX_TURBO_SLOW, RBX_TURBO_SPEED_COUNT };

#define RBX_DIP_BYTES_MAX 8
#define RBX_PROFILES 13 // re_pad's RE_PAD_PROFILES: the keys belong to a device (Vita, each kind of controller)
#define RBX_PORTS 2 // players with their own controls (Port 1 = 1P, Port 2 = 2P)

// one player's controls (Controls > Port N Controls; RetroArch style)
typedef struct {
	int keymap[RBX_ACT_COUNT];          // RBX_KEY_*
	int keyx[RBX_ACT_COUNT][RBX_KEY_EXTRA]; // more keys for the same action, from the first; RBX_KEY_NONE: no more
	int smap[2][RBX_SDIR_COUNT];        // RBX_STGT_*: what each direction of the left (0) and right (1) stick presses
	int turbo_key;                      // RBX_KEY_* or RBX_KEY_NONE (off)
	int turbo_btn;                      // game button fired by turbo, 1..RBX_TURBO_BUTTONS
	int turbo_speed;                    // RBX_TURBO_FAST..SLOW
	int combo[RBX_KEY_COUNT];          // combo keys (see RBX_COMBO_KEYS)
} rbx_port;

// Game list grouping (options): the left column by machine (board, under the maker tabs), year, genre or publisher; All: no
// groups, the games over the whole width
enum { RBX_GROUP_MACHINE, RBX_GROUP_YEAR, RBX_GROUP_GENRE, RBX_GROUP_PUBLISHER, RBX_GROUP_ALL, RBX_GROUP_COUNT };
// Game list filter (SELECT cycles in this order)
enum { RBX_LIST_PLAYABLE, RBX_LIST_ALL, RBX_LIST_FAVORITES, RBX_LIST_FILTER_COUNT };

typedef struct {
	int show_fps;
	int aspect;
	int filter;
	int scan_type;                      // RBX_SCAN_*
	int scan_strength;                  // RBX_SCAN_LIGHT..HEAVY
	int border;                         // RBX_BORDER_*
	int border_color;                   // RBX_BCOL_*
	int menu_confirm;                   // RBX_CONFIRM_*
	int start_stage;                    // 0 = Stage 1 ~ 6 = Final (rbx_core_set_start_stage)
	rbx_port port[RBX_PROFILES]; // controls profiles: one per device (re_pad_profile), not per player
	int pad_src[RBX_PORTS];             // RE_PAD_SRC_* of each player's controller (global only; 0 = Auto)
	int input_engine;                   // RE_PAD_ENGINE_* (global only): Native or the bundled RePad plugin
	int p2_added;                       // the 2P Controller row added by hand (Controls: Add / Delete), global only
	int cheat_health;                   // 1P infinite health
	int dip_count;                      // number of saved DIP bytes (0 = driver defaults). Per-game file <set>.cfg
	uint8_t dip[RBX_DIP_BYTES_MAX];
	char language[8];                   // the menus' language (lang.h: "eng", "kor", ...)
	char last_game[16];                 // set name of the last game played (game list cursor)
	int list_filter;                    // game list filter: RBX_LIST_*
	int list_group;                     // game list grouping: RBX_GROUP_*
	int show_list;                      // show the list next start even with one game (options screen Select game)
	int rotate;                         // RBX_ROT_*
	int touch_keys;                     // the touch screen buttons press their keys
	int touch_show;                     // the touch buttons are drawn over the game
	int theme;                          // RBX_THEME_* (global only)
	int neon[7];                        // the neon corridor's knobs, RBX_NEON_* (neon.h; global only)
	int neon_list[7];                   // the game list's corridor (user 11-22: set apart from the games' one)
	int neon_list_gap;                  // its walls' distance (px, the middle of the screen between them)
	int frame_buffer;                   // RBX_FRAMEBUF_* (per game or global)
} rbx_settings;

extern rbx_settings g_settings;

// Settings folder (ux0:data/rebox/). App-wide settings.cfg, per-game DIP <set>.cfg
#define RBX_DATA_DIR "ux0:data/rebox/"
#define RBX_OLD_DATA_DIR "ux0:data/nbbatman/" // folder from the nbbatman-only era (settings/ROMs carried over)
#define RBX_M92_DATA_DIR "ux0:data/m92arcade" // folder from the "M92 Arcade" era: renamed to RBX_DATA_DIR on first start
void rbx_settings_move_game_files(void);   // (at start: older games' .cfg files to cfg/)
void rbx_settings_load(void);
void rbx_settings_save(void);
// After picking a game: its DIP (if none, 157/247 for the nbbatman family, driver defaults for other games)
void rbx_settings_load_game(const char *set);
// Per-game settings (<set>_game.cfg: display, filter, scanlines, margin, FPS, button mapping, turbo). If present, rbx_settings_load_game
// overlays them on the global settings. While active, rbx_settings_save writes those items only to that file and keeps global values in the global file
int rbx_settings_has_game(void);     // current game has its own settings
int rbx_settings_save_game(void);    // save current values as this game's settings (global file untouched). 1 on success
int rbx_settings_delete_game(void);  // delete per-game settings and revert to global values. 1 on success
// Game list options: drop the current game's settings (if any) so menus edit and rbx_settings_save writes only the global
// values. The next rbx_settings_load_game applies that game's file again
void rbx_settings_use_global(void);

// the touch button layout file that goes with the settings in use: the game's (<set>_touch.cfg, when it has its own settings)
// or the global touch.cfg (keep = 1: the layout now is written to it; else it is read). Called by the settings functions
void rbx_settings_touch_follow(int keep);

// Favorite games (favorites.cfg next to settings.cfg, one set name per line; loaded by rbx_settings_load)
int rbx_fav_is(const char *set);
void rbx_fav_toggle(const char *set); // add or remove, then save

// Default button mapping (attack □, jump △, coin SELECT, start START, turbo off): every port and the touch buttons
void rbx_settings_default_keys(void);
// one port's default buttons, stick, turbo and combo keys
void rbx_settings_default_port(int port);

// Frames per turbo cycle (half pressed, half released)
int rbx_turbo_period(int port);
// RBX_ACT_* of the game button fired by turbo (turbo_btn)
int rbx_turbo_act(int port);

// RBX_KEY_* -> SCE_CTRL_* bit, display name (RBX_KEY_NONE is 0, "None")
uint32_t rbx_key_mask(int key);
uint32_t rbx_act_bits(const rbx_port *p, int act);   // the keys of an action, the more keys too
uint32_t rbx_stick_bits(int lx, int ly, int rx, int ry);   // the sticks' directions as RBX_KEY_* bits
void rbx_stick_preset(rbx_port *p, int st, int dpad);   // a stick as the d-pad, or pressing nothing
const char *rbx_key_name(int key);

#endif
