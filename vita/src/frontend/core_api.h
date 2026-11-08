// Frontend view of the ReBOX core: the core API (rbx_core.h) plus the calls the shared Re frontend (menus, input,
// video, threads; carried over from ReiREM) makes, mapped onto it (core_api.c)
#ifndef RBX_FRONTEND_CORE_API_H
#define RBX_FRONTEND_CORE_API_H

#include <stdint.h>
#include "rbx_core.h"
#include "rbx_games.h"

#define RBX_SCREEN_W 320
#define RBX_SCREEN_H 240
#define RBX_SCREEN_VERTICAL 1   // shown turned 90 degrees counter-clockwise (MAME ROT270)
#define RBX_SCREEN_FLIPPED  2   // 180 degrees

// logical inputs of the frontend (per player buttons, passed to the core as rbx_set_input bits)
enum {
	RBX_IN_P1_COIN, RBX_IN_P1_START, RBX_IN_P1_UP, RBX_IN_P1_DOWN,
	RBX_IN_P1_LEFT, RBX_IN_P1_RIGHT, RBX_IN_P1_B1, RBX_IN_P1_B2,
	RBX_IN_P2_COIN, RBX_IN_P2_START, RBX_IN_P2_UP, RBX_IN_P2_DOWN,
	RBX_IN_P2_LEFT, RBX_IN_P2_RIGHT, RBX_IN_P2_B1, RBX_IN_P2_B2,
	RBX_IN_P3_COIN, RBX_IN_P3_START, RBX_IN_P3_UP, RBX_IN_P3_DOWN,
	RBX_IN_P3_LEFT, RBX_IN_P3_RIGHT, RBX_IN_P3_B1, RBX_IN_P3_B2,
	RBX_IN_P4_COIN, RBX_IN_P4_START, RBX_IN_P4_UP, RBX_IN_P4_DOWN,
	RBX_IN_P4_LEFT, RBX_IN_P4_RIGHT, RBX_IN_P4_B1, RBX_IN_P4_B2,
	RBX_IN_X_RESET, RBX_IN_X_SERVICE,
	RBX_IN_P1_B3, RBX_IN_P1_B4, RBX_IN_P1_B5, RBX_IN_P1_B6,
	RBX_IN_P2_B3, RBX_IN_P2_B4, RBX_IN_P2_B5, RBX_IN_P2_B6,
	RBX_IN_DIGITAL_COUNT
};
void rbx_core_set_input(int index, int pressed);
void rbx_core_set_service(int test, int coin);   // test switch, service coin

const char *rbx_core_game_set(int i);
const char *rbx_core_game_parent(int i);
const char *rbx_core_game_title(int i);
const char *rbx_core_game_board(int i);
int rbx_core_game_count(void);
const char *rbx_core_game_name(void);   // running game
const char *rbx_core_set_name(void);
int rbx_core_button_count(void);
void rbx_core_screen(int *w, int *h, int *flags);

// run one frame: fb (pitch in bytes) receives the screen, audio up to max stereo samples; returns the samples written
int rbx_core_run_frame(void *fb, int pitch, int16_t *audio, int max);
int rbx_core_state_exists(const char *path);

static inline int rbx_core_dip_count(void) { return rbx_dip_count(); }
static inline const char *rbx_core_dip_name(int d) { return rbx_dip_name(d); }
static inline int rbx_core_dip_options(int d) { return rbx_dip_options(d); }
static inline const char *rbx_core_dip_option_name(int d, int o) { return rbx_dip_option_name(d, o); }
static inline int rbx_core_dip_get(int d) { return rbx_dip_get(d); }
static inline void rbx_core_dip_set(int d, int o) { rbx_dip_set(d, o); }
static inline void rbx_core_dip_defaults(void) { rbx_dip_defaults(); }
static inline int rbx_core_dip_raw(uint8_t *out, int max) { return rbx_dip_raw(out, max); }
static inline void rbx_core_dip_set_raw(const uint8_t *in, int n) { rbx_dip_set_raw(in, n); }
static inline const char *rbx_core_error(void) { return rbx_error(); }
static inline void rbx_core_reset(void) { rbx_reset(); }
static inline void rbx_core_exit(void) { rbx_exit(); }
// save states through the system's file calls in one piece (newlib's fread took ~200 ms for a 100 KB state)
int rbx_core_state_save(const char *path);
int rbx_core_state_load(const char *path);

// cheats (later: MAME cheat.xml), start stage / infinite health (ReiREM nbbatman only): none yet
static inline int rbx_core_has_cheats(void) { return 0; }
int rbx_core_cheats_load(const char *dir);
int rbx_core_cheat_count(void);
const char *rbx_core_cheat_name(int i);
int rbx_core_cheat_options(int i);
const char *rbx_core_cheat_option_name(int i, int o);
int rbx_core_cheat_get(int i);
void rbx_core_cheat_set(int i, int o);
#define RBX_STAGE_COUNT 7
static inline void rbx_core_set_start_stage(int s) { (void)s; }
static inline void rbx_core_set_infinite_health(int on) { (void)on; }

// threads: drawing on the CPU2 render thread (rbx_render_frame); sound and CPUs inside rbx_core_run_frame
static inline int rbx_core_render_frame(void *fb, int pitch) { return rbx_render_frame(fb, pitch); }
static inline void rbx_core_sound_frame(int16_t *audio) { (void)audio; }
static inline int rbx_core_audio_len(void) { return 1024; }
static inline void rbx_core_stop(void) { rbx_render_stop(); }
// the CPU1 worker: the driver's rbx_aux job (F3: the ES5510 effects DSP)
static inline int rbx_core_ym_worker(void) { return rbx_aux_has_job(); }
static inline void rbx_core_ym_worker_enable(int on) { rbx_set_aux_thread(on); }
static inline void rbx_core_ym_worker_run(void) { while (rbx_aux_run()) {} }
static inline int rbx_core_z80_worker(void) { return 0; }
static inline void rbx_core_z80_worker_enable(int on) { (void)on; }
static inline void rbx_core_z80_worker_run(void) {}

#endif
