/*  ReBOX emulation core: frontend C API (Vita, PSP, PC harness).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_CORE_H
#define RBX_CORE_H

#include <stddef.h>
#include <stdint.h>
#include "rbx_prof.h"   // (the sampled activity, ADR-155: rbx_prof_cur, rbx_prof_get_name for the sampler)

#ifdef __cplusplus
extern "C" {
#endif

// ---- game list (usable before rbx_init) ----
int rbx_game_count(void);
const char *rbx_game_set(int i);      // ROM set name (zip name)
const char *rbx_game_parent(int i);   // parent set for clones, else NULL
const char *rbx_game_title(int i);
const char *rbx_game_year(int i);
const char *rbx_game_maker(int i);
const char *rbx_game_board(int i);    // "Taito B System" ...: game list tab
int rbx_game_rot(int i);              // 0 / 90 / 180 / 270 (clockwise, MAME ROTx)
int rbx_game_flags(int i);            // RBX_GF_* (rbx.h)
int rbx_game_find(const char *set);   // index or -1
int rbx_game_players(int i);          // players with controls
int rbx_game_buttons(int i);          // buttons of player 1

// ---- core modules (rbx_module.h: a maker's boards, src/core/<id>/) ----
int rbx_module_count(void);
const char *rbx_module_id(int m);     // "taito"
const char *rbx_module_name(int m);   // "Taito"
int rbx_game_module(int i);           // the game's module
// the game's system in the list (the module's groups: Taito B / O / X / H / L / Z / F2 / F3 / SJ System, Classic, Standalone) and
// the tabs' order (the modules in order, then each module's groups)
const char *rbx_game_group(int i);
int rbx_game_group_order(int i);
int rbx_board_order(const char *board);   // the board's place in the module order

// ---- the Neo Geo system BIOS (MAME's ROM_SYSTEM_BIOS choices): used from the next game loaded; a BIOS whose file is not
// in neogeo.zip falls back to MAME's default (rbx_bios_note says so, "" else). 0 = the default, Europe MVS Ver. 2
int rbx_bios_count(void);
const char *rbx_bios_name(int i);   // MAME's name ("unibios40")
const char *rbx_bios_desc(int i);   // "Universe BIOS 4.0"
int rbx_bios_get(void);
void rbx_bios_set(const char *name);   // by MAME's name (unknown: the default)
const char *rbx_bios_note(void);

// ---- running a game ----
typedef uint32_t (*rbx_color_fn)(int r, int g, int b);
typedef struct {
	const char *set;                // ROM set to run
	const char *const *rom_dirs;    // folders searched for <set>.zip (and the parent's zip), each ending in '/'
	int rom_dir_count;
	int sample_rate;                // output rate (48000), 0 = no sound
	int bytes_per_pixel;            // 2 or 4 (rbx_run_frame output)
	rbx_color_fn color;             // NULL = 0x00RRGGBB (4 bytes) / RGB565 (2 bytes)
	void (*log)(const char *msg);   // NULL = stderr
	void (*progress)(int permille, void *user); // ROM loading progress, may be NULL
	void *progress_user;
	const char *nvram_dir;          // folder for EEPROM / NVRAM files ('/' at the end), NULL = none
	const char *cache_dir;          // folder for large ROM files kept outside memory ('/' at the end), NULL = in memory
	void (*wait_hook)(void);        // called while one thread waits for another (NULL = spin); rbx_wait_site() says on what
	uint64_t (*time_us)(void);      // a microsecond clock for the drivers' profiling logs (rbx_clock_us), may be NULL
	// memory for generated code (ARM: the F3 ESP and Konami DASP programs), NULL = none: jit_alloc once (8MB, the core
	// gives out its parts), jit_write_begin
	// before writing code into it, jit_write_end after (caches synced)
	void *(*jit_alloc)(unsigned size);
	void (*jit_write_begin)(void);
	void (*jit_write_end)(void *code, unsigned size);
	// the Neo Geo BIOS by MAME's name (rbx_bios_set), NULL / "" = the default. Given here: the core's static data is
	// the first game's again at each rbx_init, a choice made with rbx_bios_set between games would be lost
	const char *neogeo_bios;
} rbx_config;

int rbx_init(const rbx_config *cfg);   // 0 = ok, else see rbx_error(). Every call after the first starts from the
                                       // core's static state of the first (tools/rbx_state.ld): a fresh core per game
size_t rbx_fresh_size(void);           // bytes kept of that state (0 = none: the link has no rbx_state.ld)
size_t rbx_fresh_range(void);          // bytes of the core's static data
void rbx_exit(void);
const char *rbx_error(void);
int rbx_running_game(void);            // index of the running game (-1 = none)

// visible size of the unrotated screen. Rotation is rbx_game_rot()
void rbx_screen(int *w, int *h);
// the running board's frame rate in milli-Hz (its own, or the rate its sound shows; 60000 until known)
uint32_t rbx_frame_mhz(void);
// on: the frontend runs the board's frames at their own rate (a display frame without one shows the last again), so
// a frame's sound is played in 1 / its rate, not stretched to a 60 Hz frame (ADR-211)
void rbx_set_native_pace(int on);
// frames per second of the board (60.0 for most)
double rbx_fps(void);

// inputs: per player bits, plus system bits
#define RBX_IN_UP     0x0001
#define RBX_IN_DOWN   0x0002
#define RBX_IN_LEFT   0x0004
#define RBX_IN_RIGHT  0x0008
#define RBX_IN_B1     0x0010
#define RBX_IN_B2     0x0020
#define RBX_IN_B3     0x0040
#define RBX_IN_B4     0x0080
#define RBX_IN_B5     0x0100
#define RBX_IN_B6     0x0200
#define RBX_IN_START  0x0400
#define RBX_IN_COIN   0x0800
#define RBX_IN_SERVICE 0x1000   // per player service coin (SERVICE1..)
void rbx_set_input(int player, uint32_t bits);  // player 0..3
#define RBX_SYS_TEST    1       // test / service mode switch (IPT_SERVICE)
#define RBX_SYS_TILT    2
#define RBX_SYS_SERVICE 4       // service coin
void rbx_set_system(uint32_t bits);
void rbx_set_analog(int player, int axis, int delta);  // axis 0 = x, 1 = y (trackball counts per frame)

// run one frame. fb receives the visible screen (rbx_screen size, cfg bytes_per_pixel, pitch in bytes; NULL = no drawing).
// audio receives interleaved stereo int16 at the output rate; returns the number of stereo samples written
// (about sample_rate / fps, at most max_samples)
int rbx_run_frame(void *fb, int pitch, int16_t *audio, int max_samples);
void rbx_reset(void);

// render thread: with it on, rbx_run_frame does not draw (fb is ignored); a second thread loops on rbx_render_frame, which
// waits for the frame the last rbx_run_frame captured and draws it to fb. Results are the same as drawing inline.
// Turn it on or off between frames only
void rbx_set_render_thread(int on);
int rbx_render_frame(void *fb, int pitch);   // 1 = drew a frame, 0 = stopped (rbx_render_stop)
void rbx_render_stop(void);
// another frontend thread (a spare core) helps draw the frame the render thread is drawing; 0 = nothing to do now
int rbx_render_help(void);
int rbx_render_help_one(int helper);   // (rbx.h)
int rbx_video_depth(void);   // frames the emulation may post ahead of the render thread (1, or 2: show frame n - 2)
int rbx_render_can_help(void);   // the running game splits its drawing (a helper thread is worth starting)
// worker thread for a driver's own job (rbx.h rbx_aux_*): a thread loops on rbx_aux_run until it returns 0. Turn it on
// or off between frames only
void rbx_set_aux_thread(int on);
int rbx_aux_has_job(void);  // the running game has a job for the worker
int rbx_aux_by_render(void);   // (rbx.h rbx_aux_set_by_render)
int rbx_aux_run(void);    // 1 = ran the job (or found nothing to do), 0 = stopped (rbx_aux_stop)
void rbx_aux_stop(void);
// the second worker thread (rbx.h rbx_aux2_*), the same way
void rbx_set_aux2_thread(int on);
int rbx_aux2_has_job(void);
int rbx_aux2_alone(void);   // (rbx.h rbx_aux2_set_alone: 1 = CPU3, RBX_AUX2_BY_RENDER = CPU2 below the render thread)
#ifndef RBX_AUX2_BY_RENDER
#define RBX_AUX2_BY_RENDER 2
#endif
int rbx_aux3_alone(void);     // (rbx.h rbx_aux3_set_alone: 1 = alone on CPU3, low)
int rbx_aux3_has_job(void);   // (the third: rbx_set_aux3_thread / rbx_aux3_run / rbx_aux3_stop, the same way)
void rbx_set_aux3_thread(int on);
int rbx_aux3_run(void);
int rbx_aux3_try(void);       // the job if kicked since it last ran (1), else 0 at once (the thread then sleeps until
void rbx_set_aux3_wake(void (*wake)(void));   // wake: called by each kick, NULL = none)
void rbx_aux3_stop(void);
// the io worker (rbx.h rbx_io_*, ADR-160): a thread for a game with rbx_io_wanted() calls rbx_io_run (the requests
// queued, read in order; 0 = none, sleep until the wake given to rbx_set_io_wake or a short timeout). rbx_set_io_thread
// on once the thread runs, off before it is stopped (requests still queued are then read by the emulation thread)
int rbx_io_wanted(void);
void rbx_set_io_thread(int on);
void rbx_set_io_wake(void (*wake)(void));
int rbx_io_run(void);
int rbx_aux2_run(void);
void rbx_aux2_stop(void);

// DIP switches (and config fields) of the running game
int rbx_dip_count(void);
const char *rbx_dip_name(int d);
int rbx_dip_options(int d);
const char *rbx_dip_option_name(int d, int o);
int rbx_dip_get(int d);
void rbx_dip_set(int d, int o);
void rbx_dip_defaults(void);
int rbx_dip_raw(uint8_t *out, int max);          // for settings files
void rbx_dip_set_raw(const uint8_t *in, int n);

// save states (between frames). 0 = ok
int rbx_state_save(const char *path);
int rbx_state_load(const char *path);
// raw state in memory (size, then fill); for the frontends' quick save
uint32_t rbx_state_size(void);
int rbx_state_write(void *buf, uint32_t size);
// the same state given to write in parts (0 = ok), without a buffer of its size: for a game that leaves too little
// memory for one (a driver part that fills its block in place, rbx_state_var, refuses it)
int rbx_state_stream(int (*write)(void *user, const void *p, uint32_t n), void *user);
// rbx_state_read from a reader (0 = ok): read n bytes, skip n bytes, back to the start; size = the file's
int rbx_state_unstream(int (*read)(void *user, void *p, uint32_t n), int (*skip)(void *user, uint32_t n),
                       int (*rewind)(void *user), void *user, uint32_t size);
extern uint32_t rbx_state_load_us[4];   // the last rbx_state_load's parts (us): open, read, size + check, load
int rbx_state_read(const void *buf, uint32_t size);

// medal games (Taito O System): cabinet state for the frontend overlay
uint32_t rbx_medal_lamps(void);        // lamp outputs (button lamps, MAME taito_outb_w)
uint32_t rbx_medal_outputs(void);      // meters, hopper motor, bell (taito_outa_w)
uint32_t rbx_medal_paid(void);         // medals paid out by the hopper since start
uint32_t rbx_medal_inserted(void);     // medals inserted
int rbx_medal_hopper_running(void);
void rbx_medal_reset(void);            // clear the medal counters and the NVRAM (credits) and restart, at the next frame

// memory peek/poke on the main CPU (cheats)
uint8_t rbx_peek(uint32_t addr);
void rbx_poke(uint32_t addr, uint8_t v);

// MAME cheat XML (cheat.c): a set's cheats from its <set>.xml text (after rbx_init; the count, 0 = none usable). Each
// cheat has options: 0 off, then "On" or its parameter's values; they run every frame (rbx_run_frame), until rbx_exit
int rbx_cheat_load_xml(const char *xml, size_t len);
void rbx_cheat_clear(void);
int rbx_cheat_count(void);
const char *rbx_cheat_name(int i);
int rbx_cheat_options(int i);
const char *rbx_cheat_option_name(int i, int o);
int rbx_cheat_get(int i);
void rbx_cheat_set(int i, int o);
void rbx_cheat_frame(void);

#ifdef __cplusplus
}
#endif

// what the thread in wait_hook waits on (profiling): 0 other, 1 the render thread, 2..4 the workers (rbx_aux, aux2, aux3),
// 5 the driver's own wait (rbx_wait)
int rbx_wait_site(void);

#endif
