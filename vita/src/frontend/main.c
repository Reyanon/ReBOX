// ReBOX Vita port: frontend entry point
// One process runs the game list, then a game, then the list again (in-app game switching, no relaunch): each game is
// loaded on a loader thread while the main thread draws the loading screen, and is shut down completely (sound, render
// and YM threads stopped, core exit, game textures and audio port released) before the next one
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <malloc.h>

#include <psp2/appmgr.h>
#include <psp2/ctrl.h>
#include <psp2/display.h>
#include <psp2/io/fcntl.h>
#include <psp2/io/stat.h>
#include <psp2/kernel/cpu.h>
#include <psp2/kernel/processmgr.h>
#include <psp2/kernel/sysmem.h>
#include <psp2/kernel/threadmgr.h>
#include <vita2d.h>

#include "frontend/lang.h"
#include "frontend/audio.h"
#include "frontend/resample.h"
#include "frontend/input.h"
#include "frontend/medal.h"
#include "frontend/present.h"
#include "frontend/menu.h"
#include "common/re_pad.h"
#include "frontend/settings.h"
#include "frontend/sampler.h"
#include "frontend/testpad.h"
#include "frontend/states.h"
#include "frontend/threads.h"
#include "frontend/touch.h"

// the emulation thread helps draw the frame only when its frame so far took less than this (us)
#define RENDER_HELP_US 11000
#include "frontend/update.h"
#include "frontend/gamelist.h"
#include "frontend/video.h"
#include "frontend/core_api.h"
#include "rbx_games.h"
#include "platform/clock.h"
#include "platform/hud.h"
#include "platform/log.h"
#include "platform/perf.h"

#define ROM_DIR      RBX_DATA_DIR "roms/" // where the user puts ROMs (created at startup)
// ROM folder candidates: roms/, then the data folder itself; first hit per game (rbx_games.c)
// the ROM folders: roms/, the data folder, then the other folders the game lists' "rompath"s name (user 11-22)
#define ROM_DIRS_MAX 18
static const char *k_rom_dirs[ROM_DIRS_MAX] = { ROM_DIR, RBX_DATA_DIR };
static int s_rom_dirs = 2;
#define ROM_DIRS s_rom_dirs
// the ROM folders again: roms/, the data folder, the two chosen in Settings, then the game lists' (menu.c calls it
// when a chosen folder changes). Returns how many
int rbx_rom_dirs_refresh(void)
{
	int n = 2;
	for (int k = RBX_FOLDER_ROM1; k <= RBX_FOLDER_ROM2; k++) {
		const char *d = rbx_folder(k);
		int known = !d[0];
		for (int i = 0; i < n && !known; i++)
			known = !strcasecmp(k_rom_dirs[i], d);
		if (!known) {
			k_rom_dirs[n++] = d;
			RBX_LOG("roms: also in %s (Settings)", d);
		}
	}
	const int m = n;
	n += rbx_gamelist_rom_dirs(k_rom_dirs, m, k_rom_dirs + m, ROM_DIRS_MAX - m);
	for (int i = m; i < n; i++)
		RBX_LOG("roms: also in %s (game list)", k_rom_dirs[i]);
	s_rom_dirs = n;
	return n;
}
static int s_kctrl_restarted;   // started again by rbx_kctrl_init (the first start after boot)

#ifndef RBX_INTERLEAVE_MUL
#define RBX_INTERLEAVE_MUL 0
#endif
#define CLEAR_COLOR RGBA8(0x00, 0x00, 0x00, 0xFF)

// the newlib heap holds the ROM regions: PGM's largest sets (martmast, kov2p) take 116 MB of regions. The drivers'
// static buffers (BSS, 63 MB with every board in) and the heap share the app's memory: at 168 MB nothing was left for
// the threads' stacks (render thread creation 0x80024302), so 152 MB
unsigned int _newlib_heap_size_user = 152 * 1024 * 1024;

// time the threads spend waiting for each other in the core (perf log: emu and render are busy time, sound = the
// emulation thread's wait): [0] the emulation (main) thread, [1] the render thread, [2] others
SceUID g_render_tid = -1;
static SceUID s_main_tid = -1;
volatile uint32_t g_core_wait_us[3];
static uint32_t s_wait_site_us[6];   // the emulation thread's, by rbx_wait_site
static void core_wait(void)
{
	const int site = rbx_wait_site();
	const uint64_t t0 = rbx_time_us();
	sceKernelDelayThread(50);
	const uint32_t dt = (uint32_t)(rbx_time_us() - t0);
	const SceUID me = sceKernelGetThreadId();
	g_core_wait_us[me == s_main_tid ? 0 : me == g_render_tid ? 1 : 2] += dt;
	if (me == s_main_tid)
		s_wait_site_us[site % 6] += dt;
}

// executable memory for the core's generated code (the F3 ESP program): one block for the app's life
static SceUID s_jit_block = -1;
static void *s_jit_base;
static unsigned s_jit_size;
static void *jit_alloc(unsigned size)
{
	size = (size + 0xfffff) & ~0xfffffu;
	if (s_jit_block >= 0)
		return size <= s_jit_size ? s_jit_base : NULL;
	SceUID b = sceKernelAllocMemBlockForVM("rbx_jit", size);
	if (b < 0 || sceKernelGetMemBlockBase(b, &s_jit_base) < 0) {
		RBX_LOG("jit: no executable memory (0x%08x)", (unsigned)b);
		return NULL;
	}
	s_jit_block = b;
	s_jit_size = size;
	return s_jit_base;
}
// (ReST-V ADR-043: the VM domain opened once, by the emulation thread, and left open; only the written bytes synced)
static void jit_write_begin(void)
{
	static int open;
	if (!open) {
		sceKernelOpenVMDomain();
		open = 1;
	}
}
static void jit_write_end(void *code, unsigned size)
{
	if (size)   // (ReST-V ADR-175: a sync of length 0 powered the device off)
		sceKernelSyncVMDomain(s_jit_block, code, size);
}

static void core_log(const char *msg)
{
	RBX_LOG("core: %s", msg);
	(void)msg;
}

#ifdef RBX_ENABLE_LOG
// test (log builds): RBX_DATA_DIR switchtest = "frames rounds [warmup]" then set names, one per line. Each game runs that
// many frames and goes back to the list as "Select game" does; the sets take turns for that many rounds (in-app game
// switching, docs/DECISIONS.md). Without warmup: a quick save at half, its load at three quarters (save state test).
// With warmup (speed measurement): the game starts from its measuring point RBX_DATA_DIR perf/<set>.sav (made the first
// time: warmup frames into the attract mode, then saved), logs "switchtest: measuring" and runs the frames from there.
// "frames rounds warmup 1": from there player 1 also plays: a coin, start, then moves and buttons (sw_play); the
// measurement starts at "switchtest: playing" (in the game)
#define SW_MAX 1024
static struct { char set[SW_MAX][24]; int n, frames, warmup, total, done, active, play; } s_sw;

static const char *s_play_set = "";
// the inputs of player 1 at frame f of the play (after the measuring point)
static void sw_play(uint64_t f)
{
	// (the game may still be starting, or lost its lives: a coin and start every 10 s, or the measurement is of the attract
	// mode; "switchtest: playing" after the first few)
	const uint64_t c = f % 600;
	const int coin = c >= 20 && c < 30, start = (c >= 80 && c < 90) || (c >= 140 && c < 150);
	const int on = f >= 200;
	const uint64_t k = f % 120;
	rbx_core_set_input(RBX_IN_P1_COIN, coin);
	rbx_core_set_input(RBX_IN_P1_START, start);
	rbx_core_set_input(RBX_IN_P1_LEFT, on && k < 50);
	rbx_core_set_input(RBX_IN_P1_RIGHT, on && k >= 60 && k < 110);
	rbx_core_set_input(RBX_IN_P1_UP, on && f % 300 < 20);
	rbx_core_set_input(RBX_IN_P1_DOWN, on && f % 300 >= 150 && f % 300 < 170);
	rbx_core_set_input(RBX_IN_P1_B1, f >= 160 && f % 16 < 6);
	rbx_core_set_input(RBX_IN_P1_B2, on && f % 90 < 5);
	if (f == 1300)
		RBX_LOG("switchtest: playing");
	// (what the play shows: ux0:data/rebox/perf/<set>_play.ppm, made once: writing it stops a frame for ~0.7 s. Whether
	// it is there is looked up before the measurement: the lookup alone stopped a frame for 25-60 ms, cupfinal, twinqix)
	static char path[160];
	static int need;
	if (f == 1) {
		snprintf(path, sizeof(path), RBX_DATA_DIR "perf/%s_play.ppm", s_play_set);
		SceIoStat st;
		need = sceIoGetstat(path, &st) < 0;
	}
	if (f == 1250 && need)   // (before the measurement: writing it stops a frame)
		rbx_video_dump_rendered(path);
}
static void switchtest_load(void)
{
	FILE *f = fopen(RBX_DATA_DIR "switchtest", "r");
	if (!f)
		return;
	int rounds = 0;
	char line[64] = "";
	if (fgets(line, sizeof(line), f) && sscanf(line, "%d %d %d %d", &s_sw.frames, &rounds, &s_sw.warmup, &s_sw.play) >= 2)
		while (s_sw.n < SW_MAX && fscanf(f, "%23s", s_sw.set[s_sw.n]) == 1)
			s_sw.n++;
	fclose(f);
	s_sw.total = s_sw.n * rounds;
	RBX_LOG("switchtest: %d sets, %d frames each, %d games, warmup %d, play %d", s_sw.n, s_sw.frames, s_sw.total, s_sw.warmup,
		s_sw.play);
}
#endif
// (the switchtest is running: log builds only)
#ifdef RBX_ENABLE_LOG
#define SW_ACTIVE (s_sw.active)
#else
#define SW_ACTIVE 0
#endif

// pick a game in the list (log builds: the dev loop's RBX_DATA_DIR rbx_game file runs that game once at start)
static const rbx_game_found *choose_game(const char *notice)
{
	if (g_settings.show_list) { // (old options flag; the list is always shown now, so just clear it)
		g_settings.show_list = 0;
		rbx_settings_save();
	}
#ifdef RBX_ENABLE_LOG
	s_sw.active = 0;
	if (s_sw.done < s_sw.total) {
		static rbx_game_found found[RBX_GAMES_MAX];
		int present = 0;
		const int n = rbx_games_scan(k_rom_dirs, ROM_DIRS, found, RBX_GAMES_MAX, &present);
	sw_next:;
		const char *want = s_sw.set[s_sw.done % s_sw.n];
		s_sw.done++;
		for (int i = 0; i < n; i++)
			if (found[i].present && !strcmp(rbx_core_game_set(found[i].game), want)) {
				RBX_LOG("switchtest: game %d/%d %s", s_sw.done, s_sw.total, want);
				s_sw.active = 1;
				return &found[i];
			}
		RBX_LOG("switchtest: %s not found", want);
		if (s_sw.done < s_sw.total)
			goto sw_next;   // (the next set)
	}
	static int autostart_done;
	if (!autostart_done) {
		autostart_done = 1;
		FILE *f = fopen(RBX_DATA_DIR "rbx_game", "r");
		if (f) {
			static rbx_game_found found[RBX_GAMES_MAX];
			char want[24] = { 0 };
			if (fgets(want, sizeof(want), f))
				want[strcspn(want, "\r\n")] = 0;
			fclose(f);
			int present = 0;
			const int n = rbx_games_scan(k_rom_dirs, ROM_DIRS, found, RBX_GAMES_MAX, &present);
			for (int i = 0; i < n; i++)
				if (found[i].present && !strcmp(rbx_core_game_set(found[i].game), want)) {
					RBX_LOG("games: %s (rbx_game)", want);
					return &found[i];
				}
		}
	}
#endif
	return rbx_game_select(k_rom_dirs, ROM_DIRS, ROM_DIR, notice);
}

// core configuration and how the frontend runs it (threads: none yet, the core does the frame in rbx_run_frame)
typedef struct {
	rbx_config c;
	int pipelined, threaded, render_threaded;
} fe_config;

static void load_progress(int permille, void *user)
{
	(void)user;
	rbx_loading_set(permille);
}

// The core init (ROM zip reading, graphics decoding) runs on this thread (CPU1, idle until the game starts); the main
// thread only draws the loading screen, one frame per vblank, until it is done (as ReSys12 ADR-048)
static struct {
	const fe_config *cfg;
	int rc;
	volatile int done;
} s_ld;
static int load_thread(SceSize args, void *argp)
{
	(void)args;
	(void)argp;
	s_ld.rc = rbx_init(&s_ld.cfg->c);
	__atomic_store_n(&s_ld.done, 1, __ATOMIC_RELEASE);
	return sceKernelExitDeleteThread(0);
}
static int core_init_shown(const fe_config *cfg)
{
	s_ld.cfg = cfg;
	s_ld.rc = -1;
	s_ld.done = 0;
	const SceUID th = sceKernelCreateThread("rbx_load", load_thread, 0x60, 1024 * 1024, 0, SCE_KERNEL_CPU_MASK_USER_1, NULL);
	if (th < 0 || sceKernelStartThread(th, 0, NULL) < 0) { // (no thread: load here, the screen waits)
		if (th >= 0)
			sceKernelDeleteThread(th);
		RBX_LOG("load: no loader thread (0x%08x)", (unsigned)th);
		return rbx_init(&cfg->c);
	}
	while (!__atomic_load_n(&s_ld.done, __ATOMIC_ACQUIRE))
		rbx_loading_frame();
	sceKernelWaitThreadEnd(th, NULL, NULL);
	return s_ld.rc;
}

static int s_ym_worker; // other boards: YM2151 synthesis worker thread in use

// reset/load: stop sound and render threads, change core state, then restart them
static int with_threads_stopped(const fe_config *cfg, uint64_t *emu_frames, const char *load_path)
{
	const uint64_t t0 = rbx_time_us();
	rbx_present_drain();   // (the frames posted shown first: the present thread waits for the render thread's)
	if (cfg->threaded || s_ym_worker)
		rbx_threads_stop();
	if (cfg->render_threaded)
		rbx_video_stop_render_thread();
	const uint64_t t1 = rbx_time_us();
	int rc = 0;
	if (load_path) {
		rc = rbx_core_state_load(load_path);
		RBX_LOG("state: threads stopped in %.1f ms, the load %.1f ms (open %.1f read %.1f check %.1f load %.1f)", (t1 - t0) / 1000.0, (rbx_time_us() - t1) / 1000.0, rbx_state_load_us[0] / 1000.0, rbx_state_load_us[1] / 1000.0, rbx_state_load_us[2] / 1000.0, rbx_state_load_us[3] / 1000.0);
		if (rc != 0)
			rbx_core_reset(); // the sound side ran one extra line while stopping threads, so it can't just continue
	} else {
		rbx_core_reset();
	}
	*emu_frames = 0;
	if (cfg->render_threaded) {
		rbx_video_reset_rendered();
		rbx_video_start_render_thread();
	}
	if (cfg->threaded)
		rbx_threads_start_sound();
	if (s_ym_worker)
		rbx_threads_start_ym();
	return rc;
}

// the frame buffer (ADR-195) as the settings want it now: the present thread started or stopped, and the sound's ring
// a frame fuller with it. At the game's start and when the options close (only with the render thread: the frames
// it posts are the render thread's)
static void frame_buffer_apply(const fe_config *cfg)
{
	const int want = cfg->render_threaded && rbx_present_wanted();
	if (want && !rbx_present_running())
		rbx_present_start();
	else if (!want && rbx_present_running())
		rbx_present_stop();
	rbx_audio_set_extra_frames(rbx_present_running());
}

static void reset_game(const fe_config *cfg, uint64_t *emu_frames)
{
	with_threads_stopped(cfg, emu_frames, NULL);
	RBX_LOG("reset");
}

// save state (slot 0-9 or RBX_STATE_QUICK). Result shown at the bottom of the screen
static void save_state(int slot)
{
	char path[128];
	rbx_state_path(slot, path, sizeof(path));
	const uint64_t t0 = rbx_time_us();
	int rc = rbx_core_state_save(path);
	RBX_LOG("state: save %s rc %d, %.1f ms", path, rc, (rbx_time_us() - t0) / 1000.0);
	if (rc == 0)
		rbx_osd_show(T("Saved: %s"), rbx_state_slot_name(slot));
	else
		rbx_osd_show(T("Save failed: %s"), rbx_state_slot_name(slot));
	rbx_menu_states_changed();
}

static void load_state(const fe_config *cfg, uint64_t *emu_frames, int slot)
{
	char path[128];
	rbx_state_path(slot, path, sizeof(path));
	if (!rbx_core_state_exists(path)) {
		rbx_osd_show(T("No state: %s"), rbx_state_slot_name(slot));
		return;
	}
	const uint64_t t0 = rbx_time_us();
	int rc = with_threads_stopped(cfg, emu_frames, path);
	RBX_LOG("state: load %s rc %d, %.1f ms", path, rc, (rbx_time_us() - t0) / 1000.0);
	if (rc == 0)
		rbx_osd_show(T("Loaded: %s"), rbx_state_slot_name(slot));
	else
		rbx_osd_show(T("Load failed (game was reset): %s"), rbx_state_slot_name(slot));
}

// keep the system from auto-suspending or turning the screen off while running without input (demo viewing, remote
// measurement); same as PSP
// scePowerTick. Without it the system paused the app after ~10s of no input
// (ADR-211) a board more than 5% slower than the display (Midway T / Y units 54.7 Hz, Irem 55 Hz) runs at its own
// rate: at a vblank with no board frame due, the last frame is shown again and its sound is not stretched. A board
// closer to the display's rate runs a frame per vblank, its sound stretched to it (ADR-009), as smooth as the display.
typedef struct {
	uint32_t vcount, disp_mhz;
	int64_t acc;   // milli-Hz-vblanks: a board frame is due when it reaches disp_mhz
} pace_t;

static void pace_start(pace_t *p)
{
	float hz = 0.0f;
	p->disp_mhz = sceDisplayGetRefreshRate(&hz) >= 0 && hz > 50.0f && hz < 70.0f ? (uint32_t)(hz * 1000.0f + 0.5f) : 60000;
	p->vcount = (uint32_t)sceDisplayGetVcount();
	p->acc = p->disp_mhz;
}

// 1: a board frame is due at this vblank
static int pace_run(pace_t *p)
{
	const uint32_t fmhz = rbx_core_frame_mhz();
	const int native = fmhz < p->disp_mhz / 100 * 95;
	rbx_core_set_native_pace(native);
	const uint32_t vc = (uint32_t)sceDisplayGetVcount();
	uint32_t d = vc - p->vcount;
	p->vcount = vc;
	if (!native) {
		p->acc = p->disp_mhz;
		return 1;
	}
	if (d > 4)
		d = 4;   // (after a long frame or the menu: no catching up)
	p->acc += (int64_t)d * fmhz;
	if (p->acc < (int64_t)p->disp_mhz)
		return 0;
	p->acc -= p->disp_mhz;
	if (p->acc > (int64_t)p->disp_mhz)
		p->acc = p->disp_mhz;   // (a board slower than its own rate here: one frame per loop, no debt)
	return 1;
}

static void power_tick(void)
{
	static unsigned tick;
	if (++tick % 30 == 0)
		sceKernelPowerTick(SCE_KERNEL_POWER_TICK_DEFAULT);
}

enum { GAME_LIST, GAME_EXIT };
static char s_notice[256]; // shown in the game list next time (a game that did not start)

// one game from loading to its end. Returns GAME_LIST (back to the game list) or GAME_EXIT (the app exits)
static int run_game(const rbx_game_found *game)
{
	const char *set = rbx_core_game_set(game->game);
	RBX_LOG("game: %s", set);
	// the loading screen first (the game's settings are read under it, written once the game is up)
	rbx_loading_title(rbx_core_game_title(game->game));
	rbx_loading_progress(0);
	rbx_settings_load_game(set);
	static const char *dirs[ROM_DIRS_MAX];
	for (int i = 0; i < ROM_DIRS; i++)
		dirs[i] = k_rom_dirs[i];
	fe_config cfg = {
		.c = {
			.set = set,
			.rom_dirs = dirs,
			.rom_dir_count = ROM_DIRS,
			.sample_rate = RBX_AUDIO_RATE,
			.bytes_per_pixel = 2,
			.color = rbx_video_color,
			.log = core_log,
			.time_us = rbx_time_us,
			.progress = load_progress,
			// (switchtest: no NVRAM / EEPROM file read or written, the player's settings and records kept from the
			// measuring points' states)
			.nvram_dir = SW_ACTIVE ? NULL : RBX_DATA_DIR "nvram/",
			.cache_dir = RBX_DATA_DIR,   // (large ROM files read from there: the CV1000 NAND)
			.wait_hook = core_wait,
			.jit_alloc = jit_alloc,
			.jit_write_begin = jit_write_begin,
			.jit_write_end = jit_write_end,
		},
		.render_threaded = 1, // drawing on CPU2 (video.c render thread)
	};
	const uint64_t t_load = rbx_time_us();
	if (core_init_shown(&cfg) != 0) {
		RBX_LOG("core: %s init failed: %s", set, rbx_core_error());
		snprintf(s_notice, sizeof(s_notice), T("%s did not start\n%s"), set, rbx_core_error());
		rbx_loading_done();
		return GAME_LIST;
	}
	RBX_LOG("core: %s, init %u ms", rbx_core_game_name(), (unsigned)((rbx_time_us() - t_load) / 1000));
	(void)t_load;
	rbx_loading_set(950);
	rbx_settings_save(); // remember last game (even without opening the menu)
	RBX_LOG("core: cheats %d (" RBX_DATA_DIR "cheats/)", rbx_core_cheats_load(RBX_DATA_DIR "cheats/"));
	if (g_settings.dip_count > 0)
		rbx_core_dip_set_raw(g_settings.dip, g_settings.dip_count);
	rbx_core_set_start_stage(g_settings.start_stage);
	rbx_core_set_infinite_health(g_settings.cheat_health);
	int sw = RBX_SCREEN_W, sh = RBX_SCREEN_H, sflags = 0;
	rbx_core_screen(&sw, &sh, &sflags);
	RBX_LOG("core: board %s, screen %dx%d flags %d", rbx_game_board(rbx_running_game()), sw, sh, sflags);
	RBX_LOG("core: fresh copy %u of %u bytes", (unsigned)rbx_fresh_size(), (unsigned)rbx_fresh_range());
	rbx_present_set_board(rbx_game_board(rbx_running_game()));   // (the frame buffer's Auto: on for the CV1000)
	rbx_video_set_screen(sw, sh, sflags);
	rbx_fps_reset_frames();
	rbx_menu_set_game(set, rbx_core_game_name(), rbx_core_button_count());
#ifdef RBX_ENABLE_LOG
	rbx_menu_bench();
#endif
	rbx_loading_progress(1000);
	rbx_loading_done();

	if (rbx_video_init() != 0) {
		snprintf(s_notice, sizeof(s_notice), T("%s did not start\nvideo init failed"), set);
		rbx_video_fini();
		rbx_core_exit();
		return GAME_LIST;
	}
	// bezel artwork (user artwork/<set>.png, else the parent set, else the app's blank artwork)
	rbx_video_set_art(set, rbx_core_game_parent(game->game));
	rbx_video_set_marquee(set, rbx_core_game_parent(game->game));
	rbx_audio_init();
	rbx_medal_start(); // (medal games: cabinet panels beside the picture)
	s_ym_worker = 0;
	if (cfg.render_threaded && rbx_video_start_render_thread() != 0)
		RBX_LOG("video: continuing without render thread (no video)");
	if (cfg.threaded && rbx_threads_start_sound() != 0)
		RBX_LOG("threads: continuing without sound thread (no sound)");
	if (!cfg.threaded && rbx_threads_start_ym() == 0) // other boards: YM2151 synthesis on CPU1 (same result)
		s_ym_worker = 1;
	rbx_sampler_start();   // (the data folder's "prof" / "profhi": the emulation thread's activity sampled, ADR-155)
	rbx_threads_start_io();   // (a game reading a ROM region from its file: the reads ahead on a worker, ADR-160)
	frame_buffer_apply(&cfg);   // (ADR-195: the present thread, before the first frame)

	const int alen = RBX_RESAMPLE_MAX_IN; // (stereo samples per frame at most: about 800 at 48kHz, several frames' after a late worker)
	int16_t *audio = (int16_t *)calloc((size_t)alen * 2 + 16, sizeof(int16_t));

	uint64_t prev_frame = rbx_time_us();
	uint32_t frame = 0;
	uint64_t emu_frames = 0;
	int menu_open = 0, result = GAME_LIST;
	pace_t pace;
	pace_start(&pace);
	RBX_LOG("pace: display %.3f Hz", pace.disp_mhz / 1000.0);
#ifdef RBX_ENABLE_LOG
	uint64_t tframes = 0; // (switchtest: frames of this game, a state load included)
#endif
	for (;;) {
		uint64_t t0 = rbx_time_us();
		rbx_perf_record(RBX_PERF_FRAME, (uint32_t)(t0 - prev_frame));
		prev_frame = t0;
		rbx_prof_set(RBX_PA_FE_PAD);   // (the frontend's parts of the frame, ADR-161)
		power_tick();
		uint32_t buttons = rbx_input_poll();
		if (!menu_open)
			rbx_medal_touch();

		// options screen: the game is paused while open and the menu is drawn over the last frame
		if (!menu_open && rbx_menu_should_open(buttons | (rbx_touch_menu_take() ? (SCE_CTRL_SELECT | SCE_CTRL_START) : 0))) {
			rbx_present_drain();   // (ADR-195: the frames posted shown; the menu draws on this thread)
			menu_open = 1;
			RBX_LOG("menu: open f%llu", (unsigned long long)emu_frames);
			rbx_fps_hide(); // (it covered the menu, e.g. Done of the touch layout editor)
			rbx_touch_overlay_update(1);
		}
		if (menu_open) {
			rbx_menu_result r = rbx_menu_update(buttons);
			rbx_menu_draw();
			vita2d_start_drawing();
			vita2d_clear_screen();
			if (cfg.render_threaded)
				rbx_video_draw_rendered(emu_frames);
			else
				rbx_video_redraw();
			rbx_video_draw_layers();
			vita2d_end_drawing();
			vita2d_swap_buffers();
			rbx_osd_tick();
			if (r == RBX_MENU_EXIT) {
				result = GAME_EXIT;
				break;
			}
			if (r == RBX_MENU_SELECT_GAME)
				break;
			if (r == RBX_MENU_RESET)
				reset_game(&cfg, &emu_frames);
			if (r == RBX_MENU_SAVE)
				save_state(rbx_menu_state_slot());
			if (r == RBX_MENU_LOAD)
				load_state(&cfg, &emu_frames, rbx_menu_state_slot());
			if (r != RBX_MENU_STAY)
				RBX_LOG("menu: result %d", (int)r);
			if (r == RBX_MENU_CLOSE || r == RBX_MENU_RESET || r == RBX_MENU_LOAD) {
				menu_open = 0;
				rbx_menu_hide();
				rbx_video_apply_settings(); // (rotation, scanlines may have changed)
				frame_buffer_apply(&cfg);   // (Display > Frame buffer may have changed)
			}
			continue;
		}
		// quick save SELECT+L, quick load SELECT+R (once per press)
		{
			static uint32_t prev;
			const uint32_t qs = SCE_CTRL_SELECT | SCE_CTRL_LTRIGGER, ql = SCE_CTRL_SELECT | SCE_CTRL_RTRIGGER;
			if ((buttons & qs) == qs && (prev & qs) != qs)
				save_state(RBX_STATE_QUICK);
			if ((buttons & ql) == ql && (prev & ql) != ql)
				load_state(&cfg, &emu_frames, RBX_STATE_QUICK);
			prev = buttons;
		}
#ifdef RBX_AUTOPLAY
#endif
#ifdef RBX_ENABLE_LOG
		if (s_sw.active && s_sw.frames > 0 && s_sw.warmup > 0) { // (switchtest speed measurement from a saved point)
			static char perf_path[160];
			if (tframes == 0) {
				sceIoMkdir(RBX_DATA_DIR "perf", 0777);
				snprintf(perf_path, sizeof(perf_path), RBX_DATA_DIR "perf/%s.sav", set);
				if (rbx_core_state_exists(perf_path) && with_threads_stopped(&cfg, &emu_frames, perf_path) == 0) {
					RBX_LOG("switchtest: measuring from %s", perf_path);
					tframes = (uint64_t)s_sw.warmup;
				}
			}
			tframes++;
			if (tframes == (uint64_t)s_sw.warmup) {
				int rc = rbx_core_state_save(perf_path);
				RBX_LOG("switchtest: measuring point saved %s rc %d", perf_path, rc);
				RBX_LOG("switchtest: measuring");
			}
			s_play_set = set;
			if (s_sw.play && tframes >= (uint64_t)s_sw.warmup)
				sw_play(tframes - (uint64_t)s_sw.warmup);
			if (tframes >= (uint64_t)(s_sw.warmup + s_sw.frames))
				break;
		} else if (s_sw.active && s_sw.frames > 0) { // (switchtest: a save and a load, then back to the list)
			tframes++;
			if (tframes == (uint64_t)s_sw.frames / 2)
				save_state(RBX_STATE_QUICK);
			if (tframes == (uint64_t)s_sw.frames * 3 / 4)
				load_state(&cfg, &emu_frames, RBX_STATE_QUICK);
			if (tframes >= (uint64_t)s_sw.frames)
				break;
		}
#endif
		rbx_hud_update(buttons);
		rbx_prof_set(RBX_PA_FRONT);
#ifdef RBX_ENABLE_LOG
		const uint64_t t_pre = rbx_time_us();   // (a stall's breakdown: the frame's sections, below)
#endif

		const int present = rbx_present_running();   // (ADR-195: the present thread draws and swaps)
		if (!present)
			rbx_video_kick_border();   // (ADR-181: the corridor's moving parts made on CPU1 while the frame runs)
		const int ran = pace_run(&pace);
#ifdef RBX_ENABLE_LOG
		{	// (the game's frames per second against the display's)
			static uint32_t n, nr;
			static uint64_t t;
			n++;
			nr += ran;
			if (!t)
				t = t0;
			if (n == 600) {
				RBX_LOG("pace: %u game frames in %u display frames, %.2f fps (board %.3f Hz)", nr, n,
					nr * 1e6 / (double)(t0 - t), rbx_core_frame_mhz() / 1000.0);
				n = nr = 0;
				t = t0;
			}
		}
#endif
		if (!ran)
			goto show;   // (ADR-211: no board frame due at this vblank, the last one is shown again)
		int pitch = 0;
		void *fb = cfg.render_threaded ? NULL : rbx_video_target(&pitch);
		{
			const int n = rbx_core_run_frame(fb, pitch, audio, alen);
			rbx_prof_set(RBX_PA_FE_AUDIO);
			rbx_audio_push(audio, n);
			rbx_prof_set(RBX_PA_FRONT);
			{   // (a board that narrows its screen while running: show that part of the buffer)
				int vw, vh, vf;
				rbx_core_screen(&vw, &vh, &vf);
				rbx_video_set_visible(vw, vh);
			}
		}
		emu_frames++;
		{
			const uint32_t w = g_core_wait_us[0];
			g_core_wait_us[0] = 0;
			const uint32_t all = (uint32_t)(rbx_time_us() - t0);
			rbx_perf_record(RBX_PERF_EMU, all > w ? all - w : 0);
			rbx_perf_record(RBX_PERF_SOUND, w);
#ifdef RBX_ENABLE_LOG
			if (w > 8000)   // (a long wait: on what)
				RBX_LOG("wait: f%u %.1f ms: render %.1f aux %.1f aux2 %.1f aux3 %.1f driver %.1f other %.1f", (unsigned)emu_frames,
					w / 1000.0, s_wait_site_us[1] / 1000.0, s_wait_site_us[2] / 1000.0, s_wait_site_us[3] / 1000.0,
					s_wait_site_us[4] / 1000.0, s_wait_site_us[5] / 1000.0, s_wait_site_us[0] / 1000.0);
			{	// (the emulation thread's waits by site, averaged over 300 frames: what the "sound" slot of the perf is)
				static uint64_t sum[6];
				static unsigned nf;
				for (int i = 0; i < 6; i++)
					sum[i] += s_wait_site_us[i];
				if (++nf == 300) {
					RBX_LOG("waits: render %.2f aux %.2f aux2 %.2f aux3 %.2f driver %.2f other %.2f ms/frame",
						sum[1] / 300000.0, sum[2] / 300000.0, sum[3] / 300000.0, sum[4] / 300000.0, sum[5] / 300000.0,
						sum[0] / 300000.0);
					memset(sum, 0, sizeof(sum));
					nf = 0;
				}
			}
#endif
			memset(s_wait_site_us, 0, sizeof(s_wait_site_us));
		}
#if defined(RBX_ENABLE_LOG) && defined(RBX_PROFILE)
		{
			// slow frames (main > 17ms) each log their cause: sound/render wait times
			static uint64_t ws_prev, wr_prev;
			uint64_t ws, wr;
			rbx_core_waits(&ws, &wr);
			uint32_t emu_us = (uint32_t)(rbx_time_us() - t0);
			if (emu_us > 17000)
				RBX_LOG("slow: f%llu emu %.2f wait_sound %.2f wait_render %.2f",
					(unsigned long long)emu_frames, emu_us / 1000.0,
					(ws - ws_prev) / 1000.0, (wr - wr_prev) / 1000.0);
			ws_prev = ws;
			wr_prev = wr;
		}
#endif

		// (the frame emulated: until the vblank this thread has time, so it draws bands of the frame the render
		// thread is drawing, for the boards that split it)
		// (only while the frame left this thread time: after a long emulation the render thread and its helper finish
		// the bands, this thread goes on to the next frame; the bands come out the same whoever draws them)
		if (cfg.render_threaded && rbx_time_us() - t0 < RENDER_HELP_US) {
			rbx_prof_set(RBX_PA_VIDEO_OUT);   // (the sampled activity, ADR-155)
			while (rbx_render_help_one(1))
				;
		}
	show:;
		const uint64_t tf0 = rbx_time_us();
		rbx_prof_set(RBX_PA_DRAW);
		rbx_touch_overlay_update(0);
		// show the previous frame the renderer finished (depth 2, ADR-058: the one before, the renderer may still be
		// drawing the previous one)
		const uint64_t shown = emu_frames >= (uint64_t)rbx_video_depth() ? emu_frames - (uint64_t)rbx_video_depth() : 0;
		if (present) {
			// (ADR-195) the present thread draws and swaps it; this thread waits only while the frame it posted last
			// has not been taken yet (it is then a frame ahead of the display)
			rbx_prof_set(RBX_PA_SWAP);
			rbx_present_post(shown);
		} else {
			vita2d_start_drawing();
			vita2d_clear_screen();
			if (cfg.render_threaded)
				rbx_video_draw_rendered(shown);
			else
				rbx_video_draw();
			rbx_video_draw_layers();
			vita2d_end_drawing();
		}
		const uint64_t tf1 = rbx_time_us();
		rbx_prof_set(RBX_PA_SWAP);
		if (!present)
			vita2d_swap_buffers();
		rbx_prof_set(RBX_PA_FRONT);
#ifdef RBX_ENABLE_LOG
		if (!present) {	// (the frontend's part of the emulation thread's frame: the drawing lists, then the swap's wait)
			static uint64_t sum_d, sum_s;
			static uint32_t max_d, max_s, n;
			const uint64_t tf2 = rbx_time_us();
			const uint32_t d = (uint32_t)(tf1 - tf0), sw = (uint32_t)(tf2 - tf1);
			sum_d += d; sum_s += sw;
			if (d > max_d) max_d = d;
			if (sw > max_s) max_s = sw;
			if (++n == 300) {
				RBX_LOG("front: draw %.2f/%.2f ms, swap %.2f/%.2f ms (avg/max)", sum_d / 300000.0, max_d / 1000.0,
					sum_s / 300000.0, max_s / 1000.0);
				sum_d = sum_s = 0;
				max_d = max_s = n = 0;
			}
		}
#endif
#ifdef RBX_ENABLE_LOG
		const uint64_t t_swap = rbx_time_us();
#endif
		rbx_prof_set(RBX_PA_FE_TICKS);
		rbx_fps_tick(ran, rbx_core_frame_mhz() < pace.disp_mhz ? rbx_core_frame_mhz() / 1000.0f : 60.0f);
		rbx_threads_tick();
		rbx_medal_tick();
		rbx_menu_shot_poll();
		rbx_osd_tick();
		rbx_prof_set(RBX_PA_FRONT);
#ifdef RBX_ENABLE_LOG
		{	// (a frame over 100 ms: where its time went)
			const uint64_t t_end = rbx_time_us();
			if (t_end - t0 > 100000)
				RBX_LOG("stall: f%llu %.1f ms: input %.1f emu %.1f draw %.1f swap %.1f ticks %.1f", (unsigned long long)emu_frames,
					(t_end - t0) / 1000.0, (t_pre - t0) / 1000.0, (tf0 - t_pre) / 1000.0, (tf1 - tf0) / 1000.0,
					(t_swap - tf1) / 1000.0, (t_end - t_swap) / 1000.0);
		}
#endif

#if defined(RBX_ENABLE_LOG) && defined(RBX_PROFILE)
		if ((frame + 1) % 60 == 0) {
			// average ms/frame per section over the last 60 frames
			uint64_t acc[RBX_PROF_COUNT];
			int n = rbx_core_prof_take(acc, RBX_PROF_COUNT);
			char line[256];
			int len = 0;
			for (int i = 0; i < n && len < (int)sizeof(line); i++)
				len += snprintf(line + len, sizeof(line) - len, " %s %.2f",
					rbx_core_prof_name(i), acc[i] / 60.0 / 1000.0);
			RBX_LOG("prof:%s", line);
		}
#endif
#ifdef RBX_ENABLE_LOG
		if (++frame % 300 == 0) {
			rbx_audio_stats as;
			rbx_audio_get_stats(&as);
			RBX_LOG("audio: fill %d underrun %u overflow %u drc %dppm", as.fill, as.underruns, as.overflows, as.drc_ppm);
		}
#else
		(void)frame;
#endif
	}

	// game end: everything this game started stops, then the core and its buffers go (the next game in this process
	// starts from scratch, as a fresh launch would; the harness checks it hashes the same, RBX_PRERUN)
	// (the present thread first, ADR-195: it draws the layers hidden below and may wait for the render thread's frame;
	// the frames posted are shown)
	rbx_present_stop();
	rbx_audio_set_extra_frames(0);
	rbx_present_set_board(NULL);
	rbx_menu_hide();
	rbx_fps_hide();
	rbx_medal_stop();
	rbx_touch_overlay_update(1);
	rbx_sampler_stop();
	rbx_threads_stop_io();
	rbx_threads_stop();
	rbx_video_stop_render_thread();
	s_ym_worker = 0;
	rbx_input_release();
	free(audio);
	rbx_audio_fini();
	rbx_video_fini();
	rbx_core_exit();
	{
		SceKernelFreeMemorySizeInfo fm;
		fm.size = sizeof(fm);
		struct mallinfo mi = mallinfo(); // (newlib heap in use: a leak per game would grow it)
		if (sceKernelGetFreeMemorySize(&fm) < 0)
			fm.size_user = fm.size_cdram = 0;
		RBX_LOG("game end (%s): free user %u KB cdram %u KB, heap in use %u KB", result == GAME_EXIT ? "exit" : "list",
			fm.size_user >> 10, fm.size_cdram >> 10, (unsigned)mi.uordblks >> 10);
		(void)mi;
	}
	return result;
}

#if RBX_TEXT_PAD
// read-only data at the end of the code segment, only to move its end (vita/CMakeLists.txt RBX_TEXT_PAD)
const volatile unsigned char rbx_text_pad[RBX_TEXT_PAD] = { 1 };
#endif

int main(int argc, char *argv[])
{
#if RBX_TEXT_PAD
	(void)rbx_text_pad[0];
#endif
	rbx_log_init();
	rbx_testfiles_start();   // (log builds: the test files looked for off the frame loop, ADR-161)
	rbx_threads_pin_main();
	s_main_tid = sceKernelGetThreadId();
#ifdef RBX_CLOCK_SWEEP
	rbx_clock_sweep();
#endif
	rbx_clock_boost();
	{
		int restarted = 0; // started again by rbx_kctrl_init ("kctrl")
		for (int i = 0; i < argc; i++)
			restarted |= argv[i] && !strcmp(argv[i], "kctrl");
		rbx_kctrl_init(restarted); // (a first start after boot loads the kernel helper and restarts once)
		s_kctrl_restarted = restarted;
		// CPU3 unlocked by the kernel helper on a verified firmware, unless already open (a plugin, another app's unlock)
		const int open = rbx_cpu3_usable();
		rbx_kctrl_cpu3(!open);
		if (!open) {
			rbx_cpu3_recheck();
			if (!rbx_cpu3_usable())   // (logged: CPU3 open or closed, the threads' placement follows it)
				rbx_kctrl_cpu3_undo();
		}
		// it fails when the system has run long with many kernel modules loaded (0x80024302, no free physical page):
		// the PS button then goes straight home, so say why
		if (!rbx_kctrl_ready())
			snprintf(s_notice, sizeof(s_notice), "PS button menu not available (system memory is low)\n"
				"Restart the Vita to use the PS button for the menu");
	}
	rbx_update_before_exit = rbx_ps_release;

	vita2d_init();
	vita2d_set_clear_color(CLEAR_COLOR);
	vita2d_set_vblank_wait(1);
	rbx_video_splash();   // (until the game list's first frame: the start below takes a few seconds)
	rbx_hud_init();
	rbx_input_init();
	// pre-create the folders users fill
	sceIoMkdir(RBX_DATA_DIR, 0777);
	sceIoMkdir(ROM_DIR, 0777);
	sceIoMkdir(RBX_DATA_DIR "nvram", 0777);  // EEPROM / NVRAM of the games
	sceIoMkdir(RBX_DATA_DIR "cheats", 0777);  // user cheat file <set>.ini
	sceIoMkdir(RBX_DATA_DIR "artwork", 0777); // user bezel artwork <set>.png (.jpg .bmp)
	{   // (testing: RBX_DATA_DIR "e12scalar" = the CV1000 blitter without its NEON rows, ADR-105)
		SceIoStat st_e12;
		if (sceIoGetstat(RBX_DATA_DIR "e12scalar", &st_e12) >= 0) {
			setenv("RBX_EPIC12_SCALAR", "1", 1);
			RBX_LOG("epic12: NEON rows off (e12scalar)");
		}
		// (testing, ADR-125: "cps3dmaworker" = the CPS3 character DMA on the worker, not the emulation thread;
		// "cps3newdma" = its branch-light decoder)
		if (sceIoGetstat(RBX_DATA_DIR "cps3dmaworker", &st_e12) >= 0) {   // (the worker DMA is off by default since 11-20)
			setenv("RBX_CPS3_DMA_WORKER", "1", 1);
			RBX_LOG("cps3: character DMA on the worker (cps3dmaworker)");
		}
		// (ADR-130: "sh3jitold" = the CV1000 SH-3 JIT's code as before ADR-130, for an A/B)
		if (sceIoGetstat(RBX_DATA_DIR "sh3jitold", &st_e12) >= 0) {
			setenv("RBX_SH3_JITOPT", "0", 1);
			RBX_LOG("sh3 jit: the code shape before ADR-130 (sh3jitold)");
		}
		// (ADR-165: "spinold" = the SH-3 polling loop skips compare the cache array's and exc_pri's bytes, as before)
		if (sceIoGetstat(RBX_DATA_DIR "spinold", &st_e12) >= 0) {
			setenv("RBX_SH3_SPINOLD", "1", 1);
			RBX_LOG("sh3: polling loop state compares as before ADR-165 (spinold)");
		}
		// (ADR-166: "sh3coldold" = the SH-3 blocks' cold paths and update point exits inline in each block, as before)
		if (sceIoGetstat(RBX_DATA_DIR "sh3coldold", &st_e12) >= 0) {
			setenv("RBX_SH3_SHR", "0", 1);
			RBX_LOG("sh3 jit: cold paths in every block as before ADR-166 (sh3coldold)");
		}
		// (ADR-175: "idleunmap" = the SH-3 blitter polling loop's rounds recorded with the write table taken away, as
		// before, also when the loop is proven to write nothing)
		if (sceIoGetstat(RBX_DATA_DIR "idleunmap", &st_e12) >= 0) {
			setenv("RBX_SH3_IDLEPURE", "0", 1);
			RBX_LOG("sh3: polling loop rounds recorded with the write table away as before ADR-175 (idleunmap)");
		}
		// (ADR-176: "ymznohold" = the YMZ770 worker takes entries up to the frame end as before; "sndstat" = the
		// YMZ770's work and frame-end times in the log at the game's end)
		if (sceIoGetstat(RBX_DATA_DIR "ymznohold", &st_e12) >= 0) {
			setenv("RBX_CV1K_SNDHOLD", "0", 1);
			RBX_LOG("cv1k ymz770: the worker takes entries up to the frame end as before ADR-176 (ymznohold)");
		}
		if (sceIoGetstat(RBX_DATA_DIR "sndstat", &st_e12) >= 0)
			setenv("RBX_CV1K_SNDSTAT", "1", 1);
		// (ADR-177: "nandnodrop" = the NAND reads queued ahead kept at a jump; "nandnomerge" = the io worker reads one
		// request at a time; "nandstat" = the NAND statistics in the log at the game's end)
		if (sceIoGetstat(RBX_DATA_DIR "nandnodrop", &st_e12) >= 0) {
			setenv("RBX_NAND_DROP", "0", 1);
			RBX_LOG("cv1k nand: reads ahead kept at jumps as before ADR-177 (nandnodrop)");
		}
		if (sceIoGetstat(RBX_DATA_DIR "nandnomerge", &st_e12) >= 0) {
			setenv("RBX_IO_MERGE", "0", 1);
			RBX_LOG("io: one request a read as before ADR-177 (nandnomerge)");
		}
		// (ADR-178: "nandbuffered" = the region cache files read through stdio's 1 KB buffer as before)
		if (sceIoGetstat(RBX_DATA_DIR "nandbuffered", &st_e12) >= 0) {
			setenv("RBX_IO_STDBUF", "1", 1);
			RBX_LOG("io: region files buffered as before ADR-178 (nandbuffered)");
		}
		if (sceIoGetstat(RBX_DATA_DIR "nandstat", &st_e12) >= 0)
			setenv("RBX_NAND_STAT", "1", 1);
		if (sceIoGetstat(RBX_DATA_DIR "cps3newdma", &st_e12) >= 0) {
			setenv("RBX_CPS3_NEWDMA", "1", 1);
			RBX_LOG("cps3: branch-light character DMA decoder (cps3newdma)");
		}
		// (ADR-150: "e12shotwait" = the CV1000 vblank waits for the blitter and copies the screen itself, as before,
		// instead of queuing the copy for the lanes)
		if (sceIoGetstat(RBX_DATA_DIR "e12shotwait", &st_e12) >= 0) {
			setenv("RBX_CV1K_SHOTWAIT", "1", 1);
			RBX_LOG("epic12: the vblank waits for the screen's drawing (e12shotwait)");
		}
		// (ADR-170 / 171: "e12oldlist" = the CV1000 blit list read and queued as before; "sh3olddma" = the SH-3 DMA's
		// NAND to RAM copies a byte at a time as before)
		if (sceIoGetstat(RBX_DATA_DIR "e12oldlist", &st_e12) >= 0) {
			setenv("RBX_CV1K_OLDLIST", "1", 1);
			RBX_LOG("epic12: the blit list read as before ADR-170 (e12oldlist)");
		}
		// (ADR-180: "feaudold" = the sound output's resampling and gain as before: the scalar loop, a pair at a time.
		// ADR-181's "neonsync" is read by neon.c)
		if (sceIoGetstat(RBX_DATA_DIR "feaudold", &st_e12) >= 0) {
			rbx_audio_set_old(1);
			RBX_LOG("audio: resampling and gain as before ADR-180 (feaudold)");
		}
		if (sceIoGetstat(RBX_DATA_DIR "sh3olddma", &st_e12) >= 0) {
			setenv("RBX_SH3_OLDDMA", "1", 1);
			RBX_LOG("sh3: the DMA's byte copies as before ADR-171 (sh3olddma)");
		}
		// (ADR-160, the CV1000 NAND: "nandold" = the cache as before (64 chunks, none read ahead), "nandsync" = the
		// chunk cache without reads ahead; neither starts the io worker, threads.c. "nandcpu3": the worker on CPU3)
		if (sceIoGetstat(RBX_DATA_DIR "nandold", &st_e12) >= 0) {
			setenv("RBX_NAND_OLD", "1", 1);
			RBX_LOG("cv1k nand: the cache before ADR-160 (nandold)");
		} else if (sceIoGetstat(RBX_DATA_DIR "nandsync", &st_e12) >= 0) {
			setenv("RBX_NAND_AHEAD", "0", 1);
			RBX_LOG("cv1k nand: no reads ahead (nandsync)");
		}
		// (the CV1000 blitter's lanes, ADR-120: "e12onelane" one lane as ADR-105, "e12split2" / "e12split4" lane 1
		// draws 1 in 2 / 4 row blocks instead of 1 in 3)
		static const char *const sw[3][2] = { { "e12onelane", "0" }, { "e12split2", "2" }, { "e12split4", "4" } };
		char path[128];
		for (int i = 0; i < 3; i++) {
			snprintf(path, sizeof(path), "%s%s", RBX_DATA_DIR, sw[i][0]);
			if (sceIoGetstat(path, &st_e12) >= 0) {
				setenv("RBX_CV1K_SPLIT", sw[i][1], 1);
				RBX_LOG("epic12: split %s (%s)", sw[i][1], sw[i][0]);
			}
		}
		// (ADR-145, how the blitter's NEON rows prefetch: "e12pf0" the next row's first pixels only, as before; "e12pf2"
		// / "e12pf4" the first, middle and last cache lines two / four rows ahead instead of one)
		static const char *const pf[3][2] = { { "e12pf0", "0" }, { "e12pf2", "2" }, { "e12pf4", "4" } };
		for (int i = 0; i < 3; i++) {
			snprintf(path, sizeof(path), "%s%s", RBX_DATA_DIR, pf[i][0]);
			if (sceIoGetstat(path, &st_e12) >= 0) {
				setenv("RBX_EPIC12_PF", pf[i][1], 1);
				RBX_LOG("epic12: prefetch %s (%s)", pf[i][1], pf[i][0]);
			}
		}
	}
	rbx_present_check_files();                 // (testing, ADR-195: "runahead1" / "runahead0" = the frame buffer on / off)
	rbx_settings_move_game_files();            // the games' .cfg files in cfg/
	rbx_states_move_old();                      // the save states in states/
	rbx_touch_init();
	rbx_settings_load();
	rbx_bios_set(g_settings.bios_neogeo);   // (the Neo Geo BIOS chosen in Settings > BIOS)
	rbx_lang_init();   // (the menus' language: its files installed when missing, then read)
	// input engine (setting): RePad = the Bluetooth controller plugin from the app, unless it already runs (ur0:tai, or
	// another Re app); Native = whatever the system has
	// (the first start after boot has just loaded the kernel helper and restarted: the device switched off once loading
	// the RePad kernel plugin right after it. A second between the two kernel loads)
	if (s_kctrl_restarted)
		sceKernelDelayThread(1000000);
	g_settings.input_engine = re_pad_engine_init(g_settings.input_engine, "ux0:app/RBOX00001/module/repad.skprx"); // (the kernel cannot open app0:)
	RBX_LOG("input engine: %s, RePad %s", re_pad_engine_name(g_settings.input_engine), re_pad_plugin() ? "running" : "not running");
	rbx_gamelist_load();   // (the lists' ROM folders join the scan)
	rbx_folders_load();
	rbx_rom_dirs_refresh();
	rbx_menu_init();
	rbx_loading_prepare(); // (the loading screen's background, decoded once now rather than when a game is chosen)
#ifdef RBX_ENABLE_LOG
	switchtest_load();
#endif
	// GitHub releases, on its own thread (the game list offers a newer one once per run)
	rbx_update_check_start();

	for (;;) {
		const rbx_game_found *game = choose_game(s_notice[0] ? s_notice : NULL);
		s_notice[0] = 0;
		if (!game || run_game(game) == GAME_EXIT)
			break;
	}

	RBX_LOG("exit");
	rbx_testfiles_stop();
	rbx_audio_shutdown();
	rbx_menu_fini();
	rbx_hud_fini();
	vita2d_fini();
	rbx_ps_release();
	rbx_log_shutdown();
	sceKernelExitProcess(0);
	return 0;
}
