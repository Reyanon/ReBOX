/*  ReBOX core internals: game tables, ROM regions, input ports, video and audio services
    shared by the board drivers. The tables follow the MAME driver model (regions, ioports,
    DIP switches) so that drivers can be carried over from MAME with their structure intact.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_H
#define RBX_H

#include <stdint.h>
#include <stddef.h>
#include "rbx_prof.h"   // (the sampled activity marks, ADR-155)

#ifdef __cplusplus
extern "C" {
#endif

// ---- ROM layout (generated from MAME ROM_START blocks by tools/gen_games.py) ----
enum { RBX_ROM_END, RBX_ROM_LOAD, RBX_ROM_CONTINUE, RBX_ROM_RELOAD, RBX_ROM_FILL, RBX_ROM_COPY };
typedef struct {
	uint8_t op, region;
	const char *file;
	uint32_t offset, length, crc;
	uint8_t group, skip, reverse;   // ROMX_LOAD layout: bytes per group, bytes skipped after a group, byte order reversed
	                                // (reverse 2 / 3: ROM_NIBBLE, ROM_SHIFT_NIBBLE_LO / HI)
	uint32_t value;                 // FILL value, COPY (source region << 24 | source offset)
} rbx_rom_load;
#define RBX_REGION_INVERT 0x01      // ROMREGION_INVERT: the region's bytes inverted once its loads are done
typedef struct { const char *name; uint32_t size; uint8_t fill, flags; } rbx_rom_region;

// ---- input ports (MAME ioport model) ----
enum {
	RBX_IPT_UNUSED, RBX_IPT_UNKNOWN, RBX_IPT_OTHER, RBX_IPT_CUSTOM, RBX_IPT_OUTPUT,
	RBX_IPT_DIPSWITCH, RBX_IPT_CONFIG,
	RBX_IPT_SERVICE, RBX_IPT_SERVICE1, RBX_IPT_SERVICE2, RBX_IPT_SERVICE3, RBX_IPT_SERVICE4, RBX_IPT_TILT,
	RBX_IPT_COIN1, RBX_IPT_COIN2, RBX_IPT_COIN3, RBX_IPT_COIN4,
	RBX_IPT_START1, RBX_IPT_START2, RBX_IPT_START3, RBX_IPT_START4,
	RBX_IPT_JOYSTICK_UP, RBX_IPT_JOYSTICK_DOWN, RBX_IPT_JOYSTICK_LEFT, RBX_IPT_JOYSTICK_RIGHT,
	RBX_IPT_BUTTON1, RBX_IPT_BUTTON2, RBX_IPT_BUTTON3, RBX_IPT_BUTTON4, RBX_IPT_BUTTON5, RBX_IPT_BUTTON6,
	RBX_IPT_TRACKBALL_X, RBX_IPT_TRACKBALL_Y, RBX_IPT_DIAL, RBX_IPT_PADDLE, RBX_IPT_AD_STICK_X, RBX_IPT_AD_STICK_Y,
	RBX_IPT_AD_STICK_Z, RBX_IPT_PEDAL, RBX_IPT_PEDAL2, RBX_IPT_PEDAL3, RBX_IPT_LIGHTGUN_X, RBX_IPT_LIGHTGUN_Y,
	RBX_IPT_POSITIONAL, RBX_IPT_POSITIONAL_V, RBX_IPT_BUTTON7, RBX_IPT_BUTTON8,
	RBX_IPT_JOYSTICKRIGHT_UP, RBX_IPT_JOYSTICKRIGHT_DOWN, RBX_IPT_JOYSTICKRIGHT_LEFT, RBX_IPT_JOYSTICKRIGHT_RIGHT,
	// (added for the Konami module's tables: listed games of boards not ported yet use them; appended, the values above
	// unchanged)
	RBX_IPT_BUTTON9, RBX_IPT_BUTTON10, RBX_IPT_BUTTON11, RBX_IPT_BUTTON12, RBX_IPT_BUTTON13, RBX_IPT_BUTTON14,
	RBX_IPT_BUTTON15, RBX_IPT_BUTTON16, RBX_IPT_START5, RBX_IPT_START6, RBX_IPT_START7, RBX_IPT_START8,
	RBX_IPT_COIN5, RBX_IPT_COIN6, RBX_IPT_COIN7, RBX_IPT_COIN8, RBX_IPT_DOOR,
	RBX_IPT_GAMBLE_HALF, RBX_IPT_GAMBLE_HIGH, RBX_IPT_GAMBLE_LOW, RBX_IPT_GAMBLE_KEYIN, RBX_IPT_GAMBLE_KEYOUT,
	RBX_IPT_MOUSE_X, RBX_IPT_MOUSE_Y, RBX_IPT_PADDLE_V,
	RBX_IPT_COUNT
};
typedef struct { uint32_t value; const char *name; } rbx_ipt_setting;
typedef struct {
	uint32_t mask, def;
	uint16_t type;
	uint8_t player, impulse, toggle;
	const char *name;               // DIP / config name
	uint16_t set_first, set_count;  // settings (rbx_ipt_setting) of a DIP field
	uint8_t cond_port, cond_type;   // PORT_CONDITION: port index (255 = none), 1 = equals, 2 = not equals
	uint32_t cond_mask, cond_value;
} rbx_ipt_field;
typedef struct { const char *tag; uint16_t first, count; } rbx_ipt_port;

#define RBX_GF_NOT_WORKING   1
#define RBX_GF_MECHANICAL    2
#define RBX_GF_IMPERFECT_GFX 4
#define RBX_GF_IMPERFECT_SND 8
#define RBX_GF_UNSUPPORTED   16   // machine configuration not emulated (set by the core)

typedef struct {
	const char *set, *parent, *title, *year, *maker, *board, *machine;
	int rot;
	int flags;
	const rbx_rom_region *regions;
	const rbx_rom_load *loads;
	const rbx_ipt_port *ports;
	const rbx_ipt_field *fields;
	const rbx_ipt_setting *settings;
} rbx_game;

// ---- board driver ----
typedef struct {
	const char *board;                    // rbx_game.board
	int (*init)(const rbx_game *g);       // ROM regions are loaded; 0 = ok
	void (*exit)(void);
	void (*reset)(void);
	void (*run_frame)(void);              // one video frame: renders rbx_video, produces sound into rbx_audio
	int (*state_save)(void *ctx);         // through rbx_state_* (may be NULL)
	int (*state_load)(void *ctx);
	int (*supported)(const char *machine);   // machine configuration emulated
	void (*render)(void);                    // draw the last captured frame into rbx_video (any thread, see rbx_video_frame_done)
	// optional: called by a thread waiting for render (rbx_video_wait_idle) to do part of it; 0 = nothing to do now
	int (*render_help)(int helper);   // helper 1: the emulation thread, 2: a frontend thread (rbx_render_help)
	// optional: regions the driver reads with rbx_region_read instead of rbx_region (NULL-terminated): kept in a file
	// (rbx_config.cache_dir) rather than in memory when the region is one whole ROM file
	const char *const *file_regions;
	// optional (ADR-200): the sizes of the driver's largest buffers (up to RBX_EARLY_MAX, largest first), made zeroed by
	// the core before the ROM regions are loaded, so that they take the lowest holes of a fixed-size heap (newlib's,
	// the Vita) before the regions and the loader's buffers do; returns their count. The driver takes them in init with
	// rbx_early; the core frees them after the driver's exit (and when the load fails)
	int (*early)(const rbx_game *g, size_t *sizes);
	// optional (ADR-222): regions kept as blocks (NULL-terminated), read with rbx_region_blocks instead of rbx_region:
	// regions made only of word-interleaved pairs (ROM_LOAD32_WORD: group 2, skip 2), kept as 8 KB chunks each stored
	// once (chunks with the same bytes shared), so a sprite ROM full of blank or repeated pages takes its distinct ones
	const char *const *block_regions;
} rbx_driver;
#define RBX_EARLY_MAX 4
void *rbx_early(int i, size_t *size);   // block i of rbx_driver.early (NULL: none)

// ---- services for drivers ----
void rbx_log(const char *fmt, ...);
uint64_t rbx_clock_us(void);   // the frontend's clock (0 without one): profiling logs only
// executable memory from the frontend (rbx_config jit_*), `size` bytes of one block for the app's life, given out once
// (only appended to, never rewritten): NULL = none (the driver interprets); the emulation thread
void *rbx_jit_alloc(unsigned size);
void rbx_jit_write_begin(void);
// a large optional buffer outside the heap (rbx_config.big_alloc: memory blocks, then CDRAM; malloc without one);
// *big says how it was taken, for rbx_big_free. NULL: none (the driver does without)
void *rbx_big_alloc(size_t size, int *big);
void rbx_big_free(void *p, int big);
void rbx_jit_write_end(void *code, unsigned size);
void rbx_fail(const char *fmt, ...);      // set the error text returned by rbx_error()

// ROM folders of the running configuration (ending with a separator), NULL past the last; for sample sets next to them
const char *rbx_rom_dir(int i);

// ROM region by MAME name ("maincpu", "audiocpu", "ymsnd:adpcma" ...). NULL if the set has none
uint8_t *rbx_region(const char *name, uint32_t *size);
// the same for a region the driver can do without: NULL when missing, without an error
uint8_t *rbx_region_opt(const char *name, uint32_t *size);
// a region of rbx_driver.file_regions: its size (0 = missing, reported as rbx_region does) and its bytes
// [offset, offset + len) (0 = ok). The file's reads are slow next to memory: the driver keeps its own cache
uint32_t rbx_region_file(const char *name);
int rbx_region_read(const char *name, uint32_t offset, void *dst, uint32_t len);
// 1: the file region was loaded with ROM_LOAD16_WORD_SWAP, and rbx_region_read gives the file's bytes (the driver
// swaps each pair: byte offset ^ 1 in the region = the file's byte)
int rbx_region_file_swap16(const char *name);
// a region of rbx_driver.block_regions (ADR-222): its 8 KB chunks, byte a of the region = chunk[a >> 13][a & 0x1fff]
// (chunks with the same bytes are one, the region's fill included). NULL = missing (reported as rbx_region does)
#define RBX_BLK_SHIFT 13
#define RBX_BLK_MASK 0x1fffu
uint8_t **rbx_region_blocks(const char *name, uint32_t *size);
// each distinct chunk of the region once (to decode it in place: a decoding that depends on the place in the chunk only)
void rbx_region_blocks_each(const char *name, void (*fn)(uint8_t *chunk, void *user), void *user);
// reads of a file region ahead, on the frontend's io worker thread (ADR-160): rbx_io_submit queues one (its id, or -1:
// no worker, or no room: the driver reads it itself when needed). rbx_io_finish = the bytes in place (0) or not readable
// (-1): taken back and read here when the worker has not started it, waited for when it has; the id is free after.
// rbx_io_ready: done by the worker (finish will not wait); rbx_io_queued: not started; rbx_io_drop: no longer needed.
// The emulation thread only; every id submitted is finished or dropped before the driver's exit returns
int rbx_io_on(void);
// a file written whole (an NVRAM, a memory card) off the emulation thread when the io worker runs (ADR-224: an SD card
// write stops a frame for 100+ ms), else here; the bytes are copied. The writes of one path land in the order given.
// rbx_file_put_now: here, after the queued ones (the game's exit). 0 = queued or written
int rbx_file_put(const char *path, const void *data, uint32_t len);
int rbx_file_put_now(const char *path, const void *data, uint32_t len);
int rbx_io_submit(const char *name, uint32_t offset, void *dst, uint32_t len);
int rbx_io_ready(int id);
int rbx_io_queued(int id);
int rbx_io_finish(int id);
void rbx_io_drop(int id);
// (ADR-177) not needed any more if the worker has not started it (1: the id is free), else 0 (left as it is)
int rbx_io_cancel(int id);
void rbx_io_stat(uint64_t *worker, uint64_t *taken, uint64_t *waits, uint64_t *wait_us);   // (since the game started)
uint64_t rbx_io_nreads(void);   // (the worker's card reads: requests merged into one read count once, ADR-177)

// input ports: index by tag (-1 = none), current value (inputs, DIPs and defaults combined)
int rbx_port_find(const char *tag);
// a game whose dial aims while its joystick moves (Forgotten Worlds): the right stick's left / right turn it (set at
// init; the frontend then offers the "second stick" keys, RBX_IN_RLEFT / RBX_IN_RRIGHT)
void rbx_set_rstick_dial(int on);
uint32_t rbx_port_read(int port);
// set the IPT_CUSTOM bits of a port as line levels, 1 = asserted (EEPROM data line, sensors); active-low fields are inverted
void rbx_port_custom(int port, uint32_t mask, uint32_t value);
// analog input of a player (trackball delta per frame etc.), type RBX_IPT_TRACKBALL_X ...
int rbx_analog(int player, int type);
// rbx_analog, or without that device moving the d-pad's: step counts a frame (left / right for x axes and dials, up /
// down for y axes), as MAME's key delta x sensitivity / 100 (games with only a dial / trackball play on the d-pad)
int rbx_analog_dpad(int player, int type, int step);
// raw control bits of a player (RBX_IN_* of rbx_core.h), for digital fallbacks of analog controls (steering, pedals)
uint32_t rbx_player_input(int player);
// raw controls of a player (1-4) as RBX_IN_* bits of rbx_core.h (pedals and other inputs the ioport model lacks);
// rbx_input is the same (analog-only boards steer with the d-pad without a device)
uint32_t rbx_input_bits(int player);
uint32_t rbx_input(int player);

// video: the driver draws palette indices into a 16-bit bitmap and keeps the palette in RGB888
typedef struct {
	uint16_t *bitmap;     // indexed pixels, width x height, pitch in pixels
	int pitch;
	int width, height;    // visible area
	uint32_t *palette;    // 0x00RRGGBB
	int palette_size;
	int blank;            // 1 = draw nothing (screen off)
	int palette_used;     // entries the frame uses (0 = palette_size); set by render()
	// the frontend's frame while render() runs (NULL when there is none): a driver may write it itself, line by line in
	// its color format (out_color: 0x00RRGGBB to a pixel of out_bpp bytes), and set out_done; else the core converts
	// the bitmap afterwards
	void *out;
	int out_pitch, out_bpp;   // bytes
	uint32_t (*out_color)(int r, int g, int b);
	int out_done;
} rbx_video_t;
extern rbx_video_t rbx_video;
int rbx_video_setup(int width, int height, int palette_size);
// frame drawing in two halves: at vblank the driver waits for the previous drawing (rbx_video_wait_idle), copies what
// drawing needs (its snapshot) and calls rbx_video_frame_done; driver.render then draws from the snapshot, right away or
// on the render thread. The driver also waits before touching state the drawing side writes (sprite frame buffer)
void rbx_video_wait_idle(void);
void rbx_video_frame_done(void);
// two snapshots (core.c): set at init, the vblank waits with rbx_video_wait_slot instead of rbx_video_wait_idle, takes
// slot rbx_video_post_seq() & 1; render draws slot rbx_video_render_seq() & 1
void rbx_video_set_depth(int depth);
void rbx_video_wait_slot(void);
unsigned rbx_video_post_seq(void);
unsigned rbx_video_render_seq(void);
int rbx_video_threaded(void);   // (1: render runs on the render thread, not within rbx_video_frame_done)
// a driver job that may run on a worker thread (the F3 ES5510 effects DSP): the job does the work queued so far.
// rbx_aux_kick() after queuing (the worker starts on it), rbx_aux_sync() waits until the job has done all that was
// queued before the call. Without a worker thread both run the job themselves: the results are the same.
// The job only touches its own queue and the state the driver gives it; the driver syncs before touching that state
void rbx_aux_set_job(void (*job)(void));   // NULL = none (at exit; syncs first)
void rbx_aux_kick(void);
void rbx_aux_sync(void);
// the first worker waits for its next job by spinning this many microseconds after a job before it sleeps (jobs every
// scanline: PGM's sound CPU); 0 = sleep at once. Cleared with the job
void rbx_aux_set_spin(int us);
// the first job's thread beside the render thread (CPU2 on the Vita, below the render thread's priority: a board whose
// drawing is light, whose CPU1 is crowded; set before the game starts, cleared with the job)
void rbx_aux_set_by_render(int on);
// a second job on its own worker thread, the same way (the F3 ES5505 voices), and a third (the F3 sound board)
void rbx_aux2_set_job(void (*job)(void));
void rbx_aux2_kick(void);
void rbx_aux2_sync(void);
// the second job's thread on a core of its own (1: CPU3 on the Vita, a board without a drawing helper;
// RBX_AUX2_BY_RENDER: CPU2 on the Vita below the render thread, a board whose drawing is light; set before the game
// starts, cleared with the job)
#ifndef RBX_AUX2_BY_RENDER   // (also in rbx_core.h)
#define RBX_AUX2_BY_RENDER 2
#endif
void rbx_aux2_set_alone(int on);
void rbx_aux2_set_spin(int us);   // (as rbx_aux_set_spin, for the second worker; cleared with the job)
void rbx_aux3_set_job(void (*job)(void));
void rbx_aux3_kick(void);
void rbx_aux3_nudge(void);   // rbx_aux3_kick with its thread only (without, nothing)
void rbx_aux3_sync(void);
// the third job's thread alone on CPU3 at a low priority on the Vita (the CV1000 blitter's second lane: work the
// others take over when it falls behind); set before the game starts, cleared with the job
void rbx_aux3_set_alone(int on);
// a moment's wait (the frontend's: a sleep) on the emulation thread for worker w (0..2: rbx_aux, aux2, aux3; profiled
// as that worker's wait), and on a worker thread
void rbx_aux_wait_for(int w);
void rbx_worker_wait(void);
// a short wait (the frontend's: a sleep) while another thread works
void rbx_wait(void);
// one piece of the frame being drawn by a board that splits its drawing (helper 1: the emulation thread, 2: the
// frontend's helper core); 0 = nothing to draw now (render_help with 0x100: F3V_HELP_ONE)
int rbx_render_help_one(int helper);
// the first worker draws bands (render_help(helper | 0x100)) while it has no job; 0 = off
void rbx_aux_set_idle_help(int helper);

// audio: the driver mixes its chips into a mono stream at its own rate (rbx_audio_setup), the core resamples
typedef struct {
	int16_t *buf;          // samples of the current frame at the driver rate
	int count, cap;
	uint32_t rate;
	int stereo;            // rbx_audio_setup_stereo: buf = left, buf_r = right
	uint32_t frame_mhz;    // the board's frame rate in milli-Hz (0 = 60 Hz): a frame's sound is played in a 60 Hz frame
	int16_t *buf_r;
} rbx_audio_t;
extern rbx_audio_t rbx_audio;
int rbx_audio_setup(uint32_t rate);
int rbx_audio_setup_stereo(uint32_t rate);
static inline void rbx_audio_put(int32_t s)
{
	if (rbx_audio.count < rbx_audio.cap) {
		if (s > 32767) s = 32767;
		if (s < -32768) s = -32768;
		rbx_audio.buf[rbx_audio.count++] = (int16_t)s;
	}
}
static inline void rbx_audio_put_stereo(int32_t l, int32_t r)
{
	if (rbx_audio.count < rbx_audio.cap) {
		if (l > 32767) l = 32767;
		if (l < -32768) l = -32768;
		if (r > 32767) r = 32767;
		if (r < -32768) r = -32768;
		rbx_audio.buf[rbx_audio.count] = (int16_t)l;
		rbx_audio.buf_r[rbx_audio.count++] = (int16_t)r;
	}
}

// save state helpers: a flat buffer of tagged blocks
int rbx_state_io(void *ctx, const char *tag, void *data, uint32_t size);
// a block whose size the driver chooses (packed data): saving, *size = the bytes it will write and the result is
// where to write them (NULL in the sizing pass or on an error); loading, the block's bytes with *size set (NULL = error)
void *rbx_state_var(void *ctx, const char *tag, uint32_t *size, int load);
// the load's check walk (the save walk over a loaded state, headers compared only): a block's packed size need not be
// computed then (rbx_state_var reads it from the file)
int rbx_state_checking(void *ctx);

// clocks of a CPU for the next scanline at a frame rate in milli-Hz (fractions carried in *frac)
// floor(num / den) without a 64-bit division (a libgcc call of a few hundred cycles on ARM): a double estimate
// corrected to the exact quotient. Same result as num / den for den > 0 and a quotient below 2^32.
static inline uint32_t rbx_div64_small(uint64_t num, uint64_t den)
{
	// (32-bit halves: the conversions are single VFP instructions on ARM, not libgcc calls)
	const double fn = (double)(uint32_t)(num >> 32) * 4294967296.0 + (double)(uint32_t)num;
	const double fd = (double)(uint32_t)(den >> 32) * 4294967296.0 + (double)(uint32_t)den;
	const double q = fn / fd;
	uint64_t n = q < 4294967295.0 ? (uint32_t)q : 0xffffffffu;
	while (n && n * den > num)
		n--;
	while ((n + 1) * den <= num)
		n++;
	return (uint32_t)n;
}

// (a * b) / c with a 32-bit division when the product fits (exactly (uint64_t)a * b / c)
// v % n (0 when n is 0) with the division only past the end: a tile code against a decoded set's count
// the set bits of v (__builtin_popcount is a libgcc call on ARMv7 without a count instruction)
static inline uint32_t rbx_popcount(uint32_t v)
{
#if defined(__arm__)
	v = v - ((v >> 1) & 0x55555555u);
	v = (v & 0x33333333u) + ((v >> 2) & 0x33333333u);
	return (((v + (v >> 4)) & 0x0f0f0f0fu) * 0x01010101u) >> 24;
#else
	return (uint32_t)__builtin_popcount(v);
#endif
}
static inline uint32_t rbx_wrap(uint32_t v, uint32_t n) { return v < n ? v : n ? v % n : 0; }
static inline uint32_t rbx_muldiv(uint32_t a, uint32_t b, uint32_t c)
{
	uint64_t p = (uint64_t)a * b;
	return (p >> 32) ? (uint32_t)(p / c) : (uint32_t)p / c;
}

static inline int rbx_line_cycles(uint64_t *frac, uint32_t clock, uint32_t rate_mhz, int lines)
{
	uint64_t den = (uint64_t)rate_mhz * (uint64_t)lines;
	*frac += (uint64_t)clock * 1000;
	uint64_t n = rbx_div64_small(*frac, den);
	*frac -= n * den;
	return (int)n;
}

static inline uint16_t rbx_be16(const uint8_t *p) { return (uint16_t)(p[0] << 8 | p[1]); }

#ifdef __cplusplus
}
#endif

#endif
