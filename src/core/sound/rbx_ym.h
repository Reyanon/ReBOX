/*  Yamaha FM chips (YM2610/YM2610B/YM2203/YM2151/YM3526/YM3812) on ymfm, with a C interface.
    Time advances only through rbx_ym_advance(); timers and the IRQ line follow it.
    Output is mixed to mono with the MAME route gains of the board.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_YM_H
#define RBX_YM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { RBX_YM2610, RBX_YM2610B, RBX_YM2203, RBX_YM2151, RBX_YM3526, RBX_YM3812, RBX_YM3438, RBX_YMF278B, RBX_YM2413 };

typedef struct {
	int type;
	uint32_t clock;
	const uint8_t *adpcma; uint32_t adpcma_size;   // YM2610 sample ROMs (YMF278B: its wave ROM)
	const uint8_t *adpcmb; uint32_t adpcmb_size;
	// route gains (MAME add_route) in 1/256: YM2610 [0] SSG, [1] FM L, [2] FM R; YM2203 [0..2] SSG, [3] FM; YM2151, YM3438 [0] L, [1] R; YM2413 [0] melody, [1] rhythm; YM3526 / YM3812 / YMF278B [0]
	int gain[4];
	void (*irq)(int state);                          // IRQ line changes
	void (*port_a_write)(uint8_t data);              // YM2203 port A (bank switch)
	uint8_t (*port_a_read)(void);                    // YM2203 ports A / B as inputs (DIP switches), may be NULL
	uint8_t (*port_b_read)(void);
	void (*port_b_write)(uint8_t data);              // YM2203 port B as output (may be NULL)
	// YMF278B: the wave ROM offsets of its four 1 MB windows (the board's banks, changed by the board at any time; not
	// with rbx_ym_set_async), NULL = the ROM as one block
	const uint32_t *pcm_bank;
	// the timers as the board's (MAME's emu_timers: ymfm_set_timer, duration in input clocks, < 0 off; the board calls
	// rbx_ym_timer_expired at their time); NULL = here, counted at the samples
	void (*set_timer)(int tnum, int32_t duration);
	// the busy flag as the board's (ymfm_set_busy_end: busy for that many input clocks from now); NULL = never busy
	void (*busy_end)(uint32_t clocks);
	int (*is_busy)(void);
} rbx_ym_config;

int rbx_ym_init(int chip, const rbx_ym_config *cfg);  // chip 0..1
void rbx_ym_exit(int chip);
void rbx_ym_reset(int chip);
uint32_t rbx_ym_rate(int chip);                      // output samples per second
uint8_t rbx_ym_read(int chip, uint32_t offset);
void rbx_ym_timer_expired(int chip, int tnum);       // (rbx_ym_config.set_timer)
void rbx_ym_write(int chip, uint32_t offset, uint8_t data);
// advance by input clocks; produced samples are added (mono, gains applied) to out[] (returns count written,
// out may hold max samples)
int rbx_ym_advance(int chip, uint32_t clocks, int32_t *out, int max);
// the same in stereo (MAME's routes to the two sides): added to l[] and r[]
int rbx_ym_advance_lr(int chip, uint32_t clocks, int32_t *l, int32_t *r, int max);
// the YM2203's outputs apart (raw4[4 n + j]: 0 the FM, 1-3 the SSG channels, unscaled), not on a worker: a board that
// filters them itself (ironhors' switched RC filters)
int rbx_ym_advance_outputs(int chip, uint32_t clocks, int32_t *raw4, int max);
// scale for the board's master volume, 256 = 1.0
void rbx_ym_set_volume(int chip, int vol256);
// change one route gain of rbx_ym_config.gain (1/256) at run time (MAME filter_volume set_gain on a route)
void rbx_ym_set_gain(int chip, int route, int gain256);
int rbx_ym_state(int chip, void *ctx, int load);
// chip 0 / 1: its samples made on the rbx_aux worker (a second chip given the same writes at the same samples):
// rbx_ym_advance then adds nothing to out, rbx_ym_async_take adds the samples made since the last take (n of them).
// Not for the YM2610s (stays as it is, returns 0)
int rbx_ym_set_async(int chip, int on);   // on: 1, or 2 the two sides kept (rbx_ym_advance_lr boards)
void rbx_ym_async_take(int chip, int32_t *dst, int n);
void rbx_ym_async_take_lr(int chip, int32_t *l, int32_t *r, int n);
// a board's own steps on the same worker, in order with the chip's queue (Multi 32's MultiPCM writes and samples):
// fn(a, b, n) runs there for each rbx_ym_async_user (after the chip's steps queued before it). The board syncs
// (rbx_aux_sync) before touching what fn touches. set_user: after rbx_ym_set_async (which clears it), -1 if not async
int rbx_ym_async_set_user(int chip, void (*fn)(uint8_t a, uint16_t b, uint32_t n));
void rbx_ym_async_user(int chip, uint8_t a, uint16_t b, uint32_t n);
// on: a take that has to wait makes steps itself, the other chip's while the worker is on this one (two chips on the
// worker: their frame's last steps on two threads; the same samples). Set by the board after rbx_ym_set_async
void rbx_ym_async_help(int on);

#ifdef __cplusplus
}
#endif

#endif
