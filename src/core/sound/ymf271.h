// license:BSD-3-Clause
// copyright-holders:Olivier Galibert, R. Belmont, hap
/*  Yamaha YMF271-F "OPX" (48 slots: 4-operator FM, PCM), ported from MAME 0.289 src/devices/sound/ymf271.cpp,
    ymf271.h (BSD-3-Clause, copyright-holders: Olivier Galibert, R. Belmont, hap).
    This file is part of ReBOX. GPL-2.0-or-later.

    The chip makes 4 outputs at clock / 384 (44100 Hz at 16.9344 MHz). The owner keeps it in time: it renders the
    samples up to now before a write or a status read (MAME's stream update there) and runs the two timers (the
    timer callback starts one to fire after `clocks` chip clocks, -1 stops it; on firing it calls
    ymf271_timer_expired). The external memory (23-bit: the PCM samples, the ext port's reads and writes) is the
    owner's through read / write.
*/
#ifndef RBX_YMF271_H
#define RBX_YMF271_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define YMF271_SIN_LEN 1024
#define YMF271_LFO_LENGTH 256

typedef struct {
	uint8_t ext_en;
	uint8_t ext_out;
	uint8_t lfoFreq;
	uint8_t lfowave;
	uint8_t pms, ams;
	uint8_t detune;
	uint8_t multiple;
	uint8_t tl;
	uint8_t keyscale;
	uint8_t ar;
	uint8_t decay1rate, decay2rate;
	uint8_t decay1lvl;
	uint8_t relrate;
	uint8_t block;
	uint8_t fns_hi;
	uint32_t fns;
	uint8_t feedback;
	uint8_t waveform;
	uint8_t accon;
	uint8_t algorithm;
	uint8_t ch0_level, ch1_level, ch2_level, ch3_level;

	uint32_t startaddr;
	uint32_t loopaddr;
	uint32_t endaddr;
	uint8_t altloop;
	uint8_t fs;
	uint8_t srcnote, srcb;

	uint32_t step;
	uint64_t stepptr;

	uint8_t active;
	uint8_t bits;

	// envelope generator
	int32_t volume;
	int32_t env_state;
	int32_t env_attack_step;      // volume increase step in attack state
	int32_t env_decay1_step;
	int32_t env_decay2_step;
	int32_t env_release_step;

	int64_t feedback_modulation0;
	int64_t feedback_modulation1;

	int lfo_phase, lfo_step;
	int lfo_amplitude;
	double lfo_phasemod;
} YMF271Slot;

typedef struct {
	uint8_t sync, pfm;
} YMF271Group;

struct ymf271_t;
typedef struct { struct ymf271_t *w; uint32_t qt; int32_t *out; int out_n, g0, g1; } ymf271_wq;   // (g0: its first group, g1: the step between its groups)
typedef struct ymf271_t {
	// ---- state (saved: up to YMF271_STATE_SIZE) ----
	YMF271Slot slots[48];
	YMF271Group groups[12];
	uint8_t regs_main[0x10];
	uint32_t timerA;
	uint32_t timerB;
	uint8_t irqstate;
	uint8_t status;
	uint16_t end_status;
	uint8_t enable;
	uint32_t ext_address;
	uint8_t ext_rw;
	uint8_t ext_readlatch;
	uint32_t master_clock;
	// ---- lookup tables ----
	int16_t lut_waves[8][YMF271_SIN_LEN];
	double lut_plfo[4][8][YMF271_LFO_LENGTH];
	int lut_alfo[4][YMF271_LFO_LENGTH];
	double lut_ar[64];
	double lut_dc[64];
	double lut_lfo[256];
	int lut_attenuation[16];
	int lut_total_level[128];
	int lut_env_volume[256];
	// ---- configuration ----
	uint8_t (*read)(void *user, uint32_t addr);
	void (*write)(void *user, uint32_t addr, uint8_t data);
	void (*irq)(void *user, int state);
	void (*timer)(void *user, int which, int64_t clocks);
	void *user;
	// the external memory's reads straight from a ROM (rom[addr & rom_mask]) instead of read(): ymf271_set_rom
	const uint8_t *rom;
	uint32_t rom_mask, rom_size;   // (mask 0: not a power of two, reads at rom_size or past are 0)
	// (not saved) the step made from the LFO again only when its modulation changed or after a write / reset / load
	uint32_t step_gen, step_gen_slot[48];
	// (not saved) async (ymf271_set_async): the workers' copies (wk the first), the commands for them (one ring, each
	// worker its own read position), the samples each made of its groups
	struct ymf271_t *wk;
	int nwk;
	ymf271_wq wq[2];
	uint32_t *q, qh, qcap;
	int out_cap;
	void (*kick)(void *), (*sync)(void *);
	void *sync_user;
} ymf271_t;

#define YMF271_STATE_SIZE offsetof(ymf271_t, lut_waves)

// allocates the chip (NULL: out of memory); read / write: its external memory, irq may be NULL
ymf271_t *ymf271_create(uint32_t clock, uint8_t (*read)(void *user, uint32_t addr),
	void (*write)(void *user, uint32_t addr, uint8_t data), void (*irq)(void *user, int state),
	void (*timer)(void *user, int which, int64_t clocks), void *user);
void ymf271_destroy(ymf271_t *c);
void ymf271_reset(ymf271_t *c);
void ymf271_write(ymf271_t *c, int offset, uint8_t data);
uint8_t ymf271_read(ymf271_t *c, int offset);
void ymf271_timer_expired(ymf271_t *c, int which);   // 0: timer A, 1: timer B
// after the owner loaded the saved state into the chip
void ymf271_state_loaded(ymf271_t *c);
// before the owner saves the chip's state (async: the worker's slots into it)
void ymf271_state_save_prep(ymf271_t *c);
// the samples made on a worker thread (-1: no memory; read() is then called from the worker too: a ROM only, whose
// contents do not change, and no external writes): kick starts the worker
// on what was queued, sync waits until it did all of it; the worker's job calls ymf271_async_job. ymf271_render then
// writes nothing to mix: ymf271_async_take gives the samples made since the last take (the same values)
// workers 1 or 2 (two: the even groups and the odd ones, each job given which = 0 / 1; kick and sync act on both)
int ymf271_set_async(ymf271_t *c, int workers, void (*kick)(void *), void (*sync)(void *), void *user);
void ymf271_async_job(ymf271_t *c, int which);
int ymf271_async_take(ymf271_t *c, int32_t *mix, int samples);
// the external memory's reads are rom[addr & (size - 1)] for a power-of-two size, else rom[addr] below size and 0
// past it (the owner's read() gives the same), made here without the call; rom NULL: read() again. With async workers
// the owner calls it while they are idle (after a sync, before the next kick)
void ymf271_set_rom(ymf271_t *c, const uint8_t *rom, uint32_t size);
static inline uint32_t ymf271_rate(const ymf271_t *c) { return c->master_clock / 384; }
// `samples` samples of the 4 outputs, interleaved, into mix (4 * samples entries): MAME's mix values, full scale
// 32768 << 2 (a quarter of them for 16-bit samples)
void ymf271_render(ymf271_t *c, int32_t *mix, int samples);

#ifdef __cplusplus
}
#endif

#endif
