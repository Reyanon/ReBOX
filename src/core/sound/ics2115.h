/*  ICS2115 WaveFront synthesizer, ported from MAME src/devices/sound/ics2115.cpp (BSD-3-Clause, copyright-holders:
    Alex Marshall, nimitz, austere).
    The chip is clocked in its own ticks (33.8688 MHz on PGM): the owner advances it with ics2115_advance() up to the
    time of each register access and of the end of each slice, so that samples, voice interrupts and the two timers
    happen where they do in MAME (whose stream is brought up to date on every register access).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_ICS2115_H
#define RBX_ICS2115_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
	struct {
		int32_t left;
		uint32_t acc, start, end;
		uint16_t fc;
		uint8_t ctl, saddr;
	} osc;
	struct {
		int32_t left, add, start, end, acc;
		uint16_t regacc;
		uint8_t incr, pan, mode;
	} vol;
	uint8_t osc_conf, vol_ctrl, on, pad;
	uint16_t regs[0x20];
} ics2115_voice;

typedef struct {
	// state (saved)
	ics2115_voice voice[32];
	struct { uint8_t scale, preset; uint16_t pad; uint64_t period, left; } timer[2];   // period / left: clock ticks
	uint8_t active_osc, osc_select, reg_select, irq_enabled, irq_pending, irq_on, pad[2];
	uint16_t regs[0x40];
	uint32_t tick_frac;        // clock ticks into the current sample
	// output: samples at the chip's rate (clock / ((active_osc + 1) * 32)), mono (MAME's both outputs summed)
	int32_t *out;
	int out_n, out_cap;
	// configuration
	const uint8_t *rom;
	uint32_t rom_mask;         // (rom size - 1 for a power of two, else the largest power of two below - 1)
	uint32_t rom_size;
	void (*irq_cb)(int state);
} ics2115_t;

#define ICS2115_STATE_SIZE ((uint32_t)offsetof(ics2115_t, out))

int ics2115_init(ics2115_t *c, const uint8_t *rom, uint32_t rom_size, void (*irq_cb)(int), int out_cap);
void ics2115_exit(ics2115_t *c);
void ics2115_reset(ics2115_t *c);
// run the chip for `ticks` clock ticks: samples into out[], timers and voice interrupts on the way
void ics2115_advance(ics2115_t *c, uint32_t ticks);
// clock ticks until the next timer interrupt (UINT64_MAX: none)
uint64_t ics2115_next_event(const ics2115_t *c);
uint8_t ics2115_read(ics2115_t *c, int offset);
void ics2115_write(ics2115_t *c, int offset, uint8_t data);
static inline uint32_t ics2115_sample_ticks(const ics2115_t *c) { return (uint32_t)(c->active_osc + 1) * 32; }

#endif
