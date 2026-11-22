/*  Atari C012294 POKEY, ported to C from MAME src/devices/sound/pokey.cpp, pokey.h (BSD-3-Clause,
    copyright-holders: Brad Oliver, Eric Smith, Juergen Buchmueller). POKEY chip emulator 4.9.
    This file is part of ReBOX. GPL-2.0-or-later.

    Output: MAME's default output (LEGACY_LINEAR, used when POKEY(config, ...) has no set_output_* call):
    min(sum of the four 4-bit channel volumes x (32767 / 11 / 4), 0x7fff), a unipolar 0..32767 sample
    (MAME's stream value x 32768). MAME streams it at the chip clock; here the per-clock output is box-averaged
    over POKEY_DIVIDER clocks, giving pokey_rate() = clock / 28 (63.92 kHz at 1.79 MHz).

    Time base: the chip's own input clock (one POKEY clock = one MAME step_one_clock). The caller owns the
    conversion from its CPU cycles (Atari arcade boards clock the POKEY with the 6502: 1 cycle = 1 clock).
    MAME syncs the chip before every register access, so the caller must bring the chip up to "now" with
    pokey_advance() before pokey_read() / pokey_write(); the access then takes effect at that clock.

    The skipping engine: clocks in which no counter wraps, no borrow fires and the output is unchanged are
    advanced arithmetically (polynomial indices, prescalers, counters, pots, keyboard scan counter); only
    the clocks where something happens run the full MAME step_one_clock. The result is identical to stepping
    every clock (build with -DPOKEY_REFERENCE to step every clock, for comparison).

    Left out: the keyboard callback (no key is ever pressed: the scan counter runs, KBCODE stays 0x09);
    the SOD / OCLK output callbacks (the serial shift registers and their IRQs are kept, the serial input
    pin is pokey_sid_w; bclk_w is a no-op in MAME too); the RC / op-amp / discrete output types; the ALLPOT
    callback. MAME defers writes and timer IRQs to a scheduler sync at the current time; here they apply at
    once, at the clock the caller advanced to.
*/
#ifndef RBX_POKEY_H
#define RBX_POKEY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define POKEY_DIVIDER 28   // clocks averaged into one output sample (= the 64 kHz prescaler, DIV_64)

typedef struct {
	// ---- saved (MAME's save_item list, then the output / decimation state) ----
	int32_t borrow_cnt[4];
	int32_t counter[4];
	uint8_t filter_sample[4];
	uint8_t output[4];
	uint8_t AUDF[4];
	uint8_t AUDC[4];
	int32_t clock_cnt[3];
	uint32_t p4, p5, p9, p17;
	uint8_t POTx[8];
	uint8_t AUDCTL, ALLPOT, KBCODE, SERIN, SEROUT, IRQST, IRQEN, SKSTAT, SKCTL;
	uint8_t pot_counter, kbd_cnt, kbd_latch, kbd_state;
	uint8_t serout_full, ser_iclk, ser_oclk, sod_twotone;
	uint16_t serin_shift, serout_shift;
	uint32_t out_raw;      // (not saved by MAME; kept here so a load is exact)
	uint8_t old_raw_inval;
	uint8_t irq_line;      // the last level given to irq_cb
	uint8_t pad_[2];
	int32_t level;         // LEGACY_LINEAR output of out_raw
	uint32_t sub;          // clocks already summed into the current output sample
	uint32_t acc;          // their sum
	uint32_t ev;           // clocks until the next clock with an event (1 = the next clock)
	uint8_t state_end;
	// ---- configuration (kept over reset; not saved) ----
	uint32_t clock;
	uint8_t pot_set;       // bit n: POTn is connected (pokey_set_pot)
	uint8_t pot_in[8];
	void (*irq_cb)(void *ctx, int state);   // MAME irq_w (ASSERT_LINE = 1 / CLEAR_LINE = 0); may be NULL
	void *irq_ctx;
} pokey_t;

// device_start + device_reset: every field to its start value (a fresh chip). The poly tables are shared,
// built once (16 KB for the 17-bit polynomial).
void pokey_init(pokey_t *p, uint32_t clock);
// MAME device_reset (a machine reset): only POTGO is done, as in MAME
void pokey_reset(pokey_t *p);
uint32_t pokey_rate(const pokey_t *p);   // clock / POKEY_DIVIDER (integer part)

// Run the chip for `clocks` POKEY clocks; each completed output sample is written to out[] (up to max; the
// rest are dropped). Returns the number of samples written. A sample in progress is carried to the next call.
int pokey_advance(pokey_t *p, uint32_t clocks, int16_t *out, int max);
// Run the chip until n more output samples are complete (whole-sample granularity).
void pokey_render(pokey_t *p, int16_t *out, int n);

// Register access at the current chip time (advance first). offset & 15.
uint8_t pokey_read(pokey_t *p, uint8_t offset);
void pokey_write(pokey_t *p, uint8_t offset, uint8_t data);

// Optional inputs / outputs
void pokey_set_pot(pokey_t *p, int n, uint8_t value);   // MAME pot_r<n> (read at POTGO); n = 0..7
void pokey_sid_w(pokey_t *p, int state);                // serial input pin 24
void pokey_set_irq_cb(pokey_t *p, void (*cb)(void *ctx, int state), void *ctx);

int pokey_state(pokey_t *p, void *ctx, int load, const char *tag);

#ifdef __cplusplus
}
#endif

#endif
