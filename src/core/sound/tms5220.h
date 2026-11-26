/*  TI TMS5220 / TMS5220C LPC speech synthesizer, ported to C from MAME src/devices/sound/tms5220.cpp, tms5220.h
    (BSD-3-Clause, copyright-holders: Frank Palazzolo, Aaron Giles, Jonathan Gevaryahu, Raphael Nabet, Couriersud,
    Michael Zapf) and its coefficient table from src/devices/sound/tms5110r.hxx (BSD-3-Clause, copyright-holders:
    Frank Palazzolo, Couriersud, Jonathan Gevaryahu).
    This file is part of ReBOX. GPL-2.0-or-later.

    Time base: the chip's own input clock (the clock given to tms5220_init / tms5220_set_clock). One output
    sample is 80 clocks (MAME's stream at clock / 80). The chip keeps its position inside the current sample
    and its pending io_ready event (MAME m_timer_io_ready: 16 clocks after /WS falls or a FIFO-full retry,
    13 us after /RS falls) in clocks, so the caller must:
      - call tms5220_advance(t, clocks, ...) to bring the chip up to the current time before EVERY pin call
        (data_w, wsq_w, rsq_w, status_r, readyq_r, intq_r, set_clock); MAME syncs its stream the same way,
        and the /READY timing of the true-timing (RS/WS) interface only comes out right at clock resolution;
      - convert its CPU time to chip clocks itself, keeping the remainder (Gauntlet: 6502 at 14.318181/8 MHz,
        TMS clock 14.318181/2/(16-d) MHz, so chip clocks = cpu cycles x 4 / (16 - d)); flush up to "now" with
        the old clock before tms5220_set_clock.
    tms5220_render(t, out, n) is the same engine advancing until n more samples are complete.
    Output: MAME's sample as int16 (put_int(.., 32768)): the analog-pin value, -1 while idle.

    Left out: the VSM (speech ROM) interface, M0/M1/ADD/ROMCLK: nothing is attached, so VSM bit reads
    return 1s (a stray Speak ends on a stop frame) as in MAME without a VSM; the combined_rsq_wsq_w hack
    (TI 99/8 only); the CD2501ECD variant. The TMS5200 (CD2501E) is the 5220 with its own pitch and LPC tables
    (tms5110r.hxx T0285_2501E_coeff).
*/
#ifndef RBX_TMS5220_H
#define RBX_TMS5220_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// variants (MAME's TMS5220_IS_* values)
#define TMS5220_VARIANT_5220C 7
#define TMS5220_VARIANT_5200  8
#define TMS5220_VARIANT_5220  9

#define TMS5220_FIFO_SIZE 16

typedef struct {
	// ---- saved (MAME's register_for_save_states, then the timer and the time base) ----
	uint8_t PDC, CTL_pins, state;
	uint8_t next_is_address, schedule_dummy_read, addr_bit, CTL_buffer;
	uint8_t read_byte_register, RDB_flag;
	uint32_t address;
	uint8_t fifo[TMS5220_FIFO_SIZE];
	uint8_t fifo_head, fifo_tail, fifo_count, fifo_bits_taken;
	uint8_t previous_talk_status, SPEN, DDIS, TALK, TALKD, buffer_low, buffer_empty, irq_pin, ready_pin;
	uint8_t command_register, data_latched;
	uint8_t OLDE, OLDP;
	uint8_t new_frame_energy_idx, new_frame_pitch_idx;
	uint8_t new_frame_k_idx[10];
	int16_t current_energy, current_pitch;
	int16_t current_k[10];
	uint16_t previous_energy;
	uint8_t subcycle, subc_reload, PC, IP, inhibit, uv_zpar, zpar, pitch_zero, c_variant_rate;
	uint16_t pitch_count;
	int32_t u[11];
	int32_t x[10];
	uint16_t RNG;
	int16_t excitation_data;
	uint8_t digital_select, io_ready, true_timing, rs_ws, read_latch, write_latch;
	// m_timer_io_ready
	uint8_t io_pending, io_param;
	uint32_t io_clocks;    // clocks until it fires
	uint32_t pos;          // clocks into the current output sample (0..79)
	uint8_t state_end;
	// ---- configuration (not saved) ----
	int variant;
	uint32_t clock;
	void (*irq_cb)(void *ctx, int state);     // MAME irq_cb: called with !irq_pin (active low); may be NULL
	void (*ready_cb)(void *ctx, int state);   // MAME ready_cb: called with !ready_pin (active low); may be NULL
	void *cb_ctx;
	const uint16_t *pitch_tab;      // the variant's tables (m_coeff)
	const int16_t (*k_tab)[32];
} tms5220_t;

void tms5220_init(tms5220_t *t, int variant, uint32_t clock);   // device_start + device_reset
void tms5220_reset(tms5220_t *t);                               // device_reset
void tms5220_set_clock(tms5220_t *t, uint32_t clock);          // set_unscaled_clock (a pending io_ready keeps its time)
uint32_t tms5220_rate(const tms5220_t *t);                      // clock / 80
void tms5220_set_callbacks(tms5220_t *t, void (*irq_cb)(void *, int), void (*ready_cb)(void *, int), void *ctx);

// Run the chip for `clocks` input clocks; each completed sample is written to out[] (up to max; the rest are
// dropped). Returns the number of samples written. io_ready events fire at their exact clock.
int tms5220_advance(tms5220_t *t, uint32_t clocks, int16_t *out, int max);
// Run the chip until n more samples are complete.
void tms5220_render(tms5220_t *t, int16_t *out, int n);

// Pins (advance to the current time first). /RS and /WS are active low; the first rsq_w / wsq_w call switches
// the chip to MAME's "true timing" mode, before that data_w writes straight through.
void tms5220_data_w(tms5220_t *t, uint8_t data);
void tms5220_wsq_w(tms5220_t *t, int state);
void tms5220_rsq_w(tms5220_t *t, int state);
uint8_t tms5220_status_r(tms5220_t *t);
int tms5220_readyq_r(tms5220_t *t);   // /READY: 0 = ready
int tms5220_intq_r(tms5220_t *t);     // /INT: 0 = interrupt

int tms5220_state(tms5220_t *t, void *ctx, int load, const char *tag);

#ifdef __cplusplus
}
#endif

#endif
