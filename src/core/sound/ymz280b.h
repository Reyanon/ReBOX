/*  Yamaha YMZ280B PCMD8 (8-voice ADPCM / PCM8 / PCM16 player), ported from MAME 0.289
    src/devices/sound/ymz280b.cpp / ymz280b.h (BSD-3-Clause, copyright-holders: Aaron Giles).
    The chip runs at MAME's internal stream rate, clock / 192 (MAME: m_master_clock * 2, with
    m_master_clock = clock / 384), and renders 2 channels (left, right) at that rate.
    End-of-sample IRQs: MAME latches the status bit from a zero-time timer that fires after the stream
    update; here the latch happens at the end of the ymz280b_render() call that ended the sample.
    Save state: the struct from the start up to YMZ280B_STATE_SIZE (no pointers inside).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_YMZ280B_H
#define RBX_YMZ280B_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint8_t playing;         // 1 while actively playing
	uint8_t ended;           // the voice hit its stop address in this update
	uint8_t keyon;
	uint8_t looping;
	uint8_t mode;            // 0 none, 1 ADPCM, 2 PCM8, 3 PCM16
	uint8_t level;
	uint8_t pan;
	uint8_t irq_schedule;    // 1 if the end-of-sample IRQ waits to be latched
	uint16_t fnum;
	int16_t last_sample, curr_sample;
	uint32_t start, stop, loop_start, loop_end, position;   // in nibbles
	int32_t signal, step;    // ADPCM state
	int32_t loop_signal, loop_step;
	uint32_t loop_count;
	int32_t output_left, output_right;
	int32_t output_step;     // fnum + 1, in 1/512 input samples per output sample
	int32_t output_pos;      // fractional position (FRAC_BITS = 9)
} ymz280b_voice_t;

typedef struct {
	// ---- state (saved) ----
	ymz280b_voice_t v[8];
	uint8_t current_register;
	uint8_t status_register;
	uint8_t irq_state;
	uint8_t irq_mask;
	uint8_t irq_enable;
	uint8_t keyon_enable;
	uint8_t ext_mem_enable;
	uint8_t ext_readlatch;
	uint32_t ext_mem_address_hi;
	uint32_t ext_mem_address_mid;
	uint32_t ext_mem_address;
	// ---- configuration (set by ymz280b_init) ----
	uint32_t clock;
	const uint8_t *rom;
	uint32_t rom_size;
	uint32_t rom_mask;       // ROM size rounded up to a power of two, minus 1 (mirroring)
	void (*irq)(void *user, int state);
	void *user;
} ymz280b_t;

#define YMZ280B_STATE_SIZE offsetof(ymz280b_t, clock)

// rom: the sample region (24-bit address space; a power-of-two region mirrors, bytes past a
// non-power-of-two region read 0). irq may be NULL.
void ymz280b_init(ymz280b_t *c, uint32_t clock, const uint8_t *rom, uint32_t rom_size,
	void (*irq)(void *user, int state), void *user);
void ymz280b_reset(ymz280b_t *c);
// offset & 1: 0 = address port, 1 = data port. Render the chip up to the current time first.
void ymz280b_write(ymz280b_t *c, int offset, uint8_t v);
// offset & 1: 0 = external memory readback (0xff while disabled), 1 = status (clears it).
uint8_t ymz280b_read(ymz280b_t *c, int offset);
// output rate in Hz: clock / 192 (88200 for the usual 16.9344 MHz)
uint32_t ymz280b_rate(const ymz280b_t *c);
// adds n stereo samples (16-bit scale) at ymz280b_rate() into left[] / right[]; returns n
int ymz280b_render(ymz280b_t *c, int32_t *left, int32_t *right, int n);

#ifdef __cplusplus
}
#endif

#endif
