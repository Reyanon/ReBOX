/*  Namco C140 PCM sound chip (24 voices, stereo, 12-bit linear / 8-bit compressed samples; Namco System 2) and its
    C219 variant (16 voices, 8-bit linear / mu-law samples on a byte bus with bank registers, noise, sign / left
    inversion; Namco NA-1 / NA-2).
    Ported from MAME 0.289 src/devices/sound/c140.cpp / c140.h (BSD-3-Clause, copyright-holders: R. Belmont),
    the sample ROM read from src/mame/namco/namcos2_m.cpp c140_rom_r (BSD-3-Clause, copyright-holders: K.Wilkins,
    R. Belmont).
    This file is part of ReBOX. GPL-2.0-or-later.

    Approximations:
    - MAME updates its stream before every register write; here registers act at render-call boundaries (render in
      slices for finer timing). The key-on status read (register 5) shows the key as of the last render.
    - MAME's float delta (frequency * baserate * 2.0f / sample_rate) is frequency * 2 exactly (baserate == rate).
    - Output at a rate other than the chip's: each output sample is the mean of the chip samples in its period
      (box filter), the previous one held when none falls in it (MAME uses its own resampler).
    - The INT1 timer counts in 1/65536 chip ticks given by the caller (c140_timer_advance), not attotime.
*/
#ifndef RBX_C140_H
#define RBX_C140_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
	int32_t ptoffset, pos, key, lastdt, prevdt, dltdt;
	int32_t bank, mode, sample_start, sample_end, sample_loop;
} c140_voice_t;

typedef void (*c140_irq_cb)(void *user, int state);

typedef struct {
	// state (saved: from here to state_end)
	uint8_t reg[0x200];
	c140_voice_t v[24];
	int32_t timer_on;            // INT1 timer armed
	uint64_t timer_left;         // chip ticks x 65536 until INT1
	uint64_t frac;               // chip samples against output samples
	int32_t cur_l, cur_r;        // last output sample (held)
	uint8_t state_end;
	// configuration
	const uint8_t *rom;          // "c140" region as ROM_REGION16_BE bytes (namcos2: 0x200000)
	uint32_t rom_size;
	uint32_t clock;              // = sample rate (namcos2: 49152000/384/6 = 21333)
	int gain;                    // 1/256; 256 = MAME stream value x 32768
	c140_irq_cb irq;             // INT1 (namcos2: the sound 6809's FIRQ)
	void *irq_user;
	int16_t pcmtbl[256];
	// the C219 (c219_init): its sample bus (MAME device_rom_interface<25, 1, 0, BIG>) as host-order 16-bit words of
	// mem_size bytes from address 0 (byte a at mem[a ^ 1]); 0 elsewhere
	int c219;
	const uint8_t *mem;
	uint32_t mem_size;
	uint16_t lfsr;               // (the C219's noise; state, saved by the board apart from C140_STATE_SIZE)
} c140_t;

#define C140_STATE_SIZE offsetof(c140_t, state_end)

// rom: the region as loaded by ROM_REGION16_BE (big-endian words); samples are read through c140_rom_r
void c140_init(c140_t *t, uint32_t clock, const uint8_t *rom, uint32_t rom_size, int gain256,
               c140_irq_cb irq, void *irq_user);
void c140_reset(c140_t *t);
// namcos2_base_state::c140_rom_r: word offset on the chip's sample bus -> 16-bit data
uint16_t c140_rom_r(const c140_t *t, uint32_t offset);
// register window, offset 0x000-0x1ff
uint8_t c140_r(c140_t *t, int offset);
void c140_w(c140_t *t, int offset, uint8_t data);
// the C219 (MAME c219_device: clock = sample rate; no INT1 timer)
void c219_init(c140_t *t, uint32_t clock, const uint8_t *mem, uint32_t mem_size, int gain256);
uint8_t c219_r(c140_t *t, int offset);
void c219_w(c140_t *t, int offset, uint8_t data);
// advance the INT1 timer by ticks65536 / 65536 chip clocks (raises the irq callback when it expires)
void c140_timer_advance(c140_t *t, uint64_t ticks65536);
// add n samples at out_rate (16-bit scale, gain applied) to left / right
void c140_render(c140_t *t, int32_t *left, int32_t *right, int n, uint32_t out_rate);

#endif
