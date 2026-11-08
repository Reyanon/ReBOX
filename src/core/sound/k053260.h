/*  Konami 053260 KDSC PCM sound chip, ported to C from MAME 0.289 src/devices/sound/k053260.cpp (BSD-3-Clause,
    copyright-holders: Ernesto Corvi, Alex W. Jackson). 4 voices of 8-bit PCM or 4-bit KADPCM from a ROM (21-bit
    address), 7-bit volume, 8-step pan, 4 latch ports between the main and the sound CPU.

    Output: MAME streams the chip at clock / 64 (CLOCKS_PER_SAMPLE = 64; 3.579545 MHz: 55.9 kHz). We run the voices
    at exactly that native rate and convert to 48 kHz with a box filter: each 48 kHz sample is the time-weighted
    average of the native samples it overlaps (zero-order hold integrated over the output period). Cost: about
    1.2-1.3 native steps per output sample, a few float ops each. MAME's own resampler differs in its filter, so the
    48 kHz waveform is close but not bit-identical to MAME's.

    SH1 / SH2 / TIM2: in MAME a timer fires every 16 clocks from reset, cycling 4 states: SH1 rises at 16, falls at
    32, SH2 rises at 48, falls at 64 (period 64 clocks), TIM2 rises at every 112th SH1 fall and falls at the next
    one. These do not depend on the sound stream, so they are exposed as pure functions of the chip clocks elapsed
    since reset (the board keeps that count against its own scheduler, exact to the clock):
      k053260_sh1(t) / k053260_sh2(t) / k053260_tim2(t): the line level at clock t,
      k053260_sh1_next_rise(t): the clocks from t to the next SH1 rising edge (asterix: Z80 NMI).
    TIM2 (MAME m_tim2_count) is counted from reset here; MAME does not clear its counter at reset (only at start),
    so it differs after a soft reset only.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_K053260_H
#define RBX_K053260_H

#include <stdint.h>

typedef struct k053260_s k053260_t;
struct k053260_s {
	// ---- state (k053260_state) ----
	uint8_t portdata[4];   // 0,1: main->sub, 2,3: sub->main
	uint8_t keyon, mode;
	struct k053260_voice {
		uint32_t position, start;
		int32_t pan_volume[2];
		uint16_t counter, length, pitch;
		int8_t output;
		uint8_t playing, volume, pan, loop, kadpcm, reverse;
	} v[4];
	float hold_l, hold_r;   // the current native sample (clamped)
	uint32_t rem;           // resampler units left in the current native sample
	// ---- configuration ----
	const uint8_t *rom;
	uint32_t rom_size;
	uint32_t unit_native, unit_out;   // a native sample / a 48 kHz sample in common time units (reduced)
	float inv_out;
};

void k053260_init(k053260_t *k, const uint8_t *rom, uint32_t rom_size, uint32_t clock);
void k053260_reset(k053260_t *k);
// main CPU side: write offset 0/1 -> ports 0/1 (read by the sound CPU at 0x00/0x01); read 0/1 <- ports 2/3
uint8_t k053260_main_read(k053260_t *k, uint32_t offset);
void k053260_main_write(k053260_t *k, uint32_t offset, uint8_t data);
// sound CPU side: registers 0x00-0x2f (offset masked with 0x3f)
uint8_t k053260_read(k053260_t *k, uint32_t offset);
void k053260_write(k053260_t *k, uint32_t offset, uint8_t data);
// n 48 kHz samples added to l[] and r[] (16-bit full scale). Call it up to the current time before a register
// write or a 0x29 read to keep MAME's stream->update() ordering.
void k053260_render(k053260_t *k, int32_t *l, int32_t *r, int n);
int k053260_state(k053260_t *k, void *ctx, int load, const char *prefix);

// line outputs as functions of the chip clocks since reset
static inline int k053260_sh1(uint64_t t) { return (t & 63) >= 16 && (t & 63) < 32; }
static inline int k053260_sh2(uint64_t t) { return (t & 63) >= 48; }
static inline uint32_t k053260_sh1_next_rise(uint64_t t) { return (uint32_t)((16 - (t & 63)) & 63) ? (uint32_t)((16 - (t & 63)) & 63) : 64; }
static inline int k053260_tim2(uint64_t t)
{
	if (t < 32) return 0;
	uint64_t falls = ((t - 32) >> 6) + 1;   // SH1 falls so far
	return falls % 112 == 0;
}

#endif
