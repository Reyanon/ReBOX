/*  Namco wavetable sound generators: NAMCO_WSG (3 voices, Pac-Man), NAMCO_15XX (8 voices, Mappy) and NAMCO_CUS30
    (8 voices, stereo, noise; Namco System 1, Baraduke). Ported from MAME 0.289 src/devices/sound/namco.cpp / namco.h
    (BSD-3-Clause, copyright-holders: Nicola Salmoria, Aaron Giles). POLEPOS_WSG (quadrophonic) is not ported.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_NAMCO_WSG_H
#define RBX_NAMCO_WSG_H

#include <stddef.h>
#include <stdint.h>

enum { NAMCO_WSG_TYPE_WSG = 0, NAMCO_WSG_TYPE_15XX, NAMCO_WSG_TYPE_CUS30 };

typedef struct {
	uint32_t frequency, counter;
	int32_t volume[2];           // (left, right; MAME keeps 4 for Pole Position)
	int32_t noise_sw, noise_state, noise_seed;
	uint32_t noise_counter;
	int32_t noise_hold;
	uint16_t waveform_select;
} namco_wsg_voice_t;

typedef struct {
	// state (saved: from here to state_end)
	namco_wsg_voice_t v[8];
	uint8_t sound_enable;
	uint8_t soundregs[0x40];
	uint8_t ram[0x400];          // the device's 0x400-byte map, by offset: CUS30 0x000-0x0ff wave RAM, 0x140-0x3ff
	                             // shared RAM; 15XX 0x040-0x3ff shared RAM (the register windows are unused here)
	uint64_t frac;               // (the chip's samples against the output's)
	int32_t cur_l, cur_r;        // (the last output sample, held when no chip sample fell in an output period)
	uint8_t state_end;
	// configuration
	int type, voices, stereo, packed;
	const uint8_t *prom;         // wave PROM (0x100 bytes, low 4 bits) for WSG / 15XX
	uint32_t sample_rate;        // the chip's stream rate (clock doubled up to >= 192 kHz)
	int f_fracbits;
	int32_t mix_res;             // 128 * voices
	int gain;                    // 1/256
} namco_wsg_t;

#define NAMCO_WSG_STATE_SIZE offsetof(namco_wsg_t, state_end)

// type: NAMCO_WSG_TYPE_*; stereo: CUS30 only (set_stereo); prom: wave PROM for WSG / 15XX (NULL for CUS30);
// gain256: 256 = MAME's stream values x 32768 (route gains such as namcos1's 0.50 are the caller's: pass 128)
void namco_wsg_init(namco_wsg_t *t, int type, uint32_t clock, int stereo, const uint8_t *prom, int gain256);
void namco_wsg_sound_enable_w(namco_wsg_t *t, int state);

// NAMCO_WSG: pacman_sound_w, offset 0x00-0x1f
void namco_wsg_pacman_sound_w(namco_wsg_t *t, int offset, uint8_t data);
// NAMCO_15XX: namco_15xx_r/w, offset 0x00-0x3f; amap: the whole 0x400-byte window (0x040-0x3ff shared RAM)
uint8_t namco_15xx_r(namco_wsg_t *t, int offset);
void namco_15xx_w(namco_wsg_t *t, int offset, uint8_t data);
uint8_t namco_15xx_amap_r(namco_wsg_t *t, int offset);
void namco_15xx_amap_w(namco_wsg_t *t, int offset, uint8_t data);
// NAMCO_CUS30: cus30_r/w, offset 0x00-0x3f; amap: the whole 0x400-byte window as namcos1 maps it
// (0x000-0x0ff wave RAM, 0x100-0x13f cus30_r/w, 0x140-0x3ff shared RAM)
uint8_t namco_cus30_r(namco_wsg_t *t, int offset);
void namco_cus30_w(namco_wsg_t *t, int offset, uint8_t data);
uint8_t namco_cus30_amap_r(namco_wsg_t *t, int offset);
void namco_cus30_amap_w(namco_wsg_t *t, int offset, uint8_t data);

// add n samples at out_rate (16-bit scale, gain applied) to left / right (right may be NULL; mono chips add the
// same value to both). out_rate must be >= sample_rate / 1024.
void namco_wsg_render(namco_wsg_t *t, int32_t *left, int32_t *right, int n, uint32_t out_rate);

#endif
