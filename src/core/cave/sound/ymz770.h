/*  Yamaha YMZ770C "AMMS-A" (Cave CV1000), ported to C from MAME 0.260 src/devices/sound/ymz770.cpp, ymz770.h
    (BSD-3-Clause, copyright-holders: Olivier Galibert, R. Belmont, MetalliC) and mpeg_audio.cpp, mpeg_audio.h
    (BSD-3-Clause, copyright-holders: Olivier Galibert; the AMM decoder): 8 channels of AMM (MPEG layer 2 like)
    phrases, 8 sequencers, 16 kHz stereo output. (MAME 0.289's version latches the channel registers and follows the
    phrases' sample rate: not ported yet.)
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_YMZ770_H
#define RBX_YMZ770_H

#include <stddef.h>
#include <stdint.h>

#define YMZ770_RATE 16000

typedef struct {
	int sampling_rate, last_frame_number;
	int param_index;
	int channel_count, total_bands, joint_bands;
	int band_param[2][32];
	int scfsi[2][32];
	int scf[2][3][32];
	double amp_values[2][3][32];
	double bdata[2][3][32];
	double subbuffer[2][32];
	double audio_buffer[2][32 * 32];
	int audio_buffer_pos[2];
	int current_pos, current_limit;
} ymz_mpeg_t;

typedef struct {
	uint16_t phrase;
	uint8_t pan, pan_delay, pan1, pan1_delay;
	int32_t volume;
	uint8_t volume_target, volume_delay, volume2, loop;
	uint8_t is_playing, last_block, is_paused, pad;
	int16_t output_data[0x1000];
	int output_remaining, output_ptr, atbl, pptr;
	ymz_mpeg_t dec;
} ymz_channel_t;

typedef struct {
	uint32_t delay;
	uint16_t sequence, timer, stopchan;
	uint8_t loop, bank, is_playing, is_paused;
	uint32_t offset;
} ymz_sequence_t;

typedef struct {
	/* saved */
	uint8_t cur_reg, mute, doen, vlma, vlma1, bsl, cpl, pad;
	ymz_channel_t ch[8];
	ymz_sequence_t seq[8];
	/* not saved */
	const uint8_t *rom;
	uint32_t rom_size;
} ymz770_t;

#define YMZ770_STATE_SIZE ((uint32_t)offsetof(ymz770_t, rom))

void ymz770_init(ymz770_t *y, const uint8_t *rom, uint32_t rom_size);   // clears the state, then reset
void ymz770_reset(ymz770_t *y);
// the write handler (offset & 1: data to the selected register, else the register); bring the output up to the
// write's time first (MAME m_stream->update())
void ymz770_write(ymz770_t *y, uint32_t offset, uint8_t data);
// n samples (16 kHz): left / right (either may be NULL)
void ymz770_render(ymz770_t *y, int16_t *left, int16_t *right, int n);

#endif
