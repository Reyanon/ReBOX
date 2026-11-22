/*  MPEG audio decoder (layer II and the Yamaha AMM variant only), ported from MAME 0.289
    src/devices/sound/mpeg_audio.cpp / mpeg_audio.h (BSD-3-Clause, copyright-holders: Olivier Galibert).
    MAME's limit_hit exception is a longjmp back to rbx_mpeg_audio_decode_buffer(): the work done before the
    limit was hit (output samples, synthesis window) is kept, as in MAME.
    Save state: the struct from the start up to RBX_MPEG_AUDIO_STATE_SIZE (the synthesis window, the only
    state that lives from one decode_buffer call to the next; the rest is rebuilt from each frame before use).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_MPEG_AUDIO_H
#define RBX_MPEG_AUDIO_H

#include <stdint.h>
#include <stddef.h>
#include <setjmp.h>

#ifdef __cplusplus
extern "C" {
#endif

// Accepted layers.  Beware that AMM is incompatible with L2 (and not automatically recognizable) and that 2.5
// implies 2.
enum {
	RBX_MPEG_L1 = 1,
	RBX_MPEG_L2 = 2,
	RBX_MPEG_L2_5 = 4,
	RBX_MPEG_L3 = 8,
	RBX_MPEG_AMM = 16
};

typedef struct {
	// ---- state (saved) ----
	int audio_buffer_pos[2];
	double audio_buffer[2][32*32];
	// ---- per-frame scratch (rebuilt from the bitstream before use, not saved) ----
	int sampling_rate, last_frame_number;
	int param_index, cbr_param_index;
	int channel_count, total_bands, joint_bands;
	int band_param[2][32];
	int scfsi[2][32];
	int scf[2][3][32];
	double amp_values[2][3][32];
	double bdata[2][3][32];
	double subbuffer[2][32];
	int current_pos, current_limit;
	// ---- configuration (set by rbx_mpeg_audio_init) ----
	const uint8_t *base;
	int accepted, position_align, lsb_first;
	jmp_buf limit_hit;
} rbx_mpeg_audio_t;

#define RBX_MPEG_AUDIO_STATE_SIZE offsetof(rbx_mpeg_audio_t, sampling_rate)

// base           = start of the mpeg data block
// accepted       = binary or of accepted layers
// lsb_first      = read bits out of bytes lsb-first rather than msb first
// position_align = position alignment after reading a block (0 = pure bitstream, must be a power of 2 otherwise)
void rbx_mpeg_audio_init(rbx_mpeg_audio_t *m, const void *base, unsigned int accepted, int lsb_first,
	int position_align);

// Decode one mpeg buffer.
// pos            = position in *bits* relative to base
// limit          = maximum accepted position in bits
// output         = output samples, interleaved
// output_samples = number of samples written to output per channel
// sample_rate    = output sample rate
// channels       = number of channels written to output (total sample count is output_samples*channels)
// atbl           = parameters index for older AMMSL CBR samples
// returns 1 if the buffer was complete and the new position in *pos, 0 otherwise.
// One call generates 0 or 1 frame: 384 samples per channel in layer I and 1152 otherwise (up to 1152 in the amm
// case, <1152 indicating end of stream). Layers I and III are not supported (MAME aborts; here 0 is returned).
int rbx_mpeg_audio_decode_buffer(rbx_mpeg_audio_t *m, int *pos, int limit, int16_t *output,
	int *output_samples, int *sample_rate, int *channels, int atbl);

// clear the audio (synthesis) buffer
void rbx_mpeg_audio_clear(rbx_mpeg_audio_t *m);

#ifdef __cplusplus
}
#endif

#endif
