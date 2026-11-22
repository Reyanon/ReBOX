/*  Yamaha YMZ770C "AMMS-A" and YMZ774 "AMMS2C" (16-channel AMM / MPEG phrase player with sequencer), ported
    from MAME 0.289 src/devices/sound/ymz770.cpp / ymz770.h (BSD-3-Clause, copyright-holders: Olivier Galibert,
    R. Belmont, MetalliC). The AMM decoder is mpeg_audio.c.
    Output: 2 channels (left, right) at ymz77x_rate(): clock / 1024 (YMZ770) or clock / 512 (YMZ774), and then
    whatever sample rate the last decoded AMM frame header gave (MAME changes the stream rate the same way; here
    the driver reads the new rate after a render).
    MAME's put_int(.., 32768) float output is clamped to 16 bits here (the final mix clamp of a lone chip).
    Save state: ymz77x_state() — the struct up to YMZ77X_STATE_SIZE (no pointers inside), the 16 channel output
    buffers and the 16 decoders' synthesis windows (rbx_mpeg_audio_t up to RBX_MPEG_AUDIO_STATE_SIZE). MAME saves
    neither of the decoders' windows; here they are saved so that a load resumes with the same samples.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_YMZ774_H
#define RBX_YMZ774_H

#include <stdint.h>
#include <stddef.h>

#include "mpeg_audio.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { YMZ_TYPE_770 = 0, YMZ_TYPE_774 = 1 };

#define YMZ770_OUTPUT_SIZE 0x1000   // decoded samples per channel buffer (MAME output_data)

typedef struct {
	uint8_t pan;
	uint8_t pan_delay;
	uint8_t pan1;
	uint8_t pan1_delay;
	uint8_t volume_target;
	uint8_t volume_delay;
	uint8_t volume2;
} ymz_voice_params_t;

typedef struct {
	ymz_voice_params_t current;
	ymz_voice_params_t latch;
	uint16_t phrase;
	int32_t volume;
	uint8_t loop;
	uint8_t pending;
	uint8_t is_playing;
	uint8_t is_paused;
	uint8_t last_block;
	int output_remaining;       // int, not int32_t: passed to the decoder as int * (int32_t is long on newlib ARM)
	int output_ptr;
	int atbl;
	int pptr;                   // bit position in the ROM
} ymz_channel_t;

typedef struct {
	uint32_t delay;
	uint16_t sequence;
	uint16_t timer;
	uint16_t stopchan;
	uint8_t loop;
	uint8_t bank;
	uint32_t offset;
	uint8_t is_playing;
	uint8_t is_paused;
} ymz_sequence_t;

typedef struct {
	uint8_t sqc;
	uint8_t loop;
	uint8_t is_playing;
	uint8_t is_waiting;
	uint32_t offset;
} ymz_sqc_t;

typedef struct ymz77x_s {
	// ---- state (saved) ----
	ymz_channel_t ch[16];
	ymz_sequence_t seq[8];
	ymz_sqc_t sqc[8];
	uint32_t sclock;
	uint8_t cur_reg;
	uint8_t mute;   // mute chip
	uint8_t doen;   // digital output enable
	uint8_t vlma;   // overall volume L0/R0
	uint8_t vlma1;  // overall volume L1/R1
	uint8_t bsl;    // boost level
	uint8_t cpl;    // clip limiter
	uint8_t bank;   // YMZ774 register bank
	// ---- configuration (set by ymz77x_init) ----
	int type;                   // YMZ_TYPE_770 / YMZ_TYPE_774
	uint32_t clock, divider;
	const uint8_t *rom;
	uint32_t rom_size;
	int16_t *output_data;       // [16][YMZ770_OUTPUT_SIZE], heap
	rbx_mpeg_audio_t *dec;      // [16], heap
	uint32_t volinc[256];       // YMZ774 volume increments, fixed point fractions of 0x20000
	// ReBOX: the ROM read when needed (rom NULL, ymz77x_init_fetch): 4 KB pages in a direct-mapped cache, and the
	// decoder given a window of the bytes at its position (an AMM frame is far smaller)
	int (*fetch)(void *user, uint32_t offset, void *dst, uint32_t len);
	void *fetch_user;
	int fetch_swap16;           // (the fetched bytes are the file's of a ROM_LOAD16_WORD_SWAP region: pairs swapped here)
	uint8_t *pages;             // [YMZ_PAGES][YMZ_PAGE], heap
	uint32_t page_tag[64];
	uint8_t *window;            // [YMZ_WINDOW], heap
	uint32_t page_misses;
} ymz77x_t;

typedef struct ymz77x_s ymz774_t;

#define YMZ77X_STATE_SIZE offsetof(ymz77x_t, type)

// rom: the chip's region, the same bytes as MAME's m_rom. Allocates the decoders and the output buffers
// (about 400 KB); returns 0, or -1 when out of memory. Calls ymz77x_reset.
int ymz77x_init(ymz77x_t *c, int type, uint32_t clock, const uint8_t *rom, uint32_t rom_size);
// the same with the ROM (rom_size bytes) read through fetch (0 = ok) when needed, not held in memory
int ymz77x_init_fetch(ymz77x_t *c, int type, uint32_t clock, uint32_t rom_size,
                      int (*fetch)(void *user, uint32_t offset, void *dst, uint32_t len), void *user, int swap16);
void ymz77x_exit(ymz77x_t *c);
void ymz77x_reset(ymz77x_t *c);
// offset & 1: 0 = register select, 1 = data (YMZ774 registers when type is YMZ_TYPE_774). Render the chip up to
// the current time first (MAME m_stream->update()).
void ymz77x_write(ymz77x_t *c, uint32_t offset, uint8_t data);
// YMZ774 only: offset & 1 with register 0xe3 / 0xe4 selected = channels 8-15 / 0-7 playing; else 0. Render the
// chip up to the current time first for those two.
uint8_t ymz774_read(ymz77x_t *c, uint32_t offset);
// writes n interleaved stereo samples (L, R) at the rate ymz77x_rate() had at the start of the call
void ymz77x_render(ymz77x_t *c, int16_t *out_lr, int frames);
uint32_t ymz77x_rate(const ymz77x_t *c);
// save / load (load != 0) through rbx_state_io; tags start with prefix. Returns 0 on success.
int ymz77x_state(ymz77x_t *c, void *ctx, int load, const char *prefix);

// YMZ774 at 22.5792 MHz (IGS PGM2)
static inline int ymz774_init(ymz774_t *c, const uint8_t *rom, uint32_t rom_size)
{
	return ymz77x_init(c, YMZ_TYPE_774, 22579200, rom, rom_size);
}
static inline void ymz774_exit(ymz774_t *c) { ymz77x_exit(c); }
static inline void ymz774_reset(ymz774_t *c) { ymz77x_reset(c); }
static inline void ymz774_write(ymz774_t *c, uint32_t offset, uint8_t data) { ymz77x_write(c, offset, data); }
static inline void ymz774_render(ymz774_t *c, int16_t *out_lr, int frames) { ymz77x_render(c, out_lr, frames); }
static inline uint32_t ymz774_rate(const ymz774_t *c) { return ymz77x_rate(c); }

#ifdef __cplusplus
}
#endif

#endif
