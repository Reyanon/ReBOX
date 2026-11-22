/*  The sound of the small Seibu boards: their chips' samples, each at its own rate, resampled (nearest sample) into one
    mono stream at the board's output rate, a slice (a scanline) at a time, handed to rbx_audio at the frame's end.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_SEIBU_SMIX_H
#define RBX_SEIBU_SMIX_H

#include <stdint.h>
#include <string.h>
#include "../../rbx.h"

#define SMIX_CAP 4096
#define SMIX_TMP 1024

typedef struct {
	int32_t out[SMIX_CAP];
	int32_t tmp[SMIX_TMP];
	int n;              // output samples of the frame so far
	int slice;          // output samples of the current slice
	uint64_t frac;
	uint32_t rate, frame_mhz;
	int lines;
} smix_t;

static inline void smix_init(smix_t *m, uint32_t rate, uint32_t frame_mhz, int lines)
{
	memset(m, 0, sizeof(*m));
	m->rate = rate;
	m->frame_mhz = frame_mhz;
	m->lines = lines;
}

// a new slice (one of the frame's `lines`): the output samples it makes
static inline int smix_begin(smix_t *m)
{
	m->slice = rbx_line_cycles(&m->frac, m->rate, m->frame_mhz, m->lines);
	if (m->n + m->slice > SMIX_CAP)
		m->slice = SMIX_CAP - m->n;
	return m->slice;
}

// a cleared buffer for a chip's samples of this slice (at most SMIX_TMP)
static inline int32_t *smix_tmp(smix_t *m)
{
	memset(m->tmp, 0, sizeof(m->tmp));
	return m->tmp;
}

// the buffer as it is (not cleared): for a chip whose samples are assigned to all of the cnt entries it hands on
static inline int32_t *smix_buf(smix_t *m) { return m->tmp; }

// cnt samples of a chip added to out[] (n samples), stretched to its length: out[i] += src[i * cnt / n], the index
// kept as a running quotient (no division a sample: the Vita has no divide instruction)
static inline void smix_stretch(int32_t *d, int n, const int32_t *src, int cnt)
{
	if (cnt <= 0 || n <= 0)
		return;
	int idx = 0, rem = 0;   // (idx = i * cnt / n, rem = i * cnt % n)
	for (int i = 0; i < n; i++) {
		d[i] += src[idx];
		rem += cnt;
		while (rem >= n) {
			rem -= n;
			idx++;
		}
	}
}

// cnt samples of a chip (from smix_tmp) added to the slice, stretched to its length
static inline void smix_add(smix_t *m, const int32_t *src, int cnt)
{
	smix_stretch(m->out + m->n, m->slice, src, cnt);
}

// a constant level (a DAC, an MSM5205's output) over the slice
static inline void smix_level(smix_t *m, int32_t v)
{
	int32_t *d = m->out + m->n;
	for (int i = 0; i < m->slice; i++)
		d[i] += v;
}

static inline void smix_end(smix_t *m) { m->n += m->slice; m->slice = 0; }

// the frame's samples to rbx_audio
static inline void smix_flush(smix_t *m)
{
	for (int i = 0; i < m->n; i++)
		rbx_audio_put(m->out[i]);
	memset(m->out, 0, sizeof(int32_t) * (size_t)(m->n + 1 < SMIX_CAP ? m->n + 1 : SMIX_CAP));
	m->n = 0;
}

#endif
