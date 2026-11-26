// The output's DC blocker and level-following gain (audio.c rbx_audio_push), in a header of its own so that a host test
// can run the same code (ADR-161: the same samples out as before, checked against the old code on x86 and ARM).
// This file is part of ReBOX. GPL-2.0-or-later.
#ifndef RBX_FRONTEND_AUDIO_GAIN_H
#define RBX_FRONTEND_AUDIO_GAIN_H

#include <stdint.h>

static int32_t s_env = 4096 << 8;   // the output gain's peak envelope of the input (both sides, x256): out_gain below
static int32_t s_gain256 = 6 * 256;
static int32_t s_gain_e = -1;       // the envelope level s_gain256 was worked out for (-1: none yet)
// Sound options (user 11-26, issue #3: the level-following gain heard as the volume moving): Normalize off = a fixed
// gain (GAIN_FIXED); Volume scales either (x256: 256 = 100%, the gain as before). s_gainv is the gain applied
#define GAIN_FIXED (2 * 256)
static int32_t s_norm = 1, s_vol256 = 256;
static int32_t s_gainv = 6 * 256;
static inline int32_t gain_applied(int32_t g256)
{
	return ((s_norm ? g256 : GAIN_FIXED) * s_vol256) >> 8;
}
static inline void gain_set_level(int normalize, int vol256)
{
	s_norm = normalize ? 1 : 0;
	s_vol256 = vol256 < 0 ? 0 : vol256 > 512 ? 512 : vol256;
	s_gainv = gain_applied(s_gain256);
}
// a DC blocker ahead of the gain (a one-pole high pass near 5 Hz): legacy AY outputs carry a large constant level
// (B-Wings: 12000 of a 15000 peak), which the gain would take for loudness
static int32_t s_dc[2];   // (x256)
static inline int32_t dc_block(int ch, int32_t v)
{
	const int32_t x = v << 8;
	s_dc[ch] += (x - s_dc[ch]) >> 11;
	int32_t y = (x - s_dc[ch]) >> 8;
	return y > 32767 ? 32767 : y < -32768 ? -32768 : y;
}

static inline void gain_reset(void)
{
	s_env = 4096 << 8;   // (the next game's gain from x6)
	s_dc[0] = s_dc[1] = 0;
	s_gain256 = 6 * 256;
	s_gain_e = -1;
	s_gainv = gain_applied(s_gain256);
}

// output gain: the sound was too quiet on the Vita speaker. x6 (x2 as ReSys12 / ReST-V, x2 more by user request
// 2026-10-31, ADR-009, and x1.5 more 2026-11-01: still quiet in Bubble Memories), with a soft limit above 75% of full scale instead of clipping: past the knee the curve bends
// smoothly towards full scale (same slope at the knee)
// 2026-11-03 (ADR-021): the fixed x6 kept loud games (Diet Go Go: 4x the level of the others) in the soft limit nearly
// all the time, heard as an amplified, muffled sound. The gain now follows the level: x6 at most, less where the
// peaks would pass the knee (an envelope of the peaks: at once up, ~11 s down), the soft limit left for what is past it.
// 2026-11-03: the envelope in x256 (the decay rounded up to 1 a sample came back from a peak in 0.6 s, not the 1.4 s
// meant: Diet Go Go's gain swung 5 dB a second with its drums, heard as a wavering sound), and a slower decay so a
// game's gain settles
// (ADR-161: the gain divided out only when the level changes, in 32 bits: the Cortex-A9 has no divide instruction and
// the 64-bit one a sample was a library call. 24576 * 256 fits, e >= 4096: the same quotient)
static inline void gain_track(int32_t l, int32_t r)
{
	int32_t m = l < 0 ? -l : l;
	const int32_t mr = r < 0 ? -r : r;
	if (mr > m)
		m = mr;
	m <<= 8;
	if (m > s_env)
		s_env = m;
	else
		s_env -= (s_env - m) >> 19;
	const int32_t e = (s_env >> 8) < 4096 ? 4096 : (s_env >> 8);
	if (e != s_gain_e) {
		s_gain_e = e;
		const int32_t g = (int32_t)((uint32_t)(24576 * 256) / (uint32_t)e);
		s_gain256 = g > 6 * 256 ? 6 * 256 : g;
		s_gainv = gain_applied(s_gain256);
	}
}
// (ADR-161: in 32 bits, same values: |v| <= 32768 after the DC blocker and the gain <= 6 * 256, so v * gain fits; past
// the knee e <= 6 * 32768 - KNEE, e * ROOM < 2^31. With Volume up to 200%: the gain <= 12 * 256, v * gain < 2^27, and
// e * ROOM < 3.1e9 fits the unsigned 32 bits)
static inline int16_t out_gain(int32_t v)
{
	enum { KNEE = 24576, ROOM = 32767 - KNEE };
	v = (v * s_gainv) >> 8;
	const int32_t m = v < 0 ? -v : v;
	if (m <= KNEE)
		return (int16_t)v;
	const int32_t e = m - KNEE, y = KNEE + (int32_t)((uint32_t)e * ROOM / (uint32_t)(e + ROOM));
	return (int16_t)(v < 0 ? -y : y);
}

// (ADR-180) dc_block, gain_track and out_gain over n sample pairs, the state in registers: a pair at a time through the
// statics, each store into the ring (uint32_t, which may alias the int32_t state) made the compiler load and store
// the state again (about 69 instructions a pair). The first nw pairs go to ring[(head + i) & mask] packed as the ring
// holds them (left low); the rest only move the state on, as ring_put dropping them on a full ring did. Same values
// as the three functions above (the aud test, ADR-180).
static inline void gain_block(const int16_t *in, int n, uint32_t *ring, uint32_t head, uint32_t mask, int nw)
{
	enum { KNEE = 24576, ROOM = 32767 - KNEE };
	int32_t dc0 = s_dc[0], dc1 = s_dc[1], env = s_env, g256 = s_gain256, ge = s_gain_e;
	const int32_t norm = s_norm, vol = s_vol256;
	int32_t gv = ((norm ? g256 : GAIN_FIXED) * vol) >> 8;   // (from the options each block: set from the menu's thread)
	for (int i = 0; i < n; i++) {
		int32_t x = in[i * 2] << 8, l, r;
		dc0 += (x - dc0) >> 11;
		l = (x - dc0) >> 8;
		l = l > 32767 ? 32767 : l < -32768 ? -32768 : l;
		x = in[i * 2 + 1] << 8;
		dc1 += (x - dc1) >> 11;
		r = (x - dc1) >> 8;
		r = r > 32767 ? 32767 : r < -32768 ? -32768 : r;
		{   // gain_track
			int32_t m = l < 0 ? -l : l;
			const int32_t mr = r < 0 ? -r : r;
			if (mr > m)
				m = mr;
			m <<= 8;
			if (m > env)
				env = m;
			else
				env -= (env - m) >> 19;
			const int32_t e = (env >> 8) < 4096 ? 4096 : (env >> 8);
			if (e != ge) {
				ge = e;
				const int32_t g = (int32_t)((uint32_t)(24576 * 256) / (uint32_t)e);
				g256 = g > 6 * 256 ? 6 * 256 : g;
				gv = ((norm ? g256 : GAIN_FIXED) * vol) >> 8;
			}
		}
		if (i >= nw)
			continue;
		int32_t o[2] = { (l * gv) >> 8, (r * gv) >> 8 };
		for (int k = 0; k < 2; k++) {   // out_gain
			const int32_t v = o[k], m = v < 0 ? -v : v;
			if (m > KNEE) {
				const int32_t e = m - KNEE, y = KNEE + (int32_t)((uint32_t)e * ROOM / (uint32_t)(e + ROOM));
				o[k] = v < 0 ? -y : y;
			}
		}
		ring[(head + (uint32_t)i) & mask] = (uint16_t)o[0] | ((uint32_t)(uint16_t)o[1] << 16);
	}
	s_dc[0] = dc0;
	s_dc[1] = dc1;
	s_env = env;
	s_gain256 = g256;
	s_gain_e = ge;
	s_gainv = gv;
}

#endif
