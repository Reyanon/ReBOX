/*  OKI MSM5205 ADPCM, ported from MAME src/devices/sound/msm5205.cpp (BSD-3-Clause, copyright-holders: Aaron Giles).
    Master clock mode (prescaler 1/96, 1/48, 1/64): the chip clocks itself, calling vck() (MAME vck_legacy_callback)
    at each ADPCM capture so the board can latch the next nibble with msm5205u_data_w.
    Time advances only through msm5205u_render (output samples at the board's rate, held between captures).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_MSM5205U_H
#define RBX_MSM5205U_H

#include <stdint.h>

// MAME prescaler selectors
enum { MSM5205_S96_3B, MSM5205_S48_3B, MSM5205_S64_3B, MSM5205_SEX_3B,
       MSM5205_S96_4B, MSM5205_S48_4B, MSM5205_S64_4B, MSM5205_SEX_4B };

typedef struct msm5205u {
	// state (saved)
	int32_t signal, step;
	int32_t countdown;          // master clocks to the next ADPCM capture
	uint32_t frac;              // master clock fraction of the output rate
	uint8_t data, reset, bitwidth, pad;
	// configuration
	uint32_t clock;
	int prescaler;              // 0 = VCK slave mode (not emulated)
	void (*vck)(struct msm5205u *m, void *user);
	void *user;
} msm5205u_t;

void msm5205u_init(msm5205u_t *m, uint32_t clock, int select, void (*vck)(msm5205u_t *, void *), void *user);
void msm5205u_reset(msm5205u_t *m);
void msm5205u_reset_w(msm5205u_t *m, int state);
void msm5205u_data_w(msm5205u_t *m, uint8_t data);
// produce n samples at rate: out[i] += output * gain / 256 (output full scale = +-16384, MAME 0.5)
void msm5205u_render(msm5205u_t *m, int32_t *out, int n, uint32_t rate, int gain);
// output now (for boards that mix themselves): -16384..16380
static inline int msm5205u_output(const msm5205u_t *m) { return (m->signal & ~3) * 8; }
// advance by master clocks (no output)
void msm5205u_advance(msm5205u_t *m, uint32_t clocks);
#define MSM5205_STATE_SIZE ((uint32_t)(sizeof(int32_t) * 3 + sizeof(uint32_t) + 4))

#endif
