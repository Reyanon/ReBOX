/*  OKI MSM5232 8-channel tone generator, ported from MAME src/devices/sound/msm5232.cpp
    (GPL-2.0+, copyright-holders: Jarek Burczynski, Hiromitsu Shioya).
    The 11 MAME outputs (group 1: 2', 4', 8', 16'; group 2: 2', 4', 8', 16'; solo 8', solo 16'; noise) are mixed to
    mono here with a gain per output (MAME route gain x output gain, 1/256 units: output 1.0 = 32768 * gain / 256).
    The native rate is clock / 16; `divider` native samples are averaged into one output sample.
    Save state: the whole msm5232_t up to MSM5232_STATE_END is plain data.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_MSM5232_H
#define RBX_MSM5232_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { MSM5232_OUT_G1_2, MSM5232_OUT_G1_4, MSM5232_OUT_G1_8, MSM5232_OUT_G1_16, MSM5232_OUT_G2_2, MSM5232_OUT_G2_4,
       MSM5232_OUT_G2_8, MSM5232_OUT_G2_16, MSM5232_OUT_SOLO8, MSM5232_OUT_SOLO16, MSM5232_OUT_NOISE, MSM5232_OUTPUTS };

typedef struct {
	uint8_t mode;
	int32_t TG_count_period, TG_count;
	uint8_t TG_cnt, TG_out16, TG_out8, TG_out4, TG_out2;
	int32_t egvol, eg_sect, counter, eg;
	uint8_t eg_arm;
	double ar_rate, dr_rate, rr_rate;
	int32_t pitch, GF;
} msm5232_voice;

typedef struct msm5232_t msm5232_t;
struct msm5232_t {
	// ---- state (saved) ----
	msm5232_voice voi[8];
	uint32_t EN_out16[2], EN_out8[2], EN_out4[2], EN_out2[2];
	int32_t noise_cnt, noise_rng, noise_clocks;
	uint8_t control1, control2;
	int32_t gate;
	uint32_t frac;              // input clocks not yet turned into a native sample
	uint32_t div_count;
	int64_t div_acc;
	int32_t gain[MSM5232_OUTPUTS];   // 1/256 (route gain x output gain, may be changed at any time)
	// ---- configuration ----
	uint32_t clock, divider;
	int32_t noise_step;
	uint32_t update_step;
	double ar_tbl[8], dr_tbl[16];
	double cap[8];
	void (*gate_cb)(void *user, int state);
	void *user;
};
#define MSM5232_STATE_END offsetof(msm5232_t, clock)

// caps: the 8 external capacitors in farads (MAME set_capacitors); gains start at 0
void msm5232_init(msm5232_t *m, uint32_t clock, const double caps[8], int divider);
void msm5232_reset(msm5232_t *m);
uint32_t msm5232_rate(const msm5232_t *m);
void msm5232_write(msm5232_t *m, int offset, uint8_t data);
// advance by input clocks; produced mono samples are added to out[] (at most max); returns the count produced
int msm5232_render(msm5232_t *m, uint32_t clocks, int32_t *out, int max);
uint32_t msm5232_state_size(void);

#ifdef __cplusplus
}
#endif

#endif
