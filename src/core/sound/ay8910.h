/*  AY-3-8910 / YM2149 PSG, ported from MAME src/devices/sound/ay8910.cpp
    (BSD-3-Clause, copyright-holders: Couriersud; based on code by Ville Hallik, Michael Cuddy,
    Tatsuyuki Satoh, Fabrice Frances, Nicola Salmoria).
    Covers the AY8910 / AY8912 / AY8913 / YM2149 types (no AY8914 / AY8930 extensions), with the MAME
    output models: separate channel outputs (mixed to mono here) or AY8910_SINGLE_OUTPUT (resistor-mixed
    3D model), legacy (normalized) or not, and YM2149 pin 26 low (clock / 2).
    Save state: the whole ay8910_t except the callbacks and the userdata may be copied as a block
    (ay8910_state_size() bytes from the start of the struct; see AY8910_STATE_END).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_AY8910_H
#define RBX_AY8910_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// chip types
enum { AY8910_TYPE_AY8910, AY8910_TYPE_YM2149 };

// MAME set_flags values
#define AY8910_LEGACY_OUTPUT  0x01
#define AY8910_SINGLE_OUTPUT  0x02
#define YM2149_PIN26_LOW      0x10

typedef struct {
	uint32_t period;
	uint8_t volume, output, duty_cycle;
	int32_t count;
} ay8910_tone_t;

typedef struct {
	uint32_t period;
	int32_t count;
	int8_t step;
	uint32_t volume;
	uint8_t hold, alternate, attack, holding;
} ay8910_env_t;

typedef struct ay8910_t ay8910_t;
struct ay8910_t {
	// ---- state (saved) ----
	uint8_t regs[16];
	uint8_t active, register_latch, last_enable;
	uint8_t prescale_noise, count_noise, noise_out;
	uint32_t rng;
	ay8910_tone_t tone[3];
	ay8910_env_t env;
	uint8_t vol_enabled[3];
	uint32_t frac;          // input clocks not yet turned into a sample
	uint32_t div_count;     // native samples accumulated into the current output sample
	int32_t div_acc;
	int32_t dc;             // DC blocker state (x256)
	// ---- configuration (set by ay8910_init) ----
	int type, flags;
	uint32_t clock;
	uint32_t clk_per_sample; // input clocks per native sample (8, or 16 with YM2149 pin 26 low)
	uint32_t divider;        // native samples averaged into one output sample
	int gain;                // 1/256
	int dc_block;
	uint8_t env_step_mask, step, zero_is_off, single;
	// output model: per-channel tables (separate outputs) in 1/32768 units
	int32_t vol_table[3][32], env_table[3][32];
	// single output: 1 / resistance of the levels, plus the normalization
	double r_up, r_down, rl, res_inv[32], res_env_inv[32], norm_min, norm_scale;
	uint32_t last_index;
	int32_t last_value;
	double res_load[3];      // MAME m_res_load (set_resistors_load; 1000 by default)
	// ---- callbacks (not saved; set after ay8910_init, which clears them) ----
	uint8_t (*port_a_read)(void *user);
	uint8_t (*port_b_read)(void *user);
	void (*port_a_write)(void *user, uint8_t data);
	void (*port_b_write)(void *user, uint8_t data);
	void *user;
};
#define AY8910_STATE_END offsetof(ay8910_t, type)

// type AY8910_TYPE_*, flags AY8910_* / YM2149_* as in MAME set_flags (MAME's default is AY8910_LEGACY_OUTPUT),
// gain = MAME route gain in 1/256 (output 1.0 = 32767 * gain / 256), divider = native samples averaged per output
// sample (1 = native rate clock / 8), dc_block = remove the DC level of the non-legacy models (MAME keeps it)
void ay8910_init(ay8910_t *ay, int type, uint32_t clock, int flags, int gain, int divider, int dc_block);
void ay8910_reset(ay8910_t *ay);
uint32_t ay8910_rate(const ay8910_t *ay);   // output samples per second
// at most the samples ay8910_render puts out for that many clocks (a buffer added to: zeroed that far, not further)
static inline int ay8910_out_max(const ay8910_t *ay, uint32_t clocks)
{
	return (int)(clocks / (ay->clk_per_sample * ay->divider)) + 2;
}
void ay8910_address_w(ay8910_t *ay, uint8_t data);
void ay8910_data_w(ay8910_t *ay, uint8_t data);
uint8_t ay8910_data_r(ay8910_t *ay);
void ay8910_write(ay8910_t *ay, int offset, uint8_t data);   // MAME address_data_w: offset bit 0 = data
// advance by input clocks; the produced mono samples are added to out[] (at most max); returns the count produced
int ay8910_render(ay8910_t *ay, uint32_t clocks, int32_t *out, int max);
uint32_t ay8910_state_size(void);
// MAME set_resistors_load (the separate outputs' tables; call after ay8910_init)
void ay8910_set_res_loads(ay8910_t *ay, double r0, double r1, double r2);
// advance by input clocks; each native sample's three channel levels (the separate outputs' tables: 32767 = 5 V of
// AY8910_DISCRETE_OUTPUT, no gain) go to out[3 * n + c] (at most max samples); returns the count produced
int ay8910_render_ch(ay8910_t *ay, uint32_t clocks, int16_t *out, int max);

#ifdef __cplusplus
}
#endif

#endif
