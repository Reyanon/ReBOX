/*  Space Invaders family sound boards, after MAME src/mame/midw8080/8080bw_a.cpp and mw8080bw_a.cpp
    (license BSD-3-Clause, copyright-holders: Nicola Salmoria, Tormod Tjaberg, Mirko Buffoni, Lee Taylor,
    Valerio Verrando, Zsolt Vasvari, Aaron Giles, Jonathan Gevaryahu, hap, Robbbert; thanks-to: Michael Strutts,
    Marco Cassili). The sound port handlers (which bit starts which sample on which channel, mute, flip screen and
    colour latches) follow MAME line by line.
    The sound generators are simplified: MAME uses netlist / discrete circuits, SN76477 models and the MAME sample
    sets. Here:
      - samples: if the MAME sample set zip (invaders.zip, lrescue.zip, lupin3.zip with 1.wav ... ) is in a "samples"
        folder next to a ROM folder it is used; otherwise every sample is synthesised (noise bursts, tone sweeps,
        beeps) so the games are never silent. No sample data is shipped.
      - SN76477 saucer / helicopter: a square VCO swept by a triangle SLF (the invaders configuration).
      - DISCRETE_NOTE music (indianbt, ballbomb, lupin3, polaris): the counter chain is the MAME node (count1 from the
        data to max1, count2 to max2, 7492 transform, R1 DAC), followed by one-pole filters.
      - polaris and schaser discrete effects: noise (17-bit LFSR) and oscillators gated and enveloped like the
        circuits describe (enable / decay times from the RC values), without the op-amp models.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "si8080.h"
#include "../../rbx.h"
#include "../../zip.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FS ((double)SIA_RATE)
#define NCHAN 6
#define MAXSAMP 10

typedef struct { int16_t *data; uint32_t len; } sample_t;

// one-pole filters
typedef struct { float y, a; } lp_t;
static void lp_set(lp_t *f, double tau) { f->a = tau > 0 ? (float)(1.0 - exp(-1.0 / (FS * tau))) : 1.0f; }
static inline float lp_run(lp_t *f, float x) { f->y += (x - f->y) * f->a; return f->y; }
static inline float hp_run(lp_t *f, float x) { f->y += (x - f->y) * f->a; return x - f->y; }

typedef struct {
	// DISCRETE_NOTE
	double t_clock, t_left;
	int count1, count2, max1, max2;
	uint8_t data, kind;   // kind 0 = 7492 / 3-bit R1 DAC (indianbt), 1 = 2-bit R1 DAC (polaris)
	lp_t dac, hp;
	float gain;
} note_t;

typedef struct {
	int idx;        // sample, -1 = idle
	uint32_t pos;
	uint8_t loop;
} chan_t;

// everything that is saved (no pointers)
typedef struct {
	chan_t ch[NCHAN];
	uint8_t port1_last, port2_last;
	uint8_t mute;
	uint8_t sn_on;              // SN76477 VCO audible
	double sn_phase, sn_slf;
	uint8_t speaker;
	lp_t spk_hp;
	note_t note;
	uint8_t lupin3_step;
	uint32_t lfsr_lo, lfsr_hi, rng;
	double lfsr_lo_t, lfsr_hi_t;
	// schaser
	uint8_t dot_en, dot_sel, snd_en, music_bit, explosion, effect, last_effect, e555_low;
	double e555_left, e555_remain;   // seconds until the 555 output toggles; remaining high time while held
	double dot_phase, exp_env, eff_phase;
	lp_t dot_lp1, dot_lp2, exp_lp, mus_hp;
	// polaris
	uint8_t sx[12];
	double env_shot, env_hit, env_exp, env_hit2, plane_f, ship_phase, ship_mod, plane_phase, sonar_t, sonar_env, sonar_phase;
	int sonar_count;
	lp_t pol_exp_lp, pol_lo_bp, pol_out_hp;
} sia_state_t;

static sia_state_t A;
static int s_board = SIA_NONE;
static sample_t s_samp[MAXSAMP];
static int s_nsamp;
static int s_loaded_wav;

// ---- sample synthesis (used when the MAME sample set is not present) ----
static uint32_t s_seed = 1;
static float frand(void) { s_seed = s_seed * 1664525u + 1013904223u; return (float)((int32_t)s_seed >> 8) / 8388608.0f; }

typedef struct {
	float dur;               // seconds
	float noise, ncut;       // noise level, low-pass time constant (s)
	float tone, f0, f1;      // square tone level, start / end frequency
	float tau;               // exponential decay time (0 = flat)
	float gate;              // on/off beeping rate in Hz (0 = continuous)
	float warble;            // alternate f0/f1 at this rate instead of sweeping
} synth_t;

static int16_t *synth(const synth_t *s, uint32_t *len)
{
	uint32_t n = (uint32_t)(s->dur * FS);
	int16_t *d = (int16_t *)malloc(n * 2);
	if (!d) { *len = 0; return NULL; }
	float ny = 0, ph = 0;
	float na = s->ncut > 0 ? (float)(1.0 - exp(-1.0 / (FS * s->ncut))) : 1.0f;
	for (uint32_t i = 0; i < n; i++) {
		float t = (float)i / (float)FS;
		float env = s->tau > 0 ? expf(-t / s->tau) : 1.0f;
		if (s->gate > 0 && fmodf(t * s->gate, 1.0f) >= 0.5f)
			env = 0;
		float f = s->warble > 0 ? (fmodf(t * s->warble, 1.0f) < 0.5f ? s->f0 : s->f1) : s->f0 + (s->f1 - s->f0) * (t / s->dur);
		ph += f / (float)FS;
		ph -= floorf(ph);
		ny += (frand() - ny) * na;
		float v = s->noise * ny * 2.5f + s->tone * (ph < 0.5f ? 1.0f : -1.0f);
		float fade = i + 64 > n ? (float)(n - i) / 64.0f : 1.0f;   // no click at the end
		int x = (int)(v * env * fade * 12000.0f);
		d[i] = (int16_t)(x > 32767 ? 32767 : x < -32768 ? -32768 : x);
	}
	*len = n;
	return d;
}

// MAME sample names, in order, and the synthesised stand-ins
static const char *const invaders_names[] = { "1", "2", "3", "4", "5", "6", "7", "8", "9", NULL };
static const synth_t invaders_synth[] = {
	{ 0.40f, 0.6f, 0.0002f, 0.25f, 1500, 300, 0.12f, 0, 0 },     // shot / missile
	{ 1.10f, 1.0f, 0.0010f, 0, 0, 0, 0.35f, 0, 0 },              // base hit / explosion
	{ 0.35f, 0.3f, 0.0004f, 0.5f, 900, 150, 0.15f, 0, 0 },       // invader hit
	{ 0.10f, 0, 0, 0.8f, 98, 90, 0.05f, 0, 0 },                  // fleet move 1
	{ 0.10f, 0, 0, 0.8f, 87, 80, 0.05f, 0, 0 },                  // fleet move 2
	{ 0.10f, 0, 0, 0.8f, 77, 71, 0.05f, 0, 0 },                  // fleet move 3
	{ 0.10f, 0, 0, 0.8f, 65, 60, 0.05f, 0, 0 },                  // fleet move 4
	{ 0.90f, 0, 0, 0.5f, 700, 1000, 0.5f, 0, 16 },               // UFO / saucer hit
	{ 0.80f, 0, 0, 0.45f, 1100, 1100, 0, 8, 0 },                 // bonus base
};
static const char *const lrescue_names[] = { "alienexplosion", "rescueshipexplosion", "beamgun", "thrust", "bonus2",
                                             "bonus3", "shootingstar", "stepl", "steph", NULL };
static const synth_t lrescue_synth[] = {
	{ 0.50f, 0.8f, 0.0003f, 0, 0, 0, 0.18f, 0, 0 },
	{ 1.20f, 1.0f, 0.0012f, 0, 0, 0, 0.40f, 0, 0 },
	{ 0.25f, 0, 0, 0.45f, 2000, 600, 0.10f, 0, 0 },
	{ 0.25f, 0.6f, 0.0015f, 0, 0, 0, 0, 0, 0 },
	{ 0.30f, 0, 0, 0.45f, 1500, 1500, 0, 10, 0 },
	{ 0.40f, 0, 0, 0.45f, 800, 1600, 0, 10, 0 },
	{ 1.00f, 0, 0, 0.35f, 3000, 500, 0.6f, 0, 0 },
	{ 0.04f, 0, 0, 0.7f, 180, 180, 0.02f, 0, 0 },
	{ 0.04f, 0, 0, 0.7f, 260, 260, 0.02f, 0, 0 },
};
static const char *const lupin3_names[] = { "cap", "bark", "walk1", "walk2", "warp", "extend", "kick", NULL };
static const synth_t lupin3_synth[] = {
	{ 0.80f, 0, 0, 0.45f, 900, 300, 0, 6, 0 },
	{ 0.30f, 0.9f, 0.0004f, 0, 0, 0, 0, 7, 0 },
	{ 0.03f, 0, 0, 0.6f, 300, 300, 0.02f, 0, 0 },
	{ 0.03f, 0, 0, 0.6f, 400, 400, 0.02f, 0, 0 },
	{ 0.50f, 0, 0, 0.4f, 200, 2000, 0, 0, 0 },
	{ 0.80f, 0, 0, 0.45f, 1200, 1200, 0, 8, 0 },
	{ 0.15f, 0.6f, 0.0008f, 0.6f, 90, 60, 0.06f, 0, 0 },
};

// ---- WAV samples from the MAME sample set zip ----
static int16_t *wav_decode(const uint8_t *b, uint32_t n, uint32_t *len)
{
	if (n < 12 || memcmp(b, "RIFF", 4) || memcmp(b + 8, "WAVE", 4))
		return NULL;
	int fmt = 0, chans = 0, bits = 0;
	uint32_t rate = 0;
	const uint8_t *data = NULL;
	uint32_t dlen = 0;
	for (uint32_t p = 12; p + 8 <= n;) {
		uint32_t cl = (uint32_t)(b[p + 4] | b[p + 5] << 8 | b[p + 6] << 16 | (uint32_t)b[p + 7] << 24);
		const uint8_t *c = b + p + 8;
		if (!memcmp(b + p, "fmt ", 4) && cl >= 16 && p + 8 + 16 <= n) {
			fmt = c[0] | c[1] << 8;
			chans = c[2] | c[3] << 8;
			rate = (uint32_t)(c[4] | c[5] << 8 | c[6] << 16 | (uint32_t)c[7] << 24);
			bits = c[14] | c[15] << 8;
		} else if (!memcmp(b + p, "data", 4)) {
			data = c;
			dlen = cl <= n - (p + 8) ? cl : n - (p + 8);
		}
		p += 8 + cl + (cl & 1);
	}
	if (fmt != 1 || !data || !rate || chans < 1 || (bits != 8 && bits != 16))
		return NULL;
	uint32_t frame = (uint32_t)(chans * bits / 8), in = dlen / frame;
	uint32_t out = (uint32_t)((uint64_t)in * SIA_RATE / rate);
	int16_t *d = (int16_t *)malloc((out ? out : 1) * 2);
	if (!d)
		return NULL;
	for (uint32_t i = 0; i < out; i++) {
		// linear interpolation to the output rate, first channel
		double sp = (double)i * rate / SIA_RATE;
		uint32_t k = (uint32_t)sp;
		double fr = sp - k;
		int v[2];
		for (int j = 0; j < 2; j++) {
			uint32_t kk = k + (uint32_t)j < in ? k + (uint32_t)j : in - 1;
			const uint8_t *s = data + kk * frame;
			v[j] = bits == 8 ? ((int)s[0] - 128) << 8 : (int16_t)(s[0] | s[1] << 8);
		}
		d[i] = (int16_t)(v[0] + (v[1] - v[0]) * fr);
	}
	*len = out;
	return d;
}

static rbx_zip *open_samples_zip(const char *name)
{
	char path[600];
	for (int i = 0; rbx_rom_dir(i); i++) {
		const char *dir = rbx_rom_dir(i);
		// "samples" next to the ROM folder (dir ends with a separator), then inside it
		size_t l = strlen(dir);
		while (l > 0 && (dir[l - 1] == '/' || dir[l - 1] == '\\'))
			l--;
		size_t cut = l;
		while (cut > 0 && dir[cut - 1] != '/' && dir[cut - 1] != '\\')
			cut--;
		snprintf(path, sizeof(path), "%.*ssamples/%s.zip", (int)cut, dir, name);
		rbx_zip *z = rbx_zip_open(path);
		if (z)
			return z;
		snprintf(path, sizeof(path), "%.*s/samples/%s.zip", (int)l, dir, name);
		z = rbx_zip_open(path);
		if (z)
			return z;
	}
	return NULL;
}

static void load_samples(const char *setname, const char *const *names, const synth_t *syn)
{
	rbx_zip *z = open_samples_zip(setname);
	s_nsamp = 0;
	s_loaded_wav = 0;
	s_seed = 1;
	for (int i = 0; names[i] && i < MAXSAMP; i++) {
		sample_t *s = &s_samp[i];
		s->data = NULL;
		s->len = 0;
		if (z) {
			char fn[64];
			snprintf(fn, sizeof(fn), "%s.wav", names[i]);
			int e = rbx_zip_find(z, fn, 0, 0);
			if (e >= 0) {
				uint32_t n = rbx_zip_entry_size(z, e);
				uint8_t *buf = (uint8_t *)malloc(n ? n : 1);
				if (buf && !rbx_zip_read(z, e, buf))
					s->data = wav_decode(buf, n, &s->len);
				free(buf);
				if (s->data)
					s_loaded_wav++;
			}
		}
		if (!s->data)
			s->data = synth(&syn[i], &s->len);
		s_nsamp = i + 1;
	}
	if (z)
		rbx_zip_close(z);
	rbx_log("si8080: samples %s: %d of %d from the sample set, others synthesised", setname, s_loaded_wav, s_nsamp);
}

static void sample_start(int ch, int idx, int loop)
{
	if (ch < 0 || ch >= NCHAN || idx < 0 || idx >= s_nsamp || !s_samp[idx].data)
		return;
	A.ch[ch].idx = idx;
	A.ch[ch].pos = 0;
	A.ch[ch].loop = (uint8_t)loop;
}

static void sample_stop(int ch)
{
	if (ch >= 0 && ch < NCHAN)
		A.ch[ch].idx = -1;
}

// ---- DISCRETE_NOTE ----
static void note_init(note_t *n, double clock, int max1, int max2, int kind, float gain)
{
	memset(n, 0, sizeof(*n));
	n->t_clock = 1.0 / clock;
	n->max1 = max1;
	n->max2 = max2;
	n->kind = (uint8_t)kind;
	n->gain = gain;
	n->data = (uint8_t)max1;
	n->count1 = max1;
	if (kind == 0) {
		lp_set(&n->dac, 9560.0 * 0.1e-6);    // R1 DAC: 47k || 12k into 0.1uF
		lp_set(&n->hp, 10000.0 * 0.1e-6);    // DISCRETE_CRFILTER 10k, 0.1uF
	} else {
		lp_set(&n->dac, 0);
		lp_set(&n->hp, 580000.0 * 0.01e-6);  // op-amp high pass (580k, 0.01uF)
	}
}

static float note_step(note_t *n)
{
	double cycles = (n->t_left + 1.0 / FS) / n->t_clock;
	int inc = (int)cycles;
	n->t_left = (cycles - inc) * n->t_clock;
	if (n->data != n->max1) {
		for (int c = 0; c < inc; c++) {
			if (++n->count1 > n->max1) {
				n->count1 = n->data;
				if (++n->count2 > n->max2)
					n->count2 = 0;
			}
		}
	}
	int v = n->count2;
	float volts;
	if (n->kind == 0) {
		v += v > 2;   // "01>0+": 7492 outputs 0,1,2,4,5,6
		// R1 ladder {0, 47k, 12k}: bit 0 is not connected
		volts = 3.4f * ((v & 2 ? 1.0f / 47.0f : 0) + (v & 4 ? 1.0f / 12.0f : 0)) / (1.0f / 47.0f + 1.0f / 12.0f);
	} else {
		volts = 3.4f * ((v & 1 ? 1.0f / 47.0f : 0) + (v & 2 ? 1.0f / 12.0f : 0)) / (1.0f / 47.0f + 1.0f / 12.0f);
	}
	return hp_run(&n->hp, lp_run(&n->dac, volts)) * n->gain;
}

// ---- noise ----
static inline int lfsr_clock(uint32_t *r)
{
	// polaris_lfsr: 17 bits, XOR of bits 4 and 16 shifted into bit 0, output bit 12
	uint32_t fb = ((*r >> 4) ^ (*r >> 16)) & 1;
	*r = ((*r << 1) | fb) & 0x1ffff;
	if (!*r)
		*r = 1;
	return (*r >> 12) & 1;
}

static float noise_at(uint32_t *r, double *t, double freq)
{
	*t += freq / FS;
	while (*t >= 1.0) {
		*t -= 1.0;
		lfsr_clock(r);
	}
	return (*r >> 12) & 1 ? 1.0f : -1.0f;
}

// ---- SN76477 saucer (invaders configuration: VCO 0.1uF / 8.2k swept by the SLF 1uF / 120k) ----
static float sn_saucer(void)
{
	A.sn_slf += 5.3 / FS;
	if (A.sn_slf >= 1.0) A.sn_slf -= 1.0;
	double tri = A.sn_slf < 0.5 ? A.sn_slf * 2 : 2 - A.sn_slf * 2;
	A.sn_phase += (420.0 + 520.0 * tri) / FS;
	if (A.sn_phase >= 1.0) A.sn_phase -= 1.0;
	return A.sn_on ? (A.sn_phase < 0.5 ? 0.25f : -0.25f) : 0.0f;
}

// ---- init ----
int sia_init(int board, const char *set)
{
	(void)set;
	s_board = board;
	memset(&A, 0, sizeof(A));
	s_nsamp = 0;
	switch (board) {
	case SIA_LRESCUE:
		load_samples("lrescue", lrescue_names, lrescue_synth);
		break;
	case SIA_LUPIN3:
		load_samples("lupin3", lupin3_names, lupin3_synth);
		break;
	case SIA_SCHASER: case SIA_CRASHRD: case SIA_POLARIS: case SIA_NONE:
		break;
	default:
		load_samples("invaders", invaders_names, invaders_synth);
		break;
	}
	sia_reset();
	return 0;
}

void sia_exit(void)
{
	for (int i = 0; i < MAXSAMP; i++) {
		free(s_samp[i].data);
		s_samp[i].data = NULL;
	}
	s_nsamp = 0;
}

void sia_reset(void)
{
	memset(&A, 0, sizeof(A));
	for (int i = 0; i < NCHAN; i++)
		A.ch[i].idx = -1;
	A.lupin3_step = 2;
	A.lfsr_lo = A.lfsr_hi = 1;
	A.rng = 1;
	lp_set(&A.spk_hp, 0.02);
	switch (s_board) {
	case SIA_INDIANBT: note_init(&A.note, 7680.0 * 8, 255, 5, 0, 21000.0f * 0.25f / 3.4f); break;
	case SIA_LUPIN3: note_init(&A.note, 7680.0 * 8, 255, 5, 0, 21000.0f / 3.4f * 0.5f); break;
	case SIA_BALLBOMB: note_init(&A.note, 3993.6 * 8, 255, 5, 0, 21000.0f / 3.4f * 0.5f); break;
	case SIA_POLARIS: note_init(&A.note, 23396, 255, 3, 1, 2500.0f); break;
	default: note_init(&A.note, 1000, 255, 5, 0, 0); break;
	}
	lp_set(&A.dot_lp1, 560.0 * 0.1e-6);
	lp_set(&A.dot_lp2, (6800.0 + 560.0) * 0.1e-6);
	lp_set(&A.exp_lp, 0.0012);
	lp_set(&A.mus_hp, (470000.0 + 50000.0) * 0.01e-6);
	lp_set(&A.pol_exp_lp, 6800.0 * 0.22e-6);
	lp_set(&A.pol_lo_bp, 0.0006);
	lp_set(&A.pol_out_hp, 0.01);
	A.mute = 0;
	if (s_board == SIA_SCHASER || s_board == SIA_CRASHRD)
		A.mute = 1;   // schaser_sh reset writes 0 to both ports: sound disabled
	A.plane_f = 700;
}

// ---- port handlers (MAME 8080bw_a.cpp / mw8080bw_a.cpp) ----
#define BIT(x, n) (((x) >> (n)) & 1)

static void invadpt2_sh_port_1_w(uint8_t data, sia_latch *l)
{
	uint8_t rising = data & ~A.port1_last;
	A.sn_on = BIT(data, 0);                         // SAUCER SOUND
	if (BIT(rising, 1)) sample_start(0, 0, 0);      // MISSILE SOUND
	if (BIT(rising, 2)) sample_start(1, 1, 0);      // EXPLOSION
	if (BIT(rising, 3)) sample_start(2, 2, 0);      // INVADER HIT
	if (BIT(rising, 4)) sample_start(5, 8, 0);      // BONUS MISSILE BASE
	l->screen_red = data & 0x04;
	A.mute = !BIT(data, 5);
	A.port1_last = data;
}

static void invadpt2_sh_port_2_w(uint8_t data, sia_latch *l)
{
	uint8_t rising = data & ~A.port2_last;
	if (BIT(rising, 0)) sample_start(4, 3, 0);      // FLEET
	if (BIT(rising, 1)) sample_start(4, 4, 0);
	if (BIT(rising, 2)) sample_start(4, 5, 0);
	if (BIT(rising, 3)) sample_start(4, 6, 0);
	if (BIT(rising, 4)) sample_start(3, 7, 0);      // SAUCER HIT
	l->flip_screen = BIT(data, 5) & l->cab;
	l->color_map = BIT(data, 5);
	A.port2_last = data;
}

void sia_write(int which, uint8_t data, sia_latch *l)
{
	uint8_t r1 = data & ~A.port1_last, r2 = data & ~A.port2_last, f2 = ~data & A.port2_last;
	switch (s_board) {
	case SIA_INVADERS:
		// invaders_audio_device::p1_w / p2_w: the same latch bits drive the discrete sounds
		if (which == 1) {
			uint8_t sr = r1;
			A.sn_on = BIT(data, 0);
			if (BIT(sr, 1)) sample_start(0, 0, 0);
			if (BIT(sr, 2)) sample_start(1, 1, 0);
			if (BIT(sr, 3)) sample_start(2, 2, 0);
			if (BIT(sr, 4)) sample_start(5, 8, 0);
			A.mute = !BIT(data, 5);
			A.port1_last = data;
		} else {
			if (BIT(r2, 0)) sample_start(4, 3, 0);
			if (BIT(r2, 1)) sample_start(4, 4, 0);
			if (BIT(r2, 2)) sample_start(4, 5, 0);
			if (BIT(r2, 3)) sample_start(4, 6, 0);
			if (BIT(r2, 4)) sample_start(3, 7, 0);
			if (l->cab_invaders)   // the flip screen line is only connected on the cocktail PCB
				l->flip_screen = BIT(data, 5);
			A.port2_last = data;
		}
		break;
	case SIA_INVADPT2:
		if (which == 1) invadpt2_sh_port_1_w(data, l); else invadpt2_sh_port_2_w(data, l);
		break;
	case SIA_SPCEWARS:
		if (which == 1) {   // spcewars_sh_port_w
			A.sn_on = BIT(data, 0);
			if (BIT(r1, 1)) sample_start(0, 0, 0);   // Shot Sound
			if (BIT(r1, 2)) sample_start(1, 1, 0);   // Base Hit
			if (BIT(r1, 3)) sample_start(2, 2, 0);   // Invader Hit
			A.speaker = BIT(data, 4);                // Various bitstream tunes
			A.port1_last = data;
		} else
			invadpt2_sh_port_2_w(data, l);
		break;
	case SIA_LRESCUE:
		if (which == 1) {
			if (BIT(r1, 0)) sample_start(0, 3, 0);   // Thrust
			if (BIT(r1, 1)) sample_start(1, 2, 0);   // Shot Sound
			if (BIT(r1, 2)) sample_start(0, 1, 0);   // Death
			if (BIT(r1, 3)) sample_start(1, 0, 0);   // Alien Hit
			if (BIT(r1, 4)) sample_start(2, 5, 0);   // Bonus Ship (not confirmed)
			A.mute = !BIT(data, 5);
			l->screen_red = data & 0x04;
			A.port1_last = data;
		} else {
			if (BIT(r2, 0)) sample_start(1, 8, 0);   // Footstep high tone
			if (BIT(r2, 1)) sample_start(1, 7, 0);   // Footstep low tone
			if (BIT(r2, 2)) sample_start(1, 4, 0);   // Bonus when counting men saved
			A.speaker = BIT(data, 3);                // Bitstream tunes - endlevel and bonus1
			if (BIT(r2, 4)) sample_start(3, 6, 0);   // Shooting Star and Rescue Ship sounds
			if (BIT(f2, 4)) sample_stop(3);
			l->flip_screen = BIT(data, 5) & l->cab;
			A.port2_last = data;
		}
		break;
	case SIA_BALLBOMB:
		if (which == 1) {
			if (BIT(r1, 0)) sample_start(1, 2, 0);   // Hit a balloon
			if (BIT(r1, 1)) sample_start(2, 0, 0);   // Shot Sound
			if (BIT(r1, 2)) sample_start(2, 1, 0);   // Base Hit
			if (BIT(r1, 3)) sample_start(1, 7, 0);   // Hit a Bomb
			if (BIT(r1, 4)) sample_start(3, 8, 0);   // Bonus Base at 1500 points
			A.mute = !BIT(data, 5);
			l->screen_red = data & 0x04;
			A.port1_last = data;
		} else if (which == 2) {
			if (BIT(data, 0)) sample_start(0, 7, 0);   // Indicates plane will drop bombs
			if (BIT(data, 2)) sample_start(0, 4, 0);   // Plane is dropping new balloons at start of level
			if (BIT(r2, 4)) sample_start(2, 2, 0);     // Balloon hit and bomb drops
			l->flip_screen = BIT(data, 5) & l->cab;
			A.port2_last = data;
		} else
			A.note.data = data | 0x80;                 // ballbomb_01_w
		break;
	case SIA_INDIANBT:
		if (which == 1) {
			if (BIT(r1, 0)) sample_start(1, 7, 0);   // Death
			if (BIT(r1, 1)) sample_start(0, 1, 0);   // Shot Sound
			if (BIT(r1, 2)) sample_start(2, 3, 0);   // Move
			if (BIT(r1, 3)) sample_start(3, 2, 0);   // Hit
			A.mute = !BIT(data, 5);
			A.port1_last = data;
		} else if (which == 2) {
			if (BIT(r2, 0)) sample_start(4, 0, 0);   // Bird dropped an egg, Lasso used
			if (BIT(r2, 1)) sample_start(4, 2, 0);   // Egg hatches, egg shot
			if (BIT(r2, 3)) sample_start(5, 0, 0);   // Grabber, Lasso caught something
			if (BIT(r2, 4)) sample_start(3, 7, 0);   // Lasso sound
			l->flip_screen = BIT(data, 5) & l->cab;
			A.port2_last = data;
		} else
			A.note.data = data;                      // indianbt_sh_port_3_w
		break;
	case SIA_INDIANBTBR:
		if (which == 1) {
			if (BIT(r1, 0)) sample_start(4, 7, 0);   // Lasso
			if (BIT(r1, 2)) sample_start(0, 1, 0);   // Shot Sound
			if (BIT(r1, 3)) sample_start(3, 2, 0);   // Hit
			A.mute = !BIT(data, 5);
			A.port1_last = data;
		} else {
			if (BIT(r2, 3)) sample_start(2, 3, 0);   // Move
			if (BIT(r2, 4)) sample_start(3, 7, 0);   // Death
			l->flip_screen = BIT(data, 5) & l->cab;
			A.port2_last = data;
		}
		break;
	case SIA_LUPIN3:
		if (which == 1) {
			if (BIT(r1, 0)) {
				sample_start(0, A.lupin3_step, 0);   // Walking, steal money
				A.lupin3_step ^= 1;
			}
			A.sn_on = BIT(data, 1);                  // Helicopter (SN76477 /ENABLE = ~bit 1)
			if (BIT(r1, 2)) sample_start(1, 4, 0);   // Translocate
			if (BIT(r1, 3)) sample_start(0, 0, 0);   // Jail
			if (BIT(r1, 4)) sample_start(2, 5, 0);   // Bonus Man
			A.port1_last = data;
		} else if (which == 2) {
			if (r2 & 0x01) sample_start(0, 6, 0);    // Lands on top of building, wife kicks man
			if (r2 & 0x10) sample_start(3, 1, 0);    // Dog barking
			l->color_map = data & 0x40;
			l->flip_screen = BIT(data, 5) & l->cab;
			A.port2_last = data;
		} else
			A.note.data = data;                      // lupin3_00_w
		break;
	case SIA_SCHASERCV:
		if (which == 1) {
			if (BIT(r1, 1)) sample_start(1, 6, 0);   // Ran over a dot
			if (BIT(r1, 4)) sample_start(0, 1, 0);   // Death
			A.port1_last = data;
		} else {
			A.speaker = BIT(data, 0);                // End-of-Level
			A.mute = !BIT(data, 4);
			l->flip_screen = BIT(data, 5) & l->cab;
		}
		break;
	case SIA_SCHASER:
		if (which == 1) {
			// bit 0 dot enable, 1 dot pitch, 2-4 effect select (555 rate), 5 explosion
			A.dot_en = BIT(data, 0);
			A.dot_sel = BIT(data, 1);
			int effect = (data >> 2) & 7;
			if (A.last_effect != effect) {
				static const double rc[8] = {
					0, (15e3 + 20e3) * 1e-6, (39e3 + 20e3) * 1e-6, (1.0 / (1.0 / 15e3 + 1.0 / 39e3) + 20e3) * 1e-6,
					(82e3 + 20e3) * 1e-6, (1.0 / (1.0 / 15e3 + 1.0 / 82e3) + 20e3) * 1e-6,
					(1.0 / (1.0 / 15e3 + 1.0 / 82e3) + 20e3) * 1e-6,
					(1.0 / (1.0 / 15e3 + 1.0 / 39e3 + 1.0 / 82e3) + 20e3) * 1e-6 };
				if (effect) {
					if (A.e555_remain > 0)
						A.e555_left = A.e555_remain;   // timer re-enabled, use up remaining 555 high time
					else if (!A.e555_low)
						A.e555_left = 0.8873 * rc[effect];
					A.e555_remain = 0;
				} else if (!A.e555_low) {
					A.e555_remain = A.e555_left;       // disable effect - stops at end of low cycle
					A.e555_left = -1;
				}
				A.last_effect = (uint8_t)effect;
			}
			A.effect = (uint8_t)effect;
			if (BIT(data, 5) && !A.explosion)
				A.exp_env = 1.0;
			A.explosion = BIT(data, 5);
			A.port1_last = data;
		} else {
			A.music_bit = BIT(data, 0);
			A.snd_en = BIT(data, 1);
			A.mute = !BIT(data, 1);
			l->schaser_bg_disable = BIT(data, 3);
			l->schaser_bg_select = BIT(data, 4);
			l->flip_screen = BIT(data, 5) & l->cab;
			A.port2_last = data;
		}
		break;
	case SIA_CRASHRD:
		if (which == 1) {   // crashrd_port03_w
			A.snd_en = BIT(data, 5);
			A.mute = !BIT(data, 5);
			A.dot_en = BIT(data, 4);
			A.dot_sel = BIT(data, 0);
			if (BIT(data, 2) && !A.explosion)
				A.exp_env = 1.0;
			A.explosion = BIT(data, 2);
		} else
			A.music_bit = BIT(data, 0);   // crashrd_port05_w: bitstream audio
		break;
	case SIA_POLARIS:
		if (which == 1)
			A.note.data = data;           // polaris_sh_port_1_w: music data
		else if (which == 2) {
			// SX0 shot, SX1 ship hit, SX2 ship, SX3 explosion, SX5 sound enable
			if (!BIT(data, 0) && A.sx[0]) A.env_shot = 1.0;
			if (!BIT(data, 1) && A.sx[1]) A.env_hit = 1.0;
			if (!BIT(data, 3) && A.sx[3]) A.env_exp = 1.0;
			A.sx[0] = BIT(data, 0); A.sx[1] = BIT(data, 1); A.sx[2] = BIT(data, 2);
			A.sx[3] = BIT(data, 3); A.sx[5] = BIT(data, 5);
		} else {
			// SX6 plane down, SX7 plane up, SX9 / SX10 hit, SX11 flip
			l->flip_screen = BIT(data, 5) & l->cab;
			if (A.sx[10] && A.sx[9] && !(BIT(data, 4) && BIT(data, 3))) A.env_hit2 = 1.0;
			A.sx[6] = BIT(data, 0); A.sx[7] = BIT(data, 1);
			A.sx[9] = BIT(data, 3); A.sx[10] = BIT(data, 4);
			if (A.sx[9] && A.sx[10]) A.env_hit2 = 1.0;
		}
		break;
	default:
		break;
	}
}

// ---- mixing ----
static inline float decay(double *env, double tau)
{
	float v = (float)*env;
	if (*env > 0.0001)
		*env *= exp(-1.0 / (FS * tau));
	else
		*env = 0;
	return v;
}

static float schaser_step(void)
{
	// 555 effect timer: the SN76477 sounds while the 555 output is low (or during an explosion)
	if (A.e555_left >= 0 && (A.effect || A.e555_low)) {
		A.e555_left -= 1.0 / FS;
		if (A.e555_left < 0) {
			static const double rc[8] = {
				0, (15e3 + 20e3) * 1e-6, (39e3 + 20e3) * 1e-6, (1.0 / (1.0 / 15e3 + 1.0 / 39e3) + 20e3) * 1e-6,
				(82e3 + 20e3) * 1e-6, (1.0 / (1.0 / 15e3 + 1.0 / 82e3) + 20e3) * 1e-6,
				(1.0 / (1.0 / 15e3 + 1.0 / 82e3) + 20e3) * 1e-6,
				(1.0 / (1.0 / 15e3 + 1.0 / 39e3 + 1.0 / 82e3) + 20e3) * 1e-6 };
			A.e555_low = !A.e555_low;
			if (A.e555_low)
				A.e555_left = 0.693 * 2 * 20e3 * 1e-6 / 2;   // PERIOD_OF_555_ASTABLE(0, 20k, 1uF) / 2
			else
				A.e555_left = A.effect ? 0.8873 * rc[A.effect] : -1;
		}
	}
	float out = 0;
	// dot: 4V / 8V square through two RC filters
	double f = A.dot_sel ? 18352.94 / 2 / 4 : 18352.94 / 2 / 8;
	A.dot_phase += f / FS;
	if (A.dot_phase >= 1.0) A.dot_phase -= 1.0;
	float dot = A.dot_en ? (A.dot_phase < 0.5 ? 3.4f : 0.0f) : 0.0f;
	out += lp_run(&A.dot_lp2, lp_run(&A.dot_lp1, dot)) * 0.096f * 0.7f;
	// SN76477: VCO tone for the effect, noise (mixer B) for the explosion with the one-shot envelope
	float sn = 0;
	if (A.explosion || A.exp_env > 0.001) {
		float n = noise_at(&A.lfsr_lo, &A.lfsr_lo_t, 9000.0);
		sn += lp_run(&A.exp_lp, n) * (float)decay(&A.exp_env, 1.2) * 2.0f;
	}
	if (A.e555_low && !A.explosion) {
		A.eff_phase += 180.0 / FS;
		if (A.eff_phase >= 1.0) A.eff_phase -= 1.0;
		sn += A.eff_phase < 0.5 ? 0.5f : -0.5f;
	}
	out += sn * 0.9f * 0.6f;
	// 1-bit music DAC through the coupling capacitor
	out += hp_run(&A.mus_hp, A.music_bit ? 3.4f : 0.0f) * 0.088f * 0.7f;
	return A.snd_en ? out * (26000.0f * 4.3f) / 3.4f : 0.0f;   // (x 4.3: MAME's level in play, schaser's music)
}

static float polaris_step(void)
{
	float lo = noise_at(&A.lfsr_lo, &A.lfsr_lo_t, 800.8);
	float hi = noise_at(&A.lfsr_hi, &A.lfsr_hi_t, 23396.0);
	float lo_f = lp_run(&A.pol_lo_bp, lo);
	float out = 0;
	// shot: low noise while enabled, then decaying high noise (680k, 1uF)
	if (A.sx[0]) out += lo_f * 0.5f;
	else if (A.env_shot > 0) out += hi * decay(&A.env_shot, 0.68) * 0.25f;
	// ship hit: low noise, decaying after release
	if (A.sx[1]) out += lo_f * 0.6f;
	else if (A.env_hit > 0) { float e = decay(&A.env_hit, 0.68); out += (lo_f * 0.6f + hi * 0.1f) * e; }
	// ship: triangle VCO modulated by a 5.75 Hz sawtooth
	A.ship_mod += 5.75 / FS;
	if (A.ship_mod >= 1.0) A.ship_mod -= 1.0;
	A.ship_phase += (180.0 + 220.0 * (1.0 - A.ship_mod)) / FS;
	if (A.ship_phase >= 1.0) A.ship_phase -= 1.0;
	if (A.sx[2]) out += (float)(A.ship_phase < 0.5 ? A.ship_phase * 4 - 1 : 3 - A.ship_phase * 4) * 0.3f;
	// explosion: low noise, decaying (680k, 0.33uF), low-pass filtered
	float ex = 0;
	if (A.sx[3]) ex = lo;
	else if (A.env_exp > 0) ex = lo * decay(&A.env_exp, 0.224);
	out += lp_run(&A.pol_exp_lp, ex) * 1.2f;
	// plane: VCO enabled by SX7, frequency falls while SX6 is set
	double target = A.sx[6] ? 350.0 : 900.0;
	A.plane_f += (target - A.plane_f) / (FS * 1.1);
	A.plane_phase += (A.plane_f + 40.0 * hi) / FS;
	if (A.plane_phase >= 1.0) A.plane_phase -= 1.0;
	if (A.sx[7]) out += (A.plane_phase < 0.5 ? 0.2f : -0.2f);
	// hit: SX10/SX9 select low / high noise, both decaying when set together
	if (A.sx[9] && A.sx[10]) out += (lo_f + hi * 0.3f) * decay(&A.env_hit2, 0.15) * 0.6f;
	else if (A.sx[9]) out += hi * 0.15f;
	else if (A.sx[10]) out += lo_f * 0.5f;
	// background sonar: 60 Hz / 16 clocks a 5-bit counter, a ping every 8 counts unless exploding
	A.sonar_t += (60.0 / 16) / FS;
	if (A.sonar_t >= 1.0) {
		A.sonar_t -= 1.0;
		A.sonar_count = (A.sonar_count + 1) & 31;
		if ((A.sonar_count & 7) == 4 && !A.sx[3])
			A.sonar_env = 1.0;
	}
	if (A.sonar_env > 0) {
		A.sonar_phase += (1100.0 + 60.0 * lo_f) / FS;
		if (A.sonar_phase >= 1.0) A.sonar_phase -= 1.0;
		out += (float)(A.sonar_phase < 0.5 ? A.sonar_phase * 4 - 1 : 3 - A.sonar_phase * 4) * decay(&A.sonar_env, 0.25) * 0.3f;
	}
	out = out * 9000.0f + note_step(&A.note);
	return A.sx[5] ? hp_run(&A.pol_out_hp, out) : 0.0f;
}

int16_t sia_sample(void)
{
	float out = 0;
	switch (s_board) {
	case SIA_SCHASER: case SIA_CRASHRD:
		out = schaser_step();
		break;
	case SIA_POLARIS:
		out = polaris_step();
		break;
	case SIA_NONE:
		return 0;
	default: {
		int32_t mix = 0;
		for (int c = 0; c < NCHAN; c++) {
			chan_t *ch = &A.ch[c];
			if (ch->idx < 0)
				continue;
			const sample_t *s = &s_samp[ch->idx];
			mix += s->data[ch->pos];
			if (++ch->pos >= s->len) {
				if (ch->loop)
					ch->pos = 0;
				else
					ch->idx = -1;
			}
		}
		out = (float)mix * (s_board == SIA_LRESCUE ? 0.75f : 1.0f) * 0.7f;
		if (s_board == SIA_INVADERS || s_board == SIA_INVADPT2 || s_board == SIA_SPCEWARS || s_board == SIA_SCHASERCV ||
		    s_board == SIA_BALLBOMB || s_board == SIA_INDIANBT || s_board == SIA_INDIANBTBR || s_board == SIA_LUPIN3)
			out += sn_saucer() * 16000.0f;
		if (s_board == SIA_SPCEWARS || s_board == SIA_LRESCUE || s_board == SIA_SCHASERCV)
			out += hp_run(&A.spk_hp, A.speaker ? 1.0f : 0.0f) * 32767.0f * 0.25f;
		if (s_board == SIA_BALLBOMB || s_board == SIA_INDIANBT || s_board == SIA_LUPIN3)
			out += note_step(&A.note);
		break;
	}
	}
	if (A.mute)
		return 0;
	int v = (int)out;
	return (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v);
}

int sia_state(void *ctx, int load)
{
	(void)load;
	return rbx_state_io(ctx, "sia", &A, sizeof(A));
}
