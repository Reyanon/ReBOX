#include "frontend/resample.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#define RES_NEON 1
#endif

#define TAPS   16
#define HALF   (TAPS / 2)
#define PHASES 1024
#define HIST   (TAPS - 1) // input samples carried over to the next call

static float s_coef[PHASES][TAPS];
// (ADR-180) the taps of each phase in the order the NEON loop takes them: 12-15, 8-11, 0-3, 4-7 (resample_neon)
// (ADR-185: static, not made on the heap at the first game's start, where it stayed above that game's buffers)
static float s_coefn_mem[PHASES][TAPS] __attribute__((aligned(16)));
static float (*s_coefn)[TAPS];
static int s_neon = 1;   // (rbx_resample_set_neon: 0 = the loop as before ADR-180, "feaudold")
static float s_buf[(RBX_RESAMPLE_MAX_IN + HIST) * 2]; // first HIST pairs are the tail of the previous input
static uint32_t s_pos; // next output position (16.16, in s_buf sample pairs)

static double sinc(double x)
{
	if (fabs(x) < 1e-9)
		return 1.0;
	return sin(M_PI * x) / (M_PI * x);
}

static void rint_check(void);
void rbx_resample_init(void)
{
	// for output position f between x[i] and x[i+1], tap j multiplies x[i + j - (HALF-1)]
	for (int p = 0; p < PHASES; p++) {
		double f = (double)p / PHASES, w[TAPS], sum = 0;
		for (int j = 0; j < TAPS; j++) {
			double u = f - (j - (HALF - 1));
			w[j] = fabs(u) < HALF ? sinc(u) * sinc(u / HALF) : 0.0;
			sum += w[j];
		}
		for (int j = 0; j < TAPS; j++)
			s_coef[p][j] = (float)(w[j] / sum); // DC gain 1
	}
#ifdef RES_NEON
	s_coefn = s_coefn_mem;   // (made again with the same values each game)
	if (s_coefn) {
		static const unsigned char ord[TAPS] = { 12, 13, 14, 15, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7 };
		for (int p = 0; p < PHASES; p++)
			for (int j = 0; j < TAPS; j++)
				s_coefn[p][j] = s_coef[p][ord[j]];
	}
#endif
	memset(s_buf, 0, sizeof(s_buf));
	s_pos = (uint32_t)(HALF - 1) << 16;
	rint_check();
}

// (ADR-161) lrintf was a library call twice a sample on the Vita (GCC has no inline lrint for VFPv3). VCVTR rounds by
// the FPSCR's mode as lrintf does (the default, to nearest even); rbx_resample_init checks the two agree on ties,
// signs and the ends of the range and keeps lrintf if they do not (RBX_RESAMPLE_LRINTF: always lrintf)
#if defined(__arm__) && defined(__ARM_FP) && !defined(RBX_RESAMPLE_LRINTF)
static inline int32_t rint_vcvtr(float v)
{
	union { float f; int32_t i; } r;
	__asm__("vcvtr.s32.f32 %0, %1" : "=t"(r.f) : "t"(v));
	return r.i;
}
#define RINT_FAST 1
#endif
static int s_rint_fast;   // (the check passed)

static inline int16_t clip16(float v)
{
	if (v > 32767.0f) return 32767;
	if (v < -32768.0f) return -32768;
#ifdef RINT_FAST
	if (s_rint_fast)
		return (int16_t)rint_vcvtr(v);
#endif
	return (int16_t)lrintf(v);
}

static void rint_check(void)
{
	s_rint_fast = 0;
#ifdef RINT_FAST
	static const float t[] = { 0.0f, -0.0f, 0.5f, -0.5f, 1.5f, -1.5f, 2.5f, -2.5f, 0.49999997f, -0.49999997f, 1.0000001f,
		32766.5f, -32767.5f, 32767.0f, -32768.0f, 12345.5f, -12345.5f, 100.25f, -100.75f, 7.4999995f };
	for (unsigned i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
		volatile float v = t[i];
		if (rint_vcvtr(v) != (int32_t)lrintf(v))
			return;
	}
	uint32_t x = 12345;
	for (int i = 0; i < 4096; i++) {   // (and values over the range, halves among them)
		x = x * 1103515245u + 12345u;
		volatile float v = (float)((int32_t)(x >> 8) % 65536 - 32768) * (i & 1 ? 0.5f : 0.37f);
		if (rint_vcvtr(v) != (int32_t)lrintf(v))
			return;
	}
	s_rint_fast = 1;
#endif
}
int rbx_resample_fast_rint(void) { return s_rint_fast; }
void rbx_resample_set_neon(int on) { s_neon = on; }
int rbx_resample_neon(void)
{
#ifdef RES_NEON
	return s_neon && s_coefn;
#else
	return 0;
#endif
}

#ifdef RES_NEON
// (ADR-180) The same sums as the loop below in the shipped build (-O3 -ffast-math -flto: GCC vectorizes it and picks
// its own order of adds; in rbx_audio_push inlined into the frame loop, read from its code): each channel in 4 lanes,
// lane k taps 12+k, 8+k, k, 4+k, product by product (VMLA: the product rounded, then added; the Cortex-A9 has no fused
// multiply-add), then lanes 0+2 and 1+3, then the two. (The same loop built alone, or in threads.c's unused sound
// thread, takes taps 1 and 0 the other way round in lanes 0 and 1.) The sums stay in NEON to the 16-bit samples: the
// clip as a min / max, the rounding to nearest even by adding 1.5 * 2^23 (the sum's units are then whole numbers;
// NEON rounds to nearest) and taking the integer from the bits. Before, each sample went to the ARM registers four
// times (the compares, the rounding), each a stall of the NEON queue.
static int resample_neon(uint32_t total, uint32_t step, int16_t *out, uint32_t *ppos)
{
	const float32x2_t hi = vdup_n_f32(32767.0f), lo = vdup_n_f32(-32768.0f), mg = vdup_n_f32(12582912.0f);
	const int32x2_t mgi = vdup_n_s32(0x4B400000);
	uint32_t pos = *ppos;
	int n = 0;
	for (;;) {
		const uint32_t i = pos >> 16;
		if (i + HALF >= total)
			break;
		const float *c = s_coefn[(pos & 0xffff) >> 6];
		const float *x = s_buf + (i - (HALF - 1)) * 2;
		const float32x4x2_t x0 = vld2q_f32(x), x4 = vld2q_f32(x + 8), x8 = vld2q_f32(x + 16), x12 = vld2q_f32(x + 24);
		const float32x4_t c12 = vld1q_f32(c), c8 = vld1q_f32(c + 4), c0 = vld1q_f32(c + 8), c4 = vld1q_f32(c + 12);
		float32x4_t l = vmulq_f32(x12.val[0], c12), r = vmulq_f32(x12.val[1], c12);
		l = vmlaq_f32(l, x8.val[0], c8);
		r = vmlaq_f32(r, x8.val[1], c8);
		l = vmlaq_f32(l, x0.val[0], c0);
		r = vmlaq_f32(r, x0.val[1], c0);
		l = vmlaq_f32(l, x4.val[0], c4);
		r = vmlaq_f32(r, x4.val[1], c4);
		float32x2_t lr = vpadd_f32(vadd_f32(vget_low_f32(l), vget_high_f32(l)), vadd_f32(vget_low_f32(r), vget_high_f32(r)));
		lr = vmax_f32(vmin_f32(lr, hi), lo);
		const int32x2_t v = vsub_s32(vreinterpret_s32_f32(vadd_f32(lr, mg)), mgi);
		vst1_lane_s32((int32_t *)(void *)(out + n * 2), vreinterpret_s32_s16(vmovn_s32(vcombine_s32(v, v))), 0);
		n++;
		pos += step;
	}
	*ppos = pos;
	return n;
}
#endif

int rbx_resample(const int16_t *in, int frames, uint32_t step, int16_t *out)
{
	if (frames > RBX_RESAMPLE_MAX_IN)
		frames = RBX_RESAMPLE_MAX_IN;
	float *dst = s_buf + HIST * 2;
	for (int i = 0; i < frames * 2; i++)
		dst[i] = in[i];
	const uint32_t total = HIST + (uint32_t)frames;

	int n = 0;
	uint32_t pos = s_pos;
#ifdef RES_NEON
	if (s_neon && s_coefn)
		n = resample_neon(total, step, out, &pos);
	else
#endif
	for (;;) {
		uint32_t i = pos >> 16;
		if (i + HALF >= total)
			break;
		const float *c = s_coef[(pos & 0xffff) >> 6];
		const float *x = s_buf + (i - (HALF - 1)) * 2;
		float l = 0, r = 0;
		for (int j = 0; j < TAPS; j++) {
			l += x[j * 2] * c[j];
			r += x[j * 2 + 1] * c[j];
		}
		out[n * 2] = clip16(l);
		out[n * 2 + 1] = clip16(r);
		n++;
		pos += step;
	}

	// move the last HIST pairs to the front and adjust the position
	memmove(s_buf, s_buf + (total - HIST) * 2, HIST * 2 * sizeof(float));
	s_pos = pos - ((total - HIST) << 16);
	return n;
}
