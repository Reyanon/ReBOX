/*  The sound board of Irem M92 / M107 (irem_snd.h). Ported from MAME src/mame/irem/m92.cpp and m107.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail).
    This file is part of ReBOX. GPL-2.0-or-later.

    The V35 runs on a core of its own, a few lines behind the main board and never a frame: the main board posts it each line
    and takes its answers ANSWER_LINES lines after the line they were written in, waiting for the V35 if it is further
    behind, so the result is the same whatever the threads do. The games send commands frames apart without waiting
    (nbbatman's sound start, again at every coin and stage), and some need the answer before the next: a frame's lag loses
    the order, and the music with it (ADR-029).
*/
#include "irem_snd.h"
#include "../../rbx.h"
#include "../../cpu/nec/nec.h"
#include "../../sound/iremga20.h"
#include "../../sound/rbx_ym.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define V35_CLOCK 14318181u
#define YM_CLOCK (V35_CLOCK / 4)
#define GA_OVER 16             // (GA20 samples, clock / 4, for a YM2151 sample, clock / 64)
#define OUT_MAX 2048
#define ANSWER_LINES 16

static nec_t s_c;
static ga20_t s_ga;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t s_ram[0x4000];
static void (*s_answer)(int state);
static int s_stereo, s_lines;
static uint32_t s_frame_mhz;

static struct {
	uint8_t latch, latch_pending, held, pad;
	uint8_t latch2, latch2_pending, pad2[2];
	int64_t clk_end;
	uint64_t frac;
	uint32_t ym_frac;
	uint8_t state_end;
} t;

// main board to V35: its lines and commands, in order
enum { Q_ADV, Q_LATCH, Q_RESET };
#define Q_SIZE 4096
typedef struct { uint32_t kind, v; } q_ev_t;
static q_ev_t s_q[Q_SIZE];
static volatile uint32_t s_q_in, s_q_out;
// V35 to main board
#define A_SIZE 256
typedef struct { uint32_t line; uint8_t v; } ans_t;
static ans_t s_ans[A_SIZE];
static volatile uint32_t s_a_in;
static uint32_t s_a_out;
static uint32_t s_line;                // (lines since the reset, main board)
static uint32_t s_job_line;            // (the same, the line the V35 is in)
static volatile uint32_t s_job_done;   // (lines the V35 has finished)

static int32_t s_ga_out[OUT_MAX];
static int s_out_pos;
static int32_t s_mix[OUT_MAX], s_mix_r[OUT_MAX];

// ---- the V35's map (sound_map) ----
static uint8_t snd_r8(uint32_t a)
{
	if (a >= 0xa8000 && a < 0xa8040)
		return (a & 1) ? 0 : ga20_read(&s_ga, (a - 0xa8000) >> 1);
	if (a >= 0xa8040 && a < 0xa8044)
		return (a & 1) ? 0 : rbx_ym_read(0, (a - 0xa8040) >> 1);
	if (a == 0xa8044)
		return t.latch;
	return 0;
}
static uint16_t snd_r16(uint32_t a) { return snd_r8(a); }
static void snd_w8(uint32_t a, uint8_t v)
{
	if (a >= 0xa8000 && a < 0xa8040) {
		if (!(a & 1))
			ga20_write(&s_ga, (a - 0xa8000) >> 1, v);
	} else if (a >= 0xa8040 && a < 0xa8044) {
		if (!(a & 1))
			rbx_ym_write(0, (a - 0xa8040) >> 1, v);
	} else if (a == 0xa8044) {   // (acknowledge)
		t.latch_pending = 0;
		nec_set_input(&s_c, NEC_LINE_INTP1, 0);
	} else if (a == 0xa8046) {
		const uint32_t i = s_a_in;
		if (i - __atomic_load_n(&s_a_out, __ATOMIC_ACQUIRE) < A_SIZE) {
			s_ans[i & (A_SIZE - 1)].line = s_job_line;
			s_ans[i & (A_SIZE - 1)].v = v;
			__atomic_store_n(&s_a_in, i + 1, __ATOMIC_RELEASE);
		}
	}
}
static void snd_w16(uint32_t a, uint16_t v) { snd_w8(a, (uint8_t)v); }
static void ym_irq(int state) { nec_set_input(&s_c, NEC_LINE_INTP0, state); }

static void map_v35(void)
{
	nec_map(&s_c, 0, 0x20000, s_rom, NULL);
	nec_map(&s_c, 0xa0000, 0xa4000, s_ram, s_ram);
	nec_map(&s_c, 0xff000, 0x100000, s_rom + 0x1f000, NULL);
}

static void sound_run(uint32_t clocks)
{
	t.clk_end += clocks;
	if (!t.held)
		nec_run_until(&s_c, t.clk_end);
	else
		s_c.total = t.clk_end;
	t.ym_frac += clocks;
	const uint32_t yc = t.ym_frac / 4;
	t.ym_frac -= yc * 4;
	int32_t dummy[64];
	const int n = rbx_ym_advance(0, yc, dummy, 64);
	if (n > 0 && s_out_pos + n <= OUT_MAX) {
		memset(s_ga_out + s_out_pos, 0, sizeof(int32_t) * (size_t)n);
		ga20_render(&s_ga, s_ga_out + s_out_pos, n, GA_OVER);
		s_out_pos += n;
	}
}

// the worker's job: all that is posted
static void job(void)
{
	uint32_t o = s_q_out;
	const uint32_t in = __atomic_load_n(&s_q_in, __ATOMIC_ACQUIRE);
	for (; o != in; o++) {
		const q_ev_t e = s_q[o & (Q_SIZE - 1)];
		switch (e.kind) {
		case Q_ADV:
			sound_run(e.v);
			s_job_line++;
			__atomic_store_n(&s_job_done, s_job_line, __ATOMIC_RELEASE);
			break;
		case Q_LATCH:
			// (two writes in one line: the V35 takes the first as it does between them on the board, borrowing the time)
			if (!t.held)
				for (int k = 0; k < 64 && t.latch_pending; k++)
					nec_run_until(&s_c, s_c.total + 16);
			t.latch = (uint8_t)e.v;
			t.latch_pending = 1;
			nec_set_input(&s_c, NEC_LINE_INTP1, 1);
			break;
		case Q_RESET:   // (MAME resets the device when the line clears, and suspends it while asserted)
			if (e.v)
				t.held = 1;
			else if (t.held) {
				t.held = 0;
				nec_reset(&s_c);
			}
			break;
		}
		__atomic_store_n(&s_q_out, o + 1, __ATOMIC_RELEASE);
	}
}

// ---- the main board's side ----
static void q_put(uint32_t kind, uint32_t v)
{
	const uint32_t i = s_q_in;
	if (i - __atomic_load_n(&s_q_out, __ATOMIC_ACQUIRE) >= Q_SIZE)
		rbx_aux2_sync();
	s_q[i & (Q_SIZE - 1)].kind = kind;
	s_q[i & (Q_SIZE - 1)].v = v;
	__atomic_store_n(&s_q_in, i + 1, __ATOMIC_RELEASE);
}

void isnd_latch_w(uint8_t v) { q_put(Q_LATCH, v); }
void isnd_reset_line(int asserted) { q_put(Q_RESET, (uint32_t)asserted); }

uint8_t isnd_latch2_r(void)
{
	if (t.latch2_pending) {
		t.latch2_pending = 0;
		s_answer(0);
	}
	return t.latch2;
}

void isnd_line(int line)
{
	(void)line;
	const uint32_t upto = s_line - ANSWER_LINES;   // (the answers of the lines before this one)
	if ((int32_t)(__atomic_load_n(&s_job_done, __ATOMIC_ACQUIRE) - upto) < 0)
		rbx_aux2_sync();
	while (s_a_out != __atomic_load_n(&s_a_in, __ATOMIC_ACQUIRE) && (int32_t)(s_ans[s_a_out & (A_SIZE - 1)].line - upto) < 0) {
		t.latch2 = s_ans[s_a_out & (A_SIZE - 1)].v;
		__atomic_store_n(&s_a_out, s_a_out + 1, __ATOMIC_RELEASE);
		if (!t.latch2_pending) {
			t.latch2_pending = 1;
			s_answer(1);
		}
	}
	q_put(Q_ADV, (uint32_t)rbx_line_cycles(&t.frac, V35_CLOCK, s_frame_mhz, s_lines));
	s_line++;
	rbx_aux2_kick();
}

void isnd_frame(void)
{
	rbx_aux2_sync();
	const int n = s_out_pos;
	if (s_stereo) {
		memcpy(s_mix, s_ga_out, sizeof(int32_t) * (size_t)n);
		memcpy(s_mix_r, s_ga_out, sizeof(int32_t) * (size_t)n);
		rbx_ym_async_take_lr(0, s_mix, s_mix_r, n);
		for (int i = 0; i < n; i++)
			rbx_audio_put_stereo(s_mix[i], s_mix_r[i]);
	} else {
		for (int i = 0; i < n; i++)
			s_mix[i] = s_ga_out[i] * 2;   // (MAME: the chip's two outputs, the same, both on the mono speaker)
		rbx_ym_async_take(0, s_mix, n);
		for (int i = 0; i < n; i++)
			rbx_audio_put(s_mix[i]);
	}
	s_out_pos = 0;
}

int isnd_init(const uint8_t *dec, int stereo, uint32_t frame_mhz, int lines, void (*answer)(int state))
{
	uint32_t gs = 0;
	s_rom = rbx_region("soundcpu", &s_rom_size);
	uint8_t *ga = rbx_region("irem", &gs);
	if (!s_rom || s_rom_size < 0x20000 || !ga)
		return -1;
	s_answer = answer;
	s_stereo = stereo;
	s_lines = lines;
	s_frame_mhz = frame_mhz;
	nec_init(&s_c, NEC_V35, dec);
	s_c.read8 = snd_r8;
	s_c.read16 = snd_r16;
	s_c.write8 = snd_w8;
	s_c.write16 = snd_w16;
	map_v35();
	ga20_init(&s_ga, ga, gs);
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 102;   // (0.40)
	yc.irq = ym_irq;
	if (rbx_ym_init(0, &yc) || rbx_ym_set_async(0, stereo ? 2 : 1))
		return -1;
	if (stereo ? rbx_audio_setup_stereo(rbx_ym_rate(0)) : rbx_audio_setup(rbx_ym_rate(0)))
		return -1;
	rbx_aux2_set_job(job);
	rbx_aux2_set_alone(1);   // (the CPU on a core of its own where there is one)
	return 0;
}

void isnd_exit(void)
{
	rbx_aux2_set_job(NULL);
	rbx_ym_exit(0);
}

void isnd_reset(void)
{
	rbx_aux2_sync();
	rbx_aux_sync();
	s_q_in = s_q_out = s_a_in = s_a_out = 0;
	s_line = s_job_line = s_job_done = 0;
	s_out_pos = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_c.iram, 0, sizeof(s_c.iram));
	s_c.total = 0;
	nec_reset(&s_c);
	ga20_reset(&s_ga);
	rbx_ym_reset(0);
}

int isnd_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux2_sync();
	rbx_aux_sync();
	static uint8_t cpu[sizeof(nec_t)];
	if (!load)
		memcpy(cpu, &s_c, NEC_STATE_SIZE);
	rc |= rbx_state_io(ctx, "isnd_cpu", cpu, NEC_STATE_SIZE);
	rc |= rbx_state_io(ctx, "isnd", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "isnd_ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "isnd_ga", &s_ga, GA20_STATE_SIZE);
	uint32_t n[3] = { s_line, s_a_in, s_a_out };
	rc |= rbx_state_io(ctx, "isnd_n", n, sizeof(n));
	rc |= rbx_state_io(ctx, "isnd_ans", s_ans, sizeof(s_ans));
	rc |= rbx_ym_state(0, ctx, load);
	if (!rc && load) {
		memcpy(&s_c, cpu, NEC_STATE_SIZE);
		nec_after_load(&s_c);
		s_out_pos = 0;
		s_q_in = s_q_out = 0;
		s_line = s_job_line = s_job_done = n[0];
		s_a_in = n[1];
		s_a_out = n[2];
	}
	return rc ? -1 : 0;
}
