/*  The Konami GX sound board, after MAME src/mame/konami/konamigx.cpp (BSD-3-Clause, copyright-holders: R. Belmont,
    Acho A. Tang, Phil Stroffolino, Olivier Galibert) and src/devices/sound/k056800.cpp (BSD-3-Clause,
    copyright-holders: Ville Linde). See gx_sound.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "gx_sound.h"
#include "../../rbx.h"

// the second Musashi instance (the sound CPU names)
#include "../../cpu/m68k_snd/m68k_snd_names.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/tms57002/tms57002.h"
#include "../../sound/k054539.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SND_CLOCK 8000000u   // SUB_CLOCK / 2
#define DASP_CLOCK 12000000u // MASTER_CLOCK / 2
#define DASP_CYCLES ((int)(DASP_CLOCK / GXS_RATE))   // (250 a sample)

static uint64_t s_samples;
static int32_t s_out_l[2048], s_out_r[2048];   // (the board job's samples of a frame)
static struct {
	uint8_t ram[0x10000];
	uint8_t ctx[2048];   // (the Musashi context while saved)
	k054539_t k539[2];
	tms57002_t dasp;
	uint8_t dasp_ram[0x40000];   // (the DASP's data map: gxtmsmap)
	// K056800
	uint8_t host_to_snd[4], snd_to_host[2], int_pending, int_enabled;
	uint8_t sound_ctrl, sound_intck, running;
	uint64_t cyc_frac;
	uint8_t *rom;
	uint32_t rom_size;
} s;

static uint16_t dasp_status(const tms57002_t *d)   // (tms57002_status_word_r)
{
	return (uint16_t)((tms57002_dready_r(d) ? 4 : 0) | (tms57002_pc0_r(d) ? 2 : 0) | (tms57002_empty_r(d) ? 1 : 0));
}

// ---- interrupts ----
static void k056800_int(int state) { m68k_bus_set_line(1, state); }
static void k054539_irq_gen(void *user, int state)   // (the first K054539's timer)
{
	(void)user;
	if (s.sound_ctrl & 1)
		if (!s.sound_intck && state)   // (an interrupt on the rising edge)
			m68k_bus_set_line(2, 1);
	s.sound_intck = (uint8_t)state;
}

// ---- the DASP on its own worker (rbx_aux2), behind the board's job in the same frame: the job queues the chips'
// samples and its writes in their order, the worker runs the DASP through them (its outputs by sample index), the job
// waits for it before a read of the DASP and at the frame's end. The DASP sees what it saw inline, in the same order:
// the same output with or without the worker thread ----
#define DQ_SIZE 4096   // (a power of 2)
enum { DQ_SAMPLE, DQ_DATA, DQ_CTRL, DQ_RESET };
typedef struct { int32_t in[4]; uint16_t kind, idx; } dq_ev_t;
static dq_ev_t s_dq[DQ_SIZE];
static struct { volatile uint32_t v; uint8_t pad[60]; } s_dq_head __attribute__((aligned(64)));   // (written by the job)
static struct { volatile uint32_t v; uint8_t pad[60]; } s_dq_tail __attribute__((aligned(64)));   // (by the worker)
static int32_t s_dsp_l[2048], s_dsp_r[2048];   // the DASP's part of each sample of the frame (x 0.3, 16-bit units)

static void dsp_job(void)
{
	uint32_t t = s_dq_tail.v;
	const uint32_t h = __atomic_load_n(&s_dq_head.v, __ATOMIC_ACQUIRE);
	while (t != h) {
		const dq_ev_t *e = &s_dq[t & (DQ_SIZE - 1)];
		switch (e->kind) {
		case DQ_SAMPLE: {
			int32_t out[4];
			tms57002_sample(&s.dasp, e->in, out, DASP_CYCLES);
			// (out[] full scale x 2^31: x 0.3 / 65536 in 16-bit units; 19661 / 2^32 = 0.3 / 65536)
			s_dsp_l[e->idx] = (int32_t)(((int64_t)out[0] + out[2]) * 19661 >> 32);
			s_dsp_r[e->idx] = (int32_t)(((int64_t)out[1] + out[3]) * 19661 >> 32);
			break;
		}
		case DQ_DATA: tms57002_data_w(&s.dasp, (uint8_t)e->in[0]); break;
		case DQ_CTRL:   // (tms57002_control_word_w's DASP lines)
			tms57002_pload_w(&s.dasp, e->in[0] & 4);
			tms57002_cload_w(&s.dasp, e->in[0] & 8);
			tms57002_set_reset_line(&s.dasp, !(e->in[0] & 0x10));
			break;
		case DQ_RESET: tms57002_set_reset_line(&s.dasp, e->in[0]); break;
		}
		t++;
	}
	__atomic_store_n(&s_dq_tail.v, t, __ATOMIC_RELEASE);
}

// the worker done with all that was queued (then the DASP is the job's to read)
static void dsp_catch_up(void)
{
	if (__atomic_load_n(&s_dq_tail.v, __ATOMIC_ACQUIRE) != s_dq_head.v)
		rbx_aux2_sync();
}

static void dq_put(int kind, const int32_t *in, int idx)
{
	if (s_dq_head.v - __atomic_load_n(&s_dq_tail.v, __ATOMIC_ACQUIRE) >= DQ_SIZE)
		rbx_aux2_sync();   // (full)
	dq_ev_t *e = &s_dq[s_dq_head.v & (DQ_SIZE - 1)];
	e->kind = (uint16_t)kind;
	e->idx = (uint16_t)idx;
	if (kind == DQ_SAMPLE)
		memcpy(e->in, in, sizeof(e->in));
	else
		e->in[0] = in[0];
	__atomic_store_n(&s_dq_head.v, s_dq_head.v + 1, __ATOMIC_RELEASE);
}
static void dq_put1(int kind, int32_t v) { dq_put(kind, &v, 0); }

// ---- the 68000's bus ----
static uint8_t snd_r8(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x200000 && a < 0x200500)   // the K054539s: the first on the even bytes, the second on the odd ones
		return k054539_read(&s.k539[a & 1], (a - 0x200000) >> 1);
	if (a == 0x300001) {
		dsp_catch_up();
		return tms57002_data_r(&s.dasp);
	}
	if (a >= 0x400000 && a < 0x400020) {   // the K056800's sound side (odd bytes)
		if (!(a & 1))
			return 0xff;
		const int r = (int)((a - 0x400000) >> 1) & 7;
		return r < 4 ? s.host_to_snd[r] : 0;
	}
	if (a == 0x500000 || a == 0x500001) {
		dsp_catch_up();
		const uint16_t v = dasp_status(&s.dasp);
		return (uint8_t)((a & 1) ? v : v >> 8);
	}
	return 0xff;
}
static uint16_t snd_r16(uint32_t a)
{
	a &= 0xffffff;
	if (a == 0x500000) {
		dsp_catch_up();
		return dasp_status(&s.dasp);
	}
	return (uint16_t)(snd_r8(a) << 8 | snd_r8(a + 1));
}
static void control_w(uint8_t data)   // (tms57002_control_word_w, the low byte)
{
	if (!(data & 1))
		m68k_bus_set_line(2, 0);
	dq_put1(DQ_CTRL, data);
	s.sound_ctrl = data;
}
static void snd_w8(uint32_t a, uint8_t v)
{
	a &= 0xffffff;
	if (a >= 0x200000 && a < 0x200500)
		k054539_write(&s.k539[a & 1], (a - 0x200000) >> 1, v);
	else if (a == 0x300001)
		dq_put1(DQ_DATA, v);
	else if (a >= 0x400000 && a < 0x400020) {
		if (!(a & 1))
			return;
		if (getenv("GXSDBG") && ((a - 0x400000) >> 1) != 4)
			fprintf(stderr, "  t%llu snd %06x w %06x %02x\n", (unsigned long long)s_samples / 800, m68k_get_reg(NULL, M68K_REG_PC), a & ~1u, v);
		switch ((int)((a - 0x400000) >> 1) & 7) {
		case 0: case 1: s.snd_to_host[((a - 0x400000) >> 1) & 1] = v; break;
		case 4:   // (the sound CPU's interrupt control)
			s.int_enabled = (v & 1) != 0;
			if (s.int_enabled) {
				if (s.int_pending)
					k056800_int(1);
			} else {   // (disable / acknowledge)
				s.int_pending = 0;
				k056800_int(0);
			}
			break;
		default: break;
		}
	} else if (a == 0x500001)
		control_w(v);
}
static void snd_w16(uint32_t a, uint16_t v)
{
	a &= 0xffffff;
	if (a == 0x500000) {
		control_w((uint8_t)v);
		return;
	}
	snd_w8(a, (uint8_t)(v >> 8));
	snd_w8(a + 1, (uint8_t)v);
}

int gxs_init(uint8_t *rom, uint32_t rom_size, const uint8_t *pcm, uint32_t pcm_size)
{
	if (m68k_context_size() > sizeof(s.ctx))
		return -1;
	memset(&s, 0, sizeof(s));
	s.rom = rom;
	s.rom_size = rom_size;
	m68k_bus_swap16(rom, rom_size);
	tms57002_init(&s.dasp, s.dasp_ram, sizeof(s.dasp_ram));
	for (int i = 0; i < 2; i++) {
		k054539_init(&s.k539[i], pcm, pcm_size);
		s.k539[i].timer_cb = i == 0 ? k054539_irq_gen : NULL;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_select(NULL);   // (this instance's m68k_bus: Taito EN selected a bus of its own; after F3 the reads went to
	                         // its handlers, sexyparo's RAM check BAD / a crash)
	m68k_bus_reset_map();
	m68k_bus_map(0x000000, 0x03ffff, rom, rom_size < 0x40000 ? rom_size : 0x40000, 1, 0);
	m68k_bus_map(0x100000, 0x10ffff, s.ram, sizeof(s.ram), 1, 1);
	m68k_bus.read8 = snd_r8;
	m68k_bus.read16 = snd_r16;
	m68k_bus.write8 = snd_w8;
	m68k_bus.write16 = snd_w16;
	gxs_reset();
	return 0;
}

void gxs_exit(void) {}
uint32_t gxs_debug_pc(void) { return m68k_get_reg(NULL, M68K_REG_PC) | (uint32_t)s.running << 31 | (uint32_t)s.snd_to_host[0] << 24; }

void gxs_set_gains(const char *set)
{
	for (int c = 0; c < 2; c++)
		for (int i = 0; i < 8; i++)
			k054539_set_gain(&s.k539[c], i, 1.0);
	if (!strcmp(set, "tkmmpzdm"))   // (MAME: boost the voice, chip 1 channels 3-7)
		for (int i = 3; i <= 7; i++)
			k054539_set_gain(&s.k539[1], i, 2.0);
	else if (!strcmp(set, "dragoonj") || !strcmp(set, "dragoona"))   // (soften the percussions, boost the voice)
		for (int i = 0; i <= 3; i++) {
			k054539_set_gain(&s.k539[1], i, 0.8);
			k054539_set_gain(&s.k539[1], i + 4, 2.0);
		}
}

void gxs_reset(void)
{
	memset(s.ram, 0, sizeof(s.ram));
	for (int i = 0; i < 2; i++)
		k054539_reset(&s.k539[i]);
	tms57002_reset(&s.dasp);   // (MAME machine reset: the devices reset, then the DASP held in reset)
	tms57002_set_reset_line(&s.dasp, 1);
	memset(s.host_to_snd, 0, sizeof(s.host_to_snd));
	memset(s.snd_to_host, 0, sizeof(s.snd_to_host));
	s.int_pending = s.int_enabled = 0;
	s.sound_ctrl = s.sound_intck = 0;
	s.running = 0;
	s.cyc_frac = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
}

static void run_line(int run)
{
	if (run) {
		if (!s.running) {   // (the reset and halt lines released: the 68000 starts from its vectors)
			m68k_bus_irq_reset();
			m68k_pulse_reset();
		}
		s.running = 1;
		if (s.sound_ctrl & 0x10)
			dq_put1(DQ_RESET, 0);
	} else {
		s.sound_ctrl = 0;
		s.running = 0;
		dq_put1(DQ_RESET, 1);
	}
}

static void host_w(int offset, uint8_t data)
{
	const int r = offset & 7;
	if (r < 4)
		s.host_to_snd[r] = data;
	else if (r == 7 && s.int_enabled) {   // (the sound interrupt)
		s.int_pending = 1;
		k056800_int(1);
	}
}

static void render(int32_t *l, int32_t *r, int n)
{
	while (n > 0) {
		int k = n < 16 ? n : 16;   // (a block: the 68000 then the chips; the timer's toggle ends a block)
		const int due = k054539_timer_due(&s.k539[0]);
		if (due > 0 && due < k)
			k = due;
		if (s.running) {
			s.cyc_frac += (uint64_t)SND_CLOCK * (uint64_t)k;
			const int cyc = (int)(s.cyc_frac / GXS_RATE);
			s.cyc_frac -= (uint64_t)cyc * GXS_RATE;
			if (cyc > 0)
				m68k_execute(cyc);
		}
		int32_t al[16], ar[16], bl[16], br[16];
		memset(al, 0, sizeof(al));
		memset(ar, 0, sizeof(ar));
		memset(bl, 0, sizeof(bl));
		memset(br, 0, sizeof(br));
		k054539_render(&s.k539[0], al, ar, k);
		k054539_render(&s.k539[1], bl, br, k);
		for (int i = 0; i < k; i++) {
			// (MAME: the chips to the speaker at 1.0 and to the DASP's inputs at 0.5, chip 1 L/R in 0/1, chip 2 in
			// 2/3; the DASP's outputs to the speaker at 0.3, 0 and 2 left, 1 and 3 right. The inputs are full
			// scale x 65536, a chip's 16-bit sample x 0.5 that is the sample itself)
			// (the DASP's part added at the frame's end: dsp_job, gxs job)
			const int32_t in[4] = { al[i], ar[i], bl[i], br[i] };
			dq_put(DQ_SAMPLE, in, (int)(l - s_out_l) + i);
			l[i] += al[i] + bl[i];
			r[i] += ar[i] + br[i];
		}
		rbx_aux2_kick();   // (the worker on these samples)
		l += k;
		r += k;
		n -= k;
		s_samples += (uint64_t)k;
	}
}

static int board_state(void *ctx, int load)
{
	int rc = 0;
	if (!load)
		m68k_get_context(s.ctx);
	rc |= rbx_state_io(ctx, "gxs_cpu", s.ctx, m68k_context_size());
	rc |= rbx_state_io(ctx, "gxs_bus", &m68k_bus.irq_pending, sizeof(uint32_t) * 2);
	rc |= rbx_state_io(ctx, "gxs_ram", s.ram, sizeof(s.ram));
	rc |= tms57002_state(&s.dasp, ctx, load);
	rc |= rbx_state_io(ctx, "gxs_dram", s.dasp_ram, sizeof(s.dasp_ram));
	uint8_t v[12] = { s.host_to_snd[0], s.host_to_snd[1], s.host_to_snd[2], s.host_to_snd[3], s.snd_to_host[0],
	                  s.snd_to_host[1], s.int_pending, s.int_enabled, s.sound_ctrl, s.sound_intck, s.running };
	rc |= rbx_state_io(ctx, "gxs_v", v, sizeof(v));
	rc |= rbx_state_io(ctx, "gxs_frac", &s.cyc_frac, sizeof(s.cyc_frac));
	for (int i = 0; i < 2; i++)
		rc |= k054539_state(&s.k539[i], ctx, load, i ? "gxs_k539b_" : "gxs_k539a_");
	if (load && !rc) {
		memcpy(s.host_to_snd, v, 4);
		memcpy(s.snd_to_host, v + 4, 2);
		s.int_pending = v[6];
		s.int_enabled = v[7];
		s.sound_ctrl = v[8];
		s.sound_intck = v[9];
		s.running = v[10];
		m68k_set_context(s.ctx);
	}
	return rc;
}

// ---- the board a frame behind on the worker (rbx_aux): the host's writes queued with the samples before them, the
// job runs the frame through them; what the board says back (snd_to_host) is seen by the host when the job is done
// (the same sound and replies with or without the worker thread) ----
#define Q_MAX 4096
enum { Q_ADV, Q_HOST, Q_RUN };
typedef struct { uint16_t kind, arg; uint32_t v; } q_ev_t;
static q_ev_t s_q[2][Q_MAX];
static int s_qn[2], s_qcur;

static int s_out_n;
static uint8_t s_pub[2];   // (snd_to_host as of the last job)
static volatile int s_job_frame;

static void q_put(int kind, int arg, uint32_t v)
{
	const int c = s_qcur;
	if (s_qn[c] < Q_MAX) {
		s_q[c][s_qn[c]].kind = (uint16_t)kind;
		s_q[c][s_qn[c]].arg = (uint16_t)arg;
		s_q[c][s_qn[c]].v = v;
		s_qn[c]++;
	}
}

static void job(void)
{
	if (!__atomic_load_n(&s_job_frame, __ATOMIC_ACQUIRE))
		return;
	const int b = s_qcur ^ 1;
	int n = 0;
	memset(s_out_l, 0, sizeof(s_out_l));
	memset(s_out_r, 0, sizeof(s_out_r));
	for (int i = 0; i < s_qn[b]; i++) {
		const q_ev_t *e = &s_q[b][i];
		switch (e->kind) {
		case Q_ADV:
			if (n + (int)e->v <= 2048) {
				render(s_out_l + n, s_out_r + n, (int)e->v);
				n += (int)e->v;
			}
			break;
		case Q_HOST: host_w(e->arg, (uint8_t)e->v); break;
		case Q_RUN: run_line((int)e->v); break;
		}
	}
	s_qn[b] = 0;
	dsp_catch_up();   // (the DASP's part of the frame's samples)
	for (int i = 0; i < n; i++) {
		s_out_l[i] += s_dsp_l[i];
		s_out_r[i] += s_dsp_r[i];
	}
	s_out_n = n;
	__atomic_store_n(&s_job_frame, 0, __ATOMIC_RELEASE);
}

void gxs_start(void)
{
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = 0;
	s_pub[0] = s_pub[1] = 0;
	s_job_frame = 0;
	s_dq_head.v = s_dq_tail.v = 0;
	rbx_aux2_set_alone(1);   // (the DASP worker on a core of its own: CPU3 on the Vita)
	rbx_aux2_set_job(dsp_job);
	rbx_aux_set_job(job);
}

void gxs_stop(void)
{
	rbx_aux_set_job(NULL);
	rbx_aux2_set_job(NULL);
}

uint8_t gxs_host_r(int offset)
{
	const int r = offset & 7;
	return r < 2 ? s_pub[r] : 0;   // (2: the volume busy bits, 0)
}
void gxs_host_w(int offset, uint8_t data) { q_put(Q_HOST, offset & 7, data); }
void gxs_run_line(int run) { q_put(Q_RUN, 0, (uint32_t)(run != 0)); }
void gxs_advance(int n) { if (n > 0) q_put(Q_ADV, 0, (uint32_t)n); }

int gxs_frame(int32_t *l, int32_t *r, int max)
{
	rbx_aux_sync();   // (the last frame's job done: its samples out, the board's replies seen)
	int n = s_out_n < max ? s_out_n : max;
	memcpy(l, s_out_l, sizeof(int32_t) * (size_t)n);
	memcpy(r, s_out_r, sizeof(int32_t) * (size_t)n);
	s_out_n = 0;
	s_pub[0] = s.snd_to_host[0];
	s_pub[1] = s.snd_to_host[1];
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
	return n;
}

void gxs_sync_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_out_n = 0;
	s_pub[0] = s_pub[1] = 0;
	s_job_frame = 0;
	gxs_reset();
}

int gxs_state(void *ctx, int load)
{
	rbx_aux_sync();   // (the job done: the board at the frame's end, its samples waiting)
	int rc = board_state(ctx, load);
	int32_t v[3] = { s_out_n, s_pub[0], s_pub[1] };
	rc |= rbx_state_io(ctx, "gxs_q", v, sizeof(v));
	rc |= rbx_state_io(ctx, "gxs_outl", s_out_l, sizeof(s_out_l));
	rc |= rbx_state_io(ctx, "gxs_outr", s_out_r, sizeof(s_out_r));
	if (load && !rc) {
		s_out_n = v[0] >= 0 && v[0] <= 2048 ? v[0] : 0;
		s_pub[0] = (uint8_t)v[1];
		s_pub[1] = (uint8_t)v[2];
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
	}
	return rc;
}
