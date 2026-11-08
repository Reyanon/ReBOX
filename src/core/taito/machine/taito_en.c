/*  Taito Ensoniq sound board, ported from MAME src/mame/taito/taito_en.cpp (BSD-3-Clause, copyright-holders:
    Bryan McPhail, Aaron Giles, R. Belmont, Philip Bennett). MC68681 after src/devices/machine/mc68681.cpp
    (BSD-3-Clause, copyright-holders: Mariusz Wojcieszek, R. Belmont, Joseph Zatarski); MB87078 after
    src/devices/sound/mb87077.cpp (BSD-3-Clause, copyright-holders: Fabio Priuli, Philip Bennett, hap);
    ES5510 host interface after src/devices/cpu/es5510/es5510.cpp (BSD-3-Clause, copyright-holders: Christian Brunschen).
    See taito_en.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "taito_en.h"
#include "../../rbx.h"
// the sound 68000 is its own Musashi instance (cpu/m68k_snd: the same sources, snd_ names), so that it can run on
// another thread: every m68k_* name below is that instance's
#include "../../cpu/m68k_snd/m68k_snd_names.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../sound/es5505.h"
#include "../../sound/es5510.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern void (*rbx_m68k_int_hook)(int level);

#define SND_CLOCK 15238095u     // 30.47618 MHz / 2: 68000 and ES5505
#define DUART_CLOCK 4000000u    // 16 MHz / 4

// ---- MC68681 (counter/timer, interrupt and vector; the serial channels only report "transmitter ready") ----
typedef struct {
	uint8_t ACR, IMR, ISR, OPCR, OPR, IVR, IPCR, half_period;
	uint16_t CTR;
	uint8_t tx_on[2], rx_on[2], mr_ptr[2], mr1[2], mr2[2], csr[2];
	uint8_t read_vector, irq_state, timer_on, pad;
	uint64_t expire;        // sound cycles << 16
	uint64_t period;        // one counter tick in sound cycles << 16
} duart_t;


static struct {
	ten_config cfg;
	uint8_t *rom;           // audiocpu region (host words), padded
	uint32_t rom_size;
	uint32_t bank_max;
	uint8_t bank_entry[3];
	uint8_t in_reset, release_pending, pad;
	uint8_t osram[0x10000];
	uint16_t otisbank[32];
	uint32_t bankmask;
	es5505_t es;            // the ES5505 the voices job plays (its thread)
	es5505_t esc;           // the emulation thread's copy: controls and accumulators only (es5505_sample_ctl)
	duart_t du;
	es5510_t esp;
	int esp_halted;         // pump: DUART OP6 (ESPHALT)
	// MB87078
	uint16_t mb_latch[4];
	uint8_t mb_data, mb_control;
	int32_t gain[2];        // left (channel 2), right (channel 3): output scale << 16
	// timing
	uint64_t now, target, next_sample;
	uint64_t frac;
	int in_exec;
	uint8_t ctx[1024];      // sound CPU Musashi context (save states)
	uint32_t audio_rate;
	uint32_t line_q, line_r;   // sound clocks per line: quotient, remainder (x frame rate x lines)
} s;

// The sound board runs TEN_LAG scanlines behind the main CPU (its line m after the main CPU's line m + TEN_LAG), so
// that it can run beside it (another thread with its own 68000). The dual-port RAM is then two copies: the sound
// side's (s_sdp, the RAM itself: the main CPU's writes of a line reach it before the sound board's line of that number,
// as they did when the two ran by turns) and the main CPU's (s_dpram: its own writes, and the sound board's up to line
// n - TEN_LAG - 1 before its line n; a byte the main CPU wrote at a later line than the sound write keeps the main
// CPU's value, as in the RAM's order). The main CPU's writes, reset line and bank changes go to the sound side in a
// queue with their line; the sound board's writes back in another. The same order whatever runs when: deterministic.
// (MAME runs the two CPUs by quanta of up to a frame on this board, with no synchronization at the RAM.)
#define TEN_LAG 16
static uint8_t s_dpram[0x1000];  // the main CPU's: host 16-bit word order (byte n at n ^ 1); 0x800 used, the rest fills
                                 // the main CPU page
static uint8_t s_sdp[0x800];     // the sound side's (byte n)
static uint32_t s_mtag[0x800];   // the main line each byte was last written by the main CPU
enum { M2S_DP, M2S_RESET, M2S_BANK };
typedef struct { uint32_t line; uint16_t idx; uint8_t kind, val; } ten_msg;
#define MQ_SIZE 4096
static ten_msg s_m2s[MQ_SIZE], s_s2m[MQ_SIZE];
// main lines done, the sound board's next line, the queue counters. The sound board is the third worker job (rbx_aux3:
// its own thread on the Vita): it runs the lines the main CPU has done, the main CPU waits only when it would need
// writes of a sound line not run yet (TEN_LAG lines of slack); the frame end, resets and save states sync with it.
// Each counter is written by one side (release) and read by the other (acquire)
static struct { uint32_t line, snd_line, m2s_in, m2s_out, s2m_in, s2m_out; } s_lag;
static struct { uint64_t end[32]; int n; } s_run = { { 0 }, 1 };   // the run of lines being done: their ends
#define LD(x) __atomic_load_n(&(x), __ATOMIC_ACQUIRE)
#define ST(x, v) __atomic_store_n(&(x), (v), __ATOMIC_RELEASE)

uint8_t *ten_dpram(void) { return s_dpram; }

static void m2s_push(int kind, int idx, int val)
{
	while (s_lag.m2s_in - LD(s_lag.m2s_out) >= MQ_SIZE) {   // (full: the sound board takes some)
		rbx_aux3_kick();
		rbx_wait();
	}
	ten_msg *m = &s_m2s[s_lag.m2s_in & (MQ_SIZE - 1)];
	m->line = s_lag.line;
	m->idx = (uint16_t)idx;
	m->kind = (uint8_t)kind;
	m->val = (uint8_t)val;
	ST(s_lag.m2s_in, s_lag.m2s_in + 1);
}

// a main CPU write of the RAM (68000 byte address offset a, 16-bit word data and byte mask)
void ten_dpram_w(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffe;
	for (int b = 0; b < 2; b++) {
		if (!(mask & (b ? 0x00ff : 0xff00)))
			continue;
		const uint32_t j = a + (uint32_t)b;
		const uint8_t d = (uint8_t)(b ? v : v >> 8);
		s_dpram[j ^ 1] = d;
		if (j < 0x800) {
			s_mtag[j] = s_lag.line;
			m2s_push(M2S_DP, (int)j, d);
		}
	}
}

// the sound board's writes up to its line m into the main CPU's copy
static void s2m_apply(uint32_t m)
{
	const uint32_t in = LD(s_lag.s2m_in);
	uint32_t out = s_lag.s2m_out;
	while (out != in && (int32_t)(s_s2m[out & (MQ_SIZE - 1)].line - m) <= 0) {
		const ten_msg *e = &s_s2m[out++ & (MQ_SIZE - 1)];
		if ((int32_t)(s_mtag[e->idx] - e->line) <= 0)
			s_dpram[e->idx ^ 1] = e->val;
	}
	ST(s_lag.s2m_out, out);
}

// the sound CPU's bus (its instance's m68k_bus_select)
static m68k_bus_t s_bus;

static uint64_t cur_time(void) { return s.now + (s.in_exec ? (uint64_t)m68k_cycles_run() : 0); }

// ---- audio ----
// The ES5510 runs as an rbx_aux job (a worker thread on the Vita): emit_until queues each sample's ES5505 outputs, the
// job runs the ESP over the queue, and the outputs go to rbx_audio in order at the frame end (esp_flush). The emulation
// side syncs with the job before it touches the ESP (host interface, reset, save states): same results as inline
#define EQ_SIZE 8192   // samples (a frame is ~ 500)
typedef struct { int16_t ser[6]; int32_t dry[2], out[2], gain[2]; uint8_t halted; } esp_sample;
static esp_sample s_eq[EQ_SIZE];
// The queue counters, each thread's on its own cache lines (a counter the other core writes all the time is a cache
// miss for each access; the emulation thread checks the room left only when the counter it last read says full)
static struct {
	unsigned eq_in;      // samples queued (published with release)
	unsigned eq_out;     // samples handed to rbx_audio
	unsigned hq_in;      // host writes queued (release)
	unsigned wq_in;      // ES5505 accesses queued (release)
	unsigned hq_room, wq_room;   // the in counts up to which the queues had room when last checked
	uint8_t pad[40];
} s_qe __attribute__((aligned(64)));
// what the jobs see of eq_in / hq_in / wq_in: published at a scanline's end and before any wait for a job (one
// release store a line, not one for each of the ~2600 ES5505 accesses a frame)
static struct { unsigned eq_in, hq_in, wq_in; uint8_t pad[52]; } s_qp __attribute__((aligned(64)));
static struct { unsigned eq_done, hq_done; uint8_t pad[56]; } s_q1 __attribute__((aligned(64)));   // the ESP job's
static struct { unsigned es_done, wq_done; uint8_t pad[56]; } s_q2 __attribute__((aligned(64)));   // the voices job's
#define s_eq_in s_qe.eq_in
#define s_eq_out s_qe.eq_out
#define s_hq_in s_qe.hq_in
#define s_wq_in s_qe.wq_in
#define s_eq_done s_q1.eq_done
#define s_hq_done s_q1.hq_done
#define s_es_done s_q2.es_done
#define s_wq_done s_q2.wq_done
// host interface writes of the sound CPU, applied by the job before the sample they precede (no wait)
#define HQ_SIZE 4096
typedef struct { unsigned at; uint8_t offset, data; } esp_host_write;
static esp_host_write s_hq[HQ_SIZE];
// The ES5505 voices run as the second job (rbx_aux2: another worker thread): the sound CPU's register accesses go to
// the emulation thread's copy (esc, which steps only the controls and accumulators per sample) and, queued with the
// sample they precede, to the job's ES5505 (es), which plays the samples. Both see the same accesses at the same
// samples, so their controls and accumulators stay the same; a read of the filters waits for the job
#define WQ_SIZE 8192
enum { ESW_WRITE, ESW_READ, ESW_BANK };
typedef struct { unsigned at; uint8_t kind, offset; uint16_t mask; uint32_t data; } es_access;
static es_access s_wq[WQ_SIZE];
// the host's view of the latches: what it wrote, and what the ESP put there (gpr / instr read by 0x80, dil read from the
// delay RAM by 0x0f) once the job has applied that command (stale until then: a read waits for the job)
static struct { int32_t gpr, dol, dil; uint32_t dadr; uint64_t instr; uint8_t ram_sel, stale_gpr, stale_dil; } s_hl;
// the general registers and the program as the host wrote them, and which registers the ESP program may have written
// since the last sync (a register no instruction writes holds what the host wrote: a 0x80 read of it needs no wait)
static struct { int32_t gpr[0xc0]; uint64_t instr[0xa0]; uint8_t esp_writes[0xc0]; } s_hp;

static void esp_mark_dests(uint64_t instr)
{
	int a, c;
	es5510_instr_dests(instr, &a, &c);
	if (a >= 0)
		s_hp.esp_writes[a] = 1;
	if (c >= 0)
		s_hp.esp_writes[c] = 1;
}

static void esp_host_apply(unsigned upto)
{
	const unsigned end = __atomic_load_n(&s_qp.hq_in, __ATOMIC_ACQUIRE);
	while (s_hq_done != end && (int)(s_hq[s_hq_done & (HQ_SIZE - 1)].at - upto) <= 0) {
		const esp_host_write *w = &s_hq[s_hq_done & (HQ_SIZE - 1)];
		es5510_host_w(&s.esp, w->offset, w->data);
		__atomic_store_n(&s_hq_done, s_hq_done + 1, __ATOMIC_RELEASE);
	}
}

// the queued ES5505 accesses up to sample upto (the voices job)
static void es_apply(unsigned upto)
{
	const unsigned end = __atomic_load_n(&s_qp.wq_in, __ATOMIC_ACQUIRE);
	while (s_wq_done != end && (int)(s_wq[s_wq_done & (WQ_SIZE - 1)].at - upto) <= 0) {
		const es_access *w = &s_wq[s_wq_done & (WQ_SIZE - 1)];
		switch (w->kind) {
		case ESW_WRITE: es5505_write(&s.es, w->offset, (uint16_t)w->data, w->mask); break;
		case ESW_READ: (void)es5505_read(&s.es, w->offset); break;   // (its side effects: the vector, O1(n-1))
		case ESW_BANK: s.es.bank[w->offset] = w->data; break;
		}
		__atomic_store_n(&s_wq_done, s_wq_done + 1, __ATOMIC_RELEASE);
	}
}

// the voices job: the ES5505 outputs of the queued samples, scaled by the MB87078 gains they were queued with (MAME
// routes: ES5505 outputs * 0.18 * MB87078 gain (percent / 32), 20-bit to 16-bit, into the pump, esqpump.cpp: channel
// 0 goes dry to pump outputs 2/3, channels 1-3 feed the ES5510 serial inputs)
static uint64_t s_esp_us, s_es_us;   // (profiling: the jobs' time)
static unsigned s_esp_n, s_sync_rd, s_sync_fe, s_lag_max;
static uint64_t s_lag_us;   // (profiling: the main CPU's wait for the sound board's lines, and the pieces drawn meanwhile)
static unsigned s_lag_help;
static void es_job_run(void);
static void es_job(void)
{
	const uint64_t t0 = rbx_clock_us();
	es_job_run();
	s_es_us += rbx_clock_us() - t0;
	// (the ESP takes the voices' new samples now: kicked with this job, it may have run first and found none, then
	// slept until the next kick, a frame later at the frame end: pbobble4 two frames behind, the frame end waiting)
	rbx_aux_kick();
}

static void es_job_run(void)
{
	const unsigned end = __atomic_load_n(&s_qp.eq_in, __ATOMIC_ACQUIRE);
	unsigned i = __atomic_load_n(&s_es_done, __ATOMIC_ACQUIRE);
	for (; i != end; i++) {
		es_apply(i);
		esp_sample *q = &s_eq[i & (EQ_SIZE - 1)];
		int32_t o[8], v[8];
		es5505_sample(&s.es, o);
		for (int k = 0; k < 8; k++)
			v[k] = (int32_t)(((int64_t)o[k] * q->gain[k & 1]) >> 16);
		for (int k = 0; k < 6; k++)
			q->ser[k] = (int16_t)v[2 + k];
		q->dry[0] = v[0];
		q->dry[1] = v[1];
		__atomic_store_n(&s_es_done, i + 1, __ATOMIC_RELEASE);
	}
	es_apply(end);
}

// both jobs done with all queued (the ESP's input comes from the voices job)
static void q_publish(void)
{
	__atomic_store_n(&s_qp.hq_in, s_hq_in, __ATOMIC_RELEASE);
	__atomic_store_n(&s_qp.wq_in, s_wq_in, __ATOMIC_RELEASE);
	__atomic_store_n(&s_qp.eq_in, s_eq_in, __ATOMIC_RELEASE);   // (each read with acquire by the jobs)
}

// the voices job done with all queued
static void es_sync(void)
{
	q_publish();
	rbx_aux2_sync();
}

static void esp_sync(void)
{
	es_sync();
	rbx_aux_sync();
}

extern unsigned es5510_interp_runs;
extern uint64_t rbx_aux_sync_us[3];
static void esp_job_run(void);
static void esp_job(void)
{
	const uint64_t t0 = rbx_clock_us();
	const unsigned n0 = s_eq_done;
	esp_job_run();
	s_esp_us += rbx_clock_us() - t0;
	s_esp_n += s_eq_done - n0;
}

static void esp_job_run(void)
{
	const unsigned end = __atomic_load_n(&s_es_done, __ATOMIC_ACQUIRE);
	unsigned i = __atomic_load_n(&s_eq_done, __ATOMIC_ACQUIRE);
	es5510_t *e = &s.esp;
	for (; i != end; i++) {
		esp_host_apply(i);
		esp_sample *q = &s_eq[i & (EQ_SIZE - 1)];
		e->ser0l = q->ser[0];
		e->ser0r = q->ser[1];
		e->ser1l = q->ser[2];
		e->ser1r = q->ser[3];
		e->ser2l = q->ser[4];
		e->ser2r = q->ser[5];
		if (!q->halted)
			es5510_run_once(e);
		q->out[0] = (q->dry[0] + e->ser3l) >> 1;
		q->out[1] = (q->dry[1] + e->ser3r) >> 1;
	}
	esp_host_apply(end);
	__atomic_store_n(&s_eq_done, i, __ATOMIC_RELEASE);
}

// the latches from the ESP (after a sync)
static void esp_host_refresh(void)
{
	s_hl.gpr = s.esp.gpr_latch;
	s_hl.dol = s.esp.dol_latch;
	s_hl.dil = s.esp.dil_latch;
	s_hl.dadr = s.esp.dadr_latch;
	s_hl.instr = s.esp.instr_latch;
	s_hl.ram_sel = s.esp.ram_sel;
	s_hl.stale_gpr = s_hl.stale_dil = 0;
	memcpy(s_hp.gpr, s.esp.gpr, sizeof(s_hp.gpr));
	memcpy(s_hp.instr, s.esp.instr, sizeof(s_hp.instr));
	memset(s_hp.esp_writes, 0, sizeof(s_hp.esp_writes));
	for (int i = 0; i < 0xa0; i++)
		esp_mark_dests(s_hp.instr[i]);
}

// sound CPU write to the ES5510 host interface: queued for the job, the latches kept here (es5510_host_w)
static void esp_host_w(int offset, uint8_t data)
{
	if (s_hq_in == s_qe.hq_room) {   // (room by the counter last read: read it again, wait if full)
		if (s_hq_in - __atomic_load_n(&s_hq_done, __ATOMIC_ACQUIRE) >= HQ_SIZE)
			esp_sync();
		s_qe.hq_room = __atomic_load_n(&s_hq_done, __ATOMIC_ACQUIRE) + HQ_SIZE;
	}
	esp_host_write *w = &s_hq[s_hq_in & (HQ_SIZE - 1)];
	w->at = s_eq_in;
	w->offset = (uint8_t)offset;
	w->data = data;
	s_hq_in++;
	switch (offset) {
	case 0x00: s_hl.gpr = (s_hl.gpr & 0x00ffff) | (data << 16); break;
	case 0x01: s_hl.gpr = (s_hl.gpr & 0xff00ff) | (data << 8); break;
	case 0x02: s_hl.gpr = (s_hl.gpr & 0xffff00) | data; break;
	case 0x03: case 0x04: case 0x05: case 0x06: case 0x07: case 0x08: {
		const int sh = (8 - offset) * 8;
		s_hl.instr = (s_hl.instr & ~((uint64_t)0xff << sh)) | ((uint64_t)data << sh);
		break;
	}
	case 0x0c: s_hl.dol = (s_hl.dol & 0x00ffff) | (data << 16); break;
	case 0x0d: s_hl.dol = (s_hl.dol & 0xff00ff) | (data << 8); break;
	case 0x0e: s_hl.dol = (s_hl.dol & 0xffff00) | data; break;
	case 0x0f:
		s_hl.dadr = (s_hl.dadr & 0x00ffff) | ((uint32_t)data << 16);
		if (s_hl.ram_sel)
			s_hl.stale_dil = 1;
		break;
	case 0x10: s_hl.dadr = (s_hl.dadr & 0xff00ff) | ((uint32_t)data << 8); break;
	case 0x11: s_hl.dadr = (s_hl.dadr & 0xffff00) | data; break;
	case 0x14: s_hl.ram_sel = data & 0x80; break;
	case 0x80:   // read into the latches (es5510_host_w 0x80)
		if (data < 0xc0 && !s_hp.esp_writes[data]) {
			if (data < 0xa0)
				s_hl.instr = s_hp.instr[data];
			s_hl.gpr = s_hp.gpr[data] & 0xffffff;
		} else if (data < 0xc0 || data >= 0xea)
			s_hl.stale_gpr = 1;   // (the ESP's own value: wait for it)
		break;
	case 0xa0:   // write_reg
		if (data < 0xc0)
			s_hp.gpr[data] = s_hl.gpr & 0x00ffffff;
		break;
	case 0xc0:
		if (data < 0xa0) {
			s_hp.instr[data] = s_hl.instr & 0xffffffffffffull;
			esp_mark_dests(s_hp.instr[data]);
		}
		break;
	case 0xe0:
		if (data < 0xa0) {
			s_hp.instr[data] = s_hl.instr;
			esp_mark_dests(s_hp.instr[data]);
		}
		if (data < 0xc0)
			s_hp.gpr[data] = s_hl.gpr & 0x00ffffff;
		break;
	}
}

static uint8_t esp_host_r(int offset)
{
	if ((s_hl.stale_gpr && offset <= 0x08) || (s_hl.stale_dil && (offset == 0x09 || offset == 0x0a))) {
		s_sync_rd++;
		esp_sync();
		esp_host_refresh();
	}
	switch (offset) {   // es5510_host_r on the latches
	case 0x00: return (uint8_t)(s_hl.gpr >> 16);
	case 0x01: return (uint8_t)(s_hl.gpr >> 8);
	case 0x02: return (uint8_t)s_hl.gpr;
	case 0x03: return (uint8_t)(s_hl.instr >> 40);
	case 0x04: return (uint8_t)(s_hl.instr >> 32);
	case 0x05: return (uint8_t)(s_hl.instr >> 24);
	case 0x06: return (uint8_t)(s_hl.instr >> 16);
	case 0x07: return (uint8_t)(s_hl.instr >> 8);
	case 0x08: return (uint8_t)s_hl.instr;
	case 0x09: return (uint8_t)(s_hl.dil >> 16);
	case 0x0a: return (uint8_t)(s_hl.dil >> 8);
	case 0x0c: return (uint8_t)(s_hl.dol >> 16);
	case 0x0d: return (uint8_t)(s_hl.dol >> 8);
	case 0x0e: return 0xff;
	case 0x0f: return (uint8_t)(s_hl.dadr >> 16);
	case 0x10: return (uint8_t)(s_hl.dadr >> 8);
	case 0x11: return (uint8_t)s_hl.dadr;
	case 0x16: return 0x27;
	}
	return 0;
}

// the samples the ESP job has run, out to rbx_audio in order
static void esp_out(void)
{
	const unsigned done = __atomic_load_n(&s_eq_done, __ATOMIC_ACQUIRE);
	for (; s_eq_out != done; s_eq_out++) {
		const esp_sample *q = &s_eq[s_eq_out & (EQ_SIZE - 1)];
		rbx_audio_put_stereo(q->out[0], q->out[1]);
	}
}

// all queued samples through the ESP and out to rbx_audio
static void esp_flush(void)
{
	esp_sync();
	esp_out();
}

static void emit_until(uint64_t t)
{
	while (s.next_sample <= t) {
		es5505_sample_ctl(&s.esc);
		if (s_eq_in - s_eq_out >= EQ_SIZE)
			esp_flush();
		esp_sample *q = &s_eq[s_eq_in & (EQ_SIZE - 1)];
		q->gain[0] = s.gain[0];
		q->gain[1] = s.gain[1];
		q->halted = (uint8_t)s.esp_halted;
		s_eq_in++;
		s.next_sample += (uint64_t)es5505_sample_clocks(&s.esc);
	}
}

// a sound CPU access of the ES5505 for the voices job, before the next sample
static void es_queue(int kind, int offset, uint32_t data, uint16_t mask)
{
	if (s_wq_in == s_qe.wq_room) {
		if (s_wq_in - __atomic_load_n(&s_wq_done, __ATOMIC_ACQUIRE) >= WQ_SIZE)
			es_sync();
		s_qe.wq_room = __atomic_load_n(&s_wq_done, __ATOMIC_ACQUIRE) + WQ_SIZE;
	}
	es_access *w = &s_wq[s_wq_in & (WQ_SIZE - 1)];
	w->at = s_eq_in;
	w->kind = (uint8_t)kind;
	w->offset = (uint8_t)offset;
	w->mask = mask;
	w->data = data;
	s_wq_in++;
}

static uint16_t es_read(int offset)
{
	if (es5505_read_needs_sound(&s.esc, offset)) {   // (the filters: the job's ES5505, caught up)
		es_sync();
		return es5505_read(&s.es, offset);
	}
	es_queue(ESW_READ, offset, 0, 0);
	return es5505_read(&s.esc, offset);
}

static void es_write(int offset, uint16_t data, uint16_t mask)
{
	es_queue(ESW_WRITE, offset, data, mask);
	es5505_write(&s.esc, offset, data, mask);
}

static inline void sync_audio(void)
{
	const uint64_t t = cur_time();
	if (s.next_sample <= t)
		emit_until(t);
}

// ---- MB87078 ----
// gain of each MB87078 step: 0.18 * (pct / 32) * 2^-4 (20 to 16 bits) in 16.16, made once (the game writes the volume
// often and pow() is slow on the Vita)
static int32_t s_mb_gain[66];

static void mb_recalc(void)
{
	if (!s_mb_gain[0])
		for (int idx = 0; idx < 66; idx++) {
			double g = idx == 65 ? 0.0 : pow(10.0, (-0.5 * idx) / 20.0);
			int pct = (int)(g * 100.0 + 0.5);
			s_mb_gain[idx] = (int32_t)((0.18 * pct / 32.0 / 16.0) * 65536.0 + 0.5);
		}
	for (int ch = 2; ch < 4; ch++) {
		uint16_t l = s.mb_latch[ch];
		int idx;
		if (~l & 0x40) idx = 65;
		else if (l & 0x100) idx = 64;
		else if (l & 0x80) idx = 0;
		else idx = ~l & 0x3f;
		s.gain[ch - 2] = s_mb_gain[idx];
	}
}

static void mb_write(int offset, uint8_t data)
{
	if (offset & 1)
		s.mb_control = data & 0x1f;
	else {
		s.mb_data = data & 0x3f;
		s.mb_latch[s.mb_control & 3] = (uint16_t)((s.mb_control << 4 & 0x1c0) | s.mb_data);
		mb_recalc();
	}
}

// ---- DUART ----
// MAME taito_en duart_output: OP6 (inverted on the pins) is ESPHALT for the pump
static void esp_output(void)
{
	int halted = !(s.du.OPR & 0x40);
	if (halted != s.esp_halted) {
		sync_audio();
		s.esp_halted = halted;
	}
}

static void duart_irq_update(void)
{
	duart_t *d = &s.du;
	uint8_t isr = d->ISR & ~0x11;
	if (d->tx_on[0]) isr |= 0x01;   // TxRDYA
	if (d->tx_on[1]) isr |= 0x10;   // TxRDYB
	d->ISR = isr;
	int irq = (d->ISR & d->IMR) != 0;
	if (irq != d->irq_state) {
		d->irq_state = (uint8_t)irq;
		if (s.in_exec)
			m68k_end_timeslice();   // the new line level is applied between instructions
	}
	if (!irq)
		d->read_vector = 0;
	if (d->OPCR & 0xf0) {   // OP4-OP7 can follow interrupt status bits
		if (d->OPCR & 0x10) d->OPR = (uint8_t)((d->OPR & ~0x10) | ((d->ISR & 0x02) ? 0x10 : 0));
		if (d->OPCR & 0x20) d->OPR = (uint8_t)((d->OPR & ~0x20) | ((d->ISR & 0x20) ? 0x20 : 0));
		if (d->OPCR & 0x40) d->OPR = (uint8_t)((d->OPR & ~0x40) | ((d->ISR & 0x01) ? 0x40 : 0));
		if (d->OPCR & 0x80) d->OPR = (uint8_t)((d->OPR & ~0x80) | ((d->ISR & 0x10) ? 0x80 : 0));
		esp_output();
	}
}

static uint32_t ct_rate(void)
{
	duart_t *d = &s.du;
	int mode = (d->ACR >> 4) & 3;
	if (d->ACR & 0x40)
		return mode == 3 ? DUART_CLOCK / 16 : DUART_CLOCK;
	return mode == 0 ? DUART_CLOCK : DUART_CLOCK / 16;   // TxCA / TxCB: no serial clocks here
}

static void start_ct(int count)
{
	duart_t *d = &s.du;
	d->period = ((uint64_t)SND_CLOCK << 16) / ct_rate();
	d->expire = (cur_time() << 16) + (uint64_t)count * d->period;
	d->timer_on = 1;
}

static void duart_timer_fire(void)
{
	duart_t *d = &s.du;
	uint64_t base = d->expire;
	if (d->ACR & 0x40) {
		d->half_period ^= 1;
		if ((d->OPCR & 0xc) == 0x4)
			d->OPR ^= 0x8;
		if (!d->half_period) {
			d->ISR |= 0x08;
			duart_irq_update();
		}
		int count = d->CTR ? d->CTR : 1;
		d->expire = base + (uint64_t)count * d->period;
	} else {
		if ((d->OPCR & 0xc) == 0x4)
			d->OPR |= 0x8;
		d->ISR |= 0x08;
		duart_irq_update();
		d->expire = base + 0xffffull * d->period;
	}
}

static uint8_t duart_read(int offset)
{
	duart_t *d = &s.du;
	uint8_t r = 0xff;
	switch (offset & 0xf) {
	case 0x00: case 0x08: {
		int ch = offset >> 3;
		r = d->mr_ptr[ch] ? d->mr2[ch] : d->mr1[ch];
		d->mr_ptr[ch] = 1;
		break;
	}
	case 0x01: case 0x09: {
		int ch = offset >> 3;
		r = d->tx_on[ch] ? 0x0c : 0x00;
		break;
	}
	case 0x03: case 0x0b: r = 0; break;
	case 0x04:
		r = d->IPCR;
		d->IPCR &= 0x0f;
		d->ISR &= ~0x80;
		duart_irq_update();
		break;
	case 0x05: r = d->ISR; break;
	case 0x06: case 0x07: {
		uint64_t now = cur_time() << 16;
		uint32_t cnt = 0;
		if (d->timer_on && d->expire > now && d->period)
			cnt = (uint32_t)((d->expire - now) / d->period);
		r = (uint8_t)((offset & 0xf) == 6 ? cnt >> 8 : cnt);
		break;
	}
	case 0x0a: r = 0x61; break;
	case 0x0c: r = d->IVR; break;
	case 0x0d:
		r = 0x80 | 0x03;   // IP0/IP1 tied to 5V
		if (d->read_vector) r &= ~0x40; else r |= 0x40;
		break;
	case 0x0e:
		if (d->ACR & 0x40)
			d->half_period = 0;
		start_ct(d->CTR ? d->CTR : 1);
		break;
	case 0x0f:
		if (!(d->ACR & 0x40)) {
			d->timer_on = 0;
			if ((d->OPCR & 0xc) == 0x4)
				d->OPR &= ~0x8;
		}
		d->ISR &= ~0x08;
		duart_irq_update();
		break;
	}
	return r;
}

static void duart_write(int offset, uint8_t data)
{
	duart_t *d = &s.du;
	switch (offset & 0xf) {
	case 0x00: case 0x08: {
		int ch = offset >> 3;
		if (d->mr_ptr[ch]) d->mr2[ch] = data; else d->mr1[ch] = data;
		d->mr_ptr[ch] = 1;
		break;
	}
	case 0x01: case 0x09: d->csr[offset >> 3] = data; break;
	case 0x02: case 0x0a: {
		int ch = offset >> 3;
		if (data & 0x01) d->rx_on[ch] = 1;
		if (data & 0x02) d->rx_on[ch] = 0;
		if (data & 0x04) d->tx_on[ch] = 1;
		if (data & 0x08) d->tx_on[ch] = 0;
		switch ((data >> 4) & 7) {
		case 1: d->mr_ptr[ch] = 0; break;
		case 2: d->rx_on[ch] = 0; break;
		case 3: d->tx_on[ch] = 0; break;
		}
		duart_irq_update();
		break;
	}
	case 0x03: case 0x0b: break;   // transmit: nothing listens
	case 0x04: {
		uint8_t old = d->ACR;
		d->ACR = data;
		if ((old ^ data) & 0x40) {
			if (data & 0x40) {
				d->half_period = 0;
				start_ct(d->CTR ? d->CTR : 1);
			} else
				d->timer_on = 0;
		}
		if ((((d->IPCR >> 4) & data) & 0x0f) != 0)
			d->ISR |= 0x80;
		duart_irq_update();
		break;
	}
	case 0x05: d->IMR = data; duart_irq_update(); break;
	case 0x06: d->CTR = (uint16_t)((d->CTR & 0x00ff) | data << 8); break;
	case 0x07: d->CTR = (uint16_t)((d->CTR & 0xff00) | data); break;
	case 0x0c: d->IVR = data; break;
	case 0x0d: d->OPCR = data; break;
	case 0x0e: d->OPR |= data; esp_output(); break;
	case 0x0f: d->OPR &= ~data; esp_output(); break;
	}
}

static void duart_reset(void)
{
	memset(&s.du, 0, sizeof(s.du));
	s.du.IVR = 0x0f;
	s.du.period = ((uint64_t)SND_CLOCK << 16) / DUART_CLOCK;
}


// ---- sound CPU map ----
static void bank_map(int i)
{
	static const uint32_t start[3] = { 0xc00000, 0xc20000, 0xc40000 };
	static const uint32_t len[3] = { 0x20000, 0x20000, 0x40000 };
	uint8_t *p = s.rom + 0x100000 + (uint32_t)s.bank_entry[i] * 0x20000;
	for (uint32_t a = start[i]; a < start[i] + len[i]; a += 0x1000) {
		uint8_t *pg = p + (a - start[i]);
		s_bus.rd[a >> 12] = pg;
	}
}

static uint32_t run_line(void);
static void snd_dp_w(uint32_t j, uint8_t v)
{
	s_sdp[j] = v;
	while (s_lag.s2m_in - LD(s_lag.s2m_out) >= MQ_SIZE)   // (full: the main CPU takes them at its lines)
		rbx_wait();
	ten_msg *m = &s_s2m[s_lag.s2m_in & (MQ_SIZE - 1)];
	m->line = run_line();
	m->idx = (uint16_t)j;
	m->kind = M2S_DP;
	m->val = v;
	ST(s_lag.s2m_in, s_lag.s2m_in + 1);
}

static uint16_t snd_read16(uint32_t a)
{
	if (a >= 0x140000 && a <= 0x140fff) return (uint16_t)(s_sdp[(a - 0x140000) >> 1] << 8);
	if (a >= 0x200000 && a <= 0x20001f) { sync_audio(); return es_read((a >> 1) & 0xf); }
	if (a >= 0x260000 && a <= 0x2601ff) { sync_audio(); return esp_host_r((int)((a - 0x260000) >> 1)); }
	if (a >= 0x280000 && a <= 0x28001f) return duart_read((int)((a >> 1) & 0xf));
	if (a == 0xfffffc) return s.du.IVR;
	return 0;
}

static uint8_t snd_read8(uint32_t a)
{
	if (a >= 0x140000 && a <= 0x140fff) return (a & 1) ? 0 : s_sdp[(a - 0x140000) >> 1];
	if (a >= 0x200000 && a <= 0x20001f) { sync_audio(); uint16_t v = es_read((a >> 1) & 0xf); return (uint8_t)((a & 1) ? v : v >> 8); }
	if (a >= 0x260000 && a <= 0x2601ff) { sync_audio(); return (a & 1) ? esp_host_r((int)((a - 0x260000) >> 1)) : 0; }
	if (a >= 0x280000 && a <= 0x28001f) return (a & 1) ? duart_read((int)((a >> 1) & 0xf)) : 0;
	return 0;
}

static void snd_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if ((a & ~0x1fu) == 0x200000) { sync_audio(); es_write((a >> 1) & 0xf, v, mask); return; }   // (the most written)
	if (a >= 0x140000 && a <= 0x140fff) { if (mask & 0xff00) snd_dp_w((a - 0x140000) >> 1, (uint8_t)(v >> 8)); return; }
	if (a >= 0x260000 && a <= 0x2601ff) { if (mask & 0xff) { sync_audio(); esp_host_w((int)((a - 0x260000) >> 1), (uint8_t)v); } return; }
	if (a >= 0x280000 && a <= 0x28001f) { if (mask & 0xff) duart_write((int)((a >> 1) & 0xf), (uint8_t)v); return; }
	if (a >= 0x300000 && a <= 0x30003f) {
		int o = (int)((a - 0x300000) >> 1);
		sync_audio();
		s.otisbank[o] = v;
		s.esc.bank[o] = (uint32_t)(s.otisbank[o] & s.bankmask) << 20;
		es_queue(ESW_BANK, o, s.esc.bank[o], 0);
		return;
	}
	if (a >= 0x340000 && a <= 0x340003) { if (mask & 0xff00) { sync_audio(); mb_write((int)(((a - 0x340000) >> 1) ^ 1), (uint8_t)(v >> 8)); } return; }
}

static void snd_write16(uint32_t a, uint16_t v) { snd_write16m(a, v, 0xffff); }
static void snd_write8(uint32_t a, uint8_t v)
{
	if (a & 1)
		snd_write16m(a & ~1u, v, 0x00ff);
	else
		snd_write16m(a, (uint16_t)(v << 8), 0xff00);
}

// the DUART interrupt is vectored (MAME fc7_map reads the vector from the DUART); Musashi has taken the level 6
// autovector, so redirect to the DUART vector here
static void snd_int_hook(int level)
{
	if (level == 6 && s.du.irq_state) {
		s.du.read_vector = 1;
		uint32_t vec = s.du.IVR;
		uint32_t pc = m68k_read_memory_32(vec << 2);
		if (pc == 0)
			pc = m68k_read_memory_32(15 << 2);
		m68k_set_reg(M68K_REG_PC, pc);
		// line still asserted (level triggered): the interrupt mask keeps it from being retaken
		m68k_set_irq(6);
	}
}


// ---- public ----
static void snd_job(void);
static void snd_quiesce(void);
int ten_init(const ten_config *cfg)
{
	uint32_t rsize, esize;
	uint8_t *rom = rbx_region("taito_en:audiocpu", &rsize);
	uint8_t *ens = rbx_region("taito_en:ensoniq", &esize);
	if (!rom || !ens || rsize < 0x120000) {
		rbx_fail("sound board ROM regions missing");
		return -1;
	}
	rbx_aux3_set_job(NULL);
	rbx_aux_set_job(NULL);
	rbx_aux2_set_job(NULL);
	memset(&s, 0, sizeof(s));
	memset(&s_qe, 0, sizeof(s_qe));
	memset(&s_q1, 0, sizeof(s_q1));
	memset(&s_q2, 0, sizeof(s_q2));
	memset(&s_qp, 0, sizeof(s_qp));
	memset(&s_lag, 0, sizeof(s_lag));
	if (m68k_context_size() > sizeof(s.ctx)) {
		rbx_fail("m68k context too large");
		return -1;
	}
	s.cfg = *cfg;
	s.rom_size = rsize;
	s.rom = (uint8_t *)calloc(1, rsize + 0x40000);
	if (!s.rom || es5510_init(&s.esp)) {
		ten_exit();
		rbx_fail("out of memory (sound)");
		return -1;
	}
	memcpy(s.rom, rom, rsize);
	m68k_bus_swap16(s.rom, rsize);
	s.bank_max = (rsize - 0x100000) / 0x20000;
	s.bankmask = esize / 0x200000 ? esize / 0x200000 - 1 : 0;
	es5505_init(&s.es, ens, esize);
	s.esc = s.es;
	es5505_ctl_begin(&s.esc);
	rbx_aux2_set_job(es_job);
	rbx_aux_set_job(esp_job);
	rbx_aux3_set_job(snd_job);

	// sound CPU (its instance: tables for the 68000) and map (MAME en_sound_map); banks by bank_map
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	rbx_m68k_int_hook = snd_int_hook;
	memset(&s_bus, 0, sizeof(s_bus));
	m68k_bus_select(&s_bus);
	m68k_bus_map(0x000000, 0x03ffff, s.osram, sizeof(s.osram), 1, 1);
	m68k_bus_map(0xff0000, 0xffffff, s.osram, sizeof(s.osram), 1, 1);
	s_bus.read8 = snd_read8;
	s_bus.read16 = snd_read16;
	s_bus.write8 = snd_write8;
	s_bus.write16 = snd_write16;
	ten_reset(1);
	return 0;
}

void ten_exit(void)
{
	rbx_aux3_set_job(NULL);
	rbx_aux_set_job(NULL);
	rbx_aux2_set_job(NULL);
	free(s.rom);
	es5510_exit(&s.esp);
	s.rom = NULL;
}

void ten_reset(int hard)
{
	snd_quiesce();
	if (hard) {   // power on; a soft reset (watchdog) keeps the RAM as MAME does
		memset(s.osram, 0, sizeof(s.osram));
		memset(s_dpram, 0, sizeof(s_dpram));
		memset(s_sdp, 0, sizeof(s_sdp));
		memset(s.otisbank, 0, sizeof(s.otisbank));
	} else {   // (the RAM as the queued writes leave it, both copies)
		while (s_lag.m2s_out != s_lag.m2s_in) {
			const ten_msg *e = &s_m2s[s_lag.m2s_out++ & (MQ_SIZE - 1)];
			if (e->kind == M2S_DP)
				s_sdp[e->idx] = e->val;
		}
		for (int j = 0; j < 0x800; j++)
			s_dpram[j ^ 1] = s_sdp[j];
	}
	// (the sound board starts again TEN_LAG lines behind from here)
	s_lag.m2s_out = s_lag.m2s_in;
	s_lag.s2m_out = s_lag.s2m_in;
	s_lag.snd_line = s_lag.line;
	for (int j = 0; j < 0x800; j++)
		s_mtag[j] = s_lag.line - 1;
	for (int i = 0; i < 3; i++) {
		s.bank_entry[i] = (uint8_t)(s.bank_max ? i % s.bank_max : 0);
		bank_map(i);
	}
	// stack and reset vectors from the program ROM
	memcpy(s.osram, s.rom + 0x100000, 8);
	esp_flush();
	es5505_reset(&s.es);
	es5505_reset(&s.esc);
	duart_reset();
	es5510_reset(&s.esp);
	esp_host_refresh();
	s.esp_halted = 1;
	for (int i = 0; i < 4; i++)
		s.mb_latch[i] = 0x7f;
	s.mb_data = s.mb_control = 0;
	mb_recalc();
	s.now = s.target = s.next_sample = 0;
	s.frac = 0;
	s.in_reset = 1;
	s.release_pending = 0;
	m68k_pulse_reset();
	m68k_set_irq(0);
}

static void reset_line(int asserted)
{
	if (asserted)
		s.in_reset = 1;
	else if (s.in_reset) {
		s.in_reset = 0;
		s.release_pending = 1;
	}
}

static void set_bank(int bank, int entry)
{
	if (bank < 0 || bank > 2 || !s.bank_max)
		return;
	s.bank_entry[bank] = (uint8_t)(entry % s.bank_max);
	bank_map(bank);
}

void ten_set_reset_line(int asserted) { m2s_push(M2S_RESET, 0, asserted); }
void ten_set_bank(int bank, int entry) { m2s_push(M2S_BANK, bank, entry); }

// the main CPU's writes and line changes up to its line m, before the sound board's line m
static void m2s_apply(uint32_t m)
{
	const uint32_t in = LD(s_lag.m2s_in);
	uint32_t out = s_lag.m2s_out;
	while (out != in && (int32_t)(s_m2s[out & (MQ_SIZE - 1)].line - m) <= 0) {
		const ten_msg *e = &s_m2s[out++ & (MQ_SIZE - 1)];
		switch (e->kind) {
		case M2S_DP: s_sdp[e->idx] = e->val; break;
		case M2S_RESET: reset_line(e->val); break;
		case M2S_BANK: set_bank(e->idx, e->val); break;
		}
		ST(s_lag.m2s_out, out);
	}
}

void ten_frame_begin(void)
{
	es5510_jit_install();
	uint32_t rate = SND_CLOCK / (uint32_t)es5505_sample_clocks(&s.esc);
	if (rate != s.audio_rate || !rbx_audio.stereo || rbx_audio.rate != rate) {
		rbx_audio_setup_stereo(rate);
		s.audio_rate = rate;
	}
}

static void snd_lines(int count);

// the sound board's job: its lines up to the main CPU's last one done, in runs of lines between the main CPU's writes
// (a run is one slice of the 68000: the same as line by line, each write of the RAM tagged with the line its
// instruction started in)
#define RUN_MAX 32
static void snd_job(void)
{
	const uint32_t target = LD(s_lag.line);
	uint32_t m = s_lag.snd_line;
	while (m != target) {
		m2s_apply(m);
		uint32_t lim = target;
		const uint32_t out = s_lag.m2s_out;
		if (out != LD(s_lag.m2s_in) && (int32_t)(s_m2s[out & (MQ_SIZE - 1)].line - lim) < 0)
			lim = s_m2s[out & (MQ_SIZE - 1)].line;   // (the next line with something from the main CPU)
		int count = (int)(lim - m);
		if (count > RUN_MAX)
			count = RUN_MAX;
		snd_lines(count);
		m += (uint32_t)count;
		ST(s_lag.snd_line, m);
	}
}

void ten_line(void)
{
	// the main CPU's line n done: the sound board may run it; the main CPU's line n + 1 takes the sound board's writes
	// up to its line n - TEN_LAG (waiting for that line if it has not run yet)
	const uint32_t n = s_lag.line;
	ST(s_lag.line, n + 1);
	// (a kick every 16 lines: the job runs all the lines done, in runs between the main CPU's writes; with its thread
	// a nudge every 4 too: its waking took up most of the 16 lines of slack, the main CPU waited 4.6 ms a frame, pbobble3.
	// Without the thread the kicks run the job here: as before, the runs the same)
	if ((n & 15) == 15)
		rbx_aux3_kick();
	else if ((n & 3) == 3)
		rbx_aux3_nudge();
	const uint32_t need = n - TEN_LAG;
	if ((int32_t)(LD(s_lag.snd_line) - need) <= 0) {
		const uint64_t t0 = rbx_clock_us();
		rbx_aux3_kick();
		while ((int32_t)(LD(s_lag.snd_line) - need) <= 0)
			s_lag_help += (unsigned)rbx_render_help_one(1);   // (draw meanwhile)
		s_lag_us += rbx_clock_us() - t0;
	}
	s2m_apply(need);
}

// the sound board idle: all the main CPU's lines run (it touches nothing after its last line until the next kick)
static void snd_quiesce(void)
{
	rbx_aux3_kick();
	while (LD(s_lag.snd_line) != LD(s_lag.line))
		(void)rbx_render_help_one(1);
}

// the line of the current instruction in the run (snd_dp_w's tag)
static uint32_t run_line(void)
{
	const uint64_t t = cur_time();
	int k = 0;
	while (k < s_run.n - 1 && t >= s_run.end[k])
		k++;
	return s_lag.snd_line + (uint32_t)k;
}

static void snd_lines(int count)
{
	// rbx_line_cycles without the 64-bit division per line (same sequence); each line's end in the run
	const uint64_t den = (uint64_t)s.cfg.frame_rate_mhz * (uint64_t)s.cfg.lines;
	if (!s.line_q) {
		s.line_q = (uint32_t)(((uint64_t)SND_CLOCK * 1000) / den);
		s.line_r = (uint32_t)(((uint64_t)SND_CLOCK * 1000) % den);
	}
	for (int k = 0; k < count; k++) {
		s.frac += s.line_r;
		int lc = (int)s.line_q;
		if (s.frac >= den) {
			s.frac -= den;
			lc++;
		}
		s.target += (uint64_t)lc;
		s_run.end[k] = s.target;
	}
	s_run.n = count;
	if (s.release_pending) {
		s.release_pending = 0;
		m68k_pulse_reset();
	}
	while (s.now < s.target) {
		uint64_t end = s.target;
		if (s.du.timer_on) {
			uint64_t t = (s.du.expire + 0xffff) >> 16;
			if (t <= s.now)
				t = s.now + 1;
			if (t < end)
				end = t;
		}
		if (s.in_reset)
			s.now = end;
		else {
			m68k_set_irq(s.du.irq_state ? 6 : 0);
			s.in_exec = 1;
			const int used = m68k_execute((int)(end - s.now));
			s.in_exec = 0;
			s.now += (uint64_t)used;
		}
		while (s.du.timer_on && s.du.expire <= (s.now << 16))
			duart_timer_fire();
	}
	if (!s.in_reset)
		m68k_set_irq(s.du.irq_state ? 6 : 0);
	s_run.n = 1;
	emit_until(s.now);
	q_publish();
	rbx_aux2_kick();
	rbx_aux_kick();
}

void ten_frame_end(void)
{
	snd_quiesce();   // (the sound board idle until the next frame's first line: the queues and rbx_audio are ours)
	// what the ESP has done so far: with the worker thread the rest goes out with the next frame (the sound is up to a
	// frame later, the samples the same), so the ESP only has to keep up, not to finish within the frame
	q_publish();
	rbx_aux2_kick();
	rbx_aux_kick();
	// (the workers more than two frames behind: catch up here rather than let the queues fill up and stall much longer)
	{
		const unsigned lag = s_eq_in - __atomic_load_n(&s_eq_done, __ATOMIC_ACQUIRE);
		if (lag > s_lag_max)
			s_lag_max = lag;
	}
	if (s_eq_in - __atomic_load_n(&s_eq_done, __ATOMIC_ACQUIRE) > 2 * (s.audio_rate / 60)) {
		s_sync_fe++;
		esp_sync();
	}
	{
		static unsigned f;
		if (++f % 300 == 0) {
			rbx_log("ten: esp %.2f ms/frame (%u samples, interpreted %u), voices %.2f ms/frame; syncs: reads %u, frame "
			        "ends %u; lag max %u; sound board wait %.2f ms/frame (%u pieces drawn); syncs %.2f %.2f %.2f ms/frame",
			        s_esp_us / 300000.0, s_esp_n, es5510_interp_runs, s_es_us / 300000.0, s_sync_rd, s_sync_fe, s_lag_max,
			        s_lag_us / 300000.0, s_lag_help, rbx_aux_sync_us[0] / 300000.0, rbx_aux_sync_us[1] / 300000.0,
			        rbx_aux_sync_us[2] / 300000.0);
			rbx_aux_sync_us[0] = rbx_aux_sync_us[1] = rbx_aux_sync_us[2] = 0;
			s_lag_us = 0;
			s_lag_help = 0;
			s_esp_us = s_es_us = 0;
			s_esp_n = es5510_interp_runs = s_sync_rd = s_sync_fe = s_lag_max = 0;
		}
	}
	esp_out();
}

int ten_state(void *ctx, int load)
{
	snd_quiesce();
	int rc = 0;
	esp_flush();
	es5505_t es = s.es;
	m68k_get_context(s.ctx);
	rc |= rbx_state_io(ctx, "en_cpu", s.ctx, m68k_context_size());
	rc |= rbx_state_io(ctx, "en_ram", s.osram, sizeof(s.osram));
	rc |= rbx_state_io(ctx, "en_dpram", s_dpram, sizeof(s_dpram));
	rc |= rbx_state_io(ctx, "en_otisbank", s.otisbank, sizeof(s.otisbank));
	rc |= rbx_state_io(ctx, "en_banks", s.bank_entry, sizeof(s.bank_entry));
	rc |= rbx_state_io(ctx, "en_es5505", &es, ES5505_STATE_SIZE);
	rc |= rbx_state_io(ctx, "en_duart", &s.du, sizeof(s.du));
	int16_t *dram = s.esp.dram;
	rc |= rbx_state_io(ctx, "en_esp", &s.esp, sizeof(s.esp));
	s.esp.dram = dram;
	rc |= rbx_state_io(ctx, "en_esp_dram", dram, ES5510_DRAM_SIZE * sizeof(int16_t));
	rc |= rbx_state_io(ctx, "en_esp_halt", &s.esp_halted, sizeof(s.esp_halted));
	rc |= rbx_state_io(ctx, "en_mb", s.mb_latch, sizeof(s.mb_latch));
	rc |= rbx_state_io(ctx, "en_now", &s.now, sizeof(s.now));
	rc |= rbx_state_io(ctx, "en_target", &s.target, sizeof(s.target));
	rc |= rbx_state_io(ctx, "en_next", &s.next_sample, sizeof(s.next_sample));
	rc |= rbx_state_io(ctx, "en_frac", &s.frac, sizeof(s.frac));
	rc |= rbx_state_io(ctx, "en_reset", &s.in_reset, 2);
	rc |= rbx_state_io(ctx, "en_sdp", s_sdp, sizeof(s_sdp));
	rc |= rbx_state_io(ctx, "en_mtag", s_mtag, sizeof(s_mtag));
	rc |= rbx_state_io(ctx, "en_m2s", s_m2s, sizeof(s_m2s));
	rc |= rbx_state_io(ctx, "en_s2m", s_s2m, sizeof(s_s2m));
	rc |= rbx_state_io(ctx, "en_lag", &s_lag, sizeof(s_lag));
	if (rc)
		return -1;
	if (load) {
		const uint8_t *rom = s.es.rom;
		uint32_t mask = s.es.rom_mask;
		s.es = es;
		s.es.rom = rom;
		s.es.rom_mask = mask;
		s.esc = s.es;
		es5505_ctl_begin(&s.esc);
		m68k_set_context(s.ctx);
		for (int i = 0; i < 3; i++)
			bank_map(i);
		mb_recalc();
		es5510_invalidate(&s.esp);
		esp_host_refresh();
	}
	return 0;
}
