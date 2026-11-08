/*  The Z80 sound board of the Irem 16-bit boards (irem_zsnd.h). Ported from MAME src/mame/irem/m72.cpp, m72_a.cpp, m90.cpp
    (BSD-3-Clause, copyright-holders: Nicola Salmoria, Bryan McPhail) and src/devices/machine/rstbuf.cpp (BSD-3-Clause,
    copyright-holders: AJR).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "irem_zsnd.h"
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define Z80_CLOCK 3579545u
#define OUT_MAX 2048

static z80 s_z;
static const uint8_t *s_zmap[256];
static zsnd_cfg s_cfg;
static uint8_t *s_rom;
static uint32_t s_rom_size, s_smask;
static uint8_t *s_ram;              // (RAM mode: the 64K; ROM mode: 0x1000 at f000)
static uint8_t s_ram_own[0x1000];

static struct {
	uint8_t latch, latch_pending, latch2, latch2_pending;
	uint8_t rst, held, dac, pad;
	uint32_t sample_addr;
	uint32_t ym_frac;
	int32_t nmi_cnt;
	unsigned long z_end;
	uint64_t frac;
	uint8_t state_end;
} t;

enum { Q_ADV, Q_LATCH, Q_LATCH2, Q_RESET, Q_START };
#define Q_MAX 4096
typedef struct { uint32_t kind, v; } q_ev_t;
static q_ev_t s_q[2][Q_MAX];
static int s_qn[2], s_qcur;
static volatile int s_job_frame;

#define A_MAX 512
typedef struct { uint16_t line; uint8_t v, pending; } ans_t;
static ans_t s_ans[2][A_MAX];
static int s_ansn[2], s_anspos, s_job_line;

static int32_t s_dac_out[OUT_MAX], s_mix[OUT_MAX];
static int32_t s_ym_hold[OUT_MAX];   // (the YM2151 samples of the last frame, taken at a save: the state has no ring)
static int s_ym_held;
static int s_out_n, s_out_pos;
static int16_t s_main_dac[1024];    // (the MCU's writes by line, -1 none)
static uint8_t s_main_last;
static int s_has_main_dac;
static int s_line;                  // (the Z80's INT, live: an acknowledge in a run drops it, or the Z80 would take RST 38)

static void q_put(uint32_t kind, uint32_t v)
{
	const int c = s_qcur;
	if (s_qn[c] < Q_MAX) {
		s_q[c][s_qn[c]].kind = kind;
		s_q[c][s_qn[c]].v = v;
		s_qn[c]++;
	}
}

// ---- rst_neg_buffer: the two interrupt sources on the Z80's bus ----
static inline void rst_set(uint8_t bit, int state)
{
	if (state)
		t.rst |= bit;
	else
		t.rst &= (uint8_t)~bit;
	s_z.int_data = (uint8_t)(t.rst ^ 0xff);
	s_line = s_cfg.ports != ZP_DYNA && t.rst != 0;
}
static void ym_irq(int state)
{
	if (s_cfg.ports != ZP_DYNA)
		rst_set(0x10, state);   // (rst28)
}

// ---- m72_audio ----
static inline uint8_t sample_r(void) { return s_cfg.samples ? s_cfg.samples[t.sample_addr & s_smask] : 0; }
static inline void sample_w(uint8_t v)
{
	t.dac = v;
	t.sample_addr++;
}
static void addr_shift_w(uint32_t o, uint8_t v, int shift)
{
	t.sample_addr >>= shift;
	if (o == 1)
		t.sample_addr = (t.sample_addr & 0x00ff) | ((uint32_t)v << 8);
	else
		t.sample_addr = (t.sample_addr & 0xff00) | v;
	t.sample_addr <<= shift;
}

// ---- the Z80's buses ----
static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (s_cfg.ram)
		return s_cfg.ram[a];
	if (a >= 0xf000)
		return s_ram_own[a - 0xf000];
	return a < s_rom_size ? s_rom[a] : 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_cfg.ram)
		s_cfg.ram[a] = v;
	else if (a >= 0xf000)
		s_ram_own[a - 0xf000] = v;
}
static void ack_w(void)
{
	t.latch_pending = 0;
	rst_set(0x20, 0);
}
static void answer(uint8_t v, int pending)
{
	const int b = s_qcur ^ 1;
	if (s_ansn[b] < A_MAX) {
		s_ans[b][s_ansn[b]].line = (uint16_t)s_job_line;
		s_ans[b][s_ansn[b]].v = v;
		s_ans[b][s_ansn[b]].pending = (uint8_t)pending;
		s_ansn[b]++;
	}
}
static uint8_t z_in(z80 *z, uint8_t p)
{
	(void)z;
	switch (s_cfg.ports) {
	case ZP_RTYPE:
		if (p < 2) return rbx_ym_read(0, p);
		if (p == 2) return t.latch;
		break;
	case ZP_M72:
		if (p < 2) return rbx_ym_read(0, p);
		if (p == 2) return t.latch;
		if (p == 0x84) return sample_r();
		break;
	case ZP_PROT:
		if (p < 2) return rbx_ym_read(0, p);
		if (p == 2) return t.latch;
		if (p == 0x84) {   // (soundlatch2: the pending line, the NMI, falls)
			t.latch2_pending = 0;
			return t.latch2;
		}
		break;
	case ZP_RTYPE2:
		if (p < 2) return rbx_ym_read(0, p);
		if (p == 0x80) return t.latch;
		if (p == 0x84) return sample_r();
		break;
	case ZP_M99:
		if (p == 0x40 || p == 0x41) return rbx_ym_read(0, p & 1);
		if (p == 0x42) return t.latch;
		break;
	case ZP_DYNA:
		if (p < 2) return rbx_ym_read(0, p);
		if (p == 0x80) {
			t.latch_pending = 0;
			return t.latch;
		}
		break;
	}
	return 0;
}
static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	switch (s_cfg.ports) {
	case ZP_RTYPE:
		if (p < 2) rbx_ym_write(0, p, v);
		else if (p == 6) ack_w();
		break;
	case ZP_M72:
		if (p < 2) rbx_ym_write(0, p, v);
		else if (p == 6) ack_w();
		else if (p == 0x82) sample_w(v);
		break;
	case ZP_PROT:
		if (p < 2) rbx_ym_write(0, p, v);
		else if (p == 6) ack_w();
		else if (p == 0x82) answer(v, 1);
		break;
	case ZP_RTYPE2:
		if (p < 2) rbx_ym_write(0, p, v);
		else if (p == 0x80 || p == 0x81) addr_shift_w(p & 1, v, 5);
		else if (p == 0x82) sample_w(v);
		else if (p == 0x83) ack_w();
		break;
	case ZP_M99:
		if (p < 2) addr_shift_w(p, v, 4);   // (poundfor_sample_addr_w)
		else if (p == 2 || p == 3) {}
		else if (p == 0x40 || p == 0x41) rbx_ym_write(0, p & 1, v);
		else if (p == 0x42) ack_w();
		break;
	case ZP_DYNA:
		if (p < 2) rbx_ym_write(0, p, v);
		else if (p == 0x82) t.dac = v;
		break;
	}
}

static void z_setup(void)
{
	s_line = s_cfg.ports != ZP_DYNA && t.rst != 0;
	z80_init(&s_z);
	s_z.read_byte = z_rd;
	s_z.write_byte = z_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = (uint8_t)(t.rst ^ 0xff);
	s_z.read_map = s_zmap;
}

// ---- the worker's job ----
static void periodic(void)
{
	switch (s_cfg.nmi) {
	case ZN_PULSE:
		z80_gen_nmi(&s_z);
		break;
	case ZN_FAKE: {
		const uint8_t s = sample_r();
		if (s)
			sample_w(s);
		break;
	}
	case ZN_FAKE80: {
		const uint8_t s = sample_r();
		if (s != 0x80)
			sample_w(s);
		break;
	}
	case ZN_IRQHOLD: {
		int line = 1;
		z80_run(&s_z, s_z.cyc + 1, &line, 1);
		break;
	}
	}
}

static void sound_run(uint32_t cyc)
{
	if (s_cfg.nmi != ZN_NONE && --t.nmi_cnt <= 0) {
		t.nmi_cnt = s_cfg.nmi_lines;
		if (!t.held)
			periodic();
	}
	t.z_end += cyc;
	if (!t.held)
		z80_run(&s_z, t.z_end, &s_line, 0);
	else
		s_z.cyc = t.z_end;
	int32_t dummy[64];
	const int n = rbx_ym_advance(0, cyc, dummy, 64);
	const int32_t d = ((int32_t)t.dac - 0x80) * s_cfg.dac_gain;
	for (int i = 0; i < n && s_out_pos < OUT_MAX; i++)
		s_dac_out[s_out_pos++] = d;
}

static void job(void)
{
	if (!__atomic_load_n(&s_job_frame, __ATOMIC_ACQUIRE))
		return;
	const int b = s_qcur ^ 1;
	s_out_pos = 0;
	s_job_line = 0;
	for (int i = 0; i < s_qn[b]; i++) {
		const q_ev_t *e = &s_q[b][i];
		switch (e->kind) {
		case Q_ADV:
			s_job_line = (int)(e->v >> 16);
			sound_run(e->v & 0xffff);
			break;
		case Q_LATCH:
			t.latch = (uint8_t)e->v;
			t.latch_pending = 1;
			if (s_cfg.ports == ZP_DYNA)
				z80_gen_nmi(&s_z);
			else
				rst_set(0x20, 1);   // (rst18)
			break;
		case Q_LATCH2:
			t.latch2 = (uint8_t)e->v;
			if (!t.latch2_pending)
				z80_gen_nmi(&s_z);
			t.latch2_pending = 1;
			break;
		case Q_RESET:
			if (e->v)
				t.held = 1;
			else if (t.held) {
				t.held = 0;
				const unsigned long c = s_z.cyc;
				z_setup();
				s_z.cyc = c;
			}
			break;
		case Q_START:
			t.sample_addr = e->v;
			break;
		}
	}
	s_qn[b] = 0;
	s_out_n = s_out_pos;
	__atomic_store_n(&s_job_frame, 0, __ATOMIC_RELEASE);
}

// ---- the main side ----
void zsnd_latch_w(uint8_t v) { q_put(Q_LATCH, v); }
void zsnd_latch2_w(uint8_t v) { q_put(Q_LATCH2, v); }
void zsnd_reset_line(int asserted)
{
	// (the main CPU writes the Z80's RAM while it holds it: the Z80 a frame behind must stop now, not a frame later)
	if (asserted && s_cfg.ram) {
		rbx_aux2_sync();
		t.held = 1;
	}
	q_put(Q_RESET, (uint32_t)asserted);
}
void zsnd_sample_start(uint32_t a) { q_put(Q_START, a); }

void zsnd_dac_main(int line, uint8_t v)
{
	s_has_main_dac = 1;
	s_main_dac[line & 1023] = v;
}

void zsnd_line(int line)
{
	const int b = s_qcur;
	while (s_anspos < s_ansn[b] && s_ans[b][s_anspos].line <= line) {
		if (s_cfg.mcu_latch)
			s_cfg.mcu_latch(s_ans[b][s_anspos].v, s_ans[b][s_anspos].pending);
		s_anspos++;
	}
	const uint32_t c = (uint32_t)rbx_line_cycles(&t.frac, Z80_CLOCK, s_cfg.frame_mhz, s_cfg.lines);
	q_put(Q_ADV, (uint32_t)line << 16 | c);
}

void zsnd_frame(void)
{
	rbx_aux2_sync();
	const int n = s_out_n;
	memcpy(s_mix, s_dac_out, sizeof(int32_t) * (size_t)n);
	if (s_ym_held) {
		for (int i = 0; i < n; i++)
			s_mix[i] += s_ym_hold[i];
		s_ym_held = 0;
	} else
		rbx_ym_async_take(0, s_mix, n);
	if (s_has_main_dac && n > 0) {
		const int lines = s_cfg.lines;
		int l = -1;
		for (int i = 0; i < n; i++) {
			const int ln = i * lines / n;
			while (l < ln) {
				l++;
				if (s_main_dac[l] >= 0)
					s_main_last = (uint8_t)s_main_dac[l];
			}
			s_mix[i] += ((int32_t)s_main_last - 0x80) * s_cfg.dac_gain;
		}
		memset(s_main_dac, 0xff, sizeof(s_main_dac));
	}
	for (int i = 0; i < n; i++)
		rbx_audio_put(s_mix[i]);
	s_out_n = 0;
	s_ansn[s_qcur] = 0;
	s_anspos = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux2_kick();
}

int zsnd_init(const zsnd_cfg *cfg)
{
	s_cfg = *cfg;
	s_rom_size = 0;
	s_rom = cfg->ram ? NULL : rbx_region("soundcpu", &s_rom_size);
	if (!cfg->ram && !s_rom)
		return -1;
	uint32_t m = 1;
	while (m < cfg->samples_size)
		m <<= 1;
	s_smask = m - 1;
	memset(s_zmap, 0, sizeof(s_zmap));
	if (cfg->ram)
		for (int p = 0; p < 256; p++)
			s_zmap[p] = cfg->ram + p * 256;
	else {
		for (int p = 0; p < 0xf0; p++)
			if ((uint32_t)p * 256 + 256 <= s_rom_size)
				s_zmap[p] = s_rom + p * 256;
		for (int p = 0; p < 0x10; p++)
			s_zmap[0xf0 + p] = s_ram_own + p * 256;
	}
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = Z80_CLOCK;
	yc.gain[0] = yc.gain[1] = cfg->ym_gain;
	yc.irq = ym_irq;
	if (rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1) || rbx_audio_setup(rbx_ym_rate(0)))
		return -1;
	rbx_aux2_set_job(job);
	rbx_aux2_set_alone(1);   // (the CPU on a core of its own where there is one)
	return 0;
}

void zsnd_exit(void)
{
	rbx_aux2_set_job(NULL);
	rbx_ym_exit(0);
}

void zsnd_reset(int held)
{
	rbx_aux2_sync();
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_ansn[0] = s_ansn[1] = 0;
	s_anspos = 0;
	s_qcur = 0;
	s_job_frame = 0;
	s_out_n = s_out_pos = 0;
	s_ym_held = 0;
	s_has_main_dac = 0;
	s_main_last = 0x80;
	memset(s_main_dac, 0xff, sizeof(s_main_dac));
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	t.dac = 0x80;
	t.held = (uint8_t)held;
	memset(s_ram_own, 0, sizeof(s_ram_own));
	z_setup();
	rbx_ym_reset(0);
}

int zsnd_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux2_sync();
	rbx_aux_sync();
	if (!load && !s_ym_held && s_out_n > 0) {
		memset(s_ym_hold, 0, sizeof(int32_t) * (size_t)s_out_n);
		rbx_ym_async_take(0, s_ym_hold, s_out_n);
		s_ym_held = 1;
	}
	int32_t held = s_ym_held;
	rc |= rbx_state_io(ctx, "zsnd_yh", &held, sizeof(held));
	rc |= rbx_state_io(ctx, "zsnd_yo", s_ym_hold, sizeof(s_ym_hold));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "zsnd_z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zsnd", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "zsnd_ram", s_ram_own, sizeof(s_ram_own));
	int32_t n[5] = { s_out_n, s_ansn[0], s_ansn[1], s_anspos | s_qcur << 16, s_has_main_dac | s_main_last << 8 };
	rc |= rbx_state_io(ctx, "zsnd_n", n, sizeof(n));
	rc |= rbx_state_io(ctx, "zsnd_out", s_dac_out, sizeof(s_dac_out));
	rc |= rbx_state_io(ctx, "zsnd_ans", s_ans, sizeof(s_ans));
	rc |= rbx_state_io(ctx, "zsnd_mdac", s_main_dac, sizeof(s_main_dac));
	rc |= rbx_ym_state(0, ctx, load);
	if (!rc && load) {
		z.read_byte = z_rd;
		z.write_byte = z_wr;
		z.port_in = z_in;
		z.port_out = z_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		z.op_rom = NULL;
		s_z = z;
		s_line = s_cfg.ports != ZP_DYNA && t.rst != 0;
		s_out_n = n[0] >= 0 && n[0] <= OUT_MAX ? n[0] : 0;
		s_ansn[0] = n[1] & (A_MAX * 2 - 1);
		s_ansn[1] = n[2] & (A_MAX * 2 - 1);
		s_anspos = n[3] & 0xffff;
		s_qcur = (n[3] >> 16) & 1;
		s_ym_held = held != 0;
		s_has_main_dac = n[4] & 1;
		s_main_last = (uint8_t)(n[4] >> 8);
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
	}
	return rc ? -1 : 0;
}
