/*  The Irem M52 / M62 sound board (irem_audio.h), ported from MAME src/mame/irem/irem.cpp (BSD-3-Clause,
    copyright-holders: Couriersud).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "irem_audio.h"
#include "../../rbx.h"
#include "../../cpu/m6800/m6801.h"
#include "../../sound/ay8910.h"
#include "../../sound/msm5205.h"

#include <stddef.h>
#include <string.h>

#define CPU_CLOCK 3579545u
#define MSM_CLOCK 384000u
#define AY_CLOCK (CPU_CLOCK / 4)
#define AY_DIV 2
#define SND_RATE (AY_CLOCK / 8 / AY_DIV)
#define Q_MAX 2048
#define OUT_MAX 2048

enum { Q_ADV, Q_CMD };
typedef struct { uint32_t kind, v; } q_ev_t;

static q_ev_t s_q[2][Q_MAX];
static int s_qn[2], s_qcur;
static int32_t s_out[OUT_MAX];
static int s_out_n, s_out_pos;
static volatile int s_job_frame;

static m6801 s_cpu;
static ay8910_t s_ay[2];        // (0: 45M, 1: 45L)
static msm5205_t s_msm[2];
static const uint8_t *s_rom;
static uint32_t s_rom_size;
static int s_type, s_nmsm;
static uint32_t s_htotal, s_den;

static struct {
	uint8_t port1, port2, soundlatch, vck;
	int32_t edge_fp, period_fp;   // (E clocks to VCK's next edge and its half period, 16.16: 0 = stopped)
	uint32_t line_frac;
} a;

// ---- the MSM5205s' clock ----
static int32_t half_period(int prescaler)   // (half a prescaler period of the 384 kHz clock, in the 6803's E clocks)
{
	return prescaler ? (int32_t)(((uint64_t)(uint32_t)prescaler * (CPU_CLOCK / 4) << 15) / MSM_CLOCK) : 0;
}

static void playmode(int select)
{
	const int old = s_msm[0].prescaler;
	msm5205_set_selector(&s_msm[0], select);
	if (s_msm[0].prescaler == old)
		return;
	a.period_fp = half_period(s_msm[0].prescaler);
	a.edge_fp = a.period_fp;
}

static void vck_edge(void)
{
	a.vck ^= 1;
	m6801_set_irq_line(&s_cpu, M6801_NMI_LINE, a.vck);
	if (!a.vck)
		for (int i = 0; i < s_nmsm; i++)
			msm5205_vclk_capture(&s_msm[i]);
}

// ---- the 6803's ports ----
static uint8_t port_r(void *u, int port)
{
	(void)u;
	if (port == 1) {
		if (a.port2 & 0x08)
			return ay8910_data_r(&s_ay[0]);
		if (a.port2 & 0x10)
			return ay8910_data_r(&s_ay[1]);
		return 0xff;
	}
	return port == 2 ? 0x00 : 0xff;
}

static void port_w(void *u, int port, uint8_t v, uint8_t ddr)
{
	(void)u;
	(void)ddr;
	if (port == 1)
		a.port1 = v;
	else if (port == 2) {
		if ((a.port2 & 0x01) && !(v & 0x01)) {   // (the write latch)
			if (a.port2 & 0x04) {   // (control or data port)
				if (a.port2 & 0x08)
					ay8910_address_w(&s_ay[0], a.port1);
				if (a.port2 & 0x10)
					ay8910_address_w(&s_ay[1], a.port1);
			} else {
				if (a.port2 & 0x08)
					ay8910_data_w(&s_ay[0], a.port1);
				if (a.port2 & 0x10)
					ay8910_data_w(&s_ay[1], a.port1);
			}
		}
		a.port2 = v;
	}
}

static uint8_t ay_soundlatch(void *u) { (void)u; return a.soundlatch; }

static void ay_portb(void *u, uint8_t v)   // (ay8910_45M_portb_w)
{
	(void)u;
	playmode((v >> 2) & 7);   // (bits 2-4: the clock and 3 / 4 bit playback)
	msm5205_reset_w(&s_msm[0], v & 1);
	if (s_nmsm > 1) {
		msm5205_set_selector(&s_msm[1], ((v >> 2) & 4) | 3);   // (always the slave)
		msm5205_reset_w(&s_msm[1], (v >> 1) & 1);
	}
}

// ---- the 6803's bus ----
static void irq_ack(void)
{
	if (a.soundlatch & 0x80)
		m6801_set_irq_line(&s_cpu, M6801_IRQ1_LINE, 0);
}

static uint8_t bus_r(void *u, uint16_t addr)
{
	(void)u;
	if (s_type == IREM_AUDIO_M52_SOUNDC) {
		const uint16_t m = addr & 0x7fff;
		return m >= 0x2000 ? s_rom[m] : 0;
	}
	return addr >= 0x4000 ? s_rom[addr] : 0;
}

static void bus_w(void *u, uint16_t addr, uint8_t v)
{
	(void)u;
	if (s_type == IREM_AUDIO_M62) {
		if (addr & 0x0800) {   // (m62_sound_map: mirror 0xf7fc)
			switch (addr & 3) {
			case 0: irq_ack(); break;
			case 1: msm5205_data_w(&s_msm[0], v); break;
			case 2: msm5205_data_w(&s_msm[1], v); break;
			}
		}
		return;
	}
	const uint16_t m = s_type == IREM_AUDIO_M52_SOUNDC ? addr & 0x7fff : addr;
	const uint16_t top = s_type == IREM_AUDIO_M52_SOUNDC ? 0x1000 : 0x2000;
	if (m < top) {   // (m52_adpcm_w)
		if (m & 1)
			msm5205_data_w(&s_msm[0], v);
		if ((m & 2) && s_nmsm > 1)
			msm5205_data_w(&s_msm[1], v);
	} else if (m < top * 2)
		irq_ack();
}

// ---- on the worker ----
static void sound_run(uint32_t e)
{
	uint32_t left = e;
	while (left) {
		uint32_t step = left;
		if (a.period_fp) {
			const uint32_t n = ((uint32_t)a.edge_fp + 0xffffu) >> 16;
			if (n && n < step)
				step = n;
		}
		m6801_execute(&s_cpu, (int)step);
		left -= step;
		if (a.period_fp) {
			a.edge_fp -= (int32_t)(step << 16);
			if (a.edge_fp <= 0) {
				a.edge_fp += a.period_fp;
				vck_edge();
			}
		}
	}
	int32_t mix[64];
	const int max = ay8910_out_max(&s_ay[0], e);
	if (max > 64)
		return;
	memset(mix, 0, (size_t)max * sizeof(mix[0]));
	const int n = ay8910_render(&s_ay[0], e, mix, max);
	ay8910_render(&s_ay[1], e, mix, max);
	const int32_t msm = msm5205_output(&s_msm[0]) + (s_nmsm > 1 ? msm5205_output(&s_msm[1]) : 0);
	for (int i = 0; i < n && s_out_pos < OUT_MAX; i++)
		s_out[s_out_pos++] = mix[i] + msm;
}

static void job(void)
{
	if (!__atomic_load_n(&s_job_frame, __ATOMIC_ACQUIRE))
		return;
	const int b = s_qcur ^ 1;
	s_out_pos = 0;
	for (int i = 0; i < s_qn[b]; i++) {
		const q_ev_t *ev = &s_q[b][i];
		if (ev->kind == Q_ADV)
			sound_run(ev->v);
		else {
			a.soundlatch = (uint8_t)ev->v;
			if (!(ev->v & 0x80))
				m6801_set_irq_line(&s_cpu, M6801_IRQ1_LINE, 1);
		}
	}
	s_qn[b] = 0;
	s_out_n = s_out_pos;
	__atomic_store_n(&s_job_frame, 0, __ATOMIC_RELEASE);
}

// ---- on the emulation thread ----
static void q_put(uint32_t kind, uint32_t v)
{
	const int c = s_qcur;
	if (s_qn[c] < Q_MAX) {
		s_q[c][s_qn[c]].kind = kind;
		s_q[c][s_qn[c]].v = v;
		s_qn[c]++;
	}
}

void irem_audio_cmd_w(uint8_t v) { q_put(Q_CMD, v); }

void irem_audio_line(void)
{
	a.line_frac += CPU_CLOCK * s_htotal;   // (below 2^32: htotal is below 600)
	const uint32_t e = a.line_frac / s_den;
	a.line_frac -= e * s_den;
	q_put(Q_ADV, e);
}

void irem_audio_frame(void)
{
	rbx_aux_sync();
	for (int i = 0; i < s_out_n; i++)
		rbx_audio_put(s_out[i]);
	s_out_n = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

int irem_audio_init(int type, uint32_t htotal, uint32_t pix_clock)
{
	s_rom = rbx_region("irem_audio:iremsound", &s_rom_size);
	const uint32_t base = type == IREM_AUDIO_M52_SOUNDC ? 0x2000 : 0x4000;
	if (!s_rom || s_rom_size < (type == IREM_AUDIO_M52_SOUNDC ? 0x8000u : 0x10000u))
		return -1;
	s_type = type;
	s_nmsm = type == IREM_AUDIO_M52_SOUNDC ? 1 : 2;
	s_htotal = htotal;
	s_den = pix_clock * 4;
	memset(&s_cpu, 0, sizeof(s_cpu));
	s_cpu.read = bus_r;
	s_cpu.write = bus_w;
	s_cpu.port_r = port_r;
	s_cpu.port_w = port_w;
	s_cpu.type = M6801_TYPE_6801;
	s_cpu.rom = s_rom + base;
	s_cpu.rom_base = base;
	s_cpu.rom_size = type == IREM_AUDIO_M52_SOUNDC ? 0x6000 : 0xc000;
	for (int i = 0; i < 2; i++) {
		// (the M62's AYs go to its netlist: the plain output at its level, measured on the coin sounds against MAME's
		// netlist (ldrun, spelunkr, kidniki, horizon: twice as loud at 96); the M52's are one output over 470 ohms)
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, AY_CLOCK, type == IREM_AUDIO_M62 ? AY8910_LEGACY_OUTPUT : AY8910_SINGLE_OUTPUT,
		            type == IREM_AUDIO_M62 ? 48 : type == IREM_AUDIO_M52_LARGE ? 205 : 512, AY_DIV,   // (M52 large: 0.80)
		            type != IREM_AUDIO_M62);
		if (type != IREM_AUDIO_M62)
			ay8910_set_res_loads(&s_ay[i], 470, 470, 470);
	}
	s_ay[0].port_a_read = ay_soundlatch;
	s_ay[0].port_b_write = ay_portb;
	msm5205_init(&s_msm[0], MSM_CLOCK, MSM5205_S96_4B, NULL);
	msm5205_init(&s_msm[1], MSM_CLOCK, MSM5205_SEX_4B, NULL);
	if (rbx_audio_setup(SND_RATE))
		return -1;
	rbx_aux_set_job(job);
	return 0;
}

void irem_audio_exit(void) { rbx_aux_set_job(NULL); }

void irem_audio_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&a, 0, sizeof(a));
	ay8910_reset(&s_ay[0]);
	ay8910_reset(&s_ay[1]);
	msm5205_set_selector(&s_msm[0], MSM5205_S96_4B);
	msm5205_set_selector(&s_msm[1], MSM5205_SEX_4B);
	msm5205_reset(&s_msm[0]);
	msm5205_reset(&s_msm[1]);
	a.period_fp = half_period(96);
	a.edge_fp = a.period_fp;
	m6801_init(&s_cpu);
	m6801_reset(&s_cpu);
	m6801_set_irq_line(&s_cpu, M6801_IRQ1_LINE, 1);
}

int irem_audio_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "ia", &a, sizeof(a));
	rc |= rbx_state_io(ctx, "ia_cpu", m6801_state_data(&s_cpu), m6801_state_size());
	rc |= rbx_state_io(ctx, "ia_ay0", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "ia_ay1", &s_ay[1], ay8910_state_size());
	rc |= rbx_state_io(ctx, "ia_msm", s_msm, sizeof(s_msm));
	int32_t n = s_out_n;
	rc |= rbx_state_io(ctx, "ia_n", &n, sizeof(n));
	rc |= rbx_state_io(ctx, "ia_out", s_out, sizeof(s_out));
	if (!rc && load) {
		s_out_n = n >= 0 && n <= OUT_MAX ? n : 0;
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
	}
	return rc ? -1 : 0;
}
