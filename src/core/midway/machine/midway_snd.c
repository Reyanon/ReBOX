/*  The Bally Midway sound boards of the MCR boards: SSIO, Turbo Cheap Squeak, Sounds Good, Cheap Squeak Deluxe and
    Squawk & Talk. Ported to C from MAME src/mame/bally/midway_sound.cpp, src/mame/bally/csd.cpp and
    src/mame/shared/ballysound.cpp (BSD-3-Clause, copyright-holders: Aaron Giles); the DAC filters as
    src/devices/sound/flt_biquad.cpp (BSD-3-Clause, copyright-holders: K.Wilkins, Couriersud, Derrick Renaud, Frank
    Palazzolo, Jonathan Gevaryahu) and src/devices/sound/flt_rc.cpp (BSD-3-Clause, copyright-holders: Derrick Renaud,
    Couriersud). See midway_snd.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "midway_snd.h"
#include "../../rbx.h"
#include "../../rbx_prof.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/m6800/m6800.h"
#include "../../machine/pia6821.h"
#include "../../sound/ay8910.h"
#include "../../sound/tms5220.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

// ---- box-filtered DAC levels on a CPU's clock into MSND_RATE samples (16.16 cycles) ----
typedef struct {
	int64_t t, next, acc;
	int32_t level, pad_;
} boxch_t;

typedef struct {
	int64_t P16, inv;
} boxcfg_t;

static void box_setup(boxcfg_t *b, double clock)
{
	b->P16 = (int64_t)(clock / MSND_RATE * 65536.0 + 0.5);
	b->inv = (int64_t)(4294967296.0 / (double)b->P16 + 0.5);
}

// run the channel to `cycles`; each completed sample goes to emit (its average level)
static void box_run(boxch_t *c, const boxcfg_t *b, int64_t cycles, void (*emit)(int32_t s))
{
	const int64_t t = cycles * 65536;
	if (t <= c->t)
		return;
	while (c->next <= t) {
		c->acc += (int64_t)c->level * (c->next - c->t);
		c->t = c->next;
		c->next += b->P16;
		emit((int32_t)((c->acc * b->inv) >> 32));
		c->acc = 0;
	}
	c->acc += (int64_t)c->level * (t - c->t);
	c->t = t;
}

static inline void fifo_put(msnd_fifo_t *f, int32_t l, int32_t r)
{
	if (f->n < MSND_FIFO) {
		f->l[f->n] = l;
		f->r[f->n] = r;
		f->n++;
	}
}

// ---- the LM359 filters of the DAC boards (opamp_mfb_lowpass_setup, at MSND_RATE) ----
typedef struct { float b[3], a[3]; } biq_t;
static biq_t s_dacf[3];   // (the same three stages on the TCS and the SG; configuration)

static void biquad_setup(double r1, double r2, double r3, double c1, double c2, biq_t *q)
{
	const double pi = 3.14159265358979323846;
	const double gain = -r3 / r1;
	double fc, qv = sqrt(2.0) / 2.0;
	const int one_pole = c1 == 0;
	if (one_pole)
		fc = (r1 * r3) / (2 * pi * ((r1 * r2) + (r1 * r3) + (r2 * r3)) * r3 * c2);
	else {
		fc = 1.0 / (2 * pi * sqrt(r2 * r3 * c1 * c2));
		qv = sqrt(r2 * r3 * c1 * c2) / ((r3 * c2) + (r2 * c2) + ((r2 * c2) * -gain));
	}
	double b0, b1, b2, a1, a2;
	const double sr = MSND_RATE;
	if (fc >= sr / 2) {
		b0 = 1; b1 = b2 = a1 = a2 = 0;
	} else if (one_pole) {
		a1 = exp(-2.0 * pi * (fc / sr));
		b0 = 1.0 - a1;
		a1 = -a1;
		b1 = b2 = a2 = 0;
	} else {
		const double K = tan(pi * fc / sr), K2 = K * K, KoQ = K / qv, normal = 1.0 / (1.0 + KoQ + K2);
		b0 = K2 * normal;
		b1 = 2.0 * b0;
		b2 = b0;
		a1 = 2.0 * (K2 - 1.0) * normal;
		a2 = (1.0 - KoQ + K2) * normal;
	}
	q->b[0] = (float)(b0 * gain);
	q->b[1] = (float)(b1 * gain);
	q->b[2] = (float)(b2 * gain);
	q->a[0] = 1;
	q->a[1] = (float)a1;
	q->a[2] = (float)a2;
}

static inline float biquad(float *w, const biq_t *q, float in)
{
	w[2] = w[1];
	w[1] = w[0];
	w[0] = -q->a[1] * w[1] - q->a[2] * w[2] + in;
	return q->b[0] * w[0] + q->b[1] * w[1] + q->b[2] * w[2];
}

static void dac_filters_setup(void)
{
	static int done;
	if (done)
		return;
	done = 1;
	biquad_setup(120e3, 0, 120e3, 0, 270e-12, &s_dacf[0]);               // U3.1, 1st order
	biquad_setup(33e3, 18e3, 33e3, 5600e-12, 270e-12, &s_dacf[1]);       // U3.2
	biquad_setup(150e3, 82e3, 150e3, 470e-12, 150e-12, &s_dacf[2]);      // U2.2
}

// =====================================================================================================================
// SSIO
// =====================================================================================================================
#define SSIO_TICK 40   // the 14024's clock: 16 MHz / 2 / 16 / 10 = 50 kHz, 40 sound CPU clocks

static struct {
	// state (saved: up to cpu)
	uint8_t data[4];
	uint8_t status, count14024, mute, irq;
	uint8_t overall[2], duty[2][3];
	uint8_t suspended, in_reset, dip, pad_;
	int64_t next_tick;     // the 14024's next clock (sound CPU clocks)
	int64_t ay_time;       // the AYs rendered up to here
	int64_t cyc_base;      // (the Z80's cycle count is an unsigned long: 32 bits on the Vita; time = base + cyc)
	int32_t dsum[2];
	int32_t dcnt, pad2_;
	uint8_t ram[0x400];
	z80 cpu;
	// configuration
	ay8910_t ay[2];
	const uint8_t *rom;
	int32_t lookup[16];    // the duty cycles' volumes (Q15)
	int32_t gain;          // the board's output (Q15, x 0.33 of the AYs' routes)
	const uint8_t *rmap[256];
	msnd_fifo_t fifo;
} S1;

static inline int64_t ssio_now(void) { return S1.cyc_base + (int64_t)S1.cpu.cyc; }

static void ssio_ticks_to(int64_t now)
{
	while (S1.next_tick <= now) {
		S1.next_tick += SSIO_TICK;
		S1.count14024 = (S1.count14024 + 1) & 0x7f;
		if ((S1.count14024 & 0x3f) == 0)   // (bit 6 inverted is /SINT)
			S1.irq = (S1.count14024 >> 6) & 1;
	}
}

static int64_t ssio_irq_event(void)   // the next 14024 clock that changes /SINT
{
	const int ticks = 64 - (S1.count14024 & 0x3f);
	return S1.next_tick + (int64_t)(ticks - 1) * SSIO_TICK;
}

static void ssio_ay_sync(int64_t now)
{
	int16_t buf[3 * 72];
	while (S1.ay_time < now) {
		int64_t span = now - S1.ay_time;
		if (span > 512)
			span = 512;
		const int n0 = ay8910_render_ch(&S1.ay[0], (uint32_t)span, buf, 72);
		int16_t buf1[3 * 72];
		const int n1 = ay8910_render_ch(&S1.ay[1], (uint32_t)span, buf1, 72);
		const int n = n0 < n1 ? n0 : n1;
		S1.ay_time += span;
		const int32_t *g = S1.lookup;
		int32_t g0[3], g1[3];
		for (int c = 0; c < 3; c++) {
			g0[c] = S1.mute ? 0 : g[S1.duty[0][c]];
			g1[c] = S1.mute ? 0 : g[S1.duty[1][c]];
		}
		for (int i = 0; i < n; i++) {
			S1.dsum[0] += (buf[3 * i] * g0[0] + buf[3 * i + 1] * g0[1] + buf[3 * i + 2] * g0[2]) >> 15;
			S1.dsum[1] += (buf1[3 * i] * g1[0] + buf1[3 * i + 1] * g1[1] + buf1[3 * i + 2] * g1[2]) >> 15;
			if (++S1.dcnt == 5) {   // 250 kHz native samples, 5 a sample
				fifo_put(&S1.fifo, (int32_t)(((int64_t)S1.dsum[0] * S1.gain) / (5 << 15)),
				         (int32_t)(((int64_t)S1.dsum[1] * S1.gain) / (5 << 15)));
				S1.dsum[0] = S1.dsum[1] = 0;
				S1.dcnt = 0;
			}
		}
	}
}

static void ssio_porta0(void *u, uint8_t d) { (void)u; S1.duty[0][0] = d & 15; S1.duty[0][1] = d >> 4; }
static void ssio_portb0(void *u, uint8_t d) { (void)u; S1.duty[0][2] = d & 15; S1.overall[0] = (d >> 4) & 7; }
static void ssio_porta1(void *u, uint8_t d) { (void)u; S1.duty[1][0] = d & 15; S1.duty[1][1] = d >> 4; }
static void ssio_portb1(void *u, uint8_t d)
{
	(void)u;
	S1.duty[1][2] = d & 15;
	S1.overall[1] = (d >> 4) & 7;
	S1.mute = d & 0x80;
}

static uint8_t ssio_rd(void *u, uint16_t a)
{
	(void)u;
	switch (a >> 12) {
	case 0x8: return S1.ram[a & 0x3ff];
	case 0x9: return S1.data[a & 3];
	case 0xa:
		if ((a & 3) == 1) { ssio_ay_sync(ssio_now()); return ay8910_data_r(&S1.ay[0]); }
		return 0xff;
	case 0xb:
		if ((a & 3) == 1) { ssio_ay_sync(ssio_now()); return ay8910_data_r(&S1.ay[1]); }
		return 0xff;
	case 0xe:   // irq_clear: the 14024 count reset, /SINT cleared
		ssio_ticks_to(ssio_now());
		S1.count14024 = 0;
		S1.irq = 0;
		z80_break = 1;
		return 0xff;
	case 0xf: return S1.dip;
	default: return a < 0x4000 ? S1.rom[a] : 0xff;
	}
}

static void ssio_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	switch (a >> 12) {
	case 0x8: S1.ram[a & 0x3ff] = v; break;
	case 0xa: case 0xb: {
		ay8910_t *ay = &S1.ay[(a >> 12) & 1];
		if ((a & 3) == 0)
			ay8910_address_w(ay, v);
		else if ((a & 3) == 2) {
			ssio_ay_sync(ssio_now());
			ay8910_data_w(ay, v);
		}
		break;
	}
	case 0xc: S1.status = v; break;
	default: break;
	}
}

static void ssio_map(void)
{
	for (int p = 0; p < 256; p++)
		S1.rmap[p] = p < 0x40 ? S1.rom + p * 256 : (p >> 4) == 0x8 ? S1.ram + (p & 3) * 256 : NULL;
	S1.cpu.read_map = S1.rmap;
	S1.cpu.read_byte = ssio_rd;
	S1.cpu.write_byte = ssio_wr;
	S1.cpu.userdata = NULL;
}

int msnd_ssio_init(const uint8_t *cpu_rom, uint32_t cpu_size, const uint8_t *prom, float gain)
{
	if (!cpu_rom || cpu_size < 0x4000)
		return -1;
	memset(&S1, 0, sizeof(S1));
	S1.rom = cpu_rom;
	S1.gain = (int32_t)(gain * 0.33f * 32768.0f);
	// compute_ay8910_modulation: the PROM's clocks counted for each duty cycle value
	for (int volval = 0; volval < 16; volval++) {
		int remaining = volval, prev = 1, curclock;
		for (curclock = 0; curclock < 160 && remaining; curclock++) {
			const int cur = prom ? prom[curclock / 8] & (0x80 >> (curclock % 8)) : (curclock & 1);
			if (cur == 0 && prev != 0)
				remaining--;
			prev = cur;
		}
		S1.lookup[15 - volval] = (int32_t)(curclock / 160.0 * 32768.0 + 0.5);
	}
	for (int i = 0; i < 2; i++) {
		ay8910_init(&S1.ay[i], AY8910_TYPE_AY8910, 2000000, AY8910_LEGACY_OUTPUT, 256, 1, 0);
		S1.ay[i].port_a_write = i ? ssio_porta1 : ssio_porta0;
		S1.ay[i].port_b_write = i ? ssio_portb1 : ssio_portb0;
	}
	msnd_ssio_reset();
	return 0;
}

void msnd_ssio_reset(void)
{
	memset(S1.data, 0, sizeof(S1.data));
	S1.status = 0;
	S1.count14024 = 0;
	S1.irq = 0;
	S1.mute = 0;
	S1.in_reset = 0;
	memset(S1.duty, 0, sizeof(S1.duty));
	memset(S1.ram, 0, sizeof(S1.ram));
	memset(S1.dsum, 0, sizeof(S1.dsum));
	S1.dcnt = 0;
	S1.fifo.n = 0;
	z80_init(&S1.cpu);
	S1.cpu.ix = S1.cpu.iy = 0xffff;   // (MAME z80: IX and IY are FFFF after a reset)
	S1.cpu.int_data = 0xff;           // (no daisy chain: RST 38 in IM 0)
	ssio_map();
	S1.next_tick = SSIO_TICK;
	S1.ay_time = 0;
	S1.cyc_base = 0;
	for (int i = 0; i < 2; i++)
		ay8910_reset(&S1.ay[i]);
}

void msnd_ssio_write(int offset, uint8_t data) { S1.data[offset & 3] = data; }
uint8_t msnd_ssio_status(void) { return S1.status; }
void msnd_ssio_suspend(int on) { S1.suspended = (uint8_t)on; }
void msnd_ssio_set_dip(uint8_t v) { S1.dip = v; }
int64_t msnd_ssio_now(void) { return ssio_now(); }
msnd_fifo_t *msnd_ssio_fifo(void) { return &S1.fifo; }

void msnd_ssio_reset_write(int state)
{
	if (state) {   // going high halts the CPU (device_reset: the latches too)
		memset(S1.data, 0, sizeof(S1.data));
		S1.status = 0;
		S1.count14024 = 0;
		S1.irq = 0;
		if (!S1.in_reset) {
			const unsigned long cyc = S1.cpu.cyc;
			z80_init(&S1.cpu);
			S1.cpu.ix = S1.cpu.iy = 0xffff;
			S1.cpu.int_data = 0xff;
			ssio_map();
			S1.cpu.cyc = cyc;
		}
	}
	S1.in_reset = (uint8_t)(state != 0);
}

void msnd_ssio_run(int64_t cycles)
{
	if (S1.cpu.cyc > 0x40000000ul) {   // (rebased long before the 32-bit count wraps)
		S1.cpu.cyc -= 0x40000000ul;
		S1.cyc_base += 0x40000000;
	}
	if (S1.suspended || S1.in_reset) {
		ssio_ticks_to(cycles);
		if (ssio_now() < cycles)
			S1.cpu.cyc = (unsigned long)(cycles - S1.cyc_base);
	} else {
		while (ssio_now() < cycles) {
			ssio_ticks_to(ssio_now());
			int64_t end = ssio_irq_event();
			if (end > cycles)
				end = cycles;
			if (end <= ssio_now())
				end = ssio_now() + 1;
			int line = S1.irq;
			z80_break = 0;
			z80_run(&S1.cpu, (unsigned long)(end - S1.cyc_base), &line, 0);
			z80_break = 0;
		}
		ssio_ticks_to(ssio_now());
	}
	ssio_ay_sync(cycles);
}

// =====================================================================================================================
// Turbo Cheap Squeak (MC6809E at 2 MHz, PIA, DAC, filters)
// =====================================================================================================================
static struct {
	uint8_t status, irq, in_reset, pad_;
	uint16_t dacval, pad2_;
	boxch_t ch;
	float w[3][3];
	uint8_t ram[0x800];
	m6809_t cpu;
	pia6821_t pia;
	// configuration
	const uint8_t *rom;
	boxcfg_t box;
	float gain;
	msnd_fifo_t fifo;
} S2;

static void tcs_emit(int32_t s)
{
	float y = (float)s;
	for (int i = 0; i < 3; i++)
		y = biquad(S2.w[i], &s_dacf[i], y);
	const int32_t v = (int32_t)(y * S2.gain);
	fifo_put(&S2.fifo, v, v);
}

static void tcs_dac(void)
{
	box_run(&S2.ch, &S2.box, S2.cpu.cycles, tcs_emit);
	S2.ch.level = ((int32_t)S2.dacval - 512) * 64;   // 10 bits to +-32767
}

static void tcs_pa(void *c, uint8_t d) { (void)c; S2.dacval = (uint16_t)((d << 2) | (S2.dacval & 3)); tcs_dac(); }
static void tcs_pb(void *c, uint8_t d)
{
	(void)c;
	S2.dacval = (uint16_t)((S2.dacval & ~3) | (d >> 6));
	tcs_dac();
	S2.status = (d >> 4) & 3;
}
static void tcs_irq(void *c, int st)
{
	(void)c; (void)st;
	m6809_set_irq(&S2.cpu, S2.pia.irq_a_state | S2.pia.irq_b_state);
}

static inline int pia_alt(uint16_t a) { return ((a << 1) & 2) | ((a >> 1) & 1); }   // (read_alt / write_alt)

static uint8_t tcs_rd(void *c, uint16_t a)
{
	(void)c;
	if ((a & 0xc000) == 0x4000)
		return pia6821_read(&S2.pia, pia_alt(a & 3));
	return 0xff;
}
static void tcs_wr(void *c, uint16_t a, uint8_t v)
{
	(void)c;
	if ((a & 0xc000) == 0x4000)
		pia6821_write(&S2.pia, pia_alt(a & 3), v);
}

static void tcs_map(void)
{
	m6809_init(&S2.cpu);
	S2.cpu.read = tcs_rd;
	S2.cpu.write = tcs_wr;
	for (int p = 0; p < 0x40; p++)
		S2.cpu.rpage[p] = S2.cpu.wpage[p] = S2.ram + (p & 7) * 256;
	for (int p = 0x80; p < 0x100; p++)
		S2.cpu.rpage[p] = S2.rom + (p - 0x80) * 256 + 0x8000;
}

int msnd_tcs_init(const uint8_t *cpu_rom, uint32_t cpu_size, float gain)
{
	if (!cpu_rom || cpu_size < 0x10000)
		return -1;
	memset(&S2, 0, sizeof(S2));
	dac_filters_setup();
	S2.rom = cpu_rom;
	S2.gain = gain;
	box_setup(&S2.box, 2000000.0);
	pia6821_config(&S2.pia);
	S2.pia.out_a_cb = tcs_pa;
	S2.pia.out_b_cb = tcs_pb;
	S2.pia.irqa_cb = tcs_irq;
	S2.pia.irqb_cb = tcs_irq;
	msnd_tcs_reset();
	return 0;
}

void msnd_tcs_reset(void)
{
	S2.status = 0;
	S2.dacval = 0;
	S2.in_reset = 0;
	memset(&S2.ch, 0, sizeof(S2.ch));
	S2.ch.next = S2.box.P16;
	S2.ch.level = -512 * 64;
	memset(S2.w, 0, sizeof(S2.w));
	memset(S2.ram, 0, sizeof(S2.ram));
	S2.fifo.n = 0;
	tcs_map();
	pia6821_reset(&S2.pia);
	m6809_reset(&S2.cpu);
}

void msnd_tcs_write(uint8_t data)
{
	pia6821_portb_w(&S2.pia, (data >> 1) & 0x0f);
	pia6821_ca1_w(&S2.pia, ~data & 1);
}
uint8_t msnd_tcs_read(void) { return S2.status; }
void msnd_tcs_reset_write(int state)
{
	if (state && !S2.in_reset) {
		const int64_t cy = S2.cpu.cycles;
		m6809_reset(&S2.cpu);
		S2.cpu.cycles = cy;
	}
	S2.in_reset = (uint8_t)(state != 0);
}
int64_t msnd_tcs_now(void) { return S2.cpu.cycles; }
msnd_fifo_t *msnd_tcs_fifo(void) { return &S2.fifo; }

void msnd_tcs_run(int64_t cycles)
{
	if (S2.in_reset) {
		if (S2.cpu.cycles < cycles)
			S2.cpu.cycles = cycles;
	} else
		m6809_run_until(&S2.cpu, cycles);
	box_run(&S2.ch, &S2.box, cycles, tcs_emit);
}

// =====================================================================================================================
// Sounds Good / Cheap Squeak Deluxe: 68000 at 8 MHz on the cpu/m68k_snd instance
// =====================================================================================================================
#include "../../cpu/m68k_snd/m68k_snd_names.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"

static m68k_bus_t s_bus68;

static struct {
	uint8_t status, in_reset, suspended, pad_;
	uint16_t dacval, pad2_;
	int64_t now;
	boxch_t ch;
	float w[3][3];
	uint32_t irq_asserted;
	uint8_t ram[0x1000];
	uint8_t ctx[512];
	pia6821_t pia;
	// configuration
	int type;
	uint8_t *rom;
	uint32_t rom_size;
	boxcfg_t box;
	float gain;
	int in_exec;
	msnd_fifo_t fifo;
} S3;

static int64_t s68_time(void) { return S3.now + (S3.in_exec ? m68k_cycles_run() : 0); }

static void s68_emit(int32_t s)
{
	float y = (float)s;
	if (S3.type == MSND_68K_SG)
		for (int i = 0; i < 3; i++)
			y = biquad(S3.w[i], &s_dacf[i], y);
	const int32_t v = (int32_t)(y * S3.gain);
	fifo_put(&S3.fifo, v, v);
}

static void s68_dac(void)
{
	box_run(&S3.ch, &S3.box, s68_time(), s68_emit);
	S3.ch.level = ((int32_t)S3.dacval - 512) * 64;
}

static void s68_pa(void *c, uint8_t d) { (void)c; S3.dacval = (uint16_t)((d << 2) | (S3.dacval & 3)); s68_dac(); }
static void s68_pb(void *c, uint8_t d)
{
	(void)c;
	const uint8_t z_mask = (uint8_t)~S3.pia.ddr_b;   // (port_b_z_mask: the input bits)
	if (S3.type == MSND_68K_SG) {
		S3.dacval = (uint16_t)((S3.dacval & ~3) | (d >> 6));
		s68_dac();
	}
	if (~z_mask & 0x10) S3.status = (uint8_t)((S3.status & ~1) | ((d >> 4) & 1));
	if (~z_mask & 0x20) S3.status = (uint8_t)((S3.status & ~2) | ((d >> 4) & 2));
	if (S3.type == MSND_68K_CSD) {
		S3.dacval = (uint16_t)((S3.dacval & ~3) | (d >> 6));
		s68_dac();
	}
}
static void s68_irq(void *c, int st)
{
	(void)c; (void)st;
	m68k_bus_set_line(4, S3.pia.irq_a_state | S3.pia.irq_b_state);
	if (S3.in_exec)
		m68k_end_timeslice();
}

// the PIA: SG at 0x060000-0x06ffff (D8-D15), CSD at 0x018000-0x01bfff (either byte lane)
static int s68_pia_at(uint32_t a)
{
	if (S3.type == MSND_68K_SG)
		return (a & 0x070000) == 0x060000;
	return (a & 0x01c000) == 0x018000;
}

static uint8_t s68_read8(uint32_t a)
{
	a &= S3.type == MSND_68K_SG ? 0x7ffff : 0x1ffff;
	if (s68_pia_at(a)) {
		if (S3.type == MSND_68K_SG && (a & 1))
			return 0xff;
		return pia6821_read(&S3.pia, pia_alt((a >> 1) & 3));
	}
	return 0xff;
}
static uint16_t s68_read16(uint32_t a)
{
	a &= S3.type == MSND_68K_SG ? 0x7ffff : 0x1ffff;
	if (s68_pia_at(a)) {
		const uint8_t v = pia6821_read(&S3.pia, pia_alt((a >> 1) & 3));
		return S3.type == MSND_68K_SG ? (uint16_t)(v << 8 | 0xff) : (uint16_t)(v << 8 | v);
	}
	return 0xffff;
}
static void s68_write8(uint32_t a, uint8_t v)
{
	a &= S3.type == MSND_68K_SG ? 0x7ffff : 0x1ffff;
	if (s68_pia_at(a)) {
		if (S3.type == MSND_68K_SG && (a & 1))
			return;
		pia6821_write(&S3.pia, pia_alt((a >> 1) & 3), v);
	}
}
static void s68_write16(uint32_t a, uint16_t v)
{
	a &= S3.type == MSND_68K_SG ? 0x7ffff : 0x1ffff;
	if (s68_pia_at(a)) {
		// (both lanes reach the CSD's PIA: the MSB then the LSB, as MAME's two umask entries)
		pia6821_write(&S3.pia, pia_alt((a >> 1) & 3), (uint8_t)(v >> 8));
		if (S3.type == MSND_68K_CSD)
			pia6821_write(&S3.pia, pia_alt((a >> 1) & 3), (uint8_t)v);
	}
}

static void s68_select(void)
{
	m68k_bus_select(&s_bus68);
}

int msnd_68k_init(int type, const uint8_t *cpu_rom, uint32_t cpu_size, float gain)
{
	const uint32_t need = type == MSND_68K_SG ? 0x40000 : 0x8000;
	if (!cpu_rom || cpu_size < need)
		return -1;
	memset(&S3, 0, sizeof(S3));
	if (m68k_context_size() > sizeof(S3.ctx))
		return -1;
	dac_filters_setup();
	S3.type = type;
	S3.gain = gain;
	S3.rom_size = need;
	S3.rom = (uint8_t *)malloc(need);
	if (!S3.rom)
		return -1;
	memcpy(S3.rom, cpu_rom, need);
	m68k_bus_swap16(S3.rom, need);
	box_setup(&S3.box, 8000000.0);
	pia6821_config(&S3.pia);
	S3.pia.out_a_cb = s68_pa;
	S3.pia.out_b_cb = s68_pb;
	S3.pia.irqa_cb = s68_irq;
	S3.pia.irqb_cb = s68_irq;
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	memset(&s_bus68, 0, sizeof(s_bus68));
	s68_select();
	m68k_bus_reset_map();
	if (type == MSND_68K_SG) {
		// (global mask 0x7ffff: the 512 KB mirrored up to 16 MB)
		for (uint32_t base = 0; base < 0x1000000; base += 0x80000) {
			m68k_bus_map(base, base + 0x3ffff, S3.rom, need, 1, 0);
			m68k_bus_map(base + 0x70000, base + 0x7ffff, S3.ram, 0x1000, 1, 1);
		}
	} else {
		for (uint32_t base = 0; base < 0x1000000; base += 0x20000) {
			m68k_bus_map(base, base + 0x7fff, S3.rom, need, 1, 0);
			m68k_bus_map(base + 0x1c000, base + 0x1cfff, S3.ram, 0x1000, 1, 1);
		}
	}
	s_bus68.read8 = s68_read8;
	s_bus68.read16 = s68_read16;
	s_bus68.write8 = s68_write8;
	s_bus68.write16 = s68_write16;
	msnd_68k_reset();
	return 0;
}

void msnd_68k_exit(void)
{
	free(S3.rom);
	S3.rom = NULL;
	m68k_bus_select(NULL);
}

void msnd_68k_reset(void)
{
	S3.status = 0;
	S3.dacval = 0;
	S3.in_reset = 0;
	S3.now = 0;
	memset(&S3.ch, 0, sizeof(S3.ch));
	S3.ch.next = S3.box.P16;
	S3.ch.level = -512 * 64;
	memset(S3.w, 0, sizeof(S3.w));
	memset(S3.ram, 0, sizeof(S3.ram));
	S3.fifo.n = 0;
	s68_select();
	m68k_bus_set_line(4, 0);
	pia6821_reset(&S3.pia);
	m68k_pulse_reset();
}

void msnd_68k_write(uint8_t data)   // (Sounds Good synced_write)
{
	pia6821_portb_w(&S3.pia, (data >> 1) & 0x0f);
	pia6821_ca1_w(&S3.pia, ~data & 1);
}
void msnd_68k_sr_w(uint8_t data) { pia6821_portb_w(&S3.pia, data & 0x0f); }
void msnd_68k_sirq_w(int state) { pia6821_ca1_w(&S3.pia, !state); }
uint8_t msnd_68k_read(void) { return S3.status; }
void msnd_68k_suspend(int on) { S3.suspended = (uint8_t)on; }
int64_t msnd_68k_now(void) { return S3.now; }
msnd_fifo_t *msnd_68k_fifo(void) { return &S3.fifo; }

void msnd_68k_reset_write(int state)
{
	if (!state && S3.in_reset) {   // (released: the CPU starts from its vectors)
		s68_select();
		m68k_pulse_reset();
	}
	S3.in_reset = (uint8_t)(state != 0);
}

void msnd_68k_run(int64_t cycles)
{
	if (S3.suspended || S3.in_reset || !S3.rom) {
		if (S3.now < cycles)
			S3.now = cycles;
	} else {
		s68_select();
		while (S3.now < cycles) {
			S3.in_exec = 1;
			const int used = m68k_execute((int)(cycles - S3.now));
			S3.in_exec = 0;
			S3.now += used;
		}
	}
	box_run(&S3.ch, &S3.box, S3.now, s68_emit);
}

static int s68_state(void *ctx, int load)
{
	int rc = 0;
	s68_select();
	m68k_get_context(S3.ctx);
	S3.irq_asserted = s_bus68.irq_asserted;
	rc |= rbx_state_io(ctx, "ms68", &S3, (uint32_t)offsetof(__typeof__(S3), pia));
	rc |= rbx_state_io(ctx, "ms68_pia", &S3.pia, PIA6821_STATE_SIZE);
	if (!rc && load) {
		m68k_set_context(S3.ctx);
		s_bus68.irq_asserted = S3.irq_asserted;
		m68k_bus_update_irq();
	}
	return rc;
}

// =====================================================================================================================
// Squawk & Talk (M6802, two PIAs, AD558 DAC, TMS5200)
// =====================================================================================================================
#define SNT_CPU (3579545.0 / 4)
#define SNT_TMS 640000u

static struct {
	uint8_t select, irq, pad_[2];
	int32_t hp_x, tms_frac;
	int64_t hp_y;
	int64_t tms_cyc;        // the CPU time the TMS has been run to
	int32_t tms_level, tms_hp_x;
	int64_t tms_hp_y;
	boxch_t ch;
	uint8_t ram[0x80];
	m6800_t cpu;
	pia6821_t pia1, pia2;
	tms5220_t tms;
	// configuration
	const uint8_t *rom;
	boxcfg_t box;
	float gain;
	msnd_fifo_t fifo;
} S4;

static void snt_emit(int32_t s)
{
	// the DAC's high pass (2 kohm, 2 uF: 39.8 Hz) at 0.75, the speech's AC filter
	const int32_t k = 32768 - (int32_t)(32768.0 * 2 * 3.14159265 * 39.8 / MSND_RATE);
	const int64_t y = ((int64_t)(s - S4.hp_x) << 16) + ((S4.hp_y * k) >> 15);
	S4.hp_x = s;
	S4.hp_y = y;
	int32_t v = (int32_t)((y >> 16) * 3 / 4);
	const int32_t k2 = 32768 - (int32_t)(32768.0 * 2 * 3.14159265 * 5 / MSND_RATE);
	const int64_t y2 = ((int64_t)(S4.tms_level - S4.tms_hp_x) << 16) + ((S4.tms_hp_y * k2) >> 15);
	S4.tms_hp_x = S4.tms_level;
	S4.tms_hp_y = y2;
	v += (int32_t)(y2 >> 16);
	v = (int32_t)((float)v * S4.gain);
	fifo_put(&S4.fifo, v, v);
}

static void snt_tms_to(int64_t now)   // run the TMS5200 up to the CPU's time
{
	if (now <= S4.tms_cyc)
		return;
	// TMS clocks = CPU clocks x 640000 / 894886.25 (the remainder kept)
	const uint64_t num = (uint64_t)(now - S4.tms_cyc) * (SNT_TMS * 4ull) + (uint64_t)S4.tms_frac;
	const uint32_t clocks = (uint32_t)(num / 3579545ull);
	S4.tms_frac = (int32_t)(num - (uint64_t)clocks * 3579545ull);
	S4.tms_cyc = now;
	int16_t out[64];
	uint32_t left = clocks;
	while (left) {
		const uint32_t c = left > 4000 ? 4000 : left;
		const int n = tms5220_advance(&S4.tms, c, out, 64);
		if (n)
			S4.tms_level = out[n - 1];
		left -= c;
	}
}

static void snt_sync(void)
{
	snt_tms_to(S4.cpu.cycles);
	box_run(&S4.ch, &S4.box, S4.cpu.cycles, snt_emit);
}

static void snt_irq(void *c, int st)
{
	(void)c; (void)st;
	m6800_set_irq(&S4.cpu, S4.pia1.irq_a_state | S4.pia1.irq_b_state | S4.pia2.irq_a_state | S4.pia2.irq_b_state);
}
static uint8_t snt_pia1_ina(void *c) { (void)c; snt_sync(); return tms5220_status_r(&S4.tms); }
static void snt_pia1_outa(void *c, uint8_t d) { (void)c; snt_sync(); tms5220_data_w(&S4.tms, d); }
static void snt_pia1_outb(void *c, uint8_t d)
{
	(void)c;
	snt_sync();
	tms5220_rsq_w(&S4.tms, d & 1);
	tms5220_wsq_w(&S4.tms, (d >> 1) & 1);
}
static uint8_t snt_pia2_ina(void *c) { (void)c; return (uint8_t)(~S4.select & 0x1f); }
static void snt_tms_ready(void *c, int st) { (void)c; pia6821_ca2_w(&S4.pia1, st); }
static void snt_tms_irq(void *c, int st) { (void)c; pia6821_cb1_w(&S4.pia1, st); }

static uint8_t snt_rd(void *c, uint16_t a)
{
	(void)c;
	if (a < 0x80 && !(a & 0x4f00))   // (the 6802's internal RAM)
		return S4.ram[a & 0x7f];
	if ((a & 0x8000) == 0) {
		const uint16_t m = a & ~0x4f6c;
		if ((m & 0xfff0) == 0x0080) return pia6821_read(&S4.pia2, a & 3);
		if ((m & 0xfff0) == 0x0090) return pia6821_read(&S4.pia1, a & 3);
		return 0xff;
	}
	return S4.rom[0x8000 | (a & 0x3fff)];
}
static void snt_wr(void *c, uint16_t a, uint8_t v)
{
	(void)c;
	if (a < 0x80 && !(a & 0x4f00)) {
		S4.ram[a & 0x7f] = v;
		return;
	}
	if (a & 0x8000)
		return;
	const uint16_t m = a & ~0x4f6c;
	if ((m & 0xfff0) == 0x0080) { pia6821_write(&S4.pia2, a & 3, v); return; }
	if ((m & 0xfff0) == 0x0090) { pia6821_write(&S4.pia1, a & 3, v); return; }
	if ((a & ~0x40ff) == 0x1000) {   // the DAC (AD558)
		box_run(&S4.ch, &S4.box, S4.cpu.cycles, snt_emit);
		S4.ch.level = ((int32_t)v - 128) * 256;
	}
}

int msnd_snt_init(const uint8_t *cpu_rom, uint32_t cpu_size, float gain)
{
	if (!cpu_rom || cpu_size < 0x10000)
		return -1;
	memset(&S4, 0, sizeof(S4));
	S4.rom = cpu_rom;
	S4.gain = gain;
	box_setup(&S4.box, SNT_CPU);
	pia6821_config(&S4.pia1);
	pia6821_config(&S4.pia2);
	S4.pia1.in_a_cb = snt_pia1_ina;
	S4.pia1.out_a_cb = snt_pia1_outa;
	S4.pia1.out_b_cb = snt_pia1_outb;
	S4.pia1.irqa_cb = S4.pia1.irqb_cb = snt_irq;
	S4.pia2.in_a_cb = snt_pia2_ina;
	S4.pia2.irqa_cb = S4.pia2.irqb_cb = snt_irq;
	tms5220_init(&S4.tms, TMS5220_VARIANT_5200, SNT_TMS);
	tms5220_set_callbacks(&S4.tms, snt_tms_irq, snt_tms_ready, NULL);
	msnd_snt_reset();
	return 0;
}

void msnd_snt_reset(void)
{
	S4.select = 0;
	S4.hp_x = 0;
	S4.hp_y = 0;
	S4.tms_cyc = 0;
	S4.tms_frac = 0;
	S4.tms_level = 0;
	S4.tms_hp_x = 0;
	S4.tms_hp_y = 0;
	memset(&S4.ch, 0, sizeof(S4.ch));
	S4.ch.next = S4.box.P16;
	memset(S4.ram, 0, sizeof(S4.ram));
	S4.fifo.n = 0;
	m6800_init(&S4.cpu);
	S4.cpu.read = snt_rd;
	S4.cpu.write = snt_wr;
	for (int p = 0x80; p < 0x100; p++)
		S4.cpu.rpage[p] = S4.rom + 0x8000 + ((p * 256) & 0x3fff);
	pia6821_reset(&S4.pia1);
	pia6821_reset(&S4.pia2);
	tms5220_reset(&S4.tms);
	m6800_reset(&S4.cpu);
}

void msnd_snt_select(uint8_t data) { S4.select = data; }
void msnd_snt_int(int state) { pia6821_cb1_w(&S4.pia2, !state); }
int64_t msnd_snt_now(void) { return S4.cpu.cycles; }
msnd_fifo_t *msnd_snt_fifo(void) { return &S4.fifo; }

void msnd_snt_run(int64_t cycles)
{
	m6800_run_until(&S4.cpu, cycles);
	snt_tms_to(cycles);
	box_run(&S4.ch, &S4.box, cycles, snt_emit);
}

// =====================================================================================================================
// Williams D-11581 background board (src/mame/shared/s11c_bg.cpp, BSD-3-Clause, copyright-holders: Barry Rodewald,
// Jonathan Gevaryahu): MC6809E at 2 MHz, YM2151 (rbx_ym chip 0), PIA, MC1408 DAC, HC-55516 behind its two filters
// =====================================================================================================================
#include "../../sound/hc55516.h"
#include "../../sound/rbx_ym.h"

static struct {
	uint8_t bank, old_resetq, ym_reset, pad_;
	int32_t ym_frac;
	boxch_t dac, cvsd;
	float f1[3], f2[3];
	uint8_t ram[0x800];
	m6809_t cpu;
	pia6821_t pia;
	hc55516_t hc;
	// configuration
	const uint8_t *rom;
	uint32_t rom_size;
	boxcfg_t box;
	biq_t q1, q2;
	float dac_gain, cvsd_gain, gain;
	msnd_fifo_t fifo;
} S6;

static void bg_emit(int32_t s)
{
	// the CVSD's box-filtered level beside the DAC's (the same grid), its filters (MAME's 1/4 before, x 4 after)
	const int32_t cv = S6.cvsd.level;   // (sampled: the CVSD's clock is slow next to MSND_RATE)
	const float y = biquad(S6.f2, &S6.q2, biquad(S6.f1, &S6.q1, (float)cv * 0.25f));
	float v = (float)s * S6.dac_gain + y * S6.cvsd_gain;
	// the YM2151 at its own rate (3.579545 MHz / 64): the samples of this output sample's span, averaged
	S6.ym_frac += 3579545;
	const uint32_t clocks = (uint32_t)(S6.ym_frac / MSND_RATE);
	S6.ym_frac -= (int32_t)(clocks * MSND_RATE);
	int32_t ym[4] = { 0, 0, 0, 0 };
	const int n = rbx_ym_advance(0, clocks, ym, 4);
	int32_t ys = 0;
	for (int i = 0; i < n; i++)
		ys += ym[i];
	if (n > 1)
		ys /= n;
	v += (float)ys;
	const int32_t o = (int32_t)(v * S6.gain);
	fifo_put(&S6.fifo, o, o);
}

static void bg_map_bank(void)
{
	for (int p = 0x80; p < 0x100; p++) {
		const uint32_t o = (uint32_t)S6.bank * 0x8000 + (uint32_t)(p - 0x80) * 256;
		S6.cpu.rpage[p] = o + 256 <= S6.rom_size ? S6.rom + o : NULL;
	}
}

static void bg_common_reset(void)
{
	S6.bank = 0;
	bg_map_bank();
	const int64_t cy = S6.cpu.cycles;
	m6809_reset(&S6.cpu);
	S6.cpu.cycles = cy;
}

static void bg_sync(void) { box_run(&S6.dac, &S6.box, S6.cpu.cycles, bg_emit); }

static uint8_t bg_rd(void *c, uint16_t a)
{
	(void)c;
	if (a < 0x2000) return S6.ram[a & 0x7ff];
	if (a < 0x4000) { bg_sync(); return rbx_ym_read(0, a & 1); }
	if (a < 0x6000) return pia6821_read(&S6.pia, a & 3);
	return 0xff;   // (an unmapped bank: open bus)
}

static void bg_wr(void *c, uint16_t a, uint8_t v)
{
	(void)c;
	if (a < 0x2000) { S6.ram[a & 0x7ff] = v; return; }
	if (a < 0x4000) { bg_sync(); rbx_ym_write(0, a & 1, v); return; }
	if (a < 0x6000) { pia6821_write(&S6.pia, a & 3, v); return; }
	if (a < 0x6800) {   // bg_cvsd_clock_set_w
		bg_sync();
		hc55516_clock_w(&S6.hc, 1);
		S6.cvsd.level = hc55516_out(&S6.hc);
		return;
	}
	if (a < 0x7000) {   // bg_cvsd_digit_clock_clear_w
		bg_sync();
		hc55516_clock_w(&S6.hc, 0);
		hc55516_digit_w(&S6.hc, v & 1);
		S6.cvsd.level = hc55516_out(&S6.hc);
		return;
	}
	if (a >= 0x7800 && a < 0x8000) {   // bgbank_w: bits 1 0 3 2
		S6.bank = (uint8_t)(((v & 3) << 2) | ((v >> 2) & 3));
		bg_map_bank();
	}
}

static void bg_pa(void *c, uint8_t d) { (void)c; bg_sync(); S6.dac.level = ((int32_t)d - 128) * 256; }
static void bg_ca2(void *c, int st)
{
	(void)c;
	if (st != S6.ym_reset) {
		S6.ym_reset = (uint8_t)st;
		if (st) {
			bg_sync();
			rbx_ym_reset(0);
		}
	}
}
static void bg_irqa(void *c, int st) { (void)c; m6809_set_firq(&S6.cpu, st); }
static void bg_irqb(void *c, int st) { (void)c; m6809_set_nmi(&S6.cpu, st); }
static void bg_ym_irq(int st) { pia6821_ca1_w(&S6.pia, !st); }   // (IRQ is not true state)

int msnd_bg_init(int type, const uint8_t *cpu_rom, uint32_t cpu_size, float gain)
{
	if (!cpu_rom || cpu_size < 0x8000)
		return -1;
	memset(&S6, 0, sizeof(S6));
	S6.rom = cpu_rom;
	S6.rom_size = cpu_size;
	S6.gain = gain;
	box_setup(&S6.box, 2000000.0);
	biquad_setup(43000, 36000, 180000, 0, 180e-12, &S6.q1);
	biquad_setup(27000, 15000, 27000, 4700e-12, 1200e-12, &S6.q2);
	// the mixing resistors: S11C_BG (D-11581-20xx) and S11_BG (D-11581)
	const float dacr = type == MSND_BG_S11C ? 0.1304f : 0.1566f;
	const float ymr = type == MSND_BG_S11C ? 0.08473f : 0.0987f;
	const float cvr = type == MSND_BG_S11C ? 0.3396f : 0.1977f;
	S6.dac_gain = dacr;
	S6.cvsd_gain = cvr * 4.0f;
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = 3579545;
	yc.gain[0] = yc.gain[1] = (int)(ymr * 256 + 0.5f);
	yc.irq = bg_ym_irq;
	if (rbx_ym_init(0, &yc))
		return -1;
	pia6821_config(&S6.pia);
	S6.pia.out_a_cb = bg_pa;
	S6.pia.ca2_cb = bg_ca2;
	S6.pia.irqa_cb = bg_irqa;
	S6.pia.irqb_cb = bg_irqb;
	msnd_bg_reset();
	return 0;
}

void msnd_bg_exit(void) { rbx_ym_exit(0); }

void msnd_bg_reset(void)
{
	S6.bank = 0;
	S6.old_resetq = 1;
	S6.ym_reset = 0;
	S6.ym_frac = 0;
	memset(&S6.dac, 0, sizeof(S6.dac));
	memset(&S6.cvsd, 0, sizeof(S6.cvsd));
	S6.dac.next = S6.box.P16;
	S6.dac.level = -128 * 256;
	memset(S6.f1, 0, sizeof(S6.f1));
	memset(S6.f2, 0, sizeof(S6.f2));
	memset(S6.ram, 0, sizeof(S6.ram));
	S6.fifo.n = 0;
	hc55516_reset(&S6.hc);
	m6809_init(&S6.cpu);
	S6.cpu.read = bg_rd;
	S6.cpu.write = bg_wr;
	for (int p = 0; p < 0x20; p++)
		S6.cpu.rpage[p] = S6.cpu.wpage[p] = S6.ram + (p & 7) * 256;
	bg_map_bank();
	pia6821_reset(&S6.pia);
	rbx_ym_reset(0);
	m6809_reset(&S6.cpu);
}

void msnd_bg_data_w(uint8_t data) { pia6821_portb_w(&S6.pia, data); }
void msnd_bg_ctrl_w(int state) { pia6821_cb1_w(&S6.pia, state); }
void msnd_bg_extra_w(int state) { pia6821_cb2_w(&S6.pia, state); }
void msnd_bg_resetq_w(int state)
{
	if (S6.old_resetq && !state)   // (released: the board's reset)
		bg_common_reset();
	S6.old_resetq = (uint8_t)(state != 0);
}
int64_t msnd_bg_now(void) { return S6.cpu.cycles; }

void msnd_bg_run(int64_t cycles)
{
	m6809_run_until(&S6.cpu, cycles);
	box_run(&S6.dac, &S6.box, cycles, bg_emit);
}

static int bg_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "ms_bg", &S6, (uint32_t)offsetof(__typeof__(S6), cpu));
	rc |= rbx_state_io(ctx, "ms_bg_cpu", &S6.cpu, M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ms_bg_pia", &S6.pia, PIA6821_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ms_bg_hc", &S6.hc, sizeof(S6.hc));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "ms_bg_fifo", &S6.fifo, sizeof(S6.fifo));
	if (!rc && load)
		bg_map_bank();
	return rc;
}

// =====================================================================================================================
// the mix
// =====================================================================================================================
static struct {
	int32_t hp_x[2], pad_[2];
	int64_t hp_y[2];
} S5;

void msnd_mix_reset(void) { memset(&S5, 0, sizeof(S5)); }

static inline int32_t dc_block(int side, int32_t x)
{
	const int32_t k = 32768 - (int32_t)(32768.0 * 2 * 3.14159265 * 5 / MSND_RATE);
	const int64_t y = ((int64_t)(x - S5.hp_x[side]) << 16) + ((S5.hp_y[side] * k) >> 15);
	S5.hp_x[side] = x;
	S5.hp_y[side] = y;
	return (int32_t)(y >> 16);
}

void msnd_mix(int boards)
{
	msnd_fifo_t *f[5];
	int nf = 0;
	if (boards & MSND_B_SSIO) f[nf++] = &S1.fifo;
	if (boards & MSND_B_TCS) f[nf++] = &S2.fifo;
	if (boards & MSND_B_68K) f[nf++] = &S3.fifo;
	if (boards & MSND_B_SNT) f[nf++] = &S4.fifo;
	if (boards & MSND_B_BG) f[nf++] = &S6.fifo;
	if (!nf)
		return;
	int k = f[0]->n;
	for (int i = 1; i < nf; i++)
		if (f[i]->n < k)
			k = f[i]->n;
	for (int j = 0; j < k; j++) {
		int32_t l = 0, r = 0;
		for (int i = 0; i < nf; i++) {
			l += f[i]->l[j];
			r += f[i]->r[j];
		}
		rbx_audio_put_stereo(dc_block(0, l), dc_block(1, r));
	}
	for (int i = 0; i < nf; i++) {
		msnd_fifo_t *q = f[i];
		for (int j = k; j < q->n; j++) {
			q->l[j - k] = q->l[j];
			q->r[j - k] = q->r[j];
		}
		q->n -= k;
	}
}

int msnd_state(int boards, void *ctx, int load)
{
	int rc = 0;
	if (boards & MSND_B_SSIO) {
		rc |= rbx_state_io(ctx, "ms_ssio", &S1, (uint32_t)offsetof(__typeof__(S1), cpu));
		rc |= rbx_state_io(ctx, "ms_ssio_cpu", &S1.cpu, sizeof(S1.cpu));
		for (int i = 0; i < 2; i++)
			rc |= rbx_state_io(ctx, i ? "ms_ay1" : "ms_ay0", &S1.ay[i], ay8910_state_size());
		rc |= rbx_state_io(ctx, "ms_ssio_fifo", &S1.fifo, sizeof(S1.fifo));
		if (load)
			ssio_map();
	}
	if (boards & MSND_B_TCS) {
		rc |= rbx_state_io(ctx, "ms_tcs", &S2, (uint32_t)offsetof(__typeof__(S2), cpu));
		rc |= rbx_state_io(ctx, "ms_tcs_cpu", &S2.cpu, M6809_STATE_SIZE);
		rc |= rbx_state_io(ctx, "ms_tcs_pia", &S2.pia, PIA6821_STATE_SIZE);
		rc |= rbx_state_io(ctx, "ms_tcs_fifo", &S2.fifo, sizeof(S2.fifo));
	}
	if (boards & MSND_B_68K) {
		rc |= s68_state(ctx, load);
		rc |= rbx_state_io(ctx, "ms68_fifo", &S3.fifo, sizeof(S3.fifo));
	}
	if (boards & MSND_B_SNT) {
		rc |= rbx_state_io(ctx, "ms_snt", &S4, (uint32_t)offsetof(__typeof__(S4), cpu));
		rc |= rbx_state_io(ctx, "ms_snt_cpu", &S4.cpu, M6800_STATE_SIZE);
		rc |= rbx_state_io(ctx, "ms_snt_pia1", &S4.pia1, PIA6821_STATE_SIZE);
		rc |= rbx_state_io(ctx, "ms_snt_pia2", &S4.pia2, PIA6821_STATE_SIZE);
		rc |= tms5220_state(&S4.tms, ctx, load, "ms_snt_tms");
		rc |= rbx_state_io(ctx, "ms_snt_fifo", &S4.fifo, sizeof(S4.fifo));
	}
	if (boards & MSND_B_BG)
		rc |= bg_state(ctx, load);
	rc |= rbx_state_io(ctx, "ms_mix", &S5, sizeof(S5));
	return rc ? -1 : 0;
}
