/*  The Williams / Midway CVSD and ADPCM sound boards, ported to C from MAME src/mame/shared/williamssound.cpp
    (BSD-3-Clause, copyright-holders: Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "williams_snd.h"
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../machine/pia6821.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/okim6295.h"
#include "../../sound/hc55516.h"

#include <stdlib.h>
#include <string.h>

#define FM_CLOCK 3579545u
#define TICKS 8000000u          // the board's time unit (8 MHz)

static int s_type;
static m6809_t s_cpu;
static pia6821_t s_pia;
static okim6295_t s_oki;
static hc55516_t s_cvsd;
static uint8_t *s_rom, *s_okirom;
static uint32_t s_rom_size, s_oki_size;
static uint8_t s_ram[0x2000];
static uint8_t *s_hidden;                  // install_hidden_ram
static uint16_t s_hid_start, s_hid_end;
static uint16_t s_prot_start, s_prot_end;  // CVSD small: writes into ROM bank 0

// state (saved)
static struct {
	uint8_t bank, okibank, latch, talkback, sound_int_state, in_reset, dac, ym_reset_state;
	int64_t irq_clear_at;     // CPU cycle of the IRQ-state clear (-1 none)
	uint64_t sample;          // samples made
	uint64_t next_tick;       // the tick of the next sample (sample + 1) * 64 / 3579545 s, and its remainder
	uint32_t next_frac;
	uint32_t oki_acc;
	int32_t oki_out;
} S;

static void set_bank(uint8_t b);
static void bank_w(uint8_t b);
static void set_async(int on);
static void wms_sync(void);
static uint8_t s_int0;

// ---- the CPU's bus ----
static uint8_t rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (s_hidden && a >= s_hid_start && a <= s_hid_end)
		return s_hidden[a - s_hid_start];
	if (s_type == WMS_SND_CVSD) {
		if (a < 0x2000) return s_ram[a & 0x7ff];
		if (a < 0x4000) return rbx_ym_read(0, a & 1);
		if (a < 0x6000) return pia6821_read(&s_pia, a & 3);
		if (a < 0x8000) return 0xff;
		uint32_t o = 0x10000 + 0x8000 * ((S.bank >> 2) & 3) + 0x20000 * (S.bank & 3) + (a - 0x8000);
		return o < s_rom_size ? s_rom[o] : 0xff;
	}
	if (a < 0x2000) return s_ram[a];
	if (a < 0x4000) {
		switch (a & 0x3c00) {
		case 0x2400: return rbx_ym_read(0, a & 1);
		case 0x2c00: return oki_read(&s_oki);
		case 0x3000:   // command_r
			m6809_set_irq(&s_cpu, 0);
			S.irq_clear_at = s_cpu.cycles + 20;   // (10 us)
			return S.latch;
		}
		return 0xff;
	}
	uint32_t o = a < 0xc000 ? 0x10000 + S.bank * 0x8000u + (a - 0x4000) : 0x10000 + 0x4000 + 7 * 0x8000 + (a - 0xc000);
	return o < s_rom_size ? s_rom[o] : 0xff;
}

static void wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (s_hidden && a >= s_hid_start && a <= s_hid_end) {
		s_hidden[a - s_hid_start] = v;
		return;
	}
	if (s_type == WMS_SND_CVSD) {
		if (s_prot_end && a >= s_prot_start && a <= s_prot_end) {   // cvsd_protection_w
			uint32_t o = 0x10000 + (uint32_t)(a - 0x8000);
			if (o < s_rom_size) s_rom[o] = v;
			return;
		}
		if (a < 0x2000) { s_ram[a & 0x7ff] = v; return; }
		if (a < 0x4000) { rbx_ym_write(0, a & 1, v); return; }
		if (a < 0x6000) { pia6821_write(&s_pia, a & 3, v); return; }
		if (a < 0x6800) { hc55516_clock_w(&s_cvsd, 0); hc55516_digit_w(&s_cvsd, v & 1); return; }
		if (a < 0x7000) { hc55516_clock_w(&s_cvsd, 1); return; }
		if (a >= 0x7800 && a < 0x8000) bank_w(v & 0x0f);
		return;
	}
	if (a < 0x2000) { s_ram[a] = v; return; }
	switch (a & 0xfc00) {
	case 0x2000: bank_w(v & 7); break;
	case 0x2400: rbx_ym_write(0, a & 1, v); break;
	case 0x2800: S.dac = v; break;
	case 0x2c00: oki_write(&s_oki, v); break;
	case 0x3400: S.okibank = v & 7; break;
	case 0x3c00: S.talkback = v; break;
	}
}

// the 6809's direct pages: RAM and the banked ROM, rebuilt on a bank change; the pages of the protection overlays and
// the I/O stay on the handlers. set_bank: all pages (reset, state load, overlays); bank_w: the CPU's bank writes, only
// the banked pages and only when the bank changes (the other pages do not depend on it)
// (the banked pages' read pointers of every bank are made once by set_bank: a bank write copies its 128; their write
// pointers are always NULL)
static void set_bank_pages(uint8_t b, int lo, int hi);
static const uint8_t *s_bank_rpage[16][128];
static void set_bank(uint8_t b)
{
	const int lo = s_type == WMS_SND_CVSD ? 0x80 : 0x40;
	for (int k = 0; k < 16; k++) {
		set_bank_pages((uint8_t)k, lo, lo + 127);
		memcpy(s_bank_rpage[k], &s_cpu.rpage[lo], sizeof(s_bank_rpage[k]));
	}
	set_bank_pages(b, 0, 255);
}
static void bank_w(uint8_t b)
{
	if (b != S.bank) {
		S.bank = b;
		memcpy(&s_cpu.rpage[s_type == WMS_SND_CVSD ? 0x80 : 0x40], s_bank_rpage[b & 15], sizeof(s_bank_rpage[0]));
	}
}
static void set_bank_pages(uint8_t b, int lo, int hi)
{
	S.bank = b;
	for (int p = lo; p <= hi; p++) {
		uint16_t a = (uint16_t)(p << 8);
		const uint8_t *r = NULL;
		uint8_t *w = NULL;
		if (s_type == WMS_SND_CVSD) {
			if (a < 0x2000) { r = w = s_ram + (a & 0x7ff); }
			else if (a >= 0x8000) {
				uint32_t o = 0x10000 + 0x8000 * ((b >> 2) & 3) + 0x20000 * (b & 3) + (a - 0x8000);
				if (o + 0x100 <= s_rom_size) r = s_rom + o;
			}
		} else {
			if (a < 0x2000) { r = w = s_ram + a; }
			else if (a >= 0x4000) {
				uint32_t o = a < 0xc000 ? 0x10000 + b * 0x8000u + (a - 0x4000) : 0x10000 + 0x4000 + 7 * 0x8000 + (a - 0xc000);
				if (o + 0x100 <= s_rom_size) r = s_rom + o;
			}
		}
		if ((s_hidden && a <= s_hid_end && a + 0xff >= s_hid_start) || (s_prot_end && a <= s_prot_end && a + 0xff >= s_prot_start))
			r = NULL, w = NULL;
		if (a >= 0x8000)
			w = NULL;
		s_cpu.rpage[p] = r;
		s_cpu.wpage[p] = w;
	}
}

// ---- the CVSD board's PIA and the YM2151's IRQ ----
static void pia_out_a(void *ctx, uint8_t d) { (void)ctx; S.dac = d; }
static void pia_out_b(void *ctx, uint8_t d) { (void)ctx; S.talkback = d; }
static void pia_ca2(void *ctx, int st)
{
	(void)ctx;
	if (st != S.ym_reset_state) {   // (ym2151_device::reset_w)
		S.ym_reset_state = (uint8_t)st;
		if (st)
			rbx_ym_reset(0);
	}
}
static void pia_irqa(void *ctx, int st) { (void)ctx; m6809_set_firq(&s_cpu, st); }
static void pia_irqb(void *ctx, int st) { (void)ctx; m6809_set_nmi(&s_cpu, st); }
static void ym_irq(int st)
{
	if (s_type == WMS_SND_CVSD)
		pia6821_ca1_w(&s_pia, !st);   // (IRQ is not true state)
	else
		m6809_set_firq(&s_cpu, st);
}

// the OKI's 256 KB: 0x00000-0x1ffff the selected bank, 0x20000-0x3ffff the ROM at 0x60000
static uint8_t oki_rd(void *user, uint32_t a)
{
	(void)user;
	static const uint32_t base[8] = { 0x40000, 0x40000, 0x20000, 0x00000, 0xe0000, 0xc0000, 0xa0000, 0x80000 };
	uint32_t o = a < 0x20000 ? base[S.okibank] + a : 0x60000 + (a - 0x20000);
	return o < s_oki_size ? s_okirom[o] : 0;
}

int wms_snd_init(int type, uint8_t *cpu_rom, uint32_t cpu_size, uint8_t *oki_rom, uint32_t oki_size)
{
	s_type = type;
	s_rom = cpu_rom;
	s_rom_size = cpu_size;
	s_okirom = oki_rom;
	s_oki_size = oki_size;
	s_hidden = NULL;
	s_hid_start = s_hid_end = 0;
	s_prot_start = s_prot_end = 0;
	memset(&S, 0, sizeof(S));
	S.next_tick = 64u * TICKS / FM_CLOCK;
	S.next_frac = 64u * TICKS % FM_CLOCK;
	m6809_init(&s_cpu);
	s_cpu.read = rd;
	s_cpu.write = wr;
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = FM_CLOCK;
	yc.gain[0] = yc.gain[1] = 26;   // (0.10)
	yc.irq = ym_irq;
	if (rbx_ym_init(0, &yc))
		return -1;
	if (type == WMS_SND_CVSD) {
		pia6821_config(&s_pia);
		s_pia.out_a_cb = pia_out_a;
		s_pia.out_b_cb = pia_out_b;
		s_pia.ca2_cb = pia_ca2;
		s_pia.irqa_cb = pia_irqa;
		s_pia.irqb_cb = pia_irqb;
	} else {
		oki_init(&s_oki, 8000000 / 8, 1, NULL, 0x40000, 38);   // (0.15)
		s_oki.read = oki_rd;
	}
	set_async(1);   // (the board on the worker thread beside the TMS34010)
	return 0;
}

void wms_snd_exit(void)
{
	set_async(0);
	rbx_ym_exit(0);
	free(s_hidden);
	s_hidden = NULL;
}

void wms_snd_hidden_ram(uint16_t start, uint16_t end)
{
	free(s_hidden);
	s_hidden = (uint8_t *)calloc((size_t)(end - start + 1), 1);
	s_hid_start = start;
	s_hid_end = end;
	set_bank(S.bank);
}

void wms_snd_rom_protection(uint16_t start, uint16_t end)
{
	s_prot_start = start;
	s_prot_end = end;
	set_bank(S.bank);
}

static void board_reset(void)   // (the board's device_reset)
{
	S.sound_int_state = 0;
	m6809_set_firq(&s_cpu, 0);
	m6809_set_irq(&s_cpu, 0);
	m6809_set_nmi(&s_cpu, 0);
	S.irq_clear_at = -1;
}

void wms_snd_reset(void)
{
	wms_sync();
	s_int0 = 0;
	// (the board's time restarts with the machine's: wms_snd_run counts from 0 again)
	memset(&S, 0, sizeof(S));
	S.next_tick = 64u * TICKS / FM_CLOCK;
	S.next_frac = 64u * TICKS % FM_CLOCK;
	S.ym_reset_state = 1;
	S.irq_clear_at = -1;
	memset(s_ram, 0, sizeof(s_ram));
	if (s_hidden)
		memset(s_hidden, 0, (size_t)(s_hid_end - s_hid_start + 1));
	rbx_ym_reset(0);
	if (s_type == WMS_SND_CVSD) {
		pia6821_reset(&s_pia);
		pia6821_ca1_w(&s_pia, 1);   // (device_start: reset the IRQ state)
		hc55516_reset(&s_cvsd);
	} else
		oki_reset(&s_oki);
	set_bank(0);
	board_reset();
	m6809_reset(&s_cpu);
	s_cpu.cycles = 0;
}

static void write_now(uint16_t data)
{
	if (s_type == WMS_SND_CVSD) {   // (sync_write)
		pia6821_portb_w(&s_pia, data & 0xff);
		pia6821_cb1_w(&s_pia, (data >> 8) & 1);
		pia6821_cb2_w(&s_pia, (data >> 9) & 1);
	} else {                        // (sync_command)
		S.latch = data & 0xff;
		if (!(data & 0x200)) {
			m6809_set_irq(&s_cpu, 1);
			S.sound_int_state = 1;
		}
	}
}

static void reset_write_now(int state)
{
	if (state) {
		set_bank(0);
		board_reset();
		S.in_reset = 1;
	} else if (S.in_reset) {
		S.in_reset = 0;
		int64_t cyc = s_cpu.cycles;
		m6809_reset(&s_cpu);
		s_cpu.cycles = cyc;
	}
}


uint32_t wms_snd_rate(void) { return rbx_ym_rate(0); }

// the tick of sample k: k * 64 / 3579545 s
// the tick of the next sample: (S.sample + 1) * 64 * TICKS / FM_CLOCK, kept by steps (no 64-bit division a sample)
static inline void next_sample(void)
{
	S.sample++;
	S.next_frac += 64u * TICKS % FM_CLOCK;
	S.next_tick += 64u * TICKS / FM_CLOCK;
	if (S.next_frac >= FM_CLOCK) {
		S.next_frac -= FM_CLOCK;
		S.next_tick++;
	}
}

static void run_now(uint64_t ticks)
{
	for (;;) {
		uint64_t st = S.next_tick;
		if (st > ticks)
			st = ticks;
		int64_t target = (int64_t)(st / 4);   // (2 MHz)
		while (s_cpu.cycles < target) {
			int64_t t = target;
			if (S.irq_clear_at >= 0 && S.irq_clear_at < t)
				t = S.irq_clear_at > s_cpu.cycles ? S.irq_clear_at : s_cpu.cycles + 1;
			if (S.in_reset)
				s_cpu.cycles = t;
			else
				m6809_run_until(&s_cpu, t);
			if (S.irq_clear_at >= 0 && s_cpu.cycles >= S.irq_clear_at) {
				S.irq_clear_at = -1;
				S.sound_int_state = 0;
			}
		}
		if (S.next_tick > ticks)
			break;
		// one output sample
		int32_t ym = 0;
		rbx_ym_advance(0, 64, &ym, 1);
		int32_t mix = ym;
		if (s_type == WMS_SND_CVSD) {
			mix += (int32_t)S.dac * 32;                         // (MC1408, unsigned: 0.25)
			mix += hc55516_out(&s_cvsd) * 154 / 256;           // (0.60)
		} else {
			mix += (int32_t)S.dac * 13;                         // (AD7524, unsigned: 0.10)
			S.oki_acc += s_oki.rate;
			while (S.oki_acc >= rbx_ym_rate(0)) {
				S.oki_acc -= rbx_ym_rate(0);
				S.oki_out = oki_sample(&s_oki);
			}
			mix += S.oki_out;
		}
		rbx_audio_put(mix);
		next_sample();
	}
}

/*  The board on the worker thread (ReBOX, rbx_aux; RBX_WMS_NOTHREAD=1 = off): the host's runs, command writes and
    reset line changes are queued in their order and done by the job, so the board does exactly what it does inline
    while the host's CPU runs on. The host's IRQ read, the state, the reset and the frame's end (the samples) wait for
    the queue to be done first. Without a worker thread the kick runs the job at once. */
enum { Q_RUN, Q_WRITE, Q_RESET };
#define QSIZE 1024
static struct { uint8_t kind; uint16_t v; uint64_t t; } s_q[QSIZE];
static unsigned s_qhead, s_qtail;   // (the job's / the host's: atomics)
static int s_async;
// (the host's: the ADPCM board's sound_int_state is known to be 0. Only a command write sets it, so after a synced read
// of 0 it stays 0 until the next write: the reads need no round trip with the job)
static uint8_t s_int0;

static void wms_job(void)
{
	unsigned h = __atomic_load_n(&s_qhead, __ATOMIC_ACQUIRE);
	const unsigned t = __atomic_load_n(&s_qtail, __ATOMIC_ACQUIRE);
	for (; h != t; h = (h + 1) % QSIZE) {
		switch (s_q[h].kind) {
		case Q_RUN: run_now(s_q[h].t); break;
		case Q_WRITE: write_now(s_q[h].v); break;
		case Q_RESET: reset_write_now(s_q[h].v); break;
		}
		__atomic_store_n(&s_qhead, (h + 1) % QSIZE, __ATOMIC_RELEASE);
	}
}

// (profiling, the log every 300 frames: the host's syncs by kind, those with work still queued; no clock reads)
enum { SY_IRQ, SY_FRAME, SY_FULL, SY_OTHER };
static unsigned s_sy[4], s_sy_q, s_sy_known0;
extern uint64_t rbx_aux_sync_us[3];

static void wms_sync_k(int kind)
{
	if (s_async) {
		s_sy[kind]++;
		if (__atomic_load_n(&s_qhead, __ATOMIC_ACQUIRE) != s_qtail)
			s_sy_q++;
		rbx_aux_sync();
	}
}
static void wms_sync(void) { wms_sync_k(SY_OTHER); }

static void wms_stat(void)
{
	static unsigned f;
	static uint64_t wait0;
	if (++f % 300)
		return;
	const uint64_t wait = rbx_aux_sync_us[0];
	rbx_log("wms: host syncs/frame: irq reads %.1f (known 0 %.1f) frame %.1f full %.1f (work queued %.1f); "
	        "sync wait %.2f ms/frame", s_sy[SY_IRQ] / 300.0, s_sy_known0 / 300.0, s_sy[SY_FRAME] / 300.0,
	        s_sy[SY_FULL] / 300.0, s_sy_q / 300.0, (wait - wait0) / 300000.0);
	wait0 = wait;
	memset(s_sy, 0, sizeof(s_sy));
	s_sy_q = s_sy_known0 = 0;
}

static void q_put(int kind, uint16_t v, uint64_t t)
{
	const unsigned tail = s_qtail, next = (tail + 1) % QSIZE;
	if (next == __atomic_load_n(&s_qhead, __ATOMIC_ACQUIRE))
		wms_sync_k(SY_FULL);   // (full: the job catches up)
	s_q[tail].kind = (uint8_t)kind;
	s_q[tail].v = v;
	s_q[tail].t = t;
	__atomic_store_n(&s_qtail, next, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

static void set_async(int on)
{
	if (s_async) {
		rbx_aux_set_job(NULL);   // (syncs first)
		s_async = 0;
	}
	s_qhead = s_qtail = 0;
	if (on && !getenv("RBX_WMS_NOTHREAD")) {
		s_async = 1;
		rbx_aux_set_job(wms_job);
		rbx_aux_set_spin(300);   // (a job every scanline)
	}
}

void wms_snd_write(uint16_t data)
{
	s_int0 = 0;
	if (s_async)
		q_put(Q_WRITE, data, 0);
	else
		write_now(data);
}

void wms_snd_reset_write(int state)
{
	if (s_async)
		q_put(Q_RESET, (uint16_t)(state != 0), 0);
	else
		reset_write_now(state);
}

void wms_snd_run(uint64_t ticks)
{
	if (s_async)
		q_put(Q_RUN, 0, ticks);
	else
		run_now(ticks);
}

int wms_snd_irq_read(void)
{
	if (s_int0) {
		s_sy_known0++;
		return 0;
	}
	wms_sync_k(SY_IRQ);
	s_int0 = !S.sound_int_state;
	return S.sound_int_state;
}

int wms_snd_irq_known0(void) { return s_int0; }

void wms_snd_frame_end(void)
{
	wms_sync_k(SY_FRAME);
	wms_stat();
}

int wms_snd_state(void *ctx, int load)
{
	wms_sync();
	s_int0 = 0;
	if (rbx_state_io(ctx, "wsS", &S, sizeof(S)) || rbx_state_io(ctx, "wscpu", &s_cpu, M6809_STATE_SIZE) ||
		rbx_state_io(ctx, "wsram", s_ram, sizeof(s_ram)) || rbx_ym_state(0, ctx, load))
		return -1;
	if (s_hidden && rbx_state_io(ctx, "wshid", s_hidden, (uint32_t)(s_hid_end - s_hid_start + 1)))
		return -1;
	if (s_type == WMS_SND_CVSD) {
		if (rbx_state_io(ctx, "wspia", &s_pia, PIA6821_STATE_SIZE) || rbx_state_io(ctx, "wscvsd", &s_cvsd, sizeof(s_cvsd)))
			return -1;
		if (s_prot_end && rbx_state_io(ctx, "wsprot", s_rom + 0x10000 + (s_prot_start - 0x8000), (uint32_t)(s_prot_end - s_prot_start + 1)))
			return -1;
	} else {
		// (saved without the host pointers: the state's bytes do not depend on where the ROM is)
		const uint8_t *rom = s_oki.rom;
		uint8_t (*r)(void *, uint32_t) = s_oki.read;
		void *user = s_oki.user;
		okim6295_t o;
		memcpy(&o, &s_oki, sizeof(o));
		o.rom = NULL;
		o.read = NULL;
		o.user = NULL;
		if (rbx_state_io(ctx, "wsoki", &o, sizeof(o)))
			return -1;
		if (load)
			memcpy(&s_oki, &o, sizeof(o));
		s_oki.rom = rom;
		s_oki.read = r;
		s_oki.user = user;
	}
	if (load)
		set_bank(S.bank);
	return 0;
}
