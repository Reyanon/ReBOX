/*  Seibu Sound System. Ported from MAME 0.289 src/mame/shared/seibusound.cpp (BSD-3-Clause, copyright-holders: Bryan
    McPhail, R. Belmont) and src/mame/seibu/sei80bu.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail, R. Belmont).
    See seibusnd.h.
    This file is part of ReBOX. GPL-2.0-or-later.

    The Z80 is a device of the board's scheduler (sched.h) after its CPUs; MAME's synchronizes (the interrupt logic's
    updates, the FM chip's mode writes) stop it after the instruction, the FM chip's timers are the scheduler's.
    Cabal's variant (seibusnd_config.cabal): the SEI80BU over 0x0000-0x1fff only, no bank, no OKI; two Seibu ADPCM
    interfaces (seibu_adpcm_device) with their MSM5205s, whose VCK and capture timers are the scheduler's (MAME's
    msm5205_device toggle_vck / update_adpcm).
    Approximation: the OKI and the MSM5205s are resampled to the FM rate by holding their last sample.
*/
#include "seibusnd.h"
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/okim6295.h"
#include "../../sound/msm5205.h"
#include "sched.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

static seibusnd_config s_cfg;
#ifdef RBX_FG_PROF
unsigned long rbx_fgprof[20];
unsigned rbx_fgp_mpc[65536], rbx_fgp_spc[65536];
#if defined(__linux__) && defined(__x86_64__)
#include <linux/perf_event.h>
#include <sys/syscall.h>
#include <unistd.h>
unsigned long long rbx_fgp_ins(void)
{
	static int fd = -2;
	if (fd == -2) {
		struct perf_event_attr a;
		memset(&a, 0, sizeof(a));
		a.type = PERF_TYPE_HARDWARE;
		a.size = sizeof(a);
		a.config = PERF_COUNT_HW_INSTRUCTIONS;
		a.exclude_kernel = 1;
		a.exclude_hv = 1;
		fd = (int)syscall(__NR_perf_event_open, &a, 0, -1, -1, 0);
	}
	unsigned long long v = 0;
	if (fd >= 0 && read(fd, &v, 8) != 8)
		v = 0;
	return v;
}
#else
unsigned long long rbx_fgp_ins(void) { return 0; }
#endif
#endif
static z80 s_z;
static okim6295_t s_oki, s_oki2;
static uint8_t *s_data, *s_op;          // the decrypted (or plain) data / opcode images of the 64 KB space
static const uint8_t *s_rmap[256];
static int s_nbanks;
static uint32_t s_ym_rate;
static int32_t *s_mix;
static int s_mix_cap;
static msm5205_t s_msm[2];
static uint8_t *s_adpcm[2];             // (Cabal: the ADPCM ROMs, their lines swapped back)

static struct {
	uint8_t ram[0x800];
	uint8_t main2sub[2], sub2main[2];
	uint8_t main2sub_pending, sub2main_pending;
	uint8_t rst10_irq, rst18_irq, rst10_service, rst18_service;
	uint8_t bank, ym_irq, ym_addr, ym2_addr;
	int64_t busy_end;                   // the FM chip busy to then (attoseconds from the frame's start)
	int64_t busy2_end;                  // (Dead Angle's second YM2203)
	int64_t sec, att;                   // the frame's start since the reset (seconds, attoseconds)
	uint64_t ym_n;                      // the FM chip's samples made since the reset
	int32_t mix_n;                      // samples made this frame
	uint64_t oki_acc;                   // (the OKI's samples against the FM's)
	int32_t oki_hold;
	// Cabal's ADPCM interfaces (seibu_adpcm_device) and the MSM5205s' VCK
	uint16_t ad_current[2], ad_end[2];
	uint8_t ad_nibble[2], ad_playing[2], vck, oki_bank;
	int32_t oki2_hold;                  // (Raiden II's second OKI)
	uint8_t state_end;
} s;

// ---- SEI80BU (sei80bu_device data_r / opcode_r) ----
static inline int bit(unsigned v, int n) { return (v >> n) & 1; }
static inline uint8_t bs(uint8_t v, int b7, int b6, int b5, int b4, int b3, int b2, int b1, int b0)
{
	return (uint8_t)(bit(v, b7) << 7 | bit(v, b6) << 6 | bit(v, b5) << 5 | bit(v, b4) << 4 | bit(v, b3) << 3 |
	                 bit(v, b2) << 2 | bit(v, b1) << 1 | bit(v, b0));
}

static uint8_t dec_data(uint16_t a, uint8_t src)
{
	if (bit(a, 9) & bit(a, 8)) src ^= 0x80;
	if (bit(a, 11) & bit(a, 4) & bit(a, 1)) src ^= 0x40;
	if (bit(a, 11) & !bit(a, 8) & bit(a, 1)) src ^= 0x04;
	if (bit(a, 13) & !bit(a, 6) & bit(a, 4)) src ^= 0x02;
	if (!bit(a, 11) & bit(a, 9) & bit(a, 2)) src ^= 0x01;
	if (bit(a, 13) & bit(a, 4)) src = bs(src, 7, 6, 5, 4, 3, 2, 0, 1);
	if (bit(a, 8) & bit(a, 4)) src = bs(src, 7, 6, 5, 4, 2, 3, 1, 0);
	return src;
}

static uint8_t dec_op(uint16_t a, uint8_t src)
{
	if (bit(a, 9) & bit(a, 8)) src ^= 0x80;
	if (bit(a, 11) & bit(a, 4) & bit(a, 1)) src ^= 0x40;
	if (!bit(a, 13) & bit(a, 12)) src ^= 0x20;
	if (!bit(a, 6) & bit(a, 1)) src ^= 0x10;
	if (!bit(a, 12) & bit(a, 2)) src ^= 0x08;
	if (bit(a, 11) & !bit(a, 8) & bit(a, 1)) src ^= 0x04;
	if (bit(a, 13) & !bit(a, 6) & bit(a, 4)) src ^= 0x02;
	if (!bit(a, 11) & bit(a, 9) & bit(a, 2)) src ^= 0x01;
	if (bit(a, 13) & bit(a, 4)) src = bs(src, 7, 6, 5, 4, 3, 2, 0, 1);
	if (bit(a, 8) & bit(a, 4)) src = bs(src, 7, 6, 5, 4, 2, 3, 1, 0);
	if (bit(a, 12) & bit(a, 9)) src = bs(src, 7, 6, 4, 5, 3, 2, 1, 0);
	if (bit(a, 11) & !bit(a, 6)) src = bs(src, 6, 7, 5, 4, 3, 2, 1, 0);
	return src;
}

void seibusnd_sei80bu_decrypt(const uint8_t *src, uint8_t *data, uint8_t *op, uint32_t len)
{
	for (uint32_t a = 0; a < len; a++) {
		const uint8_t v = src[a];
		if (op)
			op[a] = dec_op((uint16_t)a, v);
		if (data)
			data[a] = dec_data((uint16_t)a, v);
	}
}

// the data / opcode images of the two banks (bank_w's bit 0), made once: a bank switch is the pointers'
static uint8_t *s_img[2], *s_imgop[2];
static void make_bank(int bank)
{
	uint8_t *d = s_img[bank], *o = s_imgop[bank];
	const uint8_t *src = s_nbanks ? s_cfg.rom + 0x10000 + (uint32_t)(bank % s_nbanks) * 0x8000
	                              : (s_cfg.rom_size >= 0x10000 ? s_cfg.rom + 0x8000 : NULL);
	const int enc = s_cfg.encrypted && !s_cfg.cabal && !s_cfg.enc_low;   // (Cabal, enc_low: the SEI80BU below 0x2000 only)
	for (uint32_t a = 0x8000; a < 0x10000; a++) {
		const uint8_t v = src ? src[a - 0x8000] : 0;
		d[a] = enc ? dec_data((uint16_t)a, v) : v;
		o[a] = enc ? dec_op((uint16_t)a, v) : v;
	}
}

static void set_bank(void)
{
	s_data = s_img[s.bank & 1];
	s_op = s_imgop[s.bank & 1];
	for (int p = 0; p < 256; p++)
		s_rmap[p] = (p < 0x20 || p >= 0x80) ? s_data + p * 256 : NULL;
	s_z.op_rom = s_op;
}

// ---- the interrupt logic (update_irq_synced; MAME's sync: here at once, seen after the instruction) ----
enum { VECTOR_INIT, RST10_ASSERT, RST10_CLEAR, RST10_ACKNOWLEDGE, RST10_EOI, RST18_ASSERT, RST18_ACKNOWLEDGE, RST18_EOI };
static int s_line;

static void update_irq(int param)
{
	switch (param) {
	case VECTOR_INIT: s.rst10_irq = s.rst18_irq = 0; s.rst10_service = s.rst18_service = 0; break;
	case RST10_ASSERT: s.rst10_irq = 1; break;
	case RST10_CLEAR: s.rst10_irq = 0; break;
	case RST10_ACKNOWLEDGE: s.rst10_service = 1; break;
	case RST10_EOI: s.rst10_service = 0; break;
	case RST18_ASSERT: s.rst18_irq = 1; break;
	case RST18_ACKNOWLEDGE: s.rst18_service = 1; s.rst18_irq = 0; break;
	case RST18_EOI: s.rst18_service = 0; break;
	default: break;
	}
	s_line = (s.rst10_irq && !s.rst10_service) || (s.rst18_irq && !s.rst18_service);
}

// im0_vector_cb (the acknowledge)
static uint8_t im0_vector(void)
{
	if (s.rst18_irq && !s.rst18_service) {
		update_irq(RST18_ACKNOWLEDGE);
		return 0xdf;
	}
	if (s.rst10_irq && !s.rst10_service) {
		update_irq(RST10_ACKNOWLEDGE);
		return 0xd7;
	}
	return 0x00;
}

static void ym_irq(int state)
{
	s.ym_irq = (uint8_t)state;
	update_irq(state ? RST10_ASSERT : RST10_CLEAR);
}

// ---- the sound to a time ----
static uint32_t s_ym_clocks;   // (the FM chip's clocks a sample)
static uint64_t s_run0;       // (the Z80's cyc at its run's start)
static int s_abort;

// the FM chip's samples to the time t (from the frame's start) as MAME's stream update makes them
// (sound_stream::update: the sample of the time and those before it, floor(t * rate) + 1, none at time 0); the chip's
// mode writes see this count (ymfm's timer B load)
static void snd_to(int64_t t)
{
	int64_t sec = s.sec, a = s.att + t;
	if (a >= SCHED_AS_PER_SEC) {
		a -= SCHED_AS_PER_SEC;
		sec++;
	}
	if (!sec && !a)
		return;
	// (floor(a * rate / 10^18) without 128 bits: a = q 10^9 + r)
	const uint64_t q = (uint64_t)a / 1000000000u, r = (uint64_t)a % 1000000000u;
	const uint64_t fl = (q * s_ym_rate + r * s_ym_rate / 1000000000u) / 1000000000u;
	const uint64_t n = (uint64_t)sec * s_ym_rate + fl + 1;
	if (n <= s.ym_n)
		return;
	const uint32_t clocks = (uint32_t)(n - s.ym_n) * s_ym_clocks;
	FGP(10, n - s.ym_n);
	FGP(11, 1);
	s.ym_n = n;
	const int k0 = s.mix_n;
	if (s.mix_n < s_mix_cap)
	{
		const int n0 = s.mix_n;
#ifdef RBX_FG_PROF
		const unsigned long long i0 = rbx_fgp_ins();
#endif
		s.mix_n += rbx_ym_advance(0, clocks, s_mix + s.mix_n, s_mix_cap - s.mix_n);
		FGP(18, rbx_fgp_ins() - i0);
		if (s_cfg.ym2)
			(void)rbx_ym_advance(1, clocks, s_mix + n0, s_mix_cap - n0);
	}
	if (s_cfg.oki_rom)
		for (int i = k0; i < s.mix_n; i++) {
			s.oki_acc += s_oki.rate;
			while (s.oki_acc >= s_ym_rate) {
				s.oki_acc -= s_ym_rate;
				s.oki_hold = oki_sample(&s_oki);
				if (s_cfg.oki2_rom)
					s.oki2_hold = oki_sample(&s_oki2);
			}
			s_mix[i] += s.oki_hold + s.oki2_hold;
		}
	if (s_cfg.cabal) {
		const int32_t v = ((msm5205_output(&s_msm[0]) + msm5205_output(&s_msm[1])) * s_cfg.adpcm_gain) >> 8;
		for (int i = k0; i < s.mix_n; i++)
			s_mix[i] += v;
	}
}

// an instruction that may end past the slice runs tentatively (MAME's would stop inside it, its accesses after the
// stop in the next slice): its RAM writes undoable, its device writes held, its device accesses noted
static int s_tent, s_tent_dev;
static uint16_t s_undo_a[8], s_q_a[4];
static uint8_t s_undo_v[8], s_q_v[4];
static int s_undo_n, s_q_n;

// ---- the Z80's bus (seibu_sound_map / the SEI80BU map) ----
static uint8_t z_read(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0x2000 && a < 0x2800)
		return s.ram[a - 0x2000];
	if (a >= 0x4000 && a < 0x4020) {
		s_tent_dev |= s_tent;
		switch (a) {
		case 0x4008: case 0x4009:
			snd_to(sched_now());
			return rbx_ym_read(0, a & 1);
		case 0x4010: case 0x4011: return s.main2sub[a & 1];
		case 0x4012: return s.sub2main_pending ? 1 : 0;
		case 0x4013: return s_cfg.coin_port >= 0 ? (uint8_t)rbx_port_read(s_cfg.coin_port) : 0;
		default: return s_cfg.encrypted && !s_cfg.cabal ? s_data[a] : 0;
		}
	}
	if (s_cfg.ym2 && (a == 0x6008 || a == 0x6009)) {
		s_tent_dev |= s_tent;
		snd_to(sched_now());
		return rbx_ym_read(1, a & 1);
	}
	if (a == 0x6000 && s_cfg.oki_rom && !s_cfg.cabal) {
		s_tent_dev |= s_tent;
		snd_to(sched_now());
		return oki_read(&s_oki);
	}
	if (a == 0x6002 && s_cfg.oki2_rom) {
		s_tent_dev |= s_tent;
		snd_to(sched_now());
		return oki_read(&s_oki2);
	}
	if ((!s_cfg.encrypted || s_cfg.cabal) && a >= 0x2000 && a < 0x8000)
		return 0;   // (unmapped)
	return s_data[a];
}

// Cabal's ADPCM interfaces (seibu_adpcm_device adr_w / ctl_w)
static void adpcm_w(int c, int reg, uint8_t v)
{
	if (reg == 0) {
		s.ad_current[c] = (uint16_t)(v << 8);
		s.ad_nibble[c] = 4;
	} else if (reg == 1) {
		s.ad_end[c] = (uint16_t)(v << 8);
	} else if (v == 0) {
		msm5205_reset_w(&s_msm[c], 1);
		s.ad_playing[c] = 0;
	} else if (v == 1) {
		msm5205_reset_w(&s_msm[c], 0);
		s.ad_playing[c] = 1;
	}
}

// the FM chip's mode register (ymfm_sync_mode_write: a synchronize)
static uint8_t mode_reg(void) { return s_cfg.ym_type == RBX_YM2151 ? 0x14 : s_cfg.ym_type == RBX_YM2203 ? 0x27 : 0x04; }

static void dev_write(uint16_t a, uint8_t v)
{
	if (s_cfg.ym2 && (a == 0x6008 || a == 0x6009)) {   // (Dead Angle's second YM2203)
		snd_to(sched_now());
		if (!(a & 1))
			s.ym2_addr = v;
		else if (s.ym2_addr == 0x27)
			s_abort = 1;
		rbx_ym_write(1, a & 1, v);
		return;
	}
	if (s_cfg.cabal) {
		switch (a) {
		case 0x4005: case 0x4006: adpcm_w(0, a - 0x4005, v); return;
		case 0x401a: adpcm_w(0, 2, v); return;
		case 0x6005: case 0x6006: adpcm_w(1, a - 0x6005, v); return;
		case 0x601a: adpcm_w(1, 2, v); return;
		case 0x4000: case 0x4007: if (!s_cfg.ym2) return; break;   // (Cabal: unmapped)
		case 0x6000: return;
		default: break;
		}
	}
	switch (a) {
	case 0x4000: s.main2sub_pending = 0; s.sub2main_pending = 1; break;   // pending_w
	case 0x4001: update_irq(RST18_EOI); s_abort = 1; break;              // irq_clear_w (a synchronize)
	case 0x4002: update_irq(RST10_EOI); s_abort = 1; break;
	case 0x4003: update_irq(RST18_EOI); s_abort = 1; break;
	case 0x4007: case 0x401a:
		if ((a == 0x401a) != (s_cfg.r2map != 0))
			break;   // (the bank_w of the map: 0x4007, the Raiden II boards' 0x401a)
		if (s_nbanks && (v & 1) != s.bank) {
			s.bank = v & 1;
			set_bank();
		}
		break;
	case 0x4008: case 0x4009:
		snd_to(sched_now());
		if (!(a & 1))
			s.ym_addr = v;
		else if (s.ym_addr == mode_reg())
			s_abort = 1;   // (the mode register: ymfm_sync_mode_write)
		rbx_ym_write(0, a & 1, v);
		break;
	case 0x4018: case 0x4019: s.sub2main[a & 1] = v; break;
	case 0x6000:
		if (s_cfg.oki_rom) {
			snd_to(sched_now());
			oki_write(&s_oki, v);
		}
		break;
	case 0x6002:
		if (s_cfg.oki2_rom) {
			snd_to(sched_now());
			oki_write(&s_oki2, v);
		}
		break;
	default: break;
	}
}

static void z_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x2000 && a < 0x2800) {
		if (s_tent && s_undo_n < 8) {
			s_undo_a[s_undo_n] = a;
			s_undo_v[s_undo_n++] = s.ram[a - 0x2000];
		}
		s.ram[a - 0x2000] = v;
		return;
	}
	if (s_tent) {
		if (a >= 0x4000 && s_q_n < 4) {
			s_q_a[s_q_n] = a;
			s_q_v[s_q_n++] = v;
			s_tent_dev = 1;
		}
		return;
	}
	dev_write(a, v);
}

static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
// (Godzilla: port 0 the OKI's ROM bank, 256 KB each)
static void oki_bank(int b)
{
	s.oki_bank = (uint8_t)(b & 1);
	const uint32_t off = s.oki_bank * 0x40000u;
	s_oki.rom = s_cfg.oki_rom + (off < s_cfg.oki_size ? off : 0);
	s_oki.rom_size = s_cfg.oki_size > 0x40000 ? 0x40000 : s_cfg.oki_size;
}

static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (s_cfg.oki_bank_port && p == 0) {
		snd_to(sched_now());
		oki_bank(v);
	}
}

// ---- the main CPU's side ----
uint8_t seibusnd_main_r(int offset)
{
	switch (offset) {
	case 2: case 3: return s.sub2main[offset - 2];
	case 5: return s.main2sub_pending ? 1 : 0;
	default: return 0xff;
	}
}

void seibusnd_main_w(int offset, uint8_t data)
{
	switch (offset) {
	case 0: case 1: s.main2sub[offset] = data; break;
	case 4: sched_sync(SEIBUSND_TIMER_SYNC, RST18_ASSERT); break;   // (update_irq_lines: a synchronize)
	case 2: case 6: s.sub2main_pending = 0; s.main2sub_pending = 1; break;
	default: break;
	}
}

// ---- running (the scheduler's device) ----
static unsigned s_ei_block;   // (an EI just ran: no interrupt before the next instruction)
static int s_ahead;           // (cycles run past the Z80's time as MAME's: its instruction's rest; < 0: not run yet)
static int s_in_insn;         // (inside an instruction: its accesses at its last M-cycle's start)

// The Z80 against MAME's (cpu/z80/z80.lst): MAME's stops a slice after a step of its instruction (z80_mcycles.inc),
// its accesses at a step's start, its interrupt checked before an instruction. Here an instruction runs whole; the
// time the scheduler sees is MAME's, the steps left carried (s_rest) to the next run.
#include "z80_mcycles.inc"

static uint8_t s_rest[24];            // (a state's steps loaded)
static const uint8_t *s_rp = s_rest;  // the steps left: s_rp[s_rest_i .. s_rest_n - 1] (s_rest or the paths' table)
static int s_rest_n, s_rest_i;

// the steps of the instruction at pc that took len cycles (MAME's path of that length): their count, 0 = none found
// (op0, op1: the opcode bytes at pc and pc + 1, d3: the data byte at pc + 3)
static int insn_steps(uint8_t op0, uint8_t op1, uint8_t d3, int halted, int len, uint8_t *out)
{
	int n = 0;
	out[n++] = 2;
	out[n++] = 2;
	if (halted)
		return len == 4 ? n : 0;
	const char *const *tbl = z80mc_main;
	uint8_t op = op0;
	switch (op) {
	case 0xcb: tbl = z80mc_cb; op = op1; out[n++] = 2; out[n++] = 2; break;
	case 0xed: tbl = z80mc_ed; op = op1; out[n++] = 2; out[n++] = 2; break;
	case 0xdd: case 0xfd: {
		const uint8_t b1 = op1;
		out[n++] = 2;
		out[n++] = 2;
		if (b1 == 0xcb) {
			tbl = z80mc_xycb;
			op = d3;
			out[n++] = 3; out[n++] = 3; out[n++] = 1; out[n++] = 1;
		} else {
			tbl = op == 0xdd ? z80mc_dd : z80mc_fd;
			op = b1;
		}
		break;
	}
	default: break;
	}
	int base = 0;
	for (int k = 0; k < n; k++)
		base += out[k];
	for (const unsigned char *p = (const unsigned char *)tbl[op]; *p; p += *p) {
		const int cnt = *p - 1;
		int sum = base;
		for (int k = 0; k < cnt; k++)
			sum += p[1 + k];
		if (sum == len && n + cnt <= 24) {
			for (int k = 0; k < cnt; k++)
				out[n + k] = p[1 + k];
			return n + cnt;
		}
	}
	return 0;
}

// the instruction (steps st[0..n-1]) stopped where MAME's would: after the first step reaching `want` cycles; the rest
// kept. Returns the cycles to there
static int stop_in(const uint8_t *st, int n, int want)
{
	int c = 0, k = 0;
	while (k < n && c < want)
		c += st[k++];
	s_rp = st + k;
	s_rest_n = n - k;
	s_rest_i = 0;
	return c;
}

// insn_steps precomputed (the same results): every path of every table's opcode as [len][n][the n steps, the fetches
// before the table's own], from s_stx[table * 256 + op] to s_stx[that + 1]; the halted fetch at s_stx[6 * 256]
enum { ST_MAIN, ST_CB, ST_ED, ST_DD, ST_FD, ST_XYCB, ST_HALT = 6 * 256 };
static uint8_t *s_stp;
static uint32_t *s_stx;

static int steps_build(void)
{
	if (s_stp)
		return 0;
	static const char *const *const tbls[6] = { z80mc_main, z80mc_cb, z80mc_ed, z80mc_dd, z80mc_fd, z80mc_xycb };
	static const uint8_t pre[6][8] = { { 2, 2 }, { 2, 2, 2, 2 }, { 2, 2, 2, 2 }, { 2, 2, 2, 2 }, { 2, 2, 2, 2 },
		{ 2, 2, 2, 2, 3, 3, 1, 1 } };
	static const int pre_n[6] = { 2, 4, 4, 4, 4, 8 };
	s_stx = (uint32_t *)calloc(6 * 256 + 2, sizeof(uint32_t));
	if (!s_stx)
		return -1;
	for (int pass = 0; pass < 2; pass++) {
		uint32_t o = 1;   // (0: none)
		for (int t = 0; t < 6; t++)
			for (int op = 0; op < 256; op++) {
				if (pass)
					s_stx[t * 256 + op] = o;
				int base = 0;
				for (int k = 0; k < pre_n[t]; k++)
					base += pre[t][k];
				for (const unsigned char *p = (const unsigned char *)tbls[t][op]; *p; p += *p) {
					const int cnt = *p - 1;
					int sum = base;
					for (int k = 0; k < cnt; k++)
						sum += p[1 + k];
					if (pre_n[t] + cnt > 24 || sum > 255)
						continue;   // (never found by insn_steps / no record's length)
					if (pass) {
						uint8_t *d = s_stp + o;
						d[0] = (uint8_t)sum;
						d[1] = (uint8_t)(pre_n[t] + cnt);
						memcpy(d + 2, pre[t], (size_t)pre_n[t]);
						memcpy(d + 2 + pre_n[t], p + 1, (size_t)cnt);
					}
					o += 2u + (uint32_t)(pre_n[t] + cnt);
				}
			}
		if (pass) {
			s_stx[ST_HALT] = o;
			s_stp[o] = 4;
			s_stp[o + 1] = 2;
			s_stp[o + 2] = s_stp[o + 3] = 2;
			s_stx[ST_HALT + 1] = o + 4;
		} else if (!(s_stp = (uint8_t *)calloc(o + 4, 1)))
			return -1;
	}
	return 0;
}

// insn_steps' path: the offset of its [n] in s_stp (0: none)
static inline uint32_t steps_find(uint8_t op0, uint8_t op1, uint8_t d3, int halted, int len)
{
	int i;
	if (halted)
		i = ST_HALT;
	else
		switch (op0) {
		case 0xcb: i = ST_CB * 256 + op1; break;
		case 0xed: i = ST_ED * 256 + op1; break;
		case 0xdd: i = op1 == 0xcb ? ST_XYCB * 256 + d3 : ST_DD * 256 + op1; break;
		case 0xfd: i = op1 == 0xcb ? ST_XYCB * 256 + d3 : ST_FD * 256 + op1; break;
		default: i = op0; break;
		}
	for (uint32_t o = s_stx[i], e = s_stx[i + 1]; o < e; o += 2u + s_stp[o + 1])
		if (s_stp[o] == len)
			return o + 1;
	return 0;
}

// (RBX_SND_STEPCHECK=1: steps_find against insn_steps for every opcode, length)
static void steps_check(void)
{
	long bad = 0, n = 0;
	for (int h = 0; h < 2; h++)
		for (int op0 = 0; op0 < 256; op0++)
			for (int op1 = 0; op1 < 256; op1++)
				for (int d3 = 0; d3 < 256; d3++) {
					if (d3 && !((op0 == 0xdd || op0 == 0xfd) && op1 == 0xcb))
						break;
					if (op1 && !h && op0 != 0xcb && op0 != 0xed && op0 != 0xdd && op0 != 0xfd)
						break;
					if ((op1 || d3) && h)
						break;
					for (int len = 0; len < 64; len++) {
						uint8_t st[24];
						const int k = insn_steps((uint8_t)op0, (uint8_t)op1, (uint8_t)d3, h, len, st);
						const uint32_t o = steps_find((uint8_t)op0, (uint8_t)op1, (uint8_t)d3, h, len);
						n++;
						if (!k != !o || (k && (s_stp[o] != k || memcmp(st, s_stp + o + 1, (size_t)k))))
							bad++;
					}
				}
	rbx_log("seibusnd: steps check %ld of %ld differ", bad, n);
}

// the interrupt (IM 0 with an RST on the bus, IM 1): MAME's acknowledge (a synchronize, the slice stopped) then 2
// cycles, the rest (11) after
static int take_irq(void)
{
	if (!s_line || !s_z.iff1 || s_z.iff_delay || s_ei_block || s_z.interrupt_mode > 1)
		return 0;
	const uint8_t v = im0_vector();
	if (s_z.interrupt_mode == 0 && (v & 0xc7) != 0xc7)
		return 1;   // (a spurious one: nothing on the bus, nothing done)
	s_z.halted = 0;
	s_z.iff1 = s_z.iff2 = 0;
	s_z.r = (uint8_t)((s_z.r & 0x80) | ((s_z.r + (s_z.interrupt_mode == 0 ? 2 : 1)) & 0x7f));
	s_in_insn = 1;
	s_z.sp = (uint16_t)(s_z.sp - 1);
	z_write(NULL, s_z.sp, (uint8_t)(s_z.pc >> 8));
	s_z.sp = (uint16_t)(s_z.sp - 1);
	z_write(NULL, s_z.sp, (uint8_t)s_z.pc);
	s_in_insn = 0;
	s_z.pc = s_z.interrupt_mode == 0 ? (uint16_t)(v & 0x38) : 0x38;
	s_z.mem_ptr = s_z.pc;
	s_z.cyc += 13;
	return 1;
}

// ---- lazy synchronization (seibusnd_set_lazy) ----
// The Z80 runs ahead of the scheduler's time while nothing it reads can change: below the first timer (the FM
// chip's, the board's) and below the peer CPU's own real position (the board's callback: the main CPU stops ahead of
// its accesses to the latches, so none of them falls before it). Each instruction run ahead leaves a record of what
// the scheduler's slices need of it (its cycles, steps, device access, abort, the interrupt taken before it); z_run
// replays the records as the instructions they stand for, with the same slices, stops and carried steps, until the
// queue is empty, then runs the Z80 itself again. Its device writes run ahead as the tight run makes them (held to
// the instruction's end), at the same machine times (sched_now through the time the slice would have).
typedef struct {
	uint8_t len, fl;
	uint32_t st;   // (its steps: steps_find's)
} snd_rec;
enum { REC_HALT = 1, REC_DEV = 2, REC_ABORT = 4, REC_IRQ = 8 };
#define SND_FIFO 16384             // (a power of two)
#define SND_MARGIN 96              // Z80 cycles kept from a limit (the tight run's slices around it)
static snd_rec *s_fifo;
static uint32_t s_fh, s_ft;        // head, tail
static unsigned long s_pend;       // the cycles of the records not replayed (s_z.cyc minus these: the tight run's)
static int s_lazy, s_exec = -1;
static int64_t (*s_peer)(void);
static seibusnd_mainq *s_mq;
static int replay(void);
// replay's constants (the scheduler's quantum and periods as at its last call, the cycle counts' thresholds:
// sched_div_cycles((delta >> shift), divisor) >= k  <=>  delta >= (k * divisor) << shift), see replay
// (-DRBX_REPLAY_DOUBLE: as doubles, the same results; slower under qemu-arm's soft VFP, not measured on the Vita)
#ifdef RBX_REPLAY_DOUBLE
typedef double replay_t;
#else
typedef int64_t replay_t;
#endif
static struct replay_c {
	replay_t qu, p0, p1, t0[9], t1[5];
	int64_t kqu, kp0, kp1;   // (the values they were made from)
} s_rc;
// Spin loops run ahead (ADR-063; RBX_FG_NOSPIN=1 off), as airraid.c's main CPU: a pass from a short backward jump's
// target back to it with no device access, no write, no interrupt, no ld r,a and the registers (all but R and the
// cycle count), the interrupt line and the EI block back as they were: its further passes' records are queued
// without running them while whole passes fit below the limit.
static int s_spin;
static int s_flush;   // (ADR-090: snd_ahead's FM samples handed over at its end; RBX_FG_NOFLUSH=1 off)
#define SPIN_MAX 24
static inline int z80_same_regs(const z80 *a, const z80 *b)   // (all but cyc and r)
{
	return !memcmp(&a->pc, &b->pc, offsetof(z80, r) - offsetof(z80, pc)) &&
	       !memcmp((const uint8_t *)a + offsetof(z80, r) + 1, (const uint8_t *)b + offsetof(z80, r) + 1,
	               offsetof(z80, nmi_pending) + sizeof(a->nmi_pending) - offsetof(z80, r) - 1);
}

void seibusnd_set_lazy(int on, int64_t (*peer)(void), seibusnd_mainq *mq)
{
	if (on && !s_fifo)
		s_fifo = (snd_rec *)malloc(SND_FIFO * sizeof(snd_rec));
	s_lazy = on && s_fifo != NULL;
	s_spin = s_lazy && !getenv("RBX_FG_NOSPIN");
	s_flush = s_lazy && s_cfg.ym_async && !getenv("RBX_FG_NOFLUSH");
	memset(&s_rc, 0, sizeof(s_rc));
	s_peer = peer;
	s_fh = s_ft = 0;
	s_pend = 0;
	s_mq = s_lazy && s_exec == 1 && !getenv("RBX_FG_NOREPLAY") ? mq : NULL;
	g_sched.replay = s_mq ? replay : NULL;
}

int seibusnd_lazy_pending(void) { return s_fh != s_ft; }

static void snd_ahead(int ran);

static int z_run_tight(int cycles)
{
	s_run0 = s_z.cyc - s_pend - (unsigned long)(long)s_ahead;   // (the Z80's time as MAME's)
	if (s_ahead >= cycles) {   // (the instruction's rest still: to the step reaching the slice's end)
		int c = 0;
		while (s_rest_i < s_rest_n && c < cycles)
			c += s_rp[s_rest_i++];
		if (s_rest_i >= s_rest_n)
			c = s_ahead;
		s_ahead -= c;
		return c;
	}
	int inside = s_ahead < 0;   // (an instruction begun in MAME's time, not run yet: no interrupt before it)
	s_ahead = 0;
	s_abort = 0;
	static const uint8_t irq_steps[4] = { 2, 5, 3, 3 };
	while (s_fh != s_ft) {   // the records of the instructions run ahead, as the loop below runs them
		const long pos = (long)(s_z.cyc - s_pend - s_run0);
		if (pos >= cycles || s_abort)
			goto out;
		const snd_rec *r = &s_fifo[s_fh & (SND_FIFO - 1)];
		if (r->fl & REC_IRQ) {   // (only before an instruction not begun: never recorded inside one)
			s_fh++;
			s_pend -= r->len;
			s_ahead = 11;
			s_abort = 0;
			stop_in(irq_steps, 4, 1);
			return (int)(s_z.cyc - s_pend - s_run0) - 11;
		}
		inside = 0;
		const int tent = cycles - pos < 24, len = r->len;
		const long end = pos + len;
		const int ab = (r->fl & REC_ABORT) != 0;
		if (end > cycles) {
			if (r->st) {
				const int sp = stop_in(s_stp + r->st + 1, s_stp[r->st], (int)(cycles - pos));
				if (tent && (r->fl & REC_DEV) && sp < len) {   // (all of it next slice: the record kept)
					s_rest_n = s_rest_i = 0;
					s_ahead = -sp;
					return (int)pos + sp;
				}
				s_fh++;
				s_pend -= (unsigned long)len;
				if (!ab) {
					s_ahead = len - sp;
					return (int)pos + sp;
				}
				s_abort = 1;
				s_rest_n = s_rest_i = 0;
				continue;
			}
		}
		s_fh++;
		s_pend -= (unsigned long)len;
		if (ab)
			s_abort = 1;
	}
	for (;;) {
		const long pos = (long)(s_z.cyc - s_run0);
		if (pos >= cycles || s_abort)
			break;
		if (!inside && take_irq()) {   // (the acknowledge's synchronize: the slice ends 2 cycles on)
			s_ahead = 11;
			s_abort = 0;
			stop_in(irq_steps, 4, 1);
			return (int)(s_z.cyc - s_run0) - 11;
		}
		inside = 0;
		const int ie = s_z.iff1, halted = s_z.halted;
		const uint16_t pc0 = s_z.pc;
		const int tent = cycles - pos < 24;
		z80 saved;
		if (tent) {
			saved = s_z;
			s_tent = 1;
			s_tent_dev = s_undo_n = s_q_n = 0;
		}
		s_z.int_pending = 0;
		s_in_insn = 1;
		z80_step(&s_z);
		FGP(5, 1);
		s_in_insn = 0;
		s_tent = 0;
		const long end = (long)(s_z.cyc - s_run0);
		if (end > cycles) {   // (MAME's stops inside: at the step reaching the slice's end)
			const int len = (int)(end - pos);
			const uint32_t so = steps_find(s_op[pc0], s_op[(uint16_t)(pc0 + 1)], s_data[(uint16_t)(pc0 + 3)], halted, len);
			if (so) {
				const int sp = stop_in(s_stp + so + 1, s_stp[so], (int)(cycles - pos));
				if (tent && s_tent_dev && sp < len) {   // (its device access after the stop: all of it next slice)
					s_z = saved;
					while (s_undo_n)
						s.ram[s_undo_a[s_undo_n - 1] - 0x2000] = s_undo_v[s_undo_n - 1], s_undo_n--;
					s_q_n = 0;
					s_rest_n = s_rest_i = 0;
					s_ahead = -sp;
					return (int)pos + sp;
				}
				if (tent) {
					s_in_insn = 1;
					for (int k = 0; k < s_q_n; k++)
						dev_write(s_q_a[k], s_q_v[k]);
					s_in_insn = 0;
					s_q_n = 0;
				}
				if (!s_abort) {
					if (!ie && s_z.iff1)
						s_ei_block = 1;
					else
						s_ei_block = 0;
					s_ahead = len - sp;
					return (int)pos + sp;
				}
				s_rest_n = s_rest_i = 0;   // (a synchronize: MAME's stops after the access, the instruction's end)
			}
		}
		if (tent) {
			s_in_insn = 1;
			for (int k = 0; k < s_q_n; k++)
				dev_write(s_q_a[k], s_q_v[k]);
			s_in_insn = 0;
			s_q_n = 0;
		}
		s_ei_block = !ie && s_z.iff1;
	}
out:
	s_abort = 0;
	s_ahead = 0;
	return (int)(s_z.cyc - s_pend - s_run0);
}

static int z_run(int cycles)
{
	const int ran = z_run_tight(cycles);
	if (s_lazy && s_fh == s_ft)
		snd_ahead(ran);
	return ran;
}

// The scheduler's slices (sched_run's loop) while both CPUs replay records: the peer's z_run (its queue consumed while
// the cycles done are below the slice's) and z_run_tight's record path, without their calls. A slice whose CPU would
// run for real (its queue emptied, or nothing left to replay) is left to sched_run: before its main CPU part, or
// after it through the sound Z80's own run (the slice finished as sched_run does).
static __attribute__((noinline)) void replay_consts(const sched_t *s)
{
	const sched_exec *const e0 = &s->ex[0], *const e1 = &s->ex[1];
	s_rc.qu = (replay_t)(s_rc.kqu = s->quantum);
	s_rc.p0 = (replay_t)(s_rc.kp0 = e0->period);
	s_rc.p1 = (replay_t)(s_rc.kp1 = e1->period);
	for (int k = 0; k < 9; k++)
		s_rc.t0[k] = (replay_t)(int64_t)(((uint64_t)k * (uint64_t)e0->divisor) << e0->shift);
	for (int k = 0; k < 5; k++)
		s_rc.t1[k] = (replay_t)(int64_t)(((uint64_t)k * (uint64_t)e1->divisor) << e1->shift);
}

static int replay(void)
{
	sched_t *const s = &g_sched;
	seibusnd_mainq *const q = s_mq;
	if (q->h == q->t || s_fh == s_ft || s->nex != 2 || s->ex[0].suspended || s->ex[1].suspended)
		return 0;
	sched_exec *const e0 = &s->ex[0], *const e1 = &s->ex[1];
	// The times relative to the slice's base (sched_run's loop: target as tq, the devices' local times as r0, r1,
	// the first timer's as rh), each moved by the slice's length at its end: fewer live 64-bit values than the
	// absolute times (the base folded in), the constants read from s_rc. Bounded below 2^50 (a quantum's few
	// cycles) so that RBX_REPLAY_DOUBLE's doubles (integers below 2^53) stay exact, the same results. A first
	// timer further than 2^50 (about a millisecond): slices while it stays a quantum away.
	const int64_t lim = (int64_t)1 << 50, b0 = s->base, x0 = e0->local - b0, x1 = e1->local - b0;
	if (x0 >= lim || x0 <= -lim || x1 >= lim || x1 <= -lim)
		return 0;
	if (s_rc.kqu != s->quantum || s_rc.kp0 != e0->period || s_rc.kp1 != e1->period)
		replay_consts(s);
	const int64_t xh = sched_head_expire() - b0;
	const replay_t rh0 = (replay_t)(xh > lim ? lim : xh), rlim = xh > lim ? s_rc.qu - 1 : 0;
	replay_t r0 = (replay_t)x0, r1 = (replay_t)x1, rh = rh0, tq = 0;
	const uint8_t *const mf = q->f;
	const uint32_t mm = q->mask;
	uint32_t mh = q->h;
	const long mav0 = (long)q->pend - mf[(q->t - 1) & mm];   // (the cycles consumable with a record left)
	long mav = mav0;
	uint32_t fh = s_fh;
	const long sav0 = (long)s_pend - s_fifo[(s_ft - 1) & (SND_FIFO - 1)].len;   // (likewise; < 0: none left)
	long sav = sav0;   // (s_pend and q->pend move with these: their sums at the end)
	int ahead = s_ahead, slices = 0, fallback = 0;
	const uint8_t *rp = s_rp;
	int rn = s_rest_n, ri = s_rest_i;
	// (z_run_tight's s_run0 is not kept: it is set at each run's start, read only inside a run)
	const struct replay_c *rc = &s_rc;
	while (rh > rlim) {
		__asm__("" : "+r"(rc));   // (the constants read where used: not hoisted into the stack's spills)
		tq = rh < rc->qu ? rh : rc->qu;
		replay_t delta = tq - r0;
		if (delta >= rc->p0) {
			if (delta >= rc->t0[8])
				break;
			int n = delta >= rc->t0[4] ? 4 : 0;
			if (delta >= rc->t0[n + 2])
				n += 2;
			if (delta >= rc->t0[n + 1])
				n++;
			if (mav < n)
				break;
			long pos = 0;
			do
				pos += mf[mh++ & mm];
			while (pos < n);
			mav -= pos;
			r0 += (replay_t)pos * rc->p0;
			if (r0 < tq)
				tq = r0 > 0 ? r0 : 0;
		}
		delta = tq - r1;
		if (delta >= rc->p1) {
			if (__builtin_expect(delta >= rc->t1[4] || sav < 0, 0)) {   // (more than a quantum's few cycles, or no record)
				fallback = -1;
				goto out;
			}
			const int n = 1 + (delta >= rc->t1[2]) + (delta >= rc->t1[3]);
			int ran;
			if (ahead >= n) {   // (the instruction's rest still)
				int c = 0;
				while (ri < rn && c < n)
					c += rp[ri++];
				if (ri >= rn)
					c = ahead;
				ahead -= c;
				ran = c;
			} else {
				if (__builtin_expect(ahead + sav < n, 0)) {   // (its records would run out: z_run_tight's own)
					fallback = n;
					goto out;
				}
				long pos = ahead;
				int ab = 0;
				ahead = 0;
				for (;;) {
					if (pos >= n || ab) {
						ran = (int)pos;
						break;
					}
					const snd_rec *r = &s_fifo[fh & (SND_FIFO - 1)];
					const int len = r->len;
					if (__builtin_expect(r->fl & REC_IRQ, 0)) {
						static const uint8_t irq_rest[3] = { 5, 3, 3 };   // (stop_in(irq_steps, 4, 1))
						fh++;
						sav -= len;
						ahead = 11;
						rp = irq_rest;
						rn = 3;
						ri = 0;
						ran = (int)pos + len - 11;
						break;
					}
					const long end = pos + len;
					if (end > n && r->st) {
						const uint8_t *st = s_stp + r->st + 1;
						const int k = st[-1], want = (int)(n - pos);
						int sp = 0, j = 0;
						while (j < k && sp < want)
							sp += st[j++];
						if (__builtin_expect(want < 24 && (r->fl & REC_DEV) && sp < len, 0)) {   // (all of it next slice: kept)
							rn = ri = 0;
							ahead = -sp;
							ran = (int)pos + sp;
							break;
						}
						fh++;
						sav -= len;
						if (__builtin_expect(!(r->fl & REC_ABORT), 1)) {
							rp = st + j;
							rn = k - j;
							ri = 0;
							ahead = len - sp;
							ran = (int)pos + sp;
							break;
						}
						ab = 1;
						rn = ri = 0;
						pos = end;
						continue;
					}
					fh++;
					sav -= len;
					pos = end;
					if (r->fl & REC_ABORT)
						ab = 1;
				}
			}
			r1 += (replay_t)ran * rc->p1;
			if (r1 < tq)
				tq = r1 > 0 ? r1 : 0;
		}
		r0 -= tq;
		r1 -= tq;
		rh -= tq;
		slices++;
	}
out:;
	const int64_t base = b0 + (int64_t)(rh0 - rh);
	s->base = base;
	e0->local = base + (int64_t)r0;
	e1->local = base + (int64_t)r1;
	q->h = mh;
	q->pend -= (unsigned long)(mav0 - mav);
	s_fh = fh;
	s_pend -= (unsigned long)(sav0 - sav);
	s_ahead = ahead;
	s_rp = rp;
	s_rest_n = rn;
	s_rest_i = ri;
	FGP(6, slices);
	FGP(7, fallback != 0);
	if (fallback) {   // (the slice's main CPU part done: its sound Z80 part run for real, the slice ended as sched_run's)
		int64_t target = base + (int64_t)tq;
		const int n = fallback > 0 ? fallback
			: sched_div_cycles((uint64_t)(target - e1->local) >> e1->shift, (uint64_t)e1->divisor);
		s->executing = 1;
		const int ran = e1->run(n);
		s->executing = -1;
		e1->local += (int64_t)ran * e1->period;
		if (e1->local < target)
			target = e1->local > base ? e1->local : base;
		s->base = target;
		slices++;
	}
	return slices;
}

// the Z80 run ahead from the tight run's time (its scheduler time after `ran` cycles) to the limit, its instructions
// recorded
static void snd_ahead(int ran)
{
	sched_exec *e = &g_sched.ex[s_exec];
	const int64_t local = e->local + (int64_t)ran * e->period;
	const unsigned long c0 = s_z.cyc - (unsigned long)(long)s_ahead;   // (the cycle count at that time)
	int64_t lim = s_peer ? s_peer() : INT64_MAX;
	const int64_t h = sched_head_expire();
	if (h < lim)
		lim = h;
	lim -= SND_MARGIN * e->period;
	if (lim <= local)
		return;
	unsigned long cend = c0 + (unsigned long)((lim - local) / e->period);
	if ((long)(s_z.cyc - cend) >= 0)
		return;
	const int64_t save_local = e->local;
	const unsigned long save_run0 = s_run0;
	const int save_exec = g_sched.executing;
	e->local = local;   // (sched_now: the machine time of the cycle count, as the tight run's)
	s_run0 = c0;
	g_sched.executing = s_exec;
	int inside = s_ahead < 0;
	int head = -1, pn = 0, hline = 0;   // (a spin loop's head watched, the records of its pass: see snd_spin)
	unsigned hblock = 0;
	unsigned long hcyc = 0;
	z80 hz;
	const unsigned long c1st = s_z.cyc;
	while ((long)(s_z.cyc - cend) < 0 && s_ft - s_fh < SND_FIFO) {
		snd_rec *r = &s_fifo[s_ft & (SND_FIFO - 1)];
		const unsigned long c1 = s_z.cyc;
		if (!inside && take_irq()) {
			r->len = (uint8_t)(s_z.cyc - c1);
			r->fl = REC_IRQ;
			s_ft++;
			s_pend += r->len;
			head = -1;
			continue;
		}
		inside = 0;
		const int ie = s_z.iff1, halted = s_z.halted;
		const uint16_t pc0 = s_z.pc;
		const uint8_t op0 = s_op[pc0], op1 = s_op[(uint16_t)(pc0 + 1)], d3 = s_data[(uint16_t)(pc0 + 3)];
		s_tent = 1;
		s_tent_dev = s_undo_n = s_q_n = 0;
		s_abort = 0;
		s_z.int_pending = 0;
		s_in_insn = 1;
		z80_step(&s_z);
		FGP(3, 1);
#ifdef RBX_FG_PROF
		rbx_fgp_spc[pc0]++;
#endif
		s_tent = 0;
		for (int k = 0; k < s_q_n; k++)
			dev_write(s_q_a[k], s_q_v[k]);
		s_in_insn = 0;
		s_q_n = 0;
		s_ei_block = !ie && s_z.iff1;
		r->len = (uint8_t)(s_z.cyc - c1);
		r->st = steps_find(op0, op1, d3, halted, r->len);
		r->fl = (uint8_t)((halted ? REC_HALT : 0) | (s_tent_dev ? REC_DEV : 0) | (s_abort ? REC_ABORT : 0));
		s_abort = 0;
		s_ft++;
		s_pend += r->len;
		if (s_spin) {
			if (head >= 0) {
				if (r->fl || s_undo_n || ++pn > SPIN_MAX || (op0 == 0xed && op1 == 0x4f))
					head = -1;   // (a device, a write, a halt, too long, or ld r,a: not a spin)
				else if (s_z.pc == head) {
					const unsigned long c = s_z.cyc - hcyc;
					if (s_line == hline && s_ei_block == hblock && z80_same_regs(&s_z, &hz)) {
						const long left = (long)(cend - s_z.cyc) - 1, room = (long)(SND_FIFO - (s_ft - s_fh));
						long k = left > 0 ? left / (long)c : 0;
						if (k > room / pn)
							k = room / pn;
						const uint32_t src = s_ft - (uint32_t)pn;
						for (long i = 0; i < k; i++)
							for (int j = 0; j < pn; j++, s_ft++)
								s_fifo[s_ft & (SND_FIFO - 1)] = s_fifo[(src + (uint32_t)j) & (SND_FIFO - 1)];
						FGP(4, k * pn);
						s_pend += (unsigned long)k * c;
						s_z.cyc += (unsigned long)k * c;
						s_z.r = (uint8_t)((s_z.r & 0x80) | ((s_z.r + (unsigned long)k * ((s_z.r - hz.r) & 0x7f)) & 0x7f));
					}
					hz = s_z;
					hcyc = s_z.cyc;
					hline = s_line;
					hblock = s_ei_block;
					pn = 0;
				}
			} else if (!r->fl && !s_undo_n && s_z.pc < pc0 && pc0 - s_z.pc <= 64) {
				head = s_z.pc;
				hz = s_z;
				hcyc = s_z.cyc;
				hline = s_line;
				hblock = s_ei_block;
				pn = 0;
			}
		}
		if (r->fl & REC_DEV) {   // (a timer of the FM chip's may have been set: the limit again)
			const int64_t h2 = sched_head_expire() - SND_MARGIN * e->period;
			if (h2 < lim) {
				lim = h2;
				cend = lim <= local ? c0 : c0 + (unsigned long)((lim - local) / e->period);
			}
		}
	}
	// (ADR-090) the FM samples to here handed to the worker now, not at the frame's end: what this Z80 does next
	// (its accesses, the FM timers below the limit) is at this time or after it, and the samples to a time are the same
	// however they are split (snd_to). Otherwise the samples after its last access in the frame were made at the frame's
	// end, the emulation thread waiting for them (or making them itself)
	if (s_flush && s_z.cyc != c1st)
		snd_to(sched_now());
	e->local = save_local;
	s_run0 = save_run0;
	g_sched.executing = save_exec;
}

// (inside an instruction: the time of its accesses, its last M-cycle's start)
static int64_t z_done(void)
{
	const long d = (long)(s_z.cyc - s_run0) - (s_in_insn ? 3 : 0);
	return d > 0 ? d : 0;
}
static void z_abort(void) { s_abort = 1; }

// the FM chip's timers (ymfm_set_timer: from now, MAME attotime::from_ticks)
// its busy flag (ymfm_set_busy_end / ymfm_is_busy: MAME's machine time against the end)
static void ym_busy_end(uint32_t clocks) { s.busy_end = sched_now() + sched_ticks(clocks, s_cfg.ym_clock); }
static int ym_is_busy(void) { return sched_now() < s.busy_end; }

static void ym2_busy_end(uint32_t clocks) { s.busy2_end = sched_now() + sched_ticks(clocks, s_cfg.ym_clock); }
static int ym2_is_busy(void) { return sched_now() < s.busy2_end; }
static void ym2_set_timer(int tnum, int32_t duration)
{
	if (duration < 0)
		sched_timer_off(SEIBUSND_TIMER_YM2 + tnum);
	else
		sched_timer_set(SEIBUSND_TIMER_YM2 + tnum, sched_now() + sched_ticks(duration, s_cfg.ym_clock), 0);
}

static void ym_set_timer(int tnum, int32_t duration)
{
	if (duration < 0)
		sched_timer_off(SEIBUSND_TIMER_YM + tnum);
	else
		sched_timer_set(SEIBUSND_TIMER_YM + tnum, sched_now() + sched_ticks(duration, s_cfg.ym_clock), 0);
}

// the MSM5205s' VCK half period (24 clocks of 375 kHz, MAME clocks_to_attotime) and their capture 6 clocks after
// its falling edge (attotime::from_ticks)
static int64_t s_vck_half, s_vck_cap;

int seibusnd_fire(int id, int32_t param)
{
	if (id == SEIBUSND_TIMER_VCK) {   // toggle_vck: the rising edge's nibble (seibu_adpcm_device msm_int)
		sched_timer_set(SEIBUSND_TIMER_VCK, sched_now() + s_vck_half, 0);
		s.vck ^= 1;
		if (s.vck) {
			for (int c = 0; c < 2; c++) {
				if (!s.ad_playing[c])
					continue;
				const uint8_t val = (uint8_t)((s_adpcm[c][s.ad_current[c] % s_cfg.adpcm_size[c]] >> s.ad_nibble[c]) & 15);
				msm5205_data_w(&s_msm[c], val);
				s.ad_nibble[c] ^= 4;
				if (s.ad_nibble[c] == 4 && ++s.ad_current[c] >= s.ad_end[c]) {
					msm5205_reset_w(&s_msm[c], 1);
					s.ad_playing[c] = 0;
				}
			}
		} else {
			sched_timer_set(SEIBUSND_TIMER_CAP, sched_now() + s_vck_cap, 0);
			sched_timer_set(SEIBUSND_TIMER_CAP + 1, sched_now() + s_vck_cap, 0);
		}
		return 1;
	}
	if (id == SEIBUSND_TIMER_CAP || id == SEIBUSND_TIMER_CAP + 1) {   // update_adpcm (one a chip)
		snd_to(sched_now());
		msm5205_clock(&s_msm[id - SEIBUSND_TIMER_CAP]);
		return 1;
	}
	if (id == SEIBUSND_TIMER_SYNC) {
		update_irq(param);
		return 1;
	}
	if (id == SEIBUSND_TIMER_YM2 || id == SEIBUSND_TIMER_YM2 + 1) {
		snd_to(sched_now());
		rbx_ym_timer_expired(1, id - SEIBUSND_TIMER_YM2);
		return 1;
	}
	if (id == SEIBUSND_TIMER_YM || id == SEIBUSND_TIMER_YM + 1) {
		snd_to(sched_now());
		rbx_ym_timer_expired(0, id - SEIBUSND_TIMER_YM);
		return 1;
	}
	return 0;
}

void seibusnd_frame_end(int64_t frame_as)
{
#ifdef RBX_FG_PROF
	const unsigned long tail0 = rbx_fgprof[10];
#endif
	snd_to(frame_as);
	FGP(16, rbx_fgprof[10] - tail0);
	if (s_cfg.ym_async) {   // (the FM samples of the frame from the worker, at the places snd_to gave them)
#ifdef RBX_FG_PROF
		const unsigned long long i0 = rbx_fgp_ins();
#endif
		rbx_ym_async_take(0, s_mix, s.mix_n);
		FGP(18, rbx_fgp_ins() - i0);
		if (s_cfg.ym2)
			rbx_ym_async_take(1, s_mix, s.mix_n);
	}
	for (int i = 0; i < s.mix_n; i++) {
		rbx_audio_put(s_mix[i]);
		s_mix[i] = 0;
	}
	s.mix_n = 0;
	s.busy_end = s.busy_end > frame_as ? s.busy_end - frame_as : 0;
	s.busy2_end = s.busy2_end > frame_as ? s.busy2_end - frame_as : 0;
	s.att += frame_as;
	while (s.att >= SCHED_AS_PER_SEC) {
		s.att -= SCHED_AS_PER_SEC;
		s.sec++;
	}
}

// ---- init ----
int seibusnd_init(const seibusnd_config *cfg)
{
	s_cfg = *cfg;
	memset(&s, 0, sizeof(s));
	for (int k = 0; k < 2; k++) {
		s_img[k] = (uint8_t *)malloc(0x10000);
		s_imgop[k] = (uint8_t *)malloc(0x10000);
		if (!s_img[k] || !s_imgop[k])
			return -1;
	}
	if (steps_build())
		return -1;
	if (getenv("RBX_SND_STEPCHECK"))
		steps_check();
	s_data = s_img[0];
	s_op = s_imgop[0];
	s_nbanks = s_cfg.rom_size > 0x10000 ? (int)((s_cfg.rom_size - 0x10000) / 0x8000) : 0;
	for (uint32_t a = 0; a < 0x8000; a++) {
		const uint8_t v = a < s_cfg.rom_size ? s_cfg.rom[a] : 0;
		const int enc = s_cfg.encrypted && ((!s_cfg.cabal && !s_cfg.enc_low) || a < 0x2000);
		s_data[a] = enc ? dec_data((uint16_t)a, v) : v;
		s_op[a] = enc ? dec_op((uint16_t)a, v) : v;
		if (s_cfg.predecrypted && a < 0x2000)   // (cabalbl2: its opcodes at 0x2000 in the ROM)
			s_op[a] = s_cfg.rom[a + 0x2000];
	}
	memcpy(s_img[1], s_img[0], 0x8000);
	memcpy(s_imgop[1], s_imgop[0], 0x8000);
	make_bank(0);
	make_bank(1);
	set_bank();
	if (s_cfg.cabal) {   // the ADPCM ROMs (seibu_adpcm_device::decrypt), the MSM5205s at 12 MHz / 32, 1/48
		for (int c = 0; c < 2; c++) {
			s_adpcm[c] = (uint8_t *)malloc(s_cfg.adpcm_size[c] ? s_cfg.adpcm_size[c] : 1);
			if (!s_adpcm[c])
				return -1;
			for (uint32_t i = 0; i < s_cfg.adpcm_size[c]; i++) {
				const uint8_t v = s_cfg.adpcm[c][i];
				s_adpcm[c][i] = (uint8_t)(bit(v, 7) << 7 | bit(v, 5) << 6 | bit(v, 3) << 5 | bit(v, 1) << 4 |
				                          bit(v, 6) << 3 | bit(v, 4) << 2 | bit(v, 2) << 1 | bit(v, 0));
			}
			msm5205_init(&s_msm[c], 375000, MSM5205_S48_4B, NULL);
		}
		s_vck_half = 24 * (SCHED_AS_PER_SEC / 375000);
		s_vck_cap = 6 * (SCHED_AS_PER_SEC / 375000);
	}
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = s_cfg.ym_type;
	yc.clock = s_cfg.ym_clock;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = s_cfg.ym_gain;
	yc.irq = ym_irq;
	yc.set_timer = ym_set_timer;
	yc.busy_end = ym_busy_end;
	yc.is_busy = ym_is_busy;
	if (rbx_ym_init(0, &yc) || (s_cfg.ym_async && rbx_ym_set_async(0, 1)))
		return -1;
	if (s_cfg.ym2) {   // (its IRQ not wired)
		yc.irq = NULL;
		yc.set_timer = ym2_set_timer;
		yc.busy_end = ym2_busy_end;
		yc.is_busy = ym2_is_busy;
		if (rbx_ym_init(1, &yc) || (s_cfg.ym_async && rbx_ym_set_async(1, 1)))
			return -1;
	}
	s_ym_rate = rbx_ym_rate(0);
	s_mix_cap = (int)(s_ym_rate / 30) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(s_ym_rate))
		return -1;
	if (s_cfg.oki_rom)
		oki_init(&s_oki, s_cfg.oki_clock, s_cfg.oki_pin7, s_cfg.oki_rom, s_cfg.oki_size, s_cfg.oki_gain);
	if (s_cfg.oki2_rom)
		oki_init(&s_oki2, s_cfg.oki_clock, s_cfg.oki_pin7, s_cfg.oki2_rom, s_cfg.oki2_size, s_cfg.oki_gain);
	z80_init(&s_z);
	s_z.read_byte = z_read;
	s_z.write_byte = z_write;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	// (the pages read directly: the fixed ROM, not the I/O at 0x2000-0x7fff)
	set_bank();
	s_z.read_map = s_rmap;
	s_z.op_rom = s_op;
	s_ym_clocks = (s_cfg.ym_clock + s_ym_rate / 2) / s_ym_rate;
	s_exec = sched_add_exec(s_cfg.z80_clock, z_run, z_done, z_abort);
	seibusnd_reset();
	return 0;
}

void seibusnd_exit(void)
{
	rbx_ym_exit(0);
	if (s_cfg.ym2)
		rbx_ym_exit(1);
	for (int k = 0; k < 2; k++) {
		free(s_img[k]);
		free(s_imgop[k]);
		s_img[k] = s_imgop[k] = NULL;
	}
	free(s_mix);
	free(s_fifo);
	s_fifo = NULL;
	s_lazy = 0;
	s_spin = 0;
	s_flush = 0;
	s_peer = NULL;
	s_mq = NULL;
	g_sched.replay = NULL;
	free(s_stp);
	free(s_stx);
	s_stp = NULL;
	s_stx = NULL;
	s_rp = s_rest;
	s_rest_n = s_rest_i = 0;
	s_fh = s_ft = 0;
	s_pend = 0;
	free(s_adpcm[0]);
	free(s_adpcm[1]);
	s_data = s_op = NULL;
	s_mix = NULL;
	s_adpcm[0] = s_adpcm[1] = NULL;
}

void seibusnd_reset(void)
{
	memset(&s, 0, offsetof(__typeof__(s), state_end));
	s_line = 0;
	s_ahead = 0;
	s_ei_block = 0;
	s_rest_n = s_rest_i = 0;
	s_rp = s_rest;
	z80_init(&s_z);
	s_z.read_byte = z_read;
	s_z.write_byte = z_write;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	set_bank();
	s_z.read_map = s_rmap;
	s_z.op_rom = s_op;
	sched_timer_off(SEIBUSND_TIMER_YM);
	sched_timer_off(SEIBUSND_TIMER_YM + 1);
	rbx_ym_reset(0);
	if (s_cfg.ym2) {
		sched_timer_off(SEIBUSND_TIMER_YM2);
		sched_timer_off(SEIBUSND_TIMER_YM2 + 1);
		rbx_ym_reset(1);
	}
	if (s_cfg.oki_rom) {
		oki_reset(&s_oki);
		if (s_cfg.oki_bank_port)
			oki_bank(0);
	}
	if (s_cfg.oki2_rom)
		oki_reset(&s_oki2);
	if (s_cfg.cabal) {   // (the VCK timers from the start: MAME's device_clock_changed; the chips held in reset)
		for (int c = 0; c < 2; c++) {
			msm5205_reset(&s_msm[c]);
			msm5205_reset_w(&s_msm[c], 1);
		}
		sched_timer_set(SEIBUSND_TIMER_VCK, s_vck_half, 0);
		sched_timer_off(SEIBUSND_TIMER_CAP);
		sched_timer_off(SEIBUSND_TIMER_CAP + 1);
	}
	update_irq(VECTOR_INIT);
	s_fh = s_ft = 0;
	s_pend = 0;
}

int seibusnd_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "seibusnd", &s, (uint32_t)offsetof(__typeof__(s), state_end));
	z80 zc = s_z;
	rc |= rbx_state_io(ctx, "seibusnd_z80", &zc, (uint32_t)sizeof(zc));
	rc |= rbx_ym_state(0, ctx, load);
	if (s_cfg.ym2)
		rc |= rbx_ym_state(1, ctx, load);
	if (s_cfg.cabal)
		rc |= rbx_state_io(ctx, "seibusnd_msm", s_msm, (uint32_t)sizeof(s_msm));
	if (s_cfg.oki_rom)
		rc |= rbx_state_io(ctx, "seibusnd_oki", &s_oki, (uint32_t)offsetof(okim6295_t, rom));
	if (s_cfg.oki2_rom)
		rc |= rbx_state_io(ctx, "seibusnd_oki2", &s_oki2, (uint32_t)offsetof(okim6295_t, rom));
	struct {   // (the instruction carried between the scheduler's slices)
		int32_t ahead, rest_n, rest_i, ei_block;
		uint8_t rest[24];
	} zs;
	memset(&zs, 0, sizeof(zs));
	zs.ahead = s_ahead;
	zs.rest_n = s_rest_n;
	zs.rest_i = s_rest_i;
	zs.ei_block = (int32_t)s_ei_block;
	if (s_rest_n > 0 && s_rest_n <= 24)
		memcpy(zs.rest, s_rp, (size_t)s_rest_n);
	rc |= rbx_state_io(ctx, "seibusnd_zrun", &zs, (uint32_t)sizeof(zs));
	if (load && !rc) {
		s_fh = s_ft = 0;
		s_pend = 0;
		s_ahead = zs.ahead;
		s_rest_n = zs.rest_n;
		s_rest_i = zs.rest_i;
		s_ei_block = (unsigned)zs.ei_block;
		memcpy(s_rest, zs.rest, sizeof(s_rest));
		s_rp = s_rest;
		zc.read_byte = z_read;
		zc.write_byte = z_write;
		zc.port_in = z_in;
		zc.port_out = z_out;
		zc.read_map = s_rmap;
		zc.op_rom = s_op;
		zc.userdata = NULL;
		s_z = zc;
		update_irq(-1);
		if (s_nbanks)
			set_bank();
		if (s_cfg.oki_bank_port && s_cfg.oki_rom)
			oki_bank(s.oki_bank);
	}
	return rc;
}
