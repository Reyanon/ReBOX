/*  The Atari JSA I / JSA II sound boards. Ported from MAME src/mame/atari/atarijsa.cpp and atariscom.cpp
    (BSD-3-Clause, copyright-holders: Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "atarijsa.h"

#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/pokey.h"
#include "../../sound/tms5220.h"
#include "../../sound/okim6295.h"

#include <stdlib.h>
#include <string.h>

#define YM_CLOCK 3579545
#define IRQ_PERIOD 7168            // set_periodic_int(JSA_MASTER_CLOCK / 4 / 16 / 16 / 14): in 6502 cycles
#define BOOST_CYC 1790             // SOUND_TIMER_BOOST (1 ms)

static jsa_config C;
static m6502_t s_cpu;
static uint8_t s_ram[0x2000];
static uint8_t s_bank;
static uint8_t s_m2s_data, s_s2m_data;
static int s_m2s_ready, s_s2m_ready;
static int s_timed_int, s_ym_int;
static int s_ct1, s_ct2;
static int s_ym_vol7, s_pk_vol3, s_tms_vol3, s_oki_vol2;   // the mixer's volumes: x / 7, x / 3, x / 3, x / 2
static int s_ym_reset;             // the YM2151 /RESET held low
static uint8_t s_ym_addr;
static int s_test;
static int32_t s_irq_left;
static int32_t s_boost;
static uint32_t s_ym_debt;         // YM clocks of the 6502's time not advanced yet

// the chips and their streams
static pokey_t s_pokey;
static tms5220_t s_tms;
static okim6295_t s_oki;
static uint32_t s_tms_clock;
static int s_oki_pin7;
static uint32_t s_oki_acc;
static int32_t s_oki_hold;
static int64_t s_cyc0, s_pk_done, s_tms_last;
static uint64_t s_tms_acc;
static uint32_t s_tms_frac;
static int32_t *s_ym_l, *s_ym_r;
static int s_ym_n, s_ym_cap;
static int16_t *s_pk_buf, *s_tms_buf;
static int s_pk_n, s_pk_cap, s_tms_n, s_tms_cap;
static int32_t *s_oki_buf;

static void pokey_sync(void);
static void tms_sync(void);

// ---- the POKEY on the second worker (RBX_NOPKASYNC: here) ----
// The samples pokey_sync asks for and the 6502's POKEY writes are queued; the worker runs them in the same order with
// the same sample counts and gains, so the chip goes through exactly the same calls as here. Reads of POT0-7 /
// ALLPOT / KBCODE (the 6502's dummy reads of sta (zp),y on the write registers) cannot change while no pot scan runs
// (no keyboard here) and no POTGO / SKCTL write is pending: they are answered from a copy taken when the worker was
// last caught up; other reads wait for it.
enum { PKQ_RENDER, PKQ_WRITE };
#define PKQ_SIZE 2048
static struct { uint8_t kind, off, val, g; int32_t n; } s_pkq[PKQ_SIZE];
static uint32_t s_pkq_in, s_pkq_out;
static int s_pk_async;
static int s_pk_wpos;               // (the worker's) next sample of s_pk_buf
static int s_pk_rsafe;              // s_pk_rd holds the reads of registers 0-9
static uint8_t s_pk_rd[10];

static void pk_render(int at, int n, int g)
{
	pokey_render(&s_pokey, s_pk_buf + at, n);
	for (int i = 0; i < n; i++)
		s_pk_buf[at + i] = (int16_t)(s_pk_buf[at + i] * g * 40 / 300);
}

static void pk_job(void)
{
	uint32_t o = s_pkq_out;
	const uint32_t in = __atomic_load_n(&s_pkq_in, __ATOMIC_ACQUIRE);
	for (; o != in; o++) {
		const uint32_t k = o & (PKQ_SIZE - 1);
		if (s_pkq[k].kind == PKQ_RENDER) {
			pk_render(s_pk_wpos, s_pkq[k].n, s_pkq[k].g);
			s_pk_wpos += s_pkq[k].n;
		} else
			pokey_write(&s_pokey, s_pkq[k].off, s_pkq[k].val);
		__atomic_store_n(&s_pkq_out, o + 1, __ATOMIC_RELEASE);
	}
}

static void pk_put(uint8_t kind, uint8_t off, uint8_t val, int g, int32_t n)
{
	const uint32_t i = s_pkq_in;
	if (i - __atomic_load_n(&s_pkq_out, __ATOMIC_ACQUIRE) >= PKQ_SIZE)
		rbx_aux2_sync();
	const uint32_t k = i & (PKQ_SIZE - 1);
	s_pkq[k].kind = kind;
	s_pkq[k].off = off;
	s_pkq[k].val = val;
	s_pkq[k].g = (uint8_t)g;
	s_pkq[k].n = n;
	__atomic_store_n(&s_pkq_in, i + 1, __ATOMIC_RELEASE);
}

static void pk_wait(void)   // the worker caught up: the chip and s_pk_buf are this thread's
{
	if (s_pk_async && __atomic_load_n(&s_pkq_out, __ATOMIC_ACQUIRE) != s_pkq_in)
		rbx_aux2_sync();
}

static void pk_snap(void)   // (caught up) the reads of registers 0-9, while no pot scan runs
{
	s_pk_rsafe = s_pokey.pot_counter >= 228;
	if (s_pk_rsafe)
		for (int i = 0; i < 10; i++)
			s_pk_rd[i] = pokey_read(&s_pokey, (uint8_t)i);
}

static void update_irq(void) { m6502_set_irq(&s_cpu, s_timed_int || s_ym_int); }

static void ym_irq(int state)
{
	s_ym_int = state;
	update_irq();
}

static void update_all_volumes(void)
{
	const int g = 154 * s_ym_vol7 / 7;   // 0.60 route x ym2151_volume
	rbx_ym_set_gain(0, 0, g);
	rbx_ym_set_gain(0, 1, g);
}

static void set_bank(uint8_t b)
{
	s_bank = b & 3;
	for (int p = 0x30; p < 0x40; p++)
		s_cpu.rpage[p] = C.rom + s_bank * 0x1000 + (p - 0x30) * 256;
}

static void oki_set_pin7(int high)
{
	s_oki_pin7 = high;
	s_oki.rate = (JSA_CPU_CLOCK * 2 / 3) / (high ? 132 : 165);
}

// ---- the 6502's map ----
static uint8_t rdio_r(void)
{
	uint8_t r = (uint8_t)(rbx_port_read(C.port) & 0x0f);   // the coins; +5V, (unused) low
	if (s_s2m_ready) r |= 0x20;                // output buffer full (custom, active high)
	if (!s_m2s_ready) r |= 0x40;               // input buffer full (custom, active low)
	// 0x80: the self test, main_test_read_line (!test, active high) and then flipped when !test: always low
	if (C.type == 1) {
		if (C.tms) {
			tms_sync();
			if (tms5220_readyq_r(&s_tms) == 0) r |= 0x10;
		}
	}
	return r;
}

static uint8_t jsa_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x2000)
		return s_ram[a];
	if (a < 0x2800)
		return a < 0x2002 ? rbx_ym_read(0, a & 1) : 0xff;
	if (a < 0x2a00) {
		switch (a & 6) {
		case 0: return C.type == 2 ? oki_read(&s_oki) : 0xff;
		case 2:   // sound_command_r
			s_m2s_ready = 0;
			m6502_set_nmi(&s_cpu, 0);
			return s_m2s_data;
		case 4: return rdio_r();
		default:   // sound_irq_ack_r
			s_timed_int = 0;
			update_irq();
			return 0;
		}
	}
	if (a >= 0x2c00 && a < 0x3000 && C.type == 1) {
		if (!C.pokey)
			return 0xff;
		if (s_pk_async && (a & 0x0f) < 10 && s_pk_rsafe)
			return s_pk_rd[a & 0x0f];
		pokey_sync();
		if (s_pk_async) {
			rbx_aux2_kick();
			pk_wait();
			pk_snap();
		}
		return pokey_read(&s_pokey, a & 0x0f);
	}
	return 0xff;
}

static void wrio_w(uint8_t data)
{
	set_bank((uint8_t)(data >> 6));
	if (C.type == 1) {
		if (C.tms) {
			const int count = 5 | ((data >> 2) & 2);
			const uint32_t clk = JSA_CPU_CLOCK * 4 / (uint32_t)(16 - count);
			tms_sync();
			if (clk != s_tms_clock) {
				s_tms_clock = clk;
				tms5220_set_clock(&s_tms, clk);
			}
			tms5220_wsq_w(&s_tms, (data >> 1) & 1);
			tms5220_rsq_w(&s_tms, (data >> 2) & 1);
		}
	} else {
		oki_set_pin7(data & 8);
		if (!(data & 4))
			oki_reset(&s_oki);
	}
	// the YM2151's reset_w: reset on going low; writes ignored while low
	if (!(data & 1) && !s_ym_reset)
		rbx_ym_reset(0);
	s_ym_reset = !(data & 1);
}

static void mix_w(uint8_t data)
{
	pokey_sync();
	tms_sync();
	if (C.type == 1) {
		s_tms_vol3 = (data >> 6) & 3;
		s_pk_vol3 = (data >> 4) & 3;
	} else
		s_oki_vol2 = (data & 1) ? 2 : 1;
	s_ym_vol7 = (data >> 1) & 7;
	update_all_volumes();
}

static void jsa_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x2000) {
		s_ram[a] = v;
		return;
	}
	if (a < 0x2800) {
		if (a < 0x2002 && !s_ym_reset) {
			if (!(a & 1))
				s_ym_addr = v;
			else if (s_ym_addr == 0x1b) {   // the port: CT1 / CT2 (ym2151_port_w)
				pokey_sync();
				tms_sync();
				s_ct1 = (v >> 6) & 1;
				s_ct2 = (v >> 7) & 1;
			}
			rbx_ym_write(0, a & 1, v);
		}
		return;
	}
	if (a < 0x2a00) {
		if ((a & 6) == 6) {   // sound_irq_ack_w
			s_timed_int = 0;
			update_irq();
		}
		return;
	}
	if (a < 0x2c00) {
		switch (a & 6) {
		case 0:
			if (C.type == 2)
				oki_write(&s_oki, v);
			else if (C.tms) {
				tms_sync();
				tms5220_data_w(&s_tms, v);
			}
			break;
		case 2:   // sound_response_w (delayed_6502_write)
			s_s2m_data = v;
			s_s2m_ready = 1;
			if (C.main_int)
				C.main_int(1);
			break;
		case 4: wrio_w(v); break;
		case 6: mix_w(v); break;
		}
		return;
	}
	if (a < 0x3000 && C.type == 1 && C.pokey) {
		pokey_sync();
		if (s_pk_async) {
			pk_put(PKQ_WRITE, (uint8_t)(a & 0x0f), v, 0, 0);
			if ((a & 0x0f) == 0x0b || (a & 0x0f) == 0x0f)   // POTGO, SKCTL: the pot reads may change
				s_pk_rsafe = 0;
		} else
			pokey_write(&s_pokey, a & 0x0f, v);
	}
}

// ---- the main CPU's side ----
void jsa_main_command_w(uint8_t data)
{
	s_m2s_data = data;
	s_m2s_ready = 1;
	m6502_set_nmi(&s_cpu, 1);
	s_boost = BOOST_CYC;
}

uint8_t jsa_main_response_r(void)
{
	s_s2m_ready = 0;
	if (C.main_int)
		C.main_int(0);
	return s_s2m_data;
}

void jsa_sound_reset_w(void)
{
	m6502_reset(&s_cpu);
	update_irq();
	m6502_set_nmi(&s_cpu, s_m2s_ready);
	s_s2m_ready = 0;
	if (C.main_int)
		C.main_int(0);
	s_boost = BOOST_CYC;
}

int jsa_main_to_sound_ready(void) { return s_m2s_ready; }
int jsa_sound_to_main_ready(void) { return s_s2m_ready; }
int jsa_boosting(void) { return s_boost > 0; }
void jsa_set_test(int state) { s_test = state; }

// ---- the 6502's idle loop (RBX_NOJSAIDLE: off) ----
// Toobin's main loop at 0x4141-0x4180 polls $280C and two queues in RAM, about 40 clocks a lap, while the slices are
// about 18 clocks. A lap is probed once on the CPU with a checking bus: it may read RAM and rdio_r only and write RAM
// only with the value already there, and must come back to its first boundary with the same register block (a fixed
// point). Its boundaries' register blocks and clocks are kept; a run starting on one of them with the same RAM bytes,
// rdio_r value and bank goes straight to the boundary where the interpreter would stop (the first at or past the
// target), with the same register block. Nothing else can change within a run: the interrupt lines, $280C's inputs
// and RAM change only between runs or through the 6502's own accesses, and the lap's accesses have no side effects.
#define IL_LO 0x4141
#define IL_HI 0x4181
#define IL_SNAP ((uint32_t)offsetof(m6502_t, cycles))
#define IL_MAXB 24
#define IL_MAXK 12
#define IL_ENT 4
typedef struct {
	int nb;                         // instruction boundaries in a lap
	int32_t lap;                    // the lap's clocks
	int32_t off[IL_MAXB];           // each boundary's clocks from the first
	int32_t dl[IL_MAXB];            // and to the next
	uint8_t st[IL_MAXB][IL_SNAP];   // each boundary's register block (pc .. inst_state)
	uint8_t cw[IL_MAXB];            // and cpu7_written
	uint16_t pc[IL_MAXB];           // and pc
	int nk;                         // the RAM bytes the lap reads or writes, and their values
	uint16_t ka[IL_MAXK];
	uint8_t kv[IL_MAXK];
	int io;                         // rdio_r read, and its value
	uint8_t iov;
	uint8_t bank;
	uint32_t age;
} il_entry;
static const uint8_t s_il_code[IL_HI - IL_LO] = {
	0xa5, 0x02, 0x29, 0xfe, 0x85, 0x02, 0xa9, 0x20, 0x2c, 0x0c, 0x28, 0xd0, 0x18, 0xac, 0x34, 0x02,
	0xcc, 0x35, 0x02, 0xf0, 0x10, 0xb9, 0x24, 0x02, 0x8d, 0x02, 0x2a, 0xc8, 0xc0, 0x10, 0x90, 0x02,
	0xa0, 0x00, 0x8c, 0x34, 0x02, 0xae, 0x20, 0x02, 0xec, 0x21, 0x02, 0xf0, 0x10, 0xe8, 0xe0, 0x20,
	0x90, 0x02, 0xa2, 0x00, 0x8e, 0x20, 0x02, 0xbc, 0x00, 0x02, 0x20, 0x93, 0x43, 0x4c, 0x41, 0x41 };
static int s_il_on;
static il_entry s_il[IL_ENT], s_il_probe_e;
static uint32_t s_il_clock;
static int s_il_bad;
static int s_il_k, s_il_i;   // the entry and boundary where the last run ended

static void il_clear(void)
{
	memset(s_il, 0, sizeof(s_il));
	s_il_clock = 0;
	s_il_k = s_il_i = 0;
	s_il_on = C.type == 1 && !C.tms && !getenv("RBX_NOJSAIDLE") && !memcmp(C.rom + IL_LO, s_il_code, sizeof(s_il_code));
}

static void il_key(uint16_t a, uint8_t v)
{
	il_entry *e = &s_il_probe_e;
	for (int i = 0; i < e->nk; i++)
		if (e->ka[i] == a)
			return;
	if (e->nk == IL_MAXK) {
		s_il_bad = 1;
		return;
	}
	e->ka[e->nk] = a;
	e->kv[e->nk++] = v;
}

static uint8_t il_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x2000) {
		il_key(a, s_ram[a]);
		return s_ram[a];
	}
	if (a >= 0x2800 && a < 0x2a00 && (a & 6) == 4) {
		const uint8_t v = rdio_r();
		s_il_probe_e.io = 1;
		s_il_probe_e.iov = v;
		return v;
	}
	s_il_bad = 1;
	return 0xff;
}

static void il_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x2000 && s_ram[a] == v)
		il_key(a, v);
	else
		s_il_bad = 1;
}

// one lap from the current boundary on the checking bus; the CPU is restored afterwards. 1: a fixed point
static int il_probe(void)
{
	il_entry *e = &s_il_probe_e;
	m6502_t *c = &s_cpu;
	uint8_t save[M6502_STATE_SIZE];
	const uint8_t cw0 = c->cpu7_written;
	memcpy(save, c, M6502_STATE_SIZE);
	for (int p = 0; p < 0x20; p++)
		c->rpage[p] = c->wpage[p] = NULL;
	c->read = il_r;
	c->write = il_w;
	e->nb = e->nk = e->io = 0;
	e->bank = s_bank;
	s_il_bad = 0;
	const uint16_t pc0 = c->pc;
	const int64_t t0 = c->cycles;
	int ok = 0;
	for (;;) {
		if (e->nb == IL_MAXB)
			break;
		memcpy(e->st[e->nb], c, IL_SNAP);
		e->cw[e->nb] = c->cpu7_written;
		e->pc[e->nb] = c->pc;
		e->off[e->nb++] = (int32_t)(c->cycles - t0);
		m6502_step(c);
		if (s_il_bad || c->halted || c->waiting || c->inst_state != -1 || (uint16_t)(c->pc - 1 - IL_LO) >= IL_HI - IL_LO ||
		    c->cycles - t0 > 256)
			break;
		if (c->pc == pc0) {
			ok = !memcmp(c, e->st[0], IL_SNAP) && c->cpu7_written == e->cw[0];
			e->lap = (int32_t)(c->cycles - t0);
			for (int i = 0; i < e->nb; i++)
				e->dl[i] = (i + 1 < e->nb ? e->off[i + 1] : e->lap) - e->off[i];
			break;
		}
	}
	memcpy(c, save, M6502_STATE_SIZE);
	c->cpu7_written = cw0;
	for (int p = 0; p < 0x20; p++)
		c->rpage[p] = c->wpage[p] = s_ram + p * 256;
	c->read = jsa_r;
	c->write = jsa_w;
	return ok;
}

// the boundary of e matching the CPU now (looked for from hint first), or -1
static int il_match(const il_entry *e, int hint, int *rd, uint8_t *rdv)
{
	if (!e->nb || e->bank != s_bank)
		return -1;
	int j = hint;
	if (e->pc[j] != s_cpu.pc) {
		j = 0;
		while (j < e->nb && e->pc[j] != s_cpu.pc)
			j++;
		if (j == e->nb)
			return -1;
	}
	if (memcmp(&s_cpu, e->st[j], IL_SNAP) || s_cpu.cpu7_written != e->cw[j])
		return -1;
	for (int i = 0; i < e->nk; i++)
		if (s_ram[e->ka[i]] != e->kv[i])
			return -1;
	if (e->io) {
		if (!*rd) {
			*rdv = rdio_r();
			*rd = 1;
		}
		if (*rdv != e->iov)
			return -1;
	}
	return j;
}

static void jsa_cpu_run(int64_t target)
{
	m6502_t *c = &s_cpu;
	if (s_il_on && c->cycles < target && !c->halted && !c->waiting && c->inst_state == -1 &&
	    (uint16_t)(c->pc - 1 - IL_LO) < IL_HI - IL_LO) {
		int rd = 0, k = s_il_k;
		uint8_t rdv = 0;
		int j = il_match(&s_il[k], s_il_i, &rd, &rdv);
		for (int t = 0; j < 0 && t < IL_ENT; t++)
			if (t != s_il_k && (j = il_match(&s_il[t], 0, &rd, &rdv)) >= 0)
				k = t;
		if (j < 0 && il_probe()) {   // (into the oldest entry)
			k = 0;
			for (int t = 1; t < IL_ENT; t++)
				if (s_il[t].age < s_il[k].age)
					k = t;
			s_il[k] = s_il_probe_e;
			j = 0;
		}
		if (j >= 0) {   // (the boundaries from j on, laps unrolled, to the first at or past the target)
			il_entry *e = &s_il[k];
			e->age = ++s_il_clock;
			const int64_t r = target - c->cycles;
			int64_t adv = 0;
			while (r - adv > e->lap)
				adv += e->lap;
			int i = j;
			while (adv < r) {
				adv += e->dl[i];
				if (++i == e->nb)
					i = 0;
			}
			memcpy(c, e->st[i], IL_SNAP);
			c->cpu7_written = e->cw[i];
			c->cycles += adv;
			s_il_k = k;
			s_il_i = i;
			return;
		}
	}
	m6502_run_until(c, target);
}

// ---- time ----
void jsa_run(int cycles)
{
	s_ym_debt += (uint32_t)cycles * 2;
	if (s_boost > 0)
		s_boost = s_boost > cycles ? s_boost - cycles : 0;
	while (cycles > 0) {
		const int n = cycles < s_irq_left ? cycles : s_irq_left;
		jsa_cpu_run(s_cpu.cycles + n);
		cycles -= n;
		s_irq_left -= n;
		if (s_irq_left <= 0) {   // sound_irq_gen
			s_irq_left += IRQ_PERIOD;
			s_timed_int = 1;
			update_irq();
		}
	}
}

static void pokey_sync(void)
{
	if (!C.pokey)
		return;
	const int64_t now = s_cpu.cycles - s_cyc0;
	if (now <= s_pk_done)
		return;
	const int64_t want = rbx_div64_small((uint64_t)now * pokey_rate(&s_pokey), JSA_CPU_CLOCK);
	const int64_t have = rbx_div64_small((uint64_t)s_pk_done * pokey_rate(&s_pokey), JSA_CPU_CLOCK);
	s_pk_done = now;
	int n = (int)(want - have);
	if (s_pk_n + n > s_pk_cap)
		n = s_pk_cap - s_pk_n;
	if (n <= 0)
		return;
	const int g = s_pk_vol3 * s_ct1;   // 0.40 route x pokey_volume x CT1
	if (s_pk_async)
		pk_put(PKQ_RENDER, 0, 0, g, n);
	else
		pk_render(s_pk_n, n, g);
	s_pk_n += n;
}

static void tms_sync(void)
{
	if (!C.tms)
		return;
	const int64_t now = s_cpu.cycles - s_cyc0;
	if (now <= s_tms_last)
		return;
	s_tms_acc += (uint64_t)(now - s_tms_last) * s_tms_clock;
	s_tms_last = now;
	const uint32_t clocks = (uint32_t)(s_tms_acc / JSA_CPU_CLOCK);
	s_tms_acc -= (uint64_t)clocks * JSA_CPU_CLOCK;
	s_tms_frac += clocks;
	int n = (int)(s_tms_frac / 80);
	s_tms_frac -= (uint32_t)n * 80;
	if (s_tms_n + n > s_tms_cap)
		n = s_tms_cap - s_tms_n;
	if (n <= 0)
		return;
	tms5220_render(&s_tms, s_tms_buf + s_tms_n, n);
	const int g = s_tms_vol3 * s_ct1;   // 1.0 route x tms5220_volume x CT1
	for (int i = 0; i < n; i++)
		s_tms_buf[s_tms_n + i] = (int16_t)(s_tms_buf[s_tms_n + i] * g / 3);
	s_tms_n += n;
}

void jsa_line(void)
{
	const int n = rbx_ym_advance_lr(0, s_ym_debt, s_ym_l + s_ym_n, s_ym_r + s_ym_n, s_ym_cap - s_ym_n);
	s_ym_debt = 0;
	if (C.type == 2) {   // the OKI at the YM's samples (held)
		const uint32_t yr = rbx_ym_rate(0);
		const int g = s_oki_vol2 * s_ct1;   // 0.75 route x oki6295_volume x CT1
		for (int i = 0; i < n; i++) {
			s_oki_acc += s_oki.rate;
			while (s_oki_acc >= yr) {
				s_oki_acc -= yr;
				s_oki_hold = oki_sample(&s_oki);
			}
			s_oki_buf[s_ym_n + i] = s_oki_hold * g * 3 / 8;
		}
	}
	s_ym_n += n;
	if (s_pk_async) {   // (the worker kept busy through the frame)
		pokey_sync();
		rbx_aux2_kick();
	}
}

void jsa_frame_begin(void)
{
	pk_wait();
	s_pk_wpos = 0;
	s_cyc0 = s_cpu.cycles;
	s_pk_done = s_tms_last = 0;
	s_pk_n = s_tms_n = 0;
}

void jsa_frame_end(void)
{
	pokey_sync();
	if (s_pk_async) {
		rbx_aux2_kick();
		pk_wait();
		pk_snap();
	}
	tms_sync();
	rbx_ym_async_take_lr(0, s_ym_l, s_ym_r, s_ym_n);
	const int gain = C.gain;
	for (int i = 0; i < s_ym_n; i++) {
		int32_t o = 0;
		if (s_pk_n)
			o += s_pk_buf[(int)((int64_t)i * s_pk_n / s_ym_n)];
		if (s_tms_n)
			o += s_tms_buf[(int)((int64_t)i * s_tms_n / s_ym_n)];
		if (C.type == 2)
			o += s_oki_buf[i];
		if (C.out == JSA_OUT_MONO) {
			// JSA II: the YM's outputs to output 0; JSA I: both outputs to the speaker
			int32_t m = C.type == 2 ? s_ym_l[i] + s_ym_r[i] + o : s_ym_l[i] + s_ym_r[i] + 2 * o;
			rbx_audio_put(m * gain / 256);
		} else {
			const int32_t l = (s_ym_l[i] + o) * gain / 256, r = (s_ym_r[i] + o) * gain / 256;
			if (C.out == JSA_OUT_SWAPPED)
				rbx_audio_put_stereo(r, l);
			else
				rbx_audio_put_stereo(l, r);
		}
	}
	memset(s_ym_l, 0, sizeof(int32_t) * (size_t)s_ym_n);
	memset(s_ym_r, 0, sizeof(int32_t) * (size_t)s_ym_n);
	s_ym_n = 0;
}

// ---- init / reset ----
void jsa_reset(void)
{
	pk_wait();
	s_pk_wpos = 0;
	il_clear();
	s_pk_rsafe = 0;
	memset(s_ram, 0, sizeof(s_ram));
	m6502_init(&s_cpu, M6502_NMOS);
	for (int p = 0; p < 0x20; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = s_ram + p * 256;
	for (int p = 0x40; p < 0x100; p++)
		s_cpu.rpage[p] = C.rom + p * 256;
	s_cpu.read = jsa_r;
	s_cpu.write = jsa_w;
	s_cpu.cycles = 0;
	set_bank(0);   // (Guardians of the Hood assumes bank 0 at startup)
	s_m2s_data = s_s2m_data = 0;
	s_m2s_ready = s_s2m_ready = 0;
	s_timed_int = s_ym_int = 0;
	s_ct1 = s_ct2 = 0;
	s_ym_vol7 = 7;
	s_pk_vol3 = s_tms_vol3 = 3;
	s_oki_vol2 = 2;
	s_ym_reset = 0;
	s_ym_addr = 0;
	s_irq_left = IRQ_PERIOD;
	s_boost = 0;
	s_ym_debt = 0;
	rbx_ym_reset(0);
	update_all_volumes();
	if (C.pokey)
		pokey_reset(&s_pokey);
	if (C.tms) {
		s_tms_clock = JSA_CPU_CLOCK * 4 / 11;
		tms5220_set_clock(&s_tms, s_tms_clock);
		tms5220_reset(&s_tms);
	}
	if (C.type == 2) {
		oki_reset(&s_oki);
		oki_set_pin7(1);
	}
	s_oki_acc = 0;
	s_oki_hold = 0;
	s_tms_acc = 0;
	s_tms_frac = 0;
	s_cyc0 = s_pk_done = s_tms_last = 0;
	s_pk_n = s_tms_n = s_ym_n = 0;
	m6502_reset(&s_cpu);
}

int jsa_init(const jsa_config *c)
{
	C = *c;
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 154;   // 0.60
	yc.irq = ym_irq;
	if (rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 2))
		return -1;
	s_ym_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_ym_l = (int32_t *)calloc((size_t)s_ym_cap, sizeof(int32_t));
	s_ym_r = (int32_t *)calloc((size_t)s_ym_cap, sizeof(int32_t));
	s_oki_buf = (int32_t *)calloc((size_t)s_ym_cap, sizeof(int32_t));
	if (!s_ym_l || !s_ym_r || !s_oki_buf)
		return -1;
	if (C.pokey) {
		pokey_init(&s_pokey, JSA_CPU_CLOCK);
		s_pk_cap = (int)(pokey_rate(&s_pokey) / 50) + 64;
		s_pk_buf = (int16_t *)calloc((size_t)s_pk_cap, sizeof(int16_t));
		if (!s_pk_buf)
			return -1;
	}
	if (C.tms) {
		tms5220_init(&s_tms, TMS5220_VARIANT_5220C, JSA_CPU_CLOCK * 4 / 11);
		s_tms_cap = 20000 / 50 + 64;
		s_tms_buf = (int16_t *)calloc((size_t)s_tms_cap, sizeof(int16_t));
		if (!s_tms_buf)
			return -1;
	}
	if (C.type == 2)
		oki_init(&s_oki, JSA_CPU_CLOCK * 2 / 3, 1, C.oki, C.oki_size, 256);
	s_test = 1;
	il_clear();
	s_pk_async = C.pokey && !getenv("RBX_NOPKASYNC");
	s_pk_rsafe = 0;
	s_pkq_in = s_pkq_out = 0;
	if (s_pk_async)
		rbx_aux2_set_job(pk_job);
	return C.out == JSA_OUT_MONO ? rbx_audio_setup(rbx_ym_rate(0)) : rbx_audio_setup_stereo(rbx_ym_rate(0));
}

void jsa_exit(void)
{
	if (s_pk_async) {
		rbx_aux2_set_job(NULL);   // (syncs first)
		s_pk_async = 0;
	}
	rbx_ym_exit(0);
	free(s_ym_l);
	free(s_ym_r);
	free(s_oki_buf);
	free(s_pk_buf);
	free(s_tms_buf);
	s_ym_l = s_ym_r = s_oki_buf = NULL;
	s_pk_buf = s_tms_buf = NULL;
}

// ---- save state ----
int jsa_state(void *ctx, int load)
{
	int32_t regs[20] = { s_bank, s_m2s_data, s_s2m_data, s_m2s_ready, s_s2m_ready, s_timed_int, s_ym_int, s_ct1, s_ct2,
	                     s_ym_vol7, s_pk_vol3, s_tms_vol3, s_oki_vol2, s_ym_reset, s_ym_addr, s_irq_left, s_boost,
	                     (int32_t)s_tms_clock, (int32_t)s_tms_frac, s_oki_pin7 };
	int32_t regs2[3] = { (int32_t)s_ym_debt, (int32_t)s_oki_acc, s_oki_hold };
	uint64_t acc = s_tms_acc;
	int rc = 0;
	pk_wait();
	s_pk_rsafe = 0;
	rc |= rbx_state_io(ctx, "jsa_cpu", &s_cpu, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "jsa_ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "jsa_regs", regs, sizeof(regs));
	rc |= rbx_state_io(ctx, "jsa_regs2", regs2, sizeof(regs2));
	rc |= rbx_state_io(ctx, "jsa_tmsacc", &acc, sizeof(acc));
	rc |= rbx_ym_state(0, ctx, load);
	if (C.pokey)
		rc |= pokey_state(&s_pokey, ctx, load, "jsa_pokey");
	if (C.tms)
		rc |= tms5220_state(&s_tms, ctx, load, "jsa_tms");
	if (C.type == 2)
		rc |= rbx_state_io(ctx, "jsa_oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	if (!rc && load) {
		set_bank((uint8_t)regs[0]);
		s_m2s_data = (uint8_t)regs[1];
		s_s2m_data = (uint8_t)regs[2];
		s_m2s_ready = regs[3];
		s_s2m_ready = regs[4];
		s_timed_int = regs[5];
		s_ym_int = regs[6];
		s_ct1 = regs[7];
		s_ct2 = regs[8];
		s_ym_vol7 = regs[9];
		s_pk_vol3 = regs[10];
		s_tms_vol3 = regs[11];
		s_oki_vol2 = regs[12];
		s_ym_reset = regs[13];
		s_ym_addr = (uint8_t)regs[14];
		s_irq_left = regs[15];
		s_boost = regs[16];
		s_tms_clock = (uint32_t)regs[17];
		s_tms_frac = (uint32_t)regs[18];
		s_ym_debt = (uint32_t)regs2[0];
		s_oki_acc = (uint32_t)regs2[1];
		s_oki_hold = regs2[2];
		s_tms_acc = acc;
		if (C.type == 2)
			oki_set_pin7(regs[19]);
		update_all_volumes();
	}
	return rc ? -1 : 0;
}
