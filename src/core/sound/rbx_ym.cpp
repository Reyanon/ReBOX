/*  Yamaha FM chips on ymfm (BSD-3-Clause, Copyright (c) 2021, Aaron Giles).
    Interface glue follows MAME src/devices/sound/ymfm_mame.h and ymopn.cpp / ymopm.cpp (BSD-3-Clause,
    copyright-holders: Aaron Giles): same fidelity (OPN_FIDELITY_MED) and output order.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "rbx_ym.h"

extern "C" {
#include "../rbx.h"
}

#include "ymfm/ymfm_opn.h"
#include "ymfm/ymfm_opm.h"
#include "ymfm/ymfm_opl.h"

#include <new>
#include <vector>

namespace {

class Intf : public ymfm::ymfm_interface
{
public:
	void ymfm_set_timer(uint32_t tnum, int32_t duration) override
	{
		if (tnum > 1) return;
		if (cfg.set_timer) { cfg.set_timer((int)tnum, duration); return; }   // (the board's timers)
		if (duration < 0) { timer_on[tnum] = 0; return; }
		timer_on[tnum] = 1;
		timer_left[tnum] = duration;
	}
	void ymfm_set_busy_end(uint32_t clocks) override { if (cfg.busy_end) cfg.busy_end(clocks); }
	bool ymfm_is_busy() override { return cfg.is_busy && cfg.is_busy(); }
	void ymfm_update_irq(bool asserted) override
	{
		int s = asserted ? 1 : 0;
		if (s != irq && cfg.irq) { irq = s; cfg.irq(s); }
		irq = s;
	}
	uint8_t ymfm_external_read(ymfm::access_class type, uint32_t address) override
	{
		if (type == ymfm::ACCESS_ADPCM_A)
			return cfg.adpcma && cfg.adpcma_size ? cfg.adpcma[address % cfg.adpcma_size] : 0;
		if (type == ymfm::ACCESS_ADPCM_B)
			return cfg.adpcmb && cfg.adpcmb_size ? cfg.adpcmb[address % cfg.adpcmb_size] : 0;
		if (type == ymfm::ACCESS_PCM) {   // (YMF278B: the wave ROM, 0xff past it as MAME's unmapped high)
			if (cfg.pcm_bank)   // (four 1 MB windows, the board's banks)
				address = cfg.pcm_bank[(address >> 20) & 3] + (address & 0xfffff);
			return cfg.adpcma && address < cfg.adpcma_size ? cfg.adpcma[address] : 0xff;
		}
		if (type == ymfm::ACCESS_IO && address == 0 && cfg.port_a_read)
			return cfg.port_a_read();
		if (type == ymfm::ACCESS_IO && address == 1 && cfg.port_b_read)
			return cfg.port_b_read();
		return 0;
	}
	void ymfm_external_write(ymfm::access_class type, uint32_t address, uint8_t data) override
	{
		if (type == ymfm::ACCESS_IO && address == 0 && cfg.port_a_write)
			cfg.port_a_write(data);
		else if (type == ymfm::ACCESS_IO && address == 1 && cfg.port_b_write)
			cfg.port_b_write(data);
	}
	// run timers for clocks (fire engine_timer_expired; overshoot is kept so periods do not drift)
	void tick(uint32_t clocks)
	{
		for (int t = 0; t < 2; t++) {
			if (!timer_on[t]) continue;
			timer_left[t] -= clocks;
			while (timer_on[t] && timer_left[t] <= 0) {
				int64_t over = -timer_left[t];
				timer_on[t] = 0;
				m_engine->engine_timer_expired(t);
				if (timer_on[t]) timer_left[t] -= over;
			}
		}
	}
	// tick(clocks) when it fires no timer (then returns 1); otherwise changes nothing and returns 0
	bool quiet_tick(uint32_t clocks)
	{
		if ((timer_on[0] && timer_left[0] <= (int64_t)clocks) || (timer_on[1] && timer_left[1] <= (int64_t)clocks))
			return false;
		if (timer_on[0]) timer_left[0] -= clocks;
		if (timer_on[1]) timer_left[1] -= clocks;
		return true;
	}
	void expire(uint32_t tnum) { m_engine->engine_timer_expired(tnum); }
	void clear() { timer_on[0] = timer_on[1] = 0; timer_left[0] = timer_left[1] = 0; irq = 0; }

	rbx_ym_config cfg {};
	int64_t timer_left[2] {};
	int32_t timer_on[2] {};
	int irq = 0;
};

struct Chip {
	Intf intf;
	int type = -1;
	ymfm::ym2610 *opnb = nullptr;
	ymfm::ym2610b *opnb2 = nullptr;
	ymfm::ym2203 *opn = nullptr;
	ymfm::ym2151 *opm = nullptr;
	ymfm::ym3526 *opl = nullptr;
	ymfm::ym3812 *opl2 = nullptr;
	ymfm::ymf278b *opl4 = nullptr;
	ymfm::ym3438 *opn2 = nullptr;
	ymfm::ym2413 *opll = nullptr;
	uint32_t rate = 0, period = 1, acc = 0;
	int vol = 256;
};

Chip s_chip[4];   // ([2] [3]: the synths of 0 and 1, rbx_ym_set_async)

static inline int32_t mix_sample(const Chip &c, const int32_t *d)
{
	const int *g = c.intf.cfg.gain;
	if (c.type == RBX_YM2610 || c.type == RBX_YM2610B)
		return (d[2] * g[0] + d[0] * g[1] + d[1] * g[2]) >> 8;
	if (c.type == RBX_YM2203)
		return (d[1] * g[0] + d[2] * g[1] + d[3] * g[2] + d[0] * g[3]) >> 8;
	if (c.type == RBX_YM3526 || c.type == RBX_YM3812)
		return (d[0] * g[0]) >> 8;
	if (c.type == RBX_YMF278B)   // (all six outputs to the one speaker)
		return ((d[0] + d[1] + d[2] + d[3] + d[4] + d[5]) * g[0]) >> 8;
	return (d[0] * g[0] + d[1] * g[1]) >> 8;
}
// the two sides (MAME routes): YM2610 SSG to both, FM L / R; YM2151 L / R; the others the same on both
static inline void mix_lr(const Chip &c, const int32_t *d, int32_t *l, int32_t *r)
{
	const int *g = c.intf.cfg.gain;
	if (c.type == RBX_YM2610 || c.type == RBX_YM2610B) {
		*l = (d[2] * g[0] + d[0] * g[1]) >> 8;
		*r = (d[2] * g[0] + d[1] * g[2]) >> 8;
	} else if (c.type == RBX_YM2151) {
		*l = (d[0] * g[0]) >> 8;
		*r = (d[1] * g[1]) >> 8;
	} else
		*l = *r = mix_sample(c, d);
}

// Samples are generated in batches up to the next timer expiry: the timers only act when they expire (before the
// sample of that tick), so a batch gives the same output and interrupt timing as one sample at a time
template <typename T>
int gen(Chip &c, T *chip, int32_t *out, int max, uint32_t clocks, int32_t *outr = nullptr, int32_t *raw4 = nullptr)
{
	enum { BATCH = 64 };
	typename T::output_data o[BATCH];
	int n = 0;
	c.acc += clocks;
	while (c.acc >= c.period) {
		// first tick as is (it may fire a timer), then more samples while their ticks fire none (no divisions:
		// a 64-bit one is a libgcc call on the Vita)
		c.acc -= c.period;
		c.intf.tick(c.period);
		uint32_t k = 1;
		while (k < BATCH && c.acc >= c.period && c.intf.quiet_tick(c.period)) {
			c.acc -= c.period;
			k++;
		}
		chip->generate(o, k);
		if (raw4)   // (each output as the chip makes it, four a sample: rbx_ym_advance_outputs)
			for (uint32_t i = 0; i < k; i++) {
				if (n < max) {
					for (int j = 0; j < 4; j++)
						raw4[4 * n + j] = j < (int)T::OUTPUTS ? o[i].data[j] : 0;
					n++;
				}
			}
		else if (outr)
			for (uint32_t i = 0; i < k; i++) {
				if (n < max) {
					int32_t l, r;
					mix_lr(c, o[i].data, &l, &r);
					out[n] += (l * c.vol) >> 8;
					outr[n++] += (r * c.vol) >> 8;
				}
			}
		else
			for (uint32_t i = 0; i < k; i++)
				if (n < max)
					out[n++] += (mix_sample(c, o[i].data) * c.vol) >> 8;
	}
	return n;
}

// ---- generation on a worker (rbx_ym_set_async) ----
// The chip the board talks to keeps its timers, status, IRQ and ports and makes no samples (generating them is most
// of its cost); a second chip (the synth) is given the same writes at the same samples by a queue and makes them on
// the rbx_aux worker; rbx_ym_async_take adds them to the board's mix. Its sound is the same as one chip's (the
// synth's timers run as the first one's; nothing the board reads comes from the synth)
static void chip_reset(Chip &c)
{
	c.intf.clear();
	if (c.opnb) c.opnb->reset();
	if (c.opnb2) c.opnb2->reset();
	if (c.opn) c.opn->reset();
	if (c.opm) c.opm->reset();
	if (c.opl) c.opl->reset();
	if (c.opl2) c.opl2->reset();
	if (c.opl4) c.opl4->reset();
	if (c.opn2) c.opn2->reset();
	if (c.opll) c.opll->reset();
	c.acc = 0;
}

enum { AQ_ADV, AQ_WRITE, AQ_GAIN, AQ_VOL, AQ_RESET, AQ_EXPIRE, AQ_USER };
struct AsyncOp { uint8_t kind, a; uint16_t b; uint32_t n; };
enum { AQ_SIZE = 8192, AO_SIZE = 16384 };
struct Async {
	bool on = false;
	bool lr = false;                   // (the two sides kept: out holds left, out_r right)
	AsyncOp q[AQ_SIZE];
	unsigned q_in = 0, q_done = 0;   // (emulation thread / worker)
	int32_t out[AO_SIZE], out_r[AO_SIZE];
	unsigned out_in = 0, out_take = 0;
	void (*user)(uint8_t, uint16_t, uint32_t) = nullptr;   // (rbx_ym_async_set_user)
	int busy = 0;   // (the chip's steps being made: by the worker, or by the emulation thread, rbx_ym_async_help)
};
Async *s_async[2];
int s_help;   // rbx_ym_async_help

// (the synth of a chip whose timers are the board's: its own timer settings go nowhere)
void synth_no_timer(int tnum, int32_t duration) { (void)tnum; (void)duration; }

void synth_write(Chip &c, uint32_t offset, uint8_t data);
void synth_reset(Chip &c);
int synth_gen(Chip &c, uint32_t clocks, int32_t *out, int max, int32_t *outr = nullptr);

// a chip's queued steps (synth of chip k: s_chip[k + 2]), by whoever holds its busy flag: in their order, one thread at
// a time (0: someone else is on it)
int chip_run(int k)
{
	Async *a = s_async[k];
	int free = 0;
	if (!__atomic_compare_exchange_n(&a->busy, &free, 1, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
		return 0;
	{
		const unsigned end = __atomic_load_n(&a->q_in, __ATOMIC_ACQUIRE);
		unsigned i = a->q_done;
		Chip &c = s_chip[k + 2];
		for (; i != end; i++) {
			const AsyncOp &op = a->q[i & (AQ_SIZE - 1)];
			switch (op.kind) {
			case AQ_ADV: {
				int32_t tmp[512], tmpr[512];
				const uint32_t n = op.n;   // (samples, at most 512 a step)
				for (uint32_t j = 0; j < n; j++)
					tmp[j] = tmpr[j] = 0;
				const int got = synth_gen(c, n * c.period, tmp, (int)n, a->lr ? tmpr : nullptr);
				for (int j = 0; j < got; j++)
					a->out[(a->out_in + (unsigned)j) & (AO_SIZE - 1)] = tmp[j];
				if (a->lr)
					for (int j = 0; j < got; j++)
						a->out_r[(a->out_in + (unsigned)j) & (AO_SIZE - 1)] = tmpr[j];
				__atomic_store_n(&a->out_in, a->out_in + (unsigned)got, __ATOMIC_RELEASE);
				break;
			}
			case AQ_WRITE: synth_write(c, op.a, (uint8_t)op.b); break;
			case AQ_GAIN: c.intf.cfg.gain[op.a & 3] = op.b; break;
			case AQ_VOL: c.vol = op.b; break;
			case AQ_RESET: chip_reset(c); break;   // (the worker's synth copy reset in its turn too)
			case AQ_EXPIRE: c.intf.expire(op.a); break;   // (the board's timer, between the same samples: CSM key-ons)
			case AQ_USER: if (a->user) a->user(op.a, op.b, op.n); break;   // (the board's own step, in its turn)
			}
		}
		__atomic_store_n(&a->q_done, i, __ATOMIC_RELEASE);
	}
	__atomic_store_n(&a->busy, 0, __ATOMIC_RELEASE);
	return 1;
}

// the worker's job: every queued step of every chip on a worker (a chip the emulation thread is making, skipped: it
// makes all of that chip's steps, none being queued meanwhile)
void async_job()
{
	for (int k = 0; k < 2; k++)
		if (s_async[k] && s_async[k]->on)
			chip_run(k);
}

// the samples of chip k up to n since the last take (the worker's steps waited for). With rbx_ym_async_help, the
// emulation thread makes the other chip's queued steps while the worker is on this one, and this one's when the
// worker is not on it
void take_wait(int k, int n)
{
	Async *a = s_async[k];
	if ((int)(__atomic_load_n(&a->out_in, __ATOMIC_ACQUIRE) - a->out_take) >= n)
		return;
	if (s_help) {
		Async *o = s_async[k ^ 1];
		if (o && o->on && __atomic_load_n(&o->q_done, __ATOMIC_ACQUIRE) != o->q_in)
			chip_run(k ^ 1);
		if (chip_run(k))
			return;
	}
	rbx_aux_sync();
}

void async_push(int chip, uint8_t kind, uint8_t a, uint16_t b, uint32_t n)
{
	Async *as = s_async[chip];
	if (as->q_in - __atomic_load_n(&as->q_done, __ATOMIC_ACQUIRE) >= AQ_SIZE)
		rbx_aux_sync();   // (full: let the worker catch up)
	AsyncOp &op = as->q[as->q_in & (AQ_SIZE - 1)];
	op.kind = kind;
	op.a = a;
	op.b = b;
	op.n = n;
	__atomic_store_n(&as->q_in, as->q_in + 1, __ATOMIC_RELEASE);
}

} // namespace

extern "C" {

int rbx_ym_init(int chip, const rbx_ym_config *cfg)
{
	Chip &c = s_chip[chip];
	rbx_ym_exit(chip);
	c.intf.cfg = *cfg;
	c.intf.clear();
	c.type = cfg->type;
	c.vol = 256;
	c.acc = 0;
	switch (cfg->type) {
	case RBX_YM2610:
		c.opnb = new (std::nothrow) ymfm::ym2610(c.intf);
		if (!c.opnb) return -1;
		c.opnb->set_fidelity(ymfm::OPN_FIDELITY_MED);
		c.rate = c.opnb->sample_rate(cfg->clock);
		break;
	case RBX_YM2610B:
		c.opnb2 = new (std::nothrow) ymfm::ym2610b(c.intf);
		if (!c.opnb2) return -1;
		c.opnb2->set_fidelity(ymfm::OPN_FIDELITY_MED);
		c.rate = c.opnb2->sample_rate(cfg->clock);
		break;
	case RBX_YM2203:
		c.opn = new (std::nothrow) ymfm::ym2203(c.intf);
		if (!c.opn) return -1;
		c.opn->set_fidelity(ymfm::OPN_FIDELITY_MED);
		c.rate = c.opn->sample_rate(cfg->clock);
		break;
	case RBX_YM2151:
		c.opm = new (std::nothrow) ymfm::ym2151(c.intf);
		if (!c.opm) return -1;
		c.rate = c.opm->sample_rate(cfg->clock);
		break;
	case RBX_YM3526:
		c.opl = new (std::nothrow) ymfm::ym3526(c.intf);
		if (!c.opl) return -1;
		c.rate = c.opl->sample_rate(cfg->clock);
		break;
	case RBX_YM3812:
		c.opl2 = new (std::nothrow) ymfm::ym3812(c.intf);
		if (!c.opl2) return -1;
		c.rate = c.opl2->sample_rate(cfg->clock);
		break;
	case RBX_YMF278B:
		c.opl4 = new (std::nothrow) ymfm::ymf278b(c.intf);
		if (!c.opl4) return -1;
		c.rate = c.opl4->sample_rate(cfg->clock);
		break;
	case RBX_YM3438:
		c.opn2 = new (std::nothrow) ymfm::ym3438(c.intf);
		if (!c.opn2) return -1;
		c.rate = c.opn2->sample_rate(cfg->clock);
		break;
	case RBX_YM2413:
		c.opll = new (std::nothrow) ymfm::ym2413(c.intf);
		if (!c.opll) return -1;
		c.rate = c.opll->sample_rate(cfg->clock);
		break;
	default:
		return -1;
	}
	c.period = cfg->clock / c.rate;
	rbx_ym_reset(chip);
	return 0;
}

void rbx_ym_exit(int chip)
{
	if (chip < 2 && s_async[chip]) {
		rbx_ym_set_async(chip, 0);
		delete s_async[chip];
		s_async[chip] = nullptr;
	}
	Chip &c = s_chip[chip];
	delete c.opnb; c.opnb = nullptr;
	delete c.opnb2; c.opnb2 = nullptr;
	delete c.opn; c.opn = nullptr;
	delete c.opm; c.opm = nullptr;
	delete c.opl; c.opl = nullptr;
	delete c.opl2; c.opl2 = nullptr;
	delete c.opl4; c.opl4 = nullptr;
	delete c.opn2; c.opn2 = nullptr;
	delete c.opll; c.opll = nullptr;
	c.type = -1;
}


void rbx_ym_reset(int chip)
{
	if (chip < 2 && s_async[chip] && s_async[chip]->on) {
		async_push(chip, AQ_RESET, 0, 0, 0);
		rbx_aux_kick();
	}
	chip_reset(s_chip[chip]);
}

uint32_t rbx_ym_rate(int chip) { return s_chip[chip].rate; }

void rbx_ym_timer_expired(int chip, int tnum)
{
	// (the synth too, after the samples made so far: what a timer does to the sound, timer A's CSM key-on)
	if (chip < 2 && s_async[chip] && s_async[chip]->on)
		async_push(chip, AQ_EXPIRE, (uint8_t)tnum, 0, 0);
	s_chip[chip].intf.expire((uint32_t)tnum);
}

uint8_t rbx_ym_read(int chip, uint32_t offset)
{
	Chip &c = s_chip[chip];
	if (c.opnb) return c.opnb->read(offset & 3);
	if (c.opnb2) return c.opnb2->read(offset & 3);
	if (c.opn) return c.opn->read(offset & 1);
	if (c.opm) return c.opm->read(offset & 1);
	if (c.opl) return c.opl->read(offset & 1);
	if (c.opl2) return c.opl2->read(offset & 1);
	if (c.opl4) return c.opl4->read(offset & 7);
	if (c.opn2) return c.opn2->read(offset & 3);
	return 0xff;
}

void rbx_ym_write(int chip, uint32_t offset, uint8_t data)
{
	if (chip < 2 && s_async[chip] && s_async[chip]->on)
		async_push(chip, AQ_WRITE, (uint8_t)offset, data, 0);
	Chip &c = s_chip[chip];
	if (c.opnb) c.opnb->write(offset & 3, data);
	else if (c.opnb2) c.opnb2->write(offset & 3, data);
	else if (c.opn) c.opn->write(offset & 1, data);
	else if (c.opm) c.opm->write(offset & 1, data);
	else if (c.opl) c.opl->write(offset & 1, data);
	else if (c.opl2) c.opl2->write(offset & 1, data);
	else if (c.opl4) c.opl4->write(offset & 7, data);
	else if (c.opn2) c.opn2->write(offset & 3, data);
	else if (c.opll) c.opll->write(offset & 1, data);
}

int rbx_ym_advance(int chip, uint32_t clocks, int32_t *out, int max)
{
	Chip &c = s_chip[chip];
	if (chip < 2 && s_async[chip] && s_async[chip]->on) {   // (the timers here, the samples on the worker)
		uint32_t n = 0;
		c.acc += clocks;
		{	// (no timer fires in the whole samples: their ticks and clock counts at once, the same as one by one)
			const uint32_t m = c.acc / c.period;
			if (m > 1 && c.intf.quiet_tick(m * c.period)) {
				c.acc -= m * c.period;
				if (c.opnb) c.opnb->clock_status(m);
				else if (c.opn) c.opn->clock_status(m);
				else if (c.opm) c.opm->clock_status(m);
				else if (c.opl) c.opl->clock_status(m);
				else if (c.opl2) c.opl2->clock_status(m);
				else if (c.opl4) c.opl4->clock_status(m);
				else if (c.opn2) c.opn2->clock_status(m);
				n = m;
			}
		}
		while (c.acc >= c.period) {
			c.acc -= c.period;
			c.intf.tick(c.period);
			// (what generate() moves that a read sees: the ADPCM end flags (YM2610), the clock count of timer B's load)
			if (c.opnb) c.opnb->clock_status(1);
			else if (c.opn) c.opn->clock_status(1);
			else if (c.opm) c.opm->clock_status(1);
			else if (c.opl) c.opl->clock_status(1);
			else if (c.opl2) c.opl2->clock_status(1);
			else if (c.opl4) c.opl4->clock_status(1);
			else if (c.opn2) c.opn2->clock_status(1);
			n++;
		}
		for (uint32_t k = 0; k < n; k += 512)
			async_push(chip, AQ_ADV, 0, 0, n - k < 512 ? n - k : 512);
		if (n)
			rbx_aux_kick();
		(void)out;
		return (int)n < max ? (int)n : max;
	}
	if (c.opnb) return gen(c, c.opnb, out, max, clocks);
	if (c.opnb2) return gen(c, c.opnb2, out, max, clocks);
	if (c.opn) return gen(c, c.opn, out, max, clocks);
	if (c.opm) return gen(c, c.opm, out, max, clocks);
	if (c.opl) return gen(c, c.opl, out, max, clocks);
	if (c.opl2) return gen(c, c.opl2, out, max, clocks);
	if (c.opl4) return gen(c, c.opl4, out, max, clocks);
	if (c.opn2) return gen(c, c.opn2, out, max, clocks);
	if (c.opll) return gen(c, c.opll, out, max, clocks);
	return 0;
}

int rbx_ym_advance_lr(int chip, uint32_t clocks, int32_t *l, int32_t *r, int max)
{
	Chip &c = s_chip[chip];
	if (chip < 2 && s_async[chip] && s_async[chip]->on)   // (the samples on the worker: rbx_ym_async_take_lr)
		return rbx_ym_advance(chip, clocks, l, max);
	if (c.opnb) return gen(c, c.opnb, l, max, clocks, r);
	if (c.opnb2) return gen(c, c.opnb2, l, max, clocks, r);
	if (c.opn) return gen(c, c.opn, l, max, clocks, r);
	if (c.opm) return gen(c, c.opm, l, max, clocks, r);
	if (c.opl) return gen(c, c.opl, l, max, clocks, r);
	if (c.opl2) return gen(c, c.opl2, l, max, clocks, r);
	if (c.opl4) return gen(c, c.opl4, l, max, clocks, r);
	if (c.opn2) return gen(c, c.opn2, l, max, clocks, r);
	if (c.opll) return gen(c, c.opll, l, max, clocks, r);
	return 0;
}

int rbx_ym_advance_outputs(int chip, uint32_t clocks, int32_t *raw4, int max)
{
	Chip &c = s_chip[chip];
	if (c.opn) return gen(c, c.opn, nullptr, max, clocks, nullptr, raw4);
	return 0;
}

void rbx_ym_set_volume(int chip, int vol256)
{
	s_chip[chip].vol = vol256;
	if (chip < 2 && s_async[chip] && s_async[chip]->on)
		async_push(chip, AQ_VOL, 0, (uint16_t)vol256, 0);
}
void rbx_ym_set_gain(int chip, int route, int gain256)
{
	s_chip[chip].intf.cfg.gain[route & 3] = gain256;
	if (chip < 2 && s_async[chip] && s_async[chip]->on)
		async_push(chip, AQ_GAIN, (uint8_t)route, (uint16_t)gain256, 0);
}

static void copy_chip_state(Chip &from, Chip &to)
{
	std::vector<uint8_t> buf;
	ymfm::ymfm_saved_state st(buf, true);
	if (from.opnb) from.opnb->save_restore(st);
	if (from.opnb2) from.opnb2->save_restore(st);
	if (from.opn) from.opn->save_restore(st);
	if (from.opm) from.opm->save_restore(st);
	if (from.opl) from.opl->save_restore(st);
	if (from.opl2) from.opl2->save_restore(st);
	if (from.opl4) from.opl4->save_restore(st);
	if (from.opn2) from.opn2->save_restore(st);
	if (from.opll) from.opll->save_restore(st);
	ymfm::ymfm_saved_state ld(buf, false);
	if (to.opnb) to.opnb->save_restore(ld);
	if (to.opnb2) to.opnb2->save_restore(ld);
	if (to.opn) to.opn->save_restore(ld);
	if (to.opm) to.opm->save_restore(ld);
	if (to.opl) to.opl->save_restore(ld);
	if (to.opl2) to.opl2->save_restore(ld);
	if (to.opl4) to.opl4->save_restore(ld);
	if (to.opn2) to.opn2->save_restore(ld);
	if (to.opll) to.opll->save_restore(ld);
	to.acc = from.acc;
	to.intf.timer_on[0] = from.intf.timer_on[0];
	to.intf.timer_on[1] = from.intf.timer_on[1];
	to.intf.timer_left[0] = from.intf.timer_left[0];
	to.intf.timer_left[1] = from.intf.timer_left[1];
}

int rbx_ym_set_async(int chip, int on)
{
	s_help = 0;   // (the board sets it again after its chips: rbx_ym_async_help)
	if (chip >= 2 || s_chip[chip].type < 0)
		return -1;
	// (the YM2610s' ADPCM end flags, read by the sound CPU, move with the samples: their first chip runs the clocks
	// of the samples without the output, ym2610::clock_status. Not the YM2610B: none needs it)
	if (on && s_chip[chip].type == RBX_YM2610B)
		return 0;
	if (!on) {
		if (s_async[chip] && s_async[chip]->on) {
			rbx_aux_sync();
			s_async[chip]->on = false;
			s_async[chip]->user = nullptr;
			rbx_ym_exit(chip + 2);
			if (!(s_async[chip ^ 1] && s_async[chip ^ 1]->on))
				rbx_aux_set_job(nullptr);   // (no chip left on it)
		}
		return 0;
	}
	if (!s_async[chip]) {
		s_async[chip] = new (std::nothrow) Async;
		if (!s_async[chip])
			return -1;
	}
	Async *a = s_async[chip];
	// the synth: the same chip without the board's callbacks (IRQ, ports), from the first one's state
	rbx_ym_config cfg = s_chip[chip].intf.cfg;
	cfg.irq = nullptr;
	cfg.port_a_write = nullptr;
	cfg.port_b_write = nullptr;
	cfg.port_a_read = nullptr;
	cfg.port_b_read = nullptr;
	// (the board's timers and busy flag are the first chip's alone: the synth's timer settings go nowhere, the board's
	// expiries reach it through the queue, rbx_ym_timer_expired)
	if (cfg.set_timer)
		cfg.set_timer = synth_no_timer;
	cfg.busy_end = nullptr;
	cfg.is_busy = nullptr;
	if (rbx_ym_init(chip + 2, &cfg))
		return -1;
	copy_chip_state(s_chip[chip], s_chip[chip + 2]);
	s_chip[chip + 2].vol = s_chip[chip].vol;
	a->q_in = a->q_done = 0;
	a->out_in = a->out_take = 0;
	a->lr = on == 2;
	a->user = nullptr;
	a->on = true;
	rbx_aux_set_job(async_job);
	return 0;
}

int rbx_ym_async_set_user(int chip, void (*fn)(uint8_t a, uint16_t b, uint32_t n))
{
	Async *a = chip < 2 ? s_async[chip] : nullptr;
	if (!a || !a->on)
		return -1;
	rbx_aux_sync();
	a->user = fn;
	return 0;
}

void rbx_ym_async_help(int on) { s_help = on; }

void rbx_ym_async_user(int chip, uint8_t a, uint16_t b, uint32_t n)
{
	async_push(chip, AQ_USER, a, b, n);
}

void rbx_ym_async_take_lr(int chip, int32_t *l, int32_t *r, int n)
{
	Async *a = chip < 2 ? s_async[chip] : nullptr;
	if (!a || !a->on)
		return;
	take_wait(chip, n);
	const unsigned have = __atomic_load_n(&a->out_in, __ATOMIC_ACQUIRE) - a->out_take;
	if ((unsigned)n > have)
		n = (int)have;
	for (int i = 0; i < n; i++) {
		l[i] += a->out[(a->out_take + (unsigned)i) & (AO_SIZE - 1)];
		r[i] += a->out_r[(a->out_take + (unsigned)i) & (AO_SIZE - 1)];
	}
	a->out_take += (unsigned)n;
}

void rbx_ym_async_take(int chip, int32_t *dst, int n)
{
	Async *a = chip < 2 ? s_async[chip] : nullptr;
	if (!a || !a->on)
		return;
	take_wait(chip, n);
	const unsigned have = __atomic_load_n(&a->out_in, __ATOMIC_ACQUIRE) - a->out_take;
	if ((unsigned)n > have)
		n = (int)have;
	for (int i = 0; i < n; i++)
		dst[i] += a->out[(a->out_take + (unsigned)i) & (AO_SIZE - 1)];
	a->out_take += (unsigned)n;
}

static int ym_state_on(int chip, void *ctx, int load);
int rbx_ym_state(int chip, void *ctx, int load)
{
	if (chip < 2 && s_async[chip] && s_async[chip]->on) {   // (the synth's state is the whole chip's: the first has made no samples)
		rbx_aux_sync();
		// (the clocks between samples are the first chip's: the synth is given whole samples. Its own value is
		// never used again: it takes each step's samples whole from any remainder below a sample)
		s_chip[chip + 2].acc = s_chip[chip].acc;
		// (each its own: the first's timers, status flags and IRQ are not the synth's, whose timers never expire)
		int rc = ym_state_on(chip + 2, ctx, load);
		if (!rc)
			rc = ym_state_on(chip, ctx, load);
		if (!rc && load)
			s_async[chip]->out_in = s_async[chip]->out_take = 0;
		return rc;
	}
	return ym_state_on(chip, ctx, load);
}

static int ym_state_on(int chip, void *ctx, int load)
{
	Chip &c = s_chip[chip];
	std::vector<uint8_t> buf;
	uint32_t size = 0;
	if (!load) {
		ymfm::ymfm_saved_state st(buf, true);
		if (c.opnb) c.opnb->save_restore(st);
		if (c.opnb2) c.opnb2->save_restore(st);
		if (c.opn) c.opn->save_restore(st);
		if (c.opm) c.opm->save_restore(st);
		if (c.opl) c.opl->save_restore(st);
		if (c.opl2) c.opl2->save_restore(st);
		if (c.opl4) c.opl4->save_restore(st);
		if (c.opn2) c.opn2->save_restore(st);
		if (c.opll) c.opll->save_restore(st);
		size = (uint32_t)buf.size();
	}
	if (rbx_state_io(ctx, "ymsize", &size, 4)) return -1;
	if (load) buf.resize(size);
	if (size && rbx_state_io(ctx, "ym", buf.data(), size)) return -1;
	int32_t misc[8] = { (int32_t)c.acc, c.intf.timer_on[0], c.intf.timer_on[1], (int32_t)c.intf.timer_left[0],
	                    (int32_t)c.intf.timer_left[1], c.intf.irq, 0, 0 };
	if (rbx_state_io(ctx, "ymmisc", misc, sizeof(misc))) return -1;
	if (load) {
		ymfm::ymfm_saved_state st(buf, false);
		if (c.opnb) c.opnb->save_restore(st);
		if (c.opnb2) c.opnb2->save_restore(st);
		if (c.opn) c.opn->save_restore(st);
		if (c.opm) c.opm->save_restore(st);
		if (c.opl) c.opl->save_restore(st);
		if (c.opl2) c.opl2->save_restore(st);
		if (c.opl4) c.opl4->save_restore(st);
		if (c.opn2) c.opn2->save_restore(st);
		if (c.opll) c.opll->save_restore(st);
		c.acc = (uint32_t)misc[0];
		c.intf.timer_on[0] = misc[1]; c.intf.timer_on[1] = misc[2];
		c.intf.timer_left[0] = misc[3]; c.intf.timer_left[1] = misc[4];
		c.intf.irq = misc[5];
	}
	return 0;
}

} // extern "C"

namespace {
void synth_write(Chip &c, uint32_t offset, uint8_t data)
{
	if (c.opnb) c.opnb->write(offset & 3, data);
	else if (c.opnb2) c.opnb2->write(offset & 3, data);
	else if (c.opn) c.opn->write(offset & 1, data);
	else if (c.opm) c.opm->write(offset & 1, data);
	else if (c.opl) c.opl->write(offset & 1, data);
	else if (c.opl2) c.opl2->write(offset & 1, data);
	else if (c.opl4) c.opl4->write(offset & 7, data);
	else if (c.opn2) c.opn2->write(offset & 3, data);
	else if (c.opll) c.opll->write(offset & 1, data);
}
void synth_reset(Chip &c)
{
	c.intf.clear();
	if (c.opnb) c.opnb->reset();
	if (c.opnb2) c.opnb2->reset();
	if (c.opn) c.opn->reset();
	if (c.opm) c.opm->reset();
	if (c.opl) c.opl->reset();
	if (c.opl2) c.opl2->reset();
	if (c.opl4) c.opl4->reset();
	if (c.opn2) c.opn2->reset();
	if (c.opll) c.opll->reset();
	c.acc = 0;
}
int synth_gen(Chip &c, uint32_t clocks, int32_t *out, int max, int32_t *outr)
{
	if (c.opnb) return gen(c, c.opnb, out, max, clocks, outr);
	if (c.opnb2) return gen(c, c.opnb2, out, max, clocks, outr);
	if (c.opn) return gen(c, c.opn, out, max, clocks, outr);
	if (c.opm) return gen(c, c.opm, out, max, clocks, outr);
	if (c.opl) return gen(c, c.opl, out, max, clocks, outr);
	if (c.opl2) return gen(c, c.opl2, out, max, clocks, outr);
	if (c.opl4) return gen(c, c.opl4, out, max, clocks, outr);
	if (c.opn2) return gen(c, c.opn2, out, max, clocks, outr);
	if (c.opll) return gen(c, c.opll, out, max, clocks, outr);
	return 0;
}
} // namespace

