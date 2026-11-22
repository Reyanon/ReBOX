/*  Atari C012294 POKEY, ported to C from MAME src/devices/sound/pokey.cpp (BSD-3-Clause,
    copyright-holders: Brad Oliver, Eric Smith, Juergen Buchmueller).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "pokey.h"
#include "../rbx.h"

#include <stddef.h>
#include <string.h>

#define POKEY_DEFAULT_GAIN (32767 / 11 / 4)

#define CHAN1 0
#define CHAN2 1
#define CHAN3 2
#define CHAN4 3

// AUDCx
#define NOTPOLY5    0x80
#define POLY4       0x40
#define PURE        0x20
#define VOLUME_ONLY 0x10
#define VOLUME_MASK 0x0f

// AUDCTL
#define POLY9       0x80
#define CH1_HICLK   0x40
#define CH3_HICLK   0x20
#define CH12_JOINED 0x10
#define CH34_JOINED 0x08
#define CH1_FILTER  0x04
#define CH2_FILTER  0x02
#define CLK_15KHZ   0x01

// IRQEN / IRQST
#define IRQ_BREAK   0x80
#define IRQ_KEYBD   0x40
#define IRQ_SERIN   0x20
#define IRQ_SEROR   0x10
#define IRQ_SEROC   0x08
#define IRQ_TIMR4   0x04
#define IRQ_TIMR2   0x02
#define IRQ_TIMR1   0x01

// SKSTAT
#define SK_FRAME    0x80
#define SK_KBERR    0x40
#define SK_OVERRUN  0x20
#define SK_SERIN    0x10
#define SK_SHIFT    0x08
#define SK_KEYBD    0x04
#define SK_BUSY     0x02

// SKCTL
#define SK_BREAK    0x80
#define SK_OCLK     0x60
#define SK_ICLK     0x30
#define SK_ASYNC    0x10
#define SK_TWOTONE  0x08
#define SK_PADDLE   0x04
#define SK_RESET    0x03
#define SK_KEYSCAN  0x02
#define SK_DEBOUNCE 0x01

#define DIV_64  28
#define DIV_15  114

#define CLK_1   0
#define CLK_28  1
#define CLK_114 2

// registers
enum { AUDF1_C, AUDC1_C, AUDF2_C, AUDC2_C, AUDF3_C, AUDC3_C, AUDF4_C, AUDC4_C,
       AUDCTL_C, STIMER_C, SKREST_C, POTGO_C, SEROUT_C = 0x0d, IRQEN_C, SKCTL_C };
enum { POT0_C, POT7_C = 7, ALLPOT_C, KBCODE_C, RANDOM_C, SERIN_C = 0x0d, IRQST_C, SKSTAT_C };

#define P17_LEN 0x1ffff
#define NEVER   0xffffffffu

static const uint8_t s_intmask[4] = { IRQ_TIMR1, IRQ_TIMR2, 0, IRQ_TIMR4 };

// ---- the polynomial tables (shared, built once; the same contents every time) ----
static uint8_t s_poly4[0x0f];
static uint8_t s_poly5[0x1f];
static uint16_t s_poly9[0x1ff];
// The 17-bit polynomial as one bit stream: B[i] = bit 7 of MAME's m_poly17[i]. The register is a shift
// register (bits 16..8 and 6..0 shift right, bit 7 takes the feedback), so with the period 0x1ffff:
// bit 0 of m_poly17[i] = B[i - 7] and (m_poly17[i] >> 8) & 0xff = B[i - 16 .. i - 9] (bit 0 first).
static uint8_t s_p17bits[(P17_LEN + 7) / 8];
static int s_built;

static void poly_init_4_5(uint8_t *poly, int size)
{
	const uint32_t mask = (1u << size) - 1;
	uint32_t lfsr = 0;
	const int xorbit = size - 1;
	for (uint32_t i = 0; i < mask; i++) {
		lfsr = (lfsr << 1) | (~((lfsr >> 2) ^ (lfsr >> xorbit)) & 1);
		poly[i] = (uint8_t)(lfsr & mask);
	}
}

static void build_tables(void)
{
	if (s_built)
		return;
	poly_init_4_5(s_poly4, 4);
	poly_init_4_5(s_poly5, 5);
	uint32_t lfsr = 0x1ff;
	for (uint32_t i = 0; i < 0x1ff; i++) {
		const uint32_t in = (lfsr & 1) ^ ((lfsr >> 5) & 1);
		lfsr = (lfsr >> 1) | (in << 8);
		s_poly9[i] = (uint16_t)lfsr;
	}
	memset(s_p17bits, 0, sizeof(s_p17bits));
	lfsr = P17_LEN;
	for (uint32_t i = 0; i < P17_LEN; i++) {
		const uint32_t in8 = ((lfsr >> 8) ^ (lfsr >> 13)) & 1;
		const uint32_t in = lfsr & 1;
		lfsr >>= 1;
		lfsr = (lfsr & 0xff7f) | (in8 << 7);
		lfsr |= in << 16;
		if (lfsr & 0x80)
			s_p17bits[i >> 3] |= (uint8_t)(1u << (i & 7));
	}
	s_built = 1;
}

static inline uint32_t p17_bit(uint32_t idx) { return (s_p17bits[idx >> 3] >> (idx & 7)) & 1; }

// bit 0 of m_poly17[i]
static inline uint32_t poly17_bit0(uint32_t i)
{
	uint32_t idx = i + P17_LEN - 7;
	if (idx >= P17_LEN)
		idx -= P17_LEN;
	return p17_bit(idx);
}

// (m_poly17[i] >> 8) & 0xff
static uint32_t poly17_rand(uint32_t i)
{
	uint32_t idx = i + P17_LEN - 16;
	if (idx >= P17_LEN)
		idx -= P17_LEN;
	uint32_t v = 0;
	for (int j = 0; j < 8; j++) {
		v |= p17_bit(idx) << j;
		if (++idx == P17_LEN)
			idx = 0;
	}
	return v;
}

// ---- interrupts ----
static void irq_w(pokey_t *p, int state)
{
	p->irq_line = (uint8_t)state;
	if (p->irq_cb)
		p->irq_cb(p->irq_ctx, state);
}

// (sync_set_irqst: MAME defers it to a scheduler sync at the current time; applied at once here)
static void sync_set_irqst(pokey_t *p, uint8_t param)
{
	if (p->IRQEN & param) {
		p->IRQST |= param;
		irq_w(p, 1);
	}
}

// (the timer callbacks, all adjusted to attotime::zero in MAME: run at once; SOD / OCLK are not connected)
static void serout_ready_irq(pokey_t *p)
{
	p->IRQST &= (uint8_t)~IRQ_SEROC;
	if (p->IRQEN & IRQ_SEROR) {
		p->IRQST |= IRQ_SEROR;
		irq_w(p, 1);
	}
}

static void serout_complete_irq(pokey_t *p)
{
	p->IRQST |= IRQ_SEROC;
	if (p->IRQEN & IRQ_SEROC)
		irq_w(p, 1);
}

static void serin_ready_irq(pokey_t *p)
{
	if (!(p->serin_shift & (1 << 9)))
		p->SKSTAT |= SK_FRAME;
	p->SERIN = (uint8_t)((p->serin_shift >> 1) & 0xff);
	if (p->IRQST & IRQ_SERIN)
		p->SKSTAT |= SK_OVERRUN;
	else if (p->IRQEN & IRQ_SERIN) {
		p->IRQST |= IRQ_SERIN;
		irq_w(p, 1);
	}
}

// ---- channels ----
static inline void reset_channel(pokey_t *p, int ch)
{
	p->counter[ch] = p->AUDF[ch] ^ 0xff;
	p->borrow_cnt[ch] = 0;
}

static inline void inc_chan(pokey_t *p, int ch, int cycles)
{
	p->counter[ch] = (p->counter[ch] + 1) & 0xff;
	if (p->counter[ch] == 0 && p->borrow_cnt[ch] == 0) {
		p->borrow_cnt[ch] = cycles;
		if (p->IRQEN & s_intmask[ch])
			sync_set_irqst(p, s_intmask[ch]);
	}
}

static inline int check_borrow(pokey_t *p, int ch)
{
	if (p->borrow_cnt[ch] > 0) {
		p->borrow_cnt[ch]--;
		return p->borrow_cnt[ch] == 0;
	}
	return 0;
}

static inline void process_channel(pokey_t *p, int ch)
{
	if ((p->AUDC[ch] & NOTPOLY5) || (s_poly5[p->p5] & 1)) {
		if (p->AUDC[ch] & PURE)
			p->output[ch] ^= 1;
		else if (p->AUDC[ch] & POLY4)
			p->output[ch] = s_poly4[p->p4] & 1;
		else if (p->AUDCTL & POLY9)
			p->output[ch] = s_poly9[p->p9] & 1;
		else
			p->output[ch] = (uint8_t)poly17_bit0(p->p17);
		p->old_raw_inval = 1;
	}
}

static void process_serin(pokey_t *p)
{
	if (p->SKSTAT & SK_BUSY) {
		p->serin_shift >>= 1;
		if (!(p->SKSTAT & SK_SERIN))
			p->serin_shift |= 1 << 9;
		if (p->serin_shift & 1) {
			p->SKSTAT &= (uint8_t)~SK_BUSY;
			serin_ready_irq(p);
		}
	} else if (p->SKSTAT & SK_SERIN) {
		p->serin_shift = 1 << 9;
		p->SKSTAT |= SK_BUSY;
	}
}

static void process_serout(pokey_t *p)
{
	if (p->serout_shift == 1) {
		if (p->serout_full) {
			p->serout_shift = (uint16_t)((p->SEROUT << 1) | (1 << 9));
			p->serout_full = 0;
			serout_ready_irq(p);
		} else
			serout_complete_irq(p);
	} else
		p->serout_shift >>= 1;   // (serout_transmit_bit only drives SOD / OCLK)
}

static void pokey_potgo(pokey_t *p)
{
	p->ALLPOT = 0x00;
	p->pot_counter = 0;
	for (int pot = 0; pot < 8; pot++) {
		p->POTx[pot] = 228;
		if (p->pot_set & (1 << pot)) {
			int r = p->pot_in[pot];
			if (r >= 228)
				r = 228;
			if (r == 0)
				p->ALLPOT |= (uint8_t)(1 << pot);
			p->POTx[pot] = (uint8_t)r;
		}
	}
}

// step_pot after the counter moved (the ALLPOT bits only ever get set, so k steps at once give the same)
static inline void pot_latch(pokey_t *p)
{
	uint8_t upd = 0;
	for (int pot = 0; pot < 8; pot++)
		if (p->POTx[pot] < p->pot_counter || p->pot_counter == 228)
			upd |= (uint8_t)(1 << pot);
	p->ALLPOT |= upd;   // (sync_pot)
}

static inline int async_reset_on(const pokey_t *p)
{
	return (p->SKCTL & SK_ASYNC) && !(p->SKSTAT & (SK_BUSY | SK_SERIN));
}

static void update_level(pokey_t *p)
{
	int32_t out = 0;
	for (int i = 0; i < 4; i++)
		out += (int32_t)((p->out_raw >> (4 * i)) & 0x0f);
	out *= POKEY_DEFAULT_GAIN;
	p->level = out > 0x7fff ? 0x7fff : out;
}

// ---- MAME step_one_clock ----
static void step_one_clock(pokey_t *p)
{
	int toggle_iclk = 0;
	int toggle_oclk = 0;

	if (p->SKCTL & SK_RESET) {
		if (++p->p4 == 0x0000f)
			p->p4 = 0;
		if (++p->p5 == 0x0001f)
			p->p5 = 0;
		if (++p->p9 == 0x001ff)
			p->p9 = 0;
		if (++p->p17 == 0x1ffff)
			p->p17 = 0;

		int clock_triggered[3] = { 1, 0, 0 };
		if (++p->clock_cnt[CLK_28] >= DIV_64) {
			p->clock_cnt[CLK_28] = 0;
			clock_triggered[CLK_28] = 1;
		}
		if (++p->clock_cnt[CLK_114] >= DIV_15) {
			p->clock_cnt[CLK_114] = 0;
			clock_triggered[CLK_114] = 1;
		}

		if ((p->AUDCTL & CH1_HICLK) && clock_triggered[CLK_1])
			inc_chan(p, CHAN1, (p->AUDCTL & CH12_JOINED) ? 7 : 4);

		const int base_clock = (p->AUDCTL & CLK_15KHZ) ? CLK_114 : CLK_28;

		if (!(p->AUDCTL & CH1_HICLK) && clock_triggered[base_clock])
			inc_chan(p, CHAN1, 1);

		const int async_reset = async_reset_on(p);
		if (async_reset)
			p->ser_iclk = 1;

		if ((p->AUDCTL & CH3_HICLK) && clock_triggered[CLK_1] && !async_reset)
			inc_chan(p, CHAN3, (p->AUDCTL & CH34_JOINED) ? 7 : 4);

		if (!(p->AUDCTL & CH3_HICLK) && clock_triggered[base_clock] && !async_reset)
			inc_chan(p, CHAN3, 1);

		if (clock_triggered[base_clock]) {
			if (!(p->AUDCTL & CH12_JOINED)) {
				if (p->counter[CHAN2] == 0xff && (p->SKCTL & SK_OCLK) == 0x60)
					toggle_oclk = 1;
				inc_chan(p, CHAN2, 1);
			}
			if (!(p->AUDCTL & CH34_JOINED) && !async_reset) {
				if (p->counter[CHAN4] == 0xff) {
					if ((p->SKCTL & SK_ICLK) != 0)
						toggle_iclk = 1;
					if ((p->SKCTL & SK_OCLK) && (p->SKCTL & SK_OCLK) != 0x60)
						toggle_oclk = 1;
				}
				inc_chan(p, CHAN4, 1);
			}
		}

		if ((clock_triggered[CLK_114] || (p->SKCTL & SK_PADDLE)) && p->pot_counter < 228) {
			p->pot_counter++;
			pot_latch(p);
		}

		// (step_keyboard without a keyboard callback: only the scan counter runs)
		if (clock_triggered[CLK_114] && (p->SKCTL & SK_KEYSCAN)) {
			if (++p->kbd_cnt > 63)
				p->kbd_cnt = 0;
		}
	}

	if (check_borrow(p, CHAN3)) {
		if (p->AUDCTL & CH34_JOINED) {
			if (p->counter[CHAN4] == 0xff) {
				if ((p->SKCTL & SK_ICLK) != 0)
					toggle_iclk = 1;
				if ((p->SKCTL & SK_OCLK) != 0 && (p->SKCTL & SK_OCLK) != 0x60)
					toggle_oclk = 1;
			}
			inc_chan(p, CHAN4, 1);
		} else
			reset_channel(p, CHAN3);

		process_channel(p, CHAN3);
		if (p->AUDCTL & CH1_FILTER)
			p->filter_sample[CHAN1] = p->output[CHAN1];
		else
			p->filter_sample[CHAN1] = 1;
		p->old_raw_inval = 1;
	}

	if (check_borrow(p, CHAN4)) {
		if (p->AUDCTL & CH34_JOINED)
			reset_channel(p, CHAN3);
		reset_channel(p, CHAN4);
		process_channel(p, CHAN4);
		if (p->AUDCTL & CH2_FILTER)
			p->filter_sample[CHAN2] = p->output[CHAN2];
		else
			p->filter_sample[CHAN2] = 1;
		p->old_raw_inval = 1;
	}

	if ((p->SKCTL & SK_TWOTONE) && p->borrow_cnt[CHAN2] == 1) {
		reset_channel(p, CHAN1);
		p->old_raw_inval = 1;
		p->sod_twotone = !p->sod_twotone;
	}

	if (check_borrow(p, CHAN1)) {
		if (p->AUDCTL & CH12_JOINED) {
			if (p->counter[CHAN2] == 0xff && (p->SKCTL & SK_OCLK) == 0x60)
				toggle_oclk = 1;
			inc_chan(p, CHAN2, 1);
		} else {
			reset_channel(p, CHAN1);
			if ((p->SKCTL & SK_TWOTONE) &&
			    ((p->IRQST & IRQ_SEROC) ? !(p->SKCTL & SK_BREAK) : (p->serout_shift & 1))) {
				reset_channel(p, CHAN2);
				p->old_raw_inval = 1;
				p->sod_twotone = !p->sod_twotone;
			}
		}
		process_channel(p, CHAN1);
	}

	if (check_borrow(p, CHAN2)) {
		if (p->AUDCTL & CH12_JOINED)
			reset_channel(p, CHAN1);
		reset_channel(p, CHAN2);
		process_channel(p, CHAN2);
	}

	if (p->old_raw_inval) {
		uint32_t sum = 0;
		for (int ch = 0; ch < 4; ch++)
			sum |= ((p->output[ch] ^ p->filter_sample[ch]) || (p->AUDC[ch] & VOLUME_ONLY)) ?
			       ((uint32_t)(p->AUDC[ch] & VOLUME_MASK) << (ch * 4)) : 0;
		p->old_raw_inval = 0;
		if (sum != p->out_raw) {
			p->out_raw = sum;
			update_level(p);
		}
	}

	if (toggle_iclk) {
		p->ser_iclk = !p->ser_iclk;
		if (!p->ser_iclk)
			process_serin(p);
	}

	if (toggle_oclk && (p->serout_full || !(p->IRQST & IRQ_SEROC))) {
		p->ser_oclk = !p->ser_oclk;
		if (p->ser_oclk)
			process_serout(p);
	}
}

// ---- the skipping engine ----
// Clocks until the next clock that has to run step_one_clock: a borrow firing, a counter wrapping to 0
// (arming a borrow, raising a timer IRQ or toggling a serial clock), a pending output update or the fast
// paddle scan. The clocks before it only move counters; fast_forward does that arithmetically.
static uint32_t next_event(const pokey_t *p)
{
#ifdef POKEY_REFERENCE
	(void)p;
	return 1;
#else
	if (p->old_raw_inval)
		return 1;
	uint32_t k = NEVER;
	for (int ch = 0; ch < 4; ch++)
		if (p->borrow_cnt[ch] > 0 && (uint32_t)p->borrow_cnt[ch] < k)
			k = (uint32_t)p->borrow_cnt[ch];
	if (p->SKCTL & SK_RESET) {
		if ((p->SKCTL & SK_PADDLE) && p->pot_counter < 228)
			return 1;
		const int base = (p->AUDCTL & CLK_15KHZ) ? CLK_114 : CLK_28;
		const uint32_t div = base == CLK_114 ? DIV_15 : DIV_64;
		const uint32_t first = div - (uint32_t)p->clock_cnt[base];   // clocks to the next base tick
		const int async_reset = async_reset_on(p);
		uint32_t w;
		// counter c wraps to 0 on its (256 - c)-th increment
		w = (p->AUDCTL & CH1_HICLK) ? 256u - (uint32_t)p->counter[CHAN1]
		                            : first + (255u - (uint32_t)p->counter[CHAN1]) * div;
		if (w < k)
			k = w;
		if (!async_reset) {
			w = (p->AUDCTL & CH3_HICLK) ? 256u - (uint32_t)p->counter[CHAN3]
			                            : first + (255u - (uint32_t)p->counter[CHAN3]) * div;
			if (w < k)
				k = w;
		}
		if (!(p->AUDCTL & CH12_JOINED)) {
			w = first + (255u - (uint32_t)p->counter[CHAN2]) * div;
			if (w < k)
				k = w;
		}
		if (!(p->AUDCTL & CH34_JOINED) && !async_reset) {
			w = first + (255u - (uint32_t)p->counter[CHAN4]) * div;
			if (w < k)
				k = w;
		}
	}
	return k;
#endif
}

// n clocks without an event (n < the clocks next_event gave)
static void fast_forward(pokey_t *p, uint32_t n)
{
	if (p->SKCTL & SK_RESET) {
		p->p4 = (p->p4 + n % 0x0f) % 0x0f;
		p->p5 = (p->p5 + n % 0x1f) % 0x1f;
		p->p9 = (p->p9 + n % 0x1ff) % 0x1ff;
		p->p17 = (p->p17 + n % P17_LEN) % P17_LEN;
		const uint32_t c28 = (uint32_t)p->clock_cnt[CLK_28] + n;
		const uint32_t t28 = c28 / DIV_64;
		p->clock_cnt[CLK_28] = (int32_t)(c28 % DIV_64);
		const uint32_t c114 = (uint32_t)p->clock_cnt[CLK_114] + n;
		const uint32_t t114 = c114 / DIV_15;
		p->clock_cnt[CLK_114] = (int32_t)(c114 % DIV_15);
		const uint32_t b = (p->AUDCTL & CLK_15KHZ) ? t114 : t28;

		p->counter[CHAN1] = (int32_t)(((uint32_t)p->counter[CHAN1] + ((p->AUDCTL & CH1_HICLK) ? n : b)) & 0xff);
		const int async_reset = async_reset_on(p);
		if (async_reset)
			p->ser_iclk = 1;
		else
			p->counter[CHAN3] = (int32_t)(((uint32_t)p->counter[CHAN3] + ((p->AUDCTL & CH3_HICLK) ? n : b)) & 0xff);
		if (!(p->AUDCTL & CH12_JOINED))
			p->counter[CHAN2] = (int32_t)(((uint32_t)p->counter[CHAN2] + b) & 0xff);
		if (!(p->AUDCTL & CH34_JOINED) && !async_reset)
			p->counter[CHAN4] = (int32_t)(((uint32_t)p->counter[CHAN4] + b) & 0xff);

		if (t114) {
			if (p->pot_counter < 228) {
				const uint32_t room = 228u - p->pot_counter;
				p->pot_counter = (uint8_t)(p->pot_counter + (t114 < room ? t114 : room));
				pot_latch(p);
			}
			if (p->SKCTL & SK_KEYSCAN)
				p->kbd_cnt = (uint8_t)((p->kbd_cnt + t114) & 63);
		}
	}
	for (int ch = 0; ch < 4; ch++)
		if (p->borrow_cnt[ch] > 0)
			p->borrow_cnt[ch] -= (int32_t)n;
}

int pokey_advance(pokey_t *p, uint32_t clocks, int16_t *out, int max)
{
	int n = 0;
	uint32_t lag = 0;   // quiet clocks not yet applied to the counters
	while (clocks) {
		const uint32_t s = POKEY_DIVIDER - p->sub;
		const uint32_t k = p->ev;
		if (k <= s && k <= clocks) {
			const uint32_t q = k - 1;
			p->acc += (uint32_t)p->level * q;
			lag += q;
			if (lag) {
				fast_forward(p, lag);
				lag = 0;
			}
			step_one_clock(p);
			p->acc += (uint32_t)p->level;
			p->sub += k;
			clocks -= k;
			p->ev = next_event(p);
		} else {
			const uint32_t m = s < clocks ? s : clocks;
			p->acc += (uint32_t)p->level * m;
			lag += m;
			p->ev -= m;
			p->sub += m;
			clocks -= m;
		}
		if (p->sub == POKEY_DIVIDER) {
			if (n < max)
				out[n++] = (int16_t)(p->acc / POKEY_DIVIDER);
			p->sub = 0;
			p->acc = 0;
		}
	}
	if (lag)
		fast_forward(p, lag);
	return n;
}

void pokey_render(pokey_t *p, int16_t *out, int n)
{
	if (n <= 0)
		return;
	pokey_advance(p, (POKEY_DIVIDER - p->sub) + (uint32_t)(n - 1) * POKEY_DIVIDER, out, n);
}

uint32_t pokey_rate(const pokey_t *p) { return p->clock / POKEY_DIVIDER; }

// ---- init / reset ----
void pokey_init(pokey_t *p, uint32_t clock)
{
	build_tables();
	memset(p, 0, sizeof(*p));
	p->clock = clock;
	// (device_start)
	for (int i = 0; i < 4; i++)
		p->AUDC[i] = 0xb0;
	p->KBCODE = 0x09;
	p->SKCTL = 0;
	p->SKSTAT = 0;
	p->IRQST = IRQ_SEROC;
	p->IRQEN = 0;
	p->AUDCTL = 0;
	p->p4 = p->p5 = p->p9 = p->p17 = 0;
	p->ALLPOT = 0x00;
	p->pot_counter = 0;
	p->kbd_cnt = 0;
	p->out_raw = 0;
	p->old_raw_inval = 1;
	p->kbd_state = 0;
	p->serin_shift = 1;
	p->serout_shift = 1;
	p->serout_full = 0;
	p->ser_iclk = 1;
	p->ser_oclk = 0;
	p->sod_twotone = 0;
	p->level = 0;
	pokey_reset(p);
}

void pokey_reset(pokey_t *p)
{
	pokey_potgo(p);   // (device_reset)
	p->ev = next_event(p);
}

void pokey_set_pot(pokey_t *p, int n, uint8_t value)
{
	if (n < 0 || n > 7)
		return;
	p->pot_set |= (uint8_t)(1 << n);
	p->pot_in[n] = value;
}

void pokey_set_irq_cb(pokey_t *p, void (*cb)(void *ctx, int state), void *ctx)
{
	p->irq_cb = cb;
	p->irq_ctx = ctx;
}

void pokey_sid_w(pokey_t *p, int state)
{
	if (state)
		p->SKSTAT &= (uint8_t)~SK_SERIN;
	else
		p->SKSTAT |= SK_SERIN;
	p->ev = next_event(p);
}

// ---- registers ----
uint8_t pokey_read(pokey_t *p, uint8_t offset)
{
	int data;
	switch (offset & 15) {
	case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7: {
		const int pot = offset & 7;
		data = (p->ALLPOT & (1 << pot)) ? p->POTx[pot] : p->pot_counter;
		break;
	}
	case ALLPOT_C:
		// (no ALLPOT callback)
		data = (p->SKCTL & SK_RESET) == 0 ? p->ALLPOT : p->ALLPOT ^ 0xff;
		break;
	case KBCODE_C:
		data = p->KBCODE;
		break;
	case RANDOM_C:
		if (p->AUDCTL & POLY9)
			data = s_poly9[p->p9] & 0xff;
		else
			data = (int)poly17_rand(p->p17);
		break;
	case SERIN_C:
		data = p->SERIN;
		break;
	case IRQST_C:
		data = p->IRQST ^ 0xff;
		break;
	case SKSTAT_C:
		data = p->SKSTAT ^ 0xff;
		break;
	default:
		data = 0xff;
		break;
	}
	return (uint8_t)data;
}

static void write_internal(pokey_t *p, uint8_t offset, uint8_t data)
{
	switch (offset & 15) {
	case AUDF1_C: p->AUDF[CHAN1] = data; break;
	case AUDC1_C: p->AUDC[CHAN1] = data; p->old_raw_inval = 1; break;
	case AUDF2_C: p->AUDF[CHAN2] = data; break;
	case AUDC2_C: p->AUDC[CHAN2] = data; p->old_raw_inval = 1; break;
	case AUDF3_C: p->AUDF[CHAN3] = data; break;
	case AUDC3_C: p->AUDC[CHAN3] = data; p->old_raw_inval = 1; break;
	case AUDF4_C: p->AUDF[CHAN4] = data; break;
	case AUDC4_C: p->AUDC[CHAN4] = data; p->old_raw_inval = 1; break;

	case AUDCTL_C:
		if (data == p->AUDCTL)
			return;
		p->AUDCTL = data;
		p->old_raw_inval = 1;
		break;

	case STIMER_C:
		for (int i = 0; i < 4; i++) {
			reset_channel(p, i);
			p->output[i] = 0;
			p->filter_sample[i] = (uint8_t)(i < 2 ? 1 : 0);
		}
		p->old_raw_inval = 1;
		break;

	case SKREST_C:
		p->SKSTAT &= (uint8_t)~(SK_FRAME | SK_OVERRUN | SK_KBERR);
		break;

	case POTGO_C:
		if (p->SKCTL & SK_RESET)
			pokey_potgo(p);
		break;

	case SEROUT_C:
		p->SEROUT = data;
		if (p->SKCTL & SK_RESET)
			p->serout_full = 1;
		break;

	case IRQEN_C:
		if (p->IRQST & ~data)
			p->IRQST &= (uint8_t)(IRQ_SEROC | data);
		p->IRQEN = data;
		if (p->IRQEN & p->IRQST & IRQ_SEROC)
			irq_w(p, 1);
		else if (!(p->IRQEN & p->IRQST))
			irq_w(p, 0);
		break;

	case SKCTL_C: {
		if (data == p->SKCTL)
			return;
		p->SKCTL = data;
		int serout_complete = 0;
		if (!(data & SK_RESET)) {
			write_internal(p, IRQEN_C, 0);
			write_internal(p, SKREST_C, 0);
			p->p9 = 0;
			p->p17 = 0;
			p->p4 = 0;
			p->p5 = 0;
			p->clock_cnt[0] = 0;
			p->clock_cnt[1] = 0;
			p->clock_cnt[2] = 0;
			p->serin_shift = 1;
			p->serout_shift = 1;
			p->serout_full = 0;
			p->SKSTAT &= (uint8_t)~SK_BUSY;
			serout_complete = 1;   // (m_serout_complete_timer->adjust(attotime::zero))
		}
		if (!(data & SK_KEYSCAN)) {
			p->SKSTAT &= (uint8_t)~SK_KEYBD;
			p->kbd_cnt = 0;
			p->kbd_state = 0;
		}
		if ((data & SK_ICLK) == 0)
			p->ser_iclk = 1;
		if ((data & SK_OCLK) == 0)
			p->ser_oclk = 0;
		if ((data & SK_ASYNC) && !(p->SKSTAT & SK_BUSY)) {
			reset_channel(p, CHAN3);
			reset_channel(p, CHAN4);
		}
		p->old_raw_inval = 1;
		if (serout_complete)
			serout_complete_irq(p);
		break;
	}
	}
}

void pokey_write(pokey_t *p, uint8_t offset, uint8_t data)
{
	// (MAME: a scheduler sync at the current time, then write_internal)
	write_internal(p, offset, data);
	p->ev = next_event(p);
}

int pokey_state(pokey_t *p, void *ctx, int load, const char *tag)
{
	const int rc = rbx_state_io(ctx, tag, p, (uint32_t)offsetof(pokey_t, state_end));
	if (load && !rc)
		p->ev = next_event(p);
	return rc;
}
