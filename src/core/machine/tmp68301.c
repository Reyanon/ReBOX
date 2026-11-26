/*  Toshiba TMP68301 peripherals. Ported from MAME src/devices/cpu/m68000/tmp68301.cpp (BSD-3-Clause,
    copyright-holders: Olivier Galibert).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "tmp68301.h"

#include <string.h>

enum {
	SCR_INTM = 0x01, SCR_RES = 0x20, SCR_CKSE = 0x80,
	SMR_ST = 0x01, SMR_TXINTM = 0x02, SMR_CL_SFT = 2, SMR_PEN = 0x10, SMR_PEO = 0x20, SMR_ERINTM = 0x40, SMR_RXINTM = 0x80,
	SCMR_TXEN = 0x01, SCMR_DTR = 0x02, SCMR_RXEN = 0x04, SCMR_SBRK = 0x08, SCMR_ERS = 0x10, SCMR_RTS = 0x20,
	SSR_TXRDY = 0x01, SSR_RXRDY = 0x02, SSR_TXE = 0x04, SSR_PE = 0x08, SSR_OE = 0x10, SSR_FE = 0x20, SSR_RBRK = 0x40,
	SR_INT_ERR = 0, SR_INT_RX = 1, SR_INT_TX = 2
};
enum { SR_IDLE, SR_START, SR_BIT_0, SR_BIT_1, SR_BIT_2, SR_BIT_3, SR_BIT_4, SR_BIT_5, SR_BIT_6, SR_BIT_7, SR_PARITY,
       SR_STOP, SR_DONE };
enum { TCR_CK = 14, TCR_P = 10, TCR_T = 8, TCR_N1 = 0x0080, TCR_RP = 0x0040, TCR_MR = 4, TCR_INT = 0x0004,
       TCR_CS = 0x0002, TCR_TS = 0x0001 };

static const int s_slot_to_priority[11] = { 9, 5, 0, -1, 7, 3, 2, 6, 8, 4, 1 };
static const int s_vector_to_slot[32] = {
	0, 1, 2, -1, 8, 9, 10, -1,
	4, 4, 4, 4, 5, 5, 5, 5,
	6, 6, 6, 6, 7, 7, 7, 7,
	-1, -1, -1, -1, -1, -1, -1, -1
};
static const int s_parallel_mode_table[16] = { 0, 2, 6, 6, 1, 3, 7, 7, 0, 4, 8, 8, 1, 5, 9, 9 };
static const int s_timer_divider[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 8, 8, 8, 8, 8, 8 };
#define BASE_TIMER_IRQ 4

static uint64_t now(tmp68301_t *t) { return t->now ? t->now() : 0; }

static void events_changed(tmp68301_t *t)   // (recompute_bcount)
{
	if (t->events_changed)
		t->events_changed();
}

// ---- the interrupt controller ----
static void interrupt_get_current(const tmp68301_t *t, uint32_t *plevel, uint8_t *pvector, uint32_t *pslot)
{
	uint32_t level = 0, slot = 0;
	uint8_t vector = 0x1f;
	const uint16_t active = t->ipr & ~t->imr;
	for (int vect = 0; vect != 32; vect++) {
		const int nslot = s_vector_to_slot[vect];
		if (nslot != -1 && (t->interrupt_state & (1u << vect)) && (active & (1 << nslot))) {
			const uint32_t nlevel = t->icr[nslot >= 3 ? nslot - 1 : nslot] & 7;
			if (!nlevel || nlevel < level || (nlevel == level && s_slot_to_priority[nslot] < s_slot_to_priority[slot]))
				continue;
			level = nlevel;
			slot = (uint32_t)nslot;
			vector = (uint8_t)vect;
		}
	}
	*plevel = level;
	*pvector = vector;
	*pslot = slot;
}

static void interrupt_update(tmp68301_t *t)
{
	uint32_t level, slot;
	uint8_t vector;
	interrupt_get_current(t, &level, &vector, &slot);
	if (t->level != level) {
		t->level = (uint8_t)level;
		if (t->set_level)
			t->set_level((int)level);
	}
}

static void interrupt_internal_trigger(tmp68301_t *t, int vector)
{
	t->interrupt_state |= 1u << vector;
	t->ipr |= (uint16_t)(1 << s_vector_to_slot[vector]);
	interrupt_update(t);
}

static int serial_validate_interrupt(const tmp68301_t *t, uint8_t vector)
{
	const int slot = vector & 3, ch = (vector - 8) >> 2;
	return (t->sr_intr[ch] & (1 << slot)) != 0;
}

uint8_t tmp68301_ack(tmp68301_t *t)   // (interrupt_callback)
{
	uint32_t level, slot;
	uint8_t vector;
	interrupt_get_current(t, &level, &vector, &slot);
	if (slot < 3 && (t->hold & (1 << slot)))   // (standard_irq_callback: a HOLD_LINE input is cleared)
		tmp68301_set_input(t, (int)slot, 0, 0);
	if (vector != 0x1f) {
		t->iisr |= (uint16_t)(1 << slot);
		if (slot >= 3 || !(t->icr[slot] & 0x08))
			t->ipr &= (uint16_t)~(1 << slot);
		t->interrupt_state &= ~(1u << vector);
		if (vector >= 8 && vector < 0x14 && !serial_validate_interrupt(t, vector))
			vector |= 3;
	}
	interrupt_update(t);
	return (uint8_t)(t->ivnr | vector);
}

void tmp68301_set_input(tmp68301_t *t, int inputnum, int state, int hold)   // (execute_set_input)
{
	if (inputnum < 3) {
		if (state && hold)
			t->hold |= (uint8_t)(1 << inputnum);
		else if (!state)
			t->hold &= (uint8_t)~(1 << inputnum);
	}
	const int prev_state = (t->ext_state >> inputnum) & 1;
	if (prev_state == (state ? 1 : 0))
		return;
	t->ext_state ^= (uint8_t)(1 << inputnum);
	if (inputnum < 3) {
		if (t->icr[inputnum] & 0x08) {   // (level)
			if (state) {
				t->ipr |= (uint16_t)(1 << inputnum);
				t->interrupt_state |= 1u << inputnum;
			} else {
				t->ipr &= (uint16_t)~(1 << inputnum);
				t->interrupt_state &= ~(1u << inputnum);
			}
		} else if (state) {   // (edge)
			t->ipr |= (uint16_t)(1 << inputnum);
			t->interrupt_state |= 1u << inputnum;
		}
	} else if (state && (t->ieir & (1 << (inputnum - 3))))
		t->ipr |= (uint16_t)(1 << (inputnum + 1));
	interrupt_update(t);
}

static void icr_w(tmp68301_t *t, int reg, uint8_t data)
{
	if (reg < 3) {
		data &= 0x1f;
		if (t->icr[reg] == data)
			return;
		t->icr[reg] = data;
		// (switching to level with the pin already on raises the interrupt)
		if ((t->icr[reg] & 0x08) && (t->ext_state & (1 << reg)))
			t->ipr |= (uint16_t)(1 << reg);
	} else {
		data &= 0x07;
		if (t->icr[reg] == data)
			return;
		t->icr[reg] = data;
	}
	interrupt_update(t);
}

static void imr_w(tmp68301_t *t, uint16_t data, uint16_t mask)
{
	const uint16_t old = t->imr;
	t->imr = (uint16_t)(((t->imr & ~mask) | (data & mask)) & 0x07f7);
	if (t->imr != old)
		interrupt_update(t);
}

static void ipr_w(tmp68301_t *t, uint16_t data, uint16_t mask)
{
	const uint16_t old = t->ipr;
	uint16_t n = (uint16_t)((t->ipr & ~mask) | (data & mask));
	// (level external interrupts can't be cleared through that register)
	if (t->icr[0] & 0x08)
		n |= 0x001;
	if (t->icr[1] & 0x08)
		n |= 0x002;
	if (t->icr[2] & 0x08)
		n |= 0x004;
	t->ipr &= n;
	if (t->ipr == old)
		return;
	uint32_t vector_mask = 0;
	for (int i = 0; i != 32; i++)
		if (s_vector_to_slot[i] == -1 || !(t->ipr & (1 << s_vector_to_slot[i])))
			vector_mask |= 1u << i;
	t->interrupt_state &= vector_mask;
	interrupt_update(t);
}

// ---- the serial channels ----
static uint8_t sbrr_to_div(uint8_t value)
{
	value |= value >> 1;
	value |= value >> 2;
	value |= value >> 4;
	value |= 0x01;
	value -= value >> 1;
	return value;
}

static void serial_check_interrupt(tmp68301_t *t, int ch)
{
	const uint8_t old_interrupt = t->sr_intr[ch];
	uint8_t intr = 0;
	if (!(t->scr & SCR_INTM)) {
		if (!(t->smr[ch] & SMR_ERINTM) && (t->scmr[ch] & SCMR_RXEN) && (t->ssr[ch] & (SSR_PE | SSR_OE | SSR_FE)))
			intr |= 1 << SR_INT_ERR;
		if (!(t->smr[ch] & SMR_RXINTM) && (t->scmr[ch] & SCMR_RXEN) && (t->ssr[ch] & SSR_RXRDY))
			intr |= 1 << SR_INT_RX;
		if (!(t->smr[ch] & SMR_TXINTM) && (t->ssr[ch] & SSR_TXRDY))
			intr |= 1 << SR_INT_TX;
		if (!(t->smr[ch] & SMR_RXINTM) && (t->ssr[ch] & SSR_RBRK))
			intr |= 1 << SR_INT_ERR;
	}
	if (intr != old_interrupt)
		for (int i = 0; i != 3; i++)
			if ((intr & (1 << i)) && !(t->sr_intr[ch] & (1 << i)))
				interrupt_internal_trigger(t, 8 + 4 * ch + i);
	t->sr_intr[ch] = intr;
}

static void ssr_set(tmp68301_t *t, int ch, uint8_t val, uint8_t mask)
{
	t->ssr[ch] = (uint8_t)((t->ssr[ch] & ~mask) | val);
	serial_check_interrupt(t, ch);
}

static void serial_clock_update(tmp68301_t *t, int ch)
{
	const uint32_t divider = (uint32_t)(t->spr ? t->spr : 256) * sbrr_to_div(t->sbrr[ch]);
	// (an external clock: none on these boards, taken as the internal one)
	t->serial_gclk[ch] = divider;
}

static uint64_t serial_get_next_edge(tmp68301_t *t, int ch)
{
	const uint64_t current_time = now(t), step = t->serial_gclk[ch] ? t->serial_gclk[ch] : 1;
	return ((current_time / step) + 1) * step;
}

static void serial_tx_update(tmp68301_t *t, int ch)
{
	uint64_t next = t->serial_tx_next[ch] + 8 * t->serial_gclk[ch];
	uint8_t nstate = (uint8_t)(t->serial_tx_state[ch] + 1);
	const int cl = (t->smr[ch] >> SMR_CL_SFT) & 3;
	switch (t->serial_tx_state[ch]) {
	case SR_START:
		t->serial_tx[ch] = t->sdrt[ch];
		ssr_set(t, ch, SSR_TXE | SSR_TXRDY, SSR_TXE | SSR_TXRDY);
		break;
	case SR_BIT_4:
		if (cl == 0)
			nstate = t->smr[ch] & SMR_PEN ? SR_PARITY : SR_STOP;
		break;
	case SR_BIT_5:
		if (cl == 1)
			nstate = t->smr[ch] & SMR_PEN ? SR_PARITY : SR_STOP;
		break;
	case SR_BIT_6:
		if (cl == 2)
			nstate = t->smr[ch] & SMR_PEN ? SR_PARITY : SR_STOP;
		break;
	case SR_BIT_7:
		nstate = t->smr[ch] & SMR_PEN ? SR_PARITY : SR_STOP;
		break;
	case SR_PARITY:
		nstate = SR_STOP;
		break;
	case SR_STOP:
		if (t->smr[ch] & SMR_ST)
			next += 8 * t->serial_gclk[ch];
		if ((t->scmr[ch] & SCMR_TXEN) && !(t->ssr[ch] & SSR_TXE))
			nstate = SR_START;
		break;
	case SR_DONE:
		next = 0;
		nstate = SR_IDLE;
		break;
	default:   // (the data bits: the line goes nowhere)
		break;
	}
	t->serial_tx_next[ch] = next;
	t->serial_tx_state[ch] = nstate;
}

static void serial_rx_update(tmp68301_t *t, int ch)
{
	// (the receive line stays high: a start bit is never seen, see serial_rx_w)
	t->serial_rx_next[ch] = 0;
	t->serial_rx_state[ch] = SR_IDLE;
}

static uint8_t sdr_r(tmp68301_t *t, int ch)
{
	if (t->ssr[ch] & SSR_RXRDY)
		ssr_set(t, ch, 0, SSR_RXRDY);
	else
		ssr_set(t, ch, SSR_OE, SSR_OE);
	return t->sdrr[ch];
}

static void sdr_w(tmp68301_t *t, int ch, uint8_t data)
{
	t->sdrt[ch] = data;
	if (t->scmr[ch] & SCMR_TXEN) {
		ssr_set(t, ch, 0, SSR_TXE | SSR_TXRDY);
		if (t->serial_tx_state[ch] == SR_DONE)
			t->serial_tx_state[ch] = SR_START;
		if (t->serial_tx_state[ch] == SR_IDLE) {
			t->serial_tx_state[ch] = SR_START;
			t->serial_tx_next[ch] = serial_get_next_edge(t, ch);
			events_changed(t);
		}
	}
}

static void smr_w(tmp68301_t *t, int ch, uint8_t data)
{
	if (t->smr[ch] == data)
		return;
	t->smr[ch] = data;
	serial_check_interrupt(t, ch);
}

static void scmr_w(tmp68301_t *t, int ch, uint8_t data)
{
	data &= ch == 0 ? 0x3f : 0x1d;
	if (!(t->scmr[ch] ^ data))
		return;
	t->scmr[ch] = data;
	if ((t->scmr[ch] & SCMR_TXEN) && (t->ssr[ch] & SSR_TXE))
		t->ssr[ch] |= SSR_TXRDY;
	serial_check_interrupt(t, ch);
}

static void sbrr_w(tmp68301_t *t, int ch, uint8_t data)
{
	if (t->sbrr[ch] == data)
		return;
	t->sbrr[ch] = data;
	serial_clock_update(t, ch);
}

static void spr_w(tmp68301_t *t, uint8_t data)
{
	if (t->spr == data)
		return;
	t->spr = data;
	for (int ch = 0; ch < 3; ch++)
		serial_clock_update(t, ch);
}

static void scr_w(tmp68301_t *t, uint8_t data)
{
	data &= 0xa1;
	if (t->scr == data)
		return;
	t->scr = data;
	for (int ch = 0; ch < 3; ch++)
		serial_clock_update(t, ch);
	for (int ch = 0; ch < 3; ch++)
		serial_check_interrupt(t, ch);
}

// ---- the timers ----
static void timer_sync(tmp68301_t *t, int ch)
{
	if (!(t->tcr[ch] & TCR_TS)) {
		t->tctr[ch] = 0;
		return;
	}
	if (t->tcr[ch] & TCR_CS)
		return;
	const uint32_t div = (uint32_t)s_timer_divider[(t->tcr[ch] >> TCR_P) & 15];
	const uint64_t ctime = now(t);
	// (the shifts not folded, the computation would be incorrect)
	uint32_t ntctr = t->tctr[ch] + (uint32_t)((ctime >> div) - (t->timer_last_sync[ch] >> div));
	const uint32_t maxmode = (t->tcr[ch] >> TCR_MR) & 3;
	if (maxmode == 1 || maxmode == 2) {
		uint32_t max = maxmode == 1 ? t->tmcr1[ch] : t->tmcr2[ch];
		if (max == 0)
			max = 0x10000;
		if (t->tctr[ch] >= max) {
			if (ntctr >= 0x10000)
				ntctr -= 0x10000;
			else
				max = 0x10000;
		}
		if (ntctr >= max) {
			if (t->tcr[ch] & TCR_INT)
				interrupt_internal_trigger(t, BASE_TIMER_IRQ + ch);
			ntctr = ntctr % max;
		}
	}
	t->tctr[ch] = (uint16_t)ntctr;
	t->timer_last_sync[ch] = ctime;
}

static void timer_predict(tmp68301_t *t, int ch)
{
	if (!(t->tcr[ch] & TCR_TS))
		t->tctr[ch] = 0;
	if ((t->tcr[ch] & (TCR_INT | TCR_CS | TCR_TS)) != (TCR_INT | TCR_TS)) {
		t->timer_next[ch] = 0;
		return;
	}
	const uint32_t maxmode = (t->tcr[ch] >> TCR_MR) & 3;
	if (maxmode == 0 || maxmode == 3)   // (no max selected; alternating max mode unsupported)
		return;
	if ((t->tcr[ch] & TCR_N1) == 0)     // (single-shot mode unsupported)
		return;
	if (((t->tcr[ch] >> TCR_CK) & 3) != 0)   // (only the internal clock)
		return;
	int32_t delta = (int32_t)(maxmode == 1 ? t->tmcr1[ch] : t->tmcr2[ch]) - (int32_t)t->tctr[ch];
	if (delta <= 0)
		delta += 0x10000;
	const uint32_t div = (uint32_t)s_timer_divider[(t->tcr[ch] >> TCR_P) & 15];
	const uint64_t ctime = now(t);
	t->timer_next[ch] = ((ctime >> div) + (uint64_t)delta) << div;
	events_changed(t);
}

static void tcr_w(tmp68301_t *t, int ch, uint16_t data, uint16_t mask)
{
	timer_sync(t, ch);
	const uint16_t old = t->tmcr1[ch];   // (as MAME: the max register, not the control one)
	t->tcr[ch] = (uint16_t)((t->tcr[ch] & ~mask) | (data & mask));
	t->tcr[ch] &= ch == 0 ? 0xff87 : 0xfff7;
	if (!ch)
		t->tcr[ch] |= 0x0050;
	if (t->tcr[ch] == old)
		return;
	timer_predict(t, ch);
}

static void tmcr_w(tmp68301_t *t, uint16_t *reg, int ch, uint16_t data, uint16_t mask)
{
	timer_sync(t, ch);
	const uint16_t old = *reg;
	*reg = (uint16_t)((*reg & ~mask) | (data & mask));
	if (*reg == old)
		return;
	timer_predict(t, ch);
}

static void tctr_w(tmp68301_t *t, int ch)
{
	timer_sync(t, ch);
	if (ch)
		t->tctr[ch] = 0;
	timer_predict(t, ch);
}

// ---- public ----
void tmp68301_init(tmp68301_t *t, uint32_t clock)
{
	memset(t, 0, sizeof(*t));
	t->clock = clock;
}

void tmp68301_reset(tmp68301_t *t)   // (device_reset)
{
	memset(t, 0, TMP68301_STATE_SIZE);   // (the parallel data and the serial clocks too: MAME's power on values)
	t->amar[0] = t->amar[1] = 0x00;
	t->aamr[0] = t->aamr[1] = 0xff;
	t->aacr[0] = 0x3d;
	t->aacr[1] = 0x18;
	t->aacr[2] = 0x18;
	t->ator = 0x08;
	t->arelr = 0xfffc;
	for (int i = 0; i != 10; i++)
		t->icr[i] = 0x07;
	t->imr = 0x7f7;
	t->parallel_mode = (uint8_t)s_parallel_mode_table[0];
	t->psr = 0x40;
	t->scr = SCR_CKSE | SCR_RES | SCR_INTM;
	for (int i = 0; i != 3; i++) {
		t->smr[i] = SMR_RXINTM | SMR_ERINTM | SMR_TXINTM;
		t->scmr[i] = SCMR_ERS;
		t->sbrr[i] = 0x01;
		t->ssr[i] = SSR_TXE;
		t->serial_tx_state[i] = SR_IDLE;
		t->serial_rx_state[i] = SR_IDLE;
		serial_clock_update(t, i);
	}
	for (int i = 0; i != 3; i++)
		t->tcr[i] = i ? 0x0012 : 0x0052;
	if (t->set_level)
		t->set_level(0);
}

static uint16_t pdr_r(tmp68301_t *t)
{
	if (t->parallel_mode == 0 || t->parallel_mode == 1) {
		if (t->pdir == 0xffff)
			return t->pdr;
		const uint16_t in = t->par_r ? t->par_r() : 0xffff;
		return (uint16_t)((t->pdr & t->pdir) | (in & ~t->pdir));
	}
	return t->pdr;
}

static void pdr_w(tmp68301_t *t, uint16_t data, uint16_t mask)
{
	const uint16_t old = t->pdr;
	t->pdr = (uint16_t)((t->pdr & ~mask) | (data & mask));
	if (t->pdr == old)
		return;
	if (t->parallel_mode == 0 || t->parallel_mode == 1) {
		if (t->pdir == 0x0000)
			return;
		if (t->par_w)
			t->par_w((uint16_t)(t->pdr & t->pdir));
	}
}

static void pdir_w(tmp68301_t *t, uint16_t data, uint16_t mask)
{
	const uint16_t old = t->pdir;
	t->pdir = (uint16_t)((t->pdir & ~mask) | (data & mask));
	if (t->pdir == old)
		return;
	if (t->pdir && t->par_w)
		t->par_w((uint16_t)(t->pdr & t->pdir));
}

// a byte register (the even or odd address): its value, -1 = none there
static int byte_r(tmp68301_t *t, uint32_t a)
{
	if (a >= 0xfffc80 && a <= 0xfffc93)
		return (a & 1) ? t->icr[(a - 0xfffc80) >> 1] : -1;
	switch (a) {
	case 0xfffc00: return t->amar[0];
	case 0xfffc01: return t->aamr[0];
	case 0xfffc03: return t->aacr[0];
	case 0xfffc04: return t->amar[1];
	case 0xfffc05: return t->aamr[1];
	case 0xfffc07: return t->aacr[1];
	case 0xfffc09: return t->aacr[2];
	case 0xfffc0b: return t->ator;
	case 0xfffc9b: return t->ivnr;
	case 0xfffc9d: return t->ivnr;   // (as MAME: ivnr_r there too)
	case 0xfffd03: return t->pcr;
	case 0xfffd05: return t->psr;
	case 0xfffd07: return t->pcmr;
	case 0xfffd08: case 0xfffd09: return t->pmr;
	case 0xfffd0d: return t->ppr1;
	case 0xfffd0f: return t->ppr2;
	case 0xfffd81: return t->smr[0];
	case 0xfffd83: return t->scmr[0];
	case 0xfffd85: return t->sbrr[0];
	case 0xfffd87: return t->ssr[0];
	case 0xfffd89: return sdr_r(t, 0);
	case 0xfffd8d: return t->spr;
	case 0xfffd8f: return t->scr;
	case 0xfffd91: return t->smr[1];
	case 0xfffd93: return t->scmr[1];
	case 0xfffd95: return t->sbrr[1];
	case 0xfffd97: return t->ssr[1];
	case 0xfffd99: return sdr_r(t, 1);
	case 0xfffda1: return t->smr[2];
	case 0xfffda3: return t->scmr[2];
	case 0xfffda5: return t->sbrr[2];
	case 0xfffda7: return t->ssr[2];
	case 0xfffda9: return sdr_r(t, 2);
	default: return -1;
	}
}

static void byte_w(tmp68301_t *t, uint32_t a, uint8_t v)
{
	if (a >= 0xfffc80 && a <= 0xfffc93) {
		if (a & 1)
			icr_w(t, (int)((a - 0xfffc80) >> 1), v);
		return;
	}
	switch (a) {
	case 0xfffc00: t->amar[0] = v; break;
	case 0xfffc01: t->aamr[0] = v; break;
	case 0xfffc03: t->aacr[0] = v & 0x3f; break;
	case 0xfffc04: t->amar[1] = v; break;
	case 0xfffc05: t->aamr[1] = v; break;
	case 0xfffc07: t->aacr[1] = v & 0x3f; break;
	case 0xfffc09: t->aacr[2] = v & 0x1f; break;
	case 0xfffc0b: t->ator = v & 0x0f; break;
	case 0xfffc9b: t->ivnr = v & 0xe0; break;
	case 0xfffc9d: t->ieir = v & 0x7f; break;
	case 0xfffd03:
		t->pcr = v & 0x0f;
		t->parallel_mode = (uint8_t)s_parallel_mode_table[t->pcr];
		break;
	case 0xfffd05: break;   // (psr_w: nothing kept)
	case 0xfffd07: t->pcmr = v & 0x1f; break;
	case 0xfffd08: case 0xfffd09: t->pmr = v & 0x0f; break;
	case 0xfffd0d: t->ppr1 = v & 0x7f; break;
	case 0xfffd0f: t->ppr2 = v & 0x7f; break;
	case 0xfffd81: smr_w(t, 0, v); break;
	case 0xfffd83: scmr_w(t, 0, v); break;
	case 0xfffd85: sbrr_w(t, 0, v); break;
	case 0xfffd89: sdr_w(t, 0, v); break;
	case 0xfffd8d: spr_w(t, v); break;
	case 0xfffd8f: scr_w(t, v); break;
	case 0xfffd91: smr_w(t, 1, v); break;
	case 0xfffd93: scmr_w(t, 1, v); break;
	case 0xfffd95: sbrr_w(t, 1, v); break;
	case 0xfffd99: sdr_w(t, 1, v); break;
	case 0xfffda1: smr_w(t, 2, v); break;
	case 0xfffda3: scmr_w(t, 2, v); break;
	case 0xfffda5: sbrr_w(t, 2, v); break;
	case 0xfffda9: sdr_w(t, 2, v); break;
	default: break;
	}
}

uint16_t tmp68301_read(tmp68301_t *t, uint32_t a)
{
	a &= 0xfffffe;
	switch (a) {   // (the word registers)
	case 0xfffc0c: return t->arelr;
	case 0xfffc94: return t->imr;
	case 0xfffc96: return t->ipr;
	case 0xfffc98: return t->iisr;
	case 0xfffd00: return t->pdir;
	case 0xfffd0a: return pdr_r(t);
	case 0xfffe00: return t->tcr[0];
	case 0xfffe04: return t->tmcr1[0];
	case 0xfffe0c: timer_sync(t, 0); return t->tctr[0];
	case 0xfffe20: return t->tcr[1];
	case 0xfffe24: return t->tmcr1[1];
	case 0xfffe28: return t->tmcr2[1];
	case 0xfffe2c: timer_sync(t, 1); return t->tctr[1];
	case 0xfffe40: return t->tcr[1];   // (as MAME: tcr1_r)
	case 0xfffe44: return t->tmcr1[2];
	case 0xfffe48: return t->tmcr2[2];
	case 0xfffe4c: timer_sync(t, 2); return t->tctr[2];
	default: break;
	}
	const int hi = byte_r(t, a), lo = byte_r(t, a + 1);
	return (uint16_t)((hi < 0 ? 0 : hi) << 8 | (lo < 0 ? 0 : lo));
}

void tmp68301_write(tmp68301_t *t, uint32_t a, uint16_t data, uint16_t mask)
{
	a &= 0xfffffe;
	switch (a) {
	case 0xfffc0c: t->arelr = (uint16_t)(((t->arelr & ~mask) | (data & mask)) & 0xfffc); return;
	case 0xfffc94: imr_w(t, data, mask); return;
	case 0xfffc96: ipr_w(t, data, mask); return;
	case 0xfffc98: t->iisr &= (uint16_t)((t->iisr & ~mask) | (data & mask)); return;
	case 0xfffd00: pdir_w(t, data, mask); return;
	case 0xfffd0a: pdr_w(t, data, mask); return;
	case 0xfffe00: tcr_w(t, 0, data, mask); return;
	case 0xfffe04: tmcr_w(t, &t->tmcr1[0], 0, data, mask); return;
	case 0xfffe0c: tctr_w(t, 0); return;
	case 0xfffe20: tcr_w(t, 1, data, mask); return;
	case 0xfffe24: tmcr_w(t, &t->tmcr1[1], 1, data, mask); return;
	case 0xfffe28: tmcr_w(t, &t->tmcr2[1], 1, data, mask); return;
	case 0xfffe2c: tctr_w(t, 1); return;
	case 0xfffe40: tcr_w(t, 2, data, mask); return;
	case 0xfffe44: tmcr_w(t, &t->tmcr1[2], 2, data, mask); return;
	case 0xfffe48: tmcr_w(t, &t->tmcr2[2], 2, data, mask); return;
	case 0xfffe4c: tctr_w(t, 2); return;
	default: break;
	}
	if (mask & 0xff00)
		byte_w(t, a, (uint8_t)(data >> 8));
	if (mask & 0x00ff)
		byte_w(t, a + 1, (uint8_t)data);
}

uint64_t tmp68301_next_event(const tmp68301_t *t)
{
	uint64_t ev = 0;
	for (int i = 0; i != 3; i++) {
		if (t->serial_tx_next[i] && (!ev || t->serial_tx_next[i] < ev))
			ev = t->serial_tx_next[i];
		if (t->serial_rx_next[i] && (!ev || t->serial_rx_next[i] < ev))
			ev = t->serial_rx_next[i];
		if (t->timer_next[i] && (!ev || t->timer_next[i] < ev))
			ev = t->timer_next[i];
	}
	return ev;
}

void tmp68301_update(tmp68301_t *t, uint64_t current_time)   // (internal_update)
{
	for (int i = 0; i != 3; i++) {
		if (t->serial_tx_next[i] && current_time >= t->serial_tx_next[i])
			serial_tx_update(t, i);
		if (t->serial_rx_next[i] && current_time >= t->serial_rx_next[i])
			serial_rx_update(t, i);
	}
	for (int i = 0; i != 3; i++)
		if (t->timer_next[i] && current_time >= t->timer_next[i]) {   // (timer_update)
			timer_sync(t, i);
			timer_predict(t, i);
		}
}
