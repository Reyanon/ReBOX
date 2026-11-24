/*  Hitachi SH-2 (SH7604) CPU, ported to C from MAME 0.289 src/devices/cpu/sh: sh.cpp, sh.h (BSD-3-Clause,
    copyright-holders: David Haywood), sh2.cpp, sh2.h, sh7604.cpp, sh7604.h (BSD-3-Clause, copyright-holders:
    Juergen Buchmueller, R. Belmont). MAME's sh2 core is based on the C/C++ SH-2 core by <tiraniddo@hotmail.com>,
    with help from Chuck Mason and Olivier Galibert (SEMU).
    See sh2.h for the scope and the differences from MAME.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../cheat_cpu.h"
#include "sh2.h"
#include <string.h>

#define SH_T     0x00000001u
#define SH_S     0x00000002u
#define SH_I     0x000000f0u
#define SH_Q     0x00000100u
#define SH_M     0x00000200u
#define SH_FLAGS (SH_M | SH_Q | SH_I | SH_S | SH_T)

#define SH2_AM   0xc7ffffffu

// FRT / FTCSR bits (sh7604.h)
#define ICF   0x80
#define OCFA  0x08
#define OCFB  0x04
#define OVF   0x02
#define CCLRA 0x01

#if defined(__GNUC__)
#define LIKELY(x)   __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)
#define SH2_INLINE  static inline __attribute__((always_inline))
#define SH2_NOINLINE static __attribute__((noinline))
#else
#define LIKELY(x)   (x)
#define UNLIKELY(x) (x)
#define SH2_INLINE  static inline
#define SH2_NOINLINE static
#endif

static const int div_tab[4] = { 3, 5, 7, 0 };
static const int wdtclk_tab[8] = { 1, 6, 7, 8, 9, 10, 12, 13 };

static void recalc_irq(sh2_t *c);
static void timer_activate(sh2_t *c);

/***************************************************************************
    clock and events
***************************************************************************/

SH2_INLINE int64_t now_cycles(const sh2_t *c)
{
	return c->running ? c->slice_end - c->icount : c->total_cycles;
}

int64_t sh2_total_cycles(const sh2_t *c) { return now_cycles(c); }

// the clock of a running DMA channel's last step (the one that completes the transfer)
static int64_t dma_end_time(const sh2_t *c, int ch)
{
	uint32_t steps = (c->dma_size[ch] == 3) ? c->dma_count[ch] / 4 : c->dma_count[ch];
	return c->dma_tick[ch] + 2 * (int64_t)steps;
}

static int64_t next_event(const sh2_t *c)
{
	int64_t e = c->frt_event;
	if (c->wdt_event < e) e = c->wdt_event;
	for (int ch = 0; ch < 2; ch++)
		if (c->dma_active[ch] == 1) {
			int64_t d = dma_end_time(c, ch);
			if (d < e) e = d;
		}
	return e;
}

// an event was moved earlier: end the running slice there (now is unchanged)
static void resched(sh2_t *c)
{
	if (!c->running) return;
	int64_t e = next_event(c);
	if (e < c->slice_end) {
		c->icount -= (int32_t)(c->slice_end - e);
		c->slice_end = e;
	}
}

/***************************************************************************
    bus
***************************************************************************/

static uint32_t onchip_r(sh2_t *c, uint32_t a, uint32_t mask);
static void onchip_w(sh2_t *c, uint32_t a, uint32_t data, uint32_t mask);

// raw program space accesses (MAME m_program): a is the address as the space sees it
SH2_NOINLINE uint8_t raw_slow_r8(sh2_t *c, uint32_t a)
{
	if (a < 0x08000000) return c->read8(c->user, a);
	if (a >= 0x40000000 && a < 0xc0000000) return 0xa5;
	if (a >= 0xfffffe00) { int sh = (3 - (a & 3)) * 8; return (uint8_t)(onchip_r(c, a & ~3u, 0xffu << sh) >> sh); }
	return c->read8(c->user, a);
}

SH2_NOINLINE uint16_t raw_slow_r16(sh2_t *c, uint32_t a)
{
	a &= ~1u;
	if (a < 0x08000000) return c->read16(c->user, a);
	if (a >= 0x40000000 && a < 0xc0000000) return 0xa5a5;
	if (a >= 0xfffffe00) { int sh = (a & 2) ? 0 : 16; return (uint16_t)(onchip_r(c, a & ~3u, 0xffffu << sh) >> sh); }
	return c->read16(c->user, a);
}

SH2_NOINLINE uint32_t raw_slow_r32(sh2_t *c, uint32_t a)
{
	a &= ~3u;
	if (a < 0x08000000) return c->read32(c->user, a);
	if (a >= 0x40000000 && a < 0xc0000000) return 0xa5a5a5a5;
	if (a >= 0xfffffe00) return onchip_r(c, a, 0xffffffffu);
	return c->read32(c->user, a);
}

SH2_NOINLINE void raw_slow_w8(sh2_t *c, uint32_t a, uint8_t v)
{
	if (a < 0x08000000) { c->write8(c->user, a, v); return; }
	if (a >= 0x40000000 && a < 0xc0000000) return;
	if (a >= 0xfffffe00) { int sh = (3 - (a & 3)) * 8; onchip_w(c, a & ~3u, (uint32_t)v << sh, 0xffu << sh); return; }
	c->write8(c->user, a, v);
}

SH2_NOINLINE void raw_slow_w16(sh2_t *c, uint32_t a, uint16_t v)
{
	a &= ~1u;
	if (a < 0x08000000) { c->write16(c->user, a, v); return; }
	if (a >= 0x40000000 && a < 0xc0000000) return;
	if (a >= 0xfffffe00) { int sh = (a & 2) ? 0 : 16; onchip_w(c, a & ~3u, (uint32_t)v << sh, 0xffffu << sh); return; }
	c->write16(c->user, a, v);
}

SH2_NOINLINE void raw_slow_w32(sh2_t *c, uint32_t a, uint32_t v)
{
	a &= ~3u;
	if (a < 0x08000000) { c->write32(c->user, a, v); return; }
	if (a >= 0x40000000 && a < 0xc0000000) return;
	if (a >= 0xfffffe00) { onchip_w(c, a, v, 0xffffffffu); return; }
	c->write32(c->user, a, v);
}

#define PAGE(a) (((a) >> SH2_PAGE_SHIFT) & (SH2_PAGES - 1))

// raw accesses (DMA): a < 0x08000000 goes through the page table
SH2_INLINE uint8_t raw_r8(sh2_t *c, uint32_t a)
{
	if (a < 0x08000000) { const uint8_t *p = c->rd[PAGE(a)]; if (p) return p[(a & 0xfff) ^ 3]; }
	return raw_slow_r8(c, a);
}
SH2_INLINE uint16_t raw_r16(sh2_t *c, uint32_t a)
{
	if (a < 0x08000000) { const uint8_t *p = c->rd[PAGE(a)]; if (p) return *(const uint16_t *)(p + ((a & 0xffe) ^ 2)); }
	return raw_slow_r16(c, a);
}
SH2_INLINE uint32_t raw_r32(sh2_t *c, uint32_t a)
{
	if (a < 0x08000000) { const uint8_t *p = c->rd[PAGE(a)]; if (p) return *(const uint32_t *)(p + (a & 0xffc)); }
	return raw_slow_r32(c, a);
}
SH2_INLINE void raw_w8(sh2_t *c, uint32_t a, uint8_t v)
{
	if (a < 0x08000000) { uint8_t *p = c->wr[PAGE(a)]; if (p) { p[(a & 0xfff) ^ 3] = v; return; } }
	raw_slow_w8(c, a, v);
}
SH2_INLINE void raw_w16(sh2_t *c, uint32_t a, uint16_t v)
{
	if (a < 0x08000000) { uint8_t *p = c->wr[PAGE(a)]; if (p) { *(uint16_t *)(p + ((a & 0xffe) ^ 2)) = v; return; } }
	raw_slow_w16(c, a, v);
}
SH2_INLINE void raw_w32(sh2_t *c, uint32_t a, uint32_t v)
{
	if (a < 0x08000000) { uint8_t *p = c->wr[PAGE(a)]; if (p) { *(uint32_t *)(p + (a & 0xffc)) = v; return; } }
	raw_slow_w32(c, a, v);
}

// CPU accesses (sh2_device::read_byte ...): addresses with bits 31-28 = 0 or 2 are masked with 0xc7ffffff
#define CPU_FAST(a) (((a) & 0xd0000000u) == 0)

SH2_INLINE uint8_t RB(sh2_t *c, uint32_t a)
{
	if (LIKELY(CPU_FAST(a))) {
		const uint8_t *p = c->rd[PAGE(a)];
		if (LIKELY(p != NULL)) return p[(a & 0xfff) ^ 3];
		return c->read8(c->user, a & 0x07ffffff);
	}
	return raw_slow_r8(c, a);
}
SH2_INLINE uint16_t RW(sh2_t *c, uint32_t a)
{
	if (LIKELY(CPU_FAST(a))) {
		const uint8_t *p = c->rd[PAGE(a)];
		if (LIKELY(p != NULL)) return *(const uint16_t *)(p + ((a & 0xffe) ^ 2));
		return c->read16(c->user, a & 0x07fffffe);
	}
	return raw_slow_r16(c, a);
}
SH2_INLINE uint32_t RL(sh2_t *c, uint32_t a)
{
	if (LIKELY(CPU_FAST(a))) {
		const uint8_t *p = c->rd[PAGE(a)];
		if (LIKELY(p != NULL)) return *(const uint32_t *)(p + (a & 0xffc));
		return c->read32(c->user, a & 0x07fffffc);
	}
	return raw_slow_r32(c, a);
}
SH2_INLINE void WB(sh2_t *c, uint32_t a, uint32_t v)
{
	if (LIKELY(CPU_FAST(a))) {
		uint8_t *p = c->wr[PAGE(a)];
		if (LIKELY(p != NULL)) { p[(a & 0xfff) ^ 3] = (uint8_t)v; return; }
		c->write8(c->user, a & 0x07ffffff, (uint8_t)v);
		return;
	}
	raw_slow_w8(c, a, (uint8_t)v);
}
SH2_INLINE void WW(sh2_t *c, uint32_t a, uint32_t v)
{
	if (LIKELY(CPU_FAST(a))) {
		uint8_t *p = c->wr[PAGE(a)];
		if (LIKELY(p != NULL)) { *(uint16_t *)(p + ((a & 0xffe) ^ 2)) = (uint16_t)v; return; }
		c->write16(c->user, a & 0x07fffffe, (uint16_t)v);
		return;
	}
	raw_slow_w16(c, a, (uint16_t)v);
}
SH2_INLINE void WL(sh2_t *c, uint32_t a, uint32_t v)
{
	if (LIKELY(CPU_FAST(a))) {
		uint8_t *p = c->wr[PAGE(a)];
		if (LIKELY(p != NULL)) { *(uint32_t *)(p + (a & 0xffc)) = v; return; }
		c->write32(c->user, a & 0x07fffffc, v);
		return;
	}
	raw_slow_w32(c, a, v);
}

uint8_t sh2_read8(sh2_t *c, uint32_t a) { return RB(c, a); }
uint16_t sh2_read16(sh2_t *c, uint32_t a) { return RW(c, a); }
uint32_t sh2_read32(sh2_t *c, uint32_t a) { return RL(c, a); }
void sh2_write8(sh2_t *c, uint32_t a, uint8_t v) { WB(c, a, v); }
void sh2_write16(sh2_t *c, uint32_t a, uint16_t v) { WW(c, a, v); }
void sh2_write32(sh2_t *c, uint32_t a, uint32_t v) { WL(c, a, v); }

// instruction fetch (MAME: m_decrypted_program->read_word(pc >= 0x40000000 ? pc : pc & AM)). A page found in the
// table becomes the fetch page: fetch_page = the unmasked pc >> 12, fetch_base = its host page
static inline uint16_t fetch_in(sh2_t *c, uint32_t pc)
{
	uint32_t a = (pc >= 0x40000000) ? pc : (pc & SH2_AM);
	if (c->op_lo && a - c->op_lo_base < c->op_lo_size) {   // (a second fetch-only area: CPS3's decrypted flash)
		c->fetch_page = pc >> SH2_PAGE_SHIFT;
		c->fetch_base = c->op_lo + ((a - c->op_lo_base) & ~0xfffu);
		return *(const uint16_t *)(c->fetch_base + ((a & 0xffe) ^ 2));
	}
	if (c->op_hi && a - c->op_hi_base < c->op_hi_size) {   // (the fetch-only area: whole pages of it become fetch pages)
		const uint8_t *p = c->op_hi + ((a - c->op_hi_base) & ~0xfffu);
		if (!((a - c->op_hi_base) & ~0xfffu) && c->op_hi_size < 0x1000)
			return *(const uint16_t *)(c->op_hi + (((a - c->op_hi_base) & ~1u) ^ 2));
		c->fetch_page = pc >> SH2_PAGE_SHIFT;
		c->fetch_base = p;
		return *(const uint16_t *)(p + ((a & 0xffe) ^ 2));
	}
	if (a < 0x08000000) {
		const uint8_t *p = c->rd[PAGE(a)];
		if (p) {
			c->fetch_page = pc >> SH2_PAGE_SHIFT;
			c->fetch_base = p;
			return *(const uint16_t *)(p + ((a & 0xffe) ^ 2));
		}
		return c->read16(c->user, a & ~1u);
	}
	return raw_slow_r16(c, a);
}

SH2_NOINLINE uint16_t fetch_slow(sh2_t *c, uint32_t pc)
{
	if (LIKELY(!c->fetch_trace))
		return fetch_in(c, pc);
	c->fetch_trace(c->user, pc);
	const uint16_t op = fetch_in(c, pc);
	c->fetch_page = ~0u;   // (every fetch comes back here while traced)
	return op;
}

/***************************************************************************
    map
***************************************************************************/

int sh2_map(sh2_t *c, uint32_t start, uint32_t end, void *mem, uint32_t size, int readable, int writable)
{
	if ((start & 0xfff) || (end & 0xfff) != 0xfff || end < start || end > 0x07ffffff) return -1;
	if (size == 0) size = end - start + 1;
	if (size & 0xfff) return -1;
	for (uint32_t a = start, off = 0; ; a += 0x1000) {
		uint8_t *p = mem ? (uint8_t *)mem + off : NULL;
		if (!mem || readable) c->rd[PAGE(a)] = p;
		if (!mem || writable) c->wr[PAGE(a)] = p;
		off += 0x1000;
		if (off >= size) off = 0;
		if (a >= end - 0xfff) break;
	}
	c->fetch_page = ~0u;
	c->fetch_base = NULL;
	return 0;
}

void sh2_swap32(void *p, uint32_t size)
{
	uint8_t *b = (uint8_t *)p;
	for (uint32_t i = 0; i + 3 < size; i += 4) {
		uint8_t t0 = b[i], t1 = b[i + 1];
		b[i] = b[i + 3]; b[i + 1] = b[i + 2]; b[i + 2] = t1; b[i + 3] = t0;
	}
}

/***************************************************************************
    exceptions (sh2_device / sh7604_device)
***************************************************************************/

static void exception_internal(sh2_t *c, int irqline, int vector)
{
	c->r[15] -= 4;
	WL(c, c->r[15], c->sr);        // push SR onto stack
	c->r[15] -= 4;
	WL(c, c->r[15], c->pc);        // push PC onto stack

	// set I flags in SR
	if (irqline > 15)
		c->sr = c->sr | SH_I;
	else
		c->sr = (c->sr & ~SH_I) | ((uint32_t)irqline << 4);

	// fetch PC
	c->pc = RL(c, c->vbr + (uint32_t)vector * 4);

	if (c->sleep_mode == 1)
		c->sleep_mode = 2;
}

// MAME standard_irq_callback: a held line is dropped, the driver gives the vector
static int irq_callback(sh2_t *c, int line)
{
	if (line >= 0 && line < 16 && c->irq_line_state[line] == SH2_HOLD)
		sh2_set_irq_line(c, line, SH2_CLEAR);
	return c->irq_ack ? c->irq_ack(c->user, line) : 0;
}

static void sh2_exception(sh2_t *c, int irqline)
{
	int vector;

	if (irqline != 16) {
		if (irqline <= (int)((c->sr >> 4) & 15))   // if the cpu forbids this interrupt
			return;

		// if this is an sh2 internal irq, use its vector
		if (c->internal_irq_level == irqline) {
			vector = c->internal_irq_vector;
			// avoid spurious irqs with this (TODO: needs a better fix)
			c->internal_irq_level = -1;
		} else {
			if (c->vecmd)
				vector = irq_callback(c, irqline);
			else {
				irq_callback(c, irqline);
				vector = 64 + irqline / 2;
			}
		}
	} else
		vector = 11;

	exception_internal(c, irqline, vector);
}

static int bit_width(uint32_t v)
{
	int n = 0;
	while (v) { n++; v >>= 1; }
	return n;
}

static void check_pending_irq(sh2_t *c)
{
	if (c->pending_nmi) {
		sh2_exception(c, 16);
		c->pending_nmi = 0;
	} else {
		int irq = c->internal_irq_level;
		if (c->pending_irq) {
			int external_irq = bit_width(c->pending_irq) - 1;
			if (external_irq >= irq)
				irq = external_irq;
		}
		if (irq >= 0)
			sh2_exception(c, irq);
	}
}

void sh2_set_irq_line(sh2_t *c, int line, int state)
{
	if (line == SH2_INPUT_NMI) {
		if (c->nmi_line_state == state)
			return;
		c->nmi_line_state = (uint8_t)state;
		if (state != SH2_CLEAR) {
			c->pending_nmi = 1;
			if (c->delay || c->running)
				c->test_irq = 1;
			else
				check_pending_irq(c);
		}
		return;
	}
	if (line < 0 || line > 15) return;
	if (c->irq_line_state[line] == state)
		return;
	c->irq_line_state[line] = (uint8_t)state;
	if (state == SH2_CLEAR)
		c->pending_irq &= ~(1u << line);
	else {
		c->pending_irq |= 1u << line;
		if (c->delay || c->running)
			c->test_irq = 1;
		else
			check_pending_irq(c);
	}
}

/***************************************************************************
    FRT
***************************************************************************/

static void timer_resync(sh2_t *c)
{
	// TODO (MAME): setting 3 is "External clock: count on rising edge"
	int divider = div_tab[c->frc_tcr & 3];
	int64_t cur_time = now_cycles(c);
	uint64_t add = (uint64_t)(cur_time - c->frc_base) >> divider;

	if (add > 0) {
		if (divider)
			c->frc += (uint16_t)add;
		c->frc_base = cur_time;
	}
}

static void timer_activate(sh2_t *c)
{
	int max_delta = 0xfffff;

	c->frt_event = SH2_NEVER;

	uint16_t frc = c->frc;
	if (!(c->ftcsr & OCFA)) {
		uint16_t delta = (uint16_t)(c->ocra - frc);
		if (delta < max_delta)
			max_delta = delta;
	}

	if (!(c->ftcsr & OCFB) && (c->ocra <= c->ocrb || !(c->ftcsr & CCLRA))) {
		uint16_t delta = (uint16_t)(c->ocrb - frc);
		if (delta < max_delta)
			max_delta = delta;
	}

	if (!(c->ftcsr & OVF) && !(c->ftcsr & CCLRA)) {
		int delta = 0x10000 - frc;
		if (delta < max_delta)
			max_delta = delta;
	}

	if (max_delta != 0xfffff) {
		int divider = div_tab[c->frc_tcr & 3];
		if (divider) {
			int64_t t = now_cycles(c);
			c->frc_base = t;
			c->frt_event = t + ((int64_t)max_delta << divider);
			resched(c);
		}
		// else: an event in max_delta cycles of the external clock (MAME only logs it)
	}
}

static void timer_callback(sh2_t *c)
{
	c->frt_event = SH2_NEVER;
	timer_resync(c);
	uint16_t frc = c->frc;

	if (frc == c->ocrb)
		c->ftcsr |= OCFB;

	if (frc == 0x0000)
		c->ftcsr |= OVF;

	if (frc == c->ocra) {
		c->ftcsr |= OCFA;
		if (c->ftcsr & CCLRA)
			c->frc = 0;
	}

	recalc_irq(c);
	timer_activate(c);
}

void sh2_set_frt_input(sh2_t *c, int state)
{
	state = state ? 1 : 0;
	if (c->frt_input == state)
		return;

	c->frt_input = (uint8_t)state;

	if (c->frc_tcr & 0x80) {
		if (!state)
			return;
	} else {
		if (state)
			return;
	}

	timer_resync(c);
	c->frc_icr = c->frc;
	c->ftcsr |= ICF;
	recalc_irq(c);
}

/***************************************************************************
    WDT
***************************************************************************/

static void wtcnt_recalc(sh2_t *c)
{
	if (c->wdt_event != SH2_NEVER) {
		int64_t rem = c->wdt_event - now_cycles(c);
		if (rem < 0) rem = 0;
		c->wtcnt = (uint8_t)(0x100 - (rem >> wdtclk_tab[c->wtcsr & 7]));
	}
}

static void wdt_activate(sh2_t *c)
{
	c->wdt_event = now_cycles(c) + ((int64_t)(0x100 - c->wtcnt) << wdtclk_tab[c->wtcsr & 7]);
	resched(c);
}

static void wdtimer_callback(sh2_t *c)
{
	c->wdt_event = SH2_NEVER;
	c->wtcnt = 0;
	if (!(c->wtcsr & 0x40)) {   // timer mode
		c->wtcsr |= 0x80;
		recalc_irq(c);
		wdt_activate(c);
	} else {                    // watchdog mode
		c->rstcsr |= 0x80;
		// TODO (MAME): reset and /WDTOVF out
	}
}

/***************************************************************************
    DMAC
***************************************************************************/

// one step of a channel (MAME sh2_do_dma, called every 2 cycles)
static void do_dma(sh2_t *c, int ch)
{
	if (c->dma_count[ch] > 0) {
		switch (c->dma_size[ch]) {
		case 0: {
			uint32_t tempsrc = c->dma_src[ch];
			if (c->dma_incs[ch] == 2) tempsrc--;
			uint32_t tempdst = c->dma_dst[ch];
			if (c->dma_incd[ch] == 2) tempdst--;
			{ uint32_t d_ = raw_r8(c, tempsrc); if (c->dma_kludge) d_ = c->dma_kludge(c->user, tempsrc, tempdst, d_, 0); raw_w8(c, tempdst, (uint8_t)d_); }
			if (c->dma_incs[ch] == 2) c->dma_src[ch]--;
			if (c->dma_incd[ch] == 2) c->dma_dst[ch]--;
			if (c->dma_incs[ch] == 1) c->dma_src[ch]++;
			if (c->dma_incd[ch] == 1) c->dma_dst[ch]++;
			c->dma_count[ch]--;
			break;
		}
		case 1: {
			uint32_t tempsrc = c->dma_src[ch];
			if (c->dma_incs[ch] == 2) tempsrc -= 2;
			uint32_t tempdst = c->dma_dst[ch];
			if (c->dma_incd[ch] == 2) tempdst -= 2;
			{ uint32_t d_ = raw_r16(c, tempsrc); if (c->dma_kludge) d_ = c->dma_kludge(c->user, tempsrc, tempdst, d_, 1); raw_w16(c, tempdst, (uint16_t)d_); }
			if (c->dma_incs[ch] == 2) c->dma_src[ch] -= 2;
			if (c->dma_incd[ch] == 2) c->dma_dst[ch] -= 2;
			if (c->dma_incs[ch] == 1) c->dma_src[ch] += 2;
			if (c->dma_incd[ch] == 1) c->dma_dst[ch] += 2;
			c->dma_count[ch]--;
			break;
		}
		case 2: {
			uint32_t tempsrc = c->dma_src[ch];
			if (c->dma_incs[ch] == 2) tempsrc -= 4;
			uint32_t tempdst = c->dma_dst[ch];
			if (c->dma_incd[ch] == 2) tempdst -= 4;
			{ uint32_t d_ = raw_r32(c, tempsrc); if (c->dma_kludge) d_ = c->dma_kludge(c->user, tempsrc, tempdst, d_, c->dma_size[ch]); raw_w32(c, tempdst, d_); }
			if (c->dma_incs[ch] == 2) c->dma_src[ch] -= 4;
			if (c->dma_incd[ch] == 2) c->dma_dst[ch] -= 4;
			if (c->dma_incs[ch] == 1) c->dma_src[ch] += 4;
			if (c->dma_incd[ch] == 1) c->dma_dst[ch] += 4;
			c->dma_count[ch]--;
			break;
		}
		case 3: {
			uint32_t tempsrc = c->dma_src[ch];
			uint32_t tempdst = c->dma_dst[ch];
			if (c->dma_incd[ch] == 2) tempdst -= 16;
			{ uint32_t d_ = raw_r32(c, tempsrc); if (c->dma_kludge) d_ = c->dma_kludge(c->user, tempsrc, tempdst, d_, c->dma_size[ch]); raw_w32(c, tempdst, d_); }
			{ uint32_t d_ = raw_r32(c, tempsrc + 4); if (c->dma_kludge) d_ = c->dma_kludge(c->user, tempsrc, tempdst, d_, 3); raw_w32(c, tempdst + 4, d_); }
			{ uint32_t d_ = raw_r32(c, tempsrc + 8); if (c->dma_kludge) d_ = c->dma_kludge(c->user, tempsrc, tempdst, d_, 3); raw_w32(c, tempdst + 8, d_); }
			{ uint32_t d_ = raw_r32(c, tempsrc + 12); if (c->dma_kludge) d_ = c->dma_kludge(c->user, tempsrc, tempdst, d_, 3); raw_w32(c, tempdst + 12, d_); }
			if (c->dma_incd[ch] == 2) c->dma_dst[ch] -= 16;
			c->dma_src[ch] += 16;
			if (c->dma_incd[ch] == 1) c->dma_dst[ch] += 16;
			c->dma_count[ch] -= 4;
			break;
		}
		}
	} else {   // the dma is complete
		c->dma_tick[ch] = SH2_NEVER;
		if (c->dma_steal[ch])
			c->halted = 0;
		c->dmac[ch].tcr = 0;
		c->dmac[ch].chcr |= 2;
		c->dma_active[ch] = 0;
		c->dma_irq[ch] |= 1;
		recalc_irq(c);
	}
}

// run the steps of the channels due by clock t
static void dma_catch_up(sh2_t *c, int64_t t)
{
	for (int ch = 0; ch < 2; ch++)
		while (c->dma_active[ch] == 1 && c->dma_tick[ch] <= t) {
			c->dma_tick[ch] += 2;
			do_dma(c, ch);
		}
}

static void dmac_check(sh2_t *c, int ch)
{
	if (c->dmac[ch].chcr & c->dmaor & 1) {
		if (!c->dma_active[ch] && !(c->dmac[ch].chcr & 2)) {
			c->dma_incd[ch] = (c->dmac[ch].chcr >> 14) & 3;
			c->dma_incs[ch] = (c->dmac[ch].chcr >> 12) & 3;
			c->dma_size[ch] = (c->dmac[ch].chcr >> 10) & 3;
			c->dma_steal[ch] = (c->dmac[ch].chcr & 0x10) ? 1 : 0;

			if (c->dma_incd[ch] == 3 || c->dma_incs[ch] == 3)
				return;   // bad increment values

			c->dma_src[ch] = c->dmac[ch].sar;
			c->dma_dst[ch] = c->dmac[ch].dar;
			c->dma_count[ch] = c->dmac[ch].tcr;
			if (!c->dma_count[ch])
				c->dma_count[ch] = 0x1000000;

			c->dma_active[ch] = 1;

			c->dma_src[ch] &= SH2_AM;
			c->dma_dst[ch] &= SH2_AM;

			switch (c->dma_size[ch]) {
			case 0: break;
			case 1: c->dma_src[ch] &= ~1u; c->dma_dst[ch] &= ~1u; break;
			case 2: c->dma_src[ch] &= ~3u; c->dma_dst[ch] &= ~3u; break;
			case 3: c->dma_src[ch] &= ~3u; c->dma_dst[ch] &= ~3u; c->dma_count[ch] &= ~3u; break;
			}

			// fever soccer uses cycle-stealing mode, requiring the CPU to be halted
			if (c->dma_steal[ch]) {
				c->halted = 1;
				if (c->running) {   // MAME suspend: the CPU stops after the current instruction
					int64_t t = now_cycles(c);
					c->slice_end = t;
					c->icount = 0;
				}
			}

			c->dma_tick[ch] = now_cycles(c) + 2;
			resched(c);
		}
	} else {
		if (c->dma_active[ch]) {   // cancelled in-flight
			c->dma_tick[ch] = SH2_NEVER;
			c->dma_active[ch] = 0;
			if (c->dma_steal[ch])
				c->halted = 0;
		}
	}
}

/***************************************************************************
    INTC
***************************************************************************/

static void recalc_irq(sh2_t *c)
{
	int irq = 0;
	int vector = -1;
	int level;

	// Timer irqs
	if (c->tier & c->ftcsr & (ICF | OCFA | OCFB | OVF)) {
		level = c->lvl_frc & 15;
		if (level > irq) {
			int mask = c->tier & c->ftcsr;
			irq = level;
			if (mask & ICF)
				vector = c->vec_fic & 0x7f;
			else if (mask & (OCFA | OCFB))
				vector = c->vec_foc & 0x7f;
			else
				vector = c->vec_fov & 0x7f;
		}
	}

	// WDT irqs
	if (c->wtcsr & 0x80) {
		level = c->lvl_wdt & 15;
		if (level > irq) {
			irq = level;
			vector = (c->vcrwdt >> 8) & 0x7f;
		}
	}

	// DMA irqs
	if ((c->dmac[0].chcr & 6) == 6 && c->dma_irq[0]) {
		level = c->lvl_dmac & 15;
		if (level > irq) {
			irq = level;
			c->dma_irq[0] &= ~1;
			vector = c->vec_dmac[0] & 0x7f;
		}
	} else if ((c->dmac[1].chcr & 6) == 6 && c->dma_irq[1]) {
		level = c->lvl_dmac & 15;
		if (level > irq) {
			irq = level;
			c->dma_irq[1] &= ~1;
			vector = c->vec_dmac[1] & 0x7f;
		}
	}

	c->internal_irq_level = irq;
	c->internal_irq_vector = vector;
	// MAME sets sh7604_device::m_test_irq, which shadows the m_test_irq the interpreter reads (so MAME's interpreter
	// misses it until the next LDC SR / RTE / line change); here the interrupt is checked after this instruction
	c->test_irq = 1;
}

/***************************************************************************
    on-chip registers (sh7604_device::sh7604_map), 0xfffffe00-0xffffffff
    a = dword address; mask / data in big-endian lanes (bits 31-24 = the byte at a)
***************************************************************************/

#define COMBINE(reg, d, m) ((reg) = ((reg) & ~(m)) | ((d) & (m)))
#define L8(k)  (((mask) >> (24 - 8 * (k))) & 0xffu)          // lane k (0-3) selected
#define H16(k) (((mask) >> (16 - 16 * (k))) & 0xffffu)       // half k (0-1) selected
#define D8(k)  ((uint8_t)(data >> (24 - 8 * (k))))
#define D16(k) ((uint16_t)(data >> (16 - 16 * (k))))
#define M16(k) ((uint16_t)(mask >> (16 - 16 * (k))))
#define R8(k, v)  (r |= (uint32_t)(uint8_t)(v) << (24 - 8 * (k)))
#define R16(k, v) (r |= (uint32_t)(uint16_t)(v) << (16 - 16 * (k)))

static uint32_t onchip_r(sh2_t *c, uint32_t a, uint32_t mask)
{
	uint32_t r = 0;
	c->onchip_n++;
	switch (a & 0x1fc) {
	case 0x000:   // SCI
		if (L8(0)) R8(0, c->smr);
		if (L8(1)) R8(1, c->brr);
		if (L8(2)) R8(2, c->scr);
		if (L8(3)) R8(3, c->tdr);
		break;
	case 0x004:
		if (L8(0)) R8(0, c->ssr | 0x84);   // 0x84 is needed by EGWord on Saturn (MAME)
		break;                              // RDR reads 0
	case 0x010:   // FRT
		if (L8(0)) R8(0, c->tier);
		if (L8(1)) R8(1, c->ftcsr);
		if (H16(1)) { timer_resync(c); R16(1, c->frc); }
		break;
	case 0x014:
		if (H16(0)) R16(0, (c->tocr & 0x10) ? c->ocrb : c->ocra);
		if (L8(2)) R8(2, c->frc_tcr & 0x83);
		if (L8(3)) R8(3, (c->tocr & 0x13) | 0xe0);
		break;
	case 0x018:
		if (H16(0)) R16(0, c->frc_icr);
		break;
	case 0x060:   // INTC
		if (H16(0)) R16(0, c->iprb & 0xff00);
		if (H16(1)) R16(1, c->vcra & 0x7f7f);
		break;
	case 0x064:
		if (H16(0)) R16(0, c->vcrb & 0x7f7f);
		if (H16(1)) R16(1, c->vcrc & 0x7f7f);
		break;
	case 0x068:
		if (H16(0)) R16(0, c->vcrd & 0x7f00);
		break;
	case 0x070:
		if (L8(1)) R8(1, c->dmac[0].drcr & 3);
		if (L8(2)) R8(2, c->dmac[1].drcr & 3);
		break;
	case 0x080:   // WDT
		if (H16(0)) { wtcnt_recalc(c); R16(0, ((c->wtcsr | 0x18) << 8) | (c->wtcnt & 0xff)); }
		if (H16(1)) R16(1, (c->rstcsr & 0xe0) | 0x1f);
		break;
	case 0x090:
		if (H16(0)) R16(0, c->sbycr);
		if (L8(2)) R8(2, c->ccr & ~0x30);
		break;
	case 0x0e0:
		// TODO (MAME): flip meaning based off NMI edge select bit (NMIE)
		if (H16(0)) R16(0, (c->nmi_line_state == SH2_ASSERT ? 0 : 0x8000) | (c->intc_icr & 0x0101));
		if (H16(1)) R16(1, c->ipra & 0xfff0);
		break;
	case 0x0e4:
		if (H16(0)) R16(0, c->vcrwdt & 0x7f7f);
		break;
	case 0x100: r = c->dvsr; break;                          // DIVU
	case 0x104: r = c->dvdntl; break;
	case 0x108: r = ((uint32_t)c->divu_ovfie << 1) | c->divu_ovf; break;
	case 0x10c: r = c->vcrdiv & 0xffff; break;
	case 0x110: case 0x118: r = c->dvdnth; break;
	case 0x114: case 0x11c: r = c->dvdntl; break;
	case 0x140:   // UBC
		if (H16(0)) R16(0, c->barah);
		if (H16(1)) R16(1, c->baral);
		break;
	case 0x160:
		if (H16(0)) R16(0, c->barbh);
		if (H16(1)) R16(1, c->barbl);
		break;
	case 0x180: case 0x190: r = c->dmac[(a >> 4) & 1].sar; break;   // DMAC
	case 0x184: case 0x194: r = c->dmac[(a >> 4) & 1].dar; break;
	case 0x188: case 0x198: dma_catch_up(c, now_cycles(c)); r = c->dmac[(a >> 4) & 1].tcr; break;
	case 0x18c: case 0x19c: dma_catch_up(c, now_cycles(c)); r = c->dmac[(a >> 4) & 1].chcr; break;
	case 0x1a0: r = c->vcrdma[0] & 0x7f; break;
	case 0x1a8: r = c->vcrdma[1] & 0x7f; break;
	case 0x1b0: r = c->dmaor & 0xf; break;
	case 0x1e0: r = (c->bcr1 & ~0xe008u) | (c->is_slave ? 0x8000 : 0); break;   // BSC
	case 0x1e4: r = c->bcr2; break;
	case 0x1e8: r = c->wcr; break;
	case 0x1ec: r = c->mcr & ~0x103u; break;
	case 0x1f0: r = c->rtcsr & 0xf8; break;
	case 0x1f4: r = c->rtcnt & 0xff; break;
	case 0x1f8: r = c->rtcor & 0xff; break;
	default: break;
	}
	return r & mask;
}

static void dvdnt_w(sh2_t *c, uint32_t data, uint32_t mask)
{
	// TODO (MAME): this is really a separate register that happens to be shared with DVDNTL
	COMBINE(c->dvdntl, data, mask);
	int32_t a = (int32_t)c->dvdntl;
	int32_t b = (int32_t)c->dvsr;
	if (b) {
		if (a == INT32_MIN && b == -1) {   // the quotient the host cannot compute (SH-2 / ARM give 0x80000000, 0)
			c->dvdntl = 0x80000000u;
			c->dvdnth = 0;
		} else {
			c->dvdntl = (uint32_t)(a / b);
			c->dvdnth = (uint32_t)(a % b);
		}
		// TODO (MAME): 40 cycles
	} else {
		c->divu_ovf = 1;
		c->dvdntl = 0x7fffffff;
		c->dvdnth = 0x7fffffff;
		recalc_irq(c);
		// TODO (MAME): 8 cycles
	}
}

static void dvdntl_w(sh2_t *c, uint32_t data, uint32_t mask)
{
	COMBINE(c->dvdntl, data, mask);
	int64_t a = (int64_t)(c->dvdntl | ((uint64_t)c->dvdnth << 32));
	int64_t b = (int32_t)c->dvsr;
	if (b && !(a == INT64_MIN && b == -1)) {
		int64_t q = a / b;
		if (q != (int32_t)q) {
			c->divu_ovf = 1;
			c->dvdntl = 0x7fffffff;
			c->dvdnth = 0x7fffffff;
			recalc_irq(c);
			// TODO (MAME): 6 cycles
		} else {
			c->dvdntl = (uint32_t)q;
			c->dvdnth = (uint32_t)(a % b);
			// TODO (MAME): 39 cycles
		}
	} else {   // b == 0 (or INT64_MIN / -1, an overflow as well)
		c->divu_ovf = 1;
		c->dvdntl = 0x7fffffff;
		c->dvdnth = 0x7fffffff;
		recalc_irq(c);
		// TODO (MAME): 6 cycles
	}
}

static void wtcnt_w(sh2_t *c, uint16_t data, uint16_t mask)
{
	COMBINE(c->wtcw[0], data, mask);
	switch (c->wtcw[0] & 0xff00) {
	case 0x5a00:
		c->wtcnt = c->wtcw[0] & 0xff;
		if (c->wtcsr & 0x20)
			wdt_activate(c);
		break;
	case 0xa500:
		wtcnt_recalc(c);
		c->wtcsr &= c->wtcw[0] & 0x80;
		c->wtcsr |= c->wtcw[0] & 0x7f;
		if (c->wtcsr & 0x20)
			wdt_activate(c);
		else {
			c->wtcnt = 0;
			c->wdt_event = SH2_NEVER;
		}
		recalc_irq(c);
		break;
	}
}

static void rstcsr_w(sh2_t *c, uint16_t data, uint16_t mask)
{
	COMBINE(c->wtcw[1], data, mask);
	switch (c->wtcw[1] & 0xff00) {
	case 0xa500:   // clear WOVF flag
		if ((c->wtcw[1] & 0x80) == 0)
			c->rstcsr &= 0x7f;
		break;
	case 0x5a00:
		c->rstcsr = (c->rstcsr & 0x80) | (c->wtcw[1] & 0x60);
		break;
	}
}

static void chcr_w(sh2_t *c, int ch, uint32_t data, uint32_t mask)
{
	uint32_t old = c->dmac[ch].chcr;
	COMBINE(c->dmac[ch].chcr, data, mask);
	c->dmac[ch].chcr = (data & ~2u) | (old & c->dmac[ch].chcr & 2);
	dmac_check(c, ch);
}

static void onchip_w(sh2_t *c, uint32_t a, uint32_t data, uint32_t mask)
{
	c->onchip_n++;
	switch (a & 0x1fc) {
	case 0x000:   // SCI
		if (L8(0)) c->smr = D8(0);
		if (L8(1)) c->brr = D8(1);
		if (L8(2)) c->scr = D8(2);
		if (L8(3)) c->tdr = D8(3);
		break;
	case 0x004:
		if (L8(0)) c->ssr = D8(0);
		break;
	case 0x010:   // FRT
		if (L8(0)) {   // TIER
			timer_resync(c);
			c->tier = D8(0);
			timer_activate(c);
			recalc_irq(c);
		}
		if (L8(1)) {   // FTCSR
			uint8_t old = c->ftcsr;
			c->ftcsr = D8(1);
			timer_resync(c);
			c->ftcsr = (c->ftcsr & ~(ICF | OCFA | OCFB | OVF)) | (old & c->ftcsr & (ICF | OCFA | OCFB | OVF));
			timer_activate(c);
			recalc_irq(c);
		}
		if (H16(1)) {  // FRC
			timer_resync(c);
			COMBINE(c->frc, D16(1), M16(1));
			timer_activate(c);
			recalc_irq(c);
		}
		break;
	case 0x014:
		if (H16(0)) {  // OCRA / OCRB
			timer_resync(c);
			if (c->tocr & 0x10)
				c->ocrb = (c->ocrb & ~M16(0)) | (D16(0) & M16(0));
			else
				c->ocra = (c->ocra & ~M16(0)) | (D16(0) & M16(0));
			timer_activate(c);
			recalc_irq(c);
		}
		if (L8(2)) {   // TCR
			timer_resync(c);
			c->frc_tcr = D8(2) & 0x83;
			timer_activate(c);
			recalc_irq(c);
		}
		if (L8(3)) {   // TOCR
			timer_resync(c);
			// TODO (MAME): output levels A/B (bits 1-0)
			c->tocr = D8(3) & 0x13;
			timer_activate(c);
			recalc_irq(c);
		}
		break;
	case 0x060:   // INTC
		if (H16(0)) {
			COMBINE(c->iprb, D16(0), M16(0));
			c->lvl_sci = (c->iprb >> 12) & 0xf;
			c->lvl_frc = (c->iprb >> 8) & 0xf;
			recalc_irq(c);
		}
		if (H16(1)) { COMBINE(c->vcra, D16(1), M16(1)); recalc_irq(c); }
		break;
	case 0x064:
		if (H16(0)) { COMBINE(c->vcrb, D16(0), M16(0)); recalc_irq(c); }
		if (H16(1)) {
			COMBINE(c->vcrc, D16(1), M16(1));
			c->vec_fic = (c->vcrc >> 8) & 0x7f;
			c->vec_foc = (c->vcrc >> 0) & 0x7f;
			recalc_irq(c);
		}
		break;
	case 0x068:
		if (H16(0)) {
			COMBINE(c->vcrd, D16(0), M16(0));
			c->vec_fov = (c->vcrd >> 8) & 0x7f;
			recalc_irq(c);
		}
		break;
	case 0x070:
		if (L8(1)) { c->dmac[0].drcr = D8(1) & 3; recalc_irq(c); }
		if (L8(2)) { c->dmac[1].drcr = D8(2) & 3; recalc_irq(c); }
		break;
	case 0x080:   // WDT
		if (H16(0)) wtcnt_w(c, D16(0), M16(0));
		if (H16(1)) rstcsr_w(c, D16(1), M16(1));
		break;
	case 0x090:
		if (H16(0)) {   // FMR / SBYCR: the clock multiplier is only logged by MAME
			if (M16(0) == 0x00ff)
				c->sbycr = (uint8_t)D16(0);
		}
		if (L8(2)) c->ccr = D8(2);
		break;
	case 0x0e0:
		if (H16(0)) {
			COMBINE(c->intc_icr, D16(0), M16(0));
			c->nmie = (c->intc_icr >> 8) & 1;
			c->vecmd = c->intc_icr & 1;
		}
		if (H16(1)) {
			COMBINE(c->ipra, D16(1), M16(1));
			c->lvl_divu = (c->ipra >> 12) & 0xf;
			c->lvl_dmac = (c->ipra >> 8) & 0xf;
			c->lvl_wdt = (c->ipra >> 4) & 0xf;
			recalc_irq(c);
		}
		break;
	case 0x0e4:
		if (H16(0)) { COMBINE(c->vcrwdt, D16(0), M16(0)); recalc_irq(c); }
		break;
	case 0x100: COMBINE(c->dvsr, data, mask); break;   // DIVU
	case 0x104: dvdnt_w(c, data, mask); break;
	case 0x108:
		if (mask & 0xff) {
			// both bits are regular r/w (vblokbrk/sarukani writes a '0' to clear a divide by zero OVF)
			c->divu_ovf = data & 1;
			c->divu_ovfie = (data >> 1) & 1;
			recalc_irq(c);
		}
		break;
	case 0x10c:
		COMBINE(c->vcrdiv, data, mask);
		c->vec_divu = c->vcrdiv & 0x7f;
		recalc_irq(c);
		break;
	case 0x110: COMBINE(c->dvdnth, data, mask); break;
	case 0x114: dvdntl_w(c, data, mask); break;
	case 0x140:   // UBC
		if (H16(0)) COMBINE(c->barah, D16(0), M16(0));
		if (H16(1)) COMBINE(c->baral, D16(1), M16(1));
		break;
	case 0x160:
		if (H16(0)) COMBINE(c->barbh, D16(0), M16(0));
		if (H16(1)) COMBINE(c->barbl, D16(1), M16(1));
		break;
	case 0x180: case 0x190: dma_catch_up(c, now_cycles(c)); COMBINE(c->dmac[(a >> 4) & 1].sar, data, mask); break;
	case 0x184: case 0x194: dma_catch_up(c, now_cycles(c)); COMBINE(c->dmac[(a >> 4) & 1].dar, data, mask); break;
	case 0x188: case 0x198: {
		int ch = (a >> 4) & 1;
		dma_catch_up(c, now_cycles(c));
		COMBINE(c->dmac[ch].tcr, data, mask);
		c->dmac[ch].tcr &= 0xffffff;
		break;
	}
	case 0x18c: case 0x19c: dma_catch_up(c, now_cycles(c)); chcr_w(c, (a >> 4) & 1, data, mask); break;
	case 0x1a0: COMBINE(c->vcrdma[0], data, mask); c->vec_dmac[0] = c->vcrdma[0] & 0x7f; recalc_irq(c); break;
	case 0x1a8: COMBINE(c->vcrdma[1], data, mask); c->vec_dmac[1] = c->vcrdma[1] & 0x7f; recalc_irq(c); break;
	case 0x1b0:
		if (mask & 0xff) {
			dma_catch_up(c, now_cycles(c));
			uint8_t old = c->dmaor & 0xf;
			c->dmaor = (uint8_t)((data & ~6u) | (old & data & 6));
			dmac_check(c, 0);
			dmac_check(c, 1);
		}
		break;
	// BCR1 / BCR2 are 16-bit; a dword write needs 0xa55a in the upper half. (MAME's ACCESSING_BITS_0_31 is true
	// for any access, so narrower writes without the key are dropped too.)
	case 0x1e0:
		if ((data & 0xffff0000) == 0xa55a0000) { COMBINE(c->bcr1, data, mask); c->bcr1 &= 0xffff; }
		break;
	case 0x1e4:
		if ((data & 0xffff0000) == 0xa55a0000) { COMBINE(c->bcr2, data, mask); c->bcr2 &= 0xffff; }
		break;
	case 0x1e8: COMBINE(c->wcr, data, mask); break;
	case 0x1ec: COMBINE(c->mcr, data, mask); break;
	case 0x1f0: COMBINE(c->rtcsr, data, mask); break;
	case 0x1f4: COMBINE(c->rtcnt, data, mask); c->rtcnt &= 0xff; break;
	case 0x1f8: COMBINE(c->rtcor, data, mask); c->rtcor &= 0xff; break;
	default: break;
	}
}

/***************************************************************************
    instructions (sh_common_execution, sh2_device)
***************************************************************************/

#define N  ((op >> 8) & 15)
#define M  ((op >> 4) & 15)
#define RN c->r[N]
#define RM c->r[M]
#define SETT_IF(cond) do { if (cond) c->sr |= SH_T; else c->sr &= ~SH_T; } while (0)
#define SEXT8(x)  ((uint32_t)(int32_t)(int8_t)(x))
#define SEXT12(x) ((uint32_t)(((int32_t)((x) << 20)) >> 20))
#define SEXT16(x) ((uint32_t)(int32_t)(int16_t)(x))

SH2_NOINLINE void op_illegal(sh2_t *c)
{
	c->r[15] -= 4;
	WL(c, c->r[15], c->sr);          // push SR onto stack
	c->r[15] -= 4;
	WL(c, c->r[15], c->pc - 2);      // push PC onto stack
	c->pc = RL(c, c->vbr + 4 * 4);   // fetch PC
	c->icount -= 5;                  // TODO (MAME): timing is a guess
}

SH2_NOINLINE void op_trapa(sh2_t *c, uint32_t i)
{
	uint32_t imm = i & 0xff;
	uint32_t ea = c->vbr + imm * 4;
	c->r[15] -= 4;
	WL(c, c->r[15], c->sr);
	c->r[15] -= 4;
	WL(c, c->r[15], c->pc);
	c->pc = RL(c, ea);
	c->icount -= 7;
}

SH2_NOINLINE void op_div1(sh2_t *c, uint32_t m, uint32_t n)
{
	uint32_t old_q = c->sr & SH_Q;
	if (0x80000000 & c->r[n])
		c->sr |= SH_Q;
	else
		c->sr &= ~SH_Q;

	c->r[n] = (c->r[n] << 1) | (c->sr & SH_T);

	uint32_t tmp = c->r[n];
	if (!old_q) {
		if (!(c->sr & SH_M)) {
			c->r[n] -= c->r[m];
			if (!(c->sr & SH_Q)) { if (c->r[n] > tmp) c->sr |= SH_Q; else c->sr &= ~SH_Q; }
			else { if (c->r[n] > tmp) c->sr &= ~SH_Q; else c->sr |= SH_Q; }
		} else {
			c->r[n] += c->r[m];
			if (!(c->sr & SH_Q)) { if (c->r[n] < tmp) c->sr &= ~SH_Q; else c->sr |= SH_Q; }
			else { if (c->r[n] < tmp) c->sr |= SH_Q; else c->sr &= ~SH_Q; }
		}
	} else {
		if (!(c->sr & SH_M)) {
			c->r[n] += c->r[m];
			if (!(c->sr & SH_Q)) { if (c->r[n] < tmp) c->sr |= SH_Q; else c->sr &= ~SH_Q; }
			else { if (c->r[n] < tmp) c->sr &= ~SH_Q; else c->sr |= SH_Q; }
		} else {
			c->r[n] -= c->r[m];
			if (!(c->sr & SH_Q)) { if (c->r[n] > tmp) c->sr &= ~SH_Q; else c->sr |= SH_Q; }
			else { if (c->r[n] > tmp) c->sr |= SH_Q; else c->sr &= ~SH_Q; }
		}
	}

	uint32_t qm = c->sr & (SH_Q | SH_M);
	if (qm == 0 || qm == 0x300)   // if Q == M set T else clear T
		c->sr |= SH_T;
	else
		c->sr &= ~SH_T;
}

// 32x32 -> 64 multiply as MAME does it (DMULS / DMULU / MAC.L share it)
SH2_INLINE void mul64(uint32_t a, uint32_t b, uint32_t *hi, uint32_t *lo)
{
	uint64_t p = (uint64_t)a * b;
	*hi = (uint32_t)(p >> 32);
	*lo = (uint32_t)p;
}

SH2_NOINLINE void op_mac_l(sh2_t *c, uint32_t m, uint32_t n)
{
	int32_t tempn = (int32_t)RL(c, c->r[n]);
	c->r[n] += 4;
	int32_t tempm = (int32_t)RL(c, c->r[m]);
	c->r[m] += 4;

	// MAME's 16x16 partial products give the exact signed 64-bit product
	int64_t p = (int64_t)tempn * tempm;
	uint32_t res2 = (uint32_t)((uint64_t)p >> 32);
	uint32_t res0 = (uint32_t)p;

	if (c->sr & SH_S) {
		res0 = c->macl + res0;
		if (c->macl > res0)
			res2++;
		res2 += c->mach & 0x0000ffff;
		if ((int32_t)res2 < 0 && res2 < 0xffff8000) {
			res2 = 0x00008000;
			res0 = 0x00000000;
		} else if ((int32_t)res2 > 0 && res2 > 0x00007fff) {
			res2 = 0x00007fff;
			res0 = 0xffffffff;
		}
		c->mach = res2;
		c->macl = res0;
	} else {
		res0 = c->macl + res0;
		if (c->macl > res0)
			res2++;
		res2 += c->mach;
		c->mach = res2;
		c->macl = res0;
	}
	c->icount -= 2;
}

SH2_NOINLINE void op_mac_w(sh2_t *c, uint32_t m, uint32_t n)
{
	int32_t tempn = (int32_t)(int16_t)RW(c, c->r[n]);
	c->r[n] += 2;
	int32_t tempm = (int32_t)(int16_t)RW(c, c->r[m]);
	c->r[m] += 2;

	uint32_t templ = c->macl;
	tempm = (int32_t)((uint32_t)tempm * (uint32_t)tempn);

	int32_t dest = (c->macl >> 31) & 1;
	int32_t src = (((uint32_t)tempm >> 31) & 1) + dest;
	tempn = ((uint32_t)tempm >> 31) ? -1 : 0;

	c->macl += (uint32_t)tempm;

	int32_t ans = ((c->macl >> 31) & 1) + dest;

	if (c->sr & SH_S) {
		if (ans == 1) {
			if (src == 0)
				c->macl = 0x7fffffff;
			else if (src == 2)
				c->macl = 0x80000000;
		}
	} else {
		c->mach += (uint32_t)tempn;
		if (templ > c->macl)
			c->mach += 1;
	}
	c->icount -= 2;
}

// LDC.L @Rm+,SR / LDC Rm,SR / RTE (sh2_device)
SH2_NOINLINE void op_rte(sh2_t *c)
{
	c->delay = RL(c, c->r[15]);
	c->r[15] += 4;
	c->sr = RL(c, c->r[15]) & SH_FLAGS;
	c->r[15] += 4;
	c->icount -= 3;
	c->test_irq = 1;
}

SH2_NOINLINE void op_sleep(sh2_t *c)
{
	// 0 = normal mode, 1 = enters into power-down mode, 2 = go out the power-down mode after an exception
	if (c->sleep_mode == 1 && !c->test_irq && c->icount > 3) {
		// ReBOX: nothing can wake the CPU before the slice ends (the events end it): run the remaining SLEEP
		// rounds (3 cycles each) at once, leaving this one to finish below
		int32_t rounds = (c->icount + 2) / 3;
		c->icount -= 3 * (rounds - 1);
	}
	if (c->sleep_mode != 2)
		c->pc -= 2;
	c->icount -= 2;
	if (c->sleep_mode == 0)
		c->sleep_mode = 1;
	else if (c->sleep_mode == 2)
		c->sleep_mode = 0;
}

// CPU accesses that miss the page table (sh2_device::read_byte ... without a page)
SH2_NOINLINE uint8_t cpu_slow_r8(sh2_t *c, uint32_t a)
{
	return CPU_FAST(a) ? c->read8(c->user, a & 0x07ffffff) : raw_slow_r8(c, a);
}
SH2_NOINLINE uint16_t cpu_slow_r16(sh2_t *c, uint32_t a)
{
	return CPU_FAST(a) ? c->read16(c->user, a & 0x07fffffe) : raw_slow_r16(c, a);
}
SH2_NOINLINE uint32_t cpu_slow_r32(sh2_t *c, uint32_t a)
{
	return CPU_FAST(a) ? c->read32(c->user, a & 0x07fffffc) : raw_slow_r32(c, a);
}
SH2_NOINLINE void cpu_slow_w8(sh2_t *c, uint32_t a, uint32_t v)
{
	if (CPU_FAST(a)) c->write8(c->user, a & 0x07ffffff, (uint8_t)v); else raw_slow_w8(c, a, (uint8_t)v);
}
SH2_NOINLINE void cpu_slow_w16(sh2_t *c, uint32_t a, uint32_t v)
{
	if (CPU_FAST(a)) c->write16(c->user, a & 0x07fffffe, (uint16_t)v); else raw_slow_w16(c, a, (uint16_t)v);
}
SH2_NOINLINE void cpu_slow_w32(sh2_t *c, uint32_t a, uint32_t v)
{
	if (CPU_FAST(a)) c->write32(c->user, a & 0x07fffffc, v); else raw_slow_w32(c, a, v);
}

// threaded dispatch: every opcode maps (s_disp, built once) to one handler of this list; each handler ends with its
// own copy of the fetch and the indirect jump to the next one
#define SH2_OPLIST(X) \
	X(ILLEGAL) X(STC_SR) X(STC_GBR) X(STC_VBR) X(BSRF) X(BRAF) X(MOVB_S0) X(MOVW_S0) X(MOVL_S0) X(MULL) \
	X(CLRT) X(SETT) X(CLRMAC) X(NOP) X(DIV0U) X(MOVT) X(STS_MACH) X(STS_MACL) X(STS_PR) X(RTS) X(SLEEP) X(RTE) \
	X(MOVB_L0) X(MOVW_L0) X(MOVL_L0) X(MACL) X(MOVL_S4) \
	X(MOVB_S) X(MOVW_S) X(MOVL_S) X(MOVB_M) X(MOVW_M) X(MOVL_M) X(DIV0S) X(TST) X(AND) X(XOR) X(OR) X(CMPSTR) \
	X(XTRCT) X(MULU) X(MULS) \
	X(CMPEQ) X(CMPHS) X(CMPGE) X(DIV1) X(DMULU) X(CMPHI) X(CMPGT) X(SUB) X(SUBC) X(SUBV) X(ADD) X(DMULS) X(ADDC) \
	X(ADDV) \
	X(SHLL) X(SHLR) X(SHAR) X(STSM_MACH) X(STSM_MACL) X(STSM_PR) X(STCM_SR) X(STCM_GBR) X(STCM_VBR) X(ROTL) \
	X(ROTR) X(ROTCL) X(ROTCR) X(LDSM_MACH) X(LDSM_MACL) X(LDSM_PR) X(LDCM_SR) X(LDCM_GBR) X(LDCM_VBR) X(SHLL2) \
	X(SHLL8) X(SHLL16) X(SHLR2) X(SHLR8) X(SHLR16) X(LDS_MACH) X(LDS_MACL) X(LDS_PR) X(JSR) X(JMP) X(TAS) \
	X(LDC_SR) X(LDC_GBR) X(LDC_VBR) X(MACW) X(DT) X(CMPPZ) X(CMPPL) \
	X(MOVL_L4) \
	X(MOVB_L) X(MOVW_L) X(MOVL_L) X(MOV) X(MOVB_P) X(MOVW_P) X(MOVL_P) X(NOT) X(SWAPB) X(SWAPW) X(NEGC) X(NEG) \
	X(EXTUB) X(EXTUW) X(EXTSB) X(EXTSW) \
	X(ADDI) \
	X(MOVB_S4R0) X(MOVW_S4R0) X(MOVB_L4R0) X(MOVW_L4R0) X(CMPIM) X(BT) X(BF) X(BTS) X(BFS) \
	X(MOVW_I) X(BRA) X(BSR) \
	X(MOVB_SG) X(MOVW_SG) X(MOVL_SG) X(TRAPA) X(MOVB_LG) X(MOVW_LG) X(MOVL_LG) X(MOVA) X(TSTI) X(ANDI) X(XORI) \
	X(ORI) X(TSTB) X(ANDB) X(XORB) X(ORB) \
	X(MOVL_I) X(MOVI)

#define SH2_ENUM(n) H_##n,
enum { SH2_OPLIST(SH2_ENUM) H_COUNT };

// the opcode -> handler table and the handlers' addresses (copied by the first run) share one base
static struct { uint8_t tab[65536]; const void *lab[H_COUNT]; } s_disp;
static int s_disp_ok;

// the decoder of the old switch interpreter (MAME sh_common_execution::execute_one), one opcode -> its handler
static int decode(uint32_t op)
{
	switch (op >> 12) {
	case 0x0:
		switch (op & 0x3f) {
		case 0x02: return H_STC_SR;
		case 0x12: return H_STC_GBR;
		case 0x22: return H_STC_VBR;
		case 0x03: return H_BSRF;
		case 0x23: return H_BRAF;
		case 0x04: case 0x14: case 0x24: case 0x34: return H_MOVB_S0;
		case 0x05: case 0x15: case 0x25: case 0x35: return H_MOVW_S0;
		case 0x06: case 0x16: case 0x26: case 0x36: return H_MOVL_S0;
		case 0x07: case 0x17: case 0x27: case 0x37: return H_MULL;
		case 0x08: return H_CLRT;
		case 0x18: return H_SETT;
		case 0x28: return H_CLRMAC;
		case 0x09: return H_NOP;
		case 0x19: return H_DIV0U;
		case 0x29: return H_MOVT;
		case 0x0a: return H_STS_MACH;
		case 0x1a: return H_STS_MACL;
		case 0x2a: return H_STS_PR;
		case 0x0b: return H_RTS;
		case 0x1b: return H_SLEEP;
		case 0x2b: return H_RTE;
		case 0x0c: case 0x1c: case 0x2c: case 0x3c: return H_MOVB_L0;
		case 0x0d: case 0x1d: case 0x2d: case 0x3d: return H_MOVW_L0;
		case 0x0e: case 0x1e: case 0x2e: case 0x3e: return H_MOVL_L0;
		case 0x0f: case 0x1f: case 0x2f: case 0x3f: return H_MACL;
		default: return H_ILLEGAL;
		}
	case 0x1: return H_MOVL_S4;
	case 0x2:
		switch (op & 15) {
		case 0: return H_MOVB_S;
		case 1: return H_MOVW_S;
		case 2: return H_MOVL_S;
		case 4: return H_MOVB_M;
		case 5: return H_MOVW_M;
		case 6: return H_MOVL_M;
		case 7: return H_DIV0S;
		case 8: return H_TST;
		case 9: return H_AND;
		case 10: return H_XOR;
		case 11: return H_OR;
		case 12: return H_CMPSTR;
		case 13: return H_XTRCT;
		case 14: return H_MULU;
		case 15: return H_MULS;
		default: return H_ILLEGAL;
		}
	case 0x3:
		switch (op & 15) {
		case 0: return H_CMPEQ;
		case 2: return H_CMPHS;
		case 3: return H_CMPGE;
		case 4: return H_DIV1;
		case 5: return H_DMULU;
		case 6: return H_CMPHI;
		case 7: return H_CMPGT;
		case 8: return H_SUB;
		case 10: return H_SUBC;
		case 11: return H_SUBV;
		case 12: return H_ADD;
		case 13: return H_DMULS;
		case 14: return H_ADDC;
		case 15: return H_ADDV;
		default: return H_ILLEGAL;
		}
	case 0x4:
		switch (op & 0x3f) {
		case 0x00: case 0x20: return H_SHLL;
		case 0x01: return H_SHLR;
		case 0x21: return H_SHAR;
		case 0x02: return H_STSM_MACH;
		case 0x12: return H_STSM_MACL;
		case 0x22: return H_STSM_PR;
		case 0x03: return H_STCM_SR;
		case 0x13: return H_STCM_GBR;
		case 0x23: return H_STCM_VBR;
		case 0x04: return H_ROTL;
		case 0x05: return H_ROTR;
		case 0x24: return H_ROTCL;
		case 0x25: return H_ROTCR;
		case 0x06: return H_LDSM_MACH;
		case 0x16: return H_LDSM_MACL;
		case 0x26: return H_LDSM_PR;
		case 0x07: return H_LDCM_SR;
		case 0x17: return H_LDCM_GBR;
		case 0x27: return H_LDCM_VBR;
		case 0x08: return H_SHLL2;
		case 0x18: return H_SHLL8;
		case 0x28: return H_SHLL16;
		case 0x09: return H_SHLR2;
		case 0x19: return H_SHLR8;
		case 0x29: return H_SHLR16;
		case 0x0a: return H_LDS_MACH;
		case 0x1a: return H_LDS_MACL;
		case 0x2a: return H_LDS_PR;
		case 0x0b: return H_JSR;
		case 0x2b: return H_JMP;
		case 0x1b: return H_TAS;
		case 0x0e: return H_LDC_SR;
		case 0x1e: return H_LDC_GBR;
		case 0x2e: return H_LDC_VBR;
		case 0x0f: case 0x1f: case 0x2f: case 0x3f: return H_MACW;
		case 0x10: return H_DT;
		case 0x11: return H_CMPPZ;
		case 0x15: return H_CMPPL;
		default: return H_ILLEGAL;
		}
	case 0x5: return H_MOVL_L4;
	case 0x6:
		switch (op & 15) {
		case 0: return H_MOVB_L;
		case 1: return H_MOVW_L;
		case 2: return H_MOVL_L;
		case 3: return H_MOV;
		case 4: return H_MOVB_P;
		case 5: return H_MOVW_P;
		case 6: return H_MOVL_P;
		case 7: return H_NOT;
		case 8: return H_SWAPB;
		case 9: return H_SWAPW;
		case 10: return H_NEGC;
		case 11: return H_NEG;
		case 12: return H_EXTUB;
		case 13: return H_EXTUW;
		case 14: return H_EXTSB;
		default: return H_EXTSW;
		}
	case 0x7: return H_ADDI;
	case 0x8:
		switch ((op >> 8) & 15) {
		case 0: return H_MOVB_S4R0;
		case 1: return H_MOVW_S4R0;
		case 4: return H_MOVB_L4R0;
		case 5: return H_MOVW_L4R0;
		case 8: return H_CMPIM;
		case 9: return H_BT;
		case 11: return H_BF;
		case 13: return H_BTS;
		case 15: return H_BFS;
		default: return H_ILLEGAL;
		}
	case 0x9: return H_MOVW_I;
	case 0xa: return H_BRA;
	case 0xb: return H_BSR;
	case 0xc:
		switch ((op >> 8) & 15) {
		case 0: return H_MOVB_SG;
		case 1: return H_MOVW_SG;
		case 2: return H_MOVL_SG;
		case 3: return H_TRAPA;
		case 4: return H_MOVB_LG;
		case 5: return H_MOVW_LG;
		case 6: return H_MOVL_LG;
		case 7: return H_MOVA;
		case 8: return H_TSTI;
		case 9: return H_ANDI;
		case 10: return H_XORI;
		case 11: return H_ORI;
		case 12: return H_TSTB;
		case 13: return H_ANDB;
		case 14: return H_XORB;
		default: return H_ORB;
		}
	case 0xd: return H_MOVL_I;
	case 0xe: return H_MOVI;
	default: return H_ILLEGAL;
	}
}

static void build_disp(const void *const *lab)
{
	for (uint32_t op = 0; op < 65536; op++)
		s_disp.tab[op] = (uint8_t)decode(op);
	for (int i = 0; i < H_COUNT; i++)
		s_disp.lab[i] = lab[i];
	s_disp_ok = 1;
}

// the interpreter loop keeps pc, delay, icount, the fetch page and a copy of test_irq (tirq) in locals; they are
// written back (SYNC) before anything that can look at or change them (handlers, exceptions, the events) and read
// again after (RELOAD). c->test_irq stays the real flag: the loop writes both, RELOAD refreshes the copy
#define SYNC()   do { c->pc = pc; c->delay = delay; c->icount = icount; } while (0)
#define RELOAD() do { pc = c->pc; delay = c->delay; icount = c->icount; ftag = c->fetch_page; \
	fptr = (uintptr_t)c->fetch_base - (uintptr_t)(uint32_t)(ftag << SH2_PAGE_SHIFT); tirq = c->test_irq; } while (0)
#define CALL(stmt) do { SYNC(); stmt; RELOAD(); } while (0)

#define RD8(v, addr) do { uint32_t a_ = (addr); const uint8_t *p_; \
	if (LIKELY(CPU_FAST(a_) && (p_ = c->rd[PAGE(a_)]) != NULL)) (v) = p_[(a_ & 0xfff) ^ 3]; \
	else { SYNC(); (v) = cpu_slow_r8(c, a_); RELOAD(); } } while (0)
#define RD16(v, addr) do { uint32_t a_ = (addr); const uint8_t *p_; \
	if (LIKELY(CPU_FAST(a_) && (p_ = c->rd[PAGE(a_)]) != NULL)) (v) = *(const uint16_t *)(p_ + ((a_ & 0xffe) ^ 2)); \
	else { SYNC(); (v) = cpu_slow_r16(c, a_); RELOAD(); } } while (0)
#define RD32(v, addr) do { uint32_t a_ = (addr); const uint8_t *p_; \
	if (LIKELY(CPU_FAST(a_) && (p_ = c->rd[PAGE(a_)]) != NULL)) (v) = *(const uint32_t *)(p_ + (a_ & 0xffc)); \
	else { SYNC(); (v) = cpu_slow_r32(c, a_); RELOAD(); } } while (0)
#define WR8(addr, v) do { uint32_t a_ = (addr), v_ = (v); uint8_t *p_; \
	if (LIKELY(CPU_FAST(a_) && (p_ = c->wr[PAGE(a_)]) != NULL)) p_[(a_ & 0xfff) ^ 3] = (uint8_t)v_; \
	else { SYNC(); cpu_slow_w8(c, a_, v_); RELOAD(); } } while (0)
#define WR16(addr, v) do { uint32_t a_ = (addr), v_ = (v); uint8_t *p_; \
	if (LIKELY(CPU_FAST(a_) && (p_ = c->wr[PAGE(a_)]) != NULL)) *(uint16_t *)(p_ + ((a_ & 0xffe) ^ 2)) = (uint16_t)v_; \
	else { SYNC(); cpu_slow_w16(c, a_, v_); RELOAD(); } } while (0)
#define WR32(addr, v) do { uint32_t a_ = (addr), v_ = (v); uint8_t *p_; \
	if (LIKELY(CPU_FAST(a_) && (p_ = c->wr[PAGE(a_)]) != NULL)) *(uint32_t *)(p_ + (a_ & 0xffc)) = v_; \
	else { SYNC(); cpu_slow_w32(c, a_, v_); RELOAD(); } } while (0)

// fetch (MAME: m_decrypted_program->read_word(pc >= 0x40000000 ? pc : pc & AM)); ftag = the page of the unmasked pc
// whose host page is fetch_base; fptr = fetch_base less the page's address (fptr + pc lands in the host page)
#define FETCH() do { \
	if (LIKELY((pc >> SH2_PAGE_SHIFT) == ftag)) op = *(const uint16_t *)(fptr + ((pc & ~1u) ^ 2)); \
	else CALL(op = fetch_slow(c, pc)); } while (0)
// the next instruction, taking a pending delay slot
#define DISPATCH() do { FETCH(); \
	if (UNLIKELY(delay)) { pc = delay; delay = 0; } else pc += 2; \
	goto *s_disp.lab[s_disp.tab[op]]; } while (0)
// the end of an instruction that leaves no delay slot (delay was cleared by its fetch and nothing it calls sets it):
// the interrupt test, the cycle, the next instruction at pc + 2
#define NEXT() do { \
	if (UNLIKELY(tirq)) goto irq_check; \
	if (UNLIKELY(--icount <= 0)) goto out; \
	FETCH(); pc += 2; goto *s_disp.lab[s_disp.tab[op]]; } while (0)
// the end of a branch (a delay slot may be pending: no interrupt before it)
#define NEXT_BR() do { \
	if (UNLIKELY(tirq) && !delay) goto irq_check; \
	if (UNLIKELY(--icount <= 0)) goto out; \
	DISPATCH(); } while (0)

#define SH2_LAB(n) &&L_##n,

static void run(sh2_t *c)
{
	static const void *const lab[H_COUNT] = { SH2_OPLIST(SH2_LAB) };
	uint32_t pc, delay, tirq, op;
	int32_t icount;
	uint32_t ftag;
	uintptr_t fptr;

	if (UNLIKELY(!s_disp_ok))
		build_disp(lab);
	RELOAD();

	// an interrupt raised between slices (an event, a DMA end): taken before the first instruction, as MAME's DRC
	// entry does
	if (c->test_irq && !delay) {
		CALL(check_pending_irq(c));
		c->test_irq = 0;
	}
	tirq = c->test_irq;
	DISPATCH();   // (the first instruction runs whatever icount is)

irq_check:
	CALL(check_pending_irq(c));
	c->test_irq = 0;
	tirq = 0;
	if (UNLIKELY(--icount <= 0)) goto out;
	DISPATCH();

out:
	SYNC();
	return;

	// ---- 0xxx
L_STC_SR:   RN = c->sr; NEXT();                                                    // STC SR,Rn
L_STC_GBR:  RN = c->gbr; NEXT();                                                   // STC GBR,Rn
L_STC_VBR:  RN = c->vbr; NEXT();                                                   // STC VBR,Rn
L_BSRF:     c->pr = pc + 2; delay = pc + RN + 2; icount--; NEXT_BR();              // BSRF Rn
L_BRAF:     delay = pc + RN + 2; icount--; NEXT_BR();                              // BRAF Rn
L_MOVB_S0:  WR8(RN + c->r[0], RM); NEXT();                                         // MOV.B Rm,@(R0,Rn)
L_MOVW_S0:  WR16(RN + c->r[0], RM); NEXT();                                        // MOV.W Rm,@(R0,Rn)
L_MOVL_S0:  WR32(RN + c->r[0], RM); NEXT();                                        // MOV.L Rm,@(R0,Rn)
L_MULL:     c->macl = RN * RM; icount--; NEXT();                                   // MUL.L Rm,Rn
L_CLRT:     c->sr &= ~SH_T; NEXT();                                                // CLRT
L_SETT:     c->sr |= SH_T; NEXT();                                                 // SETT
L_CLRMAC:   c->mach = 0; c->macl = 0; NEXT();                                      // CLRMAC
L_NOP:      NEXT();                                                                // NOP
L_DIV0U:    c->sr &= ~(SH_M | SH_Q | SH_T); NEXT();                                // DIV0U
L_MOVT:     RN = c->sr & SH_T; NEXT();                                             // MOVT Rn
L_STS_MACH: RN = c->mach; NEXT();                                                  // STS MACH,Rn
L_STS_MACL: RN = c->macl; NEXT();                                                  // STS MACL,Rn
L_STS_PR:   RN = c->pr; NEXT();                                                    // STS PR,Rn
L_RTS:      delay = c->pr; icount--; NEXT_BR();                                    // RTS
L_SLEEP:    CALL(op_sleep(c)); NEXT();                                             // SLEEP
L_RTE:      CALL(op_rte(c)); NEXT_BR();                                            // RTE (sets delay)
L_MOVB_L0:  { uint32_t v; RD8(v, RM + c->r[0]); RN = SEXT8(v); } NEXT();           // MOV.B @(R0,Rm),Rn
L_MOVW_L0:  { uint32_t v; RD16(v, RM + c->r[0]); RN = SEXT16(v); } NEXT();         // MOV.W @(R0,Rm),Rn
L_MOVL_L0:  { uint32_t v; RD32(v, RM + c->r[0]); RN = v; } NEXT();                 // MOV.L @(R0,Rm),Rn
L_MACL:     CALL(op_mac_l(c, M, N)); NEXT();                                       // MAC.L @Rm+,@Rn+
L_ILLEGAL:  CALL(op_illegal(c)); NEXT();

	// ---- 1xxx
L_MOVL_S4:  WR32(RN + (op & 15) * 4, RM); NEXT();                                  // MOV.L Rm,@(disp4,Rn)

	// ---- 2xxx
L_MOVB_S:   WR8(RN, RM); NEXT();                                                   // MOV.B Rm,@Rn
L_MOVW_S:   WR16(RN, RM); NEXT();                                                  // MOV.W Rm,@Rn
L_MOVL_S:   WR32(RN, RM); NEXT();                                                  // MOV.L Rm,@Rn
L_MOVB_M:   { uint32_t d = RM; RN -= 1; WR8(RN, d); } NEXT();                      // MOV.B Rm,@-Rn
L_MOVW_M:   { uint32_t d = RM; RN -= 2; WR16(RN, d); } NEXT();                     // MOV.W Rm,@-Rn
L_MOVL_M:   { uint32_t d = RM; RN -= 4; WR32(RN, d); } NEXT();                     // MOV.L Rm,@-Rn
L_DIV0S: {                                                                         // DIV0S Rm,Rn
		uint32_t sr = c->sr & ~(SH_Q | SH_M | SH_T);
		if (RN & 0x80000000) sr |= SH_Q;
		if (RM & 0x80000000) sr |= SH_M;
		if ((RM ^ RN) & 0x80000000) sr |= SH_T;
		c->sr = sr;
	}
	NEXT();
L_TST:      SETT_IF((RN & RM) == 0); NEXT();                                       // TST Rm,Rn
L_AND:      RN &= RM; NEXT();                                                      // AND Rm,Rn
L_XOR:      RN ^= RM; NEXT();                                                      // XOR Rm,Rn
L_OR:       RN |= RM; NEXT();                                                      // OR Rm,Rn
L_CMPSTR: {                                                                        // CMP/STR Rm,Rn
		uint32_t t = RN ^ RM;
		if ((t >> 24) && ((t >> 16) & 0xff) && ((t >> 8) & 0xff) && (t & 0xff))
			c->sr &= ~SH_T;
		else
			c->sr |= SH_T;
	}
	NEXT();
L_XTRCT:    RN = (RN >> 16) | (RM << 16); NEXT();                                  // XTRCT Rm,Rn
L_MULU:     c->macl = (uint32_t)(uint16_t)RN * (uint16_t)RM; NEXT();               // MULU.W Rm,Rn
L_MULS:     c->macl = (uint32_t)((int32_t)(int16_t)RN * (int16_t)RM); NEXT();      // MULS.W Rm,Rn

	// ---- 3xxx
L_CMPEQ:    SETT_IF(RN == RM); NEXT();                                             // CMP/EQ
L_CMPHS:    SETT_IF(RN >= RM); NEXT();                                             // CMP/HS
L_CMPGE:    SETT_IF((int32_t)RN >= (int32_t)RM); NEXT();                           // CMP/GE
L_DIV1:     op_div1(c, M, N); NEXT();                                              // DIV1
L_DMULU:    mul64(RN, RM, &c->mach, &c->macl); icount--; NEXT();                   // DMULU.L
L_CMPHI:    SETT_IF(RN > RM); NEXT();                                              // CMP/HI
L_CMPGT:    SETT_IF((int32_t)RN > (int32_t)RM); NEXT();                            // CMP/GT
L_SUB:      RN -= RM; NEXT();                                                      // SUB
L_SUBC: {                                                                          // SUBC
		uint32_t tmp1 = RN - RM;
		uint32_t tmp0 = RN;
		RN = tmp1 - (c->sr & SH_T);
		SETT_IF(tmp0 < tmp1);
		if (tmp1 < RN) c->sr |= SH_T;
	}
	NEXT();
L_SUBV: {                                                                          // SUBV
		int32_t dest = (RN >> 31) & 1;
		int32_t src = ((RM >> 31) & 1) + dest;
		RN -= RM;
		int32_t ans = ((RN >> 31) & 1) + dest;
		SETT_IF(src == 1 && ans == 1);
	}
	NEXT();
L_ADD:      RN += RM; NEXT();                                                      // ADD
L_DMULS: {                                                                         // DMULS.L
		int64_t p = (int64_t)(int32_t)RN * (int32_t)RM;
		c->mach = (uint32_t)((uint64_t)p >> 32);
		c->macl = (uint32_t)p;
		icount--;
	}
	NEXT();
L_ADDC: {                                                                          // ADDC
		uint32_t tmp1 = RN + RM;
		uint32_t tmp0 = RN;
		RN = tmp1 + (c->sr & SH_T);
		SETT_IF(tmp0 > tmp1);
		if (tmp1 > RN) c->sr |= SH_T;
	}
	NEXT();
L_ADDV: {                                                                          // ADDV
		int32_t dest = (RN >> 31) & 1;
		int32_t src = ((RM >> 31) & 1) + dest;
		RN += RM;
		int32_t ans = ((RN >> 31) & 1) + dest;
		if (src != 1) SETT_IF(ans == 1); else c->sr &= ~SH_T;
	}
	NEXT();

	// ---- 4xxx
L_SHLL:     c->sr = (c->sr & ~SH_T) | ((RN >> 31) & SH_T); RN <<= 1; NEXT();       // SHLL / SHAL
L_SHLR:     c->sr = (c->sr & ~SH_T) | (RN & SH_T); RN >>= 1; NEXT();               // SHLR
L_SHAR:     c->sr = (c->sr & ~SH_T) | (RN & SH_T); RN = (uint32_t)((int32_t)RN >> 1); NEXT();   // SHAR
L_STSM_MACH: RN -= 4; WR32(RN, c->mach); NEXT();                                   // STS.L MACH,@-Rn
L_STSM_MACL: RN -= 4; WR32(RN, c->macl); NEXT();                                   // STS.L MACL,@-Rn
L_STSM_PR:  RN -= 4; WR32(RN, c->pr); NEXT();                                      // STS.L PR,@-Rn
L_STCM_SR:  RN -= 4; WR32(RN, c->sr); icount--; NEXT();                            // STC.L SR,@-Rn
L_STCM_GBR: RN -= 4; WR32(RN, c->gbr); icount--; NEXT();                           // STC.L GBR,@-Rn
L_STCM_VBR: RN -= 4; WR32(RN, c->vbr); icount--; NEXT();                           // STC.L VBR,@-Rn
L_ROTL:     c->sr = (c->sr & ~SH_T) | ((RN >> 31) & SH_T); RN = (RN << 1) | (RN >> 31); NEXT();   // ROTL
L_ROTR:     c->sr = (c->sr & ~SH_T) | (RN & SH_T); RN = (RN >> 1) | (RN << 31); NEXT();           // ROTR
L_ROTCL: {                                                                         // ROTCL
		uint32_t t = (RN >> 31) & SH_T;
		RN = (RN << 1) | (c->sr & SH_T);
		c->sr = (c->sr & ~SH_T) | t;
	}
	NEXT();
L_ROTCR: {                                                                         // ROTCR
		uint32_t t = (c->sr & SH_T) << 31;
		SETT_IF(RN & SH_T);
		RN = (RN >> 1) | t;
	}
	NEXT();
L_LDSM_MACH: { uint32_t v; RD32(v, RN); c->mach = v; RN += 4; } NEXT();            // LDS.L @Rm+,MACH
L_LDSM_MACL: { uint32_t v; RD32(v, RN); c->macl = v; RN += 4; } NEXT();            // LDS.L @Rm+,MACL
L_LDSM_PR:  { uint32_t v; RD32(v, RN); c->pr = v; RN += 4; } NEXT();               // LDS.L @Rm+,PR
L_LDCM_SR: {                                                                       // LDC.L @Rm+,SR
		uint32_t v;
		RD32(v, RN);
		c->sr = v & SH_FLAGS;
		RN += 4;
		icount -= 2;
		c->test_irq = 1;
		tirq = 1;
	}
	NEXT();
L_LDCM_GBR: { uint32_t v; RD32(v, RN); c->gbr = v; RN += 4; icount -= 2; } NEXT(); // LDC.L @Rm+,GBR
L_LDCM_VBR: { uint32_t v; RD32(v, RN); c->vbr = v; RN += 4; icount -= 2; } NEXT(); // LDC.L @Rm+,VBR
L_SHLL2:    RN <<= 2; NEXT();                                                      // SHLL2
L_SHLL8:    RN <<= 8; NEXT();                                                      // SHLL8
L_SHLL16:   RN <<= 16; NEXT();                                                     // SHLL16
L_SHLR2:    RN >>= 2; NEXT();                                                      // SHLR2
L_SHLR8:    RN >>= 8; NEXT();                                                      // SHLR8
L_SHLR16:   RN >>= 16; NEXT();                                                     // SHLR16
L_LDS_MACH: c->mach = RN; NEXT();                                                  // LDS Rm,MACH
L_LDS_MACL: c->macl = RN; NEXT();                                                  // LDS Rm,MACL
L_LDS_PR:   c->pr = RN; NEXT();                                                    // LDS Rm,PR
L_JSR:      c->pr = pc + 2; delay = RN; icount--; NEXT_BR();                       // JSR @Rm
L_JMP:      delay = RN; NEXT_BR();                                                 // JMP @Rm (MAME: no extra cycle)
L_TAS: {                                                                           // TAS.B @Rn
		uint32_t ea = RN, t;
		RD8(t, ea);
		SETT_IF(t == 0);
		WR8(ea, t | 0x80);
		icount -= 3;
	}
	NEXT();
L_LDC_SR:   c->sr = RN & SH_FLAGS; c->test_irq = 1; tirq = 1; NEXT();              // LDC Rm,SR
L_LDC_GBR:  c->gbr = RN; NEXT();                                                   // LDC Rm,GBR
L_LDC_VBR:  c->vbr = RN; NEXT();                                                   // LDC Rm,VBR
L_MACW:     CALL(op_mac_w(c, M, N)); NEXT();                                       // MAC.W @Rm+,@Rn+
L_DT:       RN--; SETT_IF(RN == 0); NEXT();                                        // DT Rn
L_CMPPZ:    SETT_IF((int32_t)RN >= 0); NEXT();                                     // CMP/PZ
L_CMPPL:    SETT_IF((int32_t)RN > 0); NEXT();                                      // CMP/PL

	// ---- 5xxx
L_MOVL_L4:  { uint32_t v; RD32(v, RM + (op & 15) * 4); RN = v; } NEXT();           // MOV.L @(disp4,Rm),Rn

	// ---- 6xxx
L_MOVB_L:   { uint32_t v; RD8(v, RM); RN = SEXT8(v); } NEXT();                     // MOV.B @Rm,Rn
L_MOVW_L:   { uint32_t v; RD16(v, RM); RN = SEXT16(v); } NEXT();                   // MOV.W @Rm,Rn
L_MOVL_L:   { uint32_t v; RD32(v, RM); RN = v; } NEXT();                           // MOV.L @Rm,Rn
L_MOV:      RN = RM; NEXT();                                                       // MOV Rm,Rn
L_MOVB_P:   { uint32_t m = M, n = N, v; RD8(v, c->r[m]); c->r[n] = SEXT8(v); if (n != m) c->r[m] += 1; } NEXT();    // MOV.B @Rm+,Rn
L_MOVW_P:   { uint32_t m = M, n = N, v; RD16(v, c->r[m]); c->r[n] = SEXT16(v); if (n != m) c->r[m] += 2; } NEXT();  // MOV.W @Rm+,Rn
L_MOVL_P:   { uint32_t m = M, n = N, v; RD32(v, c->r[m]); c->r[n] = v; if (n != m) c->r[m] += 4; } NEXT();          // MOV.L @Rm+,Rn
L_NOT:      RN = ~RM; NEXT();                                                      // NOT
L_SWAPB:    { uint32_t v = RM; RN = (v & 0xffff0000) | ((v & 0xff) << 8) | ((v >> 8) & 0xff); } NEXT();   // SWAP.B
L_SWAPW:    { uint32_t v = RM; RN = (v << 16) | (v >> 16); } NEXT();               // SWAP.W
L_NEGC: {                                                                          // NEGC
		uint32_t t = RM;
		RN = 0u - t - (c->sr & SH_T);
		SETT_IF(t || (c->sr & SH_T));
	}
	NEXT();
L_NEG:      RN = 0u - RM; NEXT();                                                  // NEG
L_EXTUB:    RN = RM & 0xff; NEXT();                                                // EXTU.B
L_EXTUW:    RN = RM & 0xffff; NEXT();                                              // EXTU.W
L_EXTSB:    RN = SEXT8(RM); NEXT();                                                // EXTS.B
L_EXTSW:    RN = SEXT16(RM); NEXT();                                               // EXTS.W

	// ---- 7xxx
L_ADDI:     RN += SEXT8(op); NEXT();                                               // ADD #imm,Rn

	// ---- 8xxx
L_MOVB_S4R0: WR8(RM + (op & 15), c->r[0]); NEXT();                                 // MOV.B R0,@(disp4,Rn)
L_MOVW_S4R0: WR16(RM + (op & 15) * 2, c->r[0]); NEXT();                            // MOV.W R0,@(disp4,Rn)
L_MOVB_L4R0: { uint32_t v; RD8(v, RM + (op & 15)); c->r[0] = SEXT8(v); } NEXT();        // MOV.B @(disp4,Rm),R0
L_MOVW_L4R0: { uint32_t v; RD16(v, RM + (op & 15) * 2); c->r[0] = SEXT16(v); } NEXT();  // MOV.W @(disp4,Rm),R0
L_CMPIM:    SETT_IF(c->r[0] == SEXT8(op)); NEXT();                                 // CMP/EQ #imm,R0
L_BT:                                                                              // BT
	if (c->sr & SH_T) {
		pc = pc + SEXT8(op) * 2 + 2;
		icount -= 2;
		if (UNLIKELY(pc == c->idle_pc[0] || pc == c->idle_pc[1] || pc == c->idle_pc[2] || pc == c->idle_pc[3]))   // (the driver's idle loop: it may skip whole rounds)
			CALL(c->idle(c));
	}
	NEXT();
L_BF:                                                                              // BF
	if (!(c->sr & SH_T)) {
		pc = pc + SEXT8(op) * 2 + 2;
		icount -= 2;
		if (UNLIKELY(pc == c->idle_pc[0] || pc == c->idle_pc[1] || pc == c->idle_pc[2] || pc == c->idle_pc[3]))
			CALL(c->idle(c));
	}
	NEXT();
L_BTS:                                                                             // BT/S
	if (c->sr & SH_T) { delay = pc + SEXT8(op) * 2 + 2; icount--; }
	NEXT_BR();
L_BFS:                                                                             // BF/S
	if (!(c->sr & SH_T)) { delay = pc + SEXT8(op) * 2 + 2; icount--; }
	NEXT_BR();

	// ---- 9xxx
L_MOVW_I:   { uint32_t v; RD16(v, pc + (op & 0xff) * 2 + 2); RN = SEXT16(v); } NEXT();   // MOV.W @(disp8,PC),Rn

	// ---- Axxx
L_BRA:                                                                             // BRA disp12
	delay = pc + SEXT12(op & 0xfff) * 2 + 2;
	icount--;
	if (UNLIKELY(delay == c->idle_pc[0] || delay == c->idle_pc[1] || delay == c->idle_pc[2] || delay == c->idle_pc[3]))
		CALL(c->idle(c));
	NEXT_BR();

	// ---- Bxxx
L_BSR:                                                                             // BSR disp12
	c->pr = pc + 2;
	delay = pc + SEXT12(op & 0xfff) * 2 + 2;
	icount--;
	NEXT_BR();

	// ---- Cxxx
L_MOVB_SG:  WR8(c->gbr + (op & 0xff), c->r[0]); NEXT();                            // MOV.B R0,@(disp8,GBR)
L_MOVW_SG:  WR16(c->gbr + (op & 0xff) * 2, c->r[0]); NEXT();                       // MOV.W R0,@(disp8,GBR)
L_MOVL_SG:  WR32(c->gbr + (op & 0xff) * 4, c->r[0]); NEXT();                       // MOV.L R0,@(disp8,GBR)
L_TRAPA:    CALL(op_trapa(c, op & 0xff)); NEXT();                                  // TRAPA #imm
L_MOVB_LG:  { uint32_t v; RD8(v, c->gbr + (op & 0xff)); c->r[0] = SEXT8(v); } NEXT();        // MOV.B @(disp8,GBR),R0
L_MOVW_LG:  { uint32_t v; RD16(v, c->gbr + (op & 0xff) * 2); c->r[0] = SEXT16(v); } NEXT();  // MOV.W @(disp8,GBR),R0
L_MOVL_LG:  { uint32_t v; RD32(v, c->gbr + (op & 0xff) * 4); c->r[0] = v; } NEXT();          // MOV.L @(disp8,GBR),R0
L_MOVA:     c->r[0] = ((pc + 2) & ~3u) + (op & 0xff) * 4; NEXT();                  // MOVA @(disp8,PC),R0
L_TSTI:     SETT_IF(((op & 0xff) & c->r[0]) == 0); NEXT();                         // TST #imm,R0
L_ANDI:     c->r[0] &= op & 0xff; NEXT();                                          // AND #imm,R0
L_XORI:     c->r[0] ^= op & 0xff; NEXT();                                          // XOR #imm,R0
L_ORI:      c->r[0] |= op & 0xff; icount -= 2; NEXT();                             // OR #imm,R0 (MAME: 2 extra)
L_TSTB: {                                                                          // TST.B #imm,@(R0,GBR)
		uint32_t v;
		RD8(v, c->gbr + c->r[0]);
		SETT_IF(((op & 0xff) & v) == 0);
		icount -= 2;
	}
	NEXT();
L_ANDB: {                                                                          // AND.B #imm,@(R0,GBR)
		uint32_t ea = c->gbr + c->r[0], v;
		RD8(v, ea);
		WR8(ea, (op & 0xff) & v);
		icount -= 2;
	}
	NEXT();
L_XORB: {                                                                          // XOR.B #imm,@(R0,GBR)
		uint32_t ea = c->gbr + c->r[0], v;
		RD8(v, ea);
		WR8(ea, v ^ (op & 0xff));
		icount -= 2;
	}
	NEXT();
L_ORB: {                                                                           // OR.B #imm,@(R0,GBR) (MAME: no extra)
		uint32_t ea = c->gbr + c->r[0], v;
		RD8(v, ea);
		WR8(ea, v | (op & 0xff));
	}
	NEXT();

	// ---- Dxxx
L_MOVL_I:   { uint32_t v; RD32(v, ((pc + 2) & ~3u) + (op & 0xff) * 4); RN = v; } NEXT();   // MOV.L @(disp8,PC),Rn

	// ---- Exxx
L_MOVI:     RN = SEXT8(op); NEXT();                                                // MOV #imm,Rn
}

/***************************************************************************
    execute
***************************************************************************/

// fire the events due by the CPU clock t, in time order
static void run_events(sh2_t *c)
{
	for (;;) {
		int64_t t = c->total_cycles;
		dma_catch_up(c, t);
		int64_t e = c->frt_event;
		int kind = 0;
		if (c->wdt_event < e) { e = c->wdt_event; kind = 1; }
		if (e > t) break;
		if (kind == 0) timer_callback(c);
		else wdtimer_callback(c);
	}
}

int sh2_execute(sh2_t *c, int cycles)
{
	int64_t start = c->total_cycles;
	if (cycles > 0)
		c->target += cycles;
	c->abort_slice = 0;

	while (c->total_cycles < c->target && !c->abort_slice) {
		int64_t stop = next_event(c);
		if (stop > c->target) stop = c->target;
		if (stop <= c->total_cycles) {   // an event is due
			run_events(c);
			stop = next_event(c);
			if (stop > c->target) stop = c->target;
			if (stop <= c->total_cycles) stop = c->total_cycles + 1;
		}
		if (stop - c->total_cycles > 0x40000000) stop = c->total_cycles + 0x40000000;

		if (c->halted) {   // cycle-steal DMA: the CPU waits for the end of the transfer
			c->total_cycles = stop;
		} else {
			c->slice_end = stop;
			c->icount = (int32_t)(stop - c->total_cycles);
			c->running = 1;
			run(c);
			c->running = 0;
			c->total_cycles = c->slice_end - c->icount;
		}
		run_events(c);
	}
	return (int)(c->total_cycles - start);
}

void sh2_end_timeslice(sh2_t *c)
{
	if (c->running) {
		c->abort_slice = 1;
		c->slice_end -= c->icount;
		c->icount = 0;
	} else
		c->abort_slice = 1;
}

void sh2_eat_cycles(sh2_t *c, int n)
{
	if (n <= 0) return;
	if (c->running)
		c->icount -= n;
	else
		c->total_cycles += n;
}

/***************************************************************************
    init / reset / state
***************************************************************************/

static uint8_t dummy_r8(void *u, uint32_t a) { (void)u; (void)a; return 0; }
static uint16_t dummy_r16(void *u, uint32_t a) { (void)u; (void)a; return 0; }
static uint32_t dummy_r32(void *u, uint32_t a) { (void)u; (void)a; return 0; }
static void dummy_w8(void *u, uint32_t a, uint8_t d) { (void)u; (void)a; (void)d; }
static void dummy_w16(void *u, uint32_t a, uint16_t d) { (void)u; (void)a; (void)d; }
static void dummy_w32(void *u, uint32_t a, uint32_t d) { (void)u; (void)a; (void)d; }

// (the cheats' access to this CPU's program space: cheat_cpu.h)
static uint8_t cheat_rd(void *x, uint32_t a) { return sh2_read8((sh2_t *)x, a); }   // (as the CPU: page table, callbacks)
static void cheat_wr(void *x, uint32_t a, uint8_t v) { sh2_write8((sh2_t *)x, a, v); }

void sh2_init(sh2_t *c)
{
	rbx_cheat_offer("sh2", cheat_rd, cheat_wr, c, 1, 1);
	memset(c, 0, sizeof(*c));
	c->read8 = dummy_r8; c->read16 = dummy_r16; c->read32 = dummy_r32;
	c->write8 = dummy_w8; c->write16 = dummy_w16; c->write32 = dummy_w32;
	c->fetch_page = ~0u;
	c->frt_event = c->wdt_event = SH2_NEVER;
	c->dma_tick[0] = c->dma_tick[1] = SH2_NEVER;
	c->internal_irq_level = -1;
	c->idle_pc[0] = c->idle_pc[1] = c->idle_pc[2] = c->idle_pc[3] = 1;
}

void sh2_reset(sh2_t *c)
{
	if (!c->read8) c->read8 = dummy_r8;
	if (!c->read16) c->read16 = dummy_r16;
	if (!c->read32) c->read32 = dummy_r32;
	if (!c->write8) c->write8 = dummy_w8;
	if (!c->write16) c->write16 = dummy_w16;
	if (!c->write32) c->write32 = dummy_w32;
	c->running = 0;
	c->abort_slice = 0;
	c->fetch_page = ~0u;
	c->fetch_base = NULL;
	c->total_cycles = c->target = 0;

	// sh2_device::device_reset
	memset(c->r, 0, sizeof(c->r));
	memset(c->irq_line_state, 0, sizeof(c->irq_line_state));
	c->pc = c->pr = c->sr = c->gbr = c->vbr = c->mach = c->macl = 0;
	c->delay = 0;
	c->pending_irq = 0;
	c->pending_nmi = 0;
	c->sleep_mode = 0;
	c->internal_irq_level = -1;
	c->sr = SH_I;
	c->pc = RL(c, 0);
	c->r[15] = RL(c, 4);
	c->test_irq = 0;
	c->internal_irq_vector = 0;
	c->halted = 0;
	c->nmi_line_state = 0;   // (MAME: cleared at device_start only)

	// sh7604_device::device_reset
	c->frc = 0;
	c->ocra = 0;
	c->ocrb = 0;
	c->frc_icr = 0;
	c->frc_base = 0;
	c->frt_input = 0;
	c->frt_event = SH2_NEVER;   // (MAME: the emu_timers are not touched by a reset; a new game starts them off)
	c->wdt_event = SH2_NEVER;
	for (int i = 0; i < 2; i++) {
		c->dma_active[i] = 0;
		c->dma_irq[i] = 0;
		c->dma_incs[i] = c->dma_incd[i] = c->dma_size[i] = c->dma_steal[i] = 0;
		c->dma_src[i] = c->dma_dst[i] = c->dma_count[i] = 0;
		c->dma_tick[i] = SH2_NEVER;
	}
	c->wtcnt = 0;
	c->wtcsr = 0;
	c->barah = c->baral = c->barbh = c->barbl = 0;
}

void sh2_post_load(sh2_t *c)
{
	c->running = 0;
	c->abort_slice = 0;
	c->fetch_page = ~0u;
	c->fetch_base = NULL;
}
