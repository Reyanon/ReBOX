/*  Exact skipping of a V30's polling loops inside one of its scheduler runs: the loop's iteration is run one
    instruction at a time with the CPU's RAM pages' writes caught; when it comes back to its first instruction with
    no write and every register as before (the memory it reads unchanged, nothing else runs during the run, no
    interrupt pending), the next iterations are the same: whole ones are counted without being run, the rest of the
    run as usual. The result is the same as running them (RBX_SEIBU_NOIDLE=1 turns it off to compare).
    The board's direct write pages must not change after necidle_init. No REP instruction is run here (one run alone
    would stop after an element, a split MAME's run does not make).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_NECIDLE_H
#define RBX_NECIDLE_H

#include "../../cpu/nec/nec.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
	nec_t *c;
	uint8_t *saved[256];
	uint8_t pages[64];
	int npages, wrote, on;
	void (*w8)(uint32_t a, uint8_t v);
	void (*w16)(uint32_t a, uint16_t v);
} necidle_t;

static necidle_t *s_ni_cur;

static void necidle_w8(uint32_t a, uint8_t v)
{
	necidle_t *ni = s_ni_cur;
	uint8_t *p = ni->saved[(a >> 12) & 0xff];
	ni->wrote = 1;
	if (p)
		p[a & 0xfff] = v;
	else
		ni->w8(a, v);
}

static void necidle_w16(uint32_t a, uint16_t v)
{
	necidle_t *ni = s_ni_cur;
	uint8_t *p = ni->saved[(a >> 12) & 0xff];
	ni->wrote = 1;
	if (p && (a & 0xfff) != 0xfff) {
		p[a & 0xfff] = (uint8_t)v;
		p[(a & 0xfff) + 1] = (uint8_t)(v >> 8);
	} else {
		necidle_w8(a, (uint8_t)v);
		necidle_w8(a + 1, (uint8_t)(v >> 8));
	}
}

// after the board mapped the CPU
static void necidle_init(necidle_t *ni, nec_t *c)
{
	memset(ni, 0, sizeof(*ni));
	ni->c = c;
	for (int p = 0; p < 256 && ni->npages < 64; p++)
		if (c->wmap[p])
			ni->pages[ni->npages++] = (uint8_t)p;
	ni->on = !getenv("RBX_SEIBU_NOIDLE");
}

// 1 = the instruction at the PC has a REP prefix (or its bytes are not on a direct page)
static int necidle_rep(const nec_t *c)
{
	for (int i = 0; i < 8; i++) {
		const uint32_t a = (((uint32_t)c->rp[6] << 4) + ((c->ip + (uint32_t)i) & 0xffff)) & 0xfffff;
		const uint8_t *p = c->omap[a >> 12];
		if (!p)
			return 1;
		switch (p[a & 0xfff]) {
		case 0xf2: case 0xf3: case 0x64: case 0x65:
			return 1;
		case 0x26: case 0x2e: case 0x36: case 0x3e: case 0xf0:
			break;
		default:
			return 0;
		}
	}
	return 1;
}

// the V30's state compared: its registers (the bank 7 of iram) up to the instruction's counters
#define NECIDLE_CMP0 offsetof(nec_t, iram[112])
#define NECIDLE_CMPN (offsetof(nec_t, ic) - NECIDLE_CMP0)

// the probe at a run's start (n: the run's cycles, start: the CPU's total at it). 1 = the run was stopped (an abort
// in an instruction run here: the run returns now)
static int necidle_probe(necidle_t *ni, int n, int64_t start, const int *aborted)
{
	nec_t *c = ni->c;
	if (!ni->on || n < 64 || c->pending_irq || c->rep_params || c->halted)
		return 0;
	const uint32_t pc = ((uint32_t)c->rp[6] << 4) + c->ip;
	static uint8_t snap[sizeof(nec_t)];
	memcpy(snap, (const uint8_t *)c + NECIDLE_CMP0, NECIDLE_CMPN);
	const int64_t t0 = c->total;
	void (*ow8)(uint32_t, uint8_t) = c->write8;
	void (*ow16)(uint32_t, uint16_t) = c->write16;
	ni->w8 = ow8;
	ni->w16 = ow16;
	ni->wrote = 0;
	for (int k = 0; k < ni->npages; k++) {
		ni->saved[ni->pages[k]] = c->wmap[ni->pages[k]];
		c->wmap[ni->pages[k]] = NULL;
	}
	c->write8 = necidle_w8;
	c->write16 = necidle_w16;
	s_ni_cur = ni;
	int64_t T = 0;
	int stop = 0;
	for (int k = 0; k < 16; k++) {
		if (necidle_rep(c))
			break;
		nec_run_until(c, c->total + 1);
		if (*aborted) {
			stop = 1;
			break;
		}
		if (ni->wrote || c->pending_irq || c->rep_params || c->halted || c->total - start >= n)
			break;
		if (((uint32_t)c->rp[6] << 4) + c->ip == pc) {
			// (the counters of the last instruction aside)
#define NECIDLE_SET(f) memcpy(snap + offsetof(nec_t, f) - NECIDLE_CMP0, &c->f, sizeof(c->f))
			NECIDLE_SET(prev_ip);
			NECIDLE_SET(cur_cycles);
			NECIDLE_SET(EA);
			NECIDLE_SET(popa_tmp);
#undef NECIDLE_SET
			if (!memcmp(snap, (const uint8_t *)c + NECIDLE_CMP0, NECIDLE_CMPN))
				T = c->total - t0;
			break;
		}
	}
	c->write8 = ow8;
	c->write16 = ow16;
	for (int k = 0; k < ni->npages; k++)
		c->wmap[ni->pages[k]] = ni->saved[ni->pages[k]];
	if (stop)
		return 1;
	if (T > 0) {
		const int64_t left = n - (c->total - start);
		if (left > 2 * T)
			c->total += (left / T - 1) * T;
	}
	return 0;
}

#endif
