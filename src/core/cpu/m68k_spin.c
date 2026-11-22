/*  Polling loops of a 68000 skipped within a slice (m68k_spin.h). Written for ReBOX's Sega X-Board / Y-Board.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include <stdlib.h>
#include <string.h>

#include "m68k_spin.h"
#include "m68k_bus.h"
#include "m68k/m68kcpu.h"   // (after the system headers: its uint / uint32 macros)

#define SPIN_SETS 32        // four templates a set
#define SPIN_WAYS 4
#define SPIN_WORDS 12       // memory words a turn touches
#define SPIN_REGS 8         // registers a turn touches (counters aside)
#define SPIN_STEPS 64       // instructions a turn
#define SPIN_TURN_MAX 4096  // cycles a turn
#define SPIN_WALK 24        // instructions run one at a time after a skip cut short by a counter

enum { T_EMPTY, T_VALID, T_FAIL };

struct spin_tpl {
	uint32_t pc;
	uint16_t key, sr;
	uint8_t state, backoff, fails, nr, nw, nc;
	int32_t cyc;
	uint8_t rid[SPIN_REGS];
	uint32_t rval[SPIN_REGS];
	uint32_t waddr[SPIN_WORDS];   // (bit 0: written)
	uint16_t wval[SPIN_WORDS];
	uint8_t creg[2], ckind[2];    // (ckind 1 / 2 / 4: addq / subq of that size, 8: dbf)
	int8_t cd[2];
	// a counter in memory (addq / subq #k,<ea> of a word or long, its words touched by nothing else): msz 0 none
	uint8_t msz;
	int8_t mcd;
	uint16_t mmask;               // (its words among waddr: left out of the comparisons)
	uint32_t maddr;
};

// the instruction stream as the CPU fetches it
static uint32_t spin_rd16(uint32_t a)
{
	a &= 0xffffff;
	if (m68k_bus_cur->op && a < m68k_bus_cur->op_size)
		return *(const uint16_t *)(m68k_bus_cur->op + a);
	const uint8_t *pg = m68k_bus_cur->rd[a >> M68K_PAGE_BITS];
	return pg ? *(const uint16_t *)(pg + (a & ((1u << M68K_PAGE_BITS) - 1))) : 0xffffffffu;
}

// ---- the instructions a template may hold ----
#define F_N 8
#define F_Z 4
#define F_V 2
#define F_C 1
static const uint8_t s_ccuse[16] = { 0, 0, F_C | F_Z, F_C | F_Z, F_C, F_C, F_Z, F_Z, F_V, F_V, F_N, F_N, F_N | F_V,
                                     F_N | F_V, F_N | F_V | F_Z, F_N | F_V | F_Z };

typedef struct { uint8_t mode, reg, sz, wr; uint32_t ext; } spin_ea_t;
typedef struct {
	uint8_t len, nea, fset, fuse;
	uint8_t cnt, creg, csz;   // cnt 1: addq / subq #k,Dn (cd = +-k), 2: dbf Dn (cd = -1), 3: addq / subq #k,<ea> (ea[0])
	int8_t cd;
	uint16_t touch;           // the registers it uses besides a counter op's own (D0-D7: bits 0-7, A0-A7: 8-15)
	spin_ea_t ea[2];
} spin_dec_t;

// an <ea> whose extension words start at *x; 0: not allowed
static int dec_ea(spin_dec_t *d, int mode, int reg, int sz, int wr, uint32_t *x, int imm_ok, int an_ok)
{
	switch (mode) {
	case 0: d->touch |= (uint16_t)(1u << reg); return 1;
	case 1:
		if (!an_ok || sz == 1)
			return 0;
		d->touch |= (uint16_t)(0x100u << reg);
		return 1;
	case 2:
	case 5: d->touch |= (uint16_t)(0x100u << reg); break;
	case 7:
		if (reg == 4) {
			if (!imm_ok)
				return 0;
			*x += sz == 4 ? 4 : 2;
			return 1;
		}
		if (reg > 1)
			return 0;
		break;
	default: return 0;
	}
	spin_ea_t *e = &d->ea[d->nea++];
	e->mode = (uint8_t)mode;
	e->reg = (uint8_t)reg;
	e->sz = (uint8_t)sz;
	e->wr = (uint8_t)wr;
	e->ext = *x;
	*x += mode == 2 ? 0 : (mode == 7 && reg == 1) ? 4 : 2;
	return 1;
}

static int spin_dec(uint32_t pc, spin_dec_t *d)
{
	const uint32_t op = spin_rd16(pc);
	const int mode = (op >> 3) & 7, reg = op & 7, s = (op >> 6) & 3;
	uint32_t x = pc + 2;
	d->nea = d->fset = d->fuse = d->cnt = 0;
	d->touch = 0;
	if (op == 0x4e71) {   // nop
	} else if ((op & 0xff00) == 0x4a00) {   // tst / tas
		if (!dec_ea(d, mode, reg, s < 3 ? 1 << s : 1, s == 3, &x, 0, 0))
			return 0;
		d->fset = 15;
	} else if ((op & 0xff00) == 0x4200 && s < 3) {   // clr
		if (!dec_ea(d, mode, reg, 1 << s, 1, &x, 0, 0))
			return 0;
		d->fset = 15;
	} else if ((op & 0xff00) == 0x0800 || ((op & 0xf100) == 0x0100 && mode != 1)) {   // btst / bchg / bclr / bset
		if (op & 0x100)
			d->touch |= (uint16_t)(1u << ((op >> 9) & 7));
		else
			x += 2;
		if (!dec_ea(d, mode, reg, mode ? 1 : 4, s != 0, &x, 0, 0))
			return 0;
		d->fset = F_Z;
	} else if ((op >> 12) >= 1 && (op >> 12) <= 3) {   // move (not movea)
		const int sz = (op >> 12) == 1 ? 1 : (op >> 12) == 3 ? 2 : 4;
		if (((op >> 6) & 7) == 1 || !dec_ea(d, mode, reg, sz, 0, &x, 1, 1) ||
		    !dec_ea(d, (op >> 6) & 7, (op >> 9) & 7, sz, 1, &x, 0, 0))
			return 0;
		d->fset = 15;
	} else if ((op & 0xf100) == 0x7000) {   // moveq
		d->touch |= (uint16_t)(1u << ((op >> 9) & 7));
		d->fset = 15;
	} else if ((op & 0xff00) == 0x0c00 && s < 3) {   // cmpi (not cmpi.l #,Dn: the FD1094's state change)
		if (mode == 0 && s == 2)
			return 0;
		x += s == 2 ? 4 : 2;
		if (!dec_ea(d, mode, reg, 1 << s, 0, &x, 0, 0))
			return 0;
		d->fset = 15;
	} else if (((op & 0xff00) == 0x0000 || (op & 0xff00) == 0x0200 || (op & 0xff00) == 0x0a00) && s < 3 && mode == 0) {
		x += s == 2 ? 4 : 2;   // ori / andi / eori #,Dn
		d->touch |= (uint16_t)(1u << reg);
		d->fset = 15;
	} else if ((op & 0xf100) == 0xb000 && s < 3) {   // cmp <ea>,Dn
		d->touch |= (uint16_t)(1u << ((op >> 9) & 7));
		if (!dec_ea(d, mode, reg, 1 << s, 0, &x, 1, 1))
			return 0;
		d->fset = 15;
	} else if ((op & 0xf000) == 0x5000 && s < 3 && mode == 0) {   // addq / subq #k,Dn
		const int k = ((op >> 9) & 7) ? (int)((op >> 9) & 7) : 8;
		d->cnt = 1;
		d->creg = (uint8_t)reg;
		d->csz = (uint8_t)(1 << s);
		d->cd = (int8_t)((op & 0x100) ? -k : k);
		d->fset = 15;
	} else if ((op & 0xf000) == 0x5000 && (s == 1 || s == 2) && (mode == 2 || mode == 5 || mode == 7)) {
		// addq / subq #k,<ea>: a counter in memory (a word or long)
		if (!dec_ea(d, mode, reg, 1 << s, 1, &x, 0, 0))
			return 0;
		const int k = ((op >> 9) & 7) ? (int)((op >> 9) & 7) : 8;
		d->cnt = 3;
		d->csz = (uint8_t)(1 << s);
		d->cd = (int8_t)((op & 0x100) ? -k : k);
		d->fset = 15;
	} else if ((op & 0xf0f8) == 0x50c8) {   // dbcc
		const int cc = (op >> 8) & 15;
		x += 2;
		if (cc == 1) {
			d->cnt = 2;
			d->creg = (uint8_t)reg;
			d->csz = 2;
			d->cd = -1;
		} else if (cc != 0) {
			d->touch |= (uint16_t)(1u << reg);
			d->fuse = s_ccuse[cc];
		}
	} else if ((op & 0xf000) == 0x6000 && ((op >> 8) & 15) != 1) {   // bcc / bra (not bsr)
		if ((op & 0xff) == 0)
			x += 2;
		d->fuse = s_ccuse[(op >> 8) & 15];
	} else
		return 0;
	d->len = (uint8_t)(x - pc);
	return 1;
}

// its words on pages nothing writes (or in the decrypted opcodes)
static int code_ok(uint32_t pc, int len)
{
	const m68k_bus_t *b = m68k_bus_cur;
	for (uint32_t a = pc; a < pc + (uint32_t)len; a += 2) {
		const uint32_t y = a & 0xffffff;
		if (b->op && y < b->op_size)
			continue;
		if (!b->rd[y >> M68K_PAGE_BITS] || b->wr[y >> M68K_PAGE_BITS])
			return 0;
	}
	return 1;
}

static uint32_t ea_addr(const spin_ea_t *e)
{
	uint32_t a;
	switch (e->mode) {
	case 2: a = REG_DA[8 + e->reg]; break;
	case 5: a = REG_DA[8 + e->reg] + (uint32_t)(int16_t)spin_rd16(e->ext); break;
	default: a = e->reg == 0 ? (uint32_t)(int16_t)spin_rd16(e->ext) : spin_rd16(e->ext) << 16 | spin_rd16(e->ext + 2); break;
	}
	return a & 0xffffff;
}

#define PAGE_OFS(y) ((y) & ((1u << M68K_PAGE_BITS) - 1))

// the words of a memory operand (as they are, before the instruction) added to t; 0: not plain RAM / ROM
static int add_words(struct spin_tpl *t, uint32_t a, int sz, int wr)
{
	if (sz > 1 && (a & 1))
		return 0;
	const uint32_t first = a & ~1u, last = (a + (uint32_t)sz - 1) & ~1u;
	for (uint32_t x = first; x <= last; x += 2) {
		const uint32_t y = x & 0xffffff, p = y >> M68K_PAGE_BITS;
		const uint8_t *rp = m68k_bus_cur->rd[p];
		if (!rp || (wr && m68k_bus_cur->wr[p] != rp))
			return 0;
		int j = 0;
		while (j < t->nw && (t->waddr[j] & ~1u) != y)
			j++;
		if (j == t->nw) {
			if (j == SPIN_WORDS)
				return 0;
			t->waddr[j] = y;
			t->wval[j] = *(const uint16_t *)(rp + PAGE_OFS(y));
			t->nw++;
		}
		if (wr)
			t->waddr[j] |= 1;
	}
	return 1;
}

static int words_same(const struct spin_tpl *t, int written_only)
{
	for (int j = 0; j < t->nw; j++) {
		if (written_only && !(t->waddr[j] & 1))
			continue;
		const uint32_t y = t->waddr[j] & ~1u, p = y >> M68K_PAGE_BITS;
		const uint8_t *rp = m68k_bus_cur->rd[p];
		if (!rp || ((t->waddr[j] & 1) && m68k_bus_cur->wr[p] != rp) ||
		    (!((t->mmask >> j) & 1) && *(const uint16_t *)(rp + PAGE_OFS(y)) != t->wval[j]))
			return 0;
	}
	return 1;
}

// a memory counter's value (its words on plain RAM pages: words_same / add_words checked them)
static uint32_t mcnt_rd(uint32_t a, int sz)
{
	const uint32_t y = a & 0xffffff;
	const uint32_t hi = *(const uint16_t *)(m68k_bus_cur->rd[y >> M68K_PAGE_BITS] + PAGE_OFS(y));
	if (sz == 2)
		return hi;
	const uint32_t z = (a + 2) & 0xffffff;
	return hi << 16 | *(const uint16_t *)(m68k_bus_cur->rd[z >> M68K_PAGE_BITS] + PAGE_OFS(z));
}
static void mcnt_wr(uint32_t a, int sz, uint32_t v)
{
	const uint32_t y = a & 0xffffff;
	if (sz == 2) {
		*(uint16_t *)(m68k_bus_cur->wr[y >> M68K_PAGE_BITS] + PAGE_OFS(y)) = (uint16_t)v;
		return;
	}
	const uint32_t z = (a + 2) & 0xffffff;
	*(uint16_t *)(m68k_bus_cur->wr[y >> M68K_PAGE_BITS] + PAGE_OFS(y)) = (uint16_t)(v >> 16);
	*(uint16_t *)(m68k_bus_cur->wr[z >> M68K_PAGE_BITS] + PAGE_OFS(z)) = (uint16_t)v;
}

static int run(int cycles, int done)
{
	rbx_m68k_next_base = done;   // (m68k_cycles_run counts the slice from its start, as one m68k_execute would)
	return m68k_execute(cycles);
}

// ---- a turn from pc kept as its template ----
// runs up to two turns an instruction at a time (the first may start with registers the loop sets); 1: t holds the
// last one. *done: the slice's cycles, advanced by what ran
static int spin_build(struct spin_tpl *t, uint32_t pc, uint16_t key, int budget, int *done)
{
	if (CPU_STOPPED || CPU_INT_CYCLES)
		return 0;
	uint32_t r0[16];
	memcpy(r0, REG_DA, sizeof(r0));
	uint32_t sr0 = (uint32_t)m68ki_get_sr();
	for (int turn = 0; turn < 2; turn++) {
		uint16_t touch = 0;
		uint8_t ccount[8] = { 0 }, ckind[8] = { 0 }, dirty = 0;
		int8_t cd[8] = { 0 };
		int cyc = 0, mcount = 0;
		uint16_t other = 0;   // (words some other operand touched)
		t->nw = 0;
		t->msz = 0;
		t->mmask = 0;
		for (int n = 0;; n++) {
			const uint32_t cur = REG_PC & 0xffffff;
			spin_dec_t d;
			if (n == SPIN_STEPS || !spin_dec(cur, &d) || !code_ok(cur, d.len) || (d.fuse & dirty))
				return 0;
			for (int i = 0; i < d.nea; i++) {
				const int nw0 = t->nw;
				const uint32_t a = ea_addr(&d.ea[i]);
				if (!add_words(t, a, d.ea[i].sz, d.ea[i].wr))
					return 0;
				// (the words of this operand among waddr)
				uint16_t mine = (uint16_t)(((1u << t->nw) - 1) & ~((1u << nw0) - 1));
				for (int j = 0; j < nw0; j++) {
					const uint32_t y = t->waddr[j] & ~1u;
					if (y >= (a & ~1u) && y <= ((a + d.ea[i].sz - 1) & ~1u))
						mine |= (uint16_t)(1u << j);
				}
				if (d.cnt == 3) {
					if (mcount++ || (a & 1))
						return 0;
					t->maddr = a;
					t->msz = d.csz;
					t->mcd = d.cd;
					t->mmask = mine;
				} else
					other |= mine;
			}
			touch |= d.touch;
			if (d.cnt == 1 || d.cnt == 2) {   // (a register counter)
				ccount[d.creg]++;
				ckind[d.creg] = d.cnt == 2 ? 8 : d.csz;
				cd[d.creg] = d.cd;
			}
			dirty = (uint8_t)((dirty & ~d.fset) | (d.cnt == 1 || d.cnt == 3 ? 15 : 0));   // (flags a counter set, never looked at)
			const int c = run(1, *done);
			cyc += c;
			*done += c;
			if ((REG_PC & 0xffffff) == pc)
				break;
			if (*done >= budget || cyc > SPIN_TURN_MAX)
				return 0;
		}
		// the turn's end against its start: the same but for counters, and its stores wrote what was there
		const uint32_t sr1 = (uint32_t)m68ki_get_sr();
		int ok = dirty == 0 && sr1 == sr0 && words_same(t, 1) && !CPU_STOPPED;
		if (ok && t->msz) {   // (the memory counter: stepped once, by nothing else, not wrapping)
			const uint32_t m = t->msz == 2 ? 0xffffu : 0xffffffffu;
			const uint32_t v1 = mcnt_rd(t->maddr, t->msz);
			uint32_t w0 = 0;
			for (int j = 0, n = 0; j < t->nw; j++)   // (its words as they were before the turn, in order)
				if ((t->mmask >> j) & 1)
					w0 = n++ ? w0 << 16 | t->wval[j] : t->wval[j];
			const int64_t u = (int64_t)w0 + t->mcd;
			ok = !(other & t->mmask) && __builtin_popcount(t->mmask) == t->msz / 2 && u >= 0 && u <= (int64_t)m &&
			     v1 == (uint32_t)u;
		}
		t->nr = t->nc = 0;
		for (int r = 0; r < 16 && ok; r++) {
			const int cnt = r < 8 && ccount[r];
			if (!((touch >> r) & 1) && !cnt)
				continue;
			const uint32_t v0 = r0[r], v1 = REG_DA[r];
			if (v1 == v0 || (touch >> r) & 1 || ccount[r] != 1) {   // (a register the turn leaves as it found it)
				if (v1 != v0 || t->nr == SPIN_REGS)
					ok = 0;
				else {
					t->rid[t->nr] = (uint8_t)r;
					t->rval[t->nr++] = v0;
				}
				continue;
			}
			// a counter: stepped once, by nothing else, not wrapping (addq / subq) or going round again (dbf)
			if (t->nc == 2) {
				ok = 0;
				break;
			}
			if (ckind[r] == 8)
				ok = (v0 & 0xffff) != 0 && v1 == ((v0 & ~0xffffu) | ((v0 - 1) & 0xffff));
			else {
				const uint32_t m = ckind[r] == 1 ? 0xffu : ckind[r] == 2 ? 0xffffu : 0xffffffffu;
				const int64_t u = (int64_t)(v0 & m) + cd[r];
				ok = u >= 0 && u <= (int64_t)m && v1 == ((v0 & ~m) | ((uint32_t)u & m));
			}
			t->creg[t->nc] = (uint8_t)r;
			t->ckind[t->nc] = ckind[r];
			t->cd[t->nc++] = cd[r];
		}
		if (ok) {
			t->pc = pc;
			t->key = key;
			t->sr = (uint16_t)sr0;
			t->cyc = cyc;
			t->state = T_VALID;
			t->fails = t->backoff = 0;
			return 1;
		}
		if (*done >= budget)
			return 0;
		memcpy(r0, REG_DA, sizeof(r0));
		sr0 = sr1;
	}
	return 0;
}

static int spin_match(const struct spin_tpl *t)
{
	for (int i = 0; i < t->nr; i++)
		if (REG_DA[t->rid[i]] != t->rval[i])
			return 0;
	return !CPU_STOPPED && (uint32_t)m68ki_get_sr() == t->sr && words_same(t, 0);
}

// whole turns skipped (registers and cycles as running them); *cut: fewer than fit (a counter's limit)
static int spin_skip(const struct spin_tpl *t, int left, int *cut)
{
	// (the rest, run with Musashi, keeps at least one instruction: it ends where running every turn ends)
	const int fit = (left - (int)CPU_INT_CYCLES - 1) / t->cyc;
	int64_t k = fit;
	for (int i = 0; i < t->nc; i++) {
		const uint32_t v = REG_DA[t->creg[i]];
		int64_t lim;
		if (t->ckind[i] == 8)
			lim = v & 0xffff;
		else {
			const uint32_t m = t->ckind[i] == 1 ? 0xffu : t->ckind[i] == 2 ? 0xffffu : 0xffffffffu;
			lim = t->cd[i] > 0 ? (int64_t)(m - (v & m)) / t->cd[i] : (int64_t)(v & m) / -t->cd[i];
		}
		if (lim < k)
			k = lim;
	}
	if (t->msz) {
		const uint32_t m = t->msz == 2 ? 0xffffu : 0xffffffffu, v = mcnt_rd(t->maddr, t->msz);
		const int64_t lim = t->mcd > 0 ? (int64_t)(m - v) / t->mcd : (int64_t)v / -t->mcd;
		if (lim < k)
			k = lim;
	}
	*cut = k < fit;
	if (k <= 0)
		return 0;
	for (int i = 0; i < t->nc; i++) {
		const uint32_t v = REG_DA[t->creg[i]];
		const uint32_t m = t->ckind[i] == 1 ? 0xffu : t->ckind[i] == 2 || t->ckind[i] == 8 ? 0xffffu : 0xffffffffu;
		REG_DA[t->creg[i]] = (v & ~m) | ((v + (uint32_t)(t->cd[i] * k)) & m);
	}
	if (t->msz)
		mcnt_wr(t->maddr, t->msz, mcnt_rd(t->maddr, t->msz) + (uint32_t)(t->mcd * k));
	return (int)k * t->cyc;
}

// an instruction that may run alone: no device behind its operands (a device may end the slice)
static int plain(uint32_t pc)
{
	spin_dec_t d;
	if (!spin_dec(pc, &d))
		return 0;
	for (int i = 0; i < d.nea; i++) {
		const uint32_t a = ea_addr(&d.ea[i]), p = a >> M68K_PAGE_BITS;
		const uint32_t q = ((a + d.ea[i].sz - 1) & 0xffffff) >> M68K_PAGE_BITS;
		if ((d.ea[i].sz > 1 && (a & 1)) || !m68k_bus_cur->rd[p] || !m68k_bus_cur->rd[q] ||
		    (d.ea[i].wr && (!m68k_bus_cur->wr[p] || !m68k_bus_cur->wr[q])))
			return 0;
	}
	return 1;
}

void m68k_spin_clear(m68k_spin_t *s)
{
	if (s->t)
		memset(s->t, 0, sizeof(struct spin_tpl) * SPIN_SETS * SPIN_WAYS);
}

void m68k_spin_free(m68k_spin_t *s)
{
	free(s->t);
	s->t = NULL;
}

int m68k_spin_execute(m68k_spin_t *s, int budget, uint16_t key)
{
	if (!s->t && !(s->t = (struct spin_tpl *)calloc(SPIN_SETS * SPIN_WAYS, sizeof(struct spin_tpl))))
		return m68k_execute(budget);
	int done = 0, walk = -1;   // walk: -1 at the slice's start, then the instructions run alone after a cut skip
	while (done < budget) {
		const uint32_t pc = REG_PC & 0xffffff;
		struct spin_tpl *set = &s->t[SPIN_WAYS * (((pc >> 1) ^ (pc >> 6) ^ key) & (SPIN_SETS - 1))], *t = NULL;
		for (int w = 0; w < SPIN_WAYS; w++)
			if (set[w].state != T_EMPTY && set[w].pc == pc && set[w].key == key)
				t = &set[w];
		if (walk != 0 && t && t->state == T_VALID && spin_match(t)) {
			int cut;
			done += spin_skip(t, budget - done, &cut);
			if (!cut)
				break;
			walk = 0;   // (a counter ran out: its loop ends soon, maybe into another)
			continue;
		}
		if (walk < 0) {
			if (t && t->backoff) {
				t->backoff--;
				break;
			}
			if (!t) {   // (a new pc: in place of one that failed)
				for (int w = 0; w < SPIN_WAYS && !t; w++)
					if (set[w].state == T_EMPTY)
						t = &set[w];
				for (int w = 0; w < SPIN_WAYS && !t; w++)
					if (set[w].state == T_FAIL)
						t = &set[w];
			}
			struct spin_tpl n;
			if (spin_build(&n, pc, key, budget, &done)) {
				if (t)
					*t = n;
				else
					set[SPIN_WAYS - 1] = n;
				continue;
			}
			if (t) {
				if (t->state == T_EMPTY || t->pc != pc || t->key != key) {
					t->pc = pc;
					t->key = key;
					t->state = T_FAIL;
					t->fails = 0;
				}
				t->fails = (uint8_t)(t->fails < 6 ? t->fails + 1 : 6);
				t->backoff = (uint8_t)((1u << t->fails) - 1);
			}
			break;
		}
		if (walk == SPIN_WALK || !plain(pc))
			break;
		done += run(1, done);
		walk++;
	}
	return done < budget ? done + run(budget - done, done) : done;
}
