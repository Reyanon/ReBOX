/*  SH-2 idle loops, see sh2idle.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "sh2idle.h"

#include <string.h>

static int plain_page(const sh2_t *c, uint32_t a)
{
	const uint32_t p = (a & 0x07ffffff) >> SH2_PAGE_SHIFT;
	return !(a & 3) && ((a >> 28) == 0 || (a >> 28) == 2) && c->rd[p] && c->wr[p];
}

// rounds that fit in the slice, leaving more than one round to the interpreter; 0 = none (calibrating)
static int32_t rounds(sh2_idle_loop *l, const sh2_t *c)
{
	if (!l->iter) {
		if (l->last_end == c->slice_end && l->last_icount > c->icount)
			l->iter = l->last_icount - c->icount;
		l->last_end = c->slice_end;
		l->last_icount = c->icount;
		return 0;
	}
	const int32_t k = (c->icount - l->iter - 2) / l->iter;
	return k > 0 ? k : 0;
}

static void idle_cb(sh2_t *c)
{
	sh2_idle_t *s = (sh2_idle_t *)c->idle_user;
	// (a BT / BF: pc is the loop's top; a BRA: delay is, pc the slot's address)
	const uint32_t at = c->delay ? c->delay : c->pc;
	sh2_idle_loop *l = &s->l[0];
	while (l->top != at)
		if (++l == s->l + 4)
			return;
	int32_t k;
	if (l->kind == 1) {
		if (!c->delay || (k = rounds(l, c)) == 0)
			return;
	} else if (l->kind == 2) {
		if (!c->delay)
			return;
		if (sh2_read32(c, l->cnt) != sh2_read32(c, l->flag)) {   // (the loop ends: no round measured across it)
			l->last_end = -1;
			return;
		}
		if ((k = rounds(l, c)) == 0)
			return;
	} else {
		if (c->delay)
			return;
		const uint32_t cnt = l->rc >= 0 ? c->r[l->rc] : l->cnt;
		if (!plain_page(c, cnt) || cnt == l->flag || sh2_read32(c, l->flag) != 0 || (k = rounds(l, c)) == 0)
			return;
		const uint32_t v = sh2_read32(c, cnt) + (uint32_t)k;
		sh2_write32(c, cnt, v);
		c->r[l->rx] = v;
	}
	c->icount -= k * l->iter;
	s->skipped += (int64_t)k * l->iter;
}

void sh2_idle_reset(sh2_idle_t *s, sh2_t *c, int off)
{
	memset(s, 0, sizeof(*s));
	s->off = off;
	for (int i = 0; i < 4; i++)
		c->idle_pc[i] = s->l[i].top = 1;
	c->idle = idle_cb;
	c->idle_user = s;
}

// MOV.L @(disp,PC),Rn at a: the constant
static uint32_t pc_const(sh2_t *c, uint32_t a, uint16_t op) { return sh2_read32(c, ((a + 4) & ~3u) + (uint32_t)(op & 0xff) * 4); }

static int match_count(sh2_t *c, uint32_t top, sh2_idle_loop *l)
{
	uint16_t op[8];
	for (int i = 0; i < 8; i++)
		op[i] = sh2_read16(c, top + 2u * (uint32_t)i);
	memset(l, 0, sizeof(*l));
	l->top = top;
	l->last_end = -1;
	const int lead = (op[0] & 0xf000) == 0xd000;   // (MOV.L @(disp,PC),Rc first: 8 instructions)
	const uint16_t *o = op + lead;
	if (o[6] != (lead ? 0x89f7 : 0x89f8) || (o[0] & 0xf00f) != 0x6002 || (o[5] & 0xf00f) != 0x2008 ||
	    ((o[5] >> 8) & 15) != ((o[5] >> 4) & 15))
		return 0;
	const int rx = (o[0] >> 8) & 15, rc = (o[0] >> 4) & 15, ry = (o[5] >> 8) & 15;
	if (lead && ((op[0] >> 8) & 15) != rc)
		return 0;
	int at_add = -1, at_st = -1, at_pc = -1, at_ld = -1, ra = -1;
	for (int i = 1; i <= 4; i++) {
		if (o[i] == (0x7001 | rx << 8))
			at_add = i;
		else if (o[i] == (0x2002 | rc << 8 | rx << 4))
			at_st = i;
		else if ((o[i] & 0xf000) == 0xd000) {
			at_pc = i;
			ra = (o[i] >> 8) & 15;
			l->flag = pc_const(c, top + 2u * (uint32_t)(i + lead), o[i]);
		} else if ((o[i] & 0xf00f) == 0x6002 && ((o[i] >> 8) & 15) == ry)
			at_ld = i;
	}
	if (at_add < 0 || at_st < 0 || at_pc < 0 || at_ld < 0 || at_add > at_st || at_pc > at_ld ||
	    ((o[at_ld] >> 4) & 15) != ra || rx == rc || ra == rc || ra == rx || ry == ra || ry == rx ||
	    (ry == rc && !lead) || !plain_page(c, l->flag))
		return 0;
	l->kind = 0;
	l->rx = rx;
	l->rc = lead ? -1 : rc;
	l->cnt = lead ? pc_const(c, top, op[0]) : 0;
	return 1;
}

// compare: L: MOV.L @(disp,PC),Ra / MOV.L @(disp,PC),Rb / MOV.L @Ra,Ra / MOV.L @Rb,Rb (either order) /
// CMP/EQ Ra,Rb (either order) / BT 1f / BRA out / NOP / 1: BRA L / NOP
static int match_cmp(sh2_t *c, uint32_t top, sh2_idle_loop *l)
{
	uint16_t op[10];
	for (int i = 0; i < 10; i++)
		op[i] = sh2_read16(c, top + 2u * (uint32_t)i);
	if ((op[0] & 0xf000) != 0xd000 || (op[1] & 0xf000) != 0xd000)
		return 0;
	const uint16_t ra = (op[0] >> 8) & 15, rb = (op[1] >> 8) & 15;
	const uint16_t lda = (uint16_t)(0x6002 | ra << 8 | ra << 4), ldb = (uint16_t)(0x6002 | rb << 8 | rb << 4);
	if (ra == rb || !((op[2] == lda && op[3] == ldb) || (op[2] == ldb && op[3] == lda)) ||
	    (op[4] != (0x3000 | ra << 8 | rb << 4) && op[4] != (0x3000 | rb << 8 | ra << 4)) || op[5] != 0x8901 ||
	    (op[6] & 0xf000) != 0xa000 || op[7] != 0x0009 || op[8] != 0xaff6 || op[9] != 0x0009)
		return 0;
	memset(l, 0, sizeof(*l));
	l->top = top;
	l->kind = 2;
	l->last_end = -1;
	l->cnt = pc_const(c, top, op[0]);
	l->flag = pc_const(c, top + 2, op[1]);
	return plain_page(c, l->cnt) && plain_page(c, l->flag);
}

void sh2_idle_learn(sh2_idle_t *s, sh2_t *c)
{
	if (s->off)
		return;
	const uint32_t pc = c->pc & ~1u;
	for (int i = 0; i < 4; i++)
		if (pc - s->l[i].top < (s->l[i].kind == 2 ? 20u : 16u))
			return;
	sh2_idle_loop l;
	int found = 0;
	for (uint32_t top = pc - 14; top <= pc && !found; top += 2)
		found = match_count(c, top, &l);
	for (uint32_t top = pc - 18; top <= pc && !found; top += 2)
		found = match_cmp(c, top, &l);
	if (!found) {
		for (uint32_t top = pc - 2; top <= pc && !found; top += 2)   // (spin: pc at the BRA or its slot)
			if (sh2_read16(c, top) == 0xaffe && sh2_read16(c, top + 2) == 0x0009) {
				memset(&l, 0, sizeof(l));
				l.top = top;
				l.kind = 1;
				l.last_end = -1;
				found = 1;
			}
	}
	if (!found)
		return;
	const int slot = s->next;
	s->next = (s->next + 1) & 3;
	s->l[slot] = l;
	c->idle_pc[slot] = l.top;
}
