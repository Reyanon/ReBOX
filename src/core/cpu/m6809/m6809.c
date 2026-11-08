/*  Motorola 6809 / 6809E CPU, ported to C from MAME src/devices/cpu/m6809 (m6809.cpp, m6809.lst, base6x09.lst,
    m6809inl.h; BSD-3-Clause, copyright-holders: Nathan Woods; "Portable 6809 emulator", John Butler).
    MAME's state-machine core (generated from the .lst files) is rewritten here as one switch per instruction. Each
    MAME memory access / dummy_read_opcode_arg / dummy_vma / eat costs one clock here too, so instruction timings
    are those of MAME. Dummy (VMA and opcode-argument) reads are counted but not sent to the bus.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "m6809.h"

#include <string.h>

enum { CC_C = 0x01, CC_V = 0x02, CC_Z = 0x04, CC_N = 0x08, CC_I = 0x10, CC_H = 0x20, CC_F = 0x40, CC_E = 0x80 };
#define CC_NZ    (CC_N | CC_Z)
#define CC_NZV   (CC_N | CC_Z | CC_V)
#define CC_NZVC  (CC_N | CC_Z | CC_V | CC_C)
#define CC_HNZVC (CC_H | CC_N | CC_Z | CC_V | CC_C)

enum { VEC_SWI3 = 0xfff2, VEC_SWI2 = 0xfff4, VEC_FIRQ = 0xfff6, VEC_IRQ = 0xfff8, VEC_SWI = 0xfffa, VEC_NMI = 0xfffc,
       VEC_RESET = 0xfffe };
enum { W_RUN, W_CWAI, W_SYNC, W_FREERUN };

static inline uint8_t RD(m6809_t *c, uint16_t a)
{
	c->cycles++;
	const uint8_t *p = c->rpage[a >> 8];
	return p ? p[a & 0xff] : c->read(c->ctx, a);
}

static inline void WR(m6809_t *c, uint16_t a, uint8_t v)
{
	c->cycles++;
	uint8_t *p = c->wpage[a >> 8];
	if (p)
		p[a & 0xff] = v;
	else
		c->write(c->ctx, a, v);
}

#define FETCH()  RD(c, c->pc++)
static const uint8_t s_k1_xor[16] = { 0x22, 0x22, 0x82, 0x82, 0x22, 0x22, 0x82, 0x82, 0x28, 0x28, 0x88, 0x88, 0x28, 0x28, 0x88, 0x88 };
static inline uint8_t opfetch(m6809_t *c)   // (an opcode byte after a prefix)
{
	const uint16_t a = c->pc++;
	const uint8_t v = RD(c, a);
	return c->konami1 ? (uint8_t)(v ^ s_k1_xor[a & 15]) : v;
}
#define OPFETCH() opfetch(c)
#define IDLE(n)  (c->cycles += (n))
#define DREG     ((uint16_t)(c->a << 8 | c->b))
#define SETD(v)  do { uint16_t v_ = (uint16_t)(v); c->a = (uint8_t)(v_ >> 8); c->b = (uint8_t)v_; } while (0)

// MAME set_flags<uint8_t> / set_flags<uint16_t>
static inline uint8_t sf8(m6809_t *c, uint8_t mask, uint32_t a, uint32_t b, uint32_t r)
{
	c->cc &= (uint8_t)~mask;
	if (mask & CC_H) c->cc |= ((a ^ b ^ r) & 0x10) ? CC_H : 0;
	if (mask & CC_N) c->cc |= (r & 0x80) ? CC_N : 0;
	if (mask & CC_Z) c->cc |= ((uint8_t)r == 0) ? CC_Z : 0;
	if (mask & CC_V) c->cc |= ((a ^ b ^ r ^ (r >> 1)) & 0x80) ? CC_V : 0;
	if (mask & CC_C) c->cc |= (r & 0x100) ? CC_C : 0;
	return (uint8_t)r;
}

static inline uint16_t sf16(m6809_t *c, uint8_t mask, uint32_t a, uint32_t b, uint32_t r)
{
	c->cc &= (uint8_t)~mask;
	if (mask & CC_N) c->cc |= (r & 0x8000) ? CC_N : 0;
	if (mask & CC_Z) c->cc |= ((uint16_t)r == 0) ? CC_Z : 0;
	if (mask & CC_V) c->cc |= ((a ^ b ^ r ^ (r >> 1)) & 0x8000) ? CC_V : 0;
	if (mask & CC_C) c->cc |= (r & 0x10000) ? CC_C : 0;
	return (uint16_t)r;
}

static inline int cond(m6809_t *c, int n)
{
	uint8_t cc = c->cc;
	int ge = !(((cc & CC_N) >> 3) ^ ((cc & CC_V) >> 1));
	switch (n & 15) {
	case 0x0: return 1;
	case 0x1: return 0;
	case 0x2: return !(cc & (CC_Z | CC_C));
	case 0x3: return (cc & (CC_Z | CC_C)) != 0;
	case 0x4: return !(cc & CC_C);
	case 0x5: return (cc & CC_C) != 0;
	case 0x6: return !(cc & CC_Z);
	case 0x7: return (cc & CC_Z) != 0;
	case 0x8: return !(cc & CC_V);
	case 0x9: return (cc & CC_V) != 0;
	case 0xa: return !(cc & CC_N);
	case 0xb: return (cc & CC_N) != 0;
	case 0xc: return ge;
	case 0xd: return !ge;
	case 0xe: return ge && !(cc & CC_Z);
	default:  return !(ge && !(cc & CC_Z));
	}
}

// ---- addressing modes ----
static inline uint16_t ea_direct(m6809_t *c)
{
	uint16_t ea = (uint16_t)(c->dp << 8 | FETCH());
	IDLE(1);
	return ea;
}

static inline uint16_t ea_extended(m6809_t *c)
{
	uint16_t h = FETCH();
	uint16_t ea = (uint16_t)(h << 8 | FETCH());
	IDLE(1);
	return ea;
}

static uint16_t ea_indexed(m6809_t *c)
{
	uint8_t pb = FETCH();
	uint16_t *r;
	switch (pb & 0x60) {
	case 0x00: r = &c->x; break;
	case 0x20: r = &c->y; break;
	case 0x40: r = &c->u; break;
	default:   r = &c->s; break;
	}
	uint16_t t;
	if (!(pb & 0x80)) {
		t = (uint16_t)(*r + (int8_t)((pb & 0x0f) | ((pb & 0x10) ? 0xf0 : 0x00)));
		IDLE(2);
		return t;
	}
	switch (pb & 0x0f) {
	case 0x0: t = *r; (*r)++; IDLE(3); break;
	case 0x1: t = *r; *r += 2; IDLE(4); break;
	case 0x2: (*r)--; t = *r; IDLE(3); break;
	case 0x3: *r -= 2; t = *r; IDLE(4); break;
	case 0x4: t = *r; IDLE(1); break;
	case 0x5: t = (uint16_t)(*r + (int8_t)c->b); IDLE(2); break;
	case 0x6: t = (uint16_t)(*r + (int8_t)c->a); IDLE(2); break;
	case 0x8: { int8_t o = (int8_t)FETCH(); t = (uint16_t)(*r + o); IDLE(1); } break;
	case 0x9: { uint16_t h = FETCH(); h = (uint16_t)(h << 8 | FETCH()); t = (uint16_t)(*r + h); IDLE(3); } break;
	case 0xb: t = (uint16_t)(*r + DREG); IDLE(5); break;
	case 0xc: { int8_t o = (int8_t)FETCH(); t = (uint16_t)(c->pc + o); IDLE(1); } break;
	case 0xd: { uint16_t h = FETCH(); h = (uint16_t)(h << 8 | FETCH()); t = (uint16_t)(c->pc + h); IDLE(4); } break;
	case 0xf: { uint16_t h = FETCH(); t = (uint16_t)(h << 8 | FETCH()); IDLE(1); } break;
	default:  t = 0; break;
	}
	if (pb & 0x10) {   // indirect
		uint16_t h = RD(c, t);
		t = (uint16_t)(h << 8 | RD(c, (uint16_t)(t + 1)));
		IDLE(1);
	}
	return t;
}

static inline uint16_t ea_mode(m6809_t *c, int mode)
{
	switch (mode) {
	case 1: return ea_direct(c);
	case 2: return ea_indexed(c);
	case 3: return ea_extended(c);
	}
	return 0;
}

// ---- stack ----
static void push_regs(m6809_t *c, int use_s, uint8_t m)
{
	uint16_t *sp = use_s ? &c->s : &c->u;
	uint16_t other = use_s ? c->u : c->s;
	if (m & 0x80) { *sp -= 1; WR(c, *sp, (uint8_t)c->pc); *sp -= 1; WR(c, *sp, (uint8_t)(c->pc >> 8)); }
	if (m & 0x40) { *sp -= 1; WR(c, *sp, (uint8_t)other); *sp -= 1; WR(c, *sp, (uint8_t)(other >> 8)); }
	if (m & 0x20) { *sp -= 1; WR(c, *sp, (uint8_t)c->y); *sp -= 1; WR(c, *sp, (uint8_t)(c->y >> 8)); }
	if (m & 0x10) { *sp -= 1; WR(c, *sp, (uint8_t)c->x); *sp -= 1; WR(c, *sp, (uint8_t)(c->x >> 8)); }
	if (m & 0x08) { *sp -= 1; WR(c, *sp, c->dp); }
	if (m & 0x04) { *sp -= 1; WR(c, *sp, c->b); }
	if (m & 0x02) { *sp -= 1; WR(c, *sp, c->a); }
	if (m & 0x01) { *sp -= 1; WR(c, *sp, c->cc); }
}

static void pull_regs(m6809_t *c, int use_s, uint8_t m)
{
	uint16_t *sp = use_s ? &c->s : &c->u;
	uint16_t *other = use_s ? &c->u : &c->s;
	uint16_t h;
	if (m & 0x01) c->cc = RD(c, (*sp)++);
	if (m & 0x02) c->a = RD(c, (*sp)++);
	if (m & 0x04) c->b = RD(c, (*sp)++);
	if (m & 0x08) c->dp = RD(c, (*sp)++);
	if (m & 0x10) { h = RD(c, (*sp)++); c->x = (uint16_t)(h << 8 | RD(c, (*sp)++)); }
	if (m & 0x20) { h = RD(c, (*sp)++); c->y = (uint16_t)(h << 8 | RD(c, (*sp)++)); }
	if (m & 0x40) { h = RD(c, (*sp)++); *other = (uint16_t)(h << 8 | RD(c, (*sp)++)); }
	if (m & 0x80) { h = RD(c, (*sp)++); c->pc = (uint16_t)(h << 8 | RD(c, (*sp)++)); }
	IDLE(1);   // (void)read_memory(sp)
}

static void vector(m6809_t *c, uint16_t v)
{
	IDLE(1);
	uint16_t h = RD(c, v);
	c->pc = (uint16_t)(h << 8 | RD(c, (uint16_t)(v + 1)));
	IDLE(1);
}

static inline uint16_t pending(m6809_t *c)
{
	if (c->nmi_asserted)
		return VEC_NMI;
	if (!(c->cc & CC_F) && c->firq_line)
		return VEC_FIRQ;
	if (!(c->cc & CC_I) && c->irq_line)
		return VEC_IRQ;
	return 0;
}

// ---- TFR / EXG registers ----
static uint16_t rd816(m6809_t *c, int r)
{
	switch (r & 15) {
	case 0: return DREG;
	case 1: return c->x;
	case 2: return c->y;
	case 3: return c->u;
	case 4: return c->s;
	case 5: return c->pc;
	case 8: return (uint16_t)(0xff00 | c->a);
	case 9: return (uint16_t)(0xff00 | c->b);
	case 10: return (uint16_t)(c->cc << 8 | c->cc);
	case 11: return (uint16_t)(c->dp << 8 | c->dp);
	}
	return 0xffff;
}

static uint16_t rd168(m6809_t *c, int r)
{
	switch (r & 15) {
	case 10: return (uint16_t)(0xff00 | c->cc);
	case 11: return (uint16_t)(0xff00 | c->dp);
	}
	return rd816(c, r);
}

static void wrreg(m6809_t *c, int r, uint16_t v)
{
	switch (r & 15) {
	case 0: SETD(v); break;
	case 1: c->x = v; break;
	case 2: c->y = v; break;
	case 3: c->u = v; break;
	case 4: c->s = v; break;
	case 5: c->pc = v; break;
	case 8: c->a = (uint8_t)v; break;
	case 9: c->b = (uint8_t)v; break;
	case 10: c->cc = (uint8_t)v; break;
	case 11: c->dp = (uint8_t)v; break;
	}
}

// ---- read-modify-write group (0x00-0x0f, 0x40-0x7f) ----
// reg: 0 = memory at ea, 1 = A, 2 = B
static void rmw(m6809_t *c, int sub, int reg, uint16_t ea)
{
	if (sub == 0xe && reg == 0) {   // JMP
		c->pc = ea;
		return;
	}
	uint8_t t = reg == 0 ? RD(c, ea) : reg == 1 ? c->a : c->b;
	uint32_t r;
	switch (sub) {
	case 0x2:   // XNC
		if (c->cc & CC_C)
			goto com;
		/* fall through */
	case 0x0: case 0x1:   // NEG
		t = sf8(c, CC_NZVC, 0, t, 0u - t);
		break;
	case 0x3:
	com:
		c->cc &= (uint8_t)~CC_V;
		c->cc |= CC_C;
		t = (uint8_t)~t;
		sf8(c, CC_NZ, 0, t, t);
		break;
	case 0x4: case 0x5:   // LSR
		c->cc = (uint8_t)((c->cc & ~CC_C) | (t & 1));
		t >>= 1;
		sf8(c, CC_NZ, 0, t, t);
		break;
	case 0x6: {   // ROR
		uint8_t nc = t & 1;
		t = (uint8_t)((t >> 1) | ((c->cc & CC_C) ? 0x80 : 0));
		c->cc = (uint8_t)((c->cc & ~CC_C) | nc);
		sf8(c, CC_NZ, 0, t, t);
		break;
	}
	case 0x7:   // ASR
		c->cc = (uint8_t)((c->cc & ~CC_C) | (t & 1));
		t = (uint8_t)((int8_t)t >> 1);
		sf8(c, CC_NZ, 0, t, t);
		break;
	case 0x8:   // ASL
		t = sf8(c, CC_NZVC, t, t, (uint32_t)t << 1);
		break;
	case 0x9:   // ROL
		r = ((uint32_t)t << 1) | ((c->cc & CC_C) ? 1 : 0);
		c->cc = (uint8_t)((c->cc & ~CC_C) | ((t & 0x80) ? CC_C : 0));
		t = sf8(c, CC_NZV, t, t, r);
		break;
	case 0xb:   // XDEC
		c->cc = (uint8_t)((c->cc & ~CC_C) | (t ? CC_C : 0));
		/* fall through */
	case 0xa:   // DEC
		t = sf8(c, CC_NZV, t, 1, (uint32_t)t - 1);
		break;
	case 0xc:   // INC
		t = sf8(c, CC_NZV, t, 1, (uint32_t)t + 1);
		break;
	case 0xd:   // TST
		sf8(c, CC_NZV, 0, t, t);
		IDLE(reg == 0 ? 2 : 1);
		return;
	case 0xe:   // XCLR (register forms)
		c->cc = (uint8_t)((c->cc & ~CC_NZV) | CC_Z);
		t = 0;
		break;
	default:    // CLR
		c->cc = (uint8_t)((c->cc & ~CC_NZVC) | CC_Z);
		t = 0;
		break;
	}
	IDLE(1);
	if (reg == 0)
		WR(c, ea, t);
	else if (reg == 1)
		c->a = t;
	else
		c->b = t;
}

static inline uint16_t opnd16(m6809_t *c, int mode, uint16_t ea)
{
	uint16_t h;
	if (mode == 0) {
		h = FETCH();
		return (uint16_t)(h << 8 | FETCH());
	}
	h = RD(c, ea);
	return (uint16_t)(h << 8 | RD(c, (uint16_t)(ea + 1)));
}

static inline void st16(m6809_t *c, int mode, uint16_t ea, uint16_t v)
{
	if (mode == 0) {   // XST16: store-immediate (writes over the operand)
		FETCH();
		WR(c, c->pc++, (uint8_t)v);
	} else {
		WR(c, ea, (uint8_t)(v >> 8));
		WR(c, (uint16_t)(ea + 1), (uint8_t)v);
	}
	sf16(c, CC_NZV, 0, v, v);
}

static inline void st8(m6809_t *c, int mode, uint16_t ea, uint8_t v)
{
	if (mode == 0)   // XST8
		FETCH();
	else
		WR(c, ea, v);
	sf8(c, CC_NZV, 0, v, v);
}

static void sw_interrupt(m6809_t *c, uint16_t vec, int set_e, uint8_t mask_if)
{
	if (set_e)
		c->cc |= CC_E;
	IDLE(2);
	push_regs(c, 1, 0xff);
	c->cc |= mask_if;
	vector(c, vec);
}

static void cmp16(m6809_t *c, uint16_t r, int mode)
{
	uint16_t ea = ea_mode(c, mode);
	uint16_t t = opnd16(c, mode, ea);
	sf16(c, CC_NZVC, r, t, (uint32_t)r - t);
	IDLE(1);
}

// 0x10 page; returns 1 if the second byte is not an instruction (fetch a new opcode)
static int page2(m6809_t *c)
{
	for (;;) {
		uint8_t op = OPFETCH();
		int mode = (op >> 4) & 3;
		uint16_t ea, t;
		if (op == 0x10 || op == 0x11)
			continue;
		if (op >= 0x20 && op <= 0x2f) {   // LBcc
			uint16_t h = FETCH();
			t = (uint16_t)(h << 8 | FETCH());
			IDLE(1);
			if (cond(c, op)) {
				c->pc = (uint16_t)(c->pc + t);
				IDLE(1);
			}
			return 0;
		}
		switch (op) {
		case 0x3e: sw_interrupt(c, VEC_SWI2, 0, 0); return 0;   // XSWI2
		case 0x3f: sw_interrupt(c, VEC_SWI2, 1, 0); return 0;   // SWI2
		case 0x83: case 0x93: case 0xa3: case 0xb3: cmp16(c, DREG, mode); return 0;
		case 0x8c: case 0x9c: case 0xac: case 0xbc: cmp16(c, c->y, mode); return 0;
		case 0x87: st8(c, 0, 0, c->a); return 0;
		case 0xc7: st8(c, 0, 0, c->b); return 0;
		case 0x8e: case 0x9e: case 0xae: case 0xbe:
			ea = ea_mode(c, mode);
			c->y = opnd16(c, mode, ea);
			sf16(c, CC_NZV, 0, c->y, c->y);
			return 0;
		case 0x8f: case 0x9f: case 0xaf: case 0xbf:
			ea = ea_mode(c, mode);
			st16(c, mode, ea, c->y);
			return 0;
		case 0xce: case 0xde: case 0xee: case 0xfe:
			ea = ea_mode(c, mode);
			c->s = opnd16(c, mode, ea);
			sf16(c, CC_NZV, 0, c->s, c->s);
			c->lds_encountered = 1;
			return 0;
		case 0xcf: case 0xdf: case 0xef: case 0xff:
			ea = ea_mode(c, mode);
			st16(c, mode, ea, c->s);
			return 0;
		case 0xc3: case 0xd3: case 0xe3: case 0xf3:   // XADD16 (flags only)
			ea = ea_mode(c, mode);
			t = opnd16(c, mode, ea);
			sf16(c, CC_NZVC, DREG, t, (uint32_t)DREG + t);
			IDLE(1);
			return 0;
		}
		return 1;
	}
}

static int page3(m6809_t *c)
{
	for (;;) {
		uint8_t op = OPFETCH();
		int mode = (op >> 4) & 3;
		uint16_t ea, t;
		switch (op) {
		case 0x10: case 0x11: continue;
		case 0x3e: sw_interrupt(c, VEC_FIRQ, 0, 0); return 0;   // XFIRQ
		case 0x3f: sw_interrupt(c, VEC_SWI3, 1, 0); return 0;   // SWI3
		case 0x83: case 0x93: case 0xa3: case 0xb3: cmp16(c, c->u, mode); return 0;
		case 0x8c: case 0x9c: case 0xac: case 0xbc: cmp16(c, c->s, mode); return 0;
		case 0x87: st8(c, 0, 0, c->a); return 0;
		case 0xc7: st8(c, 0, 0, c->b); return 0;
		case 0x8f: st16(c, 0, 0, c->x); return 0;
		case 0xcf: st16(c, 0, 0, c->u); return 0;
		case 0xc3: case 0xd3: case 0xe3: case 0xf3:
			ea = ea_mode(c, mode);
			t = opnd16(c, mode, ea);
			sf16(c, CC_NZVC, c->u, t, (uint32_t)c->u + t);
			IDLE(1);
			return 0;
		}
		return 1;
	}
}

static void execute(m6809_t *c)
{
	uint8_t op;
	uint16_t ea, t;
fetch:
	if (c->read_op) {   // LIC asserted during the first opcode byte
		c->cycles++;
		op = c->read_op(c->ctx, c->pc);
	} else {
		op = RD(c, c->pc);
		if (c->konami1)
			op ^= s_k1_xor[c->pc & 15];
	}
	c->pc++;
	if (op < 0x80) {
		switch (op >> 4) {
		case 0x0: rmw(c, op & 15, 0, ea_direct(c)); return;
		case 0x4: rmw(c, op & 15, 1, 0); return;
		case 0x5: rmw(c, op & 15, 2, 0); return;
		case 0x6: rmw(c, op & 15, 0, ea_indexed(c)); return;
		case 0x7: rmw(c, op & 15, 0, ea_extended(c)); return;
		case 0x2: {
			int8_t o = (int8_t)FETCH();
			IDLE(1);
			if (cond(c, op))
				c->pc = (uint16_t)(c->pc + o);
			return;
		}
		}
		switch (op) {
		case 0x10: if (page2(c)) goto fetch; return;
		case 0x11: if (page3(c)) goto fetch; return;
		case 0x12: case 0x1b: IDLE(1); return;   // NOP
		case 0x13:   // SYNC
			IDLE(1);
			if (c->nmi_asserted || c->firq_line || c->irq_line)
				IDLE(1);
			else
				c->wait = W_SYNC;
			return;
		case 0x14: case 0x15: c->wait = W_FREERUN; return;
		case 0x16: {   // LBRA
			uint16_t h = FETCH();
			t = (uint16_t)(h << 8 | FETCH());
			IDLE(1);
			c->pc = (uint16_t)(c->pc + t);
			IDLE(1);
			return;
		}
		case 0x17: {   // LBSR
			uint16_t h = FETCH();
			t = (uint16_t)(h << 8 | FETCH());
			ea = (uint16_t)(c->pc + t);
			IDLE(4);
			c->s--; WR(c, c->s, (uint8_t)c->pc);
			c->s--; WR(c, c->s, (uint8_t)(c->pc >> 8));
			c->pc = ea;
			return;
		}
		case 0x18: {   // X18 (undocumented)
			uint8_t v = RD(c, c->pc);
			v = (uint8_t)((c->cc & v) << 1);
			c->cc = (uint8_t)(v | ((c->cc & CC_Z) >> 1));
			return;
		}
		case 0x19: {   // DAA
			uint16_t cf = 0;
			uint8_t msn = c->a & 0xf0, lsn = c->a & 0x0f;
			if (lsn > 0x09 || (c->cc & CC_H)) cf |= 0x06;
			if (msn > 0x80 && lsn > 0x09) cf |= 0x60;
			if (msn > 0x90 || (c->cc & CC_C)) cf |= 0x60;
			t = (uint16_t)(c->a + cf);
			c->cc &= (uint8_t)~CC_V;
			if (t & 0x100)
				c->cc |= CC_C;
			c->a = (uint8_t)t;
			sf8(c, CC_NZ, 0, c->a, c->a);
			IDLE(1);
			return;
		}
		case 0x1a: c->cc |= FETCH(); IDLE(1); return;   // ORCC
		case 0x38: IDLE(1); /* fall through */          // XANDCC
		case 0x1c: c->cc &= FETCH(); IDLE(1); return;   // ANDCC
		case 0x1d:   // SEX
			t = (uint16_t)(int16_t)(int8_t)c->b;
			SETD(t);
			sf16(c, CC_NZ, 0, t, t);
			IDLE(1);
			return;
		case 0x1e: {   // EXG
			uint8_t p = FETCH();
			uint16_t r1, r2;
			if (p & 0x80) { r1 = rd816(c, p >> 4); r2 = rd816(c, p); }
			else { r1 = rd168(c, p >> 4); r2 = rd168(c, p); }
			wrreg(c, p, r1);
			wrreg(c, p >> 4, r2);
			IDLE(6);
			return;
		}
		case 0x1f: {   // TFR
			uint8_t p = FETCH();
			wrreg(c, p, rd816(c, p >> 4));
			if ((p & 0x0f) == 4)
				c->lds_encountered = 1;
			IDLE(4);
			return;
		}
		case 0x30: c->x = ea_indexed(c); sf16(c, CC_Z, 0, c->x, c->x); IDLE(1); return;
		case 0x31: c->y = ea_indexed(c); sf16(c, CC_Z, 0, c->y, c->y); IDLE(1); return;
		case 0x32: c->s = ea_indexed(c); c->lds_encountered = 1; IDLE(1); return;
		case 0x33: c->u = ea_indexed(c); IDLE(1); return;
		case 0x34: { uint8_t m = FETCH(); IDLE(3); push_regs(c, 1, m); return; }   // PSHS
		case 0x35: { uint8_t m = FETCH(); IDLE(2); pull_regs(c, 1, m); return; }   // PULS
		case 0x36: { uint8_t m = FETCH(); IDLE(3); push_regs(c, 0, m); return; }   // PSHU
		case 0x37: { uint8_t m = FETCH(); IDLE(2); pull_regs(c, 0, m); return; }   // PULU
		case 0x39: IDLE(1); pull_regs(c, 1, 0x80); return;   // RTS
		case 0x3a: c->x = (uint16_t)(c->x + c->b); IDLE(2); return;   // ABX
		case 0x3b:   // RTI
			IDLE(1);
			c->cc = RD(c, c->s++);
			pull_regs(c, 1, (uint8_t)(((c->cc & CC_E) ? 0xff : 0x81) & ~0x01));
			return;
		case 0x3c:   // CWAI
			c->cc &= FETCH();
			IDLE(2);
			c->cc |= CC_E;
			push_regs(c, 1, 0xff);
			c->wait = W_CWAI;
			return;
		case 0x3d:   // MUL
			t = (uint16_t)(c->a * c->b);
			SETD(t);
			sf16(c, CC_Z, 0, t, t);
			c->cc = (uint8_t)((c->cc & ~CC_C) | ((t & 0x80) ? CC_C : 0));
			IDLE(10);
			return;
		case 0x3e: sw_interrupt(c, VEC_RESET, 0, 0); return;   // XRES
		case 0x3f: sw_interrupt(c, VEC_SWI, 1, CC_I | CC_F); return;   // SWI
		}
		return;   // illegal: (MAME logs it)
	}

	// 0x80-0xff: accumulator / 16-bit register group
	int mode = (op >> 4) & 3;
	int isb = op & 0x40;
	uint8_t *r8 = isb ? &c->b : &c->a;
	int col = op & 15;
	if (col == 0xd && mode == 0) {
		if (!isb) {   // BSR
			int8_t o = (int8_t)FETCH();
			ea = (uint16_t)(c->pc + o);
			IDLE(3);
			c->s--; WR(c, c->s, (uint8_t)c->pc);
			c->s--; WR(c, c->s, (uint8_t)(c->pc >> 8));
			c->pc = ea;
		} else {
			c->wait = W_FREERUN;
		}
		return;
	}
	ea = ea_mode(c, mode);
#define OPND8() (mode ? RD(c, ea) : FETCH())
	switch (col) {
	case 0x0: { uint8_t v = OPND8(); *r8 = sf8(c, CC_NZVC, *r8, v, (uint32_t)*r8 - v); return; }
	case 0x1: { uint8_t v = OPND8(); sf8(c, CC_NZVC, *r8, v, (uint32_t)*r8 - v); return; }
	case 0x2: { uint8_t v = OPND8(); *r8 = sf8(c, CC_NZVC, *r8, v, (uint32_t)*r8 - v - ((c->cc & CC_C) ? 1 : 0)); return; }
	case 0x3: {
		uint16_t d = DREG;
		t = opnd16(c, mode, ea);
		if (isb)
			d = sf16(c, CC_NZVC, d, t, (uint32_t)d + t);
		else
			d = sf16(c, CC_NZVC, d, t, (uint32_t)d - t);
		SETD(d);
		IDLE(1);
		return;
	}
	case 0x4: c->cc &= (uint8_t)~CC_V; *r8 = sf8(c, CC_NZ, 0, *r8, *r8 & OPND8()); return;
	case 0x5: c->cc &= (uint8_t)~CC_V; sf8(c, CC_NZ, 0, *r8, *r8 & OPND8()); return;
	case 0x6: *r8 = OPND8(); sf8(c, CC_NZV, 0, *r8, *r8); return;
	case 0x7: st8(c, mode, ea, *r8); return;
	case 0x8: c->cc &= (uint8_t)~CC_V; *r8 = sf8(c, CC_NZ, 0, *r8, *r8 ^ OPND8()); return;
	case 0x9: { uint8_t v = OPND8(); *r8 = sf8(c, CC_HNZVC, *r8, v, (uint32_t)*r8 + v + ((c->cc & CC_C) ? 1 : 0)); return; }
	case 0xa: c->cc &= (uint8_t)~CC_V; *r8 = sf8(c, CC_NZ, 0, *r8, *r8 | OPND8()); return;
	case 0xb: { uint8_t v = OPND8(); *r8 = sf8(c, CC_HNZVC, *r8, v, (uint32_t)*r8 + v); return; }
	case 0xc:
		t = opnd16(c, mode, ea);
		if (!isb) {   // CMPX
			sf16(c, CC_NZVC, c->x, t, (uint32_t)c->x - t);
			IDLE(1);
		} else {      // LDD
			SETD(t);
			sf16(c, CC_NZV, 0, t, t);
		}
		return;
	case 0xd:
		if (!isb) {   // JSR
			IDLE(2);
			c->s--; WR(c, c->s, (uint8_t)c->pc);
			c->s--; WR(c, c->s, (uint8_t)(c->pc >> 8));
			c->pc = ea;
		} else {      // STD
			st16(c, mode, ea, DREG);
		}
		return;
	case 0xe:
		t = opnd16(c, mode, ea);
		if (isb) c->u = t; else c->x = t;
		sf16(c, CC_NZV, 0, t, t);
		return;
	default:
		st16(c, mode, ea, isb ? c->u : c->x);
		return;
	}
#undef OPND8
}

// ---- public ----
void m6809_init(m6809_t *c)
{
	memset(c, 0, sizeof(*c));
	c->cc = CC_I | CC_F;
}

void m6809_reset(m6809_t *c)
{
	c->nmi_asserted = 0;
	c->lds_encountered = 0;
	c->wait = W_RUN;
	c->dp = 0;
	c->cc |= CC_I | CC_F;
	uint16_t h = RD(c, VEC_RESET);
	c->pc = (uint16_t)(h << 8 | RD(c, VEC_RESET + 1));
	c->cycles = 0;
}

int m6809_step(m6809_t *c)
{
	int64_t start = c->cycles;
	if (c->wait) {
		if (c->wait == W_CWAI) {
			uint16_t v = pending(c);
			if (!v) {
				c->cycles += 8;
				return 8;
			}
			if (c->nmi_asserted)
				c->nmi_asserted = 0;
			if (v == VEC_FIRQ && c->firq_hold)
				c->firq_line = c->firq_hold = 0;
			if (v == VEC_IRQ && c->irq_hold)
				c->irq_line = c->irq_hold = 0;
			c->cc |= (uint8_t)(CC_I | (v != VEC_IRQ ? CC_F : 0));
			c->wait = W_RUN;
			vector(c, v);
			return (int)(c->cycles - start);
		}
		if (c->wait == W_SYNC && (c->nmi_asserted || c->firq_line || c->irq_line)) {
			c->wait = W_RUN;
			IDLE(1);
			return 1;
		}
		c->cycles += 8;
		return 8;
	}
	uint16_t v = pending(c);
	if (v) {
		IDLE(3);
		if (v == VEC_NMI) {
			c->nmi_asserted = 0;
			c->cc |= CC_E;
			push_regs(c, 1, 0xff);
			c->cc |= CC_I | CC_F;
		} else if (v == VEC_FIRQ) {
			if (c->firq_hold)
				c->firq_line = c->firq_hold = 0;
			c->cc &= (uint8_t)~CC_E;
			push_regs(c, 1, 0x81);
			c->cc |= CC_I | CC_F;
		} else {
			if (c->irq_hold)
				c->irq_line = c->irq_hold = 0;
			c->cc |= CC_E;
			push_regs(c, 1, 0xff);
			c->cc |= CC_I;
		}
		vector(c, v);
		return (int)(c->cycles - start);
	}
	execute(c);
	return (int)(c->cycles - start);
}

void m6809_run_until(m6809_t *c, int64_t target)
{
	while (c->cycles < target) {
		if (c->wait) {
			int wake = c->wait == W_CWAI ? pending(c) != 0
			         : c->wait == W_SYNC ? (c->nmi_asserted || c->firq_line || c->irq_line) : 0;
			if (!wake) {
				c->cycles = target;
				return;
			}
		}
		m6809_step(c);
	}
}

void m6809_set_irq(m6809_t *c, int state)
{
	c->irq_line = state != 0;
	c->irq_hold = 0;
}
void m6809_hold_irq(m6809_t *c) { c->irq_line = c->irq_hold = 1; }
void m6809_set_firq(m6809_t *c, int state)
{
	c->firq_line = state != 0;
	c->firq_hold = 0;
}
void m6809_hold_firq(m6809_t *c) { c->firq_line = c->firq_hold = 1; }
void m6809_set_nmi(m6809_t *c, int state)
{
	if (state && !c->nmi_line && c->lds_encountered)
		c->nmi_asserted = 1;
	c->nmi_line = state != 0;
}
