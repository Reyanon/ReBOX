/*  Motorola 6800 / 6802 / 6808 CPU, ported to C from MAME src/devices/cpu/m6800 (m6800.cpp, 6800ops.hxx;
    BSD-3-Clause, copyright-holders: Aaron Giles; "m6800: Portable 6800 class emulator", John Butler, Haruki Ikeda).
    Flag macros, opcode handlers and the cycles_6800 table are MAME's. Interrupt lines are sampled before every
    instruction; after CLI (with I set before) and TAP one more instruction runs first, as in MAME.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../cheat_cpu.h"
#include "m6800.h"

#include <string.h>

static inline uint8_t RM(m6800_t *c, uint16_t a)
{
	const uint8_t *p = c->rpage[a >> 8];
	return p ? p[a & 0xff] : c->read(c->ctx, a);
}

static inline void WM(m6800_t *c, uint16_t a, uint8_t v)
{
	uint8_t *p = c->wpage[a >> 8];
	if (p)
		p[a & 0xff] = v;
	else
		c->write(c->ctx, a, v);
}

static inline uint16_t RM16(m6800_t *c, uint16_t a)
{
	uint16_t h = RM(c, a);
	return (uint16_t)(h << 8 | RM(c, (uint16_t)(a + 1)));
}

static inline void WM16(m6800_t *c, uint16_t a, uint16_t v)
{
	WM(c, a, (uint8_t)(v >> 8));
	WM(c, (uint16_t)(a + 1), (uint8_t)v);
}

#define XX 4   // illegal opcode unknown cycle count
static const uint8_t cycles_6800[256] = {
	/* 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F */
	  XX, 2,XX,XX,XX,XX, 2, 2, 4, 4, 2, 2, 2, 2, 2, 2,
	   2, 2,XX,XX,XX,XX, 2, 2,XX, 2,XX, 2,XX,XX,XX,XX,
	   4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
	   4, 4, 4, 4, 4, 4, 4, 4,XX, 5,XX,10,XX,XX, 9,12,
	   2,XX,XX, 2, 2,XX, 2, 2, 2, 2, 2,XX, 2, 2,XX, 2,
	   2,XX,XX, 2, 2,XX, 2, 2, 2, 2, 2,XX, 2, 2,XX, 2,
	   7,XX,XX, 7, 7,XX, 7, 7, 7, 7, 7,XX, 7, 7, 4, 7,
	   6,XX,XX, 6, 6,XX, 6, 6, 6, 6, 6,XX, 6, 6, 3, 6,
	   2, 2, 2,XX, 2, 2, 2, 3, 2, 2, 2, 2, 3, 8, 3, 4,
	   3, 3, 3,XX, 3, 3, 3, 4, 3, 3, 3, 3, 4, 6, 4, 5,
	   5, 5, 5,XX, 5, 5, 5, 6, 5, 5, 5, 5, 6, 8, 6, 7,
	   4, 4, 4,XX, 4, 4, 4, 5, 4, 4, 4, 4, 5, 9, 5, 6,
	   2, 2, 2,XX, 2, 2, 2, 3, 2, 2, 2, 2,XX,XX, 3, 4,
	   3, 3, 3,XX, 3, 3, 3, 4, 3, 3, 3, 3,XX,XX, 4, 5,
	   5, 5, 5,XX, 5, 5, 5, 6, 5, 5, 5, 5,XX,XX, 6, 7,
	   4, 4, 4,XX, 4, 4, 4, 5, 4, 4, 4, 4,XX,XX, 5, 6
};
#undef XX

#define PC c->pc
#define S  c->s
#define X  c->x
#define A  c->a
#define B  c->b
#define CC c->cc

#define IMMBYTE()  RM(c, PC++)
#define IMMWORD()  (PC += 2, RM16(c, (uint16_t)(PC - 2)))
#define PUSHBYTE(v) do { WM(c, S, (uint8_t)(v)); --S; } while (0)
#define PUSHWORD(v) do { uint16_t w_ = (uint16_t)(v); WM(c, S, (uint8_t)w_); --S; WM(c, S, (uint8_t)(w_ >> 8)); --S; } while (0)
#define PULLBYTE()  (++S, RM(c, S))

#define CLR_HNZVC   CC &= 0xd0
#define CLR_NZV     CC &= 0xf1
#define CLR_NZVC    CC &= 0xf0
#define CLR_Z       CC &= 0xfb
#define CLR_C       CC &= 0xfe

#define SEC CC |= 0x01
#define CLC CC &= 0xfe
#define SEZ CC |= 0x04
#define SEV CC |= 0x02
#define CLV CC &= 0xfd
#define SEI CC |= 0x10
#define CLI CC &= ~0x10

#define SET_Z(a)        if (!(a)) SEZ
#define SET_Z8(a)       SET_Z((uint8_t)(a))
#define SET_Z16(a)      SET_Z((uint16_t)(a))
#define SET_N8(a)       CC |= (uint8_t)(((a) & 0x80) >> 4)
#define SET_N16(a)      CC |= (uint8_t)(((a) & 0x8000) >> 12)
#define SET_H(a,b,r)    CC |= (uint8_t)((((a) ^ (b) ^ (r)) & 0x10) << 1)
#define SET_C8(a)       CC |= (uint8_t)(((a) & 0x100) >> 8)
#define SET_V8(a,b,r)   CC |= (uint8_t)((((a) ^ (b) ^ (r) ^ ((r) >> 1)) & 0x80) >> 6)
#define SET_NZ8(a)      { SET_N8(a); SET_Z8(a); }
#define SET_NZ16(a)     { SET_N16(a); SET_Z16(a); }
#define SET_FLAGS8(a,b,r) { SET_N8(r); SET_Z8(r); SET_V8(a,b,r); SET_C8(r); }
#define SET_FLAGS8I(a)  { if ((uint8_t)(a) == 0) CC |= 0x04; else if ((uint8_t)(a) == 0x80) CC |= 0x0a; else if ((a) & 0x80) CC |= 0x08; }
#define SET_FLAGS8D(a)  { if ((uint8_t)(a) == 0) CC |= 0x04; else if ((uint8_t)(a) == 0x7f) CC |= 0x02; else if ((a) & 0x80) CC |= 0x08; }
#define NXORV  ((CC & 0x08) ^ ((CC & 0x02) << 2))
#define NXORC  ((CC & 0x08) ^ ((CC & 0x01) << 3))

static void enter_interrupt(m6800_t *c, uint16_t vec)
{
	if (c->wai) {
		c->cycles += 4;
		c->wai = 0;
	} else {
		PUSHWORD(PC);
		PUSHWORD(X);
		PUSHBYTE(A);
		PUSHBYTE(B);
		PUSHBYTE(CC);
		c->cycles += 12;
	}
	SEI;
	PC = RM16(c, vec);
}

// MAME check_irq_lines; 1 = an interrupt was taken
static int check_irq(m6800_t *c)
{
	if (c->nmi_pending) {
		c->nmi_pending = 0;
		enter_interrupt(c, 0xfffc);
		return 1;
	}
	if (c->irq_line && !(CC & 0x10)) {
		enter_interrupt(c, 0xfff8);
		return 1;
	}
	return 0;
}

// read-modify-write group, 8-bit value in, result out (TST and JMP handled by the caller)
static uint8_t rmw8(m6800_t *c, int sub, uint8_t t)
{
	uint16_t r;
	switch (sub) {
	case 0x0: r = (uint16_t)(0u - t); CLR_NZVC; SET_FLAGS8(0, t, r); return (uint8_t)r;   // NEG
	case 0x3: t = (uint8_t)~t; CLR_NZV; SET_NZ8(t); SEC; return t;                        // COM
	case 0x4: CLR_NZVC; CC |= (t & 0x01); t >>= 1; SET_Z8(t); if (NXORC) SEV; return t;   // LSR
	case 0x6: {                                                                          // ROR
		uint8_t rr = (uint8_t)((CC & 0x01) << 7);
		CLR_NZVC; CC |= (t & 0x01);
		rr |= t >> 1; SET_NZ8(rr);
		if (NXORC) SEV;
		return rr;
	}
	case 0x7: CLR_NZVC; CC |= (t & 0x01); t >>= 1; t |= (uint8_t)((t & 0x40) << 1); SET_NZ8(t); if (NXORC) SEV; return t;   // ASR
	case 0x8: r = (uint16_t)(t << 1); CLR_NZVC; SET_FLAGS8(t, t, r); return (uint8_t)r;   // ASL
	case 0x9: r = (uint16_t)(CC & 0x01); r |= (uint16_t)(t << 1); CLR_NZVC; SET_FLAGS8(t, t, r); return (uint8_t)r;   // ROL
	case 0xa: --t; CLR_NZV; SET_FLAGS8D(t); return t;   // DEC
	case 0xc: ++t; CLR_NZV; SET_FLAGS8I(t); return t;   // INC
	case 0xf: CLR_NZVC; SEZ; return 0;                  // CLR
	}
	return t;
}

static void execute(m6800_t *c, uint8_t op)
{
	uint16_t t, r, ea;
	if (op >= 0x40 && op < 0x80) {
		int sub = op & 15;
		if (sub == 0x1 || sub == 0x2 || sub == 0x5 || sub == 0xb || (op < 0x60 && sub == 0xe)) {   // illegal
			if (op >= 0x70) PC += 2;
			else if (op >= 0x60) PC += 1;
			return;
		}
		if (op < 0x60) {
			uint8_t *rg = op < 0x50 ? &A : &B;
			if (sub == 0xd) { CLR_NZVC; SET_NZ8(*rg); return; }   // TST
			*rg = rmw8(c, sub, *rg);
			return;
		}
		if (op < 0x70) { ea = (uint16_t)(X + RM(c, PC)); PC++; }
		else ea = IMMWORD();
		if (sub == 0xe) { PC = ea; return; }   // JMP
		if (sub == 0xd) { uint8_t v = RM(c, ea); CLR_NZVC; SET_NZ8(v); return; }   // TST
		uint8_t v = RM(c, ea);   // (CLR reads too)
		WM(c, ea, rmw8(c, sub, v));
		return;
	}
	if (op >= 0x80) {
		int mode = (op >> 4) & 3;   // 0 imm, 1 direct, 2 indexed, 3 extended
		int col = op & 15;
		int isb = op >= 0xc0;
		uint8_t *rg = isb ? &B : &A;
		// illegal columns
		if (col == 0x3 || (isb && (col == 0xc || col == 0xd))) {
			PC += (mode == 3 || (mode == 0 && col >= 0xc)) ? 2 : 1;
			return;
		}
		if (!isb && col == 0xd && mode == 0) {   // BSR
			int8_t o = (int8_t)IMMBYTE();
			PUSHWORD(PC);
			PC = (uint16_t)(PC + o);
			return;
		}
		int wide = col >= 0xc;   // CMPX / LDS / STS / LDX / STX / JSR
		switch (mode) {
		case 0: ea = PC; PC += wide ? 2 : 1; break;
		case 1: ea = IMMBYTE(); break;
		case 2: ea = (uint16_t)(X + RM(c, PC)); PC++; break;
		default: ea = IMMWORD(); break;
		}
		switch (col) {
		case 0x0: t = RM(c, ea); r = (uint16_t)(*rg - t); CLR_NZVC; SET_FLAGS8(*rg, t, r); *rg = (uint8_t)r; return;   // SUB
		case 0x1: t = RM(c, ea); r = (uint16_t)(*rg - t); CLR_NZVC; SET_FLAGS8(*rg, t, r); return;                      // CMP
		case 0x2: t = RM(c, ea); r = (uint16_t)(*rg - t - (CC & 0x01)); CLR_NZVC; SET_FLAGS8(*rg, t, r); *rg = (uint8_t)r; return;   // SBC
		case 0x4: *rg &= RM(c, ea); CLR_NZV; SET_NZ8(*rg); return;                         // AND
		case 0x5: { uint8_t v = (uint8_t)(*rg & RM(c, ea)); CLR_NZV; SET_NZ8(v); return; }  // BIT
		case 0x6: *rg = RM(c, ea); CLR_NZV; SET_NZ8(*rg); return;                          // LDA
		case 0x7: CLR_NZV; SET_NZ8(*rg); WM(c, ea, *rg); return;                           // STA
		case 0x8: *rg ^= RM(c, ea); CLR_NZV; SET_NZ8(*rg); return;                         // EOR
		case 0x9: t = RM(c, ea); r = (uint16_t)(*rg + t + (CC & 0x01)); CLR_HNZVC; SET_FLAGS8(*rg, t, r); SET_H(*rg, t, r); *rg = (uint8_t)r; return;   // ADC
		case 0xa: *rg |= RM(c, ea); CLR_NZV; SET_NZ8(*rg); return;                         // ORA
		case 0xb: t = RM(c, ea); r = (uint16_t)(*rg + t); CLR_HNZVC; SET_FLAGS8(*rg, t, r); SET_H(*rg, t, r); *rg = (uint8_t)r; return;   // ADD
		case 0xc: {   // CMPX (6800: N and V from the high bytes)
			uint16_t b = RM16(c, ea), d = X;
			uint16_t rh = (uint16_t)((d >> 8) - (b >> 8));
			CLR_NZV;
			SET_N8(rh & 0xff);
			SET_V8(d >> 8, b >> 8, rh);
			SET_Z16((uint16_t)(d - b));
			return;
		}
		case 0xd: PUSHWORD(PC); PC = ea; return;   // JSR
		case 0xe:
			if (isb) { X = RM16(c, ea); CLR_NZV; SET_NZ16(X); }
			else { S = RM16(c, ea); CLR_NZV; SET_NZ16(S); }
			return;
		default:
			if (isb) { CLR_NZV; SET_NZ16(X); WM16(c, ea, X); }
			else { CLR_NZV; SET_NZ16(S); WM16(c, ea, S); }
			return;
		}
	}
	if (op >= 0x20 && op < 0x30) {
		int8_t o = (int8_t)IMMBYTE();
		int f;
		switch (op & 15) {
		case 0x0: f = 1; break;
		case 0x1: f = 0; break;
		case 0x2: f = !(CC & 0x05); break;
		case 0x3: f = (CC & 0x05) != 0; break;
		case 0x4: f = !(CC & 0x01); break;
		case 0x5: f = (CC & 0x01) != 0; break;
		case 0x6: f = !(CC & 0x04); break;
		case 0x7: f = (CC & 0x04) != 0; break;
		case 0x8: f = !(CC & 0x02); break;
		case 0x9: f = (CC & 0x02) != 0; break;
		case 0xa: f = !(CC & 0x08); break;
		case 0xb: f = (CC & 0x08) != 0; break;
		case 0xc: f = !NXORV; break;
		case 0xd: f = NXORV != 0; break;
		case 0xe: f = !(NXORV || (CC & 0x04)); break;
		default:  f = NXORV || (CC & 0x04); break;
		}
		if (f)
			PC = (uint16_t)(PC + o);
		return;
	}
	switch (op) {
	case 0x01: return;   // NOP
	case 0x06: CC = A; c->irq_hold = 1; return;   // TAP
	case 0x07: A = CC; return;
	case 0x08: ++X; CLR_Z; SET_Z16(X); return;
	case 0x09: --X; CLR_Z; SET_Z16(X); return;
	case 0x0a: CLV; return;
	case 0x0b: SEV; return;
	case 0x0c: CLC; return;
	case 0x0d: SEC; return;
	case 0x0e: if (CC & 0x10) c->irq_hold = 1; CLI; return;
	case 0x0f: SEI; return;
	case 0x10: t = (uint16_t)(A - B); CLR_NZVC; SET_FLAGS8(A, B, t); A = (uint8_t)t; return;   // SBA
	case 0x11: t = (uint16_t)(A - B); CLR_NZVC; SET_FLAGS8(A, B, t); return;                    // CBA
	case 0x16: B = A; CLR_NZV; SET_NZ8(B); return;
	case 0x17: A = B; CLR_NZV; SET_NZ8(A); return;
	case 0x19: {   // DAA
		uint8_t msn = A & 0xf0, lsn = A & 0x0f;
		uint16_t cf = 0;
		if (lsn > 0x09 || (CC & 0x20)) cf |= 0x06;
		if (msn > 0x80 && lsn > 0x09) cf |= 0x60;
		if (msn > 0x90 || (CC & 0x01)) cf |= 0x60;
		t = (uint16_t)(cf + A);
		CLR_NZV;
		SET_NZ8((uint8_t)t); SET_C8(t);
		A = (uint8_t)t;
		return;
	}
	case 0x1b: t = (uint16_t)(A + B); CLR_HNZVC; SET_FLAGS8(A, B, t); SET_H(A, B, t); A = (uint8_t)t; return;   // ABA
	case 0x30: X = (uint16_t)(S + 1); return;
	case 0x31: ++S; return;
	case 0x32: A = PULLBYTE(); return;
	case 0x33: B = PULLBYTE(); return;
	case 0x34: --S; return;
	case 0x35: S = (uint16_t)(X - 1); return;
	case 0x36: PUSHBYTE(A); return;
	case 0x37: PUSHBYTE(B); return;
	case 0x39: t = PULLBYTE(); t = (uint16_t)(t << 8 | PULLBYTE()); PC = t; return;   // RTS
	case 0x3b:   // RTI
		CC = PULLBYTE();
		B = PULLBYTE();
		A = PULLBYTE();
		t = PULLBYTE(); X = (uint16_t)(t << 8 | PULLBYTE());
		t = PULLBYTE(); PC = (uint16_t)(t << 8 | PULLBYTE());
		return;
	case 0x3e:   // WAI
		PUSHWORD(PC);
		PUSHWORD(X);
		PUSHBYTE(A);
		PUSHBYTE(B);
		PUSHBYTE(CC);
		c->wai = 1;
		return;
	case 0x3f:   // SWI
		PUSHWORD(PC);
		PUSHWORD(X);
		PUSHBYTE(A);
		PUSHBYTE(B);
		PUSHBYTE(CC);
		SEI;
		PC = RM16(c, 0xfffa);
		return;
	}
	// illegal 1-byte opcodes: nothing
}

// (the cheats' access to this CPU's program space: cheat_cpu.h)
static uint8_t cheat_rd(void *x, uint32_t a)   // (as the CPU: its direct pages, else the callbacks)
{
	m6800_t *c = (m6800_t *)x;
	const uint8_t *p = c->rpage[(a >> 8) & 0xff];
	return p ? p[a & 0xff] : c->read(c->ctx, (uint16_t)a);
}
static void cheat_wr(void *x, uint32_t a, uint8_t v)
{
	m6800_t *c = (m6800_t *)x;
	uint8_t *p = c->wpage[(a >> 8) & 0xff];
	if (p)
		p[a & 0xff] = v;
	else
		c->write(c->ctx, (uint16_t)a, v);
}

void m6800_init(m6800_t *c)
{
	rbx_cheat_offer("m6800", cheat_rd, cheat_wr, c, 1, 1);
	memset(c, 0, sizeof(*c));
}

void m6800_reset(m6800_t *c)
{
	c->cc = 0xc0;
	SEI;
	c->pc = RM16(c, 0xfffe);
	c->wai = 0;
	c->nmi_pending = 0;
	c->irq_hold = 0;
	c->cycles = 0;
}

int m6800_step(m6800_t *c)
{
	int64_t start = c->cycles;
	if (c->irq_hold)
		c->irq_hold = 0;
	else if (check_irq(c))
		return (int)(c->cycles - start);
	if (c->wai) {
		c->cycles += 4;
		return 4;
	}
	uint8_t op = RM(c, PC);
	PC++;
	execute(c, op);
	c->cycles += cycles_6800[op];
	return (int)(c->cycles - start);
}

void m6800_run_until(m6800_t *c, int64_t target)
{
	while (c->cycles < target) {
		if (c->wai && !c->nmi_pending && !(c->irq_line && !(CC & 0x10))) {
			c->cycles = target;
			return;
		}
		m6800_step(c);
	}
}

void m6800_set_irq(m6800_t *c, int state) { c->irq_line = state != 0; }
void m6800_set_nmi(m6800_t *c, int state)
{
	if (!c->nmi_line && state)
		c->nmi_pending = 1;
	c->nmi_line = state != 0;
}
