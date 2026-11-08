/*  Hitachi HD6309 / HD6309E CPU, ported to C from MAME src/devices/cpu/m6809 (hd6309.cpp, hd6309.lst,
    base6x09.lst, m6809.cpp, m6809.h, m6809inl.h; BSD-3-Clause, copyright-holders: Nathan Woods, Tim Lindner;
    "Portable 6809 emulator", John Butler).
    MAME's state-machine core (generated from the .lst files) is rewritten here as one switch per instruction, in the
    style of cpu/m6809/m6809.c. Each MAME memory access / dummy_read_opcode_arg / dummy_vma / eat costs one clock
    here too, so instruction timings are those of MAME in both emulation and native mode. Dummy reads are counted
    but not sent to the bus.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "hd6309.h"

#include <string.h>

enum { CC_C = 0x01, CC_V = 0x02, CC_Z = 0x04, CC_N = 0x08, CC_I = 0x10, CC_H = 0x20, CC_F = 0x40, CC_E = 0x80 };
#define CC_NZ    (CC_N | CC_Z)
#define CC_NZV   (CC_N | CC_Z | CC_V)
#define CC_NZVC  (CC_N | CC_Z | CC_V | CC_C)
#define CC_HNZVC (CC_H | CC_N | CC_Z | CC_V | CC_C)

enum { VEC_ILLEGAL = 0xfff0, VEC_SWI3 = 0xfff2, VEC_SWI2 = 0xfff4, VEC_FIRQ = 0xfff6, VEC_IRQ = 0xfff8,
       VEC_SWI = 0xfffa, VEC_NMI = 0xfffc, VEC_RESET = 0xfffe };
enum { W_RUN, W_CWAI, W_SYNC };

static inline uint8_t RD(hd6309_t *c, uint16_t a)
{
	c->cycles++;
	const uint8_t *p = c->rpage[a >> 8];
	return p ? p[a & 0xff] : c->read(c->ctx, a);
}

static inline void WR(hd6309_t *c, uint16_t a, uint8_t v)
{
	c->cycles++;
	uint8_t *p = c->wpage[a >> 8];
	if (p)
		p[a & 0xff] = v;
	else
		c->write(c->ctx, a, v);
}

#define FETCH()  RD(c, c->pc++)
#define IDLE(n)  (c->cycles += (n))
#define NATIVE   (c->md & 0x01)
#define DREG     ((uint16_t)(c->a << 8 | c->b))
#define WREG     ((uint16_t)(c->e << 8 | c->f))
#define SETD(v)  do { uint16_t v_ = (uint16_t)(v); c->a = (uint8_t)(v_ >> 8); c->b = (uint8_t)v_; } while (0)
#define SETW(v)  do { uint16_t v_ = (uint16_t)(v); c->e = (uint8_t)(v_ >> 8); c->f = (uint8_t)v_; } while (0)
#define CARRY    ((c->cc & CC_C) ? 1u : 0u)

// MAME entire_state_registers / partial_state_registers
static inline uint16_t entire(hd6309_t *c) { return NATIVE ? 0x3ff : 0xff; }

// MAME set_flags<uint8_t> / set_flags<uint16_t> / set_flags<uint32_t>
static inline uint8_t sf8(hd6309_t *c, uint8_t mask, uint32_t a, uint32_t b, uint32_t r)
{
	c->cc &= (uint8_t)~mask;
	if (mask & CC_H) c->cc |= ((a ^ b ^ r) & 0x10) ? CC_H : 0;
	if (mask & CC_N) c->cc |= (r & 0x80) ? CC_N : 0;
	if (mask & CC_Z) c->cc |= ((uint8_t)r == 0) ? CC_Z : 0;
	if (mask & CC_V) c->cc |= ((a ^ b ^ r ^ (r >> 1)) & 0x80) ? CC_V : 0;
	if (mask & CC_C) c->cc |= (r & 0x100) ? CC_C : 0;
	return (uint8_t)r;
}

static inline uint16_t sf16(hd6309_t *c, uint8_t mask, uint32_t a, uint32_t b, uint32_t r)
{
	c->cc &= (uint8_t)~mask;
	if (mask & CC_N) c->cc |= (r & 0x8000) ? CC_N : 0;
	if (mask & CC_Z) c->cc |= ((uint16_t)r == 0) ? CC_Z : 0;
	if (mask & CC_V) c->cc |= ((a ^ b ^ r ^ (r >> 1)) & 0x8000) ? CC_V : 0;
	if (mask & CC_C) c->cc |= (r & 0x10000) ? CC_C : 0;
	return (uint16_t)r;
}

// single-value 32-bit form (LDQ / STQ / MULD): N and Z from r, V cleared, C untouched by MAME's math
static inline void sf32(hd6309_t *c, uint8_t mask, uint32_t r)
{
	c->cc &= (uint8_t)~mask;
	if (mask & CC_N) c->cc |= (r & 0x80000000u) ? CC_N : 0;
	if (mask & CC_Z) c->cc |= (r == 0) ? CC_Z : 0;
}

static inline int cond(hd6309_t *c, int n)
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

// 16-bit register by TFR/EXG number 0-7: D X Y U S PC W V
static uint16_t get16(hd6309_t *c, int r)
{
	switch (r & 7) {
	case 0: return DREG;
	case 1: return c->x;
	case 2: return c->y;
	case 3: return c->u;
	case 4: return c->s;
	case 5: return c->pc;
	case 6: return WREG;
	default: return c->v;
	}
}

static void set16(hd6309_t *c, int r, uint16_t v)
{
	switch (r & 7) {
	case 0: SETD(v); break;
	case 1: c->x = v; break;
	case 2: c->y = v; break;
	case 3: c->u = v; break;
	case 4: c->s = v; break;
	case 5: c->pc = v; break;
	case 6: SETW(v); break;
	default: c->v = v; break;
	}
}

// ---- addressing modes ----
static inline uint16_t ea_direct(hd6309_t *c)
{
	uint16_t ea = (uint16_t)(c->dp << 8 | FETCH());
	if (!NATIVE)
		IDLE(1);
	return ea;
}

static inline uint16_t ea_extended(hd6309_t *c)
{
	uint16_t h = FETCH();
	uint16_t ea = (uint16_t)(h << 8 | FETCH());
	if (!NATIVE)
		IDLE(1);
	return ea;
}

static inline uint16_t imm16(hd6309_t *c)
{
	uint16_t h = FETCH();
	return (uint16_t)(h << 8 | FETCH());
}

#define R4(n) case (n): case (n) + 0x20: case (n) + 0x40: case (n) + 0x60
#define R8(n) R4(n): R4((n) + 0x10)

// MAME INDEXED; returns 0 for an undefined postbyte (the caller takes the illegal-instruction trap)
static int ea_indexed(hd6309_t *c, uint16_t *ea)
{
	uint8_t pb = FETCH();
	c->opcode = pb;   // MAME reuses m_opcode for the postbyte (ireg(), add8_sets_h())
	uint16_t *r;
	switch (pb & 0x60) {
	case 0x00: r = &c->x; break;
	case 0x20: r = &c->y; break;
	case 0x40: r = &c->u; break;
	default:   r = &c->s; break;
	}
	uint16_t t, w;
	if (!(pb & 0x80)) {   // 5-bit offset
		t = (uint16_t)(*r + (int8_t)((pb & 0x0f) | ((pb & 0x10) ? 0xf0 : 0x00)));
		IDLE(2);
		*ea = t;
		return 1;
	}
	int nat = NATIVE, ind = pb & 0x10;
	switch (pb & 0x7f) {
	R4(0x00): t = *r; (*r)++; IDLE((nat && !ind) ? 2 : 3); break;
	R8(0x01): t = *r; *r += 2; IDLE((nat && !ind) ? 3 : 4); break;
	R4(0x02): (*r)--; t = *r; IDLE((nat && !ind) ? 2 : 3); break;
	R8(0x03): *r -= 2; t = *r; IDLE((nat && !ind) ? 3 : 4); break;
	R8(0x04): t = *r; IDLE(1); break;
	R8(0x05): t = (uint16_t)(*r + (int8_t)c->b); IDLE(2); break;
	R8(0x06): t = (uint16_t)(*r + (int8_t)c->a); IDLE(2); break;
	R8(0x08): { int8_t o = (int8_t)FETCH(); t = (uint16_t)(*r + o); IDLE(1); } break;
	R8(0x09): t = imm16(c); t = (uint16_t)(*r + t); IDLE(nat ? 2 : 3); break;
	R8(0x0b): t = (uint16_t)(*r + DREG); IDLE((nat && !ind) ? 3 : 5); break;
	R8(0x0c): { int8_t o = (int8_t)FETCH(); t = (uint16_t)(c->pc + o); IDLE(1); } break;
	R8(0x0d): t = imm16(c); t = (uint16_t)(c->pc + t); IDLE((nat && !ind) ? 2 : 4); break;
	case 0x1f: case 0x3f: case 0x5f: case 0x7f: t = imm16(c); IDLE(1); break;
	R8(0x07): t = (uint16_t)(*r + (int8_t)c->e); IDLE(2); break;
	R8(0x0a): t = (uint16_t)(*r + (int8_t)c->f); IDLE(2); break;
	R8(0x0e): t = (uint16_t)(*r + WREG); IDLE((nat && !ind) ? 2 : 5); break;
	case 0x0f: case 0x10: t = WREG; IDLE(1); break;
	case 0x2f: case 0x30: t = imm16(c); t = (uint16_t)(WREG + t); IDLE(nat ? 1 : 5); break;
	case 0x4f: case 0x50: t = WREG; w = (uint16_t)(t + 2); SETW(w); IDLE((nat && !ind) ? 2 : 4); break;
	case 0x6f: case 0x70: w = (uint16_t)(WREG - 2); SETW(w); t = w; IDLE((nat && !ind) ? 2 : 4); break;
	default: return 0;   // 0x92 / 0xb2 / 0xd2 / 0xf2
	}
	if (ind) {
		uint16_t h = RD(c, t);
		t = (uint16_t)(h << 8 | RD(c, (uint16_t)(t + 1)));
		IDLE(1);
	}
	*ea = t;
	return 1;
}

// mode 1 direct, 2 indexed, 3 extended; 0 = illegal indexed postbyte
static inline int ea_mode(hd6309_t *c, int mode, uint16_t *ea)
{
	switch (mode) {
	case 1: *ea = ea_direct(c); return 1;
	case 2: return ea_indexed(c, ea);
	case 3: *ea = ea_extended(c); return 1;
	}
	*ea = 0;
	return 1;
}

// ---- stack (MAME PUSH_REGISTERS / PULL_REGISTERS: 0x100 = E, 0x200 = F) ----
static void push_regs(hd6309_t *c, int use_s, uint16_t m)
{
	uint16_t *sp = use_s ? &c->s : &c->u;
	uint16_t other = use_s ? c->u : c->s;
	if (m & 0x80) { *sp -= 1; WR(c, *sp, (uint8_t)c->pc); *sp -= 1; WR(c, *sp, (uint8_t)(c->pc >> 8)); }
	if (m & 0x40) { *sp -= 1; WR(c, *sp, (uint8_t)other); *sp -= 1; WR(c, *sp, (uint8_t)(other >> 8)); }
	if (m & 0x20) { *sp -= 1; WR(c, *sp, (uint8_t)c->y); *sp -= 1; WR(c, *sp, (uint8_t)(c->y >> 8)); }
	if (m & 0x10) { *sp -= 1; WR(c, *sp, (uint8_t)c->x); *sp -= 1; WR(c, *sp, (uint8_t)(c->x >> 8)); }
	if (m & 0x08) { *sp -= 1; WR(c, *sp, c->dp); }
	if (m & 0x200) { *sp -= 1; WR(c, *sp, c->f); }
	if (m & 0x100) { *sp -= 1; WR(c, *sp, c->e); }
	if (m & 0x04) { *sp -= 1; WR(c, *sp, c->b); }
	if (m & 0x02) { *sp -= 1; WR(c, *sp, c->a); }
	if (m & 0x01) { *sp -= 1; WR(c, *sp, c->cc); }
}

static void pull_regs(hd6309_t *c, int use_s, uint16_t m)
{
	uint16_t *sp = use_s ? &c->s : &c->u;
	uint16_t *other = use_s ? &c->u : &c->s;
	uint16_t h;
	if (m & 0x01) c->cc = RD(c, (*sp)++);
	if (m & 0x02) c->a = RD(c, (*sp)++);
	if (m & 0x04) c->b = RD(c, (*sp)++);
	if (m & 0x100) c->e = RD(c, (*sp)++);
	if (m & 0x200) c->f = RD(c, (*sp)++);
	if (m & 0x08) c->dp = RD(c, (*sp)++);
	if (m & 0x10) { h = RD(c, (*sp)++); c->x = (uint16_t)(h << 8 | RD(c, (*sp)++)); }
	if (m & 0x20) { h = RD(c, (*sp)++); c->y = (uint16_t)(h << 8 | RD(c, (*sp)++)); }
	if (m & 0x40) { h = RD(c, (*sp)++); *other = (uint16_t)(h << 8 | RD(c, (*sp)++)); }
	if (m & 0x80) { h = RD(c, (*sp)++); c->pc = (uint16_t)(h << 8 | RD(c, (*sp)++)); }
	IDLE(1);
}

static void vector(hd6309_t *c, uint16_t v)
{
	IDLE(1);
	uint16_t h = RD(c, v);
	c->pc = (uint16_t)(h << 8 | RD(c, (uint16_t)(v + 1)));
	IDLE(1);
}

static inline uint16_t pending(hd6309_t *c)
{
	if (c->nmi_asserted)
		return VEC_NMI;
	if (!(c->cc & CC_F) && c->firq_line)
		return VEC_FIRQ;
	if (!(c->cc & CC_I) && c->irq_line)
		return VEC_IRQ;
	return 0;
}

// MAME SOFTWARE_INTERRUPT (SWI2, SWI3, illegal, divide by zero): I and F are left alone
static void sw_interrupt(hd6309_t *c, uint16_t vec)
{
	c->cc |= CC_E;
	IDLE(2);
	push_regs(c, 1, entire(c));
	vector(c, vec);
}

static void illegal(hd6309_t *c)
{
	c->md |= 0x40;
	sw_interrupt(c, VEC_ILLEGAL);
}

static void jsr_to(hd6309_t *c, uint16_t ea)
{
	c->s--; WR(c, c->s, (uint8_t)c->pc);
	c->s--; WR(c, c->s, (uint8_t)(c->pc >> 8));
	c->pc = ea;
}

// ---- TFR / EXG (hd6309 read_exgtfr_register / write_exgtfr_register) ----
static uint16_t rd_exg(hd6309_t *c, int r)
{
	switch (r & 15) {
	case 8: return (uint16_t)(c->a << 8 | c->a);
	case 9: return (uint16_t)(c->b << 8 | c->b);
	case 10: return (uint16_t)(c->cc << 8 | c->cc);
	case 11: return (uint16_t)(c->dp << 8 | c->dp);
	case 12: case 13: return 0;
	case 14: return (uint16_t)(c->e << 8 | c->e);
	case 15: return (uint16_t)(c->f << 8 | c->f);
	}
	return get16(c, r);
}

static void wr_exg(hd6309_t *c, int r, uint16_t v)
{
	switch (r & 15) {
	case 8: c->a = (uint8_t)(v >> 8); break;
	case 9: c->b = (uint8_t)v; break;
	case 10: c->cc = (uint8_t)v; break;
	case 11: c->dp = (uint8_t)(v >> 8); break;
	case 12: case 13: break;
	case 14: c->e = (uint8_t)(v >> 8); break;
	case 15: c->f = (uint8_t)v; break;
	default: set16(c, r, v); break;
	}
}

// ---- 8-bit read-modify-write group; reg == NULL: memory at ea ----
static void rmw8(hd6309_t *c, int sub, uint8_t *reg, uint16_t ea)
{
	if (sub == 0xe) {   // JMP
		c->pc = ea;
		return;
	}
	int nat = NATIVE;
	uint8_t t = reg ? *reg : RD(c, ea);
	int dummy = !nat;
	switch (sub) {
	case 0x0:   // NEG
		t = sf8(c, CC_NZVC, 0, t, 0u - t);
		break;
	case 0x3:   // COM
		c->cc &= (uint8_t)~CC_V;
		c->cc |= CC_C;
		t = (uint8_t)~t;
		sf8(c, CC_NZ, 0, t, t);
		break;
	case 0x4:   // LSR
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
	case 0x9: {   // ROL
		uint32_t r = ((uint32_t)t << 1) | CARRY;
		c->cc = (uint8_t)((c->cc & ~CC_C) | ((t & 0x80) ? CC_C : 0));
		t = sf8(c, CC_NZV, t, t, r);
		break;
	}
	case 0xa:   // DEC
		t = sf8(c, CC_NZV, t, 1, (uint32_t)t - 1);
		dummy = !nat || !reg;
		break;
	case 0xc:   // INC
		t = sf8(c, CC_NZV, t, 1, (uint32_t)t + 1);
		dummy = !nat || !reg;
		break;
	case 0xd:   // TST
		sf8(c, CC_NZV, 0, t, t);
		if (!nat)
			IDLE(1);
		if (!reg)
			IDLE(1);
		return;
	default:    // CLR (0xf)
		c->cc = (uint8_t)((c->cc & ~CC_NZVC) | CC_Z);
		t = 0;
		dummy = !nat || !reg;
		break;
	}
	if (dummy)
		IDLE(1);
	if (reg)
		*reg = t;
	else
		WR(c, ea, t);
}

// ---- 16-bit register read-modify-write (page 10 0x4x on D, 0x5x on W) ----
static void rmw16(hd6309_t *c, int sub, int isw)
{
	int nat = NATIVE;
	uint16_t t = isw ? WREG : DREG;
	switch (sub) {
	case 0x0:   // NEG16
		t = sf16(c, CC_NZVC, 0, t, 0u - t);
		break;
	case 0x3:   // COM16
		c->cc &= (uint8_t)~CC_V;
		c->cc |= CC_C;
		t = (uint16_t)~t;
		sf16(c, CC_NZ, 0, t, t);
		break;
	case 0x4:   // LSR16
		c->cc = (uint8_t)((c->cc & ~CC_C) | (t & 1));
		t >>= 1;
		sf16(c, CC_NZ, 0, t, t);
		break;
	case 0x6: {   // ROR16
		uint16_t nc = t & 1;
		t = (uint16_t)((t >> 1) | ((c->cc & CC_C) ? 0x8000 : 0));
		c->cc = (uint8_t)((c->cc & ~CC_C) | nc);
		sf16(c, CC_NZ, 0, t, t);
		break;
	}
	case 0x7:   // ASR16
		c->cc = (uint8_t)((c->cc & ~CC_C) | (t & 1));
		t = (uint16_t)((int16_t)t >> 1);
		sf16(c, CC_NZ, 0, t, t);
		break;
	case 0x8:   // ASL16
		t = sf16(c, CC_NZVC, t, t, (uint32_t)t << 1);
		break;
	case 0x9: {   // ROL16: MAME passes the rotated value through set_flags<uint16_t>(mask, r), so V ends up clear
		uint16_t r = (uint16_t)((t << 1) | CARRY);
		c->cc = (uint8_t)((c->cc & ~CC_C) | ((t & 0x8000) ? CC_C : 0));
		t = sf16(c, CC_NZV, 0, r, r);
		break;
	}
	case 0xa:   // DEC16 (MAME sets C too)
		t = sf16(c, CC_NZVC, t, 1, (uint32_t)t - 1);
		break;
	case 0xc:   // INC16
		t = sf16(c, CC_NZVC, t, 1, (uint32_t)t + 1);
		break;
	case 0xd:   // TST16
		sf16(c, CC_NZV, 0, t, t);
		if (!nat)
			IDLE(1);
		return;
	default:    // CLR16
		c->cc = (uint8_t)((c->cc & ~CC_NZVC) | CC_Z);
		t = 0;
		break;
	}
	if (!nat)
		IDLE(1);
	if (isw) SETW(t); else SETD(t);
}

static inline int valid_rmw_reg(int sub)
{
	return sub != 0x1 && sub != 0x2 && sub != 0x5 && sub != 0xb && sub != 0xe;
}

// ---- 8-bit / 16-bit ALU (k = low opcode nibble: 0 SUB 1 CMP 2 SBC 4 AND 5 BIT 6 LD 8 EOR 9 ADC A OR B ADD) ----
static uint8_t alu8(hd6309_t *c, int k, uint8_t d, uint8_t v)
{
	// MAME add8_sets_h(): (m_opcode & 0xfe) != 0x30 (ADDR/ADCR, but also any indexed postbyte 0x30/0x31)
	uint8_t hmask = (c->opcode & 0xfe) != 0x30 ? CC_HNZVC : CC_NZVC;
	switch (k) {
	case 0x0: case 0x1: return sf8(c, CC_NZVC, d, v, (uint32_t)d - v);
	case 0x2: return sf8(c, CC_NZVC, d, v, (uint32_t)d - v - CARRY);
	case 0x4: case 0x5: c->cc &= (uint8_t)~CC_V; return sf8(c, CC_NZ, 0, d, d & v);
	case 0x6: return sf8(c, CC_NZV, 0, v, v);
	case 0x8: c->cc &= (uint8_t)~CC_V; return sf8(c, CC_NZ, 0, d, d ^ v);
	case 0x9: return sf8(c, hmask, d, v, (uint32_t)d + v + CARRY);
	case 0xa: c->cc &= (uint8_t)~CC_V; return sf8(c, CC_NZ, 0, d, d | v);
	default:  return sf8(c, hmask, d, v, (uint32_t)d + v);
	}
}

// includes MAME's trailing cycle (native mode: none)
static uint16_t alu16(hd6309_t *c, int k, uint16_t d, uint16_t v)
{
	uint16_t r;
	switch (k) {
	case 0x0: case 0x1: r = sf16(c, CC_NZVC, d, v, (uint32_t)d - v); break;
	case 0x2: r = sf16(c, CC_NZVC, d, v, (uint32_t)d - v - CARRY); break;
	case 0x4: case 0x5: c->cc &= (uint8_t)~CC_V; return sf16(c, CC_NZ, 0, d, d & v);
	case 0x6: return sf16(c, CC_NZV, 0, v, v);
	case 0x8: c->cc &= (uint8_t)~CC_V; return sf16(c, CC_NZ, 0, d, d ^ v);
	case 0x9: r = sf16(c, CC_NZVC, d, v, (uint32_t)d + v + CARRY); break;
	case 0xa: c->cc &= (uint8_t)~CC_V; return sf16(c, CC_NZ, 0, d, d | v);
	default:  r = sf16(c, CC_NZVC, d, v, (uint32_t)d + v); break;
	}
	if (!NATIVE)
		IDLE(1);
	return r;
}

static inline int alu_writes(int k) { return k != 0x1 && k != 0x5; }

static inline uint8_t opnd8(hd6309_t *c, int mode, uint16_t ea)
{
	return mode ? RD(c, ea) : FETCH();
}

static inline uint16_t opnd16(hd6309_t *c, int mode, uint16_t ea)
{
	if (mode == 0)
		return imm16(c);
	uint16_t h = RD(c, ea);
	return (uint16_t)(h << 8 | RD(c, (uint16_t)(ea + 1)));
}

static inline void st8(hd6309_t *c, uint16_t ea, uint8_t v)
{
	WR(c, ea, v);
	sf8(c, CC_NZV, 0, v, v);
}

static inline void st16(hd6309_t *c, uint16_t ea, uint16_t v)
{
	WR(c, ea, (uint8_t)(v >> 8));
	WR(c, (uint16_t)(ea + 1), (uint8_t)v);
	sf16(c, CC_NZV, 0, v, v);
}

static void ldq(hd6309_t *c, int mode, uint16_t ea)
{
	uint8_t q[4];
	for (int i = 0; i < 4; i++)
		q[i] = mode ? RD(c, (uint16_t)(ea + i)) : FETCH();
	c->a = q[0]; c->b = q[1]; c->e = q[2]; c->f = q[3];
	sf32(c, CC_NZV, (uint32_t)q[0] << 24 | (uint32_t)q[1] << 16 | (uint32_t)q[2] << 8 | q[3]);
}

static void stq(hd6309_t *c, uint16_t ea)
{
	WR(c, ea, c->a);
	WR(c, (uint16_t)(ea + 1), c->b);
	WR(c, (uint16_t)(ea + 2), c->e);
	WR(c, (uint16_t)(ea + 3), c->f);
	sf32(c, CC_NZV, (uint32_t)c->a << 24 | (uint32_t)c->b << 16 | (uint32_t)c->e << 8 | c->f);
}

// ---- register-register ops (page 10 0x30-0x37: ADDR ADCR SUBR SBCR ANDR ORR EORR CMPR) ----
static void regreg(hd6309_t *c, int which)
{
	static const uint8_t kind[8] = { 0xb, 0x9, 0x0, 0x2, 0x4, 0xa, 0x8, 0x1 };
	int k = kind[which & 7];
	uint8_t p = FETCH();
	int dst = p & 15, src = p >> 4;
	int promote = ((p & 0x80) != 0) != ((p & 0x08) != 0);
	IDLE(1);
	if (dst < 8 || promote) {   // 16-bit
		// source: A/B -> D, E/F -> W, CC/DP/0 -> zero
		uint16_t sv;
		if (src < 8) sv = get16(c, src);
		else if (src <= 9) sv = DREG;
		else if (src >= 14) sv = WREG;
		else sv = 0;
		// destination: A/B -> D, E/F -> W, CC/DP/0 -> MAME's m_temp (holds the source value, result dropped)
		int dreg = dst < 8 ? dst : dst <= 9 ? 0 : dst >= 14 ? 6 : -1;
		uint16_t dv = dreg >= 0 ? get16(c, dreg) : sv;
		uint16_t r;
		switch (k) {
		case 0x0: case 0x1: r = sf16(c, CC_NZVC, dv, sv, (uint32_t)dv - sv); break;
		case 0x2: r = sf16(c, CC_NZVC, dv, sv, (uint32_t)dv - sv - CARRY); break;
		case 0x4: c->cc &= (uint8_t)~CC_V; r = sf16(c, CC_NZ, 0, dv, dv & sv); break;
		case 0x8: c->cc &= (uint8_t)~CC_V; r = sf16(c, CC_NZ, 0, dv, dv ^ sv); break;
		case 0x9: r = sf16(c, CC_NZVC, dv, sv, (uint32_t)dv + sv + CARRY); break;
		case 0xa: c->cc &= (uint8_t)~CC_V; r = sf16(c, CC_NZ, 0, dv, dv | sv); break;
		default:  r = sf16(c, CC_NZVC, dv, sv, (uint32_t)dv + sv); break;
		}
		if (k != 0x4 && k != 0x8 && k != 0xa && !NATIVE)   // ADD16 SUB16 CMP16 ADC16 SBC16
			IDLE(1);
		if (k != 0x1 && dreg >= 0)
			set16(c, dreg, r);
		return;
	}
	// 8-bit. MAME's read_operand() has no CC / DP / zero source (fatalerror); read them as CC, DP and 0.
	uint8_t sv, tmp = 0;
	switch (src) {
	case 8: sv = c->a; break;
	case 9: sv = c->b; break;
	case 10: sv = c->cc; break;
	case 11: sv = c->dp; break;
	case 14: sv = c->e; break;
	case 15: sv = c->f; break;
	default: sv = 0; break;
	}
	uint8_t *dp;
	switch (dst) {
	case 8: dp = &c->a; break;
	case 9: dp = &c->b; break;
	case 10: dp = &c->cc; break;
	case 11: dp = &c->dp; break;
	case 14: dp = &c->e; break;
	case 15: dp = &c->f; break;
	default:   // "0" register = MAME m_temp.b.l: the source for SUB/CMP/SBC/ADC/ADD (they load m_temp), else 0
		tmp = (k == 0x4 || k == 0x8 || k == 0xa) ? 0 : sv;
		dp = &tmp;
		break;
	}
	uint8_t r = alu8(c, k, *dp, sv);
	if (k != 0x1)
		*dp = r;
}

// ---- bit ops (page 11 0x30-0x37: BAND BIAND BOR BIOR BEOR BIEOR LDBT STBT) ----
static void bitop(hd6309_t *c, int op)
{
	uint8_t im = FETCH();
	uint16_t ea = ea_direct(c);
	uint8_t *rp;
	switch (im & 0xc0) {
	case 0x00: rp = &c->cc; break;
	case 0x40: rp = &c->a; break;
	case 0x80: rp = &c->b; break;
	default: rp = NULL; break;   // MAME m_temp.b.l
	}
	int sbit = (im >> 3) & 7, dbit = im & 7;
	if (op == 0x37) {   // STBT: register bit -> memory bit
		uint8_t rv = rp ? *rp : 0;   // MAME: stale m_temp.b.l for the undefined register 3
		uint8_t m = RD(c, ea);
		if (rv & (1 << sbit))
			m = (uint8_t)(m | (1 << dbit));
		else
			m = (uint8_t)(m & ~(1 << dbit));
		IDLE(2);
		WR(c, ea, m);
		return;
	}
	uint8_t m = RD(c, ea);
	int s = (m >> sbit) & 1;
	uint8_t *dr = rp ? rp : &m;
	int d = (*dr >> dbit) & 1;
	int res;
	switch (op) {
	case 0x30: res = d && s; break;
	case 0x31: res = d && !s; break;
	case 0x32: res = d || s; break;
	case 0x33: res = d || !s; break;
	case 0x34: res = d != s; break;
	case 0x35: res = d != !s; break;
	default:   res = s; break;   // LDBT
	}
	if (res)
		*dr = (uint8_t)(*dr | (1 << dbit));
	else
		*dr = (uint8_t)(*dr & ~(1 << dbit));
	IDLE(4);
}

// ---- MULD / DIVD / DIVQ (hd6309.cpp) ----
static void muld(hd6309_t *c, uint16_t v)
{
	uint32_t r = (uint32_t)((int32_t)(int16_t)DREG * (int32_t)(int16_t)v);
	SETD(r >> 16);
	SETW(r);
	sf32(c, CC_NZ, r);
	c->cc &= (uint8_t)~(CC_V | CC_C);
}

// returns 0 on division by zero
static int divd(hd6309_t *c, uint8_t v)
{
	int target = NATIVE ? 24 : 25;
	if (v == 0) {
		if (!NATIVE)
			target -= 2;
		IDLE(target);
		return 0;
	}
	uint16_t tmp1 = DREG;
	uint8_t tmp2 = v;
	int nsign = 0, vsign = 0;
	if (tmp1 & 0x8000) {
		tmp1 = (uint16_t)(~tmp1 + 1);
		SETD(tmp1);
		nsign = 1;
		target += 1;
	}
	if (tmp2 & 0x80) {
		tmp2 = (uint8_t)(~tmp2 + 1);
		vsign = 1;
		target += 1;
	}
	uint16_t quotient = (uint16_t)(tmp1 / tmp2);
	uint8_t remainder = (uint8_t)(tmp1 % tmp2);
	c->cc &= (uint8_t)~CC_NZVC;
	if (quotient >> 8) {   // range overflow
		c->cc |= CC_V;
		if (nsign)
			c->cc |= CC_N;
		IDLE(target - 13);
		return 1;
	}
	if (nsign)
		remainder = (uint8_t)(~remainder + 1);
	if (nsign != vsign) {
		uint16_t nq = (uint16_t)(~quotient + 1);
		if ((nq & 0x80) && !(quotient & 0x80))
			quotient = nq;
	}
	c->a = remainder;
	c->b = (uint8_t)quotient;
	if (c->b & 1)
		c->cc |= CC_C;
	if ((((quotient >> 15) ^ (c->b >> 7)) & 1) == 0) {
		if (c->b == 0) c->cc |= CC_Z;
		if (c->b & 0x80) c->cc |= CC_N;
	} else {
		c->cc |= CC_N | CC_V;
		target -= 1;
	}
	IDLE(target);
	return 1;
}

static int divq(hd6309_t *c, uint16_t v)
{
	int target = NATIVE ? 24 : 25;
	if (v == 0) {
		if (!NATIVE)
			target -= 2;
		IDLE(target);
		return 0;
	}
	uint32_t tmp1 = (uint32_t)DREG << 16 | WREG;
	uint16_t tmp2 = v;
	int nsign = 0, vsign = 0;
	if (tmp1 >> 31) {
		tmp1 = ~tmp1 + 1;
		SETD(tmp1 >> 16);
		SETW(tmp1);
		nsign = 1;
		target += 1;
	}
	if (tmp2 & 0x8000) {
		tmp2 = (uint16_t)(~tmp2 + 1);
		vsign = 1;
		target += 1;
	}
	uint32_t quotient = tmp1 / tmp2;
	uint16_t remainder = (uint16_t)(tmp1 % tmp2);
	c->cc &= (uint8_t)~CC_NZVC;
	if (quotient >> 16) {   // range overflow
		c->cc |= CC_V;
		if (nsign)
			c->cc |= CC_N;
		IDLE(target - 13);
		return 1;
	}
	if (nsign)
		remainder = (uint16_t)(~remainder + 1);
	if (nsign != vsign) {
		uint32_t nq = ~quotient + 1;
		if ((nq & 0x8000) && !(quotient & 0x8000))
			quotient = nq;
	}
	SETD(remainder);
	uint16_t w = (uint16_t)quotient;
	SETW(w);
	if (w & 1)
		c->cc |= CC_C;
	if ((((quotient >> 31) ^ (w >> 15)) & 1) == 0) {
		if (w == 0) c->cc |= CC_Z;
		if (w & 0x8000) c->cc |= CC_N;
	} else {
		c->cc |= CC_N | CC_V;
		target -= 1;
	}
	IDLE(target);
	return 1;
}

// ---- TFM: one byte per call (MAME's loop body); c->tfm stays set until W reaches 0 ----
static uint16_t *tfm_reg(hd6309_t *c, int r, uint16_t *dtmp)
{
	switch (r) {
	case 0: *dtmp = DREG; return dtmp;
	case 1: return &c->x;
	case 2: return &c->y;
	case 3: return &c->u;
	case 4: return &c->s;
	}
	return NULL;
}

static void tfm_iter(hd6309_t *c)
{
	if (WREG == 0) {
		c->tfm = 0;
		IDLE(3);
		return;
	}
	if (pending(c)) {   // TFM is abortable: restart it after the interrupt
		c->pc = (uint16_t)(c->pc - 3);
		c->tfm = 0;
		return;
	}
	int mode = c->tfm_op & 3;
	uint16_t dtmp;
	uint16_t *sr = tfm_reg(c, c->tfm_pb >> 4, &dtmp);
	if (!sr) {
		c->tfm = 0;
		illegal(c);
		return;
	}
	uint8_t v = RD(c, *sr);
	if (mode == 0 || mode == 2) (*sr)++;
	else if (mode == 1) (*sr)--;
	if (sr == &dtmp) SETD(dtmp);
	uint16_t *dr = tfm_reg(c, c->tfm_pb & 15, &dtmp);
	if (!dr) {
		c->tfm = 0;
		illegal(c);
		return;
	}
	WR(c, *dr, v);
	if (mode == 0 || mode == 3) (*dr)++;
	else if (mode == 1) (*dr)--;
	if (dr == &dtmp) SETD(dtmp);
	IDLE(1);
	SETW(WREG - 1);
	if (WREG == 0) {
		c->tfm = 0;
		IDLE(3);
	}
}

// ---- page 10 ----
static void page10(hd6309_t *c)
{
	uint8_t op = FETCH();
	c->opcode = op;
	int mode = (op >> 4) & 3;
	uint16_t ea, t;
	if (op >= 0x21 && op <= 0x2f) {   // LBcc
		t = imm16(c);
		IDLE(1);
		if (cond(c, op)) {
			c->pc = (uint16_t)(c->pc + t);
			if (!NATIVE)
				IDLE(1);
		}
		return;
	}
	if (op >= 0x30 && op <= 0x37) { regreg(c, op & 7); return; }
	switch (op) {
	case 0x38: IDLE(2); push_regs(c, 1, 0x300); return;   // PSHSW
	case 0x39: IDLE(2); pull_regs(c, 1, 0x300); return;   // PULSW
	case 0x3a: IDLE(2); push_regs(c, 0, 0x300); return;   // PSHUW
	case 0x3b: IDLE(2); pull_regs(c, 0, 0x300); return;   // PULUW
	case 0x3f: sw_interrupt(c, VEC_SWI2); return;          // SWI2
	}
	if ((op & 0xe0) == 0x40) {   // D / W register ops
		if (!valid_rmw_reg(op & 15)) { illegal(c); return; }
		rmw16(c, op & 15, op & 0x10);
		return;
	}
	if (op >= 0x80 && op <= 0xbf) {
		int col = op & 15;
		if (col == 0xd || ((col == 0x7 || col == 0xf) && mode == 0)) { illegal(c); return; }
		if (!ea_mode(c, mode, &ea)) { illegal(c); return; }
		switch (col) {
		case 0x7: st16(c, ea, WREG); return;     // STW
		case 0xf: st16(c, ea, c->y); return;     // STY
		case 0xc: t = opnd16(c, mode, ea); alu16(c, 0x1, c->y, t); return;   // CMPY
		case 0xe: t = opnd16(c, mode, ea); c->y = alu16(c, 0x6, 0, t); return; // LDY
		case 0x3: t = opnd16(c, mode, ea); alu16(c, 0x1, DREG, t); return;   // CMPD
		}
		// W: SUBW CMPW LDW ADDW; D: SBCD ANDD BITD EORD ADCD ORD
		int isw = col == 0x0 || col == 0x1 || col == 0x6 || col == 0xb;
		t = opnd16(c, mode, ea);
		uint16_t r = alu16(c, col, isw ? WREG : DREG, t);
		if (alu_writes(col)) {
			if (isw) SETW(r); else SETD(r);
		}
		return;
	}
	switch (op) {
	case 0xce: case 0xde: case 0xee: case 0xfe:   // LDS
		if (!ea_mode(c, mode, &ea)) { illegal(c); return; }
		t = opnd16(c, mode, ea);
		c->s = sf16(c, CC_NZV, 0, t, t);
		c->lds_encountered = 1;
		return;
	case 0xdf: case 0xef: case 0xff:   // STS
		if (!ea_mode(c, mode, &ea)) { illegal(c); return; }
		st16(c, ea, c->s);
		return;
	case 0xdc: case 0xec: case 0xfc:   // LDQ
		if (!ea_mode(c, mode, &ea)) { illegal(c); return; }
		ldq(c, mode, ea);
		return;
	case 0xdd: case 0xed: case 0xfd:   // STQ
		if (!ea_mode(c, mode, &ea)) { illegal(c); return; }
		stq(c, ea);
		return;
	}
	illegal(c);
}

// ---- page 11 ----
static void page11(hd6309_t *c)
{
	uint8_t op = FETCH();
	c->opcode = op;
	int mode = (op >> 4) & 3;
	uint16_t ea, t;
	if (op >= 0x30 && op <= 0x37) { bitop(c, op); return; }
	switch (op) {
	case 0x38: case 0x39: case 0x3a: case 0x3b:   // TFM
		c->tfm_pb = FETCH();
		c->tfm_op = op;
		c->tfm = 1;
		tfm_iter(c);
		return;
	case 0x3c: { uint8_t v = FETCH(); alu8(c, 0x5, c->md, v); return; }   // BITMD
	case 0x3d: { uint8_t v = FETCH(); c->md = alu8(c, 0x6, 0, v); return; }   // LDMD
	case 0x3f: sw_interrupt(c, VEC_SWI3); return;   // SWI3
	case 0x43: case 0x4a: case 0x4c: case 0x4d: case 0x4f: rmw8(c, op & 15, &c->e, 0); return;
	case 0x53: case 0x5a: case 0x5c: case 0x5d: case 0x5f: rmw8(c, op & 15, &c->f, 0); return;
	}
	if (op < 0x80) { illegal(c); return; }
	int col = op & 15;
	int isf = op & 0x40;
	int ok;
	if (!isf)
		ok = col <= 1 || col == 3 || col == 6 || col == 0xb || col == 0xc || col >= 0xd || (col == 7 && mode);
	else
		ok = col <= 1 || col == 6 || col == 0xb || (col == 7 && mode);
	if (!ok) { illegal(c); return; }
	if (!ea_mode(c, mode, &ea)) { illegal(c); return; }
	uint8_t *r8 = isf ? &c->f : &c->e;
	switch (col) {
	case 0x3: t = opnd16(c, mode, ea); alu16(c, 0x1, c->u, t); return;   // CMPU
	case 0xc: t = opnd16(c, mode, ea); alu16(c, 0x1, c->s, t); return;   // CMPS
	case 0x7: st8(c, ea, *r8); return;                                    // STE / STF
	case 0xd: {   // DIVD
		uint8_t v = opnd8(c, mode, ea);
		if (!divd(c, v)) {
			c->md |= 0x80;
			sw_interrupt(c, VEC_ILLEGAL);
		}
		return;
	}
	case 0xe:   // DIVQ
		t = opnd16(c, mode, ea);
		if (!divq(c, t)) {
			c->md |= 0x80;
			sw_interrupt(c, VEC_ILLEGAL);
		}
		return;
	case 0xf: muld(c, opnd16(c, mode, ea)); return;   // MULD
	}
	uint8_t v = opnd8(c, mode, ea);
	uint8_t r = alu8(c, col, *r8, v);   // SUB CMP LD ADD
	if (alu_writes(col))
		*r8 = r;
}

static void execute(hd6309_t *c)
{
	uint8_t op = FETCH();
	uint16_t ea, t;
	c->opcode = op;
	int nat = NATIVE;
	if (op < 0x80) {
		int sub = op & 15;
		switch (op >> 4) {
		case 0x0: case 0x6: case 0x7: {
			int m = op >> 4 == 0 ? 1 : op >> 4 == 6 ? 2 : 3;
			if (sub == 0x1 || sub == 0x2 || sub == 0x5 || sub == 0xb) {   // OIM AIM EIM TIM
				uint8_t im = FETCH();
				if (!ea_mode(c, m, &ea)) { illegal(c); return; }
				uint8_t v = RD(c, ea);
				c->cc &= (uint8_t)~CC_V;
				v = sf8(c, CC_NZ, 0, v, sub == 0x1 ? (v | im) : sub == 0x5 ? (v ^ im) : (v & im));
				if (sub == 0xb) {
					IDLE(2);
				} else {
					IDLE(1);
					WR(c, ea, v);
				}
				return;
			}
			if (!ea_mode(c, m, &ea)) { illegal(c); return; }
			rmw8(c, sub, NULL, ea);
			return;
		}
		case 0x4: case 0x5:
			if (!valid_rmw_reg(sub)) { illegal(c); return; }
			rmw8(c, sub, (op & 0x10) ? &c->b : &c->a, 0);
			return;
		case 0x2: {
			int8_t o = (int8_t)FETCH();
			IDLE(1);
			if (cond(c, op))
				c->pc = (uint16_t)(c->pc + o);
			return;
		}
		}
		switch (op) {
		case 0x10: page10(c); return;
		case 0x11: page11(c); return;
		case 0x12: if (!nat) IDLE(1); return;   // NOP
		case 0x13:   // SYNC
			IDLE(1);
			if (c->nmi_asserted || c->firq_line || c->irq_line)
				IDLE(1);
			else
				c->wait = W_SYNC;
			return;
		case 0x14: {   // SEXW
			uint16_t d = (c->e & 0x80) ? 0xffff : 0x0000;
			SETD(sf16(c, CC_N, 0, d, d));
			if (d == 0 && WREG == 0)
				c->cc |= CC_Z;
			else
				c->cc &= (uint8_t)~CC_Z;
			return;
		}
		case 0x16:   // LBRA
			t = imm16(c);
			IDLE(1);
			c->pc = (uint16_t)(c->pc + t);
			if (!nat)
				IDLE(1);
			return;
		case 0x17:   // LBSR
			t = imm16(c);
			ea = (uint16_t)(c->pc + t);
			IDLE(nat ? 2 : 4);
			jsr_to(c, ea);
			return;
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
			if (!nat)
				IDLE(1);
			return;
		}
		case 0x1a: c->cc |= FETCH(); if (!nat) IDLE(1); return;   // ORCC
		case 0x1c: c->cc &= FETCH(); if (!nat) IDLE(1); return;   // ANDCC
		case 0x1d:   // SEX
			t = (uint16_t)(int16_t)(int8_t)c->b;
			SETD(t);
			sf16(c, CC_NZ, 0, t, t);
			if (!nat)
				IDLE(1);
			return;
		case 0x1e: {   // EXG
			uint8_t p = FETCH();
			uint16_t r1 = rd_exg(c, p >> 4), r2 = rd_exg(c, p);
			wr_exg(c, p >> 4, r2);
			wr_exg(c, p, r1);
			IDLE(nat ? 3 : 6);
			return;
		}
		case 0x1f: {   // TFR
			uint8_t p = FETCH();
			wr_exg(c, p, rd_exg(c, p >> 4));
			IDLE(nat ? 2 : 4);
			return;
		}
		case 0x30: case 0x31: case 0x32: case 0x33:   // LEAX LEAY LEAS LEAU
			if (!ea_indexed(c, &ea)) { illegal(c); return; }
			if (op == 0x30) c->x = sf16(c, CC_Z, 0, ea, ea);
			else if (op == 0x31) c->y = sf16(c, CC_Z, 0, ea, ea);
			else if (op == 0x32) { c->s = ea; c->lds_encountered = 1; }
			else c->u = ea;
			IDLE(1);
			return;
		case 0x34: { uint8_t m = FETCH(); IDLE(nat ? 2 : 3); push_regs(c, 1, m); return; }   // PSHS
		case 0x35: { uint8_t m = FETCH(); IDLE(nat ? 1 : 2); pull_regs(c, 1, m); return; }   // PULS
		case 0x36: { uint8_t m = FETCH(); IDLE(nat ? 2 : 3); push_regs(c, 0, m); return; }   // PSHU
		case 0x37: { uint8_t m = FETCH(); IDLE(nat ? 1 : 2); pull_regs(c, 0, m); return; }   // PULU
		case 0x39: if (!nat) IDLE(1); pull_regs(c, 1, 0x80); return;   // RTS
		case 0x3a: c->x = (uint16_t)(c->x + c->b); if (!nat) IDLE(2); return;   // ABX
		case 0x3b:   // RTI
			IDLE(1);
			c->cc = RD(c, c->s++);
			pull_regs(c, 1, (uint16_t)(((c->cc & CC_E) ? entire(c) : 0x81) & ~0x01));
			return;
		case 0x3c:   // CWAI
			c->cc &= FETCH();
			IDLE(2);
			c->cc |= CC_E;
			push_regs(c, 1, entire(c));
			c->wait = W_CWAI;
			return;
		case 0x3d:   // MUL
			t = (uint16_t)(c->a * c->b);
			SETD(t);
			sf16(c, CC_Z, 0, t, t);
			c->cc = (uint8_t)((c->cc & ~CC_C) | ((t & 0x80) ? CC_C : 0));
			IDLE(nat ? 9 : 10);
			return;
		case 0x3f:   // SWI
			c->cc |= CC_E;
			IDLE(2);
			push_regs(c, 1, entire(c));
			c->cc |= CC_I | CC_F;
			vector(c, VEC_SWI);
			return;
		}
		illegal(c);   // 0x15 0x18 0x1b 0x38 0x3e
		return;
	}

	// 0x80-0xff: accumulator / 16-bit register group
	int mode = (op >> 4) & 3;
	int isb = op & 0x40;
	uint8_t *r8 = isb ? &c->b : &c->a;
	int col = op & 15;
	if (mode == 0) {
		if (col == 0xd) {
			if (!isb) {   // BSR
				int8_t o = (int8_t)FETCH();
				ea = (uint16_t)(c->pc + o);
				IDLE(nat ? 2 : 3);
				jsr_to(c, ea);
			} else {      // LDQ #
				ldq(c, 0, 0);
			}
			return;
		}
		if (col == 0x7 || col == 0xf) {   // 0x87 0x8f 0xc7 0xcf
			illegal(c);
			return;
		}
	}
	if (!ea_mode(c, mode, &ea)) { illegal(c); return; }
	switch (col) {
	case 0x3:   // SUBD / ADDD
		t = opnd16(c, mode, ea);
		SETD(alu16(c, isb ? 0xb : 0x0, DREG, t));
		return;
	case 0x7: st8(c, ea, *r8); return;
	case 0xc:
		t = opnd16(c, mode, ea);
		if (!isb) alu16(c, 0x1, c->x, t);       // CMPX
		else SETD(alu16(c, 0x6, 0, t));         // LDD
		return;
	case 0xd:
		if (!isb) {   // JSR (indexed: one clock less in native mode, as MAME's JSR_ind)
			IDLE((mode == 2 && nat) ? 1 : 2);
			jsr_to(c, ea);
		} else {      // STD
			st16(c, ea, DREG);
		}
		return;
	case 0xe:
		t = opnd16(c, mode, ea);
		t = alu16(c, 0x6, 0, t);
		if (isb) c->u = t; else c->x = t;
		return;
	case 0xf:
		st16(c, ea, isb ? c->u : c->x);
		return;
	}
	uint8_t v = opnd8(c, mode, ea);
	uint8_t r = alu8(c, col, *r8, v);
	if (alu_writes(col))
		*r8 = r;
}

// ---- public ----
void hd6309_init(hd6309_t *c)
{
	memset(c, 0, sizeof(*c));
	c->cc = CC_I | CC_F;
	c->v = 0xffff;   // MAME device_start: V starts at $FFFF (reset leaves it alone)
}

void hd6309_reset(hd6309_t *c)
{
	c->nmi_asserted = 0;
	c->lds_encountered = 0;
	c->wait = W_RUN;
	c->tfm = 0;
	c->md = 0;
	c->dp = 0;
	c->cc |= CC_I | CC_F;
	uint16_t h = RD(c, VEC_RESET);
	c->pc = (uint16_t)(h << 8 | RD(c, VEC_RESET + 1));
	c->cycles = 0;
}

int hd6309_step(hd6309_t *c)
{
	int64_t start = c->cycles;
	if (c->tfm) {
		tfm_iter(c);
		return (int)(c->cycles - start);
	}
	if (c->wait) {
		if (c->wait == W_CWAI) {
			uint16_t v = pending(c);
			if (!v) {
				c->cycles += 8;
				return 8;
			}
			if (c->nmi_asserted)
				c->nmi_asserted = 0;
			if (v == VEC_IRQ && c->irq_hold)
				c->irq_line = c->irq_hold = 0;
			if (v == VEC_FIRQ && c->firq_hold)
				c->firq_line = c->firq_hold = 0;
			c->cc |= (uint8_t)(CC_I | (v != VEC_IRQ ? CC_F : 0));
			c->wait = W_RUN;
			vector(c, v);
			return (int)(c->cycles - start);
		}
		if (c->nmi_asserted || c->firq_line || c->irq_line) {   // SYNC
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
			push_regs(c, 1, entire(c));
			c->cc |= CC_I | CC_F;
		} else if (v == VEC_FIRQ) {
			if (c->firq_hold)
				c->firq_line = c->firq_hold = 0;
			if (c->md & 0x02) {   // FIRQ saves the entire state
				c->cc |= CC_E;
				push_regs(c, 1, entire(c));
			} else {
				c->cc &= (uint8_t)~CC_E;
				push_regs(c, 1, 0x81);
			}
			c->cc |= CC_I | CC_F;
		} else {
			if (c->irq_hold)
				c->irq_line = c->irq_hold = 0;
			c->cc |= CC_E;
			push_regs(c, 1, entire(c));
			c->cc |= CC_I;
		}
		vector(c, v);
		return (int)(c->cycles - start);
	}
	execute(c);
	return (int)(c->cycles - start);
}

void hd6309_run_until(hd6309_t *c, int64_t target)
{
	while (c->cycles < target) {
		if (c->wait) {
			int wake = c->wait == W_CWAI ? pending(c) != 0
			         : (c->nmi_asserted || c->firq_line || c->irq_line);
			if (!wake) {
				c->cycles = target;
				return;
			}
		}
		hd6309_step(c);
	}
}

void hd6309_set_irq(hd6309_t *c, int state)
{
	c->irq_line = state != 0;
	c->irq_hold = 0;
}
void hd6309_hold_irq(hd6309_t *c) { c->irq_line = c->irq_hold = 1; }
void hd6309_set_firq(hd6309_t *c, int state)
{
	c->firq_line = state != 0;
	c->firq_hold = 0;
}
void hd6309_hold_firq(hd6309_t *c) { c->firq_line = c->firq_hold = 1; }
void hd6309_set_nmi(hd6309_t *c, int state)
{
	if (state && !c->nmi_line && c->lds_encountered)
		c->nmi_asserted = 1;
	c->nmi_line = state != 0;
}
