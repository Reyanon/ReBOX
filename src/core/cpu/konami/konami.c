/*  Konami 052001 custom CPU, ported to C from MAME src/devices/cpu/m6809 (konami.cpp, konami.lst, base6x09.lst,
    m6809.h, m6809inl.h; BSD-3-Clause, copyright-holders: Nathan Woods; based on the M6809 core by John Butler).
    MAME's state-machine core (generated from the .lst files) is rewritten here as one switch per instruction, in the
    style of cpu/m6809/m6809.c. Each MAME memory access / dummy_read_opcode_arg / dummy_vma / eat costs one clock
    here too, so instruction timings are those of MAME (konami_cpu_device: hd6309_native_mode() = true). Dummy
    (VMA and opcode-argument) reads are counted but not sent to the bus.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../cheat_cpu.h"
#include "konami.h"

#include <string.h>

enum { CC_C = 0x01, CC_V = 0x02, CC_Z = 0x04, CC_N = 0x08, CC_I = 0x10, CC_H = 0x20, CC_F = 0x40, CC_E = 0x80 };
#define CC_NZ    (CC_N | CC_Z)
#define CC_NZV   (CC_N | CC_Z | CC_V)
#define CC_NZVC  (CC_N | CC_Z | CC_V | CC_C)
#define CC_HNZVC (CC_H | CC_N | CC_Z | CC_V | CC_C)

enum { VEC_FIRQ = 0xfff6, VEC_IRQ = 0xfff8, VEC_NMI = 0xfffc, VEC_RESET = 0xfffe };

// MAME addressing modes (m_addressing_mode)
enum { M_IMM, M_EA, M_A, M_B, M_D };
// 16-bit register operands (MAME regop16)
enum { R_D, R_X, R_Y, R_U, R_S };

static inline uint8_t RD(konami_t *c, uint16_t a)
{
	c->cycles++;
	const uint8_t *p = c->rpage[a >> 8];
	return p ? p[a & 0xff] : c->read(c->ctx, a);
}

static inline void WR(konami_t *c, uint16_t a, uint8_t v)
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
#define DREG     ((uint16_t)(c->a << 8 | c->b))
#define SETD(v)  do { uint16_t v_ = (uint16_t)(v); c->a = (uint8_t)(v_ >> 8); c->b = (uint8_t)v_; } while (0)

// MAME set_flags<uint8_t> / set_flags<uint16_t>
static inline uint8_t sf8(konami_t *c, uint8_t mask, uint32_t a, uint32_t b, uint32_t r)
{
	c->cc &= (uint8_t)~mask;
	if (mask & CC_H) c->cc |= ((a ^ b ^ r) & 0x10) ? CC_H : 0;
	if (mask & CC_N) c->cc |= (r & 0x80) ? CC_N : 0;
	if (mask & CC_Z) c->cc |= ((uint8_t)r == 0) ? CC_Z : 0;
	if (mask & CC_V) c->cc |= ((a ^ b ^ r ^ (r >> 1)) & 0x80) ? CC_V : 0;
	if (mask & CC_C) c->cc |= (r & 0x100) ? CC_C : 0;
	return (uint8_t)r;
}

static inline uint16_t sf16(konami_t *c, uint8_t mask, uint32_t a, uint32_t b, uint32_t r)
{
	c->cc &= (uint8_t)~mask;
	if (mask & CC_N) c->cc |= (r & 0x8000) ? CC_N : 0;
	if (mask & CC_Z) c->cc |= ((uint16_t)r == 0) ? CC_Z : 0;
	if (mask & CC_V) c->cc |= ((a ^ b ^ r ^ (r >> 1)) & 0x8000) ? CC_V : 0;
	if (mask & CC_C) c->cc |= (r & 0x10000) ? CC_C : 0;
	return (uint16_t)r;
}

// branch condition of opcodes 0x60-0x7f (low 3 bits: BRA BHI BCC BNE BVC BPL BGE BGT; bit 4 negates)
static inline int cond(konami_t *c, int op)
{
	uint8_t cc = c->cc;
	int ge = ((cc & CC_N) != 0) == ((cc & CC_V) != 0);
	int r;
	switch (op & 7) {
	case 0: r = 1; break;
	case 1: r = !(cc & (CC_Z | CC_C)); break;
	case 2: r = !(cc & CC_C); break;
	case 3: r = !(cc & CC_Z); break;
	case 4: r = !(cc & CC_V); break;
	case 5: r = !(cc & CC_N); break;
	case 6: r = ge; break;
	default: r = ge && !(cc & CC_Z); break;
	}
	return (op & 0x10) ? !r : r;
}

// ---- indexed addressing (konami.lst INDEXED) ----
static inline uint16_t *ireg(konami_t *c, uint8_t pb)
{
	switch (pb & 0x70) {
	case 0x20: return &c->x;
	case 0x30: return &c->y;
	case 0x50: return &c->u;
	case 0x60: return &c->s;
	default:   return &c->pc;   // 0x70 (only those five reach here)
	}
}

static uint16_t ea_indexed(konami_t *c)
{
	uint8_t pb = FETCH();
	uint16_t t, h, base, *r;
	switch (pb & 0xf7) {
	case 0x07:   // extended
		h = FETCH();
		t = (uint16_t)(h << 8 | FETCH());
		IDLE(1);
		break;
	case 0x20: case 0x30: case 0x50: case 0x60: case 0x70:   // ,r+
		r = ireg(c, pb); t = *r; (*r)++; IDLE(2);
		break;
	case 0x21: case 0x31: case 0x51: case 0x61: case 0x71:   // ,r++
		r = ireg(c, pb); t = *r; *r += 2; IDLE(3);
		break;
	case 0x22: case 0x32: case 0x52: case 0x62: case 0x72:   // ,-r
		r = ireg(c, pb); (*r)--; t = *r; IDLE(2);
		break;
	case 0x23: case 0x33: case 0x53: case 0x63: case 0x73:   // ,--r
		r = ireg(c, pb); *r -= 2; t = *r; IDLE(3);
		break;
	case 0x24: case 0x34: case 0x54: case 0x64: case 0x74:   // n8,r (base taken before the offset fetch)
		base = *ireg(c, pb);
		t = (uint16_t)(base + (int8_t)FETCH());
		IDLE(2);
		break;
	case 0x25: case 0x35: case 0x55: case 0x65: case 0x75:   // n16,r (base taken before the offset fetch)
		base = *ireg(c, pb);
		h = FETCH();
		h = (uint16_t)(h << 8 | FETCH());
		t = (uint16_t)(base + h);
		IDLE(2);
		break;
	case 0x26: case 0x36: case 0x56: case 0x66: case 0x76:   // ,r
		t = *ireg(c, pb); IDLE(1);
		break;
	case 0xc4:   // direct
		t = (uint16_t)(c->dp << 8 | FETCH());
		IDLE(1);
		break;
	case 0xa0: case 0xb0: case 0xd0: case 0xe0: case 0xf0:   // A,r
		t = (uint16_t)(*ireg(c, pb) + (int8_t)c->a); IDLE(2);
		break;
	case 0xa1: case 0xb1: case 0xd1: case 0xe1: case 0xf1:   // B,r
		t = (uint16_t)(*ireg(c, pb) + (int8_t)c->b); IDLE(2);
		break;
	case 0xa7: case 0xb7: case 0xd7: case 0xe7: case 0xf7:   // D,r
		t = (uint16_t)(*ireg(c, pb) + DREG); IDLE(2);
		break;
	default:     // invalid postbyte (MAME logs it)
		t = 0;
		break;
	}
	if (pb & 0x08) {   // indirect
		h = RD(c, t);
		t = (uint16_t)(h << 8 | RD(c, (uint16_t)(t + 1)));
		IDLE(1);
	}
	return t;
}

// ---- operands (konami read_operand / write_operand) ----
static inline uint8_t rd_op(konami_t *c, int mode, uint16_t ea, int ord)
{
	switch (mode) {
	case M_EA:  return RD(c, (uint16_t)(ea + ord));
	case M_IMM: return FETCH();
	case M_A:   return c->a;
	case M_B:   return c->b;
	default:    return (ord & 1) ? c->b : c->a;   // M_D
	}
}

static inline void wr_op(konami_t *c, int mode, uint16_t ea, int ord, uint8_t v)
{
	switch (mode) {
	case M_EA:  WR(c, (uint16_t)(ea + ord), v); break;
	case M_IMM: break;
	case M_A:   c->a = v; break;
	case M_B:   c->b = v; break;
	default:    if (ord & 1) c->b = v; else c->a = v; break;   // M_D
	}
}

static inline uint16_t rd_op16(konami_t *c, int mode, uint16_t ea)
{
	uint16_t h = rd_op(c, mode, ea, 0);
	return (uint16_t)(h << 8 | rd_op(c, mode, ea, 1));
}

static inline void wr_op16(konami_t *c, int mode, uint16_t ea, uint16_t v)
{
	wr_op(c, mode, ea, 0, (uint8_t)(v >> 8));
	wr_op(c, mode, ea, 1, (uint8_t)v);
}

static inline uint16_t get16(konami_t *c, int r)
{
	switch (r) {
	case R_D: return DREG;
	case R_X: return c->x;
	case R_Y: return c->y;
	case R_U: return c->u;
	default:  return c->s;
	}
}

static inline void set16(konami_t *c, int r, uint16_t v)
{
	switch (r) {
	case R_D: SETD(v); break;
	case R_X: c->x = v; break;
	case R_Y: c->y = v; break;
	case R_U: c->u = v; break;
	default:  c->s = v; break;
	}
}

// ---- stack ----
static void push_regs(konami_t *c, int use_s, uint8_t m)
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

static void pull_regs(konami_t *c, int use_s, uint8_t m)
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
	IDLE(1);   // eat(1)
}

static void push_pc(konami_t *c, uint16_t target)   // GOTO_SUBROUTINE
{
	c->s--; WR(c, c->s, (uint8_t)c->pc);
	c->s--; WR(c, c->s, (uint8_t)(c->pc >> 8));
	c->pc = target;
}

static void vector(konami_t *c, uint16_t v)   // INTERRUPT_VECTOR
{
	IDLE(1);
	uint16_t h = RD(c, v);
	c->pc = (uint16_t)(h << 8 | RD(c, (uint16_t)(v + 1)));
	IDLE(1);
}

static inline uint16_t pending(konami_t *c)
{
	if (c->nmi_asserted)
		return VEC_NMI;
	if (!(c->cc & CC_F) && c->firq_line)
		return VEC_FIRQ;
	if (!(c->cc & CC_I) && c->irq_line)
		return VEC_IRQ;
	return 0;
}

// ---- EXG / TFR (konami read_exgtfr_register / write_exgtfr_register) ----
static uint16_t rd_xr(konami_t *c, int r)
{
	switch (r & 7) {
	case 0: return (uint16_t)(0x1000 | c->a);   // A (high byte reads as 0x10)
	case 1: return DREG;                        // D
	case 2: return c->x;
	case 3: return c->y;
	case 4: return 0;                           // DP: unemulated in MAME
	case 5: return c->u;
	case 6: return c->s;
	default: return c->pc;
	}
}

static void wr_xr(konami_t *c, int r, uint16_t v)
{
	switch (r & 7) {
	case 0: c->a = (uint8_t)v; break;
	case 1: c->b = (uint8_t)v; break;           // B (not D), as MAME
	case 2: c->x = v; break;
	case 3: c->y = v; break;
	case 4: break;                              // DP: unemulated in MAME
	case 5: c->u = v; break;
	case 6: c->s = v; break;
	default: c->pc = v; break;
	}
}

// ---- 8-bit read-modify-write group (mode M_A, M_B or M_EA) ----
enum { RMW_CLR, RMW_COM, RMW_NEG, RMW_INC, RMW_DEC, RMW_TST, RMW_LSR, RMW_ROR, RMW_ASR, RMW_ASL, RMW_ROL };

static void rmw8(konami_t *c, int fn, int mode, uint16_t ea)
{
	uint8_t t = rd_op(c, mode, ea, 0);
	switch (fn) {
	case RMW_CLR:
		c->cc = (uint8_t)((c->cc & ~CC_NZVC) | CC_Z);
		t = 0;
		if (mode == M_EA) IDLE(1);
		break;
	case RMW_COM:
		c->cc &= (uint8_t)~CC_V;
		c->cc |= CC_C;
		t = (uint8_t)~t;
		sf8(c, CC_NZ, 0, t, t);
		break;
	case RMW_NEG:
		t = sf8(c, CC_NZVC, 0, t, 0u - t);
		break;
	case RMW_INC:
		t = sf8(c, CC_NZV, t, 1, (uint32_t)t + 1);
		if (mode == M_EA) IDLE(1);
		break;
	case RMW_DEC:
		t = sf8(c, CC_NZV, t, 1, (uint32_t)t - 1);
		if (mode == M_EA) IDLE(1);
		break;
	case RMW_TST:
		sf8(c, CC_NZV, 0, t, t);
		if (mode == M_EA) IDLE(1);
		return;   // no write
	case RMW_LSR:
		c->cc = (uint8_t)((c->cc & ~CC_C) | (t & 1));
		t >>= 1;
		sf8(c, CC_NZ, 0, t, t);
		break;
	case RMW_ROR: {
		uint8_t nc = t & 1;
		t = (uint8_t)((t >> 1) | ((c->cc & CC_C) ? 0x80 : 0));
		c->cc = (uint8_t)((c->cc & ~CC_C) | nc);
		sf8(c, CC_NZ, 0, t, t);
		break;
	}
	case RMW_ASR:
		c->cc = (uint8_t)((c->cc & ~CC_C) | (t & 1));
		t = (uint8_t)((int8_t)t >> 1);
		sf8(c, CC_NZ, 0, t, t);
		break;
	case RMW_ASL:
		t = sf8(c, CC_NZVC, t, t, (uint32_t)t << 1);
		break;
	default: {   // ROL
		uint32_t r = ((uint32_t)t << 1) | ((c->cc & CC_C) ? 1 : 0);
		c->cc = (uint8_t)((c->cc & ~CC_C) | ((t & 0x80) ? CC_C : 0));
		t = sf8(c, CC_NZV, t, t, r);
		break;
	}
	}
	wr_op(c, mode, ea, 0, t);
}

// ---- 16-bit shifts / rotates by one (ROL/ROR: MAME rotate_left/right) ----
enum { SH_LSR, SH_ROR, SH_ASR, SH_ASL, SH_ROL };

static inline uint16_t shift16(konami_t *c, int fn, uint16_t t)
{
	switch (fn) {
	case SH_LSR:
		c->cc = (uint8_t)((c->cc & ~CC_C) | (t & 1));
		return sf16(c, CC_NZ, 0, (uint16_t)(t >> 1), (uint16_t)(t >> 1));
	case SH_ROR: {
		uint16_t r = (uint16_t)((t >> 1) | ((c->cc & CC_C) ? 0x8000 : 0));
		c->cc = (uint8_t)((c->cc & ~CC_C) | (t & 1));
		return sf16(c, CC_NZ, 0, r, r);
	}
	case SH_ASR: {
		uint16_t r = (uint16_t)((int16_t)t >> 1);
		c->cc = (uint8_t)((c->cc & ~CC_C) | (t & 1));
		return sf16(c, CC_NZ, 0, r, r);
	}
	case SH_ASL:
		return sf16(c, CC_NZVC, t, t, (uint32_t)t << 1);
	default: {   // ROL: set_flags<uint16_t>(CC_NZV, r) truncates r, so V always ends up clear
		uint16_t r = (uint16_t)((t << 1) | ((c->cc & CC_C) ? 1 : 0));
		c->cc = (uint8_t)((c->cc & ~CC_C) | ((t & 0x8000) ? CC_C : 0));
		return sf16(c, CC_NZV, 0, r, r);
	}
	}
}

// LSR16 ROR16 ASR16 ASL16 ROL16 (indexed only)
static void shift16_ea(konami_t *c, int fn, uint16_t ea)
{
	uint16_t t = rd_op16(c, M_EA, ea);
	t = shift16(c, fn, t);
	wr_op16(c, M_EA, ea, t);
}

// LSRDI RORDI ASRDI ASLDI ROLDI: register form counts from the immediate, indexed form from A
static void shift16_n(konami_t *c, int fn, int mode, uint16_t ea)
{
	c->bcount = mode == M_EA ? c->a : c->temp_im;
	uint16_t t = rd_op16(c, mode, ea);
	while (c->bcount-- & 0xf) {
		IDLE(1);
		t = shift16(c, fn, t);
	}
	wr_op16(c, mode, ea, t);
}

// ---- 16-bit read-modify-write group (mode M_D or M_EA) ----
enum { W_CLR, W_NEG, W_INC, W_DEC, W_TST, W_ABS };

static void rmw16(konami_t *c, int fn, int mode, uint16_t ea)
{
	uint16_t t;
	if (fn == W_CLR) {
		c->cc = (uint8_t)((c->cc & ~CC_NZVC) | CC_Z);
		wr_op16(c, mode, ea, 0);
		return;
	}
	t = rd_op16(c, mode, ea);
	switch (fn) {
	case W_NEG: t = sf16(c, CC_NZVC, 0, t, 0u - t); break;
	case W_INC: t = sf16(c, CC_NZVC, t, 1, (uint32_t)t + 1); break;
	case W_DEC: t = sf16(c, CC_NZVC, t, 1, (uint32_t)t - 1); break;
	case W_TST:
		sf16(c, CC_NZV, 0, t, t);
		if (mode == M_EA) IDLE(1);
		return;
	default:    // ABS
		t = sf16(c, CC_NZVC, 0, t, (t & 0x8000) ? 0u - t : t);
		break;
	}
	wr_op16(c, mode, ea, t);
}

// ---- 8-bit ALU (LD ADD ADC SUB SBC AND BIT EOR OR CMP) ----
enum { A_LD, A_ADD, A_ADC, A_SUB, A_SBC, A_AND, A_BIT, A_EOR, A_OR, A_CMP };

static void alu8(konami_t *c, int fn, uint8_t *r, int mode, uint16_t ea)
{
	uint8_t v = rd_op(c, mode, ea, 0);
	uint8_t cy = (c->cc & CC_C) ? 1 : 0;
	switch (fn) {
	case A_LD:  *r = v; sf8(c, CC_NZV, 0, v, v); break;
	case A_ADD: *r = sf8(c, CC_HNZVC, *r, v, (uint32_t)*r + v); break;
	case A_ADC: *r = sf8(c, CC_HNZVC, *r, v, (uint32_t)*r + v + cy); break;
	case A_SUB: *r = sf8(c, CC_NZVC, *r, v, (uint32_t)*r - v); break;
	case A_SBC: *r = sf8(c, CC_NZVC, *r, v, (uint32_t)*r - v - cy); break;
	case A_AND: c->cc &= (uint8_t)~CC_V; *r = sf8(c, CC_NZ, 0, *r, *r & v); break;
	case A_BIT: c->cc &= (uint8_t)~CC_V; sf8(c, CC_NZ, 0, *r, *r & v); break;
	case A_EOR: c->cc &= (uint8_t)~CC_V; *r = sf8(c, CC_NZ, 0, *r, *r ^ v); break;
	case A_OR:  c->cc &= (uint8_t)~CC_V; *r = sf8(c, CC_NZ, 0, *r, *r | v); break;
	default:    sf8(c, CC_NZVC, *r, v, (uint32_t)*r - v); break;   // CMP
	}
}

// ---- 16-bit ALU (LD CMP ADD SUB) ----
enum { L_LD, L_CMP, L_ADD, L_SUB };

static void alu16(konami_t *c, int fn, int reg, int mode, uint16_t ea)
{
	uint16_t v = rd_op16(c, mode, ea);
	uint16_t r = get16(c, reg);
	switch (fn) {
	case L_LD:
		set16(c, reg, v);
		sf16(c, CC_NZV, 0, v, v);
		if (reg == R_S)
			c->lds_encountered = 1;
		break;
	case L_CMP: sf16(c, CC_NZVC, r, v, (uint32_t)r - v); break;
	case L_ADD: set16(c, reg, sf16(c, CC_NZVC, r, v, (uint32_t)r + v)); break;
	default:    set16(c, reg, sf16(c, CC_NZVC, r, v, (uint32_t)r - v)); break;
	}
}

static void st16(konami_t *c, int reg, uint16_t ea)
{
	uint16_t v = get16(c, reg);
	wr_op16(c, M_EA, ea, v);
	sf16(c, CC_NZV, 0, v, v);
}

static void branch(konami_t *c, int taken)   // BRANCH
{
	int8_t o = (int8_t)FETCH();
	IDLE(1);
	if (taken)
		c->pc = (uint16_t)(c->pc + o);
}

static void execute(konami_t *c)
{
	uint8_t op = FETCH();
	uint16_t ea, t;
	switch (op) {
	// LEAX LEAY LEAU LEAS
	case 0x08: c->x = ea_indexed(c); sf16(c, CC_Z, 0, c->x, c->x); IDLE(1); return;
	case 0x09: c->y = ea_indexed(c); sf16(c, CC_Z, 0, c->y, c->y); IDLE(1); return;
	case 0x0a: c->u = ea_indexed(c); IDLE(1); return;
	case 0x0b: c->s = ea_indexed(c); c->lds_encountered = 1; IDLE(1); return;
	case 0x0c: { uint8_t m = FETCH(); IDLE(2); push_regs(c, 1, m); return; }   // PSHS
	case 0x0d: { uint8_t m = FETCH(); IDLE(2); push_regs(c, 0, m); return; }   // PSHU
	case 0x0e: { uint8_t m = FETCH(); IDLE(1); pull_regs(c, 1, m); return; }   // PULS
	case 0x0f: { uint8_t m = FETCH(); IDLE(1); pull_regs(c, 0, m); return; }   // PULU

	// 0x10-0x37: LD ADD ADC SUB SBC AND BIT EOR OR CMP, A/B, immediate/indexed
	case 0x10: case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: case 0x17:
	case 0x18: case 0x19: case 0x1a: case 0x1b: case 0x1c: case 0x1d: case 0x1e: case 0x1f:
	case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26: case 0x27:
	case 0x28: case 0x29: case 0x2a: case 0x2b: case 0x2c: case 0x2d: case 0x2e: case 0x2f:
	case 0x30: case 0x31: case 0x32: case 0x33: case 0x34: case 0x35: case 0x36: case 0x37: {
		static const uint8_t fn[10] = { A_LD, A_ADD, A_ADC, A_SUB, A_SBC, A_AND, A_BIT, A_EOR, A_OR, A_CMP };
		int mode = (op & 2) ? M_EA : M_IMM;
		ea = mode == M_EA ? ea_indexed(c) : 0;
		alu8(c, fn[(op - 0x10) >> 2], (op & 1) ? &c->b : &c->a, mode, ea);
		return;
	}
	case 0x38: { uint8_t v = FETCH(); if (c->set_lines) c->set_lines(c->ctx, v); return; }   // SETLN #
	case 0x39: { ea = ea_indexed(c); uint8_t v = RD(c, ea); if (c->set_lines) c->set_lines(c->ctx, v); return; }
	case 0x3a: ea = ea_indexed(c); WR(c, ea, sf8(c, CC_NZV, 0, c->a, c->a)); return;   // STA
	case 0x3b: ea = ea_indexed(c); WR(c, ea, sf8(c, CC_NZV, 0, c->b, c->b)); return;   // STB
	case 0x3c: c->cc &= FETCH(); return;   // ANDCC
	case 0x3d: c->cc |= FETCH(); return;   // ORCC
	case 0x3e: case 0x3f: {   // EXG / TFR (param bit 7 selects TFR, the opcode bit is ignored, as MAME)
		uint8_t p = FETCH();
		uint16_t r = rd_xr(c, p);
		if (!(p & 0x80)) {
			wr_xr(c, p, rd_xr(c, p >> 4));
			IDLE(1);
		}
		wr_xr(c, p >> 4, r);
		IDLE(2);
		return;
	}

	// 0x40-0x53: LD16 / CMP16
	case 0x40: case 0x41: case 0x42: case 0x43: case 0x44: case 0x45: case 0x46: case 0x47:
	case 0x48: case 0x49: case 0x4a: case 0x4b: case 0x4c: case 0x4d: case 0x4e: case 0x4f:
	case 0x50: case 0x51: case 0x52: case 0x53: {
		static const uint8_t reg[10] = { R_D, R_X, R_Y, R_U, R_S, R_D, R_X, R_Y, R_U, R_S };
		int i = (op - 0x40) >> 1;
		int mode = (op & 1) ? M_EA : M_IMM;
		ea = mode == M_EA ? ea_indexed(c) : 0;
		alu16(c, i < 5 ? L_LD : L_CMP, reg[i], mode, ea);
		return;
	}
	case 0x54: case 0x55: case 0x56: case 0x57: {   // ADDD SUBD
		int mode = (op & 1) ? M_EA : M_IMM;
		ea = mode == M_EA ? ea_indexed(c) : 0;
		alu16(c, (op & 2) ? L_SUB : L_ADD, R_D, mode, ea);
		return;
	}
	case 0x58: case 0x59: case 0x5a: case 0x5b: case 0x5c:   // STD STX STY STU STS
		ea = ea_indexed(c);
		st16(c, op - 0x58, ea);
		return;

	// 0x60-0x7f: short / long branches
	case 0x60: case 0x61: case 0x62: case 0x63: case 0x64: case 0x65: case 0x66: case 0x67:
	case 0x70: case 0x71: case 0x72: case 0x73: case 0x74: case 0x75: case 0x76: case 0x77:
		branch(c, cond(c, op));
		return;
	case 0x68: case 0x69: case 0x6a: case 0x6b: case 0x6c: case 0x6d: case 0x6e: case 0x6f:
	case 0x78: case 0x79: case 0x7a: case 0x7b: case 0x7c: case 0x7d: case 0x7e: case 0x7f: {
		int taken = cond(c, op);
		uint16_t h = FETCH();
		t = (uint16_t)(h << 8 | FETCH());
		IDLE(1);
		if (taken)
			c->pc = (uint16_t)(c->pc + t);
		return;
	}

	// 0x80-0xa2: 8-bit read-modify-write, A / B / indexed
	case 0x80: rmw8(c, RMW_CLR, M_A, 0); return;
	case 0x81: rmw8(c, RMW_CLR, M_B, 0); return;
	case 0x82: rmw8(c, RMW_CLR, M_EA, ea_indexed(c)); return;
	case 0x83: rmw8(c, RMW_COM, M_A, 0); return;
	case 0x84: rmw8(c, RMW_COM, M_B, 0); return;
	case 0x85: rmw8(c, RMW_COM, M_EA, ea_indexed(c)); return;
	case 0x86: rmw8(c, RMW_NEG, M_A, 0); return;
	case 0x87: rmw8(c, RMW_NEG, M_B, 0); return;
	case 0x88: rmw8(c, RMW_NEG, M_EA, ea_indexed(c)); return;
	case 0x89: rmw8(c, RMW_INC, M_A, 0); return;
	case 0x8a: rmw8(c, RMW_INC, M_B, 0); return;
	case 0x8b: rmw8(c, RMW_INC, M_EA, ea_indexed(c)); return;
	case 0x8c: rmw8(c, RMW_DEC, M_A, 0); return;
	case 0x8d: rmw8(c, RMW_DEC, M_B, 0); return;
	case 0x8e: rmw8(c, RMW_DEC, M_EA, ea_indexed(c)); return;
	case 0x8f: pull_regs(c, 1, 0x80); return;   // RTS
	case 0x90: rmw8(c, RMW_TST, M_A, 0); return;
	case 0x91: rmw8(c, RMW_TST, M_B, 0); return;
	case 0x92: rmw8(c, RMW_TST, M_EA, ea_indexed(c)); return;
	case 0x93: rmw8(c, RMW_LSR, M_A, 0); return;
	case 0x94: rmw8(c, RMW_LSR, M_B, 0); return;
	case 0x95: rmw8(c, RMW_LSR, M_EA, ea_indexed(c)); return;
	case 0x96: rmw8(c, RMW_ROR, M_A, 0); return;
	case 0x97: rmw8(c, RMW_ROR, M_B, 0); return;
	case 0x98: rmw8(c, RMW_ROR, M_EA, ea_indexed(c)); return;
	case 0x99: rmw8(c, RMW_ASR, M_A, 0); return;
	case 0x9a: rmw8(c, RMW_ASR, M_B, 0); return;
	case 0x9b: rmw8(c, RMW_ASR, M_EA, ea_indexed(c)); return;
	case 0x9c: rmw8(c, RMW_ASL, M_A, 0); return;
	case 0x9d: rmw8(c, RMW_ASL, M_B, 0); return;
	case 0x9e: rmw8(c, RMW_ASL, M_EA, ea_indexed(c)); return;
	case 0x9f:   // RTI
		IDLE(1);
		c->cc = RD(c, c->s++);
		pull_regs(c, 1, (uint8_t)(((c->cc & CC_E) ? 0xff : 0x81) & ~0x01));
		return;
	case 0xa0: rmw8(c, RMW_ROL, M_A, 0); return;
	case 0xa1: rmw8(c, RMW_ROL, M_B, 0); return;
	case 0xa2: rmw8(c, RMW_ROL, M_EA, ea_indexed(c)); return;
	case 0xa3: shift16_ea(c, SH_LSR, ea_indexed(c)); return;   // LSRW
	case 0xa4: shift16_ea(c, SH_ROR, ea_indexed(c)); return;   // RORW
	case 0xa5: shift16_ea(c, SH_ASR, ea_indexed(c)); return;   // ASRW
	case 0xa6: shift16_ea(c, SH_ASL, ea_indexed(c)); return;   // ASLW
	case 0xa7: shift16_ea(c, SH_ROL, ea_indexed(c)); return;   // ROLW
	case 0xa8: c->pc = ea_indexed(c); return;                  // JMP
	case 0xa9: ea = ea_indexed(c); IDLE(2); push_pc(c, ea); return;   // JSR
	case 0xaa: {   // BSR
		int8_t o = (int8_t)FETCH();
		ea = (uint16_t)(c->pc + o);
		IDLE(2);
		push_pc(c, ea);
		return;
	}
	case 0xab: {   // LBSR
		uint16_t h = FETCH();
		t = (uint16_t)(h << 8 | FETCH());
		ea = (uint16_t)(c->pc + t);
		IDLE(2);
		push_pc(c, ea);
		return;
	}
	case 0xac:   // DECB,JNZ
		c->b = sf8(c, CC_NZV, c->b, 1, (uint32_t)c->b - 1);
		IDLE(1);
		branch(c, !(c->cc & CC_Z));
		return;
	case 0xad:   // DECX,JNZ
		c->x = sf16(c, CC_NZV, c->x, 1, (uint32_t)c->x - 1);
		IDLE(1);
		branch(c, !(c->cc & CC_Z));
		return;
	case 0xae: return;   // NOP

	case 0xb0: c->x = (uint16_t)(c->x + c->b); return;   // ABX
	case 0xb1: {   // DAA
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
		return;
	}
	case 0xb2:   // SEX
		t = (uint16_t)(int16_t)(int8_t)c->b;
		SETD(t);
		sf16(c, CC_NZ, 0, t, t);
		return;
	case 0xb3:   // MUL
		t = (uint16_t)(c->a * c->b);
		SETD(t);
		sf16(c, CC_Z, 0, t, t);
		c->cc = (uint8_t)((c->cc & ~CC_C) | ((t & 0x80) ? CC_C : 0));
		IDLE(9);
		return;
	case 0xb4: {   // LMUL: X:Y = X * Y
		uint32_t r = (uint32_t)c->x * c->y;
		c->x = (uint16_t)(r >> 16);
		c->y = (uint16_t)r;
		c->cc = (uint8_t)((c->cc & ~(CC_Z | CC_C)) | (r == 0 ? CC_Z : 0) | ((r & 0x8000) ? CC_C : 0));
		IDLE(21);
		return;
	}
	case 0xb5: {   // DIVX: X = X / B, B = X % B (divide by zero gives 0, 0)
		uint16_t q = 0;
		uint8_t rem = 0;
		if (c->b) {
			q = (uint16_t)(c->x / c->b);
			rem = (uint8_t)(c->x % c->b);
		}
		c->x = sf16(c, CC_Z, 0, q, q);
		c->b = rem;
		c->cc = (uint8_t)((c->cc & ~CC_C) | ((q & 0x80) ? CC_C : 0));
		IDLE(10);
		return;
	}
	case 0xb6:   // BMOVE Y,X,U (runs to completion, as MAME)
		while (c->u) {
			uint8_t v = RD(c, c->y++);
			WR(c, c->x++, v);
			c->u--;
		}
		return;
	case 0xb7: {   // MOVE Y,X,U (one byte)
		uint8_t v = RD(c, c->y++);
		WR(c, c->x++, v);
		c->u--;
		return;
	}
	case 0xb8: c->temp_im = FETCH(); shift16_n(c, SH_LSR, M_D, 0); return;   // LSRD #
	case 0xb9: shift16_n(c, SH_LSR, M_EA, ea_indexed(c)); return;          // LSRD (count A)
	case 0xba: c->temp_im = FETCH(); shift16_n(c, SH_ROR, M_D, 0); return;   // RORD #
	case 0xbb: shift16_n(c, SH_ROR, M_EA, ea_indexed(c)); return;
	case 0xbc: c->temp_im = FETCH(); shift16_n(c, SH_ASR, M_D, 0); return;   // ASRD #
	case 0xbd: shift16_n(c, SH_ASR, M_EA, ea_indexed(c)); return;
	case 0xbe: c->temp_im = FETCH(); shift16_n(c, SH_ASL, M_D, 0); return;   // ASLD #
	case 0xbf: shift16_n(c, SH_ASL, M_EA, ea_indexed(c)); return;

	case 0xc0: c->temp_im = FETCH(); shift16_n(c, SH_ROL, M_D, 0); return;   // ROLD #
	case 0xc1: shift16_n(c, SH_ROL, M_EA, ea_indexed(c)); return;
	case 0xc2: rmw16(c, W_CLR, M_D, 0); return;
	case 0xc3: rmw16(c, W_CLR, M_EA, ea_indexed(c)); return;
	case 0xc4: rmw16(c, W_NEG, M_D, 0); return;
	case 0xc5: rmw16(c, W_NEG, M_EA, ea_indexed(c)); return;
	case 0xc6: rmw16(c, W_INC, M_D, 0); return;
	case 0xc7: rmw16(c, W_INC, M_EA, ea_indexed(c)); return;
	case 0xc8: rmw16(c, W_DEC, M_D, 0); return;
	case 0xc9: rmw16(c, W_DEC, M_EA, ea_indexed(c)); return;
	case 0xca: rmw16(c, W_TST, M_D, 0); return;
	case 0xcb: rmw16(c, W_TST, M_EA, ea_indexed(c)); return;
	case 0xcc: case 0xcd: {   // ABSA ABSB
		uint8_t *r = (op & 1) ? &c->b : &c->a;
		uint8_t v = *r;
		*r = sf8(c, CC_NZVC, 0, v, (v & 0x80) ? 0u - v : v);
		return;
	}
	case 0xce: rmw16(c, W_ABS, M_D, 0); return;   // ABSD
	case 0xcf:   // BSET: fill U bytes at X with A
		while (c->u) {
			IDLE(1);
			WR(c, c->x++, c->a);
			c->u--;
		}
		return;
	case 0xd0:   // BSETW: fill U words at X with D
		while (c->u) {
			IDLE(1);
			WR(c, c->x++, c->a);
			WR(c, c->x++, c->b);
			c->u--;
		}
		return;
	}
	// illegal (MAME logs it): opcode fetch only
}

// ---- public ----
// (the cheats' access to this CPU's program space: cheat_cpu.h)
static uint8_t cheat_rd(void *x, uint32_t a)   // (as the CPU: its direct pages, else the callbacks)
{
	konami_t *c = (konami_t *)x;
	const uint8_t *p = c->rpage[(a >> 8) & 0xff];
	return p ? p[a & 0xff] : c->read ? c->read(c->ctx, (uint16_t)a) : 0;
}
static void cheat_wr(void *x, uint32_t a, uint8_t v)
{
	konami_t *c = (konami_t *)x;
	uint8_t *p = c->wpage[(a >> 8) & 0xff];
	if (p)
		p[a & 0xff] = v;
	else if (c->write)
		c->write(c->ctx, (uint16_t)a, v);
}

void konami_init(konami_t *c)
{
	rbx_cheat_offer("konami", cheat_rd, cheat_wr, c, 1, 1);
	memset(c, 0, sizeof(*c));
	c->cc = CC_I | CC_F;
}

void konami_reset(konami_t *c)
{
	c->nmi_asserted = 0;
	c->lds_encountered = 0;
	c->dp = 0;
	c->cc |= CC_I | CC_F;
	uint16_t h = RD(c, VEC_RESET);
	c->pc = (uint16_t)(h << 8 | RD(c, VEC_RESET + 1));
	c->cycles = 0;
}

int konami_step(konami_t *c)
{
	int64_t start = c->cycles;
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

void konami_run_until(konami_t *c, int64_t target)
{
	while (c->cycles < target)
		konami_step(c);
}

void konami_set_irq(konami_t *c, int state) { c->irq_line = state != 0; }
void konami_set_firq(konami_t *c, int state)
{
	c->firq_line = state != 0;
	c->firq_hold = 0;
}
void konami_hold_firq(konami_t *c) { c->firq_line = c->firq_hold = 1; }
void konami_set_nmi(konami_t *c, int state)
{
	if (state && !c->nmi_line && c->lds_encountered)
		c->nmi_asserted = 1;
	c->nmi_line = state != 0;
}
