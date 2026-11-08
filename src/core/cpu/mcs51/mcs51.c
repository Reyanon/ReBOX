/*  Intel MCS-51 (8031 / 8051 / 8751) microcontroller, ported from MAME src/devices/cpu/mcs51/i8051.cpp and
    mcs51ops.cpp (BSD-3-Clause, copyright-holders: Steve Ellenoff, Manuel Abadia, Couriersud).
    See mcs51.h for what is covered.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "mcs51.h"

#include <string.h>

enum { PSW_CY = 7, PSW_AC = 6, PSW_OV = 2 };
enum { IE_A = 7 };
enum { TCON_TF1 = 7, TCON_TR1 = 6, TCON_TF0 = 5, TCON_TR0 = 4, TCON_IE1 = 3, TCON_IT1 = 2, TCON_IE0 = 1, TCON_IT0 = 0 };
enum { SCON_TI = 1, SCON_RI = 0 };
enum { TMOD_GATE1 = 7, TMOD_CT1 = 6, TMOD_GATE0 = 3, TMOD_CT0 = 2 };
enum { V_IE0 = 0x03, V_TF0 = 0x0b, V_IE1 = 0x13, V_TF1 = 0x1b, V_RITI = 0x23 };
enum { T2CON_TF2 = 7, T2CON_EXF2 = 6, T2CON_RCLK = 5, T2CON_TCLK = 4, T2CON_EXEN2 = 3, T2CON_TR2 = 2, T2CON_CT2 = 1,
       T2CON_CP = 0 };

#define BIT(x, n) (((x) >> (n)) & 1)

// machine cycles of each opcode
static const uint8_t s_cycles[256] = {
	1,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,
	2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,
	2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,
	2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,
	2,2,1,2,1,1,1,1,1,1,1,1,1,1,1,1,
	2,2,1,2,1,1,1,1,1,1,1,1,1,1,1,1,
	2,2,1,2,1,1,1,1,1,1,1,1,1,1,1,1,
	2,2,2,2,1,2,1,1,1,1,1,1,1,1,1,1,
	2,2,2,2,4,2,2,2,2,2,2,2,2,2,2,2,
	2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,
	2,2,1,2,4,1,2,2,2,2,2,2,2,2,2,2,
	2,2,1,1,2,2,2,2,2,2,2,2,2,2,2,2,
	2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
	2,2,1,1,1,2,1,1,2,2,2,2,2,2,2,2,
	2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,
	2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1
};

static uint8_t parity8(uint8_t v)
{
	v ^= v >> 4;
	v ^= v >> 2;
	v ^= v >> 1;
	return v & 1;
}

// ---- memory spaces ----
static inline uint8_t prog_r(mcs51_t *c, uint16_t a) { return a < c->rom_size ? c->rom[a] : 0; }
static inline uint8_t fetch(mcs51_t *c) { return prog_r(c, c->st.pc++); }

// internal data RAM: 0x00-0x7f (MAME idata map of the 128-byte parts; the rest is unmapped), 0x80-0xff on an 8052
static inline uint8_t idata_r(mcs51_t *c, uint8_t a)
{
	return a < 0x80 ? c->st.iram[a] : c->is_8052 ? c->st2.iram_hi[a - 0x80] : 0;
}
static inline void idata_w(mcs51_t *c, uint8_t a, uint8_t v)
{
	if (a < 0x80)
		c->st.iram[a] = v;
	else if (c->is_8052)
		c->st2.iram_hi[a - 0x80] = v;
}

static inline uint8_t port_rd(mcs51_t *c, int n) { return c->port_in ? c->port_in(c->user, n) : 0xff; }
static inline void port_wr(mcs51_t *c, int n, uint8_t v) { if (c->port_out) c->port_out(c->user, n, v); }

static void acc_w(mcs51_t *c, uint8_t v)
{
	c->st.acc = v;
	c->st.psw = (uint8_t)((c->st.psw & 0xfe) | parity8(v));
}

static void update_irq_prio(mcs51_t *c)
{
	for (int i = 0; i < 8; i++)
		c->st.irq_prio[i] = (uint8_t)(((c->st.ip >> i) & 1) | ((c->st2.iph >> i) & 1) << 1);
}

// the 8052 SFRs (i8052_device / i80c52_device sfr_map); -1: not one of them
static int sfr52_r(mcs51_t *c, uint8_t a)
{
	const mcs51_state52 *s = &c->st2;
	switch (a) {
	case 0xc8: return s->t2con;
	case 0xca: return (uint8_t)s->rcap2;
	case 0xcb: return (uint8_t)(s->rcap2 >> 8);
	case 0xcc: return (uint8_t)s->t2;
	case 0xcd: return (uint8_t)(s->t2 >> 8);
	case 0xa9: return s->saddr;
	case 0xb7: return s->iph;
	case 0xb9: return s->saden;
	}
	return -1;
}
static int sfr52_w(mcs51_t *c, uint8_t a, uint8_t v)
{
	mcs51_state52 *s = &c->st2;
	switch (a) {
	case 0xc8: s->t2con = v; return 1;
	case 0xca: s->rcap2 = (s->rcap2 & ~0xffu) | v; return 1;
	case 0xcb: s->rcap2 = (s->rcap2 & ~0xff00u) | (uint32_t)v << 8; return 1;
	case 0xcc: s->t2 = (s->t2 & ~0xffu) | v; return 1;
	case 0xcd: s->t2 = (s->t2 & ~0xff00u) | (uint32_t)v << 8; return 1;
	case 0xa9: s->saddr = v; return 1;
	case 0xb7: s->iph = v; update_irq_prio(c); return 1;
	case 0xb9: s->saden = v; return 1;
	}
	return 0;
}

static uint8_t sfr_r(mcs51_t *c, uint8_t a)
{
	mcs51_state *s = &c->st;
	if (c->is_8052) {
		const int v = sfr52_r(c, a);
		if (v >= 0)
			return (uint8_t)v;
	}
	switch (a) {
	case 0x80: return s->rwm ? s->p0 : (uint8_t)((s->p0 | c->forced_inputs[0]) & port_rd(c, 0));
	case 0x81: return s->sp;
	case 0x82: return (uint8_t)s->dptr;
	case 0x83: return (uint8_t)(s->dptr >> 8);
	case 0x87: return s->pcon;
	case 0x88: return s->tcon;
	case 0x89: return s->tmod;
	case 0x8a: return s->tl0;
	case 0x8b: return s->tl1;
	case 0x8c: return s->th0;
	case 0x8d: return s->th1;
	case 0x90: return s->rwm ? s->p1 : (uint8_t)((s->p1 | c->forced_inputs[1]) & port_rd(c, 1));
	case 0x98: return s->scon;
	case 0x99: return s->sbuf;
	case 0xa0: return s->rwm ? s->p2 : (uint8_t)((s->p2 | c->forced_inputs[2]) & port_rd(c, 2));
	case 0xa8: return s->ie;
	case 0xb0:
		if (s->rwm)
			return s->p3;
		return (uint8_t)((s->p3 | c->forced_inputs[3]) & port_rd(c, 3) & ~(BIT(s->last_line_state, MCS51_INT0_LINE) ? 4 : 0) &
		                 ~(BIT(s->last_line_state, MCS51_INT1_LINE) ? 8 : 0));
	case 0xb8: return s->ip;
	case 0xd0: return s->psw;
	case 0xe0: return s->acc;
	case 0xf0: return s->b;
	}
	return 0;
}

static void sfr_w(mcs51_t *c, uint8_t a, uint8_t v)
{
	mcs51_state *s = &c->st;
	if (c->is_8052 && sfr52_w(c, a, v))
		return;
	switch (a) {
	case 0x80: s->p0 = v; port_wr(c, 0, v); break;
	case 0x81: s->sp = v; break;
	case 0x82: s->dptr = (uint16_t)((s->dptr & 0xff00) | v); break;
	case 0x83: s->dptr = (uint16_t)((s->dptr & 0x00ff) | v << 8); break;
	case 0x87: s->pcon = v; break;
	case 0x88: s->tcon = v; break;
	case 0x89: s->tmod = v; break;
	case 0x8a: s->tl0 = v; break;
	case 0x8b: s->tl1 = v; break;
	case 0x8c: s->th0 = v; break;
	case 0x8d: s->th1 = v; break;
	case 0x90: s->p1 = v; port_wr(c, 1, v); break;
	case 0x98: s->scon = v; break;
	case 0x99: s->sbuf = v; break;   // (serial transmission not emulated)
	case 0xa0: s->p2 = v; port_wr(c, 2, v); break;
	case 0xa8: s->ie = v; break;
	case 0xb0: s->p3 = v; port_wr(c, 3, v); break;   // (TxD stays high: no serial)
	case 0xb8: s->ip = v; update_irq_prio(c); break;
	case 0xd0: s->psw = (uint8_t)((s->psw & 0x01) | (v & 0xfe)); break;
	case 0xe0: acc_w(c, v); break;
	case 0xf0: s->b = v; break;
	}
}

static inline uint8_t read_direct(mcs51_t *c, uint8_t r) { return r < 0x80 ? c->st.iram[r] : sfr_r(c, r); }
static inline void write_direct(mcs51_t *c, uint8_t r, uint8_t v)
{
	if (r < 0x80)
		c->st.iram[r] = v;
	else
		sfr_w(c, r, v);
}

static inline uint8_t r_reg(mcs51_t *c, int r) { return c->st.iram[r | (c->st.psw & 0x18)]; }
static inline void set_reg(mcs51_t *c, int r, uint8_t v) { c->st.iram[r | (c->st.psw & 0x18)] = v; }

static inline void set_cy(mcs51_t *c, int v) { c->st.psw = (uint8_t)((c->st.psw & 0x7f) | (v ? 0x80 : 0)); }
static inline void set_ac(mcs51_t *c, int v) { c->st.psw = (uint8_t)((c->st.psw & ~0x40) | (v ? 0x40 : 0)); }
static inline void set_ov(mcs51_t *c, int v) { c->st.psw = (uint8_t)((c->st.psw & ~0x04) | (v ? 0x04 : 0)); }
static inline int cy(mcs51_t *c) { return BIT(c->st.psw, PSW_CY); }

static uint8_t bit_r(mcs51_t *c, uint8_t offset)
{
	c->st.last_bit = offset;
	int pos = offset & 7;
	if (offset < 0x80)
		return (idata_r(c, (uint8_t)(((offset & 0x78) >> 3) + 0x20)) >> pos) & 1;
	return (sfr_r(c, (uint8_t)(((offset & 0x78) >> 3) * 8 + 0x80)) >> pos) & 1;
}

static void bit_w(mcs51_t *c, uint8_t offset, int bit)
{
	int pos = offset & 7;
	uint8_t m = (uint8_t)~(1 << pos), b = (uint8_t)((bit & 1) << pos);
	if (offset < 0x80) {
		uint8_t w = (uint8_t)(((offset & 0x78) >> 3) + 0x20);
		idata_w(c, w, (uint8_t)((idata_r(c, w) & m) | b));
	} else {
		uint8_t w = (uint8_t)(((offset & 0x78) >> 3) * 8 + 0x80);
		sfr_w(c, w, (uint8_t)((sfr_r(c, w) & m) | b));
	}
}

static void add_flags(mcs51_t *c, uint8_t a, uint8_t d, uint8_t cin)
{
	uint16_t result = (uint16_t)(a + d + cin);
	int16_t r1 = (int16_t)((int8_t)a + (int8_t)d + cin);
	c->st.psw = (uint8_t)((c->st.psw & 0x7f) | ((result & 0x100) >> 1));
	result = (uint16_t)((a & 0x0f) + (d & 0x0f) + cin);
	set_ac(c, (result & 0x10) >> 4);
	set_ov(c, r1 < -128 || r1 > 127);
}

static void sub_flags(mcs51_t *c, uint8_t a, uint8_t d, uint8_t cin)
{
	uint16_t result = (uint16_t)(a - (d + cin));
	int16_t r1 = (int16_t)((int8_t)a - (int8_t)(uint8_t)(d + cin));
	c->st.psw = (uint8_t)((c->st.psw & 0x7f) | ((result & 0x100) >> 1));
	result = (uint16_t)((a & 0x0f) - ((d & 0x0f) + cin));
	set_ac(c, (result & 0x10) >> 4);
	set_ov(c, r1 < -128 || r1 > 127);
}

static void push_pc(mcs51_t *c)
{
	idata_w(c, ++c->st.sp, (uint8_t)c->st.pc);
	idata_w(c, ++c->st.sp, (uint8_t)(c->st.pc >> 8));
}

static void pop_pc(mcs51_t *c)
{
	uint16_t hi = idata_r(c, c->st.sp--);
	c->st.pc = (uint16_t)(hi << 8 | idata_r(c, c->st.sp--));
}

static void clear_current_irq(mcs51_t *c)
{
	mcs51_state *s = &c->st;
	if (s->cur_irq_prio >= 0)
		s->irq_active &= (uint8_t)~(1 << s->cur_irq_prio);
	if (s->irq_active & 4) s->cur_irq_prio = 2;
	else if (s->irq_active & 2) s->cur_irq_prio = 1;
	else if (s->irq_active & 1) s->cur_irq_prio = 0;
	else s->cur_irq_prio = -1;
}

// ---- operand helpers ----
static inline void rel_jump(mcs51_t *c, int8_t rel) { c->st.pc = (uint16_t)(c->st.pc + rel); }

static void alu_add(mcs51_t *c, uint8_t d, int carry)
{
	uint8_t cin = carry ? (uint8_t)cy(c) : 0;
	uint8_t a = c->st.acc;
	add_flags(c, a, d, cin);
	acc_w(c, (uint8_t)(a + d + cin));
}

static void alu_subb(mcs51_t *c, uint8_t d)
{
	uint8_t cin = (uint8_t)cy(c);
	uint8_t a = c->st.acc;
	sub_flags(c, a, d, cin);
	acc_w(c, (uint8_t)(a - d - cin));
}

static void cjne(mcs51_t *c, uint8_t v, uint8_t d, int8_t rel)
{
	if (v != d)
		rel_jump(c, rel);
	set_cy(c, v < d);
}

static void execute_op(mcs51_t *c, uint8_t op)
{
	mcs51_state *s = &c->st;
	s->last_op = op;
	const int lo = op & 0x0f, ri = op & 1, rr = op & 7;
	const int is_ir = lo == 6 || lo == 7, is_r = lo >= 8;

	// AJMP / ACALL: aaa0 0001 / aaa1 0001
	if ((op & 0x0f) == 0x01) {
		uint8_t addr = fetch(c);
		if (op & 0x10)
			push_pc(c);
		s->pc = (uint16_t)((s->pc & 0xf800) | ((op & 0xe0) << 3) | addr);
		return;
	}

	switch (op >> 4) {
	case 0x0:   // INC
		switch (lo) {
		case 0x0: return;   // NOP
		case 0x2: { uint8_t h = fetch(c), l = fetch(c); s->pc = (uint16_t)(h << 8 | l); return; }   // LJMP
		case 0x3: acc_w(c, (uint8_t)((s->acc >> 1) | (s->acc << 7))); return;   // RR A
		case 0x4: acc_w(c, (uint8_t)(s->acc + 1)); return;
		case 0x5: { s->rwm = 1; uint8_t a = fetch(c); write_direct(c, a, (uint8_t)(read_direct(c, a) + 1)); s->rwm = 0; return; }
		}
		if (is_ir) { uint8_t a = r_reg(c, ri); idata_w(c, a, (uint8_t)(idata_r(c, a) + 1)); return; }
		set_reg(c, rr, (uint8_t)(r_reg(c, rr) + 1));
		return;
	case 0x1:   // DEC
		switch (lo) {
		case 0x0: {   // JBC
			s->rwm = 1;
			uint8_t a = fetch(c);
			int8_t rel = (int8_t)fetch(c);
			if (bit_r(c, a)) {
				rel_jump(c, rel);
				bit_w(c, a, 0);
			}
			s->rwm = 0;
			return;
		}
		case 0x2: { uint8_t h = fetch(c), l = fetch(c); push_pc(c); s->pc = (uint16_t)(h << 8 | l); return; }   // LCALL
		case 0x3: { int carry = s->acc & 1; acc_w(c, (uint8_t)((s->acc >> 1) | (cy(c) << 7))); set_cy(c, carry); return; }
		case 0x4: acc_w(c, (uint8_t)(s->acc - 1)); return;
		case 0x5: { s->rwm = 1; uint8_t a = fetch(c); write_direct(c, a, (uint8_t)(read_direct(c, a) - 1)); s->rwm = 0; return; }
		}
		if (is_ir) { uint8_t a = r_reg(c, ri); idata_w(c, a, (uint8_t)(idata_r(c, a) - 1)); return; }
		set_reg(c, rr, (uint8_t)(r_reg(c, rr) - 1));
		return;
	case 0x2:
	case 0x3:
		switch (lo) {
		case 0x0: {   // JB / JNB
			uint8_t a = fetch(c);
			int8_t rel = (int8_t)fetch(c);
			if (bit_r(c, a) == (op == 0x20))
				rel_jump(c, rel);
			return;
		}
		case 0x2:   // RET / RETI
			pop_pc(c);
			if (op == 0x32)
				clear_current_irq(c);
			return;
		case 0x3:
			if (op == 0x23)
				acc_w(c, (uint8_t)((s->acc << 1) | (s->acc >> 7)));
			else {
				int carry = s->acc >> 7;
				acc_w(c, (uint8_t)((s->acc << 1) | cy(c)));
				set_cy(c, carry);
			}
			return;
		}
		{
			uint8_t d;
			if (lo == 4) d = fetch(c);
			else if (lo == 5) d = read_direct(c, fetch(c));
			else if (is_ir) d = idata_r(c, r_reg(c, ri));
			else d = r_reg(c, rr);
			alu_add(c, d, op >= 0x30);
		}
		return;
	case 0x4:   // ORL
	case 0x5:   // ANL
	case 0x6:   // XRL
		if (lo == 0) {   // JC / JNC / JZ
			int8_t rel = (int8_t)fetch(c);
			int t = op == 0x40 ? cy(c) : op == 0x50 ? !cy(c) : s->acc == 0;
			if (t)
				rel_jump(c, rel);
			return;
		}
		if (lo == 2 || lo == 3) {
			s->rwm = 1;
			uint8_t a = fetch(c);
			uint8_t d = lo == 2 ? s->acc : fetch(c);
			uint8_t v = read_direct(c, a);
			v = op >> 4 == 4 ? (uint8_t)(v | d) : op >> 4 == 5 ? (uint8_t)(v & d) : (uint8_t)(v ^ d);
			write_direct(c, a, v);
			s->rwm = 0;
			return;
		}
		{
			uint8_t d;
			if (lo == 4) d = fetch(c);
			else if (lo == 5) d = read_direct(c, fetch(c));
			else if (is_ir) d = idata_r(c, r_reg(c, ri));
			else d = r_reg(c, rr);
			uint8_t a = s->acc;
			acc_w(c, op >> 4 == 4 ? (uint8_t)(a | d) : op >> 4 == 5 ? (uint8_t)(a & d) : (uint8_t)(a ^ d));
		}
		return;
	case 0x7:
		switch (lo) {
		case 0x0: { int8_t rel = (int8_t)fetch(c); if (s->acc != 0) rel_jump(c, rel); return; }   // JNZ
		case 0x2: { uint8_t b = bit_r(c, fetch(c)); s->psw |= (uint8_t)(b << 7); return; }   // ORL C, bit
		case 0x3: s->pc = (uint16_t)(s->acc + s->dptr); return;   // JMP @A+DPTR
		case 0x4: acc_w(c, fetch(c)); return;
		case 0x5: { uint8_t a = fetch(c), d = fetch(c); write_direct(c, a, d); return; }
		}
		if (is_ir) { uint8_t d = fetch(c); idata_w(c, r_reg(c, ri), d); return; }
		set_reg(c, rr, fetch(c));
		return;
	case 0x8:
		switch (lo) {
		case 0x0: rel_jump(c, (int8_t)fetch(c)); return;   // SJMP
		case 0x2: { uint8_t b = bit_r(c, fetch(c)); s->psw &= (uint8_t)((b << 7) | 0x7f); return; }   // ANL C, bit
		case 0x3: acc_w(c, prog_r(c, (uint16_t)(s->acc + s->pc))); return;   // MOVC A, @A+PC
		case 0x4:   // DIV AB
			if (s->b == 0)
				set_ov(c, 1);
			else {
				uint8_t q = (uint8_t)(s->acc / s->b), r = (uint8_t)(s->acc % s->b);
				acc_w(c, q);
				s->b = r;
				set_ov(c, 0);
			}
			s->psw &= 0x7f;
			return;
		case 0x5: { uint8_t src = fetch(c), dst = fetch(c); write_direct(c, dst, read_direct(c, src)); return; }
		}
		if (is_ir) { uint8_t a = fetch(c); write_direct(c, a, idata_r(c, r_reg(c, ri))); return; }
		{ uint8_t a = fetch(c); write_direct(c, a, r_reg(c, rr)); }
		return;
	case 0x9:
		switch (lo) {
		case 0x0: { uint8_t h = fetch(c), l = fetch(c); s->dptr = (uint16_t)(h << 8 | l); return; }   // MOV DPTR, #
		case 0x2: { s->rwm = 1; bit_w(c, fetch(c), cy(c)); s->rwm = 0; return; }   // MOV bit, C
		case 0x3: acc_w(c, prog_r(c, (uint16_t)(s->acc + s->dptr))); return;   // MOVC A, @A+DPTR
		}
		{
			uint8_t d;
			if (lo == 4) d = fetch(c);
			else if (lo == 5) d = read_direct(c, fetch(c));
			else if (is_ir) d = idata_r(c, r_reg(c, ri));
			else d = r_reg(c, rr);
			alu_subb(c, d);
		}
		return;
	case 0xa:
		switch (lo) {
		case 0x0: { uint8_t b = bit_r(c, fetch(c)); s->psw |= (uint8_t)((~b & 1) << 7); return; }   // ORL C, /bit
		case 0x2: set_cy(c, bit_r(c, fetch(c))); return;   // MOV C, bit
		case 0x3: s->dptr++; return;
		case 0x4: {   // MUL AB
			uint16_t r = (uint16_t)(s->acc * s->b);
			s->b = (uint8_t)(r >> 8);
			acc_w(c, (uint8_t)r);
			set_ov(c, (r & 0x100) >> 8);
			s->psw &= 0x7f;
			return;
		}
		case 0x5: return;   // reserved
		}
		if (is_ir) { uint8_t a = fetch(c); idata_w(c, r_reg(c, ri), read_direct(c, a)); return; }
		set_reg(c, rr, read_direct(c, fetch(c)));
		return;
	case 0xb:
		switch (lo) {
		case 0x0: { uint8_t b = bit_r(c, fetch(c)); b = (uint8_t)(~b & 1); s->psw &= (uint8_t)((b << 7) | 0x7f); return; }
		case 0x2: { s->rwm = 1; uint8_t a = fetch(c); bit_w(c, a, (~bit_r(c, a)) & 1); s->rwm = 0; return; }   // CPL bit
		case 0x3: s->psw ^= 0x80; return;
		case 0x4: { uint8_t d = fetch(c); int8_t rel = (int8_t)fetch(c); cjne(c, s->acc, d, rel); return; }
		case 0x5: { uint8_t a = fetch(c); int8_t rel = (int8_t)fetch(c); uint8_t d = read_direct(c, a); cjne(c, s->acc, d, rel); return; }
		}
		{
			uint8_t d = fetch(c);
			int8_t rel = (int8_t)fetch(c);
			uint8_t v = is_ir ? idata_r(c, r_reg(c, ri)) : r_reg(c, rr);
			cjne(c, v, d, rel);
		}
		return;
	case 0xc:
		switch (lo) {
		case 0x0: { uint8_t a = fetch(c); s->sp++; idata_w(c, s->sp, read_direct(c, a)); return; }   // PUSH
		case 0x2: { s->rwm = 1; bit_w(c, fetch(c), 0); s->rwm = 0; return; }   // CLR bit
		case 0x3: s->psw &= 0x7f; return;
		case 0x4: acc_w(c, (uint8_t)((s->acc << 4) | (s->acc >> 4))); return;   // SWAP
		case 0x5: { uint8_t a = fetch(c), d = read_direct(c, a), old = s->acc; acc_w(c, d); write_direct(c, a, old); return; }
		}
		if (is_ir) { uint8_t a = r_reg(c, ri), d = idata_r(c, a), old = s->acc; acc_w(c, d); idata_w(c, a, old); return; }
		{ uint8_t d = r_reg(c, rr), old = s->acc; acc_w(c, d); set_reg(c, rr, old); }
		return;
	case 0xd:
		switch (lo) {
		case 0x0: { uint8_t a = fetch(c); write_direct(c, a, idata_r(c, s->sp)); s->sp--; return; }   // POP
		case 0x2: { s->rwm = 1; bit_w(c, fetch(c), 1); s->rwm = 0; return; }   // SETB bit
		case 0x3: s->psw |= 0x80; return;
		case 0x4: {   // DA A
			uint16_t n = s->acc;
			if (BIT(s->psw, PSW_AC) || (n & 0x0f) > 0x09)
				n += 0x06;
			if (BIT(s->psw, PSW_CY) || ((n & 0xf0) > 0x90) || (n & ~0xff))
				n += 0x60;
			acc_w(c, (uint8_t)n);
			if (n & ~0xff)
				s->psw |= 0x80;
			return;
		}
		case 0x5: {   // DJNZ direct
			s->rwm = 1;
			uint8_t a = fetch(c);
			int8_t rel = (int8_t)fetch(c);
			uint8_t d = (uint8_t)(read_direct(c, a) - 1);
			write_direct(c, a, d);
			if (d)
				rel_jump(c, rel);
			s->rwm = 0;
			return;
		}
		}
		if (is_ir) {   // XCHD
			uint8_t acc = s->acc, a = r_reg(c, ri), d = idata_r(c, a);
			acc_w(c, (uint8_t)((acc & 0xf0) | (d & 0x0f)));
			idata_w(c, a, (uint8_t)((d & 0xf0) | (acc & 0x0f)));
			return;
		}
		{
			int8_t rel = (int8_t)fetch(c);
			set_reg(c, rr, (uint8_t)(r_reg(c, rr) - 1));
			if (r_reg(c, rr))
				rel_jump(c, rel);
		}
		return;
	case 0xe:
	case 0xf: {
		const int st = op >= 0xf0;
		switch (lo) {
		case 0x0:   // MOVX A,@DPTR / MOVX @DPTR,A
			if (st) { if (c->xdata_w) c->xdata_w(c->user, s->dptr, s->acc); }
			else acc_w(c, c->xdata_r ? c->xdata_r(c->user, s->dptr) : 0);
			return;
		case 0x2:
		case 0x3: {   // MOVX @Ri (P2 gives the high address byte, MAME external_ram_iaddr)
			uint16_t a = (uint16_t)(r_reg(c, ri) | s->p2 << 8);
			if (st) { if (c->xdata_w) c->xdata_w(c->user, a, s->acc); }
			else acc_w(c, c->xdata_r ? c->xdata_r(c->user, a) : 0);
			return;
		}
		case 0x4:
			if (st) acc_w(c, (uint8_t)~s->acc);
			else acc_w(c, 0);
			return;
		case 0x5: {
			uint8_t a = fetch(c);
			if (st) write_direct(c, a, s->acc);
			else acc_w(c, read_direct(c, a));
			return;
		}
		}
		if (is_ir) {
			if (st) idata_w(c, r_reg(c, ri), s->acc);
			else acc_w(c, idata_r(c, r_reg(c, ri)));
			return;
		}
		if (st) set_reg(c, rr, s->acc);
		else acc_w(c, r_reg(c, rr));
		(void)is_r;
		return;
	}
	}
}

// ---- timers / interrupts ----
static void update_timer_t0(mcs51_t *c, int cycles)
{
	mcs51_state *s = &c->st;
	int mode = s->tmod & 3;
	uint32_t count;
	if (BIT(s->tcon, TCON_TR0)) {
		uint32_t delta = BIT(s->tmod, TMOD_CT0) ? (uint32_t)s->t0_cnt : (uint32_t)cycles;
		s->t0_cnt = 0;
		if (BIT(s->tmod, TMOD_GATE0) && !BIT(s->tcon, TCON_IE0))
			delta = 0;
		switch (mode) {
		case 0:
			count = ((uint32_t)s->th0 << 5 | (s->tl0 & 0x1f)) + delta;
			if (count & 0xffffe000) s->tcon |= 1 << TCON_TF0;
			s->th0 = (uint8_t)(count >> 5);
			s->tl0 = count & 0x1f;
			break;
		case 1:
			count = ((uint32_t)s->th0 << 8 | s->tl0) + delta;
			if (count & 0xffff0000) s->tcon |= 1 << TCON_TF0;
			s->th0 = (uint8_t)(count >> 8);
			s->tl0 = (uint8_t)count;
			break;
		case 2:
			count = (uint32_t)s->tl0 + delta;
			if (count & 0xffffff00) {
				s->tcon |= 1 << TCON_TF0;
				count += s->th0;
			}
			s->tl0 = (uint8_t)count;
			break;
		case 3:
			count = (uint32_t)s->tl0 + delta;
			if (count & 0xffffff00) s->tcon |= 1 << TCON_TF0;
			s->tl0 = (uint8_t)count;
			break;
		}
	}
	if (BIT(s->tcon, TCON_TR1) && mode == 3) {
		count = (uint32_t)s->th0 + (uint32_t)cycles;
		if (count & 0xffffff00) s->tcon |= 1 << TCON_TF1;
		s->th0 = (uint8_t)count;
	}
}

static void update_timer_t1(mcs51_t *c, int cycles)
{
	mcs51_state *s = &c->st;
	int mode = (s->tmod >> 4) & 3, mode_0 = s->tmod & 3;
	uint32_t count, overflow = 0, delta;
	if (mode_0 != 3) {
		if (!BIT(s->tcon, TCON_TR1))
			return;
		delta = BIT(s->tmod, TMOD_CT1) ? (uint32_t)s->t1_cnt : (uint32_t)cycles;
		s->t1_cnt = 0;
		if (BIT(s->tmod, TMOD_GATE1) && !BIT(s->tcon, TCON_IE1))
			delta = 0;
	} else {
		delta = (uint32_t)cycles;
		s->t1_cnt = 0;
	}
	switch (mode) {
	case 0:
		count = ((uint32_t)s->th1 << 5 | (s->tl1 & 0x1f)) + delta;
		overflow = count & 0xffffe000;
		s->th1 = (uint8_t)(count >> 5);
		s->tl1 = count & 0x1f;
		break;
	case 1:
		count = ((uint32_t)s->th1 << 8 | s->tl1) + delta;
		overflow = count & 0xffff0000;
		s->th1 = (uint8_t)(count >> 8);
		s->tl1 = (uint8_t)count;
		break;
	case 2:
		count = (uint32_t)s->tl1 + delta;
		overflow = count & 0xffffff00;
		if (overflow)
			count += s->th1;
		s->tl1 = (uint8_t)count;
		break;
	}
	if (overflow && mode_0 != 3)
		s->tcon |= 1 << TCON_TF1;   // (the serial port clock is not emulated)
}

// i8052_device::update_timer_t2 (no T2 / T2EX pins: their counts stay 0)
static void update_timer_t2(mcs51_t *c, int cycles)
{
	mcs51_state52 *s = &c->st2;
	if (!BIT(s->t2con, T2CON_TR2))
		return;
	const int mode = (BIT(s->t2con, T2CON_TCLK) | BIT(s->t2con, T2CON_RCLK)) << 1 | BIT(s->t2con, T2CON_CP);
	const int delta = BIT(s->t2con, T2CON_CT2) ? 0 : (mode & 2) ? cycles * (12 / 2) : cycles;
	uint32_t count = s->t2 + (uint32_t)delta;
	switch (mode) {
	case 0:   // 16-bit auto reload
		if (count & 0xffff0000u) {
			s->t2con |= 1 << T2CON_TF2;
			count += s->rcap2;
		}
		break;
	case 1:   // 16-bit capture
		if (count & 0xffff0000u)
			s->t2con |= 1 << T2CON_TF2;
		break;
	default:  // baud rate (the serial port clock is not emulated)
		if (count & 0xffff0000u)
			count += s->rcap2;
		break;
	}
	s->t2 = count & 0xffff;   // (u16 m_t2)
}

static void check_irqs(mcs51_t *c)
{
	mcs51_state *s = &c->st;
	uint8_t ints = (uint8_t)(BIT(s->tcon, TCON_IE0) | BIT(s->tcon, TCON_TF0) << 1 | BIT(s->tcon, TCON_IE1) << 2 |
	                         BIT(s->tcon, TCON_TF1) << 3 | (BIT(s->scon, SCON_RI) | BIT(s->scon, SCON_TI)) << 4);
	uint8_t mask = BIT(s->ie, IE_A) ? s->ie : 0;
	if (c->is_8052)   // (i8052 irqs_complete_and_mask: TF2 / EXF2)
		ints |= (uint8_t)((BIT(c->st2.t2con, T2CON_TF2) | BIT(c->st2.t2con, T2CON_EXF2)) << 5);
	ints &= mask;
	if (!ints)
		return;
	int prio = -1;
	uint8_t vec = 0;
	for (int i = 0; i < (c->is_8052 ? 6 : 5); i++)
		if ((ints & (1 << i)) && s->irq_prio[i] > prio) {
			prio = s->irq_prio[i];
			vec = (uint8_t)((i << 3) | 3);
		}
	if (s->irq_active && prio <= s->cur_irq_prio)
		return;
	// MAME hack for the polling latency of JB INT0 / JB INT1
	if (vec == V_IE0 && s->last_op == 0x20 && s->last_bit == 0xb2)
		s->pc = (uint16_t)(s->ppc + 3);
	else if (vec == V_IE1 && s->last_op == 0x20 && s->last_bit == 0xb3)
		s->pc = (uint16_t)(s->ppc + 3);
	push_pc(c);
	s->pc = vec;
	s->inst_cycles += 2;
	s->cur_irq_prio = (int8_t)prio;
	s->irq_active |= (uint8_t)(1 << prio);
	switch (vec) {
	case V_IE0: if (BIT(s->tcon, TCON_IT0)) s->tcon &= (uint8_t)~(1 << TCON_IE0); break;
	case V_TF0: s->tcon &= (uint8_t)~(1 << TCON_TF0); break;
	case V_IE1: if (BIT(s->tcon, TCON_IT1)) s->tcon &= (uint8_t)~(1 << TCON_IE1); break;
	case V_TF1: s->tcon &= (uint8_t)~(1 << TCON_TF1); break;
	}
}

void mcs51_set_input(mcs51_t *c, int line, int state)
{
	mcs51_state *s = &c->st;
	uint32_t ns = (s->last_line_state & ~(1u << line)) | ((uint32_t)(state != 0) << line);
	uint32_t tr = ~s->last_line_state & ns;
	switch (line) {
	case MCS51_INT0_LINE:
		if (state) {
			if (!BIT(s->tcon, TCON_IT0) || BIT(tr, MCS51_INT0_LINE))
				s->tcon |= 1 << TCON_IE0;
		} else if (!BIT(s->tcon, TCON_IT0))
			s->tcon &= (uint8_t)~(1 << TCON_IE0);
		break;
	case MCS51_INT1_LINE:
		if (state) {
			if (!BIT(s->tcon, TCON_IT1) || BIT(tr, MCS51_INT1_LINE))
				s->tcon |= 1 << TCON_IE1;
		} else if (!BIT(s->tcon, TCON_IT1))
			s->tcon &= (uint8_t)~(1 << TCON_IE1);
		break;
	case MCS51_T0_LINE:
		if (BIT(tr, MCS51_T0_LINE) && BIT(s->tcon, TCON_TR0))
			s->t0_cnt++;
		break;
	case MCS51_T1_LINE:
		if (BIT(tr, MCS51_T1_LINE) && BIT(s->tcon, TCON_TR1))
			s->t1_cnt++;
		break;
	}
	s->last_line_state = ns;
}

int mcs51_execute(mcs51_t *c, int cycles)
{
	mcs51_state *s = &c->st;
	int start = s->icount + cycles;
	s->icount = start;
	while (s->icount > 0) {
		check_irqs(c);
		s->ppc = s->pc;
		uint8_t op = fetch(c);
		s->inst_cycles += s_cycles[op];
		execute_op(c, op);
		// MAME burn_cycles: the timers advance one machine cycle at a time
		for (int n = s->inst_cycles; n > 0; n--) {
			s->icount--;
			update_timer_t0(c, 1);
			update_timer_t1(c, 1);
			if (c->is_8052)
				update_timer_t2(c, 1);
		}
		s->inst_cycles = 0;
	}
	return start - s->icount;
}

void mcs51_init(mcs51_t *c, const uint8_t *rom, uint32_t rom_size)
{
	memset(c, 0, sizeof(*c));
	c->rom = rom;
	c->rom_size = rom_size;
}

void mcs51_reset(mcs51_t *c)
{
	mcs51_state *s = &c->st;
	uint8_t iram[128];
	memcpy(iram, s->iram, sizeof(iram));   // (RAM keeps its contents over a reset)
	memset(s, 0, sizeof(*s));
	memcpy(s->iram, iram, sizeof(iram));
	c->st2.t2con = 0;   // (i8052 / i80c52 device_reset; the upper RAM is kept too)
	c->st2.rcap2 = c->st2.t2 = 0;
	c->st2.saddr = c->st2.saden = 0;
	c->st2.iph = 0;
	s->cur_irq_prio = -1;
	s->sp = 7;
	sfr_w(c, 0xb0, 0xff);
	sfr_w(c, 0xa0, 0xff);
	sfr_w(c, 0x90, 0xff);
	sfr_w(c, 0x80, 0xff);
	update_irq_prio(c);
}
