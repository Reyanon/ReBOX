/*  Motorola MC68705 (HMOS M6805 family) microcontroller, ported from MAME
    src/devices/cpu/m6805/m6805.cpp, 6805ops.hxx, m6805defs.h (BSD-3-Clause, copyright-holders: Aaron Giles, Vas Crabb)
    and src/devices/cpu/m6805/m68705.cpp (BSD-3-Clause, copyright-holders: Vas Crabb).
    HD63705Z0: src/devices/cpu/m6805/hd6305.cpp / m6805.cpp s_hd63705_cycles (MAME 0.289, BSD-3-Clause,
    copyright-holders: Aaron Giles, Vas Crabb, Olivier Galibert).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "m68705.h"

#include <string.h>

enum { CFLAG = 0x01, ZFLAG = 0x02, NFLAG = 0x04, IFLAG = 0x08, HFLAG = 0x10 };

// timer options / TCR bits (MAME m6805_timer)
enum { TIMER_PGM = 0x01, TIMER_MOR = 0x02, TIMER_NPC = 0x04 };
enum { TCR_PS = 0x07, TCR_PSC = 0x08, TCR_TIE = 0x10, TCR_TIN = 0x20, TCR_TIM = 0x40, TCR_TIR = 0x80 };
enum { MOR_PS = 0x07, MOR_TIE = 0x10, MOR_CLS = 0x20, MOR_TOPT = 0x40 };

#define SP_MASK (c->sp_mask)
#define SP_LOW  (c->sp_low)

#define VECTOR_BOOTSTRAP 0xfff6
#define VECTOR_TIMER     0xfff8
#define VECTOR_INT       0xfffa
#define VECTOR_SWI       0xfffc
#define VECTOR_RESET     0xfffe

#define XX 4   // illegal opcode, unknown cycle count
static const uint8_t s_hmos_cycles[256] = {
	/* 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F */
	  10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
	   7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
	   4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
	   6,XX,XX, 6, 6,XX, 6, 6, 6, 6, 6,XX, 6, 6,XX, 6,
	   4,XX,XX, 4, 4,XX, 4, 4, 4, 4, 4,XX, 4, 4,XX, 4,
	   4,XX,XX, 4, 4,XX, 4, 4, 4, 4, 4,XX, 4, 4,XX, 4,
	   7,XX,XX, 7, 7,XX, 7, 7, 7, 7, 7,XX, 7, 7,XX, 7,
	   6,XX,XX, 6, 6,XX, 6, 6, 6, 6, 6,XX, 6, 6,XX, 6,
	   9, 6,XX,11,XX,XX,XX,XX,XX,XX,XX,XX,XX,XX,XX,XX,
	  XX,XX,XX,XX,XX,XX,XX, 2, 2, 2, 2, 2, 2, 2,XX, 2,
	   2, 2, 2, 2, 2, 2, 2,XX, 2, 2, 2, 2,XX, 8, 2,XX,
	   4, 4, 4, 4, 4, 4, 4, 5, 4, 4, 4, 4, 3, 7, 4, 5,
	   5, 5, 5, 5, 5, 5, 5, 6, 5, 5, 5, 5, 4, 8, 5, 6,
	   6, 6, 6, 6, 6, 6, 6, 7, 6, 6, 6, 6, 5, 9, 6, 7,
	   5, 5, 5, 5, 5, 5, 5, 6, 5, 5, 5, 5, 4, 8, 5, 6,
	   4, 4, 4, 4, 4, 4, 4, 5, 4, 4, 4, 4, 3, 7, 4, 5
};
// MAME m6805_base_device::s_hd63705_cycles
static const uint8_t s_hd63705_cycles[256] = {
	/* 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F */
	   5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
	   5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
	   3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
	   5,XX,XX, 5, 5,XX, 5, 5, 5, 5, 5,XX, 5, 4,XX, 5,
	   2,XX,11, 2, 2,XX, 2, 2, 2, 2, 2,XX, 2, 2,XX, 2,
	   2,XX,XX, 2, 2,XX, 2, 2, 2, 2, 2,XX, 2, 2,XX, 2,
	   6,XX,XX, 6, 6,XX, 6, 6, 6, 6, 6,XX, 6, 5,XX, 6,
	   5,XX,XX, 5, 5,XX, 5, 5, 5, 5, 5,XX, 5, 4,XX, 5,
	   9, 6,XX,11,XX,XX,XX,XX,XX,XX,XX,XX,XX, 2, 4, 4,
	  XX,XX,XX,XX,XX,XX,XX, 2, 1, 1, 2, 2, 2, 1,XX, 2,
	   2, 2, 2, 2, 2, 2, 2,XX, 2, 2, 2, 2,XX, 6, 2,XX,
	   3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 6, 3, 3,
	   4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 7, 4, 4,
	   5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 7, 5, 5,
	   4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 6, 4, 4,
	   3, 3, 3, 3, 3, 3, 3, 4, 3, 3, 3, 3, 2, 6, 3, 4
};
#undef XX

// ---- timer / counter (MAME m6805_timer) ----
static void set_input(m68705_t *c, int line, int state);

static void timer_tcr_w(m68705_t *c, uint8_t data)
{
	m68705_state *s = &c->st;
	if (s->t_options & TIMER_MOR)
		data |= TCR_TIE;
	if (s->t_options & TIMER_PGM) {
		s->t_divisor = data & TCR_PS;
		s->t_source = (uint8_t)((data & (TCR_TIN | TCR_TIE)) >> 4);
	}
	if ((data & TCR_PSC) && !(s->t_options & TIMER_NPC))
		s->t_prescale = 0;
	s->tcr = (uint8_t)((s->tcr & (data & TCR_TIR)) | (data & ~(TCR_TIR | TCR_PSC)));
	// level-sensitive interrupt
	set_input(c, M68705_INT_TIMER, (s->tcr & TCR_TIR) && !(s->tcr & TCR_TIM));
}

static uint8_t timer_tcr_r(const m68705_t *c)
{
	return (c->st.t_options & TIMER_MOR) ? (uint8_t)(c->st.tcr | TCR_PSC) : (uint8_t)(c->st.tcr & ~TCR_PSC);
}

static void timer_update(m68705_t *c, unsigned count)
{
	m68705_state *s = &c->st;
	if (s->t_source == M68705_TSRC_DISABLED || (s->t_source == M68705_TSRC_CLOCK_TIMER && !s->t_timer))
		return;
	unsigned prescale = (s->t_prescale & ((1u << s->t_divisor) - 1)) + ((s->t_source == M68705_TSRC_TIMER) ? s->t_edges : count);
	unsigned decrements = prescale >> s->t_divisor;
	int irq = (s->tdr ? (unsigned)s->tdr : 256u) <= decrements;
	s->t_prescale = prescale & 0x7f;
	s->tdr = (uint8_t)(s->tdr - decrements);
	s->t_edges = 0;
	if (irq) {
		s->tcr |= TCR_TIR;
		if (!(s->tcr & TCR_TIM))
			set_input(c, M68705_INT_TIMER, 1);
	}
}

void m68705_timer_w(m68705_t *c, int state)
{
	if (c->st.t_timer && !state)
		c->st.t_edges++;
	c->st.t_timer = state ? 1 : 0;
}

// ---- ports ----
static uint8_t port_r(m68705_t *c, int n)
{
	m68705_state *s = &c->st;
	if (c->port_r[n])
		s->port_input[n] = (uint8_t)(c->port_r[n](c->user, n, (uint8_t)(~s->port_ddr[n] & ~c->port_mask[n])) & ~c->port_mask[n]);
	return (uint8_t)(c->port_mask[n] | (s->port_latch[n] & s->port_ddr[n]) | (s->port_input[n] & ~s->port_ddr[n]));
}

static void port_cb_w(m68705_t *c, int n)
{
	m68705_state *s = &c->st;
	uint8_t data = c->open_drain[n] ? (uint8_t)(s->port_latch[n] | ~s->port_ddr[n]) : s->port_latch[n];
	uint8_t mask = c->open_drain[n] ? (uint8_t)(~s->port_latch[n] & s->port_ddr[n]) : s->port_ddr[n];
	if (c->port_w[n])
		c->port_w[n](c->user, n, data, mask);
}

static void port_latch_w(m68705_t *c, int n, uint8_t data)
{
	m68705_state *s = &c->st;
	data &= (uint8_t)~c->port_mask[n];
	uint8_t diff = s->port_latch[n] ^ data;
	s->port_latch[n] = data;
	if (diff & s->port_ddr[n])
		port_cb_w(c, n);
}

static void port_ddr_w(m68705_t *c, int n, uint8_t data)
{
	m68705_state *s = &c->st;
	data &= (uint8_t)~c->port_mask[n];
	if (data != s->port_ddr[n]) {
		s->port_ddr[n] = data;
		port_cb_w(c, n);
	}
}

void m68705_port_input(m68705_t *c, int n, uint8_t data)
{
	c->st.port_input[n] = (uint8_t)(data & ~c->port_mask[n]);
}

// ---- EPROM control ----
static int pcr_vpon(const m68705_t *c) { return !(c->st.pcr & 4); }
static int pcr_pge(const m68705_t *c) { return !(c->st.pcr & 2); }
static int pcr_ple(const m68705_t *c) { return !(c->st.pcr & 1); }

static void pcr_w(m68705_t *c, uint8_t data)
{
	data |= (uint8_t)((data & 0x01) << 1);   // lock out /PGE if /PLE is not asserted
	if (pcr_pge(c) && !(data & 2)) {
		if (pcr_vpon(c))
			c->rom[c->st.pl_addr & c->amask] |= c->st.pl_data;
	}
	c->st.pcr = (uint8_t)((c->st.pcr & 0xfc) | (data & 0x03));
}

static void eprom_w(m68705_t *c, uint16_t addr, uint8_t data)
{
	if (pcr_vpon(c) && pcr_ple(c) && !pcr_pge(c)) {
		c->st.pl_data = data;
		c->st.pl_addr = addr;
	}
}

// ---- memory map (MAME m6805_hmos_device / m68705p / m68705r internal_map) ----
static uint8_t reg_r(m68705_t *c, uint16_t a)
{
	int r3 = c->variant == M68705_R3;
	switch (a) {
	case 0x00: return port_r(c, 0);
	case 0x01: return port_r(c, 1);
	case 0x02: return port_r(c, 2);
	case 0x03: return r3 ? port_r(c, 3) : 0xff;
	case 0x08: return c->st.tdr;
	case 0x09: return timer_tcr_r(c);
	case 0x0a: return r3 ? c->st.mr : 0xff;
	case 0x0b: return c->st.pcr;
	case 0x0e: return r3 ? (uint8_t)(c->st.acr_mux | 0x80 | 0x78) : 0xff;
	case 0x0f:
		if (!r3) return 0xff;
		if (c->st.acr_mux < 4) return c->analog_r ? c->analog_r(c->user, c->st.acr_mux) : 0xff;
		return 0xff;
	}
	return 0xff;
}

static void reg_w(m68705_t *c, uint16_t a, uint8_t v)
{
	int r3 = c->variant == M68705_R3;
	switch (a) {
	case 0x00: port_latch_w(c, 0, v); break;
	case 0x01: port_latch_w(c, 1, v); break;
	case 0x02: port_latch_w(c, 2, v); break;
	case 0x03: if (r3) port_latch_w(c, 3, v); break;
	case 0x04: port_ddr_w(c, 0, v); break;
	case 0x05: port_ddr_w(c, 1, v); break;
	case 0x06: port_ddr_w(c, 2, v); break;
	case 0x08: c->st.tdr = v; break;
	case 0x09: timer_tcr_w(c, v); break;
	case 0x0a: if (r3) c->st.mr = v; break;
	case 0x0b: pcr_w(c, v); break;
	case 0x0e: if (r3) c->st.acr_mux = v & 7; break;
	}
}

static inline int eprom_range(const m68705_t *c, uint16_t a)
{
	// R3: 0xf39-0xf7f is not mapped
	return !(c->variant == M68705_R3 && a >= 0xf39 && a < 0xf80);
}

// HD63705Z0 (MAME hd63705z0_device::internal_map; the rest is the external bus)
static uint8_t z_rm(m68705_t *c, uint16_t a)
{
	if (a >= 0x0040 && a < 0x01c0)
		return c->zst.ram[a - 0x40];
	if (a >= 0x01c0 && a < 0x2000)
		return c->rom[a];
	return c->ext_r ? c->ext_r(c->user, a) : 0xff;
}

static void z_wm(m68705_t *c, uint16_t a, uint8_t v)
{
	if (a >= 0x0040 && a < 0x01c0)
		c->zst.ram[a - 0x40] = v;
	else if (a >= 0x2000 || a < 0x0040) {
		if (c->ext_w)
			c->ext_w(c->user, a, v);
	}
}

static inline uint8_t rm(m68705_t *c, uint16_t a)
{
	if (c->variant == M68705_HD63705Z0)
		return z_rm(c, a);
	a &= c->amask;
	if (a >= 0x80) {
		if (!eprom_range(c, a))
			return 0xff;
		// read locked out when /VPON and /PLE are asserted
		return (!pcr_vpon(c) || !pcr_ple(c)) ? c->rom[a] : 0xff;
	}
	if (a >= 0x10)
		return c->st.ram[a - 0x10];
	return reg_r(c, a);
}

static inline void wm(m68705_t *c, uint16_t a, uint8_t v)
{
	if (c->variant == M68705_HD63705Z0) {
		z_wm(c, a, v);
		return;
	}
	a &= c->amask;
	if (a >= 0x80) {
		if (eprom_range(c, a) && !(c->variant != M68705_R3 ? (a >= 0x785 && a < 0x7f8) : (a >= 0xf80 && a < 0xff8)))
			eprom_w(c, a, v);
		return;
	}
	if (a >= 0x10)
		c->st.ram[a - 0x10] = v;
	else
		reg_w(c, a, v);
}

// ---- helpers (m6805defs.h) ----
#define S_ (c->st)
#define PC (S_.pc)
#define SP (S_.s)
#define EA (S_.ea)
#define A (S_.a)
#define X (S_.x)
#define CC (S_.cc)

static inline uint8_t imm(m68705_t *c) { return rm(c, PC++); }
static inline uint16_t immw(m68705_t *c) { uint16_t h = imm(c); return (uint16_t)(h << 8 | imm(c)); }
static inline uint16_t rm16(m68705_t *c, uint16_t a) { return (uint16_t)(rm(c, a) << 8 | rm(c, (uint16_t)(a + 1))); }
static inline void pushbyte(m68705_t *c, uint8_t b) { wm(c, SP, b); if (--SP < SP_LOW) SP = SP_MASK; }
static inline void pushword(m68705_t *c, uint16_t w) { pushbyte(c, (uint8_t)w); pushbyte(c, (uint8_t)(w >> 8)); }
static inline uint8_t pullbyte(m68705_t *c) { if (++SP > SP_MASK) SP = SP_LOW; return rm(c, SP); }
static inline uint16_t pullword(m68705_t *c) { uint16_t h = pullbyte(c); return (uint16_t)(h << 8 | pullbyte(c)); }

#define SIGNED(b) ((int16_t)(int8_t)(b))
#define CLR_NZ   (CC &= (uint8_t)~(NFLAG | ZFLAG))
#define CLR_NZC  (CC &= (uint8_t)~(NFLAG | ZFLAG | CFLAG))
#define CLR_HNZC (CC &= (uint8_t)~(HFLAG | NFLAG | ZFLAG | CFLAG))
#define SET_Z8(a)  do { if (!(uint8_t)(a)) CC |= ZFLAG; } while (0)
#define SET_N8(a)  (CC |= (uint8_t)(((a) & 0x80) >> 5))
#define SET_NZ8(a) do { SET_N8(a); SET_Z8(a); } while (0)
#define SET_C8(a)  (CC |= (uint8_t)(((a) >> 8) & 1))
#define SET_NZC8(a) do { SET_NZ8(a); SET_C8(a); } while (0)
#define SET_H(a, b, r) (CC |= (uint8_t)(((a) ^ (b) ^ (r)) & 0x10))

enum { AM_IM, AM_DI, AM_EX, AM_IX, AM_IX1, AM_IX2 };

static inline void argaddr(m68705_t *c, int m)
{
	switch (m) {
	case AM_DI: EA = imm(c); break;
	case AM_EX: EA = immw(c); break;
	case AM_IX: EA = X; break;
	case AM_IX1: EA = (uint16_t)(imm(c) + X); break;
	case AM_IX2: EA = (uint16_t)(immw(c) + X); break;
	default: break;
	}
}

static inline uint8_t argbyte(m68705_t *c, int m)
{
	if (m == AM_IM)
		return imm(c);
	argaddr(c, m);
	return rm(c, EA);
}

// ---- interrupts ----
static void set_input(m68705_t *c, int line, int state)
{
	// m6805_hmos_device::execute_set_input
	state = state ? 1 : 0;
	if (c->st.irq_state[line] != state) {
		c->st.irq_state[line] = (uint8_t)state;
		if (state)
			c->st.pending |= (uint8_t)(1 << line);
		else if (line == M68705_INT_TIMER)
			c->st.pending &= (uint8_t)~(1 << line);   // level-sensitive
	}
}

void m68705_set_input(m68705_t *c, int line, int state)
{
	switch (line) {
	case M68705_VPP_LINE:
		c->st.pcr = (uint8_t)((c->st.pcr & 0xfb) | (state ? 0x00 : 0x04));
		break;
	case M68705_VIHTP_LINE:
		c->st.vihtp = state ? 1 : 0;
		break;
	default:
		set_input(c, line, state);
		break;
	}
}

void m68705_set_irq_line(m68705_t *c, int state)
{
	if (c->variant == M68705_HD63705Z0)
		m68705_hd63705_set_input(c, HD63705_INT_IRQ1, state);
	else
		set_input(c, M68705_IRQ_LINE, state);
}

static void interrupt(m68705_t *c)
{
	if ((S_.pending & 0x03) && !(CC & IFLAG)) {
		pushword(c, PC);
		pushbyte(c, X);
		pushbyte(c, A);
		pushbyte(c, CC);
		CC |= IFLAG;
		if (S_.pending & 1) {
			S_.pending &= (uint8_t)~1;
			PC = rm16(c, VECTOR_INT & c->amask);
		} else {
			PC = rm16(c, VECTOR_TIMER & c->amask);
		}
		S_.icount -= 11;
		timer_update(c, 11);
	}
}

// ---- HD63705Z0 interrupts (MAME hd63705z0_device::execute_set_input / interrupt / interrupt_vector) ----
void m68705_hd63705_set_input(m68705_t *c, int line, int state)
{
	m68705_zstate *z = &c->zst;
	if (line == HD63705_INT_NMI) {
		if (z->nmi_state != state) {
			z->nmi_state = (uint8_t)state;
			if (state != M68705_CLEAR_LINE)
				z->pending |= 1u << HD63705_INT_NMI;
		}
	} else if (line >= 0 && line <= HD63705_INT_ADCONV) {
		if (z->irq_state[line] != state) {
			z->irq_state[line] = (uint8_t)state;
			if (state != M68705_CLEAR_LINE)
				z->pending |= (uint16_t)(1u << line);
		}
	}
}

static void z_push_all(m68705_t *c)
{
	pushword(c, PC);
	pushbyte(c, X);
	pushbyte(c, A);
	pushbyte(c, CC);
	CC |= IFLAG;
	// standard_irq_callback(0): a HOLD_LINE on line 0 clears itself
	if (c->zst.irq_state[0] == M68705_HOLD_LINE)
		c->zst.irq_state[0] = M68705_CLEAR_LINE;
}

static void z_interrupt(m68705_t *c)
{
	static const struct { uint8_t line; uint16_t vec; } order[] = {
		{ HD63705_INT_IRQ1, 0x1ff8 }, { HD63705_INT_IRQ2, 0x1fec }, { HD63705_INT_ADCONV, 0x1fea },
		{ HD63705_INT_TIMER1, 0x1ff6 }, { HD63705_INT_TIMER2, 0x1ff4 }, { HD63705_INT_TIMER3, 0x1ff2 },
		{ HD63705_INT_PCI, 0x1ff0 }, { HD63705_INT_SCI, 0x1fee }
	};
	m68705_zstate *z = &c->zst;
	if (z->pending & (1u << HD63705_INT_NMI)) {
		z_push_all(c);
		PC = rm16(c, 0x1ffc);
		z->pending &= (uint16_t)~(1u << HD63705_INT_NMI);
		S_.icount -= 11;
	} else if ((z->pending & 0x1ff) && !(CC & IFLAG)) {
		z_push_all(c);
		for (unsigned i = 0; i < sizeof(order) / sizeof(order[0]); i++) {
			if (z->pending & (1u << order[i].line)) {
				z->pending &= (uint16_t)~(1u << order[i].line);
				PC = rm16(c, order[i].vec);
				break;
			}
		}
		z->pending &= (uint16_t)~1u;   // M6805_IRQ_LINE
		S_.icount -= 11;
	}
}

// ---- instructions (6805ops.hxx) ----
// read-modify-write group: 0x3x direct, 0x4x A, 0x5x X, 0x6x indexed+1, 0x7x indexed; returns 0 for an illegal opcode
static int rmw(m68705_t *c, uint8_t op)
{
	int lo = op & 0x0f;
	if (lo == 1 || lo == 2 || lo == 5 || lo == 0xb || lo == 0xe)
		return 0;
	int grp = op >> 4;
	int mode = grp == 3 ? AM_DI : grp == 6 ? AM_IX1 : AM_IX;
	uint16_t t;
	if (grp == 4) t = A;
	else if (grp == 5) t = X;
	else if (lo == 0xf) { argaddr(c, mode); t = 0; }   // CLR does not read
	else t = argbyte(c, mode);
	uint16_t r;
	switch (lo) {
	case 0x0: r = (uint16_t)-t; CLR_NZC; SET_NZC8(r); break;                              // NEG
	case 0x3: r = (uint8_t)~t; CLR_NZ; SET_NZ8(r); CC |= CFLAG; break;                    // COM
	case 0x4: CLR_NZC; CC |= t & 1; r = t >> 1; SET_Z8(r); break;                         // LSR
	case 0x6: r = (uint16_t)((CC & 1) << 7); CLR_NZC; CC |= t & 1; r |= t >> 1; SET_NZ8(r); break;   // ROR
	case 0x7: CLR_NZC; CC |= t & 1; r = (uint16_t)((t >> 1) | (t & 0x80)); SET_NZ8(r); break;        // ASR
	case 0x8: r = (uint16_t)(t << 1); CLR_NZC; SET_NZC8(r); break;                        // LSL
	case 0x9: r = (uint16_t)((CC & 1) | (t << 1)); CLR_NZC; SET_NZC8(r); break;           // ROL
	case 0xa: r = (uint8_t)(t - 1); CLR_NZ; SET_NZ8(r); break;                            // DEC
	case 0xc: r = (uint8_t)(t + 1); CLR_NZ; SET_NZ8(r); break;                            // INC
	case 0xd: CLR_NZ; SET_NZ8(t); return 1;                                               // TST
	default:  r = 0; CLR_NZ; CC |= ZFLAG; break;                                          // CLR
	}
	if (grp == 4) A = (uint8_t)r;
	else if (grp == 5) X = (uint8_t)r;
	else wm(c, EA, (uint8_t)r);
	return 1;
}

static void alu(m68705_t *c, uint8_t op)
{
	static const uint8_t modes[6] = { AM_IM, AM_DI, AM_EX, AM_IX2, AM_IX1, AM_IX };
	int m = modes[(op >> 4) - 0xa];
	int lo = op & 0x0f;
	if (m == AM_IM && (lo == 0x7 || lo == 0xc || lo == 0xf))
		return;   // illegal
	if (m == AM_IM && lo == 0xd) {   // BSR
		uint8_t t = imm(c);
		pushword(c, PC);
		PC = (uint16_t)(PC + SIGNED(t));
		return;
	}
	uint16_t t, r;
	switch (lo) {
	case 0x0: t = argbyte(c, m); r = (uint16_t)(A - t); CLR_NZC; SET_NZC8(r); A = (uint8_t)r; break;              // SUB
	case 0x1: t = argbyte(c, m); r = (uint16_t)(A - t); CLR_NZC; SET_NZC8(r); break;                              // CMP
	case 0x2: t = argbyte(c, m); r = (uint16_t)(A - t - (CC & 1)); CLR_NZC; SET_NZC8(r); A = (uint8_t)r; break;   // SBC
	case 0x3: t = argbyte(c, m); r = (uint16_t)(X - t); CLR_NZC; SET_NZC8(r); break;                              // CPX
	case 0x4: A &= argbyte(c, m); CLR_NZ; SET_NZ8(A); break;                                                      // AND
	case 0x5: r = A & argbyte(c, m); CLR_NZ; SET_NZ8(r); break;                                                   // BIT
	case 0x6: A = argbyte(c, m); CLR_NZ; SET_NZ8(A); break;                                                       // LDA
	case 0x7: CLR_NZ; SET_NZ8(A); argaddr(c, m); wm(c, EA, A); break;                                             // STA
	case 0x8: A ^= argbyte(c, m); CLR_NZ; SET_NZ8(A); break;                                                      // EOR
	case 0x9: t = argbyte(c, m); r = (uint16_t)(A + t + (CC & 1)); CLR_HNZC; SET_H(A, t, r); SET_NZC8(r); A = (uint8_t)r; break;   // ADC
	case 0xa: A |= argbyte(c, m); CLR_NZ; SET_NZ8(A); break;                                                      // ORA
	case 0xb: t = argbyte(c, m); r = (uint16_t)(A + t); CLR_HNZC; SET_H(A, t, r); SET_NZC8(r); A = (uint8_t)r; break;              // ADD
	case 0xc: argaddr(c, m); PC = EA; break;                                                                      // JMP
	case 0xd: argaddr(c, m); pushword(c, PC); PC = EA; break;                                                     // JSR
	case 0xe: X = argbyte(c, m); CLR_NZ; SET_NZ8(X); break;                                                       // LDX
	default:  CLR_NZ; SET_NZ8(X); argaddr(c, m); wm(c, EA, X); break;                                             // STX
	}
}

static void branch(m68705_t *c, uint8_t op)
{
	int f;
	switch ((op >> 1) & 7) {
	case 0: f = 1; break;                                   // BRA / BRN
	case 1: f = !(CC & (CFLAG | ZFLAG)); break;             // BHI / BLS
	case 2: f = !(CC & CFLAG); break;                       // BCC / BCS
	case 3: f = !(CC & ZFLAG); break;                       // BNE / BEQ
	case 4: f = !(CC & HFLAG); break;                       // BHCC / BHCS
	case 5: f = !(CC & NFLAG); break;                       // BPL / BMI
	case 6: f = !(CC & IFLAG); break;                       // BMC / BMS
	default:                                                // BIL / BIH (HD6305: the NMI line)
		f = c->variant == M68705_HD63705Z0 ? c->zst.nmi_state != 0 : S_.irq_state[M68705_IRQ_LINE] != 0;
		break;
	}
	uint8_t t = imm(c);
	if (f == !(op & 1))
		PC = (uint16_t)(PC + SIGNED(t));
}

static void exec_op(m68705_t *c, uint8_t op)
{
	switch (op >> 4) {
	case 0x0: {   // BRSET / BRCLR
		int b = (op >> 1) & 7;
		EA = imm(c);
		uint8_t r = rm(c, EA);
		uint8_t t = imm(c);
		if (!(op & 1)) { CC &= (uint8_t)~CFLAG; if ((r >> b) & 1) { CC |= CFLAG; PC = (uint16_t)(PC + SIGNED(t)); } }
		else { CC |= CFLAG; if (!((r >> b) & 1)) { CC &= (uint8_t)~CFLAG; PC = (uint16_t)(PC + SIGNED(t)); } }
		break;
	}
	case 0x1: {   // BSET / BCLR
		int b = (op >> 1) & 7;
		EA = imm(c);
		uint8_t t = rm(c, EA);
		wm(c, EA, (op & 1) ? (uint8_t)(t & ~(1 << b)) : (uint8_t)(t | (1 << b)));
		break;
	}
	case 0x2: branch(c, op); break;
	case 0x3: case 0x4: case 0x5: case 0x6: case 0x7: rmw(c, op); break;
	case 0x8:
		switch (op) {
		case 0x80: CC = pullbyte(c); A = pullbyte(c); X = pullbyte(c); PC = pullword(c); break;   // RTI
		case 0x81: PC = pullword(c); break;                                                    // RTS
		case 0x83:                                                                              // SWI
			pushword(c, PC); pushbyte(c, X); pushbyte(c, A); pushbyte(c, CC);
			CC |= IFLAG;
			PC = rm16(c, c->variant == M68705_HD63705Z0 ? 0x1ffa : (VECTOR_SWI & c->amask));
			break;
		}
		break;
	case 0x9:
		switch (op) {
		case 0x97: X = A; break;
		case 0x98: CC &= (uint8_t)~CFLAG; break;
		case 0x99: CC |= CFLAG; break;
		case 0x9a: CC &= (uint8_t)~IFLAG; break;
		case 0x9b: CC |= IFLAG; break;
		case 0x9c: SP = SP_MASK; break;
		case 0x9f: A = X; break;
		}
		break;
	default: alu(c, op); break;
	}
}

// ---- public ----
int m68705_init(m68705_t *c, int variant, uint8_t *rom, uint32_t size)
{
	memset(c, 0, sizeof(*c));
	c->variant = variant;
	c->rom = rom;
	c->sp_mask = 0x007f;
	c->sp_low = 0x0060;
	if (variant == M68705_HD63705Z0) {
		// configuration_params { s_hmos_b_ops, s_hd63705_cycles, 16, 0x017f, 0x0100, 0x1fff, 0x1ffa }
		c->sp_mask = 0x017f;
		c->sp_low = 0x0100;
		c->amask = 0xffff;
		if (!rom || size < 0x2000)
			return -1;
		c->st.t_source = M68705_TSRC_DISABLED;   // no internal timer (MAME TODO)
		return 0;
	}
	if (variant == M68705_R3) {
		c->amask = 0x0fff;
		c->open_drain[0] = 1;
	} else {
		c->amask = 0x07ff;
		c->open_drain[0] = 1;   // port A is open drain with internal pull-ups
		c->port_mask[2] = 0xf0; // port C is four bits wide
		c->port_mask[3] = 0xff; // no port D
	}
	if (!rom || size < (uint32_t)c->amask + 1)
		return -1;
	m68705_state *s = &c->st;
	// m6805_hmos_device constructor / device_start
	for (int i = 0; i < 4; i++) {
		s->port_input[i] = 0xff;
		s->port_latch[i] = 0xff;
		s->port_ddr[i] = 0x00;
	}
	s->t_divisor = 7;
	s->t_source = M68705_TSRC_CLOCK;
	s->t_timer = 1;
	// m68705_device::device_start: timer from the mask option register
	uint8_t options = variant == M68705_P5 ? rom[0x784] : variant == M68705_P3 ? (uint8_t)(rom[0x784] & 0xf7) : (uint8_t)(rom[0xf38] & 0xf7);
	if (options & MOR_TOPT) {
		s->t_options = TIMER_MOR;
		s->t_divisor = options & MOR_PS;
		s->t_source = (uint8_t)((options & (MOR_CLS | MOR_TIE)) >> 4);
	} else {
		s->t_options = TIMER_PGM;
	}
	s->vihtp = 0;
	s->pcr = 0xff;
	s->pl_data = 0xff;
	s->pl_addr = 0xffff;
	return 0;
}

void m68705_reset(m68705_t *c)
{
	m68705_state *s = &c->st;
	// m6805_base_device::device_reset
	s->ea = 0;
	s->pc = 0;
	s->s = SP_MASK;
	s->a = s->x = 0;
	s->cc = IFLAG;
	s->pending = 0;
	if (c->variant == M68705_HD63705Z0) {
		// m6805_base_device::device_reset (irq_state is only cleared at device_start in MAME)
		c->zst.pending = 0;
		c->zst.nmi_state = 0;
		s->pc = rm16(c, 0x1ffe);
		return;
	}
	// m6805_hmos_device::device_reset
	for (int i = 0; i < 4; i++)
		port_ddr_w(c, i, 0x00);
	s->t_edges = 0;
	s->t_prescale = 0x7f;
	s->tdr = 0xff;
	s->tcr = 0x7f;
	// m68705_device::device_reset
	s->pcr |= 0xfb;   // /VPON is driven externally
	s->pc = rm16(c, (s->vihtp ? VECTOR_BOOTSTRAP : VECTOR_RESET) & c->amask);
}

void m68705_set_reset_line(m68705_t *c, int state)
{
	if (state) {
		m68705_reset(c);
		c->st.in_reset = 1;
	} else {
		c->st.in_reset = 0;
	}
}

int m68705_execute(m68705_t *c, int cycles)
{
	m68705_state *s = &c->st;
	if (s->in_reset)
		return cycles;
	int start = s->icount + cycles;
	s->icount = start;
	if (s->icount <= 0)
		return 0;
	SP = (uint16_t)((SP & SP_MASK) | SP_LOW);
	if (c->variant == M68705_HD63705Z0) {
		do {
			if (c->zst.pending)
				z_interrupt(c);
			uint8_t op = rm(c, PC++);
			exec_op(c, op);
			s->icount -= (int32_t)s_hd63705_cycles[op];
		} while (s->icount > 0);
		return start - s->icount;
	}
	do {
		if (s->pending)
			interrupt(c);
		uint8_t op = rm(c, PC++);
		exec_op(c, op);
		unsigned n = s_hmos_cycles[op];
		s->icount -= (int32_t)n;
		timer_update(c, n);
	} while (s->icount > 0);
	return start - s->icount;
}
