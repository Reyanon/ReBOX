/*  Hudson HuC6280 CPU, ported to C from MAME 0.289 src/devices/cpu/h6280 (h6280.cpp, h6280.h;
    BSD-3-Clause, copyright-holders: Bryan McPhail; based on the 6502 emulator by Juergen Buchmueller).
    The opcode handlers keep MAME's order of operations (cycle charges before the bus accesses, operand read
    before the T flag read, etc.) so the time a memory handler sees matches MAME. Non-lazy flags
    (H6280_LAZY_FLAGS 0, as MAME builds it).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "h6280.h"

#include <string.h>

int h6280_break;

enum { fC = 0x01, fZ = 0x02, fI = 0x04, fD = 0x08, fB = 0x10, fT = 0x20, fV = 0x40, fN = 0x80 };

enum {
	RESET_VEC = 0xfffe,
	NMI_VEC = 0xfffc,
	TIMER_VEC = 0xfffa,
	IRQ1_VEC = 0xfff8,
	IRQ2_VEC = 0xfff6   // aka BRK vector
};

#define A  c->a
#define X  c->x
#define Y  c->y
#define P  c->p
#define S  c->s
#define PC c->pc
#define EA c->ea

static void set_irq_line(h6280_t *c, int line, int state);

/* ---- bus ---- */

static inline void cyc(h6280_t *c, int n)
{
	const int t = n * c->clocks_per_cycle;
	c->cycles += t;
	c->timer_value -= t;
}

static inline void check_irq_lines(h6280_t *c)
{
	if (!c->irq_pending)
		c->irq_pending = 2;
}

static uint8_t irq_status_r(h6280_t *c, uint32_t offset)
{
	switch (offset & 3) {
	default:
		return c->io_buffer;
	case 3: {
		int status = 0;
		if (c->irq_state[1])
			status |= 1;   // IRQ 2
		if (c->irq_state[0])
			status |= 2;   // IRQ 1
		if (c->irq_state[2])
			status |= 4;   // TIMER
		// MAME: m_io_buffer & ~H6280_IRQ_MASK (that enum value is 7)
		return (uint8_t)(status | (c->io_buffer & ~7));
	}
	case 2:
		return (uint8_t)(c->irq_mask | (c->io_buffer & ~7));
	}
}

static void irq_status_w(h6280_t *c, uint32_t offset, uint8_t data)
{
	c->io_buffer = data;
	switch (offset & 3) {
	default:
		break;
	case 2:   // write irq mask
		c->irq_mask = data & 0x7;
		check_irq_lines(c);
		break;
	case 3:   // timer irq ack
		set_irq_line(c, 2, 0);
		break;
	}
}

static uint8_t timer_r(h6280_t *c)
{
	// only returns countdown
	return (uint8_t)(((c->timer_value >> 10) & 0x7f) | (c->io_buffer & 0x80));
}

static void timer_w(h6280_t *c, uint32_t offset, uint8_t data)
{
	c->io_buffer = data;
	if (!(offset & 1)) {
		// counter preload: latched only on the 0->1 enable transition
		c->timer_load = ((data & 127) + 1) * 1024 * c->timer_scale;
	} else {
		// counter enable: stop -> start causes reload
		if ((data & 1) && c->timer_status == 0)
			c->timer_value = c->timer_load;
		c->timer_status = data & 1;
	}
}

static uint8_t io_read(h6280_t *c, uint32_t pa)
{
	if ((pa >> 13) == 0xff) {
		switch ((pa >> 10) & 7) {
		case 2: return c->io_buffer;                                   // PSG
		case 3: return timer_r(c);
		case 4: return c->port_in ? c->port_in(c->ctx) : c->io_buffer;
		case 5: return irq_status_r(c, pa);
		default: break;
		}
	}
	return c->read ? c->read(c->ctx, pa) : 0;
}

static void io_write(h6280_t *c, uint32_t pa, uint8_t v)
{
	if ((pa >> 13) == 0xff) {
		switch ((pa >> 10) & 7) {
		case 2:
			c->io_buffer = v;
			if (c->psg_write)
				c->psg_write(c->ctx, (int)(pa & 0xf), v);
			return;
		case 3: timer_w(c, pa, v); return;
		case 4:
			c->io_buffer = v;
			if (c->port_out)
				c->port_out(c->ctx, v);
			return;
		case 5: irq_status_w(c, pa, v); return;
		default: break;
		}
	}
	if (c->write)
		c->write(c->ctx, pa, v);
}

// physical access (m_program.read_byte / write_byte)
static inline uint8_t prd(h6280_t *c, uint32_t pa)
{
	const uint8_t *p = c->rpage[pa >> 13];
	return p ? p[pa & 0x1fff] : io_read(c, pa);
}

static inline void pwr(h6280_t *c, uint32_t pa, uint8_t v)
{
	uint8_t *p = c->wpage[pa >> 13];
	if (p)
		p[pa & 0x1fff] = v;
	else
		io_write(c, pa, v);
}

static inline uint32_t tr(const h6280_t *c, uint16_t a) { return h6280_translate(c, a); }

// program_read8 / program_write8: with the VDC / VCE penalty
static inline uint8_t rd8(h6280_t *c, uint16_t a)
{
	const uint32_t pa = tr(c, a);
	if ((pa & 0x1ff800) == 0x1fe000)
		cyc(c, 1);
	return prd(c, pa);
}

static inline void wr8(h6280_t *c, uint16_t a, uint8_t v)
{
	const uint32_t pa = tr(c, a);
	if ((pa & 0x1ff800) == 0x1fe000)
		cyc(c, 1);
	pwr(c, pa, v);
}

// zero page (through MPR1)
static inline uint8_t rd8z(h6280_t *c, uint16_t a) { return prd(c, ((uint32_t)c->mmr[1] << 13) | (a & 0x1fff)); }
static inline void wr8z(h6280_t *c, uint16_t a, uint8_t v) { pwr(c, ((uint32_t)c->mmr[1] << 13) | (a & 0x1fff), v); }

static inline uint16_t rd16(h6280_t *c, uint16_t a)
{
	return (uint16_t)(prd(c, tr(c, a)) | (prd(c, tr(c, (uint16_t)(a + 1))) << 8));
}

static inline uint16_t rd16z(h6280_t *c, uint16_t a)
{
	const uint32_t base = (uint32_t)c->mmr[1] << 13;
	if ((a & 0xff) == 0xff)
		return (uint16_t)(prd(c, base | (a & 0x1fff)) | (prd(c, base | ((a - 0xff) & 0x1fff)) << 8));
	return (uint16_t)(prd(c, base | (a & 0x1fff)) | (prd(c, base | ((a + 1) & 0x1fff)) << 8));
}

static inline void push(h6280_t *c, uint8_t v)
{
	pwr(c, ((uint32_t)c->mmr[1] << 13) | 0x100 | S, v);
	S--;
}

static inline uint8_t pull(h6280_t *c)
{
	S++;
	return prd(c, ((uint32_t)c->mmr[1] << 13) | 0x100 | S);
}

static inline uint8_t arg(h6280_t *c) { return prd(c, tr(c, PC)); }

/* ---- flags ---- */

static inline void set_nz(h6280_t *c, uint8_t n)
{
	P = (uint8_t)((P & ~(fN | fT | fZ)) | (n & fN) | (n == 0 ? fZ : 0));
}

static inline void clear_t(h6280_t *c) { P &= (uint8_t)~fT; }

/* ---- interrupts ---- */

static void do_interrupt(h6280_t *c, uint16_t vector)
{
	cyc(c, 7);   // 7 cycles for an int
	push(c, (uint8_t)(PC >> 8));
	push(c, (uint8_t)PC);
	P &= (uint8_t)~fB;   // compose_p(0, _fB)
	push(c, P);
	P = (uint8_t)((P & ~fD) | fI);
	uint8_t lo = rd8(c, vector);
	uint8_t hi = rd8(c, (uint16_t)(vector + 1));
	PC = (uint16_t)(lo | hi << 8);
}

static void check_and_take_irq_lines(h6280_t *c)
{
	if (c->nmi_state) {
		c->nmi_state = 0;
		do_interrupt(c, NMI_VEC);
	} else if (!(P & fI)) {
		if (c->irq_state[2] && !(c->irq_mask & 0x4)) {
			do_interrupt(c, TIMER_VEC);
		} else if (c->irq_state[0] && !(c->irq_mask & 0x2)) {
			if (c->irq_ack)
				c->irq_ack(c->ctx, H6280_IRQ1);
			do_interrupt(c, IRQ1_VEC);
		} else if (c->irq_state[1] && !(c->irq_mask & 0x1)) {
			if (c->irq_ack)
				c->irq_ack(c->ctx, H6280_IRQ2);
			do_interrupt(c, IRQ2_VEC);
		}
	}
}

static void set_irq_line(h6280_t *c, int line, int state)
{
	state = state ? 1 : 0;
	if (c->irq_state[line] == state)
		return;
	c->irq_state[line] = (uint8_t)state;
	check_irq_lines(c);
}

/* ---- effective addresses ---- */

static inline void ea_zpg(h6280_t *c) { EA = arg(c); PC++; }
static inline void ea_tflg(h6280_t *c) { EA = X; }
static inline void ea_zpx(h6280_t *c) { EA = (uint8_t)(arg(c) + X); PC++; }
static inline void ea_zpy(h6280_t *c) { EA = (uint8_t)(arg(c) + Y); PC++; }
static inline void ea_abs(h6280_t *c)
{
	uint8_t lo = arg(c);
	PC++;
	uint8_t hi = arg(c);
	PC++;
	EA = (uint16_t)(lo | hi << 8);
}
static inline void ea_abx(h6280_t *c) { ea_abs(c); EA = (uint16_t)(EA + X); }
static inline void ea_aby(h6280_t *c) { ea_abs(c); EA = (uint16_t)(EA + Y); }
static inline void ea_zpi(h6280_t *c) { uint8_t z = arg(c); PC++; EA = rd16z(c, z); }
static inline void ea_idx(h6280_t *c) { uint8_t z = (uint8_t)(arg(c) + X); PC++; EA = rd16z(c, z); }
static inline void ea_idy(h6280_t *c) { uint8_t z = arg(c); PC++; EA = (uint16_t)(rd16z(c, z) + Y); }
static inline void ea_ind(h6280_t *c)
{
	ea_abs(c);
	uint8_t tmp = rd8(c, EA);
	EA++;
	uint8_t hi = rd8(c, EA);
	EA = (uint16_t)(tmp | hi << 8);
}
static inline void ea_iax(h6280_t *c)
{
	ea_abs(c);
	EA = (uint16_t)(EA + X);
	uint8_t tmp = rd8(c, EA);
	EA++;
	uint8_t hi = rd8(c, EA);
	EA = (uint16_t)(tmp | hi << 8);
}

static inline uint8_t rd_imm(h6280_t *c) { uint8_t t = arg(c); PC++; return t; }
static inline uint8_t rd_zpg(h6280_t *c) { ea_zpg(c); return rd8z(c, EA); }
static inline uint8_t rd_zpx(h6280_t *c) { ea_zpx(c); return rd8z(c, EA); }
static inline uint8_t rd_zpy(h6280_t *c) { ea_zpy(c); return rd8z(c, EA); }
static inline uint8_t rd_abs(h6280_t *c) { ea_abs(c); return rd8(c, EA); }
static inline uint8_t rd_abx(h6280_t *c) { ea_abx(c); return rd8(c, EA); }
static inline uint8_t rd_aby(h6280_t *c) { ea_aby(c); return rd8(c, EA); }
static inline uint8_t rd_zpi(h6280_t *c) { ea_zpi(c); return rd8(c, EA); }
static inline uint8_t rd_idx(h6280_t *c) { ea_idx(c); return rd8(c, EA); }
static inline uint8_t rd_idy(h6280_t *c) { ea_idy(c); return rd8(c, EA); }
static inline uint8_t rd_tfl(h6280_t *c) { ea_tflg(c); return rd8z(c, EA); }

static inline void wb_ea(h6280_t *c, uint8_t v) { wr8(c, EA, v); }
static inline void wb_eaz(h6280_t *c, uint8_t v) { wr8z(c, EA, v); }
static inline void wr_zpg(h6280_t *c, uint8_t v) { ea_zpg(c); wb_eaz(c, v); }
static inline void wr_zpx(h6280_t *c, uint8_t v) { ea_zpx(c); wb_eaz(c, v); }
static inline void wr_zpy(h6280_t *c, uint8_t v) { ea_zpy(c); wb_eaz(c, v); }
static inline void wr_abs(h6280_t *c, uint8_t v) { ea_abs(c); wb_ea(c, v); }
static inline void wr_abx(h6280_t *c, uint8_t v) { ea_abx(c); wb_ea(c, v); }
static inline void wr_aby(h6280_t *c, uint8_t v) { ea_aby(c); wb_ea(c, v); }
static inline void wr_zpi(h6280_t *c, uint8_t v) { ea_zpi(c); wb_ea(c, v); }
static inline void wr_idx(h6280_t *c, uint8_t v) { ea_idx(c); wb_ea(c, v); }
static inline void wr_idy(h6280_t *c, uint8_t v) { ea_idy(c); wb_ea(c, v); }

/* ---- operations ---- */

static void bra(h6280_t *c, int cond)
{
	clear_t(c);
	if (cond) {
		cyc(c, 4);
		uint8_t tmp = arg(c);
		PC++;
		PC = (uint16_t)(PC + (int8_t)tmp);
	} else {
		PC++;
		cyc(c, 2);
	}
}

// decimal / binary add of a and tmp with C; sets C (and V when binary); returns the result
static inline uint8_t add_core(h6280_t *c, int a, uint8_t tmp)
{
	if (P & fD) {
		int cy = P & fC;
		int lo = (a & 0x0f) + (tmp & 0x0f) + cy;
		int hi = (a & 0xf0) + (tmp & 0xf0);
		P &= (uint8_t)~fC;
		if (lo > 0x09) {
			hi += 0x10;
			lo += 0x06;
		}
		if (hi > 0x90)
			hi += 0x60;
		if (hi & 0xff00)
			P |= fC;
		cyc(c, 1);
		return (uint8_t)((lo & 0x0f) + (hi & 0xf0));
	} else {
		int cy = P & fC;
		int sum = a + tmp + cy;
		P &= (uint8_t)~(fV | fC);
		if (~(a ^ tmp) & (a ^ sum) & fN)
			P |= fV;
		if (sum & 0xff00)
			P |= fC;
		return (uint8_t)sum;
	}
}

static inline uint8_t sub_core(h6280_t *c, int a, uint8_t tmp)
{
	if (P & fD) {
		int cy = (P & fC) ^ fC;
		int sum = a - tmp - cy;
		int lo = (a & 0x0f) - (tmp & 0x0f) - cy;
		int hi = (a & 0xf0) - (tmp & 0xf0);
		P &= (uint8_t)~fC;
		if (lo & 0xf0)
			lo -= 6;
		if (lo & 0x80)
			hi -= 0x10;
		if (hi & 0x0f00)
			hi -= 0x60;
		if ((sum & 0xff00) == 0)
			P |= fC;
		cyc(c, 1);
		return (uint8_t)((lo & 0x0f) + (hi & 0xf0));
	} else {
		int cy = (P & fC) ^ fC;
		int sum = a - tmp - cy;
		P &= (uint8_t)~(fV | fC);
		if ((a ^ tmp) & (a ^ sum) & fN)
			P |= fV;
		if ((sum & 0xff00) == 0)
			P |= fC;
		return (uint8_t)sum;
	}
}

static void adc(h6280_t *c, uint8_t tmp)
{
	if (P & fT) {
		clear_t(c);
		int t = rd_tfl(c);
		t = add_core(c, t, tmp);
		set_nz(c, (uint8_t)t);
		wb_eaz(c, (uint8_t)t);
		cyc(c, 3);
	} else {
		A = add_core(c, A, tmp);
		set_nz(c, A);
	}
}

static void sbc(h6280_t *c, uint8_t tmp)
{
	if (P & fT) {
		clear_t(c);
		int t = rd_tfl(c);
		t = sub_core(c, t, tmp);
		set_nz(c, (uint8_t)t);
		wb_eaz(c, (uint8_t)t);
		cyc(c, 3);
	} else {
		A = sub_core(c, A, tmp);
		set_nz(c, A);
	}
}

// AND / EOR / ORA: op 0 and, 1 eor, 2 ora
static inline uint8_t logic(int op, uint8_t a, uint8_t b)
{
	return op == 0 ? (uint8_t)(a & b) : op == 1 ? (uint8_t)(a ^ b) : (uint8_t)(a | b);
}

static inline void logic_op(h6280_t *c, int op, uint8_t tmp)
{
	if (P & fT) {
		clear_t(c);
		uint8_t t = rd_tfl(c);
		t = logic(op, t, tmp);
		wb_eaz(c, t);
		set_nz(c, t);
		cyc(c, 3);
	} else {
		A = logic(op, A, tmp);
		set_nz(c, A);
	}
}

#define and_a(t) logic_op(c, 0, (t))
#define eor(t)   logic_op(c, 1, (t))
#define ora(t)   logic_op(c, 2, (t))

static inline uint8_t asl(h6280_t *c, uint8_t tmp)
{
	clear_t(c);
	P = (uint8_t)((P & ~fC) | ((tmp >> 7) & fC));
	tmp = (uint8_t)(tmp << 1);
	set_nz(c, tmp);
	return tmp;
}

static inline uint8_t lsr(h6280_t *c, uint8_t tmp)
{
	clear_t(c);
	P = (uint8_t)((P & ~fC) | (tmp & fC));
	tmp = (uint8_t)(tmp >> 1);
	set_nz(c, tmp);
	return tmp;
}

static inline uint8_t rol(h6280_t *c, uint8_t tmp)
{
	clear_t(c);
	int t9 = (tmp << 1) | (P & fC);
	P = (uint8_t)((P & ~fC) | ((t9 >> 8) & fC));
	tmp = (uint8_t)t9;
	set_nz(c, tmp);
	return tmp;
}

static inline uint8_t ror(h6280_t *c, uint8_t tmp)
{
	clear_t(c);
	int t9 = tmp | (P & fC) << 8;
	P = (uint8_t)((P & ~fC) | (tmp & fC));
	tmp = (uint8_t)(t9 >> 1);
	set_nz(c, tmp);
	return tmp;
}

static inline uint8_t inc(h6280_t *c, uint8_t t) { clear_t(c); t++; set_nz(c, t); return t; }
static inline uint8_t dec(h6280_t *c, uint8_t t) { clear_t(c); t--; set_nz(c, t); return t; }

static inline void bit(h6280_t *c, uint8_t tmp)
{
	P = (uint8_t)((P & ~(fN | fV | fT | fZ)) | (tmp & 0x80 ? fN : 0) | (tmp & 0x40 ? fV : 0) | (tmp & A ? 0 : fZ));
}

static inline void tst(h6280_t *c, uint8_t imm, uint8_t tmp)
{
	P = (uint8_t)((P & ~(fN | fV | fT | fZ)) | (tmp & 0x80 ? fN : 0) | (tmp & 0x40 ? fV : 0) | (tmp & imm ? 0 : fZ));
}

static inline uint8_t trb(h6280_t *c, uint8_t tmp)
{
	clear_t(c);
	P = (uint8_t)((P & ~(fN | fV | fT | fZ)) | (tmp & 0x80 ? fN : 0) | (tmp & 0x40 ? fV : 0) | (tmp & ~A ? 0 : fZ));
	return (uint8_t)(tmp & ~A);
}

static inline uint8_t tsb(h6280_t *c, uint8_t tmp)
{
	clear_t(c);
	P = (uint8_t)((P & ~(fN | fV | fT | fZ)) | (tmp & 0x80 ? fN : 0) | (tmp & 0x40 ? fV : 0) | ((tmp | A) ? 0 : fZ));
	return (uint8_t)(tmp | A);
}

static inline void cmp_r(h6280_t *c, uint8_t r, uint8_t tmp)
{
	clear_t(c);
	P &= (uint8_t)~fC;
	if (r >= tmp)
		P |= fC;
	set_nz(c, (uint8_t)(r - tmp));
}

static inline void ld(h6280_t *c, uint8_t *r, uint8_t tmp) { clear_t(c); *r = tmp; set_nz(c, tmp); }

static inline uint8_t rmb(h6280_t *c, int b, uint8_t t) { clear_t(c); return (uint8_t)(t & ~(1 << b)); }
static inline uint8_t smb(h6280_t *c, int b, uint8_t t) { clear_t(c); return (uint8_t)(t | (1 << b)); }
static inline uint8_t st(h6280_t *c, uint8_t v) { clear_t(c); return v; }

static void brk(h6280_t *c)
{
	clear_t(c);
	PC++;
	push(c, (uint8_t)(PC >> 8));
	push(c, (uint8_t)PC);
	push(c, P);
	P = (uint8_t)((P & ~fD) | fI);
	uint8_t lo = rd8(c, IRQ2_VEC);
	uint8_t hi = rd8(c, IRQ2_VEC + 1);
	PC = (uint16_t)(lo | hi << 8);
}

static void jsr(h6280_t *c)
{
	clear_t(c);
	PC--;
	push(c, (uint8_t)(PC >> 8));
	push(c, (uint8_t)PC);
	PC = EA;
}

static void rti(h6280_t *c)
{
	P = pull(c);
	P |= fB;
	uint8_t lo = pull(c);
	uint8_t hi = pull(c);
	PC = (uint16_t)(lo | hi << 8);
	check_irq_lines(c);
}

static void rts(h6280_t *c)
{
	clear_t(c);
	uint8_t lo = pull(c);
	uint8_t hi = pull(c);
	PC = (uint16_t)((lo | hi << 8) + 1);
}

static void tam(h6280_t *c, uint8_t tmp)
{
	clear_t(c);
	for (int i = 0; i < 8; i++)
		if (tmp & (1 << i))
			c->mmr[i] = A;
}

static void tma(h6280_t *c, uint8_t tmp)
{
	clear_t(c);
	for (int i = 0; i < 8; i++)
		if (tmp & (1 << i))
			A = c->mmr[i];   // the highest bit set counts
}

static void st_io(h6280_t *c, int port, uint8_t v)
{
	clear_t(c);
	if (c->st_write)
		c->st_write(c->ctx, port, v);
}

// block transfers: kind 0 TII, 1 TDD, 2 TIN, 3 TIA, 4 TAI
static void xfer(h6280_t *c, int kind)
{
	clear_t(c);
	int from = rd16(c, PC);
	int to = rd16(c, (uint16_t)(PC + 2));
	int length = rd16(c, (uint16_t)(PC + 4));
	PC = (uint16_t)(PC + 6);
	int alternate = 0;
	if (!length)
		length = 0x10000;
	cyc(c, 6 * length + 17);
	while ((length--) != 0) {
		switch (kind) {
		case 0: wr8(c, (uint16_t)to, rd8(c, (uint16_t)from)); to++; from++; break;
		case 1: wr8(c, (uint16_t)to, rd8(c, (uint16_t)from)); to--; from--; break;
		case 2: wr8(c, (uint16_t)to, rd8(c, (uint16_t)from)); from++; break;
		case 3: wr8(c, (uint16_t)(to + alternate), rd8(c, (uint16_t)from)); from++; alternate ^= 1; break;
		default: wr8(c, (uint16_t)to, rd8(c, (uint16_t)(from + alternate))); to++; alternate ^= 1; break;
		}
	}
}

/* ---- one opcode ---- */

static void exec_op(h6280_t *c, uint8_t op)
{
	switch (op) {
	case 0x00: cyc(c, 8); brk(c); break;
	case 0x20: cyc(c, 7); ea_abs(c); jsr(c); break;
	case 0x40: cyc(c, 7); rti(c); break;
	case 0x60: cyc(c, 7); rts(c); break;
	case 0x80: bra(c, 1); break;
	case 0xa0: cyc(c, 2); ld(c, &Y, rd_imm(c)); break;
	case 0xc0: cyc(c, 2); cmp_r(c, Y, rd_imm(c)); break;
	case 0xe0: cyc(c, 2); cmp_r(c, X, rd_imm(c)); break;

	case 0x10: bra(c, !(P & fN)); break;
	case 0x30: bra(c, P & fN); break;
	case 0x50: bra(c, !(P & fV)); break;
	case 0x70: bra(c, P & fV); break;
	case 0x90: bra(c, !(P & fC)); break;
	case 0xb0: bra(c, P & fC); break;
	case 0xd0: bra(c, !(P & fZ)); break;
	case 0xf0: bra(c, P & fZ); break;

	case 0x01: cyc(c, 7); ora(rd_idx(c)); break;
	case 0x21: cyc(c, 7); and_a(rd_idx(c)); break;
	case 0x41: cyc(c, 7); eor(rd_idx(c)); break;
	case 0x61: cyc(c, 7); adc(c, rd_idx(c)); break;
	case 0x81: cyc(c, 7); wr_idx(c, st(c, A)); break;
	case 0xa1: cyc(c, 7); ld(c, &A, rd_idx(c)); break;
	case 0xc1: cyc(c, 7); cmp_r(c, A, rd_idx(c)); break;
	case 0xe1: cyc(c, 7); sbc(c, rd_idx(c)); break;

	case 0x11: cyc(c, 7); ora(rd_idy(c)); break;
	case 0x31: cyc(c, 7); and_a(rd_idy(c)); break;
	case 0x51: cyc(c, 7); eor(rd_idy(c)); break;
	case 0x71: cyc(c, 7); adc(c, rd_idy(c)); break;
	case 0x91: cyc(c, 7); wr_idy(c, st(c, A)); break;
	case 0xb1: cyc(c, 7); ld(c, &A, rd_idy(c)); break;
	case 0xd1: cyc(c, 7); cmp_r(c, A, rd_idy(c)); break;
	case 0xf1: cyc(c, 7); sbc(c, rd_idy(c)); break;

	case 0x02: { cyc(c, 3); clear_t(c); uint8_t t = X; X = Y; Y = t; break; }   // SXY
	case 0x22: { cyc(c, 3); clear_t(c); uint8_t t = X; X = A; A = t; break; }   // SAX
	case 0x42: { cyc(c, 3); clear_t(c); uint8_t t = Y; Y = A; A = t; break; }   // SAY
	case 0x62: cyc(c, 2); clear_t(c); A = 0; break;   // CLA
	case 0x82: cyc(c, 2); clear_t(c); X = 0; break;   // CLX
	case 0xa2: cyc(c, 2); ld(c, &X, rd_imm(c)); break;
	case 0xc2: cyc(c, 2); clear_t(c); Y = 0; break;   // CLY
	case 0xe2: cyc(c, 2); clear_t(c); break;

	case 0x12: cyc(c, 7); ora(rd_zpi(c)); break;
	case 0x32: cyc(c, 7); and_a(rd_zpi(c)); break;
	case 0x52: cyc(c, 7); eor(rd_zpi(c)); break;
	case 0x72: cyc(c, 7); adc(c, rd_zpi(c)); break;
	case 0x92: cyc(c, 7); wr_zpi(c, st(c, A)); break;
	case 0xb2: cyc(c, 7); ld(c, &A, rd_zpi(c)); break;
	case 0xd2: cyc(c, 7); cmp_r(c, A, rd_zpi(c)); break;
	case 0xf2: cyc(c, 7); sbc(c, rd_zpi(c)); break;

	case 0x03: cyc(c, 5); st_io(c, 0, rd_imm(c)); break;   // ST0
	case 0x23: cyc(c, 5); st_io(c, 3, rd_imm(c)); break;   // ST2
	case 0x43: cyc(c, 4); tma(c, rd_imm(c)); break;
	case 0x63: cyc(c, 4); clear_t(c); break;
	case 0x83: { cyc(c, 7); uint8_t imm = rd_imm(c); tst(c, imm, rd_zpg(c)); break; }
	case 0xa3: { cyc(c, 7); uint8_t imm = rd_imm(c); tst(c, imm, rd_zpx(c)); break; }
	case 0xc3: xfer(c, 1); break;   // TDD
	case 0xe3: xfer(c, 3); break;   // TIA

	case 0x13: cyc(c, 5); st_io(c, 2, rd_imm(c)); break;   // ST1
	case 0x33: cyc(c, 2); clear_t(c); break;
	case 0x53: cyc(c, 5); tam(c, rd_imm(c)); break;
	case 0x73: xfer(c, 0); break;   // TII
	case 0x93: { cyc(c, 8); uint8_t imm = rd_imm(c); tst(c, imm, rd_abs(c)); break; }
	case 0xb3: { cyc(c, 8); uint8_t imm = rd_imm(c); tst(c, imm, rd_abx(c)); break; }
	case 0xd3: xfer(c, 2); break;   // TIN
	case 0xf3: xfer(c, 4); break;   // TAI

	case 0x04: cyc(c, 6); wb_eaz(c, tsb(c, rd_zpg(c))); break;
	case 0x24: cyc(c, 4); bit(c, rd_zpg(c)); break;
	case 0x44: push(c, (uint8_t)(PC >> 8)); push(c, (uint8_t)PC); cyc(c, 4); bra(c, 1); break;   // BSR
	case 0x64: cyc(c, 4); wr_zpg(c, st(c, 0)); break;
	case 0x84: cyc(c, 4); wr_zpg(c, st(c, Y)); break;
	case 0xa4: cyc(c, 4); ld(c, &Y, rd_zpg(c)); break;
	case 0xc4: cyc(c, 4); cmp_r(c, Y, rd_zpg(c)); break;
	case 0xe4: cyc(c, 4); cmp_r(c, X, rd_zpg(c)); break;

	case 0x14: cyc(c, 6); wb_eaz(c, trb(c, rd_zpg(c))); break;
	case 0x34: cyc(c, 4); bit(c, rd_zpx(c)); break;
	case 0x54: cyc(c, 3); c->clocks_per_cycle = 4; break;   // CSL
	case 0x74: cyc(c, 4); wr_zpx(c, st(c, 0)); break;
	case 0x94: cyc(c, 4); wr_zpx(c, st(c, Y)); break;
	case 0xb4: cyc(c, 4); ld(c, &Y, rd_zpx(c)); break;
	case 0xd4: cyc(c, 3); c->clocks_per_cycle = 1; break;   // CSH
	case 0xf4: cyc(c, 2); P |= fT; break;                   // SET

	case 0x05: cyc(c, 4); ora(rd_zpg(c)); break;
	case 0x25: cyc(c, 4); and_a(rd_zpg(c)); break;
	case 0x45: cyc(c, 4); eor(rd_zpg(c)); break;
	case 0x65: cyc(c, 4); adc(c, rd_zpg(c)); break;
	case 0x85: cyc(c, 4); wr_zpg(c, st(c, A)); break;
	case 0xa5: cyc(c, 4); ld(c, &A, rd_zpg(c)); break;
	case 0xc5: cyc(c, 4); cmp_r(c, A, rd_zpg(c)); break;
	case 0xe5: cyc(c, 4); sbc(c, rd_zpg(c)); break;

	case 0x15: cyc(c, 4); ora(rd_zpx(c)); break;
	case 0x35: cyc(c, 4); and_a(rd_zpx(c)); break;
	case 0x55: cyc(c, 4); eor(rd_zpx(c)); break;
	case 0x75: cyc(c, 4); adc(c, rd_zpx(c)); break;
	case 0x95: cyc(c, 4); wr_zpx(c, st(c, A)); break;
	case 0xb5: cyc(c, 4); ld(c, &A, rd_zpx(c)); break;
	case 0xd5: cyc(c, 4); cmp_r(c, A, rd_zpx(c)); break;
	case 0xf5: cyc(c, 4); sbc(c, rd_zpx(c)); break;

	case 0x06: cyc(c, 6); wb_eaz(c, asl(c, rd_zpg(c))); break;
	case 0x26: cyc(c, 6); wb_eaz(c, rol(c, rd_zpg(c))); break;
	case 0x46: cyc(c, 6); wb_eaz(c, lsr(c, rd_zpg(c))); break;
	case 0x66: cyc(c, 6); wb_eaz(c, ror(c, rd_zpg(c))); break;
	case 0x86: cyc(c, 4); wr_zpg(c, st(c, X)); break;
	case 0xa6: cyc(c, 4); ld(c, &X, rd_zpg(c)); break;
	case 0xc6: cyc(c, 6); wb_eaz(c, dec(c, rd_zpg(c))); break;
	case 0xe6: cyc(c, 6); wb_eaz(c, inc(c, rd_zpg(c))); break;

	case 0x16: cyc(c, 6); wb_eaz(c, asl(c, rd_zpx(c))); break;
	case 0x36: cyc(c, 6); wb_eaz(c, rol(c, rd_zpx(c))); break;
	case 0x56: cyc(c, 6); wb_eaz(c, lsr(c, rd_zpx(c))); break;
	case 0x76: cyc(c, 6); wb_eaz(c, ror(c, rd_zpx(c))); break;
	case 0x96: cyc(c, 4); wr_zpy(c, st(c, X)); break;
	case 0xb6: cyc(c, 4); ld(c, &X, rd_zpy(c)); break;
	case 0xd6: cyc(c, 6); wb_eaz(c, dec(c, rd_zpx(c))); break;
	case 0xf6: cyc(c, 6); wb_eaz(c, inc(c, rd_zpx(c))); break;

	// RMBn / SMBn ZPG: bit = op >> 4 (RMB: bit 7 clear, SMB: bit 7 set)
	case 0x07: case 0x17: case 0x27: case 0x37: case 0x47: case 0x57: case 0x67: case 0x77:
		cyc(c, 7); wb_eaz(c, rmb(c, op >> 4, rd_zpg(c))); break;
	case 0x87: case 0x97: case 0xa7: case 0xb7: case 0xc7: case 0xd7: case 0xe7: case 0xf7:
		cyc(c, 7); wb_eaz(c, smb(c, (op >> 4) & 7, rd_zpg(c))); break;

	case 0x08: cyc(c, 3); clear_t(c); push(c, P); break;   // PHP
	case 0x28: cyc(c, 4); P = pull(c); P |= fB; check_irq_lines(c); break;   // PLP
	case 0x48: cyc(c, 3); clear_t(c); push(c, A); break;
	case 0x68: cyc(c, 4); clear_t(c); A = pull(c); set_nz(c, A); break;
	case 0x88: cyc(c, 2); Y = dec(c, Y); break;   // DEY
	case 0xa8: cyc(c, 2); clear_t(c); Y = A; set_nz(c, Y); break;   // TAY
	case 0xc8: cyc(c, 2); Y = inc(c, Y); break;   // INY
	case 0xe8: cyc(c, 2); X = inc(c, X); break;   // INX

	case 0x18: cyc(c, 2); clear_t(c); P &= (uint8_t)~fC; break;
	case 0x38: cyc(c, 2); clear_t(c); P |= fC; break;
	case 0x58: cyc(c, 2); clear_t(c); if (P & fI) { P &= (uint8_t)~fI; check_irq_lines(c); } break;   // CLI
	case 0x78: cyc(c, 2); clear_t(c); P |= fI; break;
	case 0x98: cyc(c, 2); clear_t(c); A = Y; set_nz(c, A); break;   // TYA
	case 0xb8: cyc(c, 2); clear_t(c); P &= (uint8_t)~fV; break;
	case 0xd8: cyc(c, 2); clear_t(c); P &= (uint8_t)~fD; break;
	case 0xf8: cyc(c, 2); clear_t(c); P |= fD; break;

	case 0x09: cyc(c, 2); ora(rd_imm(c)); break;
	case 0x29: cyc(c, 2); and_a(rd_imm(c)); break;
	case 0x49: cyc(c, 2); eor(rd_imm(c)); break;
	case 0x69: cyc(c, 2); adc(c, rd_imm(c)); break;
	case 0x89: cyc(c, 2); bit(c, rd_imm(c)); break;
	case 0xa9: cyc(c, 2); ld(c, &A, rd_imm(c)); break;
	case 0xc9: cyc(c, 2); cmp_r(c, A, rd_imm(c)); break;
	case 0xe9: cyc(c, 2); sbc(c, rd_imm(c)); break;

	case 0x19: cyc(c, 5); ora(rd_aby(c)); break;
	case 0x39: cyc(c, 5); and_a(rd_aby(c)); break;
	case 0x59: cyc(c, 5); eor(rd_aby(c)); break;
	case 0x79: cyc(c, 5); adc(c, rd_aby(c)); break;
	case 0x99: cyc(c, 5); wr_aby(c, st(c, A)); break;
	case 0xb9: cyc(c, 5); ld(c, &A, rd_aby(c)); break;
	case 0xd9: cyc(c, 5); cmp_r(c, A, rd_aby(c)); break;
	case 0xf9: cyc(c, 5); sbc(c, rd_aby(c)); break;

	case 0x0a: cyc(c, 2); A = asl(c, A); break;
	case 0x2a: cyc(c, 2); A = rol(c, A); break;
	case 0x4a: cyc(c, 2); A = lsr(c, A); break;
	case 0x6a: cyc(c, 2); A = ror(c, A); break;
	case 0x8a: cyc(c, 2); clear_t(c); A = X; set_nz(c, A); break;   // TXA
	case 0xaa: cyc(c, 2); clear_t(c); X = A; set_nz(c, X); break;   // TAX
	case 0xca: cyc(c, 2); X = dec(c, X); break;   // DEX
	case 0xea: cyc(c, 2); clear_t(c); break;

	case 0x1a: cyc(c, 2); A = inc(c, A); break;
	case 0x3a: cyc(c, 2); A = dec(c, A); break;
	case 0x5a: cyc(c, 3); clear_t(c); push(c, Y); break;
	case 0x7a: cyc(c, 4); clear_t(c); Y = pull(c); set_nz(c, Y); break;
	case 0x9a: cyc(c, 2); clear_t(c); S = X; break;   // TXS
	case 0xba: cyc(c, 2); clear_t(c); X = S; set_nz(c, X); break;   // TSX
	case 0xda: cyc(c, 3); clear_t(c); push(c, X); break;
	case 0xfa: cyc(c, 4); clear_t(c); X = pull(c); set_nz(c, X); break;

	// undefined: 2-cycle NOPs
	case 0x0b: case 0x2b: case 0x4b: case 0x6b: case 0x8b: case 0xab: case 0xcb: case 0xeb:
	case 0x1b: case 0x3b: case 0x5b: case 0x7b: case 0x9b: case 0xbb: case 0xdb: case 0xfb:
	case 0x5c: case 0xdc: case 0xfc:
		cyc(c, 2); clear_t(c); break;

	case 0x0c: cyc(c, 7); wb_ea(c, tsb(c, rd_abs(c))); break;
	case 0x2c: cyc(c, 5); bit(c, rd_abs(c)); break;
	case 0x4c: cyc(c, 4); ea_abs(c); clear_t(c); PC = EA; break;   // JMP ABS
	case 0x6c: cyc(c, 7); ea_ind(c); clear_t(c); PC = EA; break;   // JMP IND
	case 0x8c: cyc(c, 5); wr_abs(c, st(c, Y)); break;
	case 0xac: cyc(c, 5); ld(c, &Y, rd_abs(c)); break;
	case 0xcc: cyc(c, 5); cmp_r(c, Y, rd_abs(c)); break;
	case 0xec: cyc(c, 5); cmp_r(c, X, rd_abs(c)); break;

	case 0x1c: cyc(c, 7); wb_ea(c, trb(c, rd_abs(c))); break;
	case 0x3c: cyc(c, 5); bit(c, rd_abx(c)); break;
	case 0x7c: cyc(c, 7); ea_iax(c); clear_t(c); PC = EA; break;   // JMP IAX
	case 0x9c: cyc(c, 5); wr_abs(c, st(c, 0)); break;
	case 0xbc: cyc(c, 5); ld(c, &Y, rd_abx(c)); break;

	case 0x0d: cyc(c, 5); ora(rd_abs(c)); break;
	case 0x2d: cyc(c, 5); and_a(rd_abs(c)); break;
	case 0x4d: cyc(c, 5); eor(rd_abs(c)); break;
	case 0x6d: cyc(c, 5); adc(c, rd_abs(c)); break;
	case 0x8d: cyc(c, 5); wr_abs(c, st(c, A)); break;
	case 0xad: cyc(c, 5); ld(c, &A, rd_abs(c)); break;
	case 0xcd: cyc(c, 5); cmp_r(c, A, rd_abs(c)); break;
	case 0xed: cyc(c, 5); sbc(c, rd_abs(c)); break;

	case 0x1d: cyc(c, 5); ora(rd_abx(c)); break;
	case 0x3d: cyc(c, 5); and_a(rd_abx(c)); break;
	case 0x5d: cyc(c, 5); eor(rd_abx(c)); break;
	case 0x7d: cyc(c, 5); adc(c, rd_abx(c)); break;
	case 0x9d: cyc(c, 5); wr_abx(c, st(c, A)); break;
	case 0xbd: cyc(c, 5); ld(c, &A, rd_abx(c)); break;
	case 0xdd: cyc(c, 5); cmp_r(c, A, rd_abx(c)); break;
	case 0xfd: cyc(c, 5); sbc(c, rd_abx(c)); break;

	case 0x0e: cyc(c, 7); wb_ea(c, asl(c, rd_abs(c))); break;
	case 0x2e: cyc(c, 7); wb_ea(c, rol(c, rd_abs(c))); break;
	case 0x4e: cyc(c, 7); wb_ea(c, lsr(c, rd_abs(c))); break;
	case 0x6e: cyc(c, 7); wb_ea(c, ror(c, rd_abs(c))); break;
	case 0x8e: cyc(c, 5); wr_abs(c, st(c, X)); break;
	case 0xae: cyc(c, 5); ld(c, &X, rd_abs(c)); break;
	case 0xce: cyc(c, 7); wb_ea(c, dec(c, rd_abs(c))); break;
	case 0xee: cyc(c, 7); wb_ea(c, inc(c, rd_abs(c))); break;

	case 0x1e: cyc(c, 7); wb_ea(c, asl(c, rd_abx(c))); break;
	case 0x3e: cyc(c, 7); wb_ea(c, rol(c, rd_abx(c))); break;
	case 0x5e: cyc(c, 7); wb_ea(c, lsr(c, rd_abx(c))); break;
	case 0x7e: cyc(c, 7); wb_ea(c, ror(c, rd_abx(c))); break;
	case 0x9e: cyc(c, 5); wr_abx(c, st(c, 0)); break;
	case 0xbe: cyc(c, 5); ld(c, &X, rd_aby(c)); break;
	case 0xde: cyc(c, 7); wb_ea(c, dec(c, rd_abx(c))); break;
	case 0xfe: cyc(c, 7); wb_ea(c, inc(c, rd_abx(c))); break;

	// BBRn / BBSn ZPG,REL: bit = (op >> 4) & 7 (BBR: bit 7 clear, BBS: bit 7 set)
	case 0x0f: case 0x1f: case 0x2f: case 0x3f: case 0x4f: case 0x5f: case 0x6f: case 0x7f:
		{ cyc(c, 4); uint8_t t = rd_zpg(c); bra(c, !(t & (1 << (op >> 4)))); break; }
	case 0x8f: case 0x9f: case 0xaf: case 0xbf: case 0xcf: case 0xdf: case 0xef: case 0xff:
		{ cyc(c, 4); uint8_t t = rd_zpg(c); bra(c, (t & (1 << ((op >> 4) & 7))) != 0); break; }
	}
}

/* ---- public ---- */

int h6280_step(h6280_t *c)
{
	const int64_t start = c->cycles;

	c->ppc = PC;
	uint8_t in = prd(c, tr(c, PC));
	PC++;
	exec_op(c, in);

	if (c->irq_pending) {
		if (c->irq_pending == 1) {
			if (!(P & fI)) {
				c->irq_pending--;
				check_and_take_irq_lines(c);
			}
		} else {
			c->irq_pending--;
		}
	}

	// check internal timer
	if (c->timer_status) {
		if (c->timer_value <= 0) {
			if (!c->irq_pending)
				c->irq_pending = 1;
			while (c->timer_value <= 0)
				c->timer_value += c->timer_load;
			set_irq_line(c, 2, 1);
		}
	}
	return (int)(c->cycles - start);
}

#ifdef RBX_ARM_PCPROF
#include <stdlib.h>
static int h6280_noskip_init(void) { return getenv("RBX_NOH6SKIP") != NULL; }   // (profiling builds: to compare)
#define h6280_noskip h6280_noskip_init()
#else
#define h6280_noskip 0
#endif
void h6280_run_until(h6280_t *c, int64_t target)
{
	if (c->cycles >= target)
		return;
	if (c->irq_pending == 2)
		c->irq_pending--;
	do {
		// BRA to itself, or BBRi / BBSi zp to itself (battlera's BBS7 $01: a flag the interrupt clears) taken: a wait for
		// an interrupt. Nothing but the time changes until the timer fires or the run's end (the other interrupt lines
		// change between runs; the zero page is this CPU's RAM), so the whole iterations up to there pass at once.
		// (Looked at when an instruction left the pc where it was, no interrupt pending before or after it)
		const uint16_t at = PC;
		const int pend = c->irq_pending;
		const int64_t before = c->cycles;
		h6280_step(c);
		if (PC == at && !pend && !c->irq_pending && !h6280_noskip) {
			const uint32_t pa = tr(c, at);
			const uint8_t *page = c->rpage[pa >> 13];
			const uint32_t o = pa & 0x1fff;
			const int64_t per = c->cycles - before;
			if (page && o < 0x1ffe &&
			    ((page[o] == 0x80 && page[o + 1] == 0xfe) || ((page[o] & 0x0f) == 0x0f && page[o + 2] == 0xfd)) &&
			    per > 0 && c->cycles < target) {
				const int64_t span = target - c->cycles - 1;
				int64_t k = span < 0x7fffffff && per < 0x7fffffff ? (int32_t)span / (int32_t)per : span / per;
				if (c->timer_status) {
					const int64_t kt = c->timer_value > 0 && per < 0x7fffffff ? (c->timer_value - 1) / (int32_t)per : 0;
					if (kt < k)
						k = kt;
				}
				c->cycles += k * per;
				c->timer_value -= (int32_t)(k * per);
			}
		}
		if (h6280_break)
			break;
	} while (c->cycles < target);
}

void h6280_set_irq(h6280_t *c, int line, int state)
{
	if (line >= 0 && line < 3)
		set_irq_line(c, line, state);
}

void h6280_set_nmi(h6280_t *c, int state)
{
	if (!state)
		return;
	c->nmi_state = 1;
	check_irq_lines(c);
}

void h6280_reset(h6280_t *c)
{
	c->ppc = 0;
	c->pc = 0;
	c->ea = 0;
	c->a = c->x = c->y = 0;
	memset(c->mmr, 0, sizeof(c->mmr));
	c->irq_mask = 0;
	c->timer_ack = 0;
	c->timer_value = 0;
	c->io_buffer = 0;
	c->rpage[0xff] = NULL;   // the I/O page always goes through the internal decode
	c->wpage[0xff] = NULL;

	P = fI | fB;   // set I and B flags
	S = 0xff;      // stack starts at 0x01ff

	uint8_t lo = rd8(c, RESET_VEC);
	uint8_t hi = rd8(c, RESET_VEC + 1);
	PC = (uint16_t)(lo | hi << 8);

	c->clocks_per_cycle = 4;   // low speed mode
	c->timer_status = 0;       // timer off
	c->timer_load = 128 * 1024 * c->timer_scale;
	c->irq_pending = 0;
}

void h6280_init(h6280_t *c)
{
	memset(c, 0, sizeof(*c));
	c->timer_scale = 1;
	c->clocks_per_cycle = 4;
}
