/*  Mitsubishi M740 CPU and the M37450 microcontroller, ported to C from MAME 0.289 src/devices/cpu/m6502
    (m740.cpp, m6502.cpp, om740.lst, om6502.lst, dm740.lst; BSD-3-Clause, copyright-holders: Olivier Galibert;
    m3745x.cpp; BSD-3-Clause, copyright-holders: R. Belmont, Olivier Galibert). See m740.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../cheat_cpu.h"
#include "m740.h"

#include <string.h>

enum { F_N = 0x80, F_V = 0x40, F_T = 0x20, F_B = 0x10, F_D = 0x08, F_I = 0x04, F_Z = 0x02, F_C = 0x01 };
#define STATE_RESET 0xff00

// MAME m740_device line numbers (M740_INT0_LINE = 0 .. M740_INT14_LINE = 14)
enum { M740_INT2_LINE = 2, M740_INT3_LINE = 3, M740_INT4_LINE = 4, M740_INT11_LINE = 11, M740_INT12_LINE = 12,
       M740_INT13_LINE = 13, M740_MAX_INT_LINE = 14 };

// m3745x.cpp interrupt / A/D bits
#define IRQ1_INT1 0x04
#define IRQ1_INT2 0x08
#define IRQ1_INT3 0x10
#define IRQ2_ADC 0x20
#define ADCTRL_COMPLETE 0x08
#define ADC_CYCLES 50   // (MAME: cycles_to_attotime(50))

// ---- M740 interrupt lines (m740.cpp set_irq_line) ----
static void set_irq_line(m740_t *c, int line, int state)
{
	if (state)
		c->irq_multiplex |= 1u << line;
	else
		c->irq_multiplex &= ~(1u << line);
	c->irq_state = c->irq_multiplex != 0;
	if (c->irq_state) {
		for (int i = 0; i < M740_MAX_INT_LINE; i++)
			if (c->irq_multiplex & (1u << i)) {
				c->irq_vector = (uint16_t)(0xfffc - 2 * i);
				break;
			}
	}
}

// ---- M3745x (m3745x.cpp) ----
static void recalc_irqs(m740_t *c)
{
	static const int8_t irq_lines[16] = {
		-1, -1, -1, M740_INT11_LINE, M740_INT12_LINE, M740_INT13_LINE, -1, -1,
		-1, -1, M740_INT2_LINE, M740_INT3_LINE, M740_INT4_LINE, -1, -1, -1
	};
	const uint16_t all_ints = (uint16_t)(((c->intreq1 & c->intctrl1) << 8) | (c->intreq2 & c->intctrl2));
	const uint16_t changed = all_ints ^ c->last_all_ints;
	for (int i = 0; i < 16; i++)
		if ((changed & (1u << i)) && irq_lines[i] != -1)
			set_irq_line(c, irq_lines[i], (all_ints >> i) & 1);
	c->last_all_ints = all_ints;
}

static void adc_complete(m740_t *c)
{
	c->adc_busy = 0;
	c->adctrl |= ADCTRL_COMPLETE;
	c->intreq2 |= IRQ2_ADC;
	recalc_irqs(c);
}

static void send_port(m740_t *c, int i, uint8_t v)
{
	if (c->port_write)
		c->port_write(c->ctx, i + 3, v);
}

static uint8_t read_port(m740_t *c, int i)
{
	uint8_t v = c->port_read ? c->port_read(c->ctx, i + 3) : 0;
	v &= (uint8_t)(c->ddrs[i] ^ 0xff);
	v |= c->ports[i] & c->ddrs[i];
	return v;
}

static uint8_t sfr_read(m740_t *c, uint16_t a)
{
	switch (a) {
	case 0xd6: return read_port(c, 0);
	case 0xd7: return c->ddrs[0];
	case 0xd8: return read_port(c, 1);
	case 0xda: return read_port(c, 2);
	case 0xdb: return c->ddrs[2];
	case 0xdc: return read_port(c, 3);
	case 0xdd: return c->ddrs[3];
	case 0xd9: return 0xff;   // (ports_r: the P4 direction register is not readable)
	case 0xe2:
		c->intreq2 &= (uint8_t)~IRQ2_ADC;
		recalc_irqs(c);
		return c->ad_read ? c->ad_read(c->ctx, c->adctrl & 7) : 0;
	case 0xe3: return c->adctrl;
	case 0xfc: return c->intreq1;
	case 0xfd: return c->intreq2;
	case 0xfe: return c->intctrl1;
	case 0xff: return c->intctrl2;
	}
	return c->read ? c->read(c->ctx, a) : 0;   // (not in the internal map)
}

static void sfr_write(m740_t *c, uint16_t a, uint8_t v)
{
	switch (a) {
	case 0xd6: send_port(c, 0, v & c->ddrs[0]); c->ports[0] = v; return;
	case 0xd7: send_port(c, 0, c->ports[0] & v); c->ddrs[0] = v; return;
	case 0xd8: send_port(c, 1, v & c->ddrs[1]); c->ports[1] = v; return;
	case 0xda: send_port(c, 2, v & c->ddrs[2]); c->ports[2] = v; return;
	case 0xdb: send_port(c, 2, c->ports[2] & v); c->ddrs[2] = v; return;
	case 0xdc: send_port(c, 3, v & c->ddrs[3]); c->ports[3] = v; return;
	case 0xdd: send_port(c, 3, c->ports[3] & v); c->ddrs[3] = v; return;
	case 0xd9: return;   // (ports_w: no P4 direction register)
	case 0xe2: return;   // (adc_w: the result register is read-only)
	case 0xe3:
		c->adctrl = v;
		if (!(v & ADCTRL_COMPLETE)) {   // (a conversion starts: 50 cycles)
			c->adc_busy = 1;
			c->adc_due = c->cycles + ADC_CYCLES;
		}
		return;
	case 0xfc: c->intreq1 = v; recalc_irqs(c); return;
	case 0xfd: c->intreq2 = v; recalc_irqs(c); return;
	case 0xfe: c->intctrl1 = v; recalc_irqs(c); return;
	case 0xff: c->intctrl2 = v; recalc_irqs(c); return;
	}
	if (c->write)
		c->write(c->ctx, a, v);
}

// ---- bus: one cycle per line with an access (CY, counted by the generated code as MAME m6502make.py) ----
static inline uint8_t RD(m740_t *c, uint16_t a)
{
	if (a < 0x200) {
		if (a >= 0xc0 && a < 0x100)
			return sfr_read(c, a);
		return c->ram[a];
	}
	const uint8_t *p = c->rpage[a >> 8];
	if (p)
		return p[a & 0xff];
	return c->read ? c->read(c->ctx, a) : 0;
}
static inline void WR(m740_t *c, uint16_t a, uint8_t v)
{
	if (a < 0x200) {
		if (a >= 0xc0 && a < 0x100)
			sfr_write(c, a, v);
		else
			c->ram[a] = v;
		return;
	}
	uint8_t *p = c->wpage[a >> 8];
	if (p)
		p[a & 0xff] = v;
	else if (c->write)
		c->write(c->ctx, a, v);
}
#define CY(c) ((c)->cycles++)

// MAME prefetch: the next opcode (one cycle), then an interrupt taken in its place (BRK's sequence) or the PC past it.
// An A/D conversion ending by then is seen here (MAME stops the CPU at the access where its timer expires and runs
// the rest of the instruction after the callback).
static inline void PREFETCH(m740_t *c)
{
	c->ir = RD(c, c->pc);
	CY(c);
	if (c->adc_busy && c->cycles >= c->adc_due)
		adc_complete(c);
	if (c->irq_state && !(c->p & F_I)) {
		c->irq_taken = 1;
		c->ir = 0x00;
	} else
		c->pc++;
}

static inline int page_changing(uint16_t base, int delta) { return ((base + delta) ^ base) & 0xff00; }
static inline uint16_t set_l(uint16_t base, uint8_t val) { return (uint16_t)((base & 0xff00) | val); }
static inline uint16_t set_h(uint16_t base, uint8_t val) { return (uint16_t)((base & 0x00ff) | (val << 8)); }

// ---- m6502.cpp helpers ----
static void set_nz(m740_t *c, uint8_t v)
{
	c->p &= (uint8_t)~(F_Z | F_N);
	if (v & 0x80)
		c->p |= F_N;
	if (!v)
		c->p |= F_Z;
}
static void do_adc_d(m740_t *c, uint8_t val)
{
	const uint8_t cy = c->p & F_C ? 1 : 0;
	c->p &= (uint8_t)~(F_N | F_V | F_Z | F_C);
	uint8_t al = (uint8_t)((c->a & 15) + (val & 15) + cy);
	if (al > 9)
		al += 6;
	uint8_t ah = (uint8_t)((c->a >> 4) + (val >> 4) + (al > 15));
	if (!(uint8_t)(c->a + val + cy))
		c->p |= F_Z;
	else if (ah & 8)
		c->p |= F_N;
	if (~(c->a ^ val) & (c->a ^ (ah << 4)) & 0x80)
		c->p |= F_V;
	if (ah > 9)
		ah += 6;
	if (ah > 15)
		c->p |= F_C;
	c->a = (uint8_t)((ah << 4) | (al & 15));
}
static void do_adc_nd(m740_t *c, uint8_t val)
{
	const uint16_t sum = (uint16_t)(c->a + val + (c->p & F_C ? 1 : 0));
	c->p &= (uint8_t)~(F_N | F_V | F_Z | F_C);
	if (!(uint8_t)sum)
		c->p |= F_Z;
	else if ((int8_t)sum < 0)
		c->p |= F_N;
	if (~(c->a ^ val) & (c->a ^ sum) & 0x80)
		c->p |= F_V;
	if (sum & 0xff00)
		c->p |= F_C;
	c->a = (uint8_t)sum;
}
static void do_adc(m740_t *c, uint8_t val)
{
	if (c->p & F_D)
		do_adc_d(c, val);
	else
		do_adc_nd(c, val);
}
static void do_cmp(m740_t *c, uint8_t val1, uint8_t val2)
{
	c->p &= (uint8_t)~(F_N | F_Z | F_C);
	const uint16_t r = (uint16_t)(val1 - val2);
	if (!r)
		c->p |= F_Z;
	else if ((int8_t)r < 0)
		c->p |= F_N;
	if (!(r & 0xff00))
		c->p |= F_C;
}
static void do_sbc_d(m740_t *c, uint8_t val)
{
	const uint8_t cy = c->p & F_C ? 0 : 1;
	c->p &= (uint8_t)~(F_N | F_V | F_Z | F_C);
	const uint16_t diff = (uint16_t)(c->a - val - cy);
	uint8_t al = (uint8_t)((c->a & 15) - (val & 15) - cy);
	uint8_t ah = (uint8_t)((c->a >> 4) - (val >> 4) - ((int8_t)al < 0));
	if (!(uint8_t)diff)
		c->p |= F_Z;
	else if (diff & 0x80)
		c->p |= F_N;
	if ((c->a ^ val) & (c->a ^ diff) & 0x80)
		c->p |= F_V;
	if (!(diff & 0xff00))
		c->p |= F_C;
	if ((int8_t)al < 0)
		al -= 6;
	if ((int8_t)ah < 0)
		ah -= 6;
	c->a = (uint8_t)((ah << 4) | (al & 15));
}
static void do_sbc_nd(m740_t *c, uint8_t val)
{
	const uint16_t diff = (uint16_t)(c->a - val - (c->p & F_C ? 0 : 1));
	c->p &= (uint8_t)~(F_N | F_V | F_Z | F_C);
	if (!(uint8_t)diff)
		c->p |= F_Z;
	else if ((int8_t)diff < 0)
		c->p |= F_N;
	if ((c->a ^ val) & (c->a ^ diff) & 0x80)
		c->p |= F_V;
	if (!(diff & 0xff00))
		c->p |= F_C;
	c->a = (uint8_t)diff;
}
static void do_sbc(m740_t *c, uint8_t val)
{
	if (c->p & F_D)
		do_sbc_d(c, val);
	else
		do_sbc_nd(c, val);
}
static void do_bit(m740_t *c, uint8_t val)
{
	c->p &= (uint8_t)~(F_N | F_Z | F_V);
	const uint8_t r = c->a & val;
	if (!r)
		c->p |= F_Z;
	if (val & 0x80)
		c->p |= F_N;
	if (val & 0x40)
		c->p |= F_V;
}
static uint8_t do_asl(m740_t *c, uint8_t v)
{
	c->p &= (uint8_t)~(F_N | F_Z | F_C);
	const uint8_t r = (uint8_t)(v << 1);
	if (!r)
		c->p |= F_Z;
	else if ((int8_t)r < 0)
		c->p |= F_N;
	if (v & 0x80)
		c->p |= F_C;
	return r;
}
static uint8_t do_lsr(m740_t *c, uint8_t v)
{
	c->p &= (uint8_t)~(F_N | F_Z | F_C);
	if (v & 1)
		c->p |= F_C;
	v >>= 1;
	if (!v)
		c->p |= F_Z;
	return v;
}
static uint8_t do_ror(m740_t *c, uint8_t v)
{
	const int cy = c->p & F_C;
	c->p &= (uint8_t)~(F_N | F_Z | F_C);
	if (v & 1)
		c->p |= F_C;
	v >>= 1;
	if (cy)
		v |= 0x80;
	if (!v)
		c->p |= F_Z;
	else if ((int8_t)v < 0)
		c->p |= F_N;
	return v;
}
static uint8_t do_rol(m740_t *c, uint8_t v)
{
	const int cy = c->p & F_C;
	c->p &= (uint8_t)~(F_N | F_Z | F_C);
	if (v & 0x80)
		c->p |= F_C;
	v = (uint8_t)(v << 1);
	if (cy)
		v |= 0x01;
	if (!v)
		c->p |= F_Z;
	else if ((int8_t)v < 0)
		c->p |= F_N;
	return v;
}

// ---- m740.cpp helpers (the T-mode ops work on c->tmp, the byte at $00,X) ----
static uint8_t do_clb(m740_t *c, uint8_t in, uint8_t bit) { (void)c; return (uint8_t)(in & ~(1 << bit)); }
static uint8_t do_seb(m740_t *c, uint8_t in, uint8_t bit) { (void)c; return (uint8_t)(in | (1 << bit)); }
static uint8_t do_rrf(m740_t *c, uint8_t in) { (void)c; return (uint8_t)(((in & 0xf) << 4) | ((in & 0xf0) >> 4)); }

static void do_sbc_dt(m740_t *c, uint8_t val)
{
	const uint8_t cy = c->p & F_C ? 0 : 1;
	c->p &= (uint8_t)~(F_N | F_V | F_Z | F_C);
	const uint16_t diff = (uint16_t)(c->tmp - val - cy);
	uint8_t al = (uint8_t)((c->tmp & 15) - (val & 15) - cy);
	if ((int8_t)al < 0)
		al -= 6;
	uint8_t ah = (uint8_t)((c->tmp >> 4) - (val >> 4) - ((int8_t)al < 0));
	if (!(uint8_t)diff)
		c->p |= F_Z;
	else if (diff & 0x80)
		c->p |= F_N;
	if ((c->tmp ^ val) & (c->tmp ^ diff) & 0x80)
		c->p |= F_V;
	if (!(diff & 0xff00))
		c->p |= F_C;
	if ((int8_t)ah < 0)
		ah -= 6;
	c->tmp = (uint16_t)((ah << 4) | (al & 15));
}
static void do_sbc_ndt(m740_t *c, uint8_t val)
{
	const uint16_t diff = (uint16_t)(c->tmp - val - (c->p & F_C ? 0 : 1));
	c->p &= (uint8_t)~(F_N | F_V | F_Z | F_C);
	if (!(uint8_t)diff)
		c->p |= F_Z;
	else if ((int8_t)diff < 0)
		c->p |= F_N;
	if ((c->tmp ^ val) & (c->tmp ^ diff) & 0x80)
		c->p |= F_V;
	if (!(diff & 0xff00))
		c->p |= F_C;
	c->tmp = diff;
}
static void do_sbct(m740_t *c, uint8_t val)
{
	if (c->p & F_D)
		do_sbc_dt(c, val);
	else
		do_sbc_ndt(c, val);
}
static void do_adc_dt(m740_t *c, uint8_t val)
{
	const uint8_t cy = c->p & F_C ? 1 : 0;
	c->p &= (uint8_t)~(F_N | F_V | F_Z | F_C);
	uint8_t al = (uint8_t)((c->tmp & 15) + (val & 15) + cy);
	if (al > 9)
		al += 6;
	uint8_t ah = (uint8_t)((c->tmp >> 4) + (val >> 4) + (al > 15));
	if (!(uint8_t)(c->tmp + val + cy))
		c->p |= F_Z;
	else if (ah & 8)
		c->p |= F_N;
	if (~(c->tmp ^ val) & (c->tmp ^ (ah << 4)) & 0x80)
		c->p |= F_V;
	if (ah > 9)
		ah += 6;
	if (ah > 15)
		c->p |= F_C;
	c->tmp = (uint16_t)((ah << 4) | (al & 15));
}
static void do_adc_ndt(m740_t *c, uint8_t val)
{
	const uint16_t sum = (uint16_t)(c->tmp + val + (c->p & F_C ? 1 : 0));
	c->p &= (uint8_t)~(F_N | F_V | F_Z | F_C);
	if (!(uint8_t)sum)
		c->p |= F_Z;
	else if ((int8_t)sum < 0)
		c->p |= F_N;
	if (~(c->tmp ^ val) & (c->tmp ^ sum) & 0x80)
		c->p |= F_V;
	if (sum & 0xff00)
		c->p |= F_C;
	c->tmp = sum;
}
static void do_adct(m740_t *c, uint8_t val)
{
	if (c->p & F_D)
		do_adc_dt(c, val);
	else
		do_adc_ndt(c, val);
}

#include "m740_ops.inc"

// ---- API ----
// (the cheats' access to this CPU's program space: cheat_cpu.h)
static uint8_t cheat_rd(void *x, uint32_t a) { m740_t *c = (m740_t *)x; return c->read ? c->read(c->ctx, (uint16_t)a) : 0; }
static void cheat_wr(void *x, uint32_t a, uint8_t v) { m740_t *c = (m740_t *)x; if (c->write) c->write(c->ctx, (uint16_t)a, v); }

void m740_init(m740_t *c)
{
	rbx_cheat_offer("m740", cheat_rd, cheat_wr, c, 0, 1);
	memset(c, 0, sizeof(*c));
	// (m6502_device::init)
	c->x = 0x80;
	c->p = 0x36;
	c->sp = 0x0100;
	c->inst_state = STATE_RESET;
	m740_reset(c);
}

void m740_reset(m740_t *c)
{
	// m6502_device::device_reset
	c->inst_state = STATE_RESET;
	c->inst_state_base = 0;
	c->irq_taken = 0;
	c->halted = 0;
	c->waiting = 0;
	// m740_device::device_reset
	c->irq_state = 0;
	c->irq_multiplex = 0;
	c->irq_vector = 0xfffc;
	// m3745x_device::device_reset: the stack in page 1, ports input, timers stopped
	c->sp = 0x01ff;
	memset(c->ddrs, 0, sizeof(c->ddrs));
	memset(c->ports, 0, sizeof(c->ports));
	c->intreq1 = c->intreq2 = c->intctrl1 = c->intctrl2 = 0;
	c->adctrl = 0;
	c->last_all_ints = 0;
	c->adc_busy = 0;
}

void m740_set_reset(m740_t *c, int state)
{
	if (state && !c->in_reset)
		m740_reset(c);
	c->in_reset = state != 0;
}

void m740_set_input(m740_t *c, int line, int state)
{
	static const uint8_t bits[3] = { IRQ1_INT1, IRQ1_INT2, IRQ1_INT3 };
	if (line < 0 || line > 2)
		return;
	if (state)
		c->intreq1 |= bits[line];
	else
		c->intreq1 &= (uint8_t)~bits[line];
	recalc_irqs(c);
}

int m740_step(m740_t *c)
{
	const int64_t t0 = c->cycles;
	if (c->in_reset || c->halted) {
		c->cycles++;
		return 1;
	}
	if (c->waiting) {   // (STP / WIT: until an interrupt request, then the fetch where it is taken)
		if (!c->irq_state) {
			c->cycles++;
			return 1;
		}
		c->waiting = 0;
		PREFETCH(c);
	} else if (c->inst_state == STATE_RESET)
		OP_RESET(c);
	else {
		c->inst_state = c->ir | c->inst_state_base;
		s_ops[c->inst_state](c);
	}
	if (c->inst_state != STATE_RESET)
		c->inst_state = -1;
	if (c->adc_busy && c->cycles >= c->adc_due)
		adc_complete(c);
	return (int)(c->cycles - t0);
}

int m740_execute(m740_t *c, int cycles)
{
	const int64_t t0 = c->cycles;
	c->icount += cycles;
	while (c->icount > 0) {
		if (c->in_reset || c->halted || (c->waiting && !c->irq_state)) {
			// idle: to the end of the slice, or to the end of an A/D conversion (it may wake WIT)
			int64_t n = c->icount;
			if (c->adc_busy && !c->in_reset && c->adc_due - c->cycles < n)
				n = c->adc_due > c->cycles ? c->adc_due - c->cycles : 0;
			c->cycles += n;
			c->icount -= (int32_t)n;
			if (c->adc_busy && c->cycles >= c->adc_due)
				adc_complete(c);
			continue;
		}
		c->icount -= m740_step(c);
	}
	return (int)(c->cycles - t0);
}
