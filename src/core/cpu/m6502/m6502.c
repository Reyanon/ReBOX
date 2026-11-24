/*  MOS 6502 (NMOS) and Rockwell R65C02, ported to C from MAME src/devices/cpu/m6502 (m6502.cpp, w65c02.cpp,
    om6502.lst, ow65c02.lst, dm6502.lst, dr65c02.lst; BSD-3-Clause, copyright-holders: Olivier Galibert). See m6502.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../cheat_cpu.h"
#include "m6502.h"

#include <string.h>

enum { F_N = 0x80, F_V = 0x40, F_E = 0x20, F_B = 0x10, F_D = 0x08, F_I = 0x04, F_Z = 0x02, F_C = 0x01 };
#define STATE_RESET 0xff00

static inline uint8_t RD(m6502_t *c, uint16_t a)
{
	c->cycles++;
	const uint8_t *p = c->rpage[a >> 8];
	return p ? p[a & 0xff] : c->read(c->ctx, a);
}
#define RDA RD   // (MAME read_arg: the same space here)
// MAME read_sync (the opcode fetch): DECO 222 / C10707 swap its bits 5 and 6
static inline uint8_t RDSYNC(m6502_t *c, uint16_t a)
{
	if (c->op_rom && a >= c->op_start) {   // (the opcodes space)
		c->cycles++;
		return c->op_rom[a - c->op_start];
	}
	const uint8_t v = RD(c, a);
	if (c->cpu7 && c->cpu7_written) {   // (bitswap<8>(v, 6, 5, 3, 4, 2, 7, 1, 0))
		c->cpu7_written = 0;
		if ((a & 0x0104) == 0x0104)
			return (uint8_t)((v & 0x13) | ((v >> 6) & 1) << 7 | ((v >> 5) & 1) << 6 | ((v >> 3) & 1) << 5 |
			                 ((v >> 2) & 1) << 3 | ((v >> 7) & 1) << 2);
	}
	return c->deco222 ? (uint8_t)((v & 0x9f) | ((v & 0x20) << 1) | ((v & 0x40) >> 1)) : v;
}
static inline void WR(m6502_t *c, uint16_t a, uint8_t v)
{
	c->cycles++;
	c->cpu7_written = 1;
	uint8_t *p = c->wpage[a >> 8];
	if (p)
		p[a & 0xff] = v;
	else
		c->write(c->ctx, a, v);
}
// the DECO CPU-16's I/O space (no bus clock: MAME's m_io accesses)
static inline uint8_t IOR(m6502_t *c, uint8_t a) { return c->io_read ? c->io_read(c->ctx, a) : 0xff; }
static inline void IOW(m6502_t *c, uint8_t a, uint8_t v)
{
	if (c->io_write)
		c->io_write(c->ctx, a, v);
}
// MAME prefetch: the next opcode, then an interrupt taken in its place (BRK's sequence) or the PC past it
static inline void PREFETCH(m6502_t *c)
{
	c->ir = RDSYNC(c, c->pc);
	if (c->nmi_pending || (c->irq_state && !(c->p & F_I))) {
		c->irq_taken = 1;
		if (c->irq_hold && !c->nmi_pending)
			c->irq_state = 0;
		c->ir = 0x00;
	} else
		c->pc++;
}
static inline void PREFETCH_NOIRQ(m6502_t *c)
{
	c->ir = RDSYNC(c, c->pc);
	c->pc++;
}

static inline int page_changing(uint16_t base, int delta) { return ((base + delta) ^ base) & 0xff00; }
static inline uint16_t set_l(uint16_t base, uint8_t val) { return (uint16_t)((base & 0xff00) | val); }
static inline uint16_t set_h(uint16_t base, uint8_t val) { return (uint16_t)((base & 0x00ff) | (val << 8)); }

static void set_nz(m6502_t *c, uint8_t v)
{
	c->p &= (uint8_t)~(F_Z | F_N);
	if (v & 0x80)
		c->p |= F_N;
	if (!v)
		c->p |= F_Z;
}
static void do_adc_d(m6502_t *c, uint8_t val)
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
static void do_adc_nd(m6502_t *c, uint8_t val)
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
static void do_adc(m6502_t *c, uint8_t val)
{
	if (c->p & F_D)
		do_adc_d(c, val);
	else
		do_adc_nd(c, val);
}
static void do_arr_nd(m6502_t *c)
{
	const int cy = c->p & F_C;
	c->p &= (uint8_t)~(F_N | F_Z | F_C | F_V);
	c->a >>= 1;
	if (cy)
		c->a |= 0x80;
	if (!c->a)
		c->p |= F_Z;
	else if ((int8_t)c->a < 0)
		c->p |= F_N;
	if (c->a & 0x40)
		c->p |= F_V | F_C;
	if (c->a & 0x20)
		c->p ^= F_V;
}
static void do_arr_d(m6502_t *c)
{
	const int cy = c->p & F_C;
	c->p &= (uint8_t)~(F_N | F_Z | F_C | F_V);
	uint8_t a = c->a >> 1;
	if (cy)
		a |= 0x80;
	if (!a)
		c->p |= F_Z;
	else if ((int8_t)a < 0)
		c->p |= F_N;
	if ((a ^ c->a) & 0x40)
		c->p |= F_V;
	if ((c->a & 0x0f) >= 0x05)
		a = (uint8_t)(((a + 6) & 0x0f) | (a & 0xf0));
	if ((c->a & 0xf0) >= 0x50) {
		a += 0x60;
		c->p |= F_C;
	}
	c->a = a;
}
static void do_arr(m6502_t *c)
{
	if (c->p & F_D)
		do_arr_d(c);
	else
		do_arr_nd(c);
}
static void do_cmp(m6502_t *c, uint8_t val1, uint8_t val2)
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
static void do_sbc_d(m6502_t *c, uint8_t val)
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
static void do_sbc_nd(m6502_t *c, uint8_t val)
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
static void do_sbc(m6502_t *c, uint8_t val)
{
	if (c->p & F_D)
		do_sbc_d(c, val);
	else
		do_sbc_nd(c, val);
}
// w65c02 SBC: the decimal adjust carries between the digits; N and Z set by the opcode after it
static void do_sbc_cd(m6502_t *c, uint8_t val)
{
	const uint8_t cy = c->p & F_C ? 0 : 1;
	c->p &= (uint8_t)~(F_N | F_V | F_Z | F_C);
	const uint16_t diff = (uint16_t)(c->a - val - cy);
	const uint8_t al = (uint8_t)((c->a & 15) - (val & 15) - cy);
	const uint8_t ah = (uint8_t)((c->a >> 4) - (val >> 4) - ((int8_t)al < 0));
	if ((c->a ^ val) & (c->a ^ diff) & 0x80)
		c->p |= F_V;
	if (!(diff & 0xff00))
		c->p |= F_C;
	c->a = (uint8_t)((ah << 4) | (al & 15));
	if ((int8_t)al < 0)
		c->a -= 6;
	if ((int8_t)ah < 0)
		c->a -= 0x60;
}
static void do_sbc_c(m6502_t *c, uint8_t val)
{
	if (c->p & F_D)
		do_sbc_cd(c, val);
	else
		do_sbc_nd(c, val);
}
static void do_bit(m6502_t *c, uint8_t val)
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
static uint8_t do_asl(m6502_t *c, uint8_t v)
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
static uint8_t do_lsr(m6502_t *c, uint8_t v)
{
	c->p &= (uint8_t)~(F_N | F_Z | F_C);
	if (v & 1)
		c->p |= F_C;
	v >>= 1;
	if (!v)
		c->p |= F_Z;
	return v;
}
static uint8_t do_ror(m6502_t *c, uint8_t v)
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
static uint8_t do_rol(m6502_t *c, uint8_t v)
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
#include "m6502_ops.inc"

// (the cheats' access to this CPU's program space: cheat_cpu.h)
static uint8_t cheat_rd(void *x, uint32_t a)   // (as the CPU: its direct pages, else the callbacks)
{
	m6502_t *c = (m6502_t *)x;
	const uint8_t *p = c->rpage[(a >> 8) & 0xff];
	return p ? p[a & 0xff] : c->read ? c->read(c->ctx, (uint16_t)a) : 0;
}
static void cheat_wr(void *x, uint32_t a, uint8_t v)
{
	m6502_t *c = (m6502_t *)x;
	uint8_t *p = c->wpage[(a >> 8) & 0xff];
	if (p)
		p[a & 0xff] = v;
	else if (c->write)
		c->write(c->ctx, (uint16_t)a, v);
}

void m6502_init(m6502_t *c, int type)
{
	rbx_cheat_offer("m6502", cheat_rd, cheat_wr, c, 0, 1);
	memset(c, 0, sizeof(*c));
	c->type = type;
	c->x = 0x80;
	c->p = 0x36;
	c->sp = 0x0100;
	c->inst_state = STATE_RESET;
}

void m6502_reset(m6502_t *c)
{
	c->inst_state = STATE_RESET;
	c->nmi_pending = 0;
	c->irq_taken = 0;
	c->halted = 0;
	c->waiting = 0;
}

int m6502_step(m6502_t *c)
{
	const int64_t t0 = c->cycles;
	if (c->halted) {
		c->cycles++;
		return 1;
	}
	if (c->waiting) {   // (WAI: until an interrupt; then the opcode fetch where it is taken)
		if (!c->nmi_pending && !c->irq_state) {
			c->cycles++;
			return 1;
		}
		c->waiting = 0;
		PREFETCH(c);
	} else if (c->inst_state == STATE_RESET)
		s_resets[c->type](c);
	else {
		c->inst_state = c->ir;   // (MAME m_inst_state: the opcode; the bit instructions take their bit from it)
		s_tables[c->type][c->ir](c);
	}
	if (c->inst_state != STATE_RESET)
		c->inst_state = -1;
	return (int)(c->cycles - t0);
}

void m6502_run_until(m6502_t *c, int64_t target)
{
	while (c->cycles < target) {
		if (c->halted || (c->waiting && !c->nmi_pending && !c->irq_state)) {
			c->cycles = target;
			return;
		}
		m6502_step(c);
	}
}

void m6502_set_irq(m6502_t *c, int state) { c->irq_state = state != 0; }

void m6502_set_nmi(m6502_t *c, int state)
{
	// (MAME: no NMI edge at the moment the reset clears)
	if (!c->nmi_state && state && c->cycles)
		c->nmi_pending = 1;
	c->nmi_state = state != 0;
}
