/*  Intel MCS-48 / UPI-41 CPU core, ported to C99 from MAME src/devices/cpu/mcs48/mcs48.cpp
    (license: BSD-3-Clause, copyright-holders: Dan Boris, Mirko Buffoni, Aaron Giles, Couriersud).
    The opcode handlers, timer/counter, interrupt logic (including MAME's JNI/JNIBF polling hack)
    and the UPI-41 host interface follow MAME; the handler tables became one switch.
    Not emulated: the 8021/8022 variants, T0 clock output (ENT0 CLK is a no-op), EA switching at
    run time (the program comes from 'rom' in all cases), DMA handshake beyond the P2 mask.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "mcs48.h"

#include <string.h>

// timer/counter enable bits
#define TIMER_ENABLED   0x01
#define COUNTER_ENABLED 0x02

// flag bits
#define C_FLAG          0x80
#define A_FLAG          0x40
#define F_FLAG          0x20
#define B_FLAG          0x10

// status bits (UPI-41)
#define STS_IBF         0x02
#define STS_OBF         0x01

// port 2 bits (UPI-41)
#define P2_OBF          0x10
#define P2_NIBF         0x20
#define P2_DRQ          0x40
#define P2_NDACK        0x80

enum { EXPANDER_OP_READ = 0, EXPANDER_OP_WRITE = 1, EXPANDER_OP_OR = 2, EXPANDER_OP_AND = 3 };

// ---- bus helpers ----
static inline uint8_t program_r(mcs48 *c, uint16_t a) { return c->rom ? c->rom[a & c->rom_mask] : 0xff; }
static inline uint8_t ram_r(mcs48 *c, uint8_t a) { return c->ram[a & c->ram_mask]; }
static inline void ram_w(mcs48 *c, uint8_t a, uint8_t v) { c->ram[a & c->ram_mask] = v; }
static inline uint8_t port_r(mcs48 *c, int p) { return c->port_r ? c->port_r(c->userdata, p) : 0xff; }
static inline void port_w(mcs48 *c, int p, uint8_t v) { if (c->port_w) c->port_w(c->userdata, p, v); }
static inline int test_r(mcs48 *c, int t) { return c->test_r ? c->test_r(c->userdata, t) : 0; }
static inline uint8_t bus_r(mcs48 *c) { return c->bus_r ? c->bus_r(c->userdata) : 0xff; }
static inline void bus_w(mcs48 *c, uint8_t v) { if (c->bus_w) c->bus_w(c->userdata, v); }
static inline uint8_t ext_r(mcs48 *c, uint8_t a) { return c->ext_r ? c->ext_r(c->userdata, a) : 0xff; }
static inline void ext_w(mcs48 *c, uint8_t a, uint8_t v) { if (c->ext_w) c->ext_w(c->userdata, a, v); }
static inline void prog_w(mcs48 *c, int v) { if (c->prog_w) c->prog_w(c->userdata, v); }

// register Rn of the current bank
#define REG(n) c->ram[(((c->psw & B_FLAG) ? 24 : 0) + (n)) & c->ram_mask]

static inline uint8_t opcode_fetch(mcs48 *c)
{
	uint16_t address = c->pc;
	c->pc = (uint16_t)(((c->pc + 1) & 0x7ff) | (c->pc & 0x800));
	return program_r(c, address);
}
#define argument_fetch opcode_fetch

static void push_pc_psw(mcs48 *c)
{
	uint8_t sp = c->psw & 0x07;
	ram_w(c, (uint8_t)(8 + 2 * sp), (uint8_t)c->pc);
	ram_w(c, (uint8_t)(9 + 2 * sp), (uint8_t)(((c->pc >> 8) & 0x0f) | (c->psw & 0xf0)));
	c->psw = (uint8_t)((c->psw & 0xf0) | ((sp + 1) & 0x07));
}

static void pull_pc_psw(mcs48 *c)
{
	uint8_t sp = (c->psw - 1) & 0x07;
	c->pc = ram_r(c, (uint8_t)(8 + 2 * sp));
	c->pc |= (uint16_t)(ram_r(c, (uint8_t)(9 + 2 * sp)) << 8);
	c->psw = (uint8_t)(((c->pc >> 8) & 0xf0) | sp);
	c->pc &= c->irq_in_progress ? 0x7ff : 0xfff;
}

static void pull_pc(mcs48 *c)
{
	uint8_t sp = (c->psw - 1) & 0x07;
	c->pc = ram_r(c, (uint8_t)(8 + 2 * sp));
	c->pc |= (uint16_t)(ram_r(c, (uint8_t)(9 + 2 * sp)) << 8);
	c->pc &= c->irq_in_progress ? 0x7ff : 0xfff;
	c->psw = (uint8_t)((c->psw & 0xf0) | sp);
}

static void execute_add(mcs48 *c, uint8_t dat)
{
	uint16_t temp = (uint16_t)(c->a + dat);
	uint16_t temp4 = (uint16_t)((c->a & 0x0f) + (dat & 0x0f));
	c->psw &= ~(C_FLAG | A_FLAG);
	c->psw |= (temp4 << 2) & A_FLAG;
	c->psw |= (temp >> 1) & C_FLAG;
	c->a = (uint8_t)temp;
}

static void execute_addc(mcs48 *c, uint8_t dat)
{
	uint8_t carryin = (c->psw & C_FLAG) >> 7;
	uint16_t temp = (uint16_t)(c->a + dat + carryin);
	uint16_t temp4 = (uint16_t)((c->a & 0x0f) + (dat & 0x0f) + carryin);
	c->psw &= ~(C_FLAG | A_FLAG);
	c->psw |= (temp4 << 2) & A_FLAG;
	c->psw |= (temp >> 1) & C_FLAG;
	c->a = (uint8_t)temp;
}

static void execute_jmp(mcs48 *c, uint16_t address)
{
	uint16_t a11 = c->irq_in_progress ? 0 : c->a11;
	c->pc = address | a11;
}

static void execute_call(mcs48 *c, uint16_t address)
{
	push_pc_psw(c);
	execute_jmp(c, address);
}

static void execute_jcc(mcs48 *c, int result)
{
	uint16_t pch = c->pc & 0xf00;
	uint8_t offset = argument_fetch(c);
	if (result)
		c->pc = pch | offset;
}

static uint8_t p2_mask(mcs48 *c)
{
	uint8_t result = 0xff;
	if (!c->upi41)
		return result;
	if (c->flags_enabled)
		result &= (uint8_t)~(P2_OBF | P2_NIBF);
	if (c->dma_enabled)
		result &= (uint8_t)~(P2_DRQ | P2_NDACK);
	return result;
}

static void expander_operation(mcs48 *c, int operation, int port)
{
	// put opcode on low 4 bits of P2 (overwriting latch)
	port_w(c, 2, c->p2 = (uint8_t)((c->p2 & 0xf0) | (operation << 2) | (port & 3)));
	// generate high-to-low transition on PROG line
	prog_w(c, 0);
	// transfer data on low 4 bits of P2
	if (operation != EXPANDER_OP_READ)
		port_w(c, 2, c->p2 = (uint8_t)((c->p2 & 0xf0) | (c->a & 0x0f)));
	else {
		// place P20-P23 in input mode
		port_w(c, 2, c->p2 |= 0x0f);
		// input data to lower 4 bits of A (upper 4 bits are cleared)
		c->a = port_r(c, 2) & 0x0f;
	}
	// generate low-to-high transition on PROG line
	prog_w(c, 1);
}

static void burn_cycles(mcs48 *c, int count)
{
	if (c->timecount_enabled) {
		int timerover = 0;
		if (c->timecount_enabled & TIMER_ENABLED) {
			// accumulate prescaler cycles
			uint8_t oldtimer = c->timer;
			c->prescaler = (uint8_t)(c->prescaler + count);
			c->timer = (uint8_t)(c->timer + (c->prescaler >> 5));
			c->prescaler &= 0x1f;
			timerover = c->timer < oldtimer;
		} else if (c->timecount_enabled & COUNTER_ENABLED) {
			// poll the T1 test input once for each cycle
			for (; count > 0; count--, c->icount--) {
				c->t1_history = (uint8_t)((c->t1_history << 1) | (test_r(c, 1) & 1));
				if ((c->t1_history & 3) == 2) {
					if (++c->timer == 0)
						timerover = 1;
				}
			}
		}
		if (timerover) {
			c->timer_flag = 1;
			// if an overflow occurs with interrupts disabled, the overflow is not stored
			if (c->tirq_enabled)
				c->timer_overflow = 1;
		}
	}
	c->icount -= count;
}

static void check_irqs(mcs48 *c)
{
	if (c->irq_in_progress)
		return;
	// external interrupts take priority
	if ((c->irq_state || (c->sts & STS_IBF) != 0) && c->xirq_enabled) {
		burn_cycles(c, 2);
		c->irq_in_progress = 1;
		// force JNI to be taken (MAME hack)
		if (c->irq_polled) {
			c->pc = (uint16_t)(((c->prevpc + 1) & 0x7ff) | (c->prevpc & 0x800));
			execute_jcc(c, 1);
		}
		execute_call(c, 0x03);
	} else if (c->timer_overflow && c->tirq_enabled) {
		burn_cycles(c, 2);
		c->irq_in_progress = 1;
		execute_call(c, 0x07);
		c->timer_overflow = 0;
	}
}

// ---- public API ----
void mcs48_init(mcs48 *c, int type)
{
	static const struct { uint16_t rom, ram; uint8_t upi; } info[] = {
		{ 0, 64, 0 }, { 1024, 64, 0 }, { 0, 128, 0 }, { 2048, 128, 0 }, { 0, 256, 0 }, { 4096, 256, 0 },
		{ 1024, 64, 1 }, { 1024, 128, 1 }, { 2048, 128, 1 }, { 2048, 128, 1 }, { 2048, 256, 1 }, { 2048, 256, 1 },
	};
	memset(c, 0, sizeof(*c));
	if (type < 0 || type > MCS48_I8742AH)
		type = MCS48_I8048;
	c->type = type;
	c->upi41 = info[type].upi;
	c->ram_mask = (uint16_t)(info[type].ram - 1);
	c->rom_mask = info[type].rom ? (uint16_t)(info[type].rom - 1) : 0xfff;
	c->ea = info[type].rom ? 0 : 1;
}

void mcs48_reset(mcs48 *c)
{
	c->pc = 0;
	c->psw = c->psw & (C_FLAG | A_FLAG);
	c->f1 = 0;
	c->a11 = 0;
	c->tirq_enabled = 0;
	c->xirq_enabled = 0;
	c->timecount_enabled = 0;
	c->timer_flag = 0;
	c->sts = 0;
	c->flags_enabled = 0;
	c->dma_enabled = 0;
	c->irq_in_progress = 0;
	c->timer_overflow = 0;
	c->irq_polled = 0;
	// BUS is set to high-impedance (except when EA=1)
	c->dbbo = 0xff;
	if (!c->upi41)
		bus_w(c, 0xff);
	// port 1 and port 2 are set to input mode
	c->p1 = 0xff;
	c->p2 = 0xff;
	port_w(c, 1, c->p1);
	port_w(c, 2, c->p2);
}

void mcs48_set_irq(mcs48 *c, int state) { c->irq_state = state != 0; }

uint8_t upi41_master_r(mcs48 *c, int offset)
{
	// if just reading the status, return it
	if (offset & 1)
		return (uint8_t)((c->sts & 0xf3) | (c->f1 ? 8 : 0) | ((c->psw & F_FLAG) ? 4 : 0));
	// if the output buffer was full, it gets cleared now
	if (c->sts & STS_OBF) {
		c->sts &= ~STS_OBF;
		if (c->flags_enabled)
			port_w(c, 2, c->p2 &= ~P2_OBF);
	}
	return c->dbbo;
}

void upi41_master_w(mcs48 *c, int offset, uint8_t data)
{
	// (MAME applies this through scheduler synchronize; here it is immediate)
	c->dbbi = data;
	if ((c->sts & STS_IBF) == 0) {
		c->sts |= STS_IBF;
		if (c->flags_enabled)
			port_w(c, 2, c->p2 &= ~P2_NIBF);
	}
	c->f1 = (offset & 1) != 0;
}

#define B(n) burn_cycles(c, n)

int mcs48_execute(mcs48 *c, int cycles)
{
	c->icount = cycles;
	const int upi = c->upi41;
	do {
		check_irqs(c);
		c->irq_polled = 0;
		c->prevpc = c->pc;
		uint8_t op = opcode_fetch(c);
		uint8_t tmp;
		switch (op) {
		case 0x00: B(1); break;   // NOP
		case 0x02:
			if (upi) {   // OUT DBB,A
				B(1);
				c->dbbo = c->a;
				c->sts |= STS_OBF;
				if (c->flags_enabled && (c->p2 & P2_OBF) == 0)
					port_w(c, 2, c->p2 |= P2_OBF);
			} else {   // OUTL BUS,A
				B(2); bus_w(c, c->a);
			}
			break;
		case 0x03: B(2); execute_add(c, argument_fetch(c)); break;
		case 0x04: case 0x24: case 0x44: case 0x64: case 0x84: case 0xa4: case 0xc4: case 0xe4:   // JMP
			B(2); execute_jmp(c, (uint16_t)(argument_fetch(c) | ((op >> 5) << 8))); break;
		case 0x05: B(1); c->xirq_enabled = 1; break;   // EN I
		case 0x07: B(1); c->a--; break;
		case 0x08:
			if (upi) B(1); else { B(2); c->a = bus_r(c); }   // INS A,BUS
			break;
		case 0x09: B(2); c->a = port_r(c, 1) & c->p1; break;
		case 0x0a: B(2); c->a = port_r(c, 2) & c->p2; break;
		case 0x0c: case 0x0d: case 0x0e: case 0x0f: B(2); expander_operation(c, EXPANDER_OP_READ, op & 7); break;
		case 0x10: case 0x11: B(1); tmp = REG(op & 1); ram_w(c, tmp, (uint8_t)(ram_r(c, tmp) + 1)); break;
		case 0x12: case 0x32: case 0x52: case 0x72: case 0x92: case 0xb2: case 0xd2: case 0xf2:   // JBn
			B(2); execute_jcc(c, (c->a >> (op >> 5)) & 1); break;
		case 0x13: B(2); execute_addc(c, argument_fetch(c)); break;
		case 0x14: case 0x34: case 0x54: case 0x74: case 0x94: case 0xb4: case 0xd4: case 0xf4:   // CALL
			B(2); execute_call(c, (uint16_t)(argument_fetch(c) | ((op >> 5) << 8))); break;
		case 0x15: B(1); c->xirq_enabled = 0; break;   // DIS I
		case 0x16: B(2); execute_jcc(c, c->timer_flag); c->timer_flag = 0; break;   // JTF
		case 0x17: B(1); c->a++; break;
		case 0x18: case 0x19: case 0x1a: case 0x1b: case 0x1c: case 0x1d: case 0x1e: case 0x1f: B(1); REG(op & 7)++; break;
		case 0x20: case 0x21: B(1); tmp = c->a; c->a = ram_r(c, REG(op & 1)); ram_w(c, REG(op & 1), tmp); break;
		case 0x22:
			if (upi) {   // IN A,DBB
				B(1);
				c->sts &= ~STS_IBF;
				if (c->flags_enabled && (c->p2 & P2_NIBF) == 0)
					port_w(c, 2, c->p2 |= P2_NIBF);
				c->a = c->dbbi;
			} else
				B(1);
			break;
		case 0x23: B(2); c->a = argument_fetch(c); break;
		case 0x25: B(1); c->tirq_enabled = 1; break;   // EN TCNTI
		case 0x26: B(2); execute_jcc(c, test_r(c, 0) == 0); break;
		case 0x27: B(1); c->a = 0; break;
		case 0x28: case 0x29: case 0x2a: case 0x2b: case 0x2c: case 0x2d: case 0x2e: case 0x2f:
			B(1); tmp = c->a; c->a = REG(op & 7); REG(op & 7) = tmp; break;
		case 0x30: case 0x31: {   // XCHD A,@Rn
			B(1);
			uint8_t r = REG(op & 1), oldram = ram_r(c, r);
			ram_w(c, r, (uint8_t)((oldram & 0xf0) | (c->a & 0x0f)));
			c->a = (uint8_t)((c->a & 0xf0) | (oldram & 0x0f));
			break;
		}
		case 0x35: B(1); c->tirq_enabled = 0; c->timer_overflow = 0; break;   // DIS TCNTI
		case 0x36: B(2); execute_jcc(c, test_r(c, 0) != 0); break;
		case 0x37: B(1); c->a ^= 0xff; break;
		case 0x39: B(2); port_w(c, 1, c->p1 = c->a); break;
		case 0x3a: { B(2); uint8_t mask = p2_mask(c); port_w(c, 2, c->p2 = (uint8_t)((c->p2 & ~mask) | (c->a & mask))); break; }
		case 0x3c: case 0x3d: case 0x3e: case 0x3f: B(2); expander_operation(c, EXPANDER_OP_WRITE, op & 7); break;
		case 0x40: case 0x41: B(1); c->a |= ram_r(c, REG(op & 1)); break;
		case 0x42: B(1); c->a = c->timer; break;
		case 0x43: B(2); c->a |= argument_fetch(c); break;
		case 0x45:   // STRT CNT
			B(1);
			if (!(c->timecount_enabled & COUNTER_ENABLED))
				c->t1_history = (uint8_t)test_r(c, 1);
			c->timecount_enabled = COUNTER_ENABLED;
			break;
		case 0x46: B(2); execute_jcc(c, test_r(c, 1) == 0); break;
		case 0x47: B(1); c->a = (uint8_t)((c->a << 4) | (c->a >> 4)); break;
		case 0x48: case 0x49: case 0x4a: case 0x4b: case 0x4c: case 0x4d: case 0x4e: case 0x4f: B(1); c->a |= REG(op & 7); break;
		case 0x50: case 0x51: B(1); c->a &= ram_r(c, REG(op & 1)); break;
		case 0x53: B(2); c->a &= argument_fetch(c); break;
		case 0x55: B(1); c->timecount_enabled = TIMER_ENABLED; c->prescaler = 0; break;   // STRT T
		case 0x56: B(2); execute_jcc(c, test_r(c, 1) != 0); break;
		case 0x57:   // DA A
			B(1);
			if ((c->a & 0x0f) > 0x09 || (c->psw & A_FLAG)) {
				if (c->a > 0xf9)
					c->psw |= C_FLAG;
				c->a += 0x06;
			}
			if ((c->a & 0xf0) > 0x90 || (c->psw & C_FLAG)) {
				c->a += 0x60;
				c->psw |= C_FLAG;
			}
			break;
		case 0x58: case 0x59: case 0x5a: case 0x5b: case 0x5c: case 0x5d: case 0x5e: case 0x5f: B(1); c->a &= REG(op & 7); break;
		case 0x60: case 0x61: B(1); execute_add(c, ram_r(c, REG(op & 1))); break;
		case 0x62: B(1); c->timer = c->a; break;
		case 0x65: B(1); c->timecount_enabled = 0; break;   // STOP TCNT
		case 0x67: { B(1); uint8_t newc = (uint8_t)((c->a << 7) & C_FLAG); c->a = (uint8_t)((c->a >> 1) | (c->psw & C_FLAG)); c->psw = (uint8_t)((c->psw & ~C_FLAG) | newc); break; }
		case 0x68: case 0x69: case 0x6a: case 0x6b: case 0x6c: case 0x6d: case 0x6e: case 0x6f: B(1); execute_add(c, REG(op & 7)); break;
		case 0x70: case 0x71: B(1); execute_addc(c, ram_r(c, REG(op & 1))); break;
		case 0x75: B(1); break;   // ENT0 CLK (MCS-48; illegal on UPI-41): T0 clock output not emulated
		case 0x76: B(2); execute_jcc(c, c->f1); break;   // JF1
		case 0x77: B(1); c->a = (uint8_t)((c->a >> 1) | (c->a << 7)); break;
		case 0x78: case 0x79: case 0x7a: case 0x7b: case 0x7c: case 0x7d: case 0x7e: case 0x7f: B(1); execute_addc(c, REG(op & 7)); break;
		case 0x80: case 0x81:
			if (upi) B(1); else { B(2); c->a = ext_r(c, REG(op & 1)); }   // MOVX A,@Rn
			break;
		case 0x83: B(2); pull_pc(c); break;   // RET
		case 0x85: B(1); c->psw &= ~F_FLAG; break;   // CLR F0
		case 0x86:
			if (upi) { B(2); execute_jcc(c, (c->sts & STS_OBF) != 0); }   // JOBF
			else { B(2); c->irq_polled = c->irq_state == 0; execute_jcc(c, c->irq_state != 0); }   // JNI
			break;
		case 0x88:
			if (upi) B(1); else { B(2); bus_w(c, bus_r(c) | argument_fetch(c)); }   // ORL BUS,#n
			break;
		case 0x89: B(2); port_w(c, 1, c->p1 |= argument_fetch(c)); break;
		case 0x8a: B(2); port_w(c, 2, c->p2 |= argument_fetch(c) & p2_mask(c)); break;
		case 0x8c: case 0x8d: case 0x8e: case 0x8f: B(2); expander_operation(c, EXPANDER_OP_OR, op & 7); break;
		case 0x90:
			if (upi) { B(1); c->sts = (uint8_t)((c->sts & 0x0f) | (c->a & 0xf0)); }   // MOV STS,A
			else { B(2); ext_w(c, REG(0), c->a); }   // MOVX @R0,A
			break;
		case 0x91:
			if (upi) B(1); else { B(2); ext_w(c, REG(1), c->a); }
			break;
		case 0x93: B(2); c->irq_in_progress = 0; pull_pc_psw(c); break;   // RETR
		case 0x95: B(1); c->psw ^= F_FLAG; break;   // CPL F0
		case 0x96: B(2); execute_jcc(c, c->a != 0); break;
		case 0x97: B(1); c->psw &= ~C_FLAG; break;
		case 0x98:
			if (upi) B(1); else { B(2); bus_w(c, bus_r(c) & argument_fetch(c)); }   // ANL BUS,#n
			break;
		case 0x99: B(2); port_w(c, 1, c->p1 &= argument_fetch(c)); break;
		case 0x9a: B(2); port_w(c, 2, c->p2 &= (uint8_t)(argument_fetch(c) | ~p2_mask(c))); break;
		case 0x9c: case 0x9d: case 0x9e: case 0x9f: B(2); expander_operation(c, EXPANDER_OP_AND, op & 7); break;
		case 0xa0: case 0xa1: B(1); ram_w(c, REG(op & 1), c->a); break;
		case 0xa3: B(2); c->a = program_r(c, (uint16_t)((c->pc & 0xf00) | c->a)); break;   // MOVP A,@A
		case 0xa5: B(1); c->f1 = 0; break;
		case 0xa7: B(1); c->psw ^= C_FLAG; break;
		case 0xa8: case 0xa9: case 0xaa: case 0xab: case 0xac: case 0xad: case 0xae: case 0xaf: B(1); REG(op & 7) = c->a; break;
		case 0xb0: case 0xb1: B(2); tmp = argument_fetch(c); ram_w(c, REG(op & 1), tmp); break;
		case 0xb3: B(2); c->pc &= 0xf00; c->pc |= program_r(c, (uint16_t)(c->pc | c->a)); break;   // JMPP @A
		case 0xb5: B(1); c->f1 = !c->f1; break;
		case 0xb6: B(2); execute_jcc(c, (c->psw & F_FLAG) != 0); break;   // JF0
		case 0xb8: case 0xb9: case 0xba: case 0xbb: case 0xbc: case 0xbd: case 0xbe: case 0xbf: B(2); REG(op & 7) = argument_fetch(c); break;
		case 0xc5: B(1); c->psw &= ~B_FLAG; break;   // SEL RB0
		case 0xc6: B(2); execute_jcc(c, c->a == 0); break;
		case 0xc7: B(1); c->a = c->psw | 0x08; break;
		case 0xc8: case 0xc9: case 0xca: case 0xcb: case 0xcc: case 0xcd: case 0xce: case 0xcf: B(1); REG(op & 7)--; break;
		case 0xd0: case 0xd1: B(1); c->a ^= ram_r(c, REG(op & 1)); break;
		case 0xd3: B(2); c->a ^= argument_fetch(c); break;
		case 0xd5: B(1); c->psw |= B_FLAG; break;   // SEL RB1
		case 0xd6:
			if (upi) { B(2); c->irq_polled = (c->sts & STS_IBF) != 0; execute_jcc(c, (c->sts & STS_IBF) == 0); }   // JNIBF
			else B(1);
			break;
		case 0xd7: B(1); c->psw = c->a & ~0x08; break;   // MOV PSW,A
		case 0xd8: case 0xd9: case 0xda: case 0xdb: case 0xdc: case 0xdd: case 0xde: case 0xdf: B(1); c->a ^= REG(op & 7); break;
		case 0xe3: B(2); c->a = program_r(c, (uint16_t)(0x300 | c->a)); break;   // MOVP3
		case 0xe5:
			if (upi) { B(1); c->dma_enabled = 1; port_w(c, 2, c->p2); }   // EN DMA
			else { B(1); c->a11 = 0x000; }   // SEL MB0
			break;
		case 0xe6: B(2); execute_jcc(c, (c->psw & C_FLAG) == 0); break;
		case 0xe7: B(1); c->a = (uint8_t)((c->a << 1) | (c->a >> 7)); break;
		case 0xe8: case 0xe9: case 0xea: case 0xeb: case 0xec: case 0xed: case 0xee: case 0xef:   // DJNZ
			B(2); tmp = (uint8_t)(--REG(op & 7)); execute_jcc(c, tmp != 0); break;
		case 0xf0: case 0xf1: B(1); c->a = ram_r(c, REG(op & 1)); break;
		case 0xf5:
			if (upi) { B(1); c->flags_enabled = 1; port_w(c, 2, c->p2); }   // EN FLAGS
			else { B(1); c->a11 = 0x800; }   // SEL MB1
			break;
		case 0xf6: B(2); execute_jcc(c, (c->psw & C_FLAG) != 0); break;
		case 0xf7: { B(1); uint8_t newc = c->a & C_FLAG; c->a = (uint8_t)((c->a << 1) | (c->psw >> 7)); c->psw = (uint8_t)((c->psw & ~C_FLAG) | newc); break; }
		case 0xf8: case 0xf9: case 0xfa: case 0xfb: case 0xfc: case 0xfd: case 0xfe: case 0xff: B(1); c->a = REG(op & 7); break;
		default: B(1); break;   // illegal
		}
	} while (c->icount > 0);
	return cycles - c->icount;
}
