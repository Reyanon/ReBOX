/*  Intel 8080 / 8085A CPU core, ported to C from MAME src/devices/cpu/i8085/i8085.cpp
    (license BSD-3-Clause, copyright-holders: Juergen Buchmueller, Roberto Fresca, Grull Osgo;
    thanks-to: Marcel De Kogel). "Portable I8085A emulator V1.3, Copyright Juergen Buchmueller,
    partially based on information out of Z80Em by Marcel De Kogel."
    The opcode switch, flag handling, cycle tables, interrupt priorities and the post-EI delay are
    MAME's; the device framework (address spaces, devcb, state interface) is replaced by callbacks.
    Cycle counts are CPU states: on the 8085 the caller divides its input clock by 2 (MAME
    execute_clocks_to_cycles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "i8080.h"

#include <string.h>

#define SF  0x80
#define ZF  0x40
#define KF  0x20
#define HF  0x10
#define X3F 0x08
#define PF  0x04
#define VF  0x02
#define CF  0x01

#define IM_SID 0x80
#define IM_I75 0x40
#define IM_I65 0x20
#define IM_I55 0x10
#define IM_IE  0x08
#define IM_M75 0x04
#define IM_M65 0x02
#define IM_M55 0x01

#define ADDR_TRAP  0x0024
#define ADDR_RST55 0x002c
#define ADDR_RST65 0x0034
#define ADDR_RST75 0x003c

static const uint8_t lut_cycles_8080[256] = {
/*      0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F  */
/* 0 */ 4, 10,7, 5, 5, 5, 7, 4, 4, 10,7, 5, 5, 5, 7, 4,
/* 1 */ 4, 10,7, 5, 5, 5, 7, 4, 4, 10,7, 5, 5, 5, 7, 4,
/* 2 */ 4, 10,16,5, 5, 5, 7, 4, 4, 10,16,5, 5, 5, 7, 4,
/* 3 */ 4, 10,13,5, 10,10,10,4, 4, 10,13,5, 5, 5, 7, 4,
/* 4 */ 5, 5, 5, 5, 5, 5, 7, 5, 5, 5, 5, 5, 5, 5, 7, 5,
/* 5 */ 5, 5, 5, 5, 5, 5, 7, 5, 5, 5, 5, 5, 5, 5, 7, 5,
/* 6 */ 5, 5, 5, 5, 5, 5, 7, 5, 5, 5, 5, 5, 5, 5, 7, 5,
/* 7 */ 7, 7, 7, 7, 7, 7, 7, 7, 5, 5, 5, 5, 5, 5, 7, 5,
/* 8 */ 4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
/* 9 */ 4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
/* A */ 4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
/* B */ 4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
/* C */ 5, 10,10,10,11,11,7, 11,5, 10,10,10,11,11,7, 11,
/* D */ 5, 10,10,10,11,11,7, 11,5, 10,10,10,11,11,7, 11,
/* E */ 5, 10,10,18,11,11,7, 11,5, 5, 10,4, 11,11,7, 11,
/* F */ 5, 10,10,4, 11,11,7, 11,5, 5, 10,4, 11,11,7, 11 };

static const uint8_t lut_cycles_8085[256] = {
/*      0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F  */
/* 0 */ 4, 10,7, 6, 4, 4, 7, 4, 10,10,7, 6, 4, 4, 7, 4,
/* 1 */ 7, 10,7, 6, 4, 4, 7, 4, 10,10,7, 6, 4, 4, 7, 4,
/* 2 */ 4, 10,16,6, 4, 4, 7, 4, 10,10,16,6, 4, 4, 7, 4,
/* 3 */ 4, 10,13,6, 10,10,10,4, 10,10,13,6, 4, 4, 7, 4,
/* 4 */ 4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
/* 5 */ 4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
/* 6 */ 4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
/* 7 */ 7, 7, 7, 7, 7, 7, 5, 7, 4, 4, 4, 4, 4, 4, 7, 4,
/* 8 */ 4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
/* 9 */ 4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
/* A */ 4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
/* B */ 4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
/* C */ 6, 10,7, 7, 9, 12,7, 12,6, 10,7, 6, 9, 9, 7, 12,
/* D */ 6, 10,7, 10,9, 12,7, 12,6, 10,7, 10,9, 7, 7, 12,
/* E */ 6, 10,7, 16,9, 12,7, 12,6, 6, 7, 4, 9, 10,7, 12,
/* F */ 6, 10,7, 4, 9, 12,7, 12,6, 6, 7, 4, 9, 7, 7, 12 };

static uint8_t lut_zs[256], lut_zsp[256];
static int s_tables;

static void init_tables(void)
{
	for (unsigned i = 0; i < 256; i++) {
		uint8_t zs = 0;
		if (i == 0) zs |= ZF;
		if (i & 0x80) zs |= SF;
		unsigned v = i, n = 0;
		while (v) { n += v & 1; v >>= 1; }
		lut_zs[i] = zs;
		lut_zsp[i] = zs | ((n & 1) ? 0 : PF);
	}
	s_tables = 1;
}

#define S (&cpu->s)
#define BC ((uint16_t)(S->b << 8 | S->c))
#define DE ((uint16_t)(S->d << 8 | S->e))
#define HL ((uint16_t)(S->h << 8 | S->l))
#define SET_BC(v) do { uint16_t v_ = (v); S->b = (uint8_t)(v_ >> 8); S->c = (uint8_t)v_; } while (0)
#define SET_DE(v) do { uint16_t v_ = (v); S->d = (uint8_t)(v_ >> 8); S->e = (uint8_t)v_; } while (0)
#define SET_HL(v) do { uint16_t v_ = (v); S->h = (uint8_t)(v_ >> 8); S->l = (uint8_t)v_; } while (0)
#define SET_AF(v) do { uint16_t v_ = (v); S->a = (uint8_t)(v_ >> 8); S->f = (uint8_t)v_; } while (0)
#define AF ((uint16_t)(S->a << 8 | S->f))

static inline int ret_taken(const i8080_t *cpu) { (void)cpu; return 6; }
static inline int jmp_taken(const i8080_t *cpu) { return S->is_8085 ? 3 : 0; }
static inline int call_taken(const i8080_t *cpu) { return S->is_8085 ? 9 : 6; }

static void set_sod(i8080_t *cpu, int state)
{
	if (state != 0 && S->sod_state == 0) {
		S->sod_state = 1;
		if (cpu->sod) cpu->sod(cpu->user, 1);
	} else if (state == 0 && S->sod_state != 0) {
		S->sod_state = 0;
		if (cpu->sod) cpu->sod(cpu->user, 0);
	}
}

static void set_inte(i8080_t *cpu, int state)
{
	if (state != 0 && (S->im & IM_IE) == 0) {
		S->im |= IM_IE;
		if (cpu->inte) cpu->inte(cpu->user, 1);
	} else if (state == 0 && (S->im & IM_IE) != 0) {
		S->im &= ~IM_IE;
		if (cpu->inte) cpu->inte(cpu->user, 0);
	}
}

#define set_status(st) (S->status = (st))

static uint8_t get_rim_value(i8080_t *cpu)
{
	uint8_t result = S->im;
	int sid = cpu->sid ? cpu->sid(cpu->user) : 0;
	result &= ~(IM_I65 | IM_I55);
	if (S->irq_state[I8085_RST65_LINE]) result |= IM_I65;
	if (S->irq_state[I8085_RST55_LINE]) result |= IM_I55;
	result = (uint8_t)((result & ~IM_SID) | (sid ? IM_SID : 0));
	return result;
}

static inline uint8_t mem_rd(i8080_t *cpu, uint16_t a)
{
	const uint8_t *p = cpu->rpage[a >> 8];
	return p ? p[a & 0xff] : cpu->read(cpu->user, a);
}

static inline uint8_t read_inta(i8080_t *cpu)
{
	return cpu->irq_ack ? cpu->irq_ack(cpu->user, I8080_INTR_LINE) : 0xff;
}

static inline uint8_t read_arg(i8080_t *cpu)
{
	if (S->in_acknowledge)
		return read_inta(cpu);
	return mem_rd(cpu, S->pc++);
}

static inline uint16_t read_arg16(i8080_t *cpu)
{
	uint8_t lo, hi;
	if (S->in_acknowledge) {
		lo = read_inta(cpu);
		hi = read_inta(cpu);
	} else {
		lo = mem_rd(cpu, S->pc++);
		hi = mem_rd(cpu, S->pc++);
	}
	return (uint16_t)(hi << 8 | lo);
}

static inline uint8_t read_op(i8080_t *cpu) { return mem_rd(cpu, S->pc++); }
static inline uint8_t read_mem(i8080_t *cpu, uint16_t a) { return mem_rd(cpu, a); }
static inline void write_mem(i8080_t *cpu, uint16_t a, uint8_t v) { cpu->write(cpu->user, a, v); }

static inline void op_push(i8080_t *cpu, uint16_t p)
{
	cpu->write(cpu->user, --S->sp, (uint8_t)(p >> 8));
	cpu->write(cpu->user, --S->sp, (uint8_t)p);
}

static inline uint16_t op_pop(i8080_t *cpu)
{
	uint8_t lo = mem_rd(cpu, S->sp++);
	uint8_t hi = mem_rd(cpu, S->sp++);
	return (uint16_t)(hi << 8 | lo);
}

// logical
static inline void op_ora(i8080_t *cpu, uint8_t v) { S->a |= v; S->f = lut_zsp[S->a]; }
static inline void op_xra(i8080_t *cpu, uint8_t v) { S->a ^= v; S->f = lut_zsp[S->a]; }
static inline void op_ana(i8080_t *cpu, uint8_t v)
{
	uint8_t hc = (uint8_t)(((S->a | v) << 1) & HF);
	S->a &= v;
	S->f = lut_zsp[S->a];
	S->f |= S->is_8085 ? HF : hc;
}

// increase / decrease
static inline uint8_t op_inr(i8080_t *cpu, uint8_t v)
{
	uint8_t hc = ((v & 0x0f) == 0x0f) ? HF : 0;
	v++;
	S->f = (uint8_t)((S->f & CF) | lut_zsp[v] | hc);
	return v;
}

static inline uint8_t op_dcr(i8080_t *cpu, uint8_t v)
{
	uint8_t hc = ((v & 0x0f) != 0x00) ? HF : 0;
	v--;
	S->f = (uint8_t)((S->f & CF) | lut_zsp[v] | hc | VF);
	return v;
}

// arithmetic
static inline void op_add(i8080_t *cpu, uint8_t v)
{
	int q = S->a + v;
	S->f = (uint8_t)(lut_zsp[q & 0xff] | ((q >> 8) & CF) | ((S->a ^ q ^ v) & HF));
	S->a = (uint8_t)q;
}

static inline void op_adc(i8080_t *cpu, uint8_t v)
{
	int q = S->a + v + (S->f & CF);
	S->f = (uint8_t)(lut_zsp[q & 0xff] | ((q >> 8) & CF) | ((S->a ^ q ^ v) & HF));
	S->a = (uint8_t)q;
}

static inline void op_sub(i8080_t *cpu, uint8_t v)
{
	int q = S->a - v;
	S->f = (uint8_t)(lut_zsp[q & 0xff] | ((q >> 8) & CF) | (~(S->a ^ q ^ v) & HF) | VF);
	S->a = (uint8_t)q;
}

static inline void op_sbb(i8080_t *cpu, uint8_t v)
{
	int q = S->a - v - (S->f & CF);
	S->f = (uint8_t)(lut_zsp[q & 0xff] | ((q >> 8) & CF) | (~(S->a ^ q ^ v) & HF) | VF);
	S->a = (uint8_t)q;
}

static inline void op_cmp(i8080_t *cpu, uint8_t v)
{
	int q = S->a - v;
	S->f = (uint8_t)(lut_zsp[q & 0xff] | ((q >> 8) & CF) | (~(S->a ^ q ^ v) & HF) | VF);
}

static inline void op_dad(i8080_t *cpu, uint16_t v)
{
	int q = HL + v;
	S->f = (uint8_t)((S->f & ~CF) | ((q >> 16) & CF));
	SET_HL((uint16_t)q);
}

// jumps
static inline void op_jmp(i8080_t *cpu, int cond)
{
	if (cond) {
		S->pc = read_arg16(cpu);
		S->icount -= jmp_taken(cpu);
	} else {
		S->pc += 2;
	}
}

static inline void op_call(i8080_t *cpu, int cond)
{
	if (cond) {
		uint16_t p = read_arg16(cpu);
		S->icount -= call_taken(cpu);
		op_push(cpu, S->pc);
		S->pc = p;
	} else {
		S->pc += 2;
	}
}

static inline void op_ret(i8080_t *cpu, int cond)
{
	// conditional RET only
	if (cond) {
		S->icount -= ret_taken(cpu);
		S->pc = op_pop(cpu);
	}
}

static inline void op_rst(i8080_t *cpu, uint8_t v)
{
	op_push(cpu, S->pc);
	S->pc = (uint16_t)(8 * v);
}

static inline void inx_flag(i8080_t *cpu, uint16_t v, uint16_t wrap)
{
	if (S->is_8085) {
		if (v == wrap)
			S->f |= KF;
		else
			S->f &= ~KF;
	}
}

static void execute_one(i8080_t *cpu, int opcode);

static void break_halt_for_interrupt(i8080_t *cpu)
{
	if (S->halt) {
		S->pc++;
		S->halt = 0;
		set_status(0x26);   // int ack while halt
	} else
		set_status(0x23);   // int ack
	S->in_acknowledge = 1;
}

static void take_vector(i8080_t *cpu, int line, uint16_t addr)
{
	break_halt_for_interrupt(cpu);
	if (cpu->irq_ack)
		cpu->irq_ack(cpu->user, line);
	op_push(cpu, S->pc);
	set_inte(cpu, 0);
	S->pc = addr;
	S->icount -= 11;
}

static void check_for_interrupts(i8080_t *cpu)
{
	if (S->trap_pending) {
		// TRAP is the highest priority; the first RIM after a TRAP reflects the original IE state
		S->trap_im_copy = (uint8_t)(S->im | 0x80);
		S->trap_pending = 0;
		take_vector(cpu, I8085_TRAP_LINE, ADDR_TRAP);
	} else if ((S->im & IM_I75) && !(S->im & IM_M75) && (S->im & IM_IE)) {
		S->im &= ~IM_I75;
		take_vector(cpu, I8085_RST75_LINE, ADDR_RST75);
	} else if (S->irq_state[I8085_RST65_LINE] && !(S->im & IM_M65) && (S->im & IM_IE)) {
		take_vector(cpu, I8085_RST65_LINE, ADDR_RST65);
	} else if (S->irq_state[I8085_RST55_LINE] && !(S->im & IM_M55) && (S->im & IM_IE)) {
		take_vector(cpu, I8085_RST55_LINE, ADDR_RST55);
	} else if (S->irq_state[I8080_INTR_LINE] && (S->im & IM_IE)) {
		// classic INTR: the acknowledge cycle puts an opcode on the bus (normally RST n)
		break_halt_for_interrupt(cpu);
		uint8_t vector = read_inta(cpu);
		set_inte(cpu, 0);
		execute_one(cpu, vector);
	}
}

void i8080_init(i8080_t *cpu, int is_8085)
{
	if (!s_tables)
		init_tables();
	memset(&cpu->s, 0, sizeof(cpu->s));
	S->sod_state = 1;   // SOD will go low at reset
	S->is_8085 = (uint8_t)(is_8085 != 0);
}

void i8080_reset(i8080_t *cpu)
{
	S->pc = 0;
	S->halt = 0;
	S->im &= ~IM_I75;
	S->im |= IM_M55 | IM_M65 | IM_M75;
	S->after_ei = 0;
	S->trap_pending = 0;
	S->trap_im_copy = 0;
	set_inte(cpu, 0);
	set_sod(cpu, 0);
}

void i8080_set_input(i8080_t *cpu, int line, int state)
{
	int newstate = state != 0;
	if (line == I8085_TRAP_LINE) {
		// TRAP is level and edge-triggered NMI
		if (!S->nmi_state && newstate)
			S->trap_pending = 1;
		else if (!newstate)
			S->trap_pending = 0;
		S->nmi_state = (uint8_t)newstate;
	} else if (line == I8085_RST75_LINE) {
		// RST7.5 is edge-triggered
		if (!S->irq_state[I8085_RST75_LINE] && newstate)
			S->im |= IM_I75;
		S->irq_state[I8085_RST75_LINE] = (uint8_t)newstate;
	} else if (line >= 0 && line < 4) {
		S->irq_state[line] = (uint8_t)newstate;
	}
}

int i8080_run(i8080_t *cpu, int clocks)
{
	S->icount += clocks;
	if (S->icount <= 0)
		return 0;
	int start = S->icount;
	// check for TRAPs before diving in (can't do others because of after_ei)
	if (S->trap_pending || S->after_ei == 0)
		check_for_interrupts(cpu);
	do {
		// the instruction after an EI does not take an interrupt
		if (S->after_ei != 0 && --S->after_ei == 0)
			check_for_interrupts(cpu);
		S->in_acknowledge = 0;
		execute_one(cpu, read_op(cpu));
	} while (S->icount > 0);
	return start - S->icount;
}

static void execute_one(i8080_t *cpu, int opcode)
{
	const int is85 = S->is_8085;
	S->icount -= is85 ? lut_cycles_8085[opcode] : lut_cycles_8080[opcode];

	switch (opcode) {
	case 0x00: break;                                   // NOP
	case 0x01: SET_BC(read_arg16(cpu)); break;          // LXI B,nnnn
	case 0x02: write_mem(cpu, BC, S->a); break;         // STAX B
	case 0x03: SET_BC(BC + 1); inx_flag(cpu, BC, 0x0000); break;   // INX B
	case 0x04: S->b = op_inr(cpu, S->b); break;         // INR B
	case 0x05: S->b = op_dcr(cpu, S->b); break;         // DCR B
	case 0x06: S->b = read_arg(cpu); break;             // MVI B,nn
	case 0x07:                                          // RLC
		S->a = (uint8_t)((S->a << 1) | (S->a >> 7));
		S->f = (uint8_t)((S->f & 0xfe) | (S->a & CF));
		break;

	case 0x08:   // 8085: undocumented DSUB (HL - BC), otherwise undocumented NOP
		if (is85) {
			int q_low = S->l - S->c;
			uint8_t res_low = (uint8_t)q_low;
			S->f = (uint8_t)(lut_zs[res_low] | ((q_low >> 8) & CF) | ((S->l ^ res_low ^ S->c) & HF) |
			                 (((S->c ^ S->l) & (S->l ^ res_low) & SF) >> 5));
			S->l = res_low;
			int q_high = S->h - S->b - (S->f & CF);
			uint8_t res_high = (uint8_t)q_high;
			S->f = (uint8_t)(lut_zs[res_high] | ((q_high >> 8) & CF) | ((S->h ^ res_high ^ S->b) & HF) |
			                 (((S->b ^ S->h) & (S->h ^ res_high) & SF) >> 5));
			S->h = res_high;
			S->f = (uint8_t)((S->f & ~ZF) | (((S->l | S->h) == 0) ? ZF : 0));
		}
		break;
	case 0x09: op_dad(cpu, BC); break;                  // DAD B
	case 0x0a: S->a = read_mem(cpu, BC); break;         // LDAX B
	case 0x0b: SET_BC(BC - 1); inx_flag(cpu, BC, 0xffff); break;   // DCX B
	case 0x0c: S->c = op_inr(cpu, S->c); break;         // INR C
	case 0x0d: S->c = op_dcr(cpu, S->c); break;         // DCR C
	case 0x0e: S->c = read_arg(cpu); break;             // MVI C,nn
	case 0x0f:                                          // RRC
		S->f = (uint8_t)((S->f & 0xfe) | (S->a & CF));
		S->a = (uint8_t)((S->a >> 1) | (S->a << 7));
		break;

	case 0x10:   // 8085: undocumented ARHL, otherwise undocumented NOP
		if (is85) {
			S->f = (uint8_t)((S->f & ~CF) | (S->l & CF));
			SET_HL((uint16_t)((HL & 0x8000) | (HL >> 1)));
		}
		break;
	case 0x11: SET_DE(read_arg16(cpu)); break;          // LXI D,nnnn
	case 0x12: write_mem(cpu, DE, S->a); break;         // STAX D
	case 0x13: SET_DE(DE + 1); inx_flag(cpu, DE, 0x0000); break;   // INX D
	case 0x14: S->d = op_inr(cpu, S->d); break;         // INR D
	case 0x15: S->d = op_dcr(cpu, S->d); break;         // DCR D
	case 0x16: S->d = read_arg(cpu); break;             // MVI D,nn
	case 0x17: {                                        // RAL
		int c = S->f & CF;
		S->f = (uint8_t)((S->f & 0xfe) | (S->a >> 7));
		S->a = (uint8_t)((S->a << 1) | c);
		break;
	}

	case 0x18:   // 8085: undocumented RDEL, otherwise undocumented NOP
		if (is85) {
			int c = S->f & CF;
			S->f = (uint8_t)((S->f & ~(CF | VF)) | (S->d >> 7));
			SET_DE((uint16_t)((DE << 1) | c));
			if ((((DE >> 15) ^ S->f) & CF) != 0)
				S->f |= VF;
		}
		break;
	case 0x19: op_dad(cpu, DE); break;                  // DAD D
	case 0x1a: S->a = read_mem(cpu, DE); break;         // LDAX D
	case 0x1b: SET_DE(DE - 1); inx_flag(cpu, DE, 0xffff); break;   // DCX D
	case 0x1c: S->e = op_inr(cpu, S->e); break;         // INR E
	case 0x1d: S->e = op_dcr(cpu, S->e); break;         // DCR E
	case 0x1e: S->e = read_arg(cpu); break;             // MVI E,nn
	case 0x1f: {                                        // RAR
		int c = (S->f & CF) << 7;
		S->f = (uint8_t)((S->f & 0xfe) | (S->a & CF));
		S->a = (uint8_t)((S->a >> 1) | c);
		break;
	}

	case 0x20:   // 8085: RIM, otherwise undocumented NOP
		if (is85) {
			S->a = get_rim_value(cpu);
			if (S->trap_im_copy & 0x80)
				S->a = (uint8_t)((S->a & ~IM_IE) | (S->trap_im_copy & IM_IE));
			S->trap_im_copy = 0;
		}
		break;
	case 0x21: SET_HL(read_arg16(cpu)); break;          // LXI H,nnnn
	case 0x22:                                          // SHLD nnnn
		S->wz = read_arg16(cpu);
		write_mem(cpu, S->wz, S->l);
		S->wz++;
		write_mem(cpu, S->wz, S->h);
		break;
	case 0x23: SET_HL(HL + 1); inx_flag(cpu, HL, 0x0000); break;   // INX H
	case 0x24: S->h = op_inr(cpu, S->h); break;         // INR H
	case 0x25: S->h = op_dcr(cpu, S->h); break;         // DCR H
	case 0x26: S->h = read_arg(cpu); break;             // MVI H,nn
	case 0x27: {                                        // DAA
		uint8_t t = S->a;
		if ((S->f & HF) || ((S->a & 0xf) > 9))
			t += 6;
		if ((S->f & CF) || (S->a > 0x99))
			t += 0x60;
		S->f = (uint8_t)((S->f & 0x23) | ((S->a > 0x99) ? 1 : 0) | ((S->a ^ t) & 0x10) | lut_zsp[t]);
		S->a = t;
		S->wz = (uint16_t)((S->wz & 0xff) | (t << 8));
		break;
	}

	case 0x28:   // 8085: undocumented LDHI nn, otherwise undocumented NOP
		if (is85) {
			S->wz = read_arg(cpu);
			SET_DE((uint16_t)(HL + S->wz));
		}
		break;
	case 0x29: op_dad(cpu, HL); break;                  // DAD H
	case 0x2a:                                          // LHLD nnnn
		S->wz = read_arg16(cpu);
		S->l = read_mem(cpu, S->wz);
		S->wz++;
		S->h = read_mem(cpu, S->wz);
		break;
	case 0x2b: SET_HL(HL - 1); inx_flag(cpu, HL, 0xffff); break;   // DCX H
	case 0x2c: S->l = op_inr(cpu, S->l); break;         // INR L
	case 0x2d: S->l = op_dcr(cpu, S->l); break;         // DCR L
	case 0x2e: S->l = read_arg(cpu); break;             // MVI L,nn
	case 0x2f:                                          // CMA
		S->a ^= 0xff;
		if (is85)
			S->f |= VF;
		break;

	case 0x30:   // 8085: SIM, otherwise undocumented NOP
		if (is85) {
			if (S->a & 0x08) {
				S->im &= ~(IM_M55 | IM_M65 | IM_M75 | IM_I55 | IM_I65);
				S->im |= S->a & (IM_M55 | IM_M65 | IM_M75);
				if ((S->im & IM_M55) == 0 && S->irq_state[I8085_RST55_LINE])
					S->im |= IM_I55;
				if ((S->im & IM_M65) == 0 && S->irq_state[I8085_RST65_LINE])
					S->im |= IM_I65;
			}
			if (S->a & 0x10)
				S->im &= ~IM_I75;
			if (S->a & 0x40)
				set_sod(cpu, S->a >> 7);
			check_for_interrupts(cpu);
		}
		break;
	case 0x31: S->sp = read_arg16(cpu); break;          // LXI SP,nnnn
	case 0x32:                                          // STA nnnn
		S->wz = read_arg16(cpu);
		write_mem(cpu, S->wz, S->a);
		break;
	case 0x33: S->sp++; inx_flag(cpu, S->sp, 0x0000); break;   // INX SP
	case 0x34: {                                        // INR M
		uint8_t v = op_inr(cpu, read_mem(cpu, HL));
		S->wz = (uint16_t)((S->wz & 0xff00) | v);
		write_mem(cpu, HL, v);
		break;
	}
	case 0x35: {                                        // DCR M
		uint8_t v = op_dcr(cpu, read_mem(cpu, HL));
		S->wz = (uint16_t)((S->wz & 0xff00) | v);
		write_mem(cpu, HL, v);
		break;
	}
	case 0x36: {                                        // MVI M,nn
		uint8_t v = read_arg(cpu);
		S->wz = (uint16_t)((S->wz & 0xff00) | v);
		write_mem(cpu, HL, v);
		break;
	}
	case 0x37: S->f = (uint8_t)((S->f & 0xfe) | CF); break;   // STC

	case 0x38:   // 8085: undocumented LDSI nn, otherwise undocumented NOP
		if (is85) {
			S->wz = read_arg(cpu);
			SET_DE((uint16_t)(S->sp + S->wz));
		}
		break;
	case 0x39: op_dad(cpu, S->sp); break;               // DAD SP
	case 0x3a:                                          // LDA nnnn
		S->wz = read_arg16(cpu);
		S->a = read_mem(cpu, S->wz);
		break;
	case 0x3b: S->sp--; inx_flag(cpu, S->sp, 0xffff); break;   // DCX SP
	case 0x3c: S->a = op_inr(cpu, S->a); break;         // INR A
	case 0x3d: S->a = op_dcr(cpu, S->a); break;         // DCR A
	case 0x3e: S->a = read_arg(cpu); break;             // MVI A,nn
	case 0x3f: S->f = (uint8_t)((S->f & 0xfe) | (~S->f & CF)); break;   // CMC

	// MOV [B/C/D/E/H/L/M/A],[B/C/D/E/H/L/M/A]
	case 0x40: break;
	case 0x41: S->b = S->c; break;
	case 0x42: S->b = S->d; break;
	case 0x43: S->b = S->e; break;
	case 0x44: S->b = S->h; break;
	case 0x45: S->b = S->l; break;
	case 0x46: S->b = read_mem(cpu, HL); break;
	case 0x47: S->b = S->a; break;

	case 0x48: S->c = S->b; break;
	case 0x49: break;
	case 0x4a: S->c = S->d; break;
	case 0x4b: S->c = S->e; break;
	case 0x4c: S->c = S->h; break;
	case 0x4d: S->c = S->l; break;
	case 0x4e: S->c = read_mem(cpu, HL); break;
	case 0x4f: S->c = S->a; break;

	case 0x50: S->d = S->b; break;
	case 0x51: S->d = S->c; break;
	case 0x52: break;
	case 0x53: S->d = S->e; break;
	case 0x54: S->d = S->h; break;
	case 0x55: S->d = S->l; break;
	case 0x56: S->d = read_mem(cpu, HL); break;
	case 0x57: S->d = S->a; break;

	case 0x58: S->e = S->b; break;
	case 0x59: S->e = S->c; break;
	case 0x5a: S->e = S->d; break;
	case 0x5b: break;
	case 0x5c: S->e = S->h; break;
	case 0x5d: S->e = S->l; break;
	case 0x5e: S->e = read_mem(cpu, HL); break;
	case 0x5f: S->e = S->a; break;

	case 0x60: S->h = S->b; break;
	case 0x61: S->h = S->c; break;
	case 0x62: S->h = S->d; break;
	case 0x63: S->h = S->e; break;
	case 0x64: break;
	case 0x65: S->h = S->l; break;
	case 0x66: S->h = read_mem(cpu, HL); break;
	case 0x67: S->h = S->a; break;

	case 0x68: S->l = S->b; break;
	case 0x69: S->l = S->c; break;
	case 0x6a: S->l = S->d; break;
	case 0x6b: S->l = S->e; break;
	case 0x6c: S->l = S->h; break;
	case 0x6d: break;
	case 0x6e: S->l = read_mem(cpu, HL); break;
	case 0x6f: S->l = S->a; break;

	case 0x70: write_mem(cpu, HL, S->b); break;
	case 0x71: write_mem(cpu, HL, S->c); break;
	case 0x72: write_mem(cpu, HL, S->d); break;
	case 0x73: write_mem(cpu, HL, S->e); break;
	case 0x74: write_mem(cpu, HL, S->h); break;
	case 0x75: write_mem(cpu, HL, S->l); break;
	case 0x76:                                          // HLT (instead of MOV M,M)
		S->pc--;
		S->halt = 1;
		set_status(0x8a);
		break;
	case 0x77: write_mem(cpu, HL, S->a); break;

	case 0x78: S->a = S->b; break;
	case 0x79: S->a = S->c; break;
	case 0x7a: S->a = S->d; break;
	case 0x7b: S->a = S->e; break;
	case 0x7c: S->a = S->h; break;
	case 0x7d: S->a = S->l; break;
	case 0x7e: S->a = read_mem(cpu, HL); break;
	case 0x7f: break;

	// alu op [B/C/D/E/H/L/M/A]
	case 0x80: op_add(cpu, S->b); break;
	case 0x81: op_add(cpu, S->c); break;
	case 0x82: op_add(cpu, S->d); break;
	case 0x83: op_add(cpu, S->e); break;
	case 0x84: op_add(cpu, S->h); break;
	case 0x85: op_add(cpu, S->l); break;
	case 0x86: op_add(cpu, read_mem(cpu, HL)); break;
	case 0x87: op_add(cpu, S->a); break;

	case 0x88: op_adc(cpu, S->b); break;
	case 0x89: op_adc(cpu, S->c); break;
	case 0x8a: op_adc(cpu, S->d); break;
	case 0x8b: op_adc(cpu, S->e); break;
	case 0x8c: op_adc(cpu, S->h); break;
	case 0x8d: op_adc(cpu, S->l); break;
	case 0x8e: op_adc(cpu, read_mem(cpu, HL)); break;
	case 0x8f: op_adc(cpu, S->a); break;

	case 0x90: op_sub(cpu, S->b); break;
	case 0x91: op_sub(cpu, S->c); break;
	case 0x92: op_sub(cpu, S->d); break;
	case 0x93: op_sub(cpu, S->e); break;
	case 0x94: op_sub(cpu, S->h); break;
	case 0x95: op_sub(cpu, S->l); break;
	case 0x96: op_sub(cpu, read_mem(cpu, HL)); break;
	case 0x97: op_sub(cpu, S->a); break;

	case 0x98: op_sbb(cpu, S->b); break;
	case 0x99: op_sbb(cpu, S->c); break;
	case 0x9a: op_sbb(cpu, S->d); break;
	case 0x9b: op_sbb(cpu, S->e); break;
	case 0x9c: op_sbb(cpu, S->h); break;
	case 0x9d: op_sbb(cpu, S->l); break;
	case 0x9e: op_sbb(cpu, read_mem(cpu, HL)); break;
	case 0x9f: op_sbb(cpu, S->a); break;

	case 0xa0: op_ana(cpu, S->b); break;
	case 0xa1: op_ana(cpu, S->c); break;
	case 0xa2: op_ana(cpu, S->d); break;
	case 0xa3: op_ana(cpu, S->e); break;
	case 0xa4: op_ana(cpu, S->h); break;
	case 0xa5: op_ana(cpu, S->l); break;
	case 0xa6: op_ana(cpu, read_mem(cpu, HL)); break;
	case 0xa7: op_ana(cpu, S->a); break;

	case 0xa8: op_xra(cpu, S->b); break;
	case 0xa9: op_xra(cpu, S->c); break;
	case 0xaa: op_xra(cpu, S->d); break;
	case 0xab: op_xra(cpu, S->e); break;
	case 0xac: op_xra(cpu, S->h); break;
	case 0xad: op_xra(cpu, S->l); break;
	case 0xae: op_xra(cpu, read_mem(cpu, HL)); break;
	case 0xaf: op_xra(cpu, S->a); break;

	case 0xb0: op_ora(cpu, S->b); break;
	case 0xb1: op_ora(cpu, S->c); break;
	case 0xb2: op_ora(cpu, S->d); break;
	case 0xb3: op_ora(cpu, S->e); break;
	case 0xb4: op_ora(cpu, S->h); break;
	case 0xb5: op_ora(cpu, S->l); break;
	case 0xb6: op_ora(cpu, read_mem(cpu, HL)); break;
	case 0xb7: op_ora(cpu, S->a); break;

	case 0xb8: op_cmp(cpu, S->b); break;
	case 0xb9: op_cmp(cpu, S->c); break;
	case 0xba: op_cmp(cpu, S->d); break;
	case 0xbb: op_cmp(cpu, S->e); break;
	case 0xbc: op_cmp(cpu, S->h); break;
	case 0xbd: op_cmp(cpu, S->l); break;
	case 0xbe: op_cmp(cpu, read_mem(cpu, HL)); break;
	case 0xbf: op_cmp(cpu, S->a); break;

	case 0xc0: op_ret(cpu, !(S->f & ZF)); break;        // RNZ
	case 0xc1: SET_BC(op_pop(cpu)); break;              // POP B
	case 0xc2: op_jmp(cpu, !(S->f & ZF)); break;        // JNZ nnnn
	case 0xc3: op_jmp(cpu, 1); break;                   // JMP nnnn
	case 0xc4: op_call(cpu, !(S->f & ZF)); break;       // CNZ nnnn
	case 0xc5: op_push(cpu, BC); break;                 // PUSH B
	case 0xc6: op_add(cpu, read_arg(cpu)); break;       // ADI nn
	case 0xc7: op_rst(cpu, 0); break;                   // RST 0

	case 0xc8: op_ret(cpu, S->f & ZF); break;           // RZ
	case 0xc9: S->pc = op_pop(cpu); break;              // RET
	case 0xca: op_jmp(cpu, S->f & ZF); break;           // JZ nnnn
	case 0xcb:   // 8085: undocumented RSTV, otherwise undocumented JMP nnnn
		if (is85) {
			if (S->f & VF) {
				S->icount -= ret_taken(cpu);
				op_rst(cpu, 8);
			}
		} else
			op_jmp(cpu, 1);
		break;
	case 0xcc: op_call(cpu, S->f & ZF); break;          // CZ nnnn
	case 0xcd: op_call(cpu, 1); break;                  // CALL nnnn
	case 0xce: op_adc(cpu, read_arg(cpu)); break;       // ACI nn
	case 0xcf: op_rst(cpu, 1); break;                   // RST 1

	case 0xd0: op_ret(cpu, !(S->f & CF)); break;        // RNC
	case 0xd1: SET_DE(op_pop(cpu)); break;              // POP D
	case 0xd2: op_jmp(cpu, !(S->f & CF)); break;        // JNC nnnn
	case 0xd3: {                                        // OUT nn
		set_status(0x10);
		uint8_t port = read_arg(cpu);
		S->wz = port;
		cpu->out(cpu->user, port, S->a);
		break;
	}
	case 0xd4: op_call(cpu, !(S->f & CF)); break;       // CNC nnnn
	case 0xd5: op_push(cpu, DE); break;                 // PUSH D
	case 0xd6: op_sub(cpu, read_arg(cpu)); break;       // SUI nn
	case 0xd7: op_rst(cpu, 2); break;                   // RST 2

	case 0xd8: op_ret(cpu, S->f & CF); break;           // RC
	case 0xd9:   // 8085: undocumented SHLX, otherwise undocumented RET
		if (is85) {
			S->wz = DE;
			write_mem(cpu, S->wz, S->l);
			S->wz++;
			write_mem(cpu, S->wz, S->h);
		} else
			S->pc = op_pop(cpu);
		break;
	case 0xda: op_jmp(cpu, S->f & CF); break;           // JC nnnn
	case 0xdb: {                                        // IN nn
		set_status(0x42);
		uint8_t port = read_arg(cpu);
		S->wz = port;
		S->a = cpu->in(cpu->user, port);
		break;
	}
	case 0xdc: op_call(cpu, S->f & CF); break;          // CC nnnn
	case 0xdd:   // 8085: undocumented JNX5 nnnn, otherwise undocumented CALL nnnn
		if (is85)
			op_jmp(cpu, !(S->f & KF));
		else
			op_call(cpu, 1);
		break;
	case 0xde: op_sbb(cpu, read_arg(cpu)); break;       // SBI nn
	case 0xdf: op_rst(cpu, 3); break;                   // RST 3

	case 0xe0: op_ret(cpu, !(S->f & PF)); break;        // RPO
	case 0xe1: SET_HL(op_pop(cpu)); break;              // POP H
	case 0xe2: op_jmp(cpu, !(S->f & PF)); break;        // JPO nnnn
	case 0xe3:                                          // XTHL
		S->wz = op_pop(cpu);
		op_push(cpu, HL);
		SET_HL(S->wz);
		break;
	case 0xe4: op_call(cpu, !(S->f & PF)); break;       // CPO nnnn
	case 0xe5: op_push(cpu, HL); break;                 // PUSH H
	case 0xe6: op_ana(cpu, read_arg(cpu)); break;       // ANI nn
	case 0xe7: op_rst(cpu, 4); break;                   // RST 4

	case 0xe8: op_ret(cpu, S->f & PF); break;           // RPE
	case 0xe9: S->pc = HL; break;                       // PCHL
	case 0xea: op_jmp(cpu, S->f & PF); break;           // JPE nnnn
	case 0xeb: {                                        // XCHG
		uint8_t t = S->d; S->d = S->h; S->h = t;
		t = S->e; S->e = S->l; S->l = t;
		break;
	}
	case 0xec: op_call(cpu, S->f & PF); break;          // CPE nnnn
	case 0xed:   // 8085: undocumented LHLX, otherwise undocumented CALL nnnn
		if (is85) {
			S->wz = DE;
			S->l = read_mem(cpu, S->wz);
			S->wz++;
			S->h = read_mem(cpu, S->wz);
		} else
			op_call(cpu, 1);
		break;
	case 0xee: op_xra(cpu, read_arg(cpu)); break;       // XRI nn
	case 0xef: op_rst(cpu, 5); break;                   // RST 5

	case 0xf0: op_ret(cpu, !(S->f & SF)); break;        // RP
	case 0xf1: SET_AF(op_pop(cpu)); break;              // POP PSW
	case 0xf2: op_jmp(cpu, !(S->f & SF)); break;        // JP nnnn
	case 0xf3: set_inte(cpu, 0); break;                 // DI
	case 0xf4: op_call(cpu, !(S->f & SF)); break;       // CP nnnn
	case 0xf5:                                          // PUSH PSW
		// X3F is always 0, and on 8080, VF=1 and KF=0
		S->f &= ~X3F;
		if (!is85)
			S->f = (uint8_t)((S->f & ~KF) | VF);
		op_push(cpu, AF);
		break;
	case 0xf6: op_ora(cpu, read_arg(cpu)); break;       // ORI nn
	case 0xf7: op_rst(cpu, 6); break;                   // RST 6

	case 0xf8: op_ret(cpu, S->f & SF); break;           // RM
	case 0xf9: S->sp = HL; break;                       // SPHL
	case 0xfa: op_jmp(cpu, S->f & SF); break;           // JM nnnn
	case 0xfb:                                          // EI
		set_inte(cpu, 1);
		S->after_ei = 2;
		break;
	case 0xfc: op_call(cpu, S->f & SF); break;          // CM nnnn
	case 0xfd:   // 8085: undocumented JX5 nnnn, otherwise undocumented CALL nnnn
		if (is85)
			op_jmp(cpu, S->f & KF);
		else
			op_call(cpu, 1);
		break;
	case 0xfe: op_cmp(cpu, read_arg(cpu)); break;       // CPI nn
	case 0xff: op_rst(cpu, 7); break;                   // RST 7
	}
}
