/*  Microchip PIC16C5x (and GI PIC1650 / 1654S / 1655), ported to C from MAME src/devices/cpu/pic16c5x/pic16c5x.cpp
    (BSD-3-Clause, copyright-holders: Tony La Porta; Ver 1.17). The opcode table is a switch with the same handlers
    and cycle counts; the data space maps are the address_map functions (ram_5_2ports ... ram_7_3ports) as code.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "pic16c5x.h"

#include <stdlib.h>
#include <string.h>

// STATUS: | PA (3) | TO | PD | Z | DC | C |
#define PA_REG    0xe0
#define TO_FLAG   0x10
#define PD_FLAG   0x08
#define Z_FLAG    0x04
#define DC_FLAG   0x02
#define C_FLAG    0x01
// OPTION: | 0 | 0 | T0CS | T0SE | PSA | PS (3) |
#define T0CS_FLAG 0x20
#define T0SE_FLAG 0x10
#define PSA_FLAG  0x08
#define PS_REG    0x07
// CONFIG: | CP | WDTE | FOSC |
#define WDTE_FLAG 0x04

#define PD      (p->STATUS & PD_FLAG)
#define T0CS    (p->OPTION & T0CS_FLAG)
#define T0SE    (p->OPTION & T0SE_FLAG)
#define PSA     (p->OPTION & PSA_FLAG)
#define PS      (p->OPTION & PS_REG)
#define WDTE    (p->CONFIG & WDTE_FLAG)

#define OPL     ((uint8_t)p->opcode)
#define ADDR    (OPL & 0x1f)
#define BITPOS  ((OPL >> 5) & 7)
#define SETF(f) (p->STATUS |= (f))
#define CLRF(f) (p->STATUS &= (uint8_t)~(f))

enum { PORTA = 0, PORTB, PORTC, PORTD };

static int is_nmos(const pic16c5x_t *p) { return p->picmodel == PIC1650 || p->picmodel == PIC1654S || p->picmodel == PIC1655; }

static uint8_t rd_port(pic16c5x_t *p, int port) { return p->read_port ? p->read_port(p->ctx, port) : 0; }
static void wr_port(pic16c5x_t *p, int port, uint8_t data, uint8_t mask)
{
	if (p->write_port)
		p->write_port(p->ctx, port, data, mask);
}

static void calc_zero_flag(pic16c5x_t *p)
{
	if (p->ALU == 0) SETF(Z_FLAG); else CLRF(Z_FLAG);
}
static void calc_add_flags(pic16c5x_t *p, uint8_t augend)
{
	calc_zero_flag(p);
	if (augend > p->ALU) SETF(C_FLAG); else CLRF(C_FLAG);
	if ((augend & 0x0f) > (p->ALU & 0x0f)) SETF(DC_FLAG); else CLRF(DC_FLAG);
}
static void calc_sub_flags(pic16c5x_t *p, uint8_t minuend)
{
	calc_zero_flag(p);
	if (minuend < p->ALU) CLRF(C_FLAG); else SETF(C_FLAG);
	if ((minuend & 0x0f) < (p->ALU & 0x0f)) CLRF(DC_FLAG); else SETF(DC_FLAG);
}

static void set_pc(pic16c5x_t *p, uint16_t addr) { p->PC = addr & p->program_mask; }
static uint16_t pop_stack(pic16c5x_t *p)
{
	uint16_t data = p->STACK[1];
	p->STACK[1] = p->STACK[0];
	return data & p->program_mask;
}
static void push_stack(pic16c5x_t *p, uint16_t data)
{
	p->STACK[0] = p->STACK[1];
	p->STACK[1] = data & p->program_mask;
}

// ---- the special registers ----
static uint8_t porta_r(pic16c5x_t *p)
{
	if (p->picmodel == PIC1650 || p->picmodel == PIC1654S)
		return rd_port(p, PORTA) & p->port_data[PORTA];
	if (p->picmodel == PIC1655)
		return rd_port(p, PORTA) & 0x0f;
	uint8_t data = rd_port(p, PORTA);
	data &= p->port_tris[PORTA];
	data |= (uint8_t)~p->port_tris[PORTA] & p->port_data[PORTA];
	return data & 0x0f;
}
static void porta_w(pic16c5x_t *p, uint8_t data)
{
	if (p->picmodel == PIC1650 || p->picmodel == PIC1654S)
		wr_port(p, PORTA, data, 0xff);
	else {
		data &= 0x0f;
		wr_port(p, PORTA, data & (uint8_t)~p->port_tris[PORTA] & 0x0f, (uint8_t)~p->port_tris[PORTA] & 0x0f);
	}
	p->port_data[PORTA] = data;
}
static uint8_t portb_r(pic16c5x_t *p)
{
	if (p->picmodel == PIC1650 || p->picmodel == PIC1654S)
		return rd_port(p, PORTB) & p->port_data[PORTB];
	if (p->picmodel != PIC1655) {
		uint8_t data = rd_port(p, PORTB);
		data &= p->port_tris[PORTB];
		data |= (uint8_t)~p->port_tris[PORTB] & p->port_data[PORTB];
		return data;
	}
	return p->port_data[PORTB];
}
static void portb_w(pic16c5x_t *p, uint8_t data)
{
	if (is_nmos(p))
		wr_port(p, PORTB, data, 0xff);
	else
		wr_port(p, PORTB, data & (uint8_t)~p->port_tris[PORTB], (uint8_t)~p->port_tris[PORTB]);
	p->port_data[PORTB] = data;
}
static uint8_t portc_r(pic16c5x_t *p)
{
	if (p->picmodel == PIC1650 || p->picmodel == PIC1655)
		return rd_port(p, PORTC) & p->port_data[PORTC];
	uint8_t data = rd_port(p, PORTC);
	data &= p->port_tris[PORTC];
	data |= (uint8_t)~p->port_tris[PORTC] & p->port_data[PORTC];
	return data;
}
static void portc_w(pic16c5x_t *p, uint8_t data)
{
	if (p->picmodel == PIC1650 || p->picmodel == PIC1655)
		wr_port(p, PORTC, data, 0xff);
	else
		wr_port(p, PORTC, data & (uint8_t)~p->port_tris[PORTC], (uint8_t)~p->port_tris[PORTC]);
	p->port_data[PORTC] = data;
}
static uint8_t portd_r(pic16c5x_t *p) { return rd_port(p, PORTD) & p->port_data[PORTD]; }
static void portd_w(pic16c5x_t *p, uint8_t data)
{
	wr_port(p, PORTD, data, 0xff);
	p->port_data[PORTD] = data;
}

// the number of ports in the model's data map (core registers 0-4, then the ports, then RAM)
static int nports(const pic16c5x_t *p)
{
	switch (p->picmodel) {
	case PIC16C55: case PIC16C57: case PIC1655: return 3;
	case PIC1650: return 4;
	default: return 2;
	}
}

// the data space: 7-bit maps mirror 0x00-0x0f at 0x20/0x40/0x60 (core_regs(map, 0x60) and the ports, RAM 0x07/0x08-0x0f)
static uint8_t data_read(pic16c5x_t *p, uint8_t addr)
{
	addr &= p->data_mask;
	if (p->data_width == 7 && !(addr & 0x10))
		addr &= 0x0f;
	switch (addr) {
	case 0: return 0;                                // noprw
	case 1: return p->TMR0;
	case 2: return p->PC & 0x0ff;
	case 3: return p->STATUS;
	case 4: return p->FSR | (uint8_t)~p->data_mask;
	default: break;
	}
	int np = nports(p);
	if (addr < 5 + np) {
		switch (addr - 5) {
		case 0: return porta_r(p);
		case 1: return portb_r(p);
		case 2: return portc_r(p);
		default: return portd_r(p);
		}
	}
	return p->ram[addr];
}

static void data_write(pic16c5x_t *p, uint8_t addr, uint8_t data)
{
	addr &= p->data_mask;
	if (p->data_width == 7 && !(addr & 0x10))
		addr &= 0x0f;
	switch (addr) {
	case 0: return;
	case 1:   // tmr0_w
		p->delay_timer = 2;   // Timer increment is inhibited for 2 cycles
		if (PSA == 0) p->prescaler = 0;
		p->TMR0 = data;
		return;
	case 2:   // pcl_w
		set_pc(p, (uint16_t)(((p->STATUS & PA_REG) << 4) | data));
		p->inst_cycles++;
		return;
	case 3:   // status_w
		if (is_nmos(p))
			p->STATUS = data | (uint8_t)~p->status_mask;
		else
			p->STATUS = (p->STATUS & (TO_FLAG | PD_FLAG)) | (data & (uint8_t)~(TO_FLAG | PD_FLAG));
		return;
	case 4: p->FSR = data & p->data_mask; return;
	default: break;
	}
	int np = nports(p);
	if (addr < 5 + np) {
		switch (addr - 5) {
		case 0: if (p->picmodel != PIC1655) porta_w(p, data); return;   // A is input-only on 1655
		case 1: portb_w(p, data); return;
		case 2: portc_w(p, data); return;
		default: portd_w(p, data); return;
		}
	}
	p->ram[addr] = data;
}

static uint8_t get_regfile(pic16c5x_t *p, uint8_t addr)
{
	if (addr == 0)
		addr = p->FSR;
	else if (p->data_width != 5)
		addr |= p->FSR & 0x60;   // FSR bits 6-5 are used for banking in direct mode
	return data_read(p, addr);
}
static void store_regfile(pic16c5x_t *p, uint8_t addr, uint8_t data)
{
	if (addr == 0)
		addr = p->FSR;
	else if (p->data_width != 5)
		addr |= p->FSR & 0x60;
	data_write(p, addr, data);
}
static void store_result(pic16c5x_t *p, uint8_t addr, uint8_t data)
{
	if (OPL & 0x20)
		store_regfile(p, addr, data);
	else
		p->W = data;
}

static void illegal(pic16c5x_t *p) { (void)p; }

static void tris(pic16c5x_t *p)
{
	switch (OPL & 0x7) {
	case 5:
		if (p->port_tris[PORTA] != p->W) {
			p->port_tris[PORTA] = p->W | 0xf0;
			wr_port(p, PORTA, p->port_data[PORTA] & (uint8_t)~p->port_tris[PORTA] & 0x0f, (uint8_t)~p->port_tris[PORTA] & 0x0f);
		}
		break;
	case 6:
		if (p->port_tris[PORTB] != p->W) {
			p->port_tris[PORTB] = p->W;
			wr_port(p, PORTB, p->port_data[PORTB] & (uint8_t)~p->port_tris[PORTB], (uint8_t)~p->port_tris[PORTB]);
		}
		break;
	case 7:
		if (p->picmodel == PIC16C55 || p->picmodel == PIC16C57) {
			if (p->port_tris[PORTC] != p->W) {
				p->port_tris[PORTC] = p->W;
				wr_port(p, PORTC, p->port_data[PORTC] & (uint8_t)~p->port_tris[PORTC], (uint8_t)~p->port_tris[PORTC]);
			}
		} else
			illegal(p);
		break;
	default:
		illegal(p);
		break;
	}
}

// the opcodes 0x000-0x00f (s_opcode_00x: all 1 cycle)
static void op_00x(pic16c5x_t *p)
{
	switch (OPL & 0x1f) {
	case 0: break;                                               // nop
	case 2: p->OPTION = p->W & (T0CS_FLAG | T0SE_FLAG | PSA_FLAG | PS_REG); break;   // option
	case 3:                                                      // sleep
		if (WDTE) p->WDT = 0;
		if (PSA) p->prescaler = 0;
		SETF(TO_FLAG);
		CLRF(PD_FLAG);
		break;
	case 4:                                                      // clrwdt
		p->WDT = 0;
		if (PSA) p->prescaler = 0;
		SETF(TO_FLAG);
		SETF(PD_FLAG);
		break;
	case 5: case 6: case 7: tris(p); break;
	default: illegal(p); break;
	}
}

// s_opcode_main[(opcode >> 4) & 0xff]
static void op_main(pic16c5x_t *p, int op)
{
	uint8_t a = ADDR, v;
	int c;
	switch (op >> 2) {
	case 0x00:   // 00-03: nop, illegal, movwf, movwf
		if (op == 0) break;
		if (op == 1) { illegal(p); break; }
		store_regfile(p, a, p->W);
		break;
	case 0x01:   // 04-07: clrw, illegal, clrf, clrf
		if (op == 4) { p->W = 0; SETF(Z_FLAG); break; }
		if (op == 5) { illegal(p); break; }
		store_regfile(p, a, 0);
		SETF(Z_FLAG);
		break;
	case 0x02:   // subwf
		v = get_regfile(p, a);
		p->ALU = v - p->W;
		store_result(p, a, p->ALU);
		calc_sub_flags(p, v);
		break;
	case 0x03:   // decf
		p->ALU = get_regfile(p, a) - 1;
		store_result(p, a, p->ALU);
		calc_zero_flag(p);
		break;
	case 0x04:   // iorwf
		p->ALU = get_regfile(p, a) | p->W;
		store_result(p, a, p->ALU);
		calc_zero_flag(p);
		break;
	case 0x05:   // andwf
		p->ALU = get_regfile(p, a) & p->W;
		store_result(p, a, p->ALU);
		calc_zero_flag(p);
		break;
	case 0x06:   // xorwf
		p->ALU = get_regfile(p, a) ^ p->W;
		store_result(p, a, p->ALU);
		calc_zero_flag(p);
		break;
	case 0x07:   // addwf
		v = get_regfile(p, a);
		p->ALU = v + p->W;
		store_result(p, a, p->ALU);
		calc_add_flags(p, v);
		break;
	case 0x08:   // movf
		p->ALU = get_regfile(p, a);
		store_result(p, a, p->ALU);
		calc_zero_flag(p);
		break;
	case 0x09:   // comf
		p->ALU = (uint8_t)~get_regfile(p, a);
		store_result(p, a, p->ALU);
		calc_zero_flag(p);
		break;
	case 0x0a:   // incf
		p->ALU = get_regfile(p, a) + 1;
		store_result(p, a, p->ALU);
		calc_zero_flag(p);
		break;
	case 0x0b:   // decfsz
		p->ALU = get_regfile(p, a) - 1;
		store_result(p, a, p->ALU);
		if (p->ALU == 0) { set_pc(p, p->PC + 1); p->inst_cycles++; }
		break;
	case 0x0c:   // rrf
		p->ALU = get_regfile(p, a);
		c = p->ALU & 1;
		p->ALU >>= 1;
		if (p->STATUS & C_FLAG) p->ALU |= 0x80;
		store_result(p, a, p->ALU);
		if (c) SETF(C_FLAG); else CLRF(C_FLAG);
		break;
	case 0x0d:   // rlf
		p->ALU = get_regfile(p, a);
		c = (p->ALU >> 7) & 1;
		p->ALU <<= 1;
		if (p->STATUS & C_FLAG) p->ALU |= 1;
		store_result(p, a, p->ALU);
		if (c) SETF(C_FLAG); else CLRF(C_FLAG);
		break;
	case 0x0e:   // swapf
		v = get_regfile(p, a);
		p->ALU = (uint8_t)(v << 4 | v >> 4);
		store_result(p, a, p->ALU);
		break;
	case 0x0f:   // incfsz
		p->ALU = get_regfile(p, a) + 1;
		store_result(p, a, p->ALU);
		if (p->ALU == 0) { set_pc(p, p->PC + 1); p->inst_cycles++; }
		break;
	default:
		switch (op >> 4) {
		case 0x4: case 0x5:   // bcf, bsf
			p->ALU = get_regfile(p, a);
			if (op < 0x50) p->ALU &= (uint8_t)~(1 << BITPOS);
			else p->ALU |= (uint8_t)(1 << BITPOS);
			store_regfile(p, a, p->ALU);
			break;
		case 0x6:             // btfsc
			if (!((get_regfile(p, a) >> BITPOS) & 1)) { set_pc(p, p->PC + 1); p->inst_cycles++; }
			break;
		case 0x7:             // btfss
			if ((get_regfile(p, a) >> BITPOS) & 1) { set_pc(p, p->PC + 1); p->inst_cycles++; }
			break;
		case 0x8:             // retlw
			p->W = OPL;
			set_pc(p, pop_stack(p));
			break;
		case 0x9:             // call
			push_stack(p, p->PC);
			set_pc(p, (uint16_t)((((p->STATUS & PA_REG) << 4) | OPL) & 0x6ff));
			break;
		case 0xa: case 0xb:   // goto
			set_pc(p, (uint16_t)(((p->STATUS & PA_REG) << 4) | (p->opcode & 0x1ff)));
			break;
		case 0xc:             // movlw
			p->W = OPL;
			break;
		case 0xd:             // iorlw
			p->ALU = OPL | p->W;
			p->W = p->ALU;
			calc_zero_flag(p);
			break;
		case 0xe:             // andlw
			p->ALU = OPL & p->W;
			p->W = p->ALU;
			calc_zero_flag(p);
			break;
		default:              // xorlw
			p->ALU = p->W ^ OPL;
			p->W = p->ALU;
			calc_zero_flag(p);
			break;
		}
		break;
	}
}

// ---- reset, watchdog, timer ----
static void reset_regs(pic16c5x_t *p)
{
	p->CONFIG = p->temp_config;
	p->port_tris[PORTA] = 0xff;
	p->port_tris[PORTB] = 0xff;
	p->port_tris[PORTC] = 0xff;
	p->OPTION = T0CS_FLAG | T0SE_FLAG | PSA_FLAG | PS_REG;
	set_pc(p, p->program_mask);
	p->PREVPC = p->PC;
	p->prescaler = 0;
	p->delay_timer = 0;
	p->inst_cycles = 0;
	p->count_cycles = 0;
}

static void watchdog_reset(pic16c5x_t *p)
{
	SETF(TO_FLAG | PD_FLAG | Z_FLAG | DC_FLAG | C_FLAG);
	reset_regs(p);
}

void pic16c5x_set_config(pic16c5x_t *p, uint16_t data) { p->temp_config = data; }

void pic16c5x_reset(pic16c5x_t *p)
{
	reset_regs(p);
	CLRF(PA_REG);
	SETF(TO_FLAG | PD_FLAG);
}

static void update_watchdog(pic16c5x_t *p, int counts)
{
	// 18,000 (0x464f) ticks, based on a 4 MHz input clock; not on CLRWDT or SLEEP
	if (p->opcode != 3 && p->opcode != 4) {
		uint16_t old_WDT = p->WDT;
		p->WDT -= counts;
		if (p->WDT > 0x464f)
			p->WDT = 0x464f - (0xffff - p->WDT);
		if ((old_WDT != 0 && old_WDT < p->WDT) || p->WDT == 0) {
			if (PSA) {
				p->prescaler++;
				if (p->prescaler >= (1 << PS)) {   // Prescale values from 1 to 128
					p->prescaler = 0;
					CLRF(TO_FLAG);
					watchdog_reset(p);
				}
			} else {
				CLRF(TO_FLAG);
				watchdog_reset(p);
			}
		}
	}
}

static void update_timer(pic16c5x_t *p, int counts)
{
	if (p->delay_timer > 0) {   // Timer increment is inhibited
		int dt = p->delay_timer;
		p->delay_timer -= p->inst_cycles;
		counts -= dt;
	}
	if (p->delay_timer > 0 || counts <= 0)
		return;
	if (PSA == 0) {
		p->prescaler += counts;
		if (p->prescaler >= (2 << PS)) {   // Prescale values from 2 to 256
			p->TMR0 += (uint8_t)(p->prescaler / (2 << PS));
			p->prescaler %= (2 << PS);
		}
	} else
		p->TMR0 += (uint8_t)counts;
}

void pic16c5x_set_input(pic16c5x_t *p, int state)
{
	if (T0CS && state != p->rtcc) {   // Count mode, edge triggered
		if ((T0SE && !state) || (!T0SE && state))
			p->count_cycles++;
	}
	p->rtcc = state;
}

static inline void step(pic16c5x_t *p)
{
		if (PD == 0) {   // Sleep Mode
			p->count_cycles = 0;
			p->inst_cycles = 1;
		} else {
			p->PREVPC = p->PC;
			uint32_t a = p->PC < p->rom_words ? p->PC : 0;
			p->opcode = p->rom ? (uint16_t)(p->rom[a * 2] | (p->rom[a * 2 + 1] << 8)) : 0;
			set_pc(p, p->PC + 1);
			if (is_nmos(p) || (p->opcode & 0xff0) != 0x000) {   // all opcodes except the 00? ones
				int op = (p->opcode >> 4) & 0xff;
				p->inst_cycles = (op >= 0x80 && op < 0xc0) ? 2 : 1;
				op_main(p, op);
			} else {
				p->inst_cycles = 1;
				op_00x(p);
			}
			update_timer(p, T0CS ? p->count_cycles : p->inst_cycles);
			p->count_cycles = 0;
		}
		if (WDTE)
			update_watchdog(p, p->inst_cycles);
		p->icount -= p->inst_cycles;
		p->total_cycles += p->inst_cycles;
}

/*  Polling loop skip (ReBOX, results identical to the interpreter; RBX_PIC_NOSPIN=1 = off). The security PIC of the
    Wolf unit waits for the host's clock bit in   s: CLRWDT;  s+1: BTFSS / BTFSC port,bit;  s+2: (skipped);  s+3: GOTO s.
    Within one pic16c5x_execute the port inputs do not change (the host does not run), so when one real iteration
    leaves the whole state as it found it but for the cycle counts and the timer, every following one does the same:
    the remaining whole iterations of the slice are counted off icount (the timer advanced by their cycles at once, as
    its per-instruction updates add up without a start-up delay); the last ones run normally, so the slice ends at the
    same instruction. */
static int s_spin_on = -1;
static int spin_loop(const pic16c5x_t *p, uint32_t s)
{
	if (s + 3 >= p->rom_words || PD == 0)
		return 0;
	const uint16_t o0 = (uint16_t)(p->rom[s * 2] | (p->rom[s * 2 + 1] << 8)) & 0xfff;
	const uint16_t o1 = (uint16_t)(p->rom[s * 2 + 2] | (p->rom[s * 2 + 3] << 8)) & 0xfff;
	if (o0 != 0x004 || (o1 & 0xc00) != 0x400 || (o1 & 0x300) < 0x200)   // CLRWDT; BTFSC (0x6xx) / BTFSS (0x7xx)
		return 0;
	const int f = o1 & 0x1f;
	return f >= 5 && f <= 7;   // (a port, read through the callbacks)
}

int pic16c5x_execute(pic16c5x_t *p, int cycles)
{
	int start;
	p->icount += cycles;
	start = p->icount;
	if (p->icount <= 0)
		return 0;
	if (s_spin_on < 0)
		s_spin_on = !getenv("RBX_PIC_NOSPIN");
	do {
		step(p);
		// (a jump back over 3 instructions with a long slice left: the loop above?)
		if (p->PC + 3 == p->PREVPC && (p->opcode & 0xe00) == 0xa00 && p->icount > 64 && s_spin_on && p->delay_timer <= 0 &&
			spin_loop(p, p->PC)) {
			const size_t n = pic16c5x_state_size();
			pic16c5x_t p0;
			memcpy(&p0, p, n);
			const uint32_t s = p->PC;
			for (int k = 0; k < 3 && p->PC != s + 3; k++)
				step(p);
			if (p->PC == s + 3)
				step(p);   // (the GOTO)
			const int c = p0.icount - p->icount;
			if (p->PC == s && p->PREVPC == s + 3 && c > 0 && p->delay_timer <= 0 && p->icount > c) {
				pic16c5x_t t;
				memcpy(&t, p, n);
				t.icount = p0.icount;
				t.total_cycles = p0.total_cycles;
				t.TMR0 = p0.TMR0;
				if (!PSA)
					t.prescaler = p0.prescaler;
				if (!memcmp(&t, &p0, n)) {
					const int m = (p->icount - 1) / c;   // (1..c cycles left to run)
					p->icount -= m * c;
					p->total_cycles += (uint64_t)m * (uint64_t)c;
					if (!T0CS)   // (internal clock: the timer counts every cycle)
						update_timer(p, m * c);
				}
			}
		}
	} while (p->icount > 0);
	return start - p->icount;
}

void pic16c5x_init(pic16c5x_t *p, int model, const uint8_t *rom, uint32_t rom_bytes, void *ctx)
{
	memset(p, 0, sizeof(*p));
	p->picmodel = model;
	p->program_width = (model == PIC16C57 || model == PIC16C58) ? 11 : model == PIC16C56 ? 10 : 9;
	p->data_width = (model == PIC16C57 || model == PIC16C58) ? 7 : 5;
	p->program_mask = (uint16_t)((1 << p->program_width) - 1);
	p->data_mask = (uint8_t)((1 << p->data_width) - 1);
	p->status_mask = is_nmos(p) ? 0x07 : 0xff;
	p->STATUS = (uint8_t)~p->status_mask;
	p->rom = rom;
	p->rom_words = rom_bytes / 2;
	if (p->rom_words > (uint32_t)p->program_mask + 1)
		p->rom_words = (uint32_t)p->program_mask + 1;
	p->ctx = ctx;
}

size_t pic16c5x_state_size(void) { return offsetof(pic16c5x_t, ram) + sizeof(((pic16c5x_t *)0)->ram); }
