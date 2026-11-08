/*  Texas Instruments TMS320C10 / C15 DSP, ported to C from MAME src/devices/cpu/tms320c1x/tms320c1x.cpp
    (BSD-3-Clause, copyright-holders: Tony La Porta). See tms320c1x.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "tms320c1x.h"

#include <string.h>

#define OV_FLAG   0x8000
#define OVM_FLAG  0x4000
#define INTM_FLAG 0x2000
#define ARP_REG   0x0100
#define DP_REG    0x0001
#define OV   (c->str & OV_FLAG)
#define OVM  (c->str & OVM_FLAG)
#define INTM (c->str & INTM_FLAG)
#define ARP  ((c->str & ARP_REG) >> 8)
#define DP   ((c->str & DP_REG) << 7)
#define OPL  ((uint8_t)c->opcode)
#define OPH  ((uint8_t)(c->opcode >> 8))
#define DMA_DP  (DP | (OPL & 0x7f))
#define DMA_DP1 (0x80 | OPL)
#define IND     (c->ar[ARP] & 0xff)
#define ACC_L ((uint16_t)c->acc)
#define ACC_H ((uint16_t)(c->acc >> 16))
#define SET_ACC_L(v) (c->acc = (c->acc & 0xffff0000u) | (uint16_t)(v))
#define SET_ACC_H(v) (c->acc = (c->acc & 0x0000ffffu) | ((uint32_t)(uint16_t)(v) << 16))
#define INT_PENDING 0x80000000

static inline uint16_t rd_rom(tms320c1x_t *c, uint16_t a) { return c->rom[a & c->rom_mask]; }
static inline uint16_t rd_ram(tms320c1x_t *c, uint16_t a) { return c->ram[a & c->ram_mask]; }
static inline void wr_ram(tms320c1x_t *c, uint16_t a, uint16_t v) { c->ram[a & c->ram_mask] = v; }

static inline void clr_flag(tms320c1x_t *c, uint16_t f) { c->str &= (uint16_t)~f; c->str |= 0x1efe; }
static inline void set_flag(tms320c1x_t *c, uint16_t f) { c->str |= f; c->str |= 0x1efe; }

static inline void add_overflow(tms320c1x_t *c, int32_t addval)
{
	if ((int32_t)(~(c->oldacc ^ (uint32_t)addval) & (c->oldacc ^ c->acc)) < 0) {
		set_flag(c, OV_FLAG);
		if (OVM)
			c->acc = ((int32_t)c->oldacc < 0) ? 0x80000000u : 0x7fffffffu;
	}
}
static inline void sub_overflow(tms320c1x_t *c, int32_t subval)
{
	if ((int32_t)((c->oldacc ^ (uint32_t)subval) & (c->oldacc ^ c->acc)) < 0) {
		set_flag(c, OV_FLAG);
		if (OVM)
			c->acc = ((int32_t)c->oldacc < 0) ? 0x80000000u : 0x7fffffffu;
	}
}
static inline uint16_t pop_stack(tms320c1x_t *c)
{
	const uint16_t d = c->stack[3];
	c->stack[3] = c->stack[2];
	c->stack[2] = c->stack[1];
	c->stack[1] = c->stack[0];
	return d & c->rom_mask;
}
static inline void push_stack(tms320c1x_t *c, uint16_t d)
{
	c->stack[0] = c->stack[1];
	c->stack[1] = c->stack[2];
	c->stack[2] = c->stack[3];
	c->stack[3] = d & c->rom_mask;
}
static inline void update_ar(tms320c1x_t *c)
{
	if (OPL & 0x30) {
		uint16_t t = c->ar[ARP];
		if (OPL & 0x20) t++;
		if (OPL & 0x10) t--;
		c->ar[ARP] = (uint16_t)((c->ar[ARP] & 0xfe00) | (t & 0x01ff));
	}
}
static inline void update_arp(tms320c1x_t *c)
{
	if (~OPL & 0x08) {
		if (OPL & 0x01) set_flag(c, ARP_REG);
		else clr_flag(c, ARP_REG);
	}
}
static inline void getdata(tms320c1x_t *c, int shift, int signext)
{
	c->memaccess = (OPL & 0x80) ? IND : DMA_DP;
	c->alu = rd_ram(c, c->memaccess);
	if (signext) c->alu = (uint32_t)(int32_t)(int16_t)c->alu;
	c->alu <<= shift;
	if (OPL & 0x80) {
		update_ar(c);
		update_arp(c);
	}
}
static inline void putdata(tms320c1x_t *c, uint16_t d)
{
	c->memaccess = (OPL & 0x80) ? IND : DMA_DP;
	if (OPL & 0x80) {
		update_ar(c);
		update_arp(c);
	}
	wr_ram(c, c->memaccess, d);
}
static inline void putdata_sar(tms320c1x_t *c, int r)
{
	c->memaccess = (OPL & 0x80) ? IND : DMA_DP;
	if (OPL & 0x80) {
		update_ar(c);
		update_arp(c);
	}
	wr_ram(c, c->memaccess, c->ar[r]);
}
static inline void putdata_sst(tms320c1x_t *c, uint16_t d)
{
	c->memaccess = (OPL & 0x80) ? IND : DMA_DP1;   // (page 1 only)
	if (OPL & 0x80)
		update_ar(c);
	wr_ram(c, c->memaccess, d);
}

// the cycles of the main opcodes (MAME s_opcode_main) and of the 7Fxx ones (s_opcode_7F)
static const uint8_t s_cyc_main[256] = {
	1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,   // add, sub
	1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,0,0,0,0,0,0, 1,1,0,0,0,0,0,0,   // lac, sar, lar
	2,2,2,2,2,2,2,2, 2,2,2,2,2,2,2,2, 1,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,   // in, out, sacl, sach
	1,1,1,1,1,1,1,3, 1,1,1,1,1,1,1,1, 1,1,0,0,0,0,0,0, 1,1,1,1,1,3,1,0,   // 60-7f
	1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,   // mpyk
	0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,1,1,1,0, 2,2,1,1,1,1,1,1    // f4 banz .. ff bz
};
static const uint8_t s_cyc_7f[32] = {
	1,1,1,0,0,0,0,0, 1,1,1,1,2,2,1,1, 1,0,0,0,0,0,0,0, 0,0,0,0,2,2,0,0
};

#define BRANCH(cond) do { if (cond) { c->pc = rd_rom(c, c->pc); c->icount -= s_cyc_main[OPH]; } else c->pc++; } while (0)

static inline __attribute__((always_inline)) void exec_7f(tms320c1x_t *c)
{
	switch (OPL & 0x1f) {
	case 0x00: break;                                   // nop
	case 0x01: set_flag(c, INTM_FLAG); break;           // dint
	case 0x02: clr_flag(c, INTM_FLAG); break;           // eint
	case 0x08:                                          // abst
		if ((int32_t)c->acc < 0) {
			c->acc = (uint32_t)-(int32_t)c->acc;
			if (OVM && c->acc == 0x80000000u) c->acc--;
		}
		break;
	case 0x09: c->acc = 0; break;                       // zac
	case 0x0a: clr_flag(c, OVM_FLAG); break;            // rovm
	case 0x0b: set_flag(c, OVM_FLAG); break;            // sovm
	case 0x0c: push_stack(c, c->pc); c->pc = ACC_L & c->rom_mask; break;   // cala
	case 0x0d: c->pc = pop_stack(c); break;             // ret
	case 0x0e: c->acc = c->preg; break;                 // pac
	case 0x0f: c->oldacc = c->acc; c->acc += c->preg; add_overflow(c, (int32_t)c->preg); break;   // apac
	case 0x10: c->oldacc = c->acc; c->acc -= c->preg; sub_overflow(c, (int32_t)c->preg); break;   // spac
	case 0x1c: push_stack(c, ACC_L); break;             // push
	case 0x1d: c->acc = pop_stack(c); break;            // pop (high word 0)
	default: break;                                     // (illegal)
	}
}

// (one switch on the high byte: a jump table)
static inline __attribute__((always_inline)) void exec_main(tms320c1x_t *c)
{
	const uint8_t h = OPH;
	switch (h) {
	case 0x00 ... 0x0f:   // add_sh
		c->oldacc = c->acc;
		getdata(c, h & 0xf, 1);
		c->acc += c->alu;
		add_overflow(c, (int32_t)c->alu);
		break;
	case 0x10 ... 0x1f:   // sub_sh
		c->oldacc = c->acc;
		getdata(c, h & 0xf, 1);
		c->acc -= c->alu;
		sub_overflow(c, (int32_t)c->alu);
		break;
	case 0x20 ... 0x2f:   // lac_sh
		getdata(c, h & 0xf, 1);
		c->acc = c->alu;
		break;
	case 0x40 ... 0x47:   // in_p
		c->alu = (c->alu & 0xffff0000u) | (c->in ? c->in(c->user, h & 7) : 0);
		putdata(c, (uint16_t)c->alu);
		break;
	case 0x48 ... 0x4f:   // out_p
		getdata(c, 0, 0);
		if (c->out) c->out(c->user, h & 7, (uint16_t)c->alu);
		break;
	case 0x58 ... 0x5f:   // sach_sh
		c->alu = c->acc << (h & 7);
		putdata(c, (uint16_t)(c->alu >> 16));
		break;
	case 0x80 ... 0x9f:   // mpyk
		c->preg = (uint32_t)((int32_t)(int16_t)c->treg * (int32_t)((int16_t)(c->opcode << 3) >> 3));
		break;
	case 0x30: putdata_sar(c, 0); break;
	case 0x31: putdata_sar(c, 1); break;
	case 0x38: getdata(c, 0, 0); c->ar[0] = (uint16_t)c->alu; break;
	case 0x39: getdata(c, 0, 0); c->ar[1] = (uint16_t)c->alu; break;
	case 0x50: putdata(c, ACC_L); break;   // sacl
	case 0x60: {   // addh
		c->oldacc = c->acc;
		getdata(c, 0, 0);
		SET_ACC_H(ACC_H + (uint16_t)c->alu);
		const uint16_t oh = (uint16_t)(c->oldacc >> 16), ah = (uint16_t)(c->alu >> 16);
		if ((int16_t)(~(oh ^ ah) & (oh ^ ACC_H)) < 0) {
			set_flag(c, OV_FLAG);
			if (OVM)
				SET_ACC_H(((int16_t)oh < 0) ? 0x8000 : 0x7fff);
		}
		break;
	}
	case 0x61: c->oldacc = c->acc; getdata(c, 0, 0); c->acc += c->alu; add_overflow(c, (int32_t)c->alu); break;   // adds
	case 0x62: c->oldacc = c->acc; getdata(c, 16, 0); c->acc -= c->alu; sub_overflow(c, (int32_t)c->alu); break;  // subh
	case 0x63: c->oldacc = c->acc; getdata(c, 0, 0); c->acc -= c->alu; sub_overflow(c, (int32_t)c->alu); break;   // subs
	case 0x64:   // subc
		c->oldacc = c->acc;
		getdata(c, 15, 0);
		c->alu = (uint32_t)((int32_t)c->acc - (int32_t)c->alu);
		if ((int32_t)((c->oldacc ^ c->alu) & (c->oldacc ^ c->acc)) < 0)
			set_flag(c, OV_FLAG);
		if ((int32_t)c->alu >= 0)
			c->acc = (c->alu << 1) + 1;
		else
			c->acc = c->acc << 1;
		break;
	case 0x65: getdata(c, 0, 0); c->acc = (uint32_t)(uint16_t)c->alu << 16; break;   // zalh
	case 0x66: getdata(c, 0, 0); c->acc = (uint16_t)c->alu; break;                    // zals
	case 0x67:   // tblr
		c->alu = rd_rom(c, ACC_L & c->rom_mask);
		putdata(c, (uint16_t)c->alu);
		c->stack[0] = c->stack[1];
		break;
	case 0x68:   // larp_mar
		if (OPL & 0x80) {
			update_ar(c);
			update_arp(c);
		}
		break;
	case 0x69: getdata(c, 0, 0); wr_ram(c, (uint16_t)(c->memaccess + 1), (uint16_t)c->alu); break;   // dmov
	case 0x6a: getdata(c, 0, 0); c->treg = (uint16_t)c->alu; break;   // lt
	case 0x6b:   // ltd
		c->oldacc = c->acc;
		getdata(c, 0, 0);
		c->treg = (uint16_t)c->alu;
		wr_ram(c, (uint16_t)(c->memaccess + 1), (uint16_t)c->alu);
		c->acc += c->preg;
		add_overflow(c, (int32_t)c->preg);
		break;
	case 0x6c:   // lta
		c->oldacc = c->acc;
		getdata(c, 0, 0);
		c->treg = (uint16_t)c->alu;
		c->acc += c->preg;
		add_overflow(c, (int32_t)c->preg);
		break;
	case 0x6d:   // mpy
		getdata(c, 0, 0);
		c->preg = (uint32_t)((int32_t)(int16_t)c->alu * (int32_t)(int16_t)c->treg);
		if (c->preg == 0x40000000u) c->preg = 0xc0000000u;
		break;
	case 0x6e: if (OPL & 1) set_flag(c, DP_REG); else clr_flag(c, DP_REG); break;   // ldpk
	case 0x6f: getdata(c, 0, 0); if (c->alu & 1) set_flag(c, DP_REG); else clr_flag(c, DP_REG); break;   // ldp
	case 0x70: c->ar[0] = OPL; break;   // lark
	case 0x71: c->ar[1] = OPL; break;
	case 0x78: getdata(c, 0, 0); SET_ACC_L(ACC_L ^ (uint16_t)c->alu); break;   // xor
	case 0x79: getdata(c, 0, 0); c->acc &= c->alu; break;                      // and
	case 0x7a: getdata(c, 0, 0); SET_ACC_L(ACC_L | (uint16_t)c->alu); break;   // or
	case 0x7b:   // lst
		if (OPL & 0x80)
			c->opcode |= 0x08;   // (indirect: the next ARP not supported here)
		getdata(c, 0, 0);
		c->alu &= (uint32_t)~INTM_FLAG;
		c->str &= INTM_FLAG;
		c->str |= (uint16_t)c->alu;
		c->str |= 0x1efe;
		break;
	case 0x7c: putdata_sst(c, c->str); break;   // sst
	case 0x7d:   // tblw (the program ROM: not written)
		getdata(c, 0, 0);
		c->stack[0] = c->stack[1];
		break;
	case 0x7e: c->acc = OPL; break;   // lack
	case 0xf4: {   // banz
		BRANCH(c->ar[ARP] & 0x01ff);
		const uint16_t t = (uint16_t)(c->ar[ARP] - 1);
		c->alu = (c->alu & 0xffff0000u) | t;
		c->ar[ARP] = (uint16_t)((c->ar[ARP] & 0xfe00) | (t & 0x01ff));
		break;
	}
	case 0xf5:   // bv
		if (OV) {
			clr_flag(c, OV_FLAG);
			c->pc = rd_rom(c, c->pc);
			c->icount -= s_cyc_main[h];
		} else
			c->pc++;
		break;
	case 0xf6: BRANCH(c->bio && c->bio(c->user)); break;   // bioz
	case 0xf8:   // call
		c->pc++;
		push_stack(c, c->pc);
		c->pc = rd_rom(c, (uint16_t)(c->pc - 1));
		break;
	case 0xf9: c->pc = rd_rom(c, c->pc); break;   // b
	case 0xfa: BRANCH((int32_t)c->acc < 0); break;    // blz
	case 0xfb: BRANCH((int32_t)c->acc <= 0); break;   // blez
	case 0xfc: BRANCH((int32_t)c->acc > 0); break;    // bgz
	case 0xfd: BRANCH((int32_t)c->acc >= 0); break;   // bgez
	case 0xfe: BRANCH(c->acc != 0); break;            // bnz
	case 0xff: BRANCH(c->acc == 0); break;            // bz
	default: break;                                   // (illegal)
	}
}

void tms320c1x_init(tms320c1x_t *c)
{
	memset(c, 0, sizeof(*c));
	c->ram_mask = 0xff;
	c->rom_mask = 0xfff;
}

void tms320c1x_reset(tms320c1x_t *c)
{
	c->pc = 0;
	c->acc = 0;
	c->intf = 0;
	clr_flag(c, OV_FLAG | ARP_REG | DP_REG);
	set_flag(c, OVM_FLAG | INTM_FLAG);
}

void tms320c1x_set_irq(tms320c1x_t *c, int state)
{
	if (state)
		c->intf |= INT_PENDING;
}

int tms320c1x_execute(tms320c1x_t *c, int cycles)
{
	c->icount = cycles;
	do {
		if (c->intf) {   // (not after MPY, MPYK or EINT)
			if (OPH != 0x6d && (OPH & 0xe0) != 0x80 && c->opcode != 0x7f82 && !INTM) {
				c->intf = 0;
				set_flag(c, INTM_FLAG);
				push_stack(c, c->pc);
				c->pc = 0x0002;
				c->icount -= s_cyc_7f[0x1c] + s_cyc_7f[0x01];
			}
		}
		c->prevpc = c->pc;
		c->opcode = rd_rom(c, c->pc);
		c->pc++;
		if (OPH != 0x7f) {
			c->icount -= s_cyc_main[OPH];
			exec_main(c);
		} else {
			c->icount -= s_cyc_7f[OPL & 0x1f];
			exec_7f(c);
		}
	} while (c->icount > 0);
	const int used = cycles - c->icount;
	c->cycles += used;
	return used;
}
