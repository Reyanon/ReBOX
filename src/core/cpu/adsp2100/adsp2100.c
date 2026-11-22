/*  ADSP-21xx series emulator (the ADSP-2105 of the Midway DCS sound boards), ported to C from MAME
    src/devices/cpu/adsp2100/adsp2100.cpp, adsp2100.h, 2100ops.hxx (BSD-3-Clause, copyright-holders: Aaron Giles).
    The opcode and register code is MAME's (generated from it, member names kept); the memory accessors are page
    tables. ADSP-2105 only (ADSP_CHIP), its interrupt logic is MAME's adsp2101_device.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "adsp2100.h"
#include "../../rbx.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { CHIP_TYPE_ADSP2100, CHIP_TYPE_ADSP2101, CHIP_TYPE_ADSP2104, CHIP_TYPE_ADSP2105, CHIP_TYPE_ADSP2115,
       CHIP_TYPE_ADSP2181 };
#define ADSP_CHIP CHIP_TYPE_ADSP2105
#define ADSP_TRACK_HOTSPOTS 0
#define AINLINE inline __attribute__((always_inline))

enum { ADSP2101_IRQ0 = 0, ADSP2101_SPORT1_RX = 0, ADSP2101_IRQ1 = 1, ADSP2101_SPORT1_TX = 1, ADSP2101_IRQ2 = 2,
       ADSP2101_SPORT0_RX = 3, ADSP2101_SPORT0_TX = 4, ADSP2101_TIMER = 5 };
enum { ADSP2181_IRQ0 = 0, ADSP2181_SPORT1_RX = 0, ADSP2181_IRQ1 = 1, ADSP2181_SPORT1_TX = 1, ADSP2181_IRQ2 = 2,
       ADSP2181_SPORT0_RX = 3, ADSP2181_SPORT0_TX = 4, ADSP2181_TIMER = 5, ADSP2181_IRQE = 6, ADSP2181_BDMA = 7,
       ADSP2181_IRQL1 = 8, ADSP2181_IRQL0 = 9 };
enum { PC_STACK_DEPTH = 16, CNTR_STACK_DEPTH = 4, STAT_STACK_DEPTH = 4, LOOP_STACK_DEPTH = 4 };
enum { SSFLAG = 0x80, MVFLAG = 0x40, QFLAG = 0x20, SFLAG = 0x10, CFLAG = 0x08, VFLAG = 0x04, NFLAG = 0x02,
       ZFLAG = 0x01 };
#define CLEAR_LINE 0

#define BIT2(x, n) (((x) >> (n)) & 1)
#define BIT3(x, n, w) (((x) >> (n)) & ((1u << (w)) - 1))
#define BIT_SEL(_1, _2, _3, N, ...) N
#define BIT(...) BIT_SEL(__VA_ARGS__, BIT3, BIT2, 0)(__VA_ARGS__)
static inline int32_t sext(uint32_t v, int bits) { return (int32_t)(v << (32 - bits)) >> (32 - bits); }
static inline int clz32(uint32_t x) { return x ? __builtin_clz(x) : 32; }
static inline int clo32(uint32_t x) { return clz32(~x); }
#define logerror(...) ((void)0)

// little-endian host (PC, Vita)
typedef union { uint16_t u; int16_t s; } adsp_reg16;
typedef union { struct { adsp_reg16 sr0, sr1; } srx; uint32_t sr; } adsp_shift;
typedef union { struct { adsp_reg16 mr0, mr1, mr2, mrzero; } mrx; struct { uint32_t mr0, mr1; } mry; uint64_t mr; } adsp_mac;
typedef struct {
	adsp_reg16 ax0, ax1, ay0, ay1, ar, af;
	adsp_reg16 mx0, mx1, my0, my1;
	adsp_mac mr;
	adsp_reg16 mf, si, se, sb;
	adsp_shift sr;
	adsp_reg16 zero;
} adsp_core;

// the saved state (MAME's members)
static struct {
	uint32_t m_pc, m_ppc, m_loop, m_loop_condition, m_cntr;
	uint32_t m_astat, m_sstat, m_mstat, m_mstat_prev, m_astat_clear, m_idle;
	adsp_core m_core;
	uint32_t m_i[8];
	int32_t m_m[8];
	uint32_t m_l[8], m_lmask[8], m_base[8];
	uint8_t m_px;
	uint32_t m_pmovlay, m_dmovlay;
	uint32_t m_loop_stack[LOOP_STACK_DEPTH], m_cntr_stack[CNTR_STACK_DEPTH], m_pc_stack[PC_STACK_DEPTH];
	uint16_t m_stat_stack[STAT_STACK_DEPTH][3];
	int32_t m_pc_sp, m_cntr_sp, m_stat_sp, m_loop_sp;
	uint8_t m_flagout, m_flagin, m_fl0, m_fl1, m_fl2;
	uint16_t m_imask;
	uint8_t m_icntl;
	uint16_t m_ifc;
	uint8_t m_irq_state[10], m_irq_latch[10];
	int m_icount, m_mstat_mask, m_imask_mask;
	adsp_core m_alt;
	uint64_t total;
} A;

// tables and register maps (not saved)
static struct {
	int16_t *m_read0_ptr[16];
	uint32_t *m_read1_ptr[16];
	uint32_t *m_read2_ptr[16];
	void *m_alu_xregs[8], *m_alu_yregs[4], *m_mac_xregs[8], *m_mac_yregs[4], *m_shift_xregs[8];
	uint8_t m_condition_table[0x1000];
	uint16_t m_mask_table[0x4000];
	uint16_t m_reverse_table[0x4000];
} T;

static adsp_callbacks adsp_cb;
static int s_req, s_stolen, s_executing;

// ---- memory (page tables of 1K words) ----
enum { PG_NONE, PG_MEM, PG_ROM, PG_HI, PG_IO };
typedef struct {
	uint8_t type;
	uint16_t *p16;   // (PG_MEM, PG_ROM)
	uint16_t *w16;   // (PG_MEM: the fast write path)
	uint32_t *p32;
	uint16_t (*r)(uint32_t a);
	void (*w)(uint32_t a, uint16_t d);
} dpage_t;
typedef struct {
	uint8_t type;
	uint32_t *p32;
	uint32_t (*r)(uint32_t a);
	void (*w)(uint32_t a, uint32_t d);
} ppage_t;
static dpage_t s_dpage[16];
static ppage_t s_ppage[16];

static inline uint16_t data_read(uint32_t addr)
{
	const dpage_t *p = &s_dpage[(addr >> 10) & 15];
	if (p->p16)
		return p->p16[addr & 0x3ff];
	switch (p->type) {
	case PG_HI: return (uint16_t)(p->p32[addr & 0x3ff] >> 8);
	case PG_IO: return p->r(addr & 0x3fff);
	}
	return 0;
}
static inline void data_write(uint32_t addr, uint16_t data)
{
	const dpage_t *p = &s_dpage[(addr >> 10) & 15];
	if (p->w16) {
		p->w16[addr & 0x3ff] = data;
		return;
	}
	switch (p->type) {
	case PG_HI: { uint32_t *q = &p->p32[addr & 0x3ff]; *q = ((uint32_t)data << 8) | (*q & 0xff); break; }
	case PG_IO: p->w(addr & 0x3fff, data); break;
	}
}
static inline uint16_t io_read(uint32_t addr) { (void)addr; return 0; }
static inline void io_write(uint32_t addr, uint16_t data) { (void)addr; (void)data; }
static inline uint32_t program_read(uint32_t addr)
{
	const ppage_t *p = &s_ppage[(addr >> 10) & 15];
	if (p->type == PG_MEM) return p->p32[addr & 0x3ff];
	if (p->type == PG_IO) return p->r(addr & 0x3fff);
	return 0;
}
static inline void program_write(uint32_t addr, uint32_t data)
{
	const ppage_t *p = &s_ppage[(addr >> 10) & 15];
	data &= 0xffffff;
	if (p->type == PG_MEM) p->p32[addr & 0x3ff] = data;
	else if (p->type == PG_IO) p->w(addr & 0x3fff, data);
}
static inline uint32_t opcode_read(void) { return program_read(A.m_pc); }

static void check_irqs(void);

/*===========================================================================
    ASTAT -- ALU/MAC status register
===========================================================================*/

// extracts flags
#define GET_SS          (A.m_astat & SSFLAG)
#define GET_MV          (A.m_astat & MVFLAG)
#define GET_Q           (A.m_astat &  QFLAG)
#define GET_S           (A.m_astat &  SFLAG)
#define GET_C           (A.m_astat &  CFLAG)
#define GET_V           (A.m_astat &  VFLAG)
#define GET_N           (A.m_astat &  NFLAG)
#define GET_Z           (A.m_astat &  ZFLAG)

// clears flags
#define CLR_SS          (A.m_astat &= ~SSFLAG)
#define CLR_MV          (A.m_astat &= ~MVFLAG)
#define CLR_Q           (A.m_astat &=  ~QFLAG)
#define CLR_S           (A.m_astat &=  ~SFLAG)
#define CLR_C           (A.m_astat &=  ~CFLAG)
#define CLR_V           (A.m_astat &=  ~VFLAG)
#define CLR_N           (A.m_astat &=  ~NFLAG)
#define CLR_Z           (A.m_astat &=  ~ZFLAG)

// sets flags
#define SET_SS          (A.m_astat |= SSFLAG)
#define SET_MV          (A.m_astat |= MVFLAG)
#define SET_Q           (A.m_astat |=  QFLAG)
#define SET_S           (A.m_astat |=  SFLAG)
#define SET_C           (A.m_astat |=  CFLAG)
#define SET_V           (A.m_astat |=  VFLAG)
#define SET_Z           (A.m_astat |=  ZFLAG)
#define SET_N           (A.m_astat |=  NFLAG)

// flag clearing; must be done before setting
#define CLR_FLAGS       (A.m_astat &= A.m_astat_clear)

// compute flags
#define CALC_Z(r)       (A.m_astat |= ((r & 0xffff) == 0))
#define CALC_N(r)       (A.m_astat |= (r >> 14) & 0x02)
#define CALC_V(s,d,r)   (A.m_astat |= ((s ^ d ^ r ^ (r >> 1)) >> 13) & 0x04)
#define CALC_C(r)       (A.m_astat |= (r >> 13) & 0x08)
#define CALC_C_SUB(r)   (A.m_astat |= (~r >> 13) & 0x08)
#define CALC_NZ(r)      CLR_FLAGS; CALC_N(r); CALC_Z(r)
#define CALC_NZV(s,d,r) CLR_FLAGS; CALC_N(r); CALC_Z(r); CALC_V(s,d,r)
#define CALC_NZVC(s,d,r) CLR_FLAGS; CALC_N(r); CALC_Z(r); CALC_V(s,d,r); CALC_C(r)
#define CALC_NZVC_SUB(s,d,r) CLR_FLAGS; CALC_N(r); CALC_Z(r); CALC_V(s,d,r); CALC_C_SUB(r)

// ADSP-218x constants
static const int32_t constants[] =
{
	0x0001, 0xfffe, 0x0002, 0xfffd, 0x0004, 0xfffb, 0x0008, 0xfff7,
	0x0010, 0xffef, 0x0020, 0xffdf, 0x0040, 0xffbf, 0x0080, 0xff7f,
	0x0100, 0xfeff, 0x0200, 0xfdff, 0x0400, 0xfbff, 0x0800, 0xf7ff,
	0x1000, 0xefff, 0x2000, 0xdfff, 0x4000, 0xbfff, 0x8000, 0x7fff
};



/*===========================================================================
    MSTAT -- ALU/MAC control register
===========================================================================*/

// flag definitions
enum
{
	MSTAT_BANK      = 0x01,             // register bank select
	MSTAT_REVERSE   = 0x02,             // bit-reverse addressing enable (DAG1)
	MSTAT_STICKYV   = 0x04,             // sticky ALU overflow enable
	MSTAT_SATURATE  = 0x08,             // AR saturation mode enable
	MSTAT_INTEGER   = 0x10,             // MAC result placement; 0=fractional, 1=integer
	MSTAT_TIMER     = 0x20,             // timer enable
	MSTAT_GOMODE    = 0x40              // go mode enable
};

// you must call this in order to change MSTAT
static inline void update_mstat()
{
	if ((A.m_mstat ^ A.m_mstat_prev) & MSTAT_BANK)
	{
		adsp_core temp = A.m_core;
		A.m_core = A.m_alt;
		A.m_alt = temp;
	}
	if ((A.m_mstat ^ A.m_mstat_prev) & MSTAT_TIMER)
		adsp_cb.timer_fired((A.m_mstat & MSTAT_TIMER) != 0);
	if (A.m_mstat & MSTAT_STICKYV)
		A.m_astat_clear = ~(CFLAG | NFLAG | ZFLAG);
	else
		A.m_astat_clear = ~(CFLAG | VFLAG | NFLAG | ZFLAG);
	A.m_mstat_prev = A.m_mstat;
}


/*===========================================================================
    SSTAT -- stack status register
===========================================================================*/

// flag definitions
enum
{
	PC_EMPTY        = 0x01,            // PC stack empty
	PC_OVER         = 0x02,            // PC stack overflow
	COUNT_EMPTY     = 0x04,            // count stack empty
	COUNT_OVER      = 0x08,            // count stack overflow
	STATUS_EMPTY    = 0x10,            // status stack empty
	STATUS_OVER     = 0x20,            // status stack overflow
	LOOP_EMPTY      = 0x40,            // loop stack empty
	LOOP_OVER       = 0x80             // loop stack overflow
};


/*===========================================================================
    PC stack handlers
===========================================================================*/

static inline uint32_t pc_stack_top()
{
	if (A.m_pc_sp > 0)
		return A.m_pc_stack[A.m_pc_sp - 1];
	else
		return A.m_pc_stack[0];
}

static inline void set_pc_stack_top(uint32_t top)
{
	if (A.m_pc_sp > 0)
		A.m_pc_stack[A.m_pc_sp - 1] = top;
	else
		A.m_pc_stack[0] = top;
}

static inline void pc_stack_push()
{
	if (A.m_pc_sp < PC_STACK_DEPTH)
	{
		A.m_pc_stack[A.m_pc_sp] = A.m_pc;
		A.m_pc_sp++;
		A.m_sstat &= ~PC_EMPTY;
	}
	else
		A.m_sstat |= PC_OVER;
}

static inline void pc_stack_push_val(uint32_t val)
{
	if (A.m_pc_sp < PC_STACK_DEPTH)
	{
		A.m_pc_stack[A.m_pc_sp] = val;
		A.m_pc_sp++;
		A.m_sstat &= ~PC_EMPTY;
	}
	else
		A.m_sstat |= PC_OVER;
}

static inline void pc_stack_pop()
{
	if (A.m_pc_sp > 0)
	{
		A.m_pc_sp--;
		if (A.m_pc_sp == 0)
			A.m_sstat |= PC_EMPTY;
	}
	A.m_pc = A.m_pc_stack[A.m_pc_sp];
}

static inline uint32_t pc_stack_pop_val()
{
	if (A.m_pc_sp > 0)
	{
		A.m_pc_sp--;
		if (A.m_pc_sp == 0)
			A.m_sstat |= PC_EMPTY;
	}
	return A.m_pc_stack[A.m_pc_sp];
}


/*===========================================================================
    CNTR stack handlers
===========================================================================*/

static inline uint32_t cntr_stack_top()
{
	if (A.m_cntr_sp > 0)
		return A.m_cntr_stack[A.m_cntr_sp - 1];
	else
		return A.m_cntr_stack[0];
}

static inline void cntr_stack_push()
{
	if (A.m_cntr_sp < CNTR_STACK_DEPTH)
	{
		A.m_cntr_stack[A.m_cntr_sp] = A.m_cntr;
		A.m_cntr_sp++;
		A.m_sstat &= ~COUNT_EMPTY;
	}
	else
		A.m_sstat |= COUNT_OVER;
}

static inline void cntr_stack_pop()
{
	if (A.m_cntr_sp > 0)
	{
		A.m_cntr_sp--;
		if (A.m_cntr_sp == 0)
			A.m_sstat |= COUNT_EMPTY;
	}
	A.m_cntr = A.m_cntr_stack[A.m_cntr_sp];
}


/*===========================================================================
    LOOP stack handlers
===========================================================================*/

static inline uint32_t loop_stack_top()
{
	if (A.m_loop_sp > 0)
		return A.m_loop_stack[A.m_loop_sp - 1];
	else
		return A.m_loop_stack[0];
}

static inline void loop_stack_push(uint32_t value)
{
	if (A.m_loop_sp < LOOP_STACK_DEPTH)
	{
		A.m_loop_stack[A.m_loop_sp] = value;
		A.m_loop_sp++;
		A.m_loop = value >> 4;
		A.m_loop_condition = value & 15;
		A.m_sstat &= ~LOOP_EMPTY;
	}
	else
		A.m_sstat |= LOOP_OVER;
}

static inline void loop_stack_pop()
{
	if (A.m_loop_sp > 0)
	{
		A.m_loop_sp--;
		if (A.m_loop_sp == 0)
		{
			A.m_loop = 0xffff;
			A.m_loop_condition = 0;
			A.m_sstat |= LOOP_EMPTY;
		}
		else
		{
			A.m_loop = A.m_loop_stack[A.m_loop_sp - 1] >> 4;
			A.m_loop_condition = A.m_loop_stack[A.m_loop_sp - 1] & 15;
		}
	}
}


/*===========================================================================
    STAT stack handlers
===========================================================================*/

static inline void stat_stack_push()
{
	if (A.m_stat_sp < STAT_STACK_DEPTH)
	{
		A.m_stat_stack[A.m_stat_sp][0] = A.m_mstat;
		A.m_stat_stack[A.m_stat_sp][1] = A.m_imask;
		A.m_stat_stack[A.m_stat_sp][2] = A.m_astat;
		A.m_stat_sp++;
		A.m_sstat &= ~STATUS_EMPTY;
	}
	else
		A.m_sstat |= STATUS_OVER;
}

static inline void stat_stack_pop()
{
	if (A.m_stat_sp > 0)
	{
		A.m_stat_sp--;
		if (A.m_stat_sp == 0)
			A.m_sstat |= STATUS_EMPTY;
	}
	A.m_mstat = A.m_stat_stack[A.m_stat_sp][0];
	update_mstat();
	A.m_imask = A.m_stat_stack[A.m_stat_sp][1];
	A.m_astat = A.m_stat_stack[A.m_stat_sp][2];
	check_irqs();
}



/*===========================================================================
    condition code checking
===========================================================================*/

// gcc doesn't want to inline this, so we use a macro
#define condition(c) (((c) != 14) ? (T.m_condition_table[((c) << 8) | A.m_astat]) : slow_condition())

/*
static inline int condition(int c)
{
    if (c != 14)
        return T.m_condition_table[((c) << 8) | A.m_astat];
    else
        return slow_condition();
}
*/

static int slow_condition()
{
	if ((int32_t)--A.m_cntr > 0)
		return 1;
	else
	{
		cntr_stack_pop();
		return 0;
	}
}



/*===========================================================================
    register writing
===========================================================================*/

static inline void update_i(int which)
{
	A.m_base[which] = A.m_i[which] & A.m_lmask[which];
}

static inline void update_l(int which)
{
	A.m_lmask[which] = T.m_mask_table[A.m_l[which] & 0x3fff];
	A.m_base[which] = A.m_i[which] & A.m_lmask[which];
}

static inline void update_dmovlay()
{
	(void)0;
}

static AINLINE void write_reg0(int regnum, int32_t val)
{
	switch (regnum)
	{
		case 0x00:  A.m_core.ax0.s = val;                 break;
		case 0x01:  A.m_core.ax1.s = val;                 break;
		case 0x02:  A.m_core.mx0.s = val;                 break;
		case 0x03:  A.m_core.mx1.s = val;                 break;
		case 0x04:  A.m_core.ay0.s = val;                 break;
		case 0x05:  A.m_core.ay1.s = val;                 break;
		case 0x06:  A.m_core.my0.s = val;                 break;
		case 0x07:  A.m_core.my1.s = val;                 break;
		case 0x08:  A.m_core.si.s = val;                  break;
		case 0x09:  A.m_core.se.s = (int8_t)val;          break;
		case 0x0a:  A.m_core.ar.s = val;                  break;
		case 0x0b:  A.m_core.mr.mrx.mr0.s = val;          break;
		case 0x0c:  A.m_core.mr.mrx.mr1.s = val; A.m_core.mr.mrx.mr2.s = (int16_t)val >> 15;  break;
		case 0x0d:  A.m_core.mr.mrx.mr2.s = (int8_t)val;  break;
		case 0x0e:  A.m_core.sr.srx.sr0.s = val;          break;
		case 0x0f:  A.m_core.sr.srx.sr1.s = val;          break;
	}
}

static void write_reg1(int regnum, int32_t val)
{
	int index = regnum & 3;
	switch (regnum >> 2)
	{
		case 0:
			A.m_i[index] = val & 0x3fff;
			update_i(index);
			break;

		case 1:
			A.m_m[index] = sext(val, 14);
			break;

		case 2:
			A.m_l[index] = val & 0x3fff;
			update_l(index);
			break;

		case 3:
			if (ADSP_CHIP == CHIP_TYPE_ADSP2181) // ADSP2181 Overlay register
			{
				switch (index)
				{
					case 2:
						A.m_pmovlay = val & 0x3fff;
						//update_pmovlay();
						break;
					case 3:
						A.m_dmovlay = val & 0x3fff;
						update_dmovlay();
						break;
					default:
						logerror("ADSP %04x: Writing to an invalid register! RGP=01 RegCode=%1X Val=%04X\n", A.m_ppc, regnum, val);
						break;
				}
			}
			else
				logerror("ADSP %04x: Writing to an invalid register! RGP=01 RegCode=%1X Val=%04X\n", A.m_ppc, regnum, val);
			break;
	}
}

static void write_reg2(int regnum, int32_t val)
{
	int index = 4 + (regnum & 3);
	switch (regnum >> 2)
	{
		case 0:
			A.m_i[index] = val & 0x3fff;
			update_i(index);
			break;

		case 1:
			A.m_m[index] = sext(val, 14);
			break;

		case 2:
			A.m_l[index] = val & 0x3fff;
			update_l(index);
			break;

		case 3:
			logerror("ADSP %04x: Writing to an invalid register!\n", A.m_ppc);
			break;
	}
}

static void write_reg3(int regnum, int32_t val)
{
	switch (regnum)
	{
		case 0x00:  A.m_astat = val & 0x00ff;                         break;
		case 0x01:  A.m_mstat = val & A.m_mstat_mask; update_mstat();   break;
		case 0x03:  A.m_imask = val & A.m_imask_mask; check_irqs();     break;
		case 0x04:  A.m_icntl = val & 0x001f; check_irqs();           break;
		case 0x05:  cntr_stack_push(); A.m_cntr = val & 0x3fff;       break;
		case 0x06:  A.m_core.sb.s = sext(val, 5);               break;
		case 0x07:  A.m_px = val;                                     break;
		case 0x09:  adsp_cb.sport_tx(0, val);                  break;
		case 0x0b:  adsp_cb.sport_tx(1, val);                  break;
		case 0x0c:
			A.m_ifc = val;
			if (ADSP_CHIP >= CHIP_TYPE_ADSP2181)
			{
				// clear timer
				if (BIT(val, 1)) A.m_irq_latch[ADSP2181_IRQ0] = 0;
				if (BIT(val, 2)) A.m_irq_latch[ADSP2181_IRQ1] = 0;
				if (BIT(val, 3)) A.m_irq_latch[ADSP2181_BDMA] = 0;
				if (BIT(val, 4)) A.m_irq_latch[ADSP2181_IRQE] = 0;
				if (BIT(val, 5)) A.m_irq_latch[ADSP2181_SPORT0_RX] = 0;
				if (BIT(val, 6)) A.m_irq_latch[ADSP2181_SPORT0_TX] = 0;
				if (BIT(val, 7)) A.m_irq_latch[ADSP2181_IRQ2] = 0;
				// force timer
				if (BIT(val, 9)) A.m_irq_latch[ADSP2181_IRQ0] = 1;
				if (BIT(val, 10)) A.m_irq_latch[ADSP2181_IRQ1] = 1;
				if (BIT(val, 11)) A.m_irq_latch[ADSP2181_BDMA] = 1;
				if (BIT(val, 12)) A.m_irq_latch[ADSP2181_IRQE] = 1;
				if (BIT(val, 13)) A.m_irq_latch[ADSP2181_SPORT0_RX] = 1;
				if (BIT(val, 14)) A.m_irq_latch[ADSP2181_SPORT0_TX] = 1;
				if (BIT(val, 15)) A.m_irq_latch[ADSP2181_IRQ2] = 1;
			}
			else
			{
				// clear timer
				if (BIT(val, 1)) A.m_irq_latch[ADSP2101_IRQ0] = 0;
				if (BIT(val, 2)) A.m_irq_latch[ADSP2101_IRQ1] = 0;
				if (BIT(val, 3)) A.m_irq_latch[ADSP2101_SPORT0_RX] = 0;
				if (BIT(val, 4)) A.m_irq_latch[ADSP2101_SPORT0_TX] = 0;
				if (BIT(val, 5)) A.m_irq_latch[ADSP2101_IRQ2] = 0;
				// set timer
				if (BIT(val, 7)) A.m_irq_latch[ADSP2101_IRQ0] = 1;
				if (BIT(val, 8)) A.m_irq_latch[ADSP2101_IRQ1] = 1;
				if (BIT(val, 9)) A.m_irq_latch[ADSP2101_SPORT0_RX] = 1;
				if (BIT(val, 10)) A.m_irq_latch[ADSP2101_SPORT0_TX] = 1;
				if (BIT(val, 11)) A.m_irq_latch[ADSP2101_IRQ2] = 1;
			}
			check_irqs();
			break;
		case 0x0d:  A.m_cntr = val & 0x3fff;              break;
		case 0x0f:  pc_stack_push_val(val & 0x3fff);    break;
		default:    logerror("ADSP %04x: Writing to an invalid register!\n", A.m_ppc); break;
	}
}

#define WRITE_REG(adsp,grp,reg,val) ((this->*wr_reg[grp][reg])(val))



/*===========================================================================
    register reading
===========================================================================*/

static AINLINE int32_t read_reg0(int regnum)
{
	return *T.m_read0_ptr[regnum];
}

static int32_t read_reg1(int regnum)
{
	int32_t ret = *T.m_read1_ptr[regnum];
	// ADSP2181 Overlay register
	if ((ADSP_CHIP == CHIP_TYPE_ADSP2181) && (regnum >= 0xe))
	{
		switch (regnum)
		{
			case 0xe: ret = A.m_pmovlay; break;
			case 0xf: ret = A.m_dmovlay; break;
			default:                   break;
		}
	}

	return ret;
}

static int32_t read_reg2(int regnum)
{
	return *T.m_read2_ptr[regnum];
}

static int32_t read_reg3(int regnum)
{
	switch (regnum)
	{
		case 0x00:  return A.m_astat;
		case 0x01:  return A.m_mstat;
		case 0x02:  return A.m_sstat;
		case 0x03:  return A.m_imask;
		case 0x04:  return A.m_icntl;
		case 0x05:  return A.m_cntr;
		case 0x06:  return A.m_core.sb.s;
		case 0x07:  return A.m_px;
		case 0x08:  return 0;
		case 0x0a:  return 0;
		case 0x0f:  return pc_stack_pop_val();
		default:    logerror("ADSP %04x: Reading from an invalid register! RGP=b11 RegCode=%1X\n", A.m_ppc, regnum); return 0;
	}
}



/*===========================================================================
    Modulus addressing logic
===========================================================================*/

static inline void modify_address(uint32_t ireg, uint32_t mreg)
{
	uint32_t base = A.m_base[ireg];
	uint32_t i = A.m_i[ireg];
	uint32_t l = A.m_l[ireg];

	i = (i + A.m_m[mreg]) & 0x3fff;
	if (i < base) i += l;
	else if (i >= base + l) i -= l;
	A.m_i[ireg] = i;
}



/*===========================================================================
    Data memory accessors
===========================================================================*/

static AINLINE void data_write_dag1(uint32_t op, int32_t val)
{
	uint32_t ireg = BIT(op, 2, 2);
	uint32_t mreg = BIT(op, 0, 2);
	uint32_t base = A.m_base[ireg];
	uint32_t i = A.m_i[ireg];
	uint32_t l = A.m_l[ireg];

	if ( A.m_mstat & MSTAT_REVERSE )
	{
		uint32_t ir = T.m_reverse_table[ i & 0x3fff ];
		data_write(ir, val);
	}
	else
		data_write(i, val);

	i = (i + A.m_m[mreg]) & 0x3fff;
	if (i < base) i += l;
	else if (i >= base + l) i -= l;

	A.m_i[ireg] = i;
}


static AINLINE uint32_t data_read_dag1(uint32_t op)
{
	uint32_t ireg = BIT(op, 2, 2);
	uint32_t mreg = BIT(op, 0, 2);
	uint32_t base = A.m_base[ireg];
	uint32_t i = A.m_i[ireg];
	uint32_t l = A.m_l[ireg];
	uint32_t res;

	if (A.m_mstat & MSTAT_REVERSE)
	{
		uint32_t ir = T.m_reverse_table[i & 0x3fff];
		res = data_read(ir);
	}
	else
		res = data_read(i);

	i = (i + A.m_m[mreg]) & 0x3fff;
	if (i < base) i += l;
	else if (i >= base + l) i -= l;
	A.m_i[ireg] = i;

	return res;
}

static AINLINE void data_write_dag2(uint32_t op, int32_t val)
{
	uint32_t ireg = 4 + BIT(op, 2, 2);
	uint32_t mreg = 4 + BIT(op, 0, 2);
	uint32_t base = A.m_base[ireg];
	uint32_t i = A.m_i[ireg];
	uint32_t l = A.m_l[ireg];

	data_write(i, val);

	i = (i + A.m_m[mreg]) & 0x3fff;
	if (i < base) i += l;
	else if (i >= base + l) i -= l;
	A.m_i[ireg] = i;
}


static AINLINE uint32_t data_read_dag2(uint32_t op)
{
	uint32_t ireg = 4 + BIT(op, 2, 2);
	uint32_t mreg = 4 + BIT(op, 0, 2);
	uint32_t base = A.m_base[ireg];
	uint32_t i = A.m_i[ireg];
	uint32_t l = A.m_l[ireg];

	uint32_t res = data_read(i);

	i = (i + A.m_m[mreg]) & 0x3fff;
	if (i < base) i += l;
	else if (i >= base + l) i -= l;
	A.m_i[ireg] = i;

	return res;
}

/*===========================================================================
    Program memory accessors
===========================================================================*/

static inline void pgm_write_dag2(uint32_t op, int32_t val)
{
	uint32_t ireg = 4 + BIT(op, 2, 2);
	uint32_t mreg = 4 + BIT(op, 0, 2);
	uint32_t base = A.m_base[ireg];
	uint32_t i = A.m_i[ireg];
	uint32_t l = A.m_l[ireg];

	program_write(i, (val << 8) | A.m_px);

	i = (i + A.m_m[mreg]) & 0x3fff;
	if (i < base) i += l;
	else if (i >= base + l) i -= l;
	A.m_i[ireg] = i;
}


static inline uint32_t pgm_read_dag2(uint32_t op)
{
	uint32_t ireg = 4 + BIT(op, 2, 2);
	uint32_t mreg = 4 + BIT(op, 0, 2);
	uint32_t base = A.m_base[ireg];
	uint32_t i = A.m_i[ireg];
	uint32_t l = A.m_l[ireg];
	uint32_t res;

	res = program_read(i);
	A.m_px = res;
	res >>= 8;

	i = (i + A.m_m[mreg]) & 0x3fff;
	if (i < base) i += l;
	else if (i >= base + l) i -= l;
	A.m_i[ireg] = i;

	return res;
}



/*===========================================================================
    register reading
===========================================================================*/

#define ALU_GETXREG_UNSIGNED(x) (*(uint16_t *)T.m_alu_xregs[x])
#define ALU_GETYREG_UNSIGNED(y) (*(uint16_t *)T.m_alu_yregs[y])

#define MAC_GETXREG_UNSIGNED(x) (*(uint16_t *)T.m_mac_xregs[x])
#define MAC_GETXREG_SIGNED(x)   (*( int16_t *)T.m_mac_xregs[x])
#define MAC_GETYREG_UNSIGNED(y) (*(uint16_t *)T.m_mac_yregs[y])
#define MAC_GETYREG_SIGNED(y)   (*( int16_t *)T.m_mac_yregs[y])

#define SHIFT_GETXREG_UNSIGNED(x) (*(uint16_t *)T.m_shift_xregs[x])
#define SHIFT_GETXREG_SIGNED(x)   (*( int16_t *)T.m_shift_xregs[x])



/*===========================================================================
    ALU operations (result in AR)
===========================================================================*/

static AINLINE void alu_op_ar_k(uint32_t op, const int kind)
{
	int32_t xop = BIT(op, 8, 3);
	int32_t yop = BIT(op, 11, 2);
	int32_t res;

	switch (kind)  //JB
	{
		case 0x00:
			// Y                Clear when y = 0
			res = ALU_GETYREG_UNSIGNED(yop);
			CALC_NZ(res);
			break;
		case 0x01:
			// Y + 1            PASS 1 when y = 0
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = yop + 1;
			CALC_NZ(res);
			if (yop == 0x7fff) SET_V;
			else if (yop == 0xffff) SET_C;
			break;
		case 0x02:
			// X + Y + C
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			yop += GET_C >> 3;
			res = xop + yop;
			CALC_NZVC(xop, yop, res);
			break;
		case 0x03:
			// X + Y            X when y = 0
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = xop + yop;
			CALC_NZVC(xop, yop, res);
			break;
		case 0x04:
			// NOT Y
			res = ALU_GETYREG_UNSIGNED(yop) ^ 0xffff;
			CALC_NZ(res);
			break;
		case 0x05:
			// -Y
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = -yop;
			CALC_NZ(res);
			if (yop == 0x8000) SET_V;
			if (yop == 0x0000) SET_C;
			break;
		case 0x06:
			// X - Y + C - 1    X + C - 1 when y = 0
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = xop - yop + (GET_C >> 3) - 1;
			CALC_NZVC_SUB(xop, yop, res);
			break;
		case 0x07:
			// X - Y
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = xop - yop;
			CALC_NZVC_SUB(xop, yop, res);
			break;
		case 0x08:
			// Y - 1            PASS -1 when y = 0
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = yop - 1;
			CALC_NZ(res);
			if (yop == 0x8000) SET_V;
			else if (yop == 0x0000) SET_C;
			break;
		case 0x09:
			// Y - X            -X when y = 0
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = yop - xop;
			CALC_NZVC_SUB(yop, xop, res);
			break;
		case 0x0a:
			// Y - X + C - 1    -X + C - 1 when y = 0
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = yop - xop + (GET_C >> 3) - 1;
			CALC_NZVC_SUB(yop, xop, res);
			break;
		case 0x0b:
			// NOT X
			res = ALU_GETXREG_UNSIGNED(xop) ^ 0xffff;
			CALC_NZ(res);
			break;
		case 0x0c:
			// X AND Y
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = xop & yop;
			CALC_NZ(res);
			break;
		case 0x0d:
			// X OR Y
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = xop | yop;
			CALC_NZ(res);
			break;
		case 0x0e:
			// X XOR Y
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = xop ^ yop;
			CALC_NZ(res);
			break;
		case 0x0f:
			// ABS X
			xop = ALU_GETXREG_UNSIGNED(xop);
			res = (xop & 0x8000) ? -xop : xop;
			CLR_FLAGS;
			CLR_S;
			if (xop == 0) SET_Z;
			if (xop == 0x8000) SET_N, SET_V;
			if (xop & 0x8000) SET_S;
			break;
		default:
			res = 0;    // just to keep the compiler happy
			break;
	}

	// saturate
	if ((A.m_mstat & MSTAT_SATURATE) && GET_V) res = GET_C ? -32768 : 32767;

	// set the final value
	A.m_core.ar.u = res;
}



/*===========================================================================
    ALU operations (result in AR, constant yop)
===========================================================================*/

static void alu_op_ar_const(uint32_t op)
{
	int32_t xop = BIT(op, 8, 3);
	int32_t yop = constants[BIT(op, 5, 3) | BIT(op, 11, 2) << 3];
	int32_t res;

	switch (BIT(op, 13, 4))  //JB
	{
		case 0x00:
			// Y                Clear when y = 0
			res = yop;
			CALC_NZ(res);
			break;
		case 0x01:
			// Y + 1            PASS 1 when y = 0
			res = yop + 1;
			CALC_NZ(res);
			if (yop == 0x7fff) SET_V;
			else if (yop == 0xffff) SET_C;
			break;
		case 0x02:
			// X + Y + C
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop += GET_C >> 3;
			res = xop + yop;
			CALC_NZVC(xop, yop, res);
			break;
		case 0x03:
			// X + Y            X when y = 0
			xop = ALU_GETXREG_UNSIGNED(xop);
			res = xop + yop;
			CALC_NZVC(xop, yop, res);
			break;
		case 0x04:
			// NOT Y
			res = yop ^ 0xffff;
			CALC_NZ(res);
			break;
		case 0x05:
			// -Y
			res = -yop;
			CALC_NZ(res);
			if (yop == 0x8000) SET_V;
			if (yop == 0x0000) SET_C;
			break;
		case 0x06:
			// X - Y + C - 1    X + C - 1 when y = 0
			xop = ALU_GETXREG_UNSIGNED(xop);
			res = xop - yop + (GET_C >> 3) - 1;
			CALC_NZVC_SUB(xop, yop, res);
			break;
		case 0x07:
			// X - Y
			xop = ALU_GETXREG_UNSIGNED(xop);
			res = xop - yop;
			CALC_NZVC_SUB(xop, yop, res);
			break;
		case 0x08:
			// Y - 1            PASS -1 when y = 0
			res = yop - 1;
			CALC_NZ(res);
			if (yop == 0x8000) SET_V;
			else if (yop == 0x0000) SET_C;
			break;
		case 0x09:
			// Y - X            -X when y = 0
			xop = ALU_GETXREG_UNSIGNED(xop);
			res = yop - xop;
			CALC_NZVC_SUB(yop, xop, res);
			break;
		case 0x0a:
			// Y - X + C - 1    -X + C - 1 when y = 0
			xop = ALU_GETXREG_UNSIGNED(xop);
			res = yop - xop + (GET_C >> 3) - 1;
			CALC_NZVC_SUB(yop, xop, res);
			break;
		case 0x0b:
			// NOT X
			res = ALU_GETXREG_UNSIGNED(xop) ^ 0xffff;
			CALC_NZ(res);
			break;
		case 0x0c:
			// X AND Y
			xop = ALU_GETXREG_UNSIGNED(xop);
			res = xop & yop;
			CALC_NZ(res);
			break;
		case 0x0d:
			// X OR Y
			xop = ALU_GETXREG_UNSIGNED(xop);
			res = xop | yop;
			CALC_NZ(res);
			break;
		case 0x0e:
			// X XOR Y
			xop = ALU_GETXREG_UNSIGNED(xop);
			res = xop ^ yop;
			CALC_NZ(res);
			break;
		case 0x0f:
			// ABS X
			xop = ALU_GETXREG_UNSIGNED(xop);
			res = (xop & 0x8000) ? -xop : xop;
			CLR_FLAGS;
			CLR_S;
			if (xop == 0) SET_Z;
			if (xop == 0x8000) SET_N, SET_V;
			if (xop & 0x8000) SET_S;
			break;
		default:
			res = 0;    // just to keep the compiler happy
			break;
	}

	// saturate
	if ((A.m_mstat & MSTAT_SATURATE) && GET_V) res = GET_C ? -32768 : 32767;

	// set the final value
	A.m_core.ar.u = res;
}



/*===========================================================================
    ALU operations (result in AF)
===========================================================================*/

static AINLINE void alu_op_af_k(uint32_t op, const int kind)
{
	int32_t xop = BIT(op, 8, 3);
	int32_t yop = BIT(op, 11, 2);
	int32_t res;

	switch (kind)  //JB
	{
		case 0x00:
			// Y                Clear when y = 0
			res = ALU_GETYREG_UNSIGNED(yop);
			CALC_NZ(res);
			break;
		case 0x01:
			// Y + 1            PASS 1 when y = 0
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = yop + 1;
			CALC_NZ(res);
			if (yop == 0x7fff) SET_V;
			else if (yop == 0xffff) SET_C;
			break;
		case 0x02:
			// X + Y + C
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			yop += GET_C >> 3;
			res = xop + yop;
			CALC_NZVC(xop, yop, res);
			break;
		case 0x03:
			// X + Y            X when y = 0
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = xop + yop;
			CALC_NZVC(xop, yop, res);
			break;
		case 0x04:
			// NOT Y
			res = ALU_GETYREG_UNSIGNED(yop) ^ 0xffff;
			CALC_NZ(res);
			break;
		case 0x05:
			// -Y
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = -yop;
			CALC_NZ(res);
			if (yop == 0x8000) SET_V;
			if (yop == 0x0000) SET_C;
			break;
		case 0x06:
			// X - Y + C - 1    X + C - 1 when y = 0
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = xop - yop + (GET_C >> 3) - 1;
			CALC_NZVC_SUB(xop, yop, res);
			break;
		case 0x07:
			// X - Y
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = xop - yop;
			CALC_NZVC_SUB(xop, yop, res);
			break;
		case 0x08:
			// Y - 1            PASS -1 when y = 0
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = yop - 1;
			CALC_NZ(res);
			if (yop == 0x8000) SET_V;
			else if (yop == 0x0000) SET_C;
			break;
		case 0x09:
			// Y - X            -X when y = 0
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = yop - xop;
			CALC_NZVC_SUB(yop, xop, res);
			break;
		case 0x0a:
			// Y - X + C - 1    -X + C - 1 when y = 0
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = yop - xop + (GET_C >> 3) - 1;
			CALC_NZVC_SUB(yop, xop, res);
			break;
		case 0x0b:
			// NOT X
			res = ALU_GETXREG_UNSIGNED(xop) ^ 0xffff;
			CALC_NZ(res);
			break;
		case 0x0c:
			// X AND Y
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = xop & yop;
			CALC_NZ(res);
			break;
		case 0x0d:
			// X OR Y
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = xop | yop;
			CALC_NZ(res);
			break;
		case 0x0e:
			// X XOR Y
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = xop ^ yop;
			CALC_NZ(res);
			break;
		case 0x0f:
			// ABS X
			xop = ALU_GETXREG_UNSIGNED(xop);
			res = (xop & 0x8000) ? -xop : xop;
			CLR_FLAGS;
			CLR_S;
			if (xop == 0) SET_Z;
			if (xop == 0x8000) SET_N, SET_V;
			if (xop & 0x8000) SET_S;
			break;
		default:
			res = 0;    // just to keep the compiler happy
			break;
	}

	// set the final value
	A.m_core.af.u = res;
}



/*===========================================================================
    ALU operations (result in AF, constant yop)
===========================================================================*/

static void alu_op_af_const(uint32_t op)
{
	int32_t xop = BIT(op, 8, 3);
	int32_t yop = constants[BIT(op, 5, 3) | BIT(op, 11, 2) << 3];
	int32_t res;

	switch (BIT(op, 13, 4))  //JB
	{
		case 0x00:
			// Y                Clear when y = 0
			res = yop;
			CALC_NZ(res);
			break;
		case 0x01:
			// Y + 1            PASS 1 when y = 0
			res = yop + 1;
			CALC_NZ(res);
			if (yop == 0x7fff) SET_V;
			else if (yop == 0xffff) SET_C;
			break;
		case 0x02:
			// X + Y + C
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop += GET_C >> 3;
			res = xop + yop;
			CALC_NZVC(xop, yop, res);
			break;
		case 0x03:
			// X + Y            X when y = 0
			xop = ALU_GETXREG_UNSIGNED(xop);
			res = xop + yop;
			CALC_NZVC(xop, yop, res);
			break;
		case 0x04:
			// NOT Y
			res = yop ^ 0xffff;
			CALC_NZ(res);
			break;
		case 0x05:
			// -Y
			res = -yop;
			CALC_NZ(res);
			if (yop == 0x8000) SET_V;
			if (yop == 0x0000) SET_C;
			break;
		case 0x06:
			// X - Y + C - 1    X + C - 1 when y = 0
			xop = ALU_GETXREG_UNSIGNED(xop);
			res = xop - yop + (GET_C >> 3) - 1;
			CALC_NZVC_SUB(xop, yop, res);
			break;
		case 0x07:
			// X - Y
			xop = ALU_GETXREG_UNSIGNED(xop);
			res = xop - yop;
			CALC_NZVC_SUB(xop, yop, res);
			break;
		case 0x08:
			// Y - 1            PASS -1 when y = 0
			res = yop - 1;
			CALC_NZ(res);
			if (yop == 0x8000) SET_V;
			else if (yop == 0x0000) SET_C;
			break;
		case 0x09:
			// Y - X            -X when y = 0
			xop = ALU_GETXREG_UNSIGNED(xop);
			res = yop - xop;
			CALC_NZVC_SUB(yop, xop, res);
			break;
		case 0x0a:
			// Y - X + C - 1    -X + C - 1 when y = 0
			xop = ALU_GETXREG_UNSIGNED(xop);
			res = yop - xop + (GET_C >> 3) - 1;
			CALC_NZVC_SUB(yop, xop, res);
			break;
		case 0x0b:
			// NOT X
			res = ALU_GETXREG_UNSIGNED(xop) ^ 0xffff;
			CALC_NZ(res);
			break;
		case 0x0c:
			// X AND Y
			xop = ALU_GETXREG_UNSIGNED(xop);
			res = xop & yop;
			CALC_NZ(res);
			break;
		case 0x0d:
			// X OR Y
			xop = ALU_GETXREG_UNSIGNED(xop);
			res = xop | yop;
			CALC_NZ(res);
			break;
		case 0x0e:
			// X XOR Y
			xop = ALU_GETXREG_UNSIGNED(xop);
			res = xop ^ yop;
			CALC_NZ(res);
			break;
		case 0x0f:
			// ABS X
			xop = ALU_GETXREG_UNSIGNED(xop);
			res = (xop & 0x8000) ? -xop : xop;
			CLR_FLAGS;
			CLR_S;
			if (xop == 0) SET_Z;
			if (xop == 0x8000) SET_N, SET_V;
			if (xop & 0x8000) SET_S;
			break;
		default:
			res = 0;    // just to keep the compiler happy
			break;
	}

	// set the final value
	A.m_core.af.u = res;
}



/*===========================================================================
    ALU operations (no result)
===========================================================================*/

static void alu_op_none(uint32_t op)
{
	int32_t xop = BIT(op, 8, 3);
	int32_t yop = BIT(op, 11, 2);
	int32_t res;

	switch (BIT(op, 13, 4))  //JB
	{
		case 0x00:
			// Y                Clear when y = 0
			res = ALU_GETYREG_UNSIGNED(yop);
			CALC_NZ(res);
			break;
		case 0x01:
			// Y + 1            PASS 1 when y = 0
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = yop + 1;
			CALC_NZ(res);
			if (yop == 0x7fff) SET_V;
			else if (yop == 0xffff) SET_C;
			break;
		case 0x02:
			// X + Y + C
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			yop += GET_C >> 3;
			res = xop + yop;
			CALC_NZVC(xop, yop, res);
			break;
		case 0x03:
			// X + Y            X when y = 0
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = xop + yop;
			CALC_NZVC(xop, yop, res);
			break;
		case 0x04:
			// NOT Y
			res = ALU_GETYREG_UNSIGNED(yop) ^ 0xffff;
			CALC_NZ(res);
			break;
		case 0x05:
			// -Y
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = -yop;
			CALC_NZ(res);
			if (yop == 0x8000) SET_V;
			if (yop == 0x0000) SET_C;
			break;
		case 0x06:
			// X - Y + C - 1    X + C - 1 when y = 0
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = xop - yop + (GET_C >> 3) - 1;
			CALC_NZVC_SUB(xop, yop, res);
			break;
		case 0x07:
			// X - Y
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = xop - yop;
			CALC_NZVC_SUB(xop, yop, res);
			break;
		case 0x08:
			// Y - 1            PASS -1 when y = 0
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = yop - 1;
			CALC_NZ(res);
			if (yop == 0x8000) SET_V;
			else if (yop == 0x0000) SET_C;
			break;
		case 0x09:
			// Y - X            -X when y = 0
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = yop - xop;
			CALC_NZVC_SUB(yop, xop, res);
			break;
		case 0x0a:
			// Y - X + C - 1    -X + C - 1 when y = 0
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = yop - xop + (GET_C >> 3) - 1;
			CALC_NZVC_SUB(yop, xop, res);
			break;
		case 0x0b:
			// NOT X
			res = ALU_GETXREG_UNSIGNED(xop) ^ 0xffff;
			CALC_NZ(res);
			break;
		case 0x0c:
			// X AND Y
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = xop & yop;
			CALC_NZ(res);
			break;
		case 0x0d:
			// X OR Y
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = xop | yop;
			CALC_NZ(res);
			break;
		case 0x0e:
			// X XOR Y
			xop = ALU_GETXREG_UNSIGNED(xop);
			yop = ALU_GETYREG_UNSIGNED(yop);
			res = xop ^ yop;
			CALC_NZ(res);
			break;
		case 0x0f:
			// ABS X
			xop = ALU_GETXREG_UNSIGNED(xop);
			res = (xop & 0x8000) ? -xop : xop;
			CLR_FLAGS;
			CLR_S;
			if (xop == 0) SET_Z;
			if (xop == 0x8000) SET_N, SET_V;
			if (xop & 0x8000) SET_S;
			break;
	}
}



/*===========================================================================
    MAC operations (result in MR)
===========================================================================*/

static AINLINE void mac_op_mr_k(uint32_t op, const int kind)
{
	int8_t shift = (A.m_mstat & MSTAT_INTEGER) ? 0 : 1;
	int32_t xop = BIT(op, 8, 3);
	int32_t yop = BIT(op, 11, 2);
	int32_t temp;
	int64_t res;

	switch (kind)  //JB
	{
		case 0x00:
			// no-op
			return;
		case 0x01:
			// X * Y (RND)
			xop = MAC_GETXREG_SIGNED(xop);
			yop = MAC_GETYREG_SIGNED(yop);
			temp = (xop * yop) << shift;
			res = (int64_t)temp;
#if 0
			if ((res & 0xffff) == 0x8000) res &= ~((uint64_t)0x10000);
			else res += (res & 0x8000) << 1;
#else
			temp &= 0xffff;
			res += 0x8000;
			if ( temp == 0x8000 )
				res &= ~((uint64_t)0x10000);
#endif
			break;
		case 0x02:
			// MR + X * Y (RND)
			xop = MAC_GETXREG_SIGNED(xop);
			yop = MAC_GETYREG_SIGNED(yop);
			temp = (xop * yop) << shift;
			res = A.m_core.mr.mr + (int64_t)temp;
#if 0
			if ((res & 0xffff) == 0x8000) res &= ~((uint64_t)0x10000);
			else res += (res & 0x8000) << 1;
#else
			temp &= 0xffff;
			res += 0x8000;
			if ( temp == 0x8000 )
				res &= ~((uint64_t)0x10000);
#endif
			break;
		case 0x03:
			// MR - X * Y (RND)
			xop = MAC_GETXREG_SIGNED(xop);
			yop = MAC_GETYREG_SIGNED(yop);
			temp = (xop * yop) << shift;
			res = A.m_core.mr.mr - (int64_t)temp;
#if 0
			if ((res & 0xffff) == 0x8000) res &= ~((uint64_t)0x10000);
			else res += (res & 0x8000) << 1;
#else
			temp &= 0xffff;
			res += 0x8000;
			if ( temp == 0x8000 )
				res &= ~((uint64_t)0x10000);
#endif
			break;
		case 0x04:
			// X * Y (SS)       Clear when y = 0
			xop = MAC_GETXREG_SIGNED(xop);
			yop = MAC_GETYREG_SIGNED(yop);
			temp = (xop * yop) << shift;
			res = (int64_t)temp;
			break;
		case 0x05:
			// X * Y (SU)
			xop = MAC_GETXREG_SIGNED(xop);
			yop = MAC_GETYREG_UNSIGNED(yop);
			temp = (xop * yop) << shift;
			res = (int64_t)temp;
			break;
		case 0x06:
			// X * Y (US)
			xop = MAC_GETXREG_UNSIGNED(xop);
			yop = MAC_GETYREG_SIGNED(yop);
			temp = (xop * yop) << shift;
			res = (int64_t)temp;
			break;
		case 0x07:
			// X * Y (UU)
			xop = MAC_GETXREG_UNSIGNED(xop);
			yop = MAC_GETYREG_UNSIGNED(yop);
			temp = (xop * yop) << shift;
			res = (int64_t)temp;
			break;
		case 0x08:
			// MR + X * Y (SS)
			xop = MAC_GETXREG_SIGNED(xop);
			yop = MAC_GETYREG_SIGNED(yop);
			temp = (xop * yop) << shift;
			res = A.m_core.mr.mr + (int64_t)temp;
			break;
		case 0x09:
			// MR + X * Y (SU)
			xop = MAC_GETXREG_SIGNED(xop);
			yop = MAC_GETYREG_UNSIGNED(yop);
			temp = (xop * yop) << shift;
			res = A.m_core.mr.mr + (int64_t)temp;
			break;
		case 0x0a:
			// MR + X * Y (US)
			xop = MAC_GETXREG_UNSIGNED(xop);
			yop = MAC_GETYREG_SIGNED(yop);
			temp = (xop * yop) << shift;
			res = A.m_core.mr.mr + (int64_t)temp;
			break;
		case 0x0b:
			// MR + X * Y (UU)
			xop = MAC_GETXREG_UNSIGNED(xop);
			yop = MAC_GETYREG_UNSIGNED(yop);
			temp = (xop * yop) << shift;
			res = A.m_core.mr.mr + (int64_t)temp;
			break;
		case 0x0c:
			// MR - X * Y (SS)
			xop = MAC_GETXREG_SIGNED(xop);
			yop = MAC_GETYREG_SIGNED(yop);
			temp = (xop * yop) << shift;
			res = A.m_core.mr.mr - (int64_t)temp;
			break;
		case 0x0d:
			// MR - X * Y (SU)
			xop = MAC_GETXREG_SIGNED(xop);
			yop = MAC_GETYREG_UNSIGNED(yop);
			temp = (xop * yop) << shift;
			res = A.m_core.mr.mr - (int64_t)temp;
			break;
		case 0x0e:
			// MR - X * Y (US)
			xop = MAC_GETXREG_UNSIGNED(xop);
			yop = MAC_GETYREG_SIGNED(yop);
			temp = (xop * yop) << shift;
			res = A.m_core.mr.mr - (int64_t)temp;
			break;
		case 0x0f:
			// MR - X * Y (UU)
			xop = MAC_GETXREG_UNSIGNED(xop);
			yop = MAC_GETYREG_UNSIGNED(yop);
			temp = (xop * yop) << shift;
			res = A.m_core.mr.mr - (int64_t)temp;
			break;
		default:
			res = 0;    // just to keep the compiler happy
			break;
	}

	// set the final value
	temp = BIT(res, 31, 9);
	CLR_MV;
	if (temp != 0x000 && temp != 0x1ff) SET_MV;
	A.m_core.mr.mr = res;
}



/*===========================================================================
    MAC operations (result in MR, yop == xop)
===========================================================================*/

static void mac_op_mr_xop(uint32_t op)
{
	int8_t shift = (A.m_mstat & MSTAT_INTEGER) ? 0 : 1;
	int32_t xop = BIT(op, 8, 3);
	int32_t temp;
	int64_t res;

	switch (BIT(op, 13, 4))  //JB
	{
		case 0x00:
			// no-op
			return;
		case 0x01:
			// X * Y (RND)
			xop = MAC_GETXREG_SIGNED(xop);
			temp = (xop * xop) << shift;
			res = (int64_t)temp;
#if 0
			if ((res & 0xffff) == 0x8000) res &= ~((uint64_t)0x10000);
			else res += (res & 0x8000) << 1;
#else
			temp &= 0xffff;
			res += 0x8000;
			if ( temp == 0x8000 )
				res &= ~((uint64_t)0x10000);
#endif
			break;
		case 0x02:
			// MR + X * Y (RND)
			xop = MAC_GETXREG_SIGNED(xop);
			temp = (xop * xop) << shift;
			res = A.m_core.mr.mr + (int64_t)temp;
#if 0
			if ((res & 0xffff) == 0x8000) res &= ~((uint64_t)0x10000);
			else res += (res & 0x8000) << 1;
#else
			temp &= 0xffff;
			res += 0x8000;
			if ( temp == 0x8000 )
				res &= ~((uint64_t)0x10000);
#endif
			break;
		case 0x03:
			// MR - X * Y (RND)
			xop = MAC_GETXREG_SIGNED(xop);
			temp = (xop * xop) << shift;
			res = A.m_core.mr.mr - (int64_t)temp;
#if 0
			if ((res & 0xffff) == 0x8000) res &= ~((uint64_t)0x10000);
			else res += (res & 0x8000) << 1;
#else
			temp &= 0xffff;
			res += 0x8000;
			if ( temp == 0x8000 )
				res &= ~((uint64_t)0x10000);
#endif
			break;
		case 0x04:
			// X * Y (SS)       Clear when y = 0
			xop = MAC_GETXREG_SIGNED(xop);
			temp = (xop * xop) << shift;
			res = (int64_t)temp;
			break;
		case 0x05:
			// X * Y (SU)
			xop = MAC_GETXREG_SIGNED(xop);
			temp = (xop * xop) << shift;
			res = (int64_t)temp;
			break;
		case 0x06:
			// X * Y (US)
			xop = MAC_GETXREG_UNSIGNED(xop);
			temp = (xop * xop) << shift;
			res = (int64_t)temp;
			break;
		case 0x07:
			// X * Y (UU)
			xop = MAC_GETXREG_UNSIGNED(xop);
			temp = (xop * xop) << shift;
			res = (int64_t)temp;
			break;
		case 0x08:
			// MR + X * Y (SS)
			xop = MAC_GETXREG_SIGNED(xop);
			temp = (xop * xop) << shift;
			res = A.m_core.mr.mr + (int64_t)temp;
			break;
		case 0x09:
			// MR + X * Y (SU)
			xop = MAC_GETXREG_SIGNED(xop);
			temp = (xop * xop) << shift;
			res = A.m_core.mr.mr + (int64_t)temp;
			break;
		case 0x0a:
			// MR + X * Y (US)
			xop = MAC_GETXREG_UNSIGNED(xop);
			temp = (xop * xop) << shift;
			res = A.m_core.mr.mr + (int64_t)temp;
			break;
		case 0x0b:
			// MR + X * Y (UU)
			xop = MAC_GETXREG_UNSIGNED(xop);
			temp = (xop * xop) << shift;
			res = A.m_core.mr.mr + (int64_t)temp;
			break;
		case 0x0c:
			// MR - X * Y (SS)
			xop = MAC_GETXREG_SIGNED(xop);
			temp = (xop * xop) << shift;
			res = A.m_core.mr.mr - (int64_t)temp;
			break;
		case 0x0d:
			// MR - X * Y (SU)
			xop = MAC_GETXREG_SIGNED(xop);
			temp = (xop * xop) << shift;
			res = A.m_core.mr.mr - (int64_t)temp;
			break;
		case 0x0e:
			// MR - X * Y (US)
			xop = MAC_GETXREG_UNSIGNED(xop);
			temp = (xop * xop) << shift;
			res = A.m_core.mr.mr - (int64_t)temp;
			break;
		case 0x0f:
			// MR - X * Y (UU)
			xop = MAC_GETXREG_UNSIGNED(xop);
			temp = (xop * xop) << shift;
			res = A.m_core.mr.mr - (int64_t)temp;
			break;
		default:
			res = 0;    // just to keep the compiler happy
			break;
	}

	// set the final value
	temp = BIT(res, 31, 9);
	CLR_MV;
	if (temp != 0x000 && temp != 0x1ff) SET_MV;
	A.m_core.mr.mr = res;
}



/*===========================================================================
    MAC operations (result in MF)
===========================================================================*/

static AINLINE void mac_op_mf_k(uint32_t op, const int kind)
{
	int8_t shift = (A.m_mstat & MSTAT_INTEGER) ? 0 : 1;
	int32_t xop = BIT(op, 8, 3);
	int32_t yop = BIT(op, 11, 2);
	int32_t temp;
	int64_t res;

	switch (kind)  //JB
	{
		case 0x00:
			// no-op
			return;
		case 0x01:
			// X * Y (RND)
			xop = MAC_GETXREG_SIGNED(xop);
			yop = MAC_GETYREG_SIGNED(yop);
			temp = (xop * yop) << shift;
			res = (int64_t)temp;
#if 0
			if ((res & 0xffff) == 0x8000) res &= ~((uint64_t)0x10000);
			else res += (res & 0x8000) << 1;
#else
			temp &= 0xffff;
			res += 0x8000;
			if ( temp == 0x8000 )
				res &= ~((uint64_t)0x10000);
#endif
			break;
		case 0x02:
			// MR + X * Y (RND)
			xop = MAC_GETXREG_SIGNED(xop);
			yop = MAC_GETYREG_SIGNED(yop);
			temp = (xop * yop) << shift;
			res = A.m_core.mr.mr + (int64_t)temp;
#if 0
			if ((res & 0xffff) == 0x8000) res &= ~((uint64_t)0x10000);
			else res += (res & 0x8000) << 1;
#else
			temp &= 0xffff;
			res += 0x8000;
			if ( temp == 0x8000 )
				res &= ~((uint64_t)0x10000);
#endif
			break;
		case 0x03:
			// MR - X * Y (RND)
			xop = MAC_GETXREG_SIGNED(xop);
			yop = MAC_GETYREG_SIGNED(yop);
			temp = (xop * yop) << shift;
			res = A.m_core.mr.mr - (int64_t)temp;
#if 0
			if ((res & 0xffff) == 0x8000) res &= ~((uint64_t)0x10000);
			else res += (res & 0x8000) << 1;
#else
			temp &= 0xffff;
			res += 0x8000;
			if ( temp == 0x8000 )
				res &= ~((uint64_t)0x10000);
#endif
			break;
		case 0x04:
			// X * Y (SS)       Clear when y = 0
			xop = MAC_GETXREG_SIGNED(xop);
			yop = MAC_GETYREG_SIGNED(yop);
			temp = (xop * yop) << shift;
			res = (int64_t)temp;
			break;
		case 0x05:
			// X * Y (SU)
			xop = MAC_GETXREG_SIGNED(xop);
			yop = MAC_GETYREG_UNSIGNED(yop);
			temp = (xop * yop) << shift;
			res = (int64_t)temp;
			break;
		case 0x06:
			// X * Y (US)
			xop = MAC_GETXREG_UNSIGNED(xop);
			yop = MAC_GETYREG_SIGNED(yop);
			temp = (xop * yop) << shift;
			res = (int64_t)temp;
			break;
		case 0x07:
			// X * Y (UU)
			xop = MAC_GETXREG_UNSIGNED(xop);
			yop = MAC_GETYREG_UNSIGNED(yop);
			temp = (xop * yop) << shift;
			res = (int64_t)temp;
			break;
		case 0x08:
			// MR + X * Y (SS)
			xop = MAC_GETXREG_SIGNED(xop);
			yop = MAC_GETYREG_SIGNED(yop);
			temp = (xop * yop) << shift;
			res = A.m_core.mr.mr + (int64_t)temp;
			break;
		case 0x09:
			// MR + X * Y (SU)
			xop = MAC_GETXREG_SIGNED(xop);
			yop = MAC_GETYREG_UNSIGNED(yop);
			temp = (xop * yop) << shift;
			res = A.m_core.mr.mr + (int64_t)temp;
			break;
		case 0x0a:
			// MR + X * Y (US)
			xop = MAC_GETXREG_UNSIGNED(xop);
			yop = MAC_GETYREG_SIGNED(yop);
			temp = (xop * yop) << shift;
			res = A.m_core.mr.mr + (int64_t)temp;
			break;
		case 0x0b:
			// MR + X * Y (UU)
			xop = MAC_GETXREG_UNSIGNED(xop);
			yop = MAC_GETYREG_UNSIGNED(yop);
			temp = (xop * yop) << shift;
			res = A.m_core.mr.mr + (int64_t)temp;
			break;
		case 0x0c:
			// MR - X * Y (SS)
			xop = MAC_GETXREG_SIGNED(xop);
			yop = MAC_GETYREG_SIGNED(yop);
			temp = (xop * yop) << shift;
			res = A.m_core.mr.mr - (int64_t)temp;
			break;
		case 0x0d:
			// MR - X * Y (SU)
			xop = MAC_GETXREG_SIGNED(xop);
			yop = MAC_GETYREG_UNSIGNED(yop);
			temp = (xop * yop) << shift;
			res = A.m_core.mr.mr - (int64_t)temp;
			break;
		case 0x0e:
			// MR - X * Y (US)
			xop = MAC_GETXREG_UNSIGNED(xop);
			yop = MAC_GETYREG_SIGNED(yop);
			temp = (xop * yop) << shift;
			res = A.m_core.mr.mr - (int64_t)temp;
			break;
		case 0x0f:
			// MR - X * Y (UU)
			xop = MAC_GETXREG_UNSIGNED(xop);
			yop = MAC_GETYREG_UNSIGNED(yop);
			temp = (xop * yop) << shift;
			res = A.m_core.mr.mr - (int64_t)temp;
			break;
		default:
			res = 0;    // just to keep the compiler happy
			break;
	}

	// set the final value
	A.m_core.mf.u = (uint32_t)res >> 16;
}



/*===========================================================================
    MAC operations (result in MF, yop == xop)
===========================================================================*/

static void mac_op_mf_xop(uint32_t op)
{
	int8_t shift = (A.m_mstat & MSTAT_INTEGER) ? 0 : 1;
	int32_t xop = BIT(op, 8, 3);
	int32_t temp;
	int64_t res;

	switch (BIT(op, 13, 4))  //JB
	{
		case 0x00:
			// no-op
			return;
		case 0x01:
			// X * Y (RND)
			xop = MAC_GETXREG_SIGNED(xop);
			temp = (xop * xop) << shift;
			res = (int64_t)temp;
#if 0
			if ((res & 0xffff) == 0x8000) res &= ~((uint64_t)0x10000);
			else res += (res & 0x8000) << 1;
#else
			temp &= 0xffff;
			res += 0x8000;
			if ( temp == 0x8000 )
				res &= ~((uint64_t)0x10000);
#endif
			break;
		case 0x02:
			// MR + X * Y (RND)
			xop = MAC_GETXREG_SIGNED(xop);
			temp = (xop * xop) << shift;
			res = A.m_core.mr.mr + (int64_t)temp;
#if 0
			if ((res & 0xffff) == 0x8000) res &= ~((uint64_t)0x10000);
			else res += (res & 0x8000) << 1;
#else
			temp &= 0xffff;
			res += 0x8000;
			if ( temp == 0x8000 )
				res &= ~((uint64_t)0x10000);
#endif
			break;
		case 0x03:
			// MR - X * Y (RND)
			xop = MAC_GETXREG_SIGNED(xop);
			temp = (xop * xop) << shift;
			res = A.m_core.mr.mr - (int64_t)temp;
#if 0
			if ((res & 0xffff) == 0x8000) res &= ~((uint64_t)0x10000);
			else res += (res & 0x8000) << 1;
#else
			temp &= 0xffff;
			res += 0x8000;
			if ( temp == 0x8000 )
				res &= ~((uint64_t)0x10000);
#endif
			break;
		case 0x04:
			// X * Y (SS)       Clear when y = 0
			xop = MAC_GETXREG_SIGNED(xop);
			temp = (xop * xop) << shift;
			res = (int64_t)temp;
			break;
		case 0x05:
			// X * Y (SU)
			xop = MAC_GETXREG_SIGNED(xop);
			temp = (xop * xop) << shift;
			res = (int64_t)temp;
			break;
		case 0x06:
			// X * Y (US)
			xop = MAC_GETXREG_UNSIGNED(xop);
			temp = (xop * xop) << shift;
			res = (int64_t)temp;
			break;
		case 0x07:
			// X * Y (UU)
			xop = MAC_GETXREG_UNSIGNED(xop);
			temp = (xop * xop) << shift;
			res = (int64_t)temp;
			break;
		case 0x08:
			// MR + X * Y (SS)
			xop = MAC_GETXREG_SIGNED(xop);
			temp = (xop * xop) << shift;
			res = A.m_core.mr.mr + (int64_t)temp;
			break;
		case 0x09:
			// MR + X * Y (SU)
			xop = MAC_GETXREG_SIGNED(xop);
			temp = (xop * xop) << shift;
			res = A.m_core.mr.mr + (int64_t)temp;
			break;
		case 0x0a:
			// MR + X * Y (US)
			xop = MAC_GETXREG_UNSIGNED(xop);
			temp = (xop * xop) << shift;
			res = A.m_core.mr.mr + (int64_t)temp;
			break;
		case 0x0b:
			// MR + X * Y (UU)
			xop = MAC_GETXREG_UNSIGNED(xop);
			temp = (xop * xop) << shift;
			res = A.m_core.mr.mr + (int64_t)temp;
			break;
		case 0x0c:
			// MR - X * Y (SS)
			xop = MAC_GETXREG_SIGNED(xop);
			temp = (xop * xop) << shift;
			res = A.m_core.mr.mr - (int64_t)temp;
			break;
		case 0x0d:
			// MR - X * Y (SU)
			xop = MAC_GETXREG_SIGNED(xop);
			temp = (xop * xop) << shift;
			res = A.m_core.mr.mr - (int64_t)temp;
			break;
		case 0x0e:
			// MR - X * Y (US)
			xop = MAC_GETXREG_UNSIGNED(xop);
			temp = (xop * xop) << shift;
			res = A.m_core.mr.mr - (int64_t)temp;
			break;
		case 0x0f:
			// MR - X * Y (UU)
			xop = MAC_GETXREG_UNSIGNED(xop);
			temp = (xop * xop) << shift;
			res = A.m_core.mr.mr - (int64_t)temp;
			break;
		default:
			res = 0;    // just to keep the compiler happy
			break;
	}

	// set the final value
	A.m_core.mf.u = (uint32_t)res >> 16;
}



/*===========================================================================
    SHIFT operations (result in SR/SE/SB)
===========================================================================*/

static void alu_op_ar(uint32_t op) { alu_op_ar_k(op, BIT(op, 13, 4)); }
static void alu_op_af(uint32_t op) { alu_op_af_k(op, BIT(op, 13, 4)); }
static void mac_op_mr(uint32_t op) { mac_op_mr_k(op, BIT(op, 13, 4)); }
static void mac_op_mf(uint32_t op) { mac_op_mf_k(op, BIT(op, 13, 4)); }

static void shift_op(uint32_t op)
{
	int8_t sc = A.m_core.se.s;
	int32_t xop = BIT(op, 8, 3);
	uint32_t res;

	switch (BIT(op, 11, 4))  //JB
	{
		case 0x00:
			// LSHIFT (HI)
			xop = SHIFT_GETXREG_UNSIGNED(xop) << 16;
			if (sc > 0) res = (sc < 32) ? (xop << sc) : 0;
			else res = (sc > -32) ? ((uint32_t)xop >> -sc) : 0;
			A.m_core.sr.sr = res;
			break;
		case 0x01:
			// LSHIFT (HI, OR)
			xop = SHIFT_GETXREG_UNSIGNED(xop) << 16;
			if (sc > 0) res = (sc < 32) ? (xop << sc) : 0;
			else res = (sc > -32) ? ((uint32_t)xop >> -sc) : 0;
			A.m_core.sr.sr |= res;
			break;
		case 0x02:
			// LSHIFT (LO)
			xop = SHIFT_GETXREG_UNSIGNED(xop);
			if (sc > 0) res = (sc < 32) ? (xop << sc) : 0;
			else res = (sc > -32) ? (xop >> -sc) : 0;
			A.m_core.sr.sr = res;
			break;
		case 0x03:
			// LSHIFT (LO, OR)
			xop = SHIFT_GETXREG_UNSIGNED(xop);
			if (sc > 0) res = (sc < 32) ? (xop << sc) : 0;
			else res = (sc > -32) ? (xop >> -sc) : 0;
			A.m_core.sr.sr |= res;
			break;
		case 0x04:
			// ASHIFT (HI)
			xop = SHIFT_GETXREG_SIGNED(xop) << 16;
			if (sc > 0) res = (sc < 32) ? (xop << sc) : 0;
			else res = (sc > -32) ? (xop >> -sc) : (xop >> 31);
			A.m_core.sr.sr = res;
			break;
		case 0x05:
			// ASHIFT (HI, OR)
			xop = SHIFT_GETXREG_SIGNED(xop) << 16;
			if (sc > 0) res = (sc < 32) ? (xop << sc) : 0;
			else res = (sc > -32) ? (xop >> -sc) : (xop >> 31);
			A.m_core.sr.sr |= res;
			break;
		case 0x06:
			// ASHIFT (LO)
			xop = SHIFT_GETXREG_SIGNED(xop);
			if (sc > 0) res = (sc < 32) ? (xop << sc) : 0;
			else res = (sc > -32) ? (xop >> -sc) : (xop >> 31);
			A.m_core.sr.sr = res;
			break;
		case 0x07:
			// ASHIFT (LO, OR)
			xop = SHIFT_GETXREG_SIGNED(xop);
			if (sc > 0) res = (sc < 32) ? (xop << sc) : 0;
			else res = (sc > -32) ? (xop >> -sc) : (xop >> 31);
			A.m_core.sr.sr |= res;
			break;
		case 0x08:
			// NORM (HI)
			xop = SHIFT_GETXREG_SIGNED(xop) << 16;
			if (sc > 0)
			{
				xop = ((uint32_t)xop >> 1) | ((A.m_astat & CFLAG) << 28);
				res = xop >> (sc - 1);
			}
			else res = (sc > -32) ? (xop << -sc) : 0;
			A.m_core.sr.sr = res;
			break;
		case 0x09:
			// NORM (HI, OR)
			xop = SHIFT_GETXREG_SIGNED(xop) << 16;
			if (sc > 0)
			{
				xop = ((uint32_t)xop >> 1) | ((A.m_astat & CFLAG) << 28);
				res = xop >> (sc - 1);
			}
			else res = (sc > -32) ? (xop << -sc) : 0;
			A.m_core.sr.sr |= res;
			break;
		case 0x0a:
			// NORM (LO)
			xop = SHIFT_GETXREG_UNSIGNED(xop);
			if (sc > 0) res = (sc < 32) ? (xop >> sc) : 0;
			else res = (sc > -32) ? (xop << -sc) : 0;
			A.m_core.sr.sr = res;
			break;
		case 0x0b:
			// NORM (LO, OR)
			xop = SHIFT_GETXREG_UNSIGNED(xop);
			if (sc > 0) res = (sc < 32) ? (xop >> sc) : 0;
			else res = (sc > -32) ? (xop << -sc) : 0;
			A.m_core.sr.sr |= res;
			break;
		case 0x0c:
			// EXP (HI)
			xop = SHIFT_GETXREG_SIGNED(xop);
			if (xop < 0)
			{
				SET_SS;
				res = clo32((uint32_t)(xop)) - 16 - 1;
			}
			else
			{
				CLR_SS;
				res = clz32((uint32_t)(xop)) - 16 - 1;
			}
			A.m_core.se.s = -res;
			break;
		case 0x0d:
			// EXP (HIX)
			xop = SHIFT_GETXREG_SIGNED(xop);
			if (GET_V)
			{
				A.m_core.se.s = 1;
				if (xop < 0) CLR_SS;
				else SET_SS;
			}
			else
			{
				if (xop < 0)
				{
					SET_SS;
					res = clo32((uint32_t)(xop)) - 16 - 1;
				}
				else
				{
					CLR_SS;
					res = clz32((uint32_t)(xop)) - 16 - 1;
				}
				A.m_core.se.s = -res;
			}
			break;
		case 0x0e:
			// EXP (LO)
			if (A.m_core.se.s == -15)
			{
				xop = SHIFT_GETXREG_SIGNED(xop);
				res = clz32((uint32_t)((GET_SS ? ~xop : xop) & 0xffff)) - 1;
				A.m_core.se.s = -res;
			}
			break;
		case 0x0f:
			// EXPADJ
			xop = SHIFT_GETXREG_SIGNED(xop);
			res = clz32((uint32_t)((xop < 0) ? ~xop : xop)) - 16 - 1;
			if (res < -A.m_core.sb.s)
				A.m_core.sb.s = -res;
			break;
	}
}



/*===========================================================================
    Immediate SHIFT operations (result in SR/SE/SB)
===========================================================================*/

static void shift_op_imm(uint32_t op)
{
	int8_t sc = (int8_t)op;
	int32_t xop = BIT(op, 8, 3);
	uint32_t res;

	switch (BIT(op, 11, 4))  //JB
	{
		case 0x00:
			// LSHIFT (HI)
			xop = SHIFT_GETXREG_UNSIGNED(xop) << 16;
			if (sc > 0) res = (sc < 32) ? (xop << sc) : 0;
			else res = (sc > -32) ? ((uint32_t)xop >> -sc) : 0;
			A.m_core.sr.sr = res;
			break;
		case 0x01:
			// LSHIFT (HI, OR)
			xop = SHIFT_GETXREG_UNSIGNED(xop) << 16;
			if (sc > 0) res = (sc < 32) ? (xop << sc) : 0;
			else res = (sc > -32) ? ((uint32_t)xop >> -sc) : 0;
			A.m_core.sr.sr |= res;
			break;
		case 0x02:
			// LSHIFT (LO)
			xop = SHIFT_GETXREG_UNSIGNED(xop);
			if (sc > 0) res = (sc < 32) ? (xop << sc) : 0;
			else res = (sc > -32) ? (xop >> -sc) : 0;
			A.m_core.sr.sr = res;
			break;
		case 0x03:
			// LSHIFT (LO, OR)
			xop = SHIFT_GETXREG_UNSIGNED(xop);
			if (sc > 0) res = (sc < 32) ? (xop << sc) : 0;
			else res = (sc > -32) ? (xop >> -sc) : 0;
			A.m_core.sr.sr |= res;
			break;
		case 0x04:
			// ASHIFT (HI)
			xop = SHIFT_GETXREG_SIGNED(xop) << 16;
			if (sc > 0) res = (sc < 32) ? (xop << sc) : 0;
			else res = (sc > -32) ? (xop >> -sc) : (xop >> 31);
			A.m_core.sr.sr = res;
			break;
		case 0x05:
			// ASHIFT (HI, OR)
			xop = SHIFT_GETXREG_SIGNED(xop) << 16;
			if (sc > 0) res = (sc < 32) ? (xop << sc) : 0;
			else res = (sc > -32) ? (xop >> -sc) : (xop >> 31);
			A.m_core.sr.sr |= res;
			break;
		case 0x06:
			// ASHIFT (LO)
			xop = SHIFT_GETXREG_SIGNED(xop);
			if (sc > 0) res = (sc < 32) ? (xop << sc) : 0;
			else res = (sc > -32) ? (xop >> -sc) : (xop >> 31);
			A.m_core.sr.sr = res;
			break;
		case 0x07:
			// ASHIFT (LO, OR)
			xop = SHIFT_GETXREG_SIGNED(xop);
			if (sc > 0) res = (sc < 32) ? (xop << sc) : 0;
			else res = (sc > -32) ? (xop >> -sc) : (xop >> 31);
			A.m_core.sr.sr |= res;
			break;
		case 0x08:
			// NORM (HI)
			xop = SHIFT_GETXREG_SIGNED(xop) << 16;
			if (sc > 0)
			{
				xop = ((uint32_t)xop >> 1) | ((A.m_astat & CFLAG) << 28);
				res = xop >> (sc - 1);
			}
			else res = (sc > -32) ? (xop << -sc) : 0;
			A.m_core.sr.sr = res;
			break;
		case 0x09:
			// NORM (HI, OR)
			xop = SHIFT_GETXREG_SIGNED(xop) << 16;
			if (sc > 0)
			{
				xop = ((uint32_t)xop >> 1) | ((A.m_astat & CFLAG) << 28);
				res = xop >> (sc - 1);
			}
			else res = (sc > -32) ? (xop << -sc) : 0;
			A.m_core.sr.sr |= res;
			break;
		case 0x0a:
			// NORM (LO)
			xop = SHIFT_GETXREG_UNSIGNED(xop);
			if (sc > 0) res = (sc < 32) ? (xop >> sc) : 0;
			else res = (sc > -32) ? (xop << -sc) : 0;
			A.m_core.sr.sr = res;
			break;
		case 0x0b:
			// NORM (LO, OR)
			xop = SHIFT_GETXREG_UNSIGNED(xop);
			if (sc > 0) res = (sc < 32) ? (xop >> sc) : 0;
			else res = (sc > -32) ? (xop << -sc) : 0;
			A.m_core.sr.sr |= res;
			break;
	}
}


static bool generate_irq(int which, int indx)
{
	// skip if masked
	if (!(A.m_imask & (0x20 >> indx)))
		return false;

	// clear the latch
	A.m_irq_latch[which] = 0;

	// push the PC and the status
	pc_stack_push();
	stat_stack_push();

	// vector to location & stop idling
	A.m_pc = 0x04 + indx * 4;
	A.m_idle = 0;

	// mask other interrupts based on the nesting bit
	if (A.m_icntl & 0x10) A.m_imask &= ~(0x3f >> indx);
	else A.m_imask &= ~0x3f;

	return true;
}



static void check_irqs()
{
	uint8_t check;

	// check IRQ2
	check = (A.m_icntl & 4) ? A.m_irq_latch[ADSP2101_IRQ2] : A.m_irq_state[ADSP2101_IRQ2];
	if (check && generate_irq(ADSP2101_IRQ2, 0))
		return;

	// check SPORT0 transmit
	check = A.m_irq_latch[ADSP2101_SPORT0_TX];
	if (check && generate_irq(ADSP2101_SPORT0_TX, 1))
		return;

	// check SPORT0 receive
	check = A.m_irq_latch[ADSP2101_SPORT0_RX];
	if (check && generate_irq(ADSP2101_SPORT0_RX, 2))
		return;

	// check IRQ1/SPORT1 transmit
	check = (A.m_icntl & 2) ? A.m_irq_latch[ADSP2101_IRQ1] : A.m_irq_state[ADSP2101_IRQ1];
	if (check && generate_irq(ADSP2101_IRQ1, 3))
		return;

	// check IRQ0/SPORT1 receive
	check = (A.m_icntl & 1) ? A.m_irq_latch[ADSP2101_IRQ0] : A.m_irq_state[ADSP2101_IRQ0];
	if (check && generate_irq(ADSP2101_IRQ0, 4))
		return;

	// check timer
	check = A.m_irq_latch[ADSP2101_TIMER];
	if (check && generate_irq(ADSP2101_TIMER, 5))
		return;
}


static void create_tables()
{
	// initialize the bit reversing table
	for (int i = 0; i < 0x4000; i++)
	{
		uint16_t data = 0;

		data |= (i >> 13) & 0x0001;
		data |= (i >> 11) & 0x0002;
		data |= (i >> 9)  & 0x0004;
		data |= (i >> 7)  & 0x0008;
		data |= (i >> 5)  & 0x0010;
		data |= (i >> 3)  & 0x0020;
		data |= (i >> 1)  & 0x0040;
		data |= (i << 1)  & 0x0080;
		data |= (i << 3)  & 0x0100;
		data |= (i << 5)  & 0x0200;
		data |= (i << 7)  & 0x0400;
		data |= (i << 9)  & 0x0800;
		data |= (i << 11) & 0x1000;
		data |= (i << 13) & 0x2000;

		T.m_reverse_table[i] = data;
	}

	// initialize the mask table
	for (int i = 0; i < 0x4000; i++)
	{
		if (i > 0x2000)      T.m_mask_table[i] = 0x0000;
		else if (i > 0x1000) T.m_mask_table[i] = 0x2000;
		else if (i > 0x0800) T.m_mask_table[i] = 0x3000;
		else if (i > 0x0400) T.m_mask_table[i] = 0x3800;
		else if (i > 0x0200) T.m_mask_table[i] = 0x3c00;
		else if (i > 0x0100) T.m_mask_table[i] = 0x3e00;
		else if (i > 0x0080) T.m_mask_table[i] = 0x3f00;
		else if (i > 0x0040) T.m_mask_table[i] = 0x3f80;
		else if (i > 0x0020) T.m_mask_table[i] = 0x3fc0;
		else if (i > 0x0010) T.m_mask_table[i] = 0x3fe0;
		else if (i > 0x0008) T.m_mask_table[i] = 0x3ff0;
		else if (i > 0x0004) T.m_mask_table[i] = 0x3ff8;
		else if (i > 0x0002) T.m_mask_table[i] = 0x3ffc;
		else if (i > 0x0001) T.m_mask_table[i] = 0x3ffe;
		else                 T.m_mask_table[i] = 0x3fff;
	}

	// initialize the condition table
	for (int i = 0; i < 0x100; i++)
	{
		int az = ((i & ZFLAG) != 0);
		int an = ((i & NFLAG) != 0);
		int av = ((i & VFLAG) != 0);
		int ac = ((i & CFLAG) != 0);
		int mv = ((i & MVFLAG) != 0);
		int as = ((i & SFLAG) != 0);

		T.m_condition_table[i | 0x000] = az;
		T.m_condition_table[i | 0x100] = !az;
		T.m_condition_table[i | 0x200] = !((an ^ av) | az);
		T.m_condition_table[i | 0x300] = (an ^ av) | az;
		T.m_condition_table[i | 0x400] = an ^ av;
		T.m_condition_table[i | 0x500] = !(an ^ av);
		T.m_condition_table[i | 0x600] = av;
		T.m_condition_table[i | 0x700] = !av;
		T.m_condition_table[i | 0x800] = ac;
		T.m_condition_table[i | 0x900] = !ac;
		T.m_condition_table[i | 0xa00] = as;
		T.m_condition_table[i | 0xb00] = !as;
		T.m_condition_table[i | 0xc00] = mv;
		T.m_condition_table[i | 0xd00] = !mv;
		T.m_condition_table[i | 0xf00] = 1;
	}
}



/***************************************************************************
    CORE EXECUTION LOOP
***************************************************************************/

//-------------------------------------------------
//  execute_min_cycles - return minimum number of
//  cycles it takes for one instruction to execute
//-------------------------------------------------

/*  Polling loop skip (ReBOX, results identical to the interpreter; RBX_ADSP_NOSPIN=1 = off). The DCS programs wait
    for the autobuffer's I register in a 3-instruction loop:  s: dst = reg (internal data move);  s+1: AF = x op y
    (unconditional ALU to AF);  s+2: IF cond JUMP s.  Within one adsp_execute nothing outside the core changes (the
    timers, the autobuffer's I register and the interrupts are the board's, between the calls), so once an iteration
    leaves the registers it touches as they were (a fixed point: dst already holds the moved value, AF and ASTAT what
    the ALU makes again) and its jump is taken, every following iteration does exactly the same. The check runs one
    real iteration (the move and the ALU op on the core, the jump's condition), then the remaining whole iterations of
    the slice are counted off icount only; the last 1-3 cycles run normally, so the slice ends at the same pc. */
static int s_spin_on = -1;
static uint64_t s_spin_skipped;
static uint64_t s_ran_cyc, s_skip_cyc;   // (adsp_insns: the cycles run, those of them not executed: idle, loops skipped)
static int adsp_spin(uint32_t s)
{
	const uint32_t op1 = program_read(s), op2 = program_read(s + 1), op3 = program_read(s + 2);
	// the move: dst = reg0 / reg1 / reg2 (pure reads), its source not the destination (or MR2 with MR1)
	if ((op1 & 0xfff000) != 0x0d0000 || BIT(op1, 8, 4) > 2)
		return 0;
	const int grp = BIT(op1, 8, 4), dst = BIT(op1, 4, 4), src = BIT(op1, 0, 4);
	if (grp == 0 && (src == dst || (dst == 0x0c && src == 0x0d)))
		return 0;
	// the ALU op: to AF, unconditional, no carry in (X+Y+C, X-Y+C-1, Y-X+C-1), Y not AF
	const int aop = BIT(op2, 13, 4);
	if ((op2 >> 17) != 0x13 || (op2 & 0x0f) != 0x0f || aop == 2 || aop == 6 || aop == 0x0a || BIT(op2, 11, 2) == 2)
		return 0;
	// the jump back to s, not on the counter
	if ((op3 >> 18) != 0x06 || BIT(op3, 4, 14) != s || (op3 & 0x0f) == 14)
		return 0;
	if (A.m_loop >= s && A.m_loop <= s + 2)
		return 0;
	adsp_core core;
	memcpy(&core, &A.m_core, sizeof(core));
	const uint32_t astat = A.m_astat;
	write_reg0(dst, grp == 0 ? read_reg0(src) : grp == 1 ? read_reg1(src) : read_reg2(src));
	alu_op_af(op2);
	if (memcmp(&core, &A.m_core, sizeof(core)) || astat != A.m_astat || !condition(op3 & 0x0f)) {
		memcpy(&A.m_core, &core, sizeof(core));   // (not a fixed point: undone, the interpreter goes on)
		A.m_astat = astat;
		return 0;
	}
	return 1;
}
static void spin_stat(void)   // (RBX_ADSP_SPINSTAT=1)
{
	fprintf(stderr, "adsp: %llu polling loop iterations skipped\n", (unsigned long long)s_spin_skipped);
}

static void execute_set_input(int inputnum, int state)
{
	// update the latched state
	if (state != CLEAR_LINE && A.m_irq_state[inputnum] == CLEAR_LINE)
		A.m_irq_latch[inputnum] = 1;

	// update the absolute state
	A.m_irq_state[inputnum] = state;
}


static AINLINE void exec_op(uint32_t op)
{
	uint32_t temp;
	switch (BIT(op, 16, 8))
	{
		case 0x00:
			// 00000000 00000000 00000000  NOP
			break;
		case 0x01:
			// 00000001 0xxxxxxx xxxxxxxx  dst = IO(x)
			// 00000001 1xxxxxxx xxxxxxxx  IO(x) = dst
			// ADSP-218x only
			if (ADSP_CHIP >= CHIP_TYPE_ADSP2181)
			{
				if (!BIT(op, 15))
					write_reg0(BIT(op, 0, 4), io_read(BIT(op, 4, 11)));
				else
					io_write(BIT(op, 4, 11), read_reg0(BIT(op, 0, 4)));
			}
			break;
		case 0x02:
			// 00000010 0000xxxx xxxxxxxx  modify flag out
			// 00000010 10000000 00000000  idle
			// 00000010 10000000 0000xxxx  idle (n)
			if (BIT(op, 15))
			{
				A.m_idle = 1;
				s_skip_cyc += (uint64_t)(A.m_icount > 0 ? A.m_icount : 0);
				A.m_icount = 0;
			}
			else
			{
				if (condition(BIT(op, 0, 4)))
				{
					if (BIT(op, 5)) A.m_flagout = 0;
					if (BIT(op, 4)) A.m_flagout ^= 1;
					if (ADSP_CHIP >= CHIP_TYPE_ADSP2101)
					{
						if (BIT(op, 7)) A.m_fl0 = 0;
						if (BIT(op, 6)) A.m_fl0 ^= 1;
						if (BIT(op, 9)) A.m_fl1 = 0;
						if (BIT(op, 8)) A.m_fl1 ^= 1;
						if (BIT(op, 11)) A.m_fl2 = 0;
						if (BIT(op, 10)) A.m_fl2 ^= 1;
					}
				}
			}
			break;
		case 0x03:
			// 00000011 xxxxxxxx xxxxxxxx  call or jump on flag in
			if (BIT(op, 1) ? A.m_flagin : !A.m_flagin)
			{
				if (BIT(op, 0))
					pc_stack_push();
				A.m_pc = BIT(op, 4, 12) | BIT(op, 2, 2) << 12;
			}
			break;
		case 0x04:
			// 00000100 00000000 000xxxxx  stack control
			if (BIT(op, 4)) pc_stack_pop_val();
			if (BIT(op, 3)) loop_stack_pop();
			if (BIT(op, 2)) cntr_stack_pop();
			if (BIT(op, 1))
			{
				if (BIT(op, 0)) stat_stack_pop();
				else stat_stack_push();
			}
			break;
		case 0x05:
			// 00000101 00000000 00000000  saturate MR
			if (GET_MV)
			{
				if (A.m_core.mr.mrx.mr2.u & 0x80)
					A.m_core.mr.mrx.mr2.u = 0xffff, A.m_core.mr.mrx.mr1.u = 0x8000, A.m_core.mr.mrx.mr0.u = 0x0000;
				else
					A.m_core.mr.mrx.mr2.u = 0x0000, A.m_core.mr.mrx.mr1.u = 0x7fff, A.m_core.mr.mrx.mr0.u = 0xffff;
			}
			break;
		case 0x06:
			// 00000110 000xxxxx 00000000  DIVS
			{
				int xop = BIT(op, 8, 3);
				int yop = BIT(op, 11, 2);

				xop = ALU_GETXREG_UNSIGNED(xop);
				yop = ALU_GETYREG_UNSIGNED(yop);

				temp = xop ^ yop;
				A.m_astat = (A.m_astat & ~QFLAG) | ((temp >> 10) & QFLAG);
				A.m_core.af.u = (yop << 1) | (A.m_core.ay0.u >> 15);
				A.m_core.ay0.u = (A.m_core.ay0.u << 1) | (temp >> 15);
			}
			break;
		case 0x07:
			// 00000111 00010xxx 00000000  DIVQ
			{
				int xop = BIT(op, 8, 3);
				int res;

				xop = ALU_GETXREG_UNSIGNED(xop);

				if (GET_Q)
					res = A.m_core.af.u + xop;
				else
					res = A.m_core.af.u - xop;

				temp = res ^ xop;
				A.m_astat = (A.m_astat & ~QFLAG) | ((temp >> 10) & QFLAG);
				A.m_core.af.u = (res << 1) | (A.m_core.ay0.u >> 15);
				A.m_core.ay0.u = (A.m_core.ay0.u << 1) | ((~temp >> 15) & 0x0001);
			}
			break;
		case 0x08:
			// 00001000 00000000 0000xxxx  reserved
			break;
		case 0x09:
			// 00001001 00000000 000xxxxx  modify address register
			temp = BIT(op, 2, 3);
			modify_address(temp, (temp & 4) | (op & 3));
			break;
		case 0x0a:
			// 00001010 00000000 000xxxxx  conditional return
			if (condition(BIT(op, 0, 4)))
			{
				pc_stack_pop();

				// RTI case
				if (BIT(op, 4))
					stat_stack_pop();
			}
			break;
		case 0x0b:
			// 00001011 00000000 xxxxxxxx  conditional jump (indirect address)
			if (condition(BIT(op, 0, 4)))
			{
				if (BIT(op, 4))
					pc_stack_push();
				A.m_pc = A.m_i[4 + BIT(op, 6, 2)] & 0x3fff;
			}
			break;
		case 0x0c:
			// 00001100 xxxxxxxx xxxxxxxx  mode control
			if (ADSP_CHIP >= CHIP_TYPE_ADSP2101)
			{
				if (BIT(op, 3)) A.m_mstat = BIT(op, 2) ? (A.m_mstat | MSTAT_GOMODE) : (A.m_mstat & ~MSTAT_GOMODE);
				if (BIT(op, 13)) A.m_mstat = BIT(op, 12) ? (A.m_mstat | MSTAT_INTEGER) : (A.m_mstat & ~MSTAT_INTEGER);
				if (BIT(op, 15)) A.m_mstat = BIT(op, 14) ? (A.m_mstat | MSTAT_TIMER) : (A.m_mstat & ~MSTAT_TIMER);
			}
			if (BIT(op, 5)) A.m_mstat = BIT(op, 4) ? (A.m_mstat | MSTAT_BANK) : (A.m_mstat & ~MSTAT_BANK);
			if (BIT(op, 7)) A.m_mstat = BIT(op, 6) ? (A.m_mstat | MSTAT_REVERSE) : (A.m_mstat & ~MSTAT_REVERSE);
			if (BIT(op, 9)) A.m_mstat = BIT(op, 8) ? (A.m_mstat | MSTAT_STICKYV) : (A.m_mstat & ~MSTAT_STICKYV);
			if (BIT(op, 11)) A.m_mstat = BIT(op, 10) ? (A.m_mstat | MSTAT_SATURATE) : (A.m_mstat & ~MSTAT_SATURATE);
			update_mstat();
			break;
		case 0x0d:
			// 00001101 0000xxxx xxxxxxxx  internal data move
			switch (BIT(op, 8, 4))
			{
				case 0x00:  write_reg0(BIT(op, 4, 4), read_reg0(BIT(op, 0, 4))); break;
				case 0x01:  write_reg0(BIT(op, 4, 4), read_reg1(BIT(op, 0, 4))); break;
				case 0x02:  write_reg0(BIT(op, 4, 4), read_reg2(BIT(op, 0, 4))); break;
				case 0x03:  write_reg0(BIT(op, 4, 4), read_reg3(BIT(op, 0, 4))); break;
				case 0x04:  write_reg1(BIT(op, 4, 4), read_reg0(BIT(op, 0, 4))); break;
				case 0x05:  write_reg1(BIT(op, 4, 4), read_reg1(BIT(op, 0, 4))); break;
				case 0x06:  write_reg1(BIT(op, 4, 4), read_reg2(BIT(op, 0, 4))); break;
				case 0x07:  write_reg1(BIT(op, 4, 4), read_reg3(BIT(op, 0, 4))); break;
				case 0x08:  write_reg2(BIT(op, 4, 4), read_reg0(BIT(op, 0, 4))); break;
				case 0x09:  write_reg2(BIT(op, 4, 4), read_reg1(BIT(op, 0, 4))); break;
				case 0x0a:  write_reg2(BIT(op, 4, 4), read_reg2(BIT(op, 0, 4))); break;
				case 0x0b:  write_reg2(BIT(op, 4, 4), read_reg3(BIT(op, 0, 4))); break;
				case 0x0c:  write_reg3(BIT(op, 4, 4), read_reg0(BIT(op, 0, 4))); break;
				case 0x0d:  write_reg3(BIT(op, 4, 4), read_reg1(BIT(op, 0, 4))); break;
				case 0x0e:  write_reg3(BIT(op, 4, 4), read_reg2(BIT(op, 0, 4))); break;
				case 0x0f:  write_reg3(BIT(op, 4, 4), read_reg3(BIT(op, 0, 4))); break;
			}
			break;
		case 0x0e:
			// 00001110 0xxxxxxx xxxxxxxx  conditional shift
			if (condition(BIT(op, 0, 4))) shift_op(op);
			break;
		case 0x0f:
			// 00001111 0xxxxxxx xxxxxxxx  shift immediate
			shift_op_imm(op);
			break;
		case 0x10:
			// 00010000 0xxxxxxx xxxxxxxx  shift with internal data register move
			shift_op(op);
			temp = read_reg0(BIT(op, 0, 4));
			write_reg0(BIT(op, 4, 4), temp);
			break;
		case 0x11:
			// 00010001 xxxxxxxx xxxxxxxx  shift with pgm memory read/write
			if (BIT(op, 15))
			{
				pgm_write_dag2(op, read_reg0(BIT(op, 4, 4)));
				shift_op(op);
			}
			else
			{
				shift_op(op);
				write_reg0(BIT(op, 4, 4), pgm_read_dag2(op));
			}
			break;
		case 0x12:
			// 00010010 xxxxxxxx xxxxxxxx  shift with data memory read/write DAG1
			if (BIT(op, 15))
			{
				data_write_dag1(op, read_reg0(BIT(op, 4, 4)));
				shift_op(op);
			}
			else
			{
				shift_op(op);
				write_reg0(BIT(op, 4, 4), data_read_dag1(op));
			}
			break;
		case 0x13:
			// 00010011 xxxxxxxx xxxxxxxx  shift with data memory read/write DAG2
			if (BIT(op, 15))
			{
				data_write_dag2(op, read_reg0(BIT(op, 4, 4)));
				shift_op(op);
			}
			else
			{
				shift_op(op);
				write_reg0(BIT(op, 4, 4), data_read_dag2(op));
			}
			break;
		case 0x14: case 0x15: case 0x16: case 0x17:
			// 000101xx xxxxxxxx xxxxxxxx  do until
			loop_stack_push(op & 0x3ffff);
			pc_stack_push();
			break;
		case 0x18: case 0x19: case 0x1a: case 0x1b:
			// 000110xx xxxxxxxx xxxxxxxx  conditional jump (immediate addr)
			if (condition(BIT(op, 0, 4)))
			{
				A.m_pc = BIT(op, 4, 14);
				// check for a busy loop
				if (A.m_pc == A.m_ppc) {
					s_skip_cyc += (uint64_t)(A.m_icount > 0 ? A.m_icount : 0);
					A.m_icount = 0;
				}
				// (a polling loop of three: the whole iterations left in the slice skipped, adsp_spin)
				else if (A.m_pc + 2 == A.m_ppc && A.m_icount >= 5 && s_spin_on && adsp_spin(A.m_pc)) {
					const int m = (A.m_icount - 2) / 3;   // (icount - 1 left after this one, 1-3 of them run)
					A.m_icount -= 3 * m;
					s_spin_skipped += (uint64_t)m;
					s_skip_cyc += 3u * (uint64_t)m;
				}
			}
			break;
		case 0x1c: case 0x1d: case 0x1e: case 0x1f:
			// 000111xx xxxxxxxx xxxxxxxx  conditional call (immediate addr)
			if (condition(BIT(op, 0, 4)))
			{
				pc_stack_push();
				A.m_pc = BIT(op, 4, 14);
			}
			break;
		case 0x20: case 0x21:
			// 0010000x xxxxxxxx xxxxxxxx  conditional MAC to MR
			if (condition(BIT(op, 0, 4)))
			{
				if (ADSP_CHIP >= CHIP_TYPE_ADSP2181 && (op & 0x0018f0) == 0x000010)
					mac_op_mr_xop(op);
				else
					mac_op_mr(op);
			}
			break;
		case 0x22: case 0x23:
			// 0010001x xxxxxxxx xxxxxxxx  conditional ALU to AR
			if (condition(BIT(op, 0, 4)))
			{
				if (ADSP_CHIP >= CHIP_TYPE_ADSP2181 && BIT(op, 4))
					alu_op_ar_const(op);
				else
					alu_op_ar(op);
			}
			break;
		case 0x24: case 0x25:
			// 0010010x xxxxxxxx xxxxxxxx  conditional MAC to MF
			if (condition(BIT(op, 0, 4)))
			{
				if (ADSP_CHIP >= CHIP_TYPE_ADSP2181 && (op & 0x0018f0) == 0x000010)
					mac_op_mf_xop(op);
				else
					mac_op_mf(op);
			}
			break;
		case 0x26: case 0x27:
			// 0010011x xxxxxxxx xxxxxxxx  conditional ALU to AF
			if (condition(BIT(op, 0, 4)))
			{
				if (ADSP_CHIP >= CHIP_TYPE_ADSP2181 && BIT(op, 4))
					alu_op_af_const(op);
				else
					alu_op_af(op);
			}
			break;
		case 0x28: case 0x29:
			// 0010100x xxxxxxxx xxxxxxxx  MAC to MR with internal data register move
			temp = read_reg0(BIT(op, 0, 4));
			mac_op_mr(op);
			write_reg0(BIT(op, 4, 4), temp);
			break;
		case 0x2a: case 0x2b:
			// 0010101x xxxxxxxx xxxxxxxx  ALU to AR with internal data register move
			if (ADSP_CHIP >= CHIP_TYPE_ADSP2181 && BIT(op, 0, 8) == 0xaa)
				alu_op_none(op);
			else
			{
				temp = read_reg0(BIT(op, 0, 4));
				alu_op_ar(op);
				write_reg0(BIT(op, 4, 4), temp);
			}
			break;
		case 0x2c: case 0x2d:
			// 0010110x xxxxxxxx xxxxxxxx  MAC to MF with internal data register move
			temp = read_reg0(BIT(op, 0, 4));
			mac_op_mf(op);
			write_reg0(BIT(op, 4, 4), temp);
			break;
		case 0x2e: case 0x2f:
			// 0010111x xxxxxxxx xxxxxxxx  ALU to AF with internal data register move
			temp = read_reg0(BIT(op, 0, 4));
			alu_op_af(op);
			write_reg0(BIT(op, 4, 4), temp);
			break;
		case 0x30: case 0x31: case 0x32: case 0x33:
			// 001100xx xxxxxxxx xxxxxxxx  load non-data register immediate (group 0)
			write_reg0(BIT(op, 0, 4), sext(op >> 4, 14));
			break;
		case 0x34: case 0x35: case 0x36: case 0x37:
			// 001101xx xxxxxxxx xxxxxxxx  load non-data register immediate (group 1)
			write_reg1(BIT(op, 0, 4), sext(op >> 4, 14));
			break;
		case 0x38: case 0x39: case 0x3a: case 0x3b:
			// 001110xx xxxxxxxx xxxxxxxx  load non-data register immediate (group 2)
			write_reg2(BIT(op, 0, 4), sext(op >> 4, 14));
			break;
		case 0x3c: case 0x3d: case 0x3e: case 0x3f:
			// 001111xx xxxxxxxx xxxxxxxx  load non-data register immediate (group 3)
			write_reg3(BIT(op, 0, 4), sext(op >> 4, 14));
			break;
		case 0x40: case 0x41: case 0x42: case 0x43: case 0x44: case 0x45: case 0x46: case 0x47:
		case 0x48: case 0x49: case 0x4a: case 0x4b: case 0x4c: case 0x4d: case 0x4e: case 0x4f:
			// 0100xxxx xxxxxxxx xxxxxxxx  load data register immediate
			write_reg0(BIT(op, 0, 4), BIT(op, 4, 16));
			break;
		case 0x50: case 0x51:
			// 0101000x xxxxxxxx xxxxxxxx  MAC to MR with pgm memory read
			mac_op_mr(op);
			write_reg0(BIT(op, 4, 4), pgm_read_dag2(op));
			break;
		case 0x52: case 0x53:
			// 0101001x xxxxxxxx xxxxxxxx  ALU to AR with pgm memory read
			alu_op_ar(op);
			write_reg0(BIT(op, 4, 4), pgm_read_dag2(op));
			break;
		case 0x54: case 0x55:
			// 0101010x xxxxxxxx xxxxxxxx  MAC to MF with pgm memory read
			mac_op_mf(op);
			write_reg0(BIT(op, 4, 4), pgm_read_dag2(op));
			break;
		case 0x56: case 0x57:
			// 0101011x xxxxxxxx xxxxxxxx  ALU to AF with pgm memory read
			alu_op_af(op);
			write_reg0(BIT(op, 4, 4), pgm_read_dag2(op));
			break;
		case 0x58: case 0x59:
			// 0101100x xxxxxxxx xxxxxxxx  MAC to MR with pgm memory write
			pgm_write_dag2(op, read_reg0(BIT(op, 4, 4)));
			mac_op_mr(op);
			break;
		case 0x5a: case 0x5b:
			// 0101101x xxxxxxxx xxxxxxxx  ALU to AR with pgm memory write
			pgm_write_dag2(op, read_reg0(BIT(op, 4, 4)));
			alu_op_ar(op);
			break;
		case 0x5c: case 0x5d:
			// 0101110x xxxxxxxx xxxxxxxx  ALU to MR with pgm memory write
			pgm_write_dag2(op, read_reg0(BIT(op, 4, 4)));
			mac_op_mf(op);
			break;
		case 0x5e: case 0x5f:
			// 0101111x xxxxxxxx xxxxxxxx  ALU to MF with pgm memory write
			pgm_write_dag2(op, read_reg0(BIT(op, 4, 4)));
			alu_op_af(op);
			break;
		case 0x60: case 0x61:
			// 0110000x xxxxxxxx xxxxxxxx  MAC to MR with data memory read DAG1
			mac_op_mr(op);
			write_reg0(BIT(op, 4, 4), data_read_dag1(op));
			break;
		case 0x62: case 0x63:
			// 0110001x xxxxxxxx xxxxxxxx  ALU to AR with data memory read DAG1
			alu_op_ar(op);
			write_reg0(BIT(op, 4, 4), data_read_dag1(op));
			break;
		case 0x64: case 0x65:
			// 0110010x xxxxxxxx xxxxxxxx  MAC to MF with data memory read DAG1
			mac_op_mf(op);
			write_reg0(BIT(op, 4, 4), data_read_dag1(op));
			break;
		case 0x66: case 0x67:
			// 0110011x xxxxxxxx xxxxxxxx  ALU to AF with data memory read DAG1
			alu_op_af(op);
			write_reg0(BIT(op, 4, 4), data_read_dag1(op));
			break;
		case 0x68: case 0x69:
			// 0110100x xxxxxxxx xxxxxxxx  MAC to MR with data memory write DAG1
			data_write_dag1(op, read_reg0(BIT(op, 4, 4)));
			mac_op_mr(op);
			break;
		case 0x6a: case 0x6b:
			// 0110101x xxxxxxxx xxxxxxxx  ALU to AR with data memory write DAG1
			data_write_dag1(op, read_reg0(BIT(op, 4, 4)));
			alu_op_ar(op);
			break;
		case 0x6c: case 0x6d:
			// 0111110x xxxxxxxx xxxxxxxx  MAC to MF with data memory write DAG1
			data_write_dag1(op, read_reg0(BIT(op, 4, 4)));
			mac_op_mf(op);
			break;
		case 0x6e: case 0x6f:
			// 0111111x xxxxxxxx xxxxxxxx  ALU to AF with data memory write DAG1
			data_write_dag1(op, read_reg0(BIT(op, 4, 4)));
			alu_op_af(op);
			break;
		case 0x70: case 0x71:
			// 0111000x xxxxxxxx xxxxxxxx  MAC to MR with data memory read DAG2
			mac_op_mr(op);
			write_reg0(BIT(op, 4, 4), data_read_dag2(op));
			break;
		case 0x72: case 0x73:
			// 0111001x xxxxxxxx xxxxxxxx  ALU to AR with data memory read DAG2
			alu_op_ar(op);
			write_reg0(BIT(op, 4, 4), data_read_dag2(op));
			break;
		case 0x74: case 0x75:
			// 0111010x xxxxxxxx xxxxxxxx  MAC to MF with data memory read DAG2
			mac_op_mf(op);
			write_reg0(BIT(op, 4, 4), data_read_dag2(op));
			break;
		case 0x76: case 0x77:
			// 0111011x xxxxxxxx xxxxxxxx  ALU to AF with data memory read DAG2
			alu_op_af(op);
			write_reg0(BIT(op, 4, 4), data_read_dag2(op));
			break;
		case 0x78: case 0x79:
			// 0111100x xxxxxxxx xxxxxxxx  MAC to MR with data memory write DAG2
			data_write_dag2(op, read_reg0(BIT(op, 4, 4)));
			mac_op_mr(op);
			break;
		case 0x7a: case 0x7b:
			// 0111101x xxxxxxxx xxxxxxxx  ALU to AR with data memory write DAG2
			data_write_dag2(op, read_reg0(BIT(op, 4, 4)));
			alu_op_ar(op);
			break;
		case 0x7c: case 0x7d:
			// 0111110x xxxxxxxx xxxxxxxx  MAC to MF with data memory write DAG2
			data_write_dag2(op, read_reg0(BIT(op, 4, 4)));
			mac_op_mf(op);
			break;
		case 0x7e: case 0x7f:
			// 0111111x xxxxxxxx xxxxxxxx  ALU to AF with data memory write DAG2
			data_write_dag2(op, read_reg0(BIT(op, 4, 4)));
			alu_op_af(op);
			break;
		case 0x80: case 0x81: case 0x82: case 0x83:
			// 100000xx xxxxxxxx xxxxxxxx  read data memory (immediate addr) to reg group 0
			write_reg0(BIT(op, 0, 4), data_read(BIT(op, 4, 14)));
			break;
		case 0x84: case 0x85: case 0x86: case 0x87:
			// 100001xx xxxxxxxx xxxxxxxx  read data memory (immediate addr) to reg group 1
			write_reg1(BIT(op, 0, 4), data_read(BIT(op, 4, 14)));
			break;
		case 0x88: case 0x89: case 0x8a: case 0x8b:
			// 100010xx xxxxxxxx xxxxxxxx  read data memory (immediate addr) to reg group 2
			write_reg2(BIT(op, 0, 4), data_read(BIT(op, 4, 14)));
			break;
		case 0x8c: case 0x8d: case 0x8e: case 0x8f:
			// 100011xx xxxxxxxx xxxxxxxx  read data memory (immediate addr) to reg group 3
			write_reg3(BIT(op, 0, 4), data_read(BIT(op, 4, 14)));
			break;
		case 0x90: case 0x91: case 0x92: case 0x93:
			// 1001xxxx xxxxxxxx xxxxxxxx  write data memory (immediate addr) from reg group 0
			data_write(BIT(op, 4, 14), read_reg0(BIT(op, 0, 4)));
			break;
		case 0x94: case 0x95: case 0x96: case 0x97:
			// 1001xxxx xxxxxxxx xxxxxxxx  write data memory (immediate addr) from reg group 1
			data_write(BIT(op, 4, 14), read_reg1(BIT(op, 0, 4)));
			break;
		case 0x98: case 0x99: case 0x9a: case 0x9b:
			// 1001xxxx xxxxxxxx xxxxxxxx  write data memory (immediate addr) from reg group 2
			data_write(BIT(op, 4, 14), read_reg2(BIT(op, 0, 4)));
			break;
		case 0x9c: case 0x9d: case 0x9e: case 0x9f:
			// 1001xxxx xxxxxxxx xxxxxxxx  write data memory (immediate addr) from reg group 3
			data_write(BIT(op, 4, 14), read_reg3(BIT(op, 0, 4)));
			break;
		case 0xa0: case 0xa1: case 0xa2: case 0xa3: case 0xa4: case 0xa5: case 0xa6: case 0xa7:
		case 0xa8: case 0xa9: case 0xaa: case 0xab: case 0xac: case 0xad: case 0xae: case 0xaf:
			// 1010xxxx xxxxxxxx xxxxxxxx  data memory write (immediate) DAG1
			data_write_dag1(op, BIT(op, 4, 16));
			break;
		case 0xb0: case 0xb1: case 0xb2: case 0xb3: case 0xb4: case 0xb5: case 0xb6: case 0xb7:
		case 0xb8: case 0xb9: case 0xba: case 0xbb: case 0xbc: case 0xbd: case 0xbe: case 0xbf:
			// 1011xxxx xxxxxxxx xxxxxxxx  data memory write (immediate) DAG2
			data_write_dag2(op, BIT(op, 4, 16));
			break;
		case 0xc0: case 0xc1:
			// 1100000x xxxxxxxx xxxxxxxx  MAC to MR with data read to AX0 & pgm read to AY0
			mac_op_mr(op);
			A.m_core.ax0.u = data_read_dag1(op);
			A.m_core.ay0.u = pgm_read_dag2(op >> 4);
			break;
		case 0xc2: case 0xc3:
			// 1100001x xxxxxxxx xxxxxxxx  ALU to AR with data read to AX0 & pgm read to AY0
			alu_op_ar(op);
			A.m_core.ax0.u = data_read_dag1(op);
			A.m_core.ay0.u = pgm_read_dag2(op >> 4);
			break;
		case 0xc4: case 0xc5:
			// 1100010x xxxxxxxx xxxxxxxx  MAC to MR with data read to AX1 & pgm read to AY0
			mac_op_mr(op);
			A.m_core.ax1.u = data_read_dag1(op);
			A.m_core.ay0.u = pgm_read_dag2(op >> 4);
			break;
		case 0xc6: case 0xc7:
			// 1100011x xxxxxxxx xxxxxxxx  ALU to AR with data read to AX1 & pgm read to AY0
			alu_op_ar(op);
			A.m_core.ax1.u = data_read_dag1(op);
			A.m_core.ay0.u = pgm_read_dag2(op >> 4);
			break;
		case 0xc8: case 0xc9:
			// 1100100x xxxxxxxx xxxxxxxx  MAC to MR with data read to MX0 & pgm read to AY0
			mac_op_mr(op);
			A.m_core.mx0.u = data_read_dag1(op);
			A.m_core.ay0.u = pgm_read_dag2(op >> 4);
			break;
		case 0xca: case 0xcb:
			// 1100101x xxxxxxxx xxxxxxxx  ALU to AR with data read to MX0 & pgm read to AY0
			alu_op_ar(op);
			A.m_core.mx0.u = data_read_dag1(op);
			A.m_core.ay0.u = pgm_read_dag2(op >> 4);
			break;
		case 0xcc: case 0xcd:
			// 1100110x xxxxxxxx xxxxxxxx  MAC to MR with data read to MX1 & pgm read to AY0
			mac_op_mr(op);
			A.m_core.mx1.u = data_read_dag1(op);
			A.m_core.ay0.u = pgm_read_dag2(op >> 4);
			break;
		case 0xce: case 0xcf:
			// 1100111x xxxxxxxx xxxxxxxx  ALU to AR with data read to MX1 & pgm read to AY0
			alu_op_ar(op);
			A.m_core.mx1.u = data_read_dag1(op);
			A.m_core.ay0.u = pgm_read_dag2(op >> 4);
			break;
		case 0xd0: case 0xd1:
			// 1101000x xxxxxxxx xxxxxxxx  MAC to MR with data read to AX0 & pgm read to AY1
			mac_op_mr(op);
			A.m_core.ax0.u = data_read_dag1(op);
			A.m_core.ay1.u = pgm_read_dag2(op >> 4);
			break;
		case 0xd2: case 0xd3:
			// 1101001x xxxxxxxx xxxxxxxx  ALU to AR with data read to AX0 & pgm read to AY1
			alu_op_ar(op);
			A.m_core.ax0.u = data_read_dag1(op);
			A.m_core.ay1.u = pgm_read_dag2(op >> 4);
			break;
		case 0xd4: case 0xd5:
			// 1101010x xxxxxxxx xxxxxxxx  MAC to MR with data read to AX1 & pgm read to AY1
			mac_op_mr(op);
			A.m_core.ax1.u = data_read_dag1(op);
			A.m_core.ay1.u = pgm_read_dag2(op >> 4);
			break;
		case 0xd6: case 0xd7:
			// 1101011x xxxxxxxx xxxxxxxx  ALU to AR with data read to AX1 & pgm read to AY1
			alu_op_ar(op);
			A.m_core.ax1.u = data_read_dag1(op);
			A.m_core.ay1.u = pgm_read_dag2(op >> 4);
			break;
		case 0xd8: case 0xd9:
			// 1101100x xxxxxxxx xxxxxxxx  MAC to MR with data read to MX0 & pgm read to AY1
			mac_op_mr(op);
			A.m_core.mx0.u = data_read_dag1(op);
			A.m_core.ay1.u = pgm_read_dag2(op >> 4);
			break;
		case 0xda: case 0xdb:
			// 1101101x xxxxxxxx xxxxxxxx  ALU to AR with data read to MX0 & pgm read to AY1
			alu_op_ar(op);
			A.m_core.mx0.u = data_read_dag1(op);
			A.m_core.ay1.u = pgm_read_dag2(op >> 4);
			break;
		case 0xdc: case 0xdd:
			// 1101110x xxxxxxxx xxxxxxxx  MAC to MR with data read to MX1 & pgm read to AY1
			mac_op_mr(op);
			A.m_core.mx1.u = data_read_dag1(op);
			A.m_core.ay1.u = pgm_read_dag2(op >> 4);
			break;
		case 0xde: case 0xdf:
			// 1101111x xxxxxxxx xxxxxxxx  ALU to AR with data read to MX1 & pgm read to AY1
			alu_op_ar(op);
			A.m_core.mx1.u = data_read_dag1(op);
			A.m_core.ay1.u = pgm_read_dag2(op >> 4);
			break;
		case 0xe0: case 0xe1:
			// 1110000x xxxxxxxx xxxxxxxx  MAC to MR with data read to AX0 & pgm read to MY0
			mac_op_mr(op);
			A.m_core.ax0.u = data_read_dag1(op);
			A.m_core.my0.u = pgm_read_dag2(op >> 4);
			break;
		case 0xe2: case 0xe3:
			// 1110001x xxxxxxxx xxxxxxxx  ALU to AR with data read to AX0 & pgm read to MY0
			alu_op_ar(op);
			A.m_core.ax0.u = data_read_dag1(op);
			A.m_core.my0.u = pgm_read_dag2(op >> 4);
			break;
		case 0xe4: case 0xe5:
			// 1110010x xxxxxxxx xxxxxxxx  MAC to MR with data read to AX1 & pgm read to MY0
			mac_op_mr(op);
			A.m_core.ax1.u = data_read_dag1(op);
			A.m_core.my0.u = pgm_read_dag2(op >> 4);
			break;
		case 0xe6: case 0xe7:
			// 1110011x xxxxxxxx xxxxxxxx  ALU to AR with data read to AX1 & pgm read to MY0
			alu_op_ar(op);
			A.m_core.ax1.u = data_read_dag1(op);
			A.m_core.my0.u = pgm_read_dag2(op >> 4);
			break;
		case 0xe8: case 0xe9:
			// 1110100x xxxxxxxx xxxxxxxx  MAC to MR with data read to MX0 & pgm read to MY0
			mac_op_mr(op);
			A.m_core.mx0.u = data_read_dag1(op);
			A.m_core.my0.u = pgm_read_dag2(op >> 4);
			break;
		case 0xea: case 0xeb:
			// 1110101x xxxxxxxx xxxxxxxx  ALU to AR with data read to MX0 & pgm read to MY0
			alu_op_ar(op);
			A.m_core.mx0.u = data_read_dag1(op);
			A.m_core.my0.u = pgm_read_dag2(op >> 4);
			break;
		case 0xec: case 0xed:
			// 1110110x xxxxxxxx xxxxxxxx  MAC to MR with data read to MX1 & pgm read to MY0
			mac_op_mr(op);
			A.m_core.mx1.u = data_read_dag1(op);
			A.m_core.my0.u = pgm_read_dag2(op >> 4);
			break;
		case 0xee: case 0xef:
			// 1110111x xxxxxxxx xxxxxxxx  ALU to AR with data read to MX1 & pgm read to MY0
			alu_op_ar(op);
			A.m_core.mx1.u = data_read_dag1(op);
			A.m_core.my0.u = pgm_read_dag2(op >> 4);
			break;
		case 0xf0: case 0xf1:
			// 1111000x xxxxxxxx xxxxxxxx  MAC to MR with data read to AX0 & pgm read to MY1
			mac_op_mr(op);
			A.m_core.ax0.u = data_read_dag1(op);
			A.m_core.my1.u = pgm_read_dag2(op >> 4);
			break;
		case 0xf2: case 0xf3:
			// 1111001x xxxxxxxx xxxxxxxx  ALU to AR with data read to AX0 & pgm read to MY1
			alu_op_ar(op);
			A.m_core.ax0.u = data_read_dag1(op);
			A.m_core.my1.u = pgm_read_dag2(op >> 4);
			break;
		case 0xf4: case 0xf5:
			// 1111010x xxxxxxxx xxxxxxxx  MAC to MR with data read to AX1 & pgm read to MY1
			mac_op_mr(op);
			A.m_core.ax1.u = data_read_dag1(op);
			A.m_core.my1.u = pgm_read_dag2(op >> 4);
			break;
		case 0xf6: case 0xf7:
			// 1111011x xxxxxxxx xxxxxxxx  ALU to AR with data read to AX1 & pgm read to MY1
			alu_op_ar(op);
			A.m_core.ax1.u = data_read_dag1(op);
			A.m_core.my1.u = pgm_read_dag2(op >> 4);
			break;
		case 0xf8: case 0xf9:
			// 1111100x xxxxxxxx xxxxxxxx  MAC to MR with data read to MX0 & pgm read to MY1
			mac_op_mr(op);
			A.m_core.mx0.u = data_read_dag1(op);
			A.m_core.my1.u = pgm_read_dag2(op >> 4);
			break;
		case 0xfa: case 0xfb:
			// 1111101x xxxxxxxx xxxxxxxx  ALU to AR with data read to MX0 & pgm read to MY1
			alu_op_ar(op);
			A.m_core.mx0.u = data_read_dag1(op);
			A.m_core.my1.u = pgm_read_dag2(op >> 4);
			break;
		case 0xfc: case 0xfd:
			// 1111110x xxxxxxxx xxxxxxxx  MAC to MR with data read to MX1 & pgm read to MY1
			mac_op_mr(op);
			A.m_core.mx1.u = data_read_dag1(op);
			A.m_core.my1.u = pgm_read_dag2(op >> 4);
			break;
		case 0xfe: case 0xff:
			// 1111111x xxxxxxxx xxxxxxxx  ALU to AR with data read to MX1 & pgm read to MY1
			alu_op_ar(op);
			A.m_core.mx1.u = data_read_dag1(op);
			A.m_core.my1.u = pgm_read_dag2(op >> 4);
			break;
	}

}

/*===========================================================================
    Predecoded instruction cache (ReBOX, results identical to the interpreter; RBX_ADSP_NOJIT=1 = off). Per program
    address: the opcode word seen there, a pointer to that word in the board's memory, and its handler. An entry is
    used only while the memory word still holds the opcode (every write path - program writes, the data-space view
    of the program RAM's upper bits, the board's boot loader, a state load - is covered by that check), and a page's
    entries are dropped when the board maps the page again. The handlers of the multifunction classes (conditional
    ALU/MAC, with register move, with DAG1/DAG2 data read or write) are compiled once per ALU/MAC kind, so the
    kind's switch, the DAG accessors and the register writes are inlined; everything else runs exec_op. The fetch,
    the do-until loop handling, the cycle count and the interrupt checks are the interpreter's.
===========================================================================*/

typedef void (*adsp_op_fn)(uint32_t op);
typedef struct { const uint32_t *src; uint32_t op; adsp_op_fn fn; } adsp_pd;
enum { PD_N = 0x4000 };
static adsp_pd *s_pd;                   // NULL = off
static int s_pd_on = -1;
static const uint32_t s_pd_none = 0xffffffffu;   // (an invalid entry's word: never equal to its op 0)

static void exec_generic(uint32_t op) { exec_op(op); }

// the unit: 0 MAC to MR, 1 ALU to AR, 2 MAC to MF, 3 ALU to AF
#define PD_UNIT(u, k) do { switch (u) { case 0: mac_op_mr_k(op, k); break; case 1: alu_op_ar_k(op, k); break; \
	case 2: mac_op_mf_k(op, k); break; default: alu_op_af_k(op, k); break; } } while (0)
static AINLINE void pd_class(uint32_t op, const int c, const int k)
{
	uint32_t temp;
	switch (c >> 2) {
	case 0x04:   // 0x20-0x27  conditional ALU/MAC
		if (condition(BIT(op, 0, 4)))
			PD_UNIT(c & 3, k);
		break;
	case 0x05:   // 0x28-0x2f  ALU/MAC with internal data register move
		temp = read_reg0(BIT(op, 0, 4));
		PD_UNIT(c & 3, k);
		write_reg0(BIT(op, 4, 4), temp);
		break;
	case 0x0c:   // 0x60-0x67  ALU/MAC with data memory read DAG1
		PD_UNIT(c & 3, k);
		write_reg0(BIT(op, 4, 4), data_read_dag1(op));
		break;
	case 0x0d:   // 0x68-0x6f  ALU/MAC with data memory write DAG1
		data_write_dag1(op, read_reg0(BIT(op, 4, 4)));
		PD_UNIT(c & 3, k);
		break;
	case 0x0e:   // 0x70-0x77  ALU/MAC with data memory read DAG2
		PD_UNIT(c & 3, k);
		write_reg0(BIT(op, 4, 4), data_read_dag2(op));
		break;
	case 0x0f:   // 0x78-0x7f  ALU/MAC with data memory write DAG2
		data_write_dag2(op, read_reg0(BIT(op, 4, 4)));
		PD_UNIT(c & 3, k);
		break;
	}
}
#define PD_K16(F, c) F(c, 0) F(c, 1) F(c, 2) F(c, 3) F(c, 4) F(c, 5) F(c, 6) F(c, 7) \
	F(c, 8) F(c, 9) F(c, 10) F(c, 11) F(c, 12) F(c, 13) F(c, 14) F(c, 15)
#define PD_CLASSES(F) F(0x10) F(0x11) F(0x12) F(0x13) F(0x14) F(0x15) F(0x16) F(0x17) \
	F(0x30) F(0x31) F(0x32) F(0x33) F(0x34) F(0x35) F(0x36) F(0x37) \
	F(0x38) F(0x39) F(0x3a) F(0x3b) F(0x3c) F(0x3d) F(0x3e) F(0x3f)
#define PD_DEF(c, k) static void pdh_##c##_##k(uint32_t op) { pd_class(op, c, k); }
#define PD_DEFS(c) PD_K16(PD_DEF, c)
PD_CLASSES(PD_DEFS)
#define PD_REF(c, k) pdh_##c##_##k,
#define PD_ROW(c) [c] = { PD_K16(PD_REF, c) },
static const adsp_op_fn s_pd_tab[0x40][16] = { PD_CLASSES(PD_ROW) };   // [opcode bits 17-22][kind]

static adsp_op_fn pd_select(uint32_t op)
{
	if (BIT(op, 16, 8) < 0x80) {
		const adsp_op_fn f = s_pd_tab[BIT(op, 17, 6)][BIT(op, 13, 4)];
		if (f)
			return f;
	}
	return exec_generic;
}

static void pd_flush_page(int page)
{
	if (!s_pd)
		return;
	adsp_pd *e = &s_pd[(page & 15) << 10];
	for (int i = 0; i < 0x400; i++)
		e[i].src = &s_pd_none, e[i].op = 0;
}

static void pd_init(void)
{
	if (s_pd_on < 0)
		s_pd_on = !getenv("RBX_ADSP_NOJIT");
	if (s_pd_on && !s_pd)
		s_pd = (adsp_pd *)malloc(PD_N * sizeof(adsp_pd));
	for (int p = 0; p < 16; p++)
		pd_flush_page(p);
}

void adsp_exit(void)
{
	free(s_pd);
	s_pd = NULL;
}

static adsp_op_fn pd_fill(adsp_pd *e, uint32_t pc, uint32_t op)
{
	const ppage_t *p = &s_ppage[(pc >> 10) & 15];
	const adsp_op_fn f = pd_select(op);
	if (p->type == PG_MEM) {
		e->src = &p->p32[pc & 0x3ff];
		e->op = op;
		e->fn = f;
	}
	return f;
}

static void execute_pd(void)
{
	do
	{
		const uint32_t pc = A.m_pc;
		A.m_ppc = pc;   // copy PC to previous PC

		// instruction fetch: the predecoded entry while its word is unchanged
		adsp_pd *e = &s_pd[pc & (PD_N - 1)];
		uint32_t op = e->op;
		adsp_op_fn fn = e->fn;
		if (*e->src != op) {
			op = opcode_read();
			fn = pd_fill(e, pc, op);
		}

		// advance to the next instruction
		if (pc != A.m_loop)
			A.m_pc = pc + 1;

		// handle looping
		else
		{
			// condition not met, keep looping
			if (condition(A.m_loop_condition))
				A.m_pc = pc_stack_top();

			// condition met; pop the PC and loop stacks and fall through
			else
			{
				loop_stack_pop();
				pc_stack_pop_val();
				A.m_pc++;
			}
		}

		fn(op);
		A.m_icount--;
	} while (A.m_icount > 0);
}

static void execute_run()
{
	check_irqs();

	// (IDLE: MAME ends the timeslice; here the core waits for an interrupt, the time accounted)
	if (A.m_idle) {
		s_skip_cyc += (uint64_t)(A.m_icount > 0 ? A.m_icount : 0);
		A.m_icount = 0;
		return;
	}

	if (s_pd) {
		execute_pd();
		return;
	}

	do
	{
		A.m_ppc = A.m_pc;   // copy PC to previous PC

		// instruction fetch
		uint32_t op = opcode_read();

		// advance to the next instruction
		if (A.m_pc != A.m_loop)
			A.m_pc++;

		// handle looping
		else
		{
			// condition not met, keep looping
			if (condition(A.m_loop_condition))
				A.m_pc = pc_stack_top();

			// condition met; pop the PC and loop stacks and fall through
			else
			{
				loop_stack_pop();
				pc_stack_pop_val();
				A.m_pc++;
			}
		}

		exec_op(op);
		A.m_icount--;
	} while (A.m_icount > 0);
}

/***************************************************************************
    ReBOX interface
***************************************************************************/

void adsp2105_init(const adsp_callbacks *cb)
{
	memset(&A, 0, sizeof(A));
	memset(s_dpage, 0, sizeof(s_dpage));
	memset(s_ppage, 0, sizeof(s_ppage));
	adsp_cb = *cb;
	pd_init();
	s_req = s_stolen = s_executing = 0;
	if (s_spin_on < 0) {
		s_spin_on = !getenv("RBX_ADSP_NOSPIN");
		if (getenv("RBX_ADSP_SPINSTAT"))
			atexit(spin_stat);
	}
	A.m_mstat_mask = (ADSP_CHIP >= CHIP_TYPE_ADSP2101) ? 0x7f : 0x0f;
	A.m_imask_mask = (ADSP_CHIP >= CHIP_TYPE_ADSP2181) ? 0x3ff : (ADSP_CHIP >= CHIP_TYPE_ADSP2101) ? 0x3f : 0x0f;

	// the constructor: tables and register maps
	create_tables();
	T.m_read0_ptr[0x00] = &A.m_core.ax0.s;
	T.m_read0_ptr[0x01] = &A.m_core.ax1.s;
	T.m_read0_ptr[0x02] = &A.m_core.mx0.s;
	T.m_read0_ptr[0x03] = &A.m_core.mx1.s;
	T.m_read0_ptr[0x04] = &A.m_core.ay0.s;
	T.m_read0_ptr[0x05] = &A.m_core.ay1.s;
	T.m_read0_ptr[0x06] = &A.m_core.my0.s;
	T.m_read0_ptr[0x07] = &A.m_core.my1.s;
	T.m_read0_ptr[0x08] = &A.m_core.si.s;
	T.m_read0_ptr[0x09] = &A.m_core.se.s;
	T.m_read0_ptr[0x0a] = &A.m_core.ar.s;
	T.m_read0_ptr[0x0b] = &A.m_core.mr.mrx.mr0.s;
	T.m_read0_ptr[0x0c] = &A.m_core.mr.mrx.mr1.s;
	T.m_read0_ptr[0x0d] = &A.m_core.mr.mrx.mr2.s;
	T.m_read0_ptr[0x0e] = &A.m_core.sr.srx.sr0.s;
	T.m_read0_ptr[0x0f] = &A.m_core.sr.srx.sr1.s;
	for (int index = 0; index < 4; index++) {
		T.m_read1_ptr[0x00 + index] = &A.m_i[0 + index];
		T.m_read1_ptr[0x04 + index] = (uint32_t *)&A.m_m[0 + index];
		T.m_read1_ptr[0x08 + index] = &A.m_l[0 + index];
		T.m_read1_ptr[0x0c + index] = &A.m_l[0 + index];
		T.m_read2_ptr[0x00 + index] = &A.m_i[4 + index];
		T.m_read2_ptr[0x04 + index] = (uint32_t *)&A.m_m[4 + index];
		T.m_read2_ptr[0x08 + index] = &A.m_l[4 + index];
		T.m_read2_ptr[0x0c + index] = &A.m_l[4 + index];
	}
	T.m_alu_xregs[0] = &A.m_core.ax0;
	T.m_alu_xregs[1] = &A.m_core.ax1;
	T.m_alu_xregs[2] = &A.m_core.ar;
	T.m_alu_xregs[3] = &A.m_core.mr.mrx.mr0;
	T.m_alu_xregs[4] = &A.m_core.mr.mrx.mr1;
	T.m_alu_xregs[5] = &A.m_core.mr.mrx.mr2;
	T.m_alu_xregs[6] = &A.m_core.sr.srx.sr0;
	T.m_alu_xregs[7] = &A.m_core.sr.srx.sr1;
	T.m_alu_yregs[0] = &A.m_core.ay0;
	T.m_alu_yregs[1] = &A.m_core.ay1;
	T.m_alu_yregs[2] = &A.m_core.af;
	T.m_alu_yregs[3] = &A.m_core.zero;
	T.m_mac_xregs[0] = &A.m_core.mx0;
	T.m_mac_xregs[1] = &A.m_core.mx1;
	T.m_mac_xregs[2] = &A.m_core.ar;
	T.m_mac_xregs[3] = &A.m_core.mr.mrx.mr0;
	T.m_mac_xregs[4] = &A.m_core.mr.mrx.mr1;
	T.m_mac_xregs[5] = &A.m_core.mr.mrx.mr2;
	T.m_mac_xregs[6] = &A.m_core.sr.srx.sr0;
	T.m_mac_xregs[7] = &A.m_core.sr.srx.sr1;
	T.m_mac_yregs[0] = &A.m_core.my0;
	T.m_mac_yregs[1] = &A.m_core.my1;
	T.m_mac_yregs[2] = &A.m_core.mf;
	T.m_mac_yregs[3] = &A.m_core.zero;
	T.m_shift_xregs[0] = &A.m_core.si;
	T.m_shift_xregs[1] = &A.m_core.si;
	T.m_shift_xregs[2] = &A.m_core.ar;
	T.m_shift_xregs[3] = &A.m_core.mr.mrx.mr0;
	T.m_shift_xregs[4] = &A.m_core.mr.mrx.mr1;
	T.m_shift_xregs[5] = &A.m_core.mr.mrx.mr2;
	T.m_shift_xregs[6] = &A.m_core.sr.srx.sr0;
	T.m_shift_xregs[7] = &A.m_core.sr.srx.sr1;
}

void adsp_reset(void)
{
	// ensure that zero is zero
	A.m_core.zero.u = A.m_alt.zero.u = 0;

	// recompute the memory registers with their current values
	write_reg1(0x08, A.m_l[0]);   write_reg1(0x00, A.m_i[0]);
	write_reg1(0x09, A.m_l[1]);   write_reg1(0x01, A.m_i[1]);
	write_reg1(0x0a, A.m_l[2]);   write_reg1(0x02, A.m_i[2]);
	write_reg1(0x0b, A.m_l[3]);   write_reg1(0x03, A.m_i[3]);
	write_reg2(0x08, A.m_l[4]);   write_reg2(0x00, A.m_i[4]);
	write_reg2(0x09, A.m_l[5]);   write_reg2(0x01, A.m_i[5]);
	write_reg2(0x0a, A.m_l[6]);   write_reg2(0x02, A.m_i[6]);
	write_reg2(0x0b, A.m_l[7]);   write_reg2(0x03, A.m_i[7]);

	// reset PC and loops
	A.m_pc = (ADSP_CHIP >= CHIP_TYPE_ADSP2101) ? 0 : 4;
	A.m_ppc = A.m_pc;
	A.m_loop = 0xffff;
	A.m_loop_condition = 0;

	// reset status registers
	A.m_astat_clear = ~(CFLAG | VFLAG | NFLAG | ZFLAG);
	A.m_mstat = 0;
	A.m_sstat = 0x55;
	A.m_idle = 0;
	update_mstat();

	// reset stacks
	A.m_pc_sp = 0;
	A.m_cntr_sp = 0;
	A.m_stat_sp = 0;
	A.m_loop_sp = 0;

	// reset external I/O
	A.m_flagout = 0;
	A.m_flagin = 0;
	A.m_fl0 = 0;
	A.m_fl1 = 0;
	A.m_fl2 = 0;

	// reset interrupts
	A.m_imask = 0;
	for (int irq = 0; irq < 10; irq++)
		A.m_irq_state[irq] = A.m_irq_latch[irq] = CLEAR_LINE;
}

void adsp_map_data_mem(int page, uint16_t *mem, int writable)
{
	dpage_t *p = &s_dpage[page & 15];
	memset(p, 0, sizeof(*p));
	p->type = writable ? PG_MEM : PG_ROM;
	p->p16 = mem;
	p->w16 = writable ? mem : NULL;
}
void adsp_map_data_hi(int page, uint32_t *mem)
{
	dpage_t *p = &s_dpage[page & 15];
	memset(p, 0, sizeof(*p));
	p->type = PG_HI;
	p->p32 = mem;
}
void adsp_map_data_io(int page, uint16_t (*r)(uint32_t a), void (*w)(uint32_t a, uint16_t d))
{
	dpage_t *p = &s_dpage[page & 15];
	memset(p, 0, sizeof(*p));
	p->type = PG_IO;
	p->r = r;
	p->w = w;
}
void adsp_map_data_none(int page) { memset(&s_dpage[page & 15], 0, sizeof(dpage_t)); }
void adsp_map_prog_mem(int page, uint32_t *mem)
{
	pd_flush_page(page);
	ppage_t *p = &s_ppage[page & 15];
	memset(p, 0, sizeof(*p));
	p->type = PG_MEM;
	p->p32 = mem;
}
void adsp_map_prog_io(int page, uint32_t (*r)(uint32_t a), void (*w)(uint32_t a, uint32_t d))
{
	pd_flush_page(page);
	ppage_t *p = &s_ppage[page & 15];
	memset(p, 0, sizeof(*p));
	p->type = PG_IO;
	p->r = r;
	p->w = w;
}
void adsp_map_prog_none(int page) { pd_flush_page(page); memset(&s_ppage[page & 15], 0, sizeof(ppage_t)); }
uint16_t adsp_data_read(uint32_t a) { return data_read(a & 0x3fff); }
uint32_t adsp_prog_read(uint32_t a) { return program_read(a & 0x3fff); }

int adsp_execute(int cycles)
{
	if (cycles <= 0)
		return 0;
	s_req = cycles;
	s_stolen = 0;
	A.m_icount = cycles;
	s_executing = 1;
	execute_run();
	s_executing = 0;
	int ran = cycles - (A.m_icount > 0 ? A.m_icount : 0) - s_stolen;
	if (A.m_icount < 0)
		ran = cycles - A.m_icount - s_stolen;
	A.total += (uint64_t)(int64_t)ran;
	s_ran_cyc += (uint64_t)(int64_t)ran;
	A.m_icount = 0;
	s_req = s_stolen = 0;
	return ran;
}

void adsp_abort(void)
{
	if (s_executing && A.m_icount > 0) {
		s_stolen += A.m_icount;
		A.m_icount = 0;
	}
}

uint64_t adsp_total_cycles(void)
{
	return A.total + (uint64_t)(int64_t)(s_executing ? s_req - A.m_icount - s_stolen : 0);
}

void adsp_set_input(int line, int state) { execute_set_input(line, state); }
uint32_t adsp_get_i(int n) { return A.m_i[n & 7]; }
void adsp_set_i(int n, uint32_t v) { A.m_i[n & 7] = v; update_i(n & 7); }
int32_t adsp_get_m(int n) { return A.m_m[n & 7]; }
uint32_t adsp_get_l(int n) { return A.m_l[n & 7]; }
uint32_t adsp_get_ibase(int n) { return A.m_base[n & 7]; }

void adsp_load_boot_data(const uint8_t *srcdata, uint32_t *dstdata)
{
	// see how many words we need to copy
	int pagelen = (srcdata[3] + 1) * 8;
	for (int i = 0; i < pagelen; i++) {
		uint32_t opcode = ((uint32_t)srcdata[i*4+0] << 16) | ((uint32_t)srcdata[i*4+1] << 8) | srcdata[i*4+2];
		dstdata[i] = opcode;
	}
}

int adsp_state(void *ctx, int load)
{
	(void)load;
	return rbx_state_io(ctx, "adsp", &A, sizeof(A));
}

void adsp_skip(int cycles) { if (cycles > 0) A.total += (uint64_t)cycles; }
void adsp_state_time_reset(void) { A.total = 0; }
uint32_t adsp_pc(void) { return A.m_pc; }
uint64_t adsp_insns(void) { return s_ran_cyc - s_skip_cyc; }
