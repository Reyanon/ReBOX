/*  Mitsubishi M37702 (MELPS 7700) CPU, ported to C from MAME 0.289 src/devices/cpu/m37710:
    m37710.cpp, m37710.h, m37710cm.h, m37710il.h (and m37710op.h, see m37710op.h here)
    (BSD-3-Clause, copyright-holders: R. Belmont, Karl Stenerud).
    The MAME emu_timers (timers A0-A4 / B0-B2 and the A/D converter) are counted in CPU cycles here: they fire
    at the first instruction boundary at or after their expiry (where MAME's scheduler ends the timeslice),
    then the interrupts are evaluated (MAME: the next execute_run).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../cheat_cpu.h"
#include "m37710.h"
#include <string.h>

#ifdef M37710_TRACE
void m37710_trace_hook(m37710_t *cpu);   // supplied by the test build: called before each instruction
#endif

/* ======================================================================== */
/* ================================ GENERAL =============================== */
/* ======================================================================== */

#define MAKE_INT_8(A)   ((int8_t)((A)&0xff))
#define MAKE_UINT_8(A)  ((A)&0xff)
#define MAKE_UINT_16(A) ((A)&0xffff)
#define MAKE_UINT_24(A) ((A)&0xffffff)

#define BIT_0       0x01
#define BIT_1       0x02
#define BIT_2       0x04
#define BIT_3       0x08
#define BIT_4       0x10
#define BIT_5       0x20
#define BIT_6       0x40
#define BIT_7       0x80
#define BIT(x, n)   (((x) >> (n)) & 1)

#define logerror(...) ((void)cpu)   // (MAME logs BRK / unimplemented opcodes)

/* ======================================================================== */
/* ================================== CPU ================================= */
/* ======================================================================== */

#define REG_A           cpu->a
#define REG_B           cpu->b
#define REG_BA          cpu->ba
#define REG_BB          cpu->bb
#define REG_X           cpu->x
#define REG_Y           cpu->y
#define REG_XH          cpu->xh
#define REG_YH          cpu->yh
#define REG_S           cpu->s
#define REG_PC          cpu->pc
#define REG_PPC         cpu->ppc
#define REG_PG          cpu->pg
#define REG_DT          cpu->dt
#define REG_DPR         cpu->dpr
#define REG_IPL         cpu->ipl
#define FLAG_M          cpu->flag_m
#define FLAG_X          cpu->flag_x
#define FLAG_N          cpu->flag_n
#define FLAG_V          cpu->flag_v
#define FLAG_D          cpu->flag_d
#define FLAG_I          cpu->flag_i
#define FLAG_Z          cpu->flag_z
#define FLAG_C          cpu->flag_c
#define LINE_IRQ        cpu->line_irq
#define REG_IR          cpu->ir
#define REG_IM          cpu->im
#define REG_IM2         cpu->im2
#define REG_IM3         cpu->im3
#define REG_IM4         cpu->im4
#define CLOCKS          cpu->icount
#define IRQ_DELAY       cpu->irq_delay
#define CPU_STOPPED     cpu->stopped
#define SRC             cpu->source
#define DST             cpu->destination

#define STOP_LEVEL_WAI  1
#define STOP_LEVEL_STOP 2

#define EXECUTION_MODE_M0X0 0
#define EXECUTION_MODE_M0X1 1
#define EXECUTION_MODE_M1X0 2
#define EXECUTION_MODE_M1X1 3

/* ======================================================================== */
/* ================================= CLOCK ================================ */
/* ======================================================================== */

#define CLK_OP          1
#define CLK_R8          1
#define CLK_R16         2
#define CLK_R24         3
#define CLK_W8          1
#define CLK_W16         2
#define CLK_W24         3
#define CLK_RMW8        3
#define CLK_RMW16       5

#define CLK_IMPLIED     1
#define CLK_RELATIVE_8  1
#define CLK_RELATIVE_16 2
#define CLK_IMM         0
#define CLK_AI          4
#define CLK_AXI         4
#define CLK_A           2
#define CLK_AL          3
#define CLK_ALX         3
#define CLK_AX          2
#define CLK_AY          2
#define CLK_D           1
#define CLK_DI          3
#define CLK_DIY         3
#define CLK_DLI         4
#define CLK_DLIY        4
#define CLK_DX          2
#define CLK_DXI         4
#define CLK_DY          2
#define CLK_S           2
#define CLK_SIY         5

/* AX and AY addressing modes take 1 extra cycle when writing */
#define CLK_W_IMM       0
#define CLK_W_AI        4
#define CLK_W_AXI       4
#define CLK_W_A         2
#define CLK_W_AL        3
#define CLK_W_ALX       3
#define CLK_W_AX        3
#define CLK_W_AY        3
#define CLK_W_D         1
#define CLK_W_DI        3
#define CLK_W_DIY       3
#define CLK_W_DLI       4
#define CLK_W_DLIY      4
#define CLK_W_DX        2
#define CLK_W_DXI       4
#define CLK_W_DY        2
#define CLK_W_S         2
#define CLK_W_SIY       5

#define CLK(A)          CLOCKS -= (A)
// MAME: CLOCKS = 0 (WAI / STP end the timeslice). Here the stopped CPU idles in m37710_execute up to the next
// internal timer event (MAME's timeslice ends there too), so nothing to do.
#define USE_ALL_CLKS()  ((void)0)

/* ======================================================================== */
/* ============================ STATUS REGISTER =========================== */
/* ======================================================================== */

#define FLAGPOS_N       BIT_7
#define FLAGPOS_V       BIT_6
#define FLAGPOS_D       BIT_3
#define FLAGPOS_I       BIT_2
#define FLAGPOS_Z       BIT_1
#define FLAGPOS_C       BIT_0
#define FLAGPOS_R       BIT_5
#define FLAGPOS_B       BIT_4
#define FLAGPOS_M       BIT_5
#define FLAGPOS_X       BIT_4

#define EFLAG_SET       1
#define EFLAG_CLEAR     0
#define MFLAG_SET       FLAGPOS_M
#define MFLAG_CLEAR     0
#define XFLAG_SET       FLAGPOS_X
#define XFLAG_CLEAR     0
#define NFLAG_SET       0x80
#define NFLAG_CLEAR     0
#define VFLAG_SET       0x80
#define VFLAG_CLEAR     0
#define DFLAG_SET       FLAGPOS_D
#define DFLAG_CLEAR     0
#define IFLAG_SET       FLAGPOS_I
#define IFLAG_CLEAR     0
#define BFLAG_SET       FLAGPOS_B
#define BFLAG_CLEAR     0
#define ZFLAG_SET       0
#define ZFLAG_CLEAR     1
#define CFLAG_SET       0x100
#define CFLAG_CLEAR     0

#define COND_CC()       (!(FLAG_C&0x100))
#define COND_CS()       (FLAG_C&0x100)
#define COND_EQ()       (!FLAG_Z)
#define COND_NE()       FLAG_Z
#define COND_MI()       (FLAG_N&0x80)
#define COND_PL()       (!(FLAG_N&0x80))
#define COND_VC()       (!(FLAG_V&0x80))
#define COND_VS()       (FLAG_V&0x80)

#define VFLAG_ADD_8(S, D, R)    ((S^R) & (D^R))
#define VFLAG_ADD_16(S, D, R)   (((S^R) & (D^R))>>8)
#define VFLAG_SUB_8(S, D, R)    ((S^D) & (R^D))
#define VFLAG_SUB_16(S, D, R)   (((S^D) & (R^D))>>8)

#define CFLAG_8(A)      (A)
#define CFLAG_16(A)     ((A)>>8)
#define NFLAG_8(A)      (A)
#define NFLAG_16(A)     ((A)>>8)

#define CFLAG_1()       ((FLAG_C>>8)&1)


/* ======================================================================== */
/* ================================= MEMORY =============================== */
/* ======================================================================== */

static uint8_t sfr_read(m37710_t *cpu, uint32_t a);
static void sfr_write(m37710_t *cpu, uint32_t a, uint8_t v);
static void update_irqs(m37710_t *cpu);
static void interrupt_software(m37710_t *cpu, uint32_t vector);
static void set_execution_mode(m37710_t *cpu, uint32_t mode);
static void set_irq_line(m37710_t *cpu, int line, int state);

// M37702 map (MAME m37702m2_device::map / m37702s1_device::map): 0x00-0x7f SFRs (noprw where unmapped),
// 0x80-0x27f RAM, 0xc000-0xffff mask ROM (M2). The address space is 24 bits wide.
static inline uint32_t rd8(m37710_t *cpu, uint32_t a)
{
	a &= 0xffffff;
	if (a < 0x280)
		return a < 0x80 ? sfr_read(cpu, a) : cpu->ram[a - 0x80];
	if (cpu->rom && (a & 0xffc000) == 0xc000)
		return cpu->rom[a - 0xc000];
	const uint8_t *p = cpu->rpage[a >> 12];
	if (p)
		return p[a & 0xfff];
	return cpu->read ? cpu->read(cpu->ctx, a) : 0;
}

static inline void wr8(m37710_t *cpu, uint32_t a, uint32_t v)
{
	a &= 0xffffff;
	if (a < 0x280) {
		if (a < 0x80)
			sfr_write(cpu, a, (uint8_t)v);
		else
			cpu->ram[a - 0x80] = (uint8_t)v;
		return;
	}
	if (cpu->rom && (a & 0xffc000) == 0xc000)
		return;
	uint8_t *p = cpu->wpage[a >> 12];
	if (p) {
		p[a & 0xfff] = (uint8_t)v;
		return;
	}
	if (cpu->write)
		cpu->write(cpu->ctx, a, (uint8_t)v);
}

// MAME read_word_unaligned / write_word_unaligned on the 16-bit bus: low byte first
static inline uint32_t rd16(m37710_t *cpu, uint32_t a)
{
	a &= 0xffffff;
	if (!(a & 1) && a >= 0x280) {
		const uint8_t *p;
		if (cpu->rom && (a & 0xffc000) == 0xc000)
			p = cpu->rom + (a - 0xc000);
		else if ((p = cpu->rpage[a >> 12]) != NULL)
			p += a & 0xfff;
		if (p)
			return p[0] | (p[1] << 8);
		if (cpu->read16)
			return cpu->read16(cpu->ctx, a);
	}
	uint32_t lo = rd8(cpu, a);
	return lo | (rd8(cpu, a + 1) << 8);
}

static inline void wr16(m37710_t *cpu, uint32_t a, uint32_t v)
{
	a &= 0xffffff;
	if (!(a & 1) && a >= 0x280 && !(cpu->rom && (a & 0xffc000) == 0xc000)) {
		uint8_t *p = cpu->wpage[a >> 12];
		if (p) {
			p += a & 0xfff;
			p[0] = (uint8_t)v;
			p[1] = (uint8_t)(v >> 8);
			return;
		}
		if (cpu->write16) {
			cpu->write16(cpu->ctx, a, (uint16_t)v);
			return;
		}
	}
	wr8(cpu, a, v & 0xff);
	wr8(cpu, a + 1, (v >> 8) & 0xff);
}

// MAME m37710i_read_24_*
static inline uint32_t rd24(m37710_t *cpu, uint32_t a)
{
	if (a & 1) {
		uint32_t lo = rd8(cpu, a);
		return lo | (rd16(cpu, a + 1) << 8);
	} else {
		uint32_t lo = rd16(cpu, a);
		return lo | (rd8(cpu, a + 2) << 16);
	}
}

#define m37710_read_8(addr)          rd8(cpu, (addr))
#define m37710_write_8(addr, data)   wr8(cpu, (addr), (data))
#define m37710_read_16(addr)         rd16(cpu, (addr))
#define m37710_write_16(addr, data)  wr16(cpu, (addr), (data))

/* Effective-address based memory access macros (MAME m37710cm.h) */
#define read_8_NORM(A) rd8(cpu, (A))
#define read_8_IMM(A) rd8(cpu, (A))
#define read_8_D(A) rd8(cpu, (A))
#define read_8_A(A) rd8(cpu, (A))
#define read_8_AL(A) rd8(cpu, (A))
#define read_8_DX(A) rd8(cpu, (A))
#define read_8_DY(A) rd8(cpu, (A))
#define read_8_AX(A) rd8(cpu, (A))
#define read_8_ALX(A) rd8(cpu, (A))
#define read_8_AY(A) rd8(cpu, (A))
#define read_8_DI(A) rd8(cpu, (A))
#define read_8_DLI(A) rd8(cpu, (A))
#define read_8_AI(A) rd8(cpu, (A))
#define read_8_ALI(A) rd8(cpu, (A))
#define read_8_DXI(A) rd8(cpu, (A))
#define read_8_DIY(A) rd8(cpu, (A))
#define read_8_DLIY(A) rd8(cpu, (A))
#define read_8_AXI(A) rd8(cpu, (A))
#define read_8_S(A) rd8(cpu, (A))
#define read_8_SIY(A) rd8(cpu, (A))
#define read_16_NORM(A) rd16(cpu, (A))
#define read_16_IMM(A) rd16(cpu, (A))
#define read_16_D(A) rd16(cpu, (A))
#define read_16_A(A) rd16(cpu, (A))
#define read_16_AL(A) rd16(cpu, (A))
#define read_16_DX(A) rd16(cpu, (A))
#define read_16_DY(A) rd16(cpu, (A))
#define read_16_AX(A) rd16(cpu, (A))
#define read_16_ALX(A) rd16(cpu, (A))
#define read_16_AY(A) rd16(cpu, (A))
#define read_16_DI(A) rd16(cpu, (A))
#define read_16_DLI(A) rd16(cpu, (A))
#define read_16_AI(A) rd16(cpu, (A))
#define read_16_ALI(A) rd16(cpu, (A))
#define read_16_DXI(A) rd16(cpu, (A))
#define read_16_DIY(A) rd16(cpu, (A))
#define read_16_DLIY(A) rd16(cpu, (A))
#define read_16_AXI(A) rd16(cpu, (A))
#define read_16_S(A) rd16(cpu, (A))
#define read_16_SIY(A) rd16(cpu, (A))
#define read_24_NORM(A) rd24(cpu, (A))
#define read_24_IMM(A) rd24(cpu, (A))
#define read_24_D(A) rd24(cpu, (A))
#define read_24_A(A) rd24(cpu, (A))
#define read_24_AL(A) rd24(cpu, (A))
#define read_24_DX(A) rd24(cpu, (A))
#define read_24_DY(A) rd24(cpu, (A))
#define read_24_AX(A) rd24(cpu, (A))
#define read_24_ALX(A) rd24(cpu, (A))
#define read_24_AY(A) rd24(cpu, (A))
#define read_24_DI(A) rd24(cpu, (A))
#define read_24_DLI(A) rd24(cpu, (A))
#define read_24_AI(A) rd24(cpu, (A))
#define read_24_ALI(A) rd24(cpu, (A))
#define read_24_DXI(A) rd24(cpu, (A))
#define read_24_DIY(A) rd24(cpu, (A))
#define read_24_DLIY(A) rd24(cpu, (A))
#define read_24_AXI(A) rd24(cpu, (A))
#define read_24_S(A) rd24(cpu, (A))
#define read_24_SIY(A) rd24(cpu, (A))
#define write_8_NORM(A, V) wr8(cpu, (A), (V))
#define write_8_D(A, V) wr8(cpu, (A), (V))
#define write_8_A(A, V) wr8(cpu, (A), (V))
#define write_8_AL(A, V) wr8(cpu, (A), (V))
#define write_8_DX(A, V) wr8(cpu, (A), (V))
#define write_8_DY(A, V) wr8(cpu, (A), (V))
#define write_8_AX(A, V) wr8(cpu, (A), (V))
#define write_8_ALX(A, V) wr8(cpu, (A), (V))
#define write_8_AY(A, V) wr8(cpu, (A), (V))
#define write_8_DI(A, V) wr8(cpu, (A), (V))
#define write_8_DLI(A, V) wr8(cpu, (A), (V))
#define write_8_AI(A, V) wr8(cpu, (A), (V))
#define write_8_ALI(A, V) wr8(cpu, (A), (V))
#define write_8_DXI(A, V) wr8(cpu, (A), (V))
#define write_8_DIY(A, V) wr8(cpu, (A), (V))
#define write_8_DLIY(A, V) wr8(cpu, (A), (V))
#define write_8_AXI(A, V) wr8(cpu, (A), (V))
#define write_8_S(A, V) wr8(cpu, (A), (V))
#define write_8_SIY(A, V) wr8(cpu, (A), (V))
#define write_16_NORM(A, V) wr16(cpu, (A), (V))
#define write_16_D(A, V) wr16(cpu, (A), (V))
#define write_16_A(A, V) wr16(cpu, (A), (V))
#define write_16_AL(A, V) wr16(cpu, (A), (V))
#define write_16_DX(A, V) wr16(cpu, (A), (V))
#define write_16_DY(A, V) wr16(cpu, (A), (V))
#define write_16_AX(A, V) wr16(cpu, (A), (V))
#define write_16_ALX(A, V) wr16(cpu, (A), (V))
#define write_16_AY(A, V) wr16(cpu, (A), (V))
#define write_16_DI(A, V) wr16(cpu, (A), (V))
#define write_16_DLI(A, V) wr16(cpu, (A), (V))
#define write_16_AI(A, V) wr16(cpu, (A), (V))
#define write_16_ALI(A, V) wr16(cpu, (A), (V))
#define write_16_DXI(A, V) wr16(cpu, (A), (V))
#define write_16_DIY(A, V) wr16(cpu, (A), (V))
#define write_16_DLIY(A, V) wr16(cpu, (A), (V))
#define write_16_AXI(A, V) wr16(cpu, (A), (V))
#define write_16_S(A, V) wr16(cpu, (A), (V))
#define write_16_SIY(A, V) wr16(cpu, (A), (V))

#define OPER_8_IMM() read_8_IMM(EA_IMM8())
#define OPER_8_D() read_8_D(EA_D())
#define OPER_8_A() read_8_A(EA_A())
#define OPER_8_AL() read_8_AL(EA_AL())
#define OPER_8_DX() read_8_DX(EA_DX())
#define OPER_8_DY() read_8_DY(EA_DY())
#define OPER_8_AX() read_8_AX(EA_AX())
#define OPER_8_ALX() read_8_ALX(EA_ALX())
#define OPER_8_AY() read_8_AY(EA_AY())
#define OPER_8_DI() read_8_DI(EA_DI())
#define OPER_8_DLI() read_8_DLI(EA_DLI())
#define OPER_8_AI() read_8_AI(EA_AI())
#define OPER_8_ALI() read_8_ALI(EA_ALI())
#define OPER_8_DXI() read_8_DXI(EA_DXI())
#define OPER_8_DIY() read_8_DIY(EA_DIY())
#define OPER_8_DLIY() read_8_DLIY(EA_DLIY())
#define OPER_8_AXI() read_8_AXI(EA_AXI())
#define OPER_8_S() read_8_S(EA_S())
#define OPER_8_SIY() read_8_SIY(EA_SIY())
#define OPER_16_IMM() read_16_IMM(EA_IMM16())
#define OPER_16_D() read_16_D(EA_D())
#define OPER_16_A() read_16_A(EA_A())
#define OPER_16_AL() read_16_AL(EA_AL())
#define OPER_16_DX() read_16_DX(EA_DX())
#define OPER_16_DY() read_16_DY(EA_DY())
#define OPER_16_AX() read_16_AX(EA_AX())
#define OPER_16_ALX() read_16_ALX(EA_ALX())
#define OPER_16_AY() read_16_AY(EA_AY())
#define OPER_16_DI() read_16_DI(EA_DI())
#define OPER_16_DLI() read_16_DLI(EA_DLI())
#define OPER_16_AI() read_16_AI(EA_AI())
#define OPER_16_ALI() read_16_ALI(EA_ALI())
#define OPER_16_DXI() read_16_DXI(EA_DXI())
#define OPER_16_DIY() read_16_DIY(EA_DIY())
#define OPER_16_DLIY() read_16_DLIY(EA_DLIY())
#define OPER_16_AXI() read_16_AXI(EA_AXI())
#define OPER_16_S() read_16_S(EA_S())
#define OPER_16_SIY() read_16_SIY(EA_SIY())
#define OPER_24_IMM() read_24_IMM(EA_IMM24())
#define OPER_24_D() read_24_D(EA_D())
#define OPER_24_A() read_24_A(EA_A())
#define OPER_24_AL() read_24_AL(EA_AL())
#define OPER_24_DX() read_24_DX(EA_DX())
#define OPER_24_DY() read_24_DY(EA_DY())
#define OPER_24_AX() read_24_AX(EA_AX())
#define OPER_24_ALX() read_24_ALX(EA_ALX())
#define OPER_24_AY() read_24_AY(EA_AY())
#define OPER_24_DI() read_24_DI(EA_DI())
#define OPER_24_DLI() read_24_DLI(EA_DLI())
#define OPER_24_AI() read_24_AI(EA_AI())
#define OPER_24_ALI() read_24_ALI(EA_ALI())
#define OPER_24_DXI() read_24_DXI(EA_DXI())
#define OPER_24_DIY() read_24_DIY(EA_DIY())
#define OPER_24_DLIY() read_24_DLIY(EA_DLIY())
#define OPER_24_AXI() read_24_AXI(EA_AXI())
#define OPER_24_S() read_24_S(EA_S())
#define OPER_24_SIY() read_24_SIY(EA_SIY())

/* ======================================================================== */
/* ================================= STACK ================================ */
/* ======================================================================== */

static inline void push_8(m37710_t *cpu, uint32_t value)
{
	m37710_write_8(REG_S, value);
	REG_S = MAKE_UINT_16(REG_S-1);
}

static inline uint32_t pull_8(m37710_t *cpu)
{
	REG_S = MAKE_UINT_16(REG_S+1);
	return m37710_read_8(REG_S);
}

static inline void push_16(m37710_t *cpu, uint32_t value)
{
	push_8(cpu, value>>8);
	push_8(cpu, value);
}

static inline uint32_t pull_16(m37710_t *cpu)
{
	uint32_t res = pull_8(cpu);
	return res | (pull_8(cpu) << 8);
}

static inline void push_24(m37710_t *cpu, uint32_t value)
{
	push_8(cpu, value>>16);
	push_8(cpu, (value>>8));
	push_8(cpu, value);
}

static inline uint32_t pull_24(m37710_t *cpu)
{
	uint32_t res = pull_8(cpu);
	res |= pull_8(cpu) << 8;
	return res | (pull_8(cpu) << 16);
}

/* ======================================================================== */
/* ============================ PROGRAM COUNTER =========================== */
/* ======================================================================== */

// (functions, not macros: the argument is often an operand fetch that advances PC, and it must be evaluated
// before PC is read)
static inline void jump_16(m37710_t *cpu, uint32_t address)
{
	REG_PC = MAKE_UINT_16(address);
}

static inline void jump_24(m37710_t *cpu, uint32_t address)
{
	REG_PG = address&0xff0000;
	REG_PC = MAKE_UINT_16(address);
}

static inline void branch_8(m37710_t *cpu, uint32_t offset)
{
	REG_PC = MAKE_UINT_16(REG_PC + MAKE_INT_8(offset));
}

static inline void branch_16(m37710_t *cpu, uint32_t offset)
{
	REG_PC = MAKE_UINT_16(REG_PC + offset);
}

/* ======================================================================== */
/* ============================ STATUS REGISTER =========================== */
/* ======================================================================== */

static inline uint32_t get_reg_ps(const m37710_t *cpu)
{
	return  (FLAG_N&0x80)       |
			((FLAG_V>>1)&0x40)  |
			FLAG_M              |
			FLAG_X              |
			FLAG_D              |
			FLAG_I              |
			((!FLAG_Z)<<1)      |
			((FLAG_C>>8)&1);
}

#define m37710i_push_8(V)               push_8(cpu, (V))
#define m37710i_pull_8()                pull_8(cpu)
#define m37710i_push_16(V)              push_16(cpu, (V))
#define m37710i_pull_16()               pull_16(cpu)
#define m37710i_push_24(V)              push_24(cpu, (V))
#define m37710i_pull_24()               pull_24(cpu)
#define m37710i_jump_16(A)              jump_16(cpu, (A))
#define m37710i_jump_24(A)              jump_24(cpu, (A))
#define m37710i_branch_8(O)             branch_8(cpu, (O))
#define m37710i_branch_16(O)            branch_16(cpu, (O))
#define m37710i_get_reg_ps()            get_reg_ps(cpu)
#define m37710i_set_reg_ipl(V)          (REG_IPL = (V) & 7)
#define m37710i_update_irqs()           update_irqs(cpu)
#define m37710i_interrupt_software(V)   interrupt_software(cpu, (V))
#define m37710i_set_execution_mode(C, M) set_execution_mode((C), (M))

/* ======================================================================== */
/* ============================= ADDRESS MODES ============================ */
/* ======================================================================== */

static inline uint32_t ea_imm8(m37710_t *cpu)  {REG_PC += 1; return REG_PG | MAKE_UINT_16(REG_PC-1);}
static inline uint32_t ea_imm16(m37710_t *cpu) {REG_PC += 2; return REG_PG | MAKE_UINT_16(REG_PC-2);}
static inline uint32_t ea_imm24(m37710_t *cpu) {REG_PC += 3; return REG_PG | MAKE_UINT_16(REG_PC-3);}
#define EA_IMM8()   ea_imm8(cpu)
#define EA_IMM16()  ea_imm16(cpu)
#define EA_IMM24()  ea_imm24(cpu)
static inline uint32_t ea_d(m37710_t *cpu)     {if(MAKE_UINT_8(REG_DPR)) CLK(1); return MAKE_UINT_16(REG_DPR + OPER_8_IMM());}
#define EA_D()      ea_d(cpu)
static inline uint32_t ea_a(m37710_t *cpu)     {return REG_DT | OPER_16_IMM();}
#define EA_A()      ea_a(cpu)
static inline uint32_t ea_al(m37710_t *cpu)    {return OPER_24_IMM();}
#define EA_AL()     ea_al(cpu)
static inline uint32_t ea_dx(m37710_t *cpu)    {return MAKE_UINT_16(REG_DPR + OPER_8_IMM() + REG_X);}
#define EA_DX()     ea_dx(cpu)
static inline uint32_t ea_dy(m37710_t *cpu)    {return MAKE_UINT_16(REG_DPR + OPER_8_IMM() + REG_Y);}
#define EA_DY()     ea_dy(cpu)
static inline uint32_t ea_ax(m37710_t *cpu)    {uint32_t tmp = EA_A(); if((tmp^(tmp+REG_X))&0xff00) CLK(1); return tmp + REG_X;}
#define EA_AX()     ea_ax(cpu)
static inline uint32_t ea_alx(m37710_t *cpu)   {return EA_AL() + REG_X;}
#define EA_ALX()    ea_alx(cpu)
// (MAME: the page-cross test uses X)
static inline uint32_t ea_ay(m37710_t *cpu)    {uint32_t tmp = EA_A(); if((tmp^(tmp+REG_X))&0xff00) CLK(1); return tmp + REG_Y;}
#define EA_AY()     ea_ay(cpu)
static inline uint32_t ea_di(m37710_t *cpu)    {return REG_DT | OPER_16_D();}
#define EA_DI()     ea_di(cpu)
static inline uint32_t ea_dli(m37710_t *cpu)   {return OPER_24_D();}
#define EA_DLI()    ea_dli(cpu)
static inline uint32_t ea_ai(m37710_t *cpu)    {return read_16_A(OPER_16_IMM());}
#define EA_AI()     ea_ai(cpu)
static inline uint32_t ea_ali(m37710_t *cpu)   {return OPER_24_A();}
#define EA_ALI()    ea_ali(cpu)
static inline uint32_t ea_dxi(m37710_t *cpu)   {return REG_DT | OPER_16_DX();}
#define EA_DXI()    ea_dxi(cpu)
static inline uint32_t ea_diy(m37710_t *cpu)   {uint32_t tmp = REG_DT | OPER_16_D(); if((tmp^(tmp+REG_X))&0xff00) CLK(1); return tmp + REG_Y;}
#define EA_DIY()    ea_diy(cpu)
static inline uint32_t ea_dliy(m37710_t *cpu)  {return OPER_24_D() + REG_Y;}
#define EA_DLIY()   ea_dliy(cpu)
static inline uint32_t ea_axi(m37710_t *cpu)   {return read_16_AXI(MAKE_UINT_16(OPER_16_IMM() + REG_X));}
#define EA_AXI()    ea_axi(cpu)
static inline uint32_t ea_s(m37710_t *cpu)     {return MAKE_UINT_16(REG_S + OPER_8_IMM());}
#define EA_S()      ea_s(cpu)
static inline uint32_t ea_siy(m37710_t *cpu)   {return MAKE_UINT_16(read_16_SIY(REG_S + OPER_8_IMM()) + REG_Y) | REG_DT;}
#define EA_SIY()    ea_siy(cpu)

/* ======================================================================== */
/* ======================= INTERNAL PERIPHERALS (SFR) ===================== */
/* ======================================================================== */

// the CPU's local time: the cycles accounted so far plus those of the instruction in progress (MAME adjusts
// its emu_timers from the CPU's local_time(), the same)
#define NOW(c) ((c)->cycles + ((c)->insn_icount - (c)->icount))
// an event due at cycle `due` is handled at the first instruction boundary at or after due - early (see
// m37710_set_clock)
#define DUE(c, due) ((due) - (c)->early <= (c)->cycles)

static void recalc_next_due(m37710_t *cpu)
{
	int64_t n = INT64_MAX;
	for (int i = 0; i < 8; i++)
		if (cpu->timer_on[i] && cpu->timer_due[i] < n)
			n = cpu->timer_due[i];
	if (cpu->ad_on && cpu->ad_due < n)
		n = cpu->ad_due;
	cpu->next_due = n == INT64_MAX ? n : n - cpu->early;
}

// MAME m37710_recalc_timer (the periods in CPU cycles: clocks / 2)
static void recalc_timer(m37710_t *cpu, int timer)
{
	static const int tscales[4] = { 2, 16, 64, 512 };

	// check if enabled
	if (BIT(cpu->count_start, timer)) {
		int tval = cpu->timer_reg[timer];

		// HACK: ignore if timer is 8MHz (MAME slows down to a crawl)
		if (tval == 0 && (cpu->timer_mode[timer]&0xc0) == 0) return;

		// timer mode (groups A and B alike); the event counter / one-shot / PWM / measurement modes start
		// nothing (MAME logs them only)
		if ((cpu->timer_mode[timer] & 0x3) == 0) {
			int64_t period = (int64_t)(tscales[cpu->timer_mode[timer]>>6] / 2) * (tval + 1);
			cpu->timer_due[timer] = NOW(cpu) + period;
			cpu->timer_on[timer] = 1;
			cpu->timer_reload[timer] = period;
			recalc_next_due(cpu);
		}
	}
}

// MAME m37710_external_tick
static void external_tick(m37710_t *cpu, int timer, int state)
{
	// we only care if the state is "on"
	if (!state)
		return;

	// check if enabled and in event counter mode
	if (BIT(cpu->count_start, timer)) {
		if ((cpu->timer_mode[timer] & 0x3) == 1) {
			int upcount = 0;

			// timer b always counts down
			if (timer <= 4) {
				if (BIT(cpu->timer_mode[timer], 4))
					upcount = cpu->timer_out[timer];   // up/down determined by timer out pin
				else
					upcount = cpu->up_down_reg >> timer & 1;
			}

			if (upcount)
				cpu->timer_reg[timer]++;
			else
				cpu->timer_reg[timer]--;
		}
	}
}

static uint8_t get_port_reg(m37710_t *cpu, int p)
{
	uint8_t d = cpu->port_dir[p];
	if (d != 0xff) {
		uint8_t in = cpu->port_read ? cpu->port_read(cpu->ctx, p) : 0xff;
		return (uint8_t)((in & ~d) | (cpu->port_regs[p] & d));
	}
	return cpu->port_regs[p];
}

static void set_port_reg(m37710_t *cpu, int p, uint8_t data)
{
	uint8_t d = cpu->port_dir[p];
	if (d != 0 && cpu->port_write)
		cpu->port_write(cpu->ctx, p, data & d, d);
	cpu->port_regs[p] = data;
}

// A-D conversion time in CPU cycles: MAME 57 * 2 * (f2/2 : f2/4) clocks
#define AD_CYCLES(ctl) (57 * (BIT((ctl), 7) ? 2 : 4))

static void ad_control_w(m37710_t *cpu, uint8_t data)
{
	if (BIT(data, 6) && !BIT(cpu->ad_control, 6)) {
		// A-D conversion clock may be selected as f2/4 or f2/2
		cpu->ad_on = 1;
		cpu->ad_due = NOW(cpu) + AD_CYCLES(data);
		recalc_next_due(cpu);
		if (BIT(data, 4))
			data &= 0xf8;
	} else if (!BIT(data, 6)) {
		cpu->ad_on = 0;
		recalc_next_due(cpu);
	}

	cpu->ad_control = data;
}

// MAME ad_timer_cb (re-armed from its expiry time, as MAME's adjust inside a timer callback)
static void ad_timer_cb(m37710_t *cpu)
{
	int line = cpu->ad_control & 0x07;

	cpu->ad_on = 0;
	cpu->ad_result[line] = cpu->analog_read ? cpu->analog_read(cpu->ctx, line) : 0;

	if (BIT(cpu->ad_control, 4))
		cpu->ad_control = (uint8_t)((cpu->ad_control & 0xf8) | ((line + 1) & 0x07));

	// repeat or sweep conversion
	if (BIT(cpu->ad_control, 3) || (BIT(cpu->ad_control, 4) && line != (cpu->ad_sweep & 0x03) * 2 + 1)) {
		cpu->ad_on = 1;
		cpu->ad_due += AD_CYCLES(cpu->ad_control);
	} else {
		// interrupt occurs only when conversion stops
		set_irq_line(cpu, M37710_LINE_ADC, 1);
		cpu->ad_control &= 0xbf;
	}
}

static void count_start_w(m37710_t *cpu, uint8_t data)
{
	uint8_t prevdata = cpu->count_start;
	cpu->count_start = data;

	for (int i = 0; i < 8; i++)
		if (BIT(data, i) && !BIT(prevdata, i))
			recalc_timer(cpu, i);
}

static void set_int_control(m37710_t *cpu, int level, uint8_t data)
{
	cpu->int_control[level] = data;
	update_irqs(cpu);
	// level-sense interrupts are not implemented yet (MAME)
}

// 0x70-0x7f interrupt control registers -> level (MAME ad / uart / timer / irq register maps)
static const int8_t s_int_level[16] = {
	M37710_LINE_ADC, M37710_LINE_UART0XMIT, M37710_LINE_UART0RECV, M37710_LINE_UART1XMIT, M37710_LINE_UART1RECV,
	M37710_LINE_TIMERA0, M37710_LINE_TIMERA1, M37710_LINE_TIMERA2, M37710_LINE_TIMERA3, M37710_LINE_TIMERA4,
	M37710_LINE_TIMERB0, M37710_LINE_TIMERB1, M37710_LINE_TIMERB2,
	M37710_LINE_IRQ0, M37710_LINE_IRQ1, M37710_LINE_IRQ2
};

// ports (MAME port_r<0> umask 0x00ff at 0x02-0x15, port_r<1> umask 0xff00 at 0x02-0x11): returns the port
// number and sets *dir, or -1
static inline int port_decode(uint32_t a, int *dir)
{
	int off, p;
	if (!(a & 1)) {
		off = (int)(a - 2) >> 1;
		p = off & ~1;
	} else {
		if (a > 0x11)
			return -1;
		off = (int)(a - 3) >> 1;
		p = (off & ~1) + 1;
	}
	*dir = off & 1;
	return p;
}

static uint8_t sfr_read(m37710_t *cpu, uint32_t a)
{
	if (a >= 0x02 && a <= 0x15) {
		int dir, p = port_decode(a, &dir);
		if (p < 0)
			return 0;
		return dir ? cpu->port_dir[p] : get_port_reg(cpu, p);
	}
	if (a >= 0x20 && a <= 0x2f) {
		uint16_t r = cpu->ad_result[(a - 0x20) >> 1];
		return (uint8_t)((a & 1) ? r >> 8 : r);
	}
	if (a >= 0x46 && a <= 0x55) {
		uint16_t r = cpu->timer_reg[(a - 0x46) >> 1];
		return (uint8_t)((a & 1) ? r >> 8 : r);
	}
	if (a >= 0x56 && a <= 0x5d)
		return cpu->timer_mode[a - 0x56];
	if (a >= 0x70)
		return cpu->int_control[s_int_level[a - 0x70]];
	switch (a) {
	case 0x1e: return cpu->ad_control;
	case 0x1f: return cpu->ad_sweep;
	case 0x30: return cpu->uart_mode[0];
	case 0x34: return cpu->uart_ctrl_reg0[0];
	case 0x35: return cpu->uart_ctrl_reg1[0];
	case 0x38: return cpu->uart_mode[1];
	case 0x3c: return cpu->uart_ctrl_reg0[1];
	case 0x3d: return cpu->uart_ctrl_reg1[1];
	case 0x40: return cpu->count_start;
	case 0x44: return cpu->up_down_reg & 0x1f;   // bits 7-5 read back as 0
	case 0x5e: return cpu->proc_mode & 0xf7;
	case 0x61: return cpu->watchdog_freq;
	default:   return 0;                           // (UART receive buffers read 0; the rest noprw)
	}
}

static void sfr_write(m37710_t *cpu, uint32_t a, uint8_t v)
{
	if (a >= 0x02 && a <= 0x15) {
		int dir, p = port_decode(a, &dir);
		if (p < 0)
			return;
		if (dir)
			cpu->port_dir[p] = v;
		else
			set_port_reg(cpu, p, v);
		return;
	}
	if (a >= 0x46 && a <= 0x55) {
		int t = (int)(a - 0x46) >> 1;
		if (a & 1)
			cpu->timer_reg[t] = (uint16_t)((cpu->timer_reg[t] & 0x00ff) | (v << 8));
		else
			cpu->timer_reg[t] = (uint16_t)((cpu->timer_reg[t] & 0xff00) | v);
		return;
	}
	if (a >= 0x56 && a <= 0x5d) {
		cpu->timer_mode[a - 0x56] = v;
		return;
	}
	if (a >= 0x70) {
		set_int_control(cpu, s_int_level[a - 0x70], v);
		return;
	}
	switch (a) {
	case 0x1e: ad_control_w(cpu, v); break;
	case 0x1f: cpu->ad_sweep = v; break;
	case 0x30: cpu->uart_mode[0] = v; break;
	case 0x31: cpu->uart_baud[0] = v; break;
	case 0x34: cpu->uart_ctrl_reg0[0] = (uint8_t)((v & ~8) | (cpu->uart_ctrl_reg0[0] & 8)); break; // Tx empty flag is read-only
	case 0x35: cpu->uart_ctrl_reg1[0] = (uint8_t)((cpu->uart_ctrl_reg1[0] & (BIT(v, 2) ? 0xfa : 0x0a)) | (v & 0x05)); break;
	case 0x38: cpu->uart_mode[1] = v; break;
	case 0x39: cpu->uart_baud[1] = v; break;
	case 0x3c: cpu->uart_ctrl_reg0[1] = (uint8_t)((v & ~8) | (cpu->uart_ctrl_reg0[1] & 8)); break;
	case 0x3d: cpu->uart_ctrl_reg1[1] = (uint8_t)((cpu->uart_ctrl_reg1[1] & (BIT(v, 2) ? 0xfa : 0x0a)) | (v & 0x05)); break;
	case 0x40: count_start_w(cpu, v); break;
	case 0x42: cpu->one_shot_start = v; break;
	case 0x44: cpu->up_down_reg = v; break;
	case 0x5e: cpu->proc_mode = v; break;
	case 0x61: cpu->watchdog_freq = v; break;
	default:   break;   // (0x32/0x33, 0x3a/0x3b transmit buffers, 0x60 watchdog reset: no effect in MAME)
	}
}

/* ======================================================================== */
/* =============================== INTERRUPTS ============================= */
/* ======================================================================== */

static const uint16_t s_irq_vectors[M37710_INTERRUPT_MAX] =
{
	// maskable
	0xffce, // DMA3
	0xffd0, // DMA2
	0xffd2, // DMA1
	0xffd4, // DMA0
	0xffd6, // A-D converter
	0xffd8, // UART1 transmit
	0xffda, // UART1 receive
	0xffdc, // UART0 transmit
	0xffde, // UART0 receive
	0xffe0, // Timer B2
	0xffe2, // Timer B1
	0xffe4, // Timer B0
	0xffe6, // Timer A4
	0xffe8, // Timer A3
	0xffea, // Timer A2
	0xffec, // Timer A1
	0xffee, // Timer A0
	0xfff0, // external INT2 pin
	0xfff2, // external INT1 pin
	0xfff4, // external INT0 pin

	// non-maskable
	0xfff6, // watchdog timer
	0xfff8, // debugger control (not used in shipping ICs?)
	0xfffa, // BRK
	0xfffc, // divide by zero
	0xfffe, // RESET
};

// MAME m37710_set_irq_line (maskable lines only)
static void set_irq_line(m37710_t *cpu, int line, int state)
{
	if (line < 0 || line >= M37710_MASKABLE_INTERRUPTS)
		return;
	if (!state) {
		LINE_IRQ &= ~(1u << line);
		cpu->int_control[line] &= ~8;
	} else {
		LINE_IRQ |= (1u << line);
		cpu->int_control[line] |= 8;
	}
}

// MAME m37710i_update_irqs
static void update_irqs(m37710_t *cpu)
{
	int curirq, pending = (int)LINE_IRQ;
	int wantedIRQ = -1;
	int curpri = 0;

	for (curirq = M37710_INTERRUPT_MAX - 1; curirq >= 0; curirq--) {
		if ((pending & (1 << curirq))) {
			// this IRQ is set
			if (curirq < M37710_MASKABLE_INTERRUPTS) {
				int control = cpu->int_control[curirq];
				int thispri = control & 7;
				// it's maskable, check if the level works, also make sure it's acceptable for the current CPU level
				if (!FLAG_I && thispri > curpri && thispri > (int)REG_IPL) {
					// mark us as the best candidate
					wantedIRQ = curirq;
					curpri = thispri;
				}
			} else {
				// non-maskable
				wantedIRQ = curirq;
				curpri = 7;
				break;  // no more processing, NMIs always win
			}
		}
	}

	if (wantedIRQ != -1) {
		// make sure we're running to service the interrupt
		CPU_STOPPED &= ~STOP_LEVEL_WAI;

		// auto-clear line
		set_irq_line(cpu, wantedIRQ, 0);

		// let's do it...
		// push PB, then PC, then status
		CLK(13);
		m37710i_push_8(REG_PG>>16);
		m37710i_push_16(REG_PC);
		m37710i_push_8(REG_IPL);
		m37710i_push_8(m37710i_get_reg_ps());

		// set I to 1, set IPL to the interrupt we're taking
		FLAG_I = IFLAG_SET;
		REG_IPL = (uint32_t)curpri;
		// then PG=0, PC=(vector)
		REG_PG = 0;
		REG_PC = m37710_read_16(s_irq_vectors[wantedIRQ]);
	}
}

// MAME m37710i_interrupt_software
static void interrupt_software(m37710_t *cpu, uint32_t vector)
{
	CLK(13);
	m37710i_push_8(REG_PG>>16);
	m37710i_push_16(REG_PC);
	m37710i_push_8(REG_IPL);
	m37710i_push_8(m37710i_get_reg_ps());
	FLAG_I = IFLAG_SET;
	REG_PG = 0;
	REG_PC = m37710_read_16(vector);
}

/* ======================================================================== */
/* ================================ OPCODES =============================== */
/* ======================================================================== */

#define EXECUTION_MODE EXECUTION_MODE_M0X0
#include "m37710op.h"
#undef EXECUTION_MODE
#define EXECUTION_MODE EXECUTION_MODE_M0X1
#include "m37710op.h"
#undef EXECUTION_MODE
#define EXECUTION_MODE EXECUTION_MODE_M1X0
#include "m37710op.h"
#undef EXECUTION_MODE
#define EXECUTION_MODE EXECUTION_MODE_M1X1
#include "m37710op.h"
#undef EXECUTION_MODE

static const m37710_opfn *const s_opcodes[4] =
{
	m37710i_opcodes_M0X0, m37710i_opcodes_M0X1, m37710i_opcodes_M1X0, m37710i_opcodes_M1X1,
};
static const m37710_opfn *const s_opcodes42[4] =
{
	m37710i_opcodes42_M0X0, m37710i_opcodes42_M0X1, m37710i_opcodes42_M1X0, m37710i_opcodes42_M1X1,
};
static const m37710_opfn *const s_opcodes89[4] =
{
	m37710i_opcodes89_M0X0, m37710i_opcodes89_M0X1, m37710i_opcodes89_M1X0, m37710i_opcodes89_M1X1,
};

static void set_execution_mode(m37710_t *cpu, uint32_t mode)
{
	cpu->opcodes = s_opcodes[mode];
	cpu->opcodes42 = s_opcodes42[mode];
	cpu->opcodes89 = s_opcodes89[mode];
}

/* ======================================================================== */
/* ============================ EXTERNAL API ============================== */
/* ======================================================================== */

// fire the internal timers due by cpu->cycles (MAME m37710_timer_cb / ad_timer_cb), then evaluate the
// interrupts (MAME: signal_interrupt_trigger ends the timeslice, execute_run calls m37710i_update_irqs)
static void run_events(m37710_t *cpu)
{
	for (int i = 0; i < 8; i++) {
		if (cpu->timer_on[i] && DUE(cpu, cpu->timer_due[i])) {
			while (DUE(cpu, cpu->timer_due[i]))
				cpu->timer_due[i] += cpu->timer_reload[i];
			set_irq_line(cpu, M37710_LINE_TIMERA0 - i, 1);
		}
	}
	if (cpu->ad_on && DUE(cpu, cpu->ad_due))
		ad_timer_cb(cpu);
	recalc_next_due(cpu);
	update_irqs(cpu);
}

// add the cycles used since the last accounting point to the time base; fire what is due
static inline void account(m37710_t *cpu)
{
	for (;;) {
		cpu->cycles += cpu->insn_icount - cpu->icount;
		cpu->insn_icount = cpu->icount;
		if (cpu->cycles < cpu->next_due)
			break;
		run_events(cpu);
	}
}

// (the cheats' access to this CPU's program space: cheat_cpu.h)
static uint8_t cheat_rd(void *x, uint32_t a) { m37710_t *c = (m37710_t *)x; return c->read ? c->read(c->ctx, a) : 0; }
static void cheat_wr(void *x, uint32_t a, uint8_t v) { m37710_t *c = (m37710_t *)x; if (c->write) c->write(c->ctx, a, v); }

void m37710_init(m37710_t *c)
{
	rbx_cheat_offer("m37710", cheat_rd, cheat_wr, c, 0, 1);
	memset(c, 0, sizeof(*c));
	c->next_due = INT64_MAX;
	set_execution_mode(c, EXECUTION_MODE_M0X0);
}

void m37710_reset(m37710_t *cpu)
{
	int i;

	cpu->idle_cnt = 0;

	/* Reset MAME timers */
	for (i = 0; i < 8; i++) {
		cpu->timer_on[i] = 0;
		cpu->timer_reload[i] = 0;
	}

	/* Start the CPU */
	CPU_STOPPED = 0;

	/* Reset internal registers */
	// port direction
	memset(cpu->port_dir, 0, sizeof(cpu->port_dir));

	// A-D
	cpu->ad_control &= 7;
	cpu->ad_sweep = (uint8_t)((cpu->ad_sweep & ~0xdc) | 3);
	cpu->ad_on = 0;

	// UARTs
	for (i = 0; i < 2; i++) {
		cpu->uart_mode[i] = 0;
		cpu->uart_ctrl_reg0[i] = (uint8_t)((cpu->uart_ctrl_reg0[i] & 0xe0) | 8);
		cpu->uart_ctrl_reg1[i] = 2;
	}

	// timers
	cpu->count_start = 0;
	cpu->one_shot_start &= ~0x1f;
	cpu->up_down_reg = 0;
	for (i = 0; i < 8; i++)
		cpu->timer_mode[i] = 0;
	recalc_next_due(cpu);

	cpu->proc_mode = 0; // processor mode
	cpu->watchdog_freq &= ~1; // watchdog timer frequency
	cpu->rto_control &= ~3;
	cpu->dram_control &= ~0x8f;

	// interrupt control
	for (i = 0; i <= M37710_LINE_TIMERA0; i++)
		cpu->int_control[i] &= ~0xf;
	for (i = M37710_LINE_IRQ2; i <= M37710_LINE_IRQ0; i++)
		cpu->int_control[i] &= ~0x3f;

	/* Clear IPL, m, x, D and set I */
	REG_IPL = 0;
	FLAG_M = MFLAG_CLEAR;
	FLAG_X = XFLAG_CLEAR;
	FLAG_D = DFLAG_CLEAR;
	FLAG_I = IFLAG_SET;

	/* Clear all pending interrupts (should we really do this?) */
	LINE_IRQ = 0;
	IRQ_DELAY = 0;

	/* 37710 boots in full native mode */
	REG_DPR = 0;
	REG_PG = 0;
	REG_DT = 0;
	REG_S = (REG_S & 0xff) | 0x100;
	REG_XH = REG_X & 0xff00; REG_X &= 0xff;
	REG_YH = REG_Y & 0xff00; REG_Y &= 0xff;
	REG_B = REG_A & 0xff00; REG_A &= 0xff;
	REG_BB = REG_BA & 0xff00; REG_BA &= 0xff;

	/* Set the function tables to emulation mode */
	set_execution_mode(cpu, EXECUTION_MODE_M0X0);

	/* Fetch the reset vector */
	REG_PC = m37710_read_16(0xfffe);
}

void m37710_set_input(m37710_t *c, int line, int state)
{
	switch (line) {
	case M37710_LINE_ADC:
	case M37710_LINE_IRQ0:
	case M37710_LINE_IRQ1:
	case M37710_LINE_IRQ2:
		set_irq_line(c, line, state);
		break;
	default:
		if (line >= M37710_LINE_TIMERA0IN && line <= M37710_LINE_TIMERB2IN)
			external_tick(c, line - M37710_LINE_TIMERA0IN, state);
		else if (line >= M37710_LINE_TIMERA0OUT && line <= M37710_LINE_TIMERB2OUT)
			c->timer_out[line - M37710_LINE_TIMERA0OUT] = state ? 1 : 0;
		break;
	}
}

// at the top of the wait loop LDA d ; BEQ *-2 after a turn of it (m37710_t.idle_skip): the whole turns left to
// the slice's end or the next internal event skipped (the A register and the flags are those of the turn just run)
static void idle_loop(m37710_t *cpu)
{
	const uint32_t a = REG_PG | REG_PC;
	if (a != cpu->idle_pc)
		return;
	const int64_t t = cpu->cycles - cpu->idle_c0;
	const int turn = cpu->idle_cnt == 2;
	cpu->idle_c0 = cpu->cycles;
	cpu->idle_cnt = 0;
	if (!turn || !FLAG_M || t <= 0)
		return;
	const uint32_t d = (REG_DPR + read_8_IMM(a + 1)) & 0xffff;
	if (d < 0x80 || d >= 0x280 || cpu->ram[d - 0x80] != 0)
		return;
	int64_t n = (cpu->icount - 1) / t;
	if (cpu->next_due != INT64_MAX && (cpu->next_due - 1 - cpu->cycles) / t < n)
		n = (cpu->next_due - 1 - cpu->cycles) / t;
	if (n <= 0)
		return;
	cpu->icount -= (int32_t)(n * t);
	account(cpu);
	cpu->idle_c0 = cpu->cycles;
}

int m37710_execute(m37710_t *cpu, int cycles)
{
	const int64_t t0 = cpu->cycles;
	cpu->icount += cycles;
	cpu->insn_icount = cpu->icount;

	// MAME execute_run
	update_irqs(cpu);
	account(cpu);

	while (cpu->icount > 0) {
		if (CPU_STOPPED) {
			// WAI / STP: idle to the end of the slice or to the next internal timer event (it may wake WAI)
			int64_t n = cpu->icount;
			if (cpu->next_due - cpu->cycles < n)
				n = cpu->next_due - cpu->cycles;
			if (n < 1)
				n = 1;
			cpu->icount -= (int32_t)n;
			account(cpu);
			continue;
		}
		if (cpu->idle_skip) {
			idle_loop(cpu);
			cpu->idle_cnt++;
		}
		REG_PPC = REG_PC;
#ifdef M37710_TRACE
		m37710_trace_hook(cpu);
#endif
		REG_PC++;
		REG_IR = read_8_IMM(REG_PG | REG_PPC);
		if (REG_IR == 0xa5 && cpu->idle_skip && cpu->idle_pc != (REG_PG | REG_PPC) &&
		    read_8_IMM(REG_PG | ((REG_PPC + 2) & 0xffff)) == 0xf0 && read_8_IMM(REG_PG | ((REG_PPC + 3) & 0xffff)) == 0xfc) {
			cpu->idle_pc = REG_PG | REG_PPC;   // (a new wait loop: its turns measured from here)
			cpu->idle_cnt = 1;
			cpu->idle_c0 = cpu->cycles;
		}
		cpu->opcodes[REG_IR](cpu);
		account(cpu);
	}
	return (int)(cpu->cycles - t0);
}

void m37710_set_clock(m37710_t *c, uint32_t hz)
{
	// MAME times the emu_timers with clocks_to_attotime (whole attoseconds per input clock) and the CPU with
	// attoseconds per cycle (clock / 2): when 2 * floor(1e18 / hz) < floor(1e18 / (hz / 2)) every timer period
	// is a few attoseconds short of its cycle count, the scheduler ends the timeslice a cycle before it, and the
	// event lands one cycle early
	const uint64_t as = 1000000000000000000ull;
	c->early = (2 * (as / hz) < as / ((hz + 1) / 2)) ? 1 : 0;
	recalc_next_due(c);
}

uint32_t m37710_get_ps(const m37710_t *cpu)
{
	return get_reg_ps(cpu);
}

void m37710_post_load(m37710_t *cpu)
{
	cpu->idle_cnt = 0;   // (no turn measured across a load)
	// restore proper function pointers (MAME m37710_restore_state)
	set_execution_mode(cpu, (FLAG_M>>4) | (FLAG_X>>4));
}
