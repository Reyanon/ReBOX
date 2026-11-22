/*  Toshiba TLCS-90 series MCUs, ported to C99 from MAME src/devices/cpu/tlcs90/tlcs90.cpp
    (license: BSD-3-Clause, copyright-holders: Luca Elia; "emulation by Luca Elia, based on the Z80 core
    by Juergen Buchmueller").
    This file is part of ReBOX. GPL-2.0-or-later.

    Differences from MAME (structure only):
    - the internal timers are stepped by the clocks each instruction takes instead of emu_timer callbacks
      (same period: 8 clocks x prescaler, first tick one period after the start, as timer->adjust(p, i, p));
    - opcodes MAME stops on with fatalerror (TSET, LDA, CPD, CPDR, undefined) run as a NOP of their decoded length;
    - a HALT with nothing to wake it skips ahead to the next timer tick or the end of the slice (same result
      as running the halted NOP loop one step at a time).
*/
#include "tlcs90.h"

#include <string.h>

// Regs

#define B   0
#define C   1
#define D   2
#define E   3
#define H   4
#define L   5
#define A   6

#define BC  0
#define DE  1
#define HL  2
//          3
#define IX  4
#define IY  5
#define SP  6

#define AF  7
#define AF2 8
#define PC  9

#define F   t->af.b.l

// Condition Codes

#define FLS 0x0
#define LT  0x1
#define LE  0x2
#define ULE 0x3
#define OV  0x4
#define PE  0x4
#define MI  0x5
#define Z   0x6
#define EQ  0x6
#define CR  0x7
#define ULT 0x7
#define T   0x8
#define GE  0x9
#define GT  0xa
#define UGT 0xb
#define NOV 0xc
#define PO  0xc
#define PL  0xd
#define NZ  0xe
#define NE  0xe
#define NC  0xf
#define UGE 0xf

#define CF  0x01
#define NF  0x02
#define PF  0x04
#define VF  PF
#define XCF 0x08
#define HF  0x10
#define IF  0x20
#define ZF  0x40
#define SF  0x80

static uint8_t SZ[256];       /* zero and sign flags */
static uint8_t SZ_BIT[256];   /* zero, sign and parity/overflow (=zero) flags for BIT opcode */
static uint8_t SZP[256];      /* zero, sign and parity flags */
static uint8_t SZHV_inc[256]; /* zero, sign, half carry and overflow flags INC r8 */
static uint8_t SZHV_dec[256]; /* zero, sign, half carry and overflow flags DEC r8 */
static int s_tables_done;

enum {
	UNKNOWN,
	NOP,    EX,     EXX,    LD,     LDW,    LDA,    LDI,    LDIR,
	LDD,    LDDR,   CPI,    CPIR,   CPD,    CPDR,   PUSH,   POP,
	JP,     JR,     CALL,   CALLR,  RET,    RETI,   HALT,   DI,
	EI,     SWI,    DAA,    CPL,    NEG,    LDAR,   RCF,    SCF,
	CCF,    TSET,   BIT,    SET,    RES,    INC,    DEC,    INCX,
	DECX,   INCW,   DECW,   ADD,    ADC,    SUB,    SBC,    AND,
	XOR,    OR,     CP,     RLC,    RRC,    RL,     RR,     SLA,
	SRA,    SLL,    SRL,    RLD,    RRD,    DJNZ,   MUL,    DIV
};

enum {
	TM_NONE,   TM_BIT8,   TM_CC,
	TM_I8,     TM_D8,     TM_R8,
	TM_I16,    TM_D16,    TM_R16,
	TM_MI16,   TM_MR16,   TM_MR16D8, TM_MR16R8,
	TM_R16D8,  TM_R16R8
};

// Opcodes

#define OP_16 0x80


#define OP(   X,CT )       t->op = X; t->cyc_t = (CT*2);
#define OP16( X,CT )       OP( (X)|OP_16,CT )

#define OPCC(   X,CF,CT )  OP( X, CT ) t->cyc_f = (CF*2);
#define OPCC16( X,CF,CT )  OPCC( (X)|OP_16,CF,CT )

#define BIT8( N,I )        t->mode##N = TM_BIT8;    t->r##N = I;
#define I8( N,I )          t->mode##N = TM_I8;      t->r##N = I;
#define D8( N,I )          t->mode##N = TM_D8;      t->r##N = I;
#define I16( N,I )         t->mode##N = TM_I16;     t->r##N = I;
#define D16( N,I )         t->mode##N = TM_D16;     t->r##N = I;
#define R8( N,R )          t->mode##N = TM_R8;      t->r##N = R;
#define R16( N,R )         t->mode##N = TM_R16;     t->r##N = R;
#define Q16( N,R )         t->mode##N = TM_R16;     t->r##N = R; if (t->r##N == SP) t->r##N = AF;
#define MI16( N,I )        t->mode##N = TM_MI16;    t->r##N = I;
#define MR16( N,R )        t->mode##N = TM_MR16;    t->r##N = R;
#define MR16D8( N,R,I )    t->mode##N = TM_MR16D8;  t->r##N = R; t->r##N##b = I;
#define MR16R8( N,R,g )    t->mode##N = TM_MR16R8;  t->r##N = R; t->r##N##b = g;
#define NONE( N )          t->mode##N = TM_NONE;
#define CC( N,cc )         t->mode##N = TM_CC;      t->r##N = cc;
#define R16D8( N,R,I )     t->mode##N = TM_R16D8;   t->r##N = R; t->r##N##b = I;
#define R16R8( N,R,g )     t->mode##N = TM_R16R8;   t->r##N = R; t->r##N##b = g;

/*************************************************************************************************************
    Internal registers (MAME tmp90840_regs / tmp90844_regs handlers)
*************************************************************************************************************/

static void raise_irq(tlcs90 *t, int irq) { t->irq_state |= 1 << irq; }
static void clear_irq(tlcs90 *t, int irq) { t->irq_state &= ~(1 << irq); }

static uint8_t port_in(tlcs90 *t, int p) { return t->port_r ? t->port_r(t->userdata, p) : 0xff; }
static void port_out(tlcs90 *t, int p, uint8_t v) { if (t->port_w) t->port_w(t->userdata, p, v); }

static uint8_t p1_r(tlcs90 *t)
{
	if ((t->p01cr & 0x04) != 0) return 0;   // configured as part of the address bus
	if ((t->p01cr & 0x02) != 0) return 0;   // configured as output
	return port_in(t, 1);
}

static void p1_w(tlcs90 *t, uint8_t data)
{
	(void)data;
	if ((t->p01cr & 0x04) != 0) return;
	if ((t->p01cr & 0x02) == 0) return;
	port_out(t, 1, t->port_latch[1]);       // as MAME: the data is not latched
}

static uint8_t p2_r(tlcs90 *t)
{
	if ((t->p01cr & 0x04) != 0) return 0;
	return port_in(t, 2);
}

static void p2_w(tlcs90 *t, uint8_t data)
{
	if ((t->p01cr & 0x04) != 0) return;
	t->port_latch[2] = data;
	uint8_t out_mask = t->p2cr;
	if (out_mask) port_out(t, 2, t->port_latch[2] & out_mask);
}

static uint8_t p3_r(tlcs90 *t)
{
	// 7,4,1,0
	return (t->port_latch[3] & 0x6c) | (port_in(t, 3) & 0x93);
}

static void p3_w(tlcs90 *t, uint8_t data)
{
	t->port_latch[3] = data & 0x6c;
	port_out(t, 3, t->port_latch[3]);
}

static uint8_t p4_r(tlcs90 *t)
{
	// only output
	return t->port_latch[4] & 0x0f;
}

static void p4_w(tlcs90 *t, uint8_t data)
{
	t->port_latch[4] = data & 0x0f;
	uint8_t out_mask = ~t->p4cr & 0x0f;
	if (out_mask) port_out(t, 4, t->port_latch[4] & out_mask);
}

static uint8_t p5_r(tlcs90 *t) { return port_in(t, 5) & 0x3f; }

static uint8_t p6_r(tlcs90 *t) { return (t->port_latch[6] & 0xf0) | (port_in(t, 6) & 0x0f); }

static void p6_w(tlcs90 *t, uint8_t data)
{
	// MAME masks 'data' by SMMOD (TO1 / M0 not added) but outputs the latch
	t->port_latch[6] = data & 0x0f;
	uint8_t out_mask = t->p67cr & 0x0f;
	if (out_mask) port_out(t, 6, t->port_latch[6] & out_mask);
}

static uint8_t p7_r(tlcs90 *t) { return (t->port_latch[7] & 0xf0) | (port_in(t, 7) & 0x0f); }

static void p7_w(tlcs90 *t, uint8_t data)
{
	t->port_latch[7] = data & 0x0f;
	uint8_t out_mask = t->p67cr >> 4;
	if (out_mask) port_out(t, 7, t->port_latch[7] & out_mask);
}

static uint8_t p8_r(tlcs90 *t)
{
	// 2,1,0
	return (t->port_latch[8] & 0x08) | (port_in(t, 8) & 0x07);
}

static void p8_w(tlcs90 *t, uint8_t data)
{
	t->port_latch[8] = data & 0x0f;
	uint8_t out_mask = ~t->p8cr & 0x08;
	if (out_mask) port_out(t, 8, t->port_latch[8] & out_mask);
}

static void t90_start_timer(tlcs90 *t, int i)
{
	int prescaler;

	t->timer_value[i] = 0;

	switch ((t->tmod >> (i * 2)) & 0x03)
	{
		case 0: break;  // 8-bit mode
		case 1: break;  // 16-bit mode
		case 2: return; // unsupported PPG mode
		case 3: return; // unsupported PWM mode
	}

	switch ((t->tclk >> (i * 2)) & 0x03)
	{
		case 0: return; // clocked by the previous timer's match signal (odd timers) / unsupported
		case 2: prescaler =  16;    break;
		case 3: prescaler = 256;    break;
		default:
		case 1: prescaler =   1;    break;
	}

	t->timer_period[i] = 8 * prescaler;
	t->timer_left[i] = t->timer_period[i];
	t->timer_run |= 1 << i;
}

static void t90_start_timer4(tlcs90 *t)
{
	int prescaler;

	t->timer4_value = 0;

	switch (t->t4mod & 0x03)
	{
		case 1:     prescaler =   1;    break;
		case 2:     prescaler =  16;    break;
		default:    return;             // unsupported T4MOD
	}

	t->timer_period[4] = 8 * prescaler;
	t->timer_left[4] = t->timer_period[4];
	t->timer_run |= 1 << 4;
}

static void t90_stop_timer(tlcs90 *t, int i) { t->timer_run &= ~(1 << i); }
static void t90_stop_timer4(tlcs90 *t) { t90_stop_timer(t, 4); }

static void t90_timer_callback(tlcs90 *t, int i)
{
	int mode, timer_fired;

	int mask = 0x20 | (1 << i);
	if ( (t->trun & mask) != mask )
		return;

	timer_fired = 0;

	mode = (t->tmod >> ((i & ~1) + 2)) & 0x03;
	// Match
	switch (mode)
	{
	case 0x02: // 8bit PPG
	case 0x03: // 8bit PWM
		// unhandled mode, falls through as in MAME
	case 0x00: // 8bit
		t->timer_value[i]++;
		if ( t->timer_value[i] == t->treg_8bit[i] )
			timer_fired = 1;
		break;

	case 0x01: // 16bit
		if (i & 1)
			break;
		t->timer_value[i]++;
		if (t->timer_value[i] == 0) t->timer_value[i+1]++;
		if (t->timer_value[i+1] == t->treg_8bit[i+1])
			if (t->timer_value[i] == t->treg_8bit[i])
				timer_fired = 1;
		break;
	}

	if (timer_fired) {
		// special stuff handling
		switch (mode) {
			case 0x02: // 8bit PPG
			case 0x03: // 8bit PWM
			case 0x00: // 8bit
				if (i & 1)
					break;
				if ( (t->tclk & (0x0C << (i * 2))) == 0 ) // T0/T1 match signal clocks T1/T3
					t90_timer_callback(t, i+1);
				break;
			case 0x01: // 16bit, only can happen for i=0,2
				t->timer_value[i+1] = 0;
				raise_irq(t, TLCS90_INTT0 + i+1);
				break;
		}
		// regular handling
		t->timer_value[i] = 0;
		raise_irq(t, TLCS90_INTT0 + i);
	}
}

static void t90_timer4_callback(tlcs90 *t)
{
	t->timer4_value++;

	// Match
	if ( t->timer4_value == t->treg_16bit[0] )
		raise_irq(t, TLCS90_INTT4);
	if ( t->timer4_value == t->treg_16bit[1] )
	{
		raise_irq(t, TLCS90_INTT5);
		if (t->t4mod & 0x04)
			t->timer4_value = 0;
	}
}

// advance the running timers by 'clocks'
static void timers_step(tlcs90 *t, int clocks)
{
	for (int i = 0; i < 5; i++)
	{
		if (!(t->timer_run & (1 << i))) continue;
		t->timer_left[i] -= clocks;
		while (t->timer_left[i] <= 0 && (t->timer_run & (1 << i)))
		{
			t->timer_left[i] += t->timer_period[i];
			if (i == 4) t90_timer4_callback(t);
			else        t90_timer_callback(t, i);
		}
	}
}

static void trun_w(tlcs90 *t, uint8_t data)
{
	uint8_t old = t->trun;

	// Timers 0-3
	for (int i = 0; i < 4; i++)
	{
		uint8_t mask = 0x20 | (1 << i);
		if ( (old ^ data) & mask ) // if timer bit or prescaler bit changed
		{
			if ( (data & mask) == mask )    t90_start_timer(t, i);
			else                            t90_stop_timer(t, i);
		}
	}

	// Timer 4
	uint8_t mask = 0x20 | 0x10;
	if ( (old ^ data) & mask )
	{
		if ( (data & mask) == mask )    t90_start_timer4(t);
		else                            t90_stop_timer4(t);
	}

	t->trun = data;
}

static void intel_w(tlcs90 *t, uint8_t data)
{
	t->irq_mask &= ~( (1 << TLCS90_INTT2) | (1 << TLCS90_INTT3) | (1 << TLCS90_INTT4) | (1 << TLCS90_INT1) |
	                  (1 << TLCS90_INTT5) | (1 << TLCS90_INT2) | (1 << TLCS90_INTRX) | (1 << TLCS90_INTTX) );

	t->irq_mask |= ((data & 0x80) ? (1 << TLCS90_INTT2) : 0) |
	               ((data & 0x40) ? (1 << TLCS90_INTT3) : 0) |
	               ((data & 0x20) ? (1 << TLCS90_INTT4) : 0) |
	               ((data & 0x10) ? (1 << TLCS90_INT1 ) : 0) |
	               ((data & 0x08) ? (1 << TLCS90_INTT5) : 0) |
	               ((data & 0x04) ? (1 << TLCS90_INT2 ) : 0) |
	               ((data & 0x02) ? (1 << TLCS90_INTRX) : 0) |
	               ((data & 0x01) ? (1 << TLCS90_INTTX) : 0);
}

static void inteh_w(tlcs90 *t, uint8_t data)
{
	t->irq_mask &= ~( (1 << TLCS90_INT0) | (1 << TLCS90_INTT0) | (1 << TLCS90_INTT1) );

	t->irq_mask |= ((data & 0x04) ? (1 << TLCS90_INT0 ) : 0) |
	               ((data & 0x02) ? (1 << TLCS90_INTT0) : 0) |
	               ((data & 0x01) ? (1 << TLCS90_INTT1) : 0);
}

static void irf_clear_w(tlcs90 *t, uint8_t data)
{
	if (data >= TLCS90_INTSWI + 2 && data < TLCS90_INTMAX + 2)
		clear_irq(t, data - 2);
}

static void treg_16bit_w(tlcs90 *t, int offset, uint8_t data)
{
	if (offset & 1)
		t->treg_16bit[offset >> 1] = (t->treg_16bit[offset >> 1] & 0x00ff) | (data << 8);
	else
		t->treg_16bit[offset >> 1] = (t->treg_16bit[offset >> 1] & 0xff00) | data;
}

// TMP90840 family layout (0xffc0 - 0xffef); unlisted addresses read 0 / ignore writes (reserved_r / reserved_w)
static uint8_t sfr840_r(tlcs90 *t, uint32_t a)
{
	switch (a)
	{
		case 0xffc1: return p1_r(t);
		case 0xffc4: return p2_r(t);
		case 0xffc6: return p3_r(t);
		case 0xffc8: return p4_r(t);
		case 0xffca: return p5_r(t);
		case 0xffcb: return 0x88 | t->smmod;
		case 0xffcc: return p6_r(t);
		case 0xffcd: return p7_r(t);
		case 0xffd0: return p8_r(t);
		case 0xffd8: return t->tclk;
		case 0xffda: return t->tmod;
		case 0xffdb: return t->trun;
		case 0xffe4: return t->t4mod;
		case 0xffec: return 0xf0 | (t->ixbase >> 16);
		case 0xffed: return 0xf0 | (t->iybase >> 16);
	}
	return 0;
}

static void sfr840_w(tlcs90 *t, uint32_t a, uint8_t v)
{
	switch (a)
	{
		case 0xffc1: p1_w(t, v); break;
		case 0xffc2: t->p01cr = v; break;
		case 0xffc3: irf_clear_w(t, v); break;
		case 0xffc4: p2_w(t, v); break;
		case 0xffc5: t->p2cr = v; break;
		case 0xffc6: p3_w(t, v); break;
		case 0xffc8: p4_w(t, v); break;
		case 0xffc9: t->p4cr = v; break;
		case 0xffcb: t->smmod = v; break;
		case 0xffcc: p6_w(t, v); break;
		case 0xffcd: p7_w(t, v); break;
		case 0xffce: t->p67cr = v; break;
		case 0xffd0: p8_w(t, v); break;
		case 0xffd1: t->p8cr = v; break;
		case 0xffd4: case 0xffd5: case 0xffd6: case 0xffd7: t->treg_8bit[a - 0xffd4] = v; break;
		case 0xffd8: t->tclk = v; break;
		case 0xffda: t->tmod = v; break;
		case 0xffdb: trun_w(t, v); break;
		case 0xffe0: case 0xffe1: case 0xffe2: case 0xffe3: treg_16bit_w(t, a - 0xffe0, v); break;
		case 0xffe4: t->t4mod = v; break;
		case 0xffe6: intel_w(t, v); break;
		case 0xffe7: inteh_w(t, v); break;
		case 0xffec: t->ixbase = (v & 0xf) << 16; break;
		case 0xffed: t->iybase = (v & 0xf) << 16; break;
	}
}

// TMP90844 layout (0xffc0 - 0xfff7, TMP90PH44)
static uint8_t sfr844_r(tlcs90 *t, uint32_t a)
{
	switch (a)
	{
		case 0xffc4: return p2_r(t);
		case 0xffc6: return p3_r(t);
		case 0xffc8: return p4_r(t);
		case 0xffca: return p5_r(t);
		case 0xffcb: return p6_r(t);
		case 0xffcc: return p7_r(t);
		case 0xffd8: return (t->tmod & 0x0f) << 4 | (t->tclk & 0x0f);   // t01mod_r
		case 0xffd9: return (t->tmod & 0xf0) | (t->tclk & 0xf0) >> 4;   // t23mod_r
		case 0xffdc: return t->trun;
		case 0xffe4: return t->t4mod;
	}
	return 0;
}

static void sfr844_w(tlcs90 *t, uint32_t a, uint8_t v)
{
	switch (a)
	{
		case 0xffc4: p2_w(t, v); break;
		case 0xffc5: t->p2cr = v; break;
		case 0xffc6: p3_w(t, v); break;
		case 0xffc8: p4_w(t, v); break;
		case 0xffcb: p6_w(t, v); break;
		case 0xffcc: p7_w(t, v); break;
		case 0xffcd: t->p67cr = v; break;
		case 0xffd4: case 0xffd5: case 0xffd6: case 0xffd7: t->treg_8bit[a - 0xffd4] = v; break;
		case 0xffd8:    // t01mod_w
			t->tmod = (t->tmod & 0xf0) | (v & 0xf0) >> 4;
			t->tclk = (t->tclk & 0xf0) | (v & 0x0f);
			break;
		case 0xffd9:    // t23mod_w
			t->tmod = (v & 0xf0) | (t->tmod & 0x0f);
			t->tclk = (v & 0x0f) << 4 | (t->tclk & 0x0f);
			break;
		case 0xffdc: trun_w(t, v); break;
		case 0xffe0: case 0xffe1: case 0xffe2: case 0xffe3: treg_16bit_w(t, a - 0xffe0, v); break;
		case 0xffe4: t->t4mod = v; break;
		case 0xffea: irf_clear_w(t, v); break;
		case 0xfff4: intel_w(t, v); break;
		case 0xfff5: inteh_w(t, v); break;
	}
}

/*************************************************************************************************************
    Memory
*************************************************************************************************************/

static uint8_t RM8(tlcs90 *t, uint32_t a)
{
	if (a < t->rom_size && t->rom) return t->rom[a];
	if (a >= t->ram_start && a <= 0xffbf) return t->iram[a - t->ram_start];
	if (a >= 0xffc0 && a <= t->sfr_end) return t->regs844 ? sfr844_r(t, a) : sfr840_r(t, a);
	return t->read8 ? t->read8(t->userdata, a) : 0;
}

static void WM8(tlcs90 *t, uint32_t a, uint8_t v)
{
	if (a < t->rom_size && t->rom) return;
	if (a >= t->ram_start && a <= 0xffbf) { t->iram[a - t->ram_start] = v; return; }
	if (a >= 0xffc0 && a <= t->sfr_end) { if (t->regs844) sfr844_w(t, a, v); else sfr840_w(t, a, v); return; }
	if (t->write8) t->write8(t->userdata, a, v);
}

static uint16_t RM16(tlcs90 *t, uint32_t a)    { return RM8(t, a) | (RM8(t, (a+1) & 0xffff) << 8); }
static void WM16(tlcs90 *t, uint32_t a, uint16_t v) { WM8(t, a, (uint8_t)v); WM8(t, (a+1) & 0xffff, v >> 8); }

static uint8_t  RX8 (tlcs90 *t, uint32_t a, uint32_t base) { return RM8(t, base | a); }
static uint16_t RX16(tlcs90 *t, uint32_t a, uint32_t base) { return RX8(t, a, base) | (RX8(t, (a+1) & 0xffff, base) << 8); }

static void WX8 (tlcs90 *t, uint32_t a, uint8_t  v, uint32_t base) { WM8(t, base | a, v); }
static void WX16(tlcs90 *t, uint32_t a, uint16_t v, uint32_t base) { WX8(t, a, (uint8_t)v, base); WX8(t, (a+1) & 0xffff, v >> 8, base); }

static uint8_t rd_op8(tlcs90 *t) { uint8_t b0 = RM8(t, t->addr++); t->addr &= 0xffff; return b0; }
static uint16_t rd_op16(tlcs90 *t) { uint8_t b0 = rd_op8(t); return b0 | (rd_op8(t) << 8); }

#define READ8()  rd_op8(t)
#define READ16() rd_op16(t)

static void decode(tlcs90 *t)
{
	uint8_t  b0, b1, b2, b3;
	uint16_t imm16;

	b0 = READ8();

	switch ( b0 )
	{
		case 0x00:
			OP( NOP,2 )         NONE( 1 )                   NONE( 2 )                       return;     // NOP

		case 0x01:
			OP( HALT,4 )        NONE( 1 )                   NONE( 2 )                       return;     // HALT
		case 0x02:
			OP( DI,2 )          NONE( 1 )                   NONE( 2 )                       return;     // DI
		case 0x03:
			OP( EI,2 )          NONE( 1 )                   NONE( 2 )                       return;     // EI

		case 0x07:
			OPCC( INCX,6,10 )   MI16( 1, 0xFF00|READ8() )   NONE( 2 )                       return;     // INCX ($FF00+n)

		case 0x08:
			OP( EX,2 )          R16( 1, DE )                R16( 2, HL )                    return;     // EX DE,HL
		case 0x09:
			OP( EX,2 )          R16( 1, AF )                R16( 2, AF2 )                   return;     // EX AF,AF'
		case 0x0a:
			OP( EXX,2 )         NONE( 1 )                   NONE( 2 )                       return;     // EXX

		case 0x0b:
			OP( DAA,4 )         R8( 1, A )                  NONE( 2 )                       return;     // DAA A

		case 0x0c:
			OP( RCF,2 )         NONE( 1 )                   NONE( 2 )                       return;     // RCF
		case 0x0d:
			OP( SCF,2 )         NONE( 1 )                   NONE( 2 )                       return;     // SCF
		case 0x0e:
			OP( CCF,2 )         NONE( 1 )                   NONE( 2 )                       return;     // CCF

		case 0x0f:
			OPCC( DECX,6,10 )   MI16( 1, 0xFF00|READ8() )   NONE( 2 )                       return;     // DECX ($FF00+n)

		case 0x10:
			OP( CPL,2 )         R8( 1, A )                  NONE( 2 )                       return;     // CPL A
		case 0x11:
			OP( NEG,2 )         R8( 1, A )                  NONE( 2 )                       return;     // NEG A

		case 0x12:                                                                                      // MUL HL,n
		case 0x13:                                                                                      // DIV HL,n
			OP( MUL+b0-0x12,16) R16( 1, HL )                I8( 2, READ8() )                return;

		case 0x14:  case 0x15:  case 0x16:
			OP16( ADD,6 )       R16( 1, IX+b0-0x14 )        I16( 2, READ16() )              return;     // ADD ix,mn

		case 0x17:
			OP( LDAR,8 )        R16( 1, HL )                D16( 2, READ16() )              return;     // LDAR HL,+cd

		case 0x18:
			OP( DJNZ,10 )       D8( 1, READ8() )            NONE( 2 )                       return;     // DJNZ +d
		case 0x19:
			OP16( DJNZ,10 )     R16( 1, BC )                D8( 2, READ8() )                return;     // DJNZ BC,+d

		case 0x1a:
			OPCC( JP,8,8 )      CC( 1, T )                  I16( 2, READ16() )              return;     // JP T,mn
		case 0x1b:
			OPCC16( JR,10,10 )  CC( 1, T )                  D16( 2, READ16() )              return;     // JR T,+cd

		case 0x1c:
			OPCC( CALL,14,14 )  CC( 1, T )                  I16( 2, READ16() )              return;     // CALL T,mn
		case 0x1d:
			OP( CALLR,16 )      D16( 1, READ16() )          NONE( 2 )                       return;     // CALLR +cd

		case 0x1e:
			OPCC( RET,10,10 )   CC( 1, T )                  NONE( 2 )                       return;     // RET T
		case 0x1f:
			OP( RETI,14 )       NONE( 1 )                   NONE( 2 )                       return;     // RETI

		case 0x20:  case 0x21:  case 0x22:  case 0x23:  case 0x24:  case 0x25:  case 0x26:
			OP( LD,2 )          R8( 1, A )                  R8( 2, b0 - 0x20 )              return;     // LD A,r

		case 0x27:
			OP( LD,8 )          R8( 1, A )                  MI16( 2, 0xFF00|READ8() )       return;     // LD A,($FF00+n)

		case 0x28:  case 0x29:  case 0x2a:  case 0x2b:  case 0x2c:  case 0x2d:  case 0x2e:
			OP( LD,2 )          R8( 1, b0 - 0x28 )          R8( 2, A )                      return;     // LD r,A

		case 0x2f:
			OP( LD,8 )          MI16( 1, 0xFF00|READ8() )   R8( 2, A )                      return;     // LD ($FF00+n), A

		case 0x30:  case 0x31:  case 0x32:  case 0x33:  case 0x34:  case 0x35:  case 0x36:
			OP( LD,4 )          R8( 1, b0 - 0x30 )          I8( 2, READ8() )                return;     // LD r,n

		case 0x37:
			OP( LD,10 )         MI16( 1, 0xFF00|READ8() )   I8( 2, READ8() )                return;     // LD ($FF00+w),n

		case 0x38:  case 0x39:  case 0x3a:  /*case 0x3b:*/  case 0x3c:  case 0x3d:  case 0x3e:
			OP16( LD,6 )        R16( 1, b0 - 0x38 )         I16( 2, READ16() )              return;     // LD rr,nn

		case 0x3f:
			OP( LDW,14 )        MI16( 1, 0xFF00|READ8() )   I16( 2, READ16() )              return;     // LDW ($FF00+w),mn

		case 0x40:  case 0x41:  case 0x42:  /*case 0x43:*/  case 0x44:  case 0x45:  case 0x46:
			OP16( LD,4 )        R16( 1, HL )                R16( 2, b0 - 0x40 )             return;     // LD HL,rr

		case 0x47:
			OP16( LD,10 )       R16( 1, HL )                MI16( 2, 0xFF00|READ8() )       return;     // LD HL,($FF00+n)

		case 0x48:  case 0x49:  case 0x4a:  /*case 0x4b:*/  case 0x4c:  case 0x4d:  case 0x4e:
			OP16( LD,4 )        R16( 1, b0 - 0x48 )         R16( 2, HL )                    return;     // LD rr,HL

		case 0x4f:
			OP16( LD,10 )       MI16( 1, 0xFF00|READ8() )   R16( 2, HL )                    return;     // LD ($FF00+n), HL

		case 0x50:  case 0x51:  case 0x52:  /*case 0x53:*/  case 0x54:  case 0x55:  case 0x56:
			OP( PUSH,8 )        Q16( 1, b0 - 0x50 )         NONE( 2 )                       return;     // PUSH qq
		case 0x58:  case 0x59:  case 0x5a:  /*case 0x5b:*/  case 0x5c:  case 0x5d:  case 0x5e:
			OP( POP,10 )        Q16( 1, b0 - 0x58 )         NONE( 2 )                       return;     // POP qq

		case 0x60:                                                                                      // ADD A,($FF00+n)
		case 0x61:                                                                                      // ADC A,($FF00+n)
		case 0x62:                                                                                      // SUB A,($FF00+n)
		case 0x63:                                                                                      // SBC A,($FF00+n)
		case 0x64:                                                                                      // AND A,($FF00+n)
		case 0x65:                                                                                      // XOR A,($FF00+n)
		case 0x66:                                                                                      // OR  A,($FF00+n)
		case 0x67:                                                                                      // CP  A,($FF00+n)
			OP( ADD+b0-0x60,8 ) R8( 1, A )                  MI16( 2, 0xFF00|READ8() )       return;

		case 0x68:                                                                                      // ADD A,n
		case 0x69:                                                                                      // ADC A,n
		case 0x6a:                                                                                      // SUB A,n
		case 0x6b:                                                                                      // SBC A,n
		case 0x6c:                                                                                      // AND A,n
		case 0x6d:                                                                                      // XOR A,n
		case 0x6e:                                                                                      // OR  A,n
		case 0x6f:                                                                                      // CP  A,n
			OP( ADD+b0-0x68,4 ) R8( 1, A )                  I8( 2, READ8() )                return;

		case 0x70:                                                                                      // ADD HL,($FF00+n)
		case 0x71:                                                                                      // ADC HL,($FF00+n)
		case 0x72:                                                                                      // SUB HL,($FF00+n)
		case 0x73:                                                                                      // SBC HL,($FF00+n)
		case 0x74:                                                                                      // AND HL,($FF00+n)
		case 0x75:                                                                                      // XOR HL,($FF00+n)
		case 0x76:                                                                                      // OR  HL,($FF00+n)
		case 0x77:                                                                                      // CP  HL,($FF00+n)
			OP16( ADD+b0-0x70,10 )  R16( 1, HL )            MI16( 2, 0xFF00|READ8() )       return;

		case 0x78:                                                                                      // ADD HL,mn
		case 0x79:                                                                                      // ADC HL,mn
		case 0x7a:                                                                                      // SUB HL,mn
		case 0x7b:                                                                                      // SBC HL,mn
		case 0x7c:                                                                                      // AND HL,mn
		case 0x7d:                                                                                      // XOR HL,mn
		case 0x7e:                                                                                      // OR  HL,mn
		case 0x7f:                                                                                      // CP  HL,mn
			OP16( ADD+b0-0x78,6 )   R16( 1, HL )            I16( 2, READ16() )              return;

		case 0x80:  case 0x81:  case 0x82:  case 0x83:  case 0x84:  case 0x85:  case 0x86:
			OP( INC,2 )         R8( 1, b0 - 0x80 )          NONE( 2 )                       return;     // INC r
		case 0x87:
			OP( INC,10 )        MI16( 1, 0xFF00|READ8() )   NONE( 2 )                       return;     // INC ($FF00+n)

		case 0x88:  case 0x89:  case 0x8a:  case 0x8b:  case 0x8c:  case 0x8d:  case 0x8e:
			OP( DEC,2 )         R8( 1, b0 - 0x88 )          NONE( 2 )                       return;     // DEC r
		case 0x8f:
			OP( DEC,10 )        MI16( 1, 0xFF00|READ8() )   NONE( 2 )                       return;     // DEC ($FF00+n)

		case 0x90:  case 0x91:  case 0x92:  /*case 0x93:*/  case 0x94:  case 0x95:  case 0x96:
			OP16( INC,4 )       R16( 1, b0 - 0x90 )         NONE( 2 )                       return;     // INC rr
		case 0x97:
			OP( INCW,14 )       MI16( 1, 0xFF00|READ8() )   NONE( 2 )                       return;     // INCW ($FF00+n)
		case 0x98:  case 0x99:  case 0x9a:  /*case 0x9b:*/  case 0x9c:  case 0x9d:  case 0x9e:
			OP16( DEC,4 )       R16( 1, b0 - 0x98 )         NONE( 2 )                       return;     // DEC rr
		case 0x9f:
			OP( DECW,14 )       MI16( 1, 0xFF00|READ8() )   NONE( 2 )                       return;     // DECW ($FF00+n)

		case 0xa0:                                                                                      // RLC A
		case 0xa1:                                                                                      // RRC A
		case 0xa2:                                                                                      // RL  A
		case 0xa3:                                                                                      // RR  A
		case 0xa4:                                                                                      // SLA A
		case 0xa5:                                                                                      // SRA A
		case 0xa6:                                                                                      // SLL A
		case 0xa7:                                                                                      // SRL A
			OP( RLC+b0-0xa0,2 ) R8( 1, A )                  NONE( 2 )                       return;

		case 0xa8:  case 0xa9:  case 0xaa:  case 0xab:  case 0xac:  case 0xad:  case 0xae:  case 0xaf:
			OP( BIT,8 )         BIT8( 1, b0 - 0xa8 )        MI16( 2, 0xFF00|READ8() )       return;     // BIT b,($FF00+n)
		case 0xb0:  case 0xb1:  case 0xb2:  case 0xb3:  case 0xb4:  case 0xb5:  case 0xb6:  case 0xb7:
			OP( RES,12 )        BIT8( 1, b0 - 0xb0 )        MI16( 2, 0xFF00|READ8() )       return;     // RES b,($FF00+n)
		case 0xb8:  case 0xb9:  case 0xba:  case 0xbb:  case 0xbc:  case 0xbd:  case 0xbe:  case 0xbf:
			OP( SET,12 )        BIT8( 1, b0 - 0xb8 )        MI16( 2, 0xFF00|READ8() )       return;     // SET b,($FF00+n)

		case 0xc0:  case 0xc1:  case 0xc2:  case 0xc3:  case 0xc4:  case 0xc5:  case 0xc6:  case 0xc7:
		case 0xc8:  case 0xc9:  case 0xca:  case 0xcb:  case 0xcc:  case 0xcd:  case 0xce:  case 0xcf:
			OPCC( JR,4,8 )      CC( 1, b0 - 0xc0 )          D8( 2, READ8() )                return;     // JR cc,+d

		case 0xe0:  case 0xe1:  case 0xe2:  /*case 0xe3:*/  case 0xe4:  case 0xe5:  case 0xe6:
			b1 = READ8();
			switch ( b1 )   {
				case 0x10:                                                                              // RLD (gg)
				case 0x11:                                                                              // RRD (gg)
					OP( RLD+b1-0x10,12 )    MR16( 1, b0 - 0xe0 )    NONE( 2 )               return;

				case 0x12:                                                                              // MUL HL,(gg)
				case 0x13:                                                                              // DIV HL,(gg)
					OP( MUL+b1-0x12,18 )    R16( 1, HL )            MR16( 2, b0 - 0xe0 )    return;

				case 0x14:  case 0x15:  case 0x16:
					OP16( ADD,8 )           R16( 1, IX+b1-0x14 )    MR16( 2, b0 - 0xe0 )    return;     // ADD ix,(gg)

				case 0x28:  case 0x29:  case 0x2a:  case 0x2b:  case 0x2c:  case 0x2d:  case 0x2e:
					OP( LD,6 )              R8( 1, b1 - 0x28 )      MR16( 2, b0 - 0xe0 )    return;     // LD r,(gg)
				case 0x48:  case 0x49:  case 0x4a:  /*case 0x4b:*/  case 0x4c:  case 0x4d:  case 0x4e:
					OP16( LD,8 )            R16( 1, b1 - 0x48 )     MR16( 2, b0 - 0xe0 )    return;     // LD rr,(gg)

				case 0x50:  case 0x51:  case 0x52:  /*case 0x53:*/  case 0x54:  case 0x55:  case 0x56:
					OP( EX,14 )             MR16( 1, b0 - 0xe0 )    R16( 2, b1 - 0x50 )     return;     // EX (gg),rr

				case 0x60:                                                                              // ADD A,(gg)
				case 0x61:                                                                              // ADC A,(gg)
				case 0x62:                                                                              // SUB A,(gg)
				case 0x63:                                                                              // SBC A,(gg)
				case 0x64:                                                                              // AND A,(gg)
				case 0x65:                                                                              // XOR A,(gg)
				case 0x66:                                                                              // OR  A,(gg)
				case 0x67:                                                                              // CP  A,(gg)
					OP( ADD+b1-0x60,6 )     R8( 1, A )              MR16( 2, b0 - 0xe0 )    return;

				case 0x70:                                                                              // ADD HL,(gg)
				case 0x71:                                                                              // ADC HL,(gg)
				case 0x72:                                                                              // SUB HL,(gg)
				case 0x73:                                                                              // SBC HL,(gg)
				case 0x74:                                                                              // AND HL,(gg)
				case 0x75:                                                                              // XOR HL,(gg)
				case 0x76:                                                                              // OR  HL,(gg)
				case 0x77:                                                                              // CP  HL,(gg)
					OP16( ADD+b1-0x70,8 )   R16( 1, HL )            MR16( 2, b0 - 0xe0 )    return;

				case 0x87:
					OP( INC,8 )             MR16( 1, b0 - 0xe0 )    NONE( 2 )               return;     // INC (gg)
				case 0x8f:
					OP( DEC,8 )             MR16( 1, b0 - 0xe0 )    NONE( 2 )               return;     // DEC (gg)

				case 0x97:
					OP( INCW,12 )           MR16( 1, b0 - 0xe0 )    NONE( 2 )               return;     // INCW (gg)
				case 0x9f:
					OP( DECW,12 )           MR16( 1, b0 - 0xe0 )    NONE( 2 )               return;     // DECW (gg)

				case 0xa0:                                                                              // RLC (gg)
				case 0xa1:                                                                              // RRC (gg)
				case 0xa2:                                                                              // RL  (gg)
				case 0xa3:                                                                              // RR  (gg)
				case 0xa4:                                                                              // SLA (gg)
				case 0xa5:                                                                              // SRA (gg)
				case 0xa6:                                                                              // SLL (gg)
				case 0xa7:                                                                              // SRL (gg)
					OP( RLC+b1-0xa0,8 )     MR16( 1, b0 - 0xe0 )    NONE( 2 )               return;

				case 0x18:  case 0x19:  case 0x1a:  case 0x1b:  case 0x1c:  case 0x1d:  case 0x1e:  case 0x1f:
					OP( TSET,12 )           BIT8( 1, b1 - 0x18 )    MR16( 2, b0 - 0xe0 )    return;     // TSET b,(gg)
				case 0xa8:  case 0xa9:  case 0xaa:  case 0xab:  case 0xac:  case 0xad:  case 0xae:  case 0xaf:
					OP( BIT,6 )             BIT8( 1, b1 - 0xa8 )    MR16( 2, b0 - 0xe0 )    return;     // BIT b,(gg)
				case 0xb0:  case 0xb1:  case 0xb2:  case 0xb3:  case 0xb4:  case 0xb5:  case 0xb6:  case 0xb7:
					OP( RES,10 )            BIT8( 1, b1 - 0xb0 )    MR16( 2, b0 - 0xe0 )    return;     // RES b,(gg)
				case 0xb8:  case 0xb9:  case 0xba:  case 0xbb:  case 0xbc:  case 0xbd:  case 0xbe:  case 0xbf:
					OP( SET,10 )            BIT8( 1, b1 - 0xb8 )    MR16( 2, b0 - 0xe0 )    return;     // SET b,(gg)
			}   break;
		case 0xe3:
			imm16 = READ16();
			b3 = READ8();
			switch ( b3 )   {
				case 0x10:                                                                              // RLD (mn)
				case 0x11:                                                                              // RRD (mn)
					OP( RLD+b3-0x10,16 )    MI16( 1, imm16 )        NONE( 2 )               return;

				case 0x12:                                                                              // MUL HL,(mn)
				case 0x13:                                                                              // DIV HL,(mn)
					OP( MUL+b3-0x12,22 )    R16( 1, HL )            MI16( 2, imm16 )        return;

				case 0x14:  case 0x15:  case 0x16:
					OP16( ADD,12 )          R16( 1, IX+b3-0x14 )    MI16( 2, imm16 )        return;     // ADD ix,(mn)

				case 0x28:  case 0x29:  case 0x2a:  case 0x2b:  case 0x2c:  case 0x2d:  case 0x2e:
					OP( LD,10 )             R8( 1, b3 - 0x28 )      MI16( 2, imm16 )        return;     // LD r,(mn)
				case 0x48:  case 0x49:  case 0x4a:  /*case 0x4b:*/  case 0x4c:  case 0x4d:  case 0x4e:
					OP16( LD,12 )           R16( 1, b3 - 0x48 )     MI16( 2, imm16 )        return;     // LD rr,(mn)

				case 0x50:  case 0x51:  case 0x52:  /*case 0x53:*/  case 0x54:  case 0x55:  case 0x56:
					OP( EX,18 )             MI16( 1, imm16 )        R16( 2, b3 - 0x50 )     return;     // EX (mn),rr

				case 0x60:                                                                              // ADD A,(mn)
				case 0x61:                                                                              // ADC A,(mn)
				case 0x62:                                                                              // SUB A,(mn)
				case 0x63:                                                                              // SBC A,(mn)
				case 0x64:                                                                              // AND A,(mn)
				case 0x65:                                                                              // XOR A,(mn)
				case 0x66:                                                                              // OR  A,(mn)
				case 0x67:                                                                              // CP  A,(mn)
					OP( ADD+b3-0x60,10 )    R8( 1, A )              MI16( 2, imm16 )        return;

				case 0x70:                                                                              // ADD HL,(mn)
				case 0x71:                                                                              // ADC HL,(mn)
				case 0x72:                                                                              // SUB HL,(mn)
				case 0x73:                                                                              // SBC HL,(mn)
				case 0x74:                                                                              // AND HL,(mn)
				case 0x75:                                                                              // XOR HL,(mn)
				case 0x76:                                                                              // OR  HL,(mn)
				case 0x77:                                                                              // CP  HL,(mn)
					OP16( ADD+b3-0x70,12 )  R16( 1, HL )            MI16( 2, imm16 )        return;

				case 0x87:
					OP( INC,12 )            MI16( 1, imm16 )        NONE( 2 )               return;     // INC (mn)
				case 0x8f:
					OP( DEC,12 )            MI16( 1, imm16 )        NONE( 2 )               return;     // DEC (mn)

				case 0x97:
					OP( INCW,16 )           MI16( 1, imm16 )        NONE( 2 )               return;     // INCW (mn)
				case 0x9f:
					OP( DECW,16 )           MI16( 1, imm16 )        NONE( 2 )               return;     // DECW (mn)

				case 0xa0:                                                                              // RLC (mn)
				case 0xa1:                                                                              // RRC (mn)
				case 0xa2:                                                                              // RL  (mn)
				case 0xa3:                                                                              // RR  (mn)
				case 0xa4:                                                                              // SLA (mn)
				case 0xa5:                                                                              // SRA (mn)
				case 0xa6:                                                                              // SLL (mn)
				case 0xa7:                                                                              // SRL (mn)
					OP( RLC+b3-0xa0,12 )    MI16( 1, imm16 )        NONE( 2 )               return;

				case 0x18:  case 0x19:  case 0x1a:  case 0x1b:  case 0x1c:  case 0x1d:  case 0x1e:  case 0x1f:
					OP( TSET,16 )           BIT8( 1, b3 - 0x18 )    MI16( 2, imm16 )        return;     // TSET b,(mn)
				case 0xa8:  case 0xa9:  case 0xaa:  case 0xab:  case 0xac:  case 0xad:  case 0xae:  case 0xaf:
					OP( BIT,10 )            BIT8( 1, b3 - 0xa8 )    MI16( 2, imm16 )        return;     // BIT b,(mn)
				case 0xb0:  case 0xb1:  case 0xb2:  case 0xb3:  case 0xb4:  case 0xb5:  case 0xb6:  case 0xb7:
					OP( RES,14 )            BIT8( 1, b3 - 0xb0 )    MI16( 2, imm16 )        return;     // RES b,(mn)
				case 0xb8:  case 0xb9:  case 0xba:  case 0xbb:  case 0xbc:  case 0xbd:  case 0xbe:  case 0xbf:
					OP( SET,14 )            BIT8( 1, b3 - 0xb8 )    MI16( 2, imm16 )        return;     // SET b,(mn)
			}   break;

		case 0xe7:
			b1 = READ8();
			b2 = READ8();
			switch ( b2 )   {
				case 0x10:                                                                              // RLD ($FF00+n)
				case 0x11:                                                                              // RRD ($FF00+n)
					OP( RLD+b2-0x10,14 )    MI16( 1, 0xFF00|b1 )    NONE( 2 )               return;

				case 0x12:                                                                              // MUL HL,($FF00+n)
				case 0x13:                                                                              // DIV HL,($FF00+n)
					OP( MUL+b2-0x12,20 )    R16( 1, HL )            MI16( 2, 0xFF00|b1 )    return;

				case 0x14:  case 0x15:  case 0x16:
					OP16( ADD,10 )          R16( 1, IX+b2-0x14 )    MI16( 2, 0xFF00|b1 )    return;     // ADD ix,($FF00+n)

				case 0x18:  case 0x19:  case 0x1a:  case 0x1b:  case 0x1c:  case 0x1d:  case 0x1e:  case 0x1f:
					OP( TSET,14 )           BIT8( 1, b2 - 0x18 )    MI16( 2, 0xFF00|b1 )    return;     // TSET b,($FF00+n)
				case 0x28:  case 0x29:  case 0x2a:  case 0x2b:  case 0x2c:  case 0x2d:  case 0x2e:
					OP( LD,8 )              R8( 1, b2 - 0x28 )      MI16( 2, 0xFF00|b1 )    return;     // LD r,($FF00+n)
				case 0x48:  case 0x49:  case 0x4a:  /*case 0x4b:*/  case 0x4c:  case 0x4d:  case 0x4e:
					OP16( LD,10 )           R16( 1, b2 - 0x48 )     MI16( 2, 0xFF00|b1 )    return;     // LD rr,($FF00+n)

				case 0x50:  case 0x51:  case 0x52:  /*case 0x53:*/  case 0x54:  case 0x55:  case 0x56:
					OP( EX,16 )             MI16( 1, 0xFF00|b1 )    R16( 2, b2 - 0x50 )     return;     // EX ($FF00+n),rr

				case 0xa0:                                                                              // RLC ($FF00+n)
				case 0xa1:                                                                              // RRC ($FF00+n)
				case 0xa2:                                                                              // RL  ($FF00+n)
				case 0xa3:                                                                              // RR  ($FF00+n)
				case 0xa4:                                                                              // SLA ($FF00+n)
				case 0xa5:                                                                              // SRA ($FF00+n)
				case 0xa6:                                                                              // SLL ($FF00+n)
				case 0xa7:                                                                              // SRL ($FF00+n)
					OP( RLC+b2-0xa0,10 )    MI16( 1, 0xFF00|b1 )    NONE( 2 )               return;
			}   break;

		case 0xe8:  case 0xe9:  case 0xea:  /*case 0xeb:*/  case 0xec:  case 0xed:  case 0xee:
			b1 = READ8();
			switch ( b1 )   {
				case 0x20:  case 0x21:  case 0x22:  case 0x23:  case 0x24:  case 0x25:  case 0x26:
					OP( LD,6 )              MR16( 1, b0 - 0xe8 )    R8( 2, b1 - 0x20 )      return;     // LD (gg),r
				case 0x37:
					OP( LD,8 )              MR16( 1, b0 - 0xe8 )    I8( 2, READ8() )        return;     // LD (gg),n
				case 0x3f:
					OP( LDW,12 )            MR16( 1, b0 - 0xe8 )    I16( 2, READ16() )      return;     // LDW (gg),mn
				case 0x40:  case 0x41:  case 0x42:  /*case 0x43:*/  case 0x44:  case 0x45:  case 0x46:
					OP16( LD,8 )            MR16( 1, b0 - 0xe8 )    R16( 2, b1 - 0x40 )     return;     // LD (gg),rr

				case 0x68:                                                                              // ADD (gg),n
				case 0x69:                                                                              // ADC (gg),n
				case 0x6a:                                                                              // SUB (gg),n
				case 0x6b:                                                                              // SBC (gg),n
				case 0x6c:                                                                              // AND (gg),n
				case 0x6d:                                                                              // XOR (gg),n
				case 0x6e:                                                                              // OR  (gg),n
					OP( ADD+b1-0x68,10 )    MR16( 1, b0 - 0xe8 )    I8( 2, READ8() )        return;
				case 0x6f:                                                                              // CP  (gg),n
					OP( CP,8 )              MR16( 1, b0 - 0xe8 )    I8( 2, READ8() )        return;

				case 0xc0:  case 0xc1:  case 0xc2:  case 0xc3:  case 0xc4:  case 0xc5:  case 0xc6:  case 0xc7:
				case 0xc8:  case 0xc9:  case 0xca:  case 0xcb:  case 0xcc:  case 0xcd:  case 0xce:  case 0xcf:
					OPCC( JP,6,8 )          CC( 1, b1 - 0xc0 )      R16( 2, b0 - 0xe8 )     return;     // JP [cc,]gg
				case 0xd0:  case 0xd1:  case 0xd2:  case 0xd3:  case 0xd4:  case 0xd5:  case 0xd6:  case 0xd7:
				case 0xd8:  case 0xd9:  case 0xda:  case 0xdb:  case 0xdc:  case 0xdd:  case 0xde:  case 0xdf:
					OPCC( CALL,6,14 )       CC( 1, b1 - 0xd0 )      R16( 2, b0 - 0xe8 )     return;     // CALL [cc,]gg
			}   break;
		case 0xeb:
			imm16 = READ16();
			b3 = READ8();
			switch ( b3 )   {
				case 0x20:  case 0x21:  case 0x22:  case 0x23:  case 0x24:  case 0x25:  case 0x26:
					OP( LD,10 )             MI16( 1, imm16 )        R8( 2, b3 - 0x20 )      return;     // LD (mn),r
				case 0x37:
					OP( LD,12 )             MI16( 1, imm16 )        I8( 2, READ8() )        return;     // LD (vw),n
				case 0x3f:
					OP( LDW,16 )            MI16( 1, imm16 )        I16( 2, READ16() )      return;     // LDW (vw),mn
				case 0x40:  case 0x41:  case 0x42:  /*case 0x43:*/  case 0x44:  case 0x45:  case 0x46:
					OP16( LD,12 )           MI16( 1, imm16 )        R16( 2, b3 - 0x40 )     return;     // LD (mn),rr

				case 0x68:                                                                              // ADD (vw),n
				case 0x69:                                                                              // ADC (vw),n
				case 0x6a:                                                                              // SUB (vw),n
				case 0x6b:                                                                              // SBC (vw),n
				case 0x6c:                                                                              // AND (vw),n
				case 0x6d:                                                                              // XOR (vw),n
				case 0x6e:                                                                              // OR  (vw),n
					OP( ADD+b3-0x68,14 )    MI16( 1, imm16 )        I8( 2, READ8() )        return;
				case 0x6f:                                                                              // CP  (vw),n
					OP( ADD+b3-0x68,12 )    MI16( 1, imm16 )        I8( 2, READ8() )        return;

				case 0xc0:  case 0xc1:  case 0xc2:  case 0xc3:  case 0xc4:  case 0xc5:  case 0xc6:  case 0xc7:
				case 0xc8:  case 0xc9:  case 0xca:  case 0xcb:  case 0xcc:  case 0xcd:  case 0xce:  case 0xcf:
					OPCC( JP,10,12 )        CC( 1, b3 - 0xc0 )      I16( 2, imm16 )         return;     // JP cc,mn
				case 0xd0:  case 0xd1:  case 0xd2:  case 0xd3:  case 0xd4:  case 0xd5:  case 0xd6:  case 0xd7:
				case 0xd8:  case 0xd9:  case 0xda:  case 0xdb:  case 0xdc:  case 0xdd:  case 0xde:  case 0xdf:
					OPCC( CALL,10,18 )      CC( 1, b3 - 0xd0 )      I16( 2, imm16 )         return;     // CALL cc,mn
			}   break;

		case 0xef:
			b1 = READ8();
			b2 = READ8();
			switch ( b2 )   {
				case 0x20:  case 0x21:  case 0x22:  case 0x23:  case 0x24:  case 0x25:  case 0x26:
					OP( LD,8 )              MI16( 1, 0xFF00|b1 )    R8( 2, b2 - 0x20 )      return;     // LD ($FF00+n),r
				case 0x40:  case 0x41:  case 0x42:  /*case 0x43:*/  case 0x44:  case 0x45:  case 0x46:
					OP16( LD,10 )           MI16( 1, 0xFF00|b1 )    R16( 2, b2 - 0x40 )     return;     // LD ($FF00+n),rr

				case 0x68:                                                                              // ADD ($FF00+w),n
				case 0x69:                                                                              // ADC ($FF00+w),n
				case 0x6a:                                                                              // SUB ($FF00+w),n
				case 0x6b:                                                                              // SBC ($FF00+w),n
				case 0x6c:                                                                              // AND ($FF00+w),n
				case 0x6d:                                                                              // XOR ($FF00+w),n
				case 0x6e:                                                                              // OR  ($FF00+w),n
					OP( ADD+b2-0x68,12 )    MI16( 1, 0xFF00|b1 )    I8( 2, READ8() )        return;
				case 0x6f:                                                                              // CP  ($FF00+w),n
					OP( ADD+b2-0x68,10 )    MI16( 1, 0xFF00|b1 )    I8( 2, READ8() )        return;
			}   break;

		case 0xf0:  case 0xf1:  case 0xf2:
			b1 = READ8();
			b2 = READ8();
			switch ( b2 )   {
				case 0x10:                                                                              // RLD (ix+d)
				case 0x11:                                                                              // RRD (ix+d)
					OP( RLD+b2-0x10,16 )    MR16D8( 1, IX + b0 - 0xf0, b1 )     NONE( 2 )   return;

				case 0x12:                                                                              // MUL HL,(ix+d)
				case 0x13:                                                                              // DIV HL,(ix+d)
					OP( MUL+b2-0x12,22 )    R16( 1, HL )    MR16D8( 2, IX + b0 - 0xf0, b1 ) return;

				case 0x14:  case 0x15:  case 0x16:
					OP16( ADD,12 )  R16( 1, IX+b2-0x14 )    MR16D8( 2, IX + b0 - 0xf0, b1 ) return;     // ADD ix,(jx+d)

				case 0x28:  case 0x29:  case 0x2a:  case 0x2b:  case 0x2c:  case 0x2d:  case 0x2e:
					OP( LD,10 )     R8( 1, b2 - 0x28 )  MR16D8( 2, IX + b0 - 0xf0, b1 )     return;     // LD r,(ix+d)
				case 0x48:  case 0x49:  case 0x4a:  /*case 0x4b:*/  case 0x4c:  case 0x4d:  case 0x4e:
					OP16( LD,12 )   R16( 1, b2 - 0x48 ) MR16D8( 2, IX + b0 - 0xf0, b1 )     return;     // LD rr,(ix+d)

				case 0x50:  case 0x51:  case 0x52:  /*case 0x53:*/  case 0x54:  case 0x55:  case 0x56:
					OP( EX,18 )     MR16D8( 1, IX + b0 - 0xf0, b1 ) R16( 2, b2 - 0x50 )     return;     // EX (ix+d),rr

				case 0x60:                                                                              // ADD A,(ix+d)
				case 0x61:                                                                              // ADC A,(ix+d)
				case 0x62:                                                                              // SUB A,(ix+d)
				case 0x63:                                                                              // SBC A,(ix+d)
				case 0x64:                                                                              // AND A,(ix+d)
				case 0x65:                                                                              // XOR A,(ix+d)
				case 0x66:                                                                              // OR  A,(ix+d)
				case 0x67:                                                                              // CP  A,(ix+d)
					OP( ADD+b2-0x60,10 )    R8( 1, A )  MR16D8( 2, IX + b0 - 0xf0, b1 )     return;

				case 0x70:                                                                              // ADD HL,(ix+d)
				case 0x71:                                                                              // ADC HL,(ix+d)
				case 0x72:                                                                              // SUB HL,(ix+d)
				case 0x73:                                                                              // SBC HL,(ix+d)
				case 0x74:                                                                              // AND HL,(ix+d)
				case 0x75:                                                                              // XOR HL,(ix+d)
				case 0x76:                                                                              // OR  HL,(ix+d)
				case 0x77:                                                                              // CP  HL,(ix+d)
					OP16( ADD+b2-0x70,12 )  R16( 1, HL )    MR16D8( 2, IX + b0 - 0xf0, b1 ) return;

				case 0x87:
					OP( INC,12 )            MR16D8( 1, IX + b0 - 0xf0, b1 )     NONE( 2 )   return;     // INC (ix+d)
				case 0x8f:
					OP( DEC,12 )            MR16D8( 1, IX + b0 - 0xf0, b1 )     NONE( 2 )   return;     // DEC (ix+d)

				case 0x97:
					OP( INCW,16 )           MR16D8( 1, IX + b0 - 0xf0, b1 )     NONE( 2 )   return;     // INCW (ix+d)
				case 0x9f:
					OP( DECW,16 )           MR16D8( 1, IX + b0 - 0xf0, b1 )     NONE( 2 )   return;     // DECW (ix+d)

				case 0xa0:                                                                              // RLC (ix+d)
				case 0xa1:                                                                              // RRC (ix+d)
				case 0xa2:                                                                              // RL  (ix+d)
				case 0xa3:                                                                              // RR  (ix+d)
				case 0xa4:                                                                              // SLA (ix+d)
				case 0xa5:                                                                              // SRA (ix+d)
				case 0xa6:                                                                              // SLL (ix+d)
				case 0xa7:                                                                              // SRL (ix+d)
					OP( RLC+b2-0xa0,12 )    MR16D8( 1, IX + b0 - 0xf0, b1 )     NONE( 2 )   return;

				case 0x18:  case 0x19:  case 0x1a:  case 0x1b:  case 0x1c:  case 0x1d:  case 0x1e:  case 0x1f:
					OP( TSET,16 )   BIT8( 1, b2 - 0x18 )    MR16D8( 2, IX + b0 - 0xf0, b1 ) return;     // TSET b,(ix+d)
				case 0xa8:  case 0xa9:  case 0xaa:  case 0xab:  case 0xac:  case 0xad:  case 0xae:  case 0xaf:
					OP( BIT,10 )    BIT8( 1, b2 - 0xa8 )    MR16D8( 2, IX + b0 - 0xf0, b1 ) return;     // BIT b,(ix+d)
				case 0xb0:  case 0xb1:  case 0xb2:  case 0xb3:  case 0xb4:  case 0xb5:  case 0xb6:  case 0xb7:
					OP( RES,14 )    BIT8( 1, b2 - 0xb0 )    MR16D8( 2, IX + b0 - 0xf0, b1 ) return;     // RES b,(ix+d)
				case 0xb8:  case 0xb9:  case 0xba:  case 0xbb:  case 0xbc:  case 0xbd:  case 0xbe:  case 0xbf:
					OP( SET,14 )    BIT8( 1, b2 - 0xb8 )    MR16D8( 2, IX + b0 - 0xf0, b1 ) return;     // SET b,(ix+d)
			}   break;

		case 0xf3:
			b1 = READ8();
			switch ( b1 )   {
				case 0x10:                                                                              // RLD (HL+A)
				case 0x11:                                                                              // RRD (HL+A)
					OP( RLD+b1-0x10,20 )    MR16R8( 1, HL, A )          NONE( 2 )           return;

				case 0x12:                                                                              // MUL HL,(HL+A)
				case 0x13:                                                                              // DIV HL,(HL+A)
					OP( MUL+b1-0x12,26 )    R16( 1, HL )                MR16R8( 2, HL, A )  return;

				case 0x14:  case 0x15:  case 0x16:
					OP16( ADD,16 )          R16( 1, IX+b1-0x14 )        MR16R8( 2, HL, A )  return;     // ADD ix,(HL+A)

				case 0x28:  case 0x29:  case 0x2a:  case 0x2b:  case 0x2c:  case 0x2d:  case 0x2e:
					OP( LD,14 )             R8( 1, b1 - 0x28 )          MR16R8( 2, HL, A )  return;     // LD r,(HL+A)
				case 0x48:  case 0x49:  case 0x4a:  /*case 0x4b:*/  case 0x4c:  case 0x4d:  case 0x4e:
					OP16( LD,16 )           R16( 1, b1 - 0x48 )         MR16R8( 2, HL, A )  return;     // LD rr,(HL+A)

				case 0x50:  case 0x51:  case 0x52:  /*case 0x53:*/  case 0x54:  case 0x55:  case 0x56:
					OP( EX,22 )             MR16R8( 1, HL, A )          R16( 2, b1 - 0x50 ) return;     // EX (HL+A),rr

				case 0x60:                                                                              // ADD A,(HL+A)
				case 0x61:                                                                              // ADC A,(HL+A)
				case 0x62:                                                                              // SUB A,(HL+A)
				case 0x63:                                                                              // SBC A,(HL+A)
				case 0x64:                                                                              // AND A,(HL+A)
				case 0x65:                                                                              // XOR A,(HL+A)
				case 0x66:                                                                              // OR  A,(HL+A)
				case 0x67:                                                                              // CP  A,(HL+A)
					OP( ADD+b1-0x60,14 )    R8( 1, A )                  MR16R8( 2, HL, A )  return;

				case 0x70:                                                                              // ADD HL,(HL+A)
				case 0x71:                                                                              // ADC HL,(HL+A)
				case 0x72:                                                                              // SUB HL,(HL+A)
				case 0x73:                                                                              // SBC HL,(HL+A)
				case 0x74:                                                                              // AND HL,(HL+A)
				case 0x75:                                                                              // XOR HL,(HL+A)
				case 0x76:                                                                              // OR  HL,(HL+A)
				case 0x77:                                                                              // CP  HL,(HL+A)
					OP16( ADD+b1-0x70,16 )  R16( 1, HL )                MR16R8( 2, HL, A )  return;

				case 0x87:
					OP( INC,16 )            MR16R8( 1, HL, A )          NONE( 2 )           return;     // INC (HL+A)
				case 0x8f:
					OP( DEC,16 )            MR16R8( 1, HL, A )          NONE( 2 )           return;     // DEC (HL+A)

				case 0x97:
					OP( INCW,20 )           MR16R8( 1, HL, A )          NONE( 2 )           return;     // INCW (HL+A)
				case 0x9f:
					OP( DECW,20 )           MR16R8( 1, HL, A )          NONE( 2 )           return;     // DECW (HL+A)

				case 0xa0:                                                                              // RLC (HL+A)
				case 0xa1:                                                                              // RRC (HL+A)
				case 0xa2:                                                                              // RL  (HL+A)
				case 0xa3:                                                                              // RR  (HL+A)
				case 0xa4:                                                                              // SLA (HL+A)
				case 0xa5:                                                                              // SRA (HL+A)
				case 0xa6:                                                                              // SLL (HL+A)
				case 0xa7:                                                                              // SRL (HL+A)
					OP( RLC+b1-0xa0,16 )    MR16R8( 1, HL, A )          NONE( 2 )           return;

				case 0x18:  case 0x19:  case 0x1a:  case 0x1b:  case 0x1c:  case 0x1d:  case 0x1e:  case 0x1f:
					OP( TSET,20 )           BIT8( 1, b1 - 0x18 )        MR16R8( 2, HL, A )  return;     // TSET b,(HL+A)
				case 0xa8:  case 0xa9:  case 0xaa:  case 0xab:  case 0xac:  case 0xad:  case 0xae:  case 0xaf:
					OP( BIT,14 )            BIT8( 1, b1 - 0xa8 )        MR16R8( 2, HL, A )  return;     // BIT b,(HL+A)
				case 0xb0:  case 0xb1:  case 0xb2:  case 0xb3:  case 0xb4:  case 0xb5:  case 0xb6:  case 0xb7:
					OP( RES,18 )            BIT8( 1, b1 - 0xb0 )        MR16R8( 2, HL, A )  return;     // RES b,(HL+A)
				case 0xb8:  case 0xb9:  case 0xba:  case 0xbb:  case 0xbc:  case 0xbd:  case 0xbe:  case 0xbf:
					OP( SET,18 )            BIT8( 1, b1 - 0xb8 )        MR16R8( 2, HL, A )  return;     // SET b,(HL+A)
			}   break;

		case 0xf4:  case 0xf5:  case 0xf6:
			b1 = READ8();
			b2 = READ8();
			switch ( b2 )   {
				case 0x20:  case 0x21:  case 0x22:  case 0x23:  case 0x24:  case 0x25:  case 0x26:
					OP( LD,10 )     MR16D8( 1, IX + b0 - 0xf4, b1 ) R8( 2, b2 - 0x20 )      return;     // LD (ix+d),r
				case 0x37:
					OP( LD,12 )     MR16D8( 1, IX + b0 - 0xf4, b1 ) I8( 2, READ8() )        return;     // LD (ix+d),n
				case 0x38:  case 0x39:  case 0x3a:  /*case 0x3b:*/  case 0x3c:  case 0x3d:  case 0x3e:
					OP( LDA,10 )    R16( 1, b2 - 0x38 )     R16D8( 2, IX + b0 - 0xf4, b1 )  return;     // LDA rr,ix+d
				case 0x3f:
					OP( LDW,16 )    MR16D8( 1, IX + b0 - 0xf4, b1 ) I16( 2, READ16() )      return;     // LDW (ix+d),mn
				case 0x40:  case 0x41:  case 0x42:  /*case 0x43:*/  case 0x44:  case 0x45:  case 0x46:
					OP16( LD,12 )   MR16D8( 1, IX + b0 - 0xf4, b1 ) R16( 2, b2 - 0x40 )     return;     // LD (ix+d),rr

				case 0x68:                                                                              // ADD (ix+d),n
				case 0x69:                                                                              // ADC (ix+d),n
				case 0x6a:                                                                              // SUB (ix+d),n
				case 0x6b:                                                                              // SBC (ix+d),n
				case 0x6c:                                                                              // AND (ix+d),n
				case 0x6d:                                                                              // XOR (ix+d),n
				case 0x6e:                                                                              // OR  (ix+d),n
					OP( ADD+b2-0x68,14) MR16D8( 1, IX + b0 - 0xf4, b1 ) I8( 2, READ8() )    return;
				case 0x6f:                                                                              // CP  (ix+d),n
					OP( ADD+b2-0x68,12) MR16D8( 1, IX + b0 - 0xf4, b1 ) I8( 2, READ8() )    return;

				case 0xc0:  case 0xc1:  case 0xc2:  case 0xc3:  case 0xc4:  case 0xc5:  case 0xc6:  case 0xc7:
				case 0xc8:  case 0xc9:  case 0xca:  case 0xcb:  case 0xcc:  case 0xcd:  case 0xce:  case 0xcf:
					OPCC( JP,10,12 )    CC( 1, b2 - 0xc0 )  R16D8( 2, IX + b0 - 0xf4, b1 )  return;     // JP [cc,]ix+d
				case 0xd0:  case 0xd1:  case 0xd2:  case 0xd3:  case 0xd4:  case 0xd5:  case 0xd6:  case 0xd7:
				case 0xd8:  case 0xd9:  case 0xda:  case 0xdb:  case 0xdc:  case 0xdd:  case 0xde:  case 0xdf:
					OPCC( CALL,10,18 )  CC( 1, b2 - 0xd0 )  R16D8( 2, IX + b0 - 0xf4, b1 )  return;     // CALL [cc,]ix+d
			}   break;

		case 0xf7:
			b1 = READ8();
			switch ( b1 )   {
				case 0x20:  case 0x21:  case 0x22:  case 0x23:  case 0x24:  case 0x25:  case 0x26:
					OP( LD,14 )         MR16R8( 1, HL, A )      R8( 2, b1 - 0x20 )          return;     // LD (HL+A),r
				case 0x37:
					OP( LD,16 )         MR16R8( 1, HL, A )      I8( 2, READ8() )            return;     // LD (HL+A),n
				case 0x38:  case 0x39:  case 0x3a:  /*case 0x3b:*/  case 0x3c:  case 0x3d:  case 0x3e:
					OP( LDA,14 )        R16( 1, b1 - 0x38 )     R16R8( 2, HL, A )           return;     // LDA rr,HL+A
				case 0x3f:
					OP( LDW,20 )        MR16R8( 1, HL, A )      I16( 2, READ16() )          return;     // LDW (HL+A),mn
				case 0x40:  case 0x41:  case 0x42:  /*case 0x43:*/  case 0x44:  case 0x45:  case 0x46:
					OP16( LD,16 )       MR16R8( 1, HL, A )      R16( 2, b1 - 0x40 )         return;     // LD (HL+A),rr

				case 0x68:                                                                              // ADD (HL+A),n
				case 0x69:                                                                              // ADC (HL+A),n
				case 0x6a:                                                                              // SUB (HL+A),n
				case 0x6b:                                                                              // SBC (HL+A),n
				case 0x6c:                                                                              // AND (HL+A),n
				case 0x6d:                                                                              // XOR (HL+A),n
				case 0x6e:                                                                              // OR  (HL+A),n
					OP( ADD+b1-0x68,18) MR16R8( 1, HL, A )      I8( 2, READ8() )            return;
				case 0x6f:                                                                              // CP  (HL+A),n
					OP( ADD+b1-0x68,16) MR16R8( 1, HL, A )      I8( 2, READ8() )            return;

				case 0xc0:  case 0xc1:  case 0xc2:  case 0xc3:  case 0xc4:  case 0xc5:  case 0xc6:  case 0xc7:
				case 0xc8:  case 0xc9:  case 0xca:  case 0xcb:  case 0xcc:  case 0xcd:  case 0xce:  case 0xcf:
					OPCC( JP,14,16 )    CC( 1, b1 - 0xc0 )      R16R8( 2, HL, A )           return;     // JP [cc,]HL+A
				case 0xd0:  case 0xd1:  case 0xd2:  case 0xd3:  case 0xd4:  case 0xd5:  case 0xd6:  case 0xd7:
				case 0xd8:  case 0xd9:  case 0xda:  case 0xdb:  case 0xdc:  case 0xdd:  case 0xde:  case 0xdf:
					OPCC( CALL,14,22 )  CC( 1, b1 - 0xd0 )      R16R8( 2, HL, A )           return;     // CALL [cc,]HL+A
			}   break;

		case 0xf8:  case 0xf9:  case 0xfa:  case 0xfb:  case 0xfc:  case 0xfd:  case 0xfe:
			b1 = READ8();
			switch ( b1 )   {
				case 0x12:                                                                              // MUL HL,g
				case 0x13:                                                                              // DIV HL,g
					OP( MUL+b1-0x12,18) R16( 1, HL )            R8( 2, b0 - 0xf8 )          return;

				case 0x14:  case 0x15:  case 0x16:
					OP16( ADD,8 )       R16( 1, IX+b1-0x14 )    R16( 2, b0 - 0xf8 )         return;     // ADD ix,gg

				case 0x30:  case 0x31:  case 0x32:  case 0x33:  case 0x34:  case 0x35:  case 0x36:
					OP( LD,4 )          R8( 1, b1 - 0x30 )      R8( 2, b0 - 0xf8 )          return;     // LD r,g
				case 0x38:  case 0x39:  case 0x3a:  /*case 0x3b:*/  case 0x3c:  case 0x3d:  case 0x3e:
					OP16( LD,6 )        R16( 1, b1 - 0x38 )     R16( 2, b0 - 0xf8 )         return;     // LD rr,gg

				case 0x58:                                                                              // LDI
				case 0x59:                                                                              // LDIR
				case 0x5a:                                                                              // LDD
				case 0x5b:                                                                              // LDDR
				case 0x5c:                                                                              // CPI
				case 0x5d:                                                                              // CPIR
				case 0x5e:                                                                              // CPD
				case 0x5f:                                                                              // CPDR
				if (b0 == 0xfe) {
					OPCC( LDI+b1-0x58,14,18 )   NONE( 1 )       NONE( 2 )                   return;
				}
				break;

				case 0x60:                                                                              // ADD A,g
				case 0x61:                                                                              // ADC A,g
				case 0x62:                                                                              // SUB A,g
				case 0x63:                                                                              // SBC A,g
				case 0x64:                                                                              // AND A,g
				case 0x65:                                                                              // XOR A,g
				case 0x66:                                                                              // OR  A,g
				case 0x67:                                                                              // CP  A,g
					OP( ADD+b1-0x60,4 ) R8( 1, A )              R8( 2, b0 - 0xf8 )          return;

				case 0x68:                                                                              // ADD g,n
				case 0x69:                                                                              // ADC g,n
				case 0x6a:                                                                              // SUB g,n
				case 0x6b:                                                                              // SBC g,n
				case 0x6c:                                                                              // AND g,n
				case 0x6d:                                                                              // XOR g,n
				case 0x6e:                                                                              // OR  g,n
				case 0x6f:                                                                              // CP  g,n
					OP( ADD+b1-0x68,6 ) R8( 1, b0 - 0xf8 )      I8( 2, READ8() )            return;

				case 0x70:                                                                              // ADD HL,gg
				case 0x71:                                                                              // ADC HL,gg
				case 0x72:                                                                              // SUB HL,gg
				case 0x73:                                                                              // SBC HL,gg
				case 0x74:                                                                              // AND HL,gg
				case 0x75:                                                                              // XOR HL,gg
				case 0x76:                                                                              // OR  HL,gg
				case 0x77:                                                                              // CP  HL,gg
					OP16( ADD+b1-0x70,8 )   R16( 1, HL )        R16( 2, b0 - 0xf8 )         return;

				case 0xa0:                                                                              // RLC g
				case 0xa1:                                                                              // RRC g
				case 0xa2:                                                                              // RL  g
				case 0xa3:                                                                              // RR  g
				case 0xa4:                                                                              // SLA g
				case 0xa5:                                                                              // SRA g
				case 0xa6:                                                                              // SLL g
				case 0xa7:                                                                              // SRL g
					OP( RLC+b1-0xa0,4 ) R8( 1, b0 - 0xf8 )      NONE( 2 )                   return;

				case 0x18:  case 0x19:  case 0x1a:  case 0x1b:  case 0x1c:  case 0x1d:  case 0x1e:  case 0x1f:
					OP( TSET,8 )        BIT8( 1, b1 - 0x18 )    R8( 2, b0 - 0xf8 )          return;     // TSET b,g
				case 0xa8:  case 0xa9:  case 0xaa:  case 0xab:  case 0xac:  case 0xad:  case 0xae:  case 0xaf:
					OP( BIT,4 )         BIT8( 1, b1 - 0xa8 )    R8( 2, b0 - 0xf8 )          return;     // BIT b,g
				case 0xb0:  case 0xb1:  case 0xb2:  case 0xb3:  case 0xb4:  case 0xb5:  case 0xb6:  case 0xb7:
					OP( RES,4 )         BIT8( 1, b1 - 0xb0 )    R8( 2, b0 - 0xf8 )          return;     // RES b,g
				case 0xb8:  case 0xb9:  case 0xba:  case 0xbb:  case 0xbc:  case 0xbd:  case 0xbe:  case 0xbf:
					OP( SET,4 )         BIT8( 1, b1 - 0xb8 )    R8( 2, b0 - 0xf8 )          return;     // SET b,g

				case 0xd0:  case 0xd1:  case 0xd2:  case 0xd3:  case 0xd4:  case 0xd5:  case 0xd6:  case 0xd7:
				case 0xd8:  case 0xd9:  case 0xda:  case 0xdb:  case 0xdc:  case 0xdd:  case 0xde:  case 0xdf:
				if (b0 == 0xfe) {
					OPCC( RET,6,14 )    CC( 1, b1 - 0xd0 )      NONE( 2 )                   return;     // RET cc
				}
				break;
			}   break;

		case 0xff:
			OP( SWI,20 )                NONE( 1 )               NONE( 2 )                   return;     // SWI
	}

	OP( UNKNOWN,2 )     NONE( 1 )       NONE( 2 )
}

static uint16_t r8(tlcs90 *t, const uint16_t r)
{
	switch( r )
	{
		case A: return t->af.b.h;
		case B: return t->bc.b.h;
		case C: return t->bc.b.l;
		case D: return t->de.b.h;
		case E: return t->de.b.l;
		case H: return t->hl.b.h;
		case L: return t->hl.b.l;
	}
	return 0;   // MAME: fatalerror
}

static void w8(tlcs90 *t, const uint16_t r, uint16_t value)
{
	switch( r )
	{
		case A: t->af.b.h = (uint8_t)value;   return;
		case B: t->bc.b.h = (uint8_t)value;   return;
		case C: t->bc.b.l = (uint8_t)value;   return;
		case D: t->de.b.h = (uint8_t)value;   return;
		case E: t->de.b.l = (uint8_t)value;   return;
		case H: t->hl.b.h = (uint8_t)value;   return;
		case L: t->hl.b.l = (uint8_t)value;   return;
	}
}

static uint16_t r16(tlcs90 *t, const uint16_t r)
{
	switch( r )
	{
		case BC:    return t->bc.w.l;
		case DE:    return t->de.w.l;
		case HL:    return t->hl.w.l;
		case IX:    return t->ix.w.l;
		case IY:    return t->iy.w.l;
		case SP:    return t->sp.w.l;
		case AF:    return t->af.w.l;
// one interrupt flip-flop? Needed by e.g. mjifb
		case AF2:   return (t->af2.w.l & (~IF)) | (t->af.w.l & IF);
		case PC:    return t->pc.w.l;
	}
	return 0;   // MAME: fatalerror
}

static void w16(tlcs90 *t, const uint16_t r, uint16_t value)
{
	switch( r )
	{
		case BC:    t->bc.w.l  = value;  return;
		case DE:    t->de.w.l  = value;  return;
		case HL:    t->hl.w.l  = value;  return;
		case IX:    t->ix.w.l  = value;  return;
		case IY:    t->iy.w.l  = value;  return;
		case SP:    t->sp.w.l  = value;  return;
		case AF:    t->af.w.l  = value;  return;
		case AF2:   t->af2.w.l = value;  return;
		case PC:    t->pc.d = value; return;
	}
}


#define READ_FN( N ) \
static uint8_t read##N##_8(tlcs90 *t)    { \
	switch ( t->mode##N )    { \
		case TM_CC: \
		case TM_BIT8: \
		case TM_I8:       return (uint8_t)t->r##N; \
		case TM_D8:       return (uint8_t)t->r##N; \
		case TM_R8:       return (uint8_t)r8(t, t->r##N); \
		case TM_MI16:     return RM8(t, t->r##N); \
		case TM_MR16R8:   return RM8(t, (uint16_t)(r16(t, t->r##N) + (int8_t)r8(t, t->r##N##b))); \
		case TM_MR16: \
			switch( t->r##N ) { \
				case IX:    return RX8(t, t->ix.w.l,t->ixbase); \
				case IY:    return RX8(t, t->iy.w.l,t->iybase); \
			} \
			return RM8(t, r16(t, t->r##N)); \
		case TM_MR16D8: \
			switch( t->r##N ) { \
				case IX:    return RX8(t, (uint16_t)(t->ix.w.l + (int8_t)t->r##N##b),t->ixbase); \
				case IY:    return RX8(t, (uint16_t)(t->iy.w.l + (int8_t)t->r##N##b),t->iybase); \
			} \
			return RM8(t, (uint16_t)(r16(t, t->r##N) + (int8_t)t->r##N##b)); \
	} \
	return 0; \
} \
static uint16_t read##N##_16(tlcs90 *t)  { \
	switch ( t->mode##N )    { \
		case TM_I16:      return t->r##N; \
		case TM_D16:      return t->r##N - 1; \
		case TM_R16:      return r16(t, t->r##N); \
		case TM_R16D8:    return r16(t, t->r##N) + (int8_t)t->r##N##b; \
		case TM_R16R8:    return r16(t, t->r##N) + (int8_t)r8(t, t->r##N##b); \
		case TM_MI16:     return RM16(t, t->r##N); \
		case TM_MR16R8:   return RM16(t, (uint16_t)(r16(t, t->r##N) + (int8_t)r8(t, t->r##N##b))); \
		case TM_MR16: \
			switch( t->r##N ) { \
				case IX:    return RX16(t, t->ix.w.l,t->ixbase); \
				case IY:    return RX16(t, t->iy.w.l,t->iybase); \
			} \
			return RM16(t, r16(t, t->r##N)); \
		case TM_MR16D8: \
			switch( t->r##N ) { \
				case IX:    return RX16(t, (uint16_t)(t->ix.w.l + (int8_t)t->r##N##b),t->ixbase); \
				case IY:    return RX16(t, (uint16_t)(t->iy.w.l + (int8_t)t->r##N##b),t->iybase); \
			} \
			return RM16(t, (uint16_t)(r16(t, t->r##N) + (int8_t)t->r##N##b)); \
	} \
	return 0; \
}

#define WRITE_FN( N ) \
static void write##N##_8(tlcs90 *t, uint8_t value) { \
	switch ( t->mode##N )    { \
		case TM_R8:       w8(t, t->r##N,value);     return; \
		case TM_MI16:     WM8(t, t->r##N, value);   return; \
		case TM_MR16R8:   WM8(t, (uint16_t)(r16(t, t->r##N) + (int8_t)r8(t, t->r##N##b)), value);  return; \
		case TM_MR16: \
			switch( t->r##N ) { \
				case IX:    WX8(t, t->ix.w.l,value,t->ixbase); return; \
				case IY:    WX8(t, t->iy.w.l,value,t->iybase); return; \
			} \
			WM8(t, r16(t, t->r##N), value);    return; \
		case TM_MR16D8: \
			switch( t->r##N ) { \
				case IX:    WX8(t, (uint16_t)(t->ix.w.l + (int8_t)t->r##N##b),value,t->ixbase); return; \
				case IY:    WX8(t, (uint16_t)(t->iy.w.l + (int8_t)t->r##N##b),value,t->iybase); return; \
			} \
			WM8(t, (uint16_t)(r16(t, t->r##N) + (int8_t)t->r##N##b), value);    return; \
	} \
} \
static void write##N##_16(tlcs90 *t, uint16_t value) \
{ \
	switch ( t->mode##N )    { \
		case TM_R16:      w16(t, t->r##N,value);    return; \
		case TM_MI16:     WM16(t, t->r##N, value);  return; \
		case TM_MR16R8:   WM16(t, (uint16_t)(r16(t, t->r##N) + (int8_t)r8(t, t->r##N##b)), value); return; \
		case TM_MR16: \
			switch( t->r##N ) { \
				case IX:    WX16(t, t->ix.w.l,value,t->ixbase);    return; \
				case IY:    WX16(t, t->iy.w.l,value,t->iybase);    return; \
			} \
			WM16(t, r16(t, t->r##N), value);   return; \
		case TM_MR16D8: \
			switch( t->r##N ) { \
				case IX:    WX16(t, (uint16_t)(t->ix.w.l + (int8_t)t->r##N##b),value,t->ixbase);    return; \
				case IY:    WX16(t, (uint16_t)(t->iy.w.l + (int8_t)t->r##N##b),value,t->iybase);    return; \
			} \
			WM16(t, (uint16_t)(r16(t, t->r##N) + (int8_t)t->r##N##b), value);   return; \
	} \
}

READ_FN(1)
READ_FN(2)
WRITE_FN(1)
WRITE_FN(2)

static int test_cc(tlcs90 *t, uint8_t cond)
{
	int s,v;
	switch ( cond )
	{
		case FLS:   return 0;
		case LT:    s = F & SF; v = F & VF; return (s && !v) || (!s && v);
		case LE:    s = F & SF; v = F & VF; return (F & ZF) || (s && !v) || (!s && v);
		case ULE:   return (F & CF) || (F & ZF);
		case OV:    return F & VF;
		case MI:    return F & SF;
		case Z:     return F & ZF;
		case CR:    return F & CF;
		case T:     return 1;
		case GE:    s = F & SF; v = F & VF; return (s && v) || (!s && !v);
		case GT:    s = F & SF; v = F & VF; return !((F & ZF) || (s && !v) || (!s && v));
		case UGT:   return !(F & CF) && !(F & ZF);
		case NOV:   return !(F & VF);
		case PL:    return !(F & SF);
		case NZ:    return !(F & ZF);
		case NC:    return !(F & CF);
	}
	return 0;   // MAME: fatalerror
}

static void push_rr(tlcs90 *t, uint16_t rr)
{
	t->sp.w.l -= 2;
	WM16(t, t->sp.w.l, r16(t, rr));
}
static void pop_rr(tlcs90 *t, uint16_t rr)
{
	w16(t, rr, RM16(t, t->sp.w.l));
	t->sp.w.l += 2;
}

/*************************************************************************************************************

Interrupts (MAME's table)

----------------------------------------------------------------------------------------------------------------
Priority    Type            Interrupt Source            Vector/8    Vector  Address         uDMA Address
----------------------------------------------------------------------------------------------------------------
1           Non Maskable    SWI                         -           10      0010            -
2           ""              NMI                         -           18      0018            -
3           ""              INTWD   Watch Dog           -           20      0020            -
4           Maskable        INT0    External 0          05          28      0028            FF28
5           ""              INTT0   Timer 0             06          30      0030            FF30
6           ""              INTT1   Timer 1             07          38      0038            FF38
7           ""              INTAD   A/D                 08          40      0040            FF48
""          ""              INTT2   Timer 2             ""          ""      ""              ""
8           ""              INTT3   Timer 3             09          48      0048            FF48
9           ""              INTT4   Timer 4             0A          50      0050            FF50
10          ""              INT1    External 1          0B          58      0058            FF58
11          ""              INTT5   Timer 5             0C          60      0060            FF60
12          ""              INT2    External 2          0D          68      0068            FF68
13          ""              INTRX   End Serial Receive  0E          70      0070            FF70
14          ""              INTTX   End Serial Transmit 0F          78      0078            FF78

----------------------------------------------------------------------------------------------------------------
Interrupt   Terminal    Mode            How To Set
----------------------------------------------------------------------------------------------------------------
NMI         -           Falling Edge    -
INT0        P80         Level           P8CR<EDGE> = 0
                        Rising Edge     P8CR<EDGE> = 1
INT1        P81         Rising Edge     T4MOD<CAPM1,0> = 0,0 or 0,1 or 1,1
                        Falling Edge    T4MOD<CAPM1,0> = 1,0
INT2        P82         Rising Edge     -

*************************************************************************************************************/

static void enter_halt(tlcs90 *t) { t->halt = 1; }

static void leave_halt(tlcs90 *t)
{
	if( t->halt )
		t->halt = 0;
}

static void take_interrupt(tlcs90 *t, int irq)
{
	if (irq != TLCS90_INT0 || (t->p8cr & 1) == 1)
		clear_irq(t, irq);

	leave_halt(t);

	push_rr(t, PC);
	push_rr(t, AF);

	F &= ~IF;

	t->pc.w.l = 0x10 + irq * 8;

	t->extra_cycles += 20*2;
}

static void check_interrupts(tlcs90 *t)
{
	int irq;
	int mask;

	if (!(F & IF))
		return;

	for (irq = TLCS90_INTSWI; irq < TLCS90_INTMAX; ++irq)
	{
		mask = (1 << irq);
		if(irq >= TLCS90_INT0) mask &= t->irq_mask;

		if (t->irq_state & mask)
		{
			take_interrupt(t, irq);
			return;
		}
	}
}

static void set_irq_line(tlcs90 *t, int irq, int state)
{
	if ( ((t->irq_line_state >> irq)&1) == state ) return;

	if (state)
	{
		raise_irq(t, irq);
		t->irq_line_state |= 1 << irq;
	}
	else
	{
		if (irq == TLCS90_INT0 && (t->p8cr & 1) == 0)
			clear_irq(t, irq);
		t->irq_line_state &= ~(1 << irq);
	}
}

void tlcs90_set_irq(tlcs90 *t, int line, int state)
{
	state = state ? 1 : 0;
	switch(line) {
		case TLCS90_LINE_NMI:  set_irq_line(t, TLCS90_INTNMI, state); break;
		case TLCS90_LINE_INT0: set_irq_line(t, TLCS90_INT0, state); break;
		case TLCS90_LINE_INT1: set_irq_line(t, TLCS90_INT1, state); break;
		case TLCS90_LINE_INT2: set_irq_line(t, TLCS90_INT2, state); break;
	}
}

#define Cyc()         (t->icount -= t->cyc_t)
#define Cyc_f()       (t->icount -= t->cyc_f)
#define Read1_8()     read1_8(t)
#define Read1_16()    read1_16(t)
#define Read2_8()     read2_8(t)
#define Read2_16()    read2_16(t)
#define Write1_8(v)   write1_8(t, (uint8_t)(v))
#define Write1_16(v)  write1_16(t, (uint16_t)(v))
#define Write2_8(v)   write2_8(t, (uint8_t)(v))
#define Write2_16(v)  write2_16(t, (uint16_t)(v))
#define Test(c)       test_cc(t, (uint8_t)(c))
#define Push(rr)      push_rr(t, rr)
#define Pop(rr)       pop_rr(t, rr)
#define halt()        enter_halt(t)
#define take_interrupt(irq) take_interrupt(t, irq)
#define INTSWI        TLCS90_INTSWI

// halted with nothing pending: iterations of the halted NOP loop before a timer ticks or the slice ends
static int halt_skip(tlcs90 *t)
{
	int cyc = t->cyc_t;
	if (cyc <= 0 || t->after_EI) return 1;
	int n = (t->icount + cyc - 1) / cyc;
	for (int i = 0; i < 5; i++)
		if (t->timer_run & (1 << i))
		{
			int k = (t->timer_left[i] + cyc - 1) / cyc;
			if (k < n) n = k;
		}
	return n < 1 ? 1 : n;
}

int tlcs90_execute(tlcs90 *t, int cycles)
{
	uint8_t    a8,b8;
	uint16_t   a16,b16;
	unsigned a32;
	tlcs90_pair tmp;

	t->icount = cycles;
	t->icount -= t->extra_cycles;
	t->extra_cycles = 0;

	do
	{
		int ic0 = t->icount, ex0 = t->extra_cycles;

		check_interrupts(t);

		// when in HALT state, the fetched opcode is not dispatched (aka a NOP) and the PC is not increased
		if (t->halt)
		{
			t->op = NOP;
			int n = halt_skip(t);
			if (n > 1)
			{
				t->icount -= t->cyc_t * (n - 1);
			}
		}
		else
		{
			t->prvpc.d = t->pc.d;

			t->addr = t->pc.d;
			decode(t);
			t->pc.d = t->addr;
		}

		switch ( t->op )
		{
			case NOP:
				Cyc();
				break;

			case EX:
				a16 = Read1_16();
				Write1_16( Read2_16() );
				Write2_16( a16 );
				Cyc();
				break;
			case EXX:
				tmp = t->bc; t->bc = t->bc2;   t->bc2 = tmp;
				tmp = t->de; t->de = t->de2;   t->de2 = tmp;
				tmp = t->hl; t->hl = t->hl2;   t->hl2 = tmp;
				Cyc();
				break;

			case LD:
				Write1_8( Read2_8() );
				Cyc();
				break;
			case LDW:
			case LD | OP_16:
				Write1_16( Read2_16() );
				Cyc();
				break;

//          case LDA:
//              Cyc();
//              break;

			case LDI:
#define _LDI                                            \
				WM8(t, t->de.w.l, RM8(t, t->hl.w.l) );     \
				t->de.w.l++;                         \
				t->hl.w.l++;                         \
				t->bc.w.l--;                         \
				F &= SF | ZF | IF | XCF | CF;           \
				if ( t->bc.w.l ) F |= VF;

				_LDI
				Cyc();
				break;
			case LDIR:
				_LDI
				if ( t->bc.w.l )
				{
					t->pc.w.l -= 2;
					Cyc();
				}
				else    Cyc_f();
				break;

			case LDD:
#define _LDD                                            \
				WM8(t, t->de.w.l, RM8(t, t->hl.w.l) );     \
				t->de.w.l--;                         \
				t->hl.w.l--;                         \
				t->bc.w.l--;                         \
				F &= SF | ZF | IF | XCF | CF;           \
				if ( t->bc.w.l ) F |= VF;

				_LDD
				Cyc();
				break;
			case LDDR:
				_LDD
				if ( t->bc.w.l )
				{
					t->pc.w.l -= 2;
					Cyc();
				}
				else    Cyc_f();
				break;


//          case CPD:
//              Cyc();
//              break;
//          case CPDR:
//              Cyc();
//              break;
			case CPI:
				a8 = RM8(t, t->hl.w.l);
				b8 = t->af.b.h - a8;
				t->hl.w.l++;
				t->bc.w.l--;
				F = (F & (IF | CF)) | SZ[b8] | ((t->af.b.h^a8^b8)&HF) | NF;
				if ( t->bc.w.l ) F |= VF;
				Cyc();
				break;
			case CPIR:
				a8 = RM8(t, t->hl.w.l);
				b8 = t->af.b.h - a8;
				t->hl.w.l++;
				t->bc.w.l--;
				F = (F & (IF | CF)) | SZ[b8] | ((t->af.b.h^a8^b8)&HF) | NF;
				if ( t->bc.w.l )
				{
					F |= VF;
					t->pc.w.l -= 2;
					Cyc();
				}
				else    Cyc_f();
				break;

			case PUSH:
				Push( t->r1 );
				Cyc();
				break;
			case POP:
				Pop( t->r1 );
				Cyc();
				break;

			case JP:
				if ( Test( Read1_8() ) )
				{
					t->pc.w.l = Read2_16();
					Cyc();
				}
				else    Cyc_f();
				break;
			case JR:
				if ( Test( Read1_8() ) )
				{
					t->pc.w.l += /*2 +*/ (int8_t)Read2_8();
					Cyc();
				}
				else    Cyc_f();
				break;
			case JR | OP_16:
				if ( Test( Read1_8() ) )
				{
					t->pc.w.l += /*2 +*/ Read2_16();
					Cyc();
				}
				else    Cyc_f();
				break;


			case CALL:
				if ( Test( Read1_8() ) )
				{
					Push( PC );
					t->pc.w.l = Read2_16();
					Cyc();
				}
				else    Cyc_f();
				break;
			case CALLR:
				Push( PC );
				t->pc.w.l += /*2 +*/ Read1_16();
				Cyc();
				break;

			case RET:
				if ( Test( Read1_8() ) )
				{
					Pop( PC );
					Cyc();
				}
				else    Cyc_f();
				break;
			case RETI:
				Pop( AF );
				Pop( PC );
				Cyc();
				break;

			case HALT:
				halt();
				Cyc();
				break;
			case DI:
				t->after_EI = 0;
				F &= ~IF;
				Cyc();
				break;
			case EI:
				t->after_EI = !(F & IF);
				Cyc();
				break;

			case SWI:
				Cyc();
				take_interrupt( INTSWI );
				break;

			case DAA:
			{
				uint8_t cf, nf, hf, lo, hi, diff;
				cf = F & CF;
				nf = F & NF;
				hf = F & HF;
				lo = t->af.b.h & 15;
				hi = t->af.b.h / 16;

				if (cf)
				{
					diff = (lo <= 9 && !hf) ? 0x60 : 0x66;
				}
				else
				{
					if (lo >= 10)
					{
						diff = hi <= 8 ? 0x06 : 0x66;
					}
					else
					{
						if (hi >= 10)
						{
							diff = hf ? 0x66 : 0x60;
						}
						else
						{
							diff = hf ? 0x06 : 0x00;
						}
					}
				}
				if (nf) t->af.b.h -= diff;
				else t->af.b.h += diff;

				F = SZP[A] | (F & (IF | NF));
				if (cf || (lo <= 9 ? hi >= 10 : hi >= 9)) F |= XCF | CF;
				if (nf ? hf && lo <= 5 : lo >= 10)  F |= HF;
			}
				Cyc();
				break;


			case CPL:
				t->af.b.h ^= 0xff;
				F |= HF | NF;
				Cyc();
				break;
			case NEG:
				a8 = 0;
				b8 = t->af.b.h;
				a32 = a8 - b8;
				F = (F & IF) | SZ[(uint8_t)a32] | NF;
				if (a32 & 0x100)            F |= CF | XCF;  //X?
				if ((a8 ^ a32 ^ b8) & 0x10) F |= HF;
				if ((b8 ^ a8) & (a8 ^ a32) & 0x80)  F |= VF;
				t->af.b.h = a32;
				Cyc();
				break;

			case LDAR:
				t->hl.w.l = t->pc.w.l + /*2 +*/ Read2_16();
				Cyc();
				break;

			case RCF:
				F &= SF | ZF | IF | VF;
				Cyc();
				break;
			case SCF:
				F = (F & (SF | ZF | IF | VF)) | XCF | CF;
				Cyc();
				break;
			case CCF:
				F = (F & (SF | ZF | IF | VF)) | ((F & CF)?HF:(XCF | CF));
				Cyc();
				break;

//          case TSET:
//              Cyc();
//              break;
			case BIT:
				F = (F & (IF | CF)) | HF | SZ_BIT[ Read2_8() & (1 << Read1_8()) ];
				Cyc();
				break;
			case SET:
				Write2_8( Read2_8() | (1 << Read1_8()) );
				Cyc();
				break;
			case RES:
				Write2_8( Read2_8() & (~(1 << Read1_8())) );
				Cyc();
				break;

			case INC:
				a8 = Read1_8() + 1;
				Write1_8( a8 );
				F = (F & (IF | CF)) | SZHV_inc[a8];
				if (a8 == 0)    F |= XCF;
				Cyc();
				break;
			case INCX:
				if ( F & XCF )
				{
					a8 = Read1_8() + 1;
					Write1_8( a8 );
					F = (F & (IF | CF)) | SZHV_inc[a8];
					if (a8 == 0)    F |= XCF;
					Cyc();
				}
				else    Cyc_f();
				break;
			case INC | OP_16:
				a16 = Read1_16() + 1;
				Write1_16( a16 );
				if (a16 == 0)   F |=  XCF;
				else            F &= ~XCF;
				Cyc();
				break;
			case INCW:
				a16 = Read1_16();
				a32 = a16 + 1;
				Write1_16( a32 );
				F &= IF | CF;
				if ((uint16_t)a32 == 0)   F |= ZF | XCF;
				if (a32 & 0x8000)       F |= SF;
				if ((a16 ^ 0x8000) & a32 & 0x8000)  F |= VF;
				if ((a16 ^ a32 ^ 1) & 0x1000)   F |= HF;    //??
				Cyc();
				break;


			case DEC:
				a8 = Read1_8() - 1;
				Write1_8( a8 );
				F = (F & (IF | CF)) | SZHV_dec[a8];
				if (a8 == 0)    F |= XCF;
				Cyc();
				break;
			case DECX:
				if ( F & XCF )
				{
					a8 = Read1_8() - 1;
					Write1_8( a8 );
					F = (F & (IF | CF)) | SZHV_dec[a8];
					if (a8 == 0)    F |= XCF;
					Cyc();
				}
				else    Cyc_f();
				break;
			case DEC | OP_16:
				a16 = Read1_16() - 1;
				Write1_16( a16 );
				if (a16 == 0)   F |=  XCF;
				else            F &= ~XCF;
				Cyc();
				break;
			case DECW:
				a16 = Read1_16();
				a32 = a16 - 1;
				Write1_16( a32 );
				F = (F & (IF | CF)) | NF;
				if ((uint16_t)a32 == 0)   F |= ZF | XCF;
				if (a32 & 0x8000)       F |= SF;
				if (a16 == 0x8000)      F |= VF;
				if ((a16 ^ a32 ^ 1) & 0x1000)   F |= HF;    //??
				Cyc();
				break;

			case ADD:
			case ADC:
				a8 = Read1_8();
				b8 = Read2_8();
				a32 = a8 + b8;
				if ( (t->op == ADC) && (F & CF) )    a32 += 1;
				Write1_8( a32 );
				F = (F & IF) | SZ[(uint8_t)a32];
				if (a32 & 0x100)            F |= CF | XCF;  //X?
				if ((a8 ^ a32 ^ b8) & 0x10) F |= HF;
				if ((b8 ^ a8 ^ 0x80) & (b8 ^ a32) & 0x80)   F |= VF;
				Cyc();
				break;
			case ADD | OP_16:
			case ADC | OP_16:
				a16 = Read1_16();
				b16 = Read2_16();
				a32 = a16 + b16;
				if ( (t->op == (ADC | OP_16)) && (F & CF) )  a32 += 1;
				Write1_16( a32 );
				if ( (t->op == (ADD | OP_16)) && t->mode2 == TM_R16 )
				{
					F &= SF | ZF | IF | VF;
				}
				else
				{
					F &= IF;
					if ((uint16_t)a32 == 0)           F |= ZF;
					if (a32 & 0x8000)               F |= SF;
					if ((b16 ^ a16 ^ 0x8000) & (b16 ^ a32) & 0x8000)    F |= VF;
				}
				if (a32 & 0x10000)              F |= CF | XCF;  //X?
				if ((a16 ^ a32 ^ b16) & 0x1000) F |= HF;    //??
				Cyc();
				break;

			case CP:
			case SUB:
			case SBC:
				a8 = Read1_8();
				b8 = Read2_8();
				a32 = a8 - b8;
				if ( (t->op == SBC) && (F & CF) )    a32 -= 1;
				F = (F & IF) | SZ[(uint8_t)a32] | NF;
				if (a32 & 0x100)            F |= CF | XCF;  //X?
				if ((a8 ^ a32 ^ b8) & 0x10) F |= HF;
				if ((b8 ^ a8) & (a8 ^ a32) & 0x80)  F |= VF;
				if (t->op != CP)
					Write1_8( a32 );
				Cyc();
				break;
			case CP | OP_16:
			case SUB | OP_16:
			case SBC | OP_16:
				a16 = Read1_16();
				b16 = Read2_16();
				a32 = a16 - b16;
				if ( (t->op == (SBC | OP_16)) && (F & CF) )  a32 -= 1;
				F = (F & IF) | NF;
				if ((uint16_t)a32 == 0)           F |= ZF;
				if (a32 & 0x8000)               F |= SF;
				if (a32 & 0x10000)              F |= CF | XCF;  //X?
				if ((a16 ^ a32 ^ b16) & 0x1000) F |= HF;    //??
				if ((b16 ^ a16) & (a16 ^ a32) & 0x8000) F |= VF;
				if (t->op != (CP | OP_16))
					Write1_16( a32 );
				Cyc();
				break;

			case AND:
				a8 = Read1_8() & Read2_8();
				Write1_8( a8 );
				F = (F & IF) | SZP[a8] | HF;
				Cyc();
				break;
			case AND | OP_16:
				a16 = Read1_16() & Read2_16();
				Write1_16( a16 );
				F = (F & IF) | HF;
				if (a16 == 0)       F |= ZF;
				if (a16 & 0x8000)   F |= SF;
				Cyc();
				break;
			case XOR:
				a8 = Read1_8() ^ Read2_8();
				Write1_8( a8 );
				F = (F & IF) | SZP[a8];
				Cyc();
				break;
			case XOR | OP_16:
				a16 = Read1_16() ^ Read2_16();
				Write1_16( a16 );
				F &= IF;
				if (a16 == 0)       F |= ZF;
				if (a16 & 0x8000)   F |= SF;
				Cyc();
				break;
			case OR:
				a8 = Read1_8() | Read2_8();
				Write1_8( a8 );
				F = (F & IF) | SZP[a8];
				Cyc();
				break;
			case OR | OP_16:
				a16 = Read1_16() | Read2_16();
				Write1_16( a16 );
				F &= IF;
				if (a16 == 0)       F |= ZF;
				if (a16 & 0x8000)   F |= SF;
				Cyc();
				break;

			case RLC:
				a8 = Read1_8();
				a8 = (a8 << 1) | (a8 >> 7);
				Write1_8( a8 );
				if ( t->mode1 == TM_R8 && t->r1 == A )  F &= SF | ZF | IF | PF;
				else                                F = (F & IF) | SZP[a8];
				if (a8 & 0x01)                      F |= CF | XCF;  // X?
				Cyc();
				break;
			case RRC:
				a8 = Read1_8();
				a8 = (a8 >> 1) | (a8 << 7);
				Write1_8( a8 );
				if ( t->mode1 == TM_R8 && t->r1 == A )  F &= SF | ZF | IF | PF;
				else                                F = (F & IF) | SZP[a8];
				if (a8 & 0x80)                      F |= CF | XCF;  // X?
				Cyc();
				break;
			case RL:
				a8 = Read1_8();
				b8 = a8 & 0x80;
				a8 <<= 1;
				if (F & CF) a8 |= 0x01;
				Write1_8( a8 );
				if ( t->mode1 == TM_R8 && t->r1 == A )  F &= SF | ZF | IF | PF;
				else                                F = (F & IF) | SZP[a8];
				if (b8)                             F |= CF | XCF;  // X?
				Cyc();
				break;
			case RR:
				a8 = Read1_8();
				b8 = a8 & 0x01;
				a8 >>= 1;
				if (F & CF) a8 |= 0x80;
				Write1_8( a8 );
				if ( t->mode1 == TM_R8 && t->r1 == A )  F &= SF | ZF | IF | PF;
				else                                F = (F & IF) | SZP[a8];
				if (b8)                             F |= CF | XCF;  // X?
				Cyc();
				break;

			case SLA:
			case SLL:
				a8 = Read1_8();
				b8 = a8 & 0x80;
				a8 <<= 1;
				Write1_8( a8 );
				if ( t->mode1 == TM_R8 && t->r1 == A )  F &= SF | ZF | IF | PF;
				else                                F = (F & IF) | SZP[a8];
				if (b8)                             F |= CF | XCF;  // X?
				Cyc();
				break;
			case SRA:
				a8 = Read1_8();
				b8 = a8 & 0x01;
				a8 = (a8 & 0x80) | (a8 >> 1);
				Write1_8( a8 );
				if ( t->mode1 == TM_R8 && t->r1 == A )  F &= SF | ZF | IF | PF;
				else                                F = (F & IF) | SZP[a8];
				if (b8)                             F |= CF | XCF;  // X?
				Cyc();
				break;
			case SRL:
				a8 = Read1_8();
				b8 = a8 & 0x01;
				a8 >>= 1;
				Write1_8( a8 );
				if ( t->mode1 == TM_R8 && t->r1 == A )  F &= SF | ZF | IF | PF;
				else                                F = (F & IF) | SZP[a8];
				if (b8)                             F |= CF | XCF;  // X?
				Cyc();
				break;
			case RLD:
				a8 = t->af.b.h;
				b8 = Read1_8();
				Write1_8( (b8 << 4) | (a8 & 0x0f) );
				a8 = (a8 & 0xf0) | (b8 >> 4);
				F = (F & (IF | CF)) | SZP[a8];
				t->af.b.h = a8;
				Cyc();
				break;
			case RRD:
				a8 = t->af.b.h;
				b8 = Read1_8();
				Write1_8( (b8 >> 4) | (a8 << 4) );
				a8 = (a8 & 0xf0) | (b8 & 0x0f);
				F = (F & (IF | CF)) | SZP[a8];
				t->af.b.h = a8;
				Cyc();
				break;

			case DJNZ:
				if ( --t->bc.b.h )
				{
					t->pc.w.l += /*2 +*/ (int8_t)Read1_8();
					Cyc();
				}
				else    Cyc_f();
				break;
			case DJNZ | OP_16:
				if ( --t->bc.w.l )
				{
					t->pc.w.l += /*2 +*/ (int8_t)Read2_8();
					Cyc();
				}
				else    Cyc_f();
				break;

			case MUL:
				t->hl.w.l = (uint16_t)t->hl.b.l * (uint16_t)Read2_8();
				Cyc();
				break;
			case DIV:
				a16 = t->hl.w.l;
				b16 = (uint16_t)Read2_8();
				if (b16 == 0)
				{
					F |= VF;
					t->hl.w.l = (a16 << 8) | ((a16 >> 8) ^ 0xff);
				}
				else
				{
					t->hl.b.h = a16 % b16;
					a16 /= b16;
					if (a16 > 0xff) F |=  VF;
					else            F &= ~VF;
					t->hl.b.l = a16;
				}
				Cyc();
				break;

			default:
				Cyc();  // MAME: fatalerror (TSET, LDA, CPD, CPDR, undefined)
				break;
		}

		if ( t->op != EI )
			if (t->after_EI)
			{
				F |= IF;
				t->after_EI = 0;
			}

		// internal timers: the clocks of this step (interrupt entry cycles included, though MAME charges
		// them to icount only at the end of the slice)
		if (t->timer_run)
			timers_step(t, (ic0 - t->icount) + (t->extra_cycles - ex0));

	} while( t->icount > 0 );

	t->icount -= t->extra_cycles;
	t->extra_cycles = 0;
	return cycles - t->icount;
}

void tlcs90_reset(tlcs90 *t)
{
	leave_halt(t);

	t->irq_state = 0;
	t->irq_mask = 0;
	t->pc.d = 0x0000;
	F &= ~IF;
/*
    P0/D0-D7 P1/A0-A7 P2/A8-A15 P6 P7 = INPUT
    P35/~RD P36/~WR CLK = 1 (ALWAYS OUTPUTS)
    P4/A16-A19 P83 = 0
    dedicated input ports and CPU registers remain unchanged,
    but PC IFF BX BY = 0, A undefined
*/

	memset(t->port_latch, 0, sizeof(t->port_latch));
	t->p01cr = 0;
	t->p2cr = 0;
	t->p4cr = 0;
	t->p67cr = 0;
	t->p8cr = 0;
	t->smmod = 0;

	t->ixbase = 0;
	t->iybase = 0;

	t->tmod = 0;
	t->tclk = 0;
	t->trun = 0;
	t->t4mod = 0;
	memset(t->treg_8bit, 0, sizeof(t->treg_8bit));
	memset(t->treg_16bit, 0, sizeof(t->treg_16bit));

	// as MAME, armed timers stay armed across a reset (timers 0-3 then return early with TRUN = 0;
	// the timer 4 callback does not check TRUN)
}

void tlcs90_init(tlcs90 *t, int type)
{
	// internal ROM size, internal RAM start, SFR end, TMP90844 layout (MAME tmp9xxxx_mem maps)
	static const struct { uint32_t rom, ram, sfr_end; int r844; } info[] = {
		{ 0x2000, 0xfec0, 0xffef, 0 },   // TMP90840
		{ 0,      0xfec0, 0xffef, 0 },   // TMP90841 (rom-less)
		{ 0,      0xfec0, 0xffef, 0 },   // TMP90845 (MAME uses the tmp90841 map)
		{ 0x4000, 0xfdc0, 0xffef, 0 },   // TMP91640
		{ 0,      0xfdc0, 0xffef, 0 },   // TMP91641 (rom-less)
		{ 0x4000, 0xfdc0, 0xfff7, 1 },   // TMP90PH44
	};

	if (!s_tables_done)
	{
		for (int i = 0; i < 256; i++)
		{
			int p = 0;
			if( i&0x01 ) ++p;
			if( i&0x02 ) ++p;
			if( i&0x04 ) ++p;
			if( i&0x08 ) ++p;
			if( i&0x10 ) ++p;
			if( i&0x20 ) ++p;
			if( i&0x40 ) ++p;
			if( i&0x80 ) ++p;
			SZ[i] = i ? i & SF : ZF;
			SZ_BIT[i] = i ? i & SF : ZF | PF;
			SZP[i] = SZ[i] | ((p & 1) ? 0 : PF);
			SZHV_inc[i] = SZ[i];
			if( i == 0x80 ) SZHV_inc[i] |= VF;
			if( (i & 0x0f) == 0x00 ) SZHV_inc[i] |= HF;
			SZHV_dec[i] = SZ[i] | NF;
			if( i == 0x7f ) SZHV_dec[i] |= VF;
			if( (i & 0x0f) == 0x0f ) SZHV_dec[i] |= HF;
		}
		s_tables_done = 1;
	}

	if (type < 0 || type > TLCS90_TMP90PH44) type = TLCS90_TMP90840;
	memset(t, 0, sizeof(*t));   // device_start: all registers and work state 0
	t->type = type;
	t->rom_size = info[type].rom;
	t->ram_start = info[type].ram;
	t->sfr_end = info[type].sfr_end;
	t->regs844 = info[type].r844;
}
