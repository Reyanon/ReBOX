/*  NEC V30 / V33 / V35. Ported from MAME src/devices/cpu/nec (BSD-3-Clause, copyright-holders: Bryan McPhail,
    Alex W. Jackson): nec.cpp, v25.cpp, v25sfr.cpp here, the instructions in nec_core.inc.
    Written for an ARM host: one big function per chip (the cycle counts folded at compile time, no dispatch through
    member pointers), bytes fetched through a pointer into a page of 4K, the instruction's length and the prefetch
    queue's use counted from that pointer once per instruction, memory through a page map.
    Left as MAME has them: V25 macro service, serial, and the DMA (no Irem game starts it). Not done: the 8080 mode of
    the V30 (BRKEM), the trap flag: no Irem game uses them.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "nec.h"

#include <stddef.h>
#include <string.h>

#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

typedef uint16_t __attribute__((may_alias)) u16a;

enum { DS1 = 7, PS = 6, SS = 5, DS0 = 4 };
enum { AW = 15, CW = 14, DW = 13, BW = 12, SP = 11, BP = 10, IX = 9, IY = 8 };
enum { AL = 0x1e, AH = 0x1f, CL = 0x1c, CH = 0x1d, DL = 0x1a, DH = 0x1b, BL = 0x18, BH = 0x19 };
enum { VECTOR_PC = 1, PSW_SAVE = 2, PC_SAVE = 3 };

enum {
	NEC_DIVIDE_VECTOR = 0, NEC_TRAP_VECTOR = 1, NEC_NMI_VECTOR = 2, NEC_BRKV_VECTOR = 4, NEC_CHKIND_VECTOR = 5,
	NEC_IBRK_VECTOR = 19, NEC_INTTU0_VECTOR = 28, NEC_INTTU1_VECTOR = 29, NEC_INTTU2_VECTOR = 30,
	NEC_INTD0_VECTOR = 20, NEC_INTD1_VECTOR = 21, NEC_INTP0_VECTOR = 24, NEC_INTP1_VECTOR = 25, NEC_INTP2_VECTOR = 26,
	NEC_INTSER0_VECTOR = 12, NEC_INTSR0_VECTOR = 13, NEC_INTST0_VECTOR = 14, NEC_INTSER1_VECTOR = 16,
	NEC_INTSR1_VECTOR = 17, NEC_INTST1_VECTOR = 18, NEC_INTTB_VECTOR = 31
};

enum {
	BRK = 0, INT_IRQ = 1, NMI_IRQ = 1 << 1, INTTU0 = 1 << 2, INTTU1 = 1 << 3, INTTU2 = 1 << 4, INTD0 = 1 << 5,
	INTD1 = 1 << 6, INTP0 = 1 << 7, INTP1 = 1 << 8, INTP2 = 1 << 9, INTSER0 = 1 << 10, INTSR0 = 1 << 11,
	INTST0 = 1 << 12, INTSER1 = 1 << 13, INTSR1 = 1 << 14, INTST1 = 1 << 15, INTTB = 1 << 16, BRKN = 1 << 17,
	BRKS = 1 << 18
};

#define CF (S->CarryVal != 0)
#define SF (S->SignVal < 0)
#define ZF (S->ZeroVal == 0)
#define AF (S->AuxVal != 0)
#define OF (S->OverVal != 0)

typedef uint8_t BYTE;
typedef uint16_t WORD;

static uint8_t nec_parity[256];

// ---- V25 / V35 internal data area (v25sfr.cpp) ----
#define BIT(v, n) (((v) >> (n)) & 1)

static void tm_adjust(nec_t *c, int t, int64_t delay, uint32_t param, int64_t period)
{
	c->tm_next[t] = delay > 0 ? c->total + delay : 0;
	c->tm_period[t] = period;
	c->tm_param[t] = param;
}

static void tmc0_w(nec_t *c, uint8_t d)
{
	c->TMC0 = d;
	if (BIT(d, 0)) {   // (one shot)
		if (BIT(d, 7) && c->TM0 != 0)
			tm_adjust(c, 0, (int64_t)c->PCK * c->TM0 * (BIT(d, 6) ? 128 : 12), INTTU0, 0);
		else
			c->tm_next[0] = 0;
		if (BIT(d, 5) && c->MD0 != 0)
			tm_adjust(c, 1, (int64_t)c->PCK * c->MD0 * (BIT(d, 4) ? 128 : 12), INTTU1, 0);
		else
			c->tm_next[1] = 0;
	} else {           // (interval)
		if (BIT(d, 7) && c->MD0 != 0) {
			const int64_t time = (int64_t)c->PCK * c->MD0 * (BIT(d, 6) ? 128 : 6);
			tm_adjust(c, 0, time, INTTU0, time);
			c->tm_next[1] = 0;
			c->TM0 = c->MD0;
		} else {
			c->tm_next[0] = 0;
			c->tm_next[1] = 0;
		}
	}
}

static void tmc1_w(nec_t *c, uint8_t d)
{
	c->TMC1 = d & 0xc0;
	if (BIT(d, 7) && c->MD1 != 0) {
		const int64_t time = (int64_t)c->PCK * c->MD1 * (BIT(d, 6) ? 128 : 6);
		tm_adjust(c, 2, time, INTTU1 | INTTU2, time);
		c->TM1 = c->MD1;
	} else
		c->tm_next[2] = 0;
}

static uint8_t read_irqcontrol(nec_t *c, uint32_t source, uint8_t priority)
{
	return (uint8_t)(((c->pending_irq & source) ? 0x80 : 0x00) | ((c->unmasked_irq & source) ? 0x00 : 0x40) |
	                 ((c->macro_service & source) ? 0x20 : 0x00) | ((c->bankswitch_irq & source) ? 0x10 : 0x00) | priority);
}

static void write_irqcontrol(nec_t *c, uint32_t source, uint8_t d)
{
	if (BIT(d, 7))
		c->pending_irq |= source;
	else
		c->pending_irq &= ~source;
	if (BIT(d, 6))
		c->unmasked_irq &= ~source;
	else
		c->unmasked_irq |= source;
	if (BIT(d, 5))
		c->macro_service |= source;
	else
		c->macro_service &= ~source;
	if (BIT(d, 4))
		c->bankswitch_irq |= source;
	else
		c->bankswitch_irq &= ~source;
}

static void idb_pages(nec_t *c);

static uint8_t sfr_r8(nec_t *c, uint32_t o)
{
	if (o < 0x100)
		return ((const uint8_t *)c->iram)[o];
	switch (o) {
	case 0x100: return c->port_in ? c->port_in(0) : 0xff;
	case 0x108:   // (P1 is combined with the interrupt lines)
		return (uint8_t)(((c->port_in ? c->port_in(1) : 0xff) & 0xf0) | (c->nmi_state ? 0 : 1) |
		                 (c->intp_state[0] ? 0 : 2) | (c->intp_state[1] ? 0 : 4) | (c->intp_state[2] ? 0 : 8));
	case 0x110: return c->port_in ? c->port_in(2) : 0xff;
	case 0x138: return c->port_in ? c->port_in(3) : 0xff;
	case 0x140: return c->intm;
	case 0x144: case 0x145: case 0x146: return c->ems[o - 0x144];
	case 0x14c: return read_irqcontrol(c, INTP0, c->priority_intp);
	case 0x14d: return read_irqcontrol(c, INTP1, 7);
	case 0x14e: return read_irqcontrol(c, INTP2, 7);
	case 0x165: return c->srms[0];
	case 0x166: return c->stms[0];
	case 0x168: return c->scm[0];
	case 0x169: return c->scc[0];
	case 0x16a: return c->brg[0];
	case 0x16b: return c->sce[0];
	case 0x16c: return read_irqcontrol(c, INTSER0, c->priority_ints0);
	case 0x16d: return read_irqcontrol(c, INTSR0, c->priority_ints0);
	case 0x16e: return read_irqcontrol(c, INTST0, c->priority_ints0);
	case 0x175: return c->srms[1];
	case 0x176: return c->stms[1];
	case 0x178: return c->scm[1];
	case 0x179: return c->scc[1];
	case 0x17a: return c->brg[1];
	case 0x17b: return c->sce[1];
	case 0x17c: return read_irqcontrol(c, INTSER1, c->priority_ints1);
	case 0x17d: return read_irqcontrol(c, INTSR1, c->priority_ints1);
	case 0x17e: return read_irqcontrol(c, INTST1, c->priority_ints1);
	case 0x180: return (uint8_t)c->TM0;
	case 0x181: return (uint8_t)(c->TM0 >> 8);
	case 0x182: return (uint8_t)c->MD0;
	case 0x183: return (uint8_t)(c->MD0 >> 8);
	case 0x188: return (uint8_t)c->TM1;
	case 0x189: return (uint8_t)(c->TM1 >> 8);
	case 0x18a: return (uint8_t)c->MD1;
	case 0x18b: return (uint8_t)(c->MD1 >> 8);
	case 0x194: case 0x195: case 0x196: return c->tmms[o - 0x194];
	case 0x19c: return read_irqcontrol(c, INTTU0, c->priority_inttu);
	case 0x19d: return read_irqcontrol(c, INTTU1, 7);
	case 0x19e: return read_irqcontrol(c, INTTU2, 7);
	case 0x1a0: return c->dmac[0];
	case 0x1a1: return c->dmam[0];
	case 0x1a2: return c->dmac[1];
	case 0x1a3: return c->dmam[1];
	case 0x1ac: return read_irqcontrol(c, INTD0, c->priority_intd);
	case 0x1ad: return read_irqcontrol(c, INTD1, 7);
	case 0x1e1: return c->RFM;
	case 0x1e8: return (uint8_t)c->WTC;
	case 0x1e9: return (uint8_t)(c->WTC >> 8);
	case 0x1ea: return (uint8_t)(c->F0 << 3 | c->F1 << 5);
	case 0x1eb: {
		uint8_t ret = c->RAMEN ? 0x40 : 0;
		ret |= c->TB == 13 ? 0x04 : c->TB == 16 ? 0x08 : c->TB == 20 ? 0x0c : 0;
		ret |= c->PCK == 4 ? 0x01 : c->PCK == 8 ? 0x02 : 0;
		return ret;
	}
	case 0x1ec: return read_irqcontrol(c, INTTB, 7);
	case 0x1ef: return c->IRQS;
	case 0x1fc: return c->ISPR;
	case 0x1ff: return (uint8_t)(c->IDB >> 12);
	}
	return 0;
}

static void prc_w(nec_t *c, uint8_t d)
{
	static const uint8_t timebases[4] = { 10, 13, 16, 20 };
	static const uint8_t clocks[4] = { 2, 4, 8, 0 };
	c->RAMEN = (d & 0x40) == 0x40;
	c->TB = timebases[(d & 0x0c) >> 2];
	c->PCK = clocks[d & 0x03];
	if (c->PCK == 0)
		c->PCK = 8;
	const int64_t tmp = (int64_t)c->PCK << c->TB;
	tm_adjust(c, 3, tmp, INTTB, tmp);
}

static void sfr_w8(nec_t *c, uint32_t o, uint8_t d)
{
	if (o < 0x100) {
		((uint8_t *)c->iram)[o] = d;
		return;
	}
	const uint16_t d16 = (o & 1) ? (uint16_t)(d << 8) : d;   // (a byte on a 16-bit handler)
	switch (o) {
	case 0x100: if (c->port_out) c->port_out(0, d); break;
	case 0x108: if (c->port_out) c->port_out(1, d & 0xf0); break;
	case 0x110: if (c->port_out) c->port_out(2, d); break;
	case 0x140: c->intm = d & 0x55; break;
	case 0x144: case 0x145: case 0x146: c->ems[o - 0x144] = d & 0xf7; break;
	case 0x14c: write_irqcontrol(c, INTP0, d); c->priority_intp = d & 7; break;
	case 0x14d: write_irqcontrol(c, INTP1, d); break;
	case 0x14e: write_irqcontrol(c, INTP2, d); break;
	case 0x165: c->srms[0] = d & 0xf7; break;
	case 0x166: c->stms[0] = d & 0xf7; break;
	case 0x168: c->scm[0] = d; break;
	case 0x169: c->scc[0] = d & 0x0f; break;
	case 0x16a: c->brg[0] = d; break;
	case 0x16c: write_irqcontrol(c, INTSER0, d & 0xd0); c->priority_ints0 = d & 7; break;
	case 0x16d: write_irqcontrol(c, INTSR0, d); c->priority_ints0 = d & 7; break;
	case 0x16e: write_irqcontrol(c, INTST0, d); c->priority_ints0 = d & 7; break;
	case 0x175: c->srms[1] = d & 0xf7; break;
	case 0x176: c->stms[1] = d & 0xf7; break;
	case 0x178: c->scm[1] = d; break;
	case 0x179: c->scc[1] = d & 0x0f; break;
	case 0x17a: c->brg[1] = d; break;
	case 0x17c: write_irqcontrol(c, INTSER1, d & 0xd0); c->priority_ints1 = d & 7; break;
	case 0x17d: write_irqcontrol(c, INTSR1, d); c->priority_ints1 = d & 7; break;
	case 0x17e: write_irqcontrol(c, INTST1, d); c->priority_ints1 = d & 7; break;
	case 0x180: case 0x181: c->TM0 = d16; break;
	case 0x182: case 0x183: c->MD0 = d16; break;
	case 0x188: case 0x189: c->TM1 = d16; break;
	case 0x18a: case 0x18b:
		if (c->MD1 == 0 && d16 != 0) {
			c->MD1 = d16;
			tmc1_w(c, c->TMC1);   // (MAME's HACK: start timer if necessary)
		} else
			c->MD1 = d16;
		break;
	case 0x190: tmc0_w(c, d); break;
	case 0x191: tmc1_w(c, d); break;
	case 0x194: case 0x195: case 0x196: c->tmms[o - 0x194] = d & 0xf7; break;
	case 0x19c: write_irqcontrol(c, INTTU0, d); c->priority_inttu = d & 7; break;
	case 0x19d: write_irqcontrol(c, INTTU1, d); break;
	case 0x19e: write_irqcontrol(c, INTTU2, d); break;
	case 0x1a0: c->dmac[0] = d & 0x33; break;
	case 0x1a1: c->dmam[0] = d & 0xfc; break;
	case 0x1a2: c->dmac[1] = d & 0x33; break;
	case 0x1a3: c->dmam[1] = d & 0xfc; break;
	case 0x1ac: write_irqcontrol(c, INTD0, d); c->priority_intd = d & 7; break;
	case 0x1ad: write_irqcontrol(c, INTD1, d); break;
	case 0x1e1: c->RFM = d; break;
	case 0x1e8: c->WTC = (uint16_t)((c->WTC & 0xff00) | d); break;
	case 0x1e9: c->WTC = (uint16_t)((c->WTC & 0x00ff) | d << 8); break;
	case 0x1ea: c->F0 = BIT(d, 3); c->F1 = BIT(d, 5); break;
	case 0x1eb: prc_w(c, d); break;
	case 0x1ec: write_irqcontrol(c, INTTB, d & 0xc0); break;
	case 0x1ff:
		c->IDB = (uint32_t)d << 12 | 0xe00;
		idb_pages(c);
		break;
	}
}

// the pages of the internal area and of 0xfffff go through these
static void idb_pages(nec_t *c)
{
	for (int i = 0; i < 2; i++)
		if (c->idb_page[i] != 0xff || i == 0) {
			c->rmap[c->idb_page[i]] = c->idb_r[i];
			c->wmap[c->idb_page[i]] = c->idb_w[i];
		}
	const uint8_t pg[2] = { 0xff, (uint8_t)(c->IDB >> 12) };
	for (int i = 0; i < 2; i++) {
		c->idb_page[i] = pg[i];
		c->idb_r[i] = c->rmap[pg[i]];
		c->idb_w[i] = c->wmap[pg[i]];
		c->rmap[pg[i]] = NULL;
		c->wmap[pg[i]] = NULL;
	}
}

static inline int idb_hit(const nec_t *c, uint32_t a)
{
	return (a & 0xffe00) == c->IDB && (c->RAMEN || BIT(a, 8));
}

static const uint8_t *hidden_r(const nec_t *c, uint32_t a)
{
	const uint32_t pg = a >> 12;
	for (int i = 1; i >= 0; i--)
		if (c->idb_page[i] == pg)
			return c->idb_r[i];
	return NULL;
}

static uint8_t *hidden_w(const nec_t *c, uint32_t a)
{
	const uint32_t pg = a >> 12;
	for (int i = 1; i >= 0; i--)
		if (c->idb_page[i] == pg)
			return c->idb_w[i];
	return NULL;
}

static uint8_t prog_r8(nec_t *c, uint32_t a)
{
	const uint8_t *p = hidden_r(c, a);
	return p ? p[a & 0xfff] : c->read8(a);
}

static uint16_t prog_r16(nec_t *c, uint32_t a)
{
	const uint8_t *p = hidden_r(c, a);
	return p ? *(const u16a *)(p + (a & 0xfff)) : c->read16(a);
}

static void prog_w8(nec_t *c, uint32_t a, uint8_t v)
{
	uint8_t *p = hidden_w(c, a);
	if (p)
		p[a & 0xfff] = v;
	else
		c->write8(a, v);
}

static void prog_w16(nec_t *c, uint32_t a, uint16_t v)
{
	uint8_t *p = hidden_w(c, a);
	if (p)
		*(u16a *)(p + (a & 0xfff)) = v;
	else
		c->write16(a, v);
}

static uint8_t v35_r8(nec_t *c, uint32_t a)
{
	if (idb_hit(c, a) || a == 0xfffff)
		return sfr_r8(c, a & 0x1ff);
	return prog_r8(c, a);
}

static uint16_t v35_r16(nec_t *c, uint32_t a)
{
	if (a == 0xffffe)   // (not sure about this: the manual says FFFFC-FFFFE are "reserved")
		return (uint16_t)(prog_r8(c, a) | sfr_r8(c, 0x1ff) << 8);
	if (idb_hit(c, a)) {
		const uint32_t o = a & 0x1ff;
		return (uint16_t)(sfr_r8(c, o) | sfr_r8(c, o + 1) << 8);
	}
	return prog_r16(c, a);
}

static void v35_w8(nec_t *c, uint32_t a, uint8_t v)
{
	if (idb_hit(c, a) || a == 0xfffff)
		sfr_w8(c, a & 0x1ff, v);
	else
		prog_w8(c, a, v);
}

static void v35_w16(nec_t *c, uint32_t a, uint16_t v)
{
	if (a == 0xffffe) {
		prog_w8(c, a, (uint8_t)v);
		sfr_w8(c, 0x1ff, (uint8_t)(v >> 8));
	} else if (idb_hit(c, a)) {
		const uint32_t o = a & 0x1ff;
		if (o >= 0x180 && o < 0x18c) {   // (the 16-bit timer registers take the word)
			switch (o & 0x1fe) {
			case 0x180: c->TM0 = v; return;
			case 0x182: c->MD0 = v; return;
			case 0x188: c->TM1 = v; return;
			case 0x18a:
				if (c->MD1 == 0 && v != 0) {
					c->MD1 = v;
					tmc1_w(c, c->TMC1);
				} else
					c->MD1 = v;
				return;
			}
		}
		sfr_w8(c, o, (uint8_t)v);
		sfr_w8(c, o + 1, (uint8_t)(v >> 8));
	} else
		prog_w16(c, a, v);
}

static inline void v35_setrb(nec_t *c, unsigned x)
{
	c->rbw = (uint16_t)(x << 4);
	c->rp = c->iram + c->rbw;
}
#define V35_SETRB v35_setrb


// ---- the polling loops (nec_idle_enable) ----
static inline unsigned idle_slot(uint32_t t) { return (t ^ (t >> 6) ^ (t >> 12)) & 63; }

// a taken backward branch to ip (V30 / V33): the same target a third time in a row ends the slice for a probe
static void nec_idle_see(nec_t *S, uint32_t ip)
{
	const uint32_t t = (((uint32_t)S->iram[0x70 + PS] << 4) + ip) & 0xfffff;
	if (t != S->idle_tgt) {
		S->idle_tgt = t;
		S->idle_cnt = 0;
		return;
	}
	if (++S->idle_cnt < 2)
		return;
	S->idle_cnt = 0;
	const unsigned sl = idle_slot(t);
	if (S->idle_bad[sl] == t && S->idle_badn[sl] >= 4)   // (a loop that failed its probes: left alone)
		return;
	S->stop |= 2;
}

// ---- the three cores ----
#define NEC_V_ADDR(a) ((a) & 0xfffff)
static inline uint32_t v33_addr(const nec_t *c, uint32_t a)
{
	if (unlikely(c->xa))
		return ((uint32_t)c->transtable[(a >> 14) & 63] << 14 | (a & 0x3fff)) & 0xfffff;
	return a & 0xfffff;
}

// V30
#define NEC_CORE_V33 0
#define NEC_CORE_V35 0
#define PFX(n) v30_##n
#define ADDR(S, a) NEC_V_ADDR(a)
#define SLOW_R8(S, a) (S)->read8(a)
#define SLOW_R16(S, a) (S)->read16(a)
#define SLOW_W8(S, a, v) (S)->write8((a), (v))
#define SLOW_W16(S, a, v) (S)->write16((a), (v))
#define PENDING(S) ((S)->pending_irq)
#define XA_ON(S) 0
static inline uint8_t v30_rd_op(nec_t *S, uint32_t a)
{
	a &= 0xfffff;
	const uint8_t *p = S->omap[a >> 12];
	if (!p)
		p = S->rmap[a >> 12];
	return p ? p[a & 0xfff] : S->read8(a);
}
#include "nec_core.inc"
#undef NEC_CORE_V33
#undef NEC_CORE_V35
#undef PFX
#undef ADDR
#undef XA_ON

// V33
#define NEC_CORE_V33 1
#define NEC_CORE_V35 0
#define PFX(n) v33_##n
#define ADDR(S, a) v33_addr(S, a)
#define XA_ON(S) ((S)->xa)
static inline uint8_t v33_rd_op(nec_t *S, uint32_t a)
{
	a = v33_addr(S, a);
	const uint8_t *p = S->omap[a >> 12];
	if (!p)
		p = S->rmap[a >> 12];
	return p ? p[a & 0xfff] : S->read8(a);
}
#include "nec_core.inc"
#undef NEC_CORE_V33
#undef NEC_CORE_V35
#undef PFX
#undef ADDR
#undef XA_ON
#undef SLOW_R8
#undef SLOW_R16
#undef SLOW_W8
#undef SLOW_W16
#undef PENDING

// V35
#define NEC_CORE_V33 0
#define NEC_CORE_V35 1
#define PFX(n) v35_##n
#define ADDR(S, a) NEC_V_ADDR(a)
#define SLOW_R8(S, a) v35_r8((S), (a))
#define SLOW_R16(S, a) v35_r16((S), (a))
#define SLOW_W8(S, a, v) v35_w8((S), (a), (v))
#define SLOW_W16(S, a, v) v35_w16((S), (a), (v))
#define PENDING(S) ((S)->pending_irq & (S)->unmasked_irq)
#define XA_ON(S) 0
static inline uint8_t v35_rd_op(nec_t *S, uint32_t a)
{
	// (fetches see the program space only, never the internal area)
	a &= 0xfffff;
	const uint8_t *p = S->omap[a >> 12];
	if (!p)
		p = hidden_r(S, a);
	return p ? p[a & 0xfff] : S->read8(a);
}
static inline void V35_WAKE(nec_t *c)
{
	const uint32_t pending = c->pending_irq & c->unmasked_irq;
	if (c->halted && pending) {
		for (int i = 0; i < 8; i++) {
			if (c->ISPR & (1 << i))
				break;
			if (c->priority_inttu == i && (pending & (INTTU0 | INTTU1 | INTTU2)))
				c->halted = 0;
			if (c->priority_intd == i && (pending & (INTD0 | INTD1)))
				c->halted = 0;
			if (c->priority_intp == i && (pending & (INTP0 | INTP1 | INTP2)))
				c->halted = 0;
			if (c->priority_ints0 == i && (pending & (INTSER0 | INTSR0 | INTST0)))
				c->halted = 0;
			if (c->priority_ints1 == i && (pending & (INTSER1 | INTSR1 | INTST1)))
				c->halted = 0;
			if (i == 7 && (pending & INTTB))
				c->halted = 0;
		}
	}
}
#include "nec_core.inc"

// ---- the polling loops' probe (nec_idle_enable) ----
static nec_t *s_idle_c;              // (the CPU probing)
static int s_idle_fail;              // (a write through a handler, an impure read, an I/O port)
static uint8_t *s_idle_wsaved[256];  // (the direct write pages, taken out of wmap while probing)
static uint32_t s_idle_wa[64];       // (the bytes written: their addresses and first values)
static uint8_t s_idle_wv[64];
static int s_idle_wn;
static uint8_t (*s_idle_r8)(uint32_t a), (*s_idle_in8)(uint32_t p);
static uint16_t (*s_idle_r16)(uint32_t a), (*s_idle_in16)(uint32_t p);
static void (*s_idle_w8)(uint32_t a, uint8_t v), (*s_idle_out8)(uint32_t p, uint8_t v);
static void (*s_idle_w16)(uint32_t a, uint16_t v), (*s_idle_out16)(uint32_t p, uint16_t v);

static void idle_log(uint32_t a)
{
	for (int i = 0; i < s_idle_wn; i++)
		if (s_idle_wa[i] == a)
			return;
	if (s_idle_wn == 64) {
		s_idle_fail = 1;
		return;
	}
	s_idle_wa[s_idle_wn] = a;
	s_idle_wv[s_idle_wn++] = s_idle_wsaved[a >> 12][a & 0xfff];
}
static void idle_w8(uint32_t a, uint8_t v)
{
	uint8_t *p = s_idle_wsaved[(a >> 12) & 0xff];
	if (p) {
		idle_log(a & 0xfffff);
		p[a & 0xfff] = v;
	} else {
		s_idle_fail = 1;
		s_idle_w8(a, v);
	}
}
static void idle_w16(uint32_t a, uint16_t v)   // (even addresses)
{
	uint8_t *p = s_idle_wsaved[(a >> 12) & 0xff];
	if (p) {
		idle_log(a & 0xfffff);
		idle_log((a + 1) & 0xfffff);
		p[a & 0xfff] = (uint8_t)v;
		p[(a & 0xfff) + 1] = (uint8_t)(v >> 8);
	} else {
		s_idle_fail = 1;
		s_idle_w16(a, v);
	}
}
static uint8_t idle_r8(uint32_t a)
{
	if (!s_idle_c->idle_pure || !s_idle_c->idle_pure(a))
		s_idle_fail = 1;
	return s_idle_r8(a);
}
static uint16_t idle_r16(uint32_t a)
{
	if (!s_idle_c->idle_pure || !s_idle_c->idle_pure(a) || !s_idle_c->idle_pure(a + 1))
		s_idle_fail = 1;
	return s_idle_r16(a);
}
static uint8_t idle_in8(uint32_t p) { s_idle_fail = 1; return s_idle_in8(p); }
static uint16_t idle_in16(uint32_t p) { s_idle_fail = 1; return s_idle_in16(p); }
static void idle_out8(uint32_t p, uint8_t v) { s_idle_fail = 1; s_idle_out8(p, v); }
static void idle_out16(uint32_t p, uint16_t v) { s_idle_fail = 1; s_idle_out16(p, v); }

// 1 = the instruction at the PC has a REP prefix (or its bytes are not on a direct page)
static int idle_rep(const nec_t *c)
{
	for (int i = 0; i < 8; i++) {
		const uint32_t a = (((uint32_t)c->iram[0x70 + PS] << 4) + ((c->ip + (uint32_t)i) & 0xffff)) & 0xfffff;
		const uint8_t *p = c->omap[a >> 12];
		if (!p)
			return 1;
		switch (p[a & 0xfff]) {
		case 0xf2: case 0xf3: case 0x64: case 0x65:
			return 1;
		case 0x26: case 0x2e: case 0x36: case 0x3e: case 0xf0:
			break;
		default:
			return 0;
		}
	}
	return 1;
}

// the registers and the core's state compared, from bank 7 of iram up to the instruction's counters
#define IDLE_CMP0 offsetof(nec_t, iram[112])
#define IDLE_CMPN (offsetof(nec_t, ic) - IDLE_CMP0)

// at the slice's end the core made for a polling loop's target (the PC); end: the run's. 1 = the run was aborted in
// an instruction run here (it returns now)
static int nec_idle_probe(nec_t *c, int64_t end)
{
	if (c->pending_irq || c->rep_params || c->halted || c->xa || end - c->total < 64)
		return 0;
	const uint32_t pc = (((uint32_t)c->iram[0x70 + PS] << 4) + c->ip) & 0xfffff;
	uint8_t snap[IDLE_CMPN];
	memcpy(snap, (const uint8_t *)c + IDLE_CMP0, IDLE_CMPN);
	const int64_t t0 = c->total;
	s_idle_c = c;
	s_idle_fail = 0;
	s_idle_wn = 0;
	s_idle_r8 = c->read8;
	s_idle_r16 = c->read16;
	s_idle_w8 = c->write8;
	s_idle_w16 = c->write16;
	s_idle_in8 = c->in8;
	s_idle_in16 = c->in16;
	s_idle_out8 = c->out8;
	s_idle_out16 = c->out16;
	c->read8 = idle_r8;
	c->read16 = idle_r16;
	c->write8 = idle_w8;
	c->write16 = idle_w16;
	c->in8 = idle_in8;
	c->in16 = idle_in16;
	c->out8 = idle_out8;
	c->out16 = idle_out16;
	for (int p = 0; p < 256; p++) {
		s_idle_wsaved[p] = c->wmap[p];
		c->wmap[p] = NULL;
	}
	c->idle_on = 0;
	int64_t T = 0;
	int aborted = 0, tried = 0, k;
	for (k = 0; k < 256; k++) {
		if (idle_rep(c))
			break;
		c->slice = 1;
		const int used = c->chip == NEC_V33 ? v33_run(c, 1) : v30_run(c, 1);
		c->total += used;
		c->slice = 0;
		c->ic = 0;
		if (c->stop) {   // (a handler's abort: stop & 1)
			aborted = 1;
			break;
		}
		if (s_idle_fail || c->pending_irq || c->rep_params || c->halted || c->total >= end)
			break;
		if (((((uint32_t)c->iram[0x70 + PS] << 4) + c->ip) & 0xfffff) == pc) {
			tried = 1;
			// (dead at an instruction's start without a REP pending: the EA temporaries, the prefix base, the REP's IP,
			// the last instruction's counters)
#define IDLE_SET(f) memcpy(snap + offsetof(nec_t, f) - IDLE_CMP0, &c->f, sizeof(c->f))
			IDLE_SET(prev_ip);
			IDLE_SET(rep_ip);
			IDLE_SET(EO);
			IDLE_SET(E16);
			IDLE_SET(prefix_base);
			IDLE_SET(EA);
			IDLE_SET(cur_cycles);
			IDLE_SET(popa_tmp);
#undef IDLE_SET
			int same = !memcmp(snap, (const uint8_t *)c + IDLE_CMP0, IDLE_CMPN);
			for (int i = 0; i < s_idle_wn && same; i++)
				same = s_idle_wsaved[s_idle_wa[i] >> 12][s_idle_wa[i] & 0xfff] == s_idle_wv[i];
			if (same)
				T = c->total - t0;
			break;
		}
	}
	c->read8 = s_idle_r8;
	c->read16 = s_idle_r16;
	c->write8 = s_idle_w8;
	c->write16 = s_idle_w16;
	c->in8 = s_idle_in8;
	c->in16 = s_idle_in16;
	c->out8 = s_idle_out8;
	c->out16 = s_idle_out16;
	for (int p = 0; p < 256; p++)
		c->wmap[p] = s_idle_wsaved[p];
	c->idle_on = 1;
	s_idle_c = NULL;
	if (aborted)
		return 1;
	const unsigned sl = idle_slot(pc);
	if (T > 0) {
		const int64_t left = end - c->total;
		if (left > 2 * T)
			c->total += (left / T - 1) * T;
		if (c->idle_bad[sl] == pc)
			c->idle_badn[sl] = 0;
	} else if (tried || s_idle_fail || k == 256) {
		if (c->idle_bad[sl] != pc) {
			c->idle_bad[sl] = pc;
			c->idle_badn[sl] = 0;
		}
		if (c->idle_badn[sl] < 255)
			c->idle_badn[sl]++;
	}
	return 0;
}

void nec_idle_enable(nec_t *c, int (*pure)(uint32_t a))
{
	if (c->chip == NEC_V35)
		return;
	c->idle_on = 1;
	c->idle_pure = pure;
	c->idle_tgt = 0xffffffffu;
	c->idle_cnt = 0;
	memset(c->idle_bad, 0xff, sizeof(c->idle_bad));
	memset(c->idle_badn, 0, sizeof(c->idle_badn));
}

// ---- the outside ----
static uint8_t def_r8(uint32_t a) { (void)a; return 0; }
static uint16_t def_r16(uint32_t a) { (void)a; return 0; }
static void def_w8(uint32_t a, uint8_t v) { (void)a; (void)v; }
static void def_w16(uint32_t a, uint16_t v) { (void)a; (void)v; }

void nec_init(nec_t *c, int chip, const uint8_t *dec)
{
	memset(c, 0, sizeof(*c));
	if (!nec_parity[0])
		for (int i = 0; i < 256; i++) {
			int n = 0;
			for (int j = i; j > 0; j >>= 1)
				n += j & 1;
			nec_parity[i] = !(n & 1);
		}
	c->chip = chip;
	c->dec = dec;
	c->rp = c->iram + 0x70;
	c->rbw = 0x70;
	c->read8 = c->in8 = def_r8;
	c->read16 = c->in16 = def_r16;
	c->write8 = c->out8 = def_w8;
	c->write16 = c->out16 = def_w16;
	c->PCK = 8;
	c->IDB = 0xffe00;
}

void nec_map(nec_t *c, uint32_t start, uint32_t end, const uint8_t *r, uint8_t *w)
{
	for (uint32_t p = start >> 12; p < (end + 0xfff) >> 12 && p < 256; p++, r = r ? r + 0x1000 : NULL, w = w ? w + 0x1000 : NULL) {
		c->omap[p] = r;
		c->rmap[p] = r;
		c->wmap[p] = w;
	}
}

void nec_reset(nec_t *c)
{
	if (c->chip != NEC_V35)
		memset(c->iram + 0x70, 0, 32);
	else if (!c->idb_page[0] && !c->idb_page[1]) {   // (first time: hide the pages of the internal area)
		const uint8_t pg = (uint8_t)(c->IDB >> 12);
		c->idb_page[0] = 0xff;
		c->idb_page[1] = pg;
		c->idb_r[0] = c->rmap[0xff];
		c->idb_w[0] = c->wmap[0xff];
		c->idb_r[1] = c->rmap[pg];
		c->idb_w[1] = c->wmap[pg];
		c->rmap[0xff] = NULL;
		c->wmap[0xff] = NULL;
	}
	c->ip = c->prev_ip = c->rep_ip = 0;
	c->TF = c->IF = c->DF = 0;
	c->MF = 1;
	c->SignVal = 0;
	c->AuxVal = c->OverVal = 0;
	c->ZeroVal = 1;
	c->CarryVal = 0;
	c->ParityVal = 1;
	c->pending_irq = 0;
	c->nmi_state = c->irq_state = 0;
	c->poll_state = 1;
	c->halted = 0;
	c->rep_params = 0;
	c->no_interrupt = 0;
	c->prefetch_count = 0;
	c->prefetch_reset = 0;
	c->seg_prefix = 0;
	c->xa = 0;
	if (c->chip == NEC_V35) {
		c->IBRK = 1;
		c->F0 = c->F1 = 0;
		c->unmasked_irq = INT_IRQ | NMI_IRQ;
		c->macro_service = 0;
		c->bankswitch_irq = 0;
		c->priority_inttu = c->priority_intd = c->priority_intp = c->priority_ints0 = c->priority_ints1 = 7;
		c->IRQS = c->ISPR = 0;
		c->mode_state = c->MF = c->dec ? 0 : 1;
		c->intm = 0;
		c->TMC0 = c->TMC1 = 0;
		memset(c->scm, 0, sizeof(c->scm));
		memset(c->scc, 0, sizeof(c->scc));
		memset(c->brg, 0, sizeof(c->brg));
		memset(c->sce, 0, sizeof(c->sce));
		memset(c->dmam, 0, sizeof(c->dmam));
		c->RAMEN = 1;
		c->TB = 20;
		c->PCK = 8;
		c->RFM = 0xfc;
		c->WTC = 0xffff;
		if (c->IDB != 0xffe00) {
			c->IDB = 0xffe00;
			idb_pages(c);
		}
		for (int i = 0; i < 3; i++)
			c->tm_next[i] = 0;
		const int64_t tmp = (int64_t)c->PCK << c->TB;
		tm_adjust(c, 3, tmp, INTTB, tmp);
		v35_setrb(c, 7);
	}
	c->rp[PS] = 0xffff;
	c->rp[SS] = 0;
	c->rp[DS0] = 0;
	c->rp[DS1] = 0;
	c->prefetch_reset = 1;
}

void nec_run_until(nec_t *c, int64_t end)
{
	if (c->chip != NEC_V35) {
		while (c->total < end) {
			int64_t n64 = end - c->total;
			const int n = n64 > 0x40000000 ? 0x40000000 : (int)n64;
			c->slice = n;
			c->total += c->chip == NEC_V33 ? v33_run(c, n) : v30_run(c, n);
			c->slice = 0;
			c->ic = 0;
			if (c->stop) {   // (1: a handler's abort; 2: a polling loop to probe, nec_idle_probe)
				const int st = c->stop;
				c->stop = 0;
				if ((st & 1) || nec_idle_probe(c, end)) {
					c->stop = 0;
					break;
				}
			}
		}
		return;
	}
	while (c->total < end) {
		// up to the next timer (in clocks; the CPU counts clocks / PCK)
		int64_t next = end;
		for (int i = 0; i < 4; i++)
			if (c->tm_next[i] && c->tm_next[i] < next)
				next = c->tm_next[i];
		const int pck = c->PCK;
		const int n = (int)((next - c->total + pck - 1) / pck);
		c->slice = n;
		const int used = n > 0 ? v35_run(c, n) : 0;
		c->total += (int64_t)used * pck;
		c->slice = 0;
		c->ic = 0;
		if (used <= 0)
			c->total = next;
		for (int i = 0; i < 4; i++)
			while (c->tm_next[i] && c->tm_next[i] <= c->total) {
				c->pending_irq |= c->tm_param[i];
				if (c->tm_period[i])
					c->tm_next[i] += c->tm_period[i];
				else
					c->tm_next[i] = 0;
			}
	}
}

int64_t nec_now(const nec_t *c)
{
	const int64_t done = (int64_t)(c->slice - c->ic);
	return c->total + (c->slice ? done : 0) * (c->chip == NEC_V35 ? c->PCK : 1);
}

void nec_set_input(nec_t *c, int line, int state)
{
	switch (line) {
	case NEC_LINE_IRQ:
		c->irq_state = (uint32_t)state;
		if (!state)
			c->pending_irq &= ~(uint32_t)INT_IRQ;
		else {
			c->pending_irq |= INT_IRQ;
			c->halted = 0;
		}
		break;
	case NEC_LINE_NMI:
		if (c->nmi_state == (uint32_t)state)
			return;
		c->nmi_state = (uint32_t)state;
		if (state) {
			c->pending_irq |= NMI_IRQ;
			c->halted = 0;
		}
		break;
	case NEC_LINE_INTP0:
	case NEC_LINE_INTP1:
	case NEC_LINE_INTP2:
		line -= NEC_LINE_INTP0;
		if (c->intp_state[line] == (uint8_t)state)
			return;
		c->intp_state[line] = (uint8_t)state;
		if (state)
			c->pending_irq |= (uint32_t)INTP0 << line;
		break;
	case NEC_LINE_POLL:
		c->poll_state = (uint32_t)state;
		break;
	}
}

void nec_after_load(nec_t *c)
{
	c->rp = c->chip == NEC_V35 ? c->iram + c->rbw : c->iram + 0x70;
	if (c->chip == NEC_V35)
		idb_pages(c);
}
