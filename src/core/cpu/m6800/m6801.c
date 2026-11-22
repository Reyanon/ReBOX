// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/*  Motorola MC6801 / MC6801U4, ported to C99 from MAME src/devices/cpu/m6800/m6800.cpp, 6800ops.hxx and
    m6801.cpp (BSD-3-Clause, copyright-holders: Aaron Giles). The opcode handlers are MAME's 6800ops.hxx with
    the member accesses rewritten; the 6803 opcode / cycle tables are MAME's.
    HD6301V1 / HD63701V0 (hd6301_cpu_device): hd63701_insn / cycles_63701 tables and the TRAP vector.
    Not carried over: SCI data shifting (registers only), OS3 / SC2 output, standby and NVRAM, HD6301X/Y/HD6303.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "m6801.h"

#include <string.h>

#define pPPC    m->ppc
#define pPC     m->pc
#define pS      m->s
#define pX      m->x
#define pD      m->d

#define PC      m->pc.w.l
#define PCD     m->pc.d
#define S       m->s.w.l
#define SD      m->s.d
#define X       m->x.w.l
#define D       m->d.w.l
#define A       m->d.b.h
#define B       m->d.b.l
#define CC      m->cc

#define EAD     m->ea.d
#define EA      m->ea.w.l

#define CT      m->counter.w.l
#define CTH     m->counter.w.h
#define CTD     m->counter.d
#define OC      m->output_compare[0].w.l
#define OCH     m->output_compare[0].w.h
#define OCD     m->output_compare[0].d
#define OC2     m->output_compare[1].w.l
#define OC2H    m->output_compare[1].w.h
#define OC2D    m->output_compare[1].d
#define OC3     m->output_compare[2].w.l
#define OC3H    m->output_compare[2].w.h
#define OC3D    m->output_compare[2].d
#define TOH     m->timer_over.w.h
#define TOD     m->timer_over.d

#define M6800_WAI 8
#define M6800_SLP 0x10   // HD63701 only

#define IS_HD6301(m) ((m)->type == M6801_TYPE_HD6301V1 || (m)->type == M6801_TYPE_HD63701V0)

#define M6801_TRCSR_RDRF        0x80
#define M6801_TRCSR_ORFE        0x40
#define M6801_TRCSR_TDRE        0x20
#define M6801_TRCSR_RIE         0x10
#define M6801_TRCSR_RE          0x08
#define M6801_TRCSR_TIE         0x04
#define M6801_TRCSR_TE          0x02
#define M6801_TRCSR_WU          0x01

#define M6801_P3CSR_LE          0x08
#define M6801_P3CSR_OSS         0x10
#define M6801_P3CSR_IS3_ENABLE  0x40
#define M6801_P3CSR_IS3_FLAG    0x80

#define TCSR_OLVL   0x01
#define TCSR_IEDG   0x02
#define TCSR_ETOI   0x04
#define TCSR_EOCI   0x08
#define TCSR_EICI   0x10
#define TCSR_TOF    0x20
#define TCSR_OCF    0x40
#define TCSR_ICF    0x80

#define TSR_TOF     0x04
#define TSR_OCF1    0x08
#define TSR_OCF2    0x10
#define TSR_OCF3    0x20
#define TSR_ICF1    0x40
#define TSR_ICF2    0x80

static void execute_one(m6801 *m);
static void check_irq_lines(m6801 *m);
static void eat_cycles(m6801 *m);
static void increment_counter(m6801 *m, int amount);
static void check_irq2(m6801 *m);

// ---- on-chip I/O ----
static uint8_t port_in(m6801 *m, int n) { return m->port_r ? m->port_r(m->user, n + 1) : 0xff; }
static void port_out(m6801 *m, int n, uint8_t data, uint8_t ddr) { if (m->port_w) m->port_w(m->user, n + 1, data, ddr); }

static void write_port2(m6801 *m)
{
	if (!m->port2_written) return;
	uint8_t data = m->port_data[1];
	uint8_t ddr = m->port_ddr[1] & 0x1f;
	if ((ddr != 0x1f) && ddr)
		data = (uint8_t)((m->port_data[1] & ddr) | (ddr ^ 0xff));
	if (m->trcsr & M6801_TRCSR_TE) {
		data = (uint8_t)((data & 0xef) | (m->tx << 4));
		ddr |= 0x10;
	}
	data &= 0x1f;
	port_out(m, 1, data, ddr);
}

static uint8_t port_data_r(m6801 *m, int n)   // ports 1, 2, 4
{
	uint8_t ddr = m->port_ddr[n];
	if (ddr == 0xff)
		return m->port_data[n];
	return (uint8_t)((port_in(m, n) & ~ddr) | (m->port_data[n] & ddr));
}

static void port_simple_out(m6801 *m, int n)   // ports 1, 3, 4
{
	port_out(m, n, (uint8_t)((m->port_data[n] & m->port_ddr[n]) | (m->port_ddr[n] ^ 0xff)), m->port_ddr[n]);
}

static uint8_t p3_data_r(m6801 *m)
{
	if (m->pending_isf_clear) {
		m->p3csr &= ~M6801_P3CSR_IS3_FLAG;
		m->pending_isf_clear = 0;
	}
	uint8_t data, ddr = m->port_ddr[2];
	if ((m->p3csr & M6801_P3CSR_LE) || (ddr == 0xff))
		data = m->port_data[2];
	else
		data = (uint8_t)((port_in(m, 2) & ~ddr) | (m->port_data[2] & ddr));
	m->port3_latched = 0;
	return data;
}

static void p3_data_w(m6801 *m, uint8_t data)
{
	if (m->pending_isf_clear) {
		m->p3csr &= ~M6801_P3CSR_IS3_FLAG;
		m->pending_isf_clear = 0;
	}
	m->port_data[2] = data;
	port_simple_out(m, 2);
}

// ---- timer ----
static void set_timer_event(m6801 *m)
{
	m->timer_next = (OCD < TOD) ? OCD : TOD;
	if (m->type == M6801_TYPE_6801U4) {
		if (OC2D < m->timer_next) m->timer_next = OC2D;
		if (OC3D < m->timer_next) m->timer_next = OC3D;
	}
}

static void modified_counters(m6801 *m)
{
	OCH = (OC >= CT) ? CTH : CTH + 1;
	if (m->type == M6801_TYPE_6801U4) {
		OC2H = (OC2 >= CT) ? CTH : CTH + 1;
		OC3H = (OC3 >= CT) ? CTH : CTH + 1;
	}
	set_timer_event(m);
}

static void check_timer_event(m6801 *m)
{
	// OCI
	if (CTD >= OCD) {
		OCH++;
		m->tcsr |= TCSR_OCF;
		m->pending_tcsr |= TCSR_OCF;
		// output on P21 (not on the U4, as in MAME)
		if (m->type != M6801_TYPE_6801U4 && (m->port_ddr[1] & 2)) {
			m->port_data[1] &= ~2;
			m->port_data[1] |= (m->tcsr & TCSR_OLVL) << 1;
			m->port2_written = 1;
			write_port2(m);
		}
	}
	if (m->type == M6801_TYPE_6801U4) {
		if (CTD >= OC2D) {
			OC2H++;
			m->tsr |= TSR_OCF2;
			m->pending_tsr |= TSR_OCF2;
		}
		if (CTD >= OC3D) {
			OC3H++;
			m->tsr |= TSR_OCF3;
			m->pending_tsr |= TSR_OCF3;
		}
	}
	// TOI
	if (CTD >= TOD) {
		TOH++;
		m->tcsr |= TCSR_TOF;
		m->pending_tcsr |= TCSR_TOF;
	}
	check_irq2(m);
	set_timer_event(m);
}

static void increment_counter(m6801 *m, int amount)
{
	m->icount -= amount;
	if (m->type == M6801_TYPE_6800)
		return;   // no timer (m6800_cpu_device::increment_counter)
	CTD += (uint32_t)amount;
	if (CTD >= m->timer_next)
		check_timer_event(m);
}

static void eat_cycles(m6801 *m)
{
	if (m->type == M6801_TYPE_6800) {
		if (m->icount > 0)
			m->icount = 0;
		return;
	}
	int cycles_to_eat = (int)(m->timer_next - CTD);
	if (cycles_to_eat > m->icount) cycles_to_eat = m->icount;
	if (cycles_to_eat > 0)
		increment_counter(m, cycles_to_eat);
}

static void cleanup_counters(m6801 *m)
{
	if (m->type == M6801_TYPE_6800)
		return;
	if (m->type == M6801_TYPE_6801U4) {
		OC2H -= CTH;
		OC3H -= CTH;
	}
	OCH -= CTH;
	TOH -= CTH;
	CTH = 0;
	set_timer_event(m);
	if (CTD >= m->timer_next)
		check_timer_event(m);
}

// ---- register file ----
static int io_read(m6801 *m, uint16_t a, uint8_t *v)
{
	switch (a) {
	case 0x00: case 0x01: case 0x04: case 0x05: *v = 0xff; return 1;
	case 0x02: *v = port_data_r(m, 0); return 1;
	case 0x03: *v = port_data_r(m, 1); return 1;
	case 0x06: *v = p3_data_r(m); return 1;
	case 0x07: *v = port_data_r(m, 3); return 1;
	case 0x08: m->pending_tcsr = 0; *v = m->tcsr; return 1;
	case 0x09:
		if (!(m->pending_tcsr & TCSR_TOF)) m->tcsr &= ~TCSR_TOF;
		*v = m->counter.b.h; return 1;
	case 0x0a: *v = m->counter.b.l; return 1;
	case 0x0b: *v = m->output_compare[0].b.h; return 1;
	case 0x0c: *v = m->output_compare[0].b.l; return 1;
	case 0x0d:
		if (!(m->pending_tcsr & TCSR_ICF)) m->tcsr &= ~TCSR_ICF;
		*v = (uint8_t)(m->input_capture >> 8); return 1;
	case 0x0e: *v = (uint8_t)m->input_capture; return 1;
	case 0x0f:
		if (m->p3csr & M6801_P3CSR_IS3_FLAG) m->pending_isf_clear = 1;
		*v = m->p3csr; return 1;
	case 0x10: *v = m->rmcr; return 1;
	case 0x11:
		if (m->trcsr & M6801_TRCSR_TDRE) m->trcsr_read_tdre = 1;
		if (m->trcsr & M6801_TRCSR_ORFE) m->trcsr_read_orfe = 1;
		if (m->trcsr & M6801_TRCSR_RDRF) m->trcsr_read_rdrf = 1;
		*v = m->trcsr; return 1;
	case 0x12:
		if (m->trcsr_read_orfe) { m->trcsr_read_orfe = 0; m->trcsr &= ~M6801_TRCSR_ORFE; }
		if (m->trcsr_read_rdrf) { m->trcsr_read_rdrf = 0; m->trcsr &= ~M6801_TRCSR_RDRF; }
		*v = m->rdr; return 1;
	case 0x14: *v = m->ram_ctrl | 0x3f; return 1;
	}
	if (m->type != M6801_TYPE_6801U4)
		return 0;
	switch (a) {
	case 0x15: return io_read(m, 0x09, v);
	case 0x16: return io_read(m, 0x0a, v);
	case 0x18: *v = m->tcr[1] | 0x03; return 1;
	case 0x19:
		m->pending_tcsr &= ~(TCSR_ICF | TCSR_OCF | TCSR_TOF);
		m->pending_tsr = 0;
		*v = (uint8_t)(m->tsr | 0x03 | ((m->tcsr & TCSR_ICF) ? TSR_ICF1 : 0) | ((m->tcsr & TCSR_TOF) ? TSR_TOF : 0) |
				((m->tcsr & TCSR_OCF) ? TSR_OCF1 : 0));
		return 1;
	case 0x1a: *v = m->output_compare[1].b.h; return 1;
	case 0x1b: *v = m->output_compare[1].b.l; return 1;
	case 0x1c: *v = m->output_compare[2].b.h; return 1;
	case 0x1d: *v = m->output_compare[2].b.l; return 1;
	}
	return 0;
}

static void ocr_w(m6801 *m, int n, int high, uint8_t data)
{
	if (n == 0) {
		if (!(m->pending_tcsr & TCSR_OCF)) m->tcsr &= ~TCSR_OCF;
	} else {
		uint8_t f = (uint8_t)(TSR_OCF2 * n);
		if (!(m->pending_tsr & f)) m->tsr &= ~f;
	}
	uint8_t *r = high ? &m->output_compare[n].b.h : &m->output_compare[n].b.l;
	if (*r != data) {
		*r = data;
		modified_counters(m);
	}
}

static int io_write(m6801 *m, uint16_t a, uint8_t data)
{
	switch (a) {
	case 0x00:
		if (m->port_ddr[0] != data) { m->port_ddr[0] = data; port_simple_out(m, 0); }
		return 1;
	case 0x01:
		if (m->port_ddr[1] != data) { m->port_ddr[1] = data; write_port2(m); }
		return 1;
	case 0x02: m->port_data[0] = data; port_simple_out(m, 0); return 1;
	case 0x03: m->port_data[1] = data; m->port2_written = 1; write_port2(m); return 1;
	case 0x04:
		if (m->port_ddr[2] != data) { m->port_ddr[2] = data; port_simple_out(m, 2); }
		return 1;
	case 0x05:
		if (m->port_ddr[3] != data) { m->port_ddr[3] = data; port_simple_out(m, 3); }
		return 1;
	case 0x06: p3_data_w(m, data); return 1;
	case 0x07: m->port_data[3] = data; port_simple_out(m, 3); return 1;
	case 0x08:
		data &= 0x1f;
		m->tcsr = (uint8_t)(data | (m->tcsr & 0xe0));
		m->pending_tcsr &= m->tcsr;
		check_irq2(m);
		return 1;
	case 0x09:
		m->latch09 = data;
		CT = 0xfff8;
		TOH = CTH;
		modified_counters(m);
		return 1;
	case 0x0a:
		CT = (uint16_t)((m->latch09 << 8) | data);
		TOH = CTH;
		modified_counters(m);
		return 1;
	case 0x0b: ocr_w(m, 0, 1, data); return 1;
	case 0x0c: ocr_w(m, 0, 0, data); return 1;
	case 0x0f: m->p3csr = data; return 1;
	case 0x10: m->rmcr = data; return 1;
	case 0x11:
		if ((data & M6801_TRCSR_TE) && !(m->trcsr & M6801_TRCSR_TE))
			m->tx = 1;
		m->trcsr = (uint8_t)((m->trcsr & 0xe0) | (data & 0x1f));
		return 1;
	case 0x13:
		if (m->trcsr_read_tdre) { m->trcsr_read_tdre = 0; m->trcsr &= ~M6801_TRCSR_TDRE; }
		m->tdr = data;
		return 1;
	case 0x14: m->ram_ctrl = data; return 1;
	}
	if (m->type != M6801_TYPE_6801U4)
		return 0;
	switch (a) {
	case 0x15: return io_write(m, 0x09, data);
	case 0x16: return io_write(m, 0x0a, data);
	case 0x18: m->tcr[1] = data & 0xfc; check_irq2(m); return 1;
	case 0x1a: ocr_w(m, 1, 1, data); return 1;
	case 0x1b: ocr_w(m, 1, 0, data); return 1;
	case 0x1c: ocr_w(m, 2, 1, data); return 1;
	case 0x1d: ocr_w(m, 2, 0, data); return 1;
	}
	return 0;
}

// ---- memory ----
static inline int is_ram(const m6801 *m, uint16_t a)
{
	return a < 0x100 && m->type != M6801_TYPE_6800 && a >= (m->type == M6801_TYPE_6801U4 ? 0x40 : 0x80);
}

static uint8_t RM(m6801 *m, uint32_t addr)
{
	uint16_t a = (uint16_t)addr;
	if (a < 0x100) {
		if (is_ram(m, a))
			return m->ram[a];
		uint8_t v;
		if (a < 0x20 && m->type != M6801_TYPE_6800 && io_read(m, a, &v))
			return v;
	} else if (m->rom && (uint32_t)(a - m->rom_base) < m->rom_size) {
		return m->rom[a - m->rom_base];
	}
	return m->read ? m->read(m->user, a) : 0;
}

static void WM(m6801 *m, uint32_t addr, uint8_t data)
{
	uint16_t a = (uint16_t)addr;
	if (a < 0x100) {
		if (is_ram(m, a)) {
			m->ram[a] = data;
			return;
		}
		if (a < 0x20 && m->type != M6801_TYPE_6800 && io_write(m, a, data))
			return;
	}
	if (m->write)
		m->write(m->user, a, data);
}

#define RM(Addr) RM(m, Addr)
#define WM(Addr,Value) WM(m, Addr, Value)
// (opcode and argument fetches: the on-chip ROM first, the same result as RM)
static inline uint8_t rdop(m6801 *m, uint32_t addr)
{
	const uint32_t o = (uint16_t)addr - m->rom_base;
	if (m->rom && o < m->rom_size && (uint16_t)addr >= 0x100)
		return m->rom[o];
	return RM(addr);
}
#define M_RDOP(Addr) rdop(m, Addr)
#define M_RDOP_ARG(Addr) rdop(m, Addr)

static uint32_t rm16(m6801 *m, uint32_t Addr)
{
	uint32_t result = (uint32_t)RM(Addr) << 8;
	return result | RM((Addr + 1) & 0xffff);
}

static void wm16(m6801 *m, uint32_t Addr, m6801_pair *p)
{
	WM(Addr, p->b.h);
	WM((Addr + 1) & 0xffff, p->b.l);
}
#define RM16(Addr) rm16(m, Addr)

/* macros to access memory */
#define IMMBYTE(b)  b = M_RDOP_ARG(PCD); PC++
#define IMMWORD(w)  w.d = (M_RDOP_ARG(PCD)<<8) | M_RDOP_ARG((PCD+1)&0xffff); PC+=2

#define PUSHBYTE(b) WM(SD,b); --S
#define PUSHWORD(w) WM(SD,w.b.l); --S; WM(SD,w.b.h); --S
#define PULLBYTE(b) S++; b = RM(SD)
#define PULLWORD(w) S++; w.d = RM(SD)<<8; S++; w.d |= RM(SD)

/* CC masks                       HI NZVC
                                7654 3210   */
#define CLR_HNZVC   CC&=0xd0
#define CLR_NZV     CC&=0xf1
#define CLR_HNZC    CC&=0xd2
#define CLR_NZVC    CC&=0xf0
#define CLR_Z       CC&=0xfb
#define CLR_ZC      CC&=0xfa
#define CLR_C       CC&=0xfe

/* macros for CC -- CC bits affected should be reset before calling */
#define SET_Z(a)        if(!(a))SEZ
#define SET_Z8(a)       SET_Z((uint8_t)(a))
#define SET_Z16(a)      SET_Z((uint16_t)(a))
#define SET_N8(a)       CC|=(((a)&0x80)>>4)
#define SET_N16(a)      CC|=(((a)&0x8000)>>12)
#define SET_H(a,b,r)    CC|=((((a)^(b)^(r))&0x10)<<1)
#define SET_C8(a)       CC|=(((a)&0x100)>>8)
#define SET_C16(a)      CC|=(((a)&0x10000)>>16)
#define SET_V8(a,b,r)   CC|=((((a)^(b)^(r)^((r)>>1))&0x80)>>6)
#define SET_V16(a,b,r)  CC|=((((a)^(b)^(r)^((r)>>1))&0x8000)>>14)

static const uint8_t flags8i[256] = /* increment */
{
0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x0a,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08
};

static const uint8_t flags8d[256] = /* decrement */
{
0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08
};

#define SET_FLAGS8I(a)      {CC|=flags8i[(a)&0xff];}
#define SET_FLAGS8D(a)      {CC|=flags8d[(a)&0xff];}

/* combos */
#define SET_NZ8(a)          {SET_N8(a);SET_Z8(a);}
#define SET_NZ16(a)         {SET_N16(a);SET_Z16(a);}
#define SET_FLAGS8(a,b,r)   {SET_N8(r);SET_Z8(r);SET_V8(a,b,r);SET_C8(r);}
#define SET_FLAGS16(a,b,r)  {SET_N16(r);SET_Z16(r);SET_V16(a,b,r);SET_C16(r);}

/* for treating an u8 as a signed s16 */
#define SIGNED(b) ((int16_t)(b&0x80?b|0xff00:b))

/* Macros for addressing modes */
#define DIRECT IMMBYTE(EAD)
#define IMM8 EA=PC++
#define IMM16 {EA=PC;PC+=2;}
#define EXTENDED IMMWORD(m->ea)
#define INDEXED {EA=X+(uint8_t)M_RDOP_ARG(PCD);PC++;}

/* macros to set status flags */
#define SEC CC|=0x01
#define CLC CC&=0xfe
#define SEZ CC|=0x04
#define CLZ CC&=0xfb
#define SEN CC|=0x08
#define CLN CC&=0xf7
#define SEV CC|=0x02
#define CLV CC&=0xfd
#define SEH CC|=0x20
#define CLH CC&=0xdf
#define SEI CC|=0x10
#define CLI CC&=~0x10

/* macros for convenience */
#define DIRBYTE(b) {DIRECT;b=RM(EAD);}
#define DIRWORD(w) {DIRECT;w.d=RM16(EAD);}
#define EXTBYTE(b) {EXTENDED;b=RM(EAD);}
#define EXTWORD(w) {EXTENDED;w.d=RM16(EAD);}

#define IDXBYTE(b) {INDEXED;b=RM(EAD);}
#define IDXWORD(w) {INDEXED;w.d=RM16(EAD);}

/* Macros for branch instructions */
#define BRANCH(f) {IMMBYTE(t);if(f){PC+=SIGNED(t);}}
#define NXORV  ((CC&0x08)^((CC&0x02)<<2))
#define NXORC  ((CC&0x08)^((CC&0x01)<<3))

#define XX 4 // illegal opcode unknown cycle count

// ---- interrupts ----
static void enter_interrupt(m6801 *m, uint16_t irq_vector)
{
	int cycles_to_eat;
	if (m->wai_state & M6800_WAI) {
		cycles_to_eat = 4;
		m->wai_state &= ~M6800_WAI;
	} else {
		PUSHWORD(pPC);
		PUSHWORD(pX);
		PUSHBYTE(A);
		PUSHBYTE(B);
		PUSHBYTE(CC);
		cycles_to_eat = 12;
	}
	SEI;
	PCD = RM16(irq_vector);
	increment_counter(m, cycles_to_eat);
}

static void take_irq2(m6801 *m, uint16_t irq_vector)
{
	m->wai_state &= ~M6800_SLP;

	if (!(m->cc & 0x10))
		enter_interrupt(m, irq_vector);
}

static void check_irq2(m6801 *m)
{
	int u4 = m->type == M6801_TYPE_6801U4;
	if ((m->tcsr & (TCSR_EICI | TCSR_ICF)) == (TCSR_EICI | TCSR_ICF) || (u4 && (m->tcr[1] & m->tsr & TSR_ICF2)))
		take_irq2(m, 0xfff6);
	else if ((m->tcsr & (TCSR_EOCI | TCSR_OCF)) == (TCSR_EOCI | TCSR_OCF) || (u4 && (m->tcr[1] & m->tsr & (TSR_OCF2 | TSR_OCF3))))
		take_irq2(m, 0xfff4);
	else if ((m->tcsr & (TCSR_ETOI | TCSR_TOF)) == (TCSR_ETOI | TCSR_TOF))
		take_irq2(m, 0xfff2);
	else if (((m->trcsr & (M6801_TRCSR_RIE | M6801_TRCSR_RDRF)) == (M6801_TRCSR_RIE | M6801_TRCSR_RDRF)) ||
			((m->trcsr & (M6801_TRCSR_RIE | M6801_TRCSR_ORFE)) == (M6801_TRCSR_RIE | M6801_TRCSR_ORFE)) ||
			((m->trcsr & (M6801_TRCSR_TIE | M6801_TRCSR_TDRE)) == (M6801_TRCSR_TIE | M6801_TRCSR_TDRE)))
		take_irq2(m, 0xfff0);
}

static void check_irq_lines(m6801 *m)
{
	if (m->nmi_pending) {
		m->wai_state &= ~M6800_SLP;
		m->nmi_pending = 0;
		enter_interrupt(m, 0xfffc);
	} else if (m->irq_state[M6801_IRQ1_LINE]) {
		m->wai_state &= ~M6800_SLP;
		if (!(CC & 0x10)) {
			enter_interrupt(m, 0xfff8);
			if (m->irq_state[M6801_IRQ1_LINE] == M6801_HOLD_LINE)   // (MAME HOLD_LINE: cleared when taken)
				m->irq_state[M6801_IRQ1_LINE] = 0;
		}
	} else if (m->type != M6801_TYPE_6800) {
		check_irq2(m);
	}
}

// ---- opcodes (MAME 6800ops.hxx) ----
static void op_illegl1(m6801 *m)
{
	(void)m;
}

// 6800 only: illegal opcodes skip their operand bytes
static void op_illegl2(m6801 *m)
{
	PC++;
}

static void op_illegl3(m6801 *m)
{
	PC += 2;
}

static void op_nop(m6801 *m)
{
	(void)m;
}

static void op_lsrd(m6801 *m)
{
	uint16_t t;
	CLR_NZVC; t = D; CC|=(t&0x0001);
	t>>=1; SET_Z16(t);
	if (NXORC) SEV;
	D=t;
}

static void op_asld(m6801 *m)
{
	int r;
	uint16_t t;
	t = D; r=t<<1;
	CLR_NZVC; SET_FLAGS16(t,t,r);
	D=r;
}

static void op_tap(m6801 *m)
{
	CC=A;

	// TAP temporarily sets the I flag and blocks IRQ until the next opcode
	// (if the next opcode is TAP, IRQ is blocked again)
	execute_one(m);
	check_irq_lines(m);
}

static void op_tpa(m6801 *m)
{
	A=CC;
}

static void op_inx(m6801 *m)
{
	++X;
	CLR_Z; SET_Z16(X);
}

static void op_dex(m6801 *m)
{
	--X;
	CLR_Z; SET_Z16(X);
}

static void op_clv(m6801 *m)
{
	CLV;
}

static void op_sev(m6801 *m)
{
	SEV;
}

static void op_clc(m6801 *m)
{
	CLC;
}

static void op_sec(m6801 *m)
{
	SEC;
}

static void op_cli(m6801 *m)
{
	uint8_t i = CC & 0x10;
	CLI;

	// pending IRQ won't be triggered until next machine cycle
	if (i) execute_one(m);
	check_irq_lines(m);
}

static void op_sei(m6801 *m)
{
	SEI;
}

static void op_sba(m6801 *m)
{
	uint16_t t;
	t=A-B;
	CLR_NZVC; SET_FLAGS8(A,B,t);
	A=t;
}

static void op_cba(m6801 *m)
{
	uint16_t t;
	t=A-B;
	CLR_NZVC; SET_FLAGS8(A,B,t);
}

static void op_tab(m6801 *m)
{
	B=A;
	CLR_NZV; SET_NZ8(B);
}

static void op_tba(m6801 *m)
{
	A=B;
	CLR_NZV; SET_NZ8(A);
}

static void op_daa(m6801 *m)
{
	uint8_t msn, lsn;
	uint16_t t, cf = 0;
	msn=A & 0xf0; lsn=A & 0x0f;
	if (lsn>0x09 || CC&0x20) cf |= 0x06;
	if (msn>0x80 && lsn>0x09) cf |= 0x60;
	if (msn>0x90 || CC&0x01) cf |= 0x60;
	t = cf + A;
	CLR_NZV; // keep carry from previous operation
	SET_NZ8((uint8_t)t); SET_C8(t);
	A = t;
}

static void op_aba(m6801 *m)
{
	uint16_t t;
	t=A+B;
	CLR_HNZVC; SET_FLAGS8(A,B,t); SET_H(A,B,t);
	A=t;
}

static void op_bra(m6801 *m)
{
	uint8_t t;
	BRANCH(1);
}

static void op_brn(m6801 *m)
{
	uint8_t t;
	BRANCH(0);
}

static void op_bhi(m6801 *m)
{
	uint8_t t;
	BRANCH(!(CC&0x05));
}

static void op_bls(m6801 *m)
{
	uint8_t t;
	BRANCH(CC&0x05);
}

static void op_bcc(m6801 *m)
{
	uint8_t t;
	BRANCH(!(CC&0x01));
}

static void op_bcs(m6801 *m)
{
	uint8_t t;
	BRANCH(CC&0x01);
}

static void op_bne(m6801 *m)
{
	uint8_t t;
	BRANCH(!(CC&0x04));
}

static void op_beq(m6801 *m)
{
	uint8_t t;
	BRANCH(CC&0x04);
}

static void op_bvc(m6801 *m)
{
	uint8_t t;
	BRANCH(!(CC&0x02));
}

static void op_bvs(m6801 *m)
{
	uint8_t t;
	BRANCH(CC&0x02);
}

static void op_bpl(m6801 *m)
{
	uint8_t t;
	BRANCH(!(CC&0x08));
}

static void op_bmi(m6801 *m)
{
	uint8_t t;
	BRANCH(CC&0x08);
}

static void op_bge(m6801 *m)
{
	uint8_t t;
	BRANCH(!NXORV);
}

static void op_blt(m6801 *m)
{
	uint8_t t;
	BRANCH(NXORV);
}

static void op_bgt(m6801 *m)
{
	uint8_t t;
	BRANCH(!(NXORV||CC&0x04));
}

static void op_ble(m6801 *m)
{
	uint8_t t;
	BRANCH(NXORV||CC&0x04);
}

static void op_tsx(m6801 *m)
{
	X = ( S + 1 );
}

static void op_ins(m6801 *m)
{
	++S;
}

static void op_pula(m6801 *m)
{
	PULLBYTE(m->d.b.h);
}

static void op_pulb(m6801 *m)
{
	PULLBYTE(m->d.b.l);
}

static void op_des(m6801 *m)
{
	--S;
}

static void op_txs(m6801 *m)
{
	S = ( X - 1 );
}

static void op_psha(m6801 *m)
{
	PUSHBYTE(m->d.b.h);
}

static void op_pshb(m6801 *m)
{
	PUSHBYTE(m->d.b.l);
}

static void op_pulx(m6801 *m)
{
	PULLWORD(pX);
}

static void op_rts(m6801 *m)
{
	PULLWORD(pPC);
}

static void op_abx(m6801 *m)
{
	X += B;
}

static void op_rti(m6801 *m)
{
	PULLBYTE(CC);
	PULLBYTE(B);
	PULLBYTE(A);
	PULLWORD(pX);
	PULLWORD(pPC);

	check_irq_lines(m);
}

static void op_pshx(m6801 *m)
{
	PUSHWORD(pX);
}

static void op_mul(m6801 *m)
{
	uint16_t t;
	t=A*B;
	CLR_C;
	if (t & 0x80) SEC;
	D=t;
}

static void op_wai(m6801 *m)
{
	// WAI stacks the entire machine state on the hardware stack,
	// then waits for an interrupt.
	m->wai_state |= M6800_WAI;

	PUSHWORD(pPC);
	PUSHWORD(pX);
	PUSHBYTE(A);
	PUSHBYTE(B);
	PUSHBYTE(CC);

	check_irq_lines(m);
	if (m->wai_state & M6800_WAI)
		eat_cycles(m);
}

static void op_swi(m6801 *m)
{
	PUSHWORD(pPC);
	PUSHWORD(pX);
	PUSHBYTE(A);
	PUSHBYTE(B);
	PUSHBYTE(CC);

	SEI;
	PCD = RM16(0xfffa);
}

static void op_nega(m6801 *m)
{
	uint16_t r;
	r=-A;
	CLR_NZVC; SET_FLAGS8(0,A,r);
	A=r;
}

static void op_coma(m6801 *m)
{
	A = ~A;
	CLR_NZV; SET_NZ8(A); SEC;
}

static void op_lsra(m6801 *m)
{
	CLR_NZVC; CC|=(A&0x01);
	A>>=1; SET_Z8(A);
	if (NXORC) SEV;
}

static void op_rora(m6801 *m)
{
	uint8_t r;
	r=(CC&0x01)<<7;
	CLR_NZVC; CC|=(A&0x01);
	r |= A>>1; SET_NZ8(r);
	if (NXORC) SEV;
	A=r;
}

static void op_asra(m6801 *m)
{
	CLR_NZVC; CC|=(A&0x01);
	A>>=1; A|=((A&0x40)<<1);
	SET_NZ8(A);
	if (NXORC) SEV;
}

static void op_asla(m6801 *m)
{
	uint16_t r;
	r=A<<1;
	CLR_NZVC; SET_FLAGS8(A,A,r);
	A=r;
}

static void op_rola(m6801 *m)
{
	uint16_t t,r;
	t = A; r = CC&0x01; r |= t<<1;
	CLR_NZVC; SET_FLAGS8(t,t,r);
	A=r;
}

static void op_deca(m6801 *m)
{
	--A;
	CLR_NZV; SET_FLAGS8D(A);
}

static void op_inca(m6801 *m)
{
	++A;
	CLR_NZV; SET_FLAGS8I(A);
}

static void op_tsta(m6801 *m)
{
	CLR_NZVC; SET_NZ8(A);
}

static void op_clra(m6801 *m)
{
	A=0;
	CLR_NZVC; SEZ;
}

static void op_negb(m6801 *m)
{
	uint16_t r;
	r=-B;
	CLR_NZVC; SET_FLAGS8(0,B,r);
	B=r;
}

static void op_comb(m6801 *m)
{
	B = ~B;
	CLR_NZV; SET_NZ8(B); SEC;
}

static void op_lsrb(m6801 *m)
{
	CLR_NZVC; CC|=(B&0x01);
	B>>=1; SET_Z8(B);
	if (NXORC) SEV;
}

static void op_rorb(m6801 *m)
{
	uint8_t r;
	r=(CC&0x01)<<7;
	CLR_NZVC; CC|=(B&0x01);
	r |= B>>1; SET_NZ8(r);
	if (NXORC) SEV;
	B=r;
}

static void op_asrb(m6801 *m)
{
	CLR_NZVC; CC|=(B&0x01);
	B>>=1; B|=((B&0x40)<<1);
	SET_NZ8(B);
	if (NXORC) SEV;
}

static void op_aslb(m6801 *m)
{
	uint16_t r;
	r=B<<1;
	CLR_NZVC; SET_FLAGS8(B,B,r);
	B=r;
}

static void op_rolb(m6801 *m)
{
	uint16_t t,r;
	t = B; r = CC&0x01; r |= t<<1;
	CLR_NZVC; SET_FLAGS8(t,t,r);
	B=r;
}

static void op_decb(m6801 *m)
{
	--B;
	CLR_NZV; SET_FLAGS8D(B);
}

static void op_incb(m6801 *m)
{
	++B;
	CLR_NZV; SET_FLAGS8I(B);
}

static void op_tstb(m6801 *m)
{
	CLR_NZVC; SET_NZ8(B);
}

static void op_clrb(m6801 *m)
{
	B=0;
	CLR_NZVC; SEZ;
}

static void op_neg_ix(m6801 *m)
{
	uint16_t r,t;
	IDXBYTE(t); r=-t;
	CLR_NZVC; SET_FLAGS8(0,t,r);
	WM(EAD,r);
}

static void op_com_ix(m6801 *m)
{
	uint8_t t;
	IDXBYTE(t); t = ~t;
	CLR_NZV; SET_NZ8(t); SEC;
	WM(EAD,t);
}

static void op_lsr_ix(m6801 *m)
{
	uint8_t t;
	IDXBYTE(t); CLR_NZVC; CC|=(t&0x01);
	t>>=1; SET_Z8(t);
	if (NXORC) SEV;
	WM(EAD,t);
}

static void op_ror_ix(m6801 *m)
{
	uint8_t t,r;
	IDXBYTE(t); r=(CC&0x01)<<7;
	CLR_NZVC; CC|=(t&0x01);
	r |= t>>1; SET_NZ8(r);
	if (NXORC) SEV;
	WM(EAD,r);
}

static void op_asr_ix(m6801 *m)
{
	uint8_t t;
	IDXBYTE(t); CLR_NZVC; CC|=(t&0x01);
	t>>=1; t|=((t&0x40)<<1);
	SET_NZ8(t);
	if (NXORC) SEV;
	WM(EAD,t);
}

static void op_asl_ix(m6801 *m)
{
	uint16_t t,r;
	IDXBYTE(t); r=t<<1;
	CLR_NZVC; SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

static void op_rol_ix(m6801 *m)
{
	uint16_t t,r;
	IDXBYTE(t); r = CC&0x01; r |= t<<1;
	CLR_NZVC; SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

static void op_dec_ix(m6801 *m)
{
	uint8_t t;
	IDXBYTE(t); --t;
	CLR_NZV; SET_FLAGS8D(t);
	WM(EAD,t);
}

static void op_inc_ix(m6801 *m)
{
	uint8_t t;
	IDXBYTE(t); ++t;
	CLR_NZV; SET_FLAGS8I(t);
	WM(EAD,t);
}

static void op_tst_ix(m6801 *m)
{
	uint8_t t;
	IDXBYTE(t); CLR_NZVC; SET_NZ8(t);
}

static void op_jmp_ix(m6801 *m)
{
	INDEXED; PC=EA;
}

static void op_clr_ix(m6801 *m)
{
	INDEXED; RM(EAD); WM(EAD,0);
	CLR_NZVC; SEZ;
}

static void op_neg_ex(m6801 *m)
{
	uint16_t r,t;
	EXTBYTE(t); r=-t;
	CLR_NZVC; SET_FLAGS8(0,t,r);
	WM(EAD,r);
}

static void op_com_ex(m6801 *m)
{
	uint8_t t;
	EXTBYTE(t); t = ~t;
	CLR_NZV; SET_NZ8(t); SEC;
	WM(EAD,t);
}

static void op_lsr_ex(m6801 *m)
{
	uint8_t t;
	EXTBYTE(t);
	CLR_NZVC;
	CC|=(t&0x01);
	t>>=1;
	SET_Z8(t);
	if (NXORC) SEV;
	WM(EAD,t);
}

static void op_ror_ex(m6801 *m)
{
	uint8_t t,r;
	EXTBYTE(t); r=(CC&0x01)<<7;
	CLR_NZVC; CC|=(t&0x01);
	r |= t>>1; SET_NZ8(r);
	if (NXORC) SEV;
	WM(EAD,r);
}

static void op_asr_ex(m6801 *m)
{
	uint8_t t;
	EXTBYTE(t); CLR_NZVC; CC|=(t&0x01);
	t>>=1; t|=((t&0x40)<<1);
	SET_NZ8(t);
	if (NXORC) SEV;
	WM(EAD,t);
}

static void op_asl_ex(m6801 *m)
{
	uint16_t t,r;
	EXTBYTE(t); r=t<<1;
	CLR_NZVC; SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

static void op_rol_ex(m6801 *m)
{
	uint16_t t,r;
	EXTBYTE(t); r = CC&0x01; r |= t<<1;
	CLR_NZVC; SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

static void op_dec_ex(m6801 *m)
{
	uint8_t t;
	EXTBYTE(t); --t;
	CLR_NZV; SET_FLAGS8D(t);
	WM(EAD,t);
}

static void op_inc_ex(m6801 *m)
{
	uint8_t t;
	EXTBYTE(t); ++t;
	CLR_NZV; SET_FLAGS8I(t);
	WM(EAD,t);
}

static void op_tst_ex(m6801 *m)
{
	uint8_t t;
	EXTBYTE(t); CLR_NZVC; SET_NZ8(t);
}

static void op_jmp_ex(m6801 *m)
{
	EXTENDED; PC=EA;
}

static void op_clr_ex(m6801 *m)
{
	EXTENDED; RM(EAD); WM(EAD,0);
	CLR_NZVC; SEZ;
}

static void op_suba_im(m6801 *m)
{
	uint16_t    t,r;
	IMMBYTE(t); r = A-t;
	CLR_NZVC; SET_FLAGS8(A,t,r);
	A = r;
}

static void op_cmpa_im(m6801 *m)
{
	uint16_t    t,r;
	IMMBYTE(t); r = A-t;
	CLR_NZVC; SET_FLAGS8(A,t,r);
}

static void op_sbca_im(m6801 *m)
{
	uint16_t    t,r;
	IMMBYTE(t); r = A-t-(CC&0x01);
	CLR_NZVC; SET_FLAGS8(A,t,r);
	A = r;
}

static void op_subd_im(m6801 *m)
{
	uint32_t r,d;
	m6801_pair b;
	IMMWORD(b);
	d = D;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D = r;
}

static void op_anda_im(m6801 *m)
{
	uint8_t t;
	IMMBYTE(t); A &= t;
	CLR_NZV; SET_NZ8(A);
}

static void op_bita_im(m6801 *m)
{
	uint8_t t,r;
	IMMBYTE(t); r = A&t;
	CLR_NZV; SET_NZ8(r);
}

static void op_lda_im(m6801 *m)
{
	IMMBYTE(A);
	CLR_NZV; SET_NZ8(A);
}

static void op_sta_im(m6801 *m)
{
	CLR_NZV; SET_NZ8(A);
	IMM8; WM(EAD,A);
}

static void op_eora_im(m6801 *m)
{
	uint8_t t;
	IMMBYTE(t); A ^= t;
	CLR_NZV; SET_NZ8(A);
}

static void op_adca_im(m6801 *m)
{
	uint16_t t,r;
	IMMBYTE(t); r = A+t+(CC&0x01);
	CLR_HNZVC; SET_FLAGS8(A,t,r); SET_H(A,t,r);
	A = r;
}

static void op_ora_im(m6801 *m)
{
	uint8_t t;
	IMMBYTE(t); A |= t;
	CLR_NZV; SET_NZ8(A);
}

static void op_adda_im(m6801 *m)
{
	uint16_t t,r;
	IMMBYTE(t); r = A+t;
	CLR_HNZVC; SET_FLAGS8(A,t,r); SET_H(A,t,r);
	A = r;
}

static void op_cpx_im(m6801 *m)
{
	uint32_t r,d;
	m6801_pair b;
	IMMWORD(b);
	d = X;
	r = d - b.d;
	CLR_NZVC; SET_FLAGS16(d,b.d,r);
}

static void op_bsr(m6801 *m)
{
	uint8_t t;
	IMMBYTE(t);
	PUSHWORD(pPC);
	PC += SIGNED(t);
}

static void op_lds_im(m6801 *m)
{
	IMMWORD(m->s);
	CLR_NZV;
	SET_NZ16(S);
}

static void op_sts_im(m6801 *m)
{
	CLR_NZV;
	SET_NZ16(S);
	IMM16;
	wm16(m, EAD, &m->s);
}

static void op_suba_di(m6801 *m)
{
	uint16_t    t,r;
	DIRBYTE(t); r = A-t;
	CLR_NZVC; SET_FLAGS8(A,t,r);
	A = r;
}

static void op_cmpa_di(m6801 *m)
{
	uint16_t    t,r;
	DIRBYTE(t); r = A-t;
	CLR_NZVC; SET_FLAGS8(A,t,r);
}

static void op_sbca_di(m6801 *m)
{
	uint16_t    t,r;
	DIRBYTE(t); r = A-t-(CC&0x01);
	CLR_NZVC; SET_FLAGS8(A,t,r);
	A = r;
}

static void op_subd_di(m6801 *m)
{
	uint32_t r,d;
	m6801_pair b;
	DIRWORD(b);
	d = D;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D=r;
}

static void op_anda_di(m6801 *m)
{
	uint8_t t;
	DIRBYTE(t); A &= t;
	CLR_NZV; SET_NZ8(A);
}

static void op_bita_di(m6801 *m)
{
	uint8_t t,r;
	DIRBYTE(t); r = A&t;
	CLR_NZV; SET_NZ8(r);
}

static void op_lda_di(m6801 *m)
{
	DIRBYTE(A);
	CLR_NZV;
	SET_NZ8(A);
}

static void op_sta_di(m6801 *m)
{
	CLR_NZV;
	SET_NZ8(A);
	DIRECT;
	WM(EAD,A);
}

static void op_eora_di(m6801 *m)
{
	uint8_t t;
	DIRBYTE(t);
	A ^= t;
	CLR_NZV;
	SET_NZ8(A);
}

static void op_adca_di(m6801 *m)
{
	uint16_t t,r;
	DIRBYTE(t);
	r = A+t+(CC&0x01);
	CLR_HNZVC;
	SET_FLAGS8(A,t,r);
	SET_H(A,t,r);
	A = r;
}

static void op_ora_di(m6801 *m)
{
	uint8_t t;
	DIRBYTE(t);
	A |= t;
	CLR_NZV;
	SET_NZ8(A);
}

static void op_adda_di(m6801 *m)
{
	uint16_t t,r;
	DIRBYTE(t);
	r = A + t;
	CLR_HNZVC;
	SET_FLAGS8(A,t,r);
	SET_H(A,t,r);
	A = r;
}

static void op_cpx_di(m6801 *m)
{
	uint32_t r,d;
	m6801_pair b;
	DIRWORD(b);
	d = X;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

static void op_jsr_di(m6801 *m)
{
	DIRECT;
	PUSHWORD(pPC);
	PC = EA;
}

static void op_lds_di(m6801 *m)
{
	DIRWORD(m->s);
	CLR_NZV;
	SET_NZ16(S);
}

static void op_sts_di(m6801 *m)
{
	CLR_NZV;
	SET_NZ16(S);
	DIRECT;
	wm16(m, EAD, &m->s);
}

static void op_suba_ix(m6801 *m)
{
	uint16_t    t,r;
	IDXBYTE(t);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

static void op_cmpa_ix(m6801 *m)
{
	uint16_t    t,r;
	IDXBYTE(t);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
}

static void op_sbca_ix(m6801 *m)
{
	uint16_t    t,r;
	IDXBYTE(t);
	r = A - t - (CC&0x01);
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

static void op_subd_ix(m6801 *m)
{
	uint32_t r,d;
	m6801_pair b;
	IDXWORD(b);
	d = D;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D = r;
}

static void op_anda_ix(m6801 *m)
{
	uint8_t t;
	IDXBYTE(t); A &= t;
	CLR_NZV;
	SET_NZ8(A);
}

static void op_bita_ix(m6801 *m)
{
	uint8_t t,r;
	IDXBYTE(t); r = A&t;
	CLR_NZV;
	SET_NZ8(r);
}

static void op_lda_ix(m6801 *m)
{
	IDXBYTE(A);
	CLR_NZV;
	SET_NZ8(A);
}

static void op_sta_ix(m6801 *m)
{
	CLR_NZV;
	SET_NZ8(A);
	INDEXED;
	WM(EAD,A);
}

static void op_eora_ix(m6801 *m)
{
	uint8_t t;
	IDXBYTE(t);
	A ^= t;
	CLR_NZV;
	SET_NZ8(A);
}

static void op_adca_ix(m6801 *m)
{
	uint16_t t,r;
	IDXBYTE(t);
	r = A + t + (CC&0x01);
	CLR_HNZVC;
	SET_FLAGS8(A,t,r);
	SET_H(A,t,r);
	A = r;
}

static void op_ora_ix(m6801 *m)
{
	uint8_t t;
	IDXBYTE(t);
	A |= t;
	CLR_NZV;
	SET_NZ8(A);
}

static void op_adda_ix(m6801 *m)
{
	uint16_t t,r;
	IDXBYTE(t);
	r = A+t;
	CLR_HNZVC;
	SET_FLAGS8(A,t,r);
	SET_H(A,t,r);
	A = r;
}

static void op_cpx_ix(m6801 *m)
{
	uint32_t r,d;
	m6801_pair b;
	IDXWORD(b);
	d = X;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

static void op_jsr_ix(m6801 *m)
{
	INDEXED;
	PUSHWORD(pPC);
	PC = EA;
}

static void op_lds_ix(m6801 *m)
{
	IDXWORD(m->s);
	CLR_NZV;
	SET_NZ16(S);
}

static void op_sts_ix(m6801 *m)
{
	CLR_NZV;
	SET_NZ16(S);
	INDEXED;
	wm16(m, EAD, &m->s);
}

static void op_suba_ex(m6801 *m)
{
	uint16_t    t,r;
	EXTBYTE(t);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

static void op_cmpa_ex(m6801 *m)
{
	uint16_t    t,r;
	EXTBYTE(t);
	r = A-t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
}

static void op_sbca_ex(m6801 *m)
{
	uint16_t    t,r;
	EXTBYTE(t);
	r = A-t-(CC&0x01);
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

static void op_subd_ex(m6801 *m)
{
	uint32_t r,d;
	m6801_pair b;
	EXTWORD(b);
	d = D;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D=r;
}

static void op_anda_ex(m6801 *m)
{
	uint8_t t;
	EXTBYTE(t);
	A &= t;
	CLR_NZV;
	SET_NZ8(A);
}

static void op_bita_ex(m6801 *m)
{
	uint8_t t,r;
	EXTBYTE(t);
	r = A&t;
	CLR_NZV;
	SET_NZ8(r);
}

static void op_lda_ex(m6801 *m)
{
	EXTBYTE(A);
	CLR_NZV;
	SET_NZ8(A);
}

static void op_sta_ex(m6801 *m)
{
	CLR_NZV;
	SET_NZ8(A);
	EXTENDED;
	WM(EAD,A);
}

static void op_eora_ex(m6801 *m)
{
	uint8_t t;
	EXTBYTE(t);
	A ^= t;
	CLR_NZV;
	SET_NZ8(A);
}

static void op_adca_ex(m6801 *m)
{
	uint16_t t,r;
	EXTBYTE(t);
	r = A+t+(CC&0x01);
	CLR_HNZVC;
	SET_FLAGS8(A,t,r);
	SET_H(A,t,r);
	A = r;
}

static void op_ora_ex(m6801 *m)
{
	uint8_t t;
	EXTBYTE(t);
	A |= t;
	CLR_NZV;
	SET_NZ8(A);
}

static void op_adda_ex(m6801 *m)
{
	uint16_t t,r;
	EXTBYTE(t);
	r = A+t;
	CLR_HNZVC;
	SET_FLAGS8(A,t,r);
	SET_H(A,t,r);
	A = r;
}

static void op_cpx_ex(m6801 *m)
{
	uint32_t r,d;
	m6801_pair b;
	EXTWORD(b);
	d = X;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

static void op_jsr_ex(m6801 *m)
{
	EXTENDED;
	PUSHWORD(pPC);
	PC = EA;
}

static void op_lds_ex(m6801 *m)
{
	EXTWORD(m->s);
	CLR_NZV;
	SET_NZ16(S);
}

static void op_sts_ex(m6801 *m)
{
	CLR_NZV;
	SET_NZ16(S);
	EXTENDED;
	wm16(m, EAD, &m->s);
}

static void op_subb_im(m6801 *m)
{
	uint16_t    t,r;
	IMMBYTE(t);
	r = B-t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

static void op_cmpb_im(m6801 *m)
{
	uint16_t    t,r;
	IMMBYTE(t);
	r = B-t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
}

static void op_sbcb_im(m6801 *m)
{
	uint16_t    t,r;
	IMMBYTE(t);
	r = B-t-(CC&0x01);
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

static void op_addd_im(m6801 *m)
{
	uint32_t r,d;
	m6801_pair b;
	IMMWORD(b);
	d = D;
	r = d + b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D = r;
}

static void op_andb_im(m6801 *m)
{
	uint8_t t;
	IMMBYTE(t);
	B &= t;
	CLR_NZV;
	SET_NZ8(B);
}

static void op_bitb_im(m6801 *m)
{
	uint8_t t,r;
	IMMBYTE(t);
	r = B&t;
	CLR_NZV;
	SET_NZ8(r);
}

static void op_ldb_im(m6801 *m)
{
	IMMBYTE(B);
	CLR_NZV;
	SET_NZ8(B);
}

static void op_stb_im(m6801 *m)
{
	CLR_NZV;
	SET_NZ8(B);
	IMM8;
	WM(EAD,B);
}

static void op_eorb_im(m6801 *m)
{
	uint8_t t;
	IMMBYTE(t);
	B ^= t;
	CLR_NZV;
	SET_NZ8(B);
}

static void op_adcb_im(m6801 *m)
{
	uint16_t t,r;
	IMMBYTE(t);
	r = B+t+(CC&0x01);
	CLR_HNZVC;
	SET_FLAGS8(B,t,r);
	SET_H(B,t,r);
	B = r;
}

static void op_orb_im(m6801 *m)
{
	uint8_t t;
	IMMBYTE(t);
	B |= t;
	CLR_NZV;
	SET_NZ8(B);
}

static void op_addb_im(m6801 *m)
{
	uint16_t t,r;
	IMMBYTE(t);
	r = B+t;
	CLR_HNZVC;
	SET_FLAGS8(B,t,r);
	SET_H(B,t,r);
	B = r;
}

static void op_ldd_im(m6801 *m)
{
	IMMWORD(m->d);
	CLR_NZV;
	SET_NZ16(D);
}

static void op_std_im(m6801 *m)
{
	IMM16;
	CLR_NZV;
	SET_NZ16(D);
	wm16(m, EAD, &m->d);
}

static void op_ldx_im(m6801 *m)
{
	IMMWORD(m->x);
	CLR_NZV;
	SET_NZ16(X);
}

static void op_stx_im(m6801 *m)
{
	CLR_NZV;
	SET_NZ16(X);
	IMM16;
	wm16(m, EAD, &m->x);
}

static void op_subb_di(m6801 *m)
{
	uint16_t    t,r;
	DIRBYTE(t);
	r = B-t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

static void op_cmpb_di(m6801 *m)
{
	uint16_t    t,r;
	DIRBYTE(t);
	r = B-t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
}

static void op_sbcb_di(m6801 *m)
{
	uint16_t    t,r;
	DIRBYTE(t);
	r = B-t-(CC&0x01);
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

static void op_addd_di(m6801 *m)
{
	uint32_t r,d;
	m6801_pair b;
	DIRWORD(b);
	d = D;
	r = d + b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D = r;
}

static void op_andb_di(m6801 *m)
{
	uint8_t t;
	DIRBYTE(t);
	B &= t;
	CLR_NZV;
	SET_NZ8(B);
}

static void op_bitb_di(m6801 *m)
{
	uint8_t t,r;
	DIRBYTE(t);
	r = B&t;
	CLR_NZV;
	SET_NZ8(r);
}

static void op_ldb_di(m6801 *m)
{
	DIRBYTE(B);
	CLR_NZV;
	SET_NZ8(B);
}

static void op_stb_di(m6801 *m)
{
	CLR_NZV;
	SET_NZ8(B);
	DIRECT;
	WM(EAD,B);
}

static void op_eorb_di(m6801 *m)
{
	uint8_t t;
	DIRBYTE(t);
	B ^= t;
	CLR_NZV;
	SET_NZ8(B);
}

static void op_adcb_di(m6801 *m)
{
	uint16_t t,r;
	DIRBYTE(t);
	r = B+t+(CC&0x01);
	CLR_HNZVC;
	SET_FLAGS8(B,t,r);
	SET_H(B,t,r);
	B = r;
}

static void op_orb_di(m6801 *m)
{
	uint8_t t;
	DIRBYTE(t);
	B |= t;
	CLR_NZV;
	SET_NZ8(B);
}

static void op_addb_di(m6801 *m)
{
	uint16_t t,r;
	DIRBYTE(t);
	r = B+t;
	CLR_HNZVC;
	SET_FLAGS8(B,t,r);
	SET_H(B,t,r);
	B = r;
}

static void op_ldd_di(m6801 *m)
{
	DIRWORD(m->d);
	CLR_NZV;
	SET_NZ16(D);
}

static void op_std_di(m6801 *m)
{
	DIRECT;
	CLR_NZV;
	SET_NZ16(D);
	wm16(m, EAD, &m->d);
}

static void op_ldx_di(m6801 *m)
{
	DIRWORD(m->x);
	CLR_NZV;
	SET_NZ16(X);
}

static void op_stx_di(m6801 *m)
{
	CLR_NZV;
	SET_NZ16(X);
	DIRECT;
	wm16(m, EAD, &m->x);
}

static void op_subb_ix(m6801 *m)
{
	uint16_t    t,r;
	IDXBYTE(t);
	r = B-t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

static void op_cmpb_ix(m6801 *m)
{
	uint16_t    t,r;
	IDXBYTE(t);
	r = B-t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
}

static void op_sbcb_ix(m6801 *m)
{
	uint16_t    t,r;
	IDXBYTE(t);
	r = B-t-(CC&0x01);
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

static void op_addd_ix(m6801 *m)
{
	uint32_t r,d;
	m6801_pair b;
	IDXWORD(b);
	d = D;
	r = d + b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D = r;
}

static void op_andb_ix(m6801 *m)
{
	uint8_t t;
	IDXBYTE(t);
	B &= t;
	CLR_NZV;
	SET_NZ8(B);
}

static void op_bitb_ix(m6801 *m)
{
	uint8_t t,r;
	IDXBYTE(t);
	r = B&t;
	CLR_NZV;
	SET_NZ8(r);
}

static void op_ldb_ix(m6801 *m)
{
	IDXBYTE(B);
	CLR_NZV;
	SET_NZ8(B);
}

static void op_stb_ix(m6801 *m)
{
	CLR_NZV;
	SET_NZ8(B);
	INDEXED;
	WM(EAD,B);
}

static void op_eorb_ix(m6801 *m)
{
	uint8_t t;
	IDXBYTE(t);
	B ^= t;
	CLR_NZV;
	SET_NZ8(B);
}

static void op_adcb_ix(m6801 *m)
{
	uint16_t t,r;
	IDXBYTE(t);
	r = B+t+(CC&0x01);
	CLR_HNZVC;
	SET_FLAGS8(B,t,r);
	SET_H(B,t,r);
	B = r;
}

static void op_orb_ix(m6801 *m)
{
	uint8_t t;
	IDXBYTE(t);
	B |= t;
	CLR_NZV;
	SET_NZ8(B);
}

static void op_addb_ix(m6801 *m)
{
	uint16_t t,r;
	IDXBYTE(t);
	r = B+t;
	CLR_HNZVC;
	SET_FLAGS8(B,t,r);
	SET_H(B,t,r);
	B = r;
}

static void op_ldd_ix(m6801 *m)
{
	IDXWORD(m->d);
	CLR_NZV;
	SET_NZ16(D);
}

static void op_std_ix(m6801 *m)
{
	INDEXED;
	CLR_NZV;
	SET_NZ16(D);
	wm16(m, EAD, &m->d);
}

static void op_ldx_ix(m6801 *m)
{
	IDXWORD(m->x);
	CLR_NZV;
	SET_NZ16(X);
}

static void op_stx_ix(m6801 *m)
{
	CLR_NZV;
	SET_NZ16(X);
	INDEXED;
	wm16(m, EAD, &m->x);
}

static void op_subb_ex(m6801 *m)
{
	uint16_t    t,r;
	EXTBYTE(t);
	r = B-t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

static void op_cmpb_ex(m6801 *m)
{
	uint16_t    t,r;
	EXTBYTE(t);
	r = B-t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
}

static void op_sbcb_ex(m6801 *m)
{
	uint16_t    t,r;
	EXTBYTE(t);
	r = B-t-(CC&0x01);
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

static void op_addd_ex(m6801 *m)
{
	uint32_t r,d;
	m6801_pair b;
	EXTWORD(b);
	d = D;
	r = d + b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D = r;
}

static void op_andb_ex(m6801 *m)
{
	uint8_t t;
	EXTBYTE(t);
	B &= t;
	CLR_NZV;
	SET_NZ8(B);
}

static void op_bitb_ex(m6801 *m)
{
	uint8_t t,r;
	EXTBYTE(t);
	r = B & t;
	CLR_NZV;
	SET_NZ8(r);
}

static void op_ldb_ex(m6801 *m)
{
	EXTBYTE(B);
	CLR_NZV;
	SET_NZ8(B);
}

static void op_stb_ex(m6801 *m)
{
	CLR_NZV;
	SET_NZ8(B);
	EXTENDED;
	WM(EAD,B);
}

static void op_eorb_ex(m6801 *m)
{
	uint8_t t;
	EXTBYTE(t);
	B ^= t;
	CLR_NZV;
	SET_NZ8(B);
}

static void op_adcb_ex(m6801 *m)
{
	uint16_t t,r;
	EXTBYTE(t);
	r = B+t+(CC&0x01);
	CLR_HNZVC;
	SET_FLAGS8(B,t,r);
	SET_H(B,t,r);
	B = r;
}

static void op_orb_ex(m6801 *m)
{
	uint8_t t;
	EXTBYTE(t);
	B |= t;
	CLR_NZV;
	SET_NZ8(B);
}

static void op_addb_ex(m6801 *m)
{
	uint16_t t,r;
	EXTBYTE(t);
	r = B+t;
	CLR_HNZVC;
	SET_FLAGS8(B,t,r);
	SET_H(B,t,r);
	B = r;
}

static void op_ldd_ex(m6801 *m)
{
	EXTWORD(m->d);
	CLR_NZV;
	SET_NZ16(D);
}

static void op_std_ex(m6801 *m)
{
	EXTENDED;
	CLR_NZV;
	SET_NZ16(D);
	wm16(m, EAD, &m->d);
}

static void op_ldx_ex(m6801 *m)
{
	EXTWORD(m->x);
	CLR_NZV;
	SET_NZ16(X);
}

static void op_stx_ex(m6801 *m)
{
	CLR_NZV;
	SET_NZ16(X);
	EXTENDED;
	wm16(m, EAD, &m->x);
}

// 6800 CPX: N and V from the high bytes only, Z from the 16-bit result, C unchanged
static void op_cmpx_im(m6801 *m)
{
	m6801_pair r, d, b;
	IMMWORD(b);
	d.d = X;
	r.w.l = (uint16_t)(d.b.h - b.b.h);
	CLR_NZV;
	SET_N8(r.b.l);
	SET_V8(d.b.h, b.b.h, r.w.l);
	r.d = d.d - b.d;
	SET_Z16(r.d);
}

// 6800 CPX: N and V from the high bytes only, Z from the 16-bit result, C unchanged
static void op_cmpx_di(m6801 *m)
{
	m6801_pair r, d, b;
	DIRWORD(b);
	d.d = X;
	r.w.l = (uint16_t)(d.b.h - b.b.h);
	CLR_NZV;
	SET_N8(r.b.l);
	SET_V8(d.b.h, b.b.h, r.w.l);
	r.d = d.d - b.d;
	SET_Z16(r.d);
}

// 6800 CPX: N and V from the high bytes only, Z from the 16-bit result, C unchanged
static void op_cmpx_ix(m6801 *m)
{
	m6801_pair r, d, b;
	IDXWORD(b);
	d.d = X;
	r.w.l = (uint16_t)(d.b.h - b.b.h);
	CLR_NZV;
	SET_N8(r.b.l);
	SET_V8(d.b.h, b.b.h, r.w.l);
	r.d = d.d - b.d;
	SET_Z16(r.d);
}

// 6800 CPX: N and V from the high bytes only, Z from the 16-bit result, C unchanged
static void op_cmpx_ex(m6801 *m)
{
	m6801_pair r, d, b;
	EXTWORD(b);
	d.d = X;
	r.w.l = (uint16_t)(d.b.h - b.b.h);
	CLR_NZV;
	SET_N8(r.b.l);
	SET_V8(d.b.h, b.b.h, r.w.l);
	r.d = d.d - b.d;
	SET_Z16(r.d);
}

// ---- HD63701 only (6800ops.hxx) ----
// illegal opcode: hd6301_cpu_device::take_trap
static void op_trap(m6801 *m)
{
	PC--;
	enter_interrupt(m, 0xffee);
}

/* $12 ILLEGAL */
static void op_undoc1(m6801 *m)
{
	X += RM( S + 1 );
}

/* $13 ILLEGAL */
static void op_undoc2(m6801 *m)
{
	X += RM( S + 1 );
}

/* $18 XGDX inherent ----- */
static void op_xgdx(m6801 *m)
{
	uint16_t t = X;
	X = D;
	D=t;
}

/* $1a SLP */
static void op_slp(m6801 *m)
{
	// wait for next IRQ (same as waiting of WAI)
	m->wai_state |= M6800_SLP;

	check_irq_lines(m);
	if (m->wai_state & M6800_SLP)
		eat_cycles(m);
}

/* $61 AIM --**0- */
static void op_aim_ix(m6801 *m)
{
	uint8_t t, r;
	IMMBYTE(t);
	IDXBYTE(r);
	r &= t;
	CLR_NZV; SET_NZ8(r);
	WM(EAD,r);
}

/* $62 OIM --**0- */
static void op_oim_ix(m6801 *m)
{
	uint8_t t, r;
	IMMBYTE(t);
	IDXBYTE(r);
	r |= t;
	CLR_NZV; SET_NZ8(r);
	WM(EAD,r);
}

/* $65 EIM --**0- */
static void op_eim_ix(m6801 *m)
{
	uint8_t t, r;
	IMMBYTE(t);
	IDXBYTE(r);
	r ^= t;
	CLR_NZV; SET_NZ8(r);
	WM(EAD,r);
}

/* $6b TIM --**0- */
static void op_tim_ix(m6801 *m)
{
	uint8_t t, r;
	IMMBYTE(t);
	IDXBYTE(r);
	r &= t;
	CLR_NZV; SET_NZ8(r);
}

/* $71 AIM --**0- */
static void op_aim_di(m6801 *m)
{
	uint8_t t, r;
	IMMBYTE(t);
	DIRBYTE(r);
	r &= t;
	CLR_NZV; SET_NZ8(r);
	WM(EAD,r);
}

/* $72 OIM --**0- */
static void op_oim_di(m6801 *m)
{
	uint8_t t, r;
	IMMBYTE(t);
	DIRBYTE(r);
	r |= t;
	CLR_NZV; SET_NZ8(r);
	WM(EAD,r);
}

/* $75 EIM --**0- */
static void op_eim_di(m6801 *m)
{
	uint8_t t, r;
	IMMBYTE(t);
	DIRBYTE(r);
	r ^= t;
	CLR_NZV; SET_NZ8(r);
	WM(EAD,r);
}

/* $7b TIM --**0- */
static void op_tim_di(m6801 *m)
{
	uint8_t t, r;
	IMMBYTE(t);
	DIRBYTE(r);
	r &= t;
	CLR_NZV; SET_NZ8(r);
}

static void (*const insn_6803[256])(m6801 *) = {
	op_illegl1, op_nop, op_illegl1, op_illegl1, op_lsrd, op_asld, op_tap, op_tpa,
	op_inx, op_dex, op_clv, op_sev, op_clc, op_sec, op_cli, op_sei,
	op_sba, op_cba, op_illegl1, op_illegl1, op_illegl1, op_illegl1, op_tab, op_tba,
	op_illegl1, op_daa, op_illegl1, op_aba, op_illegl1, op_illegl1, op_illegl1, op_illegl1,
	op_bra, op_brn, op_bhi, op_bls, op_bcc, op_bcs, op_bne, op_beq,
	op_bvc, op_bvs, op_bpl, op_bmi, op_bge, op_blt, op_bgt, op_ble,
	op_tsx, op_ins, op_pula, op_pulb, op_des, op_txs, op_psha, op_pshb,
	op_pulx, op_rts, op_abx, op_rti, op_pshx, op_mul, op_wai, op_swi,
	op_nega, op_illegl1, op_illegl1, op_coma, op_lsra, op_illegl1, op_rora, op_asra,
	op_asla, op_rola, op_deca, op_illegl1, op_inca, op_tsta, op_illegl1, op_clra,
	op_negb, op_illegl1, op_illegl1, op_comb, op_lsrb, op_illegl1, op_rorb, op_asrb,
	op_aslb, op_rolb, op_decb, op_illegl1, op_incb, op_tstb, op_illegl1, op_clrb,
	op_neg_ix, op_illegl1, op_illegl1, op_com_ix, op_lsr_ix, op_illegl1, op_ror_ix, op_asr_ix,
	op_asl_ix, op_rol_ix, op_dec_ix, op_illegl1, op_inc_ix, op_tst_ix, op_jmp_ix, op_clr_ix,
	op_neg_ex, op_illegl1, op_illegl1, op_com_ex, op_lsr_ex, op_illegl1, op_ror_ex, op_asr_ex,
	op_asl_ex, op_rol_ex, op_dec_ex, op_illegl1, op_inc_ex, op_tst_ex, op_jmp_ex, op_clr_ex,
	op_suba_im, op_cmpa_im, op_sbca_im, op_subd_im, op_anda_im, op_bita_im, op_lda_im, op_sta_im,
	op_eora_im, op_adca_im, op_ora_im, op_adda_im, op_cpx_im, op_bsr, op_lds_im, op_sts_im,
	op_suba_di, op_cmpa_di, op_sbca_di, op_subd_di, op_anda_di, op_bita_di, op_lda_di, op_sta_di,
	op_eora_di, op_adca_di, op_ora_di, op_adda_di, op_cpx_di, op_jsr_di, op_lds_di, op_sts_di,
	op_suba_ix, op_cmpa_ix, op_sbca_ix, op_subd_ix, op_anda_ix, op_bita_ix, op_lda_ix, op_sta_ix,
	op_eora_ix, op_adca_ix, op_ora_ix, op_adda_ix, op_cpx_ix, op_jsr_ix, op_lds_ix, op_sts_ix,
	op_suba_ex, op_cmpa_ex, op_sbca_ex, op_subd_ex, op_anda_ex, op_bita_ex, op_lda_ex, op_sta_ex,
	op_eora_ex, op_adca_ex, op_ora_ex, op_adda_ex, op_cpx_ex, op_jsr_ex, op_lds_ex, op_sts_ex,
	op_subb_im, op_cmpb_im, op_sbcb_im, op_addd_im, op_andb_im, op_bitb_im, op_ldb_im, op_stb_im,
	op_eorb_im, op_adcb_im, op_orb_im, op_addb_im, op_ldd_im, op_std_im, op_ldx_im, op_stx_im,
	op_subb_di, op_cmpb_di, op_sbcb_di, op_addd_di, op_andb_di, op_bitb_di, op_ldb_di, op_stb_di,
	op_eorb_di, op_adcb_di, op_orb_di, op_addb_di, op_ldd_di, op_std_di, op_ldx_di, op_stx_di,
	op_subb_ix, op_cmpb_ix, op_sbcb_ix, op_addd_ix, op_andb_ix, op_bitb_ix, op_ldb_ix, op_stb_ix,
	op_eorb_ix, op_adcb_ix, op_orb_ix, op_addb_ix, op_ldd_ix, op_std_ix, op_ldx_ix, op_stx_ix,
	op_subb_ex, op_cmpb_ex, op_sbcb_ex, op_addd_ex, op_andb_ex, op_bitb_ex, op_ldb_ex, op_stb_ex,
	op_eorb_ex, op_adcb_ex, op_orb_ex, op_addb_ex, op_ldd_ex, op_std_ex, op_ldx_ex, op_stx_ex,
};

static const uint8_t cycles_6803[256] = {
	XX, 2, XX, XX, 3, 3, 2, 2, 3, 3, 2, 2, 2, 2, 2, 2,
	2, 2, XX, XX, XX, XX, 2, 2, XX, 2, XX, 2, XX, XX, XX, XX,
	3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
	3, 3, 4, 4, 3, 3, 3, 3, 5, 5, 3, 10, 4, 10, 9, 12,
	2, XX, XX, 2, 2, XX, 2, 2, 2, 2, 2, XX, 2, 2, XX, 2,
	2, XX, XX, 2, 2, XX, 2, 2, 2, 2, 2, XX, 2, 2, XX, 2,
	6, XX, XX, 6, 6, XX, 6, 6, 6, 6, 6, XX, 6, 6, 3, 6,
	6, XX, XX, 6, 6, XX, 6, 6, 6, 6, 6, XX, 6, 6, 3, 6,
	2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 2, 4, 6, 3, 3,
	3, 3, 3, 5, 3, 3, 3, 3, 3, 3, 3, 3, 5, 5, 4, 4,
	4, 4, 4, 6, 4, 4, 4, 4, 4, 4, 4, 4, 6, 6, 5, 5,
	4, 4, 4, 6, 4, 4, 4, 4, 4, 4, 4, 4, 6, 6, 5, 5,
	2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 2, 3, XX, 3, 3,
	3, 3, 3, 5, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4,
	4, 4, 4, 6, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5,
	4, 4, 4, 6, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5,
};

// MAME m6800_cpu_device::m6800_insn / cycles_6800
static void (*const insn_6800[256])(m6801 *) = {
	op_illegl1, op_nop, op_illegl1, op_illegl1, op_illegl1, op_illegl1, op_tap, op_tpa,
	op_inx, op_dex, op_clv, op_sev, op_clc, op_sec, op_cli, op_sei,
	op_sba, op_cba, op_illegl1, op_illegl1, op_illegl1, op_illegl1, op_tab, op_tba,
	op_illegl1, op_daa, op_illegl1, op_aba, op_illegl1, op_illegl1, op_illegl1, op_illegl1,
	op_bra, op_brn, op_bhi, op_bls, op_bcc, op_bcs, op_bne, op_beq,
	op_bvc, op_bvs, op_bpl, op_bmi, op_bge, op_blt, op_bgt, op_ble,
	op_tsx, op_ins, op_pula, op_pulb, op_des, op_txs, op_psha, op_pshb,
	op_illegl1, op_rts, op_illegl1, op_rti, op_illegl1, op_illegl1, op_wai, op_swi,
	op_nega, op_illegl1, op_illegl1, op_coma, op_lsra, op_illegl1, op_rora, op_asra,
	op_asla, op_rola, op_deca, op_illegl1, op_inca, op_tsta, op_illegl1, op_clra,
	op_negb, op_illegl1, op_illegl1, op_comb, op_lsrb, op_illegl1, op_rorb, op_asrb,
	op_aslb, op_rolb, op_decb, op_illegl1, op_incb, op_tstb, op_illegl1, op_clrb,
	op_neg_ix, op_illegl2, op_illegl2, op_com_ix, op_lsr_ix, op_illegl2, op_ror_ix, op_asr_ix,
	op_asl_ix, op_rol_ix, op_dec_ix, op_illegl2, op_inc_ix, op_tst_ix, op_jmp_ix, op_clr_ix,
	op_neg_ex, op_illegl3, op_illegl3, op_com_ex, op_lsr_ex, op_illegl3, op_ror_ex, op_asr_ex,
	op_asl_ex, op_rol_ex, op_dec_ex, op_illegl3, op_inc_ex, op_tst_ex, op_jmp_ex, op_clr_ex,
	op_suba_im, op_cmpa_im, op_sbca_im, op_illegl2, op_anda_im, op_bita_im, op_lda_im, op_sta_im,
	op_eora_im, op_adca_im, op_ora_im, op_adda_im, op_cmpx_im, op_bsr, op_lds_im, op_sts_im,
	op_suba_di, op_cmpa_di, op_sbca_di, op_illegl2, op_anda_di, op_bita_di, op_lda_di, op_sta_di,
	op_eora_di, op_adca_di, op_ora_di, op_adda_di, op_cmpx_di, op_jsr_di, op_lds_di, op_sts_di,
	op_suba_ix, op_cmpa_ix, op_sbca_ix, op_illegl2, op_anda_ix, op_bita_ix, op_lda_ix, op_sta_ix,
	op_eora_ix, op_adca_ix, op_ora_ix, op_adda_ix, op_cmpx_ix, op_jsr_ix, op_lds_ix, op_sts_ix,
	op_suba_ex, op_cmpa_ex, op_sbca_ex, op_illegl3, op_anda_ex, op_bita_ex, op_lda_ex, op_sta_ex,
	op_eora_ex, op_adca_ex, op_ora_ex, op_adda_ex, op_cmpx_ex, op_jsr_ex, op_lds_ex, op_sts_ex,
	op_subb_im, op_cmpb_im, op_sbcb_im, op_illegl2, op_andb_im, op_bitb_im, op_ldb_im, op_stb_im,
	op_eorb_im, op_adcb_im, op_orb_im, op_addb_im, op_illegl3, op_illegl3, op_ldx_im, op_stx_im,
	op_subb_di, op_cmpb_di, op_sbcb_di, op_illegl2, op_andb_di, op_bitb_di, op_ldb_di, op_stb_di,
	op_eorb_di, op_adcb_di, op_orb_di, op_addb_di, op_illegl2, op_illegl2, op_ldx_di, op_stx_di,
	op_subb_ix, op_cmpb_ix, op_sbcb_ix, op_illegl2, op_andb_ix, op_bitb_ix, op_ldb_ix, op_stb_ix,
	op_eorb_ix, op_adcb_ix, op_orb_ix, op_addb_ix, op_illegl2, op_illegl2, op_ldx_ix, op_stx_ix,
	op_subb_ex, op_cmpb_ex, op_sbcb_ex, op_illegl3, op_andb_ex, op_bitb_ex, op_ldb_ex, op_stb_ex,
	op_eorb_ex, op_adcb_ex, op_orb_ex, op_addb_ex, op_illegl3, op_illegl3, op_ldx_ex, op_stx_ex,
};

static const uint8_t cycles_6800[256] = {
	XX, 2, XX, XX, XX, XX, 2, 2, 4, 4, 2, 2, 2, 2, 2, 2,
	2, 2, XX, XX, XX, XX, 2, 2, XX, 2, XX, 2, XX, XX, XX, XX,
	4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
	4, 4, 4, 4, 4, 4, 4, 4, XX, 5, XX, 10, XX, XX, 9, 12,
	2, XX, XX, 2, 2, XX, 2, 2, 2, 2, 2, XX, 2, 2, XX, 2,
	2, XX, XX, 2, 2, XX, 2, 2, 2, 2, 2, XX, 2, 2, XX, 2,
	7, XX, XX, 7, 7, XX, 7, 7, 7, 7, 7, XX, 7, 7, 4, 7,
	6, XX, XX, 6, 6, XX, 6, 6, 6, 6, 6, XX, 6, 6, 3, 6,
	2, 2, 2, XX, 2, 2, 2, 3, 2, 2, 2, 2, 3, 8, 3, 4,
	3, 3, 3, XX, 3, 3, 3, 4, 3, 3, 3, 3, 4, 6, 4, 5,
	5, 5, 5, XX, 5, 5, 5, 6, 5, 5, 5, 5, 6, 8, 6, 7,
	4, 4, 4, XX, 4, 4, 4, 5, 4, 4, 4, 4, 5, 9, 5, 6,
	2, 2, 2, XX, 2, 2, 2, 3, 2, 2, 2, 2, XX, XX, 3, 4,
	3, 3, 3, XX, 3, 3, 3, 4, 3, 3, 3, 3, XX, XX, 4, 5,
	5, 5, 5, XX, 5, 5, 5, 6, 5, 5, 5, 5, XX, XX, 6, 7,
	4, 4, 4, XX, 4, 4, 4, 5, 4, 4, 4, 4, XX, XX, 5, 6,
};

// MAME m6801_cpu_device::hd63701_insn / cycles_63701 (hd6301_cpu_device)
static void (*const insn_63701[256])(m6801 *) = {
	op_trap, op_nop, op_trap, op_trap, op_lsrd, op_asld, op_tap, op_tpa,
	op_inx, op_dex, op_clv, op_sev, op_clc, op_sec, op_cli, op_sei,
	op_sba, op_cba, op_undoc1, op_undoc2, op_trap, op_trap, op_tab, op_tba,
	op_xgdx, op_daa, op_slp, op_aba, op_trap, op_trap, op_trap, op_trap,
	op_bra, op_brn, op_bhi, op_bls, op_bcc, op_bcs, op_bne, op_beq,
	op_bvc, op_bvs, op_bpl, op_bmi, op_bge, op_blt, op_bgt, op_ble,
	op_tsx, op_ins, op_pula, op_pulb, op_des, op_txs, op_psha, op_pshb,
	op_pulx, op_rts, op_abx, op_rti, op_pshx, op_mul, op_wai, op_swi,
	op_nega, op_trap, op_trap, op_coma, op_lsra, op_trap, op_rora, op_asra,
	op_asla, op_rola, op_deca, op_trap, op_inca, op_tsta, op_trap, op_clra,
	op_negb, op_trap, op_trap, op_comb, op_lsrb, op_trap, op_rorb, op_asrb,
	op_aslb, op_rolb, op_decb, op_trap, op_incb, op_tstb, op_trap, op_clrb,
	op_neg_ix, op_aim_ix, op_oim_ix, op_com_ix, op_lsr_ix, op_eim_ix, op_ror_ix, op_asr_ix,
	op_asl_ix, op_rol_ix, op_dec_ix, op_tim_ix, op_inc_ix, op_tst_ix, op_jmp_ix, op_clr_ix,
	op_neg_ex, op_aim_di, op_oim_di, op_com_ex, op_lsr_ex, op_eim_di, op_ror_ex, op_asr_ex,
	op_asl_ex, op_rol_ex, op_dec_ex, op_tim_di, op_inc_ex, op_tst_ex, op_jmp_ex, op_clr_ex,
	op_suba_im, op_cmpa_im, op_sbca_im, op_subd_im, op_anda_im, op_bita_im, op_lda_im, op_trap,
	op_eora_im, op_adca_im, op_ora_im, op_adda_im, op_cpx_im, op_bsr, op_lds_im, op_trap,
	op_suba_di, op_cmpa_di, op_sbca_di, op_subd_di, op_anda_di, op_bita_di, op_lda_di, op_sta_di,
	op_eora_di, op_adca_di, op_ora_di, op_adda_di, op_cpx_di, op_jsr_di, op_lds_di, op_sts_di,
	op_suba_ix, op_cmpa_ix, op_sbca_ix, op_subd_ix, op_anda_ix, op_bita_ix, op_lda_ix, op_sta_ix,
	op_eora_ix, op_adca_ix, op_ora_ix, op_adda_ix, op_cpx_ix, op_jsr_ix, op_lds_ix, op_sts_ix,
	op_suba_ex, op_cmpa_ex, op_sbca_ex, op_subd_ex, op_anda_ex, op_bita_ex, op_lda_ex, op_sta_ex,
	op_eora_ex, op_adca_ex, op_ora_ex, op_adda_ex, op_cpx_ex, op_jsr_ex, op_lds_ex, op_sts_ex,
	op_subb_im, op_cmpb_im, op_sbcb_im, op_addd_im, op_andb_im, op_bitb_im, op_ldb_im, op_trap,
	op_eorb_im, op_adcb_im, op_orb_im, op_addb_im, op_ldd_im, op_trap, op_ldx_im, op_trap,
	op_subb_di, op_cmpb_di, op_sbcb_di, op_addd_di, op_andb_di, op_bitb_di, op_ldb_di, op_stb_di,
	op_eorb_di, op_adcb_di, op_orb_di, op_addb_di, op_ldd_di, op_std_di, op_ldx_di, op_stx_di,
	op_subb_ix, op_cmpb_ix, op_sbcb_ix, op_addd_ix, op_andb_ix, op_bitb_ix, op_ldb_ix, op_stb_ix,
	op_eorb_ix, op_adcb_ix, op_orb_ix, op_addb_ix, op_ldd_ix, op_std_ix, op_ldx_ix, op_stx_ix,
	op_subb_ex, op_cmpb_ex, op_sbcb_ex, op_addd_ex, op_andb_ex, op_bitb_ex, op_ldb_ex, op_stb_ex,
	op_eorb_ex, op_adcb_ex, op_orb_ex, op_addb_ex, op_ldd_ex, op_std_ex, op_ldx_ex, op_stx_ex,
};

static const uint8_t cycles_63701[256] = {
	XX, 1, XX, XX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, XX, XX, XX, XX, 1, 1, 2, 2, 4, 1, XX, XX, XX, XX,
	3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
	1, 1, 3, 3, 1, 1, 4, 4, 4, 5, 1, 10, 5, 7, 9, 12,
	1, XX, XX, 1, 1, XX, 1, 1, 1, 1, 1, XX, 1, 1, XX, 1,
	1, XX, XX, 1, 1, XX, 1, 1, 1, 1, 1, XX, 1, 1, XX, 1,
	6, 7, 7, 6, 6, 7, 6, 6, 6, 6, 6, 5, 6, 4, 3, 5,
	6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 4, 6, 4, 3, 5,
	2, 2, 2, 3, 2, 2, 2, XX, 2, 2, 2, 2, 3, 5, 3, XX,
	3, 3, 3, 4, 3, 3, 3, 3, 3, 3, 3, 3, 4, 5, 4, 4,
	4, 4, 4, 5, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5,
	4, 4, 4, 5, 4, 4, 4, 4, 4, 4, 4, 4, 5, 6, 5, 5,
	2, 2, 2, 3, 2, 2, 2, XX, 2, 2, 2, 2, 3, XX, 3, XX,
	3, 3, 3, 4, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4,
	4, 4, 4, 5, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5,
	4, 4, 4, 5, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5,
};

#undef XX

// ---- execution ----
static void execute_one(m6801 *m)
{
	pPPC = pPC;
	uint8_t ireg = M_RDOP(PCD);
	PC++;
	if (m->type == M6801_TYPE_6800) {
		insn_6800[ireg](m);
		increment_counter(m, cycles_6800[ireg]);
	} else if (IS_HD6301(m)) {
		insn_63701[ireg](m);
		increment_counter(m, cycles_63701[ireg]);
	} else {
		insn_6803[ireg](m);
		increment_counter(m, cycles_6803[ireg]);
	}
}

void m6801_init(m6801 *m)
{
	memset(&m->ppc, 0, m6801_state_size());
}

void m6801_reset(m6801 *m)
{
	// m6800_cpu_device::device_reset
	m->cc = 0xc0;
	SEI;
	PCD = RM16(0xfffe);
	m->wai_state = 0;
	m->nmi_state = 0;
	m->nmi_pending = 0;
	// m6801_cpu_device::device_reset
	m->is3_state = 0;
	memset(m->port_ddr, 0, sizeof(m->port_ddr));
	m->p3csr = 0x00;
	m->pending_isf_clear = 0;
	m->port2_written = 0;
	m->port3_latched = 0;
	m->tcsr = 0x00;
	m->pending_tcsr = 0x00;
	CTD = 0x0000;
	OCD = 0xffff;
	TOD = 0xffff;
	m->timer_next = 0xffff;
	m->ram_ctrl |= 0x40;
	m->latch09 = 0;
	m->trcsr = M6801_TRCSR_TDRE;
	m->tx = 1;
	m->trcsr_read_tdre = 0;
	m->trcsr_read_orfe = 0;
	m->trcsr_read_rdrf = 0;
	m->rmcr = 0;
	// m6801u4_cpu_device::device_reset
	m->tcr[0] = 0;
	m->tcr[1] = 0;
	m->tsr = 0;
	m->pending_tsr = 0;
}

int m6801_execute(m6801 *m, int cycles)
{
	int32_t start = m->icount + cycles;
	m->icount = start;
	if (m->reset_held) {
		m->icount = 0;
		return start > 0 ? start : 0;
	}
	if (m->icount <= 0)
		return 0;
	check_irq_lines(m);
	cleanup_counters(m);
	do {
		if (m->wai_state & (M6800_WAI | M6800_SLP))
			eat_cycles(m);
		else
			execute_one(m);
	} while (m->icount > 0);
	return start - m->icount;
}

void m6801_set_irq_line(m6801 *m, int line, int state)
{
	const int hold = line == M6801_IRQ1_LINE && state == M6801_HOLD_LINE;
	state = state != 0;
	switch (line) {
	case M6801_NMI_LINE:
		if (!m->nmi_state && state)
			m->nmi_pending = 1;
		m->nmi_state = (uint8_t)state;
		break;
	case M6801_IS3_LINE:
		if (!m->is3_state && state) {
			if (!m->port3_latched && (m->p3csr & M6801_P3CSR_LE)) {
				// latch input data to port 3
				uint8_t ddr = m->port_ddr[2];
				m->port_data[2] = (uint8_t)((port_in(m, 2) & ~ddr) | (m->port_data[2] & ddr));
				m->port3_latched = 1;
				m->p3csr |= M6801_P3CSR_IS3_FLAG;
			}
		}
		m->is3_state = (uint8_t)state;
		break;
	case M6801_TIN_LINE:
		if (state != m->irq_state[M6801_TIN_LINE]) {
			m->irq_state[M6801_TIN_LINE] = (uint8_t)state;
			if (((m->tcsr & TCSR_IEDG) ^ (state == 0 ? TCSR_IEDG : 0)) == 0)
				return;
			// active edge in
			m->tcsr |= TCSR_ICF;
			m->pending_tcsr |= TCSR_ICF;
			m->input_capture = CT;
		}
		break;
	default:
		m->irq_state[M6801_IRQ1_LINE] = (uint8_t)(hold ? M6801_HOLD_LINE : state);
		break;
	}
}

void m6801_set_reset(m6801 *m, int state)
{
	// as MAME: the CPU stops while RESET is held and restarts from the reset vector on release
	if (!state && m->reset_held)
		m6801_reset(m);
	m->reset_held = state != 0;
}
