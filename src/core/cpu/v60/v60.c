/*  NEC V60 CPU: ported to C from MAME 0.289 src/devices/cpu/v60/v60.cpp, v60.h (BSD-3-Clause, copyright-holders:
    Farfetch'd, R. Belmont). The instruction and addressing mode functions are MAME's .hxx files converted by
    tools/v60conv.py (v60_ops.inc). See v60.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../cheat_cpu.h"
#include "v60.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

v60_bus_t v60_bus;

// ---- the CPU's state (MAME v60_device members; the hot ones in one struct: the JIT's generated code reaches them from
// one base register) ----
static struct v60_core {
	uint32_t reg[68];
	struct { uint8_t CY, OV, S, Z; } flags;
	int icount;
	uint32_t ppc;
	uint32_t touch;   // bumped by every write and every handler access (the polling loop check: an iteration that touched neither)
} s_v;
#define m_reg s_v.reg
#define m_flags s_v.flags
#define m_icount s_v.icount
#define m_PPC s_v.ppc
#define s_touch s_v.touch
static uint8_t m_irq_line, m_nmi_line;
static int m_stall_io;
static int s_stolen, s_executing, s_req;

// the decoder's scratch (one struct: the polling loop check compares it too)
static struct v60_scratch {
	uint32_t op1, op2, lenop1, lenop2, bamoffset1, bamoffset2, amout, bamoffset, amlength1, amlength2, modadd,
	    modwritevalw;
	uint16_t modwritevalh;
	uint8_t flag1, flag2, instflags, subop, amflag, modm, modval, modval2, modwritevalb, moddim;
} s_scr;
#define m_op1 s_scr.op1
#define m_op2 s_scr.op2
#define m_flag1 s_scr.flag1
#define m_flag2 s_scr.flag2
#define m_instflags s_scr.instflags
#define m_lenop1 s_scr.lenop1
#define m_lenop2 s_scr.lenop2
#define m_subop s_scr.subop
#define m_bamoffset1 s_scr.bamoffset1
#define m_bamoffset2 s_scr.bamoffset2
#define m_amflag s_scr.amflag
#define m_amout s_scr.amout
#define m_bamoffset s_scr.bamoffset
#define m_amlength1 s_scr.amlength1
#define m_amlength2 s_scr.amlength2
#define m_modadd s_scr.modadd
#define m_modm s_scr.modm
#define m_modval s_scr.modval
#define m_modval2 s_scr.modval2
#define m_modwritevalb s_scr.modwritevalb
#define m_modwritevalh s_scr.modwritevalh
#define m_modwritevalw s_scr.modwritevalw
#define m_moddim s_scr.moddim

typedef uint32_t (*am_func)(void);
typedef uint32_t (*op6_func)(int reg);

// MAME's fatalerror stops the machine: here the instruction does not advance (the CPU stays on it); reported once
static int s_fatal_reported;
static uint32_t v60_fatal(const char *fmt, ...)
{
	if (!s_fatal_reported) {
		s_fatal_reported = 1;
		va_list ap;
		va_start(ap, fmt);
		fprintf(stderr, "v60: ");
		vfprintf(stderr, fmt, ap);
		va_end(ap);
	}
	return 0;
}
static void v60_log(const char *fmt, ...) { (void)fmt; }
#define assert(x) ((void)0)

// ---- the bus (MAME memory_access<24, 1, 0, ENDIANNESS_LITTLE>: native 16-bit accesses) ----
static inline uint16_t r16n(uint32_t a, uint16_t mask)
{
	a &= v60_bus.amask & ~1u;
	const uint8_t *p = v60_bus.rd[a >> V60_PAGE_SHIFT];
	if (p) {
		uint16_t v;
		memcpy(&v, p + (a & 0xfff), 2);
		return v;
	}
	s_touch++;
	return v60_bus.read16(a, mask);
}
static inline void w16n(uint32_t a, uint16_t d, uint16_t mask)
{
	s_touch++;
	a &= v60_bus.amask & ~1u;
	uint8_t *p = v60_bus.wr[a >> V60_PAGE_SHIFT];
	if (p) {
		p += a & 0xfff;
		if (mask == 0xffff)
			memcpy(p, &d, 2);
		else if (mask & 0xff)
			p[0] = (uint8_t)d;
		else
			p[1] = (uint8_t)(d >> 8);
		return;
	}
	v60_bus.write16(a, d, mask);
}

static inline uint8_t V60_RB(uint32_t a)
{
	a &= v60_bus.amask;
	const uint8_t *p = v60_bus.rd[a >> V60_PAGE_SHIFT];
	if (p)
		return p[a & 0xfff];
	s_touch++;
	return (a & 1) ? (uint8_t)(v60_bus.read16(a & ~1u, 0xff00) >> 8) : (uint8_t)v60_bus.read16(a, 0x00ff);
}
static inline uint16_t V60_RW(uint32_t a)
{
	a &= v60_bus.amask;
	if (!(a & 1))
		return r16n(a, 0xffff);
	const uint8_t *p = v60_bus.rd[a >> V60_PAGE_SHIFT];
	if (p && (a & 0xfff) != 0xfff)
		return (uint16_t)(p[a & 0xfff] | (p[(a & 0xfff) + 1] << 8));
	return (uint16_t)((r16n(a - 1, 0xff00) >> 8) | ((r16n(a + 1, 0x00ff) & 0xff) << 8));
}
static inline uint32_t V60_RD(uint32_t a)
{
	a &= v60_bus.amask;
	const uint8_t *p = v60_bus.rd[a >> V60_PAGE_SHIFT];
	if (p && (a & 0xfff) <= 0xffc) {
		uint32_t v;
		memcpy(&v, p + (a & 0xfff), 4);
		return v;
	}
	if (!(a & 1))
		return r16n(a, 0xffff) | ((uint32_t)r16n(a + 2, 0xffff) << 16);
	return (r16n(a - 1, 0xff00) >> 8) | ((uint32_t)r16n(a + 1, 0xffff) << 8) | ((uint32_t)(r16n(a + 3, 0x00ff) & 0xff) << 24);
}
static inline uint64_t V60_RQ(uint32_t a) { return V60_RD(a) | ((uint64_t)V60_RD(a + 4) << 32); }

static inline void V60_WB(uint32_t a, uint8_t d)
{
	s_touch++;
	a &= v60_bus.amask;
	uint8_t *p = v60_bus.wr[a >> V60_PAGE_SHIFT];
	if (p) {
		p[a & 0xfff] = d;
		return;
	}
	if (a & 1)
		v60_bus.write16(a & ~1u, (uint16_t)(d << 8), 0xff00);
	else
		v60_bus.write16(a, d, 0x00ff);
}
static inline void V60_WW(uint32_t a, uint16_t d)
{
	a &= v60_bus.amask;
	if (!(a & 1)) {
		w16n(a, d, 0xffff);
		return;
	}
	w16n(a - 1, (uint16_t)(d << 8), 0xff00);
	w16n(a + 1, (uint16_t)(d >> 8), 0x00ff);
}
static inline void V60_WD(uint32_t a, uint32_t d)
{
	s_touch++;
	a &= v60_bus.amask;
	uint8_t *p = v60_bus.wr[a >> V60_PAGE_SHIFT];
	if (p && (a & 0xfff) <= 0xffc) {
		memcpy(p + (a & 0xfff), &d, 4);
		return;
	}
	if (!(a & 1)) {
		w16n(a, (uint16_t)d, 0xffff);
		w16n(a + 2, (uint16_t)(d >> 16), 0xffff);
		return;
	}
	w16n(a - 1, (uint16_t)(d << 8), 0xff00);
	w16n(a + 1, (uint16_t)(d >> 8), 0xffff);
	w16n(a + 3, (uint16_t)(d >> 24), 0x00ff);
}
static inline void V60_WQ(uint32_t a, uint64_t d)
{
	V60_WD(a, (uint32_t)d);
	V60_WD(a + 4, (uint32_t)(d >> 32));
}

// the I/O space (nothing on the boards here: reads 0)
#define V60_IO_RB(a) ((uint8_t)0)
#define V60_IO_RW(a) ((uint16_t)0)
#define V60_IO_RD(a) ((uint32_t)0)
#define V60_IO_WB(a, d) ((void)(a), (void)(d))
#define V60_IO_WW(a, d) ((void)(a), (void)(d))
#define V60_IO_WD(a, d) ((void)(a), (void)(d))

static int s_maps_changed = 1;   // (the JIT: its blocks are made again for the new maps)
void v60_bus_clear(void)
{
	memset(v60_bus.rd, 0, sizeof(v60_bus.rd));
	memset(v60_bus.wr, 0, sizeof(v60_bus.wr));
	v60_bus.amask = 0xffffff;
	s_maps_changed = 1;
}

void v60_bus_set_bits(int bits)
{
	v60_bus.amask = bits >= 26 ? 0x3ffffffu : (1u << bits) - 1;
	s_maps_changed = 1;
}

void v60_bus_map(uint32_t start, uint32_t end, void *mem, uint32_t size, int readable, int writable)
{
	s_maps_changed = 1;
	for (uint32_t a = start; a <= end && a <= v60_bus.amask; a += 1u << V60_PAGE_SHIFT) {
		uint8_t *p = (uint8_t *)mem + ((a - start) & (size - 1));
		v60_bus.rd[a >> V60_PAGE_SHIFT] = readable ? p : NULL;
		v60_bus.wr[a >> V60_PAGE_SHIFT] = writable ? p : NULL;
	}
}

// ---- MAME emu helpers (eminline.h, bitswap) ----
static inline int64_t mul_32x32(int32_t a, int32_t b) { return (int64_t)a * (int64_t)b; }
static inline uint64_t mulu_32x32(uint32_t a, uint32_t b) { return (uint64_t)a * (uint64_t)b; }
static inline int32_t div_64x32_rem(int64_t a, int32_t b, int32_t *rem)
{
	const int32_t res = (int32_t)(a / b);
	*rem = (int32_t)(a - (int64_t)b * res);
	return res;
}
static inline uint32_t divu_64x32_rem(uint64_t a, uint32_t b, uint32_t *rem)
{
	const uint32_t res = (uint32_t)(a / b);
	*rem = (uint32_t)(a - (uint64_t)b * res);
	return res;
}
static inline uint8_t v60_bitswap8(uint32_t v, int b7, int b6, int b5, int b4, int b3, int b2, int b1, int b0)
{
	return (uint8_t)((((v >> b7) & 1) << 7) | (((v >> b6) & 1) << 6) | (((v >> b5) & 1) << 5) | (((v >> b4) & 1) << 4) |
	                 (((v >> b3) & 1) << 3) | (((v >> b2) & 1) << 2) | (((v >> b1) & 1) << 1) | ((v >> b0) & 1));
}
static inline uint32_t swapendian_int32(uint32_t v)
{
	return (v >> 24) | ((v >> 8) & 0xff00) | ((v << 8) & 0xff0000) | (v << 24);
}

// ---- v60.cpp ----
static inline float u2f(uint32_t v) { float f; memcpy(&f, &v, 4); return f; }
static inline uint32_t f2u(float f) { uint32_t v; memcpy(&v, &f, 4); return v; }

// memory accessors
#define OpRead8(a)   V60_RB(a)
#define OpRead16(a)  V60_RW(a)
#define OpRead32(a)  V60_RD(a)

// macros stolen from MAME for flags calc
// note that these types are in x86 naming:
// byte = 8 bit, word = 16 bit, long = 32 bit

// parameter x = result, y = source 1, z = source 2

#define SetOFL_Add(x, y,z)  (_OV = (((x) ^ (y)) & ((x) ^ (z)) & 0x80000000) ? 1: 0)
#define SetOFW_Add(x, y,z)  (_OV = (((x) ^ (y)) & ((x) ^ (z)) & 0x8000) ? 1 : 0)
#define SetOFB_Add(x, y,z)  (_OV = (((x) ^ (y)) & ((x) ^ (z)) & 0x80) ? 1 : 0)

#define SetOFL_Sub(x, y,z)  (_OV = (((z) ^ (y)) & ((z) ^ (x)) & 0x80000000) ? 1 : 0)
#define SetOFW_Sub(x, y,z)  (_OV = (((z) ^ (y)) & ((z) ^ (x)) & 0x8000) ? 1 : 0)
#define SetOFB_Sub(x, y,z)  (_OV = (((z) ^ (y)) & ((z) ^ (x)) & 0x80) ? 1 : 0)

#define SetCFB(x)           {_CY = ((x) & 0x100) ? 1 : 0; }
#define SetCFW(x)           {_CY = ((x) & 0x10000) ? 1 : 0; }
#define SetCFL(x)           {_CY = ((x) & (((uint64_t)1) << 32)) ? 1 : 0; }

#define SetSF(x)            (_S = (x))
#define SetZF(x)            (_Z = (x))

#define SetSZPF_Byte(x)     {_Z = ((uint8_t)(x) == 0);  _S = ((x)&0x80) ? 1 : 0; }
#define SetSZPF_Word(x)     {_Z = ((uint16_t)(x) == 0);  _S = ((x)&0x8000) ? 1 : 0; }
#define SetSZPF_Long(x)     {_Z = ((uint32_t)(x) == 0);  _S = ((x)&0x80000000) ? 1 : 0; }

#define ORB(dst, src)       { (dst) |= (src); _OV = 0; SetSZPF_Byte(dst); }
#define ORW(dst, src)       { (dst) |= (src); _OV = 0; SetSZPF_Word(dst); }
#define ORL(dst, src)       { (dst) |= (src); _OV = 0; SetSZPF_Long(dst); }

#define ANDB(dst, src)      { (dst) &= (src); _OV = 0; SetSZPF_Byte(dst); }
#define ANDW(dst, src)      { (dst) &= (src); _OV = 0; SetSZPF_Word(dst); }
#define ANDL(dst, src)      { (dst) &= (src); _OV = 0; SetSZPF_Long(dst); }

#define XORB(dst, src)      { (dst) ^= (src); _OV = 0; SetSZPF_Byte(dst); }
#define XORW(dst, src)      { (dst) ^= (src); _OV = 0; SetSZPF_Word(dst); }
#define XORL(dst, src)      { (dst) ^= (src); _OV = 0; SetSZPF_Long(dst); }

#define SUBB(dst, src, c)   { unsigned res = (dst) - (src) - (c); SetCFB(res); SetOFB_Sub(res, src, dst); SetSZPF_Byte(res); dst = (uint8_t)res; }
#define SUBW(dst, src, c)   { unsigned res = (dst) - (src) - (c); SetCFW(res); SetOFW_Sub(res, src, dst); SetSZPF_Word(res); dst = (uint16_t)res; }
#define SUBL(dst, src, c)   { uint64_t res = (uint64_t)(dst) - (int64_t)(src) - (c); SetCFL(res); SetOFL_Sub(res, src, dst); SetSZPF_Long(res); dst = (uint32_t)res; }

#define ADDB(dst, src, c)   { unsigned res = (dst) + (src) + (c); SetCFB(res); SetOFB_Add(res, src, dst); SetSZPF_Byte(res); dst = (uint8_t)res; }
#define ADDW(dst, src, c)   { unsigned res = (dst) + (src) + (c); SetCFW(res); SetOFW_Add(res, src, dst); SetSZPF_Word(res); dst = (uint16_t)res; }
#define ADDL(dst, src, c)   { uint64_t res = (uint64_t)(dst) + (uint64_t)(src) + (c); SetCFL(res); SetOFL_Add(res, src, dst); SetSZPF_Long(res); dst = (uint32_t)res; }

#define SETREG8(a, b)       (a) = ((a) & ~0xff) | ((b) & 0xff)
#define SETREG16(a, b)      (a) = ((a) & ~0xffff) | ((b) & 0xffff)

#undef _S

#define _CY     m_flags.CY
#define _OV     m_flags.OV
#define _S      m_flags.S
#define _Z      m_flags.Z

// Defines of all v60 register...
#define R0      m_reg[0]
#define R1      m_reg[1]
#define R2      m_reg[2]
#define R3      m_reg[3]
#define R4      m_reg[4]
#define R5      m_reg[5]
#define R6      m_reg[6]
#define R7      m_reg[7]
#define R8      m_reg[8]
#define R9      m_reg[9]
#define R10     m_reg[10]
#define R11     m_reg[11]
#define R12     m_reg[12]
#define R13     m_reg[13]
#define R14     m_reg[14]
#define R15     m_reg[15]
#define R16     m_reg[16]
#define R17     m_reg[17]
#define R18     m_reg[18]
#define R19     m_reg[19]
#define R20     m_reg[20]
#define R21     m_reg[21]
#define R22     m_reg[22]
#define R23     m_reg[23]
#define R24     m_reg[24]
#define R25     m_reg[25]
#define R26     m_reg[26]
#define R27     m_reg[27]
#define R28     m_reg[28]
#define AP      m_reg[29]
#define FP      m_reg[30]
#define SP      m_reg[31]

#define PC      m_reg[32]
#define PSW     m_reg[33]

// Privileged registers
#define ISP     m_reg[36]
#define L0SP    m_reg[37]
#define L1SP    m_reg[38]
#define L2SP    m_reg[39]
#define L3SP    m_reg[40]
#define SBR     m_reg[41]
#define TR      m_reg[42]
#define SYCW    m_reg[43]
#define TKCW    m_reg[44]
#define PIR     m_reg[45]
//10-14 reserved
#define PSW2    m_reg[51]
#define ATBR0   m_reg[52]
#define ATLR0   m_reg[53]
#define ATBR1   m_reg[54]
#define ATLR1   m_reg[55]
#define ATBR2   m_reg[56]
#define ATLR2   m_reg[57]
#define ATBR3   m_reg[58]
#define ATLR3   m_reg[59]
#define TRMODE  m_reg[60]
#define ADTR0   m_reg[61]
#define ADTR1   m_reg[62]
#define ADTMR0  m_reg[63]
#define ADTMR1  m_reg[64]
//29-31 reserved

// Defines...
#define NORMALIZEFLAGS() \
{ \
	_S    = _S  ? 1 : 0; \
	_OV   = _OV ? 1 : 0; \
	_Z    = _Z  ? 1 : 0; \
	_CY   = _CY ? 1 : 0; \
}

static inline void v60SaveStack(void)
{
	if (PSW & 0x10000000)
		ISP = SP;
	else
		m_reg[37 + ((PSW >> 24) & 3)] = SP;
}

static inline void v60ReloadStack(void)
{
	if (PSW & 0x10000000)
		SP = ISP;
	else
		SP = m_reg[37 + ((PSW >> 24) & 3)];
}

static inline uint32_t v60ReadPSW(void)
{
	PSW &= 0xfffffff0;
	PSW |= (_Z?1:0) | (_S?2:0) | (_OV?4:0) | (_CY?8:0);
	return PSW;
}

static inline void v60WritePSW(uint32_t newval)
{
	/* determine if we need to save / restore the stacks */
	int updateStack = 0;

	/* if the interrupt state is changing, we definitely need to update */
	if ((newval ^ PSW) & 0x10000000)
		updateStack = 1;

	/* if we are not in interrupt mode and the level is changing, we also must update */
	else if (!(PSW & 0x10000000) && ((newval ^ PSW) & 0x03000000))
		updateStack = 1;

	/* save the previous stack value */
	if (updateStack)
		v60SaveStack();

	/* set the new value and update the flags */
	PSW = newval;
	_Z =  (uint8_t)(PSW & 1);
	_S =  (uint8_t)(PSW & 2);
	_OV = (uint8_t)(PSW & 4);
	_CY = (uint8_t)(PSW & 8);

	/* fetch the new stack value */
	if (updateStack)
		v60ReloadStack();
}

static inline uint32_t v60_update_psw_for_exception(int is_interrupt, int target_level)
{
	uint32_t oldPSW = v60ReadPSW();
	uint32_t newPSW = oldPSW;

	// Change to interrupt context
	newPSW &= ~(3 << 24);  // PSW.EL = 0
	newPSW |= target_level << 24; // set target level
	newPSW &= ~(1 << 18);  // PSW.IE = 0
	newPSW &= ~(1 << 16);  // PSW.TE = 0
	newPSW &= ~(1 << 27);  // PSW.TP = 0
	newPSW &= ~(1 << 17);  // PSW.AE = 0
	newPSW &= ~(1 << 29);  // PSW.EM = 0
	if (is_interrupt)
		newPSW |=  (1 << 28);// PSW.IS = 1
	newPSW |=  (1u << 31);  // PSW.ASA = 1
	v60WritePSW(newPSW);

	return oldPSW;
}

#define GETINTVECT(nint)                    V60_RD((SBR & ~0xfff) + (nint) * 4)
#define EXCEPTION_CODE_AND_SIZE(code, size) (((code) << 16) | (size))

#if defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wunused-function"   // (MAME's unused error handlers)
#endif
#include "v60_ops.inc"

// ---- device ----
// (the cheats' access to this CPU's program space: cheat_cpu.h)
static uint8_t cheat_rd(void *x, uint32_t a) { (void)x; return V60_RB(a); }
static void cheat_wr(void *x, uint32_t a, uint8_t v) { (void)x; V60_WB(a, v); }

void v60_init(void)
{
	rbx_cheat_offer("v60", cheat_rd, cheat_wr, NULL, 0, 1);
	m_stall_io = 0;
	m_irq_line = 0;
	m_nmi_line = 0;
	for (int i = 0; i < 68; i++)
		if (i != 31 && i != 32 && i != 33 && i != 41 && i != 43 && i != 44 && i != 45 && i != 51)
			m_reg[i] = 0;
	m_reg[31] = m_reg[32] = m_reg[33] = m_reg[41] = m_reg[43] = m_reg[44] = m_reg[51] = 0;
	m_reg[45] = 0x00006000;   // PIR of the V60
	memset(&m_flags, 0, sizeof(m_flags));
	m_op1 = m_op2 = 0;
	m_flag1 = m_flag2 = 0;
	m_instflags = 0;
	m_lenop1 = m_lenop2 = 0;
	m_subop = 0;
	m_bamoffset1 = m_bamoffset2 = 0;
	m_amflag = 0;
	m_amout = 0;
	m_bamoffset = 0;
	m_amlength1 = m_amlength2 = 0;
	m_modadd = 0;
	m_modm = m_modval = m_modval2 = 0;
	m_modwritevalb = 0;
	m_modwritevalh = 0;
	m_modwritevalw = 0;
	m_moddim = 0;
	m_PPC = 0;
	m_icount = 0;
	s_fatal_reported = 0;
	s_stolen = 0;
	s_executing = 0;
}

void v60_set_pir(uint32_t pir) { m_reg[45] = pir; }

void v60_reset(void)
{
	PSW   = 0x10000000;
	PC    = 0xfffffff0;
	SBR   = 0x00000000;
	SYCW  = 0x00000070;
	TKCW  = 0x0000e000;
	PSW2  = 0x0000f002;

	_CY   = 0;
	_OV   = 0;
	_S    = 0;
	_Z    = 0;
}

static void v60_do_irq(int vector)
{
	uint32_t oldPSW = v60_update_psw_for_exception(1, 0);

	// Push PC and PSW onto the stack
	SP-=4;
	V60_WD(SP, oldPSW);
	SP-=4;
	V60_WD(SP, PC);

	// Jump to vector for user interrupt
	PC = GETINTVECT(vector);
}

static void v60_try_irq(void)
{
	if (m_irq_line == 0)
		return;
	if ((PSW & (1 << 18)) != 0) {
		int vector;
		if (m_irq_line != 1)
			m_irq_line = 0;

		vector = v60_bus.irq_cycle ? v60_bus.irq_cycle() : 0;

		v60_do_irq(vector + 0x40);
	}
}

void v60_set_nmi_line(int state)
{
	if (state) {
		if (m_nmi_line == 0) {
			m_nmi_line = 1;
			v60_do_irq(2);
		}
	} else
		m_nmi_line = 0;
}

void v60_set_irq_line(int state)
{
	m_irq_line = (uint8_t)state;
	v60_try_irq();
}

// ---- polling loops (not MAME's: a speed-up with the same results) ----
// Inside a slice only the V60 changes memory. At a backward branch to T: when the CPU is back at T with the same
// registers, flags and decoder scratch as at its last visit, and touched no handler and wrote nothing in between,
// every further iteration is the same: whole iterations are skipped (their cycles taken), the last ones run.
static struct {
	uint32_t pc, touch;
	int icount, fails, cool;
	uint32_t reg[68];
	uint8_t flags[4];
	struct v60_scratch scr;
} s_spin;

static inline int spin_same(void)
{
	return !memcmp(s_spin.reg, m_reg, sizeof(m_reg)) && !memcmp(s_spin.flags, &m_flags, 4) &&
	       !memcmp(&s_spin.scr, &s_scr, sizeof(s_scr));
}

// a timeout loop: T: [TESTB / TESTH / TESTW <ea>] DBcc rN, T (<ea>: disp[rM], [rM] or rM, M != N), the state the
// same but rN one less: every iteration until rN gets to 0 is the same (the test reads the same byte, the branch is
// taken), so whole iterations are skipped with rN counted down
static int spin_counter(void)
{
	const uint32_t t = PC;
	uint32_t db = t, len = 0;
	int ea_reg = -1;
	const uint8_t op = OpRead8(t);
	if (op >= 0xf0 && op <= 0xf5) {
		const uint8_t am = OpRead8(t + 1);
		const int g = am >> 5;
		ea_reg = am & 0x1f;
		if (!(op & 1))
			len = g == 0 ? 3 : g == 1 ? 4 : g == 2 ? 6 : g == 3 ? 2 : 0;
		else
			len = g == 3 ? 2 : 0;
		if (!len)
			return 0;
		db = t + len;
	}
	const uint8_t dbop = OpRead8(db), dbr = OpRead8(db + 1);
	if ((dbop != 0xc6 && dbop != 0xc7) || (dbop == 0xc7 && (dbr >> 5) == 5) || m_PPC != db ||
	    (uint16_t)OpRead16(db + 2) != (uint16_t)(0u - len))
		return 0;
	const int r = dbr & 0x1f;
	if (r == ea_reg || s_spin.reg[r] - m_reg[r] != 1)
		return 0;
	const uint32_t keep = m_reg[r];
	m_reg[r] = s_spin.reg[r];
	const int same = spin_same();
	m_reg[r] = keep;
	if (!same)
		return 0;
	const int c = s_spin.icount - m_icount;
	if (m_icount > c) {
		uint32_t q = (uint32_t)((m_icount - 1) / c);
		if (q > m_reg[r] - 1)
			q = m_reg[r] - 1;
		m_icount -= (int)q * c;
		m_reg[r] -= q;
	}
	s_spin.icount = m_icount;
	s_spin.reg[r] = m_reg[r] + 1;   // (the next visit: one less again)
	return 1;
}

static int spin_check(void)   // (1: the state was the same as at the last visit)
{
	if (PC != s_spin.pc) {
		s_spin.pc = PC;
		s_spin.fails = s_spin.cool = 0;
	} else if (s_spin.cool) {
		s_spin.cool--;
		return 0;
	} else if (s_spin.touch == s_touch && s_spin.icount > m_icount && spin_same()) {
		const int c = s_spin.icount - m_icount;
		if (m_icount > c)
			m_icount -= (m_icount - 1) / c * c;
		s_spin.icount = m_icount;
		return 1;
	} else if (s_spin.touch == s_touch && s_spin.icount > m_icount && spin_counter()) {
		return 1;
	} else if (++s_spin.fails >= 8) {   // (a loop that changes something: checked again later)
		s_spin.fails = 0;
		s_spin.cool = 256;
		return 0;
	}
	s_spin.touch = s_touch;
	s_spin.icount = m_icount;
	memcpy(s_spin.reg, m_reg, sizeof(m_reg));
	memcpy(s_spin.flags, &m_flags, 4);
	memcpy(&s_spin.scr, &s_scr, sizeof(s_scr));
	return 0;
}

#include "v60_jit.inc"

#if defined(__linux__)
// RBX_V60_TIME=1 (the PC / qemu harness): the time in v60_execute, printed at exit
#include <stdlib.h>
#include <time.h>
static int s_time_on = -1;
static uint64_t s_time_us, s_time_n;
static uint64_t v60_now_us(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}
static void v60_time_print(void)
{
	fprintf(stderr, "v60 time: %.1f ms in %llu slices\n", (double)s_time_us / 1000.0, (unsigned long long)s_time_n);
}
static int v60_execute_(int cycles);
int v60_execute(int cycles)
{
	if (s_time_on < 0) {
		s_time_on = getenv("RBX_V60_TIME") != NULL;
		if (s_time_on)
			atexit(v60_time_print);
	}
	if (!s_time_on)
		return v60_execute_(cycles);
	const uint64_t t = v60_now_us();
	const int r = v60_execute_(cycles);
	s_time_us += v60_now_us() - t;
	s_time_n++;
	return r;
}
#define v60_execute v60_execute_
#endif

int v60_execute(int cycles)
{
	m_icount = cycles;
	s_req = cycles;
	s_stolen = 0;
	s_executing = 1;
	if (m_irq_line != 0)
		v60_try_irq();

	s_spin.pc = ~0u;   // (memory may have changed outside the slice)
#ifdef V60_JIT
	if (vj_on())
		vj_run();
	else
#endif
	while (m_icount > 0)
	{
		uint32_t inc;
		m_PPC = PC;
		m_icount -= 8;  /* fix me -- this is just an average */
		inc = s_OpCodeTable[OpRead8(PC)]();
		PC += inc;
		if (m_irq_line != 0)
			v60_try_irq();
		if (PC <= m_PPC && m_PPC - PC < 256)   // (a backward branch)
			spin_check();
	}
	s_executing = 0;
	return cycles - m_icount - s_stolen;
}

void v60_abort_timeslice(void)
{
	if (!s_executing)
		return;
	const int delta = m_icount;
	s_stolen += delta;
	m_icount -= delta;
}

void v60_release(void)
{
#ifdef V60_JIT
	vj_release();
#endif
}

int v60_executing(void) { return s_executing; }
int v60_cycles_left(void) { return m_icount; }
int v60_cycles_run(void) { return s_executing ? s_req - m_icount - s_stolen : 0; }
uint32_t v60_pc(void) { return PC; }
uint32_t v60_ppc(void) { return m_PPC; }

void v60_get_state(v60_state_t *st)
{
	memset(st, 0, sizeof(*st));
	memcpy(st->reg, m_reg, sizeof(m_reg));
	st->cy = m_flags.CY;
	st->ov = m_flags.OV;
	st->s = m_flags.S;
	st->z = m_flags.Z;
	st->irq_line = m_irq_line;
	st->nmi_line = m_nmi_line;
	st->stall_io = (uint8_t)m_stall_io;
	st->ppc = m_PPC;
}

void v60_set_state(const v60_state_t *st)
{
	memcpy(m_reg, st->reg, sizeof(m_reg));
	m_flags.CY = st->cy;
	m_flags.OV = st->ov;
	m_flags.S = st->s;
	m_flags.Z = st->z;
	m_irq_line = st->irq_line;
	m_nmi_line = st->nmi_line;
	m_stall_io = st->stall_io;
	m_PPC = st->ppc;
}
