/*  ARM7TDMI CPU (ARMv4T, little endian), ported to C from MAME src/devices/cpu/arm7
    (arm7.cpp, arm7core.h, arm7core.hxx, arm7help.h, arm7ops.cpp; MAME 0.289; BSD-3-Clause,
    copyright-holders: Steve Ellenoff, R. Belmont, Ryan Holtz). Thumb: arm7thumb.inc (arm7thmb.cpp).
    execute_run, arm7_check_irq_state, decodeShift, HandleALU / HandleMul / HandleSMulLong / HandleUMulLong /
    HandleMemSingle / HandleHalfWordDT / HandleSwap / HandlePSRTransfer / HandleMemBlock / HandleBranch /
    HandleCoProc*, the arm7ops_* decoders and the CP15 register callbacks keep MAME's semantics and icount
    deductions one to one, quirks included (examples: the ALU uses R15 + 8 for Rn even with a register-specified
    shift; MSR/MRS on the SPSR in USR / SYS mode use R0, as MAME's register table maps it; LDRH/LDRSH into R15 load
    value + 8; SMLAWy stores the product; the ARMv5 encodings MAME's ARM7 decoder accepts (BLX, CLZ, QADD.., SMLAxy..,
    LDRD/STRD, Thumb BLX) run as in MAME).
    ReBOX: the registers of the current mode are kept in r[0..15] (MAME looks each access up through
    sRegisterTable); the bank is switched where the CPSR mode changes. Memory goes through a two-level table of 1 KB
    pages, the rest to callbacks.
    Not ported: the MMU / TLB / FCSE translation and the data / prefetch aborts (CP15 writes are stored and ignored;
    fetches and accesses are never translated), the instruction prefetch buffer (instructions are read when they
    run, which differs only for code that rewrites the next two words), 26-bit mode (MAME's ARM7 has no
    ARCHFLAG_MODE26, so set_cpsr always forces bit 4), the ARM9 (archRev >= 5) cond = NV table, the disassembler,
    the debugger and the DRC. Invalid CPSR modes (4-6, 8-a, c-e) use the user bank (MAME maps every register of
    them to R0). The Thumb encodings MAME stops on with fatalerror only advance the PC.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../cheat_cpu.h"
#include "arm7.h"

#include <stdlib.h>
#include <string.h>

#if defined(__GNUC__)
#define A7_INLINE static inline __attribute__((always_inline))
#define LIKELY(x) __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)
#else
#define A7_INLINE static inline
#define LIKELY(x) (x)
#define UNLIKELY(x) (x)
#endif

#define N_BIT 31
#define Z_BIT 30
#define C_BIT 29
#define V_BIT 28
#define N_MASK 0x80000000u
#define Z_MASK 0x40000000u
#define C_MASK 0x20000000u
#define V_MASK 0x10000000u
#define Q_MASK 0x08000000u
#define I_MASK 0x00000080u
#define F_MASK 0x00000040u
#define T_MASK 0x00000020u
#define SIGN_BIT 0x80000000u
#define MODE_FLAG 0xfu

#define INSN_SDT_L 0x00100000u
#define INSN_SDT_W 0x00200000u
#define INSN_SDT_B 0x00400000u
#define INSN_SDT_U 0x00800000u
#define INSN_SDT_P 0x01000000u
#define INSN_BDT_L 0x00100000u
#define INSN_BDT_W 0x00200000u
#define INSN_BDT_S 0x00400000u
#define INSN_BDT_U 0x00800000u
#define INSN_BDT_P 0x01000000u
#define INSN_MUL_A 0x00200000u
#define INSN_I     0x02000000u
#define INSN_S     0x00100000u
#define INSN_BL    0x01000000u

#define R15   (c->r[15])
#define CPSR  (c->cpsr)
#define MODE  (c->cpsr & MODE_FLAG)
#define ICOUNT (c->icount)

// MAME ARM9_COPRO_ID_MFR_ARM | ARM9_COPRO_ID_ARCH_V4T | ARM9_COPRO_ID_PART_GENERICARM7
#define COPRO_ID 0x41027000u
#define COPRO_CTRL_MASK 0x0000338fu

/* ---- register bank (MAME sRegisterTable, R8-R14 and the SPSR) ---- */

enum {
	eR8 = 8, eR9, eR10, eR11, eR12, eR13, eR14,
	eR8_FIQ = 17, eR9_FIQ, eR10_FIQ, eR11_FIQ, eR12_FIQ, eR13_FIQ, eR14_FIQ, eSPSR_FIQ,
	eR13_IRQ, eR14_IRQ, eSPSR_IRQ,
	eR13_SVC, eR14_SVC, eSPSR_SVC,
	eR13_ABT, eR14_ABT, eSPSR_ABT,
	eR13_UND, eR14_UND, eSPSR_UND
};

#define USRB { eR8, eR9, eR10, eR11, eR12, eR13, eR14 }
// m_r[] index of R8..R14 in each mode (invalid modes: the user bank)
static const uint8_t regtab[16][7] = {
	USRB,                                                                   // USR
	{ eR8_FIQ, eR9_FIQ, eR10_FIQ, eR11_FIQ, eR12_FIQ, eR13_FIQ, eR14_FIQ }, // FIQ
	{ eR8, eR9, eR10, eR11, eR12, eR13_IRQ, eR14_IRQ },                     // IRQ
	{ eR8, eR9, eR10, eR11, eR12, eR13_SVC, eR14_SVC },                     // SVC
	USRB, USRB, USRB,
	{ eR8, eR9, eR10, eR11, eR12, eR13_ABT, eR14_ABT },                     // ABT
	USRB, USRB, USRB,
	{ eR8, eR9, eR10, eR11, eR12, eR13_UND, eR14_UND },                     // UND
	USRB, USRB, USRB,
	USRB                                                                    // SYS
};
// m_r[] index of the SPSR in each mode, 0 = MAME's table entry 0 (R0: USR, SYS and invalid modes)
static const uint8_t spsrtab[16] = { 0, eSPSR_FIQ, eSPSR_IRQ, eSPSR_SVC, 0, 0, 0, eSPSR_ABT, 0, 0, 0, eSPSR_UND, 0, 0, 0, 0 };

static void rebank(arm7_t *c, uint32_t o, uint32_t m)
{
	for (int i = 0; i < 7; i++) {
		const uint32_t oi = regtab[o][i], ni = regtab[m][i];
		if (oi != ni) {
			c->bank[oi] = c->r[8 + i];
			c->r[8 + i] = c->bank[ni];
		}
	}
}

// MAME set_cpsr (no ARCHFLAG_MODE26: bit 4 forced)
A7_INLINE void set_cpsr(arm7_t *c, uint32_t val)
{
	const uint32_t old = c->cpsr & MODE_FLAG;
	val |= 0x10;
	c->cpsr = val;
	if (UNLIKELY((val & MODE_FLAG) != old))
		rebank(c, old, val & MODE_FLAG);
}

// MAME SwitchMode
A7_INLINE void switch_mode(arm7_t *c, uint32_t mode) { set_cpsr(c, (c->cpsr & ~MODE_FLAG) | mode); }

// MAME GetModeRegister / SetModeRegister: register i of the given mode
A7_INLINE uint32_t *mode_reg(arm7_t *c, uint32_t mode, uint32_t i)
{
	if (i < 8 || i == 15)
		return &c->r[i];
	const uint32_t idx = regtab[mode & MODE_FLAG][i - 8];
	return idx == regtab[c->cpsr & MODE_FLAG][i - 8] ? &c->r[i] : &c->bank[idx];
}

// MAME GetRegister(SPSR) / SetRegister(SPSR)
A7_INLINE uint32_t *spsr_reg(arm7_t *c)
{
	const uint32_t idx = spsrtab[c->cpsr & MODE_FLAG];
	return idx ? &c->bank[idx] : &c->r[0];
}

/* ---- memory (MAME arm7_cpu_read32/16/8, arm7_cpu_write32/16/8, the opcode cache) ---- */

A7_INLINE uint32_t rotr32(uint32_t v, uint32_t n) { n &= 31; return n ? (v >> n) | (v << (32 - n)) : v; }

A7_INLINE uint8_t *page_r(const arm7_t *c, uint32_t a)
{
	const uint32_t t = c->l1[a >> ARM7_L1_SHIFT];
	return t ? c->l2[t - 1].r[(a >> ARM7_PAGE_SHIFT) & (ARM7_L2_SIZE - 1)] : NULL;
}

A7_INLINE uint8_t *page_w(const arm7_t *c, uint32_t a)
{
	const uint32_t t = c->l1[a >> ARM7_L1_SHIFT];
	return t ? c->l2[t - 1].w[(a >> ARM7_PAGE_SHIFT) & (ARM7_L2_SIZE - 1)] : NULL;
}

#define POFS(a) ((a) & ((1u << ARM7_PAGE_SHIFT) - 1))

A7_INLINE uint32_t RD32(arm7_t *c, uint32_t addr)
{
	const uint32_t a = addr & ~3u;
	const uint8_t *p = page_r(c, a);
	uint32_t v;
	if (LIKELY(p != NULL))
		memcpy(&v, p + POFS(a), 4);
	else
		v = c->read32(c->user, a);
	if (UNLIKELY(addr & 3))
		v = rotr32(v, 8 * (addr & 3));
	return v;
}

A7_INLINE uint32_t RD16(arm7_t *c, uint32_t addr)
{
	const uint32_t a = addr & ~1u;
	const uint8_t *p = page_r(c, a);
	uint16_t h;
	if (LIKELY(p != NULL))
		memcpy(&h, p + POFS(a), 2);
	else
		h = c->read16(c->user, a);
	uint32_t v = h;
	if (UNLIKELY(addr & 1))
		v = ((v >> 8) & 0xff) | ((v & 0xff) << 24);
	return v;
}

A7_INLINE uint32_t RD8(arm7_t *c, uint32_t addr)
{
	const uint8_t *p = page_r(c, addr);
	if (LIKELY(p != NULL))
		return p[POFS(addr)];
	return c->read8(c->user, addr);
}

A7_INLINE void WR32(arm7_t *c, uint32_t addr, uint32_t v)
{
	const uint32_t a = addr & ~3u;
	uint8_t *p = page_w(c, a);
	if (LIKELY(p != NULL)) {
		uint32_t o;
		memcpy(&o, p + POFS(a), 4);
		c->writes += o != v;   // (idle_skip: a write of the same value changes nothing)
		memcpy(p + POFS(a), &v, 4);
	} else {
		c->writes++;
		c->write32(c->user, a, v);
	}
}

A7_INLINE void WR16(arm7_t *c, uint32_t addr, uint32_t v)
{
	const uint32_t a = addr & ~1u;
	uint8_t *p = page_w(c, a);
	const uint16_t h = (uint16_t)v;
	if (LIKELY(p != NULL)) {
		uint16_t o;
		memcpy(&o, p + POFS(a), 2);
		c->writes += o != h;
		memcpy(p + POFS(a), &h, 2);
	} else {
		c->writes++;
		c->write16(c->user, a, h);
	}
}

A7_INLINE void WR8(arm7_t *c, uint32_t addr, uint32_t v)
{
	uint8_t *p = page_w(c, addr);
	if (LIKELY(p != NULL)) {
		c->writes += p[POFS(addr)] != (uint8_t)v;
		p[POFS(addr)] = (uint8_t)v;
	} else {
		c->writes++;
		c->write8(c->user, addr, (uint8_t)v);
	}
}

// opcode fetch (MAME m_pr32 through the program cache): ARM at pc & ~3, Thumb the halfword of the word at pc & ~3
A7_INLINE uint32_t FETCH32(arm7_t *c, uint32_t a)
{
	const uint8_t *p = page_r(c, a);
	uint32_t v;
	if (LIKELY(p != NULL))
		memcpy(&v, p + POFS(a), 4);
	else
		v = c->read32(c->user, a);
	return v;
}

A7_INLINE uint32_t FETCH16(arm7_t *c, uint32_t a)
{
	const uint8_t *p = page_r(c, a);
	if (LIKELY(p != NULL)) {
		uint16_t h;
		memcpy(&h, p + POFS(a), 2);
		return h;
	}
	return (uint16_t)(c->read32(c->user, a & ~3u) >> ((a & 2) ? 16 : 0));
}

/* ---- interrupts (MAME update_irq_state / arm7_check_irq_state) ---- */

A7_INLINE void update_irq_state(arm7_t *c)
{
	c->pending_interrupt = (uint8_t)(c->pending_und || c->pending_swi || c->pending_fiq || c->pending_irq);
}

// exception priorities (MAME): data abort (not emulated), FIQ, IRQ, prefetch abort (not emulated), undefined, SWI
static void check_irq_state(arm7_t *c)
{
	if (!c->pending_interrupt)
		return;

	const uint32_t cpsr = c->cpsr;       // save current CPSR
	const uint32_t pc = c->r[15] + 4;    // save old pc (already incremented in pipeline)

	if (c->pending_fiq && (cpsr & F_MASK) == 0) {
		if (c->irq_ack)
			c->irq_ack(c->user, ARM7_FIRQ_LINE);
		switch_mode(c, ARM7_MODE_FIQ);
		c->r[14] = pc;
		*spsr_reg(c) = cpsr;
		set_cpsr(c, c->cpsr | I_MASK | F_MASK);
		set_cpsr(c, c->cpsr & ~T_MASK);
		R15 = 0x1c;
		return;
	}

	if (c->pending_irq && (cpsr & I_MASK) == 0) {
		if (c->irq_ack)
			c->irq_ack(c->user, ARM7_IRQ_LINE);
		switch_mode(c, ARM7_MODE_IRQ);
		c->r[14] = pc;
		*spsr_reg(c) = cpsr;
		set_cpsr(c, c->cpsr | I_MASK);
		set_cpsr(c, c->cpsr & ~T_MASK);
		R15 = 0x18;
		return;
	}

	if (c->pending_und) {
		switch_mode(c, ARM7_MODE_UND);
		// compensate for prefetch (MAME)
		c->r[14] = (c->cpsr & T_MASK) ? pc - 4 + 2 : pc - 4 + 4 - 4;
		*spsr_reg(c) = cpsr;
		set_cpsr(c, c->cpsr | I_MASK);
		set_cpsr(c, c->cpsr & ~T_MASK);
		R15 = 0x04;
		c->pending_und = 0;
		update_irq_state(c);
		return;
	}

	if (c->pending_swi) {
		switch_mode(c, ARM7_MODE_SVC);
		c->r[14] = (c->cpsr & T_MASK) ? pc - 4 + 2 : pc - 4 + 4;
		*spsr_reg(c) = cpsr;
		set_cpsr(c, c->cpsr | I_MASK);
		set_cpsr(c, c->cpsr & ~T_MASK);
		R15 = 0x08;
		c->pending_swi = 0;
		update_irq_state(c);
		return;
	}
}

/* ---- CP15 (MAME arm7_rt_r_callback / arm7_rt_w_callback, arm7_do_callback, arm7_dt_*_callback) ---- */

static uint32_t rt_r_callback(arm7_t *c, uint32_t opcode)
{
	const uint32_t cReg = (opcode >> 16) & 0xf;
	const uint32_t op2 = (opcode >> 5) & 7;
	const uint32_t op3 = opcode & 0xf;
	const uint32_t cpnum = (opcode >> 8) & 0xf;
	uint32_t data = 0;

	if (c->arch5) {   // MAME arm946es_cpu_device::arm7_rt_r_callback
		if (cpnum != 15)
			return 0;
		switch (cReg) {
		case 0:
			switch (op2) {
			case 0: data = 0x41059461u; break;   // chip ID
			case 1: data = 0x0f0d2112u; break;   // cache ID
			case 2: data = (6u << 6) | (5u << 18); break;   // TCM size
			}
			break;
		case 1: return c->cp15_control;
		case 9:
			if (op3 == 1)
				return op2 == 0 ? c->cp15_dtcm_reg : c->cp15_itcm_reg;
			break;
		}
		return data;
	}
	if (cpnum != 15) {
		// not XScale: unhandled coprocessor
		c->pending_und = 1;
		update_irq_state(c);
		return 0;
	}
	switch (cReg) {
	case 0: // ID
		switch (op2) {
		case 0: data = COPRO_ID; break;
		case 1: data = 0x0f0d2112u; break; // cache type (MAME's value)
		default: data = 0; break;
		}
		break;
	case 1: data = c->cp15_control | 0x70; break; // bits 4-6 always read back as 1
	case 2: data = c->cp15_tlb_base; break;
	case 3: data = c->cp15_dac; break;
	case 5:
		if (op3 == 0) data = c->cp15_fsr[0];
		else if (op3 == 1) data = c->cp15_fsr[1];
		break;
	case 6: data = c->cp15_far; break;
	case 13: data = c->cp15_pid; break;
	default: break; // 4, 7-12 reserved, 14 breakpoint, 15 test
	}
	return data;
}

static void rt_w_callback(arm7_t *c, uint32_t opcode, uint32_t data)
{
	const uint32_t cReg = (opcode >> 16) & 0xf;
	const uint32_t op3 = opcode & 0xf;
	const uint32_t cpnum = (opcode >> 8) & 0xf;

	if (c->arch5) {   // MAME arm946es_cpu_device::arm7_rt_w_callback (MPU, cache commands: ignored)
		if (cpnum != 15)
			return;
		const uint32_t op2 = (opcode >> 5) & 7;
		if (cReg == 1)
			c->cp15_control = data;
		else if (cReg == 9 && op3 == 1) {
			if (op2 == 0)
				c->cp15_dtcm_reg = data;
			else if (op2 == 1)
				c->cp15_itcm_reg = data;
		}
		if (cReg == 1 || cReg == 9)
			c->tcm_use |= (c->cp15_control & ((1u << 16) | (1u << 18))) != 0;   // (TCM: not emulated, noted)
		return;
	}
	if (cpnum != 15) {
		if (cpnum == 14)
			return; // MAME eats CP14 writes
		c->pending_und = 1;
		update_irq_state(c);
		return;
	}
	switch (cReg) {
	case 1: c->cp15_control = data & COPRO_CTRL_MASK; break; // ReBOX: the MMU enable bit is stored, not acted on
	case 2: c->cp15_tlb_base = data; break;
	case 3: c->cp15_dac = data; break;
	case 5:
		if (op3 == 0) c->cp15_fsr[0] = data;
		else if (op3 == 1) c->cp15_fsr[1] = data;
		break;
	case 6: c->cp15_far = data; break;
	case 13: c->cp15_pid = data; break;
	default: break; // reserved, cache / TLB / read buffer operations, breakpoint, test
	}
}

/* ---- ARM instructions (arm7ops.cpp) ---- */

// MAME decodeShift; pCarry != NULL gets the shifter carry (nonzero = set)
static uint32_t decode_shift(arm7_t *c, uint32_t insn, uint32_t *pCarry)
{
	uint32_t k = (insn >> 7) & 0x1f;
	uint32_t rm = c->r[insn & 0xf];
	const uint32_t t = (insn >> 4) & 7;

	if ((insn & 0xf) == 0xf) {
		// "If a register is used to specify the shift amount the PC will be 12 bytes ahead." (instead of 8)
		rm += (t & 1) ? 12 : 8;
	}

	if (t & 1) {
		// register shift: the bottom 8 bits
		k = c->r[k >> 1] & 0xff;
		if (k == 0) {
			if (pCarry)
				*pCarry = CPSR & C_MASK;
			return rm;
		}
	}
	switch (t >> 1) {
	case 0: // LSL
		if (k >= 32) {
			if (pCarry)
				*pCarry = (k == 32) ? rm & 1 : 0;
			return 0;
		}
		if (pCarry)
			*pCarry = k ? (rm & (1u << (32 - k))) : (CPSR & C_MASK);
		return k ? (rm << k) : rm;

	case 1: // LSR
		if (k == 0 || k == 32) {
			if (pCarry)
				*pCarry = rm & SIGN_BIT;
			return 0;
		} else if (k > 32) {
			if (pCarry)
				*pCarry = 0;
			return 0;
		}
		if (pCarry)
			*pCarry = rm & (1u << (k - 1));
		return rm >> k;

	case 2: // ASR
		if (k == 0 || k > 32)
			k = 32;
		if (pCarry)
			*pCarry = rm & (1u << (k - 1));
		if (k >= 32)
			return (rm & SIGN_BIT) ? 0xffffffffu : 0;
		if (rm & SIGN_BIT)
			return (rm >> k) | (0xffffffffu << (32 - k));
		return rm >> k;

	default: // ROR and RRX
		if (k) {
			k &= 31;
			if (k) {
				if (pCarry)
					*pCarry = rm & (1u << (k - 1));
				return rotr32(rm, k);
			}
			if (pCarry)
				*pCarry = rm & SIGN_BIT;
			return rm;
		}
		if (pCarry)
			*pCarry = rm & 1;
		return (rm >> 1) | ((CPSR & C_MASK) << 2);
	}
}

static int load_inc(arm7_t *c, uint32_t pat, uint32_t rbv, uint32_t mode)
{
	int result = 0;
	rbv &= ~3u;
	for (uint32_t i = 0; i < 16; i++) {
		if ((pat >> i) & 1) {
			rbv += 4;
			const uint32_t data = RD32(c, rbv);
			*mode_reg(c, mode, i) = data; // R15: the full word (MODE32)
			result++;
		}
	}
	return result;
}

static int load_dec(arm7_t *c, uint32_t pat, uint32_t rbv, uint32_t mode)
{
	int result = 0;
	rbv &= ~3u;
	for (int i = 15; i >= 0; i--) {
		if ((pat >> i) & 1) {
			rbv -= 4;
			const uint32_t data = RD32(c, rbv);
			*mode_reg(c, mode, (uint32_t)i) = data;
			result++;
		}
	}
	return result;
}

static int store_inc(arm7_t *c, uint32_t pat, uint32_t rbv, uint32_t mode)
{
	int result = 0;
	for (uint32_t i = 0; i < 16; i++) {
		if ((pat >> i) & 1) {
			rbv += 4;
			WR32(c, rbv, *mode_reg(c, mode, i));
			result++;
		}
	}
	return result;
}

static int store_dec(arm7_t *c, uint32_t pat, uint32_t rbv, uint32_t mode)
{
	int result = 0;
	for (uint32_t p = pat & 0xffff; p; p &= p - 1)
		result++;
	rbv -= (uint32_t)result << 2;
	for (uint32_t i = 0; i <= 15; i++) {
		if ((pat >> i) & 1) {
			WR32(c, rbv, *mode_reg(c, mode, i));
			rbv += 4;
		}
	}
	return result;
}

static void HandleBranch(arm7_t *c, uint32_t insn)
{
	uint32_t off = (insn & 0x00ffffffu) << 2;
	if (insn & INSN_BL)
		c->r[14] = R15 + 4;
	// sign-extend the 26-bit offset
	off = (off & 0x02000000u) ? (off | 0xfc000000u) : off;
	R15 += off + 8;
}

static void HandleMemSingle(arm7_t *c, uint32_t insn)
{
	uint32_t rnv, off;

	if (insn & INSN_I)
		off = decode_shift(c, insn, NULL);
	else
		off = insn & 0xfff;

	const uint32_t rn = (insn >> 16) & 0xf;

	if (insn & INSN_SDT_P) {
		// pre-indexed
		rnv = (insn & INSN_SDT_U) ? c->r[rn] + off : c->r[rn] - off;
		if (insn & INSN_SDT_W)
			c->r[rn] = rnv;
		else if (rn == 15)
			rnv = rnv + 8;
	} else {
		// post-indexed
		rnv = (rn == 15) ? R15 + 8 : c->r[rn];
	}

	const uint32_t rd = (insn >> 12) & 0xf;
	if (insn & INSN_SDT_L) {
		if (insn & INSN_SDT_B) {
			c->r[rd] = RD8(c, rnv);
		} else {
			const uint32_t data = RD32(c, rnv);
			if (rd == 15) {
				R15 = data - 4;
				// LDR, PC takes 2S + 2N + 1I (5 total cycles)
				ICOUNT -= 2;
				if ((data & 1) && c->arch5) {   // (archRev 5: bit 0 enters Thumb)
					set_cpsr(c, CPSR | T_MASK);
					R15--;
				}
			} else {
				c->r[rd] = data;
			}
		}
	} else {
		if (insn & INSN_SDT_B)
			WR8(c, rnv, c->r[rd] & 0xff);
		else
			WR32(c, rnv, rd == 15 ? R15 + 8 + 4 : c->r[rd]); // manual says STR rd = PC, +12
		// store takes only 2 N cycles, so add + 1
		ICOUNT += 1;
	}

	// post-indexing writeback (the loaded value wins when rd == rn)
	if (!(insn & INSN_SDT_P)) {
		if (rd != rn)
			c->r[rn] = (insn & INSN_SDT_U) ? rnv + off : rnv - off;
	}
}

static void HandleHalfWordDT(arm7_t *c, uint32_t insn)
{
	uint32_t rnv, off;

	if (insn & 0x400000)
		off = (((insn >> 8) & 0x0f) << 4) | (insn & 0x0f);
	else
		off = c->r[insn & 0x0f];

	const uint32_t rn = (insn >> 16) & 0xf;

	if (insn & INSN_SDT_P) {
		rnv = (insn & INSN_SDT_U) ? c->r[rn] + off : c->r[rn] - off;
		if (insn & INSN_SDT_W)
			c->r[rn] = rnv;
		else if (rn == 15)
			rnv = rnv + 8;
	} else {
		rnv = (rn == 15) ? R15 + 8 : c->r[rn];
	}

	const uint32_t rd = (insn >> 12) & 0xf;

	if (insn & INSN_SDT_L) {
		uint32_t newval;
		if (insn & 0x40) {
			if (insn & 0x20) {
				// signed halfword (archRev 4: an odd address gives the high byte sign-extended)
				int32_t data = (int32_t)(int16_t)(uint16_t)RD16(c, rnv & ~1u);
				if ((rnv & 1) && !c->arch5)
					data >>= 8;
				newval = (uint32_t)data;
			} else {
				// signed byte
				const uint32_t databyte = RD8(c, rnv) & 0xff;
				const uint32_t signbyte = (databyte & 0x80) ? 0xffffffu : 0;
				newval = (signbyte << 8) | databyte;
			}
		} else {
			newval = RD16(c, rnv);
		}
		if (rd == 15) {
			R15 = newval + 8;
			// LDR(H,SH,SB) PC takes 2S + 2N + 1I (5 total cycles)
			ICOUNT -= 2;
		} else {
			c->r[rd] = newval;
			R15 += 4;
		}
	} else {
		if ((insn & 0x60) == 0x40) { // LDRD (MAME runs it on ARM7 too)
			c->r[rd] = RD32(c, rnv);
			c->r[(rd + 1) & 15] = RD32(c, rnv + 4);
			R15 += 4;
		} else if ((insn & 0x60) == 0x60) { // STRD
			WR32(c, rnv, c->r[rd]);
			WR32(c, rnv + 4, c->r[(rd + 1) & 15]);
			R15 += 4;
		} else {
			WR16(c, rnv, rd == 15 ? R15 + 8 + 4 : c->r[rd]); // manual says STR RD=PC, +12 of address
			R15 += 4;
			// STRH takes 2 cycles, so we add + 1
			ICOUNT += 1;
		}
	}

	if (!(insn & INSN_SDT_P)) {
		if (rd != rn)
			c->r[rn] = (insn & INSN_SDT_U) ? rnv + off : rnv - off;
	}
}

static void HandleSwap(arm7_t *c, uint32_t insn)
{
	const uint32_t rn = c->r[(insn >> 16) & 0xf];
	const uint32_t rm = c->r[insn & 0xf];
	const uint32_t rd = (insn >> 12) & 0xf;
	uint32_t tmp;

	if (insn & 0x400000) {
		tmp = RD8(c, rn);
		WR8(c, rn, rm);
	} else {
		tmp = RD32(c, rn);
		WR32(c, rn, rm);
	}
	c->r[rd] = tmp;
	R15 += 4;
	// 1S+2N+1I cycles
	ICOUNT -= 1;
}

static void HandlePSRTransfer(arm7_t *c, uint32_t insn)
{
	const int spsr = (insn & 0x400000) != 0;
	const uint32_t oldmode = MODE;
	uint32_t newval = spsr ? *spsr_reg(c) : CPSR;

	if (insn & 0x00200000) {
		// MSR
		uint32_t val;
		if (insn & INSN_I) {
			const uint32_t by = (insn >> 8) & 0xf;
			val = by ? rotr32(insn & 0xff, by << 1) : (insn & 0xff);
		} else {
			val = c->r[insn & 0x0f];
		}

		if (!spsr) {
			if (oldmode != ARM7_MODE_USER) {
				if (insn & 0x00010000) newval = (newval & 0xffffff00u) | (val & 0x000000ffu);
				if (insn & 0x00020000) newval = (newval & 0xffff00ffu) | (val & 0x0000ff00u);
				if (insn & 0x00040000) newval = (newval & 0xff00ffffu) | (val & 0x00ff0000u);
			}
			// status flags can be modified regardless of mode (MAME keeps the Q bit writable)
			if (insn & 0x00080000)
				newval = (newval & 0x00ffffffu) | (val & 0xf8000000u);
		} else {
			if (((CPSR & 0x1f) > 0x10) && ((CPSR & 0x1f) < 0x1f)) {
				if (insn & 0x00010000) newval = (newval & 0xffffff00u) | (val & 0xffu);
				if (insn & 0x00020000) newval = (newval & 0xffff00ffu) | (val & 0xff00u);
				if (insn & 0x00040000) newval = (newval & 0xff00ffffu) | (val & 0xff0000u);
				if (insn & 0x00080000) newval = (newval & 0x00ffffffu) | (val & 0xf8000000u);
			}
		}

		if (!spsr)
			set_cpsr(c, newval);
		else
			*spsr_reg(c) = newval;

		if ((newval & MODE_FLAG) != oldmode)
			switch_mode(c, MODE);
	} else {
		// MRS
		c->r[(insn >> 12) & 0x0f] = spsr ? *spsr_reg(c) : CPSR;
	}
}

#define IsNeg(i) ((i) >> 31)
#define IsPos(i) ((~(i)) >> 31)
#define NZ_FLAGS(rd) (((rd) & SIGN_BIT) | ((uint32_t)((rd) == 0) << Z_BIT))
#define DIFFER(a, b) (((a) ^ (b)) >> 31)

A7_INLINE uint32_t add_flags(uint32_t cpsr, uint32_t rd, uint32_t rn, uint32_t op2)
{
	return (cpsr & ~(N_MASK | Z_MASK | V_MASK | C_MASK))
		| ((uint32_t)((!DIFFER(rn, op2)) && DIFFER(rn, rd)) << V_BIT)
		| (((IsNeg(rn) & IsNeg(op2)) | (IsNeg(rn) & IsPos(rd)) | (IsNeg(op2) & IsPos(rd))) ? C_MASK : 0)
		| NZ_FLAGS(rd);
}

A7_INLINE uint32_t sub_flags(uint32_t cpsr, uint32_t rd, uint32_t rn, uint32_t op2)
{
	return (cpsr & ~(N_MASK | Z_MASK | V_MASK | C_MASK))
		| ((uint32_t)(DIFFER(rn, op2) && DIFFER(rn, rd)) << V_BIT)
		| (((IsNeg(rn) & IsPos(op2)) | (IsNeg(rn) & IsPos(rd)) | (IsPos(op2) & IsPos(rd))) ? C_MASK : 0)
		| NZ_FLAGS(rd);
}

static void HandleALU(arm7_t *c, uint32_t insn)
{
	uint32_t op2, sc = 0, rd = 0, rn = 0;
	const uint32_t opcode = (insn >> 21) & 0xf;

	// Normal Data Processing : 1S
	// Data Processing with register specified shift : 1S + 1I
	// Data Processing with PC written : 2S + 1N
	// Data Processing with register specified shift and PC written : 2S + 1N + 1I

	if (insn & INSN_I) {
		const uint32_t by = (insn >> 8) & 0xf;
		if (by) {
			op2 = rotr32(insn & 0xff, by << 1);
			sc = op2 & SIGN_BIT;
		} else {
			op2 = insn & 0xfff;
			sc = CPSR & C_MASK;
		}
	} else {
		op2 = decode_shift(c, insn, (insn & INSN_S) ? &sc : NULL);
		if (!(insn & INSN_S))
			sc = 0;
		// extra cycle (register operand)
		ICOUNT -= 1;
	}

	if ((opcode & 0xd) != 0xd) { // no Rn in MOV / MVN
		const uint32_t rnn = (insn >> 16) & 0xf;
		rn = (rnn == 15) ? R15 + 8 : c->r[rnn];
	}

	const int s = (insn & INSN_S) != 0;
	switch (opcode) {
	case 0x6: // SBC
		rd = rn - op2 - ((CPSR & C_MASK) ? 0 : 1);
		if (s) set_cpsr(c, sub_flags(CPSR, rd, rn, op2));
		break;
	case 0xa: // CMP
	case 0x2: // SUB
		rd = rn - op2;
		if (s) set_cpsr(c, sub_flags(CPSR, rd, rn, op2));
		break;
	case 0x7: // RSC
		rd = op2 - rn - ((CPSR & C_MASK) ? 0 : 1);
		if (s) set_cpsr(c, sub_flags(CPSR, rd, op2, rn));
		break;
	case 0x3: // RSB
		rd = op2 - rn;
		if (s) set_cpsr(c, sub_flags(CPSR, rd, op2, rn));
		break;
	case 0x5: // ADC
		rd = rn + op2 + ((CPSR & C_MASK) >> C_BIT);
		if (s) set_cpsr(c, add_flags(CPSR, rd, rn, op2));
		break;
	case 0xb: // CMN
	case 0x4: // ADD
		rd = rn + op2;
		if (s) set_cpsr(c, add_flags(CPSR, rd, rn, op2));
		break;
	default:
		switch (opcode) {
		case 0x0: case 0x8: rd = rn & op2; break;  // AND, TST
		case 0xe: rd = rn & ~op2; break;           // BIC
		case 0x9: case 0x1: rd = rn ^ op2; break;  // TEQ, EOR
		case 0xc: rd = rn | op2; break;            // ORR
		case 0xd: rd = op2; break;                 // MOV
		default: rd = ~op2; break;                 // MVN
		}
		if (s) set_cpsr(c, (CPSR & ~(N_MASK | Z_MASK | C_MASK)) | NZ_FLAGS(rd) | ((uint32_t)(sc != 0) << C_BIT));
		break;
	}
	R15 += 4; // (in MAME's flag macros)

	const uint32_t rdn = (insn >> 12) & 0xf;
	if ((opcode & 0xc) != 0x8) {
		if (rdn == 15 && !s) {
			// Rd = R15, S clear: result to R15, CPSR not affected
			R15 = rd;
			ICOUNT -= 2;
		} else if (rdn == 15) {
			// Rd = R15, S set: the current mode's SPSR goes to the CPSR (not in user mode)
			if (MODE != ARM7_MODE_USER) {
				set_cpsr(c, *spsr_reg(c));
				switch_mode(c, MODE);
			}
			R15 = rd;
			ICOUNT -= 2;
		} else {
			c->r[rdn] = rd;
		}
	} else if (rdn == 15) {
		// TST class with Rd = R15 (S always set here): MAME puts the result in R15
		if (s)
			R15 = rd;
		ICOUNT -= 2;
	}

	// compensate for the -3 at the end
	ICOUNT += 2;
}

A7_INLINE int mul_cycles(uint32_t rs)
{
	if (rs < 0x00000100u) return 1;
	if (rs < 0x00010000u) return 2;
	if (rs < 0x01000000u) return 3;
	return 4;
}

static void HandleMul(arm7_t *c, uint32_t insn)
{
	const uint32_t rm = c->r[insn & 0xf];
	uint32_t rs = c->r[(insn >> 8) & 0xf];
	uint32_t r = rm * rs;

	if (insn & INSN_MUL_A) {
		r += c->r[(insn >> 12) & 0xf];
		ICOUNT -= 1; // extra cycle for MLA
	}
	c->r[(insn >> 16) & 0xf] = r;
	if (insn & INSN_S)
		set_cpsr(c, (CPSR & ~(N_MASK | Z_MASK)) | NZ_FLAGS(r));

	if (rs & SIGN_BIT)
		rs = 0u - rs;
	ICOUNT -= 1 + mul_cycles(rs);
	ICOUNT += 3;
}

A7_INLINE uint32_t nz_flags64(uint64_t res)
{
	return (uint32_t)((res & ((uint64_t)1 << 63)) >> 32) | ((uint32_t)(res == 0) << Z_BIT);
}

static void HandleSMulLong(arm7_t *c, uint32_t insn)
{
	const int32_t rm = (int32_t)c->r[insn & 0xf];
	const int32_t rs = (int32_t)c->r[(insn >> 8) & 0xf];
	const uint32_t rhi = (insn >> 16) & 0xf;
	const uint32_t rlo = (insn >> 12) & 0xf;

	int64_t res = (int64_t)rm * (int64_t)rs;
	if (insn & INSN_MUL_A) {
		const uint64_t acum = ((uint64_t)c->r[rhi] << 32) | c->r[rlo];
		res = (int64_t)((uint64_t)res + acum);
		ICOUNT -= 1;
	}
	c->r[rhi] = (uint32_t)((uint64_t)res >> 32);
	c->r[rlo] = (uint32_t)res;
	if (insn & INSN_S)
		set_cpsr(c, (CPSR & ~(N_MASK | Z_MASK)) | nz_flags64((uint64_t)res));

	// signed compare after negation (INT_MIN stays negative and counts as < 0x100, as in MAME)
	int32_t ars = rs < 0 ? (int32_t)(0u - (uint32_t)rs) : rs;
	int m;
	if (ars < 0x00000100) m = 1;
	else if (ars < 0x00010000) m = 2;
	else if (ars < 0x01000000) m = 3;
	else m = 4;
	ICOUNT -= 1 + m + 1;
	ICOUNT += 3;
}

static void HandleUMulLong(arm7_t *c, uint32_t insn)
{
	const uint32_t rm = c->r[insn & 0xf];
	const uint32_t rs = c->r[(insn >> 8) & 0xf];
	const uint32_t rhi = (insn >> 16) & 0xf;
	const uint32_t rlo = (insn >> 12) & 0xf;

	uint64_t res = (uint64_t)rm * rs;
	if (insn & INSN_MUL_A) {
		res += ((uint64_t)c->r[rhi] << 32) | c->r[rlo];
		ICOUNT -= 1;
	}
	c->r[rhi] = (uint32_t)(res >> 32);
	c->r[rlo] = (uint32_t)res;
	if (insn & INSN_S)
		set_cpsr(c, (CPSR & ~(N_MASK | Z_MASK)) | nz_flags64(res));

	ICOUNT -= 1 + mul_cycles(rs) + 1;
	ICOUNT += 3;
}

static void HandleMemBlock(arm7_t *c, uint32_t insn)
{
	const uint32_t rb = (insn >> 16) & 0xf;
	uint32_t rbp = c->r[rb];
	int result;

	if (insn & INSN_BDT_L) {
		// S set and R15 not in the list: user bank transfer
		const uint32_t mode = ((insn & INSN_BDT_S) && (insn & 0x8000) == 0) ? ARM7_MODE_USER : MODE;
		if (insn & INSN_BDT_U) {
			if (!(insn & INSN_BDT_P))
				rbp = rbp - 4;
			result = load_inc(c, insn & 0xffff, rbp, mode);
			// "A LDM will always overwrite the updated base if the base is in the list."
			if ((insn & INSN_BDT_W) && ((insn >> rb) & 1) == 0)
				c->r[rb] = c->r[rb] + (uint32_t)result * 4;
		} else {
			if (!(insn & INSN_BDT_P))
				rbp = rbp + 4;
			result = load_dec(c, insn & 0xffff, rbp, mode);
			if ((insn & INSN_BDT_W) && ((insn >> rb) & 1) == 0)
				c->r[rb] = c->r[rb] - (uint32_t)result * 4;
		}
		// R15 included? (the CPSR restore comes last)
		if (insn & 0x8000) {
			R15 -= 4;
			if (insn & INSN_BDT_S) {
				set_cpsr(c, *spsr_reg(c));
				switch_mode(c, MODE);
			} else if ((R15 & 1) && c->arch5) {   // (archRev 5: bit 0 enters Thumb)
				set_cpsr(c, CPSR | T_MASK);
				R15--;
			}
			// LDM PC - takes 2 extra cycles
			ICOUNT -= 2;
		}
		// LDM (NO PC) takes (n)S + 1N + 1I cycles (n = # of register transfers)
		ICOUNT -= result + 1 + 1;
	} else {
		// STM
		if (insn & 0x8000)
			R15 += 12; // the stored PC is the STM address + 12
		const uint32_t mode = (insn & INSN_BDT_S) ? ARM7_MODE_USER : MODE;
		if (insn & INSN_BDT_U) {
			if (!(insn & INSN_BDT_P))
				rbp = rbp - 4;
			result = store_inc(c, insn & 0xffff, rbp, mode);
			if (insn & INSN_BDT_W)
				c->r[rb] = c->r[rb] + (uint32_t)result * 4;
		} else {
			// decrementing, but the CPU writes in incrementing order
			if (!(insn & INSN_BDT_P))
				rbp = rbp + 4;
			result = store_dec(c, insn & 0xffff, rbp, mode);
			if (insn & INSN_BDT_W)
				c->r[rb] = c->r[rb] - (uint32_t)result * 4;
		}
		if (insn & 0x8000)
			R15 -= 12;
		// STM takes (n-1)S + 2N cycles (n = # of register transfers)
		ICOUNT -= (result - 1) + 2;
	}
	// the cycle count is given for each case: remove the -3 at the end
	ICOUNT += 3;
}

static void HandleCoProcRT(arm7_t *c, uint32_t insn)
{
	if (insn & 0x00100000) {
		// MRC
		const uint32_t res = rt_r_callback(c, insn);
		if (!c->pending_und)
			c->r[(insn >> 12) & 0xf] = res;
	} else {
		// MCR
		rt_w_callback(c, insn, c->r[(insn >> 12) & 0xf]);
	}
}

static void HandleCoProcDT(arm7_t *c, uint32_t insn)
{
	// LDC / STC: no coprocessor takes them on a plain ARM7 (MAME arm7_dt_r/w_callback): undefined instruction.
	// MAME's base register handling ends at the pending undefined check, so nothing else changes.
	(void)insn;
	c->pending_und = 1;
	update_irq_state(c);
}

// MAME saturate_qbit_overflow
A7_INLINE int64_t saturate_q(arm7_t *c, int64_t res)
{
	if (res > 2147483647) {
		res = 2147483647;
		set_cpsr(c, CPSR | Q_MASK);
	} else if (res < (-2147483647 - 1)) {
		res = (-2147483647 - 1);
		set_cpsr(c, CPSR | Q_MASK);
	}
	return res;
}

A7_INLINE int32_t sext16(uint32_t v) { return (int32_t)(int16_t)(uint16_t)v; }

// the ARMv5E encodings MAME's ARM7 decoder (arm7ops_0123) accepts in front of the ARMv4 ones; 1 = handled
static int arm_v5_misc(arm7_t *c, uint32_t insn)
{
	const uint32_t m = insn & 0x0ff000f0u;
	if (m == 0x01200030u) { // BLX Rn
		c->r[14] = R15 + 4;
		R15 = c->r[insn & 0x0f];
		if (R15 & 1) {
			R15--;
			set_cpsr(c, CPSR | T_MASK);
		}
		return 1;
	}
	if (m == 0x01600010u) { // CLZ
		const uint32_t v = c->r[insn & 0xf];
		uint32_t n = 0;
		while (n < 32 && !(v & (0x80000000u >> n)))
			n++;
		c->r[(insn >> 12) & 0xf] = n;
		R15 += 4;
		return 1;
	}
	if (m == 0x01000050u || m == 0x01200050u) { // QADD, QSUB
		const int32_t src1 = (int32_t)c->r[insn & 0xf];
		const int32_t src2 = (int32_t)c->r[(insn >> 16) & 0xf];
		const int64_t res = saturate_q(c, m == 0x01000050u ? (int64_t)src1 + src2 : (int64_t)src1 - src2);
		c->r[(insn >> 12) & 0xf] = (uint32_t)(int32_t)res;
		R15 += 4;
		return 1;
	}
	if (m == 0x01400050u || m == 0x01600050u) { // QDADD, QDSUB
		const int32_t src1 = (int32_t)c->r[insn & 0xf];
		int32_t src2 = (int32_t)c->r[(insn >> 16) & 0xf];
		saturate_q(c, (int64_t)src2 * 2);
		src2 = (int32_t)((uint32_t)src2 * 2u);
		const int64_t res = saturate_q(c, m == 0x01400050u ? (int64_t)src1 + src2 : (int64_t)src1 - src2);
		c->r[(insn >> 12) & 0xf] = (uint32_t)(int32_t)res;
		R15 += 4;
		return 1;
	}
	const uint32_t m9 = insn & 0x0ff00090u;
	if (m9 == 0x01000080u) { // SMLAxy
		int32_t src1 = (int32_t)c->r[insn & 0xf];
		int32_t src2 = (int32_t)c->r[(insn >> 8) & 0xf];
		src1 = (insn & 0x20) ? (src1 >> 16) : sext16((uint32_t)src1);
		src2 = (insn & 0x40) ? (src2 >> 16) : sext16((uint32_t)src2);
		const int32_t res1 = (int32_t)((uint32_t)src1 * (uint32_t)src2);
		// MAME widens the accumulator unsigned here
		saturate_q(c, (int64_t)res1 + (int64_t)c->r[(insn >> 12) & 0xf]);
		c->r[(insn >> 16) & 0xf] = (uint32_t)res1 + c->r[(insn >> 12) & 0xf];
		R15 += 4;
		return 1;
	}
	if (m9 == 0x01400080u) { // SMLALxy (MAME multiplies the full registers)
		const int32_t src1 = (int32_t)c->r[insn & 0xf];
		const int32_t src2 = (int32_t)c->r[(insn >> 8) & 0xf];
		uint64_t dst = (uint64_t)c->r[(insn >> 12) & 0xf];
		dst |= (uint64_t)c->r[(insn >> 16) & 0xf] << 32;
		dst += (uint64_t)((int64_t)src1 * src2);
		c->r[(insn >> 12) & 0xf] = (uint32_t)dst;
		c->r[(insn >> 16) & 0xf] = (uint32_t)(dst >> 32);
		R15 += 4;
		return 1;
	}
	if (m9 == 0x01600080u) { // SMULxy
		int32_t src1 = (int32_t)c->r[insn & 0xf];
		int32_t src2 = (int32_t)c->r[(insn >> 8) & 0xf];
		src1 = (insn & 0x20) ? (src1 >> 16) : sext16((uint32_t)src1);
		src2 = (insn & 0x40) ? (src2 >> 16) : sext16((uint32_t)src2);
		c->r[(insn >> 16) & 0xf] = (uint32_t)src1 * (uint32_t)src2;
		R15 += 4;
		return 1;
	}
	const uint32_t mb = insn & 0x0ff000b0u;
	if (mb == 0x012000a0u || mb == 0x01200080u) { // SMULWy, SMLAWy
		const int32_t src1 = (int32_t)c->r[insn & 0xf];
		int32_t src2 = (int32_t)c->r[(insn >> 8) & 0xf];
		src2 = (insn & 0x40) ? (src2 >> 16) : sext16((uint32_t)src2);
		const int32_t res = (int32_t)(((int64_t)src1 * src2) >> 16);
		if (mb == 0x01200080u) {
			const int32_t src3 = (int32_t)c->r[(insn >> 12) & 0xf];
			saturate_q(c, (int64_t)src3 + res);
			// MAME writes the product, not the sum
		}
		c->r[(insn >> 16) & 0xf] = (uint32_t)res;
		R15 += 4;
		return 1;
	}
	return 0;
}

// MAME arm7ops_0123
static void arm7ops_0123(arm7_t *c, uint32_t insn)
{
	if ((insn & 0x0ffffff0u) == 0x012fff10u) {
		// BX
		R15 = c->r[insn & 0x0f];
		if (R15 & 1) {
			R15--;
			set_cpsr(c, CPSR | T_MASK);
		}
		return;
	}
	if (UNLIKELY((insn & 0x0f900000u) == 0x01000000u) && arm_v5_misc(c, insn))
		return;

	if ((insn & 0x0e000000u) == 0 && (insn & 0x80) && (insn & 0x10)) {
		// multiply, swap or halfword data transfer
		if (insn & 0x60) {
			HandleHalfWordDT(c, insn);
		} else if (insn & 0x01000000u) {
			HandleSwap(c, insn);
		} else {
			if (insn & 0x800000) {
				if (insn & 0x00400000)
					HandleSMulLong(c, insn);
				else
					HandleUMulLong(c, insn);
			} else {
				HandleMul(c, insn);
			}
			R15 += 4;
		}
	} else if ((insn & 0x0c000000u) == 0) {
		if (((insn & 0x00100000u) == 0) && ((insn & 0x01800000u) == 0x01000000u)) {
			HandlePSRTransfer(c, insn);
			ICOUNT += 2; // PSR only takes 1 S cycle
			R15 += 4;
		} else {
			HandleALU(c, insn);
		}
	}
}

static void arm_exec(arm7_t *c, uint32_t insn)
{
	switch ((insn >> 24) & 0xf) {
	case 0x0: case 0x1: case 0x2: case 0x3:
		arm7ops_0123(c, insn);
		break;
	case 0x4: case 0x5: case 0x6: case 0x7:
		HandleMemSingle(c, insn);
		R15 += 4;
		break;
	case 0x8: case 0x9:
		HandleMemBlock(c, insn);
		R15 += 4;
		break;
	case 0xa: case 0xb:
		HandleBranch(c, insn);
		break;
	case 0xc: case 0xd:
		HandleCoProcDT(c, insn);
		R15 += 4;
		break;
	case 0xe:
		if (insn & 0x10)
			HandleCoProcRT(c, insn);
		else {
			// CDP: no coprocessor on a plain ARM7 (MAME arm7_do_callback)
			c->pending_und = 1;
			update_irq_state(c);
		}
		R15 += 4;
		break;
	default: // SWI
		c->pending_swi = 1;
		update_irq_state(c);
		check_irq_state(c);
		break;
	}
}

// archRev 5, cond = NV (MAME arm9ops_*): BLX <imm> (enters Thumb, H gives bit 1); CPS / SETEND / PLD / SRS / RFE /
// MCRR / the rest are unsupported on archRev 5 and only advance the PC
static void arm9_exec_nv(arm7_t *c, uint32_t insn)
{
	if (((insn >> 24) & 0xe) == 0xa) {
		uint32_t off = ((insn & 0x00ffffffu) << 2) | ((insn & 0x01000000u) >> 23);
		c->r[14] = R15 + 4;
		off = (off & 0x02000000u) ? (off | 0xfc000000u) : off;
		R15 += off + 8;
		set_cpsr(c, CPSR | T_MASK);
		return;
	}
	R15 += 4;
}

#include "arm7thumb.inc"

/* ---- condition codes ---- */

// cond_ok[cond] bit f = the condition passes for flags nibble f (N=8 Z=4 C=2 V=1); NV never passes (archRev 4)
static uint16_t cond_ok[16];

static void build_cond_table(void)
{
	for (int cond = 0; cond < 16; cond++) {
		uint16_t m = 0;
		for (int f = 0; f < 16; f++) {
			const int n = (f >> 3) & 1, z = (f >> 2) & 1, cf = (f >> 1) & 1, v = f & 1;
			int ok;
			switch (cond) {
			case 0x0: ok = z; break;
			case 0x1: ok = !z; break;
			case 0x2: ok = cf; break;
			case 0x3: ok = !cf; break;
			case 0x4: ok = n; break;
			case 0x5: ok = !n; break;
			case 0x6: ok = v; break;
			case 0x7: ok = !v; break;
			case 0x8: ok = cf && !z; break;
			case 0x9: ok = !cf || z; break;
			case 0xa: ok = n == v; break;
			case 0xb: ok = n != v; break;
			case 0xc: ok = !z && n == v; break;
			case 0xd: ok = z || n != v; break;
			case 0xe: ok = 1; break;
			default: ok = 0; break;
			}
			if (ok)
				m |= (uint16_t)(1u << f);
		}
		cond_ok[cond] = m;
	}
}

/* ---- API ---- */

// (the cheats' access to this CPU's program space: cheat_cpu.h)
static uint8_t cheat_rd(void *x, uint32_t a) { return (uint8_t)RD8((arm7_t *)x, a); }   // (as the CPU: pages, callbacks)
static void cheat_wr(void *x, uint32_t a, uint8_t v) { WR8((arm7_t *)x, a, v); }

void arm7_init(arm7_t *c)
{
	rbx_cheat_offer("arm7", cheat_rd, cheat_wr, c, 0, 1);
	memset(c, 0, sizeof(*c));
	build_cond_table();
	for (int k = 0; k < 4; k++)
		c->tlb_r[k].tag = c->tlb_w[k].tag = 0xffffffffu;
}

void arm7_reset(arm7_t *c)
{
	memset(c->r, 0, sizeof(c->r));
	memset(c->bank, 0, sizeof(c->bank));
	c->pending_irq = c->pending_fiq = c->pending_und = c->pending_swi = c->pending_interrupt = 0;
	c->cp15_control = c->cp15_tlb_base = c->cp15_dac = c->cp15_far = c->cp15_pid = 0;
	c->cp15_fsr[0] = c->cp15_fsr[1] = 0;
	if (c->arch5) {   // (MAME arm946es: cp15_control 0x78 from the constructor, the TCM registers 0)
		c->cp15_control = 0x78;
		c->cp15_itcm_reg = c->cp15_dtcm_reg = 0;
	}
	c->icount = 0;
	c->total_cycles = 0;
	// start up in SVC mode with interrupts disabled (the registers are all 0, so no bank switch is needed)
	c->cpsr = I_MASK | F_MASK | 0x10 | ARM7_MODE_SVC;
	c->r[15] = 0;
}

// ReBOX: the common data processing forms executed in the run loop without the decoders (the same results and cycle
// counts as HandleALU): Rd not R15; an immediate operand, or a register (not R15) without a shift (LSL #0)
A7_INLINE void alu_fast(arm7_t *c, uint32_t insn, uint32_t op2, uint32_t sc)
{
	const uint32_t opcode = (insn >> 21) & 0xf;
	uint32_t rn = 0, rd;
	if ((opcode & 0xd) != 0xd) {
		const uint32_t rnn = (insn >> 16) & 0xf;
		rn = (rnn == 15) ? R15 + 8 : c->r[rnn];
	}
	const int s = (insn & INSN_S) != 0;
	switch (opcode) {
	case 0x6: rd = rn - op2 - ((CPSR & C_MASK) ? 0 : 1); if (s) set_cpsr(c, sub_flags(CPSR, rd, rn, op2)); break;
	case 0xa: case 0x2: rd = rn - op2; if (s) set_cpsr(c, sub_flags(CPSR, rd, rn, op2)); break;
	case 0x7: rd = op2 - rn - ((CPSR & C_MASK) ? 0 : 1); if (s) set_cpsr(c, sub_flags(CPSR, rd, op2, rn)); break;
	case 0x3: rd = op2 - rn; if (s) set_cpsr(c, sub_flags(CPSR, rd, op2, rn)); break;
	case 0x5: rd = rn + op2 + ((CPSR & C_MASK) >> C_BIT); if (s) set_cpsr(c, add_flags(CPSR, rd, rn, op2)); break;
	case 0xb: case 0x4: rd = rn + op2; if (s) set_cpsr(c, add_flags(CPSR, rd, rn, op2)); break;
	default:
		switch (opcode) {
		case 0x0: case 0x8: rd = rn & op2; break;
		case 0xe: rd = rn & ~op2; break;
		case 0x9: case 0x1: rd = rn ^ op2; break;
		case 0xc: rd = rn | op2; break;
		case 0xd: rd = op2; break;
		default: rd = ~op2; break;
		}
		if (s) set_cpsr(c, (CPSR & ~(N_MASK | Z_MASK | C_MASK)) | NZ_FLAGS(rd) | ((uint32_t)(sc != 0) << C_BIT));
		break;
	}
	R15 += 4;
	if ((opcode & 0xc) != 0x8)
		c->r[(insn >> 12) & 0xf] = rd;
	ICOUNT += 2;
}

// ReBOX: idle loop (arm7_run, after a short backward jump to R15): 1 = the slice ends here. Up to 4 heads are kept,
// for loops that hold inner backward jumps; the registers are compared at every 8th arrival at a head, or each time
// when a round since the last arrival took 24 cycles or more (a working loop pays a counter, not a copy)
static int idle_check(arm7_t *c)
{
	arm7_idle_head *h = NULL;
	for (int k = 0; k < ARM7_IDLE_HEADS; k++)
		if (c->idle[k].pc == R15) {
			h = &c->idle[k];
			break;
		}
	if (!h) {
		h = &c->idle[c->idle_next];
		c->idle_next = (c->idle_next + 1) % ARM7_IDLE_HEADS;
		h->pc = R15;
		h->cnt = 0;
		h->valid = 0;
		h->stamp = c->icount;
		return 0;
	}
	if ((uint32_t)(h->stamp - c->icount) < 24 && (++h->cnt & 7) != 0)
		return 0;
	h->stamp = c->icount;
	if (h->valid && c->writes == h->writes && !c->pending_interrupt && !memcmp(h->regs, c->r, sizeof(c->r)) &&
	    h->cpsr == c->cpsr) {
		c->idle_hits++;
		c->icount = 0;
		return 1;
	}
	h->valid = 1;
	h->writes = c->writes;
	memcpy(h->regs, c->r, sizeof(c->r));
	h->cpsr = c->cpsr;
	return 0;
}

#include "arm7_jit.inc"

int arm7_run(arm7_t *c, int cycles)
{
	if (cycles <= 0)
		return 0;
	c->slice = cycles;
	c->icount = cycles;
	c->running = 1;
#ifdef ARM7_JIT_HOST
	s_jin_run++;
#endif
	// (the code page cache of this slice: page tables do not change while it runs, except through arm7_map from a
	// handler, which no PGM handler does for a code page)
	uint32_t fpage = 0xffffffffu;
	const uint8_t *fbase = NULL;
#ifdef ARM7_JIT_HOST
	int natural = 0;   // (the pc came by a jump or a block's end: a block may start here; arm7_jit.inc)
#endif
	do {
#ifdef ARM7_JIT_HOST
		if (c->jit && !(R15 & ((c->cpsr & T_MASK) ? 1u : 3u))) {   // (translated code: arm7_jit.inc)
			const jit_block_fn f = jit_lookup(c, (c->cpsr & T_MASK) ? R15 | 1 : R15, natural);
			const uint32_t prev = f ? f(c) : 1;
			if (prev != 1) {   // (1: not run, near the slice's end)
				natural = 1;   // (a jump, or the end of a block; or the slice's end, and then the loop ends)
				if (UNLIKELY(R15 < prev) && prev - R15 <= 0x40 && c->idle_skip && idle_check(c))
					break;
				continue;
			}
		}
#endif
		const uint32_t prev_pc = R15;
		if (UNLIKELY(prev_pc == c->hook_pc) && c->hook_eat)
			c->icount -= c->hook_eat;
		if (c->cpsr & T_MASK) {
			// "In Thumb state, bit [0] is undefined and must be ignored."
			const uint32_t op = FETCH16(c, R15 & ~1u);
			thumb_exec(c, op);
		} else {
			// "In ARM state, bits [1:0] of r15 are undefined and must be ignored."
			// (the code page of the last fetch kept: no table lookup while the pc stays in it)
			uint32_t insn;
			const uint32_t fa = R15 & ~3u;
			if (LIKELY((fa >> ARM7_PAGE_SHIFT) == fpage) && fbase)
				memcpy(&insn, fbase + POFS(fa), 4);
			else {
				fpage = fa >> ARM7_PAGE_SHIFT;
				fbase = page_r(c, fa);
				insn = FETCH32(c, fa);
			}
			const uint32_t cond = insn >> 28;
			if (LIKELY(cond == 0xe) || ((cond_ok[cond] >> (c->cpsr >> 28)) & 1)) {
				// (the common forms first: data processing with Rd not R15 and not a PSR transfer, and B / BL)
				if ((insn & 0x0e000000u) == 0x02000000u && (insn & 0xf000u) != 0xf000u &&
				           (insn & 0x01900000u) != 0x01000000u) {
					const uint32_t by = (insn >> 8) & 0xf;
					uint32_t op2, sc;
					if (by) {
						op2 = rotr32(insn & 0xff, by << 1);
						sc = op2 & SIGN_BIT;
					} else {
						op2 = insn & 0xfff;   // (as HandleALU)
						sc = CPSR & C_MASK;
					}
					alu_fast(c, insn, op2, (insn & INSN_S) ? sc : 0);
				} else if ((insn & 0x0e000ff0u) == 0 && (insn & 0xf000u) != 0xf000u && (insn & 0xfu) != 0xfu &&
				           (insn & 0x01900000u) != 0x01000000u) {
					ICOUNT -= 1;   // (register operand)
					alu_fast(c, insn, c->r[insn & 0xf], (insn & INSN_S) ? (CPSR & C_MASK) : 0);
				} else if ((insn & 0x0f200000u) == 0x05000000u && (insn & 0xf000u) != 0xf000u &&
				           (insn & 0xf0000u) != 0xf0000u) {
					// LDR / STR (B) Rd, [Rn, #+-imm12], Rd and Rn not R15 (as HandleMemSingle)
					const uint32_t off = insn & 0xfff;
					const uint32_t a = (insn & INSN_SDT_U) ? c->r[(insn >> 16) & 0xf] + off : c->r[(insn >> 16) & 0xf] - off;
					const uint32_t rd = (insn >> 12) & 0xf;
					if (insn & INSN_SDT_L)
						c->r[rd] = (insn & INSN_SDT_B) ? RD8(c, a) : RD32(c, a);
					else {
						if (insn & INSN_SDT_B)
							WR8(c, a, c->r[rd] & 0xff);
						else
							WR32(c, a, c->r[rd]);
						ICOUNT += 1;
					}
					R15 += 4;
				} else if ((insn & 0x0ffffff0u) == 0x012fff10u) {   // BX (as arm7ops_0123)
					R15 = c->r[insn & 0x0f];
					if (R15 & 1) {
						R15--;
						set_cpsr(c, CPSR | T_MASK);
					}
				} else if ((insn & 0x0e000000u) == 0x0a000000u) {
					uint32_t off = (insn & 0x00ffffffu) << 2;
					if (insn & INSN_BL)
						c->r[14] = R15 + 4;
					off = (off & 0x02000000u) ? (off | 0xfc000000u) : off;
					R15 += off + 8;
				} else
				arm_exec(c, insn);
			} else if (UNLIKELY(cond == 0xf) && c->arch5) {
				arm9_exec_nv(c, insn);
			} else {
				// any unexecuted instruction only takes 1 cycle
				R15 += 4;
				c->icount += 2;
			}
		}
		if (c->pending_interrupt)
			check_irq_state(c);
		// all instructions remove 3 cycles (the others have adjusted icount already)
		c->icount -= 3;
		// ReBOX: idle loop. A short backward jump to a loop head that was reached the last time with the same
		// registers and flags, no memory written in between and no interrupt waiting: the loop is waiting for
		// something outside the ARM (a latch, a shared RAM word, an interrupt), which cannot change during this
		// slice, so its remaining iterations change nothing but the time: the slice ends here
#ifdef ARM7_JIT_HOST
		natural = R15 != prev_pc + ((c->cpsr & T_MASK) ? 2u : 4u);
#endif
		if (UNLIKELY(R15 < prev_pc) && prev_pc - R15 <= 0x40 && c->idle_skip && idle_check(c))
			break;
	} while (c->icount > 0);
	c->running = 0;
#ifdef ARM7_JIT_HOST
	s_jin_run--;
#endif
	const int ran = c->slice - c->icount;
	c->total_cycles += ran;
	return ran;
}

void arm7_set_irq_line(arm7_t *c, int line, int state)
{
	if (line == ARM7_IRQ_LINE)
		c->pending_irq = state != 0;
	else if (line == ARM7_FIRQ_LINE)
		c->pending_fiq = state != 0;
	update_irq_state(c);
	if (!c->running)
		check_irq_state(c);
}

int arm7_map(arm7_t *c, uint32_t start, uint32_t end, void *mem, int writable)
{
	if ((start & 0x3ff) != 0 || (end & 0x3ff) != 0x3ff || end < start)
		return -1;
	for (int k = 0; k < 4; k++)   // (the translated code's page cache)
		c->tlb_r[k].tag = c->tlb_w[k].tag = 0xffffffffu;
	uint8_t *base = (uint8_t *)mem;
	uint32_t a = start;
	for (;;) {
		const uint32_t blk = a >> ARM7_L1_SHIFT;
		if (c->l1[blk] == 0) {
			if (mem == NULL)
				goto next; // nothing mapped in this block
			if (c->l2_used >= ARM7_L2_POOL)
				return -1;
			memset(&c->l2[c->l2_used], 0, sizeof(c->l2[0]));
			c->l1[blk] = (uint8_t)(++c->l2_used);
		}
		{
			const uint32_t pg = (a >> ARM7_PAGE_SHIFT) & (ARM7_L2_SIZE - 1);
			uint8_t *p = base ? base + (a - start) : NULL;
			c->l2[c->l1[blk] - 1].r[pg] = p;
			c->l2[c->l1[blk] - 1].w[pg] = writable ? p : NULL;
		}
	next:
		if (a >= end - 0x3ff)
			break;
		a += 0x400;
	}
	return 0;
}

void arm7_end_timeslice(arm7_t *c)
{
	if (!c->running)
		return;
	c->slice -= c->icount;
	c->icount = 0;
}

void arm7_eat_cycles(arm7_t *c, int n)
{
	if (!c->running)
		return;
	c->icount = (n > c->icount) ? 0 : c->icount - n;
}

void arm7_set_cpsr(arm7_t *c, uint32_t v) { set_cpsr(c, v); }

uint32_t *arm7_mode_reg(arm7_t *c, int mode, int i) { return mode_reg(c, (uint32_t)mode, (uint32_t)i & 15); }

uint32_t *arm7_spsr(arm7_t *c) { return spsr_reg(c); }
