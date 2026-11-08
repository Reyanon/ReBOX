/*  Ensoniq ES5510 (ESP), ported from MAME src/devices/cpu/es5510/es5510.cpp (BSD-3-Clause, copyright-holders:
    Christian Brunschen). See es5510.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "es5510.h"

#include "rbx.h"

#include <stdlib.h>
#include <string.h>

enum { ST_RUNNING, ST_HALTED };
enum { SD_REG = 1, SD_DELAY = 2, SD_BOTH = 3 };
enum { RC_READ, RC_WRITE, RC_DUMP };
enum { RA_DELAY, RA_TABLE_A, RA_TABLE_B, RA_IO };

#define FLAG_N 0x80
#define FLAG_C 0x40
#define FLAG_V 0x20
#define FLAG_LT 0x10
#define FLAG_Z 0x08
#define FLAG_NOT 0x04
#define FLAG_MASK (FLAG_N | FLAG_C | FLAG_V | FLAG_LT | FLAG_Z)
#define OP_CMP 4

static const uint8_t ALU_OPERANDS[16] = { 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0 };
// alu_src, alu_dst, mac_src, mac_dst
static const uint8_t OPERAND_SELECT[16][4] = {
	{ SD_REG, SD_REG, SD_REG, SD_REG }, { SD_REG, SD_REG, SD_REG, SD_DELAY }, { SD_REG, SD_REG, SD_REG, SD_BOTH },
	{ SD_REG, SD_REG, SD_DELAY, SD_REG }, { SD_REG, SD_REG, SD_DELAY, SD_BOTH }, { SD_REG, SD_DELAY, SD_REG, SD_REG },
	{ SD_REG, SD_DELAY, SD_DELAY, SD_REG }, { SD_REG, SD_BOTH, SD_REG, SD_REG }, { SD_REG, SD_BOTH, SD_DELAY, SD_REG },
	{ SD_DELAY, SD_REG, SD_REG, SD_REG }, { SD_DELAY, SD_REG, SD_REG, SD_DELAY }, { SD_DELAY, SD_REG, SD_REG, SD_BOTH },
	{ SD_DELAY, SD_REG, SD_DELAY, SD_REG }, { SD_DELAY, SD_REG, SD_DELAY, SD_BOTH }, { SD_DELAY, SD_BOTH, SD_REG, SD_REG },
	{ SD_DELAY, SD_BOTH, SD_DELAY, SD_REG },
};
static const uint8_t RAM_CYCLE[8] = { RC_READ, RC_WRITE, RC_READ, RC_WRITE, RC_READ, RC_DUMP, RC_READ, RC_WRITE };
static const uint8_t RAM_ACCESS[8] = { RA_DELAY, RA_DELAY, RA_TABLE_A, RA_TABLE_A, RA_TABLE_B, RA_DELAY, RA_IO, RA_IO };

static inline uint8_t set_flag_to(uint8_t ccr, uint8_t flag, int set) { return set ? (uint8_t)(ccr | flag) : (uint8_t)(ccr & ~flag); }
static inline int32_t sext24(int32_t v) { return (int32_t)((uint32_t)v << 8) >> 8; }

static inline int32_t add(int32_t a, int32_t b, uint8_t *flags)
{
	int32_t aSign = a & 0x00800000, bSign = b & 0x00800000;
	int32_t result = a + b;
	int32_t resultSign = result & 0x00800000;
	int overflow = (aSign == bSign) && (aSign != resultSign);
	int carry = (result & 0x01000000) != 0;
	int negative = resultSign != 0;
	int lessThan = overflow ^ negative;
	// all five flags are set (as MAME's sequence of setFlagTo)
	*flags = (uint8_t)((*flags & ~(FLAG_C | FLAG_N | FLAG_Z | FLAG_V | FLAG_LT)) | (carry ? FLAG_C : 0) | (negative ? FLAG_N : 0) |
		((result & 0x00ffffff) == 0 ? FLAG_Z : 0) | (overflow ? FLAG_V : 0) | (lessThan ? FLAG_LT : 0));
	return result & 0x00ffffff;
}

static inline int32_t saturate(int32_t value, uint8_t *flags, int negative)
{
	if (*flags & FLAG_V) {
		*flags = set_flag_to(*flags, FLAG_N, negative);
		*flags &= ~FLAG_Z;
		return negative ? 0x00800000 : 0x007fffff;
	}
	return value;
}

static inline int32_t negate(int32_t value) { return ((value ^ 0x00ffffff) + 1) & 0x00ffffff; }

static inline __attribute__((always_inline)) int32_t asl(int32_t value, int shift, uint8_t *flags)
{
	int32_t src24 = value & 0x00ffffff;
	int carry = (src24 >> (24 - shift)) & 1;
	// shift <= 8: the 24-bit value times 2^shift fits in 32 bits
	int32_t shifted = (int32_t)((uint32_t)sext24(src24) << shift);
	int overflow = shifted > 0x007fffff || shifted < -0x00800000;
	int32_t result = overflow ? (shifted < 0 ? 0x00800000 : 0x007fffff) : ((int32_t)shifted & 0x00ffffff);
	*flags = set_flag_to(*flags, FLAG_C, carry);
	*flags = set_flag_to(*flags, FLAG_V, overflow);
	*flags = set_flag_to(*flags, FLAG_N, (result & 0x00800000) != 0);
	*flags = set_flag_to(*flags, FLAG_Z, (result & 0x00ffffff) == 0);
	return result;
}

static inline int16_t dram_r(es5510_t *e, int32_t addr) { return e->dram[addr & (ES5510_DRAM_SIZE - 1)]; }
static inline void dram_w(es5510_t *e, int32_t addr, int16_t v) { e->dram[addr & (ES5510_DRAM_SIZE - 1)] = v; }

static int32_t read_reg(es5510_t *e, uint8_t reg)
{
	if (reg < 0xc0)
		return e->gpr[reg];
	switch (reg) {
	case 234: return e->ser0r * 256;
	case 235: return e->ser0l * 256;
	case 236: return e->ser1r * 256;
	case 237: return e->ser1l * 256;
	case 238: return e->ser2r * 256;
	case 239: return e->ser2l * 256;
	case 240: return e->ser3r * 256;
	case 241: return e->ser3l * 256;
	case 242: return e->mac_overflow ? (e->machl < 0 ? 0x00000000 : 0x00ffffff) : (int32_t)(e->machl & 0x00ffffff);
	case 243: return e->mac_overflow ? (e->machl < 0 ? 0x00800000 : 0x007fffff) : (int32_t)((e->machl >> 24) & 0x00ffffff);
	case 244: return e->dil * 256;
	case 245: return e->dlength;
	case 246: return e->abase;
	case 247: return e->bbase;
	case 248: return e->dbase;
	case 249: return e->sigreg;
	case 250: return (int32_t)e->ccr * 65536;
	case 251: return (int32_t)e->cmr * 65536;
	case 252: return 0x00ffffff;
	case 253: return 0x00800000;
	case 254: return 0x007fffff;
	}
	return 0;
}

static int64_t sext48(int64_t v) { return (int64_t)((uint64_t)v << 16) >> 16; }

static __attribute__((noinline)) void write_reg(es5510_t *e, uint8_t reg, int32_t value)
{
	value &= 0x00ffffff;
	if (reg < 0xc0) {
		e->gpr[reg] = value;
		return;
	}
	switch (reg) {
	case 234: e->ser0r = (int16_t)((value >> 8) & 0xffff); break;
	case 235: e->ser0l = (int16_t)((value >> 8) & 0xffff); break;
	case 236: e->ser1r = (int16_t)((value >> 8) & 0xffff); break;
	case 237: e->ser1l = (int16_t)((value >> 8) & 0xffff); break;
	case 238: e->ser2r = (int16_t)((value >> 8) & 0xffff); break;
	case 239: e->ser2l = (int16_t)((value >> 8) & 0xffff); break;
	case 240: e->ser3r = (int16_t)((value >> 8) & 0xffff); break;
	case 241: e->ser3l = (int16_t)((value >> 8) & 0xffff); break;
	case 242: e->machl = sext48((e->machl & ((int64_t)0x00ffffff << 24)) | value); break;
	case 243:
		e->machl = sext48((e->machl & 0x00ffffff) | ((int64_t)value << 24));
		e->mac_overflow = 0;
		break;
	case 244: {
		int n = 0;
		int32_t x = value;
		while (x & 1) { n++; x >>= 1; }
		e->memshift = (int8_t)n;
		e->memsiz = 0x00ffffff >> (24 - n);
		e->memmask = 0x00ffffff & ~e->memsiz;
		e->memincrement = 1 << n;
		e->dbase &= e->memmask;
		break;
	}
	case 245: e->dlength = value; break;
	case 246: e->abase = value; break;
	case 247: e->bbase = value; break;
	case 248: e->dbase = value; break;
	case 249:
		e->sigreg = value;
		e->mulshift = (value >> 22) & 1 ? 1 : 2;
		break;
	case 250: e->ccr = (int8_t)((value >> 16) & FLAG_MASK); break;
	case 251: e->cmr = (int8_t)((value >> 16) & (FLAG_MASK | FLAG_NOT)); break;
	}
}

static inline __attribute__((always_inline)) int32_t alu_operation(uint8_t op, int32_t a, int32_t b, uint8_t *flags)
{
	int32_t tmp;
	switch (op) {
	case 0x0: tmp = add(a, b, flags); return saturate(tmp, flags, (a & 0x00800000) != 0);
	case 0x1: tmp = add(a, negate(b), flags); return saturate(tmp, flags, (a & 0x00800000) != 0);
	case 0x2: return add(a, b, flags);
	case 0x3: return add(a, negate(b), flags);
	case 0x4: add(a, negate(b), flags); return a;
	case 0x5:
	case 0x6:
	case 0x7:
		a = op == 5 ? (a & b) : op == 6 ? (a | b) : (a ^ b);
		*flags = set_flag_to(*flags, FLAG_N, (a & 0x00800000) != 0);
		*flags = set_flag_to(*flags, FLAG_Z, a == 0);
		return a;
	case 0x8: {
		*flags &= ~FLAG_N;
		int neg = (b & 0x00800000) != 0;
		*flags = set_flag_to(*flags, FLAG_C, neg);
		int32_t r = neg ? (0x00ffffff ^ b) : b;
		*flags = set_flag_to(*flags, FLAG_Z, (r & 0x00ffffff) == 0);
		return r;
	}
	case 0x9: return b;
	case 0xa: return asl(b, 2, flags);
	case 0xb: return asl(b, 8, flags);
	case 0xc:
		*flags &= ~FLAG_N;
		*flags = set_flag_to(*flags, FLAG_C, (b & 0x00800000) != 0);
		return (b << 15) & 0x007fffff;
	case 0xd: return add(0x007fffff, negate(b), flags);
	case 0xe:
		*flags = set_flag_to(*flags, FLAG_N, (b & 0x00800000) != 0);
		*flags = set_flag_to(*flags, FLAG_C, (b & 1) != 0);
		return (b >> 1) | (b & 0x00800000);
	}
	return 0;
}

// decoded program (MAME decodes each instruction word as it executes; here the whole program is decoded again
// whenever it may have changed: host program writes, reset, state load)
typedef struct {
	uint8_t aReg, bReg, cReg, dReg, op, operands, asrc, adst, msrc, mdst, skippable, accumulate, cycle, access, nop, nop_run;
	uint8_t alu_nop, mac_nop;
	uint8_t mwk, awk;   // where the MAC / ALU result goes (WK_*), for exec_noskip
	uint32_t rk;        // the RAM stage bits for exec_noskip: io << 24 | cycle << 25
} es5510_dec;
#define WK_GPR 1   // a plain register (0x00-0xbf)
#define WK_SPR 2   // a special register (0xc0-0xfb)
#define WK_DOL 4   // the delay line out
static inline uint8_t write_kind(uint8_t dst, uint8_t reg)
{
	uint8_t k = 0;
	if (dst & SD_REG)
		k |= reg < 0xc0 ? WK_GPR : reg < 252 ? WK_SPR : 0;
	if (dst & SD_DELAY)
		k |= WK_DOL;
	return k;
}
// the general register an instruction word may write (its ALU and MAC results), -1 for none: two values
void es5510_instr_dests(uint64_t instr, int *alu_reg, int *mac_reg)
{
	const uint8_t *sel = OPERAND_SELECT[(instr >> 8) & 0x0f];
	const int a = (int)((instr >> 16) & 0xff), c = (int)((instr >> 32) & 0xff);
	*alu_reg = (sel[1] & SD_REG) && a < 0xc0 ? a : -1;
	*mac_reg = (sel[3] & SD_REG) && c < 0xc0 ? c : -1;
}

static es5510_dec s_dec[0x100];   // [0xa0..0xff]: instruction word 0 (pc past the program memory)
static int s_dec_dirty = 1;
#if defined(__arm__)
static int s_jit_ok;   // (es5510_jit.inc)
#endif
static int s_dec_skips = 1;   // the program has skippable instructions (or no END): general exec

void es5510_invalidate(es5510_t *e) { (void)e; s_dec_dirty = 1; }

static void decode_one(es5510_dec *d, uint64_t instr)
{
	const uint8_t *sel = OPERAND_SELECT[(instr >> 8) & 0x0f];
	int rc = (int)((instr >> 3) & 7);
	d->aReg = (uint8_t)(instr >> 16);
	d->bReg = (uint8_t)(instr >> 24);
	d->cReg = (uint8_t)(instr >> 32);
	d->dReg = (uint8_t)(instr >> 40);
	d->op = (uint8_t)((instr >> 12) & 0x0f);
	d->operands = ALU_OPERANDS[d->op];
	d->asrc = sel[0];
	d->adst = sel[1];
	d->msrc = sel[2];
	d->mdst = sel[3];
	d->skippable = (instr & 0x80) != 0;
	d->accumulate = (instr >> 6) & 1;
	d->cycle = RAM_CYCLE[rc];
	d->access = RAM_ACCESS[rc];
	// no-op: MOV ZERO > ZERO, MAC + ZERO * ZERO > ZERO, read the delay line
	d->nop = d->op == 9 && d->aReg == 0xff && d->bReg == 0xff && d->cReg == 0xff && d->dReg == 0xff &&
		d->asrc == SD_REG && d->adst == SD_REG && d->msrc == SD_REG && d->mdst == SD_REG && !d->skippable &&
		d->accumulate && d->cycle == RC_READ && d->access == RA_DELAY;
}

// ALU part without effect: MOV (no flags) into an ignored register (0xfc-0xff)
static inline int alu_is_nop(uint8_t op, uint8_t dst, uint8_t aReg) { return op == 9 && dst == SD_REG && aReg >= 252; }
// MAC part without effect: product 0 (ZERO operand), accumulating into MACHL unchanged, result to an ignored register
static inline int mac_is_nop(uint8_t src, uint8_t dst, uint8_t cReg, uint8_t dReg, uint8_t acc)
{
	return acc && dst == SD_REG && cReg >= 252 && ((src == SD_REG && cReg == 255) || dReg == 255);
}

static void decode_all(const es5510_t *e)
{
	for (int i = 0; i < 0x100; i++) {
		es5510_dec *d = &s_dec[i];
		decode_one(d, i < 0xa0 ? e->instr[i] : 0);
		d->alu_nop = (uint8_t)alu_is_nop(d->op, d->adst, d->aReg);
		d->mac_nop = (uint8_t)mac_is_nop(d->msrc, d->mdst, d->cReg, d->dReg, d->accumulate);
		d->mwk = write_kind(d->mdst, d->cReg);
		d->awk = d->op == OP_CMP ? 0 : write_kind(d->adst, d->aReg);
		d->rk = (uint32_t)(d->access == RA_IO) << 24 | (uint32_t)d->cycle << 25;
	}
	// length of the no-op run starting at each pc (within the program memory)
	int run = 0;
	for (int i = 0x9f; i >= 0; i--) {
		run = s_dec[i].nop ? run + 1 : 0;
		s_dec[i].nop_run = (uint8_t)run;
	}
	for (int i = 0xa0; i < 0x100; i++)
		s_dec[i].nop_run = 0;
	s_dec_skips = 1;
	for (int i = 0; i < 0xa0; i++) {
		if (s_dec[i].skippable)
			break;
		if (s_dec[i].op == 0xf) {
			s_dec_skips = 0;
			break;
		}
	}
	s_dec_dirty = 0;
#if defined(__arm__)
	s_jit_ok = 0;   // (compiled again)
#endif
}

// MAME (dbase + offset) % (dlength + memincrement), without the division in the usual cases (no hardware divide
// on the Vita's Cortex-A9)
static inline int32_t delay_mod(int32_t x, int32_t m)
{
	if (x >= 0 && x < m)
		return x;
	if (m > 0 && x >= m && x - m < m)
		return x - m;
	return m ? x % m : 0;
}

static const int32_t SPR_CONST[4] = { 0x00ffffff, 0x00800000, 0x007fffff, 0 };   // MINUS1, MIN, MAX, ZERO

// special registers for exec: the values it keeps in locals are passed in
static __attribute__((noinline)) int32_t read_spr_fast(es5510_t *e, uint8_t reg, int16_t dil, int8_t ccr, int64_t machl, uint8_t mac_overflow,
	int32_t dbase, int32_t dlength)
{
	switch (reg) {
	case 234: return e->ser0r * 256;
	case 235: return e->ser0l * 256;
	case 236: return e->ser1r * 256;
	case 237: return e->ser1l * 256;
	case 238: return e->ser2r * 256;
	case 239: return e->ser2l * 256;
	case 240: return e->ser3r * 256;
	case 241: return e->ser3l * 256;
	case 242: return mac_overflow ? (machl < 0 ? 0x00000000 : 0x00ffffff) : (int32_t)(machl & 0x00ffffff);
	case 243: return mac_overflow ? (machl < 0 ? 0x00800000 : 0x007fffff) : (int32_t)((machl >> 24) & 0x00ffffff);
	case 244: return dil * 256;
	case 245: return dlength;
	case 246: return e->abase;
	case 247: return e->bbase;
	case 248: return dbase;
	case 249: return e->sigreg;
	case 250: return (int32_t)ccr * 65536;
	case 251: return (int32_t)e->cmr * 65536;
	case 252: return 0x00ffffff;
	case 253: return 0x00800000;
	case 254: return 0x007fffff;
	}
	return 0;
}

static __attribute__((noinline)) void write_ser(es5510_t *e, uint8_t reg, int32_t value)
{
	int16_t v = (int16_t)(((value & 0x00ffffff) >> 8) & 0xffff);
	switch (reg) {
	case 234: e->ser0r = v; break;
	case 235: e->ser0l = v; break;
	case 236: e->ser1r = v; break;
	case 237: e->ser1l = v; break;
	case 238: e->ser2r = v; break;
	case 239: e->ser2l = v; break;
	case 240: e->ser3r = v; break;
	default: e->ser3l = v; break;
	}
}

// MAME execute_run, one instruction per iteration, with the pipeline and the hot registers in locals. Special
// registers (0xc0-0xff) go through read_reg / write_reg on the structure, so the locals they touch are stored
// before and reloaded after (SPILL / FILL)
static __attribute__((unused)) void exec(es5510_t *e, int max_steps)   // reference: es5510_run_once uses exec_noskip
{
	int32_t *gpr = e->gpr;
	int16_t *dram = e->dram;
	uint8_t pc = e->pc;
	int16_t dil = e->dil;
	int16_t dol0 = e->dol[0], dol1 = e->dol[1];
	int32_t dol_count = e->dol_count;
	int8_t ccr = e->ccr;
	int64_t machl = e->machl;
	uint8_t mac_overflow = e->mac_overflow;
	int32_t dbase = e->dbase, dlength = e->dlength, memmask = e->memmask, memincrement = e->memincrement;
	int8_t memshift = e->memshift;
	int32_t mulshift = e->mulshift;
	es5510_alu alu = e->alu;
	es5510_mac mac = e->mulacc;
	int32_t ram_a = e->ram.address, ram_p_a = e->ram_p.address, ram_pp_a = e->ram_pp.address;
	uint8_t ram_io = e->ram.io, ram_p_io = e->ram_p.io, ram_pp_io = e->ram_pp.io;
	uint8_t ram_cy = e->ram.cycle, ram_p_cy = e->ram_p.cycle, ram_pp_cy = e->ram_pp.cycle;
#define SPILL() do { e->dil = dil; e->dol[0] = dol0; e->dol[1] = dol1; e->dol_count = dol_count; e->ccr = ccr; \
	e->machl = machl; e->mac_overflow = mac_overflow; e->dbase = dbase; e->dlength = dlength; } while (0)
#define FILL() do { ccr = e->ccr; machl = e->machl; mac_overflow = e->mac_overflow; dbase = e->dbase; dlength = e->dlength; \
	memmask = e->memmask; memincrement = e->memincrement; memshift = e->memshift; mulshift = e->mulshift; } while (0)
#define RREG(r, out) do { uint8_t r_ = (r); (out) = r_ < 0xc0 ? gpr[r_] : r_ >= 252 ? SPR_CONST[r_ - 252] : \
	read_spr_fast(e, r_, dil, ccr, machl, mac_overflow, dbase, dlength); } while (0)
#define WREG(r, v) do { uint8_t r_ = (r); if (r_ < 0xc0) gpr[r_] = (v) & 0x00ffffff; \
	else if (r_ >= 234 && r_ <= 241) write_ser(e, r_, (v)); \
	else if (r_ < 252) { SPILL(); write_reg(e, r_, (v)); FILL(); } } while (0)
#define DOL(v) do { int16_t d16_ = (int16_t)((v) >> 8); if (dol_count >= 2) { dol0 = dol1; dol1 = d16_; } \
	else if (dol_count++ == 0) dol0 = d16_; else dol1 = d16_; } while (0)

#define DELAY_ADDR(p, out) do { int32_t m_ = dlength + memincrement, x_ = dbase + ((p) < 0xc0 ? gpr[(p)] : 0); \
	x_ = delay_mod(x_, m_); (out) = (x_ & memmask) >> memshift; } while (0)

	int nop_steps = 0;   // consecutive no-op instructions just executed
	// the MAC / ALU operations in flight that cannot change anything are not computed (their operand values in the
	// structure are then stale, which only the same no-op computation would read)
	uint8_t mac_nop = (uint8_t)mac_is_nop(mac.src, mac.dst, mac.cReg, mac.dReg, mac.accumulate);
	uint8_t alu_nop = (uint8_t)alu_is_nop(alu.op, alu.dst, alu.aReg);
	for (int n = 0; n < max_steps; n++) {
		// a run of no-op instructions (MOV ZERO > ZERO, MAC ZERO * ZERO > ZERO, read the delay line) leaves the state
		// as it is once the pipeline holds two of them, except for the delay line reads that reach DIL two
		// instructions later: skip to the last two of the run and set up the RAM pipeline as they left it
		if (nop_steps >= 2 && s_dec[pc].nop_run > 2) {
			int k = s_dec[pc].nop_run - 2;
			if (n + k < max_steps) {
				pc = (uint8_t)(pc + k);
				n += k;
				DELAY_ADDR((uint8_t)(pc - 1), ram_a);
				DELAY_ADDR((uint8_t)(pc - 2), ram_p_a);
				ram_io = ram_p_io = 0;
				ram_cy = ram_p_cy = RC_READ;
			}
		}
		ram_pp_a = ram_p_a; ram_pp_io = ram_p_io; ram_pp_cy = ram_p_cy;
		ram_p_a = ram_a; ram_p_io = ram_io; ram_p_cy = ram_cy;
		const es5510_dec *d = &s_dec[pc < 0xa0 ? pc : 0xa0];
		if (ram_pp_cy != RC_WRITE)
			dil = ram_pp_io ? 0 : dram[ram_pp_a & (ES5510_DRAM_SIZE - 1)];
		ram_cy = d->cycle;
		ram_io = d->access == RA_IO;
		int32_t offset = pc < 0xc0 ? gpr[pc] : 0;
		switch (d->access) {
		case RA_DELAY: {
			int32_t m = dlength + memincrement;
			int32_t x = dbase + offset;
			x = delay_mod(x, m);
			ram_a = (x & memmask) >> memshift;
			break;
		}
		case RA_TABLE_A: ram_a = ((e->abase + offset) & memmask) >> memshift; break;
		case RA_TABLE_B: ram_a = ((e->bbase + offset) & memmask) >> memshift; break;
		default: ram_a = offset & 0x00fffff0; break;
		}
		int skip = 0;
		if (d->skippable) {
			int8_t cmr = e->cmr;
			int sat = (ccr & cmr & FLAG_MASK) != 0;
			if (cmr & FLAG_NOT)
				sat = !sat;
			skip = sat;
		}
		// multiplier result N-1
		if (mac.write_result && !mac_nop) {
			mac.product = ((int64_t)sext24(mac.cValue) * (int64_t)sext24(mac.dValue)) * ((int64_t)1 << mulshift);
			mac.result = mac.accumulate ? mac.product + machl : mac.product;
			mac_overflow = mac.result < -((int64_t)1 << 47) || mac.result >= ((int64_t)1 << 47);
			machl = mac.result;
			int32_t tmp = mac_overflow ? (machl < 0 ? 0x00800000 : 0x007fffff) : (int32_t)((mac.result & 0x0000ffffff000000LL) >> 24);
			if (mac.dst & SD_REG)
				WREG(mac.cReg, tmp);
			if (mac.dst & SD_DELAY)
				DOL(tmp);
		}
		// multiplier cycle N
		mac.cReg = d->cReg;
		mac.dReg = d->dReg;
		mac.src = d->msrc;
		mac.dst = d->mdst;
		mac.accumulate = d->accumulate;
		mac.write_result = !skip;
		mac_nop = d->mac_nop;
		if (!mac_nop) {
			if (mac.src == SD_REG)
				RREG(mac.cReg, mac.cValue);
			else
				mac.cValue = (int32_t)((uint32_t)(int32_t)dil << 8);
			RREG(mac.dReg, mac.dValue);
		}
		// ALU result N-1
		if (alu.write_result && !alu_nop) {
			uint8_t flags = (uint8_t)ccr;
			alu.result = alu_operation(alu.op, alu.aValue, alu.bValue, &flags);
			if (alu.op != OP_CMP) {
				if (alu.dst & SD_REG)
					WREG(alu.aReg, alu.result);
				if (alu.dst & SD_DELAY)
					DOL(alu.result);
			}
			if (alu.update_ccr)
				ccr = (int8_t)flags;
		}
		// ALU cycle N
		alu.aReg = d->aReg;
		alu.bReg = d->bReg;
		alu.op = d->op;
		alu.src = d->asrc;
		alu.dst = d->adst;
		alu.write_result = !skip || alu.op == OP_CMP;
		alu.update_ccr = !d->skippable || alu.op == OP_CMP;
		int ended = 0;
		alu_nop = d->alu_nop;
		if (alu_nop) {
		} else if (alu.op == 0xf) {
			// MAME alu_operation_end
			if (e->halt_asserted) {
				e->state = ST_HALTED;
				e->host_control &= ~0x04;
				ended = 1;
			}
			dbase -= memincrement;
			if (dbase < 0)
				dbase = dlength;
			if (e->state == ST_RUNNING)
				pc = 0;
		} else if (d->operands == 1) {
			if (alu.src == SD_REG)
				RREG(alu.bReg, alu.bValue);
			else
				alu.bValue = (int32_t)((uint32_t)(int32_t)dil << 8);
		} else {
			if (alu.src == SD_REG)
				RREG(alu.aReg, alu.aValue);
			else
				alu.aValue = (int32_t)((uint32_t)(int32_t)dil << 8);
			RREG(alu.bReg, alu.bValue);
		}
		// RAM cycle N-1
		if (ram_p_cy != RC_READ) {
			if (ram_p_cy == RC_WRITE && !ram_p_io)
				dram[ram_p_a & (ES5510_DRAM_SIZE - 1)] = dol0;
			dol0 = dol1;
			if (dol_count > 0)
				dol_count--;
		}
		nop_steps = d->nop ? nop_steps + 1 : 0;
		pc++;
		if (ended)
			break;
	}
	SPILL();
	e->pc = pc;
	e->alu = alu;
	e->mulacc = mac;
	e->ram.address = ram_a; e->ram.io = ram_io; e->ram.cycle = ram_cy;
	e->ram_p.address = ram_p_a; e->ram_p.io = ram_p_io; e->ram_p.cycle = ram_p_cy;
	e->ram_pp.address = ram_pp_a; e->ram_pp.io = ram_pp_io; e->ram_pp.cycle = ram_pp_cy;
#undef SPILL
#undef FILL
#undef RREG
#undef WREG
#undef DOL
#undef DELAY_ADDR
}

// exec with the in-flight instruction as a pointer to its decoded entry (pd), its skip state (pf) and operand values, with the
// instruction kinds precomputed at decode. Same steps as exec, skippable instructions included.
#if defined(__arm__)
__attribute__((target("arm")))   // (ARM rather than Thumb-2 code for the ESP loop: measured faster on the Vita)
#endif
static void exec_noskip(es5510_t *e, int max_steps)
{
	int32_t *gpr = e->gpr;
	int16_t *dram = e->dram;
	uint8_t pc = e->pc;
	int16_t dil = e->dil;
	int16_t dol0 = e->dol[0], dol1 = e->dol[1];
	int32_t dol_count = e->dol_count;
	int8_t ccr = e->ccr;
	int64_t machl = e->machl;
	uint8_t mac_overflow = e->mac_overflow;
	int32_t dbase = e->dbase, dlength = e->dlength, memmask = e->memmask, memincrement = e->memincrement;
	int8_t memshift = e->memshift;
	int32_t mulshift = e->mulshift;
	int32_t dmod = dlength + memincrement;
	// RAM pipeline stages packed in one word each: address (24 bits) | io << 24 | cycle << 25
#define RPK(st) ((uint32_t)(st).address | (uint32_t)((st).io != 0) << 24 | (uint32_t)(st).cycle << 25)
	uint32_t ram = RPK(e->ram), ram_p = RPK(e->ram_p), ram_pp = RPK(e->ram_pp);
#undef RPK
	// the instruction in flight, from the structure (it may come from another program or a skipped step)
	es5510_dec entry;
	memset(&entry, 0, sizeof(entry));
	entry.cReg = e->mulacc.cReg;
	entry.dReg = e->mulacc.dReg;
	entry.msrc = e->mulacc.src;
	entry.mdst = e->mulacc.dst;
	entry.accumulate = e->mulacc.accumulate;
	entry.aReg = e->alu.aReg;
	entry.bReg = e->alu.bReg;
	entry.op = e->alu.op;
	entry.asrc = e->alu.src;
	entry.adst = e->alu.dst;
	entry.mac_nop = (uint8_t)(!e->mulacc.write_result || mac_is_nop(entry.msrc, entry.mdst, entry.cReg, entry.dReg, entry.accumulate));
	entry.alu_nop = (uint8_t)(!e->alu.write_result || alu_is_nop(entry.op, entry.adst, entry.aReg));
	entry.mwk = write_kind(entry.mdst, entry.cReg);
	entry.awk = entry.op == OP_CMP ? 0 : write_kind(entry.adst, entry.aReg);
	// the in-flight instruction's skip state (PF_*): MAC result not written, ALU result not written, ccr kept
	enum { PF_MSKIP = 1, PF_ASKIP = 2, PF_NOCCR = 4 };
	unsigned pf = (e->mulacc.write_result ? 0 : PF_MSKIP) | (e->alu.write_result ? 0 : PF_ASKIP) |
		(e->alu.update_ccr ? 0 : PF_NOCCR);
	const es5510_dec *pd = &entry;
	int32_t cV = e->mulacc.cValue, dV = e->mulacc.dValue, aV = e->alu.aValue, bV = e->alu.bValue;
#define SPILL() do { e->dil = dil; e->dol[0] = dol0; e->dol[1] = dol1; e->dol_count = dol_count; e->ccr = ccr; \
	e->machl = machl; e->mac_overflow = mac_overflow; e->dbase = dbase; e->dlength = dlength; } while (0)
#define FILL() do { ccr = e->ccr; machl = e->machl; mac_overflow = e->mac_overflow; dbase = e->dbase; dlength = e->dlength; \
	memmask = e->memmask; memincrement = e->memincrement; memshift = e->memshift; mulshift = e->mulshift; 	dmod = dlength + memincrement; } while (0)
#define RREG(r, out) do { uint8_t r_ = (r); (out) = r_ < 0xc0 ? gpr[r_] : r_ >= 252 ? SPR_CONST[r_ - 252] : \
	read_spr_fast(e, r_, dil, ccr, machl, mac_overflow, dbase, dlength); } while (0)
#define WREG(r, v) do { uint8_t r_ = (r); if (r_ < 0xc0) gpr[r_] = (v) & 0x00ffffff; \
	else if (r_ >= 234 && r_ <= 241) write_ser(e, r_, (v)); \
	else if (r_ < 252) { SPILL(); write_reg(e, r_, (v)); FILL(); } } while (0)
#define WSPR(r, v) do { uint8_t r_ = (r); int32_t v_ = (v) & 0x00ffffff; 	switch (r_) { 	case 234: case 235: case 236: case 237: case 238: case 239: case 240: case 241: write_ser(e, r_, v_); break; 	case 242: machl = sext48((machl & ((int64_t)0x00ffffff << 24)) | v_); break; 	case 243: machl = sext48((machl & 0x00ffffff) | ((int64_t)v_ << 24)); mac_overflow = 0; break; 	case 245: dlength = v_; dmod = dlength + memincrement; break; 	case 246: e->abase = v_; break; 	case 247: e->bbase = v_; break; 	case 248: dbase = v_; break; 	case 250: ccr = (int8_t)((v_ >> 16) & FLAG_MASK); break; 	case 251: e->cmr = (int8_t)((v_ >> 16) & (FLAG_MASK | FLAG_NOT)); break; 	default: SPILL(); write_reg(e, r_, v_); FILL(); break; } } while (0)
#define DOL(v) do { int16_t d16_ = (int16_t)((v) >> 8); if (dol_count >= 2) { dol0 = dol1; dol1 = d16_; } \
	else if (dol_count++ == 0) dol0 = d16_; else dol1 = d16_; } while (0)
#define DELAY_ADDR(p, out) do { int32_t m_ = dlength + memincrement, x_ = dbase + ((p) < 0xc0 ? gpr[(p)] : 0); \
	x_ = delay_mod(x_, m_); (out) = (x_ & memmask) >> memshift; } while (0)

	int nop_steps = 0;
	for (int n = 0; n < max_steps; n++) {
		if (nop_steps >= 2 && s_dec[pc].nop_run > 2) {
			int k = s_dec[pc].nop_run - 2;
			if (n + k < max_steps) {
				pc = (uint8_t)(pc + k);
				n += k;
				int32_t a_;
				DELAY_ADDR((uint8_t)(pc - 1), a_);
				ram = (uint32_t)a_ | (uint32_t)RC_READ << 25;
				DELAY_ADDR((uint8_t)(pc - 2), a_);
				ram_p = (uint32_t)a_ | (uint32_t)RC_READ << 25;
				pd = &s_dec[(uint8_t)(pc - 1)];
				pf = 0;
			}
		}
		ram_pp = ram_p; ram_p = ram;
		const es5510_dec *d = &s_dec[pc];
		if ((ram_pp >> 25) != RC_WRITE)
			dil = (ram_pp & (1u << 24)) ? 0 : dram[ram_pp & (ES5510_DRAM_SIZE - 1)];
		int32_t ram_a;
		{
			int32_t offset = pc < 0xc0 ? gpr[pc] : 0;
			switch (d->access) {
			case RA_DELAY: ram_a = (delay_mod(dbase + offset, dmod) & memmask) >> memshift; break;
			case RA_TABLE_A: ram_a = ((e->abase + offset) & memmask) >> memshift; break;
			case RA_TABLE_B: ram_a = ((e->bbase + offset) & memmask) >> memshift; break;
			default: ram_a = offset & 0x00fffff0; break;
			}
			ram = (uint32_t)ram_a | d->rk;
		}
		// skip condition of instruction N (on ccr before the ALU result N-1, as exec)
		unsigned npf = 0;
		if (d->skippable) {
			int8_t cmr = e->cmr;
			int sat = (ccr & cmr & FLAG_MASK) != 0;
			if (cmr & FLAG_NOT)
				sat = !sat;
			if (d->op != OP_CMP)
				npf = sat ? PF_MSKIP | PF_ASKIP | PF_NOCCR : PF_NOCCR;
			else
				npf = sat ? PF_MSKIP : 0;
		}
		// multiplier result N-1
		if (!pd->mac_nop && !(pf & PF_MSKIP)) {
			// product and result go straight to the structure (only the state needs them)
			int64_t mac_product = ((int64_t)sext24(cV) * (int64_t)sext24(dV)) * ((int64_t)1 << mulshift);
			int64_t mac_result = pd->accumulate ? mac_product + machl : mac_product;
			e->mulacc.product = mac_product;
			e->mulacc.result = mac_result;
			// outside [-2^47, 2^47): the upper 32 bits are not the sign extension of their low 16
			int32_t hi = (int32_t)((uint64_t)mac_result >> 32);
			mac_overflow = (int32_t)((uint32_t)hi << 16) >> 16 != hi;
			machl = mac_result;
			int32_t tmp = mac_overflow ? (hi < 0 ? 0x00800000 : 0x007fffff) :
				(int32_t)((((uint32_t)hi << 8) | ((uint32_t)mac_result >> 24)) & 0x00ffffff);
			uint8_t k = pd->mwk;
			if (k & WK_GPR)
				gpr[pd->cReg] = tmp & 0x00ffffff;
			else if (k & WK_SPR)
				WSPR(pd->cReg, tmp);
			if (k & WK_DOL)
				DOL(tmp);
		}
		// multiplier cycle N
		if (!d->mac_nop) {
			if (d->msrc == SD_REG)
				RREG(d->cReg, cV);
			else
				cV = (int32_t)((uint32_t)(int32_t)dil << 8);
			RREG(d->dReg, dV);
		}
		// ALU result N-1
		if (!pd->alu_nop && !(pf & PF_ASKIP)) {
			uint8_t flags = (uint8_t)ccr;
			int32_t alu_result = alu_operation(pd->op, aV, bV, &flags);
			uint8_t k = pd->awk;
			if (k & WK_GPR)
				gpr[pd->aReg] = alu_result & 0x00ffffff;
			else if (k & WK_SPR)
				WSPR(pd->aReg, alu_result);
			if (k & WK_DOL)
				DOL(alu_result);
			e->alu.result = alu_result;
			if (!(pf & PF_NOCCR))
				ccr = (int8_t)flags;
		}
		// ALU cycle N
		int ended = 0;
		if (d->alu_nop) {
		} else if (d->op == 0xf) {
			if (e->halt_asserted) {
				e->state = ST_HALTED;
				e->host_control &= ~0x04;
				ended = 1;
			}
			dbase -= memincrement;
			if (dbase < 0)
				dbase = dlength;
			if (e->state == ST_RUNNING)
				pc = 0;
		} else if (d->operands == 1) {
			if (d->asrc == SD_REG)
				RREG(d->bReg, bV);
			else
				bV = (int32_t)((uint32_t)(int32_t)dil << 8);
		} else {
			if (d->asrc == SD_REG)
				RREG(d->aReg, aV);
			else
				aV = (int32_t)((uint32_t)(int32_t)dil << 8);
			RREG(d->bReg, bV);
		}
		// RAM cycle N-1
		if ((ram_p >> 25) != RC_READ) {
			if ((ram_p >> 24) == (RC_WRITE << 1))
				dram[ram_p & (ES5510_DRAM_SIZE - 1)] = dol0;
			dol0 = dol1;
			if (dol_count > 0)
				dol_count--;
		}
		nop_steps = d->nop ? nop_steps + 1 : 0;
		pd = d;
		pf = npf;
		pc++;
		if (ended)
			break;
	}
	SPILL();
	e->pc = pc;
	// the in-flight instruction back into the structure
	e->mulacc.cReg = pd->cReg;
	e->mulacc.dReg = pd->dReg;
	e->mulacc.src = pd->msrc;
	e->mulacc.dst = pd->mdst;
	e->mulacc.accumulate = pd->accumulate;
	e->mulacc.write_result = !(pf & PF_MSKIP);
	e->mulacc.cValue = cV;
	e->mulacc.dValue = dV;
	e->alu.aReg = pd->aReg;
	e->alu.bReg = pd->bReg;
	e->alu.op = pd->op;
	e->alu.src = pd->asrc;
	e->alu.dst = pd->adst;
	e->alu.write_result = !(pf & PF_ASKIP);
	e->alu.update_ccr = !(pf & PF_NOCCR);
	e->alu.aValue = aV;
	e->alu.bValue = bV;
#define RUNPK(st, w) do { (st).address = (int32_t)((w) & 0x00ffffff); (st).cycle = (uint8_t)((w) >> 25); 	(st).io = (uint8_t)((w) >> 24 & 1); } while (0)
	RUNPK(e->ram, ram);
	RUNPK(e->ram_p, ram_p);
	RUNPK(e->ram_pp, ram_pp);
#undef RUNPK
#undef SPILL
#undef FILL
#undef RREG
#undef WREG
#undef DOL
#undef WSPR
#undef DELAY_ADDR
}

#if defined(__arm__)
#include "es5510_jit.inc"
#if !defined(__vita__)
#include <stdio.h>
// PC harness check (ARM build, RBX_ESPCHK=N): every Nth compiled pass also runs exec_noskip from the same state, the
// results compared
static void jit_check(es5510_t *e)
{
	static es5510_t before, ref, x, y;
	static int16_t *bd, *rd;
	static unsigned long long n, bad;
	if (!bd) {
		bd = (int16_t *)malloc(ES5510_DRAM_SIZE * sizeof(int16_t));
		rd = (int16_t *)malloc(ES5510_DRAM_SIZE * sizeof(int16_t));
	}
	memcpy(&before, e, sizeof(before));
	memcpy(bd, e->dram, ES5510_DRAM_SIZE * sizeof(int16_t));
	if (!jit_run(e)) {
		exec_noskip(e, 512);
		return;
	}
	memcpy(&ref, &before, sizeof(ref));
	ref.dram = rd;
	memcpy(rd, bd, ES5510_DRAM_SIZE * sizeof(int16_t));
	exec_noskip(&ref, 512);
	memcpy(&x, e, sizeof(x));
	memcpy(&y, &ref, sizeof(y));
	x.dram = y.dram = NULL;
	n++;
	if (memcmp(&x, &y, sizeof(x)) || memcmp(e->dram, rd, ES5510_DRAM_SIZE * sizeof(int16_t))) {
		if (bad++ < 5) {
			const uint8_t *p = (const uint8_t *)&x, *q = (const uint8_t *)&y;
			size_t i = 0;
			while (i < sizeof(x) && p[i] == q[i])
				i++;
			rbx_log("ESPCHK check %llu differs at struct byte %u of %u\n", n, (unsigned)i, (unsigned)sizeof(x));
		}
		int16_t *d = e->dram;
		memcpy(e, &ref, sizeof(*e));
		e->dram = d;
		memcpy(d, rd, ES5510_DRAM_SIZE * sizeof(int16_t));
	} else if (n % 1000 == 0)
		rbx_log("ESPCHK %llu checks, %llu differ\n", n, bad);
}
#endif
#endif

#if !defined(__arm__)
void es5510_jit_install(void) {}
#endif

unsigned es5510_interp_runs;   // (profiling: the samples run by the interpreter)
void es5510_run_once(es5510_t *e)
{
	// MAME run_once: one instruction with HALT off, then HALT on and run to the END that halts
	if (s_dec_dirty)
		decode_all(e);
	e->halt_asserted = 0;
	if (e->state == ST_HALTED) {
		e->state = ST_RUNNING;
		e->host_control |= 0x04;
		e->pc = 0;
	} else
		exec_noskip(e, 1);
	e->halt_asserted = 1;
	// (bounded: a program without END would run forever)
	if (e->state != ST_HALTED) {
#if defined(__arm__)
#if !defined(__vita__)
		static int chk = -1;
		static unsigned cnt;
		if (chk < 0) {
			const char *c = getenv("RBX_ESPCHK");
			chk = c ? atoi(c) : 0;
		}
		if (chk > 0 && ++cnt % (unsigned)chk == 0) {
			jit_check(e);
			return;
		}
#endif
		if (jit_run(e))
			return;
#endif
		es5510_interp_runs++;
		exec_noskip(e, 512);
	}
}

int es5510_init(es5510_t *e)
{
	int16_t *d = (int16_t *)calloc(ES5510_DRAM_SIZE, sizeof(int16_t));
	memset(e, 0, sizeof(*e));
	e->dram = d;
	if (!d)
		return -1;
	es5510_reset(e);
	return 0;
}

void es5510_exit(es5510_t *e)
{
	free(e->dram);
	e->dram = NULL;
}

void es5510_reset(es5510_t *e)
{
	s_dec_dirty = 1;
	int16_t *d = e->dram;
	memset(e, 0, sizeof(*e));
	e->dram = d;
	if (d)
		memset(d, 0, ES5510_DRAM_SIZE * sizeof(int16_t));
	// constructor defaults, then device_reset
	e->memsiz = 0x00ffffff;
	e->memmask = 0;
	e->memincrement = 0x01000000;
	e->memshift = 24;
	e->mulshift = 2;
	e->state = ST_RUNNING;
	e->host_control = 0x04;
}

uint8_t es5510_host_r(es5510_t *e, int offset)
{
	switch (offset) {
	case 0x00: return (uint8_t)(e->gpr_latch >> 16);
	case 0x01: return (uint8_t)(e->gpr_latch >> 8);
	case 0x02: return (uint8_t)e->gpr_latch;
	case 0x03: return (uint8_t)(e->instr_latch >> 40);
	case 0x04: return (uint8_t)(e->instr_latch >> 32);
	case 0x05: return (uint8_t)(e->instr_latch >> 24);
	case 0x06: return (uint8_t)(e->instr_latch >> 16);
	case 0x07: return (uint8_t)(e->instr_latch >> 8);
	case 0x08: return (uint8_t)e->instr_latch;
	case 0x09: return (uint8_t)(e->dil_latch >> 16);
	case 0x0a: return (uint8_t)(e->dil_latch >> 8);
	case 0x0b: return 0;
	case 0x0c: return (uint8_t)(e->dol_latch >> 16);
	case 0x0d: return (uint8_t)(e->dol_latch >> 8);
	case 0x0e: return 0xff;
	case 0x0f: return (uint8_t)(e->dadr_latch >> 16);
	case 0x10: return (uint8_t)(e->dadr_latch >> 8);
	case 0x11: return (uint8_t)e->dadr_latch;
	case 0x12: return 0;
	case 0x16: return 0x27;
	}
	return 0;
}

void es5510_host_w(es5510_t *e, int offset, uint8_t data)
{
	switch (offset) {
	case 0x00: e->gpr_latch = (e->gpr_latch & 0x00ffff) | (data << 16); break;
	case 0x01: e->gpr_latch = (e->gpr_latch & 0xff00ff) | (data << 8); break;
	case 0x02: e->gpr_latch = (e->gpr_latch & 0xffff00) | data; break;
	case 0x03: e->instr_latch = (e->instr_latch & 0x00ffffffffffull) | ((uint64_t)data << 40); break;
	case 0x04: e->instr_latch = (e->instr_latch & 0xff00ffffffffull) | ((uint64_t)data << 32); break;
	case 0x05: e->instr_latch = (e->instr_latch & 0xffff00ffffffull) | ((uint64_t)data << 24); break;
	case 0x06: e->instr_latch = (e->instr_latch & 0xffffff00ffffull) | ((uint64_t)data << 16); break;
	case 0x07: e->instr_latch = (e->instr_latch & 0xffffffff00ffull) | ((uint64_t)data << 8); break;
	case 0x08: e->instr_latch = (e->instr_latch & 0xffffffffff00ull) | data; break;
	case 0x0c: e->dol_latch = (e->dol_latch & 0x00ffff) | (data << 16); break;
	case 0x0d: e->dol_latch = (e->dol_latch & 0xff00ff) | (data << 8); break;
	case 0x0e: e->dol_latch = (e->dol_latch & 0xffff00) | data; break;
	case 0x0f:
		e->dadr_latch = (e->dadr_latch & 0x00ffff) | ((uint32_t)data << 16);
		if (e->ram_sel)
			e->dil_latch = dram_r(e, (int32_t)e->dadr_latch) * 256;
		else
			dram_w(e, (int32_t)e->dadr_latch, (int16_t)(e->dol_latch >> 8));
		break;
	case 0x10: e->dadr_latch = (e->dadr_latch & 0xff00ff) | ((uint32_t)data << 8); break;
	case 0x11: e->dadr_latch = (e->dadr_latch & 0xffff00) | data; break;
	case 0x12:
		e->host_control = (uint8_t)((e->host_control & 0x4) | (data & 0x3));
		if (e->host_control & 2) {
			if (e->state == ST_HALTED && e->dram)
				memset(e->dram, 0, ES5510_DRAM_SIZE * sizeof(int16_t));
			e->host_control &= ~0x2;
		}
		break;
	case 0x14: e->ram_sel = data & 0x80; break;
	case 0x18: e->host_serial = data; break;
	case 0x1f:
		if (e->halt_asserted)
			e->state = ST_HALTED;
		break;
	case 0x80:
		if (data < 0xa0) e->instr_latch = e->instr[data];
		if (data < 0xc0) e->gpr_latch = e->gpr[data] & 0xffffff;
		else if (data >= 0xea) e->gpr_latch = read_reg(e, data);
		break;
	case 0xa0: write_reg(e, data, e->gpr_latch); break;
	case 0xc0: if (data < 0xa0) e->instr[data] = e->instr_latch & 0xffffffffffffull; s_dec_dirty = 1; break;
	case 0xe0:
		if (data < 0xa0) e->instr[data] = e->instr_latch;
		s_dec_dirty = 1;
		write_reg(e, data, e->gpr_latch);
		break;
	}
}
