/*  i386 JIT (ADR-042): guest i386 code translated to host ARM code (included by i386.c; ARM hosts with executable
    memory from the frontend, rbx_jit_alloc). The instruction semantics are MAME's (i386ops.hxx, i386op16.hxx,
    i386op32.hxx, i386.cpp: BSD-3-Clause, copyright-holders: Ville Linde, Barry Rodewald, Carl, Philip Bennett): the
    flags as their ALU helpers set them, the cycles of their handlers, the interpreter's functions for the rest.
    This file is part of ReBOX. GPL-2.0-or-later.

    A block is the guest code from a pc up to an unconditional transfer, an instruction the decoder does not know, the
    end of its 64 KB page or 32 instructions; only code in read-only mapped pages (ROM) is translated, so no write can
    change it. Blocks run only in the state they were made for (ij_mode_ok: protected mode without paging, flat 32-bit
    CS / DS / ES / SS with base 0, no single step, no breakpoints, no RF, no lock, no pending interrupt or delayed IF):
    then no segment check or address translation can fault and an access is the page's memory or the bus handler.
    Every guest instruction is done as i386_step does it: the same cycles (the handler's cycle_table_pm entries, in its
    order against its accesses), the slice's end after it when the cycles ran out, a return to the run loop after an
    access through a bus handler that left an interrupt pending (the next step's test), after any transfer (the idle
    test of the run loop: I.pc below the instruction's) and after an instruction done by the interpreter that left the
    flow or the state. The guest registers and flags live in I. Native: mov, movzx / movsx, lea, push / pop, call /
    ret / jmp / jcc / loop / jecxz, the ALU ops (8 / 32 bit), test, inc / dec, setcc, xchg, cdq / cwde, the shifts
    (through MAME's i386_shift_rotate8 / 32); the rest is one i386_step_rf call each.
    Host registers in a block: r4 = &I, r5 = I.cycles (stored around calls), r6 = s_rmap, r7 = s_wmap, r8 = an
    interrupt pending after a handler access, r10 = the parity table, r11 = an address kept over an access. */

#if defined(__arm__)
#define I386_JIT 1
#include "i386_jitdec.h"
#include <stdio.h>

#define IJ_CODE_SIZE (6u << 20)   // (or 4 or 2 MB: what rbx_jit_alloc has left)
#define IJ_TAB_BITS 13
#define IJ_MAX_INSNS 32
#define IJ_NONE ((ij_fn)(uintptr_t)1)   // (a table entry: no block can be made at that pc)

// a block returns the pc of the last instruction it ran (the run loop's idle test: I.pc < it)
typedef uint32_t (*ij_fn)(void);
static uint32_t *s_ij_code;   // IJ_CODE_SIZE executable (NULL: none yet)
static int s_ij_none;
static unsigned s_ij_used, s_ij_size, s_sr_words;   // words (the first s_sr_words: the shared routines)
static struct { uint32_t pc; ij_fn fn; } s_ij_tab[1u << IJ_TAB_BITS][4];
static uint8_t s_ij_next[1u << IJ_TAB_BITS];
static int s_ij_on = -1;      // (-1: RBX_I386_NOJIT not read yet)
static int s_ij_build = 1;    // a block may be made at I.pc (the flow came there by a transfer or a block's end)
// the fields i386_step leaves after the last instruction when a block ran it (bit 31; bit 30: the block left at a
// transfer or its end), set in I at the run's end (only read by the next step, which sets them again)
static uint32_t s_ij_pack;
static uint32_t s_ij_xlr;      // (SR_EXIT's lr: the stub's data words)
static uint32_t *s_ij_slot;    // a chain slot to patch with a jump to the block at I.pc (NULL: none)
static int s_ij_maps_old = 1;  // (s_ij_r4k / s_ij_w4k to be made again: the maps changed)
static uint8_t s_ij_par[256];  // (i386_parity_table as bytes)
static int s_ij_stat;          // RBX_I386_JITSTAT=1: the interpreted instructions counted (i386_exit prints them)
static uint32_t s_ij_cnt[512];
static unsigned s_ij_nb, s_ij_ni;   // (blocks, instructions made since the code memory was last cleared)
static uint64_t s_ij_us;            // (the time in i386_run)
static int s_ij_dbg, s_ij_oplo = 0, s_ij_ophi = 0x1ff;   // (RBX_I386_JITDBG=bits,lo,hi: 1 no native, 2 no direct, 4 no chain; native opcodes lo-hi)
static uint32_t s_ij_runs, s_ij_steps, s_ij_isteps;   // (blocks run, steps of the run loop, in blocks)

static void ij_flush(void)
{
	memset(s_ij_tab, 0, sizeof(s_ij_tab));
	memset(s_ij_next, 0, sizeof(s_ij_next));
	s_ij_used = s_sr_words;   // (the shared routines stay)
	s_ij_build = 1;
	s_ij_slot = NULL;
	s_ij_maps_old = 1;
}

// the state the blocks are made for (checked before a block and after an instruction the interpreter did in one):
// DS, ES, SS valid expand-up writable data segments with base 0 and the limits the code was made for (s_ij_lim, the
// accesses checked against them: an instruction whose access would be out of them is left to the interpreter)
static uint32_t s_ij_lim[6] = { 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu };
static inline int ij_seg_ok(int s)
{
	const I386_SREG *g = &I.sreg[s];
	return g->valid && g->base == 0 && g->limit == s_ij_lim[s] && (g->flags & 0x0e) == 0x02;   // (data, up, writable)
}
static void ij_flush(void);
static void ij_limits(void)   // (not in the blocks' state: new segment limits, the code made again for them)
{
	if (I.sreg[DS].limit != s_ij_lim[DS] || I.sreg[ES].limit != s_ij_lim[ES] || I.sreg[SS].limit != s_ij_lim[SS]) {
		ij_flush();
		s_ij_lim[DS] = I.sreg[DS].limit;
		s_ij_lim[ES] = I.sreg[ES].limit;
		s_ij_lim[SS] = I.sreg[SS].limit;
	}
}
static inline int ij_mode_ok(void)
{
	return !(I.irq_state && I.IF) && !(!I.smm && I.smi) && (I.dr[7] & 0xff) == 0 && !I.TF && !I.RF && I.auto_clear_RF &&
	       !I.lock && !I.delayed_interrupt_enable && (I.cr[0] & (CR0_PE | CR0_PG)) == CR0_PE && !I.VM &&
	       I.a20_mask == ~0u && I.sreg[CS].d && I.sreg[CS].base == 0 && I.sreg[SS].d && ij_seg_ok(DS) &&
	       ij_seg_ok(ES) && ij_seg_ok(SS);
}

// (RBX_I386_JITTRACE=file: the state after each run, to compare a run with and without the JIT)
static FILE *s_ij_tf;
static uint32_t s_ij_tn;
static void ij_trace(void)
{
	if (!s_ij_tf)
		return;
	uint32_t h = 0;
	for (int i = 0x10000; i < 0x40000 && !(s_ij_tn & 255); i += 4)
		if (s_rmap[i >> 16]) {
			uint32_t v;
			memcpy(&v, s_rmap[i >> 16] + (i & 0xffff), 4);
			h = h * 31 + v;
		}
	fprintf(s_ij_tf, "%u pc %08x cy %d r %08x %08x %08x %08x %08x %08x %08x %08x f %d%d%d%d%d%d%d ram %08x\n", s_ij_tn++, I.pc,
	        I.cycles, I.reg.d[0], I.reg.d[1], I.reg.d[2], I.reg.d[3], I.reg.d[4], I.reg.d[5], I.reg.d[6], I.reg.d[7],
	        I.CF, I.ZF, I.SF, I.OF, I.PF, I.AF, I.IF, h);
}
static uint32_t s_ij_tdet = 0xffffffffu;   // (RBX_I386_JITTRACEN: that run's iterations too)
static void ij_trace1(void)
{
	fprintf(s_ij_tf, "  it pc %08x cy %d r %08x %08x %08x %08x %08x %08x %08x %08x f %d%d%d%d%d%d%d irq %d\n", I.pc, I.cycles,
	        I.reg.d[0], I.reg.d[1], I.reg.d[2], I.reg.d[3], I.reg.d[4], I.reg.d[5], I.reg.d[6], I.reg.d[7],
	        I.CF, I.ZF, I.SF, I.OF, I.PF, I.AF, I.IF, I.irq_state);
}
static void ij_apply(void)   // (the run's end: the last instruction's fields)
{
	const uint32_t p = s_ij_pack;
	if (!(p >> 31))
		return;
	s_ij_pack = 0;
	I.opcode = (uint8_t)p;
	I.operand_size = (p >> 8) & 1;
	I.address_size = (p >> 9) & 1;
	I.operand_prefix = (p >> 10) & 1;
	I.address_prefix = (p >> 11) & 1;
	I.segment_prefix = (p >> 12) & 1;
	I.xmm_operand_size = (p >> 10) & 1;   // (0x66 toggles it as the operand size)
	I.ext = 1;
}

// ---- helpers called by the generated code ----
// one instruction by the interpreter: 1 = the flow went on to next in the blocks' state
static int ij_interp(uint32_t pc, uint32_t next)
{
	I.eip = I.pc = pc;
	s_ij_pack = 0;
	s_ij_ok = 0;
	if (s_ij_stat) {
		s_ij_isteps++;
		i386_dinsn d;
		uint8_t b[16];
		for (int k = 0; k < 16; k++)
			b[k] = s_rmap[(pc + k) >> 16] ? s_rmap[(pc + k) >> 16][(pc + k) & 0xffff] : 0;
		i386_jit_decode(b, 1, &d);
		s_ij_cnt[(d.two ? 256 : 0) + d.op]++;
	}
	i386_step_rf();
	return I.pc == next && !I.halted && (s_ij_ok = ij_mode_ok());
}
static uint32_t s_ij_slow[257];   // (RBX_I386_JITSTAT: the slow accesses by 4 KB page below 1 MB, the rest)
#define IJ_SLOWCNT(ea) (s_ij_stat ? (void)s_ij_slow[(ea) < 0x100000u ? (ea) >> 12 : 256]++ : (void)0)
// the accesses the page maps do not cover: the interpreter's, with I.pc where its handler has it (the board's
// handlers read it); the value | an interrupt pending after it << 32
static uint64_t ij_rd8(uint32_t ea, uint32_t pc)
{
	I.eip = I.pc = pc;
	IJ_SLOWCNT(ea);
	const uint32_t v = READ8(ea);
	return v | (uint64_t)(I.irq_state && I.IF) << 32;
}
static uint64_t ij_rd16(uint32_t ea, uint32_t pc)
{
	I.eip = I.pc = pc;
	IJ_SLOWCNT(ea);
	const uint32_t v = READ16(ea);
	return v | (uint64_t)(I.irq_state && I.IF) << 32;
}
static uint64_t ij_rd32(uint32_t ea, uint32_t pc)
{
	I.eip = I.pc = pc;
	IJ_SLOWCNT(ea);
	const uint32_t v = READ32(ea);
	return v | (uint64_t)(I.irq_state && I.IF) << 32;
}
static uint32_t ij_wr8(uint32_t ea, uint32_t v, uint32_t pc)
{
	I.eip = I.pc = pc;
	IJ_SLOWCNT(ea);
	WRITE8(ea, (uint8_t)v);
	return I.irq_state && I.IF;
}
static uint32_t ij_wr16(uint32_t ea, uint32_t v, uint32_t pc)
{
	I.eip = I.pc = pc;
	IJ_SLOWCNT(ea);
	WRITE16(ea, (uint16_t)v);
	return I.irq_state && I.IF;
}
static uint32_t ij_wr32(uint32_t ea, uint32_t v, uint32_t pc)
{
	I.eip = I.pc = pc;
	IJ_SLOWCNT(ea);
	WRITE32(ea, v);
	return I.irq_state && I.IF;
}
// an instruction by its handler (the interpreter's step without its tests, the blocks' state holds): start = where
// the handler fetches from (after the opcode), info = the step's fields (ij_pack's bits; segment override << 13;
// 0x0f table << 16), p = the instruction. 0: on to next, 1: an interrupt left pending (leave at next), 2: the flow
// went elsewhere (a fault, a trap: leave with I.pc as it is)
static uint32_t ij_direct(uint32_t start, uint32_t info, uint32_t next, uint32_t p)
{
	s_ij_pack = 0;
	I.operand_size = (info >> 8) & 1;
	I.address_size = (info >> 9) & 1;
	I.operand_prefix = I.xmm_operand_size = (info >> 10) & 1;
	I.address_prefix = (info >> 11) & 1;
	I.segment_prefix = (info >> 12) & 1;
	if (I.segment_prefix)
		I.segment_override = (info >> 13) & 7;
	I.opcode = (uint8_t)info;
	I.prev_eip = p;
	I.eip = I.pc = start;
	if (s_ij_stat) {
		s_ij_isteps++;
		s_ij_cnt[((info >> 16) & 1 ? 256 : 0) + (info & 0xff)]++;
	}
	if ((info >> 16) & 1)
		(I.operand_size ? I.opcode_table2_32 : I.opcode_table2_16)[info & 0xff]();
	else
		(I.operand_size ? I.opcode_table1_32 : I.opcode_table1_16)[info & 0xff]();
	if (I.pc != next) {
		s_ij_ok = 0;
		return 2;
	}
	return I.irq_state && I.IF;
}
// the 4 KB parts of plain memory below 1 MB (i386_map_split's), for the accesses the 64 KB maps do not cover
static uint8_t *s_ij_r4k[256], *s_ij_w4k[256];
static void ij_maps(void)
{
	s_ij_maps_old = 0;
	memset(s_ij_r4k, 0, sizeof(s_ij_r4k));
	memset(s_ij_w4k, 0, sizeof(s_ij_w4k));
	for (int k = 0; k < s_nsplit; k++)
		if (s_split[k].page < 16)
			for (int j = 0; j < 16; j++) {
				s_ij_r4k[s_split[k].page * 16 + (uint32_t)j] = s_split[k].r[j];
				s_ij_w4k[s_split[k].page * 16 + (uint32_t)j] = s_split[k].w[j];
			}
}
// a chained backward transfer to t: i386_idle_see's count of it (1: it would probe, the block leaves for the run
// loop's i386_idle_see, nothing counted here)
static int ij_idlechk(uint32_t t)
{
	if (t != s_idle_tgt) {
		s_idle_tgt = t;
		s_idle_cnt = 0;
		return 0;
	}
	if (s_idle_cnt + 1 < 2) {
		s_idle_cnt++;
		return 0;
	}
	return 1;
}
// the shifts and rotates (their flags and cycles)
static uint32_t ij_sh8(uint32_t modrm, uint32_t v, uint32_t n) { return i386_shift_rotate8((uint8_t)modrm, v, (uint8_t)n); }
static uint32_t ij_sh32(uint32_t modrm, uint32_t v, uint32_t n) { return i386_shift_rotate32((uint8_t)modrm, v, (uint8_t)n); }

// ---- emitter ----
static uint32_t s_ij_buf[4096];   // a block being made
static unsigned s_ij_n;
#define IJ_BUF_LIMIT 3200          // (no new instruction past this many words)
#define IJ_MAXS 160
// the exits: stub k leaves with I.eip = I.pc = next, prev_eip = pc0, s_ij_pack = pack (ret: I.pc set already)
static struct { uint32_t next, pc0, pack; int ret; } s_stub[IJ_MAXS];
static unsigned s_nstub;
static unsigned s_site_at[IJ_MAXS * 3], s_site_to[IJ_MAXS * 3], s_nsite;   // branches to a stub / tail
static int s_ij_full;   // (no more stubs: the block is cut)

static void E(uint32_t w) { s_ij_buf[s_ij_n++] = w; }
#define IOFS(f) ((uint32_t)offsetof(i386_t, f))
#define OREG(r) (4u * (uint32_t)(r))
#define OREG8(r) (4u * ((uint32_t)(r) & 3) + ((uint32_t)(r) >> 2))
#define CY(x) ((uint32_t)s_cycle_table_pm[x])
static void e_ldr(int rt, uint32_t off) { E(0xe5940000u | (uint32_t)rt << 12 | off); }    // ldr rt, [r4, #off]
static void e_str(int rt, uint32_t off) { E(0xe5840000u | (uint32_t)rt << 12 | off); }    // str rt, [r4, #off]
static void e_ldrb(int rt, uint32_t off) { E(0xe5d40000u | (uint32_t)rt << 12 | off); }   // ldrb rt, [r4, #off]
static void e_strb(int rt, uint32_t off) { E(0xe5c40000u | (uint32_t)rt << 12 | off); }   // strb rt, [r4, #off]
static void e_ldrh(int rt, uint32_t off) { E(0xe1d400b0u | (uint32_t)rt << 12 | (off & 0xf0) << 4 | (off & 0xf)); }
static void e_strh(int rt, uint32_t off) { E(0xe1c400b0u | (uint32_t)rt << 12 | (off & 0xf0) << 4 | (off & 0xf)); }
static void e_mov(int rd, int rm) { E(0xe1a00000u | (uint32_t)rd << 12 | (uint32_t)rm); }
static void e_lsl(int rd, int rm, int n) { if (n) E(0xe1a00000u | (uint32_t)rd << 12 | (uint32_t)n << 7 | (uint32_t)rm); else if (rd != rm) e_mov(rd, rm); }
static void e_lsr(int rd, int rm, int n) { if (n) E(0xe1a00020u | (uint32_t)rd << 12 | (uint32_t)n << 7 | (uint32_t)rm); else if (rd != rm) e_mov(rd, rm); }
static uint32_t ubfx(int rd, int rn, int lsb, int w) { return 0xe7e00050u | (uint32_t)(w - 1) << 16 | (uint32_t)rd << 12 | (uint32_t)lsb << 7 | (uint32_t)rn; }
static void e_imm(int rd, uint32_t v)   // movw / movt
{
	E(0xe3000000u | (v & 0xf000u) << 4 | (uint32_t)rd << 12 | (v & 0xfffu));
	if (v >> 16)
		E(0xe3400000u | (v >> 16 & 0xf000u) << 4 | (uint32_t)rd << 12 | (v >> 16 & 0xfffu));
}
static int arm_imm(uint32_t v)   // the 12-bit operand for v, -1: none
{
	for (int r = 0; r < 16; r++) {
		const uint32_t x = r ? (v << (2 * r)) | (v >> (32 - 2 * r)) : v;
		if (x < 256)
			return r << 8 | (int)x;
	}
	return -1;
}
static void e_addi(int rd, int rn, uint32_t v)   // rd = rn + v (r12 for a large v)
{
	int k;
	if (!v) {
		if (rd != rn)
			e_mov(rd, rn);
	} else if ((k = arm_imm(v)) >= 0)
		E(0xe2800000u | (uint32_t)rn << 16 | (uint32_t)rd << 12 | (uint32_t)k);
	else if ((k = arm_imm(0u - v)) >= 0)
		E(0xe2400000u | (uint32_t)rn << 16 | (uint32_t)rd << 12 | (uint32_t)k);
	else {
		e_imm(12, v);
		E(0xe080000cu | (uint32_t)rn << 16 | (uint32_t)rd << 12);
	}
}
static void e_cyc(uint32_t n)   // r5 -= n (the handler's CYCLES)
{
	const int k = arm_imm(n);
	if (k >= 0)
		E(0xe2455000u | (uint32_t)k);
	else {
		e_imm(12, n);
		E(0xe045500cu);   // sub r5, r5, r12
	}
}
static void e_call(const void *fn)   // r12 = fn; blx r12 (r0-r3 the arguments), I.cycles = r5 around it
{
	e_str(5, IOFS(cycles));
	e_imm(12, (uint32_t)(uintptr_t)fn);
	E(0xe12fff3cu);
	e_ldr(5, IOFS(cycles));
}
static unsigned e_bcond(uint32_t cond) { E(cond << 28 | 0x0a000000u); return s_ij_n - 1; }
static void e_patch(unsigned at) { s_ij_buf[at] |= ((s_ij_n - at - 2) & 0x00ffffffu); }
static void e_bto(uint32_t cond, uint32_t to)   // a branch to stub `to`
{
	s_site_at[s_nsite] = s_ij_n;
	s_site_to[s_nsite++] = to;
	E(cond << 28 | 0x0a000000u);
}
// the routines all blocks share, at the start of the code memory (ij_code_init); bl to one patched at install
enum { SR_EXIT, SR_EXITR0, SR_RET, SR_INTERP, SR_IDLE, SR_DIRECT, SR_RD8, SR_RD16, SR_RD32, SR_WR8, SR_WR16, SR_WR32, SR_FLAGS,
       SR_N = SR_FLAGS + 5 * 3 };
static unsigned s_sr_at[SR_N];
static unsigned s_bl_at[1024], s_nbl;
static uint8_t s_bl_to[1024];
static void e_bl(int sr)
{
	if (s_nbl >= 1024) {
		s_ij_full = 1;
		s_nbl = 0;
	}
	s_bl_at[s_nbl] = s_ij_n;
	s_bl_to[s_nbl++] = (uint8_t)sr;
	E(0xeb000000u);
}
#define SR_F(kind, sh) (SR_FLAGS + (kind) * 3 + ((sh) == 0 ? 0 : (sh) == 16 ? 1 : 2))
static uint32_t e_newstub(uint32_t next, uint32_t pc0, uint32_t pack, int ret)
{
	if (s_nstub >= IJ_MAXS - 1) {
		s_ij_full = 1;
		return s_nstub - 1;   // (never taken: the block is thrown away)
	}
	s_stub[s_nstub].next = next;
	s_stub[s_nstub].pc0 = pc0;
	s_stub[s_nstub].pack = pack;
	s_stub[s_nstub].ret = ret;
	return s_nstub++;
}
static void e_imm2(int rd, uint32_t v)   // movw + movt (a fixed length)
{
	E(0xe3000000u | (v & 0xf000u) << 4 | (uint32_t)rd << 12 | (v & 0xfffu));
	E(0xe3400000u | (v >> 16 & 0xf000u) << 4 | (uint32_t)rd << 12 | (v >> 16 & 0xfffu));
}
#define IJ_ENTRY 11   // (words of e_begin: where a chained jump from another block enters, the registers set)
static void e_begin(void)
{
	s_ij_n = s_nstub = s_nsite = s_nbl = 0;
	s_ij_full = 0;
	E(0xe92d4df0u);   // push {r4-r8, r10, r11, lr}
	e_imm2(4, (uint32_t)(uintptr_t)&I);
	e_ldr(5, IOFS(cycles));
	e_imm2(6, (uint32_t)(uintptr_t)s_rmap);
	e_imm2(7, (uint32_t)(uintptr_t)s_wmap);
	E(0xe3a08000u);   // mov r8, #0
	e_imm2(10, s_ij_lim[DS]);
}
// the stubs after the block's code (bl to the exit routine and its data words), the branches to them patched.
// A transfer's (chain): the run loop's tests (the cycles, no interrupt pending after a handler access; a backward
// one: SR_IDLE, i386_idle_see's count unless it would probe), then a slot the run loop patches into a jump to the
// target's block (IJ_PACK_CHAIN: the slot just before the SR_EXIT bl; IJ_PACK_IDLE: SR_IDLE counted it)
#define IJ_PACK_CHAIN (1u << 29)
#define IJ_PACK_IDLE (1u << 28)
static void e_finish(void)
{
	unsigned stub_at[IJ_MAXS];
	for (unsigned k = 0; k < s_nstub; k++) {
		stub_at[k] = s_ij_n;
		if (s_stub[k].ret == 1) {
			e_bl(SR_RET);
			E(s_stub[k].pc0);
		} else if (s_stub[k].ret == 2) {
			const int idle = s_idle_on && s_stub[k].next < s_stub[k].pc0;
			E(0xe3550000u);                      // cmp r5, #0
			const unsigned b0 = e_bcond(0xd);    // ble -> x2
			E(0xe3580000u);                      // cmp r8, #0
			const unsigned b1 = e_bcond(0x1);    // bne -> x2
			unsigned b2 = 0;
			if (idle) {
				e_imm(0, s_stub[k].next);
				e_bl(SR_IDLE);
				E(0xe3500000u);                  // cmp r0, #0
				b2 = e_bcond(0x1);               // bne -> x2
			}
			E(0xe1a00000u);                      // the slot (nop)
			e_bl(SR_EXIT);
			E(s_stub[k].next);
			E(s_stub[k].pc0);
			E(s_stub[k].pack | IJ_PACK_CHAIN | (idle ? IJ_PACK_IDLE : 0));
			if (idle) {   // x2: not counted
				e_patch(b2);
				e_patch(b0);
				e_patch(b1);
				e_bl(SR_EXIT);
				E(s_stub[k].next);
				E(s_stub[k].pc0);
				E(s_stub[k].pack);
			} else {      // (x2: the slot's exit, IJ_PACK_CHAIN)
				s_ij_buf[b0] |= (s_ij_n - 4 - b0 - 2) & 0x00ffffffu;
				s_ij_buf[b1] |= (s_ij_n - 4 - b1 - 2) & 0x00ffffffu;
			}
		} else {
			e_bl(SR_EXIT);
			E(s_stub[k].next);
			E(s_stub[k].pc0);
			E(s_stub[k].pack);
		}
	}
	for (unsigned k = 0; k < s_nsite; k++)
		s_ij_buf[s_site_at[k]] |= (stub_at[s_site_to[k]] - s_site_at[k] - 2) & 0x00ffffffu;
}

// ---- memory ----
// a load of `size` bytes at r0 -> r0 (zero-extended): the page directly when mapped (and aligned), else the
// interpreter's access with I.pc = pcat (SR_RD*). Clobbers r1-r3, r12
static void e_load(int size, uint32_t pcat)
{
	E(0xe1a02820u);   // lsr r2, r0, #16
	E(0xe7963102u);   // ldr r3, [r6, r2, lsl #2]
	E(0xe3530000u);   // cmp r3, #0
	const unsigned miss = e_bcond(0x0);
	unsigned unal = 0;
	if (size > 1) {
		E(size == 4 ? 0xe3100003u : 0xe3100001u);   // tst r0, #3 / #1
		unal = e_bcond(0x1);
	}
	E(ubfx(2, 0, 0, 16));
	E(size == 4 ? 0xe7930002u : size == 2 ? 0xe19300b2u : 0xe7d30002u);   // ldr / ldrh / ldrb r0, [r3, r2]
	const unsigned done = e_bcond(0xe);
	e_patch(miss);
	if (unal)
		e_patch(unal);
	e_imm(1, pcat);
	e_bl(size == 4 ? SR_RD32 : size == 2 ? SR_RD16 : SR_RD8);
	e_patch(done);
}
// a store of `size` bytes of r1 at r0 (SR_WR* when not mapped). Clobbers r0-r3, r12
static void e_store(int size, uint32_t pcat)
{
	E(0xe1a02820u);   // lsr r2, r0, #16
	E(0xe7973102u);   // ldr r3, [r7, r2, lsl #2]
	E(0xe3530000u);   // cmp r3, #0
	const unsigned miss = e_bcond(0x0);
	unsigned unal = 0;
	if (size > 1) {
		E(size == 4 ? 0xe3100003u : 0xe3100001u);
		unal = e_bcond(0x1);
	}
	E(ubfx(2, 0, 0, 16));
	E(size == 4 ? 0xe7831002u : size == 2 ? 0xe18310b2u : 0xe7c31002u);   // str / strh / strb r1, [r3, r2]
	const unsigned done = e_bcond(0xe);
	e_patch(miss);
	if (unal)
		e_patch(unal);
	e_imm(2, pcat);
	e_bl(size == 4 ? SR_WR32 : size == 2 ? SR_WR16 : SR_WR8);
	e_patch(done);
}
// the exit before the instruction being made (nothing of it done: the interpreter does it): made when first used
static uint32_t s_pre_next, s_pre_pc0, s_pre_pack, s_pre_stub;
static int s_pre_made;
// the segment limit test of i386_translate (size 1: MAME's GetEA, PUSH, POP) or i386_limit_check(SS, ESP + 3) (off 3)
// for the offset in r0: out of it, the exit before the instruction. Clobbers r12
static void e_lim(int seg, uint32_t off)
{
	const uint32_t lim = s_ij_lim[seg];
	if (lim == 0xffffffffu && !off)
		return;
	if (!s_pre_made) {
		s_pre_stub = e_newstub(s_pre_next, s_pre_pc0, s_pre_pack, 0);
		s_pre_made = 1;
	}
	if (lim < off) {   // (never in the limit)
		e_bto(0xe, s_pre_stub);
		return;
	}
	if (!off && lim == s_ij_lim[DS])
		E(0xe150000au);   // cmp r0, r10 (DS's limit)
	else {
		e_imm(12, lim - off);
		E(0xe150000cu);   // cmp r0, r12
	}
	e_bto(0x8, s_pre_stub);   // bhi
}
// the effective address of the ModRM operand (32-bit addressing, modrm_to_EA; the segment's base 0) -> r0, its
// segment's limit tested (e_lim). Clobbers r1, r12
static void e_ea2(const i386_dinsn *d, int chk)
{
	const int mod = d->modrm >> 6, rm = d->modrm & 7;
	int base = rm, idx = -1, sc = 0;
	if (rm == 4) {
		base = d->sib & 7;
		if (base == 5 && mod == 0)
			base = -1;
		idx = (d->sib >> 3) & 7;
		if (idx == 4)
			idx = -1;
		sc = d->sib >> 6;
	} else if (rm == 5 && mod == 0)
		base = -1;
	const int seg = d->seg ? d->seg - 1 : (base == ESP || base == EBP) ? SS : DS;
	if (base >= 0) {
		e_ldr(0, OREG(base));
		if (idx >= 0) {
			e_ldr(1, OREG(idx));
			E(0xe0800001u | (uint32_t)sc << 7);   // add r0, r0, r1, lsl #sc
		}
		e_addi(0, 0, (uint32_t)d->disp);
	} else if (idx >= 0) {
		e_ldr(1, OREG(idx));
		e_lsl(0, 1, sc);
		e_addi(0, 0, (uint32_t)d->disp);
	} else
		e_imm(0, (uint32_t)d->disp);
	if (chk)
		e_lim(seg, 0);
}
#define e_ea(d) e_ea2(d, 1)

// ---- flags (host NZCV of the op on r1 (dst), r2 (src) -> r0, the operands shifted left by sh for 8 bits) ----
enum { FK_ADD, FK_SUB, FK_LOGIC, FK_INC, FK_TESTI };
static void sr_flags(int k, int sh);
// (inline: a shared routine costs a call and an indirect return per ALU op)
static void e_flags(int k, int sh) { sr_flags(k, sh); }
// (the shared routine SR_F(k, sh): bl right after the op, the host flags kept by the bl)
static void sr_flags(int k, int sh)
{
	E(0xe10f3000u);   // mrs r3, APSR
	E(ubfx(12, 3, 30, 1));
	e_strb(12, IOFS(ZF));
	E(ubfx(12, 3, 31, 1));
	e_strb(12, IOFS(SF));
	if (k == FK_ADD || k == FK_SUB) {
		E(ubfx(12, 3, 29, 1));
		if (k == FK_SUB)
			E(0xe22cc001u);   // eor r12, r12, #1 (ARM's carry: no borrow)
		e_strb(12, IOFS(CF));
	}
	if (k == FK_ADD || k == FK_SUB || k == FK_INC) {
		E(ubfx(12, 3, 28, 1));
		e_strb(12, IOFS(OF));
		E(0xe020c001u);   // eor r12, r0, r1
		E(0xe02cc002u);   // eor r12, r12, r2
		E(ubfx(12, 12, 4 + sh, 1));
		e_strb(12, IOFS(AF));
	} else {
		E(0xe3a0c000u);   // mov r12, #0
		e_strb(12, IOFS(CF));
		e_strb(12, IOFS(OF));
		if (k == FK_TESTI)
			e_strb(12, IOFS(AF));
	}
	E(ubfx(12, 0, sh, 8));
	e_imm(3, (uint32_t)(uintptr_t)s_ij_par);
	E(0xe7d3c00cu);   // ldrb r12, [r3, r12]
	e_strb(12, IOFS(PF));
}
// r0 = r1 <op> r2 with the host flags (0 add, 1 or, 2 adc, 3 sbb, 4 and, 5 sub, 6 xor, 7 cmp); returns the flags' kind
static int e_aluop(int op)
{
	switch (op) {
	case 0: E(0xe0910002u); return FK_ADD;      // adds
	case 1: E(0xe1910002u); return FK_LOGIC;    // orrs
	case 2:
		e_ldrb(3, IOFS(CF));
		E(0xe1b030a3u);   // lsrs r3, r3, #1 (C = CF)
		E(0xe0b10002u);   // adcs
		return FK_ADD;
	case 3:
		e_ldrb(3, IOFS(CF));
		E(0xe2733000u);   // rsbs r3, r3, #0 (C = !CF)
		E(0xe0d10002u);   // sbcs
		return FK_SUB;
	case 4: E(0xe0110002u); return FK_LOGIC;    // ands
	case 6: E(0xe0310002u); return FK_LOGIC;    // eors
	default: E(0xe0510002u); return FK_SUB;     // subs (sub, cmp)
	}
}
// the condition cc (jcc / setcc) -> r0 (0 / 1 before the polarity), cmp r0, #0; returns the host condition of
// "true" (NE, or EQ for the odd ones)
static uint32_t e_cond(int cc)
{
	switch (cc >> 1) {
	case 0: e_ldrb(0, IOFS(OF)); break;
	case 1: e_ldrb(0, IOFS(CF)); break;
	case 2: e_ldrb(0, IOFS(ZF)); break;
	case 3: e_ldrb(0, IOFS(CF)); e_ldrb(1, IOFS(ZF)); E(0xe1800001u); break;   // orr r0, r0, r1
	case 4: e_ldrb(0, IOFS(SF)); break;
	case 5: e_ldrb(0, IOFS(PF)); break;
	case 6: e_ldrb(0, IOFS(SF)); e_ldrb(1, IOFS(OF)); E(0xe0200001u); break;   // eor r0, r0, r1
	default:
		e_ldrb(0, IOFS(SF)); e_ldrb(1, IOFS(OF)); E(0xe0200001u);
		e_ldrb(1, IOFS(ZF)); E(0xe1800001u);
		break;
	}
	E(0xe3500000u);   // cmp r0, #0
	return (cc & 1) ? 0x0 : 0x1;
}

// the fields of i386_step after the instruction d (I.opcode: its last opcode byte)
static uint32_t ij_pack(const i386_dinsn *d, int natural)
{
	return 1u << 31 | (natural ? 1u << 30 : 0) | d->op | (uint32_t)d->op32 << 8 | (uint32_t)d->addr32 << 9 |
	       (uint32_t)(d->op32 != 1) << 10 | (uint32_t)(d->addr32 != 1) << 11 | (uint32_t)(d->seg != 0) << 12;
}
// after an instruction: the slice's end, an interrupt left pending by a handler access
static void e_end(const i386_dinsn *d, uint32_t p, uint32_t next, int mem)
{
	const uint32_t k = e_newstub(next, p, ij_pack(d, 0), 0);
	E(0xe3550000u);   // cmp r5, #0
	e_bto(0xd, k);    // ble
	if (mem) {
		E(0xe3580000u);   // cmp r8, #0
		e_bto(0x1, k);    // bne
	}
}
// leave to a known target (a transfer)
static void e_jump(const i386_dinsn *d, uint32_t p, uint32_t target)
{
	e_bto(0xe, e_newstub(target, p, ij_pack(d, 1), 2));
}
// leave to the target in r0
static void e_jump_r0(const i386_dinsn *d, uint32_t p)
{
	e_bl(SR_EXITR0);
	E(p);
	E(ij_pack(d, 1));
}
// the limit test of a push's ESP - 4 (before anything of the instruction is done). Clobbers r0, r12
static void e_pushchk(void)
{
	e_ldr(0, OREG(ESP));
	E(0xe2400004u);   // sub r0, r0, #4
	e_lim(SS, 0);
}
// push r1 (PUSH32: the write at ESP - 4, then ESP); chk: its limit test here
static void e_push(uint32_t pcat, int chk)
{
	e_ldr(0, OREG(ESP));
	E(0xe2400004u);   // sub r0, r0, #4
	if (chk)
		e_lim(SS, 0);
	e_mov(11, 0);
	e_store(4, pcat);
	e_str(11, OREG(ESP));
}
// pop -> r0 (POP32: the read at ESP, then ESP + 4)
static void e_pop(uint32_t pcat)
{
	e_ldr(0, OREG(ESP));
	e_lim(SS, 3);
	e_mov(11, 0);
	e_load(4, pcat);
	E(0xe28bb004u);   // add r11, r11, #4
	e_str(11, OREG(ESP));
}
// a register operand -> rt (size 1 / 2 / 4)
static void e_getreg(int rt, int r, int size)
{
	if (size == 4) e_ldr(rt, OREG(r));
	else if (size == 2) e_ldrh(rt, OREG(r));
	else e_ldrb(rt, OREG8(r));
}
static void e_setreg(int rt, int r, int size)
{
	if (size == 4) e_str(rt, OREG(r));
	else if (size == 2) e_strh(rt, OREG(r));
	else e_strb(rt, OREG8(r));
}

// an ALU op (0-7 as e_aluop; 8 test, 9 test with an immediate: AF = 0 too) of size 1 / 4: form 0 rm, reg; 1 reg,
// rm; 2 rm, imm; 3 the accumulator, imm. cr / cm: the cycles with a register / memory operand. 0: not native
static int e_alu(const i386_dinsn *d, int op, int size, int form, uint32_t imm, uint32_t cr, uint32_t cm, uint32_t p,
                 uint32_t next)
{
	const int sh = size == 1 ? 24 : size == 2 ? 16 : 0;
	if (sh && (op == 2 || op == 3))
		return 0;   // (adc / sbb 8: the interpreter)
	const int mem = form != 3 && (d->modrm >> 6) != 3;
	const int reg = (d->modrm >> 3) & 7, rm = d->modrm & 7;
	const int store = op != 7 && op < 8;
	if (mem) {
		e_ea(d);
		e_mov(11, 0);
		e_load(size, form == 2 ? next - d->imm_len : next);
		e_lsl(form == 1 ? 2 : 1, 0, sh);
	}
	if (form == 0 || form == 2) {   // dst = rm
		if (!mem) {
			e_getreg(1, rm, size);
			e_lsl(1, 1, sh);
		}
		if (form == 0) {
			e_getreg(2, reg, size);
			e_lsl(2, 2, sh);
		} else
			e_imm(2, imm << sh);
	} else if (form == 1) {         // dst = reg, src = rm
		if (!mem) {
			e_getreg(2, rm, size);
			e_lsl(2, 2, sh);
		}
		e_getreg(1, reg, size);
		e_lsl(1, 1, sh);
	} else {                        // the accumulator, imm
		e_getreg(1, 0, size);
		e_lsl(1, 1, sh);
		e_imm(2, imm << sh);
	}
	int fk;
	if (op >= 8) {
		E(0xe0110002u);   // ands
		fk = op == 9 ? FK_TESTI : FK_LOGIC;
	} else
		fk = e_aluop(op);
	e_flags(fk, sh);
	if (store) {
		if (mem && form != 1) {
			e_lsr(1, 0, sh);
			e_mov(0, 11);
			e_store(size, next);
		} else {
			e_lsr(0, 0, sh);
			e_setreg(0, form == 1 ? reg : form == 3 ? 0 : rm, size);
		}
	}
	e_cyc(mem ? cm : cr);
	e_end(d, p, next, mem);
	return 1;
}

// inc / dec (dec: 1) of the operand (32 or 8 bits; reg >= 0: that register, else the ModRM operand)
static void e_incdec(const i386_dinsn *d, int dec, int size, int reg, uint32_t p, uint32_t next)
{
	const int sh = size == 1 ? 24 : size == 2 ? 16 : 0;
	const int mem = reg < 0 && (d->modrm >> 6) != 3;
	if (reg < 0 && !mem)
		reg = d->modrm & 7;
	if (mem) {
		e_ea(d);
		e_mov(11, 0);
		e_load(size, next);
		e_lsl(1, 0, sh);
	} else {
		e_getreg(1, reg, size);
		e_lsl(1, 1, sh);
	}
	E(0xe3a02000u | (sh == 24 ? 0x401u : sh == 16 ? 0x801u : 1u));   // mov r2, #1 << sh
	E(dec ? 0xe0510002u : 0xe0910002u);             // subs / adds r0, r1, r2
	e_flags(FK_INC, sh);
	if (mem) {
		e_lsr(1, 0, sh);
		e_mov(0, 11);
		e_store(size, next);
	} else {
		e_lsr(0, 0, sh);
		e_setreg(0, reg, size);
	}
	e_cyc(mem ? (dec ? CY(CYCLES_DEC_MEM) : CY(CYCLES_INC_MEM)) : (dec ? CY(CYCLES_DEC_REG) : CY(CYCLES_INC_REG)));
	e_end(d, p, next, mem);
}

// a shift / rotate group (C0 C1 D0 D1 D2 D3) through MAME's i386_shift_rotate8 / 32 (its flags and cycles)
static void e_shift(const i386_dinsn *d, int size, int how, uint32_t p, uint32_t next)
{
	const int mem = (d->modrm >> 6) != 3, rm = d->modrm & 7;
	if (mem) {
		e_ea(d);
		e_mov(11, 0);
		e_load(size, how == 0 ? next - 1 : next);   // (C0 / C1: the count fetched after the read)
		e_mov(1, 0);
	} else
		e_getreg(1, rm, size);
	if (how == 0)
		E(0xe3a02000u | (d->imm & 0x1f));   // mov r2, #imm & 31
	else if (how == 1)
		E(0xe3a02001u);                      // mov r2, #1
	else
		e_ldrb(2, OREG8(1));                 // CL
	e_imm(0, d->modrm);
	e_call(size == 4 ? (const void *)ij_sh32 : (const void *)ij_sh8);
	if (mem) {
		e_mov(1, 0);
		e_mov(0, 11);
		e_store(size, next);
	} else
		e_setreg(0, rm, size);
	e_end(d, p, next, mem);
}

// the instruction d at p natively: 1 = done (its exits made), 2 = done and the block ends (a transfer), 0 = not
static int ij_native(const i386_dinsn *d, uint32_t p, uint32_t next)
{
	if (d->rep || d->lock || !d->addr32 || (d->seg && d->seg != 1 && d->seg != 3 && d->seg != 4))
		return 0;   // (ES, SS, DS overrides: flat)
	const int op = d->op, mod = d->modrm >> 6, reg = (d->modrm >> 3) & 7, rm = d->modrm & 7;
	const int mem = d->has_modrm && mod != 3;
	const int wsz = d->op32 ? 4 : 2;   // (the operand size of the word ops)
	if (!d->op32) {   // (16-bit operands: the moves, the ALU ops, test, inc / dec)
		if (d->two ? !(op == 0xb6 || op == 0xbe)
		           : !((op < 0x40 && (op & 7) < 6 && (op & 1)) || op == 0x81 || op == 0x83 || op == 0x85 ||
		               op == 0xa9 || (op == 0xf7 && reg == 0) || (op >= 0x40 && op <= 0x4f) ||
		               (op == 0xff && reg <= 1) || op == 0x89 || op == 0x8b || op == 0xc7 ||
		               (op >= 0xb8 && op <= 0xbf) || op == 0xa1 || op == 0xa3 || op == 0x98))
			return 0;
	}
	if (d->seg) {     // (the interpreter's I.segment_override, kept after the instruction)
		E(0xe3a00000u | (uint32_t)(d->seg - 1));   // mov r0, #seg
		e_str(0, IOFS(segment_override));
	}
	if (d->two) {
		if (op >= 0x80 && op <= 0x8f) {   // jcc rel32
			const uint32_t t = next + d->imm;
			const uint32_t c = e_cond(op & 15);
			const unsigned nt = e_bcond(c ^ 1);
			e_cyc(CY(CYCLES_JCC_FULL_DISP));
			e_jump(d, p, t);
			e_patch(nt);
			e_cyc(CY(CYCLES_JCC_FULL_DISP_NOBRANCH));
			e_end(d, p, next, 0);
			return 1;
		}
		if (op >= 0x90 && op <= 0x9f) {   // setcc rm8
			if (mem) {
				e_ea(d);
				e_mov(11, 0);
			}
			const uint32_t c = e_cond(op & 15);
			E(0xe3a01000u);              // mov r1, #0
			E(c << 28 | 0x03a01001u);    // mov<c> r1, #1
			if (mem) {
				e_mov(0, 11);
				e_store(1, next);
				e_cyc(CY(CYCLES_SETCC_MEM));
			} else {
				e_strb(1, OREG8(rm));
				e_cyc(CY(CYCLES_SETCC_REG));
			}
			e_end(d, p, next, mem);
			return 1;
		}
		if (op == 0xb6 || op == 0xb7 || op == 0xbe || op == 0xbf) {   // movzx / movsx r32, rm8 / rm16
			const int size = (op & 1) ? 2 : 1, sx = op >= 0xbe;
			if (mem) {
				e_ea(d);
				e_load(size, next);
			} else
				e_getreg(0, rm, size);
			if (sx)
				E(size == 1 ? 0xe6af0070u : 0xe6bf0070u);   // sxtb / sxth r0, r0
			e_setreg(0, reg, wsz);   // (r16: its low half)
			e_cyc(sx ? (mem ? CY(CYCLES_MOVSX_MEM_REG) : CY(CYCLES_MOVSX_REG_REG))
			         : (mem ? CY(CYCLES_MOVZX_MEM_REG) : CY(CYCLES_MOVZX_REG_REG)));
			e_end(d, p, next, mem);
			return 1;
		}
		return 0;
	}
	if (op < 0x40 && (op & 7) < 6) {   // the ALU ops
		const int a = op >> 3, f = op & 7;
		const int size = (f & 1) ? wsz : 1;
		const int cmp = a == 7;
		if (f < 4) {
			const int form = (f & 2) ? 1 : 0;
			uint32_t cr = cmp ? CY(CYCLES_CMP_REG_REG) : CY(CYCLES_ALU_REG_REG);
			uint32_t cm = form ? (cmp ? CY(CYCLES_CMP_MEM_REG) : CY(CYCLES_ALU_MEM_REG))
			                   : (cmp ? CY(CYCLES_CMP_REG_MEM) : CY(CYCLES_ALU_REG_MEM));
			return e_alu(d, a, size, form, 0, cr, cm, p, next);
		}
		const uint32_t c = cmp ? CY(CYCLES_CMP_IMM_ACC) : CY(CYCLES_ALU_IMM_ACC);
		return e_alu(d, a, size, 3, d->imm, c, c, p, next);
	}
	if (op == 0x80 || op == 0x81 || op == 0x83) {
		const int size = op == 0x80 ? 1 : wsz;
		const uint32_t imm = op == 0x83 ? (uint32_t)(int32_t)(int8_t)d->imm : d->imm;
		const int cmp = reg == 7;
		return e_alu(d, reg, size, 2, imm, cmp ? CY(CYCLES_CMP_REG_REG) : CY(CYCLES_ALU_REG_REG),
		             cmp ? CY(CYCLES_CMP_REG_MEM) : CY(CYCLES_ALU_REG_MEM), p, next);
	}
	if (op == 0x84 || op == 0x85)
		return e_alu(d, 8, op == 0x85 ? wsz : 1, 0, 0, CY(CYCLES_TEST_REG_REG), CY(CYCLES_TEST_REG_MEM), p, next);
	if (op == 0xa8)
		return e_alu(d, 8, 1, 3, d->imm, CY(CYCLES_ALU_IMM_ACC), 0, p, next);
	if (op == 0xa9)
		return e_alu(d, 8, wsz, 3, d->imm, CY(CYCLES_TEST_IMM_ACC), 0, p, next);
	if ((op == 0xf6 || op == 0xf7) && reg == 0)
		return e_alu(d, 9, op == 0xf7 ? wsz : 1, 2, d->imm, CY(CYCLES_TEST_IMM_REG), CY(CYCLES_TEST_IMM_MEM), p, next);
	if (op >= 0x40 && op <= 0x4f) {
		e_incdec(d, op >= 0x48, wsz, op & 7, p, next);
		return 1;
	}
	if ((op == 0xfe || op == 0xff) && reg <= 1) {
		e_incdec(d, reg, op == 0xff ? wsz : 1, -1, p, next);
		return 1;
	}
	if (op == 0x88 || op == 0x89 || op == 0x8a || op == 0x8b) {   // mov
		const int size = !(op & 1) ? 1 : d->op32 ? 4 : 2;
		if (op <= 0x89) {   // rm <- reg
			if (mem) {
				e_ea(d);
				e_getreg(1, reg, size);
				e_store(size, next);
			} else {
				e_getreg(0, reg, size);
				e_setreg(0, rm, size);
			}
			e_cyc(mem ? CY(CYCLES_MOV_REG_MEM) : CY(CYCLES_MOV_REG_REG));
		} else {            // reg <- rm
			if (mem) {
				e_ea(d);
				e_load(size, next);
			} else
				e_getreg(0, rm, size);
			e_setreg(0, reg, size);
			e_cyc(mem ? CY(CYCLES_MOV_MEM_REG) : CY(CYCLES_MOV_REG_REG));
		}
		e_end(d, p, next, mem);
		return 1;
	}
	if ((op == 0xc6 || op == 0xc7) && reg == 0) {   // mov rm, imm
		const int size = op == 0xc6 ? 1 : d->op32 ? 4 : 2;
		if (mem) {
			e_ea(d);
			e_imm(1, d->imm);
			e_store(size, next);
		} else {
			e_imm(0, d->imm);
			e_setreg(0, rm, size);
		}
		e_cyc(mem ? CY(CYCLES_MOV_IMM_MEM) : CY(CYCLES_MOV_IMM_REG));
		e_end(d, p, next, mem);
		return 1;
	}
	if (op >= 0xb0 && op <= 0xbf) {   // mov r, imm
		e_imm(0, d->imm);
		e_setreg(0, op & 7, op >= 0xb8 ? wsz : 1);
		e_cyc(CY(CYCLES_MOV_IMM_REG));
		e_end(d, p, next, 0);
		return 1;
	}
	if (op >= 0xa0 && op <= 0xa3) {   // mov acc <-> moffs
		const int size = (op & 1) ? wsz : 1;
		if (d->imm > s_ij_lim[d->seg ? d->seg - 1 : DS])
			return 0;   // (out of the segment: the interpreter's fault)
		e_imm(0, d->imm);
		if (op <= 0xa1) {
			e_load(size, next);
			e_setreg(0, 0, size);
			e_cyc(op == 0xa1 ? CY(CYCLES_MOV_MEM_ACC) : CY(CYCLES_MOV_IMM_MEM));   // (MAME's)
		} else {
			e_getreg(1, 0, size);
			e_store(size, next);
			e_cyc(op == 0xa3 ? CY(CYCLES_MOV_ACC_MEM) : CY(CYCLES_MOV_MEM_ACC));
		}
		e_end(d, p, next, 1);
		return 1;
	}
	if (op == 0x8d && mem) {   // lea
		e_ea2(d, 0);
		e_str(0, OREG(reg));
		e_cyc(CY(CYCLES_LEA));
		e_end(d, p, next, 0);
		return 1;
	}
	if (op == 0x90) {   // nop
		e_cyc(CY(CYCLES_NOP));
		e_end(d, p, next, 0);
		return 1;
	}
	if (op >= 0x91 && op <= 0x97) {   // xchg eax, r32
		e_ldr(0, OREG(EAX));
		e_ldr(1, OREG(op & 7));
		e_str(1, OREG(EAX));
		e_str(0, OREG(op & 7));
		e_cyc(CY(CYCLES_XCHG_REG_REG));
		e_end(d, p, next, 0);
		return 1;
	}
	if (op == 0x87 && !mem) {   // xchg r32, r32
		e_ldr(0, OREG(reg));
		e_ldr(1, OREG(rm));
		e_str(1, OREG(reg));
		e_str(0, OREG(rm));
		e_cyc(CY(CYCLES_XCHG_REG_REG));
		e_end(d, p, next, 0);
		return 1;
	}
	if (op == 0xac || op == 0xad || op == 0xaa || op == 0xab) {   // lods, stos (not rep: 32-bit addresses)
		const int size = (op & 1) ? 4 : 1, st = op <= 0xab, ri = st ? EDI : ESI;
		if (size == 4 && !d->op32)
			return 0;
		e_ldr(0, OREG(ri));
		e_lim(st ? ES : d->seg ? d->seg - 1 : DS, st ? 0 : (uint32_t)size - 1);   // (lodsd: translate size 4)
		if (st) {
			e_getreg(1, EAX, size);
			e_store(size, next);
		} else {
			e_load(size, next);
			e_setreg(0, EAX, size);
		}
		e_ldr(1, OREG(ri));
		e_ldrb(2, IOFS(DF));
		E(0xe3520000u);                       // cmp r2, #0
		E(0x02811000u | (uint32_t)size);      // addeq r1, r1, #size
		E(0x12411000u | (uint32_t)size);      // subne r1, r1, #size
		e_str(1, OREG(ri));
		e_cyc(st ? CY(CYCLES_STOS) : CY(CYCLES_LODS));
		e_end(d, p, next, 1);
		return 1;
	}
	if (op == 0xc9 && d->op32) {   // leave (the stack 32-bit)
		e_ldr(0, OREG(EBP));
		e_lim(SS, 3);
		e_str(0, OREG(ESP));
		e_mov(11, 0);
		e_load(4, next);
		E(0xe28bb004u);   // add r11, r11, #4
		e_str(11, OREG(ESP));
		e_str(0, OREG(EBP));
		e_cyc(CY(CYCLES_LEAVE));
		e_end(d, p, next, 1);
		return 1;
	}
	if (op == 0xf8 || op == 0xf9 || op == 0xfc || op == 0xfd) {   // clc stc cld std
		E(0xe3a00000u | (op & 1));   // mov r0, #0 / #1
		e_strb(0, op <= 0xf9 ? IOFS(CF) : IOFS(DF));
		e_cyc(op == 0xf8 ? CY(CYCLES_CLC) : op == 0xf9 ? CY(CYCLES_STC) : op == 0xfc ? CY(CYCLES_CLD) : CY(CYCLES_STD));
		e_end(d, p, next, 0);
		return 1;
	}
	if (op == 0x98) {   // cwde / cbw
		e_ldr(0, OREG(EAX));
		E(d->op32 ? 0xe6bf0070u : 0xe6af0070u);   // sxth / sxtb r0, r0
		e_setreg(0, EAX, wsz);
		e_cyc(CY(CYCLES_CBW));
		e_end(d, p, next, 0);
		return 1;
	}
	if (op == 0x99) {   // cdq
		e_ldr(0, OREG(EAX));
		E(0xe1a00fc0u);   // asr r0, r0, #31
		e_str(0, OREG(EDX));
		e_cyc(CY(CYCLES_CWD));
		e_end(d, p, next, 0);
		return 1;
	}
	if (op >= 0x50 && op <= 0x57) {   // push r32
		e_ldr(1, OREG(op & 7));
		e_push(next, 1);
		e_cyc(CY(CYCLES_PUSH_REG_SHORT));
		e_end(d, p, next, 1);
		return 1;
	}
	if (op == 0x68 || op == 0x6a) {   // push imm
		e_imm(1, op == 0x6a ? (uint32_t)(int32_t)(int8_t)d->imm : d->imm);
		e_push(next, 1);
		e_cyc(CY(CYCLES_PUSH_IMM));
		e_end(d, p, next, 1);
		return 1;
	}
	if (op >= 0x58 && op <= 0x5f) {   // pop r32
		e_pop(next);
		e_str(0, OREG(op & 7));
		e_cyc(CY(CYCLES_POP_REG_SHORT));
		e_end(d, p, next, 1);
		return 1;
	}
	if (op == 0xff && reg == 6) {   // push rm32
		e_pushchk();
		if (mem) {
			e_ea(d);
			e_load(4, next);
			e_mov(1, 0);
		} else
			e_ldr(1, OREG(rm));
		e_push(next, 0);
		e_cyc(CY(CYCLES_PUSH_RM));
		e_end(d, p, next, 1);
		return 1;
	}
	if ((op >= 0x70 && op <= 0x7f) || op == 0xe3) {   // jcc rel8, jecxz
		const uint32_t t = next + (uint32_t)(int32_t)(int8_t)d->imm;
		uint32_t c;
		if (op == 0xe3) {
			e_ldr(0, OREG(ECX));
			E(0xe3500000u);   // cmp r0, #0
			c = 0x0;          // eq
		} else
			c = e_cond(op & 15);
		const unsigned nt = e_bcond(c ^ 1);
		e_cyc(op == 0xe3 ? CY(CYCLES_JCXZ) : CY(CYCLES_JCC_DISP8));
		e_jump(d, p, t);
		e_patch(nt);
		e_cyc(op == 0xe3 ? CY(CYCLES_JCXZ_NOBRANCH) : CY(CYCLES_JCC_DISP8_NOBRANCH));
		e_end(d, p, next, 0);
		return 1;
	}
	if (op == 0xe2) {   // loop
		const uint32_t t = next + (uint32_t)(int32_t)(int8_t)d->imm;
		e_ldr(0, OREG(ECX));
		E(0xe2500001u);   // subs r0, r0, #1
		e_str(0, OREG(ECX));
		e_cyc(CY(CYCLES_LOOP));   // (sub: the flags kept)
		e_bto(0x1, e_newstub(t, p, ij_pack(d, 1), 2));   // bne
		e_end(d, p, next, 0);
		return 1;
	}
	if (op == 0xeb || op == 0xe9) {   // jmp
		e_cyc(op == 0xeb ? CY(CYCLES_JMP_SHORT) : CY(CYCLES_JMP));
		e_jump(d, p, next + (op == 0xeb ? (uint32_t)(int32_t)(int8_t)d->imm : d->imm));
		return 2;
	}
	if (op == 0xe8) {   // call rel32
		e_imm(1, next);
		e_push(next, 1);
		e_cyc(CY(CYCLES_CALL));
		e_jump(d, p, next + d->imm);
		return 2;
	}
	if (op == 0xc3 || op == 0xc2) {   // ret (imm16)
		e_pop(next);
		if (op == 0xc2) {
			e_ldr(1, OREG(ESP));
			e_addi(1, 1, (uint32_t)(int32_t)(int16_t)d->imm);
			e_str(1, OREG(ESP));
		}
		e_cyc(op == 0xc3 ? CY(CYCLES_RET) : CY(CYCLES_RET_IMM));
		e_jump_r0(d, p);
		return 2;
	}
	if (op == 0xff && (reg == 2 || reg == 4)) {   // call / jmp rm32
		if (reg == 2)
			e_pushchk();
		if (mem) {
			e_ea(d);
			e_load(4, next);
		} else
			e_ldr(0, OREG(rm));
		if (reg == 2) {
			e_cyc(mem ? CY(CYCLES_CALL_MEM) : CY(CYCLES_CALL_REG));
			e_str(0, IOFS(eip));   // (the target kept over the push)
			e_imm(1, next);
			e_push(next, 0);
			e_ldr(0, IOFS(eip));
		} else
			e_cyc(mem ? CY(CYCLES_JMP_MEM) : CY(CYCLES_JMP_REG));
		e_jump_r0(d, p);
		return 2;
	}
	if (op == 0xc0 || op == 0xc1) {
		e_shift(d, op == 0xc1 ? 4 : 1, 0, p, next);
		return 1;
	}
	if (op >= 0xd0 && op <= 0xd3) {
		e_shift(d, (op & 1) ? 4 : 1, op <= 0xd1 ? 1 : 2, p, next);
		return 1;
	}
	return 0;
}

// the code memory, once
static int ij_code_init(void)
{
	if (s_ij_code)
		return 1;
	if (s_ij_none)
		return 0;
	for (unsigned sz = IJ_CODE_SIZE; sz >= (2u << 20) && !s_ij_code; sz -= 2u << 20) {   // (6, 4 or 2 MB)
		s_ij_code = (uint32_t *)rbx_jit_alloc(sz);
		s_ij_size = sz / 4;
	}
	if (!s_ij_code) {
		s_ij_none = 1;
		return 0;
	}
	for (int i = 0; i < 256; i++)
		s_ij_par[i] = (uint8_t)i386_parity_table[i];
	// the shared routines
	s_ij_n = 0;
	static const void *const rw[6] = { (const void *)ij_rd8, (const void *)ij_rd16, (const void *)ij_rd32,
	                                   (const void *)ij_wr8, (const void *)ij_wr16, (const void *)ij_wr32 };
	// SR_EXIT (bl, then the words next, pc0, pack): I.eip = I.pc = next, I.prev_eip = pc0, s_ij_pack = pack;
	// SR_EXITR0 (bl, then pc0, pack): the same to r0; SR_RET (bl, then pc0): I.pc set already. They return pc0
	s_sr_at[SR_EXIT] = s_ij_n;
	E(0xe89e0007u);   // ldm lr, {r0, r1, r2}
	e_imm(3, (uint32_t)(uintptr_t)&s_ij_xlr);
	E(0xe583e000u);   // str lr, [r3] (the stub's data: the run loop's chain patch)
	const unsigned b = e_bcond(0xe);
	s_sr_at[SR_EXITR0] = s_ij_n;
	E(0xe89e0006u);   // ldm lr, {r1, r2}
	e_patch(b);
	e_str(0, IOFS(eip));
	e_str(0, IOFS(pc));
	e_str(1, IOFS(prev_eip));
	e_imm(3, (uint32_t)(uintptr_t)&s_ij_pack);
	E(0xe5832000u);   // str r2, [r3]
	const unsigned b2 = e_bcond(0xe);
	s_sr_at[SR_RET] = s_ij_n;
	E(0xe59e1000u);   // ldr r1, [lr]
	e_patch(b2);
	e_str(5, IOFS(cycles));
	E(0xe1a00001u);   // mov r0, r1
	E(0xe8bd8df0u);   // pop {r4-r8, r10, r11, pc} (the block's frame)
	// SR_INTERP (r0 = pc, r1 = next), SR_RD* (r0 = ea, r1 = pc; r8 |= the interrupt), SR_WR* (r0, r1 = value, r2 = pc)
	// (SR_RD* / SR_WR*, r3 = the 64 KB page's memory or 0: an unaligned access inside a direct page, or one inside a
	// 4 KB part of plain memory below 1 MB (i386_map_split's: s_ij_r4k / s_ij_w4k), byte by byte as the
	// interpreter's prog_* do it; else the C helper)
	for (int k = -2; k < 7; k++) {   // (SR_IDLE: r0 = the target; SR_DIRECT: r0-r3 as ij_direct)
		s_sr_at[k == -2 ? SR_DIRECT : k < 0 ? SR_IDLE : k == 0 ? SR_INTERP : SR_RD8 + k - 1] = s_ij_n;
		if (k > 0) {
			const int size = 1 << ((k - 1) % 3), wr = k > 3;
			E(0xe3530000u);                   // cmp r3, #0
			const unsigned sp = e_bcond(0x0); // beq: a split part?
			unsigned c[4], nc = 0;
			if (size > 1) {
				E(0xe280c000u | (uint32_t)(size - 1));   // add r12, r0, #size - 1
				E(0xe02cc000u);                          // eor r12, r12, r0
				E(0xe1b0c82cu);                          // lsrs r12, r12, #16
				c[nc++] = e_bcond(0x1);                  // bne: over the page's end
			}
			E(ubfx(12, 0, 0, 16));
			E(0xe083300cu);                   // add r3, r3, r12
			const unsigned go = e_bcond(0xe);
			e_patch(sp);
			E(0xe3500601u);                   // cmp r0, #0x100000
			c[nc++] = e_bcond(0x2);           // bhs
			e_imm(3, (uint32_t)(uintptr_t)(wr ? s_ij_w4k : s_ij_r4k));
			E(0xe1a0c620u);                   // lsr r12, r0, #12
			E(0xe793310cu);                   // ldr r3, [r3, r12, lsl #2]
			E(0xe3530000u);                   // cmp r3, #0
			c[nc++] = e_bcond(0x0);           // beq
			if (size > 1) {
				E(0xe280c000u | (uint32_t)(size - 1));
				E(0xe02cc000u);
				E(0xe1b0c62cu);               // lsrs r12, r12, #12
				c[nc++] = e_bcond(0x1);
			}
			E(ubfx(12, 0, 0, 12));
			E(0xe083300cu);                   // add r3, r3, r12
			e_patch(go);
			if (!wr) {
				E(0xe5d30000u);               // ldrb r0, [r3]
				for (int i = 1; i < size; i++) {
					E(0xe5d3c000u | (uint32_t)i);                 // ldrb r12, [r3, #i]
					E(0xe180000cu | (uint32_t)(8 * i) << 7);      // orr r0, r0, r12, lsl #8i
				}
			} else {
				E(0xe5c31000u);               // strb r1, [r3]
				for (int i = 1; i < size; i++) {
					E(0xe1a0c021u | (uint32_t)(8 * i) << 7);      // lsr r12, r1, #8i
					E(0xe5c3c000u | (uint32_t)i);                 // strb r12, [r3, #i]
				}
			}
			E(0xe12fff1eu);                   // bx lr
			for (unsigned i = 0; i < nc; i++)
				e_patch(c[i]);
		}
		E(0xe92d4008u);   // push {r3, lr}
		e_call(k == -2 ? (const void *)ij_direct : k < 0 ? (const void *)ij_idlechk : k == 0 ? (const void *)ij_interp
		                                                                                       : rw[k - 1]);
		if (k > 0)
			E(k <= 3 ? 0xe1888001u : 0xe1888000u);   // orr r8, r8, r1 / r0
		E(0xe8bd8008u);   // pop {r3, pc}
	}
	for (int k = 0; k < 5; k++)
		for (int s = 0; s < 3; s++) {
			s_sr_at[SR_FLAGS + k * 3 + s] = s_ij_n;
			sr_flags(k, s == 0 ? 0 : s == 1 ? 16 : 24);
			E(0xe12fff1eu);   // bx lr
		}
	s_sr_words = s_ij_n;
	rbx_jit_write_begin();
	memcpy(s_ij_code, s_ij_buf, s_ij_n * 4);
	rbx_jit_write_end(s_ij_code, s_ij_n * 4);
	s_ij_used = s_sr_words;
	return 1;
}
static ij_fn ij_install(void)
{
	if (s_ij_used + s_ij_n > s_ij_size) {   // (full: all of it again)
		rbx_log("i386 jit: code memory full, cleared (%u blocks, %u instructions)", s_ij_nb, s_ij_ni);
		s_ij_nb = s_ij_ni = 0;
		ij_flush();
		if (s_ij_used + s_ij_n > s_ij_size)
			return NULL;
	}
	uint32_t *dst = s_ij_code + s_ij_used;
	for (unsigned k = 0; k < s_nbl; k++)
		s_ij_buf[s_bl_at[k]] |= (s_sr_at[s_bl_to[k]] - (s_ij_used + s_bl_at[k]) - 2) & 0x00ffffffu;
	rbx_jit_write_begin();
	memcpy(dst, s_ij_buf, s_ij_n * 4);
	rbx_jit_write_end(dst, s_ij_n * 4);
	s_ij_used += s_ij_n;
	s_ij_nb++;
	return (ij_fn)(void *)dst;
}

// an instruction ij_direct may do: no prefix that is an opcode handler itself (rep, lock), no transfer, nothing that
// loads a segment, the flags IF / TF / RF, a control or debug register, or halts (those leave the blocks' state)
static int ij_direct_ok(const i386_dinsn *d)
{
	if (d->rep || d->lock)
		return 0;
	const int op = d->op, reg = (d->modrm >> 3) & 7;
	if (d->two)
		return (op >= 0x90 && op <= 0x9f) || op == 0xa0 || op == 0xa3 || op == 0xa4 || op == 0xa5 || op == 0xa8 ||
		       op == 0xab || op == 0xac || op == 0xad || op == 0xaf || op == 0xb3 || (op >= 0xb6 && op <= 0xbf &&
		       op != 0xb8 && op != 0xb9);
	if (op == 0x07 || op == 0x17 || op == 0x1f || op == 0x8e || op == 0x9d || op == 0xfa || op == 0xfb || op == 0xf4 ||
	    (op >= 0xcc && op <= 0xcf) || op == 0xc4 || op == 0xc5 || (op >= 0x70 && op <= 0x7f) || (op >= 0xe0 && op <= 0xe3) ||
	    (op >= 0xe8 && op <= 0xeb) || op == 0x9a || op == 0xc2 || op == 0xc3 || op == 0xca || op == 0xcb ||
	    (op == 0xff && reg >= 2 && reg <= 5))
		return 0;
	return 1;
}

// translate the block at pc: NULL = none now, IJ_NONE = none ever (nothing translatable there)
static ij_fn ij_build(uint32_t pc)
{
	const uint8_t *page = s_rmap[pc >> 16];
	if (!page || s_wmap[pc >> 16])
		return IJ_NONE;
	e_begin();
	uint32_t p = pc, last = pc;
	int n = 0, last_interp = 0;
	i386_dinsn d, dl;
	memset(&dl, 0, sizeof(dl));
	for (; n < IJ_MAX_INSNS && s_ij_n < IJ_BUF_LIMIT && s_nstub < IJ_MAXS - 8; n++) {
		uint8_t bytes[16];
		if ((p >> 16) != (pc >> 16))   // (the page's end)
			break;
		const uint32_t o = p & 0xffff, left = 0x10000 - o;
		memset(bytes, 0, sizeof(bytes));
		memcpy(bytes, page + o, left < 16 ? left : 16);
		const int len = i386_jit_decode(bytes, 1, &d);
		if (!len || (uint32_t)len > left)   // (unknown, or over the page's end: the interpreter)
			break;
		const uint32_t next = p + (uint32_t)len;
		s_pre_next = p;   // (the exit before it: the last one run is the one before, or none)
		s_pre_pc0 = n ? last : p;
		s_pre_pack = n && !last_interp ? ij_pack(&dl, 0) : 0;
		s_pre_made = 0;
		const int dop = (d.two ? 256 : 0) + d.op;
		const int nat = (s_ij_dbg & 1) || dop < s_ij_oplo || dop > s_ij_ophi ? 0 : ij_native(&d, p, next);
		last = p;
		dl = d;
		p = next;
		if (nat == 2) {
			n++;
			goto done;
		}
		if (nat == 1) {
			last_interp = 0;
			continue;
		}
		if (!(s_ij_dbg & 2) && ij_direct_ok(&d)) {   // through its handler (ij_direct)
			const uint32_t start = next - d.has_modrm - d.has_sib - d.disp_len - d.imm_len - d.imm2_len;
			e_imm(0, start);
			e_imm(1, ij_pack(&d, 0) & 0x1fffu | (d.seg ? (uint32_t)(d.seg - 1) << 13 : 0) | (uint32_t)d.two << 16);
			e_imm(2, next);
			e_imm(3, last);
			e_bl(SR_DIRECT);
			E(0xe3500001u);   // cmp r0, #1
			e_bto(0x8, e_newstub(0, last, 0, 1));   // bhi: left
			const uint32_t k = e_newstub(next, last, ij_pack(&d, 0), 0);
			e_bto(0x0, k);    // beq: an interrupt pending
			E(0xe3550000u);   // cmp r5, #0
			e_bto(0xd, k);    // ble: the slice's end
			last_interp = 0;
			continue;
		}
		// through the interpreter: on in the block when the flow went to next in the blocks' state
		int end = 0;   // an unconditional transfer: the block ends after it
		if (!d.two && (d.op == 0xc3 || d.op == 0xc2 || d.op == 0xe8 || d.op == 0xe9 || d.op == 0xeb || d.op == 0xcb ||
		               d.op == 0xca || d.op == 0xcf || d.op == 0xcc || d.op == 0xcd || d.op == 0xce || d.op == 0xf4 ||
		               d.op == 0x9a || d.op == 0xea || (d.op == 0xff && ((d.modrm >> 3) & 7) >= 2 &&
		                                                 ((d.modrm >> 3) & 7) <= 5)))
			end = 1;
		e_imm(0, last);
		e_imm(1, next);
		e_bl(SR_INTERP);
		const uint32_t k = e_newstub(0, last, 0, 1);
		E(0xe3500000u);   // cmp r0, #0
		e_bto(0x0, k);    // beq: left
		E(0xe3550000u);   // cmp r5, #0
		e_bto(0xd, k);    // ble: the slice's end
		last_interp = 1;
		if (end) {
			n++;
			break;
		}
	}
	if (!n)
		return IJ_NONE;
	// (the end: on to p)
	if (last_interp)
		e_bto(0xe, e_newstub(0, last, 0, 1));   // (I.pc already there)
	else
		e_bto(0xe, e_newstub(p, last, ij_pack(&dl, 1), 2));
done:
	if (s_ij_full)
		return NULL;
	s_ij_ni += (unsigned)n;
	e_finish();
	return ij_install();
}

// the block for pc (made the first time when the flow came there by a transfer); NULL: interpret
static inline ij_fn ij_find(uint32_t pc)
{
	const unsigned i = ((pc >> 2) ^ (pc >> 15)) & ((1u << IJ_TAB_BITS) - 1);
	for (int w = 0; w < 4; w++)
		if (s_ij_tab[i][w].pc == pc && s_ij_tab[i][w].fn)
			return s_ij_tab[i][w].fn == IJ_NONE ? NULL : s_ij_tab[i][w].fn;
	if (!s_ij_build || !ij_code_init())
		return NULL;
	if (s_ij_maps_old)   // (no block since the maps changed: s_ij_r4k / w4k made before the first one)
		ij_maps();
	ij_fn f = ij_build(pc);
	if (!f)
		return NULL;
	const unsigned w = s_ij_next[i]++ & 3;
	s_ij_tab[i][w].pc = pc;
	s_ij_tab[i][w].fn = f;
	return f == IJ_NONE ? NULL : f;
}

// the slot of the last exit made a jump into f (after its e_begin: the registers are the same in every block)
static void ij_chain(ij_fn f)
{
	uint32_t *slot = s_ij_slot;
	s_ij_slot = NULL;
	if (*slot != 0xe1a00000u)   // (patched already: left by the tests before it)
		return;
	const uint32_t *t = (const uint32_t *)(void *)f + IJ_ENTRY;
	rbx_jit_write_begin();
	*slot = 0xea000000u | ((uint32_t)(t - slot - 2) & 0x00ffffffu);
	rbx_jit_write_end(slot, 4);
}

static void i386_jit_flush(void) { ij_flush(); }
static int ij_enabled(void)
{
	if (s_ij_on < 0) {
		s_ij_on = !(getenv("RBX_I386_NOJIT") && atoi(getenv("RBX_I386_NOJIT")));
		s_ij_stat = getenv("RBX_I386_JITSTAT") && atoi(getenv("RBX_I386_JITSTAT"));
		if (getenv("RBX_I386_JITTRACE")) s_ij_tf = fopen(getenv("RBX_I386_JITTRACE"), "w");
		if (getenv("RBX_I386_JITTRACEN")) s_ij_tdet = (uint32_t)atoi(getenv("RBX_I386_JITTRACEN"));
		if (getenv("RBX_I386_JITDBG")) sscanf(getenv("RBX_I386_JITDBG"), "%d,%x,%x", &s_ij_dbg, &s_ij_oplo, &s_ij_ophi);
	}
	return s_ij_on;
}
static uint32_t s_ij_lreg[3];   // (the run loop's steps: below the ROM, not in the blocks' state, else)
static int s_ij_ok_dbg(void) { return ij_mode_ok(); }
static void i386_jit_report(void)
{
	if (!s_ij_stat)
		return;
	rbx_log("i386 jit: run loop steps: RAM %u, other state %u, ROM %u", s_ij_lreg[0], s_ij_lreg[1], s_ij_lreg[2]);
	rbx_log("i386 jit: %u ms in i386_run, %u blocks, %u instructions made; %u blocks run, %u steps, %u steps in blocks",
	        (unsigned)(s_ij_us / 1000), s_ij_nb, s_ij_ni, s_ij_runs, s_ij_steps, s_ij_isteps);
	for (int i = 0; i < 257; i++)
		if (s_ij_slow[i] > 10000)
			rbx_log("i386 jit: slow accesses %05x: %u", i << 12, s_ij_slow[i]);
	for (int k = 0; k < 40; k++) {   // (the most interpreted opcodes)
		int best = -1;
		for (int i = 0; i < 512; i++)
			if (s_ij_cnt[i] && (best < 0 || s_ij_cnt[i] > s_ij_cnt[best]))
				best = i;
		if (best < 0)
			break;
		rbx_log("i386 jit: interpreted %s%02x: %u", best >= 256 ? "0f " : "", best & 255, s_ij_cnt[best]);
		s_ij_cnt[best] = 0;
	}
}

#else
static void i386_jit_flush(void) {}
static void i386_jit_report(void) {}
#endif
