// license:BSD-3-Clause
// copyright-holders:Ville Linde, Barry Rodewald, Carl, Philip Bennett
/*  Intel 386 (DX) CPU, MAME's i386_device (the 386 only) as plain C: see i386.h.
    Ported from MAME 0.289 src/devices/cpu/i386/i386.cpp, i386.h (BSD-3-Clause, copyright-holders: Ville Linde,
    Barry Rodewald, Carl, Philip Bennett); the opcode handlers, the segment / task / interrupt code, the opcode and
    cycle tables are MAME's i386ops.hxx, i386op16.hxx, i386op32.hxx, i386segs.hxx, i386ops.h, cycles.h converted
    with their members as fields of one state struct (I.x for m_x) and their throws as longjmps (*.inc).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../cheat_cpu.h"
#include "i386.h"
#include "../../rbx.h"

#include <setjmp.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

typedef uint32_t offs_t;
typedef uint32_t vtlb_entry;

// MAME's emucore / vtlb helpers the sources use
#define WORD_ALIGNED(a) (((a) & 1) == 0)
#define DWORD_ALIGNED(a) (((a) & 3) == 0)
#define I386_MAX(a, b) ((a) > (b) ? (a) : (b))
#define I386_MIN(a, b) ((a) < (b) ? (a) : (b))
#define logerror(...) ((void)0)
#define LOGMASKED(...) ((void)0)
#define fatalerror(...) (rbx_log(__VA_ARGS__), abort())
#define vtlb_flush_dynamic() ((void)0)
#define debugger_privilege_hook() ((void)0)
#define debugger_exception_hook(irq) ((void)(irq))
#define BIT(x, n) (((x) >> (n)) & 1)
static inline uint32_t rotl_32(uint32_t v, int n) { n &= 31; return n ? (v << n) | (v >> (32 - n)) : v; }
static inline uint32_t rotr_32(uint32_t v, int n) { n &= 31; return n ? (v >> n) | (v << (32 - n)) : v; }
enum { TR_READ = 0, TR_WRITE = 1, TR_FETCH = 2, TR_USER = 4 };   // (emumem.h TRANSLATE_*)
enum {   // (vtlb.h)
	READ_ALLOWED = 0x01, WRITE_ALLOWED = 0x02, FETCH_ALLOWED = 0x04, FLAG_VALID = 0x08,
	USER_READ_ALLOWED = 0x10, USER_WRITE_ALLOWED = 0x20, USER_FETCH_ALLOWED = 0x40
};

#define X86_NUM_CPUS 4

// i386.h CR0_BITS, CR4_BITS
#define CR0_PG 0x80000000u
#define CR0_CD (1u << 30)
#define CR0_NW (1u << 29)
#define CR0_AM (1u << 18)
#define CR0_WP (1u << 16)
#define CR0_NE (1u << 5)
#define CR0_ET (1u << 4)
#define CR0_TS (1u << 3)
#define CR0_EM (1u << 2)
#define CR0_MP (1u << 1)
#define CR0_PE (1u << 0)
#define CR4_PSE (1u << 4)
#define CR4_PVI (1u << 1)
#define CR4_VME (1u << 0)

// ---- the state (i386.h's members) ----
typedef union {
	uint32_t d[8];
	uint16_t w[16];
	uint8_t b[32];
} I386_GPR;

typedef struct {
	uint16_t selector;
	uint16_t flags;
	uint32_t base;
	uint32_t limit;
	int d;      // Operand size
	bool valid;
} I386_SREG;

typedef struct {
	uint32_t base;
	uint16_t limit;
} I386_SYS_TABLE;

typedef struct {
	uint16_t segment;
	uint16_t flags;
	uint32_t base;
	uint32_t limit;
} I386_SEG_DESC;

typedef struct {
	uint16_t segment;
	uint16_t selector;
	uint32_t offset;
	uint8_t ar;  // access rights
	uint8_t dpl;
	uint8_t dword_count;
	uint8_t present;
} I386_CALL_GATE;

typedef void (*i386_op_func)(void);
typedef struct {
	uint8_t opcode;
	uint32_t flags;
	i386_op_func handler16;
	i386_op_func handler32;
	bool lockable;
} X86_OPCODE;

typedef struct {
	// ---- state (saved) ----
	I386_GPR reg;
	I386_SREG sreg[6];
	uint32_t eip;
	uint32_t pc;
	uint32_t prev_eip;
	uint32_t eflags;
	uint32_t eflags_mask;
	uint8_t CF, DF, SF, OF, ZF, PF, AF, IF, TF, IOP1, IOP2, NT, RF, VM, AC, VIF, VIP, ID;
	uint8_t CPL;  // current privilege level
	bool auto_clear_RF;
	uint8_t performed_intersegment_jump;
	uint8_t delayed_interrupt_enable;
	uint32_t cr[5];       // Control registers
	uint32_t dr[8];       // Debug registers
	uint32_t tr[8];       // Test registers
	I386_SYS_TABLE gdtr;    // Global Descriptor Table Register
	I386_SYS_TABLE idtr;    // Interrupt Descriptor Table Register
	I386_SEG_DESC task;     // Task register
	I386_SEG_DESC ldtr;     // Local Descriptor Table Register
	uint8_t ext;  // external interrupt
	int halted;
	int operand_size;
	int xmm_operand_size;
	int address_size;
	int operand_prefix;
	int address_prefix;
	int segment_prefix;
	int segment_override;
	int cycles;
	int base_cycles;
	uint8_t opcode;
	uint8_t irq_state;
	uint8_t nmi_state;
	bool smm, smi, lock;
	offs_t a20_mask;
	uint32_t cpu_version;
	uint32_t cpuid_id0;
	uint64_t tsc;
	int stolen;          // (cycles given up by i386_abort in this run)
	int spinning;        // (spin_until_interrupt: suspended until an input line is asserted)
	uint8_t state_end;
	// ---- not saved ----
	uint8_t opcode_bytes[16];
	uint32_t opcode_pc;
	int opcode_bytes_length;
	offs_t opcode_addrs[16];
	uint32_t opcode_addrs_index;
	const uint8_t *cycle_table_pm;
	const uint8_t *cycle_table_rm;
	i386_op_func opcode_table1_16[256];
	i386_op_func opcode_table1_32[256];
	i386_op_func opcode_table2_16[256];
	i386_op_func opcode_table2_32[256];
	i386_op_func opcode_table366_16[256];
	i386_op_func opcode_table366_32[256];
	i386_op_func opcode_table3f2_16[256];
	i386_op_func opcode_table3f2_32[256];
	i386_op_func opcode_table3f3_16[256];
	i386_op_func opcode_table3f3_32[256];
	bool lock_table[2][256];
} i386_t;

static i386_t I;
static i386_bus s_bus;
static uint8_t **s_rmap, **s_wmap;   // 64 KB pages: the page's host memory, NULL: the handlers

// the polling loops' skipping (i386_idle_enable; not state: it never changes a result)
static int s_idle_on, s_idle_fail;
static int (*s_idle_pure)(uint32_t addr, int write);
static uint32_t s_idle_tgt;
static int s_idle_cnt;
static uint32_t s_idle_bad[64];
static uint8_t s_idle_badn[64];
static uint16_t s_idle_wpages[64];   // (the pages i386_map made writable)
static int s_idle_nwp, s_idle_wover;
static uint8_t *s_idle_wsaved[64];
static uint32_t s_idle_wa[64];       // (the bytes written while probing: addresses and first values)
static uint8_t s_idle_wv[64];
static int s_idle_wn;
static i386_bus s_idle_bus;          // (the board's handlers while probing)
static void idle_wpage_add(uint32_t page);
static void idle_traps(int on);
static void i386_jit_flush(void);   // (i386_jit.inc: the translated code thrown away)
static void i386_jit_report(void);  // (i386_jit.inc: RBX_I386_JITSTAT's counts)
static int s_ij_ok;                 // (i386_jit.inc: the blocks' state checked, nothing changed it since)

// ---- MAME's throws: the innermost handler and the exception (fault | error << 32) ----
static jmp_buf *s_jmp;
static uint64_t s_exc;
#if defined(__GNUC__)
__attribute__((noreturn))
#endif
static void i386_throw(uint64_t e)
{
	s_exc = e;
	longjmp(*s_jmp, 1);
}
#define I386_TRY_BEGIN jmp_buf jb_; jmp_buf *const prev_ = s_jmp; s_jmp = &jb_;

#include "i386priv.inc"

#define FAULT_THROW(fault, error) { i386_throw((uint64_t)(fault) | (uint64_t)(error) << 32); }
#define PF_THROW(error) { I.cr[2] = address; FAULT_THROW(FAULT_PF, error); }

static int i386_parity_table[256];
static MODRM_TABLE i386_MODRM_table[256];

typedef enum X86_CYCLES X86_CYCLES;
typedef struct X86_CYCLE_TABLE X86_CYCLE_TABLE;
#include "i386cycles.inc"
static uint8_t s_cycle_table_rm[CYCLES_NUM_OPCODES], s_cycle_table_pm[CYCLES_NUM_OPCODES];

// ---- the program and I/O spaces ----
// (ReBOX) 64 KB pages of the handlers with 4 KB parts of plain memory in them (i386_map_split): read and written here
// as the handlers would (their memory without side effects), not while a polling loop is probed (its accesses are
// watched through the handlers)
#define I386_NSPLIT 4
static struct { uint32_t page; uint8_t *r[16], *w[16]; } s_split[I386_NSPLIT];
static int s_nsplit;
static int s_idle_probing;
static inline uint8_t *split_ptr(offs_t a, int size, int write)
{
	if (!s_nsplit || s_idle_probing || (a & 0xfff) + (uint32_t)size > 0x1000)
		return NULL;
	for (int i = 0; i < s_nsplit; i++)
		if (s_split[i].page == a >> 16) {
			uint8_t *const q = write ? s_split[i].w[(a >> 12) & 15] : s_split[i].r[(a >> 12) & 15];
			return q ? q + (a & 0xfff) : NULL;
		}
	return NULL;
}
static inline uint32_t prog_r32_mask(offs_t a, uint32_t mask)
{
	const uint8_t *p = s_rmap[a >> 16];
	if (p) {
		uint32_t v;
		memcpy(&v, p + (a & 0xffff), 4);
		return v;
	}
	const uint8_t *q = split_ptr(a, 4, 0);
	if (q) {
		uint32_t v;
		memcpy(&v, q, 4);
		return v;
	}
	return s_bus.read32(a & ~3u, mask);
}
static inline uint32_t prog_r32(offs_t a) { return prog_r32_mask(a, 0xffffffffu); }
static inline uint8_t prog_r8(offs_t a)
{
	const uint8_t *p = s_rmap[a >> 16];
	if (p)
		return p[a & 0xffff];
	const uint8_t *q = split_ptr(a, 1, 0);
	if (q)
		return *q;
	const int sh = (int)(a & 3) * 8;
	return (uint8_t)(s_bus.read32(a & ~3u, 0xffu << sh) >> sh);
}
static inline uint16_t prog_r16(offs_t a)
{
	const uint8_t *p = s_rmap[a >> 16];
	if (p) {
		uint16_t v;
		memcpy(&v, p + (a & 0xffff), 2);
		return v;
	}
	const uint8_t *q = split_ptr(a, 2, 0);
	if (q) {
		uint16_t v;
		memcpy(&v, q, 2);
		return v;
	}
	const int sh = (int)(a & 2) * 8;
	return (uint16_t)(s_bus.read32(a & ~3u, 0xffffu << sh) >> sh);
}
static inline void prog_w32_mask(offs_t a, uint32_t v, uint32_t mask)
{
	uint8_t *p = s_wmap[a >> 16];
	if (p)
		p += a & 0xffff;
	else
		p = split_ptr(a, 4, 1);
	if (p) {
		for (int i = 0; i < 4; i++)
			if ((mask >> (i * 8)) & 0xff)
				p[i] = (uint8_t)(v >> (i * 8));
		return;
	}
	s_bus.write32(a & ~3u, v, mask);
}
static inline void prog_w32(offs_t a, uint32_t v)
{
	uint8_t *p = s_wmap[a >> 16];
	if (p) {
		memcpy(p + (a & 0xffff), &v, 4);
		return;
	}
	uint8_t *q = split_ptr(a, 4, 1);
	if (q) {
		memcpy(q, &v, 4);
		return;
	}
	s_bus.write32(a & ~3u, v, 0xffffffffu);
}
static inline void prog_w8(offs_t a, uint8_t v)
{
	uint8_t *p = s_wmap[a >> 16];
	if (p) {
		p[a & 0xffff] = v;
		return;
	}
	uint8_t *q = split_ptr(a, 1, 1);
	if (q) {
		*q = v;
		return;
	}
	const int sh = (int)(a & 3) * 8;
	s_bus.write32(a & ~3u, (uint32_t)v << sh, 0xffu << sh);
}
static inline void prog_w16(offs_t a, uint16_t v)
{
	uint8_t *p = s_wmap[a >> 16];
	if (p) {
		memcpy(p + (a & 0xffff), &v, 2);
		return;
	}
	uint8_t *q = split_ptr(a, 2, 1);
	if (q) {
		memcpy(q, &v, 2);
		return;
	}
	const int sh = (int)(a & 2) * 8;
	s_bus.write32(a & ~3u, (uint32_t)v << sh, 0xffffu << sh);
}
static uint32_t io_r32_mask(offs_t a, uint32_t mask) { return s_bus.io_read32 ? s_bus.io_read32(a & ~3u, mask) : 0; }
static uint32_t io_r32(offs_t a) { return io_r32_mask(a, 0xffffffffu); }
static uint8_t io_r8(offs_t a) { const int sh = (int)(a & 3) * 8; return (uint8_t)(io_r32_mask(a, 0xffu << sh) >> sh); }
static uint16_t io_r16(offs_t a) { const int sh = (int)(a & 2) * 8; return (uint16_t)(io_r32_mask(a, 0xffffu << sh) >> sh); }
static void io_w32_mask(offs_t a, uint32_t v, uint32_t mask) { if (s_bus.io_write32) s_bus.io_write32(a & ~3u, v, mask); }
static void io_w32(offs_t a, uint32_t v) { io_w32_mask(a, v, 0xffffffffu); }
static void io_w8(offs_t a, uint8_t v) { const int sh = (int)(a & 3) * 8; io_w32_mask(a, (uint32_t)v << sh, 0xffu << sh); }
static void io_w16(offs_t a, uint16_t v) { const int sh = (int)(a & 2) * 8; io_w32_mask(a, (uint32_t)v << sh, 0xffffu << sh); }

// ---- declarations ----
// (MAME's default arguments: i386_translate(segment, ip, rwn, size = 1), i386_limit_check(seg, offset, size = 1))
#define I386_ARG4(a, b, c, d, ...) d
#define I386_ARG5(a, b, c, d, e, ...) e
#define i386_translate(...) I386_ARG5(__VA_ARGS__, i386_translate_sz, i386_translate_1, 0, 0)(__VA_ARGS__)
#define i386_translate_1(s, ip, rwn) i386_translate_sz(s, ip, rwn, 1)
#define i386_limit_check(...) I386_ARG4(__VA_ARGS__, i386_limit_check_sz, i386_limit_check_1, 0)(__VA_ARGS__)
#define i386_limit_check_1(s, off) i386_limit_check_sz(s, off, 1)
static uint32_t i386_translate_sz(int segment, uint32_t ip, int rwn, int size);
static bool i386_translate_address(int intention, bool debug, offs_t *address, vtlb_entry *entry);
static inline bool translate_address(int pl, int type, offs_t *address, uint32_t *error);
static void i386_decode_three_byte66(void);
static void i386_decode_three_bytef2(void);
static void i386_decode_three_bytef3(void);
static uint8_t READ8PL(uint32_t ea, uint8_t privilege);
static uint16_t READ16PL(uint32_t ea, uint8_t privilege);
static uint32_t READ32PL(uint32_t ea, uint8_t privilege);
static uint64_t READ64PL(uint32_t ea, uint8_t privilege);
static void WRITE8PL(uint32_t ea, uint8_t privilege, uint8_t value);
static void WRITE16PL(uint32_t ea, uint8_t privilege, uint16_t value);
static void WRITE32PL(uint32_t ea, uint8_t privilege, uint32_t value);
static void WRITE64PL(uint32_t ea, uint8_t privilege, uint64_t value);
static inline uint8_t READ8(uint32_t ea) { return READ8PL(ea, I.CPL); }
static inline uint16_t READ16(uint32_t ea) { return READ16PL(ea, I.CPL); }
static inline uint32_t READ32(uint32_t ea) { return READ32PL(ea, I.CPL); }
static inline uint64_t READ64(uint32_t ea) { return READ64PL(ea, I.CPL); }
static inline void WRITE8(uint32_t ea, uint8_t value) { WRITE8PL(ea, I.CPL, value); }
static inline void WRITE16(uint32_t ea, uint16_t value) { WRITE16PL(ea, I.CPL, value); }
static inline void WRITE32(uint32_t ea, uint32_t value) { WRITE32PL(ea, I.CPL, value); }
static inline void WRITE64(uint32_t ea, uint64_t value) { WRITE64PL(ea, I.CPL, value); }
static void i386_invalid(void);
static void i386_trap(int irq, int irq_gate);
static void i386_trap_with_error(int irq, int irq_gate, int trap_level, uint32_t error);
static void i386_triple_fault(void);
#define I386_IRQ_ACK() (s_bus.irq_ack ? s_bus.irq_ack() : 0)
#include "i386proto.inc"

// the 386 has no SMI (i386_check_irq_line's SMM entry: never taken)
static void enter_smm(void) {}

// the debug registers' data breakpoints are not emulated (MAME's memory taps)
static void dri_changed(void) {}
static void dr7_changed(uint32_t old_val, uint32_t new_val) { (void)old_val; (void)new_val; }

// translate_address without the TLB: MAME's TLB miss path (the page walk) for every access (paging off: inline)
static bool translate_address_pg(int pl, int type, offs_t *address, uint32_t *error);
static inline bool translate_address(int pl, int type, offs_t *address, uint32_t *error)
{
	if (!(I.cr[0] & CR0_PG)) // Some (very few) old OS's won't work with this
		return true;
	return translate_address_pg(pl, type, address, error);
}
static bool translate_address_pg(int pl, int type, offs_t *address, uint32_t *error)
{
	vtlb_entry entry = 0;
	if (type == TR_FETCH)
		type = TR_READ;
	if (pl == 3)
		type |= TR_USER;
	if (!i386_translate_address(type, false, address, &entry)) {
		*error = ((type & TR_WRITE) ? 2 : 0) | ((I.CPL == 3) ? 4 : 0);
		if (entry)
			*error |= 1;
		return false;
	}
	return true;
}

// (i386.cpp: the ALU helpers without a carry in)
#define SUB8(dst, src) SBB8(dst, src, 0)
#define SUB16(dst, src) SBB16(dst, src, 0)
#define SUB32(dst, src) SBB32(dst, src, 0)
#define ADD8(dst, src) ADC8(dst, src, 0)
#define ADD16(dst, src) ADC16(dst, src, 0)
#define ADD32(dst, src) ADC32(dst, src, 0)

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"   // (POP8: MAME's, no 386 opcode uses it)
#endif
#include "i386core.inc"
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

/* Three-byte opcode prefix 66 0f xx */
static void i386_decode_three_byte66(void)
{
	I.opcode = FETCH();
	if( I.operand_size )
		I.opcode_table366_32[I.opcode]();
	else
		I.opcode_table366_16[I.opcode]();
}

/* Three-byte opcode prefix f2 0f xx */
static void i386_decode_three_bytef2(void)
{
	I.opcode = FETCH();
	if( I.operand_size )
		I.opcode_table3f2_32[I.opcode]();
	else
		I.opcode_table3f2_16[I.opcode]();
}

/* Three-byte opcode prefix f3 0f */
static void i386_decode_three_bytef3(void)
{
	I.opcode = FETCH();
	if( I.operand_size )
		I.opcode_table3f3_32[I.opcode]();
	else
		I.opcode_table3f3_16[I.opcode]();
}

#include "i386ops.inc"
#include "i386op16.inc"
#include "i386op32.inc"
#include "i386segs.inc"
#include "i386optab.inc"
// ---- i386_device: start, reset, run ----
static void build_cycle_table(void)
{
	for (size_t i = 0; i < sizeof(x86_cycle_table) / sizeof(x86_cycle_table[0]); i++) {
		const int opcode = x86_cycle_table[i].op;
		s_cycle_table_rm[opcode] = x86_cycle_table[i].cpu_cycles[CPU_CYCLES_I386][0];
		s_cycle_table_pm[opcode] = x86_cycle_table[i].cpu_cycles[CPU_CYCLES_I386][1];
	}
}

static void build_opcode_table(uint32_t features)
{
	for (int i = 0; i < 256; i++) {
		I.opcode_table1_16[i] = i386_invalid;
		I.opcode_table1_32[i] = i386_invalid;
		I.opcode_table2_16[i] = i386_invalid;
		I.opcode_table2_32[i] = i386_invalid;
		I.opcode_table366_16[i] = i386_invalid;
		I.opcode_table366_32[i] = i386_invalid;
		I.opcode_table3f2_16[i] = i386_invalid;
		I.opcode_table3f2_32[i] = i386_invalid;
		I.opcode_table3f3_16[i] = i386_invalid;
		I.opcode_table3f3_32[i] = i386_invalid;
		I.lock_table[0][i] = false;
		I.lock_table[1][i] = false;
	}
	for (size_t i = 0; i < sizeof(s_x86_opcode_table) / sizeof(X86_OPCODE); i++) {
		const X86_OPCODE *op = &s_x86_opcode_table[i];
		if (!(op->flags & features))
			continue;
		if (op->flags & OP_2BYTE) {
			I.opcode_table2_32[op->opcode] = op->handler32;
			I.opcode_table2_16[op->opcode] = op->handler16;
			I.opcode_table366_32[op->opcode] = op->handler32;
			I.opcode_table366_16[op->opcode] = op->handler16;
			I.lock_table[1][op->opcode] = op->lockable;
		} else if (op->flags & OP_3BYTE66) {
			I.opcode_table366_32[op->opcode] = op->handler32;
			I.opcode_table366_16[op->opcode] = op->handler16;
		} else if (op->flags & OP_3BYTEF2) {
			I.opcode_table3f2_32[op->opcode] = op->handler32;
			I.opcode_table3f2_16[op->opcode] = op->handler16;
		} else if (op->flags & OP_3BYTEF3) {
			I.opcode_table3f3_32[op->opcode] = op->handler32;
			I.opcode_table3f3_16[op->opcode] = op->handler16;
		} else if (!(op->flags & (OP_3BYTE38 | OP_3BYTE3A | OP_4BYTE3866 | OP_4BYTE3A66 | OP_4BYTE38F2 | OP_4BYTE3AF2 |
		                          OP_4BYTE38F3))) {   // (those tables: none of the 386's)
			I.opcode_table1_32[op->opcode] = op->handler32;
			I.opcode_table1_16[op->opcode] = op->handler16;
			I.lock_table[0][op->opcode] = op->lockable;
		}
	}
}

// (the cheats' access to this CPU's program space: cheat_cpu.h)
static uint8_t cheat_rd(void *x, uint32_t a) { (void)x; return prog_r8(a); }
static void cheat_wr(void *x, uint32_t a, uint8_t v) { (void)x; prog_w8(a, v); }

int i386_init(const i386_bus *bus)
{
	rbx_cheat_offer("i386", cheat_rd, cheat_wr, NULL, 0, 1);
	static const int regs8[8] = {AL,CL,DL,BL,AH,CH,DH,BH};
	static const int regs16[8] = {AX,CX,DX,BX,SP,BP,SI,DI};
	static const int regs32[8] = {EAX,ECX,EDX,EBX,ESP,EBP,ESI,EDI};
	i386_exit();
	s_bus = *bus;
	s_idle_on = s_idle_probing = 0;
	s_idle_nwp = s_idle_wover = 0;
	s_nsplit = 0;
	s_rmap = (uint8_t **)calloc(65536, sizeof(uint8_t *));
	s_wmap = (uint8_t **)calloc(65536, sizeof(uint8_t *));
	if (!s_rmap || !s_wmap) {
		i386_exit();
		return -1;
	}
	memset(&I, 0, sizeof(I));
	build_cycle_table();
	for (int i = 0; i < 256; i++) {
		int c = 0;
		for (int j = 0; j < 8; j++)
			if (i & (1 << j))
				c++;
		i386_parity_table[i] = ~(c & 0x1) & 0x1;
	}
	for (int i = 0; i < 256; i++) {
		i386_MODRM_table[i].reg.b = regs8[(i >> 3) & 0x7];
		i386_MODRM_table[i].reg.w = regs16[(i >> 3) & 0x7];
		i386_MODRM_table[i].reg.d = regs32[(i >> 3) & 0x7];
		i386_MODRM_table[i].rm.b = regs8[i & 0x7];
		i386_MODRM_table[i].rm.w = regs16[i & 0x7];
		i386_MODRM_table[i].rm.d = regs32[i & 0x7];
	}
	build_opcode_table(OP_I386);
	I.cycle_table_rm = s_cycle_table_rm;
	I.cycle_table_pm = s_cycle_table_pm;
	i386_jit_flush();
	return 0;
}

void i386_exit(void)
{
	i386_jit_report();
	free(s_rmap);
	free(s_wmap);
	s_rmap = s_wmap = NULL;
	s_nsplit = 0;
}

void i386_map_split(uint32_t start, uint32_t end, uint8_t *mem, int writable)
{
	i386_jit_flush();
	for (uint32_t a = start & ~0xfffu; a - start < end - start; a += 0x1000) {
		int k = 0;
		while (k < s_nsplit && s_split[k].page != a >> 16)
			k++;
		if (k == s_nsplit) {
			if (s_nsplit >= I386_NSPLIT)
				return;
			memset(&s_split[k], 0, sizeof(s_split[k]));
			s_split[k].page = a >> 16;
			s_nsplit++;
		}
		s_split[k].r[(a >> 12) & 15] = mem ? mem + (a - start) : NULL;
		s_split[k].w[(a >> 12) & 15] = mem && writable ? mem + (a - start) : NULL;
	}
}

void i386_map(uint32_t start, uint32_t end, uint8_t *mem, int writable)
{
	i386_jit_flush();
	for (uint32_t a = start; a - start < end - start; a += 0x10000) {   // (mem NULL: back to the handlers)
		s_rmap[a >> 16] = mem ? mem + (a - start) : NULL;
		s_wmap[a >> 16] = mem && writable ? mem + (a - start) : NULL;
		if (mem && writable)
			idle_wpage_add(a >> 16);
	}
}

// zero_state, device_reset
void i386_reset(void)
{
	i386_jit_flush();
	s_ij_ok = 0;
	memset(&I, 0, offsetof(i386_t, state_end));

	I.sreg[CS].selector = 0xf000;
	I.sreg[CS].base     = 0xffff0000;
	I.sreg[CS].limit    = 0xffff;
	I.sreg[CS].flags    = 0x93;
	I.sreg[CS].valid    = true;

	I.sreg[DS].base = I.sreg[ES].base = I.sreg[FS].base = I.sreg[GS].base = I.sreg[SS].base = 0x00000000;
	I.sreg[DS].limit = I.sreg[ES].limit = I.sreg[FS].limit = I.sreg[GS].limit = I.sreg[SS].limit = 0xffff;
	I.sreg[DS].flags = I.sreg[ES].flags = I.sreg[FS].flags = I.sreg[GS].flags = I.sreg[SS].flags = 0x0093;
	I.sreg[DS].valid = I.sreg[ES].valid = I.sreg[FS].valid = I.sreg[GS].valid = I.sreg[SS].valid = true;

	I.idtr.base = 0;
	I.idtr.limit = 0x3ff;

	I.a20_mask = ~0u;

	I.cr[0] = 0x7fffffe0; // reserved bits set to 1
	I.eflags = 0;
	I.eflags_mask = 0x00037fd7;
	I.eip = 0xfff0;

	// [11:8] Family
	// [ 7:4] Model
	// [ 3:0] Stepping ID
	// Family 3 (386), Model 0 (DX), Stepping 8 (D1)
	REG32(EAX) = 0;
	REG32(EDX) = (3 << 8) | (0 << 4) | (8);
	I.cpu_version = REG32(EDX);

	I.CPL = 0;

	I.auto_clear_RF = true;

	CHANGE_PC(I.eip);
}

// a triple fault: MAME pulses the reset line
static void i386_triple_fault(void)
{
	i386_reset();
	I.cycles = 0;
}

// execute_set_input (the INT line; the NMI)
void i386_set_irq(int state)
{
	if (state) {
		I.halted = 0;
		I.spinning = 0;
	}
	s_ij_ok = 0;
	I.irq_state = (uint8_t)(state ? 1 : 0);
}

void i386_set_nmi(int state)
{
	s_ij_ok = 0;
	if (state && !I.nmi_state) {   // (edge triggered)
		I.halted = 0;
		I.spinning = 0;
		jmp_buf jb, *prev = s_jmp;
		s_jmp = &jb;
		if (!setjmp(jb))
			i386_trap(2, 1);
		s_jmp = prev;
	}
	I.nmi_state = (uint8_t)(state ? 1 : 0);
}

void i386_spin_until_interrupt(void)
{
	I.spinning = 1;
	I.cycles = 0;   // (the rest of the slice is eaten, not given back)
}

void i386_abort(void)
{
	I.stolen += I.cycles;
	I.cycles = 0;
}

int i386_done(void) { return I.base_cycles - I.cycles - I.stolen; }
uint32_t i386_pc(void) { return I.pc; }

// one instruction of execute_run's loop (inlined in the run's loop and the polling loops' probe)
#if defined(__GNUC__)
__attribute__((always_inline))
#endif
static inline void i386_step(void)
{
	if ((!I.smm && I.smi) || (I.irq_state && I.IF))   // (i386_check_irq_line's conditions first: no call most steps)
		i386_check_irq_line();

	// (the instruction breakpoints of DR0-DR3)
	if ((I.dr[7] & 0xff) != 0)
	for (int i = 0; i < 4; i++) {
		bool dri_enabled = (I.dr[7] & (1 << ((i << 1) + 1))) || (I.dr[7] & (1 << (i << 1)));
		if (dri_enabled && !I.RF) {
			int breakpoint_type = (I.dr[7] >> (i << 2)) & 3;
			if (breakpoint_type == 0) {
				uint32_t phys_addr = 0;
				uint32_t error;
				if (I.cr[0] & CR0_PG) {
					offs_t addr = I.dr[i];
					phys_addr = translate_address(I.CPL, TR_FETCH, &addr, &error);
					I.dr[i] = (uint32_t)addr;
				} else
					phys_addr = I.dr[i];
				if (I.pc == phys_addr) {
					I.dr[6] |= 1 << i;
					i386_trap(1, 0);
					break;
				}
			}
		}
	}

	I.operand_size = I.sreg[CS].d;
	I.xmm_operand_size = 0;
	I.address_size = I.sreg[CS].d;
	I.operand_prefix = 0;
	I.address_prefix = 0;

	I.ext = 1;
	int old_tf = I.TF;

	I.segment_prefix = 0;
	I.prev_eip = I.eip;

	if (I.delayed_interrupt_enable != 0) {
		I.IF = 1;
		I.delayed_interrupt_enable = 0;
	}
	i386_decode_opcode();
	if (I.TF && old_tf) {
		I.prev_eip = I.eip;
		I.ext = 1;
		I.dr[6] |= (1 << 14); //Set BS bit of DR6.
		i386_trap(1, 0);
	}
	if (I.lock && (I.opcode != 0xf0))
		I.lock = false;
}

// execute_run's loop body
#if defined(__GNUC__)
__attribute__((always_inline))
#endif
static inline void i386_step_rf(void)
{
	i386_step();
	if (I.RF && I.auto_clear_RF) I.RF = 0;
	if (!I.auto_clear_RF) I.auto_clear_RF = true;
}

// ---- the polling loops (i386_idle_enable) ----
static void idle_wpage_add(uint32_t page)
{
	for (int i = 0; i < s_idle_nwp; i++)
		if (s_idle_wpages[i] == page)
			return;
	if (s_idle_nwp < 64)
		s_idle_wpages[s_idle_nwp++] = (uint16_t)page;
	else
		s_idle_wover = 1;   // (too many pages to take out: no skipping)
}

static uint8_t *idle_wpage(uint32_t addr)
{
	for (int i = 0; i < s_idle_nwp; i++)
		if (s_idle_wpages[i] == addr >> 16)
			return s_idle_wsaved[i];
	return NULL;
}

static void idle_log(uint32_t a, const uint8_t *p)
{
	for (int i = 0; i < s_idle_wn; i++)
		if (s_idle_wa[i] == a)
			return;
	if (s_idle_wn == 64) {
		s_idle_fail = 1;
		return;
	}
	s_idle_wa[s_idle_wn] = a;
	s_idle_wv[s_idle_wn++] = *p;
}

static uint32_t idle_read32(uint32_t addr, uint32_t mask)
{
	for (int i = 0; i < 4; i++)
		if (((mask >> (i * 8)) & 0xff) && (!s_idle_pure || !s_idle_pure(addr + (uint32_t)i, 0)))
			s_idle_fail = 1;
	return s_idle_bus.read32(addr, mask);
}
static void idle_write32(uint32_t addr, uint32_t data, uint32_t mask)
{
	uint8_t *p = idle_wpage(addr);
	if (!p) {
		for (int i = 0; i < 4; i++)
			if (((mask >> (i * 8)) & 0xff) && (!s_idle_pure || !s_idle_pure(addr + (uint32_t)i, 1)))
				s_idle_fail = 1;
		s_idle_bus.write32(addr, data, mask);
		return;
	}
	p += addr & 0xffff;
	for (int i = 0; i < 4; i++)
		if ((mask >> (i * 8)) & 0xff) {
			idle_log(addr + (uint32_t)i, p + i);
			p[i] = (uint8_t)(data >> (i * 8));
		}
}
static uint32_t idle_io_read32(uint32_t addr, uint32_t mask)
{
	s_idle_fail = 1;
	return s_idle_bus.io_read32 ? s_idle_bus.io_read32(addr, mask) : 0;
}
static void idle_io_write32(uint32_t addr, uint32_t data, uint32_t mask)
{
	s_idle_fail = 1;
	if (s_idle_bus.io_write32)
		s_idle_bus.io_write32(addr, data, mask);
}

static void idle_traps(int on)
{
	if (on) {
		s_idle_bus = s_bus;
		s_bus.read32 = idle_read32;
		s_bus.write32 = idle_write32;
		s_bus.io_read32 = idle_io_read32;
		s_bus.io_write32 = idle_io_write32;
		for (int i = 0; i < s_idle_nwp; i++) {
			s_idle_wsaved[i] = s_wmap[s_idle_wpages[i]];
			s_wmap[s_idle_wpages[i]] = NULL;
		}
	} else {
		s_bus = s_idle_bus;
		for (int i = 0; i < s_idle_nwp; i++)
			s_wmap[s_idle_wpages[i]] = s_idle_wsaved[i];
	}
	s_idle_probing = on;
}

// at the start of a loop's iteration (the PC a taken backward transfer's target the third time in a row): one
// iteration run with the writes and the handler reads caught; the same state after it (every byte written back to
// its old value) = the iterations left in the run are the same, those whole ones are counted without being run
static void i386_idle_probe(void)
{
	s_ij_ok = 0;
	if (I.cycles < 64 || I.halted || I.spinning || (I.cr[0] & CR0_PG))
		return;
	const uint32_t pc = I.pc;
	static uint8_t snap[offsetof(i386_t, state_end)];
	memcpy(snap, &I, sizeof(snap));
	const int c0 = I.cycles;
	s_idle_fail = 0;
	s_idle_wn = 0;
	idle_traps(1);
	int T = 0, tried = 0, k;
	for (k = 0; k < 256; k++) {
		i386_step_rf();
		if (s_idle_fail || I.cycles <= 0 || I.halted || I.spinning)
			break;
		if (I.pc == pc) {
			tried = 1;
#define IDLE_SET(f) memcpy(snap + offsetof(i386_t, f), &I.f, sizeof(I.f))
			IDLE_SET(cycles);
			IDLE_SET(base_cycles);
			IDLE_SET(tsc);
			IDLE_SET(stolen);
#undef IDLE_SET
			int same = !memcmp(snap, &I, sizeof(snap));
			for (int i = 0; i < s_idle_wn && same; i++)
				same = idle_wpage(s_idle_wa[i])[s_idle_wa[i] & 0xffff] == s_idle_wv[i];
			if (same)
				T = c0 - I.cycles;
			break;
		}
	}
	idle_traps(0);
	const unsigned sl = (pc ^ (pc >> 6) ^ (pc >> 12)) & 63;
	if (T > 0) {
		const int left = I.cycles;
		if (left > 2 * T)
			I.cycles -= (left / T - 1) * T;
		if (s_idle_bad[sl] == pc)
			s_idle_badn[sl] = 0;
	} else if (tried || s_idle_fail || k == 256) {
		if (s_idle_bad[sl] != pc) {
			s_idle_bad[sl] = pc;
			s_idle_badn[sl] = 0;
		}
		if (s_idle_badn[sl] < 255)
			s_idle_badn[sl]++;
	}
}

// after an instruction that moved the PC back from pc0
static void i386_idle_see(void)
{
	const uint32_t t = I.pc;
	if (t != s_idle_tgt) {
		s_idle_tgt = t;
		s_idle_cnt = 0;
		return;
	}
	if (++s_idle_cnt < 2)
		return;
	s_idle_cnt = 0;
	const unsigned sl = (t ^ (t >> 6) ^ (t >> 12)) & 63;
	if (s_idle_bad[sl] == t && s_idle_badn[sl] >= 4)   // (a loop that failed its probes: left alone)
		return;
	i386_idle_probe();
}

void i386_idle_enable(int (*pure)(uint32_t addr, int write))
{
	s_idle_pure = pure;
	s_idle_tgt = 0xffffffffu;
	s_idle_cnt = 0;
	memset(s_idle_bad, 0xff, sizeof(s_idle_bad));
	memset(s_idle_badn, 0, sizeof(s_idle_badn));
	s_idle_on = !s_idle_wover;
	i386_jit_flush();   // (the blocks' backward transfers count for the idle test or not)
}

#include "i386_jit.inc"

static int i386_run_(int cycles);
#if defined(I386_JIT) && defined(__linux__)
#include <time.h>
// (RBX_I386_JITSTAT=1 on the harness: the time in i386_run, printed by i386_exit)
int i386_run(int cycles)
{
	if (!s_ij_stat)
		return i386_run_(cycles);
	struct timespec t0, t1;
	clock_gettime(CLOCK_MONOTONIC, &t0);
	const int r = i386_run_(cycles);
	clock_gettime(CLOCK_MONOTONIC, &t1);
	s_ij_us += (uint64_t)(t1.tv_sec - t0.tv_sec) * 1000000u + (uint64_t)((t1.tv_nsec - t0.tv_nsec) / 1000);
	return r;
}
#else
int i386_run(int cycles) { return i386_run_(cycles); }
#endif

static int i386_run_(int cycles)
{
	I.cycles = cycles;
	I.base_cycles = cycles;
	I.stolen = 0;
	if (I.halted || I.spinning) {
		I.tsc += (uint64_t)cycles;
		I.cycles = 0;
		return cycles;
	}
	CHANGE_PC(I.eip);
	// the faults thrown in an instruction come back here (MAME's catch in execute_run), the loop goes on
	jmp_buf jb, *const prev = s_jmp;
	s_jmp = &jb;
	if (setjmp(jb)) {
		if (s_idle_probing)   // (a fault in a probed instruction: the probe ends, the fault taken as usual)
			idle_traps(0);
		const uint64_t e = s_exc;
		I.ext = 1;
		s_ij_ok = 0;
		i386_trap_with_error((int)(e & 0xffffffff), 0, 0, (uint32_t)(e >> 32));
		if (I.RF && I.auto_clear_RF) I.RF = 0;
		if (!I.auto_clear_RF) I.auto_clear_RF = true;
	}
#ifdef I386_JIT
	const int jit = ij_enabled();
#endif
	while (I.cycles > 0 && !I.halted) {
#ifdef I386_JIT
		if (s_ij_tf && s_ij_tn == s_ij_tdet) ij_trace1();
#endif
#ifdef I386_JIT
		if (jit) {
			if (s_ij_ok || (s_ij_ok = ij_mode_ok())) {   // (the translated block at the pc, the same steps)
				const ij_fn f = ij_find(I.pc);
				if (f) {
					if (s_ij_slot && !(s_ij_dbg & 4))   // (the last block left by a transfer to here: chained from now on)
						ij_chain(f);
					s_ij_runs++;
					const uint32_t pc0 = f();
					const uint32_t pk = s_ij_pack;
					s_ij_build = !pk || ((pk >> 30) & 1);
					s_ij_slot = (pk & IJ_PACK_CHAIN) ? (uint32_t *)(uintptr_t)s_ij_xlr - 2 : NULL;
					if (s_idle_on && I.pc < pc0 && !(pk & IJ_PACK_IDLE))
						i386_idle_see();
					continue;
				}
			} else
				ij_limits();
		}
		s_ij_pack = 0;
		s_ij_ok = 0;
		s_ij_slot = NULL;
		s_ij_steps++;
		if (s_ij_stat) s_ij_lreg[I.pc < 0x200000 ? 0 : !s_ij_ok_dbg() ? 1 : 2]++;
#endif
		const uint32_t pc0 = I.pc;
		i386_step_rf();
		if (s_idle_on && I.pc < pc0)   // (a taken backward transfer)
			i386_idle_see();
#ifdef I386_JIT
		if (I.pc - pc0 > 15)   // (a transfer: a block may start here)
			s_ij_build = 1;
#endif
	}
#ifdef I386_JIT
	ij_apply();
	ij_trace();
#endif
	s_jmp = prev;
	if (I.halted && I.cycles > 0)   // (HLT: the rest of the slice idles)
		I.cycles = 0;
	const int done = cycles - I.cycles - I.stolen;
	I.tsc += (uint64_t)(cycles - I.cycles);
	return done;
}

// i386_postload
int i386_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "i386", &I, (uint32_t)offsetof(i386_t, state_end));
	if (load && !rc) {
		i386_jit_flush();
		s_ij_ok = 0;
		I.cycle_table_rm = s_cycle_table_rm;
		I.cycle_table_pm = s_cycle_table_pm;
		jmp_buf jb, *prev = s_jmp;
		s_jmp = &jb;
		if (!setjmp(jb)) {
			for (int i = 0; i < 6; i++)
				i386_load_segment_descriptor(i);
			CHANGE_PC(I.eip);
		}
		s_jmp = prev;
	}
	return rc;
}
