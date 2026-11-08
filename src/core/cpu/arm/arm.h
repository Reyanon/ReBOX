/*  Acorn ARM2 / ARM3 CPU (26-bit address bus, PSR held in R15), ported to C from MAME src/devices/cpu/arm
    (arm.cpp, arm.h; MAME 0.289; BSD-3-Clause, copyright-holders: Bryan McPhail, Phil Stroffolino).
    Instruction semantics, the pipelined R15 offsets, flag rules, block-transfer quirks, the coprocessor handling
    (Data East 156 BCD/divider copro, VL86C020 ID/control copro) and the cycle counts follow MAME's arm_cpu_device.
    The disassembler and the debugger state registration are not ported.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_ARM_H
#define RBX_ARM_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { ARM_COPRO_UNKNOWN_CP15 = 0, ARM_COPRO_VL86C020 = 1 };
enum { ARM_IRQ_LINE = 0, ARM_FIRQ_LINE = 1 };

#define ARM_PAGE_SHIFT 12
#define ARM_PAGES      (1u << (26 - ARM_PAGE_SHIFT))   // 16384 pages of 4 KB cover the 26-bit space

typedef struct arm_s arm_t;
struct arm_s {
	// r[0..15]: the registers seen in bank_mode (R15 = NZCVIF|PC|mode). bank[]: MAME's m_sArmRegister[8..26] (R8-R14
	// user, R8-R14 FIQ, R13-R14 IRQ, R13-R14 SVC) for the registers not seen in bank_mode (the seen ones are in r[]).
	// The core switches the bank when it changes R15's mode bits; arm_step / arm_run also switch it first when R15's
	// mode differs from bank_mode (R15 written from outside). arm_reg gives any register by MAME index.
	uint32_t r[16];
	uint32_t bank[19];
	uint32_t copro[16];        // m_coproRegister
	uint8_t pending_irq, pending_fiq;
	uint8_t copro_type;        // ARM_COPRO_*: set before use (MAME set_copro_type), kept by reset
	uint8_t bank_mode;         // the mode r[8..14] belong to
	int64_t cycles;            // clocks executed since reset
	// bus (not saved). Pages hold the guest memory as 32-bit words in host (little-endian) order: a word access
	// reads/writes the aligned host word; a byte access uses byte (a & 0xfff) for little-endian, (a & 0xfff) ^ 3 when
	// big_endian is set (the lanes of a big-endian 32-bit bus). NULL page = use the callbacks. The word callbacks get
	// the address masked to 0x03fffffc, the byte callbacks the 26-bit address unchanged (a big-endian bus picks the
	// lane 3 - (a & 3) itself, as a MAME ENDIANNESS_BIG 32-bit space does).
	uint8_t *rpage[ARM_PAGES];
	uint8_t *wpage[ARM_PAGES];
	uint32_t (*read32)(void *ctx, uint32_t a);
	void (*write32)(void *ctx, uint32_t a, uint32_t v);
	uint8_t (*read8)(void *ctx, uint32_t a);
	void (*write8)(void *ctx, uint32_t a, uint8_t v);
	void *ctx;
	int big_endian;            // ARM_BE variant
	// called when an IRQ / FIQ is taken (where MAME calls standard_irq_callback), before the vector instruction;
	// line = ARM_IRQ_LINE / ARM_FIRQ_LINE. Lets a driver drop a HOLD_LINE input. NULL = not called
	void (*irq_ack)(void *ctx, int line);
};

// clear registers and the bus tables (call once, then set the bus and arm_reset)
void arm_init(arm_t *c);
// MAME device_reset: registers and copro registers 0, lines clear, SVC mode with IRQ and FIQ masked, PC 0
void arm_reset(arm_t *c);
// interrupt check (MAME arm_check_irq_state) then one instruction; returns the clocks used
int arm_step(arm_t *c);
// run until c->cycles >= until, or until arm_break is set (checked after each instruction)
void arm_run(arm_t *c, int64_t until);
void arm_set_irq(arm_t *c, int state);
void arm_set_fiq(arm_t *c, int state);
// MAME pc() (STATE_GENPC = R15 & 0x03fffffc): inside a memory handler, the address of the executing instruction
// (R15 is advanced only after the instruction; an STM with R15 in the list runs with R15 + 12, as in MAME)
static inline uint32_t arm_pc(const arm_t *c) { return c->r[15] & 0x03fffffcu; }
// a register by MAME index (0-15 R0-R15 user, 16-22 R8-R14 FIQ, 23-24 R13-R14 IRQ, 25-26 R13-R14 SVC), wherever the
// bank keeps it now
uint32_t *arm_reg(arm_t *c, int index);
// write R15 from outside and switch the bank to its mode
void arm_set_r15(arm_t *c, uint32_t v);

// ReBOX: set (by a memory handler) to end arm_run after the instruction that set it (the caller clears it)
extern int arm_break;

// save state: the register block (from r up to cycles) as one blob (consistent by itself: bank_mode is in it)
#define ARM_STATE_SIZE ((uint32_t)(offsetof(arm_t, rpage)))

#ifdef __cplusplus
}
#endif

#endif
