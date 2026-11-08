/*  68000 bus for the Musashi core: 4KB page table for RAM/ROM, board handlers for the rest,
    and HOLD_LINE interrupt lines (cleared when the CPU takes them, as in MAME).
    Memory is kept in host-order 16-bit words (byte n of a word is at (n ^ 1) on little-endian hosts).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_M68K_BUS_H
#define RBX_M68K_BUS_H

#include <stdint.h>

#define M68K_PAGE_BITS 12
#define M68K_PAGES (1 << (24 - M68K_PAGE_BITS))

typedef struct {
	uint8_t *rd[M68K_PAGES];   // page base for reads (NULL = handler)
	uint8_t *wr[M68K_PAGES];   // page base for writes (NULL = handler)
	uint8_t (*read8)(uint32_t a);
	uint16_t (*read16)(uint32_t a);
	void (*write8)(uint32_t a, uint8_t v);
	void (*write16)(uint32_t a, uint16_t v);
	uint32_t irq_pending;      // HOLD_LINE levels (bit n = level n), cleared when the CPU takes them
	uint32_t irq_asserted;     // ASSERT_LINE levels (m68k_bus_set_line), held until cleared
	// level 7 (the NMI: taken on its edge) taken while its line stays asserted: not raised again until it drops
	uint32_t nmi_taken;
	// MAME's AS_OPCODES (encrypted CPUs, DECO 102): the instruction stream's words (opcodes, extension words,
	// immediates) below op_size come from op (host-order words from 0); NULL: from the bus as any read
	const uint8_t *op;
	uint32_t op_size;
	// the mapped ranges that do not cover whole pages (m68k_bus_map): kept out of the page table, looked up before
	// the board's handlers (the newest first)
	struct { uint32_t start, end, size; uint8_t *mem; uint8_t rd, wr; } part[24];
	int nparts;
} m68k_bus_t;

// the bus of the board CPU. Boards with two 68000s keep one m68k_bus_t per CPU and switch with m68k_bus_select
// (together with m68k_set_context); every function below works on the selected bus
extern m68k_bus_t m68k_bus;
extern m68k_bus_t *m68k_bus_cur;
void m68k_bus_select(m68k_bus_t *b);   // NULL = m68k_bus

// boards with two 68000s (one Musashi core, contexts swapped with m68k_get_context / m68k_set_context): select
// the page table and handlers of the CPU about to run (NULL = m68k_bus). Mapping functions act on the selected bus
void m68k_bus_select(m68k_bus_t *b);

void m68k_bus_reset_map(void);
// map [start, end] (inclusive) to host memory; mirror = size of the memory block (wraps). The pages it covers whole go
// in the page table; the rest of the range (a block smaller than a page, an unaligned start or end) is reached
// through the handlers' path, so that nothing past the block is touched
void m68k_bus_map(uint32_t start, uint32_t end, void *mem, uint32_t size, int readable, int writable);

#ifdef RBX_LITTLE_ENDIAN_HOST
#endif
#define M68K_BYTE_XOR 1   // all supported hosts (ARM, MIPS PSP, x86) are little-endian

// swap a big-endian byte image into host 16-bit words in place
void m68k_bus_swap16(uint8_t *p, uint32_t size);

// interrupt lines (HOLD_LINE): raise a level; the bus clears it when the CPU acknowledges
void m68k_bus_hold_irq(int level);
void m68k_bus_clear_irq(int level);
void m68k_bus_irq_reset(void);
uint32_t m68k_bus_irq_pending(void);
void m68k_bus_set_irq_pending(uint32_t mask);
// level interrupt line (MAME ASSERT_LINE / CLEAR_LINE): stays up through acknowledges until cleared
void m68k_bus_set_line(int level, int state);
// hand the interrupt level of the selected bus to the CPU (after m68k_set_context)
void m68k_bus_update_irq(void);

static inline uint16_t m68k_word(const uint8_t *hostmem, uint32_t a) { return *(const uint16_t *)(hostmem + (a & ~1u)); }

#endif
