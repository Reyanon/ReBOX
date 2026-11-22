/*  68000 bus for the Musashi core (see m68k_bus.h).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include <stdio.h>
#include "m68k_bus.h"
#include "m68k/m68k.h"

#include <string.h>

m68k_bus_t m68k_bus;
m68k_bus_t *m68k_bus_cur = &m68k_bus;

void (*rbx_m68k_int_hook)(int level);
void (*rbx_m68k_cmpild_hook)(int reg, unsigned int imm);
void (*rbx_m68k_rte_hook)(void);
void (*rbx_m68k_irqack_hook)(int level);

void m68k_bus_select(m68k_bus_t *b) { m68k_bus_cur = b ? b : &m68k_bus; }

void m68k_bus_reset_map(void)
{
	memset(m68k_bus_cur->rd, 0, sizeof(m68k_bus_cur->rd));
	memset(m68k_bus_cur->wr, 0, sizeof(m68k_bus_cur->wr));
	m68k_bus_cur->op = NULL;
	m68k_bus_cur->op_size = 0;
	m68k_bus_cur->nparts = 0;
	memset(m68k_bus_cur->partpg, 0, sizeof(m68k_bus_cur->partpg));
	m68k_bus_cur->read32 = NULL;
	m68k_bus_cur->write32 = NULL;
	rbx_m68k_cmpild_hook = NULL;   // (set again by the boards that use them)
	rbx_m68k_rte_hook = NULL;
	rbx_m68k_irqack_hook = NULL;
}

void m68k_bus_map(uint32_t start, uint32_t end, void *mem, uint32_t size, int readable, int writable)
{
	const uint32_t ps = 1u << M68K_PAGE_BITS;
	int partial = 0;
	if (end > 0xffffff)
		end = 0xffffff;
	for (uint32_t pb = start & ~(ps - 1); pb <= end; pb += ps) {
		const uint32_t o = pb >= start ? (pb - start) % size : 0;
		if (pb < start || pb + ps - 1 > end || o + ps > size) {   // (not the whole page: through the handlers' path)
			partial = 1;
			if (readable)
				m68k_bus_cur->rd[pb >> M68K_PAGE_BITS] = NULL;
			if (writable)
				m68k_bus_cur->wr[pb >> M68K_PAGE_BITS] = NULL;
			continue;
		}
		uint8_t *p = (uint8_t *)mem + o;
		if (readable)
			m68k_bus_cur->rd[pb >> M68K_PAGE_BITS] = p;
		if (writable)
			m68k_bus_cur->wr[pb >> M68K_PAGE_BITS] = p;
	}
	if (partial && m68k_bus_cur->nparts < (int)(sizeof(m68k_bus_cur->part) / sizeof(m68k_bus_cur->part[0]))) {
		const int n = m68k_bus_cur->nparts++;
		m68k_bus_cur->part[n].start = start;
		m68k_bus_cur->part[n].end = end;
		m68k_bus_cur->part[n].size = size;
		m68k_bus_cur->part[n].mem = (uint8_t *)mem;
		m68k_bus_cur->part[n].rd = (uint8_t)readable;
		m68k_bus_cur->part[n].wr = (uint8_t)writable;
		for (uint32_t pg = start >> M68K_PAGE_BITS; pg <= end >> M68K_PAGE_BITS; pg++)
			m68k_bus_cur->partpg[pg >> 5] |= 1u << (pg & 31);
	} else if (partial)
		fprintf(stderr, "m68k_bus_map: too many partial ranges (%06x-%06x)\n", (unsigned)start, (unsigned)end);
}

// a partial range overlaps the page of a (a: 24 bits)
static inline int part_pg(uint32_t a)
{
	return (m68k_bus_cur->partpg[a >> (M68K_PAGE_BITS + 5)] >> ((a >> M68K_PAGE_BITS) & 31)) & 1;
}

// the word (host order) of a partial range holding byte address a, or NULL
static uint8_t *part_word(uint32_t a, int write)
{
	for (int i = m68k_bus_cur->nparts - 1; i >= 0; i--) {
		const uint32_t s = m68k_bus_cur->part[i].start;
		if (a >= s && a <= m68k_bus_cur->part[i].end && (write ? m68k_bus_cur->part[i].wr : m68k_bus_cur->part[i].rd))
			return m68k_bus_cur->part[i].mem + (((a & ~1u) - s) % m68k_bus_cur->part[i].size);
	}
	return NULL;
}

void m68k_bus_swap16(uint8_t *p, uint32_t size)
{
	for (uint32_t i = 0; i + 1 < size; i += 2) {
		uint8_t t = p[i];
		p[i] = p[i + 1];
		p[i + 1] = t;
	}
}

unsigned int m68k_read_memory_8(unsigned int a)
{
	a &= 0xffffff;
	uint8_t *p = m68k_bus_cur->rd[a >> M68K_PAGE_BITS];
	if (p)
		return p[(a & ((1u << M68K_PAGE_BITS) - 1)) ^ M68K_BYTE_XOR];
	if (part_pg(a) && (p = part_word(a, 0)) != NULL)
		return p[(a & 1) ^ M68K_BYTE_XOR];
	return m68k_bus_cur->read8(a);
}

unsigned int m68k_read_memory_16(unsigned int a)
{
	a &= 0xffffff;
	if (a & 1)   // 68020 misaligned access (the 68000 takes an address error instead)
		return m68k_read_memory_8(a) << 8 | m68k_read_memory_8(a + 1);
	uint8_t *p = m68k_bus_cur->rd[a >> M68K_PAGE_BITS];
	if (p)
		return *(uint16_t *)(p + (a & ((1u << M68K_PAGE_BITS) - 1)));
	if (part_pg(a) && (p = part_word(a, 0)) != NULL)
		return *(uint16_t *)p;
	return m68k_bus_cur->read16(a);
}

unsigned int m68k_read_memory_32(unsigned int a)
{
	// both words in one directly mapped page: one lookup (otherwise word by word, high word first)
	const unsigned int m = a & 0xffffff, off = m & ((1u << M68K_PAGE_BITS) - 1);
	if (!(m & 1) && off <= (1u << M68K_PAGE_BITS) - 4) {
		const uint8_t *p = m68k_bus_cur->rd[m >> M68K_PAGE_BITS];
		if (p)
			return (unsigned int)*(const uint16_t *)(p + off) << 16 | *(const uint16_t *)(p + off + 2);
		if (m68k_bus_cur->read32 && !m68k_bus_cur->nparts)
			return m68k_bus_cur->read32(m);
	}
	return m68k_read_memory_16(a) << 16 | m68k_read_memory_16(a + 2);
}

void m68k_write_memory_8(unsigned int a, unsigned int v)
{
	a &= 0xffffff;
	uint8_t *p = m68k_bus_cur->wr[a >> M68K_PAGE_BITS];
	if (p)
		p[(a & ((1u << M68K_PAGE_BITS) - 1)) ^ M68K_BYTE_XOR] = (uint8_t)v;
	else if (part_pg(a) && (p = part_word(a, 1)) != NULL)
		p[(a & 1) ^ M68K_BYTE_XOR] = (uint8_t)v;
	else
		m68k_bus_cur->write8(a, (uint8_t)v);
}

void m68k_write_memory_16(unsigned int a, unsigned int v)
{
	a &= 0xffffff;
	if (a & 1) {   // 68020 misaligned access
		m68k_write_memory_8(a, (v >> 8) & 0xff);
		m68k_write_memory_8(a + 1, v & 0xff);
		return;
	}
	uint8_t *p = m68k_bus_cur->wr[a >> M68K_PAGE_BITS];
	if (p)
		*(uint16_t *)(p + (a & ((1u << M68K_PAGE_BITS) - 1))) = (uint16_t)v;
	else if (part_pg(a) && (p = part_word(a, 1)) != NULL)
		*(uint16_t *)p = (uint16_t)v;
	else
		m68k_bus_cur->write16(a, (uint16_t)v);
}

void m68k_write_memory_32(unsigned int a, unsigned int v)
{
	const unsigned int m = a & 0xffffff, off = m & ((1u << M68K_PAGE_BITS) - 1);
	if (!(m & 1) && off <= (1u << M68K_PAGE_BITS) - 4) {
		uint8_t *p = m68k_bus_cur->wr[m >> M68K_PAGE_BITS];
		if (p) {
			*(uint16_t *)(p + off) = (uint16_t)(v >> 16);
			*(uint16_t *)(p + off + 2) = (uint16_t)v;
			return;
		}
		if (m68k_bus_cur->write32 && !part_pg(m)) {   // (an unmapped page: through the handlers)
			m68k_bus_cur->write32(m, v);
			return;
		}
	}
	m68k_write_memory_16(a, v >> 16);
	m68k_write_memory_16(a + 2, v & 0xffff);
}

unsigned int m68k_read_disassembler_8(unsigned int a) { return m68k_read_memory_8(a); }
unsigned int m68k_read_disassembler_16(unsigned int a) { return m68k_read_memory_16(a); }
unsigned int m68k_read_disassembler_32(unsigned int a) { return m68k_read_memory_32(a); }

// ---- interrupts: HOLD_LINE levels cleared when taken, plus ASSERT_LINE levels ----
static void update_irq(void)
{
	uint32_t lines = (m68k_bus_cur->irq_pending | m68k_bus_cur->irq_asserted) & 0xfe;   // (levels 1-7)
	if (m68k_bus_cur->nmi_taken && !(m68k_bus_cur->irq_pending & 0x80))
		lines &= ~0x80u;   // (Musashi takes 7 on its rising edge: raising it again would take it again)
	m68k_set_irq(lines ? 31 - __builtin_clz(lines) : 0);   // (the highest level up)
}

static void int_taken(int level)
{
	if (level == 7 && (m68k_bus_cur->irq_asserted & 0x80))
		m68k_bus_cur->nmi_taken = 1;
	m68k_bus_cur->irq_pending &= ~(1u << level);
	update_irq();
}

void m68k_bus_hold_irq(int level)
{
	rbx_m68k_int_hook = int_taken;
	m68k_bus_cur->irq_pending |= 1u << level;
	update_irq();
}

void m68k_bus_clear_irq(int level)
{
	m68k_bus_cur->irq_pending &= ~(1u << level);
	update_irq();
}

void m68k_bus_irq_reset(void)
{
	m68k_bus_cur->irq_pending = 0;
	m68k_bus_cur->irq_asserted = 0;
	m68k_bus_cur->nmi_taken = 0;
	rbx_m68k_int_hook = int_taken;
	m68k_set_irq(0);
}

uint32_t m68k_bus_irq_pending(void) { return m68k_bus_cur->irq_pending; }
void m68k_bus_set_irq_pending(uint32_t mask) { m68k_bus_cur->irq_pending = mask; update_irq(); }

void m68k_bus_set_line(int level, int state)
{
	rbx_m68k_int_hook = int_taken;
	if (state)
		m68k_bus_cur->irq_asserted |= 1u << level;
	else {
		m68k_bus_cur->irq_asserted &= ~(1u << level);
		if (level == 7)
			m68k_bus_cur->nmi_taken = 0;
	}
	update_irq();
}

void m68k_bus_update_irq(void) { update_irq(); }

uint8_t rbx_board_peek(uint32_t addr) { return (uint8_t)m68k_read_memory_8(addr); }
void rbx_board_poke(uint32_t addr, uint8_t v) { m68k_write_memory_8(addr, v); }
