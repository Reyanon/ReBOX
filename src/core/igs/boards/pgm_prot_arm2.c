/*  PGM protection: IGS027A type 2 (55857F: Knights of Valour 2 / Plus, DoDonPachi 2, Martial Masters, Dragon World
    2001 / Pretty Chance), the ARM7 running its internal ROM ("prot") and the encrypted external ROM ("user1"),
    ported from MAME src/mame/igs/pgmprot_igs027a_type2.cpp (BSD-3-Clause, copyright-holders: David Haywood, ElSemi,
    iq_132).
    The external ROM is read through a decrypted copy, rebuilt for the words of a xor table entry when the ARM writes
    that entry (MAME xors each read), so the ARM executes it from mapped memory.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "pgm.h"
#include "../../cpu/arm7/arm7.h"
#include "../machine/pgmcrypt.h"

#include <stdlib.h>
#include <string.h>

static arm7_t *s_arm;
static uint8_t *s_irom;            // "prot", 0x4000
static uint32_t s_irom_size;
static const uint32_t *s_ext;      // "user1" (encrypted)
static uint32_t s_ext_words;
static uint32_t *s_extdec;         // decrypted copy (0x400000 bytes)
static uint32_t s_clock;
static int s_region_pc;            // the ARM pc of the region write (kov2_arm_region_w ...)

static struct {
	uint32_t latch_68k_w, latch_arm_w;
	uint32_t xor_table[0x100];
	uint8_t xor_ram[0x400];
	uint8_t iram[0x400];           // 0x10000000
	uint8_t arm_ram[0x10000];      // 0x18000000
	uint8_t share[0x10000];        // 0x48000000 (the 68000 sees it at 0xd00000)
	uint64_t frac;
} *s;

static void rebuild_ext(unsigned idx)
{
	const uint32_t n = s_ext_words < 0x100000 ? s_ext_words : 0x100000;
	for (uint32_t i = idx; i < n; i += 0x100)
		s_extdec[i] = s_ext[i] ^ s->xor_table[idx];
	arm7_jit_reset();   // (code translated from the old words: gone)
}

// ---- ARM bus (the unmapped part) ----
static uint32_t arm_r32(void *u, uint32_t a)
{
	(void)u;
	if (a == 0x38000000) {   // arm7_latch_arm_r
		arm7_set_irq_line(s_arm, ARM7_FIRQ_LINE, 0);
		return s->latch_68k_w;
	}
	if (a >= 0x08000000 && a <= 0x083fffff)
		return 0;   // (past the external ROM)
	return 0;
}
static uint16_t arm_r16(void *u, uint32_t a) { return (uint16_t)(arm_r32(u, a & ~3u) >> ((a & 2) * 8)); }
static uint8_t arm_r8(void *u, uint32_t a) { return (uint8_t)(arm_r32(u, a & ~3u) >> ((a & 3) * 8)); }

static void arm_w32m(uint32_t a, uint32_t d, uint32_t mask)
{
	if (a == 0x38000000) {
		s->latch_arm_w = (s->latch_arm_w & ~mask) | (d & mask);
		return;
	}
	if (a >= 0x48000000 && a <= 0x4800ffff) {   // the page with the region word
		const uint32_t o = a & 0xfffc;
		uint32_t *w = (uint32_t *)(s->share + o);
		if (o == 0x138) {
			const uint32_t hack = pgm_region_hack();
			if ((int)arm7_pc(s_arm) == s_region_pc && hack != 0xff)
				d = (d & 0xffff0000) | hack;
		}
		*w = (*w & ~mask) | (d & mask);
		return;
	}
	if (a >= 0x50000000 && a <= 0x500003ff) {   // xor_table_w (byte lane 0 of each word)
		uint32_t *w = (uint32_t *)(s->xor_ram + (a & 0x3fc));
		*w = (*w & ~mask) | (d & mask);
		if (mask & 0xff) {
			const unsigned idx = (a & 0x3ff) >> 2;
			const uint32_t data = *w & 0xff;
			s->xor_table[idx] = (data << 24) | (data << 8);
			rebuild_ext(idx);
		}
		return;
	}
}
static void arm_w32(void *u, uint32_t a, uint32_t d) { (void)u; arm_w32m(a, d, 0xffffffffu); }
static void arm_w16(void *u, uint32_t a, uint16_t d)
{
	(void)u;
	const int sh = (a & 2) * 8;
	arm_w32m(a & ~3u, (uint32_t)d << sh, 0xffffu << sh);
}
static void arm_w8(void *u, uint32_t a, uint8_t d)
{
	(void)u;
	const int sh = (a & 3) * 8;
	arm_w32m(a & ~3u, (uint32_t)d << sh, 0xffu << sh);
}

// ---- 68000 side ----
static int a2_read16(uint32_t a, uint16_t *v)
{
	if ((a & ~1u) == 0xd10000) {
		*v = (uint16_t)s->latch_arm_w;
		return 1;
	}
	return 0;
}

static int a2_write16(uint32_t a, uint16_t v, uint16_t mask)
{
	if ((a & ~1u) == 0xd10000) {
		s->latch_68k_w = (s->latch_68k_w & ~(uint32_t)mask) | (v & mask);
		arm7_set_irq_line(s_arm, ARM7_FIRQ_LINE, 1);
		return 1;
	}
	return 0;
}

// ---- the idle loop (ReBOX speed: the result is the same as running it) ----
// Between interrupts these games loop in Thumb code on
//     loop: bl count ; b loop        count: ldr r0,=word ; ldrh r1,[r0] ; adds r1,#1 ; strh r1,[r0] ; bx lr
// so the ARM's time goes into a 16-bit counter in its RAM. While it sits at the loop's head with no interrupt
// waiting, whole iterations are applied at once: the counter goes up by their number, r0, r1, lr and the flags are
// what the last one leaves, and their clocks are taken from the slice. The clocks of one iteration are measured
// once by running it an instruction at a time.
static struct { uint32_t head, word, cycles; } s_idle;

static uint16_t ext16(uint32_t a)
{
	if (a >= 0x08000000 && a < 0x08400000)
		return ((const uint16_t *)s_extdec)[(a - 0x08000000) >> 1];
	if (a < s_irom_size)
		return (uint16_t)(s_irom[a] | s_irom[a + 1] << 8);
	return 0;
}

static int idle_loop_at(uint32_t pc)
{
	if (pc == s_idle.head)
		return 1;
	const uint16_t h0 = ext16(pc), h1 = ext16(pc + 2), h2 = ext16(pc + 4);
	if ((h0 & 0xf800) != 0xf000 || (h1 & 0xf800) != 0xf800 || h2 != 0xe7fc)
		return 0;
	int32_t off = (int32_t)((uint32_t)(h0 & 0x7ff) << 21) >> 9;
	const uint32_t t = pc + 4 + (uint32_t)off + ((uint32_t)(h1 & 0x7ff) << 1);
	const uint16_t t0 = ext16(t);
	if ((t0 & 0xff00) != 0x4800 || ext16(t + 2) != 0x8801 || ext16(t + 4) != 0x3101 || ext16(t + 6) != 0x8001 ||
	    ext16(t + 8) != 0x4770)
		return 0;
	const uint32_t lit = ((t + 4) & ~3u) + (uint32_t)(t0 & 0xff) * 4;
	const uint32_t word = (uint32_t)ext16(lit) | (uint32_t)ext16(lit + 2) << 16;
	if (word < 0x18000000 || word > 0x1800fffe)
		return 0;
	s_idle.head = pc;
	s_idle.word = word;
	s_idle.cycles = 0;
	return 1;
}

static int arm_idle(int budget)
{
	uint32_t pc = arm7_pc(s_arm);
	if (!(s_arm->cpsr & 0x20) || s_arm->pending_interrupt)
		return budget;
	// inside the loop (a slice ended there): an instruction at a time up to its head
	if (s_idle.head && s_idle.cycles && pc != s_idle.head) {
		int k = 0;
		const uint32_t callee = s_idle.head + 4 + (uint32_t)((int32_t)((uint32_t)(ext16(s_idle.head) & 0x7ff) << 21) >> 9) +
		                        ((uint32_t)(ext16(s_idle.head + 2) & 0x7ff) << 1);
		while (((pc > s_idle.head && pc <= s_idle.head + 4) || (pc >= callee && pc <= callee + 8)) && k < 12 && budget > 0 &&
		       !s_arm->pending_interrupt) {
			budget -= arm7_run(s_arm, 1);
			pc = arm7_pc(s_arm);
			k++;
		}
		if (pc != s_idle.head || budget <= 0 || s_arm->pending_interrupt)
			return budget;
	}
	if (!idle_loop_at(pc))
		return budget;
	if (!s_idle.cycles) {   // (one iteration an instruction at a time: the same as a normal run)
		int c = 0;
		do
			c += arm7_run(s_arm, 1);
		while (arm7_pc(s_arm) != s_idle.head && c < 200 && !s_arm->pending_interrupt);
		if (arm7_pc(s_arm) != s_idle.head || c >= 200) {
			s_idle.head = 0;
			return budget - c;
		}
		s_idle.cycles = (uint32_t)c;
		budget -= c;
	}
	const uint32_t n = budget > 0 ? (uint32_t)budget / s_idle.cycles : 0;
	if (!n)
		return budget;
	uint16_t *w = (uint16_t *)(s->arm_ram + (s_idle.word & 0xfffe));
	const uint16_t last = (uint16_t)(*w + n - 1);
	*w = (uint16_t)(last + 1);
	s_arm->r[0] = s_idle.word;
	s_arm->r[1] = (uint32_t)last + 1;
	s_arm->r[14] = (s_idle.head + 4) | 1;
	s_arm->cpsr &= 0x0fffffff;   // (adds r1,#1 of a halfword: N, Z, C and V clear)
	s_arm->total_cycles += (int64_t)n * s_idle.cycles;
	return budget - (int)(n * s_idle.cycles);
}

static void a2_line(int line)
{
	(void)line;
	int budget = pgm_line_cycles(&s->frac, s_clock);
	budget = arm_idle(budget);
	if (budget > 0)
		arm7_run(s_arm, budget);
}

static void a2_exit(void)
{
	free(s_arm);
	free(s_extdec);
	free(s);
	s_arm = NULL;
	s_extdec = NULL;
	s = NULL;
}

static int a2_init(const char *init)
{
	s_irom = pgm_region("prot", &s_irom_size);
	uint32_t ext_size;
	uint8_t *ext = pgm_region("user1", &ext_size);
	s_arm = (arm7_t *)calloc(1, sizeof(arm7_t));
	s = calloc(1, sizeof(*s));
	s_extdec = (uint32_t *)calloc(1, 0x400000);
	if (!s_irom || !ext || !s_arm || !s || !s_extdec) {
		rbx_fail("IGS027A: ROM missing or out of memory");
		a2_exit();
		return -1;
	}
	// the decryption of the external ROM (MAME's init_*), and its ARM pc for the region hack
	s_region_pc = -1;
	s_clock = 20000000;
	if (!strcmp(init, "kov2")) { pgm_kov2_decrypt(ext, ext_size); s_region_pc = 0x190; }
	else if (!strcmp(init, "kov2p")) { pgm_kov2p_decrypt(ext, ext_size); s_region_pc = 0x1b0; }
	else if (!strcmp(init, "martmast")) { pgm_mm_decrypt(ext, ext_size); s_region_pc = 0x170; }
	else if (!strcmp(init, "ddp2")) pgm_ddp2_decrypt(ext, ext_size);
	else if (!strcmp(init, "dw2001") || !strcmp(init, "dwpc101j")) pgm_mm_decrypt(ext, ext_size);
	else if (!strcmp(init, "dwpc")) {
		pgm_dwpc_decrypt(ext, ext_size);
		if (s_irom_size > 0x3c8)
			s_irom[0x3c8] = 0x01;
	}
	if (strstr(pgm_game->machine, "_22m"))
		s_clock = 22000000;
	s_ext = (const uint32_t *)ext;
	s_ext_words = ext_size / 4;
	for (unsigned i = 0; i < 0x100; i++)
		rebuild_ext(i);
	arm7_init(s_arm);
	s_arm->idle_skip = 1;   // (ReBOX: waiting loops end the slice, arm7.h)
	s_arm->jit = 1;         // (ReBOX: its ROM code translated on ARM hosts, arm7_jit.inc)
	arm7_jit_reset();
	s_arm->read32 = arm_r32;
	s_arm->read16 = arm_r16;
	s_arm->read8 = arm_r8;
	s_arm->write32 = arm_w32;
	s_arm->write16 = arm_w16;
	s_arm->write8 = arm_w8;
	arm7_map(s_arm, 0x00000000, 0x00003fff, s_irom, 0);
	arm7_map(s_arm, 0x08000000, 0x083fffff, s_extdec, 0);
	arm7_map(s_arm, 0x10000000, 0x100003ff, s->iram, 1);
	arm7_map(s_arm, 0x18000000, 0x1800ffff, s->arm_ram, 1);
	arm7_map(s_arm, 0x48000000, 0x4800ffff, s->share, 1);
	arm7_map(s_arm, 0x48000000, 0x480003ff, s->share, 0);   // (the region word's page: writes to the handler)
	arm7_map(s_arm, 0x50000000, 0x500003ff, s->xor_ram, 0);
	pgm_map(0xd00000, 0xd0ffff, s->share, sizeof(s->share), 1);
	return 0;
}

static void a2_reset(void)
{
	// (MAME: the latches and the xor table are set in init only; the RAM keeps its contents)
	arm7_reset(s_arm);
	arm7_jit_reset();   // (the reset may patch the internal ROM)
}

static int a2_state(void *ctx, int load)
{
	uint8_t regs[ARM7_STATE_SIZE];
	if (!load)
		memcpy(regs, s_arm, ARM7_STATE_SIZE);
	int rc = rbx_state_io(ctx, "arm7", regs, ARM7_STATE_SIZE);
	rc |= rbx_state_io(ctx, "arm2", s, sizeof(*s));
	if (rc)
		return -1;
	if (load) {
		memcpy(s_arm, regs, ARM7_STATE_SIZE);
		for (unsigned i = 0; i < 0x100; i++)
			rebuild_ext(i);
	}
	return 0;
}

const pgm_prot pgm_prot_arm2 = { a2_init, a2_reset, a2_exit, a2_read16, a2_write16, a2_line, NULL, a2_state, 0x5fffff };
