/*  PGM protection: IGS027A type 3 (55857G: Demon Front, The Gladiator, Spectral vs. Generation, Happy 6-in-1,
    The Killing Blade Plus), the ARM7 running its internal ROM ("prot", partly or wholly replaced by MAME's fake code
    where it is not dumped) and the external ROM ("user1"), with two banks of shared RAM swapped between the CPUs.
    Ported from MAME src/mame/igs/pgmprot_igs027a_type3.cpp (BSD-3-Clause, copyright-holders: David Haywood,
    Xing Xing).
    MAME's speedup handlers sit on mapped memory; here the 1 KB ARM page (or the 4 KB 68000 page) holding the word is
    left unmapped and served by the bus callbacks, which check the pc as MAME does.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "pgm.h"
#include "../../cpu/arm7/arm7.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/m68k_bus.h"
#include "../machine/pgmcrypt.h"

#include <stdlib.h>
#include <string.h>

static arm7_t *s_arm;
static uint8_t *s_irom;            // "prot", 0x4000
static uint32_t s_irom_size;
static uint8_t *s_ext;             // "user1" (decrypted in place)
static uint32_t s_ext_size;
static uint32_t s_clock;

// ARM speedup (theglad_speedup_r ...): the word at sp_addr read by the instruction at sp_pc eats 500 cycles; the
// 1 KB page at sp_page (host memory sp_mem) goes through the callbacks
static uint32_t s_sp_addr, s_sp_pc, s_sp_page;
static uint8_t *s_sp_mem;
static int s_sp_writable;
static int s_main_speedup;         // dmnfrnt_main_speedup_r

static struct {
	uint32_t latch_68k_w, latch_arm_w;
	uint32_t ram_sel;
	uint32_t fiq_pulse;            // svg_68k_nmi_w: the FIQ line is up until taken (or one ARM slice)
	uint8_t share[2][0x20000];     // m_svg_shareram (the ARM sees bank ram_sel at 0x38000000, the 68000 the other)
	uint8_t arm_ram[0x40000];      // 0x18000000
	uint8_t arm_ram2[0x400];       // 0x10000000
	uint8_t ram50[0x400];          // 0x50000000
	uint64_t frac;
} *s;

static void map_share(void)
{
	const int sel = s->ram_sel & 1;
	arm7_map(s_arm, 0x38000000, 0x3800ffff, s->share[sel], 1);
	pgm_map(0x500000, 0x50ffff, s->share[sel ^ 1], 0x10000, 1);
}

// ---- ARM bus (the unmapped part) ----
static uint32_t arm_r32(void *u, uint32_t a)
{
	(void)u;
	if (s_sp_mem && a - s_sp_page < 0x400) {
		if ((a & ~3u) == s_sp_addr && arm7_pc(s_arm) == s_sp_pc)
			arm7_eat_cycles(s_arm, 500);
		return *(const uint32_t *)(s_sp_mem + (a & 0x3fc));
	}
	if (a == 0x48000000)   // svg_latch_arm_r
		return s->latch_68k_w;
	return 0;
}
static uint16_t arm_r16(void *u, uint32_t a) { return (uint16_t)(arm_r32(u, a & ~3u) >> ((a & 2) * 8)); }
static uint8_t arm_r8(void *u, uint32_t a) { return (uint8_t)(arm_r32(u, a & ~3u) >> ((a & 3) * 8)); }

static void arm_w32m(uint32_t a, uint32_t d, uint32_t mask)
{
	if (s_sp_mem && a - s_sp_page < 0x400) {
		if (s_sp_writable) {
			uint32_t *w = (uint32_t *)(s_sp_mem + (a & 0x3fc));
			*w = (*w & ~mask) | (d & mask);
		}
		return;
	}
	if (a == 0x48000000) {   // svg_latch_arm_w
		s->latch_arm_w = (s->latch_arm_w & ~mask) | (d & mask);
		return;
	}
	if (a == 0x40000018) {   // svg_arm7_ram_sel_w (MAME synchronizes: the 68000 is already ahead here)
		const uint32_t sel = d & 1;
		if (sel != s->ram_sel) {
			s->ram_sel = sel;
			map_share();
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

static void arm_irq_ack(void *u, int line)
{
	(void)u;
	if (line == ARM7_FIRQ_LINE && s->fiq_pulse) {
		s->fiq_pulse = 0;
		arm7_set_irq_line(s_arm, ARM7_FIRQ_LINE, 0);
	}
}

// ---- 68000 side ----
static int a3_read16(uint32_t a, uint16_t *v)
{
	a &= ~1u;
	if (a == 0x5c0000) {   // svg_68k_nmi_r
		*v = 0;
		return 1;
	}
	if (a == 0x5c0300) {   // svg_latch_68k_r
		*v = (uint16_t)s->latch_arm_w;
		return 1;
	}
	if (s_main_speedup && (a & 0xfff000) == 0x80a000) {   // the page of dmnfrnt_main_speedup_r
		*v = pgm_mainram[(a & 0x1ffff) >> 1];
		if (a == 0x80a03c) {
			const uint32_t pc = m68k_get_reg(NULL, M68K_REG_PPC);
			if (pc == 0x10193a || pc == 0x1019a4)
				pgm_m68k_spin();
		}
		return 1;
	}
	return 0;
}

static int a3_write16(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= ~1u;
	if (a == 0x5c0000) {   // svg_68k_nmi_w: pulse the ARM's FIQ
		s->fiq_pulse = 1;
		arm7_set_irq_line(s_arm, ARM7_FIRQ_LINE, 1);
		return 1;
	}
	if (a == 0x5c0300) {   // svg_latch_68k_w
		s->latch_68k_w = (s->latch_68k_w & ~(uint32_t)mask) | (v & mask);
		return 1;
	}
	return 0;
}

static void a3_line(int line)
{
	(void)line;
	arm7_run(s_arm, pgm_line_cycles(&s->frac, s_clock));
	if (s->fiq_pulse) {   // (not taken within the pulse: dropped)
		s->fiq_pulse = 0;
		arm7_set_irq_line(s_arm, ARM7_FIRQ_LINE, 0);
	}
}

// ---- MAME's replacement internal ROM code ----
static void w16(int off, uint16_t v)
{
	if ((uint32_t)off + 1 < s_irom_size)
		((uint16_t *)s_irom)[off / 2] = v;
}

// pgm_create_dummy_internal_arm_region
static void dummy_internal_arm_region(int size)
{
	for (int i = 0; i < size / 2; i += 2) {   // fill with BX R14
		w16(i * 2, 0xff1e);
		w16(i * 2 + 2, 0xe12f);
	}
	// jump straight to the external area
	w16(0x0000, 0xd088);
	w16(0x0002, 0xe59f);
	w16(0x0004, 0x0680);
	w16(0x0006, 0xe3a0);
	w16(0x0008, 0xff10);
	w16(0x000a, 0xe12f);
	w16(0x0090, 0x0400);
	w16(0x0092, 0x1000);
}

static int wseq(int base, const uint16_t *v, int n)
{
	for (int i = 0; i < n; i++, base += 2)
		w16(base, v[i]);
	return base;
}

// pgm_create_dummy_internal_arm_region_theglad
static void dummy_internal_arm_region_theglad(int is_svg)
{
	for (int i = 0; i < 0x188 / 2; i += 2) {
		w16(i * 2, 0xfffe);
		w16(i * 2 + 2, 0xeaff);
	}
	// the interrupt code appears to be at 0x08000010: point the FIQ vector there
	int base = 0x1c;
	static const uint16_t fiq[] = { 0xf000, 0xe59f };
	base = wseq(base, fiq, 2);
	if (is_svg == 0) {
		static const uint16_t t[] = { 0x0010, 0x0800, 0x0010, 0x0800 };
		base = wseq(base, t, 4);
	} else {
		static const uint16_t t[] = { 0x0038, 0x0800, 0x0038, 0x0800 };
		base = wseq(base, t, 4);
	}
	// startup code to set up the stacks (as killbldp)
	static const uint16_t start[] = {
		0x00D2, 0xE3A0, 0xF000, 0xE121,
		0x4001, 0xE3A0, 0x4B06, 0xE284, 0x0CFA, 0xE3A0, 0xD804, 0xE080, 0x00D1, 0xE3A0, 0xF000, 0xE121,
		0x0CF6, 0xE3A0, 0xD804, 0xE080, 0x00D7, 0xE3A0, 0xF000, 0xE121, 0x0CFF, 0xE3A0, 0xD804, 0xE080,
		0x00DB, 0xE3A0, 0xF000, 0xE121, 0x4140, 0xE1C4, 0x0CFE, 0xE3A0, 0xD804, 0xE080, 0x00D3, 0xE3A0,
		0xF000,
		0xE121, 0x4A01, 0xE3A0, 0x0B01, 0xE3A0, 0xD804, 0xE080, 0x5A0F, 0xE3A0, 0x0008, 0xE3A0, 0x8805,
		0xE080, 0x0010, 0xE3A0, 0x0000, 0xE5C8, 0x7805, 0xE1A0, 0x6A01, 0xE3A0, 0x0012, 0xE3A0, 0x0A02,
		0xE280, 0x6806, 0xE080, 0x6000, 0xE587,
		// set the SR13 to something sensible
		0x00D3, 0xE3A0, 0xF000, 0xE121, 0x4001, 0xE3A0, 0x4B06, 0xE284, 0x0CF2, 0xE3A0, 0xD804, 0xE080,
		0x0013, 0xE3A0, 0xF000, 0xE121,
		0x0028, 0xEA00,   // jump to 0x184
	};
	wseq(0x30, start, (int)(sizeof(start) / sizeof(start[0])));

	static const uint16_t rst[] = { 0x000a, 0xEA00 };
	wseq(0, rst, 2);

	// 0xe8 / 0xfc: as killbldp; 0x110: set up the stack again, soft reset with a RAM variable set to 0
	static const uint16_t e8[] = {
		0xE004, 0xE52D, 0x00D3, 0xE3A0, 0xF000, 0xE121, 0xE004, 0xE49D, 0xFF1E, 0xE12F,
		0xE004, 0xE52D, 0x0013, 0xE3A0, 0xF000, 0xE121, 0xE004, 0xE49D, 0xFF1E, 0xE12F,
		0x00D1, 0xE3A0, 0xF000, 0xE121, 0xD0b8, 0xE59F, 0x00D3, 0xE3A0, 0xF000, 0xE121, 0xD0b0, 0xE59F,
		0x10b8, 0xE59F, 0x0000, 0xE3A0, 0x0000, 0xE581,
	};
	base = wseq(0xe8, e8, (int)(sizeof(e8) / sizeof(e8[0])));
	if (is_svg == 0) {   // jump to the start of the external ROM
		w16(base, 0xF302);
		w16(base + 2, 0xE3A0);
	} else {
		w16(base, 0xf0b0);
		w16(base + 2, 0xe59f);
	}

	w16(0x150, 0xff1e);
	w16(0x152, 0xe12f);

	// the non-EO area starts in the middle of a function (sets 0x10000038 to 1)
	w16(0x184, 0x105c);
	w16(0x186, 0xE59F);
}

// pgm_patch_external_arm_rom_jumptable_theglada
static void patch_jumptable_theglada(uint32_t base)
{
	static const uint16_t sub[] = {
		0x00FC, 0x00E8, 0x0110, 0x0150, 0x0194, 0x06C8, 0x071C, 0x0728,
		0x0734, 0x0740, 0x0784, 0x0794, 0x07FC, 0x0840, 0x086C, 0x0988,
		0x0A54, 0x0AA8, 0x0AD4, 0x0EB8, 0x0EF8, 0x0F2C, 0x0F3C, 0x0F78,
		0x0FA8, 0x0FD8, 0x1028, 0x1038, 0x1048, 0x1058, 0x1068, 0x1070,
		0x1090, 0x10B0, 0x10D4, 0x1100, 0x113C, 0x1198, 0x1234, 0x1258,
		0x127C, 0x12A8, 0x12E4, 0x1368, 0x142C, 0x0B10, 0x0B54, 0x0B74,
		0x0C08, 0x0C90, 0x0D18, 0x0D90, 0x1570, 0x1600, 0x1640, 0x1694,
		0x1730, 0x176C, 0x17AC, 0x17D8, 0x18C4, 0x18E0, 0x1904, 0x1930,
		0x19D8, 0x1A38, 0x1950, 0x1970, 0x1990, 0x19B8, 0x19C8, 0x1A9C,
		0x1AC4, 0x1AE8, 0x1B20, 0x1B48, 0x1B70, 0x1B8C, 0x1BB4, 0x1BD8,
		0x1BFC, 0x1C10, 0x1C24, 0x1CA0, 0x1D5C, 0x1D7C, 0x1D8C, 0x1DAC,
		0x1DCC, 0x1DE0, 0x1DF4, 0x1E1C, 0x1E2C, 0x1E60, 0x1E94, 0x1EA4,
		0x1ECC, 0x1ED8, 0x1EE4, 0x1F14, 0x1F44, 0x1FB4, 0x1FC4, 0x2040,
		0x20BC, 0x2140, 0x21C4, 0x2240, 0x22BC, 0x2340, 0x23C4, 0x23D0,
		0x2400, 0x2430, 0x244C, 0x245C, 0x246C, 0x2FCC, 0x3000, 0x3028,
		0x3050, 0x30A4, 0x30F8, 0x3120, 0x249C, 0x24C0, 0x27BC, 0x2B40,
		0x2BF4, 0x2CD8, 0x2E2C
	};
	for (size_t i = 0; i < sizeof(sub) / sizeof(sub[0]); i++, base += 4)   // (the low halfword of each entry)
		if (base + 1 < s_ext_size)
			((uint16_t *)s_ext)[base / 2] = sub[i];
}

// pgm_descramble_happy6 + pgm_descramble_happy6_2 on 0x800000 bytes
static int descramble_happy6(uint8_t *src, uint8_t *buf)
{
	int w = 0;
	for (int j = 0; j < 0x800; j += 0x200)
		for (int i = j; i < 0x800000; i += 0x800) {
			memcpy(buf + w, src + i, 0x200);
			w += 0x200;
		}
	memcpy(src, buf, 0x800000);
	w = 0;
	for (int k = 0; k < 0x800000; k += 0x100000)
		for (int j = 0; j < 0x40000; j += 0x10000)
			for (int i = j; i < 0x100000; i += 0x40000) {
				memcpy(buf + w, src + i + k, 0x10000);
				w += 0x10000;
			}
	memcpy(src, buf, 0x800000);
	return 0;
}

static int init_happy6_gfx(void)
{
	static const struct { const char *name; uint32_t off; } r[] = {
		{ "igs023", 0x180000 }, { "igs023:sprcol", 0 }, { "igs023:sprcol", 0x800000 },
		{ "igs023:sprmask", 0 }, { "ics", 0x400000 },
	};
	uint8_t *buf = (uint8_t *)malloc(0x800000);
	if (!buf)
		return -1;
	for (size_t i = 0; i < sizeof(r) / sizeof(r[0]); i++) {
		uint32_t size;
		uint8_t *p = pgm_region(r[i].name, &size);
		if (p && size >= r[i].off + 0x800000)
			descramble_happy6(p + r[i].off, buf);
	}
	free(buf);
	return 0;
}

// ---- setup ----
static void a3_exit(void)
{
	free(s_arm);
	free(s);
	s_arm = NULL;
	s = NULL;
	s_sp_mem = NULL;
	s_main_speedup = 0;
}

static void speedup(uint32_t addr, uint32_t pc, uint8_t *page_mem, int writable)
{
	s_sp_addr = addr;
	s_sp_pc = pc;
	s_sp_page = addr & ~0x3ffu;
	s_sp_mem = page_mem;
	s_sp_writable = writable;
}

static int a3_init(const char *init)
{
	s_irom = pgm_region("prot", &s_irom_size);
	s_ext = pgm_region("user1", &s_ext_size);
	s_arm = (arm7_t *)calloc(1, sizeof(arm7_t));
	s = calloc(1, sizeof(*s));
	s_sp_mem = NULL;
	s_main_speedup = 0;
	if (!s_irom || s_irom_size < 0x4000 || !s_ext || !s_arm || !s) {
		rbx_fail("IGS027A: ROM missing or out of memory");
		a3_exit();
		return -1;
	}
	// MAME's init_* (svg_basic_init, the decryption, svg_latch_init, the internal ROM code, the speedups)
	if (!strcmp(init, "theglad") || !strcmp(init, "theglada")) {
		pgm_theglad_decrypt(s_ext, s_ext_size);
		dummy_internal_arm_region_theglad(0);
		speedup(0x1000000c, 0x7c4, s->arm_ram2, 1);
		if (!strcmp(init, "theglada"))
			patch_jumptable_theglada(0x82078);
	} else if (!strcmp(init, "svg")) {
		pgm_svg_decrypt(s_ext, s_ext_size);
		dummy_internal_arm_region_theglad(1);
		speedup(0xb90, 0xb90, s_irom + 0x800, 0);
	} else if (!strcmp(init, "svgpcb")) {
		pgm_svgpcb_decrypt(s_ext, s_ext_size);
		dummy_internal_arm_region_theglad(0);
		speedup(0x9e0, 0x9e0, s_irom + 0x800, 0);
	} else if (!strcmp(init, "killbldp")) {
		pgm_killbldp_decrypt(s_ext, s_ext_size);
		speedup(0x1000000c, 0x7d8, s->arm_ram2, 1);
	} else if (!strcmp(init, "dmnfrnt")) {
		pgm_dfront_decrypt(s_ext, s_ext_size);
		dummy_internal_arm_region(0x4000);   // the internal ROM is bypassed
		speedup(0x18000444, 0x8000fea, s->arm_ram + 0x400, 1);
		s_main_speedup = 1;
		s->ram_sel = 1;
		// the internal ROM probably supplies the region: both banks (MAME)
		s->share[1][0x158] = 0x05;
		s->share[1][0x159] = 0x00;
		s->share[0][0x158] = 0x05;
		s->share[0][0x159] = 0x00;
	} else if (!strcmp(init, "happy6")) {
		if (init_happy6_gfx()) {
			rbx_fail("IGS027A: out of memory");
			a3_exit();
			return -1;
		}
		pgm_happy6_decrypt(s_ext, s_ext_size);
		dummy_internal_arm_region_theglad(0);
		speedup(0x1000000c, 0xa08, s->arm_ram2, 1);
	}
	s_clock = 20000000;
	const char *m = pgm_game->machine;
	if (strstr(m, "_22m")) s_clock = 22000000;
	else if (strstr(m, "_24m")) s_clock = 24000000;
	else if (strstr(m, "_33_8688m")) s_clock = 33868800;
	else if (strstr(m, "_33m")) s_clock = 33000000;

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
	s_arm->irq_ack = arm_irq_ack;
	uint32_t ext_map = s_ext_size > 0x800000 ? 0x800000 : s_ext_size & ~0x3ffu;
	arm7_map(s_arm, 0x00000000, 0x00003fff, s_irom, 0);
	if (ext_map)
		arm7_map(s_arm, 0x08000000, 0x08000000 + ext_map - 1, s_ext, 0);
	arm7_map(s_arm, 0x10000000, 0x100003ff, s->arm_ram2, 1);
	arm7_map(s_arm, 0x18000000, 0x1803ffff, s->arm_ram, 1);
	arm7_map(s_arm, 0x50000000, 0x500003ff, s->ram50, 1);
	map_share();
	if (s_sp_mem) {   // (ReBOX: as a pc hook in the core, the page kept mapped; arm7.h hook_pc)
		s_arm->hook_pc = s_sp_pc;
		s_arm->hook_eat = 500;
		s_sp_mem = NULL;
	}
	if (s_main_speedup)
		m68k_bus.rd[0x80a000 >> M68K_PAGE_BITS] = NULL;   // reads of that page reach a3_read16
	return 0;
}

static void a3_reset(void)
{
	// the internal ROMs are not fully dumped: the region word is patched in (machine_reset)
	static const struct { const char *set; int base; } t[] = {
		{ "theglad", 0x3316 }, { "theglad100", 0x3316 }, { "theglad101", 0x3316 }, { "theglad104", 0x3316 },
		{ "happy6", 0x3586 }, { "happy6101", 0x3586 }, { "happy6100hk", 0x3586 },
		{ "svgpcb", 0x3a8e }, { "svg", 0x3c3e }, { "svgtw", 0x3a8e }, { "svghk", 0x3a8e },
	};
	for (size_t i = 0; i < sizeof(t) / sizeof(t[0]); i++)
		if (!strcmp(pgm_game->set, t[i].set)) {
			const uint32_t hack = pgm_region_hack();
			if (hack != 0xff)
				w16(t[i].base, (uint16_t)hack);
		}
	// (MAME: the latches, the bank select and the RAM keep their contents through a reset)
	s->fiq_pulse = 0;
	arm7_reset(s_arm);
	arm7_jit_reset();   // (the reset may patch the internal ROM)
}

static int a3_state(void *ctx, int load)
{
	uint8_t regs[ARM7_STATE_SIZE];
	if (!load)
		memcpy(regs, s_arm, ARM7_STATE_SIZE);
	int rc = rbx_state_io(ctx, "arm7", regs, ARM7_STATE_SIZE);
	rc |= rbx_state_io(ctx, "arm3", s, sizeof(*s));
	if (rc)
		return -1;
	if (load) {
		memcpy(s_arm, regs, ARM7_STATE_SIZE);
		map_share();
	}
	return 0;
}

const pgm_prot pgm_prot_arm3 = { a3_init, a3_reset, a3_exit, a3_read16, a3_write16, a3_line, NULL, a3_state, 0x1fffff };
