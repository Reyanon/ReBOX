/*  PGM protection: the IGS025 state machine with the IGS012 (Dragon World 2), IGS022 (The Killing Blade, Dragon
    World 3 / EX) and IGS028 (Oriental Legend Special) devices, ported from MAME src/mame/igs/igs025.cpp, igs022.cpp,
    igs028.cpp, pgmprot_igs025_igs012.cpp, pgmprot_igs025_igs022.cpp and pgmprot_igs025_igs028.cpp (BSD-3-Clause,
    copyright-holders: David Haywood, ElSemi, iq_132).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "pgm.h"
#include "../machine/pgmcrypt.h"

#include <string.h>

#include "../machine/igs025_data.inc"

enum { V_DRGW2, V_KILLBLD, V_DW3, V_OLDS };
static int s_var;

// ---- IGS025 ----
static struct {
	uint32_t game_id, region;
	uint16_t hold, hilo, hilo_select, bs, cmd3;
	uint8_t swap, pad;
	int32_t cmd, reg, ptr;
} s_025;

static void ext_execute(void);

static uint8_t source_r(uint32_t region, uint8_t addr)
{
	switch (s_var) {
	case V_DRGW2: return region < 8 ? s_drgw2_source[region][addr] : 0;
	case V_KILLBLD: return region < 0x0c ? s_killbld_source[region][addr] : 0;
	case V_DW3: return region < 8 ? s_dw3_source[region][addr] : 0;
	default: return region < 8 ? s_olds_source[region][addr] : 0;
	}
}

static void calc_hold(int y, int z)
{
	const uint16_t old = s_025.hold;
	uint16_t h = (uint16_t)((old << 1) | (old >> 15));
	h ^= 0x2bad;
	h ^= (uint16_t)((z >> y) & 1);
	h ^= (uint16_t)((old >> 7) & 1);
	h ^= (uint16_t)(((~old >> 13) & 1) << 4);
	h ^= (uint16_t)(((old >> 3) & 1) << 11);
	h ^= (uint16_t)((s_025.hilo & ~0x0408) << 1);
	s_025.hold = h;
}

static void calc_hilo(void)
{
	s_025.hilo_select++;
	if (s_025.hilo_select > 0xeb)
		s_025.hilo_select = 0;
	const uint8_t source = source_r(s_025.region, (uint8_t)s_025.hilo_select);
	if (s_025.hilo_select & 1)
		s_025.hilo = (uint16_t)((s_025.hilo & 0x00ff) | (source << 8));
	else
		s_025.hilo = (uint16_t)((s_025.hilo & 0xff00) | source);
}

static uint8_t bitrev7(uint8_t v)   // bitswap<8>(v, 0,1,2,3,4,5,6,7)
{
	uint8_t r = 0;
	for (int i = 0; i < 8; i++)
		r |= (uint8_t)(((v >> i) & 1) << (7 - i));
	return r;
}

static uint16_t hold_swap(uint16_t h)   // bitswap<8>(hold, 5,2,9,7,10,13,12,15)
{
	static const int b[8] = { 5, 2, 9, 7, 10, 13, 12, 15 };
	uint16_t r = 0;
	for (int i = 0; i < 8; i++)
		r |= (uint16_t)(((h >> b[i]) & 1) << (7 - i));
	return r;
}

static uint16_t igs025_r(int offset)
{
	if (!offset)
		return 0;
	switch (s_025.cmd) {
	case 0x00: return bitrev7((uint8_t)((s_025.swap + 1) & 0x7f));
	case 0x01: return (uint16_t)(s_025.reg & 0x7f);
	case 0x02: return (uint16_t)(s_025.bs | 0x80);
	case 0x03: return s_025.cmd3;
	case 0x05:
		switch (s_025.ptr) {
		case 1: return (uint16_t)(0x3f00 | ((s_025.game_id >> 0) & 0xff));
		case 2: return (uint16_t)(0x3f00 | ((s_025.game_id >> 8) & 0xff));
		case 3: return (uint16_t)(0x3f00 | ((s_025.game_id >> 16) & 0xff));
		case 4: return (uint16_t)(0x3f00 | ((s_025.game_id >> 24) & 0xff));
		default: return (uint16_t)(0x3f00 | hold_swap(s_025.hold));
		}
	case 0x40:
		calc_hilo();
		return 0;
	}
	return 0;
}

static void killbld_025_w(int offset, uint16_t data)
{
	if (!offset) {
		s_025.cmd = data;
		return;
	}
	switch (s_025.cmd) {
	case 0x00: s_025.reg = data; break;
	case 0x01:
		if (data == 0x0002)
			ext_execute();
		break;
	case 0x02:
		if (data == 0x0001) {
			ext_execute();
			s_025.reg++;
		}
		break;
	case 0x03: s_025.swap = (uint8_t)data; break;
	case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26: case 0x27:
		s_025.ptr++;
		calc_hold(s_025.cmd & 0x0f, data & 0xff);
		break;
	}
}

static void olds_025_w(int offset, uint16_t data)
{
	if (!offset) {
		s_025.cmd = data;
		return;
	}
	switch (s_025.cmd) {
	case 0x00: s_025.reg = data; break;
	case 0x02: s_025.bs = (uint16_t)(((data & 0x03) << 6) | ((data & 0x04) << 3) | ((data & 0x08) << 1)); break;
	case 0x03:
		ext_execute();
		s_025.cmd3 = (uint16_t)(((data >> 4) + 1) & 0x3);
		break;
	case 0x04: s_025.ptr = data; break;
	case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26: case 0x27:
		s_025.ptr++;
		calc_hold(s_025.cmd & 0x0f, data & 0xff);
		break;
	}
}

static void drgw2_025_w(int offset, uint16_t data)
{
	if (!offset) {
		s_025.cmd = data;
		return;
	}
	switch (s_025.cmd) {
	case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26: case 0x27:
		s_025.ptr++;
		calc_hold(s_025.cmd & 0x0f, data & 0xff);
		break;
	}
}

static void igs025_reset(void)
{
	s_025.hold = s_025.hilo = s_025.hilo_select = 0;
	s_025.cmd = s_025.reg = s_025.ptr = 0;
	s_025.swap = 0;
	s_025.bs = s_025.cmd3 = 0;
}

// ---- shared protection RAM (IGS022 at 0x300000, IGS028 at 0x400000) ----
static uint16_t s_shared[0x2000];
static const uint8_t *s_prom;   // the device's ROM ("igs022" / "igs028")
static uint32_t s_prom_size;

static uint16_t prom16le(uint32_t a) { return a + 1 < s_prom_size ? (uint16_t)(s_prom[a] | s_prom[a + 1] << 8) : 0; }
static uint16_t prom16be(uint32_t a) { return a + 1 < s_prom_size ? (uint16_t)(s_prom[a] << 8 | s_prom[a + 1]) : 0; }

// ---- IGS022 ----
#define NUM_REGS 0x300
#define STACK_SIZE 0x100
static struct {
	uint32_t regs[NUM_REGS];
	uint32_t stack[STACK_SIZE];
	uint8_t sp;
} s_022;

static void d022_dma(uint16_t src, uint16_t dst, uint16_t size, uint16_t mode)
{
	const uint16_t param = mode >> 8;
	mode &= 7;
	switch (mode) {
	case 0: case 1: case 2: case 3: case 4:
		for (int x = 0; x < size; x++) {
			uint16_t dat = prom16le((uint32_t)(uint16_t)(src + x) << 1);
			const uint8_t taboff = (uint8_t)((x * 2) + (param & 0xff));
			uint16_t extraxor = prom16le(taboff);
			switch (mode) {
			case 1: dat = (uint16_t)(dat - extraxor); break;
			case 2: dat = (uint16_t)(dat + extraxor); break;
			case 3: dat ^= extraxor; break;
			case 4:
				extraxor = 0;
				if ((x & 0x003) == 0x000) extraxor |= 0x0049;
				if ((x & 0x003) == 0x001) extraxor |= 0x0047;
				if ((x & 0x003) == 0x002) extraxor |= 0x0053;
				if ((x & 0x003) == 0x003) extraxor |= 0x0020;
				if ((x & 0x300) == 0x000) extraxor |= 0x4900;
				if ((x & 0x300) == 0x100) extraxor |= 0x4700;
				if ((x & 0x300) == 0x200) extraxor |= 0x5300;
				if ((x & 0x300) == 0x300) extraxor |= 0x2000;
				dat = (uint16_t)(dat - extraxor);
				break;
			}
			s_shared[(dst + x) & 0x1fff] = dat;
		}
		break;
	case 5:
		for (int x = 0; x < size; x++)
			s_shared[(dst + x) & 0x1fff] = prom16be((uint32_t)(uint16_t)(src + x) << 1);
		break;
	case 6:
		for (int x = 0; x < size; x++) {
			uint16_t dat = prom16le((uint32_t)(uint16_t)(src + x) << 1);
			s_shared[(dst + x) & 0x1fff] = (uint16_t)(((dat & 0xf0f0) >> 4) | ((dat & 0x0f0f) << 4));
		}
		break;
	}
}

static void push_stack(uint32_t d)
{
	if (s_022.sp < STACK_SIZE - 1)
		++s_022.sp;
	s_022.stack[s_022.sp] = d;
}

static uint32_t pop_stack(void)
{
	const uint32_t d = s_022.stack[s_022.sp];
	if (s_022.sp > 0)
		--s_022.sp;
	return d;
}

static uint32_t d022_read_reg(uint16_t o) { return o < NUM_REGS ? s_022.regs[o] : o == 0x400 ? pop_stack() : 0; }
static void d022_write_reg(uint16_t o, uint32_t d)
{
	if (o < NUM_REGS)
		s_022.regs[o] = d;
	else if (o == 0x300)
		push_stack(d);
}

static void d022_reset(void)
{
	memset(&s_022, 0, sizeof(s_022));
	for (int i = 0; i < 0x2000; i++)
		s_shared[i] = 0xa55a;
	d022_dma((uint16_t)(prom16le(0x100) >> 1), prom16le(0x102), prom16le(0x104), prom16be(0x106));
	s_shared[0x2a2 / 2] = prom16le(0x114);
}

static void d022_command(void)
{
	const uint16_t cmd = s_shared[0x200 / 2];
	switch (cmd) {
	case 0x12:
		push_stack((uint32_t)(s_shared[0x288 / 2] << 16) + s_shared[0x28a / 2]);
		s_shared[0x202 / 2] = 0x23;
		break;
	case 0x2d: s_shared[0x202 / 2] = 0x3c; break;
	case 0x45: {
		const uint32_t d = pop_stack();
		s_shared[0x28c / 2] = (uint16_t)(d >> 16);
		s_shared[0x28e / 2] = (uint16_t)d;
		s_shared[0x202 / 2] = 0x56;
		break;
	}
	case 0x4f:
		d022_dma((uint16_t)(s_shared[0x290 / 2] >> 1), s_shared[0x292 / 2], s_shared[0x294 / 2], s_shared[0x296 / 2]);
		s_shared[0x202 / 2] = 0x5e;
		break;
	case 0x5a: s_shared[0x202 / 2] = 0x4b; break;
	case 0x6d: {
		const uint32_t p1 = (uint32_t)(s_shared[0x298 / 2] << 16) | s_shared[0x29a / 2];
		const uint32_t p2 = (uint32_t)(s_shared[0x29c / 2] << 16) | s_shared[0x29e / 2];
		const uint16_t src1 = (uint16_t)(p1 >> 16), src2 = (uint16_t)p1, dst = (uint16_t)(p2 >> 16);
		switch (p2 & 0xffff) {
		case 0x0: d022_write_reg(dst, d022_read_reg(src1) + d022_read_reg(src2)); break;
		case 0x1: { const uint32_t a = d022_read_reg(src1), b = d022_read_reg(src2); d022_write_reg(dst, a - b); break; }
		case 0x6: { const uint32_t a = d022_read_reg(src1), b = d022_read_reg(src2); d022_write_reg(dst, b - a); break; }
		case 0x9: d022_write_reg(dst, p1); break;
		case 0xa: {
			const uint32_t d = s_022.regs[src1 % NUM_REGS];
			s_shared[0x29c / 2] = (uint16_t)(d >> 16);
			s_shared[0x29e / 2] = (uint16_t)d;
			break;
		}
		}
		s_shared[0x202 / 2] = 0x7c;
		break;
	}
	}
}

// ---- IGS028 ----
static uint32_t d028_addr(uint16_t addr)
{
	switch (addr & 0xff) {
	case 0x0: case 0x5: case 0xa: return 0x402a00 + ((uint32_t)(addr >> 8) << 2);
	case 0x2: case 0x8: return 0x402e00 + ((uint32_t)(addr >> 8) << 2);
	case 0x1: return 0x40307e;
	case 0x3: return 0x403090;
	case 0x4: return 0x40309a;
	case 0x6: return 0x4030a4;
	case 0x7: return 0x403000;
	case 0x9: return 0x40306e;
	case 0xb: return 0x403044;
	}
	return 0;
}

static uint32_t d028_read_reg(uint16_t addr)
{
	const uint32_t a = ((d028_addr(addr) - 0x400000) / 2) & 0x1fff;
	return (uint32_t)s_shared[a] << 16 | s_shared[(a + 1) & 0x1fff];
}

static void d028_write_reg(uint16_t addr, uint32_t v)
{
	const uint32_t a = ((d028_addr(addr) - 0x400000) / 2) & 0x1fff;
	s_shared[a] = (uint16_t)(v >> 16);
	s_shared[(a + 1) & 0x1fff] = (uint16_t)v;
}

static void d028_dma(uint16_t src, uint16_t dst, uint16_t size, uint16_t mode)
{
	const uint16_t param = mode >> 8;
	mode &= 0x0f;
	if (mode & 8)
		return;
	for (uint32_t x = 0; x < size; x++) {
		uint16_t d = prom16le((uint32_t)(uint16_t)(src + x) << 1);
		const uint32_t taboff = ((x * 2) + (param & 0xff)) & 0xff;
		const uint16_t extraxor = prom16le(0x100 + taboff);
		switch (mode & 7) {
		case 0: d = (uint16_t)(d - extraxor); break;
		case 1: d = (uint16_t)(((d & 0xf0f0) >> 4) | ((d & 0x0f0f) << 4)); break;
		case 2: d ^= extraxor; break;
		case 5: d = (uint16_t)((d >> 8) | (d << 8)); break;
		case 6: d = (uint16_t)(d + extraxor); break;
		default: d = 0x4e75; break;   // (MAME's hack)
		}
		s_shared[(dst + x) & 0x1fff] = d;
	}
}

static void d028_command(void)
{
	const uint16_t cmd = s_shared[0x3026 / 2];
	if (cmd == 0x12)
		d028_dma((uint16_t)(s_shared[0x306a / 2] >> 1), s_shared[0x3084 / 2] & 0x1fff, s_shared[0x30a2 / 2] & 0x1fff,
		         s_shared[0x303e / 2]);
	else if (cmd == 0x64) {
		const uint16_t p1 = s_shared[0x3050 / 2], p2 = s_shared[0x3082 / 2], p3 = s_shared[0x3054 / 2], p4 = s_shared[0x3088 / 2];
		if (p2 == 0x02)
			d028_write_reg(p1, d028_read_reg(p1) + 0x10000);
		switch (p4) {
		case 0xd: d028_write_reg(p1, d028_read_reg(p3)); break;
		case 0x0: d028_write_reg(p3, d028_read_reg(p2) ^ d028_read_reg(p1)); break;
		case 0xe: d028_write_reg(p3, d028_read_reg(p3) + 0x10000); break;
		case 0x2: d028_write_reg(p1, d028_read_reg(p2) + d028_read_reg(p3)); break;
		case 0x6: d028_write_reg(p3, d028_read_reg(p2) & d028_read_reg(p1)); break;
		case 0x1: d028_write_reg(p2, d028_read_reg(p1) + 0x10000); break;
		case 0x7: d028_write_reg(p3, d028_read_reg(p1)); break;
		}
	}
}

static void d028_reset(void)
{
	s_shared[0x1000 / 2] = 0x4749;
	s_shared[0x1002 / 2] = 0x2e53;
	s_shared[0x1004 / 2] = 0x3832;
	s_shared[0x3064 / 2] = 0xb315;
}

static void ext_execute(void)
{
	if (s_var == V_KILLBLD || s_var == V_DW3)
		d022_command();
	else if (s_var == V_OLDS)
		d028_command();
}

// ---- hookups ----
static uint32_t s_base;   // the IGS025's address

static int prot_read16(uint32_t a, uint16_t *v)
{
	if (a >= s_base && a <= s_base + 3) {
		*v = igs025_r((int)((a - s_base) >> 1));
		return 1;
	}
	if (s_var == V_DRGW2 && a >= 0xd00000 && a <= 0xd00fff) {
		*v = 0;
		return 1;
	}
	return 0;
}

static int prot_write16(uint32_t a, uint16_t v, uint16_t mask)
{
	(void)mask;
	if (a >= s_base && a <= s_base + 3) {
		const int o = (int)((a - s_base) >> 1);
		if (s_var == V_DRGW2)
			drgw2_025_w(o, v);
		else if (s_var == V_OLDS)
			olds_025_w(o, v);
		else
			killbld_025_w(o, v);
		return 1;
	}
	return s_var == V_DRGW2 && a >= 0xd00000 && a <= 0xd00fff;
}

static void patch(uint32_t a, uint16_t v)
{
	if (a + 1 < pgm_rom_size)
		*(uint16_t *)(pgm_rom + a) = v;
}

static void drgw2_region(int region, uint32_t p0, uint32_t p1, uint32_t p2)
{
	s_025.region = (uint32_t)region;
	s_025.game_id = (uint32_t)(region | (region << 8) | (region << 16) | (region << 24));
	if (p0) {
		patch(p0, 0x4e93);
		patch(p1, 0x4e93);
		patch(p2, 0x4e93);
	}
}

static int init_drgw2(const char *init)
{
	s_var = V_DRGW2;
	memset(&s_025, 0, sizeof(s_025));
	s_base = 0xd80000;
	pgm_drgw2_decrypt(pgm_rom, pgm_rom_size);
	if (!strcmp(init, "drgw2")) drgw2_region(6, 0x131098, 0x13113e, 0x1311ce);
	else if (!strcmp(init, "dw2v100x")) drgw2_region(6, 0x131084, 0x13112a, 0x1311ba);
	else if (!strcmp(init, "drgw2c")) drgw2_region(5, 0x1303bc, 0x130462, 0x1304f2);
	else if (!strcmp(init, "drgw2c101")) drgw2_region(5, 0x1306e4, 0x13078a, 0x13081a);
	else if (!strcmp(init, "drgw2j")) drgw2_region(1, 0x1302c0, 0x130366, 0x1303f6);
	else if (!strcmp(init, "drgw2hk")) drgw2_region(1, 0x12f520, 0x12f5c6, 0x12f656);
	return 0;
}

static int init_022(const char *init)
{
	memset(&s_025, 0, sizeof(s_025));
	s_prom = pgm_region("igs022", &s_prom_size);
	if (!s_prom) {
		rbx_fail("igs022 ROM missing");
		return -1;
	}
	if (!strcmp(init, "killbld")) {
		s_var = V_KILLBLD;
		s_base = 0xd40000;
		pgm_killbld_decrypt(pgm_rom, pgm_rom_size);
	} else {
		s_var = V_DW3;
		s_base = 0xda5610;
		pgm_dw3_decrypt(pgm_rom, pgm_rom_size);
	}
	pgm_map(0x300000, 0x303fff, s_shared, sizeof(s_shared), 1);
	return 0;
}

static int init_028(const char *init)
{
	(void)init;
	memset(&s_025, 0, sizeof(s_025));
	s_var = V_OLDS;
	s_base = 0xdcb400;
	s_prom = pgm_region("igs028", &s_prom_size);
	if (!s_prom) {
		rbx_fail("igs028 ROM missing");
		return -1;
	}
	pgm_map(0x400000, 0x403fff, s_shared, sizeof(s_shared), 1);
	return 0;
}

static void prot_reset(void)
{
	igs025_reset();
	const uint32_t region = pgm_region_port() & 0xff;
	switch (s_var) {
	case V_KILLBLD:
		d022_reset();
		s_025.region = region - 0x16;
		s_025.game_id = 0x89911400 | region;
		break;
	case V_DW3:
		d022_reset();
		s_025.region = region;
		s_025.game_id = 0x00060000 | region;
		break;
	case V_OLDS:
		memset(s_shared, 0, sizeof(s_shared));
		d028_reset();
		s_025.region = region;
		s_025.game_id = 0x00900000 | region;
		break;
	}
}

static int prot_state(void *ctx, int load)
{
	(void)load;
	int rc = rbx_state_io(ctx, "igs025", &s_025, sizeof(s_025));
	if (s_var != V_DRGW2)
		rc |= rbx_state_io(ctx, "pshared", s_shared, sizeof(s_shared));
	if (s_var == V_KILLBLD || s_var == V_DW3)
		rc |= rbx_state_io(ctx, "igs022", &s_022, sizeof(s_022));
	return rc;
}

const pgm_prot pgm_prot_025_012 = { init_drgw2, prot_reset, NULL, prot_read16, prot_write16, NULL, NULL, prot_state, 0x1fffff };
const pgm_prot pgm_prot_025_022 = { init_022, prot_reset, NULL, prot_read16, prot_write16, NULL, NULL, prot_state, 0x2fffff };
const pgm_prot pgm_prot_025_028 = { init_028, prot_reset, NULL, prot_read16, prot_write16, NULL, NULL, prot_state, 0x3fffff };
