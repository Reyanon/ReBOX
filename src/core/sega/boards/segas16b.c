/*  Sega System 16B. Ported from MAME src/mame/sega/segas16b.cpp, segas16b_v.cpp, 315_5195.cpp, segaic16.cpp,
    sega16sp.cpp (BSD-3-Clause, copyright-holders: Aaron Giles, Charles MacDonald, Nicola Salmoria, Mirko Buffoni,
    Thierry Lescot, David Haywood).
    A 68000 at 10 MHz behind the 315-5195 memory mapper (eight regions placed by the CPU: the ROMs, the work RAM, the
    sprite RAM, the tile and text RAM, the palette, the I/O; the mapper's own registers everywhere else), the 16B
    tilemaps (16 pages of 64x32 8x8 tiles, two layers of 2x2 pages with row and column scroll and alternates, the text
    layer), the 16B sprites (zoomed lines of 4-bit pixels), a palette of 2048 colours with shadow / hilight. Sound: a
    Z80 at 5 MHz, a YM2151, a uPD7759. 400x262 at 6.29 MHz (60.05 Hz), 320x224 shown.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../rbx_core.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/mcs51/mcs51.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/upd7759.h"
#include "../machine/fd1094.h"
#include "../machine/fd1089.h"
#include "../machine/mc8123.h"

extern void (*rbx_m68k_cmpild_hook)(int reg, unsigned int imm);
extern void (*rbx_m68k_rte_hook)(void);
extern void (*rbx_m68k_irqack_hook)(int level);

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 10000000
#define Z80_CLOCK 5000000
#define YM_CLOCK 4000000
#define UPD_CLOCK 640000
#define FRAME_MHZ 60055          // 25.1748 MHz / 4 / (400 x 262)
#define LINES 262
#define VIS_W 320
#define VIS_H 224

enum { RB_5358_SMALL, RB_5358, RB_5521, RB_5704, RB_5797, RB_KOREAN };

// ---- ROMs ----
static uint8_t *s_rom, *s_tiles, *s_spr, *s_zrom;
static uint32_t s_rom_size, s_tiles_size, s_spr_size, s_zrom_size;
static int s_romboard;
static int s_fd1094_on;
static fd1094_t s_fd1094;
static uint8_t *s_zop;   // (MC-8123: the Z80 opcodes space, 64 KB: the ROM decrypted, the bank copied)
static uint16_t *s_fd1089_op;   // (FD1089A / B: the opcodes decrypted at init)
static int s_port_service, s_port_p1, s_port_unused, s_port_p2, s_port_dsw1, s_port_dsw2;
// the custom I/O of an init (MAME's m_custom_io_r): the trackballs (dunkshot, sdi, exctleag), Passing Shot's 4 players
enum { IO_STD, IO_DUNKSHOT, IO_SDI, IO_PASSSHTJ, IO_SJRYUKO, IO_HWCHAMP };
static int s_io, s_port_p3, s_port_p4;
static uint8_t s_tb[8];        // ANALOGX1, ANALOGY1 .. ANALOGY4 (the trackballs' counters; saved)
static int s_port_mj[6];      // (sjryuko: the mahjong rows MJ0-5, selected by the output latch's bit 2)
static uint8_t s_tb_rev;
// hwchamp: MONITOR, RIGHT, LEFT, RIGHT_LIMIT, LEFT_LIMIT (the handles, moved by the controls a frame) and the
// MSM6253's shift register (saved)
static uint8_t s_hw[6];       // (PORT_REVERSE by port)
static uint16_t *s_wram16c;   // (System 16C: 256 KB of work RAM in place of the 16 KB; saved)
static int s_alt_tiles;        // TILEMAP_16B_ALT (dunkshot, sjryuko, timescan)

// ---- the state (saved) ----
static struct segas16b_state {   // (on the heap while the game runs, ADR-040)
	uint16_t workram[0x2000];   // 16 KB
	uint16_t spriteram[0x400];
	uint16_t paletteram[0x800];
	uint16_t tileram[0x8000];
	uint16_t textram[0x800];
	uint8_t regs[0x20];         // the 315-5195's
	uint8_t to_sound, from_sound, mj_input_num, mj_last_val;   // (mj_: sjryuko)
	int32_t pbf;                // (the sound CPU's IRQ: a command waiting)
	uint8_t display_enable, flip, tbank[2];
	uint16_t latched_page[4], latched_yscroll[4], latched_xscroll[4];
	uint32_t zbank;             // (the sound bank's offset in the sound region)
	uint64_t cpu_frac, z80_frac, ym_frac, upd_frac, mcu_frac;
	uint16_t mult[2], cmp[2][16];   // the 315-5248 multiplier, the 315-5250 compare timers
	uint8_t cmp_bit[2], reset_held, pad2;
	int32_t spin;                   // (the 68000's cycles to skip: spin_68k_w)
	uint32_t frame;
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)

static z80 s_z80;
static uint8_t s_zram[0x800];
static const uint8_t *s_zmap[256];
static upd7759_t s_upd;
static int32_t *s_upd_buf;
static uint64_t s_upd_frac;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;

// ---- the 315-5195 memory mapper ----
enum { E_MEM, E_IO, E_BANK5704, E_MATH5797, E_RGN2 };
typedef struct {
	uint32_t start, end, mirror, wbase, wend;   // (wbase .. wend: the region's window)
	int kind, rd, wr;                           // rd / wr: 1 handled, 0 falls through, -1 (writes) dropped
	uint8_t *mem;
	uint32_t memsize;
} mapent_t;
static mapent_t s_map[24];
static int s_nmap, s_curregion;

static void map_add(uint32_t offset, uint32_t length, uint32_t mirror, int kind, uint8_t *mem, uint32_t memsize, int rd, int wr)
{
	static const uint32_t region_size_map[4] = { 0x00ffff, 0x01ffff, 0x07ffff, 0x1fffff };
	const uint32_t size_mask = region_size_map[b.regs[0x10 + 2 * s_curregion] & 3];
	const uint32_t base = ((uint32_t)b.regs[0x11 + 2 * s_curregion] << 16) & ~size_mask;
	if (s_nmap >= (int)(sizeof(s_map) / sizeof(s_map[0])))
		return;
	mapent_t *e = &s_map[s_nmap++];
	e->mirror = mirror & size_mask;
	e->start = base + (offset & size_mask);
	e->end = e->start + (length - 1 < size_mask ? length - 1 : size_mask);
	e->wbase = base;
	e->wend = base + size_mask;
	e->kind = kind;
	e->mem = mem;
	e->memsize = memsize;
	e->rd = rd;
	e->wr = wr;
}
static void map_rom(uint32_t offset, uint32_t length, uint32_t mirror, uint32_t rgnoffset)
{
	if (rgnoffset >= s_rom_size)
		return;
	uint32_t len = length;
	if (rgnoffset + len > s_rom_size)
		len = s_rom_size - rgnoffset;
	map_add(offset, len, mirror, E_MEM, s_rom + rgnoffset, len, 1, -1);
}
// segas16b_state::memory_mapper
static void memory_mapper(int index)
{
	switch (index) {
	case 7: map_add(0x00000, 0x04000, 0xffc000, E_IO, NULL, 0, 1, 1); break;
	case 6: map_add(0x00000, 0x01000, 0xfff000, E_MEM, (uint8_t *)b.paletteram, 0x1000, 1, 1); break;
	case 5:
		map_add(0x00000, 0x10000, 0xfe0000, E_MEM, (uint8_t *)b.tileram, 0x10000, 1, 1);
		map_add(0x10000, 0x01000, 0xfef000, E_MEM, (uint8_t *)b.textram, 0x1000, 1, 1);
		break;
	case 4: map_add(0x00000, 0x00800, 0xfff800, E_MEM, (uint8_t *)b.spriteram, 0x800, 1, 1); break;
	case 3: {   // 16k or 256k of work RAM
		uint8_t *const w = s_wram16c ? (uint8_t *)s_wram16c : (uint8_t *)b.workram;
		const uint32_t n = s_wram16c ? 0x40000 : (uint32_t)sizeof(b.workram);
		map_add(0x00000, n, ~(n - 1), E_MEM, w, n, 1, 1);
		break;
	}
	case 2:
		switch (s_romboard) {
		case RB_5358_SMALL: map_rom(0x00000, 0x20000, 0xfe0000, 0x20000); break;
		case RB_5358: map_rom(0x00000, 0x20000, 0xfe0000, 0x40000); break;
		case RB_5521: case RB_5704: map_add(0x00000, 0x10000, 0xff0000, E_BANK5704, NULL, 0, 0, 1); break;
		case RB_5797: map_add(0x00000, 0x10000, 0xff0000, E_RGN2, NULL, 0, 1, 1); break;   // (a second compare timer)
		default: break;
		}
		break;
	case 1:
		switch (s_romboard) {
		case RB_5358_SMALL: map_rom(0x00000, 0x20000, 0xfe0000, 0x10000); break;
		case RB_5358: map_rom(0x00000, 0x20000, 0xfe0000, 0x20000); break;
		case RB_5521: case RB_5704: map_rom(0x00000, 0x40000, 0xfc0000, 0x40000); break;
		case RB_5797: map_add(0x00000, 0x04000, 0xffc000, E_MATH5797, NULL, 0, 1, 1); break;
		default: break;
		}
		break;
	case 0:
		switch (s_romboard) {
		case RB_5358_SMALL: case RB_5358: map_rom(0x00000, 0x20000, 0xfe0000, 0x00000); break;
		case RB_5521: case RB_5704: case RB_KOREAN: map_rom(0x00000, 0x40000, 0xfc0000, 0x00000); break;
		case RB_5797: map_rom(0x00000, 0x80000, 0xf80000, 0x00000); break;
		}
		break;
	}
}

// the page table: a page goes straight to memory when one entry covers all of it with no later entry in its window
static void update_mapping(void)
{
	s_nmap = 0;
	for (int index = 7; index >= 0; index--) {
		s_curregion = index;
		memory_mapper(index);
	}
	const uint32_t ps = 1u << M68K_PAGE_BITS;
	for (uint32_t p = 0; p < M68K_PAGES; p++) {
		const uint32_t a0 = p * ps, a1 = a0 + ps - 1;
		m68k_bus_cur->rd[p] = m68k_bus_cur->wr[p] = NULL;
		for (int i = s_nmap - 1; i >= 0; i--) {
			const mapent_t *e = &s_map[i];
			if (a1 < e->wbase || a0 > e->wend)
				continue;
			// (the top entry of this page: direct only when it covers the page without a mirror below a page)
			if (e->kind == E_MEM && !(e->mirror & (ps - 1))) {
				const uint32_t ae = a0 & ~e->mirror;
				if (ae >= e->start && ae + ps - 1 <= e->end && ae + ps - 1 - e->start < e->memsize) {
					m68k_bus_cur->rd[p] = e->mem + (ae - e->start);
					if (e->wr > 0)
						m68k_bus_cur->wr[p] = e->mem + (ae - e->start);
				}
			}
			break;
		}
	}
}

static uint16_t open_bus(void);
static void fd_update(void);
static uint16_t io_r(uint32_t offset);
static void io_w(uint32_t offset, uint16_t data, uint16_t mask);
static void mapper_w(int offset, uint8_t data);
static uint8_t mapper_r(int offset);

// the 315-5248 multiplier and the 315-5250 compare timer (segaic16_m.cpp; their interrupt lines not wired on 16B)
static uint16_t mult_r(uint32_t offset)
{
	switch (offset & 3) {
	case 0: return b.mult[0];
	case 1: return b.mult[1];
	case 2: return (uint16_t)(((int32_t)(int16_t)b.mult[0] * (int16_t)b.mult[1]) >> 16);
	default: return (uint16_t)(((int32_t)(int16_t)b.mult[0] * (int16_t)b.mult[1]) & 0xffff);
	}
}
static void cmp_execute(int t, int update_history)
{
	uint16_t *r = b.cmp[t];
	const int16_t bound1 = (int16_t)r[0], bound2 = (int16_t)r[1], value = (int16_t)r[2];
	const int16_t min = bound1 < bound2 ? bound1 : bound2, max = bound1 > bound2 ? bound1 : bound2;
	if (value < min) {
		r[7] = (uint16_t)min;
		r[3] = 0x8000;
	} else if (value > max) {
		r[7] = (uint16_t)max;
		r[3] = 0x4000;
	} else {
		r[7] = (uint16_t)value;
		r[3] = 0x0000;
	}
	if (update_history)
		r[4] |= (uint16_t)((r[3] == 0) << b.cmp_bit[t]++);
}
static uint16_t cmp_r(int t, uint32_t offset)
{
	const uint16_t *r = b.cmp[t];
	switch (offset & 15) {
	case 0x0: return r[0];
	case 0x1: return r[1];
	case 0x2: return r[2];
	case 0x3: return r[3];
	case 0x4: return r[4];
	case 0x5: return r[1];
	case 0x6: return r[2];
	case 0x7: return r[7];
	}
	return 0xffff;
}
static void cmp_w(int t, uint32_t offset, uint16_t data, uint16_t mask)
{
	uint16_t *r = b.cmp[t];
#define CMB(i) r[i] = (uint16_t)((data & mask) | (r[i] & ~mask))
	switch (offset & 15) {
	case 0x0: CMB(0); cmp_execute(t, 0); break;
	case 0x1: CMB(1); cmp_execute(t, 0); break;
	case 0x2: CMB(2); cmp_execute(t, 1); break;
	case 0x4: r[4] = 0; b.cmp_bit[t] = 0; break;
	case 0x6: CMB(2); cmp_execute(t, 0); break;
	case 0x8: case 0xc: CMB(8); break;
	case 0xa: case 0xe: CMB(10); break;
	case 0xb: case 0xf: r[11] = data & 0xff; break;
	}
#undef CMB
}

static uint16_t main_r16(uint32_t a)
{
	a &= 0xfffffe;
	for (int i = s_nmap - 1; i >= 0; i--) {
		const mapent_t *e = &s_map[i];
		const uint32_t ae = a & ~e->mirror;
		if (ae < e->start || ae > e->end || e->rd <= 0)
			continue;
		const uint32_t o = ae - e->start;
		switch (e->kind) {
		case E_MEM: return o + 1 < e->memsize ? *(const uint16_t *)(e->mem + o) : 0xffff;
		case E_IO: return io_r(o >> 1);
		case E_RGN2: return cmp_r(1, o >> 1);
		case E_MATH5797: {   // rom_5797_bank_math_r
			const uint32_t off = (o >> 1) & 0x1fff;
			switch (off & (0x3000 / 2)) {
			case 0x0000 / 2: return mult_r(off);
			case 0x1000 / 2: return cmp_r(0, off);
			}
			return open_bus();
		}
		}
	}
	return (uint16_t)(0xff00 | mapper_r((int)((a >> 1) & 0x1f)));
}
static uint8_t main_r8(uint32_t a)
{
	const uint16_t w = main_r16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}
static void main_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xfffffe;
	for (int i = s_nmap - 1; i >= 0; i--) {
		const mapent_t *e = &s_map[i];
		const uint32_t ae = a & ~e->mirror;
		if (ae < e->start || ae > e->end || e->wr == 0)
			continue;
		if (e->wr < 0)
			return;
		const uint32_t o = ae - e->start;
		switch (e->kind) {
		case E_MEM:
			if (o + 1 < e->memsize) {
				uint16_t *p = (uint16_t *)(e->mem + o);
				*p = (uint16_t)((v & mask) | (*p & ~mask));
			}
			return;
		case E_IO: io_w(o >> 1, v, mask); return;
		case E_BANK5704:   // rom_5704_bank_w: the tile banks
			if (mask & 0xff)
				b.tbank[(o >> 1) & 1] = v & 7;
			return;
		case E_RGN2: cmp_w(1, o >> 1, v, mask); return;
		case E_MATH5797: {   // rom_5797_bank_math_w
			const uint32_t off = (o >> 1) & 0x1fff;
			switch (off & (0x3000 / 2)) {
			case 0x0000 / 2: b.mult[off & 1] = (uint16_t)((v & mask) | (b.mult[off & 1] & ~mask)); break;
			case 0x1000 / 2: cmp_w(0, off, v, mask); break;
			case 0x2000 / 2:
				if (mask & 0xff)
					b.tbank[off & 1] = v & 7;
				break;
			}
			return;
		}
		}
	}
	if (mask & 0xff)
		mapper_w((int)((a >> 1) & 0x1f), (uint8_t)v);
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v)
{
	main_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00);
}

static int s_open_bus_recurse;
static uint16_t open_bus(void)   // the word at the PC
{
	if (s_open_bus_recurse)
		return 0xffff;
	s_open_bus_recurse = 1;
	const uint16_t r = main_r16(m68k_get_reg(NULL, M68K_REG_PC));
	s_open_bus_recurse = 0;
	return r;
}

static void mapper_w(int offset, uint8_t data)
{
	offset &= 0x1f;
	const uint8_t oldval = b.regs[offset];
	b.regs[offset] = data;
	switch (offset) {
	case 0x02:   // the 68000's reset line (the MCU)
		if ((oldval ^ data) & 3) {
			const int held = (data & 3) == 3;
			if (held && !b.reset_held) {
				if (s_fd1094_on) {   // (the FD1094 is the CPU: reset with it)
					fd1094_change_state(&s_fd1094, FD1094_STATE_RESET);
					fd_update();
				}
				m68k_pulse_reset();
			}
			b.reset_held = (uint8_t)held;
		}
		break;
	case 0x03:   // to the sound CPU (its IRQ)
		b.to_sound = data;
		b.pbf = 1;
		break;
	case 0x04:   // an IRQ to the 68000 (the MCU)
		if ((data & 7) != 7)
			m68k_bus_hold_irq(~data & 7);
		break;
	case 0x05:
		if (data == 0x01) {
			const uint32_t addr = ((uint32_t)b.regs[0x0a] << 17) | ((uint32_t)b.regs[0x0b] << 9) | ((uint32_t)b.regs[0x0c] << 1);
			main_w16(addr, (uint16_t)(b.regs[0x00] << 8 | b.regs[0x01]));
		} else if (data == 0x02) {
			const uint32_t addr = ((uint32_t)b.regs[0x07] << 17) | ((uint32_t)b.regs[0x08] << 9) | ((uint32_t)b.regs[0x09] << 1);
			const uint16_t result = main_r16(addr);
			b.regs[0x00] = (uint8_t)(result >> 8);
			b.regs[0x01] = (uint8_t)result;
		}
		break;
	default:
		if (offset >= 0x10 && oldval != data)
			update_mapping();
		break;
	}
}
static uint8_t mapper_r(int offset)
{
	offset &= 0x1f;
	switch (offset) {
	case 0x00: case 0x01: return b.regs[offset];
	case 0x02: return (b.regs[0x02] & 3) == 3 ? 0x00 : 0x0f;
	case 0x03: return b.from_sound;
	}
	return (uint8_t)open_bus();
}

// standard_io_r / standard_io_w
static uint16_t io_r(uint32_t offset)
{
	offset &= 0x1fff;
	if ((offset & (0x3000 / 2)) == 0x3000 / 2)
		switch (s_io) {
		case IO_DUNKSHOT:
			return (uint16_t)(((uint32_t)s_tb[(offset / 2) & 7] << 4) >> (8 * (offset & 1)));
		case IO_SDI:
			return s_tb[(offset / 2) & 3];
		case IO_HWCHAMP:
			switch (offset & (0x30 / 2)) {
			case 0x20 / 2: {   // msm6253 d0_r (the open bus high, a byte)
				const uint16_t bit = (s_hw[5] >> 7) & 1;
				s_hw[5] = (uint8_t)(s_hw[5] << 1);
				return (uint16_t)(bit | 0xfe);
			}
			case 0x30 / 2: {   // the handles' limits (MAME's arbitrary thresholds)
				const uint8_t left = s_hw[4], right = s_hw[3];
				uint16_t r = 0xf0;
				if (left < 0x40) r |= 1 << 3;
				if (left > 0xc0) r |= 1 << 2;
				if (right < 0x40) r |= 1 << 1;
				if (right > 0xc0) r |= 1 << 0;
				return r;
			}
			}
			break;
		case IO_PASSSHTJ: {
			const int ports[4] = { s_port_p1, s_port_p2, s_port_p3, s_port_p4 };
			const int pt = ports[offset & 3];
			return pt >= 0 ? (uint16_t)rbx_port_read(pt) : 0xffff;
		}
		}
	if (s_io == IO_SJRYUKO && (offset & (0x3000 / 2)) == 0x1000 / 2 && (offset & 3) != 0 && (offset & 3) != 3) {
		const int pt = s_port_mj[b.mj_input_num];
		const uint16_t v = pt >= 0 ? (uint16_t)(rbx_port_read(pt) & 0xff) : 0xff;
		if ((offset & 3) == 1)
			return v != 0xff ? (uint16_t)(0xff & ~(1 << b.mj_input_num)) : 0xff;
		return v;
	}
	switch (offset & (0x3000 / 2)) {
	case 0x1000 / 2: {
		const int ports[4] = { s_port_service, s_port_p1, s_port_unused, s_port_p2 };
		const int p = ports[offset & 3];
		return p >= 0 ? (uint16_t)rbx_port_read(p) : 0xffff;
	}
	case 0x2000 / 2: {
		const int p = (offset & 1) ? s_port_dsw1 : s_port_dsw2;
		return p >= 0 ? (uint16_t)rbx_port_read(p) : 0xffff;
	}
	}
	return open_bus();
}
static void io_w(uint32_t offset, uint16_t data, uint16_t mask)
{
	(void)mask;
	offset &= 0x1fff;
	if ((offset & (0x3000 / 2)) == 0x3000 / 2 && s_io == IO_HWCHAMP && (offset & (0x30 / 2)) == 0x20 / 2) {
		static const uint8_t ch[4] = { 0, 1, 2, 0 };   // msm6253 address_w: MONITOR, RIGHT, LEFT (channel 3 none)
		s_hw[5] = (offset & 3) == 3 ? 0xff : s_hw[ch[offset & 3]];
	}
	if ((offset & (0x3000 / 2)) == 0 && s_io == IO_SJRYUKO) {
		if (((b.mj_last_val ^ data) & 4) && (data & 4))
			b.mj_input_num = (uint8_t)((b.mj_input_num + 1) % 6);
		b.mj_last_val = (uint8_t)data;
	}
	if ((offset & (0x3000 / 2)) == 0) {
		b.flip = (data >> 6) & 1;
		b.display_enable = (data >> 5) & 1;
	}
}

// ---- the Z80 (sound_map, sound_portmap) ----
static void z_bank(void)
{
	const uint32_t size = s_zrom_size > 0x10000 ? s_zrom_size - 0x10000 : 0;
	for (int p = 0x80; p < 0xe0; p++) {
		const uint32_t o = size ? 0x10000 + rbx_wrap((b.zbank + (uint32_t)(p - 0x80) * 256), size) : 0;
		s_zmap[p] = size && o + 256 <= s_zrom_size ? s_zrom + o : NULL;
		if (s_zop) {
			if (s_zmap[p])
				memcpy(s_zop + p * 256, s_zmap[p], 256);
			else
				memset(s_zop + p * 256, 0xff, 256);
		}
	}
}
static uint8_t z_read(void *u, uint16_t a)
{
	(void)u;
	const uint8_t *p = s_zmap[a >> 8];
	if (p)
		return p[a & 0xff];
	if (a >= 0xf800)
		return s_zram[a - 0xf800];
	if (a == 0xe800) {
		b.pbf = 0;
		return b.to_sound;
	}
	return 0xff;
}
static void z_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xf800)
		s_zram[a - 0xf800] = v;
}
static void upd7759_control_w(uint8_t data)
{
	const int size = (int)s_zrom_size - 0x10000;
	if (size <= 0)
		return;
	// (this order: /MD low with /RESET low starts no sample)
	upd7759_md_w(&s_upd, !((data >> 7) & 1));
	upd7759_reset_w(&s_upd, (data >> 6) & 1);
	int bankoffs = 0;
	switch (s_romboard) {
	case RB_5358: case RB_5358_SMALL:
		if (!(data & 0x04)) bankoffs = 0x00000;
		if (!(data & 0x08)) bankoffs = 0x10000;
		if (!(data & 0x10)) bankoffs = 0x20000;
		if (!(data & 0x20)) bankoffs = 0x30000;
		bankoffs += (data & 0x03) * 0x4000;
		break;
	case RB_5521: case RB_5704:
		bankoffs = ((data & 0x08) >> 3) * 0x20000;
		bankoffs += (data & 0x07) * 0x4000;
		break;
	case RB_5797:
		bankoffs = ((data & 0x08) >> 3) * 0x40000;
		bankoffs += ((data & 0x10) >> 4) * 0x20000;
		bankoffs += (data & 0x07) * 0x04000;
		break;
	}
	b.zbank = (uint32_t)(rbx_wrap(bankoffs, size));
	z_bank();
}
static uint8_t z_in(z80 *z, uint8_t p)
{
	(void)z;
	switch (p & 0xc0) {
	case 0x00: return rbx_ym_read(0, p & 1);
	case 0x80: return (uint8_t)(upd7759_busy_r(&s_upd) << 7);
	case 0xc0:
		b.pbf = 0;
		return b.to_sound;
	}
	return 0xff;
}
static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	switch (p & 0xc0) {
	case 0x00: rbx_ym_write(0, p & 1, v); break;
	case 0x40: upd7759_control_w(v); break;
	case 0x80: upd7759_port_w(&s_upd, v); break;
	}
}
static void z_start(void)
{
	z80_init(&s_z80);
	s_z80.read_byte = z_read;
	s_z80.write_byte = z_write;
	s_z80.port_in = z_in;
	s_z80.port_out = z_out;
	s_z80.read_map = s_zmap;
	s_z80.op_rom = s_zop;
}

// ---- drawing ----
static struct {
	uint16_t paletteram[0x800];
	uint16_t tileram[0x8000];
	uint16_t textram[0x800];
	uint16_t spriteram[0x400];
	uint16_t latched_page[4], latched_yscroll[4], latched_xscroll[4];
	uint8_t display_enable, flip, tbank[2];
	int pal_dirty;
} *s_snap;
static uint16_t s_pal_conv[0x800];
static int s_pal_valid;
static uint8_t s_pal_normal[32], s_pal_shadow[32], s_pal_hilight[32];

static void palette_init(void)   // sega_16bit_common_base::palette_init (compute_resistor_weights, 6 resistors)
{
	static const double rn[6] = { 3900, 2000, 1000, 1000.0 / 2, 1000.0 / 4, 0 };
	static const double rs[6] = { 3900, 2000, 1000, 1000.0 / 2, 1000.0 / 4, 470 };
	double wn[6], ws[6];
	for (int set = 0; set < 2; set++) {
		const double *r = set ? rs : rn;
		double *w = set ? ws : wn, max = 0;
		for (int n = 0; n < 6; n++) {
			double r0 = 1.0 / 1e12, r1 = 1.0 / 1e12;
			for (int j = 0; j < 6; j++) {
				if (r[j] == 0)
					continue;
				if (j == n)
					r1 += 1.0 / r[j];
				else
					r0 += 1.0 / r[j];
			}
			r0 = 1.0 / r0;
			r1 = 1.0 / r1;
			double v = 255.0 * r0 / (r1 + r0);
			if (v > 255) v = 255;
			w[n] = r[n] != 0 ? v : 0;
			max += w[n];
		}
		const double scale = 255.0 / max;
		for (int n = 0; n < 6; n++)
			w[n] *= scale;
	}
	for (int v = 0; v < 32; v++) {
		double n = 0, s = 0;
		for (int bit = 0; bit < 5; bit++)
			if ((v >> bit) & 1) {
				n += wn[bit];
				s += ws[bit];
			}
		s_pal_normal[v] = (uint8_t)(int)(n + 0.5);
		s_pal_shadow[v] = (uint8_t)(int)(s + 0.5);
		s_pal_hilight[v] = (uint8_t)(int)(s + ws[5] + 0.5);
	}
}

// the tiles unpacked: a byte a pixel (gfx_8x8x3_planar)
static uint8_t *s_tdec;
static uint32_t s_tdec_n;
static void decode_tiles(void)
{
	const uint32_t third = s_tiles_size / 3;
	s_tdec_n = third / 8;
	s_tdec = (uint8_t *)malloc((size_t)s_tdec_n * 64 + 64);
	if (!s_tdec)
		return;
	for (uint32_t c = 0; c < s_tdec_n; c++)
		for (int py = 0; py < 8; py++) {
			const uint32_t o = c * 8 + (uint32_t)py;
			const uint8_t p0 = s_tiles[o], p1 = s_tiles[third + o], p2 = s_tiles[2 * third + o];
			for (int px = 0; px < 8; px++) {
				const int bit = 7 - px;
				s_tdec[c * 64 + (uint32_t)py * 8 + (uint32_t)px] =
					(uint8_t)(((p2 >> bit) & 1) << 2 | ((p1 >> bit) & 1) << 1 | ((p0 >> bit) & 1));
			}
		}
	memset(s_tdec + (size_t)s_tdec_n * 64, 0, 64);
}
static inline const uint8_t *tile_row(uint32_t code, int py)
{
	return code < s_tdec_n ? s_tdec + code * 64 + (uint32_t)py * 8 : s_tdec + (size_t)s_tdec_n * 64;
}

// one layer's line of the 16B tilemaps (tilemap_16b_draw_layer + draw_virtual_tilemap): col[x] the colour,
// cat[x] the tile category | 2 when the pen is not 0
static void layer_line(int which, int y, uint16_t *col, uint8_t *cat)
{
	const uint16_t *textram = s_snap->textram;
	const uint16_t xscroll = s_snap->latched_xscroll[which], yscroll = s_snap->latched_yscroll[which];
	const uint16_t rowscroll = textram[0xf80 / 2 + 0x40 / 2 * which + y / 8];
	uint16_t pages = s_snap->latched_page[which];
	uint16_t effx = (xscroll & 0x8000) ? rowscroll : xscroll, effy = yscroll;
	const int alt = (rowscroll & 0x8000) != 0;
	if (alt) {
		effx = s_snap->latched_xscroll[which + 2];
		effy = s_snap->latched_yscroll[which + 2];
		pages = s_snap->latched_page[which + 2];
	}
	effx = (uint16_t)((0xc0 - effx) & 0x3ff);
	const int colscroll = (yscroll & 0x8000) && !alt;
	for (int x = 0; x < VIS_W;) {
		int ey = effy, n = VIS_W - x;
		if (colscroll) {   // (a column of 16 pixels: x = 8 + 16 k .. 23 + 16 k)
			ey = textram[0xf16 / 2 + 0x40 / 2 * which + (x + 8) / 16];
			const int next = ((x + 8) / 16) * 16 + 8;
			n = (next > x ? next : next + 16) - x;
			if (n > VIS_W - x)
				n = VIS_W - x;
		}
		const int vy = (y + (ey & 0x1ff)) & 0x1ff, py = vy & 0xff;
		const int pbase = ((vy >> 8) & 1) * 8;
		const int end = x + n;
		while (x < end) {
			const int vx = (x + effx) & 0x3ff, px = vx & 0x1ff;
			const int page = (pages >> (pbase + ((vx >> 9) & 1) * 4)) & 15;
			const uint16_t data = s_snap->tileram[page * 64 * 32 + (py >> 3) * 64 + (px >> 3)];
			int code = data & 0x1fff;
			code = s_snap->tbank[code / 0x1000] * 0x1000 + code % 0x1000;
			const uint16_t cb = (uint16_t)(((data >> (s_alt_tiles ? 5 : 6)) & 0x7f) * 8);
			const uint8_t c = (uint8_t)((data >> 15) & 1);
			const uint8_t *row = tile_row((uint32_t)code, py & 7);
			int k = 8 - (px & 7);
			if (k > end - x)
				k = end - x;
			for (int i = 0; i < k; i++) {
				const uint8_t p = row[(px & 7) + i];
				col[x + i] = (uint16_t)(cb + p);
				cat[x + i] = (uint8_t)(c | (p ? 2 : 0));
			}
			x += k;
		}
	}
}

// the sprites of a band (sega_sys16b_sprite_device::draw; the zoom counter and the address kept locally)
static void draw_sprites(int y0, int y1, uint16_t (*sbuf)[VIS_W])
{
	for (int y = y0; y < y1; y++)
		for (int x = 0; x < VIS_W; x++)
			sbuf[y - y0][x] = 0xffff;
	static const uint8_t default_banklist[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
	static const uint8_t alternate_banklist[16] = { 0, 255, 255, 255, 255, 255, 255, 3, 255, 255, 255, 2, 255, 1, 0, 255 };
	const uint8_t *banklist = (s_romboard == RB_5358 || s_romboard == RB_5358_SMALL) ? alternate_banklist : default_banklist;
	const uint16_t *spritebase = (const uint16_t *)s_spr;
	const uint8_t numbanks = (uint8_t)(s_spr_size / 0x20000);
	const uint16_t *ram = s_snap->spriteram;
	for (const uint16_t *data = ram; data < ram + 0x400; data += 8) {
		if (data[2] & 0x8000)
			break;
		const int bottom = data[0] >> 8, top = data[0] & 0xff;
		const int xpos = data[1] & 0x1ff;
		const int hide = data[2] & 0x4000, flip = data[2] & 0x100;
		const int pitch = (int8_t)(data[2] & 0xff);
		uint16_t addr = data[3];
		int bank = banklist[(data[4] >> 8) & 0xf];
		const int colpri = ((data[4] & 0xff) << 4) | (((data[1] >> 9) & 0xf) << 12);
		const int vzoom = (data[5] >> 5) & 0x1f, hzoom = data[5] & 0x1f;
		if (hide || top >= bottom || bank == 255 || top >= y1 || bottom <= y0)
			continue;
		if (numbanks)
			bank %= numbanks;
		const uint16_t *sd = spritebase + 0x10000 * bank;
		uint16_t zacc = (uint16_t)(data[5] & 0x03ff);
		for (int y = top; y < bottom; y++) {
			addr = (uint16_t)(addr + pitch);
			zacc = (uint16_t)(zacc + (vzoom << 10));
			if (zacc & 0x8000) {
				addr = (uint16_t)(addr + pitch);
				zacc &= ~0x8000;
			}
			if (y < y0 || y >= y1)
				continue;
			uint16_t *dest = sbuf[y - y0];
			int xacc = 4 * hzoom, x;
			uint16_t a7;
			if (!flip) {
				a7 = (uint16_t)(addr - 1);
				for (x = xpos; ((xpos - x) & 0x1ff) != 1;) {
					const uint16_t pixels = sd[++a7];
					int pix = 0;
					for (int sh = 12; sh >= 0; sh -= 4) {
						pix = (pixels >> sh) & 15;
						xacc = (xacc & 0x3f) + hzoom;
						if (xacc < 0x40) {
							const int sx = x - 184;
							if (sx >= 0 && sx < VIS_W && pix != 0 && pix != 15)
								dest[sx] = (uint16_t)(colpri | pix);
							x++;
						}
					}
					if (pix == 15)
						break;
				}
			} else {
				a7 = (uint16_t)(addr + 1);
				for (x = xpos; ((xpos - x) & 0x1ff) != 1;) {
					const uint16_t pixels = sd[--a7];
					int pix = 0;
					for (int sh = 0; sh <= 12; sh += 4) {
						pix = (pixels >> sh) & 15;
						xacc = (xacc & 0x3f) + hzoom;
						if (xacc < 0x40) {
							const int sx = x - 184;
							if (sx >= 0 && sx < VIS_W && pix != 0 && pix != 15)
								dest[sx] = (uint16_t)(colpri | pix);
							x++;
						}
					}
					if (pix == 15)
						break;
				}
			}
		}
	}
}

#define BANDS 4
static uint16_t (*s_sbuf_hp)[VIS_H][VIS_W];   // the sprites (MAME's bitmap: 0xffff empty), by band
#define s_sbuf (*s_sbuf_hp)
static int s_band_next, s_band_left, s_band_active, s_band_helpers;

static void run_band(int band)
{
	const int y0 = VIS_H * band / BANDS, y1 = VIS_H * (band + 1) / BANDS;
	if (!s_snap->display_enable) {
		for (int y = y0; y < y1; y++)   // (the palette's black pen, past its 4096 colours)
			for (int x = 0; x < VIS_W; x++)
				rbx_video.bitmap[y * rbx_video.pitch + x] = 0x1000;
		return;
	}
	draw_sprites(y0, y1, s_sbuf + y0);
	uint16_t bcol[VIS_W], fcol[VIS_W];
	uint8_t bcat[VIS_W], fcat[VIS_W];
	for (int y = y0; y < y1; y++) {
		uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
		layer_line(1, y, bcol, bcat);
		layer_line(0, y, fcol, fcat);
		const uint16_t *text = &s_snap->textram[(y >> 3) * 64];
		const uint16_t *sp = s_sbuf[y];
		const uint16_t tb = (uint16_t)(s_snap->tbank[0] * 0x1000);
		for (int x = 0; x < VIS_W; x++) {
			// the background opaque (pen 0 shows), its pens over 0 marked 1 / 2 by category; the foreground's 2 / 4
			uint16_t col = bcol[x];
			uint8_t pri = (bcat[x] & 2) ? ((bcat[x] & 1) ? 0x02 : 0x01) : 0;
			if (fcat[x] & 2) {
				col = fcol[x];
				pri |= (fcat[x] & 1) ? 0x04 : 0x02;
			}
			const int tx = x + 192;
			const uint16_t data = text[(tx >> 3) & 63];
			const uint8_t pt = tile_row((uint32_t)(tb + (data & (s_alt_tiles ? 0xff : 0x1ff))), y & 7)[tx & 7];
			if (pt) {
				col = (uint16_t)(((data >> (s_alt_tiles ? 8 : 9)) & 7) * 8 + pt);
				pri |= (data & 0x8000) ? 0x08 : 0x04;
			}
			if (sp[x] != 0xffff) {
				const int priority = (sp[x] >> 10) & 3;
				if ((1 << priority) > pri) {
					if ((sp[x] & 0x03f0) == 0x03f0)
						col = (uint16_t)(col + 0x800);
					else
						col = (uint16_t)(0x400 | (sp[x] & 0x3ff));
				}
			}
			d[x] = col;
		}
	}
}

static void s16b_render(void)
{
	if (s_snap->pal_dirty || !s_pal_valid) {
		for (int i = 0; i < 0x800; i++) {
			const uint16_t v = s_snap->paletteram[i];
			if (s_pal_valid && s_pal_conv[i] == v)
				continue;
			s_pal_conv[i] = v;
			const int r = ((v >> 12) & 1) | ((v << 1) & 0x1e), g = ((v >> 13) & 1) | ((v >> 3) & 0x1e),
			          bb = ((v >> 14) & 1) | ((v >> 7) & 0x1e);
			rbx_video.palette[i] = (uint32_t)(s_pal_normal[r] << 16 | s_pal_normal[g] << 8 | s_pal_normal[bb]);
			rbx_video.palette[i + 0x800] = (v & 0x8000)
				? (uint32_t)(s_pal_hilight[r] << 16 | s_pal_hilight[g] << 8 | s_pal_hilight[bb])
				: (uint32_t)(s_pal_shadow[r] << 16 | s_pal_shadow[g] << 8 | s_pal_shadow[bb]);
		}
		rbx_video.palette[0x1000] = 0;
		s_pal_valid = 1;
		s_snap->pal_dirty = 0;
	}
	__atomic_store_n(&s_band_next, 0, __ATOMIC_SEQ_CST);
	__atomic_store_n(&s_band_left, BANDS, __ATOMIC_SEQ_CST);
	__atomic_store_n(&s_band_active, 1, __ATOMIC_SEQ_CST);
	for (;;) {
		const int bnd = __atomic_fetch_add(&s_band_next, 1, __ATOMIC_ACQ_REL);
		if (bnd >= BANDS)
			break;
		run_band(bnd);
		__atomic_sub_fetch(&s_band_left, 1, __ATOMIC_ACQ_REL);
	}
	while (__atomic_load_n(&s_band_left, __ATOMIC_ACQUIRE) > 0)
		;
	__atomic_store_n(&s_band_active, 0, __ATOMIC_SEQ_CST);
	while (__atomic_load_n(&s_band_helpers, __ATOMIC_ACQUIRE) > 0)
		;
}

static int s16b_render_help(int helper)
{
	const int one = helper & 0x100;
	helper &= 0xff;
	if (helper < 1 || helper > 2 || !__atomic_load_n(&s_band_active, __ATOMIC_SEQ_CST))
		return 0;
	__atomic_add_fetch(&s_band_helpers, 1, __ATOMIC_SEQ_CST);
	int did = 0;
	if (__atomic_load_n(&s_band_active, __ATOMIC_SEQ_CST))
		for (;;) {
			const int bnd = __atomic_fetch_add(&s_band_next, 1, __ATOMIC_ACQ_REL);
			if (bnd >= BANDS)
				break;
			run_band(bnd);
			__atomic_sub_fetch(&s_band_left, 1, __ATOMIC_ACQ_REL);
			did = 1;
			if (one)
				break;
		}
	__atomic_sub_fetch(&s_band_helpers, 1, __ATOMIC_SEQ_CST);
	return did;
}

static void vblank_draw(void)
{
	rbx_video_wait_idle();
	if (memcmp(s_snap->paletteram, b.paletteram, sizeof(b.paletteram))) {
		memcpy(s_snap->paletteram, b.paletteram, sizeof(b.paletteram));
		s_snap->pal_dirty = 1;
	}
	memcpy(s_snap->tileram, b.tileram, sizeof(b.tileram));
	memcpy(s_snap->textram, b.textram, sizeof(b.textram));
	memcpy(s_snap->spriteram, b.spriteram, sizeof(b.spriteram));
	memcpy(s_snap->latched_page, b.latched_page, sizeof(b.latched_page));
	memcpy(s_snap->latched_yscroll, b.latched_yscroll, sizeof(b.latched_yscroll));
	memcpy(s_snap->latched_xscroll, b.latched_xscroll, sizeof(b.latched_xscroll));
	s_snap->display_enable = b.display_enable;
	s_snap->flip = b.flip;
	s_snap->tbank[0] = b.tbank[0];
	s_snap->tbank[1] = b.tbank[1];
	rbx_video_frame_done();
}

// ---- the i8751 (system16b_i8751): the mapper's registers as its data space, SERVICE on P1, spin_68k_w ----
static int s_mcu_on, s_mcu_sim;   // (s_mcu_sim: 1 tturf, 2 wb3: MAME's simulations)
static mcs51_t s_mcu;
static const uint8_t *s_mcu_rom;
static uint32_t s_mcu_size;
static uint8_t mcu_xdata_r(void *u, uint16_t a)
{
	(void)u;
	a &= 0xff;
	return a < 0x20 ? mapper_r(a) : 0xff;
}
static void mcu_xdata_w(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	a &= 0xff;
	if (a < 0x20)
		mapper_w(a, v);
}
static uint8_t mcu_port_in(void *u, int port)
{
	(void)u;
	if (port == 1 && s_port_service >= 0)
		return (uint8_t)rbx_port_read(s_port_service);
	return 0xff;
}
static void mcu_port_out(void *u, int port, uint8_t v)
{
	(void)u;
	(void)v;
	if (port == 1)   // spin_68k_w: the 68000 waits 20000 cycles
		b.spin = 20000;
}
static void i8751_sim(void)   // tturf_i8751_sim, wb3_i8751_sim (at the vblank)
{
	m68k_bus_hold_irq(4);
	if (s_mcu_sim == 1) {
		const uint16_t temp = b.workram[0x01d0 / 2];
		if ((temp & 0xff00) != 0x0000) {
			mapper_w(0x03, (uint8_t)temp);
			b.workram[0x01d0 / 2] = temp & 0x00ff;
		}
		b.workram[0x01e6 / 2] = (uint16_t)((s_port_service >= 0 ? rbx_port_read(s_port_service) : 0xff) << 8);
		b.workram[0x01e8 / 2] = (uint16_t)((s_port_p1 >= 0 ? rbx_port_read(s_port_p1) : 0xff) << 8);
		b.workram[0x01ea / 2] = (uint16_t)((s_port_p2 >= 0 ? rbx_port_read(s_port_p2) : 0xff) << 8);
	} else {
		const uint16_t temp = b.workram[0x0008 / 2];
		if ((temp & 0x00ff) != 0x0000) {
			mapper_w(0x03, (uint8_t)(temp >> 8));
			b.workram[0x0008 / 2] = temp & 0xff00;
		}
	}
}

// ---- the FD1094: the opcodes of its state ----
static void fd_update(void)
{
	if (!s_fd1094_on)
		return;
	const uint16_t *op = fd1094_opcodes(&s_fd1094, fd1094_state(&s_fd1094));
	m68k_bus.op = (const uint8_t *)op;
	m68k_bus.op_size = op ? s_rom_size : 0;
}
static void fd_cmpild(int reg, unsigned int imm)   // cmp_callback: CMPI.L #$xxxxFFFF,D0
{
	if (reg == 0 && (imm & 0xffff) == 0xffff) {
		fd1094_change_state(&s_fd1094, (int)(imm >> 16));
		fd_update();
	}
}
static void fd_rte(void)
{
	fd1094_change_state(&s_fd1094, FD1094_STATE_RTE);
	fd_update();
}
static void fd_irqack(int level)
{
	(void)level;
	fd1094_change_state(&s_fd1094, FD1094_STATE_IRQ);
	fd_update();
}

// hwchamp's handles on the pad: left / right the monitor (AD_STICK_X), button 1 / 2 the left / right punch
// (AD_STICK_Z, at rest 0x20), up / down both fists' limits (AD_STICK_Y): the keys' delta 32 x 70 % a frame
static int hw_toward(int v, int target)
{
	return v < target ? (v + 22 > target ? target : v + 22) : (v - 22 < target ? target : v - 22);
}
static void hw_controls(void)
{
	const uint32_t in = rbx_input_bits(1);
	s_hw[0] = (uint8_t)hw_toward(s_hw[0], (in & RBX_IN_LEFT) ? 0x00 : (in & RBX_IN_RIGHT) ? 0xff : 0x80);
	s_hw[1] = (uint8_t)hw_toward(s_hw[1], (in & RBX_IN_B2) ? 0xff : 0x20);
	s_hw[2] = (uint8_t)hw_toward(s_hw[2], (in & RBX_IN_B1) ? 0xff : 0x20);
	const int lim = (in & RBX_IN_UP) ? 0x00 : (in & RBX_IN_DOWN) ? 0xff : 0x80;
	s_hw[3] = (uint8_t)hw_toward(s_hw[3], lim);
	s_hw[4] = (uint8_t)hw_toward(s_hw[4], lim);
}

// ---- the frame ----
static void s16b_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	if (s_io == IO_HWCHAMP)
		hw_controls();
	if (s_io == IO_DUNKSHOT || s_io == IO_SDI)
		for (int i = 0; i < (s_io == IO_DUNKSHOT ? 8 : 4); i++) {
			const int d = rbx_analog_dpad(i / 2 + 1, (i & 1) ? RBX_IPT_TRACKBALL_Y : RBX_IPT_TRACKBALL_X, 4);
			s_tb[i] = (uint8_t)(s_tb[i] + ((s_tb_rev >> i) & 1 ? -d : d));
		}
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_H) {   // the vblank: drawn, IRQ 4 (or the MCU's IRQ0 / the simulation)
			vblank_draw();
			if (s_mcu_sim)
				i8751_sim();
			else if (!s_mcu_on)
				m68k_bus_hold_irq(4);
		}
		if (s_mcu_on)
			mcs51_set_input(&s_mcu, MCS51_INT0_LINE, line >= VIS_H);   // (the screen's vblank)
		if (line == 261)   // tilemap_16b_latch_values
			for (int i = 0; i < 4; i++) {
				b.latched_page[i] = b.textram[0xe80 / 2 + i];
				b.latched_yscroll[i] = b.textram[0xe90 / 2 + i];
				b.latched_xscroll[i] = b.textram[0xe98 / 2 + i];
			}
		{
			int mc = rbx_line_cycles(&b.cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES);
			if (b.spin > 0) {   // (spin_until_time)
				const int skip = b.spin < mc ? b.spin : mc;
				b.spin -= skip;
				mc -= skip;
			}
			if (mc > 0 && !b.reset_held)
				m68k_execute(mc);
		}
		if (s_mcu_on)   // (8 MHz / 12: the machine cycles)
			mcs51_execute(&s_mcu, rbx_line_cycles(&b.mcu_frac, 8000000 / 12, FRAME_MHZ, LINES));
		const int zc = rbx_line_cycles(&b.z80_frac, Z80_CLOCK, FRAME_MHZ, LINES);
		const int uc = rbx_line_cycles(&b.upd_frac, UPD_CLOCK, FRAME_MHZ, LINES);
		for (int q = 0; q < 4; q++) {   // (the Z80 and the uPD7759's DRQ -> NMI in quarters of a line)
			const int zq = zc * (q + 1) / 4 - zc * q / 4, uq = uc * (q + 1) / 4 - uc * q / 4;
			z80_run(&s_z80, s_z80.cyc + (unsigned long)zq, &b.pbf, 0);
			if (upd7759_slave_advance(&s_upd, uq))
				z80_gen_nmi(&s_z80);
		}
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&b.ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
		const int k = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
		upd7759_render(&s_upd, s_upd_buf + s_mix_n, k);
		s_mix_n += k;
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i] + s_upd_buf[i] * 113 / 256);   // (the uPD7759: 0.48 x the netlist)
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	b.frame++;
}

static void s16b_reset(void)
{
	memset(&b, 0, offsetof(__typeof__(b), state_end));
	b.tbank[0] = 0;
	b.tbank[1] = 1;
	update_mapping();
	memset(s_zram, 0, sizeof(s_zram));
	memset(s_zmap, 0, sizeof(s_zmap));
	memset(s_tb, 0, sizeof(s_tb));
	if (s_wram16c)
		memset(s_wram16c, 0, 0x40000);
	{
		static const uint8_t hw0[6] = { 0x80, 0x20, 0x20, 0x80, 0x80, 0 };
		memcpy(s_hw, hw0, sizeof(s_hw));
	}
	for (uint32_t p = 0; p < 0x80 && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	z_bank();
	z_start();
	rbx_ym_reset(0);
	upd7759_init(&s_upd, NULL, 0, UPD_CLOCK, rbx_ym_rate(0));
	s_mix_n = 0;
	s_pal_valid = 0;
	if (s_mcu_on) {
		mcs51_init(&s_mcu, s_mcu_rom, s_mcu_size);
		s_mcu.xdata_r = mcu_xdata_r;
		s_mcu.xdata_w = mcu_xdata_w;
		s_mcu.port_in = mcu_port_in;
		s_mcu.port_out = mcu_port_out;
		mcs51_reset(&s_mcu);
	}
	m68k_bus_irq_reset();
	if (s_fd1094_on) {
		fd1094_change_state(&s_fd1094, FD1094_STATE_RESET);
		fd_update();
		rbx_m68k_cmpild_hook = fd_cmpild;
		rbx_m68k_rte_hook = fd_rte;
		rbx_m68k_irqack_hook = fd_irqack;
	}
	m68k_pulse_reset();
}

static void s16b_exit(void)
{
	fd1094_exit(&s_fd1094);
	free(s_zop);
	s_zop = NULL;
	free(s_wram16c);
	s_wram16c = NULL;
	free(s_fd1089_op);
	s_fd1089_op = NULL;
	rbx_ym_exit(0);
	free(s_mix);
	free(s_upd_buf);
	s_mix = NULL;
	s_upd_buf = NULL;
	free(s_snap);
	s_snap = NULL;
	free(s_tdec);
	s_tdec = NULL;
	free(s_bp);   // (last: the render thread and the saves are done with it)
	s_bp = NULL;
	free(s_sbuf_hp);
	s_sbuf_hp = NULL;
}

static int romboard_of(const char *machine)
{
	const char *init = strchr(machine, ':');
	if (!init)
		return -1;
	if (strstr(init, "5358_small")) return RB_5358_SMALL;
	if (strstr(init, "5358")) return RB_5358;
	if (strstr(init, "5521")) return RB_5521;
	if (strstr(init, "5704")) return RB_5704;
	if (strstr(init, "5797")) return RB_5797;
	if (strstr(init, "korean")) return RB_KOREAN;
	return -1;
}
static int s16b_supported(const char *machine)
{
	// (for now: the plain boards; FD1094 / FD1089 / MC8123 / the i8751 / the 5797's math chip: not yet)
	static const char *const ok[] = { "system16b:", "system16b_fd1094:", "system16b_5797:", "system16b_fd1094_5797:",
	                                  "system16b_i8751:", "system16b_i8751_5797:", "system16b_fd1089a:",
	                                  "system16b_fd1089b:", "system16b_mc8123:", "hwchamp:", "hwchamp_fd1094:", "system16c:", NULL };
	int known = 0;
	for (int i = 0; ok[i]; i++)
		if (!strncmp(machine, ok[i], strlen(ok[i])))
			known = 1;
	return known && romboard_of(machine) >= 0 && romboard_of(machine) != RB_KOREAN;
}

static int s16b_init(const rbx_game *g)
{
	if (!s_sbuf_hp && !(s_sbuf_hp = calloc(1, sizeof(*s_sbuf_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_bp && !(s_bp = (struct segas16b_state *)calloc(1, sizeof(*s_bp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s16b_supported(g->machine)) {
		rbx_fail("%s: machine %s not emulated", g->set, g->machine);
		return -1;
	}
	s_romboard = romboard_of(g->machine);
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_tiles = rbx_region("tiles", &s_tiles_size);
	s_spr = rbx_region("sprites", &s_spr_size);
	s_zrom = rbx_region("soundcpu", &s_zrom_size);
	if (!s_rom || !s_tiles || !s_spr || !s_zrom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_bus_swap16(s_spr, s_spr_size);
	s_fd1094_on = !strncmp(g->machine, "system16b_fd1094", 16) || !strncmp(g->machine, "hwchamp_fd1094:", 15);
	free(s_fd1089_op);
	s_fd1089_op = NULL;
	if (!strncmp(g->machine, "system16b_fd1089", 16)) {
		uint32_t ksz;
		const uint8_t *key = rbx_region("maincpu:key", &ksz);
		s_fd1089_op = (uint16_t *)malloc(s_rom_size);
		if (!key || ksz < 0x2000 || !s_fd1089_op) {
			rbx_fail("%s: no FD1089 key", g->set);
			return -1;
		}
		fd1089_decrypt(g->machine[16] == 'b', (uint16_t *)s_rom, s_rom_size, key, s_fd1089_op);
	}
	free(s_zop);
	s_zop = NULL;
	free(s_wram16c);
	s_wram16c = NULL;
	if (!strncmp(g->machine, "system16c:", 10) && !(s_wram16c = (uint16_t *)calloc(1, 0x40000))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!strncmp(g->machine, "system16b_mc8123", 16)) {
		uint32_t ksz;
		const uint8_t *key = rbx_region("soundcpu:key", &ksz);
		s_zop = (uint8_t *)malloc(0x10000);
		if (!key || ksz < 0x2000 || !s_zop || s_zrom_size < 0x8000) {
			rbx_fail("%s: no MC-8123 key", g->set);
			return -1;
		}
		memset(s_zop, 0xff, 0x10000);
		mc8123_decode(s_zrom, s_zop, 0x8000, key);
	}
	s_mcu_sim = strstr(g->machine, ":tturf_") ? 1 : strstr(g->machine, ":wb3_") ? 2 : 0;
	s_mcu_rom = rbx_region("mcu", &s_mcu_size);
	s_mcu_on = !strncmp(g->machine, "system16b_i8751", 15) && !s_mcu_sim && s_mcu_rom;
	memset(&s_fd1094, 0, sizeof(s_fd1094));
	if (s_fd1094_on) {
		uint32_t ksz;
		const uint8_t *key = rbx_region("maincpu:key", &ksz);
		if (!key || ksz < 0x2000) {
			rbx_fail("%s: no FD1094 key", g->set);
			return -1;
		}
		fd1094_init(&s_fd1094, (const uint16_t *)s_rom, s_rom_size, key);
	}   // (the sprite words read in host order)
	decode_tiles();
	palette_init();
	memset(&b, 0, sizeof(b));
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
	if (s_fd1089_op) {
		m68k_bus.op = (const uint8_t *)s_fd1089_op;
		m68k_bus.op_size = s_rom_size;
	}
	s_port_service = rbx_port_find("SERVICE");
	s_port_p1 = rbx_port_find("P1");
	s_port_unused = rbx_port_find("UNUSED");
	s_port_p2 = rbx_port_find("P2");
	s_port_dsw1 = rbx_port_find("DSW1");
	s_port_dsw2 = rbx_port_find("DSW2");
	s_port_p3 = rbx_port_find("P3");
	s_port_p4 = rbx_port_find("P4");
	{
		const char *init = strchr(g->machine, ':');
		s_io = strstr(init, ":dunkshot_") ? IO_DUNKSHOT
		     : (strstr(init, ":sdi_") || strstr(init, ":defense_") || strstr(init, ":exctleag_")) ? IO_SDI
		     : strstr(init, ":passshtj_") ? IO_PASSSHTJ : strstr(init, ":sjryuko_") ? IO_SJRYUKO
		     : strstr(init, ":hwchamp_") ? IO_HWCHAMP : IO_STD;
		for (int i = 0; i < 6; i++) {
			char tag[4] = { 'M', 'J', (char)('0' + i), 0 };
			s_port_mj[i] = rbx_port_find(tag);
		}
		s_tb_rev = s_io == IO_DUNKSHOT ? 0x99 : 0x01;
		s_alt_tiles = strstr(init, ":dunkshot_") || strstr(init, ":sjryuko_") || strstr(init, ":timescan_");
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 50;   // (0.43 x the netlist: as loud as MAME 0.260 with USE_NL)
	if (!s_snap || !s_tdec || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		s16b_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_upd_buf = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || !s_upd_buf || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 0x1001)) {
		rbx_fail("%s: out of memory", g->set);
		s16b_exit();
		return -1;
	}
	s16b_reset();
	return 0;
}

static int s16b_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	z80 z = s_z80;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "s16b", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	if (s_io == IO_DUNKSHOT || s_io == IO_SDI)
		rc |= rbx_state_io(ctx, "trackball", s_tb, sizeof(s_tb));
	if (s_io == IO_HWCHAMP)
		rc |= rbx_state_io(ctx, "hwchamp", s_hw, sizeof(s_hw));
	if (s_wram16c)
		rc |= rbx_state_io(ctx, "workram16c", s_wram16c, 0x40000);
	if (s_mcu_on)
		rc |= rbx_state_io(ctx, "i8751", &s_mcu.st, sizeof(s_mcu.st));
	if (s_fd1094_on) {
		uint8_t fs[2] = { s_fd1094.st, s_fd1094.irqmode };
		rc |= rbx_state_io(ctx, "fd1094", fs, sizeof(fs));
		if (load) {
			s_fd1094.st = fs[0];
			s_fd1094.irqmode = fs[1];
		}
	}
	rc |= rbx_ym_state(0, ctx, load);
	rc |= upd7759_state(&s_upd, ctx, load, "upd7759");
	rc |= upd7759_slave_state(&s_upd, ctx, "upd7759s");
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		z.read_byte = z_read;
		z.write_byte = z_write;
		z.port_in = z_in;
		z.port_out = z_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		z.op_rom = s_zop;
		s_z80 = z;
		update_mapping();
		z_bank();
		fd_update();
		if (s_fd1094_on) {
			rbx_m68k_cmpild_hook = fd_cmpild;
			rbx_m68k_rte_hook = fd_rte;
			rbx_m68k_irqack_hook = fd_irqack;
		}
		s_pal_valid = 0;
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int s16b_save(void *ctx) { return s16b_state(ctx, 0); }
static int s16b_load(void *ctx) { return s16b_state(ctx, 1); }

const rbx_driver rbx_drv_segas16b = { "Sega System 16B", s16b_init, s16b_exit, s16b_reset, s16b_frame, s16b_save,
                                      s16b_load, s16b_supported, s16b_render, s16b_render_help };
