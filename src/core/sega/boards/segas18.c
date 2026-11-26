/*  Sega System 18. Ported from MAME src/mame/sega/segas18.cpp, segas18_v.cpp, 315_5195.cpp, 315_5296.cpp,
    segaic16.cpp, sega16sp.cpp (BSD-3-Clause, copyright-holders: Aaron Giles, Charles MacDonald, Nicola Salmoria,
    David Haywood).
    A 68000 at 10 MHz behind the 315-5195 memory mapper (as on System 16B), the 315-5296 I/O chip, the 16B tilemaps
    with 8 tile banks, the 16B sprites with banks, and a 315-5313 (the Mega Drive VDP) as one more layer mixed among
    them. Sound: a Z80 at 8 MHz with two YM3438 and a RF5C68. 342x262, 57.23 Hz, 320x224 shown.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../rbx_core.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/mcs51/mcs51.h"
#include "../../machine/i8255.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/rf5c68.h"
#include "../machine/fd1094.h"
#include "../video/s315_5313.h"

extern void (*rbx_m68k_cmpild_hook)(int reg, unsigned int imm);
extern void (*rbx_m68k_rte_hook)(void);
extern void (*rbx_m68k_irqack_hook)(int level);

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 10000000
#define Z80_CLOCK 8000000
#define YM_CLOCK 8000000
#define RF_CLOCK 10000000
#define FRAME_MHZ 57230          // (verified on a PCB)
#define LINES 262
#define HTOTAL 342
#define VIS_W 320
#define VIS_H 224
#define PAL_VDP 0x1000           // the VDP's colours (normal; MAME's shadow / hilight copies unused by the mixer)
#define PAL_BLACK 0x1040

enum { RB_SHADOW, RB_5874, RB_5987, RB_7525 };

// ---- ROMs ----
static uint8_t *s_rom, *s_tiles, *s_spr, *s_zrom;
static uint32_t s_rom_size, s_tiles_size, s_spr_size, s_zrom_size;
static int s_romboard, s_fd1094_on, s_ddcrew4p;
static fd1094_t s_fd1094;
static int s_port_p1, s_port_p2, s_port_p3, s_port_service, s_port_coinage, s_port_dsw;
static int s_port_exp3, s_port_exp4, s_port_exservice;

// ---- the state (saved) ----
static struct segas18_state {   // (on the heap while the game runs, ADR-040)
	uint16_t workram[0x2000];   // 16 KB
	uint16_t spriteram[0x400];
	uint16_t paletteram[0x800];
	uint16_t tileram[0x8000];
	uint16_t textram[0x800];
	uint8_t regs[0x20];         // the 315-5195's
	uint8_t to_sound, from_sound, pbf, mcu_int;
	uint8_t display_enable, flip, vdp_enable, vdp_mixing;
	uint8_t tbank[8], sbank[16];
	uint16_t latched_page[4], latched_yscroll[4], latched_xscroll[4];
	uint8_t io_dir, io_cnt, io_latch[8];
	uint8_t zbank, reset_held, pad[2];
	uint64_t cpu_frac, z80_frac, ym_frac, mcu_frac;
	uint32_t frame;
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)

static z80 s_z80;
static int s_zirq;
static uint8_t s_zram[0x2000];
static const uint8_t *s_zmap[256];
static rf5c68_t s_rf;
static s315_5313_t s_vdp;
static i8255_t s_ppi;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static int s_line, s_line_mc;   // (the line being run, its 68000 cycles: the VDP's HV counter)

// ---- the 315-5195 memory mapper (as segas16b.c) ----
enum { E_MEM, E_IO, E_VDP, E_BANK5987, E_BANK7525 };
typedef struct {
	uint32_t start, end, mirror, wbase, wend;
	int kind, rd, wr;
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
static void map_rom(uint32_t offset, uint32_t length, uint32_t mirror, uint32_t rgnoffset, int wkind)
{
	if (rgnoffset >= s_rom_size)
		return;
	uint32_t len = length;
	if (rgnoffset + len > s_rom_size)
		len = s_rom_size - rgnoffset;
	map_add(offset, len, mirror, wkind < 0 ? E_MEM : wkind, s_rom + rgnoffset, len, 1, wkind < 0 ? -1 : 1);
}
// segas18_state::memory_mapper
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
	case 3: map_add(0x00000, 0x04000, 0xffc000, E_MEM, (uint8_t *)b.workram, 0x4000, 1, 1); break;
	case 2:
		if (s_romboard != RB_SHADOW)
			map_add(0x00000, 0x00010, 0xfffff0, E_VDP, NULL, 0, 1, 1);
		break;
	case 1:
		switch (s_romboard) {
		case RB_SHADOW: map_add(0x00000, 0x00010, 0xfffff0, E_VDP, NULL, 0, 1, 1); break;
		case RB_5874: map_rom(0x00000, 0x80000, 0xf80000, 0x80000, -1); break;
		case RB_5987:
			if (s_rom_size <= 0x100000)
				map_rom(0x00000, 0x80000, 0xf80000, 0x80000, E_BANK5987);
			else
				map_rom(0x00000, 0x100000, 0xf00000, 0x100000, E_BANK5987);
			break;
		case RB_7525: map_rom(0x00000, 0x80000, 0xf80000, 0x80000, E_BANK7525); break;
		}
		break;
	case 0:
		if ((s_romboard == RB_5987 || s_romboard == RB_7525) && s_rom_size > 0x100000)
			map_rom(0x00000, 0x100000, 0xf00000, 0x00000, -1);
		else
			map_rom(0x00000, 0x80000, 0xf80000, 0x00000, -1);
		break;
	}
}
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
static int hpos_now(void);

// ROM writes: the tile / sprite banks (rom_5987_bank_w, rom_837_7525_bank_w)
static void bank5987_w(uint32_t offset, uint16_t data, uint16_t mask)
{
	if (!(mask & 0xff))
		return;
	offset &= 0xf;
	data &= 0xff;
	if (offset < 8) {
		const uint32_t maxbanks = (s_tiles_size / 3 / 8) >> 10;
		if (maxbanks && data >= maxbanks)
			data = (uint16_t)(rbx_wrap(data, maxbanks));
		b.tbank[offset] = (uint8_t)data;
	} else {
		const uint32_t maxbanks = s_spr_size >> 18;
		if (data >= maxbanks)
			data = 255;
		b.sbank[(offset - 8) * 2 + 0] = (uint8_t)(data == 255 ? 255 : data * 2 + 0);
		b.sbank[(offset - 8) * 2 + 1] = (uint8_t)(data == 255 ? 255 : data * 2 + 1);
	}
}
static void bank7525_w(uint32_t offset, uint16_t data, uint16_t mask)
{
	if (!(mask & 0xff))
		return;
	offset &= 0xf;
	data &= 0xff;
	if (offset < 8) {
		data &= 0x9f;
		if (data & 0x80) data += 0x20;
		b.tbank[offset] = (uint8_t)(data & 0x3f);
	}
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
		case E_MEM: case E_BANK5987: case E_BANK7525:
			return o + 1 < e->memsize ? *(const uint16_t *)(e->mem + o) : 0xffff;
		case E_IO: return io_r(o >> 1);
		case E_VDP: return s315_5313_r(&s_vdp, (int)(o >> 1), 0xffff, s_line, hpos_now());
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
		case E_VDP: s315_5313_w(&s_vdp, (int)(o >> 1), v, mask); return;
		case E_BANK5987: bank5987_w(o >> 1, v, mask); return;
		case E_BANK7525: bank7525_w(o >> 1, v, mask); return;
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
static uint16_t open_bus(void)
{
	if (s_open_bus_recurse)
		return 0xffff;
	s_open_bus_recurse = 1;
	const uint16_t r = main_r16(m68k_get_reg(NULL, M68K_REG_PC));
	s_open_bus_recurse = 0;
	return r;
}

static void mcu_int_line(int state);
static void mapper_w(int offset, uint8_t data)
{
	offset &= 0x1f;
	const uint8_t oldval = b.regs[offset];
	b.regs[offset] = data;
	switch (offset) {
	case 0x02:
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
	case 0x03:   // to the sound CPU (its NMI)
		b.to_sound = data;
		if (!b.pbf)
			z80_gen_nmi(&s_z80);
		b.pbf = 1;
		break;
	case 0x04:
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
	case 0x03:
		mcu_int_line(0);
		return b.from_sound;
	}
	return (uint8_t)open_bus();
}

// ---- the 315-5296 I/O chip and misc_io_r / misc_io_w ----
static void io_out_port(int port, uint8_t data)
{
	switch (port) {
	case 3:   // misc_outputs_w (the grayscale: unused by MAME's mixer)
		b.flip = (data >> 5) & 1;
		break;
	case 7:   // rom_5874_bank_w
		if (s_romboard == RB_5874 || s_romboard == RB_SHADOW)
			for (int i = 0; i < 4; i++) {
				b.tbank[0 + i] = (uint8_t)((data & 0xf) * 4 + i);
				b.tbank[4 + i] = (uint8_t)(((data >> 4) & 0xf) * 4 + i);
			}
		break;
	}
}
static uint8_t io5296_r(int offset)
{
	offset &= 0x3f;
	if (offset < 8) {
		if ((b.io_dir >> offset) & 1)
			return b.io_latch[offset];
		const int ports[8] = { s_port_p1, s_port_p2, s_port_p3, -1, s_port_service, s_port_coinage, s_port_dsw, -1 };
		return ports[offset] >= 0 ? (uint8_t)rbx_port_read(ports[offset]) : 0xff;
	}
	switch (offset) {
	case 0x8: return 'S';
	case 0x9: return 'E';
	case 0xa: return 'G';
	case 0xb: return 'A';
	case 0xc: case 0xe: return b.io_cnt;
	case 0xd: case 0xf: return b.io_dir;
	}
	return 0xff;
}
static void io5296_w(int offset, uint8_t data)
{
	offset &= 0x3f;
	if (offset < 8) {
		if ((b.io_dir >> offset) & 1)
			io_out_port(offset, data);
		b.io_latch[offset] = data;
	} else if (offset == 0xe) {   // CNT1 the display, CNT2 the VDP
		b.display_enable = (data >> 1) & 1;
		b.vdp_enable = (data >> 2) & 1;
		b.io_cnt = data;
	} else if (offset == 0xf) {
		for (int i = 0; i < 8; i++)
			if (((b.io_dir ^ data) >> i) & 1)
				io_out_port(i, ((data >> i) & 1) ? b.io_latch[i] : 0);
		b.io_dir = data;
	}
}
static uint16_t io_r(uint32_t offset)
{
	offset &= 0x1fff;
	switch (offset & (0x3000 / 2)) {
	case 0x0000 / 2: case 0x1000 / 2:
		return (uint16_t)(io5296_r((int)offset) | (open_bus() & 0xff00));
	case 0x2000 / 2: {
		const int p = (offset & 1) ? s_port_coinage : s_port_service;
		return p >= 0 ? (uint16_t)(rbx_port_read(p) & 0xff) : 0xff;
	}
	}
	if (s_ddcrew4p && (offset == 0x3020 / 2 || offset == 0x3022 / 2 || offset == 0x3024 / 2))
		return i8255_read(&s_ppi, (int)(offset & 3));
	return open_bus();
}
static void io_w(uint32_t offset, uint16_t data, uint16_t mask)
{
	offset &= 0x1fff;
	switch (offset & (0x3000 / 2)) {
	case 0x0000 / 2: case 0x1000 / 2:
		if (mask & 0xff) {
			io5296_w((int)offset, (uint8_t)data);
			return;
		}
		break;
	case 0x2000 / 2:
		if (mask & 0xff) {
			b.vdp_mixing = (uint8_t)data;
			return;
		}
		break;
	}
	if (s_ddcrew4p && offset >= 0x3020 / 2 && offset <= 0x3026 / 2)
		i8255_write(&s_ppi, (int)(offset & 3), (uint8_t)data);
}
static uint8_t ppi_pa(void *u) { (void)u; return s_port_exp3 >= 0 ? (uint8_t)rbx_port_read(s_port_exp3) : 0xff; }
static uint8_t ppi_pb(void *u) { (void)u; return s_port_exp4 >= 0 ? (uint8_t)rbx_port_read(s_port_exp4) : 0xff; }
static uint8_t ppi_pc(void *u) { (void)u; return s_port_exservice >= 0 ? (uint8_t)rbx_port_read(s_port_exservice) : 0xff; }

// ---- the VDP's links: the 68000's space for its DMA, its colours, the HV position ----
static uint16_t vdp_read68k(void *u, uint32_t a) { (void)u; return main_r16(a & 0xfffffe); }
static void vdp_spin(void *u, int ns) { (void)u; (void)ns; }   // (a few cycles: dropped)
static uint16_t s_vdp_pal[0x40];   // (the 9-bit colours; saved)
static int s_vdp_pal_dirty;
static void vdp_cram_w(void *u, int index, uint16_t rgb9)
{
	(void)u;
	s_vdp_pal[index & 0x3f] = rgb9;
	s_vdp_pal_dirty = 1;
}
static int hpos_now(void)
{
	if (s_line_mc <= 0)
		return 0;
	int done = m68k_cycles_run();
	if (done > s_line_mc)
		done = s_line_mc;
	return done * HTOTAL / s_line_mc;
}

// ---- the Z80 (sound_map, sound_portmap) ----
static void z_bank(void)
{
	for (int p = 0xa0; p < 0xc0; p++) {
		const uint32_t o = (uint32_t)b.zbank * 0x2000 + (uint32_t)(p - 0xa0) * 256;
		s_zmap[p] = o + 256 <= s_zrom_size ? s_zrom + o : NULL;
	}
}
static uint8_t z_read(void *u, uint16_t a)
{
	(void)u;
	const uint8_t *p = s_zmap[a >> 8];
	if (p)
		return p[a & 0xff];
	if (a >= 0xe000)
		return s_zram[a - 0xe000];
	if (a >= 0xd000 && a < 0xe000)
		return rf5c68_mem_r(&s_rf, a & 0xfff);
	return 0xff;
}
static void z_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xe000)
		s_zram[a - 0xe000] = v;
	else if (a >= 0xd000)
		rf5c68_mem_w(&s_rf, a & 0xfff, v);
	else if (a >= 0xc000 && (a & 0x0f) <= 8)
		rf5c68_w(&s_rf, a & 0x0f, v);
}
static void mcu_int_line(int state);
static uint8_t z_in(z80 *z, uint8_t p)
{
	(void)z;
	switch (p & 0xe0) {
	case 0x80: return rbx_ym_read((p >> 4) & 1, p & 3);
	case 0xc0:   // pread
		b.pbf = 0;
		return b.to_sound;
	}
	return 0xff;
}
static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	switch (p & 0xe0) {
	case 0x80: rbx_ym_write((p >> 4) & 1, p & 3, v); break;
	case 0xa0: b.zbank = v; z_bank(); break;
	case 0xc0:   // pwrite (write_from_sound)
		b.from_sound = v;
		mcu_int_line(1);
		break;
	}
}
static void ym1_irq(int state) { s_zirq = state; }
static void z_start(void)
{
	z80_init(&s_z80);
	s_z80.read_byte = z_read;
	s_z80.write_byte = z_write;
	s_z80.port_in = z_in;
	s_z80.port_out = z_out;
	s_z80.read_map = s_zmap;
}

// ---- the i8751 (system18_i8751): the mapper as its data space, INT0 the vblank, INT1 the mapper ----
static int s_mcu_on;
static mcs51_t s_mcu;
static const uint8_t *s_mcu_rom;
static uint32_t s_mcu_size;
static void mcu_int_line(int state)
{
	b.mcu_int = (uint8_t)state;
	if (s_mcu_on)
		mcs51_set_input(&s_mcu, MCS51_INT1_LINE, state);
}
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

// ---- the FD1094 ----
static void fd_update(void)
{
	if (!s_fd1094_on)
		return;
	const uint16_t *op = fd1094_opcodes(&s_fd1094, fd1094_state(&s_fd1094));
	m68k_bus.op = (const uint8_t *)op;
	m68k_bus.op_size = op ? s_rom_size : 0;
}
static void fd_cmpild(int reg, unsigned int imm)
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

// ---- drawing ----
static struct {
	uint16_t paletteram[0x800];
	uint16_t tileram[0x8000];
	uint16_t textram[0x800];
	uint16_t spriteram[0x400];
	uint16_t latched_page[4], latched_yscroll[4], latched_xscroll[4];
	uint16_t vdp_pal[0x40];
	uint8_t display_enable, flip, tbank[8], sbank[16];
	int pal_dirty;
} *s_snap;
// the VDP as the frame began (its state copied at line 0: no VDP interrupt reaches the 68000 on System 18, so the
// games change it in the vblank) and each line's VDP enable and mixing: two sets, the drawing reads the other
static s315_5313_t *s_vstate[2];
static uint8_t s_vline_on[2][VIS_H], s_vline_mix[2][VIS_H];
static int s_vset;
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
	if (!s_tdec_n)
		return s_tdec;
	return s_tdec + (rbx_wrap(code, s_tdec_n)) * 64 + (uint32_t)py * 8;   // (the gfx element wraps the code)
}

// one layer's line of the 16B tilemaps (8 banks of 0x400 tiles)
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
		if (colscroll) {
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
			code = s_snap->tbank[code / 0x400] * 0x400 + code % 0x400;
			const uint16_t cb = (uint16_t)(((data >> 6) & 0x7f) * 8);
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

// the sprites of a band (sega_sys16b_sprite_device::draw, the banks set by the ROM board)
static void draw_sprites(int y0, int y1, uint16_t (*sbuf)[VIS_W])
{
	for (int y = y0; y < y1; y++)
		for (int x = 0; x < VIS_W; x++)
			sbuf[y - y0][x] = 0xffff;
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
		int bank = s_snap->sbank[(data[4] >> 8) & 0xf];
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
static uint16_t (*s_sbuf_hp)[VIS_H][VIS_W];
#define s_sbuf (*s_sbuf_hp)
static int s_band_next, s_band_left, s_band_active, s_band_helpers;

static inline void vdp_pixel(uint16_t raw, int pri_or, uint16_t *col, uint8_t *pri)   // draw_vdp
{
	if ((raw & 0x100) && (raw & 0x0f)) {
		*col = (uint16_t)(PAL_VDP + (raw & 0x3f));
		*pri |= (uint8_t)pri_or;
	}
}

static void run_band(int band)
{
	const int y0 = VIS_H * band / BANDS, y1 = VIS_H * (band + 1) / BANDS;
	if (!s_snap->display_enable) {
		for (int y = y0; y < y1; y++)
			for (int x = 0; x < VIS_W; x++)
				rbx_video.bitmap[y * rbx_video.pitch + x] = PAL_BLACK;
		return;
	}
	draw_sprites(y0, y1, s_sbuf + y0);
	const int set = s_vset ^ 1;
	uint16_t bcol[VIS_W], fcol[VIS_W];
	uint8_t bcat[VIS_W], fcat[VIS_W];
	for (int y = y0; y < y1; y++) {
		uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
		layer_line(1, y, bcol, bcat);
		layer_line(0, y, fcol, fcat);
		const uint16_t *text = &s_snap->textram[(y >> 3) * 64];
		const uint16_t *sp = s_sbuf[y];
		const uint16_t tb = (uint16_t)(s_snap->tbank[0] * 0x400);
		const int von = s_vline_on[set][y];
		const int mixing = s_vline_mix[set][y];
		const int vl = (mixing >> 1) & 3, vp = (mixing & 1) ? (1 << vl) : 0;
		uint16_t vr[VIS_W];
		if (von)
			s315_5313_render_line(s_vstate[set], y, vr);
		for (int x = 0; x < VIS_W; x++) {
			uint16_t col = bcol[x];
			uint8_t pri = 0;
			if (von && vl == 0) vdp_pixel(vr[x], vp, &col, &pri);
			if (bcat[x] & 2) {
				col = bcol[x];
				pri |= (bcat[x] & 1) ? 0x02 : 0x01;
			}
			if (von && vl == 1) vdp_pixel(vr[x], vp, &col, &pri);
			if (fcat[x] & 2) {
				col = fcol[x];
				pri |= (fcat[x] & 1) ? 0x04 : 0x02;
			}
			if (von && vl == 2) vdp_pixel(vr[x], vp, &col, &pri);
			const int tx = x + 192;
			const uint16_t data = text[(tx >> 3) & 63];
			const uint8_t pt = tile_row((uint32_t)(tb + (data & 0x1ff)), y & 7)[tx & 7];
			if (pt) {
				col = (uint16_t)(((data >> 9) & 7) * 8 + pt);
				pri |= (data & 0x8000) ? 0x08 : 0x04;
			}
			if (von && vl == 3) vdp_pixel(vr[x], vp, &col, &pri);
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

static void s18_render(void)
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
		for (int i = 0; i < 0x40; i++)
			rbx_video.palette[PAL_VDP + i] = s315_5313_rgb(s_snap->vdp_pal[i]);
		rbx_video.palette[PAL_BLACK] = 0;
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

static int s18_render_help(int helper)
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
	if (memcmp(s_snap->paletteram, b.paletteram, sizeof(b.paletteram)) || s_vdp_pal_dirty) {
		memcpy(s_snap->paletteram, b.paletteram, sizeof(b.paletteram));
		memcpy(s_snap->vdp_pal, s_vdp_pal, sizeof(s_vdp_pal));
		s_vdp_pal_dirty = 0;
		s_snap->pal_dirty = 1;
	}
	memcpy(s_snap->tileram, b.tileram, sizeof(b.tileram));
	memcpy(s_snap->textram, b.textram, sizeof(b.textram));
	memcpy(s_snap->spriteram, b.spriteram, sizeof(b.spriteram));
	memcpy(s_snap->latched_page, b.latched_page, sizeof(b.latched_page));
	memcpy(s_snap->latched_yscroll, b.latched_yscroll, sizeof(b.latched_yscroll));
	memcpy(s_snap->latched_xscroll, b.latched_xscroll, sizeof(b.latched_xscroll));
	memcpy(s_snap->tbank, b.tbank, sizeof(b.tbank));
	memcpy(s_snap->sbank, b.sbank, sizeof(b.sbank));
	s_snap->display_enable = b.display_enable;
	s_snap->flip = b.flip;
	s_vset ^= 1;   // (the lines just made go to the drawing)
	rbx_video_frame_done();
}

// ---- the frame ----
static void s18_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		s315_5313_scanline(&s_vdp, line);
		if (line == 0)
			memcpy(s_vstate[s_vset], &s_vdp, S315_5313_STATE_SIZE);
		if (line < VIS_H) {   // (the line's VDP enable and mixing)
			s_vline_on[s_vset][line] = b.vdp_enable;
			s_vline_mix[s_vset][line] = b.vdp_mixing;
		}
		if (line == VIS_H) {   // the vblank: drawn, IRQ 4 (or the MCU's INT0)
			vblank_draw();
			if (!s_mcu_on)
				m68k_bus_hold_irq(4);
		}
		if (s_mcu_on)
			mcs51_set_input(&s_mcu, MCS51_INT0_LINE, line == VIS_H);
		if (line == 261)
			for (int i = 0; i < 4; i++) {
				b.latched_page[i] = b.textram[0xe80 / 2 + i];
				b.latched_yscroll[i] = b.textram[0xe90 / 2 + i];
				b.latched_xscroll[i] = b.textram[0xe98 / 2 + i];
			}
		{
			const int mc = rbx_line_cycles(&b.cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES);
			s_line_mc = mc;
			if (mc > 0 && !b.reset_held)
				m68k_execute(mc);
			s_line_mc = 0;
		}
		if (s_mcu_on)
			mcs51_execute(&s_mcu, rbx_line_cycles(&b.mcu_frac, 8000000 / 12, FRAME_MHZ, LINES));
		const int zc = rbx_line_cycles(&b.z80_frac, Z80_CLOCK, FRAME_MHZ, LINES);
		z80_run(&s_z80, s_z80.cyc + (unsigned long)zc, &s_zirq, 0);
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&b.ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
		const int k = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
		rbx_ym_advance(1, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
		rf5c68_render(&s_rf, s_mix + s_mix_n, k, rbx_ym_rate(0));
		s_mix_n += k;
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	rbx_ym_async_take(1, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	b.frame++;
}

static void s18_reset(void)
{
	memset(&b, 0, offsetof(__typeof__(b), state_end));
	for (int i = 0; i < 8; i++)
		b.tbank[i] = (uint8_t)i;
	for (int i = 0; i < 16; i++)
		b.sbank[i] = (uint8_t)i;
	update_mapping();
	memset(s_zram, 0, sizeof(s_zram));
	memset(s_zmap, 0, sizeof(s_zmap));
	for (uint32_t p = 0; p < 0xa0 && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	z_bank();
	z_start();
	s_zirq = 0;
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	rf5c68_init(&s_rf, RF_CLOCK);
	s315_5313_init(&s_vdp);
	s_vdp.read68k = vdp_read68k;
	s_vdp.spin = vdp_spin;
	s_vdp.cram_w = vdp_cram_w;
	s315_5313_reset(&s_vdp);
	memset(s_vdp_pal, 0, sizeof(s_vdp_pal));
	s_vdp_pal_dirty = 1;
	memset(s_vline_on, 0, sizeof(s_vline_on));
	memset(&s_ppi, 0, sizeof(s_ppi));
	s_ppi.in_pa = ppi_pa;
	s_ppi.in_pb = ppi_pb;
	s_ppi.in_pc = ppi_pc;
	s_ppi.tri_pa = s_ppi.tri_pb = s_ppi.tri_pc = 0xff;
	i8255_reset(&s_ppi);
	s_mix_n = 0;
	s_pal_valid = 0;
	if (s_mcu_on) {
		mcs51_init(&s_mcu, s_mcu_rom, s_mcu_size);
		s_mcu.xdata_r = mcu_xdata_r;
		s_mcu.xdata_w = mcu_xdata_w;
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

static void s18_exit(void)
{
	fd1094_exit(&s_fd1094);
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix);
	s_mix = NULL;
	free(s_snap);
	s_snap = NULL;
	free(s_tdec);
	s_tdec = NULL;
	for (int i = 0; i < 2; i++) {
		free(s_vstate[i]);
		s_vstate[i] = NULL;
	}
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
	if (strstr(init, ":generic_shad")) return RB_SHADOW;
	if (strstr(init, ":generic_5874")) return RB_5874;
	if (strstr(init, ":generic_5987") || strstr(init, ":ddcrew4p")) return RB_5987;
	if (strstr(init, ":hamaway")) return RB_7525;
	return -1;   // (lghost, wwally: their custom I/O not yet)
}
static int s18_supported(const char *machine)
{
	static const char *const ok[] = { "system18:", "system18_fd1094:", "system18_i8751:", "system18_fd1094_i8751:",
	                                  "ddcrew4p:", "ddcrew4p_fd1094:", NULL };
	for (int i = 0; ok[i]; i++)
		if (!strncmp(machine, ok[i], strlen(ok[i])))
			return romboard_of(machine) >= 0;
	return 0;
}

static int s18_init(const rbx_game *g)
{
	if (!s_sbuf_hp && !(s_sbuf_hp = calloc(1, sizeof(*s_sbuf_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_bp && !(s_bp = (struct segas18_state *)calloc(1, sizeof(*s_bp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s18_supported(g->machine)) {
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
	{
		char prefix[48];
		const size_t n = (size_t)(strchr(g->machine, ':') - g->machine);
		memcpy(prefix, g->machine, n < sizeof(prefix) ? n : sizeof(prefix) - 1);
		prefix[n < sizeof(prefix) ? n : sizeof(prefix) - 1] = 0;
		s_fd1094_on = strstr(prefix, "fd1094") != NULL;
		s_mcu_rom = rbx_region("mcu", &s_mcu_size);
		s_mcu_on = strstr(prefix, "i8751") != NULL && s_mcu_rom;
	}
	s_ddcrew4p = strstr(g->machine, ":ddcrew4p") != NULL;
	memset(&s_fd1094, 0, sizeof(s_fd1094));
	if (s_fd1094_on) {
		uint32_t ksz;
		const uint8_t *key = rbx_region("maincpu:key", &ksz);
		if (!key || ksz < 0x2000) {
			rbx_fail("%s: no FD1094 key", g->set);
			return -1;
		}
		fd1094_init(&s_fd1094, (const uint16_t *)s_rom, s_rom_size, key);
	}
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
	s_port_p1 = rbx_port_find("P1");
	s_port_p2 = rbx_port_find("P2");
	s_port_p3 = rbx_port_find("P3");
	s_port_service = rbx_port_find("SERVICE");
	s_port_coinage = rbx_port_find("COINAGE");
	s_port_dsw = rbx_port_find("DSW");
	s_port_exp3 = rbx_port_find("EXP3");
	s_port_exp4 = rbx_port_find("EXP4");
	s_port_exservice = rbx_port_find("EXSERVICE");
	s_snap = calloc(1, sizeof(*s_snap));
	for (int i = 0; i < 2; i++)
		s_vstate[i] = calloc(1, sizeof(s315_5313_t));
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM3438;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 102;   // (0.40, both sides to the mono speaker)
	yc.irq = ym1_irq;
	int bad = !s_snap || !s_tdec || !s_vstate[0] || !s_vstate[1] || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	yc.irq = NULL;
	bad = bad || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	if (bad) {
		rbx_fail("%s: out of memory", g->set);
		s18_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, PAL_BLACK + 1)) {
		rbx_fail("%s: out of memory", g->set);
		s18_exit();
		return -1;
	}
	s18_reset();
	return 0;
}

static int s18_state(void *ctx, int load)
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
	rc |= rbx_state_io(ctx, "s18", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= rbx_state_io(ctx, "zirq", &s_zirq, sizeof(s_zirq));
	rc |= rbx_state_io(ctx, "rf5c68", &s_rf, (uint32_t)RF5C68_STATE_SIZE);
	rc |= rbx_state_io(ctx, "vdp", &s_vdp, (uint32_t)S315_5313_STATE_SIZE);
	rc |= rbx_state_io(ctx, "vdppal", s_vdp_pal, sizeof(s_vdp_pal));
	if (s_ddcrew4p)
		rc |= rbx_state_io(ctx, "i8255", (uint8_t *)&s_ppi + I8255_STATE_OFFSET, (uint32_t)I8255_STATE_SIZE);
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
	rc |= rbx_ym_state(1, ctx, load);
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
		s_z80 = z;
		update_mapping();
		z_bank();
		fd_update();
		if (s_fd1094_on) {
			rbx_m68k_cmpild_hook = fd_cmpild;
			rbx_m68k_rte_hook = fd_rte;
			rbx_m68k_irqack_hook = fd_irqack;
		}
		s_vdp_pal_dirty = 1;
		s_pal_valid = 0;
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int s18_save(void *ctx) { return s18_state(ctx, 0); }
static int s18_load(void *ctx) { return s18_state(ctx, 1); }

const rbx_driver rbx_drv_segas18 = { "Sega System 18", s18_init, s18_exit, s18_reset, s18_frame, s18_save,
                                     s18_load, s18_supported, s18_render, s18_render_help };
