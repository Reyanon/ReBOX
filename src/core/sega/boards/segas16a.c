/*  Sega System 16A. Ported from MAME src/mame/sega/segas16a.cpp, segaic16.cpp, sega16sp.cpp (BSD-3-Clause,
    copyright-holders: Aaron Giles, Charles MacDonald, Nicola Salmoria, Mirko Buffoni, Thierry Lescot, David Haywood).
    A 68000 at 10 MHz (a fixed map: the ROMs, the tile / text RAM, the sprite RAM, the palette, the I/O with an 8255,
    the work RAM), the 16A tilemaps (8 pages of 64x32 8x8 tiles, two layers of 2x2 pages with row and column scroll
    switched by the 8255, the text layer), the 16A sprites (lines of 4-bit pixels), a palette of 2048 colours with
    shadow / hilight. Sound: a Z80 at 4 MHz woken by the 8255's handshake (its NMI), a YM2151 and a uPD7751 (an 8048
    playing samples from ROM through an 8243 and a DAC). An i8751 on some boards. 342x262, 60 Hz, 320x224 shown.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../rbx_core.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/mcs48/mcs48.h"
#include "../../cpu/mcs51/mcs51.h"
#include "../../machine/i8255.h"
#include "../../sound/rbx_ym.h"
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
#define Z80_CLOCK 4000000
#define YM_CLOCK 4000000
#define N7751_CYCLES 400000      // (6 MHz / 15)
#define FRAME_MHZ 60000
#define LINES 262
#define VIS_W 320
#define VIS_H 224

// ---- ROMs ----
static uint8_t *s_rom, *s_tiles, *s_spr, *s_zrom, *s_n7751rom, *s_n7751data;
static uint32_t s_rom_size, s_tiles_size, s_spr_size, s_zrom_size, s_n7751_size, s_n7751data_size;
static int s_fd1094_on, s_has7751;
static fd1094_t s_fd1094;
static uint16_t *s_fd1089_op;
static uint8_t *s_zop;
static int s_port_service, s_port_p1, s_port_unused, s_port_p2, s_port_dsw1, s_port_dsw2, s_port_p3, s_port_p4;
static int s_port_mj[6];
// the custom I/O of an init (m_custom_io_r)
enum { IO_STD, IO_PASSSHT, IO_SDI, IO_SJRYUKO };
static int s_io;
static uint8_t s_tb[4];   // (sdi: ANALOGX1, ANALOGY1, ANALOGX2, ANALOGY2; saved)

// ---- the state (saved) ----
static struct segas16a_state {   // (on the heap while the game runs, ADR-040)
	uint16_t workram[0x2000];   // 16 KB
	uint16_t spriteram[0x400];
	uint16_t paletteram[0x800];
	uint16_t tileram[0x4000];
	uint16_t textram[0x800];
	uint8_t video_control, mcu_control, to_sound, n7751_command;
	uint32_t n7751_rom_address;
	uint8_t read_port, mj_input_num, display_enable, colscroll, rowscroll, nmi, n7751_reset, n7751_irq;
	uint8_t i8243_p2out, i8243_p2, i8243_opcode, i8243_prog, i8243_p[4];
	uint8_t dac, reset_held, pad[2];
	uint64_t cpu_frac, z80_frac, ym_frac, n7751_frac, mcu_frac;
	int32_t n7751_left;
	uint32_t frame;
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)

static z80 s_z80;
static uint8_t s_zram[0x800];
static const uint8_t *s_zmap[256];
static mcs48 s_n7751;
static i8255_t s_ppi;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static int s_zirq;   // (the Z80's IRQ line: not wired)

static void z_nmi_line(int state);

// ---- the 8255 (ports: A the sound latch, B misc_control_w, C tilemap_sound_w) ----
static void ppi_out_pa(void *u, uint8_t v)
{
	(void)u;
	b.to_sound = v;
}
static void mcu_int1(int state);
static void ppi_out_pb(void *u, uint8_t data)
{
	(void)u;
	b.video_control = data;
	mcu_int1(!(data & 0x40));
	b.display_enable = (data >> 4) & 1;
}
static void ppi_out_pc(void *u, uint8_t data)
{
	(void)u;
	z_nmi_line(!(data & 0x80));
	b.colscroll = !(data & 0x04);
	b.rowscroll = !(data & 0x02);
}

// ---- the main CPU (system16a_map) ----
static uint16_t open_bus(void);
static uint16_t io_r(uint32_t offset)   // standard_io_r and the inits' custom ones
{
	offset &= 0x3fff / 2;
	if ((offset & (0x3000 / 2)) == 0x1000 / 2)
		switch (s_io) {
		case IO_PASSSHT:
			if ((offset & 3) == 0)
				b.read_port = 0;
			else if ((offset & 3) == 1) {
				const int ports[4] = { s_port_p1, s_port_p2, s_port_p3, s_port_p4 };
				const int p = ports[(b.read_port++) & 3];
				return p >= 0 ? (uint16_t)(rbx_port_read(p) & 0xff) : 0xffff;   // (8-bit ports: the bits no field covers read 0)
			}
			break;
		case IO_SDI:
			if ((offset & 3) == 1)
				return s_tb[(b.video_control & 4) ? 1 : 0];
			if ((offset & 3) == 3)
				return s_tb[(b.video_control & 4) ? 3 : 2];
			break;
		case IO_SJRYUKO:
			if ((offset & 3) == 1 || (offset & 3) == 2) {
				const int p = s_port_mj[b.mj_input_num];
				const uint16_t v = p >= 0 ? (uint16_t)(rbx_port_read(p) & 0xff) : 0xff;
				if ((offset & 3) == 1)
					return v != 0xff ? (uint16_t)(0xff & ~(1 << b.mj_input_num)) : 0xff;
				return v;
			}
			break;
		}
	switch (offset & (0x3000 / 2)) {
	case 0x0000 / 2:
		return i8255_read(&s_ppi, (int)(offset & 3));
	case 0x1000 / 2: {
		const int ports[4] = { s_port_service, s_port_p1, s_port_unused, s_port_p2 };
		const int p = ports[offset & 3];
		return p >= 0 ? (uint16_t)(rbx_port_read(p) & 0xff) : 0xffff;   // (8-bit ports: the bits no field covers read 0)
	}
	case 0x2000 / 2: {
		const int p = (offset & 1) ? s_port_dsw2 : s_port_dsw1;
		return p >= 0 ? (uint16_t)(rbx_port_read(p) & 0xff) : 0xffff;   // (8-bit ports: the bits no field covers read 0)
	}
	}
	return 0xffff;
}
static void io_w(uint32_t offset, uint16_t data, uint16_t mask)
{
	offset &= 0x3fff / 2;
	if ((offset & (0x3000 / 2)) == 0 && (mask & 0xff)) {
		const uint8_t old = b.video_control;
		i8255_write(&s_ppi, (int)(offset & 3), (uint8_t)data);
		if (s_io == IO_SJRYUKO && ((old ^ b.video_control) & 4) && (b.video_control & 4))   // sjryuko_lamp_changed_w
			b.mj_input_num = (uint8_t)((b.mj_input_num + 1) % 6);
	}
}

// the regions behind handlers: the sprite RAM (2 KB mirrored in its 4 KB page), the I/O, the watchdog
static int area(uint32_t a)
{
	if ((a & ~0x3bf800u) >= 0x440000 && (a & ~0x3bf800u) <= 0x4407ff) return 1;
	if ((a & ~0x39c000u) >= 0xc40000 && (a & ~0x39c000u) <= 0xc43fff) return 2;
	if (a >= 0xc60000 && a <= 0xc6ffff) return 3;
	return 0;
}
static uint16_t main_r16(uint32_t a)
{
	a &= 0xfffffe;
	switch (area(a)) {
	case 1: return b.spriteram[(a & 0x7ff) >> 1];
	case 2: return io_r((a & 0x3fff) >> 1);
	case 3: return 0xffff;
	}
	return 0xffff;
}
static uint8_t main_r8(uint32_t a)
{
	const uint16_t w = main_r16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}
static void main_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xfffffe;
	switch (area(a)) {
	case 1: {
		uint16_t *p = &b.spriteram[(a & 0x7ff) >> 1];
		*p = (uint16_t)((v & mask) | (*p & ~mask));
		break;
	}
	case 2: io_w((a & 0x3fff) >> 1, v, mask); break;
	}
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

// the page table: the map's entries in order (the later ones over the earlier), the handlers' pages left NULL
static void build_map(void)
{
	const uint32_t ps = 1u << M68K_PAGE_BITS;
	for (uint32_t p = 0; p < M68K_PAGES; p++) {
		const uint32_t a = p * ps;
		uint8_t *rd = NULL, *wr = NULL;
		if ((a & ~0x380000u) < 0x40000) {
			const uint32_t o = a & 0x3ffff;
			rd = o + ps <= s_rom_size ? s_rom + o : NULL;
		}
		if ((a & ~0xb88000u) >= 0x400000 && (a & ~0xb88000u) <= 0x407fff)
			rd = wr = (uint8_t *)b.tileram + (a & 0x7fff);
		if ((a & ~0xb8f000u) >= 0x410000 && (a & ~0xb8f000u) <= 0x410fff)
			rd = wr = (uint8_t *)b.textram;
		if ((a & ~0x3bf000u) >= 0x840000 && (a & ~0x3bf000u) <= 0x840fff)
			rd = wr = (uint8_t *)b.paletteram;
		if ((a & ~0x38c000u) >= 0xc70000 && (a & ~0x38c000u) <= 0xc73fff)
			rd = wr = (uint8_t *)b.workram + (a & 0x3fff);
		if (area(a))   // (the sprites, the I/O, the watchdog: handlers; none of their pages meets the work RAM's)
			rd = wr = NULL;
		m68k_bus.rd[p] = rd;
		m68k_bus.wr[p] = wr;
	}
}

// ---- the Z80 (sound_map, sound_portmap) ----
static void z_nmi_line(int state)
{
	if (state && !b.nmi)
		z80_gen_nmi(&s_z80);
	b.nmi = (uint8_t)state;
}
static uint8_t sound_data_r(void)
{
	i8255_pc6_w(&s_ppi, 0);   // (the /ACK)
	return b.to_sound;
}
static uint8_t z_read(void *u, uint16_t a)
{
	(void)u;
	const uint8_t *p = s_zmap[a >> 8];
	if (p)
		return p[a & 0xff];
	if (a >= 0xf800)
		return s_zram[a - 0xf800];
	if (a == 0xe800)
		return sound_data_r();
	return 0xff;
}
static void z_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xf800)
		s_zram[a - 0xf800] = v;
}
static void n7751_command_w(uint8_t data)
{
	const int numroms = (int)(s_n7751data_size / 0x8000);
	b.n7751_rom_address &= 0x3fff;
	b.n7751_rom_address |= (uint32_t)(data & 0x01) << 14;
	if (!(data & 0x04) && numroms >= 2) b.n7751_rom_address |= 0x08000;
	if (!(data & 0x08) && numroms >= 3) b.n7751_rom_address |= 0x10000;
	if (!(data & 0x10) && numroms >= 4) b.n7751_rom_address |= 0x18000;
	b.n7751_command = data >> 5;
}
static uint8_t z_in(z80 *z, uint8_t p)
{
	(void)z;
	switch (p & 0xc0) {
	case 0x00: return rbx_ym_read(0, p & 1);
	case 0xc0: return sound_data_r();
	}
	return 0xff;
}
static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	switch (p & 0xc0) {
	case 0x00: rbx_ym_write(0, p & 1, v); break;
	case 0x80:
		if (s_has7751)
			n7751_command_w(v);
		break;
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

// ---- the uPD7751 (an 8048: BUS the sample ROM, P1 the DAC, P2 the command and the 8243, PROG the 8243) ----
static void n7751_control_w(uint8_t data)   // the YM2151's CT pins: /RESET, /IRQ
{
	const int reset = !(data & 0x01);
	if (reset && !b.n7751_reset)
		mcs48_reset(&s_n7751);
	b.n7751_reset = (uint8_t)reset;
	b.n7751_irq = !(data & 0x02);
	mcs48_set_irq(&s_n7751, b.n7751_irq);
}
static uint8_t n7751_bus_r(void *u)
{
	(void)u;
	return b.n7751_rom_address < s_n7751data_size ? s_n7751data[b.n7751_rom_address] : 0xff;
}
static uint8_t n7751_port_r(void *u, int port)
{
	(void)u;
	if (port == 2)
		return (uint8_t)(0x80 | ((b.n7751_command & 0x07) << 4) | (b.i8243_p2out & 0x0f));
	return 0xff;
}
static void n7751_port_w(void *u, int port, uint8_t v)
{
	(void)u;
	if (port == 1)
		b.dac = v;
	else if (port == 2)
		b.i8243_p2 = v & 0x0f;
}
static void i8243_output(int which)   // upd7751_rom_offset_w<4 which>
{
	const int shift = which * 4;
	const uint32_t mask = (0xfu << shift) & 0x3fff;
	b.n7751_rom_address = (b.n7751_rom_address & ~mask) | (((uint32_t)b.i8243_p[which] << shift) & mask);
}
static void n7751_prog_w(void *u, int state)   // i8243_device::prog_w
{
	(void)u;
	if (b.i8243_prog && !state) {
		b.i8243_opcode = b.i8243_p2;
		b.i8243_p2out = (b.i8243_opcode >> 2) == 0 ? (uint8_t)(b.i8243_p[b.i8243_opcode & 3] & 0x0f) : 0x0f;
	} else if (!b.i8243_prog && state) {
		const int w = b.i8243_opcode & 3;
		switch (b.i8243_opcode >> 2) {
		case 0: b.i8243_p2out = 0x0f; break;
		case 1: b.i8243_p[w] = b.i8243_p2 & 0x0f; i8243_output(w); break;
		case 2: b.i8243_p[w] |= b.i8243_p2 & 0x0f; i8243_output(w); break;
		case 3: b.i8243_p[w] &= b.i8243_p2 & 0x0f; i8243_output(w); break;
		}
	}
	b.i8243_prog = (uint8_t)state;
}
static void n7751_setup(void)
{
	mcs48_init(&s_n7751, MCS48_I8048);
	s_n7751.rom = s_n7751rom;
	s_n7751.rom_mask = 0x3ff;
	s_n7751.bus_r = n7751_bus_r;
	s_n7751.port_r = n7751_port_r;
	s_n7751.port_w = n7751_port_w;
	s_n7751.prog_w = n7751_prog_w;
}

// ---- the i8751 (system16a_i8751): P1 the control lines, its data space the 68000's ----
static int s_mcu_on, s_mcu_sim;   // (s_mcu_sim: dumpmtmt's simulation)
static mcs51_t s_mcu;
static const uint8_t *s_mcu_rom;
static uint32_t s_mcu_size;
static void mcu_int1(int state)
{
	if (s_mcu_on)
		mcs51_set_input(&s_mcu, MCS51_INT1_LINE, state);
}
static uint8_t mcu_xdata_r(void *u, uint16_t offset)
{
	(void)u;
	switch ((b.mcu_control >> 3) & 7) {
	case 0:
		if (offset < 0x3fff)
			return 0x00;   // (the watchdog: the MCU data space unmapped value)
		if (offset >= 0x4000 && offset < 0x8000)
			return ((const uint8_t *)b.workram)[(0x0001 ^ (offset & 0x3fff)) ^ M68K_BYTE_XOR];
		if (offset >= 0x8000 && offset < 0xc000)
			return main_r8(0xc40001 ^ (offset & 0x3fff));
		return 0xff;
	case 1:
		if (offset < 0x8000)
			return ((const uint8_t *)b.tileram)[(0x0001 ^ (offset & 0x7fff)) ^ M68K_BYTE_XOR];
		if (offset < 0x9000)
			return ((const uint8_t *)b.textram)[(0x0001 ^ (offset & 0xfff)) ^ M68K_BYTE_XOR];
		return 0xff;
	case 3: return ((const uint8_t *)b.paletteram)[(0x0001 ^ (offset & 0xfff)) ^ M68K_BYTE_XOR];
	case 5: return s_rom[0x00000 + offset];   // (memregion base(): the words in host order, as here)
	case 6: return s_rom[0x10000 + offset];
	case 7: return s_rom[0x20000 + offset];
	}
	return 0xff;
}
static void mcu_xdata_w(void *u, uint16_t offset, uint8_t data)
{
	(void)u;
	switch ((b.mcu_control >> 3) & 7) {
	case 0:
		if (offset >= 0x4000 && offset < 0x8000)
			((uint8_t *)b.workram)[(0x0001 ^ (offset & 0x3fff)) ^ M68K_BYTE_XOR] = data;
		else if (offset >= 0x8000 && offset < 0xc000)
			main_w8(0xc40001 ^ (offset & 0x3fff), data);
		break;
	case 1:
		if (offset < 0x8000)
			((uint8_t *)b.tileram)[(0x0001 ^ (offset & 0x7fff)) ^ M68K_BYTE_XOR] = data;
		else if (offset < 0x9000)
			((uint8_t *)b.textram)[(0x0001 ^ (offset & 0xfff)) ^ M68K_BYTE_XOR] = data;
		break;
	case 3:
		((uint8_t *)b.paletteram)[(0x0001 ^ (offset & 0xfff)) ^ M68K_BYTE_XOR] = data;
		break;
	}
}
static void mcu_port_out(void *u, int port, uint8_t data)   // mcu_control_w
{
	(void)u;
	if (port != 1)
		return;
	const int held = (data & 0x40) != 0;
	if (held && !b.reset_held)
		m68k_pulse_reset();
	b.reset_held = (uint8_t)held;
	for (int irqline = 1; irqline <= 7; irqline++)
		m68k_bus_set_line(irqline, (~data & 7) == irqline);
	if (data & 0x40)
		b.display_enable = 1;
	b.mcu_control = data;
}
static uint8_t mcu_port_in(void *u, int port)
{
	(void)u;
	(void)port;
	return 0xff;
}
static void dumpmtmt_sim(void)   // dumpmtmt_i8751_sim
{
	uint8_t flag = (uint8_t)(b.workram[0x200 / 2] >> 8), tick = (uint8_t)b.workram[0x200 / 2];
	uint8_t sec = (uint8_t)(b.workram[0x202 / 2] >> 8), min = (uint8_t)b.workram[0x202 / 2];
	m68k_bus_hold_irq(4);
	if (tick == 0 && sec == 0 && min == 0)
		flag = 1;
	else if (tick != 0)
		tick--;
	else {
		tick = 0x40;
		if (sec != 0)
			sec = (uint8_t)((sec & 0xf) ? sec - 1 : (sec - 0x10) + 9);
		else {
			sec = 0x59;
			if (min != 0)
				min--;
			else {
				flag = 1;
				tick = sec = min = 0;
			}
		}
	}
	b.workram[0x200 / 2] = (uint16_t)((flag << 8) + tick);
	b.workram[0x202 / 2] = (uint16_t)((sec << 8) + min);
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
	uint16_t tileram[0x4000];
	uint16_t textram[0x800];
	uint16_t spriteram[0x400];
	uint8_t display_enable, flip, colscroll, rowscroll;
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

// one layer's line of the 16A tilemaps (tilemap_16a_draw_layer + draw_virtual_tilemap): col[x] the colour,
// cat[x] the tile category | 2 when the pen is not 0
static void layer_line(int which, int y, uint16_t *col, uint8_t *cat)
{
	const uint16_t *textram = s_snap->textram;
	const uint16_t xscroll = textram[0xff8 / 2 + which] & 0x1ff, yscroll = textram[0xf24 / 2 + which] & 0x0ff;
	uint16_t pages = textram[(s_snap->flip ? 0xe8e / 2 : 0xe9e / 2) - which];
	pages = (uint16_t)(((pages >> 4) & 0x0707) | ((pages << 4) & 0x7070));
	uint16_t effx = s_snap->rowscroll ? (textram[0xf80 / 2 + (y / 8) * 2 + which] & 0x1ff) : xscroll;
	effx = (uint16_t)((0xc8 - effx) & 0x3ff);
	for (int x = 0; x < VIS_W;) {
		int ey = yscroll, n = VIS_W - x;
		if (s_snap->colscroll) {   // (columns of 16 pixels)
			ey = textram[0xf30 / 2 + (x / 16) * 2 + which] & 0x0ff;
			n = 16 - (x & 15);
		}
		const int vy = (y + ey) & 0x1ff, py = vy & 0xff;
		const int pbase = ((vy >> 8) & 1) * 8;
		const int end = x + n;
		while (x < end) {
			const int vx = (x + effx) & 0x3ff, px = vx & 0x1ff;
			const int page = (pages >> (pbase + ((vx >> 9) & 1) * 4)) & 15;
			const uint16_t data = s_snap->tileram[(page & 7) * 64 * 32 + (py >> 3) * 64 + (px >> 3)];
			const uint32_t code = (uint32_t)(((data >> 1) & 0x1000) | (data & 0xfff));
			const uint16_t cb = (uint16_t)(((data >> 5) & 0x7f) * 8);
			const uint8_t c = (uint8_t)((data >> 12) & 1);
			const uint8_t *row = tile_row(code, py & 7);
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

// the sprites of a band (sega_sys16a_sprite_device::draw; the address kept locally)
static void draw_sprites(int y0, int y1, uint16_t (*sbuf)[VIS_W])
{
	for (int y = y0; y < y1; y++)
		for (int x = 0; x < VIS_W; x++)
			sbuf[y - y0][x] = 0xffff;
	const uint16_t *spritebase = (const uint16_t *)s_spr;
	const uint8_t numbanks = (uint8_t)(s_spr_size / 0x10000);
	const uint16_t *ram = s_snap->spriteram;
	for (const uint16_t *data = ram; data < ram + 0x400; data += 8) {
		const int bottom = data[0] >> 8;
		if (bottom > 0xf0)
			break;
		const int top = data[0] & 0xff;
		const int xpos = data[1] & 0x1ff;
		const int pitch = (int16_t)data[2];
		uint16_t addr = data[3];
		const int colpri = (((data[4] >> 8) & 0x3f) << 4) | ((data[4] & 3) << 10);
		int bank = (data[4] >> 4) & 7;
		if (top >= bottom)
			continue;
		if (numbanks)
			bank %= numbanks;
		const uint16_t *sd = spritebase + 0x8000 * bank;
		// (the sprite's lines: y + 1 on the screen, MAME's origin -1)
		for (int y = top; y < bottom; y++) {
			addr = (uint16_t)(addr + pitch);
			const int sy = y + 1;
			if (sy < y0 || sy >= y1)
				continue;
			uint16_t *dest = sbuf[sy - y0];
			int x;
			uint16_t a7;
			if (!(addr & 0x8000)) {
				a7 = (uint16_t)(addr - 1);
				for (x = xpos; ((xpos - x) & 0x1ff) != 1;) {
					const uint16_t pixels = sd[++a7 & 0x7fff];
					int pix = 0;
					for (int sh = 12; sh >= 0; sh -= 4) {
						pix = (pixels >> sh) & 15;
						const int sx = x - 189;
						if (sx >= 0 && sx < VIS_W && pix != 0 && pix != 15)
							dest[sx] = (uint16_t)(colpri | pix);
						x++;
					}
					if (pix == 15)
						break;
				}
			} else {
				a7 = (uint16_t)(addr + 1);
				for (x = xpos; ((xpos - x) & 0x1ff) != 1;) {
					const uint16_t pixels = sd[--a7 & 0x7fff];
					int pix = 0;
					for (int sh = 0; sh <= 12; sh += 4) {
						pix = (pixels >> sh) & 15;
						const int sx = x - 189;
						if (sx >= 0 && sx < VIS_W && pix != 0 && pix != 15)
							dest[sx] = (uint16_t)(colpri | pix);
						x++;
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
		for (int x = 0; x < VIS_W; x++) {
			uint16_t col = bcol[x];
			uint8_t pri = (bcat[x] & 2) ? ((bcat[x] & 1) ? 0x02 : 0x01) : 0;
			if (fcat[x] & 2) {
				col = fcol[x];
				pri |= (fcat[x] & 1) ? 0x04 : 0x02;
			}
			const int tx = x + 192;
			const uint16_t data = text[(tx >> 3) & 63];
			const uint8_t pt = tile_row((uint32_t)(data & 0xff), y & 7)[tx & 7];
			if (pt) {
				col = (uint16_t)(((data >> 8) & 7) * 8 + pt);
				pri |= (data & 0x0800) ? 0x08 : 0x04;
			}
			if (sp[x] != 0xffff) {
				const int priority = sp[x] >> 10;
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

static void s16a_render(void)
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

static int s16a_render_help(int helper)
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
	s_snap->display_enable = b.display_enable;
	s_snap->flip = (b.video_control >> 7) & 1;
	s_snap->colscroll = b.colscroll;
	s_snap->rowscroll = b.rowscroll;
	rbx_video_frame_done();
}

// ---- the frame ----
static void s16a_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	if (s_io == IO_SDI)
		for (int i = 0; i < 4; i++) {   // (ANALOGX1 reversed)
			const int d = rbx_analog_dpad(i / 2 + 1, (i & 1) ? RBX_IPT_TRACKBALL_Y : RBX_IPT_TRACKBALL_X, 4);
			s_tb[i] = (uint8_t)(s_tb[i] + (i == 0 ? -d : d));
		}
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_H) {   // the vblank: drawn, IRQ 4 (or the MCU's INT0 / the simulation)
			vblank_draw();
			if (s_mcu_sim)
				dumpmtmt_sim();
			else if (!s_mcu_on)
				m68k_bus_hold_irq(4);
		}
		if (s_mcu_on)
			mcs51_set_input(&s_mcu, MCS51_INT0_LINE, line >= VIS_H);
		{
			const int mc = rbx_line_cycles(&b.cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES);
			if (!s_mcu_on) {
				if (mc > 0 && !b.reset_held)
					m68k_execute(mc);
			} else {   // (the MCU and the 68000 in quarters of a line: its IRQs and resets seen in time)
				const int uc = rbx_line_cycles(&b.mcu_frac, 8000000 / 12, FRAME_MHZ, LINES);
				for (int q = 0; q < 4; q++) {
					const int m = mc * (q + 1) / 4 - mc * q / 4, u = uc * (q + 1) / 4 - uc * q / 4;
					if (m > 0 && !b.reset_held)
						m68k_execute(m);
					mcs51_execute(&s_mcu, u);
				}
			}
		}
		const int zc = rbx_line_cycles(&b.z80_frac, Z80_CLOCK, FRAME_MHZ, LINES);
		z80_run(&s_z80, s_z80.cyc + (unsigned long)zc, &s_zirq, 0);   // (no IRQ: the NMI only)
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&b.ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
		const int k = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
		if (s_has7751) {
			// the 7751 in steps of the YM's samples: the DAC's level at each
			const int nc = rbx_line_cycles(&b.n7751_frac, N7751_CYCLES, FRAME_MHZ, LINES);
			for (int i = 0; i < k; i++) {
				const int want = nc * (i + 1) / k - nc * i / k;
				b.n7751_left += want;
				if (!b.n7751_reset && b.n7751_left > 0)
					b.n7751_left -= mcs48_execute(&s_n7751, b.n7751_left);
				else if (b.n7751_reset)
					b.n7751_left = 0;
				s_mix[s_mix_n + i] += ((int)b.dac - 128) * 1024 / 10;   // (DAC_8BIT_R2R: v / 256 over -1 .. 1, x 0.4)
			}
			if (!k) {
				b.n7751_left += nc;
				if (!b.n7751_reset && b.n7751_left > 0)
					b.n7751_left -= mcs48_execute(&s_n7751, b.n7751_left);
			}
		}
		s_mix_n += k;
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);   // (the YM's samples, made on the worker, added to the DAC's)
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	b.frame++;
}

static void s16a_reset(void)
{
	memset(&b, 0, offsetof(__typeof__(b), state_end));
	b.i8243_p2out = 0x0f;
	b.i8243_p2 = 0x0f;
	b.i8243_prog = 1;
	memset(s_zram, 0, sizeof(s_zram));
	memset(s_zmap, 0, sizeof(s_zmap));
	memset(s_tb, 0, sizeof(s_tb));
	for (uint32_t p = 0; p < 0x80 && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	z_start();
	rbx_ym_reset(0);
	s_mix_n = 0;
	s_pal_valid = 0;
	memset(&s_ppi, 0, sizeof(s_ppi));
	s_ppi.out_pa = ppi_out_pa;
	s_ppi.out_pb = ppi_out_pb;
	s_ppi.out_pc = ppi_out_pc;
	s_ppi.tri_pa = s_ppi.tri_pb = s_ppi.tri_pc = 0;   // (the tri callbacks set to 0)
	if (s_has7751) {
		n7751_setup();
		mcs48_reset(&s_n7751);
	}
	if (s_mcu_on) {
		mcs51_init(&s_mcu, s_mcu_rom, s_mcu_size);
		s_mcu.xdata_r = mcu_xdata_r;
		s_mcu.xdata_w = mcu_xdata_w;
		s_mcu.port_in = mcu_port_in;
		s_mcu.port_out = mcu_port_out;
		mcs51_reset(&s_mcu);
	}
	m68k_bus_irq_reset();
	i8255_reset(&s_ppi);
	if (s_fd1094_on) {
		fd1094_change_state(&s_fd1094, FD1094_STATE_RESET);
		fd_update();
		rbx_m68k_cmpild_hook = fd_cmpild;
		rbx_m68k_rte_hook = fd_rte;
		rbx_m68k_irqack_hook = fd_irqack;
	}
	m68k_pulse_reset();
}

static void s16a_exit(void)
{
	fd1094_exit(&s_fd1094);
	free(s_zop);
	s_zop = NULL;
	free(s_fd1089_op);
	s_fd1089_op = NULL;
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	free(s_snap);
	s_snap = NULL;
	free(s_tdec);
	s_tdec = NULL;
	free(s_bp);   // (last: the render thread and the saves are done with it)
	s_bp = NULL;
	free(s_sbuf_hp);
	s_sbuf_hp = NULL;
}

static int s16a_supported(const char *machine)
{
	static const char *const ok[] = { "system16a:", "system16a_no7751:", "system16a_fd1089a:", "system16a_fd1089b:",
	                                  "system16a_fd1094:", "system16a_i8751:", "system16a_fd1089a_no7751:",
	                                  "system16a_fd1089b_no7751:", "system16a_fd1094_no7751:", NULL };
	for (int i = 0; ok[i]; i++)
		if (!strncmp(machine, ok[i], strlen(ok[i])))
			return !strstr(machine, ":mjleague");
	return 0;
}

static int s16a_init(const rbx_game *g)
{
	if (!s_sbuf_hp && !(s_sbuf_hp = calloc(1, sizeof(*s_sbuf_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_bp && !(s_bp = (struct segas16a_state *)calloc(1, sizeof(*s_bp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s16a_supported(g->machine)) {
		rbx_fail("%s: machine %s not emulated", g->set, g->machine);
		return -1;
	}
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_tiles = rbx_region("gfx1", &s_tiles_size);
	s_spr = rbx_region("sprites", &s_spr_size);
	s_zrom = rbx_region("soundcpu", &s_zrom_size);
	if (!s_rom || !s_tiles || !s_spr || !s_zrom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_has7751 = !strstr(g->machine, "no7751");
	s_n7751rom = rbx_region("n7751", &s_n7751_size);
	if (!s_n7751rom)
		s_n7751rom = rbx_region("upd7751", &s_n7751_size);
	s_n7751data = rbx_region("upd7751data", &s_n7751data_size);
	if (s_has7751 && (!s_n7751rom || !s_n7751data))
		s_has7751 = 0;
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_bus_swap16(s_spr, s_spr_size);
	s_fd1094_on = !strncmp(g->machine, "system16a_fd1094", 16);
	free(s_fd1089_op);
	s_fd1089_op = NULL;
	if (!strncmp(g->machine, "system16a_fd1089", 16)) {
		uint32_t ksz;
		const uint8_t *key = rbx_region("maincpu:key", &ksz);
		s_fd1089_op = (uint16_t *)malloc(s_rom_size);
		if (!key || ksz < 0x2000 || !s_fd1089_op) {
			rbx_fail("%s: no FD1089 key", g->set);
			return -1;
		}
		fd1089_decrypt(g->machine[16] == 'b', (uint16_t *)s_rom, s_rom_size, key, s_fd1089_op);
	}
	s_mcu_sim = strstr(g->machine, ":dumpmtmt") != NULL;
	s_mcu_rom = rbx_region("mcu", &s_mcu_size);
	s_mcu_on = !strncmp(g->machine, "system16a_i8751", 15) && !s_mcu_sim && s_mcu_rom;
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
	build_map();
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
	for (int i = 0; i < 6; i++) {
		char tag[4] = { 'M', 'J', (char)('0' + i), 0 };
		s_port_mj[i] = rbx_port_find(tag);
	}
	s_io = strstr(g->machine, ":passsht16a") ? IO_PASSSHT : strstr(g->machine, ":sdi") ? IO_SDI
	     : strstr(g->machine, ":sjryukoa") ? IO_SJRYUKO : IO_STD;
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = s_has7751 ? 110 : 128;   // (0.43, or 0.5 on the boards without the 7751)
	yc.port_a_write = s_has7751 ? n7751_control_w : NULL;
	if (!s_snap || !s_tdec || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		s16a_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 0x1001)) {
		rbx_fail("%s: out of memory", g->set);
		s16a_exit();
		return -1;
	}
	s16a_reset();
	return 0;
}

static int s16a_state(void *ctx, int load)
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
	rc |= rbx_state_io(ctx, "s16a", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= rbx_state_io(ctx, "i8255", (uint8_t *)&s_ppi + I8255_STATE_OFFSET, (uint32_t)I8255_STATE_SIZE);
	if (s_has7751)
		rc |= rbx_state_io(ctx, "upd7751", (uint8_t *)&s_n7751 + MCS48_STATE_OFFSET, (uint32_t)MCS48_STATE_SIZE);
	if (s_io == IO_SDI)
		rc |= rbx_state_io(ctx, "trackball", s_tb, sizeof(s_tb));
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

static int s16a_save(void *ctx) { return s16a_state(ctx, 0); }
static int s16a_load(void *ctx) { return s16a_state(ctx, 1); }

const rbx_driver rbx_drv_segas16a = { "Sega System 16A", s16a_init, s16a_exit, s16a_reset, s16a_frame, s16a_save,
                                      s16a_load, s16a_supported, s16a_render, s16a_render_help };
