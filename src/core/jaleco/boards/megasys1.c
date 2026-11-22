/*  Jaleco Mega System 1 (types A, B, C, D, Z). Ported from MAME 0.289 src/mame/jaleco/megasys1.cpp, megasys1.h,
    megasys1_v.cpp, ms1_tmap.cpp (tilemaps), ms1_gatearray.cpp (D65006 / GS-88000 ROM decryption and overlay)
    (BSD-3-Clause, copyright-holders: Luca Elia; ms1_tmap: Luca Elia, AJR; gate array: Luca Elia, David Haywood).
    A 68000 (6 / 8 / 12 MHz) with a second 68000 at 7 MHz for the sound (YM2151 + two OKIs; type Z: a Z80 with a
    YM2203, type D: one OKI on the main CPU); up to three tilemap chips (8x8 or 16x16 tiles, 512 to 4096 pixel wide
    layers), 128 sprites placed through the object RAM into a sprite frame buffer (kept for the trail effect), the
    layer order from a PROM (MAME's priority_create), a palette of 1024 RRRRGGGGBBBBRGBx. Types B and C read the
    inputs through a TMP91640 MCU (or its simulation), type D through a protection latch (Peek-a-Boo!).
    The registers that change mid-frame (scrolls, layer and sprite controls) are kept for each line, as MAME's
    partial updates draw them. Not emulated: p47bl (its extra Z80 and MSM5205s).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/m68k_multi.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/tlcs90/tlcs90.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/decoband.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

enum { T_A, T_B, T_C, T_D, T_Z, T_BBL, T_BMONK };
enum { GA_NONE, GA_D65006, GA_GS88000, GA_UNK };

typedef struct {
	const char *name;
	int type, ga, iganinju, soldam, kickoffb, iomcu, iosim;
} machine_t;

static const machine_t s_machines[] = {
	{ "system_A", T_A, GA_NONE, 0, 0, 0, 0, 0 },
	{ "system_A_kickoffb", T_A, GA_NONE, 0, 0, 1, 0, 0 },
	{ "system_A_d65006", T_A, GA_D65006, 0, 0, 0, 0, 0 },
	{ "system_A_d65006_iganinju", T_A, GA_D65006, 1, 0, 0, 0, 0 },
	{ "system_A_iganinju", T_A, GA_NONE, 1, 0, 0, 0, 0 },
	{ "system_A_gs88000", T_A, GA_GS88000, 0, 0, 0, 0, 0 },
	{ "system_A_unkarray", T_A, GA_UNK, 0, 0, 0, 0, 0 },
	{ "system_A_gs88000_soldam", T_A, GA_GS88000, 0, 1, 0, 0, 0 },
	{ "system_A_d65006_soldam", T_A, GA_D65006, 0, 1, 0, 0, 0 },
	{ "system_B_iomcu", T_B, GA_NONE, 0, 0, 0, 1, 0 },
	{ "system_B_hayaosi1", T_B, GA_NONE, 0, 0, 0, 0, 1 },
	{ "system_B_monkelf", T_BMONK, GA_NONE, 0, 0, 0, 0, 0 },
	{ "system_Bbl", T_BBL, GA_NONE, 0, 0, 0, 0, 0 },
	{ "system_C_iomcu", T_C, GA_NONE, 0, 0, 0, 1, 0 },
	{ "system_C_iosim", T_C, GA_NONE, 0, 0, 0, 0, 1 },
	{ "system_D", T_D, GA_NONE, 0, 0, 0, 0, 0 },
	{ "system_Z", T_Z, GA_NONE, 0, 0, 0, 0, 0 },
};

#define VIS_Y0 16
#define VIS_Y1 239
#define VIS_H (VIS_Y1 - VIS_Y0 + 1)
#define SND_CLOCK 7000000
#define SLICES 2

static const machine_t *s_mc;
static int s_type, s_hachoo, s_nscroll, s_iomcu;   // (s_iomcu: the MCU runs, its dump present)
static uint32_t s_cpu_clock, s_fps_mhz;
static int s_lines;
static uint8_t *s_rom, *s_srom, *s_scroll_gfx[3], *s_spr_gfx, *s_oki_rom[2], *s_prom, *s_mcu_rom;
static uint32_t s_rom_size, s_srom_size, s_scroll_size[3], s_spr_size, s_oki_size[2];
static uint32_t s_scroll_n[3], s_spr_n;
static int s_port_sys, s_port_p1, s_port_p2, s_port_dsw, s_port_dsw1, s_port_dsw2;
static const uint8_t *s_ip_seq;   // the input MCU simulation's codes (hayaosi1, chimeraba)
static uint32_t s_layers_order[16];
static okim6295_t s_oki[2];
static int s_noki;
static z80 s_z80;
static tlcs90 s_mcu;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static uint32_t s_out_rate;

static const uint8_t hayaosi1_seq[7] = { 0x51, 0x52, 0x53, 0x54, 0x55, 0xfc, 0x06 };
static const uint8_t chimeraba_seq[7] = { 0x56, 0x52, 0x53, 0x55, 0x54, 0xfa, 0x06 };
static const uint16_t d65006_seq[5] = { 0x0000, 0x0055, 0x00aa, 0x00ff, 0x835d };
static const uint16_t gs88000_seq[5] = { 0x00ff, 0x0055, 0x00aa, 0x0000, 0x889e };

// ---- state ----
static struct megasys1_state {   // (on the heap while the game runs, ADR-040)
	uint16_t ram[0x10000 / 2];
	uint16_t sram[0x10000 / 2];        // the sound 68000's RAM
	uint16_t pal[0x400];
	uint16_t objram[0x2000 / 2];
	uint16_t scrollram[3][0x4000 / 2];
	uint16_t obj_buf[0x1000], obj_buf2[0x1000], spr_buf[0x1000], spr_buf2[0x1000];
	uint16_t scrollx[3], scrolly[3], scroll_flag[3];
	uint16_t active_layers, sprite_flag, sprite_bank, screen_flag;
	uint16_t latch[2];                 // to the sound CPU, from it
	uint16_t ga_hs_ram[8];
	uint8_t ga_hs;
	uint16_t ip_latched;
	uint8_t mcu_input, mcu_io, mcu_int0;
	uint16_t prot_val;                 // (Peek-a-Boo!)
	uint8_t okibank, snd_reset, zlatch;
	uint8_t zram[0x800];
	uint64_t cpu_frac, snd_frac, mcu_frac, out_frac, ym_frac;
	uint32_t oki_acc[2];
	int32_t oki_hold[2];
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)

// the per-line registers (MAME draws them by partial updates)
typedef struct {
	uint16_t scrollx[3], scrolly[3], scroll_flag[3];
	uint16_t active_layers, sprite_flag, sprite_bank, screen_flag;
} line_regs;

static struct {
	uint16_t scrollram[3][0x4000 / 2];
	uint16_t pal[0x400];
	uint16_t obj[0x1000], spr[0x1000];   // the sprites two frames back
	uint16_t zspr[0x800];                // (type Z: Sprite Data in the work RAM)
	line_regs line[VIS_H];
} *s_snap;
static uint16_t (*s_sprbmp_hp)[256 * 256];
#define s_sprbmp (*s_sprbmp_hp)
static line_regs s_line_cur[VIS_H];        // (the emulation side's, copied at the vblank)      // (the sprite frame buffer, kept between frames: the drawing side's)

static inline int pal5(int v) { return (v << 3) | (v >> 2); }

// ---- the sound side ----
static uint8_t oki_read_d(void *user, uint32_t a)
{
	(void)user;
	if (a < 0x20000)
		return a < s_oki_size[0] ? s_oki_rom[0][a] : 0;
	// (init_peekaboo: banks 0-6 at 0x20000 + n x 0x20000, bank 7 the first of them)
	const uint32_t o = 0x20000 + ((b.okibank & 7) == 7 ? 0 : (uint32_t)(b.okibank & 7) * 0x20000) + (a - 0x20000);
	return o < s_oki_size[0] ? s_oki_rom[0][o] : 0;
}

static uint8_t oki_status(int c)
{
	if (!s_hachoo)
		return 0;   // (MAME m_ignore_oki_status: the 'protection')
	return oki_read(&s_oki[c]);
}

static void ym_irq(int state)   // (sound_irq: the YM's IRQ holds the sound 68000's level 4)
{
	if (state)
		m68k_multi_hold_irq(1, 4);
}
static int s_zirq;
static void ym_irq_z(int state) { s_zirq = state; }

// the sound 68000 (types A, B, C)
static uint16_t snd_r16(uint32_t a)
{
	a &= 0xfffffe;
	if (a == 0x040000 || (s_type != T_A && a == 0x060000))
		return b.latch[0];
	if (a >= 0x080000 && a < 0x080004)
		return rbx_ym_read(0, (a >> 1) & 1);
	if (a == 0x0a0000)
		return oki_status(0);
	if (a == 0x0c0000)
		return s_noki > 1 ? oki_status(1) : 0;
	return 0xffff;
}
static uint8_t snd_r8(uint32_t a)
{
	const uint16_t w = snd_r16(a);
	return (a & 1) ? (uint8_t)w : (uint8_t)(w >> 8);
}
static void snd_w16m(uint32_t a, uint16_t d, uint16_t mask)
{
	a &= 0xfffffe;
	if (a == 0x060000 || (s_type != T_A && a == 0x040000))
		b.latch[1] = (uint16_t)((b.latch[1] & ~mask) | (d & mask));
	else if (a >= 0x080000 && a < 0x080004) {
		if (mask & 0xff)
			rbx_ym_write(0, (a >> 1) & 1, (uint8_t)d);
	} else if (a >= 0x0a0000 && a < 0x0a0004)
		oki_write(&s_oki[0], (mask & 0xff) ? (uint8_t)d : (uint8_t)(d >> 8));   // (byte smearing: jitsupro)
	else if (a >= 0x0c0000 && a < 0x0c0004 && s_noki > 1)
		oki_write(&s_oki[1], (mask & 0xff) ? (uint8_t)d : (uint8_t)(d >> 8));
}
static void snd_w8(uint32_t a, uint8_t v) { snd_w16m(a, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00); }
static void snd_w16(uint32_t a, uint16_t v) { snd_w16m(a, v, 0xffff); }

// the Z80 (type Z)
static uint8_t z_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x4000)
		return a < s_srom_size ? s_srom[a] : 0xff;
	if (a >= 0xc000 && a < 0xc800)
		return b.zram[a - 0xc000];
	if (a == 0xe000)
		return (uint8_t)b.latch[0];
	return 0xff;
}
static void z_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xc800)
		b.zram[a - 0xc000] = v;
}
static uint8_t z_in(z80 *z, uint8_t port) { (void)z; return port < 2 ? rbx_ym_read(0, port) : 0xff; }
static void z_out(z80 *z, uint8_t port, uint8_t v) { (void)z; if (port < 2) rbx_ym_write(0, port, v); }

// ---- the input MCU (types B, C: TMP91640) ----
static uint8_t mcu_read(void *u, uint32_t a)
{
	(void)u;
	switch ((a >> 16) & 0xf) {   // mcu_capture_inputs_r
	case 1: return (uint8_t)rbx_port_read(s_port_p1);
	case 2: return (uint8_t)rbx_port_read(s_port_p2);
	case 3: return (uint8_t)rbx_port_read(s_port_dsw1);
	case 4: return (uint8_t)rbx_port_read(s_port_dsw2);
	case 5: return (uint8_t)rbx_port_read(s_port_sys);
	case 7: m68k_multi_hold_irq(0, 2); return 0;
	default: return 0;
	}
}
static uint8_t mcu_port_r(void *u, int port) { (void)u; return port == 1 ? b.mcu_input : 0xff; }
static void mcu_port_w(void *u, int port, uint8_t v) { (void)u; if (port == 2) b.mcu_io = v; }

// ---- the main CPU ----
static void screen_flag_w(uint16_t d, uint16_t mask)
{
	b.screen_flag = (uint16_t)((b.screen_flag & ~mask) | (d & mask));
	const int rst = (b.screen_flag >> 4) & 1;
	if (s_type == T_D || s_type == T_BBL)
		return;
	if (rst && !b.snd_reset) {
		if (s_type == T_Z)
			z80_init(&s_z80), s_z80.read_byte = z_read, s_z80.write_byte = z_write, s_z80.port_in = z_in, s_z80.port_out = z_out;
		else
			m68k_multi_reset_cpu(1);
		rbx_ym_reset(0);
	}
	b.snd_reset = (uint8_t)rst;
	if (rst)
		for (int c = 0; c < s_noki; c++)
			oki_reset(&s_oki[c]);
}

static void soundlatch_w(uint16_t d, int level)
{
	b.latch[0] = d;
	if (s_type == T_Z) {
		b.latch[0] = d & 0xff;   // (soundlatch_z_w: the Z80's line 5, nothing)
		return;
	}
	m68k_multi_hold_irq(1, level);
}

static uint16_t scroll_r(int t, int off)
{
	switch (off) {
	case 0: return b.scrollx[t];
	case 1: return b.scrolly[t];
	case 2: return b.scroll_flag[t];
	default: return 0xffff;
	}
}
static void scroll_w(int t, int off, uint16_t d, uint16_t mask)
{
	uint16_t *r = off == 0 ? &b.scrollx[t] : off == 1 ? &b.scrolly[t] : off == 2 ? &b.scroll_flag[t] : NULL;
	if (r)
		*r = (uint16_t)((*r & ~mask) | (d & mask));
}

// the gate array's overlay of 0x40 bytes in the ROM: its 4 KB page read through the handler while it is on (the rest
// of the ROM stays in the page table)
static int s_ga_page = -1;
static void ga_map(void)
{
	const int want = b.ga_hs ? (int)((((uint32_t)b.ga_hs_ram[4] << 6) & 0x3ffc0) >> M68K_PAGE_BITS) : -1;
	if (want == s_ga_page)
		return;
	if (s_ga_page >= 0)
		m68k_bus_cur->rd[s_ga_page] = s_rom + ((uint32_t)s_ga_page << M68K_PAGE_BITS);
	if (want >= 0)
		m68k_bus_cur->rd[want] = NULL;
	s_ga_page = want;
}

static uint16_t ga_r(uint32_t a)
{
	const uint16_t *rom = (const uint16_t *)s_rom;
	if (b.ga_hs && ((b.ga_hs_ram[4] << 6) & 0x3ffc0) == (a & 0x3ffc0))
		return s_mc->ga == GA_D65006 ? d65006_seq[4] : s_mc->ga == GA_GS88000 ? gs88000_seq[4] : 0;
	return rom[(a & 0x3ffff) >> 1];
}
static void ga_w(uint32_t a, uint16_t d, uint16_t mask)
{
	const uint16_t *seq = s_mc->ga == GA_D65006 ? d65006_seq : s_mc->ga == GA_GS88000 ? gs88000_seq : NULL;
	if (!seq)
		return;
	const int off = (int)((a >> 1) & 7);
	b.ga_hs_ram[off] = (uint16_t)((b.ga_hs_ram[off] & ~mask) | (d & mask));
	b.ga_hs = (b.ga_hs_ram[0] == seq[0] && b.ga_hs_ram[1] == seq[1] && b.ga_hs_ram[2] == seq[2] && b.ga_hs_ram[3] == seq[3] &&
	           off == 4);
	ga_map();
}

static uint16_t port16(int p) { return p >= 0 ? (uint16_t)rbx_port_read(p) : 0xffff; }

static void ip_select_w(uint16_t d)   // the input MCU simulation
{
	if (!s_ip_seq)
		return;
	int i;
	for (i = 0; i < 7; i++)
		if ((d & 0xff) == s_ip_seq[i])
			break;
	switch (i) {
	case 0: b.ip_latched = port16(s_port_sys); break;
	case 1: b.ip_latched = port16(s_port_p1); break;
	case 2: b.ip_latched = port16(s_port_p2); break;
	case 3: b.ip_latched = port16(s_port_dsw1); break;
	case 4: b.ip_latched = port16(s_port_dsw2); break;
	case 5: b.ip_latched = 0x0d; break;
	case 6: b.ip_latched = 0x06; break;
	default: return;
	}
	m68k_multi_hold_irq(0, 2);
}

static uint16_t main_r16(uint32_t a)
{
	switch (s_type) {
	case T_A: case T_Z:
		a &= 0xfffff;
		if (s_mc->ga != GA_NONE && a < 0x40000) return ga_r(a);
		if (a == 0x080000) return port16(s_port_sys);
		if (a == 0x080002) return port16(s_port_p1);
		if (a == 0x080004) return port16(s_port_p2);
		if (a == 0x080006) return port16(s_port_dsw);
		if (a == 0x080008 && s_type == T_A) return b.latch[1];
		if (a >= 0x084200 && a < 0x084206) return scroll_r(0, (int)((a - 0x084200) >> 1));
		if (a >= 0x084208 && a < 0x08420e) return scroll_r(1, (int)((a - 0x084208) >> 1));
		if (s_type == T_A) {
			if (a >= 0x084008 && a < 0x08400e) return scroll_r(2, (int)((a - 0x084008) >> 1));
			if (a == 0x084100) return b.sprite_flag;
		}
		return 0xffff;
	case T_B: case T_BBL: case T_BMONK:
		a &= 0xfffff;
		if (a >= 0x044008 && a < 0x04400e) return scroll_r(2, (int)((a - 0x044008) >> 1));
		if (a == 0x044100) return b.sprite_flag;
		if (a >= 0x044200 && a < 0x044206) return scroll_r(0, (int)((a - 0x044200) >> 1));
		if (a >= 0x044208 && a < 0x04420e) return scroll_r(1, (int)((a - 0x044208) >> 1));
		if (s_type == T_B && a == 0x0e0000) return s_iomcu ? b.mcu_io : b.ip_latched;
		if (s_type == T_BBL) {
			if (a == 0x0e0002) return port16(s_port_sys);
			if (a == 0x0e0004) return port16(s_port_p1);
			if (a == 0x0e0006) return port16(s_port_p2);
			if (a == 0x0e0008) return port16(s_port_dsw1);
			if (a == 0x0e000a) return port16(s_port_dsw2);
		}
		if (s_type == T_BMONK) {
			if (a == 0x0e0002) return port16(s_port_p1);
			if (a == 0x0e0004) return port16(s_port_p2);
			if (a == 0x0e0006) return port16(s_port_dsw1);
			if (a == 0x0e0008) return port16(s_port_dsw2);
			if (a == 0x0e000a) return port16(s_port_sys);
		}
		return 0xffff;
	case T_C:
		a &= 0x1fffff;
		if (a >= 0x0c2000 && a < 0x0c2006) return scroll_r(0, (int)((a - 0x0c2000) >> 1));
		if (a >= 0x0c2008 && a < 0x0c200e) return scroll_r(1, (int)((a - 0x0c2008) >> 1));
		if (a >= 0x0c2100 && a < 0x0c2106) return scroll_r(2, (int)((a - 0x0c2100) >> 1));
		if (a == 0x0c2200) return b.sprite_flag;
		if (a == 0x0c8000) return b.latch[1];
		if (a == 0x0d8000) return s_iomcu ? b.mcu_io : b.ip_latched;
		return 0xffff;
	default:   // T_D
		a &= 0xffffff;
		if (a >= 0x0c2000 && a < 0x0c2006) return scroll_r(0, (int)((a - 0x0c2000) >> 1));
		if (a >= 0x0c2008 && a < 0x0c200e) return scroll_r(1, (int)((a - 0x0c2008) >> 1));
		if (a == 0x0c2200) return b.sprite_flag;
		if (a == 0x0e0000) return port16(s_port_dsw);
		if (a == 0x0f0000) return port16(s_port_sys);
		if (a == 0x0f8000) return oki_read(&s_oki[0]);
		if (a == 0x100000) {   // protection_peekaboo_r
			switch (b.prot_val) {
			case 0x02: return 0x03;
			case 0x51: return port16(s_port_p1);
			case 0x52: return port16(s_port_p2);
			default: return b.prot_val;
			}
		}
		return 0xffff;
	}
}

static void ram_w(uint16_t *w, uint16_t d) { *w = d; }   // (MAME ram_w: a byte write mirrors into both bytes)

static void main_w16m(uint32_t a, uint16_t d, uint16_t mask)
{
	switch (s_type) {
	case T_A: case T_Z:
		a &= 0xfffff;
		if (a >= 0x0f0000) { ram_w(&b.ram[(a & 0xffff) >> 1], d); return; }
		if (s_mc->ga != GA_NONE && a >= 0x20000 && a < 0x30000) { ga_w(a, d, mask); return; }
		if (a >= 0x084200 && a < 0x084206) { scroll_w(0, (int)((a - 0x084200) >> 1), d, mask); return; }
		if (a >= 0x084208 && a < 0x08420e) { scroll_w(1, (int)((a - 0x084208) >> 1), d, mask); return; }
		if (a == 0x084300) { screen_flag_w(d, mask); return; }
		if (a == 0x084308) { soundlatch_w(d, 4); return; }
		if (s_type == T_A) {
			if (a == 0x084000) b.active_layers = (uint16_t)((b.active_layers & ~mask) | (d & mask));
			else if (a >= 0x084008 && a < 0x08400e) scroll_w(2, (int)((a - 0x084008) >> 1), d, mask);
			else if (a == 0x084100) b.sprite_flag = (uint16_t)((b.sprite_flag & ~mask) | (d & mask));
		}
		return;
	case T_B: case T_BBL: case T_BMONK:
		a &= 0xfffff;
		if (a >= 0x060000 && a < 0x080000) { ram_w(&b.ram[(a & 0xffff) >> 1], d); return; }
		if (a == 0x044000) b.active_layers = (uint16_t)((b.active_layers & ~mask) | (d & mask));
		else if (a >= 0x044008 && a < 0x04400e) scroll_w(2, (int)((a - 0x044008) >> 1), d, mask);
		else if (a == 0x044100) b.sprite_flag = (uint16_t)((b.sprite_flag & ~mask) | (d & mask));
		else if (a >= 0x044200 && a < 0x044206) {
			const int off = (int)((a - 0x044200) >> 1);
			if (s_type == T_BMONK && off == 0)   // monkelf_scroll0_w
				d = (uint16_t)(d - (((d & 0x0f) > 0x0d) ? 0x10 : 0));
			scroll_w(0, off, d, mask);
		} else if (a >= 0x044208 && a < 0x04420e) {
			const int off = (int)((a - 0x044208) >> 1);
			if (s_type == T_BMONK && off == 0)
				d = (uint16_t)(d - (((d & 0x0f) > 0x0b) ? 0x10 : 0));
			scroll_w(1, off, d, mask);
		} else if (a == 0x044300) screen_flag_w(d, mask);
		else if (a == 0x044308 && s_type != T_BBL) soundlatch_w(d, 4);
		else if (a == 0x0e0000 && s_type == T_B) {
			if (s_iomcu) {   // ip_select_iomcu_w: data to the MCU's port 1, its INT0 pulsed
				b.mcu_input = (uint8_t)d;
				tlcs90_set_irq(&s_mcu, TLCS90_LINE_INT0, 1);
				b.mcu_int0 = 1;
			} else
				ip_select_w(d);
		}
		return;
	case T_C:
		a &= 0x1fffff;
		if (a >= 0x1c0000) { ram_w(&b.ram[(a & 0xffff) >> 1], d); return; }
		if (a >= 0x0c2000 && a < 0x0c2006) scroll_w(0, (int)((a - 0x0c2000) >> 1), d, mask);
		else if (a >= 0x0c2008 && a < 0x0c200e) scroll_w(1, (int)((a - 0x0c2008) >> 1), d, mask);
		else if (a >= 0x0c2100 && a < 0x0c2106) scroll_w(2, (int)((a - 0x0c2100) >> 1), d, mask);
		else if (a == 0x0c2108) b.sprite_bank = (uint16_t)((b.sprite_bank & ~mask) | (d & mask));
		else if (a == 0x0c2200) b.sprite_flag = (uint16_t)((b.sprite_flag & ~mask) | (d & mask));
		else if (a == 0x0c2208) b.active_layers = (uint16_t)((b.active_layers & ~mask) | (d & mask));
		else if (a == 0x0c2308) screen_flag_w(d, mask);
		else if (a == 0x0c8000) soundlatch_w(d, 2);
		else if (a == 0x0d8000) {
			if (s_iomcu) {
				b.mcu_input = (uint8_t)d;
				tlcs90_set_irq(&s_mcu, TLCS90_LINE_INT0, 1);
				b.mcu_int0 = 1;
			} else
				ip_select_w(d);
		}
		return;
	default:   // T_D
		a &= 0xffffff;
		if (a >= 0x0c2000 && a < 0x0c2006) scroll_w(0, (int)((a - 0x0c2000) >> 1), d, mask);
		else if (a >= 0x0c2008 && a < 0x0c200e) scroll_w(1, (int)((a - 0x0c2008) >> 1), d, mask);
		else if (a == 0x0c2200) b.sprite_flag = (uint16_t)((b.sprite_flag & ~mask) | (d & mask));
		else if (a == 0x0c2208) b.active_layers = (uint16_t)((b.active_layers & ~mask) | (d & mask));
		else if (a == 0x0c2308) screen_flag_w(d, mask);
		else if (a == 0x0f8000) { if (mask & 0xff) oki_write(&s_oki[0], (uint8_t)d); }
		else if (a == 0x100000) {   // protection_peekaboo_w
			b.prot_val = (uint16_t)((b.prot_val & ~mask) | (d & mask));
			if ((b.prot_val & 0x90) == 0x90)
				b.okibank = (uint8_t)(b.prot_val & 7);
			m68k_multi_hold_irq(0, 4);
		}
		return;
	}
}

static uint8_t main_r8(uint32_t a)
{
	const uint16_t w = main_r16(a & ~1u);
	return (a & 1) ? (uint8_t)w : (uint8_t)(w >> 8);
}
static uint16_t main_r16h(uint32_t a) { return main_r16(a & ~1u); }
static void main_w8(uint32_t a, uint8_t v)
{
	const uint32_t aa = a & ~1u;
	// (the work RAM: a byte write lands in both bytes, MAME ram_w)
	main_w16m(aa, (uint16_t)(v << 8 | v), (a & 1) ? 0x00ff : 0xff00);
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a & ~1u, v, 0xffff); }

static void map_main(void)
{
	m68k_bus_reset_map();
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16h;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
	switch (s_type) {
	case T_A: case T_Z: {
		const uint32_t rend = s_type == T_A ? 0x7ffff : 0x3ffff;
		m68k_bus_map(0x000000, rend, s_rom, s_rom_size, 1, 0);
		m68k_bus_map(0x088000, 0x0887ff, b.pal, 0x800, 1, 1);
		m68k_bus_map(0x08c000, 0x08ffff, b.objram, 0x2000, 1, 1);
		m68k_bus_map(0x090000, 0x093fff, b.scrollram[0], 0x4000, 1, 1);
		m68k_bus_map(0x094000, 0x097fff, b.scrollram[1], 0x4000, 1, 1);
		if (s_type == T_A)
			m68k_bus_map(0x098000, 0x09bfff, b.scrollram[2], 0x4000, 1, 1);
		m68k_bus_map(0x0f0000, 0x0fffff, b.ram, 0x10000, 1, 0);
		break;
	}
	case T_B: case T_BBL: case T_BMONK:
		m68k_bus_map(0x000000, 0x03ffff, s_rom, s_rom_size, 1, 0);
		m68k_bus_map(0x048000, 0x0487ff, b.pal, 0x800, 1, 1);
		m68k_bus_map(0x04e000, 0x04ffff, b.objram, 0x2000, 1, 1);
		m68k_bus_map(0x050000, 0x053fff, b.scrollram[0], 0x4000, 1, 1);
		m68k_bus_map(0x054000, 0x057fff, b.scrollram[1], 0x4000, 1, 1);
		m68k_bus_map(0x058000, 0x05bfff, b.scrollram[2], 0x4000, 1, 1);
		m68k_bus_map(0x060000, 0x07ffff, b.ram, 0x10000, 1, 0);
		if (s_rom_size > 0x40000)
			m68k_bus_map(0x080000, 0x0bffff, s_rom + 0x40000, s_rom_size - 0x40000, 1, 0);
		break;
	case T_C:
		m68k_bus_map(0x000000, 0x07ffff, s_rom, s_rom_size, 1, 0);
		m68k_bus_map(0x0d2000, 0x0d3fff, b.objram, 0x2000, 1, 1);
		m68k_bus_map(0x0e0000, 0x0e7fff, b.scrollram[0], 0x4000, 1, 1);
		m68k_bus_map(0x0e8000, 0x0effff, b.scrollram[1], 0x4000, 1, 1);
		m68k_bus_map(0x0f0000, 0x0f7fff, b.scrollram[2], 0x4000, 1, 1);
		m68k_bus_map(0x0f8000, 0x0f87ff, b.pal, 0x800, 1, 1);
		m68k_bus_map(0x1c0000, 0x1fffff, b.ram, 0x10000, 1, 0);
		break;
	default:   // T_D
		m68k_bus_map(0x000000, 0x03ffff, s_rom, s_rom_size, 1, 0);
		m68k_bus_map(0x0ca000, 0x0cbfff, b.objram, 0x2000, 1, 1);
		m68k_bus_map(0x0d0000, 0x0d3fff, b.scrollram[1], 0x4000, 1, 1);
		m68k_bus_map(0x0d8000, 0x0dbfff, b.pal, 0x800, 1, 1);
		m68k_bus_map(0x0e8000, 0x0ebfff, b.scrollram[0], 0x4000, 1, 1);
		m68k_bus_map(0x1f0000, 0x1fffff, b.ram, 0x10000, 1, 1);
		break;
	}
	// MAME's global_mask (A, B, Z: 0xfffff, C: 0x1fffff): the page table mirrored over the 16 MB
	const uint32_t gm = s_type == T_C ? 0x1fffff : s_type == T_D ? 0 : 0xfffff;
	if (gm) {
		const uint32_t np = (gm + 1) >> M68K_PAGE_BITS;
		for (uint32_t pg = np; pg < M68K_PAGES; pg++) {
			m68k_bus.rd[pg] = m68k_bus.rd[pg % np];
			m68k_bus.wr[pg] = m68k_bus.wr[pg % np];
		}
	}
}

static void map_sound(void)
{
	m68k_bus_reset_map();
	m68k_bus.read8 = snd_r8;
	m68k_bus.read16 = snd_r16;
	m68k_bus.write8 = snd_w8;
	m68k_bus.write16 = snd_w16;
	m68k_bus_map(0x000000, 0x01ffff, s_srom, s_srom_size, 1, 0);
	m68k_bus_map(0x0e0000, 0x0fffff, b.sram, sizeof(b.sram), 1, 1);
}

// ---- video ----
// priority_create: the PROM's 16 layer orders
static void priority_create(void)
{
	const uint8_t *pri_prom = s_prom;
	for (int pri_code = 0; pri_code < 0x10; pri_code++) {
		uint32_t layers_order[2];
		for (int offset = 0; offset < 2; offset++) {
			uint8_t enable_mask = 0xf;
			layers_order[offset] = 0xfffff;
			do {
				const uint8_t top = pri_prom[pri_code * 0x20 + offset + enable_mask * 2] & 3;
				const uint8_t top_mask = (uint8_t)(1 << top);
				uint8_t result = 0;
				for (int i = 0; i < 0x10; i++) {
					const uint8_t opacity = (uint8_t)(i & enable_mask);
					const uint8_t layer = pri_prom[pri_code * 0x20 + offset + opacity * 2];
					if (opacity) {
						if (opacity & top_mask) {
							if (layer != top) result |= 1;
						} else {
							if (layer == top) result |= 2;
							else result |= 4;
						}
					}
				}
				layers_order[offset] = ((layers_order[offset] << 4) | top) & 0xfffff;
				enable_mask = (uint8_t)(enable_mask & ~top_mask);
				if (result & 1) { layers_order[offset] = 0xfffff; break; }
				if ((result & 6) == 6) { layers_order[offset] = 0xfffff; break; }
				if (result == 2)
					enable_mask = 0;
			} while (enable_mask);
		}
		uint32_t order = 0xfffff;
		for (int i = 5; i > 0;) {
			int layer;
			const uint8_t layer0 = layers_order[0] & 0x0f, layer1 = layers_order[1] & 0x0f;
			if (layer0 != 3) {
				if (layer1 == 3) {
					layer = 4;
					layers_order[0] <<= 4;
				} else {
					layer = layer0;
					if (layer0 != layer1) { order = 0xfffff; break; }
				}
			} else {
				if (layer1 == 3) {
					layer = 0x43;
					order <<= 4;
					i--;
				} else {
					layer = 3;
					layers_order[1] <<= 4;
				}
			}
			order = (order << 4) | (uint32_t)layer;
			i--;
			layers_order[0] >>= 4;
			layers_order[1] >>= 4;
		}
		s_layers_order[pri_code] = order & 0xfffff;
	}
}

static inline int wrap(int v, int n) { return v < 0 ? n - (-v) % n : v % n; }

// one tilemap's line: the pens (palette index) into d, or 0xffff transparent (pen 15), opaque: every pixel
static void tmap_line(int t, const line_regs *lr, int y, uint16_t *d)
{
	const uint16_t flag = lr->scroll_flag[t];
	const int small = (flag >> 4) & 1, sz = flag & 3;
	// pixel sizes (MAME m_tilemap[small][sz])
	static const int w16[4] = { 4096, 2048, 1024, 512 }, h16[4] = { 512, 1024, 2048, 4096 };
	static const int w8[4] = { 2048, 1024, 1024, 512 }, h8[4] = { 256, 512, 512, 1024 };
	const int W = small ? w8[sz] : w16[sz], H = small ? h8[sz] : h16[sz];
	const int ncols = W / 8;
	const int flip = lr->screen_flag & 1;
	const int sx = lr->scrollx[t], sy = lr->scrolly[t];
	// MAME tilemap: effective scroll (dx = dy = 0) with the flip around the visible area's centre (256 x 256)
	const int yv = wrap(!flip ? -sy : 256 - H - (0 - sy), H);
	const int py = wrap(y - yv, H);
	const int ly = flip ? H - 1 - py : py;
	const int xv = wrap(!flip ? -sx : 256 - W - (0 - sx), W);
	const uint16_t *ram = s_snap->scrollram[t];
	const uint8_t *gfx = s_scroll_gfx[t];
	const uint32_t n = s_scroll_n[t];
	const int row = ly >> 3, fy = ly & 7;
	const uint16_t cbase = (uint16_t)(t == 1 && s_type == T_Z ? 512 : t * 256);
	const int f8 = (t == 1 && s_mc->soldam) ? 4 : 1;
	int lcol = -1;
	const uint8_t *tp = NULL;
	uint16_t pb = 0;
	for (int x = 0; x < 256; x++) {
		const int px = (x - xv) & (W - 1);
		const int lx = flip ? W - 1 - px : px;
		const int col = lx >> 3;
		if (col != lcol) {
			lcol = col;
			uint32_t tile;
			uint16_t code;
			if (small) {   // scan_8x8
				const int idx = col * 32 + (row / 32) * 1024 * (ncols / 32) + (row % 32);
				code = ram[idx & 0x1fff];
				tile = (uint32_t)(code & 0xfff) * (uint32_t)f8;
			} else {       // scan_16x16
				const int idx = (((col / 2) * 16 + ((row / 2) / 16) * 256 * (ncols / 32) + ((row / 2) % 16)) * 4) + (row & 1) + (col & 1) * 2;
				code = ram[(idx >> 2) & 0x1fff];
				tile = (uint32_t)(code & 0xfff) * 4 + (uint32_t)(idx & 3);
			}
			tp = n ? gfx + rbx_wrap(tile, n) * 32 + fy * 4 : NULL;
			pb = (uint16_t)(cbase + (code >> 12) * 16);
		}
		if (!tp) {
			d[x] = 0xffff;
			continue;
		}
		const int fx = lx & 7;
		const uint8_t v = tp[fx >> 1];
		const int pix = (fx & 1) ? (v & 15) : (v >> 4);
		d[x] = pix == 15 ? (uint16_t)(0x8000 | (pb + 15)) : (uint16_t)(pb + pix);   // (0x8000: transparent, its pen kept)
	}
}

// the sprites of the frame (A/B/C/D) into the frame buffer, rows y0..y1 (a segment of equal sprite registers)
static void draw_sprites_seg(const line_regs *lr, int y0, int y1)
{
	if (!((lr->sprite_flag >> 4) & 1)) {
		for (int y = y0; y <= y1; y++)
			for (int x = 0; x < 256; x++)
				s_sprbmp[y * 256 + x] = 0x7fff;
	} else {   // partial_clear_sprite_bitmap
		const int param = lr->sprite_flag & 0x0f;
		for (int y = y0; y <= y1; y++)
			for (int x = 0; x < 256; x++) {
				const uint16_t pixel = s_sprbmp[y * 256 + x];
				s_sprbmp[y * 256 + x] = pixel & 0x7fff;
				if (((pixel & 0xf0) >> 4) < param)
					s_sprbmp[y * 256 + x] = 0x7fff;
			}
	}
	const uint32_t color_mask = ((lr->sprite_flag >> 8) & 1) ? 0x07 : 0x0f;
	const uint16_t *objectram = s_snap->obj, *spriteram = s_snap->spr;
	for (int offs = (0x800 - 8) / 2; offs >= 0; offs -= 4)
		for (int sprite = 0; sprite < 4; sprite++) {
			const uint16_t *od = &objectram[offs + 0x400 * sprite];
			const uint16_t *sd = &spriteram[(od[0] & 0x7f) * 8];
			const uint16_t attr = sd[4];
			if (((attr & 0xc0) >> 6) != sprite)
				continue;
			int sx = (sd[5] + od[1]) & 0x1ff, sy = (sd[6] + od[2]) & 0x1ff;
			if (sx & 0x100) sx -= 0x200;
			if (sy & 0x100) sy -= 0x200;
			uint32_t code = (uint32_t)(sd[7] + od[3]);
			const uint32_t color = attr & color_mask;
			int flipx = (attr >> 6) & 1, flipy = (attr >> 7) & 1;
			const uint32_t pri = (attr >> 3) & 1;
			const int mosaic = (attr & 0x0f00) >> 8, mossol = (attr >> 12) & 1;
			code = (code & 0xfff) + ((uint32_t)(lr->sprite_bank & 1) << 12);
			if (lr->screen_flag & 1) {
				flipx = !flipx;
				flipy = !flipy;
				sx = 240 - sx;
				sy = 240 - sy;
			}
			if (!s_spr_n || sy > y1 || sy + 15 < y0)
				continue;
			const uint8_t *g = s_spr_gfx + rbx_wrap(code, s_spr_n) * 128;
			const int yxor = flipy ? 15 : 0, xxor = flipx ? 15 : 0;
			const uint16_t col = (uint16_t)(color << 4);
			for (int yy = 0; yy < 16; yy++) {
				const int dy = sy + yy;
				if (dy < y0 || dy > y1)
					continue;
				const int srcy = yy ^ yxor;
				const int ry = mossol ? (srcy | mosaic) : (srcy & ~mosaic);
				uint16_t *dst = s_sprbmp + dy * 256;
				for (int xx = 0; xx < 16; xx++) {
					const int dx = sx + xx;
					if (dx < 0 || dx > 255)
						continue;
					const int srcx = xx ^ xxor;
					const int rx = mossol ? (srcx | mosaic) : (srcx & ~mosaic);
					const uint8_t v = g[((rx & 8) << 3) + ry * 4 + ((rx & 7) >> 1)];
					const int pxl = (rx & 1) ? (v & 15) : (v >> 4);
					if (pxl != 15 && !(dst[dx] & 0x8000))
						dst[dx] = (uint16_t)((pxl + col) | (pri << 14) | 0x8000);
				}
			}
		}
}

// type Z: the 128 sprites of the Sprite Data drawn over the line (prio_transpen), last first
static void z_sprites_line(int y, const line_regs *lr, uint16_t *d, const uint8_t *prio)
{
	const uint16_t *sp = s_snap->zspr;
	for (int sprite = 0x7f; sprite >= 0; sprite--) {
		const uint16_t *s = &sp[sprite * 8];
		const uint16_t attr = s[4];
		int sx = s[5] & 0x1ff, sy = s[6] & 0x1ff;
		if (sx & 0x100) sx -= 0x200;
		if (sy & 0x100) sy -= 0x200;
		const uint32_t code = s[7], color = attr & 0x0f;
		int flipx = (attr >> 6) & 1, flipy = (attr >> 7) & 1;
		if (lr->screen_flag & 1) {
			flipx = !flipx;
			flipy = !flipy;
			sx = 240 - sx;
			sy = 240 - sy;
		}
		if (y < sy || y > sy + 15 || !s_spr_n)
			continue;
		const uint32_t pmask = (attr & 8) ? 0x0c : 0x0a;
		const uint8_t *g = s_spr_gfx + rbx_wrap(code, s_spr_n) * 128;
		const int ty = flipy ? 15 - (y - sy) : y - sy;
		for (int xx = 0; xx < 16; xx++) {
			const int x = sx + xx;
			if (x < 0 || x > 255)
				continue;
			const int tx = flipx ? 15 - xx : xx;
			const uint8_t v = g[((tx & 8) << 3) + ty * 4 + ((tx & 7) >> 1)];
			const int pxl = (tx & 1) ? (v & 15) : (v >> 4);
			if (pxl != 15 && !((pmask >> (prio[x] & 0x1f)) & 1))
				d[x] = (uint16_t)(256 + color * 16 + pxl);
		}
	}
}

static void render_line(int y)
{
	const line_regs *lr = &s_snap->line[y - VIS_Y0];
	uint32_t pri, active;
	if (s_type == T_Z) {
		active = 0x000b;
		pri = 0x0314f;
	} else {
		uint32_t really = 0;
		pri = s_layers_order[(lr->active_layers & 0x0f00) >> 8];
		if (pri == 0xfffff)
			pri = 0x04132;
		for (int i = 0; i < 5; i++)
			really |= 1u << ((pri >> (4 * i)) & 0x0f);
		active = lr->active_layers & really;
		active |= 1u << ((pri & 0xf0000) >> 16);
	}
	uint16_t d[256], t[256];
	uint8_t prio[256];
	memset(prio, 0, sizeof(prio));
	int opaque = 1;
	uint8_t primask = 0;
	for (int i = 0; i < 5; i++) {
		const int layer = (int)((pri & 0xf0000) >> 16);
		pri <<= 4;
		if (layer <= 2) {
			if (layer < s_nscroll && ((active >> layer) & 1)) {
				tmap_line(layer, lr, y, t);
				for (int x = 0; x < 256; x++) {
					if (opaque) {
						d[x] = t[x] & 0x7fff;
						prio[x] = primask;
					} else if (!(t[x] & 0x8000)) {
						d[x] = t[x];
						prio[x] = primask;
					}
				}
				opaque = 0;
			}
		} else if (layer == 3 || layer == 4) {
			if (opaque) {
				opaque = 0;
				memset(d, 0, sizeof(d));
			}
			if ((lr->sprite_flag >> 8) & 1)
				primask = (uint8_t)(primask | (1 << (layer - 3)));
			else if (layer == 3)
				primask |= 3;
		}
	}
	if (opaque)
		memset(d, 0, sizeof(d));
	if ((active >> 3) & 1) {
		if (s_type == T_Z)
			z_sprites_line(y, lr, d, prio);
		else {   // mix_sprite_bitmap
			const uint16_t *src = s_sprbmp + y * 256;
			for (int x = 0; x < 256; x++) {
				const uint16_t pixel = src[x];
				if ((pixel & 0xf) != 0xf) {
					const uint32_t priority = ((pixel >> 14) & 1) ? 0x0c : 0x0a;
					if (!((priority >> (prio[x] & 0x1f)) & 1))
						d[x] = (uint16_t)((pixel & 0x3fff) + 0x300);
				}
			}
		}
	}
	memcpy(rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch, d, sizeof(d));
}

#define BANDS 8
static void ms_band(int band)
{
	const int r0 = VIS_Y0 + VIS_H * band / BANDS, r1 = VIS_Y0 + VIS_H * (band + 1) / BANDS;
	for (int y = r0; y < r1; y++)
		render_line(y);
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, ms_band };
static int ms_help(int helper) { return decoband_help(&s_bands, helper); }

static void ms_render(void)
{
	for (int i = 0; i < 0x400; i++) {   // RRRRGGGGBBBBRGBx (type D: RGBx 555)
		const uint16_t v = s_snap->pal[i];
		int r, g, bl;
		if (s_type == T_D) {
			r = (v >> 11) & 31;
			g = (v >> 6) & 31;
			bl = (v >> 1) & 31;
		} else {
			r = ((v >> 11) & 0x1e) | ((v >> 3) & 1);
			g = ((v >> 7) & 0x1e) | ((v >> 2) & 1);
			bl = ((v >> 3) & 0x1e) | ((v >> 1) & 1);
		}
		rbx_video.palette[i] = (uint32_t)(pal5(r) << 16 | pal5(g) << 8 | pal5(bl));
	}
	if (s_type != T_Z) {   // the sprite frame buffer, in segments of equal sprite registers (partial updates)
		int y0 = VIS_Y0;
		for (int y = VIS_Y0 + 1; y <= VIS_Y1 + 1; y++) {
			const line_regs *a = &s_snap->line[y0 - VIS_Y0];
			const line_regs *c = y <= VIS_Y1 ? &s_snap->line[y - VIS_Y0] : NULL;
			if (!c || c->sprite_flag != a->sprite_flag || c->sprite_bank != a->sprite_bank ||
			    ((c->screen_flag ^ a->screen_flag) & 1) || ((c->active_layers ^ a->active_layers) & 8)) {
				if ((a->active_layers >> 3) & 1 || 1)
					draw_sprites_seg(a, y0, y - 1);
				y0 = y;
			}
		}
	}
	decoband_run(&s_bands);
}

static void line_capture(int y)
{
	if (y < VIS_Y0 || y > VIS_Y1)
		return;
	line_regs *l = &s_line_cur[y - VIS_Y0];
	memcpy(l->scrollx, b.scrollx, sizeof(l->scrollx));
	memcpy(l->scrolly, b.scrolly, sizeof(l->scrolly));
	memcpy(l->scroll_flag, b.scroll_flag, sizeof(l->scroll_flag));
	l->active_layers = b.active_layers;
	l->sprite_flag = b.sprite_flag;
	l->sprite_bank = b.sprite_bank;
	l->screen_flag = b.screen_flag;
}

static void vblank_capture(void)
{
	memcpy(s_snap->scrollram, b.scrollram, sizeof(b.scrollram));
	memcpy(s_snap->pal, b.pal, sizeof(b.pal));
	memcpy(s_snap->obj, b.obj_buf2, sizeof(b.obj_buf2));
	memcpy(s_snap->spr, b.spr_buf2, sizeof(b.spr_buf2));
	memcpy(s_snap->zspr, &b.ram[0x8000 / 2], sizeof(s_snap->zspr));
	memcpy(s_snap->line, s_line_cur, sizeof(s_line_cur));
	rbx_video_frame_done();
	// screen_vblank: the sprites two frames ahead
	memcpy(b.obj_buf2, b.obj_buf, sizeof(b.obj_buf));
	memcpy(b.obj_buf, b.objram, sizeof(b.objram));
	memcpy(b.spr_buf2, b.spr_buf, sizeof(b.spr_buf));
	memcpy(b.spr_buf, &b.ram[0x8000 / 2], sizeof(b.spr_buf));
}

// ---- frame ----
static void sound_line(void)
{
	if (s_type == T_D) {
		int n = rbx_line_cycles(&b.out_frac, s_out_rate, s_fps_mhz, s_lines);
		if (n > s_mix_cap - s_mix_n)
			n = s_mix_cap - s_mix_n;
		for (int i = 0; i < n; i++)
			s_mix[s_mix_n + i] = oki_sample(&s_oki[0]);
		s_mix_n += n;
		return;
	}
	if (s_type == T_BBL) {
		int n = rbx_line_cycles(&b.out_frac, s_out_rate, s_fps_mhz, s_lines);
		if (n > s_mix_cap - s_mix_n)
			n = s_mix_cap - s_mix_n;
		for (int i = 0; i < n; i++)
			s_mix[s_mix_n + i] = oki_sample(&s_oki[0]);
		s_mix_n += n;
		return;
	}
	const uint32_t ymclk = s_type == T_Z ? 1500000 : SND_CLOCK / 2;
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&b.ym_frac, ymclk, s_fps_mhz, s_lines);
	const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	for (int i = 0; i < n; i++) {
		int32_t s = 0;
		for (int c = 0; c < s_noki; c++) {
			b.oki_acc[c] += s_oki[c].rate;
			while (b.oki_acc[c] >= s_out_rate) {
				b.oki_acc[c] -= s_out_rate;
				b.oki_hold[c] = oki_sample(&s_oki[c]);
			}
			s += b.oki_hold[c];
		}
		s_mix[s_mix_n + i] += s;
	}
	s_mix_n += n;
}

static void scanline_irqs(int line)
{
	switch (s_type) {
	case T_A: case T_Z:
		if (s_mc->iganinju) {
			if (b.ram[0] != 0 && line == 240)
				m68k_multi_hold_irq(0, 2);
			break;
		}
		if (line == 240) m68k_multi_hold_irq(0, 2);
		if (line == 96) m68k_multi_hold_irq(0, 1);
		if (line == 16) m68k_multi_hold_irq(0, 3);
		break;
	case T_B: case T_C:
		if (line == 240) m68k_multi_hold_irq(0, 4);
		if (line == 96) m68k_multi_hold_irq(0, 1);
		if (s_iomcu) {
			if (line == 16) tlcs90_set_irq(&s_mcu, TLCS90_LINE_INT1, 1);
			if (line == 240) tlcs90_set_irq(&s_mcu, TLCS90_LINE_INT1, 0);
		} else if (line == 16)
			m68k_multi_hold_irq(0, 2);
		break;
	case T_BBL: case T_BMONK:
		if (line == 240) m68k_multi_hold_irq(0, 4);
		if (line == 0) m68k_multi_hold_irq(0, 2);
		if (line == 128) m68k_multi_hold_irq(0, 1);
		break;
	case T_D:
		if (line == 240) m68k_multi_hold_irq(0, 2);
		break;
	}
}

static void ms_frame(void)
{
	rbx_audio.frame_mhz = s_fps_mhz;
	for (int line = 0; line < s_lines; line++) {
		if (line == VIS_Y1 + 1) {
			rbx_video_wait_idle();
			vblank_capture();
		}
		scanline_irqs(line);
		for (int sl = 0; sl < SLICES; sl++) {
			const int cyc = rbx_line_cycles(&b.cpu_frac, s_cpu_clock, s_fps_mhz, s_lines * SLICES);
			m68k_multi_switch(0);
			m68k_execute(cyc);
			if (s_type == T_A || s_type == T_B || s_type == T_C || s_type == T_BMONK) {
				const int sc = rbx_line_cycles(&b.snd_frac, SND_CLOCK, s_fps_mhz, s_lines * SLICES);
				if (!b.snd_reset) {
					m68k_multi_switch(1);
					m68k_execute(sc);
				}
			} else if (s_type == T_Z) {
				const int zc = rbx_line_cycles(&b.snd_frac, 3000000, s_fps_mhz, s_lines * SLICES);
				if (!b.snd_reset)
					z80_run(&s_z80, s_z80.cyc + (unsigned long)zc, &s_zirq, 0);
			}
			if (s_iomcu) {
				tlcs90_execute(&s_mcu, rbx_line_cycles(&b.mcu_frac, s_cpu_clock, s_fps_mhz, s_lines * SLICES));
				if (b.mcu_int0) {   // (pulse_input_line: the INT0 pulse over once the MCU has run)
					tlcs90_set_irq(&s_mcu, TLCS90_LINE_INT0, 0);
					b.mcu_int0 = 0;
				}
			}
		}
		line_capture(line);
		sound_line();
	}
	m68k_multi_switch(0);
	if (s_type != T_D && s_type != T_BBL)
		rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static void ms_reset(void)
{
	rbx_video_wait_idle();
	memset(&b, 0, sizeof(b));
	for (int t = 0; t < 3; t++)   // (ms1_tmap device_reset: the scroll RAM filled with 0xffff)
		for (int i = 0; i < 0x2000; i++)
			b.scrollram[t][i] = 0xffff;
	b.ip_latched = 0x0006;
	for (int i = 0; i < 256 * 256; i++)
		s_sprbmp[i] = 0x7fff;
	m68k_multi_reset();
	for (int c = 0; c < s_noki; c++)
		oki_reset(&s_oki[c]);
	if (s_type != T_D && s_type != T_BBL)
		rbx_ym_reset(0);
	if (s_type == T_Z) {
		z80_init(&s_z80);
		s_z80.read_byte = z_read;
		s_z80.write_byte = z_write;
		s_z80.port_in = z_in;
		s_z80.port_out = z_out;
	}
	if (s_iomcu)
		tlcs90_reset(&s_mcu);
	s_zirq = 0;
	s_mix_n = 0;
	if (s_ga_page >= 0) {   // (the overlay page back in the main CPU's page table)
		m68k_multi_switch(0);
		m68k_bus_cur->rd[s_ga_page] = s_rom + ((uint32_t)s_ga_page << M68K_PAGE_BITS);
		s_ga_page = -1;
	}
}

static const machine_t *find_machine(const char *machine)
{
	for (size_t i = 0; i < sizeof(s_machines) / sizeof(s_machines[0]); i++) {
		const size_t l = strlen(s_machines[i].name);
		if (!strncmp(machine, s_machines[i].name, l) && (machine[l] == ':' || !machine[l]))
			return &s_machines[i];
	}
	return NULL;
}

static int ms_supported(const char *machine) { return find_machine(machine) != NULL; }

static void ms_exit(void)
{
	rbx_video_wait_idle();
	if (s_type != T_D && s_type != T_BBL)
		rbx_ym_exit(0);
	m68k_multi_exit();
	free(s_snap);
	s_snap = NULL;
	free(s_mix);
	s_mix = NULL;
	free(s_bp);   // (last: the render thread and the saves are done with it)
	s_bp = NULL;
	free(s_sprbmp_hp);
	s_sprbmp_hp = NULL;
}

// the graphics' data and address lines swapped (init_*_gfx)
static void gfx_unmangle(uint8_t *rom, uint32_t size, int kind)
{
	uint8_t *buf = (uint8_t *)malloc(size);
	if (!buf)
		return;
	for (uint32_t i = 0; i < size; i++) {
		const uint8_t v = rom[i];
		if (kind == 0)   // rodland: 76543210 -> 64537210
			rom[i] = (uint8_t)((v & 0x27) | ((v & 0x80) >> 4) | ((v & 0x48) << 1) | ((v & 0x10) << 2));
		else if (kind == 1)   // jitsupro: bitswap 4,3,5,7,6,2,1,0
			rom[i] = (uint8_t)(((v >> 4) & 1) << 7 | ((v >> 3) & 1) << 6 | ((v >> 5) & 1) << 5 | ((v >> 7) & 1) << 4 |
			                   ((v >> 6) & 1) << 3 | (v & 7));
		else             // stdragona: bitswap 3,7,5,6,4,2,1,0
			rom[i] = (uint8_t)(((v >> 3) & 1) << 7 | ((v >> 7) & 1) << 6 | ((v >> 5) & 1) << 5 | ((v >> 6) & 1) << 4 |
			                   ((v >> 4) & 1) << 3 | (v & 7));
	}
	memcpy(buf, rom, size);
	for (uint32_t i = 0; i < size; i++) {
		uint32_t a;
		if (kind == 0)
			a = (i & ~0x2508u) | ((i & 0x2000) >> 10) | ((i & 0x0400) << 3) | ((i & 0x0100) << 2) | ((i & 0x0008) << 5);
		else {
			static const int js[16] = { 0xf, 0xe, 0x8, 0xc, 0xb, 0x3, 0x9, 0xd, 0x7, 0x6, 0x5, 0x4, 0xa, 0x2, 0x1, 0x0 };
			static const int sd[16] = { 0xf, 0xe, 0x3, 0xc, 0xb, 0xd, 0x9, 0xa, 0x7, 0x6, 0x5, 0x4, 0x8, 0x2, 0x1, 0x0 };
			const int *bs = kind == 1 ? js : sd;
			uint32_t w = 0;
			for (int k = 0; k < 16; k++)
				w |= ((i >> bs[k]) & 1) << (15 - k);
			a = (i & ~0xffffu) | w;
		}
		rom[i] = a < size ? buf[a] : 0;
	}
	free(buf);
}

static uint16_t bitswap16(uint16_t x, const int *b16)
{
	uint16_t r = 0;
	for (int k = 0; k < 16; k++)
		r |= (uint16_t)(((x >> b16[k]) & 1) << (15 - k));
	return r;
}

static void ga_decode(void)
{
	uint16_t *ram = (uint16_t *)s_rom;
	int size = (int)s_rom_size;
	if (size > 0x40000)
		size = 0x40000;
	static const int d0[16] = { 0xd,0xe,0xf,0x0,0x1,0x8,0x9,0xa,0xb,0xc,0x5,0x6,0x7,0x2,0x3,0x4 };
	static const int d1[16] = { 0xf,0xd,0xb,0x9,0x7,0x5,0x3,0x1,0xe,0xc,0xa,0x8,0x6,0x4,0x2,0x0 };
	static const int d2[16] = { 0x0,0x1,0x2,0x3,0x4,0x5,0x6,0x7,0xb,0xa,0x9,0x8,0xf,0xe,0xd,0xc };
	static const int g0[16] = { 0xd,0xe,0xf,0x0,0xa,0x9,0x8,0x1,0x6,0x5,0xc,0xb,0x7,0x2,0x3,0x4 };
	static const int g1[16] = { 0xf,0xd,0xb,0x9,0x7,0x5,0x3,0x1,0x8,0xa,0xc,0xe,0x0,0x2,0x4,0x6 };
	static const int g2[16] = { 0x4,0x5,0x6,0x7,0x0,0x1,0x2,0x3,0xb,0xa,0x9,0x8,0xf,0xe,0xd,0xc };
	static const int u0[16] = { 0xd,0x0,0xa,0x9,0x6,0xe,0xb,0xf,0x5,0xc,0x7,0x2,0x3,0x8,0x1,0x4 };
	static const int u1[16] = { 0x4,0x5,0x6,0x7,0x0,0x1,0x2,0x3,0xb,0xa,0x9,0x8,0xf,0xe,0xd,0xc };
	static const int u2[16] = { 0xf,0xd,0xb,0x9,0xc,0xe,0x0,0x7,0x5,0x3,0x1,0x8,0xa,0x2,0x4,0x6 };
	static const int u3[16] = { 0x4,0x5,0x1,0x2,0xe,0xd,0x3,0xb,0xa,0x9,0x6,0x7,0x0,0x8,0xf,0xc };
	for (int i = 0; i < size / 2; i++) {
		const uint16_t x = ram[i];
		const int alt = (i | (0x248 / 2)) != i;
		uint16_t y;
		if (s_mc->ga == GA_UNK) {
			if (i < 0x08000 / 2) y = bitswap16(x, alt ? u0 : u1);
			else if (i < 0x10000 / 2) y = bitswap16(x, alt ? u2 : u3);
			else if (i < 0x18000 / 2) y = bitswap16(x, alt ? u0 : u1);
			else if (i < 0x20000 / 2) y = bitswap16(x, u1);
			else y = bitswap16(x, u3);
		} else {
			const int *b0 = s_mc->ga == GA_D65006 ? d0 : g0, *b1 = s_mc->ga == GA_D65006 ? d1 : g1, *b2 = s_mc->ga == GA_D65006 ? d2 : g2;
			if (i < 0x08000 / 2) y = bitswap16(x, alt ? b0 : b1);
			else if (i < 0x10000 / 2) y = bitswap16(x, b2);
			else if (i < 0x18000 / 2) y = bitswap16(x, alt ? b0 : b1);
			else if (i < 0x20000 / 2) y = bitswap16(x, b1);
			else y = bitswap16(x, b2);
		}
		ram[i] = y;
	}
}

static int ms_init(const rbx_game *g)
{
	if (!s_sprbmp_hp && !(s_sprbmp_hp = calloc(1, sizeof(*s_sprbmp_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_bp && !(s_bp = (struct megasys1_state *)calloc(1, sizeof(*s_bp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_mc = find_machine(g->machine);
	if (!s_mc) {
		rbx_fail("%s: machine %s not emulated", g->set, g->machine);
		return -1;
	}
	s_type = s_mc->type;
	const char *init = strchr(g->machine, ':');
	init = init ? init + 1 : "";
	s_hachoo = !strncmp(g->set, "hachoo", 6);
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("audiocpu", &s_srom_size);
	s_scroll_gfx[0] = rbx_region("scroll1", &s_scroll_size[0]);
	s_scroll_gfx[1] = rbx_region("scroll2", &s_scroll_size[1]);
	s_scroll_gfx[2] = rbx_region("scroll3", &s_scroll_size[2]);
	s_spr_gfx = rbx_region("sprites", &s_spr_size);
	s_oki_rom[0] = rbx_region("oki1", &s_oki_size[0]);
	s_oki_rom[1] = rbx_region("oki2", &s_oki_size[1]);
	uint32_t psz = 0, msz = 0;
	s_prom = rbx_region("proms", &psz);
	s_mcu_rom = s_mc->iomcu ? rbx_region_opt("iomcu", &msz) : NULL;
	if (!s_rom || !s_scroll_gfx[0] || !s_spr_gfx || (s_type != T_Z && (!s_prom || psz < 0x200)) ||
	    ((s_type == T_A || s_type == T_B || s_type == T_C || s_type == T_BMONK || s_type == T_Z) && !s_srom) ||
	    0) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_nscroll = s_type == T_Z || s_type == T_D ? 2 : 3;
	m68k_bus_swap16(s_rom, s_rom_size);
	if (s_srom && s_type != T_Z)
		m68k_bus_swap16(s_srom, s_srom_size);
	if (s_mc->ga != GA_NONE)
		ga_decode();
	if (!strcmp(init, "rodland_gfx") || !strcmp(init, "jitsupro_gfx") || !strcmp(init, "stdragon_gfx")) {
		const int kind = init[0] == 'r' ? 0 : init[0] == 'j' ? 1 : 2;
		gfx_unmangle(s_scroll_gfx[0], s_scroll_size[0], kind);
		gfx_unmangle(s_spr_gfx, s_spr_size, kind);
	}
	if (!strcmp(init, "lordofkbp")) {   // (bytes of the swapped region, as MAME's)
		for (uint32_t i = 0x20000; i < 0x40000 && i < s_rom_size; i += 2)
			s_rom[i] = (uint8_t)((i & 2) ? 0x51 - s_rom[i] : 0xc7 - s_rom[i]);
	}
	if (!strcmp(init, "monkelf")) {
		((uint16_t *)s_rom)[0x00744 / 2] = 0x4e71;
		for (int i = 0x1fe; i >= 0; i -= 2)
			s_prom[i] = s_prom[i + 1] = (uint8_t)((s_prom[i / 2] >> 4) & 0x0f);
	}
	s_ip_seq = !strcmp(init, "hayaosi1") ? hayaosi1_seq : !strcmp(init, "chimeraba") ? chimeraba_seq : NULL;
	s_iomcu = s_mc->iomcu && s_mcu_rom;
	if (s_mc->iomcu && !s_mcu_rom) {   // (no MCU dump in the user's set: MAME's former simulation and its codes)
		static const struct { const char *set; uint8_t seq[7]; } old[] = {
			{ "64street", { 0x57, 0x53, 0x54, 0x55, 0x56, 0xfa, 0x06 } },
			{ "avspirit", { 0x37, 0x35, 0x36, 0x33, 0x34, 0xff, 0x06 } },
			{ "bigstrik", { 0x58, 0x54, 0x55, 0x56, 0x57, 0xfb, 0x06 } },
			{ "chimerab", { 0x56, 0x52, 0x53, 0x55, 0x54, 0xfa, 0x06 } },
			{ "cybattlr", { 0x56, 0x52, 0x53, 0x54, 0x55, 0xf2, 0x06 } },
			{ "edf", { 0x20, 0x21, 0x22, 0x23, 0x24, 0xf0, 0x06 } },
		};
		const char *key = g->parent ? g->parent : g->set;
		for (size_t i = 0; i < sizeof(old) / sizeof(old[0]); i++)
			if (!strcmp(key, old[i].set))
				s_ip_seq = old[i].seq;
	}
	for (int t = 0; t < 3; t++)
		s_scroll_n[t] = s_scroll_gfx[t] ? s_scroll_size[t] / 32 : 0;
	s_spr_n = s_spr_size / 128;
	if (s_type != T_Z)
		priority_create();
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_p1 = rbx_port_find("P1");
	s_port_p2 = rbx_port_find("P2");
	s_port_dsw = rbx_port_find("DSW");
	s_port_dsw1 = rbx_port_find("DSW1");
	s_port_dsw2 = rbx_port_find("DSW2");
	// clocks, lines
	switch (s_type) {
	case T_A: s_cpu_clock = 6000000; s_lines = 263; s_fps_mhz = 56191; break;   // 6 MHz / (406 x 263)
	case T_B: s_cpu_clock = 8000000; s_lines = 278; s_fps_mhz = 56205; break;   // 6 MHz / (384 x 278)
	case T_C: s_cpu_clock = 12000000; s_lines = 278; s_fps_mhz = 56205; break;
	case T_BMONK: s_cpu_clock = 8000000; s_lines = 263; s_fps_mhz = 56191; break;
	case T_BBL: s_cpu_clock = 6000000; s_lines = 256; s_fps_mhz = 56180; break;
	case T_D: s_cpu_clock = 8000000; s_lines = 256; s_fps_mhz = 56180; break;
	default: s_cpu_clock = 6000000; s_lines = 256; s_fps_mhz = 56180; break;   // Z
	}
	// sound
	memset(s_oki, 0, sizeof(s_oki));
	s_noki = 0;
	if (s_type == T_D) {
		oki_init(&s_oki[0], 2000000, 1, s_oki_rom[0], s_oki_size[0], 256);
		s_oki[0].read = oki_read_d;
		s_noki = 1;
		s_out_rate = s_oki[0].rate;
	} else if (s_type == T_BBL) {
		oki_init(&s_oki[0], 4000000, 1, s_oki_rom[0], s_oki_size[0], 77);
		s_noki = 1;
		s_out_rate = s_oki[0].rate;
	} else {
		const uint32_t okiclk = s_mc->iosim && s_type == T_B ? 2000000 : 4000000;
		if (s_oki_rom[0]) oki_init(&s_oki[s_noki++], okiclk, 1, s_oki_rom[0], s_oki_size[0], 77);
		if (s_oki_rom[1] && !s_mc->kickoffb) oki_init(&s_oki[s_noki++], okiclk, 1, s_oki_rom[1], s_oki_size[1], 77);
		rbx_ym_config yc;
		memset(&yc, 0, sizeof(yc));
		if (s_type == T_Z) {
			yc.type = RBX_YM2203;
			yc.clock = 1500000;
			yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 128;
			yc.irq = ym_irq_z;
		} else if (s_mc->kickoffb) {
			yc.type = RBX_YM2203;
			yc.clock = SND_CLOCK / 2;
			yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 205;
			yc.irq = ym_irq;
		} else {
			yc.type = RBX_YM2151;
			yc.clock = SND_CLOCK / 2;
			yc.gain[0] = yc.gain[1] = 102;   // (0.80, both sides to mono: x0.5)
			yc.irq = ym_irq;
		}
		if (rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
		s_out_rate = rbx_ym_rate(0);
	}
	s_mix_cap = (int)(s_out_rate / 40) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_mix || !s_snap || rbx_audio_setup(s_out_rate) || rbx_video_setup(256, VIS_H, 0x400) ||
	    m68k_multi_init(2)) {
		rbx_fail("%s: out of memory", g->set);
		ms_exit();
		return -1;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	s_ga_page = -1;
	map_main();
	m68k_multi_capture(0);
	if (s_srom && s_type != T_Z)
		map_sound();
	else
		map_main();
	m68k_multi_capture(1);
	m68k_multi_finish();
	if (s_iomcu) {
		tlcs90_init(&s_mcu, TLCS90_TMP91640);
		s_mcu.rom = s_mcu_rom;
		s_mcu.read8 = mcu_read;
		s_mcu.port_r = mcu_port_r;
		s_mcu.port_w = mcu_port_w;
	}
	ms_reset();
	return 0;
}

static int ms_state(void *ctx, int load)
{
	if (load)
		rbx_video_wait_idle();
	int rc = 0;
	rc |= m68k_multi_state(ctx, load);
	rc |= rbx_state_io(ctx, "megasys1", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	for (int c = 0; c < s_noki; c++)
		rc |= rbx_state_io(ctx, c ? "oki1" : "oki0", &s_oki[c], (uint32_t)offsetof(okim6295_t, rom));
	if (s_type != T_D && s_type != T_BBL)
		rc |= rbx_ym_state(0, ctx, load);
	if (s_type == T_Z) {
		z80 z = s_z80;
		rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
		if (!rc && load) {
			z.read_byte = z_read;
			z.write_byte = z_write;
			z.port_in = z_in;
			z.port_out = z_out;
			s_z80 = z;
		}
	}
	if (s_iomcu)
		rc |= rbx_state_io(ctx, "iomcu", (uint8_t *)&s_mcu + TLCS90_STATE_OFFSET, (uint32_t)TLCS90_STATE_SIZE);
	if (!rc && load && s_mc->ga != GA_NONE) {   // (the overlay page as the loaded state has it, on the main CPU's bus)
		const int cur = m68k_multi_current();
		m68k_multi_switch(0);
		ga_map();
		m68k_multi_switch(cur);
	}
	return rc ? -1 : 0;
}

static int ms_save(void *ctx) { return ms_state(ctx, 0); }
static int ms_load(void *ctx) { return ms_state(ctx, 1); }

const rbx_driver rbx_drv_megasys1 = { "Jaleco Mega System 1", ms_init, ms_exit, ms_reset, ms_frame, ms_save, ms_load,
                                      ms_supported, ms_render, ms_help };
