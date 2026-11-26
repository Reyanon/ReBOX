/*  Namco System 2 (the standard board: two 68000s, the MC6809E sound CPU with YM2151 + C140, the C65 I/O MCU
    (HD63705), the C148 interrupt controllers, the C116 palette / clip, the C123 tilemaps, the C102 ROZ layer and the
    System 2 sprites).
    Ported from MAME 0.289 src/mame/namco/namcos2.cpp, namcos2.h, namcos2_m.cpp, namcos2_v.cpp, namcos2_sprite.cpp,
    namcos2_roz.cpp, namco65.cpp, namco_c148.cpp, namco_c139.cpp, namco_c116.cpp, namco_c123tmap.cpp
    (BSD-3-Clause, copyright-holders: K.Wilkins, R. Belmont, David Haywood, Phil Stroffolino, Alex W. Jackson).
    This file is part of ReBOX. GPL-2.0-or-later.

    Not done: the C68 (M37450) boards, Final Lap / Four Trax (C45 road), Metal Hawk, the C355 sprite boards (Steel
    Gunner, Suzuka 8 Hours, Lucky & Wild), the raster split at the position IRQ, the serial link (C139), outputs.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k_multi.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/m6805/m68705.h"
#include "../../cpu/m740/m740.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/c140.h"
#include "../video/c355spr.h"
#include "../video/c169roz.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define VIS_W 288
#define VIS_H 224
#define LINES 264
#define FRAME_MHZ 60606          // 6.144 MHz / (384 x 264)
#define M68K_CLOCK 12288000
#define SND_CLOCK 2048000        // the 68B09 (E clock)
#define MCU_CLOCK 512000         // the C65: 2.048 MHz / 4
#define YM_CLOCK 3579545
#define C140_CLOCK 21333         // 49.152 MHz / 384 / 6
#define PAL_BLACK 0x2000

enum {
	GT_NONE, GT_ORDYNE, GT_MIRNINJA, GT_PHELIOS, GT_DIRTFOXJ, GT_FINEHOUR, GT_BURNFORC, GT_MARVLAND, GT_DSABER,
	GT_RTHUN2, GT_COSMOGNG, GT_GOLLYGHO, GT_BUBBLETR, GT_SWS, GT_SWS92, GT_SWS92G, GT_SWS93, GT_SGUNNER2, GT_SUZUK8H2
};

// ---- the set's configuration ----
static uint8_t *s_rom[2], *s_data, *s_sndrom, *s_mcuint, *s_mcuext, *s_chr, *s_mask, *s_roz, *s_sprrom, *s_c140rom;
static uint32_t s_rom_size[2], s_data_size, s_sndrom_size, s_mcuint_size, s_mcuext_size, s_chr_size, s_mask_size,
	s_roz_size, s_sprrom_size, s_c140_size;
static uint8_t *s_sprpix;       // the sprites, a byte a pixel (32x32)
static uint32_t s_spr_count, s_chr_count, s_mask_count, s_roz_count;
static int s_gametype;
static int s_port_mcub, s_port_mcuc, s_port_mcuh, s_port_dsw, s_port_di[4], s_port_an[8];
// light guns on the A/D channels (Steel Gunner, Lucky & Wild ...): the axis type of each channel (0 none), its player,
// reversed (MAME PORT_REVERSE); the position follows rbx_analog (the d-pad without a pointing device)
static int s_an_gun[8], s_an_player[8], s_an_rev[8];

static m6809_t s_snd;
static m68705_t s_mcu;
static m740_t s_c68;
static int s_is_c68;          // (the C68 I/O MCU, an M37450, in place of the C65)
static uint8_t *s_c68rom;
static uint32_t s_c68rom_size;
static c140_t s_c140;

// ---- the board's state (saved) ----
typedef struct {
	uint8_t level[5];   // cpu, ex, pos, sci, vblank
	uint8_t bus;
} c148_t;
enum { L_CPU, L_EX, L_POS, L_SCI, L_VBLANK };

static struct namcos2_state {   // (on the heap while the game runs, ADR-040)
	c148_t intc[2];
	uint8_t slave_held, slave_needreset, snd_held, mcu_held;
	uint8_t sendval, soundbank, an_ctrl, an_data, an_complete, c68_mux, pad[2];
	uint16_t gfx_ctrl;
	uint16_t control[0x20];          // the C123's
	uint16_t c116_regs[8];
	uint16_t rozctrl[8];
	uint32_t rng;
	int64_t t_snd;
	uint64_t m68k_frac, snd_frac, mcu_frac, ym_frac, c140_frac, irq120;
	uint8_t ram0[0x10000];           // master 0x100000
	uint8_t ram1[0x40000];           // slave 0x100000
	uint8_t vram[0x10000];           // 0x400000 (host-order words)
	uint8_t spr[0x10000];            // 0xc00000 (Final Lap: 0x800000, 64 KB)
	uint32_t fl_prot_count;
	uint16_t c169ctl[16];
	uint8_t rozram[0x20000];         // 0xc80000 (Final Lap: the C45 road's RAM at 0x880000)
	uint8_t sci[0x4000];             // 0x480000
	uint8_t pal_r[0x2000], pal_g[0x2000], pal_b[0x2000];
	uint8_t dpram[0x800];
	uint8_t nvram[0x2000];
	uint8_t sndram[0x2000];
	uint8_t mcuram[0x40];
	uint8_t gun[8];                  // (the light guns' positions, 0-255)
} *s_bp;
#define b (*s_bp)

// ---- the drawing's copy ----
typedef struct {
	uint16_t control[0x20], c116_regs[8], rozctrl[8], gfx_ctrl, c169ctl[16];
	int pal_dirty;
	uint8_t vram[0x10000];
	uint8_t spr[0x4000];
	uint8_t rozram[0x20000];
	uint8_t pal_r[0x2000], pal_g[0x2000], pal_b[0x2000];
	uint8_t c169ram[0x10000];   // (Lucky & Wild's C169 RAM)
} snap_t;
static snap_t *s_snap;
static int s_pal_dirty, s_pal_valid;
static uint8_t *s_pri;          // VIS_W x VIS_H
static uint16_t *s_sprbuf;      // the sprites' bitmap (VIS_W x VIS_H)

static int32_t *s_mix_l, *s_mix_r;
static int s_mix_n, s_mix_cap;
static uint32_t s_ym_rate;
static int s_c140_gain, s_ym_gain;
// the board: 0 the standard one; 1 Final Lap / Four Trax (C45 road, no ROZ). fl_spr: Final Lap's sprite tile / size
// bits; fl_tile: finalap2 / finalap3's tile bit order
static int s_fl, s_fl_spr, s_fl_tile;
// the C355 sprite boards: 1 Steel Gunner 1 / 2; 2 Suzuka 8 Hours 1 / 2 (with the C45 road); 3 Lucky & Wild (the road
// and the C169 ROZ, its RAM in b.spr)
static int s_sg;
static c355spr_t s_c355, s_c355r;   // (the live one; the drawing's copy)
// Metal Hawk: the C169 ROZ (its RAM in rozram, its registers in b.c169ctl) and its own sprites
static int s_mh;
static c169roz_t s_c169;
static const uint8_t *s_clut;

static inline uint16_t rd16(const uint8_t *mem, uint32_t a) { return *(const uint16_t *)(mem + (a & ~1u)); }
static inline void wr16(uint8_t *mem, uint32_t a, uint16_t v, uint16_t mask)
{
	uint16_t *p = (uint16_t *)(mem + (a & ~1u));
	*p = (uint16_t)((*p & ~mask) | (v & mask));
}

// ---- the C148s ----
static void irq_clear(int cpu, int level)
{
	if (level <= 0)
		return;
	m68k_multi_set_line(cpu, level, 0);
	if (m68k_multi_current() == cpu)
		m68k_bus_clear_irq(level);
}

static void slave_reset_line(int state);
static void snd_reset_line(int state);

static uint8_t c148_r(int cpu, uint32_t off)
{
	c148_t *c = &b.intc[cpu];
	switch (off >> 13) {
	case 0x04 >> 1: return c->bus;                       // 0x04000
	case 0x06 >> 1: return c->level[L_CPU];
	case 0x08 >> 1: return c->level[L_EX];
	case 0x0a >> 1: return c->level[L_POS];
	case 0x0c >> 1: return c->level[L_SCI];
	case 0x0e >> 1: return c->level[L_VBLANK];
	case 0x16 >> 1: irq_clear(cpu, c->level[L_CPU]); return 0;
	case 0x18 >> 1: irq_clear(cpu, c->level[L_EX]); return 0;
	case 0x1a >> 1: irq_clear(cpu, c->level[L_POS]); return 0;
	case 0x1c >> 1: irq_clear(cpu, c->level[L_SCI]); return 0;
	case 0x1e >> 1: irq_clear(cpu, c->level[L_VBLANK]); return 0;
	case 0x20 >> 1: return 7;                            // ext_r (the EEPROM ready)
	default: return 0;
	}
}

static void c148_w(int cpu, uint32_t off, uint8_t v)
{
	c148_t *c = &b.intc[cpu];
	const int r = (int)(off >> 13);
	switch (r) {
	case 0x04 >> 1: c->bus = v & 7; break;
	case 0x06 >> 1: case 0x08 >> 1: case 0x0a >> 1: case 0x0c >> 1: case 0x0e >> 1: {
		static const int which[5] = { L_CPU, L_EX, L_POS, L_SCI, L_VBLANK };
		const int k = which[r - (0x06 >> 1)];
		irq_clear(cpu, c->level[k]);   // (the level write acknowledges first)
		c->level[k] = v & 7;
		break;
	}
	case 0x10 >> 1:   // cpu_irq_assert_w: the other CPU's CPU IRQ
		if (b.intc[cpu ^ 1].level[L_CPU])
			m68k_multi_set_line(cpu ^ 1, b.intc[cpu ^ 1].level[L_CPU], 1);
		break;
	case 0x16 >> 1: irq_clear(cpu, c->level[L_CPU]); break;
	case 0x18 >> 1: irq_clear(cpu, c->level[L_EX]); break;
	case 0x1a >> 1: irq_clear(cpu, c->level[L_POS]); break;
	case 0x1c >> 1: irq_clear(cpu, c->level[L_SCI]); break;
	case 0x1e >> 1: irq_clear(cpu, c->level[L_VBLANK]); break;
	case 0x22 >> 1: if (cpu == 0) snd_reset_line(!(v & 1)); break;     // sound_reset_w
	case 0x24 >> 1: if (cpu == 0) slave_reset_line(!(v & 1)); break;   // system_reset_w (the slave and the C65)
	default: break;
	}
}

static void slave_reset_line(int state)
{
	if (state) {
		b.slave_held = 1;
		b.mcu_held = 1;
	} else {
		if (b.slave_held)
			b.slave_needreset = 1;
		b.slave_held = 0;
		b.mcu_held = 0;
	}
	if (s_is_c68)
		m740_set_reset(&s_c68, state);
	else
		m68705_set_reset_line(&s_mcu, state);
}

static void snd_reset_line(int state)
{
	if (state)
		b.snd_held = 1;
	else if (b.snd_held) {
		b.snd_held = 0;
		m6809_reset(&s_snd);
		s_snd.cycles = b.t_snd;
	}
}

// ---- the key chip (namcos2_68k_key_r / w) ----
static uint16_t rand16(void)
{
	b.rng = b.rng * 1103515245u + 12345u;
	return (uint16_t)(b.rng >> 16);
}

static uint16_t key_r(int off)
{
	switch (s_gametype) {
	case GT_ORDYNE:
		switch (off) { case 2: return 0x1001; case 3: return 0x1; case 4: return 0x110; case 5: return 0x10; case 6: return 0xb0; case 7: return 0xb0; }
		break;
	case GT_MIRNINJA: if (off == 7) return 0xb1; break;
	case GT_PHELIOS:
		switch (off) { case 0: return 0xf0; case 1: return 0xff0; case 2: return 0xb2; case 3: return 0xb2; case 4: return 0xf; case 5: return 0xf00f; case 7: return 0xb2; }
		break;
	case GT_DIRTFOXJ: if (off == 1) return 0xb4; break;
	case GT_FINEHOUR: if (off == 7) return 0xbc; break;
	case GT_BURNFORC: if (off == 1) return 0xbd; break;
	case GT_MARVLAND:
		switch (off) { case 0: return 0x10; case 1: return 0x110; case 4: return 0xbe; case 6: return 0x1001; case 7: return b.sendval ? 0xbe : 1; }
		break;
	case GT_DSABER: if (off == 2) return 0xc0; break;
	case GT_RTHUN2:
		if ((off == 4 || off == 7) && b.sendval) {
			b.sendval = 0;
			return 0x13f;
		}
		if (off == 2) return 0;
		break;
	case GT_COSMOGNG: if (off == 3) return 0x14a; break;
	case GT_SWS: if (off == 4) return 0x142; break;
	case GT_SGUNNER2: if (off == 4) return 0x15a; break;
	case GT_SUZUK8H2: if (off == 3) return 0x14d; if (off == 2) return 0; break;
	case GT_SWS92: if (off == 3) return 0x14b; break;
	case GT_SWS92G: if (off == 3) return 0x14c; break;
	case GT_SWS93: if (off == 3) return 0x14e; break;
	case GT_GOLLYGHO:
		switch (off) { case 0: return 2; case 1: return 2; case 2: return 0; case 4: return 0x143; }
		break;
	case GT_BUBBLETR:
		switch (off) { case 0: return 2; case 1: return 2; case 2: return 0; case 4: return 0x141; }
		break;
	default: break;
	}
	return rand16();
}

static void key_w(int off, uint16_t v)
{
	if (s_gametype == GT_MARVLAND && off == 5 && v == 0x615e) b.sendval = 1;
	if (s_gametype == GT_RTHUN2 && (off == 4 || off == 7) && v == 0x13ec) b.sendval = 1;
	if (s_gametype == GT_MARVLAND && off == 6 && v == 0x1001) b.sendval = 0;
}

// ---- the C116 (namcos2 c116_r and the device's write) ----
static uint8_t c116_r(uint32_t off)
{
	off &= 0x7fff;
	if ((off & 0x1800) == 0x1800) {
		off &= 0x180f;
		if (off > 0x180b) return 0xff;
		const int reg = (int)((off & 0xf) >> 1);
		return (off & 1) ? (uint8_t)b.c116_regs[reg] : (uint8_t)(b.c116_regs[reg] >> 8);
	}
	const uint32_t c = ((off & 0x6000) >> 2) | (off & 0x7ff);
	switch (off & 0x1800) {
	case 0x0000: return b.pal_r[c];
	case 0x0800: return b.pal_g[c];
	default: return b.pal_b[c];
	}
}

static void c116_w(uint32_t off, uint8_t v)
{
	off &= 0x7fff;
	const uint32_t c = ((off & 0x6000) >> 2) | (off & 0x7ff);
	switch (off & 0x1800) {
	case 0x0000: b.pal_r[c] = v; s_pal_dirty = 1; break;
	case 0x0800: b.pal_g[c] = v; s_pal_dirty = 1; break;
	case 0x1000: b.pal_b[c] = v; s_pal_dirty = 1; break;
	default: {
		const int reg = (int)((off & 0xf) >> 1);
		if (off & 1)
			b.c116_regs[reg] = (uint16_t)((b.c116_regs[reg] & 0xff00) | v);
		else
			b.c116_regs[reg] = (uint16_t)((b.c116_regs[reg] & 0x00ff) | (v << 8));
		break;
	}
	}
}

// ---- the 68000s' handlers (what the page table does not cover) ----
static uint16_t bus_r16(uint32_t a)
{
	const int cpu = m68k_multi_current();
	a &= 0xffffff;
	if (a >= 0x1c0000 && a < 0x200000) return c148_r(cpu, a - 0x1c0000);
	if (cpu == 0 && a >= 0x180000 && a < 0x184000) return b.nvram[(a & 0x3fff) >> 1];
	if (a >= 0x420000 && a < 0x420040) return b.control[(a & 0x3f) >> 1];
	if (a >= 0x440000 && a < 0x450000) return c116_r((a & 0xffff) >> 1);
	if (a >= 0x460000 && a < 0x470000) return b.dpram[(a & 0xfff) >> 1];
	if (a >= 0x4a0000 && a < 0x4a0010) return (a & 0xe) == 0 ? 4 : 0;   // the C139's status
	if (s_sg) {
		if (a >= 0x800000 && a < 0x814200) return c355spr_spriteram_r(&s_c355, (a - 0x800000) >> 1);
		if (s_sg >= 2 && a >= 0x900000 && a < 0x900008) return c355spr_position_r(&s_c355, (a & 7) >> 1);
		if (s_sg == 3 && a >= 0xd00000 && a < 0xd00020) return b.c169ctl[(a & 0x1f) >> 1];
		if (s_sg == 1 && a >= 0xa00000 && a < 0xa00010) return key_r((int)((a & 0xf) >> 1));
		if (s_sg >= 2 && a >= 0xf00000 && a < 0xf00008) return key_r((int)((a & 7) >> 1));
		return 0;
	}
	if (s_mh) {
		if (a >= 0xd00000 && a < 0xd00020) return b.c169ctl[(a & 0x1f) >> 1];
		if (a == 0xe00000) return b.gfx_ctrl;
		return 0;
	}
	if (s_fl) {
		if (a >= 0x300000 && a < 0x340000) {   // finallap_prot_r
			static const uint16_t table0[8] = { 0x0000, 0x0040, 0x0440, 0x2440, 0x2480, 0xa080, 0x8081, 0x8041 };
			static const uint16_t table1[8] = { 0x0040, 0x0060, 0x0060, 0x0860, 0x0864, 0x08e4, 0x08e5, 0x08a5 };
			const uint32_t off = (a - 0x300000) >> 1;
			switch (off) {
			case 0: return 0x0101;
			case 1: return 0x3e55;
			case 2: return (uint16_t)((table1[b.fl_prot_count & 7] & 0xff00) >> 8);
			case 3: return (uint16_t)(table1[b.fl_prot_count++ & 7] & 0x00ff);
			case 0x3fffc / 2: return (uint16_t)(table0[b.fl_prot_count & 7] & 0xff00);
			case 0x3fffe / 2: return (uint16_t)((table0[b.fl_prot_count++ & 7] & 0x00ff) << 8);
			default: return 0;
			}
		}
		if (a == 0x840000) return b.gfx_ctrl;
		return 0;
	}
	if (a == 0xc40000) return b.gfx_ctrl;
	if (a >= 0xcc0000 && a < 0xcc0010) return b.rozctrl[(a & 0xf) >> 1];
	if (a >= 0xd00000 && a < 0xd00010) return key_r((int)((a & 0xf) >> 1));
	return 0;
}

static uint8_t bus_r8(uint32_t a)
{
	const uint16_t w = bus_r16(a & ~1u);
	return (a & 1) ? (uint8_t)w : (uint8_t)(w >> 8);
}

static void bus_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	const int cpu = m68k_multi_current();
	a &= 0xffffff;
	if (a >= 0x1c0000 && a < 0x200000) {
		if (mask & 0xff) c148_w(cpu, a - 0x1c0000, (uint8_t)v);
		else if (((a - 0x1c0000) >> 13) == (0x10 >> 1) || ((a - 0x1c0000) >> 13) >= (0x16 >> 1))
			c148_w(cpu, a - 0x1c0000, (uint8_t)(v >> 8));   // (the word-wide handlers: any byte)
		return;
	}
	if (cpu == 0 && a >= 0x180000 && a < 0x184000) {
		if (mask & 0xff) b.nvram[(a & 0x3fff) >> 1] = (uint8_t)v;
		return;
	}
	if (a >= 0x420000 && a < 0x420040) {
		uint16_t *p = &b.control[(a & 0x3f) >> 1];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
		return;
	}
	if (a >= 0x440000 && a < 0x450000) {
		if (mask & 0xff) c116_w((a & 0xffff) >> 1, (uint8_t)v);
		return;
	}
	if (a >= 0x460000 && a < 0x470000) {
		if (mask & 0xff) b.dpram[(a & 0xfff) >> 1] = (uint8_t)v;
		return;
	}
	if (s_sg) {
		if (a >= 0x800000 && a < 0x814200)
			c355spr_spriteram_w(&s_c355, (a - 0x800000) >> 1, v, mask);
		else if (s_sg >= 2 && a >= 0x900000 && a < 0x900008)
			c355spr_position_w(&s_c355, (a & 7) >> 1, v, mask);
		else if (s_sg == 3 && a >= 0xd00000 && a < 0xd00020) {
			uint16_t *q = &b.c169ctl[(a & 0x1f) >> 1];
			*q = (uint16_t)((*q & ~mask) | (v & mask));
		}
		else if (s_sg == 1 && a >= 0xa00000 && a < 0xa00010)
			key_w((int)((a & 0xf) >> 1), v);
		else if (s_sg >= 2 && a >= 0xf00000 && a < 0xf00008)
			key_w((int)((a & 7) >> 1), v);
		return;
	}
	if (s_mh) {
		if (a >= 0xd00000 && a < 0xd00020) {
			uint16_t *q = &b.c169ctl[(a & 0x1f) >> 1];
			*q = (uint16_t)((*q & ~mask) | (v & mask));
		} else if (a == 0xe00000)
			b.gfx_ctrl = (uint16_t)((b.gfx_ctrl & ~mask) | (v & mask));
		return;
	}
	if (a == (s_fl ? 0x840000u : 0xc40000u)) {
		b.gfx_ctrl = (uint16_t)((b.gfx_ctrl & ~mask) | (v & mask));
		return;
	}
	if (s_fl)
		return;
	if (a >= 0xcc0000 && a < 0xcc0010) {
		uint16_t *p = &b.rozctrl[(a & 0xf) >> 1];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
		return;
	}
	if (a >= 0xd00000 && a < 0xd00010) {
		key_w((int)((a & 0xf) >> 1), v);
		return;
	}
}

static void bus_w16(uint32_t a, uint16_t v) { bus_w16m(a, v, 0xffff); }
static void bus_w8(uint32_t a, uint8_t v)
{
	if (a & 1)
		bus_w16m(a & ~1u, v, 0x00ff);
	else
		bus_w16m(a, (uint16_t)(v << 8), 0xff00);
}

// ---- the sound CPU (sound_default_am) ----
static void snd_bank(void)
{
	const uint32_t max = s_sndrom_size / 0x4000 ? s_sndrom_size / 0x4000 : 1;
	const uint32_t base = (rbx_wrap(b.soundbank, max)) * 0x4000;
	for (int p = 0; p < 0x40; p++)
		s_snd.rpage[p] = s_sndrom + base + (uint32_t)p * 256;
}

static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a == 0x4000 || a == 0x4001) return rbx_ym_read(0, a & 1);
	if (a >= 0x5000 && a < 0x7000) return c140_r(&s_c140, a & 0x1ff);
	return 0;
}

static int s_dbg_ymw, s_dbg_c140w, s_dbg_bank;
static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a == 0x4000 || a == 0x4001) s_dbg_ymw++;
	else if (a >= 0x5000 && a < 0x7000) s_dbg_c140w++;
	else if (a == 0xc000 || a == 0xc001) s_dbg_bank = v;
	if (a == 0x4000 || a == 0x4001)
		rbx_ym_write(0, a & 1, v);
	else if (a >= 0x5000 && a < 0x7000)
		c140_w(&s_c140, a & 0x1ff, v);
	else if (a == 0xc000 || a == 0xc001) {
		b.soundbank = v >> 4;
		snd_bank();
	}
}

static void ym_irq(int state) { (void)state; }   // (the YM2151's IRQ is not connected on System 2)
static void c140_irq(void *u, int state) { (void)u; m6809_set_firq(&s_snd, state); }

// ---- the C65 (namco65.cpp mcu_map) ----
// (MAME: the bits no field of a port covers read as 0)
static uint8_t s_cov[64];
static uint8_t port8(int p)
{
	for (int i = 0; i < 8; i++)
		if (s_an_gun[i] && p == s_port_an[i] && p >= 0)
			return b.gun[i];
	return p >= 0 ? (uint8_t)(rbx_port_read(p) & s_cov[p & 63]) : 0xff;
}

static uint8_t mcu_r(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x40) {
		switch (a) {
		case 0x00: return 0;
		case 0x01: return port8(s_port_mcub);
		case 0x02: return port8(s_port_mcuc);
		case 0x03: {   // port D: the analog inputs as bits
			uint8_t d = 0;
			for (int i = 0; i < 8; i++)
				if (port8(s_port_an[i]) > 0x7f) d |= (uint8_t)(1 << i);
			return d;
		}
		case 0x07: return port8(s_port_mcuh);
		case 0x10: {
			if (b.an_complete == 2) b.an_complete = 1;
			return (uint8_t)((b.an_complete ? 0x80 : 0) | (b.an_ctrl & 0x3f));
		}
		case 0x11:
			if (b.an_complete == 1) b.an_complete = 0;
			return b.an_data;
		default: return b.mcuram[a];
		}
	}
	if (a == 0x2000) return port8(s_port_dsw);
	if (a >= 0x3000 && a < 0x3004) return port8(s_port_di[a & 3]);
	if (a >= 0x5000 && a < 0x5800) return b.dpram[a & 0x7ff];
	if (a >= 0x8000) return s_mcuext ? s_mcuext[rbx_wrap(a - 0x8000, s_mcuext_size)] : 0xff;
	return 0;
}

static void mcu_w(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x40) {
		if (a == 0x10) {   // namcos2_mcu_analog_ctrl_w: an instant conversion
			b.an_ctrl = v;
			if (v & 0x40) {
				b.an_complete = 2;
				b.an_data = port8(s_port_an[(v >> 2) & 7]);
				if (v & 0x20) {
					m68705_hd63705_set_input(&s_mcu, HD63705_INT_ADCONV, M68705_ASSERT_LINE);
					m68705_hd63705_set_input(&s_mcu, HD63705_INT_ADCONV, M68705_CLEAR_LINE);
				}
			}
		} else if (a != 0x03 && a != 0x11)
			b.mcuram[a] = v;
	} else if (a >= 0x5000 && a < 0x5800)
		b.dpram[a & 0x7ff] = v;
}

// ---- the C68 (namco68.cpp c68_default_am, the M37450's ports) ----
static uint8_t c68_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a == 0x2000) return port8(s_port_dsw);
	if (a >= 0x3000 && a < 0x3004) return port8(s_port_di[a & 3]);
	if (a >= 0x5000 && a < 0x5800) return b.dpram[a & 0x7ff];
	if (a >= 0x6000 && a < 0x7000) {   // ack_mcu_vbl_r
		m740_set_input(&s_c68, M3745X_INT1_LINE, 0);
		return 0;
	}
	return 0;
}

static void c68_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a >= 0x5000 && a < 0x5800)
		b.dpram[a & 0x7ff] = v;
}

static uint8_t c68_port_r(void *ctx, int port)
{
	(void)ctx;
	if (port == 3) {   // mcuc_r: the nibbles swapped
		const uint8_t c = port8(s_port_mcuc);
		return (uint8_t)((c << 4) | (c >> 4));
	}
	if (port == 5) {   // c68_p5_r: players 1 / 2 by the mux
		const uint16_t r = (uint16_t)(port8(s_port_mcuh) << 8 | port8(s_port_mcub));
		static const uint8_t m1[8] = { 6, 8, 10, 12, 4, 2, 0, 14 }, m0[8] = { 7, 9, 11, 13, 5, 3, 1, 15 };
		const uint8_t *m = b.c68_mux ? m1 : m0;
		uint8_t v = 0;
		for (int i = 0; i < 8; i++)
			v |= (uint8_t)(((r >> m[i]) & 1) << (7 - i));
		return v;
	}
	return 0;
}

static void c68_port_w(void *ctx, int port, uint8_t v)
{
	(void)ctx;
	if (port == 3)
		b.c68_mux = (v & 0x80) ? 1 : 0;
}

static uint8_t c68_ad_r(void *ctx, int ch) { (void)ctx; return port8(s_port_an[ch & 7]); }

// the C68's wait for its interrupt: LDA zp ; BEQ * (A5 zz F0 FC) with the byte 0 and no interrupt request or A/D
// conversion pending (the M740 prefetches: pc is past the opcode in ir). Not exact to the clock: where in the loop it
// stands when the interrupt comes differs by part of one turn
static int c68_idle(void)
{
	const m740_t *c = &s_c68;
	if (c->irq_state || c->adc_busy || c->waiting || c->in_reset)
		return 0;
	uint16_t top;
	if (c->ir == 0xa5)
		top = (uint16_t)(c->pc - 1);
	else if (c->ir == 0xf0)
		top = (uint16_t)(c->pc - 3);
	else
		return 0;
	const uint8_t *pg = c->rpage[top >> 8], *pg2 = c->rpage[(uint16_t)(top + 3) >> 8];
	if (!pg || !pg2)
		return 0;
	if (pg[top & 0xff] != 0xa5 || pg2[(uint16_t)(top + 2) & 0xff] != 0xf0 || pg2[(uint16_t)(top + 3) & 0xff] != 0xfc)
		return 0;
	const uint8_t zz = c->rpage[(uint16_t)(top + 1) >> 8][(uint16_t)(top + 1) & 0xff];
	if (zz >= 0xc0)
		return 0;
	return c->ram[zz] == 0;
}

// ---- drawing ----
#define BANDS 8
static int s_band_next, s_band_left, s_band_active, s_band_helpers;
static int s_cx0, s_cx1, s_cy0, s_cy1;   // (the C116's window this frame, inclusive)
static const int s_adj[4] = { 4, 2, 1, 0 };

static inline uint16_t vword(const snap_t *sn, uint32_t w) { return rd16(sn->vram, (w & 0x7fff) * 2); }

static void draw_c123(const snap_t *sn, int i, int y, int x0, int x1, uint16_t *d, uint8_t *pri, int p)
{
	const uint16_t cbase = (uint16_t)(0x1000 + (sn->control[0x18 + i] & 7) * 0x100);
	const int flip = (sn->control[1] & 0x8000) != 0;
	int ty, sx, cols, base, wmask;
	if (i < 4) {
		int scx = sn->control[i * 4 + 1] & 0x1ff, scy = sn->control[i * 4 + 3] & 0x1ff;
		if (flip) {
			scx = -scx;
			scy = -scy;
			ty = (-25 - y + scy) & 0x1ff;
			sx = -45 - s_adj[i] + scx;
		} else {
			ty = (y + scy + 24) & 0x1ff;
			sx = scx + 44 + s_adj[i];
		}
		cols = 64;
		base = i * 0x1000;
		wmask = 0x1ff;
	} else {
		ty = flip ? 223 - y : y;
		sx = flip ? 287 : 0;
		cols = 36;
		base = i == 4 ? 0x4008 : 0x4408;
		wmask = 0x3ff;
	}
	const int row = (ty >> 3) * cols, fy = ty & 7;
	for (int x = x0; x < x1;) {
		const int tx = flip ? (sx - x) & wmask : i < 4 ? (x + sx) & wmask : x;
		const uint16_t code = vword(sn, (uint32_t)(base + row + (tx >> 3)));
		// TilemapCB: the tile's bits 15..11 from 13 12 11 15 14; the mask by the code itself
		// (finalap2 / finalap3: bits 14..11 from 13 12 11 14, bit 15 dropped)
		const uint32_t tile = rbx_wrap(s_fl_tile ? (uint32_t)((code & 0x07ff) | ((code & 0x3800) << 1) | ((code & 0x4000) >> 3))
		                                         : (uint32_t)((code & 0x07ff) | ((code & 0x3800) << 2) | ((code & 0xc000) >> 3)), s_chr_count);
		const uint8_t m = s_mask[rbx_wrap(code, s_mask_count) * 8 + (uint32_t)fy];
		const uint8_t *src = s_chr + tile * 64 + (uint32_t)fy * 8;
		if (flip) {   // (the tilemap's x going down: this tile's pixels from fx to 0)
			int fx = tx & 7;
			if (!m) {
				x += fx + 1;
				continue;
			}
			for (; fx >= 0 && x < x1; fx--, x++)
				if (m & (0x80 >> fx)) {
					d[x] = (uint16_t)(cbase + src[fx]);
					pri[x] = (uint8_t)p;
				}
			continue;
		}
		int fx = tx & 7;
		if (!m) {
			x += 8 - fx;
			continue;
		}
		if (m == 0xff && fx == 0 && x + 8 <= x1) {   // (an opaque row of the tile)
			for (int k = 0; k < 8; k++) {
				d[x + k] = (uint16_t)(cbase + src[k]);
				pri[x + k] = (uint8_t)p;
			}
			x += 8;
			continue;
		}
		for (; fx < 8 && x < x1; fx++, x++)
			if (m & (0x80 >> fx)) {
				d[x] = (uint16_t)(cbase + src[fx]);
				pri[x] = (uint8_t)p;
			}
	}
}

static void draw_roz(const snap_t *sn, int y, int x0, int x1, uint16_t *d, uint8_t *pri, int p)
{
	int32_t incxx = (int16_t)sn->rozctrl[0], incxy = (int16_t)sn->rozctrl[1], incyx = (int16_t)sn->rozctrl[2],
		incyy = (int16_t)sn->rozctrl[3];
	int32_t startx = (int16_t)sn->rozctrl[4], starty = (int16_t)sn->rozctrl[5];
	uint32_t size = 2048;
	int wrap = 1;
	switch (sn->rozctrl[7]) {
	case 0x4488: case 0x44cc: wrap = 0; break;
	case 0x44ee: wrap = 0; size = 256; break;
	default: break;
	}
	startx <<= 4;
	starty <<= 4;
	startx += 38 * incxx;
	starty += 38 * incxy;
	const uint32_t sx0 = (uint32_t)(startx << 8), sy0 = (uint32_t)(starty << 8);
	const uint32_t ixx = (uint32_t)(incxx << 8), ixy = (uint32_t)(incxy << 8), iyx = (uint32_t)(incyx << 8),
		iyy = (uint32_t)(incyy << 8);
	uint32_t srcx = sx0 + (uint32_t)x0 * ixx + (uint32_t)y * iyx, srcy = sy0 + (uint32_t)x0 * ixy + (uint32_t)y * iyy;
	const uint32_t smask = size - 1;
	const uint16_t color = sn->gfx_ctrl & 0x0f00;
	uint32_t last = 0xffffffffu;   // (the tile last looked up: its cell)
	const uint8_t *tile = NULL;
	for (int x = x0; x < x1; x++, srcx += ixx, srcy += ixy) {
		uint32_t xp = srcx >> 16, yp = srcy >> 16;
		if (wrap) {
			xp &= smask;
			yp &= smask;
		} else if (xp >= size || yp >= size)
			continue;
		const uint32_t cell = (yp >> 3) * 256 + (xp >> 3);
		if (cell != last) {
			last = cell;
			tile = s_roz + rbx_wrap(rd16(sn->rozram, cell * 2), s_roz_count) * 64;
		}
		const uint8_t pix = tile[(yp & 7) * 8 + (xp & 7)];
		if (pix != 0xff) {
			d[x] = (uint16_t)(pix + color);
			pri[x] = (uint8_t)p;
		}
	}
}

// the sprites of the band into s_sprbuf (zdrawgfxzoom), lines y0..y1-1 within the clip
static void draw_sprites(const snap_t *sn, int y0, int y1)
{
	const int offset = (sn->gfx_ctrl & 0xf) * 128 * 4;
	for (int y = y0; y < y1; y++)
		for (int x = s_cx0; x <= s_cx1; x++)
			s_sprbuf[y * VIS_W + x] = 0xffff;
	for (int loop = 0; loop < 128; loop++) {
		const uint32_t o = (uint32_t)(offset + loop * 4) * 2;
		const uint16_t word0 = rd16(sn->spr, o), word1 = rd16(sn->spr, o + 2), offset4 = rd16(sn->spr, o + 4),
			word3 = rd16(sn->spr, o + 6);
		const int sizey = ((word0 >> 10) & 0x3f) + 1;
		const int is_32 = s_fl_spr ? (word1 >> 13) & 1 : (word0 >> 9) & 1;
		int sizex = (word3 >> 10) & 0x3f;
		if (!is_32) sizex >>= 1;
		if (!(sizey - 1) || !sizex)
			continue;
		const int scalex = (sizex << 16) / (is_32 ? 0x20 : 0x10), scaley = (sizey << 16) / (is_32 ? 0x20 : 0x10);
		if (!scalex || !scaley)
			continue;
		const uint32_t prival = s_fl ? (word3 & 0xf) : (word3 & 7);   // (the Final Lap boards: all 4 bits)
		const uint32_t color = (word3 >> 4) & 0xf;
		int sx = (offset4 & 0x7ff) - 0x50 + 0x07, sy = (0x1ff - (word0 & 0x1ff)) - 0x50 + 0x02;
		const int flipy = (word1 >> 15) & 1, flipx = (word1 >> 14) & 1;
		const int gw = is_32 ? 32 : 16, gh = gw;
		const int ox = is_32 ? 0 : ((word1 & 1) ? 16 : 0), oy = is_32 ? 0 : ((word1 & 2) ? 16 : 0);
		const uint32_t sprn = (word1 >> 2) & (s_fl_spr ? 0x7ff : 0xfff);
		const uint8_t *src_base = s_sprpix + rbx_wrap(sprn, s_spr_count) * 1024 + oy * 32 + ox;
		const uint16_t pal = (uint16_t)(256 * color);
		const int sw = (scalex * gw + 0x8000) >> 16, sh = (scaley * gh + 0x8000) >> 16;
		if (!sw || !sh)
			continue;
		int dx = (gw << 16) / sw, dy = (gh << 16) / sh;
		int ex = sx + sw, ey = sy + sh;
		int x_index_base, y_index;
		if (flipx) {
			x_index_base = (sw - 1) * dx;
			dx = -dx;
		} else
			x_index_base = 0;
		if (flipy) {
			y_index = (sh - 1) * dy;
			dy = -dy;
		} else
			y_index = 0;
		const int cminy = s_cy0 > y0 ? s_cy0 : y0, cmaxy = (s_cy1 < y1 - 1 ? s_cy1 : y1 - 1);
		if (sx < s_cx0) {
			x_index_base += (s_cx0 - sx) * dx;
			sx = s_cx0;
		}
		if (sy < cminy) {
			y_index += (cminy - sy) * dy;
			sy = cminy;
		}
		if (ex > s_cx1 + 1) ex = s_cx1 + 1;
		if (ey > cmaxy + 1) ey = cmaxy + 1;
		if (ex <= sx)
			continue;
		for (int y = sy; y < ey; y++, y_index += dy) {
			const uint8_t *src = src_base + (y_index >> 16) * 32;
			uint16_t *dst = s_sprbuf + y * VIS_W;
			int x_index = x_index_base;
			for (int x = sx; x < ex; x++, x_index += dx) {
				const uint8_t c = src[x_index >> 16];
				if (c != 0xff)
					dst[x] = (uint16_t)(((prival & 0xf) << 12) | ((pal + c) & 0xfff));
			}
		}
	}
}

// sgunner_state::sprite_mix_callback_c355
static int sg_mix(void *user, uint16_t *dest, uint8_t *destpri, uint16_t colbase, uint16_t src, int srcpri, int pri)
{
	(void)user;
	(void)pri;
	if (*destpri <= srcpri && (src & 0xff) != 0xff) {
		if (src == 0xffe)
			*dest |= 0x800;
		else
			*dest = (uint16_t)(colbase + src);
		*destpri = (uint8_t)srcpri;
		return 1;
	}
	return 0;
}

// the C45 road (namco_c45road.cpp draw), the line y of the screen, priority p
#define ROADLINE(i) rd16(ram, 0x1fa00 + (uint32_t)(i) * 2)
static void draw_road(const snap_t *sn, int y, int x0, int x1, uint16_t *d, uint8_t *pri, int p)
{
	const uint8_t *ram = sn->rozram;
	int screenx = ROADLINE(y + 15);
	if (p != ((screenx & 0xf000) >> 12))
		return;
	const uint32_t zoomx = ROADLINE(0x200 + y + 15) & 0x3ff;
	if (!zoomx)
		return;
	const uint32_t sourcey = (uint32_t)(ROADLINE(0x100 + y + 15) + ROADLINE(0x1ff)) & (512 * 16 - 1);
	const uint32_t dsourcex = (uint32_t)(1024 << 16) / zoomx;
	if (!dsourcex)
		return;
	screenx = (screenx & 0x800) ? (screenx & 0x0fff) - 0x1000 : (screenx & 0x0fff);
	screenx += -72;
	int numpixels = (int)((uint32_t)(44 * 16 << 16) / dsourcex);
	uint32_t sourcex = 0;
	int clip = x0 - screenx;
	if (clip > 0) {
		numpixels -= clip;
		sourcex += dsourcex * (uint32_t)clip;
		screenx = x0;
	}
	clip = (screenx + numpixels) - x1;
	if (clip > 0)
		numpixels -= clip;
	const uint32_t row = (sourcey >> 4) * 64, fy = sourcey & 15;
	while (numpixels-- > 0) {
		const uint32_t sx = (sourcex >> 16) & 1023;
		const uint16_t data = rd16(ram, (row + (sx >> 4)) * 2);
		// 16x16 2bpp tiles from 0x10000: rows of two words, plane 0 in each word's high byte (the 68000's order)
		const uint32_t tile = data & 0x3ff, fx = sx & 15;
		const uint16_t w = rd16(ram, 0x10000 + (tile * 32 + fy * 2 + (fx >> 3)) * 2);
		const int pix = (((w >> (15 - (fx & 7))) & 1) << 1) | ((w >> (7 - (fx & 7))) & 1);
		int pen = 0xf00 + (data >> 10) * 4 + pix;
		if (s_clut)
			pen = (pen & ~0xff) | s_clut[pen & 0xff];
		d[screenx] = (uint16_t)pen;
		pri[screenx] = (uint8_t)p;
		screenx++;
		sourcex += dsourcex;
	}
}
#undef ROADLINE

// Metal Hawk's ROZ tiles (RozCB_metlhawk)
static void mh_roz_cb(uint16_t code, int *tile, int *mask, int which)
{
	(void)which;
	const int c = code & 0x1fff;
	*mask = code;
	*tile = (c & 0x01ff) | ((c & 0x0e00) << 1) | ((c & 0x1000) >> 3);
}

// RozCB_luckywld
static void lw_roz_cb(uint16_t code, int *tile, int *mask, int which)
{
	(void)which;
	*mask = code;
	int mangle = (code & 0x01ff) | ((code & 0x3000) >> 3);
	switch ((code >> 9) & 7) {
	case 0: mangle += 0x1c00; break;
	case 1: mangle |= 0x0800; break;
	default: break;
	}
	*tile = mangle;
}

static void draw_sprites_mh(const snap_t *sn, int y0, int y1)
{
	for (int y = y0; y < y1; y++)
		for (int x = s_cx0; x <= s_cx1; x++)
			s_sprbuf[y * VIS_W + x] = 0xffff;
	for (int loop = 0; loop < 128; loop++) {
		const uint32_t o = (uint32_t)loop * 16;
		const uint16_t ypos = rd16(sn->spr, o), xpos = rd16(sn->spr, o + 6);
		const int sizey = ((ypos >> 10) & 0x3f) + 1, sizex = (xpos >> 10) & 0x3f;
		if (!(sizey - 1) || !sizex)
			continue;
		const uint16_t attrs = rd16(sn->spr, o + 14), tile = rd16(sn->spr, o + 2), flags = rd16(sn->spr, o + 12);
		const uint32_t prival = attrs & 0xf, color = (attrs >> 4) & 0xf, sprn = (tile >> 2) & 0x0fff;
		const int big = (flags >> 3) & 1;
		int sx = (xpos & 0x03ff) - 0x50 + 0x07, sy = (0x1ff - (ypos & 0x01ff)) - 0x50 + 0x02;
		const int flipx = (flags >> 1) & 1, flipy = (flags >> 2) & 1, swap = flags & 1;
		const int scalex = (sizex << 16) / 0x20, scaley = (sizey << 16) / (big ? 0x20 : 0x10);
		int ox, oy, gw;
		if (big) {
			if (sizex < 0x20) sx -= (0x20 - sizex) / 0x8;
			if (sizey < 0x20) sy += (0x20 - sizey) / 0xc;
			ox = oy = 0;
			gw = 32;
		} else {
			ox = (tile & 1) ? 16 : 0;
			oy = (tile & 2) ? 16 : 0;
			gw = 16;
		}
		if (!scalex || !scaley)
			continue;
		const int gh = gw;
		const uint8_t *tb = s_sprpix + rbx_wrap(sprn, s_spr_count) * 1024;
		const uint16_t pal = (uint16_t)(256 * color);
		const int sw = (scalex * gw + 0x8000) >> 16, sh = (scaley * gh + 0x8000) >> 16;
		if (!sw || !sh)
			continue;
		int dx = (gw << 16) / sw, dy = (gh << 16) / sh;
		int ex = sx + sw, ey = sy + sh, x_index_base, y_index;
		if (flipx) {
			x_index_base = (sw - 1) * dx;
			dx = -dx;
		} else
			x_index_base = 0;
		if (flipy) {
			y_index = (sh - 1) * dy;
			dy = -dy;
		} else
			y_index = 0;
		const int cminy = s_cy0 > y0 ? s_cy0 : y0, cmaxy = (s_cy1 < y1 - 1 ? s_cy1 : y1 - 1);
		if (sx < s_cx0) {
			x_index_base += (s_cx0 - sx) * dx;
			sx = s_cx0;
		}
		if (sy < cminy) {
			y_index += (cminy - sy) * dy;
			sy = cminy;
		}
		if (ex > s_cx1 + 1) ex = s_cx1 + 1;
		if (ey > cmaxy + 1) ey = cmaxy + 1;
		if (ex <= sx)
			continue;
		for (int y = sy; y < ey; y++, y_index += dy) {
			uint16_t *dst = s_sprbuf + y * VIS_W;
			const int ty = oy + (y_index >> 16);
			int x_index = x_index_base;
			for (int x = sx; x < ex; x++, x_index += dx) {
				const int tx = ox + (x_index >> 16);
				// (gfx 1, metlhawk_sprite_layout_swapped: x and y exchanged)
				const uint8_t c = swap ? tb[tx * 32 + ty] : tb[ty * 32 + tx];
				if (c != 0xff)
					dst[x] = (uint16_t)(((prival & 0xf) << 12) | ((pal + c) & 0xfff));
			}
		}
	}
}

static void run_band(int band)
{
	const snap_t *sn = s_snap;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const int y0 = VIS_H * band / BANDS, y1 = VIS_H * (band + 1) / BANDS;
	for (int y = y0; y < y1; y++) {
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < VIS_W; x++)
			d[x] = PAL_BLACK;
	}
	int cy0 = s_cy0 > y0 ? s_cy0 : y0, cy1 = s_cy1 + 1 < y1 ? s_cy1 + 1 : y1;
	if (s_cx0 > s_cx1 || cy0 >= cy1)
		return;
	const int rozpri = (sn->gfx_ctrl & 0x7000) >> 12;
	memset(s_pri + cy0 * VIS_W, 0, (size_t)(cy1 - cy0) * VIS_W);
	for (int y = cy0; y < cy1; y++) {
		uint16_t *d = bm + y * pitch;
		uint8_t *pri = s_pri + y * VIS_W;
		if (s_sg == 1) {   // screen_update_sgunner: the tilemaps 0-7
			for (int p = 0; p < 8; p++)
				for (int i = 0; i < 6; i++) {
					const uint16_t c = sn->control[0x10 + i];
					if ((c & 7) == p && !(c & 8))
						draw_c123(sn, i, y, s_cx0, s_cx1 + 1, d, pri, p);
				}
			continue;
		}
		if (s_mh) {   // screen_update_metlhawk: priorities 0-15, the tilemaps on the even ones, the ROZ on all
			c169roz_layer lp[2];
			c169roz_line_params(&s_c169, sn->c169ctl, (const uint16_t *)sn->rozram, y, lp);
			for (int p = 0; p < 16; p++) {
				if (!(p & 1))
					for (int i = 0; i < 6; i++) {
						const uint16_t c = sn->control[0x10 + i];
						if ((c & 7) == p / 2 && !(c & 8))
							draw_c123(sn, i, y, s_cx0, s_cx1 + 1, d, pri, p);
					}
				c169roz_draw_line_p(&s_c169, lp, (const uint16_t *)sn->rozram, y, s_cx0, s_cx1 + 1, d, pri, p, (uint8_t)p);
			}
			continue;
		}
		if (s_fl || s_sg >= 2) {   // screen_update_finallap / luckywld: priorities 0-15, the tilemaps on the even ones, the road on all
			for (int p = 0; p < 16; p++) {
				if (!(p & 1))
					for (int i = 0; i < 6; i++) {
						const uint16_t c = sn->control[0x10 + i];
						if ((c & 7) == p / 2 && !(c & 8))
							draw_c123(sn, i, y, s_cx0, s_cx1 + 1, d, pri, p);
					}
				draw_road(sn, y, s_cx0, s_cx1 + 1, d, pri, p);
				if (s_sg == 3)
					c169roz_draw_line(&s_c169, sn->c169ctl, (const uint16_t *)sn->c169ram, y, s_cx0, s_cx1 + 1, d, pri, p, (uint8_t)p);
			}
			continue;
		}
		for (int p = 0; p < 8; p++) {
			for (int i = 0; i < 6; i++) {
				const uint16_t c = sn->control[0x10 + i];
				if ((c & 7) == p && !(c & 8))
					draw_c123(sn, i, y, s_cx0, s_cx1 + 1, d, pri, p);
			}
			if (rozpri == p)
				draw_roz(sn, y, s_cx0, s_cx1 + 1, d, pri, p);
		}
	}
	if (s_sg) {   // the C355's bitmap (made before the bands) mixed in
		const c355spr_rect clip = { s_cx0, s_cx1, cy0, cy1 - 1 };
		c355spr_copy(&s_c355r, bm, pitch, s_pri, VIS_W, &clip, 0);
		return;
	}
	if (s_mh)
		draw_sprites_mh(sn, cy0, cy1);
	else
		draw_sprites(sn, cy0, cy1);
	for (int y = cy0; y < cy1; y++) {   // (copybitmap with the mix callback)
		uint16_t *d = bm + y * pitch;
		uint8_t *pri = s_pri + y * VIS_W;
		const uint16_t *src = s_sprbuf + y * VIS_W;
		for (int x = s_cx0; x <= s_cx1; x++) {
			const uint16_t sv = src[x];
			if (sv == 0xffff)
				continue;
			const uint8_t sp = (uint8_t)(sv >> 12);
			const uint16_t c = sv & 0xfff;
			if (pri[x] <= sp && (c & 0xff) != 0xff) {
				if (c == 0xffe)
					d[x] = (d[x] & 0x1000) ? (uint16_t)(d[x] | 0x800) : PAL_BLACK;
				else
					d[x] = c;
				pri[x] = sp;
			}
		}
	}
}

static void ns2_render(void)
{
	snap_t *sn = s_snap;
	if (sn->pal_dirty || !s_pal_valid) {
		uint32_t *pal = rbx_video.palette;
		for (int i = 0; i < 0x2000; i++)
			pal[i] = (uint32_t)(sn->pal_r[i] << 16 | sn->pal_g[i] << 8 | sn->pal_b[i]);
		pal[PAL_BLACK] = 0;
		sn->pal_dirty = 0;
		s_pal_valid = 1;
	}
	s_cx0 = sn->c116_regs[0] - 0x4a;
	s_cx1 = sn->c116_regs[1] - 0x4a - 1;
	s_cy0 = sn->c116_regs[2] - 0x21;
	s_cy1 = sn->c116_regs[3] - 0x21 - 1;
	if (s_cx0 < 0) s_cx0 = 0;
	if (s_cx1 > VIS_W - 1) s_cx1 = VIS_W - 1;
	if (s_cy0 < 0) s_cy0 = 0;
	if (s_cy1 > VIS_H - 1) s_cy1 = VIS_H - 1;
	if (s_sg && s_cx0 <= s_cx1 && s_cy0 <= s_cy1) {   // (the C355's sprites: the whole frame's bitmap first)
		const c355spr_rect clip = { s_cx0, s_cx1, s_cy0, s_cy1 };
		c355spr_build_list_and_render(&s_c355r, &clip);
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

static int ns2_render_help(int helper)
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

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	snap_t *sn = s_snap;
	memcpy(sn->control, b.control, sizeof(b.control));
	memcpy(sn->c116_regs, b.c116_regs, sizeof(b.c116_regs));
	memcpy(sn->rozctrl, b.rozctrl, sizeof(b.rozctrl));
	sn->gfx_ctrl = b.gfx_ctrl;
	memcpy(sn->vram, b.vram, sizeof(b.vram));
	memcpy(sn->spr, b.spr, sizeof(sn->spr));   // (the 16 sprite banks)
	if (s_sg) {   // (the C355's RAM and position registers for the drawing)
		memcpy(s_c355r.ram[0], s_c355.ram[0], sizeof(s_c355.ram[0]));
		memcpy(s_c355r.position, s_c355.position, sizeof(s_c355.position));
	}
	memcpy(sn->c169ctl, b.c169ctl, sizeof(b.c169ctl));
	if (s_sg == 3)
		memcpy(sn->c169ram, b.spr, sizeof(sn->c169ram));
	if (s_fl || s_mh || s_sg >= 2 || ((b.gfx_ctrl & 0x7000) >> 12) != 0)   // (the ROZ layer drawn: its RAM; the road's)
		memcpy(sn->rozram, b.rozram, sizeof(b.rozram));
	if (s_pal_dirty) {
		memcpy(sn->pal_r, b.pal_r, sizeof(b.pal_r));
		memcpy(sn->pal_g, b.pal_g, sizeof(b.pal_g));
		memcpy(sn->pal_b, b.pal_b, sizeof(b.pal_b));
		sn->pal_dirty = 1;
		s_pal_dirty = 0;
	}
	rbx_video_frame_done();
}

// ---- the sound CPU's wait: BRA * (20 fe), its work all in its interrupts ----
// Exact: the slice runs it alone, its interrupt lines change between slices; at the loop's top with no interrupt
// taken there, it spins to the slice's end (3 clocks a turn).
static int64_t s_snd_tgt;

static void snd_idle_hook(m6809_t *c)
{
	const uint8_t *p = c->rpage[c->pc >> 8];
	const uint16_t n1 = (uint16_t)(c->pc + 1);
	const uint8_t *p1 = c->rpage[n1 >> 8];
	if (!p || !p1 || p[c->pc & 0xff] != 0x20 || p1[n1 & 0xff] != 0xfe)
		return;
	const int64_t left = s_snd_tgt - 1 - c->cycles;
	if (left >= 3)
		c->cycles += (left / 3) * 3;
}

// ---- the 68000s' wait loops ----
// A slice runs one CPU alone and the interrupt lines only change between slices, so a CPU found (at a slice's start)
// inside a loop that only reads plain memory, with its exit condition false and no interrupt it would take, would
// spin through the whole slice: the slice is skipped. (Not exact to the clock: where in the loop the CPU stands when
// the condition changes differs by part of one turn.) Forms:
//   b0b9 aaaa aaaa  67f8 / 6700 fff8    CMP.L abs,D0 ; BEQ  (loops while (abs).l == D0)
//   4a79 aaaa aaaa  0c79 iiii aaaa aaaa  66f6   TST.W abs ; CMPI.W #i,abs ; BNE  (loops while (abs).w != i)
//   4a79 aaaa aaaa  67f8                 TST.W abs ; BEQ  (loops while (abs).w == 0)
//   24bc 0000 0000  33c0 001e 6000  0c79 iiii aaaa aaaa  66ea   MOVE.L #0,(A2) ; MOVE.W D0,watchdog ;
//                                   CMPI.W #i,abs ; BNE  (loops while (abs).w != i; (A2) already 0, A2 not in abs)
static int s_noidle;

static int plain_page(uint32_t a) { return m68k_bus_cur->rd[(a & 0xffffff) >> M68K_PAGE_BITS] != NULL; }

static uint16_t key_r(int off);

// A loop of reads and tests only: from pc, TST / CMPI / BTST of plain memory, LEA and branches (EQ / NE / BRA) are
// followed with the flags they set; coming back to pc within a few steps, nothing changed, the CPU spins there
static int walk_idle(const uint8_t *r, uint32_t pc)
{
	uint32_t areg[8];
	for (int i = 0; i < 8; i++)
		areg[i] = m68k_get_reg(NULL, (m68k_register_t)(M68K_REG_A0 + i));
	uint32_t at = pc;
	int z = -1;   // (the Z flag: -1 unknown)
	for (int step = 0; step < 24; step++) {
		if (at >= 0x40000 - 12)
			return 0;
		const uint16_t op = rd16(r, at);
		if (op == 0x4a39 || op == 0x4a79 || op == 0x4ab9) {   // TST.B / .W / .L abs.L
			const uint32_t a = ((uint32_t)rd16(r, at + 2) << 16 | rd16(r, at + 4)) & 0xffffff;
			const int wd = (a >= 0x1e6000 && a < 0x1e8000);   // (the C148's watchdog reads 0)
			if (!wd && !plain_page(a))
				return 0;
			const uint32_t v = wd ? 0 : op == 0x4a39 ? m68k_read_memory_8(a) : op == 0x4a79 ? m68k_read_memory_16(a) : m68k_read_memory_32(a);
			z = v == 0;
			at += 6;
		} else if ((op & 0xfff8) == 0x4a68) {   // TST.W d16(An)
			const uint32_t a = (areg[op & 7] + (uint32_t)(int16_t)rd16(r, at + 2)) & 0xffffff;
			if (!plain_page(a))
				return 0;
			z = m68k_read_memory_16(a) == 0;
			at += 4;
		} else if (op == 0x0c79) {   // CMPI.W #i,abs.L
			const uint32_t a = ((uint32_t)rd16(r, at + 4) << 16 | rd16(r, at + 6)) & 0xffffff;
			if (!plain_page(a))
				return 0;
			z = m68k_read_memory_16(a) == rd16(r, at + 2);
			at += 8;
		} else if (op == 0x0839) {   // BTST #n,abs.L
			const uint32_t a = ((uint32_t)rd16(r, at + 4) << 16 | rd16(r, at + 6)) & 0xffffff;
			if (!plain_page(a & ~1u))
				return 0;
			z = !((m68k_read_memory_8(a) >> (rd16(r, at + 2) & 7)) & 1);
			at += 8;
		} else if ((op & 0xf1ff) == 0x41f9) {   // LEA abs.L,An
			areg[(op >> 9) & 7] = ((uint32_t)rd16(r, at + 2) << 16 | rd16(r, at + 4)) & 0xffffff;
			at += 6;
		} else if ((op & 0xff00) == 0x6000 || (op & 0xff00) == 0x6600 || (op & 0xff00) == 0x6700) {   // BRA / BNE / BEQ
			const int8_t d8 = (int8_t)(op & 0xff);
			const uint32_t target = d8 ? at + 2 + (uint32_t)(int32_t)d8 : at + 2 + (uint32_t)(int32_t)(int16_t)rd16(r, at + 2);
			const uint32_t next = d8 ? at + 2 : at + 4;
			int taken;
			if ((op & 0xff00) == 0x6000)
				taken = 1;
			else {
				if (z < 0)
					return 0;
				taken = (op & 0xff00) == 0x6700 ? z : !z;
			}
			at = taken ? target : next;
		} else
			return 0;
		if (at == pc)
			return 1;
	}
	return 0;
}

static int idle_now(int c, int mc)
{
	const uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC) & 0xffffff;
	if (pc >= 0x40000)
		return 0;
	{	// (an interrupt it would take: run)
		const uint32_t lines = m68k_bus_cur->irq_pending | m68k_bus_cur->irq_asserted;
		int level = 0;
		for (int l = 7; l > 0; l--)
			if (lines & (1u << l)) {
				level = l;
				break;
			}
		if (level && (level == 7 || level > (int)((m68k_get_reg(NULL, M68K_REG_SR) >> 8) & 7)))
			return 0;
	}
	const uint8_t *r = s_rom[c];
	uint32_t pc_scan = pc;
	{	// (stopped inside the RNG routine of the JSR form: from its return address)
		uint32_t s0 = 0xffffffffu;
		for (uint32_t k = 0; k <= 18 && k <= pc; k += 2)
			if (rd16(r, pc - k) == 0x202e && rd16(r, pc - k + 4) == 0x2200 && rd16(r, pc - k + 18) == 0x4e75) {
				s0 = pc - k;
				break;
			}
		if (s0 < 0x40000 - 20) {
			const uint32_t sp = m68k_get_reg(NULL, M68K_REG_A7) & 0xffffff;
			if (!plain_page(sp) || !plain_page(sp + 2))
				return 0;
			const uint32_t ret = m68k_read_memory_32(sp) & 0xffffff;
			if (ret < 6 || ret >= 0x40000)
				return 0;
			pc_scan = ret - 6;
		}
	}
	{	// (stopped inside the key routine of the BSR form: from its return address)
		const uint32_t s0 = rd16(r, pc) == 0x3039 ? pc : rd16(r, pc) == 0x4e75 && pc >= 6 ? pc - 6 : 0xffffffffu;
		if (s0 < 0x40000 - 8 && rd16(r, s0) == 0x3039 && rd16(r, s0 + 2) == 0x00d0 && rd16(r, s0 + 6) == 0x4e75) {
			const uint32_t sp = m68k_get_reg(NULL, M68K_REG_A7) & 0xffffff;
			if (!plain_page(sp) || !plain_page(sp + 2))
				return 0;
			const uint32_t ret = m68k_read_memory_32(sp) & 0xffffff;
			if (ret < 10 || ret >= 0x40000)
				return 0;
			pc_scan = ret - 10;   // (the loop's top: handled as if there)
		}
	}
	for (int back = 0; back <= 20; back += 2) {
		const uint32_t t = pc_scan - (uint32_t)back;
		if (t >= 0x40000 - 16)
			continue;
		const uint16_t w0 = rd16(r, t);
		if (w0 == 0xb0b9) {   // CMP.L abs,D0
			const uint32_t a = (uint32_t)rd16(r, t + 2) << 16 | rd16(r, t + 4);
			const uint16_t w3 = rd16(r, t + 6);
			const int len = w3 == 0x67f8 ? 8 : (w3 == 0x6700 && rd16(r, t + 8) == 0xfff8) ? 10 : 0;
			if (!len || pc - t >= (uint32_t)len || !plain_page(a) || !plain_page(a + 2))
				continue;
			return m68k_read_memory_32(a) == m68k_get_reg(NULL, M68K_REG_D0);
		}
		if (w0 == 0x4a79 && rd16(r, t + 6) == 0x4a79 && rd16(r, t + 12) == 0x67f2) {   // TST.W wd ; TST.W abs ; BEQ
			const uint32_t ta = (uint32_t)rd16(r, t + 2) << 16 | rd16(r, t + 4);
			const uint32_t a = (uint32_t)rd16(r, t + 8) << 16 | rd16(r, t + 10);
			if (pc - t >= 14 || !plain_page(a) || !(plain_page(ta) || (ta >= 0x1e6000 && ta < 0x1e8000)))
				continue;
			return m68k_read_memory_16(a) == 0;
		}
		if (w0 == 0x4a79 && rd16(r, t + 6) == 0x6100 && rd16(r, t + 10) == 0x4a79 && rd16(r, t + 16) == 0x67ee) {
			// TST.W wd ; BSR (MOVE.W key,D0 ; RTS) ; TST.W abs ; BEQ (92 clocks a turn): the key chip's reads made
			// for the turns skipped, D0 its last value
			const uint32_t ta = (uint32_t)rd16(r, t + 2) << 16 | rd16(r, t + 4);
			const uint32_t sub = t + 8 + (uint32_t)(int16_t)rd16(r, t + 8);
			const uint32_t a = (uint32_t)rd16(r, t + 12) << 16 | rd16(r, t + 14);
			if (pc_scan - t >= 18 || sub >= 0x40000 - 8 || rd16(r, sub) != 0x3039 || rd16(r, sub + 2) != 0x00d0 || rd16(r, sub + 6) != 0x4e75 ||
			    !plain_page(a) || !(plain_page(ta) || (ta >= 0x1e6000 && ta < 0x1e8000)))
				continue;
			const uint32_t ka = 0xd00000 | rd16(r, sub + 4);
			if (ka >= 0xd00010 || m68k_read_memory_16(a) != 0)
				return 0;
			int n = mc / 92;
			if (n < 1)
				n = 1;
			uint16_t v = 0;
			for (int i = 0; i < n; i++)
				v = key_r((int)((ka & 0xf) >> 1));
			m68k_set_reg(M68K_REG_D0, (m68k_get_reg(NULL, M68K_REG_D0) & 0xffff0000u) | v);
			return 1;
		}
		if ((w0 & 0xfff8) == 0x4a68 && rd16(r, t + 4) == 0x67fa) {   // TST.W d16(An) ; BEQ (loops while 0)
			const uint32_t a = (m68k_get_reg(NULL, (m68k_register_t)(M68K_REG_A0 + (w0 & 7))) + (uint32_t)(int16_t)rd16(r, t + 2)) & 0xffffff;
			if (pc - t >= 6 || !plain_page(a))
				continue;
			return m68k_read_memory_16(a) == 0;
		}
		if (w0 == 0x4df9 && rd16(r, t + 6) == 0x4bf9 && rd16(r, t + 12) == 0x4a6e && rd16(r, t + 16) == 0x67ee) {
			// LEA abs,A6 ; LEA abs,A5 ; TST.W d16(A6) ; BEQ  (the LEAs the same each turn)
			const uint32_t a6 = (uint32_t)rd16(r, t + 2) << 16 | rd16(r, t + 4);
			const uint32_t a = (a6 + (uint32_t)(int16_t)rd16(r, t + 14)) & 0xffffff;
			if (pc - t >= 18 || pc != t || !plain_page(a))
				continue;
			return m68k_read_memory_16(a) == 0;
		}
		if (w0 == 0x4eb9 && rd16(r, t + 6) == 0x4a6e && rd16(r, t + 10) == 0x67f4) {
			// JSR rng ; TST.W d16(A6) ; BEQ (126 clocks a turn), rng: seed = seed * 5 + 1 at d(A6) (D1 the old, D0 the
			// new): the seed stepped for the turns skipped
			const uint32_t sub = (uint32_t)rd16(r, t + 2) << 16 | rd16(r, t + 4);
			if (pc_scan - t >= 12 || sub >= 0x40000 - 20 || rd16(r, sub) != 0x202e || rd16(r, sub + 4) != 0x2200 ||
			    rd16(r, sub + 6) != 0xd080 || rd16(r, sub + 8) != 0xd080 || rd16(r, sub + 10) != 0xd081 || rd16(r, sub + 12) != 0x5280 ||
			    rd16(r, sub + 14) != 0x2d40 || rd16(r, sub + 16) != rd16(r, sub + 2) || rd16(r, sub + 18) != 0x4e75)
				continue;
			const uint32_t a6 = m68k_get_reg(NULL, M68K_REG_A6);
			const uint32_t fa = (a6 + (uint32_t)(int16_t)rd16(r, t + 8)) & 0xffffff;
			const uint32_t sa = (a6 + (uint32_t)(int16_t)rd16(r, sub + 2)) & 0xffffff;
			if (!plain_page(fa) || !plain_page(sa) || !plain_page(sa + 2) || m68k_read_memory_16(fa) != 0)
				return 0;
			int n = mc / 126;
			if (n < 1)
				n = 1;
			uint32_t seed = m68k_read_memory_32(sa), old = seed;
			for (int i = 0; i < n; i++) {
				old = seed;
				seed = seed * 5 + 1;
			}
			m68k_write_memory_32(sa, seed);
			m68k_set_reg(M68K_REG_D0, seed);
			m68k_set_reg(M68K_REG_D1, old);
			return 1;
		}
		if (w0 == 0x4a79 && rd16(r, t + 6) == 0x67f8) {
			const uint32_t a = (uint32_t)rd16(r, t + 2) << 16 | rd16(r, t + 4);
			if (pc - t >= 8 || !plain_page(a))
				continue;
			return m68k_read_memory_16(a) == 0;
		}
		if (w0 == 0x24bc && rd16(r, t + 2) == 0 && rd16(r, t + 4) == 0 && rd16(r, t + 6) == 0x33c0 && rd16(r, t + 8) == 0x001e &&
		    rd16(r, t + 10) == 0x6000 && rd16(r, t + 12) == 0x0c79 && rd16(r, t + 20) == 0x66ea) {
			const uint32_t a = (uint32_t)rd16(r, t + 16) << 16 | rd16(r, t + 18);
			const uint32_t a2 = m68k_get_reg(NULL, M68K_REG_A2) & 0xffffff;
			if (pc - t >= 22 || !plain_page(a) || !plain_page(a2) || m68k_read_memory_32(a2) != 0 || (a2 <= a + 1 && a <= a2 + 3))
				continue;
			return m68k_read_memory_16(a) != rd16(r, t + 14);
		}
		if (w0 == 0x4a79 && rd16(r, t + 6) == 0x0c79 && rd16(r, t + 14) == 0x66f6) {
			const uint32_t ta = (uint32_t)rd16(r, t + 2) << 16 | rd16(r, t + 4);
			const uint32_t a = (uint32_t)rd16(r, t + 10) << 16 | rd16(r, t + 12);
			if (pc - t >= 16 || !plain_page(a) || !(plain_page(ta) || (ta >= 0x1e6000 && ta < 0x1e8000)))
				continue;
			return m68k_read_memory_16(a) != rd16(r, t + 8);
		}
	}
	static int nowalk = -1;
	if (nowalk < 0) nowalk = getenv("NS2_NOWALK") != NULL;
	return nowalk ? 0 : walk_idle(r, pc);   // (any other loop of reads and tests)
}

// ---- frame ----
static uint32_t *s_pcprof;   // (debug: NS2_PCPROF=1, the 68000s' PCs at the ends of their slices, listed at exit)
static void ns2_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	s_mix_n = 0;
	for (int i = 0; i < 8; i++)
		if (s_an_gun[i]) {
			int v = rbx_analog(s_an_player[i], s_an_gun[i]);
			v = b.gun[i] + (s_an_rev[i] ? -v : v);
			b.gun[i] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
		}
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_H)
			vblank_capture();
		if (line == 200 && !b.mcu_held) {   // (the I/O MCU's IRQ a little before vblank)
			if (s_is_c68)
				m740_set_input(&s_c68, M3745X_INT1_LINE, 1);
			else
				m68705_hd63705_set_input(&s_mcu, HD63705_INT_IRQ1, M68705_HOLD_LINE);
		}
		if (line == 240)
			for (int c = 0; c < 2; c++)
				if (b.intc[c].level[L_VBLANK])
					m68k_multi_hold_irq(c, b.intc[c].level[L_VBLANK]);
		if (line == ((b.c116_regs[5] - 32) & 0xff))
			for (int c = 0; c < 2; c++)
				if (b.intc[c].level[L_POS])
					m68k_multi_set_line(c, b.intc[c].level[L_POS], 1);
		// the 68000s (one slice a line, about MAME's 12 kHz quantum)
		const int mc = rbx_line_cycles(&b.m68k_frac, M68K_CLOCK, FRAME_MHZ, LINES);
		m68k_multi_switch(0);
		if (s_noidle || !idle_now(0, mc)) {
			m68k_execute(mc);
			if (s_pcprof)
				s_pcprof[(m68k_get_reg(NULL, M68K_REG_PC) >> 1) & 0x1ffff]++;
		}
		if (!b.slave_held) {
			if (b.slave_needreset) {
				m68k_multi_reset_cpu(1);
				b.slave_needreset = 0;
			}
			m68k_multi_switch(1);
			if (s_noidle || !idle_now(1, mc)) {
				m68k_execute(mc);
				if (s_pcprof)
					s_pcprof[0x20000 + ((m68k_get_reg(NULL, M68K_REG_PC) >> 1) & 0x1ffff)]++;
			}
		}
		// the sound CPU: IRQ at 120 Hz
		b.irq120 += 120 * 1000;
		if (b.irq120 >= (uint64_t)FRAME_MHZ * LINES) {
			b.irq120 -= (uint64_t)FRAME_MHZ * LINES;
			m6809_hold_irq(&s_snd);
		}
		const int sc = rbx_line_cycles(&b.snd_frac, SND_CLOCK, FRAME_MHZ, LINES);
		b.t_snd += sc;
		if (b.snd_held)
			s_snd.cycles = b.t_snd;
		else {
			s_snd_tgt = b.t_snd;
			m6809_run_until(&s_snd, b.t_snd);
			if (s_snd.idle_hook && s_snd.pc != s_snd.idle_pc[0]) {   // (stopped in a BRA *: its address)
				const uint8_t *p = s_snd.rpage[s_snd.pc >> 8];
				if (p && p[s_snd.pc & 0xff] == 0x20 && (s_snd.pc & 0xff) != 0xff && p[(s_snd.pc & 0xff) + 1] == 0xfe)
					s_snd.idle_pc[0] = s_snd.pc;
			}
		}
		if (s_pcprof)
			s_pcprof[0x40000 + s_snd.pc]++;
		if (s_is_c68) {
			if (!b.mcu_held) {
				const int mcyc = rbx_line_cycles(&b.mcu_frac, 2048000, FRAME_MHZ, LINES);
				if (s_pcprof)   // (the C68's PC every few clocks)
					for (int q = 0; q < mcyc; q += 8) {
						s_pcprof[0x50000 + s_c68.pc]++;
						m740_execute(&s_c68, 8);
					}
				else if (!s_noidle && c68_idle())
					s_c68.cycles += mcyc;   // (its wait loop: the line skipped)
				else
					m740_execute(&s_c68, mcyc);
			}
		} else if (!b.mcu_held) {
			static int tr = -1;
			static int fcount;
			if (line == 0) fcount++;
			if (tr < 0) tr = getenv("NS2_MCUTR") ? 1 : 0;
			if (tr > 0 && fcount >= 100 && fcount < 102) {
				int left = rbx_line_cycles(&b.mcu_frac, MCU_CLOCK, FRAME_MHZ, LINES);
				while (left > 0) {
					rbx_log("T %04x", s_mcu.st.pc);
					left -= m68705_execute(&s_mcu, 1);
				}
			} else
			m68705_execute(&s_mcu, rbx_line_cycles(&b.mcu_frac, MCU_CLOCK, FRAME_MHZ, LINES));
		}
		c140_timer_advance(&s_c140, rbx_line_cycles(&b.c140_frac, 1398101333u, FRAME_MHZ, LINES));
		// the sound of this line
		const uint32_t yc = (uint32_t)rbx_line_cycles(&b.ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
		const int k = rbx_ym_advance_lr(0, yc, s_mix_l + s_mix_n, s_mix_r + s_mix_n, s_mix_cap - s_mix_n);
		if (k > 0) {
			c140_render(&s_c140, s_mix_l + s_mix_n, s_mix_r + s_mix_n, k, s_ym_rate);
			s_mix_n += k;
		}
	}
	m68k_multi_switch(0);
	if (getenv("NS2_DBG")) {
		static int fr;
		if (getenv("NS2_MCUPC")) rbx_log("m %d %04x", fr, s_mcu.st.pc);
		if (++fr % 100 == 0) {
			const unsigned pc0 = m68k_get_reg(NULL, M68K_REG_PC), sr0 = m68k_get_reg(NULL, M68K_REG_SR);
			m68k_multi_switch(1);
			const unsigned pc1 = m68k_get_reg(NULL, M68K_REG_PC);
			m68k_multi_switch(0);
			{
				char hx[128]; int n = 0;
				for (int k = -8; k < 8; k++) n += snprintf(hx + n, sizeof(hx) - n, "%04x ", rd16(s_rom[0], (pc0 + k * 2) & 0x3ffff));
				rbx_log("code %s", hx);
				m68k_multi_switch(1);
				const unsigned q = m68k_get_reg(NULL, M68K_REG_PC);
				m68k_multi_switch(0);
				n = 0;
				for (int k = -8; k < 8; k++) n += snprintf(hx + n, sizeof(hx) - n, "%04x ", rd16(s_rom[1], (q + k * 2) & 0x3ffff));
				rbx_log("code1 %s", hx);
				n = 0;
				for (int k = 0; k < 0x40; k++) n += snprintf(hx + n, sizeof(hx) - n, "%02x", b.dpram[k]);
				rbx_log("dp %s", hx);
				rbx_log("snd writes ym %d c140 %d bank %02x sndrom %u", s_dbg_ymw, s_dbg_c140w, s_dbg_bank, s_sndrom_size);
				rbx_log("ports b %02x c %02x h %02x dsw %02x di %02x %02x %02x %02x an %02x ram b0 %02x", port8(s_port_mcub), port8(s_port_mcuc),
				        port8(s_port_mcuh), port8(s_port_dsw), port8(s_port_di[0]), port8(s_port_di[1]), port8(s_port_di[2]), port8(s_port_di[3]),
				        port8(s_port_an[0]), s_mcu.zst.ram[0xb0 - 0x40]);
			}
			rbx_log("f%d pc0 %06x sr %04x pc1 %06x held %d/%d/%d snd pc %04x mcu pc %04x lv %d%d%d%d%d c116 %04x %04x %04x %04x",
			        fr, pc0, sr0, pc1, b.slave_held, b.snd_held, b.mcu_held, s_snd.pc, s_mcu.st.pc,
			        b.intc[0].level[0], b.intc[0].level[1], b.intc[0].level[2], b.intc[0].level[3], b.intc[0].level[4],
			        b.c116_regs[0], b.c116_regs[1], b.c116_regs[2], b.c116_regs[3]);
		}
	}
	rbx_ym_async_take_lr(0, s_mix_l, s_mix_r, s_mix_n);
	for (int i = 0; i < s_mix_n; i++) {
		rbx_audio_put_stereo(s_mix_l[i], s_mix_r[i]);
		s_mix_l[i] = s_mix_r[i] = 0;
	}
}

// ---- init / reset ----
static void ns2_reset(void)
{
	memset(b.intc, 0, sizeof(b.intc));
	b.sendval = 0;
	b.soundbank = 0;
	b.an_ctrl = 0;
	b.an_data = 0xaa;
	b.an_complete = 0;
	b.gfx_ctrl = 0;
	b.fl_prot_count = 0;
	memset(b.gun, 0x80, sizeof(b.gun));
	memset(b.c169ctl, 0, sizeof(b.c169ctl));
	if (s_sg) {
		c355spr_reset(&s_c355);
		c355spr_reset(&s_c355r);
	}
	memset(b.control, 0, sizeof(b.control));
	memset(b.c116_regs, 0, sizeof(b.c116_regs));
	memset(b.rozctrl, 0, sizeof(b.rozctrl));
	b.rng = 0x12345678;
	b.t_snd = 0;
	b.m68k_frac = b.snd_frac = b.mcu_frac = b.ym_frac = b.c140_frac = b.irq120 = 0;
	snd_bank();
	m68k_multi_reset();
	m6809_reset(&s_snd);
	s_snd.cycles = 0;
	if (s_is_c68) {
		m740_reset(&s_c68);
		b.c68_mux = 0;
	} else
		m68705_reset(&s_mcu);
	b.snd_held = 1;          // (machine_reset: the sound CPU, the slave and the C65 held)
	b.slave_held = 1;
	b.mcu_held = 1;
	b.slave_needreset = 0;
	if (s_is_c68)
		m740_set_reset(&s_c68, 1);
	else
		m68705_set_reset_line(&s_mcu, 1);
	rbx_ym_reset(0);
	c140_reset(&s_c140);
	s_pal_dirty = 1;
}

static const struct { const char *init; int type; } s_types[] = {
	{ "ordyne", GT_ORDYNE }, { "mirninja", GT_MIRNINJA }, { "phelios", GT_PHELIOS }, { "dirtfoxj", GT_DIRTFOXJ },
	{ "finehour", GT_FINEHOUR }, { "burnforc", GT_BURNFORC }, { "marvland", GT_MARVLAND }, { "dsaber", GT_DSABER },
	{ "dsaberj", GT_DSABER }, { "rthun2", GT_RTHUN2 }, { "rthun2j", GT_RTHUN2 }, { "cosmogng", GT_COSMOGNG },
	{ "gollygho", GT_GOLLYGHO }, { "bubbletr", GT_BUBBLETR }, { "sws", GT_SWS }, { "sws92", GT_SWS92 },
	{ "sws92g", GT_SWS92G }, { "sws93", GT_SWS93 }, { "sgunner2", GT_SGUNNER2 }, { "suzuk8h2", GT_SUZUK8H2 },
};

static void ns2_exit(void);

static int ns2_init(const rbx_game *g)
{
	if (!s_bp && !(s_bp = (struct namcos2_state *)calloc(1, sizeof(*s_bp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_rom[0] = rbx_region("maincpu", &s_rom_size[0]);
	s_rom[1] = rbx_region("slave", &s_rom_size[1]);
	s_data = rbx_region("data_rom", &s_data_size);
	s_sndrom = rbx_region("audiocpu", &s_sndrom_size);
	s_mcuint = rbx_region("c65mcu:mcu", &s_mcuint_size);
	s_c68rom = rbx_region("c68mcu:mcu", &s_c68rom_size);
	s_is_c68 = s_c68rom != NULL;
	s_mcuext = rbx_region("c65mcu:external", &s_mcuext_size);
	s_chr = rbx_region("c123tmap", &s_chr_size);
	s_mask = rbx_region("c123tmap:mask", &s_mask_size);
	s_roz = rbx_region("s2roz", &s_roz_size);
	{
		uint32_t gs = 0, ms = 0;
		memset(&s_c169, 0, sizeof(s_c169));
		s_c169.gfx = rbx_region("c169roz", &gs);
		s_c169.mask = rbx_region("c169roz:mask", &ms);
		s_c169.count = s_c169.gfx ? gs / 256 : 0;
		s_c169.mask_count = s_c169.mask ? ms / 32 : 0;
		s_c169.ram_words = 0x8000;
		s_c169.color_base = 0;
		s_c169.cb = !strncmp(g->machine, "luckywld:", 9) ? lw_roz_cb : mh_roz_cb;
	}
	s_sprrom = rbx_region("sprite", &s_sprrom_size);
	{
		uint32_t csz = 0;
		const uint8_t *c355gfx = rbx_region("c355spr", &csz);
		if (c355gfx && (!strncmp(g->machine, "sgunner", 7) || !strncmp(g->machine, "suzuka8h:", 9) ||
		                !strncmp(g->machine, "luckywld:", 9))) {
			if (c355spr_init(&s_c355, 1, 1, c355gfx, csz) || c355spr_init(&s_c355r, VIS_W, VIS_H, c355gfx, csz)) {
				rbx_fail("%s: out of memory", g->set);
				return -1;
			}
			s_c355r.scrolloffs[0] = s_c355.scrolloffs[0] = 0x26;
			s_c355r.scrolloffs[1] = s_c355.scrolloffs[1] = 0x19;
			s_c355r.mix_cb = sg_mix;
		}
		if (c355gfx && !s_sprrom) {   // (the C355 boards have no System 2 sprite ROMs)
			static uint8_t none[1024];
			s_sprrom = none;
			s_sprrom_size = sizeof(none);
		}
	}
	s_c140rom = rbx_region("c140", &s_c140_size);
	if (!s_rom[0] || !s_rom[1] || !s_sndrom || s_sndrom_size < 0x4000 || (!s_is_c68 && (!s_mcuint || s_mcuint_size < 0x2000)) ||
	    (s_is_c68 && s_c68rom_size < 0x8000) || !s_chr ||
	    !s_mask || !s_sprrom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom[0], s_rom_size[0]);
	m68k_bus_swap16(s_rom[1], s_rom_size[1]);
	if (s_data)
		m68k_bus_swap16(s_data, s_data_size);
	s_chr_count = s_chr_size / 64;
	s_mask_count = s_mask_size / 8;
	s_roz_count = s_roz ? s_roz_size / 64 : 0;
	s_spr_count = s_sprrom_size / 1024;
	s_sprpix = (uint8_t *)malloc((size_t)s_spr_count * 1024 + 1);
	s_snap = (snap_t *)calloc(1, sizeof(snap_t));
	s_pri = (uint8_t *)calloc(1, VIS_W * VIS_H);
	s_sprbuf = (uint16_t *)calloc(VIS_W * VIS_H, sizeof(uint16_t));
	if (!s_sprpix || !s_snap || !s_pri || !s_sprbuf) {
		ns2_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_roz) {   // (no ROZ ROMs: a blank layer)
		static uint8_t blank[64];
		memset(blank, 0xff, sizeof(blank));
		s_roz = blank;
		s_roz_count = 1;
	}
	if (!strncmp(g->machine, "metlhawk:", 9)) {   // init_metlhawk: the sprites unscrambled; raw 32x32 8bpp
		uint8_t *data = s_sprrom;
		const uint32_t size = s_sprrom_size;
		for (uint32_t i = 0; i + 32 * 32 <= size; i += 32 * 32)
			for (uint32_t j = 0; j < 32 * 32; j += 32 * 4)
				for (uint32_t k = 0; k < 32; k += 4) {
					uint8_t v;
					uint32_t a = i + j + k + 32;
					v = data[a];
					data[a] = data[a + 3];
					data[a + 3] = data[a + 2];
					data[a + 2] = data[a + 1];
					data[a + 1] = v;
					a += 32;
					v = data[a];
					data[a] = data[a + 2];
					data[a + 2] = v;
					v = data[a + 1];
					data[a + 1] = data[a + 3];
					data[a + 3] = v;
					a += 32;
					data[a] = data[a + 1];
					data[a + 1] = data[a + 2];
					data[a + 2] = data[a + 3];
					data[a + 3] = v;
					a = i + j + k;
					for (int l = 0; l < 4; l++) {
						v = data[a + (uint32_t)l + 32];
						data[a + (uint32_t)l + 32] = data[a + (uint32_t)l + 32 * 3];
						data[a + (uint32_t)l + 32 * 3] = v;
					}
				}
		memcpy(s_sprpix, s_sprrom, (size_t)s_spr_count * 1024);
	} else
	// obj_layout: 8 planes 4 bits apart, 4 pixels to a 32-bit group, rows of 32 bytes
	// (byte j of a group: its high nibble plane 2j, its low nibble plane 2j+1; bit 3-k of a nibble pixel k)
	for (uint32_t g = 0; g < s_spr_count * 256; g++) {
		const uint8_t *base = s_sprrom + g * 4;
		uint8_t *out = s_sprpix + g * 4;
		for (int k = 0; k < 4; k++) {
			uint8_t pen = 0;
			for (int j = 0; j < 4; j++) {
				pen |= (uint8_t)(((base[j] >> (7 - k)) & 1) << (7 - 2 * j));
				pen |= (uint8_t)(((base[j] >> (3 - k)) & 1) << (6 - 2 * j));
			}
			out[k] = pen;
		}
	}
	s_fl = !strncmp(g->machine, "finallap:", 9) || !strncmp(g->machine, "finalap2:", 9) ||
	       !strncmp(g->machine, "finalap3:", 9) || !strncmp(g->machine, "base_fl:", 8);
	s_fl_spr = !strncmp(g->machine, "finallap:", 9);
	s_mh = !strncmp(g->machine, "metlhawk:", 9);
	s_sg = (!strncmp(g->machine, "sgunner:", 8) || !strncmp(g->machine, "sgunner2:", 9)) ? 1
	     : !strncmp(g->machine, "suzuka8h:", 9) ? 2 : !strncmp(g->machine, "luckywld:", 9) ? 3 : 0;
	s_fl_tile = !strncmp(g->machine, "finalap2:", 9) || !strncmp(g->machine, "finalap3:", 9);
	{
		uint32_t csz = 0;
		s_clut = rbx_region("c45_road:clut", &csz);
		if (csz < 0x100)
			s_clut = NULL;
	}
	const char *init = strchr(g->machine, ':');
	init = init ? init + 1 : "";
	s_gametype = GT_NONE;
	for (size_t i = 0; i < sizeof(s_types) / sizeof(s_types[0]); i++)
		if (!strcmp(init, s_types[i].init))
			s_gametype = s_types[i].type;
	// the mixer: base 0.75 / 0.80, base2 / assaultp 1.0 / 0.80, base3 0.45 / 1.0
	if (!strncmp(g->machine, "base3:", 6)) {
		s_c140_gain = 115;
		s_ym_gain = 256;
	} else if (!strncmp(g->machine, "base2:", 6) || !strncmp(g->machine, "assaultp:", 9)) {
		s_c140_gain = 256;
		s_ym_gain = 205;
	} else {
		s_c140_gain = 192;
		s_ym_gain = 205;
	}
	memset(s_cov, 0xff, sizeof(s_cov));
	for (int p = 0; g->ports[p].tag && p < 64; p++) {
		uint32_t m = 0;
		for (int f = 0; f < g->ports[p].count; f++)
			m |= g->fields[g->ports[p].first + f].mask;
		s_cov[p] = (uint8_t)m;
	}
	s_port_mcub = rbx_port_find("MCUB");
	s_port_mcuc = rbx_port_find("MCUC");
	s_port_mcuh = rbx_port_find("MCUH");
	s_port_dsw = rbx_port_find("DSW");
	static const char *const di[4] = { "MCUDI0", "MCUDI1", "MCUDI2", "MCUDI3" };
	static const char *const an[8] = { "AN0", "AN1", "AN2", "AN3", "AN4", "AN5", "AN6", "AN7" };
	for (int i = 0; i < 4; i++) s_port_di[i] = rbx_port_find(di[i]);
	for (int i = 0; i < 8; i++) {
		s_port_an[i] = rbx_port_find(an[i]);
		s_an_gun[i] = 0;
		if (s_port_an[i] < 0)
			continue;
		const rbx_ipt_port *pp = &g->ports[s_port_an[i]];
		for (int f = 0; f < pp->count; f++) {
			const rbx_ipt_field *fd = &g->fields[pp->first + f];
			if (fd->type == RBX_IPT_LIGHTGUN_X || fd->type == RBX_IPT_LIGHTGUN_Y) {
				s_an_gun[i] = fd->type;
				s_an_player[i] = fd->player ? fd->player : 1;
				s_an_rev[i] = !strncmp(g->machine, "sgunner:", 8);   // (the Steel Gunner 1 ports: PORT_REVERSE)
			}
		}
	}

	memset(&b, 0, sizeof(b));
	memset(b.nvram, 0xff, sizeof(b.nvram));   // (NVRAM DEFAULT_ALL_1)
	{
		uint32_t nsz = 0;
		const uint8_t *nv = rbx_region("nvram", &nsz);
		if (nv)
			memcpy(b.nvram, nv, nsz < sizeof(b.nvram) ? nsz : sizeof(b.nvram));
	}
	// the 68000s
	s_noidle = getenv("NS2_NOIDLE") != NULL;
	if (getenv("NS2_PCPROF") && !s_pcprof)
		s_pcprof = (uint32_t *)calloc(2 * 0x20000 + 0x20000, sizeof(uint32_t));
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	if (m68k_multi_init(2)) {
		ns2_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	for (int c = 0; c < 2; c++) {
		m68k_bus_reset_map();
		m68k_bus_map(0x000000, 0x03ffff, s_rom[c], s_rom_size[c], 1, 0);
		if (c == 0)
			m68k_bus_map(0x100000, 0x10ffff, b.ram0, sizeof(b.ram0), 1, 1);
		else
			m68k_bus_map(0x100000, 0x13ffff, b.ram1, sizeof(b.ram1), 1, 1);
		if (s_data && s_fl) {   // (Final Lap: 0x300000-0x33ffff its protection)
			m68k_bus_map(0x200000, 0x2fffff, s_data, s_data_size, 1, 0);
			if (s_data_size > 0x140000)
				m68k_bus_map(0x340000, 0x3fffff, s_data + 0x140000, s_data_size - 0x140000, 1, 0);
		} else if (s_data)
			m68k_bus_map(0x200000, 0x3fffff, s_data, s_data_size, 1, 0);
		m68k_bus_map(0x400000, 0x41ffff, b.vram, sizeof(b.vram), 1, 1);
		m68k_bus_map(0x480000, 0x483fff, b.sci, sizeof(b.sci), 1, 1);
		if (s_mh) {
			m68k_bus_map(0xc00000, 0xc03fff, b.spr, 0x4000, 1, 1);
			m68k_bus_map(0xc40000, 0xc4ffff, b.rozram, 0x10000, 1, 1);   // (the C169's RAM)
		} else if (s_sg >= 2) {
			m68k_bus_map(0xa00000, 0xa1ffff, b.rozram, sizeof(b.rozram), 1, 1);   // (the C45 road)
			if (s_sg == 3)
				m68k_bus_map(0xc00000, 0xc0ffff, b.spr, 0x10000, 1, 1);   // (the C169's RAM)
		}
		else if (s_sg) {
		} else if (s_fl) {
			m68k_bus_map(0x800000, 0x80ffff, b.spr, sizeof(b.spr), 1, 1);
			m68k_bus_map(0x880000, 0x89ffff, b.rozram, sizeof(b.rozram), 1, 1);
		} else {
			m68k_bus_map(0xc00000, 0xc03fff, b.spr, 0x4000, 1, 1);
			m68k_bus_map(0xc80000, 0xc9ffff, b.rozram, sizeof(b.rozram), 1, 1);
		}
		m68k_bus.read8 = bus_r8;
		m68k_bus.read16 = bus_r16;
		m68k_bus.write8 = bus_w8;
		m68k_bus.write16 = bus_w16;
		m68k_multi_capture(c);
	}
	m68k_multi_finish();
	// the sound CPU
	m6809_init(&s_snd);
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	if (!getenv("NS2_NOIDLE"))
		s_snd.idle_hook = snd_idle_hook;
	for (int p = 0x70; p < 0x80; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = b.dpram + (p & 7) * 256;
	for (int p = 0x80; p < 0xa0; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = b.sndram + (p - 0x80) * 256;
	for (int p = 0xc0; p < 0x100; p++)
		s_snd.rpage[p] = s_sndrom + (p - 0xc0) * 256;
	for (int p = 0xc0; p < 0x100; p++)
		s_snd.wpage[p] = NULL;
	// the I/O MCU: the C65 or the C68
	if (s_is_c68) {
		m740_init(&s_c68);
		for (int p = 0x80; p < 0x100; p++)
			s_c68.rpage[p] = s_c68rom + (p - 0x80) * 256;
		s_c68.read = c68_r;
		s_c68.write = c68_w;
		s_c68.port_read = c68_port_r;
		s_c68.port_write = c68_port_w;
		s_c68.ad_read = c68_ad_r;
	} else {
		if (m68705_init(&s_mcu, M68705_HD63705Z0, s_mcuint, s_mcuint_size)) {
			ns2_exit();
			rbx_fail("%s: MCU init failed", g->set);
			return -1;
		}
		s_mcu.ext_r = mcu_r;
		s_mcu.ext_w = mcu_w;
	}
	// sound
	c140_init(&s_c140, C140_CLOCK, s_c140rom, s_c140rom ? s_c140_size : 0, s_c140_gain, c140_irq, NULL);
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = s_ym_gain;
	yc.irq = ym_irq;
	int rc = rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 2);
	s_ym_rate = rc ? 0 : rbx_ym_rate(0);
	s_mix_cap = (int)(s_ym_rate / 50) + 256;
	s_mix_l = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_mix_r = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	rc = rc || !s_mix_l || !s_mix_r || rbx_audio_setup_stereo(s_ym_rate) || rbx_video_setup(VIS_W, VIS_H, 0x2001);
	if (rc) {
		ns2_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_pal_valid = 0;
	ns2_reset();
	return 0;
}

static void ns2_exit(void)
{
	if (s_pcprof) {
		for (int c = 0; c < 2; c++) {
			uint64_t tot = 0;
			for (int i = 0; i < 0x20000; i++) tot += s_pcprof[c * 0x20000 + i];
			for (int k = 0; k < 8; k++) {
				int best = 0;
				for (int i = 1; i < 0x20000; i++)
					if (s_pcprof[c * 0x20000 + i] > s_pcprof[c * 0x20000 + best]) best = i;
				char hx[64]; int n = 0;
				for (int j = 0; j < 6; j++) n += snprintf(hx + n, sizeof(hx) - n, "%04x ", rd16(s_rom[c], (uint32_t)(best * 2 + j * 2) & 0x3ffff));
				rbx_log("PC cpu%d %06x %.1f%% %s", c, best * 2, tot ? 100.0 * s_pcprof[c * 0x20000 + best] / (double)tot : 0.0, hx);
				s_pcprof[c * 0x20000 + best] = 0;
			}
		}
		for (int k = 0; k < 6; k++) {
			int best = 0;
			for (int i = 1; i < 0x10000; i++)
				if (s_pcprof[0x40000 + i] > s_pcprof[0x40000 + best]) best = i;
			uint8_t by[6];
			for (int j = 0; j < 6; j++) {
				const uint16_t ad = (uint16_t)(best + j);
				by[j] = s_snd.rpage[ad >> 8] ? s_snd.rpage[ad >> 8][ad & 0xff] : 0;
			}
			rbx_log("PC snd %04x %u %02x %02x %02x %02x %02x %02x dp %02x", best, s_pcprof[0x40000 + best], by[0], by[1], by[2], by[3], by[4], by[5], s_snd.dp);
			s_pcprof[0x40000 + best] = 0;
		}
		for (int k = 0; k < 6 && s_is_c68; k++) {
			int best = 0;
			for (int i = 1; i < 0x10000; i++)
				if (s_pcprof[0x50000 + i] > s_pcprof[0x50000 + best]) best = i;
			uint8_t by[6];
			for (int j = 0; j < 6; j++) {
				const uint16_t ad = (uint16_t)(best + j);
				by[j] = s_c68.rpage[ad >> 8] ? s_c68.rpage[ad >> 8][ad & 0xff] : 0;
			}
			rbx_log("PC c68 %04x %u %02x %02x %02x %02x %02x %02x", best, s_pcprof[0x50000 + best], by[0], by[1], by[2], by[3], by[4], by[5]);
			s_pcprof[0x50000 + best] = 0;
		}
		free(s_pcprof);
		s_pcprof = NULL;
	}
	rbx_ym_exit(0);
	m68k_multi_exit();
	c355spr_free(&s_c355);
	c355spr_free(&s_c355r);
	free(s_sprpix);
	free(s_snap);
	free(s_pri);
	free(s_sprbuf);
	free(s_mix_l);
	free(s_mix_r);
	s_sprpix = NULL;
	s_snap = NULL;
	s_pri = NULL;
	s_sprbuf = NULL;
	s_mix_l = s_mix_r = NULL;
	free(s_bp);   // (last: the render thread and the saves are done with it)
	s_bp = NULL;
}

static int ns2_state(void *ctx, int load)
{
	int rc = m68k_multi_state(ctx, load);
	uint32_t asserted[2] = { m68k_multi_asserted(0), m68k_multi_asserted(1) };
	rc |= rbx_state_io(ctx, "asserted", asserted, sizeof(asserted));
	rc |= rbx_state_io(ctx, "board", &b, sizeof(b));
	rc |= rbx_state_io(ctx, "snd", &s_snd, M6809_STATE_SIZE);
	if (s_is_c68)
		rc |= rbx_state_io(ctx, "c68", &s_c68, M740_STATE_SIZE);
	else {
		rc |= rbx_state_io(ctx, "mcu", &s_mcu.st, sizeof(s_mcu.st));
		rc |= rbx_state_io(ctx, "mcuz", &s_mcu.zst, sizeof(s_mcu.zst));
	}
	rc |= rbx_state_io(ctx, "c140", &s_c140, (uint32_t)C140_STATE_SIZE);
	if (s_sg)
		rc |= rbx_state_io(ctx, "c355", C355SPR_SAVE_PTR(&s_c355), (uint32_t)C355SPR_SAVE_SIZE);
	rc |= rbx_ym_state(0, ctx, load);
	if (load && !rc) {
		m68k_multi_set_asserted(0, asserted[0]);
		m68k_multi_set_asserted(1, asserted[1]);
		snd_bank();
		s_pal_dirty = 1;
		m68k_multi_switch(0);
	}
	return rc;
}

static int ns2_save(void *ctx) { return ns2_state(ctx, 0); }
static int ns2_load(void *ctx) { return ns2_state(ctx, 1); }
static int ns2_supported(const char *machine)
{
	return !strncmp(machine, "base:", 5) || !strncmp(machine, "base2:", 6) || !strncmp(machine, "base3:", 6) ||
	       !strncmp(machine, "assaultp:", 9) || !strncmp(machine, "base_c68:", 9) || !strncmp(machine, "finallap:", 9) ||
	       !strncmp(machine, "finalap2:", 9) || !strncmp(machine, "finalap3:", 9) || !strncmp(machine, "base_fl:", 8) ||
	       !strncmp(machine, "sgunner:", 8) || !strncmp(machine, "sgunner2:", 9) || !strncmp(machine, "suzuka8h:", 9) ||
	       !strncmp(machine, "metlhawk:", 9) || !strncmp(machine, "luckywld:", 9);
}

const rbx_driver rbx_drv_namcos2 = { "Namco System 2", ns2_init, ns2_exit, ns2_reset, ns2_frame, ns2_save, ns2_load,
                                     ns2_supported, ns2_render, ns2_render_help };
