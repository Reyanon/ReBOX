/*  Namco NB-1 / NB-2: an MC68EC020, the C75 MCU (an M37702 with Namco's BIOS: the sound driver and the inputs) on
    the shared RAM, the C352 PCM chip, the C116 palette / window, the C123 tilemaps, the C355 zooming sprites (triple
    buffered), the NB-2's C169 ROZ layers and tile / sprite / ROZ banking, the key custom chips, the parallel EEPROM,
    Point Blank / Gun Bullet's light guns.
    Ported from MAME 0.289 src/mame/namco/namconb1.cpp (BSD-3-Clause, copyright-holders: Phil Stroffolino),
    src/mame/namco/namco_c116.cpp, namco_c123tmap.cpp (BSD-3-Clause, copyright-holders: David Haywood, Phil
    Stroffolino), src/mame/namco/namcomcu.cpp (BSD-3-Clause, copyright-holders: Alex W. Jackson),
    src/devices/machine/eeprompar.cpp and the random words of src/emu/machine.cpp (BSD-3-Clause, copyright-holders:
    Aaron Giles); the sprites, the ROZ layers and the C352 in their own files.
    This file is part of ReBOX. GPL-2.0-or-later.

    As MAME: the 68020 and the MCU in scanline slices (the MCU caught up to the 68020's time at each of its interrupt
    line changes and at the MCU control, MAME's set_input_line sync), the MCU's two 60 Hz interrupts at their own
    times, the C352 rendered up to the MCU's time at each access, the MCU's 300-cycle spin after the sync flag.
    Not done: Gun Bullet's force feedback, the hardware random generator's seed (MAME: a NOP).
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/m37710/m37710.h"
#include "../../sound/c352.h"
#include "../../video/decoband.h"
#include "../video/c355spr.h"
#include "../video/c169roz.h"

#include <stdlib.h>
#include <string.h>

#define VIS_W 288
#define VIS_H 224
#define LINES 264
#define FRAME_MHZ 59659           // 6.048 MHz / (384 x 264)
#define M68K_CLOCK 24192000       // 48.384 MHz / 2
#define LINE68 1536               // (the 68020's clocks a line)
#define MCU_CLOCK 16128000        // 48.384 MHz / 3 (the CPU: half, 512 cycles a line)
#define MCU_DIV 3                 // (68020 clocks a MCU cycle)
#define MCUIRQ_PERIOD 403200      // (the MCU's 60 Hz timers in 68020 clocks)
#define SND_FRAME 1408            // (84 kHz: 24.192 MHz / 288, 1408 a frame, one per 96 MCU cycles)
#define SND_RATE 84000
#define PAL_BLACK 0x4000          // (the C116's black pen after its shadows; | 0x800 by a sprite shadow: black too)
#define NPENS 0x4801

enum {
	GT_NONE, GT_NEBULRAY, GT_GUNBULET, GT_GSLGR94U, GT_GSLGR94J, GT_SWS95, GT_SWS96, GT_SWS97, GT_VSHOOT, GT_OUTFOXIES,
	GT_MACHBRKR
};
enum { MODE_NB1, MODE_GUN, MODE_NB2_OUTFXIES, MODE_NB2_MACHBRKR };

// ---- the set's configuration ----
static uint8_t *s_prg, *s_data, *s_c75data, *s_mcurom, *s_chr, *s_mask, *s_c352rom;
static uint32_t s_prg_size, s_data_size, s_c75data_size, s_mcurom_size, s_chr_size, s_mask_size, s_c352_size;
static uint32_t s_chr_count, s_mask_count;
static int s_gametype, s_mode, s_nb2, s_nebulray;
static int s_port_p[4], s_port_misc, s_port_gun[4];   // (LIGHT0_X, LIGHT0_Y, LIGHT1_X, LIGHT1_Y)

static m37710_t s_mcu;
static c352_t s_c352;
static c355spr_t s_c355, s_c355v;   // (the live one; a view of the drawing's copy of its bitmap)
static c169roz_t s_c169;

// ---- the board's state (saved: up to state_end; on the heap while the game runs, ADR-040) ----
static struct nb_state {
	uint8_t ram1c[0x10000];      // 0x1c0000
	uint8_t share[0x8000];       // 0x200000 (the MCU's 0x4000: host-order words, the MCU's byte n = byte n)
	uint8_t ram208[0xf8000];     // 0x208000
	uint8_t c123vram[0x10000];
	uint16_t c123ctl[0x20];
	uint8_t c169ram[0x20000];    // (NB-2)
	uint16_t c169ctl[16];
	uint16_t spritebank[8], spritebank_delayed[8];   // (host-order words: byte n = the 68020's)
	uint16_t tilebank[8], rozbank[8], unk640[8];
	uint8_t pal_r[0x2000], pal_g[0x2000], pal_b[0x2000];
	uint16_t c116_regs[8];
	uint8_t eeprom[0x800];
	uint8_t mcuram[0x2000];      // (nothing: the MCU's map has none past its internal RAM)
	uint8_t vbl_irq_level, pos_irq_level, unk_irq_level, port6;
	uint8_t mcu_halt, pad[3];
	uint16_t count;
	uint32_t rng;
	int32_t gun[4];
	int64_t t68, t68_target;     // the 68020's clocks since the start / the end of the current slice
	int64_t mcuirq_next;         // (the MCU's 60 Hz timers: their next time in 68020 clocks)
	int64_t mcu_frame0;
	int64_t frames;
	int32_t snd_done;
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)

static int64_t s_mcu_t0;         // (the MCU's cycles at the 68020's clock 0)
static int s_sync;               // (a sync point in the 68020's slice: the MCU to its time first)
static uint32_t s_key32;         // (read32: the key chip read as one access)

// ---- the drawing's copies ----
// one a screen update (the position IRQ's partial updates: up to one a line, Nebulas Ray's stars), their RAMs
// copied only when written since the copy before (a pool of copies a frame)
typedef struct {
	uint16_t c123ctl[0x20];
	uint16_t c116_regs[8];
	uint16_t c169ctl[16];
	uint16_t tilebank[8], rozbank[8];
	const uint8_t *c123vram, *c169ram;
	int y0, y1;
} snap_t;
#define NSNAP (VIS_H + 1)
#define VPOOL 8
#define RPOOL 4
static snap_t (*s_snap_hp)[NSNAP];
#define s_snap (*s_snap_hp)
static int s_nsnap, s_frame_nsnap, s_last_line;
static uint8_t *s_vpool[VPOOL], *s_rpool[RPOOL];
static int s_nvp, s_nrp;          // (the copies made this frame)
static int s_vram_w, s_roz_w;     // (the RAMs written since their last copy)
static uint16_t *s_sprbm;        // (the sprites' bitmap of the frame drawn: the C355's, rendered at the vblank before)
static uint8_t *s_pri;
static uint8_t s_pal_r[0x2000], s_pal_g[0x2000], s_pal_b[0x2000];
static int s_pal_dirty, s_pal_valid;
static const snap_t *s_draw_sn;  // (the copy being drawn: the C169's tile callback reads its ROZ banks)

static int32_t s_mix_l[SND_FRAME + 64], s_mix_r[SND_FRAME + 64];

static inline uint16_t rd16(const uint8_t *mem, uint32_t a) { return *(const uint16_t *)(mem + (a & ~1u)); }
static inline void wr16(uint8_t *mem, uint32_t a, uint16_t v, uint16_t mask)
{
	uint16_t *p = (uint16_t *)(mem + (a & ~1u));
	*p = (uint16_t)((*p & ~mask) | (v & mask));
}
// a big-endian byte of a register block kept as host-order words
static inline uint8_t be8(const uint16_t *w, int n) { return (n & 1) ? (uint8_t)w[(n >> 1) & 7] : (uint8_t)(w[(n >> 1) & 7] >> 8); }

// MAME's running_machine::rand
static uint32_t mrand(void)
{
	b.rng = 1664525u * b.rng + 1013904223u;
	return (b.rng >> 16) | (b.rng << 16);
}

// ---- sound: the C352 made up to the MCU's time ----
static int64_t mcu_now(void) { return s_mcu.cycles + (s_mcu.insn_icount - s_mcu.icount); }

static void snd_to(int n)
{
	if (n > SND_FRAME)
		n = SND_FRAME;
	if (n > b.snd_done) {
		c352_render(&s_c352, s_mix_l + b.snd_done, s_mix_r + b.snd_done, n - b.snd_done);
		b.snd_done = n;
	}
}

static void snd_catchup(void)
{
	const int64_t e = mcu_now() - b.mcu_frame0;
	snd_to(e <= 0 ? 0 : (int)(e / 96));
}

// ---- the key chip ----
static uint32_t custom_key_r(int offset)
{
	const uint16_t old_count = b.count;
	do   // (a random number, not the same twice in a row)
		b.count = (uint16_t)mrand();
	while (b.count == old_count);
	const uint32_t c = b.count;
	switch (s_gametype) {
	case GT_GUNBULET: return 0;
	case GT_SWS95: if (offset == 0) return 0x0189; if (offset == 1) return c << 16; break;
	case GT_SWS96: if (offset == 0) return 0x01aa << 16; if (offset == 4) return c << 16; break;
	case GT_SWS97: if (offset == 2) return 0x1b2 << 16; if (offset == 5) return c << 16; break;
	case GT_GSLGR94U: if (offset == 0) return 0x0167; if (offset == 1) return c << 16; break;
	case GT_GSLGR94J: if (offset == 1) return 0; if (offset == 3) return (0x0171 << 16) | c; break;
	case GT_NEBULRAY: if (offset == 1) return 0x016e; if (offset == 3) return c; break;
	case GT_VSHOOT: if (offset == 2) return c << 16; if (offset == 3) return 0x0170 << 16; break;
	case GT_OUTFOXIES: if (offset == 0) return 0x0186; if (offset == 1) return c << 16; break;
	default: break;
	}
	return 0;
}

static uint32_t gun_r(int offset)
{
	int result = 0;
	const int x1 = b.gun[0], y1 = b.gun[1], x2 = b.gun[2], y2 = b.gun[3];
	switch (offset) {
	case 0: case 1: result = (uint8_t)(0x0f + y2 * 224 / 255); break;   // Y (p2)
	case 2: case 3: result = (uint8_t)(0x26 + x2 * 288 / 314); break;   // X (p2)
	case 4: case 5: result = (uint8_t)(0x0f + y1 * 224 / 255); break;   // Y (p1)
	case 6: case 7: result = (uint8_t)(0x26 + x1 * 288 / 314); break;   // X (p1)
	default: break;
	}
	return (uint32_t)result << 24;
}

// ---- the C116 (palette, window registers) ----
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

// ---- the CPU registers ----
static void mcu_run_until(int64_t t68);

static void set_irq(int level, int state)
{
	if (level >= 1 && level <= 7)
		m68k_bus_set_line(level, state);
}

// a set_input_line from the 68020: the MCU to its time first (MAME synchronize), the slice ends there
static void sync68(void)
{
	b.t68 += m68k_cycles_run();
	mcu_run_until(b.t68);
	b.t68 -= m68k_cycles_run();
	s_sync = 1;
	m68k_end_timeslice();
}

static void mcu_control(uint8_t data)
{
	sync68();
	if (data & 1) {   // (halt cleared, reset pulsed: the MCU starts over)
		b.mcu_halt = 0;
		m37710_reset(&s_mcu);
	} else
		b.mcu_halt = 1;
}

static void cpureg_w(uint32_t offset, uint8_t data)
{
	offset &= 0x1f;
	if (!s_nb2) {
		switch (offset) {
		case 0x01: sync68(); set_irq(b.pos_irq_level, 0); b.pos_irq_level = data & 0x0f; break;
		case 0x02: sync68(); set_irq(b.unk_irq_level, 0); b.unk_irq_level = data & 0x0f; break;
		case 0x04: sync68(); set_irq(b.vbl_irq_level, 0); b.vbl_irq_level = data & 0x0f; break;
		case 0x06: sync68(); set_irq(b.pos_irq_level, 0); break;
		case 0x07: sync68(); set_irq(b.unk_irq_level, 0); break;
		case 0x09: sync68(); set_irq(b.vbl_irq_level, 0); break;
		case 0x18: mcu_control(data); break;
		default: break;
		}
	} else {
		switch (offset) {
		case 0x00: sync68(); set_irq(b.vbl_irq_level, 0); b.vbl_irq_level = data & 0x0f; break;
		case 0x01: sync68(); set_irq(b.unk_irq_level, 0); b.unk_irq_level = data & 0x0f; break;
		case 0x02: sync68(); set_irq(b.pos_irq_level, 0); b.pos_irq_level = data & 0x0f; break;
		case 0x04: sync68(); set_irq(b.vbl_irq_level, 0); break;
		case 0x05: sync68(); set_irq(b.unk_irq_level, 0); break;
		case 0x06: sync68(); set_irq(b.pos_irq_level, 0); break;
		case 0x16: mcu_control(data); break;
		default: break;
		}
	}
}

// ---- the 68020's bus ----
static uint16_t c123ctl_r(uint32_t a) { return b.c123ctl[(a >> 1) & 0x1f]; }

static uint16_t bus_r16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x600000 && a < 0x620000)
		return c355spr_spriteram_r(&s_c355, (a - 0x600000) >> 1);
	if (a >= 0x620000 && a < 0x620008)
		return c355spr_position_r(&s_c355, (a - 0x620000) >> 1);
	if (a >= 0x1e4000 && a < 0x1e4004)
		return (a & 2) ? (uint16_t)mrand() : (uint16_t)(mrand() >> 16);   // (a word of a fresh random word)
	if (!s_nb2) {
		if (a >= 0x400000 && a < 0x400020)
			return 0xffff;   // (cpureg_r)
		if (a >= 0x580000 && a < 0x580800)
			return (uint16_t)(b.eeprom[a - 0x580000] << 8 | b.eeprom[a - 0x580000 + 1]);
		if (a >= 0x660000 && a < 0x660040)
			return c123ctl_r(a);
		if (a >= 0x680000 && a < 0x680010)
			return b.spritebank[(a >> 1) & 7];
		if (a >= 0x6e0000 && a < 0x6e0020) {
			const uint32_t v = custom_key_r((int)((a >> 2) & 7));
			return (a & 2) ? (uint16_t)v : (uint16_t)(v >> 16);
		}
		if (a >= 0x700000 && a < 0x708000)
			return (uint16_t)(c116_r(a - 0x700000) << 8 | c116_r(a - 0x700000 + 1));
		if (s_mode == MODE_GUN && a >= 0x100000 && a < 0x100020) {
			const uint32_t v = gun_r((int)((a >> 2) & 7));
			return (a & 2) ? (uint16_t)v : (uint16_t)(v >> 16);
		}
	} else {
		if (a >= 0x640000 && a < 0x640010)
			return b.unk640[(a >> 1) & 7];
		if (a >= 0x6c0000 && a < 0x6c0040)
			return c123ctl_r(a);
		if (a >= 0x740000 && a < 0x740020)
			return b.c169ctl[(a >> 1) & 15];
		if (a >= 0x800000 && a < 0x808000)
			return (uint16_t)(c116_r(a - 0x800000) << 8 | c116_r(a - 0x800000 + 1));
		if (a >= 0x900008 && a < 0x900010)
			return b.spritebank[(a >> 1) & 3];
		if (a >= 0x940000 && a < 0x940010)
			return b.tilebank[(a >> 1) & 7];
		if (a >= 0x980000 && a < 0x980010)
			return b.rozbank[(a >> 1) & 7];
		if (a >= 0xa00000 && a < 0xa00800)
			return (uint16_t)(b.eeprom[a - 0xa00000] << 8 | b.eeprom[a - 0xa00000 + 1]);
		if (a >= 0xc00000 && a < 0xc00020) {
			const uint32_t v = custom_key_r((int)((a >> 2) & 7));
			return (a & 2) ? (uint16_t)v : (uint16_t)(v >> 16);
		}
		if (a >= 0xf00000 && a < 0xf00020)
			return 0xffff;
	}
	return 0;
}

static uint8_t bus_r8(uint32_t a)
{
	a &= 0xffffff;
	if (!s_nb2 && a >= 0x700000 && a < 0x708000)
		return c116_r(a - 0x700000);
	if (s_nb2 && a >= 0x800000 && a < 0x808000)
		return c116_r(a - 0x800000);
	const uint16_t v = bus_r16(a & ~1u);
	return (uint8_t)((a & 1) ? v : v >> 8);
}

// a 32-bit access as one (the key chip, the random generator: one word a read)
static uint32_t bus_r32(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x1e4000 && a < 0x1e4004)
		return mrand();
	if ((!s_nb2 && a >= 0x6e0000 && a < 0x6e0020) || (s_nb2 && a >= 0xc00000 && a < 0xc00020))
		return custom_key_r((int)((a >> 2) & 7));
	if (s_mode == MODE_GUN && a >= 0x100000 && a < 0x100020)
		return gun_r((int)((a >> 2) & 7));
	(void)s_key32;
	return (uint32_t)bus_r16(a) << 16 | bus_r16(a + 2);
}

static void bus_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a >= 0x600000 && a < 0x620000) {
		c355spr_spriteram_w(&s_c355, (a - 0x600000) >> 1, v, mask);
		return;
	}
	if (a >= 0x620000 && a < 0x620008) {
		c355spr_position_w(&s_c355, (a - 0x620000) >> 1, v, mask);
		return;
	}
	{	// the tilemap RAM, the ROZ RAM (their written flags for the partial updates' copies)
		const uint32_t vb = s_nb2 ? 0x680000 : 0x640000;
		if (a >= vb && a < vb + 0x10000) {
			wr16(b.c123vram, a - vb, v, mask);
			s_vram_w = 1;
			return;
		}
		if (s_nb2 && a >= 0x700000 && a < 0x720000) {
			wr16(b.c169ram, a - 0x700000, v, mask);
			s_roz_w = 1;
			return;
		}
	}
	uint32_t c116 = 0, ee = 0, cpu = 0;
	if (!s_nb2) {
		if (a >= 0x400000 && a < 0x400020) cpu = 0x400000;
		else if (a >= 0x580000 && a < 0x580800) ee = 0x580000;
		else if (a >= 0x700000 && a < 0x708000) c116 = 0x700000;
		else if (a >= 0x660000 && a < 0x660040) {
			uint16_t *p = &b.c123ctl[(a >> 1) & 0x1f];
			*p = (uint16_t)((*p & ~mask) | (v & mask));
			return;
		} else if (a >= 0x680000 && a < 0x680010) {
			uint16_t *p = &b.spritebank[(a >> 1) & 7];
			*p = (uint16_t)((*p & ~mask) | (v & mask));
			return;
		}
	} else {
		if (a >= 0xf00000 && a < 0xf00020) cpu = 0xf00000;
		else if (a >= 0xa00000 && a < 0xa00800) ee = 0xa00000;
		else if (a >= 0x800000 && a < 0x808000) c116 = 0x800000;
		else {
			uint16_t *p = NULL;
			if (a >= 0x6c0000 && a < 0x6c0040) p = &b.c123ctl[(a >> 1) & 0x1f];
			else if (a >= 0x740000 && a < 0x740020) p = &b.c169ctl[(a >> 1) & 15];
			else if (a >= 0x640000 && a < 0x640010) p = &b.unk640[(a >> 1) & 7];
			else if (a >= 0x900008 && a < 0x900010) p = &b.spritebank[(a >> 1) & 3];
			else if (a >= 0x940000 && a < 0x940010) p = &b.tilebank[(a >> 1) & 7];
			else if (a >= 0x980000 && a < 0x980010) p = &b.rozbank[(a >> 1) & 7];
			if (p)
				*p = (uint16_t)((*p & ~mask) | (v & mask));
			return;
		}
	}
	if (cpu || ee || c116) {   // (8-bit devices: a byte each)
		const uint32_t base = cpu | ee | c116;
		for (int k = 0; k < 2; k++) {
			if (!(mask & (k ? 0x00ff : 0xff00)))
				continue;
			const uint8_t d = (uint8_t)(k ? v : v >> 8);
			const uint32_t off = a - base + (uint32_t)k;
			if (cpu)
				cpureg_w(off, d);
			else if (ee)
				b.eeprom[off & 0x7ff] = d;
			else
				c116_w(off, d);
		}
	}
	// (the key chip, the random seed: nothing)
}

static void bus_w16(uint32_t a, uint16_t v) { bus_w16m(a, v, 0xffff); }
static void bus_w8(uint32_t a, uint8_t v)
{
	if (a & 1)
		bus_w16m(a & ~1u, v, 0x00ff);
	else
		bus_w16m(a, (uint16_t)(v << 8), 0xff00);
}

// ---- the MCU's bus (namcoc75_am) ----
static uint16_t mcu_r16(void *ctx, uint32_t a)
{
	(void)ctx;
	if (a >= 0x2000 && a < 0x3000) {
		snd_catchup();
		return c352_read(&s_c352, ((a - 0x2000) >> 1) & 0x7ff);
	}
	return 0;
}

static uint8_t mcu_r(void *ctx, uint32_t a)
{
	if (a >= 0x2000 && a < 0x3000) {
		const uint16_t v = mcu_r16(ctx, a & ~1u);
		return (uint8_t)((a & 1) ? v >> 8 : v);
	}
	return 0;
}

// mcu_shared_w at the sync flag (word 0x3000): the BIOS's short window; MAME lets the 68020 at it (spin 300 cycles)
static void share_flag(uint32_t a, uint16_t data, uint16_t mask)
{
	(void)mask;
	if (a == 0xa000 && (data & 0x80)) {
		s_mcu.icount -= 300;
	}
}

static void mcu_w16(void *ctx, uint32_t a, uint16_t v)
{
	(void)ctx;
	if (a >= 0x2000 && a < 0x3000) {
		snd_catchup();
		c352_write(&s_c352, ((a - 0x2000) >> 1) & 0x7ff, v, 0xffff);
	} else if (a >= 0x4000 && a < 0xc000) {
		wr16(b.share, a - 0x4000, v, 0xffff);
		share_flag(a, v, 0xffff);
	}
}

static void mcu_w(void *ctx, uint32_t a, uint8_t v)
{
	(void)ctx;
	if (a >= 0x2000 && a < 0x3000) {
		snd_catchup();
		c352_write(&s_c352, ((a - 0x2000) >> 1) & 0x7ff, (a & 1) ? (uint16_t)(v << 8) : v, (a & 1) ? 0xff00 : 0x00ff);
	} else if (a >= 0x4000 && a < 0xc000) {
		b.share[a - 0x4000] = v;
		share_flag(a & ~1u, (a & 1) ? (uint16_t)(v << 8) : v, (a & 1) ? 0xff00 : 0x00ff);
	}
}

static uint8_t mcu_port_r(void *ctx, int port)
{
	(void)ctx;
	if (port == 6)
		return b.port6;
	if (port == 7) {
		int p = -1;
		switch (b.port6 & 0xf0) {
		case 0x00: p = s_port_p[3]; break;
		case 0x20: p = s_port_misc; break;
		case 0x40: p = s_port_p[0]; break;
		case 0x60: p = s_port_p[1]; break;
		default: return 0xff;
		}
		return p >= 0 ? (uint8_t)rbx_port_read(p) : 0xff;
	}
	return 0xff;
}

static void mcu_port_w(void *ctx, int port, uint8_t data, uint8_t mask)
{
	(void)ctx;
	(void)mask;
	if (port == 6)
		b.port6 = data;
}

// player 3's bits on the analog inputs (dac_bit_r)
static uint16_t mcu_an_r(void *ctx, int ch)
{
	(void)ctx;
	static const uint8_t bit[8] = { 6, 5, 4, 0, 1, 2, 3, 7 };
	const uint32_t v = s_port_p[2] >= 0 ? rbx_port_read(s_port_p[2]) : 0xff;
	return (uint16_t)((v << (7 - bit[ch & 7])) & 0x80);
}


// ---- the screen updates: copies for the drawing ----
static void capture(int y1)
{
	if (y1 > VIS_H - 1)
		y1 = VIS_H - 1;
	if (s_frame_nsnap >= NSNAP || y1 < s_last_line)
		return;
	if (s_frame_nsnap == 0) {
		rbx_video_wait_idle();
		memcpy(s_sprbm, s_c355.bitmap, (size_t)VIS_W * VIS_H * sizeof(uint16_t));
		s_nvp = s_nrp = 0;
	}
	snap_t *sn = &s_snap[s_frame_nsnap];
	sn->y0 = s_last_line;
	sn->y1 = y1;
	s_last_line = y1 + 1;
	memcpy(sn->c123ctl, b.c123ctl, sizeof(sn->c123ctl));
	memcpy(sn->c116_regs, b.c116_regs, sizeof(sn->c116_regs));
	if (s_vram_w || !s_nvp) {   // (a new copy; past the pool's end the last one taken again)
		if (s_nvp < VPOOL)
			s_nvp++;
		memcpy(s_vpool[s_nvp - 1], b.c123vram, sizeof(b.c123vram));
		s_vram_w = 0;
	}
	sn->c123vram = s_vpool[s_nvp - 1];
	if (s_nb2) {
		memcpy(sn->c169ctl, b.c169ctl, sizeof(sn->c169ctl));
		memcpy(sn->tilebank, b.tilebank, sizeof(sn->tilebank));
		memcpy(sn->rozbank, b.rozbank, sizeof(sn->rozbank));
		if (s_roz_w || !s_nrp) {
			if (s_nrp < RPOOL)
				s_nrp++;
			memcpy(s_rpool[s_nrp - 1], b.c169ram, sizeof(b.c169ram));
			s_roz_w = 0;
		}
		sn->c169ram = s_rpool[s_nrp - 1];
	}
	s_frame_nsnap++;
}

static void frame_capture(void)
{
	capture(VIS_H - 1);
	if (!s_frame_nsnap)
		rbx_video_wait_idle();
	if (s_pal_dirty) {
		memcpy(s_pal_r, b.pal_r, sizeof(s_pal_r));
		memcpy(s_pal_g, b.pal_g, sizeof(s_pal_g));
		memcpy(s_pal_b, b.pal_b, sizeof(s_pal_b));
		s_pal_dirty = 0;
		s_pal_valid = 0;
	}
	s_nsnap = s_frame_nsnap;
	s_frame_nsnap = 0;
	s_last_line = 0;
	rbx_video_frame_done();
}

// ---- the sprites' tile banks (objcode2tile) ----
static int objcode2tile(void *user, int code)
{
	(void)user;
	switch (s_mode) {
	case MODE_NB2_MACHBRKR: case MODE_NB2_OUTFXIES: {
		const int n = (code >> 11) & 0xf;
		const int bank = n < 8 ? be8(b.spritebank_delayed, n) : 0;   // (MAME: past its 8 bytes)
		int bits = (bank & 0x1f) | ((bank & 0x40) >> 1);   // (bitswap<6>(bank & 0x5f, 6, 4, 3, 2, 1, 0))
		if (s_mode == MODE_NB2_OUTFXIES)   // (bitswap<6>(bank, 6, 4, 3, 1, 2, 0): bits 1 and 2 crossed)
			bits = (bits & ~6) | ((bits & 2) << 1) | ((bits & 4) >> 1);
		return (code & 0x7ff) | (bits << 11);
	}
	default: {
		const int bank = b.spritebank_delayed[(code >> 11) & 7];
		return (code & 0x7ff) | (bank << 11);
	}
	}
}

// sprite_mix_callback
static int sprite_mix(void *user, uint16_t *dest, uint8_t *destpri, uint16_t colbase, uint16_t src, int srcpri, int pri)
{
	(void)user;
	(void)pri;
	if (srcpri >= *destpri && (src & 0xff) != 0xff) {
		if (src == 0xffe)
			*dest |= 0x800;
		else
			*dest = (uint16_t)(colbase + src);
		*destpri = (uint8_t)srcpri;
		return 1;
	}
	return 0;
}

static void screen_vblank(void)
{
	static const c355spr_rect vis = { 0, VIS_W - 1, 0, VIS_H - 1 };
	c355spr_vblank(&s_c355, 1, &vis);
	if (s_nb2)
		memcpy(b.spritebank_delayed, b.spritebank, 8);   // (NB-2: 8 bytes)
	else
		memcpy(b.spritebank_delayed, b.spritebank, sizeof(b.spritebank));
}

// ---- the drawing ----
// the tilemaps' tile callbacks: the pixel tile and the mask tile of a code
static inline void c123_tile(const snap_t *sn, uint16_t code, uint32_t *tile, uint32_t *mask)
{
	switch (s_mode) {
	case MODE_NB2_MACHBRKR: {
		const uint32_t bank = be8(sn->tilebank, (code >> 13) + 8);
		*tile = *mask = (code & 0x1fffu) | (bank << 13);
		break;
	}
	case MODE_NB2_OUTFXIES:   // (the pixmap index mangled: bits 6 and 8 crossed; the mask's not)
		*tile = (code & ~0x1c0u) | ((code & 0x40u) << 2) | (code & 0x80u) | ((code & 0x100u) >> 2);
		*mask = code;
		break;
	default:
		*tile = *mask = code;
		break;
	}
}

static void c169_tile(uint16_t code, int *tile, int *mask, int which)
{
	const snap_t *sn = s_draw_sn;
	const int bank = be8(sn->rozbank, which * 8 + ((code >> 11) & 7));
	int mangle = (code & 0x7ff) | (bank << 11);
	*mask = mangle;
	if (s_mode == MODE_NB2_OUTFXIES) {   // (bitswap<19>: bits 4 and 6 crossed)
		mangle &= 0x7ffff;
		mangle = (mangle & ~0x70) | ((mangle & 0x10) << 2) | (mangle & 0x20) | ((mangle & 0x40) >> 2);
	}
	*tile = mangle;
}

static const int s_adj[4] = { 4, 2, 1, 0 };

static void draw_c123(const snap_t *sn, int i, int y, int x0, int x1, uint16_t *d, uint8_t *pri, int p)
{
	const uint16_t cbase = (uint16_t)(0x1000 + (sn->c123ctl[0x18 + i] & 7) * 0x100);
	const int flip = (sn->c123ctl[1] & 0x8000) != 0;
	int ty, sx, cols, base, wmask;
	if (i < 4) {
		int scx = sn->c123ctl[i * 4 + 1] & 0x1ff, scy = sn->c123ctl[i * 4 + 3] & 0x1ff;
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
		const uint16_t code = rd16(sn->c123vram, ((uint32_t)(base + row + (tx >> 3)) & 0x7fff) * 2);
		uint32_t tile, mk;
		c123_tile(sn, code, &tile, &mk);
		const uint8_t m = s_mask[rbx_wrap(mk, s_mask_count) * 8 + (uint32_t)fy];
		const uint8_t *src = s_chr + rbx_wrap(tile, s_chr_count) * 64 + (uint32_t)fy * 8;
		int fx = tx & 7;
		if (flip) {
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

typedef struct {
	const snap_t *sn;
	int y0, y1;
	int cx0, cx1, cy0, cy1;   // (the window, inclusive)
} seg_t;
static seg_t s_seg[NSNAP];

#define BANDS 8

static void run_band(int band)
{
	const int r0 = VIS_H * band / BANDS, r1 = VIS_H * (band + 1) / BANDS - 1;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int si = 0; si < s_nsnap; si++) {
		const seg_t *g = &s_seg[si];
		const snap_t *sn = g->sn;
		const int y0 = g->y0 > r0 ? g->y0 : r0, y1 = g->y1 < r1 ? g->y1 : r1;
		if (y0 > y1)
			continue;
		for (int y = y0; y <= y1; y++) {
			uint16_t *d = bm + y * pitch;
			for (int x = 0; x < VIS_W; x++)
				d[x] = PAL_BLACK;
			memset(s_pri + y * VIS_W, 0, VIS_W);
		}
		const int cy0 = g->cy0 > y0 ? g->cy0 : y0, cy1 = g->cy1 < y1 ? g->cy1 : y1;
		if (g->cx0 > g->cx1 || cy0 > cy1)
			continue;
		for (int y = cy0; y <= cy1; y++) {
			uint16_t *d = bm + y * pitch;
			uint8_t *pri = s_pri + y * VIS_W;
			if (!s_nb2) {
				for (int p = 0; p < 8; p++)
					for (int i = 0; i < 6; i++) {
						const uint16_t c = sn->c123ctl[0x10 + i];
						if ((c & 7) == p && !(c & 8))
							draw_c123(sn, i, y, g->cx0, g->cx1 + 1, d, pri, p);
					}
			} else {
				c169roz_layer lp[2];
				c169roz_line_params(&s_c169, sn->c169ctl, (const uint16_t *)sn->c169ram, y, lp);
				for (int p = 0; p < 16; p++) {
					c169roz_draw_line_p(&s_c169, lp, (const uint16_t *)sn->c169ram, y, g->cx0, g->cx1 + 1, d, pri, p, (uint8_t)p);
					if (!(p & 1))
						for (int i = 0; i < 6; i++) {
							const uint16_t c = sn->c123ctl[0x10 + i];
							if ((c & 7) == p / 2 && !(c & 8))
								draw_c123(sn, i, y, g->cx0, g->cx1 + 1, d, pri, p);
						}
				}
			}
		}
		// the sprites (the C355's bitmap mixed in)
		const c355spr_rect clip = { g->cx0, g->cx1, cy0, cy1 };
		c355spr_copy(&s_c355v, bm, pitch, s_pri, VIS_W, &clip, 0);
	}
}

static decoband_t s_bands = { 0, 0, 0, 0, BANDS, run_band };

static void nb_render(void)
{
	if (!s_pal_valid) {
		uint32_t *pal = rbx_video.palette;
		for (int i = 0; i < 0x2000; i++) {
			const uint32_t r = s_pal_r[i], g = s_pal_g[i], bb = s_pal_b[i];
			pal[i] = r << 16 | g << 8 | bb;
			pal[i + 0x2000] = (r * 6 / 10) << 16 | (g * 6 / 10) << 8 | (bb * 6 / 10);   // (the shadows: unused)
		}
		for (int i = 0x4000; i < NPENS; i++)
			pal[i] = 0;
		s_pal_valid = 1;
	}
	if (!s_nsnap)
		return;
	for (int i = 0; i < s_nsnap; i++) {
		seg_t *g = &s_seg[i];
		const snap_t *sn = &s_snap[i];
		g->sn = sn;
		g->y0 = sn->y0;
		g->y1 = sn->y1;
		const int xo = s_nb2 ? 0x4b : 0x4a;
		if (s_nb2) {
			g->cx0 = sn->c116_regs[0] - xo;
			g->cx1 = sn->c116_regs[1] - xo - 1;
			g->cy0 = sn->c116_regs[2] - 0x21;
			g->cy1 = sn->c116_regs[3] - 0x21 - 1;
		} else {
			g->cx0 = (int16_t)sn->c116_regs[0] - xo;
			g->cx1 = (int16_t)sn->c116_regs[1] - xo - 1;
			g->cy0 = (int16_t)sn->c116_regs[2] - 0x21;
			g->cy1 = (int16_t)sn->c116_regs[3] - 0x21 - 1;
		}
		if (g->cx0 < 0) g->cx0 = 0;
		if (g->cx1 > VIS_W - 1) g->cx1 = VIS_W - 1;
		if (g->cy0 < 0) g->cy0 = 0;
		if (g->cy1 > VIS_H - 1) g->cy1 = VIS_H - 1;
	}
	s_draw_sn = &s_snap[s_nsnap - 1];   // (the C169's banks: the last copy's; see c169_tile)
	decoband_run(&s_bands);
}

static int nb_render_help(int helper) { return decoband_help(&s_bands, helper); }

// ---- the frame ----
static void scanline_cb(int scanline)
{
	if (scanline == 224 && b.vbl_irq_level)
		set_irq(b.vbl_irq_level, 1);
	const uint32_t posirq_scanline = (uint32_t)b.c116_regs[5] - 32;
	if ((uint32_t)scanline == posirq_scanline) {
		const int to = s_nebulray ? (int)posirq_scanline - 1 : (int)posirq_scanline;
		if (to >= 0)
			capture(to);   // (update_partial)
		if (b.pos_irq_level)
			set_irq(b.pos_irq_level, 1);
	}
}

static void mcu_run_until(int64_t t68)
{
	const int64_t target = s_mcu_t0 + t68 / MCU_DIV;
	if (b.mcu_halt) {   // (halted: its time passes)
		if (target > s_mcu.cycles) {
			s_mcu.cycles = target;
			s_mcu.icount = s_mcu.insn_icount = 0;
		}
		return;
	}
	const int64_t n = target - s_mcu.cycles - s_mcu.icount;
	if (n > 0)
		m37710_execute(&s_mcu, (int)n);
}

// ---- the 68020's wait loops (NB1_NOIDLE=1: off) ----
// The 68020's slice runs it alone (the MCU after it, the interrupt lines changed between slices or at its own sync
// points), so at the top of a loop that only tests memory, writes constants and does nothing else, its exit condition
// false and no interrupt it would take, every turn to the slice's end is the same: one turn is run (its clocks
// measured), the whole turns that fit before the slice's end are skipped, the rest run as usual: exact. The forms
// (the games' main loops waiting for their interrupts):
//   [NOP | MOVE.B / MOVE.W #i,abs.L | TST.x abs / (An) / d16(An)]...  BEQ / BNE (to the top)
// (the reads: plain memory or the CPU registers, which read 0xff; the writes: plain memory, the same value each turn)
static int s_noidle;

static int plain_rd(uint32_t a) { return m68k_bus.rd[(a & 0xffffff) >> M68K_PAGE_BITS] != NULL; }
static int plain_wr(uint32_t a) { return m68k_bus.wr[(a & 0xffffff) >> M68K_PAGE_BITS] != NULL; }
static int cpureg_rd(uint32_t a)
{
	a &= 0xffffff;
	return s_nb2 ? (a >= 0xf00000 && a < 0xf00020) : (a >= 0x400000 && a < 0x400020);
}

static void idle68(int64_t t)
{
	const uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC) & 0xffffff;
	if (pc + 64 > s_prg_size)
		return;
	{	// (an interrupt it would take: run)
		const uint32_t lines = m68k_bus.irq_pending | m68k_bus.irq_asserted;
		for (int l = 7; l > 0; l--)
			if (lines & (1u << l)) {
				if (l == 7 || l > (int)((m68k_get_reg(NULL, M68K_REG_SR) >> 8) & 7))
					return;
				break;
			}
	}
	uint32_t o = pc;
	int z = -1;   // (the last test's Z flag)
	for (int step = 0; step < 12; step++) {
		const uint16_t w = rd16(s_prg, o);
		if (w == 0x4e71) {   // NOP
			o += 2;
			continue;
		}
		if (w == 0x13fc || w == 0x33fc) {   // MOVE.B / MOVE.W #i,abs.L
			const uint32_t a = ((uint32_t)rd16(s_prg, o + 4) << 16 | rd16(s_prg, o + 6)) & 0xffffff;
			if (!plain_wr(a))
				return;
			o += 8;
			continue;
		}
		const int sz = (w >> 6) & 3;   // TST.B / .W / .L (4a00 | size << 6 | mode)
		if ((w & 0xff00) == 0x4a00 && sz < 3) {
			const int mode = (w >> 3) & 7, reg = w & 7;
			uint32_t a;
			if (mode == 7 && reg == 0) {   // abs.W
				a = (uint32_t)(int32_t)(int16_t)rd16(s_prg, o + 2) & 0xffffff;
				o += 4;
			} else if (mode == 7 && reg == 1) {   // abs.L
				a = ((uint32_t)rd16(s_prg, o + 2) << 16 | rd16(s_prg, o + 4)) & 0xffffff;
				o += 6;
			} else if (mode == 2) {   // (An)
				a = m68k_get_reg(NULL, (m68k_register_t)(M68K_REG_A0 + reg)) & 0xffffff;
				o += 2;
			} else if (mode == 5) {   // d16(An)
				a = (m68k_get_reg(NULL, (m68k_register_t)(M68K_REG_A0 + reg)) + (uint32_t)(int16_t)rd16(s_prg, o + 2)) & 0xffffff;
				o += 4;
			} else
				return;
			uint32_t v;
			if (cpureg_rd(a))
				v = sz == 0 ? 0xff : sz == 1 ? 0xffff : 0xffffffffu;
			else if (plain_rd(a) && plain_rd(a + 3))
				v = sz == 0 ? m68k_read_memory_8(a) : sz == 1 ? m68k_read_memory_16(a) : m68k_read_memory_32(a);
			else
				return;
			z = v == 0;
			continue;
		}
		if ((w & 0xff00) == 0x6700 || (w & 0xff00) == 0x6600) {   // BEQ / BNE to the top
			const int8_t d8 = (int8_t)(w & 0xff);
			if (!d8 || o + 2 + (uint32_t)(int32_t)d8 != pc || z < 0)
				return;
			if ((w & 0xff00) == 0x6700 ? !z : z)
				return;   // (it leaves)
			break;
		}
		return;
	}
	// one turn, its clocks measured
	int tc = 0;
	for (int k = 0; k < 16; k++) {
		tc += m68k_execute(1);
		if ((m68k_get_reg(NULL, M68K_REG_PC) & 0xffffff) == pc)
			break;
	}
	b.t68 += tc;
	if ((m68k_get_reg(NULL, M68K_REG_PC) & 0xffffff) != pc || tc <= 0 || b.t68 >= t)
		return;
	const int64_t n = (t - b.t68 - 1) / tc;
	if (n > 0)
		b.t68 += n * tc;
}

// the 68020 to the time t (the slices end at its sync points)
#ifdef RBX_M68K_TRACE
extern long long rbx_m68k_trace_base;
#endif
static void run68_until(int64_t t)
{
	while (b.t68 < t) {
		if (!s_noidle) {
			idle68(t);
			if (b.t68 >= t)
				break;
		}
		s_sync = 0;
#ifdef RBX_M68K_TRACE
		rbx_m68k_trace_base = b.t68;
#endif
		b.t68 += m68k_execute((int)(t - b.t68));
	}
}

static void nb_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int i = 0; i < 4; i++)   // (the light guns: the pointer's moves)
		if (s_port_gun[i] >= 0) {
			int v = b.gun[i] + rbx_analog(i / 2 + 1, (i & 1) ? RBX_IPT_LIGHTGUN_Y : RBX_IPT_LIGHTGUN_X);
			b.gun[i] = v < 0 ? 0 : v > 255 ? 255 : v;
		}
	b.mcu_frame0 = s_mcu.cycles;
	b.snd_done = 0;
	for (int idx = 0; idx < LINES; idx++) {
		const int line = (idx + 224) % LINES;   // (the frame from the vblank: MAME starts there)
		// the scanline timer runs from line 0 of the first frame
		if (b.frames > 0 || idx >= LINES - 224)
			scanline_cb(line);
		b.t68_target += LINE68;
		while (b.mcuirq_next <= b.t68_target) {   // (the MCU's 60 Hz timers within the line)
			run68_until(b.mcuirq_next);
			mcu_run_until(b.mcuirq_next);
			m37710_set_input(&s_mcu, M37710_LINE_IRQ0, 1);
			m37710_set_input(&s_mcu, M37710_LINE_IRQ2, 1);
			b.mcuirq_next += MCUIRQ_PERIOD;
		}
		run68_until(b.t68_target);
		mcu_run_until(b.t68_target);
		snd_to((idx + 1) * SND_FRAME / LINES);
	}
	snd_to(SND_FRAME);
	frame_capture();
	screen_vblank();
	b.frames++;
	for (int i = 0; i < SND_FRAME; i++) {
		rbx_audio_put_stereo(s_mix_l[i], s_mix_r[i]);
		s_mix_l[i] = s_mix_r[i] = 0;
	}
}

// ---- init / reset ----
static void nb_reset(void)
{
	b.vbl_irq_level = b.pos_irq_level = b.unk_irq_level = 0;
	b.mcu_halt = 0;
	b.port6 = 0;
	b.count = 0;
	b.rng = 0x9d14abd7;
	b.t68 = b.t68_target = 0;
	b.mcuirq_next = MCUIRQ_PERIOD;
	b.frames = 0;
	for (int i = 0; i < 4; i++)
		b.gun[i] = 0x80;
	m68k_bus_irq_reset();
	for (int l = 1; l < 8; l++)
		m68k_bus_set_line(l, 0);
	m68k_pulse_reset();
	b.t68 = 4;   // (MAME's 68020: the first instruction 4 clocks after the reset)
	m37710_reset(&s_mcu);
	s_mcu_t0 = s_mcu.cycles;
	c352_reset(&s_c352);
	c355spr_reset(&s_c355);
	s_frame_nsnap = 0;
	s_last_line = 0;
	s_vram_w = s_roz_w = 1;
	s_pal_dirty = 1;
}

static const struct { const char *init; int type; } s_types[] = {
	{ "nebulray", GT_NEBULRAY }, { "gslgr94u", GT_GSLGR94U }, { "gslgr94j", GT_GSLGR94J }, { "sws95", GT_SWS95 },
	{ "sws96", GT_SWS96 }, { "sws97", GT_SWS97 }, { "gunbulet", GT_GUNBULET }, { "vshoot", GT_VSHOOT },
	{ "machbrkr", GT_MACHBRKR }, { "outfxies", GT_OUTFOXIES },
};

static void nb_exit(void);

static int nb_init(const rbx_game *g)
{
	if (!s_snap_hp && !(s_snap_hp = calloc(1, sizeof(*s_snap_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_prg = rbx_region("maincpu", &s_prg_size);
	s_data = rbx_region("data", &s_data_size);
	s_c75data = rbx_region("c75data", &s_c75data_size);
	s_mcurom = rbx_region("mcu", &s_mcurom_size);
	s_chr = rbx_region("c123tmap", &s_chr_size);
	s_mask = rbx_region("c123tmap:mask", &s_mask_size);
	s_c352rom = rbx_region("c352", &s_c352_size);
	uint32_t spr_size = 0;
	const uint8_t *spr = rbx_region("c355spr", &spr_size);
	if (!s_prg || !s_mcurom || s_mcurom_size < 0x4000 || !s_chr || !s_mask || !spr) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_prg, s_prg_size);
	if (s_data)
		m68k_bus_swap16(s_data, s_data_size);
	s_chr_count = s_chr_size / 64;
	s_mask_count = s_mask_size / 8;
	const char *init = strchr(g->machine, ':');
	init = init ? init + 1 : "";
	s_gametype = GT_NONE;
	for (size_t i = 0; i < sizeof(s_types) / sizeof(s_types[0]); i++)
		if (!strcmp(init, s_types[i].init))
			s_gametype = s_types[i].type;
	s_mode = !strncmp(g->machine, "gunbulet:", 9) ? MODE_GUN : !strncmp(g->machine, "outfxies:", 9) ? MODE_NB2_OUTFXIES
	       : !strncmp(g->machine, "machbrkr:", 9) ? MODE_NB2_MACHBRKR : MODE_NB1;
	s_nb2 = s_mode == MODE_NB2_OUTFXIES || s_mode == MODE_NB2_MACHBRKR;
	s_nebulray = s_gametype == GT_NEBULRAY;
	static const char *const pn[4] = { "P1", "P2", "P3", "P4" };
	for (int i = 0; i < 4; i++)
		s_port_p[i] = rbx_port_find(pn[i]);
	s_port_misc = rbx_port_find("MISC");
	static const char *const gn[4] = { "LIGHT0_X", "LIGHT0_Y", "LIGHT1_X", "LIGHT1_Y" };
	for (int i = 0; i < 4; i++)
		s_port_gun[i] = rbx_port_find(gn[i]);

	if (!s_bp && !(s_bp = (struct nb_state *)malloc(sizeof(*s_bp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	memset(&b, 0, sizeof(b));
	memset(b.eeprom, 0xff, sizeof(b.eeprom));
	{
		uint32_t esz = 0;
		const uint8_t *e = rbx_region("eeprom", &esz);
		if (e)
			memcpy(b.eeprom, e, esz < sizeof(b.eeprom) ? esz : sizeof(b.eeprom));
	}
	for (int i = 0; i < VPOOL; i++)
		if (!s_vpool[i] && !(s_vpool[i] = (uint8_t *)calloc(1, sizeof(b.c123vram)))) {
			nb_exit();
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
	for (int i = 0; i < RPOOL && s_nb2; i++)
		if (!s_rpool[i] && !(s_rpool[i] = (uint8_t *)calloc(1, sizeof(b.c169ram)))) {
			nb_exit();
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
	s_pri = (uint8_t *)calloc(1, VIS_W * VIS_H);
	s_sprbm = (uint16_t *)malloc((size_t)VIS_W * VIS_H * sizeof(uint16_t));
	if (!s_pri || !s_sprbm || c355spr_init(&s_c355, VIS_W, VIS_H, spr, spr_size) || rbx_audio_setup_stereo(SND_RATE) ||
	    rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		nb_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	memset(s_sprbm, 0xff, (size_t)VIS_W * VIS_H * sizeof(uint16_t));
	s_c355.scrolloffs[0] = 0x26;
	s_c355.scrolloffs[1] = 0x19;
	s_c355.buffer = 2;   // (triple buffered)
	s_c355.colbase = 0;
	s_c355.code2tile = objcode2tile;
	s_c355.mix_cb = sprite_mix;
	s_c355v = s_c355;
	s_c355v.bitmap = s_sprbm;
	memset(&s_c169, 0, sizeof(s_c169));
	if (s_nb2) {
		uint32_t gs = 0, ms = 0;
		s_c169.gfx = rbx_region("c169roz", &gs);
		s_c169.mask = rbx_region("c169roz:mask", &ms);
		s_c169.count = s_c169.gfx ? gs / 256 : 0;
		s_c169.mask_count = s_c169.mask ? ms / 32 : 0;
		s_c169.ram_words = 0x20000 / 2;
		s_c169.color_base = 0x1800;
		s_c169.cb = c169_tile;
	}
	// the 68EC020
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68EC020);
	m68k_set_mame_timing(1);   // (MAME's 68EC020 times: m68k.h)
	m68k_bus_reset_map();
	m68k_bus_map(0x000000, 0x0fffff, s_prg, s_prg_size, 1, 0);
	m68k_bus_map(0x1c0000, 0x1cffff, b.ram1c, sizeof(b.ram1c), 1, 1);
	m68k_bus_map(0x200000, 0x207fff, b.share, sizeof(b.share), 1, 1);
	m68k_bus_map(0x208000, 0x2fffff, b.ram208, sizeof(b.ram208), 1, 1);
	m68k_bus_map(0x600000, 0x61ffff, s_c355.ram[0], sizeof(s_c355.ram[0]), 1, 0);   // (writes: the page 0 tables)
	if (!s_nb2) {
		m68k_bus_map(0x640000, 0x64ffff, b.c123vram, sizeof(b.c123vram), 1, 0);   // (writes: bus_w16m)
	} else {
		if (s_data)
			m68k_bus_map(0x400000, 0x4fffff, s_data, s_data_size, 1, 0);
		m68k_bus_map(0x680000, 0x68ffff, b.c123vram, sizeof(b.c123vram), 1, 0);
		m68k_bus_map(0x700000, 0x71ffff, b.c169ram, sizeof(b.c169ram), 1, 0);
	}
	m68k_bus.read8 = bus_r8;
	m68k_bus.read16 = bus_r16;
	m68k_bus.read32 = bus_r32;
	m68k_bus.write8 = bus_w8;
	m68k_bus.write16 = bus_w16;
	// the C75 (an M37702M2 with its BIOS)
	m37710_init(&s_mcu);
	s_mcu.rom = s_mcurom;
	for (int p = 4; p < 0xc; p++) {   // (the shared RAM: the sync flag's page writes through mcu_w)
		s_mcu.rpage[p] = b.share + (p - 4) * 0x1000;
		s_mcu.wpage[p] = p == 0xa ? NULL : b.share + (p - 4) * 0x1000;
	}
	if (s_c75data)
		for (uint32_t p = 0; p < 0x80; p++)
			s_mcu.rpage[0x200 + p] = s_c75data + (p * 0x1000) % s_c75data_size;
	s_mcu.read = mcu_r;
	s_mcu.write = mcu_w;
	s_mcu.read16 = mcu_r16;
	s_mcu.write16 = mcu_w16;
	s_mcu.port_read = mcu_port_r;
	s_mcu.port_write = mcu_port_w;
	s_mcu.analog_read = mcu_an_r;
	m37710_set_clock(&s_mcu, MCU_CLOCK);
	s_noidle = getenv("NB1_NOIDLE") != NULL;
	s_mcu.idle_skip = !s_noidle;   // (the BIOS's wait for its interrupts)
	c352_init(&s_c352, s_c352rom, s_c352rom ? s_c352_size : 0);
	s_pal_valid = 0;
	s_nsnap = 0;
	nb_reset();
	return 0;
}

static void nb_exit(void)
{
	m68k_set_mame_timing(0);
	for (int i = 0; i < VPOOL; i++) {
		free(s_vpool[i]);
		s_vpool[i] = NULL;
	}
	for (int i = 0; i < RPOOL; i++) {
		free(s_rpool[i]);
		s_rpool[i] = NULL;
	}
	c355spr_free(&s_c355);
	free(s_sprbm);
	free(s_pri);
	s_sprbm = NULL;
	s_pri = NULL;
	s_nsnap = 0;
	memset(&s_c355, 0, sizeof(s_c355));
	memset(&s_c355v, 0, sizeof(s_c355v));
	free(s_bp);   // (last: the render thread and the saves are done with it)
	s_bp = NULL;
	free(s_snap_hp);
	s_snap_hp = NULL;
}

static int nb_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	int rc = 0;
	if ((uint32_t)m68k_context_size() > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	rc |= rbx_state_io(ctx, "nb1_cpu", cpu, (uint32_t)m68k_context_size());
	rc |= rbx_state_io(ctx, "nb1_bus", &m68k_bus.irq_pending, sizeof(uint32_t) * 2);
	rc |= rbx_state_io(ctx, "nb1", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= rbx_state_io(ctx, "nb1_mcu", &s_mcu, M37710_STATE_SIZE);
	rc |= rbx_state_io(ctx, "nb1_mcut0", &s_mcu_t0, sizeof(s_mcu_t0));
	rc |= rbx_state_io(ctx, "nb1_c352", &s_c352, (uint32_t)C352_STATE_SIZE);
	rc |= rbx_state_io(ctx, "nb1_c355", C355SPR_SAVE_PTR(&s_c355), (uint32_t)C355SPR_SAVE_SIZE);
	rc |= rbx_state_io(ctx, "nb1_c355bm", s_c355.bitmap, (uint32_t)(VIS_W * VIS_H * sizeof(uint16_t)));
	if (!rc && load) {
		m68k_set_context(cpu);
		m37710_post_load(&s_mcu);
		s_pal_dirty = 1;
		s_vram_w = s_roz_w = 1;
	}
	return rc ? -1 : 0;
}

static int nb_save(void *ctx) { return nb_state(ctx, 0); }
static int nb_load(void *ctx) { return nb_state(ctx, 1); }
static int nb_supported(const char *machine)
{
	return !strncmp(machine, "namconb1:", 9) || !strncmp(machine, "gunbulet:", 9) || !strncmp(machine, "outfxies:", 9) ||
	       !strncmp(machine, "machbrkr:", 9);
}

const rbx_driver rbx_drv_namconb1 = { "Namco NB-1/NB-2", nb_init, nb_exit, nb_reset, nb_frame, nb_save, nb_load,
                                      nb_supported, nb_render, nb_render_help };
