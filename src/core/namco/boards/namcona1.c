/*  Namco NA-1 / NA-2: an MC68000, the C69 / C70 MCU (an M37702 with Namco's BIOS: it boots the 68000, reads the
    inputs, drives the sound), the C219 PCM chip on the MCU's bus (its samples in the shared work RAM), the blitter,
    four scrolling 64 x 64 tilemaps with a line scroll / line select, the ROZ layer, the sprites with their shape
    masks, the graphics RAM (8 / 4 bpp tiles and 1 bpp shapes), the key custom chips and the parallel EEPROM.
    Ported from MAME 0.289 src/mame/namco/namcona1.cpp, namcona1.h, namcona1_v.cpp (BSD-3-Clause,
    copyright-holders: Phil Stroffolino), src/mame/namco/namcomcu.cpp (BSD-3-Clause, copyright-holders: Alex W.
    Jackson), src/devices/machine/eeprompar.cpp and the tile caching of src/emu/tilemap.cpp (BSD-3-Clause,
    copyright-holders: Aaron Giles), the random words of src/emu/machine.cpp running_machine::rand (BSD-3-Clause,
    copyright-holders: Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.

    As MAME: the 68000 and the MCU in scanline slices (the MCU caught up to the 68000's time when it writes mailbox 4,
    MAME's set_input_line sync), the 68000 started on the slice after the MCU releases it, the C219 rendered up to the
    MCU's time at each access; the tilemaps' tiles cached as MAME's tilemap system does (a shape RAM write alone does
    not reach a cached tile); the 68000 with MAME's operand-dependent DIVx / MULx times (m68k_set_mame_timing).
    Not done: X-Day 2's printer and RTC (the RTC reads 0), Zelos / Bin Bin Bingo's terminal link (MAME: not
    working either), the flip screen (MAME: TBA).
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/m37710/m37710.h"
#include "../../sound/c140.h"
#include "../../video/decoband.h"

#include <stdlib.h>
#include <string.h>

#define VIS_W 304                 // (the visible area: 0-303 x 32-255 of 320 x 256)
#define VIS_Y0 32
#define VIS_H 224
#define LINES 256
#define FRAME_MHZ 60000
#define M68K_CLOCK 12528250       // 50.113 MHz / 4
#define MCU_CLOCK 12528250        // (its input clock; the CPU runs at half: 6264125 cycles a second)
#define MCU_HZ 6264125
#define SND_RATE 44100
#define SND_FRAME 735             // (44100 / 60)
#define NPENS 0x4000              // 0x1000 pens, their 0x1000 alternate (RRRGGGBB) pens, the 0x2000 shadows

enum {
	GT_NONE = -1, GT_CGANGPZL, GT_EMERALDA, GT_KNCKHEAD, GT_BKRTMAQ, GT_EXVANIA, GT_QUIZTOU, GT_SWCOURT, GT_TINKLPIT,
	GT_NUMANATH, GT_FA, GT_XDAY2, GT_SWCOURTB
};

// ---- the set's configuration ----
static uint8_t *s_prg, *s_mask, *s_mcurom;
static uint32_t s_prg_size, s_mask_size, s_mcurom_size;
static int s_gametype;
static int s_xday2, s_zelos;
static uint32_t s_eeprom_size;
static int s_port_mux[4], s_port_p3;

static m37710_t s_mcu;
static c140_t s_c219;

// ---- the board's state (saved: up to state_end) ----
static struct na1_state {   // (on the heap while the game runs, ADR-040)
	uint8_t workram[0x80000];    // 0x000000 (host-order words; the MCU's 0x200000 and the C219's samples)
	uint8_t videoram[0xc000];    // 0xff0000
	uint8_t ram_c000[0x2000];    // 0xffc000
	uint8_t scroll[0x1000];      // 0xffe000
	uint8_t spriteram[0x1000];   // 0xfff000
	uint8_t cgram[0x40000];      // 0xf40000 (graphics RAM, mode 2)
	uint8_t shaperam[0x8000];    // (graphics RAM, mode 3: plain bytes in the 68000's order)
	uint8_t mcuram[0x8000];      // the MCU's 0x3000-0xafff
	uint8_t eeprom[0x2000];
	uint16_t vreg[0x80];         // 0xefff00
	uint16_t paletteram[0x1000]; // 0xf00000
	uint32_t pal_rgb[0x2000];    // the palette device's pens 0-0x1fff
	uint16_t mailbox[8];
	uint8_t port4, port5, port6, port8;
	uint8_t enable_interrupts, palette_is_dirty, m68k_held, m68k_needreset;
	uint16_t count, zelos_ctrl;
	uint32_t keyval, rng;
	int64_t t68, t68_target;     // the 68000's cycles since reset / the end of the current line
	int64_t mcu_frame0;          // the MCU's cycles at the frame's start
	uint64_t m68k_frac;
	int32_t snd_done;            // samples of this frame made
	// the tilemaps' tiles as MAME's tilemap system caches them (the four layers, the ROZ layer): taken again when a
	// tile is marked dirty (its video RAM written, the CG RAM changed, the layer's bpp switched) and the layer drawn;
	// a shape RAM write alone leaves the cached mask (MAME reads the mask in tile_update only)
	uint32_t tinfo[5][4096];     // tile | color << 12 | opaque << 15 | bpp4 << 16
	uint8_t tmask[5][4096][8];
	uint8_t tdirty[5][4096];
	int32_t ndirty[5];           // (tiles marked dirty a layer)
	int32_t nvis[5];             // (cached tiles with a pixel to draw a layer: none, the layer is not drawn)
	uint32_t gfx_seq, layer_seq[5];
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)

// ---- the drawing's copies: one a screen update (the position IRQ's partial update and the vblank) ----
typedef struct {
	uint16_t vreg[0x80];
	uint8_t videoram[0xc000];
	uint8_t scroll[0x1000];
	uint8_t spriteram[0x1000];
	uint8_t cgram[0x40000];
	uint8_t shaperam[0x8000];
	uint32_t tinfo[5][4096];
	uint8_t tmask[5][4096][8];
	int32_t nvis[5];
	int y0, y1;                  // the lines it draws (screen lines, inclusive)
} snap_t;
static snap_t *s_snap[2];
static int s_nsnap;              // copies of the frame being drawn
static int s_frame_nsnap;        // (the emulation side: copies made this frame)
static int s_last_line;          // (the emulation side: the next line to draw)
static uint8_t s_cg_dirty[2][64];   // (the graphics RAM's 4 KB pages written since that copy was made)
static uint8_t s_tc_dirty[2][5];    // (the tile caches refreshed since that copy was made)
static uint32_t (*s_pal_snap_hp)[0x2000];
#define s_pal_snap (*s_pal_snap_hp)
static int s_pal_snap_dirty, s_pal_valid, s_pal_full;   // (full: every pen set again)
static uint8_t *s_pri;           // VIS_W x VIS_H

static int32_t s_mix_l[SND_FRAME + 64], s_mix_r[SND_FRAME + 64];

static inline uint16_t rd16(const uint8_t *mem, uint32_t a) { return *(const uint16_t *)(mem + (a & ~1u)); }
static inline void wr16(uint8_t *mem, uint32_t a, uint16_t v, uint16_t mask)
{
	uint16_t *p = (uint16_t *)(mem + (a & ~1u));
	*p = (uint16_t)((*p & ~mask) | (v & mask));
}
static inline uint8_t rd8(const uint8_t *mem, uint32_t a) { return mem[a ^ M68K_BYTE_XOR]; }
static inline uint64_t rd64(const uint8_t *p) { uint64_t v; memcpy(&v, p, 8); return v; }

// ---- sound: the C219 made up to the MCU's time (MAME updates its stream on every access) ----
static int64_t mcu_now(void) { return s_mcu.cycles + (s_mcu.insn_icount - s_mcu.icount); }

static void snd_to(int n)
{
	if (n > SND_FRAME)
		n = SND_FRAME;
	if (n > b.snd_done) {
		// (MAME routes output 0 to the right speaker, 1 to the left)
		c140_render(&s_c219, s_mix_r + b.snd_done, s_mix_l + b.snd_done, n - b.snd_done, SND_RATE);
		b.snd_done = n;
	}
}

static void snd_catchup(void)
{
	const int64_t e = mcu_now() - b.mcu_frame0;
	snd_to(e <= 0 ? 0 : (int)((uint64_t)e * (SND_RATE * 60) / MCU_HZ));
}

// ---- the palette (update_palette) ----
static inline uint32_t pal5(int v) { v &= 0x1f; return (uint32_t)((v << 3) | (v >> 2)); }
static void update_palette(int offset)
{
	const uint16_t data = b.paletteram[offset];   // -RRRRRGG GGGBBBBB
	// the sprites' alternate interpretation (Emeraldia): RRRGGGBB RRRGGGBB
	const int r = (((data & 0x00e0) >> 5) + ((data & 0xe000) >> 13) * 2) * 0xff / (0x7 * 3);
	const int g = (((data & 0x001c) >> 2) + ((data & 0x1c00) >> 10) * 2) * 0xff / (0x7 * 3);
	const int bl = (((data & 0x0003) >> 0) + ((data & 0x0300) >> 8) * 2) * 0xff / (0x3 * 3);
	b.pal_rgb[offset + 0x1000] = (uint32_t)(r << 16 | g << 8 | bl);
	b.pal_rgb[offset] = pal5(data >> 10) << 16 | pal5(data >> 5) << 8 | pal5(data);
	s_pal_snap_dirty = 1;
}

static void paletteram_w(uint32_t offset, uint16_t data, uint16_t mask)
{
	offset &= 0xfff;
	b.paletteram[offset] = (uint16_t)((b.paletteram[offset] & ~mask) | (data & mask));
	if (b.vreg[0x8e / 2])
		update_palette((int)offset);   // (graphics enabled: at once)
	else
		b.palette_is_dirty = 1;
}

// ---- the graphics RAM ----
static uint16_t gfxram_r(uint32_t offset)
{
	const uint16_t type = b.vreg[0x0c / 2];
	if (type == 0x03) {
		if (offset < 0x4000)
			return (uint16_t)(b.shaperam[offset * 2] << 8 | b.shaperam[offset * 2 + 1]);
	} else if (type == 0x02)
		return rd16(b.cgram, offset * 2);
	return 0;
}

static void gfxram_w(uint32_t offset, uint16_t data, uint16_t mask)
{
	const uint16_t type = b.vreg[0x0c / 2];
	if (type == 0x03) {
		if (offset < 0x4000) {
			if (mask & 0xff00)
				b.shaperam[offset * 2] = (uint8_t)(data >> 8);
			if (mask & 0x00ff)
				b.shaperam[offset * 2 + 1] = (uint8_t)data;
		}
	} else if (type == 0x02) {
		const uint16_t old = rd16(b.cgram, offset * 2);
		wr16(b.cgram, offset * 2, data, mask);
		if (rd16(b.cgram, offset * 2) != old) {   // (MAME: the gfx marked dirty, the tilemaps then redrawn whole)
			s_cg_dirty[0][(offset * 2) >> 12] = s_cg_dirty[1][(offset * 2) >> 12] = 1;
			b.gfx_seq++;
		}
	}
}

static inline void tile_dirty(uint32_t which, uint32_t i)
{
	if (!b.tdirty[which][i]) {
		b.tdirty[which][i] = 1;
		b.ndirty[which]++;
	}
}

static void layer_dirty(int which)
{
	memset(b.tdirty[which], 1, sizeof(b.tdirty[which]));
	b.ndirty[which] = 4096;
}

// videoram_w: the tiles marked dirty (the ROZ layer's by MAME's own index arithmetic)
static void videoram_w(uint32_t offset, uint16_t data, uint16_t mask)
{
	wr16(b.videoram, offset * 2, data, mask);
	if (offset < 0x8000 / 2)
		tile_dirty(offset >> 12, offset & 0xfff);
	else if (offset < 0x8800 / 2) {
		for (int i = 0; i < 4; i++)
			for (int k = 0; k < 4; k++)
				tile_dirty(4, (((offset & 0x3cf) << 2) + (uint32_t)i + (uint32_t)k * 0x40) & 0xfff);
	}
}

// ---- the blitter ----
static int transfer_dword(uint32_t dest, uint32_t source)
{
	uint16_t data;
	if (source >= 0x400000 && source < 0xc00000)
		data = source - 0x400000 < s_mask_size ? rd16(s_mask, source - 0x400000) : 0;
	else if (source >= 0xc00000 && source < 0xe00000)
		data = source - 0xc00000 < s_prg_size ? rd16(s_prg, source - 0xc00000) : 0;
	else if (source < 0x80000 && source >= 0x1000)
		data = rd16(b.workram, source);
	else
		return -1;   // (bad blit source)
	if (dest >= 0xf00000 && dest < 0xf02000)
		paletteram_w((dest - 0xf00000) / 2, data, 0xffff);
	else if (dest >= 0xf40000 && dest < 0xf80000)
		gfxram_w((dest - 0xf40000) / 2, data, 0xffff);
	else if (dest >= 0xff0000 && dest < 0xffc000)
		videoram_w((dest - 0xff0000) / 2, data, 0xffff);
	else if (dest >= 0xfff000 && dest < 0x1000000)
		wr16(b.spriteram, dest - 0xfff000, data, 0xffff);
	else if (dest >= 0xf00000 * 2 && dest < 0xf02000 * 2)   // (xday2: a mirror for the palette, MAME assumes)
		paletteram_w((dest - 0xf00000 * 2) / 2, data, 0xffff);
	else
		return -1;
	return 0;
}

static void blit_setup(int format, int *bytes_per_row, int *pitch, int mode)
{
	if (mode == 3) {   // tile data
		switch (format) {
		case 0x0001: *bytes_per_row = 0x1000; *pitch = 0x1000; break;
		case 0x0081: *bytes_per_row = 4 << 3; *pitch = 36 << 3; break;
		default: *bytes_per_row = (64 - (format >> 2)) << 3; *pitch = 0x200; break;
		}
	} else {   // shape data
		switch (format) {
		case 0x00bd: *bytes_per_row = 4; *pitch = 0x120; break;   // (Numan Athletics)
		case 0x008d: *bytes_per_row = 8; *pitch = 0x120; break;
		case 0x0000: *bytes_per_row = 0x10; *pitch = 0; break;
		case 0x0001: *bytes_per_row = 0x1000; *pitch = 0x1000; break;
		case 0x0401: *bytes_per_row = 4 << 6; *pitch = 36 << 6; break;   // (F/A)
		default: *bytes_per_row = (64 - (format >> 5)) << 6; *pitch = 0x1000; break;
		}
	}
}

static void blit(void)
{
	const int src1 = b.vreg[0x1], dst1 = b.vreg[0x4], gfxbank = b.vreg[0x6];
	uint32_t src_baseaddr = 2 * (0xffffff & ((uint32_t)b.vreg[0x7] << 16 | b.vreg[0x8]));
	uint32_t dst_baseaddr = 2 * (0xffffff & ((uint32_t)b.vreg[0x9] << 16 | b.vreg[0xa]));
	int num_bytes = b.vreg[0xb];
	int dst_bytes_per_row, dst_pitch, src_bytes_per_row, src_pitch;
	blit_setup(dst1, &dst_bytes_per_row, &dst_pitch, gfxbank);
	blit_setup(src1, &src_bytes_per_row, &src_pitch, gfxbank);
	if (num_bytes & 1)
		num_bytes++;
	if (dst_baseaddr < 0xf00000)
		dst_baseaddr += 0xf40000;
	int dst_offset = 0, src_offset = 0;
	while (num_bytes > 0) {
		if (transfer_dword(dst_baseaddr + (uint32_t)dst_offset, src_baseaddr + (uint32_t)src_offset))
			return;
		num_bytes -= 2;
		dst_offset += 2;
		if (dst_offset >= dst_bytes_per_row) {
			dst_baseaddr += (uint32_t)dst_pitch;
			dst_offset = 0;
		}
		src_offset += 2;
		if (src_offset >= src_bytes_per_row) {
			src_baseaddr += (uint32_t)src_pitch;
			src_offset = 0;
		}
	}
}

static void vreg_w(uint32_t offset, uint16_t data, uint16_t mask)
{
	offset &= 0x7f;
	const uint16_t old = b.vreg[offset];
	b.vreg[offset] = (uint16_t)((b.vreg[offset] & ~mask) | (data & mask));
	switch (offset) {
	case 0x18 / 2: blit(); break;
	case 0x1a / 2: b.enable_interrupts = 1; break;   // (interrupt enable mask; 0 enables the level)
	case 0xbc / 2:   // a layer's bpp switched: all its tiles dirty
		for (int i = 0; i <= 4; i++)
			if ((old ^ b.vreg[offset]) & (1 << i))
				layer_dirty(i);
		break;
	default: break;
	}
}

// ---- the key custom chip ----
// MAME's running_machine::rand (its seed 0x9d14abd7 from the machine's start): the key chip's random words
static uint16_t rand16(void)
{
	b.rng = 1664525u * b.rng + 1013904223u;
	return (uint16_t)((b.rng >> 16) | (b.rng << 16));
}

static uint16_t custom_key_r(int offset)
{
	const uint16_t old_count = b.count;
	do
		b.count = rand16();
	while (old_count == b.count);
	switch (s_gametype) {
	case GT_BKRTMAQ: if (offset == 2) return 0x015c; break;
	case GT_FA: if (offset == 2) return 0x015d; if (offset == 4) return b.count; break;
	case GT_EXVANIA: if (offset == 2) return 0x015e; break;
	case GT_CGANGPZL: if (offset == 1) return 0x0164; if (offset == 2) return b.count; break;
	case GT_SWCOURT: if (offset == 1) return 0x0165; if (offset == 2) return b.count; break;
	case GT_EMERALDA: if (offset == 1) return 0x0166; if (offset == 2) return b.count; break;
	case GT_NUMANATH: if (offset == 1) return 0x0167; if (offset == 2) return b.count; break;
	case GT_KNCKHEAD: if (offset == 1) return 0x0168; if (offset == 2) return b.count; break;
	case GT_QUIZTOU: if (offset == 2) return 0x016d; break;
	case GT_TINKLPIT:
		if (offset == 7) return 0x016f;
		if (offset == 4) b.keyval = 0;
		if (offset == 3) {
			static const uint8_t bits[16] = { 22, 26, 31, 23, 18, 20, 16, 30, 24, 21, 25, 19, 17, 29, 28, 27 };
			uint16_t res = 0;
			for (int i = 0; i < 16; i++)
				res |= (uint16_t)(((b.keyval >> bits[i]) & 1) << (15 - i));
			b.keyval >>= 1;
			if (!b.keyval || (__builtin_popcount(b.keyval & 0x58000c00) & 1))
				b.keyval ^= 0x80000000u;
			return res;
		}
		break;
	case GT_XDAY2: if (offset == 2) return 0x018a; if (offset == 3) return b.count; break;
	case GT_SWCOURTB: if (offset == 1) return 0x8061; if (offset == 2) return b.count; break;
	default: return 0;
	}
	return rand16();
}

// ---- the MCU's mailbox ----
static void write_version_info(void)
{
	// NA-2: the BIOS version string the game looks for after its command ("NSA-BIOS ver1.31")
	static const uint16_t source[8] = { 0x534e, 0x2d41, 0x4942, 0x534f, 0x7620, 0x7265, 0x2e31, 0x3133 };
	for (int i = 0; i < 8; i++)
		wr16(b.workram, 0x1000 + (uint32_t)i * 2, source[i], 0xffff);
}

static int s_sync_irq0;   // (the 68000 wrote mailbox 4: the MCU catches up, then takes IRQ 0)

static void mailbox_w_68k(uint32_t offset, uint16_t data, uint16_t mask)
{
	if (offset == 4) {
		s_sync_irq0 = 1;   // (MAME set_input_line: synced to the 68000's time)
		m68k_end_timeslice();
	}
	uint16_t *m = &b.mailbox[offset & 7];
	*m = (uint16_t)((*m & ~mask) | (data & mask));
	// (MAME: the C70 BIOS never answers the version command)
	if ((s_gametype == GT_NUMANATH || s_gametype == GT_KNCKHEAD) && (rd16(b.workram, 0xf72) >> 8) == 7)
		write_version_info();
}

// ---- the 68000's bus ----
static uint16_t bus_r16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x3f8000 && a < 0x400000)
		return b.mailbox[(a >> 1) & 7];
	if (a >= 0xe00000 && a < 0xe00000 + s_eeprom_size * 2)
		return b.eeprom[(a - 0xe00000) >> 1];
	if (a >= 0xe40000 && a < 0xe40010)
		return custom_key_r((int)((a >> 1) & 7));
	if (a >= 0xf40000 && a < 0xf80000)
		return gfxram_r((a - 0xf40000) >> 1);
	if (s_xday2) {
		if ((a & ~1u) == 0xd80000)
			return 3;   // (printer status: battery ok)
	}
	return 0;
}

static uint8_t bus_r8(uint32_t a)
{
	const uint16_t v = bus_r16(a & ~1u);
	return (uint8_t)((a & 1) ? v : v >> 8);
}

static void bus_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a >= 0xefff00 && a < 0xf00000) {
		vreg_w((a - 0xefff00) >> 1, v, mask);
		return;
	}
	if (a >= 0xf00000 && a < 0xf02000) {
		paletteram_w((a - 0xf00000) >> 1, v, mask);
		return;
	}
	if (a >= 0xf40000 && a < 0xf80000) {
		gfxram_w((a - 0xf40000) >> 1, v, mask);
		return;
	}
	if (a >= 0xff0000 && a < 0xffc000) {
		videoram_w((a - 0xff0000) >> 1, v, mask);
		return;
	}
	if (a >= 0x3f8000 && a < 0x400000) {
		mailbox_w_68k((a - 0x3f8000) >> 1, v, mask);
		return;
	}
	if (a >= 0xe00000 && a < 0xe00000 + s_eeprom_size * 2) {
		if (mask & 0x00ff)
			b.eeprom[(a - 0xe00000) >> 1] = (uint8_t)v;
		return;
	}
	if (s_zelos && (a & ~1u) == 0xd00000) {
		b.zelos_ctrl = (uint16_t)((b.zelos_ctrl & ~mask) | (v & mask));
		return;
	}
	// (custom_key_w, xday2's lamps / printer / RTC: nothing)
}

static void bus_w16(uint32_t a, uint16_t v) { bus_w16m(a, v, 0xffff); }
static void bus_w8(uint32_t a, uint8_t v)
{
	if (a & 1)
		bus_w16m(a & ~1u, v, 0x00ff);
	else
		bus_w16m(a, (uint16_t)(v << 8), 0xff00);
}

// ---- the MCU's bus (namcona1_mcu_map) ----
static uint8_t mcu_r(void *ctx, uint32_t a)
{
	(void)ctx;
	if (a >= 0x800 && a < 0x1000) {   // the mailbox (16-bit, little-endian)
		const uint16_t w = b.mailbox[((a - 0x800) >> 1) & 7];
		return (uint8_t)((a & 1) ? w >> 8 : w);
	}
	if (a >= 0x1000 && a < 0x2000) {   // the C219 (c219_le_r)
		snd_catchup();
		return c219_r(&s_c219, (int)(((a - 0x1000) & 0x1ff) ^ 1));
	}
	if (a >= 0x2000 && a < 0x3000)   // the shared work RAM's first page (byte-swapped: byte n is the 68000's)
		return rd8(b.workram, a - 0x2000);
	if (a >= 0x200000 && a < 0x280000)
		return rd8(b.workram, a - 0x200000);
	return 0;
}

static void mcu_w(void *ctx, uint32_t a, uint8_t v)
{
	(void)ctx;
	if (a >= 0x800 && a < 0x1000) {
		uint16_t *m = &b.mailbox[((a - 0x800) >> 1) & 7];
		*m = (a & 1) ? (uint16_t)((*m & 0x00ff) | v << 8) : (uint16_t)((*m & 0xff00) | v);
		return;
	}
	if (a >= 0x1000 && a < 0x2000) {
		snd_catchup();
		c219_w(&s_c219, (int)(((a - 0x1000) & 0x1ff) ^ 1), v);
		return;
	}
	if (a >= 0x2000 && a < 0x3000) {
		b.workram[(a - 0x2000) ^ M68K_BYTE_XOR] = v;
		return;
	}
	if (a >= 0x200000 && a < 0x280000)
		b.workram[(a - 0x200000) ^ M68K_BYTE_XOR] = v;
}

static uint8_t mcu_port_r(void *ctx, int port)
{
	(void)ctx;
	switch (port) {
	case 4: return b.port4;
	case 5: return b.port5;
	case 6: return 0;
	case 7:
		if (!(b.port6 & 0x80)) {
			const int p = s_port_mux[(b.port6 >> 5) & 3];
			return p >= 0 ? (uint8_t)rbx_port_read(p) : 0xff;
		}
		return 0xff;
	case 8: return b.port8;
	default: return 0xff;
	}
}

static void mcu_port_w(void *ctx, int port, uint8_t data, uint8_t mask)
{
	(void)ctx;
	(void)mask;
	switch (port) {
	case 4:   // bit 3 releases the 68000's reset
		if ((data & 0x08) && !(b.port4 & 0x08) && b.m68k_held) {
			b.m68k_held = 0;
			b.m68k_needreset = 1;   // (it starts on the next slice: its reset reads the vectors then)
		}
		b.port4 = data;
		break;
	case 5:   // bit 0 mirrors bit 1 (checked at CCD3 in the C69 BIOS)
		b.port5 = (uint8_t)((data & 0xfe) | ((data & 2) >> 1));
		break;
	case 6: b.port6 = data; break;
	case 8: b.port8 = data; break;   // (bit 5 toggles: a watchdog?)
	default: break;
	}
}

// each bit of player 3's inputs on one of the analog inputs
static uint16_t mcu_an_r(void *ctx, int ch)
{
	(void)ctx;
	static const uint8_t bit[8] = { 6, 5, 4, 0, 1, 2, 3, 7 };
	const uint32_t v = s_port_p3 >= 0 ? rbx_port_read(s_port_p3) : 0xff;
	return ((v >> bit[ch & 7]) & 1) ? 0xffff : 0x0000;
}

// ---- the screen updates: copies for the drawing ----
// a tile taken again (tile_update: tilemap_get_info / roz_get_info)
static void tile_update(int which, int i)
{
	b.tdirty[which][i] = 0;
	b.ndirty[which]--;
	const uint32_t vis0 = (b.tinfo[which][i] & 0x8000) || rd64(b.tmask[which][i]);
	const uint32_t bpp4 = (b.vreg[0xbc / 2] >> which) & 1;
	uint16_t data;
	uint32_t tile;
	if (which < 4) {
		data = rd16(b.videoram, (uint32_t)(which * 0x1000 + i) * 2);
		tile = data & 0xfff;
	} else {
		const int c = i & 0x3f, r = i >> 6;   // (each logical tile: 4 x 4 tiles)
		data = rd16(b.videoram, 0x8000 + (uint32_t)(((r >> 2) << 6) + (c >> 2)) * 2);
		tile = ((data & 0xfbfu) + (uint32_t)(c & 3) + (uint32_t)((r & 3) << 6)) & 0xfff;   // (bit 6 masked: Emeraldia Japan)
	}
	const uint32_t color = bpp4 ? (data & 0x7000u) >> 12 : 0;
	b.tinfo[which][i] = tile | color << 12 | (data & 0x8000u) | bpp4 << 16;
	if (!(data & 0x8000))
		memcpy(b.tmask[which][i], b.shaperam + tile * 8, 8);
	b.nvis[which] += (int32_t)((data & 0x8000) || rd64(b.tmask[which][i])) - (int32_t)vis0;
	s_tc_dirty[0][which] = s_tc_dirty[1][which] = 1;
}

// realize_all_dirty_tiles: the CG RAM changed since the layer's last drawing: all its tiles dirty
static void tiles_realize(int which)
{
	if (b.layer_seq[which] != b.gfx_seq) {
		b.layer_seq[which] = b.gfx_seq;
		layer_dirty(which);
	}
}

// MAME's drawing of the update's lines [y0, y1] in the window [x0, x1]: a tilemap updates its dirty tiles as they are
// drawn (draw_instance: the tiles under the drawn pixels of each line, the scroll carried down from line 0 as in
// draw_background; not on the pixel mode lines); the ROZ layer updates all (draw_roz: pixmap())
static void tiles_draw(int x0, int x1, int y0, int y1)
{
	if (x1 > 319)
		x1 = 319;   // (the screen bitmap: 320 wide)
	for (int which = 0; which < 4; which++) {
		const uint8_t *scroll = b.scroll + which * 0x400;
		const int xadjust = 0x3a - which * 2;
		if (!b.ndirty[which] && b.layer_seq[which] == b.gfx_seq)
			continue;   // (nothing to take again)
		int scrollx = xadjust, scrolly = 0, realized = 0;
		for (int line = 0; line <= y1; line++) {
			const int xdata = rd16(scroll, (uint32_t)line * 2), ydata = rd16(scroll, (uint32_t)line * 2 + 0x200);
			if (xdata) {
				if (xdata & 0x4000)
					scrollx = xadjust + xdata;
				else
					scrollx += xdata & 0x1ff;
			}
			if (ydata & 0x4000)
				scrolly = (ydata - line) & 0x1ff;
			if (line < y0 || xdata == 0xc001)
				continue;
			if (!realized) {
				tiles_realize(which);
				realized = 1;
			}
			const int row = ((line + scrolly) & 511) >> 3;
			uint8_t *dirty = b.tdirty[which] + row * 64;
			for (int x = x0; x <= x1;) {
				const int tx = (x + scrollx) & 511;
				if (dirty[tx >> 3])
					tile_update(which, row * 64 + (tx >> 3));
				x += 8 - (tx & 7);
			}
		}
	}
	tiles_realize(4);
	for (int i = 0; i < 4096 && b.ndirty[4]; i++)
		if (b.tdirty[4][i])
			tile_update(4, i);
}

static void capture(int y1)
{
	if (y1 > LINES - 1)
		y1 = LINES - 1;
	if (s_frame_nsnap >= 2 || y1 < s_last_line)
		return;
	if (s_frame_nsnap == 0)
		rbx_video_wait_idle();
	const int n = s_frame_nsnap;
	snap_t *sn = s_snap[n];
	sn->y0 = s_last_line;
	sn->y1 = y1;
	s_last_line = y1 + 1;
	memcpy(sn->vreg, b.vreg, sizeof(sn->vreg));
	memcpy(sn->videoram, b.videoram, sizeof(sn->videoram));
	memcpy(sn->scroll, b.scroll, sizeof(sn->scroll));
	memcpy(sn->spriteram, b.spriteram, sizeof(sn->spriteram));
	memcpy(sn->shaperam, b.shaperam, sizeof(sn->shaperam));
	for (int p = 0; p < 64; p++)
		if (s_cg_dirty[n][p]) {
			memcpy(sn->cgram + p * 0x1000, b.cgram + p * 0x1000, 0x1000);
			s_cg_dirty[n][p] = 0;
		}
	// screen_update: the palette's delayed updates once the graphics are enabled and the window is valid
	const int min_x = b.vreg[0x80 / 2] - 0x48, max_x = b.vreg[0x82 / 2] - 0x48 - 1;
	int min_y = b.vreg[0x84 / 2], max_y = b.vreg[0x86 / 2] - 1;
	if (min_y < sn->y0) min_y = sn->y0;
	if (max_y > sn->y1) max_y = sn->y1;
	if (b.vreg[0x8e / 2] && min_x >= 0 && max_x >= 0 && min_x <= max_x && min_y <= max_y) {
		if (b.palette_is_dirty) {
			for (int i = 0; i < 0x1000; i++)
				update_palette(i);
			b.palette_is_dirty = 0;
		}
		tiles_draw(min_x, max_x, min_y, max_y);
	}
	memcpy(sn->nvis, b.nvis, sizeof(sn->nvis));
	for (int l = 0; l < 5; l++)
		if (s_tc_dirty[n][l]) {
			memcpy(sn->tinfo[l], b.tinfo[l], sizeof(sn->tinfo[l]));
			memcpy(sn->tmask[l], b.tmask[l], sizeof(sn->tmask[l]));
			s_tc_dirty[n][l] = 0;
		}
	s_frame_nsnap = n + 1;
}

static void frame_capture(void)
{
	capture(LINES - 1);
	if (!s_frame_nsnap)   // (nothing left to draw: still a frame)
		rbx_video_wait_idle();
	if (s_pal_snap_dirty) {
		memcpy(s_pal_snap, b.pal_rgb, sizeof(s_pal_snap));
		s_pal_snap_dirty = 0;
		s_pal_valid = 0;
	}
	s_nsnap = s_frame_nsnap;
	s_frame_nsnap = 0;
	s_last_line = 0;
	rbx_video_frame_done();
}

// ---- the drawing (namcona1_v.cpp) ----
typedef struct {
	const snap_t *sn;
	int y0, y1;                    // screen lines of this update (visible part)
	int on;                        // graphics enabled, the window valid
	int dx0, dx1, dy0, dy1;        // the display window (clipped to the bitmap and the update)
	uint16_t bg;
	int lpri[5];
	uint16_t paloff[5];
	int16_t scx[4][LINES], scy[4][LINES];
	int32_t pix[4][LINES];         // the line's pixel mode source (word offset), -1 = tiles
	uint32_t roz_sx, roz_sy;
	int32_t incxx, incxy, incyx, incyy;
} seg_t;
static seg_t (*s_seg_hp)[2];
#define s_seg (*s_seg_hp)

static void seg_setup(seg_t *g, const snap_t *sn)
{
	const uint16_t *vr = sn->vreg;
	g->sn = sn;
	g->y0 = sn->y0 > VIS_Y0 ? sn->y0 : VIS_Y0;
	g->y1 = sn->y1;
	const int min_x = vr[0x80 / 2] - 0x48, max_x = vr[0x82 / 2] - 0x48 - 1;
	int min_y = vr[0x84 / 2], max_y = vr[0x86 / 2] - 1;
	if (min_y < sn->y0) min_y = sn->y0;
	if (max_y > sn->y1) max_y = sn->y1;
	g->on = vr[0x8e / 2] && min_x >= 0 && max_x >= 0 && min_x <= max_x && min_y <= max_y;
	g->dx0 = min_x;
	g->dx1 = max_x < VIS_W - 1 ? max_x : VIS_W - 1;
	g->dy0 = min_y;
	g->dy1 = max_y;
	g->bg = (uint16_t)((vr[0xba / 2] & 0xf) << 8);
	for (int i = 0; i < 4; i++) {
		g->lpri[i] = vr[0xa0 / 2 + i] & 7;
		g->paloff[i] = (uint16_t)((vr[0xb0 / 2 + i] & 0xf) << 8);
	}
	g->lpri[4] = vr[0xa0 / 2 + 5] & 7;
	g->paloff[4] = (uint16_t)((vr[0xba / 2] & 0xf) << 8);
	if (!g->on)
		return;
	// the line scroll of the tilemaps (from line 0: the values carry down the lines)
	for (int which = 0; which < 4; which++) {
		const uint8_t *scroll = sn->scroll + which * 0x400;
		const int xadjust = 0x3a - which * 2;
		int scrollx = xadjust, scrolly = 0;
		for (int line = 0; line < LINES; line++) {
			const int xdata = rd16(scroll, (uint32_t)line * 2);
			const int ydata = rd16(scroll, (uint32_t)line * 2 + 0x200);
			if (xdata) {
				if (xdata & 0x4000)   // (resets the current x scroll: Knuckle Heads)
					scrollx = xadjust + xdata;
				else
					scrollx += xdata & 0x1ff;
			}
			if (ydata & 0x4000)   // line select
				scrolly = (ydata - line) & 0x1ff;
			g->scx[which][line] = (int16_t)(scrollx & 0x1ff);
			g->scy[which][line] = (int16_t)scrolly;
			g->pix[which][line] = xdata == 0xc001 ? ydata + 25 : -1;   // (the pixel line mode: Numan Athletics)
		}
	}
	// the ROZ layer
	g->incxx = (int16_t)vr[0xc0 / 2] * 256;
	g->incxy = (int16_t)vr[0xc2 / 2] * 256;
	g->incyx = (int16_t)vr[0xc4 / 2] * 256;
	g->incyy = (int16_t)vr[0xc6 / 2] * 256;
	const int16_t xoffset = (int16_t)vr[0xc8 / 2], yoffset = (int16_t)vr[0xca / 2];
	const int dx = 46, dy = -8;   // (horizontal / vertical adjust)
	g->roz_sx = (uint32_t)(xoffset * 4096 + g->incxx * dx + g->incyx * dy);
	g->roz_sy = (uint32_t)(yoffset * 4096 + g->incxy * dx + g->incyy * dy);
}

static void draw_tile_line(const seg_t *g, int which, int y, uint16_t *d, uint8_t *pri, int p)
{
	const snap_t *sn = g->sn;
	const int x0 = g->dx0, x1 = g->dx1;
	const int32_t src = g->pix[which][y];
	if (src >= 0) {   // draw_pixel_line (pens from 0)
		for (int x = 0; x < VIS_W; x += 2) {
			const uint16_t data = rd16(sn->videoram, ((uint32_t)(src + x / 2) * 2) % sizeof(sn->videoram));
			pri[x] = pri[x + 1] = 0xff;
			if (x >= x0 && x <= x1)
				d[x] = data >> 8;
			if (x + 1 >= x0 && x + 1 <= x1)
				d[x + 1] = data & 0xff;
		}
		return;
	}
	if (!sn->nvis[which])
		return;   // (no cached tile with a pixel)
	const int ty = (y + g->scy[which][y]) & 511;
	const int row = (ty >> 3) * 64;
	const int fy = ty & 7;
	const uint16_t paloff = g->paloff[which];
	int tx = (x0 + g->scx[which][y]) & 511;
	for (int x = x0; x <= x1;) {
		const int idx = row + (tx >> 3);
		const uint32_t info = sn->tinfo[which][idx];
		const uint32_t tile = info & 0xfff;
		const int bpp4 = (info >> 16) & 1;
		const uint8_t *pix = sn->cgram + tile * 64 + fy * 8;
		const uint8_t m = (info & 0x8000) ? 0xff : sn->tmask[which][idx][fy];
		const uint16_t base = (uint16_t)(paloff + ((info >> 12) & 7) * 16);
		const int fx0 = tx & 7;
		int n = 8 - fx0;
		if (n > x1 - x + 1)
			n = x1 - x + 1;
		if (m == 0xff) {   // (the whole tile row drawn)
			const uint8_t cm = bpp4 ? 0x0f : 0xff;
			for (int k = 0; k < n; k++) {
				d[x + k] = (uint16_t)(base + (pix[(fx0 + k) ^ M68K_BYTE_XOR] & cm));
				pri[x + k] = (uint8_t)p;
			}
		} else if (m) {
			for (int k = 0; k < n; k++)
				if (m & (0x80 >> (fx0 + k))) {
					const uint8_t c = pix[(fx0 + k) ^ M68K_BYTE_XOR];
					d[x + k] = (uint16_t)(base + (bpp4 ? (c & 0xf) : c));
					pri[x + k] = (uint8_t)p;
				}
		}
		x += n;
		tx = (tx + n) & 511;
	}
}

static void draw_roz_line(const seg_t *g, int y, uint16_t *d, uint8_t *pri, int p)
{
	const snap_t *sn = g->sn;
	const int x0 = g->dx0, x1 = g->dx1;
	uint32_t cx = g->roz_sx + (uint32_t)x0 * (uint32_t)g->incxx + (uint32_t)y * (uint32_t)g->incyx;
	uint32_t cy = g->roz_sy + (uint32_t)x0 * (uint32_t)g->incxy + (uint32_t)y * (uint32_t)g->incyy;
	const uint16_t paloff = g->paloff[4];
	int last = -1;   // (the tile row of the pixel before: its mask, pixels, pens)
	uint8_t m = 0, cm = 0;
	const uint8_t *pix = NULL;
	uint16_t base = 0;
	for (int x = x0; x <= x1; x++, cx += (uint32_t)g->incxx, cy += (uint32_t)g->incxy) {
		if (cx >= (512u << 16) || cy >= (512u << 16))
			continue;
		const int xpos = (int)(cx >> 16), ypos = (int)(cy >> 16);
		const int key = ypos << 6 | xpos >> 3;   // (the tile and its line)
		if (key != last) {
			last = key;
			const int idx = (ypos >> 3) * 64 + (xpos >> 3), fy = ypos & 7;
			const uint32_t info = sn->tinfo[4][idx];
			m = (info & 0x8000) ? 0xff : sn->tmask[4][idx][fy];
			pix = sn->cgram + (info & 0xfff) * 64 + (uint32_t)fy * 8;
			cm = ((info >> 16) & 1) ? 0x0f : 0xff;
			base = (uint16_t)(paloff + ((info >> 12) & 7) * 16);
		}
		const int fx = xpos & 7;
		if (!(m & (0x80 >> fx)))
			continue;
		d[x] = (uint16_t)(base + (pix[fx ^ M68K_BYTE_XOR] & cm));
		pri[x] = (uint8_t)p;
	}
}

// pdraw_tile within the lines [ly0, ly1] (screen lines)
static void pdraw_tile(const seg_t *g, int ly0, int ly1, uint32_t code, uint32_t color, int sx, int sy, int flipx,
                       int flipy, uint8_t priority, int shadow, int opaque, int bpp4)
{
	const snap_t *sn = g->sn;
	code &= 0xfff;
	const uint32_t pal_base = bpp4 ? (color % 512) * 16 : (color % 32) * 256;
	int ex = sx + 8, ey = sy + 8;
	int x_index_base = flipx ? 7 : 0, dx = flipx ? -1 : 1;
	int y_index = flipy ? 7 : 0, dy = flipy ? -1 : 1;
	if (sx < g->dx0) {
		x_index_base += (g->dx0 - sx) * dx;
		sx = g->dx0;
	}
	if (sy < ly0) {
		y_index += (ly0 - sy) * dy;
		sy = ly0;
	}
	if (ex > g->dx1 + 1)
		ex = g->dx1 + 1;
	if (ey > ly1 + 1)
		ey = ly1 + 1;
	if (ex <= sx)
		return;
	const int shade = shadow && ((!bpp4 && color == 0x0f) || (bpp4 && color == 0xff));
	for (int y = sy; y < ey; y++, y_index += dy) {
		const uint8_t *src = sn->cgram + code * 64 + (uint32_t)y_index * 8;
		const uint8_t mrow = sn->shaperam[code * 8 + (uint32_t)y_index];
		uint16_t *dest = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		uint8_t *pri = s_pri + (y - VIS_Y0) * VIS_W;
		int x_index = x_index_base;
		for (int x = sx; x < ex; x++, x_index += dx) {
			if (opaque) {
				if (pri[x] <= priority) {
					const uint8_t c = src[x_index ^ M68K_BYTE_XOR];
					dest[x] = (uint16_t)(pal_base + (bpp4 ? (c & 0xf) : c));
				}
				pri[x] = 0xff;
			} else if (mrow & (0x80 >> x_index)) {
				if (pri[x] <= priority) {
					const uint8_t c = src[x_index ^ M68K_BYTE_XOR];
					const uint16_t pen = (uint16_t)(pal_base + (bpp4 ? (c & 0xf) : c));
					if (shadow) {
						if (shade) {   // (the shadow table: a pen's shadow)
							if (dest[x] < 0x2000)
								dest[x] = (uint16_t)(dest[x] + 0x2000);
						} else
							dest[x] = (uint16_t)(pen + 0x1000);
					} else
						dest[x] = pen;
				}
				pri[x] = 0xff;
			}
		}
	}
}

static void draw_sprites(const seg_t *g, int ly0, int ly1)
{
	const uint8_t *source = g->sn->spriteram;
	if (g->sn->vreg[0x22 / 2] & 1)
		source += 0x800;   // (the alternate sprite RAM bank)
	for (int which = 0; which < 0x100; which++, source += 8) {
		const uint16_t ypos = rd16(source, 0);    // FHHH---Y YYYYYYYY  flipy, height, ypos
		const uint16_t tile = rd16(source, 2);    // O???TTTT TTTTTTTT  opaque tile
		const uint16_t color = rd16(source, 4);   // FSWWOOOO CCCCBPPP  flipx, shadow, width, 4bpp color offset, color, 4bpp, pri
		const uint16_t xpos = rd16(source, 6);    // -------X XXXXXXXX
		const uint8_t priority = color & 7;
		const int width = ((color >> 12) & 3) + 1, height = ((ypos >> 12) & 7) + 1;
		const int flipy = (ypos & 0x8000) != 0, flipx = (color & 0x8000) != 0;
		int bpp4, palbase;
		if (color & 8) {
			palbase = (color & 0xf0) | ((color & 0xf00) >> 8);
			bpp4 = 1;
		} else {
			palbase = (color & 0xf0) >> 4;
			bpp4 = 0;
		}
		for (int row = 0; row < height; row++) {
			int sy = (ypos & 0x1ff) - 30 + 32;
			sy += (flipy ? height - 1 - row : row) << 3;
			sy = ((sy + 8) & 0x1ff) - 8;
			if (sy > ly1 || sy + 8 <= ly0)
				continue;
			for (int col = 0; col < width; col++) {
				int sx = (xpos & 0x1ff) - 10;
				sx += (flipx ? width - 1 - col : col) << 3;
				sx = ((sx + 16) & 0x1ff) - 8;
				pdraw_tile(g, ly0, ly1, (uint32_t)((tile & 0xfff) + (row << 6) + col), (uint32_t)palbase, sx, sy, flipx,
				           flipy, priority, (color & 0x4000) != 0, (tile & 0x8000) != 0, bpp4);
			}
		}
	}
}

#define BANDS 8   // (bands of lines drawn apart: by the render thread and its helpers, decoband.h)

static void run_band(int band)
{
	const int r0 = VIS_H * band / BANDS, r1 = VIS_H * (band + 1) / BANDS;   // (bitmap rows)
	for (int si = 0; si < s_nsnap; si++) {
		const seg_t *g = &s_seg[si];
		int y0 = g->y0 > r0 + VIS_Y0 ? g->y0 : r0 + VIS_Y0, y1 = g->y1 < r1 + VIS_Y0 - 1 ? g->y1 : r1 + VIS_Y0 - 1;
		if (y0 > y1)
			continue;
		for (int y = y0; y <= y1; y++) {
			uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
			uint8_t *pri = s_pri + (y - VIS_Y0) * VIS_W;
			for (int x = 0; x < VIS_W; x++)
				d[x] = g->bg;
			memset(pri, 0, VIS_W);
			if (!g->on || y < g->dy0 || y > g->dy1)
				continue;
			for (int p = 0; p < 8; p++)
				for (int which = 4; which >= 0; which--)
					if (g->lpri[which] == p) {
						if (which == 4) {
							if (g->sn->nvis[4])
								draw_roz_line(g, y, d, pri, p);
						}
						else
							draw_tile_line(g, which, y, d, pri, p);
					}
		}
		if (!g->on)
			continue;
		if (y0 < g->dy0) y0 = g->dy0;
		if (y1 > g->dy1) y1 = g->dy1;
		if (y0 <= y1)
			draw_sprites(g, y0, y1);
	}
}

static decoband_t s_bands = { 0, 0, 0, 0, BANDS, run_band };

static void na1_render(void)
{
	if (!s_pal_valid) {   // (the pens changed since the last drawing)
		uint32_t *pal = rbx_video.palette;
		for (int i = 0; i < 0x2000; i++) {
			const uint32_t c = s_pal_snap[i];
			if (!s_pal_full && pal[i] == c)
				continue;
			pal[i] = c;
			// (the shadows: MAME's default shadow factor 0.6)
			const uint32_t r = ((c >> 16) & 0xff) * 6 / 10, gg = ((c >> 8) & 0xff) * 6 / 10, bb = (c & 0xff) * 6 / 10;
			pal[i + 0x2000] = r << 16 | gg << 8 | bb;
		}
		s_pal_valid = 1;
		s_pal_full = 0;
	}
	for (int i = 0; i < s_nsnap; i++)
		seg_setup(&s_seg[i], s_snap[i]);
	if (!s_nsnap)
		return;
	decoband_run(&s_bands);
}

static int na1_render_help(int helper) { return decoband_help(&s_bands, helper); }

// ---- the frame ----
static void scanline_interrupt(int scanline)
{
	const uint16_t enabled = b.enable_interrupts ? (uint16_t)~b.vreg[0x1a / 2] : 0;
	if (scanline == 224) {   // vblank
		m37710_set_input(&s_mcu, M37710_LINE_IRQ1, 1);
		wr16(b.workram, 0xf60, 0, 0xffff);   // (simulate_mcu: "mcu ready")
		if (enabled & 8)
			m68k_bus_hold_irq(4);
	}
	// the position IRQ (Emeraldia's "how to play" dolphin)
	const int posirq_scanline = b.vreg[0x8a / 2] & 0xff;
	if (scanline == posirq_scanline && (enabled & 4)) {
		if (posirq_scanline)
			capture(posirq_scanline);   // (update_partial)
		m68k_bus_hold_irq(3);
	}
}

static int64_t s_mcu_t0;   // (the MCU's cycles at the 68000's cycle 0)

// ---- the 68000's wait loops (NA1_NOIDLE=1: off) ----
// The 68000's slice runs it alone (the MCU after it, the interrupt lines raised between slices), so at the top of a
// loop that reads and tests only, its exit condition false and no interrupt it would take, every turn to the slice's
// end is the same: one turn is run (its clocks measured), the whole turns that fit before the slice's end are
// skipped, the rest run as usual: exact. The forms (the games' main loops waiting for the vblank IRQ):
//   [MOVE.W key,D0 | SWAP D0]...  TST.B / TST.W abs  BEQ / BNE (to the top)
// the key chip's reads made for the turns skipped (its random words), D0 as they leave it.
static int s_noidle;

static void idle68(void)
{
	const uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC) & 0xffffff;
	if (pc < 0xc00000 || pc - 0xc00000 + 64 > s_prg_size)
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
	const uint32_t o0 = pc - 0xc00000;
	uint8_t ops[4];   // (key read: its word offset; 0x80: SWAP D0)
	int nops = 0;
	uint32_t o = o0;
	for (;;) {
		const uint16_t w = rd16(s_prg, o);
		if (w == 0x3039 && rd16(s_prg, o + 2) == 0x00e4 && (rd16(s_prg, o + 4) & 0xfff1) == 0) {
			if (nops == 4)
				return;
			ops[nops++] = (uint8_t)(rd16(s_prg, o + 4) >> 1);
			o += 6;
		} else if (w == 0x4840) {
			if (nops == 4)
				return;
			ops[nops++] = 0x80;
			o += 2;
		} else
			break;
	}
	const uint16_t tw = rd16(s_prg, o);
	uint32_t a;
	if (tw == 0x4a38 || tw == 0x4a78) {
		a = (uint32_t)(int32_t)(int16_t)rd16(s_prg, o + 2) & 0xffffff;
		o += 4;
	} else if (tw == 0x4a39 || tw == 0x4a79) {
		a = ((uint32_t)rd16(s_prg, o + 2) << 16 | rd16(s_prg, o + 4)) & 0xffffff;
		o += 6;
	} else
		return;
	const uint16_t bw = rd16(s_prg, o);
	const int8_t d8 = (int8_t)(bw & 0xff);
	if (((bw & 0xff00) != 0x6700 && (bw & 0xff00) != 0x6600) || !d8 || o + 2 + (uint32_t)(int32_t)d8 != o0)
		return;
	if (!m68k_bus.rd[a >> M68K_PAGE_BITS])
		return;   // (plain memory only)
	const uint32_t v = tw == 0x4a38 || tw == 0x4a39 ? m68k_read_memory_8(a) : m68k_read_memory_16(a & ~1u);
	if ((bw & 0xff00) == 0x6700 ? v != 0 : v == 0)
		return;   // (it leaves)
	// one turn, its clocks measured
	int t = 0;
	for (int k = 0; k < 8; k++) {
		t += m68k_execute(1);
		if ((m68k_get_reg(NULL, M68K_REG_PC) & 0xffffff) == pc)
			break;
	}
	b.t68 += t;
	if ((m68k_get_reg(NULL, M68K_REG_PC) & 0xffffff) != pc || t <= 0 || b.t68 >= b.t68_target)
		return;
	const int64_t n = (b.t68_target - b.t68 - 1) / t;
	if (n <= 0)
		return;
	if (nops) {
		uint32_t d0 = m68k_get_reg(NULL, M68K_REG_D0);
		for (int64_t i = 0; i < n; i++)
			for (int j = 0; j < nops; j++)
				d0 = ops[j] == 0x80 ? d0 << 16 | d0 >> 16 : (d0 & 0xffff0000u) | custom_key_r(ops[j]);
		m68k_set_reg(M68K_REG_D0, d0);
	}
	b.t68 += n * t;
}

// run the MCU to the 68000's time t68 (its clock is half the 68000's)

static void mcu_run_until(int64_t t68)
{
	const int64_t target = s_mcu_t0 + t68 / 2;
	const int64_t n = target - s_mcu.cycles - s_mcu.icount;   // (icount: the overshoot carried, already in cycles)
	if (n > 0)
		m37710_execute(&s_mcu, (int)n);
}

static void na1_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	b.mcu_frame0 = s_mcu.cycles;
	b.snd_done = 0;
	for (int line = 0; line < LINES; line++) {
		scanline_interrupt(line);
		b.t68_target += rbx_line_cycles(&b.m68k_frac, M68K_CLOCK, FRAME_MHZ, LINES);
		while (b.t68 < b.t68_target) {
			if (b.m68k_held) {
				b.t68 = b.t68_target;
				break;
			}
			if (b.m68k_needreset) {
				b.m68k_needreset = 0;
				m68k_pulse_reset();
			}
			if (!s_noidle) {
				idle68();
				if (b.t68 >= b.t68_target)
					break;
			}
			s_sync_irq0 = 0;
			b.t68 += m68k_execute((int)(b.t68_target - b.t68));
			if (s_sync_irq0) {   // (the mailbox's IRQ: the MCU to this time first)
				s_sync_irq0 = 0;
				mcu_run_until(b.t68);
				m37710_set_input(&s_mcu, M37710_LINE_IRQ0, 1);
			}
		}
		mcu_run_until(b.t68_target);
		snd_to((int)((uint64_t)(line + 1) * SND_FRAME / LINES));
	}
	snd_to(SND_FRAME);
	frame_capture();
	for (int i = 0; i < SND_FRAME; i++) {
		rbx_audio_put_stereo(s_mix_l[i], s_mix_r[i]);
		s_mix_l[i] = s_mix_r[i] = 0;
	}
}

// ---- init / reset ----
static void na1_reset(void)
{
	memset(b.vreg, 0, sizeof(b.vreg));
	memset(b.mailbox, 0, sizeof(b.mailbox));
	b.enable_interrupts = 0;
	b.port4 = 0;
	b.port5 = 1;
	b.port6 = b.port8 = 0;
	b.count = 0;
	b.keyval = 0;
	b.zelos_ctrl = 0;
	b.rng = 0x9d14abd7;
	b.m68k_held = 1;   // (machine_reset: the MCU boots the 68000)
	b.m68k_needreset = 0;
	b.t68 = b.t68_target = 0;
	b.m68k_frac = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	m37710_reset(&s_mcu);
	s_mcu_t0 = s_mcu.cycles;
	c140_reset(&s_c219);
	s_sync_irq0 = 0;
	s_frame_nsnap = 0;
	s_last_line = 0;
	memset(s_cg_dirty, 1, sizeof(s_cg_dirty));
	memset(s_tc_dirty, 1, sizeof(s_tc_dirty));
	for (int l = 0; l < 5; l++)
		layer_dirty(l);
	s_pal_snap_dirty = 1;
}

static const struct { const char *init; int type; } s_types[] = {
	{ "bkrtmaq", GT_BKRTMAQ }, { "cgangpzl", GT_CGANGPZL }, { "emeraldj", GT_EMERALDA }, { "exvania", GT_EXVANIA },
	{ "fa", GT_FA }, { "swcourt", GT_SWCOURT }, { "swcourtb", GT_SWCOURTB }, { "tinklpit", GT_TINKLPIT },
	{ "emeralda", GT_EMERALDA }, { "knckhead", GT_KNCKHEAD }, { "numanath", GT_NUMANATH }, { "quiztou", GT_QUIZTOU },
	{ "xday2", GT_XDAY2 }, { "zelos", GT_NONE },
};

static void na1_exit(void);

static int na1_init(const rbx_game *g)
{
	if (!s_pal_snap_hp && !(s_pal_snap_hp = calloc(1, sizeof(*s_pal_snap_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_seg_hp && !(s_seg_hp = calloc(1, sizeof(*s_seg_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_bp && !(s_bp = (struct na1_state *)calloc(1, sizeof(*s_bp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_prg = rbx_region("maincpu", &s_prg_size);
	s_mask = rbx_region("maskrom", &s_mask_size);
	s_mcurom = rbx_region("mcu", &s_mcurom_size);
	if (!s_prg || !s_mask || !s_mcurom || s_mcurom_size < 0x4000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_prg, s_prg_size);
	m68k_bus_swap16(s_mask, s_mask_size);
	const char *init = strchr(g->machine, ':');
	init = init ? init + 1 : "";
	s_gametype = GT_NONE;
	for (size_t i = 0; i < sizeof(s_types) / sizeof(s_types[0]); i++)
		if (!strcmp(init, s_types[i].init))
			s_gametype = s_types[i].type;
	s_xday2 = !strncmp(g->machine, "xday2:", 6);
	s_zelos = !strncmp(g->machine, "zelos:", 6);
	s_eeprom_size = s_xday2 ? 0x2000 : 0x800;   // (EEPROM_2864 / EEPROM_2816)
	static const char *const mux[4] = { "P4", "DSW", "P1", "P2" };
	for (int i = 0; i < 4; i++)
		s_port_mux[i] = rbx_port_find(mux[i]);
	s_port_p3 = rbx_port_find("P3");

	memset(&b, 0, sizeof(b));
	memset(b.eeprom, 0xff, sizeof(b.eeprom));
	{
		uint32_t esz = 0;
		const uint8_t *e = rbx_region("eeprom", &esz);
		if (e)
			memcpy(b.eeprom, e, esz < s_eeprom_size ? esz : s_eeprom_size);
	}
	for (int i = 0; i < 2; i++)
		if (!s_snap[i] && !(s_snap[i] = (snap_t *)calloc(1, sizeof(snap_t)))) {
			na1_exit();
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
	s_pri = (uint8_t *)calloc(1, VIS_W * VIS_H);
	if (!s_pri || rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		na1_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	memset(s_pal_snap, 0, sizeof(s_pal_snap));
	// the 68000 (namcona1_main_map)
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);   // (MAME's DIVS / DIVU / MULS / MULU / ADDQ.W An times: the self test's timing loops)
	m68k_bus_reset_map();
	m68k_bus_map(0x000000, 0x07ffff, b.workram, sizeof(b.workram), 1, 1);
	m68k_bus_map(0x400000, 0x400000 + s_mask_size - 1, s_mask, s_mask_size, 1, 0);
	m68k_bus_map(0xc00000, 0xc00000 + s_prg_size - 1, s_prg, s_prg_size, 1, 0);
	m68k_bus_map(0xefff00, 0xefffff, b.vreg, sizeof(b.vreg), 1, 0);
	m68k_bus_map(0xf00000, 0xf01fff, b.paletteram, sizeof(b.paletteram), 1, 0);
	m68k_bus_map(0xff0000, 0xffbfff, b.videoram, sizeof(b.videoram), 1, 0);   // (writes: videoram_w)
	m68k_bus_map(0xffc000, 0xffdfff, b.ram_c000, sizeof(b.ram_c000), 1, 1);
	m68k_bus_map(0xffe000, 0xffefff, b.scroll, sizeof(b.scroll), 1, 1);
	m68k_bus_map(0xfff000, 0xffffff, b.spriteram, sizeof(b.spriteram), 1, 1);
	m68k_bus.read8 = bus_r8;
	m68k_bus.read16 = bus_r16;
	m68k_bus.write8 = bus_w8;
	m68k_bus.write16 = bus_w16;
	// the MCU (C69 / C70: an M37702M2 with its BIOS)
	m37710_init(&s_mcu);
	s_mcu.rom = s_mcurom;
	for (int p = 3; p < 0xb; p++)
		s_mcu.rpage[p] = s_mcu.wpage[p] = b.mcuram + (p - 3) * 0x1000;
	s_mcu.read = mcu_r;
	s_mcu.write = mcu_w;
	s_mcu.port_read = mcu_port_r;
	s_mcu.port_write = mcu_port_w;
	s_mcu.analog_read = mcu_an_r;
	m37710_set_clock(&s_mcu, MCU_CLOCK);
	s_noidle = getenv("NA1_NOIDLE") != NULL;
	s_mcu.idle_skip = !s_noidle;   // (the BIOS's wait for its interrupts)
	// the C219 (its samples in the work RAM)
	c219_init(&s_c219, SND_RATE, b.workram, sizeof(b.workram), 256);
	s_pal_valid = 0;
	s_pal_full = 1;
	s_nsnap = 0;
	na1_reset();
	return 0;
}

static void na1_exit(void)
{
	m68k_set_mame_timing(0);
	for (int i = 0; i < 2; i++) {
		free(s_snap[i]);
		s_snap[i] = NULL;
	}
	free(s_pri);
	s_pri = NULL;
	s_nsnap = 0;
	free(s_bp);   // (last: the render thread and the saves are done with it)
	s_bp = NULL;
	free(s_pal_snap_hp);
	s_pal_snap_hp = NULL;
	free(s_seg_hp);
	s_seg_hp = NULL;
}

static int na1_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	int rc = 0;
	if ((uint32_t)m68k_context_size() > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	rc |= rbx_state_io(ctx, "na1_cpu", cpu, (uint32_t)m68k_context_size());
	rc |= rbx_state_io(ctx, "na1_bus", &m68k_bus.irq_pending, sizeof(uint32_t) * 2);
	rc |= rbx_state_io(ctx, "na1", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= rbx_state_io(ctx, "na1_mcu", &s_mcu, M37710_STATE_SIZE);
	rc |= rbx_state_io(ctx, "na1_mcut0", &s_mcu_t0, sizeof(s_mcu_t0));
	rc |= rbx_state_io(ctx, "na1_c219", &s_c219, (uint32_t)C140_STATE_SIZE);
	rc |= rbx_state_io(ctx, "na1_lfsr", &s_c219.lfsr, sizeof(s_c219.lfsr));
	if (!rc && load) {
		m68k_set_context(cpu);
		m37710_post_load(&s_mcu);
		memset(s_cg_dirty, 1, sizeof(s_cg_dirty));
		memset(s_tc_dirty, 1, sizeof(s_tc_dirty));
		s_pal_snap_dirty = 1;
	}
	return rc ? -1 : 0;
}

static int na1_save(void *ctx) { return na1_state(ctx, 0); }
static int na1_load(void *ctx) { return na1_state(ctx, 1); }
static int na1_supported(const char *machine)
{
	return !strncmp(machine, "namcona1:", 9) || !strncmp(machine, "namcona2:", 9) || !strncmp(machine, "xday2:", 6) ||
	       !strncmp(machine, "zelos:", 6);
}

const rbx_driver rbx_drv_namcona1 = { "Namco NA-1/NA-2", na1_init, na1_exit, na1_reset, na1_frame, na1_save, na1_load,
                                      na1_supported, na1_render, na1_render_help };
