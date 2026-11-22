/*  NMK16 boards. Ported from MAME src/mame/nmk/nmk16.cpp, nmk16_v.cpp, nmk16spr.cpp, nmk_irq.cpp and
    src/devices/machine/nmk112.cpp (BSD-3-Clause, copyright-holders: Mirko Buffoni, Richard Bush, Nicola Salmoria,
    Bryan McPhail, David Haywood, R. Belmont, Alex Marshall, Angelo Salese, Luca Elia).
    A 68000, the interrupts and the sprite DMA at the lines a timing PROM gives (NMK IRQ generator), a 16x16 tile
    layer of 16x16 pages (bank of 0x2000 tiles), an 8x8 text layer, 16x16 sprites (made of 8x8 quarters) double
    buffered, a palette RRRRGGGGBBBBRGBx. Sound: a Z80 with a YM2203 and two OKI6295 behind a NMK112 (Macross II,
    Thunder Dragon 2), or the two OKIs alone (Bombjack Twin, Nouryoku). 384x224 of 512x278 at 8 MHz (56.2 Hz).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/okim6295.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define LINES 278
#define FRAME_MHZ 56205          // 8 MHz / (512 x 278)
#define VIS_Y0 16
#define VIS_WMAX 384
static int s_visw = 384, s_visx0 = 28;   // (hi-res: 28..411; low-res: 92..347; lines 16..239)
#define VIS_W s_visw
#define VIS_X0 s_visx0
#define VIS_H 224
#define YM_CLOCK 1500000
#define OKI_CLOCK 4000000
#define PAL_BLACK 0x400

enum { K_MACROSS2, K_TDRAGON2, K_TDRAGON3H, K_BJTWIN, K_NMK004, K_RAPHERO };

// the NMK004 boards' maps (low-res, the macross video): the 68000's addresses of each part
typedef struct {
	const char *machine;
	uint32_t io, io_mirror, pal, pal_size, scroll, bg, bg_size, tx, tx_size, ram, ram_mirror;
	int scroll_kind;   // 0 scroll_w<0>, 1 mustang_scroll_w, 2 vandyke_scroll_w
	int dsw1_at4, flip_inv;
	uint32_t cpu_clock, nmk004_clock, oki_clock;
	int prot;   // the protection MCU: 1 NMK-110 (TMP91640), 2 NMK-215 (TMP90840) with the NMK214s
	int video;  // 0 macross; 1 strahl (bg1 in RAM at bg1); 2 bioship (bg0 the ROM map, bg1 in RAM at bg)
	uint32_t bg1, scroll1;
} layout_t;
static const layout_t s_layouts[] = {
	{ "tdragon", 0x0c0000, 0x20000, 0x0c8000, 0x800, 0x0c4000, 0x0cc000, 0x4000, 0x0d0000, 0x800, 0x080000, 0x30000, 0, 0, 0, 8000000, 8000000, 4000000 },
	{ "hachamf", 0x080000, 0, 0x088000, 0x800, 0x08c000, 0x090000, 0x4000, 0x09c000, 0x800, 0x0f0000, 0, 0, 0, 0, 10000000, 8000000, 4000000 },
	{ "blkheart", 0x080000, 0, 0x088000, 0x800, 0x08c000, 0x090000, 0x4000, 0x09c000, 0x800, 0x0f0000, 0, 0, 0, 0, 8000000, 8000000, 4000000 },
	{ "acrobatm", 0x0c0000, 0, 0x0c4000, 0x600, 0x0c8000, 0x0cc000, 0x4000, 0x0d4000, 0x800, 0x080000, 0, 0, 0, 0, 10000000, 8000000, 4000000 },
	{ "mustang", 0x080000, 0, 0x088000, 0x800, 0x08c000, 0x090000, 0x4000, 0x09c000, 0x800, 0x0f0000, 0, 1, 1, 0, 8000000, 8000000, 4000000 },
	{ "vandyke", 0x080000, 0, 0x088000, 0x800, 0x08c000, 0x090000, 0x4000, 0x09d000, 0x800, 0x0f0000, 0, 2, 0, 1, 10000000, 8000000, 4000000 },
	{ "tdragon_prot", 0x0c0000, 0x20000, 0x0c8000, 0x800, 0x0c4000, 0x0cc000, 0x4000, 0x0d0000, 0x800, 0x080000, 0x30000, 0, 0, 0, 8000000, 8000000, 4000000, 1 },
	{ "hachamf_prot", 0x080000, 0, 0x088000, 0x800, 0x08c000, 0x090000, 0x4000, 0x09c000, 0x800, 0x0f0000, 0, 0, 0, 0, 10000000, 8000000, 4000000, 1 },
	{ "macross_prot", 0x080000, 0, 0x088000, 0x800, 0x08c000, 0x090000, 0x4000, 0x09c000, 0x800, 0x0f0000, 0, 0, 0, 0, 10000000, 8000000, 4000000, 2 },
	{ "strahl", 0x080000, 0, 0x08c000, 0x800, 0x084000, 0x090000, 0x4000, 0x09c000, 0x800, 0x0f0000, 0, 0, 0, 0, 12000000, 8000000, 4000000, 0, 1, 0x094000, 0x088000 },
	{ "bioship", 0x080000, 0, 0x088000, 0x800, 0x08c010, 0x090000, 0x4000, 0x09c000, 0x800, 0x0f0000, 0, 3, 0, 0, 10000000, 8000000, 4000000, 0, 2, 0, 0x08c000 },
	{ NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};
static const layout_t *s_lay;

// ---- ROMs ----
static uint8_t *s_rom, *s_zrom, *s_fg, *s_bg, *s_spr, *s_oki[2];
static uint32_t s_rom_size, s_zrom_size, s_fg_size, s_bg_size, s_spr_size, s_oki_size[2];
static const uint8_t *s_vprom;
static uint32_t s_vprom_size;
static int s_kind, s_hacky_irq;
static int s_port_in0, s_port_in1, s_port_dsw1, s_port_dsw2;

// ---- the state (saved) ----
static struct nmk16_state {   // (on the heap while the game runs, ADR-040)
	uint16_t mainram[0x8000];      // 0x1f0000 (or 0x0f0000) 64 KB
	uint16_t paletteram[0x400];
	uint16_t bgvram[0x8000];       // 64 KB (bjtwin: the first 4 KB)
	uint16_t txvram[0x800];
	uint16_t spr_old[0x800], spr_old2[0x800];   // sprite DMA's two buffers
	uint8_t scroll[4];
	uint8_t bgbank, tilerambank, flip, vtiming_val;
	uint8_t soundlatch, soundlatch2, zbank, zreset;
	uint8_t okibank[2][4];
	int16_t bjtwin_scrolly;
	uint8_t to_nmk004, to_main;
	uint16_t vscroll[4], mustang_xscroll;
	uint8_t cpu_reset_held, nmk004_okibank[2], pad2;
	uint8_t cpu_halt, bus_status, init_data_214, init_clock_214, unscramble, pad3[3];
	uint8_t nmk214_cfg[2], nmk214_init[2];
	uint16_t scrollram[0x100], scrollramy[0x100];
	uint16_t bgvram1[0x2000];
	uint8_t scroll1[4], bioship_bank, pad4[3];
	uint64_t cpu_frac, z80_frac, ym_frac, oki_frac, mcu_frac;
	int32_t oki_cur;
	uint32_t frame;
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)

#include "../../cpu/tlcs90/tlcs90.h"
static tlcs90 s_n4;
static const uint8_t *s_n4rom, *s_n4ext;
static uint32_t s_n4ext_size;
static uint8_t s_n4ram[0x800];
static z80 s_z80;
static int s_zirq;
static uint8_t s_zram[0x2000];
static const uint8_t *s_zmap[256];
static okim6295_t s_okic[2];
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;

// ---- the NMK112 (the OKIs' 4 x 64 KB banks, the sample tables paged with them) ----
static uint8_t nmk112_read(void *user, uint32_t a)
{
	const int c = (int)(intptr_t)user;
	if (s_kind == K_NMK004) {   // (oki1_map / oki2_map with the NMK004's banks)
		const uint32_t o = a < 0x20000 ? a : 0x20000 + (uint32_t)b.nmk004_okibank[c] * 0x20000 + (a - 0x20000);
		return o < s_oki_size[c] ? s_oki[c][o] : 0;
	}
	const uint32_t pages = s_oki_size[c] >> 16;
	if (!pages)
		return 0;
	const uint32_t bank = (a < 0x400) ? (a >> 8) & 3 : (a >> 16) & 3;   // (the table: 0x100 bytes of each bank)
	const uint32_t page = b.okibank[c][bank] % pages;
	const uint32_t off = (a < 0x400) ? (a & 0xff) | (bank << 8) : (a & 0xffff);
	return s_oki[c][(page << 16) | off];
}

// ---- the 68000 ----
static uint16_t port16(int p) { return p >= 0 ? (uint16_t)rbx_port_read(p) : 0xffff; }
static uint32_t ram_index(uint32_t a)
{
	uint32_t w = (a & 0xffff) >> 1;
	if (s_kind == K_TDRAGON2 || s_kind == K_TDRAGON3H || s_kind == K_RAPHERO) {   // bitswap<16>(15..11, 7, 9, 8, 10, 6..0)
		const uint32_t b7 = (w >> 7) & 1, b9 = (w >> 9) & 1, b8 = (w >> 8) & 1, b10 = (w >> 10) & 1;
		w = (w & 0xf87f) | (b7 << 10) | (b9 << 9) | (b8 << 8) | (b10 << 7);
	}
	return w;
}
static void scroll_w(int offset, uint8_t data)   // scroll_w<0>
{
	b.scroll[offset & 3] = data;
	if (!(offset & 2) && (offset & 3) == 0)
		b.tilerambank = (uint8_t)((b.scroll[0] >> 4) & 3);   // (64 KB of tile RAM: 4 banks)
}
static uint16_t lay_r16(uint32_t a);
static void lay_w16m(uint32_t a, uint16_t v, uint16_t mask);
static uint16_t main_r16(uint32_t a)
{
	a &= 0xfffffe;
	if (s_lay)
		return lay_r16(a);
	if (s_kind == K_BJTWIN) {
		switch (a) {
		case 0x080000: return port16(s_port_in0);
		case 0x080002: return port16(s_port_in1);
		case 0x080008: return port16(s_port_dsw1);
		case 0x08000a: return port16(s_port_dsw2);
		case 0x084000: return (uint16_t)(0xff00 | oki_read(&s_okic[0]));
		case 0x084010: return (uint16_t)(0xff00 | oki_read(&s_okic[1]));
		}
		if (a >= 0x088000 && a < 0x088800) return b.paletteram[(a & 0x7ff) >> 1];
		if (a >= 0x09c000 && a < 0x09e000) return b.bgvram[(a & 0xfff) >> 1];
		if (a >= 0x0f0000) return b.mainram[(a & 0xffff) >> 1];
		return 0xffff;
	}
	switch (a) {
	case 0x100000: return port16(s_port_in0);
	case 0x100002: return port16(s_port_in1);
	case 0x100008: return port16(s_port_dsw1);
	case 0x10000a: return s_kind == K_TDRAGON3H ? (uint16_t)(0xff00 | b.soundlatch2) : port16(s_port_dsw2);
	case 0x10000e: return s_kind == K_TDRAGON3H ? port16(s_port_dsw2) : (uint16_t)(0xff00 | b.soundlatch2);
	}
	if (a >= 0x120000 && a < 0x120800) return b.paletteram[(a & 0x7ff) >> 1];
	if (s_kind == K_RAPHERO && a >= 0x130000 && a < 0x130800) {
		if (a < 0x130200) return b.scrollram[(a & 0x1ff) >> 1];
		if (a < 0x130400) return b.scrollramy[(a & 0x1ff) >> 1];
		return 0xffff;
	}
	if (a >= 0x130000 && a < 0x130008) return (uint16_t)(0xff00 | b.scroll[(a & 7) >> 1]);
	if (a >= 0x140000 && a < 0x150000) return b.bgvram[(a & 0xffff) >> 1];
	if (a >= 0x170000 && a < 0x172000) return b.txvram[(a & 0xfff) >> 1];
	if (a >= 0x1f0000) return b.mainram[ram_index(a)];
	return 0xffff;
}
static uint8_t main_r8(uint32_t a)
{
	const uint16_t w = main_r16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}
static void mem16(uint16_t *p, uint16_t v, uint16_t mask) { *p = (uint16_t)((*p & ~mask) | (v & mask)); }
static void z_reset_line(int held);
static void main_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xfffffe;
	if (s_lay) {
		lay_w16m(a, v, mask);
		return;
	}
	if (s_kind == K_BJTWIN) {
		if (a == 0x080014 && (mask & 0xff)) { b.flip = v & 1; return; }
		if (a == 0x084000 && (mask & 0xff)) { oki_write(&s_okic[0], (uint8_t)v); return; }
		if (a == 0x084010 && (mask & 0xff)) { oki_write(&s_okic[1], (uint8_t)v); return; }
		if (a >= 0x084020 && a < 0x084030 && (mask & 0xff)) {   // nmk112 okibank_w (umask 00ff)
			const int off = (int)((a - 0x084020) >> 1);
			b.okibank[(off >> 2) & 1][off & 3] = (uint8_t)v;
			return;
		}
		if (a >= 0x088000 && a < 0x088800) { mem16(&b.paletteram[(a & 0x7ff) >> 1], v, mask); return; }
		if (a == 0x094000 && (mask & 0xff)) { b.bgbank = (uint8_t)v; return; }
		if (a == 0x094002 && (mask & 0xff)) { b.bjtwin_scrolly = (int16_t)(-(int)(uint8_t)v); return; }
		if (a >= 0x09c000 && a < 0x09e000) { mem16(&b.bgvram[(a & 0xfff) >> 1], v, mask); return; }
		if (a >= 0x0f0000) { mem16(&b.mainram[(a & 0xffff) >> 1], v, mask); return; }
		return;
	}
	if (a == 0x100014 && (mask & 0xff)) { b.flip = v & 1; return; }
	if (a == 0x100016) {   // macross2_sound_reset_w (raphero: nothing)
		if (s_kind != K_RAPHERO)
			z_reset_line(!v);
		return;
	}
	if (a == 0x100018 && (mask & 0xff)) { b.bgbank = (uint8_t)v; return; }
	if (a == 0x10001e && (mask & 0xff)) { b.soundlatch = (uint8_t)v; return; }
	if (a >= 0x120000 && a < 0x120800) { mem16(&b.paletteram[(a & 0x7ff) >> 1], v, mask); return; }
	if (s_kind == K_RAPHERO && a >= 0x130000 && a < 0x130800) {
		if (a < 0x130200) {   // raphero_scroll_w: the tile RAM bank in the first word
			mem16(&b.scrollram[(a & 0x1ff) >> 1], v, mask);
			if (a == 0x130000)
				b.tilerambank = (uint8_t)((b.scrollram[0] >> 12) & 3);
		} else if (a < 0x130400)
			mem16(&b.scrollramy[(a & 0x1ff) >> 1], v, mask);
		return;
	}
	if (a >= 0x130000 && a < 0x130008) { if (mask & 0xff) scroll_w((int)((a & 7) >> 1), (uint8_t)v); return; }
	if (a >= 0x140000 && a < 0x150000) { mem16(&b.bgvram[(a & 0xffff) >> 1], v, mask); return; }
	if (a >= 0x170000 && a < 0x172000) { mem16(&b.txvram[(a & 0xfff) >> 1], v, mask); return; }
	if (a >= 0x1f0000) { mem16(&b.mainram[ram_index(a)], v, mask); return; }
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v)
{
	main_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00);
}

// ---- the NMK004 boards' maps ----
static inline int in_range(uint32_t a, uint32_t base, uint32_t size, uint32_t mirror)
{
	return base && (a & ~mirror) >= base && (a & ~mirror) < base + size;
}
static uint16_t lay_r16(uint32_t a)
{
	const layout_t *l = s_lay;
	if (in_range(a, l->io, 0x20, l->io_mirror)) {
		switch ((a & ~l->io_mirror) - l->io) {
		case 0x00: return port16(s_port_in0);
		case 0x02: return port16(s_port_in1);
		case 0x04: return l->dsw1_at4 ? port16(s_port_dsw1) : 0xffff;
		case 0x08: return l->dsw1_at4 ? 0xffff : port16(s_port_dsw1);
		case 0x0a: return port16(s_port_dsw2);
		case 0x0e: return (uint16_t)(0xff00 | b.to_main);   // nmk004 read (byte 0x0f)
		}
		return 0xffff;
	}
	if (in_range(a, l->pal, l->pal_size, 0)) return b.paletteram[(a - l->pal) >> 1];
	if (l->scroll_kind == 0 && in_range(a, l->scroll, 8, 0)) return (uint16_t)(0xff00 | b.scroll[(a - l->scroll) >> 1]);
	if (l->bg1 && in_range(a, l->bg1, 0x4000, 0)) return b.bgvram1[(a - l->bg1) >> 1];
	if (in_range(a, l->bg, l->bg_size, 0)) return b.bgvram[(a - l->bg) >> 1];
	if (in_range(a, l->tx, l->tx_size, 0)) return b.txvram[(a - l->tx) >> 1];
	if (in_range(a, l->ram, 0x10000, l->ram_mirror)) return b.mainram[((a & ~l->ram_mirror) - l->ram) >> 1];
	return 0xffff;
}
static void lay_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	const layout_t *l = s_lay;
	if (in_range(a, l->io, 0x20, l->io_mirror)) {
		switch ((a & ~l->io_mirror) - l->io) {
		case 0x14: if (mask & 0xff) b.flip = (uint8_t)((l->flip_inv ? ~v : v) & 1); break;
		case 0x16:   // nmk004_x0016_w (the watchdog NMI; bioship's inverted)
			tlcs90_set_irq(&s_n4, TLCS90_LINE_NMI, l->video == 2 ? !(v & 1) : (v & 1));
			break;
		case 0x18: if (mask & 0xff) b.bgbank = (uint8_t)v; break;
		case 0x1e: if (mask & 0xff) b.to_nmk004 = (uint8_t)v; break;
		}
		return;
	}
	if (in_range(a, l->pal, l->pal_size, 0)) { mem16(&b.paletteram[(a - l->pal) >> 1], v, mask); return; }
	if (l->scroll1 && in_range(a, l->scroll1, 8, 0)) {   // scroll_w<1> (bioship: the high bytes)
		const int off = (int)((a - l->scroll1) >> 1);
		if (l->scroll_kind == 3) {
			if (mask & 0xff00) b.scroll1[off & 3] = (uint8_t)(v >> 8);
		} else if (mask & 0xff)
			b.scroll1[off & 3] = (uint8_t)v;
		return;
	}
	if (l->bg1 && in_range(a, l->bg1, 0x4000, 0)) { mem16(&b.bgvram1[(a - l->bg1) >> 1], v, mask); return; }
	if (l->video == 2 && a == 0x084000) { if (mask & 0xff) b.bioship_bank = (uint8_t)v; return; }   // bioship_bank_w
	if (in_range(a, l->scroll, 8, 0)) {
		const int off = (int)((a - l->scroll) >> 1);
		if (l->scroll_kind == 3) {   // scroll_w<0>, umask ff00
			if (mask & 0xff00) b.scroll[off & 3] = (uint8_t)(v >> 8);
		} else if (l->scroll_kind == 0) {
			if (mask & 0xff) scroll_w(off, (uint8_t)v);
		} else if (l->scroll_kind == 1) {   // mustang_scroll_w (at the first word)
			if (off == 0) {
				if ((v & 0xff00) == 0x0000) b.mustang_xscroll = (uint16_t)((b.mustang_xscroll & 0x00ff) | ((v & 0xff) << 8));
				else if ((v & 0xff00) == 0x0100) b.mustang_xscroll = (uint16_t)((b.mustang_xscroll & 0xff00) | (v & 0xff));
			}
		} else   // vandyke_scroll_w
			b.vscroll[off & 3] = v;
		return;
	}
	if (in_range(a, l->bg, l->bg_size, 0)) { mem16(&b.bgvram[(a - l->bg) >> 1], v, mask); return; }
	if (in_range(a, l->tx, l->tx_size, 0)) { mem16(&b.txvram[(a - l->tx) >> 1], v, mask); return; }
	if (in_range(a, l->ram, 0x10000, l->ram_mirror)) { mem16(&b.mainram[((a & ~l->ram_mirror) - l->ram) >> 1], v, mask); return; }
}

// ---- the NMK004 (a TMP90C840: its internal ROM, the board's 0x2000-0xefff, the YM2203, the OKIs, the latches) ----
static uint8_t n4_read(void *u, uint32_t a)
{
	(void)u;
	a &= 0xffff;
	if (a >= 0x2000 && a < 0xf000) return a < s_n4ext_size ? s_n4ext[a] : 0xff;
	if (a >= 0xf000 && a < 0xf800) return s_n4ram[a - 0xf000];
	if (a == 0xf800 || a == 0xf801) return rbx_ym_read(0, a & 1);
	if (a == 0xf900) return oki_read(&s_okic[0]);
	if (a == 0xfa00) return oki_read(&s_okic[1]);
	if (a == 0xfb00) return b.to_nmk004;
	return 0xff;
}
static void n4_write(void *u, uint32_t a, uint8_t v)
{
	(void)u;
	a &= 0xffff;
	if (a >= 0xf000 && a < 0xf800) s_n4ram[a - 0xf000] = v;
	else if (a == 0xf800 || a == 0xf801) rbx_ym_write(0, a & 1, v);
	else if (a == 0xf900) oki_write(&s_okic[0], v);
	else if (a == 0xfa00) oki_write(&s_okic[1], v);
	else if (a == 0xfc00) b.to_main = v;
	else if (a == 0xfc01) b.nmk004_okibank[0] = v & 3;
	else if (a == 0xfc02) b.nmk004_okibank[1] = v & 3;
}
static void n4_port_w(void *u, int port, uint8_t v)   // port4_w: the 68000's reset
{
	(void)u;
	if (port != 4)
		return;
	const int held = v & 1;
	if (held && !b.cpu_reset_held)
		m68k_pulse_reset();
	b.cpu_reset_held = (uint8_t)held;
}
static void n4_setup(void)
{
	tlcs90_init(&s_n4, TLCS90_TMP90840);
	s_n4.rom = s_n4rom;
	s_n4.read8 = n4_read;
	s_n4.write8 = n4_write;
	s_n4.port_w = n4_port_w;
	tlcs90_reset(&s_n4);
}
static void ym_irq_n4(int state) { tlcs90_set_irq(&s_n4, TLCS90_LINE_INT0, state); }
static uint8_t rh_read(void *u, uint32_t a)
{
	(void)u;
	a &= 0xffff;
	if (a < 0x8000) return a < s_zrom_size ? s_zrom[a] : 0xff;
	if (a < 0xc000) {
		const uint32_t o = (uint32_t)(b.zbank & 7) * 0x4000 + (a - 0x8000);
		return o < s_zrom_size ? s_zrom[o] : 0xff;
	}
	if (a == 0xc000 || a == 0xc001) return rbx_ym_read(0, a & 1);
	if (a == 0xc800) return oki_read(&s_okic[0]);
	if (a == 0xc808) return oki_read(&s_okic[1]);
	if (a == 0xd800) return b.soundlatch;
	if (a >= 0xe000) return s_zram[a - 0xe000];
	return 0xff;
}
static void rh_write(void *u, uint32_t a, uint8_t v)
{
	(void)u;
	a &= 0xffff;
	if (a == 0xc000 || a == 0xc001) rbx_ym_write(0, a & 1, v);
	else if (a == 0xc800) oki_write(&s_okic[0], v);
	else if (a == 0xc808) oki_write(&s_okic[1], v);
	else if (a >= 0xc810 && a <= 0xc817) b.okibank[(a >> 2) & 1][a & 3] = v;
	else if (a == 0xd000) b.zbank = v & 7;
	else if (a == 0xd800) b.soundlatch2 = v;
	else if (a >= 0xe000) s_zram[a - 0xe000] = v;
}
static void rh_setup(void)
{
	tlcs90_init(&s_n4, TLCS90_TMP90841);
	s_n4.read8 = rh_read;
	s_n4.write8 = rh_write;
	tlcs90_reset(&s_n4);
}

// ---- the protection MCU (tdragon_prot_state / macross_prot_state): a TLCS-90 on the 68000's bus ----
#include "../machine/nmk214.h"
static int s_prot;
static tlcs90 s_pmcu;
static const uint8_t *s_prom;
static int s_gfx_decoded, s_line_now;
static const uint8_t s_214_spr_bits[13] = { 0, 1, 2, 3, 10, 12, 13, 14, 15, 16, 17, 18, 19 };
static const uint8_t s_214_bg_bits[13] = { 0, 1, 2, 3, 11, 13, 14, 15, 16, 17, 18, 19, 20 };
static uint8_t main_r8(uint32_t a);
static void main_w8(uint32_t a, uint8_t v);
static uint8_t pm_read(void *u, uint32_t a) { (void)u; return main_r8(a & 0xfffff); }
static void pm_write(void *u, uint32_t a, uint8_t v) { (void)u; main_w8(a & 0xfffff, v); }
static void decode_nmk214(void)
{
	nmk214_t spr = { 0, s_214_spr_bits, b.nmk214_cfg[0], 1 }, bg = { 1, s_214_bg_bits, b.nmk214_cfg[1], 1 };
	for (uint32_t a = 0; a < s_bg_size; a++)
		s_bg[a] = nmk214_decode_byte(&bg, a, s_bg[a]);
	for (uint32_t a = 0; a + 1 < s_spr_size; a += 2) {   // (16-bit big-endian words)
		const uint16_t w = nmk214_decode_word(&spr, a / 2, (uint16_t)(s_spr[a] << 8 | s_spr[a + 1]));
		s_spr[a] = (uint8_t)(w >> 8);
		s_spr[a + 1] = (uint8_t)w;
	}
	s_gfx_decoded = 1;
}
static void pm_port_w(void *u, int port, uint8_t v)
{
	(void)u;
	if (port == 6) {   // mcu_port6_w: the 68000's bus taken / given back
		if (v == 0x08) b.cpu_halt = 1;
		else if (v == 0x0b) b.cpu_halt = 0;
	} else if (port == 3 && s_prot == 2) {   // mcu_port3_to_214_w
		if (b.init_clock_214 == 0 && (v & 4)) {
			for (int i = 0; i < 2; i++)
				if (((b.init_data_214 >> 3) & 1) == i) {   // (nmk214 set_init_config: mode 0 sprites, 1 background)
					b.nmk214_cfg[i] = b.init_data_214 & 7;
					b.nmk214_init[i] = 1;
				}
			if (!b.unscramble && b.nmk214_init[0] && b.nmk214_init[1]) {
				b.unscramble = 1;
				if (!s_gfx_decoded)
					decode_nmk214();
			}
		}
		b.init_clock_214 = (v >> 2) & 1;
	} else if (port == 7 && s_prot == 2)   // mcu_port7_to_214_w
		b.init_data_214 = v;
}
static uint8_t pm_port_r(void *u, int port)
{
	(void)u;
	if (port == 5) return (uint8_t)(s_line_now >> 2);   // mcu_port5_r: the screen's line
	if (port == 6) {   // mcu_port6_r
		b.bus_status ^= 0x04;
		return b.bus_status;
	}
	return 0xff;
}
static void pm_setup(void)
{
	tlcs90_init(&s_pmcu, s_prot == 1 ? TLCS90_TMP91640 : TLCS90_TMP90840);
	s_pmcu.rom = s_prom;
	s_pmcu.read8 = pm_read;
	s_pmcu.write8 = pm_write;
	s_pmcu.port_r = pm_port_r;
	s_pmcu.port_w = pm_port_w;
	tlcs90_reset(&s_pmcu);
}

// ---- the Z80 (macross2_sound_map, macross2_sound_io_map) ----
static void z_bank(void)
{
	for (int p = 0x80; p < 0xc0; p++) {
		const uint32_t o = (uint32_t)(b.zbank & 7) * 0x4000 + (uint32_t)(p - 0x80) * 256;
		s_zmap[p] = o + 256 <= s_zrom_size ? s_zrom + o : NULL;
	}
}
static uint8_t z_read(void *u, uint16_t a)
{
	(void)u;
	const uint8_t *p = s_zmap[a >> 8];
	if (p)
		return p[a & 0xff];
	if (a >= 0xc000 && a < 0xe000)
		return s_zram[a - 0xc000];
	if (a == 0xf000)
		return b.soundlatch;
	return 0xff;
}
static void z_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xe000)
		s_zram[a - 0xc000] = v;
	else if (a == 0xe001) {
		b.zbank = v & 7;
		z_bank();
	} else if (a == 0xf000)
		b.soundlatch2 = v;
}
static uint8_t z_in(z80 *z, uint8_t p)
{
	(void)z;
	if (p <= 0x01 && s_kind != K_TDRAGON3H) return rbx_ym_read(0, p & 1);
	if (p == 0x80 && s_kind != K_TDRAGON3H) return oki_read(&s_okic[0]);
	if (p == 0x88) return oki_read(&s_okic[1]);
	return 0xff;
}
static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (p <= 0x01 && s_kind != K_TDRAGON3H) rbx_ym_write(0, p & 1, v);
	else if (p == 0x80 && s_kind != K_TDRAGON3H) oki_write(&s_okic[0], v);
	else if (p == 0x88) oki_write(&s_okic[1], v);
	else if (p >= 0x90 && p <= 0x97) b.okibank[(p >> 2) & 1][p & 3] = v;
}
static void ym_irq(int state) { s_zirq = state; }
static void z_start(void)
{
	z80_init(&s_z80);
	s_z80.read_byte = z_read;
	s_z80.write_byte = z_write;
	s_z80.port_in = z_in;
	s_z80.port_out = z_out;
	s_z80.read_map = s_zmap;
}
static void z_reset_line(int held)
{
	if (held && !b.zreset)
		z_start();
	b.zreset = (uint8_t)held;
}

// ---- drawing ----
typedef struct { uint32_t code, colour; uint8_t flipx, flipy; int16_t x, y; } spr_t;
static struct {
	uint16_t paletteram[0x400];
	uint16_t bgvram[0x2000];     // (the tile RAM bank shown)
	uint16_t txvram[0x800];
	uint8_t scroll[4], bgbank, flip;
	int16_t bjtwin_scrolly;
	int scrollx, scrolly;
	int linescroll;
	int video, scrollx1, scrolly1;
	uint8_t bioship_bank;
	uint16_t bgvram1[0x2000];
	uint16_t scrollram[0x100], scrollramy[0x100];
	int pal_dirty, nspr;
	spr_t spr[0x1000 / 0x10 * 16 * 16];
} *s_snap;
static uint16_t s_pal_conv[0x400];
static int s_txbase = 0x300;   // (gfx_macross2: 0x300; gfx_macross: 0x200)
static int s_bgbase, s_sprbase = 0x100;
static uint8_t *s_bg2;
static uint32_t s_bg2_size, s_tilerom_size;
static const uint16_t *s_tilerom;
static int s_pal_valid;

static inline uint8_t pix8(const uint8_t *g, uint32_t count, uint32_t code, int x, int y)   // gfx_8x8x4_packed_msb
{
	const uint8_t v = g[(code % count) * 32 + (uint32_t)y * 4 + (uint32_t)(x >> 1)];
	return (x & 1) ? (v & 15) : (v >> 4);
}
static inline uint8_t pix16(const uint8_t *g, uint32_t count, uint32_t code, int x, int y)   // ..._col_2x2_group_packed_msb
{
	const uint32_t q = (uint32_t)((x >> 3) * 2 + (y >> 3));
	const uint8_t v = g[(code % count) * 128 + q * 32 + (uint32_t)(y & 7) * 4 + (uint32_t)((x & 7) >> 1)];
	return (x & 1) ? (v & 15) : (v >> 4);
}

// nmk_16bit_sprite_device::draw_sprites (its list, the cliprect the visible area: 28..411, 16..239)
static void build_sprites(const uint16_t *spriteram, int colmask)
{
	const int xmask = 0x1ff, ymask = 0x1ff, xpos_max = 0x200, ypos_max = 0x200, videoshift = 28 + 64;
	const int minx = VIS_X0, maxx = VIS_X0 + VIS_W - 1, miny = VIS_Y0, maxy = VIS_Y0 + VIS_H - 1;
	const uint32_t max_clock = s_visw == 256 ? 384 * 263 : 512 * 263;
	uint32_t clk = 0;
	s_snap->nspr = 0;
	for (int offs = 0; offs < 0x800; offs += 8) {
		clk += 16;
		if (clk >= max_clock)
			break;
		if (!(spriteram[offs + 0] & 1))
			continue;
		int sx = (spriteram[offs + 4] & xmask) + videoshift, sy = spriteram[offs + 6] & ymask;
		int code = spriteram[offs + 3];
		const uint32_t colour = spriteram[offs + 7] & (uint32_t)colmask;
		const int w = spriteram[offs + 1] & 0x00f, h = (spriteram[offs + 1] & 0x0f0) >> 4;
		const int flipx = 0, flipy = 0;
		clk += 128u * (uint32_t)w * (uint32_t)h;
		if (clk >= max_clock)
			break;
		const int delta = 16, xinc = delta, yinc = delta;
		int xx_base = w, yy = h;
		if (sx > maxx) sx -= xpos_max;
		if (sx < minx - 0xf) {
			const int pixels = ((minx - 0xf) - sx) / 16;
			code += pixels;
			sx += pixels * xinc;
			xx_base -= pixels;
		}
		if (sy > maxy) sy -= ypos_max;
		if (sy < miny - 0xf) {
			const int pixels = ((miny - 0xf) - sy) / 16;
			code += pixels * (w + 1);
			sy += pixels * yinc;
			yy -= pixels;
		}
		if (xx_base < 0 || yy < 0)
			continue;
		do {
			if (sy > maxy) sy -= ypos_max;
			if (sy < miny - 0xf) {
				const int pixels = ((miny - 0xf) - sy) / 16;
				code += pixels * (w + 1);
				sy += pixels * yinc;
				yy -= pixels;
			}
			if (yy < 0)
				continue;
			int x = sx, xx = xx_base, codecol = code;
			do {
				if (x > maxx) x -= xpos_max;
				if (x < minx - 0xf) {
					const int pixels = ((minx - 0xf) - x) / 16;
					codecol += pixels;
					x += pixels * xinc;
					xx -= pixels;
				}
				if (xx < 0)
					continue;
				spr_t *s = &s_snap->spr[s_snap->nspr++];
				s->code = (uint32_t)codecol;
				s->colour = colour;
				s->flipx = (uint8_t)flipx;
				s->flipy = (uint8_t)flipy;
				s->x = (int16_t)(x - VIS_X0);
				s->y = (int16_t)(sy - VIS_Y0);
				codecol++;
				x += xinc;
			} while (--xx >= 0);
			code += w + 1;
			sy += yinc;
		} while (--yy >= 0);
	}
}

#define BANDS 4
static uint8_t (*s_pri_hp)[VIS_H][VIS_WMAX];
#define s_pri (*s_pri_hp)
static int s_band_next, s_band_left, s_band_active, s_band_helpers;

static void draw_band(int y0, int y1)
{
	const uint32_t bgcount = s_bg_size / (s_kind == K_BJTWIN ? 32 : 128), fgcount = s_fg_size / 32;
	const uint32_t sprcount = s_spr_size / 128;
	for (int y = y0; y < y1; y++) {
		uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
		uint8_t *pri = s_pri[y];
		const int sy = y + VIS_Y0;
		if (s_kind == K_BJTWIN) {   // the 8x8 background (64x32, columns), opaque
			const int ty = (sy + s_snap->bjtwin_scrolly) & 0xff;
			for (int x = 0; x < VIS_W; x++) {
				const int tx = (x + VIS_X0 - 92) & 0x1ff;
				const uint16_t code = s_snap->bgvram[((tx >> 3) * 32 + (ty >> 3)) & 0x7ff];
				const int bank = (code >> 11) & 1;
				const uint32_t tile = (uint32_t)(code & 0x7ff) + (bank ? (uint32_t)s_snap->bgbank << 11 : 0);
				const uint8_t p = bank ? pix8(s_bg, bgcount, tile, tx & 7, ty & 7) : pix8(s_fg, fgcount, tile, tx & 7, ty & 7);
				d[x] = (uint16_t)((code >> 12) * 16 + p);
				pri[x] = 1;
			}
			continue;
		}
		// the 16x16 background (256x32 tiles in 16x16 pages), opaque
		int scrollx = s_snap->scrollx, scrolly = s_snap->scrolly;
		if (s_snap->linescroll) {   // bg_update: each line its own scroll (index: the screen line)
			scrolly = s_snap->scrollramy[0] + s_snap->scrollramy[sy & 0xff];
			scrollx = s_snap->scrollram[0] + s_snap->scrollram[sy & 0xff];
		}
		const int ty = (sy + scrolly) & 0x1ff;
		for (int x = 0; x < VIS_W; x++) {
			const int tx = (x + VIS_X0 - 92 + scrollx) & 0xfff;
			const int col = tx >> 4, row = ty >> 4;
			const uint32_t idx = (uint32_t)(((row & 0xf) | ((col & 0xff) << 4) | ((row & 0x10) << 8)) & 0x1fff);
			if (s_snap->video == 2) {   // bioship_get_bg_tile_info: the ROM map, gfx 3 (bg2tile)
				const uint32_t ri = ((uint32_t)s_snap->bioship_bank << 13 | idx) % (s_tilerom_size / 2 ? s_tilerom_size / 2 : 1);
				const uint16_t code = s_tilerom ? (uint16_t)(s_tilerom[ri] << 8 | s_tilerom[ri] >> 8) : 0;   // (ROM_REGION16_BE)
				d[x] = (uint16_t)((code >> 12) * 16 + pix16(s_bg2, s_bg2_size / 128, code & 0xfff, tx & 15, ty & 15));
			} else {
				const uint16_t code = s_snap->bgvram[idx];
				const uint32_t tile = (uint32_t)(code & 0xfff) | ((uint32_t)s_snap->bgbank << 12);
				d[x] = (uint16_t)(s_bgbase + (code >> 12) * 16 + pix16(s_bg, bgcount, tile, tx & 15, ty & 15));
			}
			pri[x] = 1;
		}
		if (s_snap->video) {   // the second background (16x16 pages, pen 15 transparent)
			const uint32_t cnt3 = s_bg2_size / 128;
			const int vx0 = s_snap->video == 1 ? s_snap->scrollx1 : s_snap->scrollx1, vy0 = s_snap->scrolly1;
			const int ty1 = (sy + vy0) & 0x1ff;
			for (int x = 0; x < VIS_W; x++) {
				const int tx = (x + VIS_X0 - 92 + vx0) & 0xfff;
				const int col = tx >> 4, row = ty1 >> 4;
				const uint16_t code = s_snap->bgvram1[((row & 0xf) | ((col & 0xff) << 4) | ((row & 0x10) << 8)) & 0x1fff];
				uint8_t p;
				uint16_t base;
				if (s_snap->video == 1) {   // strahl: gfx 3 (bg2tile), banked
					p = pix16(s_bg2, cnt3, (uint32_t)(code & 0xfff) | ((uint32_t)s_snap->bgbank << 12), tx & 15, ty1 & 15);
					base = 0x200;
				} else {   // bioship: gfx 1 (bgtile)
					p = pix16(s_bg, bgcount, (uint32_t)(code & 0xfff) | ((uint32_t)s_snap->bgbank << 12), tx & 15, ty1 & 15);
					base = 0x100;
				}
				if (p != 15) {
					d[x] = (uint16_t)(base + (code >> 12) * 16 + p);
					pri[x] |= 1;
				}
			}
		}
		// the text layer (64x32, columns; pen 15 transparent)
		const int txy = sy & 0xff;
		for (int x = 0; x < VIS_W; x++) {
			const int tx = (x + VIS_X0 - 92) & 0x1ff;
			const uint16_t code = s_snap->txvram[((tx >> 3) * 32 + (txy >> 3)) & 0x7ff];
			const uint8_t p = pix8(s_fg, fgcount, code & 0xfff, tx & 7, txy & 7);
			if (p != 15) {
				d[x] = (uint16_t)(s_txbase + (code >> 12) * 16 + p);
				pri[x] |= 2;
			}
		}
	}
	// the sprites (prio_transpen, pen 15; GFX_PMASK_2: under the text layer), the first drawn decides
	const uint32_t pmask = 0xcccc | 0x80000000u;
	for (int i = s_snap->nspr - 1; i >= 0; i--) {
		const spr_t *s = &s_snap->spr[i];
		if (s->y > y1 - 1 || s->y + 15 < y0 || s->x > VIS_W - 1 || s->x + 15 < 0)
			continue;
		const uint16_t base = (uint16_t)(s_sprbase + s->colour * 16);
		for (int py = 0; py < 16; py++) {
			const int y = s->y + py;
			if (y < y0 || y >= y1)
				continue;
			uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
			uint8_t *pri = s_pri[y];
			for (int px = 0; px < 16; px++) {
				const int x = s->x + px;
				if (x < 0 || x >= VIS_W)
					continue;
				const uint8_t p = pix16(s_spr, sprcount, s->code, s->flipx ? 15 - px : px, s->flipy ? 15 - py : py);
				if (p == 15)
					continue;
				if (!((1u << (pri[x] & 0x1f)) & pmask))
					d[x] = (uint16_t)(base + p);
				pri[x] = 31;
			}
		}
	}
}

static void run_band(int band) { draw_band(VIS_H * band / BANDS, VIS_H * (band + 1) / BANDS); }

static void nmk_render(void)
{
	if (s_snap->pal_dirty || !s_pal_valid) {
		for (int i = 0; i < 0x400; i++) {
			const uint16_t v = s_snap->paletteram[i];
			if (s_pal_valid && s_pal_conv[i] == v)
				continue;
			s_pal_conv[i] = v;
			const int r = ((v >> 11) & 0x1e) | ((v >> 3) & 1), g = ((v >> 7) & 0x1e) | ((v >> 2) & 1),
			          bb = ((v >> 3) & 0x1e) | ((v >> 1) & 1);
			rbx_video.palette[i] = (uint32_t)(((r << 3) | (r >> 2)) << 16 | ((g << 3) | (g >> 2)) << 8 | ((bb << 3) | (bb >> 2)));
		}
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

static int nmk_render_help(int helper)
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

static void draw_frame(void)   // (at VBOUT: MAME's screen update for the visible lines)
{
	rbx_video_wait_idle();
	if (memcmp(s_snap->paletteram, b.paletteram, sizeof(b.paletteram))) {
		memcpy(s_snap->paletteram, b.paletteram, sizeof(b.paletteram));
		s_snap->pal_dirty = 1;
	}
	if (s_kind == K_BJTWIN)
		memcpy(s_snap->bgvram, b.bgvram, 0x800 * 2);
	else
		memcpy(s_snap->bgvram, b.bgvram + (b.tilerambank << 13), sizeof(s_snap->bgvram));
	memcpy(s_snap->txvram, b.txvram, sizeof(b.txvram));
	memcpy(s_snap->scroll, b.scroll, sizeof(b.scroll));
	s_snap->bgbank = b.bgbank;
	s_snap->flip = b.flip;
	s_snap->bjtwin_scrolly = b.bjtwin_scrolly;
	s_snap->linescroll = s_kind == K_RAPHERO;
	s_snap->video = s_lay ? s_lay->video : 0;
	s_snap->scrollx1 = (b.scroll1[0] << 8) | b.scroll1[1];
	s_snap->scrolly1 = (b.scroll1[2] << 8) | b.scroll1[3];
	s_snap->bioship_bank = b.bioship_bank;
	if (s_snap->video == 1)
		memcpy(s_snap->bgvram1, b.bgvram1, sizeof(b.bgvram1));
	else if (s_snap->video == 2) {   // (bioship: its RAM map is the second layer, drawn over the ROM one)
		memcpy(s_snap->bgvram1, b.bgvram, sizeof(s_snap->bgvram1));
		s_snap->scrollx1 = (b.scroll1[0] << 8) | b.scroll1[1];
	}
	memcpy(s_snap->scrollram, b.scrollram, sizeof(b.scrollram));
	memcpy(s_snap->scrollramy, b.scrollramy, sizeof(b.scrollramy));
	if (s_lay && s_lay->scroll_kind == 1) {   // mustang_scroll_w
		s_snap->scrollx = b.mustang_xscroll;
		s_snap->scrolly = 0;
	} else if (s_lay && s_lay->scroll_kind == 2) {   // vandyke_scroll_w
		s_snap->scrollx = b.vscroll[0] * 256 + (b.vscroll[1] >> 8);
		s_snap->scrolly = b.vscroll[2] * 256 + (b.vscroll[3] >> 8);
	} else {
		s_snap->scrollx = (b.scroll[0] << 8) | b.scroll[1];
		s_snap->scrolly = (b.scroll[2] << 8) | b.scroll[3];
	}
	// (macross: the sprite DMA's older buffer; bjtwin: the single one)
	build_sprites(s_kind == K_BJTWIN ? b.spr_old : b.spr_old2, (s_kind == K_BJTWIN || s_kind == K_NMK004) ? 0x0f : 0x1f);
	rbx_video_frame_done();
}
static void sprite_dma(void)
{
	memcpy(b.spr_old2, b.spr_old, sizeof(b.spr_old));
	memcpy(b.spr_old, b.mainram + (s_lay && s_lay->video == 1 ? 0xf000 : 0x8000) / 2, sizeof(b.spr_old));
}

// nmk_irq_device::scanline_callback (the V-timing PROM, read on even lines), or the hacky timing
static void irq_line(int y)
{
	if (s_hacky_irq) {
		if (y == 240) m68k_bus_hold_irq(4);
		if (y == 242) sprite_dma();
		if (y == 16) m68k_bus_hold_irq(2);
		if (y == 68 || y == 196) m68k_bus_hold_irq(1);
		return;
	}
	if ((y & 1) || !s_vprom_size)
		return;
	const uint32_t start = 0x75, usage = 0x100;
	const uint32_t address = ((((uint32_t)y / 2 + start) % (usage - start)) + start) % s_vprom_size;
	const uint8_t val = s_vprom[address];
	if ((val & ~b.vtiming_val) & 0x80) {
		const int level = ((val >> 6) & 1) << 2 | ((val >> 5) & 1) << 1 | ((val >> 4) & 1);
		if (level)
			m68k_bus_hold_irq(level);
	}
	if ((val & ~b.vtiming_val) & 0x01)
		sprite_dma();
	b.vtiming_val = val;
}

// ---- the frame ----
static void nmk_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	const uint32_t rate = s_kind == K_BJTWIN ? s_okic[0].rate : rbx_ym_rate(0);
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0 + VIS_H)
			draw_frame();
		irq_line(line);
		const int mc = rbx_line_cycles(&b.cpu_frac, s_lay ? s_lay->cpu_clock : s_kind == K_TDRAGON3H ? 12000000 : s_kind == K_RAPHERO ? 14000000 : 10000000,
		                               FRAME_MHZ, LINES);
		s_line_now = line;
		if (mc > 0 && !b.cpu_reset_held && !b.cpu_halt)
			m68k_execute(mc);
		if (s_prot)
			tlcs90_execute(&s_pmcu, rbx_line_cycles(&b.mcu_frac, 4000000, FRAME_MHZ, LINES));
		int k;
		if (s_kind == K_BJTWIN) {
			k = (int)rbx_line_cycles(&b.ym_frac, rate, FRAME_MHZ, LINES);   // (the OKIs' samples of the line)
			if (k > s_mix_cap - s_mix_n)
				k = s_mix_cap - s_mix_n;
			for (int i = 0; i < k; i++)
				s_mix[s_mix_n + i] += oki_sample(&s_okic[0]) + oki_sample(&s_okic[1]);
		} else {
			if (s_kind == K_NMK004 || s_kind == K_RAPHERO)
				tlcs90_execute(&s_n4, rbx_line_cycles(&b.z80_frac, s_lay ? s_lay->nmk004_clock : 8000000, FRAME_MHZ, LINES));
			else {
				const int zc = rbx_line_cycles(&b.z80_frac, 4000000, FRAME_MHZ, LINES);
				if (!b.zreset)
					z80_run(&s_z80, s_z80.cyc + (unsigned long)zc, &s_zirq, 0);
			}
			k = s_kind == K_TDRAGON3H ? (int)rbx_line_cycles(&b.ym_frac, rate, FRAME_MHZ, LINES)
			                          : rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&b.ym_frac, YM_CLOCK, FRAME_MHZ, LINES), s_mix + s_mix_n, s_mix_cap - s_mix_n);
			if (k > s_mix_cap - s_mix_n)
				k = s_mix_cap - s_mix_n;
			for (int i = 0; i < k; i++) {   // (the OKIs held between their samples)
				b.oki_frac += s_okic[1].rate;
				while (b.oki_frac >= rate) {
					b.oki_frac -= rate;
					b.oki_cur = (s_kind == K_TDRAGON3H ? 0 : oki_sample(&s_okic[0])) + oki_sample(&s_okic[1]);
				}
				s_mix[s_mix_n + i] += b.oki_cur;
			}
		}
		s_mix_n += k;
	}
	if (s_kind != K_BJTWIN && s_kind != K_TDRAGON3H)
		rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	b.frame++;
}

static void nmk_reset(void)
{
	memset(&b, 0, offsetof(__typeof__(b), state_end));
	b.vtiming_val = 0xff;
	memset(s_zram, 0, sizeof(s_zram));
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x80 && (uint32_t)p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	z_bank();
	z_start();
	s_zirq = 0;
	b.to_nmk004 = b.to_main = 0xff;
	memset(s_n4ram, 0, sizeof(s_n4ram));
	if (s_kind == K_NMK004)
		n4_setup();
	if (s_kind == K_RAPHERO)
		rh_setup();
	b.bus_status = 0x04;
	if (s_prot)
		pm_setup();
	for (int c = 0; c < 2; c++) {
		oki_init(&s_okic[c], OKI_CLOCK, 0, NULL, 0x40000, s_kind == K_BJTWIN ? 51 : 26);   // (0.20 / 0.10)
		s_okic[c].read = nmk112_read;
		s_okic[c].user = (void *)(intptr_t)c;
	}
	if (s_kind != K_BJTWIN && s_kind != K_TDRAGON3H)
		rbx_ym_reset(0);
	s_mix_n = 0;
	s_pal_valid = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
}

static void nmk_exit(void)
{
	if (s_kind != K_BJTWIN && s_kind != K_TDRAGON3H)
		rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	free(s_snap);
	s_snap = NULL;
	free(s_bp);   // (last: the render thread and the saves are done with it)
	s_bp = NULL;
	free(s_pri_hp);
	s_pri_hp = NULL;
}

static int kind_of(const char *machine)
{
	if (!strncmp(machine, "macross2:", 9)) return K_MACROSS2;
	if (!strncmp(machine, "tdragon2:", 9)) return K_TDRAGON2;
	if (!strncmp(machine, "tdragon3h:", 10)) return K_TDRAGON3H;
	if (!strncmp(machine, "raphero:", 8)) return K_RAPHERO;
	if (!strcmp(machine, "bjtwin") || !strcmp(machine, "bjtwin_prot")) return K_BJTWIN;
	for (int i = 0; s_layouts[i].machine; i++)
		if (!strcmp(machine, s_layouts[i].machine) || (!strncmp(machine, s_layouts[i].machine, strlen(s_layouts[i].machine)) &&
		                                                machine[strlen(s_layouts[i].machine)] == ':'))
			return s_layouts[i].prot == 1 ? -1 : K_NMK004;   // (the NMK-110 boards: their MCU's protection not right yet)
	return -1;
}
static int nmk_supported(const char *machine) { return kind_of(machine) >= 0; }

static int nmk_init(const rbx_game *g)
{
	if (!s_pri_hp && !(s_pri_hp = calloc(1, sizeof(*s_pri_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_bp && !(s_bp = (struct nmk16_state *)calloc(1, sizeof(*s_bp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_kind = kind_of(g->machine);
	if (s_kind < 0) {
		rbx_fail("%s: machine %s not emulated", g->set, g->machine);
		return -1;
	}
	s_hacky_irq = 0;
	s_lay = NULL;
	if (s_kind == K_NMK004)
		for (int i = 0; s_layouts[i].machine; i++)
			if (!strncmp(g->machine, s_layouts[i].machine, strlen(s_layouts[i].machine)))
				s_lay = &s_layouts[i];
	s_visw = s_lay ? 256 : 384;
	s_visx0 = s_lay ? 92 : 28;
	s_txbase = s_lay ? 0x200 : 0x300;
	s_bgbase = 0;
	s_sprbase = 0x100;
	if (s_lay && s_lay->video == 1) {   // gfx_strahl
		s_txbase = 0x000;
		s_bgbase = 0x300;
	} else if (s_lay && s_lay->video == 2) {   // gfx_bioship
		s_txbase = 0x300;
		s_sprbase = 0x200;
	}
	s_bg2 = rbx_region("bg2tile", &s_bg2_size);
	s_tilerom = (const uint16_t *)rbx_region("tilerom", &s_tilerom_size);
	s_hacky_irq = s_lay && s_lay->video == 1;   // (strahl: set_hacky_interrupt_timing)
	s_prot = s_lay ? s_lay->prot : !strcmp(g->machine, "bjtwin_prot") ? 2 : 0;
	s_gfx_decoded = 0;
	{
		uint32_t psize = 0;
		s_prom = rbx_region("protcpu", &psize);
		if (s_prot && !s_prom) {
			rbx_fail("%s: no protection MCU ROM", g->set);
			return -1;
		}
	}
	s_n4ext = rbx_region("nmk004", &s_n4ext_size);
	{
		uint32_t n4size = 0;
		s_n4rom = rbx_region("nmk004:mcu", &n4size);
	}
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_fg = rbx_region("fgtile", &s_fg_size);
	s_bg = rbx_region("bgtile", &s_bg_size);
	s_spr = rbx_region("sprites", &s_spr_size);
	s_oki[0] = rbx_region("oki1", &s_oki_size[0]);
	s_oki[1] = rbx_region("oki2", &s_oki_size[1]);
	s_vprom = rbx_region("nmk_irq:vtiming", &s_vprom_size);
	if (!s_rom || !s_fg || !s_bg || !s_spr || !s_oki[1] || (s_kind != K_BJTWIN && s_kind != K_NMK004 && !s_zrom) ||
	    (s_kind == K_NMK004 && (!s_n4ext || !s_n4rom))) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (!s_oki[0]) {
		s_oki[0] = s_oki[1];
		s_oki_size[0] = 0;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	s_port_in0 = rbx_port_find("IN0");
	s_port_in1 = rbx_port_find("IN1");
	s_port_dsw1 = rbx_port_find("DSW1");
	s_port_dsw2 = rbx_port_find("DSW2");
	memset(&b, 0, sizeof(b));
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0x000000, 0x07ffff, s_rom, s_rom_size, 1, 0);
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
	if (s_kind == K_NMK004)
		m68k_bus_map(0x000000, s_rom_size > 0x40000 ? 0x07ffff : 0x03ffff, s_rom, s_rom_size, 1, 0);
	if (s_kind == K_MACROSS2 || s_kind == K_BJTWIN)
		m68k_bus_map(s_kind == K_BJTWIN ? 0x0f0000 : 0x1f0000, s_kind == K_BJTWIN ? 0x0fffff : 0x1fffff, b.mainram, sizeof(b.mainram), 1, 1);
	s_snap = calloc(1, sizeof(*s_snap));
	int bad = !s_snap;
	if (!bad && s_kind != K_BJTWIN && s_kind != K_TDRAGON3H) {
		rbx_ym_config yc;
		memset(&yc, 0, sizeof(yc));
		yc.type = RBX_YM2203;
		yc.clock = YM_CLOCK;
		yc.gain[0] = yc.gain[1] = yc.gain[2] = 128;   // (SSG 0.50 each, FM 1.20)
		yc.gain[3] = 307;
		yc.irq = (s_kind == K_NMK004 || s_kind == K_RAPHERO) ? ym_irq_n4 : ym_irq;
		bad = rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	}
	if (bad) {
		rbx_fail("%s: out of memory", g->set);
		nmk_exit();
		return -1;
	}
	const uint32_t rate = (s_kind == K_BJTWIN || s_kind == K_TDRAGON3H) ? OKI_CLOCK / 165 : rbx_ym_rate(0);
	s_mix_cap = (int)(rate / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rate) || rbx_video_setup(s_visw, VIS_H, PAL_BLACK + 1)) {
		rbx_fail("%s: out of memory", g->set);
		nmk_exit();
		return -1;
	}
	nmk_reset();
	return 0;
}

static int nmk_state(void *ctx, int load)
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
	rc |= rbx_state_io(ctx, "nmk16", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= rbx_state_io(ctx, "zirq", &s_zirq, sizeof(s_zirq));
	rc |= rbx_state_io(ctx, "oki0", s_okic[0].v, sizeof(s_okic[0].v));
	rc |= rbx_state_io(ctx, "oki1", s_okic[1].v, sizeof(s_okic[1].v));
	if (s_kind == K_NMK004 || s_kind == K_RAPHERO) {
		rc |= rbx_state_io(ctx, "nmk004", (uint8_t *)&s_n4 + TLCS90_STATE_OFFSET, (uint32_t)TLCS90_STATE_SIZE);
		rc |= rbx_state_io(ctx, "nmk004ram", s_n4ram, sizeof(s_n4ram));
	}
	if (s_prot)
		rc |= rbx_state_io(ctx, "protmcu", (uint8_t *)&s_pmcu + TLCS90_STATE_OFFSET, (uint32_t)TLCS90_STATE_SIZE);
	if (s_kind != K_BJTWIN && s_kind != K_TDRAGON3H)
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
		s_z80 = z;
		z_bank();
		if (b.unscramble && !s_gfx_decoded)   // (device_post_load: a state saved with the graphics unscrambled)
			decode_nmk214();
		s_pal_valid = 0;
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int nmk_save(void *ctx) { return nmk_state(ctx, 0); }
static int nmk_load(void *ctx) { return nmk_state(ctx, 1); }

const rbx_driver rbx_drv_nmk16 = { "NMK16", nmk_init, nmk_exit, nmk_reset, nmk_frame, nmk_save, nmk_load,
                                   nmk_supported, nmk_render, nmk_render_help };
