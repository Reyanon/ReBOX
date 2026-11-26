/*  Data East Liberation hardware: Liberation / Dual Assault (and the bootleg), Boomer Rang'r, Kamikaze Cabbie / Yellow
    Cab. Ported from MAME src/mame/dataeast/liberate.cpp, liberate_v.cpp (BSD-3-Clause, copyright-holders: Bryan
    McPhail).
    The main CPU: a DECO CPU-16 at 2 MHz (Liberation: its opcodes bit-swapped, a decrypted copy; the others: the
    program ROM's nibbles swapped; the bootleg: a plain 6502), its I/O port 0 switching in a view over 0x4000-0x800f
    (the tile map ROM and the inputs), its IRQ from a coin or service at vblank (acknowledged by an I/O write). The
    video: a 512x512 background of 16x16 tiles paged in from the map ROM, an 8x8 text layer, 16x16 sprites (Boomer
    Rang'r: two priorities around the background's two pen groups), a 32-colour PROM. The sound: a DECO 222 at 1.5 MHz
    (NMI at 960 Hz, IRQ on the latch write) and two AY-3-8912.
    Pro Soccer: the text layer's graphics in RAM (written through the "char RAM" window), the CPUs at 1.25 MHz, a 256-line
    tall picture. Pro Sports: its own map and I/O, four graphics sets for the text and the sprites (Pro Golf's in RAM),
    a palette RAM, the background from a tile RAM.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../sound/ay8910.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 60000          // set_refresh_hz(60), set_size(256, 256)
#define LINES 256
static int s_vx0, s_vy0, s_vw, s_vh;   // set_visarea(0, 255, 8, 247); prosoccr (8, 247, 0, 255)
static int s_vbl_lines;               // vblank 529 us (prosport 1529)
static uint32_t s_cpu_clock, s_snd_clock;

enum { K_LIBERATE, K_LIBERATB, K_BOOMRANG, K_KAMIKCAB, K_PROSOCCR, K_PROSPORT };
static int s_k, s_yellowcb;
static uint8_t *s_rom, *s_srom;
static uint32_t s_rom_size, s_srom_size;
static uint8_t (*s_opcodes_hp)[0x8000];           // (Liberation: the decrypted opcodes of 0x8000-0xffff)
#define s_opcodes (*s_opcodes_hp)
static const uint8_t *s_user1;              // the background map ROM (and the view's ROM at 0x4000)
static uint32_t s_user1_size;
static uint8_t s_ram[0x1000], s_ram2[0x600];   // 0x0000-0x0fff, 0x6200-0x67ff
static uint8_t s_cram[0x400], s_vram[0x400], s_spriteram[0x800];
static uint8_t s_io_ram[16];
static int s_io_sel;                        // the view (MAME m_deco16_io selected)
static int s_latch, s_bg_color, s_bg_disable, s_flip;
static int s_port_in[4], s_port_dsw[2], s_port_tilt;
static int s_line;
static uint32_t s_frame;
static int64_t s_cpu_target, s_snd_target;
static uint64_t s_cpu_frac, s_snd_frac, s_ay_frac, s_nmi_frac;
static m6502_t s_cpu, s_snd;
static uint32_t s_rgb[256], s_rgb_init[256];
// Pro Soccer / Pro Sports: the text (and sprite) graphics as bytes (gfx_8x8x3_planar / sprites over 0x6000), the
// written ones kept here (fg[0]: prosoccr's fg_gfx; prosport: [0..3] its four sets, [3] Pro Golf's written)
static uint8_t *s_fg[4];
static const uint8_t *s_fg_rom[4], *s_shared;
static uint8_t s_charram[0x3000];           // prosoccr: m_charram
static uint8_t s_readback;                  // prosoccr: m_gfx_rom_readback
static uint8_t s_bgram[0x400], s_palram[0x20], s_ram3[0xc00];   // prosport: bg_vram, the palette, 0x2400-0x2fff
static rbx_gfx s_gfxpro;                    // prosport: pro_tiles
static rbx_gfx s_gfxchar, s_gfxspr, s_gfxt1, s_gfxt2;

typedef struct {
	uint8_t cram[0x400], vram[0x400], spriteram[0x800], io_ram[16];
	uint8_t bg_color, bg_disable, flip;
	uint8_t bgram[0x400], fg[0x6000];      // (prosport's tile RAM; the text set in use)
	uint32_t rgb[256];
	uint32_t frame;
} snap_t;
static snap_t *s_snap;

// ---- sound: DECO 222, two AY-3-8912 ----
static ay8910_t s_ay[2];
static uint8_t s_sram[0x200];
static uint8_t s_slatch;
static int32_t *s_mix;
static int s_mix_cap;

static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x200) return s_sram[a];
	if (s_k == K_PROSOCCR) {   // prosoccr_sound_map
		if (a == 0xa000) return s_slatch;
		if (a >= 0xe000) return s_srom[a];
		return 0;
	}
	if (a == 0xb000) return s_slatch;
	if (a >= 0xc000) return s_srom[a];
	return 0;
}
static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x200) s_sram[a] = v;
	else if (s_k == K_PROSOCCR) {
		if (a == 0x2000) ay8910_data_w(&s_ay[0], v);
		else if (a == 0x4000) ay8910_address_w(&s_ay[0], v);
		else if (a == 0x6000) ay8910_data_w(&s_ay[1], v);
		else if (a == 0x8000) ay8910_address_w(&s_ay[1], v);
	}
	else if (a == 0x3000) ay8910_data_w(&s_ay[0], v);
	else if (a == 0x4000) ay8910_address_w(&s_ay[0], v);
	else if (a == 0x7000) ay8910_data_w(&s_ay[1], v);
	else if (a == 0x8000) ay8910_address_w(&s_ay[1], v);
}

static void sound_line(void)
{
	s_snd_target += rbx_line_cycles(&s_snd_frac, s_snd_clock, FRAME_MHZ, LINES);
	// nmi_line_pulse at 16 * 60 Hz: the edges falling in this line
	s_nmi_frac += (uint64_t)960 * 1000;
	while (s_nmi_frac >= (uint64_t)FRAME_MHZ * LINES) {
		s_nmi_frac -= (uint64_t)FRAME_MHZ * LINES;
		m6502_set_nmi(&s_snd, 1);
		m6502_set_nmi(&s_snd, 0);
	}
	m6502_run_until(&s_snd, s_snd_target);
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&s_ay_frac, 1500000, FRAME_MHZ, LINES);
	const int n0 = ay8910_render(&s_ay[0], clocks, s_mix, s_mix_cap);
	const int n1 = ay8910_render(&s_ay[1], clocks, s_mix, s_mix_cap);
	const int n = n0 > n1 ? n0 : n1;
	for (int k = 0; k < n; k++) {
		rbx_audio_put(s_mix[k]);
		s_mix[k] = 0;
	}
}

// ---- main bus ----
static int vblank(void) { return ((s_line - 248 + LINES) % LINES) < s_vbl_lines; }

static uint8_t deco16_io_r(int o)   // IN1, IN2 (bit 7: the vblank), IN3, DSW1, DSW2
{
	switch (o) {
	case 0: return (uint8_t)rbx_port_read(s_port_in[1]);
	case 1:
		rbx_port_custom(s_port_in[2], 0x80, vblank() ? 0x80 : 0);
		return (uint8_t)rbx_port_read(s_port_in[2]);
	case 2: return (uint8_t)rbx_port_read(s_port_in[3]);
	case 3: return (uint8_t)rbx_port_read(s_port_dsw[0]);
	case 4: return (uint8_t)rbx_port_read(s_port_dsw[1]);
	}
	return 0xff;
}
static uint8_t in0_r(void)   // IN0: the vblank on every bit (yellowcb: bit 2)
{
	rbx_port_custom(s_port_in[0], 0xff, vblank() ? 0xff : 0);
	return (uint8_t)rbx_port_read(s_port_in[0]);
}

static void deco16_io_w(int o, uint8_t v)
{
	s_io_ram[o] = v;
	if (s_k == K_PROSPORT) {   // prosport_io_w
		if (o == 0) s_flip = (v & 0x80) != 0;
		else if (o == 2) { s_slatch = v; m6502_set_irq(&s_snd, 1); }
		else if (o == 4) m6502_set_irq(&s_cpu, 0);
		return;
	}
	if (s_k == K_PROSOCCR) {   // prosoccr_io_w
		if (o == 7) s_bg_disable = ~v & 0x10;
		else if (o == 8) m6502_set_irq(&s_cpu, 0);
		else if (o == 9) { s_slatch = v; m6502_set_irq(&s_snd, 1); }
		return;
	}
	switch (o) {
	case 6:   // the background colour, its disable, the flip
		s_bg_color = (v >> 4) & 3;
		s_bg_disable = v & 4;
		s_flip = v & 1;
		break;
	case 8:   // the IRQ's acknowledge
		m6502_set_irq(&s_cpu, 0);
		break;
	case 9:   // the sound latch (HOLD_LINE on the sound CPU's IRQ)
		s_slatch = v;
		m6502_set_irq(&s_snd, 1);
		break;
	}
}

// the "char RAM" windows: offset 0..0x17ff, planes of 0x800 at 0x0000 / 0x0800 / 0x1000
static int fg_off(uint16_t o) { return (o & 0x1800) == 0x1800 ? -1 : (int)((o & 0x7ff) + ((o & 0x1800) >> 11) * 0x2000); }

static uint8_t pro_r(uint16_t a)
{
	if (s_k == K_PROSPORT) {   // prosport_map
		const uint16_t m = a & 0xdfff;   // (the mirror 0x2000 of the low page)
		if (m < 0x200 && a < 0x2400) return s_ram[m];
		if (m >= 0x200 && m < 0x220 && a < 0x2400) return s_palram[m - 0x200];
		if (m >= 0x220 && m < 0x400 && a < 0x2400) return s_ram[m];
		if (a >= 0x0400 && a < 0x0800) return s_bgram[a - 0x400];
		if (a >= 0x0800 && a < 0x2000) {   // prosport_charram_r: Pro Golf's set at +0x800
			const int o = fg_off((uint16_t)(a - 0x800));
			return o < 0 ? 0 : s_fg[3][o + 0x800];
		}
		if (a >= 0x2400 && a < 0x3000) return s_ram3[a - 0x2400];
		if (a >= 0x3000 && a < 0x3400) return s_cram[a & 0x3ff];
		if (a >= 0x3400 && a < 0x3800) return s_vram[a & 0x3ff];
		if (a >= 0x3800 && a < 0x4000) return s_spriteram[a & 0x7ff];
		if (s_io_sel && a >= 0x8000 && a < 0x8010) return deco16_io_r(a - 0x8000);
		return a >= 0x4000 ? s_rom[a] : 0;
	}
	// prosoccr_map
	if (a < 0x1000) return s_ram[a];
	if (a >= 0x2000 && a < 0x4000) return s_rom[0xa000 + (a - 0x2000)];
	if (s_io_sel && a >= 0x4000 && a < 0x8010)
		return a < 0x8000 ? s_user1[rbx_wrap((a - 0x4000), s_user1_size)] : deco16_io_r(a - 0x8000);
	if (a >= 0x4000 && a < 0x4400) return s_cram[a & 0x3ff];
	if (a >= 0x4800 && a < 0x4c00) return s_cram[a & 0x3ff];
	if (a >= 0x4400 && a < 0x4800) return s_vram[a & 0x3ff];
	if (a >= 0x4c00 && a < 0x5000) return s_spriteram[a & 0x3ff];
	if (a >= 0x6200 && a < 0x6800) return s_ram2[a - 0x6200];
	if (a >= 0x8000 && a < 0x9800) {   // prosoccr_charram_r
		const uint16_t o = (uint16_t)(a - 0x8000);
		if (s_readback) {
			const int f = fg_off(o);
			return f < 0 ? 0 : s_shared[f];
		}
		return s_charram[o];
	}
	return a >= 0xa000 ? s_rom[a] : 0;
}
static void pro_w(uint16_t a, uint8_t v)
{
	if (s_k == K_PROSPORT) {
		const uint16_t m = a & 0xdfff;
		if (m < 0x200 && a < 0x2400) s_ram[m] = v;
		else if (m >= 0x200 && m < 0x220 && a < 0x2400) {   // palette_device::write8, BGR_233_inverted
			const uint8_t d = (uint8_t)~v;
			s_palram[m - 0x200] = v;
			s_rgb[m - 0x200] = (uint32_t)(((d & 7) << 5 | (d & 7) << 2 | (d & 7) >> 1) << 16 |
			                              (((d >> 3) & 7) << 5 | ((d >> 3) & 7) << 2 | ((d >> 3) & 7) >> 1) << 8 |
			                              ((d >> 6) & 3) * 0x55);
		} else if (m >= 0x220 && m < 0x400 && a < 0x2400) s_ram[m] = v;
		else if (a >= 0x0400 && a < 0x0800) s_bgram[a - 0x400] = v;
		else if (a >= 0x0800 && a < 0x2000) {   // prosport_charram_w
			const int o = fg_off((uint16_t)(a - 0x800));
			if (o >= 0)
				s_fg[3][o + 0x800] = v;
		} else if (a >= 0x2400 && a < 0x3000) s_ram3[a - 0x2400] = v;
		else if (a >= 0x3000 && a < 0x3400) s_cram[a & 0x3ff] = v;
		else if (a >= 0x3400 && a < 0x3800) s_vram[a & 0x3ff] = v;
		else if (a >= 0x3800 && a < 0x4000) s_spriteram[a & 0x7ff] = v;
		else if (a >= 0x8000 && a < 0x8010) deco16_io_w(a - 0x8000, v);
		return;
	}
	if (a < 0x1000) s_ram[a] = v;
	else if (s_io_sel && a >= 0x8000 && a < 0x8010) deco16_io_w(a - 0x8000, v);   // (the view's variant: rw)
	else if (a >= 0x4000 && a < 0x4400) s_cram[a & 0x3ff] = v;
	else if (a >= 0x4800 && a < 0x4c00) s_cram[a & 0x3ff] = v;
	else if (a >= 0x4400 && a < 0x4800) s_vram[a & 0x3ff] = v;
	else if (a >= 0x4c00 && a < 0x5000) s_spriteram[a & 0x3ff] = v;
	else if (a >= 0x6200 && a < 0x6800) s_ram2[a - 0x6200] = v;
	else if (a >= 0x8000 && a < 0x9800) {   // prosoccr_charram_w (the readback never set)
		const uint16_t o = (uint16_t)(a - 0x8000);
		s_charram[o + s_readback * 0x1800] = v;
		const int f = fg_off(o);
		if (f >= 0)
			s_fg[0][f] = v;
	} else if (a == 0x9800) s_readback = v & 1;
}

static uint8_t main_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (s_k >= K_PROSOCCR)
		return pro_r(a);
	if (a < 0x1000) {
		if (s_k == K_LIBERATB && a == 0x00fe) return in0_r();
		return s_ram[a];
	}
	if (a < 0x4000) return s_rom[0x9000 + (a - 0x1000)];   // (mirror of the main ROM)
	if (s_k == K_LIBERATB) {
		if (a >= 0xf000 && a < 0xf005) return deco16_io_r(a - 0xf000);
	} else if (s_io_sel && a < 0x8010) {   // the view
		if (a < 0x8000) return s_user1[rbx_wrap((a - 0x4000), s_user1_size)];
		return deco16_io_r(a - 0x8000);
	}
	if (a < 0x4400) return s_cram[a & 0x3ff];
	if (a < 0x4800) return s_vram[a & 0x3ff];
	if (a < 0x5000) return s_spriteram[a & 0x7ff];
	if (a >= 0x6200 && a < 0x6800) return s_ram2[a - 0x6200];
	if (a >= 0x8000) {
		if (s_yellowcb && a == 0xa000) return in0_r();   // init_yellowcb: install_read_port(0xa000, "IN0")
		return s_rom[a];
	}
	return 0;
}
static void main_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (s_k >= K_PROSOCCR) {
		pro_w(a, v);
		return;
	}
	if (a < 0x1000) { s_ram[a] = v; return; }
	if (s_k == K_LIBERATB) {
		if (a >= 0xf000 && a < 0xf010) { deco16_io_w(a - 0xf000, v); return; }
	} else if (a >= 0x8000 && a < 0x8010) {   // (the view's variant keeps the writes: MAME's "what was there before")
		deco16_io_w(a - 0x8000, v);
		return;
	}
	if (a >= 0x4000 && a < 0x4400) s_cram[a & 0x3ff] = v;
	else if (a >= 0x4400 && a < 0x4800) s_vram[a & 0x3ff] = v;
	else if (a >= 0x4800 && a < 0x5000) s_spriteram[a & 0x7ff] = v;
	else if (a >= 0x6200 && a < 0x6800) s_ram2[a - 0x6200] = v;
}

// the DECO CPU-16's I/O space: port 0 IN0 / the view's switch (bit 0; kamikcab: bit 2), port 1 TILT
static uint8_t io_r(void *ctx, uint8_t a)
{
	(void)ctx;
	if (a == 0) return in0_r();
	if (a == 1) return (uint8_t)rbx_port_read(s_port_tilt);
	return 0;
}
static void io_w(void *ctx, uint8_t a, uint8_t v)
{
	(void)ctx;
	if (a == 0)
		s_io_sel = (v >> (s_k == K_KAMIKCAB ? 2 : 0)) & 1;
}

// deco16_interrupt: at vblank, a coin or the service (IN3 0x43, active low) raises the IRQ once
static void vbl_irq(void)
{
	const int p = ~rbx_port_read(s_port_in[3]);
	if ((p & 0x43) && !s_latch) {
		m6502_set_irq(&s_cpu, 1);
		s_latch = 1;
	} else if (!(p & 0x43))
		s_latch = 0;
}

// ---- video ----
static uint16_t (*s_scr_hp)[256 * 256];
#define s_scr (*s_scr_hp)

static const uint8_t *elem(const rbx_gfx *g, uint32_t code) { return g->pix + (size_t)rbx_wrap(code, g->count) * g->width * g->height; }

static void blit(const uint8_t *pix, int w, int h, int base, int fx, int fy, int sx, int sy, uint32_t opaque_mask)
{
	for (int y = 0; y < h; y++) {
		const int dy = sy + y;
		if (dy < 0 || dy > 255)
			continue;
		const uint8_t *row = pix + (fy ? h - 1 - y : y) * w;
		for (int x = 0; x < w; x++) {
			const int dx = sx + x;
			if (dx < 0 || dx > 255)
				continue;
			const uint8_t p = row[fx ? w - 1 - x : x];
			if ((opaque_mask >> p) & 1)
				s_scr[dy * 256 + dx] = (uint16_t)(base + p);
		}
	}
}

// the background (get_back_tile_info with back_scan, 512x512, scrolled; flip: the whole map mirrored); mask: the pens
// drawn
static void draw_back(const snap_t *s, uint32_t mask)
{
	const int sx0 = -s->io_ram[0], sy0 = s->io_ram[1];
	const uint16_t base = (uint16_t)(s->bg_color * 8);
	for (int y = 0; y < 256; y++) {
		const int ty = s->flip ? 511 - ((y - (256 - 512 + sy0)) & 511) : (y + sy0) & 511;
		const int row = ty >> 4;
		uint16_t *d = s_scr + y * 256;
		for (int x = 0; x < 256;) {   // (a run of pixels in one tile: its lookup once)
			const int tx = s->flip ? 511 - ((x - (256 - 512 + sx0)) & 511) : (x + sx0) & 511;
			const int col = tx >> 4;
			int idx = (row & 0xf) + ((15 - (col & 0xf)) << 4) + ((row & 0x10) << 5) + ((col & 0x10) << 4);
			// the 512x512 map's four quarters from the pages io_ram[2..5]
			const int page = (idx & 0x100) ? ((idx & 0x200) ? s->io_ram[5] : s->io_ram[4]) : ((idx & 0x200) ? s->io_ram[3] : s->io_ram[2]);
			idx = (idx & 0xff) + (page << 8);
			const uint8_t tile = s_user1[rbx_wrap((uint32_t)idx, s_user1_size)];
			const rbx_gfx *g = tile > 0x7f ? &s_gfxt2 : &s_gfxt1;
			const uint8_t *src = elem(g, tile & 0x7f) + (ty & 15) * 16;
			int n = s->flip ? (tx & 15) + 1 : 16 - (tx & 15);
			if (n > 256 - x) n = 256 - x;
			for (int k = 0; k < n; k++) {
				const uint8_t p = src[s->flip ? (tx & 15) - k : (tx & 15) + k];
				if ((mask >> p) & 1)
					d[x + k] = (uint16_t)(base + p);
			}
			x += n;
		}
	}
}

static void draw_fix(const snap_t *s)   // get_fix_tile_info with fix_scan, pen 0 transparent
{
	for (int y = 0; y < 256; y++) {
		const int ty = s->flip ? 255 - y : y;
		uint16_t *d = s_scr + y * 256;
		for (int x = 0; x < 256; x += 8) {   // (a tile's 8 pixels: one lookup)
			const int tx = s->flip ? 255 - x : x;
			const int idx = ((ty >> 3) & 0x1f) + ((31 - ((tx >> 3) & 0x1f)) << 5);
			const int code = s->vram[idx] + (s->cram[idx] << 8), color = (s->cram[idx] & 0x70) >> 4;
			const uint8_t *src = elem(&s_gfxchar, (uint32_t)code) + (ty & 7) * 8;
			for (int k = 0; k < 8; k++) {
				const uint8_t p = src[s->flip ? 7 - k : k];
				if (p)
					d[x + k] = (uint16_t)(color * 8 + p);
			}
		}
	}
}

static void liberate_sprites(const snap_t *s)
{
	const uint8_t *sp = s->spriteram;
	for (int offs = 0; offs < 0x800; offs += 4) {
		const int code = sp[offs + 1] + ((sp[offs] & 0x60) << 3);
		int sx = 240 - sp[offs + 3], sy = 240 - sp[offs + 2], sy2;
		const int color = (sp[offs + 1] & 0x08) >> 3;
		int fx = (sp[offs] & 4) != 0, fy = (sp[offs] & 2) != 0;
		const int multi = sp[offs] & 0x10;
		if (multi && !fy)
			sy -= 16;
		if (s->flip) {
			sy = 240 - sy;
			sx = 240 - sx;
			sy2 = fy ? sy + 16 : sy - 16;
			fx = !fx;
			fy = !fy;
		} else
			sy2 = fy ? sy - 16 : sy + 16;
		blit(elem(&s_gfxspr, (uint32_t)code), 16, 16, color * 8, fx, fy, sx, sy, 0xfe);
		if (multi)
			blit(elem(&s_gfxspr, (uint32_t)code + 1), 16, 16, color * 8, fx, fy, sx, sy2, 0xfe);
	}
}

static void boomrang_sprites(const snap_t *s, int pri)
{
	const uint8_t *sp = s->spriteram;
	for (int offs = 0; offs < 0x800; offs += 4) {
		if ((sp[offs] & 1) != 1 || (sp[offs] & 8) != pri)
			continue;
		int code = sp[offs + 1] + ((sp[offs] & 0xe0) << 3), code2 = code + 1;
		const int multi = sp[offs] & 0x10;
		int sy = sp[offs + 2];
		if (multi)
			sy += 16;
		int sx = 240 - sp[offs + 3], sy2;
		sy = 240 - sy;
		const int color = (sp[offs] & 4) >> 2;
		int fx = 0, fy = (sp[offs] & 2) != 0;
		if (fy && multi) {
			code2 = code;
			code++;
		}
		if (s->flip) {
			sy = 240 - sy;
			sx = 240 - sx;
			fx = !fx;
			fy = !fy;
			sy2 = sy - 16;
		} else
			sy2 = sy + 16;
		blit(elem(&s_gfxspr, (uint32_t)code), 16, 16, color * 8, fx, fy, sx, sy, 0xfe);
		if (multi)
			blit(elem(&s_gfxspr, (uint32_t)code2), 16, 16, color * 8, fx, fy, sx, sy2, 0xfe);
	}
}

// the text / sprite graphics of a 0x6000-byte set, decoded from its bytes (gfx_8x8x3_planar, sprites)
static uint8_t s_dyn8[1024 * 64], s_dyn16[256 * 256];
static void decode_set(const uint8_t *r, int sprites)
{
	for (int c = 0; c < 1024; c++)
		for (int y = 0; y < 8; y++) {
			const int o = c * 8 + y;
			for (int x = 0; x < 8; x++) {
				const int b = 7 - x;
				s_dyn8[c * 64 + y * 8 + x] = (uint8_t)(((r[0x4000 + o] >> b) & 1) << 2 | ((r[0x2000 + o] >> b) & 1) << 1 | ((r[o] >> b) & 1));
			}
		}
	if (sprites)
		for (int c = 0; c < 256; c++)
			for (int y = 0; y < 16; y++)
				for (int x = 0; x < 16; x++) {
					const int o = c * 32 + (x < 8 ? 16 : 0) + y, b = 7 - (x & 7);
					s_dyn16[c * 256 + y * 16 + x] = (uint8_t)(((r[0x4000 + o] >> b) & 1) << 2 | ((r[0x2000 + o] >> b) & 1) << 1 | ((r[o] >> b) & 1));
				}
}

static void prosport_render(const snap_t *s)
{
	for (int i = 0; i < 256 * 256; i++)
		s_scr[i] = 0;
	// the background (prosport_get_back_tile_info, back_scan; scroll y (io0 bit 3) << 5, x (io0 bit 1) << 7 | io1)
	const int scrolly = (s->io_ram[0] & 8) << 5, scrollx = -(((s->io_ram[0] & 2) << 7) | s->io_ram[1]);
	for (int y = 0; y < 256; y++) {
		const int ty = s->flip ? 511 - ((y - (256 - 512 + scrolly)) & 511) : (y + scrolly) & 511;
		for (int x = 0; x < 256; x++) {
			const int tx = s->flip ? 511 - ((x - (256 - 512 + scrollx)) & 511) : (x + scrollx) & 511;
			const int col = tx >> 4, row = ty >> 4;
			const int idx = (row & 0xf) + ((15 - (col & 0xf)) << 4) + ((row & 0x10) << 5) + ((col & 0x10) << 4);
			int tile = (s->bgram[idx & 0x3ff] & 0xf0) >> 4;
			if (idx & 8)
				tile += 0x10;
			tile += s->io_ram[0] & 0x20;
			s_scr[y * 256 + x] = elem(&s_gfxpro, (uint32_t)tile)[(ty & 15) * 16 + (tx & 15)];
		}
	}
	for (int offs = 0; offs < 0x400; offs++) {   // the text: transpen, colour 1
		const int tile = s->vram[offs] + ((s->cram[offs] & 3) << 8);
		blit(s_dyn8 + (tile & 0x3ff) * 64, 8, 8, 8, 0, 0, 248 - 8 * (offs / 32), 8 * (offs % 32), 0xfe);
	}
	const uint8_t *sp = s->spriteram;   // prosport_draw_sprites (multi off)
	for (int offs = 0; offs < 0x800; offs += 4) {
		if ((sp[offs] & 1) != 1)
			continue;
		const int code = sp[offs + 1] + ((sp[offs] & 3) << 8);
		int sy = sp[offs + 2];
		if (sp[offs] & 0x10)
			sy += 16;
		int sx = 240 - sp[offs + 3];
		sy = 240 - sy;
		int fy = (sp[offs] & 2) != 0, fx = (sp[offs] & 4) != 0;
		if (s->flip) {
			sy = 240 - sy;
			sx = 240 - sx;
			fx = !fx;
			fy = !fy;
		}
		blit(s_dyn16 + (code & 0xff) * 256, 16, 16, 8, fx, fy, sx, sy, 0xfe);
	}
}

static void prosoccr_render(const snap_t *s)
{
	if (s->bg_disable)
		for (int i = 0; i < 256 * 256; i++)
			s_scr[i] = 32;
	else {   // get_back_tile_info (the bg colour 0; bg_gfx from colour 8)
		const int sx0 = -s->io_ram[0], sy0 = s->io_ram[1];
		for (int y = 0; y < 256; y++) {
			const int ty = (y + sy0) & 511;
			for (int x = 0; x < 256; x++) {
				const int tx = (x + sx0) & 511;
				const int col = tx >> 4, row = ty >> 4;
				int idx = (row & 0xf) + ((15 - (col & 0xf)) << 4) + ((row & 0x10) << 5) + ((col & 0x10) << 4);
				const int page = (idx & 0x100) ? ((idx & 0x200) ? s->io_ram[5] : s->io_ram[4]) : ((idx & 0x200) ? s->io_ram[3] : s->io_ram[2]);
				idx = (idx & 0xff) + (page << 8);
				const uint8_t tile = s_user1[rbx_wrap((uint32_t)idx, s_user1_size)];
				s_scr[y * 256 + x] = (uint16_t)(8 + elem(&s_gfxt1, tile & 0x7f)[(ty & 15) * 16 + (tx & 15)]);
			}
		}
	}
	for (int y = 0; y < 256; y++)   // the text (fix_scan, from the RAM-written set)
		for (int x = 0; x < 256; x++) {
			const int idx = ((y >> 3) & 0x1f) + ((31 - ((x >> 3) & 0x1f)) << 5);
			const int code = s->vram[idx] + (s->cram[idx] << 8), color = (s->cram[idx] & 0x70) >> 4;
			const uint8_t p = s_dyn8[(code & 0x3ff) * 64 + (y & 7) * 8 + (x & 7)];
			if (p)
				s_scr[y * 256 + x] = (uint16_t)(color * 8 + p);
		}
	const uint8_t *sp = s->spriteram;   // prosoccr_draw_sprites
	for (int offs = 0; offs < 0x400; offs += 4) {
		if ((sp[offs] & 1) != 1)
			continue;
		blit(elem(&s_gfxspr, sp[offs + 1]), 16, 16, 0, (sp[offs] & 4) != 0, (sp[offs] & 2) != 0, 240 - sp[offs + 3],
		     240 - sp[offs + 2], 0xfe);
	}
}

static void lb_render(void)
{
	const snap_t *s = s_snap;
	for (int i = 0; i < 256; i++)
		rbx_video.palette[i] = s->rgb[i];
	if (s_k == K_PROSOCCR || s_k == K_PROSPORT) {
		decode_set(s->fg, s_k == K_PROSPORT);
		if (s_k == K_PROSPORT)
			prosport_render(s);
		else
			prosoccr_render(s);
	} else
	if (s_k == K_BOOMRANG || s_k == K_KAMIKCAB) {   // screen_update_boomrang
		if (s->bg_disable) {
			for (int i = 0; i < 256 * 256; i++)
				s_scr[i] = 32;
		} else
			draw_back(s, 0x81);   // TILEMAP_DRAW_LAYER1 (transmask bgmask 0x7e: pens 0 and 7)
		boomrang_sprites(s, 8);
		if (!s->bg_disable)
			draw_back(s, 0xfe);   // TILEMAP_DRAW_LAYER0 (fgmask 0x01: pens 1-7)
		boomrang_sprites(s, 0);
		draw_fix(s);
	} else if (1) {   // screen_update_liberate
		if (s->bg_disable) {
			for (int i = 0; i < 256 * 256; i++)
				s_scr[i] = 32;
		} else
			draw_back(s, 0xff);
		liberate_sprites(s);
		draw_fix(s);
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = 0; y < s_vh; y++)
		memcpy(bm + y * pitch, s_scr + (s_vy0 + y) * 256 + s_vx0, (size_t)s_vw * 2);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	snap_t *s = s_snap;
	memcpy(s->cram, s_cram, sizeof(s_cram));
	memcpy(s->vram, s_vram, sizeof(s_vram));
	memcpy(s->spriteram, s_spriteram, sizeof(s_spriteram));
	memcpy(s->io_ram, s_io_ram, sizeof(s_io_ram));
	s->bg_color = (uint8_t)s_bg_color;
	s->bg_disable = (uint8_t)s_bg_disable;
	s->flip = (uint8_t)s_flip;
	memcpy(s->rgb, s_rgb, sizeof(s_rgb));
	if (s_k == K_PROSOCCR)
		memcpy(s->fg, s_fg[0], 0x6000);
	else if (s_k == K_PROSPORT) {   // (the set in use: io0 bit 6 Pro Golf's, else bits 4-5)
		memcpy(s->fg, s_fg[(s_io_ram[0] & 0x40) ? 3 : (s_io_ram[0] & 0x30) >> 4], 0x6000);
		memcpy(s->bgram, s_bgram, sizeof(s_bgram));
	}
	s->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void lb_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_ram2, 0, sizeof(s_ram2));
	memset(s_cram, 0, sizeof(s_cram));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_io_ram, 0, sizeof(s_io_ram));
	memset(s_charram, 0, sizeof(s_charram));
	memset(s_bgram, 0, sizeof(s_bgram));
	memset(s_palram, 0, sizeof(s_palram));
	memset(s_ram3, 0, sizeof(s_ram3));
	s_readback = 0;
	memcpy(s_rgb, s_rgb_init, sizeof(s_rgb));
	for (int i = 0; i < 4; i++)   // (the RAM-written sets back to their ROM)
		if (s_fg[i])
			memcpy(s_fg[i], s_fg_rom[i], 0x6000);
	s_io_sel = 0;
	s_latch = s_bg_color = s_bg_disable = s_flip = 0;
	memset(s_sram, 0, sizeof(s_sram));
	s_slatch = 0;
	ay8910_reset(&s_ay[0]);
	ay8910_reset(&s_ay[1]);
	m6502_reset(&s_cpu);
	m6502_set_irq(&s_cpu, 0);
	m6502_set_nmi(&s_cpu, 0);
	m6502_reset(&s_snd);
	m6502_set_irq(&s_snd, 0);
	m6502_set_nmi(&s_snd, 0);
	s_line = 0;
	s_frame = 0;
	s_cpu_target = s_snd_target = 0;
	s_cpu_frac = s_snd_frac = s_ay_frac = s_nmi_frac = 0;
}

static void lb_exit(void);
static int lb_common(const rbx_game *g);

// Pro Soccer / Pro Sports: their regions and graphics
static int pro_init(const rbx_game *g)
{
	uint32_t ps = 0, sz;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("audiocpu", &s_srom_size);
	s_user1 = rbx_region("user1", &s_user1_size);
	const uint8_t *prom = rbx_region("proms", &ps);
	if (!s_rom || s_rom_size < 0x10000 || !s_srom || s_srom_size < 0x10000 || !s_user1) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	for (uint32_t i = 0; i < 0x10000; i++)   // init_prosport: the nibbles swapped
		s_rom[i] = (uint8_t)((s_rom[i] << 4) | (s_rom[i] >> 4));
	static const char *const sets[4] = { "prosport_fg_gfx", "protenns_fg_gfx", "probowl_fg_gfx", "progolf_fg_gfx" };
	memset(s_rgb, 0, sizeof(s_rgb));
	int rc = 0;
	if (s_k == K_PROSOCCR) {
		uint32_t sps, bgs;
		s_fg_rom[0] = rbx_region("fg_gfx", &sz);
		s_shared = rbx_region("shared_gfx", &sz);
		const uint8_t *sp = rbx_region("sp_gfx", &sps), *bg = rbx_region("bg_gfx", &bgs);
		if (!s_fg_rom[0] || !s_shared || !sp || !bg || !prom) {
			rbx_fail("%s: ROM regions missing", g->set);
			return -1;
		}
		s_fg[0] = (uint8_t *)malloc(0x6000);
		static uint32_t xs[16], ys[16], xt[16], yt[16];
		static const uint32_t xtt[16] = { 24, 25, 26, 27, 16, 17, 18, 19, 8, 9, 10, 11, 0, 1, 2, 3 };
		for (int i = 0; i < 8; i++) {
			xs[i] = (uint32_t)(16 * 8 + i);
			xs[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++) {
			ys[i] = (uint32_t)(i * 8);
			xt[i] = xtt[i];
			yt[i] = (uint32_t)(i * 32);
		}
		const uint32_t t = sps / 3 * 8, h = bgs / 2 * 8;
		const uint32_t p3[3] = { 2 * t, t, 0 }, pb[3] = { 4, 0, h + 4 };   // (prosoccr_bg_gfx)
		const rbx_gfx_layout ls = { 16, 16, 3, p3, xs, ys, 16 * 16 }, lb = { 16, 16, 3, pb, xt, yt, 64 * 8 };
		rc = !s_fg[0] || rbx_gfx_decode(&s_gfxspr, &ls, sp, sps / 3) || rbx_gfx_decode(&s_gfxt1, &lb, bg, bgs / 2);
		for (int i = 0; i < 32 && (uint32_t)i < ps; i++) {   // liberate_palette
			const uint8_t p = prom[i];
			s_rgb[i] = (uint32_t)((0x21 * (p & 1) + 0x47 * ((p >> 1) & 1) + 0x97 * ((p >> 2) & 1)) << 16 |
			                      (0x21 * ((p >> 3) & 1) + 0x47 * ((p >> 4) & 1) + 0x97 * ((p >> 5) & 1)) << 8 |
			                      (0x52 * ((p >> 6) & 1) + 0xad * ((p >> 7) & 1)));
		}
	} else {
		uint32_t g2s;
		const uint8_t *gfx2 = rbx_region("gfx2", &g2s);
		for (int i = 0; i < 4; i++) {
			s_fg_rom[i] = rbx_region(sets[i], &sz);
			if (!s_fg_rom[i] || sz < 0x6000)
				rc = 1;
		}
		if (rc || !gfx2) {
			rbx_fail("%s: ROM regions missing", g->set);
			return -1;
		}
		for (int i = 0; i < 4; i++)
			s_fg[i] = (uint8_t *)malloc(0x6000);
		static uint32_t xp[16], yp[16];
		static const uint32_t xpp[16] = { 384, 385, 386, 387, 256, 257, 258, 259, 128, 129, 130, 131, 0, 1, 2, 3 };
		for (int i = 0; i < 16; i++) {
			xp[i] = xpp[i];
			yp[i] = (uint32_t)(i * 8);
		}
		const uint32_t h = g2s / 2 * 8;
		const uint32_t pp[3] = { 4, 0, h + 4 };   // (pro_tiles)
		const rbx_gfx_layout lp = { 16, 16, 3, pp, xp, yp, 512 };
		rc = !s_fg[0] || !s_fg[1] || !s_fg[2] || !s_fg[3] || rbx_gfx_decode(&s_gfxpro, &lp, gfx2, g2s / 2);
	}
	if (rc) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		lb_exit();
		return -1;
	}
	memcpy(s_rgb_init, s_rgb, sizeof(s_rgb));
	return lb_common(g);
}

static int lb_init(const rbx_game *g)
{
	if (!s_opcodes_hp && !(s_opcodes_hp = calloc(1, sizeof(*s_opcodes_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_scr_hp && !(s_scr_hp = calloc(1, sizeof(*s_scr_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	const char *m = g->machine;
	s_k = !strcmp(m, "liberatb") ? K_LIBERATB : !strcmp(m, "boomrang") ? K_BOOMRANG : !strcmp(m, "kamikcab") ? K_KAMIKCAB
	    : !strcmp(m, "prosoccr") ? K_PROSOCCR : !strcmp(m, "prosport") ? K_PROSPORT : K_LIBERATE;
	s_vx0 = 0; s_vy0 = 8; s_vw = 256; s_vh = 240;
	s_vbl_lines = s_k == K_PROSPORT ? 24 : 9;
	s_cpu_clock = s_k == K_PROSOCCR ? 1250000 : 2000000;
	s_snd_clock = s_k == K_PROSOCCR ? 1250000 : s_k == K_PROSPORT ? 750000 : 1500000;
	if (s_k == K_PROSOCCR) { s_vx0 = 8; s_vy0 = 0; s_vw = 240; s_vh = 256; }
	if (s_k >= K_PROSOCCR)
		return pro_init(g);
	s_yellowcb = !strcmp(g->set, "yellowcbj") || !strcmp(g->set, "yellowcbb");
	uint32_t g1s, g2s, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("audiocpu", &s_srom_size);
	uint8_t *gfx1 = rbx_region("gfx1", &g1s), *gfx2 = rbx_region("gfx2", &g2s);
	s_user1 = rbx_region("user1", &s_user1_size);
	const uint8_t *prom = rbx_region("proms", &ps);
	if (!s_rom || s_rom_size < 0x10000 || !s_srom || s_srom_size < 0x10000 || !gfx1 || !gfx2 || !s_user1 || !prom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (s_k == K_LIBERATE)   // init_liberate: the opcodes' bits swapped (a decrypted copy)
		for (int a = 0; a < 0x8000; a++) {
			uint8_t v = s_rom[a + 0x8000];
			v = (uint8_t)((v & 0xd7) | ((v & 0x08) << 2) | ((v & 0x20) >> 2));
			v = (uint8_t)((v & 0xbb) | ((v & 0x04) << 4) | ((v & 0x40) >> 4));
			v = (uint8_t)((v & 0x7d) | ((v & 0x02) << 6) | ((v & 0x80) >> 6));
			s_opcodes[a] = v;
		}
	else   // init_prosport: the program's nibbles swapped
		for (uint32_t i = 0; i < 0x10000; i++)
			s_rom[i] = (uint8_t)((s_rom[i] << 4) | (s_rom[i] >> 4));
	{	// gfx_8x8x3_planar, sprites (RGN_FRAC(1,3)); tiles1 / tiles2 (128 each)
		static uint32_t x8[8], y8[8], xs[16], ys[16], xt[16], yt[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 8);
			xs[i] = (uint32_t)(16 * 8 + i);
			xs[8 + i] = (uint32_t)i;
		}
		static const uint32_t xtt[16] = { 24, 25, 26, 27, 16, 17, 18, 19, 8, 9, 10, 11, 0, 1, 2, 3 };
		for (int i = 0; i < 16; i++) {
			ys[i] = (uint32_t)(i * 8);
			xt[i] = xtt[i];
			yt[i] = (uint32_t)(i * 32);
		}
		const uint32_t t = g1s / 3 * 8;
		const uint32_t p3[3] = { 2 * t, t, 0 };
		const uint32_t p1[3] = { 4, 0, 0x4000 * 8 + 4 }, p2[3] = { 0x2000 * 8 + 4, 0x2000 * 8, 0x4000 * 8 };
		const rbx_gfx_layout l8 = { 8, 8, 3, p3, x8, y8, 8 * 8 };
		const rbx_gfx_layout ls = { 16, 16, 3, p3, xs, ys, 16 * 16 };
		const rbx_gfx_layout lt1 = { 16, 16, 3, p1, xt, yt, 64 * 8 }, lt2 = { 16, 16, 3, p2, xt, yt, 64 * 8 };
		if (rbx_gfx_decode(&s_gfxchar, &l8, gfx1, g1s / 3) || rbx_gfx_decode(&s_gfxspr, &ls, gfx1, g1s / 3) ||
		    rbx_gfx_decode(&s_gfxt1, &lt1, gfx2, 128 * 64) || rbx_gfx_decode(&s_gfxt2, &lt2, gfx2, 128 * 64)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			lb_exit();
			return -1;
		}
	}
	memset(s_rgb, 0, sizeof(s_rgb));   // liberate_palette: 32 from the PROM, black at 32 (and past it)
	for (int i = 0; i < 32 && (uint32_t)i < ps; i++) {
		const uint8_t p = prom[i];
		const int r = 0x21 * (p & 1) + 0x47 * ((p >> 1) & 1) + 0x97 * ((p >> 2) & 1);
		const int gg = 0x21 * ((p >> 3) & 1) + 0x47 * ((p >> 4) & 1) + 0x97 * ((p >> 5) & 1);
		const int b = 0x52 * ((p >> 6) & 1) + 0xad * ((p >> 7) & 1);
		s_rgb[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	memcpy(s_rgb_init, s_rgb, sizeof(s_rgb));
	return lb_common(g);
}

// the sound, the CPUs, the ports (all the machines)
static int lb_common(const rbx_game *g)
{
	s_snap = (snap_t *)calloc(1, sizeof(*s_snap));
	// AY-3-8912 1.5 MHz 0.30 and 0.50 (MAME's default legacy output)
	ay8910_init(&s_ay[0], AY8910_TYPE_AY8910, 1500000, AY8910_LEGACY_OUTPUT, 77, 4, 0);
	ay8910_init(&s_ay[1], AY8910_TYPE_AY8910, 1500000, AY8910_LEGACY_OUTPUT, 128, 4, 0);
	s_mix_cap = (int)(ay8910_rate(&s_ay[0]) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_snap || !s_mix || rbx_audio_setup(ay8910_rate(&s_ay[0])) || rbx_video_setup(s_vw, s_vh, 256)) {
		rbx_fail("%s: out of memory", g->set);
		lb_exit();
		return -1;
	}
	m6502_init(&s_cpu, s_k == K_LIBERATB ? M6502_NMOS : M6502_DECO16);
	const uint32_t rom_from = s_k == K_PROSOCCR ? 0xa0 : s_k == K_PROSPORT ? 0x81 : 0x81;   // (whole ROM pages)
	s_cpu.read = main_r;
	s_cpu.write = main_w;
	s_cpu.io_read = io_r;
	s_cpu.io_write = io_w;
	if (s_k == K_LIBERATE) {
		s_cpu.op_rom = s_opcodes;
		s_cpu.op_start = 0x8000;
	}
	for (uint32_t p = rom_from; p < 0x100; p++)   // (the ROM's pages above the I/O registers' page)
		if (!(s_yellowcb && p == 0xa0) && !(s_k == K_LIBERATB && p == 0xf0))
			s_cpu.rpage[p] = s_rom + p * 256;
	if (s_k == K_PROSPORT)
		for (uint32_t p = 0x40; p < 0x80; p++)
			s_cpu.rpage[p] = s_rom + p * 256;
	m6502_init(&s_snd, M6502_NMOS);
	s_snd.deco222 = 1;
	s_snd.irq_hold = 1;   // (the latch: HOLD_LINE)
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	static const char *const ins[4] = { "IN0", "IN1", "IN2", "IN3" };
	for (int i = 0; i < 4; i++)
		s_port_in[i] = rbx_port_find(ins[i]);
	s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	s_port_tilt = rbx_port_find("TILT");
	lb_reset();
	return 0;
}

static void lb_exit(void)
{
	for (int i = 0; i < 4; i++) {
		free(s_fg[i]);
		s_fg[i] = NULL;
	}
	rbx_gfx_free(&s_gfxpro);
	rbx_gfx_free(&s_gfxchar);
	rbx_gfx_free(&s_gfxspr);
	rbx_gfx_free(&s_gfxt1);
	rbx_gfx_free(&s_gfxt2);
	free(s_snap);
	s_snap = NULL;
	free(s_mix);
	s_mix = NULL;
	free(s_opcodes_hp);
	s_opcodes_hp = NULL;
	free(s_scr_hp);
	s_scr_hp = NULL;
}

static void lb_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == 248) {
			vblank_capture();
			vbl_irq();
		}
		s_cpu_target += rbx_line_cycles(&s_cpu_frac, s_cpu_clock, FRAME_MHZ, LINES);
		m6502_run_until(&s_cpu, s_cpu_target);
		sound_line();
	}
	s_frame++;
}

static int lb_state(void *ctx, int load)
{
	uint8_t cpu[M6502_STATE_SIZE], snd[M6502_STATE_SIZE];
	if (!load) {
		memcpy(cpu, &s_cpu, M6502_STATE_SIZE);
		memcpy(snd, &s_snd, M6502_STATE_SIZE);
	}
	int64_t tg[2] = { s_cpu_target, s_snd_target };
	uint64_t fr[4] = { s_cpu_frac, s_snd_frac, s_ay_frac, s_nmi_frac };
	int32_t misc[10] = { (int32_t)s_frame, s_io_sel, s_latch, s_bg_color, s_bg_disable, s_flip, s_slatch, 0, 0, 0 };
	uint8_t ay[2][256];
	const uint32_t ays = ay8910_state_size();
	if (!load)
		for (int i = 0; i < 2; i++)
			memcpy(ay[i], &s_ay[i], ays);
	int rc = 0;
	rc |= rbx_state_io(ctx, "cpu", cpu, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "snd", snd, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "ram2", s_ram2, sizeof(s_ram2));
	rc |= rbx_state_io(ctx, "cram", s_cram, sizeof(s_cram));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "io", s_io_ram, sizeof(s_io_ram));
	if (s_k >= K_PROSOCCR) {
		rc |= rbx_state_io(ctx, "chr", s_charram, sizeof(s_charram));
		rc |= rbx_state_io(ctx, "bgr", s_bgram, sizeof(s_bgram));
		rc |= rbx_state_io(ctx, "palr", s_palram, sizeof(s_palram));
		rc |= rbx_state_io(ctx, "rgb", s_rgb, sizeof(s_rgb));
		rc |= rbx_state_io(ctx, "ram3", s_ram3, sizeof(s_ram3));
		rc |= rbx_state_io(ctx, "fgw", s_fg[s_k == K_PROSOCCR ? 0 : 3], 0x6000);
		rc |= rbx_state_io(ctx, "rdb", &s_readback, 1);
	}
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_state_io(ctx, "ay", ay, sizeof(ay));
	rc |= rbx_state_io(ctx, "tg", tg, sizeof(tg));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		memcpy(&s_cpu, cpu, M6502_STATE_SIZE);
		memcpy(&s_snd, snd, M6502_STATE_SIZE);
		for (int i = 0; i < 2; i++)
			memcpy(&s_ay[i], ay[i], ays);
		s_cpu_target = tg[0];
		s_snd_target = tg[1];
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_ay_frac = fr[2];
		s_nmi_frac = fr[3];
		s_frame = (uint32_t)misc[0];
		s_io_sel = misc[1];
		s_latch = misc[2];
		s_bg_color = misc[3];
		s_bg_disable = misc[4];
		s_flip = misc[5];
		s_slatch = (uint8_t)misc[6];
	}
	return rc ? -1 : 0;
}

static int lb_save(void *ctx) { return lb_state(ctx, 0); }
static int lb_load(void *ctx) { return lb_state(ctx, 1); }
static int lb_supported(const char *machine)
{
	return !strcmp(machine, "liberate") || !strcmp(machine, "liberatb") || !strcmp(machine, "boomrang") ||
	       !strcmp(machine, "kamikcab") || !strcmp(machine, "prosoccr") || !strcmp(machine, "prosport");
}

const rbx_driver rbx_drv_liberate = { "DECO Liberation", lb_init, lb_exit, lb_reset, lb_frame, lb_save, lb_load,
                                      lb_supported, lb_render, NULL };
