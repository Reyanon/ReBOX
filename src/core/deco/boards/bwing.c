/*  Data East B-Wings / Zaviga. Ported from MAME src/mame/dataeast/bwing.cpp (BSD-3-Clause, copyright-holders:
    Acho A. Tang, Alex W. Jackson).
    Two 6809E at 1.5 MHz sharing 2 KB (their IRQs raised by each other's strobes; the coins on the main CPU's NMI), a
    text layer, two 64x64 maps of 16x16 tiles whose graphics are in RAM (written through a view over 0x2000-0x3fff),
    sprites (some doubled in height) under the text and, for colour 0, under the front map; a 64-entry 9-bit palette
    (one component latched through a register). The sound: a DECO CPU-16 (its ROM's nibbles swapped) with an NMI at
    1 kHz (masked, acknowledged), two AY-3-8912 and an 8-bit DAC.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../cpu/m6809/m6809.h"
#include "../../sound/ay8910.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define CLOCK 1500000            // 24 MHz / 16
#define FRAME_MHZ 57444          // set_raw(24 MHz / 4, 384, 0, 256, 272, 8, 248)
#define LINES 272
#define VIS_Y0 8
#define VIS_H 240

static uint8_t *s_rom[2], *s_srom;
static uint32_t s_rom_size[2], s_srom_size;
static uint8_t s_shared[0x800], s_ram[2][0x800], s_vram[0x400], s_ram14[0x400], s_spriteram[0x200];
static uint8_t s_pal[0x40], s_palext[0x40], s_ctrlram[8];
static uint8_t s_fgram[0x1000], s_bgram[0x1000], s_gfxram[0x6000];
static uint8_t s_sreg[8], s_palatch, s_mapmask, s_view;
static uint8_t s_snd_nmimask, s_u8f, s_latch;
static int s_coin_nmi;
static int s_port_dsw[2], s_port_in[3], s_port_vbl;
static int s_line;
static uint32_t s_frame;
static int64_t s_target[3];
static uint64_t s_frac[3], s_ay_frac, s_nmi_frac;
static m6809_t s_cpu[2];
static m6502_t s_snd;
static uint8_t s_sram[0x200];
static ay8910_t s_ay[2];
static int32_t *s_mix;
static int s_mix_cap;
static int32_t s_dac;
static rbx_gfx s_gfxchar, s_gfxspr;
static struct {
	uint8_t vram[0x400], spriteram[0x200], fgram[0x1000], bgram[0x1000], gfxram[0x6000], pal[0x40], palext[0x40];
	uint8_t sreg[8], mapmask;
} *s_snap;

// ---- sound ----
static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x200) return s_sram[a];
	if (a == 0xa000) return s_latch;
	if (a >= 0xe000) return s_srom[(a - 0xe000) % s_srom_size];
	return 0;
}
static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x200) s_sram[a] = v;
	else if (a == 0x0200) s_dac = ((int32_t)v - 128) * 256;   // DAC08
	else if (a == 0x1000) m6502_set_nmi(&s_snd, 0);           // sound_nmiack_w
	else if (a == 0x2000) ay8910_data_w(&s_ay[0], v);
	else if (a == 0x4000) ay8910_address_w(&s_ay[0], v);
	else if (a == 0x6000) ay8910_data_w(&s_ay[1], v);
	else if (a == 0x8000) ay8910_address_w(&s_ay[1], v);
	else if (a == 0xd000) s_snd_nmimask = (v >> 7) & 1;
}
static int vblank(void) { return s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H; }
static uint8_t snd_io_r(void *ctx, uint8_t a)
{
	(void)ctx;
	if (a == 0) {
		rbx_port_custom(s_port_vbl, 0xff, vblank() ? 0xff : 0);
		return (uint8_t)rbx_port_read(s_port_vbl);
	}
	return 0xff;
}
static void snd_io_w(void *ctx, uint8_t a, uint8_t v) { (void)ctx; if (a == 0) s_u8f = v; }

// ---- main CPUs ----
static void main_ctrl_w(int o, uint8_t v)
{
	switch (o) {
	case 0: m6809_set_irq(&s_cpu[1], 1); break;    // MSSTB
	case 1: m6809_set_irq(&s_cpu[0], 0); break;    // IRQACK
	case 2: m6809_set_firq(&s_cpu[0], 0); break;   // FIRQACK
	case 3: m6809_set_nmi(&s_cpu[0], 0); break;    // NMIACK
	case 5:   // SNDREQ (0x80: the protection trick, the sub CPU's NMI)
		if (v == 0x80)
			m6809_set_nmi(&s_cpu[1], 1);
		else {
			s_latch = v;
			m6502_set_irq(&s_snd, 1);   // (HOLD_LINE)
		}
		break;
	}
}

static uint8_t main_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x800) return s_shared[a];
	if (a < 0x1000) return s_ram[0][a - 0x800];
	if (a < 0x1400) return s_vram[a - 0x1000];
	if (a < 0x1800) return s_ram14[a - 0x1400];
	if (a < 0x1a00) return s_spriteram[a - 0x1800];
	if (a < 0x1a40) return s_pal[a - 0x1a00];
	if (a < 0x1a80) return s_palext[a - 0x1a40];
	if (a >= 0x1b00 && a < 0x1b05) {
		switch (a & 7) {
		case 0: return (uint8_t)rbx_port_read(s_port_dsw[0]);
		case 1: return (uint8_t)rbx_port_read(s_port_dsw[1]);
		case 2: return (uint8_t)rbx_port_read(s_port_in[0]);
		case 3: return (uint8_t)rbx_port_read(s_port_in[1]);
		case 4:
			rbx_port_custom(s_port_in[2], 0x80, vblank() ? 0x80 : 0);
			return (uint8_t)rbx_port_read(s_port_in[2]);
		}
	}
	if (a >= 0x1c00 && a < 0x1c08) return s_ctrlram[a & 7];
	if (a >= 0x2000 && a < 0x4000) {   // the view: the maps, else the gfx RAM's write-only windows (their RAM unread)
		if (s_view == 0) return a < 0x3000 ? s_fgram[a - 0x2000] : s_bgram[a - 0x3000];
		return 0;
	}
	return a >= 0x4000 ? s_rom[0][a] : 0;
}
static void main_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x800) s_shared[a] = v;
	else if (a < 0x1000) s_ram[0][a - 0x800] = v;
	else if (a < 0x1400) s_vram[a - 0x1000] = v;
	else if (a < 0x1800) s_ram14[a - 0x1400] = v;
	else if (a < 0x1a00) s_spriteram[a - 0x1800] = v;
	else if (a < 0x1a40) {   // paletteram_w: the RAM (bits 0x77) and the latched third component
		s_pal[a - 0x1a00] = v & 0x77;
		s_palext[a - 0x1a00] = s_palatch & 7;
	} else if (a >= 0x1b00 && a < 0x1b08) {   // scrollreg_w
		s_sreg[a & 7] = v;
		if ((a & 7) == 6)
			s_palatch = v;
		else if ((a & 7) == 7) {
			s_mapmask = v;
			s_view = v >> 6;
		}
	} else if (a >= 0x1c00 && a < 0x1c08) {
		s_ctrlram[a & 7] = v;
		main_ctrl_w(a & 7, v);
	} else if (a >= 0x2000 && a < 0x4000) {
		if (s_view == 0) {
			if (a < 0x3000) s_fgram[a - 0x2000] = v;
			else s_bgram[a - 0x3000] = v;
		} else
			s_gfxram[(s_view - 1) * 0x2000 + (a - 0x2000)] = v;
	}
}

static uint8_t sub_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x800) return s_shared[a];
	if (a < 0x1000) return s_ram[1][a - 0x800];
	return a >= 0xa000 ? s_rom[1][a] : 0;
}
static void sub_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x800) s_shared[a] = v;
	else if (a < 0x1000) s_ram[1][a - 0x800] = v;
	else if (a >= 0x1800 && a < 0x1804) {   // sub_ctrl_w
		switch (a & 3) {
		case 0: m6809_set_irq(&s_cpu[0], 1); break;    // SMSTB
		case 1: m6809_set_firq(&s_cpu[1], 0); break;
		case 2: m6809_set_irq(&s_cpu[1], 0); break;
		case 3: m6809_set_nmi(&s_cpu[1], 0); break;
		}
	}
}

// the coins on the main CPU's NMI (MAME coin_inserted: newval ? CLEAR : ASSERT)
static void coins(void)
{
	const int nmi = (rbx_port_read(s_port_in[2]) & 3) != 3;
	if (nmi != s_coin_nmi)
		m6809_set_nmi(&s_cpu[0], nmi);
	s_coin_nmi = nmi;
}

// ---- video ----
static uint16_t (*s_scr_hp)[256 * 256];
#define s_scr (*s_scr_hp)
static uint8_t (*s_pri_hp)[256 * 256];
#define s_pri (*s_pri_hp)
static uint8_t (*s_ramtile_hp)[2][128 * 256];   // the gfx RAM's two tile sets (ram_tilelayout)
#define s_ramtile (*s_ramtile_hp)
static uint8_t (*s_ramtile_src_hp)[0x6000];    // the gfx RAM they were decoded from (only changed tiles again)
#define s_ramtile_src (*s_ramtile_src_hp)
static int s_ramtile_ok;

static uint32_t pal3(int v) { return (uint32_t)((v << 5) | (v << 2) | (v >> 1)); }

// a 64x64 map of 16x16 tiles (scan_cols), flipped in x or y as a whole (MAME set_flip), pen 0 transparent unless
// opaque, the priority value ORed in
static void draw_map(const uint8_t *ram, const uint8_t *tiles, int base, int sx, int sy, int flipx, int flipy, int opaque,
                     uint8_t pri)
{
	for (int y = 0; y < VIS_H; y++) {
		const int sy_ = y + VIS_Y0;
		int ty = flipy ? 1023 - ((sy_ - (256 - 1024 + sy)) & 1023) : (sy_ + sy) & 1023;
		const int row = ty >> 4;
		uint16_t *d = s_scr + sy_ * 256;
		uint8_t *pr = s_pri + sy_ * 256;
		for (int x = 0; x < 256;) {   // (a run of pixels in one tile: its lookup once)
			const int tx = flipx ? 1023 - ((x - (256 - 1024 + sx)) & 1023) : (x + sx) & 1023;
			const int col = tx >> 4;
			const int idx = (row & 0xf) | ((col & 0xf) << 4) | ((row & 0x30) << 4) | ((col & 0x30) << 6);
			const uint8_t t = ram[idx];
			const uint8_t *src = tiles + (t & 0x7f) * 256 + (ty & 15) * 16;
			const uint16_t b = (uint16_t)(base + ((t >> 7) & 1) * 8);
			int n = flipx ? (tx & 15) + 1 : 16 - (tx & 15);
			if (n > 256 - x) n = 256 - x;
			for (int k = 0; k < n; k++) {
				const uint8_t p = src[flipx ? (tx & 15) - k : (tx & 15) + k];
				if (opaque || p) {
					d[x + k] = (uint16_t)(b + p);
					pr[x + k] |= pri;
				}
			}
			x += n;
		}
	}
}

static void bw_render(void)
{
	for (int i = 0; i < 64; i++) {   // b3g3r3_inv: the byte (bits 0-2 r, 4-6 g) and the latched b
		const uint32_t raw = (uint32_t)s_snap->pal[i] | (uint32_t)s_snap->palext[i] << 8;
		rbx_video.palette[i] = pal3(~raw & 7) << 16 | pal3((~raw >> 4) & 7) << 8 | pal3((~raw >> 8) & 7);
	}
	const uint8_t *g = s_snap->gfxram;   // ram_tilelayout: RGN_FRAC(1,6), planes 0x4000 / 0x2000 / 0, bits reversed
	for (int set = 0; set < 2; set++)
		for (int c = 0; c < 128; c++) {
			const int t0 = set * 0x1000 + c * 32;   // (the tile's 32 bytes in each of the three planes)
			if (s_ramtile_ok && !memcmp(s_ramtile_src + t0, g + t0, 32) &&
			    !memcmp(s_ramtile_src + 0x2000 + t0, g + 0x2000 + t0, 32) &&
			    !memcmp(s_ramtile_src + 0x4000 + t0, g + 0x4000 + t0, 32))
				continue;
			for (int k = 0; k < 3; k++)
				memcpy(s_ramtile_src + k * 0x2000 + t0, g + k * 0x2000 + t0, 32);
			for (int y = 0; y < 16; y++)
				for (int x = 0; x < 16; x++) {
					const int o = set * 0x1000 + c * 32 + (x < 8 ? 0 : 16) + y, b = x & 7;
					s_ramtile[set][c * 256 + y * 16 + x] = (uint8_t)(((g[0x4000 + o] >> b) & 1) << 2 |
					                                                 ((g[0x2000 + o] >> b) & 1) << 1 | ((g[o] >> b) & 1));
				}
		}
	s_ramtile_ok = 1;
	const uint8_t mm = s_snap->mapmask, *sr = s_snap->sreg;
	const int flipx = (mm >> 5) & 1, flipy = !flipx, shiftx = flipx ? -8 : 8;
	memset(s_pri, 0, sizeof(s_pri));
	if (!(mm & 1))
		draw_map(s_snap->bgram, s_ramtile[1], 0x30, (((sr[1] << 2) & 0x300) + sr[2] + shiftx) & 0x3ff,
		         ((sr[1] << 4) & 0x300) + sr[3], flipx, flipy, 1, 1);
	else
		for (int i = VIS_Y0 * 256; i < (VIS_Y0 + VIS_H) * 256; i++)
			s_scr[i] = 0x30;
	if (!(mm & 2))
		draw_map(s_snap->fgram, s_ramtile[0], 0x10, (((sr[1] << 6) & 0x300) + sr[4] + shiftx) & 0x3ff,
		         ((sr[1] << 8) & 0x300) + sr[5], flipx, flipy, 0, 2);
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++)   // the text (TILEMAP_SCAN_COLS 32x32, flipped likewise)
		for (int x = 0; x < 256; x += 8) {   // (a tile's 8 pixels: one lookup)
			const int tx = flipx ? 255 - x : x, ty = flipy ? 255 - y : y;
			const uint8_t code = s_snap->vram[(tx >> 3) * 32 + (ty >> 3)];
			const uint8_t *src = s_gfxchar.pix + (size_t)(code % s_gfxchar.count) * 64 + (ty & 7) * 8;
			for (int k = 0; k < 8; k++) {
				const uint8_t p = src[flipx ? 7 - k : k];
				if (p) {
					s_scr[y * 256 + x + k] = p;
					s_pri[y * 256 + x + k] |= 4;
				}
			}
		}
	const uint8_t *sp = s_snap->spriteram;   // draw_sprites (from the end; prio_transpen)
	for (int i = 0x200 - 4; i >= 0; i -= 4) {
		const int attrib = sp[i];
		if (!(attrib & 1))
			continue;
		int code = sp[i + 1], y = sp[i + 2], x = sp[i + 3];
		const int color = (attrib >> 3) & 1;
		const uint32_t primask = 0xf0f0u | (color ? 0 : 0xccccu);   // GFX_PMASK_4 | GFX_PMASK_2
		code += (attrib << 3) & 0x100;
		y -= (attrib << 1) & 0x100;
		x -= (attrib << 2) & 0x100;
		int fx = (attrib >> 2) & 1, fy = !((attrib >> 1) & 1);
		if (mm & 0x20) {
			fx = !fx;
			fy = !fy;
			x = 240 - x;
			y = 240 - y;
		}
		const uint8_t *src = s_gfxspr.pix + (size_t)(code % s_gfxspr.count) * 256;
		const int hscale = (attrib & 0x10) ? 2 : 1;   // (prio_zoom_transpen: twice as tall)
		for (int yy = 0; yy < 16 * hscale; yy++) {
			const int dy = y + yy;
			if (dy < VIS_Y0 || dy >= VIS_Y0 + VIS_H)
				continue;
			const int srow = fy ? 15 - yy / hscale : yy / hscale;
			for (int xx = 0; xx < 16; xx++) {
				const int dx = x + xx;
				if (dx < 0 || dx > 255)
					continue;
				const uint8_t p = src[srow * 16 + (fx ? 15 - xx : xx)];
				if (!p)
					continue;
				uint8_t *pr = &s_pri[dy * 256 + dx];
				if (!(((1u << *pr) & (primask | 0x80000000u))))
					s_scr[dy * 256 + dx] = (uint16_t)(0x20 + color * 8 + p);
				*pr = 31;
			}
		}
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = 0; y < VIS_H; y++)
		memcpy(bm + y * pitch, s_scr + (VIS_Y0 + y) * 256, 256 * 2);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->vram, s_vram, sizeof(s_vram));
	memcpy(s_snap->spriteram, s_spriteram, sizeof(s_spriteram));
	memcpy(s_snap->fgram, s_fgram, sizeof(s_fgram));
	memcpy(s_snap->bgram, s_bgram, sizeof(s_bgram));
	memcpy(s_snap->gfxram, s_gfxram, sizeof(s_gfxram));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	memcpy(s_snap->palext, s_palext, sizeof(s_palext));
	memcpy(s_snap->sreg, s_sreg, sizeof(s_sreg));
	s_snap->mapmask = s_mapmask;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void bw_reset(void)
{
	memset(s_shared, 0, sizeof(s_shared));
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_ram14, 0, sizeof(s_ram14));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_palext, 0, sizeof(s_palext));
	memset(s_ctrlram, 0, sizeof(s_ctrlram));
	memset(s_fgram, 0, sizeof(s_fgram));
	memset(s_bgram, 0, sizeof(s_bgram));
	memset(s_gfxram, 0, sizeof(s_gfxram));
	memset(s_sreg, 0, sizeof(s_sreg));
	s_palatch = s_mapmask = s_view = 0;
	s_snd_nmimask = s_u8f = s_latch = 0;
	s_coin_nmi = 0;
	s_dac = 0;
	memset(s_sram, 0, sizeof(s_sram));
	ay8910_reset(&s_ay[0]);
	ay8910_reset(&s_ay[1]);
	for (int c = 0; c < 2; c++) {
		m6809_reset(&s_cpu[c]);
		m6809_set_irq(&s_cpu[c], 0);
		m6809_set_firq(&s_cpu[c], 0);
		m6809_set_nmi(&s_cpu[c], 0);
	}
	m6502_reset(&s_snd);
	m6502_set_irq(&s_snd, 0);
	m6502_set_nmi(&s_snd, 0);
	s_line = 0;
	s_frame = 0;
	memset(s_target, 0, sizeof(s_target));
	memset(s_frac, 0, sizeof(s_frac));
	s_ay_frac = s_nmi_frac = 0;
}

static void bw_exit(void);

static int bw_init(const rbx_game *g)
{
	if (!s_pri_hp && !(s_pri_hp = calloc(1, sizeof(*s_pri_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ramtile_hp && !(s_ramtile_hp = calloc(1, sizeof(*s_ramtile_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ramtile_src_hp && !(s_ramtile_src_hp = calloc(1, sizeof(*s_ramtile_src_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_scr_hp && !(s_scr_hp = calloc(1, sizeof(*s_scr_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_ramtile_ok = 0;
	uint32_t cs, ss;
	s_rom[0] = rbx_region("maincpu", &s_rom_size[0]);
	s_rom[1] = rbx_region("subcpu", &s_rom_size[1]);
	s_srom = rbx_region("audiocpu", &s_srom_size);
	uint8_t *chars = rbx_region("chars", &cs), *spr = rbx_region("sprites", &ss);
	if (!s_rom[0] || s_rom_size[0] < 0x10000 || !s_rom[1] || s_rom_size[1] < 0x10000 || !s_srom || s_srom_size < 0x10 ||
	    !chars || !spr) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{	// init_bwing: the sound ROM's nibbles swapped, its vectors relocated
		uint8_t *r = s_srom;
		const uint32_t j = s_srom_size;
		for (uint32_t i = 0; i < j; i++)
			r[i] = (uint8_t)((r[i] >> 4) | (r[i] << 4));
		r[j - (0x10 - 0x4)] = r[j - (0x10 - 0xb)] = r[j - (0x10 - 0x6)];
		r[j - (0x10 - 0x5)] = r[j - (0x10 - 0xa)] = r[j - (0x10 - 0x7)];
	}
	{	// charlayout (2 planes, RGN_FRAC(1,2), bits reversed), spritelayout (RGN_FRAC(1,3), bits reversed)
		static uint32_t x8[8], y8[8], xs[16], ys[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)(7 - i);
			y8[i] = (uint32_t)(i * 8);
			xs[i] = (uint32_t)(7 - i);
			xs[8 + i] = (uint32_t)(128 + 7 - i);
		}
		for (int i = 0; i < 16; i++)
			ys[i] = (uint32_t)(i * 8);
		const uint32_t pc[2] = { 0, cs / 2 * 8 }, t = ss / 3 * 8, ps[3] = { 2 * t, t, 0 };
		const rbx_gfx_layout lc = { 8, 8, 2, pc, x8, y8, 8 * 8 }, ls = { 16, 16, 3, ps, xs, ys, 32 * 8 };
		if (rbx_gfx_decode(&s_gfxchar, &lc, chars, cs / 2) || rbx_gfx_decode(&s_gfxspr, &ls, spr, ss / 3)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	// AY-3-8912 1.5 MHz 0.5 each (legacy output), the DAC 0.1
	for (int i = 0; i < 2; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, CLOCK, AY8910_LEGACY_OUTPUT, 128, 4, 0);
	s_mix_cap = (int)(ay8910_rate(&s_ay[0]) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_snap || !s_mix || rbx_audio_setup(ay8910_rate(&s_ay[0])) || rbx_video_setup(256, VIS_H, 64)) {
		rbx_fail("%s: out of memory", g->set);
		bw_exit();
		return -1;
	}
	for (int c = 0; c < 2; c++) {
		m6809_init(&s_cpu[c]);
		s_cpu[c].read = c ? sub_r : main_r;
		s_cpu[c].write = c ? sub_w : main_w;
		for (int p = c ? 0xa0 : 0x40; p < 0x100; p++)
			s_cpu[c].rpage[p] = s_rom[c] + p * 256;
	}
	m6502_init(&s_snd, M6502_DECO16);
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	s_snd.io_read = snd_io_r;
	s_snd.io_write = snd_io_w;
	s_snd.irq_hold = 1;
	s_port_dsw[0] = rbx_port_find("DSW0");
	s_port_dsw[1] = rbx_port_find("DSW1");
	s_port_in[0] = rbx_port_find("IN0");
	s_port_in[1] = rbx_port_find("IN1");
	s_port_in[2] = rbx_port_find("IN2");
	s_port_vbl = rbx_port_find("VBLANK");
	bw_reset();
	return 0;
}

static void bw_exit(void)
{
	rbx_gfx_free(&s_gfxchar);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
	free(s_mix);
	s_mix = NULL;
	free(s_pri_hp);
	s_pri_hp = NULL;
	free(s_ramtile_hp);
	s_ramtile_hp = NULL;
	free(s_ramtile_src_hp);
	s_ramtile_src_hp = NULL;
	free(s_scr_hp);
	s_scr_hp = NULL;
}

static void bw_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	coins();
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H)   // (VIDEO_UPDATE_BEFORE_VBLANK)
			vblank_capture();
		for (int c = 0; c < 2; c++) {
			s_target[c] += rbx_line_cycles(&s_frac[c], CLOCK, FRAME_MHZ, LINES);
			m6809_run_until(&s_cpu[c], s_target[c]);
		}
		// sound_interrupt at 1 kHz: the NMI asserted unless masked
		s_nmi_frac += (uint64_t)1000 * 1000;
		while (s_nmi_frac >= (uint64_t)FRAME_MHZ * LINES) {
			s_nmi_frac -= (uint64_t)FRAME_MHZ * LINES;
			if (!s_snd_nmimask)
				m6502_set_nmi(&s_snd, 1);
		}
		s_target[2] += rbx_line_cycles(&s_frac[2], CLOCK, FRAME_MHZ, LINES);
		m6502_run_until(&s_snd, s_target[2]);
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&s_ay_frac, CLOCK, FRAME_MHZ, LINES);
		const int n0 = ay8910_render(&s_ay[0], clocks, s_mix, s_mix_cap);
		const int n1 = ay8910_render(&s_ay[1], clocks, s_mix, s_mix_cap);
		const int n = n0 > n1 ? n0 : n1;
		for (int k = 0; k < n; k++) {
			rbx_audio_put(s_mix[k] + s_dac / 10);
			s_mix[k] = 0;
		}
	}
	s_frame++;
}

static int bw_state(void *ctx, int load)
{
	uint8_t cpu[2][M6809_STATE_SIZE], snd[M6502_STATE_SIZE];
	if (!load) {
		for (int c = 0; c < 2; c++)
			memcpy(cpu[c], &s_cpu[c], M6809_STATE_SIZE);
		memcpy(snd, &s_snd, M6502_STATE_SIZE);
	}
	uint64_t fr[5] = { s_frac[0], s_frac[1], s_frac[2], s_ay_frac, s_nmi_frac };
	int32_t misc[12] = { (int32_t)s_frame, s_palatch, s_mapmask, s_view, s_snd_nmimask, s_u8f, s_latch, s_coin_nmi,
	                     s_dac, 0, 0, 0 };
	uint8_t ay[2][256];
	const uint32_t ays = ay8910_state_size();
	if (!load)
		for (int i = 0; i < 2; i++)
			memcpy(ay[i], &s_ay[i], ays);
	int rc = 0;
	rc |= rbx_state_io(ctx, "cpu", cpu, sizeof(cpu));
	rc |= rbx_state_io(ctx, "snd", snd, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "shared", s_shared, sizeof(s_shared));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "ram14", s_ram14, sizeof(s_ram14));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "palx", s_palext, sizeof(s_palext));
	rc |= rbx_state_io(ctx, "ctrl", s_ctrlram, sizeof(s_ctrlram));
	rc |= rbx_state_io(ctx, "fg", s_fgram, sizeof(s_fgram));
	rc |= rbx_state_io(ctx, "bg", s_bgram, sizeof(s_bgram));
	rc |= rbx_state_io(ctx, "gfx", s_gfxram, sizeof(s_gfxram));
	rc |= rbx_state_io(ctx, "sreg", s_sreg, sizeof(s_sreg));
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_state_io(ctx, "ay", ay, sizeof(ay));
	rc |= rbx_state_io(ctx, "tg", s_target, sizeof(s_target));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		for (int c = 0; c < 2; c++)
			memcpy(&s_cpu[c], cpu[c], M6809_STATE_SIZE);
		memcpy(&s_snd, snd, M6502_STATE_SIZE);
		for (int i = 0; i < 2; i++)
			memcpy(&s_ay[i], ay[i], ays);
		for (int i = 0; i < 3; i++)
			s_frac[i] = fr[i];
		s_ay_frac = fr[3];
		s_nmi_frac = fr[4];
		s_frame = (uint32_t)misc[0];
		s_palatch = (uint8_t)misc[1];
		s_mapmask = (uint8_t)misc[2];
		s_view = (uint8_t)misc[3];
		s_snd_nmimask = (uint8_t)misc[4];
		s_u8f = (uint8_t)misc[5];
		s_latch = (uint8_t)misc[6];
		s_coin_nmi = misc[7];
		s_dac = misc[8];
	}
	return rc ? -1 : 0;
}

static int bw_save(void *ctx) { return bw_state(ctx, 0); }
static int bw_load(void *ctx) { return bw_state(ctx, 1); }
static int bw_supported(const char *machine) { return !strcmp(machine, "bwing"); }

const rbx_driver rbx_drv_bwing = { "DECO B-Wings", bw_init, bw_exit, bw_reset, bw_frame, bw_save, bw_load, bw_supported,
                                   bw_render, NULL };
