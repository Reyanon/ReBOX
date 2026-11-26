/*  Capcom Last Duel, Mad Gear / Led Storm and Led Storm Rally 2011. Ported from MAME src/mame/capcom/lastduel.cpp
    (BSD-3-Clause, copyright-holders: Bryan McPhail).
    68000 at 10 MHz: IRQ 2 / 4 (Mad Gear 5 / 6) held at the vblank's start and end; a BG and an FG layer of 16x16 tiles
    (the FG in two pen groups: some pens over the sprites), 16x16 sprites from a buffer copied on a register write, an
    8x8 text layer of 2 bits. Sound: Z80 at 3.58 MHz with two YM2203s (the first one's IRQ), the latch polled; Mad Gear
    adds an OKI6295 and a banked sound ROM.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/decoband.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 10000000
#define Z80_CLOCK 3579545
#define YM_CLOCK 3579545
#define FRAME_MHZ 57444          // set_raw(24 MHz / 3, 512, 64, 448, 272, 8, 248)
#define LINES 272
#define VIS_X0 64
#define VIS_Y0 8
#define VIS_W 384
#define VIS_H 240
#define PAL_N 1024
#define BLACK PAL_N

static int s_madgear;
static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static uint16_t s_ram[0x8000];          // lastduel 0xfe0000 .. 0xfe7fff (mirrored); madgear 0xff0000 .. 0xffffff
static uint16_t s_unused[0x400];        // 0xfc0000 .. 0xfc07ff
static uint16_t s_spriteram[0x400];     // 0xfc0800 .. 0xfc0fff
static uint16_t s_spritebuf[0x400];
static uint16_t s_txram[0x1000];        // lastduel 0xfcc000 (0x2000 bytes); madgear 0xfc8000
static uint16_t s_vram[2][0x4000];      // FG, BG (lastduel 0x2000 words each, madgear 0x2000 / 0x4000)
static uint16_t s_pal[0x400];
static uint16_t s_vctrl[8];
static uint8_t s_flip;
static rbx_gfx s_gfxspr, s_gfxtx, s_gfxbg, s_gfxfg;
static int s_port[4];
static int s_line;
static uint64_t s_cpu_frac, s_z80_frac, s_ym_frac;
static struct {
	uint16_t spr[0x400], tx[0x1000], vram[2][0x4000], pal[0x400], vctrl[8];
	uint8_t flip;
} *s_snap;

// ---- sound ----
static z80 s_z80;
static uint8_t s_zram[0x800];
static uint8_t s_latch, s_bank;
static okim6295_t s_oki;
static int s_ym_irq;
static uint32_t s_oki_acc;
static int32_t s_oki_hold;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static const uint8_t *s_zmap[256];

static void ym_irq(int state) { s_ym_irq = state; }

static void set_bank(void)
{
	for (uint32_t p = 0; p < 0x50; p++) {   // 0x8000 .. 0xcfff: the ROM at 0x8000 + bank * 0x4000
		const uint32_t o = 0x8000 + (uint32_t)s_bank * 0x4000 + p * 256;
		s_zmap[0x80 + p] = o + 256 <= s_zrom_size ? s_zrom + o : NULL;
	}
}

static uint8_t z80_read(void *u, uint16_t a)
{
	(void)u;
	if (s_madgear) {
		if (a < 0x8000) return a < s_zrom_size ? s_zrom[a] : 0;
		if (a < 0xd000) {
			const uint32_t o = 0x8000 + (uint32_t)s_bank * 0x4000 + (a - 0x8000u);
			return o < s_zrom_size ? s_zrom[o] : 0;
		}
		if (a < 0xd800) return s_zram[a - 0xd000];
		if (a == 0xf000 || a == 0xf001) return rbx_ym_read(0, a & 1);
		if (a == 0xf002 || a == 0xf003) return rbx_ym_read(1, a & 1);
		if (a == 0xf006) return s_latch;
		return 0;
	}
	if (a < 0xe000) return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0xe800) return s_zram[a - 0xe000];
	if (a == 0xe800 || a == 0xe801) return rbx_ym_read(0, a & 1);
	if (a == 0xf000 || a == 0xf001) return rbx_ym_read(1, a & 1);
	if (a == 0xf800) return s_latch;
	return 0;
}
static void z80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_madgear) {
		if (a >= 0xd000 && a < 0xd800) s_zram[a - 0xd000] = v;
		else if (a == 0xf000 || a == 0xf001) rbx_ym_write(0, a & 1, v);
		else if (a == 0xf002 || a == 0xf003) rbx_ym_write(1, a & 1, v);
		else if (a == 0xf004) oki_write(&s_oki, v);
		else if (a == 0xf00a) {
			s_bank = v & 1;
			set_bank();
		}
		return;
	}
	if (a >= 0xe000 && a < 0xe800) s_zram[a - 0xe000] = v;
	else if (a == 0xe800 || a == 0xe801) rbx_ym_write(0, a & 1, v);
	else if (a == 0xf000 || a == 0xf001) rbx_ym_write(1, a & 1, v);
}
static uint8_t z80_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z80_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

static void z80_start(void)
{
	z80_init(&s_z80);
	s_z80.read_byte = z80_read;
	s_z80.write_byte = z80_write;
	s_z80.port_in = z80_in;
	s_z80.port_out = z80_out;
	s_z80.read_map = s_zmap;
}

static void sound_line(void)
{
	const int cycles = rbx_line_cycles(&s_z80_frac, Z80_CLOCK, FRAME_MHZ, LINES);
	z80_run(&s_z80, s_z80.cyc + (unsigned long)cycles, &s_ym_irq, 0);
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&s_ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
	const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	rbx_ym_advance(1, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	if (s_madgear) {
		const uint32_t yr = rbx_ym_rate(0);
		for (int i = 0; i < n; i++) {
			s_oki_acc += s_oki.rate;
			while (s_oki_acc >= yr) {
				s_oki_acc -= yr;
				s_oki_hold = oki_sample(&s_oki);
			}
			s_mix[s_mix_n + i] += s_oki_hold;
		}
	}
	s_mix_n += n;
}

static void sound_frame_end(void)
{
	rbx_ym_async_take(0, s_mix, s_mix_n);
	rbx_ym_async_take(1, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

// ---- bus ----
static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0xfc4000 && a < 0xfc4008) {   // lastduel: P1_P2, SYSTEM, DSW1, DSW2; madgear: DSW1, DSW2, P1_P2, SYSTEM
		static const int ld[4] = { 0, 1, 2, 3 }, mg[4] = { 2, 3, 0, 1 };
		const int i = (int)((a >> 1) & 3);
		return (uint16_t)rbx_port_read(s_port[s_madgear ? mg[i] : ld[i]]);
	}
	return 0;
}
static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a == 0xfc4000 && (mask & 0x00ff))
		s_flip = v & 1;   // flip_w (and the coin lockouts / counters)
	else if (a == 0xfc4002 && (mask & 0x00ff))
		s_latch = (uint8_t)v;
	else if (a >= (s_madgear ? 0xfd0000u : 0xfc8000u) && a < (s_madgear ? 0xfd0010u : 0xfc8010u)) {   // vctrl_w
		const int o = (int)((a >> 1) & 7);
		s_vctrl[o] = (uint16_t)((s_vctrl[o] & ~mask) | (v & mask));
		if (o == 4)
			memcpy(s_spritebuf, s_spriteram, sizeof(s_spriteram));
	}
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
// the FG's pen groups (set_transmask: the pens transparent in layer 0 / layer 1)
static uint16_t s_fg_mask0[2], s_fg_mask1[2];
static uint16_t s_line_idx[VIS_H][VIS_W];   // (the frame, indexed; the bitmap's rows)

enum { L_BG_OPAQUE, L_BG_TRANS, L_FG_L0, L_FG_L1, L_FG_OPAQUE };

// one line of a 16x16 layer into the frame: layer 0 = FG, 1 = BG (a tile's span at a time)
static void tile_line(int layer, int mode, int y, uint16_t *dst)
{
	const int fg = layer == 0, flip = s_snap->flip;
	const uint16_t *vram = s_snap->vram[layer];
	const rbx_gfx *gfx = fg ? &s_gfxfg : &s_gfxbg;
	const uint16_t base = fg ? 0x100 : 0x000;
	const uint32_t sy = s_snap->vctrl[fg ? 0 : 2], sx = s_snap->vctrl[fg ? 1 : 3];
	const uint32_t hmask = s_madgear ? 511 : 1023;
	const uint32_t ty = (uint32_t)(flip ? 255 - y + (int)sy : y + (int)sy) & hmask;
	uint32_t tx = (uint32_t)(flip ? 511 - VIS_X0 + (int)sx : VIS_X0 + (int)sx) & 1023;
	int x = 0;
	while (x < VIS_W) {
		uint32_t code, attr, group;
		if (s_madgear) {   // TILEMAP_SCAN_COLS 64x32: the tile, its attributes 0x800 after
			const uint32_t i = (tx >> 4) * 32 + (ty >> 4);
			code = vram[i] & 0x1fffu;
			attr = vram[i + 0x800];
			group = (attr & 0x10) >> 4;
		} else {           // TILEMAP_SCAN_ROWS 64x64, two words a tile
			const uint32_t i = (ty >> 4) * 64 + (tx >> 4);
			code = vram[2 * i] & 0x1fffu;
			attr = vram[2 * i + 1];
			group = (attr & 0x80) >> 7;
		}
		code %= gfx->count;
		uint32_t mask;   // (the pens not drawn)
		switch (mode) {
		case L_BG_OPAQUE: case L_FG_OPAQUE: mask = 0; break;
		case L_BG_TRANS: mask = 0x8000; break;
		case L_FG_L0: mask = s_fg_mask0[group]; break;
		default: mask = s_fg_mask1[group]; break;
		}
		const uint32_t fy = (attr & 0x40) ? 15 - (ty & 15) : (ty & 15);
		const uint8_t *src = gfx->pix + code * 256 + fy * 16;
		const uint16_t c0 = (uint16_t)(base + (attr & 0xf) * 16);
		const int fx = (attr & 0x20) != 0;
		int px = (int)(tx & 15), n = flip ? px + 1 : 16 - px;
		if (n > VIS_W - x)
			n = VIS_W - x;
		if (mask != 0xffff && (mask || gfx->used[code] || mode == L_BG_OPAQUE || mode == L_FG_OPAQUE))
			for (int k = 0; k < n; k++) {
				const int c = flip ? px - k : px + k;
				const uint8_t p = src[fx ? 15 - c : c];
				if (!((mask >> p) & 1))
					dst[x + k] = (uint16_t)(c0 + p);
			}
		x += n;
		tx = (tx + (uint32_t)(flip ? -n : n)) & 1023;
	}
}

static void text_line(int y, uint16_t *dst)   // 64x32 of 8x8, 2 bits (pen 3 transparent), no scroll
{
	const int flip = s_snap->flip;
	const uint32_t ty = (uint32_t)(flip ? 255 - y : y) & 255;
	uint32_t tx = (uint32_t)(flip ? 511 - VIS_X0 : VIS_X0) & 511;
	int x = 0;
	while (x < VIS_W) {
		const uint16_t t = s_snap->tx[(ty >> 3) * 64 + (tx >> 3)];
		const uint32_t code = rbx_wrap((t & 0x7ffu), s_gfxtx.count);
		const uint32_t fy = (t & 0x800) ? 7 - (ty & 7) : (ty & 7);
		const uint8_t *src = s_gfxtx.pix + code * 64 + fy * 8;
		const uint16_t c0 = (uint16_t)(0x300 + (t >> 12) * 4);
		int px = (int)(tx & 7), n = flip ? px + 1 : 8 - px;
		if (n > VIS_W - x)
			n = VIS_W - x;
		for (int k = 0; k < n; k++) {
			const uint8_t p = src[flip ? px - k : px + k];
			if (p != 3)
				dst[x + k] = (uint16_t)(c0 + p);
		}
		x += n;
		tx = (tx + (uint32_t)(flip ? -n : n)) & 511;
	}
}

static void draw_sprites(int pri, int pri_mask, int flipy_mask)
{
	const uint16_t *spr = s_snap->spr;
	for (int offs = 0x400 - 4; offs >= 0; offs -= 4) {
		const int attr = spr[offs + 1];
		if (pri_mask) {
			if (pri == 1 && (attr & pri_mask))
				continue;
			if (pri == 0 && !(attr & pri_mask))
				continue;
		}
		const uint32_t code = rbx_wrap(spr[offs], s_gfxspr.count);
		int sx = spr[offs + 3] & 0x1ff, sy = spr[offs + 2] & 0x1ff;
		if (sy > 0x100)
			sy -= 0x200;
		int flipx = attr & 0x20, flipy = attr & flipy_mask;
		if (s_snap->flip) {
			sx = 496 - sx;
			sy = 240 - sy;
			flipx = !flipx;
			flipy = !flipy;
		}
		if (!s_gfxspr.used[code])
			continue;
		sx -= VIS_X0;
		sy -= VIS_Y0;
		if (sx >= VIS_W || sx + 16 <= 0 || sy >= VIS_H || sy + 16 <= 0)
			continue;
		const uint16_t c0 = (uint16_t)(0x200 + (attr & 0x0f) * 16);
		const uint8_t *g = s_gfxspr.pix + code * 256;
		for (int j = 0; j < 16; j++) {
			const int yy = sy + j;
			if ((unsigned)yy >= VIS_H)
				continue;
			const uint8_t *s = g + (flipy ? 15 - j : j) * 16;
			uint16_t *d = s_line_idx[yy];
			for (int i = 0; i < 16; i++) {
				const int xx = sx + i;
				if ((unsigned)xx >= VIS_W)
					continue;
				const uint8_t p = s[flipx ? 15 - i : i];
				if (p != 15)
					d[xx] = (uint16_t)(c0 + p);
			}
		}
	}
}

// the layer passes between the sprite passes (screen_update_lastduel / screen_update_madgear)
#define BANDS 8
static int s_phase;
static void ld_band(int band)
{
	for (int y = VIS_H * band / BANDS; y < VIS_H * (band + 1) / BANDS; y++) {
		uint16_t *d = s_line_idx[y];
		const int by = y + VIS_Y0;
		const int alt = s_madgear && (s_snap->vctrl[7] & 0x80);
		switch (s_phase) {
		case 0:   // before the (first) sprites
			if (alt)
				tile_line(0, L_FG_OPAQUE, by, d);   // (LAYER1 | OPAQUE: every pixel)
			else {
				tile_line(1, L_BG_OPAQUE, by, d);
				tile_line(0, L_FG_L1, by, d);
			}
			break;
		case 1:   // between the sprite passes
			tile_line(0, L_FG_L0, by, d);
			if (alt)
				tile_line(1, L_BG_TRANS, by, d);
			break;
		default:  // the text, then the frame out
			text_line(by, d);
			memcpy(rbx_video.bitmap + y * rbx_video.pitch, d, VIS_W * 2);
			break;
		}
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, ld_band };
static int ld_help(int helper) { return decoband_help(&s_bands, helper); }

static void ld_render(void)
{
	for (int i = 0; i < PAL_N; i++) {   // RRRRGGGGBBBBIIII
		const uint32_t raw = s_snap->pal[i], bright = 0x10 + (raw & 0x0f);
		const uint32_t r = ((raw >> 12) & 0x0f) * bright * 0x11 / 0x1f, g = ((raw >> 8) & 0x0f) * bright * 0x11 / 0x1f;
		const uint32_t b = ((raw >> 4) & 0x0f) * bright * 0x11 / 0x1f;
		rbx_video.palette[i] = r << 16 | g << 8 | b;
	}
	rbx_video.palette[BLACK] = 0;
	s_phase = 0;
	decoband_run(&s_bands);
	if (s_madgear)
		draw_sprites(0, 0x10, 0x80);
	else
		draw_sprites(0, 0, 0x40);
	s_phase = 1;
	decoband_run(&s_bands);
	if (s_madgear)
		draw_sprites(1, 0x10, 0x80);
	s_phase = 2;
	decoband_run(&s_bands);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spr, s_spritebuf, sizeof(s_spritebuf));
	memcpy(s_snap->tx, s_txram, sizeof(s_txram));
	memcpy(s_snap->vram, s_vram, sizeof(s_vram));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	memcpy(s_snap->vctrl, s_vctrl, sizeof(s_vctrl));
	s_snap->flip = s_flip;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void ld_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_unused, 0, sizeof(s_unused));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_spritebuf, 0, sizeof(s_spritebuf));
	memset(s_txram, 0, sizeof(s_txram));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_vctrl, 0, sizeof(s_vctrl));
	s_flip = 0;
	s_latch = 0;
	s_bank = 0;
	s_ym_irq = 0;
	memset(s_zram, 0, sizeof(s_zram));
	memset(s_zmap, 0, sizeof(s_zmap));
	for (uint32_t p = 0; p < (s_madgear ? 0x80u : 0xe0u) && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	if (s_madgear)
		set_bank();
	z80_start();
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	oki_reset(&s_oki);
	s_oki_acc = 0;
	s_oki_hold = 0;
	s_mix_n = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_line = 0;
	s_cpu_frac = s_z80_frac = s_ym_frac = 0;
}

static void ld_exit(void);

static int ld_init(const rbx_game *g)
{
	s_madgear = !strncmp(g->machine, "madgear", 7);
	uint32_t ss, ts, bs, fs, osize = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *spr = rbx_region("sprites", &ss), *tx = rbx_region("gfx2", &ts), *bg = rbx_region("gfx3", &bs);
	uint8_t *fg = rbx_region("gfx4", &fs), *oki = s_madgear ? rbx_region("oki", &osize) : NULL;
	if (!s_rom || !s_zrom || !spr || !tx || !bg || !fg || (s_madgear && !oki)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	{	// sprite_layout, text_layout, madgear_tile
		static uint32_t xs[16], ys[16], xt[8], yt[8], xm[16], ym[16];
		for (int i = 0; i < 8; i++) {
			xs[i] = (uint32_t)i;
			xs[8 + i] = (uint32_t)(8 * 4 * 16 + i);
			yt[i] = (uint32_t)(i * 4 * 2 * 2);
		}
		for (int i = 0; i < 4; i++) {
			xt[i] = (uint32_t)i;
			xt[4 + i] = (uint32_t)(4 * 2 + i);
			xm[i] = (uint32_t)i;
			xm[4 + i] = (uint32_t)(4 * 4 + i);
			xm[8 + i] = (uint32_t)(8 * 4 * 16 + i);
			xm[12 + i] = (uint32_t)(8 * 4 * 16 + 4 * 4 + i);
		}
		for (int i = 0; i < 16; i++) {
			ys[i] = (uint32_t)(i * 8 * 4);
			ym[i] = (uint32_t)(i * 8 * 4);
		}
		static const uint32_t ps[4] = { 16, 0, 24, 8 }, pt[2] = { 4, 0 }, pm[4] = { 3 * 4, 2 * 4, 1 * 4, 0 * 4 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, xs, ys, 128 * 8 };
		const rbx_gfx_layout lt = { 8, 8, 2, pt, xt, yt, 16 * 8 };
		const rbx_gfx_layout lm = { 16, 16, 4, pm, xm, ym, 64 * 16 };
		if (rbx_gfx_decode(&s_gfxspr, &ls, spr, ss) || rbx_gfx_decode(&s_gfxtx, &lt, tx, ts) ||
		    rbx_gfx_decode(&s_gfxbg, &lm, bg, bs) || rbx_gfx_decode(&s_gfxfg, &lm, fg, fs)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			ld_exit();
			return -1;
		}
	}
	if (s_madgear) {   // video_start madgear: set_transmask(0, 0xffff, 0x8000), (1, 0x80ff, 0xff00)
		s_fg_mask0[0] = 0xffff; s_fg_mask1[0] = 0x8000;
		s_fg_mask0[1] = 0x80ff; s_fg_mask1[1] = 0xff00;
	} else {           // lastduel: (0, 0xffff, 0x0001), (1, 0xf07f, 0x0f81)
		s_fg_mask0[0] = 0xffff; s_fg_mask1[0] = 0x0001;
		s_fg_mask0[1] = 0xf07f; s_fg_mask1[1] = 0x0f81;
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;   // two YM2203s, all outputs 0.40
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 102;
	yc.irq = ym_irq;
	int bad = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	yc.irq = NULL;
	if (!bad)
		bad = rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	if (bad) {
		rbx_fail("%s: out of memory", g->set);
		ld_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, PAL_N + 1)) {
		rbx_fail("%s: out of memory", g->set);
		ld_exit();
		return -1;
	}
	if (s_madgear)
		oki_init(&s_oki, 1000000, 1, oki, osize, 251);
	static const char *const names[4] = { "P1_P2", "SYSTEM", "DSW1", "DSW2" };
	for (int i = 0; i < 4; i++)
		s_port[i] = rbx_port_find(names[i]);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);   // (MAME's DIVx / MULx / ADDQ.W An times)
	m68k_bus_reset_map();
	m68k_bus_map(0, s_rom_size - 1, s_rom, s_rom_size, 1, 0);
	for (uint32_t m = 0; m < 0x4000; m += 0x1000) {   // (mirror 0x003000)
		m68k_bus_map(0xfc0000 + m, 0xfc07ff + m, s_unused, sizeof(s_unused), 1, 1);
		m68k_bus_map(0xfc0800 + m, 0xfc0fff + m, s_spriteram, sizeof(s_spriteram), 1, 1);
	}
	if (s_madgear) {
		m68k_bus_map(0xfc8000, 0xfc9fff, s_txram, 0x2000, 1, 1);
		m68k_bus_map(0xfcc000, 0xfcc7ff, s_pal, sizeof(s_pal), 1, 1);
		m68k_bus_map(0xfd4000, 0xfd7fff, s_vram[0], 0x4000, 1, 1);
		m68k_bus_map(0xfd8000, 0xfdffff, s_vram[1], 0x8000, 1, 1);
		m68k_bus_map(0xff0000, 0xffffff, s_ram, sizeof(s_ram), 1, 1);
	} else {
		m68k_bus_map(0xfcc000, 0xfcdfff, s_txram, 0x2000, 1, 1);
		m68k_bus_map(0xfd0000, 0xfd3fff, s_vram[0], 0x4000, 1, 1);
		m68k_bus_map(0xfd4000, 0xfd7fff, s_vram[1], 0x4000, 1, 1);
		m68k_bus_map(0xfd8000, 0xfd87ff, s_pal, sizeof(s_pal), 1, 1);
		for (uint32_t m = 0; m < 0x20000; m += 0x8000)   // (mirror 0x018000)
			m68k_bus_map(0xfe0000 + m, 0xfe7fff + m, s_ram, 0x8000, 1, 1);
	}
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	ld_reset();
	return 0;
}

static void ld_exit(void)
{
	m68k_set_mame_timing(0);
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gfxspr);
	rbx_gfx_free(&s_gfxtx);
	rbx_gfx_free(&s_gfxbg);
	rbx_gfx_free(&s_gfxfg);
	free(s_snap);
	s_snap = NULL;
}

static void ld_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // the vblank's start: the frame drawn, IRQ 2 (5)
			vblank_capture();
			m68k_bus_hold_irq(s_madgear ? 5 : 2);
		} else if (line == VIS_Y0)      // its end: IRQ 4 (6), the controls
			m68k_bus_hold_irq(s_madgear ? 6 : 4);
		m68k_execute(rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		sound_line();
	}
	sound_frame_end();
}

static int ld_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[3] = { s_cpu_frac, s_z80_frac, s_ym_frac };
	int32_t misc[7] = { s_latch, s_bank, s_ym_irq, (int32_t)s_oki_acc, s_oki_hold, s_flip, 0 };
	z80 z = s_z80;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "unused", s_unused, sizeof(s_unused));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "sprb", s_spritebuf, sizeof(s_spritebuf));
	rc |= rbx_state_io(ctx, "tx", s_txram, sizeof(s_txram));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "vctrl", s_vctrl, sizeof(s_vctrl));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	if (s_madgear)
		rc |= rbx_state_io(ctx, "oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		s_latch = (uint8_t)misc[0];
		s_bank = (uint8_t)misc[1];
		if (s_madgear)
			set_bank();
		z.read_byte = z80_read;
		z.write_byte = z80_write;
		z.port_in = z80_in;
		z.port_out = z80_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		s_z80 = z;
		s_cpu_frac = fr[0];
		s_z80_frac = fr[1];
		s_ym_frac = fr[2];
		s_ym_irq = misc[2];
		s_oki_acc = (uint32_t)misc[3];
		s_oki_hold = misc[4];
		s_flip = (uint8_t)misc[5];
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int ld_save(void *ctx) { return ld_state(ctx, 0); }
static int ld_load(void *ctx) { return ld_state(ctx, 1); }
static int ld_supported(const char *machine) { return !strcmp(machine, "lastduel") || !strncmp(machine, "madgear", 7); }

const rbx_driver rbx_drv_lastduel = { "Capcom Last Duel", ld_init, ld_exit, ld_reset, ld_frame, ld_save, ld_load,
                                      ld_supported, ld_render, ld_help };
