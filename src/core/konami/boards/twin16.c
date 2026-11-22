/*  Konami Twin 16 (Devil World, Vulcan Venture / Gradius II, The Final Round, Hard Puncher, M.I.A. and Cue Brick in
    their Japanese sets). Ported from MAME src/mame/konami/twin16.cpp (BSD-3-Clause, copyright-holders: Phil Stroffolino)
    and twin16_v.cpp (BSD-3-Clause, copyright-holders: Phil Stroffolino). Two 68000s at 9.216 MHz (cpu/m68k_multi.c; The
    Final Round has one), the Twin16 video (a fixed layer, two scroll layers whose tiles are in the "zip" RAM, the sprites
    with their preprocessor and the 007789 PAL's priorities, drawn here into the indexed bitmap); the sound: a Z80 at
    3.58 MHz, a YM2151, a K007232, a uPD7759, a frame behind on the worker.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k_multi.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/k007232.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/upd7759.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 9216000u    // (18.432 MHz / 2)
#define Z80_CLOCK 3579545u
#define CHIP_CLOCK 3579545u
#define UPD_CLOCK 640000u
#define SND_RATE 48000
#define PIX_CLOCK 6144000u    // (set_raw(18.432 MHz / 3, 384, 0, 320, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_Y0 16
#define VIS_H 224

enum { M_TWIN16, M_DEVILW, M_MIAJ, M_CUEBRICKJ, M_FROUND };
enum { FLIPY = 0x01, FLIPX = 0x02, TILE_FLIPY = 0x20 };              // (m_video_register)
enum { BG_OVER_SPRITES = 0x01, BG_NO_SHADOW = 0x02, SPRITE_OCCUPIED = 0x04 };

static int s_mach, s_slices, s_vis_x0, s_vis_w;
static uint8_t *s_rom, *s_subrom, *s_zrom, *s_data;
static uint32_t s_rom_size, s_subrom_size, s_zrom_size, s_data_size;
static uint16_t *s_gfxrom;          // (host order words: the sprites' data and CPU B's 600000)
static uint32_t s_gfxrom_words;
static const uint8_t *s_fixed, *s_tiles;   // ("fixed": the fixed layer's tiles; "tiles": The Final Round's scroll tiles)
static uint32_t s_tiles_size;
static uint8_t *s_fixgfx, *s_scrgfx;       // (decoded: a byte a pixel, 64 a tile)
static uint32_t s_scr_ntiles;
static uint8_t s_dirty[0x2000];            // (the zip RAM's tiles written since the last decoding)
static int s_any_dirty;
static z80 s_z;
static const uint8_t *s_zmap[256];
static k007232_t s_k232;
static upd7759_t s_upd;
static uint8_t (*s_nv_hp)[0x8000];
#define s_nv (*s_nv_hp)
static char s_nv_path[256];
static int s_port_sys, s_port_p[3], s_port_dsw[3];

static struct {
	uint16_t fixram[0x2000], videoram[2][0x1000], spriteram[2][0x2000], sprite_buffer[0x800];
	uint16_t zipram[0x20000], sprite_gfx_ram[0x10000];
	uint8_t pal8[0x800];
	uint16_t scrollx[3], scrolly[3], video_register;
	uint8_t sprite_process_enable, sprite_busy, sprite_busy_lines, need_process_spriteram;
	uint16_t cpua_register, cpub_register;
	uint8_t gfx_bank[4], nvbank;
	uint8_t zram[0x800];
	uint8_t soundlatch, z80_irq;
	uint64_t k232_frac;
	int32_t k232_a, k232_b;
	uint64_t cpu_frac, z_frac, z_end, ym_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;
static uint16_t s_ram[2][0x2000], s_comram[0x2000];   // (CPU A's and B's 060000, the shared 040000)

// ---- the drawing's snapshot ----
static struct {
	uint16_t fixram[0x2000], videoram[2][0x1000], sprites[0x800], sprite_gfx_ram[0x10000];
	uint8_t pal8[0x800], gfx_bank[4];
	uint16_t scrollx[3], scrolly[3], video_register;
} s_vs;
static uint8_t (*s_pri_hp)[VIS_H][320];
#define s_pri (*s_pri_hp)

static void decode_tile(uint8_t *d, uint32_t r0, uint32_t r1, uint32_t r2, uint32_t r3, uint32_t r4, uint32_t r5,
                        uint32_t r6, uint32_t r7)
{
	const uint32_t rows[8] = { r0, r1, r2, r3, r4, r5, r6, r7 };
	for (int y = 0; y < 8; y++)   // (gfx_8x8x4_packed_msb: the first pixel in the top nibble)
		for (int x = 0; x < 8; x++)
			d[y * 8 + x] = (uint8_t)(rows[y] >> (28 - 4 * x) & 15);
}
static void decode_rom_tiles(uint8_t *d, const uint8_t *rom, uint32_t ntiles)
{
	for (uint32_t i = 0; i < ntiles; i++) {
		const uint8_t *r = rom + i * 32;
		uint32_t w[8];
		for (int y = 0; y < 8; y++)
			w[y] = (uint32_t)r[y * 4] << 24 | (uint32_t)r[y * 4 + 1] << 16 | (uint32_t)r[y * 4 + 2] << 8 | r[y * 4 + 3];
		decode_tile(d + i * 64, w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7]);
	}
}
static void decode_zip_tile(uint32_t i)   // (the zip RAM: host order words)
{
	const uint16_t *r = t.zipram + i * 16;
	uint32_t w[8];
	for (int y = 0; y < 8; y++)
		w[y] = (uint32_t)r[y * 2] << 16 | r[y * 2 + 1];
	decode_tile(s_scrgfx + i * 64, w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7]);
}

static void capture(void)
{
	if (s_any_dirty) {   // (the drawing is idle here)
		for (uint32_t i = 0; i < 0x2000; i++)
			if (s_dirty[i]) {
				s_dirty[i] = 0;
				decode_zip_tile(i);
			}
		s_any_dirty = 0;
	}
	memcpy(s_vs.fixram, t.fixram, sizeof(s_vs.fixram));
	memcpy(s_vs.videoram, t.videoram, sizeof(s_vs.videoram));
	memcpy(s_vs.sprites, t.spriteram[1] + 0x1800, sizeof(s_vs.sprites));
	if (s_mach != M_FROUND)
		memcpy(s_vs.sprite_gfx_ram, t.sprite_gfx_ram, sizeof(s_vs.sprite_gfx_ram));
	memcpy(s_vs.pal8, t.pal8, sizeof(s_vs.pal8));
	memcpy(s_vs.gfx_bank, t.gfx_bank, sizeof(s_vs.gfx_bank));
	memcpy(s_vs.scrollx, t.scrollx, sizeof(s_vs.scrollx));
	memcpy(s_vs.scrolly, t.scrolly, sizeof(s_vs.scrolly));
	s_vs.video_register = t.video_register;

}

// ---- the drawing (screen_update) ----
static inline uint64_t row8(const uint8_t *p)   // (a tile row's 8 pixels: 0 = all transparent)
{
	uint64_t v;
	memcpy(&v, p, 8);
	return v;
}

// a scroll layer (the tilemap drawn with every category): opaque draws every pixel; the opaque pixels OR in pri0
// (category 0) or pri1 (category 1). MAME's two draws of a layer (an OPAQUE one then a masked one with priority bits,
// or category 0 then 1) are one here
static void draw_scroll(uint16_t *bm, int which, int opaque, uint8_t pri0, uint8_t pri1)
{
	const int vr = s_vs.video_register, w = s_vis_w;
	const int fx = vr & FLIPX, fy = vr & FLIPY;
	// (a flipped tilemap: mirrored around the visible area's centre, x0 + x1 + 1 and y0 + y1 + 1)
	const int xext = 2 * s_vis_x0 + w, yext = 2 * VIS_Y0 + VIS_H;
	const int tfy = (vr & TILE_FLIPY) ? 7 : 0, sc = s_vs.scrollx[which + 1], fround = s_mach == M_FROUND;
	const uint32_t tmask = s_scr_ntiles - 1;
	const int cbase = 512 + (which << 3) * 16;
	const int pri = pri0 | pri1;
	for (int r = 0; r < VIS_H; r++) {
		const int y = VIS_Y0 + r;
		const int sy = fy ? (yext - 1 - y + s_vs.scrolly[which + 1]) & 511 : (y + s_vs.scrolly[which + 1]) & 511;
		const uint16_t *row = s_vs.videoram[which] + (sy >> 3) * 64;
		const int py = (sy & 7) ^ tfy;
		uint16_t *d = bm + r * rbx_video.pitch;
		uint8_t *pr = s_pri[r];
		int i = 0, sx = fx ? (xext - 1 - s_vis_x0 + sc) & 511 : (s_vis_x0 + sc) & 511;
		while (i < w) {
			const uint16_t data = row[sx >> 3];
			const uint32_t code = fround ? (uint32_t)s_vs.gfx_bank[(data >> 11) & 3] << 11 | (data & 0x7ff) : data & 0x1fff;
			const uint8_t *src = s_scrgfx + (code & tmask) * 64 + py * 8;
			const uint16_t base = (uint16_t)(cbase + (data >> 13) * 16);
			const uint8_t pv = (data & 0x8000) ? pri1 : pri0;
			const int px0 = sx & 7;
			int n = fx ? px0 + 1 : 8 - px0;   // (the pixels of this tile in the row, rightwards or leftwards)
			if (n > w - i)
				n = w - i;
			if (!opaque && !row8(src)) {   // (a transparent tile row)
			} else if (!fx) {
				const uint8_t *sp = src + px0;
				if (opaque)
					for (int k = 0; k < n; k++)
						d[i + k] = (uint16_t)(base + sp[k]);
				else
					for (int k = 0; k < n; k++)
						if (sp[k])
							d[i + k] = (uint16_t)(base + sp[k]);
				if (pri && pv)
					for (int k = 0; k < n; k++)
						if (sp[k])
							pr[i + k] |= pv;
			} else
				for (int k = 0; k < n; k++) {
					const int p = src[px0 - k];
					if (p || opaque)
						d[i + k] = (uint16_t)(base + p);
					if (p && pv)
						pr[i + k] |= pv;
				}
			i += n;
			sx = fx ? (sx - n) & 511 : (sx + n) & 511;
		}
	}
}

// (sprite_callback) a sprite's words from its pen address: contiguous within a sprite
static const uint16_t *sprite_words(uint32_t pa)
{
	if (s_mach == M_FROUND)
		return s_gfxrom + (pa & 0x7ffff);
	switch ((pa >> 18) & 3) {
	case 0: case 1: return s_gfxrom + (pa & 0x7ffff);
	case 2: return s_gfxrom + ((0x80000 + (pa & 0x3ffff) + ((pa >> 20 & 1) ? 0x40000 : 0)) & (s_gfxrom_words - 1));
	default: return s_vs.sprite_gfx_ram + (pa & 0xffff);
	}
}

static void draw_sprites(uint16_t *bm)
{
	const int vr = s_vs.video_register;
	const int x0 = s_vis_x0, x1 = s_vis_x0 + s_vis_w - 1, y0 = VIS_Y0, y1 = VIS_Y0 + VIS_H - 1;
	for (int o = 0x800 - 4; o >= 0; o -= 4) {
		const uint16_t *source = s_vs.sprites + o;
		const uint16_t attributes = source[3];
		uint16_t code = source[0];
		if (code == 0xffff || !(attributes & 0x8000))
			continue;
		int xpos = source[1], ypos = source[2];
		const int pal_base = 0x100 + ((attributes & 0xf) << 4);
		const int height = 16 << ((attributes >> 6) & 3), width = 16 << ((attributes >> 4) & 3);
		int flipy = (attributes >> 9) & 1, flipx = (attributes >> 8) & 1;
		if ((height & width) == 64)
			code &= ~8;
		else if ((height & width) == 32)
			code &= ~3;
		else if ((height | width) == 48)
			code &= ~1;
		const uint32_t pen_addr = (uint32_t)code << 6;
		if (vr & FLIPY) {
			if (ypos > 65000)
				ypos -= 65536;
			ypos = 256 - ypos - height;
			flipy = !flipy;
		}
		if (vr & FLIPX) {
			if (xpos > 65000)
				xpos -= 65536;
			xpos = 320 - xpos - width;
			flipx = !flipx;
		}
		if (xpos >= 320)
			xpos -= 65536;
		if (ypos >= 256)
			ypos -= 65536;
		if (xpos > x1 || xpos + width - 1 < x0 || ypos > y1 || ypos + height - 1 < y0)
			continue;
		const int pitch = width >> 2;
		const uint16_t *words = sprite_words(pen_addr);
		// the sprite's columns c on the screen: x = xpos + (flipx ? width - 1 - c : c)
		int c0 = 0, c1 = width - 1;
		if (!flipx) {
			if (xpos < x0) c0 = x0 - xpos;
			if (xpos + width - 1 > x1) c1 = x1 - xpos;
		} else {
			if (xpos + width - 1 > x1) c0 = xpos + width - 1 - x1;
			if (xpos < x0) c1 = xpos + width - 1 - x0;
		}
		for (int yy = 0; yy < height; yy++) {
			const int sy = flipy ? ypos + height - 1 - yy : ypos + yy;
			if (sy < y0 || sy > y1)
				continue;
			const uint16_t *wr = words + yy * pitch;
			uint16_t *dest = bm + (sy - y0) * rbx_video.pitch - x0;
			uint8_t *pdest = s_pri[sy - y0] - x0;
			for (int c = c0; c <= c1; c++) {
				const int pixel = (wr[c >> 2] >> (12 - 4 * (c & 3))) & 15;
				if (!pixel)
					continue;
				const int sx = flipx ? xpos + width - 1 - c : xpos + c;
				const uint8_t pd = pdest[sx];
				if (pd & SPRITE_OCCUPIED)
					continue;
				pdest[sx] = (uint8_t)(pd | SPRITE_OCCUPIED);
				if (pixel == 15) {   // (shadow)
					if (!(pd & BG_NO_SHADOW))
						dest[sx] |= 0x400;
				} else if (!(pd & BG_OVER_SPRITES))
					dest[sx] = (uint16_t)(pal_base + pixel);
			}
		}
	}
}

static void draw_fixed(uint16_t *bm)
{
	const int vr = s_vs.video_register, fx = vr & FLIPX, fy = vr & FLIPY;
	for (int r = 0; r < VIS_H; r++) {
		const int y = VIS_Y0 + r, ty = fy ? (2 * VIS_Y0 + VIS_H - 1 - y) & 255 : y;
		const uint16_t *row = s_vs.fixram + ((ty >> 3) & 31) * 64;
		uint16_t *d = bm + r * rbx_video.pitch;
		for (int i = 0; i < s_vis_w; i += 8) {   // (the visible area starts on a tile: x0 0 or 8)
			const int x = s_vis_x0 + i, tx = fx ? (2 * s_vis_x0 + s_vis_w - 1 - x) & 511 : x;
			const uint16_t attr = row[(tx >> 3) & 63];
			const int py = (ty & 7) ^ ((attr >> 14 & 1) ? 7 : 0);
			const uint8_t *src = s_fixgfx + (attr & 0x1ff) * 64 + py * 8;
			if (!row8(src))
				continue;
			const int rev = ((attr >> 13) & 1) ^ (fx ? 1 : 0);   // (the tile's flip x, and a flipped screen's)
			const uint16_t base = (uint16_t)(((attr >> 9) & 15) * 16);
			const int n = s_vis_w - i < 8 ? s_vis_w - i : 8;
			for (int k = 0; k < n; k++) {
				const int p = src[rev ? 7 - k : k];
				if (p)
					d[i + k] = (uint16_t)(base + p);
			}
		}
	}
}

static void tw_render(void)
{
	for (int i = 0; i < 1024; i++) {   // (xBGR_555 of two bytes; the shadows x 0.4 at 1024)
		const uint16_t w = (uint16_t)(s_vs.pal8[2 * i] << 8 | s_vs.pal8[2 * i + 1]);
		const int r = w & 31, g = (w >> 5) & 31, b = (w >> 10) & 31;
		const int r8 = r << 3 | r >> 2, g8 = g << 3 | g >> 2, b8 = b << 3 | b >> 2;
		rbx_video.palette[i] = (uint32_t)(r8 << 16 | g8 << 8 | b8);
		rbx_video.palette[i + 1024] = (uint32_t)((r8 * 4 / 10) << 16 | (g8 * 4 / 10) << 8 | (b8 * 4 / 10));
	}
	uint16_t *bm = rbx_video.bitmap;
	memset(s_pri, 0, sizeof(s_pri));
	switch ((s_vs.video_register >> 2) & 3) {   // (the 007789 PAL's modes)
	case 0:
		draw_scroll(bm, 1, 1, 0, 0);
		draw_scroll(bm, 0, 0, 0, 0);
		break;
	case 1:
		draw_scroll(bm, 1, 1, 0, 0);
		draw_scroll(bm, 0, 0, BG_OVER_SPRITES, BG_OVER_SPRITES);
		break;
	case 2:
		draw_scroll(bm, 0, 1, BG_NO_SHADOW, BG_NO_SHADOW);
		draw_scroll(bm, 1, 0, 0, 0);
		break;
	case 3:
		draw_scroll(bm, 0, 1, 0, 0);
		draw_scroll(bm, 1, 0, 0, BG_OVER_SPRITES | BG_NO_SHADOW);
		break;
	}
	draw_sprites(bm);
	draw_fixed(bm);
}

// ---- the video's registers and sprites ----
static void set_sprite_timer(void)
{
	if (t.sprite_busy)
		return;
	t.sprite_busy = 1;
	t.sprite_busy_lines = 4;   // (frame_period / height * 4)
}

static void spriteram_process(void)
{
	const uint16_t dx = t.scrollx[0], dy = t.scrolly[0];
	set_sprite_timer();
	for (int i = 0x1800; i < 0x2000; i++)
		t.spriteram[0][i] = 0xffff;
	for (int o = 0; o < 0x1800; o += 0x50 / 2) {
		const uint16_t *source = t.spriteram[0] + o;
		const uint16_t priority = source[0];
		if (!(priority & 0x8000))
			continue;
		uint16_t *dest = t.spriteram[0] + (0x1800 | ((priority & 0xff) << 2));
		const uint32_t xpos = (uint32_t)source[4] << 16 | source[5], ypos = (uint32_t)source[6] << 16 | source[7];
		dest[0] = source[3];
		dest[1] = (uint16_t)((xpos >> 8) - dx);
		dest[2] = (uint16_t)((ypos >> 8) - dy);
		dest[3] = (uint16_t)(0x8000 | (source[2] & 0x03ff));
	}
	t.need_process_spriteram = 0;
}

static void sprite_process_enable_w(uint8_t data)
{
	const uint8_t old = t.sprite_process_enable;
	t.sprite_process_enable = data;
	if (!old && data)
		spriteram_process();
}

static void screen_vblank(void)
{
	set_sprite_timer();
	if (t.sprite_process_enable) {
		if (t.need_process_spriteram)
			spriteram_process();
		t.need_process_spriteram = 1;
		memcpy(t.spriteram[1] + 0x1800, t.sprite_buffer, 0x800 * 2);
		memcpy(t.sprite_buffer, t.spriteram[0] + 0x1800, 0x800 * 2);
	} else
		memcpy(t.spriteram[1], t.spriteram[0], 0x2000 * 2);
	// (virq_callback)
	if (t.cpua_register & 0x20)
		m68k_multi_hold_irq(0, 5);
	if (s_mach != M_FROUND && (t.cpub_register & 0x02))
		m68k_multi_hold_irq(1, 5);
}

static void video_register_w(int offset, uint16_t data, uint16_t mask)
{
	uint16_t *r;
	switch (offset) {
	case 0: r = &t.video_register; break;
	case 1: r = &t.scrollx[0]; break;
	case 2: r = &t.scrolly[0]; break;
	case 3: r = &t.scrollx[1]; break;
	case 4: r = &t.scrolly[1]; break;
	case 5: r = &t.scrollx[2]; break;
	case 6: r = &t.scrolly[2]; break;
	default: return;
	}
	*r = (uint16_t)((*r & ~mask) | (data & mask));
}

// ---- the sound board a frame behind on the worker ----
#define Q_MAX 2048
enum { Q_ADV, Q_LATCH, Q_IRQ };
typedef struct { uint16_t kind, arg; uint32_t v; } q_ev_t;
static q_ev_t (*s_q_hp)[2][Q_MAX];
#define s_q (*s_q_hp)
static int s_qn[2], s_qcur;
static int32_t s_out_l[2048], s_out_r[2048];
static int s_out_n, s_out_pos;
static volatile int s_job_frame;

static void q_put(int kind, uint32_t v)
{
	const int c = s_qcur;
	if (s_qn[c] < Q_MAX) {
		s_q[c][s_qn[c]].kind = (uint16_t)kind;
		s_q[c][s_qn[c]].v = v;
		s_qn[c]++;
	}
}

// ---- the registers ----
static void cpua_register_w(uint16_t v, uint16_t mask)
{
	const uint16_t old = t.cpua_register;
	t.cpua_register = (uint16_t)((old & ~mask) | (v & mask));
	if (t.cpua_register == old)
		return;
	const int rising = ~old & t.cpua_register;
	if (rising & 0x08)
		q_put(Q_IRQ, 0);
	sprite_process_enable_w((uint8_t)(~t.cpua_register >> 6 & 1));
	if (s_mach != M_FROUND && (rising & 0x10))
		m68k_multi_hold_irq(1, 6);
}
static void cpub_register_w(uint16_t v, uint16_t mask)
{
	const uint16_t old = t.cpub_register;
	t.cpub_register = (uint16_t)((old & ~mask) | (v & mask));
	if (t.cpub_register == old)
		return;
	if (~old & t.cpub_register & 0x01)
		m68k_multi_hold_irq(0, 6);
}
static void zipram_w(uint32_t o, uint16_t v, uint16_t mask)
{
	const uint16_t old = t.zipram[o];
	t.zipram[o] = (uint16_t)((old & ~mask) | (v & mask));
	if (old != t.zipram[o]) {
		s_dirty[o >> 4] = 1;
		s_any_dirty = 1;
	}
}

// ---- CPU A's bus ----
static uint16_t main_r16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x080000 && a < 0x081000)
		return t.pal8[(a - 0x080000) >> 1];
	switch (a) {
	case 0x0a0000: return (uint16_t)rbx_port_read(s_port_sys);
	case 0x0a0002: return (uint16_t)rbx_port_read(s_port_p[0]);
	case 0x0a0004: return (uint16_t)rbx_port_read(s_port_p[1]);
	case 0x0a0006: return s_mach == M_FROUND ? 0 : (uint16_t)rbx_port_read(s_port_p[2]);
	case 0x0a0010: return (uint16_t)rbx_port_read(s_port_dsw[1]);
	case 0x0a0012: return (uint16_t)rbx_port_read(s_port_dsw[0]);
	case 0x0a0018: return (uint16_t)rbx_port_read(s_port_dsw[2]);
	case 0x0c000e: return t.sprite_busy;
	}
	if (s_mach == M_CUEBRICKJ && a >= 0x0b0000 && a < 0x0b0400) {
		const uint8_t *p = s_nv + t.nvbank * 0x400 + (a - 0x0b0000);
		return (uint16_t)(p[0] << 8 | p[1]);
	}
	if (s_mach == M_FROUND && a >= 0x500000 && a < 0x600000) {   // (the tiles ROM)
		const uint32_t o = a - 0x500000;
		return o + 1 < s_tiles_size ? (uint16_t)(s_tiles[o] << 8 | s_tiles[o + 1]) : 0;
	}
	return 0;
}
static uint8_t main_r8(uint32_t a)
{
	const uint16_t w = main_r16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}
static void main_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a >= 0x080000 && a < 0x081000) {
		if (mask & 0x00ff)
			t.pal8[(a - 0x080000) >> 1] = (uint8_t)v;
	} else if (a == 0x0a0000)
		cpua_register_w(v, mask);
	else if (a == 0x0a0008 && (mask & 0x00ff))
		q_put(Q_LATCH, (uint8_t)v);
	else if (a >= 0x0c0000 && a < 0x0c0010)
		video_register_w((int)((a - 0x0c0000) >> 1), v, mask);
	else if (s_mach == M_FROUND && a == 0x0e0000) {   // (gfx_bank_w)
		if (mask & 0x00ff) {
			t.gfx_bank[0] = v & 15;
			t.gfx_bank[1] = (v >> 4) & 15;
		}
		if (mask & 0xff00) {
			t.gfx_bank[2] = (v >> 8) & 15;
			t.gfx_bank[3] = (v >> 12) & 15;
		}
	} else if (s_mach == M_CUEBRICKJ && a >= 0x0b0000 && a < 0x0b0400) {
		uint8_t *p = s_nv + t.nvbank * 0x400 + (a - 0x0b0000);
		if (mask & 0xff00) p[0] = (uint8_t)(v >> 8);
		if (mask & 0x00ff) p[1] = (uint8_t)v;
	} else if (s_mach == M_CUEBRICKJ && a == 0x0b0400 && (mask & 0xff00))
		t.nvbank = (uint8_t)((v >> 8) & 0x1f);
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v) { main_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00); }

// ---- CPU B's bus ----
static uint16_t sub_r16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x700000 && a < 0x780000) {   // (the gfx ROM's bank: gfxrom + 0x100000 + entry x 0x80000)
		const uint32_t w = (0x100000 + ((t.cpub_register >> 2) & 1) * 0x80000 + (a - 0x700000)) >> 1;
		return w < s_gfxrom_words ? s_gfxrom[w] : 0;
	}
	return 0;
}
static uint8_t sub_r8(uint32_t a)
{
	const uint16_t w = sub_r16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}
static void sub_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a == 0x0a0000)
		cpub_register_w(v, mask);
	else if (a >= 0x500000 && a < 0x540000)
		zipram_w((a - 0x500000) >> 1, v, mask);
}
static void sub_w16(uint32_t a, uint16_t v) { sub_w16m(a, v, 0xffff); }
static void sub_w8(uint32_t a, uint8_t v) { sub_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00); }

// ---- the sound board (sound_map) ----
static void k007232_port(uint8_t data)   // (volume_callback)
{
	k007232_set_volume(&s_k232, 0, (data >> 4) * 0x11, 0);
	k007232_set_volume(&s_k232, 1, 0, (data & 0x0f) * 0x11);
}

static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0x8800)
		return t.zram[a - 0x8000];
	if (a == 0xa000)
		return t.soundlatch;
	if (a >= 0xb000 && a < 0xb00e)
		return k007232_read(&s_k232, a - 0xb000);
	if (a == 0xc000 || a == 0xc001)
		return rbx_ym_read(0, a & 1);
	if (a == 0xf000)
		return (uint8_t)upd7759_busy_r(&s_upd);
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8800)
		t.zram[a - 0x8000] = v;
	else if (a == 0x9000)
		upd7759_reset_w(&s_upd, (v >> 1) & 1);
	else if (a >= 0xb000 && a < 0xb00e)
		k007232_write(&s_k232, a - 0xb000, v);
	else if (a == 0xc000 || a == 0xc001)
		rbx_ym_write(0, a & 1, v);
	else if (a == 0xd000)
		upd7759_port_w(&s_upd, v);
	else if (a == 0xe000)
		upd7759_start_w(&s_upd, !(v & 1));
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void sound_run(int n)
{
	if (n <= 0)
		return;
	t.z_frac += (uint64_t)Z80_CLOCK * (uint64_t)n;
	const unsigned long cyc = (unsigned long)(t.z_frac / SND_RATE);
	t.z_frac -= (uint64_t)cyc * SND_RATE;
	int line = t.z80_irq;
	t.z_end += cyc;
	z80_run(&s_z, (unsigned long)t.z_end, &line, 1);
	t.z80_irq = (uint8_t)line;
	if (n > 64)
		n = 64;
	int32_t yl[96], yr[96], up[64];
	memset(yl, 0, sizeof(yl));
	memset(yr, 0, sizeof(yr));
	t.ym_frac += (uint64_t)CHIP_CLOCK * (uint64_t)n;
	const uint32_t yc = (uint32_t)(t.ym_frac / SND_RATE);
	t.ym_frac -= (uint64_t)yc * SND_RATE;
	const int yn = rbx_ym_advance_lr(0, yc, yl, yr, 96);
	upd7759_render(&s_upd, up, n);
	for (int i = 0; i < n; i++) {
		t.k232_frac += CHIP_CLOCK / 128;
		while (t.k232_frac >= SND_RATE) {
			t.k232_frac -= SND_RATE;
			k007232_sample(&s_k232, &t.k232_a, &t.k232_b);
		}
		const int k = yn > 0 ? i * yn / n : -1;
		// (YM 1.0 to its side; K007232 0.12 and uPD7759 0.20 to both)
		const int32_t c = (t.k232_a + t.k232_b) * 12 / 100 + up[i] / 5;
		if (s_out_pos < 2048) {
			s_out_l[s_out_pos] = (k >= 0 ? yl[k] : 0) + c;
			s_out_r[s_out_pos] = (k >= 0 ? yr[k] : 0) + c;
			s_out_pos++;
		}
	}
}

static void job(void)
{
	if (!__atomic_load_n(&s_job_frame, __ATOMIC_ACQUIRE))
		return;
	const int b = s_qcur ^ 1;
	s_out_pos = 0;
	for (int i = 0; i < s_qn[b]; i++) {
		const q_ev_t *e = &s_q[b][i];
		switch (e->kind) {
		case Q_ADV: sound_run((int)e->v); break;
		case Q_LATCH: t.soundlatch = (uint8_t)e->v; break;
		case Q_IRQ: t.z80_irq = 1; break;
		}
	}
	s_qn[b] = 0;
	s_out_n = s_out_pos;
	__atomic_store_n(&s_job_frame, 0, __ATOMIC_RELEASE);
}

static void sound_frame(void)
{
	rbx_aux_sync();
	for (int i = 0; i < s_out_n; i++)
		rbx_audio_put_stereo(s_out_l[i], s_out_r[i]);
	s_out_n = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void tw_exit(void);
static void tw_reset(void);

static int tw_init(const rbx_game *g)
{
	if (!s_nv_hp && !(s_nv_hp = calloc(1, sizeof(*s_nv_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_pri_hp && !(s_pri_hp = calloc(1, sizeof(*s_pri_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t pcm_size, upd_size, fixed_size, gfx_size;
	const char *m = g->machine;
	s_mach = !strcmp(m, "devilw") ? M_DEVILW : !strcmp(m, "miaj") ? M_MIAJ : !strcmp(m, "cuebrickj") ? M_CUEBRICKJ :
	         !strcmp(m, "fround") ? M_FROUND : M_TWIN16;
	s_slices = s_mach == M_DEVILW ? 10 : 2;   // (set_maximum_quantum: 60000 Hz for devilw (its watchdog), else 6000)
	s_vis_x0 = s_mach == M_MIAJ || s_mach == M_CUEBRICKJ ? 8 : 0;
	s_vis_w = s_mach == M_MIAJ || s_mach == M_CUEBRICKJ ? 304 : 320;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_subrom = s_mach == M_FROUND ? NULL : rbx_region("sub", &s_subrom_size);
	s_data = s_mach == M_FROUND ? NULL : rbx_region("data", &s_data_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_fixed = rbx_region("fixed", &fixed_size);
	uint8_t *gfx = rbx_region("gfxrom", &gfx_size);
	s_tiles = s_mach == M_FROUND ? rbx_region("tiles", &s_tiles_size) : NULL;
	const uint8_t *pcm = rbx_region("k007232", &pcm_size);
	const uint8_t *upd = rbx_region("upd", &upd_size);
	if (!s_rom || !s_zrom || !s_fixed || !gfx || !pcm || !upd || (s_mach == M_FROUND ? !s_tiles : !s_subrom)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	m68k_bus_swap16(gfx, gfx_size);
	s_gfxrom = (uint16_t *)gfx;
	s_gfxrom_words = gfx_size / 2;
	s_fixgfx = (uint8_t *)malloc(512 * 64);
	s_scr_ntiles = s_mach == M_FROUND ? s_tiles_size / 32 : 0x2000;
	s_scrgfx = (uint8_t *)calloc((size_t)s_scr_ntiles * 64, 1);
	if (!s_fixgfx || !s_scrgfx || m68k_multi_init(s_mach == M_FROUND ? 1 : 2)) {
		rbx_fail("%s: out of memory", g->set);
		tw_exit();
		return -1;
	}
	decode_rom_tiles(s_fixgfx, s_fixed, fixed_size / 32 < 512 ? fixed_size / 32 : 512);
	if (s_mach == M_FROUND)
		decode_rom_tiles(s_scrgfx, s_tiles, s_scr_ntiles);
	m68k_init();
	m68k_bus_swap16(s_rom, s_rom_size);
	if (s_subrom)
		m68k_bus_swap16(s_subrom, s_subrom_size);
	if (s_data)
		m68k_bus_swap16(s_data, s_data_size);
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	for (int c = 0; c < (s_mach == M_FROUND ? 1 : 2); c++) {
		m68k_bus_reset_map();
		m68k_bus_map(0x040000, 0x043fff, s_comram, sizeof(s_comram), 1, 1);
		m68k_bus_map(0x060000, 0x063fff, s_ram[c], sizeof(s_ram[c]), 1, 1);
		if (!c) {
			m68k_bus_map(0x000000, 0x03ffff, s_rom, s_rom_size, 1, 0);
			m68k_bus_map(0x100000, 0x103fff, t.fixram, sizeof(t.fixram), 1, 1);
			m68k_bus_map(0x120000, 0x121fff, t.videoram[0], sizeof(t.videoram[0]), 1, 1);
			m68k_bus_map(0x122000, 0x123fff, t.videoram[1], sizeof(t.videoram[1]), 1, 1);
			m68k_bus_map(0x140000, 0x143fff, t.spriteram[0], sizeof(t.spriteram[0]), 1, 1);
			if (s_mach == M_FROUND)
				m68k_bus_map(0x600000, 0x6fffff, s_gfxrom, gfx_size, 1, 0);
		} else {
			m68k_bus_map(0x000000, 0x03ffff, s_subrom, s_subrom_size, 1, 0);
			m68k_bus_map(0x080000, 0x09ffff, s_data, s_data_size, 1, 0);
			m68k_bus_map(0x400000, 0x403fff, t.spriteram[0], sizeof(t.spriteram[0]), 1, 1);
			m68k_bus_map(0x480000, 0x481fff, t.videoram[0], sizeof(t.videoram[0]), 1, 1);
			m68k_bus_map(0x482000, 0x483fff, t.videoram[1], sizeof(t.videoram[1]), 1, 1);
			m68k_bus_map(0x500000, 0x53ffff, t.zipram, sizeof(t.zipram), 1, 0);
			m68k_bus_map(0x600000, 0x6fffff, s_gfxrom, gfx_size < 0x100000 ? gfx_size : 0x100000, 1, 0);
			m68k_bus_map(0x780000, 0x79ffff, t.sprite_gfx_ram, sizeof(t.sprite_gfx_ram), 1, 1);
		}
		m68k_bus.read8 = c ? sub_r8 : main_r8;
		m68k_bus.read16 = c ? sub_r16 : main_r16;
		m68k_bus.write8 = c ? sub_w8 : main_w8;
		m68k_bus.write16 = c ? sub_w16 : main_w16;
		m68k_multi_capture(c);
	}
	m68k_multi_finish();
	k007232_init(&s_k232, pcm, pcm_size, k007232_port);
	upd7759_init(&s_upd, upd, upd_size, UPD_CLOCK, SND_RATE);
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = CHIP_CLOCK;
	yc.gain[0] = yc.gain[1] = 256;
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: YM2151", g->set);
		tw_exit();
		return -1;
	}
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_p[2] = rbx_port_find("P3");
	s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	s_port_dsw[2] = rbx_port_find("DSW3");
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(s_vis_w, VIS_H, 2048)) {
		rbx_fail("%s: out of memory", g->set);
		tw_exit();
		return -1;
	}
	memset(s_nv, 0, sizeof(s_nv));
	s_nv_path[0] = 0;
	if (s_mach == M_CUEBRICKJ) {
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			FILE *f = fopen(s_nv_path, "rb");
			if (f) {
				if (fread(s_nv, 1, sizeof(s_nv), f) != sizeof(s_nv))
					memset(s_nv, 0, sizeof(s_nv));
				fclose(f);
			}
		}
	}
	tw_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void tw_exit(void)
{
	if (s_nv_path[0]) {
		FILE *f = fopen(s_nv_path, "wb");
		if (f) {
			fwrite(s_nv, 1, sizeof(s_nv), f);
			fclose(f);
		}
		s_nv_path[0] = 0;
	}
	rbx_aux_set_job(NULL);
	m68k_multi_exit();
	rbx_ym_exit(0);
	free(s_fixgfx);
	free(s_scrgfx);
	s_fixgfx = s_scrgfx = NULL;
	free(s_nv_hp);
	s_nv_hp = NULL;
	free(s_pri_hp);
	s_pri_hp = NULL;
	free(s_q_hp);
	s_q_hp = NULL;
}

static void tw_reset(void)
{
	rbx_aux_sync();
	rbx_video_wait_idle();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_comram, 0, sizeof(s_comram));
	for (int i = 0; i < 0x800; i++)
		t.sprite_buffer[i] = 0xffff;
	t.sprite_process_enable = 1;
	if (s_mach != M_FROUND)
		memset(s_scrgfx, 0, (size_t)s_scr_ntiles * 64);
	memset(s_dirty, 0, sizeof(s_dirty));   // (another game's zip RAM writes: not over The Final Round's ROM tiles)
	s_any_dirty = 0;
	k007232_reset(&s_k232);
	upd7759_reset(&s_upd);
	upd7759_reset_w(&s_upd, 0);   // (machine_reset)
	upd7759_start_w(&s_upd, 1);
	rbx_ym_reset(0);
	z80_init(&s_z);
	s_z.read_byte = z_rd;
	s_z.write_byte = z_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	for (int p = 0; p < 256; p++)
		s_zmap[p] = p < 0x80 && (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
	for (int p = 0; p < 8; p++)
		s_zmap[0x80 + p] = t.zram + p * 256;
	s_z.read_map = s_zmap;
	m68k_multi_reset();
	for (int c = 0; c < (s_mach == M_FROUND ? 1 : 2); c++)
		m68k_multi_set_asserted(c, 0);
}

static void tw_frame(void)
{
	const int ncpu = s_mach == M_FROUND ? 1 : 2;
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {
			rbx_video_wait_idle();
			capture();
			rbx_video_frame_done();
			screen_vblank();
		}
		t.cpu_frac += (uint64_t)CPU_CLOCK * HTOTAL;
		const int cyc = (int)(t.cpu_frac / PIX_CLOCK);
		t.cpu_frac -= (uint64_t)cyc * PIX_CLOCK;
		for (int k = 0; k < s_slices; k++) {
			const int n = cyc * (k + 1) / s_slices - cyc * k / s_slices;
			for (int c = 0; c < ncpu; c++) {
				m68k_multi_switch(c);
				int left = n;
				while (left > 0)
					left -= m68k_execute(left);
			}
		}
		m68k_multi_switch(0);
		if (t.sprite_busy && t.sprite_busy_lines && !--t.sprite_busy_lines)   // (sprite_tick)
			t.sprite_busy = 0;
		t.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(t.snd_frac / PIX_CLOCK);
		t.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		if (ns > 0)
			q_put(Q_ADV, (uint32_t)ns);
	}
	sound_frame();

	t.frame++;
}

static int tw_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rbx_video_wait_idle();
	const int ncpu = s_mach == M_FROUND ? 1 : 2;
	uint32_t asserted[2] = { m68k_multi_asserted(0), ncpu > 1 ? m68k_multi_asserted(1) : 0 };
	rc |= m68k_multi_state(ctx, load);
	rc |= rbx_state_io(ctx, "tw_irq", asserted, sizeof(asserted));
	rc |= rbx_state_io(ctx, "tw", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "tw_ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "tw_com", s_comram, sizeof(s_comram));
	if (s_mach == M_CUEBRICKJ)
		rc |= rbx_state_io(ctx, "tw_nv", s_nv, sizeof(s_nv));
	rc |= k007232_state(&s_k232, ctx, load, "tw_k232");
	rc |= upd7759_state(&s_upd, ctx, load, "tw_upd");
	rc |= rbx_ym_state(0, ctx, load);
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "tw_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "tw_outl", s_out_l, sizeof(s_out_l));
	rc |= rbx_state_io(ctx, "tw_outr", s_out_r, sizeof(s_out_r));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "tw_z80", &z, sizeof(z));
	if (!rc && load) {
		m68k_multi_set_asserted(0, asserted[0]);
		if (ncpu > 1)
			m68k_multi_set_asserted(1, asserted[1]);
		s_out_n = qn >= 0 && qn <= 2048 ? qn : 0;
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
		if (s_mach != M_FROUND) {
			for (uint32_t i = 0; i < 0x2000; i++)
				decode_zip_tile(i);
			memset(s_dirty, 0, sizeof(s_dirty));
			s_any_dirty = 0;
		}
		z.read_byte = z_rd;
		z.write_byte = z_wr;
		z.port_in = z_in;
		z.port_out = z_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		z.op_rom = NULL;
		s_z = z;
	}
	return rc ? -1 : 0;
}

static int tw_save(void *ctx) { return tw_state(ctx, 0); }
static int tw_load(void *ctx) { return tw_state(ctx, 1); }
static int tw_supported(const char *m)
{
	return !strcmp(m, "twin16") || !strcmp(m, "devilw") || !strcmp(m, "miaj") || !strcmp(m, "cuebrickj") ||
	       !strcmp(m, "fround");
}

const rbx_driver rbx_drv_twin16 = { "Konami Twin 16", tw_init, tw_exit, tw_reset, tw_frame, tw_save, tw_load,
                                    tw_supported, tw_render, NULL };
