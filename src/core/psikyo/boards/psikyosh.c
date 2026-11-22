/*  Psikyo PS3-V1 / PS5 / PS5V2 (SH-2 boards: Sol Divide, Strikers 1945 II, Daraku Tenshi, Space Bomber, Gunbird 2,
    Strikers 1945 III, Dragon Blaze, Tetris The Grand Master 2, Gunbarich, Mahjong G-Taste). Ported from MAME 0.289
    src/mame/psikyo/psikyosh.cpp, psikyosh.h, psikyosh_v.cpp (BSD-3-Clause, copyright-holders: David Haywood,
    Paul Priest).
    An SH-2 (SH7604) at 28.6 MHz, the PS6406B video chip (32 banks of 0x800 bytes for sprites, the sprite list,
    the line fill / blend values and up to four 512x256 / 512x512 tile layers of 4 or 8 bpp 16x16 tiles with per-line
    scroll, zoom, alpha and bank; zoomed, alpha blended sprites with a priority z-buffer), RGB output; a YMF278B and
    a 93C56 EEPROM. 60 Hz, 320x224 (TGM2: 61.68 Hz, 320x240).
    Drawn a line at a time with MAME's screen_update order (fill, then for each priority 0-7: sprites, layers, blend);
    the sprite list is the one taken at the previous vblank (screen_vblank after screen_update), as in MAME.
    The bootleg Strikers 1945 III sets (MAME: not working) are not emulated.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/sh2/sh2.h"
#include "../../cpu/sh2/sh2idle.h"
#include "../../sound/rbx_ym.h"
#include "../../machine/eepromser.h"
#include "../../video/decoband.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define MASTER_CLOCK 57272727
#define CPU_CLOCK (MASTER_CLOCK / 2)
#define VIS_W 320
#define SPR_MAX (0x800 / 2)

enum { B_PS3, B_PS5 };

// ---- ROMs, configuration ----
static uint8_t *s_rom, *s_gfx, *s_ymf;
static uint32_t s_rom_size, s_gfx_size, s_ymf_size;
static uint32_t s_n4, s_n8;   // tiles of the gfx region at 4 and 8 bpp
static int s_board, s_vis_h, s_lines;
static uint32_t s_frame_mhz;
static uint32_t s_vbase, s_iobase, s_ymfbase;   // video chip, inputs / EEPROM, sound chip
static int s_port_in, s_port_jp4, s_port_ctrl, s_port_mj;
static sh2_t s_cpu;
static eepser_t s_eeprom;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;

typedef struct {
	int16_t ypos, xpos;
	uint8_t high, wide, flpy, flpx, spr_pri, bg_pri, zoomy, zoomx, dpth, alpha;
	uint32_t tnum;
	uint16_t colr;
} spr_t;

static struct psh_state {   // (on the heap while the game runs, ADR-040)
	uint32_t ram[0x100000 / 4];     // main RAM (host-order words, as all the SH-2's memory)
	uint32_t spr[0x10000 / 4];      // sprite / tile / register banks
	uint32_t pal[0x5000 / 4];       // RGBx 888
	uint32_t zoom[0x1000 / 4];      // the sprite zoom table (0x200 bytes used)
	uint32_t vidregs[8];
	spr_t list[SPR_MAX];            // the sprite list taken at the last vblank
	int32_t list_n;
	int32_t gfxbank;
	uint64_t ym_frac, cpu_frac;
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)

// ---- the drawing snapshot ----
static struct {
	uint32_t spr[0x10000 / 4];
	uint32_t pens[0x1400];          // 0x00RRGGBB
	uint16_t zoom[0x100];
	uint32_t vidregs[8];
	spr_t list[SPR_MAX];
	int list_n;
} *s_snap;
static int s_565;

static inline int pal6bit(int x) { x &= 0x3f; return (x << 2) | (x >> 4); }

// MAME alpha_blend_r32
static inline uint32_t blend(uint32_t d, uint32_t s, uint32_t level)
{
	const uint32_t inv = 256 - level;
	return ((((s & 0x0000ff) * level + (d & 0x0000ff) * inv) >> 8)) |
	       ((((s & 0x00ff00) * level + (d & 0x00ff00) * inv) >> 8) & 0x00ff00) |
	       ((((s & 0xff0000) * level + (d & 0xff0000) * inv) >> 8) & 0xff0000);
}

// ---- memory ----
static void gfxbank_map(void)
{
	const uint32_t banks = s_gfx_size / 0x20000;
	const uint32_t bank = banks ? (uint32_t)b.gfxbank % banks : 0;
	uint8_t *p = s_gfx + bank * 0x20000;
	sh2_map(&s_cpu, s_vbase + 0x60000, s_vbase + 0x7ffff, p, 0x20000, 1, 0);
	if (s_board == B_PS3)
		sh2_map(&s_cpu, 0x04060000, 0x0407ffff, p, 0x20000, 1, 0);
}

static void vidregs_w(int off, uint32_t data, uint32_t mask)
{
	const uint32_t old = b.vidregs[off];
	b.vidregs[off] = (old & ~mask) | (data & mask);
	if (off == 4 && ((old ^ b.vidregs[off]) & 0xfff)) {   // (the gfx bank for the ROM test)
		b.gfxbank = (int32_t)(b.vidregs[off] & 0xfff);
		gfxbank_map();
	}
}

static uint32_t inputs_r(void)
{
	uint32_t value = rbx_port_read(s_port_in);
	if (s_port_ctrl >= 0 && s_port_mj >= 0 && (rbx_port_read(s_port_ctrl) & 1)) {   // mjgtaste_input_r
		static const uint16_t key_codes[] = {
			0x8000 | 0x0080, 0x8000 | 0x0040, 0x8000 | 0x0010, 0x8000 | 0x0020,
			0x4000 | 0x0080, 0x4000 | 0x0040, 0x4000 | 0x0010, 0x4000 | 0x0020,
			0x1000 | 0x0080, 0x1000 | 0x0040, 0x1000 | 0x0010, 0x1000 | 0x0020,
			0x2000 | 0x0080, 0x2000 | 0x0040, 0x0800 | 0x0080, 0x2000 | 0x0020,
			0x2000 | 0x0010, 0x0800 | 0x0040, 0x0800 | 0x0010, 0x0400 | 0x0080 };
		const int n = (int)(sizeof(key_codes) / sizeof(key_codes[0]));
		uint32_t keys = rbx_port_read(s_port_mj);
		if (~value & 0x01000000)
			keys |= 1u << (n - 1);
		value |= 0xffff0000;
		for (int i = 0; i < 32; i++)
			if (keys & (1u << i)) {
				if (i < n)
					value &= ~((uint32_t)key_codes[i] << 16);
				break;
			}
	}
	return value;
}

static uint32_t jp4_r(void)
{
	if (s_port_jp4 < 0)
		return 0xffffffff;
	rbx_port_custom(s_port_jp4, 0x10000000, eepser_do_read_93cxx(&s_eeprom, sh2_total_cycles(&s_cpu)) ? 0x10000000 : 0);
	return rbx_port_read(s_port_jp4);
}

static void eeprom_w(uint8_t data)
{
	const int64_t now = sh2_total_cycles(&s_cpu);
	eepser_di_write(&s_eeprom, (data >> 5) & 1);
	eepser_cs_write(&s_eeprom, (data >> 7) & 1, now);
	eepser_clk_write(&s_eeprom, (data >> 6) & 1, now);
}

// a 32-bit read of the long at a (word aligned), for the narrower accesses
static uint32_t rd32(uint32_t a)
{
	if (a - s_iobase < 8)
		return (a & 4) ? jp4_r() : inputs_r();
	if (a - (s_vbase + 0x5ffe0) < 0x20)
		return b.vidregs[(a >> 2) & 7];
	return 0;
}

static uint8_t r8(void *u, uint32_t a)
{
	(void)u;
	if (a - s_ymfbase < 8)
		return rbx_ym_read(0, a & 7);
	return (uint8_t)(rd32(a & ~3u) >> (24 - 8 * (a & 3)));
}
static uint16_t r16(void *u, uint32_t a)
{
	if (a - s_ymfbase < 8)
		return (uint16_t)(r8(u, a) << 8 | r8(u, a + 1));
	return (uint16_t)(rd32(a & ~3u) >> (16 - 8 * (a & 2)));
}
static uint32_t r32(void *u, uint32_t a)
{
	if (a - s_ymfbase < 8)
		return (uint32_t)r16(u, a) << 16 | r16(u, a + 2);
	return rd32(a);
}

// a write of the bytes of mask (bits of the long at a & ~3)
static void wr32(uint32_t a, uint32_t data, uint32_t mask)
{
	a &= ~3u;
	if (a - s_ymfbase < 8) {
		for (int i = 0; i < 4; i++)
			if (mask & (0xffu << (24 - 8 * i)))
				rbx_ym_write(0, (a + (uint32_t)i) & 7, (uint8_t)(data >> (24 - 8 * i)));
		return;
	}
	if (a == s_iobase + 4) {
		if (mask & 0xff000000)
			eeprom_w((uint8_t)(data >> 24));
		return;
	}
	if (a == s_vbase + 0x5ffdc) {   // irqctrl_w (a write of the long: MAME's handler takes any)
		if (!(data & 0x00c00000))
			sh2_set_irq_line(&s_cpu, 4, SH2_CLEAR);
		return;
	}
	if (a - (s_vbase + 0x5ffe0) < 0x20)
		vidregs_w((int)((a >> 2) & 7), data, mask);
}
static void w8(void *u, uint32_t a, uint8_t d)
{
	(void)u;
	const int sh = 24 - 8 * (int)(a & 3);
	wr32(a, (uint32_t)d << sh, 0xffu << sh);
}
static void w16(void *u, uint32_t a, uint16_t d)
{
	(void)u;
	const int sh = 16 - 8 * (int)(a & 2);
	wr32(a, (uint32_t)d << sh, 0xffffu << sh);
}
static void w32(void *u, uint32_t a, uint32_t d)
{
	(void)u;
	wr32(a, d, 0xffffffffu);
}

static void map_mem(void)
{
	sh2_map(&s_cpu, 0x00000000, 0x000fffff, s_rom, s_rom_size < 0x100000 ? s_rom_size : 0x100000, 1, 0);
	if (s_board == B_PS3) {
		if (s_rom_size > 0x100000)
			sh2_map(&s_cpu, 0x02000000, 0x020fffff, s_rom + 0x100000, s_rom_size - 0x100000, 1, 0);
	} else if (s_rom_size > 0x100000)
		sh2_map(&s_cpu, 0x05000000, 0x0507ffff, s_rom + 0x100000, s_rom_size - 0x100000 < 0x80000 ? s_rom_size - 0x100000 : 0x80000, 1, 0);
	sh2_map(&s_cpu, s_vbase, s_vbase + 0xffff, b.spr, sizeof(b.spr), 1, 1);
	sh2_map(&s_cpu, s_vbase + 0x40000, s_vbase + 0x44fff, b.pal, sizeof(b.pal), 1, 1);
	sh2_map(&s_cpu, s_vbase + 0x50000, s_vbase + 0x50fff, b.zoom, sizeof(b.zoom), 1, 1);
	sh2_map(&s_cpu, 0x06000000, 0x060fffff, b.ram, sizeof(b.ram), 1, 1);
	gfxbank_map();
}

static void ym_irq(int state) { sh2_set_irq_line(&s_cpu, 12, state ? SH2_ASSERT : SH2_CLEAR); }

// ---- video ----
#define VR(n) (s_snap->vidregs[n])
static inline int bg_large(int n) { return ((VR(7) << (4 * n)) & 0x1000) != 0; }
static inline int bg_8bpp(int n) { return ((VR(7) << (4 * n)) & 0x4000) != 0; }
static inline int bg_enable(int n) { return ((VR(7) << (4 * n)) & 0x8000) != 0; }
static inline int bg_type(int n) { return (int)(((VR(6) << (8 * n)) & 0x7f000000) >> 24); }
static inline int bg_line(int n) { return ((VR(6) << (8 * n)) & 0x80000000) != 0; }
static inline int sprite_pri(int n) { return (int)(((VR(2) << (4 * n)) & 0xf0000000) >> 28); }

static uint8_t s_alphatable[256];
static uint16_t s_bg_zoom[256];

// a sprite ready to draw (draw_sprites + drawgfxzoom's set-up)
typedef struct {
	int sx, sy, w, h;           // on the screen
	int zoomed, zoomx, zoomy;   // (6.10 slopes)
	int flipx, flipy, wide, high, dpth;
	uint32_t code;
	uint32_t pbase;
	int alpha;                  // 0xff solid, 0-0xfe blended, -1 per pen
	uint16_t z;
} prep_t;
static prep_t (*s_prep_hp)[SPR_MAX];
#define s_prep (*s_prep_hp)
static int s_prep_n[8], s_prep_at[8][SPR_MAX];

static void prepare_sprites(void)
{
	memset(s_prep_n, 0, sizeof(s_prep_n));
	int np = 0;
	for (int i = 0; i < s_snap->list_n; i++) {
		const spr_t *sp = &s_snap->list[i];
		const int pri = sprite_pri(sp->bg_pri);
		if (pri > 7)
			continue;
		const uint32_t zoomy = s_snap->zoom[sp->zoomy], zoomx = s_snap->zoom[sp->zoomx];
		if (!zoomy || !zoomx)
			continue;
		const int a = sp->alpha;
		const uint8_t at = (uint8_t)(VR(a >> 2) >> (24 - 8 * (a & 3)));
		prep_t *p = &s_prep[np];
		p->alpha = (at & 0x80) ? -1 : pal6bit(0x3f - (at & 0x3f));
		p->flipx = sp->flpx;
		p->flipy = sp->flpy;
		p->wide = sp->wide;
		p->high = sp->high;
		p->dpth = sp->dpth;
		p->code = sp->tnum;
		p->pbase = (uint32_t)sp->colr * 16;
		p->z = (uint16_t)i;
		p->sx = sp->xpos;
		p->sy = sp->ypos;
		p->zoomx = (int)zoomx;
		p->zoomy = (int)zoomy;
		if (zoomx == 0x400 && zoomy == 0x400) {
			p->zoomed = 0;
			p->w = sp->wide * 16;
			p->h = sp->high * 16;
		} else {
			p->zoomed = 1;
			p->h = (int)(((uint32_t)sp->high * 16u * (0x400u * 0x400u) / zoomy + 0x200) >> 10);
			p->w = (int)(((uint32_t)sp->wide * 16u * (0x400u * 0x400u) / zoomx + 0x200) >> 10);
			if (!p->w || !p->h)
				continue;
		}
		if (p->sy >= s_vis_h || p->sy + p->h <= 0 || p->sx >= VIS_W || p->sx + p->w <= 0)
			continue;
		s_prep_at[pri][s_prep_n[pri]++] = np++;
	}
}

// the row r of the tile column tx of a sprite's whole image (tiles across, then down): its 16 pixels (8 bytes at
// 4 bpp), NULL outside the image (MAME: what an earlier sprite left in its zoom bitmap)
static inline const uint8_t *spr_row(const prep_t *p, int tx, int r)
{
	if (tx >= p->wide || r >= p->high * 16)
		return NULL;
	const uint32_t t = p->code + (uint32_t)((r >> 4) * p->wide + tx);
	if (p->dpth)
		return s_gfx + rbx_wrap(t, s_n8) * 256 + (uint32_t)(r & 15) * 16;
	return s_gfx + rbx_wrap(t, s_n4) * 128 + (uint32_t)(r & 15) * 8;
}

// a tile row with no pixel set (8 bytes at 4 bpp, 16 at 8 bpp; the rows are 8-byte aligned)
static inline int row_empty(const uint8_t *row, int d8)
{
	const uint64_t *q = (const uint64_t *)(const void *)row;
	return d8 ? !(q[0] | q[1]) : !q[0];
}

static inline uint8_t row_pix(const uint8_t *row, int d8, int fx)
{
	if (!row)
		return 0;
	if (d8)
		return row[fx];
	const uint8_t v = row[fx >> 1];
	return (fx & 1) ? (v & 15) : (v >> 4);
}

static inline void spr_put(const prep_t *p, uint32_t *d, uint16_t *zb, int x, uint8_t src)
{
	if (p->z > 0 && p->z < zb[x])
		return;
	if (!src)
		return;
	const uint32_t c = s_snap->pens[p->pbase + src];
	if (p->alpha == 0xff)
		d[x] = c;
	else if (p->alpha >= 0)
		d[x] = blend(d[x], c, (uint32_t)p->alpha);
	else if (s_alphatable[src] == 0xff)
		d[x] = c;
	else
		d[x] = blend(d[x], c, s_alphatable[src]);
	if (p->z > 0)
		zb[x] = p->z;
}

static void draw_sprites_line(int y, int pri, uint32_t *d, uint16_t *zb)
{
	for (int k = 0; k < s_prep_n[pri]; k++) {
		const prep_t *p = &s_prep[s_prep_at[pri][k]];
		if (y < p->sy || y >= p->sy + p->h)
			continue;
		const int x0 = p->sx < 0 ? 0 : p->sx, x1 = p->sx + p->w > VIS_W ? VIS_W : p->sx + p->w;
		int r, c, dc;   // (the source row; the source column of x0 in 6.10 and its step)
		if (!p->zoomed) {
			r = p->flipy ? p->h - 1 - (y - p->sy) : y - p->sy;
			c = (p->flipx ? p->w - 1 - (x0 - p->sx) : x0 - p->sx) << 10;
			dc = p->flipx ? -0x400 : 0x400;
		} else {
			const int dy = p->flipy ? -p->zoomy : p->zoomy;
			r = ((p->flipy ? (p->h - 1) * p->zoomy : 0) + (y - p->sy) * dy) >> 10;
			dc = p->flipx ? -p->zoomx : p->zoomx;
			c = (p->flipx ? (p->w - 1) * p->zoomx : 0) + (x0 - p->sx) * dc;
		}
		if (!p->zoomed && !p->flipx) {   // (whole tile columns: a span of up to 16 pixels each)
			for (int x = x0; x < x1;) {
				const int sc = c >> 10, fx = sc & 15;
				int n = 16 - fx;
				if (n > x1 - x)
					n = x1 - x;
				const uint8_t *row = spr_row(p, sc >> 4, r);
				if (row && !row_empty(row, p->dpth)) {
					if (p->dpth) {
						for (int i = 0; i < n; i++)
							spr_put(p, d, zb, x + i, row[fx + i]);
					} else {
						for (int i = 0; i < n; i++) {
							const uint8_t v = row[(fx + i) >> 1];
							spr_put(p, d, zb, x + i, ((fx + i) & 1) ? (v & 15) : (v >> 4));
						}
					}
				}
				x += n;
				c += n << 10;
			}
			continue;
		}
		int ltx = -1;
		const uint8_t *row = NULL;
		for (int x = x0; x < x1; x++, c += dc) {
			const int sc = c >> 10;
			if ((sc >> 4) != ltx) {
				ltx = sc >> 4;
				row = spr_row(p, ltx, r);
			}
			spr_put(p, d, zb, x, row_pix(row, p->dpth, sc & 15));
		}
	}
}

static void draw_layer_line(int layer, int y, int req_pri, uint32_t *d)
{
	const int linebank = bg_type(layer);
	if (linebank > 0x1f)
		return;
	const int height = bg_large(layer) ? 512 : 256;
	const uint32_t *scroll_reg = &s_snap->spr[(uint32_t)linebank * 0x200];
	const uint32_t *pzab_reg = scroll_reg + 0x100;
	const int regaddr = (bg_line(layer) ? y : 0xfc + layer) & 0xff;
	const uint32_t pzab = pzab_reg[regaddr];
	if ((int)(pzab >> 24) != req_pri)
		return;
	const int tilebank = (int)(pzab & 0xff);
	if (tilebank < 0x0a || tilebank > 0x1f)
		return;
	const int scrollx = (int)(scroll_reg[regaddr] & 0x1ff), scrolly = (int)((scroll_reg[regaddr] >> 16) & 0x1ff);
	const int zoom = (int)((pzab >> 16) & 0xff);
	const int alphamap = (int)((pzab >> 15) & 1);
	int alpha = (int)((pzab >> 8) & 0x3f);
	if (!alphamap)
		alpha = pal6bit(0x3f - alpha);
	else
		alpha = -1;
	if (alpha == 0)
		return;   // (nothing drawn: MAME's "alpha > 0")
	const int srcy = (y - scrolly) & (height - 1);
	const int bank = (tilebank + (srcy >> 8)) & 0x1f;
	const int d8 = bg_8bpp(layer);
	const int zstep = s_bg_zoom[zoom];
	const uint32_t *map = &s_snap->spr[((uint32_t)bank << 9) + (uint32_t)((srcy & 0xff) >> 4) * 32];
	const uint32_t fy = (uint32_t)(srcy & 15);
	if (zstep == 0x400) {   // (unzoomed: whole tile columns, the pixel mode chosen once a span)
		for (int x = 0, sx = (-scrollx) & 0x1ff; x < VIS_W;) {
			const int fx = sx & 15;
			int n = 16 - fx;
			if (n > VIS_W - x)
				n = VIS_W - x;
			const uint32_t data = map[sx >> 4];
			const uint32_t tileno = data & 0x7ffff, cbase = (data >> 24) * 16;
			const uint8_t *row = d8 ? s_gfx + rbx_wrap(tileno, s_n8) * 256 + fy * 16 : s_gfx + rbx_wrap(tileno, s_n4) * 128 + fy * 8;
			if (row_empty(row, d8)) {
				x += n;
				sx = (sx + n) & 0x1ff;
				continue;
			}
			uint8_t px[16];
			if (d8)
				memcpy(px, row + fx, (size_t)n);
			else
				for (int i = 0; i < n; i++) {
					const uint8_t v = row[(fx + i) >> 1];
					px[i] = ((fx + i) & 1) ? (v & 15) : (v >> 4);
				}
			uint32_t *dd = d + x;
			if (alpha == 0xff) {
				for (int i = 0; i < n; i++)
					if (px[i])
						dd[i] = s_snap->pens[(px[i] + cbase) & 0xfff];
			} else if (alpha > 0) {
				for (int i = 0; i < n; i++)
					if (px[i])
						dd[i] = blend(dd[i], s_snap->pens[(px[i] + cbase) & 0xfff], (uint32_t)alpha);
			} else {
				for (int i = 0; i < n; i++) {
					const uint32_t pix = px[i];
					if (!pix)
						continue;
					const uint32_t color = s_snap->pens[(pix + cbase) & 0xfff];
					if (s_alphatable[pix] == 0xff)
						dd[i] = color;
					else if (s_alphatable[pix] > 0)
						dd[i] = blend(dd[i], color, s_alphatable[pix]);
				}
			}
			x += n;
			sx = (sx + n) & 0x1ff;
		}
		return;
	}
	int lcol = -1;
	const uint8_t *row = NULL;
	uint32_t cbase = 0;
	for (int x = 0, xcnt = 0; x < VIS_W; x++, xcnt += zstep) {
		const int srcx = ((xcnt >> 10) - scrollx) & 0x1ff;
		if ((srcx >> 4) != lcol) {
			lcol = srcx >> 4;
			const uint32_t data = map[lcol];
			const uint32_t tileno = data & 0x7ffff;
			cbase = (data >> 24) * 16;
			row = d8 ? s_gfx + rbx_wrap(tileno, s_n8) * 256 + fy * 16 : s_gfx + rbx_wrap(tileno, s_n4) * 128 + fy * 8;
		}
		const uint32_t pix = row_pix(row, d8, srcx & 15);
		if (!pix)
			continue;
		const uint32_t color = s_snap->pens[(pix + cbase) & 0xfff];
		if (alpha < 0) {
			if (s_alphatable[pix] == 0xff)
				d[x] = color;
			else if (s_alphatable[pix] > 0)
				d[x] = blend(d[x], color, s_alphatable[pix]);
		} else if (alpha == 0xff)
			d[x] = color;
		else
			d[x] = blend(d[x], color, (uint32_t)alpha);
	}
}

static void draw_line(int y, uint32_t *d)
{
	uint16_t zb[VIS_W];
	memset(zb, 0, sizeof(zb));
	const int fbank = (int)(VR(7) >> 24);
	if (fbank > 0x1f) {   // prelineblend
		for (int x = 0; x < VIS_W; x++)
			d[x] = 0;
	} else {
		const uint32_t c = s_snap->spr[(uint32_t)fbank * 0x200 + (uint32_t)y] >> 8;
		for (int x = 0; x < VIS_W; x++)
			d[x] = c;
	}
	for (int pri = 0; pri <= 7; pri++) {
		draw_sprites_line(y, pri, d, zb);
		for (int i = 0; i <= 3; i++)
			if (bg_enable(i))
				draw_layer_line(i, y, pri, d);
		if ((int)(VR(2) & 0xf) == pri && fbank <= 0x1f) {   // postlineblend
			const uint32_t color = s_snap->spr[(uint32_t)fbank * 0x200 + 0x100 + (uint32_t)y];
			if (color & 0x80) {
				for (int x = 0; x < VIS_W; x++)
					d[x] = color >> 8;
			} else if (color & 0x7f) {
				for (int x = 0; x < VIS_W; x++)
					d[x] = blend(d[x], color >> 8, (color & 0x7f) << 1);
			}
		}
	}
}

#define BANDS 8
static void psh_band(int band)
{
	const int r0 = s_vis_h * band / BANDS, r1 = s_vis_h * (band + 1) / BANDS;
	for (int y = r0; y < r1; y++) {
		uint32_t d[VIS_W];
		draw_line(y, d);
		uint8_t *o = (uint8_t *)rbx_video.out + (size_t)y * rbx_video.out_pitch;
		if (s_565)
			for (int x = 0; x < VIS_W; x++)
				((uint16_t *)o)[x] = (uint16_t)((d[x] >> 8 & 0xf800) | (d[x] >> 5 & 0x07e0) | (d[x] >> 3 & 0x001f));
		else if (rbx_video.out_bpp == 4)
			for (int x = 0; x < VIS_W; x++)
				((uint32_t *)o)[x] = rbx_video.out_color((int)(d[x] >> 16) & 0xff, (int)(d[x] >> 8) & 0xff, (int)d[x] & 0xff);
		else
			for (int x = 0; x < VIS_W; x++)
				((uint16_t *)o)[x] = (uint16_t)rbx_video.out_color((int)(d[x] >> 16) & 0xff, (int)(d[x] >> 8) & 0xff, (int)d[x] & 0xff);
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, psh_band };
static int psh_help(int helper) { return decoband_help(&s_bands, helper); }

static void psh_render(void)
{
	if (!rbx_video.out)
		return;
	s_565 = rbx_video.out_bpp == 2 && rbx_video.out_color(0xff, 0, 0) == 0xf800 && rbx_video.out_color(0, 0xff, 0) == 0x07e0 &&
	        rbx_video.out_color(0, 0, 0xff) == 0x001f && rbx_video.out_color(0x84, 0x82, 0x21) == 0x8404;
	prepare_sprites();
	decoband_run(&s_bands);
	rbx_video.out_done = 1;
}

// get_sprites (screen_vblank): the list from bank 7
static void get_sprites(void)
{
	int n = 0;
	for (int c = 0; c < 0x800 / 2; c++) {
		const uint32_t w = b.spr[0x3800 / 4 + c / 2];
		const uint16_t listdat = (uint16_t)((c & 1) ? w : w >> 16);
		const uint32_t sprnum = (uint32_t)(listdat & 0x3ff) << 2;
		const uint32_t s0 = b.spr[sprnum], s1 = b.spr[sprnum + 1], s2 = b.spr[sprnum + 2];
		spr_t *sp = &b.list[n++];
		const int ypos = (int)((s0 >> 16) & 0x3ff), xpos = (int)(s0 & 0x3ff);
		sp->ypos = (int16_t)((ypos ^ 0x200) - 0x200);
		sp->xpos = (int16_t)((xpos ^ 0x200) - 0x200);
		sp->high = (uint8_t)(((s1 >> 24) & 0x0f) + 1);
		sp->wide = (uint8_t)(((s1 >> 8) & 0x0f) + 1);
		sp->flpy = (uint8_t)(s1 >> 31);
		sp->spr_pri = (uint8_t)((s1 >> 28) & 3);
		sp->flpx = (uint8_t)((s1 >> 15) & 1);
		sp->bg_pri = (uint8_t)((s1 >> 12) & 3);
		sp->zoomy = (uint8_t)(s1 >> 16);
		sp->zoomx = (uint8_t)s1;
		sp->tnum = s2 & 0x7ffff;
		sp->alpha = (uint8_t)((s2 >> 20) & 7);
		sp->dpth = (uint8_t)((s2 >> 23) & 1);
		sp->colr = (uint16_t)(s2 >> 24);
		if (listdat & 0x4000)
			break;
	}
	b.list_n = n;
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spr, b.spr, sizeof(b.spr));
	for (int i = 0; i < 0x1400; i++)
		s_snap->pens[i] = (b.pal[i] >> 8) & 0xffffff;
	for (int i = 0; i < 0x100; i++)
		s_snap->zoom[i] = (uint16_t)((i & 1) ? b.zoom[i / 2] : b.zoom[i / 2] >> 16);
	memcpy(s_snap->vidregs, b.vidregs, sizeof(b.vidregs));
	memcpy(s_snap->list, b.list, sizeof(b.list[0]) * (size_t)b.list_n);
	s_snap->list_n = b.list_n;
	rbx_video_frame_done();
	get_sprites();
}


// ---- the idle loops: cpu/sh2/sh2idle.c (the vblank waits counting rounds) ----
static sh2_idle_t s_idle;
static int s_noidle;

// ---- frame ----
static void psh_frame(void)
{
	rbx_audio.frame_mhz = s_frame_mhz;
	for (int line = 0; line < s_lines; line++) {
		if (line == s_vis_h) {
			vblank_capture();
			sh2_set_irq_line(&s_cpu, 4, SH2_ASSERT);   // (the vblank interrupt)
		}
		const int cyc = rbx_line_cycles(&b.cpu_frac, CPU_CLOCK, s_frame_mhz, s_lines);
		sh2_execute(&s_cpu, cyc);
		if (!(line & 15))
			sh2_idle_learn(&s_idle, &s_cpu);
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&b.ym_frac, CPU_CLOCK, s_frame_mhz, s_lines);
		s_mix_n += rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static void psh_reset(void)
{
	rbx_video_wait_idle();
	memset(&b, 0, sizeof(b));
	map_mem();
	sh2_reset(&s_cpu);
	sh2_idle_reset(&s_idle, &s_cpu, s_noidle);
	rbx_ym_reset(0);
	s_mix_n = 0;
}

static int psh_supported(const char *machine)
{
	return !strncmp(machine, "psikyo3v1:", 10) || !strncmp(machine, "psikyo5:", 8) || !strncmp(machine, "psikyo5_240:", 12) ||
	       !strncmp(machine, "psikyo5_mahjong:", 16);
}

static void psh_exit(void)
{
	rbx_video_wait_idle();
	eepser_nv_save(&s_eeprom);
	rbx_ym_exit(0);
	free(s_snap);
	s_snap = NULL;
	free(s_mix);
	s_mix = NULL;
	free(s_bp);   // (last: the render thread and the saves are done with it)
	s_bp = NULL;
	free(s_prep_hp);
	s_prep_hp = NULL;
}

static int psh_init(const rbx_game *g)
{
	if (!s_prep_hp && !(s_prep_hp = calloc(1, sizeof(*s_prep_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_bp && !(s_bp = (struct psh_state *)calloc(1, sizeof(*s_bp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!psh_supported(g->machine)) {
		rbx_fail("%s: machine %s not emulated", g->set, g->machine);
		return -1;
	}
	s_board = !strncmp(g->machine, "psikyo3v1:", 10) ? B_PS3 : B_PS5;
	const int tall = !strncmp(g->machine, "psikyo5_240:", 12);
	s_vis_h = tall ? 240 : 224;
	s_lines = tall ? 262 : 256;
	s_frame_mhz = tall ? 61681 : 60000;   // (TGM2: 57.272727 MHz / 8 / (443 x 262))
	s_vbase = s_board == B_PS3 ? 0x03000000 : 0x04000000;
	s_iobase = s_board == B_PS3 ? 0x05800000 : 0x03000000;
	s_ymfbase = s_board == B_PS3 ? 0x05000000 : 0x03100000;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_gfx = rbx_region("gfx", &s_gfx_size);
	s_ymf = rbx_region("ymf", &s_ymf_size);
	if (!s_rom || !s_gfx || !s_ymf) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	sh2_swap32(s_rom, s_rom_size & ~3u);   // (MAME's region of a big-endian CPU in host-order words)
	s_n4 = s_gfx_size / 128;
	s_n8 = s_gfx_size / 256;
	s_noidle = getenv("RBX_PSH_NOIDLE") != NULL;
	s_port_in = rbx_port_find("INPUTS");

	s_port_jp4 = rbx_port_find("JP4");
	s_port_ctrl = rbx_port_find("CONTROLLER");
	s_port_mj = rbx_port_find("MAHJONG");
	for (int i = 0; i < 0xc0; i++)
		s_alphatable[i] = 0xff;
	for (int i = 0; i < 0x40; i++)
		s_alphatable[i + 0xc0] = (uint8_t)pal6bit(0x3f - i);
	for (int i = 0; i < 0x100; i++)
		s_bg_zoom[i] = (uint16_t)((64 * 0x400) / (i + 64));
	eepser_init(&s_eeprom, EEPSER_93CXX, 256, 8, CPU_CLOCK);
	s_eeprom.cmd_address_bits = 9;   // (93C56 8-bit: MAME's 9 address bits of a command)
	for (int i = 0; i < 256; i++)
		s_eeprom.data[i] = 0;        // (default_value(0))
	uint32_t esz = 0;
	const uint8_t *e = rbx_region("eeprom", &esz);
	if (e)
		eepser_default(&s_eeprom, e, esz);
	eepser_nv_load(&s_eeprom, g->set);
	sh2_init(&s_cpu);
	s_cpu.read8 = r8;
	s_cpu.read16 = r16;
	s_cpu.read32 = r32;
	s_cpu.write8 = w8;
	s_cpu.write16 = w16;
	s_cpu.write32 = w32;
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YMF278B;
	yc.clock = CPU_CLOCK;
	yc.adpcma = s_ymf;
	yc.adpcma_size = s_ymf_size;
	yc.gain[0] = 256;
	yc.irq = ym_irq;
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		psh_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, s_vis_h, 1)) {
		rbx_fail("%s: out of memory", g->set);
		psh_exit();
		return -1;
	}
	psh_reset();
	return 0;
}

static int psh_state(void *ctx, int load)
{
	if (load)
		rbx_video_wait_idle();
	int rc = 0;
	rc |= rbx_state_io(ctx, "sh2", &s_cpu, SH2_STATE_SIZE);
	rc |= rbx_state_io(ctx, "psikyosh", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= eepser_state(&s_eeprom, ctx, load, "eeprom");
	rc |= rbx_ym_state(0, ctx, load);
	if (!rc && load) {
		sh2_post_load(&s_cpu);
		gfxbank_map();
	}
	return rc ? -1 : 0;
}

static int psh_save(void *ctx) { return psh_state(ctx, 0); }
static int psh_load(void *ctx) { return psh_state(ctx, 1); }

const rbx_driver rbx_drv_psikyosh = { "Psikyo PS3/PS5", psh_init, psh_exit, psh_reset, psh_frame, psh_save, psh_load,
                                      psh_supported, psh_render, psh_help };
