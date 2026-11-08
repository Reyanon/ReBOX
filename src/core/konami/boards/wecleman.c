/*  Konami WEC Le Mans 24 and Hot Chase. Ported from MAME src/mame/konami/wecleman.cpp and wecleman_v.cpp
    (BSD-3-Clause, copyright-holders: Luca Elia) and k007452.cpp (BSD-3-Clause). Two 68000s at 10 MHz (cpu/m68k_multi.c,
    the sub CPU held in reset by the main one), the blitter that copies the sprite lists, the zooming sprites;
    WEC Le Mans: two page tilemaps with line scroll, the text layer, the road (two priorities), the clouds blended in
    RGB (MAME's bitmap_rgb32), a Z80 with a YM2151, a K007232 and the K007452 math chip; Hot Chase: two K051316 layers,
    its road, a 6809 with three K007232s. The sound runs with the main CPU line by line.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k_multi.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/k007232.h"
#include "../../sound/rbx_ym.h"
#include "../video/k051316.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 10000000u
#define LINES 256              // (set_size(336, 256) at 60 Hz)
#define SND_RATE 48000
#define BMP_W 336
#define BMP_H 256
#define BMP_PAD 8
#define VIS_W 320
#define VIS_H 224
#define BLEND_STEPS 16
#define BLEND_MIN 0
#define BLEND_MAX (BLEND_STEPS * 0x20 - 1)
#define BLEND_INC 1
#define BLEND_DEC -8
#define NUM_SPRITES 256

static int s_hot;   // (Hot Chase, else WEC Le Mans)
static uint8_t *s_rom, *s_subrom, *s_zrom, *s_spr, *s_layers, *s_road;
static uint32_t s_rom_size, s_subrom_size, s_zrom_size, s_spr_size, s_layers_size, s_road_size;
static uint8_t *s_tilegfx;   // (WEC: the 8x8 3 bpp tiles decoded, 64 bytes each)
static uint8_t *s_roadgfx;   // (the road's lines: WEC 256 x 512 3 bpp, Hot Chase 8192 x 64 4 bpp elements)
static z80 s_z;
static const uint8_t *s_zmap[256];
static m6809_t s_m09;
static k007232_t s_k232[3];
static k051316_t s_k316[2];
static int s_port_in0, s_port_in1, s_port_dsw_lo, s_port_dsw_hi, s_port_accel, s_port_steer;

static struct {
	uint16_t ram[0x4000], ram2[0x2000], share[0x2000], pageram[0x2000], txtram[0x800], spriteram[0x800], roadram[0x800];
	uint16_t subram[0x810];
	uint16_t palram[0x1000];
	uint32_t pens[0x2000];   // (the palette's pens as MAME keeps them, its hacks included; 0x2000: Hot Chase's 8192)
	uint16_t blitter[9], prot[3];
	int32_t prot_state, irqctrl, selected_ip, cloud_blend, cloud_ds, cloud_visible, sound_hs;
	int16_t bg_rowscroll[512], fg_rowscroll[512];
	uint8_t sub_run, sub_reset, z_run, zr_main;
	uint8_t k7452_regs[6];
	uint16_t k7452_mul, k7452_quo, k7452_rem;
	uint8_t zram[0x800], soundlatch, z80_irq;
	uint64_t cpu_frac, z_frac, z_end, ym_frac, k232_frac, snd_frac, firq_frac;
	int64_t m09_end;
	int32_t k232_out[3][2];
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct {
	uint16_t pageram[0x2000], txtram[0x800], spriteram[0x800], roadram[0x800];
	uint32_t pens[0x2000];
	int16_t bg_rowscroll[512], fg_rowscroll[512];
	int video_on, cloud_draw, cloud_alpha;
	k051316_t k316[2];
} s_vs;
static uint32_t s_frame32[BMP_H][BMP_W];   // (WEC: the rgb32 bitmap with its pads)
static uint16_t s_frame16[VIS_H][VIS_W];   // (Hot Chase: the ind16 bitmap)
static uint8_t s_dummy_pri[VIS_H * VIS_W];
#define BLACK_PEN 0x2000
static int32_t s_snd[2][2048];
static int s_snd_n;

static inline uint8_t pal4(int v) { return (uint8_t)((v & 15) * 17); }
static inline uint8_t pal5(int v) { v &= 31; return (uint8_t)(v << 3 | v >> 2); }
static inline uint32_t rgb(int r, int g, int b) { return (uint32_t)(r << 16 | g << 8 | b); }

// ---- the palette ----
static void palette_w(int o)
{
	const uint16_t w = t.palram[o];
	if (!s_hot) {   // (SSSSBBBBGGGGRRRR)
		t.pens[o] = rgb(pal4(w), pal4(w >> 4), pal4(w >> 8));
		return;
	}
	// (SBGRBBBBGGGGRRRR, its half bright at + 0x800)
	int r = ((w << 1) & 0x1e) | ((w >> 12) & 1), g = ((w >> 3) & 0x1e) | ((w >> 13) & 1), b = ((w >> 7) & 0x1e) | ((w >> 14) & 1);
	t.pens[o] = rgb(pal5(r), pal5(g), pal5(b));
	r >>= 1;
	g >>= 1;
	b >>= 1;
	t.pens[o + 0x800] = rgb(pal5(r) / 2, pal5(g) / 2, pal5(b) / 2);
}

// ---- the capture (screen_update's state changes are made here, on the emulation's side) ----
static void capture(void)
{
	s_vs.video_on = (t.irqctrl & 0x40) != 0;
	if (!s_hot) {
		const int fg_y = t.txtram[0x0f24 >> 1] & 511, bg_y = t.txtram[0x0f26 >> 1] & 511;
		for (int i = 0; i < (28 << 2); i += 4) {
			const int fg_x = (int16_t)t.txtram[(i + 0xf80) >> 1] + (0xb0 - BMP_PAD);
			const int bg_x = (int16_t)t.txtram[(i + 0xf82) >> 1] + (0xb0 - BMP_PAD);
			const int k = i << 1;
			for (int j = 0; j < 8; j++) {
				t.fg_rowscroll[(fg_y + k + j) & 511] = (int16_t)fg_x;
				t.bg_rowscroll[(bg_y + k + j) & 511] = (int16_t)bg_x;
			}
		}
		t.pens[0x27] = t.pens[0x24];   // (palette hacks)
		s_vs.cloud_draw = 0;
		if (t.cloud_visible) {
			t.pens[0] = t.pens[0x40] = t.pens[0x200] = t.pens[0x205];
			s_vs.cloud_draw = s_vs.video_on;
			s_vs.cloud_alpha = t.cloud_blend / BLEND_STEPS;
			t.cloud_blend += t.cloud_ds;
			if (t.cloud_blend < BLEND_MIN) {
				t.cloud_blend = BLEND_MIN;
				t.cloud_ds = 0;
				t.ram[0x494 >> 1] |= 1;   // (*m_videostatus |= 1)
			} else if (t.cloud_blend > BLEND_MAX) {
				t.cloud_blend = BLEND_MAX;
				t.cloud_ds = 0;
				t.cloud_visible = 0;
			}
		}
		memcpy(s_vs.pageram, t.pageram, sizeof(s_vs.pageram));
		memcpy(s_vs.txtram, t.txtram, sizeof(s_vs.txtram));
		memcpy(s_vs.bg_rowscroll, t.bg_rowscroll, sizeof(s_vs.bg_rowscroll));
		memcpy(s_vs.fg_rowscroll, t.fg_rowscroll, sizeof(s_vs.fg_rowscroll));
	} else
		for (int i = 0; i < 2; i++)
			memcpy(&s_vs.k316[i], &s_k316[i], offsetof(k051316_t, state_end));
	memcpy(s_vs.spriteram, t.spriteram, sizeof(s_vs.spriteram));
	memcpy(s_vs.roadram, t.roadram, sizeof(s_vs.roadram));
	memcpy(s_vs.pens, t.pens, sizeof(s_vs.pens));
}

// ---- the sprites (get_sprite_info, sortsprite, do_blit_zoom32) ----
typedef struct {
	const uint8_t *pen_data;
	int line_offset, pal_base, x, y, total_width, total_height, tile_width, tile_height, flags, shadow;
} sprite_t;
static sprite_t s_sprites[NUM_SPRITES];
static int s_spr_idx[NUM_SPRITES], s_spr_pri[NUM_SPRITES], s_spr_count;

static int get_sprite_info(void)
{
	static const int wec_bank[0x40] = { 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12,
	                                    13, 13, 14, 14, 15, 15, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9,
	                                    10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15 };
	const int offsx = s_hot ? -0xc0 : -0xbc + BMP_PAD, offsy = s_hot ? 0 : 1 + BMP_PAD;
	int count = 0;
	for (int n = 0; n < NUM_SPRITES; n++) {
		const uint16_t *source = s_vs.spriteram + n * 8;
		sprite_t *sp = &s_sprites[count];
		if (source[0] == 0xffff)
			break;
		sp->y = source[0] & 0xff;
		sp->total_height = (source[0] >> 8) - sp->y;
		if (sp->total_height < 1)
			continue;
		sp->x = source[1] & 0x1ff;
		const int bank = source[1] >> 10;
		if (bank == 0x3f)
			continue;
		sp->tile_width = source[2] & 0xff;
		if (sp->tile_width < 1)
			continue;
		sp->shadow = source[2] & 0x4000;
		const int code = source[3], zoom = source[4];
		sp->pal_base = (source[7] & 0xff) << 4;
		int gfx = ((s_hot ? (bank < 48 ? bank : bank - 48) : wec_bank[bank]) << 15) + (code & 0x7fff);
		sp->flags = 0;
		if (code & 0x8000) {
			sp->flags |= 1;
			gfx += 1 - sp->tile_width;
		}
		if (source[1] & 0x0200)
			sp->flags |= 2;
		gfx <<= 3;
		sp->tile_width <<= 3;
		sp->tile_height = (sp->total_height * 0x80) / (0x80 - (zoom >> 8));
		if (gfx < 0 || (uint32_t)(gfx + sp->tile_width * sp->tile_height - 1) >= s_spr_size)
			continue;
		sp->pen_data = s_spr + gfx;
		sp->line_offset = sp->tile_width;
		sp->total_width = sp->tile_width - (sp->tile_width * (zoom & 0xff)) / 0x80;
		sp->total_height += 1;
		sp->x += offsx;
		sp->y += offsy;
		s_spr_idx[count] = count;
		s_spr_pri[count] = source[7] >> 8;
		count++;
	}
	return count;
}

static void sortsprite(int *idx_array, const int *key_array, int size)
{
	idx_array += size;
	for (int j = -size; j < -1; j++) {
		const int src_idx = idx_array[j];
		int low_pos = j, low_val = key_array[src_idx], hi_idx = src_idx;
		for (int i = j + 1; i; i++) {
			const int tgt_idx = idx_array[i], tgt_val = key_array[tgt_idx];
			if (low_val > tgt_val) {
				low_val = tgt_val;
				low_pos = i;
			} else if (low_val == tgt_val && hi_idx <= tgt_idx) {
				hi_idx = tgt_idx;
				low_pos = i;
			}
		}
		low_val = idx_array[low_pos];
		idx_array[low_pos] = src_idx;
		idx_array[j] = low_val;
	}
}

// (do_blit_zoom32) into the WEC rgb32 bitmap (rgb) or the Hot Chase ind16 one; the clip in bitmap coordinates
static void blit_zoom(const sprite_t *sp, int cminx, int cmaxx, int cminy, int cmaxy, int rgbmode)
{
	int x1, x2, y1, y2, dx, dy, xcount0 = 0, ycount0 = 0;
	if (sp->flags & 1) {
		x2 = sp->x;
		x1 = x2 + sp->total_width;
		dx = -1;
		if (x2 < cminx) x2 = cminx;
		if (x1 > cmaxx) {
			xcount0 = x1 - cmaxx;
			x1 = cmaxx;
		}
		if (x2 >= x1) return;
		x1--;
		x2--;
	} else {
		x1 = sp->x;
		x2 = x1 + sp->total_width;
		dx = 1;
		if (x1 < cminx) {
			xcount0 = cminx - x1;
			x1 = cminx;
		}
		if (x2 > cmaxx) x2 = cmaxx;
		if (x1 >= x2) return;
	}
	if (sp->flags & 2) {
		y2 = sp->y;
		y1 = y2 + sp->total_height;
		dy = -1;
		if (y2 < cminy) y2 = cminy;
		if (y1 > cmaxy) {
			ycount0 = cmaxy;
			y1 = cmaxy;
		}
		if (y2 >= y1) return;
		y1--;
		y2--;
	} else {
		y1 = sp->y;
		y2 = y1 + sp->total_height;
		dy = 1;
		if (y1 < cminy) {
			ycount0 = cminy - y1;
			y1 = cminy;
		}
		if (y2 > cmaxy) y2 = cmaxy;
		if (y1 >= y2) return;
	}
	const int src_fdy = (sp->tile_height << 20) / sp->total_height;
	int src_f0y = src_fdy * ycount0 + (1 << 19);
	const int src_fdx = (sp->tile_width << 20) / sp->total_width;
	const int src_f0x = src_fdx * xcount0;
	if (x1 > cminx) {
		x1 -= dx;
		x2 -= dx;
	}
	const uint32_t *pal = s_vs.pens + sp->pal_base;
	for (int sy = y1; sy != y2; sy += dy) {
		const uint8_t *row = sp->pen_data + (src_f0y >> 20) * sp->line_offset;
		int src_fpx = src_f0x;
		if (rgbmode) {
			uint32_t *dst = s_frame32[sy];
			for (int sx = x1; sx != x2; sx += dx) {
				const int pix = row[src_fpx >> 20];
				if (pix & 0x80)
					break;
				if (pix && (unsigned)sx < BMP_W) {
					if (!sp->shadow || pix != 0xa)
						dst[sx] = pal[pix];
					else
						dst[sx] = (dst[sx] >> 1) & 0x7f7f7f;
				}
				src_fpx += src_fdx;
			}
		} else {
			uint16_t *dst = s_frame16[sy];
			for (int sx = x1; sx != x2; sx += dx) {
				const int pix = row[src_fpx >> 20];
				if (pix & 0x80)
					break;
				if (pix) {
					if (!sp->shadow || pix != 0xa)
						dst[sx] = (uint16_t)(sp->pal_base + pix);
					else if (dst[sx] != BLACK_PEN)
						dst[sx] |= 0x800;
				}
				src_fpx += src_fdx;
			}
		}
		src_f0y += src_fdy;
	}
}

// ---- WEC Le Mans: the layers ----
static inline int tile_pix(int code, int x, int y) { return s_tilegfx[(code & 0xfff) * 64 + y * 8 + x]; }

static inline uint64_t row8(const uint8_t *p)   // (a tile row's 8 pixels: 0 = all transparent)
{
	uint64_t v;
	memcpy(&v, p, 8);
	return v;
}

// a row of 8x8 tiles from tx (the tilemap's x) over the visible area: code_at gives a tile column's code
#define TILE_RUN(code_expr)                                                                                          \
	do {                                                                                                         \
		int x = BMP_PAD;                                                                                         \
		while (x < BMP_PAD + VIS_W) {                                                                            \
			const int px0 = tx & 7;                                                                              \
			int n = 8 - px0;                                                                                     \
			if (n > BMP_PAD + VIS_W - x)                                                                         \
				n = BMP_PAD + VIS_W - x;                                                                         \
			const int code = (code_expr);                                                                        \
			const uint8_t *src = s_tilegfx + (code & 0xfff) * 64 + (ty & 7) * 8;                                 \
			if (row8(src)) {                                                                                     \
				const uint32_t *pal = s_vs.pens + (((code >> 5) & 0x78) + (code >> 12)) * 8;                     \
				for (int k = 0; k < n; k++) {                                                                    \
					const int p = src[px0 + k];                                                                  \
					if (p)                                                                                       \
						d[x + k] = pal[p];                                                                       \
				}                                                                                                \
			}                                                                                                    \
			x += n;                                                                                              \
			tx += n;                                                                                             \
		}                                                                                                        \
	} while (0)

// a page tilemap (128 x 64 tiles): the page selector at txtram 0xefe (bg) / 0xefc (fg)
static void draw_pages(int fg)
{
	const uint16_t sel = s_vs.txtram[(fg ? 0xefc : 0xefe) >> 1];
	const int page[4] = { (sel >> 4) & 3, sel & 3, (sel >> 12) & 3, (sel >> 8) & 3 };
	const int sc_y = (s_vs.txtram[(fg ? 0x0f24 : 0x0f26) >> 1] & 511) - BMP_PAD;
	const int16_t *rows = fg ? s_vs.fg_rowscroll : s_vs.bg_rowscroll;
	for (int y = BMP_PAD; y < BMP_PAD + VIS_H; y++) {
		const int ty = (y + sc_y) & 511, trow = ty >> 3;
		int tx = (BMP_PAD + rows[ty]) & 1023;
		uint32_t *d = s_frame32[y];
		const uint16_t *pr[2] = { s_vs.pageram + ((trow & 0x1f) << 6) + (page[(trow >> 5) << 1] << 11),
		                          s_vs.pageram + ((trow & 0x1f) << 6) + (page[1 + ((trow >> 5) << 1)] << 11) };
#define PAGE_CODE(t) ({ const int tc_ = ((t) >> 3) & 127; int c_ = pr[tc_ >> 6][tc_ & 0x3f];                       \
		if (fg && (!c_ || c_ == 0xffff)) c_ = 0x20; c_; })
		TILE_RUN(PAGE_CODE(tx));
#undef PAGE_CODE
	}
}

static void draw_text(void)
{
	for (int y = BMP_PAD; y < BMP_PAD + VIS_H; y++) {
		const int ty = (y - BMP_PAD) & 255;   // (scrolly -BMP_PAD)
		int tx = BMP_PAD + 512 - 320 - 16 - BMP_PAD;
		uint32_t *d = s_frame32[y];
		const uint16_t *row = s_vs.txtram + (ty >> 3) * 64;
		TILE_RUN(row[(tx >> 3) & 63]);
	}
}

static void draw_road_wec(int priority)
{
	static const uint16_t road_color[48] = {
		0x3f1, 0x3f3, 0x3f5, 0x3fd, 0x3fd, 0x3fb, 0x3fd, 0x7ff, 0x3f0, 0x3f2, 0x3f4, 0x3fc, 0x3fc, 0x3fb, 0x3fc, 0x7fe,
		0, 0, 0, 0x3f9, 0x3f9, 0, 0, 0, 0, 0, 0, 0x3f8, 0x3f8, 0, 0, 0,
		0, 0, 0, 0x3f7, 0, 0, 0, 0, 0, 0, 0, 0x3f6, 0, 0, 0, 0 };
	const uint16_t *roadram = s_vs.roadram;
	if (priority == 2) {
		for (int sy = 0; sy < VIS_H; sy++) {
			if ((roadram[sy] >> 8) != 0x02)
				continue;
			const uint32_t pix = s_vs.pens[(roadram[sy + 512] & 0xf) + 0x7f0];
			uint32_t *d = &s_frame32[sy + BMP_PAD][BMP_PAD];
			for (int sx = 0; sx < VIS_W; sx++)
				d[sx] = pix;
		}
		return;
	}
	uint32_t road_rgb[48];
	for (int i = 0; i < 48; i++)
		road_rgb[i] = road_color[i] ? s_vs.pens[road_color[i]] : 0xffffffffu;
	for (int sy = 0; sy < VIS_H; sy++) {
		uint16_t road = roadram[sy];
		if ((road >> 8) != 0x04)
			continue;
		road &= 255;
		const uint8_t *src = s_roadgfx + road * 512;
		const int mdy = (road * 5) >> 8, tdy = (road * 7) >> 8;   // (MIDCURB_DY, TOPCURB_DY)
		int scrollx = roadram[sy + 256] + (0x18 - 0xe00);
		const uint32_t *pal = road_rgb + ((roadram[sy + 512] << 3) & 8);
		uint32_t *d = &s_frame32[sy + BMP_PAD][BMP_PAD];
		for (int sx = 0; sx < VIS_W; sx++, scrollx++) {
			if (scrollx >= 0 && scrollx < 512) {
				const int pix = src[scrollx];
				d[sx] = pal[pix];
				uint32_t tmp = pal[pix + 16];
				if (tmp != 0xffffffffu && sy + BMP_PAD - mdy >= 0)
					s_frame32[sy + BMP_PAD - mdy][BMP_PAD + sx] = tmp;
				tmp = pal[pix + 32];
				if (tmp != 0xffffffffu && sy + BMP_PAD - tdy >= 0)
					s_frame32[sy + BMP_PAD - tdy][BMP_PAD + sx] = tmp;
			} else
				d[sx] = pal[7];
		}
	}
}

// (draw_cloud: the tilemap at pageram 0x1800, 64 x 32 tiles, alpha 0-1f)
static void draw_cloud(int scrollx, int scrolly, int alpha)
{
	if (alpha > 0x1f)
		return;
	scrollx &= 511;
	scrolly &= 255;
	int tmskipy = scrolly / 8;
	const int tmskipx = scrollx / 8, dx = -(scrollx & 7), dy = -(scrolly & 7);
	const uint16_t *tm = s_vs.pageram + 0x1800;
	for (int i = 0; i < 20; i++, tmskipy++) {
		const uint16_t *tmap = tm + ((tmskipy & 31) << 6);
		for (int j = 0; j < 41; j++) {
			const uint16_t td = tmap[(tmskipx + j) & 63];
			const uint8_t *src = s_tilegfx + (td & 0xfff) * 64;
			const uint32_t *pal = s_vs.pens + (((td >> 5) & 0x78) + (td >> 12)) * 8;
			const int x0 = BMP_PAD + dx + 8 + j * 8, y0 = BMP_PAD + dy + i * 8;
			for (int ty = 0; ty < 8; ty++) {
				const int y = y0 + ty;
				if (y < 0 || y >= BMP_H)
					continue;
				for (int tx = 0; tx < 8; tx++) {
					const int x = x0 + tx;
					if (x < 0 || x >= BMP_W)
						continue;
					const uint32_t s = pal[src[ty * 8 + tx]];
					if (!alpha) {
						s_frame32[y][x] = s;
						continue;
					}
					const uint32_t d = s_frame32[y][x];
					const int sr = (s >> 3) & 0x1f, sg = (s >> 11) & 0x1f, sb = (s >> 19) & 0x1f;
					int dr = (d >> 3) & 0x1f, dg = (d >> 11) & 0x1f, db = (d >> 19) & 0x1f;
					dr = (((dr - sr) * alpha) >> 5) + dr;
					dg = (((dg - sg) * alpha) >> 5) + dg;
					db = (((db - sb) * alpha) >> 5) + db;
					s_frame32[y][x] = rgb(pal5(db), pal5(dg), pal5(dr));
				}
			}
		}
	}
}

static inline uint16_t to565(uint32_t c) { return (uint16_t)((c >> 8 & 0xf800) | (c >> 5 & 0x07e0) | (c >> 3 & 0x001f)); }

static void render_wec(void)
{
	for (int y = 0; y < BMP_H; y++)
		for (int x = 0; x < BMP_W; x++)
			s_frame32[y][x] = 0;   // (the black pen)
	const int bg_y = s_vs.txtram[0x0f26 >> 1] & 511;
	if (s_vs.video_on) {
		draw_road_wec(2);
		draw_pages(0);
	}
	if (s_vs.cloud_draw)
		draw_cloud(s_vs.txtram[0xfee >> 1] + 0xb0, bg_y, s_vs.cloud_alpha);
	if (s_vs.video_on) {
		draw_pages(1);
		draw_road_wec(4);
		s_spr_count = get_sprite_info();
		sortsprite(s_spr_idx, s_spr_pri, s_spr_count);
		for (int i = 0; i < s_spr_count; i++)
			blit_zoom(&s_sprites[s_spr_idx[i]], BMP_PAD, BMP_PAD + VIS_W - 1, BMP_PAD, BMP_PAD + VIS_H - 1, 1);
		draw_text();
	}
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
		const uint32_t *s = &s_frame32[y + BMP_PAD][BMP_PAD];
		for (int x = 0; x < VIS_W; x++)
			d[x] = to565(s[x]);
	}
}

// ---- Hot Chase ----
static void zoom_cb(int chip, int *code, int *color)
{
	if (!chip) {   // (hotchase_zoom_callback_1)
		*code |= (*color & 0x03) << 8;
		*color = (*color & 0xfc) >> 2;
	} else
		*color = ((*color & 0x3f) << 1) | ((*code & 0x80) >> 7);
}

static void draw_road_hot(void)
{
	for (int sy = 0; sy < VIS_H; sy++) {
		const uint32_t code0 = (uint32_t)s_vs.roadram[sy * 2 + 1] + ((uint32_t)s_vs.roadram[sy * 2] << 16);
		const int color = (int)((code0 & 0x00f00000) >> 20);
		const int scrollx = (int)((code0 & 0x0007fc00) >> 10) * 2;
		int code = (int)(code0 & 0x1ff) * (512 / 32);
		uint16_t *d = s_frame16[sy];
		for (int sx = 0; sx < 2 * 512; sx += 64, code++) {
			const int x0 = ((sx - scrollx) & 0x3ff) - (384 - 32);
			if (x0 >= VIS_W || x0 + 63 < 0)
				continue;
			const uint8_t *src = s_roadgfx + (size_t)(code & 8191) * 64;
			const int k0 = x0 < 0 ? -x0 : 0, k1 = x0 + 64 > VIS_W ? VIS_W - x0 : 64;
			const uint16_t base = (uint16_t)(0x700 + color * 16);
			for (int k = k0; k < k1; k++)
				if (src[k])
					d[x0 + k] = (uint16_t)(base + src[k]);
		}
	}
}

static void render_hot(void)
{
	for (int y = 0; y < VIS_H; y++)
		for (int x = 0; x < VIS_W; x++)
			s_frame16[y][x] = BLACK_PEN;
	if (s_vs.video_on) {
		for (int i = 0; i < 2; i++) {
			s_vs.k316[i].gfx = s_k316[i].gfx;
			s_vs.k316[i].ntiles = s_k316[i].ntiles;
			s_vs.k316[i].pix = s_k316[i].pix;
			s_vs.k316[i].key = s_k316[i].key;
			s_vs.k316[i].chip = i;
			s_vs.k316[i].dx = 8;
			s_vs.k316[i].dy = -16;
			s_vs.k316[i].cb = zoom_cb;
			s_vs.k316[i].bpp = s_k316[i].bpp;
		}
		s_vs.k316[0].wrap = 1;   // (set_wrap(1))
		k051316_zoom_draw(&s_vs.k316[0], &s_frame16[0][0], s_dummy_pri, VIS_W, 0, 0, 0, 0, VIS_W - 1, VIS_H - 1, 0, 0);
		draw_road_hot();
		s_spr_count = get_sprite_info();
		for (int i = 0; i < s_spr_count; i++)
			blit_zoom(&s_sprites[i], 0, VIS_W - 1, 0, VIS_H - 1, 0);
		k051316_zoom_draw(&s_vs.k316[1], &s_frame16[0][0], s_dummy_pri, VIS_W, 0, 0, 0, 0, VIS_W - 1, VIS_H - 1, 0, 0);
	}
	static uint16_t pen565[BLACK_PEN + 1];   // (the frame's pens in RGB 565, the black pen last)
	for (int i = 0; i < BLACK_PEN; i++)
		pen565[i] = to565(s_vs.pens[i]);
	pen565[BLACK_PEN] = 0;
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
		const uint16_t *src = s_frame16[y];
		for (int x = 0; x < VIS_W; x++)
			d[x] = pen565[src[x] & 0x3fff];
	}
}

static void we_render(void)
{
	if (s_hot)
		render_hot();
	else
		render_wec();
}

// ---- the main CPU ----
static uint16_t prot_r(void)   // (wecleman_protection_r: a blend of two 4-4-4 colours)
{
	const int data0 = t.prot[0], data1 = t.prot[1], blend = t.prot[2] & 0x3ff;
	int r0 = data0 & 0xf, g0 = data0 & 0xf0, b0 = data0 & 0xf00;
	int r1 = (data1 & 0xf) - r0, g1 = (data1 & 0xf0) - g0, b1 = (data1 & 0xf00) - b0;
	r1 = (r1 * blend) >> 10;
	g1 = (g1 * blend) >> 10;
	b1 = (b1 * blend) >> 10;
	r0 += r1;
	g0 = (g0 + g1) & 0xf0;
	b0 = (b0 + b1) & 0xf00;
	return (uint16_t)(r0 | g0 | b0);
}

static uint8_t selected_ip_r(void)
{
	switch ((t.selected_ip >> 5) & 3) {
	case 0: return (uint8_t)rbx_port_read(s_port_accel);
	case 2: return (uint8_t)rbx_port_read(s_port_steer);
	default: return 0;
	}
}

// ---- WEC Le Mans: the sound board a frame behind on the worker (the main CPU reads nothing back) ----
#define Q_MAX 2048
enum { Q_ADV, Q_LATCH, Q_IRQ, Q_ZRUN };
typedef struct { uint16_t kind, arg; uint32_t v; } q_ev_t;
static q_ev_t s_q[2][Q_MAX];
static int s_qn[2], s_qcur;
static int32_t s_out[2][2048];
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

static void wec_z_run(int zr)   // (the Z80's reset line: out of reset from 0)
{
	if (zr && !t.z_run) {
		const z80 keep = s_z;
		z80_init(&s_z);
		s_z.read_byte = keep.read_byte;
		s_z.write_byte = keep.write_byte;
		s_z.port_in = keep.port_in;
		s_z.port_out = keep.port_out;
		s_z.read_map = keep.read_map;
		s_z.int_data = 0xff;
		t.z_end = 0;
	}
	t.z_run = (uint8_t)zr;
}

static void irqctrl_w(uint16_t data)
{
	if ((t.irqctrl & 1) && !(data & 1))   // (1 -> 0: the sub CPU's IRQ 4)
		m68k_multi_hold_irq(1, 4);
	const uint8_t sub = (data & 2) != 0;
	if (sub && !t.sub_run)
		t.sub_reset = 1;   // (out of reset: from its vectors, between the CPUs' slices: we_frame)
	t.sub_run = sub;
	if ((t.irqctrl & 4) && !(data & 4)) {
		if (!s_hot)
			q_put(Q_IRQ, 0);
		else {
			t.sound_hs = 0;
			m6809_set_irq(&s_m09, 1);   // (HOLD_LINE: dropped when taken, see sound_line)
		}
	}
	const uint8_t zr = (data & 8) != 0;
	if (!s_hot) {
		if (zr != t.zr_main)
			q_put(Q_ZRUN, zr);
	} else {
		if (zr && !t.z_run)
			m6809_reset(&s_m09);
		t.z_run = zr;
	}
	t.zr_main = zr;
	t.irqctrl = data;
}

static void blitter_w(int offset, uint16_t data, uint16_t mask);

static uint16_t main_r16(uint32_t a)
{
	a &= 0xffffff;
	switch (a) {
	case 0x140010: return (uint16_t)rbx_port_read(s_port_in0);
	case 0x140012: return (uint16_t)((rbx_port_read(s_port_in1) & ~0x18u) | (s_hot && t.sound_hs ? 0x10 : 0));
	case 0x140014: return (uint16_t)rbx_port_read(s_port_dsw_lo);
	case 0x140016: return (uint16_t)rbx_port_read(s_port_dsw_hi);
	case 0x140020: return selected_ip_r();
	}
	if (!s_hot && a == 0x060006)
		return prot_r();
	if (s_hot && a >= 0x100000 && a < 0x104000 && !(a & 0x1000))
		return k051316_read(&s_k316[(a >> 13) & 1], (int)((a & 0xfff) >> 1));
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
	if (a >= 0x080000 && a < 0x080012) {
		blitter_w((int)((a - 0x080000) >> 1), v, mask);
		return;
	}
	const uint32_t pal = 0x110000, pal_end = s_hot ? 0x112000 : 0x111000;
	if (a >= pal && a < pal_end) {
		const int o = (int)((a - pal) >> 1);
		t.palram[o] = (uint16_t)((t.palram[o] & ~mask) | (v & mask));
		palette_w(o);
		return;
	}
	if (!s_hot && a >= 0x040000 && a < 0x041000) {   // (the RAM's first page: videostatus at 040494)
		uint16_t *p = &t.ram[(a - 0x040000) >> 1];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
		if (a == 0x040494 && (mask & 0x00ff)) {   // (wecleman_videostatus_w)
			if ((v & 0x7f) == 0 && !t.cloud_ds)
				t.cloud_ds = BLEND_INC;
			else if ((v & 0x7f) == 1 && !t.cloud_visible) {
				t.cloud_ds = BLEND_DEC;
				t.cloud_visible = 1;
			}
		}
		return;
	}
	if (!s_hot && a >= 0x060000 && a < 0x060006) {   // (wecleman_protection_w)
		const int o = (int)((a - 0x060000) >> 1);
		if (o == 2)
			t.prot_state = v & 0x2000;
		if (!t.prot_state)
			t.prot[o] = (uint16_t)((t.prot[o] & ~mask) | (v & mask));
		return;
	}
	if (s_hot && a >= 0x100000 && a < 0x104000) {
		k051316_t *k = &s_k316[(a >> 13) & 1];
		if (mask & 0x00ff) {
			if (a & 0x1000)
				k051316_ctrl_w(k, (int)((a & 0x1f) >> 1), (uint8_t)v);
			else
				k051316_write(k, (int)((a & 0xfff) >> 1), (uint8_t)v);
		}
		return;
	}
	if (a == 0x140000 && (mask & 0x00ff)) {
		if (s_hot)
			t.soundlatch = (uint8_t)v;
		else
			q_put(Q_LATCH, (uint8_t)v);
	}
	else if (a == 0x140002 && (mask & 0x00ff))
		t.selected_ip = v & 0xff;
	else if (a == 0x140004 && (mask & 0x00ff))
		irqctrl_w(v & 0xff);
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v) { main_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00); }

// (blitter_w: mspace word accesses on the main CPU's bus)
static uint16_t mspace_r(uint32_t a) { return (uint16_t)m68k_read_memory_16(a & 0xffffff); }
static void mspace_w(uint32_t a, uint16_t v) { m68k_write_memory_16(a & 0xffffff, v); }
static void blitter_w(int offset, uint16_t data, uint16_t mask)
{
	t.blitter[offset] = (uint16_t)((t.blitter[offset] & ~mask) | (data & mask));
	if (offset != 0x10 / 2 || !(mask & 0xff00))
		return;
	const uint16_t *r = t.blitter;
	const int minterm = (r[0] & 0xff00) >> 8;
	int list_len = r[0] & 0xff;
	uint32_t src = ((uint32_t)r[2] << 16 | r[3]) & ~1u, list = ((uint32_t)r[4] << 16 | r[5]) & ~1u;
	uint32_t dest = ((uint32_t)r[6] << 16 | r[7]) & ~1u;
	int size = r[8] & 0xff;
	if (minterm != 2) {
		for (; size > 0; size--, src += 2, dest += 2)
			mspace_w(dest, mspace_r(src));
		return;
	}
	const int color_offs = s_hot ? 0 : 0x40;
	for (; list_len > 0; list_len--) {
		uint32_t i = src + mspace_r(list + 2);
		const uint32_t j = i + ((uint32_t)size << 1);
		uint32_t destptr = dest;
		for (; i < j; destptr += 2, i += 2)
			mspace_w(destptr, mspace_r(i));
		mspace_w(dest + 14, (uint16_t)(mspace_r(list) + color_offs));
		dest += 16;
		list += 4;
	}
	mspace_w(dest, 0xffff);   // (the sprite list's end marker)
}

static uint16_t sub_r16(uint32_t a) { (void)a; return 0; }
static uint8_t sub_r8(uint32_t a) { (void)a; return 0; }
static void sub_w16(uint32_t a, uint16_t v) { (void)a; (void)v; }
static void sub_w8(uint32_t a, uint8_t v) { (void)a; (void)v; }

// ---- the sound CPUs ----
static void k007232_port(uint8_t data)   // (wecleman_volume_callback)
{
	k007232_set_volume(&s_k232[0], 0, (data >> 4) * 0x11, 0);
	k007232_set_volume(&s_k232[0], 1, 0, (data & 0x0f) * 0x11);
}
static uint8_t k7452_r(int o)
{
	switch (o & 7) {
	case 0: return (uint8_t)t.k7452_mul;
	case 1: return (uint8_t)(t.k7452_mul >> 8);
	case 2: return (uint8_t)t.k7452_rem;
	case 3: return (uint8_t)(t.k7452_rem >> 8);
	case 4: return (uint8_t)t.k7452_quo;
	case 5: return (uint8_t)(t.k7452_quo >> 8);
	default: return 0;
	}
}
static void k7452_w(int o, uint8_t d)
{
	o &= 7;
	if (o < 6)
		t.k7452_regs[o] = d;
	if (o == 1)
		t.k7452_mul = (uint16_t)(t.k7452_regs[0] * t.k7452_regs[1]);
	else if (o == 5) {
		const uint16_t dividend = (uint16_t)(t.k7452_regs[4] << 8 | t.k7452_regs[5]);
		const uint16_t divisor = (uint16_t)(t.k7452_regs[2] << 8 | t.k7452_regs[3]);
		if (!divisor) {
			t.k7452_quo = 0xffff;
			t.k7452_rem = 0;
		} else {
			t.k7452_quo = dividend / divisor;
			t.k7452_rem = dividend % divisor;
		}
	}
}

static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0x8400)
		return t.zram[a - 0x8000];
	if (a >= 0x9000 && a < 0x9008)
		return k7452_r(a - 0x9000);
	if (a == 0xa000)
		return t.soundlatch;
	if (a >= 0xb000 && a < 0xb00e)
		return k007232_read(&s_k232[0], a - 0xb000);
	if (a == 0xc000 || a == 0xc001)
		return rbx_ym_read(0, a & 1);
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8400)
		t.zram[a - 0x8000] = v;
	else if (a >= 0x9000 && a < 0x9008)
		k7452_w(a - 0x9000, v);
	else if (a >= 0xb000 && a < 0xb00e)
		k007232_write(&s_k232[0], a - 0xb000, v);
	else if (a == 0xc000 || a == 0xc001)
		rbx_ym_write(0, a & 1, v);
	else if (a == 0xf000)
		k007232_set_bank(&s_k232[0], 0, ~v & 1);
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static uint8_t m09_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x0800)
		return t.zram[a];
	if (a >= 0x8000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	const int chip = (a >> 12) - 1;
	if (chip >= 0 && chip < 3 && (a & 0xfff) < 0x0e)
		return k007232_read(&s_k232[chip], (a & 0xf) ^ 1);
	if (a == 0x6000)
		return t.soundlatch;
	return 0;
}
static void m09_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0800) {
		t.zram[a] = v;
		return;
	}
	const int chip = (a >> 12) - 1;
	if (chip >= 0 && chip < 3 && (a & 0xfff) < 0x0e)
		k007232_write(&s_k232[chip], (a & 0xf) ^ 1, v);
	else if (a >= 0x4000 && a < 0x4008) {   // (hotchase_sound_control_w)
		const int o = a & 7;
		if (o < 6)
			k007232_set_volume(&s_k232[o >> 1], o & 1, (v & 0x0f) * 0x08, (v >> 4) * 0x08);
		else if (o == 6) {
			k007232_set_bank(&s_k232[0], (v >> 1) & 1, (v >> 3) & 1);
			k007232_set_bank(&s_k232[1], (v >> 2) & 1, (v >> 4) & 1);
		} else
			k007232_set_bank(&s_k232[2], v & 7, (v >> 3) & 7);
	} else if (a == 0x7000)
		t.sound_hs = 1;
}

// n samples of WEC Le Mans's board (on the worker): the Z80, the YM2151, the K007232
static void wec_sound(int n)
{
	if (n > 64)
		n = 64;
	int32_t l[64], r[64];
	t.z_frac += 3579545ull * (uint64_t)n;
	const unsigned long cyc = (unsigned long)(t.z_frac / SND_RATE);
	t.z_frac -= (uint64_t)cyc * SND_RATE;
	t.z_end += cyc;
	if (t.z_run) {
		int line = t.z80_irq;
		z80_run(&s_z, (unsigned long)t.z_end, &line, 1);
		t.z80_irq = (uint8_t)line;
	} else
		s_z.cyc = (unsigned long)t.z_end;
	int32_t yl[96], yr[96];
	memset(yl, 0, sizeof(yl));
	memset(yr, 0, sizeof(yr));
	t.ym_frac += 3579545ull * (uint64_t)n;
	const uint32_t yc = (uint32_t)(t.ym_frac / SND_RATE);
	t.ym_frac -= (uint64_t)yc * SND_RATE;
	const int yn = rbx_ym_advance_lr(0, yc, yl, yr, 96);
	for (int i = 0; i < n; i++) {
		t.k232_frac += 3579545 / 128;
		while (t.k232_frac >= SND_RATE) {
			t.k232_frac -= SND_RATE;
			k007232_sample(&s_k232[0], &t.k232_out[0][0], &t.k232_out[0][1]);
		}
		const int k = yn > 0 ? i * yn / n : -1;
		const int32_t c = (t.k232_out[0][0] + t.k232_out[0][1]) / 5;   // (both outputs, 0.20, to each side)
		l[i] = (k >= 0 ? yl[k] : 0) + c;
		r[i] = (k >= 0 ? yr[k] : 0) + c;
	}
	for (int i = 0; i < n && s_out_pos < 2048; i++, s_out_pos++) {
		s_out[0][s_out_pos] = l[i];
		s_out[1][s_out_pos] = r[i];
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
		case Q_ADV: wec_sound((int)e->v); break;
		case Q_LATCH: t.soundlatch = (uint8_t)e->v; break;
		case Q_IRQ: t.z80_irq = 1; break;
		case Q_ZRUN: wec_z_run((int)e->v); break;
		}
	}
	s_qn[b] = 0;
	s_out_n = s_out_pos;
	__atomic_store_n(&s_job_frame, 0, __ATOMIC_RELEASE);
}

static void wec_sound_frame(void)
{
	rbx_aux_sync();
	for (int i = 0; i < s_out_n; i++)
		rbx_audio_put_stereo(s_out[0][i], s_out[1][i]);
	s_out_n = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

static void sound_line(int n)
{
	if (n <= 0)
		return;
	if (n > 64)
		n = 64;
	int32_t l[64], r[64];
	memset(l, 0, sizeof(l));
	memset(r, 0, sizeof(r));
	if (!s_hot) {
		q_put(Q_ADV, (uint32_t)n);
		return;
	} else {
		// (the 6809E at 3.58 MHz / 2, its FIRQ at 496 Hz)
		t.firq_frac += 496ull * (uint64_t)n;
		if (t.firq_frac >= SND_RATE) {
			t.firq_frac -= SND_RATE;
			if (t.z_run)
				m6809_set_firq(&s_m09, 1);
		}
		t.m09_end += (int64_t)(1789772ull * (uint64_t)n / SND_RATE);
		if (t.z_run) {
			while (s_m09.cycles < t.m09_end) {
				const uint8_t irq = s_m09.irq_line, firq = s_m09.firq_line;
				const uint8_t cc = s_m09.cc;
				m6809_step(&s_m09);
				// (HOLD_LINE: dropped once the CPU took it: its mask bit now set, E set for an IRQ, clear for a FIRQ)
				if (irq && !(cc & 0x10) && (s_m09.cc & 0x90) == 0x90)
					m6809_set_irq(&s_m09, 0);
				if (firq && !(cc & 0x40) && (s_m09.cc & 0xc0) == 0x40)
					m6809_set_firq(&s_m09, 0);
			}
		} else
			s_m09.cycles = t.m09_end;
		for (int i = 0; i < n; i++) {
			t.k232_frac += 3579545 / 128;
			while (t.k232_frac >= SND_RATE) {
				t.k232_frac -= SND_RATE;
				for (int c = 0; c < 3; c++)
					k007232_sample(&s_k232[c], &t.k232_out[c][0], &t.k232_out[c][1]);
			}
			for (int c = 0; c < 3; c++) {   // (output 0 left, 1 right, 0.20)
				l[i] += t.k232_out[c][0] / 5;
				r[i] += t.k232_out[c][1] / 5;
			}
		}
	}
	for (int i = 0; i < n && s_snd_n < 2048; i++, s_snd_n++) {
		s_snd[0][s_snd_n] = l[i];
		s_snd[1][s_snd_n] = r[i];
	}
}

// ---- init ----
static void bitswap_region(uint8_t *src, uint32_t len, const int *b21)   // (b21: address bits 20..0 from these)
{
	uint8_t *buf = (uint8_t *)malloc(len);
	if (!buf)
		return;
	memcpy(buf, src, len);
	for (uint32_t i = 0; i < len; i++) {
		uint32_t a = i & ~0x1fffffu;
		for (int k = 0; k < 21; k++)
			a |= ((i >> b21[k]) & 1u) << (20 - k);
		src[i] = a < len ? buf[a] : 0;
	}
	free(buf);
}
static inline uint8_t fix_nibbles(uint8_t d)
{
	if ((d & 0xf0) == 0xf0) d &= 0x0f;
	if ((d & 0x0f) == 0x0f) d &= 0xf0;
	return d;
}

static void we_exit(void);
static void we_reset(void);

static int we_init(const rbx_game *g)
{
	s_hot = !strcmp(g->machine, "hotchase");
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_subrom = rbx_region("sub", &s_subrom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_spr = rbx_region("sprites", &s_spr_size);
	s_road = rbx_region("road", &s_road_size);
	s_layers = s_hot ? NULL : rbx_region("layers", &s_layers_size);
	uint32_t ks[3] = { 0, 0, 0 }, zs[2] = { 0, 0 };
	const uint8_t *pcm[3] = { rbx_region("k007232_1", &ks[0]), s_hot ? rbx_region("k007232_2", &ks[1]) : NULL,
	                          s_hot ? rbx_region("k007232_3", &ks[2]) : NULL };
	const uint8_t *zr[2] = { s_hot ? rbx_region("k051316_1", &zs[0]) : NULL, s_hot ? rbx_region("k051316_2", &zs[1]) : NULL };
	if (!s_rom || !s_subrom || !s_zrom || !s_spr || !s_road || !pcm[0] || (!s_hot && !s_layers) ||
	    (s_hot && (!pcm[1] || !pcm[2] || !zr[0] || !zr[1]))) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	if (!s_hot) {   // (init_wecleman)
		for (uint32_t i = 0; i < s_spr_size; i++) {   // (bitswap<8>(7,0,1,2,3,4,5,6))
			const uint8_t d = s_spr[i];
			uint8_t o = d & 0x80;
			for (int k = 0; k < 7; k++)
				o |= (uint8_t)(((d >> k) & 1) << (6 - k));
			s_spr[i] = o;
		}
		static const int spr_b[21] = { 0, 1, 20, 19, 18, 17, 14, 9, 16, 6, 4, 7, 8, 15, 10, 11, 13, 5, 12, 3, 2 };
		static const int lay_b[21] = { 20, 19, 18, 17, 16, 15, 12, 7, 14, 4, 2, 5, 6, 13, 8, 9, 11, 3, 10, 1, 0 };
		static const int road_b[21] = { 20, 19, 18, 17, 16, 15, 14, 7, 12, 4, 2, 5, 6, 13, 8, 9, 11, 3, 10, 1, 0 };
		bitswap_region(s_spr, s_spr_size, spr_b);
		uint8_t *src = s_spr + s_spr_size / 2 - 1, *dst = s_spr + s_spr_size - 1;   // (wecleman_unpack_sprites)
		while (dst > src) {
			const uint8_t d = fix_nibbles(*src--);
			*dst-- = d & 0xf;
			*dst-- = d >> 4;
		}
		bitswap_region(s_layers, s_layers_size, lay_b);
		bitswap_region(s_road, s_road_size, road_b);
		// (wecleman_bg_layout: 3 planes 0x8000 apart, the first pixel bit 7, the others bits 0-6)
		s_tilegfx = (uint8_t *)malloc(4096 * 64);
		s_roadgfx = (uint8_t *)malloc(256 * 512);
		if (!s_tilegfx || !s_roadgfx)
			return -1;
		for (int c = 0; c < 4096; c++)
			for (int y = 0; y < 8; y++)
				for (int x = 0; x < 8; x++) {
					const int bit = x ? x - 1 : 7;
					int p = 0;
					for (int pl = 0; pl < 3; pl++)
						p = p << 1 | ((s_layers[pl * 0x8000 + c * 8 + y] >> bit) & 1);
					s_tilegfx[c * 64 + y * 8 + x] = (uint8_t)p;
				}
		s_tilegfx[0xaca * 64 + 7] = 0;   // (video_start's patch)
		// (wecleman_road_layout: 64x1, planes at 0x8000 0x4000 0 (the first the high bit), 8 bytes an element)
		for (int e = 0; e < 2048; e++)
			for (int x = 0; x < 64; x++) {
				const int off = (x & ~7) + ((x & 7) ? 8 - (x & 7) : 0);   // (0,7,6,5,4,3,2,1 within each byte)
				const int byte = e * 8 + off / 8, bit = 7 - (off & 7);
				int p = 0;
				for (int pl = 0; pl < 3; pl++) {
					const int base = (2 - pl) * 0x4000;
					p = p << 1 | ((s_road[base + byte] >> bit) & 1);
				}
				s_roadgfx[e * 64 + x] = (uint8_t)p;
			}
	} else {   // (init_hotchase: hotchase_sprite_decode(3, 0x100000))
		const uint32_t bank_size = 0x100000;
		uint8_t *temp = (uint8_t *)malloc(bank_size);
		if (!temp)
			return -1;
		for (int i = 3; i > 0; i--) {
			uint8_t *finish = s_spr + 2 * bank_size * (uint32_t)i, *dest = finish - 2 * bank_size;
			memcpy(temp, s_spr + bank_size * (uint32_t)(i - 1), bank_size);
			const uint8_t *p1 = temp, *p2 = temp + bank_size / 2;
			do {
				for (int k = 0; k < 2; k++) {
					const uint8_t d = fix_nibbles(*p1++);
					*dest++ = d >> 4;
					*dest++ = d & 0xf;
				}
				for (int k = 0; k < 2; k++) {
					const uint8_t d = fix_nibbles(*p2++);
					*dest++ = d >> 4;
					*dest++ = d & 0xf;
				}
			} while (dest < finish);
		}
		free(temp);
		// (hotchase_road_layout: 64x1 4 bpp packed, each nibble twice, 16 bytes an element)
		s_roadgfx = (uint8_t *)malloc(8192 * 64);
		if (!s_roadgfx)
			return -1;
		for (int e = 0; e < 8192; e++)
			for (int x = 0; x < 64; x++) {
				const int nib = x >> 1;
				const uint8_t b = s_road[e * 16 + nib / 2];
				s_roadgfx[e * 64 + x] = (nib & 1) ? (b & 15) : (b >> 4);
			}
		for (int i = 0; i < 2; i++)
			if (k051316_init(&s_k316[i], i, zr[i], zs[i], 8, -16, zoom_cb))
				return -1;
	}
	if (m68k_multi_init(2)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	m68k_init();
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_bus_swap16(s_subrom, s_subrom_size);
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	for (int c = 0; c < 2; c++) {
		m68k_bus_reset_map();
		if (!c) {
			m68k_bus_map(0x000000, 0x03ffff, s_rom, s_rom_size, 1, 0);
			m68k_bus_map(0x130000, 0x130fff, t.spriteram, sizeof(t.spriteram), 1, 1);
			if (!s_hot) {
				m68k_bus_map(0x040000, 0x040fff, t.ram, 0x1000, 1, 0);   // (writes: the videostatus hook)
				m68k_bus_map(0x041000, 0x043fff, (uint8_t *)t.ram + 0x1000, 0x3000, 1, 1);
				m68k_bus_map(0x080000, 0x080fff, t.blitter, sizeof(t.blitter), 1, 0);
				m68k_bus_map(0x100000, 0x103fff, t.pageram, sizeof(t.pageram), 1, 1);
				m68k_bus_map(0x108000, 0x108fff, t.txtram, sizeof(t.txtram), 1, 1);
				m68k_bus_map(0x110000, 0x110fff, t.palram, 0x1000, 1, 0);
				m68k_bus_map(0x124000, 0x127fff, t.share, sizeof(t.share), 1, 1);
			} else {
				m68k_bus_map(0x040000, 0x041fff, t.ram, 0x2000, 1, 1);
				m68k_bus_map(0x060000, 0x063fff, t.ram2, sizeof(t.ram2), 1, 1);
				m68k_bus_map(0x080000, 0x080fff, t.blitter, sizeof(t.blitter), 1, 0);
				m68k_bus_map(0x110000, 0x111fff, t.palram, 0x2000, 1, 0);
				m68k_bus_map(0x120000, 0x123fff, t.share, sizeof(t.share), 1, 1);
			}
			m68k_bus.read8 = main_r8;
			m68k_bus.read16 = main_r16;
			m68k_bus.write8 = main_w8;
			m68k_bus.write16 = main_w16;
		} else {
			m68k_bus_map(0x000000, s_hot ? 0x01ffff : 0x00ffff, s_subrom, s_subrom_size, 1, 0);
			if (!s_hot) {
				m68k_bus_map(0x060000, 0x060fff, t.roadram, sizeof(t.roadram), 1, 1);
				m68k_bus_map(0x070000, 0x073fff, t.share, sizeof(t.share), 1, 1);
			} else {
				m68k_bus_map(0x020000, 0x020fff, t.roadram, sizeof(t.roadram), 1, 1);
				m68k_bus_map(0x040000, 0x043fff, t.share, sizeof(t.share), 1, 1);
				m68k_bus_map(0x060000, 0x060fff, t.subram, 0x1000, 1, 1);
				m68k_bus_map(0x061000, 0x06101f, t.subram + 0x800, 0x20, 1, 1);
			}
			m68k_bus.read8 = sub_r8;
			m68k_bus.read16 = sub_r16;
			m68k_bus.write8 = sub_w8;
			m68k_bus.write16 = sub_w16;
		}
		m68k_multi_capture(c);
	}
	m68k_multi_finish();
	for (int c = 0; c < (s_hot ? 3 : 1); c++)
		k007232_init(&s_k232[c], pcm[c], ks[c], s_hot ? NULL : k007232_port);
	if (!s_hot) {
		rbx_ym_config yc;
		memset(&yc, 0, sizeof(yc));
		yc.type = RBX_YM2151;
		yc.clock = 3579545;
		yc.gain[0] = yc.gain[1] = 218;   // (0.85)
		if (rbx_ym_init(0, &yc)) {
			rbx_fail("%s: YM2151", g->set);
			return -1;
		}
	}
	s_port_in0 = rbx_port_find("IN0");
	s_port_in1 = rbx_port_find("IN1");
	s_port_dsw_lo = rbx_port_find(s_hot ? "DSW2" : "DSWA");
	s_port_dsw_hi = rbx_port_find(s_hot ? "DSW1" : "DSWB");
	s_port_accel = rbx_port_find("ACCEL");
	s_port_steer = rbx_port_find("STEER");
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, 0x10000)) {
		rbx_fail("%s: out of memory", g->set);
		we_exit();
		return -1;
	}
	for (int i = 0; i < 0x10000; i++) {   // (the bitmap: RGB 565)
		const int r = (i >> 11) & 31, gg = (i >> 5) & 63, b = i & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (gg << 2 | gg >> 4) << 8 | (b << 3 | b >> 2));
	}
	we_reset();
	rbx_aux_set_job(s_hot ? NULL : job);
	return 0;
}

static void we_exit(void)
{
	rbx_aux_set_job(NULL);
	m68k_multi_exit();
	if (!s_hot)
		rbx_ym_exit(0);
	else
		for (int i = 0; i < 2; i++)
			k051316_exit(&s_k316[i]);
	free(s_tilegfx);
	free(s_roadgfx);
	s_tilegfx = s_roadgfx = NULL;
}

static void we_reset(void)
{
	rbx_aux_sync();
	rbx_video_wait_idle();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	t.cloud_blend = BLEND_MAX;
	s_snd_n = 0;
	if (s_hot) {   // (machine_reset: the palette white)
		for (int i = 0; i < 0x1000; i++) {
			t.palram[i] = 0xffff;
			t.pens[i] = 0xffffff;
		}
		for (int i = 0; i < 2; i++)
			k051316_reset(&s_k316[i]);
		for (int c = 0; c < 3; c++)
			k007232_reset(&s_k232[c]);
		m6809_init(&s_m09);
		s_m09.read = m09_rd;
		s_m09.write = m09_wr;
		s_m09.ctx = NULL;
		for (int p = 0; p < 8; p++)   // (its RAM and ROM read directly)
			s_m09.rpage[p] = s_m09.wpage[p] = t.zram + p * 256;
		for (int p = 0x80; p < 0x100; p++)
			s_m09.rpage[p] = (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
		m6809_reset(&s_m09);
	} else {
		k007232_reset(&s_k232[0]);
		k007232_set_bank(&s_k232[0], 0, 1);   // (machine_reset)
		rbx_ym_reset(0);
		z80_init(&s_z);
		s_z.read_byte = z_rd;
		s_z.write_byte = z_wr;
		s_z.port_in = z_in;
		s_z.port_out = z_out;
		s_z.int_data = 0xff;
		memset(s_zmap, 0, sizeof(s_zmap));
		for (int p = 0; p < 0x80; p++)
			s_zmap[p] = (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
		s_z.read_map = s_zmap;
	}
	m68k_multi_reset();
	for (int c = 0; c < 2; c++)
		m68k_multi_set_asserted(c, 0);
}

static void we_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		// (the scanline timer: WEC IRQ 4 at 232 and IRQ 5 every 64 lines; Hot Chase IRQ 4 at 224)
		if (!s_hot) {
			if (line == 232)
				m68k_multi_hold_irq(0, 4);
			else if (!(line % 64))
				m68k_multi_hold_irq(0, 5);
		} else if (line == 224)
			m68k_multi_hold_irq(0, 4);
		if (line == (s_hot ? 224 : 232)) {   // (the frame's update at the vblank)
			rbx_video_wait_idle();
			capture();
			rbx_video_frame_done();
		}
		t.cpu_frac += CPU_CLOCK;
		const int cyc = (int)(t.cpu_frac / (60 * LINES));
		t.cpu_frac -= (uint64_t)cyc * 60 * LINES;
		for (int c = 0; c < 2; c++) {
			if (c && t.sub_reset) {
				t.sub_reset = 0;
				m68k_multi_reset_cpu(1);
			}
			if (c && !t.sub_run)
				continue;
			m68k_multi_switch(c);
			int left = cyc;
			while (left > 0)
				left -= m68k_execute(left);
		}
		m68k_multi_switch(0);
		t.snd_frac += SND_RATE;
		const int ns = (int)(t.snd_frac / (60 * LINES));
		t.snd_frac -= (uint64_t)ns * 60 * LINES;
		sound_line(ns);
	}
	if (!s_hot)
		wec_sound_frame();
	else {
		for (int i = 0; i < s_snd_n; i++)
			rbx_audio_put_stereo(s_snd[0][i], s_snd[1][i]);
		s_snd_n = 0;
	}
	t.frame++;
}

static int we_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rbx_video_wait_idle();
	uint32_t asserted[2] = { m68k_multi_asserted(0), m68k_multi_asserted(1) };
	rc |= m68k_multi_state(ctx, load);
	rc |= rbx_state_io(ctx, "we_irq", asserted, sizeof(asserted));
	rc |= rbx_state_io(ctx, "we", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	for (int c = 0; c < (s_hot ? 3 : 1); c++) {
		static const char *const tags[3] = { "we_k2a", "we_k2b", "we_k2c" };
		rc |= k007232_state(&s_k232[c], ctx, load, tags[c]);
	}
	if (s_hot) {
		rc |= k051316_state(&s_k316[0], ctx, load, "we_316a");
		rc |= k051316_state(&s_k316[1], ctx, load, "we_316b");
		rc |= rbx_state_io(ctx, "we_6809", &s_m09, M6809_STATE_SIZE);
	} else {
		rc |= rbx_ym_state(0, ctx, load);
		int32_t qn = s_out_n;
		rc |= rbx_state_io(ctx, "we_q", &qn, sizeof(qn));
		rc |= rbx_state_io(ctx, "we_out", s_out, sizeof(s_out));
		if (!rc && load) {
			s_out_n = qn >= 0 && qn <= 2048 ? qn : 0;
			s_qn[0] = s_qn[1] = 0;
			s_job_frame = 0;
		}
		z80 z = s_z;
		rc |= rbx_state_io(ctx, "we_z80", &z, sizeof(z));
		if (!rc && load) {
			z.read_byte = z_rd;
			z.write_byte = z_wr;
			z.port_in = z_in;
			z.port_out = z_out;
			z.userdata = NULL;
			z.read_map = s_zmap;
			z.op_rom = NULL;
			s_z = z;
		}
	}
	if (!rc && load) {
		m68k_multi_set_asserted(0, asserted[0]);
		m68k_multi_set_asserted(1, asserted[1]);
	}
	return rc ? -1 : 0;
}

static int we_save(void *ctx) { return we_state(ctx, 0); }
static int we_load(void *ctx) { return we_state(ctx, 1); }
static int we_supported(const char *machine) { return !strcmp(machine, "wecleman") || !strcmp(machine, "hotchase"); }

const rbx_driver rbx_drv_wecleman = { "Konami WEC Le Mans 24", we_init, we_exit, we_reset, we_frame, we_save, we_load,
                                      we_supported, we_render, NULL };
