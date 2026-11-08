/*  Konami 68000 based medal games. Ported from MAME src/mame/konami/konmedal68k.cpp (BSD-3-Clause, copyright-holders:
    R. Belmont), with the drawing of k054156_k054157_k056832.cpp (tilemap_draw_common, tilemap_draw_dj: David Haywood)
    and the hopper of src/devices/machine/ticket.cpp (BSD-3-Clause, copyright-holders: Aaron Giles).
    A 68000 at 8.4672 MHz, the K056832 tilemaps (4 bpp, no sprites), the K055555 only for the background colour and the
    layer enables (the layers in a fixed order), a YMZ280B. The screen 59.62 Hz; the slot machines' 512 lines high.
    The pixels are palette indices: MAME's colours exact, the solid background colour (a raw RGB value in MAME's
    fill_backcolor) one more entry of the palette.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../sound/ymz280b.h"
#include "../video/k056832.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAIN_CLOCK 8467200u    // (XTAL(33'868'800) / 4)
#define YMZ_CLOCK 16934400u    // (XTAL(33'868'800) / 2)
#define FRAME_MHZ 59620u
#define HOPPER_PERIOD 846720   // (the hopper's 100 ms in CPU clocks)
#define NUM_LAYERS 4
#define K55_INPUT_ENABLES 45

enum { M_KZAURUS, M_KOROPENS, M_PWRCHANC, M_SPCPOKAN, M_GS662, M_SLOT };

static uint8_t *s_rom;
static uint32_t s_rom_size;
static ymz280b_t s_ymz;
static int32_t *s_ymz_l, *s_ymz_r;
static int s_ymz_cap, s_ymz_n;
static uint32_t s_ymz_line0, s_ymz_line_n;
static int s_port_in0, s_port_in1, s_port_dsw;
static k056832_t s_k;
static int s_m;                      // M_*
static int s_lines, s_vbl;           // the screen's lines, m_vbl_scanline
static int s_scr_w, s_scr_h;         // the screen's size (the tilemaps' flip)
static int s_x0, s_y0, s_w, s_h;     // the visible area
static int s_pal_n;                  // the palette entries the CPU reaches
static uint8_t *s_tiles_raw, *s_tiles;   // the decoded tiles, 64 bytes each
static uint32_t s_ntiles;
static uint32_t s_pens[0x4000];      // the palette in 0x00RRGGBB
static char s_nv_path[512];

static struct {
	uint8_t ram[0x4000];        // 400000: the NVRAM (host-order words)
	uint16_t pal[0x8000];       // b00000: high word first; the slot machines' 4096 words
	uint8_t extra[0xc000];      // b04000: the slot machines' RAM (host-order words)
	uint8_t k55555[0x80];
	uint8_t control, control2;
	uint8_t hop_status, hop_power;
	int64_t hop_due;            // (the hopper's timer in CPU clocks, -1 off)
	int64_t cyc_end, cycles_done;
	uint64_t cyc_frac, snd_frac;
	int32_t line_cyc;
	uint8_t state_end;
} b;

// ---- the snapshot at vblank (what the drawing reads) ----
typedef struct {
	k056832_t k;
	uint32_t pens[0x4000];
	uint8_t k55555[0x80];
} snap_t;
static snap_t *s_snap;

// ---- the YMZ280B, rendered up to the CPU's time before each access ----
static void ymz_to(int n)
{
	if (n > s_ymz_cap)
		n = s_ymz_cap;
	if (n > s_ymz_n) {
		ymz280b_render(&s_ymz, s_ymz_l + s_ymz_n, s_ymz_r + s_ymz_n, n - s_ymz_n);
		s_ymz_n = n;
	}
}
static void ymz_sync(void)
{
	const int d = m68k_cycles_run();
	ymz_to((int)(s_ymz_line0 + (b.line_cyc > 0 ? (uint32_t)((uint64_t)(d < b.line_cyc ? d : b.line_cyc) * s_ymz_line_n / (uint32_t)b.line_cyc) : 0)));
}

// ---- the hopper (ticket_dispenser_device, hopper type) ----
static void hopper_motor(int state)
{
	if (state && !b.hop_power) {
		b.hop_due = b.cycles_done + m68k_cycles_run() + HOPPER_PERIOD;
		b.hop_power = 1;
		b.hop_status = 0;
	} else if (!state && b.hop_power) {
		if (!b.hop_status)
			b.hop_due = -1;
		b.hop_power = 0;
	}
}
static void hopper_tick(void)
{
	if (b.hop_due >= 0 && b.cycles_done >= b.hop_due) {
		b.hop_status ^= 1;
		b.hop_due = b.hop_power ? b.hop_due + HOPPER_PERIOD : -1;
	}
}

// ---- the palette ----
static inline uint32_t pal4(int v) { v &= 15; return (uint32_t)(v << 4 | v); }

static void pen_update(int i)
{
	if (s_m == M_SLOT) {   // (xBGR_444)
		const int v = b.pal[i];
		s_pens[i] = pal4(v) << 16 | pal4(v >> 4) << 8 | pal4(v >> 8);
	} else {   // (xBGR_888, the high word first)
		const uint32_t c = (uint32_t)b.pal[i * 2] << 16 | b.pal[i * 2 + 1];
		s_pens[i] = (c & 0xff) << 16 | (c & 0xff00) | (c >> 16 & 0xff);
	}
}

// ---- the tile ROM's readback ----
static uint16_t rom_word(uint32_t addr)   // (the word of piratesh_rom_r, rom_word_r)
{
	const uint32_t n = s_k.rom_size;
	return (uint16_t)((addr + 1 < n ? s_k.rom[addr + 1] : 0) | (addr < n ? s_k.rom[addr] : 0) << 8);
}
static uint16_t vrom_r(uint32_t offset)
{
	const uint32_t bank = 0x2000u * (uint32_t)s_k.cur_gfx_banks;
	if (s_m == M_KOROPENS || s_m == M_PWRCHANC || s_m == M_GS662)   // (vrom_koropens_r)
		return (b.control2 & 0x10) ? 0 : rom_word(bank + 2 * offset);
	if (b.control2 & 0x10)
		offset |= 0x1000;
	if (s_m == M_SPCPOKAN && (offset & 1)) {   // (vrom_spcpokan_r)
		offset |= 0x100000;
		offset &= ~1u;
	}
	return rom_word(bank + offset);
}

// ---- the main CPU's bus ----
static int vram_window(uint32_t a)
{
	if (a >= 0xa00000 && a < 0xa04000)
		return 1;
	return (s_m == M_PWRCHANC || s_m == M_GS662) && a >= 0xa04000 && a < 0xa08000;
}

static uint16_t main_r16(uint32_t a)
{
	a &= 0xffffff;
	if (a == 0x800004)
		return (uint16_t)rbx_port_read(s_port_dsw);
	if (a == 0x800006) {
		const uint32_t v = rbx_port_read(s_port_in1);
		return s_m == M_SLOT ? (uint16_t)v : (uint16_t)((v & ~0x100u) | (b.hop_status ? 0x100 : 0));
	}
	if (a == 0x800008)
		return (uint16_t)rbx_port_read(s_port_in0);
	if (a >= 0x830000 && a < 0x830040)
		return k056832_word_r(&s_k, (int)((a - 0x830000) >> 1));
	if (a >= 0x880000 && a < 0x880004) {   // (the YMZ280B: umask ff00)
		ymz_sync();
		return (uint16_t)(ymz280b_read(&s_ymz, (int)((a - 0x880000) >> 1)) << 8);
	}
	if (vram_window(a))
		return k056832_ram_word_r(&s_k, (int)((a & 0x1fff) >> 1));
	if (s_m == M_SLOT) {
		if (a >= 0xb00000 && a < 0xb04000) {
			const uint32_t o = (a - 0xb00000) >> 1;
			return (o & 1) ? b.pal[o >> 1] : 0;
		}
	} else if (a >= 0xb00000 && a < (s_m == M_GS662 ? 0xb10000u : 0xb04000u))
		return b.pal[(a - 0xb00000) >> 1];
	if (a >= 0xc00000 && a < 0xc02000)
		return vrom_r((a - 0xc00000) >> 1);
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
	if (a == 0x800000) {   // (control_w: bits 3, 4 enable or ack IRQ 3, 4)
		b.control = (uint8_t)v;
		if (!(v & 0x08))
			m68k_bus_set_line(3, 0);
		if (!(v & 0x10))
			m68k_bus_set_line(4, 0);
	} else if (a == 0x810000)
		b.control2 = (uint8_t)v;
	else if (a == 0x820000) {
		if (s_m != M_SLOT && (mask & 0x80))
			hopper_motor((v & 0x80) != 0);
	} else if (a >= 0x830000 && a < 0x830040)
		k056832_word_w(&s_k, (int)((a - 0x830000) >> 1), v, mask);
	else if (a >= 0x840000 && a < 0x840010)
		k056832_b_word_w(&s_k, (int)((a - 0x840000) >> 1), v, mask);
	else if (a >= 0x870000 && a < 0x870060)   // (K055555_word_w)
		b.k55555[(a - 0x870000) >> 1] = (uint8_t)(mask == 0x00ff ? v : v >> 8);
	else if (a >= 0x880000 && a < 0x880004) {
		if (mask & 0xff00) {
			ymz_sync();
			ymz280b_write(&s_ymz, (int)((a - 0x880000) >> 1), (uint8_t)(v >> 8));
		}
	} else if (vram_window(a))
		k056832_ram_word_w(&s_k, (int)((a & 0x1fff) >> 1), v, mask);
	else if (s_m == M_SLOT) {
		if (a >= 0xb00000 && a < 0xb04000) {
			const uint32_t o = (a - 0xb00000) >> 1;
			if (o & 1) {
				uint16_t *p = &b.pal[o >> 1];
				*p = (uint16_t)((v & mask) | (*p & ~mask));
				pen_update((int)(o >> 1));
			}
		}
	} else if (a >= 0xb00000 && a < (s_m == M_GS662 ? 0xb10000u : 0xb04000u)) {
		const uint32_t o = (a - 0xb00000) >> 1;
		b.pal[o] = (uint16_t)((v & mask) | (b.pal[o] & ~mask));
		pen_update((int)(o >> 1));
	}
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v)   // (the 68000 drives a byte on both lanes)
{
	main_w16m(a & ~1u, (uint16_t)(v << 8 | v), (a & 1) ? 0x00ff : 0xff00);
}

// ---- the drawing (MAME's bitmap coordinates; the visible area clipped by the callers) ----
static inline const uint8_t *tile_info(const k056832_t *k, const uint16_t *mem, int layer, int line, int *fx, int *penbase,
                                       uint32_t *code)
{
	static const struct { int flips, palm1, pals2, palm2; } sm[4] = {
		{ 6, 0x3f, 0, 0x00 }, { 4, 0x0f, 2, 0x30 }, { 2, 0x03, 2, 0x3c }, { 0, 0x00, 2, 0x3f } };
	const int fbits = (k->regs[3] >> 6) & 3;
	const int attr = mem[0];
	const int flip = ((k->regs[1] >> (layer << 1)) & 3) & (attr >> sm[fbits].flips & 3);
	*code = mem[1];
	*fx = flip & 1;
	*penbase = ((attr & sm[fbits].palm1) | (attr >> sm[fbits].pals2 & sm[fbits].palm2)) * 16;
	return s_tiles + (size_t)rbx_wrap(*code, s_ntiles) * 64 + (size_t)(((flip & 2) ? 7 - line : line) * 8);
}

// the page's tilemap (64x32 tiles) over the rectangle at the scroll, its pen 0 transparent
static void draw_page(const k056832_t *k, int page, int layer, int x0, int x1, int y0, int y1, int scrollx, int scrolly)
{
	const int flipx = (k->regs[0] & 0x10) != 0, flipy = (k->regs[0] & 0x20) != 0;
	int tlayer = layer;
	if (k->layer_association) {   // (get_tile_info's layer)
		tlayer = k->layer_assoc_with_page[page];
		if (tlayer == -1)
			tlayer = 0;
	} else if (!page)
		tlayer = 0;
	const uint16_t *const vram = &k->videoram[page << 12];
	for (int y = y0; y <= y1; y++) {
		uint16_t *const dst = rbx_video.bitmap + (y - s_y0) * rbx_video.pitch - s_x0;
		const int sy = flipy ? (s_scr_h - 1 - y + scrolly) & 255 : (y + scrolly) & 255;
		const int ly = sy & 7, row = sy >> 3;
		if (!k->page_tile_mode[page]) {   // (update_linemap: the line 512 pixels of 8 tiles)
			int fx, pb;
			uint32_t code;
			(void)tile_info(k, vram + (sy << 1), tlayer, 0, &fx, &pb, &code);
			for (int x = x0; x <= x1; x++) {
				const int lx = flipx ? (s_scr_w - 1 - x + scrollx) & 511 : (x + scrollx) & 511;
				const int p = s_tiles[(size_t)rbx_wrap((code & ~7u) | (uint32_t)(lx >> 6), s_ntiles) * 64 + (size_t)(lx & 63)];
				if (p)
					dst[x] = (uint16_t)(pb + p);
			}
			continue;
		}
		for (int x = x0; x <= x1;) {
			const int sx = flipx ? (s_scr_w - 1 - x + scrollx) & 511 : (x + scrollx) & 511;
			int n = flipx ? (sx & 7) + 1 : 8 - (sx & 7);
			if (n > x1 - x + 1)
				n = x1 - x + 1;
			int fx, pb;
			uint32_t code;
			const uint8_t *const src = tile_info(k, vram + (((row << 6) + (sx >> 3)) << 1), tlayer, ly, &fx, &pb, &code);
			uint64_t all;
			memcpy(&all, src, 8);
			if (all) {
				int px = sx & 7;
				const int dir = flipx ? -1 : 1;
				for (int i = 0; i < n; i++, px += dir) {
					const int p = src[fx ? 7 - px : px];
					if (p)
						dst[x + i] = (uint16_t)(pb + p);
				}
			}
			x += n;
		}
	}
}

// the rows of a page in a layer (the line loop, the same in tilemap_draw_common and tilemap_draw_dj)
typedef struct {
	const k056832_t *k;
	int layer, ay, corr, width, colspan, flipx;
	const uint16_t *scroll;
	int line_height, sdat_wrapmask, sdat_adv;
	int cminx, cmaxx, cminy, cmaxy;
} lines_t;

static void draw_lines(const lines_t *l, int page, int c, int sdat_start, int line_starty, int line_endy, int clipy, int clipmaxy)
{
	uint32_t last_dx = 0x100000;
	int last_visible = 0, dminx = 0, dmaxx = -1, dx = 0;
	for (int sdat_walk = sdat_start, line_y = line_starty; line_y < line_endy; sdat_walk += l->sdat_adv, line_y += l->line_height) {
		int dminy = line_y, dmaxy = line_y + l->line_height - 1;
		if (dminy < clipy)
			dminy = clipy;
		if (dmaxy > clipmaxy)
			dmaxy = clipmaxy;
		if (dminy > l->cmaxy || dmaxy < l->cminy)
			continue;
		const int sdat_offs = sdat_walk & l->sdat_wrapmask;
		const int ry0 = dminy < l->cminy ? l->cminy : dminy, ry1 = dmaxy > l->cmaxy ? l->cmaxy : dmaxy;
		dx = (int)((uint32_t)l->scroll[sdat_offs] << 16 | l->scroll[sdat_offs + 1]) + l->corr;
		if (last_dx == (uint32_t)dx) {
			if (last_visible)
				draw_page(l->k, page, l->layer, dminx, dmaxx, ry0, ry1, dx, l->ay);
			continue;
		}
		last_dx = (uint32_t)dx;
		int clipw, clipx;
		if (l->colspan > 1) {
			int sx = (int)((unsigned)dx & (unsigned)(l->width - 1));
			int tx = c << 9;
			if (!l->flipx) {
				if (c == 0 && sx > l->width - K056832_PAGE_WIDTH)   // (right-edge wraparound, off-screen pages culled)
					sx -= l->width;
				if (sx + K056832_PAGE_WIDTH <= tx || sx - K056832_PAGE_WIDTH >= tx) {
					last_visible = 0;
					continue;
				}
				if ((tx -= sx) <= 0) {
					clipw = K056832_PAGE_WIDTH + tx;
					clipx = 0;
				} else {
					clipw = K056832_PAGE_WIDTH - tx;
					clipx = tx;
				}
			} else {
				tx += K056832_PAGE_WIDTH;
				if (c == l->colspan - 1 && sx < K056832_PAGE_WIDTH)
					sx += l->width;
				if (sx + K056832_PAGE_WIDTH <= tx || sx - K056832_PAGE_WIDTH >= tx) {
					last_visible = 0;
					continue;
				}
				if ((tx -= sx) >= 0) {
					clipw = K056832_PAGE_WIDTH - tx;
					clipx = 0;
				} else {
					clipw = K056832_PAGE_WIDTH + tx;
					clipx = -tx;
				}
			}
		} else {
			clipw = K056832_PAGE_WIDTH;
			clipx = 0;
		}
		last_visible = 1;
		dminx = clipx < l->cminx ? l->cminx : clipx;
		dmaxx = clipx + clipw - 1 > l->cmaxx ? l->cmaxx : clipx + clipw - 1;
		draw_page(l->k, page, l->layer, dminx, dmaxx, ry0, ry1, dx, l->ay);
	}
}

static inline int page_of(const k056832_t *k, int layer, int page)   // (the page belongs to the layer drawn)
{
	return k->layer_association ? k->layer_assoc_with_page[page] == layer : k->layer_assoc_with_page[page] != -1;
}

static void tilemap_draw(const k056832_t *k, int layer, int dj, int by0, int by1)   // (by0..by1: the band's rows)
{
	const int rowstart = k->y[layer], colstart = k->x[layer], rowspan = k->h[layer] + 1, colspan = k->w[layer] + 1;
	int dy = k->dy[layer];
	const int dx0 = k->dx[layer];
	const int scrollbank = ((k->regs[0x18] >> 1) & 0xc) | (k->regs[0x18] & 3);
	const int scrollmode = k->regs[0x05] >> (k->lsram_page[layer][0] << 1) & 3;
	const int height = rowspan * K056832_PAGE_HEIGHT, width = colspan * K056832_PAGE_WIDTH;
	int corr;
	const int flipy = k->regs[0] & 0x20;
	if (flipy) {
		corr = k->regs[0x3c / 2];
		if (corr & 0x400)
			corr |= (int)0xfffff800;
	} else
		corr = 0;
	dy += corr;
	const int ay = (int)((unsigned)(dy - k->layer_offs[layer][1]) % (unsigned)height);
	const int flipx = k->regs[0] & 0x10;
	if (flipx) {
		corr = k->regs[0x3a / 2];
		if (corr & 0x800)
			corr |= (int)0xfffff000;
	} else
		corr = 0;
	corr -= k->layer_offs[layer][0];
	uint16_t ram16[2];
	lines_t l = { k, layer, ay, corr, width, colspan, flipx != 0, NULL, 0, 0, 0, s_x0, s_x0 + s_w - 1, by0, by1 };
	switch (scrollmode) {
	case 0:   // linescroll
		l.scroll = &k->videoram[scrollbank << 12] + (k->lsram_page[layer][1] >> 1);
		l.line_height = 1;
		l.sdat_wrapmask = 0x3ff;
		l.sdat_adv = 2;
		break;
	case 2:   // rowscroll
		l.scroll = &k->videoram[scrollbank << 12] + (k->lsram_page[layer][1] >> 1);
		l.line_height = 8;
		l.sdat_wrapmask = 0x3ff;
		l.sdat_adv = 16;
		break;
	default:   // xyscroll
		l.scroll = ram16;
		l.line_height = K056832_PAGE_HEIGHT;
		l.sdat_wrapmask = 0;
		l.sdat_adv = 0;
		ram16[0] = 0;
		ram16[1] = (uint16_t)dx0;
		break;
	}
	if (flipy)
		l.sdat_adv = -l.sdat_adv;
	if (dj) {   // (tilemap_draw_dj: a page wrapping at the bottom drawn a second time at the top)
		int need_wrap = -1;
		for (int r = 0; r <= rowspan; r++) {
			int sy = ay, ty, cliph, clipy, line_starty, line_endy, sdat_start;
			if (r == rowspan) {
				if (need_wrap < 0)
					continue;
				ty = need_wrap * K056832_PAGE_HEIGHT;
			} else
				ty = r * K056832_PAGE_HEIGHT;
			if (sy + height <= ty || sy - height >= ty)
				continue;
			ty -= sy;
			if (r == rowspan) {
				cliph = K056832_PAGE_HEIGHT + ty;
				clipy = line_starty = 0;
				line_endy = cliph;
				ty = -ty;
				sdat_start = ty;
				if (scrollmode == 2) {
					sdat_start &= ~7;
					line_starty -= ty & 7;
				}
			} else {
				if (ty < 0)
					ty += height;
				clipy = ty;
				cliph = K056832_PAGE_HEIGHT;
				if (clipy + cliph > height) {
					cliph = height - clipy;
					need_wrap = r;
				}
				line_starty = ty;
				line_endy = line_starty + cliph;
				sdat_start = 0;
			}
			const int rr = r == rowspan ? need_wrap : r;
			sdat_start = (sdat_start + rr * K056832_PAGE_HEIGHT) << 1;
			for (int c = 0; c < colspan; c++) {
				const int page = (((rowstart + rr) & 3) << 2) + ((colstart + c) & 3);
				if (page_of(k, layer, page))
					draw_lines(&l, page, c, sdat_start, line_starty, line_endy, clipy, clipy + cliph - 1);
			}
		}
		return;
	}
	for (int r = 0; r < rowspan; r++) {   // (tilemap_draw_common)
		int cliph, clipy, line_starty, line_endy, sdat_start;
		if (rowspan > 1) {
			int sy = ay, ty = r * K056832_PAGE_HEIGHT;
			if (!flipy) {
				if (r == 0 && sy > height - K056832_PAGE_HEIGHT)   // (bottom-edge wraparound, off-screen pages culled)
					sy -= height;
				if (sy + K056832_PAGE_HEIGHT <= ty || sy - K056832_PAGE_HEIGHT >= ty)
					continue;
				if ((ty -= sy) >= 0) {
					cliph = K056832_PAGE_HEIGHT - ty;
					clipy = line_starty = ty;
					line_endy = K056832_PAGE_HEIGHT;
					sdat_start = 0;
				} else {
					cliph = K056832_PAGE_HEIGHT + ty;
					ty = -ty;
					clipy = line_starty = 0;
					line_endy = cliph;
					sdat_start = ty;
					if (scrollmode == 2) {
						sdat_start &= ~7;
						line_starty -= ty & 7;
					}
				}
			} else {
				ty += K056832_PAGE_HEIGHT;
				if (r == rowspan - 1 && sy < K056832_PAGE_HEIGHT)
					sy += height;
				if (sy + K056832_PAGE_HEIGHT <= ty || sy - K056832_PAGE_HEIGHT >= ty)
					continue;
				if ((ty -= sy) <= 0) {
					cliph = K056832_PAGE_HEIGHT + ty;
					clipy = line_starty = -ty;
					line_endy = K056832_PAGE_HEIGHT;
					sdat_start = K056832_PAGE_HEIGHT - 1;
					if (scrollmode == 2)
						sdat_start &= ~7;
				} else {
					cliph = K056832_PAGE_HEIGHT - ty;
					clipy = line_starty = 0;
					line_endy = cliph;
					sdat_start = cliph - 1;
					if (scrollmode == 2) {
						sdat_start &= ~7;
						line_starty -= ty & 7;
					}
				}
			}
		} else {
			cliph = line_endy = K056832_PAGE_HEIGHT;
			clipy = line_starty = 0;
			sdat_start = flipy ? K056832_PAGE_HEIGHT - 1 : dy;
			if (scrollmode == 2) {
				sdat_start &= ~7;
				line_starty -= dy & 7;
			}
		}
		sdat_start = (sdat_start + r * K056832_PAGE_HEIGHT) << 1;
		for (int c = 0; c < colspan; c++) {
			const int page = (((rowstart + r) & 3) << 2) + ((colstart + c) & 3);
			if (page_of(k, layer, page))
				draw_lines(&l, page, c, sdat_start, line_starty, line_endy, clipy, clipy + cliph - 1);
		}
	}
}

// the index of a pen of fill_backcolor's gradients (past the palette RAM: black)
#define PEN_SOLID (s_pal_n)
#define PEN_BLACK (s_pal_n + 1)
static inline uint16_t gradient_pen(int pen) { return (uint16_t)(pen < s_pal_n ? pen : PEN_BLACK); }

static void fill_backcolor(int pen_idx, int mode, int ry0, int ry1)   // (ry0..ry1: the rows, from the top)
{
	const int pitch = rbx_video.pitch;
	if (!(mode & 2)) {   // (solid: MAME fills the bitmap with the number itself, an RGB value)
		rbx_video.palette[PEN_SOLID] = (uint32_t)pen_idx;
		for (int y = ry0; y <= ry1; y++) {
			uint16_t *d = rbx_video.bitmap + y * pitch;
			for (int x = 0; x < s_w; x++)
				d[x] = (uint16_t)PEN_SOLID;
		}
	} else if (!(mode & 1)) {   // (vertical gradient)
		for (int y = ry0; y <= ry1; y++) {
			uint16_t *d = rbx_video.bitmap + y * pitch;
			const uint16_t p = gradient_pen(pen_idx + s_y0 + y);
			for (int x = 0; x < s_w; x++)
				d[x] = p;
		}
	} else {   // (horizontal gradient: as MAME, one pen, the rows offset twice by the clip's left: its first columns kept)
		const uint16_t p = gradient_pen(pen_idx + s_x0);
		for (int y = ry0; y <= ry1; y++) {
			uint16_t *d = rbx_video.bitmap + y * pitch;
			for (int x = s_x0; x < s_w; x++)
				d[x] = p;
		}
	}
}

// the frame in bands of rows, shared with the threads that help the render thread (render_help): each band draws the
// back colour and the four layers clipped to its rows, so the result is the one drawing's
#define BANDS 4
static struct { int next, left, active, helpers; } s_bj;

static void draw_band(int b)
{
	const snap_t *s = s_snap;
	static const int order[NUM_LAYERS] = { 3, 2, 0, 1 };
	const int r0 = s_h * b / BANDS, r1 = s_h * (b + 1) / BANDS - 1;
	fill_backcolor(s->k55555[0] << 9, s->k55555[1], r0, r1);
	for (int i = 0; i < NUM_LAYERS; i++) {
		const int layer = order[i];
		if (s->k55555[K55_INPUT_ENABLES] & (1 << layer))   // (K55_INP_VRAM_A << layer)
			tilemap_draw(&s->k, layer, s_m == M_SLOT, s_y0 + r0, s_y0 + r1);
	}
}

static int take_bands(int one)
{
	int n = 0;
	for (;;) {
		const int j = __atomic_fetch_add(&s_bj.next, 1, __ATOMIC_ACQ_REL);
		if (j >= BANDS)
			return n;
		draw_band(j);
		__atomic_sub_fetch(&s_bj.left, 1, __ATOMIC_ACQ_REL);
		n++;
		if (one)
			return n;
	}
}

static int konmedal68k_help(int helper)
{
	if (!__atomic_load_n(&s_bj.active, __ATOMIC_SEQ_CST))
		return 0;
	__atomic_add_fetch(&s_bj.helpers, 1, __ATOMIC_SEQ_CST);
	int did = 0;
	if (__atomic_load_n(&s_bj.active, __ATOMIC_SEQ_CST))
		did = take_bands(helper & 0x100) > 0;
	__atomic_sub_fetch(&s_bj.helpers, 1, __ATOMIC_SEQ_CST);
	return did;
}

static void konmedal68k_render(void)
{
	const snap_t *s = s_snap;
	memcpy(rbx_video.palette, s->pens, (size_t)s_pal_n * sizeof(uint32_t));
	rbx_video.palette[PEN_BLACK] = 0;
	rbx_video.palette_used = s_pal_n + 2;
	s_bj.next = 0;
	s_bj.left = BANDS;
	__atomic_store_n(&s_bj.active, 1, __ATOMIC_SEQ_CST);
	take_bands(0);
	while (__atomic_load_n(&s_bj.left, __ATOMIC_ACQUIRE)) {
	}
	__atomic_store_n(&s_bj.active, 0, __ATOMIC_SEQ_CST);
	while (__atomic_load_n(&s_bj.helpers, __ATOMIC_SEQ_CST)) {   // (none may still take a band of the next frame)
	}
}

static void video_capture(void)
{
	rbx_video_wait_idle();
	s_snap->k = s_k;
	memcpy(s_snap->pens, s_pens, (size_t)s_pal_n * sizeof(uint32_t));
	memcpy(s_snap->k55555, b.k55555, sizeof(b.k55555));
	rbx_video_frame_done();
}

// ---- init / frame ----
static void konmedal68k_exit(void);
static void konmedal68k_reset(void);

static int decode_tiles(void)
{
	// (charlayout4dj: planes 8*3, 8*1, 8*2, 8*0, 32 bytes a tile)
	s_ntiles = s_k.rom_size / 32;
	s_tiles_raw = (uint8_t *)malloc((size_t)s_ntiles * 64 + 63);
	if (!s_tiles_raw)
		return -1;
	s_tiles = (uint8_t *)(((uintptr_t)s_tiles_raw + 63) & ~(uintptr_t)63);
	const uint8_t *src = s_k.rom;
	uint8_t *d = s_tiles;
	for (uint32_t c = 0; c < s_ntiles; c++)
		for (int y = 0; y < 8; y++, src += 4)
			for (int x = 7; x >= 0; x--)
				*d++ = (uint8_t)(((src[3] >> x) & 1) << 3 | ((src[1] >> x) & 1) << 2 | ((src[2] >> x) & 1) << 1 | ((src[0] >> x) & 1));
	return 0;
}

static int konmedal68k_init(const rbx_game *g)
{
	uint32_t tile_size, ysize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	const uint8_t *tiles = rbx_region("k056832", &tile_size);
	const uint8_t *ymz = rbx_region("ymz", &ysize);
	if (!s_rom || !tiles || !ymz) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	static const char *const names[] = { "kzaurus", "koropens", "pwrchanc", "spcpokan", "gs662", "slot" };
	s_m = -1;
	for (int i = 0; i < 6; i++)
		if (!strcmp(g->machine, names[i]))
			s_m = i;
	if (s_m < 0) {
		rbx_fail("%s: unknown machine", g->set);
		return -1;
	}
	memset(&b, 0, sizeof(b));
	memset(s_pens, 0, sizeof(s_pens));
	b.hop_due = -1;
	s_ymz_n = 0;
	s_scr_w = 512;
	s_scr_h = s_m == M_SLOT ? 512 : 256;
	s_lines = s_scr_h;
	s_vbl = s_m == M_SLOT ? 480 : 240;
	s_x0 = s_m == M_GS662 ? 0 : 40;
	s_w = s_m == M_SLOT ? 320 : 360;
	s_y0 = 16;
	s_h = s_m == M_SLOT ? 480 : 224;
	s_pal_n = s_m == M_GS662 ? 0x4000 : 0x1000;   // (0x10000 / 0x4000 bytes of xBGR_888; the slots' 4096 xBGR_444)
	k056832_init(&s_k, tiles, tile_size, K056832_BPP_4, 1, 0);   // (K056832_BPP_4dj: decoded here)
	static const int offs[2][4][2] = { { { -4, -8 }, { 12, 8 }, { 6, -8 }, { 6, -8 } },       // (video_start)
	                                   { { 0, -11 }, { 2, -11 }, { 4, -11 }, { 6, -11 } } };
	for (int l = 0; l < 4; l++)
		k056832_set_layer_offs(&s_k, l, offs[s_m == M_SLOT][l][0], offs[s_m == M_SLOT][l][1]);
	m68k_init();
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0x000000, 0x07ffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x400000, 0x403fff, b.ram, sizeof(b.ram), 1, 1);
	if (s_m == M_SLOT)
		m68k_bus_map(0xb04000, 0xb0ffff, b.extra, sizeof(b.extra), 1, 1);
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
	ymz280b_init(&s_ymz, YMZ_CLOCK, ymz, ysize, NULL, NULL);
	const uint32_t rate = ymz280b_rate(&s_ymz);
	s_ymz_cap = (int)(rate / 50) + 64;
	s_ymz_l = (int32_t *)calloc((size_t)s_ymz_cap, sizeof(int32_t));
	s_ymz_r = (int32_t *)calloc((size_t)s_ymz_cap, sizeof(int32_t));
	s_snap = (snap_t *)calloc(1, sizeof(snap_t));
	s_port_in0 = rbx_port_find("IN0");
	s_port_in1 = rbx_port_find("IN1");
	s_port_dsw = rbx_port_find("DSW");
	if (!s_ymz_l || !s_ymz_r || !s_snap || rbx_audio_setup_stereo(rate) || rbx_video_setup(s_w, s_h, s_pal_n + 2) || decode_tiles()) {
		rbx_fail("%s: out of memory", g->set);
		konmedal68k_exit();
		return -1;
	}
	rbx_audio.frame_mhz = FRAME_MHZ;
	s_nv_path[0] = 0;
	{   // (the NVRAM, kept between runs)
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			FILE *f = fopen(s_nv_path, "rb");
			if (f) {
				if (fread(b.ram, 1, sizeof(b.ram), f) != sizeof(b.ram))
					memset(b.ram, 0, sizeof(b.ram));
				fclose(f);
			}
		}
	}
	konmedal68k_reset();
	return 0;
}

static void konmedal68k_exit(void)
{
	if (s_nv_path[0]) {
		FILE *f = fopen(s_nv_path, "wb");
		if (f) {
			fwrite(b.ram, 1, sizeof(b.ram), f);
			fclose(f);
		}
		s_nv_path[0] = 0;
	}
	free(s_tiles_raw);
	free(s_snap);
	free(s_ymz_l);
	free(s_ymz_r);
	s_tiles_raw = s_tiles = NULL;
	s_snap = NULL;
	s_ymz_l = s_ymz_r = NULL;
}

static void konmedal68k_reset(void)
{
	b.control = b.control2 = 0;
	b.cyc_end = b.cycles_done = 0;
	b.cyc_frac = b.snd_frac = 0;
	b.hop_due = -1;
	b.hop_status = b.hop_power = 0;
	s_ymz_n = 0;
	ymz280b_reset(&s_ymz);
	m68k_bus_irq_reset();
	m68k_pulse_reset();
}

static void konmedal68k_frame(void)
{
	const uint32_t rate = ymz280b_rate(&s_ymz);
	const int capture_line = s_y0 + s_h;   // (the screen's update at the vblank's start: the state before the IRQs act)
	for (int line = 0; line < s_lines; line++) {
		if (line == capture_line)
			video_capture();
		if (line == s_vbl && (b.control & 0x08))   // (scanline)
			m68k_bus_set_line(3, 1);
		if (line == s_vbl + 15 && (b.control & 0x10))
			m68k_bus_set_line(4, 1);
		hopper_tick();
		b.cyc_end += rbx_line_cycles(&b.cyc_frac, MAIN_CLOCK, FRAME_MHZ, s_lines);
		s_ymz_line0 = (uint32_t)s_ymz_n;
		s_ymz_line_n = (uint32_t)rbx_line_cycles(&b.snd_frac, rate, FRAME_MHZ, s_lines);
		const int64_t run = b.cyc_end - b.cycles_done;
		b.line_cyc = (int32_t)run;
		if (run > 0)
			b.cycles_done += m68k_execute((int)run);
		ymz_to((int)(s_ymz_line0 + s_ymz_line_n));
	}
	for (int i = 0; i < s_ymz_n; i++)   // (the speaker's routes at 0.75)
		rbx_audio_put_stereo(s_ymz_l[i] * 3 / 4, s_ymz_r[i] * 3 / 4);
	memset(s_ymz_l, 0, sizeof(int32_t) * (size_t)s_ymz_n);
	memset(s_ymz_r, 0, sizeof(int32_t) * (size_t)s_ymz_n);
	s_ymz_n = 0;
}

static int konmedal68k_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	int rc = 0;
	if ((uint32_t)m68k_context_size() > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	rc |= rbx_state_io(ctx, "konmedal68k_cpu", cpu, (uint32_t)m68k_context_size());
	rc |= rbx_state_io(ctx, "konmedal68k_bus", &m68k_bus.irq_pending, sizeof(uint32_t) * 2);
	rc |= rbx_state_io(ctx, "konmedal68k", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= k056832_state(&s_k, ctx, load);
	rc |= rbx_state_io(ctx, "konmedal68k_ymz", &s_ymz, (uint32_t)YMZ280B_STATE_SIZE);
	if (!rc && load) {
		m68k_set_context(cpu);
		for (int i = 0; i < s_pal_n; i++)
			pen_update(i);
	}
	return rc ? -1 : 0;
}

static int konmedal68k_save(void *ctx) { return konmedal68k_state(ctx, 0); }
static int konmedal68k_load(void *ctx) { return konmedal68k_state(ctx, 1); }
static int konmedal68k_supported(const char *machine)
{
	return !strcmp(machine, "kzaurus") || !strcmp(machine, "koropens") || !strcmp(machine, "pwrchanc") ||
	       !strcmp(machine, "spcpokan") || !strcmp(machine, "gs662") || !strcmp(machine, "slot");
}

const rbx_driver rbx_drv_konmedal68k = { "Konami Medal 68000", konmedal68k_init, konmedal68k_exit, konmedal68k_reset,
                                         konmedal68k_frame, konmedal68k_save, konmedal68k_load, konmedal68k_supported,
                                         konmedal68k_render, konmedal68k_help };
