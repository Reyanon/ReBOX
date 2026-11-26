/*  Seta 2 ("Newer Seta Hardware"): a TMP68301 (68HC000 with its interrupt controller, timers and ports), the X1-020 /
    DX-101 sprite generator (every layer is a sprite, the "floating tilemaps" too; zoom, shadows, raster interrupts) and
    the X1-010 with its 8 sample banks. Ported from MAME src/mame/seta/seta2.cpp (BSD-3-Clause, copyright-holders:
    Luca Elia, David Haywood), src/devices/video/x1_020_dx_101.cpp (BSD-3-Clause, copyright-holders: Luca Elia, David
    Haywood) and src/devices/machine/ticket.cpp (BSD-3-Clause, copyright-holders: Aaron Giles).
    Guardians, Gundam EX Revue, the My Angel quizzes, Puzzle De Bowling, Penguin Brothers, the Sammy gun games and the
    medal games. Not run: Star Audition (camera, flash), Namco Stars / Balance Try (OKI M9810, sub CPU), as MAME.
    The sprites are drawn as MAME draws them, a line at a time from the list of the line's moment: at each raster
    interrupt the lines up to it (MAME's partial updates), the rest at the vblank, into a frame the render side copies.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../machine/eepromser.h"
#include "../../machine/tmp68301.h"
#include "../../sound/x1_010.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern void (*rbx_m68k_int_hook)(int level);   // (m68kcpu.h)

#define LINES 256                    // (set_size(0x200, 0x100), set_refresh_hz(60))
#define FRAME_MHZ 60000u
#define PENS 0x8000
#define POS_FRAME (LINES * 512)      // (a frame's pixel positions: 0x200 a line)
#define TICKET_FRAMES 12             // (TICKET_DISPENSER / HOPPER: 200 ms)

enum { K_GUNDAMEX, K_GRDIANS, K_MJ4SIMAI, K_MYANGEL, K_MYANGEL2, K_PZLBOWL, K_PENBROS, K_ABLASTB, K_REELQUAK, K_SAMSHOOT,
       K_TELPACFL };

typedef struct {
	const char *name;
	int kind;
	uint32_t clock;     // the CPU's and the X1-010's
	uint32_t rom_end;   // the program ROM at 0 .. rom_end
	uint32_t video;     // the video map (sprites, palette, RAM, registers); 0: penbros' layout
	uint32_t x1snd;     // the X1-010's registers
	uint32_t banks;     // sound_bank_w (8 bytes on the low lane); 0: none
	int vis_w, vis_h;
} machine_t;

#define CLK_50M3 16666666u      // (XTAL(50'000'000) / 3)
#define CLK_32M2 16265235u      // (XTAL(32'530'470) / 2: the P0-113A boards)

static const machine_t s_machines[] = {
	{ "gundamex", K_GUNDAMEX, CLK_32M2, 0x1fffff, 0xc00000, 0xb00000, 0xe00010, 0x180, 0xe0 },
	{ "grdians", K_GRDIANS, CLK_50M3, 0x1fffff, 0xc00000, 0xb00000, 0xe00010, 0x130, 0xe8 },
	{ "grdiansa", K_GRDIANS, CLK_32M2, 0x1fffff, 0xc00000, 0xb00000, 0xe00010, 0x130, 0xe8 },
	{ "mj4simai", K_MJ4SIMAI, CLK_50M3, 0x1fffff, 0xc00000, 0xb00000, 0x600300, 0x180, 0xf0 },
	{ "myangel", K_MYANGEL, CLK_50M3, 0x1fffff, 0xc00000, 0xb00000, 0x700310, 0x178, 0xf0 },
	{ "myangel2", K_MYANGEL2, CLK_50M3, 0x1fffff, 0xd00000, 0xb00000, 0x600300, 0x178, 0xf0 },
	{ "pzlbowl", K_PZLBOWL, CLK_50M3, 0x0fffff, 0x800000, 0x900000, 0x400300, 0x180, 0xf0 },
	{ "penbros", K_PENBROS, CLK_50M3, 0x0fffff, 0, 0xa00000, 0x500300, 0x140, 0xe0 },
	{ "ablastb", K_ABLASTB, 16000000u, 0x0fffff, 0, 0xa00000, 0, 0x140, 0xe0 },
	{ "reelquak", K_REELQUAK, CLK_50M3, 0x0fffff, 0xc00000, 0xb00000, 0x400300, 0x140, 0xf0 },
	{ "samshoot", K_SAMSHOOT, CLK_50M3, 0x1fffff, 0x800000, 0x900000, 0x400300, 0x140, 0xf0 },
	{ "telpacfl", K_TELPACFL, CLK_50M3, 0x0fffff, 0xb00000, 0x900000, 0xe00010, 0x180, 0xf0 },
};

// the ports the maps read
enum { P_P1, P_P2, P_SYSTEM, P_DSW1, P_DSW2, P_IN0, P_IN1, P_TICKET, P_BUTTONS, P_TRIGGER, P_PUMP, P_COIN,
       P_SERVICE, P_UNKNOWN, P_KNOB, P_KEY0, P_COUNT = P_KEY0 + 10 };
static const char *const s_port_tags[P_KEY0] = { "P1", "P2", "SYSTEM", "DSW1", "DSW2", "IN0", "IN1", "TICKET",
	"BUTTONS", "TRIGGER", "PUMP", "COIN", "SERVICE", "UNKNOWN", "KNOB" };

static const machine_t *s_m;
static int s_tmp_on;                 // (a TMP68301: all but the bootleg's plain 68000)
static int s_gun2;                   // (a second light gun: wschamp, trophyh)
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t *s_gfx;               // the "video" region, decoded in place: 8x8 tiles of 64 bytes, a byte a pixel
static uint32_t s_gfx_tiles;
static const uint8_t *s_x1rom;
static uint32_t s_x1rom_size;
static uint8_t *s_ram;               // 0x200000-0x23ffff
static uint8_t *s_ram3;              // 0x300000-0x30ffff (RAM, the NVRAM)
static uint8_t *s_vram5;             // the video map's 0x50000-0x5ffff
static uint16_t *s_spr;              // sprite RAM, 0x20000 words (host order)
static uint16_t *s_pal;              // palette RAM, 0x8000 words
static uint16_t s_vregs[0x20];
static uint16_t s_priv[0x800];       // (m_private_spriteram)
// the drawing's own copies of what it reads (sprite RAM, video registers, private list), one for each worker, brought
// up to date from the queue below in the order the CPU made its writes: the bands within the frame are drawn on the
// two workers, every other band each, while the CPU runs on (grdians' raster interrupts every two lines write a word
// of sprite RAM between the bands). The same pixels: each band sees the state of its moment
typedef struct {
	uint16_t *spr;
	uint16_t vregs[0x20];
	uint16_t priv[0x800];
	volatile uint32_t t;   // (the queue's events applied)
} drawstate_t;
static drawstate_t s_rep[2];
enum { EV_SPRW = 0x00000000u, EV_VREG = 0x01000000u, EV_SPRBUF = 0x02000000u, EV_BAND = 0x03000000u, EV_KIND = 0xff000000u };
#define EVQ (1u << 15)               // events (two words each)
static uint32_t *s_evq;
static volatile uint32_t s_evh;   // (written by the CPU's thread)
static uint16_t *s_fb;               // the frame being drawn (vis_w x vis_h)
static uint16_t *s_snap_fb;          // the last frame, for render
static uint16_t *s_snap_pal;
static int s_pal_dirty;
static int s_drawn;                  // lines of s_fb drawn this frame
static int s_port[P_COUNT];
static tmp68301_t s_tmp;
static x1_010_t s_snd;
static eepser_t s_eeprom;
static int s_eeprom_on;
static char s_nv_path[512];
static uint32_t s_nv_size;
static int s_in_exec;
static uint64_t s_cpu_frac, s_snd_frac, s_frame_frac;

static struct {
	uint64_t cyc;                    // the CPU's cycles since power on
	uint64_t fstart;                 // the frame's first cycle
	uint64_t raster_at, vbl_end_at;  // the raster interrupt's and the vblank's end (0: none)
	uint32_t fcyc;                   // the frame's cycles
	uint32_t rng;
	uint16_t raster_en, raster_pos;  // (m_rasterenabled, m_rasterposition)
	uint8_t bank[8];
	uint8_t keyrow;
	uint8_t tk_power, tk_status;
	uint8_t pad;
	int tk_count;
	int gun[2][2], knob;
	uint32_t frame;
} s_st;

#define COMBINE(d, v, m) ((d) = (uint16_t)(((d) & ~(m)) | ((v) & (m))))

// ---- the clock ----
static uint64_t cpu_now(void) { return s_st.cyc + (s_in_exec ? (uint64_t)m68k_cycles_run() : 0); }

static void end_slice(void)
{
	if (s_in_exec)
		m68k_end_timeslice();
}

// the beam's position (0x200 a line) at a cycle, within its frame
static uint32_t beam_pos(uint64_t c, uint64_t *frame_base)
{
	const uint64_t d = c - s_st.fstart;
	const uint64_t p = d * POS_FRAME / s_st.fcyc;
	*frame_base = s_st.fstart + (p / POS_FRAME) * s_st.fcyc;
	return (uint32_t)(p % POS_FRAME);
}

// ---- the X1-020 / DX-101 ----
typedef struct { int gfx_mask, gfx_shift, shadow; } drawmode;
static const drawmode BPP_MASK_TABLE[8] = {
	{ 0xff, 0, 4 }, { 0x30, 4, 2 }, { 0x07, 0, 3 }, { 0xff, 0, 0 },
	{ 0x0f, 0, 3 }, { 0xf0, 4, 4 }, { 0x3f, 0, 5 }, { 0xff, 0, 8 },
};

static inline const uint8_t *tile_data(uint32_t code)   // (gfx(0)->get_data(m_realtilenumber[code]))
{
	return s_gfx + (size_t)rbx_wrap(code & 0x7ffff, s_gfx_tiles) * 64;
}

static inline void draw_pixel(uint16_t *dest, const uint8_t *source, uint32_t realcolor, int sx, int maxx, int column,
                              int shadow, int gfx_mask, int gfx_shift, int opaque)
{
	if (sx >= 0 && sx < maxx) {
		const int realsx = sx >> 16;
		const uint8_t pen = (uint8_t)((source[column] & gfx_mask) >> gfx_shift);
		if (pen || opaque) {
			if (!shadow)
				dest[realsx] = (uint16_t)((realcolor + pen) & 0x7fff);
			else {
				const int pen_shift = 15 - shadow, pen_mask = (1 << pen_shift) - 1;
				dest[realsx] = (uint16_t)(((dest[realsx] & pen_mask) | (pen << pen_shift)) & 0x7fff);
			}
		}
	}
}

static void drawgfx_line(uint16_t *dest, int width, int which_gfx, const uint8_t *addr, uint32_t realcolor, int flipx,
                         int flipy, int base_sx, uint32_t xzoom, int use_shadow, int line, int opaque)
{
	const drawmode *dm = &BPP_MASK_TABLE[which_gfx];
	const int shadow = use_shadow ? dm->shadow : 0, gfx_mask = dm->gfx_mask, gfx_shift = dm->gfx_shift;
	const int maxx = width << 16;
	if (flipy)
		line = 7 - line;
	const uint8_t *source = addr + line * 8;
	if (xzoom == 0x10000) {   // (no zoom: MAME's enlarge loop, a column a pixel)
		const int px0 = base_sx >> 16;
		if (!shadow && px0 >= 0 && px0 + 8 <= width) {   // (the 8 columns on the screen: the same pixels, fewer tests)
			uint64_t row;
			memcpy(&row, source, 8);
			if (!opaque && !(row & ((uint64_t)(uint8_t)gfx_mask * 0x0101010101010101ull)))
				return;   // (a transparent line of the tile)
			uint16_t *d = dest + px0;
			const int step = flipx ? -1 : 1;
			const uint8_t *s = flipx ? source + 7 : source;
			if (opaque)
				for (int i = 0; i < 8; i++, s += step)
					d[i] = (uint16_t)((realcolor + (uint32_t)((*s & gfx_mask) >> gfx_shift)) & 0x7fff);
			else
				for (int i = 0; i < 8; i++, s += step) {
					const uint32_t pen = (uint32_t)((*s & gfx_mask) >> gfx_shift);
					if (pen)
						d[i] = (uint16_t)((realcolor + pen) & 0x7fff);
				}
			return;
		}
		for (int i = 0; i < 8; i++) {
			const int px = px0 + i;
			if (px < 0 || px >= width)
				continue;
			const uint8_t pen = (uint8_t)((source[flipx ? 7 - i : i] & gfx_mask) >> gfx_shift);
			if (pen || opaque) {
				if (!shadow)
					dest[px] = (uint16_t)((realcolor + pen) & 0x7fff);
				else {
					const int pen_shift = 15 - shadow, pen_mask = (1 << pen_shift) - 1;
					dest[px] = (uint16_t)(((dest[px] & pen_mask) | (pen << pen_shift)) & 0x7fff);
				}
			}
		}
		return;
	}
	if (xzoom < 0x10000) {   // shrink
		const int x0 = flipx ? (int)((uint32_t)base_sx + 8 * xzoom - xzoom) : base_sx;
		const int x1 = flipx ? (int)((uint32_t)base_sx - xzoom) : (int)((uint32_t)x0 + 8 * xzoom);
		const int dx = flipx ? -(int)xzoom : (int)xzoom;
		for (int sx = x0, column = 0; sx != x1; sx += dx, column++)
			draw_pixel(dest, source, realcolor, sx, maxx, column, shadow, gfx_mask, gfx_shift, opaque);
	} else {   // enlarge
		const int x0 = base_sx;
		const int x1 = (int)((uint32_t)x0 + 8 * xzoom);
		const int column_inc = flipx ? -1 : 1;
		int column = flipx ? 7 : 0;
		uint32_t countx = 0;
		if ((int64_t)x1 + 0x10000 <= INT32_MAX && x1 > x0) {   // (only the columns on the screen: the same pixels)
			const int64_t n = ((int64_t)x1 - x0 + 0xffff) >> 16;
			const int64_t k0 = x0 < 0 ? ((-(int64_t)x0 + 0xffff) >> 16) : 0;
			int64_t k1 = (((int64_t)maxx - x0) + 0xffff) >> 16;
			if (k1 > n)
				k1 = n;
			if (k0 >= k1)
				return;
			if (k0) {   // (each step adds 0x10000 and takes xzoom away at most once)
				const uint64_t t = (uint64_t)k0 << 16;
				column += column_inc * (int)(t / xzoom);
				countx = (uint32_t)(t % xzoom);
			}
			for (int sx = x0 + (int)(k0 << 16), k = (int)k0; k < k1; sx += 0x10000, k++) {
				draw_pixel(dest, source, realcolor, sx, maxx, column, shadow, gfx_mask, gfx_shift, opaque);
				countx += 0x10000;
				if (countx >= xzoom) {
					column += column_inc;
					countx -= xzoom;
				}
			}
			return;
		}
		for (int sx = x0; sx < x1; sx += 0x10000) {
			draw_pixel(dest, source, realcolor, sx, maxx, column, shadow, gfx_mask, gfx_shift, opaque);
			countx += 0x10000;
			if (countx >= xzoom) {
				column += column_inc;
				countx -= xzoom;
			}
		}
	}
}

static inline void get_tile(const uint16_t *spr, int is_16x16, int x, int y, int page, uint32_t *code, int *flipx, int *flipy, int *color)
{
	const int xtile = x >> (is_16x16 ? 4 : 3);
	int ytile = y >> (is_16x16 ? 4 : 3);
	ytile ^= 0x1f;   // (the tilemap in RAM is flipped)
	const uint16_t *s3 = &spr[2 * ((page * 0x2000 / 4) + ((ytile & 0x1f) << 6) + (xtile & 0x03f))];
	const int attr = s3[0];
	uint32_t c = s3[1] + ((uint32_t)(attr & 0x0007) << 16);
	*flipx = (attr >> 4) & 1;
	*flipy = (attr >> 3) & 1;
	*color = (attr & 0xffe0) >> 5;
	if (is_16x16) {
		c &= ~3u;
		if (!*flipx) {
			if (x & 8)
				c += 1;
		} else if (!(x & 8))
			c += 1;
		if (!*flipy) {
			if (y & 8)
				c += 2;
		} else if (!(y & 8))
			c += 2;
	}
	*code = c;
}

static inline int covers(int u, int firstline, int endline)
{
	if (endline >= firstline)
		return u >= firstline && u <= endline;
	return !(u > endline && u < firstline);   // (a sprite across 0)
}

// the lines of y0..y1 a sprite's firstline..endline cover (covers), as runs r[0..1], r[2..3] ... The zoomed scanline of
// a line (map) rises in at most MAXSEG steady stretches (seg: their first and last lines; nozoom or no y zoom: the line
// itself, one stretch), each searched for the lines in the range. -1: too many stretches, the whole band (r[0..1])
// with each line tested by the caller
#define MAXSEG 8
typedef struct { int n, a[MAXSEG], b[MAXSEG]; } segs_t;
static inline int first_at_least(const int *map, int a, int b, int v)   // (the first line of a..b whose map >= v)
{
	int lo = a, hi = b + 1;
	while (lo < hi) {
		const int m = (lo + hi) >> 1;
		if (map[m] >= v) hi = m; else lo = m + 1;
	}
	return lo;
}
static inline int last_at_most(const int *map, int a, int b, int v)   // (the last line of a..b whose map <= v)
{
	return first_at_least(map, a, b, v + 1) - 1;
}
static int band_runs(const int *map, const segs_t *sg, int y0, int y1, int firstline, int endline, int *r)
{
	if (sg->n < 0) {
		r[0] = y0;
		r[1] = y1;
		return -1;
	}
	int n = 0;
	for (int k = 0; k < sg->n; k++) {
		const int a = sg->a[k], b = sg->b[k];
		if (endline >= firstline) {
			const int lo = first_at_least(map, a, b, firstline), hi = last_at_most(map, a, b, endline);
			if (lo <= hi) { r[n++] = lo; r[n++] = hi; }
		} else {   // (a sprite across 0: up to endline, and from firstline)
			const int hi = last_at_most(map, a, b, endline), lo = first_at_least(map, a, b, firstline);
			if (hi >= a) { r[n++] = a; r[n++] = hi; }
			if (lo <= b) { r[n++] = lo; r[n++] = b; }
		}
	}
	return n / 2;
}

// draw_sprites for the screen lines y0..y1 of s_fb: MAME's draw_sprites_line for each line, with the list walked once
// (each line is drawn from the same list in the same order: the same pixels)
static void draw_lines(const drawstate_t *R, int y0, int y1)
{
	const int w = s_m->vis_w;
	for (int y = y0; y <= y1; y++)   // (bitmap.fill(palette().pen(0)))
		memset(s_fb + (size_t)y * w, 0, (size_t)w * 2);
	const uint16_t *const s_vregs = R->vregs;   // (the drawing's copies)
	const uint16_t *const s_spr = R->spr, *const s_priv = R->priv;
	if (s_vregs[0x30 / 2] & 1)   // (blank screen)
		return;

	uint32_t yoffset = ((uint32_t)s_vregs[0x1a / 2] << 16) | s_vregs[0x18 / 2];
	yoffset &= 0x07ffffff;
	yoffset = 0x07ffffff - yoffset;
	uint32_t yzoom = ((uint32_t)s_vregs[0x1e / 2] << 16) | s_vregs[0x1c / 2];
	yzoom &= 0x07ffffff;
	int yzoominverted = 0;
	if (yzoom & 0x04000000) {
		yzoom = 0x8000000 - yzoom;
		yzoominverted = 1;
	}
	int xoffset = (int)((((uint32_t)s_vregs[0x12 / 2] << 16) | s_vregs[0x10 / 2]) & 0x07ffffff);
	if (xoffset & 0x04000000)
		xoffset -= 0x08000000;
	uint32_t xzoom = ((uint32_t)s_vregs[0x16 / 2] << 16) | s_vregs[0x14 / 2];
	if (xzoom & 0x04000000)
		xzoom = 0x8000000 - xzoom;
	if (!xzoom)
		return;
	const uint32_t inc2 = (uint32_t)(0x100000000ULL / xzoom);

	int map[LINES];   // (the zoomed scanline of each screen line)
	for (int y = y0; y <= y1; y++) {
		int yy;
		if (!yzoominverted)
			yy = y;
		else {
			uint32_t v = (uint32_t)y * yzoom + yoffset;
			v &= 0x07ffffff;
			yy = (int)(v >> 16);
			if (yy & 0x400)
				yy -= 0x800;
		}
		map[y] = yy;
	}
	int ident[LINES];   // (nozoom sprites: the line itself)
	segs_t sg_zoom, sg_id = { 1, { y0 }, { y1 } };
	for (int y = y0; y <= y1; y++)
		ident[y] = y;
	sg_zoom.n = 0;
	for (int y = y0; y <= y1 && sg_zoom.n >= 0; y++)
		if (y == y0 || map[y] < map[y - 1]) {
			if (sg_zoom.n == MAXSEG)
				sg_zoom.n = -1;
			else {
				if (sg_zoom.n)
					sg_zoom.b[sg_zoom.n - 1] = y - 1;
				sg_zoom.a[sg_zoom.n++] = y;
			}
		}
	if (sg_zoom.n > 0)
		sg_zoom.b[sg_zoom.n - 1] = y1;

	for (const uint16_t *s1 = s_priv; s1 < s_priv + 0x800; s1 += 4) {
		int num = s1[0];
		int xoffs = s1[1], yoffs = s1[2];
		const int sprite = s1[3];
		const uint16_t *s2 = &s_spr[(sprite & 0x7fff) * 4];
		const uint16_t *const end = s_spr + 0x20000;
		const int global_sizex = xoffs & 0xfc00, global_sizey = yoffs & 0xfc00;
		const int nozoom = (num >> 14) & 1, opaque = (num >> 13) & 1, use_global_size = (num >> 12) & 1;
		int use_shadow = (num >> 11) & 1;
		const int which_gfx = (num & 0x0700) >> 8;
		xoffs &= 0x3ff;
		yoffs &= 0x3ff;
		if (yoffs & 0x200)
			yoffs -= 0x400;
		const int global_xoffset = nozoom ? 0x80 : 0, global_yoffset = nozoom ? -0x90 : 0;
		int usedxoffset;
		uint32_t usedxzoom;
		if (nozoom) {
			use_shadow = 0;
			usedxzoom = 0x10000;
			usedxoffset = 0;
		} else {
			usedxzoom = inc2;
			usedxoffset = xoffset;
		}
		num = (num & 0x00ff) + 1;
		if ((sprite & 0x7fff) < 0x3000 / 2 / 4) {
			for (; num > 0; num--, s2 += 4) {
				if (s2 >= end)
					break;
				if (sprite & 0x8000) {   // a "floating tilemap" sprite
					int sy = s2[1] & 0x3ff;
					sy += global_yoffset;
					sy &= 0x3ff;
					if (sy & 0x200)
						sy -= 0x400;
					const int local_sizey = s2[1] & 0xfc00;
					int height = use_global_size ? global_sizey : local_sizey;
					height = ((height & 0xfc00) >> 10) + 1;
					int firstline = (sy + yoffs) & 0x3ff;
					if (firstline & 0x200)
						firstline -= 0x400;
					int endline = firstline + height * 0x10 - 1;
					if (endline & 0x200)
						endline -= 0x400;
					int sx = s2[0];
					const int local_sizex = sx & 0xfc00;
					sx &= 0x3ff;
					sx -= global_xoffset;
					int width = use_global_size ? global_sizex : local_sizex;
					width = (width & 0xfc00) >> 10;   // (no + 1: reelquak's reels)
					if (!width)
						continue;
					int firstcolumn = sx + xoffs;
					firstcolumn = (firstcolumn & 0x1ff) - (firstcolumn & 0x200);
					const int lastcolumn = firstcolumn + width * 0x10 - 1;
					if (firstcolumn > w - 1 || lastcolumn < 0)
						continue;
					int scrolly = s2[3] & 0x1ff;
					scrolly += global_yoffset;
					int scrollx = s2[2];
					const int is_16x16 = (scrollx >> 15) & 1, page = (scrollx & 0x7c00) >> 10;
					scrollx &= 0x3ff;
					const int dx = sx + (scrollx & 0x3ff) + xoffs + 0x10;
					// (the columns of the sprite: the same for each line, found once)
					uint8_t cx[0x80];
					int16_t cdst[0x80];
					int run[4 * MAXSEG];
					const int nr = band_runs(nozoom ? ident : map, nozoom ? &sg_id : &sg_zoom, y0, y1, firstline, endline, run);
					const int test = nr < 0, nrun = test ? 1 : nr;   // (test: each line tested)
					int ncol = -1;
					for (int q = 0; q < nrun && ncol < 0; q++)   // (only when a line of the band shows the sprite)
						for (int y = run[2 * q]; y <= run[2 * q + 1] && ncol < 0; y++)
							if (!test || covers(nozoom ? y : map[y], firstline, endline))
								ncol = 0;
					if (ncol < 0)
						continue;
					for (int x = 0; x < 0x80; x++) {
						const int px = (((dx + x * 8) + 0x10) & 0x3ff) - 0x10;
						int dst_x = px & 0x3ff;
						dst_x = (dst_x & 0x1ff) - (dst_x & 0x200);
						if (dst_x < firstcolumn - 8 || dst_x > lastcolumn)
							continue;
						cx[ncol] = (uint8_t)x;
						cdst[ncol++] = (int16_t)dst_x;
					}
					for (int q = 0; q < nrun; q++)
					for (int y = run[2 * q]; y <= run[2 * q + 1]; y++) {
						const int u = nozoom ? y : map[y];
						if (test && !covers(u, firstline, endline))
							continue;
						const int sourceline = (u - scrolly) & 0x1ff, tileline = sourceline & 0x07;
						uint16_t *dest = s_fb + (size_t)y * w;
						for (int k = 0; k < ncol; k++) {
							const int x = cx[k], dst_x = cdst[k];
							uint32_t code;
							int fx, fy, color;
							get_tile(s_spr, is_16x16, x * 8, sourceline, page, &code, &fx, &fy, &color);
							uint32_t realsx = (uint32_t)dst_x;
							realsx -= (uint32_t)(usedxoffset >> 16);
							realsx = realsx * usedxzoom;
							drawgfx_line(dest, w, which_gfx, tile_data(code), (uint32_t)color << 4, fx, fy, (int)realsx,
							             usedxzoom, use_shadow, tileline, opaque);
						}
					}
				} else {   // a "normal" sprite
					int sy = s2[1] & 0x1ff;
					if (sy & 0x100)
						sy -= 0x200;
					sy += global_yoffset;
					sy &= 0x3ff;
					int sizey = use_global_size ? global_sizey : s2[1] & 0xfc00;
					sizey = (1 << ((sizey & 0x0c00) >> 10)) - 1;
					int firstline = (sy + yoffs) & 0x3ff;
					int endline = (firstline + (sizey + 1) * 8) - 1;
					endline &= 0x3ff;
					if (firstline & 0x200)
						firstline -= 0x400;
					if (endline & 0x200)
						endline -= 0x400;
					const int attr = s2[2];
					int code = s2[3] + ((attr & 0x0007) << 16);
					const int flipx = (attr >> 4) & 1, flipy = (attr >> 3) & 1;
					int color = (attr & 0xffe0) >> 5;
					int sx = s2[0];
					int sizex = use_global_size ? global_sizex : sx;
					sizex = (1 << ((sizex & 0x0c00) >> 10)) - 1;
					sx += xoffs;
					sx = (sx & 0x1ff) - (sx & 0x200);
					sx -= global_xoffset;
					if (nozoom)
						color = 0x7ff;   // (grdians' map)
					code &= ~((sizex + 1) * (sizey + 1) - 1);
					const int code_inc = flipx ? -1 : 1;
					int run[4 * MAXSEG];
					const int nr = band_runs(nozoom ? ident : map, nozoom ? &sg_id : &sg_zoom, y0, y1, firstline, endline, run);
					const int test = nr < 0, nrun = test ? 1 : nr;
					for (int q = 0; q < nrun; q++)
					for (int y = run[2 * q]; y <= run[2 * q + 1]; y++) {
						const int u = nozoom ? y : map[y];
						if (test && !covers(u, firstline, endline))
							continue;
						int line = u - firstline;
						const int ty = line >> 3;
						line &= 0x7;
						int basecode = code + ((flipy ? (sizey - ty) : ty) * (sizex + 1)) + (flipx ? sizex : 0);
						uint16_t *dest = s_fb + (size_t)y * w;
						for (int x = 0; x <= sizex; x++) {
							uint32_t realsx = (uint32_t)(sx + x * 8);
							realsx -= (uint32_t)(usedxoffset >> 16);
							realsx = realsx * usedxzoom;
							drawgfx_line(dest, w, which_gfx, tile_data((uint32_t)basecode), (uint32_t)color << 4, flipx,
							             flipy, (int)realsx, usedxzoom, use_shadow, line, opaque);
							basecode += code_inc;
						}
					}
				}
			}
		}
		if (s1[0] & 0x8000)   // (the end of the list)
			break;
	}
}

// a band in up to three parts at once: the emulation thread and the driver's two worker threads (each line is drawn
// only from the state, which no one changes while they draw: the same pixels in any order)
static volatile int s_part_y[2][2], s_part_on[2];
static void part_job(int k)
{
	if (s_part_on[k]) {
		draw_lines(&s_rep[0], s_part_y[k][0], s_part_y[k][1]);
		s_part_on[k] = 0;
	}
}

static void draw_band(int y0, int y1)
{
	const int n = y1 - y0 + 1;
	if (n < 2) {
		draw_lines(&s_rep[0], y0, y1);
		return;
	}
	if (n == 2) {   // (raster interrupts every two lines: a line each)
		s_part_y[0][0] = s_part_y[0][1] = y1; s_part_on[0] = 1;
		rbx_aux_kick();
		draw_lines(&s_rep[0], y0, y0);
		rbx_aux_sync();
		return;
	}
	const int a = y0 + n / 3, b = y0 + 2 * n / 3;
	s_part_y[0][0] = a; s_part_y[0][1] = b - 1; s_part_on[0] = 1;
	s_part_y[1][0] = b; s_part_y[1][1] = y1; s_part_on[1] = 1;
	rbx_aux_kick();
	rbx_aux2_kick();
	draw_lines(&s_rep[0], y0, a - 1);
	rbx_aux_sync();
	rbx_aux2_sync();
}

// ---- the drawing's queue ----
static void sprite_buffer_on(uint16_t *s_spr, uint16_t *s_priv);
static void ev_push(uint32_t a, uint32_t b)
{
	const uint32_t h = s_evh;
	const uint32_t t0 = __atomic_load_n(&s_rep[0].t, __ATOMIC_ACQUIRE), t1 = __atomic_load_n(&s_rep[1].t, __ATOMIC_ACQUIRE);
	if (h - t0 >= EVQ || h - t1 >= EVQ) {   // (full: the drawing catches up)
		rbx_aux_sync();
		rbx_aux2_sync();
	}
	s_evq[(h % EVQ) * 2] = a;
	s_evq[(h % EVQ) * 2 + 1] = b;
	__atomic_store_n(&s_evh, h + 1, __ATOMIC_RELEASE);
}

// worker k's copies brought to the queue's end, its bands (every other one) drawn on the way
static void ev_drain(int k)
{
	drawstate_t *R = &s_rep[k];
	for (;;) {
		const uint32_t t = R->t, h = __atomic_load_n(&s_evh, __ATOMIC_ACQUIRE);
		if (t == h)
			return;
		for (uint32_t i = t; i != h; i++) {
			const uint32_t a = s_evq[(i % EVQ) * 2], b = s_evq[(i % EVQ) * 2 + 1];
			switch (a & EV_KIND) {
			case EV_SPRW: R->spr[a & 0x1ffff] = (uint16_t)b; break;
			case EV_VREG: R->vregs[a & 0x1f] = (uint16_t)b; break;
			case EV_SPRBUF: sprite_buffer_on(R->spr, R->priv); break;
			default:
				if ((int)(a & 1) == k)
					draw_lines(R, (int)(b & 0xffff), (int)(b >> 16));
				break;
			}
		}
		__atomic_store_n(&R->t, h, __ATOMIC_RELEASE);
	}
}
static void aux_job0(void)
{
	part_job(0);   // (a part of a band drawn at once: draw_band)
	ev_drain(0);
}
static void aux_job1(void)
{
	part_job(1);
	ev_drain(1);
}

// the drawing's copies the same as the CPU's state again (power on, a state loaded); the queue empty
static void draw_state_sync(void)
{
	rbx_aux_sync();
	rbx_aux2_sync();
	for (int k = 0; k < 2; k++) {
		memcpy(s_rep[k].spr, s_spr, 0x40000);
		memcpy(s_rep[k].vregs, s_vregs, sizeof(s_vregs));
		memcpy(s_rep[k].priv, s_priv, sizeof(s_priv));
	}
}

static uint32_t s_bandseq;   // (the bands within the frame so far: which worker draws the next)
// (screen.update_partial: the lines up to `line` not drawn yet) A band within the frame goes to the queue; the last,
// at vblank, is drawn at once in three parts after the queue (the frame is then complete)
static void update_to(int line)
{
	if (line >= s_m->vis_h)
		line = s_m->vis_h - 1;
	if (line < s_drawn)
		return;
	if (line < s_m->vis_h - 1) {
		ev_push(EV_BAND | (s_bandseq++ & 1), (uint32_t)s_drawn | (uint32_t)line << 16);
		rbx_aux_kick();
		rbx_aux2_kick();
	} else {
		rbx_aux_sync();
		rbx_aux2_sync();
		draw_band(s_drawn, line);
	}
	s_drawn = line + 1;
}

// the vregs 0x24 / 0x26 write: the list at 0x3000 copied to the private one, its sprites to 0x0000 (MAME's guess);
// on the CPU's state, and on the drawing's copies when the queue gets there
static void sprite_buffer_on(uint16_t *s_spr, uint16_t *s_priv)
{
	int current_sprite_entry = 0;
	for (int i = 0; i < 0x1000 / 2; i += 4) {
		uint16_t num = s_priv[i + 0] = s_spr[(0x3000 / 2) + i + 0];
		s_priv[i + 1] = s_spr[(0x3000 / 2) + i + 1];
		s_priv[i + 2] = s_spr[(0x3000 / 2) + i + 2];
		const int sprite = s_spr[(0x3000 / 2) + i + 3];
		s_priv[i + 3] = (uint16_t)(((current_sprite_entry / 4) & 0x7fff) | (sprite & 0x8000));
		const int list2addr = (sprite & 0x7fff) * 4;
		num &= 0xff;
		for (int j = 0; j <= num; j++)
			if (current_sprite_entry < 0x3000 / 2) {
				for (int k = 0; k < 4; k++)
					s_spr[current_sprite_entry + k] = s_spr[(list2addr + (j * 4) + k) & 0x1ffff];
				current_sprite_entry += 4;
			}
		if (s_priv[i + 0] & 0x8000) {   // (the end marker: its sprite is drawn, mj4simai's logo)
			if (sprite == 0x00)   // (penbros' dummy entry pointing at 0: made invalid, filtered when drawn)
				s_priv[i + 3] |= 0x4000;
			break;
		}
	}
}

static void sprite_buffer(void)
{
	sprite_buffer_on(s_spr, s_priv);
	ev_push(EV_SPRBUF, 0);
}

static void raster_schedule(void)   // (the 0x3c write: m_raster_timer->adjust(time_until_pos(vpos, hpos)))
{
	const uint64_t c = cpu_now();
	uint64_t base;
	const uint32_t p = beam_pos(c, &base);
	int hpos = 0;
	if (s_st.raster_pos == p / 512)
		hpos = (int)(p % 512) + 0x100;
	uint64_t target = (uint64_t)(s_st.raster_pos % LINES) * 512 + (uint64_t)hpos;
	while (target <= p)
		target += POS_FRAME;
	s_st.raster_at = base + (target * s_st.fcyc + POS_FRAME - 1) / POS_FRAME;
	end_slice();
}

static void vregs_w(uint32_t offset, uint16_t data, uint16_t mask)
{
	offset &= 0x1f;
	COMBINE(s_vregs[offset], data, mask);
	ev_push(EV_VREG | offset, s_vregs[offset]);
	switch (offset * 2) {
	case 0x24:
	case 0x26:
		if (data)
			sprite_buffer();
		break;
	case 0x3c:
		COMBINE(s_st.raster_en, data, mask);
		raster_schedule();
		break;
	case 0x3e:
		COMBINE(s_st.raster_pos, data, mask);
		break;
	default:
		break;
	}
}

static void raster_fire(void)   // (raster_timer_done)
{
	if (!s_tmp_on || !(s_st.raster_en & 1))
		return;
	uint64_t base;
	const int vpos = (int)(beam_pos(s_st.cyc, &base) / 512);
	update_to(vpos - 1);
	tmp68301_set_input(&s_tmp, 1, 1, 1);
}

// ---- the medal hoppers (ticket_dispenser_device) ----
static void ticket_motor(int state)
{
	if (state && !s_st.tk_power) {
		s_st.tk_count = TICKET_FRAMES;
		s_st.tk_power = 1;
		s_st.tk_status = 0;
	} else if (!state && s_st.tk_power) {
		if (s_m->kind != K_TELPACFL || !s_st.tk_status)   // (a HOPPER finishes the medal it is paying)
			s_st.tk_count = 0;
		s_st.tk_power = 0;
	}
}

static void ticket_frame(void)
{
	if (s_st.tk_count > 0 && --s_st.tk_count == 0) {
		if (s_st.tk_power) {
			s_st.tk_status ^= 1;
			s_st.tk_count = TICKET_FRAMES;
		} else if (s_m->kind == K_TELPACFL)
			s_st.tk_status ^= 1;
	}
	if (s_m->kind == K_REELQUAK)
		rbx_port_custom(s_port[P_TICKET], 0x0001, s_st.tk_status ? 1u : 0);
	else
		rbx_port_custom(s_port[P_P1], 0x0008, s_st.tk_status ? 0x0008u : 0);
}

// ---- the parallel port ----
static uint16_t par_r(void)
{
	if (s_m->kind == K_GUNDAMEX)   // (gundamex_eeprom_r)
		return (uint16_t)((eepser_do_read(&s_eeprom) & 1) << 3);
	if (s_m->kind == K_TELPACFL)   // (the knob)
		return (uint16_t)s_st.knob;
	return 0xffff;
}

static void par_w(uint16_t data)
{
	if (s_m->kind == K_GUNDAMEX) {   // (gundamex_eeprom_w)
		const int64_t t = (int64_t)cpu_now();
		eepser_clk_write(&s_eeprom, (data >> 1) & 1, t);
		eepser_di_write(&s_eeprom, data & 1);
		eepser_cs_write(&s_eeprom, (data >> 2) & 1, t);
	} else if (s_m->kind == K_REELQUAK)   // (reelquak_leds_w: the lamps, the ticket dispenser)
		ticket_motor((data >> 8) & 1);
}

static void tmp_level(int level) { m68k_set_irq((unsigned)level); }

static void tmp_int_hook(int level)   // (the CPU takes the interrupt: the vector the TMP68301 gives)
{
	(void)level;
	const uint8_t vec = tmp68301_ack(&s_tmp);
	uint32_t pc = m68k_read_memory_32((uint32_t)vec << 2);
	if (pc == 0)
		pc = m68k_read_memory_32(15 << 2);
	m68k_set_reg(M68K_REG_PC, pc);
	m68k_set_irq(s_tmp.level);   // (the core cleared its level when it took this one)
}

// ---- memory ----
static uint16_t port(int p) { return s_port[p] >= 0 ? (uint16_t)rbx_port_read(s_port[p]) : 0xffff; }

static uint16_t mj_key_r(int which)   // (mj4simai_key_r)
{
	uint16_t result = 0x3f;
	for (int i = 0; i < 5; i++)
		if ((s_st.keyrow >> i) & 1)
			result &= port(P_KEY0 + which * 5 + i);
	return (uint16_t)(0xffc0 | result);
}

static uint16_t gun_r(int p)
{
	if (p == 1 && !s_gun2)
		return 0xffff;   // (deerhunt, turkhunt: no second gun)
	return (uint16_t)(s_st.gun[p][1] << 8 | s_st.gun[p][0]);
}

static uint16_t io_r(uint32_t a)
{
	switch (s_m->kind) {
	case K_GUNDAMEX:
		switch (a) {
		case 0x600000: return port(P_DSW1);
		case 0x600002: return port(P_DSW2);
		case 0x700000: return port(P_P1);
		case 0x700002: return port(P_P2);
		case 0x700004: return port(P_SYSTEM);
		case 0x700008: return port(P_IN0);
		case 0x70000a: return port(P_IN1);
		}
		break;
	case K_GRDIANS:
		switch (a) {
		case 0x600000: return port(P_DSW1);
		case 0x600002: return port(P_DSW2);
		case 0x700000: return port(P_P1);
		case 0x700002: return port(P_P2);
		case 0x700004: return port(P_SYSTEM);
		}
		break;
	case K_MJ4SIMAI:
		switch (a) {
		case 0x600000: return mj_key_r(0);
		case 0x600002: return mj_key_r(1);
		case 0x600100: return port(P_SYSTEM);
		case 0x600300: return port(P_DSW1);
		case 0x600302: return port(P_DSW2);
		}
		break;
	case K_MYANGEL:
	case K_MYANGEL2: {
		const uint32_t b = s_m->kind == K_MYANGEL ? 0x700000 : 0x600000;
		if (a == b) return port(P_P1);
		if (a == b + 2) return port(P_P2);
		if (a == b + 4) return port(P_SYSTEM);
		if (a == b + 0x300) return port(P_DSW1);
		if (a == b + 0x302) return port(P_DSW2);
		break;
	}
	case K_PZLBOWL:
		switch (a) {
		case 0x400300: return port(P_DSW1);
		case 0x400302: return port(P_DSW2);
		case 0x500000: return port(P_P1);
		case 0x500002: return port(P_P2);
		case 0x500004:   // (pzlbowl_coins_r on the low byte: the port and a random bit)
			s_st.rng = s_st.rng * 1103515245u + 12345u;
			return (uint16_t)((port(P_SYSTEM) | ((s_st.rng >> 16) & 0x80)) & 0xff);
		case 0x700000: {   // (pzlbowl_protection_r: a byte of the program, its offset in RAM)
			const uint32_t addr = ((uint32_t)m68k_word(s_ram, 0xba16) << 16) | m68k_word(s_ram, 0xba18);
			return s_rom[(addr - 2) & (s_rom_size - 1)];
		}
		}
		break;
	case K_PENBROS:
	case K_ABLASTB: {
		const uint32_t d = s_m->kind == K_PENBROS ? 0x500300 : 0x508300;
		if (a == 0x600000) return port(P_P1);
		if (a == 0x600002) return port(P_P2);
		if (a == 0x600004) return port(P_SYSTEM);
		if (a == d) return port(P_DSW1);
		if (a == d + 2) return port(P_DSW2);
		break;
	}
	case K_REELQUAK:
		switch (a) {
		case 0x400000: return port(P_P1);
		case 0x400002: return port(P_TICKET);
		case 0x400004: return port(P_SYSTEM);
		case 0x400300: return port(P_DSW1);
		case 0x400302: return port(P_DSW2);
		}
		break;
	case K_SAMSHOOT:
		switch (a) {
		case 0x400000: return port(P_DSW1);
		case 0x400002: return port(P_BUTTONS);
		case 0x500000: return gun_r(0);
		case 0x580000: return gun_r(1);
		case 0x700000: return port(P_TRIGGER);
		case 0x700002: return port(P_PUMP);
		case 0x700004: return port(P_COIN);
		}
		break;
	case K_TELPACFL:
		switch (a) {
		case 0x600000: return port(P_DSW1);
		case 0x600002: return port(P_DSW2);
		case 0x700000: return port(P_COIN);
		case 0x700002: return port(P_P1);
		case 0x700004: return port(P_SERVICE);
		case 0x700006: return port(P_UNKNOWN);
		}
		break;
	}
	return 0;
}

static uint32_t vregs_base(void) { return s_m->video ? s_m->video + 0x60000 : 0xb60000; }

static uint16_t main_read16(uint32_t a)
{
	a &= 0xfffffe;
	if (a >= 0xfffc00)
		return s_tmp_on ? tmp68301_read(&s_tmp, a) : 0;
	const uint32_t vr = vregs_base();
	if (a >= vr && a < vr + 0x40)
		return s_vregs[(a - vr) >> 1];
	if (a >= s_m->x1snd && a < s_m->x1snd + 0x4000)
		return x1_010_word_r(&s_snd, (a - s_m->x1snd) >> 1);
	return io_r(a);
}

static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a);
	return (uint8_t)((a & 1) ? v : v >> 8);
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xfffffe;
	if (a >= 0xfffc00) {
		if (s_tmp_on)
			tmp68301_write(&s_tmp, a, v, mask);
		return;
	}
	const uint32_t vr = vregs_base();
	if (a >= vr && a < vr + 0x40) {
		vregs_w((a - vr) >> 1, v, mask);
		return;
	}
	if (a >= s_m->x1snd && a < s_m->x1snd + 0x4000) {
		x1_010_word_w(&s_snd, (a - s_m->x1snd) >> 1, (uint16_t)(v & mask));
		return;
	}
	const uint32_t sb = s_m->video ? s_m->video : 0xb00000;   // (sprite RAM: written here, its writes queued for the drawing)
	if (a >= sb && a < sb + 0x40000) {
		const uint32_t i = (a - sb) >> 1;
		COMBINE(s_spr[i], v, mask);
		ev_push(EV_SPRW | i, s_spr[i]);
		return;
	}
	if (s_m->banks && a >= s_m->banks && a < s_m->banks + 0x10) {   // (sound_bank_w, umask16(0x00ff))
		if (mask & 0x00ff)
			s_st.bank[((a - s_m->banks) >> 1) & 7] = (uint8_t)v;
		return;
	}
	switch (s_m->kind) {
	case K_MJ4SIMAI:
		if (a == 0x600004 && (mask & 0x00ff))   // (the keyboard row)
			s_st.keyrow = (uint8_t)v;
		break;
	case K_TELPACFL:
		if (a == 0x70000c && (mask & 0x00ff))   // (telpacfl_lamp2_w: the medal hopper's motor)
			ticket_motor((v >> 3) & 1);
		break;
	default:
		break;
	}
}

static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	if (a & 1)
		main_write16m(a, v, 0x00ff);
	else
		main_write16m(a, (uint16_t)(v << 8), 0xff00);
}

// ---- sound ----
static void set_banks(void)
{
	const uint32_t max = s_x1rom_size / 0x20000;
	for (int i = 0; i < 8; i++)
		s_snd.bank[i] = max ? s_x1rom + (size_t)(s_st.bank[i] % max) * 0x20000 : s_x1rom;
}

static void run_sound(void)
{
	const int n = rbx_line_cycles(&s_snd_frac, s_m->clock / 512, FRAME_MHZ, LINES);
	if (n <= 0)
		return;
	set_banks();
	int32_t l[8] = { 0 }, r[8] = { 0 };
	const int m = n > 8 ? 8 : n;
	x1_010_render(&s_snd, l, r, m);
	for (int i = 0; i < m; i++)
		rbx_audio_put(l[i] + r[i]);   // (both outputs at 1.0 to the mono speaker)
}

// ---- the frame ----
static void capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap_fb, s_fb, (size_t)s_m->vis_w * s_m->vis_h * 2);
	if (memcmp(s_snap_pal, s_pal, PENS * 2)) {
		memcpy(s_snap_pal, s_pal, PENS * 2);
		s_pal_dirty = 1;
	}
	rbx_video_frame_done();
}

static void s2_render(void)
{
	if (s_pal_dirty) {   // (xRGB_555)
		for (int i = 0; i < PENS; i++) {
			const uint16_t p = s_snap_pal[i];
			const int r = (p >> 10) & 31, g = (p >> 5) & 31, b = p & 31;
			rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (b << 3 | b >> 2));
		}
		s_pal_dirty = 0;
	}
	const int w = s_m->vis_w, h = s_m->vis_h;
	for (int y = 0; y < h; y++)
		memcpy(rbx_video.bitmap + (size_t)y * rbx_video.pitch, s_snap_fb + (size_t)y * w, (size_t)w * 2);
}

static void vblank_begin(void)
{
	if (s_tmp_on) {
		tmp68301_set_input(&s_tmp, 0, 1, 0);
		s_st.vbl_end_at = s_st.cyc + (uint64_t)s_m->clock * 2500 / 1000000;   // (set_vblank_time(2500 us))
		if (!s_st.vbl_end_at)
			s_st.vbl_end_at = 1;
	} else
		m68k_bus_hold_irq(2);   // (ablastb: irq2_line_hold)
	update_to(s_m->vis_h - 1);
	capture();
}

static void run_until(uint64_t target)
{
	while (s_st.cyc < target) {
		uint64_t next = target;
		uint64_t ev = s_tmp_on ? tmp68301_next_event(&s_tmp) : 0;
		if (ev && ev < next)
			next = ev;
		if (s_st.raster_at && s_st.raster_at < next)
			next = s_st.raster_at;
		if (s_st.vbl_end_at && s_st.vbl_end_at < next)
			next = s_st.vbl_end_at;
		if (next > s_st.cyc) {
			uint64_t n = next - s_st.cyc;
			if (n > 0x40000000u)
				n = 0x40000000u;
			s_in_exec = 1;
			const int ran = m68k_execute((int)n);
			s_in_exec = 0;
			s_st.cyc += (uint64_t)(ran > 0 ? ran : 0);
		}
		if (s_st.vbl_end_at && s_st.cyc >= s_st.vbl_end_at) {
			s_st.vbl_end_at = 0;
			tmp68301_set_input(&s_tmp, 0, 0, 0);
		}
		if (s_st.raster_at && s_st.cyc >= s_st.raster_at) {
			s_st.raster_at = 0;
			raster_fire();
		}
		if (s_tmp_on && (ev = tmp68301_next_event(&s_tmp)) != 0 && s_st.cyc >= ev)
			tmp68301_update(&s_tmp, s_st.cyc);
	}
}

static void read_controls(void)
{
	if (s_m->kind == K_SAMSHOOT)
		for (int p = 0; p < 2; p++) {   // (PORT_MINMAX(0x25, 0xc5) and (0x0800, 0xf800))
			int x = s_st.gun[p][0] + rbx_analog(p + 1, RBX_IPT_LIGHTGUN_X);
			int y = s_st.gun[p][1] + rbx_analog(p + 1, RBX_IPT_LIGHTGUN_Y);
			s_st.gun[p][0] = x < 0x25 ? 0x25 : x > 0xc5 ? 0xc5 : x;
			s_st.gun[p][1] = y < 0x08 ? 0x08 : y > 0xf8 ? 0xf8 : y;
		}
	else if (s_m->kind == K_TELPACFL) {   // (the knob: PORT_KEYDELTA(15), no centering)
		const int k = s_st.knob + rbx_analog_dpad(1, RBX_IPT_PADDLE, 15);
		s_st.knob = k < 0 ? 0 : k > 0xff ? 0xff : k;
	}
	if (s_m->kind == K_REELQUAK || s_m->kind == K_TELPACFL)
		ticket_frame();
}

static void s2_frame(void)
{
	read_controls();
	const uint64_t fc = rbx_line_cycles(&s_frame_frac, s_m->clock, FRAME_MHZ, 1);
	if (s_st.frame)
		s_st.fstart += s_st.fcyc;
	s_st.fcyc = (uint32_t)fc;
	s_drawn = 0;
	if (s_m->kind == K_SAMSHOOT && s_st.frame)   // (set_periodic_int(irq2_line_hold, 60 Hz): from 1/60 s on)
		tmp68301_set_input(&s_tmp, 2, 1, 1);
	for (int line = 0; line < LINES; line++) {
		if (line == s_m->vis_h)
			vblank_begin();
		run_until(s_st.fstart + (uint64_t)(line + 1) * s_st.fcyc / LINES);
		run_sound();
	}
	s_st.frame++;
}

// ---- init / reset ----
static void s2_exit(void);

static void nv_save(void)
{
	if (!s_nv_path[0] || !s_nv_size)
		return;
	rbx_file_put_now(s_nv_path, s_ram3, s_nv_size);
}

static void s2_reset(void)
{
	memset(s_ram, 0, 0x40000);
	if (!s_nv_size)
		memset(s_ram3, 0, 0x10000);
	memset(s_vram5, 0, 0x10000);
	memset(s_spr, 0, 0x40000);
	memset(s_pal, 0, PENS * 2);
	memset(s_vregs, 0, sizeof(s_vregs));
	memset(s_priv, 0, sizeof(s_priv));
	draw_state_sync();
	memset(&s_st, 0, sizeof(s_st));
	s_st.fcyc = s_m->clock / 60;
	s_st.knob = 0;
	for (int p = 0; p < 2; p++) {   // (the guns' defaults: 0x80, 0x8000)
		s_st.gun[p][0] = 0x80;
		s_st.gun[p][1] = 0x80;
	}
	s_drawn = 0;
	s_cpu_frac = s_snd_frac = s_frame_frac = 0;
	x1_010_reset(&s_snd);
	set_banks();
	m68k_bus_irq_reset();
	if (s_tmp_on) {
		rbx_m68k_int_hook = tmp_int_hook;
		tmp68301_reset(&s_tmp);
	}
	m68k_pulse_reset();
}

// the 8x8 tiles' bit planes (tile_layout: plane p of a row in its byte 7 - p, msb first) to a byte a pixel, in place
static void decode_gfx(uint8_t *g, uint32_t tiles)
{
	for (uint32_t t = 0; t < tiles; t++) {
		uint8_t *p = g + (size_t)t * 64, out[64];
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				uint8_t pen = 0;
				for (int b = 0; b < 8; b++)
					pen |= (uint8_t)(((p[y * 8 + b] >> (7 - x)) & 1) << b);
				out[y * 8 + x] = pen;
			}
		memcpy(p, out, 64);
	}
}

static int s2_init(const rbx_game *g)
{
	s_m = NULL;
	for (size_t i = 0; i < sizeof(s_machines) / sizeof(s_machines[0]); i++)
		if (!strcmp(s_machines[i].name, g->machine))
			s_m = &s_machines[i];
	if (!s_m) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	const machine_t *m = s_m;
	s_tmp_on = m->kind != K_ABLASTB;
	s_gun2 = !strncmp(g->set, "wschamp", 7) || !strncmp(g->set, "trophyh", 7);
	uint32_t gsize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_gfx = rbx_region("video", &gsize);
	s_x1rom = rbx_region("x1snd", &s_x1rom_size);
	if (!s_rom || !s_gfx || !s_x1rom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_gfx_tiles = gsize / 64;
	decode_gfx(s_gfx, s_gfx_tiles);
	m68k_bus_swap16(s_rom, s_rom_size);

	s_ram = (uint8_t *)calloc(1, 0x40000);
	s_ram3 = (uint8_t *)calloc(1, 0x10000);
	s_vram5 = (uint8_t *)calloc(1, 0x10000);
	s_spr = (uint16_t *)calloc(1, 0x40000);
	s_pal = (uint16_t *)calloc(1, PENS * 2);
	s_snap_pal = (uint16_t *)calloc(1, PENS * 2);
	s_fb = (uint16_t *)calloc((size_t)m->vis_w * m->vis_h, 2);
	s_snap_fb = (uint16_t *)calloc((size_t)m->vis_w * m->vis_h, 2);
	s_rep[0].spr = (uint16_t *)calloc(1, 0x40000);
	s_rep[1].spr = (uint16_t *)calloc(1, 0x40000);
	s_evq = (uint32_t *)calloc(EVQ, 8);
	s_evh = s_rep[0].t = s_rep[1].t = 0;
	s_bandseq = 0;
	// the drawing's helpers (raster interrupts every few lines: they spin between the bands)
	rbx_aux_set_job(aux_job0);
	rbx_aux_set_spin(400);
	rbx_aux2_set_alone(RBX_AUX2_BY_RENDER);
	rbx_aux2_set_job(aux_job1);
	rbx_aux2_set_spin(400);
	if (!s_ram || !s_ram3 || !s_vram5 || !s_spr || !s_pal || !s_snap_pal || !s_fb || !s_snap_fb || !s_rep[0].spr || !s_rep[1].spr || !s_evq ||
	    rbx_audio_setup(m->clock / 512) || rbx_video_setup(m->vis_w, m->vis_h, PENS)) {
		rbx_fail("%s: out of memory", g->set);
		s2_exit();
		return -1;
	}
	s_pal_dirty = 1;
	for (int i = 0; i < P_KEY0; i++)
		s_port[i] = rbx_port_find(s_port_tags[i]);
	for (int i = 0; i < 10; i++) {
		char tag[8];
		snprintf(tag, sizeof(tag), "KEY%d", i);
		s_port[P_KEY0 + i] = rbx_port_find(tag);
	}

	x1_010_init(&s_snd, s_x1rom, s_x1rom_size);
	tmp68301_init(&s_tmp, m->clock);
	s_tmp.now = cpu_now;
	s_tmp.par_r = par_r;
	s_tmp.par_w = par_w;
	s_tmp.set_level = tmp_level;
	s_tmp.events_changed = end_slice;

	s_eeprom_on = m->kind == K_GUNDAMEX;
	if (s_eeprom_on) {   // (EEPROM_93C46_16BIT, its ROM's contents)
		eepser_init(&s_eeprom, EEPSER_93CXX, 64, 16, (int64_t)m->clock);
		uint32_t esize;
		const uint8_t *e = rbx_region_opt("eeprom", &esize);
		eepser_default(&s_eeprom, e, e ? esize : 0);
		eepser_nv_load(&s_eeprom, g->set);
	}

	// the battery backed RAM (nvram_device::DEFAULT_ALL_0), kept as MAME's <set>.nv
	s_nv_size = m->kind == K_REELQUAK || m->kind == K_TELPACFL ? 0x4000 : m->kind == K_SAMSHOOT ? 0x10000 : 0;
	s_nv_path[0] = 0;
	if (s_nv_size) {
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			FILE *f = fopen(s_nv_path, "rb");
			if (f) {
				if (fread(s_ram3, 1, s_nv_size, f) != s_nv_size)
					memset(s_ram3, 0, s_nv_size);
				fclose(f);
			}
		}
	}

	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);
	m68k_bus_reset_map();
	const uint32_t rom_map = s_rom_size < m->rom_end + 1 ? s_rom_size : m->rom_end + 1;
	m68k_bus_map(0, rom_map - 1, s_rom, rom_map, 1, 0);
	if (m->kind == K_GUNDAMEX && s_rom_size >= 0x580000)   // (the ROM at 0x500000)
		m68k_bus_map(0x500000, 0x57ffff, s_rom + 0x500000, 0x80000, 1, 0);
	switch (m->kind) {
	case K_PENBROS:
		m68k_bus_map(0x200000, 0x23ffff, s_ram, 0x40000, 1, 1);
		m68k_bus_map(0x300000, 0x30ffff, s_ram3, 0x10000, 1, 1);
		break;
	case K_ABLASTB:
		m68k_bus_map(0x200000, 0x23ffff, s_ram, 0x40000, 1, 1);
		break;
	default:
		m68k_bus_map(0x200000, 0x20ffff, s_ram, 0x10000, 1, 1);
		break;
	}
	if (m->kind == K_GRDIANS)   // (? tile data)
		m68k_bus_map(0x304000, 0x30ffff, s_ram3 + 0x4000, 0xc000, 1, 1);
	if (s_nv_size)
		m68k_bus_map(0x300000, 0x300000 + s_nv_size - 1, s_ram3, s_nv_size, 1, 1);
	if (m->video) {
		m68k_bus_map(m->video, m->video + 0x3ffff, s_spr, 0x40000, 1, 0);   // (writes: main_write16m)
		m68k_bus_map(m->video + 0x40000, m->video + 0x4ffff, s_pal, 0x10000, 1, 1);
		m68k_bus_map(m->video + 0x50000, m->video + 0x5ffff, s_vram5, 0x10000, 1, 1);
	} else {
		m68k_bus_map(0xb00000, 0xb3ffff, s_spr, 0x40000, 1, 0);
		m68k_bus_map(0xb40000, 0xb4ffff, s_pal, 0x10000, 1, 1);
	}
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	s2_reset();
	return 0;
}

static void s2_exit(void)
{
	rbx_aux_set_job(NULL);   // (syncs first)
	rbx_aux2_set_job(NULL);
	if (s_ram3)
		nv_save();
	if (s_eeprom_on)
		eepser_nv_save(&s_eeprom);
	s_eeprom_on = 0;
	s_nv_size = 0;
	s_nv_path[0] = 0;
	rbx_m68k_int_hook = NULL;
	free(s_ram);
	free(s_ram3);
	free(s_vram5);
	free(s_spr);
	free(s_pal);
	free(s_snap_pal);
	free(s_fb);
	free(s_snap_fb);
	free(s_rep[0].spr);
	free(s_rep[1].spr);
	s_rep[0].spr = s_rep[1].spr = NULL;
	free(s_evq);
	s_ram = s_ram3 = s_vram5 = NULL;
	s_spr = s_pal = s_snap_pal = s_fb = s_snap_fb = NULL;
	s_evq = NULL;
	s_evh = s_rep[0].t = s_rep[1].t = 0;
}

// ---- save states ----
static int s2_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	rbx_aux_sync();   // (the drawing's queue done: it reads nothing saved)
	const uint32_t csize = m68k_context_size();
	if (csize > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus.irq_pending, m68k_bus.irq_asserted };
	uint64_t fr[3] = { s_cpu_frac, s_snd_frac, s_frame_frac };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "tmp", &s_tmp, TMP68301_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ram", s_ram, 0x40000);
	rc |= rbx_state_io(ctx, "ram3", s_ram3, 0x10000);
	rc |= rbx_state_io(ctx, "vram5", s_vram5, 0x10000);
	rc |= rbx_state_io(ctx, "spr", s_spr, 0x40000);
	rc |= rbx_state_io(ctx, "pal", s_pal, PENS * 2);
	rc |= rbx_state_io(ctx, "vregs", s_vregs, sizeof(s_vregs));
	rc |= rbx_state_io(ctx, "priv", s_priv, sizeof(s_priv));
	rc |= rbx_state_io(ctx, "x1010", &s_snd, X1_010_STATE_SIZE);
	rc |= rbx_state_io(ctx, "st", &s_st, sizeof(s_st));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	if (s_eeprom_on)
		rc |= eepser_state(&s_eeprom, ctx, load, "eeprom");
	if (rc)
		return -1;
	if (load) {
		m68k_set_context(cpu);
		m68k_bus.irq_pending = irq[0];
		m68k_bus.irq_asserted = irq[1];
		if (s_tmp_on) {
			rbx_m68k_int_hook = tmp_int_hook;
			m68k_set_irq(s_tmp.level);
		} else
			m68k_bus_update_irq();
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_frame_frac = fr[2];
		set_banks();
		draw_state_sync();
	}
	return 0;
}

static int s2_save(void *ctx) { return s2_state(ctx, 0); }
static int s2_load(void *ctx) { return s2_state(ctx, 1); }
static int s2_supported(const char *machine)
{
	if (!strcmp(machine, "ablastb"))   // (MAME: not working, an FPGA for the sound; a black screen here too)
		return 0;
	for (size_t i = 0; i < sizeof(s_machines) / sizeof(s_machines[0]); i++)
		if (!strcmp(s_machines[i].name, machine))
			return 1;
	return 0;
}

const rbx_driver rbx_drv_seta2 = { "Seta 2", s2_init, s2_exit, s2_reset, s2_frame, s2_save, s2_load, s2_supported,
                                   s2_render, NULL };
