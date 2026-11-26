/*  Technos Double Dragon 3, The Combatribes and WWF WrestleFest. Ported from MAME src/mame/technos/ddragon3.cpp and
    ddragon3_v.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail, David Haywood).
    68000 (10 MHz; WrestleFest 12 MHz) with an IRQ every 16 lines (level 5; WrestleFest 2) and one at line 248 (6; 3),
    both held until acknowledged; two 16x16 tilemaps of 512x512 (BG: one word per tile, FG: two), 16x16 sprites in
    chains from a buffered sprite RAM, WrestleFest's 8x8 text layer. The screen is drawn in 16-line pieces (MAME's
    update_partial at each raster IRQ), so the scroll registers are latched per piece. Sound: Z80 at 3.58 MHz with a
    YM2151 (its IRQ) and an OKI6295 at 1.056 MHz (pin 7 high, a 256 KB bank), the latch on the NMI.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define Z80_CLOCK 3579545
#define YM_CLOCK 3579545
#define OKI_CLOCK 1056000
#define FRAME_MHZ 57445          // set_raw(28 MHz / 4, 448, 0, 320, 272, 8, 248)
#define LINES 272
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 320
#define VBL_LINE 248

enum { M_DD3, M_DD3B, M_CTRIBE, M_WWF, M_WWFB };

typedef struct {
	uint16_t fg_sx, fg_sy, bg_sx, bg_sy, vreg, tilebase;
	uint8_t flip, pri;
} line_regs;

static int s_mach;
static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static uint16_t (*s_ram_hp)[0x2000];               // 0x1c0000 .. 0x1c3fff
#define s_ram (*s_ram_hp)
static uint16_t s_ram2[0x400];               // ctribe 0x082800 .. 0x082fff
static uint16_t s_fgram[0x800];              // 0x080000 .. 0x080fff (32x32, two words a tile)
static uint16_t s_bgram[0x800];              // 0x082000 .. 0x0827ff (WrestleFest .. 0x082fff)
static uint16_t s_fg0ram[0x1000];            // WrestleFest 0x0c0000 .. 0x0c1fff
static uint16_t s_spriteram[0x1000];         // 0x800 words (WrestleFest 0x1000)
static uint16_t s_spritebuf[0x1000];
static uint16_t (*s_pal_hp)[0x2000];
#define s_pal (*s_pal_hp)
static uint32_t s_spr_words, s_pal_n;
static uint16_t s_vreg, s_fg_sx, s_fg_sy, s_bg_sx, s_bg_sy, s_tilebase;
static uint8_t s_flip, s_pri;
static int s_irq_raster, s_irq_vbl;
static int s_sprite_xoff, s_bg0_dx, s_bg1_dx[2];
static line_regs s_lines[LINES];
static int s_seg_start;
static rbx_gfx s_gfxtile, s_gfxspr, s_gfxtext;
static int s_port[6], s_dsw[2];
static int s_line;
static uint32_t s_cpu_clock;
static uint64_t s_cpu_frac, s_z80_frac, s_ym_frac;
static struct {
	uint16_t fgram[0x800], bgram[0x800], fg0ram[0x1000], spr[0x1000], pal[0x2000];
	line_regs lines[LINES];
} *s_snap;

// ---- sound: Z80, YM2151, OKI ----
static z80 s_z80;
static uint8_t s_zram[0x800];
static uint8_t s_latch;
static int s_latch_pending;
static okim6295_t s_oki;
static int s_ym_irq;
static uint32_t s_oki_acc;
static int32_t s_oki_hold;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static const uint8_t *s_zmap[256];

static void ym_irq(int state) { s_ym_irq = state; }

static uint8_t latch_read(void)
{
	s_latch_pending = 0;
	return s_latch;
}
static void latch_write(uint8_t v)
{
	s_latch = v;
	if (!s_latch_pending)   // (data_pending to the NMI: an edge only when it rises)
		z80_gen_nmi(&s_z80);
	s_latch_pending = 1;
}

static uint8_t z80_read(void *u, uint16_t a)
{
	(void)u;
	if (s_mach == M_CTRIBE) {
		if (a < 0x8000) return a < s_zrom_size ? s_zrom[a] : 0;
		if (a < 0x8800) return s_zram[a - 0x8000];
		if (a == 0x8800 || a == 0x8801) return rbx_ym_read(0, 1);   // status_r
		if (a == 0x9800) return oki_read(&s_oki);
		if (a == 0xa000) return latch_read();
		return 0;
	}
	if (a < 0xc000) return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0xc800) return s_zram[a - 0xc000];
	if (a == 0xc800 || a == 0xc801) return rbx_ym_read(0, a & 1);
	if (a == 0xd800) return oki_read(&s_oki);
	if (a == 0xe000) return latch_read();
	return 0;
}
static void z80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_mach == M_CTRIBE) {
		if (a >= 0x8000 && a < 0x8800) s_zram[a - 0x8000] = v;
		else if (a == 0x8800 || a == 0x8801) rbx_ym_write(0, a & 1, v);
		else if (a == 0x9800) oki_write(&s_oki, v);
		return;
	}
	if (a >= 0xc000 && a < 0xc800) s_zram[a - 0xc000] = v;
	else if (a == 0xc800 || a == 0xc801) rbx_ym_write(0, a & 1, v);
	else if (a == 0xd800) oki_write(&s_oki, v);
	else if (a == 0xe800) s_oki.bank = (uint32_t)(v & 1) * 0x40000;   // oki_bankswitch_w
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
	const uint32_t yr = rbx_ym_rate(0);
	for (int i = 0; i < n; i++) {
		s_oki_acc += s_oki.rate;
		while (s_oki_acc >= yr) {
			s_oki_acc -= yr;
			s_oki_hold = oki_sample(&s_oki);
		}
		s_mix[s_mix_n + i] += s_oki_hold;
	}
	s_mix_n += n;
}

static void sound_frame_end(void)
{
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

// ---- registers ----
static inline void combine(uint16_t *w, uint16_t v, uint16_t mask) { *w = (uint16_t)((*w & ~mask) | (v & mask)); }

static void scroll_w(int offset, uint16_t v, uint16_t mask)
{
	switch (offset) {
	case 0: combine(&s_fg_sx, v, mask); break;
	case 1: combine(&s_fg_sy, v, mask); break;
	case 2: combine(&s_bg_sx, v, mask); break;
	case 3: combine(&s_bg_sy, v, mask); break;
	case 5: s_flip = v & 1; break;
	case 6: combine(&s_tilebase, v, mask); s_tilebase &= 0x1ff; break;
	}
}
static uint16_t scroll_r(int offset)
{
	switch (offset) {
	case 0: return s_fg_sx;
	case 1: return s_fg_sy;
	case 2: return s_bg_sx;
	case 3: return s_bg_sy;
	case 5: return s_flip;
	case 6: return s_tilebase;
	}
	return 0;
}

static int vblank(void) { return s_line >= VBL_LINE || s_line < VIS_Y0; }

static uint16_t port_in(int i)
{
	if (s_mach == M_WWF || s_mach == M_WWFB) {
		const uint32_t d1 = (uint32_t)rbx_port_read(s_dsw[0]), d2 = (uint32_t)rbx_port_read(s_dsw[1]);
		switch (i) {
		case 0: rbx_port_custom(s_port[0], 0x3000, ((d2 & 0xc0) >> 6) << 12); break;
		case 1: rbx_port_custom(s_port[1], 0x3f00, (d2 & 0x3f) << 8); break;
		case 2: rbx_port_custom(s_port[2], 0x3f00, (d1 & 0x3f) << 8); break;
		case 3: rbx_port_custom(s_port[3], 0x0700, ((d1 & 0xc0) >> 6) << 8 | (vblank() ? 0x0400 : 0)); break;
		}
	} else if (s_mach != M_DD3 && i == 0)
		rbx_port_custom(s_port[0], 0x0800, vblank() ? 0x0800 : 0);
	return (uint16_t)rbx_port_read(s_port[i]);
}

// ---- bus ----
static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	switch (s_mach) {
	case M_DD3:
		if (a >= 0x100000 && a < 0x100008) return port_in((int)((a >> 1) & 3));
		break;
	case M_DD3B:
	case M_CTRIBE:
		if (a >= 0x180000 && a < 0x180008) return port_in((int)((a >> 1) & 3));
		if (s_mach == M_CTRIBE && a >= 0x0c0000 && a < 0x0c0010) return scroll_r((int)((a >> 1) & 7));
		break;
	default:
		if (a >= 0x140020 && a < 0x140028) return port_in((int)((a >> 1) & 3));
		if (a >= 0x100000 && a < 0x100008) return scroll_r((int)((a >> 1) & 3));
		if (a >= 0x180000 && a < 0x190000) {   // (A5 and A6 not connected)
			const uint32_t o = (a >> 1) & 0x7fff;
			return s_pal[(o & 0x000f) | (o & 0x7fc0) >> 2];
		}
		break;
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
	if (s_mach == M_WWF || s_mach == M_WWFB) {
		if (a >= 0x0c0000 && a < 0x0c2000)   // (wwfwfest_fg0_videoram_w: data & 0xff, no mask)
			s_fg0ram[(a >> 1) & 0xfff] = v & 0xff;
		else if (a >= 0x100000 && a < 0x100008)
			scroll_w((int)((a >> 1) & 3), v, mask);
		else if (a >= 0x10000a && a < 0x10000c)
			s_flip = v & 1;
		else if (a >= 0x140000 && a < 0x140004)
			m68k_bus_set_line(a & 2 ? 2 : 3, 0);
		else if (a >= 0x140008 && a < 0x14000a)
			memcpy(s_spritebuf, s_spriteram, s_spr_words * 2);
		else if (a >= 0x14000c && a < 0x14000e)
			latch_write((uint8_t)v);
		else if (a == 0x140010 && (mask & 0x00ff))
			s_pri = (uint8_t)v;
		else if (a >= 0x180000 && a < 0x190000) {
			const uint32_t o = (a >> 1) & 0x7fff;
			combine(&s_pal[(o & 0x000f) | (o & 0x7fc0) >> 2], v, mask);
		}
		return;
	}
	const uint32_t io = s_mach == M_DD3 ? 0x100000 : 0x140000;
	if (a >= 0x0c0000 && a < 0x0c0010)
		scroll_w((int)((a >> 1) & 7), v, mask);
	else if (a >= io && a < io + 8) {
		switch ((a >> 1) & 3) {
		case 0: combine(&s_vreg, v, mask); break;
		case 1: if (mask & 0x00ff) latch_write((uint8_t)v); break;
		case 2: m68k_bus_set_line(6, 0); break;
		case 3: m68k_bus_set_line(5, 0); break;
		}
	} else if (s_mach != M_DD3 && a >= 0x140008 && a < 0x14000a)
		memcpy(s_spritebuf, s_spriteram, s_spr_words * 2);
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static void latch_lines(int end)   // (update_partial(end - 1): the lines since the last piece)
{
	line_regs r;
	r.fg_sx = s_fg_sx;
	r.fg_sy = s_fg_sy;
	r.bg_sx = s_bg_sx;
	r.bg_sy = s_bg_sy;
	r.vreg = s_vreg;
	r.tilebase = s_tilebase;
	r.flip = s_flip;
	r.pri = s_pri;
	for (int y = s_seg_start; y < end && y < LINES; y++)
		s_lines[y] = r;
	s_seg_start = end;
}

// one line of a 512x512 tilemap: kind 0 = BG (a word a tile), 1 = FG (two words, flips)
static void draw_tm_line(int kind, const uint16_t *vram, int y, uint32_t sx, uint32_t sy, int flip, int tilebase,
                         uint16_t base, int opaque, uint16_t *dst)
{
	const uint32_t ty = (uint32_t)(flip ? (255 - y) + (int)sy : y + (int)sy) & 511;
	const uint32_t row = (ty >> 4) * 32, fy = ty & 15;
	int x = 0;
	while (x < VIS_W) {
		const uint32_t tx = (uint32_t)(flip ? (VIS_W - 1 - x) + (int)sx : x + (int)sx) & 511;
		const uint32_t col = tx >> 4;
		uint32_t code, color;
		int fx = 0, fyy = 0;
		if (kind == 0) {
			const uint16_t attr = vram[row + col];
			code = (attr & 0x0fffu) | (uint32_t)(tilebase & 1) << 12;
			color = attr >> 12;
		} else {
			const uint16_t *t = &vram[(row + col) * 2];
			code = t[1] & 0x1fffu;
			color = t[0] & 0x000fu;
			fx = (t[0] >> 6) & 1;
			fyy = (t[0] >> 7) & 1;
		}
		code %= s_gfxtile.count;
		const uint8_t *src = s_gfxtile.pix + code * 256 + (fyy ? 15 - fy : fy) * 16;
		const uint16_t c0 = (uint16_t)(base + color * 16);
		int px = (int)(tx & 15);
		if (!opaque && !s_gfxtile.used[code]) {
			x += flip ? px + 1 : 16 - px;
			continue;
		}
		if (!flip) {
			for (; px < 16 && x < VIS_W; px++, x++) {
				const uint8_t p = src[fx ? 15 - px : px];
				if (p || opaque)
					dst[x] = (uint16_t)(c0 + p);
			}
		} else {
			for (; px >= 0 && x < VIS_W; px--, x++) {
				const uint8_t p = src[fx ? 15 - px : px];
				if (p || opaque)
					dst[x] = (uint16_t)(c0 + p);
			}
		}
	}
}

static void draw_layer(int layer, const line_regs *r, int sx, int sy, int y0, int y1, int opaque)
{
	const int wwf = s_mach == M_WWF || s_mach == M_WWFB;
	const uint16_t base = layer ? (wwf ? 0x1000 : 256) : (wwf ? 0x0c00 : 512);
	for (int y = y0; y < y1; y++)
		draw_tm_line(layer, layer ? s_snap->fgram : s_snap->bgram, y, (uint32_t)sx, (uint32_t)sy, r->flip,
		             r->tilebase, base, opaque, rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch);
}

static void draw_text(int y0, int y1, int flip)   // WrestleFest's 8x8 text layer (64x32, no scroll)
{
	for (int y = y0; y < y1; y++) {
		uint16_t *dst = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const int ty = (flip ? 255 - y : y) & 255;
		int last = -1;
		uint32_t cbase = 0;
		const uint8_t *trow = NULL;
		for (int x = 0; x < VIS_W; x++) {
			const int tx = (flip ? VIS_W - 1 - x : x) & 511;
			if ((tx >> 3) != last) {   // (a character looked up once for its 8 pixels)
				last = tx >> 3;
				const uint16_t *t = &s_snap->fg0ram[((ty >> 3) * 64 + (tx >> 3)) * 2];
				const uint32_t code = rbx_wrap((uint32_t)((t[0] & 0x00ff) | (t[1] & 0x000f) << 8), s_gfxtext.count);
				trow = s_gfxtext.pix + code * 64 + (ty & 7) * 8;
				cbase = (uint32_t)((t[1] & 0x00f0) >> 4) * 16;
			}
			const uint8_t p = trow[tx & 7];
			if (p)
				dst[x] = (uint16_t)(cbase + p);
		}
	}
}

static void draw_tile16(uint32_t code, uint16_t c0, int flipx, int flipy, int sx, int sy, int y0, int y1)
{
	code %= s_gfxspr.count;
	if (!s_gfxspr.used[code])
		return;
	sy -= VIS_Y0;
	y0 -= VIS_Y0;
	y1 -= VIS_Y0;
	if (sy >= y1 || sy + 16 <= y0 || sx >= VIS_W || sx + 16 <= 0)
		return;
	const uint8_t *src = s_gfxspr.pix + code * 256;
	for (int j = 0; j < 16; j++) {
		const int yy = sy + j;
		if (yy < y0 || yy >= y1)
			continue;
		const uint8_t *s = src + (flipy ? 15 - j : j) * 16;
		uint16_t *d = rbx_video.bitmap + yy * rbx_video.pitch;
		for (int i = 0; i < 16; i++) {
			const int xx = sx + i;
			if ((unsigned)xx >= VIS_W)
				continue;
			const uint8_t p = s[flipx ? 15 - i : i];
			if (p)
				d[xx] = (uint16_t)(c0 + p);
		}
	}
}

static void draw_sprites(int y0, int y1, int flip)
{
	const uint16_t *src = s_snap->spr;
	const uint16_t base = (s_mach == M_WWF || s_mach == M_WWFB) ? 0x400 : 0;
	for (uint32_t o = 0; o + 8 <= s_spr_words; o += 8) {
		const uint16_t *s = src + o;
		if (!(s[1] & 0x0001))
			continue;
		int xpos = (s[5] & 0x00ff) | (s[1] & 0x0004) << 6;
		if (xpos > 512 - 16)
			xpos -= 512;
		xpos += s_sprite_xoff;
		int ypos = (s[0] & 0x00ff) | (s[1] & 0x0002) << 7;
		ypos = (256 - ypos) & 0x1ff;
		ypos -= 16;
		int flipx = (s[1] >> 4) & 1, flipy = (s[1] >> 3) & 1;
		const int chain = (s[1] & 0x00e0) >> 5;
		const int bw = chain >= 4 ? 3 : chain >= 2 ? 2 : chain ? 1 : 0;
		const uint32_t number = (uint32_t)((s[2] & 0x00ff) | (s[3] & 0x00ff) << 8) & (uint32_t)~((1u << bw) - 1) & 0xffff;
		const uint16_t c0 = (uint16_t)(base + (s[4] & 0x000f) * 16);
		if (flip) {
			flipy = !flipy;
			flipx = !flipx;
			ypos = 240 - ypos - s_sprite_xoff;
			xpos = 304 - xpos;
		}
		for (int count = 0; count <= chain; count++) {
			int yy;
			if (flip)
				yy = !flipy ? ypos + 16 * chain - 16 * count : ypos + 16 * count;
			else
				yy = flipy ? ypos - 16 * chain + 16 * count : ypos - 16 * count;
			draw_tile16(number + (uint32_t)count, c0, flipx, flipy, xpos, yy, y0, y1);
		}
	}
}

static void draw_piece(const line_regs *r, int y0, int y1)
{
	if (s_mach == M_WWF || s_mach == M_WWFB) {
		int fsx, fsy, bsx, bsy;
		if (r->pri == 0x78) {
			fsy = r->fg_sy; fsx = r->fg_sx + s_bg0_dx;
			bsy = r->bg_sy; bsx = r->bg_sx + s_bg1_dx[0];
		} else {
			bsy = r->fg_sy; bsx = r->fg_sx + s_bg1_dx[1];
			fsy = r->bg_sy; fsx = r->bg_sx + s_bg0_dx;
		}
		if (r->pri == 0x7b) {
			draw_layer(1, r, fsx, fsy, y0, y1, 1);
			draw_layer(0, r, bsx, bsy, y0, y1, 0);
			draw_sprites(y0, y1, r->flip);
		}
		if (r->pri == 0x7c) {
			draw_layer(1, r, fsx, fsy, y0, y1, 1);
			draw_sprites(y0, y1, r->flip);
			draw_layer(0, r, bsx, bsy, y0, y1, 0);
		}
		if (r->pri == 0x78) {
			draw_layer(0, r, bsx, bsy, y0, y1, 1);
			draw_layer(1, r, fsx, fsy, y0, y1, 0);
			draw_sprites(y0, y1, r->flip);
		}
		draw_text(y0, y1, r->flip);
		return;
	}
	if (s_mach == M_CTRIBE) {
		if (r->vreg & 8) {
			draw_layer(1, r, r->fg_sx, r->fg_sy, y0, y1, 1);
			draw_sprites(y0, y1, r->flip);
			draw_layer(0, r, r->bg_sx, r->bg_sy, y0, y1, 0);
		} else {
			draw_layer(0, r, r->bg_sx, r->bg_sy, y0, y1, 1);
			draw_layer(1, r, r->fg_sx, r->fg_sy, y0, y1, 0);
			draw_sprites(y0, y1, r->flip);
		}
		return;
	}
	if ((r->vreg & 0x60) == 0x40) {
		draw_layer(0, r, r->bg_sx, r->bg_sy, y0, y1, 1);
		draw_layer(1, r, r->fg_sx, r->fg_sy, y0, y1, 0);
		draw_sprites(y0, y1, r->flip);
	} else if ((r->vreg & 0x60) == 0x60) {
		draw_layer(1, r, r->fg_sx, r->fg_sy, y0, y1, 1);
		draw_layer(0, r, r->bg_sx, r->bg_sy, y0, y1, 0);
		draw_sprites(y0, y1, r->flip);
	} else {
		draw_layer(0, r, r->bg_sx, r->bg_sy, y0, y1, 1);
		draw_sprites(y0, y1, r->flip);
		draw_layer(1, r, r->fg_sx, r->fg_sy, y0, y1, 0);
	}
}

static void dd3_render(void)
{
	const int n = (int)s_pal_n;
	if (s_mach == M_DD3 || s_mach == M_DD3B) {
		for (int i = 0; i < n; i++) {   // xBGR_555
			const uint16_t p = s_snap->pal[i];
			const uint32_t r = p & 31, g = (p >> 5) & 31, b = (p >> 10) & 31;
			rbx_video.palette[i] = (r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (b << 3 | b >> 2);
		}
	} else {
		for (int i = 0; i < n; i++) {   // xBGR_444
			const uint16_t p = s_snap->pal[i];
			rbx_video.palette[i] = (uint32_t)(((p & 15) * 17) << 16 | (((p >> 4) & 15) * 17) << 8 | (((p >> 8) & 15) * 17));
		}
	}
	int y = VIS_Y0;
	while (y < VIS_Y0 + VIS_H) {   // (pieces of lines with the same registers)
		const line_regs *r = &s_snap->lines[y];
		int y1 = y + 1;
		while (y1 < VIS_Y0 + VIS_H && !memcmp(&s_snap->lines[y1], r, sizeof(*r)))
			y1++;
		draw_piece(r, y, y1);
		y = y1;
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->fgram, s_fgram, sizeof(s_fgram));
	memcpy(s_snap->bgram, s_bgram, sizeof(s_bgram));
	memcpy(s_snap->fg0ram, s_fg0ram, sizeof(s_fg0ram));
	memcpy(s_snap->spr, s_spritebuf, s_spr_words * 2);
	memcpy(s_snap->pal, s_pal, s_pal_n * 2);
	memcpy(s_snap->lines, s_lines, sizeof(s_lines));
	rbx_video_frame_done();
}

// ---- init / frame ----
static void dd3_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_ram2, 0, sizeof(s_ram2));
	memset(s_fgram, 0, sizeof(s_fgram));
	memset(s_bgram, 0, sizeof(s_bgram));
	memset(s_fg0ram, 0, sizeof(s_fg0ram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_spritebuf, 0, sizeof(s_spritebuf));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_lines, 0, sizeof(s_lines));
	s_vreg = s_fg_sx = s_fg_sy = s_bg_sx = s_bg_sy = s_tilebase = 0;
	s_flip = s_pri = 0;
	s_seg_start = 0;
	s_latch = 0;
	s_latch_pending = 0;
	s_ym_irq = 0;
	memset(s_zram, 0, sizeof(s_zram));
	z80_start();
	rbx_ym_reset(0);
	oki_reset(&s_oki);
	s_oki.bank = 0;
	s_oki_acc = 0;
	s_oki_hold = 0;
	s_mix_n = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_line = 0;
	s_cpu_frac = s_z80_frac = s_ym_frac = 0;
}

static void dd3_exit(void);

static int dd3_init(const rbx_game *g)
{
	if (!s_pal_hp && !(s_pal_hp = calloc(1, sizeof(*s_pal_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ram_hp && !(s_ram_hp = calloc(1, sizeof(*s_ram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	const char *m = g->machine;
	s_mach = !strcmp(m, "ddragon3b") ? M_DD3B : !strcmp(m, "ctribe") ? M_CTRIBE : !strcmp(m, "wwfwfest") ? M_WWF
	       : !strcmp(m, "wwfwfstb") ? M_WWFB : M_DD3;
	const int wwf = s_mach == M_WWF || s_mach == M_WWFB;
	uint32_t tsize, ssize, osize, xsize = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *tiles = rbx_region(wwf ? "gfx3" : "gfx1", &tsize), *spr = rbx_region("gfx2", &ssize);
	uint8_t *oki = rbx_region("oki", &osize), *text = wwf ? rbx_region("gfx1", &xsize) : NULL;
	if (!s_rom || s_rom_size < 0x80000 || !s_zrom || !tiles || !spr || !oki || (wwf && !text)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	{	// wwf_tile_layout (RGN_FRAC(1,2)), wwf_sprite_layout (RGN_FRAC(1,4)), tiles8x8_layout
		static uint32_t xt[16], yt[16], xs[16], ys[16], x8[8], y8[8];
		for (int i = 0; i < 8; i++) {
			xt[i] = (uint32_t)i;
			xt[8 + i] = (uint32_t)(32 * 8 + i);
			xs[i] = (uint32_t)i;
			xs[8 + i] = (uint32_t)(16 * 8 + i);
			y8[i] = (uint32_t)(i * 8);
		}
		static const uint32_t x8v[8] = { 1, 0, 8 * 8 + 1, 8 * 8 + 0, 16 * 8 + 1, 16 * 8 + 0, 24 * 8 + 1, 24 * 8 + 0 };
		memcpy(x8, x8v, sizeof(x8));
		for (int i = 0; i < 16; i++) {
			yt[i] = (uint32_t)(i * 16);
			ys[i] = (uint32_t)(i * 8);
		}
		const uint32_t th = tsize * 4, sq = ssize * 2;
		const uint32_t pt[4] = { 8, 0, th + 8, th }, ps[4] = { 0, sq, sq * 2, sq * 3 }, p8[4] = { 0, 2, 4, 6 };
		const rbx_gfx_layout lt = { 16, 16, 4, pt, xt, yt, 64 * 8 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, xs, ys, 32 * 8 };
		const rbx_gfx_layout l8 = { 8, 8, 4, p8, x8, y8, 32 * 8 };
		if (rbx_gfx_decode(&s_gfxtile, &lt, tiles, tsize / 2) || rbx_gfx_decode(&s_gfxspr, &ls, spr, ssize / 4) ||
		    (wwf && rbx_gfx_decode(&s_gfxtext, &l8, text, xsize))) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			dd3_exit();
			return -1;
		}
	}
	s_cpu_clock = wwf ? 12000000 : 10000000;
	s_spr_words = wwf ? 0x1000 : 0x800;
	s_pal_n = wwf ? 0x2000 : 0x300;
	s_irq_raster = wwf ? 2 : 5;
	s_irq_vbl = wwf ? 3 : 6;
	s_sprite_xoff = s_bg0_dx = s_bg1_dx[0] = s_bg1_dx[1] = 0;
	if (s_mach == M_WWFB) {   // video_start wwfwfstb
		s_sprite_xoff = 2;
		s_bg0_dx = s_bg1_dx[0] = -4;
		s_bg1_dx[1] = -2;
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	// (stereo to mono: (L + R) / 2) ddragon3 YM 0.25 / OKI 0.75, ctribe 0.60 / 0.40; WrestleFest mono 0.33 / 0.66
	yc.gain[0] = yc.gain[1] = wwf ? 84 : s_mach == M_CTRIBE ? 77 : 32;
	yc.irq = ym_irq;
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		dd3_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, (int)s_pal_n)) {
		rbx_fail("%s: out of memory", g->set);
		dd3_exit();
		return -1;
	}
	oki_init(&s_oki, OKI_CLOCK, 1, oki, osize, wwf ? 169 : s_mach == M_CTRIBE ? 102 : 192);
	memset(s_zmap, 0, sizeof(s_zmap));
	const uint32_t zrom_end = s_mach == M_CTRIBE ? 0x80 : 0xc0;
	for (uint32_t p = 0; p < zrom_end && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	static const char *const names[3][4] = { { "P1_P2", "SYSTEM", "DSW", "P3" }, { "IN0", "IN1", "IN2", "IN3" },
	                                         { "P1", "P2", "P3", "P4" } };
	const int pi = wwf ? 2 : s_mach == M_DD3 ? 0 : 1;
	for (int i = 0; i < 4; i++)
		s_port[i] = rbx_port_find(names[pi][i]);
	s_dsw[0] = rbx_port_find("DSW1");
	s_dsw[1] = rbx_port_find("DSW2");
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);   // (MAME's DIVx / MULx / ADDQ.W An times)
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x7ffff, s_rom, 0x80000, 1, 0);
	m68k_bus_map(0x080000, 0x080fff, s_fgram, sizeof(s_fgram), 1, 1);
	m68k_bus_map(0x1c0000, 0x1c3fff, s_ram, sizeof(s_ram), 1, 1);
	switch (s_mach) {
	case M_DD3:
		m68k_bus_map(0x082000, 0x0827ff, s_bgram, 0x800, 1, 1);
		m68k_bus_map(0x140000, 0x1405ff, s_pal, 0x600, 1, 1);
		m68k_bus_map(0x180000, 0x180fff, s_spriteram, 0x1000, 1, 1);
		break;
	case M_DD3B:
	case M_CTRIBE:
		m68k_bus_map(0x081000, 0x081fff, s_spriteram, 0x1000, 1, 1);
		m68k_bus_map(0x082000, 0x0827ff, s_bgram, 0x800, 1, 1);
		if (s_mach == M_CTRIBE)
			m68k_bus_map(0x082800, 0x082fff, s_ram2, sizeof(s_ram2), 1, 1);
		m68k_bus_map(0x100000, 0x1005ff, s_pal, 0x600, 1, 1);
		break;
	default:
		m68k_bus_map(0x082000, 0x082fff, s_bgram, 0x1000, 1, 1);
		m68k_bus_map(0x0c0000, 0x0c1fff, s_fg0ram, sizeof(s_fg0ram), 1, 0);
		m68k_bus_map(0x0c2000, 0x0c3fff, s_spriteram, 0x2000, 1, 1);
		break;
	}
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	dd3_reset();
	return 0;
}

static void dd3_exit(void)
{
	m68k_set_mame_timing(0);
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gfxtile);
	rbx_gfx_free(&s_gfxspr);
	rbx_gfx_free(&s_gfxtext);
	free(s_snap);
	s_snap = NULL;
	free(s_pal_hp);
	s_pal_hp = NULL;
	free(s_ram_hp);
	s_ram_hp = NULL;
}

static void dd3_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line % 16 == 0) {   // ddragon3_scanline: the raster IRQ every 16 lines
			if (line > 0)
				latch_lines(line);
			else
				s_seg_start = 0;
			m68k_bus_set_line(s_irq_raster, 1);
		}
		if (line == VBL_LINE) {
			latch_lines(line);
			m68k_bus_set_line(s_irq_vbl, 1);
			vblank_capture();
			if (s_mach == M_DD3 || s_mach == M_WWFB)   // (vblank_copy_rising)
				memcpy(s_spritebuf, s_spriteram, s_spr_words * 2);
		}
		m68k_execute(rbx_line_cycles(&s_cpu_frac, s_cpu_clock, FRAME_MHZ, LINES));
		sound_line();
	}
	sound_frame_end();
}

static int dd3_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[3] = { s_cpu_frac, s_z80_frac, s_ym_frac };
	uint16_t regs[8] = { s_vreg, s_fg_sx, s_fg_sy, s_bg_sx, s_bg_sy, s_tilebase, s_flip, s_pri };
	int32_t misc[6] = { s_latch, s_latch_pending, s_ym_irq, (int32_t)s_oki_acc, s_oki_hold, (int32_t)s_oki.bank };
	z80 z = s_z80;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "ram2", s_ram2, sizeof(s_ram2));
	rc |= rbx_state_io(ctx, "fg", s_fgram, sizeof(s_fgram));
	rc |= rbx_state_io(ctx, "bg", s_bgram, sizeof(s_bgram));
	rc |= rbx_state_io(ctx, "fg0", s_fg0ram, sizeof(s_fg0ram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "sprb", s_spritebuf, sizeof(s_spritebuf));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "regs", regs, sizeof(regs));
	rc |= rbx_state_io(ctx, "lines", s_lines, sizeof(s_lines));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= rbx_state_io(ctx, "oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
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
		s_vreg = regs[0];
		s_fg_sx = regs[1];
		s_fg_sy = regs[2];
		s_bg_sx = regs[3];
		s_bg_sy = regs[4];
		s_tilebase = regs[5];
		s_flip = (uint8_t)regs[6];
		s_pri = (uint8_t)regs[7];
		s_latch = (uint8_t)misc[0];
		s_latch_pending = misc[1];
		s_ym_irq = misc[2];
		s_oki_acc = (uint32_t)misc[3];
		s_oki_hold = misc[4];
		s_oki.bank = (uint32_t)misc[5];
		s_seg_start = 0;
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int dd3_save(void *ctx) { return dd3_state(ctx, 0); }
static int dd3_load(void *ctx) { return dd3_state(ctx, 1); }
static int dd3_supported(const char *machine)
{
	return !strcmp(machine, "ddragon3") || !strcmp(machine, "ddragon3b") || !strcmp(machine, "ctribe") ||
	       !strcmp(machine, "wwfwfest") || !strcmp(machine, "wwfwfstb");
}

const rbx_driver rbx_drv_ddragon3 = { "Technos Double Dragon 3", dd3_init, dd3_exit, dd3_reset, dd3_frame, dd3_save,
                                      dd3_load, dd3_supported, dd3_render, NULL };
