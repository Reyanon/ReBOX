/*  Technos Shadow Force. Ported from MAME src/mame/technos/shadfrce.cpp (BSD-3-Clause, copyright-holders: David
    Haywood).
    68000 at 14 MHz: a raster IRQ (level 1) at a programmed line, one every 16 lines (2) and one at line 248 (3), held
    until acknowledged; an 8x8 text layer, two 16x16 tilemaps of 6 bits (BG0 over the opaque BG1, sprites of 5 bits
    behind BG0 when their priority bit is set), a brightness register over the whole palette. MAME draws the screen in
    pieces at the IRQs (update_partial), so the scroll registers are latched per line. Sound: Z80 at 3.58 MHz, YM2151,
    OKI6295 at 1.687 MHz with a 256 KB bank, the latch on the NMI.
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

#define CPU_CLOCK 14000000
#define Z80_CLOCK 3579545
#define YM_CLOCK 3579545
#define OKI_CLOCK (13495200 / 8)
#define FRAME_MHZ 57445          // set_raw(28 MHz / 4, 448, 0, 320, 272, 8, 248)
#define LINES 272
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 320
#define VBL_LINE 248
#define PAL_N 0x4000
#define BLACK PAL_N              // (black_pen: an entry after the palette)

typedef struct { uint16_t sx[2], sy[2]; uint8_t flip, enable; } line_regs;

static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static uint16_t s_bg0ram[0x800];        // 0x100000 .. 0x100fff (two words a tile)
static uint16_t s_ram1[0x800];          // 0x101000 .. 0x101fff
static uint16_t s_bg1ram[0x400];        // 0x102000 .. 0x1027ff
static uint16_t s_ram2[0xc00];          // 0x102800 .. 0x103fff
static uint16_t s_fgram[0x1000];        // 0x140000 .. 0x141fff
static uint16_t s_spriteram[0x1000];    // 0x142000 .. 0x143fff
static uint16_t s_spritebuf[0x1000];
static uint16_t (*s_pal_hp)[PAL_N];           // 0x180000 .. 0x187fff
#define s_pal (*s_pal_hp)
static uint16_t (*s_ram3_hp)[0x8000];         // 0x1f0000 .. 0x1fffff
#define s_ram3 (*s_ram3_hp)
static uint16_t s_sx[2], s_sy[2];
static uint8_t s_flip, s_video_enable, s_irqs_enable, s_raster_irq_enable, s_vblank, s_brt;
static uint16_t s_raster_scanline, s_prev_value;
static line_regs s_lines[LINES];
static int s_seg_start;
static rbx_gfx s_gfxfg, s_gfxspr, s_gfxbg;
static int s_port[8];
static uint64_t s_cpu_frac, s_z80_frac, s_ym_frac;
static uint8_t (*s_pri_buf_hp)[512 * VIS_H];   // (the bitmap's pitch)
#define s_pri_buf (*s_pri_buf_hp)
static struct {
	uint16_t bg0[0x800], bg1[0x400], fg[0x1000], spr[0x1000], pal[PAL_N];
	line_regs lines[LINES];
	uint8_t brt;
} *s_snap;

// ---- sound ----
static z80 s_z80;
static uint8_t s_zram[0x800], s_zram2[0x1000];
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

static uint8_t z80_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xc000) return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0xc800) return s_zram[a - 0xc000];
	if (a == 0xc800 || a == 0xc801) return rbx_ym_read(0, a & 1);
	if (a == 0xd800) return oki_read(&s_oki);
	if (a == 0xe000) {
		s_latch_pending = 0;
		return s_latch;
	}
	if (a >= 0xf000) return s_zram2[a - 0xf000];
	return 0;
}
static void z80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xc800) s_zram[a - 0xc000] = v;
	else if (a == 0xc800 || a == 0xc801) rbx_ym_write(0, a & 1, v);
	else if (a == 0xd800) oki_write(&s_oki, v);
	else if (a == 0xe800) s_oki.bank = (uint32_t)(v & 1) * 0x40000;   // oki_bankswitch_w
	else if (a >= 0xf000) s_zram2[a - 0xf000] = v;
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

// ---- I/O ----
enum { P_P1, P_P2, P_EXTRA, P_OTHER, P_SYSTEM, P_MISC, P_DSW1, P_DSW2 };

static uint16_t input_ports_r(int offset)
{
	uint32_t p[8];
	for (int i = 0; i < 8; i++)
		p[i] = (uint32_t)rbx_port_read(s_port[i]);
	switch (offset) {
	case 0: return (uint16_t)((p[P_P1] & 0xff) | (p[P_DSW2] & 0xc0) << 6 | (p[P_SYSTEM] & 0x0f) << 8);
	case 1: return (uint16_t)((p[P_P2] & 0xff) | (p[P_DSW2] & 0x3f) << 8);
	case 2: return (uint16_t)((p[P_EXTRA] & 0xff) | (p[P_DSW1] & 0x3f) << 8);
	default: return (uint16_t)((p[P_OTHER] & 0xff) | (p[P_DSW1] & 0xc0) << 2 | (p[P_MISC] & 0x38) << 8 | s_vblank << 8);
	}
}

static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x1d0020 && a < 0x1d0028)
		return input_ports_r((int)((a >> 1) & 3));
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
	if (a >= 0x1c0000 && a < 0x1c0008) {
		const int r = (int)((a >> 1) & 3);
		if (r & 1)
			s_sy[r >> 1] = v & 0x1ff;
		else
			s_sx[r >> 1] = v & 0x1ff;
	} else if (a == 0x1c000a)
		s_flip = v & 1;
	else if (a >= 0x1d0000 && a < 0x1d0006)
		m68k_bus_set_line((int)(((a >> 1) & 3) ^ 3), 0);   // irq_ack_w
	else if (a == 0x1d0006) {   // irq_w
		s_irqs_enable = v & 1;
		s_video_enable = (v & 8) != 0;
		if ((~s_prev_value & 4) && (v & 4))
			s_raster_irq_enable = 1;
		if ((s_prev_value & 4) && (~v & 4))
			s_raster_irq_enable = 0;
		s_prev_value = v;
	} else if (a == 0x1d0008)
		s_raster_scanline = v;
	else if (a == 0x1d000c) {
		if (mask & 0xff00) {   // the sound latch
			s_latch = (uint8_t)(v >> 8);
			if (!s_latch_pending)
				z80_gen_nmi(&s_z80);
			s_latch_pending = 1;
		}
		if (mask & 0x00ff)     // screen_brt_w
			s_brt = (uint8_t)v;
	}
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static void latch_lines(int end)   // (update_partial(end - 1))
{
	line_regs r;
	memset(&r, 0, sizeof(r));
	r.sx[0] = s_sx[0];
	r.sx[1] = s_sx[1];
	r.sy[0] = s_sy[0];
	r.sy[1] = s_sy[1];
	r.flip = s_flip;
	r.enable = s_video_enable;
	for (int y = s_seg_start; y < end && y < LINES; y++)
		s_lines[y] = r;
	if (end > s_seg_start)
		s_seg_start = end;
}

// one line of a BG tilemap (32x32 of 16x16, 6 bits): which 0 = BG0 (two words, flips, transparent, priority 1),
// 1 = BG1 (one word, opaque)
static void draw_bg_line(int which, const line_regs *r, int y, uint16_t *dst, uint8_t *pri)
{
	const uint32_t ty = (uint32_t)(r->flip ? 255 - y + r->sy[which] : y + r->sy[which]) & 511;
	const uint32_t row = (ty >> 4) * 32, fy = ty & 15;
	for (int x = 0; x < VIS_W;) {
		const uint32_t tx = (uint32_t)(r->flip ? VIS_W - 1 - x + r->sx[which] : x + r->sx[which]) & 511;
		uint32_t code, color;
		int fx = 0, fyy = 0;
		if (which == 0) {
			const uint16_t *t = &s_snap->bg0[(row + (tx >> 4)) * 2];
			code = t[1] & 0x3fffu;
			color = t[0] & 0x001fu;
			if (color & 0x10)   // (skip hole)
				color ^= 0x30;
			fx = (t[0] >> 6) & 1;
			fyy = (t[0] >> 7) & 1;
		} else {
			const uint16_t t = s_snap->bg1[row + (tx >> 4)];
			code = t & 0x0fffu;
			color = (t >> 12) + 64u;
		}
		code %= s_gfxbg.count;
		const uint8_t *src = s_gfxbg.pix + code * 256 + (fyy ? 15 - fy : fy) * 16;
		const uint16_t c0 = (uint16_t)(0x2000 + color * 64);
		int px = (int)(tx & 15);
		const int step = r->flip ? -1 : 1;
		if (which == 0 && !s_gfxbg.used[code]) {
			x += r->flip ? px + 1 : 16 - px;
			continue;
		}
		for (; px >= 0 && px < 16 && x < VIS_W; px += step, x++) {
			const uint8_t p = src[fx ? 15 - px : px];
			if (which)
				dst[x] = (uint16_t)(c0 + p);
			else if (p) {
				dst[x] = (uint16_t)(c0 + p);
				pri[x] = 1;
			}
		}
	}
}

static void draw_fg_line(int flip, int y, uint16_t *dst)   // 64x32 of 8x8, no scroll
{
	const int ty = (flip ? 255 - y : y) & 255;
	for (int x = 0; x < VIS_W; x++) {
		const int tx = (flip ? VIS_W - 1 - x : x) & 511;
		const uint16_t *t = &s_snap->fg[((ty >> 3) * 64 + (tx >> 3)) * 2];
		const uint32_t code = (uint32_t)((t[0] & 0x00ff) | (t[1] & 0x000f) << 8) % s_gfxfg.count;
		const uint8_t p = s_gfxfg.pix[code * 64 + (ty & 7) * 8 + (tx & 7)];
		if (p)
			dst[x] = (uint16_t)(((t[1] & 0x00f0) >> 4) * 64 + p);
	}
}

static void draw_sprites(int y0, int y1)
{
	uint16_t *bm = rbx_video.bitmap + (y0 - VIS_Y0) * rbx_video.pitch;
	uint8_t *pri = s_pri_buf + (y0 - VIS_Y0) * rbx_video.pitch;
	const int bh = y1 - y0;
	for (int o = 0x1000 - 8; o >= 0; o -= 8) {
		const uint16_t *s = s_snap->spr + o;
		if (!(s[0] & 0x0004))
			continue;
		const int ypos = 0x100 - (((s[0] & 0x0003) << 8) | (s[1] & 0x00ff));
		const int xpos = (((s[4] & 0x0001) << 8) | (s[5] & 0x00ff)) + 1;
		const uint32_t tile = (uint32_t)((s[2] & 0x00ff) << 8 | (s[3] & 0x00ff));
		const int height = ((s[0] & 0x00e0) >> 5) + 1;
		const int flipx = (s[0] & 0x0010) >> 4, flipy = (s[0] & 0x0008) >> 3;
		int pal = s[4] & 0x003e;
		const uint32_t pmask = (s[4] & 0x0040) ? 0x02 : 0x00;
		if (pal & 0x20)   // (skip hole)
			pal ^= 0x60;
		const uint16_t c0 = (uint16_t)(0x1000 + pal * 32);
		for (int h = 0; h < height; h++) {
			const uint32_t code = (tile + (uint32_t)h) % s_gfxspr.count;
			const int sy = ypos - h * 16 - 16 - y0;
			for (int k = 0; k < 4; k++)
				rbx_gfx_prio_transpen(&s_gfxspr, code, c0, flipx, flipy, xpos - (k & 1 ? 0x200 : 0), sy + (k & 2 ? 0x200 : 0),
				                      bm, pri, rbx_video.pitch, VIS_W, bh, pmask);
		}
	}
}

// (the sprites don't depend on a piece's registers and the pieces are lines apart: the BG layers of every line, the
// sprites once, then the text layer, as MAME's pieces in turn; a disabled piece black over all)
#define BANDS 8
static void bg_band(int band)
{
	for (int l = VIS_Y0 + VIS_H * band / BANDS; l < VIS_Y0 + VIS_H * (band + 1) / BANDS; l++) {
		const line_regs *r = &s_snap->lines[l];
		uint16_t *d = rbx_video.bitmap + (l - VIS_Y0) * rbx_video.pitch;
		uint8_t *pr = s_pri_buf + (l - VIS_Y0) * rbx_video.pitch;
		memset(pr, 0, VIS_W);
		if (r->enable) {
			draw_bg_line(1, r, l, d, pr);
			draw_bg_line(0, r, l, d, pr);
		}
	}
}
static void fg_band(int band)
{
	for (int l = VIS_Y0 + VIS_H * band / BANDS; l < VIS_Y0 + VIS_H * (band + 1) / BANDS; l++) {
		const line_regs *r = &s_snap->lines[l];
		uint16_t *d = rbx_video.bitmap + (l - VIS_Y0) * rbx_video.pitch;
		if (r->enable)
			draw_fg_line(r->flip, l, d);
		else
			for (int x = 0; x < VIS_W; x++)
				d[x] = BLACK;
	}
}
static decoband_t s_bg_bands = { 0, 0, 0, 0, BANDS, bg_band }, s_fg_bands = { 0, 0, 0, 0, BANDS, fg_band };
static int sf_help(int helper) { return decoband_help(&s_bg_bands, helper) || decoband_help(&s_fg_bands, helper); }

static void sf_render(void)
{
	const uint32_t brt = s_snap->brt;
	for (int i = 0; i < PAL_N; i++) {   // xBGR_555, set_pen_contrast(brt / 255)
		const uint16_t p = s_snap->pal[i];
		uint32_t r = p & 31, g = (p >> 5) & 31, b = (p >> 10) & 31;
		r = ((r << 3 | r >> 2) * brt + 127) / 255;
		g = ((g << 3 | g >> 2) * brt + 127) / 255;
		b = ((b << 3 | b >> 2) * brt + 127) / 255;
		rbx_video.palette[i] = r << 16 | g << 8 | b;
	}
	rbx_video.palette[BLACK] = 0;
	if (rbx_video.pitch > 512)
		return;
	decoband_run(&s_bg_bands);
	draw_sprites(VIS_Y0, VIS_Y0 + VIS_H);
	decoband_run(&s_fg_bands);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->bg0, s_bg0ram, sizeof(s_bg0ram));
	memcpy(s_snap->bg1, s_bg1ram, sizeof(s_bg1ram));
	memcpy(s_snap->fg, s_fgram, sizeof(s_fgram));
	memcpy(s_snap->spr, s_spritebuf, sizeof(s_spritebuf));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	memcpy(s_snap->lines, s_lines, sizeof(s_lines));
	s_snap->brt = s_brt;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void sf_reset(void)
{
	memset(s_bg0ram, 0, sizeof(s_bg0ram));
	memset(s_ram1, 0, sizeof(s_ram1));
	memset(s_bg1ram, 0, sizeof(s_bg1ram));
	memset(s_ram2, 0, sizeof(s_ram2));
	memset(s_fgram, 0, sizeof(s_fgram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_spritebuf, 0, sizeof(s_spritebuf));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_ram3, 0, sizeof(s_ram3));
	memset(s_lines, 0, sizeof(s_lines));
	s_sx[0] = s_sx[1] = s_sy[0] = s_sy[1] = 0;
	s_flip = s_video_enable = s_irqs_enable = s_raster_irq_enable = s_vblank = 0;
	s_brt = 0xff;   // (pens at full contrast until written)
	s_raster_scanline = s_prev_value = 0;
	s_seg_start = 0;
	s_latch = 0;
	s_latch_pending = 0;
	s_ym_irq = 0;
	memset(s_zram, 0, sizeof(s_zram));
	memset(s_zram2, 0, sizeof(s_zram2));
	z80_start();
	rbx_ym_reset(0);
	oki_reset(&s_oki);
	s_oki.bank = 0;
	s_oki_acc = 0;
	s_oki_hold = 0;
	s_mix_n = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_cpu_frac = s_z80_frac = s_ym_frac = 0;
}

static void sf_exit(void);

static int sf_init(const rbx_game *g)
{
	if (!s_pal_hp && !(s_pal_hp = calloc(1, sizeof(*s_pal_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_pri_buf_hp && !(s_pri_buf_hp = calloc(1, sizeof(*s_pri_buf_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ram3_hp && !(s_ram3_hp = calloc(1, sizeof(*s_ram3_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t csize, ssize, tsize, osize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *chars = rbx_region("chars", &csize), *spr = rbx_region("sprites", &ssize), *tiles = rbx_region("tiles", &tsize);
	uint8_t *oki = rbx_region("oki", &osize);
	if (!s_rom || s_rom_size < 0x100000 || !s_zrom || !chars || !spr || !tiles || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	{	// fg8x8x4_layout, sp16x16x5_layout (RGN_FRAC(1,5)), bg16x16x6_layout (RGN_FRAC(1,3))
		static const uint32_t x8[8] = { 1, 0, 8 * 8 + 1, 8 * 8 + 0, 16 * 8 + 1, 16 * 8 + 0, 24 * 8 + 1, 24 * 8 + 0 };
		static uint32_t y8[8], xs[16], ys[16], xb[16], yb[16];
		for (int i = 0; i < 8; i++) {
			y8[i] = (uint32_t)(i * 8);
			xs[i] = (uint32_t)i;
			xs[8 + i] = (uint32_t)(16 * 8 + i);
			xb[i] = (uint32_t)i;
			xb[8 + i] = (uint32_t)(16 * 16 + i);
		}
		for (int i = 0; i < 16; i++) {
			ys[i] = (uint32_t)(i * 8);
			yb[i] = (uint32_t)(i * 16);
		}
		const uint32_t p8[4] = { 0, 2, 4, 6 };
		const uint32_t f5 = ssize / 5 * 8, f3 = tsize / 3 * 8;
		const uint32_t ps[5] = { f5 * 4, f5 * 3, f5 * 2, f5, 0 };
		const uint32_t pb[6] = { 8, 0, f3 + 8, f3, f3 * 2 + 8, f3 * 2 };
		const rbx_gfx_layout l8 = { 8, 8, 4, p8, x8, y8, 4 * 8 * 8 };
		const rbx_gfx_layout ls = { 16, 16, 5, ps, xs, ys, 16 * 16 };
		const rbx_gfx_layout lb = { 16, 16, 6, pb, xb, yb, 2 * 16 * 16 };
		if (rbx_gfx_decode(&s_gfxfg, &l8, chars, csize) || rbx_gfx_decode(&s_gfxspr, &ls, spr, ssize / 5) ||
		    rbx_gfx_decode(&s_gfxbg, &lb, tiles, tsize / 3)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			sf_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 64;   // (0.50 to both sides: (L + R) / 2)
	yc.irq = ym_irq;
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		sf_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, PAL_N + 1)) {
		rbx_fail("%s: out of memory", g->set);
		sf_exit();
		return -1;
	}
	oki_init(&s_oki, OKI_CLOCK, 1, oki, osize, 128);
	memset(s_zmap, 0, sizeof(s_zmap));
	for (uint32_t p = 0; p < 0xc0 && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	static const char *const names[8] = { "P1", "P2", "EXTRA", "OTHER", "SYSTEM", "MISC", "DSW1", "DSW2" };
	for (int i = 0; i < 8; i++)
		s_port[i] = rbx_port_find(names[i]);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);   // (MAME's DIVx / MULx / ADDQ.W An times)
	m68k_bus_reset_map();
	m68k_bus_map(0, 0xfffff, s_rom, 0x100000, 1, 0);
	m68k_bus_map(0x100000, 0x100fff, s_bg0ram, sizeof(s_bg0ram), 1, 1);
	m68k_bus_map(0x101000, 0x101fff, s_ram1, sizeof(s_ram1), 1, 1);
	m68k_bus_map(0x102000, 0x1027ff, s_bg1ram, sizeof(s_bg1ram), 1, 1);
	m68k_bus_map(0x102800, 0x103fff, s_ram2, sizeof(s_ram2), 1, 1);
	m68k_bus_map(0x140000, 0x141fff, s_fgram, sizeof(s_fgram), 1, 1);
	m68k_bus_map(0x142000, 0x143fff, s_spriteram, sizeof(s_spriteram), 1, 1);
	m68k_bus_map(0x180000, 0x187fff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus_map(0x1f0000, 0x1fffff, s_ram3, sizeof(s_ram3), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	sf_reset();
	return 0;
}

static void sf_exit(void)
{
	m68k_set_mame_timing(0);
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gfxfg);
	rbx_gfx_free(&s_gfxspr);
	rbx_gfx_free(&s_gfxbg);
	free(s_snap);
	s_snap = NULL;
	free(s_pal_hp);
	s_pal_hp = NULL;
	free(s_pri_buf_hp);
	s_pri_buf_hp = NULL;
	free(s_ram3_hp);
	s_ram3_hp = NULL;
}

static void sf_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {   // scanline()
		if (line == 0) {
			s_vblank = 0;
			s_seg_start = 0;
		} else if (line == VBL_LINE - 1)   // (MAME's hack: one line early)
			s_vblank = 4;
		if (s_raster_irq_enable && line == s_raster_scanline) {
			s_raster_scanline = (uint16_t)((s_raster_scanline + 1) % 240);
			if (s_raster_scanline > 0)
				latch_lines(s_raster_scanline);
			m68k_bus_set_line(1, 1);
		}
		if (s_irqs_enable && line % 16 == 0) {
			if (line > 0)
				latch_lines(line);
			m68k_bus_set_line(2, 1);
		}
		if (line == VBL_LINE) {   // (the end of the visible area: update_now at the vblank)
			latch_lines(line);
			if (s_irqs_enable)
				m68k_bus_set_line(3, 1);
			vblank_capture();
			memcpy(s_spritebuf, s_spriteram, sizeof(s_spriteram));   // (vblank_copy_rising)
		}
		m68k_execute(rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		sound_line();
	}
	sound_frame_end();
}

static int sf_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[3] = { s_cpu_frac, s_z80_frac, s_ym_frac };
	uint16_t regs[12] = { s_sx[0], s_sx[1], s_sy[0], s_sy[1], s_flip, s_video_enable, s_irqs_enable, s_raster_irq_enable,
	                      s_vblank, s_brt, s_raster_scanline, s_prev_value };
	int32_t misc[6] = { s_latch, s_latch_pending, s_ym_irq, (int32_t)s_oki_acc, s_oki_hold, (int32_t)s_oki.bank };
	z80 z = s_z80;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "bg0", s_bg0ram, sizeof(s_bg0ram));
	rc |= rbx_state_io(ctx, "ram1", s_ram1, sizeof(s_ram1));
	rc |= rbx_state_io(ctx, "bg1", s_bg1ram, sizeof(s_bg1ram));
	rc |= rbx_state_io(ctx, "ram2", s_ram2, sizeof(s_ram2));
	rc |= rbx_state_io(ctx, "fg", s_fgram, sizeof(s_fgram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "sprb", s_spritebuf, sizeof(s_spritebuf));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "ram3", s_ram3, sizeof(s_ram3));
	rc |= rbx_state_io(ctx, "regs", regs, sizeof(regs));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= rbx_state_io(ctx, "zram2", s_zram2, sizeof(s_zram2));
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
		s_sx[0] = regs[0];
		s_sx[1] = regs[1];
		s_sy[0] = regs[2];
		s_sy[1] = regs[3];
		s_flip = (uint8_t)regs[4];
		s_video_enable = (uint8_t)regs[5];
		s_irqs_enable = (uint8_t)regs[6];
		s_raster_irq_enable = (uint8_t)regs[7];
		s_vblank = (uint8_t)regs[8];
		s_brt = (uint8_t)regs[9];
		s_raster_scanline = regs[10];
		s_prev_value = regs[11];
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

static int sf_save(void *ctx) { return sf_state(ctx, 0); }
static int sf_load(void *ctx) { return sf_state(ctx, 1); }
static int sf_supported(const char *machine) { return !strcmp(machine, "shadfrce"); }

const rbx_driver rbx_drv_shadfrce = { "Technos Shadow Force", sf_init, sf_exit, sf_reset, sf_frame, sf_save, sf_load,
                                      sf_supported, sf_render, sf_help };
