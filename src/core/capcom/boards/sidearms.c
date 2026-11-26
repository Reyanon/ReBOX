/*  Capcom Side Arms and Philko's boards on its design (Turtle Ship, Dyger, Twin Falcons / Whizz). Ported from MAME
    src/mame/capcom/sidearms.cpp, sidearms_v.cpp (BSD-3-Clause, copyright-holders: Paul Leaman, Curt Coder).
    Z80 at 8 MHz (IRQ at lines 112 and 240), Z80 sound CPU at 4 MHz with two YM2203 (Whizz: a YM2151, the sound
    latch's IRQ). Video: a 4096x4096 background of 32x32 tiles from ROM, a 512x512 text layer, 16x16 sprites copied
    on request (86S105 DMA), Side Arms' starfield; xBRG 444 palette RAM.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../../video/decoband.h"
#include "cap8.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 61035          // set_raw(16 MHz / 2, 512, 64, 448, 256, 16, 240)
#define LINES 256
#define VIS_X0 64
#define VIS_W 384
#define VIS_Y0 16
#define VIS_H 224
#define MAIN_CLOCK 8000000
#define SND_CLOCK 4000000
#define YM_CLOCK 4000000

enum { G_SIDEARMS, G_TURTSHIP, G_DYGER, G_WHIZZ };

static int s_game;
static uint8_t *s_rom, *s_zrom, *s_tilerom, *s_starrom;
static uint32_t s_rom_size, s_zrom_size, s_tilerom_size;
static rbx_gfx s_gchar, s_gtile, s_gspr;
static int s_port[8], s_vbl_port;
static uint32_t s_vbl_mask;
static z80 s_main, s_snd;
static const uint8_t *s_mmap[256], *s_zmap[256];
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
// the memory map's bases (sidearms_map / turtship_map / whizz_map)
static uint16_t s_a_pal, s_a_io, s_a_vram, s_a_ram, s_a_spr;

static struct {
	uint8_t ram[0x1000], spr[0x1000], sprbuf[0x1000], vram[0x800], cram[0x800], pal[0x400], palext[0x400];
	uint8_t zram[0x800];
	uint8_t bank, latch, bgx[2], bgy[2];
	uint8_t bgon, objon, staron, charon, flipon;
	uint32_t hflop, hcount, vcount;
	int ym_irq, latch_pending, main_irq, line;
	uint64_t main_frac, snd_frac, ym_frac;
	uint8_t state_end;
} *t;   // (calloc in init: ADR-040)

static struct {
	uint8_t spr[0x1000], vram[0x800], cram[0x800], pal[0x400], palext[0x400];
	uint16_t sx, sy;
	uint8_t bgon, objon, staron, charon, flipon;
	uint32_t hflop, hcount, vcount;
} *s_snap;

// ---- sound ----
static void ym_irq(int state) { t->ym_irq = state; }

static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_zrom[a];
	if (s_game == G_WHIZZ)
		return a >= 0xf800 ? t->zram[a - 0xf800] : 0;
	if (a >= 0xc000 && a < 0xc800) return t->zram[a - 0xc000];
	if (a == 0xd000) return t->latch;
	if (a == 0xf000 || a == 0xf001) return rbx_ym_read(0, a & 1);
	if (a == 0xf002 || a == 0xf003) return rbx_ym_read(1, a & 1);
	return 0;
}
static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_game == G_WHIZZ) {
		if (a >= 0xf800)
			t->zram[a - 0xf800] = v;
		return;
	}
	if (a >= 0xc000 && a < 0xc800) t->zram[a - 0xc000] = v;
	else if (a == 0xf000 || a == 0xf001) rbx_ym_write(0, a & 1, v);
	else if (a == 0xf002 || a == 0xf003) rbx_ym_write(1, a & 1, v);
}
static uint8_t snd_in(z80 *z, uint8_t port)   // (whizz_io_map)
{
	(void)z;
	if (s_game != G_WHIZZ) return 0xff;
	if (port <= 1) return rbx_ym_read(0, port & 1);
	if (port == 0xc0) {
		t->latch_pending = 0;
		return t->latch;
	}
	return 0xff;
}
static void snd_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	if (s_game == G_WHIZZ && port <= 1)
		rbx_ym_write(0, port & 1, v);
}

static void snd_start(void)
{
	z80_init(&s_snd);
	s_snd.read_byte = snd_read;
	s_snd.write_byte = snd_write;
	s_snd.port_in = snd_in;
	s_snd.port_out = snd_out;
	s_snd.read_map = s_zmap;
}

// ---- main CPU ----
static void map_bank(void)
{
	for (int p = 0; p < 0x40; p++)
		s_mmap[0x80 + p] = s_rom + rbx_wrap((0x8000 + (uint32_t)t->bank * 0x4000 + p * 256), s_rom_size);
}

static void control_w(uint8_t v)
{
	if (v & 0x10)   // (pulse the sound CPU's reset)
		snd_start();
	if (t->staron != (v & 0x20)) {
		t->staron = v & 0x20;
		t->hflop = 1;
		t->hcount = t->vcount = 0;
	}
	t->charon = v & 0x40;
	t->flipon = v & 0x80;
}

static uint8_t io_read(int o)
{
	if (s_game == G_TURTSHIP || s_game == G_DYGER) {   // turtship_ports_r: bit `o` of the five ports
		if (o > 7) return 0xff;
		int r = 0;
		for (int i = 0; i < 5; i++)
			r |= (int)((rbx_port_read(s_port[i]) >> o) & 1) << i;
		return (uint8_t)r;
	}
	if (s_game == G_WHIZZ)
		return o <= 7 ? (uint8_t)rbx_port_read(s_port[o]) : 0xff;
	return o <= 5 ? (uint8_t)rbx_port_read(s_port[o]) : 0xff;
}

static void io_write(int o, uint8_t v)
{
	switch (o) {
	case 0x0: t->latch = v; t->latch_pending = 1; break;
	case 0x1: t->bank = s_game == G_WHIZZ ? (uint8_t)(((v >> 6) & 1) << 1 | ((v >> 7) & 1)) : (uint8_t)(v & 7); map_bank(); break;
	case 0x2: memcpy(t->sprbuf, t->spr, sizeof(t->spr)); break;   // (86S105 DMA)
	case 0x4: control_w(v); break;
	case 0x5:
		if (s_game != G_WHIZZ) {   // star_scrollx_w
			const uint32_t last = t->hcount;
			t->hcount = (t->hcount + 1) & 0x1ff;
			if (t->hcount & ~last & 0x100)
				t->hflop ^= 1;
		}
		break;
	case 0x6:
		if (s_game != G_WHIZZ)
			t->vcount = (t->vcount + 1) & 0xff;
		break;
	case 0x8: case 0x9: t->bgx[o & 1] = v; break;
	case 0xa: case 0xb: t->bgy[o & 1] = v; break;
	case 0xc: t->bgon = v & 1; t->objon = v & 2; break;
	}
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_rom[a];
	if (a < 0xc000) return s_rom[rbx_wrap((0x8000 + (uint32_t)t->bank * 0x4000 + (a - 0x8000)), s_rom_size)];
	if (a >= s_a_io && a < s_a_io + 0x800) return io_read(a - s_a_io);
	if (a >= s_a_vram && a < s_a_vram + 0x800) return t->vram[a - s_a_vram];
	if (a >= s_a_vram + 0x800 && a < s_a_vram + 0x1000) return t->cram[a - s_a_vram - 0x800];
	if (a >= s_a_ram && a < s_a_ram + 0x1000) return t->ram[a - s_a_ram];
	if (a >= s_a_spr && a < s_a_spr + 0x1000) return t->spr[a - s_a_spr];
	return 0xff;   // (unmap_value_high: the palette is write-only)
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xc000) return;
	if (s_game == G_WHIZZ && (a == 0xe805 || a == 0xe806)) {   // (the star counters over the RAM)
		io_write(a - 0xe800, v);
		return;
	}
	if (a >= s_a_pal && a < s_a_pal + 0x400) t->pal[a - s_a_pal] = v;
	else if (a >= s_a_pal + 0x400 && a < s_a_pal + 0x800) t->palext[a - s_a_pal - 0x400] = v;
	else if (a >= s_a_io && a < s_a_io + 0x10) io_write(a - s_a_io, v);
	else if (a >= s_a_vram && a < s_a_vram + 0x800) t->vram[a - s_a_vram] = v;
	else if (a >= s_a_vram + 0x800 && a < s_a_vram + 0x1000) t->cram[a - s_a_vram - 0x800] = v;
	else if (a >= s_a_ram && a < s_a_ram + 0x1000) t->ram[a - s_a_ram] = v;
	else if (a >= s_a_spr && a < s_a_spr + 0x1000) t->spr[a - s_a_spr] = v;
}

static void main_pages(void)
{
	memset(s_mmap, 0, sizeof(s_mmap));
	for (int p = 0; p < 0x80; p++)
		s_mmap[p] = s_rom + p * 256;
	map_bank();
	for (int p = 0; p < 0x10; p++) {
		s_mmap[(s_a_ram >> 8) + p] = t->ram + p * 256;
		s_mmap[(s_a_spr >> 8) + p] = t->spr + p * 256;
	}
	for (int p = 0; p < 8; p++) {
		s_mmap[(s_a_vram >> 8) + p] = t->vram + p * 256;
		s_mmap[(s_a_vram >> 8) + 8 + p] = t->cram + p * 256;
	}
	if (s_game == G_WHIZZ)
		s_mmap[0xe8] = NULL;   // (e805 / e806: reads still the RAM; through main_read)
}

static uint8_t z_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

// ---- video ----
// the bitmap rows [r0, r1)
static void draw_starfield(int r0, int r1)
{
	for (int y = r0; y < r1; y++)
		memset(rbx_video.bitmap + y * rbx_video.pitch, 0, VIS_W * 2);
	if (s_game != G_SIDEARMS || !s_snap->staron)
		return;
	const uint32_t hflop = s_snap->hflop, vcount = s_snap->vcount, hcount = s_snap->hcount & 0xff;
	const int flip = s_snap->flipon != 0;
	for (int r = r0; r < r1; r++) {
		const int y = flip ? 239 - r : r + VIS_Y0;
		uint16_t *line = rbx_video.bitmap + r * rbx_video.pitch;
		uint32_t hadd = (hcount + 64) & ~0x1fu, vadd = vcount + (uint32_t)y;
		uint32_t i = (vadd << 4) & 0xff0;
		i |= (hflop ^ (hadd >> 8)) << 3;
		i |= (hadd >> 5) & 7;
		uint32_t latch = s_starrom[i + 0x3000];
		hadd = hcount + 63;
		for (int x = 64; x < 448; x++) {
			i = hadd;
			hadd = hcount + (uint32_t)(x & 0xff);
			vadd = vcount + (uint32_t)y;
			if (!((vadd ^ (uint32_t)(x >> 3)) & 4)) continue;
			if ((vadd | (hadd >> 1)) & 2) continue;
			if (!(~i & 0x1f)) {
				i = (vadd << 4) & 0xff0;
				i |= (hflop ^ (hadd >> 8)) << 3;
				i |= (hadd >> 5) & 7;
				latch = s_starrom[i + 0x3000];
			}
			if (~((latch ^ hadd) ^ 1) & 0x1f) continue;
			line[flip ? 447 - x : x - VIS_X0] = (uint16_t)((latch >> 5) | 0x378);
		}
	}
}

// (the tile's lookups once per tile: the pixels are the same as per-pixel lookups)
static void draw_bg(int y0, int y1)
{
	const int flip = s_snap->flipon != 0, philko = s_game != G_SIDEARMS;
	for (int y = y0; y < y1; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y + s_snap->sy : y + s_snap->sy) & 4095;
		uint32_t last = ~0u, fx = 0;
		const uint8_t *src = NULL;
		uint16_t base = 0;
		for (int x = 0; x < VIS_W; x++) {
			const int xa = x + VIS_X0;
			const uint32_t tx = (uint32_t)(flip ? 511 - xa + s_snap->sx : xa + s_snap->sx) & 4095;
			if ((tx >> 5) != last) {
				last = tx >> 5;
				const uint32_t col = tx >> 5, row = ty >> 5;
				uint32_t off = ((row << 7) + col) << 1;   // (tilemap_scan)
				off = ((off & 0xf801) | ((off & 0x0700) >> 7) | ((off & 0x00fe) << 3)) & 0x7fff;
				const uint8_t attr = s_tilerom[rbx_wrap((off + 1), s_tilerom_size)];
				uint32_t code = s_tilerom[rbx_wrap(off, s_tilerom_size)];
				int color;
				if (philko) {
					code |= (uint32_t)(((attr >> 6) & 2) | (attr & 1)) * 0x100;
					color = (attr >> 3) & 0x0f;
				} else {
					code |= (uint32_t)(attr << 8) & 0x100;
					color = (attr >> 3) & 0x1f;
				}
				code %= s_gtile.count;
				const uint32_t py = (attr & 4) ? 31 - (ty & 31) : ty & 31;
				src = s_gtile.pix + code * 1024 + py * 32;
				fx = (attr & 2) ? 31 : 0;
				base = (uint16_t)(color * 16);
			}
			const uint8_t p = src[(tx & 31) ^ fx];
			if (philko || p != 15)
				d[x] = (uint16_t)(base + p);
		}
	}
}

static void draw_chars(int y0, int y1)
{
	const int flip = s_snap->flipon != 0;
	for (int y = y0; y < y1; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y : y) & 511;
		uint32_t last = ~0u;
		const uint8_t *src = NULL;
		uint16_t base = 0;
		for (int x = 0; x < VIS_W; x++) {
			const int xa = x + VIS_X0;
			const uint32_t tx = (uint32_t)(flip ? 511 - xa : xa) & 511;
			if ((tx >> 3) != last) {
				last = tx >> 3;
				const uint32_t ti = ((ty >> 3) * 64 + (tx >> 3)) & 0x7ff;
				const uint8_t attr = s_snap->cram[ti];
				const uint32_t code = rbx_wrap((uint32_t)(s_snap->vram[ti] + ((attr << 2) & 0x300)), s_gchar.count);
				src = s_gchar.pix + code * 64 + (ty & 7) * 8;
				base = (uint16_t)(768 + (attr & 0x3f) * 4);
			}
			const uint8_t p = src[tx & 7];
			if (p != 3)
				d[x] = (uint16_t)(base + p);
		}
	}
}

static void draw_sprites_region(int start, int end)
{
	const int flip = s_snap->flipon != 0;
	for (int o = end - 32; o >= start; o -= 32) {
		const uint8_t *s = &s_snap->spr[o];
		int y = s[2];
		if (!y || s[5] == 0xc3)
			continue;
		const int attr = s[1];
		int x = s[3] + ((attr << 4) & 0x100);
		if (flip) {
			x = 62 * 8 - x;
			y = 30 * 8 - y;
		}
		cap8_transpen(&s_gspr, (uint32_t)(s[0] + ((attr << 3) & 0x700)), (uint16_t)(512 + (attr & 0xf) * 16), flip, flip, x, y, 15,
		              VIS_Y0, VIS_Y0 + VIS_H, VIS_Y0, VIS_X0, VIS_W);
	}
}

// the frame in bands of lines drawn in parallel (decoband.h): the stars and the background, the sprites once, the text
#define BANDS 8
static int s_phase;
static void sa_band(int band)
{
	const int r0 = VIS_H * band / BANDS, r1 = VIS_H * (band + 1) / BANDS;
	if (s_phase == 0) {
		draw_starfield(r0, r1);
		if (s_snap->bgon)
			draw_bg(r0 + VIS_Y0, r1 + VIS_Y0);
	} else if (s_snap->charon)
		draw_chars(r0 + VIS_Y0, r1 + VIS_Y0);
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, sa_band };
static int sa_help(int helper) { return decoband_help(&s_bands, helper); }

static void sa_render(void)
{
	for (int i = 0; i < 1024; i++) {   // xBRG_444 (the ext byte high)
		const uint32_t w = (uint32_t)(s_snap->palext[i] << 8 | s_snap->pal[i]);
		rbx_video.palette[i] = ((w >> 4) & 15) * 17 << 16 | (w & 15) * 17 << 8 | ((w >> 8) & 15) * 17;
	}
	s_phase = 0;
	decoband_run(&s_bands);
	if (s_snap->objon) {
		if (s_game == G_DYGER || s_game == G_WHIZZ)
			draw_sprites_region(0x0000, 0x1000);
		else {
			draw_sprites_region(0x0700, 0x0800);
			draw_sprites_region(0x0e00, 0x1000);
			draw_sprites_region(0x0800, 0x0f00);
			draw_sprites_region(0x0000, 0x0700);
		}
	}
	s_phase = 1;
	decoband_run(&s_bands);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spr, t->sprbuf, sizeof(t->sprbuf));
	memcpy(s_snap->vram, t->vram, sizeof(t->vram));
	memcpy(s_snap->cram, t->cram, sizeof(t->cram));
	memcpy(s_snap->pal, t->pal, sizeof(t->pal));
	memcpy(s_snap->palext, t->palext, sizeof(t->palext));
	s_snap->sx = (uint16_t)(t->bgx[0] + ((t->bgx[1] << 8) & 0xf00));
	s_snap->sy = (uint16_t)(t->bgy[0] + ((t->bgy[1] << 8) & 0xf00));
	s_snap->bgon = t->bgon;
	s_snap->objon = t->objon;
	s_snap->staron = t->staron;
	s_snap->charon = t->charon;
	s_snap->flipon = t->flipon;
	s_snap->hflop = t->hflop;
	s_snap->hcount = t->hcount;
	s_snap->vcount = t->vcount;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void sa_reset(void)
{
	memset(t, 0, offsetof(__typeof__(*t), state_end));
	t->hflop = 1;   // (video_start)
	main_pages();
	z80_init(&s_main);
	s_main.read_byte = main_read;
	s_main.write_byte = main_write;
	s_main.port_in = z_in;
	s_main.port_out = z_out;
	s_main.read_map = s_mmap;
	snd_start();
	rbx_ym_reset(0);
	if (s_game != G_WHIZZ)
		rbx_ym_reset(1);
	s_mix_n = 0;
}

static void sa_exit(void);

static int sa_init(const rbx_game *g)
{
	free(t);
	t = calloc(1, sizeof(*t));
	if (!t) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	const char *init = strchr(g->machine, ':');
	init = init ? init + 1 : "";
	s_game = !strcmp(init, "turtship") ? G_TURTSHIP : !strcmp(init, "dyger") ? G_DYGER : !strcmp(init, "whizz") ? G_WHIZZ : G_SIDEARMS;
	if (s_game == G_TURTSHIP || s_game == G_DYGER) {
		s_a_ram = 0xc000; s_a_spr = 0xd000; s_a_pal = 0xe000; s_a_io = 0xe800; s_a_vram = 0xf000;
	} else {
		s_a_pal = 0xc000; s_a_io = 0xc800; s_a_vram = 0xd000; s_a_ram = 0xe000; s_a_spr = 0xf000;
	}
	uint32_t s1, s2, s3, ss = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_tilerom = rbx_region("gfx4", &s_tilerom_size);
	s_starrom = s_game == G_SIDEARMS ? rbx_region("user1", &ss) : NULL;
	uint8_t *chars = rbx_region("gfx1", &s1), *tiles = rbx_region("gfx2", &s2), *spr = rbx_region("gfx3", &s3);
	if (!s_rom || s_rom_size < 0x10000 || !s_zrom || s_zrom_size < 0x8000 || !s_tilerom || s_tilerom_size < 0x8000 ||
	    (s_game == G_SIDEARMS && (!s_starrom || ss < 0x4000)) || !chars || !tiles || !spr) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{	// charlayout, tilelayout (32x32, RGN_FRAC(1,2)), spritelayout (RGN_FRAC(1,2))
		static const uint32_t pc[2] = { 4, 0 }, xc[8] = { 0, 1, 2, 3, 8, 9, 10, 11 };
		static const uint32_t yc[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
		static const uint32_t xs[16] = { 0, 1, 2, 3, 8, 9, 10, 11, 256, 257, 258, 259, 264, 265, 266, 267 };
		static uint32_t ys[16], xt[32], yt[32];
		for (int i = 0; i < 16; i++)
			ys[i] = (uint32_t)i * 16;
		for (int i = 0; i < 32; i++) {
			xt[i] = (uint32_t)(i >> 3) * 512 + (uint32_t)((i & 4) ? 8 + (i & 3) : (i & 3));
			yt[i] = (uint32_t)i * 16;
		}
		const uint32_t h2 = s2 / 2 * 8, h3 = s3 / 2 * 8;
		const uint32_t p2[4] = { h2 + 4, h2, 4, 0 }, p3[4] = { h3 + 4, h3, 4, 0 };
		const rbx_gfx_layout lc = { 8, 8, 2, pc, xc, yc, 128 };
		const rbx_gfx_layout lt = { 32, 32, 4, p2, xt, yt, 2048 }, ls = { 16, 16, 4, p3, xs, ys, 512 };
		if (rbx_gfx_decode(&s_gchar, &lc, chars, s1) || rbx_gfx_decode(&s_gtile, &lt, tiles, s2 / 2) ||
		    rbx_gfx_decode(&s_gspr, &ls, spr, s3 / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			sa_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	int rc = !s_snap;
	if (s_game == G_WHIZZ) {   // YM2151 1.0 (both outputs, mono)
		yc.type = RBX_YM2151;
		yc.clock = YM_CLOCK;
		yc.gain[0] = yc.gain[1] = 256;
		rc = rc || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	} else {
		yc.type = RBX_YM2203;
		yc.clock = YM_CLOCK;
		yc.gain[0] = yc.gain[1] = yc.gain[2] = 38;   // (0.15 SSG, 0.25 FM)
		yc.gain[3] = 64;
		yc.irq = ym_irq;
		rc = rc || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
		yc.irq = NULL;
		rc = rc || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		sa_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 1024)) {
		rbx_fail("%s: out of memory", g->set);
		sa_exit();
		return -1;
	}
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x80; p++)
		s_zmap[p] = s_zrom + p * 256;
	if (s_game == G_WHIZZ) {
		static const char *const names[8] = { "DSW0", "DSW1", "DSW2", "IN0", "IN1", "IN2", "IN3", "IN4" };
		for (int i = 0; i < 8; i++)
			s_port[i] = rbx_port_find(names[i]);
		s_vbl_port = s_port[6];
		s_vbl_mask = 0x10;
	} else {
		static const char *const names[6] = { "SYSTEM", "P1", "P2", "DSW0", "DSW1", "DSW2" };
		for (int i = 0; i < 6; i++)
			s_port[i] = rbx_port_find(names[i]);
		s_vbl_port = s_port[0];
		s_vbl_mask = 0x08;
	}
	sa_reset();
	return 0;
}

static void sa_exit(void)
{
	rbx_video_wait_idle();
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gchar);
	rbx_gfx_free(&s_gtile);
	rbx_gfx_free(&s_gspr);
	free(s_snap);
	s_snap = NULL;
	free(t);
	t = NULL;
}

static void sa_frame(void)
{
	static int32_t tmp[1024];
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		t->line = line;
		if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		if (line == 112 || line == 240) {   // (scanline timer: HOLD_LINE)
			s_main.int_data = 0xff;
			t->main_irq = 1;
		}
		rbx_port_custom(s_vbl_port, s_vbl_mask, (line < VIS_Y0 || line >= VIS_Y0 + VIS_H) ? s_vbl_mask : 0);
		z80_run(&s_main, s_main.cyc + (unsigned long)rbx_line_cycles(&t->main_frac, MAIN_CLOCK, FRAME_MHZ, LINES), &t->main_irq, 1);
		z80_run(&s_snd, s_snd.cyc + (unsigned long)rbx_line_cycles(&t->snd_frac, SND_CLOCK, FRAME_MHZ, LINES),
		        s_game == G_WHIZZ ? &t->latch_pending : &t->ym_irq, 0);
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&t->ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
		s_mix_n += rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
		if (s_game != G_WHIZZ)
			rbx_ym_advance(1, clocks, tmp, 1024);
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	if (s_game != G_WHIZZ)
		rbx_ym_async_take(1, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int sa_state(void *ctx, int load)
{
	int rc = 0;
	z80 c = s_main, zs = s_snd;
	rc |= rbx_state_io(ctx, "sa", t, (uint32_t)offsetof(__typeof__(*t), state_end));
	rc |= rbx_state_io(ctx, "sa_main", &c, sizeof(c));
	rc |= rbx_state_io(ctx, "sa_snd", &zs, sizeof(zs));
	rc |= rbx_ym_state(0, ctx, load);
	if (s_game != G_WHIZZ)
		rc |= rbx_ym_state(1, ctx, load);
	if (!rc && load) {
		c.read_byte = main_read;
		c.write_byte = main_write;
		c.port_in = z_in;
		c.port_out = z_out;
		c.userdata = NULL;
		c.read_map = s_mmap;
		s_main = c;
		zs.read_byte = snd_read;
		zs.write_byte = snd_write;
		zs.port_in = snd_in;
		zs.port_out = snd_out;
		zs.userdata = NULL;
		zs.read_map = s_zmap;
		s_snd = zs;
		map_bank();
	}
	return rc ? -1 : 0;
}

static int sa_save(void *ctx) { return sa_state(ctx, 0); }
static int sa_load(void *ctx) { return sa_state(ctx, 1); }
static int sa_supported(const char *machine)
{
	return !strcmp(machine, "sidearms:sidearms") || !strcmp(machine, "turtship:turtship") || !strcmp(machine, "turtship:dyger") ||
	       !strcmp(machine, "whizz:whizz");
}

const rbx_driver rbx_drv_sidearms = { "Capcom Side Arms", sa_init, sa_exit, sa_reset, sa_frame, sa_save, sa_load,
                                      sa_supported, sa_render, sa_help };
