/*  Konami Double Dribble. Ported from MAME src/mame/konami/ddribble.cpp (BSD-3-Clause, copyright-holders: Manuel
    Abadia). Three 6809Es at 1.536 MHz sharing RAM (main / sub, sub / sound), two 005885s (a 64x32 tilemap of 8x8 tiles
    and the sprites each), the sprites' colours through a lookup PROM; the sound: a YM2203 (its samples made on the
    worker) and a VLM5030, with the CPUs line by line on the main thread (the SSG's RC filters not done).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/vlm5030.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 1536000u     // (18.432 MHz / 12, the three of them)
#define YM_CLOCK 3579545u
#define LINES 256              // (set_refresh_hz(60), set_size(256, 256), set_vblank_time(0))
#define CPU_LINE (CPU_CLOCK / 60 / LINES)   // (100 clocks a line)
#define VIS_W 256
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 0x140            // (64 + 256 pens over 64 colours)

static uint8_t *s_rom, *s_srom, *s_arom, *s_vlmrom, *s_prom;
static uint32_t s_rom_size, s_srom_size, s_arom_size, s_vlmrom_size;
static uint8_t *s_chr[2], *s_spr[2];   // (gfx1, gfx2: their characters and their sprites, a byte a pixel)
static uint32_t s_nchr[2], s_nspr[2];
static m6809_t s_cpu, s_sub, s_snd;
static vlm5030_t s_vlm;
static int s_port_p[2], s_port_dsw[3], s_port_system;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;

static struct {
	uint8_t pal[0x80];        // 1800-187f
	uint8_t fg[0x1000], spr0[0x1000], shared[0x2000], bg[0x1000], spr1[0x1000], snd_shared[0x800];
	uint8_t vregs[2][5], charbank[2], int_enable[2];
	uint8_t mainbank, vlmbank;
	uint32_t rand;            // (vlm5030_busy_r: machine().rand(), a 32-bit LCG here)
	int16_t vlm_out;
	int64_t cpu_end;          // (the CPUs' line target: absolute, an instruction's overshoot not carried on)
	uint64_t ym_frac, vlm_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t pal[0x80], fg[0x1000], bg[0x1000], spr0[0x80], spr1[0x140], vregs[2][5], charbank[2]; } s_vs;

static void capture(void)
{
	memcpy(s_vs.pal, t.pal, sizeof(s_vs.pal));
	memcpy(s_vs.fg, t.fg, sizeof(s_vs.fg));
	memcpy(s_vs.bg, t.bg, sizeof(s_vs.bg));
	memcpy(s_vs.spr0, t.spr0, sizeof(s_vs.spr0));
	memcpy(s_vs.spr1, t.spr1, sizeof(s_vs.spr1));
	memcpy(s_vs.vregs, t.vregs, sizeof(s_vs.vregs));
	memcpy(s_vs.charbank, t.charbank, sizeof(s_vs.charbank));
}

// a tilemap (tilemap_scan: 64x32, the second half 0x800 on), scrolled; fg its pen 0 transparent
static void draw_tilemap(int which, uint16_t *bm, int pitch)
{
	const uint8_t *vram = which ? s_vs.bg : s_vs.fg;
	const uint8_t *r = s_vs.vregs[which];
	const int scrollx = r[1] | ((r[2] & 0x01) << 8), scrolly = r[0];
	const int base = which ? 16 : 48;   // (gfx2's characters colours 16-31, gfx1's 48-63)
	const uint8_t *gfx = s_chr[which ? 1 : 0];
	const uint32_t n = s_nchr[which ? 1 : 0];
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		const int ty = (y + scrolly) & 255;
		for (int x = 0; x < VIS_W;) {
			const int tx = (x + scrollx) & 511;
			const int col = tx >> 3, row = ty >> 3;
			const int idx = (col & 0x1f) + ((row & 0x1f) << 5) + ((col & 0x20) << 6);
			const int attr = vram[idx];
			int num = vram[idx + 0x400] + ((attr & 0xc0) << 2) + ((attr & 0x20) << 5);
			num += which ? s_vs.charbank[1] << 11 : (s_vs.charbank[0] & 2) << 10;
			const int fl = (attr & 0x30) >> 4;   // (TILE_FLIPYX)
			const uint8_t *src = gfx + (rbx_wrap((uint32_t)num, n)) * 64 + ((fl & 2) ? 7 - (ty & 7) : (ty & 7)) * 8;
			int cnt = 8 - (tx & 7);
			if (cnt > VIS_W - x)
				cnt = VIS_W - x;
			for (int i = 0; i < cnt; i++) {
				const int px = (tx + i) & 7;
				const int p = src[(fl & 1) ? 7 - px : px];
				if (p || which)
					d[x + i] = (uint16_t)(base + p);
			}
			x += cnt;
		}
	}
}

static void draw_sprites(const uint8_t *source, int length, int set, int flipscreen, uint16_t *bm, int pitch)
{
	const uint8_t *gfx = s_spr[set];
	const uint32_t n = s_nspr[set];
	for (const uint8_t *finish = source + length; source + 5 <= finish; source += 5) {
		int number = source[0] | ((source[1] & 0x07) << 8);
		const int attr = source[4];
		int sx = source[3] | ((attr & 0x01) << 8), sy = source[2];
		int flipx = (attr & 0x20) != 0, flipy = (attr & 0x40) != 0;
		const int color = (source[1] & 0xf0) >> 4;
		int width, height;
		if (sx & 0x100)
			sx -= 0x200;
		if (flipscreen) {
			flipx = !flipx;
			flipy = !flipy;
			sx = 240 - sx;
			sy = 240 - sy;
			if ((attr & 0x1c) == 0x10) {
				sx -= 0x10;
				sy -= 0x10;
			}
		}
		switch (attr & 0x1c) {
		case 0x10: width = height = 2; number &= ~3; break;
		case 0x08: width = 1; height = 2; number &= ~2; break;
		case 0x04: width = 2; height = 1; number &= ~1; break;
		default: width = height = 1; break;
		}
		// (gfx1's sprites one colour at 32, gfx2's sixteen at 64 through the PROM)
		const int pb = set ? 64 + 16 * color : 32;
		for (int y = 0; y < height; y++)
			for (int x = 0; x < width; x++) {
				const int ex = flipx ? width - 1 - x : x, ey = flipy ? height - 1 - y : y;
				const uint8_t *src = gfx + (rbx_wrap((uint32_t)(number + ex + 2 * ey), n)) * 256;
				const int ox = sx + x * 16, oy = sy + y * 16;
				for (int j = 0; j < 16; j++) {
					const int yy = oy + j;
					if (yy < VIS_Y0 || yy >= VIS_Y0 + VIS_H)
						continue;
					const uint8_t *s = src + (flipy ? 15 - j : j) * 16;
					uint16_t *d = bm + (yy - VIS_Y0) * pitch;
					for (int i = 0; i < 16; i++) {
						const int xx = ox + i;
						if (xx < 0 || xx >= VIS_W)
							continue;
						const int p = s[flipx ? 15 - i : i];
						if (p)
							d[xx] = (uint16_t)(pb + p);
					}
				}
			}
	}
}

static void dd_render(void)   // (screen_update)
{
	uint32_t col[64];
	for (int i = 0; i < 64; i++) {   // (xBGR_555, the high byte first)
		const uint16_t v = (uint16_t)(s_vs.pal[2 * i] << 8 | s_vs.pal[2 * i + 1]);
		const int r = v & 31, g = (v >> 5) & 31, b = (v >> 10) & 31;
		col[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (b << 3 | b >> 2));
	}
	for (int i = 0; i < 0x40; i++)   // (palette: 10-3f direct, 40-13f the PROM's, 00-0f unused)
		rbx_video.palette[i] = i < 0x10 ? 0 : col[i];
	for (int i = 0; i < 0x100; i++)
		rbx_video.palette[0x40 + i] = col[s_prom[i] & 0x0f];
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	draw_tilemap(1, bm, pitch);
	draw_sprites(s_vs.spr0, 0x07d, 0, s_vs.vregs[0][4] & 0x08, bm, pitch);
	draw_sprites(s_vs.spr1, 0x140, 1, s_vs.vregs[1][4] & 0x08, bm, pitch);
	draw_tilemap(0, bm, pitch);
}

// ---- the buses (maincpu_map, subcpu_map, audiocpu_map) ----
static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }

static void map_bank(void)
{
	for (int p = 0; p < 0x20; p++)
		s_cpu.rpage[0x80 + p] = s_rom + rbx_wrap(((uint32_t)t.mainbank * 0x2000 + (uint32_t)p * 256), s_rom_size);
}

static void k005885_w(int which, int offset, uint8_t v)
{
	if (offset == 3)   // (the characters' bank)
		t.charbank[which] = v & 0x03;
	else if (offset == 4)   // (the IRQ's enable, the flip screen)
		t.int_enable[which] = v & 0x02;
	t.vregs[which][offset] = v;
}

static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a >= 0x1800 && a < 0x1880)
		return t.pal[a - 0x1800];
	if (a >= 0x2000 && a < 0x3000)
		return t.fg[a - 0x2000];
	if (a >= 0x3000 && a < 0x4000)
		return t.spr0[a - 0x3000];
	if (a >= 0x4000 && a < 0x6000)
		return t.shared[a - 0x4000];
	if (a >= 0x6000 && a < 0x7000)
		return t.bg[a - 0x6000];
	if (a >= 0x7000 && a < 0x8000)
		return t.spr1[a - 0x7000];
	if (a >= 0x8000 && a < 0xa000)
		return s_rom[rbx_wrap(((uint32_t)t.mainbank * 0x2000 + (a - 0x8000)), s_rom_size)];
	if (a >= 0xa000)
		return s_rom[rbx_wrap(a, s_rom_size)];
	return 0;
}
static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0005)
		k005885_w(0, a, v);
	else if (a >= 0x0800 && a < 0x0805)
		k005885_w(1, a - 0x0800, v);
	else if (a >= 0x1800 && a < 0x1880)
		t.pal[a - 0x1800] = v;
	else if (a >= 0x2000 && a < 0x3000)
		t.fg[a - 0x2000] = v;
	else if (a >= 0x3000 && a < 0x4000)
		t.spr0[a - 0x3000] = v;
	else if (a >= 0x4000 && a < 0x6000)
		t.shared[a - 0x4000] = v;
	else if (a >= 0x6000 && a < 0x7000)
		t.bg[a - 0x6000] = v;
	else if (a >= 0x7000 && a < 0x8000)
		t.spr1[a - 0x7000] = v;
	else if (a == 0x8000) {   // (bankswitch_w)
		t.mainbank = v & 0x07;
		map_bank();
	}
}

static uint8_t sub_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x2000)
		return t.shared[a];
	if (a < 0x2800)
		return t.snd_shared[a - 0x2000];
	switch (a) {
	case 0x2800: return port(s_port_dsw[0]);
	case 0x2801: return port(s_port_p[0]);
	case 0x2802: return port(s_port_p[1]);
	case 0x2803: return port(s_port_system);
	case 0x2c00: return port(s_port_dsw[1]);
	case 0x3000: return port(s_port_dsw[2]);
	}
	if (a >= 0x8000)
		return s_srom[rbx_wrap((a - 0x8000), s_srom_size)];
	return 0;
}
static void sub_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x2000)
		t.shared[a] = v;
	else if (a < 0x2800)
		t.snd_shared[a - 0x2000] = v;
}

static void vlm_bank(void)
{
	const uint32_t off = (uint32_t)t.vlmbank * 0x10000u;
	s_vlm.rom = s_vlmrom + (off < s_vlmrom_size ? off : 0);
	s_vlm.rom_size = s_vlmrom_size < 0x10000 ? s_vlmrom_size : 0x10000;
}
static void ym_port_a(uint8_t v)   // (vlm5030_ctrl_w; bits 0-2 the SSG's filters)
{
	vlm5030_rst(&s_vlm, (v >> 6) & 1);
	vlm5030_st(&s_vlm, (v >> 5) & 1);
	vlm5030_vcu(&s_vlm, (v >> 4) & 1);
	t.vlmbank = (v >> 3) & 1;
	vlm_bank();
}
static uint8_t ym_port_b(void)   // (vlm5030_busy_r: "patch", machine().rand())
{
	t.rand = t.rand * 1103515245u + 12345u;
	return (uint8_t)(t.rand >> 16);
}

static uint8_t snd_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x0800)
		return t.snd_shared[a];
	if (a == 0x1000 || a == 0x1001)
		return rbx_ym_read(0, a & 1);
	if (a >= 0x8000)
		return s_arom[rbx_wrap((a - 0x8000), s_arom_size)];
	return 0;
}
static void snd_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0800)
		t.snd_shared[a] = v;
	else if (a == 0x1000 || a == 0x1001)
		rbx_ym_write(0, a & 1, v);
	else if (a == 0x3000)
		vlm5030_data_w(&s_vlm, v);
}

// ---- the sound: a line's YM2203 samples (0.25 each route), the VLM5030 (1.0) at its rate held between ----
static void sound_line(void)
{
	t.ym_frac += (uint64_t)YM_CLOCK;
	const uint32_t yc = (uint32_t)(t.ym_frac / (60u * LINES));
	t.ym_frac -= (uint64_t)yc * (60u * LINES);
	const int n = rbx_ym_advance(0, yc, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	const uint32_t rate = rbx_ym_rate(0);
	for (int i = 0; i < n; i++) {
		t.vlm_frac += vlm5030_rate(&s_vlm);
		while (t.vlm_frac >= rate) {
			t.vlm_frac -= rate;
			int16_t s;
			vlm5030_render(&s_vlm, &s, 1);
			t.vlm_out = s;
		}
		s_mix[s_mix_n + i] += t.vlm_out;
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

// ---- init / frame ----
static void dd_exit(void);
static void dd_reset(void);

static int decode(const uint8_t *rom, uint32_t size, int sprites, uint8_t **out, uint32_t *count)
{
	// (charlayout: 8x8, 4 bpp packed, a row of 4 bytes; spritelayout: 16x16, the right half 32 bytes on, the lower
	// 64; the first pixel in the top nibble)
	const uint32_t esz = sprites ? 128 : 32, npix = sprites ? 256 : 64;
	*count = size / esz;
	*out = (uint8_t *)malloc((size_t)*count * npix);
	if (!*out)
		return -1;
	for (uint32_t e = 0; e < *count; e++) {
		const uint8_t *s = rom + e * esz;
		uint8_t *d = *out + (size_t)e * npix;
		const int w = sprites ? 16 : 8;
		for (int y = 0; y < w; y++)
			for (int x = 0; x < w; x++) {
				const uint32_t o = (uint32_t)(sprites ? ((x >> 3) * 32 + (y >> 3) * 64 + (y & 7) * 4 + (x & 7) / 2)
				                                      : (y * 4 + x / 2));
				d[y * w + x] = (x & 1) ? s[o] & 15 : s[o] >> 4;
			}
	}
	return 0;
}

static int dd_init(const rbx_game *g)
{
	uint32_t g1, g2, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("subcpu", &s_srom_size);
	s_arom = rbx_region("audiocpu", &s_arom_size);
	s_vlmrom = rbx_region("vlm", &s_vlmrom_size);
	s_prom = rbx_region("proms", &ps);
	const uint8_t *gfx1 = rbx_region("gfx1", &g1), *gfx2 = rbx_region("gfx2", &g2);
	if (!s_rom || !s_srom || !s_arom || !s_vlmrom || !s_prom || ps < 0x100 || !gfx1 || !gfx2) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	if (decode(gfx1, g1 / 2, 0, &s_chr[0], &s_nchr[0]) || decode(gfx1 + g1 / 2, g1 / 2, 1, &s_spr[0], &s_nspr[0]) ||
	    decode(gfx2, g2 / 2, 0, &s_chr[1], &s_nchr[1]) || decode(gfx2 + g2 / 2, g2 / 2, 1, &s_spr[1], &s_nspr[1])) {
		rbx_fail("%s: out of memory", g->set);
		dd_exit();
		return -1;
	}
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 64;   // (0.25)
	yc.port_a_write = ym_port_a;
	yc.port_b_read = ym_port_b;
	if (rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: YM2203", g->set);
		dd_exit();
		return -1;
	}
	vlm5030_init(&s_vlm, YM_CLOCK, s_vlmrom, s_vlmrom_size < 0x10000 ? s_vlmrom_size : 0x10000);
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_system = rbx_port_find("SYSTEM");
	static const char *const dn[3] = { "DSW1", "DSW2", "DSW3" };
	for (int i = 0; i < 3; i++)
		s_port_dsw[i] = rbx_port_find(dn[i]);
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		dd_exit();
		return -1;
	}
	dd_reset();
	return 0;
}

static void dd_exit(void)
{
	rbx_ym_exit(0);
	for (int i = 0; i < 2; i++) {
		free(s_chr[i]);
		free(s_spr[i]);
		s_chr[i] = s_spr[i] = NULL;
	}
	free(s_mix);
	s_mix = NULL;
}

static void dd_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	s_mix_n = 0;
	if (s_mix)
		memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_cap);
	rbx_ym_reset(0);
	vlm5030_reset(&s_vlm);
	vlm_bank();
	m6809_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	for (int p = 0x18; p < 0x19; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = NULL;
	for (int p = 0x20; p < 0x30; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.fg + (p - 0x20) * 256;
	for (int p = 0x30; p < 0x40; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.spr0 + (p - 0x30) * 256;
	for (int p = 0x40; p < 0x60; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.shared + (p - 0x40) * 256;
	for (int p = 0x60; p < 0x70; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.bg + (p - 0x60) * 256;
	for (int p = 0x70; p < 0x80; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.spr1 + (p - 0x70) * 256;
	for (int p = 0xa0; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + ((uint32_t)p * 256) % s_rom_size;
	map_bank();
	m6809_reset(&s_cpu);
	m6809_init(&s_sub);
	s_sub.read = sub_rd;
	s_sub.write = sub_wr;
	for (int p = 0; p < 0x20; p++)
		s_sub.rpage[p] = s_sub.wpage[p] = t.shared + p * 256;
	for (int p = 0x20; p < 0x28; p++)
		s_sub.rpage[p] = s_sub.wpage[p] = t.snd_shared + (p - 0x20) * 256;
	for (int p = 0x80; p < 0x100; p++)
		s_sub.rpage[p] = s_srom + ((uint32_t)(p - 0x80) * 256) % s_srom_size;
	m6809_reset(&s_sub);
	m6809_init(&s_snd);
	s_snd.read = snd_rd;
	s_snd.write = snd_wr;
	for (int p = 0; p < 8; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = t.snd_shared + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		s_snd.rpage[p] = s_arom + ((uint32_t)(p - 0x80) * 256) % s_arom_size;
	m6809_reset(&s_snd);
}

static void dd_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, vblank_irq)
			capture();
			rbx_video_frame_done();
			if (t.int_enable[0])
				m6809_hold_firq(&s_cpu);
			if (t.int_enable[1])
				m6809_hold_firq(&s_sub);
		}
		t.cpu_end += CPU_LINE;
		m6809_run_until(&s_cpu, t.cpu_end);
		m6809_run_until(&s_sub, t.cpu_end);
		m6809_run_until(&s_snd, t.cpu_end);
		sound_line();
	}
	sound_frame_end();
	t.frame++;
}

static int dd_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "dd_cpu", &s_cpu, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "dd_sub", &s_sub, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "dd_snd", &s_snd, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "dd", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= vlm5030_state(&s_vlm, ctx, load, "dd_vlm");
	if (!rc && load) {
		s_mix_n = 0;
		memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_cap);
		map_bank();
		vlm_bank();
	}
	return rc ? -1 : 0;
}

static int dd_save(void *ctx) { return dd_state(ctx, 0); }
static int dd_load(void *ctx) { return dd_state(ctx, 1); }
static int dd_supported(const char *machine) { return !strcmp(machine, "ddribble"); }

const rbx_driver rbx_drv_ddribble = { "Konami Double Dribble", dd_init, dd_exit, dd_reset, dd_frame, dd_save, dd_load,
                                      dd_supported, dd_render, NULL };
