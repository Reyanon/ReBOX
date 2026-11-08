/*  Konami Jackal / Top Gunner. Ported from MAME src/mame/konami/jackal.cpp (BSD-3-Clause, copyright-holders:
    Kenneth Lin). Two 6809Es at 1.536 MHz sharing RAM (the main CPU, the slave with the palette and the sound), a
    32x32 tilemap of 8x8 8 bpp tiles with row / column scroll, the 16x16 and 8x8 sprites of two RAMs through the lookup
    PROMs; the sound: a YM2151 (its samples made on the worker), the CPUs line by line on the main thread. The tilemap's
    flip screen not done.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../sound/rbx_ym.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 1536000u     // (18.432 MHz / 12, the two of them)
#define YM_CLOCK 3579545u
#define LINES 256              // (set_refresh_hz(60), set_size(256, 256), set_vblank_time(0))
#define CPU_LINE (CPU_CLOCK / 60 / LINES)   // (100 clocks a line)
#define VIS_X0 8
#define VIS_W 240
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 0x300            // (0x300 pens over 0x200 colours)

static uint8_t *s_rom, *s_srom, *s_prom;
static uint32_t s_rom_size, s_srom_size;
static uint8_t *s_chr, *s_spr16[2], *s_spr8[2];   // (a byte a pixel: the 8 bpp characters, the two sprite sets)
static uint32_t s_nchr, s_nspr16, s_nspr8;
static m6809_t s_cpu, s_slave;
static int s_port_dsw[2], s_port_in[3], s_port_dial[2];
static int32_t *s_mix_l, *s_mix_r;
static int s_mix_n, s_mix_cap;

static struct {
	uint8_t videoctrl[4];
	uint8_t scrollram[2][0x40], spriteram[2][0x1000], videoram[2][0x1000];
	uint8_t mainsub[0x1fa0];   // (main 0060-1fff, slave 6060-7fff)
	uint8_t pal[0x400];        // (the slave's 4000-43ff: xBGR_555, little-endian)
	uint8_t sram[0x60];        // (the slave's 6000-605f)
	uint8_t irq_enable, flip, spritebank, scrollbank, mainbank;
	int64_t cpu_end;           // (the CPUs' line target: absolute, an instruction's overshoot not carried on)
	uint64_t ym_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t videoctrl[4], scroll[0x40], spr[2][0x800], vram[0x800], pal[0x400], flip; } s_vs;

static void capture(void)
{
	memcpy(s_vs.videoctrl, t.videoctrl, sizeof(s_vs.videoctrl));
	memcpy(s_vs.scroll, t.scrollram[0], sizeof(s_vs.scroll));
	const int half = (t.videoctrl[3] >> 3) & 1;
	for (int i = 0; i < 2; i++)
		memcpy(s_vs.spr[i], t.spriteram[i] + half * 0x800, 0x800);
	memcpy(s_vs.vram, t.videoram[0], sizeof(s_vs.vram));
	memcpy(s_vs.pal, t.pal, sizeof(s_vs.pal));
	s_vs.flip = t.flip;
}

static void draw_background(uint16_t *bm, int pitch)
{
	const uint8_t *c = s_vs.videoctrl;
	const int rows = (c[2] & 0x02) && (c[2] & 0x08), cols = (c[2] & 0x02) && (c[2] & 0x04);
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch - VIS_X0;
		for (int x = VIS_X0; x < VIS_X0 + VIS_W;) {
			// (set_scroll_rows(32): the row's scroll x by the tilemap's row; set_scroll_cols(32): the column's y)
			const int sy = cols ? s_vs.scroll[(x / 8) & 31] : c[0];
			const int ty = (y + sy) & 255;
			const int sx = rows ? s_vs.scroll[(ty / 8) & 31] : c[1];
			const int tx = (x + sx) & 255;
			int n = 8 - (tx & 7);
			if (cols && n > 8 - (x & 7))
				n = 8 - (x & 7);
			if (n > VIS_X0 + VIS_W - x)
				n = VIS_X0 + VIS_W - x;
			const int idx = (ty >> 3) * 32 + (tx >> 3);
			const int attr = s_vs.vram[idx];
			const int code = s_vs.vram[0x400 + idx] + ((attr & 0xc0) << 2) + ((attr & 0x30) << 6);
			const int fx = (attr & 0x10) != 0, fy = (attr & 0x20) != 0;
			const uint8_t *src = s_chr + ((uint32_t)code % s_nchr) * 64 + (fy ? 7 - (ty & 7) : (ty & 7)) * 8;
			for (int i = 0; i < n; i++) {
				const int px = (tx + i) & 7;
				d[x + i] = src[fx ? 7 - px : px];   // (colours 256-511 without the lookup: pens 0-255)
			}
			x += n;
		}
	}
}

static void draw_tile(int set, int big, int code, int color, int flipx, int flipy, int sx, int sy, uint16_t *bm, int pitch)
{
	const int sz = big ? 16 : 8;
	const uint8_t *src = big ? s_spr16[set] + ((uint32_t)code % s_nspr16) * 256 : s_spr8[set] + ((uint32_t)code % s_nspr8) * 64;
	const int pb = (set ? 0x200 : 0x100) + 16 * color;
	for (int j = 0; j < sz; j++) {
		const int yy = sy + j;
		if (yy < VIS_Y0 || yy >= VIS_Y0 + VIS_H)
			continue;
		const uint8_t *s = src + (flipy ? sz - 1 - j : j) * sz;
		uint16_t *d = bm + (yy - VIS_Y0) * pitch - VIS_X0;
		for (int i = 0; i < sz; i++) {
			const int xx = sx + i;
			if (xx < VIS_X0 || xx >= VIS_X0 + VIS_W)
				continue;
			const int p = s[flipx ? sz - 1 - i : i];
			if (p)
				d[xx] = (uint16_t)(pb + p);
		}
	}
}

static void draw_sprites_region(const uint8_t *sram, int length, int set, uint16_t *bm, int pitch)
{
	const int flip = s_vs.flip;
	for (int offs = 0; offs + 5 <= length; offs += 5) {
		const int sn1 = sram[offs], sn2 = sram[offs + 1], attr = sram[offs + 4];
		int sy = sram[offs + 2], sx = sram[offs + 3];
		int flipx = (attr & 0x20) != 0, flipy = (attr & 0x40) != 0;
		const int color = (sn2 & 0xf0) >> 4;
		if (attr & 0x01)
			sx -= 256;
		if (sy > 0xf0)
			sy -= 256;
		if (flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			flipx = !flipx;
			flipy = !flipy;
		}
#define SPR(big, code, x, y) draw_tile(set, big, code, color, flipx, flipy, x, y, bm, pitch)
		if (attr & 0x0c) {   // (half-size sprites)
			const int spritenum = sn1 * 4 + ((sn2 & (8 + 4)) >> 2) + ((sn2 & (2 + 1)) << 10);
			int mod = -8;
			if (flip) {
				sx += 8;
				sy -= 8;
				mod = 8;
			}
			if ((attr & 0x0c) == 0x0c) {
				if (flip)
					sy += 16;
				SPR(0, spritenum, sx, sy);
			}
			if ((attr & 0x0c) == 0x08) {
				sy += 8;
				SPR(0, spritenum, sx, sy);
				SPR(0, spritenum - 2, sx, sy + mod);
			}
			if ((attr & 0x0c) == 0x04) {
				SPR(0, spritenum, sx, sy);
				SPR(0, spritenum + 1, sx + mod, sy);
			}
		} else {
			const int spritenum = sn1 + ((sn2 & 0x03) << 8);
			if (attr & 0x10) {
				if (flip) {
					sx -= 16;
					sy -= 16;
				}
				SPR(1, spritenum, flipx ? sx + 16 : sx, flipy ? sy + 16 : sy);
				SPR(1, spritenum + 1, flipx ? sx : sx + 16, flipy ? sy + 16 : sy);
				SPR(1, spritenum + 2, flipx ? sx + 16 : sx, flipy ? sy : sy + 16);
				SPR(1, spritenum + 3, flipx ? sx : sx + 16, flipy ? sy : sy + 16);
			} else
				SPR(1, spritenum, sx, sy);
		}
#undef SPR
	}
}

static void jk_render(void)   // (screen_update)
{
	uint32_t col[0x200];
	for (int i = 0; i < 0x200; i++) {   // (xBGR_555, little-endian)
		const uint16_t v = (uint16_t)(s_vs.pal[2 * i] | s_vs.pal[2 * i + 1] << 8);
		const int r = v & 31, g = (v >> 5) & 31, b = (v >> 10) & 31;
		col[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (b << 3 | b >> 2));
	}
	for (int i = 0; i < 0x100; i++) {   // (palette: 0-ff the colours 100-1ff; 100-1ff, 200-2ff the PROMs)
		rbx_video.palette[i] = col[0x100 + i];
		rbx_video.palette[0x100 + i] = col[s_prom[i] & 0x0f];
		rbx_video.palette[0x200 + i] = col[(s_prom[0x100 + i] & 0x0f) | 0x10];
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	draw_background(bm, pitch);
	draw_sprites_region(s_vs.spr[1], 0x0f5, 1, bm, pitch);
	draw_sprites_region(s_vs.spr[0], 0x500, 0, bm, pitch);
}

// ---- the buses (main_map, slave_map) ----
static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }

static void map_banks(void)
{
	for (int p = 0; p < 0x80; p++)   // (4000-bfff: the ROM's 0x4000 + 0x8000 x bank)
		s_cpu.rpage[0x40 + p] = s_rom + (0x4000u + (uint32_t)t.mainbank * 0x8000u + (uint32_t)p * 256) % s_rom_size;
	for (int p = 0; p < 0x10; p++) {
		s_cpu.rpage[0x20 + p] = s_cpu.wpage[0x20 + p] = NULL;   // (2000-2fff: the video RAM's view, its writes as is)
		s_cpu.rpage[0x30 + p] = s_cpu.wpage[0x30 + p] = t.spriteram[t.spritebank] + p * 256;
		s_cpu.rpage[0x20 + p] = s_cpu.wpage[0x20 + p] = t.videoram[t.scrollbank] + p * 256;
	}
}

static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x0004)
		return t.videoctrl[a];
	switch (a) {
	case 0x0010: return port(s_port_dsw[0]);
	case 0x0011: return port(s_port_in[1]);
	case 0x0012: return port(s_port_in[2]);
	case 0x0013: return port(s_port_in[0]);
	case 0x0014: case 0x0015: {   // (rotary_r)
		const int pi = s_port_dial[a - 0x0014];
		const int v = pi >= 0 ? (int)(rbx_port_read(pi) & 0xff) : 0;
		return (uint8_t)((1 << (v & 7)) ^ 0xff);
	}
	case 0x0018: return port(s_port_dsw[1]);
	}
	if (a >= 0x0020 && a < 0x0060)
		return t.scrollram[t.scrollbank][a - 0x0020];
	if (a >= 0x0060 && a < 0x2000)
		return t.mainsub[a - 0x0060];
	if (a < 0x3000)
		return a >= 0x2000 ? t.videoram[t.scrollbank][a - 0x2000] : 0;
	if (a < 0x4000)
		return t.spriteram[t.spritebank][a - 0x3000];
	if (a < 0xc000)
		return s_rom[(0x4000u + (uint32_t)t.mainbank * 0x8000u + (a - 0x4000)) % s_rom_size];
	return s_rom[(a - 0xc000) % s_rom_size];
}

static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0004)
		t.videoctrl[a] = v;
	else if (a == 0x0004) {   // (flipscreen_w)
		t.irq_enable = (v >> 1) & 1;
		t.flip = (v >> 3) & 1;
	} else if (a == 0x001c) {   // (rambank_w)
		t.spritebank = (v >> 3) & 1;
		t.scrollbank = (v >> 4) & 1;   // (and the video RAM's view)
		t.mainbank = (v >> 5) & 1;
		map_banks();
	} else if (a >= 0x0020 && a < 0x0060)
		t.scrollram[t.scrollbank][a - 0x0020] = v;
	else if (a >= 0x0060 && a < 0x2000)
		t.mainsub[a - 0x0060] = v;
	else if (a >= 0x2000 && a < 0x3000)
		t.videoram[t.scrollbank][a - 0x2000] = v;
	else if (a >= 0x3000 && a < 0x4000)
		t.spriteram[t.spritebank][a - 0x3000] = v;
}

static uint8_t slave_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a == 0x2000 || a == 0x2001)
		return rbx_ym_read(0, a & 1);
	if (a >= 0x4000 && a < 0x4400)
		return t.pal[a - 0x4000];
	if (a >= 0x6000 && a < 0x6060)
		return t.sram[a - 0x6000];
	if (a >= 0x6060 && a < 0x8000)
		return t.mainsub[a - 0x6060];
	if (a >= 0x8000)
		return s_srom[(a - 0x8000) % s_srom_size];
	return 0;
}
static void slave_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a == 0x2000 || a == 0x2001)
		rbx_ym_write(0, a & 1, v);
	else if (a >= 0x4000 && a < 0x4400)
		t.pal[a - 0x4000] = v;
	else if (a >= 0x6000 && a < 0x6060)
		t.sram[a - 0x6000] = v;
	else if (a >= 0x6060 && a < 0x8000)
		t.mainsub[a - 0x6060] = v;
}

// ---- the sound: a line's YM2151 samples (0.50 each side, made on the worker) ----
static void sound_line(void)
{
	t.ym_frac += (uint64_t)YM_CLOCK;
	const uint32_t yc = (uint32_t)(t.ym_frac / (60u * LINES));
	t.ym_frac -= (uint64_t)yc * (60u * LINES);
	s_mix_n += rbx_ym_advance_lr(0, yc, s_mix_l + s_mix_n, s_mix_r + s_mix_n, s_mix_cap - s_mix_n);
}

static void sound_frame_end(void)
{
	rbx_ym_async_take_lr(0, s_mix_l, s_mix_r, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put_stereo(s_mix_l[i], s_mix_r[i]);
	memset(s_mix_l, 0, sizeof(int32_t) * (size_t)s_mix_n);
	memset(s_mix_r, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

// ---- init / frame ----
static void jk_exit(void);
static void jk_reset(void);

static int decode_spr(const uint8_t *rom, uint32_t size, int big, uint8_t **out, uint32_t *count)
{
	// (spritelayout: 16x16, 4 bpp packed, the right half 32 bytes on, the lower 64; spritelayout8: 8x8, a row of 4
	// bytes; the first pixel in the top nibble)
	const uint32_t esz = big ? 128 : 32, w = big ? 16 : 8;
	*count = size / esz;
	*out = (uint8_t *)malloc((size_t)*count * w * w);
	if (!*out)
		return -1;
	for (uint32_t e = 0; e < *count; e++) {
		const uint8_t *s = rom + e * esz;
		uint8_t *d = *out + (size_t)e * w * w;
		for (uint32_t y = 0; y < w; y++)
			for (uint32_t x = 0; x < w; x++) {
				const uint32_t o = (x >> 3) * 32 + (y >> 3) * 64 + (y & 7) * 4 + (x & 7) / 2;
				d[y * w + x] = (x & 1) ? s[o] & 15 : s[o] >> 4;
			}
	}
	return 0;
}

static int jk_init(const rbx_game *g)
{
	uint32_t gs, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("slave", &s_srom_size);
	s_prom = rbx_region("proms", &ps);
	const uint8_t *gfx = rbx_region("gfx", &gs);
	if (!s_rom || !s_srom || !s_prom || ps < 0x200 || !gfx || gs < 0x80000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	// (charlayout: 8 bpp, the planes 0-3 (the high nibble) from the first quarter, 4-7 from the third; 8x8, a row of 4
	// bytes)
	s_nchr = 0x20000 / 32;
	s_chr = (uint8_t *)malloc((size_t)s_nchr * 64);
	if (!s_chr || decode_spr(gfx + 0x20000, 0x20000, 1, &s_spr16[0], &s_nspr16) ||
	    decode_spr(gfx + 0x20000, 0x20000, 0, &s_spr8[0], &s_nspr8) ||
	    decode_spr(gfx + 0x60000, 0x20000, 1, &s_spr16[1], &s_nspr16) ||
	    decode_spr(gfx + 0x60000, 0x20000, 0, &s_spr8[1], &s_nspr8)) {
		rbx_fail("%s: out of memory", g->set);
		jk_exit();
		return -1;
	}
	for (uint32_t e = 0; e < s_nchr; e++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				const uint32_t o = e * 32 + (uint32_t)(y * 4 + x / 2);
				const int hi = (x & 1) ? gfx[o] & 15 : gfx[o] >> 4;
				const int lo = (x & 1) ? gfx[0x40000 + o] & 15 : gfx[0x40000 + o] >> 4;
				s_chr[e * 64 + (uint32_t)(y * 8 + x)] = (uint8_t)(hi << 4 | lo);
			}
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 128;   // (0.50)
	if (rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 2)) {
		rbx_fail("%s: YM2151", g->set);
		jk_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix_l = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_mix_r = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	static const char *const in[3] = { "IN0", "IN1", "IN2" };
	for (int i = 0; i < 3; i++)
		s_port_in[i] = rbx_port_find(in[i]);
	s_port_dial[0] = rbx_port_find("DIAL0");
	s_port_dial[1] = rbx_port_find("DIAL1");
	if (!s_mix_l || !s_mix_r || rbx_audio_setup_stereo(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		jk_exit();
		return -1;
	}
	jk_reset();
	return 0;
}

static void jk_exit(void)
{
	rbx_ym_exit(0);
	free(s_chr);
	s_chr = NULL;
	for (int i = 0; i < 2; i++) {
		free(s_spr16[i]);
		free(s_spr8[i]);
		s_spr16[i] = s_spr8[i] = NULL;
	}
	free(s_mix_l);
	free(s_mix_r);
	s_mix_l = s_mix_r = NULL;
}

static void jk_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	s_mix_n = 0;
	if (s_mix_l) {
		memset(s_mix_l, 0, sizeof(int32_t) * (size_t)s_mix_cap);
		memset(s_mix_r, 0, sizeof(int32_t) * (size_t)s_mix_cap);
	}
	rbx_ym_reset(0);
	m6809_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	for (int p = 0x01; p < 0x20; p++)   // (0100-1fff of the shared RAM; 0060-00ff through the handlers)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.mainsub + p * 256 - 0x60;
	for (int p = 0xc0; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + ((uint32_t)(p - 0xc0) * 256) % s_rom_size;
	map_banks();
	m6809_reset(&s_cpu);
	m6809_init(&s_slave);
	s_slave.read = slave_rd;
	s_slave.write = slave_wr;
	for (int p = 0x40; p < 0x44; p++)
		s_slave.rpage[p] = s_slave.wpage[p] = t.pal + (p - 0x40) * 256;
	for (int p = 0x61; p < 0x80; p++)   // (6100-7fff; 6000-60ff through the handlers)
		s_slave.rpage[p] = s_slave.wpage[p] = t.mainsub + (p - 0x60) * 256 - 0x60;
	for (int p = 0x80; p < 0x100; p++)
		s_slave.rpage[p] = s_srom + ((uint32_t)(p - 0x80) * 256) % s_srom_size;
	m6809_reset(&s_slave);
}

static void jk_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, vblank_irq)
			capture();
			rbx_video_frame_done();
			if (t.irq_enable) {
				m6809_hold_irq(&s_cpu);
				m6809_set_nmi(&s_slave, 1);   // (pulse_input_line)
				m6809_set_nmi(&s_slave, 0);
			}
		}
		t.cpu_end += CPU_LINE;
		m6809_run_until(&s_cpu, t.cpu_end);
		m6809_run_until(&s_slave, t.cpu_end);
		sound_line();
	}
	sound_frame_end();
	t.frame++;
}

static int jk_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "jk_cpu", &s_cpu, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "jk_slave", &s_slave, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "jk", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_ym_state(0, ctx, load);
	if (!rc && load) {
		s_mix_n = 0;
		memset(s_mix_l, 0, sizeof(int32_t) * (size_t)s_mix_cap);
		memset(s_mix_r, 0, sizeof(int32_t) * (size_t)s_mix_cap);
		map_banks();
	}
	return rc ? -1 : 0;
}

static int jk_save(void *ctx) { return jk_state(ctx, 0); }
static int jk_load(void *ctx) { return jk_state(ctx, 1); }
static int jk_supported(const char *machine) { return !strcmp(machine, "jackal"); }

const rbx_driver rbx_drv_jackal = { "Konami Jackal", jk_init, jk_exit, jk_reset, jk_frame, jk_save, jk_load, jk_supported,
                                    jk_render, NULL };
