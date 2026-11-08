/*  Konami GX400 / Nemesis hardware (Nemesis / Gradius, TwinBee, Galactic Warriors, Konami GT, RF2, Salamander /
    Life Force, Black Panther, City Bomber, Kitten Kaboodle, Hyper Crash). Ported from MAME src/mame/konami/nemesis.cpp
    and nemesis_v.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail). A 68000 at 9.216 MHz (Hyper Crash 6.144), two
    tilemaps and the sprites drawn from the character RAM (any of eight sizes, zoomed), drawn here into the indexed
    bitmap; the sound: a Z80 with two AY-3-8910s and a K005289 (+ a VLM5030 on the GX400 boards), or a K007232 with a
    YM2151 (+ a VLM5030) or a YM3812 and a K051649. The sound runs with the main CPU line by line (the GX400 boards
    share RAM between the two CPUs).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"
#include "../../sound/k005289.h"
#include "../../sound/k007232.h"
#include "../../sound/k051649.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/vlm5030.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define SND_RATE 48000
#define PIX_CLOCK 6144000u    // (set_raw(18.432 MHz / 3, 384, 0, 256, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_W 256
#define VIS_Y0 16
#define VIS_H 224

enum { M_NEMESIS, M_KONAMIGT, M_GX400, M_GRADIUS, M_RF2, M_SALAMAND, M_BLKPNTHR, M_CITYBOMB, M_NYANPANI, M_HCRASH };
enum { SND_NEMESIS, SND_GX400, SND_SALAMAND, SND_BLKPNTHR, SND_CITY };
enum { IRQ_VBL1, IRQ_VBL2, IRQ_GX400, IRQ_KONAMIGT, IRQ_HCRASH };

// a machine's video blocks (MAME's shares) and its other RAM
typedef struct {
	const char *name;
	int sound, irq;
	uint32_t cpu_clock, z80_clock;
	uint32_t rom_end;                 // (the program ROM's end)
	uint32_t xs[2], ys[2], vr[2], cr[2], chr, spr, pal;
	int pal_resnet;                   // (the GX400 boards' palette words through the resistor network, else xBGR_555 bytes)
	uint32_t ram[3][2];               // (plain RAM ranges)
} mach_t;

static const mach_t s_machs[] = {
	{ "nemesis", SND_NEMESIS, IRQ_VBL1, 9216000, 1789772, 0x03ffff, { 0x050000, 0x050400 }, { 0x050f80, 0x050f00 },
	  { 0x052000, 0x053000 }, { 0x054000, 0x055000 }, 0x040000, 0x056000, 0x05a000, 1,
	  { { 0x060000, 0x067fff }, { 0x050800, 0x050eff }, { 0x051000, 0x051fff } } },
	{ "konamigt", SND_NEMESIS, IRQ_KONAMIGT, 9216000, 1789772, 0x03ffff, { 0x050000, 0x050400 }, { 0x050f80, 0x050f00 },
	  { 0x052000, 0x053000 }, { 0x054000, 0x055000 }, 0x040000, 0x056000, 0x05a000, 1,
	  { { 0x060000, 0x067fff }, { 0x050800, 0x050eff }, { 0x051000, 0x051fff } } },
	{ "gx400", SND_GX400, IRQ_GX400, 9216000, 1789772, 0x00ffff, { 0x050000, 0x050400 }, { 0x050f80, 0x050f00 },
	  { 0x052000, 0x053000 }, { 0x054000, 0x055000 }, 0x030000, 0x056000, 0x05a000, 1,
	  { { 0x010000, 0x01ffff }, { 0x050800, 0x050eff }, { 0x051000, 0x051fff } } },
	{ "gradius", SND_GX400, IRQ_GX400, 9216000, 1789772, 0x00ffff, { 0x050000, 0x050400 }, { 0x050f80, 0x050f00 },
	  { 0x052000, 0x053000 }, { 0x054000, 0x055000 }, 0x030000, 0x056000, 0x05a000, 1,
	  { { 0x010000, 0x01ffff }, { 0x070000, 0x073fff }, { 0x050800, 0x050eff } } },
	{ "rf2_gx400", SND_GX400, IRQ_GX400, 9216000, 1789772, 0x00ffff, { 0x050000, 0x050400 }, { 0x050f80, 0x050f00 },
	  { 0x052000, 0x053000 }, { 0x054000, 0x055000 }, 0x030000, 0x056000, 0x05a000, 1,
	  { { 0x010000, 0x01ffff }, { 0x050800, 0x050eff }, { 0x051000, 0x051fff } } },
	{ "salamand", SND_SALAMAND, IRQ_VBL1, 9216000, 3579545, 0x07ffff, { 0x190400, 0x190000 }, { 0x190f00, 0x190f80 },
	  { 0x101000, 0x100000 }, { 0x103000, 0x102000 }, 0x120000, 0x180000, 0x090000, 0,
	  { { 0x080000, 0x087fff }, { 0x190800, 0x190eff }, { 0x191000, 0x191fff } } },
	{ "blkpnthr", SND_BLKPNTHR, IRQ_VBL2, 9216000, 3579545, 0x07ffff, { 0x180000, 0x180400 }, { 0x180f80, 0x180f00 },
	  { 0x102000, 0x103000 }, { 0x100000, 0x101000 }, 0x120000, 0x190000, 0x080000, 0,
	  { { 0x090000, 0x097fff }, { 0x180800, 0x180eff }, { 0x181000, 0x181fff } } },
	{ "citybomb", SND_CITY, IRQ_VBL1, 9216000, 3579545, 0x01ffff, { 0x300000, 0x300400 }, { 0x300f80, 0x300f00 },
	  { 0x210000, 0x211000 }, { 0x212000, 0x213000 }, 0x200000, 0x310000, 0x0e0000, 0,
	  { { 0x080000, 0x087fff }, { 0x300800, 0x300eff }, { 0x301000, 0x301fff } } },
	{ "nyanpani", SND_CITY, IRQ_VBL1, 9216000, 3579545, 0x01ffff, { 0x310000, 0x310400 }, { 0x310f80, 0x310f00 },
	  { 0x200000, 0x201000 }, { 0x202000, 0x203000 }, 0x210000, 0x300000, 0x060000, 0,
	  { { 0x040000, 0x047fff }, { 0x310800, 0x310eff }, { 0x311000, 0x311fff } } },
	{ "hcrash", SND_SALAMAND, IRQ_HCRASH, 6144000, 3579545, 0x00ffff, { 0x190400, 0x190000 }, { 0x190f00, 0x190f80 },
	  { 0x101000, 0x100000 }, { 0x103000, 0x102000 }, 0x120000, 0x180000, 0x090000, 0,
	  { { 0x080000, 0x083fff }, { 0x190800, 0x190eff }, { 0x191000, 0x191fff } } },
};

// the palette's resistor network (create_palette_lookups: compute_res_net of 4.7k 2.4k 1.2k 620 300 open collector,
// emitter follower, 1k bias, scaled from black to white)
static const uint8_t s_resnet[32] = { 0, 1, 2, 4, 5, 6, 8, 9, 11, 13, 15, 18, 20, 22, 25, 28,
                                      33, 36, 41, 46, 51, 57, 64, 73, 80, 91, 104, 120, 142, 168, 204, 255 };

static const mach_t *s_m;
static int s_mach;
static uint8_t *s_rom, *s_zrom, *s_maindata;
static uint32_t s_rom_size, s_zrom_size, s_maindata_size;
static const uint8_t *s_vlmrom;
static uint32_t s_vlmrom_size;
static uint8_t s_charnib[0x20000];   // (the character RAM, a nibble a byte: every gfx layout is its linear pixels)
static uint8_t s_chdirty[0x200];     // (64-word blocks written since the last decoding)
static int s_any_dirty;
static z80 s_z;
static const uint8_t *s_zmap[256];
static ay8910_t s_ay[2];
static k005289_t s_k5289;
static vlm5030_t s_vlm;
static k007232_t s_k232;
static k051649_t s_scc;
static int s_ports[6];   // (IN0 IN1 IN2 DSW0 DSW1 TEST)
static int s_port_in3, s_port_wheel, s_port_accel;

static struct {
	uint16_t xs[2][0x200], ys[2][0x40], vr[2][0x800], cr[2][0x800], chr[0x8000], spr[0x800];
	uint16_t palw[0x800];    // (the GX400 boards' palette words)
	uint8_t pal8[0x2000];    // (the others' palette bytes, umask 00ff)
	uint16_t ram[3][0x8000];
	uint16_t extra[0x2000];  // (gradius's 070000 work RAM)
	uint8_t shared[0x4000];  // (GX400: the sound CPU's 4000-7fff, the main CPU's 020000 (low bytes))
	uint8_t voiceram[0x800], zram[0x800];
	uint8_t irq1_on, irq2_on, irq4_on, sound_irq_clk, flipx, flipy, speech_offset, selected_ip;
	uint8_t soundlatch, z80_irq, k232_bank;
	uint64_t cpu_frac, z_frac, z_end, ym_frac, snd_frac, ay_frac, k232_frac, scc_frac, vlm_frac;
	int32_t k232_a, k232_b, scc_out, vlm_out;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct {
	uint16_t xs[2][0x200], ys[2][0x40], vr[2][0x800], cr[2][0x800], spr[0x800];
	uint16_t palw[0x800];
	uint8_t pal8[0x1000], flip;
} s_vs;
static uint8_t s_pri[VIS_H][VIS_W];
static int32_t s_snd[2][2048];
static int s_snd_n;

static void decode_chars(void)
{
	for (int b = 0; b < 0x200; b++) {
		if (!s_chdirty[b])
			continue;
		s_chdirty[b] = 0;
		for (int w = b * 64; w < b * 64 + 64; w++) {
			const uint16_t v = t.chr[w];
			uint8_t *d = s_charnib + w * 4;
			d[0] = v >> 12;
			d[1] = (v >> 8) & 15;
			d[2] = (v >> 4) & 15;
			d[3] = v & 15;
		}
	}
	s_any_dirty = 0;
}

static void capture(void)
{
	if (s_any_dirty)   // (the drawing is idle here)
		decode_chars();
	memcpy(s_vs.xs, t.xs, sizeof(s_vs.xs));
	memcpy(s_vs.ys, t.ys, sizeof(s_vs.ys));
	memcpy(s_vs.vr, t.vr, sizeof(s_vs.vr));
	memcpy(s_vs.cr, t.cr, sizeof(s_vs.cr));
	memcpy(s_vs.spr, t.spr, sizeof(s_vs.spr));
	if (s_m->pal_resnet)
		memcpy(s_vs.palw, t.palw, sizeof(s_vs.palw));
	else
		memcpy(s_vs.pal8, t.pal8, sizeof(s_vs.pal8));
	s_vs.flip = (uint8_t)(t.flipx | t.flipy << 1);
}

// ---- the drawing (screen_update) ----
// a tilemap pixel: bits 0-10 the pen, bit 12 opaque, bits 13-14 the category
static inline uint32_t tile_pixel(int which, int lx, int ly)
{
	const int ti = ((ly >> 3) & 31) * 64 + ((lx >> 3) & 63);
	const uint16_t code = s_vs.vr[which][ti], color = s_vs.cr[which][ti];
	if (!(code & 0xf800))   // (the blank tile: transparent, category 0)
		return 0;
	int px = lx & 7, py = ly & 7;
	if (color & 0x80)
		px ^= 7;
	if (code & 0x0800)
		py ^= 7;
	const int p = s_charnib[(code & 0x7ff) * 64 + py * 8 + px];
	const int force = !(code & 0x2000) || (code & 0xc000) == 0x4000;   // (TILE_FORCE_LAYER0: no transparency)
	const int mask = (code >> 12) & 1;
	int layer = (code >> 14) & 1;
	if (mask && !layer)
		layer = 1;
	return (uint32_t)((color & 0x7f) * 16 + p) | ((p || force) ? 0x1000u : 0) | (uint32_t)(mask | layer << 1) << 13;
}

// a layer's row of pixels (tile_pixel's values) over the visible area: tile by tile without a flipped screen
static void layer_row(int w, int y, int sx, uint16_t *out)
{
	const int fx = s_vs.flip & 1, fy = (s_vs.flip >> 1) & 1;
	// (a flipped tilemap: mirrored around the visible area's centre, 256 x 256; the column scroll of the flipped one)
	int x = 0, lx = fx ? (255 + sx) & 511 : sx & 511;
	while (x < VIS_W) {
		const int col = lx >> 3, px0 = lx & 7;
		int n = fx ? px0 + 1 : 8 - px0;
		if (n > VIS_W - x)
			n = VIS_W - x;
		const int ysc = s_vs.ys[w][fx ? (col + 0x20) & 0x3f : col];
		const int ly = fy ? (255 - y + ysc) & 255 : (y + ysc) & 255;
		const int ti = (ly >> 3) * 64 + col;
		const uint16_t code = s_vs.vr[w][ti], color = s_vs.cr[w][ti];
		if (!(code & 0xf800))   // (the blank tile: transparent)
			memset(out + x, 0, (size_t)n * sizeof(uint16_t));
		else {
			const int py = (code & 0x0800) ? 7 - (ly & 7) : (ly & 7);
			const uint8_t *src = s_charnib + (code & 0x7ff) * 64 + py * 8;
			const int force = !(code & 0x2000) || (code & 0xc000) == 0x4000;
			const int mask = (code >> 12) & 1;
			int layer = (code >> 14) & 1;
			if (mask && !layer)
				layer = 1;
			const uint16_t base = (uint16_t)((color & 0x7f) * 16 | (mask | layer << 1) << 13);
			const int rev = (color & 0x80) != 0;
			for (int k = 0; k < n; k++) {
				const int tx = fx ? px0 - k : px0 + k;
				const int p = src[rev ? 7 - tx : tx];
				out[x + k] = (uint16_t)(base + p) | ((p || force) ? 0x1000 : 0);
			}
		}
		x += n;
		lx = fx ? (lx - n) & 511 : (lx + n) & 511;
	}
}

static void draw_tilemaps(uint16_t *bm)
{
	const int fx = s_vs.flip & 1;
	uint16_t row[2][VIS_W];
	for (int r = 0; r < VIS_H; r++) {
		const int y = VIS_Y0 + r, oy = fx ? 255 - y : y;   // (offset_y: with m_flipscreen, the flip x one)
		for (int w = 0; w < 2; w++)
			layer_row(w, y, (s_vs.xs[w][oy & 0xff] & 0xff) + ((s_vs.xs[w][0x100 + (oy & 0xff)] & 1) << 8) - (fx ? 0x107 : 0),
			          row[w]);
		uint16_t *d = bm + r * rbx_video.pitch;
		uint8_t *pr = s_pri[r];
		// (MAME's eight draws in one: by category pair, layer 1 then layer 0; the even category priority 1, the odd 2)
		for (int x = 0; x < VIS_W; x++) {
			const uint16_t v1 = row[1][x], v0 = row[0][x];
			const int o1 = v1 & 0x1000, o0 = v0 & 0x1000;
			uint16_t pen = 0;
			uint8_t pv = 0;
			if (o1) {
				pv = (v1 & 0x2000) ? 2 : 1;
				pen = v1 & 0x7ff;
			}
			if (o0) {
				pv |= (v0 & 0x2000) ? 2 : 1;
				if (!o1 || (v0 >> 14) >= (v1 >> 14))   // (layer 0 over layer 1 unless in an earlier pair)
					pen = v0 & 0x7ff;
			}
			d[x] = pen;
			pr[x] = pv;
		}
	}
}

static const struct { uint8_t w, h; } s_sprsize[8] = { { 32, 32 }, { 16, 32 }, { 32, 16 }, { 64, 64 },
                                                       { 8, 8 }, { 16, 8 }, { 8, 16 }, { 16, 16 } };

// prio_zoom_transpen (drawgfxzoom_core), pmask 0xffcc: drawn where the priority is 0 or 1, which then becomes 31
static void draw_sprite(uint16_t *bm, uint32_t start, int w, int h, int color, int flipx, int flipy, int destx, int desty,
                        uint32_t scale)
{
	const int cl = 0, cr = VIS_W - 1, ct = VIS_Y0, cb = VIS_Y0 + VIS_H - 1;
	const uint32_t dstwidth = (scale * (uint32_t)w + 0x8000) >> 16, dstheight = (scale * (uint32_t)h + 0x8000) >> 16;
	if (dstwidth < 1 || dstheight < 1)
		return;
	int32_t dx = (w << 16) / (int32_t)dstwidth, dy = (h << 16) / (int32_t)dstheight;
	int destendx = destx + (int)dstwidth - 1;
	if (destx > cr || destendx < cl)
		return;
	int32_t srcx = 0, srcy = 0;
	if (destx < cl) {
		srcx = (cl - destx) * dx;
		destx = cl;
	}
	if (destendx > cr)
		destendx = cr;
	int destendy = desty + (int)dstheight - 1;
	if (desty > cb || destendy < ct)
		return;
	if (desty < ct) {
		srcy = (ct - desty) * dy;
		desty = ct;
	}
	if (destendy > cb)
		destendy = cb;
	if (flipx) {
		srcx = ((int32_t)dstwidth - 1) * dx - srcx;
		dx = -dx;
	}
	if (flipy) {
		srcy = ((int32_t)dstheight - 1) * dy - srcy;
		dy = -dy;
	}
	const uint16_t pen = (uint16_t)(color * 16);
	for (int y = desty; y <= destendy; y++, srcy += dy) {
		const uint8_t *row = s_charnib + ((start + (uint32_t)(srcy >> 16) * (uint32_t)w) & 0x1ffff);
		uint16_t *d = bm + (y - VIS_Y0) * rbx_video.pitch;
		uint8_t *p = s_pri[y - VIS_Y0];
		int32_t cx = srcx;
		for (int x = destx; x <= destendx; x++, cx += dx) {
			const int v = row[cx >> 16];
			if (!v)
				continue;
			if (p[x] < 2) {   // (0xffcc and the 1 << 31 of the drawn sprites: priorities 0 and 1 only)
				d[x] = (uint16_t)(pen + v);
				p[x] = 31;
			} else if (p[x] != 31)
				p[x] = 31;
		}
	}
}

static void draw_sprites(uint16_t *bm)
{
	// (priority 255 down to 0, each from the last sprite to the first)
	int order[256], n = 0;
	int count[256];
	memset(count, 0, sizeof(count));
	for (int a = 0x800 - 8; a >= 0; a -= 8)
		count[s_vs.spr[a] & 0xff]++;
	int startp[256], pos = 0;
	for (int p = 255; p >= 0; p--) {
		startp[p] = pos;
		pos += count[p];
	}
	for (int a = 0x800 - 8; a >= 0; a -= 8)
		order[startp[s_vs.spr[a] & 0xff]++] = a;
	n = pos;
	const int flip = s_vs.flip & 1;
	for (int i = 0; i < n; i++) {
		const uint16_t *s = s_vs.spr + order[i];
		int zoom = s[2] & 0xff;
		int code;
		if (!(s[2] & 0xff00) && ((s[3] & 0xff00) != 0xff00))
			code = s[3] + ((s[4] & 0xc0) << 2);
		else
			code = (s[3] & 0xff) + ((s[4] & 0xc0) << 2);
		if (zoom == 0xff && code == 0)
			continue;
		const int size = s[1];
		zoom += (size & 0xc0) << 2;
		int sx = s[5] & 0xff, sy = s[6] & 0xff;
		if (s[4] & 1)
			sx -= 0x100;
		const int color = (s[4] & 0x1e) >> 1;
		int flipx = s[1] & 1, flipy = (s[4] >> 5) & 1;
		const int idx = (size >> 3) & 7, w = s_sprsize[idx].w, h = s_sprsize[idx].h;
		code = code * 8 * 16 / (w * h);
		if (!zoom)
			continue;
		const uint32_t scale = (uint32_t)((1 << 16) * 0x80 / zoom) + 0x02ab;
		if (flip) {
			sx = 256 - (int)((scale * (uint32_t)w) >> 16) - sx;
			sy = 256 - (int)((scale * (uint32_t)h) >> 16) - sy;
			flipx = !flipx;
			flipy = !flipy;
		}
		draw_sprite(bm, (uint32_t)code * (uint32_t)(w * h), w, h, color, flipx, flipy, sx, sy, scale);
	}
}

static void nm_render(void)
{
	for (int i = 0; i < 2048; i++) {
		int r, g, b;
		if (s_m->pal_resnet) {   // (palette_w)
			const uint16_t v = s_vs.palw[i];
			r = s_resnet[v & 31];
			g = s_resnet[(v >> 5) & 31];
			b = s_resnet[(v >> 10) & 31];
		} else {   // (xBGR_555, membits 8: the high byte first)
			const uint16_t v = (uint16_t)(s_vs.pal8[2 * i] << 8 | s_vs.pal8[2 * i + 1]);
			r = (v & 31) << 3 | (v & 31) >> 2;
			g = ((v >> 5) & 31) << 3 | ((v >> 5) & 31) >> 2;
			b = ((v >> 10) & 31) << 3 | ((v >> 10) & 31) >> 2;
		}
		rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	uint16_t *bm = rbx_video.bitmap;
	draw_tilemaps(bm);
	draw_sprites(bm);
}

// ---- the latches ----
static void irq_enable(int level, int state)
{
	uint8_t *on = level == 1 ? &t.irq1_on : level == 2 ? &t.irq2_on : &t.irq4_on;
	*on = (uint8_t)(state != 0);
	if (!state)
		m68k_bus_set_line(level, 0);
}
static void sound_irq_w(int state)
{
	if (state && !t.sound_irq_clk)
		t.z80_irq = 1;   // (HOLD_LINE)
	t.sound_irq_clk = (uint8_t)(state != 0);
}
static void outlatch_bit(int bit, int state)   // (the LS259s of the 68000 boards: outlatch)
{
	if (bit == 2)
		sound_irq_w(state);
	else if (bit == 7 && (s_mach == M_GX400 || s_mach == M_GRADIUS || s_mach == M_RF2))
		irq_enable(4, state);
}
static void intlatch_bit(int bit, int state)
{
	if (s_mach == M_NEMESIS) {
		if (bit == 0)
			irq_enable(1, state);
	} else if (bit == 0)
		irq_enable(2, state);
	else if (bit == 1)
		irq_enable(1, state);
	if (bit == 2)
		t.flipx = (uint8_t)(state != 0);
	else if (bit == 3)
		t.flipy = (uint8_t)(state != 0);
}
static void salamand_intlatch_w(uint8_t data)
{
	irq_enable(1, data & 1);
	irq_enable(2, (data >> 1) & 1);
	t.flipx = (data >> 2) & 1;
	t.flipy = (data >> 3) & 1;
}
static void blkpnthr_intlatch_w(uint8_t data) { salamand_intlatch_w((uint8_t)((data & 0xfc) | (data & 1) << 1 | (data >> 1 & 1))); }
static void outlatch_w(uint8_t data) { sound_irq_w((data >> 3) & 1); }

static uint16_t konamigt_input(void)
{
	const int data = (int)rbx_port_read(s_port_in3), data2 = (int)rbx_port_read(s_port_wheel);
	int ret = 0;
	if (data & 0x20)
		ret |= 0x0300;
	if (data & 0x40)
		ret |= 0xf000;
	return (uint16_t)(ret | (data2 & 0x7f));
}
static uint8_t selected_ip_r(void)
{
	switch (t.selected_ip & 0xf) {
	case 0xc: case 0: return (uint8_t)rbx_port_read(s_port_accel);
	case 0xd: case 1: return (uint8_t)rbx_port_read(s_port_wheel);
	default: return 0xff;
	}
}

static void chr_w(uint32_t o, uint16_t v, uint16_t mask)
{
	const uint16_t old = t.chr[o];
	t.chr[o] = (uint16_t)((old & ~mask) | (v & mask));
	if (old != t.chr[o]) {
		s_chdirty[o >> 6] = 1;
		s_any_dirty = 1;
	}
}

// ---- the main CPU's bus: the ports and handlers (the RAM blocks are mapped) ----
static uint16_t port(int i) { return (uint16_t)rbx_port_read(s_ports[i]); }
enum { IN0, IN1, IN2, DSW0, DSW1, TEST };

static uint16_t main_r16(uint32_t a)
{
	a &= 0xffffff;
	switch (s_mach) {
	case M_NEMESIS: case M_KONAMIGT:
		switch (a) {
		case 0x05c400: return port(DSW0);
		case 0x05c402: return port(DSW1);
		case 0x05cc00: return port(IN0);
		case 0x05cc02: return port(IN1);
		case 0x05cc04: return port(IN2);
		case 0x05cc06: return port(TEST);
		case 0x070000: return s_mach == M_KONAMIGT ? konamigt_input() : 0;
		}
		break;
	case M_GX400: case M_GRADIUS: case M_RF2:
		if (a >= 0x020000 && a < 0x028000)
			return t.shared[(a - 0x020000) >> 1];
		if (a >= 0x080000 && a < 0x0c0000 && s_maindata) {
			const uint32_t o = a - 0x080000;
			return o + 1 < s_maindata_size ? (uint16_t)(s_maindata[o] << 8 | s_maindata[o + 1]) : 0;
		}
		switch (a) {
		case 0x05c402: return port(DSW0);
		case 0x05c404: return port(DSW1);
		case 0x05c406: return port(TEST);
		case 0x05cc00: return port(IN0);
		case 0x05cc02: return port(IN1);
		case 0x05cc04: return port(IN2);
		case 0x070000: return s_mach == M_RF2 ? konamigt_input() : 0;
		}
		break;
	case M_SALAMAND: case M_BLKPNTHR:
		switch (a) {
		case 0x0c0002: return port(DSW0);
		case 0x0c2000: return port(IN0);
		case 0x0c2002: return port(IN1);
		case 0x0c2004: return port(IN2);
		case 0x0c2006: return port(DSW1);
		}
		break;
	case M_CITYBOMB: case M_NYANPANI: {
		const uint32_t b = s_mach == M_CITYBOMB ? 0x0f0000 : 0x070000;
		if (a == b) return port(DSW1);
		if (a == b + 2) return port(IN2);
		if (a == b + 4) return port(IN1);
		if (a == b + 6) return port(IN0);
		if (a == b + 8) return port(DSW0);
		if (s_mach == M_CITYBOMB && a == 0x0f0020) return selected_ip_r();
		const uint32_t db = s_mach == M_CITYBOMB ? 0x100000 : 0x100000, de = s_mach == M_CITYBOMB ? 0x1c0000 : 0x140000;
		if (a >= db && a < de && s_maindata) {
			const uint32_t o = a - db;
			return o + 1 < s_maindata_size ? (uint16_t)(s_maindata[o] << 8 | s_maindata[o + 1]) : 0;
		}
		break;
	}
	case M_HCRASH:
		switch (a) {
		case 0x0c0002: return port(DSW0);
		case 0x0c0004: return port(DSW1);
		case 0x0c0006: return port(TEST);
		case 0x0c000a: return port(IN0);
		case 0x0c2000: return konamigt_input();
		case 0x0c4000: return port(IN1);
		case 0x0c4002: return selected_ip_r();
		}
		if (a >= 0x040000 && a < 0x060000 && s_maindata) {
			const uint32_t o = a - 0x040000;
			return o + 1 < s_maindata_size ? (uint16_t)(s_maindata[o] << 8 | s_maindata[o + 1]) : 0;
		}
		break;
	}
	if (!s_m->pal_resnet && a >= s_m->pal && a < s_m->pal + 0x2000)
		return t.pal8[(a - s_m->pal) >> 1];
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
	if (a >= s_m->chr && a < s_m->chr + 0x10000) {
		chr_w((a - s_m->chr) >> 1, v, mask);
		return;
	}
	if (s_m->pal_resnet && a >= s_m->pal && a < s_m->pal + 0x1000) {
		uint16_t *p = &t.palw[(a - s_m->pal) >> 1];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
		return;
	}
	if (!s_m->pal_resnet && a >= s_m->pal && a < s_m->pal + 0x2000) {
		if (mask & 0x00ff)
			t.pal8[(a - s_m->pal) >> 1] = (uint8_t)v;
		return;
	}
	const uint8_t lo = (uint8_t)v, hi = (uint8_t)(v >> 8);
	switch (s_mach) {
	case M_NEMESIS: case M_KONAMIGT: case M_GX400: case M_GRADIUS: case M_RF2:
		if (a == 0x05c000 && (mask & 0x00ff))
			t.soundlatch = lo;
		else if (a >= 0x05e000 && a < 0x05e010) {   // (outlatch: umask ff00, intlatch: umask 00ff; write_d0)
			const int bit = (int)((a - 0x05e000) >> 1);
			if (mask & 0xff00)
				outlatch_bit(bit, hi & 1);
			if (mask & 0x00ff)
				intlatch_bit(bit, lo & 1);
		} else if (s_mach >= M_GX400 && a >= 0x020000 && a < 0x028000 && (mask & 0x00ff))
			t.shared[(a - 0x020000) >> 1] = lo;
		break;
	case M_SALAMAND: case M_BLKPNTHR: case M_HCRASH:
		if (a == 0x0a0000) {
			if (mask & 0xff00)
				outlatch_w(hi);
			if (mask & 0x00ff) {
				if (s_mach == M_SALAMAND)
					salamand_intlatch_w(lo);
				else
					blkpnthr_intlatch_w(lo);
			}
		} else if (a == 0x0c0000 && (mask & 0x00ff))
			t.soundlatch = lo;
		else if (s_mach == M_HCRASH && a == 0x0c4000 && (mask & 0x00ff))
			t.selected_ip = lo;
		break;
	case M_CITYBOMB: case M_NYANPANI: {
		const uint32_t b = s_mach == M_CITYBOMB ? 0x0f0000 : 0x070000, l = s_mach == M_CITYBOMB ? 0x0f8000 : 0x078000;
		if (a == b + 0x10 && (mask & 0x00ff))
			t.soundlatch = lo;
		else if (a == l) {
			if (mask & 0xff00) {
				outlatch_w(hi);
				if (s_mach == M_CITYBOMB)   // (citybomb_outlatch_w: the steering and accel)
					t.selected_ip = (uint8_t)(((~hi) >> 4) & 1);
			}
			if (mask & 0x00ff)
				salamand_intlatch_w(lo);
		}
		break;
	}
	}
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v) { main_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00); }

// ---- the sound board ----
static void k007232_port(uint8_t data)   // (volume_callback)
{
	k007232_set_volume(&s_k232, 0, (data >> 4) * 0x11, 0);
	k007232_set_volume(&s_k232, 1, 0, (data & 0x0f) * 0x11);
}
static uint8_t ay1_port_a(void *u)   // (nemesis_portA_r)
{
	(void)u;
	int res = (int)((s_z.cyc / 512) & 0x0f) | 0xd0;
	if (s_m->sound == SND_GX400 && vlm5030_bsy(&s_vlm))
		res |= 0x20;
	return (uint8_t)res;
}
static void ay2_port_a(void *u, uint8_t d) { (void)u; k005289_control_a_w(&s_k5289, d); }
static void ay2_port_b(void *u, uint8_t d) { (void)u; k005289_control_b_w(&s_k5289, d); }

static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	switch (s_m->sound) {
	case SND_NEMESIS: case SND_GX400:
		if (s_m->sound == SND_NEMESIS) {
			if (a < 0x4000)
				return a < s_zrom_size ? s_zrom[a] : 0;
			if (a < 0x4800)
				return t.zram[a - 0x4000];
		} else {
			if (a < 0x2000)
				return a < s_zrom_size ? s_zrom[a] : 0;
			if (a >= 0x4000 && a < 0x8000)
				return t.shared[a - 0x4000];
			if (a >= 0x8000 && a < 0x8800)
				return t.voiceram[a - 0x8000];
		}
		if (a == 0xe001)
			return t.soundlatch;
		if (a == 0xe086)
			return ay8910_data_r(&s_ay[0]);
		if (a == 0xe205)
			return ay8910_data_r(&s_ay[1]);
		return 0;
	default:
		if (a < 0x8000)
			return a < s_zrom_size ? s_zrom[a] : 0;
		if (a < 0x8800)
			return t.zram[a - 0x8000];
		if (s_m->sound == SND_CITY) {
			if (a >= 0x9800 && a < 0x9900)
				return k051649_scc_r(&s_scc, a & 0xff, 0);
			if (a == 0xa000 || a == 0xa001)
				return rbx_ym_read(0, a & 1);
			if (a >= 0xb000 && a < 0xb00e)
				return k007232_read(&s_k232, a - 0xb000);
			if (a == 0xd000)
				return t.soundlatch;
			return 0;
		}
		if (a == 0xa000)
			return t.soundlatch;
		if (a >= 0xb000 && a < 0xb00e)
			return k007232_read(&s_k232, a - 0xb000);
		if (a == 0xc000 || a == 0xc001)
			return rbx_ym_read(0, a & 1);
		if (a == 0xe000 && s_m->sound == SND_SALAMAND)
			return (uint8_t)vlm5030_bsy(&s_vlm);
		return 0;
	}
}
static void speech_w(int offset, uint8_t data)   // (gx400 speech_w)
{
	if (!(offset & 8) && (t.speech_offset & 8))
		vlm5030_data_w(&s_vlm, data);
	vlm5030_st(&s_vlm, (offset >> 5) & 1);
	vlm5030_rst(&s_vlm, (offset >> 6) & 1);
	t.speech_offset = (uint8_t)offset;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	switch (s_m->sound) {
	case SND_NEMESIS: case SND_GX400:
		if (s_m->sound == SND_NEMESIS && a >= 0x4000 && a < 0x4800)
			t.zram[a - 0x4000] = v;
		else if (s_m->sound == SND_GX400 && a >= 0x4000 && a < 0x8000)
			t.shared[a - 0x4000] = v;
		else if (s_m->sound == SND_GX400 && a >= 0x8000 && a < 0x8800)
			t.voiceram[a - 0x8000] = v;
		else if (a >= 0xa000 && a < 0xb000)
			k005289_ld1_w(&s_k5289, a - 0xa000);
		else if (a >= 0xc000 && a < 0xd000)
			k005289_ld2_w(&s_k5289, a - 0xc000);
		else if (s_m->sound == SND_GX400 && (a & ~0x78) == 0xe000)
			speech_w(a & 0x78, v);
		else if (a == 0xe003)
			k005289_tg1_w(&s_k5289);
		else if (a == 0xe004)
			k005289_tg2_w(&s_k5289);
		else if (a == 0xe005)
			ay8910_address_w(&s_ay[1], v);
		else if (a == 0xe006)
			ay8910_address_w(&s_ay[0], v);
		else if (a == 0xe106)
			ay8910_data_w(&s_ay[0], v);
		else if (a == 0xe405)
			ay8910_data_w(&s_ay[1], v);
		break;   // (e007: the RC filters, not emulated)
	default:
		if (a >= 0x8000 && a < 0x8800)
			t.zram[a - 0x8000] = v;
		else if (s_m->sound == SND_CITY) {
			if (a >= 0x9800 && a < 0x9900)
				k051649_scc_w(&s_scc, a & 0xff, v);
			else if (a == 0xa000 || a == 0xa001)
				rbx_ym_write(0, a & 1, v);
			else if (a >= 0xb000 && a < 0xb00e)
				k007232_write(&s_k232, a - 0xb000, v);
			else if (a == 0xc000) {   // (city_sound_bank_w)
				t.k232_bank = v;
				k007232_set_bank(&s_k232, v & 3, (v >> 2) & 3);
			}
		} else if (a >= 0xb000 && a < 0xb00e)
			k007232_write(&s_k232, a - 0xb000, v);
		else if (a == 0xc000 || a == 0xc001)
			rbx_ym_write(0, a & 1, v);
		else if (s_m->sound == SND_SALAMAND && a == 0xd000)
			vlm5030_data_w(&s_vlm, v);
		else if (s_m->sound == SND_SALAMAND && a == 0xf000) {   // (speech_start_w)
			vlm5030_rst(&s_vlm, v & 1);
			vlm5030_st(&s_vlm, (v >> 1) & 1);
		}
		break;
	}
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// n samples of the line: the Z80 for their time, then the chips (at their own rates, the last value held), mixed
static void sound_line(int n)
{
	if (n <= 0)
		return;
	t.z_frac += (uint64_t)s_m->z80_clock * (uint64_t)n;
	const unsigned long cyc = (unsigned long)(t.z_frac / SND_RATE);
	t.z_frac -= (uint64_t)cyc * SND_RATE;
	int line = t.z80_irq;
	t.z_end += cyc;
	z80_run(&s_z, (unsigned long)t.z_end, &line, 1);
	t.z80_irq = (uint8_t)line;
	if (n > 64)
		n = 64;
	int32_t l[64], r[64];
	memset(l, 0, sizeof(l));
	memset(r, 0, sizeof(r));
	if (s_m->sound == SND_NEMESIS || s_m->sound == SND_GX400) {
		// (AY 1: one output at 0.20, AY 2: its three outputs at 1.0, the K005289 0.35 / 0.60, the VLM5030 0.70)
		int32_t a1[96], a2[96], kk[64];
		memset(a1, 0, sizeof(a1));
		memset(a2, 0, sizeof(a2));
		t.ay_frac += 1789772ull * (uint64_t)n;
		const uint32_t ac = (uint32_t)(t.ay_frac / SND_RATE);
		t.ay_frac -= (uint64_t)ac * SND_RATE;
		const int n1 = ay8910_render(&s_ay[0], ac, a1, 96), n2 = ay8910_render(&s_ay[1], ac, a2, 96);
		k005289_render(&s_k5289, kk, n);
		const int kg = (s_mach == M_KONAMIGT || s_mach == M_RF2) ? 60 : 35;
		for (int i = 0; i < n; i++) {
			int32_t m = (n1 > 0 ? a1[i * n1 / n] : 0) + (n2 > 0 ? a2[i * n2 / n] : 0) + kk[i] * kg / 100;
			if (s_m->sound == SND_GX400) {
				t.vlm_frac += vlm5030_rate(&s_vlm);
				while (t.vlm_frac >= SND_RATE) {
					t.vlm_frac -= SND_RATE;
					int16_t s;
					vlm5030_render(&s_vlm, &s, 1);
					t.vlm_out = s;
				}
				m += t.vlm_out * 70 / 100;
			}
			l[i] = r[i] = m;
		}
	} else {
		int32_t yl[96], yr[96], ym[96];
		memset(yl, 0, sizeof(yl));
		memset(yr, 0, sizeof(yr));
		memset(ym, 0, sizeof(ym));
		t.ym_frac += 3579545ull * (uint64_t)n;
		const uint32_t yc = (uint32_t)(t.ym_frac / SND_RATE);
		t.ym_frac -= (uint64_t)yc * SND_RATE;
		const int yn = s_m->sound == SND_CITY ? rbx_ym_advance(0, yc, ym, 96) : rbx_ym_advance_lr(0, yc, yl, yr, 96);
		for (int i = 0; i < n; i++) {
			t.k232_frac += 3579545 / 128;
			while (t.k232_frac >= SND_RATE) {
				t.k232_frac -= SND_RATE;
				k007232_sample(&s_k232, &t.k232_a, &t.k232_b);
			}
			const int k = yn > 0 ? i * yn / n : -1;
			const int32_t kp = t.k232_a + t.k232_b;
			if (s_m->sound == SND_CITY) {   // (mono: K007232 0.30, YM3812 1.0, K051649 0.38)
				k051649_render(&s_scc, &t.scc_out, 1);
				l[i] = r[i] = (k >= 0 ? ym[k] : 0) + kp * 30 / 100 + t.scc_out * 38 / 100;
				continue;
			}
			int32_t v = 0;
			if (s_m->sound == SND_SALAMAND) {
				t.vlm_frac += vlm5030_rate(&s_vlm);
				while (t.vlm_frac >= SND_RATE) {
					t.vlm_frac -= SND_RATE;
					int16_t s;
					vlm5030_render(&s_vlm, &s, 1);
					t.vlm_out = s;
				}
				v = t.vlm_out * (s_mach == M_HCRASH ? 200 : 250) / 100;
			}
			// (salamand: K007232 0.08, the YM2151's sides swapped; blkpnthr 0.10; hcrash 0.10)
			const int kg = s_mach == M_SALAMAND ? 8 : 10;
			l[i] = (k >= 0 ? (s_mach == M_SALAMAND ? yr[k] : yl[k]) : 0) + kp * kg / 100 + v;
			r[i] = (k >= 0 ? (s_mach == M_SALAMAND ? yl[k] : yr[k]) : 0) + kp * kg / 100 + v;
		}
	}
	for (int i = 0; i < n && s_snd_n < 2048; i++, s_snd_n++) {
		s_snd[0][s_snd_n] = l[i];
		s_snd[1][s_snd_n] = r[i];
	}
}

// ---- init / frame ----
static void nm_exit(void);
static void nm_reset(void);

static int nm_init(const rbx_game *g)
{
	s_mach = -1;
	for (int i = 0; i < (int)(sizeof(s_machs) / sizeof(s_machs[0])); i++)
		if (!strcmp(s_machs[i].name, g->machine))
			s_mach = i;
	if (s_mach < 0)
		return -1;
	s_m = &s_machs[s_mach];
	uint32_t k5_size = 0, pcm_size = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_maindata = rbx_region("maindata", &s_maindata_size);
	const uint8_t *k5prom = rbx_region("k005289", &k5_size);
	const uint8_t *pcm = rbx_region("k007232", &pcm_size);
	s_vlmrom = rbx_region("vlm", &s_vlmrom_size);
	const int sn = s_m->sound;
	if (!s_rom || !s_zrom || ((sn == SND_NEMESIS || sn == SND_GX400) && !k5prom) ||
	    (sn >= SND_SALAMAND && !pcm) || (sn == SND_SALAMAND && !s_vlmrom)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	m68k_init();
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0x000000, s_m->rom_end, s_rom, s_rom_size, 1, 0);
	for (int w = 0; w < 2; w++) {
		m68k_bus_map(s_m->xs[w], s_m->xs[w] + 0x3ff, t.xs[w], sizeof(t.xs[w]), 1, 1);
		m68k_bus_map(s_m->ys[w], s_m->ys[w] + 0x7f, t.ys[w], sizeof(t.ys[w]), 1, 1);
		m68k_bus_map(s_m->vr[w], s_m->vr[w] + 0xfff, t.vr[w], sizeof(t.vr[w]), 1, 1);
		m68k_bus_map(s_m->cr[w], s_m->cr[w] + 0xfff, t.cr[w], sizeof(t.cr[w]), 1, 1);
	}
	m68k_bus_map(s_m->chr, s_m->chr + 0xffff, t.chr, sizeof(t.chr), 1, 0);
	m68k_bus_map(s_m->spr, s_m->spr + 0xfff, t.spr, sizeof(t.spr), 1, 1);
	if (s_m->pal_resnet)
		m68k_bus_map(s_m->pal, s_m->pal + 0xfff, t.palw, sizeof(t.palw), 1, 0);
	for (int i = 0; i < 3; i++)
		m68k_bus_map(s_m->ram[i][0], s_m->ram[i][1], t.ram[i], s_m->ram[i][1] - s_m->ram[i][0] + 1, 1, 1);
	if (s_mach == M_GX400 || s_mach == M_GRADIUS || s_mach == M_RF2 || s_mach == M_NEMESIS || s_mach == M_KONAMIGT)
		m68k_bus_map(0x057000, 0x057fff, t.extra, 0x1000, 1, 1);   // (needed for twinbee)
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
	if (sn == SND_NEMESIS || sn == SND_GX400) {
		ay8910_init(&s_ay[0], AY8910_TYPE_AY8910, 1789772, AY8910_LEGACY_OUTPUT | AY8910_SINGLE_OUTPUT, 51, 4, 0);
		ay8910_init(&s_ay[1], AY8910_TYPE_AY8910, 1789772, AY8910_LEGACY_OUTPUT, 256, 4, 0);
		s_ay[0].port_a_read = ay1_port_a;
		s_ay[1].port_a_write = ay2_port_a;
		s_ay[1].port_b_write = ay2_port_b;
		k005289_init(&s_k5289, 3579545, k5prom);
		k005289_set_rate(&s_k5289, SND_RATE);
		if (sn == SND_GX400)
			vlm5030_init(&s_vlm, 3579545, t.voiceram, sizeof(t.voiceram));
	} else {
		k007232_init(&s_k232, pcm, pcm_size, k007232_port);
		if (sn == SND_SALAMAND)
			vlm5030_init(&s_vlm, 3579545, s_vlmrom, s_vlmrom_size);
		if (sn == SND_CITY) {
			k051649_init(&s_scc, 3579545);
			k051649_set_rate(&s_scc, SND_RATE);
		}
		rbx_ym_config yc;
		memset(&yc, 0, sizeof(yc));
		yc.type = sn == SND_CITY ? RBX_YM3812 : RBX_YM2151;
		yc.clock = 3579545;
		// (salamand 1.2, blkpnthr 1.0, hcrash 0.50, YM3812 1.0)
		yc.gain[0] = yc.gain[1] = s_mach == M_SALAMAND ? 307 : s_mach == M_HCRASH ? 128 : 256;
		if (rbx_ym_init(0, &yc)) {
			rbx_fail("%s: YM", g->set);
			return -1;
		}
	}
	static const char *const pn[6] = { "IN0", "IN1", "IN2", "DSW0", "DSW1", "TEST" };
	for (int i = 0; i < 6; i++)
		s_ports[i] = rbx_port_find(pn[i]);
	s_port_in3 = rbx_port_find("IN3");
	s_port_wheel = rbx_port_find("WHEEL");
	s_port_accel = rbx_port_find("ACCEL");
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, 2048)) {
		rbx_fail("%s: out of memory", g->set);
		nm_exit();
		return -1;
	}
	rbx_audio.frame_mhz = (uint32_t)((uint64_t)PIX_CLOCK * 1000 / (HTOTAL * LINES));   // (60.61 Hz: a frame's sound in a 60 Hz frame)
	nm_reset();
	return 0;
}

static void nm_exit(void)
{
	if (s_m && s_m->sound >= SND_SALAMAND)
		rbx_ym_exit(0);
}

static void nm_reset(void)
{
	rbx_video_wait_idle();
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	memset(s_charnib, 0, sizeof(s_charnib));
	memset(s_chdirty, 0, sizeof(s_chdirty));
	s_any_dirty = 0;
	s_snd_n = 0;
	const int sn = s_m->sound;
	if (sn == SND_NEMESIS || sn == SND_GX400) {
		ay8910_reset(&s_ay[0]);
		ay8910_reset(&s_ay[1]);
		k005289_reset(&s_k5289);
		if (sn == SND_GX400)
			vlm5030_reset(&s_vlm);
	} else {
		k007232_reset(&s_k232);
		if (sn == SND_SALAMAND)
			vlm5030_reset(&s_vlm);
		if (sn == SND_CITY)
			k051649_reset(&s_scc);
		rbx_ym_reset(0);
	}
	z80_init(&s_z);
	s_z.read_byte = z_rd;
	s_z.write_byte = z_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	memset(s_zmap, 0, sizeof(s_zmap));
	const int romp = sn == SND_NEMESIS ? 0x40 : sn == SND_GX400 ? 0x20 : 0x80;
	for (int p = 0; p < romp; p++)
		s_zmap[p] = (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
	s_z.read_map = s_zmap;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
}

static void nm_frame(void)
{
	const int irq = s_m->irq;
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update)
			rbx_video_wait_idle();
			capture();
			rbx_video_frame_done();
			if (irq == IRQ_VBL1 && t.irq1_on)
				m68k_bus_set_line(1, 1);
			if (irq == IRQ_VBL2 && t.irq2_on)
				m68k_bus_set_line(2, 1);
			if (s_m->sound == SND_GX400)   // (screen_vblank to the sound CPU's NMI)
				z80_gen_nmi(&s_z);
		}
		if (irq >= IRQ_GX400) {   // (the scanline timers)
			const int even = irq == IRQ_HCRASH || !(t.frame & 1);
			if (vpos == 240 && t.irq1_on && even)
				m68k_bus_set_line(1, 1);
			if (vpos == 0 && t.irq2_on)
				m68k_bus_set_line(2, 1);
			if (irq == IRQ_GX400 && vpos == 120 && t.irq4_on)
				m68k_bus_set_line(4, 1);
		}
		t.cpu_frac += (uint64_t)s_m->cpu_clock * HTOTAL;
		int cyc = (int)(t.cpu_frac / PIX_CLOCK);
		t.cpu_frac -= (uint64_t)cyc * PIX_CLOCK;
		while (cyc > 0)
			cyc -= m68k_execute(cyc);
		t.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(t.snd_frac / PIX_CLOCK);
		t.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		sound_line(ns);
	}
	for (int i = 0; i < s_snd_n; i++)
		rbx_audio_put_stereo(s_snd[0][i], s_snd[1][i]);
	s_snd_n = 0;
	t.frame++;
}

static int nm_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	int rc = 0;
	rbx_video_wait_idle();
	if ((uint32_t)m68k_context_size() > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	rc |= rbx_state_io(ctx, "nm_cpu", cpu, (uint32_t)m68k_context_size());
	rc |= rbx_state_io(ctx, "nm_bus", &m68k_bus.irq_pending, sizeof(uint32_t) * 2);
	rc |= rbx_state_io(ctx, "nm", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	const int sn = s_m->sound;
	if (sn == SND_NEMESIS || sn == SND_GX400) {
		rc |= rbx_state_io(ctx, "nm_ay1", &s_ay[0], (uint32_t)AY8910_STATE_END);
		rc |= rbx_state_io(ctx, "nm_ay2", &s_ay[1], (uint32_t)AY8910_STATE_END);
		rc |= k005289_state(&s_k5289, ctx, load, "nm_k5289");
	} else {
		rc |= k007232_state(&s_k232, ctx, load, "nm_k232");
		rc |= rbx_ym_state(0, ctx, load);
		if (sn == SND_CITY)
			rc |= k051649_state(&s_scc, ctx, load, "nm_scc");
	}
	if (sn == SND_GX400 || sn == SND_SALAMAND)
		rc |= vlm5030_state(&s_vlm, ctx, load, "nm_vlm");
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "nm_z80", &z, sizeof(z));
	if (!rc && load) {
		m68k_set_context(cpu);
		memset(s_chdirty, 1, sizeof(s_chdirty));
		decode_chars();
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

static int nm_save(void *ctx) { return nm_state(ctx, 0); }
static int nm_load(void *ctx) { return nm_state(ctx, 1); }
static int nm_supported(const char *machine)
{
	for (int i = 0; i < (int)(sizeof(s_machs) / sizeof(s_machs[0])); i++)
		if (!strcmp(s_machs[i].name, machine))
			return 1;
	return 0;
}

const rbx_driver rbx_drv_nemesis = { "Konami Nemesis", nm_init, nm_exit, nm_reset, nm_frame, nm_save, nm_load,
                                     nm_supported, nm_render, NULL };
