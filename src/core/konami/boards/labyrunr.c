/*  Konami Labyrinth Runner / Trick Trap. Ported from MAME src/mame/konami/labyrunr.cpp (BSD-3-Clause,
    copyright-holders: Nicola Salmoria) and the devices it uses (k007121.cpp, k051733.cpp). An HD6309E at 3 MHz, one
    K007121 (its two 32x32 tilemaps, the playfield with column scroll, as the driver's), a K051733, the indirect palette
    through the colour PROM; the sound: two YM2203s on the main CPU (no sound CPU), made on this thread.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/hd6309/hd6309.h"
#include "../../sound/rbx_ym.h"
#include "../machine/k051733.h"
#include "../video/k007121.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 3000000u     // (24 MHz / 8)
#define YM_CLOCK 3000000u
#define SND_RATE 48000
#define PIX_CLOCK 6000000u     // (set_raw(24 MHz / 4, 384, 0, 280, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_X0 0
#define VIS_W 280
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 2048             // (8 * 16 * 16)

static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t *s_gfx;
static uint32_t s_gfx_count;
static uint16_t s_ctab[NPENS];
static uint16_t s_transmask[128];
static hd6309_t s_cpu;
static k007121_t s_k121;
static k051733_t s_k733;
static int s_port_system, s_port_p[2], s_port_dsw[3];
static uint8_t s_pri[VIS_W * VIS_H];
static int32_t s_out[2048];
static int s_out_pos;

static struct {
	uint8_t pal[0x100];      // 1000-10ff
	uint8_t ram[0x800];      // 1800-1fff
	uint8_t spr[0x1000];     // 2000-2fff
	uint8_t vram[2][0x800];  // 3000-37ff, 3800-3fff
	uint8_t mainbank;
	int64_t cycles_done;
	int64_t cpu_end, snd_end;   // (the CPUs' line targets: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, ym_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t pal[0x100], vram[2][0x800]; k007121_t k121; } s_vs;

static void capture(void)
{
	memcpy(s_vs.pal, t.pal, sizeof(s_vs.pal));
	memcpy(s_vs.vram, t.vram, sizeof(s_vs.vram));
	memcpy(&s_vs.k121, &s_k121, sizeof(k007121_t));
}

typedef struct { int x0, x1; } clip_t;   // (the clips' x ranges, the screen's coordinates; empty when x0 > x1)

// a tilemap (get_tile_info): cat -1 any; column scroll from the chip's scroll RAM when cols
static void draw_layer(int which, int scrollx, int scrolly, int cols, int cat, int opaque, clip_t c)
{
	const uint8_t *v = s_vs.vram[which], *k = s_vs.k121.ctrl;
	const int bit0 = k[5] & 3, bit1 = (k[5] >> 2) & 3, bit2 = (k[5] >> 4) & 3, bit3 = (k[5] >> 6) & 3;
	const int mask = (k[4] & 0xf0) >> 4, colbase = (k[6] & 0x30) * 2 + 16;
	uint16_t *bm = rbx_video.bitmap;
	if (c.x0 < VIS_X0)
		c.x0 = VIS_X0;
	if (c.x1 > VIS_X0 + VIS_W - 1)
		c.x1 = VIS_X0 + VIS_W - 1;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * rbx_video.pitch - VIS_X0;
		for (int x = c.x0; x <= c.x1;) {
			const int tx = (x + scrollx) & 255;
			int n = 8 - (tx & 7);
			if (n > c.x1 - x + 1)
				n = c.x1 - x + 1;
			const int ty = (y + (cols ? s_vs.k121.scroll[((tx >> 3) - 2) & 0x1f] : scrolly)) & 255;
			const int idx = (ty >> 3) * 32 + (tx >> 3), attr = v[idx];
			const int tc = which ? 0 : (attr & 0x40) >> 6;
			if (cat < 0 || tc == cat) {
				int bank = ((attr >> (bit0 + 3)) & 1) | ((attr >> (bit1 + 2)) & 2) | ((attr >> (bit2 + 1)) & 4) | ((attr >> bit3) & 8);
				bank = (bank & ~mask) | (k[4] & mask);
				bank = ((attr & 0x80) >> 7) | (bank << 1) | ((k[3] & 1) << 5);
				const uint32_t code = (uint32_t)(v[idx + 0x400] + bank * 256) % s_gfx_count;
				const uint8_t *src = s_gfx + code * 64 + (ty & 7) * 8 + (tx & 7);
				const int pb = (colbase + (attr & 7)) * 16;
				for (int i = 0; i < n; i++) {
					const int p = src[i];
					if (p || opaque)
						d[x + i] = (uint16_t)(pb + p);
				}
			}
			x += n;
		}
	}
}

static void sprite_cb(int *code, int *color, int colbank) { (void)code; *color += colbank; }

static void lr_render(void)   // (screen_update)
{
	for (int i = 0; i < NPENS; i++) {   // (the indirect palette: xBGR_555, the high byte first)
		const int e = s_ctab[i];
		const uint16_t v = (uint16_t)(s_vs.pal[2 * e] << 8 | s_vs.pal[2 * e + 1]);
		const int r = v & 31, g = (v >> 5) & 31, b = (v >> 10) & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (b << 3 | b >> 2));
	}
	k007121_t *k = &s_vs.k121;
	const uint8_t *c = k->ctrl;
	const int bgpen = (c[6] & 0x30) * 2 + 16;
	for (int y = 0; y < VIS_H; y++)
		for (int x = 0; x < VIS_W; x++)
			rbx_video.bitmap[y * rbx_video.pitch + x] = (uint16_t)(bgpen << 4);
	memset(s_pri, 0, sizeof(s_pri));
	const int vx0 = VIS_X0, vx1 = VIS_X0 + VIS_W - 1, flip = k->flipscreen;
	clip_t clip[3] = { { vx0, vx1 }, { vx0, vx1 }, { vx0, vx1 } };
	int sx0, sx1, sy0, sy1, cols = 0;
	if (c[3] & 0x10) {   // (the 5 columns of tilemap 1 at the side)
		if (flip) {
			clip[0].x1 -= 40;
			clip[1].x0 = clip[1].x1 - 39;
		} else {
			clip[0].x0 += 40;
			clip[1].x1 = 39;
		}
		clip[2] = clip[0];
		sx0 = c[0] - 40;
		sx1 = sy1 = 0;
		k->spr_dx = 40;
		k->spr_flip_dx = 16;
		cols = (c[1] & 0x02) != 0;
		sy0 = c[2];
	} else {
		if (c[1] & 1) {
			clip[0].x0 = clip[0].x1 - c[0] - 8;
			clip[1].x1 = clip[1].x1 - c[0] - 8;
		} else {
			clip[0].x1 = clip[0].x1 - c[0] - 8;
			clip[1].x0 = clip[1].x1 - c[0] - 8;
		}
		if (c[3] & 0x40) {
			clip[2].x0 += 24;
			clip[2].x1 -= 16;
		} else {
			clip[2].x0 += 16;
			clip[2].x1 -= 8;
		}
		for (int i = 2; i >= 0; i--) {
			if (flip) {
				const int x1 = vx1 - clip[i].x0, x0 = vx1 - clip[i].x1;
				clip[i].x0 = x0;
				clip[i].x1 = x1;
			}
			const clip_t b = i == 2 ? (clip_t){ vx0, vx1 } : clip[2];
			if (clip[i].x0 < b.x0)
				clip[i].x0 = b.x0;
			if (clip[i].x1 > b.x1)
				clip[i].x1 = b.x1;
		}
		sx0 = sx1 = c[0] - 16;
		sy0 = sy1 = c[2];
		k->spr_dx = 16;
		k->spr_flip_dx = -8;
	}
	const int opaque = !(c[3] & 0x20);
	draw_layer(0, sx0, sy0, cols, 0, opaque, clip[0]);
	k007121_sprites_draw(k, s_gfx, s_gfx_count, 0, s_transmask, sprite_cb, rbx_video.bitmap, rbx_video.pitch, VIS_X0,
	                     VIS_Y0, clip[2].x0, VIS_Y0, clip[2].x1, VIS_Y0 + VIS_H - 1, s_pri, (uint32_t)((c[3] & 0x20) >> 4));
	draw_layer(0, sx0, sy0, cols, 1, opaque, clip[0]);
	draw_layer(1, sx1, sy1, 0, -1, 0, clip[1]);
}

// ---- the main CPU's bus (prg_map) ----
static void map_bank(void)
{
	for (int p = 0; p < 0x40; p++)
		s_cpu.rpage[0x40 + p] = s_rom + ((uint32_t)t.mainbank * 0x4000 + (uint32_t)p * 256) % s_rom_size;
}

static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x1000) {
		if (a >= 0x0020 && a < 0x0060)
			return s_k121.scroll[a - 0x0020];
		if (a >= 0x0d00 && a < 0x0d20)
			return k051733_read(&s_k733, a - 0x0d00);
		switch (a) {
		case 0x0800: return rbx_ym_read(0, 1);   // (data_r)
		case 0x0801: return rbx_ym_read(0, 0);   // (status_r)
		case 0x0900: return rbx_ym_read(1, 1);
		case 0x0901: return rbx_ym_read(1, 0);
		case 0x0a00: return (uint8_t)rbx_port_read(s_port_p[1]);
		case 0x0a01: return (uint8_t)rbx_port_read(s_port_p[0]);
		case 0x0b00: return (uint8_t)rbx_port_read(s_port_system);
		}
		return 0;
	}
	if (a < 0x1100)
		return t.pal[a - 0x1000];
	if (a < 0x1800)
		return 0;
	if (a < 0x2000)
		return t.ram[a - 0x1800];
	if (a < 0x3000)
		return t.spr[a - 0x2000];
	if (a < 0x4000)
		return t.vram[(a >> 11) & 1][a & 0x7ff];
	if (a < 0x8000)
		return s_rom[((uint32_t)t.mainbank * 0x4000 + (a - 0x4000)) % s_rom_size];
	return s_rom[0x18000 + (a - 0x8000)];
}

static void cpu_lines(int set, int cleared)   // (the K007121's IRQ and NMI)
{
	if (cleared & K007121_IRQ)
		hd6309_set_irq(&s_cpu, 0);
	if (set & K007121_IRQ)
		hd6309_set_irq(&s_cpu, 1);
	if (cleared & K007121_NMI)
		hd6309_set_nmi(&s_cpu, 0);
	if (set & K007121_NMI)
		hd6309_set_nmi(&s_cpu, 1);
}

static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x1000) {
		if (a < 0x0008)
			cpu_lines(0, k007121_ctrl_w(&s_k121, a, v));
		else if (a >= 0x0020 && a < 0x0060)
			s_k121.scroll[a - 0x0020] = v;
		else if (a >= 0x0d00 && a < 0x0d20)
			k051733_write(&s_k733, a - 0x0d00, v);
		else
			switch (a) {
			case 0x0800: rbx_ym_write(0, 1, v); break;   // (data_w)
			case 0x0801: rbx_ym_write(0, 0, v); break;   // (address_w)
			case 0x0900: rbx_ym_write(1, 1, v); break;
			case 0x0901: rbx_ym_write(1, 0, v); break;
			case 0x0c00:   // (bankswitch_w)
				t.mainbank = v & 7;
				map_bank();
				break;
			}
		return;
	}
	if (a < 0x1100)
		t.pal[a - 0x1000] = v;
	else if (a >= 0x1800 && a < 0x2000)
		t.ram[a - 0x1800] = v;
	else if (a >= 0x2000 && a < 0x3000)
		t.spr[a - 0x2000] = v;
	else if (a >= 0x3000 && a < 0x4000)
		t.vram[(a >> 11) & 1][a & 0x7ff] = v;
}

// ---- the sound: the two YM2203s, a line's samples (SSG 0.40, FM 0.80 each, their gains) ----
static uint8_t ym_dsw1(void) { return (uint8_t)rbx_port_read(s_port_dsw[0]); }
static uint8_t ym_dsw2(void) { return (uint8_t)rbx_port_read(s_port_dsw[1]); }
static uint8_t ym_dsw3(void) { return (uint8_t)rbx_port_read(s_port_dsw[2]); }

static void sound_line(int n)
{
	if (n > 64)
		n = 64;
	int32_t y0[96], y1[96];
	memset(y0, 0, sizeof(y0));
	memset(y1, 0, sizeof(y1));
	t.ym_frac += (uint64_t)YM_CLOCK * (uint64_t)n;
	const uint32_t yc = (uint32_t)(t.ym_frac / SND_RATE);
	t.ym_frac -= (uint64_t)yc * SND_RATE;
	const int n0 = rbx_ym_advance(0, yc, y0, 96), n1 = rbx_ym_advance(1, yc, y1, 96);
	for (int i = 0; i < n && s_out_pos < 2048; i++, s_out_pos++)
		s_out[s_out_pos] = (n0 > 0 ? y0[i * n0 / n] : 0) + (n1 > 0 ? y1[i * n1 / n] : 0);
}

// ---- init / frame ----
static void lr_exit(void);
static void lr_reset(void);

static int lr_init(const rbx_game *g)
{
	uint32_t gsize, psize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	const uint8_t *gfx = rbx_region("gfx", &gsize), *prom = rbx_region("proms", &psize);
	if (!s_rom || !gfx || !prom || psize < 0x100) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	s_gfx_count = gsize / 32;   // (gfx_8x8x4_packed_msb)
	s_gfx = (uint8_t *)malloc((size_t)s_gfx_count * 64);
	if (!s_gfx) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	for (uint32_t n = 0; n < s_gfx_count; n++)
		for (int b = 0; b < 32; b++) {
			s_gfx[n * 64 + b * 2] = gfx[n * 32 + b] >> 4;
			s_gfx[n * 64 + b * 2 + 1] = gfx[n * 32 + b] & 15;
		}
	for (int pal = 0; pal < 8; pal++)   // (palette: odd banks direct, even ones through the PROM)
		for (int i = 0; i < 0x100; i++)
			s_ctab[(pal << 8) | i] = (uint16_t)((pal & 1) ? (pal << 4) | (i & 0x0f) : !prom[i] ? 0 : (pal << 4) | (prom[i] & 0x0f));
	for (int c = 0; c < 128; c++) {   // (transpen_mask(gfx, color, 0))
		uint16_t m = 0;
		for (int p = 0; p < 16; p++)
			if (s_ctab[(c * 16 + p) % NPENS] == 0)
				m |= (uint16_t)(1u << p);
		s_transmask[c] = m;
	}
	k007121_init(&s_k121, 0, 0);
	for (int i = 0; i < 2; i++) {
		rbx_ym_config yc;
		memset(&yc, 0, sizeof(yc));
		yc.type = RBX_YM2203;
		yc.clock = YM_CLOCK;
		yc.gain[0] = yc.gain[1] = yc.gain[2] = 102;   // (SSG 0.40)
		yc.gain[3] = 205;                             // (FM 0.80)
		yc.port_a_read = i ? NULL : ym_dsw1;
		yc.port_b_read = i ? ym_dsw3 : ym_dsw2;
		if (rbx_ym_init(i, &yc)) {
			rbx_fail("%s: YM2203", g->set);
			lr_exit();
			return -1;
		}
	}
	s_port_system = rbx_port_find("SYSTEM");
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	static const char *const dn[3] = { "DSW1", "DSW2", "DSW3" };
	for (int i = 0; i < 3; i++)
		s_port_dsw[i] = rbx_port_find(dn[i]);
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		lr_exit();
		return -1;
	}
	lr_reset();
	return 0;
}

static void lr_exit(void)
{
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_gfx);
	s_gfx = NULL;
}

static void lr_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	s_out_pos = 0;
	k007121_reset(&s_k121);
	memset(s_k121.scroll, 0, sizeof(s_k121.scroll));
	k051733_reset(&s_k733);
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	hd6309_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	for (int p = 0x80; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + 0x18000 + (uint32_t)(p - 0x80) * 256;
	for (int p = 0x18; p < 0x20; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + (p - 0x18) * 256;
	for (int p = 0x20; p < 0x30; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.spr + (p - 0x20) * 256;
	map_bank();
	hd6309_reset(&s_cpu);
}

static void lr_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the screen's update)
			capture();
			rbx_video_frame_done();
		}
		if (!(vpos & 15))   // (the K007121's scanline timer: every 16 lines)
			cpu_lines(k007121_scanline(&s_k121, vpos, t.frame, t.spr), 0);
		t.cyc_frac += (uint64_t)CPU_CLOCK * HTOTAL;
		const int64_t cyc = (int64_t)(t.cyc_frac / PIX_CLOCK);
		t.cyc_frac -= (uint64_t)cyc * PIX_CLOCK;
		hd6309_run_until(&s_cpu, t.cpu_end += cyc);
		t.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(t.snd_frac / PIX_CLOCK);
		t.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		if (ns > 0)
			sound_line(ns);
	}
	for (int i = 0; i < s_out_pos; i++)
		rbx_audio_put_stereo(s_out[i], s_out[i]);
	s_out_pos = 0;
	t.frame++;
}

static int lr_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "lr_cpu", &s_cpu, (uint32_t)offsetof(hd6309_t, rpage));
	rc |= rbx_state_io(ctx, "lr", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "lr_k121", &s_k121, (uint32_t)offsetof(k007121_t, state_end));
	rc |= rbx_state_io(ctx, "lr_k733", &s_k733, sizeof(s_k733));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	if (!rc && load)
		map_bank();
	return rc ? -1 : 0;
}

static int lr_save(void *ctx) { return lr_state(ctx, 0); }
static int lr_load(void *ctx) { return lr_state(ctx, 1); }
static int lr_supported(const char *machine) { return !strcmp(machine, "labyrunr"); }

const rbx_driver rbx_drv_labyrunr = { "Konami Labyrinth Runner", lr_init, lr_exit, lr_reset, lr_frame, lr_save, lr_load,
                                      lr_supported, lr_render, NULL };
