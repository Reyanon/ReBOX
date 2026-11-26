/*  Technos Xain'd Sleena / Solar-Warrior. Ported from MAME src/mame/technos/xain.cpp, xain_v.cpp, xain.h
    (BSD-3-Clause, copyright-holders: Carlos A. Lozano, Rob Rosenbrock, Phil Stroffolino, Bryan McPhail).
    Two MC6809E at 1.5 MHz sharing 8 KB of RAM (MAME: perfect quantum on the main CPU; here 8 slices a line), the main
    CPU's NMI at vcount F8 and FIRQ every 16 lines, IRQs between the two; a Taito 68705 MCU at 3 MHz (not on the
    bootlegs); MC6809 sound CPU at 1.5 MHz with two YM2203 (the first on the FIRQ). Video: two 512x512 tilemaps of
    16x16, a 256x256 text layer, 16x16 sprites (two high), eight priority orders; drawn line by line (MAME's
    update_partial on every scanline): scroll and order latched per line, the video RAM by pieces.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../machine/taito68705.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 57445          // set_raw(12 MHz / 2, 384, 0, 256, 272, 8, 248)
#define LINES 272
#define SLICES 8
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 256
#define CPU_CLOCK 1500000
#define MCU_CLOCK 750000         // (3 MHz / 4)
#define YM_CLOCK 3000000

typedef struct {
	uint16_t sx[2], sy[2];
	uint8_t pri, flip;
} line_regs;

#define MAXSEG 32
typedef struct {
	int y0;
	uint8_t chr[0x800], bg[2][0x800], spr[0x180];
} vseg;
typedef struct {
	int n;
	vseg seg[MAXSEG];
} vbuf;

static int s_has_mcu;
static uint8_t *s_rom, *s_srom, *s_zrom;
static uint32_t s_rom_size, s_srom_size, s_zrom_size;
static rbx_gfx s_gchar, s_gbg[2], s_gspr;   // s_gbg[N]: bg tilemap N's gfx (N 0: gfx3, base 384; N 1: gfx2, base 256)
static int s_port[5];
static m6809_t s_main, s_sub, s_snd;
static taito68705_t s_mcu;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static int s_line;
static vbuf *s_vb[2];
static int s_vb_cur, s_vdirty;

static struct {
	uint8_t ram[0x2000], chr[0x800], bg[2][0x800], spr[0x180], pal[0x400];
	uint8_t sndram[0x800];
	uint8_t scrollx[2][2], scrolly[2][2];
	uint8_t pri, flip, bank[2], vblank;
	uint8_t latch;
	int32_t ym_irq;
	int64_t main_t, sub_t, snd_t;
	uint64_t main_frac, sub_frac, snd_frac, mcu_frac, ym_frac;
	line_regs lines[LINES];
	uint8_t state_end;
} *t;   // (calloc in init: ADR-040)

static struct {
	uint8_t pal[0x400];
	line_regs lines[VIS_H];
	const vbuf *vb;
} *s_snap;

// ---- sound ----
static void ym_irq(int state)
{
	t->ym_irq = state;
	m6809_set_firq(&s_snd, state);
}

static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x800) return t->sndram[a];
	if (a == 0x1000) {
		m6809_set_irq(&s_snd, 0);
		return t->latch;
	}
	if (a >= 0x4000) return s_zrom[rbx_wrap(a, s_zrom_size)];
	return 0;
}
static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x800) t->sndram[a] = v;
	else if (a == 0x2800 || a == 0x2801) rbx_ym_write(0, a & 1, v);
	else if (a == 0x3000 || a == 0x3001) rbx_ym_write(1, a & 1, v);
}

static void sound_line(void)
{
	static int32_t tmp[1024];
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&t->ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
	const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	rbx_ym_advance(1, clocks, tmp, 1024);
	s_mix_n += n;
}

// ---- main / sub CPUs ----
static void map_banks(void)
{
	for (int p = 0; p < 0x40; p++) {
		s_main.rpage[0x40 + p] = s_rom + 0x4000 + (uint32_t)t->bank[0] * 0xc000 + p * 256;
		s_sub.rpage[0x40 + p] = s_srom + 0x4000 + (uint32_t)t->bank[1] * 0xc000 + p * 256;
	}
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0x8000) return s_rom[a];
	if (a >= 0x4000) return s_rom[0x4000 + (uint32_t)t->bank[0] * 0xc000 + (a - 0x4000)];
	if (a < 0x2000) return t->ram[a];
	if (a < 0x2800) return t->chr[a - 0x2000];
	if (a < 0x3000) return t->bg[1][a - 0x2800];
	if (a < 0x3800) return t->bg[0][a - 0x3000];
	if (a < 0x3980) return t->spr[a - 0x3800];
	switch (a) {
	case 0x3a00: return (uint8_t)rbx_port_read(s_port[0]);
	case 0x3a01: return (uint8_t)rbx_port_read(s_port[1]);
	case 0x3a02: return (uint8_t)rbx_port_read(s_port[2]);
	case 0x3a03: return (uint8_t)rbx_port_read(s_port[3]);
	case 0x3a04: return s_has_mcu ? taito68705_data_r(&s_mcu) : 0;
	case 0x3a05: {
		uint32_t st = 3;
		if (s_has_mcu)
			st = (taito68705_mcu_semaphore_r(&s_mcu) ? 0 : 1) | (taito68705_host_semaphore_r(&s_mcu) ? 0 : 2);
		rbx_port_custom(s_port[4], 0x38, st << 3 | (t->vblank ? 0x20u : 0));
		return (uint8_t)rbx_port_read(s_port[4]);
	}
	case 0x3a06:   // mcu_comm_reset_r
		if (!s_has_mcu)
			return 0;
		taito68705_reset_w(&s_mcu, 1);
		taito68705_reset_w(&s_mcu, 0);
		return 0xff;
	}
	return 0;
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000) return;
	if (a < 0x2000) t->ram[a] = v;
	else if (a < 0x2800) { t->chr[a - 0x2000] = v; s_vdirty = 1; }
	else if (a < 0x3000) { t->bg[1][a - 0x2800] = v; s_vdirty = 1; }
	else if (a < 0x3800) { t->bg[0][a - 0x3000] = v; s_vdirty = 1; }
	else if (a < 0x3980) { t->spr[a - 0x3800] = v; s_vdirty = 1; }
	else if (a >= 0x3c00) t->pal[a - 0x3c00] = v;
	else switch (a) {
	case 0x3a00: case 0x3a01: t->scrollx[1][a & 1] = v; break;
	case 0x3a02: case 0x3a03: t->scrolly[1][a & 1] = v; break;
	case 0x3a04: case 0x3a05: t->scrollx[0][a & 1] = v; break;
	case 0x3a06: case 0x3a07: t->scrolly[0][a & 1] = v; break;
	case 0x3a08: t->latch = v; m6809_set_irq(&s_snd, 1); break;
	case 0x3a09: m6809_set_nmi(&s_main, 0); break;
	case 0x3a0a: m6809_set_firq(&s_main, 0); break;
	case 0x3a0b: m6809_set_irq(&s_main, 0); break;
	case 0x3a0c: m6809_set_irq(&s_sub, 1); break;
	case 0x3a0d: t->flip = v & 1; break;
	case 0x3a0e: if (s_has_mcu) taito68705_data_w(&s_mcu, v); break;
	case 0x3a0f:
		t->pri = v & 7;
		t->bank[0] = (v >> 3) & 1;
		map_banks();
		break;
	}
}

static uint8_t sub_read(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0x8000) return s_srom[a];
	if (a >= 0x4000) return s_srom[0x4000 + (uint32_t)t->bank[1] * 0xc000 + (a - 0x4000)];
	if (a < 0x2000) return t->ram[a];
	return 0;
}
static void sub_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x2000) t->ram[a] = v;
	else if (a == 0x2000) m6809_set_irq(&s_main, 1);
	else if (a == 0x2800) m6809_set_irq(&s_sub, 0);
	else if (a == 0x3000) {
		t->bank[1] = v & 1;
		map_banks();
	}
}

// ---- video ----
static int s_clip0, s_clip1;

static void draw_bg(const vseg *sg, int n, const line_regs *r, int y, int opaque, uint16_t *d)
{
	const uint8_t *vram = sg->bg[n];
	const rbx_gfx *g = &s_gbg[n];
	const uint16_t base = n ? 256 : 384;
	const uint32_t ty = (uint32_t)(r->flip ? 255 - y + r->sy[n] : y + r->sy[n]) & 511;
	const uint32_t row = ty >> 4, fy = ty & 15;
	for (int x = 0; x < VIS_W; x++) {
		const uint32_t tx = (uint32_t)(r->flip ? 255 - x + r->sx[n] : x + r->sx[n]) & 511;
		const uint32_t col = tx >> 4;
		const uint32_t idx = (col & 0x0f) + ((row & 0x0f) << 4) + ((col & 0x10) << 4) + ((row & 0x10) << 5);
		const uint8_t attr = vram[idx | 0x400];
		const uint32_t code = rbx_wrap((uint32_t)(vram[idx] | (attr & 7) << 8), g->count);
		const uint32_t px = attr & 0x80 ? 15 - (tx & 15) : tx & 15;
		const uint8_t p = g->pix[code * 256 + fy * 16 + px];
		if (p || opaque)
			d[x] = (uint16_t)(base + ((attr & 0x70) >> 4) * 16 + p);
	}
}

static void draw_char(const vseg *sg, const line_regs *r, int y, int opaque, uint16_t *d)
{
	const uint32_t ty = (uint32_t)(r->flip ? 255 - y : y) & 255;
	for (int x = 0; x < VIS_W; x++) {
		const uint32_t tx = (uint32_t)(r->flip ? 255 - x : x) & 255;
		const uint32_t idx = (ty >> 3) * 32 + (tx >> 3);
		const uint8_t attr = sg->chr[idx | 0x400];
		const uint32_t code = rbx_wrap((uint32_t)(sg->chr[idx] | (attr & 3) << 8), s_gchar.count);
		const uint8_t p = s_gchar.pix[code * 64 + (ty & 7) * 8 + (tx & 7)];
		if (p || opaque)
			d[x] = (uint16_t)(((attr & 0xe0) >> 5) * 16 + p);
	}
}

static void draw_spr16(uint32_t code, int color, int fx, int fy, int sx, int sy)
{
	code %= s_gspr.count;
	if (!s_gspr.used[code] || sy >= s_clip1 || sy + 16 <= s_clip0)
		return;
	const uint8_t *src = s_gspr.pix + code * 256;
	for (int j = 0; j < 16; j++) {
		const int yy = sy + j;
		if (yy < s_clip0 || yy >= s_clip1)
			continue;
		const uint8_t *s = src + (fy ? 15 - j : j) * 16;
		uint16_t *d = rbx_video.bitmap + (yy - VIS_Y0) * rbx_video.pitch;
		for (int i = 0; i < 16; i++) {
			const int xx = sx + i;
			if ((unsigned)xx >= VIS_W)
				continue;
			const uint8_t p = s[fx ? 15 - i : i];
			if (p)
				d[xx] = (uint16_t)(128 + color * 16 + p);
		}
	}
}

static void draw_sprites(const vseg *sg, int flip)
{
	for (int o = 0; o < 0x180; o += 4) {
		const uint8_t *s = &sg->spr[o];
		const int attr = s[1];
		const uint32_t num = (uint32_t)(s[2] | (attr & 7) << 8);
		const int color = (attr & 0x38) >> 3;
		int sx = 238 - s[3];
		if (sx <= -7) sx += 256;
		int sy = 240 - s[0];
		if (sy <= -7) sy += 256;
		int fx = (attr & 0x40) != 0, fy = 0;
		if (flip) {
			sx = 238 - sx;
			sy = 240 - sy;
			fx = !fx;
			fy = !fy;
		}
		if (attr & 0x80) {
			draw_spr16(num, color, fx, fy, sx, fy ? sy + 16 : sy - 16);
			draw_spr16(num + 1, color, fx, fy, sx, sy);
		} else
			draw_spr16(num, color, fx, fy, sx, sy);
	}
}

// layer codes: 0 bg0, 1 bg1, 2 char, 3 sprites
static const uint8_t s_order[8][4] = {
	{ 0, 1, 3, 2 }, { 1, 0, 3, 2 }, { 2, 0, 3, 1 }, { 2, 1, 3, 0 },
	{ 0, 2, 3, 1 }, { 1, 2, 3, 0 }, { 0, 3, 1, 2 }, { 1, 3, 0, 2 },
};

static void draw_piece(const vseg *sg, int y0, int y1)
{
	// (pieces of lines with the same priority order; lines[] from VIS_Y0)
	const line_regs *L = s_snap->lines - VIS_Y0;
	int y = y0;
	while (y < y1) {
		const int pri = L[y].pri;
		int ye = y + 1;
		while (ye < y1 && L[ye].pri == pri)
			ye++;
		for (int k = 0; k < 4; k++) {
			const int l = s_order[pri][k];
			if (l == 3) {
				s_clip0 = y;
				s_clip1 = ye;
				draw_sprites(sg, L[y].flip);
				continue;
			}
			for (int yy = y; yy < ye; yy++) {
				uint16_t *d = rbx_video.bitmap + (yy - VIS_Y0) * rbx_video.pitch;
				if (l == 2)
					draw_char(sg, &L[yy], yy, k == 0, d);
				else
					draw_bg(sg, l, &L[yy], yy, k == 0, d);
			}
		}
		y = ye;
	}
}

static void xn_render(void)
{
	for (int i = 0; i < 512; i++) {   // xBGR_444
		const uint32_t p = (uint32_t)(s_snap->pal[0x200 + i] << 8 | s_snap->pal[i]);
		rbx_video.palette[i] = ((p & 15) * 17) << 16 | (((p >> 4) & 15) * 17) << 8 | ((p >> 8) & 15) * 17;
	}
	const vbuf *vb = s_snap->vb;
	for (int k = 0; k < vb->n; k++) {
		const int y0 = vb->seg[k].y0, y1 = k + 1 < vb->n ? vb->seg[k + 1].y0 : VIS_Y0 + VIS_H;
		draw_piece(&vb->seg[k], y0, y1);
	}
}

static void vram_line(int line)   // (after a visible line: a new piece when the RAM changed)
{
	vbuf *b = s_vb[s_vb_cur];
	if (line == VIS_Y0)
		b->n = 0;
	else if (!s_vdirty)
		return;
	vseg *sg;
	if (b->n < MAXSEG) {
		sg = &b->seg[b->n++];
		sg->y0 = line;
	} else
		sg = &b->seg[MAXSEG - 1];
	memcpy(sg->chr, t->chr, sizeof(t->chr));
	memcpy(sg->bg, t->bg, sizeof(t->bg));
	memcpy(sg->spr, t->spr, sizeof(t->spr));
	s_vdirty = 0;
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->pal, t->pal, sizeof(t->pal));
	memcpy(s_snap->lines, t->lines, sizeof(line_regs) * VIS_H);
	s_snap->vb = s_vb[s_vb_cur];
	s_vb_cur ^= 1;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void xn_reset(void)
{
	memset(t, 0, offsetof(__typeof__(*t), state_end));
	m6809_init(&s_main);
	s_main.read = main_read;
	s_main.write = main_write;
	m6809_init(&s_sub);
	s_sub.read = sub_read;
	s_sub.write = sub_write;
	for (int p = 0; p < 0x20; p++) {
		s_main.rpage[p] = s_main.wpage[p] = t->ram + p * 256;
		s_sub.rpage[p] = s_sub.wpage[p] = t->ram + p * 256;
	}
	for (int p = 0x80; p < 0x100; p++) {
		s_main.rpage[p] = s_rom + p * 256;
		s_sub.rpage[p] = s_srom + p * 256;
	}
	map_banks();
	m6809_reset(&s_main);
	m6809_reset(&s_sub);
	m6809_init(&s_snd);
	s_snd.read = snd_read;
	s_snd.write = snd_write;
	for (int p = 0; p < 8; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = t->sndram + p * 256;
	for (int p = 0x40; p < 0x100 && (uint32_t)p * 256 + 256 <= s_zrom_size; p++)
		s_snd.rpage[p] = s_zrom + p * 256;
	m6809_reset(&s_snd);
	if (s_has_mcu)
		taito68705_reset(&s_mcu);
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	s_mix_n = 0;
	s_line = 0;
	s_vdirty = 0;
}

static void xn_exit(void);

static int xn_init(const rbx_game *g)
{
	free(t);
	t = calloc(1, sizeof(*t));
	if (!t) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_has_mcu = !strcmp(g->machine, "xsleena");
	uint32_t csize, b1size, b0size, ssize, msize = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("sub", &s_srom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *chars = rbx_region("gfx1", &csize), *bg1 = rbx_region("gfx2", &b1size), *bg0 = rbx_region("gfx3", &b0size);
	uint8_t *spr = rbx_region("gfx4", &ssize);
	uint8_t *mrom = s_has_mcu ? rbx_region("mcu:mcu", &msize) : NULL;
	if (!s_rom || s_rom_size < 0x14000 || !s_srom || s_srom_size < 0x14000 || !s_zrom || s_zrom_size < 0x10000 || !chars ||
	    !bg1 || !bg0 || !spr || (s_has_mcu && (!mrom || msize < 0x800))) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (s_has_mcu) {
		memset(&s_mcu, 0, sizeof(s_mcu));
		if (taito68705_init(&s_mcu, TAITO68705_STD, mrom, 0x800)) {
			rbx_fail("%s: MCU", g->set);
			return -1;
		}
	}
	{	// charlayout, tilelayout (RGN_FRAC(1,2))
		static const uint32_t pc[4] = { 0, 2, 4, 6 };
		static const uint32_t xc[8] = { 1, 0, 8 * 8 + 1, 8 * 8 + 0, 16 * 8 + 1, 16 * 8 + 0, 24 * 8 + 1, 24 * 8 + 0 };
		static const uint32_t yc[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
		static const uint32_t xt[16] = { 3, 2, 1, 0, 16 * 8 + 3, 16 * 8 + 2, 16 * 8 + 1, 16 * 8 + 0,
		                                 32 * 8 + 3, 32 * 8 + 2, 32 * 8 + 1, 32 * 8 + 0, 48 * 8 + 3, 48 * 8 + 2, 48 * 8 + 1, 48 * 8 + 0 };
		static uint32_t yt[16];
		for (int i = 0; i < 16; i++)
			yt[i] = (uint32_t)i * 8;
		const uint32_t p1[4] = { b1size * 4, b1size * 4 + 4, 0, 4 }, p0[4] = { b0size * 4, b0size * 4 + 4, 0, 4 };
		const uint32_t ps[4] = { ssize * 4, ssize * 4 + 4, 0, 4 };
		const rbx_gfx_layout lc = { 8, 8, 4, pc, xc, yc, 32 * 8 };
		const rbx_gfx_layout l1 = { 16, 16, 4, p1, xt, yt, 64 * 8 }, l0 = { 16, 16, 4, p0, xt, yt, 64 * 8 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, xt, yt, 64 * 8 };
		if (rbx_gfx_decode(&s_gchar, &lc, chars, csize) || rbx_gfx_decode(&s_gbg[1], &l1, bg1, b1size / 2) ||
		    rbx_gfx_decode(&s_gbg[0], &l0, bg0, b0size / 2) || rbx_gfx_decode(&s_gspr, &ls, spr, ssize / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			xn_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	s_vb[0] = calloc(1, sizeof(vbuf));
	s_vb[1] = calloc(1, sizeof(vbuf));
	s_vb_cur = 0;
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = 128;   // (SSG 0.50, FM 0.40)
	yc.gain[3] = 102;
	yc.irq = ym_irq;
	int rc = !s_snap || !s_vb[0] || !s_vb[1] || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	yc.irq = NULL;
	rc = rc || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		xn_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 512)) {
		rbx_fail("%s: out of memory", g->set);
		xn_exit();
		return -1;
	}
	static const char *const names[5] = { "P1", "P2", "DSW0", "DSW1", "VBLANK" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	xn_reset();
	return 0;
}

static void xn_exit(void)
{
	rbx_video_wait_idle();
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gchar);
	rbx_gfx_free(&s_gbg[0]);
	rbx_gfx_free(&s_gbg[1]);
	rbx_gfx_free(&s_gspr);
	free(s_snap);
	s_snap = NULL;
	free(s_vb[0]);
	free(s_vb[1]);
	s_vb[0] = s_vb[1] = NULL;
	free(t);
	t = NULL;
}

static inline int scanline_to_vcount(int scanline)
{
	const int vcount = scanline + 8;
	return vcount < 0x100 ? vcount : ((vcount - 0x18) | 0x100);
}

static void xn_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		const int vold = scanline_to_vcount(line == 0 ? LINES - 1 : line - 1), vc = scanline_to_vcount(line);
		if (!(vold & 8) && (vc & 8))
			m6809_set_firq(&s_main, 1);
		if (vc == 0xf8)
			m6809_set_nmi(&s_main, 1);
		t->vblank = vc >= 248 - 1;
		for (int s = 0; s < SLICES; s++) {
			m6809_run_until(&s_main, t->main_t += rbx_line_cycles(&t->main_frac, CPU_CLOCK, FRAME_MHZ, LINES * SLICES));
			m6809_run_until(&s_sub, t->sub_t += rbx_line_cycles(&t->sub_frac, CPU_CLOCK, FRAME_MHZ, LINES * SLICES));
		}
		if (s_has_mcu)
			taito68705_execute(&s_mcu, rbx_line_cycles(&t->mcu_frac, MCU_CLOCK, FRAME_MHZ, LINES));
		m6809_run_until(&s_snd, t->snd_t += rbx_line_cycles(&t->snd_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		sound_line();
		if (line >= VIS_Y0 && line < VIS_Y0 + VIS_H) {   // (update_partial at the next line's start)
			line_regs *r = &t->lines[line - VIS_Y0];
			for (int n = 0; n < 2; n++) {
				r->sx[n] = (uint16_t)(t->scrollx[n][0] | t->scrollx[n][1] << 8);
				r->sy[n] = (uint16_t)(t->scrolly[n][0] | t->scrolly[n][1] << 8);
			}
			r->pri = t->pri;
			r->flip = t->flip;
			vram_line(line);
		}
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	rbx_ym_async_take(1, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int xn_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "xn", t, (uint32_t)offsetof(__typeof__(*t), state_end));
	rc |= rbx_state_io(ctx, "xn_main", &s_main, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "xn_sub", &s_sub, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "xn_snd", &s_snd, (uint32_t)offsetof(m6809_t, rpage));
	if (s_has_mcu) {
		rc |= rbx_state_io(ctx, "xn_tmcu", &s_mcu.st, sizeof(s_mcu.st));
		rc |= rbx_state_io(ctx, "xn_mcu", &s_mcu.mcu.st, sizeof(s_mcu.mcu.st));
	}
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	if (!rc && load) {
		map_banks();
		s_vdirty = 1;
	}
	return rc ? -1 : 0;
}

static int xn_save(void *ctx) { return xn_state(ctx, 0); }
static int xn_load(void *ctx) { return xn_state(ctx, 1); }
static int xn_supported(const char *machine) { return !strcmp(machine, "xsleena") || !strcmp(machine, "xsleenab"); }

const rbx_driver rbx_drv_xain = { "Technos Xain'd Sleena", xn_init, xn_exit, xn_reset, xn_frame, xn_save, xn_load,
                                  xn_supported, xn_render, NULL };
