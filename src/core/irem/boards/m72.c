/*  Irem M72, M81, M82 and M84. Ported from MAME src/mame/irem/m72.cpp, m72.h, m72_v.cpp (BSD-3-Clause, copyright-holders:
    Nicola Salmoria), the dual port RAM of src/devices/machine/mb8421.h (BSD-3-Clause, copyright-holders: hap, AJR); the
    scroll arithmetic of src/emu/tilemap.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria, Aaron Giles).
    One driver, a table row per MAME machine: a V30 at 8 MHz (a V35 on Ken-Go / Cosmic Cop), a uPD71059C, two 64x64 tile
    layers, sprites buffered by a write, an 8751 on the protected boards on the main thread, the Z80 sound board of
    irem_zsnd.c. The raster interrupt's update_partial as a list of scroll registers per segment (m92.c).
    Not done: flip screen, Pound for Pound (trackballs), imgfightjb (i80c31 bootleg), the lohtb sets (MAME: not working).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/mcs51/mcs51.h"
#include "../../cpu/nec/nec.h"
#include "../../cpu/nec/irem_cpu.h"
#include "../machine/irem_zsnd.h"
#include "../machine/pic8259.h"
#include "../video/irem_gfx.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define LINES 284
#define HTOTAL 512
#define VIS_X0 64
#define VIS_W 384
#define VIS_H 256
#define VBLANK_LINE 256
#define FRAME_MHZ 55018u       // (8 MHz / 512 / 284)
#define NPENS 513              // (the last: black, the display off)
#define MAX_SEG 256
#define MCU_SLICE 10

enum { K_M72, K_M81, K_M84, K_KENGO, K_M82 };
enum { P_NONE, P_DBREED, P_DKGEN };
typedef struct {
	const char *machine;
	int kind;
	uint32_t rom_end, ram;      // ROM up to; work RAM at
	int mcu, ports, nmi, prot, airduel;
	int dx[2];                  // fg, bg
	int rtype2_tiles;
	uint32_t pal[2], vram[2];
} m72_cfg;

#define PALS_M72 { 0xc8000, 0xcc000 }
#define VRAM_M72 { 0xd0000, 0xd8000 }
static const m72_cfg s_cfgs[] = {
	{ "rtype", K_M72, 0x40000, 0x40000, 0, ZP_RTYPE, ZN_NONE, P_NONE, 0, { 0, 0 }, 0, PALS_M72, VRAM_M72 },
	{ "m72_8751", K_M72, 0x80000, 0xa0000, 1, ZP_PROT, ZN_NONE, P_NONE, 0, { 0, 0 }, 0, PALS_M72, VRAM_M72 },
	{ "m72_airduel", K_M72, 0x80000, 0xa0000, 1, ZP_RTYPE, ZN_NONE, P_NONE, 1, { 0, 0 }, 0, PALS_M72, VRAM_M72 },
	{ "m72_xmultipl", K_M72, 0x80000, 0x80000, 1, ZP_PROT, ZN_NONE, P_NONE, 0, { 0, 0 }, 0, PALS_M72, VRAM_M72 },
	{ "m72_dbreed", K_M72, 0x80000, 0x90000, 1, ZP_PROT, ZN_NONE, P_NONE, 0, { 0, 0 }, 0, PALS_M72, VRAM_M72 },
	{ "m72_dbreedw", K_M72, 0x80000, 0x90000, 0, ZP_M72, ZN_PULSE, P_DBREED, 0, { 0, 0 }, 0, PALS_M72, VRAM_M72 },
	{ "m72", K_M72, 0x80000, 0xa0000, 0, ZP_M72, ZN_FAKE, P_DKGEN, 0, { 0, 0 }, 0, PALS_M72, VRAM_M72 },
	{ "m81_hharry", K_M81, 0x80000, 0xa0000, 0, ZP_RTYPE2, ZN_PULSE, P_NONE, 0, { 4, 6 }, 0, PALS_M72, VRAM_M72 },
	{ "m81_xmultipl", K_M81, 0x80000, 0x9c000, 0, ZP_RTYPE2, ZN_PULSE, P_NONE, 0, { 4, 6 }, 0, PALS_M72, VRAM_M72 },
	{ "m81_dbreed", K_M81, 0x80000, 0x88000, 0, ZP_RTYPE2, ZN_PULSE, P_NONE, 0, { 4, 6 }, 0, PALS_M72, VRAM_M72 },
	{ "rtype2", K_M84, 0x80000, 0xe0000, 0, ZP_RTYPE2, ZN_PULSE, P_NONE, 0, { 4, 4 }, 1, { 0xc8000, 0xd8000 }, { 0xd0000, 0xd4000 } },
	{ "hharryu", K_M84, 0x80000, 0xe0000, 0, ZP_RTYPE2, ZN_PULSE, P_NONE, 0, { 4, 6 }, 1, { 0xa0000, 0xa8000 }, { 0xd0000, 0xd4000 } },
	{ "cosmccop", K_KENGO, 0x80000, 0xe0000, 0, ZP_RTYPE2, ZN_PULSE, P_NONE, 0, { 4, 6 }, 1, { 0xa0000, 0xa8000 }, { 0x80000, 0x84000 } },
	{ "kengo", K_KENGO, 0x80000, 0xe0000, 0, ZP_RTYPE2, ZN_PULSE, P_NONE, 0, { 4, 6 }, 1, { 0xa0000, 0xa8000 }, { 0x80000, 0x84000 } },
	{ "m82", K_M82, 0x80000, 0xd0000, 0, ZP_RTYPE2, ZN_PULSE, P_NONE, 0, { 4, 6 }, 1, { 0xcc000, 0xa4000 }, { 0xac000, 0xb0000 } },
};

static const m72_cfg *s_k;
static nec_t s_cpu;
static pic8259_t s_pic;
static mcs51_t s_mcu;
static uint8_t *s_rom, *s_samples;
static uint32_t s_rom_size, s_samples_size;
static const uint8_t *s_mcu_rom;
static uint32_t s_mcu_size;
static rbx_gfx s_spr, s_spr2, s_t0, s_t1;
static const rbx_gfx *s_bg_gfx;
static uint8_t *s_t0_rows, *s_t1_rows, *s_pri;
static const uint8_t *s_bg_rows;
static int s_port_in0, s_port_in1, s_port_dsw, s_port_j3;
static const uint8_t *s_prot_code, *s_prot_crc;

#define CODE_LEN 96
#define CRC_LEN 18
static const uint8_t dbreedm72_code[CODE_LEN] = { 0xea, 0x6c, 0x00, 0x00, 0x00 };
static const uint8_t dbreedm72_crc[CRC_LEN] = { 0xa4, 0x96, 0x5f, 0xc0, 0xab, 0x49, 0x9f, 0x19, 0x84, 0xe6, 0xd6, 0xca, 0x00, 0x00 };
static const uint8_t dkgenm72_code[CODE_LEN] = { 0xea, 0x3d, 0x00, 0x00, 0x10 };
static const uint8_t dkgenm72_crc[CRC_LEN] = { 0xc8, 0xb4, 0xdc, 0xf8, 0xd3, 0xba, 0x48, 0xed, 0x79, 0x08, 0x1c, 0xb3, 0x00, 0x00 };

static struct {
	uint16_t ram[0x2000];
	uint16_t vram[2][0x8000];
	uint16_t spriteram[0x800], spritebuf[0x200], spriteram2[0x800], rowscroll[0x800];
	uint16_t palette[2][0x600];
	uint8_t pen_set[2][256];   // (a pen never written keeps MAME's start palette: the rainbow of allocate_palette)
	uint16_t dpram[0x800], prot[0x800];
	uint8_t soundram[0x10000];
	uint16_t scrollx[2], scrolly[2], tmcontrol;
	int32_t raster;
	uint8_t video_off, rowscroll_on, mculatch, mculatch_pending, pad[2];
	uint32_t mcu_sample_addr;
	int64_t cpu_end, mcu_next;
	uint32_t frame;
	uint8_t state_end;
} t;
static uint32_t s_line;

typedef struct {
	uint16_t end, scrollx[2], scrolly[2], video_off, rowscroll_on, tmcontrol;
} seg_t;

static seg_t s_seg[MAX_SEG];
static int s_nseg;
static struct {
	seg_t seg[MAX_SEG + 1];
	int nseg;
	uint16_t vram[2][0x8000], spr[0x200], spr2[0x200], rowscroll[0x200], palette[2][0x600];
	uint8_t pen_set[2][256];
} s_vs;

static void seg_capture(int line)
{
	if (s_nseg >= MAX_SEG || line <= 0 || line > VBLANK_LINE)
		return;
	seg_t *g = &s_seg[s_nseg++];
	g->end = (uint16_t)line;
	memcpy(g->scrollx, t.scrollx, sizeof(g->scrollx));
	memcpy(g->scrolly, t.scrolly, sizeof(g->scrolly));
	g->video_off = t.video_off;
	g->rowscroll_on = t.rowscroll_on;
	g->tmcontrol = t.tmcontrol;
}

static void capture(void)
{
	rbx_video_wait_idle();
	seg_capture(VBLANK_LINE);
	memcpy(s_vs.seg, s_seg, sizeof(seg_t) * (size_t)s_nseg);
	s_vs.nseg = s_nseg;
	s_nseg = 0;
	const size_t vb = s_k->kind == K_M82 ? 0x10000 : 0x4000;
	memcpy(s_vs.vram[0], t.vram[0], 0x4000);
	memcpy(s_vs.vram[1], t.vram[1], vb);
	memcpy(s_vs.spr, t.spritebuf, sizeof(s_vs.spr));
	memcpy(s_vs.palette, t.palette, sizeof(s_vs.palette));
	memcpy(s_vs.pen_set, t.pen_set, sizeof(s_vs.pen_set));
	if (s_k->kind == K_M82) {
		memcpy(s_vs.spr2, t.spriteram2, sizeof(s_vs.spr2));
		memcpy(s_vs.rowscroll, t.rowscroll, sizeof(s_vs.rowscroll));
	}
	rbx_video_frame_done();
}

// a layer's pixels of a line: pens, and the group of the tile in the top bits of the flags
static uint16_t s_px[2][VIS_W];
static uint8_t s_grp[2][VIS_W];

static void layer_line(int lay, const seg_t *g, int y, int *any)
{
	const uint16_t *vram = s_vs.vram[lay];
	const rbx_gfx *gx = lay == 0 || s_k->rtype2_tiles ? &s_t0 : s_bg_gfx;
	const uint8_t *rows = lay == 0 || s_k->rtype2_tiles ? s_t0_rows : s_bg_rows;
	int large = 0, width = 512;
	int scrollx = g->scrollx[lay];
	const int scrolly = g->scrolly[lay];
	int dx = s_k->dx[lay];
	const int sy = (y + 128 + scrolly) & 511;
	if (s_k->kind == K_M82 && lay == 1) {
		large = (g->tmcontrol & 0x40) != 0;
		if (large) {
			width = 1024;
			dx = 4 - 256;
		}
		if (g->rowscroll_on)
			scrollx = s_vs.rowscroll[(sy - scrolly) & 0x1ff];
	}
	int px = (VIS_X0 - dx + scrollx) & (width - 1);
	const int row = sy >> 3, ty = sy & 7;
	const uint32_t rowbase = large ? (uint32_t)row * 256 : (uint32_t)row * 64;
	const int cols = large ? 128 : 64;
	int col = px >> 3, off = px & 7, x = 0;
	uint16_t *d = s_px[lay];
	uint8_t *fl = s_grp[lay];
	int seen = 0;
	while (x < VIS_W) {
		const uint32_t idx = (rowbase + (uint32_t)col) * 2;
		const uint32_t code0 = vram[idx], attr = vram[idx + 1];
		uint32_t code;
		int fx, fy, group;
		if (s_k->rtype2_tiles) {
			code = code0;
			fx = (attr >> 5) & 1;
			fy = (attr >> 6) & 1;
			group = (attr >> 7) & 3;
		} else {
			code = code0 & 0x3fff;
			fx = (code0 >> 14) & 1;
			fy = (code0 >> 15) & 1;
			group = (attr >> 6) & 3;
		}
		code = rbx_wrap(code, gx->count);
		const int n = 8 - off < VIS_W - x ? 8 - off : VIS_W - x;
		const int row8 = fy ? 7 - ty : ty;
		const uint32_t pen = 256 + ((attr & 0xf) << 4);
		const uint8_t *s = gx->pix + (size_t)code * 64 + row8 * 8;
		col = (col + 1) & (cols - 1);
		seen |= rows[code] != 0;
		for (int i = 0; i < n; i++) {
			const uint32_t p = s[fx ? 7 - off - i : off + i];
			d[x + i] = (uint16_t)(pen + p);
			fl[x + i] = (uint8_t)group;
		}
		x += n;
		off = 0;
	}
	if (any)
		*any = seen;
}

static void m72_line(const seg_t *g, int y, uint16_t *d, uint8_t *pri)
{
	layer_line(0, g, y, NULL);
	layer_line(1, g, y, NULL);
	for (int x = 0; x < VIS_W; x++) {
		const uint32_t bp = s_px[1][x], fp = s_px[0][x];
		const int bpen = bp & 15, fpen = fp & 15, bg = s_grp[1][x], fgp = s_grp[0][x];
		// (the two passes of screen_update: the sprites' priority with the first transmasks, then the second)
		const int bf_pri = bg == 0 ? 0 : bg == 1 ? bpen >= 8 : bpen != 0;
		const int bf = bg == 0 || bg == 2 ? 0 : bg == 1 ? bpen >= 8 : bpen != 0;
		const int ff = fgp == 0 ? 0 : fgp == 1 ? fpen >= 8 : fpen != 0;
		const int fb = fgp == 0 ? fpen != 0 : fgp == 1 ? (fpen != 0 && fpen < 8) : 0;
		d[x] = (uint16_t)(ff ? fp : bf ? bp : fb ? fp : bp);
		pri[x] = (uint8_t)(bf_pri | ff);
	}
}

// the M82: tiles behind the sprites to the screen, those over them to f (0xffff: none), for after the sprites
#define M82_NONE 0xffff
static uint16_t (*s_m82_front_hp)[VIS_H][VIS_W];
#define s_m82_front (*s_m82_front_hp)

static void m82_line(const seg_t *g, int y, uint16_t *d, uint16_t *f)
{
	layer_line(0, g, y, NULL);
	layer_line(1, g, y, NULL);
	for (int x = 0; x < VIS_W; x++) {
		const uint32_t bp = s_px[1][x], fp = s_px[0][x];
		const int bpen = bp & 15, fpen = fp & 15, bg = s_grp[1][x], fgp = s_grp[0][x];
		const int fb = fgp == 0 ? fpen != 0 : fgp == 1 ? (fpen != 0 && fpen < 8) : 0;
		const int bb = bg == 0 ? 1 : bg == 1 ? bpen < 8 : bpen == 0;
		if (fb)
			d[x] = (uint16_t)fp;
		else if (bb)
			d[x] = (uint16_t)bp;
		const int ff = fgp == 0 ? 0 : fgp == 1 ? fpen >= 8 : fpen != 0;
		const int bf = bg == 0 ? 0 : bg == 1 ? bpen >= 8 : bpen != 0;
		f[x] = ff ? (uint16_t)fp : bf ? (uint16_t)bp : M82_NONE;
	}
}

static void sprite_blit(const rbx_gfx *gx, uint32_t code, int color, int fx, int fy, int x, int y, uint16_t *bm, int pitch, uint32_t pmask)
{
	rbx_gfx_prio_transpen(gx, code, (uint16_t)(color << 4), fx, fy, x - VIS_X0, y, bm, s_pri, pitch, VIS_W, VIS_H, pmask);
}

static void draw_sprites(uint16_t *bm, int pitch)
{
	static int list[0x200];
	const uint16_t *spriteram = s_vs.spr;
	int n = 0;
	for (int i = 0, w = 0; i < 0x200; i += w * 4) {
		list[n++] = i;
		w = 1 << ((spriteram[i + 2] & 0xc000) >> 14);
	}
	for (int i = n - 1; i >= 0; i--) {
		const int offs = list[i];
		const uint32_t code = spriteram[offs + 1];
		const int color = spriteram[offs + 2] & 0x0f;
		const int sx = -256 + (spriteram[offs + 3] & 0x3ff);
		int sy = 384 - (spriteram[offs + 0] & 0x1ff);
		const int flipx = (spriteram[offs + 2] & 0x0800) != 0;
		const int flipy = (spriteram[offs + 2] & 0x0400) != 0;
		const int w = 1 << ((spriteram[offs + 2] & 0xc000) >> 14);
		const int h = 1 << ((spriteram[offs + 2] & 0x3000) >> 12);
		sy -= 16 * h;
		for (int x = 0; x < w; x++)
			for (int y = 0; y < h; y++) {
				uint32_t c = code;
				c += flipx ? (uint32_t)(8 * (w - 1 - x)) : (uint32_t)(8 * x);
				c += flipy ? (uint32_t)(h - 1 - y) : (uint32_t)y;
				sprite_blit(&s_spr, c, color, flipx, flipy, sx + 16 * x, sy + 16 * y, bm, pitch, ~1u);
			}
	}
}

static void majtitle_sprites(uint16_t *bm, int pitch)
{
	const uint16_t *sr = s_vs.spr2;
	for (int offs = 0; offs < 0x200; offs += 4) {
		const uint32_t code = sr[offs + 1];
		const int color = sr[offs + 2] & 0x0f;
		const int sx = -256 + (sr[offs + 3] & 0x3ff);
		int sy = 384 - (sr[offs + 0] & 0x1ff);
		const int flipx = (sr[offs + 2] & 0x0800) != 0;
		const int flipy = (sr[offs + 2] & 0x0400) != 0;
		const int h = 1 << ((sr[offs + 2] & 0x3000) >> 12);
		sy -= 16 * h;
		for (int y = 0; y < h; y++) {
			const uint32_t c = rbx_wrap(code + (flipy ? (uint32_t)(h - 1 - y) : (uint32_t)y), s_spr2.count);
			if (!s_spr2.used[c])
				continue;
			const uint8_t *src = s_spr2.pix + (size_t)c * 256;
			for (int j = 0; j < 16; j++) {
				const int yy = sy + 16 * y + j;
				if (yy < 0 || yy >= VIS_H)
					continue;
				const uint8_t *row = src + (flipy ? 15 - j : j) * 16;
				uint16_t *d = bm + yy * pitch;
				for (int i = 0; i < 16; i++) {
					const int xx = sx - VIS_X0 + i;
					const int p = row[flipx ? 15 - i : i];
					if (p && xx >= 0 && xx < VIS_W)
						d[xx] = (uint16_t)((color << 4) + p);
				}
			}
		}
	}
}

static void m72_render(void)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	uint32_t *pal = rbx_video.palette;
	for (int n = 0; n < 2; n++)
		for (int i = 0; i < 256; i++) {
			const uint32_t r = s_vs.palette[n][i] & 0x1f, g = s_vs.palette[n][i + 0x200] & 0x1f, b = s_vs.palette[n][i + 0x400] & 0x1f;
			const int e = (n << 8) + i;
			pal[e] = s_vs.pen_set[n][i] ? (r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (b << 3 | b >> 2)
			                            : ((e & 1) ? 0xff0000u : 0) | ((e & 2) ? 0xff00u : 0) | ((e & 4) ? 0xffu : 0);
		}
	pal[512] = 0;
	const int m82 = s_k->kind == K_M82;
	int sn = 0;
	for (int y = 0; y < VIS_H; y++) {
		while (sn < s_vs.nseg - 1 && y >= s_vs.seg[sn].end)
			sn++;
		const seg_t *g = &s_vs.seg[sn];
		uint16_t *d = bm + y * pitch;
		uint8_t *pri = s_pri + y * pitch;
		if (g->video_off) {
			for (int x = 0; x < VIS_W; x++)
				d[x] = 512;
			memset(pri, 31, VIS_W);
			if (m82)
				memset(s_m82_front[y], 0xff, sizeof(s_m82_front[y]));
			continue;
		}
		if (m82) {
			memset(pri, 0, VIS_W);
			m82_line(g, y, d, s_m82_front[y]);
		} else
			m72_line(g, y, d, pri);
	}
	if (m82) {
		majtitle_sprites(bm, pitch);
		draw_sprites(bm, pitch);
		for (int y = 0; y < VIS_H; y++) {
			uint16_t *d = bm + y * pitch;
			const uint16_t *f = s_m82_front[y];
			for (int x = 0; x < VIS_W; x++)
				if (f[x] != M82_NONE)
					d[x] = f[x];
		}
	} else
		draw_sprites(bm, pitch);
}

// ---- the main board ----
static void pic_out(int state) { nec_set_input(&s_cpu, NEC_LINE_IRQ, state); }
static int pic_ack(void) { return pic8259_ack(&s_pic); }
static inline void combine(uint16_t *r, uint16_t data, uint16_t mask) { *r = (uint16_t)((*r & ~mask) | (data & mask)); }

static void irq_line_w(uint16_t data)
{
	t.raster = (data & 0x1ff) - 128;
	if (s_k->kind == K_KENGO)
		nec_set_input(&s_cpu, NEC_LINE_INTP2, 0);
	else
		pic8259_set_irq(&s_pic, 2, 0);
}

static void dmaon_w(void) { memcpy(t.spritebuf, t.spriteram, sizeof(t.spritebuf)); }

static void prot_copy(uint16_t *dest, const uint8_t *src, int bytes)
{
	for (int i = 0; i < bytes; i += 2)
		dest[i / 2] = (uint16_t)(src[i] | src[i + 1] << 8);
}

// ---- dual port RAM: left the main CPU, right the MCU ----
static uint16_t dpram_left_r(uint32_t o)
{
	return t.dpram[o & 0x7ff];
}
static void dpram_left_w(uint32_t o, uint16_t d, uint16_t m)
{
	o &= 0x7ff;
	combine(&t.dpram[o], d, m);
	if (o == 0x7ff)
		mcs51_set_input(&s_mcu, MCS51_INT0_LINE, 1);
}
static uint8_t mcu_xr(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0xc000 && a < 0xd000) {
		const uint32_t o = (a - 0xc000u) >> 1;
		if (o == 0x7ff)
			mcs51_set_input(&s_mcu, MCS51_INT0_LINE, 0);
		return (uint8_t)(t.dpram[o] >> ((a & 1) ? 8 : 0));
	}
	if (a == 0)
		return s_samples ? s_samples[t.mcu_sample_addr++ % (s_samples_size ? s_samples_size : 1)] : 0;
	if (a == 2)
		return t.mculatch;
	return 0;
}
static void mcu_xw(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xd000) {
		const uint32_t o = (a - 0xc000u) >> 1;
		if (a & 1)
			combine(&t.dpram[o], (uint16_t)(v << 8), 0xff00);
		else
			combine(&t.dpram[o], v, 0x00ff);
	} else if (a == 0)
		t.mcu_sample_addr = (t.mcu_sample_addr & 0xffe000) | ((uint32_t)v << 5);
	else if (a == 1)
		t.mcu_sample_addr = (t.mcu_sample_addr & 0x1fff) | ((uint32_t)v << 13);
	else if (a == 2) {
		t.mculatch_pending = 0;
		mcs51_set_input(&s_mcu, MCS51_INT1_LINE, 0);
	}
}
static void mcu_port_out(void *u, int port, uint8_t data)
{
	(void)u;
	if (port == 1)
		zsnd_dac_main((int)s_line, data);
}
static void mculatch_w(uint8_t v, int pending)
{
	t.mculatch = v;
	t.mculatch_pending = (uint8_t)pending;
	mcs51_set_input(&s_mcu, MCS51_INT1_LINE, pending);
}

static void map_common(void)
{
	const m72_cfg *k = s_k;
	nec_map(&s_cpu, 0, k->rom_end, s_rom, NULL);
	nec_map(&s_cpu, k->ram, k->ram + 0x4000, (uint8_t *)t.ram, (uint8_t *)t.ram);
	nec_map(&s_cpu, 0xc0000, 0xc1000, (uint8_t *)t.spriteram, (uint8_t *)t.spriteram);
	nec_map(&s_cpu, k->vram[0], k->vram[0] + 0x4000, (uint8_t *)t.vram[0], (uint8_t *)t.vram[0]);
	if (k->kind == K_M82) {
		nec_map(&s_cpu, k->vram[1], k->vram[1] + 0x10000, (uint8_t *)t.vram[1], (uint8_t *)t.vram[1]);
		nec_map(&s_cpu, 0xa0000, 0xa1000, (uint8_t *)t.rowscroll, (uint8_t *)t.rowscroll);
		nec_map(&s_cpu, 0xc8000, 0xc9000, (uint8_t *)t.spriteram2, (uint8_t *)t.spriteram2);
	} else
		nec_map(&s_cpu, k->vram[1], k->vram[1] + 0x4000, (uint8_t *)t.vram[1], (uint8_t *)t.vram[1]);
	if (k->kind == K_M72)
		nec_map(&s_cpu, 0xe0000, 0xf0000, t.soundram, t.soundram);
	s_cpu.omap[0xff] = s_rom + 0xff000;
}

static uint16_t main_r16(uint32_t a)
{
	const m72_cfg *k = s_k;
	for (int n = 0; n < 2; n++)
		if (a >= k->pal[n] && a < k->pal[n] + 0xc00)
			return (uint16_t)(t.palette[n][((a - k->pal[n]) >> 1) & ~0x100u] | 0xffe0);
	if (k->mcu && a >= 0xb0000 && a < 0xb1000)
		return dpram_left_r((a - 0xb0000) >> 1);
	if (k->prot && a >= 0xb0000 && a < 0xb1000) {
		if (a == 0xb0ffa)
			prot_copy(t.prot, s_prot_code, CODE_LEN);
		return t.prot[(a - 0xb0000) >> 1];
	}
	if (a >= 0xffff0)
		return *(const uint16_t *)(s_rom + (a & 0xffffe));
	return 0;
}
static uint8_t main_r8(uint32_t a)
{
	const m72_cfg *k = s_k;
	if (k->prot && a == 0xb0ffa)     // (an access of the high byte only copies the code)
		return (uint8_t)t.prot[0xffa / 2];
	const uint16_t v = main_r16(a & ~1u);
	return (a & 1) ? (uint8_t)(v >> 8) : (uint8_t)v;
}
static void main_wm(uint32_t a, uint16_t v, uint16_t m)
{
	const m72_cfg *k = s_k;
	for (int n = 0; n < 2; n++)
		if (a >= k->pal[n] && a < k->pal[n] + 0xc00) {
			combine(&t.palette[n][((a - k->pal[n]) >> 1) & ~0x100u], v, m);
			t.pen_set[n][((a - k->pal[n]) >> 1) & 0xff] = 1;
			return;
		}
	if (k->mcu && a >= 0xb0000 && a < 0xb1000)
		dpram_left_w((a - 0xb0000) >> 1, v, m);
	else if (k->prot && a >= 0xb0000 && a < 0xb1000) {
		combine(&t.prot[(a - 0xb0000) >> 1], (uint16_t)(v ^ 0xffff), m);
		if (a == 0xb0ffe && (m & 0xff00) && (v >> 8) == 0)
			prot_copy(&t.prot[0x7f0], s_prot_crc, CRC_LEN);
	} else if (k->kind == K_M84 || k->kind == K_KENGO) {
		if (a == 0xb0000)
			irq_line_w(v);
		else if (a == 0xbc000 && (m & 0xff))
			dmaon_w();
	} else if (k->kind == K_M82) {
		if (a == 0xe0000)
			irq_line_w(v);
		else if (a == 0xec000 && (m & 0xff))
			dmaon_w();
	}
}
static void main_w8(uint32_t a, uint8_t v)
{
	if (a & 1)
		main_wm(a & ~1u, (uint16_t)(v << 8), 0xff00);
	else
		main_wm(a, v, 0x00ff);
}
static void main_w16(uint32_t a, uint16_t v) { main_wm(a, v, 0xffff); }

static void port02_w(uint8_t v)
{
	t.video_off = (v >> 3) & 1;
	if (s_k->kind == K_M72)   // (bit 4 resets the sound CPU, active low)
		zsnd_reset_line(!((v >> 4) & 1));
}

static void io_wm(uint32_t p, uint16_t v, uint16_t m)
{
	const m72_cfg *k = s_k;
	switch (p) {
	case 0:
		if (m & 0xff)
			zsnd_latch_w((uint8_t)v);
		break;
	case 2:
		if (m & 0xff)
			port02_w((uint8_t)v);
		break;
	case 4:
		if ((k->kind == K_M72 || k->kind == K_M81) && (m & 0xff))
			dmaon_w();
		break;
	case 6:
		if (k->kind == K_M72 || k->kind == K_M81)
			irq_line_w(v);
		break;
	case 0x40:
	case 0x42:
		if ((m & 0xff) && k->kind != K_KENGO)
			pic8259_write(&s_pic, (int)(p - 0x40) >> 1, (uint8_t)v);
		break;
	case 0x80: combine(&t.scrolly[0], v, m); break;
	case 0x82: combine(&t.scrollx[0], v, m); break;
	case 0x84: combine(&t.scrolly[1], v, m); break;
	case 0x86: combine(&t.scrollx[1], v, m); break;
	case 0x8c:
		if (k->kind == K_M82)
			combine(&t.tmcontrol, v, m);
		break;
	case 0x8e:
		if (k->kind == K_M82 && (m & 0xff00))
			t.rowscroll_on = (v & 0xff00) != 0;
		break;
	case 0xc0:
		if (k->airduel) {
			if (m & 0xff)
				mculatch_w((uint8_t)v, 1);
		} else if (k->mcu) {
			if (m & 0xff)
				zsnd_latch2_w((uint8_t)v);
		} else if (k->prot == P_DBREED) {
			static const int a[9] = { 0x00000, 0x00020, 0x02c40, 0x08160, 0x0c8c0, 0x0ffe0, 0x13000, 0x15820, 0x15f40 };
			if ((m & 0xff) && (v & 0xff) < 9)
				zsnd_sample_start((uint32_t)a[v & 0xff]);
		} else if (k->prot == P_DKGEN) {
			static const int a[28] = { 0x00000, 0x00020, 0x01800, 0x02da0, 0x03be0, 0x05ae0, 0x06100, 0x06de0,
			                           0x07260, 0x07a60, 0x08720, 0x0a5c0, 0x0c3c0, 0x0c7a0, 0x0e140, 0x0fb00,
			                           0x10fa0, 0x10fc0, 0x10fe0, 0x11f40, 0x12b20, 0x130a0, 0x13c60, 0x14740,
			                           0x153c0, 0x197e0, 0x1af40, 0x1c080 };
			if ((m & 0xff) && (v & 0xff) < 28)
				zsnd_sample_start((uint32_t)a[v & 0xff]);
		}
		break;
	}
}
static void main_out8(uint32_t p, uint8_t v)
{
	if (p & 1)
		io_wm(p & ~1u, (uint16_t)(v << 8), 0xff00);
	else
		io_wm(p, v, 0x00ff);
}
static void main_out16(uint32_t p, uint16_t v) { io_wm(p, v, 0xffff); }
static uint16_t main_in16(uint32_t p)
{
	switch (p) {
	case 0: return (uint16_t)rbx_port_read(s_port_in0);
	case 2: return (uint16_t)rbx_port_read(s_port_in1);
	case 4: return (uint16_t)rbx_port_read(s_port_dsw);
	case 0x40: return s_k->kind == K_KENGO ? 0 : pic8259_read(&s_pic, 0);
	case 0x42: return s_k->kind == K_KENGO ? 0 : pic8259_read(&s_pic, 1);
	}
	return 0;
}
static uint8_t main_in8(uint32_t p)
{
	if (p == 0x41 || p == 0x43)
		return 0;
	const uint16_t v = main_in16(p & ~1u);
	return (p & 1) ? (uint8_t)(v >> 8) : (uint8_t)v;
}

// ---- init / frame ----
static void m72_exit(void);
static void m72_reset(void);

static const m72_cfg *find_cfg(const char *m)
{
	for (size_t i = 0; i < sizeof(s_cfgs) / sizeof(s_cfgs[0]); i++)
		if (!strcmp(m, s_cfgs[i].machine))
			return &s_cfgs[i];
	return NULL;
}

static int m72_init(const rbx_game *g)
{
	if (!s_m82_front_hp && !(s_m82_front_hp = calloc(1, sizeof(*s_m82_front_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t ss = 0, s2 = 0, t0 = 0, t1 = 0;
	s_k = find_cfg(g->machine);
	s_rom = rbx_region("maincpu", &s_rom_size);
	const uint8_t *spr = rbx_region("sprites", &ss), *sp2 = rbx_region("sprites2", &s2);
	const uint8_t *tl0 = rbx_region("tiles0", &t0), *tl1 = rbx_region("tiles1", &t1);
	s_mcu_rom = rbx_region("mcu", &s_mcu_size);
	s_samples_size = 0;
	s_samples = rbx_region("samples", &s_samples_size);
	if (!s_k || !s_rom || s_rom_size < 0x100000 || !spr || !tl0 || (s_k->mcu && !s_mcu_rom)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	s_port_in0 = rbx_port_find("IN0");
	s_port_in1 = rbx_port_find("IN1");
	s_port_dsw = rbx_port_find("DSW");
	s_port_j3 = rbx_port_find("JumperJ3");
	s_prot_code = s_k->prot == P_DBREED ? dbreedm72_code : dkgenm72_code;
	s_prot_crc = s_k->prot == P_DBREED ? dbreedm72_crc : dkgenm72_crc;
	int bad = irem_gfx_decode(&s_spr, spr, ss, IGFX_SPRITE, NULL) || irem_gfx_decode(&s_t0, tl0, t0, IGFX_CHAR, &s_t0_rows);
	if (tl1 && !bad)
		bad = irem_gfx_decode(&s_t1, tl1, t1, IGFX_CHAR, &s_t1_rows);
	if (sp2 && !bad)
		bad = irem_gfx_decode(&s_spr2, sp2, s2, IGFX_SPRITE, NULL);
	zsnd_cfg z;
	memset(&z, 0, sizeof(z));
	z.ports = s_k->ports;
	z.nmi = s_k->nmi;
	z.nmi_lines = 2;   // (MASTER_CLOCK / 8 / 512: two lines)
	z.samples = s_samples;
	z.samples_size = s_samples_size;
	z.ram = s_k->kind == K_M72 ? t.soundram : NULL;
	z.ym_gain = z.dac_gain = 84;   // (0.33; both YM routes go to the speaker at that gain)
	z.frame_mhz = FRAME_MHZ;
	z.lines = LINES;
	z.mcu_latch = s_k->mcu ? mculatch_w : NULL;
	if (bad || rbx_video_setup(VIS_W, VIS_H, NPENS) || zsnd_init(&z)) {
		rbx_fail("%s: out of memory", g->set);
		m72_exit();
		return -1;
	}
	s_pri = (uint8_t *)malloc((size_t)rbx_video.pitch * VIS_H);
	if (!s_pri) {
		m72_exit();
		return -1;
	}
	rbx_audio.frame_mhz = FRAME_MHZ;
	s_pic.out_int = pic_out;
	m72_reset();
	return 0;
}

static void m72_exit(void)
{
	zsnd_exit();
	irem_gfx_free(&s_spr);
	irem_gfx_free(&s_spr2);
	irem_gfx_free(&s_t0);
	irem_gfx_free(&s_t1);
	free(s_t0_rows);
	free(s_t1_rows);
	free(s_pri);
	s_t0_rows = s_t1_rows = s_pri = NULL;
	memset(&s_spr2, 0, sizeof(s_spr2));
	memset(&s_t1, 0, sizeof(s_t1));
	free(s_m82_front_hp);
	s_m82_front_hp = NULL;
}

static void m72_reset(void)
{
	const m72_cfg *k = s_k;
	zsnd_reset(k->kind == K_M72);
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	t.raster = -1;
	if (k->prot)
		memset(t.prot, 0, sizeof(t.prot));
	s_nseg = 0;
	s_line = 0;
	memset(&s_vs, 0, sizeof(s_vs));
	if (k->kind != K_KENGO) {
		pic8259_reset(&s_pic);
		nec_init(&s_cpu, NEC_V30, NULL);
		s_cpu.irq_ack = pic_ack;
	} else
		nec_init(&s_cpu, NEC_V35, !strcmp(k->machine, "kengo") ? gunforce_decryption_table : NULL);
	s_cpu.read8 = main_r8;
	s_cpu.read16 = main_r16;
	s_cpu.write8 = main_w8;
	s_cpu.write16 = main_w16;
	s_cpu.in8 = main_in8;
	s_cpu.in16 = main_in16;
	s_cpu.out8 = main_out8;
	s_cpu.out16 = main_out16;
	map_common();
	nec_reset(&s_cpu);
	if (k->mcu) {
		mcs51_init(&s_mcu, s_mcu_rom, s_mcu_size);
		s_mcu.xdata_r = mcu_xr;
		s_mcu.xdata_w = mcu_xw;
		s_mcu.port_out = mcu_port_out;
		mcs51_reset(&s_mcu);
	}
}

static void m72_frame(void)
{
	const m72_cfg *k = s_k;
	const int v35 = k->kind == K_KENGO;
	const int cyc = v35 ? HTOTAL * 2 : HTOTAL;   // (the V35 counts its input clock, 16 MHz)
	// (on M81 the source of the BG tiles depends on jumper J3; read here: the ports have no values yet at reset)
	const int bg_t1 = k->kind == K_M72 || (k->kind == K_M81 && s_port_j3 >= 0 && (rbx_port_read(s_port_j3) & 1));
	s_bg_gfx = bg_t1 && s_t1.pix ? &s_t1 : &s_t0;
	s_bg_rows = bg_t1 && s_t1.pix ? s_t1_rows : s_t0_rows;
	const int quantum = !v35 && t.frame < 25 * FRAME_MHZ / 1000;   // (machine_reset's add_quantum: MASTER_CLOCK / 4 / 12, 25 s)
	s_cpu.rep_q = quantum ? 12 : 0;
	for (int i = 0; i < LINES; i++) {
		const int vpos = (VBLANK_LINE + i) % LINES;
		s_line = (uint32_t)i;
		if (v35) {
			if (vpos == t.raster) {
				seg_capture(vpos + 1);
				nec_set_input(&s_cpu, NEC_LINE_INTP2, 1);
			} else
				nec_set_input(&s_cpu, NEC_LINE_INTP2, 0);
			if (vpos == VBLANK_LINE)
				capture();
			nec_set_input(&s_cpu, NEC_LINE_INTP0, vpos == VBLANK_LINE);
		} else {
			if (vpos == t.raster) {
				seg_capture(vpos + 1);
				pic8259_set_irq(&s_pic, 2, 1);
			} else
				pic8259_set_irq(&s_pic, 2, 0);
			if (vpos == VBLANK_LINE)
				capture();
			pic8259_set_irq(&s_pic, 0, vpos == VBLANK_LINE);
		}
		t.cpu_end += cyc;
		if (k->mcu) {
			// (the MCU at 8 MHz / 12, a machine cycle at a time while MAME's quantum lasts: the protection's handshake
			// leaves a jump open for two of them; then as many as MCU_SLICE at once)
			const int m = quantum ? 1 : MCU_SLICE;
			while (t.mcu_next + 12 * m <= t.cpu_end) {
				t.mcu_next += 12 * m;
				nec_run_until(&s_cpu, t.mcu_next);
				mcs51_execute(&s_mcu, m);
			}
		}
		nec_run_until(&s_cpu, t.cpu_end);
		zsnd_line(i);
	}
	zsnd_frame();
	t.frame++;
}

static int m72_state_io(void *ctx, int load)
{
	int rc = 0;
	rbx_video_wait_idle();
	rbx_aux2_sync();
	static uint8_t cpu[sizeof(nec_t)];
	if (!load)
		memcpy(cpu, &s_cpu, NEC_STATE_SIZE);
	rc |= rbx_state_io(ctx, "m72_cpu", cpu, NEC_STATE_SIZE);
	rc |= rbx_state_io(ctx, "m72", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "m72_pic", &s_pic, PIC8259_STATE_SIZE);
	int32_t ns = s_nseg;
	rc |= rbx_state_io(ctx, "m72_ns", &ns, sizeof(ns));
	rc |= rbx_state_io(ctx, "m72_seg", s_seg, sizeof(s_seg));
	if (s_k->mcu)
		rc |= rbx_state_io(ctx, "m72_mcu", &s_mcu.st, sizeof(s_mcu.st));
	rc |= zsnd_state(ctx, load);
	if (!rc && load) {
		memcpy(&s_cpu, cpu, NEC_STATE_SIZE);
		nec_after_load(&s_cpu);
		s_nseg = ns >= 0 && ns <= MAX_SEG ? ns : 0;
	}
	return rc ? -1 : 0;
}

static int m72_save(void *ctx) { return m72_state_io(ctx, 0); }
static int m72_load(void *ctx) { return m72_state_io(ctx, 1); }
static int m72_supported(const char *machine) { return find_cfg(machine) != NULL; }

const rbx_driver rbx_drv_m72 = { "Irem M72", m72_init, m72_exit, m72_reset, m72_frame, m72_save, m72_load, m72_supported,
                                 m72_render, NULL };
