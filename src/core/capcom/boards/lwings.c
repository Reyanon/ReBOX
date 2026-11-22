/*  Capcom Legendary Wings, Section Z, Trojan / Tatakai no Banka (and the Buraiken bootleg). Ported from MAME
    src/mame/capcom/lwings.cpp (BSD-3-Clause, copyright-holders: Paul Leaman).
    Z80 at 6 MHz (Section Z, Trojan 3 MHz; NMI at line 248 when enabled, IRQ at line 128; the sprite DMA holds it off
    the bus while it copies), Z80 sound CPU at 3 MHz with two YM2203 at 1.5 MHz (IRQ 4 times a frame). Trojan: a third
    Z80 at 3 MHz feeding an MSM5205 (its IRQ on the VCK), a second 512x256 background from ROM, the front background
    split in two layers around the sprites. Video: a 512x512 tilemap of 16x16, a 256x256 text layer, 16x16 sprites
    from a buffered RAM; RGBx 444 palette RAM.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/msm5205.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "cap8.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 55408          // set_raw(12 MHz / 2, 384, 0, 256, 282, 8, 248)
#define LINES 282
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 256
#define SND_CLOCK 3000000
#define YM_CLOCK 1500000

static int s_trojan, s_rot0;
static uint8_t *s_rom, *s_zrom, *s_arom, *s_bg2rom;
static uint32_t s_rom_size, s_zrom_size, s_arom_size, s_bg2rom_size;
static uint32_t s_main_clock;
static uint16_t s_base_chr, s_base_bg1, s_base_spr;
static rbx_gfx s_gchar, s_gbg1, s_gspr, s_gbg2;
static int s_port[5];
static z80 s_main, s_snd, s_adp;
static msm5205_t s_msm;
static const uint8_t *s_mmap[256], *s_zmap[256], *s_amap[256];
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static int s_line;

static struct {
	uint8_t ram[0x2000], sprbuf[0x180], fg[0x800], bg1[0x800], pal[0x400], palext[0x400];
	uint8_t zram[0x800];
	uint8_t scrollx[2], scrolly[2], bg2_scrollx, bg2_image;
	uint8_t bank, flip, nmi_mask, sprbank, snd_reset, latch, latch_pending, latch2;
	int main_irq, snd_irq, adp_irq;
	uint64_t main_frac, snd_frac, adp_frac, ym_frac;
	uint32_t msm_acc;
	uint8_t state_end;
} *t;   // (calloc in init: ADR-040)

static struct {
	uint8_t spr[0x180], fg[0x800], bg1[0x800], pal[0x400], palext[0x400];
	uint16_t sx, sy;
	uint8_t bg2_sx, bg2_image, flip, sprbank;
} *s_snap;

static uint8_t z_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

// ---- sound ----
static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_zrom[a];
	if (a >= 0xc000 && a < 0xc800) return t->zram[a - 0xc000];
	if (a == 0xc800) {
		t->latch_pending = 0;
		return t->latch;
	}
	if (a == 0xe006) {   // avengers_soundlatch_ack_r
		const uint8_t v = (uint8_t)(t->latch | (t->latch_pending ? 0x80 : 0));
		t->latch_pending = 0;
		return v;
	}
	return 0;
}
static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xc800) t->zram[a - 0xc000] = v;
	else if (a == 0xe000 || a == 0xe001) rbx_ym_write(0, a & 1, v);
	else if (a == 0xe002 || a == 0xe003) rbx_ym_write(1, a & 1, v);
}

// the ADPCM CPU (trojan_adpcm_map / _io_map: no RAM)
static uint8_t adp_read(void *u, uint16_t a) { (void)u; return a < s_arom_size ? s_arom[a] : 0; }
static void adp_write(void *u, uint16_t a, uint8_t v) { (void)u; (void)a; (void)v; }
static uint8_t adp_in(z80 *z, uint8_t port)
{
	(void)z;
	return port == 0 ? t->latch2 : 0xff;
}
static void adp_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	if (port != 1)
		return;
	msm5205_set_selector(&s_msm, ((v >> 6) & 1) | 4);   // (s1_w: bit 6; 4-bit)
	msm5205_reset_w(&s_msm, (v >> 7) & 1);
	msm5205_data_w(&s_msm, v & 0x0f);
}
static void msm_vck(void) { t->adp_irq = 1; }

static void z_setup(z80 *z, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t), const uint8_t **map)
{
	z80_init(z);
	z->read_byte = rd;
	z->write_byte = wr;
	z->port_in = z_in;
	z->port_out = z_out;
	z->read_map = map;
}

// ---- main CPU ----
static void map_bank(void)
{
	for (int p = 0; p < 0x40; p++)
		s_mmap[0x80 + p] = s_rom + 0x10000 + (uint32_t)t->bank * 0x4000 + p * 256;
}

static int vblank(void) { return s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H; }

static void bankswitch_w(uint8_t v)
{
	t->flip = !(v & 1);
	t->bank = (v >> 1) & 3;
	map_bank();
	t->nmi_mask = (v >> 3) & 1;
	t->sprbank = (v >> 4) & 1;
	if ((v & 0x20) && !t->snd_reset) {
		z_setup(&s_snd, snd_read, snd_write, s_zmap);
		t->snd_irq = 0;
	}
	t->snd_reset = (v >> 5) & 1;
}

static void sprite_dma(void)   // (sprite_dma_trigger_w: in vblank the CPU gives the bus for 2 pixel clocks a byte)
{
	if (!vblank())
		return;
	memcpy(t->sprbuf, t->ram + 0x1e00, sizeof(t->sprbuf));
	s_main.cyc += (unsigned long)(0x180 * 2) * s_main_clock / 6000000;
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_rom[a];
	if (a < 0xc000) return s_rom[0x10000 + (uint32_t)t->bank * 0x4000 + (a - 0x8000)];
	if (a < 0xe000) return t->ram[a - 0xc000];
	if (a < 0xe800) return t->fg[a - 0xe000];
	if (a < 0xf000) return t->bg1[a - 0xe800];
	if (a >= 0xf808 && a <= 0xf80c) {
		static const int lw[5] = { 0, 1, 2, 3, 4 }, tr[5] = { 0, 1, 2, 4, 3 };   // (trojan: DSWB at f80b, DSWA at f80c)
		return (uint8_t)rbx_port_read(s_port[s_trojan ? tr[a - 0xf808] : lw[a - 0xf808]]);
	}
	return 0xff;   // (unmap_value_high)
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xc000) return;
	if (a < 0xe000) { t->ram[a - 0xc000] = v; return; }
	if (a < 0xe800) { t->fg[a - 0xe000] = v; return; }
	if (a < 0xf000) { t->bg1[a - 0xe800] = v; return; }
	if (a < 0xf400) { t->palext[a - 0xf000] = v; return; }
	if (a < 0xf800) { t->pal[a - 0xf400] = v; return; }
	if (!s_trojan) {
		switch (a) {
		case 0xf808: case 0xf809: t->scrollx[a & 1] = v; break;
		case 0xf80a: case 0xf80b: t->scrolly[a & 1] = v; break;
		case 0xf80c: t->latch = v; t->latch_pending = 1; break;
		case 0xf80d: sprite_dma(); break;
		case 0xf80e: bankswitch_w(v); break;
		}
		return;
	}
	switch (a) {
	case 0xf800: case 0xf801: t->scrollx[a & 1] = v; break;
	case 0xf802: case 0xf803: t->scrolly[a & 1] = v; break;
	case 0xf804: t->bg2_scrollx = v; break;
	case 0xf805: t->bg2_image = v; break;
	case 0xf808: sprite_dma(); break;
	case 0xf80c: t->latch = v; t->latch_pending = 1; break;
	case 0xf80d: t->latch2 = v; break;
	case 0xf80e: bankswitch_w(v); break;
	}
}

// ---- video ----
static void draw_bg1(int layer)   // layer: -1 whole (lwings), 1 back, 0 front (trojan split)
{
	const int flip = s_snap->flip;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y + s_snap->sy : y + s_snap->sy) & 511;
		for (int x = 0; x < VIS_W; x++) {   // TILEMAP_SCAN_COLS 32 x 32 of 16x16
			const uint32_t tx = (uint32_t)(flip ? 255 - x + s_snap->sx : x + s_snap->sx) & 511;
			const uint32_t idx = (tx >> 4) * 32 + (ty >> 4);
			const uint8_t c = s_snap->bg1[idx + 0x400];
			const uint32_t code = (uint32_t)(s_snap->bg1[idx] + ((c & 0xe0) << 3)) % s_gbg1.count;
			int fx, fy;
			if (s_trojan) {
				fx = (c & 0x10) != 0;
				fy = 0;
			} else {
				fx = (c & 0x08) != 0;
				fy = (c & 0x10) != 0;
			}
			const uint32_t px = fx ? 15 - (tx & 15) : tx & 15, py = fy ? 15 - (ty & 15) : ty & 15;
			const uint8_t p = s_gbg1.pix[code * 256 + py * 16 + px];
			if (layer >= 0) {   // (set_transmask: group 0 0xffff / 0x0001, group 1 0xf07f / 0x0f81)
				const uint32_t mask = (c & 8) ? (layer ? 0x0f81u : 0xf07fu) : (layer ? 0x0001u : 0xffffu);
				if ((mask >> p) & 1)
					continue;
			}
			d[x] = (uint16_t)(s_base_bg1 + (c & 7) * 16 + p);
		}
	}
}

static void draw_bg2(void)   // trojan: 32 x 16 of 16x16 from ROM (get_bg2_memory_offset)
{
	const int flip = s_snap->flip;
	const uint32_t mask = s_bg2rom_size - 1;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y : y) & 255;
		for (int x = 0; x < VIS_W; x++) {
			const uint32_t tx = (uint32_t)(flip ? 255 - x + s_snap->bg2_sx : x + s_snap->bg2_sx) & 511;
			const uint32_t ti = (((ty >> 4) * 0x800) | ((tx >> 4) * 2)) + (uint32_t)s_snap->bg2_image * 0x20;
			const uint8_t c = s_bg2rom[(ti + 1) & mask];
			const uint32_t code = (uint32_t)(s_bg2rom[ti & mask] + ((c & 0x80) << 1)) % s_gbg2.count;
			const uint32_t px = (c & 0x10) ? 15 - (tx & 15) : tx & 15, py = (c & 0x20) ? 15 - (ty & 15) : ty & 15;
			d[x] = (uint16_t)((c & 7) * 16 + s_gbg2.pix[code * 256 + py * 16 + px]);
		}
	}
}

static void draw_sprites(void)
{
	const int flip = s_snap->flip;
	for (int o = 0x180 - 4; o >= 0; o -= 4) {
		const uint8_t *s = &s_snap->spr[o];
		int sx = s[3] - 0x100 * (s[1] & 1), sy = s[2];
		if (sy > 0xf8)
			sy -= 0x100;
		uint32_t code;
		int color, fx, fy;
		if (s_trojan) {
			code = (uint32_t)(s[0] | ((((s[1] >> 7) & 1) << 2 | ((s[1] >> 5) & 1) << 1 | ((s[1] >> 6) & 1)) << 8));
			color = (s[1] & 0x0e) >> 1;
			if (s_rot0) {
				fx = (s[1] & 0x10) != 0;
				fy = 1;
			} else {
				fx = 0;
				fy = !(s[1] & 0x10);
			}
		} else {
			code = (uint32_t)(s[0] | (s[1] & 0xc0) << 2) + s_snap->sprbank * 0x400u;
			color = (s[1] & 0x38) >> 3;
			fx = (s[1] & 2) != 0;
			fy = (s[1] & 4) != 0;
		}
		if (flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			fx = !fx;
			fy = !fy;
		}
		cap8_transpen(&s_gspr, code, (uint16_t)(s_base_spr + color * 16), fx, fy, sx, sy, 15, VIS_Y0, VIS_Y0 + VIS_H, VIS_Y0, 0, VIS_W);
	}
}

static void lw_render(void)
{
	const int npal = s_trojan ? 1024 : 1024;
	for (int i = 0; i < npal; i++) {   // RGBx_444 (the ext byte high)
		const uint32_t w = (uint32_t)(s_snap->palext[i] << 8 | s_snap->pal[i]);
		rbx_video.palette[i] = ((w >> 12) & 15) * 17 << 16 | ((w >> 8) & 15) * 17 << 8 | ((w >> 4) & 15) * 17;
	}
	if (s_trojan) {
		draw_bg2();
		draw_bg1(1);
		draw_sprites();
		draw_bg1(0);
	} else {
		draw_bg1(-1);
		draw_sprites();
	}
	const int flip = s_snap->flip;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y : y) & 255;
		for (int x = 0; x < VIS_W; x++) {
			const uint32_t tx = (uint32_t)(flip ? 255 - x : x) & 255;
			const uint32_t idx = (ty >> 3) * 32 + (tx >> 3);
			const uint8_t c = s_snap->fg[idx + 0x400];
			const uint32_t code = (uint32_t)(s_snap->fg[idx] + ((c & 0xc0) << 2)) % s_gchar.count;
			const uint32_t px = (c & 0x10) ? 7 - (tx & 7) : tx & 7, py = (c & 0x20) ? 7 - (ty & 7) : ty & 7;
			const uint8_t p = s_gchar.pix[code * 64 + py * 8 + px];
			if (p != 3)
				d[x] = (uint16_t)(s_base_chr + (c & 0x0f) * 4 + p);
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spr, t->sprbuf, sizeof(t->sprbuf));
	memcpy(s_snap->fg, t->fg, sizeof(t->fg));
	memcpy(s_snap->bg1, t->bg1, sizeof(t->bg1));
	memcpy(s_snap->pal, t->pal, sizeof(t->pal));
	memcpy(s_snap->palext, t->palext, sizeof(t->palext));
	s_snap->sx = (uint16_t)(t->scrollx[0] | t->scrollx[1] << 8);
	s_snap->sy = (uint16_t)(t->scrolly[0] | t->scrolly[1] << 8);
	s_snap->bg2_sx = t->bg2_scrollx;
	s_snap->bg2_image = t->bg2_image;
	s_snap->flip = t->flip;
	s_snap->sprbank = t->sprbank;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void lw_reset(void)
{
	memset(t, 0, offsetof(__typeof__(*t), state_end));
	map_bank();
	z_setup(&s_main, main_read, main_write, s_mmap);
	z_setup(&s_snd, snd_read, snd_write, s_zmap);
	if (s_trojan) {
		z_setup(&s_adp, adp_read, adp_write, s_amap);
		s_adp.port_in = adp_in;
		s_adp.port_out = adp_out;
		msm5205_reset(&s_msm);
		msm5205_set_selector(&s_msm, MSM5205_S96_4B);
	}
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	s_mix_n = 0;
	s_line = 0;
}

static void lw_exit(void);

static int lw_init(const rbx_game *g)
{
	free(t);
	t = calloc(1, sizeof(*t));
	if (!t) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	const char *m = g->machine;
	s_trojan = !strcmp(m, "trojan") || !strcmp(m, "buraikenb");
	s_rot0 = g->rot == 0;
	s_main_clock = !strcmp(m, "lwings") || !strcmp(m, "buraikenb") ? 6000000 : 3000000;
	uint32_t s1, s2, s3, s4 = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("soundcpu", &s_zrom_size);
	uint8_t *chars = rbx_region("gfx1", &s1), *bg1 = rbx_region("gfx2", &s2), *spr = rbx_region("gfx3", &s3);
	uint8_t *bg2 = s_trojan ? rbx_region("gfx4", &s4) : NULL;
	s_bg2rom = s_trojan ? rbx_region("gfx5", &s_bg2rom_size) : NULL;
	s_arom = s_trojan ? rbx_region("adpcmcpu", &s_arom_size) : NULL;
	if (!s_rom || s_rom_size < 0x20000 || !s_zrom || s_zrom_size < 0x8000 || !chars || !bg1 || !spr ||
	    (s_trojan && (!bg2 || !s_bg2rom || !s_arom))) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_base_chr = s_trojan ? 768 : 512;
	s_base_bg1 = s_trojan ? 256 : 0;
	s_base_spr = s_trojan ? 640 : 384;
	{	// charlayout, bg1_tilelayout (RGN_FRAC(1,4)), spritelayout / bg2_tilelayout (RGN_FRAC(1,2))
		static const uint32_t pc[2] = { 0, 4 }, xc[8] = { 0, 1, 2, 3, 8, 9, 10, 11 };
		static const uint32_t yc[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
		static const uint32_t xt[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 128, 129, 130, 131, 132, 133, 134, 135 };
		static const uint32_t xs[16] = { 0, 1, 2, 3, 8, 9, 10, 11, 256, 257, 258, 259, 264, 265, 266, 267 };
		static uint32_t yt[16], ys[16];
		for (int i = 0; i < 16; i++) {
			yt[i] = (uint32_t)i * 8;
			ys[i] = (uint32_t)i * 16;
		}
		const uint32_t q = s2 / 4 * 8, h = s3 / 2 * 8, h4 = s4 / 2 * 8;
		const uint32_t pt[4] = { 3 * q, 2 * q, q, 0 }, ps[4] = { h + 4, h, 4, 0 }, p2[4] = { h4, h4 + 4, 0, 4 };
		const rbx_gfx_layout lc = { 8, 8, 2, pc, xc, yc, 128 };
		const rbx_gfx_layout lt = { 16, 16, 4, pt, xt, yt, 256 }, ls = { 16, 16, 4, ps, xs, ys, 512 };
		const rbx_gfx_layout l2 = { 16, 16, 4, p2, xs, ys, 512 };
		if (rbx_gfx_decode(&s_gchar, &lc, chars, s1) || rbx_gfx_decode(&s_gbg1, &lt, bg1, s2 / 4) ||
		    rbx_gfx_decode(&s_gspr, &ls, spr, s3 / 2) || (s_trojan && rbx_gfx_decode(&s_gbg2, &l2, bg2, s4 / 2))) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			lw_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = 51;   // (SSG 0.20, FM 0.10)
	yc.gain[3] = 26;
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1) || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1)) {
		rbx_fail("%s: out of memory", g->set);
		lw_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 1024)) {
		rbx_fail("%s: out of memory", g->set);
		lw_exit();
		return -1;
	}
	if (s_trojan)
		msm5205_init(&s_msm, 384000, MSM5205_S96_4B, msm_vck);
	memset(s_mmap, 0, sizeof(s_mmap));
	memset(s_zmap, 0, sizeof(s_zmap));
	memset(s_amap, 0, sizeof(s_amap));
	for (int p = 0; p < 0x80; p++) {
		s_mmap[p] = s_rom + p * 256;
		s_zmap[p] = s_zrom + p * 256;
	}
	if (s_trojan)
		for (int p = 0; p < 0x100 && (uint32_t)p * 256 + 256 <= s_arom_size; p++)
			s_amap[p] = s_arom + p * 256;
	static const char *const names[5] = { "SERVICE", "P1", "P2", "DSWA", "DSWB" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	lw_reset();
	return 0;
}

static void lw_exit(void)
{
	rbx_video_wait_idle();
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gchar);
	rbx_gfx_free(&s_gbg1);
	rbx_gfx_free(&s_gspr);
	rbx_gfx_free(&s_gbg2);
	free(s_snap);
	s_snap = NULL;
	free(t);
	t = NULL;
}

static void lw_frame(void)
{
	static int32_t tmp[1024];
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		if ((line & 7) == 0) {   // (scanline, every 8 lines)
			if (line == 248 && t->nmi_mask)
				z80_gen_nmi(&s_main);
			if (line == 128) {
				s_main.int_data = 0xff;
				t->main_irq = 1;
			}
			if (line % 64 == 32 && !t->snd_reset) {
				s_snd.int_data = 0xff;
				t->snd_irq = 1;
			}
		}
		z80_run(&s_main, s_main.cyc + (unsigned long)rbx_line_cycles(&t->main_frac, s_main_clock, FRAME_MHZ, LINES), &t->main_irq, 1);
		const int cs = rbx_line_cycles(&t->snd_frac, SND_CLOCK, FRAME_MHZ, LINES);
		if (t->snd_reset)
			s_snd.cyc += (unsigned long)cs;
		else
			z80_run(&s_snd, s_snd.cyc + (unsigned long)cs, &t->snd_irq, 1);
		if (s_trojan) {
			if (t->adp_irq)
				s_adp.int_data = 0xff;
			z80_run(&s_adp, s_adp.cyc + (unsigned long)rbx_line_cycles(&t->adp_frac, SND_CLOCK, FRAME_MHZ, LINES), &t->adp_irq, 1);
		}
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&t->ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
		const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
		rbx_ym_advance(1, clocks, tmp, 1024);
		if (s_trojan) {   // (MSM5205 0.50)
			const uint32_t yr = rbx_ym_rate(0);
			for (int i = 0; i < n; i++) {
				t->msm_acc += s_msm.clock;
				const uint32_t k = t->msm_acc / yr;
				t->msm_acc -= k * yr;
				msm5205_advance(&s_msm, k);
				s_mix[s_mix_n + i] += msm5205_output(&s_msm) / 2;
			}
		}
		s_mix_n += n;
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	rbx_ym_async_take(1, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int z_state(void *ctx, const char *tag, z80 *z, int load, uint8_t (*rd)(void *, uint16_t),
                   void (*wr)(void *, uint16_t, uint8_t), const uint8_t **map, uint8_t (*in)(z80 *, uint8_t),
                   void (*out)(z80 *, uint8_t, uint8_t))
{
	z80 c = *z;
	const int rc = rbx_state_io(ctx, tag, &c, sizeof(c));
	if (!rc && load) {
		c.read_byte = rd;
		c.write_byte = wr;
		c.port_in = in;
		c.port_out = out;
		c.userdata = NULL;
		c.read_map = map;
		*z = c;
	}
	return rc;
}

static int lw_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "lw", t, (uint32_t)offsetof(__typeof__(*t), state_end));
	rc |= z_state(ctx, "lw_main", &s_main, load, main_read, main_write, s_mmap, z_in, z_out);
	rc |= z_state(ctx, "lw_snd", &s_snd, load, snd_read, snd_write, s_zmap, z_in, z_out);
	if (s_trojan) {
		rc |= z_state(ctx, "lw_adp", &s_adp, load, adp_read, adp_write, s_amap, adp_in, adp_out);
		rc |= rbx_state_io(ctx, "lw_msm", &s_msm, (uint32_t)offsetof(msm5205_t, vck));
	}
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	if (!rc && load)
		map_bank();
	return rc ? -1 : 0;
}

static int lw_save(void *ctx) { return lw_state(ctx, 0); }
static int lw_load(void *ctx) { return lw_state(ctx, 1); }
static int lw_supported(const char *machine)
{
	return !strcmp(machine, "lwings") || !strcmp(machine, "sectionz") || !strcmp(machine, "trojan") ||
	       !strcmp(machine, "buraikenb");
}

const rbx_driver rbx_drv_lwings = { "Capcom Legendary Wings", lw_init, lw_exit, lw_reset, lw_frame, lw_save, lw_load,
                                    lw_supported, lw_render, NULL };
