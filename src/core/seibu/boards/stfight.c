/*  Seibu Kaihatsu Empire City: 1931 / Street Fight: a Z80 (opcodes and data encrypted apart), a Z80 sound CPU with two
    YM2203s, a 68705P5 MCU (coins, the MSM5205 ADPCM sample addresses); the Street Fight video: two 2048x4096 ROM-mapped
    16x16 tilemaps, an 8x8 text layer and 16x16 sprites mixed through color lookup PROMs, 256 colors (xBRG_444 RAM).
    Ported from MAME 0.289 src/mame/seibu/stfight.cpp (BSD-3-Clause, copyright-holders: Mark McDougall) and
    src/mame/seibu/stfight_dev.cpp (BSD-3-Clause, copyright-holders: Mark McDougall, David Haywood).
    This file is part of ReBOX. GPL-2.0-or-later.

    The CPUs interleaved a scanline at a time. Not done: Cross Shooter (2 PCB stack, the Air Raid video), the bootleg
    without opcode encryption (empcityubl, no ROMs here), the coin counters.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/m6805/m68705.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/msm5205.h"
#include "../../video/decoband.h"
#include "../../video/gfxlayout.h"
#include "../machine/smix.h"
#include "../video/seibu_draw.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 60000
#define LINES 256
#define VIS_Y0 16
#define VIS_H 224
#define VIS_W 256
#define CPU_CLOCK 3000000
#define YM_CLOCK 1500000
#define MSM_CLOCK 384000

static uint8_t *s_rom, *s_zrom, *s_op, *s_mcurom;
static uint32_t s_rom_size, s_zrom_size;
static const uint8_t *s_adpcm, *s_fgmap, *s_bgmap;
static uint8_t s_clut[4][0x100];   // tx, fg, bg, spr
static rbx_gfx s_gtx, s_gfg, s_gbg[2], s_gspr;
static int s_port[6];
static z80 s_main, s_snd;
static m68705_t s_mcu;
static msm5205_t s_msm;
static smix_t *s_mix;
// The YM2203s' samples are made on the rbx_aux worker (rbx_ym_set_async, as goal92.c): a line notes where its slice is and
// how many samples each chip made; at the frame's end they are taken and stretched into the same slices
#define YM_BUF 8192
typedef struct { int16_t off, slice, cnt[2]; } ym_line_t;
static ym_line_t *s_ym_line;   // [LINES]
static int32_t *s_ym_buf;      // [YM_BUF]
static uint16_t *s_sprbm;   // the sprites' line buffer of the frame (VIS_W x VIS_H)
static uint32_t s_ym_rate;

static struct {
	uint8_t ram[0x1000], zram[0x800], pal[0x200], tx[0x800], vregs[0x10], spr[0x1000];
	uint8_t coin_state, fm_data, cpu_to_mcu_empty, cpu_to_mcu_data, port_a_out, port_c_out, vck2, adpcm_reset;
	uint8_t nmi_line, sprite_bank_reg, tx_en, bg_en, fg_en;
	uint16_t adpcm_data_offs, sprite_base;
	int main_irq, snd_irq;
	int rst08_line;   // the line the RST 08 timer fires at (-1 none)
	uint64_t main_frac, snd_frac, mcu_frac, ym_frac, msm_frac;
	uint8_t state_end;
} t;

static struct {
	uint8_t tx[0x800], vregs[0x10], spr[0x1000];
	uint16_t sprite_base;
	uint8_t tx_en, bg_en, fg_en;
	uint32_t pal[256];
} *s_snap;

// ---- sound ----
static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_zrom[a];
	if (a == 0xc000 || a == 0xc001) return rbx_ym_read(0, a & 1);
	if (a == 0xc800 || a == 0xc801) return rbx_ym_read(1, a & 1);
	if (a == 0xf000) {   // fm_r: the command acknowledged
		const uint8_t v = t.fm_data;
		t.fm_data &= 0x7f;
		return v;
	}
	if (a >= 0xf800) return t.zram[a - 0xf800];
	return 0xff;
}

static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a == 0xc000 || a == 0xc001) rbx_ym_write(0, a & 1, v);
	else if (a == 0xc800 || a == 0xc801) rbx_ym_write(1, a & 1, v);
	else if (a >= 0xf800) t.zram[a - 0xf800] = v;
}

// ---- the main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_rom[a];
	if (a < 0xc000) return s_rom[0x10000 + (a - 0x8000)];   // (mainbank, entry 0)
	if (a < 0xc200) return t.pal[a - 0xc000];
	if (a >= 0xc200 && a <= 0xc204) return (uint8_t)rbx_port_read(s_port[a - 0xc200]);
	if (a == 0xc205) return t.coin_state;
	if (a >= 0xd000 && a < 0xd800) return t.tx[a - 0xd000];
	if (a >= 0xd800 && a < 0xd809) return t.vregs[a - 0xd800];
	if (a >= 0xe000 && a < 0xf000) return t.ram[a - 0xe000];
	if (a >= 0xf000) return t.spr[a - 0xf000];
	return 0xff;
}

static void vh_latch_w(int off, uint8_t v)
{
	t.vregs[off] = v;
	if (off == 7) {
		t.tx_en = (v & 0x80) != 0;
		t.bg_en = (v & 0x20) != 0;
		t.fg_en = (v & 0x10) != 0;
	}
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xc200) { t.pal[a - 0xc000] = v; return; }
	if (a >= 0xd000 && a < 0xd800) { t.tx[a - 0xd000] = v; return; }
	if (a >= 0xd800 && a < 0xd809) { vh_latch_w(a - 0xd800, v); return; }
	if (a >= 0xe000 && a < 0xf000) { t.ram[a - 0xe000] = v; return; }
	if (a >= 0xf000) { t.spr[a - 0xf000] = v; return; }
	switch (a) {
	case 0xc500: t.fm_data = (uint8_t)(0x80 | v); break;
	case 0xc600: t.cpu_to_mcu_data = v & 0x0f; t.cpu_to_mcu_empty = 0; break;
	case 0xc700:   // coin_w: the coin signals acknowledged (active low)
		if (!(v & 1)) t.coin_state |= 1;
		if (!(v & 2)) t.coin_state |= 2;
		break;
	case 0xc807: t.sprite_base = (uint16_t)(((v & 0x04) << 7) | ((v & 0x01) << 8)); break;
	default: break;
	}
}

static uint8_t z_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

// ---- the MCU ----
static uint8_t mcu_port_r(void *u, int n, uint8_t mask)
{
	(void)u; (void)mask;
	if (n == 1)
		return (uint8_t)(((rbx_port_read(s_port[5]) & 3) << 6) | (t.cpu_to_mcu_empty ? 0x10 : 0) | (t.cpu_to_mcu_data & 0x0f));
	return 0xff;
}

static void mcu_port_w(void *u, int n, uint8_t data, uint8_t mask)
{
	(void)u; (void)mask;
	switch (n) {
	case 0: t.port_a_out = data; break;
	case 1: if (!(data & 0x20)) t.cpu_to_mcu_empty = 1; break;
	case 2:
		if ((t.port_c_out & 1) && !(data & 1)) t.coin_state &= ~1;
		if ((t.port_c_out & 2) && !(data & 2)) t.coin_state &= ~2;
		t.adpcm_reset = (data >> 2) & 1;
		if (!t.adpcm_reset && (t.port_c_out & 4))
			t.adpcm_data_offs = (uint16_t)(t.port_a_out << 9);
		msm5205_reset_w(&s_msm, t.adpcm_reset);
		{   // the main CPU's NMI (handshake error, stuck coin): asserted while bit 3 is low
			const uint8_t line = !(data & 8);
			if (line && !t.nmi_line)
				z80_gen_nmi(&s_main);
			t.nmi_line = line;
		}
		t.port_c_out = data;
		break;
	default: break;
	}
}

static void adpcm_int(void)
{
	m68705_set_input(&s_mcu, M68705_IRQ_LINE, t.vck2 ? 1 : 0);
	t.vck2 = !t.vck2;
	if (!t.adpcm_reset) {
		uint8_t d = s_adpcm[(t.adpcm_data_offs >> 1) & 0x7fff];
		if (!(t.adpcm_data_offs & 1))
			d >>= 4;
		t.adpcm_data_offs++;
		msm5205_data_w(&s_msm, d & 0x0f);
	}
}

// ---- the drawing ----
static inline uint32_t fg_scan(int col, int row)
{
	return (uint32_t)((col & 0x0f) + ((row & 0x0f) << 4) + ((col & 0x70) << 4) + ((row & 0xf0) << 7));
}

static inline uint32_t bg_scan(int col, int row)
{
	return (uint32_t)(((col & 0x0e) >> 1) + ((row & 0x0f) << 3) + ((col & 0x70) << 3) + ((row & 0x80) << 3) +
	                  ((row & 0x10) << 7) + ((col & 0x01) << 12) + ((row & 0x60) << 8));
}

// The frame is drawn in bands of lines (render_help: the threads helping the render thread take bands too): a band
// draws its own lines of the sprite bitmap (each sprite clipped to the band), composes them a tile row at a time
// (mix_txlayer's pens through the CLUTs, made into tables at the init) and writes them to the frontend's frame.
#define BANDS 4
static uint16_t s_lbg[128], s_lfg[128], s_lspr[256], s_ltx[64];   // (the CLUTs' pens; s_ltx: 0xffff transparent)
static uint8_t s_sprline[VIS_H];   // (the sprites of a line: bit 0 a pixel with pen bit 0x100 clear, bit 1 set)
static uint32_t s_outlut[256];     // (the palette in the frontend's colours)

static void luts_init(void)
{
	for (int i = 0; i < 128; i++) {
		s_lbg[i] = (uint16_t)(s_clut[2][i] & 0x3f);
		s_lfg[i] = (uint16_t)((s_clut[1][i] & 0x3f) + 0x40);
	}
	for (int i = 0; i < 256; i++)
		s_lspr[i] = (uint16_t)((s_clut[3][i] & 0x3f) + 0x80);
	for (int i = 0; i < 64; i++) {
		const uint8_t real = s_clut[0][i];
		s_ltx[i] = (real & 0x0f) != 0x0f ? (uint16_t)((real & 0x3f) + 0xc0) : 0xffff;
	}
}

// the sprites (drawgfx transpen 0x0f) into their bitmap's rows [r0, r1)
static void sprites_band(int r0, int r1)
{
	for (int i = r0 * VIS_W; i < r1 * VIS_W; i++)
		s_sprbm[i] = 0xffff;
	memset(s_sprline + r0, 0, (size_t)(r1 - r0));
	for (int offs = 4096 - 32; offs >= 0; offs -= 32) {
		const uint8_t *s = &s_snap->spr[offs];
		const int attr = s[1];
		const int color = (attr & 0x0f) | ((attr & 0x20) >> 1);
		const int sy = s[2];
		int sx = s[3];
		if (sy <= 0)
			continue;
		if (sx >= 0xf0 && (attr & 0x80))
			sx -= 0x100;
		const int ry = sy - VIS_Y0;
		if (ry >= r1 || ry + 16 <= r0 || sx <= -16 || sx >= VIS_W)
			continue;
		const uint8_t *src = sd_elem(&s_gspr, (uint32_t)(s_snap->sprite_base + s[0]));
		const uint16_t cb = (uint16_t)(color * 16);
		const uint8_t flag = (cb & 0x100) ? 2 : 1;
		const int flipx = (attr & 0x10) != 0;
		const int j0 = ry < r0 ? r0 - ry : 0, j1 = ry + 16 > r1 ? r1 - ry : 16;
		const int i0 = sx < 0 ? -sx : 0, i1 = sx > VIS_W - 16 ? VIS_W - sx : 16;
		for (int j = j0; j < j1; j++) {
			const uint8_t *row = src + j * 16;
			uint16_t *d = s_sprbm + (ry + j) * VIS_W + sx;
			int any = 0;
			for (int i = i0; i < i1; i++) {
				const uint8_t c = row[flipx ? 15 - i : i];
				if (c != 0x0f) {
					d[i] = (uint16_t)(cb + c);
					any = 1;
				}
			}
			if (any)
				s_sprline[ry + j] |= flag;
		}
	}
}

// the sprite pixels with (pen & 0x100) == cond over the line
static inline void sprite_mix(uint16_t *d, const uint16_t *sp, int cond)
{
	for (int x = 0; x < VIS_W; x++) {
		const uint16_t v = sp[x];
		if (v != 0xffff && (v & 0x100) == cond && (v & 0xf) != 0xf)
			d[x] = s_lspr[v & 0xff];
	}
}

static void sf_band(int band)
{
	const int r0 = VIS_H * band / BANDS, r1 = VIS_H * (band + 1) / BANDS;
	const uint8_t *vr = s_snap->vregs;
	const int fgsx = vr[1] << 8 | vr[0], fgsy = vr[3] << 8 | vr[2], bgsx = vr[5] << 8 | vr[4], bgsy = vr[8] << 8 | vr[6];
	const int sprites = (vr[7] & 0x40) != 0;
	if (sprites)
		sprites_band(r0, r1);
	for (int r = r0; r < r1; r++) {
		const int y = r + VIS_Y0;
		uint16_t *d = rbx_video.bitmap + r * rbx_video.pitch;
		const uint16_t *sp = s_sprbm + r * VIS_W;
		const int sl = sprites ? s_sprline[r] : 0;
		memset(d, 0, VIS_W * sizeof(uint16_t));
		if (s_snap->bg_en) {
			const int py = (y + bgsy) & 4095;
			const size_t roff = (size_t)(py & 15) * 16;
			int px = bgsx & 2047;
			for (int x = 0; x < VIS_W;) {
				const int fx = px & 15;
				int n = 16 - fx;
				if (n > VIS_W - x)
					n = VIS_W - x;
				const uint32_t idx = bg_scan(px >> 4, py >> 4);
				const uint8_t attr = s_bgmap[0x8000 + idx];
				const uint32_t code = (uint32_t)((attr & 0x80) << 1) + s_bgmap[idx];
				const uint8_t *row = sd_elem(&s_gbg[(attr & 0x20) >> 5], code) + roff + fx;
				const uint16_t *lut = s_lbg + (attr & 7) * 16;
				for (int i = 0; i < n; i++)
					if (row[i] != 0x0f)
						d[x + i] = lut[row[i]];
				x += n;
				px = (px + n) & 2047;
			}
		}
		if (sl & 2)
			sprite_mix(d, sp, 0x100);
		if (s_snap->fg_en) {
			const int py = (y + fgsy) & 4095;
			const size_t roff = (size_t)(py & 15) * 16;
			int px = fgsx & 2047;
			for (int x = 0; x < VIS_W;) {
				const int fx = px & 15;
				int n = 16 - fx;
				if (n > VIS_W - x)
					n = VIS_W - x;
				const uint32_t idx = fg_scan(px >> 4, py >> 4);
				const uint8_t attr = s_fgmap[0x8000 + idx];
				const uint32_t code = (uint32_t)(((attr & 0x80) << 2) | ((attr & 0x20) << 3)) + s_fgmap[idx];
				const uint8_t *row = sd_elem(&s_gfg, code) + roff + fx;
				const uint16_t *lut = s_lfg + (attr & 7) * 16;
				for (int i = 0; i < n; i++)
					if (row[i] != 0x0f)
						d[x + i] = lut[row[i]];
				x += n;
				px = (px + n) & 2047;
			}
		}
		if (sl & 1)
			sprite_mix(d, sp, 0x000);
		if (s_snap->tx_en)
			for (int x = 0; x < VIS_W; x += 8) {
				const int idx = (y >> 3) * 32 + (x >> 3);
				const uint8_t attr = s_snap->tx[idx + 0x400];
				const uint32_t code = (uint32_t)(s_snap->tx[idx] + ((attr & 0x80) << 1));
				const int fx = (attr >> 5) & 1, fy = (attr >> 6) & 1;
				const uint8_t *row = sd_elem(&s_gtx, code) + (fy ? 7 - (y & 7) : y & 7) * 8;
				const uint16_t *lut = s_ltx + (attr & 0x0f) * 4;
				for (int i = 0; i < 8; i++) {
					const uint16_t v = lut[row[fx ? 7 - i : i]];
					if (v != 0xffff)
						d[x + i] = v;
				}
			}
		if (rbx_video.out) {
			uint8_t *o = (uint8_t *)rbx_video.out + (size_t)r * rbx_video.out_pitch;
			if (rbx_video.out_bpp == 2)
				for (int x = 0; x < VIS_W; x++)
					((uint16_t *)o)[x] = (uint16_t)s_outlut[d[x] & 0xff];
			else
				for (int x = 0; x < VIS_W; x++)
					((uint32_t *)o)[x] = s_outlut[d[x] & 0xff];
		}
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, sf_band };
static int sf_help(int helper) { return decoband_help(&s_bands, helper); }

static void sf_render(void)
{
	memcpy(rbx_video.palette, s_snap->pal, sizeof(s_snap->pal));
	if (rbx_video.out)   // (the frame written here: the core's conversion of the bitmap, the same colours)
		for (int i = 0; i < 256; i++) {
			const uint32_t c = s_snap->pal[i];
			s_outlut[i] = rbx_video.out_color((int)(c >> 16) & 0xff, (int)(c >> 8) & 0xff, (int)c & 0xff);
		}
	decoband_run(&s_bands);
	if (rbx_video.out)
		rbx_video.out_done = 1;
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->tx, t.tx, sizeof(t.tx));
	memcpy(s_snap->vregs, t.vregs, sizeof(t.vregs));
	memcpy(s_snap->spr, t.spr, sizeof(t.spr));
	s_snap->sprite_base = t.sprite_base;
	s_snap->tx_en = t.tx_en;
	s_snap->bg_en = t.bg_en;
	s_snap->fg_en = t.fg_en;
	for (int i = 0; i < 256; i++) {   // xBRG_444: B 11-8, R 7-4, G 3-0
		const uint16_t w = (uint16_t)(t.pal[i] | t.pal[0x100 + i] << 8);
		s_snap->pal[i] = (uint32_t)((w >> 4) & 0xf) * 0x110000u | (uint32_t)(w & 0xf) * 0x1100u | (uint32_t)((w >> 8) & 0xf) * 0x11u;
	}
	rbx_video_frame_done();
}

// ---- init / frame ----
static void z_setup(z80 *z, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t))
{
	z80_init(z);
	z->read_byte = rd;
	z->write_byte = wr;
	z->port_in = z_in;
	z->port_out = z_out;
}

static void sf_reset(void)
{
	rbx_video_wait_idle();
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	t.cpu_to_mcu_empty = 1;
	t.cpu_to_mcu_data = 0x0f;
	t.port_a_out = 0xff;
	t.port_c_out = 0xff;
	t.adpcm_reset = 1;
	t.coin_state = 3;
	t.tx_en = t.bg_en = t.fg_en = 1;
	t.rst08_line = -1;
	z_setup(&s_main, main_read, main_write);
	s_main.op_rom = s_op;
	z_setup(&s_snd, snd_read, snd_write);
	m68705_reset(&s_mcu);
	msm5205_reset(&s_msm);
	msm5205_reset_w(&s_msm, 1);
	for (int i = 0; i < 2; i++) {
		rbx_ym_reset(i);
		rbx_ym_write(i, 0, 0x2f);   // init_ymhack: the prescaler FM 1/2, PSG 1/1
	}
	s_mix->n = 0;
	s_mix->frac = 0;
}

static void sf_exit(void);

static const uint8_t *nib_region(const char *name, char *buf, uint8_t *out)
{
	uint32_t hs = 0, ls = 0, s = 0;
	strcpy(buf, name);
	strcat(buf, ":nib_high");
	const uint8_t *hi = rbx_region_opt(buf, &hs);
	strcpy(buf, name);
	strcat(buf, ":nib_low");
	const uint8_t *lo = rbx_region_opt(buf, &ls);
	if (hi && lo && hs >= 0x100 && ls >= 0x100) {
		for (int i = 0; i < 0x100; i++)
			out[i] = (uint8_t)((hi[i] & 0x0f) << 4 | (lo[i] & 0x0f));
		return out;
	}
	const uint8_t *r = rbx_region_opt(name, &s);
	if (!r || s < 0x100)
		return NULL;
	memcpy(out, r, 0x100);
	return out;
}

static int sf_init(const rbx_game *g)
{
	uint32_t txs, fgs, bgs, sps, fms, bms, as, ms;
	char nb[64];
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_mcurom = rbx_region("mcu", &ms);
	const uint8_t *tx = rbx_region("stfight_vid:tx_gfx", &txs), *fg = rbx_region("stfight_vid:fg_gfx", &fgs),
	              *bg = rbx_region("stfight_vid:bg_gfx", &bgs), *sp = rbx_region("stfight_vid:spr_gfx", &sps);
	s_fgmap = rbx_region("stfight_vid:fg_map", &fms);
	s_bgmap = rbx_region("stfight_vid:bg_map", &bms);
	s_adpcm = rbx_region("adpcm", &as);
	if (!s_rom || s_rom_size < 0x14000 || !s_zrom || s_zrom_size < 0x8000 || !s_mcurom || ms < 0x800 || !tx || !fg || !bg ||
	    !sp || fgs < 0x20000 || bgs < 0x20000 || sps < 0x20000 || !s_fgmap || fms < 0x10000 || !s_bgmap || bms < 0x10000 ||
	    !s_adpcm || as < 0x8000 || !nib_region("stfight_vid:tx_clut", nb, s_clut[0]) ||
	    !nib_region("stfight_vid:fg_clut", nb, s_clut[1]) || !nib_region("stfight_vid:bg_clut", nb, s_clut[2]) ||
	    !nib_region("stfight_vid:spr_clut", nb, s_clut[3])) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	luts_init();
	{   // charlayout, fglayout, bglayout (two, 0x20 apart), spritelayout
		static const uint32_t cp[2] = { 4, 0 }, cx[8] = { 0, 1, 2, 3, 8, 9, 10, 11 };
		static const uint32_t cy[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
		static const uint32_t fp[4] = { 64 * 1024 * 8 + 0, 64 * 1024 * 8 + 4, 0, 4 };
		static const uint32_t bp[4] = { 64 * 1024 * 8 + 4, 64 * 1024 * 8 + 0, 4, 0 };
		static const uint32_t fx[16] = { 0, 1, 2, 3, 8, 9, 10, 11, 256, 257, 258, 259, 264, 265, 266, 267 };
		static const uint32_t bx[16] = { 0, 1, 2, 3, 8, 9, 10, 11, 512, 513, 514, 515, 520, 521, 522, 523 };
		static const uint32_t fy[16] = { 0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240 };
		const rbx_gfx_layout lc = { 8, 8, 2, cp, cx, cy, 128 };
		const rbx_gfx_layout lf = { 16, 16, 4, fp, fx, fy, 512 };
		const rbx_gfx_layout lb = { 16, 16, 4, bp, bx, fy, 1024 };
		if (rbx_gfx_decode(&s_gtx, &lc, tx, 512 * 16) || rbx_gfx_decode(&s_gfg, &lf, fg, 1024 * 64) ||
		    rbx_gfx_decode(&s_gbg[0], &lb, bg, 512 * 128) || rbx_gfx_decode(&s_gbg[1], &lb, bg + 0x20, 512 * 128) ||
		    rbx_gfx_decode(&s_gspr, &lf, sp, 1024 * 64)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			sf_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	s_mix = calloc(1, sizeof(*s_mix));
	s_ym_line = calloc(LINES, sizeof(*s_ym_line));
	s_ym_buf = calloc(YM_BUF, sizeof(*s_ym_buf));
	s_sprbm = calloc(VIS_W * VIS_H, sizeof(uint16_t));
	s_op = calloc(1, 0x10000);
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = 38;   // (0.15)
	yc.gain[3] = 26;                             // (0.10)
	if (!s_snap || !s_mix || !s_ym_line || !s_ym_buf || !s_sprbm || !s_op || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1) ||
	    rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1)) {
		rbx_fail("%s: out of memory", g->set);
		sf_exit();
		return -1;
	}
	s_ym_rate = rbx_ym_rate(0);
	if (rbx_audio_setup(48000) || rbx_video_setup(VIS_W, VIS_H, 256)) {
		rbx_fail("%s: out of memory", g->set);
		sf_exit();
		return -1;
	}
	smix_init(s_mix, 48000, FRAME_MHZ, LINES);
	{   // init_stfight: opcodes and operands decrypted apart
		for (uint32_t a = 0; a < 0x8000; a++) {
			const uint8_t src = s_rom[a];
			s_op[a] = (uint8_t)((src & 0xa6) | ((((src << 2) ^ src) << 3) & 0x40) | (~((src ^ (a >> 1)) >> 2) & 0x10) |
			                    (~(((src << 1) ^ a) << 2) & 0x08) | (((src ^ (src >> 3)) >> 1) & 0x01));
			s_rom[a] = (uint8_t)((src & 0xa6) | (~((src ^ (src << 1)) << 5) & 0x40) | (((src ^ (a << 3)) << 1) & 0x10) |
			                     (((src ^ a) >> 1) & 0x08) | (~((src >> 6) ^ a) & 0x01));
		}
		memcpy(s_op + 0x8000, s_rom + 0x10000, 0x4000);   // (the bank: fetched as data)
	}
	m68705_init(&s_mcu, M68705_P5, s_mcurom, 0x800);
	s_mcu.port_r[1] = mcu_port_r;
	s_mcu.port_w[0] = s_mcu.port_w[1] = s_mcu.port_w[2] = mcu_port_w;
	msm5205_init(&s_msm, MSM_CLOCK, MSM5205_S48_4B, adpcm_int);
	static const char *const names[6] = { "P1", "P2", "START", "DSW0", "DSW1", "COIN" };
	for (int i = 0; i < 6; i++)
		s_port[i] = rbx_port_find(names[i]);
	sf_reset();
	return 0;
}

static void sf_exit(void)
{
	rbx_video_wait_idle();
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	rbx_gfx_free(&s_gtx);
	rbx_gfx_free(&s_gfg);
	rbx_gfx_free(&s_gbg[0]);
	rbx_gfx_free(&s_gbg[1]);
	rbx_gfx_free(&s_gspr);
	free(s_snap);
	s_snap = NULL;
	free(s_mix);
	s_mix = NULL;
	free(s_ym_line);
	s_ym_line = NULL;
	free(s_ym_buf);
	s_ym_buf = NULL;
	free(s_sprbm);
	s_sprbm = NULL;
	free(s_op);
	s_op = NULL;
}

static void sf_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0 + VIS_H) {   // vb_interrupt: RST 10, RST 08 1/120 s later
			vblank_capture();
			s_main.int_data = 0xcf;
			t.main_irq = 1;
			t.rst08_line = (line + LINES / 2) % LINES;
		} else if (line == t.rst08_line) {
			s_main.int_data = 0xd7;
			t.main_irq = 1;
			t.rst08_line = -1;
		}
		if ((line & 127) == 0) {   // (the sound CPU's 120 Hz IRQ 0)
			s_snd.int_data = 0xff;
			t.snd_irq = 1;
		}
		const int c = rbx_line_cycles(&t.main_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		z80_run(&s_main, s_main.cyc + (unsigned long)c, &t.main_irq, 1);
		const int cs = rbx_line_cycles(&t.snd_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		z80_run(&s_snd, s_snd.cyc + (unsigned long)cs, &t.snd_irq, 1);
		const int cm = rbx_line_cycles(&t.mcu_frac, CPU_CLOCK / 4, FRAME_MHZ, LINES);
		m68705_execute(&s_mcu, cm);
		smix_begin(s_mix);
		const uint32_t yc = (uint32_t)rbx_line_cycles(&t.ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
		ym_line_t *yl = &s_ym_line[line];
		yl->off = (int16_t)s_mix->n;
		yl->slice = (int16_t)s_mix->slice;
		for (int i = 0; i < 2; i++)   // (the samples on the worker: nothing written to the buffer)
			yl->cnt[i] = (int16_t)rbx_ym_advance(i, yc, s_mix->tmp, SMIX_TMP);
		msm5205_advance(&s_msm, (uint32_t)rbx_line_cycles(&t.msm_frac, MSM_CLOCK, FRAME_MHZ, LINES));
		smix_level(s_mix, msm5205_output(&s_msm) / 2);   // (0.50)
		smix_end(s_mix);
	}
	for (int i = 0; i < 2; i++) {   // the frame's YM samples into the lines' slices
		int total = 0;
		for (int line = 0; line < LINES; line++)
			total += s_ym_line[line].cnt[i];
		if (total > YM_BUF)
			total = YM_BUF;
		memset(s_ym_buf, 0, sizeof(int32_t) * (size_t)total);
		rbx_ym_async_take(i, s_ym_buf, total);
		int pos = 0;
		for (int line = 0; line < LINES; line++) {
			const ym_line_t *yl = &s_ym_line[line];
			const int cnt = pos + yl->cnt[i] <= total ? yl->cnt[i] : total - pos;
			smix_stretch(s_mix->out + yl->off, yl->slice, s_ym_buf + pos, cnt);
			pos += cnt;
		}
	}
	smix_flush(s_mix);
}

static int z_state(void *ctx, const char *tag, z80 *z, int load, uint8_t (*rd)(void *, uint16_t),
                   void (*wr)(void *, uint16_t, uint8_t), const uint8_t *op)
{
	z80 c = *z;
	const int rc = rbx_state_io(ctx, tag, &c, sizeof(c));
	if (!rc && load) {
		c.read_byte = rd;
		c.write_byte = wr;
		c.port_in = z_in;
		c.port_out = z_out;
		c.userdata = NULL;
		c.read_map = NULL;
		c.op_rom = op;
		*z = c;
	}
	return rc;
}

static int sf_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "stfight", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= z_state(ctx, "stfight_main", &s_main, load, main_read, main_write, s_op);
	rc |= z_state(ctx, "stfight_snd", &s_snd, load, snd_read, snd_write, NULL);
	rc |= rbx_state_io(ctx, "stfight_mcu", &s_mcu.st, sizeof(s_mcu.st));
	rc |= rbx_state_io(ctx, "stfight_msm", &s_msm, (uint32_t)offsetof(msm5205_t, vck));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	return rc ? -1 : 0;
}

static int sf_save(void *ctx) { return sf_state(ctx, 0); }
static int sf_load(void *ctx) { return sf_state(ctx, 1); }
static int sf_supported(const char *m) { return !strcmp(m, "stfight"); }

const rbx_driver rbx_drv_stfight = { "Seibu Street Fight", sf_init, sf_exit, sf_reset, sf_frame, sf_save, sf_load,
                                     sf_supported, sf_render, sf_help };
