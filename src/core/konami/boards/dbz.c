/*  Banpresto / Konami Dragon Ball Z and Dragon Ball Z 2. Ported from MAME src/mame/konami/dbz.cpp (BSD-3-Clause,
    copyright-holders: R. Belmont, Pierpaolo Prazzoli, David Haywood) and the devices it uses (k053936.cpp: David
    Haywood; tilemap.cpp's draw_roz). A 68000 at 16 MHz, the K056832 tilemaps (4 bpp), two K053936 ROZ layers of 16x16
    tiles, the K053246/K053247 sprites, the K053251 priorities and colour bases (drawn by konamigx_v.c's pre-GX mode,
    MAME's bitmap_ind16); the sound: a Z80 at 4 MHz, a YM2151 (its IRQ), an OKI M6295, a latch.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../video/k056832.h"
#include "../video/k055673.h"
#include "konamigx.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define MAIN_CLOCK 16000000u
#define Z80_CLOCK 4000000u
#define YM_CLOCK 4000000u
#define OKI_CLOCK 1056000u
#define SND_RATE 48000
#define LINES 320          // (set_size(64*8, 40*8), 55 Hz, the visible area 384 x 256)
#define FPS 55
#define VIS_W 384
#define VIS_H 256

static uint8_t *s_rom, *s_zrom, *s_sprrom;
static uint32_t s_rom_size, s_zrom_size, s_sprrom_size;
static uint8_t *s_roz_gfx[2];   // (roztiles0 / 1 decoded: gfx_16x16x4_packed_msb, a byte a pixel)
static uint32_t s_roz_n[2];
static z80 s_z;
static const uint8_t *s_zmap[256];
static okim6295_t s_oki;
static int s_port_p12, s_port_sys, s_port_dsw2;
static int s_dbz2;

static struct {
	uint16_t bg_vram[2][0x1000];   // (500000: bg 1, 508000: bg 0)
	uint16_t k936_ctrl[2][0x10];
	uint16_t k936_line[2][0x2000];
	uint16_t extra[0x1800];        // (4a1000-4a3fff)
	uint8_t k53251[16];
	uint8_t ccu[16];
	uint8_t zram[0x4000];
	uint8_t latch, z80_irq;
	uint16_t control;
	int64_t cycles_done;
	uint64_t cyc_frac, z_frac, z_end, ym_frac, oki_frac, snd_frac;
	int32_t oki_hold;
	uint32_t frame;
	uint8_t state_end;
} d;

// ---- the drawing's snapshot and state (screen_update) ----
static struct { uint8_t k53251[16]; uint16_t bg_vram[2][0x1000], ctrl[2][0x10], line[2][0x2000]; } s_vs;
static int s_layer_colorbase[6], s_layerpri[5], s_sprite_colorbase;

// (the sound board's queue, see job)
#define Q_MAX 2048
enum { Q_ADV, Q_LATCH, Q_NMI };
typedef struct { uint16_t kind, arg; uint32_t v; } q_ev_t;
static q_ev_t s_q[2][Q_MAX];
static int s_qn[2], s_qcur;
static int32_t s_out_l[2048], s_out_r[2048];
static int s_out_n, s_out_pos;
static volatile int s_job_frame;

static void q_put(int kind, uint32_t v)
{
	const int c = s_qcur;
	if (s_qn[c] < Q_MAX) {
		s_q[c][s_qn[c]].kind = (uint16_t)kind;
		s_q[c][s_qn[c]].v = v;
		s_qn[c]++;
	}
}


static int k53251_palette_index(const uint8_t *r, int ci)   // (k053251_device::reset_indexes)
{
	switch (ci) {
	case 0: return 32 * (r[9] & 3);
	case 1: return 32 * (r[9] >> 2 & 3);
	case 2: return 32 * (r[9] >> 4 & 3);
	case 3: return 16 * (r[10] & 7);
	default: return 16 * (r[10] >> 3 & 7);
	}
}

static void tile_cb(int layer, int *code, int *color, int *flip, int *category)
{
	(void)code;
	(void)flip;
	(void)category;
	*color = (s_layer_colorbase[layer] << 1) + ((*color & 0x3c) >> 2);
}

static void sprite_cb(int *code, int *color, int *primask)
{
	(void)code;
	const int pri = (*color & 0x3c0) >> 5;
	if (pri <= s_layerpri[3])
		*primask = 0xff00;
	else if (pri <= s_layerpri[2])
		*primask = 0xfff0;
	else if (pri <= s_layerpri[1])
		*primask = 0xfffc;
	else
		*primask = 0xfffe;
	*color = (s_sprite_colorbase << 1) + (*color & 0x1f);
}

static void capture(void)
{
	memcpy(s_vs.k53251, d.k53251, sizeof(s_vs.k53251));
	memcpy(s_vs.bg_vram, d.bg_vram, sizeof(s_vs.bg_vram));
	memcpy(s_vs.ctrl, d.k936_ctrl, sizeof(s_vs.ctrl));
	memcpy(s_vs.line, d.k936_line, sizeof(s_vs.line));
}

// ---- a K053936 drawing its 64 x 32 tilemap of 16 x 16 tiles (k053936_device::zoom_draw, tilemap_t::draw_roz) ----
// which: the bg tilemap (0: roztiles0 at 508000, 1: roztiles1 at 500000), chip: its K053936
static void roz_rows(int chip, int which, int opaque, int priority, int r0, int r1)   // (rows r0..r1 of the frame)
{
	const int xoff = -46, yoff = -16;   // (set_offsets(-46, -16), set_wrap(1))
	const uint16_t *ctrl = s_vs.ctrl[chip];
	const uint16_t *vram = s_vs.bg_vram[which];
	const uint8_t *gfx = s_roz_gfx[which];
	const uint32_t ntiles = s_roz_n[which];
	const int cb = s_layer_colorbase[4 + which] << 1;
	uint16_t *frame = gx_pre_frame16();
	uint8_t *pri = gx_pre_pri();
	const uint8_t pc = (uint8_t)priority;
	// (one screen line from x0 to x1 with the start and increments in 16.16, the map wrapped: 1024 x 512)
#define ROZ_LINE(Y, X0, X1, SX, SY, IXX, IXY) do { \
		uint32_t cx = (SX), cy = (SY); \
		uint16_t *o = frame + (Y) * VIS_W; \
		uint8_t *pr = pri + (Y) * VIS_W; \
		if ((IXX) == 0x10000 && (IXY) == 0 && !(cx & 0xffff)) { /* (a pixel a pixel along a row: by tiles) */ \
			const uint32_t py = (cy >> 16) & 511; \
			const uint16_t *vrow = vram + (py >> 4) * 128; \
			uint32_t px = (cx >> 16) & 1023; \
			for (int x = (X0); x <= (X1);) { \
				const uint16_t attr = vrow[(px >> 4) * 2], tile = vrow[(px >> 4) * 2 + 1] & 0x7fff; \
				const uint32_t t = ntiles ? (tile < ntiles ? tile : tile % ntiles) : 0; \
				const uint8_t *row = gfx ? gfx + t * 256 + (py & 15) * 16 : NULL; \
				const uint32_t base = (uint32_t)(((attr & 0x0f) + cb) * 16); \
				int k = 16 - (int)(px & 15); \
				if (k > (X1) - x + 1) \
					k = (X1) - x + 1; \
				for (int i = 0; i < k; i++, x++, px++) { \
					const uint32_t tx = (attr & 0x80) ? 15 - (px & 15) : (px & 15); \
					const int p = row ? row[tx] : 0; \
					if (p || opaque) { \
						o[x] = (uint16_t)((base + (uint32_t)p) & 0x1fff); \
						pr[x] |= pc; \
					} \
				} \
				px &= 1023; \
			} \
			break; \
		} \
		for (int x = (X0); x <= (X1); x++, cx += (uint32_t)(IXX), cy += (uint32_t)(IXY)) { \
			const uint32_t px = (cx >> 16) & 1023, py = (cy >> 16) & 511; \
			const int ti = (int)((py >> 4) * 64 + (px >> 4)); \
			const uint16_t attr = vram[ti * 2], tile = vram[ti * 2 + 1] & 0x7fff; \
			const uint32_t tx = (attr & 0x80) ? 15 - (px & 15) : (px & 15); \
			const uint32_t t = ntiles ? (tile < ntiles ? tile : tile % ntiles) : 0; \
			const int p = gfx ? gfx[t * 256 + (py & 15) * 16 + tx] : 0; \
			if (p || opaque) { \
				o[x] = (uint16_t)((((attr & 0x0f) + cb) * 16 + p) & 0x1fff); \
				pr[x] |= pc; \
			} \
		} \
	} while (0)
	if (ctrl[0x07] & 0x0040) {   // (per-line: lineaddr from the line RAM)
		for (int y = r0; y <= r1; y++) {
			const uint16_t *la = s_vs.line[chip] + 4 * ((y - yoff) & 0x1ff);
			uint32_t startx = (uint32_t)(256 * (int16_t)(la[0] + ctrl[0x00]));
			uint32_t starty = (uint32_t)(256 * (int16_t)(la[1] + ctrl[0x01]));
			int incxx = (int16_t)la[2], incxy = (int16_t)la[3];
			if (ctrl[0x06] & 0x8000)
				incxx *= 256;
			if (ctrl[0x06] & 0x0080)
				incxy *= 256;
			startx -= (uint32_t)(xoff * incxx);
			starty -= (uint32_t)(xoff * incxy);
			startx <<= 5;
			starty <<= 5;
			incxx <<= 5;
			incxy <<= 5;
			// (draw_roz_core: the start at the clip's left and this line, wrapped)
			ROZ_LINE(y, 0, VIS_W - 1, startx, starty, incxx, incxy);
		}
	} else {   // ("simple" mode)
		uint32_t startx = (uint32_t)(256 * (int16_t)ctrl[0x00]), starty = (uint32_t)(256 * (int16_t)ctrl[0x01]);
		int incyx = (int16_t)ctrl[0x02], incyy = (int16_t)ctrl[0x03], incxx = (int16_t)ctrl[0x04],
		    incxy = (int16_t)ctrl[0x05];
		if (ctrl[0x06] & 0x4000) {
			incyx *= 256;
			incyy *= 256;
		}
		if (ctrl[0x06] & 0x0040) {
			incxx *= 256;
			incxy *= 256;
		}
		startx -= (uint32_t)(yoff * incyx);
		starty -= (uint32_t)(yoff * incyy);
		startx -= (uint32_t)(xoff * incxx);
		starty -= (uint32_t)(xoff * incxy);
		startx <<= 5;
		starty <<= 5;
		incxx <<= 5;
		incxy <<= 5;
		incyx <<= 5;
		incyy <<= 5;
		startx += (uint32_t)(r0 * incyx);   // (the band's first row)
		starty += (uint32_t)(r0 * incyy);
		for (int y = r0; y <= r1; y++, startx += (uint32_t)incyx, starty += (uint32_t)incyy)
			ROZ_LINE(y, 0, VIS_W - 1, startx, starty, incxx, incxy);
	}
#undef ROZ_LINE
}

// the recorded call (gx_pre_call): chip, which, opaque and priority packed in arg
static void roz_call(int arg, int y0, int y1) { roz_rows(arg & 1, (arg >> 1) & 1, (arg >> 2) & 1, arg >> 3, y0, y1); }
static void roz_draw(int chip, int which, int opaque, int priority)
{
	gx_pre_call(roz_call, chip | which << 1 | opaque << 2 | priority << 3);
}

static void draw(void)   // (screen_update)
{
	static const int ci[6] = { 3, 4, 4, 4, 2, 1 };
	s_sprite_colorbase = k53251_palette_index(s_vs.k53251, 0);
	for (int plane = 0; plane < 6; plane++)
		s_layer_colorbase[plane] = k53251_palette_index(s_vs.k53251, ci[plane]);
	int layer[5] = { 0, 1, 3, 4, 5 };
	s_layerpri[0] = s_vs.k53251[3];   // (get_priority: CI3, CI4, CI0, CI2, CI1)
	s_layerpri[1] = s_vs.k53251[4];
	s_layerpri[2] = s_vs.k53251[0];
	s_layerpri[3] = s_vs.k53251[2];
	s_layerpri[4] = s_vs.k53251[1];
	for (int a = 0; a < 4; a++)   // (konami_sortlayers5: swapped when pri[a] <= pri[b])
		for (int b = a + 1; b < 5; b++)
			if (s_layerpri[a] <= s_layerpri[b]) {
				int t = s_layerpri[a]; s_layerpri[a] = s_layerpri[b]; s_layerpri[b] = t;
				t = layer[a]; layer[a] = layer[b]; layer[b] = t;
			}
	memset(gx_pre_pri(), 0, (size_t)(VIS_W * VIS_H));
	for (int plane = 0; plane < 5; plane++) {
		const int opaque = plane == 0, pri = plane == 0 ? 0 : 1 << (plane - 1);
		if (layer[plane] == 4)
			roz_draw(1, 0, opaque, pri);
		else if (layer[plane] == 5)
			roz_draw(0, 1, opaque, pri);
		else if (opaque)
			gx_pre_tilemap_opaque(layer[plane], pri);
		else
			gx_pre_tilemap(layer[plane], pri);
	}
	gx_pre_sprites();
}

// ---- the main CPU's bus ----
static uint8_t k053246_r(int offset)
{
	if (!gx.k55673.objcha_line)
		return 0;
	const uint8_t *r = gx.k55673.kx46_regs;
	uint32_t addr = (uint32_t)r[6] << 17 | (uint32_t)r[7] << 9 | (uint32_t)r[4] << 1 | (uint32_t)((offset & 1) ^ 1);
	return s_sprrom[addr & (s_sprrom_size - 1)];
}

static uint16_t main_r16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x490000 && a < 0x494000)
		return k056832_ram_word_r(&gx.k56832, (int)((a & 0x1fff) >> 1));
	if (a >= 0x498000 && a < 0x4a0000) {   // (rom_word_8000_r)
		const k056832_t *k = &gx.k56832;
		const uint32_t addr = 0x8000u * (uint32_t)k->cur_gfx_banks + (a - 0x498000);
		return (uint16_t)((addr + 1 < k->rom_size ? k->rom[addr + 1] : 0) | (addr < k->rom_size ? k->rom[addr] : 0) << 8);
	}
	if (a >= 0x4a0000 && a < 0x4a1000)
		return k055673_ram_r(&gx.k55673, (int)((a - 0x4a0000) >> 1));
	if (a >= 0x4a1000 && a < 0x4a4000)
		return d.extra[(a - 0x4a1000) >> 1];
	if (a == 0x4c0000)
		return (uint16_t)(k053246_r(0) << 8 | k053246_r(1));
	if (a == 0x4e0000)
		return (uint16_t)rbx_port_read(s_port_p12);
	if (a == 0x4e0002)
		return (uint16_t)rbx_port_read(s_port_sys);
	if (a == 0x4e4000)
		return (uint16_t)(rbx_port_read(s_port_dsw2) & 0xff);
	if (a >= 0x4f8000 && a < 0x4f8020)   // (the K053252: umask ff00)
		return (uint16_t)(d.ccu[((a - 0x4f8000) >> 1) & 15] << 8);
	if (a >= 0x500000 && a < 0x502000)
		return d.bg_vram[1][(a - 0x500000) >> 1];
	if (a >= 0x508000 && a < 0x50a000)
		return d.bg_vram[0][(a - 0x508000) >> 1];
	if (a >= 0x510000 && a < 0x514000)
		return d.k936_line[0][(a - 0x510000) >> 1];
	if (a >= 0x518000 && a < 0x51c000)
		return d.k936_line[1][(a - 0x518000) >> 1];
	return 0;
}
static uint8_t main_r8(uint32_t a)
{
	const uint16_t w = main_r16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}

static inline void combine(uint16_t *p, uint16_t v, uint16_t mask) { *p = (uint16_t)((v & mask) | (*p & ~mask)); }

static void main_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a >= 0x490000 && a < 0x494000)
		k056832_ram_word_w(&gx.k56832, (int)((a & 0x1fff) >> 1), v, mask);
	else if (a >= 0x4a0000 && a < 0x4a1000)
		k055673_ram_w(&gx.k55673, (int)((a - 0x4a0000) >> 1), v, mask);
	else if (a >= 0x4a1000 && a < 0x4a4000)
		combine(&d.extra[(a - 0x4a1000) >> 1], v, mask);
	else if ((a & ~0x4000u) >= 0x4c0000 && (a & ~0x4000u) < 0x4c0008) {   // (k053246_w, mirrored)
		const int o = (int)((a & ~0x4000u) - 0x4c0000);
		if (mask & 0xff00) k055673_k053246_w(&gx.k55673, o, (uint8_t)(v >> 8));
		if (mask & 0x00ff) k055673_k053246_w(&gx.k55673, o + 1, (uint8_t)v);
	} else if (a >= 0x4c8000 && a < 0x4c8008)
		k056832_b_word_w(&gx.k56832, (int)((a - 0x4c8000) >> 1), v, mask);
	else if (a >= 0x4cc000 && a < 0x4cc040)
		k056832_word_w(&gx.k56832, (int)((a - 0x4cc000) >> 1), v, mask);
	else if (a >= 0x4d0000 && a < 0x4d0020)
		combine(&d.k936_ctrl[0][(a - 0x4d0000) >> 1], v, mask);
	else if (a >= 0x4d4000 && a < 0x4d4020)
		combine(&d.k936_ctrl[1][(a - 0x4d4000) >> 1], v, mask);
	else if (a == 0x4ec000) {   // (control_w: bit 10 the K053246's readback)
		combine(&d.control, v, mask);
		gx.k55673.objcha_line = (d.control >> 10) & 1;
	} else if (a == 0x4f0000) {
		if (mask & 0xff00)
			q_put(Q_LATCH, (uint8_t)(v >> 8));
	} else if (a == 0x4f4000)
		q_put(Q_NMI, 0);   // (sound_cause_nmi_w: a pulse)
	else if (a >= 0x4f8000 && a < 0x4f8020) {   // (the K053252: umask ff00; INT1 ack clears IRQ 2)
		if (mask & 0xff00) {
			const int r = ((a - 0x4f8000) >> 1) & 15;
			d.ccu[r] = (uint8_t)(v >> 8);
			if (r == 0x0e)
				m68k_bus_set_line(2, 0);
		}
	} else if (a >= 0x4fc000 && a < 0x4fc020) {
		if (mask & 0x00ff)
			d.k53251[((a - 0x4fc000) >> 1) & 0xf] = (uint8_t)(v & 0x3f);
	} else if (a >= 0x500000 && a < 0x502000)
		combine(&d.bg_vram[1][(a - 0x500000) >> 1], v, mask);
	else if (a >= 0x508000 && a < 0x50a000)
		combine(&d.bg_vram[0][(a - 0x508000) >> 1], v, mask);
	else if (a >= 0x510000 && a < 0x514000)
		combine(&d.k936_line[0][(a - 0x510000) >> 1], v, mask);
	else if (a >= 0x518000 && a < 0x51c000)
		combine(&d.k936_line[1][(a - 0x518000) >> 1], v, mask);
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v)
{
	main_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00);
}

// ---- the sound board (sound_map) ----
static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_zrom_size ? s_zrom[a] : 0xff;
	if (a < 0xc000)
		return d.zram[a - 0x8000];
	if (a == 0xc000 || a == 0xc001)
		return rbx_ym_read(0, a & 1);
	if (a >= 0xd000 && a <= 0xd002)
		return oki_read(&s_oki);
	if (a == 0xe000)
		return d.latch;
	return 0xff;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0xc000)
		d.zram[a - 0x8000] = v;
	else if (a == 0xc000 || a == 0xc001)
		rbx_ym_write(0, a & 1, v);
	else if (a >= 0xd000 && a <= 0xd002)
		oki_write(&s_oki, v);
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }
static void ym_irq(int state) { d.z80_irq = (uint8_t)(state != 0); }   // (irq_handler: the Z80's IRQ line)

static void sound_run(int n)
{
	if (n <= 0)
		return;
	d.z_frac += (uint64_t)Z80_CLOCK * (uint64_t)n;
	const unsigned long cyc = (unsigned long)(d.z_frac / SND_RATE);
	d.z_frac -= (uint64_t)cyc * SND_RATE;
	d.z_end += cyc;
	int line = d.z80_irq;
	z80_run(&s_z, (unsigned long)d.z_end, &line, 0);
	int32_t yl[96], yr[96];
	memset(yl, 0, sizeof(yl));
	memset(yr, 0, sizeof(yr));
	d.ym_frac += (uint64_t)YM_CLOCK * (uint64_t)n;
	const uint32_t yc = (uint32_t)(d.ym_frac / SND_RATE);
	d.ym_frac -= (uint64_t)yc * SND_RATE;
	const int yn = rbx_ym_advance_lr(0, yc, yl, yr, 96);
	for (int i = 0; i < n; i++) {
		d.oki_frac += OKI_CLOCK / 132;   // (the OKI's rate: clock / 132 with pin 7 high)
		while (d.oki_frac >= SND_RATE) {
			d.oki_frac -= SND_RATE;
			d.oki_hold = oki_sample(&s_oki);
		}
		const int k = yn > 0 ? i * yn / n : -1;
		if (s_out_pos < 2048) {
			s_out_l[s_out_pos] = (k >= 0 ? yl[k] : 0) + d.oki_hold;
			s_out_r[s_out_pos] = (k >= 0 ? yr[k] : 0) + d.oki_hold;
			s_out_pos++;
		}
	}
}

// ---- the sound board a frame behind on the worker (rbx_aux): the latch and the NMI queued with the samples before
// them (the host reads nothing back) ----
static void job(void)
{
	if (!__atomic_load_n(&s_job_frame, __ATOMIC_ACQUIRE))
		return;
	const int b = s_qcur ^ 1;
	s_out_pos = 0;
	for (int i = 0; i < s_qn[b]; i++) {
		const q_ev_t *e = &s_q[b][i];
		switch (e->kind) {
		case Q_ADV: sound_run((int)e->v); break;
		case Q_LATCH: d.latch = (uint8_t)e->v; break;
		case Q_NMI: z80_gen_nmi(&s_z); break;
		}
	}
	s_qn[b] = 0;
	s_out_n = s_out_pos;
	__atomic_store_n(&s_job_frame, 0, __ATOMIC_RELEASE);
}

static void sound_frame(void)
{
	rbx_aux_sync();
	for (int i = 0; i < s_out_n; i++)
		rbx_audio_put_stereo(s_out_l[i], s_out_r[i]);
	s_out_n = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void dbz_exit(void);
static void dbz_reset(void);

static void patch16(uint32_t a, uint16_t v)   // (the init's ROM patches, the ROM as big-endian bytes)
{
	if (a + 1 < s_rom_size) {
		s_rom[a] = (uint8_t)(v >> 8);
		s_rom[a + 1] = (uint8_t)v;
	}
}

static int dbz_init(const rbx_game *g)
{
	uint32_t tile_size, oki_size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	const uint8_t *tiles = rbx_region("k056832", &tile_size);
	s_sprrom = rbx_region("k053246", &s_sprrom_size);
	const uint8_t *oki = rbx_region("oki", &oki_size);
	uint32_t rsz[2];
	const uint8_t *roz[2] = { rbx_region("roztiles0", &rsz[0]), rbx_region("roztiles1", &rsz[1]) };
	if (!s_rom || !s_zrom || !tiles || !s_sprrom || !oki || !roz[0] || !roz[1]) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	// (init_dbz / init_dbza / init_dbz2: the ROM tests skipped, a loop bound)
	s_dbz2 = !strcmp(g->set, "dbz2") || !strcmp(g->set, "dbz2bl");
	if (!strcmp(g->set, "dbz")) {
		static const uint32_t nop[] = { 0x7b0, 0x7b2, 0x7b4, 0x7b6, 0x7c0, 0x7c2, 0x7c4, 0x7c6, 0x9a8, 0x9aa, 0x9ac, 0x9ae,
		                                0x9ea, 0x9ec, 0x9ee, 0x9f0, 0x80c, 0x80e, 0x810 };
		patch16(0x76c, 0x007f);
		for (unsigned i = 0; i < sizeof(nop) / sizeof(nop[0]); i++)
			patch16(nop[i], 0x4e71);
	} else if (!strcmp(g->set, "dbza")) {
		static const uint32_t nop[] = { 0x78c, 0x78e, 0x790, 0x792, 0x982, 0x984, 0x986, 0x988, 0x98a, 0x98c, 0x98e, 0x990 };
		for (unsigned i = 0; i < sizeof(nop) / sizeof(nop[0]); i++)
			patch16(nop[i], 0x4e71);
	} else if (s_dbz2) {
		static const uint32_t nop[] = { 0xa88, 0xa8a, 0xa8c, 0xa8e, 0xa98, 0xa9a, 0xa9c, 0xa9e, 0xc66, 0xc68, 0xc6a, 0xc7c,
		                                0xc7e, 0xc80, 0xc9e, 0xca0, 0xca2, 0xcb4, 0xcb6, 0xcb8, 0xae4, 0xae6, 0xae8 };
		patch16(0xa48, 0x007f);
		for (unsigned i = 0; i < sizeof(nop) / sizeof(nop[0]); i++)
			patch16(nop[i], 0x4e71);
	}
	for (int w = 0; w < 2; w++) {   // (gfx_16x16x4_packed_msb: two pixels a byte, the high nibble first)
		s_roz_n[w] = rsz[w] / 128;
		free(s_roz_gfx[w]);
		s_roz_gfx[w] = (uint8_t *)malloc((size_t)s_roz_n[w] * 256);
		if (!s_roz_gfx[w]) {
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
		for (uint32_t i = 0; i < s_roz_n[w] * 128; i++) {
			s_roz_gfx[w][2 * i] = roz[w][i] >> 4;
			s_roz_gfx[w][2 * i + 1] = roz[w][i] & 15;
		}
	}
	memset(&gx, 0, sizeof(gx));
	memset(&d, 0, sizeof(d));
	gx.pal_entries = 0x2000;
	gx.pal555 = 1;
	gx.pal555rgb = 1;
	gx.tilecb = GX_TILECB_PRE;
	gx.mixmode = GX_MIXMODE_PRE;
	gx.pre_tilecb = tile_cb;
	gx.pre_sprcb = sprite_cb;
	gx.pre_capture = capture;
	gx.pre_draw = draw;
	gx.pre_screen_w = 512;
	gx.pre_bands = 4;   // (the drawing in four bands: the K053936 layers as recorded calls)
	k056832_init(&gx.k56832, tiles, tile_size, K056832_BPP_4, 1, 1);
	k055673_init(&gx.k55673, s_sprrom, s_sprrom_size, K055673_LAYOUT_053247, -87, 32);
	gx_video_set_area(0, VIS_W);
	gx_video_set_y0(0);
	gx_video_set_height(VIS_H);
	m68k_init();
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0x000000, 0x0fffff, s_rom, s_rom_size < 0x100000 ? s_rom_size : 0x100000, 1, 0);
	m68k_bus_map(0x480000, 0x48ffff, gx.workram, 0x10000, 1, 1);
	m68k_bus_map(0x4a8000, 0x4abfff, gx.palram, 0x4000, 1, 1);
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 256;
	yc.irq = ym_irq;
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: YM2151", g->set);
		return -1;
	}
	oki_init(&s_oki, OKI_CLOCK, 1, oki, oki_size, 256);
	s_port_p12 = rbx_port_find("P1_P2");
	s_port_sys = rbx_port_find("SYSTEM_DSW1");
	s_port_dsw2 = rbx_port_find("DSW2");
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, 0x10000) || gx_video_init()) {
		rbx_fail("%s: out of memory", g->set);
		dbz_exit();
		return -1;
	}
	for (int i = 0; i < 0x10000; i++) {   // (the bitmap: RGB 565)
		const int r = (i >> 11) & 31, gg = (i >> 5) & 63, b = i & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (gg << 2 | gg >> 4) << 8 | (b << 3 | b >> 2));
	}
	// (video_start: the layer offsets)
	k056832_set_layer_offs(&gx.k56832, 0, !strcmp(g->set, "dbz") ? -34 : -35, -16);
	k056832_set_layer_offs(&gx.k56832, 1, -31, -16);
	k056832_set_layer_offs(&gx.k56832, 3, -31, -16);
	dbz_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void dbz_exit(void)
{
	rbx_aux_set_job(NULL);
	gx_video_exit();
	rbx_ym_exit(0);
	for (int w = 0; w < 2; w++) {
		free(s_roz_gfx[w]);
		s_roz_gfx[w] = NULL;
	}
}

static void dbz_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(gx.workram, 0, sizeof(gx.workram));
	memset(d.zram, 0, sizeof(d.zram));
	memset(d.k53251, 0, sizeof(d.k53251));
	memset(d.ccu, 0, sizeof(d.ccu));
	memset(d.k936_ctrl, 0, sizeof(d.k936_ctrl));
	d.latch = 0;
	d.z80_irq = 0;
	d.control = 0;
	d.cycles_done = 0;
	d.cyc_frac = d.z_frac = d.z_end = d.ym_frac = d.oki_frac = d.snd_frac = 0;
	d.oki_hold = 0;
	d.frame = 0;
	rbx_ym_reset(0);
	oki_reset(&s_oki);
	z80_init(&s_z);
	s_z.read_byte = z_rd;
	s_z.write_byte = z_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	for (int p = 0; p < 256; p++)
		s_zmap[p] = p < 0x80 && (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
	for (int p = 0; p < 0x40; p++)
		s_zmap[0x80 + p] = d.zram + p * 256;
	s_z.read_map = s_zmap;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
}

static void dbz_frame(void)
{
	rbx_audio.frame_mhz = FPS * 1000;
	for (int line = 0; line < LINES; line++) {
		const int vpos = (line + 256) % LINES;   // (the frame from the vblank start)
		if (vpos == 256) {   // (scanline 256: vblank-out IRQ 2, asserted until the K053252's INT1 ack)
			gx_video_capture();
			m68k_bus_set_line(2, 1);
		}
		if (vpos == 0 && (gx.k55673.kx46_regs[5] & 0x10))   // (scanline 0: vblank-in IRQ 4)
			m68k_bus_hold_irq(4);
		d.cyc_frac += MAIN_CLOCK;
		const int cyc = (int)(d.cyc_frac / (FPS * LINES));
		d.cyc_frac -= (uint64_t)cyc * (FPS * LINES);
		d.cycles_done += m68k_execute(cyc);
		d.snd_frac += SND_RATE;
		const int ns = (int)(d.snd_frac / (FPS * LINES));
		d.snd_frac -= (uint64_t)ns * (FPS * LINES);
		if (ns > 0)
			q_put(Q_ADV, (uint32_t)ns);
	}
	sound_frame();
	d.frame++;
	gx.frame = d.frame;
}

static int dbz_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	int rc = 0;
	rbx_aux_sync();   // (the job done: the board at the frame's end, its samples waiting)
	if ((uint32_t)m68k_context_size() > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	rc |= rbx_state_io(ctx, "dbz_cpu", cpu, (uint32_t)m68k_context_size());
	rc |= rbx_state_io(ctx, "dbz_bus", &m68k_bus.irq_pending, sizeof(uint32_t) * 2);
	rc |= rbx_state_io(ctx, "gx", &gx, (uint32_t)offsetof(gx_t, state_end));
	rc |= k056832_state(&gx.k56832, ctx, load);
	rc |= k055673_state(&gx.k55673, ctx, load);
	rc |= rbx_state_io(ctx, "dbz", &d, (uint32_t)offsetof(__typeof__(d), state_end));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "dbz_oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "dbz_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "dbz_outl", s_out_l, sizeof(s_out_l));
	rc |= rbx_state_io(ctx, "dbz_outr", s_out_r, sizeof(s_out_r));
	if (load && !rc) {
		s_out_n = qn >= 0 && qn <= 2048 ? qn : 0;
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
	}
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "dbz_z80", &z, sizeof(z));
	if (!rc && load) {
		m68k_set_context(cpu);
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

static int dbz_save(void *ctx) { return dbz_state(ctx, 0); }
static int dbz_load(void *ctx) { return dbz_state(ctx, 1); }
static int dbz_supported(const char *machine) { return !strcmp(machine, "dbz"); }

const rbx_driver rbx_drv_dbz = { "Banpresto Dragon Ball Z", dbz_init, dbz_exit, dbz_reset, dbz_frame, dbz_save, dbz_load,
                                 dbz_supported, gx_render, gx_render_help };
