/*  Toaplan / Taito Twin Cobra, Flying Shark and Wardner boards. Ported from MAME src/mame/toaplan/twincobr.cpp,
    twincobr_m.cpp, twincobr_v.cpp, wardner.cpp, toaplan_dsp.cpp, toaplan_scu.cpp (BSD-3-Clause, copyright-holders:
    Quench, Darren Olafson, Stephane Humbert).
    Twin Cobra: a 68000 at 7 / 10 MHz; Wardner: a Z80 at 6 MHz with banked ROM. Both with a TMS32010 DSP at 14 MHz
    that halts the main CPU while it works on its RAM, a Z80 at 3.5 MHz with a YM3812 (Flyin' Shark: YM3526) sharing
    2 KB with the main CPU. Video: three 8x8 tile layers reached through offset / data ports, the SCU's 16x16
    sprites (four priorities) copied at the vblank; xBGR 555 palette RAM. 320x240 of 446x286 at 7 MHz.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/tms320c1x/tms320c1x.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../../video/decoband.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 54877          // set_raw(7 MHz, 446, 0, 320, 286, 0, 240)
#define LINES 286
#define VIS_W 320
#define VIS_H 240
#define DSP_CLOCK 3500000        // (TMS32010 at 14 MHz: a cycle is 4 clocks)
#define SND_CLOCK 3500000
#define NPAL 2048
#define PEN_BLACK NPAL

enum { M_TWINCOBR, M_TWINCOBRW, M_FSHARK, M_FSHARKBT, M_FNSHARK, M_WARDNER };

static int s_m, s_wardner;
static uint32_t s_main_clock;
static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static uint16_t *s_dsprom;
static rbx_gfx s_gchar, s_gfg, s_gbg, s_gspr;
static int s_xoffs, s_xoffs_flipped;
static int s_port_p1, s_port_p2, s_port_sys, s_port_dswa, s_port_dswb, s_port_vbl;
static z80 s_wmain, s_snd;
static tms320c1x_t s_dsp;
static const uint8_t *s_wmap[256], *s_zmap[256];
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static uint8_t *s_pri;

static struct {
	uint16_t ram[0x2000];          // twincobr 030000-033fff (host words); wardner 7000-7fff (bytes, first 4 KB)
	uint16_t spr[0x800], spr_buf[0x800];   // (wardner: little-endian bytes, as its 16-bit words)
	uint16_t pal[0x800];           // (wardner: little-endian bytes)
	uint16_t txv[0x800], fgv[0x1000], bgv[0x2000];
	uint8_t shram[0x800], zram[0x80], zram2[0x800];
	uint16_t txoffs, bgoffs, fgoffs, txsx, txsy, bgsx, bgsy, fgsx, fgsy;
	uint8_t mainlatch, coinlatch, intenable, flip, display_on, wbank;
	uint16_t bg_bank, fg_bank;
	// DSP interface (toaplan_dsp)
	uint8_t dsp_on, dsp_bio, dsp_execute, main_halt;
	uint32_t dsp_seg, dsp_addr;
	int32_t fsharkbt_8741;
	int main_irq, ym_irq, line;
	int64_t dsp_t;
	int32_t main_debt;
	uint64_t main_frac, dsp_frac, snd_frac, ym_frac;
	uint8_t state_end;
} *t;   // (calloc in init: ADR-040)

static struct {
	uint16_t txv[0x800], fgv[0x1000], bgv[0x1000], spr[0x800], pal[0x800];
	uint16_t txsx, txsy, bgsx, bgsy, fgsx, fgsy, fg_bank;
	uint8_t flip, display_on;
} *s_snap;

static inline uint16_t le16(const void *p) { const uint8_t *b = (const uint8_t *)p; return (uint16_t)(b[0] | b[1] << 8); }

// ---- the DSP and the latches ----
static void halt_main(void)
{
	t->main_halt = 1;
	if (s_wardner)
		z80_break = 1;
	else
		m68k_end_timeslice();
}

static void dsp_int_w(int enable)
{
	t->dsp_on = (uint8_t)enable;
	tms320c1x_set_irq(&s_dsp, enable);
	if (enable)
		halt_main();
}

static void int_enable_w(int state)
{
	t->intenable = (uint8_t)state;
	if (!state) {
		if (s_wardner)
			t->main_irq = 0;
		else
			m68k_bus_set_line(4, 0);
	}
}

static void mainlatch_bit(int bit, int d)
{
	switch (bit) {
	case 2: int_enable_w(d); break;
	case 3: t->flip = (uint8_t)d; break;
	case 4: t->bg_bank = d ? 0x1000 : 0; break;
	case 5: t->fg_bank = d ? 0x1000 : 0; break;
	case 6:
		if (s_wardner)
			t->display_on = (uint8_t)d;
		else if (s_m == M_TWINCOBR || s_m == M_TWINCOBRW)
			dsp_int_w(d);
		break;
	case 7:
		if (!s_wardner)
			t->display_on = (uint8_t)d;
		break;
	}
}

static void coinlatch_bit(int bit, int d)
{
	if (bit == 0 && (s_wardner || s_m == M_FSHARK || s_m == M_FSHARKBT || s_m == M_FNSHARK))
		dsp_int_w(d);
}

static void latch_w(uint8_t *q, uint8_t data, void (*cb)(int, int))   // (ls259 write_nibble_d0)
{
	const int bit = (data & 0x0e) >> 1, d = data & 1;
	if (((*q >> bit) & 1) == d)
		return;
	*q = (uint8_t)((*q & ~(1 << bit)) | d << bit);
	cb(bit, d);
}

static uint8_t w_read(uint16_t a);
static void w_write(uint16_t a, uint8_t v);
static uint16_t tc_read16(uint32_t a);
static void tc_write16(uint32_t a, uint16_t v, uint16_t mask);

static uint16_t dsp_in(void *u, int port)
{
	(void)u;
	if (port == 1) {   // dsp_r: the host's RAM
		if (s_wardner) {
			if (t->dsp_seg == 0x7000 || t->dsp_seg == 0x8000 || t->dsp_seg == 0xa000)
				return (uint16_t)(w_read((uint16_t)(t->dsp_seg + t->dsp_addr)) | w_read((uint16_t)(t->dsp_seg + t->dsp_addr + 1)) << 8);
			return 0;
		}
		if (t->dsp_seg == 0x30000 || t->dsp_seg == 0x40000 || t->dsp_seg == 0x50000)
			return tc_read16(t->dsp_seg + t->dsp_addr);
		return 0;
	}
	if (port == 2 && s_m == M_FSHARKBT) {   // fsharkbt_dsp_r: the 8741
		t->fsharkbt_8741++;
		return (uint16_t)(t->fsharkbt_8741 & 1);
	}
	return 0;
}

static void dsp_out(void *u, int port, uint16_t v)
{
	(void)u;
	switch (port) {
	case 0:   // dsp_addrsel_w
		if (s_wardner) {
			t->dsp_seg = v & 0xe000;
			t->dsp_addr = (uint32_t)(v & 0x07ff) << 1;
			if (t->dsp_seg == 0x6000)
				t->dsp_seg = 0x7000;
		} else {
			t->dsp_seg = (uint32_t)(v & 0xe000) << 3;
			t->dsp_addr = (uint32_t)(v & 0x1fff) << 1;
		}
		break;
	case 1: {   // dsp_w
		int exec = 0;
		if (s_wardner) {
			if (t->dsp_seg == 0x7000 || t->dsp_seg == 0x8000 || t->dsp_seg == 0xa000) {
				exec = t->dsp_seg == 0x7000 && t->dsp_addr < 3 && v == 0;
				w_write((uint16_t)(t->dsp_seg + t->dsp_addr), (uint8_t)v);
				w_write((uint16_t)(t->dsp_seg + t->dsp_addr + 1), (uint8_t)(v >> 8));
			}
		} else if (t->dsp_seg == 0x30000 || t->dsp_seg == 0x40000 || t->dsp_seg == 0x50000) {
			exec = t->dsp_seg == 0x30000 && t->dsp_addr < 3 && v == 0;
			tc_write16(t->dsp_seg + t->dsp_addr, v, 0xffff);
		}
		t->dsp_execute = (uint8_t)exec;
		break;
	}
	case 3:   // dsp_bio_w
		if (v & 0x8000)
			t->dsp_bio = 0;
		if (v == 0) {
			if (t->dsp_execute) {
				t->main_halt = 0;
				t->dsp_execute = 0;
			}
			t->dsp_bio = 1;
		}
		break;
	}
}

static int dsp_bio(void *u) { (void)u; return t->dsp_bio; }

static void dsp_start(void)
{
	tms320c1x_init(&s_dsp);
	s_dsp.rom = s_dsprom;
	s_dsp.rom_mask = 0xfff;
	s_dsp.ram_mask = 0xff;
	s_dsp.in = dsp_in;
	s_dsp.out = dsp_out;
	s_dsp.bio = dsp_bio;
	tms320c1x_reset(&s_dsp);
}

// ---- video ports (twincobr_v) ----
static void combine(uint16_t *r, uint16_t v, uint16_t mask) { *r = (uint16_t)((*r & ~mask) | (v & mask)); }

static void vid_w(int reg, uint16_t v, uint16_t mask)   // reg: layer * 4 + (0 scroll x, 1 scroll y, 2 offset, 3 data)
{
	switch (reg) {
	case 0: combine(&t->txsx, v, mask); break;
	case 1: combine(&t->txsy, v, mask); break;
	case 2: combine(&t->txoffs, v, mask); t->txoffs %= 0x800; break;
	case 3: combine(&t->txv[t->txoffs], v, mask); break;
	case 4: combine(&t->bgsx, v, mask); break;
	case 5: combine(&t->bgsy, v, mask); break;
	case 6: combine(&t->bgoffs, v, mask); t->bgoffs %= 0x1000; break;
	case 7: combine(&t->bgv[t->bgoffs + t->bg_bank], v, mask); break;
	case 8: combine(&t->fgsx, v, mask); break;
	case 9: combine(&t->fgsy, v, mask); break;
	case 10: combine(&t->fgoffs, v, mask); t->fgoffs %= 0x1000; break;
	case 11: combine(&t->fgv[t->fgoffs], v, mask); break;
	}
}

static uint16_t vid_r(int layer)
{
	switch (layer) {
	case 0: return t->txv[t->txoffs];
	case 1: return t->bgv[t->bgoffs + t->bg_bank];
	case 2: return t->fgv[t->fgoffs];
	}
	return 0;
}

// ---- Twin Cobra: the 68000 ----
static uint16_t tc_read16(uint32_t a)
{
	a &= 0xfffffe;
	if (a >= 0x030000 && a < 0x034000) return t->ram[(a - 0x030000) >> 1];
	if (a >= 0x040000 && a < 0x041000) return t->spr[(a - 0x040000) >> 1];
	if (a >= 0x050000 && a < 0x050e00) return t->pal[(a - 0x050000) >> 1];
	switch (a) {
	case 0x078000: return (uint16_t)(rbx_port_read(s_port_dswa) & 0xff);
	case 0x078002: return (uint16_t)(rbx_port_read(s_port_dswb) & 0xff);
	case 0x078004: return (uint16_t)(rbx_port_read(s_port_p1) & 0xff);
	case 0x078006: return (uint16_t)(rbx_port_read(s_port_p2) & 0xff);
	case 0x078008: return (uint16_t)(rbx_port_read(s_port_vbl) & 0xff);
	case 0x07e000: return vid_r(0);
	case 0x07e002: return vid_r(1);
	case 0x07e004: return vid_r(2);
	}
	if (a >= 0x07a000 && a < 0x07b000) return t->shram[(a - 0x07a000) >> 1];
	return 0;
}

static void tc_write16(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xfffffe;
	if (a >= 0x030000 && a < 0x034000) { combine(&t->ram[(a - 0x030000) >> 1], v, mask); return; }
	if (a >= 0x040000 && a < 0x041000) { combine(&t->spr[(a - 0x040000) >> 1], v, mask); return; }
	if (a >= 0x050000 && a < 0x050e00) { combine(&t->pal[(a - 0x050000) >> 1], v, mask); return; }
	if (a >= 0x070000 && a < 0x076000 && (a & 0x1fff) < 6) {
		vid_w((int)((a - 0x070000) >> 13) * 4 + (int)((a & 7) >> 1), v, mask);
		return;
	}
	if (a >= 0x07e000 && a < 0x07e006) {
		vid_w((int)((a - 0x07e000) >> 1) * 4 + 3, v, mask);
		return;
	}
	if (a >= 0x07a000 && a < 0x07b000) {
		if (mask & 0xff)
			t->shram[(a - 0x07a000) >> 1] = (uint8_t)v;
		return;
	}
	if (a == 0x07800a && (mask & 0xff)) latch_w(&t->coinlatch, (uint8_t)v, coinlatch_bit);
	else if (a == 0x07800c && (mask & 0xff)) latch_w(&t->mainlatch, (uint8_t)v, mainlatch_bit);
}

static uint16_t m_r16(uint32_t a) { return tc_read16(a); }
static uint8_t m_r8(uint32_t a) { const uint16_t w = tc_read16(a); return (uint8_t)((a & 1) ? w : w >> 8); }
static void m_w16(uint32_t a, uint16_t v) { tc_write16(a, v, 0xffff); }
static void m_w8(uint32_t a, uint8_t v) { tc_write16(a, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00); }

// ---- Wardner: the main Z80 ----
static void w_map(void)
{
	for (int p = 0x80; p < 0x100; p++)
		s_wmap[p] = t->wbank ? s_rom + (((uint32_t)(t->wbank & 7) * 0x8000 + (uint32_t)(p - 0x80) * 256) % s_rom_size) : NULL;
}

static uint8_t w_read(uint16_t a)
{
	if (a < 0x7000) return s_rom[a];
	if (a < 0x8000) return ((uint8_t *)t->ram)[a - 0x7000];
	if (t->wbank)
		return s_rom[((uint32_t)(t->wbank & 7) * 0x8000 + (a - 0x8000)) % s_rom_size];
	if (a < 0x9000) return ((uint8_t *)t->spr)[a - 0x8000];   // (view 0)
	if (a >= 0xa000 && a < 0xb000) return ((uint8_t *)t->pal)[a - 0xa000];
	if (a >= 0xc000 && a < 0xc800) return t->shram[a - 0xc000];
	return 0;
}
static void w_write(uint16_t a, uint8_t v)
{
	if (a < 0x7000) return;
	if (a < 0x8000) ((uint8_t *)t->ram)[a - 0x7000] = v;
	else if (a < 0x9000) ((uint8_t *)t->spr)[a - 0x8000] = v;
	else if (a >= 0xa000 && a < 0xb000) ((uint8_t *)t->pal)[a - 0xa000] = v;
	else if (a >= 0xc000 && a < 0xc800) t->shram[a - 0xc000] = v;
}
static uint8_t wz_read(void *u, uint16_t a) { (void)u; return w_read(a); }
static void wz_write(void *u, uint16_t a, uint8_t v) { (void)u; w_write(a, v); }

static uint8_t wz_in(z80 *z, uint8_t port)
{
	(void)z;
	switch (port) {
	case 0x50: return (uint8_t)rbx_port_read(s_port_dswa);
	case 0x52: return (uint8_t)rbx_port_read(s_port_dswb);
	case 0x54: return (uint8_t)rbx_port_read(s_port_p1);
	case 0x56: return (uint8_t)rbx_port_read(s_port_p2);
	case 0x58: return (uint8_t)rbx_port_read(s_port_sys);
	}
	if (port >= 0x60 && port <= 0x65)
		return (uint8_t)(vid_r((port - 0x60) >> 1) >> ((port & 1) * 8));
	return 0;
}
static void wz_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	const int sh = (port & 1) * 8;
	const uint16_t d = (uint16_t)(v << sh), m = (uint16_t)(0xff << sh);
	if (port >= 0x10 && port < 0x36 && (port & 0x0f) < 6) {
		const int layer = (port >> 4) - 1;   // 1x text, 2x bg, 3x fg
		vid_w(layer * 4 + ((port & 7) >> 1), d, m);
	} else if (port >= 0x60 && port <= 0x65)
		vid_w(((port - 0x60) >> 1) * 4 + 3, d, m);
	else if (port == 0x5a)
		latch_w(&t->coinlatch, v, coinlatch_bit);
	else if (port == 0x5c)
		latch_w(&t->mainlatch, v, mainlatch_bit);
	else if (port == 0x70) {   // wardner_bank_w: the view (0: RAM), the ROM bank
		t->wbank = v ? (uint8_t)(0x80 | (v & 7)) : 0;
		w_map();
	}
}

// ---- sound ----
static void ym_irq(int state) { t->ym_irq = state; }

static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_zrom[a % s_zrom_size];
	if (s_wardner) {
		if (a < 0x8080) return t->zram[a - 0x8000];
		if (a >= 0xc000 && a < 0xc800) return t->shram[a - 0xc000];
		if (a >= 0xc800 && a < 0xd000) return t->zram2[a - 0xc800];
		return 0;
	}
	if (a < 0x8800) return t->shram[a - 0x8000];
	return 0;
}
static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_wardner) {
		if (a >= 0x8000 && a < 0x8080) t->zram[a - 0x8000] = v;
		else if (a >= 0xc000 && a < 0xc800) t->shram[a - 0xc000] = v;
		else if (a >= 0xc800 && a < 0xd000) t->zram2[a - 0xc800] = v;
		return;
	}
	if (a >= 0x8000 && a < 0x8800) t->shram[a - 0x8000] = v;
}
static uint8_t snd_in(z80 *z, uint8_t port)
{
	(void)z;
	if (port <= 1) return rbx_ym_read(0, port & 1);
	if (s_wardner) return 0;
	switch (port) {
	case 0x10: return (uint8_t)rbx_port_read(s_port_sys);
	case 0x40: return (uint8_t)rbx_port_read(s_port_dswa);
	case 0x50: return (uint8_t)rbx_port_read(s_port_dswb);
	}
	return 0;
}
static void snd_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	if (port <= 1)
		rbx_ym_write(0, port & 1, v);
	else if (port == 0x20 && !s_wardner)
		latch_w(&t->coinlatch, v, coinlatch_bit);
}

// ---- video ----
// the lines [y0, y1) (the tile's lookups once per tile: the pixels are the same as per-pixel lookups)
static void draw_layer(const uint16_t *vram, uint32_t rows_mask, const rbx_gfx *g, uint16_t base, int colshift, uint32_t codemask,
                       uint32_t codeor, uint16_t sx, uint16_t sy, int opaque, uint8_t prio, int y0, int y1)
{
	const int flip = s_snap->flip;
	const int tbits = colshift == 12 ? 0x0fff : 0x07ff;
	const int cmul = colshift == 12 ? 16 : 8;
	for (int y = y0; y < y1; y++) {
		uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
		uint8_t *pr = s_pri + y * VIS_W;
		const uint32_t ty = (uint32_t)(flip ? 482 - y + sy : y + sy + 30) & rows_mask;
		const uint16_t *row = vram + (ty >> 3) * 64;
		uint32_t last = ~0u;
		const uint8_t *src = NULL;
		uint16_t cbase = 0;
		for (int x = 0; x < VIS_W; x++) {
			const uint32_t tx = (uint32_t)(flip ? 453 - x + sx : x + sx + 55) & 511;
			if ((tx >> 3) != last) {
				last = tx >> 3;
				const uint16_t c = row[tx >> 3];
				const uint32_t code = (((uint32_t)c & (uint32_t)tbits & codemask) | codeor) % g->count;
				src = g->pix + code * 64 + (ty & 7) * 8;
				cbase = (uint16_t)(base + (c >> colshift) * cmul);
			}
			const uint8_t p = src[tx & 7];
			if (opaque || p) {
				d[x] = (uint16_t)(cbase + p);
				pr[x] |= prio;
			}
		}
	}
}

// the layers in bands of lines drawn in parallel (decoband.h), then the sprites
#define BANDS 8
static void tc_band(int band)
{
	const int y0 = VIS_H * band / BANDS, y1 = VIS_H * (band + 1) / BANDS;
	memset(s_pri + y0 * VIS_W, 0, (size_t)(y1 - y0) * VIS_W);
	draw_layer(s_snap->bgv, 511, &s_gbg, 1024, 12, 0xfff, 0, s_snap->bgsx, s_snap->bgsy, 1, 1, y0, y1);
	draw_layer(s_snap->fgv, 511, &s_gfg, 1280, 12, 0xfff, s_snap->fg_bank, s_snap->fgsx, s_snap->fgsy, 0, 2, y0, y1);
	draw_layer(s_snap->txv, 255, &s_gchar, 1536, 11, 0x7ff, 0, s_snap->txsx, s_snap->txsy, 0, 4, y0, y1);
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, tc_band };
static int tc_help(int helper) { return decoband_help(&s_bands, helper); }

static void tc_render(void)
{
	for (int i = 0; i < NPAL; i++) {   // xBGR_555
		const uint16_t w = s_snap->pal[i];
		const int r = w & 31, gg = (w >> 5) & 31, bb = (w >> 10) & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (gg << 3 | gg >> 2) << 8 | (bb << 3 | bb >> 2));
	}
	rbx_video.palette[PEN_BLACK] = 0;
	if (!s_snap->display_on) {
		for (int y = 0; y < VIS_H; y++)
			for (int x = 0; x < VIS_W; x++)
				rbx_video.bitmap[y * rbx_video.pitch + x] = PEN_BLACK;
		return;
	}
	decoband_run(&s_bands);
	for (int o = 0x800 - 4; o >= 0; o -= 4) {   // (toaplan_scu draw_sprites)
		const uint16_t *s = &s_snap->spr[o];
		const uint16_t attr = s[1];
		const int prio = (attr & 0x0c00) >> 10;
		if (!prio)
			continue;
		const int sy = s[3] >> 7;
		if (sy == 0x100)
			continue;
		static const uint32_t pmask[4] = { 0xfefe, 0xfcfc, 0xf0f0, 0 };
		int sx = s[2] >> 7;
		const int fx = (attr >> 8) & 1, fy = (attr >> 9) & 1;
		if (fx)
			sx -= s_xoffs_flipped;
		rbx_gfx_prio_transpen(&s_gspr, s[0] & 0x7ffu, (uint16_t)((attr & 0x3f) * 16), fx, fy, sx - s_xoffs, sy - 16,
		                      rbx_video.bitmap, s_pri, rbx_video.pitch, VIS_W, VIS_H, pmask[prio]);
	}
}

static void vblank_capture(void)   // (VIDEO_UPDATE_BEFORE_VBLANK: the screen, then the sprite copy and the IRQ)
{
	rbx_video_wait_idle();
	memcpy(s_snap->txv, t->txv, sizeof(t->txv));
	memcpy(s_snap->fgv, t->fgv, sizeof(t->fgv));
	memcpy(s_snap->bgv, t->bgv + t->bg_bank, sizeof(s_snap->bgv));
	if (s_wardner) {
		for (int i = 0; i < 0x800; i++) {
			s_snap->spr[i] = le16((const uint8_t *)t->spr_buf + 2 * i);
			s_snap->pal[i] = le16((const uint8_t *)t->pal + 2 * i);
		}
	} else {
		memcpy(s_snap->spr, t->spr_buf, sizeof(t->spr_buf));
		memset(s_snap->pal, 0, sizeof(s_snap->pal));
		memcpy(s_snap->pal, t->pal, 0xe00);
	}
	s_snap->txsx = t->txsx; s_snap->txsy = t->txsy;
	s_snap->bgsx = t->bgsx; s_snap->bgsy = t->bgsy;
	s_snap->fgsx = t->fgsx; s_snap->fgsy = t->fgsy;
	s_snap->fg_bank = t->fg_bank;
	s_snap->flip = t->flip;
	s_snap->display_on = t->display_on;
	rbx_video_frame_done();
	memcpy(t->spr_buf, t->spr, sizeof(t->spr));
	if (t->intenable) {
		if (s_wardner)
			t->main_irq = 1;
		else
			m68k_bus_set_line(4, 1);
	}
}

// ---- init / frame ----
static void snd_start(void)
{
	z80_init(&s_snd);
	s_snd.read_byte = snd_read;
	s_snd.write_byte = snd_write;
	s_snd.port_in = snd_in;
	s_snd.port_out = snd_out;
	s_snd.read_map = s_zmap;
}

static void w_start(void)
{
	z80_init(&s_wmain);
	s_wmain.read_byte = wz_read;
	s_wmain.write_byte = wz_write;
	s_wmain.port_in = wz_in;
	s_wmain.port_out = wz_out;
	s_wmain.read_map = s_wmap;
	s_wmain.int_data = 0xff;
}

static void tc_reset(void)
{
	memset(t, 0, offsetof(__typeof__(*t), state_end));
	t->fsharkbt_8741 = -1;
	z80_break = 0;
	if (s_wardner) {
		w_map();
		w_start();
	} else {
		m68k_bus_irq_reset();
		m68k_pulse_reset();
	}
	dsp_start();
	snd_start();
	rbx_ym_reset(0);
	s_mix_n = 0;
}

static void tc_exit(void);

static int tc_init(const rbx_game *g)
{
	free(t);
	t = calloc(1, sizeof(*t));
	if (!t) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	static const char *const mn[] = { "twincobr", "twincobrw", "fshark", "fsharkbt", "fnshark", "wardner" };
	s_m = M_TWINCOBR;
	for (int i = 0; i < 6; i++)
		if (!strcmp(g->machine, mn[i]))
			s_m = i;
	s_wardner = s_m == M_WARDNER;
	s_main_clock = s_wardner ? 6000000 : s_m == M_TWINCOBRW ? 10000000 : 7000000;
	s_xoffs = s_m == M_TWINCOBR || s_m == M_TWINCOBRW ? 31 : 32;
	s_xoffs_flipped = s_m == M_TWINCOBR || s_m == M_TWINCOBRW ? 15 : 14;
	uint32_t s1, s2, s3, s4, sd;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *drom = rbx_region("dsp:dsp", &sd);
	uint8_t *chars = rbx_region("chars", &s1), *fg = rbx_region("fg_tiles", &s2), *bg = rbx_region("bg_tiles", &s3);
	uint8_t *spr = rbx_region("scu", &s4);
	if (!s_rom || s_rom_size < (s_wardner ? 0x8000u : 0x30000u) || !s_zrom || s_zrom_size < 0x8000 || !drom || !chars || !fg || !bg || !spr) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{	// charlayout (3 bpp, RGN_FRAC(1,3)), tilelayout (RGN_FRAC(1,4)), the SCU's spritelayout (RGN_FRAC(1,4))
		static const uint32_t x8[8] = { 0, 1, 2, 3, 4, 5, 6, 7 }, y8[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
		static uint32_t x16[16], y16[16];
		for (int i = 0; i < 16; i++) {
			x16[i] = (uint32_t)i;
			y16[i] = (uint32_t)i * 16;
		}
		const uint32_t q1 = s1 / 3 * 8, q2 = s2 / 4 * 8, q3 = s3 / 4 * 8, q4 = s4 / 4 * 8;
		const uint32_t pc[3] = { 0, q1, 2 * q1 }, pf[4] = { 0, q2, 2 * q2, 3 * q2 }, pb[4] = { 0, q3, 2 * q3, 3 * q3 };
		const uint32_t ps[4] = { 0, q4, 2 * q4, 3 * q4 };
		const rbx_gfx_layout lc = { 8, 8, 3, pc, x8, y8, 64 }, lf = { 8, 8, 4, pf, x8, y8, 64 }, lb = { 8, 8, 4, pb, x8, y8, 64 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, x16, y16, 256 };
		if (rbx_gfx_decode(&s_gchar, &lc, chars, s1 / 3) || rbx_gfx_decode(&s_gfg, &lf, fg, s2 / 4) ||
		    rbx_gfx_decode(&s_gbg, &lb, bg, s3 / 4) || rbx_gfx_decode(&s_gspr, &ls, spr, s4 / 4)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			tc_exit();
			return -1;
		}
	}
	s_dsprom = (uint16_t *)calloc(0x1000, sizeof(uint16_t));
	s_snap = calloc(1, sizeof(*s_snap));
	s_pri = (uint8_t *)calloc(VIS_W, VIS_H);
	if (!s_dsprom || !s_snap || !s_pri) {
		rbx_fail("%s: out of memory", g->set);
		tc_exit();
		return -1;
	}
	for (uint32_t i = 0; i < sd / 2 && i < 0x1000; i++)   // (big-endian words)
		s_dsprom[i] = (uint16_t)(drom[2 * i] << 8 | drom[2 * i + 1]);
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = s_m == M_FNSHARK ? RBX_YM3526 : RBX_YM3812;
	yc.clock = SND_CLOCK;
	yc.gain[0] = 256;
	yc.irq = ym_irq;
	if (rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		tc_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, NPAL + 1)) {
		rbx_fail("%s: out of memory", g->set);
		tc_exit();
		return -1;
	}
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x80; p++)
		s_zmap[p] = s_zrom + p * 256;
	if (s_wardner) {
		memset(s_wmap, 0, sizeof(s_wmap));
		for (int p = 0; p < 0x70; p++)
			s_wmap[p] = s_rom + p * 256;
		for (int p = 0; p < 0x10; p++)
			s_wmap[0x70 + p] = (const uint8_t *)t->ram + p * 256;
	} else {
		m68k_bus_swap16(s_rom, s_rom_size);
		m68k_init();
		m68k_set_cpu_type(M68K_CPU_TYPE_68000);
		m68k_bus_reset_map();
		m68k_bus.read8 = m_r8;
		m68k_bus.read16 = m_r16;
		m68k_bus.write8 = m_w8;
		m68k_bus.write16 = m_w16;
		m68k_bus_map(0, 0x2ffff, s_rom, s_rom_size < 0x30000 ? s_rom_size : 0x30000, 1, 0);
		m68k_bus_map(0x030000, 0x033fff, t->ram, 0x4000, 1, 1);
		m68k_bus_map(0x040000, 0x040fff, t->spr, 0x1000, 1, 1);
		m68k_bus_map(0x050000, 0x050dff, t->pal, 0xe00, 1, 1);
	}
	s_port_p1 = rbx_port_find("P1");
	s_port_p2 = rbx_port_find("P2");
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_dswa = rbx_port_find("DSWA");
	s_port_dswb = rbx_port_find("DSWB");
	s_port_vbl = s_wardner ? s_port_sys : rbx_port_find("VBLANK");
	tc_reset();
	return 0;
}

static void tc_exit(void)
{
	rbx_video_wait_idle();
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gchar);
	rbx_gfx_free(&s_gfg);
	rbx_gfx_free(&s_gbg);
	rbx_gfx_free(&s_gspr);
	free(s_dsprom);
	s_dsprom = NULL;
	free(s_snap);
	s_snap = NULL;
	free(s_pri);
	s_pri = NULL;
	free(t);
	t = NULL;
}

static void tc_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		t->line = line;
		if (line == VIS_H)
			vblank_capture();
		rbx_port_custom(s_port_vbl, 0x80, line >= VIS_H ? 0x80u : 0);
		const int mc = rbx_line_cycles(&t->main_frac, s_main_clock, FRAME_MHZ, LINES);
		if (s_wardner) {
			const unsigned long end = s_wmain.cyc + (unsigned long)mc;
			if (!t->main_halt) {
				z80_run(&s_wmain, end, &t->main_irq, 0);
				z80_break = 0;
			}
			if (t->main_halt && (long)(s_wmain.cyc - end) < 0)
				s_wmain.cyc = end;
		} else if (!t->main_halt) {
			const int want = mc - t->main_debt;
			const int used = want > 0 ? m68k_execute(want) : 0;
			t->main_debt = t->main_halt ? 0 : used - want;
		}
		t->dsp_t += rbx_line_cycles(&t->dsp_frac, DSP_CLOCK, FRAME_MHZ, LINES);
		if (t->dsp_on) {
			if (t->dsp_t > s_dsp.cycles)
				tms320c1x_execute(&s_dsp, (int)(t->dsp_t - s_dsp.cycles));
		} else
			s_dsp.cycles = t->dsp_t;
		z80_run(&s_snd, s_snd.cyc + (unsigned long)rbx_line_cycles(&t->snd_frac, SND_CLOCK, FRAME_MHZ, LINES), &t->ym_irq, 0);
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&t->ym_frac, SND_CLOCK, FRAME_MHZ, LINES);
		s_mix_n += rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int tc_state(void *ctx, int load)
{
	int rc = 0;
	uint8_t *cpu = NULL;
	uint32_t csize = 0, irq[2] = { 0, 0 };
	z80 wm = s_wmain, zs = s_snd;
	if (!s_wardner) {
		csize = m68k_context_size();
		cpu = (uint8_t *)malloc(csize);
		if (!cpu)
			return -1;
		if (!load)
			m68k_get_context(cpu);
		irq[0] = m68k_bus_irq_pending();
		irq[1] = m68k_bus_cur->irq_asserted;
		rc |= rbx_state_io(ctx, "tc_m68k", cpu, csize);
		rc |= rbx_state_io(ctx, "tc_irq", irq, sizeof(irq));
	} else
		rc |= rbx_state_io(ctx, "tc_wmain", &wm, sizeof(wm));
	rc |= rbx_state_io(ctx, "tc", t, (uint32_t)offsetof(__typeof__(*t), state_end));
	rc |= rbx_state_io(ctx, "tc_dsp", &s_dsp, TMS320C1X_STATE_SIZE);
	rc |= rbx_state_io(ctx, "tc_snd", &zs, sizeof(zs));
	rc |= rbx_ym_state(0, ctx, load);
	if (!rc && load) {
		if (s_wardner) {
			wm.read_byte = wz_read;
			wm.write_byte = wz_write;
			wm.port_in = wz_in;
			wm.port_out = wz_out;
			wm.userdata = NULL;
			wm.read_map = s_wmap;
			s_wmain = wm;
			w_map();
		} else {
			m68k_set_context(cpu);
			m68k_bus_set_irq_pending(irq[0]);
			m68k_bus_cur->irq_asserted = irq[1];
			m68k_bus_update_irq();
		}
		zs.read_byte = snd_read;
		zs.write_byte = snd_write;
		zs.port_in = snd_in;
		zs.port_out = snd_out;
		zs.userdata = NULL;
		zs.read_map = s_zmap;
		s_snd = zs;
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int tc_save(void *ctx) { return tc_state(ctx, 0); }
static int tc_load(void *ctx) { return tc_state(ctx, 1); }
static int tc_supported(const char *machine)
{
	return !strcmp(machine, "twincobr") || !strcmp(machine, "twincobrw") || !strcmp(machine, "fshark") ||
	       !strcmp(machine, "fsharkbt") || !strcmp(machine, "fnshark");
}
static int wd_supported(const char *machine) { return !strcmp(machine, "wardner"); }

const rbx_driver rbx_drv_twincobr = { "Toaplan Twin Cobra", tc_init, tc_exit, tc_reset, tc_frame, tc_save, tc_load,
                                      tc_supported, tc_render, tc_help };
const rbx_driver rbx_drv_wardner = { "Toaplan Wardner", tc_init, tc_exit, tc_reset, tc_frame, tc_save, tc_load,
                                     wd_supported, tc_render, tc_help };
