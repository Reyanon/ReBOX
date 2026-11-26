/*  Tecmo Shadow Warriors / Ninja Gaiden, Wild Fang / Tecmo Knight and Raiga - Strato Fighter. Ported from MAME
    src/mame/tecmo/gaiden.cpp and src/mame/shared/tecmo_spr.cpp (BSD-3-Clause, copyright-holders: Victor Trucco,
    Nicola Salmoria, David Haywood).
    68000 at 9.216 MHz (IRQ 5 at the vblank, held until acknowledged); a text layer of 8x8 tiles and two of 16x16, 8x8
    sprites in blocks of up to 8x8, all mixed by the Tecmo mixer with additive blending (so the frame is RGB). The
    MCUs of Wild Fang and Raiga are simulated as MAME does (their jump tables). Sound: Z80 at 4 MHz, two YM2203s (the
    first one's IRQ) and an OKI6295 at 1 MHz, the latch on the NMI.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/decoband.h"
#include "../../video/gfxlayout.h"
#include "../video/tecmo_mix.h"
#include "../video/tecmo_spr.h"

#include <stdlib.h>
#include <string.h>

// Ninja Gaiden: 68000 9.216 MHz, Z80 4 MHz, 59.17 Hz; Dragon Bowl: 10 MHz, 6 MHz, 60 Hz
static uint32_t s_cpu_clock, s_z80_clock, s_ym_clock, s_frame_mhz;
#define FRAME_MHZ s_frame_mhz
#define LINES 256                // set_size(32*8, 32*8)
#define VIS_Y0 s_vis_y0          // set_visarea(0, 255, 32, 255); Dragon Bowl 16 .. 239
static int s_vis_y0;
#define VIS_H 224
#define VIS_W 256

enum { M_GAIDEN, M_WILDFANG, M_RAIGA, M_DRGNBOWL };

static int s_mach, s_sprite_sizey;
static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static uint16_t (*s_ram_hp)[0x2000];          // 0x060000 .. 0x063fff
#define s_ram (*s_ram_hp)
static uint16_t s_txram[0x800];         // 0x070000 .. 0x070fff
static uint16_t s_fgram[0x1000];        // 0x072000 .. 0x073fff
static uint16_t s_bgram[0x1000];        // 0x074000 .. 0x075fff
static uint16_t s_spriteram[0x1000];    // 0x076000 .. 0x077fff
static uint16_t s_spritebuf[0x1000], s_spritedraw[0x1000];   // (Raiga: the buffer, and what the vblank drew from it)
static uint16_t s_pal[0x1000];          // 0x078000 .. 0x079fff
// scroll registers: [0] text, [1] FG, [2] BG; the offsets as MAME's scrolldx / scrolldy (normal, flipped)
static uint16_t s_sx[3], s_sy[3];
static int s_dx[3][2], s_dy[3][2];
static uint16_t s_spr_offset_y;
static uint8_t s_flip;
static uint8_t s_prot, s_jumpcode, s_protmode;
static const int *s_jumppoints;
static rbx_gfx s_gfxtx, s_gfxbg, s_gfxfg, s_gfxspr;
static int s_port[3];
static uint64_t s_cpu_frac, s_z80_frac, s_ym_frac;
static tecmo_mix_t s_mix_cfg;
static uint16_t (*s_sprbmp_hp)[VIS_W * VIS_H];
#define s_sprbmp (*s_sprbmp_hp)
static struct {
	uint16_t tx[0x800], fg[0x1000], bg[0x1000], spr[0x1000], pal[0x1000];
	uint16_t sx[3], sy[3];
	int dx[3][2], dy[3][2];
	uint16_t spr_offset_y;
	uint8_t flip;
} *s_snap;

// ---- the MCU simulations (protection_w / protection_r) ----
static const int wildfang_jumppoints[] = {
	0x0c0c, 0x0cac, 0x0d42, 0x0da2, 0x0eea, 0x112e, 0x1300, 0x13fa,
	0x159a, 0x1630, 0x109a, 0x1700, 0x1750, 0x1806, 0x18d6, 0x1a44,
	0x1b52
};
static const int raiga_jumppoints_00[0x100] = {
	0x6669,    -1,    -1,    -1,    -1,    -1,    -1,    -1,
	    -1,    -1,    -1,    -1,    -1,    -1,0x4a46,    -1,
	    -1,0x6704,    -2,    -1,    -1,    -1,    -1,    -1,
	    -1,    -2,    -1,    -1,    -1,    -1,    -1,    -1,
	    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,
	    -2,    -1,    -1,    -1,    -1,0x4e75,    -1,    -1,
	    -1,    -2,    -1,0x4e71,0x60fc,    -1,0x7288,    -1,
	    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1
};
static const int raiga_jumppoints_other[0x100] = {
	0x5457,0x494e,0x5f4b,0x4149,0x5345,0x525f,0x4d49,0x5941,
	0x5241,0x5349,0x4d4f,0x4a49,    -1,    -1,    -1,    -1,
	    -1,    -1,    -2,0x594f,    -1,0x4e75,    -1,    -1,
	    -1,    -2,    -1,    -1,0x4e75,    -1,0x5349,    -1,
	    -1,    -1,    -1,0x4e75,    -1,0x4849,    -1,    -1,
	    -2,    -1,    -1,0x524f,    -1,    -1,    -1,    -1,
	    -1,    -2,    -1,    -1,    -1,    -1,    -1,    -1,
	    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1
};

static void protection_w(uint8_t data)   // (the upper byte)
{
	switch (data & 0xf0) {
	case 0x00: s_prot = 0x00; break;
	case 0x10:
		s_jumpcode = (uint8_t)((data & 0x0f) << 4);
		s_prot = 0x10;
		break;
	case 0x20:
		s_jumpcode |= data & 0x0f;
		if (s_mach == M_WILDFANG) {
			if (s_jumpcode >= sizeof(wildfang_jumppoints) / sizeof(wildfang_jumppoints[0]))
				s_jumpcode = 0;
		} else {
			if (s_jumppoints[s_jumpcode] == -2) {
				s_protmode = 1;
				s_jumppoints = raiga_jumppoints_other;
			}
			if (s_jumppoints[s_jumpcode] == -1)
				s_jumpcode = 0;
		}
		s_prot = 0x20;
		break;
	case 0x30: s_prot = (uint8_t)(0x40 | ((s_jumppoints[s_jumpcode] >> 12) & 0x0f)); break;
	case 0x40: s_prot = (uint8_t)(0x50 | ((s_jumppoints[s_jumpcode] >> 8) & 0x0f)); break;
	case 0x50: s_prot = (uint8_t)(0x60 | ((s_jumppoints[s_jumpcode] >> 4) & 0x0f)); break;
	case 0x60: s_prot = (uint8_t)(0x70 | ((s_jumppoints[s_jumpcode] >> 0) & 0x0f)); break;
	}
}

// ---- sound ----
static z80 s_z80;
static uint8_t s_zram[0x800];
static uint8_t s_latch;
static int s_latch_pending;
static okim6295_t s_oki;
static int s_ym_irq;
static uint32_t s_oki_acc;
static int32_t s_oki_hold;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static const uint8_t *s_zmap[256];

static void ym_irq(int state) { s_ym_irq = state; }

static uint8_t z80_read(void *u, uint16_t a)
{
	(void)u;
	if (s_mach == M_DRGNBOWL) {   // 0x0000 .. 0xf7ff ROM, RAM
		if (a < 0xf800) return a < s_zrom_size ? s_zrom[a] : 0;
		return s_zram[a - 0xf800];
	}
	if (a < 0xf000) return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0xf800) return s_zram[a - 0xf000];
	if (a == 0xf800) return oki_read(&s_oki);
	if (a == 0xfc20) {
		s_latch_pending = 0;
		return s_latch;
	}
	return 0;
}
static void z80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_mach == M_DRGNBOWL) {
		if (a >= 0xf800) s_zram[a - 0xf800] = v;
		return;
	}
	if (a >= 0xf000 && a < 0xf800) s_zram[a - 0xf000] = v;
	else if (a == 0xf800) oki_write(&s_oki, v);
	else if (a == 0xf810 || a == 0xf811) rbx_ym_write(0, a & 1, v);
	else if (a == 0xf820 || a == 0xf821) rbx_ym_write(1, a & 1, v);
}
static uint8_t z80_in(z80 *z, uint8_t port)   // Dragon Bowl's I/O map (global_mask 0xff)
{
	(void)z;
	if (s_mach != M_DRGNBOWL) return 0xff;
	if (port == 0x00 || port == 0x01) return rbx_ym_read(0, port & 1);
	if (port == 0x80) return oki_read(&s_oki);
	if (port == 0xc0) {
		s_latch_pending = 0;   // (its INT line)
		return s_latch;
	}
	return 0xff;
}
static void z80_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	if (s_mach != M_DRGNBOWL) return;
	if (port == 0x00 || port == 0x01) rbx_ym_write(0, port & 1, v);
	else if (port == 0x80) oki_write(&s_oki, v);
}

static void z80_start(void)
{
	z80_init(&s_z80);
	s_z80.read_byte = z80_read;
	s_z80.write_byte = z80_write;
	s_z80.port_in = z80_in;
	s_z80.port_out = z80_out;
	s_z80.read_map = s_zmap;
}

static void sound_line(void)
{
	const int cycles = rbx_line_cycles(&s_z80_frac, s_z80_clock, FRAME_MHZ, LINES);
	z80_run(&s_z80, s_z80.cyc + (unsigned long)cycles, s_mach == M_DRGNBOWL ? &s_latch_pending : &s_ym_irq, 0);
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&s_ym_frac, s_ym_clock, FRAME_MHZ, LINES);
	const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	if (s_mach != M_DRGNBOWL)
		rbx_ym_advance(1, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	const uint32_t yr = rbx_ym_rate(0);
	for (int i = 0; i < n; i++) {
		s_oki_acc += s_oki.rate;
		while (s_oki_acc >= yr) {
			s_oki_acc -= yr;
			s_oki_hold = oki_sample(&s_oki);
		}
		s_mix[s_mix_n + i] += s_oki_hold;
	}
	s_mix_n += n;
}

static void sound_frame_end(void)
{
	rbx_ym_async_take(0, s_mix, s_mix_n);
	if (s_mach != M_DRGNBOWL)
		rbx_ym_async_take(1, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

// ---- bus ----
static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x07a000 && a < 0x07a006)   // SYSTEM, P1_P2, DSW
		return (uint16_t)rbx_port_read(s_port[(a - 0x07a000) >> 1]);
	if (a == 0x07a006 && s_mach != M_GAIDEN)
		return s_prot;
	return 0;
}
static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (s_mach == M_DRGNBOWL) {
		if (a == 0x07a00e && (mask & 0xff00)) {   // the sound latch (on the Z80's INT)
			s_latch = (uint8_t)(v >> 8);
			s_latch_pending = 1;
		} else if (a == 0x07e000 && (mask & 0xff00))
			m68k_bus_set_line(5, 0);
		else if (a >= 0x07f000 && a < 0x07f008) {   // BG y, BG x, FG y, FG x
			const int r = (int)((a >> 1) & 3), layer = r < 2 ? 2 : 1;
			if (r & 1)
				s_sx[layer] = v & 0x3ff;
			else
				s_sy[layer] = v & 0x1ff;
		}
		return;
	}
	if (a < 0x07a000 || a >= 0x07b000)
		return;
	const int layer = (int)((a >> 8) & 0xf) - 1;   // 0x07a1xx text, 0x07a2xx FG, 0x07a3xx BG
	if (layer >= 0 && layer < 3) {
		switch (a & 0xff) {
		case 0x00:   // offset x
			if (layer == 0) {
				s_dx[0][0] = 0xb2 - (v & 0xff);
				s_dx[0][1] = 0x4d - (v & 0xff);
			} else {
				s_dx[layer][0] = 0x3b2 - (v & 0x3ff);
				s_dx[layer][1] = 0x14d - (v & 0x3ff);
			}
			break;
		case 0x04: s_sy[layer] = v & 0x1ff; break;
		case 0x08:   // offset y
			if (layer == 0) {
				s_dy[0][0] = v & 0xff;
				s_dy[0][1] = 0xff - (v & 0xff);
			} else {
				s_dy[layer][0] = 0x20 - (v & 0x1ff);
				s_dy[layer][1] = 0x2ff - (v & 0x1ff);
			}
			break;
		case 0x0c: s_sx[layer] = layer ? (v & 0x3ff) : (v & 0xff); break;
		}
		return;
	}
	switch (a) {
	case 0x07a002:   // sproffsety_w
		if (mask & 0x00ff)
			s_spr_offset_y = v;
		break;
	case 0x07a802:   // the sound latch (umask 0x00ff, cswidth 16: a byte write smears its byte over the word)
		s_latch = (uint8_t)(mask == 0xff00 ? v >> 8 : v);
		if (!s_latch_pending)
			z80_gen_nmi(&s_z80);
		s_latch_pending = 1;
		break;
	case 0x07a804:
		if (s_mach != M_GAIDEN && (mask & 0xff00))
			protection_w((uint8_t)(v >> 8));
		break;
	case 0x07a806: m68k_bus_set_line(5, 0); break;   // irq_ack_w
	case 0x07a808:
		if (mask & 0x00ff)
			s_flip = v & 1;
		break;
	}
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
// the tilemap's coordinate for screen pixel v: MAME's effective scroll with scrolldx / scrolldy (flip: around extent)
static inline uint32_t tm_coord(int v, int extent, int scroll, const int *d, int flip, uint32_t mask)
{
	return (uint32_t)(flip ? extent - 1 - v - d[1] + scroll : v + scroll - d[0]) & mask;
}

// one line of a 16x16 layer (64x32 tiles: 1024x512), opaque: pixels as MAME's bitmap (color * 16 + pen)
static void tile16_line(int layer, int y, uint16_t *dst)
{
	const uint16_t *vram = layer == 1 ? s_snap->fg : s_snap->bg;
	const rbx_gfx *gfx = layer == 1 ? &s_gfxfg : &s_gfxbg;
	const int flip = s_snap->flip;
	const uint32_t ty = tm_coord(y, 288, s_snap->sy[layer], s_snap->dy[layer], flip, 511);
	const uint32_t row = (ty >> 4) * 64, fy = ty & 15;
	uint32_t tx = tm_coord(0, 256, s_snap->sx[layer], s_snap->dx[layer], flip, 1023);
	int x = 0;
	while (x < VIS_W) {   // (a tile's span at a time: the map runs right, or left when flipped)
		const uint32_t i = row + (tx >> 4);
		const uint32_t code = rbx_wrap((vram[0x800 + i] & 0x0fffu), gfx->count);
		uint32_t color = (vram[i] & 0xf0u) >> 4;
		if (layer == 1 && (vram[i] & 0x08))
			color += 0x10;
		const uint16_t c0 = (uint16_t)(color * 16);
		const uint8_t *src = gfx->pix + code * 256 + fy * 16;
		int px = (int)(tx & 15), n = flip ? px + 1 : 16 - px;
		if (n > VIS_W - x)
			n = VIS_W - x;
		if (flip)
			for (int k = 0; k < n; k++)
				dst[x + k] = (uint16_t)(c0 + src[px - k]);
		else
			for (int k = 0; k < n; k++)
				dst[x + k] = (uint16_t)(c0 + src[px + k]);
		x += n;
		tx = (tx + (uint32_t)(flip ? -n : n)) & 1023;
	}
}

static void text_line(int y, uint16_t *dst)   // 32x32 of 8x8, pen 0 transparent (the bitmap stays 0)
{
	const int flip = s_snap->flip;
	const uint32_t ty = tm_coord(y, 288, s_snap->sy[0], s_snap->dy[0], flip, 255);
	const uint32_t row = (ty >> 3) * 32, fy = ty & 7;
	uint32_t tx = tm_coord(0, 256, s_snap->sx[0], s_snap->dx[0], flip, 255);
	int x = 0;
	while (x < VIS_W) {
		const uint32_t i = row + (tx >> 3);
		const uint32_t code = rbx_wrap((s_snap->tx[0x400 + i] & 0x07ffu), s_gfxtx.count);
		int px = (int)(tx & 7), n = flip ? px + 1 : 8 - px;
		if (n > VIS_W - x)
			n = VIS_W - x;
		if (!s_gfxtx.used[code])
			memset(dst + x, 0, (size_t)n * 2);
		else {
			const uint16_t c0 = (uint16_t)(0x100 + ((s_snap->tx[i] & 0xf0u) >> 4) * 16);
			const uint8_t *src = s_gfxtx.pix + code * 64 + fy * 8;
			for (int k = 0; k < n; k++) {
				const uint8_t p = src[flip ? px - k : px + k];
				dst[x + k] = p ? (uint16_t)(c0 + p) : 0;
			}
		}
		x += n;
		tx = (tx + (uint32_t)(flip ? -n : n)) & 255;
	}
}

// Dragon Bowl (drgnbowl_state::screen_update): indexed layers over a priority bitmap, no mixer
static uint16_t (*s_dbbm_hp)[256 * 224];
#define s_dbbm (*s_dbbm_hp)
static uint8_t (*s_dbpri_hp)[256 * 224];
#define s_dbpri (*s_dbpri_hp)

#define DB_BANDS 8
static void db_layer_band(int band)   // (the layers' lines apart; the sprites after them all)
{
	for (int y = VIS_H * band / DB_BANDS; y < VIS_H * (band + 1) / DB_BANDS; y++) {
		const int by = y + VIS_Y0;
		uint16_t *d = s_dbbm + y * VIS_W;
		uint8_t *pr = s_dbpri + y * VIS_W;
		const uint32_t vb = (uint32_t)(by + s_snap->sy[2]) & 511, vf = (uint32_t)(by + s_snap->sy[1]) & 511;
		const uint32_t vt = (uint32_t)by & 255;
		for (int x = 0; x < VIS_W; x++) {   // BG (opaque, priority 1), set_scrolldx(-248)
			const uint32_t u = (uint32_t)(x + s_snap->sx[2] + 248) & 1023, i = (vb >> 4) * 64 + (u >> 4);
			const uint32_t code = rbx_wrap((s_snap->bg[0x800 + i] & 0x0fffu), s_gfxbg.count);
			d[x] = (uint16_t)(0x300 + ((s_snap->bg[i] & 0xf0u) >> 4) * 16 + s_gfxbg.pix[code * 256 + (vb & 15) * 16 + (u & 15)]);
			pr[x] = 1;
		}
		for (int x = 0; x < VIS_W; x++) {   // FG (pen 15 transparent, priority 2), set_scrolldx(-252)
			const uint32_t u = (uint32_t)(x + s_snap->sx[1] + 252) & 1023, i = (vf >> 4) * 64 + (u >> 4);
			const uint32_t code = rbx_wrap((s_snap->fg[0x800 + i] & 0x0fffu), s_gfxfg.count);
			const uint8_t p = s_gfxfg.pix[code * 256 + (vf & 15) * 16 + (u & 15)];
			if (p != 15) {
				d[x] = (uint16_t)(0x200 + ((s_snap->fg[i] & 0xf0u) >> 4) * 16 + p);
				pr[x] |= 2;
			}
		}
		for (int x = 0; x < VIS_W; x++) {   // text (pen 15 transparent, priority 4)
			const uint32_t i = (vt >> 3) * 32 + ((uint32_t)x >> 3);
			const uint32_t code = rbx_wrap((s_snap->tx[0x400 + i] & 0x07ffu), s_gfxtx.count);
			const uint8_t p = s_gfxtx.pix[code * 64 + (vt & 7) * 8 + (x & 7)];
			if (p != 15) {
				d[x] = (uint16_t)(((s_snap->tx[i] & 0xf0u) >> 4) * 16 + p);
				pr[x] |= 4;
			}
		}
	}
}
static decoband_t s_dbands = { 0, 0, 0, 0, DB_BANDS, db_layer_band };

static void db_draw(void)
{
	decoband_run(&s_dbands);
	const uint16_t *spr = s_snap->spr;   // (live)
	for (int i = 0; i < 0x800 / 2; i += 4) {
		const uint32_t code = rbx_wrap(((spr[i + 0] & 0xffu) | (spr[i + 3] & 0x1fu) << 8), s_gfxspr.count);
		const int y = 256 - (spr[i + 1] & 0xff) - 12 - VIS_Y0;
		int x = spr[i + 2] & 0xff;
		const uint16_t c0 = (uint16_t)(0x100 + (spr[0x400 + i] & 0x0f) * 16);
		const int flipx = (spr[i + 3] >> 6) & 1, flipy = (spr[i + 3] >> 7) & 1;
		if (spr[0x400 + i] & 0x80)
			x -= 256;
		x += 256;
		const uint32_t pmask = ((spr[i + 3] & 0x20) ? 0xf0u | 0xccu : 0u) | 0x80000000u;   // (obscured by the FG)
		const uint8_t *g = s_gfxspr.pix + code * 256;
		for (int k = 0; k < 2; k++) {
			const int sx = k ? x - 512 : x;
			if (sx >= VIS_W || sx + 16 <= 0 || y >= VIS_H || y + 16 <= 0)
				continue;
			for (int j = 0; j < 16; j++) {
				const int yy = y + j;
				if ((unsigned)yy >= VIS_H)
					continue;
				const uint8_t *src = g + (flipy ? 15 - j : j) * 16;
				for (int c = 0; c < 16; c++) {
					const int xx = sx + c;
					if ((unsigned)xx >= VIS_W)
						continue;
					const uint8_t p = src[flipx ? 15 - c : c];
					if (p == 15)
						continue;
					uint8_t *pp = &s_dbpri[yy * VIS_W + xx];
					if (!((1u << (*pp & 0x1f)) & pmask))
						s_dbbm[yy * VIS_W + xx] = (uint16_t)(c0 + p);
					*pp = 31;
				}
			}
		}
	}
}

static int s_565;

#define BANDS 8
static uint32_t (*s_pal_rgb_hp)[0x1000];
#define s_pal_rgb (*s_pal_rgb_hp)

static void gd_band(int band)
{
	const int r0 = VIS_H * band / BANDS, r1 = VIS_H * (band + 1) / BANDS;
	const int db = s_mach == M_DRGNBOWL;
	tecmo_mix_t mix = s_mix_cfg;
	mix.rand = (uint32_t)band * 0x9e3779b9u;   // (each band its own: the same picture with any number of threads)
	for (int y = r0; y < r1; y++) {
		uint16_t bg[VIS_W], fg[VIS_W], tx[VIS_W];
		uint32_t d[VIS_W];
		if (db) {
			for (int x = 0; x < VIS_W; x++)
				d[x] = s_pal_rgb[s_dbbm[y * VIS_W + x] & 0xfff];
		} else {
			tile16_line(2, y + VIS_Y0, bg);
			tile16_line(1, y + VIS_Y0, fg);
			text_line(y + VIS_Y0, tx);
			tecmo_mix_line(&mix, s_pal_rgb, bg, fg, tx, s_sprbmp + y * VIS_W, d, VIS_W);
		}
		uint8_t *o = (uint8_t *)rbx_video.out + (size_t)y * rbx_video.out_pitch;
		if (s_565)
			for (int x = 0; x < VIS_W; x++)
				((uint16_t *)o)[x] = (uint16_t)((d[x] >> 8 & 0xf800) | (d[x] >> 5 & 0x07e0) | (d[x] >> 3 & 0x001f));
		else if (rbx_video.out_bpp == 4)
			for (int x = 0; x < VIS_W; x++)
				((uint32_t *)o)[x] = rbx_video.out_color((int)(d[x] >> 16) & 0xff, (int)(d[x] >> 8) & 0xff, (int)d[x] & 0xff);
		else
			for (int x = 0; x < VIS_W; x++)
				((uint16_t *)o)[x] = (uint16_t)rbx_video.out_color((int)(d[x] >> 16) & 0xff, (int)(d[x] >> 8) & 0xff, (int)d[x] & 0xff);
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, gd_band };
static int gd_help(int helper) { return decoband_help(&s_dbands, helper) || decoband_help(&s_bands, helper); }

static void gd_render(void)
{
	if (!rbx_video.out)
		return;
	for (int i = 0; i < 0x1000; i++) {   // xBGR_444
		const uint16_t p = s_snap->pal[i];
		s_pal_rgb[i] = (uint32_t)(((p & 15) * 17) << 16 | (((p >> 4) & 15) * 17) << 8 | (((p >> 8) & 15) * 17));
	}
	s_565 = rbx_video.out_bpp == 2 && rbx_video.out_color(0xff, 0, 0) == 0xf800 && rbx_video.out_color(0, 0xff, 0) == 0x07e0 &&
	        rbx_video.out_color(0, 0, 0xff) == 0x001f;
	if (s_mach == M_DRGNBOWL)
		db_draw();
	else
		tecmo_spr_gaiden_draw(&s_gfxspr, s_snap->spr, s_sprite_sizey,
		                      s_snap->flip ? -(int)(int8_t)s_snap->spr_offset_y : (int)(int8_t)s_snap->spr_offset_y, s_snap->flip,
		                      256, s_sprbmp, VIS_W, VIS_H, VIS_Y0);
	decoband_run(&s_bands);
	rbx_video.out_done = 1;
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->tx, s_txram, sizeof(s_txram));
	memcpy(s_snap->fg, s_fgram, sizeof(s_fgram));
	memcpy(s_snap->bg, s_bgram, sizeof(s_bgram));
	if (s_mach == M_RAIGA) {   // screen_vblank: the bitmap drawn from the buffer, shown the next frame; then copy()
		memcpy(s_snap->spr, s_spritedraw, sizeof(s_spritedraw));
		memcpy(s_spritedraw, s_spritebuf, sizeof(s_spritebuf));
		memcpy(s_spritebuf, s_spriteram, sizeof(s_spriteram));
	} else
		memcpy(s_snap->spr, s_spriteram, sizeof(s_spriteram));   // (live)
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	memcpy(s_snap->sx, s_sx, sizeof(s_sx));
	memcpy(s_snap->sy, s_sy, sizeof(s_sy));
	memcpy(s_snap->dx, s_dx, sizeof(s_dx));
	memcpy(s_snap->dy, s_dy, sizeof(s_dy));
	s_snap->spr_offset_y = s_spr_offset_y;
	s_snap->flip = s_flip;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void gd_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_txram, 0, sizeof(s_txram));
	memset(s_fgram, 0, sizeof(s_fgram));
	memset(s_bgram, 0, sizeof(s_bgram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_spritebuf, 0, sizeof(s_spritebuf));
	memset(s_spritedraw, 0, sizeof(s_spritedraw));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_sx, 0, sizeof(s_sx));
	memset(s_sy, 0, sizeof(s_sy));
	memset(s_dx, 0, sizeof(s_dx));
	memset(s_dy, 0, sizeof(s_dy));
	s_spr_offset_y = 0;
	s_flip = 0;
	s_prot = s_jumpcode = s_protmode = 0;
	s_jumppoints = s_mach == M_WILDFANG ? wildfang_jumppoints : raiga_jumppoints_00;
	s_mix_cfg.rand = 0;
	s_latch = 0;
	s_latch_pending = 0;
	s_ym_irq = 0;
	memset(s_zram, 0, sizeof(s_zram));
	z80_start();
	rbx_ym_reset(0);
	if (s_mach != M_DRGNBOWL)
		rbx_ym_reset(1);
	oki_reset(&s_oki);
	s_oki_acc = 0;
	s_oki_hold = 0;
	s_mix_n = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_cpu_frac = s_z80_frac = s_ym_frac = 0;
}

static void gd_exit(void);

// bitswap<24>(i, 23, 22, 21, 20, 19, 18, 16, 17, 15, 14, 13, 4, 3, 12, 11, 10, 9, 8, 7, 6, 5, 2, 1, 0)
static uint32_t tile_swap(uint32_t i)
{
	static const int src[24] = { 23, 22, 21, 20, 19, 18, 16, 17, 15, 14, 13, 4, 3, 12, 11, 10, 9, 8, 7, 6, 5, 2, 1, 0 };
	uint32_t a = 0;
	for (int k = 0; k < 24; k++)
		a |= ((i >> src[k]) & 1u) << (23 - k);
	return a;
}

static int gd_init(const rbx_game *g)
{
	if (!s_dbbm_hp && !(s_dbbm_hp = calloc(1, sizeof(*s_dbbm_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_dbpri_hp && !(s_dbpri_hp = calloc(1, sizeof(*s_dbpri_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_pal_rgb_hp && !(s_pal_rgb_hp = calloc(1, sizeof(*s_pal_rgb_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ram_hp && !(s_ram_hp = calloc(1, sizeof(*s_ram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_sprbmp_hp && !(s_sprbmp_hp = calloc(1, sizeof(*s_sprbmp_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	const char *m = g->machine;
	s_mach = strstr(m, "raiga") ? M_RAIGA : strstr(m, "wildfang") ? M_WILDFANG : strstr(m, "drgnbowl") ? M_DRGNBOWL : M_GAIDEN;
	const int db = s_mach == M_DRGNBOWL;
	s_cpu_clock = db ? 10000000 : 9216000;
	s_z80_clock = db ? 6000000 : 4000000;
	s_ym_clock = 4000000;   // (Dragon Bowl's YM2151: 12 MHz / 3)
	s_frame_mhz = db ? 60000 : 59170;
	s_vis_y0 = db ? 16 : 32;
	s_sprite_sizey = s_mach == M_RAIGA ? 2 : 0;
	uint32_t txs, bgs, fgs, sps, osize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *txt = rbx_region("txtiles", &txs), *bgt = rbx_region("bgtiles", &bgs);
	uint8_t *fgt = db ? bgt : rbx_region("fgtiles", &fgs);
	if (db)
		fgs = bgs;
	uint8_t *spr = rbx_region("sprites", &sps), *oki = rbx_region("oki", &osize);
	if (!s_rom || s_rom_size < 0x40000 || !s_zrom || !txt || !bgt || !fgt || !spr || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (db) {   // descramble_drgnbowl: the program (set 1: A15 / A16 swapped), the tiles; the sprites inverted
		uint8_t *buf = (uint8_t *)malloc(bgs > s_rom_size ? bgs : s_rom_size);
		if (!buf) {
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
		if (!strstr(m, "drgnbowla")) {
			memcpy(buf, s_rom, s_rom_size);
			for (uint32_t i = 0; i < s_rom_size; i++)
				s_rom[i] = buf[(i & ~0x18000u) | (i >> 1 & 0x8000u) | (i << 1 & 0x10000u)];
		}
		memcpy(buf, bgt, bgs);
		for (uint32_t i = 0; i < bgs; i++)
			bgt[i] = buf[tile_swap(i)];
		free(buf);
		for (uint32_t i = 0; i < sps; i++)
			spr[i] ^= 0xff;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	if (db) {	// gfx_8x8x4_packed_msb, drgnbowl_tile2layout, drgnbowl_spritelayout
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)(i * 4);
			y8[i] = (uint32_t)(i * 32);
			x16[i] = (uint32_t)i;
			x16[8 + i] = (uint32_t)(8 * 8 * 2 + i);
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 8);
		static const uint32_t pl[4] = { 0, 1, 2, 3 };
		const uint32_t q = bgs * 2, r = sps * 2;   // (RGN_FRAC(1, 4) of the regions, in bits)
		const uint32_t pt[4] = { q * 3, q * 2, q, 0 }, ps[4] = { 0, r, r * 2, r * 3 };
		const rbx_gfx_layout l8 = { 8, 8, 4, pl, x8, y8, 32 * 8 };
		const rbx_gfx_layout lt = { 16, 16, 4, pt, x16, y16, 32 * 8 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, x16, y16, 32 * 8 };
		if (rbx_gfx_decode(&s_gfxtx, &l8, txt, txs) || rbx_gfx_decode(&s_gfxbg, &lt, bgt, bgs / 8) ||
		    rbx_gfx_decode(&s_gfxfg, &lt, bgt + 0x20000, bgs / 8) || rbx_gfx_decode(&s_gfxspr, &ls, spr, sps / 4)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			gd_exit();
			return -1;
		}
	} else {	// gfx_8x8x4_packed_msb, gfx_8x8x4_row_2x2_group_packed_msb
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)(i * 4);
			y8[i] = (uint32_t)(i * 32);
			x16[i] = (uint32_t)(i * 4);
			x16[8 + i] = (uint32_t)(256 + i * 4);
			y16[i] = (uint32_t)(i * 32);
			y16[8 + i] = (uint32_t)(512 + i * 32);
		}
		static const uint32_t pl[4] = { 0, 1, 2, 3 };
		const rbx_gfx_layout l8 = { 8, 8, 4, pl, x8, y8, 32 * 8 };
		const rbx_gfx_layout l16 = { 16, 16, 4, pl, x16, y16, 128 * 8 };
		if (rbx_gfx_decode(&s_gfxtx, &l8, txt, txs) || rbx_gfx_decode(&s_gfxbg, &l16, bgt, bgs) ||
		    rbx_gfx_decode(&s_gfxfg, &l16, fgt, fgs) || rbx_gfx_decode(&s_gfxspr, &l8, spr, sps)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			gd_exit();
			return -1;
		}
	}
	memset(&s_mix_cfg, 0, sizeof(s_mix_cfg));   // the shadoww machine's mixer
	s_mix_cfg.sprpri_shift = 10;
	s_mix_cfg.sprbln_shift = 9;
	s_mix_cfg.sprcol_shift = 4;
	s_mix_cfg.bgblend_comp = 0x0400 + 0x300;
	s_mix_cfg.fgblend_comp = 0x0400 + 0x200;
	s_mix_cfg.txblend_comp = 0x0400 + 0x100;
	s_mix_cfg.spblend_comp = 0x0400 + 0x000;
	s_mix_cfg.bgregular_comp = 0x300;
	s_mix_cfg.fgregular_comp = 0x200;
	s_mix_cfg.txregular_comp = 0x100;
	s_mix_cfg.spregular_comp = 0x000;
	s_mix_cfg.spblend_source = 0x0800 + 0x000;
	s_mix_cfg.fgblend_source = 0x0800 + 0x200;
	s_mix_cfg.revspritetile = 3;
	s_mix_cfg.bgpen = 0x000 + 0x200;
	s_mix_cfg.bgpen_blend = 0x400 + 0x200;
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;   // two YM2203s at 4 MHz: SSG 0.15 each, FM 0.60; Dragon Bowl: a YM2151 0.40 (its IRQ not wired)
	memset(&yc, 0, sizeof(yc));
	yc.clock = s_ym_clock;
	if (db) {
		yc.type = RBX_YM2151;
		yc.gain[0] = yc.gain[1] = 102;
	} else {
		yc.type = RBX_YM2203;
		yc.gain[0] = yc.gain[1] = yc.gain[2] = 38;
		yc.gain[3] = 154;
		yc.irq = ym_irq;
	}
	int bad = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	yc.irq = NULL;
	if (!bad && !db)
		bad = rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	if (bad) {
		rbx_fail("%s: out of memory", g->set);
		gd_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 1)) {
		rbx_fail("%s: out of memory", g->set);
		gd_exit();
		return -1;
	}
	oki_init(&s_oki, 1000000, 1, oki, osize, db ? 128 : 51);
	memset(s_zmap, 0, sizeof(s_zmap));
	for (uint32_t p = 0; p < (db ? 0xf8u : 0xf0u) && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	static const char *const names[3] = { "SYSTEM", "P1_P2", "DSW" };
	for (int i = 0; i < 3; i++)
		s_port[i] = rbx_port_find(names[i]);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);   // (MAME's DIVx / MULx / ADDQ.W An times)
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x3ffff, s_rom, 0x40000, 1, 0);
	m68k_bus_map(0x060000, 0x063fff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x070000, 0x070fff, s_txram, sizeof(s_txram), 1, 1);
	m68k_bus_map(0x072000, 0x073fff, s_fgram, sizeof(s_fgram), 1, 1);
	m68k_bus_map(0x074000, 0x075fff, s_bgram, sizeof(s_bgram), 1, 1);
	m68k_bus_map(0x076000, 0x077fff, s_spriteram, sizeof(s_spriteram), 1, 1);
	m68k_bus_map(0x078000, 0x079fff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	gd_reset();
	return 0;
}

static void gd_exit(void)
{
	m68k_set_mame_timing(0);
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gfxtx);
	rbx_gfx_free(&s_gfxbg);
	rbx_gfx_free(&s_gfxfg);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
	free(s_dbbm_hp);
	s_dbbm_hp = NULL;
	free(s_dbpri_hp);
	s_dbpri_hp = NULL;
	free(s_pal_rgb_hp);
	s_pal_rgb_hp = NULL;
	free(s_ram_hp);
	s_ram_hp = NULL;
	free(s_sprbmp_hp);
	s_sprbmp_hp = NULL;
}

static void gd_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	const int vbl = VIS_Y0 + VIS_H;   // (vblank time 0: at the end of the visible area; Ninja Gaiden's is the frame's end)
	for (int line = 0; line < LINES; line++) {
		if (line == vbl) {   // the frame drawn, IRQ 5 (irq5_line_assert)
			vblank_capture();
			m68k_bus_set_line(5, 1);
		}
		m68k_execute(rbx_line_cycles(&s_cpu_frac, s_cpu_clock, FRAME_MHZ, LINES));
		sound_line();
	}
	if (vbl >= LINES) {
		vblank_capture();
		m68k_bus_set_line(5, 1);
	}
	sound_frame_end();
}

static int gd_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[3] = { s_cpu_frac, s_z80_frac, s_ym_frac };
	int32_t regs[24];
	for (int i = 0; i < 3; i++) {
		regs[i] = s_sx[i];
		regs[3 + i] = s_sy[i];
		regs[6 + i] = s_dx[i][0];
		regs[9 + i] = s_dx[i][1];
		regs[12 + i] = s_dy[i][0];
		regs[15 + i] = s_dy[i][1];
	}
	regs[18] = s_spr_offset_y;
	regs[19] = s_flip;
	regs[20] = s_prot;
	regs[21] = s_jumpcode;
	regs[22] = s_protmode;
	regs[23] = (int32_t)s_mix_cfg.rand;
	int32_t misc[5] = { s_latch, s_latch_pending, s_ym_irq, (int32_t)s_oki_acc, s_oki_hold };
	z80 z = s_z80;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "tx", s_txram, sizeof(s_txram));
	rc |= rbx_state_io(ctx, "fg", s_fgram, sizeof(s_fgram));
	rc |= rbx_state_io(ctx, "bg", s_bgram, sizeof(s_bgram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "sprb", s_spritebuf, sizeof(s_spritebuf));
	rc |= rbx_state_io(ctx, "sprd", s_spritedraw, sizeof(s_spritedraw));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "regs", regs, sizeof(regs));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= rbx_state_io(ctx, "oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	rc |= rbx_ym_state(0, ctx, load);
	if (s_mach != M_DRGNBOWL)
		rc |= rbx_ym_state(1, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		z.read_byte = z80_read;
		z.write_byte = z80_write;
		z.port_in = z80_in;
		z.port_out = z80_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		s_z80 = z;
		s_cpu_frac = fr[0];
		s_z80_frac = fr[1];
		s_ym_frac = fr[2];
		for (int i = 0; i < 3; i++) {
			s_sx[i] = (uint16_t)regs[i];
			s_sy[i] = (uint16_t)regs[3 + i];
			s_dx[i][0] = regs[6 + i];
			s_dx[i][1] = regs[9 + i];
			s_dy[i][0] = regs[12 + i];
			s_dy[i][1] = regs[15 + i];
		}
		s_spr_offset_y = (uint16_t)regs[18];
		s_flip = (uint8_t)regs[19];
		s_prot = (uint8_t)regs[20];
		s_jumpcode = (uint8_t)regs[21];
		s_protmode = (uint8_t)regs[22];
		s_jumppoints = s_mach == M_WILDFANG ? wildfang_jumppoints : s_protmode ? raiga_jumppoints_other : raiga_jumppoints_00;
		s_mix_cfg.rand = (uint32_t)regs[23];
		s_latch = (uint8_t)misc[0];
		s_latch_pending = misc[1];
		s_ym_irq = misc[2];
		s_oki_acc = (uint32_t)misc[3];
		s_oki_hold = misc[4];
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int gd_save(void *ctx) { return gd_state(ctx, 0); }
static int gd_load(void *ctx) { return gd_state(ctx, 1); }
static int gd_supported(const char *machine)
{
	return !strcmp(machine, "shadoww:shadoww") || !strcmp(machine, "wildfang:wildfang") || !strcmp(machine, "raiga:raiga") ||
	       !strncmp(machine, "drgnbowl:", 9);
}

const rbx_driver rbx_drv_gaiden = { "Tecmo Ninja Gaiden", gd_init, gd_exit, gd_reset, gd_frame, gd_save, gd_load,
                                    gd_supported, gd_render, gd_help };
