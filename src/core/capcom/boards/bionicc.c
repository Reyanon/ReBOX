/*  Capcom Bionic Commando / Top Secret. Ported from MAME src/mame/capcom/bionicc.cpp and tigeroad_spr.cpp
    (BSD-3-Clause, copyright-holders: Steven Frew, Phil Stroffolino, Paul Leaman).
    68000 at 12 MHz (IRQ 2 at line 240, IRQ 4 at line 128); an i8751 MCU copies the sprite list by DMA while it holds
    the 68000 halted, and passes the sound commands to the Z80 (3.58 MHz, YM2151, NMI from the 68000). Video: a text
    layer, an 8x8 BG and a 16x16 FG layer (categories and pen groups: parts of it behind the BG, parts over the
    sprites), 16x16 sprites from a buffer.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/mcs51/mcs51.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"
#include "../../video/decoband.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 12000000
#define Z80_CLOCK 3579545
#define YM_CLOCK 3579545
#define MCU_CLOCK (6000000 / 12)   // (machine cycles)
#define FRAME_MHZ 60096            // set_raw(24 MHz / 4, 384, 0, 256, 260, 16, 240)
#define LINES 260
#define VIS_Y0 16
#define VIS_W 256
#define VIS_H 224
#define SLICES 4
#define PAL_N 1024
#define BLACK PAL_N

static uint8_t *s_rom, *s_zrom, *s_mrom;
static uint32_t s_rom_size, s_zrom_size, s_mrom_size;
static uint16_t s_lowram[0x800];        // 0xe0000 .. 0xe0fff (0xe0800 .. 0xe0cff the sprites), mirror 0x3000
static uint16_t s_spritebuf[0x280];
static uint16_t s_txram[0x800];         // 0xec000 .. 0xecfff, mirror 0x3000
static uint16_t s_fgram[0x2000];        // 0xf0000 .. 0xf3fff
static uint16_t s_bgram[0x2000];        // 0xf4000 .. 0xf7fff
static uint16_t s_pal[0x400];           // 0xf8000 .. 0xf87ff (write only)
static uint16_t s_wram[0x2000];         // 0xfc000 .. 0xfffff
static uint16_t s_scroll[4];
static uint8_t s_flip, s_halted;
static uint8_t s_audiocpu_to_mcu, s_mcu_to_audiocpu, s_mcu_p1, s_mcu_p3;
static rbx_gfx s_gfxtx, s_gfxbg, s_gfxfg, s_gfxspr;
static int s_port[2];
static uint64_t s_cpu_frac, s_z80_frac, s_ym_frac, s_mcu_frac;
static mcs51_t s_mcu;
static struct {
	uint16_t spr[0x280], tx[0x800], fg[0x2000], bg[0x2000], pal[0x400], scroll[4];
	uint8_t flip;
} *s_snap;

// ---- sound ----
static z80 s_z80;
static uint8_t s_zram[0x800];
static int s_zirq;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static const uint8_t *s_zmap[256];

static uint8_t z80_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_zrom_size ? s_zrom[a] : 0;
	if (a == 0x8000 || a == 0x8001) return rbx_ym_read(0, a & 1);
	if (a == 0xa000) return s_mcu_to_audiocpu;
	if (a >= 0xc000 && a < 0xc800) return s_zram[a - 0xc000];
	return 0;
}
static void z80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a == 0x8000 || a == 0x8001) rbx_ym_write(0, a & 1, v);
	else if (a == 0xa000) s_audiocpu_to_mcu = v;
	else if (a >= 0xc000 && a < 0xc800) s_zram[a - 0xc000] = v;
}
static uint8_t z80_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z80_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

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
	const int cycles = rbx_line_cycles(&s_z80_frac, Z80_CLOCK, FRAME_MHZ, LINES);
	z80_run(&s_z80, s_z80.cyc + (unsigned long)cycles, &s_zirq, 0);   // (no IRQ wired: the YM's is not)
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&s_ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
	s_mix_n += rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
}

static void sound_frame_end(void)
{
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

// ---- the MCU ----
static uint8_t mcu_port_in(void *u, int port)
{
	(void)u;
	return port == 1 ? s_audiocpu_to_mcu : 0xff;
}
static void mcu_port_out(void *u, int port, uint8_t data)
{
	(void)u;
	if (port == 1)
		s_mcu_p1 = data;
	else if (port == 3) {   // mcu_p3_w
		if ((s_mcu_p3 & 0x01) && !(data & 0x01)) {
			mcs51_set_input(&s_mcu, MCS51_INT0_LINE, 0);
			s_halted = 0;   // (the 68000 resumes)
		}
		if ((s_mcu_p3 & 0x10) && !(data & 0x10))
			mcs51_set_input(&s_mcu, MCS51_INT1_LINE, 0);
		if ((s_mcu_p3 & 0x40) && !(data & 0x40))
			s_mcu_to_audiocpu = s_mcu_p1;
		s_mcu_p3 = data;
	}
}
static uint32_t dma_address(uint16_t offset)
{
	return 0xe3e01u | ((offset & 0x700u) << 6) | ((offset & 0xffu) << 1);
}
static uint8_t mcu_xdata_r(void *u, uint16_t a)
{
	(void)u;
	return (s_mcu_p3 & 0x20) ? 0xff : (uint8_t)m68k_read_memory_8(dma_address(a & 0x7ff));
}
static void mcu_xdata_w(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (!(s_mcu_p3 & 0x20))
		m68k_write_memory_8(dma_address(a & 0x7ff), v);
}

// ---- bus (global_mask 0xfffff) ----
static uint16_t main_read16(uint32_t a)
{
	a &= 0xfffff;
	if (a >= 0xe4000 && a < 0xe8000)
		return (uint16_t)rbx_port_read(s_port[(a >> 1) & 1]);   // INPUTS, DSW (mirror 0x3ffc)
	return 0;
}
static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xfffff;
	if (a >= 0xe4000 && a < 0xe8000) {
		if (mask & 0xff00) {
			if (a & 2)   // audiocpu_nmi_w
				z80_gen_nmi(&s_z80);
			else         // output_w
				s_flip = (v >> 8) & 1;
		}
	} else if (a >= 0xe8010 && a < 0xe8018) {
		uint16_t *w = &s_scroll[(a >> 1) & 3];
		*w = (uint16_t)((*w & ~mask) | (v & mask));
	} else if (a == 0xe8018)
		memcpy(s_spritebuf, &s_lowram[0x400], sizeof(s_spritebuf));   // (buffered_spriteram16 write: copy)
	else if (a == 0xe801a) {   // dmaon_w: the MCU's INT0, the 68000 halted until it lets go
		mcs51_set_input(&s_mcu, MCS51_INT0_LINE, 1);
		s_halted = 1;
		m68k_end_timeslice();
	} else if (a >= 0xf8000 && a < 0xf8800) {
		uint16_t *w = &s_pal[(a - 0xf8000) >> 1];
		*w = (uint16_t)((*w & ~mask) | (v & mask));
	}
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
// FG pen groups: set_transmask(0, 0xffff, 0x8000), (1, 0xffc1, 0x803e)
static const uint16_t s_fg_mask0[2] = { 0xffff, 0xffc1 }, s_fg_mask1[2] = { 0x8000, 0x803e };
static uint16_t s_frame[VIS_H][VIS_W];

static inline uint32_t tcoord(int v, int extent, uint32_t scroll, int flip, uint32_t mask)
{
	return (uint32_t)(flip ? extent - 1 - v + (int)scroll : v + (int)scroll) & mask;
}

// the FG (64x64 of 16x16): category, layer 0 or 1 (a tile's span at a time)
static void fg_line(int by, int category, int layer, uint16_t *d)
{
	const int flip = s_snap->flip;
	const uint32_t ty = tcoord(by, 256, s_snap->scroll[1], flip, 1023);
	uint32_t tx = tcoord(0, 256, s_snap->scroll[0], flip, 1023);
	int x = 0;
	while (x < VIS_W) {
		int px = (int)(tx & 15), n = flip ? px + 1 : 16 - px;
		if (n > VIS_W - x)
			n = VIS_W - x;
		const uint32_t i = (ty >> 4) * 64 + (tx >> 4);
		const uint16_t attr = s_snap->fg[2 * i + 1];
		int cat, group, fx = 0, fy = 0;
		if ((attr & 0xc0) == 0xc0) {
			cat = 1;
			group = 0;
		} else {
			cat = 0;
			group = (attr & 0x20) >> 5;
			fx = (attr >> 7) & 1;   // (TILE_FLIPXY)
			fy = (attr >> 6) & 1;
		}
		if (cat == category) {
			const uint32_t code = ((uint32_t)(attr & 0x07) << 8 | (s_snap->fg[2 * i] & 0xffu)) % s_gfxfg.count;
			const uint32_t py = fy ? 15 - (ty & 15) : (ty & 15);
			const uint8_t *src = s_gfxfg.pix + code * 256 + py * 16;
			const uint16_t m = layer ? s_fg_mask1[group] : s_fg_mask0[group];
			const uint16_t c0 = (uint16_t)(256 + ((attr & 0x18) >> 3) * 16);
			if (m != 0xffff)
				for (int k = 0; k < n; k++) {
					const int c = flip ? px - k : px + k;
					const uint8_t p = src[fx ? 15 - c : c];
					if (!((m >> p) & 1))
						d[x + k] = (uint16_t)(c0 + p);
				}
		}
		x += n;
		tx = (tx + (uint32_t)(flip ? -n : n)) & 1023;
	}
}

static void bg_line(int by, uint16_t *d)   // 64x64 of 8x8, pen 15 transparent
{
	const int flip = s_snap->flip;
	const uint32_t ty = tcoord(by, 256, s_snap->scroll[3], flip, 511);
	uint32_t tx = tcoord(0, 256, s_snap->scroll[2], flip, 511);
	int x = 0;
	while (x < VIS_W) {
		int px = (int)(tx & 7), n = flip ? px + 1 : 8 - px;
		if (n > VIS_W - x)
			n = VIS_W - x;
		const uint32_t i = (ty >> 3) * 64 + (tx >> 3);
		const uint16_t attr = s_snap->bg[2 * i + 1];
		const uint32_t code = ((uint32_t)(attr & 0x07) << 8 | (s_snap->bg[2 * i] & 0xffu)) % s_gfxbg.count;
		const uint32_t py = (attr & 0x40) ? 7 - (ty & 7) : (ty & 7);
		const uint8_t *src = s_gfxbg.pix + code * 64 + py * 8;
		const uint16_t c0 = (uint16_t)(((attr & 0x38) >> 3) * 16);
		const int fx = (attr & 0x80) != 0;
		for (int k = 0; k < n; k++) {
			const int c = flip ? px - k : px + k;
			const uint8_t p = src[fx ? 7 - c : c];
			if (p != 15)
				d[x + k] = (uint16_t)(c0 + p);
		}
		x += n;
		tx = (tx + (uint32_t)(flip ? -n : n)) & 511;
	}
}

static void tx_line(int by, uint16_t *d)   // 32x32 of 8x8, 2 bits, pen 3 transparent
{
	const int flip = s_snap->flip;
	const uint32_t ty = tcoord(by, 256, 0, flip, 255);
	uint32_t tx = tcoord(0, 256, 0, flip, 255);
	int x = 0;
	while (x < VIS_W) {
		int px = (int)(tx & 7), n = flip ? px + 1 : 8 - px;
		if (n > VIS_W - x)
			n = VIS_W - x;
		const uint32_t i = (ty >> 3) * 32 + (tx >> 3);
		const uint16_t attr = s_snap->tx[i + 0x400];
		const uint32_t code = (((uint32_t)(attr & 0xc0) << 2) | (s_snap->tx[i] & 0xffu)) % s_gfxtx.count;
		const uint32_t py = (attr & 0x20) ? 7 - (ty & 7) : (ty & 7);
		const uint8_t *src = s_gfxtx.pix + code * 64 + py * 8;
		const uint16_t c0 = (uint16_t)(768 + (attr & 0xf) * 4);
		const int fx = (attr & 0x10) != 0;
		for (int k = 0; k < n; k++) {
			const int c = flip ? px - k : px + k;
			const uint8_t p = src[fx ? 7 - c : c];
			if (p != 3)
				d[x + k] = (uint16_t)(c0 + p);
		}
		x += n;
		tx = (tx + (uint32_t)(flip ? -n : n)) & 255;
	}
}

static void draw_sprites(void)   // tigeroad_spr_device::draw_sprites (from the end, pen 15 transparent)
{
	for (int o = 0x280 - 4; o >= 0; o -= 4) {
		const uint16_t *s = s_snap->spr + o;
		const uint32_t code = s[0] % s_gfxspr.count;
		const uint16_t attr = s[1];
		int sy = (int)((s[2] & 0x1ff) ^ 0x100) - 0x100, sx = (int)((s[3] & 0x1ff) ^ 0x100) - 0x100;
		int flipx = attr & 0x02, flipy = attr & 0x01;
		if (s_snap->flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			flipx = !flipx;
			flipy = !flipy;
		}
		sy -= VIS_Y0;
		if (!s_gfxspr.used[code] || sx >= VIS_W || sx + 16 <= 0 || sy >= VIS_H || sy + 16 <= 0)
			continue;
		const uint16_t c0 = (uint16_t)(512 + ((attr >> 2) & 0x0f) * 16);
		const uint8_t *g = s_gfxspr.pix + code * 256;
		for (int j = 0; j < 16; j++) {
			const int yy = sy + j;
			if ((unsigned)yy >= VIS_H)
				continue;
			const uint8_t *src = g + (flipy ? 15 - j : j) * 16;
			for (int i = 0; i < 16; i++) {
				const int xx = sx + i;
				if ((unsigned)xx >= VIS_W)
					continue;
				const uint8_t p = src[flipx ? 15 - i : i];
				if (p != 15)
					s_frame[yy][xx] = (uint16_t)(c0 + p);
			}
		}
	}
}

#define BANDS 8
static int s_phase;
static void bc_band(int band)
{
	for (int y = VIS_H * band / BANDS; y < VIS_H * (band + 1) / BANDS; y++) {
		uint16_t *d = s_frame[y];
		const int by = y + VIS_Y0;
		if (s_phase == 0) {   // black, FG category 1 (layer 1), BG, FG category 0 layer 1
			for (int x = 0; x < VIS_W; x++)
				d[x] = BLACK;
			fg_line(by, 1, 1, d);
			bg_line(by, d);
			fg_line(by, 0, 1, d);
		} else {              // (the sprites), FG category 0 layer 0, text
			fg_line(by, 0, 0, d);
			tx_line(by, d);
			memcpy(rbx_video.bitmap + y * rbx_video.pitch, d, VIS_W * 2);
		}
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, bc_band };
static int bc_help(int helper) { return decoband_help(&s_bands, helper); }

static void bc_render(void)
{
	for (int i = 0; i < PAL_N; i++) {   // RRRRGGGGBBBBIIII (bionicc)
		const uint32_t raw = s_snap->pal[i], bright = raw & 0x0f;
		uint8_t r = (uint8_t)(((raw >> 12) & 0x0f) * 0x11), g = (uint8_t)(((raw >> 8) & 0x0f) * 0x11);
		uint8_t b = (uint8_t)(((raw >> 4) & 0x0f) * 0x11);
		if (!(bright & 0x08)) {
			r = (uint8_t)(r / 2 + bright * 18.15);
			g = (uint8_t)(g / 2 + bright * 18.15);
			b = (uint8_t)(b / 2 + bright * 18.15);
		}
		rbx_video.palette[i] = (uint32_t)r << 16 | (uint32_t)g << 8 | b;
	}
	rbx_video.palette[BLACK] = 0;
	s_phase = 0;
	decoband_run(&s_bands);
	draw_sprites();
	s_phase = 1;
	decoband_run(&s_bands);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spr, s_spritebuf, sizeof(s_spritebuf));
	memcpy(s_snap->tx, s_txram, sizeof(s_txram));
	memcpy(s_snap->fg, s_fgram, sizeof(s_fgram));
	memcpy(s_snap->bg, s_bgram, sizeof(s_bgram));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	memcpy(s_snap->scroll, s_scroll, sizeof(s_scroll));
	s_snap->flip = s_flip;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void bc_reset(void)
{
	memset(s_lowram, 0, sizeof(s_lowram));
	memset(s_spritebuf, 0, sizeof(s_spritebuf));
	memset(s_txram, 0, sizeof(s_txram));
	memset(s_fgram, 0, sizeof(s_fgram));
	memset(s_bgram, 0, sizeof(s_bgram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_wram, 0, sizeof(s_wram));
	memset(s_scroll, 0, sizeof(s_scroll));
	s_flip = s_halted = 0;
	s_audiocpu_to_mcu = s_mcu_to_audiocpu = s_mcu_p1 = 0;
	s_mcu_p3 = 0xff;
	s_zirq = 0;
	memset(s_zram, 0, sizeof(s_zram));
	z80_start();
	rbx_ym_reset(0);
	mcs51_reset(&s_mcu);
	s_mix_n = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_cpu_frac = s_z80_frac = s_ym_frac = s_mcu_frac = 0;
}

static void bc_exit(void);

static int bc_init(const rbx_game *g)
{
	uint32_t ts, bs, fs, ss;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_mrom = rbx_region("mcu", &s_mrom_size);
	uint8_t *tx = rbx_region("gfx1", &ts), *bg = rbx_region("gfx2", &bs), *fg = rbx_region("gfx3", &fs);
	uint8_t *spr = rbx_region("spritegen", &ss);
	if (!s_rom || s_rom_size < 0x40000 || !s_zrom || !s_mrom || !tx || !bg || !fg || !spr) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	{	// vramlayout, scroll2layout, scroll1layout (RGN_FRAC(1,2)), tigeroad_spr 16x16x4
		static uint32_t x8[8], y8[8], x16[16], y16[16], xs[16], ys[16];
		for (int i = 0; i < 4; i++) {
			x8[i] = (uint32_t)i;
			x8[4 + i] = (uint32_t)(4 * 2 + i);
			x16[i] = (uint32_t)i;
			x16[4 + i] = (uint32_t)(4 * 2 + i);
			x16[8 + i] = (uint32_t)(4 * 2 * 2 * 16 + i);
			x16[12 + i] = (uint32_t)(4 * 2 * 2 * 16 + 4 * 2 + i);
		}
		for (int i = 0; i < 8; i++) {
			y8[i] = (uint32_t)(i * 4 * 2 * 2);
			xs[i] = (uint32_t)i;
			xs[8 + i] = (uint32_t)(8 * 4 * 16 + i);
		}
		for (int i = 0; i < 16; i++) {
			y16[i] = (uint32_t)(i * 4 * 2 * 2);
			ys[i] = (uint32_t)(i * 8 * 4);
		}
		const uint32_t hb = bs * 4, hf = fs * 4;
		static const uint32_t pt[2] = { 4, 0 }, ps[4] = { 0, 8, 16, 24 };
		const uint32_t pb[4] = { hb + 4, hb, 4, 0 }, pf[4] = { hf + 4, hf, 4, 0 };
		const rbx_gfx_layout lt = { 8, 8, 2, pt, x8, y8, 128 };
		const rbx_gfx_layout lb = { 8, 8, 4, pb, x8, y8, 128 };
		const rbx_gfx_layout lf = { 16, 16, 4, pf, x16, y16, 512 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, xs, ys, 16 * 16 * 4 };
		if (rbx_gfx_decode(&s_gfxtx, &lt, tx, ts) || rbx_gfx_decode(&s_gfxbg, &lb, bg, bs / 2) ||
		    rbx_gfx_decode(&s_gfxfg, &lf, fg, fs / 2) || rbx_gfx_decode(&s_gfxspr, &ls, spr, ss)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			bc_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;   // YM2151 0.60 (both outputs, mono)
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 154;
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		bc_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, PAL_N + 1)) {
		rbx_fail("%s: out of memory", g->set);
		bc_exit();
		return -1;
	}
	mcs51_init(&s_mcu, s_mrom, s_mrom_size);
	s_mcu.port_in = mcu_port_in;
	s_mcu.port_out = mcu_port_out;
	s_mcu.xdata_r = mcu_xdata_r;
	s_mcu.xdata_w = mcu_xdata_w;
	memset(s_zmap, 0, sizeof(s_zmap));
	for (uint32_t p = 0; p < 0x80 && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	s_port[0] = rbx_port_find("INPUTS");
	s_port[1] = rbx_port_find("DSW");
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);   // (MAME's DIVx / MULx / ADDQ.W An times)
	m68k_bus_reset_map();
	for (uint32_t m = 0; m < 0x1000000; m += 0x100000) {   // (global_mask 0xfffff)
		m68k_bus_map(m, m + 0x3ffff, s_rom, 0x40000, 1, 0);
		for (uint32_t k = 0; k < 0x4000; k += 0x1000) {   // (mirror 0x3000)
			m68k_bus_map(m + 0xe0000 + k, m + 0xe0fff + k, s_lowram, sizeof(s_lowram), 1, 1);
			m68k_bus_map(m + 0xec000 + k, m + 0xecfff + k, s_txram, sizeof(s_txram), 1, 1);
		}
		m68k_bus_map(m + 0xf0000, m + 0xf3fff, s_fgram, sizeof(s_fgram), 1, 1);
		m68k_bus_map(m + 0xf4000, m + 0xf7fff, s_bgram, sizeof(s_bgram), 1, 1);
		m68k_bus_map(m + 0xfc000, m + 0xfffff, s_wram, sizeof(s_wram), 1, 1);
	}
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	bc_reset();
	return 0;
}

static void bc_exit(void)
{
	m68k_set_mame_timing(0);
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gfxtx);
	rbx_gfx_free(&s_gfxbg);
	rbx_gfx_free(&s_gfxfg);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
}

static void bc_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == 240) {   // (the vblank: the frame drawn) IRQ 2
			vblank_capture();
			m68k_bus_hold_irq(2);
		} else if (line == 128)
			m68k_bus_hold_irq(4);
		for (int k = 0; k < SLICES; k++) {   // (the 68000 and the MCU in quarter lines: the DMA's handshake)
			const int c = rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES * SLICES);
			if (!s_halted)
				m68k_execute(c);
			mcs51_execute(&s_mcu, rbx_line_cycles(&s_mcu_frac, MCU_CLOCK, FRAME_MHZ, LINES * SLICES));
		}
		sound_line();
	}
	sound_frame_end();
}

static int bc_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[4] = { s_cpu_frac, s_z80_frac, s_ym_frac, s_mcu_frac };
	uint8_t regs[8] = { s_flip, s_halted, s_audiocpu_to_mcu, s_mcu_to_audiocpu, s_mcu_p1, s_mcu_p3, 0, 0 };
	z80 z = s_z80;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "lowram", s_lowram, sizeof(s_lowram));
	rc |= rbx_state_io(ctx, "sprb", s_spritebuf, sizeof(s_spritebuf));
	rc |= rbx_state_io(ctx, "tx", s_txram, sizeof(s_txram));
	rc |= rbx_state_io(ctx, "fg", s_fgram, sizeof(s_fgram));
	rc |= rbx_state_io(ctx, "bg", s_bgram, sizeof(s_bgram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "wram", s_wram, sizeof(s_wram));
	rc |= rbx_state_io(ctx, "scroll", s_scroll, sizeof(s_scroll));
	rc |= rbx_state_io(ctx, "regs", regs, sizeof(regs));
	rc |= rbx_state_io(ctx, "mcu", &s_mcu.st, sizeof(s_mcu.st));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
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
		s_mcu_frac = fr[3];
		s_flip = regs[0];
		s_halted = regs[1];
		s_audiocpu_to_mcu = regs[2];
		s_mcu_to_audiocpu = regs[3];
		s_mcu_p1 = regs[4];
		s_mcu_p3 = regs[5];
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int bc_save(void *ctx) { return bc_state(ctx, 0); }
static int bc_load(void *ctx) { return bc_state(ctx, 1); }
static int bc_supported(const char *machine) { return !strcmp(machine, "bionicc"); }

const rbx_driver rbx_drv_bionicc = { "Capcom Bionic Commando", bc_init, bc_exit, bc_reset, bc_frame, bc_save, bc_load,
                                     bc_supported, bc_render, bc_help };
