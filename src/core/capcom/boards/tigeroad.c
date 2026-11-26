/*  Capcom Tiger Road / Tora e no Michi and F-1 Dream. Ported from MAME src/mame/capcom/tigeroad.cpp, tigeroad_v.cpp,
    tigeroad_m.cpp and tigeroad_spr.cpp (BSD-3-Clause, copyright-holders: Phil Stroffolino, David Haywood).
    68000 at 10 MHz (IRQ 2 at line 240, IRQ 4 at line 128); a 4096x4096 BG of 32x32 tiles from a ROM map (two pen
    groups: some pens over the sprites), an 8x8 text layer, 16x16 sprites from a buffer. Sound: Z80 at 3.58 MHz with two
    YM2203s (the first one's IRQ), the latch polled; Tora e no Michi adds a Z80 feeding an MSM5205 on its VCK IRQ;
    F-1 Dream's i8751 takes the sound commands while the 68000 waits halted.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/mcs51/mcs51.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/msm5205.h"
#include "../../sound/rbx_ym.h"
#include "../../video/decoband.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 10000000
#define Z80_CLOCK 3579545
#define YM_CLOCK 3579545
#define MCU_CLOCK (6000000 / 12)
#define FRAME_MHZ 60096            // set_raw(24 MHz / 4, 384, 0, 256, 260, 16, 240)
#define LINES 260
#define VIS_Y0 16
#define VIS_W 256
#define VIS_H 224
#define SLICES 4

enum { M_TIGEROAD, M_TORAMICH, M_F1DREAM };

static int s_mach;
static uint8_t *s_rom, *s_zrom, *s_srom, *s_mrom;
static uint32_t s_rom_size, s_zrom_size, s_srom_size, s_mrom_size;
static const uint16_t *s_bgmap;
static uint16_t s_lowram[0x800];        // 0xe0000 .. 0xe0fff (0xe0800 .. 0xe0cff the sprites), mirror 0x3000
static uint16_t s_spritebuf[0x280];
static uint16_t s_vram[0x400];          // 0xec000 .. 0xec7ff
static uint16_t s_pal[0x400];           // 0xf8000 .. 0xf87ff
static uint16_t s_wram[0x2000];         // 0xfc000 .. 0xfffff
static uint16_t s_scroll[2];
static uint8_t s_flip, s_bank, s_halted, s_mcu_p3, s_soundlatch_data;
static rbx_gfx s_gfxtx, s_gfxbg, s_gfxspr;
static int s_port[3];
static uint64_t s_cpu_frac, s_z80_frac, s_ym_frac, s_mcu_frac, s_s80_frac;
static mcs51_t s_mcu;
static struct {
	uint16_t spr[0x280], vram[0x400], pal[0x400], scroll[2];
	uint8_t flip, bank;
} *s_snap;

// ---- sound ----
static z80 s_z80, s_s80;   // (the sound CPU; Tora e no Michi's sample CPU)
static uint8_t s_zram[0x800];
static uint8_t s_latch, s_latch2;
static int s_ym_irq, s_s_irq;
static msm5205_t s_msm;
static uint32_t s_msm_acc;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static const uint8_t *s_zmap[256], *s_smap[256];

static void ym_irq(int state) { s_ym_irq = state; }
static void msm_vck(void) { s_s_irq = 1; }   // (vck_callback: the sample CPU's IRQ, HOLD_LINE)

static uint8_t z80_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_zrom_size ? s_zrom[a] : 0;
	if (a == 0x8000 || a == 0x8001) return rbx_ym_read(0, a & 1);
	if (a == 0xa000 || a == 0xa001) return rbx_ym_read(1, a & 1);
	if (a >= 0xc000 && a < 0xc800) return s_zram[a - 0xc000];
	if (a == 0xe000) return s_latch;
	return 0;
}
static void z80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a == 0x8000 || a == 0x8001) rbx_ym_write(0, a & 1, v);
	else if (a == 0xa000 || a == 0xa001) rbx_ym_write(1, a & 1, v);
	else if (a >= 0xc000 && a < 0xc800) s_zram[a - 0xc000] = v;
}
static uint8_t z80_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z80_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	if (s_mach == M_TORAMICH && port == 0x7f)   // sound_port_map: soundlatch2
		s_latch2 = v;
}

static uint8_t s80_read(void *u, uint16_t a) { (void)u; return a < s_srom_size ? s_srom[a] : 0; }
static void s80_write(void *u, uint16_t a, uint8_t v) { (void)u; (void)a; (void)v; }
static uint8_t s80_in(z80 *z, uint8_t port) { (void)z; return port == 0x00 ? s_latch2 : 0xff; }
static void s80_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	if (port == 0x01) {   // msm5205_w: S1, reset, data
		msm5205_set_selector(&s_msm, MSM5205_S96_4B | ((v >> 6) & 1));
		msm5205_reset_w(&s_msm, (v >> 7) & 1);
		msm5205_data_w(&s_msm, v & 0xf);
	}
}

static void z80_start(void)
{
	z80_init(&s_z80);
	s_z80.read_byte = z80_read;
	s_z80.write_byte = z80_write;
	s_z80.port_in = z80_in;
	s_z80.port_out = z80_out;
	s_z80.read_map = s_zmap;
	z80_init(&s_s80);
	s_s80.read_byte = s80_read;
	s_s80.write_byte = s80_write;
	s_s80.port_in = s80_in;
	s_s80.port_out = s80_out;
	s_s80.read_map = s_smap;
}

static void sound_line(void)
{
	const int cycles = rbx_line_cycles(&s_z80_frac, Z80_CLOCK, FRAME_MHZ, LINES);
	z80_run(&s_z80, s_z80.cyc + (unsigned long)cycles, &s_ym_irq, 0);
	if (s_mach == M_TORAMICH) {
		const int sc = rbx_line_cycles(&s_s80_frac, Z80_CLOCK, FRAME_MHZ, LINES);
		z80_run(&s_s80, s_s80.cyc + (unsigned long)sc, &s_s_irq, 1);
	}
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&s_ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
	const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	rbx_ym_advance(1, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	if (s_mach == M_TORAMICH) {
		const uint32_t yr = rbx_ym_rate(0);
		for (int i = 0; i < n; i++) {
			s_msm_acc += s_msm.clock;
			const uint32_t k = s_msm_acc / yr;
			s_msm_acc -= k * yr;
			msm5205_advance(&s_msm, k);
			s_mix[s_mix_n + i] += msm5205_output(&s_msm) * 2;   // (route 1.0)
		}
	}
	s_mix_n += n;
}

static void sound_frame_end(void)
{
	rbx_ym_async_take(0, s_mix, s_mix_n);
	rbx_ym_async_take(1, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

// ---- F-1 Dream's MCU ----
static void mcu_port_out(void *u, int port, uint8_t data)
{
	(void)u;
	if (port == 1)
		s_soundlatch_data = data;
	else if (port == 3) {   // mcu_out3_w
		if ((s_mcu_p3 & 0x01) && !(data & 0x01)) {
			mcs51_set_input(&s_mcu, MCS51_INT0_LINE, 0);
			s_halted = 0;
		}
		if ((s_mcu_p3 & 0x40) && !(data & 0x40))
			s_latch = s_soundlatch_data;
		s_mcu_p3 = data;
	}
}
static uint8_t mcu_xdata_r(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x7f0 || a > 0x7ff)
		return 0;
	return (s_mcu_p3 & 0x20) ? 0xff : (uint8_t)m68k_read_memory_8(0xfffe0u | (uint32_t)(a & 0xf) << 1 | 1);
}
static void mcu_xdata_w(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x7f0 && a <= 0x7ff && !(s_mcu_p3 & 0x20))
		m68k_write_memory_8(0xfffe0u | (uint32_t)(a & 0xf) << 1 | 1, v);
}

// ---- bus (global_mask 0xfffff) ----
static uint16_t main_read16(uint32_t a)
{
	a &= 0xfffff;
	if (a >= 0xe4000 && a < 0xe4006)   // P1_P2, SYSTEM, DSW
		return (uint16_t)rbx_port_read(s_port[(a - 0xe4000) >> 1]);
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
	if (a == 0xe4000) {
		if (mask & 0xff00) {   // videoctrl_w: flip, the BG character bank (and the coin lockouts / counters)
			s_flip = (v >> 9) & 1;
			s_bank = (v >> 10) & 1;
		}
	} else if (a == 0xe4002) {
		if (s_mach == M_F1DREAM) {   // to_mcu_w: the MCU's INT0, the 68000 halted until it is done
			mcs51_set_input(&s_mcu, MCS51_INT0_LINE, 1);
			s_halted = 1;
			m68k_end_timeslice();
		} else if (mask & 0xff00)
			s_latch = (uint8_t)(v >> 8);
	} else if (a >= 0xe8000 && a < 0xe8004) {
		uint16_t *w = &s_scroll[(a >> 1) & 1];
		*w = (uint16_t)((0 & ~mask) | (v & mask));   // (scroll_w: COMBINE_DATA into a fresh 0)
	} else if (a == 0xe800e)
		memcpy(s_spritebuf, &s_lowram[0x400], sizeof(s_spritebuf));
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
// BG pen groups: set_transmask(0, 0xffff, 0), (1, 0x01ff, 0xfe00)
static const uint16_t s_bg_mask0[2] = { 0xffff, 0x01ff }, s_bg_mask1[2] = { 0x0000, 0xfe00 };
static uint16_t s_frame[VIS_H][VIS_W];

// the BG (128x128 of 32x32 tiles, tigeroad_tilemap_scan), layer 0 or 1, a tile's span at a time
static void bg_line(int by, int layer, uint16_t *d)
{
	const int flip = s_snap->flip;
	const int sx = s_snap->scroll[0], sy = -(int)s_snap->scroll[1] - 32 * 8;
	const uint32_t ty = (uint32_t)(flip ? 255 - by + sy : by + sy) & 4095;
	uint32_t tx = (uint32_t)(flip ? 255 + sx : sx) & 4095;
	const uint32_t row = ty >> 5, rr = 127 - row, fy = ty & 31;
	int x = 0;
	while (x < VIS_W) {
		int px = (int)(tx & 31), n = flip ? px + 1 : 32 - px;
		if (n > VIS_W - x)
			n = VIS_W - x;
		const uint32_t col = tx >> 5;
		const uint16_t data = s_bgmap[(col % 8) + 8 * (rr % 8) + 0x40 * (col / 8) + 0x400 * (rr / 8)];
		const uint32_t attr = data >> 8;
		const uint32_t code = rbx_wrap(((data & 0xffu) + ((attr & 0xc0) << 2) + ((uint32_t)s_snap->bank << 10)), s_gfxbg.count);
		const int group = (attr & 0x10) ? 1 : 0, fx = (attr & 0x20) != 0;
		const uint16_t m = layer ? s_bg_mask1[group] : s_bg_mask0[group];
		if (m != 0xffff) {
			const uint8_t *src = s_gfxbg.pix + code * 1024 + fy * 32;
			const uint16_t c0 = (uint16_t)(0x100 + (attr & 0x0f) * 16);
			for (int k = 0; k < n; k++) {
				const int c = flip ? px - k : px + k;
				const uint8_t p = src[fx ? 31 - c : c];
				if (!((m >> p) & 1))
					d[x + k] = (uint16_t)(c0 + p);
			}
		}
		x += n;
		tx = (tx + (uint32_t)(flip ? -n : n)) & 4095;
	}
}

static void fg_line(int by, uint16_t *d)   // 32x32 of 8x8, 2 bits, pen 3 transparent
{
	const int flip = s_snap->flip;
	const uint32_t ty = (uint32_t)(flip ? 255 - by : by) & 255;
	for (int x = 0; x < VIS_W; x += 8) {
		const uint32_t tx = (uint32_t)(flip ? 255 - x : x) & 255;
		const uint16_t data = s_snap->vram[(ty >> 3) * 32 + (tx >> 3)];
		const uint32_t attr = data >> 8;
		const uint32_t code = rbx_wrap(((data & 0xffu) + ((attr & 0xc0) << 2) + ((attr & 0x20) << 5)), s_gfxtx.count);
		const uint32_t py = (attr & 0x10) ? 7 - (ty & 7) : (ty & 7);
		const uint8_t *src = s_gfxtx.pix + code * 64 + py * 8;
		const uint16_t c0 = (uint16_t)(0x300 + (attr & 0x0f) * 4);
		for (int k = 0; k < 8; k++) {
			const uint8_t p = src[flip ? 7 - k : k];
			if (p != 3)
				d[x + k] = (uint16_t)(c0 + p);
		}
	}
}

static void draw_sprites(void)   // tigeroad_spr_device::draw_sprites with rev_y (from the end, pen 15 transparent)
{
	for (int o = 0x280 - 4; o >= 0; o -= 4) {
		const uint16_t *s = s_snap->spr + o;
		const uint32_t code = rbx_wrap(s[0], s_gfxspr.count);
		const uint16_t attr = s[1];
		int sy = (int)((s[2] & 0x1ff) ^ 0x100) - 0x100, sx = (int)((s[3] & 0x1ff) ^ 0x100) - 0x100;
		int flipx = attr & 0x02, flipy = attr & 0x01;
		if (s_snap->flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			flipx = !flipx;
			flipy = !flipy;
		}
		sy = 240 - sy - VIS_Y0;
		if (!s_gfxspr.used[code] || sx >= VIS_W || sx + 16 <= 0 || sy >= VIS_H || sy + 16 <= 0)
			continue;
		const uint16_t c0 = (uint16_t)(0x200 + ((attr >> 2) & 0x0f) * 16);
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
static void tr_band(int band)
{
	for (int y = VIS_H * band / BANDS; y < VIS_H * (band + 1) / BANDS; y++) {
		uint16_t *d = s_frame[y];
		if (s_phase == 0)
			bg_line(y + VIS_Y0, 1, d);
		else {
			bg_line(y + VIS_Y0, 0, d);
			fg_line(y + VIS_Y0, d);
			memcpy(rbx_video.bitmap + y * rbx_video.pitch, d, VIS_W * 2);
		}
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, tr_band };
static int tr_help(int helper) { return decoband_help(&s_bands, helper); }

static void tr_render(void)
{
	for (int i = 0; i < 0x400; i++) {   // xRGB_444
		const uint16_t p = s_snap->pal[i];
		rbx_video.palette[i] = (uint32_t)((((p >> 8) & 15) * 17) << 16 | (((p >> 4) & 15) * 17) << 8 | ((p & 15) * 17));
	}
	s_phase = 0;   // BG layer 1, the sprites, BG layer 0, the text
	decoband_run(&s_bands);
	draw_sprites();
	s_phase = 1;
	decoband_run(&s_bands);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spr, s_spritebuf, sizeof(s_spritebuf));
	memcpy(s_snap->vram, s_vram, sizeof(s_vram));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	memcpy(s_snap->scroll, s_scroll, sizeof(s_scroll));
	s_snap->flip = s_flip;
	s_snap->bank = s_bank;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void tr_reset(void)
{
	memset(s_lowram, 0, sizeof(s_lowram));
	memset(s_spritebuf, 0, sizeof(s_spritebuf));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_wram, 0, sizeof(s_wram));
	memset(s_scroll, 0, sizeof(s_scroll));
	s_flip = s_bank = s_halted = 0;
	s_mcu_p3 = 0xff;
	s_soundlatch_data = 0;
	s_latch = s_latch2 = 0;
	s_ym_irq = s_s_irq = 0;
	memset(s_zram, 0, sizeof(s_zram));
	z80_start();
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	if (s_mach == M_TORAMICH) {
		msm5205_reset(&s_msm);
		s_msm_acc = 0;
	}
	if (s_mach == M_F1DREAM)
		mcs51_reset(&s_mcu);
	s_mix_n = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_cpu_frac = s_z80_frac = s_ym_frac = s_mcu_frac = s_s80_frac = 0;
}

static void tr_exit(void);

static int tr_init(const rbx_game *g)
{
	const char *m = g->machine;
	s_mach = !strncmp(m, "toramich", 8) ? M_TORAMICH : !strncmp(m, "f1dream", 7) ? M_F1DREAM : M_TIGEROAD;
	uint32_t ts, bs, ss, ms;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_srom = s_mach == M_TORAMICH ? rbx_region("sample", &s_srom_size) : NULL;
	s_mrom = s_mach == M_F1DREAM ? rbx_region("mcu", &s_mrom_size) : NULL;
	uint8_t *tx = rbx_region("text", &ts), *bg = rbx_region("tiles", &bs), *spr = rbx_region("spritegen", &ss);
	const uint8_t *map = rbx_region("bgmap", &ms);
	if (!s_rom || s_rom_size < 0x40000 || !s_zrom || !tx || !bg || !spr || !map || ms < 0x8000 ||
	    (s_mach == M_TORAMICH && !s_srom) || (s_mach == M_F1DREAM && !s_mrom)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_bgmap = (const uint16_t *)map;   // (ROM_REGION16_LE: host words on a little-endian host)
	m68k_bus_swap16(s_rom, s_rom_size);
	if (strstr(m, ":tigeroadb"))   // init_tigeroadb: the sprite ROMs' nibbles bit-reversed
		for (uint32_t i = 0; i < ss; i++) {
			const uint8_t v = spr[i];
			spr[i] = (uint8_t)((v >> 4 & 1) << 7 | (v >> 5 & 1) << 6 | (v >> 6 & 1) << 5 | (v >> 7 & 1) << 4 |
			                   (v & 1) << 3 | (v >> 1 & 1) << 2 | (v >> 2 & 1) << 1 | (v >> 3 & 1));
		}
	{	// text_layout, tile_layout (32x32, RGN_FRAC(1,2)), tigeroad_spr 16x16x4
		static uint32_t xt[8], yt[8], xb[32], yb[32], xs[16], ys[16];
		for (int i = 0; i < 4; i++) {
			xt[i] = (uint32_t)i;
			xt[4 + i] = (uint32_t)(4 * 2 + i);
		}
		for (int q = 0; q < 4; q++)   // STEP4(4*2*2*32*q, 1), STEP4(4*2*2*32*q + 4*2, 1)
			for (int i = 0; i < 4; i++) {
				xb[q * 8 + i] = (uint32_t)(4 * 2 * 2 * 32 * q + i);
				xb[q * 8 + 4 + i] = (uint32_t)(4 * 2 * 2 * 32 * q + 4 * 2 + i);
			}
		for (int i = 0; i < 8; i++) {
			yt[i] = (uint32_t)(i * 4 * 2 * 2);
			xs[i] = (uint32_t)i;
			xs[8 + i] = (uint32_t)(8 * 4 * 16 + i);
		}
		for (int i = 0; i < 32; i++)
			yb[i] = (uint32_t)(i * 4 * 2 * 2);
		for (int i = 0; i < 16; i++)
			ys[i] = (uint32_t)(i * 8 * 4);
		const uint32_t h = bs * 4;
		static const uint32_t pt[2] = { 4, 0 }, ps[4] = { 0, 8, 16, 24 };
		const uint32_t pb[4] = { h + 4, h, 4, 0 };
		const rbx_gfx_layout lt = { 8, 8, 2, pt, xt, yt, 16 * 8 };
		const rbx_gfx_layout lb = { 32, 32, 4, pb, xb, yb, 256 * 8 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, xs, ys, 16 * 16 * 4 };
		if (rbx_gfx_decode(&s_gfxtx, &lt, tx, ts) || rbx_gfx_decode(&s_gfxbg, &lb, bg, bs / 2) ||
		    rbx_gfx_decode(&s_gfxspr, &ls, spr, ss)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			tr_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;   // two YM2203s, all outputs 0.25
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 64;
	yc.irq = ym_irq;
	int bad = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	yc.irq = NULL;
	if (!bad)
		bad = rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	if (bad) {
		rbx_fail("%s: out of memory", g->set);
		tr_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 0x400)) {
		rbx_fail("%s: out of memory", g->set);
		tr_exit();
		return -1;
	}
	if (s_mach == M_TORAMICH)
		msm5205_init(&s_msm, 384000, MSM5205_S96_4B, msm_vck);
	if (s_mach == M_F1DREAM) {
		mcs51_init(&s_mcu, s_mrom, s_mrom_size);
		s_mcu.port_out = mcu_port_out;
		s_mcu.xdata_r = mcu_xdata_r;
		s_mcu.xdata_w = mcu_xdata_w;
	}
	memset(s_zmap, 0, sizeof(s_zmap));
	for (uint32_t p = 0; p < 0x80 && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	memset(s_smap, 0, sizeof(s_smap));
	for (uint32_t p = 0; s_srom && p < 0x100 && p * 256 + 256 <= s_srom_size; p++)
		s_smap[p] = s_srom + p * 256;
	static const char *const names[3] = { "P1_P2", "SYSTEM", "DSW" };
	for (int i = 0; i < 3; i++)
		s_port[i] = rbx_port_find(names[i]);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);   // (MAME's DIVx / MULx / ADDQ.W An times)
	m68k_bus_reset_map();
	for (uint32_t b = 0; b < 0x1000000; b += 0xf00000) {   // (global_mask 0xfffff: the low and the top 1 MB, as the
		m68k_bus_map(b, b + 0x3ffff, s_rom, 0x40000, 1, 0);   // program uses them; every mirror is too many ranges)
		for (uint32_t k = 0; k < 0x4000; k += 0x1000)   // (mirror 0x3000)
			m68k_bus_map(b + 0xe0000 + k, b + 0xe0fff + k, s_lowram, sizeof(s_lowram), 1, 1);
		m68k_bus_map(b + 0xec000, b + 0xec7ff, s_vram, sizeof(s_vram), 1, 1);
		m68k_bus_map(b + 0xf8000, b + 0xf87ff, s_pal, sizeof(s_pal), 1, 1);
		m68k_bus_map(b + 0xfc000, b + 0xfffff, s_wram, sizeof(s_wram), 1, 1);
	}
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	tr_reset();
	return 0;
}

static void tr_exit(void)
{
	m68k_set_mame_timing(0);
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gfxtx);
	rbx_gfx_free(&s_gfxbg);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
}

static void tr_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == 240) {   // (the vblank: the frame drawn) IRQ 2
			vblank_capture();
			m68k_bus_hold_irq(2);
		} else if (line == 128)
			m68k_bus_hold_irq(4);
		if (s_mach == M_F1DREAM)
			for (int k = 0; k < SLICES; k++) {   // (the 68000 and the MCU in quarter lines: the handshake)
				const int c = rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES * SLICES);
				if (!s_halted)
					m68k_execute(c);
				mcs51_execute(&s_mcu, rbx_line_cycles(&s_mcu_frac, MCU_CLOCK, FRAME_MHZ, LINES * SLICES));
			}
		else
			m68k_execute(rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		sound_line();
	}
	sound_frame_end();
}

static int tr_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[5] = { s_cpu_frac, s_z80_frac, s_ym_frac, s_mcu_frac, s_s80_frac };
	uint8_t regs[10] = { s_flip, s_bank, s_halted, s_mcu_p3, s_soundlatch_data, s_latch, s_latch2,
	                     (uint8_t)s_ym_irq, (uint8_t)s_s_irq, 0 };
	z80 z = s_z80, z2 = s_s80;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "lowram", s_lowram, sizeof(s_lowram));
	rc |= rbx_state_io(ctx, "sprb", s_spritebuf, sizeof(s_spritebuf));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "wram", s_wram, sizeof(s_wram));
	rc |= rbx_state_io(ctx, "scroll", s_scroll, sizeof(s_scroll));
	rc |= rbx_state_io(ctx, "regs", regs, sizeof(regs));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	if (s_mach == M_TORAMICH) {
		rc |= rbx_state_io(ctx, "s80", &z2, sizeof(z2));
		rc |= rbx_state_io(ctx, "msm", &s_msm, sizeof(s_msm));
		rc |= rbx_state_io(ctx, "msmacc", &s_msm_acc, sizeof(s_msm_acc));
	}
	if (s_mach == M_F1DREAM)
		rc |= rbx_state_io(ctx, "mcu", &s_mcu.st, sizeof(s_mcu.st));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
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
		if (s_mach == M_TORAMICH) {
			z2.read_byte = s80_read;
			z2.write_byte = s80_write;
			z2.port_in = s80_in;
			z2.port_out = s80_out;
			z2.userdata = NULL;
			z2.read_map = s_smap;
			s_s80 = z2;
			s_msm.vck = msm_vck;
		}
		s_cpu_frac = fr[0];
		s_z80_frac = fr[1];
		s_ym_frac = fr[2];
		s_mcu_frac = fr[3];
		s_s80_frac = fr[4];
		s_flip = regs[0];
		s_bank = regs[1];
		s_halted = regs[2];
		s_mcu_p3 = regs[3];
		s_soundlatch_data = regs[4];
		s_latch = regs[5];
		s_latch2 = regs[6];
		s_ym_irq = regs[7];
		s_s_irq = regs[8];
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int tr_save(void *ctx) { return tr_state(ctx, 0); }
static int tr_load(void *ctx) { return tr_state(ctx, 1); }
static int tr_supported(const char *machine)
{
	return !strncmp(machine, "tigeroad", 8) || !strncmp(machine, "toramich", 8) || !strncmp(machine, "f1dream", 7);
}

const rbx_driver rbx_drv_tigeroad = { "Capcom Tiger Road", tr_init, tr_exit, tr_reset, tr_frame, tr_save, tr_load,
                                      tr_supported, tr_render, tr_help };
