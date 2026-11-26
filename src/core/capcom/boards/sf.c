/*  Capcom Street Fighter. Ported from MAME src/mame/capcom/sf.cpp (BSD-3-Clause, copyright-holders: Olivier Galibert).
    68000 at 8 MHz (IRQ 1 at the vblank, the prototype IRQ 6); two 32768-pixel-wide layers of 16x16 tiles whose maps
    are in ROM, an 8x8 text layer, 16x16 and 32x32 sprites; the protected sets' i8751 works on the 68000's bus while
    it waits halted. Sound: a Z80 with a YM2151 (its IRQ, the latch on the NMI) and a second Z80 (8 kHz IRQ, banked
    ROM) feeding two MSM5205s.
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

#define CPU_CLOCK 8000000
#define Z80_CLOCK 3579545
#define YM_CLOCK 3579545
#define MCU_CLOCK (8000000 / 12)
#define FRAME_MHZ 60000            // set_refresh_hz(60), 64*8 x 32*8, visible 64 .. 447 x 16 .. 239
#define LINES 256
#define VIS_X0 64
#define VIS_Y0 16
#define VIS_W 384
#define VIS_H 224
#define SLICES 4

enum { M_SFUS, M_SFAN, M_SFJP, M_SFP };

static int s_mach;
static uint8_t *s_rom, *s_zrom, *s_arom, *s_mrom;
static uint32_t s_rom_size, s_zrom_size, s_arom_size, s_mrom_size;
static const uint8_t *s_tilerom;
static uint16_t s_vram[0x800];          // 0x800000 .. 0x800fff
static uint16_t s_pal[0x400];           // 0xb00000 .. 0xb007ff
static uint16_t s_hiram[0x4000];        // 0xff8000 .. 0xffffff (the sprites at 0xffe000)
static uint16_t s_bgscroll, s_fgscroll;
static uint8_t s_active, s_halted, s_prot_t0;
static rbx_gfx s_gfxbg, s_gfxfg, s_gfxspr, s_gfxtx;
static int s_port[7];
static uint64_t s_cpu_frac, s_z80_frac, s_ym_frac, s_a80_frac, s_mcu_frac, s_irq8k_frac;
static mcs51_t s_mcu;
static struct {
	uint16_t vram[0x800], pal[0x400], obj[0x1000], bgscroll, fgscroll;
	uint8_t active;
} *s_snap;

// ---- sound ----
static z80 s_z80, s_a80;
static uint8_t s_zram[0x800];
static uint8_t s_latch, s_bank;
static int s_ym_irq, s_a_irq;
static msm5205_t s_msm[2];
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static const uint8_t *s_zmap[256], *s_amap[256];

static void ym_irq(int state) { s_ym_irq = state; }

static uint8_t z80_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_zrom_size ? s_zrom[a] : 0;
	if (a >= 0xc000 && a < 0xc800) return s_zram[a - 0xc000];
	if (a == 0xc800) return s_latch;
	if (a == 0xe000 || a == 0xe001) return rbx_ym_read(0, a & 1);
	return 0;
}
static void z80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xc800) s_zram[a - 0xc000] = v;
	else if (a == 0xe000 || a == 0xe001) rbx_ym_write(0, a & 1, v);
}
static uint8_t z80_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z80_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

static void set_bank(void)   // audiobank: audio2 + 0x8000 + bank * 0x8000 at 0x8000 .. 0xffff
{
	for (uint32_t p = 0; p < 0x80; p++) {
		const uint32_t o = 0x8000 + (uint32_t)s_bank * 0x8000 + p * 256;
		s_amap[0x80 + p] = o + 256 <= s_arom_size ? s_arom + o : NULL;
	}
}
static uint8_t a80_read(void *u, uint16_t a)
{
	(void)u;
	const uint32_t o = a < 0x8000 ? a : 0x8000 + (uint32_t)s_bank * 0x8000 + (a - 0x8000u);
	return o < s_arom_size ? s_arom[o] : 0;
}
static void a80_write(void *u, uint16_t a, uint8_t v) { (void)u; (void)a; (void)v; }   // (no RAM)
static uint8_t a80_in(z80 *z, uint8_t port) { (void)z; return port == 0x01 ? s_latch : 0xff; }
static void a80_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	if (port <= 1) {   // msm_w: reset, data, a VCK pulse
		msm5205_reset_w(&s_msm[port], (v >> 7) & 1);
		msm5205_data_w(&s_msm[port], v & 0xf);
		msm5205_vclk_capture(&s_msm[port]);
	} else if (port == 0x02) {
		s_bank = v;
		set_bank();
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
	z80_init(&s_a80);
	s_a80.read_byte = a80_read;
	s_a80.write_byte = a80_write;
	s_a80.port_in = a80_in;
	s_a80.port_out = a80_out;
	s_a80.read_map = s_amap;
}

static void sound_line(void)
{
	const int cycles = rbx_line_cycles(&s_z80_frac, Z80_CLOCK, FRAME_MHZ, LINES);
	z80_run(&s_z80, s_z80.cyc + (unsigned long)cycles, &s_ym_irq, 0);
	if (rbx_line_cycles(&s_irq8k_frac, 8000, FRAME_MHZ, LINES))   // (set_periodic_int 8 kHz, irq0_line_hold)
		s_a_irq = 1;
	const int ac = rbx_line_cycles(&s_a80_frac, Z80_CLOCK, FRAME_MHZ, LINES);
	z80_run(&s_a80, s_a80.cyc + (unsigned long)ac, &s_a_irq, 1);
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&s_ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
	const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	const int32_t m = (msm5205_output(&s_msm[0]) + msm5205_output(&s_msm[1])) * 2;   // (routes 1.0)
	for (int i = 0; i < n; i++)
		s_mix[s_mix_n + i] += m;
	s_mix_n += n;
}

static void sound_frame_end(void)
{
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

// ---- the protection MCU (sfjp) ----
static uint32_t prot_address(uint16_t offset)
{
	uint32_t o = (uint32_t)offset * 2 + s_prot_t0;
	return (o & 0x8000) ? (0xff8000u | o) : (0xc00000u | (o & 0x7fff));
}
static uint8_t mcu_xdata_r(void *u, uint16_t a) { (void)u; return (uint8_t)m68k_read_memory_8(prot_address(a)); }
static void mcu_xdata_w(void *u, uint16_t a, uint8_t v) { (void)u; m68k_write_memory_8(prot_address(a), v); }
static void mcu_port_out(void *u, int port, uint8_t data)
{
	(void)u;
	if (port != 3)
		return;
	s_prot_t0 = (data & 0x10) ? 1 : 0;   // prot_p3_w
	if (!(data & 0x02)) {
		s_halted = 0;
		mcs51_set_input(&s_mcu, MCS51_INT0_LINE, 0);
	}
}

// ---- bus (unmap_value_high) ----
static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0xc00000 && a < 0xc0000e) {
		const int i = (int)((a - 0xc00000) >> 1);
		if (s_port[i] < 0)
			return 0xffff;   // (nopr: the unmapped value)
		return (uint16_t)rbx_port_read(s_port[i]);
	}
	return 0xffff;
}
static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	switch (a) {
	case 0xc00014: s_fgscroll = (uint16_t)((s_fgscroll & ~mask) | (v & mask)); break;
	case 0xc00018: s_bgscroll = (uint16_t)((s_bgscroll & ~mask) | (v & mask)); break;
	case 0xc0001a:
		if (mask & 0x00ff)   // gfxctrl_w: the planes, flip
			s_active = (uint8_t)v;
		break;
	case 0xc0001c:
		if (mask & 0x00ff) {   // soundcmd_w: the latch, an NMI pulse
			s_latch = (uint8_t)v;
			z80_gen_nmi(&s_z80);
		}
		break;
	case 0xc0001e:
		if (s_mach == M_SFJP) {   // protection_w: the MCU's INT0, the 68000 halted
			mcs51_set_input(&s_mcu, MCS51_INT0_LINE, 1);
			s_halted = 1;
			m68k_end_timeslice();
		}
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
static uint16_t s_frame[VIS_H][VIS_W];

// BG (0) / FG (1): 2048x16 of 16x16 tiles (scan columns), their maps in the tile ROM; the FG's pen 15 transparent
static void layer_line(int which, int by, uint16_t *d)
{
	const int flip = (s_snap->active & 0x04) != 0;
	const rbx_gfx *gfx = which ? &s_gfxfg : &s_gfxbg;
	const uint8_t *map = s_tilerom + (which ? 0x20000 : 0);
	const uint16_t base = which ? 256 : 0;
	const uint32_t scroll = which ? s_snap->fgscroll : s_snap->bgscroll;
	const uint32_t ty = (uint32_t)(flip ? 255 - by : by) & 255, row = ty >> 4;
	uint32_t tx = (uint32_t)(flip ? 511 - VIS_X0 + (int)scroll : VIS_X0 + (int)scroll) & 32767;
	int x = 0;
	while (x < VIS_W) {
		int px = (int)(tx & 15), n = flip ? px + 1 : 16 - px;
		if (n > VIS_W - x)
			n = VIS_W - x;
		const uint8_t *b = map + 2 * ((tx >> 4) * 16 + row);
		const uint32_t code = rbx_wrap(((uint32_t)b[0x10001] << 8 | b[1]), gfx->count);
		const int attr = b[0x10000];
		const uint16_t c0 = (uint16_t)(base + (b[0] % 16) * 16);
		const uint32_t py = (attr & 2) ? 15 - (ty & 15) : (ty & 15);
		const uint8_t *src = gfx->pix + code * 256 + py * 16;
		const int fx = attr & 1;
		for (int k = 0; k < n; k++) {
			const int c = flip ? px - k : px + k;
			const uint8_t p = src[fx ? 15 - c : c];
			if (!which || p != 15)
				d[x + k] = (uint16_t)(c0 + p);
		}
		x += n;
		tx = (tx + (uint32_t)(flip ? -n : n)) & 32767;
	}
}

static void tx_line(int by, uint16_t *d)   // 64x32 of 8x8, 2 bits, pen 3 transparent
{
	const int flip = (s_snap->active & 0x04) != 0;
	const uint32_t ty = (uint32_t)(flip ? 255 - by : by) & 255;
	for (int x = 0; x < VIS_W; x++) {
		const uint32_t tx = (uint32_t)(flip ? 511 - (x + VIS_X0) : x + VIS_X0) & 511;
		const uint16_t t = s_snap->vram[(ty >> 3) * 64 + (tx >> 3)];
		const uint32_t code = rbx_wrap((t & 0x3ffu), s_gfxtx.count);
		const uint32_t px = (t & 0x400) ? 7 - (tx & 7) : (tx & 7), py = (t & 0x800) ? 7 - (ty & 7) : (ty & 7);
		const uint8_t p = s_gfxtx.pix[code * 64 + py * 8 + px];
		if (p != 3)
			d[x] = (uint16_t)(768 + (t >> 12) * 4 + p);
	}
}

static inline uint32_t invert(uint32_t nb)
{
	static const uint32_t delta[4] = { 0x00, 0x18, 0x18, 0x00 };
	return nb ^ delta[(nb >> 3) & 3];
}

static void spr16(uint32_t code, uint16_t c0, int flipx, int flipy, int sx, int sy)
{
	code %= s_gfxspr.count;
	sx -= VIS_X0;
	sy -= VIS_Y0;
	if (!s_gfxspr.used[code] || sx >= VIS_W || sx + 16 <= 0 || sy >= VIS_H || sy + 16 <= 0)
		return;
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

static void draw_sprites(void)
{
	const uint16_t *obj = s_snap->obj;
	const int fs = (s_snap->active & 0x04) != 0;
	for (int offs = 0x1000 - 0x20; offs >= 0; offs -= 0x20) {
		const uint32_t c = obj[offs];
		const int attr = obj[offs + 1];
		int sy = obj[offs + 2], sx = obj[offs + 3];
		const uint16_t c0 = (uint16_t)(512 + (attr & 0x0f) * 16);
		int flipx = attr & 0x0100, flipy = attr & 0x0200;
		if (attr & 0x400) {   // large sprite
			if (fs) {
				sx = 480 - sx;
				sy = 224 - sy;
				flipx = !flipx;
				flipy = !flipy;
			}
			uint32_t c1 = c, c2 = c + 1, c3 = c + 16, c4 = c + 17, t;
			if (flipx) {
				t = c1; c1 = c2; c2 = t;
				t = c3; c3 = c4; c4 = t;
			}
			if (flipy) {
				t = c1; c1 = c3; c3 = t;
				t = c2; c2 = c4; c4 = t;
			}
			spr16(invert(c1), c0, flipx, flipy, sx, sy);
			spr16(invert(c2), c0, flipx, flipy, sx + 16, sy);
			spr16(invert(c3), c0, flipx, flipy, sx, sy + 16);
			spr16(invert(c4), c0, flipx, flipy, sx + 16, sy + 16);
		} else {
			if (fs) {
				sx = 496 - sx;
				sy = 240 - sy;
				flipx = !flipx;
				flipy = !flipy;
			}
			spr16(invert(c), c0, flipx, flipy, sx, sy);
		}
	}
}

#define BANDS 8
static int s_phase;
static void sf_band(int band)
{
	for (int y = VIS_H * band / BANDS; y < VIS_H * (band + 1) / BANDS; y++) {
		uint16_t *d = s_frame[y];
		const int by = y + VIS_Y0;
		if (s_phase == 0) {   // BG (or pen 0), FG
			if (s_snap->active & 0x20)
				layer_line(0, by, d);
			else
				memset(d, 0, VIS_W * 2);
			if (s_snap->active & 0x40)
				layer_line(1, by, d);
		} else {              // (the sprites), the text
			if (s_snap->active & 0x08)
				tx_line(by, d);
			memcpy(rbx_video.bitmap + y * rbx_video.pitch, d, VIS_W * 2);
		}
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, sf_band };
static int sf_help(int helper) { return decoband_help(&s_bands, helper); }

static void sf_render(void)
{
	for (int i = 0; i < 0x400; i++) {   // xRGB_444
		const uint16_t p = s_snap->pal[i];
		rbx_video.palette[i] = (uint32_t)((((p >> 8) & 15) * 17) << 16 | (((p >> 4) & 15) * 17) << 8 | ((p & 15) * 17));
	}
	s_phase = 0;
	decoband_run(&s_bands);
	if (s_snap->active & 0x80)
		draw_sprites();
	s_phase = 1;
	decoband_run(&s_bands);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->vram, s_vram, sizeof(s_vram));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	memcpy(s_snap->obj, &s_hiram[0x3000], sizeof(s_snap->obj));
	s_snap->bgscroll = s_bgscroll;
	s_snap->fgscroll = s_fgscroll;
	s_snap->active = s_active;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void sf_reset(void)
{
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_hiram, 0, sizeof(s_hiram));
	s_bgscroll = s_fgscroll = 0;
	s_active = s_halted = s_prot_t0 = 0;
	s_latch = 0;
	s_bank = 0;
	s_ym_irq = s_a_irq = 0;
	memset(s_zram, 0, sizeof(s_zram));
	set_bank();
	z80_start();
	rbx_ym_reset(0);
	msm5205_reset(&s_msm[0]);
	msm5205_reset(&s_msm[1]);
	if (s_mach == M_SFJP)
		mcs51_reset(&s_mcu);
	s_mix_n = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_cpu_frac = s_z80_frac = s_ym_frac = s_a80_frac = s_mcu_frac = s_irq8k_frac = 0;
}

static void sf_exit(void);

static int sf_init(const rbx_game *g)
{
	const char *m = g->machine;
	s_mach = !strcmp(m, "sfjp") ? M_SFJP : !strcmp(m, "sfan") ? M_SFAN : !strcmp(m, "sfp") ? M_SFP : M_SFUS;
	uint32_t s1, s2, s3, s4, ts;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_arom = rbx_region("audio2", &s_arom_size);
	s_mrom = s_mach == M_SFJP ? rbx_region("protcpu", &s_mrom_size) : NULL;
	uint8_t *g1 = rbx_region("gfx1", &s1), *g2 = rbx_region("gfx2", &s2), *g3 = rbx_region("gfx3", &s3);
	uint8_t *g4 = rbx_region("gfx4", &s4);
	s_tilerom = rbx_region("tilerom", &ts);
	if (!s_rom || s_rom_size < 0x50000 || !s_zrom || !s_arom || !g1 || !g2 || !g3 || !g4 || !s_tilerom || ts < 0x40000 ||
	    (s_mach == M_SFJP && !s_mrom)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	{	// sprite_layout (RGN_FRAC(1,2)), char_layout
		static uint32_t xs[16], ys[16], xc[8], yc[8];
		for (int i = 0; i < 4; i++) {
			xs[i] = (uint32_t)i;
			xs[4 + i] = (uint32_t)(4 * 2 + i);
			xs[8 + i] = (uint32_t)(4 * 2 * 2 * 16 + i);
			xs[12 + i] = (uint32_t)(4 * 2 * 2 * 16 + 8 + i);
			xc[i] = (uint32_t)i;
			xc[4 + i] = (uint32_t)(4 * 2 + i);
		}
		for (int i = 0; i < 16; i++)
			ys[i] = (uint32_t)(i * 16);
		for (int i = 0; i < 8; i++)
			yc[i] = (uint32_t)(i * 16);
		const uint32_t h1 = s1 * 4, h2 = s2 * 4, h3 = s3 * 4;
		const uint32_t p1[4] = { 4, 0, h1 + 4, h1 }, p2[4] = { 4, 0, h2 + 4, h2 }, p3[4] = { 4, 0, h3 + 4, h3 };
		static const uint32_t pc[2] = { 4, 0 };
		const rbx_gfx_layout l1 = { 16, 16, 4, p1, xs, ys, 64 * 8 }, l2 = { 16, 16, 4, p2, xs, ys, 64 * 8 };
		const rbx_gfx_layout l3 = { 16, 16, 4, p3, xs, ys, 64 * 8 }, lc = { 8, 8, 2, pc, xc, yc, 16 * 8 };
		if (rbx_gfx_decode(&s_gfxbg, &l1, g1, s1 / 2) || rbx_gfx_decode(&s_gfxfg, &l2, g2, s2 / 2) ||
		    rbx_gfx_decode(&s_gfxspr, &l3, g3, s3 / 2) || rbx_gfx_decode(&s_gfxtx, &lc, g4, s4)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			sf_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;   // YM2151 0.60 to both sides: (L + R) / 2
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 77;
	yc.irq = ym_irq;
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		sf_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 0x400)) {
		rbx_fail("%s: out of memory", g->set);
		sf_exit();
		return -1;
	}
	msm5205_init(&s_msm[0], 384000, MSM5205_SEX_4B, NULL);
	msm5205_init(&s_msm[1], 384000, MSM5205_SEX_4B, NULL);
	if (s_mach == M_SFJP) {
		mcs51_init(&s_mcu, s_mrom, s_mrom_size);
		s_mcu.port_out = mcu_port_out;
		s_mcu.xdata_r = mcu_xdata_r;
		s_mcu.xdata_w = mcu_xdata_w;
	}
	memset(s_zmap, 0, sizeof(s_zmap));
	for (uint32_t p = 0; p < 0x80 && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	memset(s_amap, 0, sizeof(s_amap));
	for (uint32_t p = 0; p < 0x80 && p * 256 + 256 <= s_arom_size; p++)
		s_amap[p] = s_arom + p * 256;
	static const char *const names[4][7] = {
		{ "IN0", "IN1", NULL, NULL, "DSW1", "DSW2", "SYSTEM" },          // sfus
		{ "IN0", "IN1", "PUNCH", "KICK", "DSW1", "DSW2", "SYSTEM" },     // sfan
		{ "IN0", "IN1", "IN2", NULL, "DSW1", "DSW2", "SYSTEM" },         // sfjp
		{ "IN0", "IN1", "PUNCH", "KICK", "DSW1", "DSW2", "SYSTEM" },     // sfp (sfan's map)
	};
	for (int i = 0; i < 7; i++)
		s_port[i] = names[s_mach][i] ? rbx_port_find(names[s_mach][i]) : -1;
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);   // (MAME's DIVx / MULx / ADDQ.W An times)
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x4ffff, s_rom, 0x50000, 1, 0);
	m68k_bus_map(0x800000, 0x800fff, s_vram, sizeof(s_vram), 1, 1);
	m68k_bus_map(0xb00000, 0xb007ff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus_map(0xff8000, 0xffffff, s_hiram, sizeof(s_hiram), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	sf_reset();
	return 0;
}

static void sf_exit(void)
{
	m68k_set_mame_timing(0);
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gfxbg);
	rbx_gfx_free(&s_gfxfg);
	rbx_gfx_free(&s_gfxspr);
	rbx_gfx_free(&s_gfxtx);
	free(s_snap);
	s_snap = NULL;
}

static void sf_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0 + VIS_H) {   // the vblank: the frame drawn, IRQ 1 (the prototype IRQ 6) held
			vblank_capture();
			m68k_bus_hold_irq(s_mach == M_SFP ? 6 : 1);
		}
		if (s_mach == M_SFJP)
			for (int k = 0; k < SLICES; k++) {   // (the 68000 and the MCU in quarter lines: it works while the 68000 waits)
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

static int sf_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[6] = { s_cpu_frac, s_z80_frac, s_ym_frac, s_a80_frac, s_mcu_frac, s_irq8k_frac };
	uint16_t regs[10] = { s_bgscroll, s_fgscroll, s_active, s_halted, s_prot_t0, s_latch, s_bank,
	                      (uint16_t)s_ym_irq, (uint16_t)s_a_irq, 0 };
	z80 z = s_z80, z2 = s_a80;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "hiram", s_hiram, sizeof(s_hiram));
	rc |= rbx_state_io(ctx, "regs", regs, sizeof(regs));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= rbx_state_io(ctx, "a80", &z2, sizeof(z2));
	rc |= rbx_state_io(ctx, "msm", s_msm, sizeof(s_msm));
	if (s_mach == M_SFJP)
		rc |= rbx_state_io(ctx, "mcu", &s_mcu.st, sizeof(s_mcu.st));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		s_bgscroll = regs[0];
		s_fgscroll = regs[1];
		s_active = (uint8_t)regs[2];
		s_halted = (uint8_t)regs[3];
		s_prot_t0 = (uint8_t)regs[4];
		s_latch = (uint8_t)regs[5];
		s_bank = (uint8_t)regs[6];
		s_ym_irq = regs[7];
		s_a_irq = regs[8];
		set_bank();
		z.read_byte = z80_read;
		z.write_byte = z80_write;
		z.port_in = z80_in;
		z.port_out = z80_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		s_z80 = z;
		z2.read_byte = a80_read;
		z2.write_byte = a80_write;
		z2.port_in = a80_in;
		z2.port_out = a80_out;
		z2.userdata = NULL;
		z2.read_map = s_amap;
		s_a80 = z2;
		s_msm[0].vck = s_msm[1].vck = NULL;
		s_cpu_frac = fr[0];
		s_z80_frac = fr[1];
		s_ym_frac = fr[2];
		s_a80_frac = fr[3];
		s_mcu_frac = fr[4];
		s_irq8k_frac = fr[5];
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int sf_save(void *ctx) { return sf_state(ctx, 0); }
static int sf_load(void *ctx) { return sf_state(ctx, 1); }
static int sf_supported(const char *machine)
{
	return !strcmp(machine, "sfus") || !strcmp(machine, "sfan") || !strcmp(machine, "sfjp") || !strcmp(machine, "sfp");
}

const rbx_driver rbx_drv_sf = { "Capcom Street Fighter", sf_init, sf_exit, sf_reset, sf_frame, sf_save, sf_load,
                                sf_supported, sf_render, sf_help };
