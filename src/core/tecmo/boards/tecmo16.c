/*  Tecmo 16-bit: Final Star Force, Riot and Ganbare Ginkun. Ported from MAME src/mame/tecmo/tecmo16.cpp (BSD-3-Clause,
    copyright-holders: Hiroshi Hagiwara, David Haywood).
    68000 at 12 MHz, IRQ 5 on the vblank line; two 16x16 layers and an 8x8 text layer (pen 0 transparent), the Ninja
    Gaiden sprites drawn at the vblank from the buffer (shown the next frame), all through the Tecmo mixer (RGB).
    Sound: Z80 at 4 MHz, YM2151 (its IRQ), OKI6295 at 1 MHz, the latch on the NMI.
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

#define CPU_CLOCK 12000000
#define Z80_CLOCK 4000000
#define YM_CLOCK 4000000
#define OKI_CLOCK 1000000
#define FRAME_MHZ 59170          // set_refresh_hz(59.17)
#define LINES 256
#define VIS_Y0 16                // set_visarea(0, 255, 16, 239)
#define VIS_H 224
#define VIS_W 256
#define VBL_END 15               // (set_vblank_time(1000 us): about 15 lines)

enum { M_FSTARFRC, M_GINKUN, M_RIOT };

static int s_mach;
static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static uint16_t (*s_ram_hp)[0x2000];          // 0x100000 .. 0x103fff
#define s_ram (*s_ram_hp)
static uint16_t s_charram[0x800];       // 0x110000 .. 0x110fff (64x32)
static uint16_t s_vram[2][0x800], s_cram[2][0x800];   // FG, BG: tile and color (fstarfrc 32x32, the others 64x32)
static uint16_t (*s_ram2_hp)[0x3000];         // fstarfrc 0x122000 .. 0x127fff; ginkun 0x124000 .. 0x124fff
#define s_ram2 (*s_ram2_hp)
static uint16_t s_spriteram[0x800];     // 0x130000 .. 0x130fff
static uint16_t s_spritebuf[0x800];
static uint16_t s_pal[0x1000];          // 0x140000 .. 0x141fff
static uint16_t s_scroll_x[2], s_scroll_y[2], s_char_x, s_char_y;
static int s_tx_sy, s_tx_dy;            // (the text layer's scrolly and scrolldy as MAME keeps them)
static uint8_t s_flip;
static rbx_gfx s_gfxtx, s_gfxbg, s_gfxspr;
static int s_port[4];
static uint64_t s_cpu_frac, s_z80_frac, s_ym_frac;
static tecmo_mix_t s_mix_cfg;
static uint16_t (*s_sprbmp_hp)[2][VIS_W * VIS_H];   // (drawn at the vblank, shown the next frame)
#define s_sprbmp (*s_sprbmp_hp)
static int s_sprcur;
static struct {
	uint16_t charram[0x800], vram[2][0x800], cram[2][0x800], pal[0x1000];
	uint16_t sx[2], sy[2], cx;
	int tx_sy;
	uint8_t flip;
	int spr;
} *s_snap;

// ---- sound ----
static z80 s_z80;
static uint8_t s_zram[0xc00], s_zram2[2];
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
	if (a < 0xf000) return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0xfc00) return s_zram[a - 0xf000];
	if (a == 0xfc00) return oki_read(&s_oki);
	if (a == 0xfc04 || a == 0xfc05) return rbx_ym_read(0, a & 1);
	if (a == 0xfc08) {
		s_latch_pending = 0;
		return s_latch;
	}
	if (a >= 0xfffe) return s_zram2[a - 0xfffe];
	return 0;
}
static void z80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xf000 && a < 0xfc00) s_zram[a - 0xf000] = v;
	else if (a == 0xfc00) oki_write(&s_oki, v);
	else if (a == 0xfc04 || a == 0xfc05) rbx_ym_write(0, a & 1, v);
	else if (a >= 0xfffe) s_zram2[a - 0xfffe] = v;
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
	z80_run(&s_z80, s_z80.cyc + (unsigned long)cycles, &s_ym_irq, 0);
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&s_ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
	const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
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
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

// ---- bus ----
static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	switch (a) {
	case 0x150020: return (uint16_t)rbx_port_read(s_port[0]);   // EXTRA
	case 0x150030: return (uint16_t)rbx_port_read(s_port[1]);   // DSW2
	case 0x150040: return (uint16_t)rbx_port_read(s_port[2]);   // DSW1
	case 0x150050: return (uint16_t)rbx_port_read(s_port[3]);   // P1_P2
	}
	return 0;
}
static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static inline void combine(uint16_t *w, uint16_t v, uint16_t mask) { *w = (uint16_t)((*w & ~mask) | (v & mask)); }
static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	switch (a) {
	case 0x150000: s_flip = v & 1; break;
	case 0x150010:
		if (mask & 0x00ff) {   // the sound latch (0x150011)
			s_latch = (uint8_t)v;
			if (!s_latch_pending)
				z80_gen_nmi(&s_z80);
			s_latch_pending = 1;
		}
		break;
	case 0x150020: m68k_bus_set_line(5, 0); break;   // irq_150021_w
	case 0x160000: combine(&s_char_x, v, mask); break;
	case 0x160006:
		combine(&s_char_y, v, mask);
		s_tx_sy = s_char_y - 16;
		break;
	case 0x16000c: combine(&s_scroll_x[0], v, mask); break;
	case 0x160012: combine(&s_scroll_y[0], v, mask); break;
	case 0x160018: combine(&s_scroll_x[1], v, mask); break;
	case 0x16001e: combine(&s_scroll_y[1], v, mask); break;
	}
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
// one line of a 16x16 layer (pen 0 transparent: 0), which 0 = FG (32 colors: the blend bit), 1 = BG
static void tile16_line(int which, int y, uint16_t *dst)
{
	const int flip = s_snap->flip, cols = s_mach == M_FSTARFRC ? 32 : 64;
	const uint32_t wmask = (uint32_t)cols * 16 - 1;
	const uint16_t *vram = s_snap->vram[which], *cram = s_snap->cram[which];
	const uint32_t ty = (uint32_t)(flip ? 255 - y + s_snap->sy[which] : y + s_snap->sy[which]) & 511;
	const uint32_t row = (ty >> 4) * (uint32_t)cols, fy = ty & 15;
	uint32_t tx = (uint32_t)(flip ? VIS_W - 1 + s_snap->sx[which] : s_snap->sx[which]) & wmask;
	const uint16_t cmask = which ? 0x0f : 0x1f;
	int x = 0;
	while (x < VIS_W) {
		const uint32_t i = row + (tx >> 4);
		const uint32_t code = (vram[i] & 0x1fffu) % s_gfxbg.count;
		int px = (int)(tx & 15), n = flip ? px + 1 : 16 - px;
		if (n > VIS_W - x)
			n = VIS_W - x;
		if (!s_gfxbg.used[code])
			memset(dst + x, 0, (size_t)n * 2);
		else {
			const uint16_t c0 = (uint16_t)((cram[i] & cmask) * 16);
			const uint8_t *src = s_gfxbg.pix + code * 256 + fy * 16;
			for (int k = 0; k < n; k++) {
				const uint8_t p = src[flip ? px - k : px + k];
				dst[x + k] = p ? (uint16_t)(c0 + p) : 0;
			}
		}
		x += n;
		tx = (tx + (uint32_t)(flip ? -n : n)) & wmask;
	}
}

static void text_line(int y, uint16_t *dst)   // 64x32 of 8x8 (512x256)
{
	const int flip = s_snap->flip;
	const uint32_t ty = (uint32_t)(flip ? 255 - y - s_tx_dy + s_snap->tx_sy : y + s_snap->tx_sy - s_tx_dy) & 255;
	const uint32_t row = (ty >> 3) * 64, fy = ty & 7;
	uint32_t tx = (uint32_t)(flip ? VIS_W - 1 + s_snap->cx : s_snap->cx) & 511;
	int x = 0;
	while (x < VIS_W) {
		const uint16_t t = s_snap->charram[row + (tx >> 3)];
		const uint32_t code = (t & 0x0fffu) % s_gfxtx.count;
		int px = (int)(tx & 7), n = flip ? px + 1 : 8 - px;
		if (n > VIS_W - x)
			n = VIS_W - x;
		if (!s_gfxtx.used[code])
			memset(dst + x, 0, (size_t)n * 2);
		else {
			const uint16_t c0 = (uint16_t)(0x100 + (t >> 12) * 16);
			const uint8_t *src = s_gfxtx.pix + code * 64 + fy * 8;
			for (int k = 0; k < n; k++) {
				const uint8_t p = src[flip ? px - k : px + k];
				dst[x + k] = p ? (uint16_t)(c0 + p) : 0;
			}
		}
		x += n;
		tx = (tx + (uint32_t)(flip ? -n : n)) & 511;
	}
}

#define BANDS 8
static uint32_t (*s_pal_rgb_hp)[0x1000];
#define s_pal_rgb (*s_pal_rgb_hp)
static int s_565;

static void t16_band(int band)
{
	const int r0 = VIS_H * band / BANDS, r1 = VIS_H * (band + 1) / BANDS;
	tecmo_mix_t mix = s_mix_cfg;
	mix.rand = (uint32_t)band * 0x9e3779b9u;
	const uint16_t *spr = s_sprbmp[s_snap->spr];
	for (int y = r0; y < r1; y++) {
		uint16_t bg[VIS_W], fg[VIS_W], tx[VIS_W];
		uint32_t d[VIS_W];
		tile16_line(1, y + VIS_Y0, bg);
		tile16_line(0, y + VIS_Y0, fg);
		text_line(y + VIS_Y0, tx);
		tecmo_mix_line(&mix, s_pal_rgb, bg, fg, tx, spr + y * VIS_W, d, VIS_W);
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
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, t16_band };
static int t16_help(int helper) { return decoband_help(&s_bands, helper); }

static void t16_render(void)
{
	if (!rbx_video.out)
		return;
	for (int i = 0; i < 0x1000; i++) {   // xBGR_444
		const uint16_t p = s_snap->pal[i];
		s_pal_rgb[i] = (uint32_t)(((p & 15) * 17) << 16 | (((p >> 4) & 15) * 17) << 8 | (((p >> 8) & 15) * 17));
	}
	s_565 = rbx_video.out_bpp == 2 && rbx_video.out_color(0xff, 0, 0) == 0xf800 && rbx_video.out_color(0, 0xff, 0) == 0x07e0 &&
	        rbx_video.out_color(0, 0, 0xff) == 0x001f;
	decoband_run(&s_bands);
	rbx_video.out_done = 1;
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->charram, s_charram, sizeof(s_charram));
	memcpy(s_snap->vram, s_vram, sizeof(s_vram));
	memcpy(s_snap->cram, s_cram, sizeof(s_cram));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	memcpy(s_snap->sx, s_scroll_x, sizeof(s_scroll_x));
	memcpy(s_snap->sy, s_scroll_y, sizeof(s_scroll_y));
	s_snap->cx = s_char_x;
	s_snap->tx_sy = s_tx_sy;
	s_snap->flip = s_flip;
	s_snap->spr = s_sprcur;   // (the bitmap the last vblank drew)
	// screen_vblank: the next frame's sprite bitmap from the buffer, then copy()
	s_sprcur ^= 1;
	tecmo_spr_gaiden_draw(&s_gfxspr, s_spritebuf, s_mach == M_RIOT ? 0 : 2, 0, s_flip, 256, s_sprbmp[s_sprcur], VIS_W, VIS_H,
	                      VIS_Y0);
	memcpy(s_spritebuf, s_spriteram, sizeof(s_spriteram));
	rbx_video_frame_done();
}

// ---- init / frame ----
static void t16_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_charram, 0, sizeof(s_charram));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_cram, 0, sizeof(s_cram));
	memset(s_ram2, 0, sizeof(s_ram2));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_spritebuf, 0, sizeof(s_spritebuf));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_sprbmp, 0, sizeof(s_sprbmp));
	s_sprcur = 0;
	memset(s_scroll_x, 0, sizeof(s_scroll_x));
	memset(s_scroll_y, 0, sizeof(s_scroll_y));
	s_char_x = s_char_y = 0;
	s_tx_sy = s_mach == M_FSTARFRC ? -16 : 0;   // (video_start: fstarfrc set_scrolly(-16); riot set_scrolldy(-16))
	s_tx_dy = s_mach == M_RIOT ? -16 : 0;
	s_flip = 0;
	s_latch = 0;
	s_latch_pending = 0;
	s_ym_irq = 0;
	memset(s_zram, 0, sizeof(s_zram));
	memset(s_zram2, 0, sizeof(s_zram2));
	z80_start();
	rbx_ym_reset(0);
	oki_reset(&s_oki);
	s_oki_acc = 0;
	s_oki_hold = 0;
	s_mix_n = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_cpu_frac = s_z80_frac = s_ym_frac = 0;
}

static void t16_exit(void);

static int t16_init(const rbx_game *g)
{
	if (!s_pal_rgb_hp && !(s_pal_rgb_hp = calloc(1, sizeof(*s_pal_rgb_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ram_hp && !(s_ram_hp = calloc(1, sizeof(*s_ram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ram2_hp && !(s_ram2_hp = calloc(1, sizeof(*s_ram2_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_sprbmp_hp && !(s_sprbmp_hp = calloc(1, sizeof(*s_sprbmp_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_mach = !strcmp(g->machine, "riot") ? M_RIOT : !strcmp(g->machine, "ginkun") ? M_GINKUN : M_FSTARFRC;
	uint32_t txs, bgs, sps, osize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *txt = rbx_region("fgtiles", &txs), *bgt = rbx_region("bgtiles", &bgs), *spr = rbx_region("sprites", &sps);
	uint8_t *oki = rbx_region("oki", &osize);
	if (!s_rom || s_rom_size < 0x80000 || !s_zrom || !txt || !bgt || !spr || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	{	// gfx_8x8x4_packed_msb, gfx_8x8x4_row_2x2_group_packed_msb
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
		    rbx_gfx_decode(&s_gfxspr, &l8, spr, sps)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			t16_exit();
			return -1;
		}
	}
	memset(&s_mix_cfg, 0, sizeof(s_mix_cfg));
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
	s_mix_cfg.fgblend_source = 0x0800 + 0x100;
	s_mix_cfg.revspritetile = 3;
	s_mix_cfg.bgpen = 0x000 + 0x300;
	s_mix_cfg.bgpen_blend = 0x400 + 0x300;
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;   // YM2151 4 MHz 0.60 and OKI 0.40 to both sides: (L + R) / 2
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 77;
	yc.irq = ym_irq;
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		t16_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 1)) {
		rbx_fail("%s: out of memory", g->set);
		t16_exit();
		return -1;
	}
	oki_init(&s_oki, OKI_CLOCK, 1, oki, osize, 102);
	memset(s_zmap, 0, sizeof(s_zmap));
	for (uint32_t p = 0; p < 0xf0 && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	static const char *const names[4] = { "EXTRA", "DSW2", "DSW1", "P1_P2" };
	for (int i = 0; i < 4; i++)
		s_port[i] = rbx_port_find(names[i]);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);   // (MAME's DIVx / MULx / ADDQ.W An times)
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x7ffff, s_rom, 0x80000, 1, 0);
	m68k_bus_map(0x100000, 0x103fff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x110000, 0x110fff, s_charram, sizeof(s_charram), 1, 1);
	if (s_mach == M_FSTARFRC) {
		m68k_bus_map(0x120000, 0x1207ff, s_vram[0], 0x800, 1, 1);
		m68k_bus_map(0x120800, 0x120fff, s_cram[0], 0x800, 1, 1);
		m68k_bus_map(0x121000, 0x1217ff, s_vram[1], 0x800, 1, 1);
		m68k_bus_map(0x121800, 0x121fff, s_cram[1], 0x800, 1, 1);
		m68k_bus_map(0x122000, 0x127fff, s_ram2, 0x6000, 1, 1);
	} else {
		m68k_bus_map(0x120000, 0x120fff, s_vram[0], 0x1000, 1, 1);
		m68k_bus_map(0x121000, 0x121fff, s_cram[0], 0x1000, 1, 1);
		m68k_bus_map(0x122000, 0x122fff, s_vram[1], 0x1000, 1, 1);
		m68k_bus_map(0x123000, 0x123fff, s_cram[1], 0x1000, 1, 1);
		m68k_bus_map(0x124000, 0x124fff, s_ram2, 0x1000, 1, 1);
	}
	m68k_bus_map(0x130000, 0x130fff, s_spriteram, sizeof(s_spriteram), 1, 1);
	m68k_bus_map(0x140000, 0x141fff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	t16_reset();
	return 0;
}

static void t16_exit(void)
{
	m68k_set_mame_timing(0);
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gfxtx);
	rbx_gfx_free(&s_gfxbg);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
	free(s_pal_rgb_hp);
	s_pal_rgb_hp = NULL;
	free(s_ram_hp);
	s_ram_hp = NULL;
	free(s_ram2_hp);
	s_ram2_hp = NULL;
	free(s_sprbmp_hp);
	s_sprbmp_hp = NULL;
}

static void t16_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0 + VIS_H) {   // the vblank: the frame drawn, IRQ 5 up (the vblank line)
			vblank_capture();
			m68k_bus_set_line(5, 1);
		} else if (line == VIS_Y0 + VIS_H + VBL_END)
			m68k_bus_set_line(5, 0);
		m68k_execute(rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		sound_line();
	}
	sound_frame_end();
}

static int t16_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[3] = { s_cpu_frac, s_z80_frac, s_ym_frac };
	int32_t regs[10] = { s_scroll_x[0], s_scroll_x[1], s_scroll_y[0], s_scroll_y[1], s_char_x, s_char_y, s_tx_sy, s_flip,
	                     s_sprcur, 0 };
	int32_t misc[5] = { s_latch, s_latch_pending, s_ym_irq, (int32_t)s_oki_acc, s_oki_hold };
	z80 z = s_z80;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "char", s_charram, sizeof(s_charram));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "cram", s_cram, sizeof(s_cram));
	rc |= rbx_state_io(ctx, "ram2", s_ram2, sizeof(s_ram2));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "sprb", s_spritebuf, sizeof(s_spritebuf));
	rc |= rbx_state_io(ctx, "sbmp", s_sprbmp, sizeof(s_sprbmp));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "regs", regs, sizeof(regs));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= rbx_state_io(ctx, "zram2", s_zram2, sizeof(s_zram2));
	rc |= rbx_state_io(ctx, "oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	rc |= rbx_ym_state(0, ctx, load);
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
		s_scroll_x[0] = (uint16_t)regs[0];
		s_scroll_x[1] = (uint16_t)regs[1];
		s_scroll_y[0] = (uint16_t)regs[2];
		s_scroll_y[1] = (uint16_t)regs[3];
		s_char_x = (uint16_t)regs[4];
		s_char_y = (uint16_t)regs[5];
		s_tx_sy = regs[6];
		s_flip = (uint8_t)regs[7];
		s_sprcur = regs[8] & 1;
		s_latch = (uint8_t)misc[0];
		s_latch_pending = misc[1];
		s_ym_irq = misc[2];
		s_oki_acc = (uint32_t)misc[3];
		s_oki_hold = misc[4];
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int t16_save(void *ctx) { return t16_state(ctx, 0); }
static int t16_load(void *ctx) { return t16_state(ctx, 1); }
static int t16_supported(const char *machine)
{
	return !strcmp(machine, "base") || !strcmp(machine, "ginkun") || !strcmp(machine, "riot");
}

const rbx_driver rbx_drv_tecmo16 = { "Tecmo 16", t16_init, t16_exit, t16_reset, t16_frame, t16_save, t16_load,
                                     t16_supported, t16_render, t16_help };
