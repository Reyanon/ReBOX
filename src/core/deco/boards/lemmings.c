/*  Data East DE-0357: Lemmings (US prototype). Ported from MAME src/mame/dataeast/lemmings.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail).
    68000 at 14 MHz (IRQ 6 held at vblank), two sprite chips 52 / 71 (their lists buffered by a write, then again at
    vblank) drawn raw and copied by their priority bit, a 2048 x 256 pixel layer (4 bits a pixel, windowed in two
    halves for two players) and a VRAM tilemap whose characters are themselves written as pixels, the 146 (inputs and
    the sound latch), trackballs. The sound: 6809E with YM2151 (stereo) and an OKI; the latch on its FIRQ, the YM on
    its IRQ.
    The capture takes the pixel layer's visible words (the window of each half) and the VRAM characters; the render
    thread draws both layers with the sprites.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/m6809/m6809.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../machine/deco146.h"
#include "../video/decoband.h"
#include "../video/decospr.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 14000000       // 28 MHz / 2
#define SND_CLOCK 1342500        // 32.22 MHz / 24
#define YM_CLOCK (32220000 / 9)
#define OKI_CLOCK 1023924        // pin 7 high
#define FRAME_MHZ 60000          // set_refresh_hz(60)
#define LINES 256                // set_size(40*8, 32*8)
#define VIS_Y0 16                // set_visarea(0, 319, 16, 239)
#define VIS_H 224
#define VIS_W 320
#define VBL_LINES 8              // vblank 529 us
#define BLACK 0x400              // (palette_device::black_pen: a pen past the 1024)
#define BANDS 4
#define WIN_W 164                // (the pixel layer words a window needs: 320 pixels from any scroll)

static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static uint16_t s_ram[0x8000];             // 0x100000 .. 0x10ffff
static uint16_t s_spriteram[2][0x400], s_sprbuf[2][0x400], s_sprtri[2][0x400];
static uint16_t s_pal[0x800];              // xBGR_888, 1024 entries of two words
static uint16_t s_control[8];              // 0x170000 (0: the pixel layer's x scroll, 2: the second half's, 6: bit 1 windowed)
static uint16_t s_vram[0x1000];            // 0x200000 .. 0x201fff
static uint16_t s_vram_ext[0x800];         // 0x202000 .. 0x202fff
static uint16_t *s_pixel0;                 // 0x300000 .. 0x37ffff (0x40000 words)
static uint16_t *s_pixel1;                 // 0x380000 .. 0x39ffff (0x10000 words)
static uint8_t *s_vram_buf;                // the VRAM characters as written by pixel_1_w (2048 of 64 pixels)
static deco146_t s_prot;
static decospr_t s_spr[2];
static rbx_gfx s_gfxspr[2];
static int s_port_in, s_port_sys, s_port_dsw;
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac, s_snd_frac, s_ym_frac;
static uint8_t s_track[2][2];              // the trackballs' counters (AN0 .. AN3)
static uint16_t s_sbm[2][VIS_W * VIS_H];
static struct {
	uint16_t win[2][VIS_H][WIN_W];      // the pixel layer's words of each window (rows from the first visible one)
	int nseg, xs[2], x0[2], x1[2], w0[2];
	uint16_t vram[0x1000];
	uint8_t vbuf[2048 * 64];
	uint16_t spriteram[2][0x400], pal[0x800];
	uint32_t frame;
} *s_snap;

// ---- sound: 6809E, YM2151, OKI ----
static m6809_t s_snd;
static uint8_t s_sram[0x800];
static okim6295_t s_oki;
static uint32_t s_oki_acc;
static int32_t s_oki_hold;
static int32_t *s_mix_l, *s_mix_r;
static int s_mix_n, s_mix_cap;

static void ym_irq(int state) { m6809_set_irq(&s_snd, state); }
static void latch_irq(void *u, int state) { (void)u; m6809_set_firq(&s_snd, state); }

static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x0800) return s_sram[a];
	if (a == 0x0800 || a == 0x0801) return rbx_ym_read(0, a & 1);
	if (a == 0x1000) return oki_read(&s_oki);
	if (a == 0x1800) return deco146_soundlatch_r(&s_prot);
	if (a >= 0x8000) return a < s_zrom_size ? s_zrom[a] : 0xff;
	return 0;
}
static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0800) s_sram[a] = v;
	else if (a == 0x0800 || a == 0x0801) rbx_ym_write(0, a & 1, v);
	else if (a == 0x1000) oki_write(&s_oki, v);
}

// the 6809 and the chips through one line: the YM's samples (on the worker) and the OKI held at the YM's rate
static void sound_line(void)
{
	s_snd_frac += (uint64_t)SND_CLOCK * 1000;
	const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
	const int64_t cyc = (int64_t)(s_snd_frac / den);
	s_snd_frac -= (uint64_t)cyc * den;
	m6809_run_until(&s_snd, s_snd.cycles + cyc);
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&s_ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
	const int n = rbx_ym_advance_lr(0, clocks, s_mix_l + s_mix_n, s_mix_r + s_mix_n, s_mix_cap - s_mix_n);
	const uint32_t yr = rbx_ym_rate(0);
	for (int i = 0; i < n; i++) {
		s_oki_acc += s_oki.rate;
		while (s_oki_acc >= yr) {
			s_oki_acc -= yr;
			s_oki_hold = oki_sample(&s_oki);
		}
		s_mix_l[s_mix_n + i] += s_oki_hold;
		s_mix_r[s_mix_n + i] += s_oki_hold;
	}
	s_mix_n += n;
}

static void sound_frame_end(void)
{
	rbx_ym_async_take_lr(0, s_mix_l, s_mix_r, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put_stereo(s_mix_l[i], s_mix_r[i]);
	memset(s_mix_l, 0, sizeof(int32_t) * (size_t)s_mix_n);
	memset(s_mix_r, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

// ---- the 146's ports ----
static uint16_t port_a(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_in); }
static uint16_t port_b(void *u)
{
	(void)u;
	const int vbl = s_line >= VIS_Y0 + VIS_H && s_line < VIS_Y0 + VIS_H + VBL_LINES;
	rbx_port_custom(s_port_sys, 0x0008, vbl ? 0x0008 : 0);
	return (uint16_t)rbx_port_read(s_port_sys);
}
static uint16_t port_c(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_dsw); }

// ---- bus ----
// pixel_1_w: the word's two pixels into the VRAM character they fall in
static void pixel1_update(uint32_t offset)
{
	const uint16_t src = s_pixel1[offset];
	const int sy = (int)((offset << 1) >> 9);
	int sx = (int)((offset << 1) & 0x1ff);
	const int tile = ((sx / 8) * 32) + (sy / 8);
	s_vram_buf[(tile * 64) + (sx & 7) + ((sy & 7) * 8)] = (src >> 8) & 0xf;
	sx += 1;
	s_vram_buf[(tile * 64) + (sx & 7) + ((sy & 7) * 8)] = src & 0xf;
}

static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x170000 && a < 0x170010) return s_control[(a >> 1) & 7];
	if (a >= 0x190000 && a < 0x190010) {   // trackball_r
		const uint32_t o = (a >> 1) & 7;
		if (o & 2)
			return 0;
		const uint32_t n = (o & 1) | ((o & 4) >> 1);
		return s_track[n >> 1][n & 1];
	}
	if (a >= 0x1a0000 && a < 0x1a4000) return deco146_read(&s_prot, (uint16_t)(a & 0x7fe), NULL);
	if (a >= 0x300000 && a < 0x380000) return s_pixel0[(a - 0x300000) >> 1];
	if (a >= 0x380000 && a < 0x3a0000) return s_pixel1[(a - 0x380000) >> 1];
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
	if (a >= 0x300000 && a < 0x380000) {
		uint16_t *w = &s_pixel0[(a - 0x300000) >> 1];
		*w = (uint16_t)((*w & ~mask) | (v & mask));
	} else if (a >= 0x380000 && a < 0x3a0000) {
		const uint32_t o = (a - 0x380000) >> 1;
		s_pixel1[o] = (uint16_t)((s_pixel1[o] & ~mask) | (v & mask));
		pixel1_update(o);
	} else if (a >= 0x170000 && a < 0x170010) {   // control_w (offset 4: watchdog or IRQ ack, not kept)
		const uint32_t o = (a >> 1) & 7;
		if (o != 4)
			s_control[o] = (uint16_t)((s_control[o] & ~mask) | (v & mask));
	} else if (a >= 0x1a0000 && a < 0x1a4000)
		deco146_write(&s_prot, (uint16_t)(a & 0x7fe), v, mask, NULL);
	else if (a == 0x1c0000)   // the sprite lists buffered (a write of 1 once a frame)
		memcpy(s_sprbuf[0], s_spriteram[0], sizeof(s_sprbuf[0]));
	else if (a == 0x1e0000)
		memcpy(s_sprbuf[1], s_spriteram[1], sizeof(s_sprbuf[1]));
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
// MAME copy_bitmap of a window: columns x0 .. x1 - 1 (its loops stop one short of the clip's right and bottom); the
// capture keeps the words they read
static void take_window(int seg, int xscroll, int x0, int x1)
{
	const int w0 = ((x0 - xscroll) & 0x7ff) >> 1, nw = (x1 - x0) / 2 + 2;
	s_snap->xs[seg] = xscroll;
	s_snap->x0[seg] = x0;
	s_snap->x1[seg] = x1;
	s_snap->w0[seg] = w0;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H - 1; y++) {
		const uint16_t *row = s_pixel0 + (size_t)(y & 0xff) * 1024;   // (2048 pixels, two a word)
		uint16_t *d = s_snap->win[seg][y - VIS_Y0];
		for (int i = 0; i < nw; i++)
			d[i] = row[(w0 + i) & 1023];
	}
}
static void draw_window(int seg, uint16_t *bm, int pitch, int r0, int r1)
{
	const int xs = s_snap->xs[seg], x0 = s_snap->x0[seg], x1 = s_snap->x1[seg], w0 = s_snap->w0[seg];
	for (int y = r0; y < r1 && y < VIS_H - 1; y++) {
		const uint16_t *row = s_snap->win[seg][y];
		uint16_t *d = bm + y * pitch;
		for (int x = x0; x < x1; x++) {
			const int sx = (x - xs) & 0x7ff;
			const uint16_t w = row[((sx >> 1) - w0) & 1023];
			const int p = sx & 1 ? w & 0xf : (w >> 8) & 0xf;
			if (p)
				d[x] = (uint16_t)(0x100 | p);
		}
	}
}
// the VRAM tilemap (64 x 32 characters by columns, no scroll, pen 0 clear)
static void draw_tiles(uint16_t *bm, int pitch, int r0, int r1)
{
	for (int y = r0; y < r1; y++) {
		const int sy = VIS_Y0 + y;
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < VIS_W; x++) {
			const uint16_t t = s_snap->vram[(x >> 3) * 32 + (sy >> 3)];
			const uint8_t p = s_snap->vbuf[(t & 0x7ff) * 64 + (sy & 7) * 8 + (x & 7)];
			if (p)
				d[x] = (uint16_t)(((t >> 12) & 0xf) * 16 + p);
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	// the pixel layer whole, or its halves at their own scrolls (control 6 bit 1: two players)
	const int x1 = -(int16_t)s_control[0], x0 = -(int16_t)s_control[2];
	if (!(s_control[6] & 2)) {
		s_snap->nseg = 1;
		take_window(0, x1, 0, VIS_W - 1);
	} else {
		s_snap->nseg = 2;
		take_window(0, x0, 0, 159);
		take_window(1, x1, 160, 319);
	}
	memcpy(s_snap->vram, s_vram, sizeof(s_vram));
	memcpy(s_snap->vbuf, s_vram_buf, sizeof(s_snap->vbuf));
	memcpy(s_snap->spriteram, s_sprtri, sizeof(s_sprtri));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

static void copy_sprites(const uint16_t *src, uint16_t *bm, int pitch, uint16_t pri, uint16_t pmask, uint16_t colbase,
                         int r0, int r1)
{
	for (int y = r0; y < r1; y++) {
		const uint16_t *s = src + y * VIS_W;
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < VIS_W; x++) {
			const uint16_t pix = s[x];
			if ((pix & 0xf) && (pix & pmask) == pri)
				d[x] = (uint16_t)((pix & 0xff) + colbase);
		}
	}
}
// a band of lines (the render thread and its helpers draw them apart)
static void lm_band(int band)
{
	const int r0 = VIS_H * band / BANDS, r1 = VIS_H * (band + 1) / BANDS;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	// (sprites are flipped relative to the tilemaps) chip 0: the second list with the second ROMs, chip 1 the first
	for (int c = 0; c < 2; c++) {
		decospr_t spr = s_spr[c];
		spr.flip_screen = 1;
		memset(s_sbm[c] + r0 * VIS_W, 0, (size_t)(r1 - r0) * VIS_W * 2);
		decospr_draw(&spr, s_snap->spriteram[c ? 0 : 1], 0x400, s_sbm[c] + r0 * VIS_W, NULL, VIS_W, VIS_W, r1 - r0,
		             VIS_Y0 + r0, 1, s_snap->frame);
	}
	for (int y = r0; y < r1; y++)
		for (int x = 0; x < VIS_W; x++)
			bm[y * pitch + x] = BLACK;
	copy_sprites(s_sbm[0], bm, pitch, 0x0800, 0x0800, 0x300, r0, r1);
	for (int k = 0; k < s_snap->nseg; k++)
		draw_window(k, bm, pitch, r0, r1);
	copy_sprites(s_sbm[1], bm, pitch, 0x0800, 0x0800, 0x200, r0, r1);
	copy_sprites(s_sbm[0], bm, pitch, 0x0000, 0x0800, 0x300, r0, r1);
	draw_tiles(bm, pitch, r0, r1);
	copy_sprites(s_sbm[1], bm, pitch, 0x0000, 0x0800, 0x200, r0, r1);
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, lm_band };
static int lm_help(int helper) { return decoband_help(&s_bands, helper); }

static void lm_render(void)
{
	for (int i = 0; i < 0x400; i++) {   // xBGR_888 (the entry's two words, big-endian)
		const uint32_t p = (uint32_t)s_snap->pal[2 * i] << 16 | s_snap->pal[2 * i + 1];
		rbx_video.palette[i] = (p & 0xff) << 16 | (p & 0xff00) | ((p >> 16) & 0xff);
	}
	rbx_video.palette[BLACK] = 0;
	decoband_run(&s_bands);
}

// ---- init / frame ----
static void lm_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_sprbuf, 0, sizeof(s_sprbuf));
	memset(s_sprtri, 0, sizeof(s_sprtri));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_control, 0, sizeof(s_control));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_vram_ext, 0, sizeof(s_vram_ext));
	memset(s_pixel0, 0, 0x40000 * 2);
	memset(s_pixel1, 0, 0x10000 * 2);
	memset(s_vram_buf, 0, 2048 * 64);
	memset(s_track, 0, sizeof(s_track));
	deco146_reset(&s_prot);
	memset(s_sram, 0, sizeof(s_sram));
	m6809_reset(&s_snd);
	rbx_ym_reset(0);
	oki_reset(&s_oki);
	s_oki_acc = 0;
	s_oki_hold = 0;
	s_mix_n = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = s_snd_frac = s_ym_frac = 0;
}

static void lm_exit(void);

static int lm_init(const rbx_game *g)
{
	uint32_t s1s, s2s, osize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *spr1 = rbx_region("sprites1", &s1s), *spr2 = rbx_region("sprites2", &s2s), *oki = rbx_region("oki", &osize);
	if (!s_rom || s_rom_size < 0x100000 || !s_zrom || !spr1 || !spr2 || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{	// sprite_layout: 16x16, planes 0, 8, 16, 24 (4 bytes a row of 8 pixels), the right half first
		static uint32_t x16[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x16[i] = (uint32_t)(8 * 4 * 16 + i);
			x16[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 8 * 4);
		static const uint32_t ps[4] = { 0, 8, 16, 24 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, x16, y16, 16 * 16 * 4 };
		if (rbx_gfx_decode(&s_gfxspr[0], &ls, spr1, s1s) || rbx_gfx_decode(&s_gfxspr[1], &ls, spr2, s2s)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			lm_exit();
			return -1;
		}
	}
	for (int c = 0; c < 2; c++) {   // (chip 0: gfx_lemmings_spr2, chip 1: spr1)
		memset(&s_spr[c], 0, sizeof(s_spr[c]));
		s_spr[c].gfx = &s_gfxspr[c ? 0 : 1];
		s_spr[c].raw = 1;   // (alloc_sprite_bitmap: raw shift 4)
		s_spr[c].raw_shift = 4;
	}
	deco146_init(&s_prot, 0);
	s_prot.magic_read_xor_on = 1;
	s_prot.port_a = port_a;
	s_prot.port_b = port_b;
	s_prot.port_c = port_c;
	s_prot.soundlatch_irq = latch_irq;
	s_snap = calloc(1, sizeof(*s_snap));
	s_pixel0 = (uint16_t *)malloc(0x40000 * 2);
	s_pixel1 = (uint16_t *)malloc(0x10000 * 2);
	s_vram_buf = (uint8_t *)malloc(2048 * 64);
	// sound: YM2151 3.58 MHz 0.45 (L / R), OKI 1.02 MHz 0.50 (both)
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 115;
	yc.irq = ym_irq;
	if (!s_snap || !s_pixel0 || !s_pixel1 || !s_vram_buf || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 2)) {
		rbx_fail("%s: out of memory", g->set);
		lm_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix_l = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_mix_r = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix_l || !s_mix_r || rbx_audio_setup_stereo(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, BLACK + 1)) {
		rbx_fail("%s: out of memory", g->set);
		lm_exit();
		return -1;
	}
	oki_init(&s_oki, OKI_CLOCK, 1, oki, osize, 128);
	m6809_init(&s_snd);
	for (int p = 0; p < 8; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = s_sram + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		if ((uint32_t)p * 256 + 256 <= s_zrom_size)
			s_snd.rpage[p] = s_zrom + p * 256;
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	s_port_in = rbx_port_find("INPUTS");
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_dsw = rbx_port_find("DSW");
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0xfffff, s_rom, 0x100000, 1, 0);
	m68k_bus_map(0x100000, 0x10ffff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x120000, 0x1207ff, s_spriteram[0], sizeof(s_spriteram[0]), 1, 1);
	m68k_bus_map(0x140000, 0x1407ff, s_spriteram[1], sizeof(s_spriteram[1]), 1, 1);
	m68k_bus_map(0x160000, 0x160fff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus_map(0x200000, 0x201fff, s_vram, sizeof(s_vram), 1, 1);
	m68k_bus_map(0x202000, 0x202fff, s_vram_ext, sizeof(s_vram_ext), 1, 1);
	m68k_bus_map(0x300000, 0x37ffff, s_pixel0, 0x80000, 1, 1);   // (writes: the board's handler)
	m68k_bus_map(0x380000, 0x39ffff, s_pixel1, 0x20000, 1, 0);   // (writes: pixel_1_w)
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	lm_reset();
	return 0;
}

static void lm_exit(void)
{
	rbx_ym_exit(0);
	free(s_mix_l);
	free(s_mix_r);
	s_mix_l = s_mix_r = NULL;
	rbx_gfx_free(&s_gfxspr[0]);
	rbx_gfx_free(&s_gfxspr[1]);
	free(s_snap);
	free(s_pixel0);
	free(s_pixel1);
	free(s_vram_buf);
	s_snap = NULL;
	s_pixel0 = s_pixel1 = NULL;
	s_vram_buf = NULL;
}

static void lm_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int p = 0; p < 2; p++) {   // trackballs: x reversed (MAME key delta 10 x sensitivity 70%)
		s_track[p][0] = (uint8_t)(s_track[p][0] - rbx_analog_dpad(p + 1, RBX_IPT_TRACKBALL_X, 7));
		s_track[p][1] = (uint8_t)(s_track[p][1] + rbx_analog_dpad(p + 1, RBX_IPT_TRACKBALL_Y, 7));
	}
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // irq6_line_hold; screen_vblank: the buffered lists taken again
			memcpy(s_sprtri, s_sprbuf, sizeof(s_sprtri));
			vblank_capture();
			m68k_bus_hold_irq(6);
		}
		m68k_execute(rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		sound_line();
	}
	sound_frame_end();
	s_frame++;
}

static int lm_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[3] = { s_cpu_frac, s_snd_frac, s_ym_frac };
	int32_t misc[4] = { (int32_t)s_frame, (int32_t)s_oki_acc, s_oki_hold, 0 };
	uint8_t snd[M6809_STATE_SIZE];
	if (!load)
		memcpy(snd, &s_snd, M6809_STATE_SIZE);
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "sprb", s_sprbuf, sizeof(s_sprbuf));
	rc |= rbx_state_io(ctx, "sprt", s_sprtri, sizeof(s_sprtri));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "ctrl", s_control, sizeof(s_control));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "vext", s_vram_ext, sizeof(s_vram_ext));
	rc |= rbx_state_io(ctx, "pix0", s_pixel0, 0x40000 * 2);
	rc |= rbx_state_io(ctx, "pix1", s_pixel1, 0x10000 * 2);
	rc |= rbx_state_io(ctx, "track", s_track, sizeof(s_track));
	rc |= rbx_state_io(ctx, "prot", &s_prot, DECO146_STATE_SIZE);
	rc |= rbx_state_io(ctx, "m6809", snd, M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_state_io(ctx, "oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		memcpy(&s_snd, snd, M6809_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_ym_frac = fr[2];
		s_frame = (uint32_t)misc[0];
		s_oki_acc = (uint32_t)misc[1];
		s_oki_hold = misc[2];
		for (uint32_t o = 0; o < 0x10000; o++)   // (the VRAM characters from the pixels written)
			pixel1_update(o);
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int lm_save(void *ctx) { return lm_state(ctx, 0); }
static int lm_load(void *ctx) { return lm_state(ctx, 1); }
static int lm_supported(const char *machine) { return !strcmp(machine, "lemmings"); }

const rbx_driver rbx_drv_lemmings = { "DECO 16 Lemmings", lm_init, lm_exit, lm_reset, lm_frame, lm_save, lm_load,
                                      lm_supported, lm_render, lm_help };
