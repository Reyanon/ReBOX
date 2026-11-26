/*  Capcom Baseball (Mitchell hardware). Ported from MAME src/mame/capcom/cbasebal.cpp (BSD-3-Clause,
    copyright-holders: Nicola Salmoria).
    Z80 at 6 MHz with Kabuki-encrypted code (the Pang keys; 32 banks), IRQ held at vblank; a YM2413 at 3.58 MHz, an
    OKI6295, a 93C46 EEPROM. Video: a 1024x512 tilemap of 16x16 (two tile banks), a 512x256 text layer (pen 3 clear),
    16x16 sprites; xBRG 444 palette RAM, all three in a banked window. The screen is MAME's (60 Hz, 2.5 ms of vblank).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../machine/eepromser.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../../video/decoband.h"
#include "../machine/kabuki.h"
#include "cap8.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 60000
#define LINES 256
#define VIS_X0 64
#define VIS_W 384
#define VIS_Y0 16
#define VIS_H 224
#define CPU_CLOCK 6000000
#define YM_CLOCK 3579545
#define VBL_LINES 22             // (2.5 ms past line 240: the frame's first lines still in vblank)

static uint8_t *s_rom, *s_dec;
static uint32_t s_rom_size;
static uint8_t *s_opmem;   // 64 KB (calloc in init: ADR-040)
static rbx_gfx s_gtext, s_gtile, s_gspr;
static int s_port[3];
static z80 s_main;
static okim6295_t s_oki;
static eepser_t s_eeprom;
static const uint8_t *s_mmap[256];
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static int s_line;

static struct {
	uint8_t ram[0x2000], scroll[0x1000], text[0x1000], pal[0x800];
	uint8_t bank, view, flip, tilebank, spritebank, text_on, bg_on, obj_on, scrollx[2], scrolly[2];
	int irq;
	uint64_t frac, ym_frac;
	uint32_t oki_acc;
	int32_t oki_hold;
	uint8_t state_end;
} *t;   // (calloc in init: ADR-040)

static struct {
	uint8_t spr[0x200], scroll[0x1000], text[0x1000], pal[0x800];
	uint16_t sx, sy;
	uint8_t flip, tilebank, spritebank, text_on, bg_on, obj_on;
} *s_snap;

static uint8_t z_in(z80 *z, uint8_t port);
static void z_out(z80 *z, uint8_t port, uint8_t v);

static void map_bank(void)
{
	for (int p = 0; p < 0x40; p++)
		s_mmap[0x80 + p] = s_rom + 0x10000 + ((uint32_t)t->bank * 0x4000 + p * 256) % (s_rom_size - 0x10000);
	memcpy(s_opmem + 0x8000, s_dec + 0x10000 + ((uint32_t)t->bank * 0x4000) % (s_rom_size - 0x10000), 0x4000);
}

static uint8_t *view_mem(uint16_t a)   // (the banked RAM view at c000-cfff)
{
	switch (t->view) {
	case 1: return a < 0xc800 ? &t->pal[a - 0xc000] : NULL;
	case 2: return &t->text[a - 0xc000];
	default: return &t->scroll[a - 0xc000];
	}
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_rom[a];
	if (a < 0xc000) return s_rom[0x10000 + ((uint32_t)t->bank * 0x4000 + (a - 0x8000)) % (s_rom_size - 0x10000)];
	if (a < 0xd000) {
		const uint8_t *p = view_mem(a);
		return p ? *p : 0;
	}
	if (a >= 0xe000) return t->ram[a - 0xe000];
	return 0;
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xc000) return;
	if (a < 0xd000) {
		uint8_t *p = view_mem(a);
		if (p)
			*p = v;
	} else if (a >= 0xe000)
		t->ram[a - 0xe000] = v;
}

static int64_t now(void) { return (int64_t)s_main.cyc; }

static uint8_t z_in(z80 *z, uint8_t port)
{
	(void)z;
	if (port == 0x10) return (uint8_t)rbx_port_read(s_port[0]);
	if (port == 0x11) return (uint8_t)rbx_port_read(s_port[1]);
	if (port == 0x12) {   // SYSTEM: bit 6 vblank (active low), bit 7 the EEPROM's DO
		const int vbl = s_line >= VIS_Y0 + VIS_H || s_line < VBL_LINES;
		rbx_port_custom(s_port[2], 0xc0, (vbl ? 0x40u : 0) | (eepser_do_read_93cxx(&s_eeprom, now()) ? 0x80u : 0));
		return (uint8_t)rbx_port_read(s_port[2]);
	}
	return 0;
}
static void z_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	switch (port) {
	case 0x00:   // bankswitch_w
		t->bank = v & 0x1f;
		map_bank();
		t->view = (v & 0xc0) >> 6;
		break;
	case 0x01: eepser_cs_write(&s_eeprom, (v >> 4) & 1, now()); break;        // (IO_01 bit 4, active high)
	case 0x02: eepser_clk_write(&s_eeprom, !(v & 0x20), now()); break;         // (IO_02 bit 5, active low)
	case 0x03: eepser_di_write(&s_eeprom, (v >> 6) & 1); break;                // (IO_03 bit 6)
	case 0x05: oki_write(&s_oki, v); break;
	case 0x06: case 0x07: rbx_ym_write(0, port & 1, v); break;
	case 0x08: case 0x09: t->scrollx[port & 1] = v; break;
	case 0x0a: case 0x0b: t->scrolly[port & 1] = v; break;
	case 0x13:   // gfxctrl_w
		t->flip = (v & 2) != 0;
		t->tilebank = (v & 8) >> 3;
		t->spritebank = (v & 0x10) >> 4;
		t->text_on = !(v & 0x20);
		t->bg_on = !(v & 0x40);
		t->obj_on = !(v & 0x80);
		break;
	}
}

// ---- video ----
// (the tile's lookups once per tile: the pixels are the same as per-pixel lookups)
static void draw_bg(int y0, int y1)
{
	const int flip = s_snap->flip;
	for (int y = y0; y < y1; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		if (!s_snap->bg_on) {
			for (int x = 0; x < VIS_W; x++)
				d[x] = 768;
			continue;
		}
		const uint32_t ty = (uint32_t)(flip ? 255 - y + s_snap->sy : y + s_snap->sy) & 511;
		uint32_t last = ~0u, fx = 0;
		const uint8_t *src = NULL;
		uint16_t base = 0;
		for (int x = VIS_X0; x < VIS_X0 + VIS_W; x++) {   // 64 x 32 of 16x16
			const uint32_t tx = (uint32_t)(flip ? 511 - x + s_snap->sx : x + s_snap->sx) & 1023;
			if ((tx >> 4) != last) {
				last = tx >> 4;
				const uint32_t idx = (ty >> 4) * 64 + (tx >> 4);
				const uint8_t attr = s_snap->scroll[2 * idx + 1];
				const uint32_t code = rbx_wrap((uint32_t)(s_snap->scroll[2 * idx] + ((attr & 7) << 8) + 0x800 * s_snap->tilebank), s_gtile.count);
				src = s_gtile.pix + code * 256 + (ty & 15) * 16;
				fx = (attr & 8) ? 15 : 0;
				base = (uint16_t)(768 + ((attr & 0xf0) >> 4) * 16);
			}
			d[x - VIS_X0] = (uint16_t)(base + src[(tx & 15) ^ fx]);
		}
	}
}

static void draw_text(int y0, int y1)
{
	const int flip = s_snap->flip;
	for (int y = y0; y < y1; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y : y) & 255;
		uint32_t last = ~0u, fx = 0;
		const uint8_t *src = NULL;
		uint16_t base = 0;
		for (int x = VIS_X0; x < VIS_X0 + VIS_W; x++) {   // 64 x 32 of 8x8
			const uint32_t tx = (uint32_t)(flip ? 511 - x : x) & 511;
			if ((tx >> 3) != last) {
				last = tx >> 3;
				const uint32_t idx = (ty >> 3) * 64 + (tx >> 3);
				const uint8_t attr = s_snap->text[idx + 0x800];
				const uint32_t code = rbx_wrap((uint32_t)(s_snap->text[idx] + ((attr & 0xf0) << 4)), s_gtext.count);
				src = s_gtext.pix + code * 64 + (ty & 7) * 8;
				fx = (attr & 8) ? 7 : 0;
				base = (uint16_t)(256 + (attr & 7) * 4);
			}
			const uint8_t p = src[(tx & 7) ^ fx];
			if (p != 3)
				d[x - VIS_X0] = (uint16_t)(base + p);
		}
	}
}

// the frame in bands of lines drawn in parallel (decoband.h): the background, the sprites once, the text
#define BANDS 8
static int s_phase;
static void cb_band(int band)
{
	const int y0 = VIS_Y0 + VIS_H * band / BANDS, y1 = VIS_Y0 + VIS_H * (band + 1) / BANDS;
	if (s_phase == 0)
		draw_bg(y0, y1);
	else if (s_snap->text_on)
		draw_text(y0, y1);
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, cb_band };
static int cb_help(int helper) { return decoband_help(&s_bands, helper); }

static void cb_render(void)
{
	for (int i = 0; i < 1024; i++) {   // xBRG_444, little-endian words
		const uint32_t w = (uint32_t)(s_snap->pal[2 * i] | s_snap->pal[2 * i + 1] << 8);
		rbx_video.palette[i] = ((w >> 4) & 15) * 17 << 16 | (w & 15) * 17 << 8 | ((w >> 8) & 15) * 17;
	}
	const int flip = s_snap->flip;
	s_phase = 0;
	decoband_run(&s_bands);
	if (s_snap->obj_on)
		for (int o = 0x200 - 8; o >= 0; o -= 4) {
			const uint8_t *s = &s_snap->spr[o];
			const int attr = s[1];
			uint32_t code = s[0] + (uint32_t)((attr & 0xe0) << 3) + s_snap->spritebank * 0x800u;
			int fx = (attr & 8) != 0;
			int sx = s[3] + ((attr & 0x10) << 4), sy = ((s[2] + 8) & 0xff) - 8;
			if (flip) {
				sx = 496 - sx;
				sy = 240 - sy;
				fx = !fx;
			}
			cap8_transpen(&s_gspr, code, (uint16_t)(512 + (attr & 7) * 16), fx, flip, sx, sy, 15, VIS_Y0, VIS_Y0 + VIS_H, VIS_Y0,
			              VIS_X0, VIS_W);
		}
	s_phase = 1;
	decoband_run(&s_bands);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spr, t->ram + 0x1e00, 0x200);
	memcpy(s_snap->scroll, t->scroll, sizeof(t->scroll));
	memcpy(s_snap->text, t->text, sizeof(t->text));
	memcpy(s_snap->pal, t->pal, sizeof(t->pal));
	s_snap->sx = (uint16_t)(t->scrollx[0] + 256 * t->scrollx[1]);
	s_snap->sy = (uint16_t)(t->scrolly[0] + 256 * t->scrolly[1]);
	s_snap->flip = t->flip;
	s_snap->tilebank = t->tilebank;
	s_snap->spritebank = t->spritebank;
	s_snap->text_on = t->text_on;
	s_snap->bg_on = t->bg_on;
	s_snap->obj_on = t->obj_on;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void cb_reset(void)
{
	memset(t, 0, offsetof(__typeof__(*t), state_end));
	memcpy(s_opmem, s_dec, 0x8000);
	map_bank();
	z80_init(&s_main);
	s_main.read_byte = main_read;
	s_main.write_byte = main_write;
	s_main.port_in = z_in;
	s_main.port_out = z_out;
	s_main.read_map = s_mmap;
	s_main.op_rom = s_opmem;
	rbx_ym_reset(0);
	oki_reset(&s_oki);
	s_mix_n = 0;
	s_line = 0;
}

static void cb_exit(void);

static int cb_init(const rbx_game *g)
{
	free(s_opmem);
	s_opmem = (uint8_t *)calloc(1, 0x10000);
	if (!s_opmem) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	free(t);
	t = calloc(1, sizeof(*t));
	if (!t) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t s1, s2, s3, os;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *text = rbx_region("text", &s1), *tiles = rbx_region("tiles", &s2), *spr = rbx_region("sprites", &s3);
	uint8_t *oki = rbx_region("oki", &os);
	if (!s_rom || s_rom_size < 0x90000 || !text || s1 < 0x10000 || !tiles || s2 < 0x80000 || !spr || s3 < 0x80000 || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	free(s_dec);
	s_dec = (uint8_t *)calloc(1, s_rom_size);
	if (!s_dec) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	kabuki_mitchell_decode(s_rom, s_dec, s_rom_size, 0x01234567, 0x76543210, 0x6548, 0x24);   // (pang_decode)
	memset(s_opmem, 0, 0x10000);
	{	// cbasebal_textlayout, _tilelayout, _spritelayout (4096 each)
		static const uint32_t pc[2] = { 0, 4 }, xc[8] = { 11, 10, 9, 8, 3, 2, 1, 0 };
		static const uint32_t yc[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
		static const uint32_t xt[16] = { 0, 1, 2, 3, 8, 9, 10, 11, 256, 257, 258, 259, 264, 265, 266, 267 };
		static uint32_t ys[16];
		for (int i = 0; i < 16; i++)
			ys[i] = (uint32_t)i * 16;
		static const uint32_t pt[4] = { 4096 * 64 * 8 + 4, 4096 * 64 * 8 + 0, 4, 0 };
		const rbx_gfx_layout lc = { 8, 8, 2, pc, xc, yc, 128 };
		const rbx_gfx_layout lt = { 16, 16, 4, pt, xt, ys, 512 };
		if (rbx_gfx_decode(&s_gtext, &lc, text, 4096 * 16) || rbx_gfx_decode(&s_gtile, &lt, tiles, 4096 * 64) ||
		    rbx_gfx_decode(&s_gspr, &lt, spr, 4096 * 64)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			cb_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2413;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 256;
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		cb_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 1024)) {
		rbx_fail("%s: out of memory", g->set);
		cb_exit();
		return -1;
	}
	oki_init(&s_oki, 1056000, 1, oki, os, 128);   // (0.50)
	eepser_init(&s_eeprom, EEPSER_93CXX, 64, 16, CPU_CLOCK);
	eepser_nv_load(&s_eeprom, g->set);
	memset(s_mmap, 0, sizeof(s_mmap));
	for (int p = 0; p < 0x80; p++)
		s_mmap[p] = s_rom + p * 256;
	for (int p = 0xe0; p < 0x100; p++)
		s_mmap[p] = t->ram + (p - 0xe0) * 256;
	static const char *const names[3] = { "P1", "P2", "SYSTEM" };
	for (int i = 0; i < 3; i++)
		s_port[i] = rbx_port_find(names[i]);
	cb_reset();
	return 0;
}

static void cb_exit(void)
{
	rbx_video_wait_idle();
	eepser_nv_save(&s_eeprom);
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gtext);
	rbx_gfx_free(&s_gtile);
	rbx_gfx_free(&s_gspr);
	free(s_snap);
	s_snap = NULL;
	free(s_dec);
	s_dec = NULL;
	free(s_opmem);
	s_opmem = NULL;
	free(t);
	t = NULL;
}

static void cb_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {
			vblank_capture();
			s_main.int_data = 0xff;
			t->irq = 1;   // (irq0_line_hold at vblank)
		}
		z80_run(&s_main, s_main.cyc + (unsigned long)rbx_line_cycles(&t->frac, CPU_CLOCK, FRAME_MHZ, LINES), &t->irq, 1);
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&t->ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
		const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
		const uint32_t yr = rbx_ym_rate(0);
		for (int i = 0; i < n; i++) {
			t->oki_acc += s_oki.rate;
			while (t->oki_acc >= yr) {
				t->oki_acc -= yr;
				t->oki_hold = oki_sample(&s_oki);
			}
			s_mix[s_mix_n + i] += t->oki_hold;
		}
		s_mix_n += n;
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int cb_state(void *ctx, int load)
{
	int rc = 0;
	z80 c = s_main;
	rc |= rbx_state_io(ctx, "cb", t, (uint32_t)offsetof(__typeof__(*t), state_end));
	rc |= rbx_state_io(ctx, "cb_main", &c, sizeof(c));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "cb_oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	rc |= eepser_state(&s_eeprom, ctx, load, "cb_eeprom");
	if (!rc && load) {
		c.read_byte = main_read;
		c.write_byte = main_write;
		c.port_in = z_in;
		c.port_out = z_out;
		c.userdata = NULL;
		c.read_map = s_mmap;
		c.op_rom = s_opmem;
		s_main = c;
		map_bank();
	}
	return rc ? -1 : 0;
}

static int cb_save(void *ctx) { return cb_state(ctx, 0); }
static int cb_load(void *ctx) { return cb_state(ctx, 1); }
static int cb_supported(const char *machine) { return !strncmp(machine, "cbasebal", 8); }

const rbx_driver rbx_drv_cbasebal = { "Capcom Baseball", cb_init, cb_exit, cb_reset, cb_frame, cb_save, cb_load,
                                      cb_supported, cb_render, cb_help };
