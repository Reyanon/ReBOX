/*  Capcom Black Tiger / Black Dragon. Ported from MAME src/mame/capcom/blktiger.cpp (BSD-3-Clause, copyright-holders:
    Paul Leaman).
    Z80 at 6 MHz (IRQ held at vblank) with an i8751 MCU at 8 MHz on a latch pair (not on the bootlegs), Z80 sound CPU
    at 3.58 MHz with two YM2203 (the first one's IRQ). Video: a background of 16x16 tiles in four banks of RAM laid
    out 8x4 or 4x8 screens, split in two layers around the sprites; a 256x256 text layer, 16x16 sprites copied at
    vblank; xBRG 444 palette RAM.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/mcs51/mcs51.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../../video/decoband.h"
#include "cap8.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 59637          // set_raw(24 MHz / 4, 384, 0, 256, 262, 16, 240)
#define LINES 262
#define VIS_Y0 16
#define VIS_H 224
#define VIS_W 256
#define MAIN_CLOCK 6000000
#define SND_CLOCK 3579545
#define MCU_CYCLES 666667        // (8 MHz / 12)
#define YM_CLOCK 3579545

static int s_has_mcu;
static uint8_t *s_rom, *s_zrom, *s_mrom;
static uint32_t s_rom_size, s_zrom_size, s_mrom_size;
static rbx_gfx s_gchar, s_gtile, s_gspr;
static int s_port[6];
static z80 s_main, s_snd;
static mcs51_t s_mcu;
static const uint8_t *s_mmap[256], *s_zmap[256];
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;

static struct {
	uint8_t ram[0x2000], scroll[0x4000], tx[0x800], pal[0x400], palext[0x400], sprbuf[0x200];
	uint8_t zram[0x800];
	uint8_t bank, scroll_bank, layout, chon, bgon, objon, flip, snd_reset, latch, z80_latch, i8751_latch;
	uint8_t scrollx[2], scrolly[2];
	int main_irq, ym_irq;
	uint64_t main_frac, snd_frac, mcu_frac, ym_frac;
	uint8_t state_end;
} *t;   // (calloc in init: ADR-040)

static struct {
	uint8_t scroll[0x4000], tx[0x800], pal[0x400], palext[0x400], spr[0x200];
	uint16_t sx, sy;
	uint8_t layout, chon, bgon, objon, flip;
} *s_snap;

static uint8_t z_in(z80 *z, uint8_t port);
static void z_out(z80 *z, uint8_t port, uint8_t v);
static uint8_t snd_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void snd_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

// ---- sound ----
static void ym_irq(int state) { t->ym_irq = state; }

static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_zrom[a];
	if (a >= 0xc000 && a < 0xc800) return t->zram[a - 0xc000];
	if (a == 0xc800) return t->latch;
	if (a == 0xe000 || a == 0xe001) return rbx_ym_read(0, a & 1);
	if (a == 0xe002 || a == 0xe003) return rbx_ym_read(1, a & 1);
	return 0;
}
static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xc800) t->zram[a - 0xc000] = v;
	else if (a == 0xe000 || a == 0xe001) rbx_ym_write(0, a & 1, v);
	else if (a == 0xe002 || a == 0xe003) rbx_ym_write(1, a & 1, v);
}

static void snd_start(void)
{
	z80_init(&s_snd);
	s_snd.read_byte = snd_read;
	s_snd.write_byte = snd_write;
	s_snd.port_in = snd_in;
	s_snd.port_out = snd_out;
	s_snd.read_map = s_zmap;
}

// ---- MCU ----
static uint8_t mcu_in(void *u, int port)
{
	(void)u;
	if (port != 0)
		return 0xff;
	mcs51_set_input(&s_mcu, MCS51_INT1_LINE, 0);   // (from_main_r)
	return t->z80_latch;
}
static void mcu_out(void *u, int port, uint8_t v)
{
	(void)u;
	if (port == 0)
		t->i8751_latch = v;
}

// ---- main CPU ----
static void map_bank(void)
{
	for (int p = 0; p < 0x40; p++)
		s_mmap[0x80 + p] = s_rom + 0x10000 + ((uint32_t)t->bank * 0x4000 + p * 256) % (s_rom_size - 0x10000);
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_rom[a];
	if (a < 0xc000) return s_rom[0x10000 + ((uint32_t)t->bank * 0x4000 + (a - 0x8000)) % (s_rom_size - 0x10000)];
	if (a < 0xd000) return t->scroll[t->scroll_bank * 0x1000u + (a - 0xc000)];
	if (a < 0xd800) return t->tx[a - 0xd000];
	if (a < 0xdc00) return t->pal[a - 0xd800];
	if (a < 0xe000) return t->palext[a - 0xdc00];
	return t->ram[a - 0xe000];
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xc000) return;
	if (a < 0xd000) t->scroll[t->scroll_bank * 0x1000u + (a - 0xc000)] = v;
	else if (a < 0xd800) t->tx[a - 0xd000] = v;
	else if (a < 0xdc00) t->pal[a - 0xd800] = v;
	else if (a < 0xe000) t->palext[a - 0xdc00] = v;
	else t->ram[a - 0xe000] = v;
}

static uint8_t z_in(z80 *z, uint8_t port)
{
	(void)z;
	if (port <= 5) return (uint8_t)rbx_port_read(s_port[port]);
	if (port == 7) return s_has_mcu ? t->i8751_latch : 0;
	return 0;
}
static void z_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	switch (port) {
	case 0x00: t->latch = v; break;
	case 0x01: t->bank = v & 0x0f; map_bank(); break;
	case 0x04:   // video_control_w
		if ((v & 0x20) && !t->snd_reset) {
			snd_start();
			t->ym_irq = 0;
		}
		t->snd_reset = (v & 0x20) != 0;
		t->flip = (v & 0x40) != 0;
		t->chon = !(v & 0x80);
		break;
	case 0x07:   // to_mcu_w
		if (s_has_mcu) {
			mcs51_set_input(&s_mcu, MCS51_INT1_LINE, 1);
			t->z80_latch = v;
		}
		break;
	case 0x08: case 0x09: t->scrollx[port & 1] = v; break;
	case 0x0a: case 0x0b: t->scrolly[port & 1] = v; break;
	case 0x0c: t->bgon = !(v & 2); t->objon = !(v & 4); break;
	case 0x0d: t->scroll_bank = v % 4; break;
	case 0x0e: t->layout = v; break;
	}
}

// ---- video ----
// (the tile's lookups once per tile: the pixels are the same as per-pixel lookups)
static void draw_bg(int front, int y0, int y1)
{
	// (split_table: the colour's group; set_transmask(group, front, back))
	static const uint8_t split[16] = { 3, 3, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	static const uint16_t fmask[4] = { 0xffff, 0xfff0, 0xff00, 0xf000 }, bmask[4] = { 0x8000, 0x800f, 0x80ff, 0x8fff };
	const int flip = s_snap->flip, l8x4 = s_snap->layout != 0;
	const uint32_t wmask = l8x4 ? 2047 : 1023, hmask = l8x4 ? 1023 : 2047;
	for (int y = y0; y < y1; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y + s_snap->sy : y + s_snap->sy) & hmask;
		const uint32_t row = ty >> 4;
		uint32_t last = ~0u, fx = 0;
		const uint8_t *src = NULL;
		uint16_t m = 0, base = 0;
		for (int x = 0; x < VIS_W; x++) {
			const uint32_t tx = (uint32_t)(flip ? 255 - x + s_snap->sx : x + s_snap->sx) & wmask;
			const uint32_t col = tx >> 4;
			if (col != last) {
				last = col;
				const uint32_t ti = l8x4 ? (col & 0x0f) + ((row & 0x0f) << 4) + ((col & 0x70) << 4) + ((row & 0x30) << 7)
				                         : (col & 0x0f) + ((row & 0x0f) << 4) + ((col & 0x30) << 4) + ((row & 0x70) << 6);
				const uint8_t attr = s_snap->scroll[(2 * ti + 1) & 0x3fff];
				const uint32_t code = (uint32_t)(s_snap->scroll[(2 * ti) & 0x3fff] + ((attr & 7) << 8)) % s_gtile.count;
				const int color = (attr & 0x78) >> 3;
				src = s_gtile.pix + code * 256 + (ty & 15) * 16;
				fx = (attr & 0x80) ? 15 : 0;
				m = front ? fmask[split[color]] : bmask[split[color]];
				base = (uint16_t)(color * 16);
			}
			const uint8_t p = src[(tx & 15) ^ fx];
			if (!((m >> p) & 1))
				d[x] = (uint16_t)(base + p);
		}
	}
}

static void draw_tx(int y0, int y1)
{
	const int flip = s_snap->flip;
	for (int y = y0; y < y1; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y : y) & 255;
		uint32_t last = ~0u;
		const uint8_t *src = NULL;
		uint16_t base = 0;
		for (int x = 0; x < VIS_W; x++) {
			const uint32_t tx = (uint32_t)(flip ? 255 - x : x) & 255;
			if ((tx >> 3) != last) {
				last = tx >> 3;
				const uint32_t idx = (ty >> 3) * 32 + (tx >> 3);
				const uint8_t attr = s_snap->tx[idx + 0x400];
				const uint32_t code = (uint32_t)(s_snap->tx[idx] + ((attr & 0xe0) << 3)) % s_gchar.count;
				src = s_gchar.pix + code * 64 + (ty & 7) * 8;
				base = (uint16_t)(0x300 + (attr & 0x1f) * 4);
			}
			const uint8_t p = src[tx & 7];
			if (p != 3)
				d[x] = (uint16_t)(base + p);
		}
	}
}

// the frame in bands of lines drawn in parallel (decoband.h): the back layer, the sprites once, the front layers
#define BANDS 8
static int s_phase;
static void bt_band(int band)
{
	const int y0 = VIS_Y0 + VIS_H * band / BANDS, y1 = VIS_Y0 + VIS_H * (band + 1) / BANDS;
	if (s_phase == 0) {
		for (int y = y0; y < y1; y++) {
			uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
			for (int x = 0; x < VIS_W; x++)
				d[x] = 1023;
		}
		if (s_snap->bgon)
			draw_bg(0, y0, y1);
	} else {
		if (s_snap->bgon)
			draw_bg(1, y0, y1);
		if (s_snap->chon)
			draw_tx(y0, y1);
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, bt_band };
static int bt_help(int helper) { return decoband_help(&s_bands, helper); }

static void bt_render(void)
{
	for (int i = 0; i < 1024; i++) {   // xBRG_444 (the ext byte high)
		const uint32_t w = (uint32_t)(s_snap->palext[i] << 8 | s_snap->pal[i]);
		rbx_video.palette[i] = ((w >> 4) & 15) * 17 << 16 | (w & 15) * 17 << 8 | ((w >> 8) & 15) * 17;
	}
	const int flip = s_snap->flip;
	s_phase = 0;
	decoband_run(&s_bands);
	if (s_snap->objon)
		for (int o = 0x200 - 4; o >= 0; o -= 4) {
			const uint8_t *s = &s_snap->spr[o];
			const int attr = s[1];
			int sx = s[3] - ((attr & 0x10) << 4), sy = s[2], fx = (attr & 8) != 0;
			if (flip) {
				sx = 240 - sx;
				sy = 240 - sy;
				fx = !fx;
			}
			cap8_transpen(&s_gspr, (uint32_t)(s[0] | ((attr & 0xe0) << 3)), (uint16_t)(0x200 + (attr & 7) * 16), fx, flip, sx, sy, 15,
			              VIS_Y0, VIS_Y0 + VIS_H, VIS_Y0, 0, VIS_W);
		}
	s_phase = 1;
	decoband_run(&s_bands);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->scroll, t->scroll, sizeof(t->scroll));
	memcpy(s_snap->tx, t->tx, sizeof(t->tx));
	memcpy(s_snap->pal, t->pal, sizeof(t->pal));
	memcpy(s_snap->palext, t->palext, sizeof(t->palext));
	memcpy(s_snap->spr, t->sprbuf, sizeof(t->sprbuf));
	memcpy(t->sprbuf, t->ram + 0x1e00, sizeof(t->sprbuf));   // (vblank_copy_rising, after the screen's update)
	s_snap->sx = (uint16_t)(t->scrollx[0] | t->scrollx[1] << 8);
	s_snap->sy = (uint16_t)(t->scrolly[0] | t->scrolly[1] << 8);
	s_snap->layout = t->layout;
	s_snap->chon = t->chon;
	s_snap->bgon = t->bgon;
	s_snap->objon = t->objon;
	s_snap->flip = t->flip;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void bt_reset(void)
{
	memset(t, 0, offsetof(__typeof__(*t), state_end));
	t->chon = t->bgon = t->objon = 1;   // (video_start)
	map_bank();
	z80_init(&s_main);
	s_main.read_byte = main_read;
	s_main.write_byte = main_write;
	s_main.port_in = z_in;
	s_main.port_out = z_out;
	s_main.read_map = s_mmap;
	snd_start();
	if (s_has_mcu) {
		mcs51_init(&s_mcu, s_mrom, s_mrom_size);
		s_mcu.port_in = mcu_in;
		s_mcu.port_out = mcu_out;
		mcs51_reset(&s_mcu);
	}
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	s_mix_n = 0;
}

static void bt_exit(void);

static int bt_init(const rbx_game *g)
{
	free(t);
	t = calloc(1, sizeof(*t));
	if (!t) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	const char *m = g->machine, *init = strchr(m, ':');
	init = init ? init + 1 : "";
	s_has_mcu = !strncmp(m, "mcu", 3);
	uint32_t s1, s2, s3;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_mrom = s_has_mcu ? rbx_region("mcu", &s_mrom_size) : NULL;
	uint8_t *chars = rbx_region("chars", &s1), *tiles = rbx_region("tiles", &s2), *spr = rbx_region("sprites", &s3);
	if (!s_rom || s_rom_size < 0x50000 || !s_zrom || s_zrom_size < 0x8000 || (s_has_mcu && !s_mrom) || !chars || !tiles || !spr) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (!strcmp(init, "blktigerb3")) {   // init_blktigerb3: the sound ROM's address lines 3-7 reversed
		uint8_t *buf = (uint8_t *)malloc(0x8000);
		if (!buf) {
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
		for (int i = 0; i < 0x8000; i++) {
			int a = i & 0xff07;
			for (int b = 3; b <= 7; b++)
				if (i & (1 << b))
					a |= 1 << (10 - b);
			buf[i] = s_zrom[a];
		}
		memcpy(s_zrom, buf, 0x8000);
		free(buf);
	}
	{	// charlayout, spritelayout (RGN_FRAC(1,2)) for the tiles and the sprites
		static const uint32_t pc[2] = { 4, 0 }, xc[8] = { 0, 1, 2, 3, 8, 9, 10, 11 };
		static const uint32_t yc[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
		static const uint32_t xs[16] = { 0, 1, 2, 3, 8, 9, 10, 11, 256, 257, 258, 259, 264, 265, 266, 267 };
		static uint32_t ys[16];
		for (int i = 0; i < 16; i++)
			ys[i] = (uint32_t)i * 16;
		const uint32_t h2 = s2 / 2 * 8, h3 = s3 / 2 * 8;
		const uint32_t p2[4] = { h2 + 4, h2, 4, 0 }, p3[4] = { h3 + 4, h3, 4, 0 };
		const rbx_gfx_layout lc = { 8, 8, 2, pc, xc, yc, 128 };
		const rbx_gfx_layout l2 = { 16, 16, 4, p2, xs, ys, 512 }, l3 = { 16, 16, 4, p3, xs, ys, 512 };
		if (rbx_gfx_decode(&s_gchar, &lc, chars, s1) || rbx_gfx_decode(&s_gtile, &l2, tiles, s2 / 2) ||
		    rbx_gfx_decode(&s_gspr, &l3, spr, s3 / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			bt_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 38;   // (0.15)
	yc.irq = ym_irq;
	int rc = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	yc.irq = NULL;
	rc = rc || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		bt_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 1024)) {
		rbx_fail("%s: out of memory", g->set);
		bt_exit();
		return -1;
	}
	memset(s_mmap, 0, sizeof(s_mmap));
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x80; p++) {
		s_mmap[p] = s_rom + p * 256;
		s_zmap[p] = s_zrom + p * 256;
	}
	for (int p = 0xe0; p < 0x100; p++)
		s_mmap[p] = t->ram + (p - 0xe0) * 256;
	static const char *const names[6] = { "IN0", "IN1", "IN2", "DSW0", "DSW1", "FREEZE" };
	for (int i = 0; i < 6; i++)
		s_port[i] = rbx_port_find(names[i]);
	bt_reset();
	return 0;
}

static void bt_exit(void)
{
	rbx_video_wait_idle();
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gchar);
	rbx_gfx_free(&s_gtile);
	rbx_gfx_free(&s_gspr);
	free(s_snap);
	s_snap = NULL;
	free(t);
	t = NULL;
}

static void bt_frame(void)
{
	static int32_t tmp[1024];
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0 + VIS_H) {
			vblank_capture();
			s_main.int_data = 0xff;
			t->main_irq = 1;
		}
		z80_run(&s_main, s_main.cyc + (unsigned long)rbx_line_cycles(&t->main_frac, MAIN_CLOCK, FRAME_MHZ, LINES), &t->main_irq, 1);
		if (s_has_mcu)
			mcs51_execute(&s_mcu, rbx_line_cycles(&t->mcu_frac, MCU_CYCLES, FRAME_MHZ, LINES));
		const int cs = rbx_line_cycles(&t->snd_frac, SND_CLOCK, FRAME_MHZ, LINES);
		if (t->snd_reset)
			s_snd.cyc += (unsigned long)cs;
		else
			z80_run(&s_snd, s_snd.cyc + (unsigned long)cs, &t->ym_irq, 0);
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&t->ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
		s_mix_n += rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
		rbx_ym_advance(1, clocks, tmp, 1024);
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	rbx_ym_async_take(1, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int bt_state(void *ctx, int load)
{
	int rc = 0;
	z80 c = s_main, zs = s_snd;
	rc |= rbx_state_io(ctx, "bt", t, (uint32_t)offsetof(__typeof__(*t), state_end));
	rc |= rbx_state_io(ctx, "bt_main", &c, sizeof(c));
	rc |= rbx_state_io(ctx, "bt_snd", &zs, sizeof(zs));
	if (s_has_mcu)
		rc |= rbx_state_io(ctx, "bt_mcu", &s_mcu.st, sizeof(s_mcu.st));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	if (!rc && load) {
		c.read_byte = main_read;
		c.write_byte = main_write;
		c.port_in = z_in;
		c.port_out = z_out;
		c.userdata = NULL;
		c.read_map = s_mmap;
		s_main = c;
		zs.read_byte = snd_read;
		zs.write_byte = snd_write;
		zs.port_in = snd_in;
		zs.port_out = snd_out;
		zs.userdata = NULL;
		zs.read_map = s_zmap;
		s_snd = zs;
		map_bank();
	}
	return rc ? -1 : 0;
}

static int bt_save(void *ctx) { return bt_state(ctx, 0); }
static int bt_load(void *ctx) { return bt_state(ctx, 1); }
static int bt_supported(const char *machine) { return !strncmp(machine, "mcu", 3) || !strncmp(machine, "nomcu", 5); }

const rbx_driver rbx_drv_blktiger = { "Capcom Black Tiger", bt_init, bt_exit, bt_reset, bt_frame, bt_save, bt_load,
                                      bt_supported, bt_render, bt_help };
