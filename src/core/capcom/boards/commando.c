/*  Capcom Commando / Senjou no Ookami. Ported from MAME src/mame/capcom/commando.cpp (BSD-3-Clause,
    copyright-holders: Nicola Salmoria).
    Z80 at 3 MHz with encrypted opcodes, Z80 sound CPU at 3 MHz with two YM2203 at 1.5 MHz; the interrupts from a PROM
    read by the scanline counter. Video: a 512x512 tilemap of 16x16 (3 bpp), a 256x256 text layer (2 bpp, pen 3
    clear), 16x16 sprites from a buffered RAM; RGB 444 PROMs.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "cap8.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 59637          // set_raw(12 MHz / 2, 384, 0, 256, 262, 16, 240)
#define LINES 262
#define VIS_Y0 16
#define VIS_H 224
#define VIS_W 256
#define CPU_CLOCK 3000000
#define YM_CLOCK 1500000

static uint8_t *s_rom, *s_zrom, s_irqprom[0x100], *s_dec;   // s_dec 48 KB (calloc in init: ADR-040)
static uint32_t s_rom_size, s_zrom_size;
static uint32_t s_pal[256];
static rbx_gfx s_gchar, s_gtile, s_gspr;
static int s_port[5];
static z80 s_main, s_snd;
static const uint8_t *s_mmap[256], *s_zmap[256];
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;

static struct {
	uint8_t ram[0x2000], sprbuf[0x180], fg[0x800], bg[0x800];
	uint8_t zram[0x800];
	uint8_t scrollx[2], scrolly[2], flip, snd_reset, latch;
	int main_irq, snd_irq;
	uint64_t main_frac, snd_frac, ym_frac;
	uint8_t state_end;
} *t;   // (calloc in init: ADR-040)

static struct {
	uint8_t spr[0x180], fg[0x800], bg[0x800];
	uint16_t sx, sy;
	uint8_t flip;
} *s_snap;

static uint8_t z_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

// ---- sound ----
static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x4000) return s_zrom[a];
	if (a >= 0x4000 && a < 0x4800) return t->zram[a - 0x4000];
	if (a == 0x6000) return t->latch;
	return 0;
}
static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000 && a < 0x4800) t->zram[a - 0x4000] = v;
	else if (a == 0x8000 || a == 0x8001) rbx_ym_write(0, a & 1, v);
	else if (a == 0x8002 || a == 0x8003) rbx_ym_write(1, a & 1, v);
}

// ---- main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xc000) return s_rom[a];
	if (a >= 0xc000 && a <= 0xc004) return (uint8_t)rbx_port_read(s_port[a - 0xc000]);
	if (a >= 0xd000 && a < 0xd800) return t->fg[a - 0xd000];
	if (a >= 0xd800 && a < 0xe000) return t->bg[a - 0xd800];
	if (a >= 0xe000) return t->ram[a - 0xe000];
	return 0;
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xe000) { t->ram[a - 0xe000] = v; return; }
	if (a >= 0xd000 && a < 0xd800) { t->fg[a - 0xd000] = v; return; }
	if (a >= 0xd800 && a < 0xe000) { t->bg[a - 0xd800] = v; return; }
	switch (a) {
	case 0xc800: t->latch = v; break;
	case 0xc804:   // control_w
		if ((v & 0x10) && !t->snd_reset) {
			z80_init(&s_snd);
			s_snd.read_byte = snd_read;
			s_snd.write_byte = snd_write;
			s_snd.port_in = z_in;
			s_snd.port_out = z_out;
			s_snd.read_map = s_zmap;
			t->snd_irq = 0;
		}
		t->snd_reset = (v & 0x10) != 0;
		t->flip = (v & 0x80) != 0;
		break;
	case 0xc806: memcpy(t->sprbuf, t->ram + 0x1e00, sizeof(t->sprbuf)); break;   // (buffered_spriteram8 write)
	case 0xc808: case 0xc809: t->scrollx[a & 1] = v; break;
	case 0xc80a: case 0xc80b: t->scrolly[a & 1] = v; break;
	}
}

// ---- video ----
static void cm_render(void)
{
	memcpy(rbx_video.palette, s_pal, sizeof(s_pal));
	const int flip = s_snap->flip;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y + s_snap->sy : y + s_snap->sy) & 511;
		for (int x = 0; x < VIS_W; x++) {   // bg: TILEMAP_SCAN_COLS 32 x 32 of 16x16
			const uint32_t tx = (uint32_t)(flip ? 255 - x + s_snap->sx : x + s_snap->sx) & 511;
			const uint32_t idx = (tx >> 4) * 32 + (ty >> 4);
			const uint8_t attr = s_snap->bg[0x400 + idx];
			const uint32_t code = (uint32_t)(s_snap->bg[idx] + ((attr & 0xc0) << 2)) % s_gtile.count;
			const uint32_t px = (attr & 0x10) ? 15 - (tx & 15) : tx & 15, py = (attr & 0x20) ? 15 - (ty & 15) : ty & 15;
			d[x] = (uint16_t)((attr & 0x0f) * 8 + s_gtile.pix[code * 256 + py * 16 + px]);
		}
	}
	for (int o = 0x180 - 4; o >= 0; o -= 4) {
		const uint8_t *s = &s_snap->spr[o];
		const int attr = s[1], bank = (attr & 0xc0) >> 6;
		int fx = (attr & 4) != 0, fy = (attr & 8) != 0;
		int sx = s[3] - ((attr & 1) << 8), sy = s[2];
		if (flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			fx = !fx;
			fy = !fy;
		}
		if (bank < 3)
			cap8_transpen(&s_gspr, (uint32_t)(s[0] + 256 * bank), (uint16_t)(128 + ((attr & 0x30) >> 4) * 16), fx, fy, sx, sy,
			              15, VIS_Y0, VIS_Y0 + VIS_H, VIS_Y0, 0, VIS_W);
	}
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y : y) & 255;
		for (int x = 0; x < VIS_W; x++) {
			const uint32_t tx = (uint32_t)(flip ? 255 - x : x) & 255;
			const uint32_t idx = (ty >> 3) * 32 + (tx >> 3);
			const uint8_t attr = s_snap->fg[0x400 + idx];
			const uint32_t code = (uint32_t)(s_snap->fg[idx] + ((attr & 0xc0) << 2)) % s_gchar.count;
			const uint32_t px = (attr & 0x10) ? 7 - (tx & 7) : tx & 7, py = (attr & 0x20) ? 7 - (ty & 7) : ty & 7;
			const uint8_t p = s_gchar.pix[code * 64 + py * 8 + px];
			if (p != 3)
				d[x] = (uint16_t)(192 + (attr & 0x0f) * 4 + p);
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spr, t->sprbuf, sizeof(t->sprbuf));
	memcpy(s_snap->fg, t->fg, sizeof(t->fg));
	memcpy(s_snap->bg, t->bg, sizeof(t->bg));
	s_snap->sx = (uint16_t)(t->scrollx[0] | t->scrollx[1] << 8);
	s_snap->sy = (uint16_t)(t->scrolly[0] | t->scrolly[1] << 8);
	s_snap->flip = t->flip;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void z_setup(z80 *z, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t), const uint8_t **map)
{
	z80_init(z);
	z->read_byte = rd;
	z->write_byte = wr;
	z->port_in = z_in;
	z->port_out = z_out;
	z->read_map = map;
}

static void cm_reset(void)
{
	memset(t, 0, offsetof(__typeof__(*t), state_end));
	z_setup(&s_main, main_read, main_write, s_mmap);
	s_main.op_rom = s_dec;
	z_setup(&s_snd, snd_read, snd_write, s_zmap);
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	s_mix_n = 0;
}

static void cm_exit(void);

static int cm_init(const rbx_game *g)
{
	free(s_dec);
	s_dec = (uint8_t *)calloc(1, 0xc000);
	if (!s_dec) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	free(t);
	t = calloc(1, sizeof(*t));
	if (!t) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t s1, s2, s3, sp, si;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *chars = rbx_region("chars", &s1), *tiles = rbx_region("tiles", &s2), *spr = rbx_region("sprites", &s3);
	const uint8_t *prom = rbx_region("proms", &sp), *irq = rbx_region("irqprom", &si);
	if (!s_rom || s_rom_size < 0xc000 || !s_zrom || s_zrom_size < 0x4000 || !chars || !tiles || !spr || !prom || sp < 0x300 ||
	    !irq || si < 0x100) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memcpy(s_irqprom, irq, 0x100);
	{	// init_commando / init_spaceinv: the opcodes' bits swapped (commando: the first one left alone)
		const char *init = strchr(g->machine, ':');
		const int spaceinv = init && !strcmp(init + 1, "spaceinv");
		for (int a = 0; a < 0xc000; a++) {
			const uint8_t src = s_rom[a];
			s_dec[a] = (a == 0 && !spaceinv) ? src : (uint8_t)((src & 0x11) | ((src & 0xe0) >> 4) | ((src & 0x0e) << 4));
		}
	}
	for (int i = 0; i < 256; i++)   // (RGB_444_PROMS)
		s_pal[i] = (uint32_t)((prom[i] & 15) * 17) << 16 | (uint32_t)((prom[256 + i] & 15) * 17) << 8 | (uint32_t)((prom[512 + i] & 15) * 17);
	{	// charlayout, tilelayout (RGN_FRAC(1,3)), spritelayout (RGN_FRAC(1,2))
		static const uint32_t pc[2] = { 4, 0 }, xc[8] = { 0, 1, 2, 3, 8, 9, 10, 11 };
		static const uint32_t yc[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
		static const uint32_t xt[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 128, 129, 130, 131, 132, 133, 134, 135 };
		static const uint32_t xs[16] = { 0, 1, 2, 3, 8, 9, 10, 11, 256, 257, 258, 259, 264, 265, 266, 267 };
		static uint32_t yt[16], ys[16];
		for (int i = 0; i < 16; i++) {
			yt[i] = (uint32_t)i * 8;
			ys[i] = (uint32_t)i * 16;
		}
		const uint32_t f3 = s2 / 3 * 8, f2 = s3 / 2 * 8;
		const uint32_t pt[3] = { 0, f3, 2 * f3 }, ps[4] = { f2 + 4, f2, 4, 0 };
		const rbx_gfx_layout lc = { 8, 8, 2, pc, xc, yc, 128 };
		const rbx_gfx_layout lt = { 16, 16, 3, pt, xt, yt, 256 }, ls = { 16, 16, 4, ps, xs, ys, 512 };
		if (rbx_gfx_decode(&s_gchar, &lc, chars, s1) || rbx_gfx_decode(&s_gtile, &lt, tiles, s2 / 3) ||
		    rbx_gfx_decode(&s_gspr, &ls, spr, s3 / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			cm_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 38;   // (0.15)
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1) || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1)) {
		rbx_fail("%s: out of memory", g->set);
		cm_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 256)) {
		rbx_fail("%s: out of memory", g->set);
		cm_exit();
		return -1;
	}
	memset(s_mmap, 0, sizeof(s_mmap));
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0xc0; p++)
		s_mmap[p] = s_rom + p * 256;
	for (int p = 0; p < 0x40; p++)
		s_zmap[p] = s_zrom + p * 256;
	static const char *const names[5] = { "SYSTEM", "P1", "P2", "DSW1", "DSW2" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	cm_reset();
	return 0;
}

static void cm_exit(void)
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
	free(s_dec);
	s_dec = NULL;
	free(t);
	t = NULL;
}

static void cm_frame(void)
{
	static int32_t tmp[1024];
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		const uint8_t irq = s_irqprom[line & 0xff];
		if (irq & 8) {
			s_main.int_data = (uint8_t)(0xc7 | ((irq << 3) & 0x18));
			t->main_irq = 1;
		}
		if ((irq & 4) && !t->snd_reset) {
			s_snd.int_data = 0xff;
			t->snd_irq = 1;
		}
		z80_run(&s_main, s_main.cyc + (unsigned long)rbx_line_cycles(&t->main_frac, CPU_CLOCK, FRAME_MHZ, LINES), &t->main_irq, 1);
		const int cs = rbx_line_cycles(&t->snd_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		if (t->snd_reset)
			s_snd.cyc += (unsigned long)cs;
		else
			z80_run(&s_snd, s_snd.cyc + (unsigned long)cs, &t->snd_irq, 1);
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

static int z_state(void *ctx, const char *tag, z80 *z, int load, uint8_t (*rd)(void *, uint16_t),
                   void (*wr)(void *, uint16_t, uint8_t), const uint8_t **map, const uint8_t *op)
{
	z80 c = *z;
	const int rc = rbx_state_io(ctx, tag, &c, sizeof(c));
	if (!rc && load) {
		c.read_byte = rd;
		c.write_byte = wr;
		c.port_in = z_in;
		c.port_out = z_out;
		c.userdata = NULL;
		c.read_map = map;
		c.op_rom = op;
		*z = c;
	}
	return rc;
}

static int cm_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "cm", t, (uint32_t)offsetof(__typeof__(*t), state_end));
	rc |= z_state(ctx, "cm_main", &s_main, load, main_read, main_write, s_mmap, s_dec);
	rc |= z_state(ctx, "cm_snd", &s_snd, load, snd_read, snd_write, s_zmap, NULL);
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	return rc ? -1 : 0;
}

static int cm_save(void *ctx) { return cm_state(ctx, 0); }
static int cm_load(void *ctx) { return cm_state(ctx, 1); }
static int cm_supported(const char *machine) { return !strncmp(machine, "commando", 8); }

const rbx_driver rbx_drv_commando = { "Capcom Commando", cm_init, cm_exit, cm_reset, cm_frame, cm_save, cm_load,
                                      cm_supported, cm_render, NULL };
