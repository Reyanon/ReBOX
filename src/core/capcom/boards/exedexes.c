/*  Capcom Exed Exes / Savage Bees. Ported from MAME src/mame/capcom/exedexes.cpp (BSD-3-Clause, copyright-holders:
    Richard Davies).
    Z80 at 3 MHz and a Z80 sound CPU at 3 MHz with an AY-8910 at 1.5 MHz and two SN76489A at 3 MHz; the interrupts from
    a PROM read by the scanline counter. Video: a 2048x2048 background of 32x32 tiles and a 2048x2048 foreground of
    16x16 tiles from ROM, a 256x256 text layer (pens whose colour is 0xcf clear), 16x16 sprites from a buffered RAM
    (some behind the foreground); 256 colours through lookup PROMs.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"
#include "../../sound/sn76496.h"
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
#define AY_CLOCK 1500000
#define SN_CLOCK 3000000
#define NPENS 0x400

static uint8_t *s_rom, *s_zrom, *s_tilerom, s_irqprom[0x100];
static uint32_t s_rom_size, s_zrom_size, s_tilerom_size;
static uint32_t s_pens[NPENS];
static uint8_t s_ctab[NPENS];
static rbx_gfx s_gchar, s_gbg, s_gfg, s_gspr;
static int s_port[5];
static z80 s_main, s_snd;
static ay8910_t s_ay;
static sn76496_t s_sn[2];
static const uint8_t *s_mmap[256], *s_zmap[256];
static int32_t s_mix[2048];
static int s_mix_n;
static uint8_t *s_pri;   // VIS_H * VIS_W (calloc in init: ADR-040)

static struct {
	uint8_t ram[0x1000], spr[0x1000], sprbuf[0x1000], vram[0x400], cram[0x400];
	uint8_t zram[0x800];
	uint8_t latch, nbg_y[2], nbg_x[2], bg_scroll[2], chon, objon, sc1on, sc2on;
	int main_irq, snd_irq;
	uint64_t main_frac, snd_frac, ay_frac, sn_frac;
	uint8_t state_end;
} *t;   // (calloc in init: ADR-040)

static struct {
	uint8_t spr[0x1000], vram[0x400], cram[0x400];
	uint16_t bgx, fgx, fgy;
	uint8_t chon, objon, sc1on, sc2on;
} *s_snap;

static uint8_t z_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

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
	else if (a == 0x8000 || a == 0x8001) ay8910_write(&s_ay, a & 1, v);
	else if (a == 0x8002) sn76496_write(&s_sn[0], v);
	else if (a == 0x8003) sn76496_write(&s_sn[1], v);
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xc000) return a < s_rom_size ? s_rom[a] : 0;
	if (a >= 0xc000 && a <= 0xc004) return (uint8_t)rbx_port_read(s_port[a - 0xc000]);
	if (a >= 0xd000 && a < 0xd400) return t->vram[a - 0xd000];
	if (a >= 0xd400 && a < 0xd800) return t->cram[a - 0xd400];
	if (a >= 0xe000 && a < 0xf000) return t->ram[a - 0xe000];
	if (a >= 0xf000) return t->spr[a - 0xf000];
	return 0;
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xe000 && a < 0xf000) t->ram[a - 0xe000] = v;
	else if (a >= 0xf000) t->spr[a - 0xf000] = v;
	else if (a >= 0xd000 && a < 0xd400) t->vram[a - 0xd000] = v;
	else if (a >= 0xd400 && a < 0xd800) t->cram[a - 0xd400] = v;
	else switch (a) {
	case 0xc800: t->latch = v; break;
	case 0xc804: t->chon = v & 0x80; break;
	case 0xc806: memcpy(t->sprbuf, t->spr, sizeof(t->spr)); break;
	case 0xd800: case 0xd801: t->nbg_y[a & 1] = v; break;
	case 0xd802: case 0xd803: t->nbg_x[a & 1] = v; break;
	case 0xd804: case 0xd805: t->bg_scroll[a & 1] = v; break;
	case 0xd807: t->sc2on = v & 0x10; t->sc1on = v & 0x20; t->objon = v & 0x40; break;
	}
}

// ---- video ----
static void ex_render(void)
{
	memcpy(rbx_video.palette, s_pens, sizeof(s_pens));
	memset(s_pri, 0, VIS_H * VIS_W);
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		uint8_t *pri = s_pri + (y - VIS_Y0) * VIS_W;
		if (s_snap->sc2on) {   // bg: 64 x 64 of 32x32 (bg_tilemap_scan), priority 1
			const uint32_t ty = (uint32_t)y & 2047;
			for (int x = 0; x < VIS_W; x++) {
				const uint32_t tx = ((uint32_t)x + s_snap->bgx) & 2047;
				const uint32_t col = tx >> 5, row = ty >> 5;
				const uint32_t idx = ((col * 32 & 0xe0) >> 5) + ((row * 32 & 0xe0) >> 2) + ((col * 32 & 0x3f00) >> 1) + 0x4000;
				const uint8_t attr = s_tilerom[idx % s_tilerom_size];
				const uint8_t color = s_tilerom[(idx + 64) % s_tilerom_size];
				const uint32_t code = (uint32_t)(attr & 0x3f) % s_gbg.count;
				const uint32_t px = (attr & 0x40) ? 31 - (tx & 31) : tx & 31, py = (attr & 0x80) ? 31 - (ty & 31) : ty & 31;
				d[x] = (uint16_t)(64 * 4 + color * 4 + s_gbg.pix[code * 1024 + py * 32 + px]);
				pri[x] = 1;
			}
		} else
			memset(d, 0, VIS_W * 2);
		if (s_snap->sc1on) {   // fg: 128 x 128 of 16x16 (fg_tilemap_scan), pen 0 clear, priority 2
			const uint32_t ty = ((uint32_t)y + s_snap->fgy) & 2047;
			for (int x = 0; x < VIS_W; x++) {
				const uint32_t tx = ((uint32_t)x + s_snap->fgx) & 2047;
				const uint32_t col = tx >> 4, row = ty >> 4;
				const uint32_t idx = ((col * 16 & 0xf0) >> 4) + (row * 16 & 0xf0) + (col * 16 & 0x700) + ((row * 16 & 0x700) << 3);
				const uint32_t code = s_tilerom[idx % s_tilerom_size] % s_gfg.count;
				const uint8_t p = s_gfg.pix[code * 256 + (ty & 15) * 16 + (tx & 15)];
				if (p) {
					d[x] = (uint16_t)(2 * 64 * 4 + p);
					pri[x] = 2;
				}
			}
		}
	}
	if (s_snap->objon)
		for (int o = 0; o < 0x1000; o += 32) {
			const uint8_t *s = &s_snap->spr[o];
			const uint32_t pmask = (s[1] & 0x40) ? 0xcccc : 0;   // (GFX_PMASK_2)
			rbx_gfx_prio_transpen(&s_gspr, s[0], (uint16_t)(2 * 64 * 4 + 16 * 16 + (s[1] & 0x0f) * 16), s[1] & 0x10, s[1] & 0x20,
			                      s[3] - ((s[1] & 0x80) << 1), s[2] - VIS_Y0, rbx_video.bitmap, s_pri, rbx_video.pitch, VIS_W,
			                      VIS_H, pmask);
		}
	if (s_snap->chon)
		for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
			uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
			for (int x = 0; x < VIS_W; x++) {
				const uint32_t idx = ((uint32_t)y >> 3) * 32 + ((uint32_t)x >> 3);
				const uint8_t c = s_snap->cram[idx];
				const uint32_t code = (uint32_t)(s_snap->vram[idx] + 2 * (c & 0x80)) % s_gchar.count;
				const uint32_t pen = (c & 0x3f) * 4 + s_gchar.pix[code * 64 + (y & 7) * 8 + (x & 7)];
				if (s_ctab[pen] != 0xcf)   // (configure_groups(gfx, 0xcf))
					d[x] = (uint16_t)pen;
			}
		}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spr, t->sprbuf, sizeof(t->sprbuf));
	memcpy(s_snap->vram, t->vram, sizeof(t->vram));
	memcpy(s_snap->cram, t->cram, sizeof(t->cram));
	s_snap->bgx = (uint16_t)(t->bg_scroll[1] << 8 | t->bg_scroll[0]);
	s_snap->fgx = (uint16_t)(t->nbg_y[1] << 8 | t->nbg_y[0]);
	s_snap->fgy = (uint16_t)(t->nbg_x[1] << 8 | t->nbg_x[0]);
	s_snap->chon = t->chon;
	s_snap->objon = t->objon;
	s_snap->sc1on = t->sc1on;
	s_snap->sc2on = t->sc2on;
	rbx_video_frame_done();
}

static void z_setup(z80 *z, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t), const uint8_t **map)
{
	z80_init(z);
	z->read_byte = rd;
	z->write_byte = wr;
	z->port_in = z_in;
	z->port_out = z_out;
	z->read_map = map;
}

static void ex_reset(void)
{
	memset(t, 0, offsetof(__typeof__(*t), state_end));
	z_setup(&s_main, main_read, main_write, s_mmap);
	z_setup(&s_snd, snd_read, snd_write, s_zmap);
	ay8910_reset(&s_ay);
	sn76496_reset(&s_sn[0]);
	sn76496_reset(&s_sn[1]);
	s_mix_n = 0;
}

static void ex_exit(void);

static int ex_init(const rbx_game *g)
{
	free(s_pri);
	s_pri = (uint8_t *)calloc(VIS_H, VIS_W);
	if (!s_pri) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	free(t);
	t = calloc(1, sizeof(*t));
	if (!t) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t s1, s2, s3, s4, sp, si;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_tilerom = rbx_region("tilerom", &s_tilerom_size);
	uint8_t *chars = rbx_region("chars", &s1), *bg = rbx_region("32x32tiles", &s2), *fg = rbx_region("16x16tiles", &s3);
	uint8_t *spr = rbx_region("sprites", &s4);
	const uint8_t *prom = rbx_region("proms", &sp), *irq = rbx_region("irqprom", &si);
	if (!s_rom || !s_zrom || s_zrom_size < 0x4000 || !s_tilerom || !chars || !bg || !fg || !spr || !prom || sp < 0x500 ||
	    !irq || si < 0x100) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memcpy(s_irqprom, irq, 0x100);
	{	// palette (pal4bit PROMs), the lookup tables
		uint32_t rgb[256];
		for (int i = 0; i < 256; i++)
			rgb[i] = (uint32_t)((prom[i] & 15) * 17) << 16 | (uint32_t)((prom[256 + i] & 15) * 17) << 8 | (uint32_t)((prom[512 + i] & 15) * 17);
		const uint8_t *cp = prom + 0x300;
		for (int i = 0; i < 0x100; i++)
			s_ctab[i] = (uint8_t)(cp[i] | 0xc0);
		for (int i = 0x100; i < 0x200; i++)
			s_ctab[i] = cp[i];
		for (int i = 0x200; i < 0x300; i++)
			s_ctab[i] = (uint8_t)(cp[i] | 0x40);
		for (int i = 0x300; i < 0x400; i++)
			s_ctab[i] = (uint8_t)(cp[i] | (cp[i + 0x100] << 4) | 0x80);
		for (int i = 0; i < NPENS; i++)
			s_pens[i] = rgb[s_ctab[i]];
	}
	{	// charlayout, tilelayout (32x32, 2 bpp), spritelayout (RGN_FRAC(1,2)) for the 16x16 tiles and the sprites
		static const uint32_t pc[2] = { 4, 0 }, xc[8] = { 0, 1, 2, 3, 8, 9, 10, 11 };
		static const uint32_t yc[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
		static uint32_t xt[32], yt[32], xs[16], ys[16];
		for (int i = 0; i < 32; i++) {
			xt[i] = (uint32_t)((i >> 3) * 512 + ((i >> 2) & 1) * 8 + (i & 3));
			yt[i] = (uint32_t)i * 16;
		}
		for (int i = 0; i < 16; i++) {
			xs[i] = (uint32_t)((i >> 3) * 256 + ((i >> 2) & 1) * 8 + (i & 3));
			ys[i] = (uint32_t)i * 16;
		}
		const uint32_t h3 = s3 / 2 * 8, h4 = s4 / 2 * 8;
		const uint32_t p3[4] = { h3 + 4, h3, 4, 0 }, p4[4] = { h4 + 4, h4, 4, 0 };
		const rbx_gfx_layout lc = { 8, 8, 2, pc, xc, yc, 128 }, lt = { 32, 32, 2, pc, xt, yt, 2048 };
		const rbx_gfx_layout l3 = { 16, 16, 4, p3, xs, ys, 512 }, l4 = { 16, 16, 4, p4, xs, ys, 512 };
		if (rbx_gfx_decode(&s_gchar, &lc, chars, s1) || rbx_gfx_decode(&s_gbg, &lt, bg, s2) ||
		    rbx_gfx_decode(&s_gfg, &l3, fg, s3 / 2) || rbx_gfx_decode(&s_gspr, &l4, spr, s4 / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			ex_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	ay8910_init(&s_ay, AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 26, 4, 0);   // (0.10)
	sn76496_init(&s_sn[0], SN_CLOCK, SN76489A_CONFIG, 92, 8, 0);   // (0.36; the AY-8910's rate)
	sn76496_init(&s_sn[1], SN_CLOCK, SN76489A_CONFIG, 92, 8, 0);
	if (!s_snap || rbx_audio_setup(ay8910_rate(&s_ay)) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		ex_exit();
		return -1;
	}
	memset(s_mmap, 0, sizeof(s_mmap));
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0xc0 && (uint32_t)p * 256 + 256 <= s_rom_size; p++)
		s_mmap[p] = s_rom + p * 256;
	for (int p = 0; p < 0x40; p++)
		s_zmap[p] = s_zrom + p * 256;
	static const char *const names[5] = { "SYSTEM", "P1", "P2", "DSW0", "DSW1" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	ex_reset();
	return 0;
}

static void ex_exit(void)
{
	rbx_video_wait_idle();
	rbx_gfx_free(&s_gchar);
	rbx_gfx_free(&s_gbg);
	rbx_gfx_free(&s_gfg);
	rbx_gfx_free(&s_gspr);
	free(s_snap);
	s_snap = NULL;
	free(s_pri);
	s_pri = NULL;
	free(t);
	t = NULL;
}

static void ex_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		const uint8_t irq = s_irqprom[line & 0xff];
		if (irq & 8) {
			s_main.int_data = (uint8_t)(0xc7 | ((irq << 3) & 0x18));
			t->main_irq = 1;
		}
		if (irq & 4) {
			s_snd.int_data = 0xff;
			t->snd_irq = 1;
		}
		z80_run(&s_main, s_main.cyc + (unsigned long)rbx_line_cycles(&t->main_frac, CPU_CLOCK, FRAME_MHZ, LINES), &t->main_irq, 1);
		z80_run(&s_snd, s_snd.cyc + (unsigned long)rbx_line_cycles(&t->snd_frac, CPU_CLOCK, FRAME_MHZ, LINES), &t->snd_irq, 1);
		const uint32_t ac = (uint32_t)rbx_line_cycles(&t->ay_frac, AY_CLOCK, FRAME_MHZ, LINES);
		const uint32_t sc = (uint32_t)rbx_line_cycles(&t->sn_frac, SN_CLOCK, FRAME_MHZ, LINES);
		const int room = 2048 - s_mix_n;
		const int n = ay8910_render(&s_ay, ac, s_mix + s_mix_n, room);
		sn76496_render(&s_sn[0], sc, s_mix + s_mix_n, n);
		sn76496_render(&s_sn[1], sc, s_mix + s_mix_n, n);
		s_mix_n += n;
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int z_state(void *ctx, const char *tag, z80 *z, int load, uint8_t (*rd)(void *, uint16_t),
                   void (*wr)(void *, uint16_t, uint8_t), const uint8_t **map)
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
		*z = c;
	}
	return rc;
}

static int ex_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "ex", t, (uint32_t)offsetof(__typeof__(*t), state_end));
	rc |= z_state(ctx, "ex_main", &s_main, load, main_read, main_write, s_mmap);
	rc |= z_state(ctx, "ex_snd", &s_snd, load, snd_read, snd_write, s_zmap);
	rc |= rbx_state_io(ctx, "ex_ay", &s_ay, ay8910_state_size());
	rc |= rbx_state_io(ctx, "ex_sn0", &s_sn[0], sn76496_state_size());
	rc |= rbx_state_io(ctx, "ex_sn1", &s_sn[1], sn76496_state_size());
	return rc ? -1 : 0;
}

static int ex_save(void *ctx) { return ex_state(ctx, 0); }
static int ex_load(void *ctx) { return ex_state(ctx, 1); }
static int ex_supported(const char *machine) { return !strcmp(machine, "exedexes"); }

const rbx_driver rbx_drv_exedexes = { "Capcom Exed Exes", ex_init, ex_exit, ex_reset, ex_frame, ex_save, ex_load,
                                      ex_supported, ex_render, NULL };
