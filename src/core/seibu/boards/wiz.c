/*  Seibu Denshi / Seibu Kaihatsu Wiz, Kung-Fu Taikun, Stinger, Scion: a Z80 with two 8x8 column-scrolled tile layers
    drawn as sprites and two small sprite lists, a Z80 sound CPU with three (Stinger, Scion: two) AY-3-8910s; 256
    colors from three PROMs.
    Ported from MAME 0.289 src/mame/seibu/wiz.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria).
    This file is part of ReBOX. GPL-2.0-or-later.

    Not done: the Stinger / Scion discrete shot and explosion sounds, the flip screen of the sprites' clip, the coin
    counters.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"
#include "../../video/gfxlayout.h"
#include "../machine/smix.h"
#include "../video/seibu_draw.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 60000
#define LINES 256
#define VIS_Y0 16
#define VIS_H 224
#define CPU_CLOCK 3072000
#define AY_CLOCK 1536000

enum { M_KUNGFUT, M_KUNGFUTA, M_WIZ, M_STINGER, M_SCION };

static int s_m;
static uint8_t *s_rom, *s_zrom, *s_op;
static uint32_t s_rom_size, s_zrom_size;
static uint32_t s_pens[256];
static rbx_gfx s_gfx[9];
static int s_ngfx;
static int s_port[4];
static z80 s_main, s_snd;
static ay8910_t s_ay[3];
static int s_nay;
static smix_t *s_mix;
static int s_vis_x0, s_vis_w;

static struct {
	uint8_t ram[0x800], zram[0x400];
	uint8_t video[2][0x400], color[2][0x400], attr[2][0x40], spr[2][0x20];
	uint8_t latch, main_nmi_mask, sound_nmi_mask, sprite_bank, charbank[2], palbank[2], flipx, flipy, bgcolor;
	int main_irq, snd_irq;
	uint64_t main_frac, snd_frac, ay_frac;
	int snd_tick;
	uint8_t state_end;
} t;

static struct {
	uint8_t video[2][0x400], color[2][0x400], attr[2][0x40], spr[2][0x20];
	uint8_t sprite_bank, charbank[2], palbank[2], flipx, flipy, bgcolor;
} *s_snap;

// ---- the sound CPU ----
static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	a &= 0x7fff;
	if (a < 0x2000) return s_zrom[a & (s_zrom_size - 1)];
	if (a >= 0x2000 && a < 0x2400) return t.zram[a - 0x2000];
	if (s_m >= M_STINGER ? a == 0x3000 : a == 0x7000) return t.latch;
	return 0xff;
}

static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	a &= 0x7fff;
	if (a >= 0x2000 && a < 0x2400) { t.zram[a - 0x2000] = v; return; }
	if (s_m >= M_STINGER) {
		if (a == 0x3000) t.sound_nmi_mask = v & 1;
		else if (a == 0x5000 || a == 0x5001) ay8910_write(&s_ay[0], a & 1, v);
		else if (a == 0x6000 || a == 0x6001) ay8910_write(&s_ay[1], a & 1, v);
		return;
	}
	if (a == 0x4000 || a == 0x4001) ay8910_write(&s_ay[2], a & 1, v);
	else if (a == 0x5000 || a == 0x5001) ay8910_write(&s_ay[0], a & 1, v);
	else if (a == 0x6000 || a == 0x6001) ay8910_write(&s_ay[1], a & 1, v);
	else if (a == 0x7000) t.sound_nmi_mask = v & 1;
}

// ---- the main CPU ----
static uint8_t protection_r(void)
{
	const uint8_t v = t.color[1][0];
	if (s_m == M_WIZ) {
		switch (v) {
		case 0x35: return 0x25;
		case 0x8f: return 0x1f;
		case 0xa0: return 0x00;
		}
	} else if (s_m == M_KUNGFUTA) {
		switch (v) {
		case 0x3a: return 0x0a;
		case 0x5a: return 0xda;
		case 0x7a: return 0xfa;
		case 0xaa: return 0x0a;
		case 0xba: return 0x0a;
		}
	}
	return v;
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xc000) return s_rom[a];
	if (a < 0xc800) return t.ram[a - 0xc000];
	if (a >= 0xd000 && a < 0xd400) return t.video[1][a - 0xd000];
	if (a >= 0xd400 && a < 0xd800) {
		if (a == 0xd400 && (s_m == M_WIZ || s_m == M_KUNGFUTA))
			return protection_r();
		return t.color[1][a - 0xd400];
	}
	if (a >= 0xd800 && a < 0xd840) return t.attr[1][a - 0xd800];
	if (a >= 0xd840 && a < 0xd860) return t.spr[1][a - 0xd840];
	if (a >= 0xe000 && a < 0xe400) return t.video[0][a - 0xe000];
	if (a >= 0xe400 && a < 0xe800) return t.color[0][a - 0xe400];
	if (a >= 0xe800 && a < 0xe840) return t.attr[0][a - 0xe800];
	if (a >= 0xe840 && a < 0xe860) return t.spr[0][a - 0xe840];
	if (a == 0xf000) return (uint8_t)rbx_port_read(s_port[2]);
	if (a == 0xf008) return (uint8_t)rbx_port_read(s_port[3]);
	if (a == 0xf010) return (uint8_t)rbx_port_read(s_port[0]);
	if (a == 0xf018) return (uint8_t)rbx_port_read(s_port[1]);
	return 0;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xc800) { t.ram[a - 0xc000] = v; return; }
	if (a >= 0xd000 && a < 0xd400) { t.video[1][a - 0xd000] = v; return; }
	if (a >= 0xd400 && a < 0xd800) { t.color[1][a - 0xd400] = v; return; }
	if (a >= 0xd800 && a < 0xd840) { t.attr[1][a - 0xd800] = v; return; }
	if (a >= 0xd840 && a < 0xd860) { t.spr[1][a - 0xd840] = v; return; }
	if (a >= 0xe000 && a < 0xe400) { t.video[0][a - 0xe000] = v; return; }
	if (a >= 0xe400 && a < 0xe800) { t.color[0][a - 0xe400] = v; return; }
	if (a >= 0xe800 && a < 0xe840) { t.attr[0][a - 0xe800] = v; return; }
	if (a >= 0xe840 && a < 0xe860) { t.spr[0][a - 0xe840] = v; return; }
	switch (a) {
	case 0xf000: if (s_m == M_WIZ) t.sprite_bank = v & 1; break;
	case 0xf001: t.main_nmi_mask = v & 1; break;
	case 0xf002: case 0xf003: t.palbank[a & 1] = v & 1; break;
	case 0xf004: case 0xf005: t.charbank[a & 1] = v & 1; break;
	case 0xf006: t.flipx = v & 1; break;
	case 0xf007: t.flipy = v & 1; break;
	case 0xf800: t.latch = v; break;
	case 0xf818: t.bgcolor = v; break;
	default: break;
	}
}

static uint8_t z_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

// ---- the drawing ----
static void draw_tiles(const sd_target *tg, int layer, int gfx, int colortype)
{
	const rbx_gfx *g = &s_gfx[gfx];
	const int palbank = s_snap->palbank[1] << 4 | s_snap->palbank[0] << 3;
	for (int offs = 0x400 - 1; offs >= 0; offs--) {
		const int code = s_snap->video[layer][offs];
		int sx = offs & 0x1f;
		const int sy = offs >> 5;
		int color = s_snap->attr[layer][sx << 1 | 1] & 7;
		if (colortype)
			color = layer ? (s_snap->color[layer][offs] & 7) : ((color & 4) | (code & 3));
		int scroll = (8 * sy + 256 - s_snap->attr[layer][sx << 1]) & 0xff;
		if (s_snap->flipy)
			scroll = (248 - scroll) & 0xff;
		if (s_snap->flipx)
			sx = 31 - sx;
		sd_transpen(tg, g, (uint32_t)code, (uint32_t)(palbank | color) * 8, s_snap->flipx, s_snap->flipy, 8 * sx, scroll, 0);
	}
}

static void draw_sprites(const sd_target *tg, int set, int gfx)
{
	const rbx_gfx *g = &s_gfx[gfx];
	const int palbank = s_snap->palbank[1] << 4 | s_snap->palbank[0] << 3;
	for (int offs = 0x20 - 4; offs >= 0; offs -= 4) {
		const uint8_t *s = &s_snap->spr[set][offs];
		const int code = s[1];
		int sx = s[3], sy = s[0];
		const int color = s[2] & 7;
		if (!sx || !sy)
			continue;
		if (set == 0 && offs <= 8)
			sy += s_snap->flipy ? 1 : -1;
		if (s_snap->flipx)
			sx = 240 - sx;
		if (!s_snap->flipy)
			sy = 240 - sy;
		sd_transpen(tg, g, (uint32_t)code, (uint32_t)(palbank | color) * 8, s_snap->flipx, s_snap->flipy, sx, sy, 0);
	}
}

static void wz_render(void)
{
	memcpy(rbx_video.palette, s_pens, sizeof(s_pens));
	const sd_target tg = { rbx_video.bitmap, rbx_video.pitch, s_vis_x0, VIS_Y0, s_vis_w, VIS_H, NULL };
	for (int r = 0; r < VIS_H; r++) {
		uint16_t *d = rbx_video.bitmap + r * rbx_video.pitch;
		for (int x = 0; x < s_vis_w; x++)
			d[x] = s_snap->bgcolor;
	}
	if (s_m == M_WIZ) {
		draw_tiles(&tg, 0, 2 + ((s_snap->charbank[0] << 1) | s_snap->charbank[1]), 1);
		draw_tiles(&tg, 1, s_snap->charbank[1], 1);
		// (the sprites' visible area: x 16-255, or 0-239 flipped)
		sd_target st = tg;
		const int cx0 = s_snap->flipx ? 0 : 16;
		st.bm += cx0;
		st.x0 = cx0;
		st.w = 240;
		draw_sprites(&st, 1, 6);
		draw_sprites(&st, 0, 7 + s_snap->sprite_bank);
	} else {
		const int colortype = s_m == M_KUNGFUT || s_m == M_KUNGFUTA;
		draw_tiles(&tg, 0, 2 + s_snap->charbank[0], colortype);
		draw_tiles(&tg, 1, s_snap->charbank[1], colortype);
		draw_sprites(&tg, 1, 4);
		draw_sprites(&tg, 0, 5);
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->video, t.video, sizeof(t.video));
	memcpy(s_snap->color, t.color, sizeof(t.color));
	memcpy(s_snap->attr, t.attr, sizeof(t.attr));
	memcpy(s_snap->spr, t.spr, sizeof(t.spr));
	s_snap->sprite_bank = t.sprite_bank;
	memcpy(s_snap->charbank, t.charbank, 2);
	memcpy(s_snap->palbank, t.palbank, 2);
	s_snap->flipx = t.flipx;
	s_snap->flipy = t.flipy;
	s_snap->bgcolor = t.bgcolor;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void z_setup(z80 *z, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t))
{
	z80_init(z);
	z->read_byte = rd;
	z->write_byte = wr;
	z->port_in = z_in;
	z->port_out = z_out;
}

static void wz_reset(void)
{
	rbx_video_wait_idle();
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	z_setup(&s_main, main_read, main_write);
	s_main.op_rom = s_op;
	z_setup(&s_snd, snd_read, snd_write);
	for (int i = 0; i < s_nay; i++)
		ay8910_reset(&s_ay[i]);
	s_mix->n = 0;
	s_mix->frac = 0;
}

static void wz_exit(void);

// charlayout / spritelayout: 3 planes 0x2000 bytes apart, 256 elements from `off`
static int decode(rbx_gfx *g, const uint8_t *rgn, uint32_t size, uint32_t off, int sprite)
{
	static const uint32_t planes[3] = { 0x4000 * 8, 0x2000 * 8, 0 };
	static const uint32_t cx[8] = { 0, 1, 2, 3, 4, 5, 6, 7 }, cy[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
	static const uint32_t sx[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 64, 65, 66, 67, 68, 69, 70, 71 };
	static const uint32_t sy[16] = { 0, 8, 16, 24, 32, 40, 48, 56, 128, 136, 144, 152, 160, 168, 176, 184 };
	const rbx_gfx_layout lc = { 8, 8, 3, planes, cx, cy, 64 }, ls = { 16, 16, 3, planes, sx, sy, 256 };
	if (off + 0x4000 + (sprite ? 0x2000u : 0x800u) > size)
		return -1;
	return rbx_gfx_decode(g, sprite ? &ls : &lc, rgn + off, sprite ? 0x2000 : 0x800);
}

static int wz_init(const rbx_game *g)
{
	uint32_t s1, s2, sp;
	const char *m = g->machine;
	s_m = !strcmp(m, "wiz") ? M_WIZ : !strcmp(m, "kungfuta") ? M_KUNGFUTA : !strcmp(m, "stinger") ? M_STINGER
	    : !strcmp(m, "scion") ? M_SCION : M_KUNGFUT;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	const uint8_t *g1 = rbx_region("gfx1", &s1), *g2 = rbx_region("gfx2", &s2), *prom = rbx_region("proms", &sp);
	if (!s_rom || s_rom_size < 0xc000 || !s_zrom || s_zrom_size < 0x2000 || !g1 || !g2 || !prom || sp < 0x300) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_zrom_size = 0x2000;
	{   // palette: 4 bits a gun through 1k / 470 / 220 / 100 ohms
		static const double res[4] = { 1000, 470, 220, 100 };
		double w[4], sum = 0;
		for (int i = 0; i < 4; i++)
			sum += 1.0 / res[i];
		for (int i = 0; i < 4; i++)
			w[i] = 255.0 / res[i] / sum;
		for (int i = 0; i < 256; i++) {
			int c[3];
			for (int k = 0; k < 3; k++) {
				double v = 0;
				for (int bit = 0; bit < 4; bit++)
					if ((prom[i + 0x100 * k] >> bit) & 1)
						v += w[bit];
				c[k] = (int)(v + 0.5);
				if (c[k] > 255)
					c[k] = 255;
			}
			s_pens[i] = (uint32_t)(c[0] << 16 | c[1] << 8 | c[2]);
		}
	}
	int rc = 0;
	if (s_m == M_WIZ) {
		static const struct { int r; uint32_t off; int spr; } e[9] = {
			{ 1, 0, 0 }, { 1, 0x800, 0 }, { 2, 0x6000, 0 }, { 2, 0, 0 }, { 2, 0x800, 0 }, { 2, 0x6800, 0 },
			{ 1, 0, 1 }, { 2, 0, 1 }, { 2, 0x6000, 1 } };
		s_ngfx = 9;
		for (int i = 0; i < 9 && !rc; i++)
			rc = decode(&s_gfx[i], e[i].r == 1 ? g1 : g2, e[i].r == 1 ? s1 : s2, e[i].off, e[i].spr);
	} else {
		static const struct { int r; uint32_t off; int spr; } e[6] = {
			{ 1, 0, 0 }, { 1, 0x800, 0 }, { 2, 0, 0 }, { 2, 0x800, 0 }, { 1, 0, 1 }, { 2, 0, 1 } };
		s_ngfx = 6;
		for (int i = 0; i < 6 && !rc; i++)
			rc = decode(&s_gfx[i], e[i].r == 1 ? g1 : g2, e[i].r == 1 ? s1 : s2, e[i].off, e[i].spr);
	}
	s_vis_x0 = s_m == M_SCION ? 16 : 0;
	s_vis_w = 256 - s_vis_x0;
	s_snap = calloc(1, sizeof(*s_snap));
	s_mix = calloc(1, sizeof(*s_mix));
	s_nay = s_m >= M_STINGER ? 2 : 3;
	for (int i = 0; i < s_nay; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 26, 4, 0);
	if (rc || !s_snap || !s_mix || rbx_audio_setup(ay8910_rate(&s_ay[0])) || rbx_video_setup(s_vis_w, VIS_H, 256)) {
		rbx_fail("%s: out of memory", g->set);
		wz_exit();
		return -1;
	}
	smix_init(s_mix, ay8910_rate(&s_ay[0]), FRAME_MHZ, LINES);
	if (s_m == M_STINGER) {   // init_stinger: the opcodes decrypted
		static const uint8_t tbl[4][4] = { { 7, 3, 5, 0xa0 }, { 3, 7, 5, 0x88 }, { 5, 3, 7, 0x80 }, { 5, 7, 3, 0x28 } };
		s_op = calloc(1, 0x10000);
		if (!s_op) {
			rbx_fail("%s: out of memory", g->set);
			wz_exit();
			return -1;
		}
		for (int a = 0; a < 0xc000; a++) {
			const uint8_t src = s_rom[a];
			if (a & 0x2040) {
				s_op[a] = src;
				continue;
			}
			const uint8_t *tb = tbl[((a >> 3) & 1) + (((a >> 5) & 1) << 1)];
			const uint8_t v = (uint8_t)(((src >> tb[0]) & 1) << 7 | ((src >> 6) & 1) << 6 | ((src >> tb[1]) & 1) << 5 |
			                            ((src >> 4) & 1) << 4 | ((src >> tb[2]) & 1) << 3 | (src & 7));
			s_op[a] = v ^ tb[3];
		}
		for (int a = 0xc000; a < 0x10000; a++)   // (RAM and I/O above: fetched as data)
			s_op[a] = 0;
	}
	static const char *const names[4] = { "IN0", "IN1", "DSW0", "DSW1" };
	for (int i = 0; i < 4; i++)
		s_port[i] = rbx_port_find(names[i]);
	wz_reset();
	return 0;
}

static void wz_exit(void)
{
	rbx_video_wait_idle();
	for (int i = 0; i < 9; i++)
		rbx_gfx_free(&s_gfx[i]);
	free(s_snap);
	s_snap = NULL;
	free(s_mix);
	s_mix = NULL;
	free(s_op);
	s_op = NULL;
}

static void wz_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0 + VIS_H) {
			vblank_capture();
			if (t.main_nmi_mask)
				z80_gen_nmi(&s_main);
		}
		if ((line & 63) == 0 && t.sound_nmi_mask)   // (4 x 60 Hz)
			z80_gen_nmi(&s_snd);
		const int c = rbx_line_cycles(&t.main_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		z80_run(&s_main, s_main.cyc + (unsigned long)c, &t.main_irq, 1);
		const int cs = rbx_line_cycles(&t.snd_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		z80_run(&s_snd, s_snd.cyc + (unsigned long)cs, &t.snd_irq, 1);
		const uint32_t ac = (uint32_t)rbx_line_cycles(&t.ay_frac, AY_CLOCK, FRAME_MHZ, LINES);
		smix_begin(s_mix);
		for (int i = 0; i < s_nay; i++) {
			int32_t *tmp = smix_tmp(s_mix);
			const int n = ay8910_render(&s_ay[i], ac, tmp, SMIX_TMP);
			smix_add(s_mix, tmp, n);
		}
		smix_end(s_mix);
	}
	smix_flush(s_mix);
}

static int z_state(void *ctx, const char *tag, z80 *z, int load, uint8_t (*rd)(void *, uint16_t),
                   void (*wr)(void *, uint16_t, uint8_t), const uint8_t *op)
{
	z80 c = *z;
	const int rc = rbx_state_io(ctx, tag, &c, sizeof(c));
	if (!rc && load) {
		c.read_byte = rd;
		c.write_byte = wr;
		c.port_in = z_in;
		c.port_out = z_out;
		c.userdata = NULL;
		c.read_map = NULL;
		c.op_rom = op;
		*z = c;
	}
	return rc;
}

static int wz_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "wiz", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= z_state(ctx, "wiz_main", &s_main, load, main_read, main_write, s_op);
	rc |= z_state(ctx, "wiz_snd", &s_snd, load, snd_read, snd_write, NULL);
	for (int i = 0; i < s_nay; i++) {
		char tag[16] = "wiz_ay0";
		tag[6] = (char)('0' + i);
		rc |= rbx_state_io(ctx, tag, &s_ay[i], ay8910_state_size());
	}
	return rc ? -1 : 0;
}

static int wz_save(void *ctx) { return wz_state(ctx, 0); }
static int wz_load(void *ctx) { return wz_state(ctx, 1); }
static int wz_supported(const char *m)
{
	return !strcmp(m, "wiz") || !strcmp(m, "kungfut") || !strcmp(m, "kungfuta") || !strcmp(m, "stinger") ||
	       !strcmp(m, "scion");
}

const rbx_driver rbx_drv_wiz = { "Seibu Wiz", wz_init, wz_exit, wz_reset, wz_frame, wz_save, wz_load, wz_supported,
                                 wz_render, NULL };
