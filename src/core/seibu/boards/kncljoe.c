/*  Seibu Kaihatsu Knuckle Joe: a Z80 with a 512x256 8x8 tilemap (three row bands scrolled) and 16x16 sprites in four
    bands, two SN76489s; an M6803 sound CPU (Irem style) driving an AY-3-8910 through its ports; 128 + 16 colors from
    PROMs with a sprite lookup table.
    Ported from MAME 0.289 src/mame/seibu/kncljoe.cpp (BSD-3-Clause, copyright-holders: Ernesto Corvi).
    This file is part of ReBOX. GPL-2.0-or-later.

    The CPUs interleaved a scanline at a time. Not done: the flip screen, the coin counters.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/m6800/m6801.h"
#include "../../sound/ay8910.h"
#include "../../sound/sn76496.h"
#include "../../video/gfxlayout.h"
#include "../machine/smix.h"
#include "../video/seibu_draw.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 56738   // 6.144 MHz / (384 x 282)
#define LINES 282
#define VIS_X0 8
#define VIS_W 240
#define VIS_H 256
#define CPU_CLOCK 6000000
#define SND_CLOCK 3579545

static uint8_t *s_rom, *s_srom;
static uint32_t s_pens[256];
static rbx_gfx s_gtile, s_gspr[2];
static int s_port[5];
static z80 s_main;
static m6801 s_snd;
static ay8910_t s_ay;
static sn76496_t s_sn[2];
static smix_t *s_mix;

static struct {
	uint8_t video[0x1000], spr[0x800], ram[0x1000], scroll[2];
	uint8_t latch, port1, port2, tile_bank, sprite_bank, flip;
	int main_irq;
	uint64_t main_frac, snd_frac, ay_frac, sn_frac;
	uint8_t state_end;
} t;

static struct {
	uint8_t video[0x1000], spr[0x800], scroll[2], tile_bank, sprite_bank;
} *s_snap;

// ---- the sound CPU ----
static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	a &= 0x7fff;
	return a >= 0x2000 ? s_srom[a] : 0xff;
}

static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u; (void)v;
	a &= 0x7fff;
	if (a >= 0x1000 && a < 0x2000)   // sound_irq_ack_w
		m6801_set_irq_line(&s_snd, M6801_IRQ1_LINE, 0);
}

static uint8_t snd_port_r(void *u, int port)
{
	(void)u;
	if (port == 1)
		return (t.port2 & 0x08) ? ay8910_data_r(&s_ay) : 0xff;
	if (port == 2)
		return 0;
	return 0xff;
}

static void snd_port_w(void *u, int port, uint8_t data, uint8_t ddr)
{
	(void)u; (void)ddr;
	if (port == 1)
		t.port1 = data;
	else if (port == 2) {
		if ((t.port2 & 0x01) && !(data & 0x01) && (t.port2 & 0x08))   // the write latch: data_address_w
			ay8910_write(&s_ay, ((t.port2 >> 2) & 1) ^ 1, t.port1);
		t.port2 = data;
	}
}

static uint8_t ay_port_a(void *u) { (void)u; return t.latch; }

// ---- the main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xc000) return s_rom[a];
	if (a < 0xd000) return t.video[a - 0xc000];
	if (a >= 0xd800 && a <= 0xd804) return (uint8_t)rbx_port_read(s_port[a - 0xd800]);
	if (a >= 0xe800 && a < 0xf000) return t.spr[a - 0xe800];
	if (a >= 0xf000) return t.ram[a - 0xf000];
	return 0xff;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xd000) { t.video[a - 0xc000] = v; return; }
	if (a >= 0xe800 && a < 0xf000) { t.spr[a - 0xe800] = v; return; }
	if (a >= 0xf000) { t.ram[a - 0xf000] = v; return; }
	switch (a) {
	case 0xd000: case 0xd001: t.scroll[a & 1] = v; break;
	case 0xd800:   // sound_cmd_w
		if (!(v & 0x80))
			t.latch = v & 0x7f;
		else
			m6801_set_irq_line(&s_snd, M6801_IRQ1_LINE, 1);
		break;
	case 0xd801:   // control_w
		t.flip = v & 1;
		t.tile_bank = (v >> 4) & 1;
		t.sprite_bank = (v >> 2) & 1;
		break;
	case 0xd802: sn76496_write(&s_sn[0], v); break;
	case 0xd803: sn76496_write(&s_sn[1], v); break;
	default: break;
	}
}

static uint8_t z_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

// ---- the drawing ----
static void kj_render(void)
{
	memcpy(rbx_video.palette, s_pens, sizeof(s_pens));
	const int scrollx = s_snap->scroll[0] | s_snap->scroll[1] << 8;
	for (int y = 0; y < VIS_H; y++) {   // (the tilemap flipped in Y unless the screen is flipped)
		uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
		const int ty = 255 - y;
		const int sx = (ty >> 6) == 3 ? 0 : scrollx;
		for (int x = 0; x < VIS_W; x++) {
			const int tx = (x + VIS_X0 + sx) & 511;
			const int idx = (ty >> 3) * 64 + (tx >> 3);
			const uint8_t attr = s_snap->video[2 * idx + 1];
			const uint32_t code = (uint32_t)(s_snap->video[2 * idx] + ((attr & 0xc0) << 2) + (s_snap->tile_bank << 10));
			// (TILE_FLIPXY: bit 4 flips Y, bit 5 flips X)
			const int px = (attr & 0x20) ? 7 - (tx & 7) : tx & 7, py = (attr & 0x10) ? 7 - (ty & 7) : ty & 7;
			d[x] = (uint16_t)((attr & 0xf) * 8 + sd_elem(&s_gtile, code)[py * 8 + px]);
		}
	}
	const rbx_gfx *g = &s_gspr[s_snap->sprite_bank];
	for (int i = 0; i < 4; i++) {
		const int min_y = i * 64 + 1;
		const int h = min_y + 64 > VIS_H ? VIS_H - min_y : 64;
		const sd_target tg = { rbx_video.bitmap + min_y * rbx_video.pitch, rbx_video.pitch, VIS_X0, min_y, VIS_W, h, NULL };
		const int v = ~i & 3;
		const int band = ((v & 1) << 1) | ((v >> 1) & 1);
		for (int j = 0x7c; j >= 0; j -= 4) {
			const uint8_t *s = &s_snap->spr[band << 7 | j];
			const int sy = s[0] + 1, attr = s[1];
			int sx = s[3];
			const uint32_t code = (uint32_t)(s[2] | ((attr & 0x10) << 5) | ((attr & 0x20) << 3));
			if (sx >= 256 - 8)
				sx -= 256;
			sd_transpen(&tg, g, code, 0x80 + (uint32_t)(attr & 0x0f) * 8, (attr & 0x40) != 0, !(attr & 0x80), sx, sy, 0);
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->video, t.video, sizeof(t.video));
	memcpy(s_snap->spr, t.spr, sizeof(t.spr));
	memcpy(s_snap->scroll, t.scroll, 2);
	s_snap->tile_bank = t.tile_bank;
	s_snap->sprite_bank = t.sprite_bank;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void kj_reset(void)
{
	rbx_video_wait_idle();
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	z80_init(&s_main);
	s_main.read_byte = main_read;
	s_main.write_byte = main_write;
	s_main.port_in = z_in;
	s_main.port_out = z_out;
	m6801_reset(&s_snd);
	ay8910_reset(&s_ay);
	sn76496_reset(&s_sn[0]);
	sn76496_reset(&s_sn[1]);
	s_mix->n = 0;
	s_mix->frac = 0;
}

static void kj_exit(void);

static int kj_init(const rbx_game *g)
{
	uint32_t rs, ss, ts, s1, s2, ps;
	s_rom = rbx_region("maincpu", &rs);
	s_srom = rbx_region("soundcpu", &ss);
	const uint8_t *tl = rbx_region("tiles", &ts), *sp1 = rbx_region("sprites1", &s1), *sp2 = rbx_region("sprites2", &s2);
	const uint8_t *prom = rbx_region("proms", &ps);
	if (!s_rom || rs < 0xc000 || !s_srom || ss < 0x8000 || !tl || !sp1 || !sp2 || !prom || ps < 0x3a0) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{   // palette: 128 direct colors (4 bits a gun), 16 sprite colors through the lookup table
		uint32_t rgb[0x90];
		for (int i = 0; i < 0x80; i++)
			rgb[i] = (uint32_t)((prom[i] & 15) * 0x11 << 16 | (prom[i + 0x100] & 15) * 0x11 << 8 | (prom[i + 0x200] & 15) * 0x11);
		for (int i = 0; i < 0x10; i++) {
			const uint8_t p = prom[i + 0x300];
			const int r = 0x52 * ((p >> 6) & 1) + 0xad * ((p >> 7) & 1);
			const int gg = 0x21 * ((p >> 3) & 1) + 0x47 * ((p >> 4) & 1) + 0x97 * ((p >> 5) & 1);
			const int bb = 0x21 * (p & 1) + 0x47 * ((p >> 1) & 1) + 0x97 * ((p >> 2) & 1);
			rgb[0x80 + i] = (uint32_t)(r << 16 | gg << 8 | bb);
		}
		for (int i = 0; i < 0x80; i++) {
			s_pens[i] = rgb[i];
			s_pens[0x80 + i] = rgb[0x80 | (prom[0x320 + i] & 0x0f)];
		}
	}
	{   // gfx_8x8x3_planar, spritelayout (RGN_FRAC(1,3) planes)
		static const uint32_t cx[8] = { 0, 1, 2, 3, 4, 5, 6, 7 }, cy[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
		static const uint32_t sx[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 64, 65, 66, 67, 68, 69, 70, 71 };
		static const uint32_t sy[16] = { 0, 8, 16, 24, 32, 40, 48, 56, 128, 136, 144, 152, 160, 168, 176, 184 };
		const uint32_t ft = ts / 3 * 8, f1 = s1 / 3 * 8, f2 = s2 / 3 * 8;
		const uint32_t pt[3] = { 2 * ft, ft, 0 }, p1[3] = { 2 * f1, f1, 0 }, p2[3] = { 2 * f2, f2, 0 };
		const rbx_gfx_layout lt = { 8, 8, 3, pt, cx, cy, 64 };
		const rbx_gfx_layout l1 = { 16, 16, 3, p1, sx, sy, 256 }, l2 = { 16, 16, 3, p2, sx, sy, 256 };
		if (rbx_gfx_decode(&s_gtile, &lt, tl, ts / 3) || rbx_gfx_decode(&s_gspr[0], &l1, sp1, s1 / 3) ||
		    rbx_gfx_decode(&s_gspr[1], &l2, sp2, s2 / 3)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			kj_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	s_mix = calloc(1, sizeof(*s_mix));
	ay8910_init(&s_ay, AY8910_TYPE_AY8910, SND_CLOCK / 4, AY8910_LEGACY_OUTPUT, 77, 2, 0);
	s_ay.port_a_read = ay_port_a;
	for (int i = 0; i < 2; i++)
		sn76496_init(&s_sn[i], SND_CLOCK, SN76489_CONFIG, 77, 8, 0);
	if (!s_snap || !s_mix || rbx_audio_setup(48000) || rbx_video_setup(VIS_W, VIS_H, 256)) {
		rbx_fail("%s: out of memory", g->set);
		kj_exit();
		return -1;
	}
	smix_init(s_mix, 48000, FRAME_MHZ, LINES);
	memset(&s_snd, 0, sizeof(s_snd));
	s_snd.read = snd_read;
	s_snd.write = snd_write;
	s_snd.port_r = snd_port_r;
	s_snd.port_w = snd_port_w;
	s_snd.type = M6801_TYPE_6801;
	m6801_init(&s_snd);
	static const char *const names[5] = { "SYSTEM", "P1", "P2", "DSWA", "DSWB" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	kj_reset();
	return 0;
}

static void kj_exit(void)
{
	rbx_video_wait_idle();
	rbx_gfx_free(&s_gtile);
	rbx_gfx_free(&s_gspr[0]);
	rbx_gfx_free(&s_gspr[1]);
	free(s_snap);
	s_snap = NULL;
	free(s_mix);
	s_mix = NULL;
}

static void kj_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_H) {
			vblank_capture();
			s_main.int_data = 0xff;
			t.main_irq = 1;
		}
		if ((line & 3) == 0) {   // (the sound CPU's 4 kHz NMI: every 4 scanlines)
			m6801_set_irq_line(&s_snd, M6801_NMI_LINE, 1);
			m6801_set_irq_line(&s_snd, M6801_NMI_LINE, 0);
		}
		const int c = rbx_line_cycles(&t.main_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		z80_run(&s_main, s_main.cyc + (unsigned long)c, &t.main_irq, 1);
		m6801_execute(&s_snd, rbx_line_cycles(&t.snd_frac, SND_CLOCK / 4, FRAME_MHZ, LINES));
		smix_begin(s_mix);
		int32_t *tmp = smix_tmp(s_mix);
		int n = ay8910_render(&s_ay, (uint32_t)rbx_line_cycles(&t.ay_frac, SND_CLOCK / 4, FRAME_MHZ, LINES), tmp, SMIX_TMP);
		smix_add(s_mix, tmp, n);
		const uint32_t sc = (uint32_t)rbx_line_cycles(&t.sn_frac, SND_CLOCK, FRAME_MHZ, LINES);
		for (int i = 0; i < 2; i++) {
			tmp = smix_tmp(s_mix);
			n = sn76496_render(&s_sn[i], sc, tmp, SMIX_TMP);
			smix_add(s_mix, tmp, n);
		}
		smix_end(s_mix);
	}
	smix_flush(s_mix);
}

static int kj_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "kncljoe", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	z80 c = s_main;
	rc |= rbx_state_io(ctx, "kncljoe_main", &c, sizeof(c));
	if (!rc && load) {
		c.read_byte = main_read;
		c.write_byte = main_write;
		c.port_in = z_in;
		c.port_out = z_out;
		c.userdata = NULL;
		c.read_map = NULL;
		c.op_rom = NULL;
		s_main = c;
	}
	rc |= rbx_state_io(ctx, "kncljoe_snd", m6801_state_data(&s_snd), m6801_state_size());
	rc |= rbx_state_io(ctx, "kncljoe_ay", &s_ay, ay8910_state_size());
	rc |= rbx_state_io(ctx, "kncljoe_sn0", &s_sn[0], sn76496_state_size());
	rc |= rbx_state_io(ctx, "kncljoe_sn1", &s_sn[1], sn76496_state_size());
	return rc ? -1 : 0;
}

static int kj_save(void *ctx) { return kj_state(ctx, 0); }
static int kj_load(void *ctx) { return kj_state(ctx, 1); }
static int kj_supported(const char *m) { return !strcmp(m, "kncljoe"); }

const rbx_driver rbx_drv_kncljoe = { "Seibu Knuckle Joe", kj_init, kj_exit, kj_reset, kj_frame, kj_save, kj_load,
                                     kj_supported, kj_render, NULL };
