/*  Konami Green Beret / Rush'n Attack / Mr. Goemon. Ported from MAME src/mame/konami/gberet.cpp (BSD-3-Clause,
    copyright-holders: Nicola Salmoria) and k005849.cpp (the scroll RAM, the interrupts). A Z80 at 3.072 MHz (the
    K005849's IRQ and FIRQ merged on its INT, its NMI), a 64x32 tilemap with row scroll in two categories around the
    sprites (its transparent pens those of colour 0x10), the sprites, the colours through the lookup PROMs; the sound:
    an SN76489A rendered line by line on the main thread. The flip screen not done.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/sn76496.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 3072000u     // (18.432 MHz / 6)
#define SN_CLOCK 1536000u      // (18.432 MHz / 12)
#define SND_RATE 48000
#define PIX_CLOCK 6144000u     // (set_raw(18.432 MHz / 3, 384, 0+8, 256-8, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_X0 8
#define VIS_W 240
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 0x200

static uint8_t *s_rom, *s_prom;
static uint32_t s_rom_size;
static uint8_t *s_chr, *s_spr;   // (a byte a pixel)
static uint32_t s_nchr, s_nspr;
static uint32_t s_rgb[32];
static z80 s_z;
static const uint8_t *s_zmap[256];
static sn76496_t s_sn;
static int s_port_dsw[3], s_port_p[2], s_port_system, s_goemon;

static struct {
	uint8_t ram[0x2000];     // c000-dfff (c000 the colour RAM, c800 the video RAM, d000 / d100 the sprite RAMs)
	uint8_t scroll[0x40], ctrl[8];   // (the K005849)
	uint8_t soundlatch, mainbank, irq, firq, nmi;
	int64_t z_end;           // (the CPU's line target: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, sn_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t ram[0x1200], scroll[0x40], ctrl[8]; } s_vs;

static void capture(void)
{
	memcpy(s_vs.ram, t.ram, sizeof(s_vs.ram));
	memcpy(s_vs.scroll, t.scroll, sizeof(s_vs.scroll));
	memcpy(s_vs.ctrl, t.ctrl, sizeof(s_vs.ctrl));
}

static void draw_tiles(int category_only, uint16_t *bm, int pitch)
{
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch - VIS_X0;
		const int row = (y >> 3) & 31;
		const int sx = s_vs.scroll[row] | ((s_vs.scroll[row | 0x20] & 1) << 8);
		for (int x = VIS_X0; x < VIS_X0 + VIS_W;) {
			const int tx = (x + sx) & 511;
			int n = 8 - (tx & 7);
			if (n > VIS_X0 + VIS_W - x)
				n = VIS_X0 + VIS_W - x;
			const int idx = (y >> 3) * 64 + (tx >> 3);
			const int attr = s_vs.ram[idx];
			if (category_only && (attr & 0x80)) {
				x += n;
				continue;
			}
			const int code = s_vs.ram[0x800 + idx] + ((attr & 0x40) << 2);
			const int pen = (attr & 0x0f) * 16;
			const int fx = (attr & 0x10) != 0, fy = (attr & 0x20) != 0;
			const uint8_t *src = s_chr + (rbx_wrap((uint32_t)code, s_nchr)) * 64 + (fy ? 7 - (y & 7) : (y & 7)) * 8;
			const uint8_t *lut = s_prom + 0x20 + pen;
			for (int i = 0; i < n; i++) {
				const int px = (tx + i) & 7;
				const int p = src[fx ? 7 - px : px];
				if (!category_only || (lut[p] & 0x0f))   // (configure_groups(.., 0x10): colour 0x10 transparent)
					d[x + i] = (uint16_t)(pen + p);
			}
			x += n;
		}
	}
}

static void gb_render(void)   // (screen_update)
{
	for (int i = 0; i < 0x100; i++) {   // (palette: 0-ff the characters' 16-31, 100-1ff the sprites' 0-15)
		rbx_video.palette[i] = s_rgb[(s_prom[0x20 + i] & 0x0f) | 0x10];
		rbx_video.palette[0x100 + i] = s_rgb[s_prom[0x120 + i] & 0x0f];
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	draw_tiles(0, bm, pitch);   // (TILEMAP_DRAW_OPAQUE | TILEMAP_DRAW_ALL_CATEGORIES)
	const uint8_t *sr = s_vs.ram + ((s_vs.ctrl[3] & 0x08) ? 0x1000 : 0x1100);
	for (int offs = 0; offs < 0xc0; offs += 4) {   // (draw_sprites: transmask of the pens mapping to colour 0)
		if (!sr[offs + 3])
			continue;
		const int attr = sr[offs + 1];
		const int code = sr[offs] + ((attr & 0x40) << 2);
		const int color = attr & 0x0f;
		const int sx = sr[offs + 2] - 2 * (attr & 0x80), sy = sr[offs + 3];
		const int flipx = (attr & 0x10) != 0, flipy = (attr & 0x20) != 0;
		const uint8_t *src = s_spr + (rbx_wrap((uint32_t)code, s_nspr)) * 256;
		const uint8_t *lut = s_prom + 0x120 + color * 16;
		for (int j = 0; j < 16; j++) {
			const int yy = sy + j;
			if (yy < VIS_Y0 || yy >= VIS_Y0 + VIS_H)
				continue;
			const uint8_t *s = src + (flipy ? 15 - j : j) * 16;
			uint16_t *d = bm + (yy - VIS_Y0) * pitch - VIS_X0;
			for (int k = 0; k < 16; k++) {
				const int xx = sx + k;
				if (xx < VIS_X0 || xx >= VIS_X0 + VIS_W)
					continue;
				const int p = s[flipx ? 15 - k : k];
				if (lut[p] & 0x0f)
					d[xx] = (uint16_t)(0x100 + color * 16 + p);
			}
		}
	}
	draw_tiles(1, bm, pitch);   // (category 0 over the sprites)
}

// ---- the bus (prg_map; mrgoemon f800-ffff the bank) ----
static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }

static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xc000)
		return s_rom[rbx_wrap(a, s_rom_size)];
	if (a < 0xe000)
		return t.ram[a - 0xc000];
	if (a < 0xe040)
		return t.scroll[a - 0xe000];
	switch (a) {
	case 0xf200: return port(s_port_dsw[1]);
	case 0xf400: return port(s_port_dsw[2]);
	case 0xf600: return port(s_port_dsw[0]);
	case 0xf601: return port(s_port_p[1]);
	case 0xf602: return port(s_port_p[0]);
	case 0xf603: return port(s_port_system);
	}
	if (s_goemon && a >= 0xf800)
		return s_rom[rbx_wrap((0xc000u + (uint32_t)t.mainbank * 0x800u + (a - 0xf800)), s_rom_size)];
	return 0;
}

static void ctrl_w(int offset, uint8_t v)   // (k005849 ctrl_w: the interrupts cleared when their enables go off)
{
	if (offset == 4) {
		if (~v & t.ctrl[4] & 0x02)
			t.irq = 0;
		if (~v & t.ctrl[4] & 0x04)
			t.firq = 0;
		if (~v & t.ctrl[4] & 0x01)
			t.nmi = 0;
	}
	t.ctrl[offset] = v;
}

static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xe000)
		t.ram[a - 0xc000] = v;
	else if (a >= 0xe000 && a < 0xe040)
		t.scroll[a - 0xe000] = v;
	else if (a >= 0xe040 && a < 0xe048)
		ctrl_w(a - 0xe040, v);
	else
		switch (a) {
		case 0xf000:   // (coin_counter_w; mrgoemon bits 5-7 the bank)
			if (s_goemon)
				t.mainbank = (v & 0xe0) >> 5;
			break;
		case 0xf200: t.soundlatch = v; break;
		case 0xf400: sn76496_write(&s_sn, t.soundlatch); break;   // (sound_w)
		}
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// ---- the sound: a line's samples (the SN76489A 1.0) ----
static int32_t s_out[1024];
static int s_out_n;

static void sound_line(void)
{
	t.snd_frac += (uint64_t)SND_RATE * HTOTAL;
	int n = (int)(t.snd_frac / PIX_CLOCK);
	t.snd_frac -= (uint64_t)n * PIX_CLOCK;
	if (n <= 0)
		return;
	if (n > 16)
		n = 16;
	int32_t sn[64];
	memset(sn, 0, sizeof(sn));
	t.sn_frac += (uint64_t)SN_CLOCK * (uint64_t)n;
	const uint32_t sc = (uint32_t)(t.sn_frac / SND_RATE);
	t.sn_frac -= (uint64_t)sc * SND_RATE;
	const int k = sn76496_render(&s_sn, sc, sn, 64);
	for (int i = 0; i < n && s_out_n < 1024; i++)
		s_out[s_out_n++] = k > 0 ? sn[i * k / n] : 0;
}

// ---- init / frame ----
static void gb_exit(void);
static void gb_reset(void);

static int gb_init(const rbx_game *g)
{
	uint32_t ts, ss, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_prom = rbx_region("proms", &ps);
	const uint8_t *tiles = rbx_region("tiles", &ts), *sprites = rbx_region("sprites", &ss);
	if (!s_rom || !s_prom || ps < 0x220 || !tiles || !sprites) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_goemon = !strcmp(g->set, "mrgoemon") || (g->parent && !strcmp(g->parent, "mrgoemon"));
	memset(&t, 0, sizeof(t));
	// (gfx_8x8x4_packed_msb; gfx_8x8x4_row_2x2_group_packed_msb: 16x16, the right half 32 bytes on, the lower 64)
	s_nchr = ts / 32;
	s_nspr = ss / 128;
	s_chr = (uint8_t *)malloc((size_t)s_nchr * 64);
	s_spr = (uint8_t *)malloc((size_t)s_nspr * 256);
	if (!s_chr || !s_spr) {
		rbx_fail("%s: out of memory", g->set);
		gb_exit();
		return -1;
	}
	for (uint32_t e = 0; e < s_nchr; e++)
		for (int i = 0; i < 32; i++) {
			s_chr[e * 64 + (uint32_t)i * 2] = tiles[e * 32 + (uint32_t)i] >> 4;
			s_chr[e * 64 + (uint32_t)i * 2 + 1] = tiles[e * 32 + (uint32_t)i] & 15;
		}
	for (uint32_t e = 0; e < s_nspr; e++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				const uint8_t b = sprites[e * 128 + (uint32_t)((x >> 3) * 32 + (y >> 3) * 64 + (y & 7) * 4 + (x & 7) / 2)];
				s_spr[e * 256 + (uint32_t)(y * 16 + x)] = (x & 1) ? b & 15 : b >> 4;
			}
	for (int i = 0; i < 32; i++) {
		const uint8_t c = s_prom[i];
		const int r = 0x21 * (c & 1) + 0x47 * ((c >> 1) & 1) + 0x97 * ((c >> 2) & 1);
		const int gg = 0x21 * ((c >> 3) & 1) + 0x47 * ((c >> 4) & 1) + 0x97 * ((c >> 5) & 1);
		const int b = 0x52 * ((c >> 6) & 1) + 0xad * ((c >> 7) & 1);
		s_rgb[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	sn76496_init(&s_sn, SN_CLOCK, SN76489A_CONFIG, 256, 32, 1);
	static const char *const dn[3] = { "DSW1", "DSW2", "DSW3" };
	for (int i = 0; i < 3; i++)
		s_port_dsw[i] = rbx_port_find(dn[i]);
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_system = rbx_port_find("SYSTEM");
	if (rbx_audio_setup(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		gb_exit();
		return -1;
	}
	rbx_audio.frame_mhz = (uint32_t)((uint64_t)PIX_CLOCK * 1000 / ((uint64_t)HTOTAL * LINES));   // (60.61 Hz)
	gb_reset();
	return 0;
}

static void gb_exit(void)
{
	free(s_chr);
	free(s_spr);
	s_chr = s_spr = NULL;
}

static void gb_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	s_out_n = 0;
	sn76496_reset(&s_sn);
	z80_init(&s_z);
	s_z.read_byte = z_rd;
	s_z.write_byte = z_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0xc0; p++)
		s_zmap[p] = (uint32_t)p * 256 + 256 <= s_rom_size ? s_rom + p * 256 : NULL;
	for (int p = 0; p < 0x20; p++)
		s_zmap[0xc0 + p] = t.ram + p * 256;
	s_z.read_map = s_zmap;
}

static void gb_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {
			capture();
			rbx_video_frame_done();
		}
		// (the K005849's scanline timer every 16 lines: the NMI at 16 + 32n, the IRQ and on odd frames the FIRQ at 240)
		if ((vpos & 0x0f) == 0 && vpos < LINES - 7) {
			if ((t.ctrl[4] & 0x01) && (vpos & 0x1f) == 0x10 && !t.nmi) {   // (ASSERT_LINE: an edge once)
				t.nmi = 1;
				z80_gen_nmi(&s_z);
			}
			if (vpos == 240) {
				if ((t.ctrl[4] & 0x04) && (t.frame & 1))
					t.firq = 1;
				if (t.ctrl[4] & 0x02)
					t.irq = 1;
			}
		}
		t.cyc_frac += (uint64_t)CPU_CLOCK * HTOTAL;
		const unsigned long cyc = (unsigned long)(t.cyc_frac / PIX_CLOCK);
		t.cyc_frac -= (uint64_t)cyc * PIX_CLOCK;
		t.z_end += (int64_t)cyc;
		int line_irq = t.irq || t.firq;   // (a level: cleared by ctrl_w)
		z80_run(&s_z, (unsigned long)t.z_end, &line_irq, 0);
		sound_line();
	}
	for (int i = 0; i < s_out_n; i++)
		rbx_audio_put(s_out[i]);
	s_out_n = 0;
	t.frame++;
}

static int gb_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "gb", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "gb_sn", &s_sn, sn76496_state_size());
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "gb_z80", &z, sizeof(z));
	if (!rc && load) {
		s_out_n = 0;
		z.read_byte = z_rd;
		z.write_byte = z_wr;
		z.port_in = z_in;
		z.port_out = z_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		z.op_rom = NULL;
		s_z = z;
	}
	return rc ? -1 : 0;
}

static int gb_save(void *ctx) { return gb_state(ctx, 0); }
static int gb_load(void *ctx) { return gb_state(ctx, 1); }
static int gb_supported(const char *machine) { return !strcmp(machine, "gberet"); }

const rbx_driver rbx_drv_gberet = { "Konami Green Beret", gb_init, gb_exit, gb_reset, gb_frame, gb_save, gb_load,
                                    gb_supported, gb_render, NULL };
