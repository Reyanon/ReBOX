/*  Konami's Ping-Pong. Ported from MAME src/mame/konami/pingpong.cpp (BSD-3-Clause, copyright-holders: Jarek
    Parchanski). A Z80 at 3.072 MHz (its IRQ at line 240, NMIs every 32 lines), a 32x32 tilemap of 2 bpp tiles, 20
    sprites clipped to the playfield, the colours through the lookup PROMs; the sound: an SN76489A rendered line by line
    on the main thread.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/sn76496.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 3072000u     // (18.432 MHz / 6; the SN76489A too)
#define SND_RATE 48000
#define LINES 262              // (set_refresh_hz(60), set_size(456, 262), set_vblank_time(0))
#define VIS_W 256
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
static int s_port_system, s_port_inputs, s_port_dsw[2];

static struct {
	uint8_t vram[0x800];     // 8000-87ff (8000 the colour RAM, 8400 the video RAM)
	uint8_t ram[0x800];      // 9000-97ff (9003-9052 the sprites)
	uint8_t int_enable;
	int64_t z_end;           // (the CPU's line target: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, sn_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t vram[0x800], spr[0x50]; } s_vs;

static void capture(void)
{
	memcpy(s_vs.vram, t.vram, sizeof(s_vs.vram));
	memcpy(s_vs.spr, t.ram + 3, sizeof(s_vs.spr));
}

static void pp_render(void)   // (screen_update)
{
	for (int i = 0; i < 0x100; i++) {   // (palette: 0-ff the characters' 16-31, 100-1ff the sprites' (bits 0-3 reversed))
		const uint8_t c = s_prom[0x120 + i];
		const uint8_t sw = (uint8_t)((c & 0xf0) | (c & 1) << 3 | (c & 2) << 1 | (c & 4) >> 1 | (c & 8) >> 3);
		rbx_video.palette[i] = s_rgb[(s_prom[0x20 + i] & 0x0f) | 0x10];
		rbx_video.palette[0x100 + i] = s_rgb[sw & 0x1f];
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		for (int col = 0; col < 32; col++) {
			const int idx = (y >> 3) * 32 + col;
			const int attr = s_vs.vram[idx];
			const int code = s_vs.vram[0x400 + idx] + ((attr & 0x20) << 3);
			const int pen = (attr & 0x1f) * 4;
			const int fx = (attr & 0x40) != 0, fy = (attr & 0x80) != 0;
			const uint8_t *src = s_chr + (rbx_wrap((uint32_t)code, s_nchr)) * 64 + (fy ? 7 - (y & 7) : (y & 7)) * 8;
			for (int i = 0; i < 8; i++)
				d[col * 8 + i] = (uint16_t)(pen + src[fx ? 7 - i : i]);
		}
	}
	for (int offs = 0x50 - 4; offs >= 0; offs -= 4) {   // (draw_sprites: clipped to lines 32-231, transmask of colour 0)
		const int sx = s_vs.spr[offs + 3], sy = 241 - s_vs.spr[offs + 1];
		const int flipx = (s_vs.spr[offs] & 0x40) != 0, flipy = (s_vs.spr[offs] & 0x80) != 0;
		const int color = s_vs.spr[offs] & 0x1f;
		const uint8_t *src = s_spr + (rbx_wrap((uint32_t)(s_vs.spr[offs + 2] & 0x7f), s_nspr)) * 256;
		for (int j = 0; j < 16; j++) {
			const int yy = sy + j;
			if (yy < 4 * 8 || yy > 29 * 8 - 1)
				continue;
			const uint8_t *s = src + (flipy ? 15 - j : j) * 16;
			uint16_t *d = bm + (yy - VIS_Y0) * pitch;
			for (int k = 0; k < 16; k++) {
				const int xx = sx + k;
				if (xx >= VIS_W)
					continue;
				const int p = s[flipx ? 15 - k : k];
				const int pen = 0x100 + color * 4 + p;
				if (s_prom[0x120 + color * 4 + p])   // (its colour, the bit-swapped PROM value, not 0)
					d[xx] = (uint16_t)pen;
			}
		}
	}
}

// ---- the bus (pingpong_map) ----
static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }

static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return s_rom[rbx_wrap(a, s_rom_size)];
	if (a < 0x8800)
		return t.vram[a - 0x8000];
	if (a >= 0x9000 && a < 0x9800)
		return t.ram[a - 0x9000];
	switch (a) {
	case 0xa800: return port(s_port_system);
	case 0xa880: return port(s_port_inputs);
	case 0xa900: return port(s_port_dsw[0]);
	case 0xa980: return port(s_port_dsw[1]);
	}
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8800)
		t.vram[a - 0x8000] = v;
	else if (a >= 0x9000 && a < 0x9800)
		t.ram[a - 0x9000] = v;
	else if (a == 0xa000)
		t.int_enable = v & 0x0c;   // (coin_w)
	else if (a == 0xa400)
		sn76496_write(&s_sn, v);
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// ---- the sound: a line's samples (the SN76489A 1.0) ----
static int32_t s_out[1024];
static int s_out_n;

static void sound_line(void)
{
	t.snd_frac += SND_RATE;
	int n = (int)(t.snd_frac / (60u * LINES));
	t.snd_frac -= (uint64_t)n * (60u * LINES);
	if (n <= 0)
		return;
	if (n > 16)
		n = 16;
	int32_t sn[64];
	memset(sn, 0, sizeof(sn));
	t.sn_frac += (uint64_t)CPU_CLOCK * (uint64_t)n;
	const uint32_t sc = (uint32_t)(t.sn_frac / SND_RATE);
	t.sn_frac -= (uint64_t)sc * SND_RATE;
	const int k = sn76496_render(&s_sn, sc, sn, 64);
	for (int i = 0; i < n && s_out_n < 1024; i++)
		s_out[s_out_n++] = k > 0 ? sn[i * k / n] : 0;
}

// ---- init / frame ----
static void pp_exit(void);
static void pp_reset(void);

static int pp_init(const rbx_game *g)
{
	uint32_t ts, ss, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_prom = rbx_region("proms", &ps);
	const uint8_t *tiles = rbx_region("tiles", &ts), *sprites = rbx_region("sprites", &ss);
	if (!s_rom || !s_prom || ps < 0x220 || !tiles || ts < 0x2000 || !sprites || ss < 0x2000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	// (charlayout: 512 of 8x8 2 bpp, planes { 4, 0 }, x bits 3-0 of a byte, the right half 8 bytes on; spritelayout:
	// 128 of 16x16, the 4-pixel columns 24, 16, 8, 0 bytes on, the lower half 32 on)
	s_nchr = 512;
	s_nspr = 128;
	s_chr = (uint8_t *)malloc((size_t)s_nchr * 64);
	s_spr = (uint8_t *)malloc((size_t)s_nspr * 256);
	if (!s_chr || !s_spr) {
		rbx_fail("%s: out of memory", g->set);
		pp_exit();
		return -1;
	}
#define PIX(b, q) (uint8_t)((((b) >> (q)) & 1) << 1 | (((b) >> (4 + (q))) & 1))
	for (uint32_t e = 0; e < s_nchr; e++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++)
				s_chr[e * 64 + (uint32_t)(y * 8 + x)] = PIX(tiles[e * 16 + (uint32_t)((x >> 2) * 8 + y)], x & 3);
	for (uint32_t e = 0; e < s_nspr; e++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				const uint32_t o = e * 64 + (uint32_t)((3 - (x >> 2)) * 8 + (y & 7) + (y >> 3) * 32);
				s_spr[e * 256 + (uint32_t)(y * 16 + x)] = PIX(sprites[o], x & 3);
			}
#undef PIX
	for (int i = 0; i < 32; i++) {
		const uint8_t c = s_prom[i];
		const int r = 0x21 * (c & 1) + 0x47 * ((c >> 1) & 1) + 0x97 * ((c >> 2) & 1);
		const int gg = 0x21 * ((c >> 3) & 1) + 0x47 * ((c >> 4) & 1) + 0x97 * ((c >> 5) & 1);
		const int b = 0x47 * ((c >> 6) & 1) + 0x97 * ((c >> 7) & 1);
		s_rgb[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	sn76496_init(&s_sn, CPU_CLOCK, SN76489A_CONFIG, 256, 32, 1);
	s_port_system = rbx_port_find("SYSTEM");
	s_port_inputs = rbx_port_find("INPUTS");
	s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	if (rbx_audio_setup(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		pp_exit();
		return -1;
	}
	pp_reset();
	return 0;
}

static void pp_exit(void)
{
	free(s_chr);
	free(s_spr);
	s_chr = s_spr = NULL;
}

static void pp_reset(void)
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
	for (int p = 0; p < 0x80; p++)
		s_zmap[p] = (uint32_t)p * 256 + 256 <= s_rom_size ? s_rom + p * 256 : NULL;
	for (int p = 0; p < 8; p++) {
		s_zmap[0x80 + p] = t.vram + p * 256;
		s_zmap[0x90 + p] = t.ram + p * 256;
	}
	s_z.read_map = s_zmap;
}

static void pp_frame(void)
{
	int irq = 0;
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update; pingpong_interrupt's IRQ at 240)
			capture();
			rbx_video_frame_done();
			if (t.int_enable & 0x04)
				irq = 1;   // (HOLD_LINE)
		} else if (vpos % 32 == 0 && (t.int_enable & 0x08))   // (the NMI every 32 lines)
			z80_gen_nmi(&s_z);
		t.cyc_frac += CPU_CLOCK;
		const unsigned long cyc = (unsigned long)(t.cyc_frac / (60u * LINES));
		t.cyc_frac -= (uint64_t)cyc * (60u * LINES);
		t.z_end += (int64_t)cyc;
		z80_run(&s_z, (unsigned long)t.z_end, &irq, 1);
		sound_line();
	}
	for (int i = 0; i < s_out_n; i++)
		rbx_audio_put(s_out[i]);
	s_out_n = 0;
	t.frame++;
}

static int pp_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "pp", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "pp_sn", &s_sn, sn76496_state_size());
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "pp_z80", &z, sizeof(z));
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

static int pp_save(void *ctx) { return pp_state(ctx, 0); }
static int pp_load(void *ctx) { return pp_state(ctx, 1); }
static int pp_supported(const char *machine) { return !strcmp(machine, "pingpong"); }

const rbx_driver rbx_drv_pingpong = { "Konami Ping-Pong", pp_init, pp_exit, pp_reset, pp_frame, pp_save, pp_load,
                                      pp_supported, pp_render, NULL };
