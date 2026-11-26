/*  Konami Jail Break / Manhattan 24 Bunsyo. Ported from MAME src/mame/konami/jailbrek.cpp (BSD-3-Clause,
    copyright-holders: Ernesto Corvi) and k005849.cpp (the scroll RAM, the interrupts). A KONAMI1 at 1.536 MHz, a 64x32
    tilemap with row or column scroll, the sprites, the colours through the lookup PROMs; the sound: an SN76489A and a
    VLM5030 rendered line by line on the main thread. The flip screen not done.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../sound/sn76496.h"
#include "../../sound/vlm5030.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 1536000u     // (18.432 MHz / 12; the SN76489A too)
#define SND_RATE 48000
#define PIX_CLOCK 6144000u     // (set_raw(18.432 MHz / 3, 384, 0+8, 256-8, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_X0 8
#define VIS_W 240
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 0x200

static uint8_t *s_rom, *s_prom, *s_vlmrom;
static uint32_t s_rom_size, s_vlmrom_size;
static uint8_t *s_chr, *s_spr;   // (a byte a pixel)
static uint32_t s_nchr, s_nspr;
static uint32_t s_rgb[32];
static m6809_t s_cpu;
static sn76496_t s_sn;
static vlm5030_t s_vlm;
static int s_port_dsw[3], s_port_p[2], s_port_system;

static struct {
	uint8_t ram[0x2000];     // 0000-1fff (0000 the colour RAM, 0800 the video RAM, 1000-10bf the sprites)
	uint8_t scroll[0x40], ctrl[8];   // (the K005849)
	int16_t vlm_out;
	int64_t cpu_end;         // (the CPU's line target: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, sn_frac, vlm_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t ram[0x10c0], scroll[0x40], ctrl[8]; } s_vs;

static void capture(void)
{
	memcpy(s_vs.ram, t.ram, sizeof(s_vs.ram));
	memcpy(s_vs.scroll, t.scroll, sizeof(s_vs.scroll));
	memcpy(s_vs.ctrl, t.ctrl, sizeof(s_vs.ctrl));
}

static void jb_render(void)   // (screen_update)
{
	for (int i = 0; i < 0x100; i++) {   // (palette: 0-ff the characters' 16-31, 100-1ff the sprites' 0-15)
		rbx_video.palette[i] = s_rgb[(s_prom[0x40 + i] & 0x0f) | 0x10];
		rbx_video.palette[0x100 + i] = s_rgb[s_prom[0x140 + i] & 0x0f];
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const uint8_t *sc = s_vs.scroll;
	const int cols = (s_vs.ctrl[2] & 0x04) != 0;   // (32 scroll columns of 16 pixels, else 32 scroll rows)
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch - VIS_X0;
		const int row = (y >> 3) & 31;
		const int sx = cols ? 0 : sc[row] | ((sc[row | 0x20] & 1) << 8);
		for (int x = VIS_X0; x < VIS_X0 + VIS_W;) {
			const int tx = (x + sx) & 511;
			int n = 8 - (tx & 7);
			if (n > VIS_X0 + VIS_W - x)
				n = VIS_X0 + VIS_W - x;
			const int c = (tx >> 4) & 31;
			const int ty = cols ? (y + (sc[c] | ((sc[c | 0x20] & 1) << 8))) & 255 : y;
			const int idx = (ty >> 3) * 64 + (tx >> 3);
			const int attr = s_vs.ram[idx];
			const int code = s_vs.ram[0x800 + idx] + ((attr & 0xc0) << 2);
			const int pen = (attr & 0x0f) * 16;
			const uint8_t *src = s_chr + (rbx_wrap((uint32_t)code, s_nchr)) * 64 + (ty & 7) * 8;
			for (int i = 0; i < n; i++)
				d[x + i] = (uint16_t)(pen + src[(tx + i) & 7]);
			x += n;
		}
	}
	const uint8_t *sr = s_vs.ram + 0x1000;
	for (int i = 0; i < 0xc0; i += 4) {   // (draw_sprites: transmask of the pens mapping to colour 0)
		const int attr = sr[i + 1];
		const int code = sr[i] + ((attr & 0x40) << 2);
		const int color = attr & 0x0f;
		const int flipx = (attr & 0x10) != 0, flipy = (attr & 0x20) != 0;
		const int sx = sr[i + 2] - ((attr & 0x80) << 1), sy = sr[i + 3];
		const uint8_t *src = s_spr + (rbx_wrap((uint32_t)code, s_nspr)) * 256;
		const uint8_t *lut = s_prom + 0x140 + color * 16;
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
}

// ---- the main CPU's bus (prg_map) ----
static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }

static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x2000)
		return t.ram[a];
	if (a < 0x2040)
		return t.scroll[a - 0x2000];
	switch (a) {
	case 0x3100: return port(s_port_dsw[1]);
	case 0x3200: return port(s_port_dsw[2]);
	case 0x3300: return port(s_port_system);
	case 0x3301: return port(s_port_p[0]);
	case 0x3302: return port(s_port_p[1]);
	case 0x3303: return port(s_port_dsw[0]);
	case 0x6000: return (uint8_t)vlm5030_bsy(&s_vlm);
	}
	if (a >= 0x8000)
		return s_rom[rbx_wrap(a, s_rom_size)];
	return 0;
}

static void ctrl_w(int offset, uint8_t v)   // (k005849 ctrl_w: the interrupts cleared when their enables go off)
{
	if (offset == 4) {
		if (~v & t.ctrl[4] & 0x02)
			m6809_set_irq(&s_cpu, 0);
		if (~v & t.ctrl[4] & 0x01)
			m6809_set_nmi(&s_cpu, 0);
	}
	t.ctrl[offset] = v;
}

static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x2000)
		t.ram[a] = v;
	else if (a < 0x2040)
		t.scroll[a - 0x2000] = v;
	else if (a < 0x2048)
		ctrl_w(a - 0x2040, v);
	else
		switch (a) {
		case 0x3100: sn76496_write(&s_sn, v); break;
		case 0x4000:   // (speech_w)
			vlm5030_st(&s_vlm, (v >> 1) & 1);
			vlm5030_rst(&s_vlm, (v >> 2) & 1);
			break;
		case 0x5000: vlm5030_data_w(&s_vlm, v); break;
		}
}

// ---- the sound: a line's samples (the SN76489A 1.0, the VLM5030 1.0) ----
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
	t.sn_frac += (uint64_t)CPU_CLOCK * (uint64_t)n;
	const uint32_t sc = (uint32_t)(t.sn_frac / SND_RATE);
	t.sn_frac -= (uint64_t)sc * SND_RATE;
	const int k = sn76496_render(&s_sn, sc, sn, 64);
	for (int i = 0; i < n && s_out_n < 1024; i++) {
		t.vlm_frac += vlm5030_rate(&s_vlm);
		while (t.vlm_frac >= SND_RATE) {
			t.vlm_frac -= SND_RATE;
			int16_t s;
			vlm5030_render(&s_vlm, &s, 1);
			t.vlm_out = s;
		}
		s_out[s_out_n++] = (k > 0 ? sn[i * k / n] : 0) + t.vlm_out;
	}
}

// ---- init / frame ----
static void jb_exit(void);
static void jb_reset(void);

static int jb_init(const rbx_game *g)
{
	uint32_t ts, ss, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_prom = rbx_region("proms", &ps);
	s_vlmrom = rbx_region("vlm", &s_vlmrom_size);
	const uint8_t *tiles = rbx_region("tiles", &ts), *sprites = rbx_region("sprites", &ss);
	if (!s_rom || !s_prom || ps < 0x240 || !s_vlmrom || !tiles || !sprites) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	// (gfx_8x8x4_packed_msb; gfx_8x8x4_row_2x2_group_packed_msb: 16x16, the right half 32 bytes on, the lower 64)
	s_nchr = ts / 32;
	s_nspr = ss / 128;
	s_chr = (uint8_t *)malloc((size_t)s_nchr * 64);
	s_spr = (uint8_t *)malloc((size_t)s_nspr * 256);
	if (!s_chr || !s_spr) {
		rbx_fail("%s: out of memory", g->set);
		jb_exit();
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
	for (int i = 0; i < 0x20; i++) {   // (pal4bit)
		const int r = (s_prom[i] & 15) * 0x11, gg = (s_prom[i] >> 4) * 0x11, b = (s_prom[i + 0x20] & 15) * 0x11;
		s_rgb[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	sn76496_init(&s_sn, CPU_CLOCK, SN76489A_CONFIG, 256, 32, 1);
	// (vlm_map: global_mask(0x1fff), the ROM's two halves the same)
	vlm5030_init(&s_vlm, 3579545, s_vlmrom, s_vlmrom_size < 0x2000 ? s_vlmrom_size : 0x2000);
	static const char *const dn[3] = { "DSW1", "DSW2", "DSW3" };
	for (int i = 0; i < 3; i++)
		s_port_dsw[i] = rbx_port_find(dn[i]);
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_system = rbx_port_find("SYSTEM");
	if (rbx_audio_setup(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		jb_exit();
		return -1;
	}
	rbx_audio.frame_mhz = (uint32_t)((uint64_t)PIX_CLOCK * 1000 / ((uint64_t)HTOTAL * LINES));   // (60.61 Hz)
	jb_reset();
	return 0;
}

static void jb_exit(void)
{
	free(s_chr);
	free(s_spr);
	s_chr = s_spr = NULL;
}

static void jb_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	s_out_n = 0;
	sn76496_reset(&s_sn);
	vlm5030_reset(&s_vlm);
	m6809_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	s_cpu.konami1 = 1;
	for (int p = 0; p < 0x20; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + ((uint32_t)p * 256) % s_rom_size;
	m6809_reset(&s_cpu);
}

static void jb_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {
			capture();
			rbx_video_frame_done();
		}
		// (the K005849's scanline timer every 16 lines: the NMI at 16 + 32n, the IRQ at 240)
		if ((vpos & 0x0f) == 0 && vpos < LINES - 7) {
			if ((t.ctrl[4] & 0x01) && (vpos & 0x1f) == 0x10)
				m6809_set_nmi(&s_cpu, 1);
			if (vpos == 240 && (t.ctrl[4] & 0x02))
				m6809_set_irq(&s_cpu, 1);
		}
		t.cyc_frac += (uint64_t)CPU_CLOCK * HTOTAL;
		const int64_t cyc = (int64_t)(t.cyc_frac / PIX_CLOCK);
		t.cyc_frac -= (uint64_t)cyc * PIX_CLOCK;
		t.cpu_end += cyc;
		m6809_run_until(&s_cpu, t.cpu_end);
		sound_line();
	}
	for (int i = 0; i < s_out_n; i++)
		rbx_audio_put(s_out[i]);
	s_out_n = 0;
	t.frame++;
}

static int jb_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "jb_cpu", &s_cpu, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "jb", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "jb_sn", &s_sn, sn76496_state_size());
	rc |= vlm5030_state(&s_vlm, ctx, load, "jb_vlm");
	if (!rc && load)
		s_out_n = 0;
	return rc ? -1 : 0;
}

static int jb_save(void *ctx) { return jb_state(ctx, 0); }
static int jb_load(void *ctx) { return jb_state(ctx, 1); }
static int jb_supported(const char *machine) { return !strcmp(machine, "jailbrek"); }

const rbx_driver rbx_drv_jailbrek = { "Konami Jail Break", jb_init, jb_exit, jb_reset, jb_frame, jb_save, jb_load,
                                      jb_supported, jb_render, NULL };
