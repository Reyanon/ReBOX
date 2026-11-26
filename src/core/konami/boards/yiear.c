/*  Konami Yie Ar Kung-Fu. Ported from MAME src/mame/konami/yiear.cpp (BSD-3-Clause, copyright-holders: Enrique
    Sanchez, Philip Stroffolino). A 6809E at 1.536 MHz (its IRQ at vblank, NMIs every 32 lines from line 16), a 32x32
    tilemap, the sprites, 32 colours from a PROM; the sound: an SN76489A (written through a latch) and a VLM5030 rendered
    line by line on the main thread. The flip screen not done for the tilemap.
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
#define PIX_CLOCK 6144000u     // (set_raw(18.432 MHz / 3, 384, 0, 256, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_W 256
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 32

static uint8_t *s_rom, *s_vlmrom;
static uint32_t s_rom_size, s_vlmrom_size;
static uint8_t *s_chr, *s_spr;   // (a byte a pixel)
static uint32_t s_nchr, s_nspr;
static uint32_t s_rgb[32];
static m6809_t s_cpu;
static sn76496_t s_sn;
static vlm5030_t s_vlm;
static int s_port_dsw[3], s_port_p[2], s_port_system;

static struct {
	uint8_t ram[0x1000];     // 5000-5fff (5000 / 5400 the sprite RAMs, 5800 the video RAM)
	uint8_t flip, nmi_enable, irq_enable, sn_latch;
	int16_t vlm_out;
	int64_t cpu_end;         // (the CPU's line target: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, sn_frac, vlm_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t spr0[0x30], spr1[0x30], vram[0x800], flip; } s_vs;

static void capture(void)
{
	memcpy(s_vs.spr0, t.ram, sizeof(s_vs.spr0));
	memcpy(s_vs.spr1, t.ram + 0x400, sizeof(s_vs.spr1));
	memcpy(s_vs.vram, t.ram + 0x800, sizeof(s_vs.vram));
	s_vs.flip = t.flip;
}

static void yi_render(void)   // (screen_update)
{
	for (int i = 0; i < 32; i++)
		rbx_video.palette[i] = s_rgb[i];
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {   // (the tilemap: colours 16-31)
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		for (int col = 0; col < 32; col++) {
			const int offs = ((y >> 3) * 32 + col) * 2;
			const int attr = s_vs.vram[offs];
			const int code = s_vs.vram[offs + 1] | ((attr & 0x10) << 4);
			const int fx = (attr & 0x80) != 0, fy = (attr & 0x40) != 0;
			const uint8_t *src = s_chr + (rbx_wrap((uint32_t)code, s_nchr)) * 64 + (fy ? 7 - (y & 7) : (y & 7)) * 8;
			for (int i = 0; i < 8; i++)
				d[col * 8 + i] = (uint16_t)(16 + src[fx ? 7 - i : i]);
		}
	}
	for (int offs = 0x30 - 2; offs >= 0; offs -= 2) {   // (draw_sprites: colours 0-15, pen 0 transparent)
		const int attr = s_vs.spr0[offs];
		const int code = s_vs.spr1[offs + 1] + 256 * (attr & 0x01);
		const int flipx = !(attr & 0x40);
		int flipy = (attr & 0x80) != 0;
		int sy = 240 - s_vs.spr0[offs + 1];
		const int sx = s_vs.spr1[offs];
		if (s_vs.flip) {
			sy = 240 - sy;
			flipy = !flipy;
		}
		if (offs < 0x26)
			sy++;   // (MAME: the title screen and the bottom's garbage)
		const uint8_t *src = s_spr + (rbx_wrap((uint32_t)code, s_nspr)) * 256;
		for (int j = 0; j < 16; j++) {
			const int yy = sy + j;
			if (yy < VIS_Y0 || yy >= VIS_Y0 + VIS_H)
				continue;
			const uint8_t *s = src + (flipy ? 15 - j : j) * 16;
			uint16_t *d = bm + (yy - VIS_Y0) * pitch;
			for (int k = 0; k < 16; k++) {
				const int xx = sx + k;
				if (xx >= VIS_W)
					continue;
				const int p = s[flipx ? 15 - k : k];
				if (p)
					d[xx] = (uint16_t)p;
			}
		}
	}
}

// ---- the main CPU's bus (main_map) ----
static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }

static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	switch (a) {
	case 0x0000: return (uint8_t)vlm5030_bsy(&s_vlm);
	case 0x4c00: return port(s_port_dsw[1]);
	case 0x4d00: return port(s_port_dsw[2]);
	case 0x4e00: return port(s_port_system);
	case 0x4e01: return port(s_port_p[0]);
	case 0x4e02: return port(s_port_p[1]);
	case 0x4e03: return port(s_port_dsw[0]);
	}
	if (a >= 0x5000 && a < 0x6000)
		return t.ram[a - 0x5000];
	if (a >= 0x8000)
		return s_rom[rbx_wrap(a, s_rom_size)];
	return 0;
}

static void control_w(uint8_t v)   // (the interrupts cleared when their enables go off)
{
	t.flip = v & 1;
	if (t.nmi_enable && !(v & 0x02))
		m6809_set_nmi(&s_cpu, 0);
	t.nmi_enable = (v >> 1) & 1;
	if (t.irq_enable && !(v & 0x04))
		m6809_set_irq(&s_cpu, 0);
	t.irq_enable = (v >> 2) & 1;
}

static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a >= 0x5000 && a < 0x6000) {
		t.ram[a - 0x5000] = v;
		return;
	}
	switch (a) {
	case 0x4000: control_w(v); break;
	case 0x4800: t.sn_latch = v; break;                  // (konami_sn76489a_latch_w)
	case 0x4900: sn76496_write(&s_sn, t.sn_latch); break;   // (konami_sn76489a_w)
	case 0x4a00:   // (vlm5030_control_w)
		vlm5030_st(&s_vlm, (v >> 1) & 1);
		vlm5030_rst(&s_vlm, (v >> 2) & 1);
		break;
	case 0x4b00: vlm5030_data_w(&s_vlm, v); break;
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
static void yi_exit(void);
static void yi_reset(void);

static int yi_init(const rbx_game *g)
{
	uint32_t ts, ss, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_vlmrom = rbx_region("vlm", &s_vlmrom_size);
	const uint8_t *prom = rbx_region("proms", &ps);
	const uint8_t *tiles = rbx_region("tiles", &ts), *sprites = rbx_region("sprites", &ss);
	if (!s_rom || !prom || ps < 0x20 || !s_vlmrom || !tiles || !sprites) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	// (charlayout: 8x8, planes { 4, 0, half + 4, half + 0 }, x 4 pixels a byte, the right half 8 bytes on; spritelayout:
	// 16x16 the same, the 4-pixel columns 8 bytes apart, the lower half 32 bytes on)
	s_nchr = ts / 2 / 16;
	s_nspr = ss / 2 / 64;
	s_chr = (uint8_t *)malloc((size_t)s_nchr * 64);
	s_spr = (uint8_t *)malloc((size_t)s_nspr * 256);
	if (!s_chr || !s_spr) {
		rbx_fail("%s: out of memory", g->set);
		yi_exit();
		return -1;
	}
#define PIX(b1, b2, q) (uint8_t)((((b1) >> (3 - (q))) & 1) << 3 | (((b1) >> (7 - (q))) & 1) << 2 | \
                                 (((b2) >> (3 - (q))) & 1) << 1 | (((b2) >> (7 - (q))) & 1))
	for (uint32_t e = 0; e < s_nchr; e++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				const uint32_t o = e * 16 + (uint32_t)((x >> 2) * 8 + y);
				s_chr[e * 64 + (uint32_t)(y * 8 + x)] = PIX(tiles[o], tiles[ts / 2 + o], x & 3);
			}
	for (uint32_t e = 0; e < s_nspr; e++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				const uint32_t o = e * 64 + (uint32_t)((x >> 2) * 8 + (y & 7) + (y >> 3) * 32);
				s_spr[e * 256 + (uint32_t)(y * 16 + x)] = PIX(sprites[o], sprites[ss / 2 + o], x & 3);
			}
#undef PIX
	for (int i = 0; i < 32; i++) {
		const uint8_t c = prom[i];
		const int r = 0x21 * (c & 1) + 0x47 * ((c >> 1) & 1) + 0x97 * ((c >> 2) & 1);
		const int gg = 0x21 * ((c >> 3) & 1) + 0x47 * ((c >> 4) & 1) + 0x97 * ((c >> 5) & 1);
		const int b = 0x52 * ((c >> 6) & 1) + 0xad * ((c >> 7) & 1);
		s_rgb[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	sn76496_init(&s_sn, CPU_CLOCK, SN76489A_CONFIG, 256, 32, 1);
	vlm5030_init(&s_vlm, 3579545, s_vlmrom, s_vlmrom_size < 0x2000 ? s_vlmrom_size : 0x2000);
	static const char *const dn[3] = { "DSW1", "DSW2", "DSW3" };
	for (int i = 0; i < 3; i++)
		s_port_dsw[i] = rbx_port_find(dn[i]);
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_system = rbx_port_find("SYSTEM");
	if (rbx_audio_setup(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		yi_exit();
		return -1;
	}
	rbx_audio.frame_mhz = (uint32_t)((uint64_t)PIX_CLOCK * 1000 / ((uint64_t)HTOTAL * LINES));   // (60.61 Hz)
	yi_reset();
	return 0;
}

static void yi_exit(void)
{
	free(s_chr);
	free(s_spr);
	s_chr = s_spr = NULL;
}

static void yi_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	s_out_n = 0;
	sn76496_reset(&s_sn);
	vlm5030_reset(&s_vlm);
	m6809_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	for (int p = 0x50; p < 0x60; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + (p - 0x50) * 256;
	for (int p = 0x80; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + ((uint32_t)p * 256) % s_rom_size;
	m6809_reset(&s_cpu);
}

static void yi_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, vblank_irq)
			capture();
			rbx_video_frame_done();
			if (t.irq_enable)
				m6809_set_irq(&s_cpu, 1);
		}
		if (vpos >= 16 && (vpos - 16) % 32 == 0 && vpos < LINES && t.nmi_enable)   // (the 16V timer: the music's tempo)
			m6809_set_nmi(&s_cpu, 1);
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

static int yi_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "yi_cpu", &s_cpu, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "yi", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "yi_sn", &s_sn, sn76496_state_size());
	rc |= vlm5030_state(&s_vlm, ctx, load, "yi_vlm");
	if (!rc && load)
		s_out_n = 0;
	return rc ? -1 : 0;
}

static int yi_save(void *ctx) { return yi_state(ctx, 0); }
static int yi_load(void *ctx) { return yi_state(ctx, 1); }
static int yi_supported(const char *machine) { return !strcmp(machine, "yiear"); }

const rbx_driver rbx_drv_yiear = { "Konami Yie Ar Kung-Fu", yi_init, yi_exit, yi_reset, yi_frame, yi_save, yi_load,
                                   yi_supported, yi_render, NULL };
