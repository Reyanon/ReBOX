/*  Konami Kicker / Shao-lin's Road. Ported from MAME src/mame/konami/shaolins.cpp (BSD-3-Clause, copyright-holders:
    Allard van der Bas) and the palette's resistor network of src/emu/video/resnet.cpp (BSD-3-Clause,
    copyright-holders: Couriersud). A 6809E at 1.536 MHz (its IRQ at line 240, NMIs every 32 lines), a 32x32 tilemap
    with column scroll, 24 sprites (their transparent pens by the palette bank), the colours through the PROMs; the
    sound: two SN76489As rendered line by line on the main thread. The flip screen not done for the tilemap.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../sound/sn76496.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 1536000u     // (18.432 MHz / 12; the first SN76489A, the second at twice that)
#define SND_RATE 48000
#define PIX_CLOCK 6144000u     // (set_raw(18.432 MHz / 3, 384, 0, 256, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_W 256
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 0x1000           // (16 * 8 * 16 * 2 pens over 256 colours)

static uint8_t *s_rom, *s_prom;
static uint32_t s_rom_size;
static uint8_t *s_chr, *s_spr;   // (a byte a pixel)
static uint32_t s_nchr, s_nspr;
static uint32_t s_rgb[256];
static m6809_t s_cpu;
static sn76496_t s_sn[2];
static int s_port_dsw[3], s_port_p[2], s_port_system;
static int s_vpos;

static struct {
	uint8_t ram[0x1800];     // 2800-3fff (2800 / 3000 the sprite RAMs, 3800 the colour RAM, 3c00 the video RAM)
	uint8_t palettebank, scroll, nmi_enable;
	int64_t cpu_end;         // (the CPU's line target: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, sn_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t spr0[0x30], spr1[0x30], cram[0x400], vram[0x400], palettebank, scroll; } s_vs;

static void capture(void)
{
	memcpy(s_vs.spr0, t.ram, sizeof(s_vs.spr0));
	memcpy(s_vs.spr1, t.ram + 0x800, sizeof(s_vs.spr1));
	memcpy(s_vs.cram, t.ram + 0x1000, sizeof(s_vs.cram));
	memcpy(s_vs.vram, t.ram + 0x1400, sizeof(s_vs.vram));
	s_vs.palettebank = t.palettebank;
	s_vs.scroll = t.scroll;
}

static inline uint8_t ctab(int q)   // (the pen's colour: pen ((i & 0x100) << 3) | (j << 8) | (i & 0xff))
{
	const int i = (q & 0xff) | (((q >> 11) & 1) << 8), j = (q >> 8) & 7;
	return (uint8_t)((j << 5) | ((~i & 0x100) >> 4) | (s_prom[0x300 + i] & 0x0f));
}

static void sh_render(void)   // (screen_update)
{
	for (int q = 0; q < NPENS; q++)
		rbx_video.palette[q] = s_rgb[ctab(q)];
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {   // (the tilemap: columns 4-31 scrolled by scroll + 1)
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		for (int col = 0; col < 32; col++) {
			const int ty = (y + (col >= 4 ? s_vs.scroll + 1 : 0)) & 255;
			const int idx = (ty >> 3) * 32 + col;
			const int attr = s_vs.cram[idx];
			const int code = s_vs.vram[idx] + ((attr & 0x40) << 2);
			const int pen = ((attr & 0x0f) + 16 * s_vs.palettebank) * 16;
			const int fy = (attr & 0x20) != 0;
			const uint8_t *src = s_chr + ((uint32_t)code % s_nchr) * 64 + (fy ? 7 - (ty & 7) : (ty & 7)) * 8;
			for (int i = 0; i < 8; i++)
				d[col * 8 + i] = (uint16_t)(pen + src[i]);
		}
	}
	const int tcol = s_vs.palettebank << 5;
	for (int offs = 23 * 2; offs >= 0; offs -= 2) {   // (draw_sprites: transmask of the pens whose colour is bank << 5)
		const int code = s_vs.spr1[offs + 1];
		const int pen = 16 * 8 * 16 + ((s_vs.spr0[offs] & 0x0f) | (s_vs.palettebank << 4)) * 16;
		const int flipx = !(s_vs.spr0[offs] & 0x40), flipy = (s_vs.spr0[offs] & 0x80) != 0;
		const int sx = s_vs.spr1[offs], sy = 241 - s_vs.spr0[offs + 1];
		uint8_t trans[16];
		for (int p = 0; p < 16; p++)
			trans[p] = ctab(pen + p) == tcol;
		const uint8_t *src = s_spr + ((uint32_t)code % s_nspr) * 256;
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
				if (!trans[p])
					d[xx] = (uint16_t)(pen + p);
			}
		}
	}
}

// ---- the main CPU's bus (prg_map) ----
static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }

static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a >= 0x0200 && a < 0x0300)
		return (uint8_t)s_vpos;
	switch (a) {
	case 0x0500: return port(s_port_dsw[1]);
	case 0x0600: return port(s_port_dsw[2]);
	case 0x0700: return port(s_port_system);
	case 0x0701: return port(s_port_p[0]);
	case 0x0702: return port(s_port_p[1]);
	case 0x0703: return port(s_port_dsw[0]);
	}
	if (a >= 0x2800 && a < 0x4000)
		return t.ram[a - 0x2800];
	if (a >= 0x4000)
		return s_rom[a % s_rom_size];
	return 0;
}

static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a >= 0x2800 && a < 0x4000) {
		t.ram[a - 0x2800] = v;
		return;
	}
	switch (a) {
	case 0x0000: t.nmi_enable = v; break;   // (nmi_w: bit 0 the flip screen, bit 1 the NMI's enable)
	case 0x0300: sn76496_write(&s_sn[0], v); break;
	case 0x0400: sn76496_write(&s_sn[1], v); break;
	case 0x1800: t.palettebank = v & 0x07; break;
	case 0x2000: t.scroll = v; break;
	}
}

// ---- the sound: a line's samples (the SN76489As 1.0 each) ----
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
	int32_t a[64], b[64];
	memset(a, 0, sizeof(a));
	memset(b, 0, sizeof(b));
	t.sn_frac += (uint64_t)CPU_CLOCK * (uint64_t)n;
	const uint32_t sc = (uint32_t)(t.sn_frac / SND_RATE);
	t.sn_frac -= (uint64_t)sc * SND_RATE;
	const int ka = sn76496_render(&s_sn[0], sc, a, 64), kb = sn76496_render(&s_sn[1], 2 * sc, b, 64);
	for (int i = 0; i < n && s_out_n < 1024; i++)
		s_out[s_out_n++] = (ka > 0 ? a[i * ka / n] : 0) + (kb > 0 ? b[i * kb / n] : 0);
}

// ---- init / frame ----
static void sh_exit(void);
static void sh_reset(void);

// MAME compute_resistor_weights(0, 255, -1.0, ...) for three like networks: each resistor's output alone, scaled so
// that all of them on gives 255
static void resistor_weights(const int *res, int n, int pulldown, double *w)
{
	double sum = 0;
	for (int k = 0; k < n; k++) {
		double r0 = pulldown ? 1.0 / pulldown : 1.0 / 1e12, r1 = 1.0 / 1e12;
		for (int j = 0; j < n; j++) {
			if (j == k)
				r1 += 1.0 / res[j];
			else
				r0 += 1.0 / res[j];
		}
		r0 = 1.0 / r0;
		r1 = 1.0 / r1;
		w[k] = 255.0 * r0 / (r1 + r0);
		sum += w[k];
	}
	for (int k = 0; k < n; k++)
		w[k] *= 255.0 / sum;
}

static int sh_init(const rbx_game *g)
{
	uint32_t ts, ss, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_prom = rbx_region("proms", &ps);
	const uint8_t *tiles = rbx_region("tiles", &ts), *sprites = rbx_region("sprites", &ss);
	if (!s_rom || !s_prom || ps < 0x500 || !tiles || ts < 0x4000 || !sprites || ss < 0x8000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	// (charlayout: 512 of 8x8, planes { 0x2000 + 4, 0x2000 + 0, 4, 0 } in bytes, x 4 pixels a byte, the right half 8
	// bytes on; spritelayout: 256 of 16x16, planes 0x4000 apart, the 4-pixel columns 8 bytes apart, the lower half 32 on)
	s_nchr = 512;
	s_nspr = 256;
	s_chr = (uint8_t *)malloc((size_t)s_nchr * 64);
	s_spr = (uint8_t *)malloc((size_t)s_nspr * 256);
	if (!s_chr || !s_spr) {
		rbx_fail("%s: out of memory", g->set);
		sh_exit();
		return -1;
	}
#define PIX(hi, lo, q) (uint8_t)((((hi) >> (3 - (q))) & 1) << 3 | (((hi) >> (7 - (q))) & 1) << 2 | \
                                 (((lo) >> (3 - (q))) & 1) << 1 | (((lo) >> (7 - (q))) & 1))
	for (uint32_t e = 0; e < s_nchr; e++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				const uint32_t o = e * 16 + (uint32_t)((x >> 2) * 8 + y);
				s_chr[e * 64 + (uint32_t)(y * 8 + x)] = PIX(tiles[0x2000 + o], tiles[o], x & 3);
			}
	for (uint32_t e = 0; e < s_nspr; e++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				const uint32_t o = e * 64 + (uint32_t)((x >> 2) * 8 + (y & 7) + (y >> 3) * 32);
				s_spr[e * 256 + (uint32_t)(y * 16 + x)] = PIX(sprites[0x4000 + o], sprites[o], x & 3);
			}
#undef PIX
	static const int res[4] = { 2200, 1000, 470, 220 };
	double w[4];
	resistor_weights(res, 4, 470, w);
	for (int i = 0; i < 256; i++) {
		int c[3];
		for (int k = 0; k < 3; k++) {
			const uint8_t v = s_prom[i | (k << 8)];
			c[k] = (int)(w[0] * (v & 1) + w[1] * ((v >> 1) & 1) + w[2] * ((v >> 2) & 1) + w[3] * ((v >> 3) & 1) + 0.5);
		}
		s_rgb[i] = (uint32_t)(c[0] << 16 | c[1] << 8 | c[2]);
	}
	sn76496_init(&s_sn[0], CPU_CLOCK, SN76489A_CONFIG, 256, 32, 1);
	sn76496_init(&s_sn[1], 2 * CPU_CLOCK, SN76489A_CONFIG, 256, 32, 1);
	static const char *const dn[3] = { "DSW1", "DSW2", "DSW3" };
	for (int i = 0; i < 3; i++)
		s_port_dsw[i] = rbx_port_find(dn[i]);
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_system = rbx_port_find("SYSTEM");
	if (rbx_audio_setup(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		sh_exit();
		return -1;
	}
	rbx_audio.frame_mhz = (uint32_t)((uint64_t)PIX_CLOCK * 1000 / ((uint64_t)HTOTAL * LINES));   // (60.61 Hz)
	sh_reset();
	return 0;
}

static void sh_exit(void)
{
	free(s_chr);
	free(s_spr);
	s_chr = s_spr = NULL;
}

static void sh_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	s_out_n = 0;
	sn76496_reset(&s_sn[0]);
	sn76496_reset(&s_sn[1]);
	m6809_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	for (int p = 0x28; p < 0x40; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + (p - 0x28) * 256;
	for (int p = 0x40; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + ((uint32_t)p * 256) % s_rom_size;
	m6809_reset(&s_cpu);
}

static void sh_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		s_vpos = vpos;
		if (vpos == VIS_Y0 + VIS_H) {
			capture();
			rbx_video_frame_done();
			m6809_hold_irq(&s_cpu);   // (interrupt: line 240)
		}
		if ((vpos & 0x1f) == 0x10 && (t.nmi_enable & 0x02)) {   // (pulse_input_line)
			m6809_set_nmi(&s_cpu, 1);
			m6809_set_nmi(&s_cpu, 0);
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

static int sh_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "sh_cpu", &s_cpu, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "sh", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "sh_sn0", &s_sn[0], sn76496_state_size());
	rc |= rbx_state_io(ctx, "sh_sn1", &s_sn[1], sn76496_state_size());
	if (!rc && load)
		s_out_n = 0;
	return rc ? -1 : 0;
}

static int sh_save(void *ctx) { return sh_state(ctx, 0); }
static int sh_load(void *ctx) { return sh_state(ctx, 1); }
static int sh_supported(const char *machine) { return !strcmp(machine, "shaolins"); }

const rbx_driver rbx_drv_shaolins = { "Konami Kicker", sh_init, sh_exit, sh_reset, sh_frame, sh_save, sh_load,
                                      sh_supported, sh_render, NULL };
