/*  Konami Fast Lane. Ported from MAME src/mame/konami/fastlane.cpp (BSD-3-Clause, copyright-holders: Manuel Abadia)
    and the devices it uses (k007121.cpp, k051733.cpp, k007232.cpp). An HD6309E at 3 MHz, one K007121 (its two 32x32
    tilemaps as the driver's), a K051733, the indirect palette through the colour PROM; the sound: two K007232s on the
    main CPU (no sound CPU), made on this thread.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/hd6309/hd6309.h"
#include "../../sound/k007232.h"
#include "../machine/k051733.h"
#include "../video/k007121.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 3000000u     // (24 MHz / 8)
#define CHIP_CLOCK 3579545u    // (the K007232s)
#define SND_RATE 48000
#define PIX_CLOCK 6000000u     // (set_raw(24 MHz / 4, 384, 0, 280, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_X0 0
#define VIS_W 280
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 16384            // (1024 * 16)

static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t *s_gfx;
static uint32_t s_gfx_count;
static uint16_t (*s_ctab_hp)[NPENS];
#define s_ctab (*s_ctab_hp)
static uint16_t s_transmask[16];
static hd6309_t s_cpu;
static k007121_t s_k121;
static k051733_t s_k733;
static k007232_t s_k232[2];
static int s_port_system, s_port_p[2], s_port_dsw[3];
static int32_t s_out[2048];
static int s_out_pos;

static struct {
	uint8_t pal[0x800];      // 1000-17ff
	uint8_t ram[0x800];      // 1800-1fff
	uint8_t vram[2][0x800];  // 2000-27ff, 2800-2fff
	uint8_t spr[0x1000];     // 3000-3fff
	uint8_t mainbank;
	uint64_t k232_frac;
	int32_t k232_out[2][2];
	int64_t cycles_done;
	int64_t cpu_end, snd_end;   // (the CPUs' line targets: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t pal[0x800], vram[2][0x800]; k007121_t k121; } s_vs;

static void capture(void)
{
	memcpy(s_vs.pal, t.pal, sizeof(s_vs.pal));
	memcpy(s_vs.vram, t.vram, sizeof(s_vs.vram));
	memcpy(&s_vs.k121, &s_k121, sizeof(k007121_t));
}

// a tilemap (get_tile_info) between the screen's x0 and x1
static void draw_layer(int which, int scrollx, int scrolly, int opaque, int x0, int x1)
{
	const uint8_t *v = s_vs.vram[which], *k = s_vs.k121.ctrl;
	const int bit0 = k[5] & 3, bit1 = (k[5] >> 2) & 3, bit2 = (k[5] >> 4) & 3, bit3 = (k[5] >> 6) & 3;
	const int mask = (k[4] & 0xf0) >> 4;
	if (x0 < VIS_X0)
		x0 = VIS_X0;
	if (x1 > VIS_X0 + VIS_W - 1)
		x1 = VIS_X0 + VIS_W - 1;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		const int ty = (y + scrolly) & 255;
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch - VIS_X0;
		for (int x = x0; x <= x1;) {
			const int tx = (x + scrollx) & 255;
			int n = 8 - (tx & 7);
			if (n > x1 - x + 1)
				n = x1 - x + 1;
			const int idx = (ty >> 3) * 32 + (tx >> 3), attr = v[idx];
			int bank = ((attr >> (bit0 + 3)) & 1) | ((attr >> (bit1 + 2)) & 2) | ((attr >> (bit2 + 1)) & 4) | ((attr >> bit3) & 8);
			bank = (bank & ~mask) | (k[4] & mask);
			bank = ((attr & 0x80) >> 7) | (bank << 1) | ((k[3] & 1) << 5);
			const uint32_t code = rbx_wrap((uint32_t)(v[idx + 0x400] + bank * 256), s_gfx_count);
			const uint8_t *src = s_gfx + code * 64 + (ty & 7) * 8 + (tx & 7);
			const int pb = (1 + 64 * (attr & 0x0f)) * 16;
			for (int i = 0; i < n; i++) {
				const int p = src[i];
				if (p || opaque)
					d[x + i] = (uint16_t)(pb + p);
			}
			x += n;
		}
	}
}

static void fl_render(void)   // (screen_update)
{
	for (int i = 0; i < NPENS; i++) {   // (the indirect palette: xBGR_555, the high byte first)
		const int e = s_ctab[i];
		const uint16_t v = (uint16_t)(s_vs.pal[2 * e] << 8 | s_vs.pal[2 * e + 1]);
		const int r = v & 31, g = (v >> 5) & 31, b = (v >> 10) & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (b << 3 | b >> 2));
	}
	const k007121_t *k = &s_vs.k121;
	for (int y = 0; y < VIS_H; y++)
		for (int x = 0; x < VIS_W; x++)
			rbx_video.bitmap[y * rbx_video.pitch + x] = 0x10;
	const int vx0 = VIS_X0, vx1 = VIS_X0 + VIS_W - 1;
	int c0x0 = vx0, c0x1 = vx1, c1x0 = vx0, c1x1 = vx1;   // (the playfield beside the 40 columns of the second)
	if (k->flipscreen) {
		c0x1 -= 40;
		c1x0 = c1x1 - 39;
	} else {
		c0x0 += 40;
		c1x1 = 39;
	}
	draw_layer(0, k->ctrl[0] - 40, k->ctrl[2], 1, c0x0, c0x1);
	k007121_sprites_draw(k, s_gfx, s_gfx_count, 0, s_transmask, NULL, rbx_video.bitmap, rbx_video.pitch, VIS_X0, VIS_Y0,
	                     c0x0, VIS_Y0, c0x1, VIS_Y0 + VIS_H - 1, NULL, 0);
	draw_layer(1, 0, 0, 0, c1x0, c1x1);
}

// ---- the main CPU's bus (prg_map) ----
static void map_bank(void)
{
	for (int p = 0; p < 0x40; p++)
		s_cpu.rpage[0x40 + p] = s_rom + rbx_wrap((0x8000u + (uint32_t)t.mainbank * 0x4000 + (uint32_t)p * 256), s_rom_size);
}

static void cpu_lines(int set, int cleared)   // (the K007121's IRQ and NMI)
{
	if (cleared & K007121_IRQ)
		hd6309_set_irq(&s_cpu, 0);
	if (set & K007121_IRQ)
		hd6309_set_irq(&s_cpu, 1);
	if (cleared & K007121_NMI)
		hd6309_set_nmi(&s_cpu, 0);
	if (set & K007121_NMI)
		hd6309_set_nmi(&s_cpu, 1);
}

static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x1000) {
		if (a >= 0x0020 && a < 0x0060)
			return s_k121.scroll[a - 0x0020];
		if (a >= 0x0d00 && a < 0x0d0e)   // (k007232_r: offset ^ 1)
			return k007232_read(&s_k232[0], (a - 0x0d00) ^ 1);
		if (a >= 0x0e00 && a < 0x0e0e)
			return k007232_read(&s_k232[1], (a - 0x0e00) ^ 1);
		if (a >= 0x0f00 && a < 0x0f20)
			return k051733_read(&s_k733, a - 0x0f00);
		switch (a) {
		case 0x0800: return (uint8_t)rbx_port_read(s_port_dsw[2]);
		case 0x0801: return (uint8_t)rbx_port_read(s_port_p[1]);
		case 0x0802: return (uint8_t)rbx_port_read(s_port_p[0]);
		case 0x0803: return (uint8_t)rbx_port_read(s_port_system);
		case 0x0900: return (uint8_t)rbx_port_read(s_port_dsw[0]);
		case 0x0901: return (uint8_t)rbx_port_read(s_port_dsw[1]);
		}
		return 0;
	}
	if (a < 0x1800)
		return t.pal[a - 0x1000];
	if (a < 0x2000)
		return t.ram[a - 0x1800];
	if (a < 0x3000)
		return t.vram[(a >> 11) & 1][a & 0x7ff];
	if (a < 0x4000)
		return t.spr[a - 0x3000];
	if (a < 0x8000)
		return s_rom[rbx_wrap((0x8000u + (uint32_t)t.mainbank * 0x4000 + (a - 0x4000)), s_rom_size)];
	return s_rom[a - 0x8000];
}

static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x1000) {
		if (a < 0x0008)
			cpu_lines(0, k007121_ctrl_w(&s_k121, a, v));
		else if (a >= 0x0020 && a < 0x0060)
			s_k121.scroll[a - 0x0020] = v;
		else if (a >= 0x0d00 && a < 0x0d0e)
			k007232_write(&s_k232[0], (a - 0x0d00) ^ 1, v);
		else if (a >= 0x0e00 && a < 0x0e0e)
			k007232_write(&s_k232[1], (a - 0x0e00) ^ 1, v);
		else if (a >= 0x0f00 && a < 0x0f20)
			k051733_write(&s_k733, a - 0x0f00, v);
		else if (a == 0x0c00) {   // (bankswitch_w: the ROM's bank, the K007232 #1's banks)
			t.mainbank = (v & 0x0c) >> 2;
			k007232_set_bank(&s_k232[1], (v & 0x10) >> 4, 2 + ((v & 0x10) >> 4));
			map_bank();
		}
		return;
	}
	if (a < 0x1800)
		t.pal[a - 0x1000] = v;
	else if (a < 0x2000)
		t.ram[a - 0x1800] = v;
	else if (a < 0x3000)
		t.vram[(a >> 11) & 1][a & 0x7ff] = v;
	else if (a < 0x4000)
		t.spr[a - 0x3000] = v;
}

// ---- the sound: the two K007232s, a line's samples (0.50 each channel; held between theirs) ----
static void k007232_port0(uint8_t data)   // (volume_callback)
{
	k007232_set_volume(&s_k232[0], 0, (data >> 4) * 0x11, 0);
	k007232_set_volume(&s_k232[0], 1, 0, (data & 0x0f) * 0x11);
}
static void k007232_port1(uint8_t data)
{
	k007232_set_volume(&s_k232[1], 0, (data >> 4) * 0x11, 0);
	k007232_set_volume(&s_k232[1], 1, 0, (data & 0x0f) * 0x11);
}

static void sound_line(int n)
{
	for (int i = 0; i < n && s_out_pos < 2048; i++, s_out_pos++) {
		t.k232_frac += CHIP_CLOCK / 128;
		while (t.k232_frac >= SND_RATE) {
			t.k232_frac -= SND_RATE;
			for (int k = 0; k < 2; k++)
				k007232_sample(&s_k232[k], &t.k232_out[k][0], &t.k232_out[k][1]);
		}
		s_out[s_out_pos] = (t.k232_out[0][0] + t.k232_out[0][1] + t.k232_out[1][0] + t.k232_out[1][1]) / 2;
	}
}

// ---- init / frame ----
static void fl_exit(void);
static void fl_reset(void);

static int fl_init(const rbx_game *g)
{
	if (!s_ctab_hp && !(s_ctab_hp = calloc(1, sizeof(*s_ctab_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t gsize, psize, p0size, p1size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	const uint8_t *gfx = rbx_region("gfx", &gsize), *prom = rbx_region("proms", &psize);
	const uint8_t *pcm0 = rbx_region("k007232_1", &p0size), *pcm1 = rbx_region("k007232_2", &p1size);
	if (!s_rom || !gfx || !prom || psize < 0x100 || !pcm0 || !pcm1) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	s_gfx_count = gsize / 32;   // (gfx_8x8x4_packed_msb)
	s_gfx = (uint8_t *)malloc((size_t)s_gfx_count * 64);
	if (!s_gfx) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	for (uint32_t n = 0; n < s_gfx_count; n++)
		for (int b = 0; b < 32; b++) {
			s_gfx[n * 64 + b * 2] = gfx[n * 32 + b] >> 4;
			s_gfx[n * 64 + b * 2 + 1] = gfx[n * 32 + b] & 15;
		}
	for (int pal = 0; pal < 0x10; pal++)   // (palette: the PROM's 16 entries for each bank)
		for (int i = 0; i < 0x400; i++)
			s_ctab[(pal << 10) | i] = (uint16_t)((i & 0x3f0) | prom[(pal << 4) | (i & 0x0f)]);
	for (int c = 0; c < 16; c++) {   // (transpen_mask(gfx, color, 0): the sprites' colours)
		uint16_t m = 0;
		for (int p = 0; p < 16; p++)
			if (s_ctab[c * 16 + p] == 0)
				m |= (uint16_t)(1u << p);
		s_transmask[c] = m;
	}
	k007121_init(&s_k121, 40, 16);   // (set_sprite_offsets(40, 16))
	k007232_init(&s_k232[0], pcm0, p0size, k007232_port0);
	k007232_init(&s_k232[1], pcm1, p1size, k007232_port1);
	s_port_system = rbx_port_find("SYSTEM");
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	static const char *const dn[3] = { "DSW1", "DSW2", "DSW3" };
	for (int i = 0; i < 3; i++)
		s_port_dsw[i] = rbx_port_find(dn[i]);
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		fl_exit();
		return -1;
	}
	fl_reset();
	return 0;
}

static void fl_exit(void)
{
	free(s_gfx);
	s_gfx = NULL;
	free(s_ctab_hp);
	s_ctab_hp = NULL;
}

static void fl_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	s_out_pos = 0;
	k007121_reset(&s_k121);
	memset(s_k121.scroll, 0, sizeof(s_k121.scroll));
	k051733_reset(&s_k733);
	k007232_reset(&s_k232[0]);
	k007232_reset(&s_k232[1]);
	hd6309_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	for (int p = 0x80; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + (uint32_t)(p - 0x80) * 256;
	for (int p = 0x18; p < 0x20; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + (p - 0x18) * 256;
	for (int p = 0x30; p < 0x40; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.spr + (p - 0x30) * 256;
	map_bank();
	hd6309_reset(&s_cpu);
}

static void fl_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the screen's update)
			capture();
			rbx_video_frame_done();
		}
		if (!(vpos & 15))   // (the K007121's scanline timer: every 16 lines)
			cpu_lines(k007121_scanline(&s_k121, vpos, t.frame, t.spr), 0);
		t.cyc_frac += (uint64_t)CPU_CLOCK * HTOTAL;
		const int64_t cyc = (int64_t)(t.cyc_frac / PIX_CLOCK);
		t.cyc_frac -= (uint64_t)cyc * PIX_CLOCK;
		hd6309_run_until(&s_cpu, t.cpu_end += cyc);
		t.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(t.snd_frac / PIX_CLOCK);
		t.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		if (ns > 0)
			sound_line(ns);
	}
	for (int i = 0; i < s_out_pos; i++)
		rbx_audio_put_stereo(s_out[i], s_out[i]);
	s_out_pos = 0;
	t.frame++;
}

static int fl_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "fl_cpu", &s_cpu, (uint32_t)offsetof(hd6309_t, rpage));
	rc |= rbx_state_io(ctx, "fl", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "fl_k121", &s_k121, (uint32_t)offsetof(k007121_t, state_end));
	rc |= rbx_state_io(ctx, "fl_k733", &s_k733, sizeof(s_k733));
	rc |= k007232_state(&s_k232[0], ctx, load, "fl_k232a");
	rc |= k007232_state(&s_k232[1], ctx, load, "fl_k232b");
	if (!rc && load)
		map_bank();
	return rc ? -1 : 0;
}

static int fl_save(void *ctx) { return fl_state(ctx, 0); }
static int fl_load(void *ctx) { return fl_state(ctx, 1); }
static int fl_supported(const char *machine) { return !strcmp(machine, "fastlane"); }

const rbx_driver rbx_drv_fastlane = { "Konami Fast Lane", fl_init, fl_exit, fl_reset, fl_frame, fl_save, fl_load,
                                      fl_supported, fl_render, NULL };
