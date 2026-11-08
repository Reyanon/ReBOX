/*  Taito Strike Bowling, ported from MAME src/mame/taito/sbowling.cpp (license GPL-2.0+, copyright-holders:
    Jarek Burczynski, Tomasz Slanina) with compute_resistor_net_outputs from MAME src/emu/video/resnet.cpp
    (BSD-3-Clause, Couriersud).
    i8080 at 19.968 MHz / 10, 256 x 262 at 60 Hz (240 x 224 visible), RST 10h at scanline 128 and RST 08h at
    scanline 256 (HOLD_LINE). The 2-bitplane bitmap layer keeps MAME's tmpbitmap behaviour: each video RAM write
    plots 8 pixels with the colour PROM address and flip state of that moment; the background tilemap (3 bpp,
    pens 0x18-0x1f) shows where the bitmap equals the current colour PROM address (copybitmap_trans).
    Trackball: rbx_analog(TRACKBALL_X / Y) deltas (X reversed as PORT_REVERSE); without a device the d-pad rolls it.
    Sound: the AY-3-8910 (19.968 MHz / 16, route 0.33) is emulated; the board's analog sound parts are not
    emulated in MAME either (MACHINE_IMPERFECT_SOUND).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../rbx_core.h"
#include "../../cpu/i8080/i8080.h"
#include "../../sound/ay8910.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK (19968000 / 10)
#define AY_CLOCK (19968000 / 16)
#define VTOTAL 262
#define VIS_X0 8
#define VIS_Y0 32
#define VIS_W 240
#define VIS_H 224

static i8080_t s_cpu;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static const uint8_t *s_user1;
static uint8_t s_vram[0x4000], s_ram[0x400];
static uint16_t *s_tmp;      // tmpbitmap 256 x 256
static uint8_t *s_chars;     // 256 chars 8x8, 1 byte per pixel
static int s_bgmap, s_system, s_flip, s_irq_vec, s_line;
static uint32_t s_cpa;       // colour PROM address
static uint8_t s_pix_sh, s_pix[2];
static uint8_t s_track[2];   // TRACKX, TRACKY counters
static int s_port_in0, s_port_in1, s_port_dsw0, s_port_dsw1;
static uint64_t s_frac, s_ay_frac;
static ay8910_t s_ay;
static int32_t s_mix[2048];
static int s_mix_n;

typedef struct {
	uint16_t tmp[256 * 256];
	int bgmap, flip;
	uint32_t cpa;
} snap_t;
static snap_t *s_snap;

// ---- video RAM / tmpbitmap ----
// The whole bitmap is plotted again at each system_w (MAME): the same pixels when every byte since the last full plot
// was plotted with the colour address and flip of now (s_bm_ok), so that plot is left out. The full plot goes over
// 0-0x1fff once (MAME's 0x4000 offsets plot each of them twice, the same way)
static int s_bm_ok, s_bm_flip;
static uint32_t s_bm_cpa;

static void plot8(int offset)
{
	if (s_cpa != s_bm_cpa || s_flip != s_bm_flip)
		s_bm_ok = 0;
	offset &= 0x1fff;
	int y = offset / 32, x = (offset % 32) * 8;
	uint8_t v1 = s_vram[offset], v2 = s_vram[offset + 0x2000];
	for (int i = 0; i < 8; i++, x++) {
		uint16_t c = (uint16_t)(s_cpa | ((v1 & 1) * 0x20) | ((v2 & 1) * 0x40));
		int px = x, py = y;
		if (s_flip) {
			px = 255 - px;
			py = 255 - py;
		}
		s_tmp[py * 256 + px] = c;
		v1 >>= 1;
		v2 >>= 1;
	}
}

static void redraw_all(void)
{
	if (s_bm_ok && s_cpa == s_bm_cpa && s_flip == s_bm_flip)
		return;
	s_bm_cpa = s_cpa;
	s_bm_flip = s_flip;
	for (int offs = 0; offs < 0x2000; offs++)
		plot8(offs);
	s_bm_ok = 1;
}

// ---- memory / I/O ----
static uint8_t mem_r(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x3000) return a < s_rom_size ? s_rom[a] : 0xff;
	if (a >= 0x8000 && a < 0xc000) return s_vram[a - 0x8000];
	if (a == 0xf801) return ay8910_data_r(&s_ay);
	if (a >= 0xfc00) return s_ram[a - 0xfc00];
	return 0xff;
}

static void mem_w(void *u, uint16_t a, uint8_t d)
{
	(void)u;
	if (a >= 0x8000 && a < 0xc000) {
		s_vram[a - 0x8000] = d;
		plot8(a - 0x8000);
	} else if (a == 0xf800 || a == 0xf801)
		ay8910_write(&s_ay, a & 1, d);
	else if (a >= 0xfc00)
		s_ram[a - 0xfc00] = d;
}

static uint8_t io_r(void *u, uint8_t p)
{
	(void)u;
	switch (p) {
	case 0: return (uint8_t)rbx_port_read(s_port_in0);
	case 1: return (s_system & 2) ? s_track[1] : s_track[0];
	case 2: {
		int sh = s_pix_sh & 7;
		uint32_t p1 = s_pix[1], p0 = s_pix[0];
		return (uint8_t)((((p1 << (sh + 8)) | (p0 << sh)) & 0xff00) >> 8);
	}
	case 3: return (uint8_t)rbx_port_read(s_port_in1);
	case 4: return (uint8_t)rbx_port_read(s_port_dsw0);
	case 5: return (uint8_t)rbx_port_read(s_port_dsw1);
	}
	return 0xff;
}

static void io_w(void *u, uint8_t p, uint8_t d)
{
	(void)u;
	switch (p) {
	case 1: s_pix[0] = s_pix[1]; s_pix[1] = d; break;
	case 2: s_pix_sh = d; break;
	case 4:   // system_w: flip screen (bit 3), trackball axis select (bit 1)
		s_flip = (d >> 3) & 1;
		redraw_all();
		s_system = d;
		break;
	case 5:   // graph_control_w
		s_cpa = (uint32_t)(((d & 0x07) << 7) | ((d & 0xc0) >> 3));
		s_bgmap = (~d >> 4) & 3;
		break;
	}
}

static uint8_t irq_ack(void *u, int line)
{
	(void)u;
	if (line != I8080_INTR_LINE)
		return 0xff;
	i8080_set_input(&s_cpu, I8080_INTR_LINE, 0);   // HOLD_LINE
	return (uint8_t)s_irq_vec;
}

// ---- video ----
static void sb_render(void)
{
	const snap_t *s = s_snap;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const uint8_t *map = s_user1 + s->bgmap * 1024;
	for (int y = 0; y < VIS_H; y++) {
		int sy = VIS_Y0 + y;
		uint16_t *row = bm + y * pitch;
		const uint16_t *tmp = s->tmp + sy * 256;
		for (int x = 0; x < VIS_W; x++) {
			int sx = VIS_X0 + x;
			uint16_t c = tmp[sx];
			if (c == s->cpa) {
				int tx = s->flip ? 255 - sx : sx, ty = s->flip ? 255 - sy : sy;
				c = (uint16_t)(0x18 + s_chars[map[(ty >> 3) * 32 + (tx >> 3)] * 64 + (ty & 7) * 8 + (tx & 7)]);
			}
			row[x] = c;
		}
	}
}

static void vblank(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->tmp, s_tmp, sizeof(s_snap->tmp));
	s_snap->bgmap = s_bgmap;
	s_snap->flip = s_flip;
	s_snap->cpa = s_cpa;
	rbx_video_frame_done();
}

// ---- palette (compute_resistor_net_outputs, pulldown 0, pullup 100, autoscale) ----
static void net_outputs(const int *res, int count, double *o)
{
	for (int n = 0; n < (1 << count); n++) {
		double r0 = 1.0 / 1e12, r1 = 1.0 / 100;
		for (int j = 0; j < count; j++)
			if (!(n & (1 << j)))
				r0 += 1.0 / res[j];
		r0 = 1.0 / r0;
		r1 = 1.0 / r1;
		double v = 255.0 * r0 / (r1 + r0);
		o[n] = v < 0 ? 0 : v > 255 ? 255 : v;
	}
}

static void setup_palette(const uint8_t *prom)
{
	static const int rg[3] = { 470, 270, 100 }, bl[2] = { 270, 100 };
	double o_rg[8], o_b[4];
	net_outputs(rg, 3, o_rg);
	net_outputs(bl, 2, o_b);
	double mn = 255, mx = 0;
	for (int i = 0; i < 8; i++) { if (o_rg[i] < mn) mn = o_rg[i]; if (o_rg[i] > mx) mx = o_rg[i]; }
	for (int i = 0; i < 4; i++) { if (o_b[i] < mn) mn = o_b[i]; if (o_b[i] > mx) mx = o_b[i]; }
	double scale = 255.0 / (mx - mn);
	for (int i = 0; i < 8; i++) o_rg[i] = (o_rg[i] - mn) * scale;
	for (int i = 0; i < 4; i++) o_b[i] = (o_b[i] - mn) * scale;
	for (int i = 0; i < 0x400; i++) {
		uint8_t a = prom[i], b = prom[i + 0x400];
		int bb = (int)(o_b[a & 3] + 0.5);
		int g = (int)(o_rg[((a >> 2) & 1) | (((a >> 3) & 1) << 1) | ((b & 1) << 2)] + 0.5);
		int r = (int)(o_rg[((b >> 1) & 1) | (((b >> 2) & 1) << 1) | (((b >> 3) & 1) << 2)] + 0.5);
		rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | bb);
	}
}

// ---- controls ----
static void update_trackball(void)
{
	uint32_t in = rbx_input(1);
	int dx = rbx_analog(1, RBX_IPT_TRACKBALL_X), dy = rbx_analog(1, RBX_IPT_TRACKBALL_Y);
	if (!dx && !dy) {
		if (in & RBX_IN_LEFT) dx = -6;
		if (in & RBX_IN_RIGHT) dx = 6;
		if (in & RBX_IN_UP) dy = -6;
		if (in & RBX_IN_DOWN) dy = 6;
	}
	s_track[0] = (uint8_t)(s_track[0] - dx);   // PORT_REVERSE
	s_track[1] = (uint8_t)(s_track[1] + dy);
}

// ---- machine ----
static void sb_reset(void)
{
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_ram, 0, sizeof(s_ram));
	s_bgmap = s_system = s_flip = 0;
	s_cpa = 0;
	s_bm_ok = 0;   // (the bitmap as it is: not known to match)
	s_pix_sh = 0;
	s_pix[0] = s_pix[1] = 0;
	s_track[0] = s_track[1] = 0;
	s_frac = s_ay_frac = 0;
	s_mix_n = 0;
	for (int i = 0; i < 256 * 256; i++)
		s_tmp[i] = 0;
	ay8910_reset(&s_ay);
	i8080_init(&s_cpu, 0);
	i8080_reset(&s_cpu);
	i8080_set_input(&s_cpu, I8080_INTR_LINE, 0);
}

static void sb_exit(void)
{
	free(s_tmp);
	s_tmp = NULL;
	free(s_chars);
	s_chars = NULL;
	free(s_snap);
	s_snap = NULL;
}

static int sb_init(const rbx_game *g)
{
	uint32_t gsize, usize, psize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	const uint8_t *gfx = rbx_region("gfx1", &gsize);
	s_user1 = rbx_region("user1", &usize);
	const uint8_t *prom = rbx_region("proms", &psize);
	if (!s_rom || !gfx || gsize < 0x1800 || !s_user1 || usize < 0x1000 || !prom || psize < 0x800) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_port_in0 = rbx_port_find("IN0");
	s_port_in1 = rbx_port_find("IN1");
	s_port_dsw0 = rbx_port_find("DSW0");
	s_port_dsw1 = rbx_port_find("DSW1");
	s_tmp = (uint16_t *)malloc(256 * 256 * sizeof(uint16_t));
	s_chars = (uint8_t *)malloc(256 * 64);
	s_snap = (snap_t *)calloc(1, sizeof(snap_t));
	if (!s_tmp || !s_chars || !s_snap) {
		sb_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	// charlayout: 3 planes at thirds of the region (plane 0 = MSB), x = bits 0..7, 8 bytes per char
	for (int c = 0; c < 256; c++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				int o = c * 8 + y, v = 0;
				for (int p = 0; p < 3; p++)
					v = (v << 1) | ((gfx[p * 0x800 + o] >> x) & 1);
				s_chars[c * 64 + y * 8 + x] = (uint8_t)v;
			}
	if (rbx_video_setup(VIS_W, VIS_H, 0x400)) {
		sb_exit();
		return -1;
	}
	setup_palette(prom);
	ay8910_init(&s_ay, AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 84, 4, 1);   // route 0.33
	if (rbx_audio_setup(ay8910_rate(&s_ay))) {
		sb_exit();
		return -1;
	}
	memset(&s_cpu, 0, sizeof(s_cpu));
	s_cpu.read = mem_r;
	s_cpu.write = mem_w;
	s_cpu.in = io_r;
	s_cpu.out = io_w;
	s_cpu.irq_ack = irq_ack;
	for (int p = 0; p < 0x30; p++)
		s_cpu.rpage[p] = (uint32_t)(p << 8) < s_rom_size ? s_rom + (p << 8) : NULL;
	for (int p = 0x80; p < 0xc0; p++)
		s_cpu.rpage[p] = s_vram + ((p - 0x80) << 8);
	for (int p = 0xfc; p < 0x100; p++)
		s_cpu.rpage[p] = s_ram + ((p - 0xfc) << 8);
	sb_reset();
	return 0;
}

static void sb_frame(void)
{
	update_trackball();
	for (int line = 0; line < VTOTAL; line++) {
		s_line = line;
		if (line == 128 || line == 256) {
			if (line == 256)
				vblank();
			s_irq_vec = line == 256 ? 0xcf : 0xd7;   // RST 08h / RST 10h
			i8080_set_input(&s_cpu, I8080_INTR_LINE, 1);
		}
		i8080_run(&s_cpu, rbx_line_cycles(&s_frac, CPU_CLOCK, 60000, VTOTAL));
		int clocks = rbx_line_cycles(&s_ay_frac, AY_CLOCK, 60000, VTOTAL);
		s_mix_n += ay8910_render(&s_ay, (uint32_t)clocks, s_mix + s_mix_n, (int)(sizeof(s_mix) / sizeof(s_mix[0])) - s_mix_n);
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int sb_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "cpu", &s_cpu.s, sizeof(s_cpu.s));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "ay", &s_ay, ay8910_state_size());
	int32_t m[10] = { s_bgmap, s_system, s_flip, s_irq_vec, (int32_t)s_cpa, s_pix_sh, s_pix[0], s_pix[1], s_track[0],
	                  s_track[1] };
	rc |= rbx_state_io(ctx, "misc", m, sizeof(m));
	rc |= rbx_state_io(ctx, "frac", &s_frac, sizeof(s_frac));
	rc |= rbx_state_io(ctx, "ayfrac", &s_ay_frac, sizeof(s_ay_frac));
	if (rc)
		return -1;
	if (load) {
		s_bgmap = m[0]; s_system = m[1]; s_flip = m[2]; s_irq_vec = m[3]; s_cpa = (uint32_t)m[4];
		s_pix_sh = (uint8_t)m[5]; s_pix[0] = (uint8_t)m[6]; s_pix[1] = (uint8_t)m[7];
		s_track[0] = (uint8_t)m[8]; s_track[1] = (uint8_t)m[9];
		// MAME postload: rebuild the bitmap layer (with the current colour address)
		s_bm_ok = 0;
		redraw_all();
	}
	return 0;
}

static int sb_save(void *ctx) { return sb_state(ctx, 0); }
static int sb_load(void *ctx) { return sb_state(ctx, 1); }
static int sb_supported(const char *machine) { return !strcmp(machine, "sbowling"); }

const rbx_driver rbx_drv_sbowling = { "Strike Bowling", sb_init, sb_exit, sb_reset, sb_frame, sb_save, sb_load,
                                      sb_supported, sb_render };
