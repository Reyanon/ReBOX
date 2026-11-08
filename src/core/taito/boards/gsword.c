/*  Great Swordsman hardware, ported from MAME src/mame/taito/gsword.cpp, gsword.h and gsword_v.cpp
    (BSD-3-Clause, copyright-holders: Steve Ellenoff, Jarek Parchanski, Vas Crabb).
    Three Z80s (main, sub, ADPCM) at 3 MHz and three I8041A MCUs at 6 MHz, all running their real ROMs as in MAME
    (AA-013 is MAME's AA-016 stand-in): MCU1 (main) and MCU2 (sub) talk over a serial link clocked by a 5859.375 Hz
    clock on T0 (run in instruction lockstep, MAME's perfect quantum), MCU3 reads the controls and coins.
    The sub CPU's timeout counter hack at 0x4004 (init_gsword / init_gsword2) and the AY port A NMI control follow MAME.
    Sound: two AY-3-8910 at 1.5 MHz (legacy output, route 0.30 each) and an MSM5205 in VCK slave mode (SEX_4B,
    driven by the ADPCM CPU, route 0.60), mixed in mono at the AY rate, synchronised to the writing CPU, through a DC blocker.
    Joshi Volleyball (josvolly) is not supported: MAME marks it not working (incomplete MCU hookup, unknown sub CPU
    interrupt circuit).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/mcs48/mcs48.h"
#include "../../sound/ay8910.h"
#include "../../sound/msm5205.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// MAME: Z80s at 18 MHz / 6, MCUs at 12 MHz / 2 (400000 machine cycles a second), 60 Hz, 256 lines, visible 16-239
#define CLOCK 3000000
#define MCU_CYCLES 400000
#define LINES 256
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256

typedef struct {
	z80 z;
	int irq;
	unsigned long start;
	int len;
} zcpu;

static uint8_t *s_main_rom, *s_sub_rom, *s_audio_rom;
static uint8_t s_ram[0x2000];          // main 0x9000-0x9fff (0x0000-0x0fff here) and 0xa000-0xa7ff (0x1000-0x17ff)
static uint8_t s_attr[0x80];           // 0xab80-0xabff sprite attributes (write only)
static uint8_t s_vram[0x800];          // 0xb000-0xb7ff
static uint8_t s_sub_ram[0x400];       // sub 0x4000-0x43ff
static zcpu s_main, s_sub, s_audio;
static mcs48 s_mcu[3];
static uint64_t s_mt[3];               // MCU local time (machine cycles)
static uint64_t s_mcu_target;
static uint8_t s_mcu1_p1, s_mcu2_p1;
static uint8_t s_fake[2], s_latch;
static int s_protect_hack, s_nmi_enable, s_vclk;
static int s_charbank, s_charpalbank, s_flip, s_scroll;
// sound
static ay8910_t s_ay[2];
static msm5205_t s_msm;
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static uint32_t s_done;                 // Z80 clocks of the frame mixed so far
static int s_line_pos;
static int32_t s_dc_x, s_dc_y;         // DC blocker state (x256)
// video
static uint8_t *s_chr, *s_spr16, *s_spr32;
static uint8_t s_chr_opaque[1024];
static uint32_t s_colors[256];
static uint8_t s_ctab[256];
static uint8_t s_snap_ram[0x800], s_snap_attr[0x80], s_snap_vram[0x800];
static int s_snap_charbank, s_snap_palbank, s_snap_flip, s_snap_scroll;
// ports
static int s_port_mcu1p1, s_in0, s_in1, s_coins, s_dsw0, s_dsw1, s_dsw2;
static uint64_t s_frac, s_mfrac;

static uint8_t port8(int p) { return (uint8_t)rbx_port_read(p); }

// ---- sound ----
static void mix_to(uint32_t pos)
{
	if (pos <= s_done)
		return;
	uint32_t clocks = pos / 2 - s_done / 2;   // AYs at 1.5 MHz
	int n = ay8910_render(&s_ay[0], clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	ay8910_render(&s_ay[1], clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	int32_t adpcm = msm5205_output(&s_msm) * 154 / 256;   // route 0.60 (output full scale = MAME 0.5)
	for (int i = 0; i < n; i++)
		s_mix[s_mix_n + i] += adpcm;
	s_mix_n += n;
	s_done = pos;
}

static void sync_cpu(zcpu *c)
{
	int done = (int)(c->z.cyc - c->start);
	if (done > c->len) done = c->len;
	if (done < 0) done = 0;
	mix_to((uint32_t)(s_line_pos + done));
}

static void nmi_set_w(void *u, uint8_t data)
{
	(void)u;
	s_protect_hack = (data & 0x80) ? 0 : 1;
	switch (data) {
	case 0xff: s_nmi_enable = 0; break;
	case 0x02: s_nmi_enable = 0; break;
	case 0x0d: s_nmi_enable = 1; break;
	case 0x0f: s_nmi_enable = 1; break;
	case 0xfe: s_nmi_enable = 1; break;
	}
}

// ---- MCUs ----
static int tclk(int i) { return (int)((s_mt[i] * 15 / 512) & 1); }   // 12 MHz / 8 / 128 / 2 square wave

static uint8_t mcu_port_r(void *user, int port)
{
	int i = (int)(intptr_t)user;
	switch (i) {
	case 0: return port == 1 ? (uint8_t)(port8(s_port_mcu1p1) | (s_mcu2_p1 & 1)) : port8(s_dsw2);
	case 1: return port == 1 ? (uint8_t)((port8(s_dsw0) & 0xfc) | 0x02 | (s_mcu1_p1 & 1)) : port8(s_dsw1);
	default: return port == 1 ? port8(s_in0) : port8(s_in1);
	}
}
static void mcu_port_w(void *user, int port, uint8_t v)
{
	int i = (int)(intptr_t)user;
	if (port != 1) return;   // MCU3 P2: coin counters
	if (i == 0) s_mcu1_p1 = v;
	else if (i == 1) s_mcu2_p1 = v;
}
static int mcu_test_r(void *user, int t)
{
	int i = (int)(intptr_t)user;
	switch (i) {
	case 0: return t == 0 ? tclk(0) : (s_mcu2_p1 >> 1) & 1;
	case 1: return t == 0 ? tclk(1) : (s_mcu1_p1 >> 1) & 1;
	default: return (port8(s_coins) >> t) & 1;
	}
}

static void mcu_run_to(uint64_t target)
{
	// MCU1 and MCU2 exchange serial data: one instruction at a time, the one behind first
	while (s_mt[0] < target || s_mt[1] < target) {
		int i = s_mt[0] <= s_mt[1] ? 0 : 1;
		s_mt[i] += (uint64_t)mcs48_execute(&s_mcu[i], 1);
	}
	if (s_mt[2] < target)
		s_mt[2] += (uint64_t)mcs48_execute(&s_mcu[2], (int)(target - s_mt[2]));
}

// ---- main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x9000) return s_main_rom[a];
	if (a < 0xa800) return s_ram[a - 0x9000];
	if (a >= 0xb000 && a < 0xb800) return s_vram[a - 0xb000];
	return 0;
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x9000) return;
	if (a < 0xa800) { s_ram[a - 0x9000] = v; return; }
	if (a == 0xa980) { s_charbank = v; return; }
	if (a == 0xaa80) { s_charpalbank = (v & 0x60) >> 5; s_flip = v & 0x10; return; }
	if (a == 0xab00) { s_scroll = v; return; }
	if (a >= 0xab80 && a < 0xac00) { s_attr[a - 0xab80] = v; return; }
	if (a >= 0xb000 && a < 0xb800) s_vram[a - 0xb000] = v;
}
static uint8_t main_in(z80 *z, uint8_t p)
{
	(void)z;
	if (p == 0x7e || p == 0x7f) { mcu_run_to(s_mcu_target); return upi41_master_r(&s_mcu[0], p & 1); }
	return 0xff;
}
static void main_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (p == 0x7e || p == 0x7f) upi41_master_w(&s_mcu[0], p & 1, v);
}

// ---- sub CPU ----
static uint8_t sub_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x4000) return s_sub_rom[a];
	if (a < 0x4400) {
		uint8_t d = s_sub_ram[a - 0x4000];
		if (s_protect_hack && (a == 0x4004 || a == 0x4005))   // hack_r: speed up the timeout count down
			return a == 0x4004 ? (uint8_t)(d & 0x7f) : 0;
		return d;
	}
	return 0;
}
static void sub_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000 && a < 0x4400) s_sub_ram[a - 0x4000] = v;
	else if (a == 0x6000) { s_latch = v; s_audio.z.nmi_pending = 1; }   // sound command + NMI pulse
}
static uint8_t sub_in(z80 *z, uint8_t p)
{
	(void)z;
	switch (p) {
	case 0x00: case 0x01: return upi41_master_r(&s_mcu[2], p & 1);
	case 0x40: case 0x41: return upi41_master_r(&s_mcu[1], p & 1);
	case 0x60: return (uint8_t)(s_fake[0] + 1);
	case 0x61: sync_cpu(&s_sub); return ay8910_data_r(&s_ay[0]);
	case 0x80: return (uint8_t)(s_fake[1] + 1);
	case 0x81: sync_cpu(&s_sub); return ay8910_data_r(&s_ay[1]);
	}
	return 0xff;
}
static void sub_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	switch (p) {
	case 0x00: case 0x01: upi41_master_w(&s_mcu[2], p & 1, v); break;
	case 0x40: case 0x41: upi41_master_w(&s_mcu[1], p & 1, v); break;
	case 0x60: sync_cpu(&s_sub); ay8910_address_w(&s_ay[0], v); s_fake[0] = v; break;
	case 0x61: sync_cpu(&s_sub); ay8910_data_w(&s_ay[0], v); break;
	case 0x80: sync_cpu(&s_sub); ay8910_address_w(&s_ay[1], v); s_fake[1] = v; break;
	case 0x81: sync_cpu(&s_sub); ay8910_data_w(&s_ay[1], v); break;
	}
}

// ---- ADPCM CPU ----
static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_audio_rom[a];
	if (a == 0xa000) return s_latch;
	return 0;
}
static void audio_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a != 0x8000) return;
	sync_cpu(&s_audio);
	msm5205_data_w(&s_msm, v & 0x0f);
	msm5205_reset_w(&s_msm, (v >> 5) & 1);
	int vclk = (v >> 4) & 1;
	if (s_vclk && !vclk)
		msm5205_vclk_capture(&s_msm);
	s_vclk = vclk;
}

static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void zcpu_init(zcpu *c, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t),
                      uint8_t (*in)(z80 *, uint8_t), void (*out)(z80 *, uint8_t, uint8_t))
{
	z80_init(&c->z);
	c->z.read_byte = rd;
	c->z.write_byte = wr;
	c->z.port_in = in ? in : no_in;
	c->z.port_out = out ? out : no_out;
	c->z.int_data = 0xff;
	c->irq = 0;
}

static void zcpu_run(zcpu *c, int cycles)
{
	c->start = c->z.cyc;
	c->len = cycles;
	unsigned long end = c->z.cyc + (unsigned long)cycles;
	while ((long)(c->z.cyc - end) < 0) {
		c->z.int_pending = c->irq ? 1 : 0;
		z80_step(&c->z);
		if (c->irq && !c->z.int_pending)
			c->irq = 0;
	}
}

// ---- video ----
static int gbit(const uint8_t *p, uint32_t b) { return (p[b >> 3] >> (7 - (b & 7))) & 1; }

// 2 bits per pixel, planes { 0, 4 } packed into one byte; xg = bit offsets of the 4-pixel groups, yo = row offsets
static uint8_t *decode2(const uint8_t *src, int n, int w, const uint32_t *xg, const uint32_t *yo, uint32_t inc)
{
	uint8_t *out = (uint8_t *)malloc((size_t)n * (size_t)(w * w));
	if (!out)
		return NULL;
	for (int t = 0; t < n; t++)
		for (int y = 0; y < w; y++)
			for (int x = 0; x < w; x++) {
				uint32_t b = (uint32_t)t * inc + yo[y] + xg[x >> 2] + (uint32_t)(x & 3);
				out[(size_t)t * (size_t)(w * w) + (size_t)(y * w + x)] = (uint8_t)(gbit(src, b) << 1 | gbit(src, b + 4));
			}
	return out;
}

static void draw_sprites(uint16_t *bm, int pitch)
{
	const uint8_t *tile_ram = s_snap_ram + 0x380, *xy = s_snap_ram + 0x780;
	for (int offs = 0; offs < 0x80 - 1; offs += 2) {
		if (xy[offs] == 0xf1)
			continue;
		int tile = tile_ram[offs];
		int color = tile_ram[offs + 1] & 0x3f;
		int sy = 241 - xy[offs];
		int sx = xy[offs + 1] - 56;
		int flipx = s_snap_attr[offs] & 0x02, flipy = s_snap_attr[offs] & 0x01;
		if (sx < 0) sx += 256;
		int size = 16;
		const uint8_t *g;
		if (tile > 127) {
			tile -= 128;
			sy -= 16;
			size = 32;
			g = s_spr32 + (tile & 63) * 1024;
		} else
			g = s_spr16 + tile * 256;
		if (s_snap_flip) { flipx = !flipx; flipy = !flipy; }
		const uint8_t *ct = s_ctab + color * 4;
		for (int y = 0; y < size; y++) {
			int dy = sy + y;
			if (dy < VIS_Y0 || dy > VIS_Y1)
				continue;
			const uint8_t *row = g + (flipy ? size - 1 - y : y) * size;
			uint16_t *d = bm + (dy - VIS_Y0) * pitch;
			for (int x = 0; x < size; x++) {
				int dx = sx + x;
				if (dx < 0 || dx >= SCREEN_W)
					continue;
				int pen = row[flipx ? size - 1 - x : x];
				if (ct[pen] != 0x8f)   // transmask: pens mapped to colour 0x8f are transparent
					d[dx] = (uint16_t)(256 + color * 4 + pen);
			}
		}
	}
}

static void gs_render(void)
{
	uint32_t *pal = rbx_video.palette;
	for (int i = 0; i < 256; i++) {
		pal[i] = s_colors[i];
		pal[256 + i] = s_colors[s_ctab[i]];
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const int fl = s_snap_flip ? 7 : 0;
	for (int y = VIS_Y0; y <= VIS_Y1; y++) {
		int ty = (y + s_snap_scroll) & 511;
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		for (int col = 0; col < 32; col++) {
			int code = s_snap_vram[(ty >> 3) * 32 + col] + ((s_snap_charbank & 3) << 8);
			int color = ((code & 0x3c0) >> 6) + 16 * s_snap_palbank;
			const uint8_t *row = s_chr + code * 64 + ((ty & 7) ^ fl) * 8;
			uint16_t base = (uint16_t)(color * 4);
			for (int x = 0; x < 8; x++)
				d[col * 8 + x] = (uint16_t)(base + row[x ^ fl]);
		}
	}
	draw_sprites(bm, pitch);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap_ram, s_ram + 0x1000, sizeof(s_snap_ram));
	memcpy(s_snap_attr, s_attr, sizeof(s_attr));
	memcpy(s_snap_vram, s_vram, sizeof(s_vram));
	s_snap_charbank = s_charbank;
	s_snap_palbank = s_charpalbank;
	s_snap_flip = s_flip;
	s_snap_scroll = s_scroll;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void gs_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_attr, 0, sizeof(s_attr));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_sub_ram, 0, sizeof(s_sub_ram));
	memset(&s_main, 0, sizeof(s_main));
	memset(&s_sub, 0, sizeof(s_sub));
	memset(&s_audio, 0, sizeof(s_audio));
	zcpu_init(&s_main, main_read, main_write, main_in, main_out);
	zcpu_init(&s_sub, sub_read, sub_write, sub_in, sub_out);
	zcpu_init(&s_audio, audio_read, audio_write, NULL, NULL);
	for (int i = 0; i < 3; i++) {
		mcs48_reset(&s_mcu[i]);
		s_mt[i] = 0;
	}
	s_mcu_target = 0;
	s_mcu1_p1 = s_mcu2_p1 = 0xff;
	s_fake[0] = s_fake[1] = 0;
	s_latch = 0;
	s_protect_hack = s_nmi_enable = 0;
	s_vclk = 0;
	s_charbank = s_charpalbank = s_flip = s_scroll = 0;
	ay8910_reset(&s_ay[0]);
	ay8910_reset(&s_ay[1]);
	msm5205_reset(&s_msm);
	s_done = 0;
	s_mix_n = 0;
	s_frac = s_mfrac = 0;
	s_dc_x = s_dc_y = 0;
}

static void gs_exit(void)
{
	free(s_chr); free(s_spr16); free(s_spr32);
	s_chr = s_spr16 = s_spr32 = NULL;
	free(s_mix);
	s_mix = NULL;
}

static int gs_init(const rbx_game *g)
{
	if (strcmp(g->machine, "gsword")) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	uint32_t ms, ss, as, s1, s2, s3, ps, m1, m2, m3;
	s_main_rom = rbx_region("maincpu", &ms);
	s_sub_rom = rbx_region("sub", &ss);
	s_audio_rom = rbx_region("audiocpu", &as);
	uint8_t *mr[3] = { rbx_region("mcu1", &m1), rbx_region("mcu2", &m2), rbx_region("mcu3", &m3) };
	uint8_t *g1 = rbx_region("gfx1", &s1), *g2 = rbx_region("gfx2", &s2), *g3 = rbx_region("gfx3", &s3);
	uint8_t *prom = rbx_region("proms", &ps);
	if (!s_main_rom || ms < 0x10000 || !s_sub_rom || ss < 0x10000 || !s_audio_rom || as < 0x10000 || !mr[0] || !mr[1] ||
	    !mr[2] || m1 < 0x400 || m2 < 0x400 || m3 < 0x400 || !g1 || s1 < 0x4000 || !g2 || s2 < 0x2000 || !g3 ||
	    s3 < 0x4000 || !prom || ps < 0x300) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	for (int i = 0; i < 3; i++) {
		mcs48_init(&s_mcu[i], MCS48_I8041A);
		s_mcu[i].rom = mr[i];
		s_mcu[i].rom_mask = 0x3ff;
		s_mcu[i].userdata = (void *)(intptr_t)i;
		s_mcu[i].port_r = mcu_port_r;
		s_mcu[i].port_w = mcu_port_w;
		s_mcu[i].test_r = mcu_test_r;
	}
	// gsword_text: 1024 8x8; gsword_sprites1: 128 16x16; gsword_sprites2: 64 32x32
	static const uint32_t tx[2] = { 0, 64 }, ty[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
	static const uint32_t s1x[4] = { 0, 64, 128, 192 };
	static const uint32_t s1y[16] = { 0, 8, 16, 24, 32, 40, 48, 56, 256, 264, 272, 280, 288, 296, 304, 312 };
	static const uint32_t s2x[8] = { 0, 64, 128, 192, 512, 576, 640, 704 };
	uint32_t s2y[32];
	for (int i = 0; i < 8; i++) {
		s2y[i] = (uint32_t)i * 8;
		s2y[8 + i] = 256 + (uint32_t)i * 8;
		s2y[16 + i] = 1024 + (uint32_t)i * 8;
		s2y[24 + i] = 1280 + (uint32_t)i * 8;
	}
	s_chr = decode2(g1, 1024, 8, tx, ty, 128);
	s_spr16 = decode2(g2, 128, 16, s1x, s1y, 512);
	s_spr32 = decode2(g3, 64, 32, s2x, s2y, 2048);
	if (!s_chr || !s_spr16 || !s_spr32) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		gs_exit();
		return -1;
	}
	for (int t = 0; t < 1024; t++)
		s_chr_opaque[t] = 1;
	for (int i = 0; i < 256; i++) {   // gsword_palette
		int hi = prom[i | 0x100], lo = prom[i];
		int r = 0x21 * (hi & 1) + 0x47 * ((hi >> 1) & 1) + 0x97 * ((hi >> 2) & 1);
		int gg = 0x21 * ((hi >> 3) & 1) + 0x47 * (lo & 1) + 0x97 * ((lo >> 1) & 1);
		int b = 0x47 * ((lo >> 2) & 1) + 0x97 * ((lo >> 3) & 1);
		s_colors[i] = (uint32_t)(r << 16 | gg << 8 | b);
		int c = prom[i | 0x200];
		s_ctab[i] = (uint8_t)(((c & 1) << 3 | (c & 2) << 1 | (c & 4) >> 1 | (c & 8) >> 3) | 0x80);
	}
	s_port_mcu1p1 = rbx_port_find("MCU1.P1");
	s_in0 = rbx_port_find("IN0");
	s_in1 = rbx_port_find("IN1");
	s_coins = rbx_port_find("COINS");
	s_dsw0 = rbx_port_find("DSW0");
	s_dsw1 = rbx_port_find("DSW1");
	s_dsw2 = rbx_port_find("DSW2");

	ay8910_init(&s_ay[0], AY8910_TYPE_AY8910, 1500000, AY8910_LEGACY_OUTPUT, 77, 4, 0);   // route 0.30
	ay8910_init(&s_ay[1], AY8910_TYPE_AY8910, 1500000, AY8910_LEGACY_OUTPUT, 77, 4, 0);
	s_ay[1].port_a_write = nmi_set_w;
	msm5205_init(&s_msm, 400000, MSM5205_SEX_4B, NULL);
	s_mix_cap = (int)(ay8910_rate(&s_ay[0]) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(ay8910_rate(&s_ay[0])) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 512)) {
		gs_exit();
		return -1;
	}
	gs_reset();
	return 0;
}

static void gs_frame(void)
{
	s_line_pos = 0;
	for (int i = 0; i < LINES; i++) {
		int line = (VIS_Y1 + 1 + i) % LINES;
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			s_main.irq = 1;   // irq0_line_hold
		}
		if ((i & 63) == 0 && s_nmi_enable)   // sub CPU: 4 x 60 Hz NMI when enabled
			s_sub.z.nmi_pending = 1;
		int cyc = rbx_line_cycles(&s_frac, CLOCK, 60000, LINES);
		s_mcu_target += (uint64_t)rbx_line_cycles(&s_mfrac, MCU_CYCLES, 60000, LINES);
		zcpu_run(&s_main, cyc);
		zcpu_run(&s_sub, cyc);
		zcpu_run(&s_audio, cyc);
		mcu_run_to(s_mcu_target);
		s_line_pos += cyc;
		mix_to((uint32_t)s_line_pos);
	}
	// DC blocker (one-pole high pass): the MSM5205 holds its last level between samples and the legacy AY
	// output is unipolar; MAME keeps that DC level, here it would only eat the headroom
	for (int i = 0; i < s_mix_n; i++) {
		int32_t x = s_mix[i] * 256;
		s_dc_y = x - s_dc_x + s_dc_y - s_dc_y / 512;
		s_dc_x = x;
		rbx_audio_put(s_dc_y / 256);
	}
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_cap);
	s_mix_n = 0;
	s_done = 0;
}

static int zcpu_state(void *ctx, const char *tag, zcpu *c, int load)
{
	zcpu t = *c;
	if (rbx_state_io(ctx, tag, &t, sizeof(t)))
		return -1;
	if (load) {
		t.z.read_byte = c->z.read_byte;
		t.z.write_byte = c->z.write_byte;
		t.z.port_in = c->z.port_in;
		t.z.port_out = c->z.port_out;
		t.z.userdata = c->z.userdata;
		*c = t;
	}
	return 0;
}

static int gs_state(void *ctx, int load)
{
	static const char *const mtag[3] = { "mcu1", "mcu2", "mcu3" };
	int rc = 0;
	rc |= zcpu_state(ctx, "main", &s_main, load);
	rc |= zcpu_state(ctx, "sub", &s_sub, load);
	rc |= zcpu_state(ctx, "audio", &s_audio, load);
	for (int i = 0; i < 3; i++)
		rc |= rbx_state_io(ctx, mtag[i], (uint8_t *)&s_mcu[i] + MCS48_STATE_OFFSET, (uint32_t)MCS48_STATE_SIZE);
	rc |= rbx_state_io(ctx, "mtime", s_mt, sizeof(s_mt));
	rc |= rbx_state_io(ctx, "mtarget", &s_mcu_target, sizeof(s_mcu_target));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "attr", s_attr, sizeof(s_attr));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "subram", s_sub_ram, sizeof(s_sub_ram));
	rc |= rbx_state_io(ctx, "ay0", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "ay1", &s_ay[1], ay8910_state_size());
	int32_t msm[6] = { s_msm.data, s_msm.reset, s_msm.signal, s_msm.step, 0, 0 };
	rc |= rbx_state_io(ctx, "msm", msm, sizeof(msm));
	int32_t misc[14] = { s_mcu1_p1, s_mcu2_p1, s_fake[0], s_fake[1], s_latch, s_protect_hack, s_nmi_enable, s_vclk,
		s_charbank, s_charpalbank, s_flip, s_scroll, s_dc_x, s_dc_y };
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (rc)
		return -1;
	if (load) {
		s_msm.data = msm[0]; s_msm.reset = msm[1]; s_msm.signal = msm[2]; s_msm.step = msm[3];
		s_mcu1_p1 = (uint8_t)misc[0]; s_mcu2_p1 = (uint8_t)misc[1]; s_fake[0] = (uint8_t)misc[2];
		s_fake[1] = (uint8_t)misc[3]; s_latch = (uint8_t)misc[4]; s_protect_hack = misc[5]; s_nmi_enable = misc[6];
		s_vclk = misc[7]; s_charbank = misc[8]; s_charpalbank = misc[9]; s_flip = misc[10]; s_scroll = misc[11];
		s_dc_x = misc[12]; s_dc_y = misc[13];
	}
	return 0;
}

static int gs_save(void *ctx) { return gs_state(ctx, 0); }
static int gs_load(void *ctx) { return gs_state(ctx, 1); }
// josvolly: MAME MACHINE_NOT_WORKING (incomplete MCU hookup, unknown sub CPU interrupt circuit)
static int gs_supported(const char *machine) { return !strcmp(machine, "gsword"); }

const rbx_driver rbx_drv_gsword = { "Great Swordsman", gs_init, gs_exit, gs_reset, gs_frame, gs_save, gs_load, gs_supported, gs_render };
