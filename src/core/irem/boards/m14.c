/*  Irem M14: PT Reach Mahjong. Ported from MAME src/mame/irem/m14.cpp (BSD-3-Clause, copyright-holders: Angelo Salese). An
    8085A at 3 MHz, a 32x32 layer of 1 bpp tiles, a ball and a paddle drawn as rectangles. The sound is MAME's sample set:
    silent here.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/i8080/i8080.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 3000000u     // (6 MHz internally divided by 2)
#define LINES 256              // (set_refresh_hz(60), set_size(256, 256))
#define LINE_HZ (60u * LINES)
#define VW 256
#define VH 224
#define SND_RATE 48000

static uint8_t *s_rom, *s_gfx;
static uint32_t s_rom_size, s_gfx_size;
static i8080_t s_cpu;
static int s_port[3], s_fake;

static struct {
	uint8_t ram[0x400], vram[0x400], cram[0x400];
	uint8_t ballx, bally, paddlex, flip, coins;
	int32_t paddle;
	uint32_t frac, screen_frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t vram[0x400], cram[0x400], ballx, bally, paddlex, flip; } s_vs;

static void capture(void)
{
	memcpy(s_vs.vram, t.vram, sizeof(s_vs.vram));
	memcpy(s_vs.cram, t.cram, sizeof(s_vs.cram));
	s_vs.ballx = t.ballx;
	s_vs.bally = t.bally;
	s_vs.paddlex = t.paddlex;
	s_vs.flip = t.flip;
}

static void rect(uint16_t *bm, int x0, int y0, int w, int h)   // (fill with the white pen, clipped)
{
	for (int y = y0 < 0 ? 0 : y0; y < y0 + h && y < VH; y++) {
		uint16_t *d = bm + y * rbx_video.pitch;
		for (int x = x0 < 0 ? 0 : x0; x < x0 + w && x < VW; x++)
			d[x] = 0x1f;
	}
}

static void m14_render(void)
{
	uint16_t *bm = rbx_video.bitmap;
	const int flip = s_vs.flip;
	for (int pr = 0; pr < 32; pr++)
		for (int pc = 0; pc < 32; pc++) {
			const int idx = (flip ? 31 - pr : pr) * 32 + (flip ? 31 - pc : pc);
			const uint8_t *f = s_gfx + s_vs.vram[idx] * 8;
			const uint32_t base = (uint32_t)(s_vs.cram[idx] & 0x0f) * 2;
			if (pr * 8 >= VH)
				continue;
			for (int j = 0; j < 8; j++) {
				const int v = f[flip ? 7 - j : j];
				uint16_t *d = bm + (pr * 8 + j) * rbx_video.pitch + pc * 8;
				for (int k = 0; k < 8; k++)
					d[k] = (uint16_t)(base + ((v >> (flip ? k : 7 - k)) & 1));
			}
		}
	const int xoffs = -8, p_ybase = 184;
	rect(bm, flip ? 32 * 8 - (s_vs.ballx + xoffs) : s_vs.ballx + xoffs, flip ? 28 * 8 - s_vs.bally : s_vs.bally, 5, 5);
	rect(bm, flip ? 32 * 8 - (s_vs.paddlex + xoffs) : s_vs.paddlex + xoffs, flip ? 28 * 8 - p_ybase : p_ybase, 13, 5);
}

// ---- the 8085's bus ----
static uint8_t rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x2000)
		return s_rom[a];
	if (a < 0x2400)
		return t.ram[a - 0x2000];
	if (a >= 0xe000 && a < 0xe400)
		return t.vram[a - 0xe000];
	if (a >= 0xe400 && a < 0xe800)
		return t.cram[a - 0xe400];
	return 0;
}

static void wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x2000 && a < 0x2400)
		t.ram[a - 0x2000] = v;
	else if (a >= 0xe000 && a < 0xe400)
		t.vram[a - 0xe000] = v;
	else if (a >= 0xe400 && a < 0xe800)
		t.cram[a - 0xe400] = v;
}

static uint8_t in(void *u, uint8_t p)
{
	(void)u;
	switch (p) {
	case 0xf8: return (uint8_t)t.paddle;
	case 0xf9: return (uint8_t)rbx_port_read(s_port[0]);
	case 0xfa: return (uint8_t)rbx_port_read(s_port[1]);
	case 0xfb: return (uint8_t)rbx_port_read(s_port[2]);
	}
	return 0;
}

static void out(void *u, uint8_t p, uint8_t v)
{
	(void)u;
	switch (p) {
	case 0xf8: t.ballx = v; break;
	case 0xf9: t.bally = v; break;
	case 0xfa: t.paddlex = v; break;
	case 0xfb: t.flip = v & 1; break;
	}
}

static uint8_t irq_ack(void *u, int line)   // (HOLD_LINE: the coins' lines)
{
	(void)u;
	if (line == I8085_RST55_LINE || line == I8085_RST65_LINE)
		i8080_set_input(&s_cpu, line, 0);
	return 0;
}

// ---- init / frame ----
static void m14_reset(void);

static int m14_init(const rbx_game *g)
{
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_gfx = rbx_region("gfx1", &s_gfx_size);
	if (!s_rom || s_rom_size < 0x2000 || !s_gfx || s_gfx_size < 0x800) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	static const char *const pn[3] = { "IN0", "IN1", "DSW" };
	for (int i = 0; i < 3; i++)
		s_port[i] = rbx_port_find(pn[i]);
	s_fake = rbx_port_find("FAKE");
	if (rbx_audio_setup(SND_RATE) || rbx_video_setup(VW, VH, 0x20)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	for (int i = 0; i < 0x20; i++) {
		if (i & 1)
			rbx_video.palette[i] = (uint32_t)(((i >> 1) & 1) * 0xff0000 | ((i >> 2) & 1) * 0xff00 | ((i >> 3) & 1) * 0xff);
		else
			rbx_video.palette[i] = (i & 0x10) ? 0xffffff : 0x006d00;   // (pal3bit(3))
	}
	m14_reset();
	return 0;
}

static void m14_exit(void) { }

static void m14_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	memset(&s_vs, 0, sizeof(s_vs));
	s_cpu.read = rd;
	s_cpu.write = wr;
	s_cpu.in = in;
	s_cpu.out = out;
	s_cpu.irq_ack = irq_ack;
	i8080_init(&s_cpu, 1);
	i8080_reset(&s_cpu);
}

static void m14_frame(void)
{
	t.paddle += rbx_analog_dpad(1, RBX_IPT_PADDLE, 10);
	t.paddle = t.paddle < 0 ? 0 : t.paddle > 255 ? 255 : t.paddle;
	const int coins = (int)rbx_port_read(s_fake) & 3;
	if ((coins & 1) && !(t.coins & 1))
		i8080_set_input(&s_cpu, I8085_RST65_LINE, 1);
	if ((coins & 2) && !(t.coins & 2))
		i8080_set_input(&s_cpu, I8085_RST55_LINE, 1);
	t.coins = (uint8_t)coins;
	for (int line = 0; line < LINES; line++) {
		if (line == 0)
			rbx_port_custom(s_port[0], 0x20, 0);
		if (line == VH) {   // (the update; vblank_irq, and the frame number the game takes for its numbers)
			rbx_video_wait_idle();
			capture();
			rbx_video_frame_done();
			t.screen_frame++;
			rbx_port_custom(s_port[0], 0x20, 0x20);
			rbx_port_custom(s_port[1], 0x7f, t.screen_frame & 0x7f);
			i8080_set_input(&s_cpu, I8085_RST75_LINE, 1);
			i8080_set_input(&s_cpu, I8085_RST75_LINE, 0);
		}
		t.frac += CPU_CLOCK;
		const int c = (int)(t.frac / LINE_HZ);
		t.frac -= (uint32_t)c * LINE_HZ;
		i8080_run(&s_cpu, c);
	}
	for (int i = 0; i < SND_RATE / 60; i++)   // (the samples aren't played: silence)
		rbx_audio_put(0);
}

static int m14_state(void *ctx, int load)
{
	int rc = 0;
	(void)load;
	rc |= rbx_state_io(ctx, "m14", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "m14_cpu", &s_cpu.s, sizeof(s_cpu.s));
	return rc ? -1 : 0;
}

static int m14_save(void *ctx) { return m14_state(ctx, 0); }
static int m14_load(void *ctx) { return m14_state(ctx, 1); }
static int m14_supported(const char *machine) { return !strcmp(machine, "m14"); }

const rbx_driver rbx_drv_m14 = { "Irem M14", m14_init, m14_exit, m14_reset, m14_frame, m14_save, m14_load, m14_supported,
                                 m14_render, NULL };
