/*  Konami Target Panic. Ported from MAME src/mame/konami/tgtpanic.cpp (BSD-3-Clause, copyright-holders: Philip Bennett).
    A Z80 at 4 MHz with a 300 Hz IRQ and a 2 bpp bitmap with two programmable colours; no sound hardware.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 4000000u
#define SND_RATE 48000
#define LINES 256
#define VIS 192

static uint8_t *s_rom;
static uint32_t s_rom_size;
static z80 s_z;
static const uint8_t *s_map[256];
static int s_port[2];

static struct {
	uint8_t ram[0x4000];
	uint8_t color;
	int32_t irq_line;
	uint64_t z_end, cyc_frac, irq_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

static struct { uint8_t ram[0x2000], color; } s_vs;

static void capture(void)
{
	memcpy(s_vs.ram, t.ram, sizeof(s_vs.ram));
	s_vs.color = t.color;
}

static inline uint32_t pal1(int b, int g, int r)   // (pal1bit x 3)
{
	return (uint32_t)((r & 1 ? 0xff0000 : 0) | (g & 1 ? 0xff00 : 0) | (b & 1 ? 0xff : 0));
}

static void tp_render(void)   // (screen_update: the hardware doubles the lines)
{
	const int c = s_vs.color;
	rbx_video.palette[0] = 0;
	rbx_video.palette[1] = 0xffffff;
	rbx_video.palette[2] = pal1(c >> 0, c >> 1, c >> 2);
	rbx_video.palette[3] = pal1(c >> 4, c >> 5, c >> 6);
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (uint32_t offs = 0; offs < 0x2000; offs++) {
		const int y = (int)(offs & 0x7f) << 1, x = (int)(offs >> 7) << 2;
		if (y >= VIS || x >= VIS)
			continue;
		int val = s_vs.ram[offs];
		uint16_t *d = bm + y * pitch + x;
		for (int i = 0; i < 4; i++) {
			d[i] = d[i + pitch] = (uint16_t)(val & 3);
			val >>= 2;
		}
	}
}

static uint8_t rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_rom_size ? s_rom[a] : 0xff;
	if (a < 0xc000)
		return t.ram[a - 0x8000];
	return 0xff;
}
static void wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0xc000)
		t.ram[a - 0x8000] = v;
}
static uint8_t z_in(z80 *z, uint8_t p)
{
	(void)z;
	return p < 2 ? (uint8_t)rbx_port_read(s_port[p]) : 0xff;
}
static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (p == 0)
		t.color = v;
}

static void tp_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	z80_init(&s_z);
	s_z.read_byte = rd;
	s_z.write_byte = wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	memset(s_map, 0, sizeof(s_map));
	for (int p = 0; p < 0x80; p++)
		s_map[p] = (uint32_t)p * 256 + 256 <= s_rom_size ? s_rom + p * 256 : NULL;
	for (int p = 0; p < 0x40; p++)
		s_map[0x80 + p] = t.ram + p * 256;
	s_z.read_map = s_map;
}

static int tp_init(const rbx_game *g)
{
	s_rom = rbx_region("maincpu", &s_rom_size);
	if (!s_rom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_port[0] = rbx_port_find("IN0");
	s_port[1] = rbx_port_find("IN1");
	if (rbx_audio_setup(SND_RATE) || rbx_video_setup(VIS, VIS, 4)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	tp_reset();
	return 0;
}

static void tp_exit(void) {}

static void tp_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS + line) % LINES;
		if (vpos == VIS) {   // (vblank: the update)
			capture();
			rbx_video_frame_done();
		}
		t.irq_frac += 300;   // (set_periodic_int: irq0_line_hold)
		if (t.irq_frac >= 60u * LINES) {
			t.irq_frac -= 60u * LINES;
			t.irq_line = 1;
		}
		t.cyc_frac += CPU_CLOCK;
		const uint32_t cyc = (uint32_t)(t.cyc_frac / (60u * LINES));
		t.cyc_frac -= (uint64_t)cyc * (60u * LINES);
		t.z_end += cyc;
		z80_run(&s_z, (unsigned long)t.z_end, &t.irq_line, 1);
		t.snd_frac += SND_RATE;
		const int ns = (int)(t.snd_frac / (60u * LINES));
		t.snd_frac -= (uint64_t)ns * (60u * LINES);
		for (int i = 0; i < ns; i++)
			rbx_audio_put(0);
	}
	t.frame++;
}

static int tp_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "tg", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "tg_z80", &z, sizeof(z));
	if (!rc && load) {
		z.read_byte = rd;
		z.write_byte = wr;
		z.port_in = z_in;
		z.port_out = z_out;
		z.userdata = NULL;
		z.read_map = s_map;
		z.op_rom = NULL;
		s_z = z;
	}
	return rc ? -1 : 0;
}

static int tp_save(void *ctx) { return tp_state(ctx, 0); }
static int tp_load(void *ctx) { return tp_state(ctx, 1); }
static int tp_supported(const char *machine) { return !strcmp(machine, "tgtpanic"); }

const rbx_driver rbx_drv_tgtpanic = { "Konami Target Panic", tp_init, tp_exit, tp_reset, tp_frame, tp_save, tp_load,
                                      tp_supported, tp_render, NULL };
