/*  Mini Vaders, ported from MAME src/mame/taito/minivadr.cpp (license BSD-3-Clause, copyright-holders:
    Takahiro Nogi). Z80 at 4 MHz with the vblank IRQ (HOLD_LINE), 1bpp bitmap, no sound hardware (silence).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"

#include <stdlib.h>
#include <string.h>

// MAME: 60 Hz, 256 x 256, visible y 16-239, vblank time 0 (256 lines here, 15360 Hz)
#define LINES 256
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256
#define LINE_RATE 15360
#define MAIN_CLOCK 4000000
#define AUDIO_RATE 48000

static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t s_vram[0x2000];
static uint8_t s_snap[0x2000];
static z80 s_z;
static int s_irq;
static uint64_t s_frac, s_afrac;
static int s_port_inputs;

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x2000) return a < s_rom_size ? s_rom[a] : 0;
	if (a >= 0xa000 && a < 0xc000) return s_vram[a & 0x1fff];
	if (a == 0xe008) return s_port_inputs >= 0 ? (uint8_t)rbx_port_read(s_port_inputs) : 0xff;
	return 0;
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xa000 && a < 0xc000)
		s_vram[a & 0x1fff] = v;
}
static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void mv_render(void)
{
	rbx_video.palette[0] = 0;
	rbx_video.palette[1] = 0xffffff;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = VIS_Y0; y <= VIS_Y1; y++) {
		const uint8_t *src = s_snap + (y << 5);
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		for (int x = 0; x < SCREEN_W; x++)
			d[x] = (uint16_t)((src[x >> 3] >> (~x & 7)) & 1);
	}
}

static void mv_reset(void)
{
	memset(s_vram, 0, sizeof(s_vram));
	z80_init(&s_z);
	s_z.read_byte = main_read;
	s_z.write_byte = main_write;
	s_z.port_in = no_in;
	s_z.port_out = no_out;
	s_z.int_data = 0xff;
	s_irq = 0;
	s_frac = s_afrac = 0;
}

static void mv_exit(void) {}

static int mv_init(const rbx_game *g)
{
	s_rom = rbx_region("maincpu", &s_rom_size);
	if (!s_rom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_port_inputs = rbx_port_find("INPUTS");
	if (rbx_audio_setup(AUDIO_RATE) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 2))
		return -1;
	mv_reset();
	return 0;
}

static void mv_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y1 + 1) {
			rbx_video_wait_idle();
			memcpy(s_snap, s_vram, sizeof(s_vram));
			rbx_video_frame_done();
			s_irq = 1;   // irq0_line_hold
		}
		unsigned long end = s_z.cyc + (unsigned long)rbx_line_cycles(&s_frac, MAIN_CLOCK, LINE_RATE * 1000, 1);
		while ((long)(s_z.cyc - end) < 0) {
			s_z.int_pending = s_irq ? 1 : 0;
			z80_step(&s_z);
			if (s_irq && !s_z.int_pending)
				s_irq = 0;
		}
	}
	int n = rbx_line_cycles(&s_afrac, AUDIO_RATE, 60 * 1000, 1);   // silence (no sound hardware)
	for (int i = 0; i < n; i++)
		rbx_audio_put(0);
}

static int mv_state(void *ctx, int load)
{
	z80 z = s_z;
	int rc = 0;
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "irq", &s_irq, sizeof(s_irq));
	rc |= rbx_state_io(ctx, "frac", &s_frac, sizeof(s_frac));
	if (rc)
		return -1;
	if (load) {
		z.read_byte = main_read;
		z.write_byte = main_write;
		z.port_in = no_in;
		z.port_out = no_out;
		z.userdata = NULL;
		s_z = z;
	}
	return 0;
}

static int mv_save(void *ctx) { return mv_state(ctx, 0); }
static int mv_load(void *ctx) { return mv_state(ctx, 1); }
static int mv_supported(const char *machine) { return !strcmp(machine, "minivadr"); }

const rbx_driver rbx_drv_minivadr = { "Mini Vaders", mv_init, mv_exit, mv_reset, mv_frame, mv_save, mv_load,
                                      mv_supported, mv_render };
