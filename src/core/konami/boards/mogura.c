/*  Konami Mogura Desse. Ported from MAME src/mame/konami/mogura.cpp (BSD-3-Clause, copyright-holders: David Haywood,
    hap). A Z80 at 3 MHz, a 64x32 tilemap of RAM based 2 bpp characters, two 4-bit DACs in stereo (the outputs
    high-passed).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define PIX_CLOCK 6000000u     // (24 MHz / 4)
#define HTOTAL 384
#define LINES 264              // (59.19 Hz)
#define CPU_LINE 192           // (3 MHz a line)
#define SND_RATE 48000
#define VIS_W 320
#define VIS_Y0 16
#define VIS_H 224

static uint8_t *s_rom, *s_prom;
static uint32_t s_rom_size;
static uint32_t s_rgb[32];
static z80 s_z;
static const uint8_t *s_map[256];
static int s_port_system, s_port_p[4], s_port_service;

static struct {
	uint8_t ram[0x4000];   // c000-dfff the main RAM, e000-efff the characters, f000-ffff the tiles
	uint8_t control, ldac, rdac;
	int32_t irq_line;
	int32_t hpl_x, hpl_y, hpr_x, hpr_y;
	uint64_t z_end, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

static uint8_t s_vs[0x2000];   // (the drawing's snapshot: the characters and the tiles)

static void mg_render(void)   // (screen_update)
{
	memcpy(rbx_video.palette, s_rgb, sizeof(s_rgb));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const uint8_t *chr = s_vs, *tile = s_vs + 0x1000;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		const int row = y >> 3;
		for (int col = 0; col < VIS_W / 8; col++) {
			const int c = col ^ (((col >> 1) & 0x10) ^ 0x20);   // (tilemap_scan)
			const int idx = row << 6 | c;
			const int pen = ((tile[idx + 0x800] >> 1) & 7) * 4;
			const uint8_t *src = chr + (uint32_t)tile[idx] * 16 + (y & 7) * 2;
			for (int x = 0; x < 8; x++) {
				const int b = src[x >> 2], q = x & 3;
				d[col * 8 + x] = (uint16_t)(pen + (((b >> (7 - 2 * q)) & 1) << 1 | ((b >> (6 - 2 * q)) & 1)));
			}
		}
	}
}

static uint8_t rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_rom_size ? s_rom[a] : 0;
	if (a >= 0xc000)
		return t.ram[a - 0xc000];
	return 0;
}
static void wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000)
		t.ram[a - 0xc000] = v;
}
static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }
static uint8_t z_in(z80 *z, uint8_t p)
{
	(void)z;
	switch (p) {
	case 0x08: return port(s_port_system);
	case 0x0c: case 0x0d: case 0x0e: case 0x0f: return port(s_port_p[p - 0x0c]);
	case 0x10: return port(s_port_service);
	}
	return 0xff;
}
static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (p == 0x00) {   // (control_w: bit 3 the IRQ's enable)
		if (!(v & 8))
			t.irq_line = 0;
		t.control = v;
	} else if (p == 0x14) {   // (dac_w)
		t.ldac = v >> 4;
		t.rdac = v & 15;
	}
}

static void mg_reset(void)
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
		s_map[0xc0 + p] = t.ram + p * 256;
	s_z.read_map = s_map;
}

static int mg_init(const rbx_game *g)
{
	uint32_t ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_prom = rbx_region("proms", &ps);
	if (!s_rom || !s_prom || ps < 0x20) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	for (int i = 0; i < 0x20; i++) {   // (palette: the pen is bitswap<5>(i, 2, 1, 0, 4, 3))
		const uint8_t c = s_prom[i];
		const int r = 0x21 * (c & 1) + 0x47 * ((c >> 1) & 1) + 0x97 * ((c >> 2) & 1);
		const int gg = 0x21 * ((c >> 3) & 1) + 0x47 * ((c >> 4) & 1) + 0x97 * ((c >> 5) & 1);
		const int b = 0x52 * ((c >> 6) & 1) + 0xad * ((c >> 7) & 1);
		const int pen = ((i >> 2) & 1) << 4 | ((i >> 1) & 1) << 3 | (i & 1) << 2 | ((i >> 4) & 1) << 1 | ((i >> 3) & 1);
		s_rgb[pen] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	s_port_system = rbx_port_find("SYSTEM");
	static const char *const pn[4] = { "P1", "P2", "P3", "P4" };
	for (int i = 0; i < 4; i++)
		s_port_p[i] = rbx_port_find(pn[i]);
	s_port_service = rbx_port_find("SERVICE");
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, 32)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	rbx_audio.frame_mhz = (uint32_t)((uint64_t)PIX_CLOCK * 1000 / ((uint64_t)HTOTAL * LINES));   // (59.19 Hz)
	memset(&t, 0, sizeof(t));
	mg_reset();
	return 0;
}

static void mg_exit(void) {}

static void mg_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, vblank)
			memcpy(s_vs, t.ram + 0x2000, sizeof(s_vs));
			rbx_video_frame_done();
			if (t.control & 8)
				t.irq_line = 1;
		}
		t.z_end += CPU_LINE;
		z80_run(&s_z, (unsigned long)t.z_end, &t.irq_line, 0);
		t.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(t.snd_frac / PIX_CLOCK);
		t.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		const int32_t l = ((int32_t)t.ldac * 2 - 15) * 546, r = ((int32_t)t.rdac * 2 - 15) * 546;   // (-1 to 1, 0.25)
		for (int i = 0; i < ns; i++) {
			t.hpl_y = l - t.hpl_x + t.hpl_y - t.hpl_y / 225;
			t.hpl_x = l;
			t.hpr_y = r - t.hpr_x + t.hpr_y - t.hpr_y / 225;
			t.hpr_x = r;
			rbx_audio_put_stereo(t.hpl_y, t.hpr_y);
		}
	}
	t.frame++;
}

static int mg_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "mg", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "mg_z80", &z, sizeof(z));
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

static int mg_save(void *ctx) { return mg_state(ctx, 0); }
static int mg_load(void *ctx) { return mg_state(ctx, 1); }
static int mg_supported(const char *machine) { return !strcmp(machine, "mogura"); }

const rbx_driver rbx_drv_mogura = { "Konami Mogura Desse", mg_init, mg_exit, mg_reset, mg_frame, mg_save, mg_load,
                                    mg_supported, mg_render, NULL };
