/*  Konami Test Board (GX800). Ported from MAME src/mame/konami/kontest.cpp (BSD-3-Clause, copyright-holders: Angelo
    Salese, hap). A Z80 at 3 MHz, RAM based 2 bpp characters and two SN76489As in stereo.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/sn76496.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define PIX_CLOCK 6000000u     // (24 MHz / 4)
#define HTOTAL 384
#define LINES 264              // (59.19 Hz, guessed by MAME)
#define CPU_LINE 192           // (3 MHz a line)
#define SN_CLOCK 1500000u      // (24 MHz / 16)
#define SND_RATE 48000
#define VIS_W 256
#define VIS_Y0 16
#define VIS_H 224

static uint8_t *s_rom, *s_prom;
static uint32_t s_rom_size;
static uint32_t s_rgb[32];
static z80 s_z;
static const uint8_t *s_map[256];
static sn76496_t s_sn[2];
static int s_port[4];

static struct {
	uint8_t ram[0x2000];
	uint8_t control;
	int32_t irq_line;
	uint64_t z_end, sn_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

static uint8_t s_vs[0x2000];   // (the drawing's snapshot)

static void kt_render(void)   // (screen_update)
{
	memcpy(rbx_video.palette, s_rgb, sizeof(s_rgb));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		const int row = y >> 3, yi = y & 7;
		for (int x = 0; x < 32; x++) {
			const int tile = s_vs[x | (row << 6) | 0x820];
			const int attr = s_vs[x | ((row >> 1) << 6) | 0x20] & 7;
			for (int xi = 0; xi < 8; xi++) {
				const int pen = s_vs[(yi << 1) | (xi >> 2) | (tile << 4) | 0x1000];
				const int n = ~xi & 3;
				d[x * 8 + xi] = (uint16_t)(((pen >> n) & 1) | ((pen >> (n | 4)) & 1) << 1 | (attr << 2));
			}
		}
	}
}

static uint8_t rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_rom_size ? s_rom[a] : 0;
	if (a >= 0xe000)
		return t.ram[a - 0xe000];
	return 0;
}
static void wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xe000)
		t.ram[a - 0xe000] = v;
}
static uint8_t z_in(z80 *z, uint8_t p)
{
	(void)z;
	return p >= 0x0c && p <= 0x0f ? (uint8_t)rbx_port_read(s_port[p - 0x0c]) : 0xff;
}
static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (p == 0x00)
		sn76496_write(&s_sn[0], v);
	else if (p == 0x04)
		sn76496_write(&s_sn[1], v);
	else if (p == 0x08) {   // (control_w: d3 the IRQ's enable)
		if (!(v & 8))
			t.irq_line = 0;
		t.control = v;
	}
}

static void kt_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	sn76496_reset(&s_sn[0]);
	sn76496_reset(&s_sn[1]);
	z80_init(&s_z);
	s_z.read_byte = rd;
	s_z.write_byte = wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	memset(s_map, 0, sizeof(s_map));
	for (int p = 0; p < 0x80; p++)
		s_map[p] = (uint32_t)p * 256 + 256 <= s_rom_size ? s_rom + p * 256 : NULL;
	for (int p = 0; p < 0x20; p++)
		s_map[0xe0 + p] = t.ram + p * 256;
	s_z.read_map = s_map;
}

static int kt_init(const rbx_game *g)
{
	uint32_t ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_prom = rbx_region("proms", &ps);
	if (!s_rom || !s_prom || ps < 0x20) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	for (int i = 0; i < 0x20; i++) {
		const uint8_t c = s_prom[i];
		const int r = 0x21 * (c & 1) + 0x47 * ((c >> 1) & 1) + 0x97 * ((c >> 2) & 1);
		const int gg = 0x21 * ((c >> 3) & 1) + 0x47 * ((c >> 4) & 1) + 0x97 * ((c >> 5) & 1);
		const int b = 0x52 * ((c >> 6) & 1) + 0xad * ((c >> 7) & 1);
		s_rgb[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	for (int i = 0; i < 2; i++)
		sn76496_init(&s_sn[i], SN_CLOCK, SN76489A_CONFIG, 128, 32, 1);   // (0.50)
	static const char *const pn[4] = { "IN0", "IN1", "IN2", "IN3" };
	for (int i = 0; i < 4; i++)
		s_port[i] = rbx_port_find(pn[i]);
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, 32)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	rbx_audio.frame_mhz = (uint32_t)((uint64_t)PIX_CLOCK * 1000 / ((uint64_t)HTOTAL * LINES));   // (59.19 Hz)
	memset(&t, 0, sizeof(t));
	kt_reset();
	return 0;
}

static void kt_exit(void) {}

static void kt_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, vblank)
			memcpy(s_vs, t.ram, sizeof(s_vs));
			rbx_video_frame_done();
			if (t.control & 8)
				t.irq_line = 1;
		}
		t.z_end += CPU_LINE;
		z80_run(&s_z, (unsigned long)t.z_end, &t.irq_line, 0);
		t.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(t.snd_frac / PIX_CLOCK);
		t.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		if (ns > 0) {
			int32_t a[2][32];
			memset(a, 0, sizeof(a));
			t.sn_frac += (uint64_t)SN_CLOCK * (uint64_t)ns;
			const uint32_t sc = (uint32_t)(t.sn_frac / SND_RATE);
			t.sn_frac -= (uint64_t)sc * SND_RATE;
			const int k0 = sn76496_render(&s_sn[0], sc, a[0], 32), k1 = sn76496_render(&s_sn[1], sc, a[1], 32);
			for (int i = 0; i < ns; i++)   // (the first chip on the right, the second on the left)
				rbx_audio_put_stereo(k1 > 0 ? a[1][i * k1 / ns] : 0, k0 > 0 ? a[0][i * k0 / ns] : 0);
		}
	}
	t.frame++;
}

static int kt_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "kt", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "kt_sn0", &s_sn[0], sn76496_state_size());
	rc |= rbx_state_io(ctx, "kt_sn1", &s_sn[1], sn76496_state_size());
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "kt_z80", &z, sizeof(z));
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

static int kt_save(void *ctx) { return kt_state(ctx, 0); }
static int kt_load(void *ctx) { return kt_state(ctx, 1); }
static int kt_supported(const char *machine) { return !strcmp(machine, "kontest"); }

const rbx_driver rbx_drv_kontest = { "Konami Test Board", kt_init, kt_exit, kt_reset, kt_frame, kt_save, kt_load,
                                     kt_supported, kt_render, NULL };
