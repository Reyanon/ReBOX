/*  Big Event Golf hardware, ported from MAME src/mame/taito/bigevglf.cpp (GPL-2.0+, copyright-holders: Jarek
    Burczynski, Tomasz Slanina).
    Main Z80 (5 MHz, banked ROM, four 256x256 bitmap planes reached through a 256-byte window), sub Z80 (5 MHz, inputs,
    trackballs, Taito 68705P5 MCU through machine/taito68705), sound Z80 (4 MHz) with YM2149, MSM5232 and an 8-bit DAC.
    The LS74 flags MAME writes through a scheduler synchronize are written at once here (the CPUs are interleaved
    twice a scanline).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../machine/taito68705.h"
#include "../../sound/ay8910.h"
#include "../../sound/msm5232.h"

#include <stdlib.h>
#include <string.h>

#define LINES 256
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256
#define MAIN_CLOCK 5000000
#define AUDIO_CLOCK 4000000
#define MCU_CLOCK 500000      // 2 MHz / 4
#define RATE_MHZ 60000
#define SLICES 2

typedef struct {
	z80 z;
	int irq;
} zcpu;

static uint8_t *s_main_rom, *s_sub_rom, *s_audio_rom;
static uint32_t s_main_size, s_sub_size, s_audio_size;
static uint8_t s_ram[0x1000], s_share[0x400], s_pal[0x800], s_spr0[0x800], s_spr1[0xc0], s_sub_ram[0x800];
static uint8_t s_audio_ram[0x800];
static uint8_t *s_vidram;   // 4 planes of 256 x 256
static zcpu s_main, s_sub, s_audio;
static taito68705_t s_mcu;
static int s_bank, s_vidram_bank, s_plane_sel, s_plane_vis, s_ls74[2], s_port_select, s_coin_bit5;
static uint8_t s_latch[2];
static int s_latch_pending[2], s_nmi_enable, s_nmi_out;
static int s_track[2][2];   // trackball counters x 100
static uint64_t s_frac_main, s_frac_audio, s_frac_mcu;
// video
static uint8_t *s_tiles, *s_opaque;
static uint8_t *s_snap_plane;   // visible lines of the shown plane
static uint8_t s_snap_pal[0x800], s_snap_spr0[0x800], s_snap_spr1[0xc0];
// ports
static int s_p00, s_p04, s_dsw1, s_dsw2;
// sound
static ay8910_t s_ay;
static msm5232_t s_msm;
static int32_t s_dac;
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static uint32_t s_snd_done;
static unsigned long s_audio_frame_start;

static uint8_t port8(int p) { return (uint8_t)rbx_port_read(p); }

// ---- sound ----
static void snd_run_to(uint32_t pos)
{
	if (pos <= s_snd_done)
		return;
	uint32_t d = pos - s_snd_done;
	int room = s_mix_cap - s_mix_n;
	int n = ay8910_render(&s_ay, d, s_mix + s_mix_n, room);
	msm5232_render(&s_msm, d, s_mix + s_mix_n, room);
	for (int i = 0; i < n; i++)
		s_mix[s_mix_n + i] += s_dac;
	s_mix_n += n;
	s_snd_done = pos;
}
static void snd_sync(void) { snd_run_to((uint32_t)((s_audio.z.cyc - s_audio_frame_start) / 2)); }

// sound NMI: INPUT_MERGER_ALL_HIGH of (sound latch 1 pending, NMI enable)
static void update_nmi(void)
{
	int out = s_latch_pending[0] && s_nmi_enable;
	if (out && !s_nmi_out)
		s_audio.z.nmi_pending = 1;
	s_nmi_out = out;
}

static uint8_t soundstate_r(void) { return (uint8_t)((s_latch_pending[0] ? 0 : 1) | (s_latch_pending[1] ? 2 : 0)); }

// ---- main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xc000) return s_main_rom[a];
	if (a < 0xd000) return s_ram[a - 0xc000];
	if (a < 0xd800) {
		uint32_t o = 0x10000u + (uint32_t)s_bank * 0x800u + (a - 0xd000u);
		return o < s_main_size ? s_main_rom[o] : 0xff;
	}
	if (a < 0xdc00) return s_share[a - 0xd800];
	if (a >= 0xf000 && a < 0xf100) return s_vidram[0x10000 * s_plane_sel + s_vidram_bank + (a & 0xff)];
	if (a >= 0xf840 && a < 0xf900) return s_spr1[a - 0xf840];
	return 0;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xc000) return;
	if (a < 0xd000) { s_ram[a - 0xc000] = v; return; }
	if (a < 0xd800) return;
	if (a < 0xdc00) { s_share[a - 0xd800] = v; return; }
	if (a >= 0xe000 && a < 0xe800) { s_pal[a - 0xe000] = v; return; }
	if (a >= 0xe800 && a < 0xf000) { s_spr0[a - 0xe800] = v; return; }
	if (a >= 0xf000 && a < 0xf100) { s_vidram[(0x10000 * s_plane_sel + s_vidram_bank + (a & 0xff)) & 0x3ffff] = v; return; }
	if (a >= 0xf840 && a < 0xf900) { s_spr1[a - 0xf840] = v; return; }
}

static uint8_t main_in(z80 *z, uint8_t p)
{
	(void)z;
	if (p == 0x06) return (uint8_t)(s_ls74[0] | s_ls74[1] << 1);
	return 0xff;
}

static void main_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	switch (p) {
	case 0x01:   // plane select
		s_plane_sel = ((v & 4) >> 1) | (v & 1);
		s_plane_vis = ((v & 8) >> 2) | ((v & 2) >> 1);
		break;
	case 0x02: s_bank = v; break;
	case 0x03: s_ls74[0] = 1; break;
	case 0x04: s_ls74[1] = 0; break;
	case 0x05: s_vidram_bank = v * 0x100; break;
	}
}

// ---- sub CPU ----
static uint8_t sub_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x4000) return a < s_sub_size ? s_sub_rom[a] : 0xff;
	if (a < 0x4800) return s_sub_ram[a - 0x4000];
	if (a >= 0x8000 && a < 0x8400) return s_share[a - 0x8000];
	return 0;
}

static void sub_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000 && a < 0x4800) s_sub_ram[a - 0x4000] = v;
	else if (a >= 0x8000 && a < 0x8400) s_share[a - 0x8000] = v;
}

static uint8_t sub_in(z80 *z, uint8_t p)
{
	(void)z;
	switch (p) {
	case 0x00: return port8(s_p00);
	case 0x02: return (uint8_t)(s_track[s_port_select][0] / 100);
	case 0x03: return (uint8_t)(s_track[s_port_select][1] / 100);
	case 0x04:
		s_coin_bit5 ^= 0x20;
		return (uint8_t)((port8(s_p04) & 3) | (taito68705_host_semaphore_r(&s_mcu) ? 0 : 0x08) |
		                 (taito68705_mcu_semaphore_r(&s_mcu) ? 0x10 : 0) | s_coin_bit5);
	case 0x05: return port8(s_dsw1);
	case 0x06: return port8(s_dsw2);
	case 0x0b: return taito68705_data_r(&s_mcu);
	case 0x20: s_latch_pending[1] = 0; return s_latch[1];
	case 0x21: return soundstate_r();
	}
	return 0xff;
}

static void sub_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (p == 0x08) s_port_select = (v & 4) >> 2;
	else if (p == 0x0c) taito68705_data_w(&s_mcu, v);
	else if (p >= 0x10 && p <= 0x17) s_ls74[0] = 0;
	else if (p >= 0x18 && p <= 0x1f) s_ls74[1] = 1;
	else if (p == 0x20) { s_latch[0] = v; s_latch_pending[0] = 1; update_nmi(); }
}

// ---- sound CPU ----
static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xc000) return a < s_audio_size ? s_audio_rom[a] : 0xff;
	if (a < 0xc800) return s_audio_ram[a - 0xc000];
	if (a == 0xd800) { s_latch_pending[0] = 0; update_nmi(); return s_latch[0]; }
	if (a == 0xda00) return soundstate_r();
	return 0;
}

static void audio_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xc800) { s_audio_ram[a - 0xc000] = v; return; }
	if (a == 0xc800 || a == 0xc801) { snd_sync(); ay8910_write(&s_ay, a & 1, v); return; }
	if (a >= 0xca00 && a <= 0xca0d) { snd_sync(); msm5232_write(&s_msm, a & 0x0f, v); return; }
	if (a == 0xd800) { s_latch[1] = v; s_latch_pending[1] = 1; return; }
	if (a == 0xda00) { s_nmi_enable = 1; update_nmi(); return; }
	if (a == 0xdc00) { s_nmi_enable = 0; update_nmi(); return; }
	if (a == 0xde00) { snd_sync(); s_dac = ((int32_t)v - 128) * 128; return; }   // DAC_8BIT_R2R, gain 0.5 (centered)
}

static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void zcpu_init(zcpu *c, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t),
                      uint8_t (*in)(z80 *, uint8_t), void (*out)(z80 *, uint8_t, uint8_t))
{
	unsigned long cyc = c->z.cyc;
	z80_init(&c->z);
	c->z.read_byte = rd;
	c->z.write_byte = wr;
	c->z.port_in = in;
	c->z.port_out = out;
	c->z.int_data = 0xff;
	c->z.cyc = cyc;
	c->irq = 0;
}

static void zcpu_run(zcpu *c, int cycles)
{
	unsigned long end = c->z.cyc + (unsigned long)cycles;
	while ((long)(c->z.cyc - end) < 0) {
		c->z.int_pending = c->irq ? 1 : 0;
		z80_step(&c->z);
		if (c->irq && !c->z.int_pending)
			c->irq = 0;   // HOLD_LINE: acknowledged
	}
}

// ---- video ----
static void bg_render(void)
{
	uint32_t *pal = rbx_video.palette;
	for (int i = 0; i < 0x400; i++) {
		int c = s_snap_pal[i] | s_snap_pal[0x400 + i] << 8;
		int r = (c >> 4) & 15, g = c & 15, b = (c >> 8) & 15;
		pal[i] = (uint32_t)((r * 17) << 16 | (g * 17) << 8 | (b * 17));
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, h = rbx_video.height;
	for (int y = 0; y < h; y++) {
		const uint8_t *src = s_snap_plane + y * 256;
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < SCREEN_W; x++)
			d[x] = src[x];
	}
	for (int i = 0xc0 - 4; i >= 0; i -= 4) {
		int code = s_snap_spr1[i + 1];
		int sx = s_snap_spr1[i + 3];
		int sy = 200 - s_snap_spr1[i];
		uint16_t base = (uint16_t)(0x200 + (s_snap_spr1[i + 2] & 0xf) * 16);
		for (int j = 0; j < 16; j++) {
			int t = s_snap_spr0[(code << 4) + j] + ((s_snap_spr0[0x400 + (code << 4) + j] & 0xf) << 8);
			if (!s_opaque[t])
				continue;
			int x0 = sx + ((j & 1) << 3), y0 = sy + ((j >> 1) << 3);
			const uint8_t *src = s_tiles + t * 64;
			for (int y = 0; y < 8; y++) {
				int yy = y0 + y;
				if (yy < VIS_Y0 || yy > VIS_Y1)
					continue;
				uint16_t *d = bm + (yy - VIS_Y0) * pitch;
				for (int x = 0; x < 8; x++) {
					int xx = x0 + x, c = src[y * 8 + x];
					if (c && xx < SCREEN_W)
						d[xx] = (uint16_t)(base + c);
				}
			}
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap_plane, s_vidram + 0x10000 * s_plane_vis + VIS_Y0 * 256, (VIS_Y1 - VIS_Y0 + 1) * 256);
	memcpy(s_snap_pal, s_pal, sizeof(s_pal));
	memcpy(s_snap_spr0, s_spr0, sizeof(s_spr0));
	memcpy(s_snap_spr1, s_spr1, sizeof(s_spr1));
	rbx_video_frame_done();
}

// ---- init / frame ----
static void bg_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_share, 0, sizeof(s_share));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_spr0, 0, sizeof(s_spr0));
	memset(s_spr1, 0, sizeof(s_spr1));
	memset(s_sub_ram, 0, sizeof(s_sub_ram));
	memset(s_audio_ram, 0, sizeof(s_audio_ram));
	memset(s_vidram, 0, 0x40000);
	memset(&s_main, 0, sizeof(s_main));
	memset(&s_sub, 0, sizeof(s_sub));
	memset(&s_audio, 0, sizeof(s_audio));
	zcpu_init(&s_main, main_read, main_write, main_in, main_out);
	zcpu_init(&s_sub, sub_read, sub_write, sub_in, sub_out);
	zcpu_init(&s_audio, audio_read, audio_write, no_in, no_out);
	taito68705_reset(&s_mcu);
	s_bank = s_vidram_bank = s_plane_sel = s_plane_vis = 0;
	s_ls74[0] = s_ls74[1] = 0;
	s_port_select = s_coin_bit5 = 0;
	s_latch[0] = s_latch[1] = 0;
	s_latch_pending[0] = s_latch_pending[1] = 0;
	s_nmi_enable = s_nmi_out = 0;
	s_frac_main = s_frac_audio = s_frac_mcu = 0;
	ay8910_reset(&s_ay);
	msm5232_reset(&s_msm);
	s_dac = 0;
	s_mix_n = 0;
	s_snd_done = 0;
}

static void bg_exit(void)
{
	free(s_tiles);
	free(s_opaque);
	free(s_vidram);
	free(s_snap_plane);
	free(s_mix);
	s_tiles = s_opaque = s_vidram = s_snap_plane = NULL;
	s_mix = NULL;
}

static int bg_init(const rbx_game *g)
{
	uint32_t mcusize, gsize;
	s_main_rom = rbx_region("maincpu", &s_main_size);
	s_sub_rom = rbx_region("sub", &s_sub_size);
	s_audio_rom = rbx_region("audiocpu", &s_audio_size);
	uint8_t *mcu = rbx_region("bmcu:mcu", &mcusize);
	uint8_t *gfx = rbx_region("gfx", &gsize);
	if (!s_main_rom || s_main_size < 0x10000 || !s_sub_rom || !s_audio_rom || !mcu || !gfx || gsize < 0x20000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&s_mcu, 0, sizeof(s_mcu));
	if (taito68705_init(&s_mcu, TAITO68705_STD, mcu, mcusize)) {
		rbx_fail("%s: MCU ROM missing", g->set);
		return -1;
	}
	s_tiles = (uint8_t *)malloc(4096 * 64);
	s_opaque = (uint8_t *)malloc(4096);
	s_vidram = (uint8_t *)calloc(0x40000, 1);
	s_snap_plane = (uint8_t *)calloc((VIS_Y1 - VIS_Y0 + 1) * 256, 1);
	if (!s_tiles || !s_opaque || !s_vidram || !s_snap_plane) {
		bg_exit();
		return -1;
	}
	// 8x8x4: planes { 0, 1/4, 2/4, 3/4 }, x 0..7 (MSB first), 8 bytes a tile
	for (uint32_t t = 0; t < 4096; t++) {
		int any = 0;
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				int v = 0;
				for (int p = 0; p < 4; p++)
					v = v << 1 | ((gfx[p * 0x8000 + t * 8 + y] >> (7 - x)) & 1);
				s_tiles[t * 64 + y * 8 + x] = (uint8_t)v;
				any |= v;
			}
		s_opaque[t] = any != 0;
	}
	s_p00 = rbx_port_find("PORT00");
	s_p04 = rbx_port_find("PORT04");
	s_dsw1 = rbx_port_find("DSW1");
	s_dsw2 = rbx_port_find("DSW2");

	// YM2149 2 MHz (route 0.15), MSM5232 2 MHz (8 feet outputs at 1.0); both 64 clocks an output sample
	ay8910_init(&s_ay, AY8910_TYPE_YM2149, 2000000, AY8910_LEGACY_OUTPUT, 38, 8, 1);
	static const double msm_cap[8] = { 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6 };
	msm5232_init(&s_msm, 2000000, msm_cap, 4);
	for (int i = 0; i < 8; i++)
		s_msm.gain[i] = 256;
	uint32_t rate = ay8910_rate(&s_ay);
	s_mix_cap = (int)(rate / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rate) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 0x400)) {
		bg_exit();
		return -1;
	}
	memset(s_track, 0, sizeof(s_track));
	bg_reset();
	return 0;
}

static void bg_frame(void)
{
	// trackballs (PORT_SENSITIVITY(30), Y reversed)
	for (int p = 0; p < 2; p++) {
		s_track[p][0] = (s_track[p][0] + rbx_analog_dpad(p + 1, RBX_IPT_TRACKBALL_X, 10) * 30) % 25600;   // (d-pad: key delta 10)
		s_track[p][1] = (s_track[p][1] - rbx_analog_dpad(p + 1, RBX_IPT_TRACKBALL_Y, 10) * 30) % 25600;
		for (int a = 0; a < 2; a++)
			if (s_track[p][a] < 0)
				s_track[p][a] += 25600;
	}
	s_audio_frame_start = s_audio.z.cyc;
	s_snd_done = 0;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			s_main.irq = 1;
			s_sub.irq = 1;
		}
		if (line == 0 || line == LINES / 2)
			s_audio.irq = 1;   // 120 Hz periodic IRQ
		int mc = rbx_line_cycles(&s_frac_main, MAIN_CLOCK, RATE_MHZ, LINES);
		int ac = rbx_line_cycles(&s_frac_audio, AUDIO_CLOCK, RATE_MHZ, LINES);
		int uc = rbx_line_cycles(&s_frac_mcu, MCU_CLOCK, RATE_MHZ, LINES);
		for (int s = 0; s < SLICES; s++) {
			int last = s == SLICES - 1;
			zcpu_run(&s_main, mc / SLICES + (last ? mc % SLICES : 0));
			zcpu_run(&s_sub, mc / SLICES + (last ? mc % SLICES : 0));
			taito68705_execute(&s_mcu, uc / SLICES + (last ? uc % SLICES : 0));
			zcpu_run(&s_audio, ac / SLICES + (last ? ac % SLICES : 0));
		}
	}
	snd_sync();
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
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

static int bg_state(void *ctx, int load)
{
	int rc = 0;
	rc |= zcpu_state(ctx, "main", &s_main, load);
	rc |= zcpu_state(ctx, "sub", &s_sub, load);
	rc |= zcpu_state(ctx, "audio", &s_audio, load);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "share", s_share, sizeof(s_share));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "spr0", s_spr0, sizeof(s_spr0));
	rc |= rbx_state_io(ctx, "spr1", s_spr1, sizeof(s_spr1));
	rc |= rbx_state_io(ctx, "subram", s_sub_ram, sizeof(s_sub_ram));
	rc |= rbx_state_io(ctx, "audram", s_audio_ram, sizeof(s_audio_ram));
	rc |= rbx_state_io(ctx, "vidram", s_vidram, 0x40000);
	rc |= rbx_state_io(ctx, "tmcu", &s_mcu.st, sizeof(s_mcu.st));
	rc |= rbx_state_io(ctx, "tmcu6805", &s_mcu.mcu.st, sizeof(s_mcu.mcu.st));
	rc |= rbx_state_io(ctx, "ay", &s_ay, ay8910_state_size());
	rc |= rbx_state_io(ctx, "msm", &s_msm, msm5232_state_size());
	int32_t misc[16] = { s_bank, s_vidram_bank, s_plane_sel, s_plane_vis, s_ls74[0], s_ls74[1], s_port_select,
		s_coin_bit5, s_latch[0], s_latch[1], s_latch_pending[0], s_latch_pending[1], s_nmi_enable, s_nmi_out, s_dac, 0 };
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	rc |= rbx_state_io(ctx, "track", s_track, sizeof(s_track));
	if (rc)
		return -1;
	if (load) {
		s_bank = misc[0] & 0xff; s_vidram_bank = misc[1] & 0xff00; s_plane_sel = misc[2] & 3; s_plane_vis = misc[3] & 3;
		s_ls74[0] = misc[4]; s_ls74[1] = misc[5]; s_port_select = misc[6] & 1; s_coin_bit5 = misc[7];
		s_latch[0] = (uint8_t)misc[8]; s_latch[1] = (uint8_t)misc[9];
		s_latch_pending[0] = misc[10]; s_latch_pending[1] = misc[11]; s_nmi_enable = misc[12]; s_nmi_out = misc[13];
		s_dac = misc[14];
	}
	return 0;
}

static int bg_save(void *ctx) { return bg_state(ctx, 0); }
static int bg_load(void *ctx) { return bg_state(ctx, 1); }
static int bg_supported(const char *machine) { return !strcmp(machine, "bigevglf"); }

const rbx_driver rbx_drv_bigevglf = { "Big Event Golf", bg_init, bg_exit, bg_reset, bg_frame, bg_save, bg_load, bg_supported, bg_render };
