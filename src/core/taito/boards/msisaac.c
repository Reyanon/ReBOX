/*  Metal Soldier Isaac II (msisaac), ported from MAME src/mame/taito/msisaac.cpp
    (GPL-2.0+, copyright-holders: Jarek Burczynski).
    The MCU (a34_14) is not dumped: as in MAME (USE_MCU undefined) the MCU is replaced by the driver's simulation
    (mcu_r / mcu_w / mcu_status_r ported from MAME). Memory maps, the three tilemaps, the sprites, the xRGB_444
    palette and the sound board (Z80 + 2x AY8910 + MSM5232 + TA7630 channel volumes) follow MAME.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"
#include "../../sound/msm5232.h"
#include "../../sound/ta7630.h"

#include <stdlib.h>
#include <string.h>

// MAME: set_refresh_hz(60), set_size(256, 256), visarea y 8..247
#define LINES 256
#define VIS_Y0 8
#define VIS_Y1 247
#define SCREEN_W 256
#define RATE_MHZ 60000
#define MAIN_CLOCK 4000000
#define AUDIO_CLOCK 4000000
#define SND_CLOCK 2000000

typedef struct {
	z80 z;
	int irq;
} zcpu;

static uint8_t *s_main_rom, *s_audio_rom, *s_gfx1, *s_gfx2;
static uint32_t s_main_size, s_audio_size;
static zcpu s_main, s_audio;
static uint8_t s_work[0x800], s_pal[0x800], s_spr[0x80], s_vram[3][0x400], s_audio_ram[0x800];
static uint8_t s_bg2_textbank, s_scroll[6];   // f0c0..f0c5
static uint8_t s_latch, s_sound_nmi_enable, s_pending_nmi, s_snd_ctrl[2];
static uint8_t s_mcu_val, s_direction;
// sound
static ay8910_t s_ay[2];
static msm5232_t s_msm;
static int s_ta[16];
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static uint32_t s_snd_done, s_slice_start, s_slice_len;
static unsigned long s_cpu_start;
static int s_cpu_len;
static uint64_t s_frac_main, s_frac_audio, s_frac_snd;
// video
static uint8_t s_chars[2][0x400 * 64], s_tiles[2][0x100 * 256];
static uint8_t s_snap_pal[0x800], s_snap_spr[0x80], s_snap_vram[3][0x400], s_snap_scroll[6], s_snap_textbank;
// ports
static int s_p_dsw[3], s_p_in0, s_p_in1;

static uint8_t port8(int p) { return (uint8_t)rbx_port_read(p); }

// ---- sound ----
static void snd_run_to(uint32_t pos)
{
	if (pos <= s_snd_done)
		return;
	uint32_t clk = pos - s_snd_done;
	int room = s_mix_cap - s_mix_n;
	if (room <= 0) {
		s_snd_done = pos;
		return;
	}
	int32_t *o = s_mix + s_mix_n;
	int n = ay8910_render(&s_ay[0], clk, o, room);
	ay8910_render(&s_ay[1], clk, o, room);
	msm5232_render(&s_msm, clk, o, room);
	s_mix_n += n;
	s_snd_done = pos;
}

static void snd_sync(void)
{
	uint32_t done = (uint32_t)(s_audio.z.cyc - s_cpu_start);
	if ((int)done > s_cpu_len)
		done = (uint32_t)s_cpu_len;
	snd_run_to(s_slice_start + (s_cpu_len ? (uint32_t)((uint64_t)s_slice_len * done / (uint32_t)s_cpu_len) : 0));
}

// ---- MCU simulation (MAME, USE_MCU undefined) ----
static uint8_t mcu_r(void)
{
	switch (s_mcu_val) {
	case 0x5f: return (uint8_t)(s_mcu_val + 0x6b);   // start-up check
	case 0x40: case 0x41: case 0x42: return 0;
	case 0x02: {   // the MCU controls the body direction
		static const int8_t table[16] = { -1, 2, 6, -1, 0, 1, 7, 0, 4, 3, 5, 4, -1, 2, 6, -1 };
		uint8_t val = (port8(s_p_in1) >> 2) & 0x0f;
		if (table[val] >= 0)
			s_direction = (uint8_t)table[val];
		return s_direction;
	}
	case 0x07: return 0x45;   // controls the arms when they return to the player
	default: return s_mcu_val;
	}
}

// ---- main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xe000) return a < s_main_size ? s_main_rom[a] : 0;
	if (a < 0xe800) return s_work[a & 0x7ff];
	if (a < 0xf000) return s_pal[a & 0x7ff];
	if (a >= 0xf400) return s_vram[a >= 0xfc00 ? 1 : a >= 0xf800 ? 2 : 0][a & 0x3ff];
	if (a >= 0xf100 && a < 0xf180) return s_spr[a & 0x7f];
	switch (a) {
	case 0xf080: return port8(s_p_dsw[0]);
	case 0xf081: return port8(s_p_dsw[1]);
	case 0xf082: return port8(s_p_dsw[2]);
	case 0xf083: return port8(s_p_in0);
	case 0xf084: return port8(s_p_in1);
	case 0xf0e0: return mcu_r();
	case 0xf0e1: return 3;   // mcu_status_r: MCU ready / CPU data ready
	}
	return 0;
}

static void sound_nmi(void) { s_audio.z.nmi_pending = 1; }   // pulse_input_line(INPUT_LINE_NMI)

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xe000) return;
	if (a < 0xe800) { s_work[a & 0x7ff] = v; return; }
	if (a < 0xf000) { s_pal[a & 0x7ff] = v; return; }
	if (a >= 0xf400) { s_vram[a >= 0xfc00 ? 1 : a >= 0xf800 ? 2 : 0][a & 0x3ff] = v; return; }
	if (a >= 0xf100 && a < 0xf180) { s_spr[a & 0x7f] = v; return; }
	if (a >= 0xf0c0 && a <= 0xf0c5) { s_scroll[a - 0xf0c0] = v; return; }
	switch (a) {
	case 0xf000: s_bg2_textbank = v; return;
	case 0xf060:   // sound_command_w + nmi_callback
		s_latch = v;
		if (s_sound_nmi_enable)
			sound_nmi();
		else
			s_pending_nmi = 1;
		return;
	case 0xf0e0: s_mcu_val = v; return;   // mcu_w
	}
}

// ---- sound CPU ----
static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x4000) return a < s_audio_size ? s_audio_rom[a] : 0;
	if (a < 0x4800) return s_audio_ram[a & 0x7ff];
	if (a == 0xc000) return s_latch;
	return 0;
}

static void audio_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000 && a < 0x4800) { s_audio_ram[a & 0x7ff] = v; return; }
	if (a >= 0x8000 && a <= 0x8003) { snd_sync(); ay8910_write(&s_ay[(a >> 1) & 1], a & 1, v); return; }
	if (a >= 0x8010 && a <= 0x801d) { snd_sync(); msm5232_write(&s_msm, a - 0x8010, v); return; }
	switch (a) {
	case 0x8020:   // sound_control_0_w: TA7630 group 1 (low nibble), group 2 (high nibble)
		snd_sync();
		s_snd_ctrl[0] = v;
		for (int i = 0; i < 4; i++) {
			s_msm.gain[i] = s_ta[v & 0xf];
			s_msm.gain[i + 4] = s_ta[v >> 4];
		}
		return;
	case 0x8030: s_snd_ctrl[1] = v; return;
	case 0xc001:   // nmi_enable_w
		s_sound_nmi_enable = 1;
		if (s_pending_nmi) {
			sound_nmi();
			s_pending_nmi = 0;
		}
		return;
	case 0xc002: s_sound_nmi_enable = 0; return;
	}
}

static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void zcpu_init(zcpu *c, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t))
{
	unsigned long cyc = c->z.cyc;
	z80_init(&c->z);
	c->z.read_byte = rd;
	c->z.write_byte = wr;
	c->z.port_in = no_in;
	c->z.port_out = no_out;
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
			c->irq = 0;   // HOLD_LINE acknowledged
	}
}

// ---- video ----
// char_layout 8 x 8 / tile_layout 16 x 16: planes at 0 / 0x2000 / 0x4000 / 0x6000 (MSB first), bit 0 = leftmost pixel
static int gpix(const uint8_t *g, uint32_t byte, int bit)
{
	return ((g[byte] >> bit) & 1) << 3 | ((g[0x2000 + byte] >> bit) & 1) << 2 | ((g[0x4000 + byte] >> bit) & 1) << 1 |
	       ((g[0x6000 + byte] >> bit) & 1);
}

static void decode_gfx(int r, const uint8_t *g)
{
	for (uint32_t c = 0; c < 0x400; c++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++)
				s_chars[r][c * 64 + (uint32_t)(y * 8 + x)] = (uint8_t)gpix(g, c * 8 + (uint32_t)y, x);
	for (uint32_t t = 0; t < 0x100; t++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				uint32_t byte = t * 32 + (uint32_t)(y & 7) + (y >= 8 ? 16 : 0) + (x >= 8 ? 8 : 0);
				s_tiles[r][t * 256 + (uint32_t)(y * 16 + x)] = (uint8_t)gpix(g, byte, x & 7);
			}
}

// one 256 x 256 tilemap: dest (x, y) shows source (x + scrollx, y + scrolly)
static void draw_tilemap(const uint8_t *vram, const uint8_t *chars, int code_add, int color, int opaque, int scrollx, int scrolly)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = VIS_Y0; y <= VIS_Y1; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		int ly = (y + scrolly) & 0xff;
		const uint8_t *row = vram + (ly >> 3) * 32;
		for (int x = 0; x < SCREEN_W; x++) {
			int lx = (x + scrollx) & 0xff;
			uint32_t code = (uint32_t)(row[lx >> 3] + code_add) & 0x3ff;
			uint8_t p = chars[code * 64 + (uint32_t)((ly & 7) * 8 + (lx & 7))];
			if (p || opaque)
				d[x] = (uint16_t)(color + p);
		}
	}
}

static void draw_sprite(const uint8_t *gfx, int code, int color, int flipx, int flipy, int sx, int sy)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const uint8_t *g = gfx + (uint32_t)(code & 0xff) * 256;
	for (int ty = 0; ty < 16; ty++) {
		int y = sy + ty;
		if (y < VIS_Y0 || y > VIS_Y1)
			continue;
		const uint8_t *r = g + (flipy ? 15 - ty : ty) * 16;
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		for (int tx = 0; tx < 16; tx++) {
			int x = sx + tx;
			if (x < 0 || x >= SCREEN_W)
				continue;
			uint8_t p = r[flipx ? 15 - tx : tx];
			if (p)
				d[x] = (uint16_t)(color * 16 + p);
		}
	}
}

static void draw_sprites(void)
{
	for (int offs = 32 * 4 - 4; offs >= 0; offs -= 4) {
		const uint8_t *s = s_snap_spr + offs;
		int sx = s[0], sy = 240 - s[1] - 1, attr = s[2], num = s[3];
		int color = (attr >> 4) & 0xf, flipx = attr & 1, flipy = (attr & 2) != 0;
		const uint8_t *gfx = (attr & 4) ? s_tiles[1] : s_tiles[0];
		if (attr & 8) {   // double size sprite
			if (!flipy) {
				draw_sprite(gfx, num + 1, color, flipx, flipy, sx, sy - 16);
				draw_sprite(gfx, num, color, flipx, flipy, sx, sy);
			} else {
				draw_sprite(gfx, num, color, flipx, flipy, sx, sy - 16);
				draw_sprite(gfx, num + 1, color, flipx, flipy, sx, sy);
			}
		} else
			draw_sprite(gfx, num, color, flipx, flipy, sx, sy);
	}
}

static void mi_render(void)
{
	for (int i = 0; i < 1024; i++) {   // xRGB_444, little endian
		int c = s_snap_pal[i * 2] | s_snap_pal[i * 2 + 1] << 8;
		rbx_video.palette[i] = (uint32_t)(((c >> 8) & 15) * 17 << 16 | ((c >> 4) & 15) * 17 << 8 | (c & 15) * 17);
	}
	const uint8_t *sc = s_snap_scroll;
	// bg: videoram 2 (fc00), gfx 1 code 0x100 + n, colour 0x30, opaque; scrollx 9 + 2 + data
	draw_tilemap(s_snap_vram[1], s_chars[1], 0x100, 0x300, 1, 9 + 2 + sc[4], sc[5]);
	// bg2: videoram 3 (f800), gfx (textbank >> 3) & 1, colour 0x20, pen 0 transparent; scrollx 9 + 4 + data
	draw_tilemap(s_snap_vram[2], s_chars[(s_snap_textbank >> 3) & 1], 0, 0x200, 0, 9 + 4 + sc[2], sc[3]);
	draw_sprites();
	// fg: videoram 1 (f400), gfx 0, colour 0x10, pen 0 transparent; scrollx 9 + data
	draw_tilemap(s_snap_vram[0], s_chars[0], 0, 0x100, 0, 9 + sc[0], sc[1]);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap_pal, s_pal, sizeof(s_pal));
	memcpy(s_snap_spr, s_spr, sizeof(s_spr));
	memcpy(s_snap_vram, s_vram, sizeof(s_vram));
	memcpy(s_snap_scroll, s_scroll, sizeof(s_scroll));
	s_snap_textbank = s_bg2_textbank;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void mi_reset(void)
{
	// (first: the sound syncs read them, from a CPU write or a port callback of a chip reset below, before a frame
	// sets them; the last game's values made a render of billions of clocks, the next game of the board hung)
	s_slice_start = s_slice_len = s_cpu_start = s_cpu_len = 0;
	memset(s_work, 0, sizeof(s_work));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_audio_ram, 0, sizeof(s_audio_ram));
	memset(s_scroll, 0, sizeof(s_scroll));
	memset(&s_main, 0, sizeof(s_main));
	memset(&s_audio, 0, sizeof(s_audio));
	zcpu_init(&s_main, main_read, main_write);
	zcpu_init(&s_audio, audio_read, audio_write);
	// machine_reset
	s_bg2_textbank = 0;
	s_sound_nmi_enable = s_pending_nmi = 0;
	s_snd_ctrl[0] = s_snd_ctrl[1] = 0;
	s_mcu_val = s_direction = 0;
	s_latch = 0;
	ay8910_reset(&s_ay[0]);
	ay8910_reset(&s_ay[1]);
	msm5232_reset(&s_msm);
	for (int i = 0; i < 8; i++)
		s_msm.gain[i] = 256;
	s_frac_main = s_frac_audio = s_frac_snd = 0;
	s_snd_done = 0;
	s_mix_n = 0;
}

static void mi_exit(void)
{
	free(s_mix);
	s_mix = NULL;
}

static int mi_supported(const char *machine) { return !strcmp(machine, "msisaac"); }

static int mi_init(const rbx_game *g)
{
	if (!mi_supported(g->machine)) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	uint32_t s1, s2;
	s_main_rom = rbx_region("maincpu", &s_main_size);
	s_audio_rom = rbx_region("audiocpu", &s_audio_size);
	s_gfx1 = rbx_region("gfx1", &s1);
	s_gfx2 = rbx_region("gfx2", &s2);
	if (!s_main_rom || !s_audio_rom || !s_gfx1 || !s_gfx2 || s1 < 0x8000 || s2 < 0x8000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_p_dsw[0] = rbx_port_find("DSW1");
	s_p_dsw[1] = rbx_port_find("DSW2");
	s_p_dsw[2] = rbx_port_find("DSW3");
	s_p_in0 = rbx_port_find("IN0");
	s_p_in1 = rbx_port_find("IN1");
	decode_gfx(0, s_gfx1);
	decode_gfx(1, s_gfx2);
	ta7630_table(s_ta);
	for (int c = 0; c < 2; c++)   // AY8910 2 MHz, route 0.15
		ay8910_init(&s_ay[c], AY8910_TYPE_AY8910, SND_CLOCK, AY8910_LEGACY_OUTPUT, 38, 4, 1);
	static const double caps[8] = { 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6 };
	msm5232_init(&s_msm, SND_CLOCK, caps, 2);
	uint32_t rate = ay8910_rate(&s_ay[0]);
	s_mix_cap = (int)(rate / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rate) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 1024)) {
		mi_exit();
		return -1;
	}
	mi_reset();
	return 0;
}

static void mi_frame(void)
{
	for (int i = 0; i < LINES; i++) {
		int line = (VIS_Y1 + 1 + i) % LINES;
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			s_main.irq = 1;    // irq0_line_hold
			s_audio.irq = 1;   // irq0_line_hold (source of IRQs is unknown)
		}
		int mc = rbx_line_cycles(&s_frac_main, MAIN_CLOCK, RATE_MHZ, LINES);
		int ac = rbx_line_cycles(&s_frac_audio, AUDIO_CLOCK, RATE_MHZ, LINES);
		int sc = rbx_line_cycles(&s_frac_snd, SND_CLOCK, RATE_MHZ, LINES);
		zcpu_run(&s_main, mc);
		s_slice_start = s_snd_done;
		s_slice_len = (uint32_t)sc;
		s_cpu_start = s_audio.z.cyc;
		s_cpu_len = ac;
		zcpu_run(&s_audio, ac);
		snd_run_to(s_slice_start + s_slice_len);
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	s_snd_done = 0;
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

static int mi_state(void *ctx, int load)
{
	int rc = 0;
	rc |= zcpu_state(ctx, "main", &s_main, load);
	rc |= zcpu_state(ctx, "audio", &s_audio, load);
	rc |= rbx_state_io(ctx, "work", s_work, sizeof(s_work));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "aram", s_audio_ram, sizeof(s_audio_ram));
	rc |= rbx_state_io(ctx, "scroll", s_scroll, sizeof(s_scroll));
	rc |= rbx_state_io(ctx, "ay0", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "ay1", &s_ay[1], ay8910_state_size());
	rc |= rbx_state_io(ctx, "msm", &s_msm, msm5232_state_size());
	uint8_t misc[9] = { s_bg2_textbank, s_latch, s_sound_nmi_enable, s_pending_nmi, s_snd_ctrl[0], s_snd_ctrl[1],
		s_mcu_val, s_direction, 0 };
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (rc)
		return -1;
	if (load) {
		s_bg2_textbank = misc[0]; s_latch = misc[1]; s_sound_nmi_enable = misc[2]; s_pending_nmi = misc[3];
		s_snd_ctrl[0] = misc[4]; s_snd_ctrl[1] = misc[5]; s_mcu_val = misc[6]; s_direction = misc[7];
	}
	return 0;
}

static int mi_save(void *ctx) { return mi_state(ctx, 0); }
static int mi_load(void *ctx) { return mi_state(ctx, 1); }

const rbx_driver rbx_drv_msisaac = { "Metal Soldier Isaac II", mi_init, mi_exit, mi_reset, mi_frame, mi_save, mi_load, mi_supported, mi_render };
