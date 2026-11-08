/*  Kusayakyuu hardware, ported from MAME src/mame/taito/ksayakyu.cpp (license BSD-3-Clause, copyright-holders:
    Tomasz Slanina). Memory maps, the ROM bank, the vblank IRQ (cleared by the a807 read), the 60 Hz sound IRQ
    (HOLD_LINE), the sound latch / status handshake, the ROM-defined background tilemap (32 x 256 tiles with the
    video control scroll and flips), the text tilemap and the sprites follow MAME (clocks guessed there too).
    The game is ORIENTATION_FLIP_Y in MAME: the table gives rot 0 and the driver mirrors the bitmap vertically.
    Sound: two AY-3-8910 at 18.432 MHz / 16 (sound/ay8910) and a 6-bit R2R DAC (unsigned, DC removed), 0.25 each.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"

#include <stdlib.h>
#include <string.h>

// MAME: 60 Hz, 256 x 256, visible y 16-239, vblank time 0 (256 lines here, 15360 Hz)
#define LINES 256
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256
#define LINE_RATE 15360
#define CPU_LINE 150            // 18.432 MHz / 8
#define AY_LINE 75              // 18.432 MHz / 16
#define AY_CLOCK 1152000

typedef struct {
	z80 z;
	int irq;
} zcpu;

static uint8_t *s_main_rom, *s_audio_rom, *s_user1;
static uint32_t s_main_size, s_audio_size;
static uint8_t s_ram[0x800], s_videoram[0x800], s_spriteram[0x800], s_audio_ram[0x400];
static zcpu s_main, s_audio;
static int s_main_irq_line;   // vblank ASSERT_LINE, cleared by int_ack_r
static uint8_t s_bank, s_video_ctrl, s_sound_status, s_soundlatch, s_dac;
static ay8910_t s_ay[2];
static int32_t s_dac_dc;   // DC blocker state (x256)
// video
static uint8_t *s_chars, *s_tiles, *s_sprites;   // 1024 8x8 3bpp, 1024 8x8 2bpp, 256 16x16 3bpp
static uint32_t s_pal_rgb[256];
typedef struct {
	uint8_t videoram[0x800], spriteram[0x800];
	uint8_t video_ctrl;
} snap_t;
static snap_t *s_snap;
// ports
static int s_port_p1, s_port_p2, s_port_dsw;
// sound
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static uint32_t s_ay_done, s_line_ay;
static unsigned long s_audio_start;

static void ay_run_to(uint32_t pos)
{
	if (pos <= s_ay_done)
		return;
	int n = ay8910_render(&s_ay[0], pos - s_ay_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	ay8910_render(&s_ay[1], pos - s_ay_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	// 6-bit R2R DAC, 0.25 of full scale, with a DC blocker
	int32_t v = (int32_t)(s_dac & 0x3f) * 8191 / 63;
	for (int i = 0; i < n; i++) {
		int32_t x = v * 256;
		s_dac_dc += (x - s_dac_dc) >> 9;
		s_mix[s_mix_n + i] += (x - s_dac_dc) >> 8;
	}
	s_mix_n += n;
	s_ay_done = pos;
}
static void ay_sync(void)
{
	uint32_t done = (uint32_t)(s_audio.z.cyc - s_audio_start);
	if (done > CPU_LINE)
		done = CPU_LINE;
	ay_run_to(s_line_ay + AY_LINE * done / CPU_LINE);
}
static uint8_t ay1_port_a(void *u) { (void)u; return s_soundlatch; }

static uint8_t port8(int p) { return p >= 0 ? (uint8_t)rbx_port_read(p) : 0; }

// ---- main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x4000) return s_main_rom[a];
	if (a < 0x8000) return s_main_rom[0x10000 + s_bank * 0x4000 + (a - 0x4000)];
	if (a < 0xa000) return s_main_rom[a];
	if (a < 0xa800) return s_ram[a & 0x7ff];
	if (a >= 0xb000 && a < 0xb800) return s_videoram[a & 0x7ff];
	if (a >= 0xb800) return s_spriteram[a & 0x7ff];
	switch (a) {
	case 0xa800: return port8(s_port_p1) & 0xdf;   // (bit 5 has no MAME field: reads 0)
	case 0xa801: return port8(s_port_p2) & 0xdf;
	case 0xa802: return port8(s_port_dsw);
	case 0xa806: return (uint8_t)(s_sound_status | 4);
	case 0xa807: s_main_irq_line = 0; return 0xff;   // int_ack_r
	}
	return 0;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xa000) return;
	if (a < 0xa800) { s_ram[a & 0x7ff] = v; return; }
	if (a >= 0xb000 && a < 0xb800) { s_videoram[a & 0x7ff] = v; return; }
	if (a >= 0xb800) { s_spriteram[a & 0x7ff] = v; return; }
	switch (a) {
	case 0xa804: s_video_ctrl = v; break;
	case 0xa805: s_sound_status &= 0x7f; s_soundlatch = v | 0x80; break;   // latch_w
	case 0xa808: s_bank = v & 1; break;
	}
}

// ---- sound CPU ----
static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_audio_rom[a];
	if (a < 0x8400) return s_audio_ram[a & 0x3ff];
	if (a == 0xa001) { ay_sync(); return ay8910_data_r(&s_ay[0]); }
	return 0;
}
static void audio_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8400) { s_audio_ram[a & 0x3ff] = v; return; }
	switch (a) {
	case 0xa002: case 0xa003: ay_sync(); ay8910_write(&s_ay[0], (a & 1) ^ 1, v); break;   // data_address_w
	case 0xa006: case 0xa007: ay_sync(); ay8910_write(&s_ay[1], (a & 1) ^ 1, v); break;
	case 0xa008: ay_sync(); s_dac = v; break;
	case 0xa00c: s_sound_status |= 0x80; s_soundlatch = v; break;   // tomaincpu_w
	}
}
static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// ---- video ----
static void ks_render(void)
{
	const snap_t *s = s_snap;
	memcpy(rbx_video.palette, s_pal_rgb, sizeof(s_pal_rgb));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, h = rbx_video.height;
	const int flip = (s->video_ctrl & 4) ? 1 : 0;
	if (s->video_ctrl & 1) {
		// background: 32 x 256 tiles from the user1 ROM, scrolled by bits 5-7 of the control
		int bfx, bfy;
		if (flip) { bfy = 1; bfx = (s->video_ctrl & 2) ? 0 : 1; }
		else { bfy = 0; bfx = (s->video_ctrl & 2) ? 1 : 0; }
		int scroll = (s->video_ctrl & 0xe0) << 3;
		for (int sy = VIS_Y0; sy <= VIS_Y1; sy++) {
			uint16_t *d = bm + (sy - VIS_Y0) * pitch;
			// (MAME flips around the centre of the visible area: y extent 16 + 239 + 1)
			int ty = ((bfy ? 255 - sy : sy) + scroll) & 2047;
			for (int sx = 0; sx < SCREEN_W; sx++) {
				int nx = bfx ? 255 - sx : sx;
				int idx = (ty >> 3) * 32 + (nx >> 3);
				int attr = s_user1[idx + 0x2000];
				int code = s_user1[idx] + ((attr & 3) << 8);
				int px = (attr & 0x80) ? 7 - (nx & 7) : (nx & 7);
				d[sx] = (uint16_t)(0x80 + ((attr >> 2) & 0x0f) * 2 * 4 + s_tiles[code * 64 + (ty & 7) * 8 + px]);
			}
		}
	} else {
		for (int y = 0; y < h; y++)
			memset(bm + y * pitch, 0, SCREEN_W * sizeof(uint16_t));
	}
	// text layer (flipped with the screen)
	for (int sy = VIS_Y0; sy <= VIS_Y1; sy++) {
		uint16_t *d = bm + (sy - VIS_Y0) * pitch;
		int ny = flip ? 255 - sy : sy;
		for (int sx = 0; sx < SCREEN_W; sx++) {
			int nx = flip ? 255 - sx : sx;
			int idx = (ny >> 3) * 32 + (nx >> 3);
			int attr = s->videoram[idx * 2];
			int code = s->videoram[idx * 2 + 1] | ((attr & 3) << 8);
			int px = (attr & 0x80) ? 7 - (nx & 7) : (nx & 7);
			uint8_t p = s_chars[code * 64 + (ny & 7) * 8 + px];
			if (p)
				d[sx] = (uint16_t)(((attr & 0x3c) >> 2) * 8 + p);
		}
	}
	// sprites
	for (int offs = 0x800 - 4; offs >= 0; offs -= 4) {
		const uint8_t *e = s->spriteram + offs;
		int sx = e[2], sy = 240 - e[1];
		int flipx = (e[0] & 0x80) ? 1 : 0, flipy = 0;
		if (flip) { sx = 240 - sx; sy = 240 - sy; flipx ^= 1; flipy ^= 1; }
		if (sy + 16 <= VIS_Y0 || sy > VIS_Y1)
			continue;
		const uint8_t *src = s_sprites + (e[0] & 0x7f) * 256;
		uint16_t base = (uint16_t)(((e[3] & 0x78) >> 3) * 8);
		for (int yy = 0; yy < 16; yy++) {
			int dy = sy + yy;
			if (dy < VIS_Y0 || dy > VIS_Y1)
				continue;
			const uint8_t *r = src + (flipy ? 15 - yy : yy) * 16;
			uint16_t *d = bm + (dy - VIS_Y0) * pitch;
			for (int xx = 0; xx < 16; xx++) {
				int dx = sx + xx;
				uint8_t p = r[flipx ? 15 - xx : xx];
				if (p && dx >= 0 && dx < SCREEN_W)
					d[dx] = (uint16_t)(base + p);
			}
		}
	}
	// ORIENTATION_FLIP_Y
	for (int y = 0; y < h / 2; y++) {
		uint16_t *a = bm + y * pitch, *b = bm + (h - 1 - y) * pitch;
		for (int x = 0; x < SCREEN_W; x++) {
			uint16_t t = a[x];
			a[x] = b[x];
			b[x] = t;
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->videoram, s_videoram, sizeof(s_videoram));
	memcpy(s_snap->spriteram, s_spriteram, sizeof(s_spriteram));
	s_snap->video_ctrl = s_video_ctrl;
	rbx_video_frame_done();
}

// ---- init / frame ----
static int gfx_bit(const uint8_t *gfx, uint32_t size, uint32_t bit)
{
	uint32_t byte = bit >> 3;
	return byte < size ? (gfx[byte] >> (7 - (bit & 7))) & 1 : 0;
}

// planes split in equal parts of the region (RGN_FRAC), first part = most significant bit
static void decode(const uint8_t *gfx, uint32_t size, int np, int big, int count, uint8_t *out)
{
	const int w = big ? 16 : 8;
	const uint32_t part = size / (uint32_t)np, inc = big ? 256 : 64;
	for (int t = 0; t < count; t++)
		for (int y = 0; y < w; y++)
			for (int x = 0; x < w; x++) {
				uint32_t o = (uint32_t)t * inc + (uint32_t)((y & 7) * 8 + (y >= 8 ? 128 : 0) + (x & 7) + (x >= 8 ? 64 : 0));
				int v = 0;
				for (int p = 0; p < np; p++)
					v = v << 1 | gfx_bit(gfx, size, (uint32_t)p * part * 8 + o);
				out[(size_t)t * (size_t)(w * w) + (size_t)(y * w + x)] = (uint8_t)v;
			}
}

static void zcpu_init(zcpu *c, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t))
{
	z80_init(&c->z);
	c->z.read_byte = rd;
	c->z.write_byte = wr;
	c->z.port_in = no_in;
	c->z.port_out = no_out;
	c->z.int_data = 0xff;
	c->irq = 0;
}

static void ks_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_videoram, 0, sizeof(s_videoram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_audio_ram, 0, sizeof(s_audio_ram));
	zcpu_init(&s_main, main_read, main_write);
	zcpu_init(&s_audio, audio_read, audio_write);
	s_main_irq_line = 0;
	s_bank = 0;
	s_video_ctrl = 0;
	s_sound_status = 0xff;
	s_soundlatch = 0;
	s_dac = 0;
	s_dac_dc = 0;
	ay8910_reset(&s_ay[0]);
	ay8910_reset(&s_ay[1]);
	s_ay_done = 0;
	s_mix_n = 0;
}

static void ks_exit(void)
{
	free(s_chars);
	free(s_tiles);
	free(s_sprites);
	free(s_snap);
	free(s_mix);
	s_chars = s_tiles = s_sprites = NULL;
	s_snap = NULL;
	s_mix = NULL;
}

static int ks_init(const rbx_game *g)
{
	uint32_t cs = 0, ts = 0, ss = 0, us = 0, ps = 0;
	s_main_rom = rbx_region("maincpu", &s_main_size);
	s_audio_rom = rbx_region("audiocpu", &s_audio_size);
	uint8_t *chars = rbx_region("chars", &cs), *tiles = rbx_region("tiles", &ts), *spr = rbx_region("sprites", &ss);
	s_user1 = rbx_region("user1", &us);
	uint8_t *prom = rbx_region("proms", &ps);
	if (!s_main_rom || s_main_size < 0x18000 || !s_audio_rom || s_audio_size < 0x8000 || !chars || cs < 0x6000 ||
	    !tiles || ts < 0x4000 || !spr || ss < 0x6000 || !s_user1 || us < 0x4000 || !prom || ps < 0x100) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_chars = (uint8_t *)malloc(1024 * 64);
	s_tiles = (uint8_t *)malloc(1024 * 64);
	s_sprites = (uint8_t *)malloc(256 * 256);
	s_snap = (snap_t *)calloc(1, sizeof(snap_t));
	if (!s_chars || !s_tiles || !s_sprites || !s_snap) {
		ks_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	decode(chars, 0x6000, 3, 0, 1024, s_chars);
	decode(tiles, 0x4000, 2, 0, 1024, s_tiles);
	decode(spr, 0x6000, 3, 1, 256, s_sprites);
	for (int i = 0; i < 256; i++) {
		int r = prom[i] & 7, gg = (prom[i] >> 3) & 7, b = (prom[i] >> 6) & 3;
		r = (r << 5) | (r << 2) | (r >> 1);
		gg = (gg << 5) | (gg << 2) | (gg >> 1);
		b = b * 0x55;
		s_pal_rgb[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	s_port_p1 = rbx_port_find("P1");
	s_port_p2 = rbx_port_find("P2");
	s_port_dsw = rbx_port_find("DSW");
	// two AY-3-8910 at 1.152 MHz, MAME route 0.25 each (legacy output), 48 kHz
	for (int i = 0; i < 2; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 64, 3, 1);
	s_ay[0].port_a_read = ay1_port_a;
	s_mix_cap = (int)(ay8910_rate(&s_ay[0]) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(ay8910_rate(&s_ay[0])) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 256)) {
		ks_exit();
		return -1;
	}
	ks_reset();
	return 0;
}

static void zcpu_run(zcpu *c, int cycles, int level)
{
	unsigned long end = c->z.cyc + (unsigned long)cycles;
	while ((long)(c->z.cyc - end) < 0) {
		if (level)
			c->z.int_pending = s_main_irq_line ? 1 : 0;
		else
			c->z.int_pending = c->irq ? 1 : 0;
		z80_step(&c->z);
		if (!level && c->irq && !c->z.int_pending)
			c->irq = 0;   // HOLD_LINE: acknowledged
	}
}

static void ks_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			s_main_irq_line = 1;   // screen_vblank: ASSERT_LINE
		}
		if (line == 0)
			s_audio.irq = 1;       // periodic 60 Hz irq0_line_hold
		zcpu_run(&s_main, CPU_LINE, 1);
		s_line_ay = s_ay_done;
		s_audio_start = s_audio.z.cyc;
		zcpu_run(&s_audio, CPU_LINE, 0);
		ay_run_to(s_line_ay + AY_LINE);
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	s_ay_done = 0;
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

static int ks_state(void *ctx, int load)
{
	int rc = 0;
	rc |= zcpu_state(ctx, "main", &s_main, load);
	rc |= zcpu_state(ctx, "audio", &s_audio, load);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "videoram", s_videoram, sizeof(s_videoram));
	rc |= rbx_state_io(ctx, "spriteram", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "audram", s_audio_ram, sizeof(s_audio_ram));
	uint8_t regs[6] = { s_bank, s_video_ctrl, s_sound_status, s_soundlatch, s_dac, (uint8_t)s_main_irq_line };
	rc |= rbx_state_io(ctx, "regs", regs, sizeof(regs));
	rc |= rbx_state_io(ctx, "dacdc", &s_dac_dc, sizeof(s_dac_dc));
	rc |= rbx_state_io(ctx, "ay1", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "ay2", &s_ay[1], ay8910_state_size());
	if (rc)
		return -1;
	if (load) {
		s_bank = regs[0] & 1; s_video_ctrl = regs[1]; s_sound_status = regs[2]; s_soundlatch = regs[3];
		s_dac = regs[4]; s_main_irq_line = regs[5];
	}
	return 0;
}

static int ks_save(void *ctx) { return ks_state(ctx, 0); }
static int ks_load(void *ctx) { return ks_state(ctx, 1); }
static int ks_supported(const char *machine) { return !strcmp(machine, "ksayakyu"); }

const rbx_driver rbx_drv_ksayakyu = { "Kusayakyuu", ks_init, ks_exit, ks_reset, ks_frame, ks_save, ks_load,
                                      ks_supported, ks_render };
