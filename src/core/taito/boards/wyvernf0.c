/*  Wyvern F-0 hardware, ported from MAME src/mame/taito/wyvernf0.cpp (BSD-3-Clause, copyright-holders: Luca Elia).
    Z80 main (6 MHz) with ROM / RAM banking, Taito 68705P5 MCU (machine/taito68705), Z80 sound (4 MHz) with
    2x YM2149, MSM5232 and an 8-bit DAC (one mono mix as in MAME: the cabinet's second speaker is not routed).
    Video: two 32x32 tilemaps and two sprite banks (4x4 tiles through the banked code lookup RAM), mixed on one
    screen as MAME does (the real cabinet overlays two monitors with a half-silvered mirror).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../machine/taito68705.h"
#include "../../sound/ay8910.h"
#include "../../sound/msm5232.h"

#include <stdlib.h>
#include <string.h>

// 256 x 256 screen at 60 Hz, visible lines 16-239; vblank IRQ at line 240
#define LINES 256
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256
#define MAIN_CLOCK 6000000
#define AUDIO_CLOCK 4000000
#define MCU_CLOCK 750000      // 3 MHz / 4
#define RATE_MHZ 60000
#define SLICES 2

typedef struct {
	z80 z;
	int irq;
	unsigned long start;
} zcpu;

static uint8_t *s_main_rom, *s_bank_rom, *s_audio_rom;
static uint32_t s_audio_size;
static uint8_t s_ram[0x1000], s_objram[0x2000], s_vram[2][0x800], s_scroll[4], s_spr[0x100], s_pal[0x400];
static uint8_t s_audio_ram[0x800];
static zcpu s_main, s_audio;
static taito68705_t s_mcu;
static int s_rombank, s_rambank, s_flipx, s_flipy;
static uint8_t s_latch;
static int s_nmi_enable, s_pending_nmi;
static uint64_t s_frac_main, s_frac_audio, s_frac_mcu;
// video
static uint8_t *s_tiles, *s_sprites, *s_spr_opaque;
static uint32_t s_ntiles, s_nsprites;
static uint8_t s_snap_vram[2][0x800], s_snap_scroll[4], s_snap_spr[0x100], s_snap_obj[0x2000], s_snap_pal[0x400];
static int s_snap_flipx, s_snap_flipy;
// ports
static int s_port[8];
// sound
static ay8910_t s_ay[2];
static msm5232_t s_msm;
static int32_t s_dac;
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static uint32_t s_snd_done;     // sound chip clocks (2 MHz) rendered in this frame
static unsigned long s_audio_frame_start;

static uint8_t port8(int p) { return (uint8_t)rbx_port_read(p); }

// ---- sound ----
static void snd_run_to(uint32_t pos)
{
	if (pos <= s_snd_done)
		return;
	uint32_t d = pos - s_snd_done;
	int room = s_mix_cap - s_mix_n;
	int n = ay8910_render(&s_ay[0], d, s_mix + s_mix_n, room);
	ay8910_render(&s_ay[1], d, s_mix + s_mix_n, room);
	msm5232_render(&s_msm, d, s_mix + s_mix_n, room);
	for (int i = 0; i < n; i++)
		s_mix[s_mix_n + i] += s_dac;
	s_mix_n += n;
	s_snd_done = pos;
}
static void snd_sync(void) { snd_run_to((uint32_t)((s_audio.z.cyc - s_audio_frame_start) / 2)); }

// ---- main CPU ----
static void sound_command_w(uint8_t v)
{
	s_latch = v;
	if (s_nmi_enable)
		s_audio.z.nmi_pending = 1;
	else
		s_pending_nmi = 1;
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_main_rom[a];
	if (a < 0x9000) return s_ram[a - 0x8000];
	if (a < 0xa000) return s_objram[s_rambank * 0x1000 + (a - 0x9000)];
	if (a < 0xc000) return s_bank_rom[s_rombank * 0x2000 + (a - 0xa000)];
	if (a < 0xd000) return s_vram[a < 0xc800 ? 1 : 0][a & 0x7ff];
	if (a >= 0xd300 && a <= 0xd303) return s_scroll[a & 3];
	if (a == 0xd400) return taito68705_data_r(&s_mcu);
	if (a == 0xd401)
		return (uint8_t)((taito68705_host_semaphore_r(&s_mcu) ? 0 : 1) | (taito68705_mcu_semaphore_r(&s_mcu) ? 2 : 0));
	if (a >= 0xd500 && a < 0xd600) return s_spr[a & 0xff];
	if (a >= 0xd600 && a <= 0xd607) return port8(s_port[a & 7]);
	if (a == 0xd610) return s_latch;
	if (a >= 0xd800 && a < 0xdc00) return s_pal[a - 0xd800];
	return 0;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x8000) return;
	if (a < 0x9000) { s_ram[a - 0x8000] = v; return; }
	if (a < 0xa000) { s_objram[s_rambank * 0x1000 + (a - 0x9000)] = v; return; }
	if (a < 0xc000) return;
	if (a < 0xd000) { s_vram[a < 0xc800 ? 1 : 0][a & 0x7ff] = v; return; }
	if (a == 0xd100) {   // flip X / Y, coin lockout, RAM bank
		s_flipx = v & 1;
		s_flipy = (v >> 1) & 1;
		s_rambank = (v >> 7) & 1;
		return;
	}
	if (a == 0xd200) { s_rombank = v & 7; return; }
	if (a >= 0xd300 && a <= 0xd303) { s_scroll[a & 3] = v; return; }
	if (a == 0xd400) { taito68705_data_w(&s_mcu, v); return; }
	if (a >= 0xd500 && a < 0xd600) { s_spr[a & 0xff] = v; return; }
	if (a == 0xd610) { sound_command_w(v); return; }
	if (a >= 0xd800 && a < 0xdc00) { s_pal[a - 0xd800] = v; return; }
}

// ---- sound CPU ----
static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x4000 || (a >= 0xe000 && a < 0xf000)) return a < s_audio_size ? s_audio_rom[a] : 0xff;
	if (a >= 0xc000 && a < 0xc800) return s_audio_ram[a - 0xc000];
	if (a == 0xd000) return s_latch;
	return 0;
}

static void audio_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xc800) { s_audio_ram[a - 0xc000] = v; return; }
	if (a >= 0xc800 && a <= 0xc803) { snd_sync(); ay8910_write(&s_ay[(a >> 1) & 1], a & 1, v); return; }
	if (a >= 0xc900 && a <= 0xc90d) { snd_sync(); msm5232_write(&s_msm, a & 0x0f, v); return; }
	if (a == 0xd000) { s_latch = v; return; }
	if (a == 0xd200) {
		s_nmi_enable = 1;
		if (s_pending_nmi) {
			s_audio.z.nmi_pending = 1;
			s_pending_nmi = 0;
		}
		return;
	}
	if (a == 0xd400) { s_nmi_enable = 0; return; }
	if (a == 0xd600) { snd_sync(); s_dac = ((int32_t)v - 128) * 64; return; }   // DAC_8BIT_R2R, gain 0.25 (centered)
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
			c->irq = 0;   // HOLD_LINE: acknowledged
	}
}

// ---- video ----
static void decode_8x8x4(const uint8_t *src, uint32_t size, uint8_t *dst, uint8_t *opaque, uint32_t n)
{
	// planes RGN_FRAC(0..3, 4), x STEP8(7, -1) (bit x of the byte), y STEP8(0, 8)
	uint32_t q = size / 4;
	for (uint32_t t = 0; t < n; t++) {
		int any = 0;
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				int v = 0;
				for (int p = 0; p < 4; p++)
					v = v << 1 | ((src[p * q + t * 8 + y] >> x) & 1);
				dst[t * 64 + y * 8 + x] = (uint8_t)v;
				any |= v;
			}
		if (opaque)
			opaque[t] = any != 0;
	}
}

static inline int tilemap_pixel(const uint8_t *vram, int u, int v)
{
	int offs = ((v >> 3) * 32 + (u >> 3)) * 2;
	int code = vram[offs] | vram[offs + 1] << 8;
	int px = u & 7, py = v & 7;
	if (code & 0x4000) px ^= 7;
	if (code & 0x8000) py ^= 7;
	int c = s_tiles[(uint32_t)(code & 0x3ff) % s_ntiles * 64 + py * 8 + px];
	return c ? (((code & 0x3000) >> 12) << 4) | c : 0;
}

static void draw_tilemap(int layer, int dx, int dxf, int scrollx, int scrolly)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const uint8_t *vram = s_snap_vram[layer];
	const int cbase = layer << 7;   // color (layer << 3) * 16
	for (int sy = VIS_Y0; sy <= VIS_Y1; sy++) {
		int v = s_snap_flipy ? (255 - sy + scrolly) & 255 : (sy + scrolly) & 255;
		uint16_t *d = bm + (sy - VIS_Y0) * pitch;
		for (int sx = 0; sx < SCREEN_W; sx++) {
			int u = s_snap_flipx ? (255 - sx + scrollx - dxf) & 255 : (sx + scrollx - dx) & 255;
			int c = tilemap_pixel(vram, u, v);
			if (c)
				d[sx] = (uint16_t)(cbase + c);
		}
	}
}

static void draw_sprite_tile(uint32_t code, int color, int flipx, int flipy, int x0, int y0)
{
	code %= s_nsprites;
	if (!s_spr_opaque[code])
		return;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const uint8_t *src = s_sprites + code * 64;
	for (int y = 0; y < 8; y++) {
		int sy = y0 + y;
		if (sy < VIS_Y0 || sy > VIS_Y1)
			continue;
		const uint8_t *row = src + (flipy ? 7 - y : y) * 8;
		uint16_t *d = bm + (sy - VIS_Y0) * pitch;
		for (int x = 0; x < 8; x++) {
			int sx = x0 + x;
			if (sx < 0 || sx >= SCREEN_W)
				continue;
			int c = row[flipx ? 7 - x : x];
			if (c)
				d[sx] = (uint16_t)(color * 16 + c);
		}
	}
}

static void draw_sprites(int fg)
{
	const uint8_t *spr = s_snap_spr + (fg ? 0x80 : 0);
	for (int offs = 0; offs < 0x80; offs += 4) {
		int sx = spr[offs + 3] - ((spr[offs + 2] & 0x80) << 1);
		int sy = 256 - 8 - spr[offs + 0] - 23;
		int flipx = 0, flipy = (spr[offs + 1] >> 7) & 1;
		if (s_snap_flipx) {
			flipx = !flipx;
			sx = 256 - 8 - sx - 3 * 8;
		}
		if (s_snap_flipy) {
			flipy = !flipy;
			sy = 256 - 8 - sy - 3 * 8;
		}
		int code = spr[offs + 1] & 0x7f, color = spr[offs + 2] & 0x0f;
		if (fg) {
			code += 0x80;
			color += 0x10;
		}
		for (int y = 0; y < 4; y++)
			for (int x = 0; x < 4; x++) {
				int o = code * 0x20 + (x + y * 4) * 2;
				draw_sprite_tile((uint32_t)(s_snap_obj[o + 1] << 8 | s_snap_obj[o]), color, flipx, flipy,
				                 sx + (flipx ? 3 - x : x) * 8, sy + (flipy ? 3 - y : y) * 8);
			}
	}
}

static void wy_render(void)
{
	uint32_t *pal = rbx_video.palette;
	for (int i = 0; i < 512; i++) {   // xRGB_444, big endian
		int c = s_snap_pal[i * 2] << 8 | s_snap_pal[i * 2 + 1];
		int r = (c >> 8) & 15, g = (c >> 4) & 15, b = c & 15;
		pal[i] = (uint32_t)((r * 17) << 16 | (g * 17) << 8 | (b * 17));
	}
	uint16_t *bm = rbx_video.bitmap;
	for (int y = 0; y < rbx_video.height; y++)
		memset(bm + y * rbx_video.pitch, 0, SCREEN_W * sizeof(uint16_t));
	// background monitor, then foreground monitor
	draw_tilemap(0, 0x12, 0xf4, s_snap_scroll[2], s_snap_scroll[3]);
	draw_sprites(0);
	draw_sprites(1);
	draw_tilemap(1, 0x10, 0xf6, s_snap_scroll[0], s_snap_scroll[1]);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap_vram, s_vram, sizeof(s_vram));
	memcpy(s_snap_scroll, s_scroll, sizeof(s_scroll));
	memcpy(s_snap_spr, s_spr, sizeof(s_spr));
	memcpy(s_snap_obj, s_objram, sizeof(s_objram));
	memcpy(s_snap_pal, s_pal, sizeof(s_pal));
	s_snap_flipx = s_flipx;
	s_snap_flipy = s_flipy;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void wy_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_objram, 0, sizeof(s_objram));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_scroll, 0, sizeof(s_scroll));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_audio_ram, 0, sizeof(s_audio_ram));
	memset(&s_main, 0, sizeof(s_main));
	memset(&s_audio, 0, sizeof(s_audio));
	zcpu_init(&s_main, main_read, main_write);
	zcpu_init(&s_audio, audio_read, audio_write);
	taito68705_reset(&s_mcu);
	s_rombank = s_rambank = s_flipx = s_flipy = 0;
	s_latch = 0;
	s_nmi_enable = s_pending_nmi = 0;
	s_frac_main = s_frac_audio = s_frac_mcu = 0;
	ay8910_reset(&s_ay[0]);
	ay8910_reset(&s_ay[1]);
	msm5232_reset(&s_msm);
	s_dac = 0;
	s_mix_n = 0;
	s_snd_done = 0;
	s_audio_frame_start = s_audio.z.cyc;
}

static void wy_exit(void)
{
	free(s_tiles);
	free(s_sprites);
	free(s_spr_opaque);
	free(s_mix);
	s_tiles = s_sprites = s_spr_opaque = NULL;
	s_mix = NULL;
}

static int wy_init(const rbx_game *g)
{
	uint32_t msize, bsize, mcusize, tsize, ssize;
	s_main_rom = rbx_region("maincpu", &msize);
	s_bank_rom = rbx_region("rombank", &bsize);
	s_audio_rom = rbx_region("audiocpu", &s_audio_size);
	uint8_t *mcu = rbx_region("bmcu:mcu", &mcusize);
	uint8_t *tiles = rbx_region("tiles", &tsize);
	uint8_t *sprites = rbx_region("sprites", &ssize);
	if (!s_main_rom || msize < 0x8000 || !s_bank_rom || bsize < 0x10000 || !s_audio_rom || !mcu || !tiles || !sprites) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&s_mcu, 0, sizeof(s_mcu));
	if (taito68705_init(&s_mcu, TAITO68705_STD, mcu, mcusize)) {
		rbx_fail("%s: MCU ROM missing", g->set);
		return -1;
	}
	s_ntiles = tsize / 32;
	s_nsprites = ssize / 32;
	s_tiles = (uint8_t *)malloc((size_t)s_ntiles * 64);
	s_sprites = (uint8_t *)malloc((size_t)s_nsprites * 64);
	s_spr_opaque = (uint8_t *)malloc(s_nsprites);
	if (!s_tiles || !s_sprites || !s_spr_opaque) {
		wy_exit();
		return -1;
	}
	decode_8x8x4(tiles, tsize, s_tiles, NULL, s_ntiles);
	decode_8x8x4(sprites, ssize, s_sprites, s_spr_opaque, s_nsprites);
	static const char *tags[8] = { "DSW1", "DSW2", "DSW3", "SYSTEM", "JOY1", "FIRE1", "JOY2", "FIRE2" };
	for (int i = 0; i < 8; i++)
		s_port[i] = rbx_port_find(tags[i]);

	// YM2149 2 MHz (route 0.25), MSM5232 2 MHz (8 feet outputs at 0.5); both 64 clocks an output sample
	ay8910_init(&s_ay[0], AY8910_TYPE_YM2149, 2000000, AY8910_LEGACY_OUTPUT, 64, 8, 1);
	ay8910_init(&s_ay[1], AY8910_TYPE_YM2149, 2000000, AY8910_LEGACY_OUTPUT, 64, 8, 1);
	static const double msm_cap[8] = { 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6 };
	msm5232_init(&s_msm, 2000000, msm_cap, 4);
	for (int i = 0; i < 8; i++)
		s_msm.gain[i] = 128;
	uint32_t rate = ay8910_rate(&s_ay[0]);
	s_mix_cap = (int)(rate / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rate) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 512)) {
		wy_exit();
		return -1;
	}
	wy_reset();
	return 0;
}

static void wy_frame(void)
{
	s_audio_frame_start = s_audio.z.cyc;
	s_snd_done = 0;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			s_main.irq = 1;
		}
		if (line == 0 || line == LINES / 2)
			s_audio.irq = 1;   // 120 Hz periodic IRQ (music tempo)
		int mc = rbx_line_cycles(&s_frac_main, MAIN_CLOCK, RATE_MHZ, LINES);
		int ac = rbx_line_cycles(&s_frac_audio, AUDIO_CLOCK, RATE_MHZ, LINES);
		int uc = rbx_line_cycles(&s_frac_mcu, MCU_CLOCK, RATE_MHZ, LINES);
		for (int s = 0; s < SLICES; s++) {
			int n = s ? 1 : 0;
			zcpu_run(&s_main, mc / SLICES + (n ? mc % SLICES : 0));
			taito68705_execute(&s_mcu, uc / SLICES + (n ? uc % SLICES : 0));
			zcpu_run(&s_audio, ac / SLICES + (n ? ac % SLICES : 0));
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

static int wy_state(void *ctx, int load)
{
	int rc = 0;
	rc |= zcpu_state(ctx, "main", &s_main, load);
	rc |= zcpu_state(ctx, "audio", &s_audio, load);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "objram", s_objram, sizeof(s_objram));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "scroll", s_scroll, sizeof(s_scroll));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "audram", s_audio_ram, sizeof(s_audio_ram));
	rc |= rbx_state_io(ctx, "tmcu", &s_mcu.st, sizeof(s_mcu.st));
	rc |= rbx_state_io(ctx, "tmcu6805", &s_mcu.mcu.st, sizeof(s_mcu.mcu.st));
	rc |= rbx_state_io(ctx, "ay0", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "ay1", &s_ay[1], ay8910_state_size());
	rc |= rbx_state_io(ctx, "msm", &s_msm, msm5232_state_size());
	int32_t misc[8] = { s_rombank, s_rambank, s_flipx, s_flipy, s_latch, s_nmi_enable, s_pending_nmi, s_dac };
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (rc)
		return -1;
	if (load) {
		s_rombank = misc[0] & 7; s_rambank = misc[1] & 1; s_flipx = misc[2]; s_flipy = misc[3];
		s_latch = (uint8_t)misc[4]; s_nmi_enable = misc[5]; s_pending_nmi = misc[6]; s_dac = misc[7];
	}
	return 0;
}

static int wy_save(void *ctx) { return wy_state(ctx, 0); }
static int wy_load(void *ctx) { return wy_state(ctx, 1); }
static int wy_supported(const char *machine) { return !strcmp(machine, "wyvernf0"); }

const rbx_driver rbx_drv_wyvernf0 = { "Wyvern F-0", wy_init, wy_exit, wy_reset, wy_frame, wy_save, wy_load, wy_supported, wy_render };
