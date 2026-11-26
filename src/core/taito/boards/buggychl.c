/*  Buggy Challenge (buggychl, buggychlt), ported from MAME src/mame/taito/buggychl.cpp
    (BSD-3-Clause, copyright-holders: Ernesto Corvi, Nicola Salmoria).
    Memory maps, the RAM-based character set, the column + row scrolled background (copyscrollbitmap passes), the
    zoomed line-based sprites with their lookup RAM, the sky gradient blend, the Taito 68705 MCU interface
    (machine/taito68705) and the sound board (Z80 + 2x YM2149 + MSM5232 + TA7630 volume) follow MAME.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../rbx_core.h"
#include "../../cpu/z80/z80.h"
#include "../../machine/taito68705.h"
#include "../../sound/ay8910.h"
#include "../../sound/msm5232.h"
#include "../../sound/ta7630.h"

#include <stdlib.h>
#include <string.h>

// MAME: set_refresh_hz(60), set_size(256, 256), visarea y 16..239
#define LINES 256
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256
#define RATE_MHZ 60000
#define MAIN_CLOCK 6000000     // 48 MHz / 8
#define AUDIO_CLOCK 4000000    // 8 MHz / 2
#define MCU_CLOCK 750000       // 48 MHz / 8 / 2, internal cycles (/ 4)
#define SND_CLOCK 2000000      // 8 MHz / 4
#define AUDIO_IRQ_PERIOD 32768 // 8 MHz / 2 / 2 / 256 / 64 (~122 Hz) in 4 MHz CPU cycles
#define BG_PEN 0x20
#define SKY_BASE 256           // palette entries 256 + g: the blended sky colours

typedef struct {
	z80 z;
	int irq;
} zcpu;

static uint8_t *s_main_rom, *s_audio_rom, *s_spr_rom, *s_zoom;
static uint32_t s_main_size, s_audio_size, s_spr_size, s_zoom_size;
static zcpu s_main, s_audio;
static uint8_t s_work[0x1000], s_charram[0x2000], s_vram[0x800], s_spr[0x80], s_scrollv[0x20], s_scrollh[0x100];
static uint8_t s_pal[0x80], s_misc_ram[0x20], s_lookup[0x2000], s_audio_ram[0x800], s_audio_4830;
static uint8_t s_bank, s_ctrl, s_bg_scrollx;
static uint16_t s_sl_bank;
static uint8_t s_char_dirty[256];
static uint8_t s_latch[2];
static int s_latch_pending[2], s_nmi_in1, s_nmi_line;
static taito68705_t s_mcu;
static int s_wheel;
// sound
static ay8910_t s_ay[2];
static msm5232_t s_msm;
static int s_ta[16], s_ay_vol[2], s_mute;
static uint32_t s_audio_irq_count;
static int32_t *s_mix, *s_tmp;
static int s_mix_cap, s_mix_n;
static uint32_t s_snd_done, s_slice_start, s_slice_len;
static unsigned long s_cpu_start;
static int s_cpu_len;
static uint64_t s_frac_main, s_frac_audio, s_frac_mcu, s_frac_snd;
// video (render side)
static uint8_t *s_sprgfx;   // 0x4000 lines of 16 pixels
static uint32_t s_nsprlines;
static uint8_t (*s_chars_hp)[256 * 64];
#define s_chars (*s_chars_hp)
static uint8_t s_snap_charram[0x2000], s_snap_vram[0x800], s_snap_spr[0x80], s_snap_scrollv[0x20], s_snap_scrollh[0x100];
static uint8_t s_snap_pal[0x80], s_snap_lookup[0x2000], s_snap_dirty[256];
static uint8_t s_snap_ctrl, s_snap_bg_scrollx;
// ports
static int s_p_dsw[3], s_p_in0, s_p_in1, s_p_pedal;

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
	int n = 0;
	for (int c = 0; c < 2; c++) {
		memset(s_tmp, 0, sizeof(int32_t) * (size_t)(ay8910_out_max(&s_ay[c], clk) < room ? ay8910_out_max(&s_ay[c], clk) : room));
		n = ay8910_render(&s_ay[c], clk, s_tmp, room);
		const int vol = s_ay_vol[c];
		for (int k = 0; k < n; k++)
			o[k] += s_tmp[k] * vol / 256;
	}
	msm5232_render(&s_msm, clk, o, room);
	if (s_mute)   // sound_enable_w: machine().sound().system_mute()
		memset(o, 0, sizeof(int32_t) * (size_t)n);
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

static void update_nmi(void)
{
	int line = s_latch_pending[0] && s_nmi_in1;   // INPUT_MERGER_ALL_HIGH
	if (line && !s_nmi_line)
		s_audio.z.nmi_pending = 1;
	s_nmi_line = line;
}

// ta7630_volbal_ay_w: VOL/BAL for the 7630 on this 8910 output
static void ay_pa_w(void *u, uint8_t data) { snd_sync(); *(int *)u = s_ta[data >> 4]; }
static void ay_pb_w(void *u, uint8_t data) { (void)u; (void)data; }   // TRBL/BASS (not emulated, as MAME)

// ---- main CPU ----
static uint8_t mcu_status_r(void)
{
	// bit 0 = MCU ready to receive, bit 1 = MCU has sent data
	return (uint8_t)((taito68705_host_semaphore_r(&s_mcu) ? 0 : 1) | (taito68705_mcu_semaphore_r(&s_mcu) ? 2 : 0));
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_main_size ? s_main_rom[a] : 0;
	if (a < 0x9000) return s_work[a & 0xfff];
	if (a < 0xa000) return 0;
	if (a < 0xc000) {   // mainbank: 6 x 0x2000 from 0x10000
		uint32_t o = 0x10000 + (uint32_t)s_bank * 0x2000 + (a & 0x1fff);
		return o < s_main_size ? s_main_rom[o] : 0;
	}
	if (a >= 0xc800 && a < 0xd000) return s_vram[a & 0x7ff];
	if ((a & 0xff00) == 0xd400) {
		switch (a & 3) {
		case 0: return taito68705_data_r(&s_mcu);
		case 1: return mcu_status_r();
		}
		return 0;
	}
	if ((a & 0xff00) == 0xd600) {
		switch (a & 0x1b) {
		case 0x00: return port8(s_p_dsw[0]);
		case 0x01: return port8(s_p_dsw[1]);
		case 0x02: return port8(s_p_dsw[2]);
		case 0x03: return port8(s_p_in0);
		case 0x08: return (uint8_t)s_wheel;
		case 0x09: return (uint8_t)((port8(s_p_in1) & 0x0f) | (port8(s_p_pedal) & 0xf0));   // pedal_in_r: PEDAL >> 4
		case 0x10: s_latch_pending[1] = 0; return s_latch[1];
		case 0x11: return (uint8_t)((s_latch_pending[1] ? 1 : 0) | (s_latch_pending[0] ? 0 : 2));   // sound_status_main_r
		}
		return 0;
	}
	if (a >= 0xd820 && a < 0xd840) return s_misc_ram[a - 0xd820];
	return 0;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x8000) return;
	if (a < 0x9000) { s_work[a & 0xfff] = v; return; }
	if (a < 0xa000) { s_lookup[(a & 0xfff) + s_sl_bank] = v; return; }   // sprite_lookup_w
	if (a < 0xc000) {   // chargen_w
		int o = a & 0x1fff;
		if (s_charram[o] != v) {
			s_charram[o] = v;
			s_char_dirty[(o / 8) & 0xff] = 1;
		}
		return;
	}
	if (a >= 0xc800 && a < 0xd000) { s_vram[a & 0x7ff] = v; return; }
	switch (a & 0xff00) {
	case 0xd100: s_ctrl = v; return;   // ctrl_w (flip y / x, /SN3OFF bg clip, sky, OJMODE sprite colour base)
	case 0xd200: if ((v & 7) < 6) s_bank = v & 7; return;   // bankswitch_w
	case 0xd300:
		if ((a & 7) == 3) s_sl_bank = (uint16_t)((v & 0x10) << 8);   // sprite_lookup_bank_w
		return;   // d300 watchdog, d304-d307 SCCON (nop)
	case 0xd400:
		if ((a & 3) == 0) taito68705_data_w(&s_mcu, v);
		return;
	case 0xd500:
		if (a < 0xd580) s_spr[a & 0x7f] = v;
		return;
	case 0xd600:
		if ((a & 0x1b) == 0x10) { s_latch[0] = v; s_latch_pending[0] = 1; update_nmi(); }
		return;
	case 0xd700:
		if (a < 0xd780) s_pal[a & 0x7f] = v;
		return;
	}
	if (a >= 0xd820 && a < 0xd840) { s_misc_ram[a - 0xd820] = v; return; }
	if (a >= 0xd840 && a < 0xd860) { s_scrollv[a - 0xd840] = v; return; }
	if ((a & 0xff00) == 0xdb00) { s_scrollh[a & 0xff] = v; return; }
	if (a == 0xdc06) { s_bg_scrollx = v; return; }
}

// ---- sound CPU ----
static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x4000) return a < s_audio_size ? s_audio_rom[a] : 0;
	if (a < 0x4800) return s_audio_ram[a & 0x7ff];
	if (a == 0x4830) return s_audio_4830;
	if (a == 0x5000) { s_latch_pending[0] = 0; update_nmi(); return s_latch[0]; }
	if (a == 0x5001) return (uint8_t)((s_latch_pending[1] ? 2 : 0) | (s_latch_pending[0] ? 0 : 1));   // sound_status_sound_r
	if (a >= 0xe000 && a < 0xf000) return a < s_audio_size ? s_audio_rom[a] : 0;
	return 0;
}

static void audio_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x4000) return;
	if (a < 0x4800) { s_audio_ram[a & 0x7ff] = v; return; }
	if (a >= 0x4800 && a <= 0x4803) { snd_sync(); ay8910_write(&s_ay[(a >> 1) & 1], a & 1, v); return; }
	if (a >= 0x4810 && a <= 0x481d) { snd_sync(); msm5232_write(&s_msm, a - 0x4810, v); return; }
	switch (a) {
	case 0x4820:   // ta7630_volbal_msm_w
		snd_sync();
		for (int i = 0; i < 8; i++) s_msm.gain[i] = s_ta[v >> 4];
		return;
	case 0x4830: s_audio_4830 = v; return;
	case 0x5000: s_latch[1] = v; s_latch_pending[1] = 1; return;
	case 0x5001: s_nmi_in1 = 1; update_nmi(); return;
	case 0x5002: s_nmi_in1 = 0; update_nmi(); return;
	case 0x5003: snd_sync(); s_mute = !(v & 1); return;   // sound_enable_w
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
// spritelayout: 16 x 1, 4 planes at RGN_FRAC(3,4) .. (0,4); pixels 0-7 from RGN_FRAC(1,8) + n, 8-15 from n, bit 0 first
static void decode_sprites(void)
{
	const uint32_t q = s_spr_size / 4, e = s_spr_size / 8;
	for (uint32_t n = 0; n < s_nsprlines; n++)
		for (int x = 0; x < 16; x++) {
			uint32_t o = (x < 8 ? e : 0) + n;
			int b = x & 7, v = 0;
			for (int p = 3; p >= 0; p--)
				v = v << 1 | ((s_spr_rom[q * (uint32_t)p + o] >> b) & 1);
			s_sprgfx[n * 16 + (uint32_t)x] = (uint8_t)v;
		}
}

// charlayout: 8 x 8, planes at 0x1800 / 0x1000 / 0x800 / 0 bytes (MSB first), bit 0 = leftmost pixel
static void decode_char(int c)
{
	for (int y = 0; y < 8; y++) {
		int o = c * 8 + y;
		uint8_t p3 = s_snap_charram[0x1800 + o], p2 = s_snap_charram[0x1000 + o], p1 = s_snap_charram[0x800 + o], p0 = s_snap_charram[o];
		for (int x = 0; x < 8; x++)
			s_chars[c * 64 + y * 8 + x] = (uint8_t)(((p3 >> x) & 1) << 3 | ((p2 >> x) & 1) << 2 | ((p1 >> x) & 1) << 1 | ((p0 >> x) & 1));
	}
}

static void draw_sprites(uint16_t *bm, int pitch, int flipx, int flipy)
{
	const int color_base = (s_snap_ctrl & 0x10) ? 0x10 : 0x30;
	for (int offs = 0; offs < 0x80; offs += 4) {
		const uint8_t *s = s_snap_spr + offs;
		int sx = s[3] - ((s[2] & 0x80) << 1);
		int sy = 256 - 64 - s[0] + ((s[1] & 0x80) << 1);
		int flip = s[1] & 0x40;
		int zoom = s[1] & 0x3f;
		const uint8_t *zoomy = s_zoom + (zoom << 6);
		const uint8_t *zoomx = s_zoom + 0x2000 + (zoom << 3);
		const uint8_t *lookup = s_snap_lookup + ((s[2] & 0x7f) << 6);
		for (int y = 0; y < 64; y++) {
			int dy = flipy ? (255 - sy - y) : (sy + y);
			if (dy < VIS_Y0 || dy > VIS_Y1)
				continue;
			uint16_t *d = bm + (dy - VIS_Y0) * pitch;
			int charline = zoomy[y] & 0x07;
			int base_pos = zoomy[y] & 0x38;
			if (flip)
				base_pos ^= 0x38;
			int px = 0;
			for (int ch = 0; ch < 4; ch++) {
				int pos = base_pos + 2 * ch;
				uint32_t code = 8u * (uint32_t)(lookup[pos] | ((lookup[pos + 1] & 0x07) << 8));
				int realflipy = (lookup[pos + 1] & 0x80) ? !flip : flip;
				code += (uint32_t)(realflipy ? (charline ^ 7) : charline);
				const uint8_t *pen = s_sprgfx + (rbx_wrap(code, s_nsprlines)) * 16;
				for (int x = 0; x < 16; x++) {
					int col = pen[x];
					if (col) {
						int dx = flipx ? (255 - sx - px) : (sx + px);
						if (dx >= 0 && dx < SCREEN_W)
							d[dx] = (uint16_t)(color_base + col);
					}
					if (zoomx[7 - (2 * ch + x / 8)] & (1 << (x & 7)))   // "almost certainly wrong" (MAME)
						px++;
				}
			}
		}
	}
}

static void bc_render(void)
{
	for (int i = 0; i < 64; i++) {   // xRGB_444, big endian
		int c = s_snap_pal[i * 2] << 8 | s_snap_pal[i * 2 + 1];
		rbx_video.palette[i] = (uint32_t)(((c >> 8) & 15) * 17 << 16 | ((c >> 4) & 15) * 17 << 8 | (c & 15) * 17);
	}
	const uint32_t bg_rgb = rbx_video.palette[BG_PEN];
	for (int g = 0; g < 256; g++)
		rbx_video.palette[SKY_BASE + g] = (bg_rgb & 0xff0000) | (uint32_t)g << 8 | 0xff;
	for (int c = 0; c < 256; c++)
		if (s_snap_dirty[c]) {
			s_snap_dirty[c] = 0;
			decode_char(c);
		}
	const int flipy = s_snap_ctrl & 1, flipx = (s_snap_ctrl >> 1) & 1;
	const int clip_on = s_snap_ctrl & 4, sky_on = s_snap_ctrl & 8;
	const int bgx = s_snap_bg_scrollx;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	// draw_bg: tmp0 = opaque tilemap (colour 2), tmp1(x, y) = tmp0(x - xscroll, y - colscroll[that source column]),
	// dest(x, y) = tmp1(x - rowscroll[y], y), pen 0x20 transparent; clip off the 64 rightmost columns when /SN3OFF is set
	int xmin = 0, xmax = SCREEN_W - 1;
	if (clip_on) {
		if (flipx) xmin += 64;
		else xmax -= 64;
	}
	for (int y = VIS_Y0; y <= VIS_Y1; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		for (int x = 0; x < SCREEN_W; x++)
			d[x] = BG_PEN;
		const int hs = s_snap_scrollh[y];
		for (int x = xmin; x <= xmax; x++) {
			int u = (x + hs) & 0xff;
			int sx = (u + bgx - 0x12) & 0xff;
			int sy = (y + s_snap_scrollv[sx >> 3]) & 0xff;
			int tx = flipx ? 255 - sx : sx, ty = flipy ? 255 - sy : sy;
			int code = s_snap_vram[0x400 + (ty >> 3) * 32 + (tx >> 3)];
			uint8_t p = s_chars[code * 64 + (ty & 7) * 8 + (tx & 7)];
			if (p)
				d[x] = (uint16_t)(0x20 + p);
		}
	}
	draw_sprites(bm, pitch, flipx, flipy);
	// draw_fg: colour 0, pen 0 transparent
	for (int y = VIS_Y0; y <= VIS_Y1; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		int ty = flipy ? 255 - y : y;
		for (int x = 0; x < SCREEN_W; x++) {
			int tx = flipx ? 255 - x : x;
			int code = s_snap_vram[(ty >> 3) * 32 + (tx >> 3)];
			uint8_t p = s_chars[code * 64 + (ty & 7) * 8 + (tx & 7)];
			if (p)
				d[x] = p;
		}
	}
	if (sky_on) {   // blend the sky gradient into the background pen
		const int g0 = (int)((bg_rgb >> 8) & 0xff) + 0x200 - (int8_t)bgx;
		for (int y = VIS_Y0; y <= VIS_Y1; y++) {
			uint16_t *d = bm + (y - VIS_Y0) * pitch;
			const int gy = g0 - (int8_t)s_snap_scrollh[y];
			for (int x = 0; x < SCREEN_W; x++)
				if (d[x] == BG_PEN) {
					int g = (gy - x) / 2;
					if (g < 0) g = 0;
					if (g > 255) g = 255;
					d[x] = (uint16_t)(SKY_BASE + g);
				}
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap_charram, s_charram, sizeof(s_charram));
	memcpy(s_snap_vram, s_vram, sizeof(s_vram));
	memcpy(s_snap_spr, s_spr, sizeof(s_spr));
	memcpy(s_snap_scrollv, s_scrollv, sizeof(s_scrollv));
	memcpy(s_snap_scrollh, s_scrollh, sizeof(s_scrollh));
	memcpy(s_snap_pal, s_pal, sizeof(s_pal));
	memcpy(s_snap_lookup, s_lookup, sizeof(s_lookup));
	for (int i = 0; i < 256; i++) {
		s_snap_dirty[i] |= s_char_dirty[i];
		s_char_dirty[i] = 0;
	}
	s_snap_ctrl = s_ctrl;
	s_snap_bg_scrollx = s_bg_scrollx;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void bc_reset(void)
{
	// (first: the sound syncs read them, from a CPU write or a port callback of a chip reset below, before a frame
	// sets them; the last game's values made a render of billions of clocks, the next game of the board hung)
	s_slice_start = s_slice_len = s_cpu_start = s_cpu_len = 0;
	memset(s_work, 0, sizeof(s_work));
	memset(s_charram, 0, sizeof(s_charram));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_scrollv, 0, sizeof(s_scrollv));
	memset(s_scrollh, 0, sizeof(s_scrollh));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_misc_ram, 0, sizeof(s_misc_ram));
	memset(s_lookup, 0, sizeof(s_lookup));
	memset(s_audio_ram, 0, sizeof(s_audio_ram));
	memset(s_char_dirty, 1, sizeof(s_char_dirty));
	s_audio_4830 = 0;
	memset(&s_main, 0, sizeof(s_main));
	memset(&s_audio, 0, sizeof(s_audio));
	zcpu_init(&s_main, main_read, main_write);
	zcpu_init(&s_audio, audio_read, audio_write);
	s_bank = 0;
	s_ctrl = 0;
	s_bg_scrollx = 0;
	s_sl_bank = 0;
	s_latch[0] = s_latch[1] = 0;
	s_latch_pending[0] = s_latch_pending[1] = 0;
	s_nmi_in1 = s_nmi_line = 0;
	s_wheel = 0;
	taito68705_reset(&s_mcu);
	for (int c = 0; c < 2; c++) {
		ay8910_reset(&s_ay[c]);
		s_ay_vol[c] = 256;
	}
	msm5232_reset(&s_msm);
	for (int i = 0; i < 8; i++)
		s_msm.gain[i] = 256;
	s_mute = 0;
	s_audio_irq_count = 0;
	s_frac_main = s_frac_audio = s_frac_mcu = s_frac_snd = 0;
	s_snd_done = 0;
	s_mix_n = 0;
}

static void bc_exit(void)
{
	free(s_sprgfx);
	s_sprgfx = NULL;
	free(s_mix);
	free(s_tmp);
	s_mix = s_tmp = NULL;
	free(s_chars_hp);
	s_chars_hp = NULL;
}

static int bc_supported(const char *machine) { return !strcmp(machine, "buggychl"); }

static int bc_init(const rbx_game *g)
{
	if (!s_chars_hp && !(s_chars_hp = calloc(1, sizeof(*s_chars_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!bc_supported(g->machine)) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_main_rom = rbx_region("maincpu", &s_main_size);
	s_audio_rom = rbx_region("audiocpu", &s_audio_size);
	s_spr_rom = rbx_region("sprites", &s_spr_size);
	s_zoom = rbx_region("sprite_zoom_table", &s_zoom_size);
	if (!s_main_rom || !s_audio_rom || !s_spr_rom || s_spr_size < 0x20000 || !s_zoom || s_zoom_size < 0x4000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	uint32_t msize;
	uint8_t *mrom = rbx_region("bmcu:mcu", &msize);
	memset(&s_mcu, 0, sizeof(s_mcu));
	if (!mrom || msize < 0x800 || taito68705_init(&s_mcu, TAITO68705_STD, mrom, 0x800)) {
		rbx_fail("%s: MCU ROM missing", g->set);
		return -1;
	}
	s_p_dsw[0] = rbx_port_find("DSW1");
	s_p_dsw[1] = rbx_port_find("DSW2");
	s_p_dsw[2] = rbx_port_find("DSW3");
	s_p_in0 = rbx_port_find("IN0");
	s_p_in1 = rbx_port_find("IN1");
	s_p_pedal = rbx_port_find("PEDAL");
	s_nsprlines = s_spr_size / 8;
	s_sprgfx = (uint8_t *)malloc((size_t)s_nsprlines * 16);
	if (!s_sprgfx)
		return -1;
	decode_sprites();
	ta7630_table(s_ta);
	for (int c = 0; c < 2; c++) {   // YM2149 8 MHz / 4, route 0.50
		ay8910_init(&s_ay[c], AY8910_TYPE_YM2149, SND_CLOCK, AY8910_LEGACY_OUTPUT, 128, 4, 1);
		s_ay[c].port_a_write = ay_pa_w;
		s_ay[c].port_b_write = ay_pb_w;
		s_ay[c].user = &s_ay_vol[c];
	}
	static const double caps[8] = { 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6 };
	msm5232_init(&s_msm, SND_CLOCK, caps, 2);
	uint32_t rate = ay8910_rate(&s_ay[0]);
	s_mix_cap = (int)(rate / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_tmp = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || !s_tmp || rbx_audio_setup(rate) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 512)) {
		bc_exit();
		return -1;
	}
	memset(s_snap_dirty, 1, sizeof(s_snap_dirty));
	bc_reset();
	return 0;
}

static void bc_frame(void)
{
	int wd = rbx_analog(1, RBX_IPT_PADDLE);
	if (!wd) {   // no device: the d-pad turns the wheel (4 counts per frame)
		uint32_t in = rbx_input(1);
		wd = ((in & RBX_IN_RIGHT) ? 4 : 0) - ((in & RBX_IN_LEFT) ? 4 : 0);
	}
	s_wheel = (s_wheel - wd) & 0xff;   // WHEEL: IPT_DIAL PORT_REVERSE
	for (int i = 0; i < LINES; i++) {
		int line = (VIS_Y1 + 1 + i) % LINES;
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			s_main.irq = 1;   // irq0_line_hold
		}
		int mc = rbx_line_cycles(&s_frac_main, MAIN_CLOCK, RATE_MHZ, LINES);
		int ac = rbx_line_cycles(&s_frac_audio, AUDIO_CLOCK, RATE_MHZ, LINES);
		int uc = rbx_line_cycles(&s_frac_mcu, MCU_CLOCK, RATE_MHZ, LINES);
		int sc = rbx_line_cycles(&s_frac_snd, SND_CLOCK, RATE_MHZ, LINES);
		zcpu_run(&s_main, mc);
		s_slice_start = s_snd_done;
		s_slice_len = (uint32_t)sc;
		s_cpu_start = s_audio.z.cyc;
		s_cpu_len = ac;
		s_audio_irq_count += (uint32_t)ac;
		if (s_audio_irq_count >= AUDIO_IRQ_PERIOD) {
			s_audio_irq_count -= AUDIO_IRQ_PERIOD;
			s_audio.irq = 1;
		}
		zcpu_run(&s_audio, ac);
		taito68705_execute(&s_mcu, uc);
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

static int bc_state(void *ctx, int load)
{
	int rc = 0;
	rc |= zcpu_state(ctx, "main", &s_main, load);
	rc |= zcpu_state(ctx, "audio", &s_audio, load);
	rc |= rbx_state_io(ctx, "work", s_work, sizeof(s_work));
	rc |= rbx_state_io(ctx, "charram", s_charram, sizeof(s_charram));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "scrollv", s_scrollv, sizeof(s_scrollv));
	rc |= rbx_state_io(ctx, "scrollh", s_scrollh, sizeof(s_scrollh));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "mram", s_misc_ram, sizeof(s_misc_ram));
	rc |= rbx_state_io(ctx, "lookup", s_lookup, sizeof(s_lookup));
	rc |= rbx_state_io(ctx, "aram", s_audio_ram, sizeof(s_audio_ram));
	rc |= rbx_state_io(ctx, "tmcu", &s_mcu.st, sizeof(s_mcu.st));
	rc |= rbx_state_io(ctx, "tmcu6805", &s_mcu.mcu.st, sizeof(s_mcu.mcu.st));
	rc |= rbx_state_io(ctx, "ay0", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "ay1", &s_ay[1], ay8910_state_size());
	rc |= rbx_state_io(ctx, "msm", &s_msm, msm5232_state_size());
	int32_t misc[16] = { s_bank, s_ctrl, s_bg_scrollx, s_sl_bank, s_latch[0], s_latch[1], s_latch_pending[0],
		s_latch_pending[1], s_nmi_in1, s_nmi_line, s_ay_vol[0], s_ay_vol[1], s_mute, (int32_t)s_audio_irq_count,
		s_wheel, s_audio_4830 };
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (rc)
		return -1;
	if (load) {
		s_bank = (uint8_t)misc[0]; s_ctrl = (uint8_t)misc[1]; s_bg_scrollx = (uint8_t)misc[2]; s_sl_bank = (uint16_t)misc[3];
		s_latch[0] = (uint8_t)misc[4]; s_latch[1] = (uint8_t)misc[5]; s_latch_pending[0] = misc[6];
		s_latch_pending[1] = misc[7]; s_nmi_in1 = misc[8]; s_nmi_line = misc[9]; s_ay_vol[0] = misc[10];
		s_ay_vol[1] = misc[11]; s_mute = misc[12]; s_audio_irq_count = (uint32_t)misc[13]; s_wheel = misc[14];
		s_audio_4830 = (uint8_t)misc[15];
		memset(s_char_dirty, 1, sizeof(s_char_dirty));
	}
	return 0;
}

static int bc_save(void *ctx) { return bc_state(ctx, 0); }
static int bc_load(void *ctx) { return bc_state(ctx, 1); }

const rbx_driver rbx_drv_buggychl = { "Buggy Challenge", bc_init, bc_exit, bc_reset, bc_frame, bc_save, bc_load, bc_supported, bc_render };
