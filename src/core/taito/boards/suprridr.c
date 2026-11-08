/*  Super Rider hardware, ported from MAME src/mame/taito/suprridr.cpp (license BSD-3-Clause, copyright-holders:
    Aaron Giles). Memory and I/O maps, the vblank NMI (enable register) of the main CPU, the vblank IRQ (HOLD_LINE)
    of the sound CPU, the screen-flip control multiplexer, the bg tilemap with the unscrolled side columns, the fg
    tilemap and the sprites follow MAME. Sound: two AY-3-8910 at 18.432 MHz / 12 (sound/ay8910).
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
#define MAIN_LINE 200           // 3.072 MHz
#define AUDIO_CLOCK 2500000     // 10 MHz / 4
#define AY_LINE 100             // 1.536 MHz
#define AY_CLOCK 1536000

typedef struct {
	z80 z;
	int irq;
} zcpu;

static uint8_t *s_main_rom, *s_audio_rom;
static uint32_t s_main_size, s_audio_size;
static uint8_t s_ram[0x800], s_bgram[0x400], s_fgram[0x800], s_ram98[0x400], s_audio_ram[0x400];
static zcpu s_main, s_audio;
static uint8_t s_nmi_enable, s_flipx, s_flipy, s_fgdisable, s_fgscroll, s_bgscroll, s_soundlatch;
static ay8910_t s_ay[2];
static uint64_t s_audio_frac;
static unsigned long s_audio_start;
static int s_audio_len;
// video
static uint8_t *s_bgtiles, *s_fgtiles, *s_sprites;
static uint32_t s_pal_rgb[96];
typedef struct {
	uint8_t bgram[0x400], fgram[0x400], spr[0xc0];
	uint8_t flipx, flipy, fgdisable, fgscroll, bgscroll;
} snap_t;
static snap_t s_snap;
// ports
static int s_port_system, s_port_dsw, s_port_contp[2];
// sound
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static uint32_t s_ay_done, s_line_ay;

static void ay_run_to(uint32_t pos)
{
	if (pos <= s_ay_done)
		return;
	int n = ay8910_render(&s_ay[0], pos - s_ay_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	ay8910_render(&s_ay[1], pos - s_ay_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	s_mix_n += n;
	s_ay_done = pos;
}
static void ay_sync(void)
{
	if (s_audio_len <= 0)
		return;
	uint32_t done = (uint32_t)(s_audio.z.cyc - s_audio_start);
	if (done > (uint32_t)s_audio_len)
		done = (uint32_t)s_audio_len;
	ay_run_to(s_line_ay + AY_LINE * done / (uint32_t)s_audio_len);
}
static uint8_t ay2_port_a(void *u) { (void)u; return s_soundlatch; }

static uint8_t port8(int p) { return p >= 0 ? (uint8_t)rbx_port_read(p) : 0; }

// ---- main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000 || (a >= 0xc000 && a < 0xf000)) return s_main_rom[a];
	if (a < 0x8800) return s_ram[a & 0x7ff];
	if (a < 0x8c00) return s_bgram[a & 0x3ff];
	if (a >= 0x9000 && a < 0x9800) return s_fgram[a & 0x7ff];
	if (a >= 0x9800 && a < 0x9c00) return s_ram98[a & 0x3ff];
	switch (a) {
	case 0xa000: return port8(s_port_contp[s_flipx]);   // control_r: the screen flip multiplexes the controls
	case 0xa800: return port8(s_port_system);
	case 0xb000: return port8(s_port_dsw);
	}
	return 0;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x8000) return;
	if (a < 0x8800) { s_ram[a & 0x7ff] = v; return; }
	if (a < 0x8c00) { s_bgram[a & 0x3ff] = v; return; }
	if (a >= 0x9000 && a < 0x9800) { s_fgram[a & 0x7ff] = v; return; }
	if (a >= 0x9800 && a < 0x9c00) { s_ram98[a & 0x3ff] = v; return; }
	switch (a) {
	case 0xb000: s_nmi_enable = v; break;
	case 0xb006: s_flipx = v & 1; break;
	case 0xb007: s_flipy = v & 1; break;
	case 0xb800: s_soundlatch = v; break;
	case 0xc801: s_fgdisable = v & 1; break;
	case 0xc802: s_fgscroll = v; break;
	case 0xc804: s_bgscroll = v; break;
	}
}

// ---- sound CPU ----
static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x1000) return s_audio_rom[a];
	if (a >= 0x3800 && a < 0x3c00) return s_audio_ram[a & 0x3ff];
	return 0;
}
static void audio_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x3800 && a < 0x3c00)
		s_audio_ram[a & 0x3ff] = v;
}
static uint8_t audio_in(z80 *z, uint8_t p)
{
	(void)z;
	if (p == 0x8d) { ay_sync(); return ay8910_data_r(&s_ay[0]); }
	if (p == 0x8f) { ay_sync(); return ay8910_data_r(&s_ay[1]); }
	return 0;
}
static void audio_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	switch (p) {
	case 0x00: s_soundlatch = 0; break;
	case 0x8c: case 0x8d: ay_sync(); ay8910_write(&s_ay[0], p & 1, v); break;
	case 0x8e: case 0x8f: ay_sync(); ay8910_write(&s_ay[1], p & 1, v); break;
	}
}
static uint8_t main_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0; }   // port 0: watchdog
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// ---- video ----
static void sr_render(void)
{
	const snap_t *s = &s_snap;
	memcpy(rbx_video.palette, s_pal_rgb, sizeof(s_pal_rgb));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const int fx = s->flipx, fy = s->flipy;
	// bg: the left 4 columns (1 when flipped) and the right column (4 when flipped) are not scrolled
	const int lcols = fx ? 8 : 32, rcols = fx ? 32 : 8;
	for (int sy = VIS_Y0; sy <= VIS_Y1; sy++) {
		uint16_t *d = bm + (sy - VIS_Y0) * pitch;
		int ny = fy ? 255 - sy : sy;
		for (int sx = 0; sx < SCREEN_W; sx++) {
			int nx = fx ? 255 - sx : sx;
			int scroll = (sx < lcols || sx >= SCREEN_W - rcols) ? 0 : s->bgscroll;
			int ty = (ny + scroll) & 255;
			d[sx] = s_bgtiles[s->bgram[(ty >> 3) * 32 + (nx >> 3)] * 64 + (ty & 7) * 8 + (nx & 7)];
		}
		if (!s->fgdisable) {
			int ty = (ny + s->fgscroll) & 255;
			for (int sx = 0; sx < SCREEN_W; sx++) {
				int nx = fx ? 255 - sx : sx;
				uint8_t p = s_fgtiles[s->fgram[(ty >> 3) * 32 + (nx >> 3)] * 64 + (ty & 7) * 8 + (nx & 7)];
				if (p)
					d[sx] = (uint16_t)(32 + p);
			}
		}
	}
	for (int i = 0; i < 48; i++) {
		const uint8_t *e = s->spr + i * 4;
		int code = (e[1] & 0x3f) | ((e[2] >> 1) & 0x40);
		int color = (e[2] & 0x7f) % 2;
		int flx = e[1] & 0x40, fly = e[1] & 0x80;
		int x = e[3], y = 240 - e[0];
		if (fx) { flx = !flx; x = 240 - x; }
		if (fy) { fly = !fly; y = 240 - y; }
		const uint8_t *src = s_sprites + code * 256;
		uint16_t base = (uint16_t)(64 + color * 8);
		for (int yy = 0; yy < 16; yy++) {
			int dy = y + yy;
			if (dy < VIS_Y0 || dy > VIS_Y1)
				continue;
			const uint8_t *r = src + (fly ? 15 - yy : yy) * 16;
			uint16_t *d = bm + (dy - VIS_Y0) * pitch;
			for (int xx = 0; xx < 16; xx++) {
				int dx = x + xx;
				uint8_t p = r[flx ? 15 - xx : xx];
				if (p && dx >= 0 && dx < SCREEN_W)
					d[dx] = (uint16_t)(base + p);
			}
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap.bgram, s_bgram, sizeof(s_bgram));
	memcpy(s_snap.fgram, s_fgram, sizeof(s_snap.fgram));
	memcpy(s_snap.spr, s_ram98 + 0x40, sizeof(s_snap.spr));
	s_snap.flipx = s_flipx;
	s_snap.flipy = s_flipy;
	s_snap.fgdisable = s_fgdisable;
	s_snap.fgscroll = s_fgscroll;
	s_snap.bgscroll = s_bgscroll;
	rbx_video_frame_done();
}

// ---- init / frame ----
static int gfx_bit(const uint8_t *gfx, uint32_t size, uint32_t bit)
{
	uint32_t byte = bit >> 3;
	return byte < size ? (gfx[byte] >> (7 - (bit & 7))) & 1 : 0;
}

// MAME gfx_layout decode (offsets in bits, the first plane is the most significant bit)
static void decode(const uint8_t *gfx, uint32_t size, const uint32_t *planes, int np, const uint32_t *xo,
                   const uint32_t *yo, int w, uint32_t inc, int count, uint8_t *out)
{
	for (int t = 0; t < count; t++)
		for (int y = 0; y < w; y++)
			for (int x = 0; x < w; x++) {
				int v = 0;
				for (int p = 0; p < np; p++)
					v = v << 1 | gfx_bit(gfx, size, planes[p] + (uint32_t)t * inc + yo[y] + xo[x]);
				out[(size_t)t * (size_t)(w * w) + (size_t)(y * w + x)] = (uint8_t)v;
			}
}

static void zcpu_init(zcpu *c, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t),
                      uint8_t (*in)(z80 *, uint8_t), void (*out)(z80 *, uint8_t, uint8_t))
{
	z80_init(&c->z);
	c->z.read_byte = rd;
	c->z.write_byte = wr;
	c->z.port_in = in;
	c->z.port_out = out;
	c->z.int_data = 0xff;
	c->irq = 0;
}

static void sr_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_bgram, 0, sizeof(s_bgram));
	memset(s_fgram, 0, sizeof(s_fgram));
	memset(s_ram98, 0, sizeof(s_ram98));
	memset(s_audio_ram, 0, sizeof(s_audio_ram));
	zcpu_init(&s_main, main_read, main_write, main_in, no_out);
	zcpu_init(&s_audio, audio_read, audio_write, audio_in, audio_out);
	s_nmi_enable = s_flipx = s_flipy = s_fgdisable = s_fgscroll = s_bgscroll = s_soundlatch = 0;
	s_audio_frac = 0;
	ay8910_reset(&s_ay[0]);
	ay8910_reset(&s_ay[1]);
	s_ay_done = 0;
	s_mix_n = 0;
}

static void sr_exit(void)
{
	free(s_bgtiles);
	free(s_fgtiles);
	free(s_sprites);
	free(s_mix);
	s_bgtiles = s_fgtiles = s_sprites = NULL;
	s_mix = NULL;
}

static int sr_init(const rbx_game *g)
{
	uint32_t bs = 0, fs = 0, ss = 0, ps = 0;
	s_main_rom = rbx_region("maincpu", &s_main_size);
	s_audio_rom = rbx_region("audiocpu", &s_audio_size);
	uint8_t *bg = rbx_region("bgtiles", &bs), *fg = rbx_region("fgtiles", &fs), *sp = rbx_region("sprites", &ss);
	uint8_t *prom = rbx_region("proms", &ps);
	if (!s_main_rom || s_main_size < 0x10000 || !s_audio_rom || s_audio_size < 0x1000 || !bg || bs < 0x2000 || !fg ||
	    fs < 0x2000 || !sp || ss < 0x3000 || !prom || ps < 0x60) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_bgtiles = (uint8_t *)malloc(256 * 64);
	s_fgtiles = (uint8_t *)malloc(256 * 64);
	s_sprites = (uint8_t *)malloc(128 * 256);
	if (!s_bgtiles || !s_fgtiles || !s_sprites) {
		sr_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	// charlayout: 4 planes { RGN_FRAC(1,2), RGN_FRAC(1,2)+4, 0, 4 }, x { STEP4(0,1), STEP4(8,1) }, y STEP8(0,16)
	static const uint32_t cx[8] = { 0, 1, 2, 3, 8, 9, 10, 11 };
	static const uint32_t cy[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
	const uint32_t cp[4] = { 0x1000 * 8, 0x1000 * 8 + 4, 0, 4 };
	decode(bg, bs, cp, 4, cx, cy, 8, 128, 256, s_bgtiles);
	decode(fg, fs, cp, 4, cx, cy, 8, 128, 256, s_fgtiles);
	// spritelayout: 3 planes RGN_FRAC(0,3) .. (2,3), 128 sprites
	static const uint32_t sx[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 64, 65, 66, 67, 68, 69, 70, 71 };
	static const uint32_t sy[16] = { 0, 8, 16, 24, 32, 40, 48, 56, 128, 136, 144, 152, 160, 168, 176, 184 };
	const uint32_t spl[3] = { 0, 0x1000 * 8, 0x2000 * 8 };
	decode(sp, ss, spl, 3, sx, sy, 16, 256, 128, s_sprites);
	for (int i = 0; i < 96; i++) {
		uint8_t c = prom[i];
		int r = 0x21 * (c & 1) + 0x47 * ((c >> 1) & 1) + 0x97 * ((c >> 2) & 1);
		int gg = 0x21 * ((c >> 3) & 1) + 0x47 * ((c >> 4) & 1) + 0x97 * ((c >> 5) & 1);
		int b = 0x52 * ((c >> 6) & 1) + 0xad * ((c >> 7) & 1);
		s_pal_rgb[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	s_port_system = rbx_port_find("SYSTEM");
	s_port_dsw = rbx_port_find("DSW");
	s_port_contp[0] = rbx_port_find("CONTP1");
	s_port_contp[1] = rbx_port_find("CONTP2");
	// two AY-3-8910 at 1.536 MHz, MAME route 0.25 each (legacy output), 48 kHz
	for (int i = 0; i < 2; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 64, 4, 1);
	s_ay[1].port_a_read = ay2_port_a;
	s_mix_cap = (int)(ay8910_rate(&s_ay[0]) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(ay8910_rate(&s_ay[0])) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 96)) {
		sr_exit();
		return -1;
	}
	sr_reset();
	return 0;
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

static void sr_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			if (s_nmi_enable)
				z80_gen_nmi(&s_main.z);   // main_nmi_gen: pulse
			s_audio.irq = 1;              // irq0_line_hold
		}
		zcpu_run(&s_main, MAIN_LINE);
		s_line_ay = s_ay_done;
		s_audio_len = rbx_line_cycles(&s_audio_frac, AUDIO_CLOCK, LINE_RATE * 1000, 1);
		s_audio_start = s_audio.z.cyc;
		zcpu_run(&s_audio, s_audio_len);
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

static int sr_state(void *ctx, int load)
{
	int rc = 0;
	rc |= zcpu_state(ctx, "main", &s_main, load);
	rc |= zcpu_state(ctx, "audio", &s_audio, load);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "bgram", s_bgram, sizeof(s_bgram));
	rc |= rbx_state_io(ctx, "fgram", s_fgram, sizeof(s_fgram));
	rc |= rbx_state_io(ctx, "ram98", s_ram98, sizeof(s_ram98));
	rc |= rbx_state_io(ctx, "audram", s_audio_ram, sizeof(s_audio_ram));
	uint8_t regs[7] = { s_nmi_enable, s_flipx, s_flipy, s_fgdisable, s_fgscroll, s_bgscroll, s_soundlatch };
	rc |= rbx_state_io(ctx, "regs", regs, sizeof(regs));
	rc |= rbx_state_io(ctx, "afrac", &s_audio_frac, sizeof(s_audio_frac));
	rc |= rbx_state_io(ctx, "ay1", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "ay2", &s_ay[1], ay8910_state_size());
	if (rc)
		return -1;
	if (load) {
		s_nmi_enable = regs[0]; s_flipx = regs[1]; s_flipy = regs[2]; s_fgdisable = regs[3];
		s_fgscroll = regs[4]; s_bgscroll = regs[5]; s_soundlatch = regs[6];
	}
	return 0;
}

static int sr_save(void *ctx) { return sr_state(ctx, 0); }
static int sr_load(void *ctx) { return sr_state(ctx, 1); }
static int sr_supported(const char *machine) { return !strcmp(machine, "suprridr"); }

const rbx_driver rbx_drv_suprridr = { "Super Rider", sr_init, sr_exit, sr_reset, sr_frame, sr_save, sr_load,
                                      sr_supported, sr_render };
