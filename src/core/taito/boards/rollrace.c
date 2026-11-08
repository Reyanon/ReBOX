/*  Fighting Roller / Roller Aces hardware, ported from MAME src/mame/taito/rollrace.cpp (license BSD-3-Clause,
    copyright-holders: Pierpaolo Prazzoli). Memory maps, the LS259 main latch, the vblank NMI (mask), the 240 Hz
    sound NMI (mask), the fake protection read at d900, the road layer, the 32x32 sprites and the column-scrolled
    fg tilemap follow MAME. Sound: three AY-3-8910 at 1.5 MHz on a stereo speaker as in MAME (ay1 + ay2 right,
    ay3 left, 0.10 each); the analog sound effects (ec00-ec0f) are missing as in MAME.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"

#include <stdlib.h>
#include <string.h>

// MAME: 60 Hz, 256 x 256, visible y 16-239 (256 lines here, 15360 Hz)
#define LINES 256
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256
#define LINE_RATE 15360
#define MAIN_CLOCK 3000000      // 24 MHz / 8
#define AUDIO_CLOCK 1500000     // 24 MHz / 16, also the AY clock
#define SNMI_LINES 64           // 4 * 60 Hz

typedef struct {
	z80 z;
} zcpu;

static uint8_t *s_main_rom, *s_audio_rom, *s_layout;
static uint32_t s_main_size, s_audio_size;
static uint8_t s_ram[0x1000], s_videoram[0x400], s_colorram[0x80], s_spriteram[0x100], s_audio_ram[0x1000];
static zcpu s_main, s_audio;
static uint8_t s_q[8];
static uint8_t s_bkgpage, s_bkgflip, s_bkgpen, s_bkgcol, s_flipy, s_soundlatch, s_snmi_mask;
static int s_nmi_line;
static ay8910_t s_ay[3];
static uint64_t s_main_frac, s_audio_frac;
static unsigned long s_audio_start;
// video
static uint8_t *s_chars, *s_road, *s_sprites;   // 4 x 256 8x8, 1024 8x8, 3 x 64 32x32
static uint32_t s_pal_rgb[256];
typedef struct {
	uint8_t videoram[0x400], colorram[0x80], spriteram[0x80];
	uint8_t bkgpage, bkgflip, bkgpen, bkgcol, flipx, flipy, chrbank, spritebank;
} snap_t;
static snap_t s_snap;
// ports
static int s_port_p1, s_port_p2, s_port_system, s_port_dsw1, s_port_dsw2;
// sound
static int32_t *s_mixl, *s_mixr;
static int s_mix_cap, s_mix_n;
static uint32_t s_ay_done, s_line_ay;

static void ay_run_to(uint32_t pos)
{
	if (pos <= s_ay_done)
		return;
	uint32_t n = pos - s_ay_done;
	int cap = s_mix_cap - s_mix_n;
	int k = ay8910_render(&s_ay[2], n, s_mixl + s_mix_n, cap);
	ay8910_render(&s_ay[0], n, s_mixr + s_mix_n, cap);
	ay8910_render(&s_ay[1], n, s_mixr + s_mix_n, cap);
	s_mix_n += k;
	s_ay_done = pos;
}
static void ay_sync(void) { ay_run_to(s_line_ay + (uint32_t)(s_audio.z.cyc - s_audio_start)); }

static uint8_t port8(int p) { return p >= 0 ? (uint8_t)rbx_port_read(p) : 0; }

// ---- main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xa000) return a < s_main_size ? s_main_rom[a] : 0;
	if (a >= 0xc000 && a < 0xd000) return s_ram[a & 0xfff];
	if (a >= 0xe000 && a < 0xe400) return s_videoram[a & 0x3ff];
	if (a >= 0xe400 && a < 0xe480) return s_colorram[a & 0x7f];
	if (a >= 0xf000 && a < 0xf100) return s_spriteram[a & 0xff];
	switch (a) {
	case 0xd900: return 0x51;   // fake_d900_r (protection?)
	case 0xf800: return port8(s_port_p1);
	case 0xf801: return port8(s_port_p2);
	case 0xf802: return port8(s_port_system);
	case 0xf804: return port8(s_port_dsw1);
	case 0xf805: return port8(s_port_dsw2);
	}
	return 0;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xd000) { s_ram[a & 0xfff] = v; return; }
	if (a >= 0xe000 && a < 0xe400) { s_videoram[a & 0x3ff] = v; return; }
	if (a >= 0xe400 && a < 0xe480) { s_colorram[a & 0x7f] = v; return; }
	if (a >= 0xf000 && a < 0xf100) { s_spriteram[a & 0xff] = v; return; }
	if (a >= 0xfc00 && a <= 0xfc07) {
		s_q[a & 7] = v & 1;
		if ((a & 7) == 1 && !(v & 1))
			s_nmi_line = 0;   // nmi_mask_w: clear the NMI line
		return;
	}
	switch (a) {
	case 0xe800: s_soundlatch = v; break;
	case 0xf400: s_bkgcol = v; break;
	case 0xf801: s_bkgpen = v; break;
	case 0xf802: s_bkgpage = v & 0x1f; s_bkgflip = (v & 0x80) >> 7; break;
	case 0xf803: s_flipy = v & 1; break;
	}
}

// ---- sound CPU ----
static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x1000) return a < s_audio_size ? s_audio_rom[a] : 0;
	if (a >= 0x2000 && a < 0x3000) return s_audio_ram[a & 0xfff];
	if (a == 0x3000) return s_soundlatch;
	return 0;
}
static void audio_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x2000 && a < 0x3000) { s_audio_ram[a & 0xfff] = v; return; }
	switch (a) {
	case 0x3000: s_snmi_mask = v & 1; break;
	case 0x4000: case 0x4001: ay_sync(); ay8910_write(&s_ay[0], a & 1, v); break;
	case 0x5000: case 0x5001: ay_sync(); ay8910_write(&s_ay[1], a & 1, v); break;
	case 0x6000: case 0x6001: ay_sync(); ay8910_write(&s_ay[2], a & 1, v); break;
	}
}
static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// ---- video ----
static inline void put8(uint16_t *bm, int pitch, const uint8_t *src, int sx, int sy, int fx, int fy, uint16_t base)
{
	for (int y = 0; y < 8; y++) {
		int dy = sy + y;
		if (dy < VIS_Y0 || dy > VIS_Y1)
			continue;
		const uint8_t *r = src + (fy ? 7 - y : y) * 8;
		uint16_t *d = bm + (dy - VIS_Y0) * pitch + sx;
		for (int x = 0; x < 8; x++) {
			uint8_t p = r[fx ? 7 - x : x];
			if (p)
				d[x] = (uint16_t)(base + p);
		}
	}
}

static void rr_render(void)
{
	const snap_t *s = &s_snap;
	memcpy(rbx_video.palette, s_pal_rgb, sizeof(s_pal_rgb));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, h = rbx_video.height;
	for (int y = 0; y < h; y++)
		for (int x = 0; x < SCREEN_W; x++)
			bm[y * pitch + x] = s->bkgpen;
	// road
	const uint8_t *lay = s_layout + s->bkgpage * 1024;
	for (int offs = 0x3ff; offs >= 0; offs--) {
		int sy = s->bkgflip ? offs / 32 : 31 - offs / 32;
		int sx = offs % 32;
		if (s->flipx) sx = 31 - sx;
		if (s->flipy) sy = 31 - sy;
		if (sy * 8 + 8 <= VIS_Y0 || sy * 8 > VIS_Y1)
			continue;
		int code = lay[offs] + ((lay[offs + 0x4000] & 0xc0) >> 6) * 256;
		put8(bm, pitch, s_road + code * 64, sx * 8, sy * 8, s->flipx, s->bkgflip ^ s->flipy,
		     (uint16_t)((s->bkgcol % 32) * 8));
	}
	// sprites
	for (int offs = 0x80 - 4; offs >= 0; offs -= 4) {
		const uint8_t *e = s->spriteram + offs;
		int sy = e[0] - 16, sx = e[3] - 16;
		if (!sx || !sy)
			continue;
		if (s->flipx) sx = 224 - sx;
		if (s->flipy) sy = 224 - sy;
		int sfy = (e[1] & 0x80) ? 1 : 0;
		int bank = (e[1] & 0x40) >> 6;
		if (bank)
			bank += s->spritebank;
		const uint8_t *src = s_sprites + ((size_t)bank * 64 + (e[1] & 0x3f)) * 1024;
		uint16_t base = (uint16_t)((e[2] & 0x1f) * 8);
		int fx = s->flipx, fy = !(sfy ^ s->flipy);
		for (int y = 0; y < 32; y++) {
			int dy = sy + y;
			if (dy < VIS_Y0 || dy > VIS_Y1)
				continue;
			const uint8_t *r = src + (fy ? 31 - y : y) * 32;
			uint16_t *d = bm + (dy - VIS_Y0) * pitch;
			for (int x = 0; x < 32; x++) {
				int dx = sx + x;
				uint8_t p = r[fx ? 31 - x : x];
				if (p && dx >= 0 && dx < SCREEN_W)
					d[dx] = (uint16_t)(base + p);
			}
		}
	}
	// fg tilemap: column scroll, TILE_FLIPY on bottom-up rows, flipped in x and y by flip x
	const uint8_t *chars = s_chars + (size_t)s->chrbank * 256 * 64;
	for (int sy = VIS_Y0; sy <= VIS_Y1; sy++) {
		uint16_t *d = bm + (sy - VIS_Y0) * pitch;
		int ny = s->flipx ? 255 - sy : sy;
		for (int sx = 0; sx < SCREEN_W; sx++) {
			int nx = s->flipx ? 255 - sx : sx;
			int col = nx >> 3;
			int ty = (ny + s->colorram[col * 2]) & 255;
			uint8_t p = chars[s->videoram[(ty >> 3) * 32 + col] * 64 + (7 - (ty & 7)) * 8 + (nx & 7)];
			if (p)
				d[sx] = (uint16_t)((s->colorram[col * 2 + 1] & 0x1f) * 8 + p);
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap.videoram, s_videoram, sizeof(s_videoram));
	memcpy(s_snap.colorram, s_colorram, sizeof(s_colorram));
	memcpy(s_snap.spriteram, s_spriteram, sizeof(s_snap.spriteram));
	s_snap.bkgpage = s_bkgpage;
	s_snap.bkgflip = s_bkgflip;
	s_snap.bkgpen = s_bkgpen;
	s_snap.bkgcol = s_bkgcol;
	s_snap.flipx = s_q[0];
	s_snap.flipy = s_flipy;
	s_snap.chrbank = (uint8_t)(s_q[4] | s_q[5] << 1);
	s_snap.spritebank = s_q[6];
	rbx_video_frame_done();
}

// ---- init / frame ----
static int gfx_bit(const uint8_t *gfx, uint32_t size, uint32_t bit)
{
	uint32_t byte = bit >> 3;
	return byte < size ? (gfx[byte] >> (7 - (bit & 7))) & 1 : 0;
}

// MAME gfx_layout decode (offsets in bits, the first plane is the most significant bit)
static void decode(const uint8_t *gfx, uint32_t size, uint32_t base, const uint32_t *planes, int np,
                   const uint32_t *xo, const uint32_t *yo, int w, uint32_t inc, int count, uint8_t *out)
{
	for (int t = 0; t < count; t++)
		for (int y = 0; y < w; y++)
			for (int x = 0; x < w; x++) {
				int v = 0;
				for (int p = 0; p < np; p++)
					v = v << 1 | gfx_bit(gfx, size, base * 8 + planes[p] + (uint32_t)t * inc + yo[y] + xo[x]);
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
}

static void rr_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_videoram, 0, sizeof(s_videoram));
	memset(s_colorram, 0, sizeof(s_colorram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_audio_ram, 0, sizeof(s_audio_ram));
	memset(s_q, 0, sizeof(s_q));
	zcpu_init(&s_main, main_read, main_write);
	zcpu_init(&s_audio, audio_read, audio_write);
	s_bkgpage = s_bkgflip = s_bkgpen = s_bkgcol = s_flipy = s_soundlatch = s_snmi_mask = 0;
	s_nmi_line = 0;
	s_main_frac = s_audio_frac = 0;
	for (int i = 0; i < 3; i++)
		ay8910_reset(&s_ay[i]);
	s_ay_done = 0;
	s_mix_n = 0;
}

static void rr_exit(void)
{
	free(s_chars);
	free(s_road);
	free(s_sprites);
	free(s_mixl);
	free(s_mixr);
	s_chars = s_road = s_sprites = NULL;
	s_mixl = s_mixr = NULL;
}

static int rr_init(const rbx_game *g)
{
	uint32_t cs = 0, rs = 0, ls = 0, ps = 0, ss[3] = { 0, 0, 0 };
	s_main_rom = rbx_region("maincpu", &s_main_size);
	s_audio_rom = rbx_region("audiocpu", &s_audio_size);
	uint8_t *chars = rbx_region("chars", &cs), *road = rbx_region("road", &rs);
	s_layout = rbx_region("road_layout", &ls);
	uint8_t *spr[3] = { rbx_region("sprites_0", &ss[0]), rbx_region("sprites_1", &ss[1]), rbx_region("sprites_2", &ss[2]) };
	uint8_t *prom = rbx_region("proms", &ps);
	if (!s_main_rom || !s_audio_rom || !chars || cs < 0x6000 || !road || rs < 0x6000 || !s_layout || ls < 0x8000 ||
	    !spr[0] || !spr[1] || !spr[2] || !prom || ps < 0x300) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_chars = (uint8_t *)malloc(4 * 256 * 64);
	s_road = (uint8_t *)malloc(1024 * 64);
	s_sprites = (uint8_t *)malloc(3 * 64 * 1024);
	if (!s_chars || !s_road || !s_sprites) {
		rr_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	// charlayout / charlayout2: 3 planes { 0, 0x2000 * 8, 0x4000 * 8 }, rows stored bottom up
	static const uint32_t cx[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
	static const uint32_t cy[8] = { 56, 48, 40, 32, 24, 16, 8, 0 };
	static const uint32_t cp[3] = { 0, 0x2000 * 8, 0x4000 * 8 };
	for (int b = 0; b < 4; b++)
		decode(chars, cs, (uint32_t)b * 0x800, cp, 3, cx, cy, 8, 64, 256, s_chars + (size_t)b * 256 * 64);
	decode(road, rs, 0, cp, 3, cx, cy, 8, 64, 1024, s_road);
	// spritelayout: 32x32, 3 planes { 0x4000 * 8, 0x2000 * 8, 0 }
	uint32_t sx[32], sy[32];
	for (int i = 0; i < 32; i++) {
		sx[i] = (uint32_t)((i & 7) + (i >> 3) * 64);
		sy[i] = (uint32_t)((i & 7) * 8 + (i >> 3) * 256);
	}
	static const uint32_t sp[3] = { 0x4000 * 8, 0x2000 * 8, 0 };
	for (int b = 0; b < 3; b++)
		decode(spr[b], ss[b], 0, sp, 3, sx, sy, 32, 1024, 64, s_sprites + (size_t)b * 64 * 1024);
	for (int i = 0; i < 256; i++) {
		int c[3];
		for (int k = 0; k < 3; k++) {
			uint8_t v = prom[i + k * 256];
			c[k] = 0x0e * (v & 1) + 0x1f * ((v >> 1) & 1) + 0x42 * ((v >> 2) & 1) + 0x90 * ((v >> 3) & 1);
		}
		s_pal_rgb[i] = (uint32_t)(c[0] << 16 | c[1] << 8 | c[2]);
	}
	s_port_p1 = rbx_port_find("P1");
	s_port_p2 = rbx_port_find("P2");
	s_port_system = rbx_port_find("SYSTEM");
	s_port_dsw1 = rbx_port_find("DSW1");
	s_port_dsw2 = rbx_port_find("DSW2");
	// three AY-3-8910 at 1.5 MHz, MAME route 0.10 each (legacy output)
	for (int i = 0; i < 3; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, AUDIO_CLOCK, AY8910_LEGACY_OUTPUT, 26, 4, 1);
	s_mix_cap = (int)(ay8910_rate(&s_ay[0]) / 50) + 64;
	s_mixl = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_mixr = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mixl || !s_mixr || rbx_audio_setup_stereo(ay8910_rate(&s_ay[0])) ||
	    rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 256)) {
		rr_exit();
		return -1;
	}
	rr_reset();
	return 0;
}

static void zcpu_run(zcpu *c, int cycles)
{
	unsigned long end = c->z.cyc + (unsigned long)cycles;
	while ((long)(c->z.cyc - end) < 0)
		z80_step(&c->z);
}

static void rr_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			if (s_q[1] && !s_nmi_line) {   // vblank_irq: assert NMI (edge)
				s_nmi_line = 1;
				z80_gen_nmi(&s_main.z);
			}
		}
		if (line % SNMI_LINES == 0 && s_snmi_mask)
			z80_gen_nmi(&s_audio.z);   // sound_timer_irq: pulse
		zcpu_run(&s_main, rbx_line_cycles(&s_main_frac, MAIN_CLOCK, LINE_RATE * 1000, 1));
		int alen = rbx_line_cycles(&s_audio_frac, AUDIO_CLOCK, LINE_RATE * 1000, 1);
		s_line_ay = s_ay_done;
		s_audio_start = s_audio.z.cyc;
		zcpu_run(&s_audio, alen);
		ay_run_to(s_line_ay + (uint32_t)alen);
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put_stereo(s_mixl[i], s_mixr[i]);
	memset(s_mixl, 0, sizeof(int32_t) * (size_t)s_mix_n);
	memset(s_mixr, 0, sizeof(int32_t) * (size_t)s_mix_n);
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

static int rr_state(void *ctx, int load)
{
	int rc = 0;
	rc |= zcpu_state(ctx, "main", &s_main, load);
	rc |= zcpu_state(ctx, "audio", &s_audio, load);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "videoram", s_videoram, sizeof(s_videoram));
	rc |= rbx_state_io(ctx, "colorram", s_colorram, sizeof(s_colorram));
	rc |= rbx_state_io(ctx, "spriteram", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "audram", s_audio_ram, sizeof(s_audio_ram));
	rc |= rbx_state_io(ctx, "latch", s_q, sizeof(s_q));
	uint8_t regs[7] = { s_bkgpage, s_bkgflip, s_bkgpen, s_bkgcol, s_flipy, s_soundlatch, s_snmi_mask };
	rc |= rbx_state_io(ctx, "regs", regs, sizeof(regs));
	rc |= rbx_state_io(ctx, "nmi", &s_nmi_line, sizeof(s_nmi_line));
	rc |= rbx_state_io(ctx, "mfrac", &s_main_frac, sizeof(s_main_frac));
	rc |= rbx_state_io(ctx, "afrac", &s_audio_frac, sizeof(s_audio_frac));
	rc |= rbx_state_io(ctx, "ay1", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "ay2", &s_ay[1], ay8910_state_size());
	rc |= rbx_state_io(ctx, "ay3", &s_ay[2], ay8910_state_size());
	if (rc)
		return -1;
	if (load) {
		s_bkgpage = regs[0]; s_bkgflip = regs[1]; s_bkgpen = regs[2]; s_bkgcol = regs[3];
		s_flipy = regs[4]; s_soundlatch = regs[5]; s_snmi_mask = regs[6];
	}
	return 0;
}

static int rr_save(void *ctx) { return rr_state(ctx, 0); }
static int rr_load(void *ctx) { return rr_state(ctx, 1); }
static int rr_supported(const char *machine) { return !strcmp(machine, "rollace") || !strcmp(machine, "rollace2"); }

const rbx_driver rbx_drv_rollrace = { "Fighting Roller", rr_init, rr_exit, rr_reset, rr_frame, rr_save, rr_load,
                                      rr_supported, rr_render };
