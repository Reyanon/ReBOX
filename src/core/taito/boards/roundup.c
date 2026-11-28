/*  Round-Up / Fitter / The Pit / Dock Man / Port Man / Super Mouse hardware, ported from MAME
    src/mame/taito/roundup.cpp (BSD-3-Clause, copyright-holders: Zsolt Vasvari).
    Memory maps (thepit, intrepid, dockmanb, theportr), the LS259 main latch (NMI mask, sound enable, graphics bank,
    flip screen + LS157 input multiplexer), the vblank NMI, the sound CPU IRQ at line 256 (cleared by the acknowledge),
    the two AY-3-8910 at 18.432 MHz / 12 and the column-scrolled tilemap with the solid colour priority layer follow
    MAME. Desert Dan (desertdn: thepit map with ROM up to 0x7fff, tiles from bank 0 and sprites from bank 1, its own
    sprite clip) and Romar Triv (rtriv: intrepid map with the question ROM reader at 0x4000) as in MAME.
    Watchdog not emulated.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"

#include <stdlib.h>
#include <string.h>

enum { MAP_THEPIT, MAP_INTREPID, MAP_DOCKMANB, MAP_THEPORTR };
enum { K_NONE, K_DESERTDN, K_RTRIV };

typedef struct {
	const char *name;
	int map, banked, suprmous, kind;
} machine_t;

static const machine_t s_machines[] = {
	{ "roundup",  MAP_THEPIT,   0, 0 },
	{ "thepit",   MAP_THEPIT,   0, 0 },
	{ "intrepid", MAP_INTREPID, 1, 0 },
	{ "dockmanb", MAP_DOCKMANB, 1, 0 },
	{ "theportr", MAP_THEPORTR, 1, 0 },
	{ "suprmous", MAP_INTREPID, 0, 1 },   // (gfx_suprmous has no second bank)
	{ "desertdn", MAP_THEPIT,   1, 0, K_DESERTDN },   // (gfx_intrepid, latch Q5 not connected)
	{ "rtriv",    MAP_INTREPID, 1, 0, K_RTRIV },
};

// MAME set_raw(18.432 MHz / 3, 384, 0, 256, 264, 16, 240): 16 kHz lines
#define LINES 264
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256
#define MAIN_LINE 192         // 3.072 MHz
#define AUDIO_CLOCK 2500000   // 10 MHz / 4
#define LINE_RATE 16000
#define AY_LINE 96            // 1.536 MHz
#define AY_CLOCK 1536000

typedef struct {
	z80 z;
	int irq;   // IRQ line, cleared by the acknowledge
} zcpu;

static const machine_t *s_m;
static uint8_t *s_main_rom, *s_audio_rom;
static uint32_t s_main_size, s_audio_size;
static uint8_t s_ram[0x800], s_ram4000[0x800], s_colorram[0x400], s_videoram[0x400], s_ram98[0x100], s_audio_ram[0x400];
static zcpu s_main, s_audio;
static uint8_t s_latch_q[8];
static uint8_t s_soundlatch;
static int s_nmi_line;
static ay8910_t s_ay[2];
static uint64_t s_audio_frac;
static unsigned long s_audio_start;
static int s_audio_len;
// video
static uint8_t *s_chars, *s_sprites;   // [bank][256][64], [bank][64][256]
static int s_nbanks, s_gran, s_ncolors;
static uint32_t s_pal_rgb[40];
static uint8_t *s_cat;                 // per pixel category of the current frame (render side)
typedef struct {
	uint8_t colorram[0x400], videoram[0x400], ram98[0x100];
	uint8_t bank, flipx, flipy;
} snap_t;
static snap_t s_snap;
// rtriv question ROM reader (rtriv_state::question_r)
static const uint8_t *s_questions;
static uint32_t s_questions_size;
static uint8_t s_q_rom, s_q_remap[16];
static uint32_t s_q_address;
// ports
static int s_port_in0, s_port_in1, s_port_in2, s_port_dsw;
// sound
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static uint32_t s_ay_done, s_line_ay;

// ---- sound ----
static void ay_run_to(uint32_t pos)
{
	if (pos <= s_ay_done)
		return;
	int n = ay8910_render(&s_ay[0], pos - s_ay_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	ay8910_render(&s_ay[1], pos - s_ay_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	if (!s_latch_q[3])   // sound_enable_w: system_mute(!state)
		memset(s_mix + s_mix_n, 0, sizeof(int32_t) * (size_t)n);
	s_mix_n += n;
	s_ay_done = pos;
}
static void ay_sync(void)
{
	uint32_t done = (uint32_t)(s_audio.z.cyc - s_audio_start);
	if (s_audio_len <= 0)
		return;
	if (done > (uint32_t)s_audio_len)
		done = (uint32_t)s_audio_len;
	ay_run_to(s_line_ay + AY_LINE * done / (uint32_t)s_audio_len);
}
static uint8_t ay1_port_a(void *u) { (void)u; return s_soundlatch; }

// ---- main CPU ----
static uint8_t port8(int p) { return p >= 0 ? (uint8_t)rbx_port_read(p) : 0xff; }

static void latch_w(int bit, int state)
{
	s_latch_q[bit] = (uint8_t)state;
	if (bit == 0 && !state)
		s_nmi_line = 0;   // nmi_mask_w: clear the NMI line
}

static int colorram_off(uint16_t a)
{
	switch (s_m->map) {
	case MAP_THEPIT: return (a >= 0x8800 && a < 0x9000) ? (a & 0x3ff) : -1;
	case MAP_INTREPID: return ((a >= 0x8c00 && a < 0x9000) || (a >= 0x9400 && a < 0x9800)) ? (a & 0x3ff) : -1;
	default: return (a >= 0x8800 && a < 0x8c00) ? (a & 0x3ff) : -1;
	}
}

static uint8_t question_r(uint16_t offset)
{
	if ((offset & 0xc00) == 0x800)
		s_q_remap[offset & 0x0f] = (uint8_t)(((offset & 0xf0) >> 4) ^ 0x0f);
	else if ((offset & 0xc00) == 0x400) {
		s_q_rom = (uint8_t)((offset & 0x70) >> 4);
		s_q_address = (uint32_t)(((offset & 0x80) << 3) | ((offset & 0x0f) << 11));
	} else if ((offset & 0xc00) == 0xc00) {
		uint32_t real = (0x8000u * s_q_rom) | s_q_address | (offset & 0x3f0u) | s_q_remap[offset & 0x0f];
		return real < s_questions_size ? s_questions[real] : 0;
	}
	return 0;
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (s_m->map == MAP_THEPORTR && a >= 0x4000 && a < 0x4800)
		return s_ram4000[a & 0x7ff];
	if (s_m->kind == K_RTRIV && a >= 0x4000 && a < 0x5000)
		return question_r(a & 0xfff);
	if (a < 0x8000) {
		if (s_m->map == MAP_THEPIT && a >= 0x5000 && s_m->kind != K_DESERTDN)
			return 0;
		return a < s_main_size ? s_main_rom[a] : 0;
	}
	if (a < 0x8800)
		return s_m->map == MAP_THEPORTR ? 0 : s_ram[a & 0x7ff];
	if (a < 0x9800) {
		int c = colorram_off(a);
		if (c >= 0)
			return s_colorram[c];
		if (a >= 0x9000 && (a < 0x9400 || s_m->map == MAP_THEPIT))
			return s_videoram[a & 0x3ff];
		return 0;
	}
	if (a < 0xa000) {
		if ((a & 0xc0) == 0)
			return s_ram98[a & 0x3f];   // attributes, mirror 0x700
		if (a < 0x9900)
			return s_ram98[a & 0xff];
		return 0;
	}
	switch (a & 0xf800) {
	case 0xa000:
		if (a == 0xa000)
			return (uint8_t)~port8(s_latch_q[6] ? s_port_in2 : s_port_in0);   // LS157 selected by flip x, inverted
		return 0;
	case 0xa800: return a == 0xa800 ? port8(s_port_in1) : 0;
	case 0xb000: return a == 0xb000 ? port8(s_port_dsw) : 0;
	}
	return 0;   // b800: watchdog reset
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_m->map == MAP_THEPORTR && a >= 0x4000 && a < 0x4800) {
		s_ram4000[a & 0x7ff] = v;
		return;
	}
	if (a < 0x8000)
		return;
	if (a < 0x8800) {
		if (s_m->map != MAP_THEPORTR)
			s_ram[a & 0x7ff] = v;
		return;
	}
	if (a < 0x9800) {
		int c = colorram_off(a);
		if (c >= 0)
			s_colorram[c] = v;
		else if (a >= 0x9000 && (a < 0x9400 || s_m->map == MAP_THEPIT))
			s_videoram[a & 0x3ff] = v;
		return;
	}
	if (a < 0xa000) {
		if ((a & 0xc0) == 0)
			s_ram98[a & 0x3f] = v;
		else if (a < 0x9900)
			s_ram98[a & 0xff] = v;
		return;
	}
	if (a >= 0xb000 && a <= 0xb007) {
		latch_w(a & 7, v & 1);
		return;
	}
	if (a == 0xb800)
		s_soundlatch = v;
}

// ---- sound CPU ----
static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x2000)
		return a < s_audio_size ? s_audio_rom[a] : 0;
	if (a >= 0x3800 && a < 0x3c00)
		return s_audio_ram[a & 0x3ff];
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
	if (p == 0x8d) { ay_sync(); return ay8910_data_r(&s_ay[1]); }
	if (p == 0x8f) { ay_sync(); return ay8910_data_r(&s_ay[0]); }
	return 0;
}
static void audio_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	switch (p) {
	case 0x00: s_soundlatch = 0; break;   // generic_latch_8 clear_w
	case 0x8c: case 0x8d: ay_sync(); ay8910_write(&s_ay[1], p & 1, v); break;
	case 0x8e: case 0x8f: ay_sync(); ay8910_write(&s_ay[0], p & 1, v); break;
	}
}
static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// ---- video ----
static void draw_sprites(uint16_t *bm, int pitch, int prio)
{
	const snap_t *s = &s_snap;
	int cx0 = s->flipx ? 0 : 2 * 8 + 1, cx1 = s->flipx ? 30 * 8 - 2 : 32 * 8 - 1;
	if (s_m->kind == K_DESERTDN) {   // screen_update_desertdan
		cx0 = s->flipy ? 8 * 8 : 0 * 8 + 1;
		cx1 = s->flipy ? 32 * 8 - 2 : 24 * 8 - 1;
	}
	const uint8_t *spr = s->ram98 + 0x40;
	for (int offs = 0x20 - 4; offs >= 0; offs -= 4) {
		if (((spr[offs + 2] & 0x08) >> 3) != prio)
			continue;
		if (spr[offs] == 0 || spr[offs + 3] == 0)
			continue;
		uint8_t y = (uint8_t)(240 - spr[offs]);
		uint8_t x = (uint8_t)(spr[offs + 3] + 1);
		int flipx = spr[offs + 1] & 0x40, flipy = spr[offs + 1] & 0x80;
		if (s->flipy) { y = (uint8_t)(240 - y); flipy = !flipy; }
		if (s->flipx) { x = (uint8_t)(242 - x); flipx = !flipx; }
		if (offs < 16) y++;
		int code = spr[offs + 1] & 0x3f;
		int bank = s_m->kind == K_DESERTDN ? 1 : s_m->banked ? s->bank : 0;
		const uint8_t *src = s_sprites + ((size_t)bank * 64 + (size_t)code) * 256;
		uint16_t base = (uint16_t)((rbx_wrap(spr[offs + 2], s_ncolors)) * s_gran);
		for (int pass = 0; pass < 2; pass++) {
			int sx0 = pass ? x - 256 : x;
			for (int yy = 0; yy < 16; yy++) {
				int sy = y + yy;
				if (sy < VIS_Y0 || sy > VIS_Y1)
					continue;
				const uint8_t *r = src + (flipy ? 15 - yy : yy) * 16;
				uint16_t *d = bm + (sy - VIS_Y0) * pitch;
				const uint8_t *cat = s_cat + (sy - VIS_Y0) * SCREEN_W;
				for (int xx = 0; xx < 16; xx++) {
					int sx = sx0 + xx;
					if (sx < cx0 || sx > cx1)
						continue;
					uint8_t p = r[flipx ? 15 - xx : xx];
					if (!p || (!prio && cat[sx]))
						continue;
					d[sx] = (uint16_t)(base + p);
				}
			}
		}
	}
}

static void ru_render(void)
{
	const snap_t *s = &s_snap;
	memcpy(rbx_video.palette, s_pal_rgb, sizeof(s_pal_rgb));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const int bank = s_m->banked && s_m->kind != K_DESERTDN ? s->bank : 0;
	const uint8_t *chars = s_chars + (size_t)bank * 256 * 64;
	for (int sy = VIS_Y0; sy <= VIS_Y1; sy++) {
		uint16_t *d = bm + (sy - VIS_Y0) * pitch;
		uint8_t *cat = s_cat + (sy - VIS_Y0) * SCREEN_W;
		int ny0 = s->flipy ? 255 - sy : sy;
		for (int sx = 0; sx < SCREEN_W; sx++) {
			int nx = s->flipx ? 255 - sx : sx;
			int col = nx >> 3;
			int ty = (ny0 + s->ram98[col << 1]) & 255;
			int idx = (ty >> 3) * 32 + col;
			uint8_t c = s->colorram[idx];
			int back = (c & 0x70) >> 4;
			int prio = back != 0 && !(c & 0x80);
			uint16_t pen = (uint16_t)(32 + back);
			if (!prio) {
				uint8_t p = chars[s->videoram[idx] * 64 + (ty & 7) * 8 + (nx & 7)];
				if (p)
					pen = (uint16_t)((rbx_wrap(c, s_ncolors)) * s_gran + p);
			}
			d[sx] = pen;
			cat[sx] = (uint8_t)prio;
		}
	}
	draw_sprites(bm, pitch, 0);
	draw_sprites(bm, pitch, 1);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap.colorram, s_colorram, sizeof(s_colorram));
	memcpy(s_snap.videoram, s_videoram, sizeof(s_videoram));
	memcpy(s_snap.ram98, s_ram98, sizeof(s_ram98));
	s_snap.bank = s_latch_q[5];
	s_snap.flipx = s_latch_q[6];
	s_snap.flipy = s_latch_q[7];
	rbx_video_frame_done();
}

// ---- init / frame ----
static const machine_t *find_machine(const char *name)
{
	for (size_t i = 0; i < sizeof(s_machines) / sizeof(s_machines[0]); i++)
		if (!strcmp(s_machines[i].name, name))
			return &s_machines[i];
	return NULL;
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

static void ru_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_ram4000, 0, sizeof(s_ram4000));
	memset(s_colorram, 0, sizeof(s_colorram));
	memset(s_videoram, 0, sizeof(s_videoram));
	memset(s_ram98, 0, sizeof(s_ram98));
	memset(s_audio_ram, 0, sizeof(s_audio_ram));
	memset(s_latch_q, 0, sizeof(s_latch_q));
	zcpu_init(&s_main, main_read, main_write, no_in, no_out);
	zcpu_init(&s_audio, audio_read, audio_write, audio_in, audio_out);
	s_soundlatch = 0;
	s_nmi_line = 0;
	s_audio_frac = 0;
	s_q_rom = 0;
	s_q_address = 0;
	memset(s_q_remap, 0, sizeof(s_q_remap));
	ay8910_reset(&s_ay[0]);
	ay8910_reset(&s_ay[1]);
	s_ay_done = 0;
	s_mix_n = 0;
}

static void ru_exit(void)
{
	free(s_chars);
	s_chars = NULL;
	free(s_sprites);
	s_sprites = NULL;
	free(s_cat);
	s_cat = NULL;
	free(s_mix);
	s_mix = NULL;
}

// MAME gfx layout bit order: plane offsets in bits, pixel bit 7 first
static int gfx_bit(const uint8_t *gfx, uint32_t size, uint32_t bit)
{
	uint32_t byte = bit >> 3;
	return byte < size ? (gfx[byte] >> (7 - (bit & 7))) & 1 : 0;
}

static void decode(const uint8_t *gfx, uint32_t size, uint32_t base, const uint32_t *planes, int np, int big, int count,
                   uint8_t *out)
{
	const int w = big ? 16 : 8, len = big ? 256 : 64;
	for (int t = 0; t < count; t++)
		for (int y = 0; y < w; y++)
			for (int x = 0; x < w; x++) {
				uint32_t xo = (uint32_t)((x & 7) + (x >= 8 ? 64 : 0));
				uint32_t yo = (uint32_t)((y & 7) * 8 + (y >= 8 ? 128 : 0));
				uint32_t bit = base * 8 + (uint32_t)t * (uint32_t)len + xo + yo;
				int v = 0;
				for (int p = 0; p < np; p++)
					v = v << 1 | gfx_bit(gfx, size, planes[p] + bit);
				out[(size_t)t * (size_t)(w * w) + (size_t)(y * w + x)] = (uint8_t)v;
			}
}

static int ru_init(const rbx_game *g)
{
	s_m = find_machine(g->machine);
	if (!s_m) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	uint32_t gsize = 0, psize = 0;
	s_main_rom = rbx_region("maincpu", &s_main_size);
	s_audio_rom = rbx_region("audiocpu", &s_audio_size);
	uint8_t *gfx = rbx_region("gfx", &gsize);
	uint8_t *prom = rbx_region("proms", &psize);
	s_questions = s_m->kind == K_RTRIV ? rbx_region("questions", &s_questions_size) : NULL;
	if ((s_m->kind == K_RTRIV && !s_questions) || !s_main_rom || !s_audio_rom || !gfx || !prom || psize < (s_m->suprmous ? 0x40u : 0x20u)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_nbanks = s_m->banked ? 2 : 1;
	s_chars = (uint8_t *)malloc((size_t)s_nbanks * 256 * 64);
	s_sprites = (uint8_t *)malloc((size_t)s_nbanks * 64 * 256);
	s_cat = (uint8_t *)calloc(SCREEN_W * (VIS_Y1 - VIS_Y0 + 1), 1);
	if (!s_chars || !s_sprites || !s_cat) {
		ru_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (s_m->suprmous) {
		static const uint32_t pl3[3] = { 0x2000 * 8, 0x1000 * 8, 0 };
		decode(gfx, gsize, 0x0000, pl3, 3, 0, 256, s_chars);
		decode(gfx, gsize, 0x0800, pl3, 3, 1, 64, s_sprites);
		s_gran = 8;
		s_ncolors = 4;
	} else {
		static const uint32_t pl2[2] = { 0x1000 * 8, 0 };
		for (int b = 0; b < s_nbanks; b++) {
			decode(gfx, gsize, (uint32_t)b * 0x800, pl2, 2, 0, 256, s_chars + (size_t)b * 256 * 64);
			decode(gfx, gsize, (uint32_t)b * 0x800, pl2, 2, 1, 64, s_sprites + (size_t)b * 64 * 256);
		}
		s_gran = 4;
		s_ncolors = 8;
	}
	// palettes (roundup_palette / suprmous_palette)
	for (int i = 0; i < 32; i++) {
		int r, gg, b;
		if (s_m->suprmous) {
			uint8_t bb = 0, g8 = 0;
			for (int k = 0; k < 8; k++) {
				bb |= (uint8_t)(((prom[i] >> k) & 1) << (7 - k));
				g8 |= (uint8_t)(((prom[i + 0x20] >> k) & 1) << (7 - k));
			}
			int r5 = ((bb >> 5) & 7) << 2 | ((g8 >> 6) & 3);
			r = (r5 & 31) << 3 | (r5 & 31) >> 2;
			gg = (g8 & 31) << 3 | (g8 & 31) >> 2;
			b = (bb & 15) * 17;
		} else {
			uint8_t c = prom[i];
			r = 0x21 * (c & 1) + 0x47 * ((c >> 1) & 1) + 0x97 * ((c >> 2) & 1);
			gg = 0x21 * ((c >> 3) & 1) + 0x47 * ((c >> 4) & 1) + 0x97 * ((c >> 5) & 1);
			b = 0x47 * ((c >> 6) & 1) + 0x97 * ((c >> 7) & 1);
		}
		s_pal_rgb[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	for (int i = 0; i < 8; i++)
		s_pal_rgb[32 + i] = (uint32_t)((i & 4 ? 0xff0000 : 0) | (i & 2 ? 0xff00 : 0) | (i & 1 ? 0xff : 0));
	s_port_in0 = rbx_port_find("IN0");
	s_port_in1 = rbx_port_find("IN1");
	s_port_in2 = rbx_port_find("IN2");
	s_port_dsw = rbx_port_find("DSW");
	// two AY-3-8910 at 1.536 MHz, MAME route 0.25 each (legacy output), 48 kHz
	for (int i = 0; i < 2; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 64, 4, 1);
	s_ay[0].port_a_read = ay1_port_a;
	s_mix_cap = (int)(ay8910_rate(&s_ay[0]) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(ay8910_rate(&s_ay[0])) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 40)) {
		ru_exit();
		return -1;
	}
	ru_reset();
	return 0;
}

static void zcpu_run(zcpu *c, int cycles)
{
	unsigned long end = c->z.cyc + (unsigned long)cycles;
	while ((long)(c->z.cyc - end) < 0) {
		c->z.int_pending = c->irq ? 1 : 0;
		z80_step(&c->z);
		if (c->irq && !c->z.int_pending)
			c->irq = 0;   // vsync_int_ack
	}
}

static void ru_frame(void)
{
	rbx_audio.frame_mhz = LINE_RATE * 1000 / LINES;   // (the board's frame rate: a frame's sound is played in a 60 Hz frame, as its picture)
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			if (s_latch_q[0] && !s_nmi_line) {   // vblank_w: assert NMI (edge)
				s_nmi_line = 1;
				z80_gen_nmi(&s_main.z);
			}
		}
		if (line == 256)
			s_audio.irq = 1;   // vsync_callback
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

static int ru_state(void *ctx, int load)
{
	int rc = 0;
	rc |= zcpu_state(ctx, "main", &s_main, load);
	rc |= zcpu_state(ctx, "audio", &s_audio, load);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "ram4000", s_ram4000, sizeof(s_ram4000));
	rc |= rbx_state_io(ctx, "colorram", s_colorram, sizeof(s_colorram));
	rc |= rbx_state_io(ctx, "videoram", s_videoram, sizeof(s_videoram));
	rc |= rbx_state_io(ctx, "ram98", s_ram98, sizeof(s_ram98));
	rc |= rbx_state_io(ctx, "audram", s_audio_ram, sizeof(s_audio_ram));
	rc |= rbx_state_io(ctx, "latch", s_latch_q, sizeof(s_latch_q));
	rc |= rbx_state_io(ctx, "slatch", &s_soundlatch, 1);
	rc |= rbx_state_io(ctx, "nmi", &s_nmi_line, sizeof(s_nmi_line));
	rc |= rbx_state_io(ctx, "afrac", &s_audio_frac, sizeof(s_audio_frac));
	rc |= rbx_state_io(ctx, "ay1", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "ay2", &s_ay[1], ay8910_state_size());
	if (s_m->kind == K_RTRIV) {
		rc |= rbx_state_io(ctx, "qrom", &s_q_rom, 1);
		rc |= rbx_state_io(ctx, "qaddr", &s_q_address, sizeof(s_q_address));
		rc |= rbx_state_io(ctx, "qremap", s_q_remap, sizeof(s_q_remap));
	}
	return rc ? -1 : 0;
}

static int ru_save(void *ctx) { return ru_state(ctx, 0); }
static int ru_load(void *ctx) { return ru_state(ctx, 1); }
static int ru_supported(const char *machine) { return find_machine(machine) != NULL; }

const rbx_driver rbx_drv_roundup = { "Round-Up / The Pit", ru_init, ru_exit, ru_reset, ru_frame, ru_save, ru_load,
                                     ru_supported, ru_render };
