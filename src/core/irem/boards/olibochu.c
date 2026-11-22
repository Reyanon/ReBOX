/*  Irem Oli-Boo-Chu / Punching Kid. Ported from MAME src/mame/irem/olibochu.cpp (BSD-3-Clause, copyright-holders: Nicola
    Salmoria). A Z80 at 3.07 MHz interrupted twice a frame, a 32x32 tilemap of 2 bpp tiles, 8x8 and 16x16 sprites; the
    sound: a second Z80 with an AY-3-8910, a frame behind on the worker (rbx_aux). Its HC55516 samples are silent.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"
#include "../../video/gfxlayout.h"
#include "../video/irem_draw.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define PIX_CLOCK 6144000u     // (18.432 MHz / 3)
#define HTOTAL 384u
#define LINES 256
#define LINE_CYC (HTOTAL / 2)
#define FRAME_MHZ ((uint32_t)((uint64_t)PIX_CLOCK * 1000 / (HTOTAL * LINES)))
#define AY_CLOCK 1536000u
#define AY_DIV 4
#define SND_RATE (AY_CLOCK / 8 / AY_DIV)
#define AY_LINE (HTOTAL / 4)   // (a line's AY clocks)
#define VY0 8
#define VW 256
#define VH 240
#define Q_MAX 2048
#define OUT_MAX 2048

enum { Q_ADV, Q_LATCH0, Q_LATCH1, Q_IRQ };
typedef struct { uint32_t kind, v; } q_ev_t;

static uint8_t *s_rom, *s_srom;
static uint32_t s_rom_size, s_srom_size;
static rbx_gfx s_tiles, s_spr;
static z80 s_z, s_sz;
static const uint8_t *s_map[256], *s_smap[256];
static ay8910_t s_ay;
static int s_port[6];

static q_ev_t (*s_q_hp)[2][Q_MAX];
#define s_q (*s_q_hp)
static int s_qn[2], s_qcur;
static int32_t s_out[OUT_MAX];
static int s_out_n, s_out_pos;
static volatile int s_job_frame;

static struct {
	uint8_t vram[0x400], cram[0x400], spr0[0x40], spr1[0x20], ram[0x1000];
	uint16_t sound_command;
	uint8_t flip, irq, vector;
	unsigned long main_end;
	uint32_t frame;
	uint8_t state_end;
} t;

static struct {   // (the worker's)
	uint8_t ram[0x400], latch[2], irq;
	unsigned long z_end;
} a;

// ---- the drawing's snapshot ----
static struct { uint8_t vram[0x400], cram[0x400], spr0[0x40], spr1[0x20], flip; } s_vs;

static void capture(void)
{
	memcpy(s_vs.vram, t.vram, sizeof(s_vs.vram));
	memcpy(s_vs.cram, t.cram, sizeof(s_vs.cram));
	memcpy(s_vs.spr0, t.spr0, sizeof(s_vs.spr0));
	memcpy(s_vs.spr1, t.spr1, sizeof(s_vs.spr1));
	s_vs.flip = t.flip;
}

static void draw_tiles(int cat)
{
	uint16_t *bm = rbx_video.bitmap;
	const int flip = s_vs.flip;
	for (int pr = 0; pr < 32; pr++)
		for (int pc = 0; pc < 32; pc++) {
			const int idx = (flip ? 31 - pr : pr) * 32 + (flip ? 31 - pc : pc);
			const int attr = s_vs.cram[idx], bank = (attr >> 5) & 1;
			if (bank != cat)
				continue;
			const uint32_t code = rbx_wrap(s_vs.vram[idx] | (uint32_t)bank << 8, s_tiles.count);
			const int x = flip ? (pc * 8 + 128) & 255 : pc * 8;   // (flipped, over the 384 wide screen)
			if (cat && !s_tiles.used[code])
				continue;
			irem_blit(bm, rbx_video.pitch, 0, 0, VW, VH, s_tiles.pix + code * 64, 8, 8, ((uint32_t)(attr & 0x1f) | 0x20) * 4, cat ? 1 : 0,
			          ((attr & 0x40) != 0) ^ flip, ((attr & 0x80) != 0) ^ flip, x, pr * 8 - VY0);
		}
}

static void ob_render(void)
{
	uint16_t *bm = rbx_video.bitmap;
	for (int y = 0; y < VH; y++)
		memset(bm + y * rbx_video.pitch, 0, VW * 2);
	draw_tiles(0);
	for (int bank = 0; bank < 2; bank++) {
		const uint8_t *ram = bank ? s_vs.spr1 : s_vs.spr0;
		for (int offs = (bank ? 0x20 : 0x40) - 4; offs >= 0; offs -= 4) {
			const uint8_t *src = ram + offs;
			const int attr = src[1];
			int flipx = (attr >> 6) & 1, flipy = (attr >> 7) & 1, ypos = src[2], xpos = src[3];
			if (bank == 1)   // (fix wrap)
				ypos = ((ypos + 8) & 0xff) - 8;
			if (s_vs.flip) {
				xpos = (bank ? 240 : 248) - xpos;
				ypos = (bank ? 240 : 248) - ypos;
				flipx = !flipx;
				flipy = !flipy;
			}
			const rbx_gfx *g = bank ? &s_spr : &s_tiles;
			const uint32_t code = rbx_wrap(src[0], g->count);
			const int size = bank ? 16 : 8;
			irem_blit(bm, rbx_video.pitch, 0, 0, VW, VH, g->pix + code * (uint32_t)(size * size), size, size,
			          (bank ? 256u : 0u) + (uint32_t)(attr & 0x3f) * 4, 1, flipx, flipy, xpos, ypos - VY0);
		}
	}
	draw_tiles(1);
}

// ---- the sound queue ----
static void q_put(uint32_t kind, uint32_t v)
{
	const int c = s_qcur;
	if (s_qn[c] < Q_MAX) {
		s_q[c][s_qn[c]].kind = kind;
		s_q[c][s_qn[c]].v = v;
		s_qn[c]++;
	}
}

static void sound_command_w(int offset, uint8_t data)
{
	const uint32_t prev_lo = t.sound_command & 0x003f;
	t.sound_command = offset ? (uint16_t)((t.sound_command & 0xff00) | data) : (uint16_t)((t.sound_command & 0x00ff) | data << 8);
	const uint32_t hi = t.sound_command & 0xffc0, lo = t.sound_command & 0x003f;
	if (lo && lo != prev_lo)   // (the low bits, edge triggered: soundlatch d4-d7)
		q_put(Q_LATCH1, (uint32_t)(__builtin_clz(lo) - 16 - 10) & 0xf);
	const uint32_t c = 16 - (hi ? (uint32_t)__builtin_ctz(hi) : 16);   // (the high bits: soundlatch d0-d3)
	q_put(Q_LATCH0, c & 0xf);
}

// ---- the Z80's bus ----
static uint8_t rd(void *u, uint16_t x)
{
	(void)u;
	if (x < 0x8000)
		return s_rom[x];
	if (x < 0x8400)
		return t.vram[x - 0x8000];
	if (x < 0x8800)
		return t.cram[x - 0x8400];
	if (x >= 0xa000 && x <= 0xa005)
		return (uint8_t)rbx_port_read(s_port[x - 0xa000]);
	if (x >= 0xf000)
		return t.ram[x - 0xf000];
	return 0;
}

static void wr(void *u, uint16_t x, uint8_t v)
{
	(void)u;
	if (x >= 0x8000 && x < 0x8400)
		t.vram[x - 0x8000] = v;
	else if (x >= 0x8400 && x < 0x8800)
		t.cram[x - 0x8400] = v;
	else if (x >= 0x9000 && x < 0x9020)
		t.spr1[x - 0x9000] = v;
	else if (x >= 0x9800 && x < 0x9840)
		t.spr0[x - 0x9800] = v;
	else if (x == 0xa800 || x == 0xa801)
		sound_command_w(x & 1, v);
	else if (x == 0xa802)
		t.flip = v >> 7;
	else if (x >= 0xf000)
		t.ram[x - 0xf000] = v;
}

static uint8_t pin(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void pout(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void z_fix(z80 *z)
{
	z->read_byte = rd;
	z->write_byte = wr;
	z->port_in = pin;
	z->port_out = pout;
	z->userdata = NULL;
	z->read_map = s_map;
	z->op_rom = NULL;
}

// ---- the sound Z80 ----
static uint8_t srd(void *u, uint16_t x)
{
	(void)u;
	if (x < 0x2000)
		return s_srom[x];
	if (x >= 0x6000 && x < 0x6400)
		return a.ram[x - 0x6000];
	if (x == 0x7000)
		return (uint8_t)((a.latch[0] & 0xf) | (a.latch[1] << 4 & 0xf0));
	return 0;
}

static void swr(void *u, uint16_t x, uint8_t v)
{
	(void)u;
	if (x >= 0x6000 && x < 0x6400)
		a.ram[x - 0x6000] = v;
	else if (x == 0x7000 || x == 0x7001)
		ay8910_write(&s_ay, x & 1, v);
	else if (x == 0x7006 && (v & 0x80))   // (sample_start_w: the HC55516 is silent here)
		a.latch[1] = 0;
}

static uint8_t sin_(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void sout(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void sz_fix(z80 *z)
{
	z->read_byte = srd;
	z->write_byte = swr;
	z->port_in = sin_;
	z->port_out = sout;
	z->userdata = NULL;
	z->read_map = s_smap;
	z->op_rom = NULL;
}

static void sound_run(void)
{
	a.z_end += LINE_CYC;
	int irq = a.irq;
	z80_run(&s_sz, a.z_end, &irq, 1);
	a.irq = (uint8_t)irq;
	int32_t mix[16];
	memset(mix, 0, sizeof(mix));
	const int n = ay8910_render(&s_ay, AY_LINE, mix, 16);
	for (int i = 0; i < n && s_out_pos < OUT_MAX; i++)
		s_out[s_out_pos++] = mix[i];
}

static void job(void)
{
	if (!__atomic_load_n(&s_job_frame, __ATOMIC_ACQUIRE))
		return;
	const int b = s_qcur ^ 1;
	s_out_pos = 0;
	for (int i = 0; i < s_qn[b]; i++) {
		const q_ev_t *ev = &s_q[b][i];
		switch (ev->kind) {
		case Q_ADV: sound_run(); break;
		case Q_LATCH0: a.latch[0] = (uint8_t)ev->v; break;
		case Q_LATCH1: a.latch[1] = (uint8_t)ev->v; break;
		case Q_IRQ: a.irq = 1; break;
		}
	}
	s_qn[b] = 0;
	s_out_n = s_out_pos;
	__atomic_store_n(&s_job_frame, 0, __ATOMIC_RELEASE);
}

static void sound_frame(void)
{
	rbx_aux_sync();
	for (int i = 0; i < s_out_n; i++)
		rbx_audio_put(s_out[i]);
	s_out_n = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void ob_exit(void);
static void ob_reset(void);
static const uint8_t *s_prom;
static int s_conf;

static void palette(void)
{
	static const int res[3] = { 1000, 470, 220 };
	double wr3[3], wb2[2];
	const double s3 = irem_res_raw(res, 3, 0, wr3), s2 = irem_res_raw(res + 1, 2, 0, wb2);
	const double scale = 255.0 / (s3 > s2 ? s3 : s2);
	uint32_t c[32];
	for (int i = 0; i < 32; i++) {
		const int v = s_prom[i];
		const int b = (int)(scale * (wb2[0] * ((v >> 6) & 1) + wb2[1] * ((v >> 7) & 1)) + 0.5);
		c[i] = (uint32_t)(irem_res3(wr3, scale, v) << 16 | irem_res3(wr3, scale, v >> 3) << 8 | b);
	}
	const int bank = (rbx_port_read(s_conf) & 1) ? 0x10 : 0;   // (adjust_palette)
	for (int i = 0; i < 0x200; i++)
		rbx_video.palette[i] = c[(s_prom[0x20 + i] & 0xf) | bank];
}

static int ob_init(const rbx_game *g)
{
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t s8, s16, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("audiocpu", &s_srom_size);
	const uint8_t *g8 = rbx_region("gfx8x8", &s8), *g16 = rbx_region("gfx16x16", &s16);
	s_prom = rbx_region("proms", &ps);
	if (!s_rom || s_rom_size < 0x8000 || !s_srom || s_srom_size < 0x2000 || !g8 || !g16 || !s_prom || ps < 0x220) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	uint32_t p8[2] = { s8 * 4, 0 }, p16[2] = { s16 * 4, 0 }, x8[8], y16[16], x16[16];
	for (int i = 0; i < 16; i++) {
		y16[i] = (uint32_t)i * 8;
		x16[i] = (uint32_t)(i < 8 ? 7 - i : 128 + 15 - i);
		if (i < 8)
			x8[i] = (uint32_t)(7 - i);
	}
	const rbx_gfx_layout l8 = { 8, 8, 2, p8, x8, y16, 8 * 8 }, l16 = { 16, 16, 2, p16, x16, y16, 32 * 8 };
	const int bad = rbx_gfx_decode(&s_tiles, &l8, g8, s8 / 2) | rbx_gfx_decode(&s_spr, &l16, g16, s16 / 2);
	static const char *const pn[6] = { "IN0", "IN1", "IN2", "DSW0", "DSW1", "DSW2" };
	for (int i = 0; i < 6; i++)
		s_port[i] = rbx_port_find(pn[i]);
	s_conf = rbx_port_find("CONF");
	ay8910_init(&s_ay, AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 128, AY_DIV, 0);
	if (bad || rbx_audio_setup(SND_RATE) || rbx_video_setup(VW, VH, 0x200)) {
		rbx_fail("%s: out of memory", g->set);
		ob_exit();
		return -1;
	}
	rbx_audio.frame_mhz = FRAME_MHZ;
	ob_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void ob_exit(void)
{
	rbx_aux_set_job(NULL);
	rbx_gfx_free(&s_tiles);
	rbx_gfx_free(&s_spr);
	free(s_q_hp);
	s_q_hp = NULL;
}

static void ob_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	memset(&a, 0, sizeof(a));
	memset(&s_vs, 0, sizeof(s_vs));
	palette();
	ay8910_reset(&s_ay);
	z80_init(&s_z);
	z_fix(&s_z);
	s_z.int_data = 0xff;
	memset(s_map, 0, sizeof(s_map));
	for (int p = 0; p < 0x80; p++)
		s_map[p] = s_rom + p * 256;
	for (int p = 0; p < 0x10; p++)
		s_map[0xf0 + p] = t.ram + p * 256;
	z80_init(&s_sz);
	sz_fix(&s_sz);
	s_sz.int_data = 0xff;
	memset(s_smap, 0, sizeof(s_smap));
	for (int p = 0; p < 0x20; p++)
		s_smap[p] = s_srom + p * 256;
	for (int p = 0; p < 4; p++)
		s_smap[0x60 + p] = a.ram + p * 256;
}

static void ob_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		if (line == VY0 + VH) {   // (the update)
			rbx_video_wait_idle();
			capture();
			rbx_video_frame_done();
		}
		if (line == 248 || line == 128) {   // (scanline: RST 10h at the vblank, RST 08h half way)
			t.irq = 1;
			s_z.int_data = line == 248 ? 0xd7 : 0xcf;
			q_put(Q_IRQ, 0);
		}
		t.main_end += LINE_CYC;
		int irq = t.irq;
		z80_run(&s_z, t.main_end, &irq, 1);
		t.irq = (uint8_t)irq;
		q_put(Q_ADV, 0);
	}
	sound_frame();
	t.frame++;
}

static int ob_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "ob", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "ob_a", &a, sizeof(a));
	rc |= rbx_state_io(ctx, "ob_ay", &s_ay, ay8910_state_size());
	int32_t n = s_out_n;
	rc |= rbx_state_io(ctx, "ob_n", &n, sizeof(n));
	rc |= rbx_state_io(ctx, "ob_out", s_out, sizeof(s_out));
	z80 z = s_z, sz = s_sz;
	rc |= rbx_state_io(ctx, "ob_z", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "ob_sz", &sz, sizeof(sz));
	if (!rc && load) {
		s_out_n = n >= 0 && n <= OUT_MAX ? n : 0;
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
		z_fix(&z);
		sz_fix(&sz);
		s_z = z;
		s_sz = sz;
	}
	return rc ? -1 : 0;
}

static int ob_save(void *ctx) { return ob_state(ctx, 0); }
static int ob_load(void *ctx) { return ob_state(ctx, 1); }
static int ob_supported(const char *machine) { return !strcmp(machine, "olibochu"); }

const rbx_driver rbx_drv_olibochu = { "Irem Oli-Boo-Chu", ob_init, ob_exit, ob_reset, ob_frame, ob_save, ob_load, ob_supported,
                                      ob_render, NULL };
