/*  Tecfri's bootleg hardware of Spartan X. Ported from MAME src/mame/irem/spartanxtec.cpp (BSD-3-Clause, copyright-holders:
    David Haywood). A Z80, the Kung-Fu Master's tilemap with four scroll groups, sprites between its two categories; the
    sound: a second Z80 with three AY-8912s, a frame behind on the worker (rbx_aux).
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

#define CPU_CLOCK 4000000u
#define AY_CLOCK 1000000u
#define AY_DIV 2
#define SND_RATE (AY_CLOCK / 8 / AY_DIV)
#define LINES 256              // (set_refresh_hz(60), set_size(512, 256))
#define LINE_HZ (60u * LINES)
#define VX0 128
#define VW 256
#define VH 240
#define Q_MAX 2048
#define OUT_MAX 2048

enum { Q_ADV, Q_LATCH };
typedef struct { uint32_t kind, v; } q_ev_t;

static uint8_t *s_rom, *s_srom;
static uint32_t s_rom_size, s_srom_size;
static rbx_gfx s_tiles, s_spr;
static z80 s_z, s_sz;
static const uint8_t *s_map[256], *s_smap[256];
static ay8910_t s_ay[3];
static int s_port[4];

static q_ev_t (*s_q_hp)[2][Q_MAX];
#define s_q (*s_q_hp)
static int s_qn[2], s_qcur;
static int32_t s_out[OUT_MAX];
static int s_out_n, s_out_pos;
static volatile int s_job_frame;

static struct {
	uint8_t spr[0x400], tile[0x1000], ram[0x1000], slo[4], shi[4], irq;
	uint32_t cpu_frac, ay_frac;
	unsigned long main_end;
	uint32_t frame;
	uint8_t state_end;
} t;

static struct {   // (the worker's)
	uint8_t ram[0x400], latch, irq;
	uint32_t z_frac, irq_acc;
	unsigned long z_end;
} a;

// ---- the drawing's snapshot ----
static struct { uint8_t spr[0x400], tile[0x1000], slo[4], shi[4]; } s_vs;

static void capture(void)
{
	memcpy(s_vs.spr, t.spr, sizeof(s_vs.spr));
	memcpy(s_vs.tile, t.tile, sizeof(s_vs.tile));
	memcpy(s_vs.slo, t.slo, 4);
	memcpy(s_vs.shi, t.shi, 4);
}

static void draw_tiles(int cat)
{
	uint16_t *bm = rbx_video.bitmap;
	for (int r = 0; r < 30; r++) {   // (the rows past the visible area keep a scroll of 0)
		const int g = r + 2;   // ((i - 2) & 0xff from the row 2 on: four groups of 8)
		const int rs = g < 32 ? (s_vs.slo[g / 8] | s_vs.shi[g / 8] << 8) + 28 - 128 : 0;
		for (int c = 0; c < 64; c++) {
			const int x = (c * 8 - rs - VX0) & 511;
			if (x >= VW && x < 504)
				continue;
			const int idx = r * 64 + c, color = s_vs.tile[idx + 0x800];
			const int category = r < 6 || ((color & 0x1f) >> 1) > 0x0c;
			if (category != cat)
				continue;
			const uint32_t code = rbx_wrap(s_vs.tile[idx] | (uint32_t)(color & 0xc0) << 2, s_tiles.count);
			irem_blit(bm, rbx_video.pitch, 0, 0, VW, VH, s_tiles.pix + code * 64, 8, 8, 0x100 + (uint32_t)(color & 0x1f) * 8, 0, color & 0x20, 0,
			          x >= 504 ? x - 512 : x, r * 8);
		}
	}
}

static void sx_render(void)
{
	draw_tiles(0);
	for (int i = 0; i < 0x400; i += 4) {
		const uint8_t *sr = s_vs.spr + i;
		const int x = sr[2] + 128, y = (225 - sr[1]) & 0xff, attr = sr[3];
		const uint32_t code = rbx_wrap(sr[0] | (uint32_t)(attr & 0xc0) << 2, s_spr.count);
		irem_blit(rbx_video.bitmap, rbx_video.pitch, 0, 0, VW, VH, s_spr.pix + code * 256, 16, 16, (uint32_t)(attr & 0x1f) * 8, 1u << 7,
		          attr & 0x20, 0, x - VX0, y);
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

// ---- the Z80's bus ----
static uint8_t rd(void *u, uint16_t x)
{
	(void)u;
	if (x < 0x8000)
		return s_rom[x];
	if (x >= 0x8100 && x <= 0x8103)
		return (uint8_t)rbx_port_read(s_port[x - 0x8100]);
	if (x >= 0xa900 && x <= 0xa903)
		return t.slo[x - 0xa900];
	if (x >= 0xa980 && x <= 0xa983)
		return t.shi[x - 0xa980];
	if (x >= 0xc400 && x < 0xc800)
		return t.spr[x - 0xc400];
	if (x >= 0xd000 && x < 0xe000)
		return t.tile[x - 0xd000];
	if (x >= 0xe000 && x < 0xf000)
		return t.ram[x - 0xe000];
	return 0;
}

static void wr(void *u, uint16_t x, uint8_t v)
{
	(void)u;
	if (x == 0x8000)
		q_put(Q_LATCH, v);
	else if (x == 0x8200)
		t.irq = 0;
	else if (x >= 0xa900 && x <= 0xa903)
		t.slo[x - 0xa900] = v;
	else if (x >= 0xa980 && x <= 0xa983)
		t.shi[x - 0xa980] = v;
	else if (x >= 0xc400 && x < 0xc800)
		t.spr[x - 0xc400] = v;
	else if (x >= 0xd000 && x < 0xe000)
		t.tile[x - 0xd000] = v;
	else if (x >= 0xe000 && x < 0xf000)
		t.ram[x - 0xe000] = v;
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
	if (x < 0x1000)
		return s_srom[x];
	if (x >= 0x8000 && x < 0x8400)
		return a.ram[x - 0x8000];
	return x == 0xc000 ? a.latch : 0;
}

static void swr(void *u, uint16_t x, uint8_t v)
{
	(void)u;
	if (x >= 0x8000 && x < 0x8400)
		a.ram[x - 0x8000] = v;
}

static uint8_t sin_(z80 *z, uint8_t p)
{
	(void)z;
	switch (p) {
	case 0x12: return ay8910_data_r(&s_ay[2]);
	case 0x14: return ay8910_data_r(&s_ay[0]);
	case 0x18: return ay8910_data_r(&s_ay[1]);
	}
	return 0xff;
}

static void sout(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (p == 0)
		a.irq = 0;
	else if (p == 0x12 || p == 0x13)
		ay8910_write(&s_ay[2], p & 1, v);
	else if (p == 0x14 || p == 0x15)
		ay8910_write(&s_ay[0], p & 1, v);
	else if (p == 0x18 || p == 0x19)
		ay8910_write(&s_ay[1], p & 1, v);
}

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

static void sound_run(uint32_t ayc)
{
	a.irq_acc += ayc;
	if (a.irq_acc >= AY_CLOCK / 1000) {   // (the periodic IRQ at 1 kHz: irq0_line_assert)
		a.irq_acc -= AY_CLOCK / 1000;
		a.irq = 1;
	}
	a.z_end += ayc * (CPU_CLOCK / AY_CLOCK);
	int irq = a.irq;
	z80_run(&s_sz, a.z_end, &irq, 0);
	int32_t mix[64];
	const int max = ay8910_out_max(&s_ay[0], ayc);
	if (max > 64)
		return;
	memset(mix, 0, (size_t)max * sizeof(mix[0]));
	const int n = ay8910_render(&s_ay[0], ayc, mix, max);
	ay8910_render(&s_ay[1], ayc, mix, max);
	ay8910_render(&s_ay[2], ayc, mix, max);
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
		if (ev->kind == Q_ADV)
			sound_run(ev->v);
		else {
			a.latch = (uint8_t)ev->v;
			z80_gen_nmi(&s_sz);
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
static void sx_exit(void);
static void sx_reset(void);

static int sx_init(const rbx_game *g)
{
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t s1, s2, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("audiocpu", &s_srom_size);
	const uint8_t *g1 = rbx_region("gfx1", &s1), *g2 = rbx_region("gfx2", &s2), *prom = rbx_region("cprom", &ps);
	if (!s_rom || !s_srom || s_srom_size < 0x1000 || !g1 || !g2 || !prom || ps < 0x600) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	uint32_t x8[8], y16[16], x16[16], p1[3] = { 0, s1 / 3 * 8, s1 / 3 * 16 }, p2[3] = { 0, s2 / 3 * 8, s2 / 3 * 16 };
	for (int i = 0; i < 16; i++) {
		y16[i] = (uint32_t)i * 8;
		x16[i] = (uint32_t)(i < 8 ? i : 128 + i - 8);
		if (i < 8)
			x8[i] = (uint32_t)i;
	}
	const rbx_gfx_layout tl = { 8, 8, 3, p1, x8, y16, 8 * 8 }, sl = { 16, 16, 3, p2, x16, y16, 32 * 8 };
	const int bad = rbx_gfx_decode(&s_tiles, &tl, g1, s1 / 3) | rbx_gfx_decode(&s_spr, &sl, g2, s2 / 3);
	if (!bad) {   // (ROMREGION_INVERT)
		for (uint32_t i = 0; i < s_tiles.count * 64; i++)
			s_tiles.pix[i] ^= 7;
		for (uint32_t i = 0; i < s_spr.count * 256; i++)
			s_spr.pix[i] ^= 7;
	}
	static const char *const pn[4] = { "DSW1", "DSW2", "SYSTEM", "P1" };
	for (int i = 0; i < 4; i++)
		s_port[i] = rbx_port_find(pn[i]);
	for (int i = 0; i < 3; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 64, AY_DIV, 0);
	if (bad || rbx_audio_setup(SND_RATE) || rbx_video_setup(VW, VH, 0x200)) {
		rbx_fail("%s: out of memory", g->set);
		sx_exit();
		return -1;
	}
	for (int i = 0; i < 0x200; i++) {   // (pal4bit)
		const uint32_t b = prom[i] & 15, gg = prom[0x200 + i] & 15, r = prom[0x400 + i] & 15;
		rbx_video.palette[i] = (r * 17) << 16 | (gg * 17) << 8 | (b * 17);
	}
	sx_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void sx_exit(void)
{
	rbx_aux_set_job(NULL);
	rbx_gfx_free(&s_tiles);
	rbx_gfx_free(&s_spr);
	free(s_q_hp);
	s_q_hp = NULL;
}

static void sx_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	memset(&a, 0, sizeof(a));
	memset(&s_vs, 0, sizeof(s_vs));
	for (int i = 0; i < 3; i++)
		ay8910_reset(&s_ay[i]);
	z80_init(&s_z);
	z_fix(&s_z);
	s_z.int_data = 0xff;
	memset(s_map, 0, sizeof(s_map));
	for (int p = 0; p < 0x80; p++)
		s_map[p] = s_rom + p * 256;
	for (int p = 0; p < 0x10; p++) {
		s_map[0xd0 + p] = t.tile + p * 256;
		s_map[0xe0 + p] = t.ram + p * 256;
	}
	z80_init(&s_sz);
	sz_fix(&s_sz);
	s_sz.int_data = 0xff;
	memset(s_smap, 0, sizeof(s_smap));
	for (int p = 0; p < 0x10; p++)
		s_smap[p] = s_srom + p * 256;
}

static void sx_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		if (line == VH) {   // (the update; irq0_line_assert)
			rbx_video_wait_idle();
			capture();
			rbx_video_frame_done();
			t.irq = 1;
		}
		t.cpu_frac += CPU_CLOCK;
		const uint32_t c = t.cpu_frac / LINE_HZ;
		t.cpu_frac -= c * LINE_HZ;
		t.main_end += c;
		int irq = t.irq;
		z80_run(&s_z, t.main_end, &irq, 0);
		t.ay_frac += AY_CLOCK;
		const uint32_t ac = t.ay_frac / LINE_HZ;
		t.ay_frac -= ac * LINE_HZ;
		q_put(Q_ADV, ac);
	}
	sound_frame();
	t.frame++;
}

static int sx_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "sx", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "sx_a", &a, sizeof(a));
	for (int i = 0; i < 3; i++)
		rc |= rbx_state_io(ctx, "sx_ay", &s_ay[i], ay8910_state_size());
	int32_t n = s_out_n;
	rc |= rbx_state_io(ctx, "sx_n", &n, sizeof(n));
	rc |= rbx_state_io(ctx, "sx_out", s_out, sizeof(s_out));
	z80 z = s_z, sz = s_sz;
	rc |= rbx_state_io(ctx, "sx_z", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "sx_sz", &sz, sizeof(sz));
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

static int sx_save(void *ctx) { return sx_state(ctx, 0); }
static int sx_load(void *ctx) { return sx_state(ctx, 1); }
static int sx_supported(const char *machine) { return !strcmp(machine, "spartanxtec"); }

const rbx_driver rbx_drv_spartanxtec = { "Irem Spartan X (Tecfri)", sx_init, sx_exit, sx_reset, sx_frame, sx_save, sx_load,
                                         sx_supported, sx_render, NULL };
