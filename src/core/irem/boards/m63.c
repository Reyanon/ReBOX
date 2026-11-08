/*  Irem M63: Wily Tower, Atomic Boy and Fighting Basketball. Ported from MAME src/mame/irem/m63.cpp (BSD-3-Clause,
    copyright-holders: Nicola Salmoria). A Z80 at 3 MHz, a 32x32 background of 3 bpp tiles
    with column scroll, wrapping 16x16 sprites and a 2 bpp text layer; the sound: an I8039 with two AY-3-8910s (one and raw
    samples on Fighting Basketball), a frame behind on the worker (rbx_aux).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/mcs48/mcs48.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"
#include "../../video/gfxlayout.h"
#include "../video/irem_draw.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 3000000u     // (12 MHz / 4; the I8039 the same, the AYs 12 MHz / 8)
#define AY_CLOCK 1500000u
#define AY_DIV 4
#define SND_RATE (AY_CLOCK / 8 / AY_DIV)
#define LINES 256              // (set_refresh_hz(60), set_size(256, 256), set_vblank_time(0))
#define LINE_HZ (60u * LINES)
#define VY0 16
#define VW 256
#define VH 224
#define NPENS 260
#define Q_MAX 2048
#define OUT_MAX 2048
#define SAMPLE_LEN 0x2000
#define SAMPLE_RATE 8000u

enum { Q_ADV, Q_LATCH, Q_IRQ, Q_SAMPLE };
typedef struct { uint32_t kind, v; } q_ev_t;

static uint8_t *s_rom, *s_srom, *s_user1, *s_samples;
static uint32_t s_rom_size, s_srom_size, s_user1_size, s_samples_size;
static rbx_gfx s_fg, s_bg, s_spr;
static z80 s_z;
static const uint8_t *s_map[256];
static mcs48 s_mcs;
static ay8910_t s_ay[2];
static int s_fght, s_sy_offset, s_irq_hz, s_port[4];
static uint32_t s_fg_base;

static q_ev_t s_q[2][Q_MAX];
static int s_qn[2], s_qcur;
static int32_t s_out[OUT_MAX];
static int s_out_n, s_out_pos;
static volatile int s_job_frame;

static struct {
	uint8_t vid[0x1000], ram[0x1000];   // (vid: 0 RAM, 200 sprites, 300 scroll, 400 text, 800 tiles, c00 colours)
	uint8_t nmi_mask, flip, pal_bank, fg_flag, status_pub;
	uint64_t cpu_frac, ay_frac;
	unsigned long main_end;
	uint32_t frame;
	uint8_t state_end;
} t;

static struct {   // (the worker's)
	uint8_t latch, sound_irq, status, p1, p2;
	int32_t mcs_left, sample_pos;
	uint32_t mcs_frac, irq_acc, sample_frac;
} a;

// ---- the drawing's snapshot ----
static struct { uint8_t vid[0x1000], flip, pal_bank, fg_flag; } s_vs;

static void capture(void)
{
	memcpy(s_vs.vid, t.vid, sizeof(s_vs.vid));
	s_vs.flip = t.flip;
	s_vs.pal_bank = t.pal_bank;
	s_vs.fg_flag = t.fg_flag;
}

static void m63_render(void)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, flip = s_vs.flip;
	const uint8_t *scroll = s_vs.vid + 0x300, *vr2 = s_vs.vid + 0x400, *vr = s_vs.vid + 0x800, *cr = s_vs.vid + 0xc00;
	for (int pc = 0; pc < 32; pc++) {   // (the background; flipped, MAME's effective scroll: size - map size + scroll)
		const int c = flip ? 31 - pc : pc;
		const int vy = flip ? scroll[c * 8] : -scroll[c * 8];
		for (int pr = 0; pr < 32; pr++) {
			const int r = flip ? 31 - pr : pr, idx = r * 32 + c;
			const int y = ((pr * 8 + vy) & 255) - VY0;
			const int attr = cr[idx];
			const uint32_t code = rbx_wrap(vr[idx] | (uint32_t)(attr & 0x30) << 4, s_bg.count);
			const uint32_t base = (uint32_t)((attr & 0x0f) + (s_vs.pal_bank << 4)) % 32 * 8;
			const uint8_t *src = s_bg.pix + code * 64;
			irem_blit(bm, pitch, 0, 0, VW, VH, src, 8, 8, base, 0, flip, flip, pc * 8, y);
			if (y > 248 - VY0)
				irem_blit(bm, pitch, 0, 0, VW, VH, src, 8, 8, base, 0, flip, flip, pc * 8, y - 256);
		}
	}
	const uint8_t *sr = s_vs.vid + 0x200;
	for (int offs = 0; offs < 0x100; offs += 4, sr += 4) {
		const uint32_t code = rbx_wrap(sr[1] | (uint32_t)(sr[2] & 0x10) << 4, s_spr.count);
		const uint32_t color = (uint32_t)((sr[2] & 0x0f) + (s_vs.pal_bank << 4)) % 32;
		int flipx = sr[2] & 0x20, flipy = 0;
		int sx = sr[3], sy = s_sy_offset - sr[0];
		if (flip) {
			sx = 240 - sx;
			sy = s_sy_offset - sy;
			flipx = !flipx;
			flipy = !flipy;
		}
		const uint8_t *src = s_spr.pix + code * 256;
		if (!s_spr.used[code])
			continue;
		irem_blit(bm, pitch, 0, 0, VW, VH, src, 16, 16, color * 8, 1, flipx, flipy, sx, sy - VY0);
		if (sx > 0xf0)   // (sprite wrapping, verified on real hardware)
			irem_blit(bm, pitch, 0, 0, VW, VH, src, 16, 16, color * 8, 1, flipx, flipy, sx - 0x100, sy - VY0);
	}
	for (int pr = 2; pr < 30; pr++)
		for (int pc = 0; pc < 32; pc++) {
			const int idx = (flip ? 31 - pr : pr) * 32 + (flip ? 31 - pc : pc);
			const uint32_t code = rbx_wrap(vr2[idx], s_fg.count);
			if (s_fg.used[code])
				irem_blit(bm, pitch, 0, 0, VW, VH, s_fg.pix + code * 64, 8, 8, s_fg_base, 1, (s_vs.fg_flag != 0) ^ flip, flip, pc * 8,
				          pr * 8 - VY0);
		}
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
static inline void latch_w(int bit, int v)
{
	if (s_fght) {
		if (bit == 1)
			t.nmi_mask = (uint8_t)v;
		else if (bit == 2) {
			t.flip = (uint8_t)v;
			t.fg_flag = (uint8_t)v;
		}
	} else if (bit == 0)
		t.nmi_mask = (uint8_t)v;
	else if (bit == 2)
		t.flip = (uint8_t)!v;
	else if (bit == 3)
		t.pal_bank = (uint8_t)v;
}

static uint8_t rd(void *u, uint16_t a)
{
	(void)u;
	if (s_fght) {
		if (a < 0x6000 || (a >= 0x8000 && a < 0xc000))
			return s_rom[a];
		if (a >= 0xc000 && a < 0xc800)
			return t.ram[a - 0xc000];
		if (a >= 0xd000 && a < 0xe000)
			return t.vid[a - 0xd000];
		switch (a) {
		case 0xf000: return t.status_pub;
		case 0xf001: return (uint8_t)rbx_port_read(s_port[0]);
		case 0xf002: return (uint8_t)rbx_port_read(s_port[1]);
		case 0xf003: return (uint8_t)rbx_port_read(s_port[2]);
		}
		return 0;
	}
	if (a < 0xc000)
		return s_rom[a];
	if (a >= 0xd000 && a < 0xe000)
		return t.ram[a - 0xd000];
	if (a >= 0xe000 && a < 0xf000)
		return t.vid[a - 0xe000];
	switch (a) {
	case 0xf800: return (uint8_t)rbx_port_read(s_port[0]);
	case 0xf801: return (uint8_t)rbx_port_read(s_port[1]);
	case 0xf802: return (uint8_t)rbx_port_read(s_port[2]);
	case 0xf806: return (uint8_t)rbx_port_read(s_port[3]);
	}
	return 0;
}

static void wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_fght) {
		if (a >= 0xc000 && a < 0xc800)
			t.ram[a - 0xc000] = v;
		else if (a >= 0xd000 && a < 0xe000)
			t.vid[a - 0xd000] = v;
		else if (a == 0xf000)
			q_put(Q_IRQ, 0);
		else if (a == 0xf002)
			q_put(Q_LATCH, v);
		else if (a >= 0xf800 && a < 0xf808) {
			latch_w(a & 7, v & 1);
			if (a == 0xf807 && (v & 1))   // (fghtbskt_samples_w)
				q_put(Q_SAMPLE, (uint32_t)(v & 0xf0) << 8);
		}
		return;
	}
	if (a >= 0xd000 && a < 0xe000)
		t.ram[a - 0xd000] = v;
	else if (a >= 0xe000 && a < 0xf000)
		t.vid[a - 0xe000] = v;
	else if (a >= 0xf000 && a < 0xf008)
		latch_w(a & 7, v & 1);
	else if (a == 0xf800)
		q_put(Q_LATCH, v);
	else if (a == 0xf803)
		q_put(Q_IRQ, 0);
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

// ---- the I8039 ----
static uint8_t snd_r(void *u, uint8_t off)
{
	(void)u;
	switch (a.p2 & 0xf0) {
	case 0x60: return a.latch;
	case 0x70: return s_user1[(uint32_t)((a.p1 & 0x1f) << 8 | off) % s_user1_size];
	}
	return 0xff;
}

static void snd_w(void *u, uint8_t off, uint8_t v)
{
	(void)u;
	(void)v;
	if ((a.p2 & 0xf0) == 0xe0)
		ay8910_address_w(&s_ay[0], off);
	else if ((a.p2 & 0xf0) == 0xa0)
		ay8910_data_w(&s_ay[0], off);
	else if (!s_fght && (a.p1 & 0xe0) == 0x60)
		ay8910_address_w(&s_ay[1], off);
	else if (!s_fght && (a.p1 & 0xe0) == 0x40)
		ay8910_data_w(&s_ay[1], off);
	else if ((a.p2 & 0xf0) == 0x70)
		a.status = off;
}

static void snd_port_w(void *u, int port, uint8_t v)
{
	(void)u;
	if (port == 1)
		a.p1 = v;
	else {
		a.p2 = v;
		if ((v & 0xf0) == 0x50)
			mcs48_set_irq(&s_mcs, 0);
	}
}

static int snd_test(void *u, int tn)
{
	(void)u;
	if (tn == 1 && a.sound_irq) {
		a.sound_irq = 0;
		return 1;
	}
	return 0;
}

// (a line's AY clocks: the I8039 at 200000 cycles a second, its periodic T1, the AYs, the samples)
static void sound_run(uint32_t ayc)
{
	a.mcs_frac += ayc * 2;
	const int mc = (int)(a.mcs_frac / 15);
	a.mcs_frac -= (uint32_t)mc * 15;
	a.mcs_left += mc;
	if (a.mcs_left > 0)
		a.mcs_left -= mcs48_execute(&s_mcs, a.mcs_left);
	a.irq_acc += ayc * (uint32_t)s_irq_hz;
	if (a.irq_acc >= AY_CLOCK) {
		a.irq_acc -= AY_CLOCK;
		a.sound_irq = 1;
	}
	int32_t mix[64];
	const int max = ay8910_out_max(&s_ay[0], ayc);
	if (max > 64)
		return;
	memset(mix, 0, (size_t)max * sizeof(mix[0]));
	const int n = ay8910_render(&s_ay[0], ayc, mix, max);
	if (!s_fght)
		ay8910_render(&s_ay[1], ayc, mix, max);
	for (int i = 0; i < n && s_out_pos < OUT_MAX; i++) {
		int32_t x = mix[i];
		if (s_fght && a.sample_pos >= 0) {
			const uint32_t pos = (uint32_t)a.sample_pos + (a.sample_frac >> 8) / SND_RATE;
			if (pos < s_samples_size && (pos & (SAMPLE_LEN - 1)) != SAMPLE_LEN - 1)
				x += (int8_t)(s_samples[pos] ^ 0x80) * 128;   // (at 0.50)
			else
				a.sample_pos = -1;
			a.sample_frac += SAMPLE_RATE << 8;
		}
		s_out[s_out_pos++] = x;
	}
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
		case Q_ADV: sound_run(ev->v); break;
		case Q_LATCH: a.latch = (uint8_t)ev->v; break;
		case Q_IRQ: mcs48_set_irq(&s_mcs, 1); break;
		case Q_SAMPLE:
			a.sample_pos = (int32_t)ev->v;
			a.sample_frac = 0;
			break;
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
	t.status_pub = a.status;   // (the main CPU sees it at a frame's end: the same at every run)
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void m63_exit(void);
static void m63_reset(void);

static void palette(const uint8_t *prom)
{
	uint32_t *pal = rbx_video.palette;
	#define B(v, n) (((v) >> (n)) & 1)
	#define C4(v) (0x0e * B(v, 0) + 0x1f * B(v, 1) + 0x43 * B(v, 2) + 0x8f * B(v, 3))
	for (int i = 0; i < 256; i++)
		pal[i] = (uint32_t)(C4(prom[i]) << 16 | C4(prom[i + 256]) << 8 | C4(prom[i + 512]));
	if (!s_fght)
		for (int i = 0; i < 4; i++) {
			const int v = prom[768 + i];
			const int r = 0x21 * B(v, 0) + 0x47 * B(v, 1) + 0x97 * B(v, 2);
			const int g = 0x21 * B(v, 3) + 0x47 * B(v, 4) + 0x97 * B(v, 5);
			const int b = 0x52 * B(v, 6) + 0xad * B(v, 7);
			pal[256 + i] = (uint32_t)(r << 16 | g << 8 | b);
		}
	#undef C4
	#undef B
}

static int m63_init(const rbx_game *g)
{
	uint32_t s1, s2, s3, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("soundcpu", &s_srom_size);
	s_user1 = rbx_region("user1", &s_user1_size);
	s_samples = rbx_region("samples", &s_samples_size);
	const uint8_t *g1 = rbx_region("gfx1", &s1), *g2 = rbx_region("gfx2", &s2), *g3 = rbx_region("gfx3", &s3);
	const uint8_t *prom = rbx_region("proms", &ps);
	s_fght = !strcmp(g->machine, "fghtbskt");
	if (!s_rom || s_rom_size < 0xc000 || !s_srom || !s_user1 || !g1 || !g2 || !g3 || !prom || ps < 0x300 || (s_fght && !s_samples)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	s_sy_offset = s_fght ? 240 : 238;
	s_irq_hz = !strcmp(g->machine, "m63") ? 60 : 30;
	s_fg_base = s_fght ? 16 : 256;
	uint32_t x8[8], y16[16], p2[2] = { s1 * 4, 0 }, p3[3] = { s2 / 3 * 16, s2 / 3 * 8, 0 }, ps3[3] = { s3 / 3 * 16, s3 / 3 * 8, 0 }, x16[16];
	for (int i = 0; i < 16; i++) {
		y16[i] = (uint32_t)i * 8;
		x16[i] = i < 8 ? (uint32_t)i : s3 / 6 * 8 + (uint32_t)i - 8;
		if (i < 8)
			x8[i] = (uint32_t)i;
	}
	const rbx_gfx_layout l2 = { 8, 8, 2, p2, x8, y16, 64 }, l3 = { 8, 8, 3, p3, x8, y16, 64 }, ls = { 16, 16, 3, ps3, x16, y16, 16 * 8 };
	const int bad = rbx_gfx_decode(&s_fg, &l2, g1, s1 / 2) | rbx_gfx_decode(&s_bg, &l3, g2, s2 / 3) | rbx_gfx_decode(&s_spr, &ls, g3, s3 / 6);
	static const char *const pn[2][4] = { { "P1", "P2", "DSW1", "DSW2" }, { "P1", "P2", "DSW", "DSW" } };
	for (int i = 0; i < 4; i++)
		s_port[i] = rbx_port_find(pn[s_fght][i]);
	for (int i = 0; i < 2; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, s_fght ? 256 : (i ? 256 : 64), AY_DIV, 0);
	if (bad || rbx_audio_setup(SND_RATE) || rbx_video_setup(VW, VH, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		m63_exit();
		return -1;
	}
	palette(prom);
	mcs48_init(&s_mcs, MCS48_I8039);
	s_mcs.rom = s_srom;
	s_mcs.rom_mask = 0xfff;
	s_mcs.ext_r = snd_r;
	s_mcs.ext_w = snd_w;
	s_mcs.port_w = snd_port_w;
	s_mcs.test_r = snd_test;
	m63_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void m63_exit(void)
{
	rbx_aux_set_job(NULL);
	rbx_gfx_free(&s_fg);
	rbx_gfx_free(&s_bg);
	rbx_gfx_free(&s_spr);
}

static void m63_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	memset(&a, 0, sizeof(a));
	a.sample_pos = -1;
	memset(&s_vs, 0, sizeof(s_vs));
	ay8910_reset(&s_ay[0]);
	ay8910_reset(&s_ay[1]);
	mcs48_reset(&s_mcs);
	z80_init(&s_z);
	z_fix(&s_z);
	s_z.int_data = 0xff;
	memset(s_map, 0, sizeof(s_map));
	if (s_fght) {
		for (int p = 0; p < 0x60; p++)
			s_map[p] = s_rom + p * 256;
		for (int p = 0x80; p < 0xc0; p++)
			s_map[p] = s_rom + p * 256;
		for (int p = 0; p < 8; p++)
			s_map[0xc0 + p] = t.ram + p * 256;
		for (int p = 0; p < 0x10; p++)
			s_map[0xd0 + p] = t.vid + p * 256;
	} else {
		for (int p = 0; p < 0xc0; p++)
			s_map[p] = s_rom + p * 256;
		for (int p = 0; p < 0x10; p++) {
			s_map[0xd0 + p] = t.ram + p * 256;
			s_map[0xe0 + p] = t.vid + p * 256;
		}
	}
}

static void m63_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		if (line == VY0 + VH) {   // (the update; vblank_irq)
			rbx_video_wait_idle();
			capture();
			rbx_video_frame_done();
			if (t.nmi_mask)
				z80_gen_nmi(&s_z);
		}
		t.cpu_frac += CPU_CLOCK;
		const uint32_t c = (uint32_t)(t.cpu_frac / LINE_HZ);
		t.cpu_frac -= (uint64_t)c * LINE_HZ;
		t.main_end += c;
		int irq = 0;
		z80_run(&s_z, t.main_end, &irq, 0);
		t.ay_frac += AY_CLOCK;
		const uint32_t ac = (uint32_t)(t.ay_frac / LINE_HZ);
		t.ay_frac -= (uint64_t)ac * LINE_HZ;
		q_put(Q_ADV, ac);
	}
	sound_frame();
	t.frame++;
}

static int m63_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "m63", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "m63_a", &a, sizeof(a));
	rc |= rbx_state_io(ctx, "m63_mcs", (uint8_t *)&s_mcs + MCS48_STATE_OFFSET, (uint32_t)MCS48_STATE_SIZE);
	rc |= rbx_state_io(ctx, "m63_ay0", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "m63_ay1", &s_ay[1], ay8910_state_size());
	int32_t n = s_out_n;
	rc |= rbx_state_io(ctx, "m63_n", &n, sizeof(n));
	rc |= rbx_state_io(ctx, "m63_out", s_out, sizeof(s_out));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "m63_z80", &z, sizeof(z));
	if (!rc && load) {
		s_out_n = n >= 0 && n <= OUT_MAX ? n : 0;
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
		z_fix(&z);
		s_z = z;
	}
	return rc ? -1 : 0;
}

static int m63_save(void *ctx) { return m63_state(ctx, 0); }
static int m63_load(void *ctx) { return m63_state(ctx, 1); }
static int m63_supported(const char *machine)
{
	return !strcmp(machine, "m63") || !strcmp(machine, "atomboy") || !strcmp(machine, "fghtbskt");
}

const rbx_driver rbx_drv_m63 = { "Irem M63", m63_init, m63_exit, m63_reset, m63_frame, m63_save, m63_load, m63_supported,
                                 m63_render, NULL };
