/*  Irem Vigilante, Kickle Cubele / Meikyu Jima, Duintronic's Buccaneers and Ten-Level's Bowmen. Ported from MAME
    src/mame/irem/vigilant.cpp (BSD-3-Clause, copyright-holders: Mike Balfour) and the sample half of
    src/mame/irem/m72_a.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria). A Z80, a 64x32 layer of 4 bpp tiles, 16x16
    sprites and picture pages behind them; the sound: a second Z80 with a YM2151 (two YM2203s on Buccaneers and Bowmen)
    and a sample DAC, a frame behind on the worker (rbx_aux). Captain X (the MC68705 that decrypts its code) is not done.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../video/irem_draw.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define LINES 284
#define VH 256
#define PEN_BG 512
#define NPENS 544
#define Q_MAX 2048
#define OUT_MAX 8192   // (buccanrs: two YM2203 at 256 kHz, 4544 samples a frame)

enum { K_VIGILANT, K_KIKCUBIC, K_BUCCANRS, K_BOWMEN };
enum { Q_ADV, Q_LATCH };
typedef struct { uint32_t kind, v; } q_ev_t;

static uint8_t *s_rom, *s_srom, *s_samples;
static uint32_t s_rom_size, s_srom_size, s_samples_size;
static rbx_gfx s_chars, s_spr, s_bg;
static z80 s_z, s_sz;
static const uint8_t *s_map[256], *s_smap[256];
static int s_kind, s_nym, s_port[5], s_vx0, s_vw, s_pages;
static uint32_t s_pix, s_htotal, s_cpu_clock, s_snd_clock, s_ym_clock, s_den;
static int s_rate;

static q_ev_t s_q[2][Q_MAX];
static int s_qn[2], s_qcur;
static int32_t s_out[OUT_MAX];
static int s_out_n, s_out_pos;
static volatile int s_job_frame;

static struct {
	uint8_t spr[0x100], pal[0x800], vram[0x1000], ram[0x2000];
	uint8_t hs_low, rhs_low, rear_color, rear_disable, bank, irq;
	uint16_t hs_high, rhs_high;
	uint32_t cpu_frac;
	unsigned long main_end;
	uint32_t frame;
	uint8_t state_end;
} t;

static struct {   // (the worker's)
	uint8_t ram[0x1000];
	uint8_t latch, rst, dac;
	uint16_t sample_addr;
	uint32_t z_frac, ym_frac;
	unsigned long z_end;
	int32_t hp_x, hp_y;
} a;

// ---- the drawing's snapshot ----
static struct {
	uint8_t spr[0x100], pal[0x800], vram[0x1000];
	uint8_t hs_low, rhs_low, rear_color, rear_disable;
	uint16_t hs_high, rhs_high;
} s_vs;

static void capture(void)
{
	memcpy(s_vs.spr, t.spr, sizeof(s_vs.spr));
	memcpy(s_vs.pal, t.pal, sizeof(s_vs.pal));
	memcpy(s_vs.vram, t.vram, sizeof(s_vs.vram));
	s_vs.hs_low = t.hs_low;
	s_vs.hs_high = t.hs_high;
	s_vs.rhs_low = t.rhs_low;
	s_vs.rhs_high = t.rhs_high;
	s_vs.rear_color = t.rear_color;
	s_vs.rear_disable = t.rear_disable;
}

static inline uint32_t pal_rgb(int o)   // (paletteram_w: 5 bits each)
{
	const uint32_t r = (s_vs.pal[o] << 3) & 0xff, g = (s_vs.pal[o + 0x100] << 3) & 0xff, b = (s_vs.pal[o + 0x200] << 3) & 0xff;
	return r << 16 | g << 8 | b;
}

// (the bottom visible area: 48 down; Bowmen's the whole screen)
static void draw_sprites(int y0)
{
	const int bytes = s_kind == K_KIKCUBIC ? 0x100 : 0xc0;
	for (int offs = 0; offs < bytes; offs += 8) {
		const uint8_t *sr = s_vs.spr + offs;
		int code = sr[4] | (sr[5] & 0x0f) << 8;
		const uint32_t color = sr[0] & 0x0f;
		const int sx = sr[6] | (sr[7] & 1) << 8;
		int sy = 256 + 128 - (sr[2] | (sr[3] & 1) << 8);
		const int flipx = sr[5] & 0x40, flipy = sr[5] & 0x80;
		const int h = 1 << ((sr[5] & 0x30) >> 4);
		sy -= 16 * h;
		code &= ~(h - 1);
		for (int y = 0; y < h; y++) {
			const uint32_t c = rbx_wrap((uint32_t)(flipy ? code + h - 1 - y : code + y), s_spr.count);
			if (s_spr.used[c])
				irem_blit(rbx_video.bitmap, rbx_video.pitch, 0, y0, s_vw, VH, s_spr.pix + c * 256, 16, 16, color * 16, 1, flipx, flipy,
				          sx - s_vx0, sy + 16 * y);
		}
	}
}

static inline void fg_tile(uint32_t code, uint32_t color, uint32_t tm, int sx, int sy, int y0)
{
	code = rbx_wrap(code, s_chars.count);
	irem_blit(rbx_video.bitmap, rbx_video.pitch, 0, y0, s_vw, VH, s_chars.pix + code * 64, 8, 8, 256 + color * 16, tm, 0, 0, sx - s_vx0, sy);
}

static void draw_foreground(int priority, int opaque)
{
	const int scroll = -(s_vs.hs_low + s_vs.hs_high);
	for (int offs = 0; offs < 0x1000; offs += 2) {
		const int sy = 8 * ((offs / 2) / 64);
		int sx = 8 * ((offs / 2) % 64);
		const int attr = s_vs.vram[offs + 1], color = attr & 0x0f;
		const uint32_t code = s_vs.vram[offs] | (uint32_t)(attr & 0xf0) << 4;
		if (priority) {
			if ((color & 0x0c) == 0x0c && sy >= 48) {   // (mask sprites)
				sx = (sx + scroll) & 0x1ff;
				fg_tile(code, (uint32_t)color, 0x00ff, sx, sy, 48);
			}
		} else {
			if (sy >= 48)
				sx = (sx + scroll) & 0x1ff;
			fg_tile(code, (uint32_t)color, opaque || color >= 4 ? 0 : 1, sx, sy, 0);
		}
	}
}

static void draw_foreground_bowmen(int opaque)
{
	for (int offs = 0; offs < 0x1000; offs += 2) {
		const int attr = s_vs.vram[offs + 1];
		fg_tile(s_vs.vram[offs] | (uint32_t)(attr & 0xf0) << 4, (uint32_t)attr & 0x0f, opaque ? 0 : 1, 8 * ((offs / 2) % 64),
		        8 * ((offs / 2) / 64), 0);
	}
}

static void draw_background(int scrollx, int y0)   // (copyscrollbitmap over the pages of 512x256)
{
	const int W = 512 * s_pages, bowmen = s_kind == K_BOWMEN;
	uint16_t *bm = rbx_video.bitmap;
	for (int y = y0; y < VH; y++) {
		uint16_t *d = bm + y * rbx_video.pitch;
		const uint32_t base = PEN_BG + (!bowmen && y >= 128 ? 16 : 0);
		int src = (s_vx0 - scrollx) % W;
		if (src < 0)
			src += W;
		for (int x = 0; x < s_vw;) {
			const int page = src / 512, col = (src & 511) >> 5, off = src & 31;
			const uint8_t *s = s_bg.pix + ((uint32_t)page * (256 * 16) + (uint32_t)y * 16 + (uint32_t)col) * 32 + (uint32_t)off;
			int n = 32 - off;
			if (n > s_vw - x)
				n = s_vw - x;
			for (int i = 0; i < n; i++)
				d[x + i] = (uint16_t)(base + s[i]);
			x += n;
			src += n;
			if (src >= W)
				src -= W;
		}
	}
}

static void vi_render(void)
{
	uint32_t *pal = rbx_video.palette;
	for (int i = 0; i < 256; i++) {
		pal[i] = pal_rgb(i);
		if (s_kind != K_KIKCUBIC)
			pal[256 + i] = pal_rgb(0x400 + i);
	}
	if (s_kind == K_KIKCUBIC) {
		for (int offs = 0; offs < 0x1000; offs += 2) {
			const int attr = s_vs.vram[offs + 1];
			const uint32_t code = rbx_wrap(s_vs.vram[offs] | (uint32_t)(attr & 0x0f) << 8, s_chars.count);
			irem_blit(rbx_video.bitmap, rbx_video.pitch, 0, 0, s_vw, VH, s_chars.pix + code * 64, 8, 8, (uint32_t)(attr & 0xf0), 0, 0, 0,
			          8 * ((offs / 2) % 64) - s_vx0, 8 * ((offs / 2) / 64));
		}
		draw_sprites(0);
		return;
	}
	for (int i = 0; i < 16; i++) {
		pal[PEN_BG + i] = pal_rgb(0x400 + 16 * s_vs.rear_color + i);
		pal[PEN_BG + 16 + i] = pal_rgb(0x400 + 16 * s_vs.rear_color + 32 + i);
	}
	const int rs = s_vs.rhs_low + s_vs.rhs_high;
	if (s_kind == K_BOWMEN) {
		if (s_vs.rear_disable)
			draw_foreground_bowmen(1);
		else {
			draw_background(0x179 + 16 * 8 - rs, 0);
			draw_foreground_bowmen(0);
		}
		draw_sprites(0);
		return;
	}
	if (s_vs.rear_disable) {
		draw_foreground(0, 1);
		draw_sprites(48);
		draw_foreground(1, 0);
	} else {
		draw_background(0x17a + 16 * 8 - rs, 48);
		draw_foreground(0, 0);
		draw_sprites(48);
		draw_foreground(1, 0);
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
static void set_bank(int n)
{
	t.bank = (uint8_t)n;
	for (int p = 0; p < 0x40; p++)
		s_map[0x80 + p] = s_rom + 0x10000 + (uint32_t)n * 0x4000 + (uint32_t)p * 256;
}

static uint8_t rd(void *u, uint16_t x)
{
	(void)u;
	if (x < 0x8000)
		return s_rom[x];
	if (x < 0xc000)
		return s_map[x >> 8][x & 0xff];
	if (s_kind == K_KIKCUBIC) {
		if (x < 0xc100)
			return t.spr[x - 0xc000];
		if (x >= 0xc800 && x < 0xcb00)
			return t.pal[x - 0xc800];
		if (x >= 0xd000 && x < 0xe000)
			return t.vram[x - 0xd000];
		return x >= 0xe000 ? t.ram[x - 0xe000] : 0;
	}
	if (x >= 0xc020 && x < 0xc0e0)
		return t.spr[x - 0xc020];
	if (x >= 0xc800 && x < 0xd000)
		return t.pal[x - 0xc800];
	if (x >= 0xd000 && x < 0xe000)
		return t.vram[x - 0xd000];
	if (x >= 0xe000 && x < 0xf000)
		return t.ram[x - 0xe000];
	return 0;
}

static void wr(void *u, uint16_t x, uint8_t v)
{
	(void)u;
	if (x < 0xc000)
		return;
	if (s_kind == K_KIKCUBIC) {
		if (x < 0xc100)
			t.spr[x - 0xc000] = v;
		else if (x >= 0xc800 && x < 0xcb00)
			t.pal[x - 0xc800] = v;
		else if (x >= 0xd000 && x < 0xe000)
			t.vram[x - 0xd000] = v;
		else if (x >= 0xe000)
			t.ram[x - 0xe000] = v;
		return;
	}
	if (x >= 0xc020 && x < 0xc0e0)
		t.spr[x - 0xc020] = v;
	else if (x >= 0xc800 && x < 0xd000)
		t.pal[x - 0xc800] = v;
	else if (x >= 0xd000 && x < 0xe000)
		t.vram[x - 0xd000] = v;
	else if (x >= 0xe000 && x < 0xf000)
		t.ram[x - 0xe000] = v;
}

static uint8_t pin(z80 *z, uint8_t p)
{
	(void)z;
	static const uint8_t order[4][5] = { { 0, 1, 2, 3, 4 }, { 3, 4, 0, 1, 2 }, { 0, 1, 2, 3, 4 }, { 0, 1, 2, 4, 3 } };
	return p < 5 ? (uint8_t)rbx_port_read(s_port[order[s_kind][p]]) : 0;
}

static void pout(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (s_kind == K_KIKCUBIC) {
		if (p == 4)
			set_bank(v & 7);
		else if (p == 6)
			q_put(Q_LATCH, v);
		return;
	}
	if (p == 0)
		q_put(Q_LATCH, v);
	else if (p == 4)
		set_bank(v & 7);
	else if (s_kind == K_BOWMEN) {
		if (p == 0x12)
			t.rhs_low = v;
		else if (p == 0x13)
			t.rhs_high = (uint16_t)((v & 0x1f) * 256);
		else if (p == 0x14) {
			t.rear_disable = v & 0x40;
			t.rear_color = v & 0x0f;
		}
	} else if (p == 0x80)
		t.hs_low = v;
	else if (p == 0x81)
		t.hs_high = (uint16_t)(v * 256);
	else if (p == 0x82)
		t.rhs_low = v;
	else if (p == 0x83)
		t.rhs_high = (uint16_t)((v & 7) * 256);
	else if (p == 0x84) {
		t.rear_disable = v & 0x40;
		t.rear_color = v & 0x0d;
	}
}

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
static int s_irq_line;   // (the Z80's INT, live: an acknowledge in a run drops it, or the Z80 takes RST 38)
static void rst_w(int mask, int on)   // (rst_neg_buffer_device: the RST lines' open bits on the bus)
{
	a.rst = (uint8_t)(on ? a.rst & ~mask : a.rst | mask);
	s_sz.int_data = a.rst;
	s_irq_line = a.rst != 0xff;
}

static void ym_irq(int state) { rst_w(0x10, state); }

static uint8_t srd(void *u, uint16_t x)
{
	(void)u;
	return x < 0xf000 ? (x < s_srom_size ? s_srom[x] : 0) : a.ram[x - 0xf000];
}

static void swr(void *u, uint16_t x, uint8_t v)
{
	(void)u;
	if (x >= 0xf000)
		a.ram[x - 0xf000] = v;
}

static uint8_t sin_(z80 *z, uint8_t p)
{
	(void)z;
	if (p < 2 || (s_nym > 1 && p < 4))
		return rbx_ym_read(p >> 1, p & 1);
	if (p == 0x80 || (p == 0x81 && s_nym == 1))
		return a.latch;
	if (p == 0x84)
		return s_samples[a.sample_addr & (s_samples_size - 1)];
	return 0xff;
}

static void sout(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (p < 2 || (s_nym > 1 && p < 4))
		rbx_ym_write(p >> 1, p & 1, v);
	else if (p == 0x80)
		a.sample_addr = (uint16_t)((a.sample_addr & 0xff00) | v);
	else if (p == 0x81)
		a.sample_addr = (uint16_t)((a.sample_addr & 0x00ff) | v << 8);
	else if (p == 0x82) {
		a.dac = v;
		a.sample_addr++;
	} else if (p == 0x83)
		rst_w(0x20, 0);
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

static void sound_run(uint32_t line)
{
	if (line & 1)   // (sound_nmi: every other line)
		z80_gen_nmi(&s_sz);
	a.z_frac += s_snd_clock * s_htotal;
	const uint32_t c = a.z_frac / s_pix;
	a.z_frac -= c * s_pix;
	a.z_end += c;
	s_irq_line = a.rst != 0xff;
	z80_run(&s_sz, a.z_end, &s_irq_line, 0);
	a.ym_frac += s_ym_clock * s_htotal;
	const uint32_t yc = a.ym_frac / s_pix;
	a.ym_frac -= yc * s_pix;
	int32_t mix[32];
	memset(mix, 0, sizeof(mix));
	const int n = rbx_ym_advance(0, yc, mix, 32);
	if (s_nym > 1)
		rbx_ym_advance(1, yc, mix, 32);
	const int32_t dac = ((int32_t)a.dac - 128) * (s_nym > 1 ? 90 : 128);
	for (int i = 0; i < n && s_out_pos < OUT_MAX; i++) {
		const int32_t x = mix[i] + dac;
		a.hp_y = x - a.hp_x + a.hp_y - a.hp_y / 225;
		a.hp_x = x;
		s_out[s_out_pos++] = a.hp_y;
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
		if (ev->kind == Q_ADV)
			sound_run(ev->v);
		else {
			a.latch = (uint8_t)ev->v;
			rst_w(0x20, 1);
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
static void vi_exit(void);
static void vi_reset(void);

static int vi_init(const rbx_game *g)
{
	uint32_t cs, ss, bs = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("soundcpu", &s_srom_size);
	s_samples = rbx_region("m72", &s_samples_size);
	const uint8_t *chars = rbx_region("chars", &cs), *spr = rbx_region("sprites", &ss), *bg = rbx_region("bgtiles", &bs);
	s_kind = !strcmp(g->machine, "kikcubic") ? K_KIKCUBIC : !strcmp(g->machine, "buccanrs") ? K_BUCCANRS
	       : !strcmp(g->machine, "bowmen") ? K_BOWMEN : K_VIGILANT;
	const int k = s_kind;
	if (!s_rom || s_rom_size < 0x30000 || !s_srom || !s_samples || !chars || !spr || (k != K_KIKCUBIC && !bg)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (k == K_BOWMEN)   // (init_bowmen)
		for (uint32_t i = 0; i < s_rom_size; i++) {
			const int v = s_rom[i];
			s_rom[i] = (uint8_t)(((v >> 2) & 1) << 7 | ((v >> 5) & 1) << 6 | (v & 1) << 5 | ((v >> 7) & 1) << 4 | ((v >> 6) & 1) << 3 |
			                     ((v >> 1) & 1) << 2 | ((v >> 4) & 1) << 1 | ((v >> 3) & 1));
		}
	memset(&t, 0, sizeof(t));
	s_nym = k == K_BUCCANRS || k == K_BOWMEN ? 2 : 1;
	s_pages = k == K_BOWMEN ? 16 : 4;
	s_vx0 = k == K_KIKCUBIC ? 64 : 128;
	s_vw = k == K_KIKCUBIC ? 384 : 256;
	s_htotal = k == K_KIKCUBIC ? 512 : 384;
	s_pix = k == K_KIKCUBIC ? 8000000 : k == K_BOWMEN ? 6000000 : 6144000;
	s_cpu_clock = k == K_BOWMEN ? 6000000 : k == K_BUCCANRS ? 5068800 : 3579545;
	s_snd_clock = k == K_BOWMEN ? 6000000 : k == K_BUCCANRS ? 3072000 : 3579545;
	s_ym_clock = k == K_BOWMEN ? 3000000 : k == K_BUCCANRS ? 3072000 : 3579545;
	s_den = s_pix;
	uint32_t pc[4] = { cs * 4, cs * 4 + 4, 0, 4 }, ps[4] = { ss * 4, ss * 4 + 4, 0, 4 };
	static const uint32_t cx[8] = { 0, 1, 2, 3, 64, 65, 66, 67 };
	static const uint32_t cy[16] = { 0, 8, 16, 24, 32, 40, 48, 56, 64, 72, 80, 88, 96, 104, 112, 120 };
	static const uint32_t sx1[16] = { 0, 1, 2, 3, 0x80, 0x81, 0x82, 0x83, 0x100, 0x101, 0x102, 0x103, 0x180, 0x181, 0x182, 0x183 };
	static const uint32_t sx2[16] = { 3, 2, 1, 0, 0x83, 0x82, 0x81, 0x80, 0x103, 0x102, 0x101, 0x100, 0x183, 0x182, 0x181, 0x180 };
	static const uint32_t bp1[4] = { 0, 2, 4, 6 }, bp2[4] = { 6, 4, 2, 0 }, by[1] = { 0 };
	uint32_t bx[32];
	for (int i = 0; i < 32; i++)
		bx[i] = (uint32_t)(i >> 1) * 8 + ((i & 1) ? 0u : 1u);
	const rbx_gfx_layout cl = { 8, 8, 4, pc, cx, cy, 128 };
	const rbx_gfx_layout sl = { 16, 16, 4, ps, k == K_VIGILANT || k == K_KIKCUBIC ? sx1 : sx2, cy, 0x40 * 8 };
	const rbx_gfx_layout bl = { 32, 1, 4, k == K_VIGILANT ? bp1 : bp2, bx, by, 16 * 8 };
	int bad = rbx_gfx_decode(&s_chars, &cl, chars, cs / 2) | rbx_gfx_decode(&s_spr, &sl, spr, ss / 2);
	if (k != K_KIKCUBIC)
		bad |= rbx_gfx_decode(&s_bg, &bl, bg, bs);
	static const char *const pn[5] = { "IN0", "IN1", "IN2", "DSW1", "DSW2" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(pn[i]);
	rbx_ym_config yc;
	for (int i = 0; i < s_nym; i++) {
		memset(&yc, 0, sizeof(yc));
		yc.type = s_nym > 1 ? RBX_YM2203 : RBX_YM2151;
		yc.clock = s_ym_clock;
		if (s_nym > 1) {
			yc.gain[0] = yc.gain[1] = yc.gain[2] = 90;   // (0.35)
			yc.gain[3] = 128;                            // (0.50)
		} else
			yc.gain[0] = yc.gain[1] = 36;                // (0.28: MAME's left and right, here averaged on one speaker)
		if (!i)
			yc.irq = ym_irq;
		bad |= rbx_ym_init(i, &yc);
	}
	s_rate = bad ? 0 : (int)rbx_ym_rate(0);
	if (bad || rbx_audio_setup((uint32_t)s_rate) || rbx_video_setup(s_vw, VH, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		vi_exit();
		return -1;
	}
	rbx_audio.frame_mhz = (uint32_t)((uint64_t)s_pix * 1000 / (s_htotal * LINES));
	vi_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void vi_exit(void)
{
	rbx_aux_set_job(NULL);
	rbx_gfx_free(&s_chars);
	rbx_gfx_free(&s_spr);
	if (s_bg.pix)
		rbx_gfx_free(&s_bg);
	memset(&s_bg, 0, sizeof(s_bg));
	for (int i = 0; i < s_nym; i++)
		rbx_ym_exit(i);
}

static void vi_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	memset(&a, 0, sizeof(a));
	memset(&s_vs, 0, sizeof(s_vs));
	t.rear_disable = 1;   // (video_reset)
	a.rst = 0xff;
	for (int i = 0; i < s_nym; i++)
		rbx_ym_reset(i);
	z80_init(&s_z);
	z_fix(&s_z);
	s_z.int_data = 0xff;
	memset(s_map, 0, sizeof(s_map));
	for (int p = 0; p < 0x80; p++)
		s_map[p] = s_rom + p * 256;
	set_bank(0);
	for (int p = 0; p < 0x10; p++) {
		s_map[0xd0 + p] = t.vram + p * 256;
		s_map[0xe0 + p] = t.ram + p * 256;
		if (s_kind == K_KIKCUBIC)
			s_map[0xf0 + p] = t.ram + 0x1000 + p * 256;
	}
	z80_init(&s_sz);
	sz_fix(&s_sz);
	s_sz.int_data = 0xff;
	memset(s_smap, 0, sizeof(s_smap));
	for (int p = 0; p < 0xf0 && (uint32_t)p * 256 < s_srom_size; p++)
		s_smap[p] = s_srom + p * 256;
	for (int p = 0; p < 0x10; p++)
		s_smap[0xf0 + p] = a.ram + p * 256;
}

static void vi_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		if (line == 0)
			t.irq = 0;   // (the vblank's end drops the line if not taken)
		if (line == VH) {   // (the update; screen_vblank, HOLD_LINE)
			rbx_video_wait_idle();
			capture();
			rbx_video_frame_done();
			t.irq = 1;
		}
		t.cpu_frac += s_cpu_clock / 100 * s_htotal;
		const uint32_t den = s_pix / 100, c = t.cpu_frac / den;
		t.cpu_frac -= c * den;
		t.main_end += c;
		int irq = t.irq;
		z80_run(&s_z, t.main_end, &irq, 1);
		t.irq = (uint8_t)irq;
		q_put(Q_ADV, (uint32_t)line);
	}
	sound_frame();
	t.frame++;
}

static int vi_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "vi", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "vi_a", &a, sizeof(a));
	for (int i = 0; i < s_nym; i++)
		rc |= rbx_ym_state(i, ctx, load);
	int32_t n = s_out_n;
	rc |= rbx_state_io(ctx, "vi_n", &n, sizeof(n));
	rc |= rbx_state_io(ctx, "vi_out", s_out, sizeof(s_out));
	z80 z = s_z, sz = s_sz;
	rc |= rbx_state_io(ctx, "vi_z", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "vi_sz", &sz, sizeof(sz));
	if (!rc && load) {
		s_out_n = n >= 0 && n <= OUT_MAX ? n : 0;
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
		z_fix(&z);
		sz_fix(&sz);
		s_z = z;
		s_sz = sz;
		set_bank(t.bank);
	}
	return rc ? -1 : 0;
}

static int vi_save(void *ctx) { return vi_state(ctx, 0); }
static int vi_load(void *ctx) { return vi_state(ctx, 1); }
static int vi_supported(const char *machine)
{
	return !strcmp(machine, "vigilant") || !strcmp(machine, "kikcubic") || !strcmp(machine, "buccanrs") || !strcmp(machine, "bowmen");
}

const rbx_driver rbx_drv_vigilant = { "Irem Vigilante", vi_init, vi_exit, vi_reset, vi_frame, vi_save, vi_load, vi_supported,
                                      vi_render, NULL };
