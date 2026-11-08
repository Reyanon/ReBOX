/*  Data East Break Thru / Darwin 4078, from MAME src/mame/dataeast/brkthru.cpp (BSD-3-Clause, copyright-holders:
    Phil Stroffolino).
    A 6809 at 1.5 MHz (NMI at vblank when enabled, IRQ on a coin), a 512x256 background of 16x16 tiles in 3 bpp (x
    scroll; its non-zero pens over the sprites of priority bit clear), byte-coded sprites (16x16 / 16x32, wrapping),
    an 8x8 text layer, the PROM palette. The sound: a 6809 at 1.5 MHz with YM2203 and YM3526 (on its IRQ), the latch
    on its NMI.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 1500000        // 12 MHz / 8
#define SND_CLOCK 1500000        // MC6809 at 12 MHz / 2, internal / 4
#define YM1_CLOCK 1500000        // YM2203
#define YM2_CLOCK 3000000        // YM3526
#define FRAME_MHZ 57445          // set_raw(12 MHz / 2, 384, 0, 256, 272, 8, 248)
#define LINES 272
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 256

static int s_darwin;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t s_ram[0x1800];              // 0x0000 .. 0x17ff (Break Thru: text, RAM, background, sprites, RAM)
static int s_fg, s_bg, s_spr;              // (their s_ram offsets)
static int s_bank, s_bgscroll, s_bgbasecolor, s_flip, s_int_enable, s_coin_prev;
static m6809_t s_cpu;
static int s_irq;
static rbx_gfx s_gfx8, s_gfxt[8], s_gfxspr;
static uint32_t s_palette[256];
static uint8_t s_pri_buf[VIS_W * VIS_H];
static int s_port_p[2], s_port_dsw1, s_port_dsw2, s_port_flip;
static int s_line;
static uint64_t s_cpu_frac, s_snd_frac, s_ym1_frac, s_ym2_frac;
static struct {
	uint8_t fg[0x400], bg[0x400], spr[0x100];
	int bgscroll, bgbasecolor, flip;
} *s_snap;

// ---- sound: 6809, YM2203, YM3526 ----
static m6809_t s_snd;
static uint8_t s_sram[0x2000];
static uint8_t *s_zrom;
static uint32_t s_zrom_size;
static uint8_t s_latch;
static int32_t *s_mix, *s_ym2buf;
static int s_mix_n, s_mix_cap, s_ym2_n, s_ym2_cap;

static void ym2_irq(int state) { m6809_set_irq(&s_snd, state); }
static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a == 0x4000) {   // (reading the latch clears its NMI)
		m6809_set_nmi(&s_snd, 0);
		return s_latch;
	}
	if (a == 0x6000 || a == 0x6001) return rbx_ym_read(0, a & 1);
	return 0;
}
static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a == 0x2000 || a == 0x2001) rbx_ym_write(1, a & 1, v);
	else if (a == 0x6000 || a == 0x6001) rbx_ym_write(0, a & 1, v);
}
static void sound_line(void)
{
	s_snd_frac += (uint64_t)SND_CLOCK * 1000;
	const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
	const int64_t cyc = (int64_t)(s_snd_frac / den);
	s_snd_frac -= (uint64_t)cyc * den;
	m6809_run_until(&s_snd, s_snd.cycles + cyc);
	s_mix_n += rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&s_ym1_frac, YM1_CLOCK, FRAME_MHZ, LINES), s_mix + s_mix_n,
	                          s_mix_cap - s_mix_n);
	s_ym2_n += rbx_ym_advance(1, (uint32_t)rbx_line_cycles(&s_ym2_frac, YM2_CLOCK, FRAME_MHZ, LINES), s_ym2buf + s_ym2_n,
	                          s_ym2_cap - s_ym2_n);
}
static void sound_frame_end(void)
{
	rbx_ym_async_take(0, s_mix, s_mix_n);
	rbx_ym_async_take(1, s_ym2buf, s_ym2_n);
	if (s_ym2_n > 0)
		for (int i = 0; i < s_mix_n; i++)
			s_mix[i] += s_ym2buf[(int)((int64_t)i * s_ym2_n / s_mix_n)];
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	memset(s_ym2buf, 0, sizeof(int32_t) * (size_t)s_ym2_n);
	s_mix_n = s_ym2_n = 0;
}

// ---- bus ----
static int vblank(void) { return s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H; }
static void set_bank(int b)   // (the 8 KB banks from 0x10000 of the CPU's region)
{
	s_bank = b & 7;
	for (int p = 0; p < 0x20; p++)
		s_cpu.rpage[0x20 + p] = s_rom + 0x10000 + (uint32_t)s_bank * 0x2000 + (uint32_t)p * 256;
}
static void set_irq(int state)
{
	s_irq = state;
	m6809_set_irq(&s_cpu, state);
}

static uint8_t cpu_r(void *ctx, uint16_t a)
{
	(void)ctx;
	switch (a & 0x0fff) {   // (Break Thru 0x1800 .., Darwin 0x0800 ..)
	case 0x800: return (uint8_t)rbx_port_read(s_port_p[0]);
	case 0x801:
		rbx_port_custom(s_port_p[1], 0x80, vblank() ? 0x80 : 0);
		return (uint8_t)rbx_port_read(s_port_p[1]);
	case 0x802: return (uint8_t)rbx_port_read(s_port_dsw1);
	case 0x803: return (uint8_t)rbx_port_read(s_port_dsw2);
	}
	return 0;
}
static void cpu_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	switch (a & 0x0fff) {
	case 0x800: s_bgscroll = (s_bgscroll & 0x100) | v; break;   // bgscroll_w
	case 0x801:   // control_w: the bank, the background colour, the flip, the scroll MSB
		set_bank(v & 7);
		s_bgbasecolor = (v & 0x38) >> 2;
		s_flip = ((v >> 6) & 1) ^ (rbx_port_read(s_port_flip) & 1);
		s_bgscroll = (s_bgscroll & 0xff) | ((v & 0x80) << 1);
		break;
	case 0x802:   // the sound latch (its NMI)
		s_latch = v;
		m6809_set_nmi(&s_snd, 1);
		break;
	case 0x803:   // int_enable_w: bit 0 IRQ disable (and clear), bit 1 NMI enable
		s_int_enable = v;
		if (v & 1)
			set_irq(0);
		break;
	}
}
static void coin_update(void)   // (a coin's press raises the IRQ unless disabled)
{
	const int c = (int)(rbx_port_read(s_port_dsw2) & 0xe0);
	if ((s_coin_prev & ~c & 0xe0) && !(s_int_enable & 1))
		set_irq(1);
	s_coin_prev = c;
}

// ---- video ----
// a 16x16 sprite element with MAME prio_transpen (drawn where 1 << pri is not in pmask, bit 31 implied; pri = 31)
static void sprite(uint32_t code, int colour, int flip, int sx, int sy, uint32_t pmask, uint16_t *bm, int pitch)
{
	if (!s_gfxspr.count)
		return;
	code %= s_gfxspr.count;
	pmask |= 1u << 31;
	const uint8_t *src = s_gfxspr.pix + code * 256;
	for (int y = 0; y < 16; y++) {
		const int dy = sy + y - VIS_Y0;
		if (dy < 0 || dy >= VIS_H)
			continue;
		const uint8_t *row = src + (flip ? 15 - y : y) * 16;
		for (int x = 0; x < 16; x++) {
			const int dx = sx + x;
			if (dx < 0 || dx >= VIS_W)
				continue;
			const uint8_t p = row[flip ? 15 - x : x];
			if (!p)
				continue;
			uint8_t *pr = &s_pri_buf[dy * VIS_W + dx];
			if (!((1u << (*pr & 0x1f)) & pmask))
				bm[dy * pitch + dx] = (uint16_t)(0x40 + colour * 8 + p);
			*pr = 31;
		}
	}
}

static void bk_render(void)
{
	memcpy(rbx_video.palette, s_palette, sizeof(s_palette));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, flip = s_snap->flip;
	// the background: 32 columns x 16 rows of 16x16 (by columns), opaque at priority 1, its non-zero pens 2
	const uint8_t *bg = s_snap->bg;
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = bm + y * pitch;
		uint8_t *pr = s_pri_buf + y * VIS_W;
		for (int x = 0; x < VIS_W; x++) {
			int px = (x + s_snap->bgscroll) & 0x1ff, py = (y + VIS_Y0) & 0xff;
			if (flip) {
				px = 511 - px;
				py = 255 - py;
			}
			const int idx = (px >> 4) * 16 + (py >> 4);
			const int code = (bg[idx * 2] | (bg[idx * 2 + 1] << 8)) & 0x3ff;
			const rbx_gfx *g = &s_gfxt[code >> 7];
			const int colour = s_snap->bgbasecolor + ((bg[idx * 2 + 1] >> 2) & 1);
			const uint32_t c = rbx_wrap((uint32_t)(code & 0x7f), g->count);
			const uint8_t p = g->pix[(c * 16 + (uint32_t)(py & 15)) * 16 + (uint32_t)(px & 15)];
			d[x] = (uint16_t)(0x80 + colour * 8 + p);
			pr[x] = (uint8_t)(p ? 2 : 1);
		}
	}
	const uint8_t *sr = s_snap->spr;
	for (int offs = 0x100 - 4; offs >= 0; offs -= 4) {   // draw_sprites (from the end: the order sets the priorities)
		if (!(sr[offs] & 1))
			continue;
		const uint32_t pmask = (sr[offs] & 0x08) ? 0 : 0xcc;   // (GFX_PMASK_2: behind the background's pens)
		int sx = 240 - sr[offs + 3];
		if (sx < -7)
			sx += 256;
		int sy = 240 - sr[offs + 2];
		const uint32_t code = (uint32_t)(sr[offs + 1] + 128 * (sr[offs] & 0x06));
		const int colour = (sr[offs] & 0xe0) >> 5;
		if (flip) {
			sx = 240 - sx;
			sy = 240 - sy;
		}
		if (sr[offs] & 0x10) {   // double height (and again 256 lower: the wrap)
			const int y0 = flip ? sy + 16 : sy - 16;
			sprite(code & ~1u, colour, flip, sx, y0, pmask, bm, pitch);
			sprite(code | 1, colour, flip, sx, sy, pmask, bm, pitch);
			sprite(code & ~1u, colour, flip, sx, y0 + 256, pmask, bm, pitch);
			sprite(code | 1, colour, flip, sx, sy + 256, pmask, bm, pitch);
		} else {
			sprite(code, colour, flip, sx, sy, pmask, bm, pitch);
			sprite(code, colour, flip, sx, sy + 256, pmask, bm, pitch);
		}
	}
	const uint8_t *fg = s_snap->fg;
	for (int y = 0; y < VIS_H; y++) {   // the text layer: 32x32 of 8x8 (a byte a tile), 3 bpp, pen 0 clear
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < VIS_W; x++) {
			int px = x, py = y + VIS_Y0;
			if (flip) {
				px = 255 - px;
				py = 255 - py;
			}
			const uint32_t code = rbx_wrap((uint32_t)fg[(py >> 3) * 32 + (px >> 3)], s_gfx8.count);
			const uint8_t p = s_gfx8.pix[(code * 8 + (uint32_t)(py & 7)) * 8 + (uint32_t)(px & 7)];
			if (p)
				d[x] = p;
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->fg, s_ram + s_fg, sizeof(s_snap->fg));
	memcpy(s_snap->bg, s_ram + s_bg, sizeof(s_snap->bg));
	memcpy(s_snap->spr, s_ram + s_spr, sizeof(s_snap->spr));
	s_snap->bgscroll = s_bgscroll;
	s_snap->bgbasecolor = s_bgbasecolor;
	s_snap->flip = s_flip;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void bk_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	s_bgscroll = s_bgbasecolor = s_flip = s_int_enable = 0;
	s_coin_prev = 0xe0;
	set_bank(0);
	m6809_reset(&s_cpu);
	set_irq(0);
	m6809_set_nmi(&s_cpu, 0);
	memset(s_sram, 0, sizeof(s_sram));
	s_latch = 0;
	m6809_reset(&s_snd);
	m6809_set_irq(&s_snd, 0);
	m6809_set_nmi(&s_snd, 0);
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	s_mix_n = s_ym2_n = 0;
	s_line = 0;
	s_cpu_frac = s_snd_frac = s_ym1_frac = s_ym2_frac = 0;
}

static void bk_exit(void);

static int bk_init(const rbx_game *g)
{
	s_darwin = !strcmp(g->machine, "darwin");
	uint32_t csize, tsize, ssize, psize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *chars = rbx_region("chars", &csize), *tiles = rbx_region("tiles", &tsize), *spr = rbx_region("sprites", &ssize),
	        *prom = rbx_region("proms", &psize);
	if (!s_rom || s_rom_size < 0x20000 || !s_zrom || s_zrom_size < 0x10000 || !chars || csize < 0x2000 || !tiles ||
	    tsize < 0x20000 || !spr || ssize < 0x18000 || !prom || psize < 0x200) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{	// charlayout (256, 3 bpp: planes 512*8*8+4, 0, 4; the right half 256*8*8 on), tilelayout1 / 2 (128 each, 3 bpp:
		// 0x4000*8+4 / 0x3000*8, 0, 4) at 0x0000, 0x1000, 0x8000 ... 0x19000, spritelayout (1024, 3 bpp)
		static uint32_t xc[8], y8[8], xt[16], y16[16], xs[16];
		for (int i = 0; i < 4; i++) {
			xc[i] = (uint32_t)(256 * 8 * 8 + i);
			xc[4 + i] = (uint32_t)i;
			xt[i] = (uint32_t)i;
			xt[4 + i] = (uint32_t)(1024 * 8 * 8 + i);
			xt[8 + i] = (uint32_t)(16 * 8 + i);
			xt[12 + i] = (uint32_t)(16 * 8 + 1024 * 8 * 8 + i);
		}
		for (int i = 0; i < 8; i++) {
			y8[i] = (uint32_t)(i * 8);
			xs[i] = (uint32_t)(16 * 8 + i);
			xs[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 8);
		static const uint32_t pc[3] = { 512 * 8 * 8 + 4, 0, 4 };
		static const uint32_t p1[3] = { 0x4000 * 8 + 4, 0, 4 }, p2[3] = { 0x3000 * 8, 0, 4 };
		static const uint32_t ps[3] = { 2 * 1024 * 32 * 8, 1024 * 32 * 8, 0 };
		const rbx_gfx_layout l8 = { 8, 8, 3, pc, xc, y8, 8 * 8 };
		const rbx_gfx_layout l1 = { 16, 16, 3, p1, xt, y16, 32 * 8 }, l2 = { 16, 16, 3, p2, xt, y16, 32 * 8 };
		const rbx_gfx_layout ls = { 16, 16, 3, ps, xs, y16, 32 * 8 };
		static const uint32_t to[8] = { 0x00000, 0x01000, 0x08000, 0x09000, 0x10000, 0x11000, 0x18000, 0x19000 };
		int rc = rbx_gfx_decode(&s_gfx8, &l8, chars, 256 * 8) || rbx_gfx_decode(&s_gfxspr, &ls, spr, 1024 * 32);
		for (int k = 0; k < 8; k++)
			rc = rc || rbx_gfx_decode(&s_gfxt[k], (k & 1) ? &l2 : &l1, tiles + to[k], 128 * 32);
		if (rc) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			bk_exit();
			return -1;
		}
	}
	for (int i = 0; i < 256; i++) {   // palette: the RM-C3 resistor weights (red, green in one PROM, blue in the other)
		static const uint8_t w[4] = { 0x0e, 0x1f, 0x43, 0x8f };
		int r = 0, gg = 0, b = 0;
		for (int k = 0; k < 4; k++) {
			r += ((prom[i] >> k) & 1) * w[k];
			gg += ((prom[i] >> (4 + k)) & 1) * w[k];
			b += ((prom[256 + i] >> k) & 1) * w[k];
		}
		s_palette[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	s_snap = calloc(1, sizeof(*s_snap));
	// sound: YM2203 1.5 MHz (SSG 0.10, FM 0.50), YM3526 3 MHz 1.0
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM1_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = 26;
	yc.gain[3] = 128;
	int rc = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM3526;
	yc.clock = YM2_CLOCK;
	yc.gain[0] = 256;
	yc.irq = ym2_irq;
	rc = rc || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	if (!rc) {
		s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
		s_ym2_cap = (int)(rbx_ym_rate(1) / 50) + 256;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		s_ym2buf = (int32_t *)calloc((size_t)s_ym2_cap, sizeof(int32_t));
		rc = !s_mix || !s_ym2buf || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 256);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		bk_exit();
		return -1;
	}
	m6809_init(&s_snd);
	for (int p = 0; p < 0x20; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = s_sram + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		s_snd.rpage[p] = s_zrom + p * 256;
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	m6809_init(&s_cpu);
	// brkthru_main_map: text 0x0000, RAM, background 0x0c00, sprites 0x1000, RAM (s_ram as it is), I/O 0x1800;
	// darwin_main_map: sprites 0x0000 (s_ram 0x0000), I/O 0x0800, text 0x1000, RAM, background 0x1c00 (s_ram 0x0800 ..)
	s_fg = s_darwin ? 0x0800 : 0x0000;
	s_bg = s_darwin ? 0x1400 : 0x0c00;
	s_spr = s_darwin ? 0x0000 : 0x1000;
	if (s_darwin) {
		s_cpu.rpage[0] = s_cpu.wpage[0] = s_ram;
		for (int p = 0x10; p < 0x20; p++)
			s_cpu.rpage[p] = s_cpu.wpage[p] = s_ram + (p - 0x08) * 256;
	} else
		for (int p = 0; p < 0x18; p++)
			s_cpu.rpage[p] = s_cpu.wpage[p] = s_ram + p * 256;
	for (int p = 0x40; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + p * 256;
	s_cpu.read = cpu_r;
	s_cpu.write = cpu_w;
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_dsw1 = rbx_port_find("DSW1");
	s_port_dsw2 = rbx_port_find("DSW2_COIN");
	s_port_flip = rbx_port_find("FLIP");
	bk_reset();
	return 0;
}

static void bk_exit(void)
{
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix);
	free(s_ym2buf);
	s_mix = s_ym2buf = NULL;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfxspr);
	for (int k = 0; k < 8; k++)
		rbx_gfx_free(&s_gfxt[k]);
	free(s_snap);
	s_snap = NULL;
}

static void bk_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	coin_update();
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // vblank: an NMI pulse when enabled
			vblank_capture();
			if (s_int_enable & 2) {
				m6809_set_nmi(&s_cpu, 1);
				m6809_set_nmi(&s_cpu, 0);
			}
		}
		s_cpu_frac += (uint64_t)CPU_CLOCK * 1000;
		const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
		const int64_t cyc = (int64_t)(s_cpu_frac / den);
		s_cpu_frac -= (uint64_t)cyc * den;
		m6809_run_until(&s_cpu, s_cpu.cycles + cyc);
		sound_line();
	}
	sound_frame_end();
}

static int bk_state(void *ctx, int load)
{
	uint8_t cpu[M6809_STATE_SIZE], snd[M6809_STATE_SIZE];
	if (!load) {
		memcpy(cpu, &s_cpu, M6809_STATE_SIZE);
		memcpy(snd, &s_snd, M6809_STATE_SIZE);
	}
	uint64_t fr[4] = { s_cpu_frac, s_snd_frac, s_ym1_frac, s_ym2_frac };
	int32_t misc[8] = { s_bank, s_bgscroll, s_bgbasecolor, s_flip, s_int_enable, s_coin_prev, s_irq, s_latch };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m6809", cpu, M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "m6809s", snd, M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		memcpy(&s_cpu, cpu, M6809_STATE_SIZE);
		memcpy(&s_snd, snd, M6809_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_ym1_frac = fr[2];
		s_ym2_frac = fr[3];
		set_bank(misc[0]);
		s_bgscroll = misc[1];
		s_bgbasecolor = misc[2];
		s_flip = misc[3];
		s_int_enable = misc[4];
		s_coin_prev = misc[5];
		s_irq = misc[6];
		s_latch = (uint8_t)misc[7];
	}
	return rc ? -1 : 0;
}

static int bk_save(void *ctx) { return bk_state(ctx, 0); }
static int bk_load(void *ctx) { return bk_state(ctx, 1); }
static int bk_supported(const char *machine) { return !strcmp(machine, "brkthru") || !strcmp(machine, "darwin"); }

const rbx_driver rbx_drv_brkthru = { "DECO Break Thru", bk_init, bk_exit, bk_reset, bk_frame, bk_save, bk_load,
                                     bk_supported, bk_render, NULL };
