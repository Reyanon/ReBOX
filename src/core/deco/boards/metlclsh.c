/*  Data East Metal Clash, from MAME src/mame/dataeast/metlclsh.cpp (BSD-3-Clause, copyright-holders: Luca Elia).
    Two 6809s (MAME: M6809 at 1.5 MHz, internally / 4) sharing 8 KB: the main CPU's IRQ from the YM3526 and its NMI
    from the sub CPU, the sub CPU's IRQ from the main CPU and its NMI on a coin. A 512x256 background of 16x16 tiles
    (always flipped in x, x scrolled, banked graphics, behind a RAM bank), 16x16 / 16x32 sprites, an 8x8 text layer
    with a priority bit (under the background or over the sprites), the xBGR 444 palette RAM (48 entries); YM2203 and
    YM3526 on the main CPU.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 375000         // (MAME: M6809 1.5 MHz, divided by 4)
#define YM1_CLOCK 1500000        // YM2203
#define YM2_CLOCK 3000000        // YM3526
#define FRAME_MHZ 57445          // set_raw(12 MHz / 2, 384, 0, 256, 272, 8, 248)
#define LINES 272
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 256

static uint8_t *s_rom, *s_sub_rom;
static uint32_t s_rom_size, s_sub_size;
static uint8_t s_shared[0x2000];           // 0x8000 .. 0x9fff of both CPUs
static uint8_t s_pal[0x30], s_pal_ext[0x30], s_fgram[0x800], s_spr[0x200];   // (the main CPU's)
static uint8_t s_bgram[0x800], s_otherram[0x800];   // (the sub CPU's 0xd000 .. 0xd7ff, by the RAM bank)
static uint8_t s_scrollx[2];
static int s_rambank, s_write_mask, s_gfxbank, s_flip, s_coin_prev;
static int s_main_nmi, s_main_irq_ym, s_sub_irq, s_sub_nmi;
static m6809_t s_cpu[2];
static rbx_gfx s_gfx8, s_gfx16, s_gfxspr;
static int s_port_in[3], s_port_dsw;
static int s_line;
static uint64_t s_cpu_frac, s_ym1_frac, s_ym2_frac;
static int32_t *s_mix, *s_ym2buf;
static int s_mix_n, s_mix_cap, s_ym2_n, s_ym2_cap;
static struct {
	uint8_t pal[0x30], pal_ext[0x30], fgram[0x800], spr[0x200], bgram[0x800], scrollx[2];
	int gfxbank, flip;
} *s_snap;

// ---- bus ----
static int vblank(void) { return s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H; }
static void ym2_irq(int state) { m6809_set_irq(&s_cpu[0], state); s_main_irq_ym = state; }
static uint8_t in_r(uint16_t a)   // (both CPUs: 0xc000 .. 0xc003)
{
	switch (a) {
	case 0xc000: return (uint8_t)rbx_port_read(s_port_in[0]);
	case 0xc001: return (uint8_t)rbx_port_read(s_port_in[1]);
	case 0xc002: return (uint8_t)rbx_port_read(s_port_in[2]);
	case 0xc003:
		rbx_port_custom(s_port_dsw, 0x80, vblank() ? 0x80 : 0);
		return (uint8_t)rbx_port_read(s_port_dsw);
	}
	return 0;
}
static uint8_t main_r(void *ctx, uint16_t a)   // master_map
{
	(void)ctx;
	if (a >= 0xc800 && a < 0xc830) return s_pal[a - 0xc800];
	if (a >= 0xcc00 && a < 0xcc30) return s_pal_ext[a - 0xcc00];
	if (a == 0xd000 || a == 0xd001) return rbx_ym_read(0, a & 1);
	return in_r(a);
}
static void main_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a >= 0xc800 && a < 0xc830) s_pal[a - 0xc800] = v;
	else if (a >= 0xcc00 && a < 0xcc30) s_pal_ext[a - 0xcc00] = v;
	else if (a == 0xc0c2) { s_sub_irq = 1; m6809_set_irq(&s_cpu[1], 1); }   // cause_irq
	else if (a == 0xc0c3) { s_main_nmi = 0; m6809_set_nmi(&s_cpu[0], 0); }   // ack_nmi
	else if (a == 0xd000 || a == 0xd001) rbx_ym_write(0, a & 1, v);
	else if (a == 0xe000 || a == 0xe001) rbx_ym_write(1, a & 1, v);
}
static void set_rambank(int v)   // rambank_w: the tile map (1) or the bits of another RAM (the mask)
{
	s_rambank = v;
	for (int p = 0; p < 8; p++)
		s_cpu[1].rpage[0xd0 + p] = ((v & 1) ? s_bgram : s_otherram) + p * 256;
	s_write_mask = (v & 1) ? 0 : 1 << (v >> 1);
}
static uint8_t sub_r(void *ctx, uint16_t a)   // slave_map
{
	(void)ctx;
	return in_r(a);
}
static void sub_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a >= 0xd000 && a < 0xd800) {   // bgram_w
		if (s_write_mask)
			s_otherram[a - 0xd000] = (uint8_t)((s_otherram[a - 0xd000] & ~s_write_mask) | (v & s_write_mask));
		else
			s_bgram[a - 0xd000] = v;
		return;
	}
	switch (a) {
	case 0xc000:   // gfxbank_w
		if (!(v & 4))
			s_gfxbank = v & 3;
		break;
	case 0xc0c0: s_main_nmi = 1; m6809_set_nmi(&s_cpu[0], 1); break;   // cause_nmi2
	case 0xc0c1: s_sub_irq = 0; m6809_set_irq(&s_cpu[1], 0); break;    // ack_irq2
	case 0xe301: s_flip = v & 1; break;
	case 0xe401: set_rambank(v); break;
	case 0xe402: case 0xe403: s_scrollx[a - 0xe402] = v; break;
	case 0xe417: s_sub_nmi = 0; m6809_set_nmi(&s_cpu[1], 0); break;    // ack_nmi2
	}
}
static void coin_update(void)   // (a coin or the service: the sub CPU's NMI)
{
	const int c = (int)(rbx_port_read(s_port_in[2]) & 0xc0) | (int)(rbx_port_read(s_port_dsw) & 0x40) >> 1;
	if (s_coin_prev & ~c & 0xe0) {
		s_sub_nmi = 1;
		m6809_set_nmi(&s_cpu[1], 1);
	}
	s_coin_prev = c;
}

// ---- video ----
static void spr_blit(uint32_t code, int colour, int fx, int fy, int sx, int sy, uint16_t *bm, int pitch)
{
	if (!s_gfxspr.count)
		return;
	code %= s_gfxspr.count;
	const uint8_t *src = s_gfxspr.pix + code * 256;
	for (int y = 0; y < 16; y++) {
		const int dy = sy + y - VIS_Y0;
		if (dy < 0 || dy >= VIS_H)
			continue;
		const uint8_t *row = src + (fy ? 15 - y : y) * 16;
		for (int x = 0; x < 16; x++) {
			const int dx = sx + x;
			if (dx < 0 || dx >= VIS_W)
				continue;
			const uint8_t p = row[fx ? 15 - x : x];
			if (p)
				bm[dy * pitch + dx] = (uint16_t)(colour * 8 + p);
		}
	}
}
// the text layer's tiles of one category (1: under the background, 2: over the sprites)
static void fg_draw(uint16_t *bm, int pitch, int cat)
{
	const int flip = s_snap->flip;
	const uint8_t *fg = s_snap->fgram;
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < VIS_W; x++) {
			int px = x, py = y + VIS_Y0;
			if (flip) {
				px = 255 - px;
				py = 255 - py;
			}
			const int idx = (py >> 3) * 32 + (px >> 3), attr = fg[0x400 + idx];
			if (((attr & 0x80) ? 1 : 2) != cat)
				continue;
			const uint32_t code = s_gfx8.count ? (uint32_t)(fg[idx] + ((attr & 3) << 8)) % s_gfx8.count : 0;
			const uint8_t p = s_gfx8.pix[(code * 8 + (uint32_t)(py & 7)) * 8 + (uint32_t)(px & 7)];
			if (p)
				d[x] = (uint16_t)(0x20 + ((attr >> 5) & 3) * 4 + p);
		}
	}
}

static void mc_render(void)
{
	for (int i = 0; i < 0x30; i++) {   // xBGR_444
		const int raw = s_snap->pal[i] | (s_snap->pal_ext[i] << 8);
		rbx_video.palette[i] = (uint32_t)(((raw & 15) * 17) << 16 | (((raw >> 4) & 15) * 17) << 8 | ((raw >> 8) & 15) * 17);
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, flip = s_snap->flip;
	for (int y = 0; y < VIS_H; y++)
		for (int x = 0; x < VIS_W; x++)
			bm[y * pitch + x] = 0x10;
	fg_draw(bm, pitch, 1);
	if (s_snap->scrollx[0] & 0x08) {   // the background (enabled): 32x16 of 16x16, always flipped in x (MAME's
		// flipped tilemap: the scroll taken from the screen's total width 384), pen 0 clear
		const int scroll = s_snap->scrollx[1] + ((s_snap->scrollx[0] & 0x02) << 7);
		for (int y = 0; y < VIS_H; y++) {
			uint16_t *d = bm + y * pitch;
			for (int x = 0; x < VIS_W; x++) {
				int px = (scroll + 383 - x) & 0x1ff, py = (y + VIS_Y0) & 0xff;
				if (flip) {   // (flip screen: the tilemap's y flip only; MAME takes the screen's 272 lines)
					px = (x + scroll) & 0x1ff;
					py = (271 - (y + VIS_Y0)) & 0xff;
				}
				const int col = px >> 4, row = py >> 4;
				const int idx = (row & 7) + ((row & ~7) << 4) + ((col & 0xf) << 3) + ((col & ~0xf) << 4);
				const uint32_t code = s_gfx16.count ? (uint32_t)(s_snap->bgram[idx & 0x1ff] + (s_snap->gfxbank << 7)) % s_gfx16.count : 0;
				const uint8_t p = s_gfx16.pix[(code * 16 + (uint32_t)(py & 15)) * 16 + (uint32_t)(px & 15)];
				if (p)
					d[x] = (uint16_t)(0x10 + p);
			}
		}
	}
	const uint8_t *sr = s_snap->spr;
	for (int offs = 0; offs < 0x200; offs += 4) {   // draw_sprites (in the list's order)
		const int attr = sr[offs];
		if (!(attr & 1))
			continue;
		int fy = (attr & 2) != 0, fx = (attr & 4) != 0;
		const int colour = (attr & 8) >> 3, sizey = attr & 0x10;
		const uint32_t code = (uint32_t)(((attr & 0x60) << 3) + sr[offs + 1]);
		int sx = 240 - sr[offs + 3];
		if (sx < -7)
			sx += 256;
		int sy = 240 - sr[offs + 2];
		if (flip) {
			sx = 240 - sx;
			fx = !fx;
			sy = 240 - sy;
			fy = !fy;
			if (sizey)
				sy += 16;
			if (sy > 240)
				sy -= 256;
		}
		for (int wrapy = 0; wrapy <= 256; wrapy += 256) {   // (again 256 lower: the wrap)
			if (sizey) {
				spr_blit(code & ~1u, colour, fx, fy, sx, sy + (fy ? 0 : -16) + wrapy, bm, pitch);
				spr_blit(code | 1, colour, fx, fy, sx, sy + (fy ? -16 : 0) + wrapy, bm, pitch);
			} else
				spr_blit(code, colour, fx, fy, sx, sy + wrapy, bm, pitch);
		}
	}
	fg_draw(bm, pitch, 2);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	memcpy(s_snap->pal_ext, s_pal_ext, sizeof(s_pal_ext));
	memcpy(s_snap->fgram, s_fgram, sizeof(s_fgram));
	memcpy(s_snap->spr, s_spr, sizeof(s_spr));
	memcpy(s_snap->bgram, s_bgram, sizeof(s_bgram));
	memcpy(s_snap->scrollx, s_scrollx, sizeof(s_scrollx));
	s_snap->gfxbank = s_gfxbank;
	s_snap->flip = s_flip;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void mc_reset(void)
{
	memset(s_shared, 0, sizeof(s_shared));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_pal_ext, 0, sizeof(s_pal_ext));
	memset(s_fgram, 0, sizeof(s_fgram));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_bgram, 0, sizeof(s_bgram));
	memset(s_otherram, 0, sizeof(s_otherram));
	memset(s_scrollx, 0, sizeof(s_scrollx));
	s_gfxbank = s_flip = 0;
	s_coin_prev = 0xe0;
	set_rambank(0);
	s_write_mask = 0;   // (machine_reset)
	s_main_nmi = s_main_irq_ym = s_sub_irq = s_sub_nmi = 0;
	for (int c = 0; c < 2; c++) {
		m6809_reset(&s_cpu[c]);
		m6809_set_irq(&s_cpu[c], 0);
		m6809_set_nmi(&s_cpu[c], 0);
	}
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	s_mix_n = s_ym2_n = 0;
	s_line = 0;
	s_cpu_frac = s_ym1_frac = s_ym2_frac = 0;
}

static void mc_exit(void);

static int mc_init(const rbx_game *g)
{
	uint32_t ssize, tsize, fsize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_sub_rom = rbx_region("sub", &s_sub_size);
	uint8_t *spr = rbx_region("sprites", &ssize), *tiles = rbx_region("bgtiles", &tsize), *chars = rbx_region("fgtiles", &fsize);
	if (!s_rom || s_rom_size < 0x10000 || !s_sub_rom || s_sub_size < 0x10000 || !spr || ssize < 0x18000 || !tiles ||
	    tsize < 0x10000 || !chars || fsize < 0x4000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{	// spritelayout (1024, the thirds), tilelayout16 (512, 3 bpp: the quarters 2, 1, 0; x bit-reversed by bytes),
		// tilelayout8 (1024, 2 bpp in the nibbles, the left half the region's second half)
		static uint32_t xs[16], ys[16], xt[16], x8[8], y8[8];
		for (int i = 0; i < 8; i++) {
			xs[i] = (uint32_t)(8 * 8 * 2 + i);
			xs[8 + i] = (uint32_t)i;
			ys[i] = (uint32_t)(i * 8);
			ys[8 + i] = (uint32_t)(8 * 8 + i * 8);
			xt[i] = (uint32_t)(7 - i);
			xt[8 + i] = (uint32_t)(8 * 8 * 2 + 7 - i);
			y8[i] = (uint32_t)(i * 8);
		}
		for (int i = 0; i < 4; i++) {
			x8[i] = (uint32_t)(0x2000 * 8 + i);
			x8[4 + i] = (uint32_t)i;
		}
		static const uint32_t ps[3] = { 0x10000 * 8, 0x8000 * 8, 0 }, pt[3] = { 0x8000 * 8, 0x4000 * 8, 0 },
		                      pc[2] = { 0, 4 };
		const rbx_gfx_layout ls = { 16, 16, 3, ps, xs, ys, 16 * 16 };
		const rbx_gfx_layout lt = { 16, 16, 3, pt, xt, ys, 16 * 16 };
		const rbx_gfx_layout l8 = { 8, 8, 2, pc, x8, y8, 8 * 8 };
		if (rbx_gfx_decode(&s_gfxspr, &ls, spr, 1024 * 32) || rbx_gfx_decode(&s_gfx16, &lt, tiles, 512 * 32) ||
		    rbx_gfx_decode(&s_gfx8, &l8, chars, 1024 * 8)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			mc_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	// sound: YM2203 1.5 MHz (SSG 0.10, FM 0.50), YM3526 3 MHz 0.50
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
	yc.gain[0] = 128;
	yc.irq = ym2_irq;
	rc = rc || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	if (!rc) {
		s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
		s_ym2_cap = (int)(rbx_ym_rate(1) / 50) + 256;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		s_ym2buf = (int32_t *)calloc((size_t)s_ym2_cap, sizeof(int32_t));
		rc = !s_mix || !s_ym2buf || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 0x30);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		mc_exit();
		return -1;
	}
	for (int c = 0; c < 2; c++) {
		m6809_t *m = &s_cpu[c];
		m6809_init(m);
		const uint8_t *rom = c ? s_sub_rom : s_rom;
		for (int p = 0; p < 0x80; p++)
			m->rpage[p] = rom + p * 256;
		for (int p = 0; p < 0x20; p++)
			m->rpage[0x80 + p] = m->wpage[0x80 + p] = s_shared + p * 256;
		m->rpage[0xff] = rom + 0xff00;   // (the vectors: copied to 0xfff0 by the region)
		if (!c) {
			for (int p = 0xa0; p < 0xc0; p++)
				m->rpage[p] = rom + p * 256;
			for (int p = 0; p < 8; p++)
				m->rpage[0xd8 + p] = m->wpage[0xd8 + p] = s_fgram + p * 256;
			m->rpage[0xe8] = m->wpage[0xe8] = s_spr;
			m->rpage[0xe9] = m->wpage[0xe9] = s_spr + 0x100;
		}
		m->read = c ? sub_r : main_r;
		m->write = c ? sub_w : main_w;
	}
	s_port_in[0] = rbx_port_find("IN0");
	s_port_in[1] = rbx_port_find("IN1");
	s_port_in[2] = rbx_port_find("IN2");
	s_port_dsw = rbx_port_find("DSW");
	mc_reset();
	return 0;
}

static void mc_exit(void)
{
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix);
	free(s_ym2buf);
	s_mix = s_ym2buf = NULL;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
}

static void mc_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	coin_update();
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		s_cpu_frac += (uint64_t)CPU_CLOCK * 1000;
		const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
		const int64_t cyc = (int64_t)(s_cpu_frac / den);
		s_cpu_frac -= (uint64_t)cyc * den;
		m6809_run_until(&s_cpu[0], s_cpu[0].cycles + cyc);
		m6809_run_until(&s_cpu[1], s_cpu[1].cycles + cyc);
		s_mix_n += rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&s_ym1_frac, YM1_CLOCK, FRAME_MHZ, LINES), s_mix + s_mix_n,
		                          s_mix_cap - s_mix_n);
		s_ym2_n += rbx_ym_advance(1, (uint32_t)rbx_line_cycles(&s_ym2_frac, YM2_CLOCK, FRAME_MHZ, LINES),
		                          s_ym2buf + s_ym2_n, s_ym2_cap - s_ym2_n);
	}
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

static int mc_state(void *ctx, int load)
{
	uint8_t cpu[2][M6809_STATE_SIZE];
	if (!load)
		for (int c = 0; c < 2; c++)
			memcpy(cpu[c], &s_cpu[c], M6809_STATE_SIZE);
	uint64_t fr[3] = { s_cpu_frac, s_ym1_frac, s_ym2_frac };
	int32_t misc[10] = { s_rambank, s_write_mask, s_gfxbank, s_flip, s_coin_prev, s_main_nmi, s_main_irq_ym, s_sub_irq,
	                     s_sub_nmi, 0 };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m6809a", cpu[0], M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "m6809b", cpu[1], M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "shared", s_shared, sizeof(s_shared));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "palx", s_pal_ext, sizeof(s_pal_ext));
	rc |= rbx_state_io(ctx, "fg", s_fgram, sizeof(s_fgram));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "bg", s_bgram, sizeof(s_bgram));
	rc |= rbx_state_io(ctx, "other", s_otherram, sizeof(s_otherram));
	rc |= rbx_state_io(ctx, "scroll", s_scrollx, sizeof(s_scrollx));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		for (int c = 0; c < 2; c++)
			memcpy(&s_cpu[c], cpu[c], M6809_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_ym1_frac = fr[1];
		s_ym2_frac = fr[2];
		set_rambank(misc[0]);
		s_write_mask = misc[1];
		s_gfxbank = misc[2];
		s_flip = misc[3];
		s_coin_prev = misc[4];
		s_main_nmi = misc[5];
		s_main_irq_ym = misc[6];
		s_sub_irq = misc[7];
		s_sub_nmi = misc[8];
	}
	return rc ? -1 : 0;
}

static int mc_save(void *ctx) { return mc_state(ctx, 0); }
static int mc_load(void *ctx) { return mc_state(ctx, 1); }
static int mc_supported(const char *machine) { return !strcmp(machine, "metlclsh"); }

const rbx_driver rbx_drv_metlclsh = { "DECO Metal Clash", mc_init, mc_exit, mc_reset, mc_frame, mc_save, mc_load,
                                      mc_supported, mc_render, NULL };
