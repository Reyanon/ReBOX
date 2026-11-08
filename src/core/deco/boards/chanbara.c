/*  Data East Chanbara, from MAME src/mame/dataeast/chanbara.cpp (BSD-3-Clause, copyright-holders: Tomasz Slanina,
    David Haywood).
    A 6809 at 1.5 MHz with a YM2203 (its IRQ the CPU's only interrupt; its ports: the background's y scroll, the flip
    and the ROM bank), a 256x512 background of 16x16 tiles in 3 bpp, a 2 bpp 8x8 text layer, byte-coded sprites
    (16x16 / 16x32) behind the text or the background's pens by an attribute bit, the PROM palette.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 1500000        // 12 MHz / 8
#define YM_CLOCK 1500000
#define FRAME_MHZ 57445          // set_raw(12 MHz / 2, 384, 0, 256, 272, 8, 248)
#define LINES 272
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 256

static uint8_t *s_rom, *s_bank_rom;
static uint32_t s_rom_size, s_bank_size;
static uint8_t s_ram[0x1c00];              // 0x0000 .. 0x1bff (RAM, text, its colours, sprites, background, its colours)
static int s_scroll, s_flip, s_bank;
static m6809_t s_cpu;
static rbx_gfx s_gfx8, s_gfx16, s_gfxspr;
static uint32_t s_palette[256];
static uint8_t s_pri_buf[VIS_W * VIS_H];
static int s_port_dsw, s_port_sys, s_port_p[2];
static int s_line;
static uint64_t s_cpu_frac, s_ym_frac;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static struct {
	uint8_t ram[0x1c00];
	int scroll, flip;
} *s_snap;

#define TXT 0x0800   // (s_ram offsets)
#define TXT_COL 0x0c00
#define SPR 0x1000
#define BG 0x1800
#define BG_COL 0x1a00

// ---- bus ----
static int vblank(void) { return s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H; }
static void set_bank(int b)
{
	s_bank = b & 1;
	for (int p = 0; p < 0x40; p++)
		s_cpu.rpage[0x40 + p] = s_bank_rom + (uint32_t)s_bank * 0x4000 + (uint32_t)p * 256;
}
static void ym_irq(int state) { m6809_set_irq(&s_cpu, state); }
static void ym_porta_w(uint8_t v) { s_scroll = (s_scroll & ~0xff) | v; }
static void ym_portb_w(uint8_t v)   // the scroll's bit 8, the flip, the bank
{
	s_scroll = (s_scroll & 0xff) | ((v << 8) & 0x100);
	s_flip = (v >> 1) & 1;
	set_bank((v & 4) >> 2);
}
static uint8_t cpu_r(void *ctx, uint16_t a)
{
	(void)ctx;
	switch (a) {
	case 0x2000: return (uint8_t)rbx_port_read(s_port_dsw);
	case 0x2001:
		rbx_port_custom(s_port_sys, 0x80, vblank() ? 0x80 : 0);
		return (uint8_t)rbx_port_read(s_port_sys);
	case 0x2002: return (uint8_t)rbx_port_read(s_port_p[1]);
	case 0x2003: return (uint8_t)rbx_port_read(s_port_p[0]);
	case 0x3800: case 0x3801: return rbx_ym_read(0, a & 1);
	}
	return 0;
}
static void cpu_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a == 0x3800 || a == 0x3801)
		rbx_ym_write(0, a & 1, v);
}

// ---- video ----
// a 16x16 sprite element with MAME prio_transpen (drawn where 1 << pri is not in pmask, bit 31 implied; pri = 31)
static void sprite(uint32_t code, int colour, int fx, int fy, int sx, int sy, uint32_t pmask, uint16_t *bm, int pitch)
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
		const uint8_t *row = src + (fy ? 15 - y : y) * 16;
		for (int x = 0; x < 16; x++) {
			const int dx = sx + x;
			if (dx < 0 || dx >= VIS_W)
				continue;
			const uint8_t p = row[fx ? 15 - x : x];
			if (!p)
				continue;
			uint8_t *pr = &s_pri_buf[dy * VIS_W + dx];
			if (!((1u << (*pr & 0x1f)) & pmask))
				bm[dy * pitch + dx] = (uint16_t)(0x80 + colour * 8 + p);
			*pr = 31;
		}
	}
}

static void cb_render(void)
{
	memcpy(rbx_video.palette, s_palette, sizeof(s_palette));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, flip = s_snap->flip;
	const uint8_t *ram = s_snap->ram;
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = bm + y * pitch;
		uint8_t *pr = s_pri_buf + y * VIS_W;
		for (int x = 0; x < VIS_W; x++) {
			// the background (16x32 of 16x16, y scrolled; colour bit 0 flips the tile both ways): opaque, its
			// non-zero pens priority 1
			int px = x, py = (y + VIS_Y0 + s_snap->scroll) & 0x1ff;
			if (flip) {
				px = 255 - px;
				py = 511 - py;
			}
			int idx = (py >> 4) * 16 + (px >> 4);
			int attr = ram[BG_COL + idx];
			int tx = px & 15;
			const int ty = py & 15;
			if (attr & 1)   // (TILE_FLIPXY(1): the x flip)
				tx = 15 - tx;
			const uint32_t code = s_gfx16.count ? (uint32_t)ram[BG + idx] % s_gfx16.count : 0;
			uint8_t p = s_gfx16.pix[(code * 16 + (uint32_t)ty) * 16 + (uint32_t)tx];
			d[x] = (uint16_t)(((attr >> 1) & 0x1f) * 8 + p);
			pr[x] = p ? 1 : 0;
			// the text (32x32 of 8x8): its non-zero pens priority 2
			int qx = x, qy = y + VIS_Y0;
			if (flip) {
				qx = 255 - qx;
				qy = 255 - qy;
			}
			idx = (qy >> 3) * 32 + (qx >> 3);
			attr = ram[TXT_COL + idx];
			const uint32_t c8 = s_gfx8.count ? (uint32_t)(ram[TXT + idx] + ((attr & 1) << 8)) % s_gfx8.count : 0;
			p = s_gfx8.pix[(c8 * 8 + (uint32_t)(qy & 7)) * 8 + (uint32_t)(qx & 7)];
			if (p) {
				d[x] = (uint16_t)(0x40 + ((attr >> 1) & 0x1f) * 4 + p);
				pr[x] = 2;
			}
		}
	}
	for (int offs = 0x80 - 4; offs >= 0; offs -= 4) {   // draw_sprites
		const uint8_t *s = ram + SPR + offs;
		const int attr = s[0];
		if (!(attr & 1))
			continue;
		const uint32_t pmask = (s[0x80] & 0x80) ? 0xfffc : 0xfffe;   // (behind the text / behind both)
		uint32_t code = s[1];
		const int colour = s[0x80] & 0x0f;
		int fx = (attr >> 2) & 1, fy = (attr >> 1) & 1;
		int sx = (240 - s[3]) & 0xff, sy = (240 - s[2]) & 0xff;
		if (flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			fx = !fx;
			fy = !fy;
		}
		if (s[0x80] & 0x10) code += 0x200;
		if (s[0x80] & 0x20) code += 0x400;
		if (s[0x80] & 0x40) code += 0x100;
		if (attr & 0x10) {   // 16x32
			code &= ~1u;
			if (flip)
				sy += 16;
			sprite(code | (uint32_t)(fy ^ 1), colour, fx, fy, sx, sy, pmask, bm, pitch);
			sprite(code | (uint32_t)fy, colour, fx, fy, sx, sy - 16, pmask, bm, pitch);
		} else
			sprite(code, colour, fx, fy, sx, sy, pmask, bm, pitch);
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->ram, s_ram, sizeof(s_ram));
	s_snap->scroll = s_scroll;
	s_snap->flip = s_flip;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void cb_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	s_scroll = s_flip = 0;
	set_bank(0);
	m6809_reset(&s_cpu);
	m6809_set_irq(&s_cpu, 0);
	rbx_ym_reset(0);
	s_mix_n = 0;
	s_line = 0;
	s_cpu_frac = s_ym_frac = 0;
}

static void cb_exit(void);

static int cb_init(const rbx_game *g)
{
	uint32_t csize, tsize, t2size, ssize, psize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_bank_rom = rbx_region("user1", &s_bank_size);
	uint8_t *chars = rbx_region("gfx1", &csize), *tiles = rbx_region("gfx3", &tsize), *tiles2 = rbx_region("gfx4", &t2size),
	        *spr = rbx_region("sprites", &ssize), *prom = rbx_region("proms", &psize);
	if (!s_rom || s_rom_size < 0x10000 || !s_bank_rom || s_bank_size < 0x8000 || !chars || csize < 0x2000 || !tiles ||
	    tsize < 0x8000 || !tiles2 || t2size < 0x2000 || !spr || ssize < 0x30000 || !prom || psize < 0x300) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	for (int i = 0; i < 0x1000; i++) {   // init_chanbara: cp14's nibbles spread into the background's third plane
		tiles[0x4000 + i + 0x1000] = tiles2[i] & 0xf0;
		tiles[0x4000 + i] = (uint8_t)((tiles2[i] & 0x0f) << 4);
		tiles[0x4000 + i + 0x3000] = tiles2[i + 0x1000] & 0xf0;
		tiles[0x4000 + i + 0x2000] = (uint8_t)((tiles2[i + 0x1000] & 0x0f) << 4);
	}
	{	// tilelayout (512, 2 bpp in the nibbles, the left half the region's second half), tile16layout (256, 3 bpp:
		// the region's half, 0, 4; the quarters across x), spritelayout (2048, 3 bpp: the thirds)
		static uint32_t xc[8], y8[8], xt[16], y16[16], xs[16], ys[16];
		for (int i = 0; i < 4; i++) {
			xc[i] = (uint32_t)(0x1000 * 8 + i);
			xc[4 + i] = (uint32_t)i;
			xt[i] = (uint32_t)(16 * 8 + 0x2000 * 8 + i);
			xt[4 + i] = (uint32_t)i;
			xt[8 + i] = (uint32_t)(0x2000 * 8 + i);
			xt[12 + i] = (uint32_t)(16 * 8 + i);
		}
		for (int i = 0; i < 8; i++) {
			y8[i] = (uint32_t)(i * 8);
			xs[i] = (uint32_t)(2 * 8 * 8 + i);
			xs[8 + i] = (uint32_t)i;
			ys[i] = (uint32_t)(i * 8);
			ys[8 + i] = (uint32_t)(8 * 8 + i * 8);
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 8);
		static const uint32_t pc[2] = { 0, 4 }, pt[3] = { 0x4000 * 8, 0, 4 }, ps[3] = { 0x20000 * 8, 0x10000 * 8, 0 };
		const rbx_gfx_layout l8 = { 8, 8, 2, pc, xc, y8, 8 * 8 };
		const rbx_gfx_layout lt = { 16, 16, 3, pt, xt, y16, 32 * 8 };
		const rbx_gfx_layout ls = { 16, 16, 3, ps, xs, ys, 16 * 16 };
		if (rbx_gfx_decode(&s_gfx8, &l8, chars, 512 * 8) || rbx_gfx_decode(&s_gfx16, &lt, tiles, 256 * 32) ||
		    rbx_gfx_decode(&s_gfxspr, &ls, spr, 2048 * 32)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			cb_exit();
			return -1;
		}
	}
	for (int i = 0; i < 256; i++) {   // palette: pal4bit(prom << 1) of the three PROMs
		const int r = (prom[i] << 1) & 15, gg = (prom[256 + i] << 1) & 15, b = (prom[512 + i] << 1) & 15;
		s_palette[i] = (uint32_t)((r * 17) << 16 | (gg * 17) << 8 | b * 17);
	}
	s_snap = calloc(1, sizeof(*s_snap));
	// sound: YM2203 1.5 MHz, all outputs 1.0
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 256;
	yc.irq = ym_irq;
	yc.port_a_write = ym_porta_w;
	yc.port_b_write = ym_portb_w;
	int rc = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	if (!rc) {
		s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		rc = !s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 256);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		cb_exit();
		return -1;
	}
	m6809_init(&s_cpu);
	for (int p = 0; p < 0x1c; p++)   // (RAM, text, its colours, sprites, background, its colours: 0x1100 .. 0x17ff open)
		if (p < 0x11 || p >= 0x18)
			s_cpu.rpage[p] = s_cpu.wpage[p] = s_ram + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + p * 256;
	s_cpu.read = cpu_r;
	s_cpu.write = cpu_w;
	s_port_dsw = rbx_port_find("DSW1");
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	cb_reset();
	return 0;
}

static void cb_exit(void)
{
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
}

static void cb_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		s_cpu_frac += (uint64_t)CPU_CLOCK * 1000;
		const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
		const int64_t cyc = (int64_t)(s_cpu_frac / den);
		s_cpu_frac -= (uint64_t)cyc * den;
		m6809_run_until(&s_cpu, s_cpu.cycles + cyc);
		s_mix_n += rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&s_ym_frac, YM_CLOCK, FRAME_MHZ, LINES), s_mix + s_mix_n,
		                          s_mix_cap - s_mix_n);
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int cb_state(void *ctx, int load)
{
	uint8_t cpu[M6809_STATE_SIZE];
	if (!load)
		memcpy(cpu, &s_cpu, M6809_STATE_SIZE);
	uint64_t fr[2] = { s_cpu_frac, s_ym_frac };
	int32_t misc[3] = { s_scroll, s_flip, s_bank };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m6809", cpu, M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		memcpy(&s_cpu, cpu, M6809_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_ym_frac = fr[1];
		s_scroll = misc[0];
		s_flip = misc[1];
		set_bank(misc[2]);
	}
	return rc ? -1 : 0;
}

static int cb_save(void *ctx) { return cb_state(ctx, 0); }
static int cb_load(void *ctx) { return cb_state(ctx, 1); }
static int cb_supported(const char *machine) { return !strcmp(machine, "chanbara"); }

const rbx_driver rbx_drv_chanbara = { "DECO Chanbara", cb_init, cb_exit, cb_reset, cb_frame, cb_save, cb_load,
                                      cb_supported, cb_render, NULL };
