/*  Technos Battle Lane! Vol. 5. Ported from MAME src/mame/technos/battlane.cpp (BSD-3-Clause, copyright-holders:
    Paul Leaman).
    Two MC6809E at 1.5 MHz on one memory map (their own ROM at 0x4000), IRQs between them through the CPU control
    register, both NMIs at vblank when enabled; a YM3526 on the main CPU's FIRQ. Video: a 512x512 tilemap of 16x16, 16x16
    sprites, a 256x256 3-bit bitmap drawn by planes; palette written as RGB (the blue low bit from the video control).
    The screen is MAME's 60 Hz with 2.5 ms of vblank (not raw timings); the CPUs run by MAME's 6 kHz quantum.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "tec8.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 60000
#define CPU_CLOCK 1500000
#define YM_CLOCK 3000000
#define FRAME_CYCLES 25000       // (1.5 MHz / 60)
#define VIS_CYCLES 21250         // (the frame less 2.5 ms)
#define VIS_STEPS 256            // (steps of about 83 cycles; MAME: a 6 kHz quantum)
#define VBL_STEPS 45
#define VIS_X0 8
#define VIS_W 240
#define VIS_H 256

static uint8_t *s_rom, *s_srom;
static uint32_t s_rom_size, s_srom_size;
static rbx_gfx s_gspr, s_gtile[2];
static int s_port[4];
static m6809_t s_cpu[2];
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static int s_vblank;

static struct {
	uint8_t ram[0x1000], tile[0x800], spr[0x100], bram[0x2000];
	uint8_t bitmap[256 * 256];
	uint32_t pal[64];
	uint8_t video_ctrl, cpu_control;
	uint16_t scrollx, scrolly;
	int32_t ym_irq;
	int64_t cyc[2];
	uint64_t ym_frac;
	uint8_t state_end;
} *t;   // (calloc in init: ADR-040)

static struct {
	uint8_t tile[0x800], spr[0x100], bitmap[256 * 256];
	uint32_t pal[64];
	uint16_t scrollx, scrolly;
	uint8_t flip;
} *s_snap;

static void ym_irq(int state)
{
	t->ym_irq = state;
	m6809_set_firq(&s_cpu[0], state);
}

// ---- the CPUs' map (prg_map) ----
static uint8_t cpu_read(void *u, uint16_t a)
{
	const int n = (int)(intptr_t)u;
	if (a >= 0x4000) return n ? s_srom[a] : s_rom[a];
	if (a < 0x1000) return t->ram[a];
	if (a < 0x1800) return t->tile[a - 0x1000];
	if (a < 0x1900) return t->spr[a - 0x1800];
	if (a >= 0x2000) return t->bram[a - 0x2000];
	switch (a) {
	case 0x1c00: return (uint8_t)rbx_port_read(s_port[0]);
	case 0x1c01:
		rbx_port_custom(s_port[1], 0x80, s_vblank ? 0x80u : 0);
		return (uint8_t)rbx_port_read(s_port[1]);
	case 0x1c02: return (uint8_t)rbx_port_read(s_port[2]);
	case 0x1c03: return (uint8_t)rbx_port_read(s_port[3]);
	case 0x1c04: case 0x1c05: return rbx_ym_read(0, a & 1);
	}
	return 0;
}

static void cpu_command_w(uint8_t v)
{
	t->cpu_control = v;
	if (v & 4) m6809_set_irq(&s_cpu[0], 0); else m6809_hold_irq(&s_cpu[0]);
	if (v & 2) m6809_set_irq(&s_cpu[1], 0); else m6809_hold_irq(&s_cpu[1]);
}

static void cpu_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000) return;
	if (a < 0x1000) { t->ram[a] = v; return; }
	if (a < 0x1800) { t->tile[a - 0x1000] = v; return; }
	if (a < 0x1900) { t->spr[a - 0x1800] = v; return; }
	if (a >= 0x2000) {   // bitmap_w
		const uint32_t o = a - 0x2000u;
		t->bram[o] = v;
		int orval = (~t->video_ctrl >> 1) & 7;
		if (!orval)
			orval = 7;
		uint8_t *p = &t->bitmap[(o & 0xff) * 256 + (o >> 8) * 8];
		for (int i = 0; i < 8; i++) {
			if (v & (1 << i)) p[i] |= (uint8_t)orval;
			else p[i] &= (uint8_t)~orval;
		}
		return;
	}
	if (a >= 0x1e00 && a < 0x1e40) {   // palette_w
		const int nv = ~v;
		const int r = 0x21 * (nv & 1) + 0x47 * ((nv >> 1) & 1) + 0x97 * ((nv >> 2) & 1);
		const int g = 0x21 * ((nv >> 3) & 1) + 0x47 * ((nv >> 4) & 1) + 0x97 * ((nv >> 5) & 1);
		const int b = 0x21 * ((~t->video_ctrl >> 7) & 1) + 0x47 * ((nv >> 6) & 1) + 0x97 * ((nv >> 7) & 1);
		t->pal[a - 0x1e00] = (uint32_t)(r << 16 | g << 8 | b);
		return;
	}
	switch (a) {
	case 0x1c00: t->video_ctrl = v; break;
	case 0x1c01: t->scrollx = (uint16_t)(((t->video_ctrl & 1) << 8) + v); break;
	case 0x1c02: t->scrolly = (uint16_t)(((t->cpu_control & 1) << 8) + v); break;
	case 0x1c03: cpu_command_w(v); break;
	case 0x1c04: case 0x1c05: rbx_ym_write(0, a & 1, v); break;
	}
}

// ---- video ----
static void bl_render(void)
{
	memcpy(rbx_video.palette, s_snap->pal, sizeof(s_snap->pal));
	const int flip = s_snap->flip;
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y + s_snap->scrolly : y + s_snap->scrolly) & 511;
		uint32_t last = 0xffffffffu, cbase = 0;
		const uint8_t *trow = NULL;
		for (int x = VIS_X0; x < VIS_X0 + VIS_W; x++) {
			const uint32_t tx = (uint32_t)(flip ? 255 - x + s_snap->scrollx : x + s_snap->scrollx) & 511;
			if ((tx >> 4) != last) {   // (a tile looked up once for its 16 pixels)
				last = tx >> 4;
				const uint32_t col = tx >> 4, row = ty >> 4;
				const uint32_t idx = (row & 0xf) * 16 + (col & 0xf) + (row & 0x10) * 32 + (col & 0x10) * 16;
				const uint8_t attr = s_snap->tile[idx + 0x400];
				const rbx_gfx *g = &s_gtile[attr & 1];
				const uint32_t code = rbx_wrap(s_snap->tile[idx], g->count);
				trow = g->pix + code * 256 + (ty & 15) * 16;
				cbase = 32 + (uint32_t)((attr >> 1) & 3) * 8;
			}
			d[x - VIS_X0] = (uint16_t)(cbase + trow[tx & 15]);
		}
	}
	for (int o = 0; o < 0x100; o += 4) {
		const uint8_t *s = &s_snap->spr[o];
		const int attr = s[1];
		uint32_t code = s[3];
		code += 256u * ((attr >> 6) & 2);
		code += 256u * ((attr >> 5) & 1);
		if (!(attr & 1))
			continue;
		const uint16_t base = (uint16_t)(((attr >> 3) & 1) * 8);
		int sx = s[2], sy = s[0], fx = (attr >> 2) & 1, fy = (attr >> 1) & 1;
		if (!flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			fx = !fx;
			fy = !fy;
		}
		if (attr & 0x10) {
			if (flip)
				sy += 16;
			for (int i = 0; i < 2; i++)
				tec8_transpen(&s_gspr, code + (uint32_t)i, base, fx, fy, sx - VIS_X0, sy - 16 * (i ^ fy), 0, VIS_H, 0, VIS_W);
		} else
			tec8_transpen(&s_gspr, code, base, fx, fy, sx - VIS_X0, sy, 0, VIS_H, 0, VIS_W);
	}
	for (int y = 0; y < 256; y++)   // draw_fg_bitmap
		for (int x = 0; x < 256; x++) {
			const uint8_t p = s_snap->bitmap[y * 256 + x];
			if (!p)
				continue;
			const int px = flip ? 255 - x : x, py = flip ? 255 - y : y;
			if (px >= VIS_X0 && px < VIS_X0 + VIS_W)
				rbx_video.bitmap[py * rbx_video.pitch + px - VIS_X0] = p;
		}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->tile, t->tile, sizeof(t->tile));
	memcpy(s_snap->spr, t->spr, sizeof(t->spr));
	memcpy(s_snap->bitmap, t->bitmap, sizeof(t->bitmap));
	memcpy(s_snap->pal, t->pal, sizeof(t->pal));
	s_snap->scrollx = t->scrollx;
	s_snap->scrolly = t->scrolly;
	s_snap->flip = (t->cpu_control & 0x80) != 0;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void bl_reset(void)
{
	memset(t, 0, offsetof(__typeof__(*t), state_end));
	for (int n = 0; n < 2; n++) {
		m6809_t *c = &s_cpu[n];
		m6809_init(c);
		c->read = cpu_read;
		c->write = cpu_write;
		c->ctx = (void *)(intptr_t)n;
		for (int p = 0; p < 0x10; p++)
			c->rpage[p] = c->wpage[p] = t->ram + p * 256;
		for (int p = 0x40; p < 0x100; p++)
			c->rpage[p] = (n ? s_srom : s_rom) + p * 256;
		m6809_reset(c);
	}
	rbx_ym_reset(0);
	s_mix_n = 0;
	s_vblank = 0;
}

static void bl_exit(void);

static int bl_init(const rbx_game *g)
{
	free(t);
	t = calloc(1, sizeof(*t));
	if (!t) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t ssize, tsize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("sub", &s_srom_size);
	uint8_t *spr = rbx_region("sprites", &ssize), *tiles = rbx_region("tiles", &tsize);
	if (!s_rom || s_rom_size < 0x10000 || !s_srom || s_srom_size < 0x10000 || !spr || !tiles || tsize < 0xc000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{	// spritelayout (RGN_FRAC(1,3)), tilelayout, tilelayout2
		static const uint32_t xs[16] = { 7, 6, 5, 4, 3, 2, 1, 0, 135, 134, 133, 132, 131, 130, 129, 128 };
		static const uint32_t ys[16] = { 120, 112, 104, 96, 88, 80, 72, 64, 56, 48, 40, 32, 24, 16, 8, 0 };
		static const uint32_t xt[16] = { 3, 2, 1, 0, 11, 10, 9, 8, 19, 18, 17, 16, 27, 26, 25, 24 };
		static uint32_t yt[16];
		for (int i = 0; i < 16; i++)
			yt[i] = (uint32_t)i * 32;
		const uint32_t f = ssize / 3 * 8;
		const uint32_t ps[3] = { 0, f, 2 * f };
		static const uint32_t p1[3] = { 0x8000 * 8 + 4, 4, 0 }, p2[3] = { 0x8000 * 8, 0x4000 * 8 + 4, 0x4000 * 8 };
		const rbx_gfx_layout ls = { 16, 16, 3, ps, xs, ys, 256 };
		const rbx_gfx_layout l1 = { 16, 16, 3, p1, xt, yt, 512 }, l2 = { 16, 16, 3, p2, xt, yt, 512 };
		if (rbx_gfx_decode(&s_gspr, &ls, spr, ssize / 3) || rbx_gfx_decode(&s_gtile[0], &l1, tiles, 256 * 64) ||
		    rbx_gfx_decode(&s_gtile[1], &l2, tiles, 256 * 64)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			bl_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM3526;
	yc.clock = YM_CLOCK;
	yc.gain[0] = 256;
	yc.irq = ym_irq;
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		bl_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 64)) {
		rbx_fail("%s: out of memory", g->set);
		bl_exit();
		return -1;
	}
	static const char *const names[4] = { "P1", "P2", "DSW1", "DSW2" };
	for (int i = 0; i < 4; i++)
		s_port[i] = rbx_port_find(names[i]);
	bl_reset();
	return 0;
}

static void bl_exit(void)
{
	rbx_video_wait_idle();
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gspr);
	rbx_gfx_free(&s_gtile[0]);
	rbx_gfx_free(&s_gtile[1]);
	free(s_snap);
	s_snap = NULL;
	free(t);
	t = NULL;
}

static void run_to(int64_t c)   // both CPUs to a frame cycle, the main CPU first
{
	for (int n = 0; n < 2; n++)
		m6809_run_until(&s_cpu[n], t->cyc[n] + c);
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&t->ym_frac, YM_CLOCK, FRAME_MHZ, VIS_STEPS + VBL_STEPS);
	s_mix_n += rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
}

static void bl_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	s_vblank = 0;
	for (int k = 0; k < VIS_STEPS; k++)
		run_to((int64_t)VIS_CYCLES * (k + 1) / VIS_STEPS);
	s_vblank = 1;
	vblank_capture();
	if (~t->cpu_control & 8) {   // cpu1_interrupt (vblank): both NMIs pulsed
		for (int n = 0; n < 2; n++) {
			m6809_set_nmi(&s_cpu[n], 1);
			m6809_set_nmi(&s_cpu[n], 0);
		}
	}
	for (int k = 0; k < VBL_STEPS; k++)
		run_to(VIS_CYCLES + (int64_t)(FRAME_CYCLES - VIS_CYCLES) * (k + 1) / VBL_STEPS);
	t->cyc[0] += FRAME_CYCLES;
	t->cyc[1] += FRAME_CYCLES;
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int bl_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "bl", t, (uint32_t)offsetof(__typeof__(*t), state_end));
	rc |= rbx_state_io(ctx, "bl_cpu0", &s_cpu[0], (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "bl_cpu1", &s_cpu[1], (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_ym_state(0, ctx, load);
	return rc ? -1 : 0;
}

static int bl_save(void *ctx) { return bl_state(ctx, 0); }
static int bl_load(void *ctx) { return bl_state(ctx, 1); }
static int bl_supported(const char *machine) { return !strcmp(machine, "battlane"); }

const rbx_driver rbx_drv_battlane = { "Technos Battle Lane", bl_init, bl_exit, bl_reset, bl_frame, bl_save, bl_load,
                                      bl_supported, bl_render, NULL };
