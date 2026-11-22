/*  Seibu Kaihatsu Air Raid / Cross Shooter (single PCB) and Fail Gate: a Z80 (Air Raid: opcodes encrypted) with a
    banked data ROM, the Air Raid video (two ROM-mapped 16x16 tilemaps, an 8x8 text layer, 16x16 sprites mixed through
    color lookup PROMs, 256 colors of xBGR_444 RAM), the Seibu Sound System (SEI80BU Z80, YM2151).
    Ported from MAME 0.289 src/mame/seibu/airraid.cpp (BSD-3-Clause, copyright-holders: Tomasz Slanina, Angelo Salese,
    hap, David Haywood) and src/mame/seibu/airraid_dev.cpp (BSD-3-Clause, copyright-holders: Tomasz Slanina, Angelo
    Salese, hap, David Haywood).
    This file is part of ReBOX. GPL-2.0-or-later.

    The CPUs interleaved by the Seibu scheduler (sched.h). Air Raid / Cross Shooter: their background, foreground and
    sprite ROMs are not dumped (MAME: not working); only the text layer shows.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../machine/sched.h"
#include "../machine/seibusnd.h"
#include "../video/seibu_draw.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define VIS_W 256
#define VIS_Y0 16
#define VIS_H 224
#define LINES 256
#define CPU_CLOCK 6000000

static uint8_t *s_rom, *s_data, *s_op;
static const uint8_t *s_bgmap, *s_fgmap;
static uint32_t s_bgmap_size, s_fgmap_size;
static uint8_t s_clut[4][0x100];   // tx, bg, fg, spr
static rbx_gfx s_gtx, s_gspr, s_gbg, s_gfg;
static int s_port[5];
static z80 s_main;
static uint16_t *s_temp;

static struct {
	uint8_t ram[0x1e00], spr[0x200], tx[0x800], pal[0x200], vregs[0x10];
	uint8_t hw, bank;
	int irq;
	uint8_t state_end;
} t;

static struct {
	uint8_t spr[0x200], tx[0x800], vregs[0x10], hw;
	uint32_t pal[256];
} *s_snap;

// ---- the main CPU ----
// run ahead (lazy synchronization, see z_run): its writes held to the instruction's end, an access to the sound
// latches undoing the instruction
static int s_defer, s_hazard, s_wq_n;
static uint16_t s_wq_a[8];
static uint8_t s_wq_v[8];

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_data[a];
	if (a < 0xc000) return s_rom[0x10000 + (uint32_t)t.bank * 0x4000 + (a - 0x8000)];   // (maindata)
	if (a >= 0xc000 && a <= 0xc004) return (uint8_t)rbx_port_read(s_port[a - 0xc000]);
	if (a >= 0xd000 && a < 0xd800) return t.tx[a - 0xd000];
	if (a >= 0xd800 && a < 0xd900) return t.pal[a - 0xd800];
	if (a >= 0xda00 && a < 0xdb00) return t.pal[0x100 + a - 0xda00];
	if (a >= 0xdc00 && a < 0xdc10) return t.vregs[a - 0xdc00];
	if (a >= 0xde00 && a < 0xde10) {
		if (s_defer) {   // (run ahead: the latches are the sound CPU's, at their time only)
			s_hazard = 1;
			return 0xff;
		}
		return seibusnd_main_r(a - 0xde00);
	}
	if (a >= 0xe000 && a < 0xfe00) return t.ram[a - 0xe000];
	if (a >= 0xfe00) return t.spr[a - 0xfe00];
	return 0xff;
}

static void main_write_now(uint16_t a, uint8_t v)
{
	if (a >= 0xd000 && a < 0xd800) t.tx[a - 0xd000] = v;
	else if (a >= 0xd800 && a < 0xd900) t.pal[a - 0xd800] = v;
	else if (a >= 0xda00 && a < 0xdb00) t.pal[0x100 + a - 0xda00] = v;
	else if (a >= 0xdc00 && a < 0xdc10) t.vregs[a - 0xdc00] = v;
	else if (a == 0xdc11) {   // bank_w: TTBB tbfs
		t.bank = (v >> 4) & 3;
		t.hw = v & 0xcf;
		FGP(12, 1);
		memcpy(s_op + 0x8000, s_rom + 0x10000 + (uint32_t)t.bank * 0x4000, 0x4000);   // (the bank's opcodes: plain)
	} else if (a >= 0xde00 && a < 0xde10) seibusnd_main_w(a - 0xde00, v);
	else if (a >= 0xe000 && a < 0xfe00) t.ram[a - 0xe000] = v;
	else if (a >= 0xfe00) t.spr[a - 0xfe00] = v;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_defer) {
		if ((a >= 0xde00 && a < 0xde10) || s_wq_n >= 8)
			s_hazard = 1;
		else {
			s_wq_a[s_wq_n] = a;
			s_wq_v[s_wq_n++] = v;
		}
		return;
	}
	main_write_now(a, v);
}

static uint8_t z_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

// ---- the drawing ----
static inline int getscroll(const uint8_t *s, int n)
{
	return ((s[n] << 1) & 0xff) + ((s[n] & 0x80) ? 1 : 0) + (((s[n - 1] << 4) | (s[n - 1] << 12)) & 0xff00);
}

// the sprites' layer (s_temp, all 0 outside a render: sprite pixels written, then mixed and cleared) through its colour
// table into the frame: MAME's whole-screen mix of the sprite bitmap (ADR-092: the rows a sprite touched, between its
// columns; elsewhere the bitmap is 0, whose colour is drawn only when opaque)
static void mix_sprites(const uint8_t *clut, int base, const int16_t *x0, const int16_t *x1)
{
	const int zero_opaque = !(clut[0] & 0x40);
	const uint16_t zv = (uint16_t)((clut[0] & 0x3f) + base);
	for (int r = 0; r < VIS_H; r++) {
		uint16_t *d = rbx_video.bitmap + r * rbx_video.pitch;
		uint16_t *s = s_temp + r * VIS_W;
		if (x0[r] > x1[r]) {   // (no sprite on the row: its bitmap all 0)
			if (zero_opaque)
				for (int x = 0; x < VIS_W; x++)
					d[x] = zv;
			continue;
		}
		const int a = zero_opaque ? 0 : x0[r], e = zero_opaque ? VIS_W - 1 : x1[r];
		for (int x = a; x <= e; x++) {
			const uint8_t real = clut[s[x] & 0xff];
			if (!(real & 0x40))
				d[x] = (uint16_t)((real & 0x3f) + base);
		}
		memset(s + x0[r], 0, sizeof(uint16_t) * (size_t)(x1[r] - x0[r] + 1));
	}
}

static inline uint32_t bg_scan(int col, int row)
{
	return (uint32_t)(((row & 0xf) * 0x10) + (col & 0xf) + (((col & 0x7f0) >> 4) * 0x100) + ((row & 0x30) >> 4) * 0x8000);
}

static inline uint32_t fg_scan(int col, int row)
{
	return (uint32_t)(((row & 0xf) * 0x10) + (col & 0xf) + (((col & 0x0f0) >> 4) * 0x100) + ((row & 0x1f0) >> 4) * 0x1000);
}

// a scrolled map's layer straight through its colour table into the frame (MAME's tilemap pixels into a bitmap, then
// its mix: the same values, a tile's run of a line at a time)
static void draw_map_mix(const uint8_t *map, uint32_t map_size, const rbx_gfx *g, int fg, int sx, int sy,
                         const uint8_t *clut, int base)
{
	const int mw = fg ? 4096 : 32768, mh = fg ? 2048 : 1024;
	for (int r = 0; r < VIS_H; r++) {
		uint16_t *d = rbx_video.bitmap + r * rbx_video.pitch;
		const int py = (r + VIS_Y0 + sy) & (mh - 1);
		int x = 0, px = sx & (mw - 1);
		while (x < VIS_W) {
			int n = 16 - (px & 15);
			if (n > VIS_W - x)
				n = VIS_W - x;
			const uint32_t idx = (fg ? fg_scan(px >> 4, py >> 4) : bg_scan(px >> 4, py >> 4)) * 2;
			const uint8_t attr = idx < map_size ? map[idx] : 0, code = idx + 1 < map_size ? map[idx + 1] : 0;
			const uint32_t tile = code | (uint32_t)((attr & 0x70) << 4);
			const uint8_t *src = sd_elem(g, tile) + (py & 15) * 16 + (px & 15);
			const int ab = (attr & 0xf) * 16;
			for (int k = 0; k < n; k++) {
				const uint8_t real = clut[(uint16_t)(ab + src[k]) & 0xff];
				if (!(real & 0x40))
					d[x + k] = (uint16_t)((real & 0x3f) + base);
			}
			x += n;
			px = (px + n) & (mw - 1);
		}
	}
}

static void ar_render(void)
{
	memcpy(rbx_video.palette, s_snap->pal, sizeof(s_snap->pal));
	const uint8_t *vr = s_snap->vregs;
	const int hw = s_snap->hw;
	for (int r = 0; r < VIS_H; r++) {
		uint16_t *d = rbx_video.bitmap + r * rbx_video.pitch;
		for (int x = 0; x < VIS_W; x++)
			d[x] = 0x80;
	}
	if (!(hw & 4)) {
		draw_map_mix(s_bgmap, s_bgmap_size, &s_gbg, 0, getscroll(vr, 0x2), getscroll(vr, 0x6) + ((hw & 0xc0) >> 6) * 256,
		             s_clut[1], 0x80);
	}
	if (!(hw & 2)) {
		draw_map_mix(s_fgmap, s_fgmap_size, &s_gfg, 1, getscroll(vr, 0xa), getscroll(vr, 0xe), s_clut[2], 0x00);
	}
	if (hw & 1) {
		int16_t x0[VIS_H], x1[VIS_H];   // (each row's columns a sprite touched; x0 > x1: none)
		for (int r = 0; r < VIS_H; r++) {
			x0[r] = VIS_W;
			x1[r] = -1;
		}
		const sd_target tg = { s_temp, VIS_W, 0, VIS_Y0, VIS_W, VIS_H, NULL };
		for (int i = 0x200 - 4; i >= 0; i -= 4) {
			const uint8_t *s = &s_snap->spr[i];
			if (s[1] & 0x80)
				continue;
			sd_transpen(&tg, &s_gspr, (uint32_t)(s[0] | (s[1] & 0x70) << 4), (uint32_t)(s[1] & 0x0f) * 16, 0, 0, s[3], s[2], 0);
			const int sx = s[3], ex = sx + s_gspr.width - 1 < VIS_W - 1 ? sx + s_gspr.width - 1 : VIS_W - 1;
			for (int j = 0; j < s_gspr.height; j++) {   // (sd_transpen's rows and columns)
				const int r = s[2] + j - VIS_Y0;
				if ((unsigned)r >= (unsigned)VIS_H)
					continue;
				if (sx < x0[r])
					x0[r] = (int16_t)sx;
				if (ex > x1[r])
					x1[r] = (int16_t)ex;
			}
		}
		mix_sprites(s_clut[3], 0x40, x0, x1);
	}
	if (hw & 8) {   // the text layer straight through its colour table (MAME: its tilemap, then the mix)
		const uint8_t *clut = s_clut[0];
		for (int r = 0; r < VIS_H; r++) {
			uint16_t *d = rbx_video.bitmap + r * rbx_video.pitch;
			const int y = r + VIS_Y0;
			const uint8_t *trow = s_snap->tx + (y >> 3) * 64;
			for (int tx = 0; tx < VIS_W / 8; tx++, d += 8) {
				const uint8_t code = trow[tx * 2], attr = trow[tx * 2 + 1];
				const uint32_t c = (uint32_t)(code << 1) | ((attr & 0x20) >> 5);
				const uint8_t *src = sd_elem(&s_gtx, c) + (y & 7) * 8;
				const int ab = (attr & 0xf) * 4;
				for (int k = 0; k < 8; k++) {
					const uint8_t real = clut[(ab + src[k]) & 0xff];
					if (!(real & 0x40))
						d[k] = (uint16_t)((real & 0x3f) + 0xc0);
				}
			}
		}
	}
}

#ifdef RBX_FG_PROF
#include <stdio.h>
#include <time.h>
static uint64_t fgp_us(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}
// RBX_FG_RBENCH=frame: at that frame's capture, the drawing before ADR-091 against ar_render on the same snapshot,
// 200 times each (their times, and whether the bitmaps are the same)
static void old_mix_layer(const uint8_t *clut, int base)
{
	for (int r = 0; r < VIS_H; r++) {
		uint16_t *d = rbx_video.bitmap + r * rbx_video.pitch;
		const uint16_t *s = s_temp + r * VIS_W;
		for (int x = 0; x < VIS_W; x++) {
			const uint8_t real = clut[s[x] & 0xff];
			if (!(real & 0x40))
				d[x] = (uint16_t)((real & 0x3f) + base);
		}
	}
}
static void old_render(void)
{
	memcpy(rbx_video.palette, s_snap->pal, sizeof(s_snap->pal));
	const uint8_t *vr = s_snap->vregs;
	const int hw = s_snap->hw;
	for (int r = 0; r < VIS_H; r++) {
		uint16_t *d = rbx_video.bitmap + r * rbx_video.pitch;
		for (int x = 0; x < VIS_W; x++)
			d[x] = 0x80;
	}
	if (!(hw & 4))
		draw_map_mix(s_bgmap, s_bgmap_size, &s_gbg, 0, getscroll(vr, 0x2), getscroll(vr, 0x6) + ((hw & 0xc0) >> 6) * 256,
		             s_clut[1], 0x80);
	if (!(hw & 2))
		draw_map_mix(s_fgmap, s_fgmap_size, &s_gfg, 1, getscroll(vr, 0xa), getscroll(vr, 0xe), s_clut[2], 0x00);
	if (hw & 1) {
		memset(s_temp, 0, sizeof(uint16_t) * VIS_W * VIS_H);
		const sd_target tg = { s_temp, VIS_W, 0, VIS_Y0, VIS_W, VIS_H, NULL };
		for (int i = 0x200 - 4; i >= 0; i -= 4) {
			const uint8_t *s = &s_snap->spr[i];
			if (s[1] & 0x80)
				continue;
			sd_transpen(&tg, &s_gspr, (uint32_t)(s[0] | (s[1] & 0x70) << 4), (uint32_t)(s[1] & 0x0f) * 16, 0, 0, s[3], s[2], 0);
		}
		old_mix_layer(s_clut[3], 0x40);
	}
	if (hw & 8) {
		for (int r = 0; r < VIS_H; r++) {
			uint16_t *d = s_temp + r * VIS_W;
			const int y = r + VIS_Y0;
			for (int x = 0; x < VIS_W; x++) {
				const int idx = (y >> 3) * 32 + (x >> 3);
				const uint8_t code = s_snap->tx[idx * 2], attr = s_snap->tx[idx * 2 + 1];
				const uint32_t c = (uint32_t)(code << 1) | ((attr & 0x20) >> 5);
				d[x] = (uint16_t)((attr & 0xf) * 4 + sd_elem(&s_gtx, c)[(y & 7) * 8 + (x & 7)]);
			}
		}
		old_mix_layer(s_clut[0], 0xc0);
	}
	memset(s_temp, 0, sizeof(uint16_t) * VIS_W * VIS_H);   // (ar_render's s_temp: all 0)
}
static void render_bench(void)
{
	static int at = -2, fr;
	if (at == -2)
		at = getenv("RBX_FG_RBENCH") ? atoi(getenv("RBX_FG_RBENCH")) : -1;
	if (++fr != at)
		return;
	const size_t sz = sizeof(uint16_t) * (size_t)rbx_video.pitch * VIS_H;
	uint16_t *a = malloc(sz), *b2 = malloc(sz);
	uint64_t t0 = fgp_us();
	for (int k = 0; k < 200; k++)
		old_render();
	const uint64_t t1 = fgp_us();
	memcpy(a, rbx_video.bitmap, sz);
	for (int k = 0; k < 200; k++)
		ar_render();
	const uint64_t t2 = fgp_us();
	memcpy(b2, rbx_video.bitmap, sz);
	fprintf(stderr, "rbench frame %d hw %02x: old %.1f us, new %.1f us, bitmaps %s\n", fr, s_snap->hw, (t1 - t0) / 200.0,
	        (t2 - t1) / 200.0, memcmp(a, b2, sz) ? "DIFFER" : "same");
	free(a);
	free(b2);
}
#endif

static void capture(void)
{
#ifdef RBX_FG_PROF
	const unsigned long long r0 = rbx_fgp_ins();
#endif
#ifdef RBX_FG_PROF
	const uint64_t w0 = fgp_us();
	rbx_video_wait_idle();
	FGP(14, fgp_us() - w0);
#else
	rbx_video_wait_idle();
#endif
	memcpy(s_snap->spr, t.spr, sizeof(t.spr));
	memcpy(s_snap->tx, t.tx, sizeof(t.tx));
	memcpy(s_snap->vregs, t.vregs, sizeof(t.vregs));
	s_snap->hw = t.hw;
	for (int i = 0; i < 256; i++) {   // xBGR_444
		const uint16_t w = (uint16_t)(t.pal[i] | t.pal[0x100 + i] << 8);
		s_snap->pal[i] = (uint32_t)(w & 0xf) * 0x110000u | (uint32_t)((w >> 4) & 0xf) * 0x1100u | (uint32_t)((w >> 8) & 0xf) * 0x11u;
	}
#ifdef RBX_FG_PROF
	render_bench();
#endif
	rbx_video_frame_done();
	FGP(19, rbx_fgp_ins() - r0);
}

// ---- the scheduler: the main Z80, the scanline timer's two interrupts ----
static unsigned long s_zrun0;

// Lazy synchronization (ADR-054; RBX_FG_TIGHT=1 off): the main CPU runs ahead of the scheduler's time, an iteration
// of z80_run (an instruction and its interrupt) at a time, to below the next of its two scanline timers, stopping
// before an instruction that touches the sound latches (undone: its writes were held). Each iteration's cycles are
// queued; z_run replays them as the scheduler's slices would have run them (while the cycles done are below the
// slice's), so the slices, and with them the sound Z80's stops, are the tight run's. The sound Z80 may then run
// ahead to below this CPU's real position (seibusnd_set_lazy).
#define MAIN_FIFO 32768            // (a power of two)
#define MAIN_MARGIN 96             // main CPU cycles kept from its timers
static seibusnd_mainq s_q;         // (.pend: the queued cycles, s_main.cyc minus these: the tight run's)
static int s_lazy_main, s_lazy_snd, s_spin;
enum { T_VBLANK = 1, T_LINE0, T_SOUND };

// Spin loops run ahead (ADR-063; RBX_FG_NOSPIN=1 off): after a short backward jump its target is a loop head
// candidate. One pass from the head back to it is watched; with no write, no latch access, no ld r,a and the
// registers back as they were (all but R and the cycle count), each further pass reads the same memory and does the
// same: its iterations' records are queued without running them, R moved by the pass's own count, while whole passes
// fit below the limit (each of their instructions starts below it, as the stepping loop's would).
#define SPIN_MAX 24                // instructions in a pass watched
static inline int z80_same_regs(const z80 *a, const z80 *b)   // (all but cyc and r)
{
	return !memcmp(&a->pc, &b->pc, offsetof(z80, r) - offsetof(z80, pc)) &&
	       !memcmp((const uint8_t *)a + offsetof(z80, r) + 1, (const uint8_t *)b + offsetof(z80, r) + 1,
	               offsetof(z80, nmi_pending) + sizeof(a->nmi_pending) - offsetof(z80, r) - 1);
}

static void main_ahead(int ran)
{
	const sched_exec *e = &g_sched.ex[0];
	int64_t lim = INT64_MAX, x;
	if (sched_timer_on(T_VBLANK, &x) && x < lim)
		lim = x;
	if (sched_timer_on(T_LINE0, &x) && x < lim)
		lim = x;
	const int64_t local = e->local + (int64_t)ran * e->period;
	lim -= MAIN_MARGIN * e->period;
	if (lim <= local)
		return;
	const unsigned long cend = s_main.cyc + (unsigned long)((lim - local) / e->period);
	int head = -1, pn = 0, hirq = 0;   // (a spin loop's head watched, the instructions of its pass)
	z80 hz;
	unsigned long hcyc = 0;
	while ((long)(s_main.cyc - cend) < 0 && s_q.t - s_q.h < MAIN_FIFO) {
		uint8_t saved[offsetof(z80, read_map) - offsetof(z80, cyc)];   // (the registers: the pointers stay)
		memcpy(saved, &s_main.cyc, sizeof(saved));
		const unsigned long cyc0 = s_main.cyc;
		const uint16_t pc0 = s_main.pc;
		const int irq = t.irq;
		s_defer = 1;
		s_hazard = 0;
		s_wq_n = 0;
		z80_run(&s_main, s_main.cyc + 1, &t.irq, 1);   // (one iteration)
		FGP(0, 1);
#ifdef RBX_FG_PROF
		rbx_fgp_mpc[pc0]++;
#endif
		s_defer = 0;
		if (s_hazard || s_main.cyc - cyc0 > 255) {
			FGP(9, 1);
			memcpy(&s_main.cyc, saved, sizeof(saved));
			t.irq = irq;
			break;
		}
		for (int k = 0; k < s_wq_n; k++)
			main_write_now(s_wq_a[k], s_wq_v[k]);
		const unsigned len = (unsigned)(s_main.cyc - cyc0);
		s_q.f[s_q.t++ & (MAIN_FIFO - 1)] = (uint8_t)len;
		s_q.pend += len;
		if (!s_spin)
			continue;
		if (head >= 0) {
			if (s_wq_n || ++pn > SPIN_MAX || (s_op[pc0] == 0xed && s_op[(uint16_t)(pc0 + 1)] == 0x4f))
				head = -1;   // (writes, too long, or ld r,a: not a spin)
			else if (s_main.pc == head) {
				const unsigned long c = s_main.cyc - hcyc;
				if (t.irq == hirq && z80_same_regs(&s_main, &hz)) {
					const long left = (long)(cend - s_main.cyc) - 1, room = (long)(MAIN_FIFO - (s_q.t - s_q.h));
					long k = left > 0 ? left / (long)c : 0;
					if (k > room / pn)
						k = room / pn;
					const uint32_t src = s_q.t - (uint32_t)pn;
					for (long i = 0; i < k; i++)
						for (int j = 0; j < pn; j++, s_q.t++)
							s_q.f[s_q.t & (MAIN_FIFO - 1)] = s_q.f[(src + (uint32_t)j) & (MAIN_FIFO - 1)];
					FGP(1, k * pn);
					s_q.pend += (unsigned long)k * c;
					s_main.cyc += (unsigned long)k * c;
					s_main.r = (uint8_t)((s_main.r & 0x80) | ((s_main.r + (unsigned long)k * ((s_main.r - hz.r) & 0x7f)) & 0x7f));
				}
				hz = s_main;   // (the next pass watched from here)
				hcyc = s_main.cyc;
				hirq = t.irq;
				pn = 0;
			}
		} else if (s_main.pc < pc0 && pc0 - s_main.pc <= 64 && !s_wq_n) {   // (a short backward jump)
			head = s_main.pc;
			hz = s_main;
			hcyc = s_main.cyc;
			hirq = t.irq;
			pn = 0;
		}
	}
}

// (seibusnd's peer: the time this CPU has really run to)
static int64_t main_real_time(void) { return g_sched.ex[0].local + (int64_t)s_q.pend * g_sched.ex[0].period; }

static int z_run(int n)
{
	long pos = 0;
	while (s_q.h != s_q.t && pos < n) {   // (the iterations run ahead, as z80_run's loop would run them)
		const unsigned v = s_q.f[s_q.h++ & (MAIN_FIFO - 1)];
		pos += (long)v;
		s_q.pend -= v;
	}
	FGP(8, 1);
	if (pos < n) {
		FGP(2, 1);
		s_zrun0 = s_main.cyc - (unsigned long)pos;
		z80_break = 0;
		z80_run(&s_main, s_main.cyc + (unsigned long)(n - pos), &t.irq, 1);
		const int brk = z80_break;
		z80_break = 0;
		pos = (long)(s_main.cyc - s_zrun0);
		if (brk)
			return (int)pos;   // (aborted: the scheduler's slice cut here)
	}
	if (s_lazy_main && s_q.h == s_q.t)
		main_ahead((int)pos);
	return (int)pos;
}
static int64_t z_done(void) { return (int64_t)(s_main.cyc - s_zrun0); }
static void z_abort(void) { z80_break = 1; }

static int64_t s_frame_as;

static void fire(int id, int32_t param)
{
	if (seibusnd_fire(id, param))
		return;
	switch (id) {
	case T_VBLANK:   // scanline 240: RST 10h
		capture();
		s_main.int_data = 0xd7;
		t.irq = 1;
		sched_timer_set(T_VBLANK, sched_now() + s_frame_as, 0);
		break;
	case T_LINE0:   // scanline 0: RST 08h
		s_main.int_data = 0xcf;
		t.irq = 1;
		sched_timer_set(T_LINE0, sched_now() + s_frame_as, 0);
		break;
	case T_SOUND:
		sched_timer_set(T_SOUND, sched_now() + SCHED_AS_PER_SEC / 50, 0);
		break;
	default: break;
	}
}

static void ar_frame(void)
{
	rbx_audio.frame_mhz = 60000;
#ifdef RBX_FG_PROF
	const uint64_t p0 = fgp_us();
	const unsigned long long i0 = rbx_fgp_ins();
#endif
	sched_run(T_VBLANK, fire);
	if (s_q.h != s_q.t || seibusnd_lazy_pending())
		rbx_log("airraid: lazy records left at the frame end");
#ifdef RBX_FG_PROF
	const uint64_t p1 = fgp_us();
#endif
	seibusnd_frame_end(s_frame_as);
	sched_rebase(s_frame_as);
#ifdef RBX_FG_PROF
	{	// (a line a frame: sched_run us, frame end us, then the counters)
		static FILE *pf;
		static int pfo = -1, fr;
		if (pfo < 0)
			pfo = getenv("RBX_FG_PROF") && (pf = fopen(getenv("RBX_FG_PROF"), "w")) != NULL;
		rbx_fgprof[15] = p1 - p0;
		rbx_fgprof[13] = fgp_us() - p1;
		rbx_fgprof[17] = rbx_fgp_ins() - i0;   // (the frame's instructions: minus [18] FM, [19] capture + drawing)
		if (pfo) {
			fprintf(pf, "%d", ++fr);
			for (int i = 0; i < 20; i++)
				fprintf(pf, " %lu", rbx_fgprof[i]);
			fputc('\n', pf);
			for (int c = 0; c < 2; c++) {   // (the frame's 12 most run PCs of each CPU)
				unsigned *h = c ? rbx_fgp_spc : rbx_fgp_mpc;
				fprintf(pf, "#%c", c ? 's' : 'm');
				for (int k = 0; k < 12; k++) {
					int best = 0;
					for (int a = 1; a < 65536; a++)
						if (h[a] > h[best])
							best = a;
					if (!h[best])
						break;
					fprintf(pf, " %04x:%u", best, h[best]);
					h[best] = 0;
				}
				fputc('\n', pf);
				memset(h, 0, sizeof(rbx_fgp_mpc));
			}
			fflush(pf);
		}
		memset(rbx_fgprof, 0, sizeof(rbx_fgprof));
	}
#endif
}

static void ar_reset(void)
{
	rbx_video_wait_idle();
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	t.hw = 0x09;
	memcpy(s_op + 0x8000, s_rom + 0x10000, 0x4000);
	z80_init(&s_main);
	s_main.read_byte = main_read;
	s_main.write_byte = main_write;
	s_main.port_in = z_in;
	s_main.port_out = z_out;
	s_main.op_rom = s_op;
	g_sched.base = 0;
	for (int i = 0; i < g_sched.nex; i++) {
		g_sched.ex[i].local = 0;
		g_sched.ex[i].suspended = 0;
	}
	for (int i = 0; i < SCHED_TIMERS; i++)
		g_sched.tm[i].id = -1;
	sched_timer_set(T_VBLANK, s_frame_as, 0);
	sched_timer_set(T_LINE0, s_frame_as / LINES * 16, 0);
	sched_timer_set(T_SOUND, SCHED_AS_PER_SEC / 50, 0);
	seibusnd_reset();
	s_q.h = s_q.t = 0;
	s_q.pend = 0;
}

static void ar_exit(void)
{
	rbx_video_wait_idle();
	seibusnd_exit();
	free(s_q.f);
	s_q.f = NULL;
	s_lazy_main = 0;
	s_spin = 0;
	rbx_gfx_free(&s_gtx);
	rbx_gfx_free(&s_gspr);
	rbx_gfx_free(&s_gbg);
	rbx_gfx_free(&s_gfg);
	free(s_snap);
	s_snap = NULL;
	free(s_temp);
	s_temp = NULL;
	free(s_data);
	s_data = NULL;
	free(s_op);
	s_op = NULL;
	free(s_rom);
	s_rom = NULL;
}

static int ar_init(const rbx_game *g)
{
	uint32_t ms, ds, ss, txs, sps, bgs, fgs, cl[4];
	const uint8_t *mrom = rbx_region("maincpu", &ms), *mdata = rbx_region("maindata", &ds);
	uint8_t *snd = rbx_region("audiocpu", &ss);
	const uint8_t *tx = rbx_region("airraid_vid:tx_gfx", &txs), *sp = rbx_region("airraid_vid:spr_gfx", &sps),
	              *bg = rbx_region("airraid_vid:bg_gfx", &bgs), *fg = rbx_region("airraid_vid:fg_gfx", &fgs);
	s_bgmap = rbx_region("airraid_vid:bg_map", &s_bgmap_size);
	s_fgmap = rbx_region("airraid_vid:fg_map", &s_fgmap_size);
	static const char *const cn[4] = { "airraid_vid:tx_clut", "airraid_vid:bg_clut", "airraid_vid:fg_clut", "airraid_vid:spr_clut" };
	const uint8_t *clut[4];
	for (int i = 0; i < 4; i++)
		clut[i] = rbx_region(cn[i], &cl[i]);
	if (!mrom || ms < 0x8000 || !mdata || ds < 0x10000 || !snd || ss < 0x10000 || !tx || !sp || !bg || !fg || !s_bgmap ||
	    !s_fgmap || !clut[0] || !clut[1] || !clut[2] || !clut[3]) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	for (int i = 0; i < 4; i++)
		memcpy(s_clut[i], clut[i], 0x100);
	s_rom = calloc(1, 0x20000);   // (the program, the data banks from 0x10000)
	s_data = calloc(1, 0x10000);
	s_op = calloc(1, 0x10000);
	s_snap = calloc(1, sizeof(*s_snap));
	s_temp = calloc(VIS_W * VIS_H, sizeof(uint16_t));
	if (!s_rom || !s_data || !s_op || !s_snap || !s_temp) {
		ar_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	memcpy(s_rom, mrom, 0x8000);
	memcpy(s_rom + 0x10000, mdata, 0x10000);
	memcpy(s_data, s_rom, 0x8000);
	memcpy(s_op, s_rom, 0x8000);
	if (!strcmp(g->machine, "airraide")) {   // init_airraide: opcodes and data decrypted apart
		static const int b8[8] = { 7, 6, 1, 4, 3, 2, 5, 0 };
		for (uint32_t a = 0; a < 0x8000; a++) {
			uint8_t o = s_rom[a], d = s_rom[a];
			if ((a & 0x20) && !(a & 0x08)) o ^= 0x40;
			if ((a & 0x400) && !(a & 0x200) && (a & 0x08)) o ^= 0x20;
			if ((((a >> 10) ^ (a >> 9)) & 1) && (a & 0x02)) o ^= 0x02;
			if ((a & 0x200) || !(a & 0x20) || (a & 0x08)) o = (uint8_t)sd_bitswap(o, 8, b8);
			if (a & 0x20) d ^= 0x40;
			if ((a & 0x200) || !(a & 0x20)) d = (uint8_t)sd_bitswap(d, 8, b8);
			s_op[a] = o;
			s_data[a] = d;
		}
	}
	{   // charlayout (2 bpp), char16layout (4 bpp)
		static const uint32_t cp[2] = { 0, 4 }, cx[8] = { 8, 9, 10, 11, 0, 1, 2, 3 }, cy[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
		static const uint32_t p16[4] = { 0, 4, 8, 12 };
		static const uint32_t x16[16] = { 0, 1, 2, 3, 16, 17, 18, 19, 512, 513, 514, 515, 528, 529, 530, 531 };
		static uint32_t y16[16];
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)i * 32;
		const rbx_gfx_layout lc = { 8, 8, 2, cp, cx, cy, 128 }, l16 = { 16, 16, 4, p16, x16, y16, 1024 };
		if (rbx_gfx_decode(&s_gtx, &lc, tx, txs) || rbx_gfx_decode(&s_gspr, &l16, sp, sps) ||
		    rbx_gfx_decode(&s_gbg, &l16, bg, bgs) || rbx_gfx_decode(&s_gfg, &l16, fg, fgs) ||
		    rbx_video_setup(VIS_W, VIS_H, 256)) {
			ar_exit();
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
	}
	static const char *const names[5] = { "IN0", "IN1", "IN2", "DSW2", "DSW1" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	s_frame_as = SCHED_AS_PER_SEC / 60;
	sched_init(SCHED_AS_PER_SEC / (CPU_CLOCK / 4));   // (set_maximum_quantum: the main CPU's clock / 4)
	(void)sched_add_exec(CPU_CLOCK, z_run, z_done, z_abort);
	seibusnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.rom = snd;
	sc.rom_size = 0x10000;
	sc.encrypted = 1;
	sc.enc_low = 1;
	sc.ym_type = RBX_YM2151;
	sc.ym_clock = 3579545;
	sc.ym_gain = 128;   // (0.50)
	sc.z80_clock = 3579545;
	sc.coin_port = rbx_port_find("COIN");
	sc.ym_async = !getenv("RBX_FG_NOASYNC");   // (the FM samples on the rbx_aux worker: the same sound)
	if (seibusnd_init(&sc)) {
		ar_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_lazy_main = !getenv("RBX_FG_TIGHT") && !getenv("RBX_FG_NOMAINAHEAD") && (s_q.f = (uint8_t *)malloc(MAIN_FIFO)) != NULL;
	s_q.mask = MAIN_FIFO - 1;
	s_lazy_snd = !getenv("RBX_FG_TIGHT") && !getenv("RBX_FG_NOSNDAHEAD");
	s_spin = !getenv("RBX_FG_NOSPIN");
	seibusnd_set_lazy(s_lazy_snd, main_real_time, s_lazy_main ? &s_q : NULL);
	ar_reset();
	return 0;
}

static int ar_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "airraid", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	z80 c = s_main;
	rc |= rbx_state_io(ctx, "airraid_main", &c, sizeof(c));
	if (!rc && load) {
		c.read_byte = main_read;
		c.write_byte = main_write;
		c.port_in = z_in;
		c.port_out = z_out;
		c.userdata = NULL;
		c.read_map = NULL;
		c.op_rom = s_op;
		s_main = c;
		s_q.h = s_q.t = 0;
		s_q.pend = 0;
		memcpy(s_op + 0x8000, s_rom + 0x10000 + (uint32_t)(t.bank & 3) * 0x4000, 0x4000);
	}
	rc |= seibusnd_state(ctx, load);
	rc |= sched_state(ctx, load);
	return rc ? -1 : 0;
}

static int ar_save(void *ctx) { return ar_state(ctx, 0); }
static int ar_load(void *ctx) { return ar_state(ctx, 1); }
static int ar_supported(const char *m) { return !strcmp(m, "airraid") || !strcmp(m, "airraide"); }

const rbx_driver rbx_drv_airraid = { "Seibu Air Raid", ar_init, ar_exit, ar_reset, ar_frame, ar_save, ar_load,
                                     ar_supported, ar_render, NULL };
