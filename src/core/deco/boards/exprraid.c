/*  Data East Express Raider / Western Express, from MAME src/mame/dataeast/exprraid.cpp (BSD-3-Clause,
    copyright-holders: Ernesto Corvi).
    A DECO CPU-16 at 1.5 MHz (its IRQ on a coin, cleared by a write; the vblank on its I/O port 1), the DECO 291
    protection (a counter), a 512x512 background read from a tile map ROM by four quadrant registers (two x scrolls,
    one for each half; its tiles of attribute bit 7 over the sprites), 16x16 / 16x32 sprites, a 2 bpp text layer, the
    PROM palette. The sound: a 6809 at 1.5 MHz with YM2203 and YM3526 (on its IRQ), the latch on its NMI. The
    bootlegs: a plain 6502 with IN0 in memory and the coins on its NMI (Western Express bootleg 1: the DECO CPU-16 with
    the 6502's vectors).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../cheat_cpu.h"
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../cpu/m6809/m6809.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 1500000        // 12 MHz / 8
#define SND_CLOCK 1500000        // MC68B09P at 12 MHz / 2, internal / 4
#define YM1_CLOCK 1500000        // YM2203
#define YM2_CLOCK 3000000        // YM3526
#define FRAME_MHZ 59637          // set_raw(12 MHz / 2, 384, 0, 256, 262, 8, 248)
#define LINES 262
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 256

enum { V_DECO16, V_B2, V_B3 };   // (the original; wexpressb2: IN0 at 0x3800; wexpressb3 / exprraidbl: at 0xffc0)
static int s_var;
static uint8_t *s_rom, *s_map;
static uint32_t s_rom_size, s_map_size;
static uint8_t s_ram[0x1000];              // 0x0000 .. 0x0fff (RAM, sprites, text, its colours)
static uint8_t s_bg_index[4], s_scrollx[2], s_scrolly, s_prot_value;
static int s_flip, s_irq, s_coin_prev;
static m6502_t s_cpu;
static rbx_gfx s_gfx8, s_gfxspr, s_gfxt[8];
static uint32_t s_palette[256];
static int s_port_in[3], s_port_dsw[2];
static int s_line;
static uint64_t s_cpu_frac, s_snd_frac, s_ym1_frac, s_ym2_frac;
static struct {
	uint8_t ram[0x1000];
	uint8_t bg_index[4], scrollx[2], scrolly;
	int flip;
} *s_snap;

#define SPR 0x0600   // (s_ram offsets)
#define TXT 0x0800
#define TXT_COL 0x0c00

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
	if (a == 0x2000 || a == 0x2001) return rbx_ym_read(0, a & 1);
	if (a == 0x4000 || a == 0x4001) return rbx_ym_read(1, a & 1);
	if (a == 0x6000) {   // (reading the latch clears its NMI)
		m6809_set_nmi(&s_snd, 0);
		return s_latch;
	}
	return 0;
}
static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a == 0x2000 || a == 0x2001) rbx_ym_write(0, a & 1, v);
	else if (a == 0x4000 || a == 0x4001) rbx_ym_write(1, a & 1, v);
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
static uint8_t in0_r(void)   // (IN0 bit 1: the vblank)
{
	rbx_port_custom(s_port_in[0], 0x02, vblank() ? 0x02 : 0);
	return (uint8_t)rbx_port_read(s_port_in[0]);
}
static void set_irq(int state)
{
	s_irq = state;
	m6502_set_irq(&s_cpu, state);
}
static uint8_t cpu_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if ((s_var == V_B2 && a == 0x3800) || (s_var == V_B3 && a == 0xffc0))
		return in0_r();
	if (a >= 0x4000) return s_rom[a];   // (wexpressb3: the page with IN0)
	switch (a) {
	case 0x1800: return (uint8_t)rbx_port_read(s_port_dsw[0]);
	case 0x1801: return (uint8_t)rbx_port_read(s_port_in[1]);
	case 0x1802: return (uint8_t)rbx_port_read(s_port_in[2]);
	case 0x1803: return (uint8_t)rbx_port_read(s_port_dsw[1]);
	case 0x2800: return s_prot_value;   // prot_data_r
	case 0x2801: return 0x02;           // prot_status_r: the device's data available
	}
	return 0;
}
static void cpu_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	switch (a) {
	case 0x2000: set_irq(0); return;   // int_clear_w
	case 0x2001: s_latch = v; m6809_set_nmi(&s_snd, 1); return;
	case 0x2002: s_flip = v & 1; return;
	case 0x2800: case 0x2801: case 0x2802: case 0x2803: s_bg_index[a & 3] = v; return;
	case 0x2804: s_scrolly = v; return;
	case 0x2805: case 0x2806: s_scrollx[a - 0x2805] = v; return;
	case 0x2807:   // prot_data_w (DECO 291): 0x80 counts, 0x90 clears
		if (v == 0x80) s_prot_value++;
		else if (v == 0x90) s_prot_value = 0;
		return;
	}
}
static uint8_t io_r(void *ctx, uint8_t a)   // master_io_map (0x01: IN0; 0x00 open)
{
	(void)ctx;
	return a == 1 ? in0_r() : 0;
}
static void coin_update(void)
{
	const int c = (int)(rbx_port_read(s_port_in[2]) & 0xc0) | (int)(rbx_port_read(s_port_dsw[1]) & 0x40) >> 1;
	if (s_var == V_DECO16) {   // coin_inserted_deco16: a press raises the IRQ
		if (s_coin_prev & ~c & 0xe0)
			set_irq(1);
	} else   // coin_inserted_nmi: the NMI held while a coin is in (the service on the IRQ as the original)
		m6502_set_nmi(&s_cpu, (c & 0xc0) != 0xc0);
	if (s_var != V_DECO16 && (s_coin_prev & ~c & 0x20))
		set_irq(1);
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
				bm[dy * pitch + dx] = (uint16_t)(64 + colour * 8 + p);
		}
	}
}

static void er_render(void)
{
	memcpy(rbx_video.palette, s_palette, sizeof(s_palette));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, flip = s_snap->flip;
	const uint8_t *ram = s_snap->ram;
	// the background (32x32 of 16x16 from the tile map ROM, by quadrants; x scrolled by halves of 256 rows),
	// drawn whole, then its tiles of attribute bit 7 again over the sprites
	for (int pass = 0; pass < 2; pass++) {
		if (pass == 1)
			for (int offs = 0; offs < 0x200; offs += 4) {   // draw_sprites (in the list's order)
				const uint8_t *s = ram + SPR + offs;
				const int attr = s[1];
				const uint32_t code = (uint32_t)(s[3] + ((attr & 0xe0) << 3));
				const int colour = (attr & 0x03) + ((attr & 0x08) >> 1);
				int fx = (attr & 0x04) != 0, fy = 0;
				int sx = ((248 - s[2]) & 0xff) - 8, sy = s[0];
				if (flip) {
					sx = 240 - sx;
					sy = 240 - sy;
					fx = !fx;
					fy = 1;
				}
				spr_blit(code, colour, fx, fy, sx, sy, bm, pitch);
				if (attr & 0x10)   // double height
					spr_blit(code + 1, colour, fx, fy, sx, sy + (flip ? -16 : 16), bm, pitch);
			}
		for (int y = 0; y < VIS_H; y++) {
			uint16_t *d = bm + y * pitch;
			const int py0 = (y + VIS_Y0 + s_snap->scrolly) & 0x1ff, py = flip ? 511 - py0 : py0;
			const int sx0 = s_snap->scrollx[py0 >> 8], syq = py >> 4;
			for (int x = 0; x < VIS_W;) {   // (a run of pixels in one tile: its lookup once)
				const int pu = (x + sx0) & 0x1ff, px = flip ? 511 - pu : pu;
				int n = 16 - (pu & 15);
				if (n > VIS_W - x)
					n = VIS_W - x;
				const int sxq = px >> 4;
				const int quadrant = (sxq >= 16) + (syq >= 16) * 2;
				const int offs = (syq % 16) * 16 + (sxq % 16) + (s_snap->bg_index[quadrant] & 0x3f) * 0x100;
				const int data = s_map[offs], attr = s_map[offs + 0x4000];
				if (pass == 1 && !(attr & 0x80)) {
					x += n;
					continue;
				}
				const rbx_gfx *g = &s_gfxt[2 * (attr & 3) + ((data & 0x80) >> 7)];
				const uint32_t code = rbx_wrap((uint32_t)(data & 0x7f), g->count);
				const uint8_t *row = g->pix + (code * 16 + (uint32_t)(py & 15)) * 16;
				const uint16_t base = (uint16_t)(((attr & 0x18) >> 3) * 8);
				int tx = px & 15, dir = flip ? -1 : 1;
				if (attr & 4) {
					tx = 15 - tx;
					dir = -dir;
				}
				for (int i = 0; i < n; i++, x++, tx += dir)
					d[x] = (uint16_t)(base + row[tx]);
			}
		}
	}
	for (int y = 0; y < VIS_H; y++) {   // the text (32x32 of 8x8, 2 bpp from 128, pen 0 clear)
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < VIS_W; x++) {
			int px = x, py = y + VIS_Y0;
			if (flip) {
				px = 255 - px;
				py = 255 - py;
			}
			const int idx = (py >> 3) * 32 + (px >> 3), attr = ram[TXT_COL + idx];
			const uint32_t code = rbx_wrap((uint32_t)(ram[TXT + idx] + ((attr & 7) << 8)), s_gfx8.count);
			const uint8_t p = s_gfx8.pix[(code * 8 + (uint32_t)(py & 7)) * 8 + (uint32_t)(px & 7)];
			if (p)
				d[x] = (uint16_t)(128 + ((attr & 0x10) >> 4) * 4 + p);
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->ram, s_ram, sizeof(s_ram));
	memcpy(s_snap->bg_index, s_bg_index, sizeof(s_bg_index));
	memcpy(s_snap->scrollx, s_scrollx, sizeof(s_scrollx));
	s_snap->scrolly = s_scrolly;
	s_snap->flip = s_flip;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void er_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_bg_index, 0, sizeof(s_bg_index));
	memset(s_scrollx, 0, sizeof(s_scrollx));
	s_scrolly = s_prot_value = 0;
	s_flip = 0;
	s_coin_prev = 0xe0;
	m6502_reset(&s_cpu);
	set_irq(0);
	m6502_set_nmi(&s_cpu, 0);
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

static void er_exit(void);

static int er_init(const rbx_game *g)
{
	rbx_cheat_prefer("m6502");   // (the cheats' main CPU: another CPU is set up first)
	s_var = !strcmp(g->machine, "wexpressb2") ? V_B2 : !strcmp(g->machine, "wexpressb3") ? V_B3 : V_DECO16;
	uint32_t csize, tsize, ssize, psize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("slave", &s_zrom_size);
	s_map = rbx_region("bgtilemap", &s_map_size);
	uint8_t *chars = rbx_region("chars", &csize), *tiles = rbx_region("bgtiles", &tsize), *spr = rbx_region("sprites", &ssize),
	        *prom = rbx_region("proms", &psize);
	if (!s_rom || s_rom_size < 0x10000 || !s_zrom || s_zrom_size < 0x10000 || !s_map || s_map_size < 0x8000 || !chars ||
	    csize < 0x4000 || !tiles || tsize < 0x20000 || !spr || ssize < 0x30000 || !prom || psize < 0x300) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (!strcmp(g->set, "wexpressb1")) {   // init_wexpressb: the 6502 vectors where the DECO CPU-16 reads them
		s_rom[0xfff7] = s_rom[0xfffa];
		s_rom[0xfff6] = s_rom[0xfffb];
		s_rom[0xfff1] = s_rom[0xfffc];
		s_rom[0xfff0] = s_rom[0xfffd];
		s_rom[0xfff3] = s_rom[0xfffe];
		s_rom[0xfff2] = s_rom[0xffff];
	}
	for (int i = 0x8000 - 0x1000, offs = 0x10000 - 0x1000; i >= 0; i -= 0x1000) {   // gfx_expand (each 4 KB twice)
		memcpy(tiles + offs, tiles + i, 0x1000);
		offs -= 0x1000;
		memcpy(tiles + offs, tiles + i, 0x1000);
		offs -= 0x1000;
	}
	{	// charlayout (1024, 2 bpp in the nibbles, the left half 0x2000 on), spritelayout (2048, 3 bpp), tile1 / tile2
		// (128 each, 3 bpp, at 0x0000 .. 0xc000)
		static uint32_t xc[8], y8[8], xs[16], y16[16], xt[16], yt[16];
		for (int i = 0; i < 4; i++) {
			xc[i] = (uint32_t)(0x2000 * 8 + i);
			xc[4 + i] = (uint32_t)i;
			xt[i] = (uint32_t)i;
			xt[4 + i] = (uint32_t)(1024 * 32 * 2 + i);
			xt[8 + i] = (uint32_t)(128 + i);
			xt[12 + i] = (uint32_t)(128 + 1024 * 32 * 2 + i);
		}
		for (int i = 0; i < 8; i++) {
			y8[i] = (uint32_t)(i * 8);
			xs[i] = (uint32_t)(128 + i);
			xs[8 + i] = (uint32_t)i;
			yt[i] = (uint32_t)(i * 8);
			yt[8 + i] = (uint32_t)(64 + i * 8);
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 8);
		static const uint32_t pc[2] = { 0, 4 }, ps[3] = { 2 * 2048 * 32 * 8, 2048 * 32 * 8, 0 };
		static const uint32_t p1[3] = { 4, 0x10000 * 8, 0x10000 * 8 + 4 }, p2[3] = { 0, 0x11000 * 8, 0x11000 * 8 + 4 };
		const rbx_gfx_layout l8 = { 8, 8, 2, pc, xc, y8, 8 * 8 };
		const rbx_gfx_layout ls = { 16, 16, 3, ps, xs, y16, 32 * 8 };
		const rbx_gfx_layout l1 = { 16, 16, 3, p1, xt, yt, 32 * 8 }, l2 = { 16, 16, 3, p2, xt, yt, 32 * 8 };
		int rc = rbx_gfx_decode(&s_gfx8, &l8, chars, 1024 * 8) || rbx_gfx_decode(&s_gfxspr, &ls, spr, 2048 * 32);
		for (int k = 0; k < 8; k++)
			rc = rc || rbx_gfx_decode(&s_gfxt[k], (k & 1) ? &l2 : &l1, tiles + 0x4000 * (k >> 1), 128 * 32);
		if (rc) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			er_exit();
			return -1;
		}
	}
	for (int i = 0; i < 256; i++)   // RGB_444_PROMS
		s_palette[i] = (uint32_t)(((prom[i] & 15) * 17) << 16 | ((prom[256 + i] & 15) * 17) << 8 | (prom[512 + i] & 15) * 17);
	s_snap = calloc(1, sizeof(*s_snap));
	// sound: YM2203 1.5 MHz 0.30, YM3526 3 MHz 0.60
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM1_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 77;
	int rc = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM3526;
	yc.clock = YM2_CLOCK;
	yc.gain[0] = 154;
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
		er_exit();
		return -1;
	}
	m6809_init(&s_snd);
	for (int p = 0; p < 0x20; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = s_sram + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		s_snd.rpage[p] = s_zrom + p * 256;
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	m6502_init(&s_cpu, s_var == V_DECO16 ? M6502_DECO16 : M6502_NMOS);
	for (int p = 0; p < 0x10; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = s_ram + p * 256;
	for (int p = 0x40; p < 0x100; p++)
		if (!(s_var == V_B2 && p == 0x38) && !(s_var == V_B3 && p == 0xff))
			s_cpu.rpage[p] = s_rom + p * 256;
	s_cpu.read = cpu_r;
	s_cpu.write = cpu_w;
	s_cpu.io_read = io_r;
	s_port_in[0] = rbx_port_find("IN0");
	s_port_in[1] = rbx_port_find("IN1");
	s_port_in[2] = rbx_port_find("IN2");
	s_port_dsw[0] = rbx_port_find("DSW0");
	s_port_dsw[1] = rbx_port_find("DSW1");
	er_reset();
	return 0;
}

static void er_exit(void)
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

static void er_frame(void)
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
		m6502_run_until(&s_cpu, s_cpu.cycles + cyc);
		sound_line();
	}
	sound_frame_end();
}

static int er_state(void *ctx, int load)
{
	uint8_t cpu[M6502_STATE_SIZE], snd[M6809_STATE_SIZE];
	if (!load) {
		memcpy(cpu, &s_cpu, M6502_STATE_SIZE);
		memcpy(snd, &s_snd, M6809_STATE_SIZE);
	}
	uint64_t fr[4] = { s_cpu_frac, s_snd_frac, s_ym1_frac, s_ym2_frac };
	uint8_t regs[8];
	memcpy(regs, s_bg_index, 4);
	regs[4] = s_scrollx[0];
	regs[5] = s_scrollx[1];
	regs[6] = s_scrolly;
	regs[7] = s_prot_value;
	int32_t misc[4] = { s_flip, s_irq, s_coin_prev, s_latch };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m6502", cpu, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "m6809", snd, M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "regs", regs, sizeof(regs));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		memcpy(&s_cpu, cpu, M6502_STATE_SIZE);
		memcpy(&s_snd, snd, M6809_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_ym1_frac = fr[2];
		s_ym2_frac = fr[3];
		memcpy(s_bg_index, regs, 4);
		s_scrollx[0] = regs[4];
		s_scrollx[1] = regs[5];
		s_scrolly = regs[6];
		s_prot_value = regs[7];
		s_flip = misc[0];
		s_irq = misc[1];
		s_coin_prev = misc[2];
		s_latch = (uint8_t)misc[3];
	}
	return rc ? -1 : 0;
}

static int er_save(void *ctx) { return er_state(ctx, 0); }
static int er_load(void *ctx) { return er_state(ctx, 1); }
static int er_supported(const char *machine)
{
	return !strcmp(machine, "exprraid") || !strcmp(machine, "wexpressb2") || !strcmp(machine, "wexpressb3");
}

const rbx_driver rbx_drv_exprraid = { "DECO Express Raider", er_init, er_exit, er_reset, er_frame, er_save, er_load,
                                      er_supported, er_render, NULL };
