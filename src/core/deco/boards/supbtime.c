/*  Data East DE-0343 family: Super Burger Time, China Town, Tumble Pop. Ported from MAME
    src/mame/dataeast/supbtime.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail).
    68000 at 10.7 MHz (IRQ 6 at vblank until the acknowledge read), tilemap chip 56 (video/deco16ic.c; Tumble Pop's
    graphics encrypted), sprite chip 52 (video/decospr.c), the sound board (machine/deco16snd.c: HuC6280, YM2151, OKI).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../video/gfxlayout.h"
#include "../machine/decocrpt.h"
#include "../machine/deco16snd.h"
#include "../video/deco16ic.h"
#include "../video/decospr.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 10738636       // 21.477272 MHz / 2
#define FRAME_MHZ 57800          // 7 MHz / (442 x 274)
#define LINES 274                // set_raw(28 MHz / 4, 442, 0, 320, 274, 8, 248)
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 320

enum { M_SUPBTIME, M_CHINATWN, M_TUMBLEP };
static int s_m;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint16_t s_ram[0x2000];            // 16 KB work RAM
static uint16_t s_spriteram[0x400];
static uint16_t s_pal[0x400];             // xBGR_444
static uint16_t s_rowscroll[2][0x400];
static deco16ic_t s_tile;
static decospr_t s_spr;
static rbx_gfx s_gfx8, s_gfx16, s_gfxspr;
static int s_port_in, s_port_dsw, s_port_sys;
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac;
static struct {
	deco16ic_t tile;
	uint16_t rowscroll[2][0x400], spriteram[0x400], pal[0x400];
	uint32_t frame;
} *s_snap;

// ---- bus ----
static uint16_t sys_r(void)
{
	const int vbl = s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H;
	rbx_port_custom(s_port_sys, 0x0008, vbl ? 0x0008 : 0);
	return (uint16_t)rbx_port_read(s_port_sys);
}

static uint16_t io_r(uint32_t a)
{
	switch (a & 0xfffffe) {
	case 0x180000: return (uint16_t)rbx_port_read(s_port_in);
	case 0x180002: return (uint16_t)rbx_port_read(s_port_dsw);
	case 0x180008: return sys_r();
	case 0x18000a: m68k_bus_set_line(6, 0); return 0xffff;   // vblank_ack_r
	}
	return 0xffff;
}

static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x180000 && a < 0x180010) return io_r(a);
	if (a >= 0x300000 && a < 0x300010) return s_m == M_TUMBLEP ? 0xffff : deco16ic_control_r(&s_tile, (int)((a >> 1) & 7));
	return 0xffff;
}
static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}

// the 68000's CLR reads its operand first (MAME's core does; Musashi does not): Super Burger Time acknowledges the
// vblank interrupt with CLR.W $18000a, so a write there is the read's acknowledge
static void ack_w(uint32_t a)
{
	if ((a & 0xfffffe) == 0x18000a)
		m68k_bus_set_line(6, 0);
}

static void main_write16(uint32_t a, uint16_t v)
{
	a &= 0xffffff;
	ack_w(a);
	if (a >= 0x300000 && a < 0x300010) {
		deco16ic_control_w(&s_tile, (int)((a >> 1) & 7), v, 0xffff);
		return;
	}
	if ((s_m == M_SUPBTIME && a == 0x1a0000) || (s_m != M_SUPBTIME && a == 0x100000))   // the sound latch (low byte)
		deco16snd_latch_w((uint8_t)v);
}
static void main_write8(uint32_t a, uint8_t v)
{
	a &= 0xffffff;
	ack_w(a);
	if (a >= 0x300000 && a < 0x300010) {
		const int sh = a & 1 ? 0 : 8;
		deco16ic_control_w(&s_tile, (int)((a >> 1) & 7), (uint16_t)(v << sh), (uint16_t)(0xff << sh));
		return;
	}
	if ((s_m == M_SUPBTIME && a == 0x1a0001) || (s_m != M_SUPBTIME && a == 0x100001))
		deco16snd_latch_w(v);
}

// ---- video ----
static void sb_render(void)
{
	for (int i = 0; i < 0x400; i++) {   // xBGR_444
		const uint16_t p = s_snap->pal[i];
		const int r = p & 15, g = (p >> 4) & 15, b = (p >> 8) & 15;
		rbx_video.palette[i] = (uint32_t)((r * 17) << 16 | (g * 17) << 8 | (b * 17));
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	deco16ic_t *t = &s_snap->tile;
	deco16ic_update(t);
	decospr_t spr = s_spr;
	spr.flip_screen = (deco16ic_control_r(t, 0) >> 7) & 1;
	if (s_m == M_TUMBLEP) {   // the back playfield opaque, the front one, the sprites over both
		deco16ic_draw(t, 1, s_snap->rowscroll[1], bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 1, 0, 0xff);
		deco16ic_draw(t, 0, s_snap->rowscroll[0], bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
		decospr_draw(&spr, s_snap->spriteram, 0x400, bm, NULL, pitch, VIS_W, VIS_H, VIS_Y0, 1, s_snap->frame);
	} else {   // pen 768, the back playfield, the sprites, the front playfield
		for (int y = 0; y < VIS_H; y++)
			for (int x = 0; x < VIS_W; x++)
				bm[y * pitch + x] = 768;
		deco16ic_draw(t, 1, s_snap->rowscroll[1], bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
		decospr_draw(&spr, s_snap->spriteram, 0x400, bm, NULL, pitch, VIS_W, VIS_H, VIS_Y0, 1, s_snap->frame);
		deco16ic_draw(t, 0, s_snap->rowscroll[0], bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	s_snap->tile = s_tile;
	memcpy(s_snap->rowscroll, s_rowscroll, sizeof(s_rowscroll));
	memcpy(s_snap->spriteram, s_spriteram, sizeof(s_spriteram));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void sb_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_rowscroll, 0, sizeof(s_rowscroll));
	memset(s_tile.vram, 0, sizeof(s_tile.vram));
	deco16ic_reset(&s_tile);
	deco16snd_reset();
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = 0;
}

static void sb_exit(void);

static int sb_init(const rbx_game *g)
{
	uint32_t snd_size, tsize, ssize, osize;
	s_m = !strcmp(g->machine, "tumblep") ? M_TUMBLEP : !strcmp(g->machine, "chinatwn") ? M_CHINATWN : M_SUPBTIME;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *snd = rbx_region("audiocpu", &snd_size), *tiles = rbx_region("tiles", &tsize),
	        *spr = rbx_region("sprites", &ssize), *oki = rbx_region("oki", &osize);
	if (!s_rom || !snd || !tiles || !spr || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (s_m == M_TUMBLEP && deco56_decrypt_gfx(tiles, tsize)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	{	// tile_8x8_layout / tile_16x16_layout (RGN_FRAC(1,2): planes half + 8, half, 8, 0; rows of 16 bits)
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 16);
			x16[i] = (uint32_t)(256 + i);
			x16[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 16);
		const uint32_t th = tsize * 4, sh = ssize * 4;
		const uint32_t pt[4] = { th + 8, th, 8, 0 }, ps[4] = { sh + 8, sh, 8, 0 };
		const rbx_gfx_layout l8 = { 8, 8, 4, pt, x8, y8, 16 * 8 };
		const rbx_gfx_layout l16 = { 16, 16, 4, pt, x16, y16, 32 * 16 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, x16, y16, 32 * 16 };
		if (rbx_gfx_decode(&s_gfx8, &l8, tiles, tsize / 2) || rbx_gfx_decode(&s_gfx16, &l16, tiles, tsize / 2) ||
		    rbx_gfx_decode(&s_gfxspr, &ls, spr, ssize / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			sb_exit();
			return -1;
		}
	}
	memset(&s_tile, 0, sizeof(s_tile));
	for (int i = 0; i < 2; i++) {
		s_tile.tm[i].size = DECO_64x32;
		s_tile.tm[i].col_bank = i ? 0x10 : 0x00;
		s_tile.tm[i].col_mask = 0x0f;
	}
	s_tile.gfx8 = &s_gfx8;
	s_tile.gfx16 = &s_gfx16;
	s_tile.base8 = s_tile.base16 = 256;   // (gfx_supbtime: colour base 256)
	s_tile.scrolldx = s_m == M_SUPBTIME ? 0 : 1;   // (screen_update_common's use_offsets)
	memset(&s_spr, 0, sizeof(s_spr));
	s_spr.gfx = &s_gfxspr;
	s_spr.color_base = 0;
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_snap) {
		rbx_fail("%s: out of memory", g->set);
		sb_exit();
		return -1;
	}
	// sound: YM2151 3.58 MHz 0.25 on each route, OKI 1 MHz (Tumble Pop 1.07 MHz) 0.5, one speaker
	deco16snd_cfg sc;
	memset(&sc, 0, sizeof(sc));
	sc.cpu_clock = 32220000 / 4;
	sc.ym_clock = 32220000 / 9;
	sc.ym_gain[0] = sc.ym_gain[1] = 64;
	sc.oki_rom[0] = oki;
	sc.oki_size[0] = osize;
	sc.oki_clock[0] = s_m == M_TUMBLEP ? 21477272 / 20 : 32220000 / 32;
	sc.oki_gain[0] = 128;
	if (deco16snd_init(snd, snd_size, &sc, FRAME_MHZ, LINES) || rbx_video_setup(VIS_W, VIS_H, 0x400)) {
		rbx_fail("%s: out of memory", g->set);
		sb_exit();
		return -1;
	}
	s_port_in = rbx_port_find("INPUTS");
	s_port_dsw = rbx_port_find("DSW");
	s_port_sys = rbx_port_find("SYSTEM");
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, s_m == M_TUMBLEP ? 0x7ffff : 0x3ffff, s_rom, s_rom_size, 1, 0);
	const uint32_t ram = s_m == M_TUMBLEP ? 0x120000 : s_m == M_CHINATWN ? 0x1a0000 : 0x100000;
	const uint32_t spr_at = s_m == M_TUMBLEP ? 0x1a0000 : 0x120000;
	m68k_bus_map(ram, ram + 0x3fff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(spr_at, spr_at + 0xfff, s_spriteram, sizeof(s_spriteram), 1, 1);   // (0x800 bytes, mirrored in the page)
	m68k_bus_map(0x140000, 0x140fff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus_map(0x320000, 0x321fff, s_tile.vram[0], sizeof(s_tile.vram[0]), 1, 1);
	m68k_bus_map(0x322000, 0x323fff, s_tile.vram[1], sizeof(s_tile.vram[1]), 1, 1);
	m68k_bus_map(0x340000, 0x340fff, s_rowscroll[0], sizeof(s_rowscroll[0]), 1, 1);
	m68k_bus_map(0x342000, 0x342fff, s_rowscroll[1], sizeof(s_rowscroll[1]), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	sb_reset();
	return 0;
}

static void sb_exit(void)
{
	deco16snd_exit();
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
}

static void sb_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // vblank_w: IRQ 6 (ASSERT_LINE) until vblank_ack_r
			vblank_capture();
			m68k_bus_set_line(6, 1);
		}
		if (line >= VIS_Y0 && line < VIS_Y0 + VIS_H)
			deco16ic_latch_line(&s_tile, line);
		m68k_execute(rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		deco16snd_line();
	}
	deco16snd_frame_end();
	s_frame++;
}

static int sb_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr = s_cpu_frac;
	uint32_t misc[1] = { s_frame };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "rows", s_rowscroll, sizeof(s_rowscroll));
	rc |= rbx_state_io(ctx, "vram", s_tile.vram, sizeof(s_tile.vram));
	rc |= rbx_state_io(ctx, "tctl", s_tile.control, sizeof(s_tile.control));
	rc |= deco16snd_state(ctx, load);
	rc |= rbx_state_io(ctx, "frac", &fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		s_cpu_frac = fr;
		s_frame = misc[0];
		for (int i = 0; i < 8; i++)
			deco16ic_control_w(&s_tile, i, s_tile.control[i], 0xffff);
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int sb_save(void *ctx) { return sb_state(ctx, 0); }
static int sb_load(void *ctx) { return sb_state(ctx, 1); }
static int sb_supported(const char *machine)
{
	return !strcmp(machine, "supbtime") || !strcmp(machine, "chinatwn") || !strcmp(machine, "tumblep");
}

const rbx_driver rbx_drv_supbtime = { "DECO 16 Tumble Pop", sb_init, sb_exit, sb_reset, sb_frame, sb_save, sb_load,
                                      sb_supported, sb_render, NULL };
