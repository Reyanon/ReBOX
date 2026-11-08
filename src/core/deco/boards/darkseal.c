/*  Data East DE-0332-2: Dark Seal / Gate of Doom. Ported from MAME src/mame/dataeast/darkseal.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail).
    68000 at 12 MHz (IRQ 6 at vblank until its acknowledge write), two tilemap chips (55), sprite chip 52 (buffered by a
    write), the palette in two halves (red / green, blue), the sound board (machine/deco16snd.c: HuC6280 with its timer
    scaled by 2, YM2203, YM2151, two OKIs, a generic latch). The screen is 256 pixels wide; the program's bytes have
    bits 1 and 6 swapped.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../video/gfxlayout.h"
#include "../machine/deco16snd.h"
#include "../video/deco16ic.h"
#include "../video/decospr.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 12000000       // 24 MHz / 2
#define FRAME_MHZ 58000          // set_refresh_hz(58)
#define LINES 256                // set_size(32*8, 32*8)
#define VIS_Y0 8                 // set_visarea(0, 255, 8, 247)
#define VIS_H 240
#define VIS_W 256
#define VBL_LINES 8              // vblank 529 us
#define BLACK 0x800              // (palette_device::black_pen: a pen past the 2048)

static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint16_t s_ram[0x2000];            // 0x100000 (16 KB)
static uint16_t s_spriteram[0x800], s_spriteram_buf[0x400];
static uint16_t s_palram[0x800], s_palext[0x800];
static uint16_t s_rowscroll[2][0x800];    // rowscroll_1 (the first chip's), rowscroll_3 (the second's)
static deco16ic_t s_tile[2];
static decospr_t s_spr;
static rbx_gfx s_gfx8, s_gfx16a, s_gfx16b, s_gfxspr;
static int s_port_in, s_port_dsw, s_port_sys;
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac;
static struct {
	deco16ic_t tile[2];
	uint16_t rowscroll[2][0x800], spriteram[0x400], palram[0x800], palext[0x800];
	uint32_t frame;
} *s_snap;

// ---- bus ----
static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	switch (a & ~1u) {
	case 0x180000: return (uint16_t)rbx_port_read(s_port_dsw);
	case 0x180002: return (uint16_t)rbx_port_read(s_port_in);
	case 0x180004: {
		const int vbl = s_line >= VIS_Y0 + VIS_H && s_line < VIS_Y0 + VIS_H + VBL_LINES;
		rbx_port_custom(s_port_sys, 0x08, vbl ? 0x08 : 0);
		return (uint16_t)rbx_port_read(s_port_sys);
	}
	}
	return 0;
}
static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a >= 0x240000 && a < 0x240010)
		deco16ic_control_w(&s_tile[1], (int)((a >> 1) & 7), v, mask);
	else if (a >= 0x2a0000 && a < 0x2a0010)
		deco16ic_control_w(&s_tile[0], (int)((a >> 1) & 7), v, mask);
	else if (a == 0x180006)   // the sprite buffer
		memcpy(s_spriteram_buf, s_spriteram, sizeof(s_spriteram_buf));
	else if (a == 0x180008) { if (mask & 0xff) deco16snd_latch_w((uint8_t)v); }
	else if (a == 0x18000a)   // irq_ack_w
		m68k_bus_set_line(6, 0);
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static void ds_render(void)
{
	for (int i = 0; i < 0x800; i++)
		rbx_video.palette[i] = (uint32_t)(s_snap->palram[i] & 0xff) << 16 | (s_snap->palram[i] & 0xff00) |
		                       (s_snap->palext[i] & 0xff);
	rbx_video.palette[BLACK] = 0;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	deco16ic_t *t0 = &s_snap->tile[0], *t1 = &s_snap->tile[1];
	deco16ic_update(t0);
	deco16ic_update(t1);
	decospr_t spr = s_spr;
	spr.flip_screen = !((deco16ic_control_r(t1, 0) >> 7) & 1);
	for (int y = 0; y < VIS_H; y++)
		for (int x = 0; x < VIS_W; x++)
			bm[y * pitch + x] = BLACK;
	const uint16_t *r1 = s_snap->rowscroll[0], *r3 = s_snap->rowscroll[1];
	deco16ic_draw(t1, 0, r3, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
	deco16ic_draw(t1, 1, r3, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
	deco16ic_draw(t0, 0, r1, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
	decospr_draw(&spr, s_snap->spriteram, 0x400, bm, NULL, pitch, VIS_W, VIS_H, VIS_Y0, 0, s_snap->frame);
	deco16ic_draw(t0, 1, r1, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->tile, s_tile, sizeof(s_tile));
	memcpy(s_snap->rowscroll, s_rowscroll, sizeof(s_rowscroll));
	memcpy(s_snap->spriteram, s_spriteram_buf, sizeof(s_spriteram_buf));
	memcpy(s_snap->palram, s_palram, sizeof(s_palram));
	memcpy(s_snap->palext, s_palext, sizeof(s_palext));
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void ds_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_spriteram_buf, 0, sizeof(s_spriteram_buf));
	memset(s_palram, 0, sizeof(s_palram));
	memset(s_palext, 0, sizeof(s_palext));
	memset(s_rowscroll, 0, sizeof(s_rowscroll));
	for (int i = 0; i < 2; i++) {
		memset(s_tile[i].vram, 0, sizeof(s_tile[i].vram));
		deco16ic_reset(&s_tile[i]);
	}
	deco16snd_reset();
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = 0;
}

static void ds_exit(void);

static int ds_init(const rbx_game *g)
{
	uint32_t snd_size, csize, t1s, t2s, ssize, o1s, o2s;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *snd = rbx_region("audiocpu", &snd_size), *chars = rbx_region("chars", &csize),
	        *t1 = rbx_region("tiles1", &t1s), *t2 = rbx_region("tiles2", &t2s), *spr = rbx_region("sprites", &ssize),
	        *oki1 = rbx_region("oki1", &o1s), *oki2 = rbx_region("oki2", &o2s);
	if (!s_rom || !snd || !chars || !t1 || !t2 || !spr || !oki1 || !oki2) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{	// charlayout / tilelayout: planes 8, 0, half + 8, half (the halves the other way round)
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 16);
			x16[i] = (uint32_t)(256 + i);
			x16[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 16);
		const uint32_t ch = csize * 4, h1 = t1s * 4, h2 = t2s * 4, hs = ssize * 4;
		const uint32_t pc[4] = { 8, 0, ch + 8, ch }, p1[4] = { 8, 0, h1 + 8, h1 }, p2[4] = { 8, 0, h2 + 8, h2 },
		               ps[4] = { 8, 0, hs + 8, hs };
		const rbx_gfx_layout l8 = { 8, 8, 4, pc, x8, y8, 16 * 8 };
		const rbx_gfx_layout la = { 16, 16, 4, p1, x16, y16, 64 * 8 }, lb = { 16, 16, 4, p2, x16, y16, 64 * 8 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, x16, y16, 64 * 8 };
		if (rbx_gfx_decode(&s_gfx8, &l8, chars, csize / 2) || rbx_gfx_decode(&s_gfx16a, &la, t1, t1s / 2) ||
		    rbx_gfx_decode(&s_gfx16b, &lb, t2, t2s / 2) || rbx_gfx_decode(&s_gfxspr, &ls, spr, ssize / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			ds_exit();
			return -1;
		}
	}
	memset(s_tile, 0, sizeof(s_tile));
	for (int c = 0; c < 2; c++) {
		for (int i = 0; i < 2; i++) {
			s_tile[c].tm[i].size = c ? DECO_64x32 : DECO_64x64;
			s_tile[c].tm[i].col_bank = 0;
			s_tile[c].tm[i].col_mask = 0x0f;
		}
		s_tile[c].gfx8 = &s_gfx8;
	}
	s_tile[0].gfx16 = &s_gfx16a;
	s_tile[0].base16 = 768;    // (gfx_darkseal: tiles1 from 768, tiles2 from 1024)
	s_tile[1].gfx16 = &s_gfx16b;
	s_tile[1].base16 = 1024;
	memset(&s_spr, 0, sizeof(s_spr));
	s_spr.gfx = &s_gfxspr;
	s_spr.color_base = 256;
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_snap) {
		rbx_fail("%s: out of memory", g->set);
		ds_exit();
		return -1;
	}
	// sound: HuC6280 8.05 MHz (timer scale 2), YM2203 4.03 MHz 0.45, YM2151 3.58 MHz 0.55, OKI 1 MHz 1.0, OKI 2 MHz
	// 0.60, one speaker
	deco16snd_cfg sc;
	memset(&sc, 0, sizeof(sc));
	sc.cpu_clock = 32220000 / 4;
	sc.timer_scale = 2;
	sc.ym_clock = 32220000 / 9;
	sc.ym_gain[0] = sc.ym_gain[1] = 141;
	sc.ym2203_clock = 32220000 / 8;
	sc.ym2203_gain[0] = sc.ym2203_gain[1] = sc.ym2203_gain[2] = sc.ym2203_gain[3] = 115;
	sc.oki_rom[0] = oki1;
	sc.oki_size[0] = o1s;
	sc.oki_clock[0] = 32220000 / 32;
	sc.oki_gain[0] = 256;
	sc.oki_rom[1] = oki2;
	sc.oki_size[1] = o2s;
	sc.oki_clock[1] = 32220000 / 16;
	sc.oki_gain[1] = 154;
	if (deco16snd_init(snd, snd_size, &sc, FRAME_MHZ, LINES) || rbx_video_setup(VIS_W, VIS_H, 0x801)) {
		rbx_fail("%s: out of memory", g->set);
		ds_exit();
		return -1;
	}
	s_port_in = rbx_port_find("P1_P2");
	s_port_dsw = rbx_port_find("DSW");
	s_port_sys = rbx_port_find("SYSTEM");
	for (uint32_t i = 0; i < s_rom_size && i < 0x80000; i++)   // driver_init: bits 1 and 6 of each byte swapped
		s_rom[i] = (uint8_t)((s_rom[i] & 0xbd) | ((s_rom[i] & 0x02) << 5) | ((s_rom[i] & 0x40) >> 5));
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x7ffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x100000, 0x103fff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x120000, 0x120fff, s_spriteram, sizeof(s_spriteram), 1, 1);
	m68k_bus_map(0x140000, 0x140fff, s_palram, sizeof(s_palram), 1, 1);
	m68k_bus_map(0x141000, 0x141fff, s_palext, sizeof(s_palext), 1, 1);
	m68k_bus_map(0x200000, 0x201fff, s_tile[1].vram[0], sizeof(s_tile[1].vram[0]), 1, 1);
	m68k_bus_map(0x202000, 0x203fff, s_tile[1].vram[1], sizeof(s_tile[1].vram[1]), 1, 1);
	m68k_bus_map(0x220000, 0x220fff, s_rowscroll[0], sizeof(s_rowscroll[0]), 1, 1);
	m68k_bus_map(0x222000, 0x222fff, s_rowscroll[1], sizeof(s_rowscroll[1]), 1, 1);
	m68k_bus_map(0x260000, 0x261fff, s_tile[0].vram[0], sizeof(s_tile[0].vram[0]), 1, 1);
	m68k_bus_map(0x262000, 0x263fff, s_tile[0].vram[1], sizeof(s_tile[0].vram[1]), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	ds_reset();
	return 0;
}

static void ds_exit(void)
{
	deco16snd_exit();
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16a);
	rbx_gfx_free(&s_gfx16b);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
}

static void ds_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // irq6_line_assert at vblank (until irq_ack_w)
			vblank_capture();
			m68k_bus_set_line(6, 1);
		}
		if (line >= VIS_Y0 && line < VIS_Y0 + VIS_H)
			for (int i = 0; i < 2; i++)
				deco16ic_latch_line(&s_tile[i], line);
		m68k_execute(rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		deco16snd_line();
	}
	deco16snd_frame_end();
	s_frame++;
}

static int ds_state(void *ctx, int load)
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
	rc |= rbx_state_io(ctx, "sprb", s_spriteram_buf, sizeof(s_spriteram_buf));
	rc |= rbx_state_io(ctx, "pal", s_palram, sizeof(s_palram));
	rc |= rbx_state_io(ctx, "palx", s_palext, sizeof(s_palext));
	rc |= rbx_state_io(ctx, "rows", s_rowscroll, sizeof(s_rowscroll));
	for (int i = 0; i < 2; i++) {
		rc |= rbx_state_io(ctx, i ? "vram2" : "vram1", s_tile[i].vram, sizeof(s_tile[i].vram));
		rc |= rbx_state_io(ctx, i ? "tctl2" : "tctl1", s_tile[i].control, sizeof(s_tile[i].control));
	}
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
		for (int c = 0; c < 2; c++)
			for (int i = 0; i < 8; i++)
				deco16ic_control_w(&s_tile[c], i, s_tile[c].control[i], 0xffff);
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int ds_save(void *ctx) { return ds_state(ctx, 0); }
static int ds_load(void *ctx) { return ds_state(ctx, 1); }
static int ds_supported(const char *machine) { return !strcmp(machine, "darkseal"); }

const rbx_driver rbx_drv_darkseal = { "DECO 16 Dark Seal", ds_init, ds_exit, ds_reset, ds_frame, ds_save, ds_load,
                                      ds_supported, ds_render, NULL };
