/*  Mitchell Mad Motor. Ported from MAME src/mame/dataeast/madmotor.cpp (BSD-3-Clause, copyright-holders: Bryan
    McPhail).
    68000 at 12 MHz (IRQ 6 held at vblank; its program's bits swapped), three BAC06 tilemap chips, the MXC06 sprites (the live list), xBGR_444
    palette, the sound board (machine/deco16snd.c: HuC6280, YM2203, YM2151, two OKIs, the generic latch).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../video/gfxlayout.h"
#include "../machine/deco16snd.h"
#include "../video/bac06.h"
#include "../video/decmxc06.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 12000000       // 24 MHz / 2
#define FRAME_MHZ 58000          // set_refresh_hz(58)
#define LINES 256                // set_size(32*8, 32*8)
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 256
#define VBL_LINES 38             // vblank 2500 us (into the next frame)

static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint16_t s_ram[0x2000];             // 0x3e0000 .. 0x3e3fff
static uint16_t s_spriteram[0x400];
static uint16_t s_pal[0x400];              // xBGR_444
static uint16_t s_ram184080[0x1c0];        // 0x184080 .. 0x1843ff
static bac06_t s_tile[3];
static decmxc06_t s_spr;
static rbx_gfx s_gfx8, s_gfx16a, s_gfx16b, s_gfxspr;
static int s_port_in, s_port_dsw, s_port_sys;
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac;
static struct {
	bac06_t tile[3];
	uint16_t spriteram[0x400], pal[0x400];
	uint32_t frame;
} *s_snap;

// ---- bus ----
static int vblank(void) { return ((s_line - (VIS_Y0 + VIS_H) + LINES) % LINES) < VBL_LINES; }
static void wmask(uint16_t *p, uint16_t v, uint16_t mask) { *p = (uint16_t)((*p & ~mask) | (v & mask)); }

static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x184000 && a < 0x184080) return s_tile[0].colscroll[(a - 0x184000) >> 1];
	if (a >= 0x184080 && a < 0x184400) return s_ram184080[(a - 0x184080) >> 1];
	if (a >= 0x184400 && a < 0x184800) return s_tile[0].rowscroll[(a - 0x184400) >> 1];
	if (a >= 0x188000 && a < 0x18a000) return bac06_vram_r(&s_tile[0], (int)((a - 0x188000) >> 1));
	if (a >= 0x198000 && a < 0x198800) return bac06_vram_r(&s_tile[1], (int)((a - 0x198000) >> 1));
	if (a >= 0x1a4000 && a < 0x1a5000) return bac06_vram_r(&s_tile[2], (int)((a - 0x1a4000) >> 1));
	switch (a) {
	case 0x3f8002: return (uint16_t)rbx_port_read(s_port_in);
	case 0x3f8004: return (uint16_t)rbx_port_read(s_port_dsw);
	case 0x3f8006:
		rbx_port_custom(s_port_sys, 0x08, vblank() ? 0x08 : 0);
		return (uint16_t)rbx_port_read(s_port_sys);
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
	static const uint32_t base[3] = { 0x180000, 0x190000, 0x1a0000 };
	for (int i = 0; i < 3; i++) {
		if (a >= base[i] && a < base[i] + 8) { bac06_ctrl_w(&s_tile[i], (int)((a >> 1) & 3), v, mask); return; }
		if (a >= base[i] + 0x10 && a < base[i] + 0x18) { bac06_scroll_w(&s_tile[i], (int)((a >> 1) & 3), v, mask); return; }
	}
	if (a >= 0x184000 && a < 0x184080) wmask(&s_tile[0].colscroll[(a - 0x184000) >> 1], v, mask);
	else if (a >= 0x184080 && a < 0x184400) wmask(&s_ram184080[(a - 0x184080) >> 1], v, mask);
	else if (a >= 0x184400 && a < 0x184800) wmask(&s_tile[0].rowscroll[(a - 0x184400) >> 1], v, mask);
	else if (a >= 0x188000 && a < 0x18a000) bac06_vram_w(&s_tile[0], (int)((a - 0x188000) >> 1), v, mask);
	else if (a >= 0x198000 && a < 0x198800) bac06_vram_w(&s_tile[1], (int)((a - 0x198000) >> 1), v, mask);
	else if (a >= 0x1a4000 && a < 0x1a5000) bac06_vram_w(&s_tile[2], (int)((a - 0x1a4000) >> 1), v, mask);
	else if (a == 0x3fc004) { if (mask & 0xff) deco16snd_latch_w((uint8_t)v); }
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static void mm_render(void)
{
	for (int i = 0; i < 0x400; i++) {   // xBGR_444
		const uint16_t p = s_snap->pal[i];
		rbx_video.palette[i] = (uint32_t)(((p & 15) * 17) << 16 | (((p >> 4) & 15) * 17) << 8 | (((p >> 8) & 15) * 17));
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	bac06_t *t0 = &s_snap->tile[0], *t1 = &s_snap->tile[1], *t2 = &s_snap->tile[2];
	const int flip = (t0->ctrlreg[0] & 0x80) != 0;
	for (int i = 0; i < 3; i++)
		s_snap->tile[i].flip_screen = flip;
	decmxc06_t spr = s_spr;
	spr.flip_screen = flip;
	bac06_draw(t2, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, BAC06_OPAQUE, 0, 0xff);
	bac06_draw(t1, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
	decmxc06_draw(&spr, s_snap->spriteram, 0x400, bm, NULL, pitch, VIS_W, VIS_H, VIS_Y0, s_snap->frame);
	bac06_draw(t0, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->tile, s_tile, sizeof(s_tile));
	memcpy(s_snap->spriteram, s_spriteram, sizeof(s_spriteram));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void mm_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_ram184080, 0, sizeof(s_ram184080));
	for (int i = 0; i < 3; i++) {
		memset(s_tile[i].vram, 0, sizeof(s_tile[i].vram));
		memset(s_tile[i].rowscroll, 0, sizeof(s_tile[i].rowscroll));
		memset(s_tile[i].colscroll, 0, sizeof(s_tile[i].colscroll));
		bac06_reset(&s_tile[i]);
	}
	deco16snd_reset();
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = 0;
}

static void mm_exit(void);

static int mm_init(const rbx_game *g)
{
	uint32_t snd_size, csize, t1s, t2s, ssize, o1s, o2s;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *snd = rbx_region("audiocpu", &snd_size), *chars = rbx_region("char", &csize),
	        *t1 = rbx_region("tiles1", &t1s), *t2 = rbx_region("tiles2", &t2s), *spr = rbx_region("sprites", &ssize),
	        *oki1 = rbx_region("oki1", &o1s), *oki2 = rbx_region("oki2", &o2s);
	if (!s_rom || !snd || !chars || !t1 || !t2 || !spr || !oki1 || !oki2) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{	// charlayout / tilelayout (RGN_FRAC(1,2): planes half + 8, half, 8, 0)
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 16);
			x16[i] = (uint32_t)(256 + i);
			x16[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 16);
		const uint32_t hc = csize * 4, h1 = t1s * 4, h2 = t2s * 4, hs = ssize * 4;
		const uint32_t pc[4] = { hc + 8, hc, 8, 0 }, p1[4] = { h1 + 8, h1, 8, 0 }, p2[4] = { h2 + 8, h2, 8, 0 },
		               ps[4] = { hs + 8, hs, 8, 0 };
		const rbx_gfx_layout l8 = { 8, 8, 4, pc, x8, y8, 16 * 8 };
		const rbx_gfx_layout la = { 16, 16, 4, p1, x16, y16, 32 * 16 }, lb = { 16, 16, 4, p2, x16, y16, 32 * 16 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, x16, y16, 32 * 16 };
		if (rbx_gfx_decode(&s_gfx8, &l8, chars, csize / 2) || rbx_gfx_decode(&s_gfx16a, &la, t1, t1s / 2) ||
		    rbx_gfx_decode(&s_gfx16b, &lb, t2, t2s / 2) || rbx_gfx_decode(&s_gfxspr, &ls, spr, ssize / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			mm_exit();
			return -1;
		}
	}
	memset(s_tile, 0, sizeof(s_tile));
	for (int i = 0; i < 3; i++)
		s_tile[i].gfx8 = &s_gfx8;
	s_tile[1].gfx16 = &s_gfx16a;
	s_tile[1].base16 = 512;
	s_tile[2].gfx16 = &s_gfx16b;
	s_tile[2].base16 = 768;
	s_tile[2].wide = 1;
	memset(&s_spr, 0, sizeof(s_spr));
	s_spr.gfx = &s_gfxspr;
	s_spr.color_base = 256;
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_snap) {
		rbx_fail("%s: out of memory", g->set);
		mm_exit();
		return -1;
	}
	// sound: HuC6280 4.03 MHz, YM2203 3.58 MHz 0.40, YM2151 3.58 MHz 0.45, OKI 1.02 MHz 0.50, OKI 2.05 MHz 0.25
	deco16snd_cfg sc;
	memset(&sc, 0, sizeof(sc));
	sc.cpu_clock = 8053000 / 2;
	sc.ym_clock = 21477272 / 6;
	sc.ym_gain[0] = sc.ym_gain[1] = 115;
	sc.ym2203_clock = 21477272 / 6;
	sc.ym2203_gain[0] = sc.ym2203_gain[1] = sc.ym2203_gain[2] = sc.ym2203_gain[3] = 102;
	sc.oki_rom[0] = oki1;
	sc.oki_size[0] = o1s;
	sc.oki_clock[0] = 1023924;
	sc.oki_gain[0] = 128;
	sc.oki_rom[1] = oki2;
	sc.oki_size[1] = o2s;
	sc.oki_clock[1] = 2047848;
	sc.oki_gain[1] = 64;
	if (deco16snd_init(snd, snd_size, &sc, FRAME_MHZ, LINES) || rbx_video_setup(VIS_W, VIS_H, 0x400)) {
		rbx_fail("%s: out of memory", g->set);
		mm_exit();
		return -1;
	}
	s_port_in = rbx_port_find("P1_P2");
	s_port_dsw = rbx_port_find("DSW");
	s_port_sys = rbx_port_find("SYSTEM");
	for (uint32_t i = 0; i < s_rom_size && i < 0x80000; i++) {   // init_madmotor: bits 2 / 5 and 0 / 7 swapped
		uint8_t b = s_rom[i];
		b = (uint8_t)((b & 0xdb) | ((b & 0x04) << 3) | ((b & 0x20) >> 3));
		s_rom[i] = (uint8_t)((b & 0x7e) | ((b & 0x01) << 7) | ((b & 0x80) >> 7));
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x7ffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x3e0000, 0x3e3fff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x3e8000, 0x3e87ff, s_spriteram, sizeof(s_spriteram), 1, 1);
	m68k_bus_map(0x3f0000, 0x3f07ff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	mm_reset();
	return 0;
}

static void mm_exit(void)
{
	deco16snd_exit();
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16a);
	rbx_gfx_free(&s_gfx16b);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
}

static void mm_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // irq6_line_hold at vblank
			vblank_capture();
			m68k_bus_hold_irq(6);
		}
		m68k_execute(rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		deco16snd_line();
	}
	deco16snd_frame_end();
	s_frame++;
}

static int mm_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr = s_cpu_frac;
	uint32_t misc[2] = { s_frame, 0 };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "r1840", s_ram184080, sizeof(s_ram184080));
	for (int i = 0; i < 3; i++) {
		static const char *const names[3] = { "bac0", "bac1", "bac2" };
		rc |= rbx_state_io(ctx, names[i], &s_tile[i], offsetof(bac06_t, gfx8));
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
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int mm_save(void *ctx) { return mm_state(ctx, 0); }
static int mm_load(void *ctx) { return mm_state(ctx, 1); }
static int mm_supported(const char *machine) { return !strcmp(machine, "madmotor"); }

const rbx_driver rbx_drv_madmotor = { "DECO Mad Motor", mm_init, mm_exit, mm_reset, mm_frame, mm_save, mm_load,
                                      mm_supported, mm_render, NULL };
