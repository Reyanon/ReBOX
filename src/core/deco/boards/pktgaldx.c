/*  Data East Pocket Gal Deluxe. Ported from MAME src/mame/dataeast/pktgaldx.cpp (BSD-3-Clause, copyright-holders:
    David Haywood). The 102 encrypted 68000 at 14 MHz (IRQ 6 from vblank until its acknowledge write), the 104
    protection / I/O chip, tilemap chip 56 (encrypted graphics), sprite chip 52 (flipped relative to the tilemaps), two
    OKI M6295 on the main CPU (the 2 MHz one banked). The bootleg (its own video) is not here.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../sound/okim6295.h"
#include "../../video/gfxlayout.h"
#include "../machine/decocrpt.h"
#include "../machine/deco102.h"
#include "../machine/deco146.h"
#include "../video/deco16ic.h"
#include "../video/decospr.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 14000000       // 28 MHz / 2
#define FRAME_MHZ 58000          // set_refresh_hz(58)
#define LINES 256                // set_size(40*8, 32*8)
#define VIS_Y0 8                 // set_visarea(0, 319, 8, 247)
#define VIS_H 240
#define VIS_W 320
#define VBL_LINES 37             // vblank 2500 us

static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint16_t *s_opcodes;
static uint16_t s_ram[0x8000];            // 0x170000 (64 KB)
static uint16_t s_spriteram[0x800];       // (0x800 bytes used; the page)
static uint16_t s_pal[0x800];             // xBGR_888 in word pairs
static uint16_t s_rowscroll[2][0x800];
static deco16ic_t s_tile;
static decospr_t s_spr;
static deco146_t s_prot;
static rbx_gfx s_gfx8, s_gfx16, s_gfxspr;
static okim6295_t s_oki[2];
static int s_port_in, s_port_dsw, s_port_sys;
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac, s_snd_frac;
static uint32_t s_out_rate, s_sfx_rate, s_sfx_phase;
static int32_t s_sfx_last;
static struct {
	deco16ic_t tile;
	uint16_t rowscroll[2][0x800], spriteram[0x400], pal[0x800];
	uint32_t frame;
} *s_snap;

// ---- the 104's ports ----
static uint16_t port_a(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_in); }
static uint16_t port_b(void *u)
{
	(void)u;
	const int vbl = s_line >= VIS_Y0 + VIS_H && s_line < VIS_Y0 + VIS_H + VBL_LINES;
	rbx_port_custom(s_port_sys, 0x0008, vbl ? 0x0008 : 0);
	return (uint16_t)rbx_port_read(s_port_sys);
}
static uint16_t port_c(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_dsw); }

// ---- bus ----
static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x167800 && a < 0x168000) return deco146_read(&s_prot, (uint16_t)(a - 0x167800), NULL);
	if ((a & ~1u) == 0x140006) return 0xff00 | oki_read(&s_oki[0]);
	if ((a & ~1u) == 0x150006) return 0xff00 | oki_read(&s_oki[1]);
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
	if (a >= 0x167800 && a < 0x168000)
		deco146_write(&s_prot, (uint16_t)(a - 0x167800), v, mask, NULL);
	else if (a >= 0x161800 && a < 0x161810)
		deco16ic_control_w(&s_tile, (int)((a >> 1) & 7), v, mask);
	else if (a >= 0x140000 && a < 0x140010) { if (mask & 0xff) oki_write(&s_oki[0], (uint8_t)v); }
	else if (a >= 0x150000 && a < 0x150010) { if (mask & 0xff) oki_write(&s_oki[1], (uint8_t)v); }
	else if (a == 0x164800) s_oki[1].bank = (uint32_t)(v & 3) * 0x40000u;   // oki_bank_w
	else if (a == 0x166800) m68k_bus_set_line(6, 0);                          // vblank_ack_w
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static int bank_cb(int bank) { return (bank & 0x70) << 8; }

static void pg_render(void)
{
	for (int i = 0; i < 0x400; i++) {   // xBGR_888 (the entry's two words, big-endian)
		const uint32_t p = (uint32_t)s_snap->pal[2 * i] << 16 | s_snap->pal[2 * i + 1];
		rbx_video.palette[i] = (p & 0xff) << 16 | (p & 0xff00) | ((p >> 16) & 0xff);
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	deco16ic_t *t = &s_snap->tile;
	deco16ic_update(t);
	decospr_t spr = s_spr;
	spr.flip_screen = !((deco16ic_control_r(t, 0) >> 7) & 1);   // (sprites are flipped relative to the tilemaps)
	for (int y = 0; y < VIS_H; y++)
		memset(bm + y * pitch, 0, VIS_W * sizeof(uint16_t));
	deco16ic_draw(t, 1, s_snap->rowscroll[1], bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
	decospr_draw(&spr, s_snap->spriteram, 0x400, bm, NULL, pitch, VIS_W, VIS_H, VIS_Y0, 1, s_snap->frame);
	deco16ic_draw(t, 0, s_snap->rowscroll[0], bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	s_snap->tile = s_tile;
	memcpy(s_snap->rowscroll, s_rowscroll, sizeof(s_rowscroll));
	memcpy(s_snap->spriteram, s_spriteram, sizeof(s_snap->spriteram));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- sound: the 2 MHz OKI's rate out, the 1 MHz one's samples held ----
static void sound_line(void)
{
	s_snd_frac += (uint64_t)s_out_rate * 1000;
	const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
	while (s_snd_frac >= den) {
		s_snd_frac -= den;
		s_sfx_phase += s_sfx_rate;
		if (s_sfx_phase >= s_out_rate) {
			s_sfx_phase -= s_out_rate;
			s_sfx_last = oki_sample(&s_oki[0]);
		}
		rbx_audio_put(s_sfx_last + oki_sample(&s_oki[1]));
	}
}

// ---- init / frame ----
static void pg_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_rowscroll, 0, sizeof(s_rowscroll));
	memset(s_tile.vram, 0, sizeof(s_tile.vram));
	deco16ic_reset(&s_tile);
	deco146_reset(&s_prot);
	for (int i = 0; i < 2; i++) {
		oki_reset(&s_oki[i]);
		s_oki[i].bank = 0;
	}
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = s_snd_frac = 0;
	s_sfx_phase = 0;
	s_sfx_last = 0;
}

static void pg_exit(void);

static int pg_init(const rbx_game *g)
{
	uint32_t tsize, ssize, o1s, o2s;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *tiles = rbx_region("tiles", &tsize), *spr = rbx_region("sprites", &ssize);
	uint8_t *oki1 = rbx_region("oki1", &o1s), *oki2 = rbx_region("oki2", &o2s);
	if (!s_rom || s_rom_size < 0x80000 || !tiles || !spr || !oki1 || !oki2) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	if (!(s_opcodes = (uint16_t *)malloc(0x80000)) || deco56_decrypt_gfx(tiles, tsize) ||
	    deco102_decrypt_cpu((uint16_t *)s_rom, s_opcodes, 0x80000, 0x42ba, 0x00, 0x00)) {
		rbx_fail("%s: out of memory", g->set);
		pg_exit();
		return -1;
	}
	{	// tile_8x8_layout / tile_16x16_layout (RGN_FRAC(1,2)), as supbtime's
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
			pg_exit();
			return -1;
		}
	}
	memset(&s_tile, 0, sizeof(s_tile));
	for (int i = 0; i < 2; i++) {
		s_tile.tm[i].size = DECO_64x32;
		s_tile.tm[i].col_bank = i ? 0x10 : 0x00;
		s_tile.tm[i].col_mask = 0x0f;
		s_tile.tm[i].bank_cb = bank_cb;
	}
	s_tile.gfx8 = &s_gfx8;
	s_tile.gfx16 = &s_gfx16;
	memset(&s_spr, 0, sizeof(s_spr));
	s_spr.gfx = &s_gfxspr;
	s_spr.color_base = 512;
	deco146_init(&s_prot, 1);
	static const uint8_t scramble[10] = { 8, 9, 4, 5, 6, 7, 1, 0, 3, 2 };   // set_interface_scramble(8,9, 4,5,6,7, 1,0,3,2)
	deco146_set_scramble(&s_prot, scramble);
	s_prot.port_a = port_a;
	s_prot.port_b = port_b;
	s_prot.port_c = port_c;
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_snap) {
		rbx_fail("%s: out of memory", g->set);
		pg_exit();
		return -1;
	}
	// sound: OKI 1 MHz 0.75, OKI 2 MHz 0.60 (banked), one speaker
	oki_init(&s_oki[0], 32220000 / 32, 1, oki1, o1s, 192);
	oki_init(&s_oki[1], 32220000 / 16, 1, oki2, o2s, 154);
	s_out_rate = s_oki[1].rate;
	s_sfx_rate = s_oki[0].rate;
	if (rbx_audio_setup(s_out_rate) || rbx_video_setup(VIS_W, VIS_H, 0x400)) {
		pg_exit();
		return -1;
	}
	s_port_in = rbx_port_find("INPUTS");
	s_port_dsw = rbx_port_find("DSW");
	s_port_sys = rbx_port_find("SYSTEM");
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x7ffff, s_rom, 0x80000, 1, 0);
	m68k_bus.op = (const uint8_t *)s_opcodes;
	m68k_bus.op_size = 0x80000;
	m68k_bus_map(0x100000, 0x100fff, s_tile.vram[0], sizeof(s_tile.vram[0]), 1, 1);
	m68k_bus_map(0x102000, 0x102fff, s_tile.vram[1], sizeof(s_tile.vram[1]), 1, 1);
	m68k_bus_map(0x110000, 0x110fff, s_rowscroll[0], sizeof(s_rowscroll[0]), 1, 1);
	m68k_bus_map(0x112000, 0x112fff, s_rowscroll[1], sizeof(s_rowscroll[1]), 1, 1);
	m68k_bus_map(0x120000, 0x120fff, s_spriteram, sizeof(s_spriteram), 1, 1);
	m68k_bus_map(0x130000, 0x130fff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus_map(0x170000, 0x17ffff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	pg_reset();
	return 0;
}

static void pg_exit(void)
{
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
	m68k_bus.op = NULL;
	m68k_bus.op_size = 0;
	free(s_opcodes);
	s_opcodes = NULL;
}

static void pg_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // vblank_w: IRQ 6 (ASSERT_LINE) until vblank_ack_w
			vblank_capture();
			m68k_bus_set_line(6, 1);
		}
		if (line >= VIS_Y0 && line < VIS_Y0 + VIS_H)
			deco16ic_latch_line(&s_tile, line);
		m68k_execute(rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		sound_line();
	}
	s_frame++;
}

static int pg_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[2] = { s_cpu_frac, s_snd_frac };
	uint32_t misc[5] = { s_frame, s_sfx_phase, (uint32_t)s_sfx_last, s_oki[1].bank, 0 };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "rows", s_rowscroll, sizeof(s_rowscroll));
	rc |= rbx_state_io(ctx, "vram", s_tile.vram, sizeof(s_tile.vram));
	rc |= rbx_state_io(ctx, "tctl", s_tile.control, sizeof(s_tile.control));
	rc |= rbx_state_io(ctx, "prot", &s_prot, DECO146_STATE_SIZE);
	rc |= rbx_state_io(ctx, "oki1", s_oki[0].v, sizeof(s_oki[0].v) + sizeof(int));
	rc |= rbx_state_io(ctx, "oki2", s_oki[1].v, sizeof(s_oki[1].v) + sizeof(int));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_frame = misc[0];
		s_sfx_phase = misc[1];
		s_sfx_last = (int32_t)misc[2];
		s_oki[1].bank = misc[3];
		for (int i = 0; i < 8; i++)
			deco16ic_control_w(&s_tile, i, s_tile.control[i], 0xffff);
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int pg_save(void *ctx) { return pg_state(ctx, 0); }
static int pg_load(void *ctx) { return pg_state(ctx, 1); }
static int pg_supported(const char *machine) { return !strcmp(machine, "pktgaldx"); }

const rbx_driver rbx_drv_pktgaldx = { "DECO 16 Pocket Gal Deluxe", pg_init, pg_exit, pg_reset, pg_frame, pg_save, pg_load,
                                      pg_supported, pg_render, NULL };
