/*  NDK / Data East Dream Ball. Ported from MAME src/mame/dataeast/dreambal.cpp (BSD-3-Clause, copyright-holders:
    David Haywood).
    68000 at 14 MHz (IRQ 6 held at vblank), the tilemap chip 141 (graphics encrypted as the 56's), no sprites, the 104
    (inputs; the EEPROM's data out on its port C), a 93C46 EEPROM (kept per set), an OKI.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../machine/eeprom93c46.h"
#include "../../sound/okim6295.h"
#include "../../video/gfxlayout.h"
#include "../machine/deco146.h"
#include "../machine/decocrpt.h"
#include "../video/deco16ic.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 14000000       // 28 MHz / 2
#define FRAME_MHZ 58000          // set_refresh_hz(58)
#define LINES 256                // set_size(64*8, 32*8)
#define VIS_Y0 8                 // set_visarea(0, 319, 8, 247)
#define VIS_H 240
#define VIS_W 320

static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint16_t s_ram[0x2000];             // 0x120000 .. 0x123fff
static uint16_t s_vram_ext[2][0x800];      // 0x101000 .. 0x101fff, 0x103000 .. 0x103fff
static uint16_t s_pal[0x200];              // xBGR_444
static deco16ic_t s_tile;
static deco146_t s_prot;
static eeprom93c46_t s_eeprom;
static char s_nv_path[256];
static okim6295_t s_oki;
static rbx_gfx s_gfx8, s_gfx16;
static int s_port_in, s_port_sys, s_port_dsw;
static uint32_t s_frame;
static uint64_t s_cpu_frac, s_snd_frac;
static struct {
	deco16ic_t tile;
	uint16_t pal[0x200];
	uint32_t frame;
} *s_snap;

// ---- the 104's ports ----
static uint16_t port_a(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_in); }
static uint16_t port_b(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_sys); }
static uint16_t port_c(void *u)   // DSW: bit 0 the EEPROM's data out
{
	(void)u;
	rbx_port_custom(s_port_dsw, 0x0001, eeprom_do(&s_eeprom) ? 0x0001 : 0);
	return (uint16_t)rbx_port_read(s_port_dsw);
}

// ---- bus ----
static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x160000 && a < 0x164000) return deco146_read(&s_prot, (uint16_t)(a & 0x7fe), NULL);   // ioprot_r
	if (a == 0x180000) return oki_read(&s_oki);
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
	if (a >= 0x161000 && a < 0x161010)   // (over the 104's range, as MAME's later map entries)
		deco16ic_control_w(&s_tile, (int)((a >> 1) & 7), v, mask);
	else if (a == 0x162000 || a == 0x163000)
		;
	else if (a >= 0x160000 && a < 0x164000)
		deco146_write(&s_prot, (uint16_t)(a & 0x7fe), v, mask, NULL);
	else if (a == 0x165000) {   // eeprom_w: clock bit 1, data in bit 0, select bit 2
		if (mask & 0xff) {   // (MAME: the clock and data first, then the select)
			eeprom_write_lines(&s_eeprom, s_eeprom.cs, (v >> 1) & 1, v & 1);
			eeprom_write_lines(&s_eeprom, (v >> 2) & 1, (v >> 1) & 1, v & 1);
		}
	} else if (a == 0x180000) {
		if (mask & 0xff)
			oki_write(&s_oki, (uint8_t)v);
	}
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static int bank_cb(int bank) { return (bank & 0x70) << 8; }

static void db_render(void)
{
	for (int i = 0; i < 0x200; i++) {   // xBGR_444
		const uint16_t p = s_snap->pal[i];
		rbx_video.palette[i] = (uint32_t)(((p & 15) * 17) << 16 | (((p >> 4) & 15) * 17) << 8 | (((p >> 8) & 15) * 17));
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	deco16ic_t *t = &s_snap->tile;
	deco16ic_update(t);
	for (int y = 0; y < VIS_H; y++)   // (bitmap.fill(0): "not confirmed")
		memset(bm + y * pitch, 0, VIS_W * 2);
	deco16ic_draw(t, 1, NULL, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 2, 0xff);
	deco16ic_draw(t, 0, NULL, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 4, 0xff);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	s_snap->tile = s_tile;
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- sound: the OKI at its rate ----
static void sound_line(void)
{
	s_snd_frac += (uint64_t)s_oki.rate * 1000;
	const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
	while (s_snd_frac >= den) {
		s_snd_frac -= den;
		rbx_audio_put(oki_sample(&s_oki));
	}
}

// ---- init / frame ----
static void db_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_vram_ext, 0, sizeof(s_vram_ext));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_tile.vram, 0, sizeof(s_tile.vram));
	deco16ic_reset(&s_tile);
	deco146_reset(&s_prot);
	oki_reset(&s_oki);
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_frame = 0;
	s_cpu_frac = s_snd_frac = 0;
}

static void db_exit(void);

static int db_init(const rbx_game *g)
{
	uint32_t tsize, osize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *tiles = rbx_region("tiles", &tsize), *oki = rbx_region("oki", &osize);
	if (!s_rom || s_rom_size < 0x80000 || !tiles || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (deco56_decrypt_gfx(tiles, tsize)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	{	// tile_8x8_layout / tile_16x16_layout (RGN_FRAC(1,2))
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 16);
			x16[i] = (uint32_t)(256 + i);
			x16[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 16);
		const uint32_t th = tsize * 4;
		const uint32_t pt[4] = { th + 8, th, 8, 0 };
		const rbx_gfx_layout l8 = { 8, 8, 4, pt, x8, y8, 16 * 8 };
		const rbx_gfx_layout l16 = { 16, 16, 4, pt, x16, y16, 32 * 16 };
		if (rbx_gfx_decode(&s_gfx8, &l8, tiles, tsize / 2) || rbx_gfx_decode(&s_gfx16, &l16, tiles, tsize / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			db_exit();
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
	deco146_init(&s_prot, 1);
	s_prot.port_a = port_a;
	s_prot.port_b = port_b;
	s_prot.port_c = port_c;
	// sound: OKI 9.8304 MHz / 8, pin 7 high, 1.0
	oki_init(&s_oki, 9830400 / 8, 1, oki, osize, 256);
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_snap || rbx_audio_setup(s_oki.rate) || rbx_video_setup(VIS_W, VIS_H, 0x200)) {
		rbx_fail("%s: out of memory", g->set);
		db_exit();
		return -1;
	}
	s_nv_path[0] = 0;
	eeprom_init(&s_eeprom);
	{
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			eeprom_load(&s_eeprom, s_nv_path);
		}
	}
	s_port_in = rbx_port_find("INPUTS");
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_dsw = rbx_port_find("DSW");
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x7ffff, s_rom, 0x80000, 1, 0);
	m68k_bus_map(0x100000, 0x100fff, s_tile.vram[0], 0x1000, 1, 1);
	m68k_bus_map(0x101000, 0x101fff, s_vram_ext[0], sizeof(s_vram_ext[0]), 1, 1);
	m68k_bus_map(0x102000, 0x102fff, s_tile.vram[1], 0x1000, 1, 1);
	m68k_bus_map(0x103000, 0x103fff, s_vram_ext[1], sizeof(s_vram_ext[1]), 1, 1);
	m68k_bus_map(0x120000, 0x123fff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x140000, 0x1403ff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	db_reset();
	return 0;
}

static void db_exit(void)
{
	if (s_nv_path[0] && s_eeprom.dirty)
		eeprom_save(&s_eeprom, s_nv_path);
	s_nv_path[0] = 0;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16);
	free(s_snap);
	s_snap = NULL;
}

static void db_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0 + VIS_H) {   // irq6_line_hold at vblank
			vblank_capture();
			m68k_bus_hold_irq(6);
		}
		if (line >= VIS_Y0 && line < VIS_Y0 + VIS_H)
			deco16ic_latch_line(&s_tile, line);
		m68k_execute(rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		sound_line();
	}
	s_frame++;
}

static int db_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[2] = { s_cpu_frac, s_snd_frac };
	uint32_t misc[2] = { s_frame, 0 };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "vext", s_vram_ext, sizeof(s_vram_ext));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "vram", s_tile.vram, sizeof(s_tile.vram));
	rc |= rbx_state_io(ctx, "tctl", s_tile.control, sizeof(s_tile.control));
	rc |= rbx_state_io(ctx, "prot", &s_prot, DECO146_STATE_SIZE);
	rc |= rbx_state_io(ctx, "eeprom", &s_eeprom, sizeof(s_eeprom));
	rc |= rbx_state_io(ctx, "oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
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
		for (int i = 0; i < 8; i++)
			deco16ic_control_w(&s_tile, i, s_tile.control[i], 0xffff);
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int db_save(void *ctx) { return db_state(ctx, 0); }
static int db_load(void *ctx) { return db_state(ctx, 1); }
static int db_supported(const char *machine) { return !strcmp(machine, "dreambal"); }

const rbx_driver rbx_drv_dreambal = { "DECO 16 Dream Ball", db_init, db_exit, db_reset, db_frame, db_save, db_load,
                                      db_supported, db_render, NULL };
