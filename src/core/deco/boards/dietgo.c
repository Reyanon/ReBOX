/*  Data East DE-0396-1 (?): Diet Go Go. Ported from MAME src/mame/dataeast/dietgo.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail, David Haywood).
    The 102 encrypted 68000 at 14 MHz (machine/deco102.c: the data and the opcodes two decryptions), IRQ 6 at vblank
    (HOLD_LINE), the 104 protection / I/O chip (inputs, sound latch), tilemap chip 141 (encrypted graphics), sprite
    chip 52, the sound board (machine/deco16snd.c: HuC6280, YM2151, OKI).
    Funky Jet / Sotsugyo Shousho (Mitchell, MAME src/mame/dataeast/funkyjet.cpp, copyright-holders: Bryan McPhail): the
    same board with a plain 68000 at 14.16 MHz, the 146 and xBGR_444 colours, sound in stereo.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../video/gfxlayout.h"
#include "../machine/decocrpt.h"
#include "../machine/deco102.h"
#include "../machine/deco146.h"
#include "../machine/deco16snd.h"
#include "../video/deco16ic.h"
#include "../video/decospr.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 14000000       // 28 MHz / 2
#define FJ_CPU_CLOCK 14161000    // 28.322 MHz / 2
#define FRAME_MHZ 58000          // set_refresh_hz(58)
#define LINES 256                // set_size(40*8, 32*8)
#define VIS_Y0 8                 // set_visarea(0, 319, 8, 247)
#define VIS_H 240
#define VIS_W 320
#define VBL_LINES 37             // vblank 2500 us at 58 Hz x 256 lines (67.3 us a line)
#define FJ_VBL_LINES 8           // funkyjet: 529 us

static int s_fj;                          // Funky Jet's board
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint16_t *s_opcodes;               // the decrypted opcodes (MAME AS_OPCODES)
static uint16_t (*s_ram_hp)[0x8000];            // 0x380000 (64 KB)
#define s_ram (*s_ram_hp)
static uint16_t s_spriteram[0x400];
static uint16_t s_pal[0x800];             // xBGR_888 in word pairs (0x300 entries; the page's 4 KB); funkyjet xBGR_444
static uint16_t s_rowscroll[2][0x800];
static deco16ic_t s_tile;
static decospr_t s_spr;
static deco146_t s_prot;
static rbx_gfx s_gfx8, s_gfx16, s_gfxspr;
static int s_port_in, s_port_dsw, s_port_sys;
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac;
static struct {
	deco16ic_t tile;
	uint16_t rowscroll[2][0x800], spriteram[0x400], pal[0x800];
	uint32_t frame;
} *s_snap;

// ---- the 104's ports and latch ----
static uint16_t port_a(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_in); }
static uint16_t port_b(void *u)
{
	(void)u;
	// (the screen's vblank: MAME's vblank time from the visible area's end)
	const int vbl = s_line >= VIS_Y0 + VIS_H && s_line < VIS_Y0 + VIS_H + (s_fj ? FJ_VBL_LINES : VBL_LINES);
	rbx_port_custom(s_port_sys, 0x0008, vbl ? 0x0008 : 0);
	return (uint16_t)rbx_port_read(s_port_sys);
}
static uint16_t port_c(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_dsw); }
static void latch_irq(void *u, int state) { (void)u; deco16snd_latch_irq(state); }
static uint8_t latch_r(void) { return deco146_soundlatch_r(&s_prot); }

// ---- bus ----
// (ioprot_r / w: the address bitswap keeps bits 0-10, the 104's range)
static uint32_t s_prot_base;              // 0x340000 (funkyjet 0x180000)
static uint16_t prot_addr(uint32_t a) { return (uint16_t)((a - s_prot_base) & 0x7fe); }

static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= s_prot_base && a < s_prot_base + 0x4000) return deco146_read(&s_prot, prot_addr(a), NULL);
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
	if (a >= (s_fj ? 0x300000u : 0x200000u) && a < (s_fj ? 0x300010u : 0x200010u))
		deco16ic_control_w(&s_tile, (int)((a >> 1) & 7), v, mask);
	else if (a >= s_prot_base && a < s_prot_base + 0x4000)
		deco146_write(&s_prot, prot_addr(a), v, mask, NULL);
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static int bank_cb(int bank) { return (bank & 0x70) << 8; }

static void dg_render(void)
{
	if (s_fj)
		for (int i = 0; i < 0x400; i++) {   // xBGR_444
			const uint16_t p = s_snap->pal[i];
			rbx_video.palette[i] = (uint32_t)(((p & 15) * 17) << 16 | (((p >> 4) & 15) * 17) << 8 | (((p >> 8) & 15) * 17));
		}
	else {
		for (int i = 0; i < 0x300; i++) {   // xBGR_888 (the entry's two words, big-endian)
			const uint32_t p = (uint32_t)s_snap->pal[2 * i] << 16 | s_snap->pal[2 * i + 1];
			rbx_video.palette[i] = (p & 0xff) << 16 | (p & 0xff00) | ((p >> 16) & 0xff);
		}
		for (int i = 0x300; i < 0x400; i++)
			rbx_video.palette[i] = 0;
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	deco16ic_t *t = &s_snap->tile;
	deco16ic_update(t);
	decospr_t spr = s_spr;
	spr.flip_screen = (deco16ic_control_r(t, 0) >> 7) & 1;
	// (pen 256 under it all: the back playfield is opaque) the back playfield, the front one, the sprites
	deco16ic_draw(t, 1, s_snap->rowscroll[1], bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 1, 0, 0xff);
	deco16ic_draw(t, 0, s_snap->rowscroll[0], bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
	decospr_draw(&spr, s_snap->spriteram, 0x400, bm, NULL, pitch, VIS_W, VIS_H, VIS_Y0, 1, s_snap->frame);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	s_snap->tile = s_tile;
	memcpy(s_snap->rowscroll, s_rowscroll, sizeof(s_rowscroll));
	memcpy(s_snap->spriteram, s_spriteram, sizeof(s_spriteram));
	memcpy(s_snap->pal, s_pal, sizeof(s_snap->pal));
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void dg_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_rowscroll, 0, sizeof(s_rowscroll));
	memset(s_tile.vram, 0, sizeof(s_tile.vram));
	deco16ic_reset(&s_tile);
	deco146_reset(&s_prot);
	deco16snd_reset();
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = 0;
}

static void dg_exit(void);

static int dg_init(const rbx_game *g)
{
	if (!s_ram_hp && !(s_ram_hp = calloc(1, sizeof(*s_ram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t snd_size, tsize, ssize, osize;
	s_fj = !strcmp(g->machine, "funkyjet");
	s_prot_base = s_fj ? 0x180000 : 0x340000;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *snd = rbx_region("audiocpu", &snd_size), *tiles = rbx_region("tiles", &tsize),
	        *spr = rbx_region("sprites", &ssize), *oki = rbx_region("oki", &osize);
	if (!s_rom || s_rom_size < 0x80000 || !snd || !tiles || !spr || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	if (s_fj ? deco74_decrypt_gfx(tiles, tsize)
	         : (!(s_opcodes = (uint16_t *)malloc(0x80000)) || deco56_decrypt_gfx(tiles, tsize) ||
	            deco102_decrypt_cpu((uint16_t *)s_rom, s_opcodes, 0x80000, 0xe9ba, 0x01, 0x19))) {
		rbx_fail("%s: out of memory", g->set);
		dg_exit();
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
			dg_exit();
			return -1;
		}
	}
	memset(&s_tile, 0, sizeof(s_tile));
	for (int i = 0; i < 2; i++) {
		s_tile.tm[i].size = DECO_64x32;
		s_tile.tm[i].col_bank = i ? 0x10 : 0x00;
		s_tile.tm[i].col_mask = 0x0f;
		s_tile.tm[i].bank_cb = s_fj ? NULL : bank_cb;
	}
	if (s_fj) {   // (gfx_funkyjet: tiles from colour 256; the scroll's 1 pixel as supbtime's chinatwn)
		s_tile.base8 = s_tile.base16 = 256;
		s_tile.scrolldx = 1;
	}
	s_tile.gfx8 = &s_gfx8;
	s_tile.gfx16 = &s_gfx16;
	memset(&s_spr, 0, sizeof(s_spr));
	s_spr.gfx = &s_gfxspr;
	s_spr.color_base = s_fj ? 0 : 512;
	deco146_init(&s_prot, !s_fj);
	static const uint8_t interleave[10] = { 4, 5, 3, 6, 2, 7, 1, 8, 0, 9 };   // set_interface_scramble_interleave
	deco146_set_scramble(&s_prot, interleave);
	s_prot.magic_read_xor_on = !s_fj;
	s_prot.port_a = port_a;
	s_prot.port_b = port_b;
	s_prot.port_c = port_c;
	s_prot.soundlatch_irq = latch_irq;
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_snap) {
		rbx_fail("%s: out of memory", g->set);
		dg_exit();
		return -1;
	}
	// sound: HuC6280 2.685 MHz, YM2151 3.58 MHz 0.45 on both outputs, OKI 1 MHz 0.60, one speaker
	deco16snd_cfg sc;
	memset(&sc, 0, sizeof(sc));
	sc.cpu_clock = s_fj ? 32220000 / 4 : 32220000 / 4 / 3;
	sc.ym_clock = 32220000 / 9;
	sc.ym_gain[0] = sc.ym_gain[1] = 115;
	sc.oki_rom[0] = oki;
	sc.oki_size[0] = osize;
	sc.oki_clock[0] = s_fj ? 28000000 / 28 : 32220000 / 32;
	sc.oki_gain[0] = s_fj ? 128 : 154;   // (funkyjet: 0.50 to both sides)
	sc.stereo = s_fj;
	sc.latch_r = latch_r;
	if (deco16snd_init(snd, snd_size, &sc, FRAME_MHZ, LINES) || rbx_video_setup(VIS_W, VIS_H, 0x400)) {
		rbx_fail("%s: out of memory", g->set);
		dg_exit();
		return -1;
	}
	s_port_in = rbx_port_find("INPUTS");
	s_port_dsw = rbx_port_find("DSW");
	s_port_sys = rbx_port_find("SYSTEM");
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x7ffff, s_rom, 0x80000, 1, 0);
	if (s_fj) {
		m68k_bus_map(0x120000, 0x120fff, s_pal, sizeof(s_pal), 1, 1);
		m68k_bus_map(0x140000, 0x143fff, s_ram, 0x4000, 1, 1);
		m68k_bus_map(0x160000, 0x160fff, s_spriteram, sizeof(s_spriteram), 1, 1);
		m68k_bus_map(0x320000, 0x321fff, s_tile.vram[0], sizeof(s_tile.vram[0]), 1, 1);
		m68k_bus_map(0x322000, 0x323fff, s_tile.vram[1], sizeof(s_tile.vram[1]), 1, 1);
		m68k_bus_map(0x340000, 0x340fff, s_rowscroll[0], sizeof(s_rowscroll[0]), 1, 1);
		m68k_bus_map(0x342000, 0x342fff, s_rowscroll[1], sizeof(s_rowscroll[1]), 1, 1);
		goto mapped;
	}
	m68k_bus.op = (const uint8_t *)s_opcodes;
	m68k_bus.op_size = 0x80000;
	m68k_bus_map(0x210000, 0x211fff, s_tile.vram[0], sizeof(s_tile.vram[0]), 0, 1);   // (write only)
	m68k_bus_map(0x212000, 0x213fff, s_tile.vram[1], sizeof(s_tile.vram[1]), 0, 1);
	m68k_bus_map(0x220000, 0x220fff, s_rowscroll[0], sizeof(s_rowscroll[0]), 0, 1);
	m68k_bus_map(0x222000, 0x222fff, s_rowscroll[1], sizeof(s_rowscroll[1]), 0, 1);
	m68k_bus_map(0x280000, 0x280fff, s_spriteram, sizeof(s_spriteram), 1, 1);
	m68k_bus_map(0x300000, 0x300fff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus_map(0x380000, 0x38ffff, s_ram, sizeof(s_ram), 1, 1);
mapped:
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	dg_reset();
	return 0;
}

static void dg_exit(void)
{
	deco16snd_exit();
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
	m68k_bus.op = NULL;
	m68k_bus.op_size = 0;
	free(s_opcodes);
	s_opcodes = NULL;
	free(s_ram_hp);
	s_ram_hp = NULL;
}

static void dg_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // irq6_line_hold at vblank
			vblank_capture();
			m68k_bus_hold_irq(6);
		}
		if (line >= VIS_Y0 && line < VIS_Y0 + VIS_H)
			deco16ic_latch_line(&s_tile, line);
		m68k_execute(rbx_line_cycles(&s_cpu_frac, s_fj ? FJ_CPU_CLOCK : CPU_CLOCK, FRAME_MHZ, LINES));
		deco16snd_line();
	}
	deco16snd_frame_end();
	s_frame++;
}

static int dg_state(void *ctx, int load)
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
	rc |= rbx_state_io(ctx, "prot", &s_prot, DECO146_STATE_SIZE);
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

static int dg_save(void *ctx) { return dg_state(ctx, 0); }
static int dg_load(void *ctx) { return dg_state(ctx, 1); }
static int dg_supported(const char *machine) { return !strcmp(machine, "dietgo"); }
static int fj_supported(const char *machine) { return !strcmp(machine, "funkyjet"); }

const rbx_driver rbx_drv_dietgo = { "DECO 16 Diet Go Go", dg_init, dg_exit, dg_reset, dg_frame, dg_save, dg_load,
                                    dg_supported, dg_render, NULL };
const rbx_driver rbx_drv_funkyjet = { "DECO 16 Funky Jet", dg_init, dg_exit, dg_reset, dg_frame, dg_save, dg_load,
                                      fj_supported, dg_render, NULL };
