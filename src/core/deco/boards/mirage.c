/*  Mitchell MT4001-2: Mirage Youjuu Mahjongden. Ported from MAME src/mame/dataeast/mirage.cpp (BSD-3-Clause,
    copyright-holders: David Haywood).
    68000 at 14 MHz (IRQ 6 held at vblank), the tilemap chip (graphics encrypted as the 56's), sprite chip 52 (its list
    buffered at vblank; the extra priority bit under the front playfield), two OKIs on the 68000's bus with banked
    samples, the mahjong key matrix, a 93C46 EEPROM (kept per set).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../machine/eeprom93c46.h"
#include "../../sound/okim6295.h"
#include "../../video/gfxlayout.h"
#include "../machine/decocrpt.h"
#include "../video/deco16ic.h"
#include "../video/decospr.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 14000000       // 28 MHz / 2
#define FRAME_MHZ 58000          // set_refresh_hz(58)
#define LINES 256                // set_size(40*8, 32*8)
#define VIS_Y0 8                 // set_visarea(0, 319, 8, 247)
#define VIS_H 240
#define VIS_W 320
#define VBL_LINES 8              // vblank 529 us

static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint16_t (*s_ram_hp)[0x2000];             // 0x170000 .. 0x173fff
#define s_ram (*s_ram_hp)
static uint16_t s_spriteram[0x400], s_spriteram_buf[0x400];
static uint16_t s_pal[0x400];              // xBGR_555
static uint16_t s_rowscroll[2][0x600];     // 0x110000 .. 0x110bff, 0x112000 .. 0x112bff
static uint16_t s_rowscroll_pad[2][0x200]; // (to 0x1000 bytes: the chip's row / column scroll reach)
static deco16ic_t s_tile;
static decospr_t s_spr;
static rbx_gfx s_gfx8, s_gfx16, s_gfxspr;
static okim6295_t s_oki_bgm, s_oki_sfx;
static int s_port_sys, s_port_key[5];
static uint8_t s_key_select;
static eeprom93c46_t s_eeprom;
static char s_nv_path[256];
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac;
static uint64_t s_snd_frac;
static uint32_t s_out_rate, s_sfx_rate, s_sfx_phase;
static int32_t s_sfx_last;
static uint8_t (*s_pri_buf_hp)[VIS_W * VIS_H];
#define s_pri_buf (*s_pri_buf_hp)
static struct {
	deco16ic_t tile;
	uint16_t rowscroll[2][0x800], spriteram[0x400], pal[0x400];
	uint32_t frame;
} *s_snap;

// ---- bus ----
static uint16_t key_matrix_r(void)
{
	// select = bitswap<5>(m_key_matrix_select, 2, 4, 1, 3, 0)
	const unsigned s = s_key_select;
	const unsigned sel = ((s >> 2) & 1) << 4 | ((s >> 4) & 1) << 3 | ((s >> 1) & 1) << 2 | ((s >> 3) & 1) << 1 | (s & 1);
	uint16_t r = 0x3f;
	for (int i = 0; i < 5; i++)
		if (sel >> i & 1)
			r &= (uint16_t)rbx_port_read(s_port_key[i]);
	return (uint16_t)(0xffc0 | r);
}

static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x140000 && a < 0x140010) return oki_read(&s_oki_sfx);
	if (a >= 0x150000 && a < 0x150010) return oki_read(&s_oki_bgm);
	if (a == 0x16c006) return key_matrix_r();
	if (a == 0x16e002) {   // SYSTEM_IN: bit 4 the vblank, bit 5 the EEPROM's data out
		const int vbl = s_line >= VIS_Y0 + VIS_H && s_line < VIS_Y0 + VIS_H + VBL_LINES;
		rbx_port_custom(s_port_sys, 0x30, (vbl ? 0x10u : 0) | (eeprom_do(&s_eeprom) ? 0x20u : 0));
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
	if (a >= 0x140000 && a < 0x140010) { if (mask & 0xff) oki_write(&s_oki_sfx, (uint8_t)v); }   // (umask16 0x00ff)
	else if (a >= 0x150000 && a < 0x150010) { if (mask & 0xff) oki_write(&s_oki_bgm, (uint8_t)v); }
	else if (a >= 0x168000 && a < 0x168010)
		deco16ic_control_w(&s_tile, (int)((a >> 1) & 7), v, mask);
	else if (a == 0x16c000)   // okim1_rombank_w
		s_oki_sfx.bank = (uint32_t)(v & 3) * 0x40000;
	else if (a == 0x16c002) {   // eeprom_okim0_rombank_w: clock bit 5, data in bit 4, select bit 6
		eeprom_write_lines(&s_eeprom, s_eeprom.cs, (v >> 5) & 1, (v >> 4) & 1);   // (MAME: clock, data, then select)
		eeprom_write_lines(&s_eeprom, (v >> 6) & 1, (v >> 5) & 1, (v >> 4) & 1);
		s_oki_bgm.bank = (uint32_t)(v & 7) * 0x40000;
	}
	else if (a == 0x16c004)   // key_matrix_w
		s_key_select = (uint8_t)(v & 0x1f);
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static int bank_cb(int bank) { return (bank & 0x70) << 8; }
static uint32_t pri_cb(uint16_t x, int extpri) { (void)x; return extpri ? 1u << 2 : 0; }   // (GFX_PMASK_2)

static void mj_render(void)
{
	for (int i = 0; i < 0x400; i++) {   // xBGR_555
		const uint16_t p = s_snap->pal[i];
		const int r = p & 31, g = (p >> 5) & 31, b = (p >> 10) & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (b << 3 | b >> 2));
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	deco16ic_t *t = &s_snap->tile;
	deco16ic_update(t);
	decospr_t spr = s_spr;
	spr.flip_screen = (deco16ic_control_r(t, 0) >> 7) & 1;
	for (int y = 0; y < VIS_H; y++)   // (bitmap.fill(256): "not verified")
		for (int x = 0; x < VIS_W; x++)
			bm[y * pitch + x] = 256;
	memset(s_pri_buf, 0, sizeof(s_pri_buf));
	deco16ic_draw(t, 1, s_snap->rowscroll[1], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 1, 1, 0xff);
	deco16ic_draw(t, 0, s_snap->rowscroll[0], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 2, 0xff);
	decospr_draw(&spr, s_snap->spriteram, 0x400, bm, s_pri_buf, pitch, VIS_W, VIS_H, VIS_Y0, 1, s_snap->frame);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	s_snap->tile = s_tile;
	for (int i = 0; i < 2; i++) {
		memcpy(s_snap->rowscroll[i], s_rowscroll[i], sizeof(s_rowscroll[i]));
		memset(s_snap->rowscroll[i] + 0x600, 0, 0x200 * 2);
	}
	memcpy(s_snap->spriteram, s_spriteram_buf, sizeof(s_spriteram_buf));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void mj_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_spriteram_buf, 0, sizeof(s_spriteram_buf));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_rowscroll, 0, sizeof(s_rowscroll));
	memset(s_tile.vram, 0, sizeof(s_tile.vram));
	deco16ic_reset(&s_tile);
	oki_reset(&s_oki_bgm);
	oki_reset(&s_oki_sfx);
	s_oki_bgm.bank = s_oki_sfx.bank = 0;
	s_snd_frac = 0;
	s_sfx_phase = 0;
	s_sfx_last = 0;
	s_key_select = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_frame = 0;
	s_cpu_frac = 0;
}

static void mj_exit(void);

static int mj_init(const rbx_game *g)
{
	if (!s_pri_buf_hp && !(s_pri_buf_hp = calloc(1, sizeof(*s_pri_buf_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ram_hp && !(s_ram_hp = calloc(1, sizeof(*s_ram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t tsize, ssize, bsize, fsize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *tiles = rbx_region("tiles", &tsize), *spr = rbx_region("sprites", &ssize),
	        *bgm = rbx_region("oki_bgm", &bsize), *sfx = rbx_region("oki_sfx", &fsize);
	if (!s_rom || s_rom_size < 0x80000 || !tiles || !spr || !bgm || !sfx) {
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
		const uint32_t th = tsize * 4, sh = ssize * 4;
		const uint32_t pt[4] = { th + 8, th, 8, 0 }, ps[4] = { sh + 8, sh, 8, 0 };
		const rbx_gfx_layout l8 = { 8, 8, 4, pt, x8, y8, 16 * 8 };
		const rbx_gfx_layout l16 = { 16, 16, 4, pt, x16, y16, 32 * 16 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, x16, y16, 32 * 16 };
		if (rbx_gfx_decode(&s_gfx8, &l8, tiles, tsize / 2) || rbx_gfx_decode(&s_gfx16, &l16, tiles, tsize / 2) ||
		    rbx_gfx_decode(&s_gfxspr, &ls, spr, ssize / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			mj_exit();
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
	s_spr.color_base = 0x200;
	s_spr.pri_cb = pri_cb;
	// sound: OKI (BGM) 2 MHz 0.30, OKI (SFX) 1 MHz 0.70, pin 7 high, one speaker
	oki_init(&s_oki_bgm, 2000000, 1, bgm, bsize, 77);
	oki_init(&s_oki_sfx, 1000000, 1, sfx, fsize, 179);
	s_out_rate = s_oki_bgm.rate;
	s_sfx_rate = s_oki_sfx.rate;
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_snap || rbx_audio_setup(s_out_rate) || rbx_video_setup(VIS_W, VIS_H, 0x400)) {
		rbx_fail("%s: out of memory", g->set);
		mj_exit();
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
	s_port_sys = rbx_port_find("SYSTEM_IN");
	for (int i = 0; i < 5; i++) {
		char n[8] = { 'K', 'E', 'Y', (char)('0' + i), 0 };
		s_port_key[i] = rbx_port_find(n);
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x7ffff, s_rom, 0x80000, 1, 0);
	m68k_bus_map(0x100000, 0x101fff, s_tile.vram[0], sizeof(s_tile.vram[0]), 1, 1);
	m68k_bus_map(0x102000, 0x103fff, s_tile.vram[1], sizeof(s_tile.vram[1]), 1, 1);
	m68k_bus_map(0x110000, 0x110bff, s_rowscroll[0], sizeof(s_rowscroll[0]), 1, 1);
	m68k_bus_map(0x112000, 0x112bff, s_rowscroll[1], sizeof(s_rowscroll[1]), 1, 1);
	m68k_bus_map(0x120000, 0x1207ff, s_spriteram, sizeof(s_spriteram), 1, 1);
	m68k_bus_map(0x130000, 0x1307ff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus_map(0x170000, 0x173fff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	mj_reset();
	return 0;
}

static void mj_exit(void)
{
	if (s_nv_path[0] && s_eeprom.dirty)
		eeprom_save(&s_eeprom, s_nv_path);
	s_nv_path[0] = 0;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
	free(s_pri_buf_hp);
	s_pri_buf_hp = NULL;
	free(s_ram_hp);
	s_ram_hp = NULL;
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
			s_sfx_last = oki_sample(&s_oki_sfx);
		}
		rbx_audio_put(s_sfx_last + oki_sample(&s_oki_bgm));
	}
}

static void mj_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // irq6_line_hold, the sprite buffer copied (vblank_copy_rising)
			memcpy(s_spriteram_buf, s_spriteram, sizeof(s_spriteram));
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

static int mj_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[2] = { s_cpu_frac, s_snd_frac };
	uint32_t misc[6] = { s_frame, s_sfx_phase, (uint32_t)s_sfx_last, s_oki_bgm.bank, s_oki_sfx.bank, s_key_select };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "sprb", s_spriteram_buf, sizeof(s_spriteram_buf));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "rows", s_rowscroll, sizeof(s_rowscroll));
	rc |= rbx_state_io(ctx, "vram", s_tile.vram, sizeof(s_tile.vram));
	rc |= rbx_state_io(ctx, "tctl", s_tile.control, sizeof(s_tile.control));
	rc |= rbx_state_io(ctx, "eeprom", &s_eeprom, sizeof(s_eeprom));
	rc |= rbx_state_io(ctx, "okib", s_oki_bgm.v, sizeof(s_oki_bgm.v) + sizeof(int));
	rc |= rbx_state_io(ctx, "okis", s_oki_sfx.v, sizeof(s_oki_sfx.v) + sizeof(int));
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
		s_oki_bgm.bank = misc[3];
		s_oki_sfx.bank = misc[4];
		s_key_select = (uint8_t)misc[5];
		for (int i = 0; i < 8; i++)
			deco16ic_control_w(&s_tile, i, s_tile.control[i], 0xffff);
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int mj_save(void *ctx) { return mj_state(ctx, 0); }
static int mj_load(void *ctx) { return mj_state(ctx, 1); }
static int mj_supported(const char *machine) { return !strcmp(machine, "mirage"); }

const rbx_driver rbx_drv_mirage = { "DECO 16 Mirage Youjuu Mahjongden", mj_init, mj_exit, mj_reset, mj_frame, mj_save,
                                    mj_load, mj_supported, mj_render, NULL };
