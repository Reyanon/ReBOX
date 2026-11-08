/*  Data East "dec8" boards with the BAC06 playfields, from MAME src/mame/dataeast/dec8.cpp, dec8_v.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail): Cobra-Command, Psycho-Nics Oscar.
    Cobra-Command: one 6809 at 1.5 MHz (NMI at vblank), two BAC06 playfields (8-bit, big-endian), the MXC06 sprites
    (a byte list copied to words by a write; colour bit 2 clear: behind the second playfield), a 2 bpp text layer, the
    DECO RM-C3 palette (two bytes an entry); the sound: R65C02 at 1.5 MHz with YM2203 and YM3812.
    Oscar: two 6809s sharing RAM (their IRQs raised and cleared by each other's writes), the coins on the main CPU's
    NMI (cleared by a write), one BAC06 (colour bit 3: pens 8-15 over the sprites), a 3 bpp text layer; the sound:
    DECO C10707 (a 6502 with its opcodes' bits 5 and 6 swapped) with YM2203 and YM3526.
    The sound CPU's IRQ from the YMs, the latch on its NMI. Reached through the dec8 driver (dec8.c).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../cpu/m6809/m6809.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../video/bac06.h"
#include "../video/decmxc06.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 1500000
#define SND_CLOCK 1500000
#define YM1_CLOCK 1500000        // YM2203
#define YM2_CLOCK 3000000        // YM3812 (Oscar: YM3526)
#define FRAME_MHZ 57445          // set_raw(12 MHz / 2, 384, 0, 256, 272, 8, 248)
#define LINES 272
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 256

enum { V_COBRA, V_OSCAR };
static int s_var;
static uint8_t *s_rom, *s_bank_rom, *s_sub_rom;
static uint32_t s_rom_size, s_bank_size, s_sub_size;
static uint8_t s_ram[0x3c00];              // 0x0000 .. 0x3bff (RAM, text, sprites, palette; the BAC06s apart)
static uint8_t s_sub_ram[0x100];           // (Oscar: the sub CPU's own 0x0f00 .. 0x0fff)
static uint16_t s_sprbuf[0x400];           // buffer_spriteram16_w
static int s_bank, s_bank_mask;
static m6809_t s_cpu[2];
static int s_irq[2], s_coin_state;
static bac06_t s_tile[2];
static decmxc06_t s_spr;
static rbx_gfx s_gfx8, s_gfxt1, s_gfxt2, s_gfxspr;
static uint8_t s_pri_buf[VIS_W * VIS_H];
static int s_port_in[3], s_port_dsw[2];
static int s_txt, s_spr_ram, s_pal;        // (the s_ram offsets of the text, the sprite list, the palette)
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac, s_snd_frac, s_ym1_frac, s_ym2_frac;
static struct {
	bac06_t tile[2];
	uint8_t txt[0x800], pal[0x400];
	uint16_t spr[0x400];
	uint32_t frame;
} *s_snap;

// ---- sound: R65C02 / DECO C10707, YM2203, YM3812 / YM3526 (dec8_s_map, oscar_s_map) ----
static m6502_t s_snd;
static uint8_t s_sram[0x600];
static uint8_t *s_zrom;
static uint32_t s_zrom_size;
static uint8_t s_latch;
static int s_ym_irq[2];
static int32_t *s_mix, *s_ym2buf;
static int s_mix_n, s_mix_cap, s_ym2_n, s_ym2_cap;

static void snd_irq(void) { m6502_set_irq(&s_snd, s_ym_irq[0] | s_ym_irq[1]); }
static void ym1_irq(int state) { s_ym_irq[0] = state; snd_irq(); }
static void ym2_irq(int state) { s_ym_irq[1] = state; snd_irq(); }
static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x0600) return s_sram[a];
	if (a == 0x6000) {
		m6502_set_nmi(&s_snd, 0);
		return s_latch;
	}
	if (a >= 0x8000) return a < s_zrom_size ? s_zrom[a] : 0;
	return 0;
}
static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0600) s_sram[a] = v;
	else if (a == 0x2000 || a == 0x2001) rbx_ym_write(0, a & 1, v);
	else if (a == 0x4000 || a == 0x4001) rbx_ym_write(1, a & 1, v);
}
static void sound_line(void)
{
	s_snd_frac += (uint64_t)SND_CLOCK * 1000;
	const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
	const int64_t cyc = (int64_t)(s_snd_frac / den);
	s_snd_frac -= (uint64_t)cyc * den;
	m6502_run_until(&s_snd, s_snd.cycles + cyc);
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
static void set_bank(int b)
{
	s_bank = b & s_bank_mask;
	const uint32_t o = (uint32_t)s_bank * 0x4000;
	for (int p = 0; p < 0x40; p++)
		s_cpu[0].rpage[0x40 + p] = o + 0x4000 <= s_bank_size ? s_bank_rom + o + (uint32_t)p * 256 : NULL;
}
static void buffer_spriteram16(void)   // buffer_spriteram16_w (the byte list as words, big-endian)
{
	for (int i = 0; i < 0x400; i++)
		s_sprbuf[i] = (uint16_t)(s_ram[s_spr_ram + i * 2 + 1] | (s_ram[s_spr_ram + i * 2] << 8));
}
static void set_irq(int c, int state)
{
	s_irq[c] = state;
	m6809_set_irq(&s_cpu[c], state);
}
// the BAC06's 8-bit accessors (MAME ctrlreg8_w, scrollreg8_w<false>, vram8_r / w<false>)
static void ctrl8_w(bac06_t *t, uint32_t o, uint8_t v) { bac06_ctrl_w(t, (int)(o >> 1), v, 0x00ff); }
static void scroll8_w(bac06_t *t, uint32_t o, uint8_t v)
{
	const uint32_t sh = o < 4 ? (~o & 1) << 3 : 0;
	bac06_scroll_w(t, (int)(o >> 1), (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}
static uint8_t vram8_r(const bac06_t *t, uint32_t o) { return (uint8_t)(bac06_vram_r(t, (int)(o >> 1)) >> ((~o & 1) << 3)); }
static void vram8_w(bac06_t *t, uint32_t o, uint8_t v)
{
	bac06_vram_w(t, (int)(o >> 1), (uint16_t)(v << ((~o & 1) << 3)), (uint16_t)(0xff << ((~o & 1) << 3)));
}

static uint8_t cobra_r(void *ctx, uint16_t a)   // cobra_map
{
	(void)ctx;
	if (a >= 0x0800 && a < 0x1000) return vram8_r(&s_tile[0], a - 0x0800u);
	if (a >= 0x1000 && a < 0x1800) return vram8_r(&s_tile[1], a - 0x1000u);
	switch (a) {
	case 0x3800: return (uint8_t)rbx_port_read(s_port_in[0]);
	case 0x3801: return (uint8_t)rbx_port_read(s_port_in[1]);
	case 0x3802: return (uint8_t)rbx_port_read(s_port_dsw[0]);
	case 0x3803: return (uint8_t)rbx_port_read(s_port_dsw[1]);
	case 0x3a00:
		rbx_port_custom(s_port_in[2], 0x80, vblank() ? 0x80 : 0);
		return (uint8_t)rbx_port_read(s_port_in[2]);
	}
	return 0;
}
static void cobra_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a >= 0x0800 && a < 0x1000) vram8_w(&s_tile[0], a - 0x0800u, v);
	else if (a >= 0x1000 && a < 0x1800) vram8_w(&s_tile[1], a - 0x1000u, v);
	else if (a >= 0x3800 && a < 0x3808) ctrl8_w(&s_tile[0], a & 7u, v);
	else if (a >= 0x3810 && a < 0x3820) scroll8_w(&s_tile[0], a & 0xfu, v);
	else if (a >= 0x3a00 && a < 0x3a08) ctrl8_w(&s_tile[1], a & 7u, v);
	else if (a >= 0x3a10 && a < 0x3a20) scroll8_w(&s_tile[1], a & 0xfu, v);
	else if (a == 0x3c00) set_bank(v);
	else if (a == 0x3c02) buffer_spriteram16();
	else if (a == 0x3e00) {
		s_latch = v;
		m6502_set_nmi(&s_snd, 1);
	}
}
static void oscar_irq_w(uint16_t a)   // (both CPUs: 0x3e80 .. 0x3e83)
{
	switch (a) {
	case 0x3e80: set_irq(1, 1); break;   // sub_irq_on_w
	case 0x3e81: set_irq(0, 0); break;   // main_irq_off_w
	case 0x3e82: set_irq(0, 1); break;   // main_irq_on_w
	case 0x3e83: set_irq(1, 0); break;   // sub_irq_off_w
	}
}
static uint8_t oscar_r(void *ctx, uint16_t a)   // oscar_map / oscar_sub_map
{
	if (ctx)
		return 0;
	if (a >= 0x2800 && a < 0x3000) return vram8_r(&s_tile[0], a - 0x2800u);
	switch (a) {
	case 0x3c00: return (uint8_t)rbx_port_read(s_port_in[0]);
	case 0x3c01: return (uint8_t)rbx_port_read(s_port_in[1]);
	case 0x3c02:
		rbx_port_custom(s_port_in[2], 0x80, vblank() ? 0x80 : 0);
		return (uint8_t)rbx_port_read(s_port_in[2]);
	case 0x3c03: return (uint8_t)rbx_port_read(s_port_dsw[0]);
	case 0x3c04: return (uint8_t)rbx_port_read(s_port_dsw[1]);
	}
	return 0;
}
static void oscar_w(void *ctx, uint16_t a, uint8_t v)
{
	if (ctx) {
		oscar_irq_w(a);
		return;
	}
	if (a >= 0x2800 && a < 0x3000) vram8_w(&s_tile[0], a - 0x2800u, v);
	else if (a >= 0x3c00 && a < 0x3c08) ctrl8_w(&s_tile[0], a & 7u, v);
	else if (a >= 0x3c10 && a < 0x3c20) scroll8_w(&s_tile[0], a & 0xfu, v);
	else if (a == 0x3c80) buffer_spriteram16();
	else if (a == 0x3d00) set_bank(v);
	else if (a == 0x3d80) {
		s_latch = v;
		m6502_set_nmi(&s_snd, 1);
	} else if (a == 0x3e00)   // coin_clear_w
		m6809_set_nmi(&s_cpu[0], 0);
	else
		oscar_irq_w(a);
}
static void coin_update(void)   // (Oscar: an input merger, any low, to the main CPU's NMI)
{
	const int state = (rbx_port_read(s_port_in[2]) & 0x07) != 0x07;
	if (state && !s_coin_state)
		m6809_set_nmi(&s_cpu[0], 1);
	s_coin_state = state;
}

// ---- video ----
static inline uint8_t rmc3(int v) { return (uint8_t)(0x0e * (v & 1) + 0x1f * ((v >> 1) & 1) + 0x43 * ((v >> 2) & 1) + 0x8f * ((v >> 3) & 1)); }

static void cobracom_colpri(void *u, uint32_t *colour, uint32_t *pri_mask)
{
	(void)u;
	*pri_mask = 0;   // (above both playfields)
	if ((*colour & 4) == 0)
		*pri_mask |= 0xcc;   // (GFX_PMASK_2: behind the second)
	*colour &= 3;
}
static void oscar_tile_cb(uint32_t *tile, uint32_t *colour, uint32_t *flags)
{
	(void)tile;
	if (*colour & 8)
		*flags |= BAC06_GROUP1;
	*colour &= 7;
}

static void d8o_render(void)
{
	const int oscar = s_var == V_OSCAR;
	for (int i = 0; i < (oscar ? 512 : 256); i++) {   // deco_rgb_decoder: xxxxBBBBGGGGRRRR (two bytes, big-endian)
		const int raw = s_snap->pal[2 * i + 1] | (s_snap->pal[2 * i] << 8);
		rbx_video.palette[i] = (uint32_t)(rmc3(raw & 15) << 16 | rmc3((raw >> 4) & 15) << 8 | rmc3((raw >> 8) & 15));
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	bac06_t *t0 = &s_snap->tile[0], *t1 = &s_snap->tile[1];
	const int flip = (t0->ctrlreg[0] & 0x80) != 0;
	t0->flip_screen = t1->flip_screen = flip;
	decmxc06_t spr = s_spr;
	spr.flip_screen = flip;
	if (oscar) {
		bac06_draw(t0, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, BAC06_LAYER1, 0, 0xff);
		decmxc06_draw(&spr, s_snap->spr, 0x400, bm, NULL, pitch, VIS_W, VIS_H, VIS_Y0, s_snap->frame);
		bac06_draw(t0, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, BAC06_LAYER0, 0, 0xff);
	} else {
		memset(s_pri_buf, 0, sizeof(s_pri_buf));
		bac06_draw(t0, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, BAC06_OPAQUE, 1, 0xff);
		bac06_draw(t1, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 2, 0xff);
		decmxc06_draw(&spr, s_snap->spr, 0x400, bm, s_pri_buf, pitch, VIS_W, VIS_H, VIS_Y0, s_snap->frame);
	}
	// the text layer: 32x32 of 8x8, pen 0 clear (Cobra-Command: 2 bpp, colour bits 13-15; Oscar: 3 bpp from 256,
	// colour bits 14-15)
	const uint8_t *txt = s_snap->txt;
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < VIS_W; x++) {
			int px = x, py = y + VIS_Y0;
			if (flip) {
				px = 255 - px;
				py = 255 - py;
			}
			const int o = ((py >> 3) * 32 + (px >> 3)) * 2;
			const int tile = txt[o + 1] | (txt[o] << 8);
			const uint32_t code = s_gfx8.count ? (uint32_t)(tile & 0xfff) % s_gfx8.count : 0;
			const uint8_t p = s_gfx8.pix[(code * 8 + (uint32_t)(py & 7)) * 8 + (uint32_t)(px & 7)];
			if (p)
				d[x] = (uint16_t)(oscar ? 256 + ((tile & 0xf000) >> 14) * 8 + p : ((tile & 0xe000) >> 13) * 4 + p);
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->tile, s_tile, sizeof(s_tile));
	memcpy(s_snap->txt, s_ram + s_txt, sizeof(s_snap->txt));
	memcpy(s_snap->pal, s_ram + s_pal, s_var == V_OSCAR ? 0x400 : 0x200);
	memcpy(s_snap->spr, s_sprbuf, sizeof(s_sprbuf));
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void d8o_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_sub_ram, 0, sizeof(s_sub_ram));
	memset(s_sprbuf, 0, sizeof(s_sprbuf));
	for (int i = 0; i < 2; i++) {
		memset(s_tile[i].vram, 0, sizeof(s_tile[i].vram));
		memset(s_tile[i].rowscroll, 0, sizeof(s_tile[i].rowscroll));
		memset(s_tile[i].colscroll, 0, sizeof(s_tile[i].colscroll));
		bac06_reset(&s_tile[i]);
	}
	set_bank(0);
	for (int c = 0; c < (s_var == V_OSCAR ? 2 : 1); c++) {
		m6809_reset(&s_cpu[c]);
		set_irq(c, 0);
		m6809_set_nmi(&s_cpu[c], 0);
	}
	s_coin_state = 0;
	memset(s_sram, 0, sizeof(s_sram));
	s_latch = 0;
	s_ym_irq[0] = s_ym_irq[1] = 0;
	m6502_reset(&s_snd);
	m6502_set_irq(&s_snd, 0);
	m6502_set_nmi(&s_snd, 0);
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	s_mix_n = s_ym2_n = 0;
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = s_snd_frac = s_ym1_frac = s_ym2_frac = 0;
}

static void d8o_exit(void);

static int d8o_init(const rbx_game *g)
{
	s_var = !strcmp(g->machine, "oscar") || !strcmp(g->machine, "oscarbl") ? V_OSCAR : V_COBRA;
	const int oscar = s_var == V_OSCAR, bl = !strcmp(g->machine, "oscarbl");
	uint32_t csize, t1size, t2size = 0, ssize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_bank_rom = rbx_region("mainbank", &s_bank_size);
	s_sub_rom = oscar ? rbx_region("sub", &s_sub_size) : NULL;
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *chars = rbx_region("char", &csize), *t1 = rbx_region("tiles1", &t1size),
	        *t2 = oscar ? NULL : rbx_region("tiles2", &t2size), *spr = rbx_region("sprites", &ssize);
	if (!s_rom || s_rom_size < 0x10000 || !s_bank_rom || s_bank_size < 0x4000 || !s_zrom || !chars || csize < 0x8000 ||
	    !t1 || t1size < 0x80000 || (!oscar && (!t2 || t2size < 0x80000)) || (oscar && (!s_sub_rom || s_sub_size < 0x10000)) ||
	    !spr || ssize < 0x80000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_bank_mask = (int)((s_bank_size / 0x4000 - 1) & 0xf);
	s_txt = 0x2000;
	s_spr_ram = oscar ? 0x3000 : 0x2800;
	s_pal = oscar ? 0x3800 : 0x3000;
	{	// charlayout_32k (2 bpp: planes at 0x4000, 0 bytes) / oscar_charlayout (3 bpp: 0x3000, 0x2000, 0x1000),
		// tiles (4096: planes at 0x60000 .. 0)
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 8);
			x16[i] = (uint32_t)(16 * 8 + i);
			x16[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 8);
		static const uint32_t pc[2] = { 0x4000 * 8, 0 }, po[3] = { 0x3000 * 8, 0x2000 * 8, 0x1000 * 8 };
		static const uint32_t pt[4] = { 0x60000 * 8, 0x40000 * 8, 0x20000 * 8, 0 };
		const rbx_gfx_layout l8 = { 8, 8, oscar ? 3 : 2, oscar ? po : pc, x8, y8, 8 * 8 };
		const rbx_gfx_layout lt = { 16, 16, 4, pt, x16, y16, 16 * 16 };
		if (rbx_gfx_decode(&s_gfx8, &l8, chars, 1024 * 8) || rbx_gfx_decode(&s_gfxt1, &lt, t1, 4096 * 32) ||
		    (!oscar && rbx_gfx_decode(&s_gfxt2, &lt, t2, 4096 * 32)) || rbx_gfx_decode(&s_gfxspr, &lt, spr, 4096 * 32)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			d8o_exit();
			return -1;
		}
	}
	memset(s_tile, 0, sizeof(s_tile));
	memset(&s_spr, 0, sizeof(s_spr));
	s_spr.gfx = &s_gfxspr;
	if (oscar) {   // set_gfx_region_wide(1, 1, 0): tiles1 from 384; the tile callback's groups and their transmasks
		s_tile[0].gfx16 = &s_gfxt1;
		s_tile[0].base16 = 384;
		s_tile[0].tile_cb = oscar_tile_cb;
		s_tile[0].transmask_on = 1;
		s_tile[0].fgmask[0] = 0xffff;
		s_tile[0].bgmask[0] = 0x0000;
		s_tile[0].fgmask[1] = 0x00ff;
		s_tile[0].bgmask[1] = 0xff00;
		s_spr.color_base = 0;
	} else {   // chip 0: (1, 1, 0) (tiles2, from 128); chip 1: (2, 2, 0) (tiles1, from 192)
		s_tile[0].gfx16 = &s_gfxt2;
		s_tile[0].base16 = 128;
		s_tile[1].gfx16 = &s_gfxt1;
		s_tile[1].base16 = 192;
		s_spr.color_base = 64;
		s_spr.colpri_cb = cobracom_colpri;
	}
	s_snap = calloc(1, sizeof(*s_snap));
	// sound: YM2203 1.5 MHz (SSG 0.20, FM 0.40), YM3812 (Oscar: YM3526) 3 MHz 0.80
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM1_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = 51;
	yc.gain[3] = 102;
	yc.irq = ym1_irq;
	int rc = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	memset(&yc, 0, sizeof(yc));
	yc.type = oscar ? RBX_YM3526 : RBX_YM3812;
	yc.clock = YM2_CLOCK;
	yc.gain[0] = 205;
	yc.irq = ym2_irq;
	rc = rc || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	if (!rc) {
		s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
		s_ym2_cap = (int)(rbx_ym_rate(1) / 50) + 256;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		s_ym2buf = (int32_t *)calloc((size_t)s_ym2_cap, sizeof(int32_t));
		rc = !s_mix || !s_ym2buf || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, oscar ? 512 : 256);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		d8o_exit();
		return -1;
	}
	m6502_init(&s_snd, oscar && !bl ? M6502_NMOS : M6502_R65C02);
	s_snd.deco222 = oscar && !bl;
	if (bl && s_zrom_size >= 0x18000) {   // oscarbl: an R65C02, the decrypted opcodes at the region's 0x10000
		s_snd.op_rom = s_zrom + 0x10000;
		s_snd.op_start = 0x8000;
	}
	for (int p = 0; p < 6; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = s_sram + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		if ((uint32_t)p * 256 + 256 <= s_zrom_size)
			s_snd.rpage[p] = s_zrom + p * 256;
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	for (int c = 0; c < 2; c++) {
		m6809_t *m = &s_cpu[c];
		m6809_init(m);
		m->ctx = c ? (void *)m : NULL;
		m->read = oscar ? oscar_r : cobra_r;
		m->write = oscar ? oscar_w : cobra_w;
		if (oscar) {   // (share1, the CPU's own page 0x0f, share2; the main CPU: the text, the sprites, the palette)
			for (int p = 0; p < 0x20; p++)
				m->rpage[p] = m->wpage[p] = c && p == 0x0f ? s_sub_ram : s_ram + p * 256;
			if (!c) {
				for (int p = 0x20; p < 0x28; p++)
					m->rpage[p] = m->wpage[p] = s_ram + p * 256;
				for (int p = 0x30; p < 0x3c; p++)
					m->rpage[p] = m->wpage[p] = s_ram + p * 256;
			}
		} else if (!c) {   // (RAM, text, sprites, palette; the BAC06s through the handlers)
			for (int p = 0; p < 0x32; p++)
				if (p < 0x08 || p >= 0x18)
					m->rpage[p] = m->wpage[p] = s_ram + p * 256;
		}
		for (int p = c ? 0x40 : 0x80; p < 0x100; p++)
			m->rpage[p] = (c ? s_sub_rom : s_rom) + p * 256;
	}
	s_port_in[0] = rbx_port_find("IN0");
	s_port_in[1] = rbx_port_find("IN1");
	s_port_in[2] = rbx_port_find("IN2");
	s_port_dsw[0] = rbx_port_find("DSW0");
	s_port_dsw[1] = rbx_port_find("DSW1");
	d8o_reset();
	return 0;
}

static void d8o_exit(void)
{
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix);
	free(s_ym2buf);
	s_mix = s_ym2buf = NULL;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfxt1);
	rbx_gfx_free(&s_gfxt2);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
}

static void d8o_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	const int oscar = s_var == V_OSCAR;
	if (oscar)
		coin_update();
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // Cobra-Command: screen_vblank on the NMI
			vblank_capture();
			if (!oscar)
				m6809_set_nmi(&s_cpu[0], 1);
		} else if (line == VIS_Y0 && !oscar)
			m6809_set_nmi(&s_cpu[0], 0);
		s_cpu_frac += (uint64_t)CPU_CLOCK * 1000;
		const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
		const int64_t cyc = (int64_t)(s_cpu_frac / den);
		s_cpu_frac -= (uint64_t)cyc * den;
		const int64_t t0 = s_cpu[0].cycles;
		m6809_run_until(&s_cpu[0], t0 + cyc);
		if (oscar)   // (MAME: a 6 kHz quantum; here a line)
			m6809_run_until(&s_cpu[1], s_cpu[1].cycles + cyc);
		sound_line();
	}
	sound_frame_end();
	s_frame++;
}

static int d8o_state(void *ctx, int load)
{
	uint8_t cpu[2][M6809_STATE_SIZE], snd[M6502_STATE_SIZE];
	if (!load) {
		for (int c = 0; c < 2; c++)
			memcpy(cpu[c], &s_cpu[c], M6809_STATE_SIZE);
		memcpy(snd, &s_snd, M6502_STATE_SIZE);
	}
	uint64_t fr[4] = { s_cpu_frac, s_snd_frac, s_ym1_frac, s_ym2_frac };
	int32_t misc[7] = { (int32_t)s_frame, s_bank, s_latch, s_ym_irq[0] | s_ym_irq[1] << 1, s_irq[0], s_irq[1],
	                    s_coin_state };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m6809", cpu[0], M6809_STATE_SIZE);
	if (s_var == V_OSCAR) {
		rc |= rbx_state_io(ctx, "m6809b", cpu[1], M6809_STATE_SIZE);
		rc |= rbx_state_io(ctx, "subram", s_sub_ram, sizeof(s_sub_ram));
	}
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "sprb", s_sprbuf, sizeof(s_sprbuf));
	rc |= rbx_state_io(ctx, "bac0", &s_tile[0], offsetof(bac06_t, gfx8));
	rc |= rbx_state_io(ctx, "bac1", &s_tile[1], offsetof(bac06_t, gfx8));
	rc |= rbx_state_io(ctx, "m6502", snd, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		memcpy(&s_snd, snd, M6502_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_ym1_frac = fr[2];
		s_ym2_frac = fr[3];
		s_frame = (uint32_t)misc[0];
		for (int c = 0; c < (s_var == V_OSCAR ? 2 : 1); c++)
			memcpy(&s_cpu[c], cpu[c], M6809_STATE_SIZE);
		set_bank(misc[1]);
		s_latch = (uint8_t)misc[2];
		s_ym_irq[0] = misc[3] & 1;
		s_ym_irq[1] = (misc[3] >> 1) & 1;
		s_irq[0] = misc[4];
		s_irq[1] = misc[5];
		s_coin_state = misc[6];
	}
	return rc ? -1 : 0;
}

static int d8o_save(void *ctx) { return d8o_state(ctx, 0); }
static int d8o_load(void *ctx) { return d8o_state(ctx, 1); }
static int d8o_supported(const char *machine)
{
	return !strcmp(machine, "cobracom") || !strcmp(machine, "oscar") || !strcmp(machine, "oscarbl");
}

const rbx_driver rbx_drv_dec8o = { "DECO dec8", d8o_init, d8o_exit, d8o_reset, d8o_frame, d8o_save, d8o_load,
                                   d8o_supported, d8o_render, NULL };
