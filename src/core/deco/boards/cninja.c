/*  Data East DE-0355 family: Caveman Ninja / Joe & Mac, Edward Randy, Robocop 2. Ported from MAME
    src/mame/dataeast/cninja.cpp, cninja_v.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail).
    68000 at 12 MHz with the IRQ controller (machine/deco_irq.h: raster IRQs 3 / 4, vblank IRQ 5), the 104 protection /
    I/O chip, two tilemap chips (the first one's second playfield split by pens over and under the sprites), sprite chip
    52 (priority), the sound board (machine/deco16snd.c: HuC6280, YM2203, YM2151 banking the second OKI). The screen is
    256 pixels wide, drawn with each line's registers. Edward Randy: the 146 at two places; Robocop 2: the 146, a 68000
    at 14 MHz, a 320 pixel screen, a priority register (the second chip's playfields as one 8 bpp picture).
    The bootlegs: a Z80 with a YM2151 and an OKI in place of the sound board (stoneage: the 104's latch as NMI;
    cninjabl2: the OKI alone with a banked ROM, the latch as IRQ, the sprite DMA on a read; cninjabl: no 104, the
    inputs and a latch of its own, its sprites in another format at another place, the graphics inverted).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../machine/decocrpt.h"
#include "../machine/deco146.h"
#include "../machine/deco16snd.h"
#include "../machine/deco_irq.h"
#include "../video/deco16ic.h"
#include "../video/decospr.h"

#include <stdlib.h>
#include <string.h>

#define LINES 274
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 320                // (the widest; the machine's own: s_m->vis_w)

// the machines: their clocks, screen and map
enum { M_CNINJA, M_EDRANDY, M_ROBOCOP2 };
typedef struct {
	const char *machine;
	uint32_t cpu_clock, frame_mhz;
	int vis_w;
	uint32_t rom_end, prot, prot2, prot_sub, irq, pal, ram, spr, dma;
	int is104, magic_xor;
} cn_map_t;
static const cn_map_t s_maps[] = {
	// set_raw(24 MHz / 4, 376, 0, 256, 274, 8, 248)
	{ "cninja", 12000000, 58237, 256, 0xbffff, 0x1bc000, 0, 0x1bc000, 0x190000, 0x19c000, 0x184000, 0x1a4000, 0x1b4000, 1, 1 },
	{ "edrandy", 12000000, 58237, 256, 0xfffff, 0x198000, 0x1a0000, 0, 0x1a4000, 0x188000, 0x194000, 0x1bc000, 0x1ac000, 0, 0 },
	// set_raw(28 MHz / 4, 442, 0, 320, 274, 8, 248)
	{ "robocop2", 14000000, 57800, 320, 0xfffff, 0x18c000, 0, 0x18c000, 0x1b0000, 0x1a8000, 0x1b8000, 0x180000, 0x198000, 0, 1 },
};
static const cn_map_t *s_m;
static int s_mi;
enum { B_NONE, B_STONEAGE, B_CNINJABL2, B_CNINJABL };
static int s_boot;
static uint16_t s_ram_ext[0x8000];        // the bootlegs' RAM at 0x180000 (cninjabl 32 KB, cninjabl2 64 KB)
static uint16_t s_priority;               // robocop2_priority_w
static int s_port_dsw3;

static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint16_t s_ram[0x2000];            // (16 KB)
static uint16_t s_spriteram[0x800], s_spriteram_buf[0x400];
static uint16_t s_palram[0x1000];         // xBGR_888, 2048 entries of two words
static uint16_t s_rowscroll[4][0x800];
static deco_irq_t s_irq;
static deco16ic_t s_tile[2];
static decospr_t s_spr;
static deco146_t s_prot;
static rbx_gfx s_gfx8, s_gfx16a, s_gfx16b, s_gfxspr;
static int s_port_in, s_port_dsw, s_port_sys;
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac;
static uint8_t s_pri_buf[VIS_W * VIS_H];
static struct {
	deco16ic_t tile[2];
	uint16_t rowscroll[4][0x800], spriteram[0x400], palram[0x1000], priority;
	uint32_t frame;
} *s_snap;

// ---- the bootlegs' sound: Z80 3.58 MHz, YM2151 3.58 MHz 0.45 per output (not cninjabl2), OKI 1 MHz ----
static z80 s_z80;
static uint8_t s_zram[0x800];
static const uint8_t *s_zrom;
static uint32_t s_zrom_size;
static const uint8_t *s_zmap[256];
static okim6295_t s_boki;
static uint8_t *s_bokibuf;                // cninjabl2: 0x00000-0x2ffff oki1, 0x30000-0x3ffff the bank of oki2
static const uint8_t *s_boki2;
static uint32_t s_boki2_size;
static int s_bokibank;
static uint8_t s_blatch;                  // cninjabl: the generic latch
static int s_blatch_pending;
static int s_bym_irq, s_bint, s_b104_irq;
static uint64_t s_z80_frac, s_ym_frac;
static uint32_t s_oki_acc;
static int32_t s_oki_hold;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
#define BZ80_CLOCK 3579545
#define BYM_CLOCK (32220000 / 9)

static void bokibank_apply(int bank)   // cninjabl2_oki_bank_w
{
	s_bokibank = bank & 7;
	if (s_boki2 && (uint32_t)s_bokibank * 0x10000u + 0x10000 <= s_boki2_size)
		memcpy(s_bokibuf + 0x30000, s_boki2 + s_bokibank * 0x10000, 0x10000);
}

static uint8_t bz80_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0x8800) return s_zram[a - 0x8000];
	if ((a == 0x8800 || a == 0x8801) && s_boot != B_CNINJABL2) return rbx_ym_read(0, a & 1);
	if (a == 0x9800) return oki_read(&s_boki);
	if (a == 0xa000) {
		if (s_boot == B_CNINJABL) {   // generic_latch_8 read: the pending flag (NMI) cleared
			s_blatch_pending = 0;
			return s_blatch;
		}
		return deco146_soundlatch_r(&s_prot);
	}
	return 0;
}
static void bz80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8800) s_zram[a - 0x8000] = v;
	else if ((a == 0x8800 || a == 0x8801) && s_boot != B_CNINJABL2) rbx_ym_write(0, a & 1, v);
	else if (a == 0x9000 && s_boot == B_CNINJABL2) bokibank_apply(v);
	else if (a == 0x9800) oki_write(&s_boki, v);
}
static uint8_t bz80_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0; }
static void bz80_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }
static void bz80_hook(z80 *z)
{
	z->read_byte = bz80_read;
	z->write_byte = bz80_write;
	z->port_in = bz80_in;
	z->port_out = bz80_out;
	z->userdata = NULL;
	z->read_map = s_zmap;
}
static void bym_irq(int state) { s_bym_irq = state; }

static void bsound_line(void)
{
	s_bint = s_boot == B_CNINJABL2 ? s_b104_irq : s_bym_irq;   // (IRQ0: the YM, or cninjabl2's latch)
	z80_run(&s_z80, s_z80.cyc + (unsigned long)rbx_line_cycles(&s_z80_frac, BZ80_CLOCK, s_m->frame_mhz, LINES), &s_bint,
	        0);
	if (s_boot != B_CNINJABL2) {   // the YM's rate out, the OKI held
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&s_ym_frac, BYM_CLOCK, s_m->frame_mhz, LINES);
		const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
		const uint32_t yr = rbx_ym_rate(0);
		for (int i = 0; i < n; i++) {
			s_oki_acc += s_boki.rate;
			while (s_oki_acc >= yr) {
				s_oki_acc -= yr;
				s_oki_hold = oki_sample(&s_boki);
			}
			s_mix[s_mix_n + i] += s_oki_hold;
		}
		s_mix_n += n;
	} else {   // the OKI's rate out
		s_ym_frac += (uint64_t)s_boki.rate * 1000;
		const uint64_t den = (uint64_t)s_m->frame_mhz * LINES;
		while (s_ym_frac >= den) {
			s_ym_frac -= den;
			rbx_audio_put(oki_sample(&s_boki));
		}
	}
}

static void bsound_frame_end(void)
{
	if (s_boot == B_CNINJABL2)
		return;
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

// ---- the 104's ports and latch ----
static uint16_t port_a(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_in); }
static uint16_t port_b(void *u)
{
	(void)u;
	const int vbl = s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H;
	rbx_port_custom(s_port_sys, 0x0008, vbl ? 0x0008 : 0);
	return (uint16_t)rbx_port_read(s_port_sys);
}
static uint16_t port_c(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_dsw); }
static void latch_irq(void *u, int state)
{
	(void)u;
	if (s_boot == B_STONEAGE) {   // (the Z80's NMI: an edge)
		if (state && !s_b104_irq)
			z80_gen_nmi(&s_z80);
		s_b104_irq = state;
	} else if (s_boot == B_CNINJABL2)
		s_b104_irq = state;
	else
		deco16snd_latch_irq(state);
}
static uint8_t latch_r(void) { return deco146_soundlatch_r(&s_prot); }
static void ym_port(uint8_t data) { deco16snd_oki(1)->bank = (data & 1) * 0x40000u; }   // sound_bankswitch_w
static void irq_line(int level, int state) { m68k_bus_set_line(level, state); }
static void irq_raster1(void *u, int state) { (void)u; irq_line(3, state); }
static void irq_raster2(void *u, int state) { (void)u; irq_line(4, state); }
static void irq_vblank(void *u, int state) { (void)u; irq_line(5, state); }

// ---- bus ----
// ioprot_r / w: the address the 146 sees (the bitswap: bits 14-17 to 11-14, 11-13 dropped)
static uint16_t prot_addr(uint32_t a)
{
	const uint32_t real = a - s_m->prot_sub;
	return (uint16_t)((((real >> 14) & 0xf) << 11) | (real & 0x7fe));
}
static int is_prot(uint32_t a)
{
	if (s_boot == B_CNINJABL)   // (no 104)
		return 0;
	return (a >= s_m->prot && a < s_m->prot + 0x4000) || (s_m->prot2 && a >= s_m->prot2 && a < s_m->prot2 + 0x4000);
}

static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (is_prot(a)) return deco146_read(&s_prot, prot_addr(a), NULL);
	if (s_boot == B_CNINJABL) {   // cninjabl_map's ports
		if (a == 0x17ff22) return (uint16_t)rbx_port_read(s_port_dsw);
		if (a == 0x17ff28) return port_b(NULL);
		if (a == 0x17ff2c) return (uint16_t)rbx_port_read(s_port_in);
	}
	if (s_boot == B_CNINJABL2 && a == s_m->dma) {   // cninjabl2_sprite_dma_r
		memcpy(s_spriteram_buf, s_spriteram, sizeof(s_spriteram_buf));
		return 0;
	}
	if (s_mi == M_ROBOCOP2 && a == 0x1f8000) return (uint16_t)rbx_port_read(s_port_dsw3);
	if (a >= s_m->irq && a < s_m->irq + 8) {
		const int vbl = s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H;
		return 0xff00 | deco_irq_r(&s_irq, (int)((a >> 1) & 3), vbl, 0);
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
	if (a >= 0x140000 && a < 0x140010)
		deco16ic_control_w(&s_tile[0], (int)((a >> 1) & 7), v, mask);
	else if (a >= 0x150000 && a < 0x150010)
		deco16ic_control_w(&s_tile[1], (int)((a >> 1) & 7), v, mask);
	else if (is_prot(a))
		deco146_write(&s_prot, prot_addr(a), v, mask, NULL);
	else if (a >= s_m->irq && a < s_m->irq + 8) {
		if (mask & 0xff)
			deco_irq_w(&s_irq, (int)((a >> 1) & 3), (uint8_t)v);
	} else if (a == s_m->dma)   // the sprite DMA
		memcpy(s_spriteram_buf, s_spriteram, sizeof(s_spriteram_buf));
	else if (s_mi == M_ROBOCOP2 && a == 0x1f0000)
		s_priority = (uint16_t)((s_priority & ~mask) | (v & mask));
	else if (s_boot == B_CNINJABL && a == 0x17ff2a && (mask & 0xff)) {   // the latch (its pending flag: NMI)
		s_blatch = (uint8_t)v;
		if (!s_blatch_pending)
			z80_gen_nmi(&s_z80);
		s_blatch_pending = 1;
	}
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static int bank_cb(int bank) { return (bank & 0xf0) ? 0x0000 : 0x1000; }   // (only 2 banks)
static int robocop2_bank_cb(int bank) { return (bank & 0x30) << 8; }
static uint16_t robocop2_mix(uint16_t p, uint16_t p2) { return (uint16_t)((p & 0x70f) | ((p2 & 0xf) << 4)); }
static uint32_t pri_cb(uint16_t x, int extpri)
{
	(void)extpri;
	switch (x & 0xc000) {
	case 0x4000: return 0xf0;
	case 0x8000: case 0xc000: return 0xf0 | 0xcc;
	default: return 0;
	}
}

// cninjabl_draw_sprites: the bootleg's list (code, y, x words; 0x180 in y ends it), drawn from its end
static void cninjabl_draw_sprites(const uint16_t *spr, uint16_t *bm, uint8_t *pb, int pitch, int w, int flip,
                                  uint32_t frame)
{
	int endoffs = 0x400 - 4;
	for (int offs = 0; offs < 0x400 - 4; offs += 4)
		if (spr[offs + 1] == 0x180) {
			endoffs = offs;
			break;
		}
	for (int offs = endoffs; offs >= 0; offs -= 4) {
		int sprite = spr[offs + 0], y = spr[offs + 1];
		if (!sprite)
			continue;
		int x = spr[offs + 2];
		const uint32_t pri = pri_cb((uint16_t)x, 0);
		if ((y & 0x1000) && (frame & 1))   // (flash)
			continue;
		const int colour = (x >> 9) & 0x1f;
		int fx = (y >> 13) & 1, fy = (y >> 14) & 1;
		int multi = (1 << ((y & 0x0600) >> 9)) - 1;   // 1x, 2x, 4x, 8x height
		y -= multi * 16;
		y += 4;
		x &= 0x1ff;
		y &= 0x1ff;
		if (x >= 256) x -= 512;
		if (y >= 256) y -= 512;
		x = 240 - x;
		y = 240 - y;
		int inc, mult;
		if (fy)
			inc = -1;
		else {
			sprite += multi;
			inc = 1;
		}
		if (flip) {
			y = 240 - y;
			x = 240 - x;
			fx = !fx;
			fy = !fy;
			mult = 16;
		} else
			mult = -16;
		for (; multi >= 0; multi--)
			rbx_gfx_prio_transpen(&s_gfxspr, rbx_wrap((uint32_t)(sprite - multi * inc), s_gfxspr.count),
			                      (uint16_t)(768 + colour * 16), fx, fy, x, y + mult * multi - VIS_Y0, bm, pb, pitch, w,
			                      VIS_H, pri);
	}
}

static void rg_render(void)
{
	for (int i = 0; i < 0x800; i++) {   // xBGR_888 (the entry's two words, big-endian)
		const uint32_t p = (uint32_t)s_snap->palram[2 * i] << 16 | s_snap->palram[2 * i + 1];
		rbx_video.palette[i] = (p & 0xff) << 16 | (p & 0xff00) | ((p >> 16) & 0xff);
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	deco16ic_t *t0 = &s_snap->tile[0], *t1 = &s_snap->tile[1];
	if (s_boot == B_CNINJABL2 || s_boot == B_CNINJABL) {   // set_enable: "force layers to be enabled"
		deco16ic_t *t = s_boot == B_CNINJABL ? t1 : t0;
		for (int i = 0; i < 2; i++)
			t->tm[i].control0 |= 0x80;
		for (int y = 0; y < 256; y++)
			t->lines[y].control0[0] |= 0x80, t->lines[y].control0[1] |= 0x80;
	}
	deco16ic_update(t0);
	deco16ic_update(t1);
	decospr_t spr = s_spr;
	spr.flip_screen = (deco16ic_control_r(t0, 0) >> 7) & 1;
	memset(s_pri_buf, 0, sizeof(s_pri_buf));
	const uint16_t *rs0 = s_snap->rowscroll[0], *rs1 = s_snap->rowscroll[1], *rs2 = s_snap->rowscroll[2],
	               *rs3 = s_snap->rowscroll[3];
	const int w = s_m->vis_w;
	uint8_t *pb = s_pri_buf;
	if (s_mi == M_CNINJA) {
		deco16ic_draw(t1, 1, rs3, bm, pb, pitch, w, VIS_Y0, 0, VIS_H, 1, 1, 0xff);
		deco16ic_draw(t1, 0, rs2, bm, pb, pitch, w, VIS_Y0, 0, VIS_H, 0, 2, 0xff);
		t0->draw_pens = 0x00fe;   // TILEMAP_DRAW_LAYER1 (transmask 0xff01: pens 1-7)
		deco16ic_draw(t0, 1, rs1, bm, pb, pitch, w, VIS_Y0, 0, VIS_H, 0, 2, 0xff);
		t0->draw_pens = 0xff00;   // TILEMAP_DRAW_LAYER0 (transmask 0x00ff: pens 8-15)
		deco16ic_draw(t0, 1, rs1, bm, pb, pitch, w, VIS_Y0, 0, VIS_H, 0, 4, 0xff);
		t0->draw_pens = 0;
	} else if (s_mi == M_EDRANDY) {
		deco16ic_draw(t1, 1, rs3, bm, pb, pitch, w, VIS_Y0, 0, VIS_H, 1, 1, 0xff);
		deco16ic_draw(t1, 0, rs2, bm, pb, pitch, w, VIS_Y0, 0, VIS_H, 0, 2, 0xff);
		deco16ic_draw(t0, 1, rs1, bm, pb, pitch, w, VIS_Y0, 0, VIS_H, 0, 4, 0xff);
	} else {   // screen_update_robocop2: pen 0x200, the switchable priority
		const uint16_t pr = s_snap->priority;
		for (int y = 0; y < VIS_H; y++)
			for (int x = 0; x < w; x++)
				bm[y * pitch + x] = 0x200;
		if (!(pr & 4))
			deco16ic_draw(t1, 1, rs3, bm, pb, pitch, w, VIS_Y0, 0, VIS_H, 1, 1, 0xff);
		if (pr & 8) {
			deco16ic_draw(t0, 1, rs1, bm, pb, pitch, w, VIS_Y0, 0, VIS_H, 0, 2, 0xff);
			if (pr & 4)
				deco16ic_draw_combine(t1, rs2, bm, pb, pitch, w, VIS_Y0, 0, VIS_H, 0, 4, robocop2_mix);
			else
				deco16ic_draw(t1, 0, rs2, bm, pb, pitch, w, VIS_Y0, 0, VIS_H, 0, 4, 0xff);
		} else {
			if (pr & 4)
				deco16ic_draw_combine(t1, rs2, bm, pb, pitch, w, VIS_Y0, 0, VIS_H, 0, 2, robocop2_mix);
			else
				deco16ic_draw(t1, 0, rs2, bm, pb, pitch, w, VIS_Y0, 0, VIS_H, 0, 2, 0xff);
			deco16ic_draw(t0, 1, rs1, bm, pb, pitch, w, VIS_Y0, 0, VIS_H, 0, 4, 0xff);
		}
	}
	if (s_boot == B_CNINJABL)
		cninjabl_draw_sprites(s_snap->spriteram, bm, pb, pitch, w, spr.flip_screen, s_snap->frame);
	else
		decospr_draw(&spr, s_snap->spriteram, 0x400, bm, pb, pitch, w, VIS_H, VIS_Y0, w > 256, s_snap->frame);
	deco16ic_draw(t0, 0, rs0, bm, pb, pitch, w, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->tile, s_tile, sizeof(s_tile));
	memcpy(s_snap->rowscroll, s_rowscroll, sizeof(s_rowscroll));
	memcpy(s_snap->spriteram, s_spriteram_buf, sizeof(s_spriteram_buf));
	memcpy(s_snap->palram, s_palram, sizeof(s_palram));
	s_snap->priority = s_priority;
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void rg_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_spriteram_buf, 0, sizeof(s_spriteram_buf));
	memset(s_palram, 0, sizeof(s_palram));
	memset(s_rowscroll, 0, sizeof(s_rowscroll));
	deco_irq_reset(&s_irq);
	s_priority = 0;
	for (int i = 0; i < 2; i++) {
		memset(s_tile[i].vram, 0, sizeof(s_tile[i].vram));
		deco16ic_reset(&s_tile[i]);
	}
	deco146_reset(&s_prot);
	memset(s_ram_ext, 0, sizeof(s_ram_ext));
	if (s_boot) {
		memset(s_zram, 0, sizeof(s_zram));
		z80_init(&s_z80);
		bz80_hook(&s_z80);
		s_z80.int_data = 0xff;
		if (s_boot != B_CNINJABL2)
			rbx_ym_reset(0);
		oki_reset(&s_boki);
		if (s_boot == B_CNINJABL2)
			bokibank_apply(0);
		s_blatch = 0;
		s_blatch_pending = s_bym_irq = s_bint = s_b104_irq = 0;
		s_z80_frac = s_ym_frac = 0;
		s_oki_acc = 0;
		s_oki_hold = 0;
		s_mix_n = 0;
	} else
		deco16snd_reset();
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = 0;
}

static void rg_exit(void);

static int rg_init(const rbx_game *g)
{
	s_mi = M_CNINJA;
	for (int k = 0; k < (int)(sizeof(s_maps) / sizeof(s_maps[0])); k++)
		if (!strcmp(s_maps[k].machine, g->machine))
			s_mi = k;
	s_m = &s_maps[s_mi];
	s_boot = !strcmp(g->machine, "stoneage") ? B_STONEAGE : !strcmp(g->machine, "cninjabl2") ? B_CNINJABL2
	       : !strcmp(g->machine, "cninjabl") ? B_CNINJABL : B_NONE;
	uint32_t snd_size, csize, t1s, t2s, ssize, o1s, o2s = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *snd = rbx_region("audiocpu", &snd_size), *chars = rbx_region("chars", &csize),
	        *t1 = rbx_region("tiles1", &t1s), *t2 = rbx_region("tiles2", &t2s), *spr = rbx_region("sprites1", &ssize),
	        *oki1 = rbx_region("oki1", &o1s), *oki2 = rbx_region("oki2", &o2s);
	if (!s_rom || !snd || !chars || !t1 || !t2 || !spr || !oki1 || ((!s_boot || s_boot == B_CNINJABL2) && !oki2)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (s_boot == B_CNINJABL) {   // ROMREGION_INVERT (after the copies from gfxtemp, as MAME's loader)
		uint8_t *inv[3] = { chars, t1, t2 };
		const uint32_t n[3] = { csize, t1s, t2s };
		for (int r = 0; r < 3; r++)
			for (uint32_t i = 0; i < n[r]; i++)
				inv[r][i] ^= 0xff;
	}
	{	// charlayout, tilelayout (RGN_FRAC(1,2))
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
		const uint32_t pc[4] = { ch + 8, ch, 8, 0 }, p1[4] = { h1 + 8, h1, 8, 0 }, p2[4] = { h2 + 8, h2, 8, 0 };
		const uint32_t ps[4] = { hs + 8, hs, 8, 0 };
		const rbx_gfx_layout l8 = { 8, 8, 4, pc, x8, y8, 16 * 8 };
		const rbx_gfx_layout la = { 16, 16, 4, p1, x16, y16, 64 * 8 }, lb = { 16, 16, 4, p2, x16, y16, 64 * 8 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, x16, y16, 64 * 8 };
		if (rbx_gfx_decode(&s_gfx8, &l8, chars, csize / 2) || rbx_gfx_decode(&s_gfx16a, &la, t1, t1s / 2) ||
		    rbx_gfx_decode(&s_gfx16b, &lb, t2, t2s / 2) || rbx_gfx_decode(&s_gfxspr, &ls, spr, ssize / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			rg_exit();
			return -1;
		}
	}
	memset(s_tile, 0, sizeof(s_tile));
	static const int banks[2][2] = { { 0x00, 0x10 }, { 0x00, 0x30 } };
	for (int c = 0; c < 2; c++) {
		for (int i = 0; i < 2; i++) {
			s_tile[c].tm[i].size = DECO_64x32;
			s_tile[c].tm[i].col_bank = banks[c][i];
			s_tile[c].tm[i].col_mask = 0x0f;
			s_tile[c].tm[i].bank_cb = s_mi == M_ROBOCOP2 ? (c || i ? robocop2_bank_cb : NULL) : c ? bank_cb : NULL;
		}
		s_tile[c].gfx8 = &s_gfx8;
	}
	s_tile[0].gfx16 = &s_gfx16a;
	s_tile[1].gfx16 = &s_gfx16b;
	s_tile[1].base16 = 512;   // (gfx_cninja: tiles2 from colour 512)
	if (s_boot == B_STONEAGE) {   // VIDEO_START stoneage: set_scrolldx(1, 0, -10, -10), set_scrolldx(0, 1, 2, 2)
		s_tile[0].dx[1][0] = -10;     // (pf2 16x16)
		s_tile[0].dx[0][1] = 2;       // (pf1 8x8)
		// set_scrolldx(3, ...): "pf4", the second chip's pf2, from when one device held both chips (since then
		// past the first chip's two playfields, no effect in MAME): as meant, the picture then lines up with cninja's
		s_tile[1].dx[1][0] = -10;
	}
	memset(&s_spr, 0, sizeof(s_spr));
	s_spr.gfx = &s_gfxspr;
	s_spr.color_base = 768;
	s_spr.pri_cb = pri_cb;
	memset(&s_irq, 0, sizeof(s_irq));
	s_irq.vis_top = VIS_Y0;
	s_irq.vis_bottom = VIS_Y0 + VIS_H - 1;
	s_irq.raster1 = irq_raster1;
	s_irq.raster2 = irq_raster2;
	s_irq.vblank = irq_vblank;
	deco146_init(&s_prot, s_m->is104);
	s_prot.magic_read_xor_on = s_m->magic_xor;
	s_prot.port_a = port_a;
	s_prot.port_b = port_b;
	s_prot.port_c = port_c;
	s_prot.soundlatch_irq = latch_irq;
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_snap) {
		rbx_fail("%s: out of memory", g->set);
		rg_exit();
		return -1;
	}
	// sound: HuC6280 4.03 MHz, YM2203 4.03 MHz 0.60, YM2151 3.58 MHz 0.45 (banking the second OKI), OKI 1 MHz 0.75,
	// OKI 2 MHz 0.60, one speaker
	deco16snd_cfg sc;
	memset(&sc, 0, sizeof(sc));
	sc.cpu_clock = 32220000 / 8;
	sc.ym_clock = 32220000 / 9;
	sc.ym_gain[0] = sc.ym_gain[1] = 115;
	sc.ym2203_clock = 32220000 / 8;
	sc.ym2203_gain[0] = sc.ym2203_gain[1] = sc.ym2203_gain[2] = sc.ym2203_gain[3] = 154;
	sc.oki_rom[0] = oki1;
	sc.oki_size[0] = o1s;
	sc.oki_clock[0] = 32220000 / 32;
	sc.oki_gain[0] = 192;
	sc.oki_rom[1] = oki2;
	sc.oki_size[1] = o2s;
	sc.oki_clock[1] = 32220000 / 16;
	sc.oki_gain[1] = 154;
	sc.ym_port = ym_port;
	sc.latch_r = latch_r;
	int src;
	if (s_boot) {
		s_zrom = snd;
		s_zrom_size = snd_size;
		memset(s_zmap, 0, sizeof(s_zmap));
		for (uint32_t p = 0; p < 0x80 && p * 256 + 256 <= snd_size; p++)
			s_zmap[p] = snd + p * 256;
		s_boki2 = oki2;
		s_boki2_size = o2s;
		if (s_boot == B_CNINJABL2) {   // OKI 1 MHz pin 7 low, 1.0; its 0x30000-0x3ffff banked
			s_bokibuf = (uint8_t *)calloc(1, 0x40000);
			src = !s_bokibuf;
			if (!src) {
				memcpy(s_bokibuf, oki1, o1s < 0x30000 ? o1s : 0x30000);
				oki_init(&s_boki, 32220000 / 32, 0, s_bokibuf, 0x40000, 256);
				src = rbx_audio_setup(s_boki.rate);
			}
		} else {   // (OKI 0.75)
			oki_init(&s_boki, 32220000 / 32, 1, oki1, o1s, 192);
			rbx_ym_config yc;
			memset(&yc, 0, sizeof(yc));
			yc.type = RBX_YM2151;
			yc.clock = BYM_CLOCK;
			yc.gain[0] = yc.gain[1] = 115;
			yc.irq = bym_irq;
			src = rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
			if (!src) {
				s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
				s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
				src = !s_mix || rbx_audio_setup(rbx_ym_rate(0));
			}
		}
	} else
		src = deco16snd_init(snd, snd_size, &sc, s_m->frame_mhz, LINES);
	if (src || rbx_video_setup(s_m->vis_w, VIS_H, 0x800)) {
		rbx_fail("%s: out of memory", g->set);
		rg_exit();
		return -1;
	}
	s_port_in = rbx_port_find("INPUTS");
	s_port_dsw = rbx_port_find("DSW");
	s_port_sys = rbx_port_find("SYSTEM");
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, s_m->rom_end, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x144000, 0x144fff, s_tile[0].vram[0], 0x1000, 1, 1);
	m68k_bus_map(0x146000, 0x146fff, s_tile[0].vram[1], 0x1000, 1, 1);
	m68k_bus_map(0x14c000, 0x14cfff, s_rowscroll[0], 0x1000, 1, 1);
	m68k_bus_map(0x14e000, 0x14efff, s_rowscroll[1], 0x1000, 1, 1);
	m68k_bus_map(0x154000, 0x154fff, s_tile[1].vram[0], 0x1000, 1, 1);
	m68k_bus_map(0x156000, 0x156fff, s_tile[1].vram[1], 0x1000, 1, 1);
	m68k_bus_map(0x15c000, 0x15cfff, s_rowscroll[2], 0x1000, 1, 1);
	m68k_bus_map(0x15e000, 0x15efff, s_rowscroll[3], 0x1000, 1, 1);
	if (s_boot == B_CNINJABL2)   // cninjabl2_map: 0x180000-0x18ffff over the RAM at 0x184000
		m68k_bus_map(0x180000, 0x18ffff, s_ram_ext, 0x10000, 1, 1);
	else if (s_boot == B_CNINJABL)
		m68k_bus_map(0x180000, 0x187fff, s_ram_ext, 0x8000, 1, 1);
	else
		m68k_bus_map(s_m->ram, s_m->ram + 0x3fff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(s_m->pal, s_m->pal + 0x1fff, s_palram, sizeof(s_palram), 1, 1);
	if (s_boot == B_CNINJABL)   // the bootleg sprite RAM
		m68k_bus_map(0x138000, 0x1387ff, s_spriteram, 0x800, 1, 1);
	else
		m68k_bus_map(s_m->spr, s_m->spr + 0xfff, s_spriteram, sizeof(s_spriteram), 1, 1);
	s_port_dsw3 = rbx_port_find("DSW3");
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	rg_reset();
	return 0;
}

static void rg_exit(void)
{
	if (!s_boot)
		deco16snd_exit();
	else if (s_boot != B_CNINJABL2)
		rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	free(s_bokibuf);
	s_bokibuf = NULL;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16a);
	rbx_gfx_free(&s_gfx16b);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
}

static void rg_frame(void)
{
	rbx_audio.frame_mhz = s_m->frame_mhz;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		deco_irq_line(&s_irq, line);
		if (line >= VIS_Y0 && line < VIS_Y0 + VIS_H)
			for (int i = 0; i < 2; i++)
				deco16ic_latch_line(&s_tile[i], line);
		m68k_execute(rbx_line_cycles(&s_cpu_frac, s_m->cpu_clock, s_m->frame_mhz, LINES));
		if (s_boot)
			bsound_line();
		else
			deco16snd_line();
	}
	if (s_boot)
		bsound_frame_end();
	else
		deco16snd_frame_end();
	s_frame++;
}

static int rg_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr = s_cpu_frac;
	uint32_t misc[2] = { s_frame, s_priority };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "sprb", s_spriteram_buf, sizeof(s_spriteram_buf));
	rc |= rbx_state_io(ctx, "palram", s_palram, sizeof(s_palram));
	rc |= rbx_state_io(ctx, "irqc", &s_irq, DECO_IRQ_STATE_SIZE);
	rc |= rbx_state_io(ctx, "rows", s_rowscroll, sizeof(s_rowscroll));
	for (int i = 0; i < 2; i++) {
		rc |= rbx_state_io(ctx, i ? "vram2" : "vram1", s_tile[i].vram, sizeof(s_tile[i].vram));
		rc |= rbx_state_io(ctx, i ? "tctl2" : "tctl1", s_tile[i].control, sizeof(s_tile[i].control));
	}
	rc |= rbx_state_io(ctx, "prot", &s_prot, DECO146_STATE_SIZE);
	z80 z = s_z80;
	uint64_t bfr[2] = { s_z80_frac, s_ym_frac };
	int32_t bm[8] = { s_blatch, s_blatch_pending, s_bym_irq, s_b104_irq, s_bokibank, (int32_t)s_oki_acc, s_oki_hold, 0 };
	if (s_boot) {
		rc |= rbx_state_io(ctx, "ramx", s_ram_ext, sizeof(s_ram_ext));
		rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
		rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
		rc |= rbx_state_io(ctx, "oki", s_boki.v, sizeof(s_boki.v) + sizeof(int));
		if (s_boot != B_CNINJABL2)
			rc |= rbx_ym_state(0, ctx, load);
		rc |= rbx_state_io(ctx, "bfr", bfr, sizeof(bfr));
		rc |= rbx_state_io(ctx, "bmisc", bm, sizeof(bm));
	} else
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
		s_priority = (uint16_t)misc[1];
		if (s_boot) {
			bz80_hook(&z);
			s_z80 = z;
			s_z80_frac = bfr[0];
			s_ym_frac = bfr[1];
			s_blatch = (uint8_t)bm[0];
			s_blatch_pending = bm[1];
			s_bym_irq = bm[2];
			s_b104_irq = bm[3];
			if (s_boot == B_CNINJABL2)
				bokibank_apply(bm[4]);
			s_oki_acc = (uint32_t)bm[5];
			s_oki_hold = bm[6];
		}
		for (int c = 0; c < 2; c++)
			for (int i = 0; i < 8; i++)
				deco16ic_control_w(&s_tile[c], i, s_tile[c].control[i], 0xffff);
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int rg_save(void *ctx) { return rg_state(ctx, 0); }
static int rg_load(void *ctx) { return rg_state(ctx, 1); }
static int rg_supported(const char *machine)
{
	return !strcmp(machine, "cninja") || !strcmp(machine, "edrandy") || !strcmp(machine, "robocop2") ||
	       !strcmp(machine, "mutantf") || !strcmp(machine, "stoneage") || !strcmp(machine, "cninjabl2") ||
	       !strcmp(machine, "cninjabl");
}

// Mutant Fighter (its own board: mutantf.c) through this driver: the game table is the one of cninja.cpp
extern const rbx_driver rbx_drv_mutantf;
static int s_mf;
static int cn_init(const rbx_game *g)
{
	s_mf = !strcmp(g->machine, "mutantf");
	return s_mf ? rbx_drv_mutantf.init(g) : rg_init(g);
}
static void cn_exit(void) { if (s_mf) rbx_drv_mutantf.exit(); else rg_exit(); }
static void cn_reset(void) { if (s_mf) rbx_drv_mutantf.reset(); else rg_reset(); }
static void cn_frame(void) { if (s_mf) rbx_drv_mutantf.run_frame(); else rg_frame(); }
static int cn_save(void *ctx) { return s_mf ? rbx_drv_mutantf.state_save(ctx) : rg_save(ctx); }
static int cn_load(void *ctx) { return s_mf ? rbx_drv_mutantf.state_load(ctx) : rg_load(ctx); }
static void cn_render(void) { if (s_mf) rbx_drv_mutantf.render(); else rg_render(); }
static int cn_help(int helper) { return s_mf ? rbx_drv_mutantf.render_help(helper) : 0; }

const rbx_driver rbx_drv_cninja = { "DECO 16 Caveman Ninja", cn_init, cn_exit, cn_reset, cn_frame, cn_save, cn_load,
                                    rg_supported, cn_render, cn_help };
