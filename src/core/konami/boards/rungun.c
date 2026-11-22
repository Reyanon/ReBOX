/*  Konami Run and Gun / Slam Dunk. Ported from MAME src/mame/konami/rungun.cpp (BSD-3-Clause, copyright-holders:
    R. Belmont) and the devices it uses (k053936.cpp: David Haywood; tilemap.cpp's draw_roz; k053252.cpp: Angelo Salese;
    k054321.cpp: Olivier Galibert; eepromser.cpp: Aaron Giles). A 68000 at 16 MHz, the K055673 sprites (4 bpp, its RNG
    layout), a K053936 (PSAC2) ROZ layer of 16 x 16 tiles, the TTL text layer, a K053252 CCU, an ER5911 EEPROM; the
    sound: a Z80 at 8 MHz, two K054539s, the K054321 latches. Drawn by konamigx_v.c's pre-GX mode (MAME's bitmap_ind16).
    The single screen sets (rng); the dual screen ones (rng_dual) not yet.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../machine/eepromser.h"
#include "../../sound/k054539.h"
#include "../video/k055673.h"
#include "konamigx.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define MAIN_CLOCK 16000000u
#define Z80_CLOCK 8000000u
#define SND_RATE 48000
#define PIX_CLOCK 8000000u   // (set_raw(32 MHz / 4, 512, 88, 88+416, 264, 24, 24+224))
#define HTOTAL 512
#define LINES 264
#define VIS_X0 72   // (the K053252 as the game sets it: x from its offset 9*8, 416 wide)
#define VIS_Y0 24
#define VIS_W 416
#define VIS_H 224
#define ROZ_XOFF 34   // (k053936 set_offsets(34, 9), no wrap)
#define ROZ_YOFF 9

static uint8_t *s_rom, *s_zrom, *s_roz_rom;
static uint32_t s_rom_size, s_zrom_size, s_roz_rom_size;
static uint8_t *s_roz_gfx, *s_ttl_gfx;   // (gfx1: gfx_16x16x4_packed_msb; gfx3: 8x8 4 bpp packed msb; a byte a pixel)
static uint32_t s_roz_n, s_ttl_n;
static z80 s_z;
static const uint8_t *s_zmap[256];
static k054539_t (*s_k539_hp)[2];
#define s_k539 (*s_k539_hp)
static eepser_t s_eeprom;
static int s_port_sys, s_port_dsw, s_port_p[4];
static int32_t s_gain_l, s_gain_r;

static struct {
	uint16_t sysreg[16];
	uint16_t banked_ram[0x2000];   // 600000: the two sprite RAM banks (0x1000 words each)
	uint16_t pal2[0x400];          // (the second screen's palette: the mux bank 1; the first is gx.palram)
	uint16_t ttl_vram[0x2000];     // (two banks of 0x1000 words)
	uint16_t psac2_vram[2][0x8000];
	uint16_t k936_ctrl[16], k936_line[0x400];
	uint8_t ccu[16];
	uint8_t zram[0x2000], zram2[2][0x1d0];
	uint8_t latch[3], k54321_active, k54321_volume;
	uint8_t sound_ctrl, sound_nmi_clk, nmi_line, z80_irq;
	uint8_t irq5_enable, single_screen, video_priority_mode, video_mux_bank, sprite_bank, roz_rombase, mute;
	uint8_t display_bank;
	int vpos;
	int64_t cycles_done;
	uint64_t cyc_frac, z_frac, z_end, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} r;

// ---- the drawing's snapshot (screen_update) ----
static struct {
	uint16_t ctrl[16], line[0x400];
	uint16_t psac2[0x8000], ttl[0x1000];
	uint8_t priority_mode;
} s_vs;

// ---- the K054321 ----
static void k54321_volume(void)
{
	const double vol = pow(2.0, ((double)r.k54321_volume - 40.0) / 10.0);
	s_gain_l = (r.k54321_active & 2) ? (int32_t)(vol * 4096.0 + 0.5) : 0;
	s_gain_r = (r.k54321_active & 1) ? (int32_t)(vol * 4096.0 + 0.5) : 0;
}
static void k54321_main_w(int reg, uint8_t v)
{
	switch (reg) {
	case 0x0: r.k54321_active = v; k54321_volume(); break;
	case 0x2: r.k54321_volume = 0; k54321_volume(); break;
	case 0x3:
		if (v && r.k54321_volume < 64) {
			r.k54321_volume++;
			k54321_volume();
		}
		break;
	case 0x6: r.latch[0] = v; break;
	case 0x7: r.latch[1] = v; break;
	}
}
static uint8_t k54321_main_r(int reg) { return reg == 0xa ? r.latch[2] : reg == 0x8 ? 0 : 0xff; }

// ---- the video callbacks (rungun_state) ----
static void sprite_cb(int *code, int *color, int *primask)
{
	(void)code;
	(void)primask;
	*color = 0x20 | (*color & 0x001f);   // (m_sprite_colorbase 0x20)
}

static void capture(void)
{
	const int b = r.display_bank;
	memcpy(s_vs.ctrl, r.k936_ctrl, sizeof(s_vs.ctrl));
	memcpy(s_vs.line, r.k936_line, sizeof(s_vs.line));
	memcpy(s_vs.psac2, r.psac2_vram[b], sizeof(s_vs.psac2));
	memcpy(s_vs.ttl, r.ttl_vram + b * 0x1000, sizeof(s_vs.ttl));
	s_vs.priority_mode = r.video_priority_mode;
}

// one line of the PSAC2 tilemap (128 x 128 tiles of 16 x 16: 2048 x 2048, not wrapped; draw_roz_core) from the
// bitmap's x0 to x1 with the start and the increments in 16.16
static void roz_line(int y, int x0, int x1, uint32_t cx, uint32_t cy, int incxx, int incxy)
{
	uint16_t *o = gx_pre_frame16() + (y - VIS_Y0) * VIS_W - VIS_X0;
	const uint16_t *vram = s_vs.psac2;
	if (incxx == 0x10000 && incxy == 0 && !(cx & 0xffff)) {   // (a pixel a pixel along a row: by tiles)
		if (cy >= 0x8000000u)
			return;
		const uint32_t py = cy >> 16;
		const uint16_t *vrow = vram + (py >> 4) * 256;
		uint32_t px = cx >> 16;
		for (int x = x0; x <= x1;) {
			int k = 16 - (int)(px & 15);
			if (k > x1 - x + 1)
				k = x1 - x + 1;
			if (px >= 2048) {   // (outside the map: nothing)
				x += k;
				px += (uint32_t)k;
				continue;
			}
			const uint16_t attr = vrow[(px >> 4) * 2], code = vrow[(px >> 4) * 2 + 1];
			const uint32_t t = (uint32_t)(code & 0x3fff);
			const uint32_t ty = (code & 0x8000) ? 15 - (py & 15) : (py & 15);
			const uint8_t *row = t < s_roz_n ? s_roz_gfx + t * 256 + ty * 16 : NULL;
			const uint16_t base = (uint16_t)((0x10 + (attr & 0x0f)) * 16);
			if (row) {
				if (code & 0x4000)
					for (int i = 0; i < k; i++, x++, px++) {
						const int p = row[15 - (px & 15)];
						if (p)
							o[x] = (uint16_t)(base + p);
					}
				else
					for (int i = 0; i < k; i++, x++, px++) {
						const int p = row[px & 15];
						if (p)
							o[x] = (uint16_t)(base + p);
					}
			} else {
				x += k;
				px += (uint32_t)k;
			}
		}
		return;
	}
	for (int x = x0; x <= x1; x++, cx += (uint32_t)incxx, cy += (uint32_t)incxy) {
		if (cx >= 0x8000000u || cy >= 0x8000000u)
			continue;
		const uint32_t px = cx >> 16, py = cy >> 16;
		const int ti = (int)((py >> 4) * 128 + (px >> 4));
		const uint16_t attr = vram[ti * 2], code = vram[ti * 2 + 1];
		const uint32_t t = (uint32_t)(code & 0x3fff);
		if (t >= s_roz_n)
			continue;
		const uint32_t tx = (code & 0x4000) ? 15 - (px & 15) : (px & 15), ty = (code & 0x8000) ? 15 - (py & 15) : (py & 15);
		const int p = s_roz_gfx[t * 256 + ty * 16 + tx];
		if (p)
			o[x] = (uint16_t)((0x10 + (attr & 0x0f)) * 16 + p);
	}
}

// k053936_device::zoom_draw (flags 0, priority 0, glfgreat_hack 1) over the visible area
static void roz_draw(void)
{
	const uint16_t *ctrl = s_vs.ctrl;
	const int cl = VIS_X0, cr = VIS_X0 + VIS_W - 1, ct = VIS_Y0, cb = VIS_Y0 + VIS_H - 1;
	if (ctrl[0x07] & 0x0040) {   // (per-line: lineaddr from the line RAM)
		int minx = cl, maxx = cr, y = ct, maxy = cb;
		if ((ctrl[0x07] & 0x0002) && ctrl[0x09]) {   // (the glfgreat hack's clip)
			minx = ctrl[0x08] + ROZ_XOFF + 2;
			maxx = ctrl[0x09] + ROZ_XOFF + 2 - 1;
			if (minx < cl)
				minx = cl;
			if (maxx > cr)
				maxx = cr;
			y = ctrl[0x0a] + ROZ_YOFF - 2;
			if (y < ct)
				y = ct;
			maxy = ctrl[0x0b] + ROZ_YOFF - 2 - 1;
			if (maxy > cb)
				maxy = cb;
		}
		if (minx > maxx)
			return;
		for (; y <= maxy; y++) {
			const uint16_t *la = s_vs.line + 4 * ((y - ROZ_YOFF) & 0x1ff);
			uint32_t startx = (uint32_t)(256 * (int16_t)(la[0] + ctrl[0x00]));
			uint32_t starty = (uint32_t)(256 * (int16_t)(la[1] + ctrl[0x01]));
			int incxx = (int16_t)la[2], incxy = (int16_t)la[3];
			if (ctrl[0x06] & 0x8000)
				incxx *= 256;
			if (ctrl[0x06] & 0x0080)
				incxy *= 256;
			startx -= (uint32_t)(ROZ_XOFF * incxx);
			starty -= (uint32_t)(ROZ_XOFF * incxy);
			// (draw_roz_core: the start advanced to the clip's left)
			const uint32_t sx = (startx << 5) + (uint32_t)minx * (uint32_t)(incxx << 5);
			const uint32_t sy = (starty << 5) + (uint32_t)minx * (uint32_t)(incxy << 5);
			roz_line(y, minx, maxx, sx, sy, incxx << 5, incxy << 5);
		}
	} else {   // ("simple" mode)
		uint32_t startx = (uint32_t)(256 * (int16_t)ctrl[0x00]), starty = (uint32_t)(256 * (int16_t)ctrl[0x01]);
		int incyx = (int16_t)ctrl[0x02], incyy = (int16_t)ctrl[0x03], incxx = (int16_t)ctrl[0x04],
		    incxy = (int16_t)ctrl[0x05];
		if (ctrl[0x06] & 0x4000) {
			incyx *= 256;
			incyy *= 256;
		}
		if (ctrl[0x06] & 0x0040) {
			incxx *= 256;
			incxy *= 256;
		}
		startx -= (uint32_t)(ROZ_YOFF * incyx);
		starty -= (uint32_t)(ROZ_YOFF * incyy);
		startx -= (uint32_t)(ROZ_XOFF * incxx);
		starty -= (uint32_t)(ROZ_XOFF * incxy);
		startx <<= 5;
		starty <<= 5;
		incxx <<= 5;
		incxy <<= 5;
		incyx <<= 5;
		incyy <<= 5;
		startx += (uint32_t)cl * (uint32_t)incxx + (uint32_t)ct * (uint32_t)incyx;
		starty += (uint32_t)cl * (uint32_t)incxy + (uint32_t)ct * (uint32_t)incyy;
		for (int y = ct; y <= cb; y++, startx += (uint32_t)incyx, starty += (uint32_t)incyy)
			roz_line(y, cl, cr, startx, starty, incxx, incxy);
	}
}

// the TTL text layer (64 x 32 tiles of 8 x 8, not scrolled, pen 0 transparent; ttl_get_tile_info)
static void ttl_draw(void)
{
	uint16_t *frame = gx_pre_frame16();
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *o = frame + (y - VIS_Y0) * VIS_W - VIS_X0;
		const uint16_t *vrow = s_vs.ttl + (y >> 3) * 64 * 2;
		for (int tx = VIS_X0 >> 3; tx < (VIS_X0 + VIS_W) >> 3; tx++) {
			const uint16_t w0 = vrow[tx * 2], w1 = vrow[tx * 2 + 1];
			const uint32_t code = (uint32_t)((w0 & 0x0f) << 8 | (w1 & 0xff));
			if (code >= s_ttl_n)
				continue;
			const uint8_t *row = s_ttl_gfx + code * 64 + (y & 7) * 8;
			const uint16_t base = (uint16_t)(((w0 & 0xf0) >> 4) * 16);
			uint16_t *d = o + tx * 8;
			for (int i = 0; i < 8; i++)
				if (row[i])
					d[i] = (uint16_t)(base + row[i]);
		}
	}
}

static void draw(void)   // (screen_update)
{
	gx_pre_fill();
	if (!s_vs.priority_mode) {
		roz_draw();
		gx_pre_sprites();
	} else {
		gx_pre_sprites();
		roz_draw();
	}
	ttl_draw();
}

// ---- the main CPU's bus ----
static inline int64_t now_cycles(void) { return r.cycles_done + m68k_cycles_run(); }

static void sprite_dma(void)   // (sprite_dma_trigger: the sprite RAM bank to the K055673)
{
	const uint32_t src = r.single_screen ? 0x1000 : (uint32_t)r.display_bank * 0x1000;
	memcpy(gx.k55673.ram, r.banked_ram + src, 0x800 * sizeof(uint16_t));
}

static uint16_t sysregs_r(int off)
{
	switch (off) {
	case 0x00 / 2:
		return (uint16_t)(rbx_port_read(s_port_p[0]) | rbx_port_read(s_port_p[2]) << 8);
	case 0x02 / 2:
		return (uint16_t)(rbx_port_read(s_port_p[1]) | rbx_port_read(s_port_p[3]) << 8);
	case 0x04 / 2: {
		const uint32_t field_bit = r.single_screen ? 1 : (~r.frame & 1);
		return (uint16_t)((rbx_port_read(s_port_sys) & 0xfdff) | field_bit << 9);
	}
	case 0x06 / 2:
		rbx_port_custom(s_port_dsw, 0x03,
		                (uint32_t)(eepser_do_read(&s_eeprom) | eepser_ready_read(&s_eeprom, now_cycles()) << 1));
		return (uint16_t)((r.sysreg[0x06 / 2] & 0xff00) | (rbx_port_read(s_port_dsw) & 0xff));
	}
	return r.sysreg[off];
}

static void sysregs_w(int off, uint16_t v, uint16_t mask)
{
	r.sysreg[off] = (uint16_t)((v & mask) | (r.sysreg[off] & ~mask));
	const uint16_t d = r.sysreg[off];
	if (off == 0x08 / 2) {
		if (mask & 0x00ff) {
			const int64_t t = now_cycles();
			eepser_di_write(&s_eeprom, d & 1);
			eepser_cs_write(&s_eeprom, d >> 1 & 1, t);
			eepser_clk_write(&s_eeprom, d >> 2 & 1, t);
			r.sprite_bank = (uint8_t)(d >> 7 & 1);
			r.video_mux_bank = (uint8_t)((d >> 7 & 1) ^ 1);
		}
		if (mask & 0xff00) {
			r.irq5_enable = (uint8_t)(d >> 10 & 1);
			if (!r.irq5_enable)
				m68k_bus_set_line(5, 0);
			r.single_screen = (uint8_t)(d >> 12 & 1);
			r.video_priority_mode = (uint8_t)(d >> 14 & 1);
		}
	} else if (off == 0x0c / 2) {
		gx.k55673.objcha_line = (d & 0x04) != 0;
		r.mute = !(d & 0x08);
		r.roz_rombase = (uint8_t)((d & 0xf0) >> 4);
	}
}

static uint8_t ccu_r(int reg)   // (k053252_device::read: the VCT read-back)
{
	const int vc = ((r.ccu[8] << 8 | r.ccu[9]) & 0x1ff) + 1;
	if (reg == 0x0e)
		return (uint8_t)(((r.vpos - vc) >> 8) & 1);
	if (reg == 0x0f)
		return (uint8_t)((r.vpos - vc) & 0xff);
	return r.ccu[reg];
}

static uint16_t main_r16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x300000 && a < 0x300800) {
		const int o = (int)((a - 0x300000) >> 1);
		return r.video_mux_bank ? r.pal2[o] : ((const uint16_t *)gx.palram)[o];
	}
	if (a >= 0x400000 && a < 0x440000) {   // (k53936_rom_r: umask 00ff)
		const uint32_t ra = ((a - 0x400000) >> 1) + (uint32_t)r.roz_rombase * 0x20000;
		return ra < s_roz_rom_size ? s_roz_rom[ra] : 0;
	}
	if (a >= 0x480000 && a < 0x480020)
		return sysregs_r((int)((a - 0x480000) >> 1));
	if (a >= 0x4c0000 && a < 0x4c0020)
		return ccu_r((int)((a - 0x4c0000) >> 1));
	if (a >= 0x580000 && a < 0x580020)
		return (uint16_t)(k54321_main_r((int)((a - 0x580000) >> 1)) << 8);
	if (a >= 0x5c0000 && a < 0x5c0010)
		return k055673_rom_word_r(&gx.k55673, (int)((a - 0x5c0000) >> 1));
	if (a >= 0x600000 && a < 0x602000)
		return r.banked_ram[r.sprite_bank * 0x1000 + ((a - 0x600000) >> 1)];
	if (a >= 0x6c0000 && a < 0x6d0000)
		return r.psac2_vram[r.video_mux_bank][(a - 0x6c0000) >> 1];
	if (a >= 0x700000 && a < 0x700800)
		return r.k936_line[(a - 0x700000) >> 1];
	if (a >= 0x740000 && a < 0x742000)
		return r.ttl_vram[r.video_mux_bank * 0x1000 + ((a - 0x740000) >> 1)];
	return 0;
}
static uint8_t main_r8(uint32_t a)
{
	const uint16_t w = main_r16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}

static inline void combine(uint16_t *p, uint16_t v, uint16_t mask) { *p = (uint16_t)((v & mask) | (*p & ~mask)); }

static void main_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a >= 0x300000 && a < 0x300800) {
		const int o = (int)((a - 0x300000) >> 1);
		combine(r.video_mux_bank ? &r.pal2[o] : &((uint16_t *)gx.palram)[o], v, mask);
	} else if (a >= 0x480000 && a < 0x480020)
		sysregs_w((int)((a - 0x480000) >> 1), v, mask);
	else if (a >= 0x4c0000 && a < 0x4c0020) {   // (the K053252: umask 00ff)
		if (mask & 0x00ff)
			r.ccu[((a - 0x4c0000) >> 1) & 15] = (uint8_t)v;
	} else if (a == 0x540000) {   // (sound_irq_w)
		if (mask & 0xff00)
			r.z80_irq = 1;
	} else if (a >= 0x580000 && a < 0x580020) {   // (the K054321: umask ff00)
		if (mask & 0xff00)
			k54321_main_w((int)((a - 0x580000) >> 1), (uint8_t)(v >> 8));
	} else if (a >= 0x5c0010 && a < 0x5c0020)
		k055673_reg_word_w(&gx.k55673, (int)((a - 0x5c0010) >> 1), v, mask);
	else if (a >= 0x600000 && a < 0x602000)
		combine(&r.banked_ram[r.sprite_bank * 0x1000 + ((a - 0x600000) >> 1)], v, mask);
	else if (a >= 0x640000 && a < 0x640008) {
		if (mask & 0xff00) k055673_k053246_w(&gx.k55673, (int)(a - 0x640000), (uint8_t)(v >> 8));
		if (mask & 0x00ff) k055673_k053246_w(&gx.k55673, (int)(a - 0x640000) + 1, (uint8_t)v);
	} else if (a >= 0x680000 && a < 0x680020)
		combine(&r.k936_ctrl[(a - 0x680000) >> 1], v, mask);
	else if (a >= 0x6c0000 && a < 0x6d0000)
		combine(&r.psac2_vram[r.video_mux_bank][(a - 0x6c0000) >> 1], v, mask);
	else if (a >= 0x700000 && a < 0x700800)
		combine(&r.k936_line[(a - 0x700000) >> 1], v, mask);
	else if (a >= 0x740000 && a < 0x742000)
		combine(&r.ttl_vram[r.video_mux_bank * 0x1000 + ((a - 0x740000) >> 1)], v, mask);
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v)
{
	main_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00);
}

// ---- the sound board (rungun_sound_map) ----
static void z_bank(void)   // (bank2: 8 entries of 0x4000 from 0x10000)
{
	const uint32_t base = 0x10000 + (uint32_t)(r.sound_ctrl & 0x07) * 0x4000;
	for (int p = 0; p < 0x40; p++)
		s_zmap[0x80 + p] = base + (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + base + (uint32_t)p * 256 : NULL;
}

static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_zrom_size ? s_zrom[a] : 0xff;
	if (a < 0xc000) {
		const uint32_t o = 0x10000 + (uint32_t)(r.sound_ctrl & 0x07) * 0x4000 + (a - 0x8000);
		return o < s_zrom_size ? s_zrom[o] : 0xff;
	}
	if (a < 0xe000)
		return r.zram[a - 0xc000];
	if (a < 0xe230)
		return k054539_read(&s_k539[0], a - 0xe000);
	if (a < 0xe400)
		return r.zram2[0][a - 0xe230];
	if (a < 0xe630)
		return k054539_read(&s_k539[1], a - 0xe400);
	if (a < 0xe800)
		return r.zram2[1][a - 0xe630];
	if (a == 0xf002)
		return r.latch[0];
	if (a == 0xf003)
		return r.latch[1];
	return 0xff;
}

static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xc000)
		return;
	if (a < 0xe000)
		r.zram[a - 0xc000] = v;
	else if (a < 0xe230)
		k054539_write(&s_k539[0], a - 0xe000, v);
	else if (a < 0xe400)
		r.zram2[0][a - 0xe230] = v;
	else if (a < 0xe630)
		k054539_write(&s_k539[1], a - 0xe400, v);
	else if (a < 0xe800)
		r.zram2[1][a - 0xe630] = v;
	else if (a == 0xf000)
		r.latch[2] = v;
	else if (a == 0xf800) {   // (sound_ctrl_w)
		if (!(v & 0x10))
			r.nmi_line = 0;
		r.sound_ctrl = v;
		z_bank();
	}
}

static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void k054539_nmi_gen(void *user, int state)   // (the first chip's timer: an NMI on the rising edge)
{
	(void)user;
	if ((r.sound_ctrl & 0x10) && !r.sound_nmi_clk && state && !r.nmi_line) {
		r.nmi_line = 1;
		z80_gen_nmi(&s_z);
	}
	r.sound_nmi_clk = (uint8_t)state;
}

static void sound_run(int n)
{
	for (int i = 0; i < n; i++) {
		r.z_frac += Z80_CLOCK;
		const unsigned long cyc = (unsigned long)(r.z_frac / SND_RATE);
		r.z_frac -= (uint64_t)cyc * SND_RATE;
		int line = r.z80_irq;
		r.z_end += cyc;
		z80_run(&s_z, (unsigned long)r.z_end, &line, 1);
		r.z80_irq = (uint8_t)line;
		int32_t l0 = 0, r0 = 0, l1 = 0, r1 = 0;
		k054539_render(&s_k539[0], &l0, &r0, 1);
		k054539_render(&s_k539[1], &l1, &r1, 1);
		// (the chips' outputs to the speaker inverted, the BGM chip at 0.6, at the K054321's gains)
		const int32_t sl = r0 + (int32_t)((int64_t)r1 * 2458 >> 12), sr = l0 + (int32_t)((int64_t)l1 * 2458 >> 12);
		if (r.mute)
			rbx_audio_put_stereo(0, 0);
		else
			rbx_audio_put_stereo((int32_t)((int64_t)sl * s_gain_l >> 12), (int32_t)((int64_t)sr * s_gain_r >> 12));
	}
}

// ---- init / frame ----
static void rng_exit(void);
static void rng_reset(void);

static uint8_t *decode_packed(const uint8_t *rom, uint32_t size, int w, uint32_t *n)   // (packed msb, 4 bpp)
{
	const uint32_t bytes = (uint32_t)(w * w / 2);
	*n = size / bytes;
	uint8_t *g = (uint8_t *)malloc((size_t)*n * (size_t)(w * w) + 1);
	if (!g)
		return NULL;
	for (uint32_t i = 0; i < *n * bytes; i++) {
		g[i * 2] = rom[i] >> 4;
		g[i * 2 + 1] = rom[i] & 15;
	}
	return g;
}

static int rng_init(const rbx_game *g)
{
	if (!s_k539_hp && !(s_k539_hp = calloc(1, sizeof(*s_k539_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t pcm_size, spr_size, ttl_size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("soundcpu", &s_zrom_size);
	s_roz_rom = rbx_region("gfx1", &s_roz_rom_size);
	const uint8_t *pcm = rbx_region("k054539", &pcm_size);
	const uint8_t *spr = rbx_region("k055673", &spr_size);
	const uint8_t *ttl = rbx_region("gfx3", &ttl_size);
	if (!s_rom || !s_zrom || !s_roz_rom || !pcm || !spr || !ttl) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&gx, 0, sizeof(gx));
	memset(&r, 0, sizeof(r));
	gx.pal_entries = 1024;
	gx.pal555 = 1;
	gx.tilecb = GX_TILECB_PRE;
	gx.mixmode = GX_MIXMODE_PRE;
	gx.pre_sprcb = sprite_cb;
	gx.pre_capture = capture;
	gx.pre_draw = draw;
	gx.pre_screen_w = HTOTAL;
	k055673_init(&gx.k55673, spr, spr_size, K055673_LAYOUT_RNG, -8, 15);
	gx_video_set_area(VIS_X0, VIS_W);
	gx_video_set_y0(VIS_Y0);
	gx_video_set_height(VIS_H);
	s_roz_gfx = decode_packed(s_roz_rom, s_roz_rom_size, 16, &s_roz_n);
	s_ttl_gfx = decode_packed(ttl, ttl_size, 8, &s_ttl_n);
	if (s_roz_n > 0x4000)   // (the codes: 14 bits)
		s_roz_n = 0x4000;
	m68k_init();
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0x000000, 0x2fffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x380000, 0x39ffff, gx.workram, 0x20000, 1, 1);
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
	for (int i = 0; i < 2; i++)
		k054539_init(&s_k539[i], pcm, pcm_size);
	s_k539[0].timer_cb = k054539_nmi_gen;
	eepser_init(&s_eeprom, EEPSER_ER5911, 128, 8, MAIN_CLOCK);
	{
		uint32_t esz;
		const uint8_t *e = rbx_region("eeprom", &esz);
		if (e)
			eepser_default(&s_eeprom, e, esz);
	}
	eepser_nv_load(&s_eeprom, g->set);   // (the contents kept between runs)
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_dsw = rbx_port_find("DSW");
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_p[2] = rbx_port_find("P3");
	s_port_p[3] = rbx_port_find("P4");
	if (!s_roz_gfx || !s_ttl_gfx || rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, 0x10000) ||
	    gx_video_init()) {
		rbx_fail("%s: out of memory", g->set);
		rng_exit();
		return -1;
	}
	for (int i = 0; i < 0x10000; i++) {   // (the bitmap: RGB 565)
		const int rr = (i >> 11) & 31, gg = (i >> 5) & 63, b = i & 31;
		rbx_video.palette[i] = (uint32_t)((rr << 3 | rr >> 2) << 16 | (gg << 2 | gg >> 4) << 8 | (b << 3 | b >> 2));
	}
	rng_reset();
	return 0;
}

static void rng_exit(void)
{
	eepser_nv_save(&s_eeprom);
	gx_video_exit();
	free(s_roz_gfx);
	free(s_ttl_gfx);
	s_roz_gfx = s_ttl_gfx = NULL;
	s_roz_n = s_ttl_n = 0;
	free(s_k539_hp);
	s_k539_hp = NULL;
}

static void rng_reset(void)
{
	uint8_t *pal = gx.palram;
	memset(gx.workram, 0, sizeof(gx.workram));
	memset(pal, 0, sizeof(gx.palram));
	memset(gx.k55673.ram, 0, sizeof(gx.k55673.ram));
	memset(&r, 0, offsetof(__typeof__(r), state_end));
	k54321_volume();
	for (int i = 0; i < 2; i++)
		k054539_reset(&s_k539[i]);
	z80_init(&s_z);
	s_z.read_byte = z_rd;
	s_z.write_byte = z_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	for (int p = 0; p < 256; p++)
		s_zmap[p] = NULL;
	for (int p = 0; p < 0x80; p++)
		s_zmap[p] = (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
	for (int p = 0; p < 0x20; p++)
		s_zmap[0xc0 + p] = r.zram + p * 256;
	s_z.read_map = s_zmap;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	for (int i = 0; i < 0x10; i++)   // (machine_reset: the system registers cleared, sound_ctrl_w(0))
		sysregs_w(i, 0, 0xffff);
	r.sound_ctrl = 0;
	z_bank();
}

static void rng_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		r.vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (line == 0) {   // (vblank: the update with this frame's bank, then the interrupt's sprite DMA, IRQ 5)
			r.display_bank = r.single_screen ? 0 : (uint8_t)(r.frame & 1);
			gx_video_capture();
			sprite_dma();
			if (r.irq5_enable)
				m68k_bus_set_line(5, 1);
		}
		if (r.vpos == VIS_Y0)   // (vblank end: the frame number)
			r.frame++;
		r.cyc_frac += (uint64_t)MAIN_CLOCK * HTOTAL;
		int64_t cyc = (int64_t)(r.cyc_frac / PIX_CLOCK);
		r.cyc_frac -= (uint64_t)cyc * PIX_CLOCK;
		while (cyc > 0) {
			const int used = m68k_execute((int)cyc);
			r.cycles_done += used;
			cyc -= used;
		}
		r.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(r.snd_frac / PIX_CLOCK);
		r.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		sound_run(ns);
	}
	gx.frame = r.frame;
}

static int rng_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	int rc = 0;
	if ((uint32_t)m68k_context_size() > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	rc |= rbx_state_io(ctx, "rng_cpu", cpu, (uint32_t)m68k_context_size());
	rc |= rbx_state_io(ctx, "rng_bus", &m68k_bus.irq_pending, sizeof(uint32_t) * 2);
	rc |= rbx_state_io(ctx, "gx", &gx, (uint32_t)offsetof(gx_t, state_end));
	rc |= k055673_state(&gx.k55673, ctx, load);
	rc |= rbx_state_io(ctx, "rng", &r, (uint32_t)offsetof(__typeof__(r), state_end));
	rc |= eepser_state(&s_eeprom, ctx, load, "rng_eeprom");
	rc |= k054539_state(&s_k539[0], ctx, load, "rng_k539a_");
	rc |= k054539_state(&s_k539[1], ctx, load, "rng_k539b_");
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "rng_z80", &z, sizeof(z));
	if (!rc && load) {
		m68k_set_context(cpu);
		z.read_byte = z_rd;
		z.write_byte = z_wr;
		z.port_in = z_in;
		z.port_out = z_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		z.op_rom = NULL;
		s_z = z;
		z_bank();
		k54321_volume();
	}
	return rc ? -1 : 0;
}

static int rng_save(void *ctx) { return rng_state(ctx, 0); }
static int rng_load(void *ctx) { return rng_state(ctx, 1); }
static int rng_supported(const char *machine) { return !strcmp(machine, "rng"); }

const rbx_driver rbx_drv_rungun = { "Konami Run and Gun", rng_init, rng_exit, rng_reset, rng_frame, rng_save, rng_load,
                                    rng_supported, gx_render, NULL };
