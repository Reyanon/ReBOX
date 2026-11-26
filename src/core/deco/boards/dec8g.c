/*  Data East "dec8" HD6309 boards, from MAME src/mame/dataeast/dec8.cpp, dec8_v.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail): The Real Ghostbusters, Meikyuu Hunter G, Gondomania, Garyo Retsuden.
    The HD63C09 at 3 MHz (its NMI: vblank gated by a latch bit taking effect one instruction late; its IRQ from the
    MCU, acknowledged by the latch), the i8751 (the command and return words, its INT0 at vblank, its INT1 pulsed by
    the command; MAME runs them in perfect sync: here interleaved by sixteenths of a line), a BAC06 playfield
    (8-bit, big-endian, row scroll), the Karnov sprites (buffered on a latch bit's rising edge), a 3 bpp text layer,
    the PROM palette. The sound: DECO 222 (Meikyuu Hunter: R65C02) at 1.5 MHz with YM2203 and YM3812, the latch
    on a short NMI pulse. Gondomania / Garyo Retsuden: a 16x16 background instead of the BAC06 (Gondomania: pens 8-15
    over the sprites of colour bit 3 clear; Garyo Retsuden: the tiles of colour bit 3 over the sprites), the RM-C3
    palette RAM, the sprite DMA on a scroll register bit, the MCU's INT1 a 4-clock pulse, R65C02 with YM2203 and
    YM3526; Gondomania's 12-way rotary joysticks. Reached through the dec8 driver (dec8.c).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/mcs51/mcs51.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../video/bac06.h"
#include "../video/deckarn.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 3000000        // HD63C09E
#define SND_CLOCK 1500000
#define MCU_CLOCK (8000000 / 12)
#define YM1_CLOCK 1500000        // YM2203
#define YM2_CLOCK 3000000        // YM3812
#define FRAME_MHZ 57445          // set_raw(12 MHz / 2, 384, 0, 256, 272, 8, 248)
#define LINES 272
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 256
#define SLICES 16

enum { V_GHOSTB, V_MEIKYUH, V_GONDO, V_GARYORET };
#define GONDO_LIKE (s_var >= V_GONDO)
static int s_var;
static uint8_t *s_rom, *s_bank_rom, *s_mcu_rom, *s_prom;
static uint32_t s_rom_size, s_bank_size, s_mcu_size, s_prom_size;
static uint8_t s_ram[0x3800];              // 0x0000 .. 0x37ff (RAM, text, the BAC06 / background, RAM / palette, sprites)
static uint16_t s_sprbuf[0x400];           // buffer_spriteram16_w
static int s_bank, s_bank_mask, s_bank_data, s_secclr, s_buffer_strobe, s_flip;
static int s_nmi_enable, s_nmi_delay, s_nmi_next;   // (the NMI gate's latch bit, applied one instruction late)
// the vblank's NMI one instruction after the vblank starts (MAME: the screen's vblank to the NMI gate's input merger
// through a synchronize; the CPU reads the vblank flag once before the NMI: ghostb's 88d5 wait at the 2nd stage)
static int s_vbl_nmi_delay;
static m6809_t s_cpu;
static int s_irq;
static bac06_t s_tile;
static rbx_gfx s_gfx8, s_gfx16, s_gfxspr;
static uint32_t s_palette[1024];          // (from the PROMs)
static uint8_t s_scroll[4];                // (Gondomania / Garyo Retsuden: x MSB, x LSB, y MSB, y LSB)
static int s_rotary[2];                    // (Gondomania: 0 .. 11)
static uint8_t (*s_pri_buf_hp)[256 * 240];
#define s_pri_buf (*s_pri_buf_hp)
static int s_port_in[4], s_port_coin, s_port_dsw[2];
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac, s_snd_frac, s_mcu_frac, s_ym1_frac, s_ym2_frac;
static struct {
	bac06_t tile;
	uint8_t txt[0x800], bg[0x800], pal[0x800];
	uint16_t spr[0x400];
	uint8_t scroll[4];
	int flip;
} *s_snap;

#define TXT 0x1800   // (s_ram offsets)
#define BG 0x2000
#define PAL 0x2800
#define SPR 0x3000

// ---- the i8751 ----
static mcs51_t s_mcu;
static uint8_t s_p0, s_p1, s_p2;
static uint16_t s_mcu_return, s_mcu_value;
static int64_t s_int1_until;               // (the INT1 pulse: cleared after 64 (Gondomania: 4) clocks of 1.5 MHz)

static uint8_t mcu_in(void *u, int port)
{
	(void)u;
	switch (port) {
	case 0: return s_p0;
	case 1: return s_p1;
	case 3: return (uint8_t)rbx_port_read(s_port_coin);
	}
	return 0xff;
}
static void set_irq(int state)
{
	s_irq = state;
	m6809_set_irq(&s_cpu, state);
}
static void mcu_out(void *u, int port, uint8_t data)
{
	(void)u;
	if (port == 0) { s_p0 = data; return; }
	if (port == 1) { s_p1 = data; return; }
	if (port != 2)
		return;
	const uint8_t fall = (uint8_t)(~data & s_p2), rise = (uint8_t)(data & ~s_p2);   // ghostb mcu_to_main_w
	s_p2 = data;
	if (fall & 0x10) s_p0 = (uint8_t)(s_mcu_value >> 8);
	if (fall & 0x20) s_p1 = (uint8_t)(s_mcu_value & 0xff);
	if (fall & 0x40) s_mcu_return = (uint16_t)((s_mcu_return & 0xff) | (s_p0 << 8));
	if (fall & 0x80) s_mcu_return = (uint16_t)((s_mcu_return & 0xff00) | s_p1);
	if ((rise & 0x04) && s_secclr)   // P22: IRQ to the main CPU
		set_irq(1);
}

// ---- sound: DECO 222 / R65C02, YM2203, YM3812 (dec8_s_map) ----
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
	if (a == 0x6000) return s_latch;
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
static void update_nmi(void) { m6809_set_nmi(&s_cpu, s_nmi_enable && vblank()); }   // (INPUT_MERGER_ALL_HIGH)
static void set_bank(int b)
{
	s_bank = b & s_bank_mask;
	const uint32_t o = (uint32_t)s_bank * 0x4000;
	for (int p = 0; p < 0x40; p++)
		s_cpu.rpage[0x40 + p] = o + 0x4000 <= s_bank_size ? s_bank_rom + o + (uint32_t)p * 256 : NULL;
}
static void buffer_sprites(int strobe)   // (sprite DMA on the rising edge)
{
	if (!s_buffer_strobe && strobe)
		for (int i = 0; i < 0x400; i++)
			s_sprbuf[i] = (uint16_t)(s_ram[SPR + i * 2 + 1] | (s_ram[SPR + i * 2] << 8));
	s_buffer_strobe = strobe;
}
static void bank_w(uint8_t v)   // ghostb_bank_w / gondo_bank_w
{
	set_bank(v >> 4);
	s_secclr = v & 1;
	if (!s_secclr)
		set_irq(0);
	if ((s_bank_data ^ v) & 2) {   // (the NMI gate: after one more instruction)
		s_nmi_next = (v >> 1) & 1;
		s_nmi_delay = 2;
	}
	s_flip = (v >> 3) & 1;
	s_bank_data = v;
	if (!GONDO_LIKE)
		buffer_sprites((v >> 2) & 1);
}
static void gondo_scroll_w(int offset, uint8_t v)
{
	if (offset == 0x00) s_scroll[1] = v;   // x LSB
	else if (offset == 0x08) s_scroll[3] = v;   // y LSB
	else if (offset == 0x10) {
		s_scroll[0] = v & 1;   // x MSB
		s_scroll[2] = (v >> 1) & 1;   // y MSB
		buffer_sprites((v >> 2) & 1);
	}
}
static uint8_t gondo_rotary(int p, int o)   // player_io_r: the rotary (one bit of 12, reversed), the joystick
{
	const int val = 1 << ((12 - s_rotary[p]) % 12);
	if (o == 0)
		return (uint8_t)~(val & 0xff);
	return (uint8_t)(((~val >> 4) & 0xf0) | (rbx_port_read(s_port_in[p]) & 0x0f));
}
// the BAC06's 8-bit accessors (MAME ctrlreg8_w, scrollreg8_r / w<false>, vram8_r / w<false>, rowscroll8_r / w<false>)
static void ctrl8_w(bac06_t *t, uint32_t o, uint8_t v) { bac06_ctrl_w(t, (int)(o >> 1), v, 0x00ff); }
static void scroll8_w(bac06_t *t, uint32_t o, uint8_t v)
{
	const uint32_t sh = o < 4 ? (~o & 1) << 3 : 0;
	bac06_scroll_w(t, (int)(o >> 1), (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}
static uint8_t scroll8_r(const bac06_t *t, uint32_t o) { return (uint8_t)(t->scrollreg[(o >> 1) & 7] >> ((~o & 1) << 3)); }
static uint8_t vram8_r(const bac06_t *t, uint32_t o) { return (uint8_t)(bac06_vram_r(t, (int)(o >> 1)) >> ((~o & 1) << 3)); }
static void vram8_w(bac06_t *t, uint32_t o, uint8_t v)
{
	bac06_vram_w(t, (int)(o >> 1), (uint16_t)(v << ((~o & 1) << 3)), (uint16_t)(0xff << ((~o & 1) << 3)));
}
static uint8_t row8_r(const bac06_t *t, uint32_t o) { return (uint8_t)(t->rowscroll[(o >> 1) & 0xfff] >> ((~o & 1) << 3)); }
static void row8_w(bac06_t *t, uint32_t o, uint8_t v)
{
	uint16_t *p = &t->rowscroll[(o >> 1) & 0xfff];
	const uint16_t m = (uint16_t)(0xff << ((~o & 1) << 3));
	*p = (uint16_t)((*p & ~m) | ((uint16_t)(v << ((~o & 1) << 3)) & m));
}

static uint8_t cpu_r(void *ctx, uint16_t a)   // ghostb_map
{
	(void)ctx;
	if (a >= 0x2000 && a < 0x2800) return vram8_r(&s_tile, a - 0x2000u);
	if (a >= 0x2c00 && a < 0x3000) return row8_r(&s_tile, a - 0x2c00u);
	if (a >= 0x3830 && a < 0x3840) return scroll8_r(&s_tile, a & 0xfu);
	switch (a) {
	case 0x3800: return (uint8_t)rbx_port_read(s_port_in[0]);
	case 0x3801: return (uint8_t)rbx_port_read(s_port_in[1]);
	case 0x3802: return (uint8_t)rbx_port_read(s_port_in[2]);
	case 0x3803:   // (DSW0 bit 3: the vblank)
		rbx_port_custom(s_port_dsw[0], 0x08, vblank() ? 0x08 : 0);
		return (uint8_t)rbx_port_read(s_port_dsw[0]);
	case 0x3820: return (uint8_t)rbx_port_read(s_port_dsw[1]);
	case 0x3840: return (uint8_t)(s_mcu_return >> 8);
	case 0x3860: return (uint8_t)(s_mcu_return & 0xff);
	}
	return 0;
}
static void cpu_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a >= 0x2000 && a < 0x2800) vram8_w(&s_tile, a - 0x2000u, v);
	else if (a >= 0x2c00 && a < 0x3000) row8_w(&s_tile, a - 0x2c00u, v);
	else if (a >= 0x3820 && a < 0x3828) ctrl8_w(&s_tile, a & 7u, v);
	else if (a >= 0x3830 && a < 0x3840) scroll8_w(&s_tile, a & 0xfu, v);
	else if (a == 0x3800) {   // sound_w: the latch, an NMI pulse of 4 sound CPU clocks
		s_latch = v;
		m6502_set_nmi(&s_snd, 1);
		m6502_set_nmi(&s_snd, 0);
	} else if (a == 0x3840) bank_w(v);
	else if (a == 0x3860) {   // i8751_hi_w: INT1 (level), cleared 64 clocks of 1.5 MHz later
		s_mcu_value = (uint16_t)((s_mcu_value & 0xff) | (v << 8));
		mcs51_set_input(&s_mcu, MCS51_INT1_LINE, 1);
		s_int1_until = s_cpu.cycles + 64 * (CPU_CLOCK / 1500000);
	} else if (a == 0x3861)
		s_mcu_value = (uint16_t)((s_mcu_value & 0xff00) | v);
}

static uint8_t go_r(void *ctx, uint16_t a)   // gondo_map / garyoret_map
{
	(void)ctx;
	if (a == 0x3800) return (uint8_t)rbx_port_read(s_port_dsw[0]);
	if (a == 0x3801) return (uint8_t)rbx_port_read(s_port_dsw[1]);
	if (s_var == V_GONDO)
		switch (a) {
		case 0x380a: case 0x380b: return gondo_rotary(0, a & 1);
		case 0x380c: case 0x380d: return gondo_rotary(1, a & 1);
		case 0x380e:   // (IN3 bit 3: the vblank)
			rbx_port_custom(s_port_in[3], 0x08, vblank() ? 0x08 : 0);
			return (uint8_t)rbx_port_read(s_port_in[3]);
		case 0x380f: return (uint8_t)rbx_port_read(s_port_in[2]);
		case 0x3838: return (uint8_t)(s_mcu_return >> 8);
		case 0x3839: return (uint8_t)(s_mcu_return & 0xff);
		}
	else
		switch (a) {
		case 0x380a:   // (IN1 bit 7: the vblank)
			rbx_port_custom(s_port_in[1], 0x80, vblank() ? 0x80 : 0);
			return (uint8_t)rbx_port_read(s_port_in[1]);
		case 0x380b: return (uint8_t)rbx_port_read(s_port_in[0]);
		case 0x383a: return (uint8_t)(s_mcu_return >> 8);
		case 0x383b: return (uint8_t)(s_mcu_return & 0xff);
		}
	return 0;
}
static void go_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a == 0x3810) {   // sound_w
		s_latch = v;
		m6502_set_nmi(&s_snd, 1);
		m6502_set_nmi(&s_snd, 0);
	} else if (a >= 0x3818 && a < 0x3830)
		gondo_scroll_w(a - 0x3818, v);
	else if (a == 0x3830)
		bank_w(v);
	else if (a == (s_var == V_GONDO ? 0x383a : 0x3838)) {   // gondo_i8751_hi_w: INT1, cleared 4 clocks of 1.5 MHz later
		s_mcu_value = (uint16_t)((s_mcu_value & 0xff) | (v << 8));
		mcs51_set_input(&s_mcu, MCS51_INT1_LINE, 1);
		s_int1_until = s_cpu.cycles + 4 * (CPU_CLOCK / 1500000);
	} else if (a == (s_var == V_GONDO ? 0x383b : 0x3839))
		s_mcu_value = (uint16_t)((s_mcu_value & 0xff00) | v);
}

// ---- video ----
static inline uint8_t rmc3(int v) { return (uint8_t)(0x0e * (v & 1) + 0x1f * ((v >> 1) & 1) + 0x43 * ((v >> 2) & 1) + 0x8f * ((v >> 3) & 1)); }

static void gondo_colpri(uint32_t *colour, uint32_t *pri_mask)
{
	*pri_mask = 0;   // (above the background)
	if (*colour & 8)
		*pri_mask |= 0xcc;   // (GFX_PMASK_2: behind its pens 8-15)
}

static void go_render(void)
{
	for (int i = 0; i < 1024; i++) {   // deco_rgb_decoder: xxxxBBBBGGGGRRRR (the byte and its extension)
		const int raw = s_snap->pal[i] | (s_snap->pal[0x400 + i] << 8);
		rbx_video.palette[i] = (uint32_t)(rmc3(raw & 15) << 16 | rmc3((raw >> 4) & 15) << 8 | rmc3((raw >> 8) & 15));
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, flip = s_snap->flip, gondo = s_var == V_GONDO;
	const int sx = ((s_snap->scroll[0] << 8) + s_snap->scroll[1]) & 0x1ff, sy = ((s_snap->scroll[2] << 8) + s_snap->scroll[3]) & 0x1ff;
	const uint8_t *bg = s_snap->bg;
	// the background (32x32 of 16x16, rows): Gondomania, pens 0-7 priority 1, 8-15 priority 2; Garyo Retsuden, the
	// tiles of colour bit 3 (category 1) drawn again over the sprites
	for (int pass = 0; pass < (gondo ? 1 : 2); pass++) {
		if (pass == 1)
			deckarn_draw(&s_gfxspr, 256, s_snap->spr, 0x400, flip, bm, pitch, VIS_W, VIS_H, VIS_Y0);
		for (int y = 0; y < VIS_H; y++) {
			uint16_t *d = bm + y * pitch;
			uint8_t *pr = s_pri_buf + y * VIS_W;
			for (int x = 0; x < VIS_W; x++) {
				int px = (x + sx) & 0x1ff, py = (y + VIS_Y0 + sy) & 0x1ff;
				if (flip) {
					px = 511 - px;
					py = 511 - py;
				}
				const int idx = (py >> 4) * 32 + (px >> 4);
				const int tile = bg[idx * 2 + 1] | (bg[idx * 2] << 8);
				const int colour = tile >> 12;
				if (pass == 1 && !(colour & 8))
					continue;
				const uint32_t code = s_gfx16.count ? rbx_wrap((uint32_t)(tile & 0xfff), s_gfx16.count) : 0;
				const uint8_t p = s_gfx16.pix[(code * 16 + (uint32_t)(py & 15)) * 16 + (uint32_t)(px & 15)];
				d[x] = (uint16_t)(768 + colour * 16 + p);
				pr[x] = (uint8_t)(p >= 8 ? 2 : 1);
			}
		}
	}
	if (gondo)
		deckarn_draw_pri(&s_gfxspr, 256, s_snap->spr, 0x400, flip, bm, s_pri_buf, gondo_colpri, pitch, VIS_W, VIS_H,
		                 VIS_Y0);
	const uint8_t *txt = s_snap->txt;
	for (int y = 0; y < VIS_H; y++) {   // the text layer: 32x32 of 8x8, 3 bpp, colour bits 12-14, pen 0 clear
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < VIS_W; x++) {
			int px = x, py = y + VIS_Y0;
			if (flip) {
				px = 255 - px;
				py = 255 - py;
			}
			const int o = ((py >> 3) * 32 + (px >> 3)) * 2;
			const int tile = txt[o + 1] | (txt[o] << 8);
			const uint32_t code = s_gfx8.count ? rbx_wrap((uint32_t)(tile & 0xfff), s_gfx8.count) : 0;
			const uint8_t p = s_gfx8.pix[(code * 8 + (uint32_t)(py & 7)) * 8 + (uint32_t)(px & 7)];
			if (p)
				d[x] = (uint16_t)(((tile & 0x7000) >> 12) * 8 + p);
		}
	}
}

static void gb_render(void)
{
	if (GONDO_LIKE) {
		go_render();
		return;
	}
	memcpy(rbx_video.palette, s_palette, sizeof(s_palette));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	bac06_draw(&s_snap->tile, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, BAC06_OPAQUE, 0, 0xff);
	deckarn_draw(&s_gfxspr, 256, s_snap->spr, 0x400, s_snap->flip, bm, pitch, VIS_W, VIS_H, VIS_Y0);
	const uint8_t *txt = s_snap->txt;
	const int flip = s_snap->flip;
	for (int y = 0; y < VIS_H; y++) {   // the text layer: 32x32 of 8x8, 3 bpp, colour bits 10-11, pen 0 clear
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < VIS_W; x++) {
			int px = x, py = y + VIS_Y0;
			if (flip) {
				px = 255 - px;
				py = 255 - py;
			}
			const int o = ((py >> 3) * 32 + (px >> 3)) * 2;
			const int tile = txt[o + 1] | (txt[o] << 8);
			const uint32_t code = s_gfx8.count ? rbx_wrap((uint32_t)(tile & 0x3ff), s_gfx8.count) : 0;
			const uint8_t p = s_gfx8.pix[(code * 8 + (uint32_t)(py & 7)) * 8 + (uint32_t)(px & 7)];
			if (p)
				d[x] = (uint16_t)(((tile & 0xc00) >> 10) * 8 + p);
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	s_snap->tile = s_tile;
	memcpy(s_snap->txt, s_ram + TXT, sizeof(s_snap->txt));
	if (GONDO_LIKE) {
		memcpy(s_snap->bg, s_ram + BG, sizeof(s_snap->bg));
		memcpy(s_snap->pal, s_ram + PAL, sizeof(s_snap->pal));
		memcpy(s_snap->scroll, s_scroll, sizeof(s_scroll));
	}
	memcpy(s_snap->spr, s_sprbuf, sizeof(s_sprbuf));
	s_snap->flip = s_flip;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void gb_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_sprbuf, 0, sizeof(s_sprbuf));
	memset(s_scroll, 0, sizeof(s_scroll));
	s_rotary[0] = s_rotary[1] = 0;
	memset(s_tile.vram, 0, sizeof(s_tile.vram));
	memset(s_tile.rowscroll, 0, sizeof(s_tile.rowscroll));
	memset(s_tile.colscroll, 0, sizeof(s_tile.colscroll));
	bac06_reset(&s_tile);
	m6809_reset(&s_cpu);
	set_irq(0);
	m6809_set_nmi(&s_cpu, 0);
	s_bank_data = 0xff;   // (machine_reset: ghostb_bank_w(0), the latch cleared)
	s_buffer_strobe = 0;
	s_nmi_enable = 0;
	bank_w(0);
	s_nmi_enable = s_nmi_next = 0;
	s_nmi_delay = 0;
	s_vbl_nmi_delay = 0;
	s_bank_data = 0;
	s_p0 = s_p1 = 0;
	s_p2 = 0xff;
	s_mcu_return = s_mcu_value = 0;
	s_int1_until = -1;
	mcs51_reset(&s_mcu);
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
	s_cpu_frac = s_snd_frac = s_mcu_frac = s_ym1_frac = s_ym2_frac = 0;
}

static void gb_exit(void);

static int gb_init(const rbx_game *g)
{
	if (!s_pri_buf_hp && !(s_pri_buf_hp = calloc(1, sizeof(*s_pri_buf_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_var = !strcmp(g->machine, "meikyuh") ? V_MEIKYUH : !strcmp(g->machine, "gondo") ? V_GONDO
	      : !strcmp(g->machine, "garyoret") ? V_GARYORET : V_GHOSTB;
	const int go = GONDO_LIKE;
	uint32_t csize, tsize, ssize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_bank_rom = rbx_region("mainbank", &s_bank_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_mcu_rom = rbx_region("mcu", &s_mcu_size);
	s_prom = go ? NULL : rbx_region("proms", &s_prom_size);
	uint8_t *chars = rbx_region("char", &csize), *tiles = rbx_region("tiles1", &tsize), *spr = rbx_region("sprites", &ssize);
	if (!s_rom || s_rom_size < 0x10000 || !s_bank_rom || s_bank_size < 0x4000 || !s_zrom || !s_mcu_rom ||
	    s_mcu_size < 0x1000 || (!go && (!s_prom || s_prom_size < 0x800)) || !chars || csize < 0x8000 || !tiles ||
	    tsize < (go ? 0x80000u : 0x40000u) ||
	    !spr || ssize < 0x80000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_bank_mask = (int)((s_bank_size / 0x4000 - 1) & 0xf);
	{	// chars_3bpp (planes at 0x6000, 0x4000, 0x2000 bytes), tiles_r (2048: planes 0x20000, 0, 0x30000, 0x10000, x
		// reversed), tiles (4096: planes at 0x60000 .. 0)
		static uint32_t x8[8], y8[8], x16[16], xr[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 8);
			x16[i] = (uint32_t)(16 * 8 + i);
			x16[8 + i] = (uint32_t)i;
			xr[i] = (uint32_t)(7 - i);
			xr[8 + i] = (uint32_t)(7 - i + 16 * 8);
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 8);
		static const uint32_t pc[3] = { 0x6000 * 8, 0x4000 * 8, 0x2000 * 8 };
		static const uint32_t pr[4] = { 0x20000 * 8, 0, 0x30000 * 8, 0x10000 * 8 };
		static const uint32_t pt[4] = { 0x60000 * 8, 0x40000 * 8, 0x20000 * 8, 0 };
		const rbx_gfx_layout l8 = { 8, 8, 3, pc, x8, y8, 8 * 8 };
		const rbx_gfx_layout lr = { 16, 16, 4, pr, xr, y16, 16 * 16 };
		const rbx_gfx_layout lt = { 16, 16, 4, pt, x16, y16, 16 * 16 };
		if (rbx_gfx_decode(&s_gfx8, &l8, chars, 1024 * 8) ||
		    (go ? rbx_gfx_decode(&s_gfx16, &lt, tiles, 4096 * 32) : rbx_gfx_decode(&s_gfx16, &lr, tiles, 2048 * 32)) ||
		    rbx_gfx_decode(&s_gfxspr, &lt, spr, 4096 * 32)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			gb_exit();
			return -1;
		}
	}
	memset(&s_tile, 0, sizeof(s_tile));   // set_gfx_region_wide(1, 1, 0): tiles_r from 512
	s_tile.gfx16 = &s_gfx16;
	s_tile.base16 = 512;
	mcs51_init(&s_mcu, s_mcu_rom, 0x1000);
	s_mcu.port_in = mcu_in;
	s_mcu.port_out = mcu_out;
	s_snap = calloc(1, sizeof(*s_snap));
	// sound: YM2203 1.5 MHz (SSG 0.20, FM 0.40), YM3812 3 MHz 0.80 (Gondomania: YM3526 0.50; Garyo Retsuden: YM3526)
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM1_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = 51;
	yc.gain[3] = 102;
	yc.irq = ym1_irq;
	int rc = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	memset(&yc, 0, sizeof(yc));
	yc.type = go ? RBX_YM3526 : RBX_YM3812;
	yc.clock = YM2_CLOCK;
	yc.gain[0] = s_var == V_GONDO ? 128 : 205;
	yc.irq = ym2_irq;
	rc = rc || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	if (!rc) {
		s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
		s_ym2_cap = (int)(rbx_ym_rate(1) / 50) + 256;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		s_ym2buf = (int32_t *)calloc((size_t)s_ym2_cap, sizeof(int32_t));
		rc = !s_mix || !s_ym2buf || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 1024);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		gb_exit();
		return -1;
	}
	for (int i = 0; !go && i < 1024; i++) {   // palette_init_proms (the resistor weights of the RM-C3)
		static const uint8_t w[4] = { 0x0e, 0x1f, 0x43, 0x8f };
		const int r = s_prom[i] & 15, gg = s_prom[i] >> 4, b = s_prom[i + 1024] & 15;
		int rr = 0, g2 = 0, bb = 0;
		for (int k = 0; k < 4; k++) {
			rr += ((r >> k) & 1) * w[k];
			g2 += ((gg >> k) & 1) * w[k];
			bb += ((b >> k) & 1) * w[k];
		}
		s_palette[i] = (uint32_t)(rr << 16 | g2 << 8 | bb);
	}
	m6502_init(&s_snd, s_var == V_GHOSTB ? M6502_NMOS : M6502_R65C02);
	s_snd.deco222 = s_var == V_GHOSTB;
	for (int p = 0; p < 6; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = s_sram + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		if ((uint32_t)p * 256 + 256 <= s_zrom_size)
			s_snd.rpage[p] = s_zrom + p * 256;
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	m6809_init(&s_cpu);
	for (int p = 0; p < 0x38; p++)   // (RAM, text, (the background), RAM / palette, sprites; the BAC06 through the handlers)
		if (go || p < 0x20 || (p >= 0x28 && p < 0x2c) || p >= 0x30)
			s_cpu.rpage[p] = s_cpu.wpage[p] = s_ram + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + p * 256;
	s_cpu.read = go ? go_r : cpu_r;
	s_cpu.write = go ? go_w : cpu_w;
	s_port_in[0] = rbx_port_find("IN0");
	s_port_in[1] = rbx_port_find("IN1");
	s_port_in[2] = rbx_port_find("IN2");
	s_port_in[3] = rbx_port_find("IN3");
	s_port_coin = rbx_port_find("COIN");
	s_port_dsw[0] = rbx_port_find("DSW0");
	s_port_dsw[1] = rbx_port_find("DSW1");
	gb_reset();
	return 0;
}

static void gb_exit(void)
{
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix);
	free(s_ym2buf);
	s_mix = s_ym2buf = NULL;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
	free(s_pri_buf_hp);
	s_pri_buf_hp = NULL;
}

// the main CPU to the target, the NMI gate's latch applied after the instruction following its write
static void run_main(int64_t target)
{
	while (s_cpu.cycles < target) {   // (one instruction at a time: the latch write counted from its own instruction)
		m6809_run_until(&s_cpu, s_cpu.cycles + 1 < target ? s_cpu.cycles + 1 : target);
		if (s_vbl_nmi_delay && --s_vbl_nmi_delay == 0)
			update_nmi();
		if (s_nmi_delay && --s_nmi_delay == 0) {
			s_nmi_enable = s_nmi_next;
			update_nmi();
		}
	}
}

static void gb_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	if (s_var == V_GONDO)
		for (int p = 0; p < 2; p++) {   // (the rotary: the positional control, 12 a turn)
			s_rotary[p] = (s_rotary[p] + rbx_analog(p + 1, RBX_IPT_POSITIONAL)) % 12;
			if (s_rotary[p] < 0)
				s_rotary[p] += 12;
		}
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H || line == VIS_Y0) {   // vblank: the NMI gate, the MCU's INT0
			if (line != VIS_Y0) {
				vblank_capture();
				s_vbl_nmi_delay = 1;
			} else
				update_nmi();
			mcs51_set_input(&s_mcu, MCS51_INT0_LINE, vblank());
		}
		s_cpu_frac += (uint64_t)CPU_CLOCK * 1000;
		const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
		const int64_t cyc = (int64_t)(s_cpu_frac / den);
		s_cpu_frac -= (uint64_t)cyc * den;
		const int mc = rbx_line_cycles(&s_mcu_frac, MCU_CLOCK, FRAME_MHZ, LINES);
		const int64_t t0 = s_cpu.cycles;
		for (int k = 0; k < SLICES; k++) {   // (MAME: perfect sync between the CPU and the MCU)
			run_main(t0 + cyc * (k + 1) / SLICES);
			if (s_int1_until >= 0 && s_cpu.cycles >= s_int1_until) {
				mcs51_set_input(&s_mcu, MCS51_INT1_LINE, 0);
				s_int1_until = -1;
			}
			mcs51_execute(&s_mcu, mc * (k + 1) / SLICES - mc * k / SLICES);
		}
		sound_line();
	}
	sound_frame_end();
	s_frame++;
}

static int gb_state(void *ctx, int load)
{
	uint8_t cpu[M6809_STATE_SIZE], snd[M6502_STATE_SIZE];
	if (!load) {
		memcpy(cpu, &s_cpu, M6809_STATE_SIZE);
		memcpy(snd, &s_snd, M6502_STATE_SIZE);
	}
	uint64_t fr[5] = { s_cpu_frac, s_snd_frac, s_mcu_frac, s_ym1_frac, s_ym2_frac };
	int64_t until = s_int1_until;
	int32_t misc[17] = { (int32_t)s_frame, s_bank, s_bank_data, s_secclr, s_buffer_strobe, s_flip, s_nmi_enable,
	                     s_nmi_delay, s_nmi_next, s_irq, s_p0, s_p1, s_p2, s_mcu_return, s_mcu_value, s_latch,
	                     s_ym_irq[0] | s_ym_irq[1] << 1 };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m6809", cpu, M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "sprb", s_sprbuf, sizeof(s_sprbuf));
	rc |= rbx_state_io(ctx, "bac0", &s_tile, offsetof(bac06_t, gfx8));
	rc |= rbx_state_io(ctx, "scroll", s_scroll, sizeof(s_scroll));
	rc |= rbx_state_io(ctx, "rot", s_rotary, sizeof(s_rotary));
	rc |= rbx_state_io(ctx, "mcu", &s_mcu.st, sizeof(s_mcu.st));
	rc |= rbx_state_io(ctx, "mcuf", s_mcu.forced_inputs, sizeof(s_mcu.forced_inputs));
	rc |= rbx_state_io(ctx, "m6502", snd, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "int1", &until, sizeof(until));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		memcpy(&s_cpu, cpu, M6809_STATE_SIZE);
		memcpy(&s_snd, snd, M6502_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_mcu_frac = fr[2];
		s_ym1_frac = fr[3];
		s_ym2_frac = fr[4];
		s_int1_until = until;
		s_frame = (uint32_t)misc[0];
		set_bank(misc[1]);
		s_bank_data = misc[2];
		s_secclr = misc[3];
		s_buffer_strobe = misc[4];
		s_flip = misc[5];
		s_nmi_enable = misc[6];
		s_nmi_delay = misc[7];
		s_nmi_next = misc[8];
		s_irq = misc[9];
		s_p0 = (uint8_t)misc[10];
		s_p1 = (uint8_t)misc[11];
		s_p2 = (uint8_t)misc[12];
		s_mcu_return = (uint16_t)misc[13];
		s_mcu_value = (uint16_t)misc[14];
		s_latch = (uint8_t)misc[15];
		s_ym_irq[0] = misc[16] & 1;
		s_ym_irq[1] = (misc[16] >> 1) & 1;
	}
	return rc ? -1 : 0;
}

static int gb_save(void *ctx) { return gb_state(ctx, 0); }
static int gb_load(void *ctx) { return gb_state(ctx, 1); }
static int gb_supported(const char *machine)
{
	return !strcmp(machine, "gondo") || !strcmp(machine, "garyoret") || !strcmp(machine, "ghostb") || !strcmp(machine, "meikyuh");
}

const rbx_driver rbx_drv_dec8g = { "DECO dec8", gb_init, gb_exit, gb_reset, gb_frame, gb_save, gb_load,
                                   gb_supported, gb_render, NULL };
