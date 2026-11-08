/*  Data East "dec8" 8-bit boards, from MAME src/mame/dataeast/dec8.cpp, dec8_v.cpp (BSD-3-Clause, copyright-holders:
    Bryan McPhail): Last Mission, Shackled / Breywood, Captain Silver.
    Two 6809s at 1.5 MHz sharing their RAM (their IRQs raised and cleared by each other's writes, the FIRQ from the
    MCU), the i8751 (the command and return words, the coins on its INT0; MAME runs them in perfect sync: here
    interleaved by sixteenths of a line), a 16x16 background (scroll; Shackled: pens 0-3 behind and 4-15 over the
    sprites by the tiles' colour bit 3) and an 8x8 text layer, the Karnov sprites (a byte list copied to words by a
    write), the DECO RM-C3 palette. The sound: R65C02 at 1.5 MHz with YM2203 and YM3526 (on its IRQ), the latch on
    its NMI. Captain Silver: other input addresses, the vblank on the sub CPU's NMI, the coins polled by the MCU; its
    sound (as Pocket Gal's): the YMs without IRQ, an MSM5205 fed a byte at a time by the R65C02 on every second
    VCK's IRQ, a banked sound ROM.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/mcs51/mcs51.h"
#include "../../sound/msm5205u.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../video/deckarn.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 1500000        // 12 MHz / 8 (the 6809s' E clock)
#define SND_CLOCK 1500000
#define MCU_CLOCK (8000000 / 12) // i8751 machine cycles
#define YM1_CLOCK 1500000        // YM2203
#define YM2_CLOCK 3000000        // YM3526
#define FRAME_MHZ 57445          // set_raw(12 MHz / 2, 384, 0, 256, 272, 8, 248)
#define LINES 272
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 256
#define SLICES 16

enum { V_LASTMISN, V_SHACKLED, V_CSILVER };
static int s_var;
static uint8_t *s_rom, *s_bank_rom, *s_sub_rom, *s_mcu_rom;
static uint32_t s_rom_size, s_bank_size, s_sub_size, s_mcu_size;
static uint8_t s_ram[0x4000];              // 0x0000 .. 0x3fff of both CPUs (share1, palette, text, sprites, share2, bg)
static uint16_t s_sprbuf[0x400];           // buffer_spriteram16_w
static uint8_t s_scroll[4];
static int s_bank, s_flip, s_sub_reset;
static m6809_t s_cpu[2];
static int s_irq[2], s_firq[2];
static rbx_gfx s_gfx8, s_gfx16, s_gfxspr;
static int s_port_in[3], s_port_coin, s_port_dsw[2];
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac, s_snd_frac, s_mcu_frac, s_ym1_frac, s_ym2_frac;
static struct {
	uint8_t ram[0x4000];
	uint16_t spr[0x400];
	uint8_t scroll[4];
	int flip;
	uint32_t frame;
} *s_snap;

#define PAL_LO 0x1000   // (s_ram offsets)
#define PAL_HI 0x1400
#define TXT 0x2000
#define SPR 0x2800
#define BG 0x3800

// ---- the i8751 ----
static mcs51_t s_mcu;
static uint8_t s_p0, s_p1, s_p2;
static uint16_t s_mcu_return, s_mcu_value;
static int s_coin_state;

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
static void set_firq_both(void)
{
	for (int c = 0; c < 2; c++)
		m6809_set_firq(&s_cpu[c], s_firq[c]);
}
static void mcu_out(void *u, int port, uint8_t data)
{
	(void)u;
	if (port == 0) { s_p0 = data; return; }
	if (port == 1) { s_p1 = data; return; }
	if (port != 2)
		return;
	const uint8_t fall = (uint8_t)(~data & s_p2), rise = (uint8_t)(data & ~s_p2);   // lastmisn mcu_to_main_w
	s_p2 = data;
	if (fall & 0x10) {
		s_p0 = (uint8_t)(s_mcu_value >> 8);
		if (s_var != V_CSILVER)
			s_mcu.forced_inputs[0] = s_p0;   // (the MCU sets p0 to 0x00)
	}
	if (fall & 0x20) s_p1 = (uint8_t)(s_mcu_value & 0xff);
	if (fall & 0x40) s_mcu_return = (uint16_t)((s_mcu_return & 0xff) | (s_p0 << 8));
	if (fall & 0x80) s_mcu_return = (uint16_t)((s_mcu_return & 0xff00) | s_p1);
	if (rise & 0x04) {   // P22: FIRQ to both CPUs
		s_firq[0] = s_firq[1] = 1;
		set_firq_both();
	}
	if (!(data & 0x01) && s_var != V_CSILVER) mcs51_set_input(&s_mcu, MCS51_INT0_LINE, 0);
	if (!(data & 0x02)) mcs51_set_input(&s_mcu, MCS51_INT1_LINE, 0);
}
static void coin_update(void)   // (an input merger, any low, to INT0; Captain Silver: none)
{
	if (s_var == V_CSILVER)
		return;
	const int state = (rbx_port_read(s_port_coin) & 0xe0) != 0xe0;
	if (state && !s_coin_state)
		mcs51_set_input(&s_mcu, MCS51_INT0_LINE, 1);
	s_coin_state = state;
}

// ---- sound: R65C02, YM2203, YM3526 (Captain Silver: and the MSM5205) ----
static m6502_t s_snd;
static uint8_t s_sram[0x800];
static uint8_t *s_zrom;
static uint32_t s_zrom_size;
static uint8_t s_latch;
static int s_ym_irq[2];
static int32_t *s_mix, *s_ym2buf;
static int s_mix_n, s_mix_cap, s_ym2_n, s_ym2_cap;
static msm5205u_t s_msm;
static uint8_t s_msm_next;
static int s_msm_toggle, s_snd_bank;

static void snd_irq(void) { m6502_set_irq(&s_snd, s_ym_irq[0] | s_ym_irq[1]); }
static void ym1_irq(int state) { s_ym_irq[0] = state; snd_irq(); }
static void ym2_irq(int state) { s_ym_irq[1] = state; snd_irq(); }
static void msm_vck(msm5205u_t *m, void *user)   // csilver adpcm_int
{
	(void)user;
	s_msm_toggle ^= 1;
	if (s_msm_toggle)
		m6502_set_irq(&s_snd, 1);
	msm5205u_data_w(m, (uint8_t)(s_msm_next >> 4));
	s_msm_next = (uint8_t)(s_msm_next << 4);
}
static void set_snd_bank(int b)
{
	s_snd_bank = b & 1;
	for (int p = 0; p < 0x40; p++)
		s_snd.rpage[0x40 + p] = s_zrom + s_snd_bank * 0x4000 + p * 256;
}
static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a == 0x3000) {
		m6502_set_nmi(&s_snd, 0);
		return s_latch;
	}
	if (s_var == V_CSILVER && a == 0x3400) {   // adpcm_reset_r
		msm5205u_reset_w(&s_msm, 0);
		return 0;
	}
	if (a >= 0x8000) return a < s_zrom_size ? s_zrom[a] : 0;
	return 0;
}
static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a == 0x0800 || a == 0x0801) rbx_ym_write(0, a & 1, v);
	else if (a == 0x1000 || a == 0x1001) rbx_ym_write(1, a & 1, v);
	else if (s_var == V_CSILVER && a == 0x1800) {   // adpcm_data_w
		s_msm_next = v;
		m6502_set_irq(&s_snd, 0);
	} else if (s_var == V_CSILVER && a == 0x2000)
		set_snd_bank((v & 0x08) >> 3);
}
static void sound_line(void)
{
	s_snd_frac += (uint64_t)SND_CLOCK * 1000;
	const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
	const int64_t cyc = (int64_t)(s_snd_frac / den);
	s_snd_frac -= (uint64_t)cyc * den;
	m6502_run_until(&s_snd, s_snd.cycles + cyc);
	const int n = rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&s_ym1_frac, YM1_CLOCK, FRAME_MHZ, LINES), s_mix + s_mix_n,
	                             s_mix_cap - s_mix_n);
	if (s_var == V_CSILVER)   // (the MSM5205 in step with the sound CPU: its VCKs raise the IRQ)
		msm5205u_render(&s_msm, s_mix + s_mix_n, n, rbx_ym_rate(0), 205);   // 0.80
	s_mix_n += n;
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

// ---- bus (both 6809s: 0x1800 .. 0x18ff through here) ----
static int vblank(void) { return s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H; }
static void set_bank(int b)
{
	s_bank = b & 0x0f;
	const uint32_t nb = s_bank_size / 0x4000;
	const uint32_t o = (uint32_t)(nb ? (uint32_t)s_bank % nb : 0) * 0x4000;
	for (int p = 0; p < 0x40; p++)
		s_cpu[0].rpage[0x40 + p] = s_bank_rom + o + (uint32_t)p * 256;
}
static void set_sub_reset(int held)   // (Last Mission: the main CPU holds the sub CPU in reset)
{
	if (held && !s_sub_reset)
		m6809_reset(&s_cpu[1]);
	s_sub_reset = held;
}
static int has_mcu_regs(int cpu) { return s_var == V_LASTMISN || cpu == 1; }

static uint8_t cpu_r(void *ctx, uint16_t a)
{
	const int cpu = ctx != NULL;
	if (s_var == V_CSILVER) {   // (csilver main_map / sub_map)
		switch (a) {
		case 0x1800: return cpu ? 0 : (uint8_t)rbx_port_read(s_port_in[1]);
		case 0x1801: return cpu ? 0 : (uint8_t)rbx_port_read(s_port_in[0]);
		case 0x1803:
			rbx_port_custom(s_port_in[2], 0x80, vblank() ? 0x80 : 0);
			return (uint8_t)rbx_port_read(s_port_in[2]);
		case 0x1804: return (uint8_t)rbx_port_read(s_port_dsw[1]);
		case 0x1805: return (uint8_t)rbx_port_read(s_port_dsw[0]);
		case 0x1c00: return cpu ? 0 : (uint8_t)(s_mcu_return >> 8);
		case 0x1e00: return cpu ? 0 : (uint8_t)(s_mcu_return & 0xff);
		}
		return 0;
	}
	if (a >= 0x1800 && a < 0x1900) {
		switch (a) {
		case 0x1800: return (uint8_t)rbx_port_read(s_port_in[0]);
		case 0x1801: return (uint8_t)rbx_port_read(s_port_in[1]);
		case 0x1802:
			rbx_port_custom(s_port_in[2], 0x80, vblank() ? 0x80 : 0);
			return (uint8_t)rbx_port_read(s_port_in[2]);
		case 0x1803: return (uint8_t)rbx_port_read(s_port_dsw[0]);
		case 0x1804: return (uint8_t)rbx_port_read(s_port_dsw[1]);
		case 0x1806: return has_mcu_regs(cpu) ? (uint8_t)(s_mcu_return >> 8) : 0;
		case 0x1807: return has_mcu_regs(cpu) ? (uint8_t)(s_mcu_return & 0xff) : 0;
		}
		return 0;
	}
	return 0;   // (Last Mission's sub CPU: its sprite RAM write only)
}
static void cpu_w(void *ctx, uint16_t a, uint8_t v)
{
	const int cpu = ctx != NULL;
	switch (a) {
	case 0x1800: s_irq[1] = 0; m6809_set_irq(&s_cpu[1], 0); return;   // sub_irq_off_w
	case 0x1801: s_irq[0] = 0; m6809_set_irq(&s_cpu[0], 0); return;   // main_irq_off_w
	case 0x1802: s_firq[0] = s_firq[1] = 0; set_firq_both(); return;   // both_firq_off_w
	case 0x1803: s_irq[0] = 1; m6809_set_irq(&s_cpu[0], 1); return;   // main_irq_on_w
	case 0x1804: s_irq[1] = 1; m6809_set_irq(&s_cpu[1], 1); return;   // sub_irq_on_w
	case 0x1805:   // buffer_spriteram16_w (the byte list as words, big-endian)
		for (int i = 0; i < 0x400; i++)
			s_sprbuf[i] = (uint16_t)(s_ram[SPR + i * 2 + 1] | (s_ram[SPR + i * 2] << 8));
		return;
	case 0x180c: s_latch = v; m6502_set_nmi(&s_snd, 1); return;
	}
	if (s_var == V_CSILVER) {   // (the main CPU's: flip, scroll_w, control_w (the bank), the MCU)
		if (cpu)
			return;
		if (a == 0x1807) s_flip = v;
		else if (a >= 0x1808 && a < 0x180c) s_scroll[a - 0x1808] = v;
		else if (a == 0x180d) set_bank(v & 0x0f);
		else if (a == 0x180e) {
			s_mcu_value = (uint16_t)((s_mcu_value & 0xff) | (v << 8));
			if (s_p2 & 2)
				mcs51_set_input(&s_mcu, MCS51_INT1_LINE, 1);
		} else if (a == 0x180f)
			s_mcu_value = (uint16_t)((s_mcu_value & 0xff00) | v);
		return;
	}
	if (a == 0x1807) {
		s_flip = v;
		return;
	}
	const int scroll_ok = s_var == V_SHACKLED || cpu == 0;
	if (scroll_ok && a == 0x1809) s_scroll[1] = v;
	else if (scroll_ok && a == 0x180b) s_scroll[3] = v;
	else if (scroll_ok && a == 0x180d) {   // lastmisn_control_w / shackled_control_w
		set_bank(v & 0x0f);   // (Shackled: the sub CPU's writes too)
		s_scroll[0] = (v >> 5) & 1;
		s_scroll[2] = (v >> 6) & 1;
		if (s_var == V_LASTMISN)
			set_sub_reset(!(v & 0x80));
	} else if (has_mcu_regs(cpu) && a == 0x180e) {   // i8751_hi_w: SECIRQ when P21 is high
		s_mcu_value = (uint16_t)((s_mcu_value & 0xff) | (v << 8));
		if (s_p2 & 2)
			mcs51_set_input(&s_mcu, MCS51_INT1_LINE, 1);
	} else if (has_mcu_regs(cpu) && a == 0x180f)
		s_mcu_value = (uint16_t)((s_mcu_value & 0xff00) | v);
}

// ---- video ----
static inline uint8_t rmc3(int v) { return (uint8_t)(0x0e * (v & 1) + 0x1f * ((v >> 1) & 1) + 0x43 * ((v >> 2) & 1) + 0x8f * ((v >> 3) & 1)); }

static void d8_render(void)
{
	const uint8_t *ram = s_snap->ram;
	for (int i = 0; i < 1024; i++) {   // deco_rgb_decoder: xxxxBBBBGGGGRRRR (the low byte and the extension)
		const int raw = ram[PAL_LO + i] | (ram[PAL_HI + i] << 8);
		rbx_video.palette[i] = (uint32_t)(rmc3(raw & 15) << 16 | rmc3((raw >> 4) & 15) << 8 | rmc3((raw >> 8) & 15));
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, flip = s_snap->flip != 0;
	const int sx = ((s_snap->scroll[0] << 8) + s_snap->scroll[1]) & 0x1ff, sy = ((s_snap->scroll[2] << 8) + s_snap->scroll[3]) & 0x1ff;
	const int shackled = s_var == V_SHACKLED;
	// the background (32x32 of 16x16, lastmisn_scan_rows), drawn in Shackled's passes: pens 0-3 (all), 4-15 of the
	// tiles without colour bit 3, then the sprites, then 4-15 of the others
	for (int pass = 0; pass < (shackled ? 3 : 1); pass++) {
		if (pass == 2)
			deckarn_draw(&s_gfxspr, 256, s_snap->spr, 0x400, flip, bm, pitch, VIS_W, VIS_H, VIS_Y0);
		for (int y = 0; y < VIS_H; y++) {
			uint16_t *d = bm + y * pitch;
			const int pyu = (y + VIS_Y0 + sy) & 0x1ff, py = flip ? 511 - pyu : pyu, row = py >> 4;
			for (int x = 0; x < VIS_W;) {   // (a run of pixels in one tile: its lookup once)
				const int pu = (x + sx) & 0x1ff, px = flip ? 511 - pu : pu;
				int n = 16 - (pu & 15);
				if (n > VIS_W - x)
					n = VIS_W - x;
				const int col = px >> 4;
				const int idx = (col & 0x0f) + ((row & 0x0f) << 4) + ((col & 0x10) << 4) + ((row & 0x10) << 5);
				const int tile = ram[BG + idx * 2 + 1] | (ram[BG + idx * 2] << 8);
				const uint32_t code = rbx_wrap((uint32_t)(tile & 0xfff), s_gfx16.count);
				const uint8_t *src = s_gfx16.pix + (code * 16 + (uint32_t)(py & 15)) * 16;
				const int colour = tile >> 12, cat = (colour & 8) ? 1 : 0;
				const uint16_t base = (uint16_t)(768 + colour * 16);
				int tx = px & 15;
				const int dir = flip ? -1 : 1;
				for (int i = 0; i < n; i++, x++, tx += dir) {
					const uint8_t p = src[tx];
					if (shackled) {   // (transmask: layer 1 pens 0-3, layer 0 pens 4-15; category colour bit 3)
						const int layer0 = p >= 4;
						if (pass == 0 ? layer0 : pass == 1 ? (!layer0 || cat) : (!layer0 || !cat))
							continue;
					}
					d[x] = (uint16_t)(base + p);
				}
			}
		}
	}
	if (!shackled)
		deckarn_draw(&s_gfxspr, 256, s_snap->spr, 0x400, flip, bm, pitch, VIS_W, VIS_H, VIS_Y0);
	for (int y = 0; y < VIS_H; y++) {   // the text layer: 32x32 of 8x8, 3 bpp, colour bits 14-15, pen 0 clear
		uint16_t *d = bm + y * pitch;
		const int py = flip ? 255 - (y + VIS_Y0) : y + VIS_Y0;
		for (int x0 = 0; x0 < VIS_W; x0 += 8) {   // (a tile's 8 pixels: one lookup)
			const int px0 = flip ? 255 - x0 : x0;
			const int o = ((py >> 3) * 32 + (px0 >> 3)) * 2;
			const int tile = ram[TXT + o + 1] | (ram[TXT + o] << 8);
			const uint32_t code = rbx_wrap((uint32_t)(tile & 0xfff), s_gfx8.count);
			const uint8_t *src = s_gfx8.pix + (code * 8 + (uint32_t)(py & 7)) * 8;
			const uint16_t base = (uint16_t)(((tile & 0xc000) >> 14) * 8);
			for (int i = 0; i < 8 && x0 + i < VIS_W; i++) {
				const uint8_t p = src[flip ? 7 - i : i];
				if (p)
					d[x0 + i] = (uint16_t)(base + p);
			}
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->ram, s_ram, sizeof(s_ram));
	memcpy(s_snap->spr, s_sprbuf, sizeof(s_sprbuf));
	memcpy(s_snap->scroll, s_scroll, sizeof(s_scroll));
	s_snap->flip = s_flip;
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void d8_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_sprbuf, 0, sizeof(s_sprbuf));
	memset(s_scroll, 0, sizeof(s_scroll));
	s_flip = 0;
	s_sub_reset = 0;
	set_bank(0);
	for (int c = 0; c < 2; c++) {
		m6809_reset(&s_cpu[c]);
		s_irq[c] = s_firq[c] = 0;
		m6809_set_irq(&s_cpu[c], 0);
		m6809_set_firq(&s_cpu[c], 0);
	}
	s_p0 = s_p1 = 0;
	s_p2 = 0xff;
	s_mcu_return = s_mcu_value = 0;
	s_coin_state = 0;
	mcs51_reset(&s_mcu);
	s_mcu.forced_inputs[0] = 0;
	memset(s_sram, 0, sizeof(s_sram));
	s_latch = 0;
	s_ym_irq[0] = s_ym_irq[1] = 0;
	m6502_reset(&s_snd);
	m6502_set_irq(&s_snd, 0);
	m6502_set_nmi(&s_snd, 0);
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	if (s_var == V_CSILVER) {
		msm5205u_reset(&s_msm);
		s_msm_next = 0;
		s_msm_toggle = 0;
		set_snd_bank(0);
	}
	s_mix_n = s_ym2_n = 0;
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = s_snd_frac = s_mcu_frac = s_ym1_frac = s_ym2_frac = 0;
}

static void d8_exit(void);

static int d8_init(const rbx_game *g)
{
	s_var = !strcmp(g->machine, "shackled") ? V_SHACKLED : !strcmp(g->machine, "csilver") ? V_CSILVER : V_LASTMISN;
	uint32_t csize, tsize, ssize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_bank_rom = rbx_region("mainbank", &s_bank_size);
	s_sub_rom = rbx_region("sub", &s_sub_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_mcu_rom = rbx_region("mcu", &s_mcu_size);
	uint8_t *chars = rbx_region("char", &csize), *tiles = rbx_region("tiles1", &tsize), *spr = rbx_region("sprites", &ssize);
	if (!s_rom || s_rom_size < 0x10000 || !s_bank_rom || !s_sub_rom || s_sub_size < 0x10000 || !s_zrom || !s_mcu_rom ||
	    s_mcu_size < 0x1000 || !chars || csize < 0x8000 || !tiles || tsize < 0x80000 || !spr || ssize < 0x80000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{	// chars_3bpp (1024: planes at 0x6000, 0x4000, 0x2000 bytes), tiles (4096: planes at 0x60000 .. 0)
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 8);
			x16[i] = (uint32_t)(16 * 8 + i);
			x16[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 8);
		static const uint32_t pc[3] = { 0x6000 * 8, 0x4000 * 8, 0x2000 * 8 };
		static const uint32_t pt[4] = { 0x60000 * 8, 0x40000 * 8, 0x20000 * 8, 0 };
		const rbx_gfx_layout l8 = { 8, 8, 3, pc, x8, y8, 8 * 8 };
		const rbx_gfx_layout lt = { 16, 16, 4, pt, x16, y16, 16 * 16 };
		if (rbx_gfx_decode(&s_gfx8, &l8, chars, 1024 * 8) || rbx_gfx_decode(&s_gfx16, &lt, tiles, 4096 * 32) ||
		    rbx_gfx_decode(&s_gfxspr, &lt, spr, 4096 * 32)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			d8_exit();
			return -1;
		}
	}
	mcs51_init(&s_mcu, s_mcu_rom, 0x1000);
	s_mcu.port_in = mcu_in;
	s_mcu.port_out = mcu_out;
	s_snap = calloc(1, sizeof(*s_snap));
	// sound: YM2203 1.5 MHz (SSG 0.20, FM 0.40), YM3526 3 MHz 0.80 (Captain Silver: 0.10, 0.20, 0.40; no IRQs)
	const int cs = s_var == V_CSILVER;
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM1_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = cs ? 26 : 51;
	yc.gain[3] = cs ? 51 : 102;
	yc.irq = cs ? NULL : ym1_irq;
	int rc = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM3526;
	yc.clock = YM2_CLOCK;
	yc.gain[0] = cs ? 102 : 205;
	yc.irq = cs ? NULL : ym2_irq;
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
		d8_exit();
		return -1;
	}
	m6502_init(&s_snd, M6502_R65C02);
	for (int p = 0; p < (cs ? 8 : 6); p++)
		s_snd.rpage[p] = s_snd.wpage[p] = s_sram + p * 256;
	if (cs)   // (MSM5205 384 kHz, S48_4B: 8 kHz)
		msm5205u_init(&s_msm, 384000, MSM5205_S48_4B, msm_vck, NULL);
	for (int p = 0x80; p < 0x100; p++)
		if ((uint32_t)p * 256 + 256 <= s_zrom_size)
			s_snd.rpage[p] = s_zrom + p * 256;
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	for (int c = 0; c < 2; c++) {
		m6809_t *m = &s_cpu[c];
		m6809_init(m);
		for (int p = 0; p < 0x40; p++)   // (the shared RAM; the I/O page 0x18 through the handlers)
			if (p != 0x18) {
				m->rpage[p] = s_ram + p * 256;
				m->wpage[p] = s_ram + p * 256;
			}
		if (cs)   // (the I/O at 0x1c00 / 0x1e00)
			m->rpage[0x1c] = m->rpage[0x1e] = NULL, m->wpage[0x1c] = m->wpage[0x1e] = NULL;
		if (c == 1 && s_var == V_LASTMISN)
			for (int p = 0x28; p < 0x30; p++)   // (Last Mission's sub CPU: the sprite RAM write only)
				m->rpage[p] = NULL;
		for (int p = c ? 0x40 : 0x80; p < 0x100; p++)
			m->rpage[p] = (c ? s_sub_rom : s_rom) + p * 256;
		m->read = cpu_r;
		m->write = cpu_w;
		m->ctx = c ? (void *)&s_cpu[1] : NULL;
	}
	s_port_in[0] = rbx_port_find("IN0");
	s_port_in[1] = rbx_port_find("IN1");
	s_port_in[2] = rbx_port_find("IN2");
	s_port_coin = rbx_port_find("COIN");
	s_port_dsw[0] = rbx_port_find("DSW0");
	s_port_dsw[1] = rbx_port_find("DSW1");
	d8_reset();
	return 0;
}

static void d8_exit(void)
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
}

static void d8_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	coin_update();
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {
			vblank_capture();
			if (s_var == V_CSILVER)   // (screen_vblank on the sub CPU's NMI)
				m6809_set_nmi(&s_cpu[1], 1);
		} else if (line == VIS_Y0 && s_var == V_CSILVER)
			m6809_set_nmi(&s_cpu[1], 0);
		s_cpu_frac += (uint64_t)CPU_CLOCK * 1000;
		const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
		const int64_t cyc = (int64_t)(s_cpu_frac / den);
		s_cpu_frac -= (uint64_t)cyc * den;
		const int mc = rbx_line_cycles(&s_mcu_frac, MCU_CLOCK, FRAME_MHZ, LINES);
		const int64_t t0 = s_cpu[0].cycles, t1 = s_cpu[1].cycles;
		for (int k = 0; k < SLICES; k++) {   // (MAME: perfect sync between the CPUs and the MCU)
			const int64_t step = cyc * (k + 1) / SLICES;
			m6809_run_until(&s_cpu[0], t0 + step);
			if (s_sub_reset)
				s_cpu[1].cycles = t1 + step;
			else
				m6809_run_until(&s_cpu[1], t1 + step);
			mcs51_execute(&s_mcu, mc * (k + 1) / SLICES - mc * k / SLICES);
		}
		sound_line();
	}
	sound_frame_end();
	s_frame++;
}

static int d8_state(void *ctx, int load)
{
	uint8_t cpu[2][M6809_STATE_SIZE], snd[M6502_STATE_SIZE];
	if (!load) {
		for (int c = 0; c < 2; c++)
			memcpy(cpu[c], &s_cpu[c], M6809_STATE_SIZE);
		memcpy(snd, &s_snd, M6502_STATE_SIZE);
	}
	uint64_t fr[5] = { s_cpu_frac, s_snd_frac, s_mcu_frac, s_ym1_frac, s_ym2_frac };
	int32_t misc[16] = { (int32_t)s_frame, s_bank, s_flip, s_sub_reset, s_irq[0], s_irq[1], s_firq[0], s_firq[1],
	                     s_p0, s_p1, s_p2, s_mcu_return, s_mcu_value, s_coin_state, s_latch,
	                     s_ym_irq[0] | s_ym_irq[1] << 1 };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m6809a", cpu[0], M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "m6809b", cpu[1], M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "sprb", s_sprbuf, sizeof(s_sprbuf));
	rc |= rbx_state_io(ctx, "scroll", s_scroll, sizeof(s_scroll));
	rc |= rbx_state_io(ctx, "mcu", &s_mcu.st, sizeof(s_mcu.st));
	rc |= rbx_state_io(ctx, "mcuf", s_mcu.forced_inputs, sizeof(s_mcu.forced_inputs));
	rc |= rbx_state_io(ctx, "m6502", snd, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	if (s_var == V_CSILVER) {
		int32_t ms[3] = { s_msm_next, s_msm_toggle, s_snd_bank };
		rc |= rbx_state_io(ctx, "msm", &s_msm, MSM5205_STATE_SIZE);
		rc |= rbx_state_io(ctx, "msmx", ms, sizeof(ms));
		if (!rc && load) {
			s_msm_next = (uint8_t)ms[0];
			s_msm_toggle = ms[1];
			set_snd_bank(ms[2]);
		}
	}
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		for (int c = 0; c < 2; c++)
			memcpy(&s_cpu[c], cpu[c], M6809_STATE_SIZE);
		memcpy(&s_snd, snd, M6502_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_mcu_frac = fr[2];
		s_ym1_frac = fr[3];
		s_ym2_frac = fr[4];
		s_frame = (uint32_t)misc[0];
		set_bank(misc[1]);
		s_flip = misc[2];
		s_sub_reset = misc[3];
		s_irq[0] = misc[4];
		s_irq[1] = misc[5];
		s_firq[0] = misc[6];
		s_firq[1] = misc[7];
		s_p0 = (uint8_t)misc[8];
		s_p1 = (uint8_t)misc[9];
		s_p2 = (uint8_t)misc[10];
		s_mcu_return = (uint16_t)misc[11];
		s_mcu_value = (uint16_t)misc[12];
		s_coin_state = misc[13];
		s_latch = (uint8_t)misc[14];
		s_ym_irq[0] = misc[15] & 1;
		s_ym_irq[1] = (misc[15] >> 1) & 1;
	}
	return rc ? -1 : 0;
}

static int d8_save(void *ctx) { return d8_state(ctx, 0); }
static int d8_load(void *ctx) { return d8_state(ctx, 1); }
static int d8_supported(const char *machine)
{
	return !strcmp(machine, "lastmisn") || !strcmp(machine, "shackled") || !strcmp(machine, "csilver");
}

// the other dec8 boards (one gen table): their drivers
extern const rbx_driver rbx_drv_dec8o, rbx_drv_dec8s, rbx_drv_dec8g;
static const rbx_driver *const s_others[] = { &rbx_drv_dec8o, &rbx_drv_dec8s, &rbx_drv_dec8g };
static const rbx_driver *s_other;

static int dx_init(const rbx_game *g)
{
	s_other = NULL;
	for (int i = 0; i < (int)(sizeof(s_others) / sizeof(s_others[0])); i++)
		if (s_others[i]->supported(g->machine))
			s_other = s_others[i];
	return s_other ? s_other->init(g) : d8_init(g);
}
static void dx_exit(void) { if (s_other) s_other->exit(); else d8_exit(); }
static void dx_reset(void) { if (s_other) s_other->reset(); else d8_reset(); }
static void dx_frame(void) { if (s_other) s_other->run_frame(); else d8_frame(); }
static int dx_save(void *ctx) { return s_other ? s_other->state_save(ctx) : d8_save(ctx); }
static int dx_load(void *ctx) { return s_other ? s_other->state_load(ctx) : d8_load(ctx); }
static void dx_render(void) { if (s_other) s_other->render(); else d8_render(); }
static int dx_supported(const char *machine)
{
	for (int i = 0; i < (int)(sizeof(s_others) / sizeof(s_others[0])); i++)
		if (s_others[i]->supported(machine))
			return 1;
	return d8_supported(machine);
}

const rbx_driver rbx_drv_dec8 = { "DECO dec8", dx_init, dx_exit, dx_reset, dx_frame, dx_save, dx_load,
                                  dx_supported, dx_render, NULL };
