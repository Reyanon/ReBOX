/*  Data East "dec1" boards of src/mame/dataeast/dec0.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail, David
    Haywood): Sly Spy / Secret Agent, Boulder Dash, Midnight Resistance. Run through the dec0 driver (dec0.c), whose
    game table it shares.
    68000 at 10 MHz (IRQ 6 held at vblank), three BAC06 tilemap chips (Sly Spy: the first two at addresses its
    protection moves round four layouts), the MXC06 sprites (no DMA), xBGR_444 palette, the sound: HuC6280 (Sly Spy:
    its program encrypted, its chips at addresses a protection counter moves) with YM2203, YM3812 (on its IRQ2) and an
    OKI, the latch on its NMI. Midnight Resistance's 12-way rotary controls. Its bootlegs (midresb, midresbj): the
    dec0 boards' sound (R65C02 1.5 MHz, the YM3812 on its IRQ), dec0's control reads and latch, the second and third
    chips without row / column scroll (their 68705 is not connected in MAME).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/h6280/h6280.h"
#include "../../cpu/m6502/m6502.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../video/bac06.h"
#include "../video/decmxc06.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 10000000       // 20 MHz / 2
#define YM1_CLOCK 1500000        // YM2203: 12 MHz / 8
#define YM2_CLOCK 3000000        // YM3812: 12 MHz / 4
#define FRAME_MHZ 57445          // set_raw(12 MHz / 2, 384, 0, 256, 272, 8, 248)
#define LINES 272
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 256

enum { V_SLYSPY, V_MIDRES };
static int s_var;
static int s_mb;                           // midresb / midresbj
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint16_t (*s_ram_hp)[0x2000];
#define s_ram (*s_ram_hp)
static uint16_t s_spriteram[0x400];
static uint16_t s_pal[0x400];              // xBGR_444
static uint16_t s_pri;
static int s_prot_state, s_sound_prot_state;   // (Sly Spy)
static bac06_t (*s_tile_hp)[3];
#define s_tile (*s_tile_hp)
static decmxc06_t s_spr;
static rbx_gfx s_gfx8, s_gfx16a, s_gfx16b, s_gfxspr;
static int s_port_in, s_port_sys, s_port_dsw;
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac, s_snd_frac, s_ym1_frac, s_ym2_frac;
static uint8_t (*s_pri_buf_hp)[VIS_W * VIS_H];
#define s_pri_buf (*s_pri_buf_hp)
static int s_rotary[2];                    // (Midnight Resistance: 0 .. 11, before the reverse)
static rbx_rotary s_rot[2];
// (a step of the value clockwise turns the player 45 degrees clockwise. Its facing, x4, 0 = up: RAM 0x1021bc /
// 0x102238, midresj 0x1021be / 0x10223a; found with the harness. -1: none, the bootleg with a joystick and midres2)
static int s_face[2];
static int mr_facing(int p) { return s_face[p] < 0 ? -1 : (s_ram[s_face[p] >> 1] >> 10) & 7; }
static struct {
	bac06_t tile[3];
	uint16_t spriteram[0x400], pal[0x400], pri;
	uint32_t frame;
} *s_snap;

// ---- sound: HuC6280, YM2203, YM3812, OKI ----
static h6280_t s_snd;
static uint32_t s_snd_clock;
static uint8_t s_sram[0x2000];
static okim6295_t s_oki;
static uint8_t s_latch;
static uint32_t s_oki_acc;
static int32_t s_oki_hold;
static int32_t *s_mix, *s_ym2buf;
static int s_mix_n, s_mix_cap, s_ym2_n, s_ym2_cap;

// the bootlegs' R65C02 (dec0_s_map)
static m6502_t s_snd65;
static uint8_t s_sram65[0x800];
static const uint8_t *s_snd_rom;
static uint32_t s_snd_rom_size;

static void ym2_irq(int state)
{
	if (s_mb)
		m6502_set_irq(&s_snd65, state);   // (midresb: the YM3812 on IRQ 0)
	else
		h6280_set_irq(&s_snd, H6280_IRQ2, state);
}
static void latch_w(uint8_t v)
{
	s_latch = v;
	if (s_mb)
		m6502_set_nmi(&s_snd65, 1);
	else
		h6280_set_nmi(&s_snd, 1);
}
static uint8_t latch_r(void)
{
	if (s_mb)
		m6502_set_nmi(&s_snd65, 0);
	else
		h6280_set_nmi(&s_snd, 0);
	return s_latch;
}
static uint8_t snd65_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x0800) return s_sram65[a];
	if (a == 0x0800 || a == 0x0801) return rbx_ym_read(0, a & 1);
	if (a == 0x1000 || a == 0x1001) return rbx_ym_read(1, a & 1);
	if (a == 0x3000) return latch_r();
	if (a == 0x3800) return oki_read(&s_oki);
	if (a >= 0x8000) return a < s_snd_rom_size ? s_snd_rom[a] : 0;
	return 0;
}
static void snd65_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0800) s_sram65[a] = v;
	else if (a == 0x0800 || a == 0x0801) rbx_ym_write(0, a & 1, v);
	else if (a == 0x1000 || a == 0x1001) rbx_ym_write(1, a & 1, v);
	else if (a == 0x3800) oki_write(&s_oki, v);
}

// Sly Spy's sound protection: the chips' places in each of the four states (MAME's m_sndview)
enum { SND_NONE, SND_YM1, SND_YM2, SND_OKI, SND_LATCH };
static int slyspy_snd_dev(uint32_t pa)
{
	static const uint8_t map[4][8] = {   // (0x080000 .. 0x0fffff by 0x10000)
		{ 0, SND_YM2, 0, SND_YM1, 0, 0, SND_OKI, SND_LATCH },
		{ 0, SND_OKI, 0, 0, SND_LATCH, 0, SND_YM1, SND_YM2 },
		{ 0, SND_LATCH, 0, SND_OKI, SND_YM1, 0, 0, SND_YM2 },
		{ 0, SND_YM2, 0, 0, SND_YM1, 0, SND_LATCH, SND_OKI },
	};
	if (pa < 0x080000 || pa >= 0x100000 || (pa & 0xfffe) != 0)
		return SND_NONE;
	return map[s_sound_prot_state & 3][(pa >> 16) & 7];
}
static uint8_t snd_r(void *ctx, uint32_t pa)
{
	(void)ctx;
	if (s_var == V_SLYSPY) {
		if (pa == 0x0a0000 || pa == 0x0a0001) {   // the protection counter
			s_sound_prot_state = (s_sound_prot_state + 1) & 3;
			return 0xff;
		}
		if (pa == 0x0d0000 || pa == 0x0d0001) {
			s_sound_prot_state = 0;
			return 0xff;
		}
		switch (slyspy_snd_dev(pa & ~1u)) {
		case SND_OKI: return oki_read(&s_oki);
		case SND_LATCH: return latch_r();
		}
		return 0;
	}
	if (pa >= 0x130000 && pa < 0x130002) return oki_read(&s_oki);
	if (pa >= 0x138000 && pa < 0x138002) return latch_r();
	return 0;
}
static void snd_w(void *ctx, uint32_t pa, uint8_t v)
{
	(void)ctx;
	int dev;
	if (s_var == V_SLYSPY)
		dev = slyspy_snd_dev(pa & ~1u);
	else
		dev = (pa & ~1u) == 0x108000 ? SND_YM2 : (pa & ~1u) == 0x118000 ? SND_YM1 : (pa & ~1u) == 0x130000 ? SND_OKI : SND_NONE;
	switch (dev) {
	case SND_YM1: rbx_ym_write(0, pa & 1, v); break;
	case SND_YM2: rbx_ym_write(1, pa & 1, v); break;
	case SND_OKI: oki_write(&s_oki, v); break;
	}
}

static void sound_line(void)
{
	s_snd_frac += (uint64_t)s_snd_clock * 1000;
	const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
	const int64_t cyc = (int64_t)(s_snd_frac / den);
	s_snd_frac -= (uint64_t)cyc * den;
	if (s_mb)
		m6502_run_until(&s_snd65, s_snd65.cycles + cyc);
	else
		h6280_run_until(&s_snd, s_snd.cycles + cyc);
	const int n = rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&s_ym1_frac, YM1_CLOCK, FRAME_MHZ, LINES), s_mix + s_mix_n,
	                             s_mix_cap - s_mix_n);
	s_ym2_n += rbx_ym_advance(1, (uint32_t)rbx_line_cycles(&s_ym2_frac, YM2_CLOCK, FRAME_MHZ, LINES), s_ym2buf + s_ym2_n,
	                          s_ym2_cap - s_ym2_n);
	const uint32_t yr = rbx_ym_rate(0);
	for (int i = 0; i < n; i++) {
		s_oki_acc += s_oki.rate;
		while (s_oki_acc >= yr) {
			s_oki_acc -= yr;
			s_oki_hold = oki_sample(&s_oki);
		}
		s_mix[s_mix_n + i] += s_oki_hold;
	}
	s_mix_n += n;
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
static uint16_t system_r(void)
{
	const uint32_t vbit = s_mb ? 0x0080 : 0x0008;   // (midresb: dec0's SYSTEM port)
	rbx_port_custom(s_port_sys, vbit, vblank() ? vbit : 0);
	return (uint16_t)rbx_port_read(s_port_sys);
}
static uint16_t rotary(int p)
{
	static const uint16_t table[12] = { 0xfffe, 0xfffd, 0xfffb, 0xfff7, 0xffef, 0xffdf, 0xffbf, 0xff7f,
	                                    0xfeff, 0xfdff, 0xfbff, 0xf7ff };
	return table[(12 - s_rotary[p]) % 12];
}
static void wmask(uint16_t *p, uint16_t v, uint16_t mask) { *p = (uint16_t)((*p & ~mask) | (v & mask)); }

// a chip's registers and RAM at its base (ctrl +0, scroll +0x10, column scroll +cs, row scroll +rs, VRAM +vr)
static int chip_w(bac06_t *t, uint32_t o, uint32_t cs, uint32_t rs, uint32_t vr, uint32_t vsize, uint16_t v, uint16_t mask)
{
	if (o < 0x8) bac06_ctrl_w(t, (int)(o >> 1), v, mask);
	else if (o >= 0x10 && o < 0x18) bac06_scroll_w(t, (int)((o - 0x10) >> 1), v, mask);
	else if (cs && o >= cs && o < cs + 0x80) wmask(&t->colscroll[(o - cs) >> 1], v, mask);
	else if (rs && o >= rs && o < rs + 0x400) wmask(&t->rowscroll[(o - rs) >> 1], v, mask);
	else if (o >= vr && o < vr + vsize) bac06_vram_w(t, (int)(((o - vr) & (vsize - 1)) >> 1), v, mask);
	else return 0;
	return 1;
}
static int chip_r(const bac06_t *t, uint32_t o, uint32_t cs, uint32_t rs, uint32_t vr, uint32_t vsize, uint16_t *v)
{
	if (cs && o >= cs && o < cs + 0x80) *v = t->colscroll[(o - cs) >> 1];
	else if (rs && o >= rs && o < rs + 0x400) *v = t->rowscroll[(o - rs) >> 1];
	else if (o >= vr && o < vr + vsize) *v = bac06_vram_r(t, (int)(((o - vr) & (vsize - 1)) >> 1));
	else return 0;
	return 1;
}

// Sly Spy: 0x240000 .. 0x24ffff by the protection's state (writes only)
static void slyspy_pf_w(uint32_t a, uint16_t v, uint16_t mask)
{
	bac06_t *t0 = &s_tile[0], *t1 = &s_tile[1];
	switch (s_prot_state) {
	case 0:
		if (a < 0x248000) chip_w(t1, a - 0x240000, 0x2000, 0x2400, 0x6000, 0x2000, v, mask);
		else chip_w(t0, a - 0x248000, 0x4000, 0x4400, 0x6000, 0x2000, v, mask);
		break;
	case 1:
		if (a >= 0x248000 && a < 0x24a000) bac06_vram_w(t0, (int)((a - 0x248000) >> 1), v, mask);
		else if (a >= 0x24c000 && a < 0x24e000) bac06_vram_w(t1, (int)((a - 0x24c000) >> 1), v, mask);
		break;
	case 2:
		if (a < 0x242000) bac06_vram_w(t1, (int)((a - 0x240000) >> 1), v, mask);
		else if (a < 0x244000) bac06_vram_w(t0, (int)((a - 0x242000) >> 1), v, mask);
		else if (a >= 0x24e000) bac06_vram_w(t0, (int)((a - 0x24e000) >> 1), v, mask);
		break;
	case 3:
		if (a < 0x242000) bac06_vram_w(t0, (int)((a - 0x240000) >> 1), v, mask);
		else if (a >= 0x248000 && a < 0x24a000) bac06_vram_w(t1, (int)((a - 0x248000) >> 1), v, mask);
		break;
	}
}

static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	uint16_t v;
	if (s_var == V_SLYSPY) {
		if (a == 0x244000) {   // prot_state_r
			s_prot_state = (s_prot_state + 1) & 3;
			return 0;
		}
		if (a >= 0x300000 && a < 0x302000 && chip_r(&s_tile[2], a - 0x300000, 0x800, 0xc00, 0x1000, 0x800, &v)) return v;
		if (a >= 0x314008 && a < 0x314010) {   // slyspy_controls_r
			switch (a & 6) {
			case 0: return (uint16_t)rbx_port_read(s_port_dsw);
			case 2: return (uint16_t)rbx_port_read(s_port_in);
			case 4: return system_r();
			}
			return 0xffff;
		}
		if (a >= 0x31c000 && a < 0x31c010) {   // prot_r (Boulder Dash's values; Sly Spy's RNG as the bootleg)
			switch (a & 0xe) {
			case 0x2: return 0x13;
			case 0x6: return 0x2;
			case 0xc: return (uint16_t)(s_ram[0x2028 / 2] >> 8);
			}
			return 0;
		}
		return 0;
	}
	if (s_mb) {
		if (a >= 0x180000 && a < 0x180010) {   // dec0_controls_r
			switch (a & 0xe) {
			case 0x0: return (uint16_t)rbx_port_read(s_port_in);
			case 0x2: return system_r();
			case 0x4: return (uint16_t)rbx_port_read(s_port_dsw);
			case 0x8: return 0;
			}
			return 0xffff;
		}
		if (a == 0x1a0000) return rotary(0);   // AN0
		if (a == 0x1a0008) return rotary(1);   // AN1
	}
	if (a >= 0x180000 && a < 0x180010) {   // midres_controls_r
		switch (a & 0xe) {
		case 0x0: return (uint16_t)rbx_port_read(s_port_in);
		case 0x2: return (uint16_t)rbx_port_read(s_port_dsw);
		case 0x4: return rotary(0);
		case 0x6: return rotary(1);
		case 0x8: return system_r();
		}
		return 0;
	}
	// (the column / row scroll at +0 / +0x400: offset by 0x20 past the registers for chip_r)
	if (a >= 0x220000 && a < 0x221000) return bac06_vram_r(&s_tile[1], (int)(((a - 0x220000) & 0x7ff) >> 1));
	if (a >= 0x240000 && a < 0x240800 && chip_r(&s_tile[1], a - 0x240000 + 0x20, 0x20, 0x420, 0x10000, 0, &v)) return v;
	if (a >= 0x2a0000 && a < 0x2a0800) return bac06_vram_r(&s_tile[2], (int)((a - 0x2a0000) >> 1));
	if (a >= 0x2c0000 && a < 0x2c0800 && chip_r(&s_tile[2], a - 0x2c0000 + 0x20, 0x20, 0x420, 0x10000, 0, &v)) return v;
	if (a >= 0x320000 && a < 0x322000) return bac06_vram_r(&s_tile[0], (int)((a - 0x320000) >> 1));
	if (a >= 0x340000 && a < 0x340800 && chip_r(&s_tile[0], a - 0x340000 + 0x20, 0x20, 0x420, 0x10000, 0, &v)) return v;
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
	if (s_var == V_SLYSPY) {
		if (a == 0x24a000) s_prot_state = 0;   // prot_state_w
		else if (a >= 0x240000 && a < 0x250000) slyspy_pf_w(a, v, mask);
		else if (a >= 0x300000 && a < 0x302000) chip_w(&s_tile[2], a - 0x300000, 0x800, 0xc00, 0x1000, 0x800, v, mask);
		else if (a == 0x314000) { if (mask & 0xff) latch_w((uint8_t)v); }
		else if (a == 0x314002) wmask(&s_pri, v, mask);
		return;
	}
	if (a == 0x160000 || (s_mb && a == 0x160010)) wmask(&s_pri, v, mask);
	else if (s_mb && a == 0x180014) { if (mask & 0xff) latch_w((uint8_t)v); }
	else if (a == 0x1a0000) { if (mask & 0xff) latch_w((uint8_t)v); }
	else if (a >= 0x200000 && a < 0x200018) chip_w(&s_tile[1], a - 0x200000, 0, 0, 0x10000, 0, v, mask);
	else if (a >= 0x220000 && a < 0x221000) bac06_vram_w(&s_tile[1], (int)(((a - 0x220000) & 0x7ff) >> 1), v, mask);
	else if (a >= 0x240000 && a < 0x240800) chip_w(&s_tile[1], a - 0x240000 + 0x20, 0x20, 0x420, 0x10000, 0, v, mask);
	else if (a >= 0x280000 && a < 0x280018) chip_w(&s_tile[2], a - 0x280000, 0, 0, 0x10000, 0, v, mask);
	else if (a >= 0x2a0000 && a < 0x2a0800) bac06_vram_w(&s_tile[2], (int)((a - 0x2a0000) >> 1), v, mask);
	else if (a >= 0x2c0000 && a < 0x2c0800) chip_w(&s_tile[2], a - 0x2c0000 + 0x20, 0x20, 0x420, 0x10000, 0, v, mask);
	else if (a >= 0x300000 && a < 0x300018) chip_w(&s_tile[0], a - 0x300000, 0, 0, 0x10000, 0, v, mask);
	else if (a >= 0x320000 && a < 0x322000) bac06_vram_w(&s_tile[0], (int)((a - 0x320000) >> 1), v, mask);
	else if (a >= 0x340000 && a < 0x340800) chip_w(&s_tile[0], a - 0x340000 + 0x20, 0x20, 0x420, 0x10000, 0, v, mask);
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static void midres_colpri(void *u, uint32_t *colour, uint32_t *pri_mask)
{
	(void)u;
	*pri_mask = 0;
	if (s_snap->pri & 2) {
		const uint32_t trans = (s_snap->pri & 4) ? 0x00 : 0x08;
		if ((*colour & 0x08) == trans)
			*pri_mask |= 0xcc;   // (GFX_PMASK_2: behind the foreground)
	}
}
static void baddudes_tile_cb(uint32_t *tile, uint32_t *colour, uint32_t *flags)
{
	(void)tile;
	if (*colour & 8)
		*flags |= BAC06_GROUP1;
}

static void d1_render(void)
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
	const uint16_t pri = s_snap->pri;
	if (s_var == V_SLYSPY) {
		bac06_draw(t2, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, BAC06_OPAQUE, 0, 0xff);
		bac06_draw(t1, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, BAC06_LAYER0 | BAC06_LAYER1, 0, 0xff);
		decmxc06_draw(&spr, s_snap->spriteram, 0x400, bm, NULL, pitch, VIS_W, VIS_H, VIS_Y0, s_snap->frame);
		if (pri & 0x80)   // (the top 8 pens of the top 8 palettes over the sprites)
			bac06_draw(t1, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, BAC06_LAYER0, 0, 0xff);
		bac06_draw(t0, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
		return;
	}
	memset(s_pri_buf, 0, sizeof(s_pri_buf));   // screen_update_robocop
	bac06_t *fg = (pri & 1) ? t2 : t1, *bg = (pri & 1) ? t1 : t2;
	bac06_draw(bg, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, BAC06_OPAQUE, 1, 0xff);
	bac06_draw(fg, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 2, 0xff);
	decmxc06_draw(&spr, s_snap->spriteram, 0x400, bm, s_pri_buf, pitch, VIS_W, VIS_H, VIS_Y0, s_snap->frame);
	bac06_draw(t0, bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->tile, s_tile, sizeof(s_tile));
	memcpy(s_snap->spriteram, s_spriteram, sizeof(s_spriteram));   // (dec0_nodma: the live list)
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	s_snap->pri = s_pri;
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void d1_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_pal, 0, sizeof(s_pal));
	s_pri = 0;
	s_prot_state = s_sound_prot_state = 0;
	for (int i = 0; i < 3; i++) {
		memset(s_tile[i].vram, 0, sizeof(s_tile[i].vram));
		memset(s_tile[i].rowscroll, 0, sizeof(s_tile[i].rowscroll));
		memset(s_tile[i].colscroll, 0, sizeof(s_tile[i].colscroll));
		bac06_reset(&s_tile[i]);
	}
	memset(s_sram, 0, sizeof(s_sram));
	s_latch = 0;
	if (s_mb) {
		memset(s_sram65, 0, sizeof(s_sram65));
		m6502_reset(&s_snd65);
		m6502_set_irq(&s_snd65, 0);
		m6502_set_nmi(&s_snd65, 0);
	} else {
		h6280_reset(&s_snd);
		h6280_set_nmi(&s_snd, 0);
	}
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	oki_reset(&s_oki);
	s_oki_acc = 0;
	s_oki_hold = 0;
	s_mix_n = s_ym2_n = 0;
	s_rotary[0] = s_rotary[1] = 0;
	memset(s_rot, 0, sizeof(s_rot));
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = s_snd_frac = s_ym1_frac = s_ym2_frac = 0;
}

static void d1_exit(void);

static int d1_init(const rbx_game *g)
{
	if (!s_pri_buf_hp && !(s_pri_buf_hp = calloc(1, sizeof(*s_pri_buf_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ram_hp && !(s_ram_hp = calloc(1, sizeof(*s_ram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_tile_hp && !(s_tile_hp = calloc(1, sizeof(*s_tile_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_var = !strcmp(g->machine, "slyspy") ? V_SLYSPY : V_MIDRES;
	s_mb = !strcmp(g->machine, "midresb") || !strcmp(g->machine, "midresbj");
	uint32_t snd_size, csize, t1s, t2s, ssize, osize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *snd = rbx_region("audiocpu", &snd_size), *chars = rbx_region("char", &csize),
	        *t1 = rbx_region("tiles1", &t1s), *t2 = rbx_region("tiles2", &t2s), *spr = rbx_region("sprites", &ssize),
	        *oki = rbx_region("oki", &osize);
	if (!s_rom || !snd || snd_size < 0x10000 || !chars || !t1 || !t2 || !spr || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (s_var == V_SLYSPY)   // init_slyspy: h6280_decrypt (bits 0 and 7 swapped)
		for (uint32_t i = 0; i < 0x10000; i++)
			snd[i] = (uint8_t)((snd[i] & 0x7e) | ((snd[i] & 1) << 7) | ((snd[i] & 0x80) >> 7));
	{	// charlayout / tilelayout (RGN_FRAC(1,4)), as dec0's
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 8);
			x16[i] = (uint32_t)(16 * 8 + i);
			x16[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 8);
		const uint32_t qc = csize * 2, q1 = t1s * 2, q2 = t2s * 2, qs = ssize * 2;
		const uint32_t pc[4] = { 0, 2 * qc, qc, 3 * qc };
		const uint32_t p1[4] = { q1, 3 * q1, 0, 2 * q1 }, p2[4] = { q2, 3 * q2, 0, 2 * q2 }, ps[4] = { qs, 3 * qs, 0, 2 * qs };
		const rbx_gfx_layout l8 = { 8, 8, 4, pc, x8, y8, 8 * 8 };
		const rbx_gfx_layout la = { 16, 16, 4, p1, x16, y16, 16 * 16 }, lb = { 16, 16, 4, p2, x16, y16, 16 * 16 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, x16, y16, 16 * 16 };
		if (rbx_gfx_decode(&s_gfx8, &l8, chars, csize / 4) || rbx_gfx_decode(&s_gfx16a, &la, t1, t1s / 4) ||
		    rbx_gfx_decode(&s_gfx16b, &lb, t2, t2s / 4) || rbx_gfx_decode(&s_gfxspr, &ls, spr, ssize / 4)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			d1_exit();
			return -1;
		}
	}
	memset(s_tile, 0, sizeof(s_tile));
	for (int i = 0; i < 3; i++) {
		s_tile[i].gfx8 = &s_gfx8;
		s_tile[i].base8 = s_var == V_MIDRES ? 256 : 0;   // (gfx_midres: the chars from 256)
	}
	s_tile[1].gfx16 = &s_gfx16a;
	s_tile[1].base16 = 512;
	s_tile[2].gfx16 = &s_gfx16b;
	s_tile[2].base16 = 768;
	if (s_mb)   // (disable_rc_scroll)
		s_tile[1].no_rc_scroll = s_tile[2].no_rc_scroll = 1;
	if (s_var == V_SLYSPY) {   // (video_start slyspy: the second chip's transmask groups)
		s_tile[1].tile_cb = baddudes_tile_cb;
		s_tile[1].transmask_on = 1;
		s_tile[1].fgmask[0] = 0xffff;
		s_tile[1].bgmask[0] = 0x0001;
		s_tile[1].fgmask[1] = 0x00ff;
		s_tile[1].bgmask[1] = 0xff01;
	}
	memset(&s_spr, 0, sizeof(s_spr));
	s_spr.gfx = &s_gfxspr;
	s_spr.color_base = s_var == V_MIDRES ? 0 : 256;
	s_spr.colpri_cb = s_var == V_MIDRES ? midres_colpri : NULL;
	s_snap = calloc(1, sizeof(*s_snap));
	// sound: YM2203 1.5 MHz (SSG 0.90, FM 0.35), YM3812 3 MHz 0.35, OKI 1 MHz (Midnight Resistance 1.056) 0.80
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM1_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = 230;
	yc.gain[3] = 90;
	int rc = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM3812;
	yc.clock = YM2_CLOCK;
	yc.gain[0] = 90;
	yc.irq = ym2_irq;
	rc = rc || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	if (!rc) {
		s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
		s_ym2_cap = (int)(rbx_ym_rate(1) / 50) + 256;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		s_ym2buf = (int32_t *)calloc((size_t)s_ym2_cap, sizeof(int32_t));
		rc = !s_mix || !s_ym2buf || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 0x400);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		d1_exit();
		return -1;
	}
	oki_init(&s_oki, s_var == V_MIDRES ? 1056000 : 1000000, 1, oki, osize, 205);
	s_snd_clock = s_var == V_SLYSPY ? 12000000 / 2 / 2 : 24000000 / 4 / 3;
	h6280_init(&s_snd);
	for (uint32_t pg = 0; pg < 8; pg++)
		s_snd.rpage[pg] = snd + pg * 0x2000;
	s_snd.rpage[0x1f0000 >> 13] = s_snd.wpage[0x1f0000 >> 13] = s_sram;
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	if (s_mb) {
		s_snd_clock = 1500000;
		s_snd_rom = snd;
		s_snd_rom_size = snd_size;
		m6502_init(&s_snd65, M6502_R65C02);
		for (int pg = 0; pg < 8; pg++)
			s_snd65.rpage[pg] = s_snd65.wpage[pg] = s_sram65 + pg * 256;
		for (int pg = 0x80; pg < 0x100; pg++)
			if ((uint32_t)pg * 256 + 256 <= snd_size)
				s_snd65.rpage[pg] = snd + pg * 256;
		s_snd65.read = snd65_r;
		s_snd65.write = snd65_w;
	}
	s_port_in = rbx_port_find("INPUTS");
	// (the rotary aims with the right stick; midres2 turns by its button 3 and midresbj by its joystick, reading the
	// rotary for nothing)
	s_face[0] = s_face[1] = -1;
	if (s_var == V_MIDRES && strcmp(g->set, "midres2") && strcmp(g->set, "midresbj")) {
		const int j = !strcmp(g->set, "midresj") ? 2 : 0;
		s_face[0] = 0x21bc + j;
		s_face[1] = 0x2238 + j;
	}
	rbx_set_rstick_dial(s_face[0] >= 0);
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_dsw = rbx_port_find("DSW");
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	if (s_var == V_SLYSPY) {
		m68k_bus_map(0, 0x5ffff, s_rom, s_rom_size, 1, 0);
		m68k_bus_map(0x304000, 0x307fff, s_ram, sizeof(s_ram), 1, 1);
		m68k_bus_map(0x308000, 0x3087ff, s_spriteram, sizeof(s_spriteram), 1, 1);
		m68k_bus_map(0x310000, 0x3107ff, s_pal, sizeof(s_pal), 1, 1);
	} else {
		m68k_bus_map(0, 0x7ffff, s_rom, s_rom_size, 1, 0);
		m68k_bus_map(0x100000, 0x103fff, s_ram, sizeof(s_ram), 1, 1);
		m68k_bus_map(0x120000, 0x1207ff, s_spriteram, sizeof(s_spriteram), 1, 1);
		m68k_bus_map(0x140000, 0x1407ff, s_pal, sizeof(s_pal), 0, 1);   // (write only)
	}
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	d1_reset();
	return 0;
}

static void d1_exit(void)
{
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix);
	free(s_ym2buf);
	s_mix = s_ym2buf = NULL;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16a);
	rbx_gfx_free(&s_gfx16b);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
	free(s_pri_buf_hp);
	s_pri_buf_hp = NULL;
	free(s_ram_hp);
	s_ram_hp = NULL;
	free(s_tile_hp);
	s_tile_hp = NULL;
}

static void d1_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	if (s_var == V_MIDRES)
		for (int p = 0; p < 2; p++)   // (the rotary: the positional control, 12 a turn, on the right stick)
			s_rotary[p] = rbx_rotary_turn(&s_rot[p], p + 1, 12, s_rotary[p], 1, 8, mr_facing(p));
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // irq6_line_hold at vblank
			vblank_capture();
			m68k_bus_hold_irq(6);
		}
		m68k_execute(rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		sound_line();
	}
	sound_frame_end();
	s_frame++;
}

static int d1_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[4] = { s_cpu_frac, s_snd_frac, s_ym1_frac, s_ym2_frac };
	int32_t misc[8] = { (int32_t)s_frame, s_pri, s_prot_state, s_sound_prot_state, s_latch, (int32_t)s_oki_acc,
	                    s_oki_hold, 0 };
	uint8_t snd[H6280_STATE_SIZE];
	uint8_t snd65[M6502_STATE_SIZE];
	if (!load) {
		memcpy(snd, &s_snd, H6280_STATE_SIZE);
		memcpy(snd65, &s_snd65, M6502_STATE_SIZE);
	}
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	for (int i = 0; i < 3; i++) {
		static const char *const names[3] = { "bac0", "bac1", "bac2" };
		rc |= rbx_state_io(ctx, names[i], &s_tile[i], offsetof(bac06_t, gfx8));
	}
	if (s_mb) {
		rc |= rbx_state_io(ctx, "m6502", snd65, M6502_STATE_SIZE);
		rc |= rbx_state_io(ctx, "sram", s_sram65, sizeof(s_sram65));
	} else {
		rc |= rbx_state_io(ctx, "h6280", snd, H6280_STATE_SIZE);
		rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	}
	rc |= rbx_state_io(ctx, "oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	rc |= rbx_state_io(ctx, "rot", s_rotary, sizeof(s_rotary));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		if (s_mb)
			memcpy(&s_snd65, snd65, M6502_STATE_SIZE);
		else
			memcpy(&s_snd, snd, H6280_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_ym1_frac = fr[2];
		s_ym2_frac = fr[3];
		s_frame = (uint32_t)misc[0];
		s_pri = (uint16_t)misc[1];
		s_prot_state = misc[2];
		s_sound_prot_state = misc[3];
		s_latch = (uint8_t)misc[4];
		s_oki_acc = (uint32_t)misc[5];
		s_oki_hold = misc[6];
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int d1_save(void *ctx) { return d1_state(ctx, 0); }
static int d1_load(void *ctx) { return d1_state(ctx, 1); }
static int d1_supported(const char *machine)
{
	return !strcmp(machine, "slyspy") || !strcmp(machine, "midres") || !strcmp(machine, "midresb") ||
	       !strcmp(machine, "midresbj");
}

const rbx_driver rbx_drv_dec1 = { "DECO dec0", d1_init, d1_exit, d1_reset, d1_frame, d1_save, d1_load,
                                  d1_supported, d1_render, NULL };
