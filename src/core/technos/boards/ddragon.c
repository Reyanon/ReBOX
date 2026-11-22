/*  Technos Double Dragon, Double Dragon II, China Gate and the boards built on them (Toffy, Dark Tower). Ported from
    MAME src/mame/technos/ddragon.cpp, ddragon_v.cpp, ddragon.h and chinagat.cpp (BSD-3-Clause, copyright-holders:
    Philip Bennett, Carlos A. Lozano, Rob Rosenbrock, Phil Stroffolino, Ernesto Corvi, David Haywood, R. Belmont;
    chinagat.cpp: Paul Hampson, Quench).
    Main CPU: HD6309E at 3 MHz (China Gate HD6309 / bootleg MC6809E at 1.5 MHz). NMI at vcount F8, FIRQ every 16 lines
    (the rising edge of vcount bit 3), IRQ from the sub CPU. Sub CPU (the sprite CPU): HD63701Y0 (Double Dragon; its
    ROM at C000, RAM 0040-013F, port 6 to the main CPU), HD6309E or MC6803 (bootlegs), Z80 (Double Dragon II), HD6309
    (China Gate, sharing the main CPU's RAM); its NMI (China Gate IRQ) from the main CPU. The shared RAM is open to the
    main CPU only while the sub CPU is held in reset or halted. Sound: MC6809 with a YM2151 and two MSM5205 (Double
    Dragon) or Z80 with a YM2151 and an OKI6295 (Double Dragon II, China Gate), Z80 with two YM2203 (Sai Yu Gou Ma Roku
    bootleg 2). Video: a 512x512 tilemap of 16x16 tiles, a 256x256 text layer of 8x8, 16x16 sprites in 1x1 to 2x2
    blocks; xBGR_444 palette. Scroll and flip latched per line (MAME's update_partial on every scanline).
    MAME runs the CPUs with a 60 kHz quantum (China Gate 6 kHz): here 4 slices a line (62.7 kHz).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/hd6309/hd6309.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/m6800/m6801.h"
#include "../../cpu/m6805/m68705.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/msm5205.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 57445          // set_raw(12 MHz / 2, 384, 0, 256, 272, 0, 240)
#define LINES 272
#define SLICES 4
#define VIS_W 256
#define VIS_H 240
#define YM_CLOCK 3579545

// machines
enum { M_DD, M_DDBL, M_DDBLA, M_DD2, M_DARK, M_TOFFY, M_CG, M_CGB2 };
// sub CPU kinds
enum { SUB_NONE, SUB_63701, SUB_6309, SUB_6803, SUB_Z80, SUB_6809 };

typedef struct {
	uint16_t sx, sy;
	uint8_t flip;
} line_regs;

static int s_mach, s_sub_kind, s_hw;   // s_hw: MAME m_technos_video_hw (0 DD, 1 China Gate, 2 DD2)
static int s_main6809;                 // the main CPU is an MC6809E (saiyugoub2)
static uint8_t *s_rom, *s_srom, *s_zrom, *s_mcurom, *s_adpcm[2];
static uint32_t s_rom_size, s_srom_size, s_zrom_size, s_adpcm_size[2];
static rbx_gfx s_gfxchar, s_gfxspr, s_gfxtile;
static int s_port[5];
// the video RAM by pieces of lines: a piece starts at a line whose drawing (update_partial at the next line) sees
// the RAM changed since the previous line (sprite multiplexing: Thunder Strike rewrites its sprites at the FIRQs)
#define MAXSEG 32
typedef struct {
	int y0;
	uint8_t fgram[0x800], bgram[0x800], spr[0x800];
} vseg;
typedef struct {
	int n;
	vseg seg[MAXSEG];
} vbuf;
static vbuf *s_vb[2];
static int s_vb_cur, s_vdirty;
static struct {
	uint8_t pal[0x400];     // 0x000-0x1ff low bytes, 0x200-0x3ff the ext (high) bytes (the palette at the frame's end)
	line_regs lines[VIS_H];
	const vbuf *vb;
} *s_snap;

// ---- state (one block for save states) ----
static struct {
	uint8_t ram[0x2000];       // main RAM (China Gate: shared with the sub CPU)
	uint8_t pal[0x400];
	uint8_t fgram[0x800], bgram[0x800], spr[0x800];
	uint8_t comram[0x400];
	uint8_t subram[0x1000];    // the sub CPU's own RAM (63701 0040-013F at [0x40..], 6309 0020-0FFF, 6803 0100-0FFF)
	uint8_t zram[0x800];       // the sound CPU's RAM (6809 0000-0FFF uses subram2)
	uint8_t sndram[0x1000];
	uint8_t scrollx_lo, scrolly_lo, scrollx_hi, scrolly_hi, flip;
	uint8_t bank, subbank;
	uint8_t sub_reset, sub_halt;   // lines held
	uint8_t sub_port;              // m_ddragon_sub_port
	uint8_t sub_nmi;               // the sprite IRQ line level
	uint8_t p6_ddr, p6_data;
	uint8_t main_nmi, main_firq, main_irq;
	uint8_t latch, latch_pending;
	int32_t ym_irq;
	uint32_t adpcm_pos[2], adpcm_end[2];
	uint8_t adpcm_idle[2];
	int32_t adpcm_data[2];
	uint8_t mcu_porta;             // darktowr
	int64_t main_t, sub_t, snd_t, mcu_t;
	uint64_t main_frac, sub_frac, snd_frac, ym_frac, mcu_frac;
	uint32_t msm_acc, oki_acc;
	int32_t oki_hold;
	line_regs lines[LINES];
	uint8_t state_end;
} *t;   // (calloc in init: ADR-040)

static hd6309_t s_main;
static m6809_t s_main09, s_sub09, s_snd09;
static hd6309_t s_sub6309;
static m6801 s_mcu;
static z80 s_subz, s_sndz;
static m68705_t s_m705;
static msm5205_t s_msm[2];
static okim6295_t s_oki;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static uint32_t s_main_clock, s_sub_clock, s_snd_clock;
static int s_snd_z80, s_two_ym;
static const uint8_t *s_zmap[256], *s_szmap[256];
static int s_line;
static int s_tstrike;
static const uint8_t *s_rom_prg;      // the main CPU's program

// ---- the main CPU's lines ----
static void main_set(int which, int state)   // 0 NMI, 1 FIRQ, 2 IRQ
{
	if (which == 0) {
		t->main_nmi = (uint8_t)state;
		if (s_main6809) m6809_set_nmi(&s_main09, state); else hd6309_set_nmi(&s_main, state);
	} else if (which == 1) {
		t->main_firq = (uint8_t)state;
		if (s_main6809) m6809_set_firq(&s_main09, state); else hd6309_set_firq(&s_main, state);
	} else {
		t->main_irq = (uint8_t)state;
		if (s_main6809) m6809_set_irq(&s_main09, state); else hd6309_set_irq(&s_main, state);
	}
}

static int sub_suspended(void) { return t->sub_reset || t->sub_halt; }

static void sub_cpu_reset(void);

static void sub_set_irq(int state)   // m_sprite_irq (NMI; China Gate IRQ)
{
	switch (s_sub_kind) {
	case SUB_63701:
	case SUB_6803:
		m6801_set_irq_line(&s_mcu, M6801_NMI_LINE, state);
		break;
	case SUB_6309:
		if (s_mach == M_CG) hd6309_set_irq(&s_sub6309, state); else hd6309_set_nmi(&s_sub6309, state);
		break;
	case SUB_6809:
		m6809_set_irq(&s_sub09, state);
		break;
	case SUB_Z80:
		if (state && !t->sub_nmi)
			z80_gen_nmi(&s_subz);
		break;
	}
	t->sub_nmi = (uint8_t)state;
}

static void sub_set_reset(int state)
{
	if (s_sub_kind == SUB_NONE)
		return;
	if (s_sub_kind == SUB_63701 || s_sub_kind == SUB_6803) {
		m6801_set_reset(&s_mcu, state);
		if (!state && t->sub_reset)   // (the NMI line keeps its level across the reset)
			s_mcu.nmi_state = t->sub_nmi;
	} else if (state && !t->sub_reset)
		sub_cpu_reset();
	t->sub_reset = (uint8_t)state;
}

// ---- sound ----
static void latch_write(uint8_t v)
{
	t->latch = v;
	if (s_snd_z80) {
		if (!t->latch_pending)
			z80_gen_nmi(&s_sndz);
	} else
		m6809_set_irq(&s_snd09, 1);
	t->latch_pending = 1;
}
static uint8_t latch_read(void)
{
	t->latch_pending = 0;
	if (!s_snd_z80)
		m6809_set_irq(&s_snd09, 0);
	return t->latch;
}

static void ym_irq(int state)
{
	t->ym_irq = state;
	if (!s_snd_z80)
		m6809_set_firq(&s_snd09, state);
}

static void adpcm_int(int i)   // ddragon_adpcm_int
{
	if (t->adpcm_pos[i] >= t->adpcm_end[i] || t->adpcm_pos[i] >= s_adpcm_size[i]) {
		t->adpcm_idle[i] = 1;
		msm5205_reset_w(&s_msm[i], 1);
	} else if (t->adpcm_data[i] != -1) {
		msm5205_data_w(&s_msm[i], (uint8_t)(t->adpcm_data[i] & 0x0f));
		t->adpcm_data[i] = -1;
	} else {
		t->adpcm_data[i] = s_adpcm[i][t->adpcm_pos[i]++];
		msm5205_data_w(&s_msm[i], (uint8_t)(t->adpcm_data[i] >> 4));
	}
}
static void adpcm_int0(void) { adpcm_int(0); }
static void adpcm_int1(void) { adpcm_int(1); }

static void adpcm_w(int offset, uint8_t data)
{
	const int chip = offset & 1;
	if (!s_adpcm[chip])
		return;
	switch (offset >> 1) {
	case 3: t->adpcm_idle[chip] = 1; msm5205_reset_w(&s_msm[chip], 1); break;
	case 2: t->adpcm_pos[chip] = (uint32_t)(data & 0x7f) << 9; break;
	case 1: t->adpcm_end[chip] = (uint32_t)(data & 0x7f) << 9; break;
	case 0: t->adpcm_idle[chip] = 0; msm5205_reset_w(&s_msm[chip], 0); break;
	}
}

static uint8_t snd09_read(void *u, uint16_t a)   // ddragon_sound_map
{
	(void)u;
	if (a < 0x1000) return t->sndram[a];
	if (a == 0x1000) return latch_read();
	if (a == 0x1800) return (uint8_t)((t->adpcm_idle[0] ? 1 : 0) | (t->adpcm_idle[1] ? 2 : 0));
	if (a == 0x2800 || a == 0x2801) return rbx_ym_read(0, a & 1);
	if (a >= 0x8000) return s_zrom[a];
	return 0;
}
static void snd09_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x1000) t->sndram[a] = v;
	else if (a == 0x2800 || a == 0x2801) rbx_ym_write(0, a & 1, v);
	else if (a >= 0x3800 && a < 0x3808) adpcm_w(a & 7, v);
}

static uint8_t sndz_read(void *u, uint16_t a)   // ddragon2_sound_map, chinagat sound_map / ym2203c_sound_map
{
	(void)u;
	if (a < 0x8000) return s_zrom[a];
	if (a < 0x8800) return t->zram[a - 0x8000];
	if (s_two_ym) {
		if (a == 0x8800 || a == 0x8801) return rbx_ym_read(0, a & 1);
		if (a == 0x8804 || a == 0x8805) return rbx_ym_read(1, a & 1);
	} else {
		if (a == 0x8800 || a == 0x8801) return rbx_ym_read(0, a & 1);
		if (a == 0x9800) return oki_read(&s_oki);
	}
	if (a == 0xa000) return latch_read();
	return 0;
}
static void sndz_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8800) t->zram[a - 0x8000] = v;
	else if (s_two_ym) {
		if (a == 0x8800 || a == 0x8801) rbx_ym_write(0, a & 1, v);
		else if (a == 0x8804 || a == 0x8805) rbx_ym_write(1, a & 1, v);
	} else if (a == 0x8800 || a == 0x8801)
		rbx_ym_write(0, a & 1, v);
	else if (a == 0x9800)
		oki_write(&s_oki, v);
}
static uint8_t z_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

static void sndz_start(void)
{
	z80_init(&s_sndz);
	s_sndz.read_byte = sndz_read;
	s_sndz.write_byte = sndz_write;
	s_sndz.port_in = z_in;
	s_sndz.port_out = z_out;
	s_sndz.read_map = s_zmap;
}

static void sound_slice(void)
{
	const int cycles = rbx_line_cycles(&t->snd_frac, s_snd_clock, FRAME_MHZ, LINES * SLICES);
	if (s_snd_z80)
		z80_run(&s_sndz, s_sndz.cyc + (unsigned long)cycles, &t->ym_irq, 0);
	else
		m6809_run_until(&s_snd09, t->snd_t += cycles);
}

static void sound_line(void)
{
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&t->ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
	const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	if (s_two_ym) {
		static int32_t tmp[512];
		const int n2 = rbx_ym_advance(1, clocks, tmp, n < 512 ? n : 512);
		(void)n2;
	}
	const uint32_t yr = rbx_ym_rate(0);
	if (s_adpcm[0]) {
		for (int i = 0; i < n; i++) {
			t->msm_acc += s_msm[0].clock;
			const uint32_t k = t->msm_acc / yr;
			t->msm_acc -= k * yr;
			msm5205_advance(&s_msm[0], k);
			msm5205_advance(&s_msm[1], k);
			s_mix[s_mix_n + i] += msm5205_output(&s_msm[0]) + msm5205_output(&s_msm[1]);
		}
	} else if (s_snd_z80 && !s_two_ym) {
		for (int i = 0; i < n; i++) {
			t->oki_acc += s_oki.rate;
			while (t->oki_acc >= yr) {
				t->oki_acc -= yr;
				t->oki_hold = oki_sample(&s_oki);
			}
			s_mix[s_mix_n + i] += t->oki_hold;
		}
	}
	s_mix_n += n;
}

static void sound_frame_end(void)
{
	rbx_ym_async_take(0, s_mix, s_mix_n);
	if (s_two_ym)
		rbx_ym_async_take(1, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

// ---- main CPU bus ----
static void map_bank(void);

static void sub_port6_w(uint8_t data)
{
	if ((data & 1) == 0)
		sub_set_irq(0);
	if (!(t->sub_port & 2) && (data & 2))
		main_set(2, 1);
	t->sub_port = data;
}

static void bankswitch_w(uint8_t data)
{
	t->scrollx_hi = data & 1;
	t->scrolly_hi = (data & 2) >> 1;
	if (s_mach == M_TOFFY) {
		t->bank = (data & 0x20) >> 5;
	} else {
		if (s_mach != M_DARK)
			t->flip = (~data & 4) ? 1 : 0;
		sub_set_reset(data & 8 ? 0 : 1);
		t->sub_halt = (data & 0x10) ? 1 : 0;
		t->bank = (data & 0xe0) >> 5;
	}
	map_bank();
}

static void interrupt_ack(int offset, uint8_t data)
{
	switch (offset) {
	case 0: main_set(0, 0); break;
	case 1: main_set(1, 0); break;
	case 2: main_set(2, 0); break;
	case 3: latch_write(data); break;
	case 4: if (s_sub_kind != SUB_NONE) sub_set_irq(1); break;
	}
}

static int vblank(void) { return s_line >= VIS_H; }

static uint8_t port_in(int i)
{
	if (s_hw == 1) {   // China Gate SYSTEM bit 0: vblank
		if (i == 0)
			rbx_port_custom(s_port[0], 0x01, vblank() ? 1 : 0);
	} else if (i == 2) {   // EXTRA: bit 3 vblank, bit 4 the sub CPU's bus free (active low)
		const int bf = s_sub_kind != SUB_NONE && sub_suspended();
		rbx_port_custom(s_port[2], 0x18, (vblank() ? 0x08u : 0) | (bf ? 0x10u : 0));
	}
	return (uint8_t)rbx_port_read(s_port[i]);
}

static uint8_t comram_r(int o)
{
	if (s_sub_kind == SUB_NONE || !sub_suspended())
		return 0xff;
	return t->comram[o];
}
static void comram_w(int o, uint8_t v)
{
	if (s_sub_kind == SUB_NONE || !sub_suspended())
		return;
	t->comram[o] = v;
}

static uint8_t darktowr_mcu_bank_r(int offset)
{
	if (s_tstrike) {   // (MAME's hack for the set mismatched against its MCU)
		const uint16_t pc = s_main.pc;
		if (pc == 0x9ace) return 0;
		if (pc == 0x9ae4) return 0x63;
		return t->ram[0xbe1];
	}
	if (offset == 0x1401 || offset == 1)
		return t->mcu_porta;
	return 0xff;
}
static void darktowr_mcu_bank_w(int offset, uint8_t data)
{
	if (offset == 0x1400 || offset == 0) {
		uint8_t v = 0;
		for (int b = 0; b < 8; b++)
			if (data & (1 << b)) v |= (uint8_t)(0x80 >> b);
		m68705_port_input(&s_m705, 1, v);
	}
}

static uint8_t dd_main_read(void *u, uint16_t a)   // base_map / ddragon2_main_map
{
	(void)u;
	if (a >= 0x8000) return s_rom_prg[a];
	if (a >= 0x4000) {
		if (s_mach == M_DARK && t->bank == 4)
			return darktowr_mcu_bank_r(a - 0x4000);
		return s_rom_prg[(0x10000u + (uint32_t)t->bank * 0x4000 + (a - 0x4000)) % s_rom_size];
	}
	if (s_mach == M_DD2) {
		if (a < 0x1800) return t->ram[a];
		if (a >= 0x3c00) return t->pal[a - 0x3c00];
	} else {
		if (a < 0x1000) return t->ram[a];
		if (a < 0x1400) return t->pal[a - 0x1000];
	}
	if (a >= 0x1800 && a < 0x2000) return t->fgram[a - 0x1800];
	if (a >= 0x2000 && a < 0x2800) return comram_r(a & 0x1ff);
	if (a >= 0x2800 && a < 0x3000) return t->spr[a - 0x2800];
	if (a >= 0x3000 && a < 0x3800) return t->bgram[a - 0x3000];
	if (a >= 0x3800 && a <= 0x3804) return port_in(a - 0x3800);
	if (a >= 0x380b && a <= 0x380f) {
		interrupt_ack(a - 0x380b, 0xff);
		return 0xff;
	}
	return 0;
}
static void dd_main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000) {
		if (a < 0x8000 && s_mach == M_DARK && t->bank == 4)
			darktowr_mcu_bank_w(a - 0x4000, v);
		return;
	}
	if (s_mach == M_DD2) {
		if (a < 0x1800) { t->ram[a] = v; return; }
		if (a >= 0x3c00) { t->pal[a - 0x3c00] = v; return; }
	} else {
		if (a < 0x1000) { t->ram[a] = v; return; }
		if (a < 0x1400) { t->pal[a - 0x1000] = v; return; }
	}
	if (a >= 0x1800 && a < 0x2000) { t->fgram[a - 0x1800] = v; s_vdirty = 1; }
	else if (a >= 0x2000 && a < 0x2800) comram_w(a & 0x1ff, v);
	else if (a >= 0x2800 && a < 0x3000) { t->spr[a - 0x2800] = v; s_vdirty = 1; }
	else if (a >= 0x3000 && a < 0x3800) { t->bgram[a - 0x3000] = v; s_vdirty = 1; }
	else if (a == 0x3808) bankswitch_w(v);
	else if (a == 0x3809) t->scrollx_lo = v;
	else if (a == 0x380a) t->scrolly_lo = v;
	else if (a >= 0x380b && a <= 0x380f) interrupt_ack(a - 0x380b, v);
}

// China Gate main_map
static uint8_t cg_main_read(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0x8000) return s_rom[a];
	if (a >= 0x4000) return s_rom[(0x10000u + (uint32_t)t->bank * 0x4000 + (a - 0x4000)) % s_rom_size];
	if (a < 0x2000) return t->ram[a];
	if (a < 0x2800) return t->fgram[a - 0x2000];
	if (a < 0x3000) return t->bgram[a - 0x2800];
	if (a >= 0x3800 && a < 0x3980) return t->spr[a - 0x3800];
	if (a >= 0x3f00 && a <= 0x3f04) {
		static const int map[5] = { 0, 1, 2, 3, 4 };
		return port_in(map[a - 0x3f00]);
	}
	return 0;
}
static void cg_main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000) return;
	if (a < 0x2000) t->ram[a] = v;
	else if (a < 0x2800) { t->fgram[a - 0x2000] = v; s_vdirty = 1; }
	else if (a < 0x3000) { t->bgram[a - 0x2800] = v; s_vdirty = 1; }
	else if (a >= 0x3000 && a < 0x3180) t->pal[a - 0x3000] = v;
	else if (a >= 0x3400 && a < 0x3580) t->pal[0x200 + a - 0x3400] = v;
	else if (a >= 0x3800 && a < 0x3980) { t->spr[a - 0x3800] = v; s_vdirty = 1; }
	else if (a >= 0x3e00 && a <= 0x3e04) {
		switch (a - 0x3e00) {
		case 0: latch_write(v); break;
		case 1: main_set(0, 0); break;
		case 2: main_set(1, 0); break;
		case 3: main_set(2, 0); break;
		case 4: sub_set_irq(1); break;
		}
	} else if (a == 0x3e06) t->scrolly_lo = v;
	else if (a == 0x3e07) t->scrollx_lo = v;
	else if (a == 0x3f00) {   // video_ctrl_w
		t->scrolly_hi = (v & 2) >> 1;
		t->scrollx_hi = v & 1;
		t->flip = (~v & 4) ? 1 : 0;
	} else if (a == 0x3f01) {
		t->bank = v & 7;
		map_bank();
	}
}

static void map_bank(void)
{
	const uint8_t *bank;
	if (s_mach == M_DARK && t->bank == 4)
		bank = NULL;
	else
		bank = s_rom_prg + (0x10000u + (uint32_t)t->bank * 0x4000) % s_rom_size;
	for (int p = 0; p < 0x40; p++) {
		const uint8_t *q = bank && (uint32_t)(bank - s_rom_prg) + (uint32_t)p * 256 + 256 <= s_rom_size ? bank + p * 256 : NULL;
		if (s_main6809) s_main09.rpage[0x40 + p] = q; else s_main.rpage[0x40 + p] = q;
	}
}

// ---- sub CPUs ----
static uint8_t mcu_read(void *u, uint16_t a)   // HD63701Y0 (ddragon_sub_map + the Y0's on-chip parts) / MC6803
{
	(void)u;
	if (s_sub_kind == SUB_63701) {
		if (a >= 0x8000 && a < 0x8200) return t->comram[a - 0x8000];
		if ((a >= 0x40 && a < 0x80) || (a >= 0x100 && a < 0x140)) return t->subram[a];
		if (a == 0x17) return (uint8_t)((t->p6_data & t->p6_ddr) | (0xff & ~t->p6_ddr));
		if (a >= 0xc000) return s_mcu.rom[a - 0xc000];
		return 0xff;
	}
	if (a >= 0x8000 && a < 0x8200) return t->comram[a - 0x8000];   // ddragonbla_sub_map
	if (a >= 0x100 && a < 0x1000) return t->subram[a];
	if (a >= 0xc000) return s_srom[a];
	return 0xff;
}
static void mcu_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8200) { t->comram[a - 0x8000] = v; return; }
	if (s_sub_kind == SUB_63701) {
		if ((a >= 0x40 && a < 0x80) || (a >= 0x100 && a < 0x140)) t->subram[a] = v;
		else if (a == 0x16 || a == 0x17) {   // (hd6301x p6_ddr_w / p6_data_w)
			if (a == 0x16) {
				if (t->p6_ddr == v) return;
				t->p6_ddr = v;
			} else
				t->p6_data = v;
			sub_port6_w((uint8_t)((t->p6_data & t->p6_ddr) | (t->p6_ddr ^ 0xff)));
		}
		return;
	}
	if (a >= 0x100 && a < 0x1000) t->subram[a] = v;
}
static void mcu_port_w(void *u, int port, uint8_t data, uint8_t ddr)   // MC6803 port 2: ddragonbla_port_w
{
	(void)u;
	(void)ddr;
	if (port != 2)
		return;
	if ((data & 8) == 0)
		sub_set_irq(0);
	if (!(t->sub_port & 0x10) && (data & 0x10))
		main_set(2, 1);
	t->sub_port = data;
}

static uint8_t sub6309_read(void *u, uint16_t a)   // sub_6309_map (ddragonbl)
{
	(void)u;
	if (a >= 0xc000) return s_srom[a];
	if (a >= 0x8000 && a < 0x8200) return t->comram[a - 0x8000];
	if (a >= 0x20 && a < 0x1000) return t->subram[a];
	return 0;
}
static void sub6309_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8200) t->comram[a - 0x8000] = v;
	else if (a >= 0x20 && a < 0x1000) t->subram[a] = v;
	else if (a == 0x17) sub_port6_w(v);
}

static uint8_t cgsub_read(void *u, uint16_t a)   // chinagat sub_map
{
	(void)u;
	if (a >= 0x8000) return s_srom[a];
	if (a >= 0x4000) return s_srom[(0x10000u + (uint32_t)t->subbank * 0x4000 + (a - 0x4000)) % s_srom_size];
	if (a < 0x2000) return t->ram[a];
	return 0;
}
static void cgsub_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x2000) t->ram[a] = v;
	else if (a == 0x2000) {
		t->subbank = v & 7;
		const uint8_t *b = s_srom + (0x10000u + (uint32_t)t->subbank * 0x4000) % s_srom_size;
		for (int p = 0; p < 0x40; p++) {
			if (s_sub_kind == SUB_6309) s_sub6309.rpage[0x40 + p] = b + p * 256;
			else s_sub09.rpage[0x40 + p] = b + p * 256;
		}
	} else if (a == 0x2800)
		sub_set_irq(0);
}

static uint8_t subz_read(void *u, uint16_t a)   // ddragon2_sub_map
{
	(void)u;
	if (a < 0xc000) return s_srom[a];
	if (a < 0xc400) return t->comram[a - 0xc000];
	return 0;
}
static void subz_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	(void)v;
	if (a >= 0xc000 && a < 0xc400) t->comram[a - 0xc000] = v;
	else if (a == 0xd000) sub_set_irq(0);
	else if (a == 0xe000) main_set(2, 1);
}

static void subz_start(void)
{
	z80_init(&s_subz);
	s_subz.read_byte = subz_read;
	s_subz.write_byte = subz_write;
	s_subz.port_in = z_in;
	s_subz.port_out = z_out;
	s_subz.read_map = s_szmap;
}

static void sub_cpu_reset(void)
{
	switch (s_sub_kind) {
	case SUB_6309:
		hd6309_reset(&s_sub6309);
		break;
	case SUB_6809:
		m6809_reset(&s_sub09);
		break;
	case SUB_Z80:
		subz_start();
		break;
	}
	t->sub_t = 0;
}

static void m705_port_w(void *u, int n, uint8_t data, uint8_t mask)   // darktowr mcu_port_a_w
{
	(void)u;
	(void)mask;
	if (n == 0)
		t->mcu_porta = data;
}

static void sub_slice(void)
{
	if (s_sub_kind == SUB_NONE)
		return;
	const int cycles = rbx_line_cycles(&t->sub_frac, s_sub_clock, FRAME_MHZ, LINES * SLICES);
	if (sub_suspended()) {
		if (s_sub_kind == SUB_Z80)
			s_subz.cyc += (unsigned long)cycles;
		return;
	}
	switch (s_sub_kind) {
	case SUB_63701:
	case SUB_6803:
		m6801_execute(&s_mcu, cycles);
		break;
	case SUB_6309:
		t->sub_t += cycles;
		if (s_sub6309.cycles < t->sub_t) hd6309_run_until(&s_sub6309, t->sub_t);
		break;
	case SUB_6809:
		t->sub_t += cycles;
		if (s_sub09.cycles < t->sub_t) m6809_run_until(&s_sub09, t->sub_t);
		break;
	case SUB_Z80: {
		int none = 0;
		z80_run(&s_subz, s_subz.cyc + (unsigned long)cycles, &none, 0);
		break;
	}
	}
}

// ---- video ----
static inline int scanline_to_vcount(int scanline)
{
	const int vcount = scanline + 8;
	return vcount < 0x100 ? vcount : ((vcount - 0x18) | 0x100);
}

static void draw_bg_line(const vseg *sg, int y, const line_regs *r, uint16_t *dst)
{
	const uint8_t *vram = sg->bgram;
	const uint32_t ty = (uint32_t)(r->flip ? 247 - y + r->sy : y + 8 + r->sy) & 511;
	const uint32_t row = ty >> 4, fy = ty & 15;
	for (int x = 0; x < VIS_W; x++) {
		const uint32_t tx = (uint32_t)(r->flip ? 255 - x + r->sx : x + r->sx) & 511;
		const uint32_t col = tx >> 4;
		const uint32_t off = ((col & 0x0f) | ((row & 0x0f) << 4) | ((col & 0x10) << 4) | ((row & 0x10) << 5)) * 2;
		const uint8_t attr = vram[off];
		const uint32_t code = (uint32_t)(vram[off | 1] | (attr & 7) << 8) % s_gfxtile.count;
		const uint32_t px = attr & 0x40 ? 15 - (tx & 15) : tx & 15, py = attr & 0x80 ? 15 - fy : fy;
		dst[x] = (uint16_t)(256 + ((attr >> 3) & 7) * 16 + s_gfxtile.pix[code * 256 + py * 16 + px]);
	}
}

static void draw_fg_line(const vseg *sg, int y, const line_regs *r, uint16_t *dst)
{
	const uint8_t *vram = sg->fgram;
	const uint32_t ty = (uint32_t)(r->flip ? 247 - y : y + 8) & 255;
	for (int x = 0; x < VIS_W; x++) {
		const uint32_t tx = (uint32_t)(r->flip ? 255 - x : x) & 255;
		const uint32_t off = ((ty >> 3) * 32 + (tx >> 3)) * 2;
		const uint8_t attr = vram[off];
		uint32_t code, color;
		if (s_hw == 1) {   // get_fg_16color_tile_info
			code = vram[off | 1] | (uint32_t)(attr & 0x0f) << 8;
			color = attr >> 4;
		} else {
			code = vram[off | 1] | (uint32_t)(attr & 0x07) << 8;
			color = attr >> 5;
		}
		code %= s_gfxchar.count;
		const uint8_t p = s_gfxchar.pix[code * 64 + (ty & 7) * 8 + (tx & 7)];
		if (p)
			dst[x] = (uint16_t)(color * 16 + p);
	}
}

static int s_clip0, s_clip1;   // the lines being drawn

static void draw_sprite16(uint32_t code, int color, int flipx, int flipy, int sx, int sy)
{
	code %= s_gfxspr.count;
	if (!s_gfxspr.used[code] || sx >= VIS_W || sx + 16 <= 0 || sy >= s_clip1 || sy + 16 <= s_clip0)
		return;
	const uint8_t *src = s_gfxspr.pix + code * 256;
	const uint16_t c0 = (uint16_t)(128 + (color & 7) * 16);   // (gfx_element: color % 8 colours)
	for (int j = 0; j < 16; j++) {
		const int yy = sy + j;
		if (yy < s_clip0 || yy >= s_clip1)
			continue;
		const uint8_t *s = src + (flipy ? 15 - j : j) * 16;
		uint16_t *d = rbx_video.bitmap + yy * rbx_video.pitch;
		for (int i = 0; i < 16; i++) {
			const int xx = sx + i;
			if ((unsigned)xx >= VIS_W)
				continue;
			const uint8_t p = s[flipx ? 15 - i : i];
			if (p)
				d[xx] = (uint16_t)(c0 + p);
		}
	}
}

static void draw_sprites(const vseg *sg, int flip)
{
	const uint8_t *src = sg->spr;
	const uint32_t bytes = s_hw == 1 ? 0x180 : 0x800;
	for (uint32_t i = 0; i + 5 <= bytes; i += 5) {
		const int attr = src[i + 1];
		if (!(attr & 0x80))
			continue;
		int sx = 240 - src[i + 4] + ((attr & 2) << 7);
		int sy = 232 - src[i + 0] + ((attr & 1) << 8);
		const int size = (attr & 0x30) >> 4;
		int flipx = attr & 8, flipy = attr & 4;
		int dx = -16, dy = -16;
		int which, color;
		if (s_hw == 2) {
			color = src[i + 2] >> 5;
			which = src[i + 3] | ((src[i + 2] & 0x1f) << 8);
		} else {
			if (s_hw == 1) {
				if (sx < -7 && sx > -16) sx += 256;
				if (sy < -7 && sy > -16) sy += 256;
			}
			color = src[i + 2] >> 4;
			which = src[i + 3] | ((src[i + 2] & 0x0f) << 8);
		}
		if (flip) {
			sx = 240 - sx;
			sy = 240 - 16 - sy;
			flipx = !flipx;
			flipy = !flipy;
			dx = -dx;
			dy = -dy;
		}
		which &= ~size;
		switch (size) {
		case 0:
			draw_sprite16((uint32_t)which, color, flipx, flipy, sx, sy);
			break;
		case 1:
			draw_sprite16((uint32_t)which, color, flipx, flipy, sx, sy + dy);
			draw_sprite16((uint32_t)which + 1, color, flipx, flipy, sx, sy);
			break;
		case 2:
			draw_sprite16((uint32_t)which, color, flipx, flipy, sx + dx, sy);
			draw_sprite16((uint32_t)which + 2, color, flipx, flipy, sx, sy);
			break;
		case 3:
			draw_sprite16((uint32_t)which, color, flipx, flipy, sx + dx, sy + dy);
			draw_sprite16((uint32_t)which + 1, color, flipx, flipy, sx + dx, sy);
			draw_sprite16((uint32_t)which + 2, color, flipx, flipy, sx, sy + dy);
			draw_sprite16((uint32_t)which + 3, color, flipx, flipy, sx, sy);
			break;
		}
	}
}

static void dd_render(void)
{
	for (int i = 0; i < 512; i++) {   // xBGR_444, the ext byte high
		const uint32_t p = (uint32_t)(s_snap->pal[0x200 + i] << 8 | s_snap->pal[i]);
		rbx_video.palette[i] = ((p & 15) * 17) << 16 | (((p >> 4) & 15) * 17) << 8 | ((p >> 8) & 15) * 17;
	}
	// (bg, sprites, fg by pieces; the sprites with the flip of the piece's first line)
	const vbuf *vb = s_snap->vb;
	for (int k = 0; k < vb->n; k++) {
		const vseg *sg = &vb->seg[k];
		const int y0 = sg->y0, y1 = k + 1 < vb->n ? vb->seg[k + 1].y0 : VIS_H;
		for (int y = y0; y < y1; y++)
			draw_bg_line(sg, y, &s_snap->lines[y], rbx_video.bitmap + y * rbx_video.pitch);
		s_clip0 = y0;
		s_clip1 = y1;
		draw_sprites(sg, s_snap->lines[y0].flip);
		for (int y = y0; y < y1; y++)
			draw_fg_line(sg, y, &s_snap->lines[y], rbx_video.bitmap + y * rbx_video.pitch);
	}
}

static void vram_line(int line)   // (after a visible line: a new piece when the RAM changed)
{
	vbuf *b = s_vb[s_vb_cur];
	if (line == 0)
		b->n = 0;
	else if (!s_vdirty)
		return;
	vseg *sg;
	if (b->n < MAXSEG) {
		sg = &b->seg[b->n++];
		sg->y0 = line;
	} else
		sg = &b->seg[MAXSEG - 1];
	memcpy(sg->fgram, t->fgram, sizeof(sg->fgram));
	memcpy(sg->bgram, t->bgram, sizeof(sg->bgram));
	memcpy(sg->spr, t->spr, sizeof(sg->spr));
	s_vdirty = 0;
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->pal, t->pal, sizeof(t->pal));
	memcpy(s_snap->lines, t->lines, sizeof(s_snap->lines));
	s_snap->vb = s_vb[s_vb_cur];
	s_vb_cur ^= 1;
	rbx_video_frame_done();
}

// ---- init / reset / frame ----
static int mach_is(const char *machine, const char *name)   // (the machine part of "machine:init")
{
	const size_t n = strlen(name);
	return !strncmp(machine, name, n) && (machine[n] == 0 || machine[n] == ':');
}

static void dd_reset(void)
{
	memset(t, 0, offsetof(__typeof__(*t), state_end));
	t->adpcm_idle[0] = t->adpcm_idle[1] = 1;
	t->adpcm_data[0] = t->adpcm_data[1] = -1;
	t->mcu_porta = 0xff;
	const int cg = s_hw == 1;
	uint8_t (*rd)(void *, uint16_t) = cg ? cg_main_read : dd_main_read;
	void (*wr)(void *, uint16_t, uint8_t) = cg ? cg_main_write : dd_main_write;
	if (s_main6809) {
		m6809_init(&s_main09);
		s_main09.read = rd;
		s_main09.write = wr;
		for (int p = 0x80; p < 0x100; p++)
			s_main09.rpage[p] = s_rom_prg + p * 256;
		for (int p = 0; p < 0x20; p++)
			s_main09.rpage[p] = s_main09.wpage[p] = t->ram + p * 256;
	} else {
		hd6309_init(&s_main);
		s_main.read = rd;
		s_main.write = wr;
		for (int p = 0x80; p < 0x100; p++)
			s_main.rpage[p] = s_rom_prg + p * 256;
		const int ramp = cg ? 0x20 : s_mach == M_DD2 ? 0x18 : 0x10;
		for (int p = 0; p < ramp; p++)
			s_main.rpage[p] = s_main.wpage[p] = t->ram + p * 256;
	}
	map_bank();
	// the sub CPU (MAME: running after reset; the main CPU holds it in reset with bankswitch bit 3 = 0)
	switch (s_sub_kind) {
	case SUB_63701:
	case SUB_6803:
		memset(&s_mcu, 0, sizeof(s_mcu));
		s_mcu.read = mcu_read;
		s_mcu.write = mcu_write;
		s_mcu.port_w = s_sub_kind == SUB_6803 ? mcu_port_w : NULL;
		s_mcu.type = s_sub_kind == SUB_63701 ? M6801_TYPE_HD63701V0 : M6801_TYPE_6801;
		if (s_sub_kind == SUB_63701) {
			// (Dark Tower's boards load the 63701 code at 0xc000 of a 64 KB region: MAME 0.260's sub map read it
			// there; 0.289's HD63701Y0 would take the region's empty first 16 KB)
			s_mcu.rom = s_mach == M_DARK ? s_srom + 0xc000 : s_srom;
			s_mcu.rom_base = 0xc000;
			s_mcu.rom_size = 0x4000;
		} else {
			s_mcu.rom = s_srom + 0xc000;
			s_mcu.rom_base = 0xc000;
			s_mcu.rom_size = 0x4000;
		}
		m6801_init(&s_mcu);
		m6801_reset(&s_mcu);
		break;
	case SUB_6309:
		hd6309_init(&s_sub6309);
		s_sub6309.read = cg ? cgsub_read : sub6309_read;
		s_sub6309.write = cg ? cgsub_write : sub6309_write;
		for (int p = 0xc0; p < 0x100; p++)
			s_sub6309.rpage[p] = s_srom + p * 256;
		if (cg) {
			for (int p = 0x80; p < 0xc0; p++)
				s_sub6309.rpage[p] = s_srom + p * 256;
			for (int p = 0; p < 0x20; p++)
				s_sub6309.rpage[p] = s_sub6309.wpage[p] = t->ram + p * 256;
			for (int p = 0; p < 0x40; p++)
				s_sub6309.rpage[0x40 + p] = s_srom + 0x10000 + p * 256;
		}
		hd6309_reset(&s_sub6309);
		break;
	case SUB_6809:
		m6809_init(&s_sub09);
		s_sub09.read = cgsub_read;
		s_sub09.write = cgsub_write;
		for (int p = 0x80; p < 0x100; p++)
			s_sub09.rpage[p] = s_srom + p * 256;
		for (int p = 0; p < 0x20; p++)
			s_sub09.rpage[p] = s_sub09.wpage[p] = t->ram + p * 256;
		for (int p = 0; p < 0x40; p++)
			s_sub09.rpage[0x40 + p] = s_srom + 0x10000 + p * 256;
		m6809_reset(&s_sub09);
		break;
	case SUB_Z80:
		subz_start();
		break;
	}
	if (s_mach == M_DARK) {
		m68705_reset(&s_m705);
	}
	// sound
	if (s_snd_z80)
		sndz_start();
	else {
		m6809_init(&s_snd09);
		s_snd09.read = snd09_read;
		s_snd09.write = snd09_write;
		for (int p = 0x80; p < 0x100; p++)
			s_snd09.rpage[p] = s_zrom + p * 256;
		for (int p = 0; p < 0x10; p++)
			s_snd09.rpage[p] = s_snd09.wpage[p] = t->sndram + p * 256;
		m6809_reset(&s_snd09);
	}
	rbx_ym_reset(0);
	if (s_two_ym)
		rbx_ym_reset(1);
	if (s_adpcm[0]) {
		for (int i = 0; i < 2; i++) {
			msm5205_reset(&s_msm[i]);
			msm5205_reset_w(&s_msm[i], 1);
		}
	}
	if (s_snd_z80 && !s_two_ym) {
		oki_reset(&s_oki);
		s_oki.bank = 0;
	}
	s_mix_n = 0;
	if (s_main6809) m6809_reset(&s_main09); else hd6309_reset(&s_main);
	s_line = 0;
}

static void dd_exit(void);

static int dd_init(const rbx_game *g)
{
	free(t);
	t = calloc(1, sizeof(*t));
	if (!t) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	const char *m = g->machine;
	s_mach = mach_is(m, "ddragonbl") ? M_DDBL : mach_is(m, "ddragonbla") ? M_DDBLA : mach_is(m, "ddragon2") ? M_DD2
	       : mach_is(m, "darktowr") ? M_DARK : mach_is(m, "toffy") ? M_TOFFY : mach_is(m, "chinagat") ? M_CG
	       : mach_is(m, "saiyugoub2") ? M_CGB2 : M_DD;
	const char *init = strchr(m, ':');
	init = init ? init + 1 : "";
	s_tstrike = !strcmp(g->set, "tstrike");
	s_hw = s_mach == M_DD2 ? 2 : (s_mach == M_CG || s_mach == M_CGB2) ? 1 : 0;
	s_main6809 = s_mach == M_CGB2;
	s_sub_kind = s_mach == M_DD || s_mach == M_DARK ? SUB_63701 : s_mach == M_DDBL ? SUB_6309
	           : s_mach == M_DDBLA ? SUB_6803 : s_mach == M_DD2 ? SUB_Z80 : s_mach == M_CG ? SUB_6309
	           : s_mach == M_CGB2 ? SUB_6809 : SUB_NONE;
	s_snd_z80 = s_mach == M_DD2 || s_hw == 1;
	s_two_ym = s_mach == M_CGB2;
	s_main_clock = s_hw == 1 ? 1500000 : 3000000;
	s_sub_clock = s_sub_kind == SUB_Z80 ? 4000000 : 1500000;
	s_snd_clock = s_snd_z80 ? 3579545 : 1500000;

	uint32_t csize, ssize, tsize, osize = 0, msize = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = s_sub_kind != SUB_NONE ? rbx_region("sub", &s_srom_size) : NULL;
	s_zrom = rbx_region("soundcpu", &s_zrom_size);
	uint8_t *chars = rbx_region("chars", &csize), *spr = rbx_region("sprites", &ssize), *tiles = rbx_region("tiles", &tsize);
	uint8_t *oki = (s_snd_z80 && !s_two_ym) ? rbx_region("oki", &osize) : NULL;
	s_adpcm[0] = s_adpcm[1] = NULL;
	if (!s_snd_z80 && s_mach != M_TOFFY) {
		s_adpcm[0] = rbx_region("adpcm1", &s_adpcm_size[0]);
		s_adpcm[1] = rbx_region("adpcm2", &s_adpcm_size[1]);
	}
	s_mcurom = s_mach == M_DARK ? rbx_region("mcu", &msize) : NULL;
	if (!s_rom || s_rom_size < 0x10000 || (s_sub_kind != SUB_NONE && (!s_srom || s_srom_size < 0x4000)) || !s_zrom ||
	    s_zrom_size < 0x10000 || !chars || !spr || !tiles || (s_snd_z80 && !s_two_ym && !oki) ||
	    (!s_snd_z80 && s_mach != M_TOFFY && (!s_adpcm[0] || !s_adpcm[1])) || (s_mach == M_DARK && !s_mcurom)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_rom_prg = s_rom;
	if (!strcmp(init, "toffy")) {   // init_toffy
		for (uint32_t i = 0; i < s_rom_size; i++) {
			const uint8_t v = s_rom[i];
			s_rom[i] = (uint8_t)((v & 0x3f) | ((v >> 1) & 0x40) | ((v << 1) & 0x80));   // bitswap 6,7,5,4,3,2,1,0
		}
		for (uint32_t i = 0; i < csize; i++) {   // 7,6,5,3,4,2,1,0
			const uint8_t v = chars[i];
			chars[i] = (uint8_t)((v & 0xe7) | ((v >> 1) & 0x08) | ((v << 1) & 0x10));
		}
		for (uint32_t i = 0; i < ssize; i++) {   // 7,6,5,4,3,2,0,1
			const uint8_t v = spr[i];
			spr[i] = (uint8_t)((v & 0xfc) | ((v >> 1) & 0x01) | ((v << 1) & 0x02));
		}
		for (uint32_t i = 0; i < tsize / 2; i++) {
			uint8_t v = tiles[i];   // 7,6,1,4,3,2,5,0: bit5 <- 1, bit1 <- 5
			tiles[i] = (uint8_t)((v & 0xdd) | ((v >> 4) & 0x02) | ((v << 4) & 0x20));
			v = tiles[i + tsize / 2];   // 7,6,2,4,3,5,1,0: bit5 <- 2, bit2 <- 5
			tiles[i + tsize / 2] = (uint8_t)((v & 0xdb) | ((v >> 3) & 0x04) | ((v << 3) & 0x20));
		}
	}
	{	// char_layout, tile_layout (RGN_FRAC(1,2))
		static const uint32_t pc[4] = { 0, 2, 4, 6 };
		static const uint32_t xc[8] = { 1, 0, 8 * 8 + 1, 8 * 8 + 0, 16 * 8 + 1, 16 * 8 + 0, 24 * 8 + 1, 24 * 8 + 0 };
		static const uint32_t yc[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
		static const uint32_t xt[16] = { 3, 2, 1, 0, 16 * 8 + 3, 16 * 8 + 2, 16 * 8 + 1, 16 * 8 + 0,
		                                 32 * 8 + 3, 32 * 8 + 2, 32 * 8 + 1, 32 * 8 + 0, 48 * 8 + 3, 48 * 8 + 2, 48 * 8 + 1, 48 * 8 + 0 };
		static uint32_t yt[16];
		for (int i = 0; i < 16; i++)
			yt[i] = (uint32_t)(i * 8);
		const uint32_t sh = ssize * 4, th = tsize * 4;
		const uint32_t ps[4] = { sh + 0, sh + 4, 0, 4 }, pt[4] = { th + 0, th + 4, 0, 4 };
		const rbx_gfx_layout lc = { 8, 8, 4, pc, xc, yc, 32 * 8 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, xt, yt, 64 * 8 };
		const rbx_gfx_layout lt = { 16, 16, 4, pt, xt, yt, 64 * 8 };
		if (rbx_gfx_decode(&s_gfxchar, &lc, chars, csize) || rbx_gfx_decode(&s_gfxspr, &ls, spr, ssize / 2) ||
		    rbx_gfx_decode(&s_gfxtile, &lt, tiles, tsize / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			dd_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	s_vb[0] = calloc(1, sizeof(vbuf));
	s_vb[1] = calloc(1, sizeof(vbuf));
	s_vb_cur = 0;
	s_vdirty = 0;
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	int rc = !s_snap || !s_vb[0] || !s_vb[1];
	if (s_two_ym) {   // 2x YM2203: SSG 0.50 each, FM 0.80
		yc.type = RBX_YM2203;
		yc.clock = YM_CLOCK;
		yc.gain[0] = yc.gain[1] = yc.gain[2] = 128;
		yc.gain[3] = 205;
		yc.irq = ym_irq;
		rc = rc || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
		yc.irq = NULL;
		rc = rc || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	} else {
		yc.type = RBX_YM2151;
		yc.clock = YM_CLOCK;
		yc.gain[0] = yc.gain[1] = s_hw == 1 ? 205 : 90;   // (0.80 China Gate, 0.35 Double Dragon)
		yc.irq = ym_irq;
		rc = rc || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		dd_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 512)) {
		rbx_fail("%s: out of memory", g->set);
		dd_exit();
		return -1;
	}
	if (oki)
		oki_init(&s_oki, s_hw == 1 ? 1065000 : 1056000, 1, oki, osize, s_hw == 1 ? 205 : 51);
	if (s_adpcm[0]) {
		msm5205_init(&s_msm[0], 12000000 / 32, MSM5205_S48_4B, adpcm_int0);
		msm5205_init(&s_msm[1], 12000000 / 32, MSM5205_S48_4B, adpcm_int1);
	}
	memset(s_zmap, 0, sizeof(s_zmap));
	memset(s_szmap, 0, sizeof(s_szmap));
	if (s_snd_z80)
		for (int p = 0; p < 0x80; p++)
			s_zmap[p] = s_zrom + p * 256;
	if (s_sub_kind == SUB_Z80)
		for (int p = 0; p < 0xc0; p++)
			s_szmap[p] = s_srom + p * 256;
	if (s_mach == M_DARK) {
		if (msize < 0x800 || m68705_init(&s_m705, M68705_P3, s_mcurom, 0x800)) {
			rbx_fail("%s: MCU ROM missing", g->set);
			dd_exit();
			return -1;
		}
		s_m705.port_w[0] = m705_port_w;
	}
	if (s_hw == 1) {
		static const char *const names[5] = { "SYSTEM", "DSW1", "DSW2", "P1", "P2" };
		for (int i = 0; i < 5; i++)
			s_port[i] = rbx_port_find(names[i]);
	} else {
		static const char *const names[5] = { "P1", "P2", "EXTRA", "DSW0", "DSW1" };
		for (int i = 0; i < 5; i++)
			s_port[i] = rbx_port_find(names[i]);
	}
	dd_reset();
	return 0;
}

static void dd_exit(void)
{
	rbx_video_wait_idle();
	rbx_ym_exit(0);
	if (s_two_ym)
		rbx_ym_exit(1);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gfxchar);
	rbx_gfx_free(&s_gfxspr);
	rbx_gfx_free(&s_gfxtile);
	free(s_snap);
	s_snap = NULL;
	free(s_vb[0]);
	free(s_vb[1]);
	s_vb[0] = s_vb[1] = NULL;
	free(t);
	t = NULL;
}

static void main_slice(void)
{
	const int cycles = rbx_line_cycles(&t->main_frac, s_main_clock, FRAME_MHZ, LINES * SLICES);
	t->main_t += cycles;
	if (s_main6809) {
		if (s_main09.cycles < t->main_t) m6809_run_until(&s_main09, t->main_t);
	} else if (s_main.cycles < t->main_t)
		hd6309_run_until(&s_main, t->main_t);
}

static void dd_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		// (scanline timer)
		const int vcount_old = scanline_to_vcount(line == 0 ? LINES - 1 : line - 1), vcount = scanline_to_vcount(line);
		if (vcount == 0xf8) {
			vblank_capture();
			main_set(0, 1);
		}
		if (!(vcount_old & 8) && (vcount & 8))
			main_set(1, 1);
		for (int s = 0; s < SLICES; s++) {
			main_slice();
			sub_slice();
			sound_slice();
			if (s_mach == M_DARK) {
				const int c = rbx_line_cycles(&t->mcu_frac, 1000000, FRAME_MHZ, LINES * SLICES);
				m68705_execute(&s_m705, c);
			}
		}
		sound_line();
		if (line < VIS_H) {   // (the registers this line was drawn with: update_partial at the next line's start)
			line_regs *r = &t->lines[line];
			r->sx = (uint16_t)(t->scrollx_hi << 8 | t->scrollx_lo);
			r->sy = (uint16_t)(t->scrolly_hi << 8 | t->scrolly_lo);
			r->flip = t->flip;
			vram_line(line);
		}
	}
	sound_frame_end();
}

static int dd_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "dd", t, (uint32_t)offsetof(__typeof__(*t), state_end));
	if (s_main6809)
		rc |= rbx_state_io(ctx, "dd_main", &s_main09, (uint32_t)offsetof(m6809_t, rpage));
	else
		rc |= rbx_state_io(ctx, "dd_main", &s_main, HD6309_STATE_SIZE);
	switch (s_sub_kind) {
	case SUB_63701:
	case SUB_6803:
		rc |= rbx_state_io(ctx, "dd_mcu", m6801_state_data(&s_mcu), m6801_state_size());
		break;
	case SUB_6309:
		rc |= rbx_state_io(ctx, "dd_sub", &s_sub6309, HD6309_STATE_SIZE);
		break;
	case SUB_6809:
		rc |= rbx_state_io(ctx, "dd_sub", &s_sub09, (uint32_t)offsetof(m6809_t, rpage));
		break;
	case SUB_Z80: {
		z80 z = s_subz;
		rc |= rbx_state_io(ctx, "dd_subz", &z, sizeof(z));
		if (!rc && load) {
			subz_start();
			z.read_byte = s_subz.read_byte;
			z.write_byte = s_subz.write_byte;
			z.port_in = s_subz.port_in;
			z.port_out = s_subz.port_out;
			z.userdata = NULL;
			z.read_map = s_szmap;
			s_subz = z;
		}
		break;
	}
	}
	if (s_snd_z80) {
		z80 z = s_sndz;
		rc |= rbx_state_io(ctx, "dd_sndz", &z, sizeof(z));
		if (!rc && load) {
			z.read_byte = sndz_read;
			z.write_byte = sndz_write;
			z.port_in = z_in;
			z.port_out = z_out;
			z.userdata = NULL;
			z.read_map = s_zmap;
			s_sndz = z;
		}
	} else
		rc |= rbx_state_io(ctx, "dd_snd", &s_snd09, (uint32_t)offsetof(m6809_t, rpage));
	if (s_mach == M_DARK)
		rc |= rbx_state_io(ctx, "dd_m705", &s_m705.st, sizeof(s_m705.st));
	rc |= rbx_ym_state(0, ctx, load);
	if (s_two_ym)
		rc |= rbx_ym_state(1, ctx, load);
	if (s_adpcm[0]) {
		rc |= rbx_state_io(ctx, "dd_msm0", &s_msm[0], (uint32_t)offsetof(msm5205_t, vck));
		rc |= rbx_state_io(ctx, "dd_msm1", &s_msm[1], (uint32_t)offsetof(msm5205_t, vck));
	}
	if (s_snd_z80 && !s_two_ym)
		rc |= rbx_state_io(ctx, "dd_oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	if (!rc && load) {
		map_bank();
		if (s_hw == 1 && s_sub_kind != SUB_NONE)
			cgsub_write(NULL, 0x2000, t->subbank);
	}
	return rc ? -1 : 0;
}

static int dd_save(void *ctx) { return dd_state(ctx, 0); }
static int dd_load(void *ctx) { return dd_state(ctx, 1); }
static int dd_supported(const char *machine)
{
	static const char *const names[] = { "ddragon", "ddragonbl", "ddragonbla", "ddragon2", "darktowr", "toffy",
	                                     "chinagat", "saiyugoub2" };
	for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++)
		if (mach_is(machine, names[i]))
			return 1;
	return 0;
}

const rbx_driver rbx_drv_ddragon = { "Technos Double Dragon", dd_init, dd_exit, dd_reset, dd_frame, dd_save, dd_load,
                                     dd_supported, dd_render, NULL };
