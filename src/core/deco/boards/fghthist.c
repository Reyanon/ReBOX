/*  Data East DECO 32 (DE-0395-1): Fighter's History. Ported from MAME src/mame/dataeast/fghthist.cpp and
    fghthist_v.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail; split from deco32.cpp).
    156 (its encryption unused) at 7 MHz; tilemap chips 56 and 74, sprite chip 52 (buffered), 146 protection / I/O
    (interleaved address lines, the magic read xor), a buffered palette updated by DMA, 93C46 EEPROM; sound: HuC6280
    with a YM2151 (its CT port banks the OKIs) and two OKI M6295 (fghthisto: a plain sound latch and its own I/O).
    Night Slashers (DE-0397-0): the 156's encryption used, the 104 protection, two sprite chips drawn raw and mixed
    with the playfields as MAME's mix_nslasher (DECO ACE: faded and raw palettes, alpha), output in RGB.
    Tattoo Assassins: the Night Slashers video without the encryption, a 1K byte-wide EEPROM read as a bit stream
    from any byte (MAME tattass_control_w), the BSMT2000 sound board of the pinballs (6809 + TMS320C15).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/arm/arm.h"
#include "../../cpu/h6280/h6280.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../machine/eeprom93c46.h"
#include "../../video/gfxlayout.h"
#include "../machine/decocrpt.h"
#include "../machine/deco146.h"
#include "../machine/deco156.h"
#include "../machine/armpoll.h"
#include "../machine/decobsmt.h"
#include "../video/deco16ic.h"
#include "../video/decospr.h"
#include "../video/decoband.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 7000000        // 28 MHz / 4
#define SND_CLOCK 2685000        // 32.22 MHz / 4 / 3
#define YM_CLOCK 3580000         // 32.22 MHz / 9
#define LINES 274                // set_raw(28 MHz / 4, 442, 0, 320, 274, 8, 248)
#define FRAME_MHZ 57800          // 7 MHz / (442 x 274)
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 320

static arm_t s_cpu;
static h6280_t s_snd;
static uint8_t *s_rom, *s_snd_rom;
static uint32_t s_rom_size, s_snd_rom_size;
static uint8_t s_ram[0x20000];                 // 0x100000 (32-bit)
static uint8_t s_snd_ram[0x2000];
static uint32_t s_palram[0x800];                 // 0x168000 buffered palette (red bits 0-7, green 8-15, blue 16-23)
static uint8_t s_pal_dirty[0x800];
static uint32_t s_pal[0x800];                    // the palette the DMA last set (0x00RRGGBB)
static uint16_t s_spriteram[0x800], s_spriteram_buf[0x800];
static uint16_t s_prot_lo[0x2000];               // (the low halves of 0x200000: the 146 only maps the high ones)
static uint32_t s_rowscroll32[4][0x800];
static uint16_t s_rowscroll[4][0x800];
static uint32_t s_pri;
static deco16ic_t s_tile[2];
static decospr_t s_spr;
static deco146_t s_prot;
static eeprom93c46_t s_eeprom;
static armpoll_t s_poll;
static char s_nv_path[256];
static int s_old;                                // fghthisto: its I/O at 0x120020, a plain sound latch
static int s_zsnd;                               // fghthistu: a Z80 for the sound (its IRQ0: the latch or the YM)
static int s_ns;                                 // Night Slashers (and Tattoo Assassins' video)
static int s_tt;                                 // Tattoo Assassins
// tattass: the EEPROM (MAME internal_read / write of the 93C76 8-bit) and its bit stream, the 'Jack' RAM, the BSMT
static uint8_t s_tt_eep[0x400];
static int s_tt_eep_dirty;
static struct { uint32_t buffer; int32_t buf_ptr, pending, read_bits, byte_addr, last_clock, bit; } s_tte;
static uint8_t s_tt_ram[0x1000];
static decobsmt_t *s_bsmt;
static uint64_t s_bsmt_frac;
// The board only hears the main CPU (the comms byte, the 6809's reset line), so it runs a frame behind on a worker
// (rbx_aux): the frame's writes queued with their line, the job runs the board through that frame applying them at
// their lines, its samples go out with the next frame (the same samples with or without the worker thread)
static int s_line, s_mix_cap;   // (defined below)
#define TT_EV_MAX 1024
typedef struct { uint16_t line; uint8_t type, val; } tt_ev_t;   // type 0: comms, 1: reset line
static tt_ev_t s_tt_ev[2][TT_EV_MAX];
static int s_tt_ev_n[2], s_tt_ev_cur, s_tt_rst_last;
static int32_t *s_tt_out_l, *s_tt_out_r;
static int s_tt_out_n;
static volatile int s_tt_job_frame;   // a frame queued (the job does the work queued: a sync runs it again)
static void tt_event(int type, int val)
{
	if (type == 1) {
		if (val == s_tt_rst_last)
			return;
		s_tt_rst_last = val;
	}
	const int c = s_tt_ev_cur;
	if (s_tt_ev_n[c] < TT_EV_MAX) {
		tt_ev_t *e = &s_tt_ev[c][s_tt_ev_n[c]++];
		e->line = (uint16_t)s_line;
		e->type = (uint8_t)type;
		e->val = (uint8_t)val;
	}
}
static void tt_job(void)   // (the frame queued last: the other buffer than the current one)
{
	if (!__atomic_load_n(&s_tt_job_frame, __ATOMIC_ACQUIRE))
		return;
	const int b = s_tt_ev_cur ^ 1;
	const tt_ev_t *e = s_tt_ev[b];
	int k = 0, n = 0;
	memset(s_tt_out_l, 0, sizeof(int32_t) * (size_t)s_mix_cap);   // (decobsmt_run adds)
	memset(s_tt_out_r, 0, sizeof(int32_t) * (size_t)s_mix_cap);
	for (int line = 0; line < LINES; line++) {
		for (; k < s_tt_ev_n[b] && e[k].line == line; k++) {
			if (e[k].type == 0)
				decobsmt_comms_w(s_bsmt, e[k].val);
			else
				decobsmt_reset_line(s_bsmt, e[k].val);
		}
		const uint32_t mc = (uint32_t)rbx_line_cycles(&s_bsmt_frac, 24000000, FRAME_MHZ, LINES);
		n += decobsmt_run(s_bsmt, mc, s_tt_out_l + n, s_tt_out_r + n, s_mix_cap - n);
	}
	s_tt_ev_n[b] = 0;
	s_tt_out_n = n;
	__atomic_store_n(&s_tt_job_frame, 0, __ATOMIC_RELEASE);
}
// nslasher: the DECO ACE (its RAM, the palette its DMA took, the faded / raw pens), the second sprite list, the
// colour banks
static uint16_t s_ace[0x28];
static uint32_t s_palbuf[0x800];
static uint32_t s_pens[0x1000];
static uint16_t s_spriteram2[0x800], s_spriteram2_buf[0x800];
static uint8_t s_spr_bank[2];
static decospr_t s_spr2;
static void ace_palette_update(void);
static int s_pens_dirty;   // (the pens computed again at the next vblank: only the drawing uses them)
static rbx_gfx s_gfxspr2;
static z80 s_z80;
static uint8_t s_z80_ram[0x800];
static int s_zirq_latch, s_zirq_ym, s_zirq;
static uint8_t s_latch, s_latch_pending, s_volume;
static rbx_gfx s_gfx8, s_gfx16, s_gfx16b, s_gfxspr;
static okim6295_t s_oki[2];
static int s_port_in0, s_port_in1;
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac, s_snd_frac, s_ym_frac;
// sound: the YM2151's rate out (stereo), the OKIs' samples held between their own
static int32_t *s_mix_l, *s_mix_r;
static int s_mix_cap, s_mix_n;
static uint32_t s_out_rate, s_oki_phase[2];
static int32_t s_oki_last[2];
static uint8_t s_pri_buf[VIS_W * VIS_H];
static struct {
	deco16ic_t tile[2];
	uint16_t rowscroll[4][0x800], spriteram[0x800];
	uint32_t pal[0x800], pri, frame;
	uint16_t spriteram2[0x800], ace[0x28];
	uint32_t pens[0x1000];
	uint8_t spr_bank[2];
} *s_snap;

static uint32_t rd32le(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }
static void wr32m(uint8_t *p, uint32_t data, uint32_t mask)
{
	for (int i = 0; i < 4; i++)
		if (mask & (0xffu << (8 * i)))
			p[i] = (uint8_t)(data >> (8 * i));
}

// ---- interrupts / ports ----
static void latch_irq(void *u, int state)
{
	(void)u;
	if (s_tt) {   // tattass_sound_irq_w: the latch, bits 0 and 3 swapped, to the BSMT board
		if (state) {
			uint8_t d = deco146_soundlatch_r(&s_prot);
			d = (uint8_t)((d & 0xf6) | ((d >> 3) & 1) | ((d & 1) << 3));
			tt_event(0, d);
		}
		return;
	}
	if (s_zsnd) {
		s_zirq_latch = state;
		s_zirq = s_zirq_latch || s_zirq_ym;   // (input merger, any high)
	} else
		h6280_set_irq(&s_snd, H6280_IRQ1, state);
}
static void ym_irq(int state)
{
	if (s_zsnd) {
		s_zirq_ym = state;
		s_zirq = s_zirq_latch || s_zirq_ym;
	} else
		h6280_set_irq(&s_snd, H6280_IRQ2, state);
}

// fghthistu's Z80: ROM 0000-7fff, RAM 8000-87ff, YM a000, OKIs b000 / c000, the 146's latch d000; its I/O space
// reads the sound ROM (z80_sound_io)
static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_snd_rom[a & (s_snd_rom_size - 1)];
	if (a < 0x8800) return s_z80_ram[a & 0x7ff];
	if ((a & 0xfffe) == 0xa000) return rbx_ym_read(0, a & 1);
	if (a == 0xb000) return oki_read(&s_oki[0]);
	if (a == 0xc000) return oki_read(&s_oki[1]);
	if (a == 0xd000) return deco146_soundlatch_r(&s_prot);
	return 0xff;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8800) s_z80_ram[a & 0x7ff] = v;
	else if ((a & 0xfffe) == 0xa000) rbx_ym_write(0, a & 1, v);
	else if (a == 0xb000) oki_write(&s_oki[0], v);
	else if (a == 0xc000) oki_write(&s_oki[1], v);
}
static uint8_t z_in(z80 *z, uint8_t port)
{
	const uint16_t a = z->mem_ptr;   // (the 16-bit port address)
	(void)port;
	return s_snd_rom[a & (s_snd_rom_size - 1)];
}
static uint16_t port_a(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_in0); }
static uint16_t port_b(void *u)   // (the EEPROM's data out, bit 0)
{
	(void)u;
	return s_tt ? (uint16_t)s_tte.bit : (uint16_t)eeprom_do(&s_eeprom);
}

// tattass_control_w: the EEPROM's bit stream (byte 0: bit 6 select, bit 5 clock, bit 4 data in), the priority (bits
// 0-2), the BSMT board's reset (bit 7 low)
static void tt_control_w(uint32_t data, uint32_t mask)
{
	if (mask & 0xff) {
		const int clk = (data >> 5) & 1, sel = (data >> 6) & 1, di = (data >> 4) & 1;
		if (!sel) {
			s_tte.buf_ptr = 0;
			s_tte.pending = 0;
			s_tte.read_bits = 0;
		}
		if (s_tte.last_clock == 0 && clk && sel) {
			if (s_tte.buf_ptr >= 32)
				s_tte.buf_ptr = 0;
			if (s_tte.pending == 1) {   // (multiple read: the next bit, across byte boundaries)
				const int d = s_tte.read_bits >> 3, m = 7 - (s_tte.read_bits & 7);
				s_tte.bit = (s_tt_eep[(s_tte.byte_addr + d) & 0x3ff] >> m) & 1;
				s_tte.read_bits++;
				s_tte.last_clock = clk;
				goto pri;
			}
			if (s_tte.pending == 2) {   // (write: the data byte)
				s_tte.buffer = (s_tte.buffer << 1) | (uint32_t)di;
				if (++s_tte.buf_ptr == 32) {
					s_tt_eep[s_tte.byte_addr & 0x3ff] = (uint8_t)s_tte.buffer;
					s_tt_eep_dirty = 1;
				}
				s_tte.last_clock = clk;
				goto pri;
			}
			s_tte.buffer = (s_tte.buffer << 1) | (uint32_t)di;
			if (++s_tte.buf_ptr == 24) {   // (the command: 110aa000 read, 000aa000 write; 8 zeroes; 8 address bits)
				s_tte.byte_addr = (int32_t)(((s_tte.buffer & 0x180000) >> 11) | (s_tte.buffer & 0xff));
				if ((s_tte.buffer & 0xc00000) == 0xc00000) {
					s_tte.bit = (s_tt_eep[s_tte.byte_addr & 0x3ff] >> 7) & 1;
					s_tte.read_bits = 1;
					s_tte.pending = 1;
				} else if ((s_tte.buffer & 0xc00000) == 0)
					s_tte.pending = 2;
			}
		} else if (!sel)
			s_tte.buf_ptr = 0;
		s_tte.last_clock = clk;
	}
pri:
	s_pri = data & 7;
	tt_event(1, !((data >> 7) & 1));
}
static uint16_t port_c(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_in1); }

static void eeprom_w(uint8_t data)   // cs bit 6, clk bit 5, di bit 4; bits 0-2: pri_w
{
	eeprom_write_lines(&s_eeprom, (data >> 6) & 1, (data >> 5) & 1, (data >> 4) & 1);
	s_pri = data & 7;
}
static void volume_apply(void)   // volume_w: 0x00 max .. 0xff min, on the YM and both OKIs (linear)
{
	const int raw = 0xff - s_volume;
	rbx_ym_set_volume(0, 256 * raw / 255);
	s_oki[0].gain = 256 * raw / 255;
	s_oki[1].gain = 90 * raw / 255;
}

// the 146's address: (real address bits 14-17 << 11) | bits 0-10 (NC 31-18, 13-11)
static uint16_t prot_addr(uint32_t o)
{
	const uint32_t real = (o >> 2) * 2;
	return (uint16_t)((((real >> 14) & 0xf) << 11) | (real & 0x7ff));
}

static inline void combine16(uint16_t *p, uint32_t data, uint32_t mask)
{
	mask &= 0xffff;
	*p = (uint16_t)((*p & ~mask) | (data & mask));
}

// the tilemap blocks: vram 0x182000 / 0x184000 (0x1c2000 / 0x1c4000), rowscroll 0x192000 / 0x194000 (0x1d2000 /
// 0x1d4000), control 0x1a0000 (0x1e0000)
static uint32_t *rows32(uint32_t a, int *idx)
{
	static const uint32_t base[4] = { 0x192000, 0x194000, 0x1d2000, 0x1d4000 };
	for (int i = 0; i < 4; i++)
		if (a >= base[i] && a < base[i] + 0x2000) {
			*idx = i;
			return &s_rowscroll32[i][(a - base[i]) >> 2];
		}
	return NULL;
}

static uint32_t bus_r32(void *ctx, uint32_t a)
{
	(void)ctx;
	a &= 0x03fffffc;
	if (a < 0x100000) return rd32le(s_rom + (a & (s_rom_size - 1)));
	if (a < 0x120000) {
		const uint32_t v = rd32le(s_ram + (a & 0x1fffc));
		if (!s_tt && a == (s_ns ? 0x100000u : 0x101a04u))   // (the wait for the interrupt's counter / flag: fghthist 0x9cf0,
			                                         // nslasher 0x9c4: armpoll)
			armpoll_read(&s_poll, &s_cpu, v, s_rom, s_rom_size);
		return v;
	}
	if (s_old && a == 0x120020) return rbx_port_read(s_port_in0) | 0xffff0000u;
	if (s_old && a == 0x120024) return rbx_port_read(s_port_in1) | 0xffff0000u;
	if (s_old && a == 0x120028) return 0xffffff00u | 0xfe | (uint32_t)eeprom_do(&s_eeprom);
	if (s_ns && a >= 0x163000 && a < 0x1630a0) return s_ace[(a - 0x163000) >> 2] | 0xffff0000u;
	if (s_tt && a >= 0x162000 && a < 0x163000) return rd32le(s_tt_ram + (a & 0xffc));   // ('Jack' RAM)
	if (a >= 0x168000 && a < 0x16a000) return s_palram[(a - 0x168000) >> 2];
	if (s_ns && a >= 0x170000 && a < 0x172000) return s_spriteram[(a - 0x170000) >> 2] ^ 0xffff0000u;
	if (s_ns && a >= 0x178000 && a < 0x17a000) return s_spriteram2[(a - 0x178000) >> 2] ^ 0xffff0000u;
	if (a == 0x16c010 || a == 0x17c020) return 0xfffffff7u;   // unk_status_r (bit 3 must be 0)
	if (!s_ns && a >= 0x178000 && a < 0x17a000) return s_spriteram[(a - 0x178000) >> 2] ^ 0xffff0000u;
	if (a >= 0x182000 && a < 0x184000) return s_tile[0].vram[0][(a - 0x182000) >> 2] | 0xffff0000u;
	if (a >= 0x184000 && a < 0x186000) return s_tile[0].vram[1][(a - 0x184000) >> 2] | 0xffff0000u;
	if (a >= 0x1a0000 && a < 0x1a0020) return deco16ic_control_r(&s_tile[0], (int)((a - 0x1a0000) >> 2)) | 0xffff0000u;
	if (a >= 0x1c2000 && a < 0x1c4000) return s_tile[1].vram[0][(a - 0x1c2000) >> 2] | 0xffff0000u;
	if (a >= 0x1c4000 && a < 0x1c6000) return s_tile[1].vram[1][(a - 0x1c4000) >> 2] | 0xffff0000u;
	if (a >= 0x1e0000 && a < 0x1e0020) return deco16ic_control_r(&s_tile[1], (int)((a - 0x1e0000) >> 2)) | 0xffff0000u;
	if (a >= 0x200000 && a < 0x208000)   // (umask32 0xffff0000: the 146 on the high halves; nslasher's low: debug_r)
		return (uint32_t)deco146_read(&s_prot, prot_addr(a - 0x200000), NULL) << 16 |
		       (s_ns ? 0xffffu : s_prot_lo[(a - 0x200000) >> 2]);
	int i;
	const uint32_t *r = rows32(a, &i);
	if (r)
		return *r;
	return s_old ? 0xffffffffu : 0;   // (fghthisto: unmap_value_high)
}

static void bus_w32m(uint32_t a, uint32_t data, uint32_t mask)
{
	a &= 0x03fffffc;
	if (a >= 0x100000 && a < 0x120000) { wr32m(s_ram + (a & 0x1fffc), data, mask); return; }
	if (s_old && a == 0x12002c) {   // eeprom_w (byte 0), volume_w (byte 1)
		if (mask & 0xff) eeprom_w((uint8_t)data);
		if (mask & 0xff00) { s_volume = (uint8_t)(data >> 8); volume_apply(); }
		return;
	}
	if (s_old && a == 0x1201fc) {   // the sound latch (byte 0): the sound CPU's IRQ1 while pending
		if (mask & 0xff) {
			s_latch = (uint8_t)data;
			s_latch_pending = 1;
			latch_irq(NULL, 1);
		}
		return;
	}
	if (a == 0x140000) { arm_set_irq(&s_cpu, 0); return; }   // vblank_ack_w
	if (s_tt && a == 0x150000) {
		tt_control_w(data, mask);
		return;
	}
	if (s_tt && a >= 0x162000 && a < 0x163000) { wr32m(s_tt_ram + (a & 0xffc), data, mask); return; }
	if (!s_old && a == 0x150000) {
		if (mask & 0xff) eeprom_w((uint8_t)data);
		if (mask & 0xff00) { s_volume = (uint8_t)(data >> 8); volume_apply(); }
		return;
	}
	if (s_ns) {   // (nslasher_common_map)
		if (a >= 0x163000 && a < 0x1630a0) {   // ace_w (umask32 0x0000ffff)
			const uint32_t i = (a - 0x163000) >> 2;
			s_ace[i] = (uint16_t)((s_ace[i] & ~mask) | (data & mask & 0xffff));
			if (i >= 0x20 && i <= 0x26)
				s_pens_dirty = 1;
			return;
		}
		if (a == 0x164000) {   // tilemap_color_bank_w: the second chip's colour banks
			if (mask & 0xff) {
				s_tile[1].tm[0].col_bank = (int)((data & 7) << 4);
				s_tile[1].tm[1].col_bank = (int)(((data >> 3) & 7) << 4);
			}
			return;
		}
		if (a == 0x164004 || a == 0x164008) {   // sprite1 / sprite2_color_bank_w (the gfx colour base, << 8)
			if (mask & 0xff)
				s_spr_bank[a == 0x164008] = (uint8_t)(data & 7);
			return;
		}
		if (a == 0x16c008) {   // palette_dma_w: the buffered palette taken, the pens again
			memcpy(s_palbuf, s_palram, sizeof(s_palbuf));
			s_pens_dirty = 1;
			return;
		}
		if (a >= 0x170000 && a < 0x172000) { combine16(&s_spriteram[(a - 0x170000) >> 2], data, mask); return; }
		if (a == 0x174010) { memcpy(s_spriteram_buf, s_spriteram, sizeof(s_spriteram)); return; }
		if (a >= 0x178000 && a < 0x17a000) { combine16(&s_spriteram2[(a - 0x178000) >> 2], data, mask); return; }
		if (a == 0x17c010) { memcpy(s_spriteram2_buf, s_spriteram2, sizeof(s_spriteram2)); return; }
		if (a >= 0x168000 && a < 0x16a000) {
			const uint32_t i = (a - 0x168000) >> 2;
			s_palram[i] = (s_palram[i] & ~mask) | (data & mask);
			return;
		}
	}
	if (a >= 0x168000 && a < 0x16a000) {   // buffered_palette_w
		const uint32_t i = (a - 0x168000) >> 2;
		s_palram[i] = (s_palram[i] & ~mask) | (data & mask);
		s_pal_dirty[i] = 1;
	} else if (a == 0x16c008) {   // palette_dma_w: the dirty entries to the palette
		for (int i = 0; i < 0x800; i++)
			if (s_pal_dirty[i]) {
				s_pal_dirty[i] = 0;
				s_pal[i] = (s_palram[i] & 0xff) << 16 | (s_palram[i] & 0xff00) | ((s_palram[i] >> 16) & 0xff);
			}
	} else if (a >= 0x178000 && a < 0x17a000) combine16(&s_spriteram[(a - 0x178000) >> 2], data, mask);
	else if (a == 0x17c010) memcpy(s_spriteram_buf, s_spriteram, sizeof(s_spriteram));   // buffer_spriteram_w
	else if (a >= 0x182000 && a < 0x184000) combine16(&s_tile[0].vram[0][(a - 0x182000) >> 2], data, mask);
	else if (a >= 0x184000 && a < 0x186000) combine16(&s_tile[0].vram[1][(a - 0x184000) >> 2], data, mask);
	else if (a >= 0x1a0000 && a < 0x1a0020) {
		if (mask & 0xffff) deco16ic_control_w(&s_tile[0], (int)((a - 0x1a0000) >> 2), (uint16_t)data, (uint16_t)mask);
	} else if (a >= 0x1c2000 && a < 0x1c4000) combine16(&s_tile[1].vram[0][(a - 0x1c2000) >> 2], data, mask);
	else if (a >= 0x1c4000 && a < 0x1c6000) combine16(&s_tile[1].vram[1][(a - 0x1c4000) >> 2], data, mask);
	else if (a >= 0x1e0000 && a < 0x1e0020) {
		if (mask & 0xffff) deco16ic_control_w(&s_tile[1], (int)((a - 0x1e0000) >> 2), (uint16_t)data, (uint16_t)mask);
	} else if (a >= 0x200000 && a < 0x208000) {
		uint16_t *lo = &s_prot_lo[(a - 0x200000) >> 2];
		*lo = (uint16_t)((*lo & ~mask) | (data & mask & 0xffff));
		if (mask & 0xffff0000u)
			deco146_write(&s_prot, prot_addr(a - 0x200000), (uint16_t)(data >> 16), (uint16_t)(mask >> 16), NULL);
	} else {
		int i;
		uint32_t *r = rows32(a, &i);
		if (r) {   // rowscroll_w: the 32-bit RAM and its low halves for the tilemaps
			*r = (*r & ~mask) | (data & mask);
			combine16(&s_rowscroll[i][r - s_rowscroll32[i]], data, mask);
		}
	}
}

static void bus_w32(void *ctx, uint32_t a, uint32_t data) { (void)ctx; bus_w32m(a, data, 0xffffffffu); }
static uint8_t bus_r8(void *ctx, uint32_t a) { return (uint8_t)(bus_r32(ctx, a) >> (8 * (a & 3))); }
static void bus_w8(void *ctx, uint32_t a, uint8_t v)
{
	(void)ctx;
	const int sh = 8 * (int)(a & 3);
	bus_w32m(a, (uint32_t)v << sh, 0xffu << sh);
}

// ---- sound CPU (21-bit physical addresses) ----
static uint8_t snd_r(void *ctx, uint32_t pa)
{
	(void)ctx;
	if (pa < 0x10000) return s_snd_rom[pa & (s_snd_rom_size - 1)];
	if ((pa & ~1u) == 0x110000) return rbx_ym_read(0, pa & 1);   // (the YM runs line by line)
	if ((pa & ~1u) == 0x120000) return oki_read(&s_oki[0]);
	if ((pa & ~1u) == 0x130000) return oki_read(&s_oki[1]);
	if (pa == 0x140000) {
		if (!s_old)
			return deco146_soundlatch_r(&s_prot);
		s_latch_pending = 0;   // (generic latch: a read acknowledges it)
		latch_irq(NULL, 0);
		return s_latch;
	}
	if (pa >= 0x1f0000 && pa < 0x1f2000) return s_snd_ram[pa & 0x1fff];
	return 0;
}
static void snd_w(void *ctx, uint32_t pa, uint8_t v)
{
	(void)ctx;
	if ((pa & ~1u) == 0x110000) rbx_ym_write(0, pa & 1, v);
	else if ((pa & ~1u) == 0x120000) oki_write(&s_oki[0], v);
	else if ((pa & ~1u) == 0x130000) oki_write(&s_oki[1], v);
	else if (pa >= 0x1f0000 && pa < 0x1f2000) s_snd_ram[pa & 0x1fff] = v;
}
static void ym_port(uint8_t data)   // sound_bankswitch_w
{
	s_oki[0].bank = (data & 1) * 0x40000u;
	s_oki[1].bank = ((data >> 1) & 1) * 0x40000u;
}

static uint32_t s_ym_line_clocks;
static void mix_ym_line(void)
{
	const int n = rbx_ym_advance_lr(0, s_ym_line_clocks, s_mix_l + s_mix_n, s_mix_r + s_mix_n, s_mix_cap - s_mix_n);
	for (int i = 0; i < n; i++) {   // the OKIs (mono to both sides) at their own rates, held
		int32_t o = 0;
		for (int k = 0; k < 2; k++) {
			s_oki_phase[k] += s_oki[k].rate;
			while (s_oki_phase[k] >= s_out_rate) {
				s_oki_phase[k] -= s_out_rate;
				s_oki_last[k] = oki_sample(&s_oki[k]);
			}
			o += s_oki_last[k];
		}
		s_mix_l[s_mix_n + i] += o;
		s_mix_r[s_mix_n + i] += o;
	}
	s_mix_n += n;
}

// ---- DECO ACE (MAME deco_ace.cpp) ----
static void ace_palette_update(void)
{
	const int fr = s_ace[0x20] & 0xff, fg = s_ace[0x21] & 0xff, fb = s_ace[0x22] & 0xff;
	const int sr = s_ace[0x23] & 0xff, sg = s_ace[0x24] & 0xff, sb = s_ace[0x25] & 0xff;
	const uint16_t mode = s_ace[0x26];
	for (int i = 0; i < 0x800; i++) {
		int b = (s_palbuf[i] >> 16) & 0xff, g = (s_palbuf[i] >> 8) & 0xff, r = s_palbuf[i] & 0xff;
		s_pens[i + 0x800] = (uint32_t)(r << 16 | g << 8 | b);   // raw
		if (mode == 0x1000) {   // additive fade
			b = b + sb > 0xff ? 0xff : b + sb;
			g = g + sg > 0xff ? 0xff : g + sg;
			r = r + sr > 0xff ? 0xff : r + sr;
		} else {                // multiplicative fade (u8 of the sum, then 0 .. 255)
			b = (uint8_t)(b + ((fb - b) * sb) / 255);
			g = (uint8_t)(g + ((fg - g) * sg) / 255);
			r = (uint8_t)(r + ((fr - r) * sr) / 255);
		}
		s_pens[i] = (uint32_t)(r << 16 | g << 8 | b);   // faded
	}
}
static uint8_t ace_alpha(const uint16_t *ace, uint8_t val)
{
	const int a = ace[val & 0x1f] & 0xff;
	if (a > 0x20)
		return 0x80;   // (TODO in MAME: a special blending command)
	const int r = 255 - (a << 3);
	return (uint8_t)(r < 0 ? 0 : r);
}

// ---- video ----
static uint32_t pri_cb(uint16_t x, int extpri)
{
	(void)x;
	return 0xff00 | (extpri ? 0xf0f0 : 0);   // GFX_PMASK_8 (under the text layer), | GFX_PMASK_4 (the uppermost playfield)
}

static int bank_cb(int bank) { return ((bank & 0x10) | ((bank & 0x40) >> 1) | ((bank & 0x20) << 1)) << 8; }
static int ns_bank_cb(int bank) { return (bank & ~0xf) << 8; }

#define BANDS 4
// lines r0 .. r1 - 1 of the picture (the tilemaps on those lines, the sprites clipped to them)
static void draw_band(int band)
{
	const int r0 = VIS_H * band / BANDS, r1 = VIS_H * (band + 1) / BANDS, pitch = rbx_video.pitch;
	uint16_t *bm = rbx_video.bitmap;
	for (int y = r0; y < r1; y++) {
		for (int x = 0; x < VIS_W; x++)
			bm[y * pitch + x] = 0x300;
		memset(s_pri_buf + y * VIS_W, 0, VIS_W);
	}
	deco16ic_t *t0 = &s_snap->tile[0], *t1 = &s_snap->tile[1];
	deco16ic_draw(t1, 1, s_snap->rowscroll[3], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, r0, r1, 0, 1, 0xff);
	if (s_snap->pri & 1) {
		deco16ic_draw(t0, 1, s_snap->rowscroll[1], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, r0, r1, 0, 2, 0xff);
		deco16ic_draw(t1, 0, s_snap->rowscroll[2], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, r0, r1, 0, 4, 0xff);
	} else {
		deco16ic_draw(t1, 0, s_snap->rowscroll[2], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, r0, r1, 0, 2, 0xff);
		deco16ic_draw(t0, 1, s_snap->rowscroll[1], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, r0, r1, 0, 4, 0xff);
	}
	deco16ic_draw(t0, 0, s_snap->rowscroll[0], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, r0, r1, 0, 8, 0xff);
	decospr_draw(&s_spr, s_snap->spriteram, 0x800, bm + r0 * pitch, s_pri_buf + r0 * pitch, pitch, VIS_W, r1 - r0,
	             VIS_Y0 + r0, 1, s_snap->frame);
}
static decoband_t s_band = { 0, 0, 0, 0, BANDS, draw_band };

// ---- Night Slashers: MAME screen_update_nslasher / mix_nslasher ----
static uint16_t s_ns_spr[2][VIS_W * VIS_H], s_ns_alpha[VIS_W * VIS_H], s_ns_top[VIS_W * VIS_H];
static int s_ns_565;   // (the frontend's out_color is RGB565 by its values: the pixels converted here)

static uint16_t ns_mix(uint16_t p, uint16_t p2) { return (uint16_t)(((p & 0x70f) + (((p & 0x30) | (p2 & 0x0f)) << 4)) & 0x7ff); }

static inline uint32_t blend(uint32_t d, uint32_t s, int level)   // MAME alpha_blend_r32
{
	return ((((s & 0xff00ff) * (uint32_t)level + (d & 0xff00ff) * (uint32_t)(256 - level)) >> 8) & 0xff00ff) |
	       ((((s & 0x00ff00) * (uint32_t)level + (d & 0x00ff00) * (uint32_t)(256 - level)) >> 8) & 0x00ff00);
}

static void ns_band(int band)
{
	const int r0 = VIS_H * band / BANDS, r1 = VIS_H * (band + 1) / BANDS, pitch = VIS_W;
	uint16_t *bm = rbx_video.bitmap;
	const uint16_t *ace = s_snap->ace;
	const uint32_t *pens = s_snap->pens;
	const uint32_t pri = s_snap->pri;
	const int alpha_tm = ace[0x17] != 0 && (pri & 3);
	{
		const size_t o = (size_t)r0 * pitch, n = (size_t)(r1 - r0) * pitch;
		for (size_t i = 0; i < n; i++)
			bm[o + i] = 0x300;
		memset(s_ns_spr[0] + o, 0, n * 2);
		memset(s_ns_spr[1] + o, 0, n * 2);
		if (alpha_tm)
			memset(s_ns_alpha + o, 0, n * 2);
		memset(s_ns_top + o, 0xff, n * 2);
		memset(s_pri_buf + o, 0, n);
	}
	for (int c = 0; c < 2; c++)   // the sprites raw into their bitmaps
		decospr_draw(c ? &s_spr2 : &s_spr, c ? s_snap->spriteram2 : s_snap->spriteram, 0x800, s_ns_spr[c] + r0 * pitch, NULL,
		             pitch, VIS_W, r1 - r0, VIS_Y0 + r0, 1, s_snap->frame);
	deco16ic_t *t0 = &s_snap->tile[0], *t1 = &s_snap->tile[1];
	if (pri & 2) {
		deco16ic_draw_combine(t1, s_snap->rowscroll[2], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, r0, r1, 0, 1, ns_mix);
		deco16ic_draw(t0, 1, s_snap->rowscroll[1], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, r0, r1, 0, 4, 0xff);
	} else {
		deco16ic_draw(t1, 1, s_snap->rowscroll[3], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, r0, r1, 0, 1, 0xff);
		if (pri & 1) {
			deco16ic_draw(t0, 1, s_snap->rowscroll[1], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, r0, r1, 0, 2, 0xff);
			deco16ic_draw(t1, 0, s_snap->rowscroll[2], alpha_tm ? s_ns_alpha : bm, s_pri_buf, pitch, VIS_W, VIS_Y0, r0, r1, 0, 4, 0xff);
		} else {
			deco16ic_draw(t1, 0, s_snap->rowscroll[2], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, r0, r1, 0, 2, 0xff);
			deco16ic_draw(t0, 1, s_snap->rowscroll[1], alpha_tm ? s_ns_alpha : bm, s_pri_buf, pitch, VIS_W, VIS_Y0, r0, r1, 0, 4, 0xff);
		}
	}
	// the top playfield (MAME: after the mixing, priority 0): drawn apart, laid over the mixed line
	deco16ic_draw(t0, 0, s_snap->rowscroll[0], s_ns_top, NULL, pitch, VIS_W, VIS_Y0, r0, r1, 0, 0, 0xff);
	const uint32_t *pal0 = pens + (s_snap->spr_bank[0] << 8), *pal1 = pens + (s_snap->spr_bank[1] << 8), *pal2 = pens;
	for (int y = r0; y < r1; y++) {
		const uint16_t *sp0 = s_ns_spr[0] + y * pitch, *sp1 = s_ns_spr[1] + y * pitch, *al = s_ns_alpha + y * pitch;
		const uint8_t *tp = s_pri_buf + y * VIS_W;
		uint32_t d[VIS_W];
		for (int x = 0; x < VIS_W; x++)
			d[x] = pens[bm[y * pitch + x] & 0xfff];
		const uint16_t *top = s_ns_top + y * pitch;
		for (int x = 0; x < VIS_W; x++) {
			const uint16_t c0p = sp0[x], c1p = sp1[x];
			if (!((c0p | c1p) & 0xff) && !(alpha_tm && (al[x] & 0xf))) {   // (no sprite, no alpha playfield: as drawn)
				if (top[x] != 0xffff)
					d[x] = pens[top[x] & 0xfff];
				continue;
			}
			const int pri0 = (c0p & 0x6000) >> 13, pri1 = (c1p & 0x6000) >> 13;
			const int col0 = (((c0p & 0x1f00) >> 8) & 15) * 32;   // (5 bpp: 16 colours of 32)
			int col1 = (c1p & 0x0f00) >> 8;
			const int alpha1 = (c1p & 0x8000) != 0, alpha2 = !(c1p & 0x1000);
			uint32_t coloffs = (pri & 4) == 0 ? 0x800 : 0;
			int drawn0 = 0;
			if (c0p & 0xff) {
				const int p3 = pri0 & 3;
				if (p3 == 0 || p3 == 1 || (p3 == 2 && alpha_tm) || (p3 == 2 && tp[x] < 4) || (p3 == 3 && tp[x] < 2)) {
					d[x] = pal0[coloffs | (uint32_t)((c0p & 0xff) + col0)];
					drawn0 = 1;
				}
			}
			coloffs = ((pri & 4) == 0 && drawn0) ? 0x800 : 0;
			const int alpha = ((!alpha1) || alpha2) ? ace_alpha(ace, (uint8_t)((col1 & 0x8) ? 0x4 + ((col1 & 0x3) / 2) : ((col1 & 0x7) / 2))) : 0xff;
			col1 = (col1 & 15) * 16;
			if (c1p & 0xff) {
				const uint32_t s1 = pal1[coloffs | (uint32_t)((c1p & 0xff) + col1)];
				const int p0 = pri0 & 3, none0 = (c0p & 0xff) == 0;
				if (alpha1) {
					if (pri1 == 0 && (none0 || (p0 != 0 && p0 != 1 && p0 != 2))) {
						if ((pri & 1) == 0 || tp[x] < 4 || (alpha_tm && (al[x] & 0xf) == 0))
							d[x] = blend(d[x], s1, alpha);
					} else if (pri1 == 1 && ((pri & 1) == 0 || tp[x] < 4) &&
					           (none0 || (p0 != 0 && p0 != 1 && ((pri & 1) == 0 || p0 != 2))))
						d[x] = blend(d[x], s1, alpha);
					else if (pri1 == 2 || pri1 == 3)
						d[x] = blend(d[x], s1, alpha);
				} else {
					if (pri1 == 0 && (none0 || p0 != 0))
						d[x] = blend(d[x], s1, alpha);
					else if (pri1 != 0)
						d[x] = blend(d[x], s1, alpha);
				}
			}
			if (alpha_tm) {
				const uint16_t p = al[x];
				if ((p & 0xf) && ((c0p & 0xff) == 0 || (pri0 & 3) == 2 || (pri0 & 3) == 3) &&
				    ((c1p & 0xff) == 0 || (pri1 & 3) == 2 || (pri1 & 3) == 3 || alpha1))
					d[x] = blend(d[x], pal2[coloffs | p], ace_alpha(ace, (uint8_t)(0x17 + (((p & 0xf0) >> 4) / 2))));
			}
			const uint16_t t = s_ns_top[y * pitch + x];
			if (t != 0xffff)
				d[x] = pens[t & 0xfff];
		}
		// out: the frontend's frame in its colour format (else nothing: the index bitmap stays)
		if (rbx_video.out) {
			uint8_t *o = (uint8_t *)rbx_video.out + (size_t)y * rbx_video.out_pitch;
			if (rbx_video.out_bpp == 4)
				for (int x = 0; x < VIS_W; x++)
					((uint32_t *)o)[x] = rbx_video.out_color((int)(d[x] >> 16) & 0xff, (int)(d[x] >> 8) & 0xff, (int)d[x] & 0xff);
			else if (s_ns_565)
				for (int x = 0; x < VIS_W; x++)
					((uint16_t *)o)[x] = (uint16_t)((d[x] >> 8 & 0xf800) | (d[x] >> 5 & 0x07e0) | (d[x] >> 3 & 0x001f));
			else
				for (int x = 0; x < VIS_W; x++)
					((uint16_t *)o)[x] = (uint16_t)rbx_video.out_color((int)(d[x] >> 16) & 0xff, (int)(d[x] >> 8) & 0xff, (int)d[x] & 0xff);
		}
	}
}
static decoband_t s_ns_bands = { 0, 0, 0, 0, BANDS, ns_band };
static int fh_help(int helper) { return decoband_help(s_ns ? &s_ns_bands : &s_band, helper); }

static void fh_render(void)
{
	if (s_ns) {
		if (rbx_video.out)
			s_ns_565 = rbx_video.out_bpp == 2 && rbx_video.out_color(0xff, 0, 0) == 0xf800 &&
			           rbx_video.out_color(0, 0xff, 0) == 0x07e0 && rbx_video.out_color(0, 0, 0xff) == 0x001f &&
			           rbx_video.out_color(0x84, 0x82, 0x21) == 0x8404;
		decoband_run(&s_ns_bands);
		rbx_video.out_done = rbx_video.out != NULL;
		return;
	}
	memcpy(rbx_video.palette, s_snap->pal, sizeof(s_snap->pal));
	decoband_run(&s_band);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->tile, s_tile, sizeof(s_tile));
	for (int i = 0; i < 4; i++)
		memcpy(s_snap->rowscroll[i], s_rowscroll[i], sizeof(s_snap->rowscroll[i]));
	memcpy(s_snap->spriteram, s_spriteram_buf, sizeof(s_spriteram_buf));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	s_snap->pri = s_pri;
	s_snap->frame = s_frame;
	if (s_ns) {
		memcpy(s_snap->spriteram2, s_spriteram2_buf, sizeof(s_spriteram2_buf));
		memcpy(s_snap->ace, s_ace, sizeof(s_ace));
		if (s_pens_dirty) {
			s_pens_dirty = 0;
			ace_palette_update();
		}
		memcpy(s_snap->pens, s_pens, sizeof(s_pens));
		memcpy(s_snap->spr_bank, s_spr_bank, sizeof(s_spr_bank));
	}
	rbx_video_frame_done();
}

// ---- init / frame ----
static void fh_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_palram, 0, sizeof(s_palram));
	memset(s_pal_dirty, 0, sizeof(s_pal_dirty));
	memset(s_prot_lo, 0, sizeof(s_prot_lo));
	s_latch = s_latch_pending = 0;
	s_volume = 0;
	memset(s_snd_ram, 0, sizeof(s_snd_ram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_spriteram_buf, 0, sizeof(s_spriteram_buf));
	memset(s_rowscroll32, 0, sizeof(s_rowscroll32));
	memset(s_rowscroll, 0, sizeof(s_rowscroll));
	s_pri = 0;
	for (int i = 0; i < 2; i++) {
		memset(s_tile[i].vram, 0, sizeof(s_tile[i].vram));
		deco16ic_reset(&s_tile[i]);
		oki_reset(&s_oki[i]);
		s_oki[i].bank = 0;
		s_oki_phase[i] = 0;
		s_oki_last[i] = 0;
	}
	deco146_reset(&s_prot);
	armpoll_reset(&s_poll);
	rbx_ym_reset(0);
	volume_apply();
	arm_reset(&s_cpu);
	h6280_reset(&s_snd);
	{
		z80 t = s_z80;
		z80_init(&s_z80);
		s_z80.read_byte = t.read_byte;
		s_z80.write_byte = t.write_byte;
		s_z80.port_in = t.port_in;
		s_z80.int_data = 0xff;
	}
	memset(s_z80_ram, 0, sizeof(s_z80_ram));
	s_zirq_latch = s_zirq_ym = s_zirq = 0;
	memset(s_ace, 0, sizeof(s_ace));
	memset(s_palbuf, 0, sizeof(s_palbuf));
	memset(s_pens, 0, sizeof(s_pens));
	s_pens_dirty = 0;
	memset(s_spriteram2, 0, sizeof(s_spriteram2));
	memset(s_spriteram2_buf, 0, sizeof(s_spriteram2_buf));
	s_spr_bank[0] = s_spr_bank[1] = 0;
	memset(&s_tte, 0, sizeof(s_tte));
	memset(s_tt_ram, 0, sizeof(s_tt_ram));
	if (s_tt) {
		rbx_aux_sync();
		s_bsmt_frac = 0;
		decobsmt_reset(s_bsmt);
		s_tt_ev_n[0] = s_tt_ev_n[1] = 0;
		s_tt_ev_cur = 0;
		s_tt_rst_last = 0;
		s_tt_out_n = 0;
		s_tt_job_frame = 0;
	}
	if (s_ns) {   // (the colour banks as the machine sets them)
		s_tile[1].tm[0].col_bank = 0x20;
		s_tile[1].tm[1].col_bank = 0x30;
	}
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = s_snd_frac = s_ym_frac = 0;
	s_mix_n = 0;
}

static void fh_exit(void);

static int fh_init(const rbx_game *g)
{
	uint32_t t1s, t2s, ss, o1s, o2s;
	s_old = !strcmp(g->machine, "fghthisto");
	s_tt = !strcmp(g->machine, "tattass");
	s_ns = !strncmp(g->machine, "nslasher", 8) || s_tt;
	s_zsnd = !strcmp(g->machine, "fghthistu") || !strcmp(g->machine, "nslasher");
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_snd_rom = rbx_region("audiocpu", &s_snd_rom_size);
	uint8_t *t1 = rbx_region("tiles1", &t1s), *t2 = rbx_region("tiles2", &t2s), *spr = rbx_region("sprites1", &ss);
	uint8_t *oki1 = rbx_region("oki1", &o1s), *oki2 = rbx_region("oki2", &o2s);
	if (s_tt) {   // (the BSMT board: its 6809's ROM, the BSMT2000's mask ROM, the samples; the EEPROM's contents)
		uint32_t bs, ps, es;
		s_snd_rom = rbx_region("decobsmt:soundcpu", &s_snd_rom_size);
		uint8_t *smp = rbx_region("decobsmt:bsmt", &bs), *prog = rbx_region("decobsmt:bsmt:bsmt2000", &ps);
		const uint8_t *eep = rbx_region("eeprom", &es);
		if (!s_snd_rom || s_snd_rom_size < 0x10000 || !smp || !prog || ps < 0x2000) {
			rbx_fail("%s: ROM regions missing", g->set);
			return -1;
		}
		s_bsmt = (decobsmt_t *)calloc(1, sizeof(*s_bsmt));
		if (!s_bsmt) {
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
		decobsmt_init(s_bsmt, s_snd_rom, prog, smp, bs);
		memset(s_tt_eep, 0xff, sizeof(s_tt_eep));
		if (eep)
			memcpy(s_tt_eep, eep, es < sizeof(s_tt_eep) ? es : sizeof(s_tt_eep));
		oki1 = oki2 = s_tt_eep;   // (no OKIs: these only stand in)
		o1s = o2s = 0x400;
	}
	if (!s_rom || !s_snd_rom || !t1 || !t2 || !spr || !oki1 || !oki2) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	uint32_t s2s = 0;
	uint8_t *spr2 = s_ns ? rbx_region("sprites2", &s2s) : NULL;
	if (s_ns) {
		if (!spr2 || t1s < 0x180000 || t2s < 0x180000) {
			rbx_fail("%s: ROM regions missing", g->set);
			return -1;
		}
		if (!s_tt)
			deco156_decrypt(s_rom, s_rom_size);
		for (int r = 0; r < 2; r++) {   // reorder_tilegfx: 0x80000 .. 0xfffff and 0x100000 .. 0x17ffff swapped
			uint8_t *m = r ? t2 : t1;
			for (uint32_t i = 0; i < 0x80000; i++) {
				const uint8_t t = m[0x80000 + i];
				m[0x80000 + i] = m[0x100000 + i];
				m[0x100000 + i] = t;
			}
		}
	}
	if (deco56_decrypt_gfx(t1, t1s) || (s_tt ? deco56_decrypt_gfx(t2, t2s) : deco74_decrypt_gfx(t2, t2s))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	{	// charlayout / tilelayout (RGN_FRAC(1,2)), tilelayout_8bpp (RGN_FRAC(1,4), 8 planes)
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int k = 0; k < 8; k++) {
			x8[k] = (uint32_t)k;
			y8[k] = (uint32_t)(k * 16);
			x16[k] = (uint32_t)(256 + k);
			x16[8 + k] = (uint32_t)k;
		}
		for (int k = 0; k < 16; k++)
			y16[k] = (uint32_t)(k * 16);
		const uint32_t h1 = t1s * 4, h2 = t2s * 4, hs = ss * 4;   // (half of a region, in bits)
		const uint32_t p1[4] = { h1 + 8, h1, 8, 0 }, p2[4] = { h2 + 8, h2, 8, 0 }, ps[4] = { hs + 8, hs, 8, 0 };
		const rbx_gfx_layout l8 = { 8, 8, 4, p1, x8, y8, 16 * 8 }, l16 = { 16, 16, 4, p1, x16, y16, 64 * 8 };
		const rbx_gfx_layout l16b = { 16, 16, 4, p2, x16, y16, 64 * 8 }, lsp = { 16, 16, 4, ps, x16, y16, 64 * 8 };
		int bad = rbx_gfx_decode(&s_gfx8, &l8, t1, t1s / 2) || rbx_gfx_decode(&s_gfx16, &l16, t1, t1s / 2) ||
		          rbx_gfx_decode(&s_gfx16b, &l16b, t2, t2s / 2);
		if (!s_ns)
			bad = bad || rbx_gfx_decode(&s_gfxspr, &lsp, spr, ss / 2);
		else {   // tilelayout_5bpp (RGN_FRAC(1,1)), sprites2: tilelayout
			static uint32_t x5[16], y5[16];
			for (int k = 0; k < 8; k++) {
				x5[k] = (uint32_t)(16 * 8 * 5 + k);
				x5[8 + k] = (uint32_t)k;
			}
			for (int k = 0; k < 16; k++)
				y5[k] = (uint32_t)(k * 8 * 5);
			static const uint32_t p5[5] = { 0, 8, 16, 24, 32 };
			const uint32_t h2s = s2s * 4;
			const uint32_t pq[4] = { h2s + 8, h2s, 8, 0 };
			const rbx_gfx_layout l5 = { 16, 16, 5, p5, x5, y5, 16 * 16 * 5 }, lq = { 16, 16, 4, pq, x16, y16, 64 * 8 };
			bad = bad || rbx_gfx_decode(&s_gfxspr, &l5, spr, ss) || rbx_gfx_decode(&s_gfxspr2, &lq, spr2, s2s / 2);
		}
		if (bad) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			fh_exit();
			return -1;
		}
	}
	memset(s_tile, 0, sizeof(s_tile));
	for (int c = 0; c < 2; c++)
		for (int i = 0; i < 2; i++) {
			s_tile[c].tm[i].size = DECO_64x32;
			s_tile[c].tm[i].col_bank = 0x10 * (2 * c + i);   // 0x00 / 0x10, 0x20 / 0x30
			s_tile[c].tm[i].col_mask = 0x0f;
			s_tile[c].tm[i].bank_cb = s_ns ? ns_bank_cb : bank_cb;
		}
	if (s_ns)   // (gfx_nslasher: the 8x8 characters from pen 0x800, the raw palette)
		s_tile[0].base8 = s_tile[1].base8 = 0x800;
	s_tile[0].gfx8 = s_tile[1].gfx8 = &s_gfx8;   // (8x8: entry 0)
	s_tile[0].gfx16 = &s_gfx16;                  // entry 1
	s_tile[1].gfx16 = &s_gfx16b;                 // entry 2
	memset(&s_spr, 0, sizeof(s_spr));
	s_spr.gfx = &s_gfxspr;
	s_spr.color_base = 1024;
	s_spr.pri_cb = pri_cb;
	s_spr.flip_screen = 1;   // (sprites are flipped relative to the tilemaps)
	if (s_ns) {   // the two chips drawn raw (set_pix_raw_shift(8)) for the mixing
		s_spr.pri_cb = NULL;
		s_spr.color_base = 0;
		s_spr.raw = 1;
		s_spr.raw_shift = 8;
		s_spr2 = s_spr;
		s_spr2.gfx = &s_gfxspr2;
	}
	deco146_init(&s_prot, s_ns);
	static const uint8_t interleave[10] = { 4, 5, 3, 6, 2, 7, 1, 8, 0, 9 };   // set_interface_scramble_interleave
	deco146_set_scramble(&s_prot, interleave);
	s_prot.magic_read_xor_on = !s_ns;
	s_prot.port_a = port_a;
	s_prot.port_b = port_b;
	s_prot.port_c = port_c;
	if (!s_old)
		s_prot.soundlatch_irq = latch_irq;
	s_nv_path[0] = 0;
	eeprom_init(&s_eeprom);
	{
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			if (s_tt) {   // (the 1K EEPROM as it is)
				FILE *f = fopen(s_nv_path, "rb");
				if (f) {
					uint8_t t[0x400];
					if (fread(t, 1, sizeof(t), f) == sizeof(t))
						memcpy(s_tt_eep, t, sizeof(t));
					fclose(f);
				}
			} else
				eeprom_load(&s_eeprom, s_nv_path);
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	// sound: YM2151 0.42 / 0.42, OKI 1 MHz 1.0, OKI 2 MHz 0.35 (both sides)
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = (s_zsnd || s_ns) ? 102 : 108;   // (fghthistu, nslasher: 0.40)
	yc.irq = ym_irq;
	yc.port_a_write = ym_port;
	oki_init(&s_oki[0], 32220000 / 32, 1, oki1, o1s, s_ns ? 205 : 256);   // (nslasher: 0.80, 0.10)
	oki_init(&s_oki[1], 32220000 / 16, 1, oki2, o2s, s_ns ? 26 : 90);
	// (the YM2151's samples made on the worker: rbx_ym_async_take_lr at the frame's end)
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 2)) {
		rbx_fail("%s: out of memory", g->set);
		fh_exit();
		return -1;
	}
	s_out_rate = s_tt ? DECOBSMT_RATE : rbx_ym_rate(0);
	s_mix_cap = (int)(s_out_rate / 50) + 256;
	s_mix_l = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_mix_r = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (s_tt) {
		s_tt_out_l = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		s_tt_out_r = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		if (!s_tt_out_l || !s_tt_out_r) {
			fh_exit();
			return -1;
		}
		rbx_aux_set_job(tt_job);
	}
	if (!s_mix_l || !s_mix_r || rbx_audio_setup_stereo(s_out_rate) || rbx_video_setup(VIS_W, VIS_H, 0x800)) {
		fh_exit();
		return -1;
	}
	s_port_in0 = rbx_port_find("IN0");
	s_port_in1 = rbx_port_find("IN1");
	arm_init(&s_cpu);
	for (uint32_t a = 0; a < 0x100000 && a < s_rom_size; a += 0x1000)
		s_cpu.rpage[a >> 12] = s_rom + a;
	for (uint32_t a = 0; a < 0x20000; a += 0x1000)
		s_cpu.rpage[(0x100000 + a) >> 12] = s_cpu.wpage[(0x100000 + a) >> 12] = s_ram + a;
	if (!s_tt)
		s_cpu.rpage[(s_ns ? 0x100000 : 0x101000) >> 12] = NULL;   // (the page with the polled location: bus_r32)
	s_cpu.read32 = bus_r32;
	s_cpu.write32 = bus_w32;
	s_cpu.read8 = bus_r8;
	s_cpu.write8 = bus_w8;
	z80_init(&s_z80);
	s_z80.read_byte = z_rd;
	s_z80.write_byte = z_wr;
	s_z80.port_in = z_in;
	s_z80.int_data = 0xff;
	h6280_init(&s_snd);
	for (uint32_t a = 0; a < 0x10000 && a < s_snd_rom_size; a += 0x2000)
		s_snd.rpage[a >> 13] = s_snd_rom + a;
	s_snd.rpage[0x1f0000 >> 13] = s_snd.wpage[0x1f0000 >> 13] = s_snd_ram;
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	fh_reset();
	return 0;
}

static void fh_exit(void)
{
	if (s_tt && s_nv_path[0] && s_tt_eep_dirty) {
		FILE *f = fopen(s_nv_path, "wb");
		if (f) {
			fwrite(s_tt_eep, 1, sizeof(s_tt_eep), f);
			fclose(f);
		}
		s_tt_eep_dirty = 0;
	} else if (s_nv_path[0] && s_eeprom.dirty)
		eeprom_save(&s_eeprom, s_nv_path);
	if (s_tt)
		rbx_aux_set_job(NULL);
	free(s_bsmt);
	s_bsmt = NULL;
	free(s_tt_out_l);
	free(s_tt_out_r);
	s_tt_out_l = s_tt_out_r = NULL;
	s_nv_path[0] = 0;
	rbx_ym_exit(0);
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16);
	rbx_gfx_free(&s_gfx16b);
	rbx_gfx_free(&s_gfxspr);
	rbx_gfx_free(&s_gfxspr2);
	free(s_snap);
	s_snap = NULL;
	free(s_mix_l);
	free(s_mix_r);
	s_mix_l = s_mix_r = NULL;
}

static void fh_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (s_ns && (line == 0 || line == VIS_Y0 || line == VIS_Y0 + VIS_H))
			rbx_port_custom(s_port_in1, 0x10, (line < VIS_Y0 || line >= VIS_Y0 + VIS_H) ? 0x10 : 0);
		if (line == VIS_Y0 + VIS_H) {
			vblank_capture();
			arm_set_irq(&s_cpu, 1);
		}
		if (line >= VIS_Y0 && line < VIS_Y0 + VIS_H)
			for (int i = 0; i < 2; i++)
				deco16ic_latch_line(&s_tile[i], line);
		s_poll.slice_end = s_cpu.cycles + rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		arm_run(&s_cpu, s_poll.slice_end);
		arm_break = 0;
		if (s_tt)   // (the BSMT board: on the worker, below)
			continue;
		if (s_zsnd) {   // (Z80 at 32.22 MHz / 9)
			z80_run(&s_z80, s_z80.cyc + (unsigned long)rbx_line_cycles(&s_snd_frac, YM_CLOCK, FRAME_MHZ, LINES), &s_zirq, 0);
			z80_break = 0;
		} else {
			h6280_run_until(&s_snd, s_snd.cycles + rbx_line_cycles(&s_snd_frac, SND_CLOCK, FRAME_MHZ, LINES));
			h6280_break = 0;
		}
		s_ym_line_clocks = (uint32_t)rbx_line_cycles(&s_ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
		mix_ym_line();
	}
	if (s_tt) {   // the previous frame's samples out, this frame's writes to the board
		rbx_aux_sync();
		memcpy(s_mix_l, s_tt_out_l, sizeof(int32_t) * (size_t)s_tt_out_n);
		memcpy(s_mix_r, s_tt_out_r, sizeof(int32_t) * (size_t)s_tt_out_n);
		s_mix_n = s_tt_out_n;
		s_tt_out_n = 0;
		s_tt_ev_cur ^= 1;
		__atomic_store_n(&s_tt_job_frame, 1, __ATOMIC_RELEASE);
		rbx_aux_kick();
	} else
		rbx_ym_async_take_lr(0, s_mix_l, s_mix_r, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put_stereo(s_mix_l[i], s_mix_r[i]);
	memset(s_mix_l, 0, sizeof(int32_t) * (size_t)s_mix_n);
	memset(s_mix_r, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	s_frame++;
}

static int fh_state(void *ctx, int load)
{
	uint8_t cpu[ARM_STATE_SIZE];
	uint8_t snd[H6280_STATE_SIZE];
	if (!load) {
		memcpy(cpu, &s_cpu, ARM_STATE_SIZE);
		memcpy(snd, &s_snd, H6280_STATE_SIZE);
	}
	uint64_t fr[3] = { s_cpu_frac, s_snd_frac, s_ym_frac };
	int32_t misc[10] = { (int32_t)s_frame, (int32_t)s_pri, s_latch | s_latch_pending << 8, s_volume, (int32_t)s_oki[0].bank,
	                     (int32_t)s_oki[1].bank, (int32_t)s_oki_phase[0], (int32_t)s_oki_phase[1], s_oki_last[0], s_oki_last[1] };
	int rc = 0;
	rc |= rbx_state_io(ctx, "arm", cpu, ARM_STATE_SIZE);
	rc |= rbx_state_io(ctx, "h6280", snd, H6280_STATE_SIZE);
	z80 zs = s_z80;
	int32_t zi[3] = { s_zirq_latch, s_zirq_ym, s_zirq };
	rc |= rbx_state_io(ctx, "z80", &zs, sizeof(zs));
	rc |= rbx_state_io(ctx, "zram", s_z80_ram, sizeof(s_z80_ram));
	rc |= rbx_state_io(ctx, "zirq", zi, sizeof(zi));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "palram", s_palram, sizeof(s_palram));
	rc |= rbx_state_io(ctx, "paldirty", s_pal_dirty, sizeof(s_pal_dirty));
	rc |= rbx_state_io(ctx, "protlo", s_prot_lo, sizeof(s_prot_lo));
	rc |= rbx_state_io(ctx, "eeprom", &s_eeprom, sizeof(s_eeprom));
	rc |= rbx_state_io(ctx, "sram", s_snd_ram, sizeof(s_snd_ram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "sprb", s_spriteram_buf, sizeof(s_spriteram_buf));
	rc |= rbx_state_io(ctx, "rows32", s_rowscroll32, sizeof(s_rowscroll32));
	rc |= rbx_state_io(ctx, "rows", s_rowscroll, sizeof(s_rowscroll));
	if (s_ns) {
		uint8_t nb[2 + 6] = { s_spr_bank[0], s_spr_bank[1], (uint8_t)s_tile[1].tm[0].col_bank, (uint8_t)s_tile[1].tm[1].col_bank };
		rc |= rbx_state_io(ctx, "ace", s_ace, sizeof(s_ace));
		rc |= rbx_state_io(ctx, "palbuf", s_palbuf, sizeof(s_palbuf));
		rc |= rbx_state_io(ctx, "spr2", s_spriteram2, sizeof(s_spriteram2));
		rc |= rbx_state_io(ctx, "spr2b", s_spriteram2_buf, sizeof(s_spriteram2_buf));
		rc |= rbx_state_io(ctx, "nsbank", nb, sizeof(nb));
		if (load && !rc) {
			s_spr_bank[0] = nb[0];
			s_spr_bank[1] = nb[1];
			s_tile[1].tm[0].col_bank = nb[2];
			s_tile[1].tm[1].col_bank = nb[3];
			ace_palette_update();
		}
	}
	for (int i = 0; i < 2; i++) {
		rc |= rbx_state_io(ctx, i ? "vram2" : "vram1", s_tile[i].vram, sizeof(s_tile[i].vram));
		rc |= rbx_state_io(ctx, i ? "tctl2" : "tctl1", s_tile[i].control, sizeof(s_tile[i].control));
	}
	rc |= rbx_state_io(ctx, "prot", &s_prot, DECO146_STATE_SIZE);
	if (s_tt) {
		int32_t tn[2] = { s_tt_out_n, s_tt_rst_last };
		rbx_aux_sync();   // (the job done: the board at the frame's end, its samples waiting)
		rc |= rbx_state_io(ctx, "ttoutn", tn, sizeof(tn));
		if (load && !rc) {
			s_tt_out_n = tn[0] < s_mix_cap ? tn[0] : 0;
			s_tt_rst_last = tn[1];
		}
		rc |= rbx_state_io(ctx, "ttoutl", s_tt_out_l, sizeof(int32_t) * (size_t)s_mix_cap);
		rc |= rbx_state_io(ctx, "ttoutr", s_tt_out_r, sizeof(int32_t) * (size_t)s_mix_cap);
		rc |= decobsmt_state(s_bsmt, ctx, load);
		rc |= rbx_state_io(ctx, "tteep", s_tt_eep, sizeof(s_tt_eep));
		rc |= rbx_state_io(ctx, "ttes", &s_tte, sizeof(s_tte));
		rc |= rbx_state_io(ctx, "ttram", s_tt_ram, sizeof(s_tt_ram));
		rc |= rbx_state_io(ctx, "ttfrac", &s_bsmt_frac, sizeof(s_bsmt_frac));
	}
	rc |= rbx_state_io(ctx, "oki1", s_oki[0].v, sizeof(s_oki[0].v) + sizeof(int));
	rc |= rbx_state_io(ctx, "oki2", s_oki[1].v, sizeof(s_oki[1].v) + sizeof(int));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (rc)
		return -1;
	if (load) {
		memcpy(&s_cpu, cpu, ARM_STATE_SIZE);
		memcpy(&s_snd, snd, H6280_STATE_SIZE);
		zs.read_byte = s_z80.read_byte;
		zs.write_byte = s_z80.write_byte;
		zs.port_in = s_z80.port_in;
		zs.port_out = s_z80.port_out;
		zs.userdata = s_z80.userdata;
		zs.read_map = s_z80.read_map;
		s_z80 = zs;
		s_zirq_latch = zi[0];
		s_zirq_ym = zi[1];
		s_zirq = zi[2];
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_ym_frac = fr[2];
		s_frame = (uint32_t)misc[0];
		s_pri = (uint32_t)misc[1];
		s_latch = (uint8_t)misc[2];
		s_latch_pending = (uint8_t)(misc[2] >> 8);
		s_volume = (uint8_t)misc[3];
		volume_apply();
		s_oki[0].bank = (uint32_t)misc[4];
		s_oki[1].bank = (uint32_t)misc[5];
		s_oki_phase[0] = (uint32_t)misc[6];
		s_oki_phase[1] = (uint32_t)misc[7];
		s_oki_last[0] = misc[8];
		s_oki_last[1] = misc[9];
		for (int c = 0; c < 2; c++)
			for (int i = 0; i < 8; i++)
				deco16ic_control_w(&s_tile[c], i, s_tile[c].control[i], 0xffff);
	}
	return 0;
}

static int fh_save(void *ctx) { return fh_state(ctx, 0); }
static int fh_load(void *ctx) { return fh_state(ctx, 1); }
static int fh_supported(const char *machine)
{
	return !strcmp(machine, "fghthist") || !strcmp(machine, "fghthisto") || !strcmp(machine, "fghthistu") ||
	       !strcmp(machine, "nslasher") || !strcmp(machine, "nslasheru") || !strcmp(machine, "tattass");
}

const rbx_driver rbx_drv_fghthist = { "DECO 32 Fighter's History", fh_init, fh_exit, fh_reset, fh_frame, fh_save, fh_load,
                                      fh_supported, fh_render, fh_help };
