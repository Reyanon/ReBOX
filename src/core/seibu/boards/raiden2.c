/*  Seibu Raiden II hardware (the V30 boards: Raiden II, Raiden DX, Zero Team, X Se Dae Quiz) and its V33 successors
    (Raiden II New / Raiden DX, New Zero Team, Zero Team 2000). A V30 (V33A) with the Seibu COP (its DMAs fill the
    private tilemap and palette buffers), four tile layers through the Seibu CRTC, the SEI252 sprites (encrypted ROMs,
    r2crypt) with the sprite "protection" helper, the Seibu Sound System (Z80, YM2151 or YM3812, one or two OKI
    MSM6295) or, on the Raiden II New board, one banked OKI on the main CPU and a 93C46 EEPROM.
    Ported from MAME 0.289 src/mame/seibu/raiden2.cpp, raiden2.h, raiden2_v.cpp (LGPL-2.1+, copyright-holders:
    Olivier Galibert, Angelo Salese, David Haywood, Tomasz Slanina), src/mame/seibu/r2dx_v33.cpp (BSD-3-Clause,
    copyright-holders: Olivier Galibert, David Haywood), src/mame/seibu/sei25x_rise1x_spr.cpp (BSD-3-Clause,
    copyright-holders: Olivier Galibert, Angelo Salese, David Haywood, Tomasz Slanina, Nicola Salmoria, Ville Linde,
    hap), src/mame/seibu/seibu_crtc.cpp (BSD-3-Clause, copyright-holders: Angelo Salese) and the buffered sprite RAM
    of src/devices/video/bufsprite.h (BSD-3-Clause, copyright-holders: Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.

    The CPUs interleaved by sched.h (MAME's default 60 Hz quantum). The screen is drawn whole at vblank from a copy
    (no raster effects); the blended colours of raiden2_state are mixed in RGB, so the driver writes the frontend's
    frame itself (rbx_video.out). Not done: the flip screen, the coin counters, the sprite key uploads' effect (MAME
    keeps them only), the V33 board's OKI sound timed within the frame (its samples are made at the frame's end).
*/
#include "../../cheat_cpu.h"
#include "../../rbx.h"
#include "../../cpu/nec/nec.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/okim6295.h"
#include "../../machine/eepromser.h"
#include "../../video/decoband.h"
#include "../../video/gfxlayout.h"
#include "../machine/r2crypt.h"
#include "../machine/sched.h"
#include "../machine/seibucop.h"
#include "../machine/seibusnd.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define VIS_W 320
#define CPU_CLOCK 16000000            // 32 MHz / 2
#define SND_CLOCK 3579545             // 28.636363 MHz / 8
#define OKI_CLOCK 1022727             // 28.636363 MHz / 28

enum { M_RAIDEN2, M_RAIDENDX, M_ZEROTEAM, M_XSEDAE, M_RDX_V33, M_NZEROTEA, M_ZEROTM2K };

static int s_m, s_vis_h, s_v33;
static nec_t s_cpu;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static const uint8_t *s_math;
static uint32_t s_math_size;
static rbx_gfx s_chars, s_tiles, s_spr;
static int s_port_dsw, s_port_p12, s_port_p34, s_port_sys, s_port_input;
static uint8_t s_blend[0x800];       // (m_blend_active)
static const int8_t *s_spri;         // (m_cur_spri; NULL: xsedae's priority drawing)
static uint32_t s_spr_base;          // (the sprite RAM in the main RAM)
static eepser_t s_eeprom;
static int s_has_eeprom;
static okim6295_t s_oki;             // (the V33 Raiden II New board's own OKI)
static const uint8_t *s_oki_rom;
static uint32_t s_oki_size;

typedef struct {
	uint8_t ram[0x20000];            // 0x00000-0x1ffff (the I/O at 0x400-0x7ff apart)
	uint16_t back[0x400], fore[0x400], mid[0x400], text[0x800], pal[0x800];   // (the private buffers)
	uint16_t sprbuf[0x800];          // (the buffered sprite RAM)
	uint16_t crtc[0x40];             // (the CRTC's register space)
	uint16_t scroll[6], tilemap_enable;
	uint32_t bg_bank, fg_bank, mid_bank, tx_bank;
	uint16_t cop_bank;
	uint8_t prg_bank, pad0;
	int32_t mainbank;
	uint16_t sprite_prot_x, sprite_prot_y, dst1, cop_spr_maxx, cop_spr_off, sprite_prot_src_addr[2];
	uint16_t sprcpt_flags2;
	uint32_t sprcpt_adr, sprcpt_idx, sprcpt_val[2], sprcpt_flags1;
	uint32_t sprcpt_data_1[0x100], sprcpt_data_2[0x40], sprcpt_data_3[6], sprcpt_data_4[4];
	// the V33 boards
	uint16_t dx, dy, angle, mcu_prog_offs;
	uint32_t sdist;
	uint16_t mcu_prog[0x800];
	uint8_t r2dxbank, r2dxgameselect, okibank, pad1;
	uint64_t oki_frac;
	uint8_t state_end;
} board_t;
static board_t *s_bp;   // (calloc at the init: ADR-040)
#define b (*s_bp)

typedef struct {
	uint16_t back[0x400], fore[0x400], mid[0x400], text[0x800], spr[0x800];
	uint32_t pal[0x800];
	uint16_t scroll[6], enable;
	uint32_t bg_bank, fg_bank, mid_bank, tx_bank;
} snap_t;
static snap_t *s_snapp;   // (calloc at the init: ADR-040)
#define s_snap (*s_snapp)
static uint16_t *s_sprbmp;           // (m_sprite_bitmap; xsedae: the frame's pens)
static uint8_t *s_pribmp;            // (xsedae: screen.priority())

// ---- the banks ----
static void map_banks(void)
{
	switch (s_m) {
	case M_RAIDEN2: case M_ZEROTEAM:   // (two banks of 0x20000 from the ROM's start)
		nec_map(&s_cpu, 0x20000, 0x40000, s_rom + (uint32_t)(b.mainbank & 1) * 0x20000, NULL);
		break;
	case M_RAIDENDX:   // (16 banks of 0x10000 from 0x100000)
		nec_map(&s_cpu, 0x20000, 0x30000, s_rom + 0x100000 + (uint32_t)(b.mainbank & 15) * 0x10000, NULL);
		break;
	case M_RDX_V33: {   // r2dx_setbanking: the game's half of the ROM, its bank
		const uint32_t gs = (uint32_t)(b.r2dxgameselect & 1) * 0x200000;
		nec_map(&s_cpu, 0x20000, 0x30000, s_rom + 0x100000 + gs + (uint32_t)(b.r2dxbank & 15) * 0x10000, NULL);
		nec_map(&s_cpu, 0x30000, 0x100000, s_rom + gs + 0x30000, NULL);
		s_oki.rom = s_oki_rom + (uint32_t)(b.okibank & 3) * 0x40000;
		s_oki.rom_size = 0x40000;
		break;
	}
	default: break;
	}
}

static void bank_reset(int bg, int fg, int mid, int tx)
{
	b.bg_bank = (uint32_t)bg;
	b.fg_bank = (uint32_t)fg;
	b.mid_bank = (uint32_t)mid;
	b.tx_bank = (uint32_t)tx;
}

// ---- the main CPU's bus (any address; the COP's and the sprite helper's accesses too) ----
static uint8_t io_r8(uint32_t a);
static void io_w(uint32_t a, uint16_t d, uint16_t mask);

static int is_io(uint32_t a)
{
	if (a < 0x400 || a >= 0x800)
		return 0;
	return !(s_m == M_RAIDENDX && a >= 0x4d0 && a < 0x4d8);   // (raidendx: RAM)
}

static uint8_t cpu_r8(uint32_t a)
{
	a &= 0xfffff;
	if (a < 0x1000)
		return is_io(a) ? io_r8(a) : b.ram[a];
	return 0;   // (unmapped)
}

static uint16_t cpu_r16(uint32_t a) { return (uint16_t)(cpu_r8(a) | cpu_r8(a + 1) << 8); }

// the handler reads without side effects whose values only the CPU changes (the RAM beside the I/O, the unmapped
// space): the polling loops' probe (nec_idle_enable)
static int cpu_pure(uint32_t a)
{
	a &= 0xfffff;
	return a >= 0x1000 || !is_io(a);
}

static void cpu_w8(uint32_t a, uint8_t v)
{
	a &= 0xfffff;
	if (a >= 0x1000)
		return;   // (the ROM, unmapped)
	if (is_io(a))
		io_w(a & ~1u, (a & 1) ? (uint16_t)(v << 8) : v, (a & 1) ? 0xff00 : 0x00ff);
	else
		b.ram[a] = v;
}

static void cpu_w16(uint32_t a, uint16_t v)
{
	a &= 0xfffff;
	if (a < 0x1000 && is_io(a)) {
		io_w(a, v, 0xffff);
		return;
	}
	cpu_w8(a, (uint8_t)v);
	cpu_w8(a + 1, (uint8_t)(v >> 8));
}

static uint8_t host_r8(uint32_t a)
{
	a &= 0xfffff;
	const uint8_t *p = s_cpu.rmap[a >> 12];
	return p ? p[a & 0xfff] : cpu_r8(a);
}
static void host_w8(uint32_t a, uint8_t v)
{
	a &= 0xfffff;
	uint8_t *p = s_cpu.wmap[a >> 12];
	if (p)
		p[a & 0xfff] = v;
	else
		cpu_w8(a, v);
}
static uint16_t host_r16(uint32_t a)
{
	a &= 0xfffff;
	if (!(a & 1) && a >= 0x1000) {
		const uint8_t *p = s_cpu.rmap[a >> 12];
		if (p)
			return (uint16_t)(p[a & 0xfff] | p[(a & 0xfff) + 1] << 8);
	}
	if (!(a & 1) && a < 0x1000 && is_io(a))
		return (uint16_t)(io_r8(a) | io_r8(a + 1) << 8);
	return (uint16_t)(host_r8(a) | host_r8(a + 1) << 8);
}
static void host_w16(uint32_t a, uint16_t v)
{
	a &= 0xfffff;
	if (!(a & 1) && a < 0x1000 && is_io(a)) {
		io_w(a, v, 0xffff);
		return;
	}
	host_w8(a, (uint8_t)v);
	host_w8(a + 1, (uint8_t)(v >> 8));
}
static uint32_t host_r32(uint32_t a) { return host_r16(a) | (uint32_t)host_r16(a + 2) << 16; }

// ---- the COP's outputs ----
static void videoram_w(int offset, uint16_t data)   // m_videoram_private_w
{
	if (offset < 0x800 / 2)
		b.back[offset] = data;
	else if (offset < 0x1000 / 2)
		b.fore[offset - 0x800 / 2] = data;
	else if (offset < 0x1800 / 2)
		b.mid[offset - 0x1000 / 2] = data;
	else if (offset < 0x2800 / 2)
		b.text[offset - 0x1800 / 2] = data;
}
static void palette_w(int offset, uint16_t data) { b.pal[offset & 0x7ff] = data; }
static uint64_t host_cycles(void) { return (uint64_t)nec_now(&s_cpu); }

// ---- the sprite "protection" (sprite_prot_*) ----
static void sprite_prot_src_w(uint16_t data)
{
	b.sprite_prot_src_addr[1] = data;
	const uint32_t src = ((uint32_t)b.sprite_prot_src_addr[0] << 4) + b.sprite_prot_src_addr[1];
	const int x = (int16_t)((host_r32(src + 0x08) >> 16) - b.sprite_prot_x);
	const int y = (int16_t)((host_r32(src + 0x04) >> 16) - b.sprite_prot_y);
	const uint16_t head1 = host_r16(src + b.cop_spr_off), head2 = host_r16(src + b.cop_spr_off + 2);
	const int w = (((head1 >> 8) & 7) + 1) << 4, h = (((head1 >> 12) & 7) + 1) << 4;
	uint16_t flag = (x - w / 2 > -w && x - w / 2 < b.cop_spr_maxx + w && y - h / 2 > -h && y - h / 2 < 256 + h) ? 1 : 0;
	flag = (uint16_t)((host_r16(src) & 0xfffe) | flag);
	host_w16(src, flag);
	if (flag & 1) {
		host_w16(b.dst1, head1);
		host_w16((uint16_t)(b.dst1 + 2), head2);
		host_w16((uint16_t)(b.dst1 + 4), (uint16_t)(x - w / 2));
		host_w16((uint16_t)(b.dst1 + 6), (uint16_t)(y - h / 2));
		b.dst1 = (uint16_t)(b.dst1 + 8);
	}
}

// the sprite key uploads (sprcpt_*: kept, not used, as MAME's)
static void combine32(uint32_t *val, int offset, uint16_t data, uint16_t mask)
{
	const int sh = offset ? 16 : 0;
	*val = (*val & ~((uint32_t)mask << sh)) | ((uint32_t)(data & mask) << sh);
}

static void sprcpt_w(uint32_t a, uint16_t d, uint16_t mask)
{
	const int off = (int)((a >> 1) & 1);
	switch (a & ~3u) {
	case 0x6a0: combine32(&b.sprcpt_val[0], off, d, mask); break;
	case 0x6a4:
		combine32(&b.sprcpt_data_3[b.sprcpt_idx % 6], off, d, mask);
		if (off == 1 && ++b.sprcpt_idx == 6)
			b.sprcpt_idx = 0;
		break;
	case 0x6a8:
		combine32(&b.sprcpt_data_4[b.sprcpt_idx % 4], off, d, mask);
		if (off == 1 && ++b.sprcpt_idx == 4)
			b.sprcpt_idx = 0;
		break;
	case 0x6ac: combine32(&b.sprcpt_flags1, off, d, mask); break;
	case 0x6b0: combine32(&b.sprcpt_data_1[b.sprcpt_adr & 0xff], off, d, mask); break;
	case 0x6b4: combine32(&b.sprcpt_data_2[b.sprcpt_adr & 0x3f], off, d, mask); break;
	case 0x6b8: combine32(&b.sprcpt_val[1], off, d, mask); break;
	case 0x6bc: combine32(&b.sprcpt_adr, off, d, mask); break;
	default: break;
	}
}

// ---- the Seibu CRTC (its register RAM, the layer enable and scroll callbacks) ----
static uint32_t crtc_off(uint32_t a)
{
	uint32_t o = (a - 0x600) >> 1;
	if (s_m == M_RAIDENDX)   // (read_alt / write_alt: offset bits 3 and 4 swapped)
		o = (o & ~0x18u) | ((o >> 1) & 0x08) | ((o << 1) & 0x10);
	return o & 0x3f;
}

static void crtc_w(uint32_t o, uint16_t data)   // (a word handler: the data's other lane is 0)
{
	b.crtc[o] = data;
	if (o == 0x1c / 2)
		b.tilemap_enable = data;
	else if (o >= 0x20 / 2 && o < 0x2c / 2)
		b.scroll[o - 0x20 / 2] = data;
}

// ---- the I/O (0x400-0x7ff) ----
static int cop_w_mapped(uint32_t off)   // (xsedae_cop_mem's COP registers)
{
	if (off < 0x1c || off == 0x8c || off == 0x8e)
		return 0;
	if (off >= 0x40 && off < 0x50 && off != 0x44)
		return 0;
	return 1;
}

static uint16_t r2_io_r(uint32_t a)
{
	uint16_t v = 0;
	switch (a) {
	case 0x470: return s_m == M_XSEDAE ? 0 : b.cop_bank;
	case 0x6c0: return b.cop_spr_off;
	case 0x6c2: return b.sprite_prot_src_addr[0];
	case 0x6dc: return b.cop_spr_maxx;
	case 0x740: return (uint16_t)rbx_port_read(s_port_dsw);
	case 0x744: return (uint16_t)rbx_port_read(s_port_p12);
	case 0x748: return (s_m == M_ZEROTEAM || s_m == M_XSEDAE) && s_port_p34 >= 0 ? (uint16_t)rbx_port_read(s_port_p34) : 0;
	case 0x74c: return (uint16_t)rbx_port_read(s_port_sys);
	case 0x762: return b.dst1;
	default: break;
	}
	if (a >= 0x600 && a < 0x640)
		return b.crtc[crtc_off(a)];
	if (a >= 0x700 && a < 0x720)
		return seibusnd_main_r((int)((a - 0x700) >> 2));
	if (a - 0x400 != 0x2c && seibucop_read(a - 0x400, &v))
		return v;
	return 0;
}

static void r2_io_w(uint32_t a, uint16_t d, uint16_t mask)
{
	switch (a) {
	case 0x470:
		if (s_m == M_RAIDEN2) {   // cop_tile_bank_2_w
			b.cop_bank = (uint16_t)((b.cop_bank & ~mask) | (d & mask));
			if (mask & 0xff00)
				b.fg_bank = 4u | (d >> 14);
		} else if (s_m == M_RAIDENDX) {   // raidendx_cop_bank_2_w
			b.cop_bank = (uint16_t)((b.cop_bank & ~mask) | (d & mask));
			b.fg_bank = 4u | ((b.cop_bank >> 4) & 3);
			b.mainbank = b.cop_bank >> 12;
			map_banks();
		}
		return;
	case 0x68e: memcpy(b.sprbuf, b.ram + s_spr_base, sizeof(b.sprbuf)); return;   // (the sprite buffer's copy)
	case 0x6c0: b.cop_spr_off = d; return;
	case 0x6c2: b.sprite_prot_src_addr[0] = d; return;
	case 0x6c6: b.dst1 = d; return;
	case 0x6ca:   // raiden2_bank_w (0x6cb; raiden2_cop_mem: not xsedae)
		if (s_m != M_XSEDAE && (mask & 0xff00)) {
			const uint8_t data = (uint8_t)(d >> 8);
			b.mainbank = !(data & 0x80);
			b.prg_bank = (data >> 7) & 1;
			map_banks();
		}
		return;
	case 0x6cc:   // tile_bank_01_w (0x6cc)
		if ((s_m == M_RAIDEN2 || s_m == M_RAIDENDX) && (mask & 0x00ff)) {
			b.bg_bank = (uint32_t)((d & 1) << 1);
			b.mid_bank = 1u | (d & 2);
		}
		return;
	case 0x6ce: b.sprcpt_flags2 = (uint16_t)((b.sprcpt_flags2 & ~mask) | (d & mask)); return;
	case 0x6d8: b.sprite_prot_x = d; return;
	case 0x6da: b.sprite_prot_y = d; return;
	case 0x6dc: b.cop_spr_maxx = d; return;
	case 0x6de: sprite_prot_src_w(d); return;
	default: break;
	}
	if (a >= 0x600 && a < 0x640) {
		crtc_w(crtc_off(a), d);
		return;
	}
	if (a >= 0x6a0 && a < 0x6c0) {
		sprcpt_w(a, d, mask);
		return;
	}
	if (a >= 0x700 && a < 0x720) {
		if (mask & 0x00ff)
			seibusnd_main_w((int)((a - 0x700) >> 2), (uint8_t)d);
		return;
	}
	if (cop_w_mapped(a - 0x400))
		(void)seibucop_write(a - 0x400, d, mask);
}

// the V33 boards (r2dx_v33.cpp)
static void r2dx_eeprom_w(uint8_t data)   // rdx_v33_eeprom_w
{
	const int64_t now = nec_now(&s_cpu);
	eepser_clk_write(&s_eeprom, (data >> 4) & 1, now);
	eepser_di_write(&s_eeprom, (data >> 5) & 1);
	eepser_cs_write(&s_eeprom, (data >> 3) & 1, now);
	b.r2dxgameselect = (data >> 2) & 1;
	b.tx_bank = b.r2dxgameselect;
	b.okibank = data & 3;
	map_banks();
}

static uint16_t v33_io_r(uint32_t a)
{
	switch (a) {
	case 0x430: return s_math[(((uint32_t)b.dy & 0xff) << 8 | (b.dx & 0xff)) % s_math_size];   // angle_r
	case 0x432: return (uint16_t)(uint32_t)sqrt((double)b.sdist);                             // dist_r
	case 0x434: case 0x436: {   // sin_r, cos_r
		const uint32_t off = 0x10000 + (uint32_t)(b.angle & 0xff) * 4 + (a == 0x436 ? 2 : 0);
		return (uint16_t)(s_math[off % s_math_size] | s_math[(off + 1) % s_math_size] << 8);
	}
	case 0x6c0: if (s_m == M_RDX_V33) return b.cop_spr_off; break;
	case 0x6c2: if (s_m == M_RDX_V33) return b.sprite_prot_src_addr[0]; break;
	case 0x6dc: if (s_m == M_RDX_V33) return b.cop_spr_maxx; break;
	case 0x740:
		if (s_m == M_RDX_V33)
			return 0xffff;   // r2dx_debug_r
		return (uint16_t)rbx_port_read(s_m == M_NZEROTEA ? s_port_dsw : s_port_p34);
	case 0x744:
		if (s_m == M_ZEROTM2K)
			rbx_port_custom(s_port_input, 0x8000, eepser_do_read_93cxx(&s_eeprom, nec_now(&s_cpu)) ? 0x8000 : 0);
		return (uint16_t)rbx_port_read(s_port_input);
	case 0x74c:
		if (s_m == M_RDX_V33)
			rbx_port_custom(s_port_sys, 0x10, eepser_do_read_93cxx(&s_eeprom, nec_now(&s_cpu)) ? 0x10 : 0);
		return (uint16_t)rbx_port_read(s_port_sys);
	case 0x762: if (s_m == M_RDX_V33) return b.dst1; break;
	case 0x780: if (s_m == M_RDX_V33) return oki_read(&s_oki); break;
	default: break;
	}
	if (a >= 0x600 && a < 0x640)
		return b.crtc[crtc_off(a)];
	if (s_m != M_RDX_V33 && a >= 0x780 && a < 0x7a0)
		return seibusnd_main_r((int)((a - 0x780) >> 2));
	return 0;
}

static void v33_io_w(uint32_t a, uint16_t d, uint16_t mask)
{
	switch (a) {
	case 0x400:   // tilemapdma_w (from 0xd000)
		for (int i = 0; i < 0x2800 / 2; i++)
			videoram_w(i, host_r16(0xd000 + 2u * (uint32_t)i));
		return;
	case 0x402:   // paldma_w (from 0x1f000)
		for (int i = 0; i < 0x1000 / 2; i++)
			palette_w(i, host_r16(0x1f000 + 2u * (uint32_t)i));
		return;
	case 0x404:   // r2dx_rom_bank_w
		if (s_m == M_RDX_V33) {
			b.r2dxbank = d & 0xf;
			map_banks();
		}
		return;
	case 0x406:   // tile_bank_w (0x406)
		if (s_m == M_RDX_V33 && (mask & 0x00ff)) {
			b.bg_bank = (d >> 4) & 1;
			b.mid_bank = 2u + ((d >> 5) & 1);
			b.fg_bank = 4u | (d & 3);
		}
		return;
	case 0x420: b.dx = (uint16_t)((b.dx & ~mask) | (d & mask)); return;
	case 0x422: b.dy = (uint16_t)((b.dy & ~mask) | (d & mask)); return;
	case 0x424: b.sdist = (b.sdist & (0xffff0000u | (uint16_t)~mask)) | (d & mask); return;
	case 0x426: b.sdist = (b.sdist & (0x0000ffffu | (uint32_t)(uint16_t)~mask << 16)) | (uint32_t)(d & mask) << 16; return;
	case 0x428: b.angle = (uint16_t)((b.angle & ~mask) | (d & mask)); return;
	case 0x68e: memcpy(b.sprbuf, b.ram + s_spr_base, sizeof(b.sprbuf)); return;
	case 0x6b0: b.mcu_prog[(b.mcu_prog_offs * 2u) & 0x7ff] = d; return;
	case 0x6b2: b.mcu_prog[(b.mcu_prog_offs * 2u + 1) & 0x7ff] = d; return;
	case 0x6bc: b.mcu_prog_offs = d; return;
	case 0x700:
		if (s_m == M_RDX_V33 && (mask & 0x00ff))
			r2dx_eeprom_w((uint8_t)d);
		return;
	case 0x748:
		if (s_m == M_ZEROTM2K) {   // zerotm2k_eeprom_w
			const int64_t now = nec_now(&s_cpu);
			eepser_clk_write(&s_eeprom, (d >> 1) & 1, now);
			eepser_di_write(&s_eeprom, (d >> 2) & 1);
			eepser_cs_write(&s_eeprom, d & 1, now);
		}
		return;
	case 0x780:
		if (s_m == M_RDX_V33) {
			if (mask & 0x00ff)
				oki_write(&s_oki, (uint8_t)d);
			return;
		}
		break;
	default: break;
	}
	if (s_m == M_RDX_V33) {   // (the sprite helper: rdx_v33_map only)
		switch (a) {
		case 0x6c0: b.cop_spr_off = d; return;
		case 0x6c2: b.sprite_prot_src_addr[0] = d; return;
		case 0x6c6: b.dst1 = d; return;
		case 0x6d8: b.sprite_prot_x = d; return;
		case 0x6da: b.sprite_prot_y = d; return;
		case 0x6dc: b.cop_spr_maxx = d; return;
		case 0x6de: sprite_prot_src_w(d); return;
		default: break;
		}
	}
	if (a >= 0x600 && a < 0x640) {
		crtc_w(crtc_off(a), d);
		return;
	}
	if (s_m != M_RDX_V33 && a >= 0x780 && a < 0x7a0 && (mask & 0x00ff))
		seibusnd_main_w((int)((a - 0x780) >> 2), (uint8_t)d);
}

static uint8_t io_r8(uint32_t a)
{
	const uint16_t v = s_v33 ? v33_io_r(a & ~1u) : r2_io_r(a & ~1u);
	return (uint8_t)((a & 1) ? v >> 8 : v);
}

static void io_w(uint32_t a, uint16_t d, uint16_t mask)
{
	if (s_v33)
		v33_io_w(a, d, mask);
	else
		r2_io_w(a, d, mask);
}

static int vector_r(void) { return 0xc0 / 4; }

// ---- the drawing ----
static void capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap.back, b.back, sizeof(s_snap.back));
	memcpy(s_snap.fore, b.fore, sizeof(s_snap.fore));
	memcpy(s_snap.mid, b.mid, sizeof(s_snap.mid));
	memcpy(s_snap.text, b.text, sizeof(s_snap.text));
	memcpy(s_snap.spr, b.sprbuf, sizeof(s_snap.spr));
	memcpy(s_snap.scroll, b.scroll, sizeof(s_snap.scroll));
	s_snap.enable = b.tilemap_enable;
	s_snap.bg_bank = b.bg_bank;
	s_snap.fg_bank = b.fg_bank;
	s_snap.mid_bank = b.mid_bank;
	s_snap.tx_bank = b.tx_bank;
	for (int i = 0; i < 0x800; i++) {   // xBGR_555
		const uint16_t w = b.pal[i];
		const uint32_t r = w & 0x1f, g = (w >> 5) & 0x1f, bl = (w >> 10) & 0x1f;
		s_snap.pal[i] = (r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (bl << 3 | bl >> 2);
	}
	rbx_video_frame_done();
}

// The frame is drawn in bands of lines (render_help: the threads helping the render thread take bands too); a band
// draws its own lines of the sprite bitmap (each sprite clipped to the band) and composes them, so every pixel gets
// the same writes in the same order as the whole-screen drawing.
#define BANDS 8
static int s_565;                    // (out_color is RGB565 by its values: the line converted inline)
static uint8_t s_sprpri[256];        // (raw sprites: the priorities a line's sprite pixels have, bit n = priority n)
static int16_t s_sprx0[256], s_sprx1[256];   // (raw sprites: the columns [x0, x1) a line's sprite tiles cover)
static uint8_t s_blend_grp[0x80];    // (any blended colour among a 16-colour group's 15 drawn pens)
static uint16_t *s_tile_tr, *s_char_tr;   // (per element: bit n = row n all pen 15; calloc at the init)

// an element's rows that are all the transparent pen (bit n: row n), for a set of 16 or 8 pixels wide
static uint16_t *rows_transparent(const rbx_gfx *g, uint8_t tp)
{
	uint16_t *m = (uint16_t *)calloc(g->count ? g->count : 1, sizeof(uint16_t));
	if (!m)
		return NULL;
	const size_t w = (size_t)g->width, h = (size_t)g->height;
	for (uint32_t e = 0; e < g->count; e++) {
		const uint8_t *p = g->pix + (size_t)e * w * h;
		uint16_t bits = 0;
		for (size_t r = 0; r < h; r++, p += w) {
			size_t i = 0;
			while (i < w && p[i] == tp)
				i++;
			if (i == w)
				bits |= (uint16_t)(1u << r);
		}
		m[e] = bits;
	}
	return m;
}

static inline uint32_t alpha_blend(uint32_t d, uint32_t s)   // alpha_blend_r32 at 0x7f
{
	const uint32_t l = 0x7f, il = 256 - l;
	return (((s & 0xff) * l + (d & 0xff) * il) >> 8) | ((((s & 0xff00) * l + (d & 0xff00) * il) >> 8) & 0xff00) |
	       ((((s & 0xff0000) * l + (d & 0xff0000) * il) >> 8) & 0xff0000);
}

// a tile row's n pixels (pen 15 transparent) over d: colours cb + pen, blended where s_blend says
static inline void tile_span(uint32_t *d, const uint8_t *src, int n, uint32_t cb)
{
	const uint32_t *pp = s_snap.pal + cb;
	if (!s_blend_grp[cb >> 4]) {
		for (int i = 0; i < n; i++) {
			const uint8_t pen = src[i];
			if (pen != 15)
				d[i] = pp[pen];
		}
		return;
	}
	const uint8_t *bl = s_blend + cb;
	for (int i = 0; i < n; i++) {
		const uint8_t pen = src[i];
		if (pen != 15)
			d[i] = bl[pen] ? alpha_blend(d[i], pp[pen]) : pp[pen];
	}
}

// a 16x16 layer's line (32 x 32, scan rows) over d, a tile row at a time
static void layer16_mix(const uint16_t *ram, uint32_t bank, uint32_t cbase, int sx, int sy, int y, uint32_t *d)
{
	const int ty = (y + sy) & 511;
	const uint16_t *row = ram + (ty >> 4) * 32;
	const size_t roff = (size_t)(ty & 15) * 16;
	int tx = sx & 511;
	for (int x = 0; x < VIS_W;) {
		const int fx = tx & 15;
		int n = 16 - fx;
		if (n > VIS_W - x)
			n = VIS_W - x;
		const uint16_t data = row[tx >> 4];
		const uint32_t code = rbx_wrap((data & 0xfffu) | bank << 12, s_tiles.count);
		if (!((s_tile_tr[code] >> (ty & 15)) & 1))
			tile_span(d + x, s_tiles.pix + (size_t)code * 256 + roff + (size_t)fx, n, cbase + (uint32_t)(data >> 12) * 16);
		x += n;
		tx = (tx + n) & 511;
	}
}

static void text_mix(int y, uint32_t *d)   // (64 x 32 of 8x8, scan rows; not scrolled)
{
	const int ty = y & 255;
	const uint16_t *row = s_snap.text + (ty >> 3) * 64;
	const size_t roff = (size_t)(ty & 7) * 8;
	for (int x = 0; x < VIS_W; x += 8) {
		const uint16_t data = row[x >> 3];
		const uint32_t code = rbx_wrap((data & 0xfffu) | s_snap.tx_bank << 12, s_chars.count);
		if (!((s_char_tr[code] >> (ty & 7)) & 1))
			tile_span(d + x, s_chars.pix + (size_t)code * 64 + roff, 8, 0x700 + (uint32_t)((data >> 12) & 0xf) * 16);
	}
}

static void layer_mix(int layer, int y, uint32_t *d)
{
	switch (layer) {
	case 0: layer16_mix(s_snap.back, s_snap.bg_bank, 0x400, s_snap.scroll[0], s_snap.scroll[1], y, d); break;   // (color | 0 << 4)
	case 1: layer16_mix(s_snap.mid, s_snap.mid_bank, 0x600, s_snap.scroll[2], s_snap.scroll[3], y, d); break;   // (color | 2 << 4)
	case 2: layer16_mix(s_snap.fore, s_snap.fg_bank, 0x500, s_snap.scroll[4], s_snap.scroll[5], y, d); break;   // (color | 1 << 4)
	default: text_mix(y, d); break;
	}
}

// xsedae: a layer's line as pens (0xffff: pen 15) for its priority drawing
static void layer_pens(int layer, int y, uint16_t *out)
{
	const uint16_t *ram;
	uint32_t bank, cbase;
	int sx, sy;
	if (layer == 3) {
		const int ty = y & 255;
		for (int x = 0; x < VIS_W; x++) {
			const uint16_t data = s_snap.text[(ty >> 3) * 64 + (x >> 3)];
			const uint32_t code = rbx_wrap((data & 0xfffu) | s_snap.tx_bank << 12, s_chars.count);
			const uint8_t pen = s_chars.pix[(size_t)code * 64 + (size_t)(ty & 7) * 8 + (size_t)(x & 7)];
			out[x] = pen == 15 ? 0xffff : (uint16_t)(0x700 + ((data >> 12) & 0xf) * 16 + pen);
		}
		return;
	}
	switch (layer) {
	case 0: ram = s_snap.back; bank = s_snap.bg_bank; cbase = 0x400; sx = s_snap.scroll[0]; sy = s_snap.scroll[1]; break;
	case 1: ram = s_snap.mid; bank = s_snap.mid_bank; cbase = 0x600; sx = s_snap.scroll[2]; sy = s_snap.scroll[3]; break;
	default: ram = s_snap.fore; bank = s_snap.fg_bank; cbase = 0x500; sx = s_snap.scroll[4]; sy = s_snap.scroll[5]; break;
	}
	const int ty = (y + sy) & 511;
	for (int x = 0; x < VIS_W; x++) {
		const int tx = (x + sx) & 511;
		const uint16_t data = ram[(ty >> 4) * 32 + (tx >> 4)];
		const uint32_t code = rbx_wrap((data & 0xfffu) | bank << 12, s_tiles.count);
		const uint8_t pen = s_tiles.pix[(size_t)code * 256 + (size_t)(ty & 15) * 16 + (size_t)(tx & 15)];
		out[x] = pen == 15 ? 0xffff : (uint16_t)(cbase + (data >> 12) * 16 + pen);
	}
}

// sei25x_rise1x draw_sprites: each sprite's tiles through plot (raw: m_sprite_bitmap; prio: xsedae's prio_transpen),
// the lines [y0, y1) only
static void draw_sprites(int raw, int y0, int y1)
{
	const uint16_t *ram = s_snap.spr;
	const uint32_t count = s_spr.count;
	const int start = raw ? 0x800 - 4 : 0, end = raw ? -4 : 0x800, inc = raw ? -4 : 4;
	for (int i = start; i != end; i += inc) {
		uint32_t code = ram[i + 1];
		if (rbx_wrap(code, count) == 0)
			continue;
		const int flipy = (ram[i] >> 15) & 1, sizey = ((ram[i] >> 12) & 7) + 1, flipx = (ram[i] >> 11) & 1,
		          sizex = ((ram[i] >> 8) & 7) + 1, pri = (ram[i] >> 6) & 3;
		const uint32_t color = ram[i] & 0x3f;
		int x = ram[i + 2] & 0x1ff, y = ram[i + 3] & 0x1ff;
		if (x >= 0x180) x -= 0x200;
		if (y >= 0x180) y -= 0x200;
		if (y >= y1 || y + 16 * sizey <= y0)
			continue;
		// raw: (color << 4) | (pri << 14) added to the pens; prio: xsedae pri_callback's mask
		static const uint32_t pmasks[4] = { 0xcccc | 0xf0f0 | 0xff00, 0xf0f0 | 0xff00, 0xff00, 0 };
		const uint16_t cadd = raw ? (uint16_t)(color << 4 | (uint32_t)pri << 14) : (uint16_t)(color * 16);
		const uint32_t pmask = pmasks[pri] | 1u << 31;
		for (int ax = 0; ax < sizex; ax++) {
			const int sx = flipx ? x + 16 * (sizex - ax - 1) : x + 16 * ax;
			if (sx <= -16 || sx >= VIS_W) {
				code += (uint32_t)sizey;
				continue;
			}
			const int px0 = sx < 0 ? -sx : 0, px1 = sx > VIS_W - 16 ? VIS_W - sx : 16;
			for (int ay = 0; ay < sizey; ay++) {
				const int sy = flipy ? y + 16 * (sizey - ay - 1) : y + 16 * ay;
				const uint32_t c = code++;
				if (sy >= y1 || sy + 16 <= y0)
					continue;
				const uint8_t *t = s_spr.pix + (size_t)rbx_wrap(c, count) * 256;
				const int py0 = sy < y0 ? y0 - sy : 0, py1 = sy + 16 > y1 ? y1 - sy : 16;
				for (int py = py0; py < py1; py++) {
					const int dy = sy + py;
					const uint8_t *row = t + (flipy ? 15 - py : py) * 16;
					uint16_t *d = s_sprbmp + (size_t)dy * VIS_W + sx;
					if (raw) {
						int any = 0;
						if (flipx) {
							for (int px = px0; px < px1; px++) {
								const uint8_t pen = row[15 - px];
								if (pen != 15) {
									d[px] = (uint16_t)(cadd + pen);
									any = 1;
								}
							}
						} else {
							for (int px = px0; px < px1; px++) {
								const uint8_t pen = row[px];
								if (pen != 15) {
									d[px] = (uint16_t)(cadd + pen);
									any = 1;
								}
							}
						}
						if (any) {
							s_sprpri[dy] |= (uint8_t)(1u << pri);
							if (s_sprx0[dy] > sx + px0)
								s_sprx0[dy] = (int16_t)(sx + px0);
							if (s_sprx1[dy] < sx + px1)
								s_sprx1[dy] = (int16_t)(sx + px1);
						}
					} else {
						uint8_t *p = s_pribmp + (size_t)dy * VIS_W + sx;
						for (int px = px0; px < px1; px++) {
							const uint8_t pen = row[flipx ? 15 - px : px];
							if (pen == 15)
								continue;
							if (!((1u << (p[px] & 0x1f)) & pmask))
								d[px] = (uint16_t)(cadd + pen);
							p[px] = 31;
						}
					}
				}
			}
		}
	}
}

static void out_line(int y, const uint32_t *d)
{
	uint8_t *o = (uint8_t *)rbx_video.out + (size_t)y * rbx_video.out_pitch;
	if (s_565)
		for (int x = 0; x < VIS_W; x++)
			((uint16_t *)o)[x] = (uint16_t)((d[x] >> 8 & 0xf800) | (d[x] >> 5 & 0x07e0) | (d[x] >> 3 & 0x001f));
	else if (rbx_video.out_bpp == 4)
		for (int x = 0; x < VIS_W; x++)
			((uint32_t *)o)[x] = rbx_video.out_color((int)(d[x] >> 16) & 0xff, (int)(d[x] >> 8) & 0xff, (int)d[x] & 0xff);
	else
		for (int x = 0; x < VIS_W; x++)
			((uint16_t *)o)[x] = (uint16_t)rbx_video.out_color((int)(d[x] >> 16) & 0xff, (int)(d[x] >> 8) & 0xff, (int)d[x] & 0xff);
}

// xsedae_state::screen_update: the layers with priorities, the sprites' prio_transpen
static void xsedae_band(int y0, int y1)
{
	const int en = ~s_snap.enable;
	const uint32_t *pal = s_snap.pal;
	uint32_t d[VIS_W];
	uint16_t line[VIS_W];
	for (int y = y0; y < y1; y++) {
		uint16_t *bm = s_sprbmp + (size_t)y * VIS_W;
		uint8_t *pr = s_pribmp + (size_t)y * VIS_W;
		for (int x = 0; x < VIS_W; x++)
			bm[x] = 0xffff;
		memset(pr, 0, VIS_W);
		for (int l = 0; l < 4; l++) {
			if (!((en >> l) & 1))
				continue;
			layer_pens(l, y, line);
			for (int x = 0; x < VIS_W; x++)
				if (line[x] != 0xffff) {
					bm[x] = line[x];
					pr[x] |= (uint8_t)(1 << l);
				}
		}
	}
	if (en & 0x10)
		draw_sprites(0, y0, y1);
	for (int y = y0; y < y1; y++) {
		const uint16_t *bm = s_sprbmp + (size_t)y * VIS_W;
		for (int x = 0; x < VIS_W; x++)
			d[x] = bm[x] == 0xffff ? 0 : pal[bm[x] & 0x7ff];
		out_line(y, d);
	}
}

// raiden2_state::screen_update: the sprites drawn raw, each priority's blended in among the layers
static void r2_band(int band)
{
	const int h = s_vis_h, y0 = h * band / BANDS, y1 = h * (band + 1) / BANDS;
	if (!s_spri) {
		xsedae_band(y0, y1);
		return;
	}
	const int en = ~s_snap.enable;
	const uint32_t *pal = s_snap.pal;
	uint32_t d[VIS_W];
	if (en & 0x10) {
		for (int i = y0 * VIS_W; i < y1 * VIS_W; i++)
			s_sprbmp[i] = 0xffff;
		for (int y = y0; y < y1; y++) {
			s_sprpri[y] = 0;
			s_sprx0[y] = VIS_W;
			s_sprx1[y] = 0;
		}
		draw_sprites(1, y0, y1);
	}
	for (int y = y0; y < y1; y++) {
		for (int x = 0; x < VIS_W; x++)
			d[x] = 0;
		const uint16_t *sp = s_sprbmp + (size_t)y * VIS_W;
		const int spri = (en & 0x10) ? s_sprpri[y] : 0;
		for (int step = 0; step < 5; step++) {
			const int sl = s_spri[step];
			// (only a line's sprite pixels of that priority, within its sprite tiles' columns)
			if (sl >= 0 && ((spri >> sl) & 1))
				for (int x = s_sprx0[y]; x < s_sprx1[y]; x++) {
					const uint16_t v = sp[x];
					if ((v & 0xc000) == (uint16_t)(sl << 14) && (v & 0xf) != 0xf) {
						const uint16_t c = v & 0x7ff;
						d[x] = s_blend[c] ? alpha_blend(d[x], pal[c]) : pal[c];
					}
				}
			if (step < 4 && ((en >> step) & 1))
				layer_mix(step, y, d);
		}
		out_line(y, d);
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, r2_band };
static int r2_help(int helper) { return decoband_help(&s_bands, helper); }

static void r2_render(void)
{
	if (!rbx_video.out)
		return;
	s_565 = rbx_video.out_bpp == 2 && rbx_video.out_color(0xff, 0, 0) == 0xf800 && rbx_video.out_color(0, 0xff, 0) == 0x07e0 &&
	        rbx_video.out_color(0, 0, 0xff) == 0x001f && rbx_video.out_color(0x84, 0x82, 0x21) == 0x8404;
	decoband_run(&s_bands);
	rbx_video.out_done = 1;
}

// ---- the scheduler ----
static int64_t s_run0;
static int cpu_run(int n)
{
	s_run0 = s_cpu.total;
	nec_run_until(&s_cpu, s_run0 + n);
	return (int)(s_cpu.total - s_run0);
}
static int64_t cpu_done(void) { return nec_now(&s_cpu) - s_run0; }
static void cpu_abort(void) { s_cpu.stop = 1; }

enum { T_VBLANK = 1, T_SOUND };
static int64_t s_frame_as;
static uint32_t s_fps_mhz;

static void fire(int id, int32_t param)
{
	if (!s_oki_rom && seibusnd_fire(id, param))
		return;
	switch (id) {
	case T_VBLANK:   // the screen update, the V30's IRQ (irq0_line_hold), the timer again
		capture();
		nec_set_input(&s_cpu, NEC_LINE_IRQ, 1);
		sched_timer_set(T_VBLANK, sched_now() + s_frame_as, 0);
		break;
	case T_SOUND:
		sched_timer_set(T_SOUND, sched_now() + SCHED_AS_PER_SEC / 50, 0);
		break;
	default: break;
	}
}

static void r2_frame(void)
{
	rbx_audio.frame_mhz = s_fps_mhz;
	sched_run(T_VBLANK, fire);
	if (s_oki_rom) {   // (the V33 board's OKI: the frame's samples)
		b.oki_frac += (uint64_t)s_oki.rate * 1000u;
		const uint32_t n = (uint32_t)(b.oki_frac / s_fps_mhz);
		b.oki_frac -= (uint64_t)n * s_fps_mhz;
		for (uint32_t i = 0; i < n; i++)
			rbx_audio_put(oki_sample(&s_oki));
	} else
		seibusnd_frame_end(s_frame_as);
	sched_rebase(s_frame_as);
}

static void r2_reset(void)
{
	rbx_video_wait_idle();
	memset(&b, 0, offsetof(__typeof__(b), state_end));
	switch (s_m) {
	case M_RAIDEN2: bank_reset(0, 6, 1, 0); b.mainbank = 1; break;
	case M_RAIDENDX: bank_reset(0, 6, 1, 0); b.mainbank = 0; break;
	case M_RDX_V33: bank_reset(0, 6, 2, 0); break;
	default: bank_reset(0, 2, 1, 0); b.mainbank = 1; break;   // (zeroteam, xsedae, nzerotea, zerotm2k)
	}
	map_banks();
	if (!s_v33)
		seibucop_reset();
	nec_reset(&s_cpu);
	s_cpu.total = 0;
	g_sched.base = 0;
	for (int i = 0; i < g_sched.nex; i++) {
		g_sched.ex[i].local = 0;
		g_sched.ex[i].suspended = 0;
	}
	for (int i = 0; i < SCHED_TIMERS; i++)
		g_sched.tm[i].id = -1;
	sched_timer_set(T_VBLANK, s_frame_as, 0);
	sched_timer_set(T_SOUND, SCHED_AS_PER_SEC / 50, 0);
	if (s_oki_rom)
		oki_reset(&s_oki);
	else
		seibusnd_reset();
}

static void r2_exit(void)
{
	if (s_has_eeprom)
		eepser_nv_save(&s_eeprom);
	s_has_eeprom = 0;
	if (!s_oki_rom)
		seibusnd_exit();
	s_oki_rom = NULL;
	rbx_gfx_free(&s_chars);
	rbx_gfx_free(&s_tiles);
	rbx_gfx_free(&s_spr);
	free(s_tile_tr);
	free(s_char_tr);
	s_tile_tr = NULL;
	s_char_tr = NULL;
	free(s_sprbmp);
	free(s_pribmp);
	s_sprbmp = NULL;
	s_pribmp = NULL;
	free(s_bp);
	s_bp = NULL;
	free(s_snapp);
	s_snapp = NULL;
}

// the blended colours (raiden_blended_colors, zeroteam_blended_colors)
static const uint16_t raiden_blended_colors[] = {
	0x380,
	0x3c0, 0x3c1, 0x3c2, 0x3c3, 0x3c4, 0x3c5, 0x3c6, 0x3c7, 0x3c8, 0x3c9, 0x3ca, 0x3cb, 0x3cc, 0x3cd, 0x3ce,
	0x3d0, 0x3d1, 0x3d2, 0x3d3, 0x3d4, 0x3d5, 0x3d6, 0x3d7, 0x3d8, 0x3d9, 0x3da, 0x3db, 0x3dc, 0x3dd, 0x3de,
	0x3f0, 0x3f1, 0x3f2, 0x3f3, 0x3f4, 0x3f5, 0x3f6, 0x3f7, 0x3f8, 0x3f9, 0x3fa, 0x3fb, 0x3fc, 0x3fd, 0x3fe,
	0x4f8, 0x4f9, 0x4fa, 0x4fb, 0x4fc, 0x4fd, 0x4fe,
	0x5c8, 0x5c9, 0x5ca, 0x5cb, 0x5cc, 0x5cd, 0x5ce,
	0x5de,
	0x5e8, 0x5e9, 0x5ea, 0x5eb, 0x5ec, 0x5ed, 0x5ee,
	0x5f8, 0x5f9, 0x5fa, 0x5fb, 0x5fc, 0x5fd, 0x5fe,
	0x6c8, 0x6c9, 0x6ca, 0x6cb, 0x6cc, 0x6cd, 0x6ce,
	0x6d8, 0x6d9, 0x6da, 0x6db, 0x6dc, 0x6dd, 0x6de,
	0x6e8, 0x6e9, 0x6ea, 0x6eb, 0x6ec, 0x6ed, 0x6ee,
	0x6f8, 0x6f9, 0x6fa, 0x6fb, 0x6fc, 0x6fd, 0x6fe,
	0x70d, 0x70e,
	0x71c, 0x71d, 0x71e,
	0x72d, 0x72e,
	0x73d, 0x73e,
	0x74d, 0x74e,
	0x75c,
	0x76c, 0x76d, 0x76e,
	0x77d, 0x77e,
	0x7c8, 0x7c9, 0x7ca, 0x7cb, 0x7cc, 0x7cd, 0x7ce,
	0xffff,
};
static const uint16_t zeroteam_blended_colors[] = { 0x37e, 0x38e, 0x52e, 0x5de, 0xffff };
static const int8_t spri_raiden[5] = { 0, 1, 2, 3, -1 }, spri_zeroteam[5] = { -1, 0, 1, 2, 3 };

// MAME's charlayout, tilelayout, gfx_16x16x4_packed_lsb
static const uint32_t pl_tile[4] = { 8, 12, 0, 4 };
static const uint32_t ch_x[8] = { 3, 2, 1, 0, 19, 18, 17, 16 };
static const uint32_t ch_y[8] = { 0, 32, 64, 96, 128, 160, 192, 224 };
static const uint32_t tl_x[16] = { 3, 2, 1, 0, 19, 18, 17, 16, 515, 514, 513, 512, 531, 530, 529, 528 };
static const uint32_t tl_y[16] = { 0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448, 480 };
static const uint32_t pl_spr[4] = { 0, 1, 2, 3 };
static const uint32_t sp_x[16] = { 4, 0, 12, 8, 20, 16, 28, 24, 36, 32, 44, 40, 52, 48, 60, 56 };
static const uint32_t sp_y[16] = { 0, 64, 128, 192, 256, 320, 384, 448, 512, 576, 640, 704, 768, 832, 896, 960 };

static int r2_init(const rbx_game *g)
{
	rbx_cheat_prefer("nec");   // (the cheats' main CPU: another CPU is set up first)
	const char *mc = g->machine;
	s_m = !strcmp(mc, "raidendx") ? M_RAIDENDX : !strcmp(mc, "zeroteam") ? M_ZEROTEAM : !strcmp(mc, "xsedae") ? M_XSEDAE :
	      !strcmp(mc, "rdx_v33") ? M_RDX_V33 : !strcmp(mc, "nzerotea") ? M_NZEROTEA : !strcmp(mc, "zerotm2k") ? M_ZEROTM2K :
	      M_RAIDEN2;
	s_v33 = s_m >= M_RDX_V33;
	s_bp = calloc(1, sizeof(board_t));
	s_snapp = calloc(1, sizeof(snap_t));
	if (!s_bp || !s_snapp) {
		r2_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t chs, tls, sps, snd_size = 0, okis = 0, oki2s = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *ch = rbx_region("chars", &chs), *tl = rbx_region("tiles", &tls), *sp = rbx_region("sprites", &sps);
	if (!s_rom || !ch || !tl || !sp) {
		r2_exit();
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	const uint8_t *snd = NULL, *oki = NULL, *oki2 = NULL;
	if (s_m != M_RDX_V33) {
		snd = rbx_region("audiocpu", &snd_size);
		oki = s_m == M_RAIDEN2 || s_m == M_RAIDENDX ? rbx_region("oki1", &okis) : rbx_region("oki", &okis);
		if (s_m == M_RAIDEN2 || s_m == M_RAIDENDX)
			oki2 = rbx_region("oki2", &oki2s);
		if (!snd) {
			r2_exit();
			rbx_fail("%s: ROM regions missing", g->set);
			return -1;
		}
	}
	if (s_v33) {
		s_math = rbx_region("math", &s_math_size);
		if (!s_math || s_math_size < 0x10400) {
			r2_exit();
			rbx_fail("%s: ROM regions missing", g->set);
			return -1;
		}
	}
	if ((s_m == M_RAIDENDX && s_rom_size < 0x200000) || (s_m == M_RDX_V33 && s_rom_size < 0x400000) || s_rom_size < 0x100000) {
		r2_exit();
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	// the init_*: the blending, the sprites' decryption (zerotm2k: the BG ROM's two address lines)
	memset(s_blend, 0, sizeof(s_blend));
	const uint16_t *bl = NULL;
	switch (s_m) {
	case M_RAIDEN2: case M_RAIDENDX: case M_RDX_V33:
		bl = raiden_blended_colors;
		s_spri = spri_raiden;
		raiden2_decrypt_sprites(sp, sps / 4);
		break;
	case M_ZEROTEAM: case M_NZEROTEA:
		bl = zeroteam_blended_colors;
		s_spri = spri_zeroteam;
		zeroteam_decrypt_sprites(sp, (sps < 0x400000 ? sps : 0x400000) / 4);
		break;
	case M_ZEROTM2K:
		bl = zeroteam_blended_colors;
		s_spri = spri_zeroteam;
		if (tls >= 0x180000) {
			uint8_t *buf = (uint8_t *)malloc(0x80000), *src = tl + 0x100000;
			if (!buf) {
				r2_exit();
				rbx_fail("%s: out of memory", g->set);
				return -1;
			}
			for (uint32_t i = 0; i < 0x80000; i++)
				buf[i] = src[(i & ~0x60u) | ((i >> 1) & 0x20) | ((i << 1) & 0x40)];
			memcpy(src, buf, 0x80000);
			free(buf);
		}
		break;
	default:   // (xsedae: no blending, its own screen update)
		s_spri = NULL;
		break;
	}
	while (bl && *bl != 0xffff)
		s_blend[*bl++ & 0x7ff] = 1;
	for (int i = 0; i < 0x80; i++) {
		s_blend_grp[i] = 0;
		for (int p = 0; p < 15; p++)
			s_blend_grp[i] |= s_blend[i * 16 + p];
	}
	s_vis_h = (s_m == M_RAIDEN2 || s_m == M_RAIDENDX || s_m == M_RDX_V33) ? 240 : 256;
	s_spr_base = (s_m == M_ZEROTEAM || s_m == M_XSEDAE) ? 0xf000 : 0xc000;
	const rbx_gfx_layout chl = { 8, 8, 4, pl_tile, ch_x, ch_y, 256 };
	const rbx_gfx_layout tll = { 16, 16, 4, pl_tile, tl_x, tl_y, 1024 };
	const rbx_gfx_layout spl = { 16, 16, 4, pl_spr, sp_x, sp_y, 1024 };
	s_sprbmp = (uint16_t *)calloc((size_t)VIS_W * 256, sizeof(uint16_t));
	s_pribmp = (uint8_t *)calloc((size_t)VIS_W * 256, 1);
	if (!s_sprbmp || !s_pribmp || rbx_gfx_decode(&s_chars, &chl, ch, chs) || rbx_gfx_decode(&s_tiles, &tll, tl, tls) ||
	    rbx_gfx_decode(&s_spr, &spl, sp, sps) || rbx_video_setup(VIS_W, s_vis_h, 2048)) {
		r2_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_tile_tr = rows_transparent(&s_tiles, 15);
	s_char_tr = rows_transparent(&s_chars, 15);
	if (!s_tile_tr || !s_char_tr) {
		r2_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_port_dsw = rbx_port_find("DSW");
	s_port_p12 = rbx_port_find("P1_P2");
	s_port_p34 = rbx_port_find("P3_P4");
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_input = rbx_port_find("INPUT");
	// the screens: the V30 boards' raw (8 MHz, 512 x 282), the V33 boards' 55.47 Hz
	if (s_v33) {
		s_frame_as = (int64_t)(1e18 / 55.47);
		s_fps_mhz = 55470;
	} else {
		s_frame_as = (int64_t)512 * 282 * (SCHED_AS_PER_SEC / 8000000);
		s_fps_mhz = (uint32_t)(8000000000ull / (512 * 282));
	}
	sched_init(SCHED_AS_PER_SEC / 60);   // (no maximum quantum set: MAME's 60 Hz default)
	(void)sched_add_exec(CPU_CLOCK, cpu_run, cpu_done, cpu_abort);
	if (s_m == M_RDX_V33) {   // (one OKI on the main CPU, banked by the EEPROM latch)
		uint32_t os = 0;
		s_oki_rom = rbx_region("oki", &os);
		if (!s_oki_rom || os < 0x100000 || rbx_audio_setup(OKI_CLOCK / 132)) {
			s_oki_rom = NULL;
			r2_exit();
			rbx_fail("%s: ROM regions missing", g->set);
			return -1;
		}
		s_oki_size = os;
		oki_init(&s_oki, OKI_CLOCK, 1, s_oki_rom, 0x40000, 128);   // (0.5)
	} else {
		seibusnd_config sc;
		memset(&sc, 0, sizeof(sc));
		sc.rom = snd;
		sc.rom_size = snd_size;
		sc.coin_port = rbx_port_find("COIN");
		sc.z80_clock = SND_CLOCK;
		sc.ym_clock = SND_CLOCK;
		if (s_m == M_ZEROTEAM || s_m == M_NZEROTEA || s_m == M_ZEROTM2K) {
			sc.ym_type = RBX_YM3812;
			sc.ym_gain = 256;
		} else {
			sc.ym_type = RBX_YM2151;
			sc.ym_gain = 128;   // (0.50)
		}
		sc.oki_rom = oki;
		sc.oki_size = okis;
		sc.oki_clock = OKI_CLOCK;
		sc.oki_pin7 = 1;
		sc.oki_gain = 102;   // (0.40)
		sc.r2map = 1;
		sc.oki2_rom = oki2;
		sc.oki2_size = oki2s;
		sc.ym_async = 1;   // (the FM samples on the rbx_aux worker)
		if (seibusnd_init(&sc)) {
			r2_exit();
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
	}
	if (!s_v33) {
		const seibucop_config cc = { videoram_w, palette_w, host_cycles, host_r8, host_r16, host_w8, host_w16, 1 };
		seibucop_init(&cc);
	}
	s_has_eeprom = s_m == M_RDX_V33 || s_m == M_ZEROTM2K;
	if (s_has_eeprom) {   // (EEPROM_93C46_16BIT)
		uint32_t es = 0;
		const uint8_t *e = rbx_region_opt("eeprom", &es);
		eepser_init(&s_eeprom, EEPSER_93CXX, 64, 16, CPU_CLOCK);
		if (e)
			eepser_default(&s_eeprom, e, es);
		eepser_nv_load(&s_eeprom, g->set);
	}
	// the V30 / V33A
	nec_init(&s_cpu, s_v33 ? NEC_V33 : NEC_V30, NULL);
	s_cpu.irq_ack = vector_r;
	s_cpu.read8 = cpu_r8;
	s_cpu.read16 = cpu_r16;
	s_cpu.write8 = cpu_w8;
	s_cpu.write16 = cpu_w16;
	nec_map(&s_cpu, 0x01000, 0x20000, s_bp->ram + 0x1000, s_bp->ram + 0x1000);
	switch (s_m) {
	case M_RAIDEN2: case M_ZEROTEAM: nec_map(&s_cpu, 0x40000, 0x100000, s_rom + 0x40000, NULL); break;
	case M_RAIDENDX: nec_map(&s_cpu, 0x30000, 0x100000, s_rom + 0x30000, NULL); break;
	case M_RDX_V33: break;   // (map_banks)
	default: nec_map(&s_cpu, 0x20000, 0x100000, s_rom + 0x20000, NULL); break;
	}
	if (!getenv("RBX_SEIBU_NOIDLE"))   // (the vblank waits: the same result, RBX_SEIBU_NOIDLE=1 to compare)
		nec_idle_enable(&s_cpu, cpu_pure);
	r2_reset();
	return 0;
}

static int r2_state(void *ctx, int load)
{
	if (load)
		rbx_video_wait_idle();
	int rc = rbx_state_io(ctx, "raiden2", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= rbx_state_io(ctx, "raiden2_cpu", &s_cpu, NEC_STATE_SIZE);
	if (!s_v33)
		rc |= seibucop_state(ctx, load);
	if (s_oki_rom)
		rc |= rbx_state_io(ctx, "raiden2_oki", &s_oki, (uint32_t)offsetof(okim6295_t, rom));
	else
		rc |= seibusnd_state(ctx, load);
	if (s_has_eeprom)
		rc |= eepser_state(&s_eeprom, ctx, load, "raiden2_eeprom");
	rc |= sched_state(ctx, load);
	if (load && !rc) {
		nec_after_load(&s_cpu);
		map_banks();
	}
	return rc ? -1 : 0;
}

static int r2_save(void *ctx) { return r2_state(ctx, 0); }
static int r2_load(void *ctx) { return r2_state(ctx, 1); }
static int r2_supported(const char *machine)
{
	return !strcmp(machine, "raiden2") || !strcmp(machine, "raidendx") || !strcmp(machine, "zeroteam") ||
	       !strcmp(machine, "xsedae");
}
static int v33_supported(const char *machine)
{
	return !strcmp(machine, "rdx_v33") || !strcmp(machine, "nzerotea") || !strcmp(machine, "zerotm2k");
}

const rbx_driver rbx_drv_raiden2 = { "Seibu Raiden II", r2_init, r2_exit, r2_reset, r2_frame, r2_save, r2_load,
                                     r2_supported, r2_render, r2_help };
const rbx_driver rbx_drv_r2dx_v33 = { "Seibu Raiden II V33", r2_init, r2_exit, r2_reset, r2_frame, r2_save, r2_load,
                                      v33_supported, r2_render, r2_help };
