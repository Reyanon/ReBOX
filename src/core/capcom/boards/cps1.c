/*  Capcom CPS1. Ported from MAME src/mame/capcom/cps1.cpp, cps1_v.cpp (BSD-3-Clause, copyright-holders: Paul Leaman,
    Andreas Naive, Nicola Salmoria, Charles MacDonald).
    A 68000 at 10 or 12 MHz; the CPS-A (base registers of the scroll, object, palette RAM in the graphics RAM, scrolls)
    and the CPS-B (layer order and enables, the priority masks, protection, extra inputs; per game: gen/cps1cfg.inc);
    three tilemaps (8x8, 16x16 with line scroll, 32x32) and 16x16 sprites from one graphics ROM through the B-board's
    PAL (the bank mapper); a palette of 12-bit colours with brightness, copied when its base is written. Sound: a Z80
    at 3.58 MHz, a YM2151, an OKI. 512x262 at 8 MHz (59.64 Hz), 384x224 shown.
    The QSound boards (CPS1.5: qsound_main_map, qsound_sub_map): a Kabuki Z80 at 8 MHz (the opcodes decrypted, IRQ at
    250 Hz), RAM shared with the 68000, the QSound DSP (sound/qsound.c), a 93C46 EEPROM (8-bit). Kabuki: MAME
    src/mame/capcom/kabuki.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria).
    CPS2 (src/mame/capcom/cps2.cpp, the same copyright-holders): a 68000 at 16 MHz (x 0.7375 as MAME) running the
    opcodes decrypted (machine/cps2crypt.c, the game's key), the CPS-A / CPS-B (the "cps2" configuration) with the
    raster counters (IRQ 4, the lines drawn with the registers of MAME's partial updates), the object RAM (two banks,
    latched at the vblank, 1024 sprites with priorities against the layers), the QSound Z80 (not encrypted, held in
    reset by the EEPROM port), a 93C46 EEPROM (16-bit).
    (Bootlegs: not yet.)
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/qsound.h"
#include "../../machine/eepromser.h"
#include "../machine/cps2crypt.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "../gen/cps1cfg.inc"

#define FRAME_MHZ 59637          // 8 MHz / (512 x 262)
#define LINES 262
#define VIS_X0 64
#define VIS_Y0 16
#define VIS_W 384
#define VIS_H 224
#define Z80_CLOCK 3579545

// CPS-A registers (words)
enum { A_OBJ = 0, A_SCROLL1 = 1, A_SCROLL2 = 2, A_SCROLL3 = 3, A_OTHER = 4, A_PALETTE = 5, A_S1X = 6, A_S1Y = 7,
       A_S2X = 8, A_S2Y = 9, A_S3X = 10, A_S3Y = 11, A_ROWSCROLL = 16, A_VIDEOCONTROL = 17 };

static const struct CPS1config *s_cfg;
static uint8_t *s_rom, *s_zrom, *s_gfx;
static uint32_t s_rom_size, s_zrom_size, s_gfx_size;
static uint32_t s_cpu_clock;
static uint32_t s_cpsb_base;          // 0x800140 (sf2ee / sf2ue: 0x8001c0)
static int s_port_in0, s_port_in1, s_port_in2, s_port_in3, s_port_dsw[3];

static struct cps1_state {   // (on the heap while the game runs, ADR-040)
	uint16_t ram[0x8000];
	uint16_t gfxram[0x20000];      // 0x900000 .. 0x92ffff mapped (the bases reach 0x3ffff)
	uint16_t cps_a[0x20], cps_b[0x20];
	uint16_t palraw[0xc00];        // (copied when the palette base is written)
	uint8_t latch[2], zbank, oki_pin7;
	uint8_t pad[4];
	uint32_t frame;
	uint64_t cpu_frac, z80_frac, ym_frac;
	uint32_t oki_acc;
	int32_t oki_hold;
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)
static uint16_t s_buffered_obj[0x400];   // (latched at the vblank; saved too)

static z80 s_z80;
static uint8_t s_zram[0x800];
static const uint8_t *s_zmap[256];
static int s_ym_irq;
static okim6295_t s_oki;
static const uint8_t *s_oki_rom;
static uint32_t s_oki_size;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;

// ---- the QSound boards ----
#define QZ80_CLOCK 8000000
#define QZ80_IRQ 32000           // (8 MHz / 32000: 250 Hz)
#define QS_CLOCK 60000000        // (one sample per 2 x 1248 clocks)
static int s_qs;
static int s_pang3;              // (pang3: the EEPROM at 0x80017a, init_pang3b / init_pang3)
static struct {
	uint8_t shared[2][0x1000];   // the Z80's c000 / f000, the 68000's f18000 / f1e000 (low bytes)
	int32_t zirq_left, zline;
	uint64_t qs_frac;
	uint32_t qs_acc;
	int64_t mcyc;               // (the 68000's cycles: the EEPROM's times)
	uint8_t state_end;
} q;
static qsound_t s_qsound;
static eepser_t s_eeprom;
static uint8_t *s_zop;           // the opcodes decrypted (64 KB, 0x0000 .. 0x7fff)
static const uint8_t *s_araw;    // audiorom_raw (slammast's protection), 0x8000
static uint32_t s_araw_size;

// ---- CPS2 ----
#define CPS2_CLOCK 11800000      // (16 MHz x 0.7375: MAME's clock scale for the wait states not emulated)
static int s_cps2, s_cps2_dead, s_cps2_vol;
static struct {
	uint16_t objram[2][0x1000];
	uint16_t output[6];         // 0x400000: the object output (CPS2_OBJ_BASE, UNK1, PRI, UNK2, XOFFS, YOFFS)
	uint16_t ram66[0x2000];     // 0x660000 .. 0x663fff
	uint16_t ram664;
	uint16_t rcnt[3], rrel[3];  // the raster counters and their reloads
	uint16_t pri_ctrl;          // (CPS2_OBJ_PRI latched at the vblank)
	uint8_t objbank, ipl, zreset, pad;
	int32_t rirq_at;            // (this line's raster IRQ: the cycles into the line, -1 none)
	uint8_t state_end;
} c2;
static uint16_t s_c2obj[0x1000];   // (the object RAM latched at the vblank; saved too)
static uint16_t *s_c2out;          // the output registers (the dead boards': in the RAM at 0xfffff0)
static uint16_t *s_c2op;           // the opcodes decrypted
static void z_start(void);
static void qsound_line(void);
extern void (*rbx_m68k_int_hook)(int level);

// Kabuki (kabuki.cpp)
static int kb_bitswap1(int src, int key, int select)
{
	if (select & (1 << ((key >> 0) & 7))) src = (src & 0xfc) | ((src & 0x01) << 1) | ((src & 0x02) >> 1);
	if (select & (1 << ((key >> 4) & 7))) src = (src & 0xf3) | ((src & 0x04) << 1) | ((src & 0x08) >> 1);
	if (select & (1 << ((key >> 8) & 7))) src = (src & 0xcf) | ((src & 0x10) << 1) | ((src & 0x20) >> 1);
	if (select & (1 << ((key >> 12) & 7))) src = (src & 0x3f) | ((src & 0x40) << 1) | ((src & 0x80) >> 1);
	return src;
}
static int kb_bitswap2(int src, int key, int select)
{
	if (select & (1 << ((key >> 12) & 7))) src = (src & 0xfc) | ((src & 0x01) << 1) | ((src & 0x02) >> 1);
	if (select & (1 << ((key >> 8) & 7))) src = (src & 0xf3) | ((src & 0x04) << 1) | ((src & 0x08) >> 1);
	if (select & (1 << ((key >> 4) & 7))) src = (src & 0xcf) | ((src & 0x10) << 1) | ((src & 0x20) >> 1);
	if (select & (1 << ((key >> 0) & 7))) src = (src & 0x3f) | ((src & 0x40) << 1) | ((src & 0x80) >> 1);
	return src;
}
static int kb_bytedecode(int src, uint32_t swap_key1, uint32_t swap_key2, int xor_key, int select)
{
	src = kb_bitswap1(src, (int)(swap_key1 & 0xffff), select & 0xff);
	src = ((src & 0x7f) << 1) | ((src & 0x80) >> 7);
	src = kb_bitswap2(src, (int)(swap_key1 >> 16), select & 0xff);
	src ^= xor_key;
	src = ((src & 0x7f) << 1) | ((src & 0x80) >> 7);
	src = kb_bitswap2(src, (int)(swap_key2 & 0xffff), select >> 8);
	src = ((src & 0x7f) << 1) | ((src & 0x80) >> 7);
	src = kb_bitswap1(src, (int)(swap_key2 >> 16), select >> 8);
	return src;
}
// cps1_decode: 0x0000 .. 0x7fff, the opcodes to dst, the data in place
static void kabuki_cps1(uint8_t *src, uint8_t *dst, uint32_t swap_key1, uint32_t swap_key2, int addr_key, int xor_key)
{
	for (int a = 0; a < 0x8000; a++) {
		dst[a] = (uint8_t)kb_bytedecode(src[a], swap_key1, swap_key2, xor_key, a + addr_key);
		src[a] = (uint8_t)kb_bytedecode(src[a], swap_key1, swap_key2, xor_key, ((a ^ 0x1fc0) + addr_key + 1));
	}
}

// ---- the snapshot drawn ----
// the registers a line is drawn with: those at MAME's next partial update after it (a raster IRQ, else the vblank)
typedef struct {
	int32_t scrollx[3], scrolly[3];
	uint16_t layercontrol, videocontrol, rowscroll_offs;
	uint8_t enable[3], pad;
} line_regs;
static line_regs s_lines[VIS_H];
static int s_line_next;   // (the next screen line latched)
static struct {
	uint16_t scroll[3][0x2000];    // the three tilemaps' RAM (0x4000 bytes each)
	uint16_t other[0x400];
	uint16_t obj[0x400];
	uint16_t palraw[0xc00];
	line_regs line[VIS_H];
	uint16_t prio[4];
	int pal_dirty;
	// CPS2
	uint16_t obj2[0x1000];
	uint16_t pri_ctrl, xoffs, yoffs;
} *s_snap;
static uint32_t s_n8, s_n16, s_n32;   // elements of the decoded graphics
static uint16_t s_pal_conv[0xc00];
static int s_pal_valid;

// ---- graphics: groups of 4 bytes (planes 24, 16, 8, 0) to 8 pixels, 4 bpp (the even pixel in the low nibble) ----
static void decode_gfx(void)
{
	for (uint32_t i = 0; i + 4 <= s_gfx_size; i += 4) {
		const uint8_t b0 = s_gfx[i], b1 = s_gfx[i + 1], b2 = s_gfx[i + 2], b3 = s_gfx[i + 3];
		uint8_t o[4];
		for (int x = 0; x < 8; x += 2) {
			uint8_t px[2];
			for (int k = 0; k < 2; k++) {
				const int bit = 7 - (x + k);
				px[k] = (uint8_t)(((b3 >> bit) & 1) << 3 | ((b2 >> bit) & 1) << 2 | ((b1 >> bit) & 1) << 1 | ((b0 >> bit) & 1));
			}
			o[x >> 1] = (uint8_t)(px[0] | px[1] << 4);
		}
		memcpy(s_gfx + i, o, 4);
	}
	s_n8 = s_gfx_size / 64;
	s_n16 = s_gfx_size / 128;
	s_n32 = s_gfx_size / 512;
}
// 8 pixels of a row from 4 bytes
static inline uint8_t gpix(uint32_t byteofs, int x) { const uint8_t v = s_gfx[byteofs + (uint32_t)(x >> 1)]; return (x & 1) ? (v >> 4) : (v & 15); }

enum { GT_SPRITES = 1, GT_SCROLL1 = 2, GT_SCROLL2 = 4, GT_SCROLL3 = 8, GT_STARS = 16 };
static int bank_mapper(int type, int code)
{
	const struct gfx_range *range = s_cfg->bank_mapper;
	int shift = 0;
	switch (type) {
	case GT_SPRITES: shift = 1; break;
	case GT_SCROLL1: shift = 0; break;
	case GT_SCROLL2: shift = 1; break;
	case GT_SCROLL3: shift = 3; break;
	}
	code <<= shift;
	for (; range && range->type; range++)
		if (code >= range->start && code <= range->end && (range->type & type)) {
			int base = 0;
			for (int i = 0; i < range->bank; i++)
				base += s_cfg->bank_sizes[i];
			return (base + (code & (s_cfg->bank_sizes[range->bank] - 1))) >> shift;
		}
	return -1;
}

static uint32_t cps1_base(int reg, uint32_t boundary)   // a word offset into the graphics RAM
{
	uint32_t base = (uint32_t)b.cps_a[reg] * 256;
	base &= ~(boundary - 1);
	return (base & 0x3ffff) / 2;
}

// the screen lines up to upto (inclusive) not latched yet: with the registers now
static void latch_lines(int upto)
{
	if (upto > VIS_Y0 + VIS_H - 1)
		upto = VIS_Y0 + VIS_H - 1;
	if (upto < s_line_next)
		return;
	line_regs r;
	r.scrollx[0] = b.cps_a[A_S1X]; r.scrolly[0] = b.cps_a[A_S1Y];
	r.scrollx[1] = b.cps_a[A_S2X]; r.scrolly[1] = b.cps_a[A_S2Y];
	r.scrollx[2] = b.cps_a[A_S3X]; r.scrolly[2] = b.cps_a[A_S3Y];
	const uint16_t lc = b.cps_b[s_cfg->layer_control / 2], vc = b.cps_a[A_VIDEOCONTROL];
	r.layercontrol = lc;
	r.videocontrol = vc;
	r.rowscroll_offs = b.cps_a[A_ROWSCROLL];
	r.enable[0] = (lc & s_cfg->layer_enable_mask[0]) != 0;
	r.enable[1] = (lc & s_cfg->layer_enable_mask[1]) && ((vc >> 2) & 1);
	r.enable[2] = (lc & s_cfg->layer_enable_mask[2]) && ((vc >> 3) & 1);
	r.pad = 0;
	for (int y = s_line_next < VIS_Y0 ? VIS_Y0 : s_line_next; y <= upto; y++)
		s_lines[y - VIS_Y0] = r;
	s_line_next = upto + 1;
}

static void build_palette(void)   // cps1_build_palette: the pages enabled, from the palette base
{
	const uint32_t base = cps1_base(A_PALETTE, 0x400);
	const uint16_t ctrl = s_cfg->palette_control >= 0 ? b.cps_b[s_cfg->palette_control / 2] : 0x3f;
	uint32_t src = base;
	for (int page = 0; page < 6; page++) {
		if ((ctrl >> page) & 1) {
			for (int o = 0; o < 0x200; o++)
				b.palraw[0x200 * page + o] = b.gfxram[(src++) & 0x1ffff];
		} else if (src != base)
			src += 0x200;
	}
}

// ---- the 68000's devices ----
static uint16_t cps_b_r(int offset)
{
	const struct CPS1config *c = s_cfg;
	// (as MAME: an unused address, -1, compares as -1 / 2 = 0: the register 0 reads the ID or the product)
	if (offset == c->cpsb_addr / 2)
		return (uint16_t)c->cpsb_value;
	if (offset == c->mult_result_lo / 2)
		return (uint16_t)(((uint32_t)b.cps_b[c->mult_factor1 / 2] * b.cps_b[c->mult_factor2 / 2]) & 0xffff);
	if (offset == c->mult_result_hi / 2)
		return (uint16_t)(((uint32_t)b.cps_b[c->mult_factor1 / 2] * b.cps_b[c->mult_factor2 / 2]) >> 16);
	if (c->in2_addr && offset == c->in2_addr / 2)
		return s_port_in2 >= 0 ? (uint16_t)rbx_port_read(s_port_in2) : 0xffff;
	if (c->in3_addr && offset == c->in3_addr / 2)
		return s_port_in3 >= 0 ? (uint16_t)rbx_port_read(s_port_in3) : 0xffff;
	return 0xffff;
}

static uint16_t main_r16(uint32_t a)
{
	a &= 0xfffffe;
	if (a >= 0x800000 && a <= 0x800007)
		return (uint16_t)rbx_port_read(s_port_in1);
	if (a >= 0x800018 && a <= 0x80001f) {   // cps1_dsw_r
		const int o = (int)((a - 0x800018) >> 1);
		const int p = o == 0 ? s_port_in0 : s_port_dsw[o - 1];
		const int in = p >= 0 ? (int)(rbx_port_read(p) & 0xff) : 0xff;
		return (uint16_t)(in << 8 | 0xff);
	}
	if (s_pang3 && a == 0x80017a)   // EEPROMIN
		return (uint16_t)(eepser_do_read_93cxx(&s_eeprom, q.mcyc + m68k_cycles_run()) & 1);
	if (a >= s_cpsb_base && a < s_cpsb_base + 0x40)
		return cps_b_r((int)((a - s_cpsb_base) >> 1));
	if (s_qs && a >= 0xf00000) {   // qsound_main_map
		if (a < 0xf10000) {   // qsound_rom_r
			const uint32_t o = (a - 0xf00000) >> 1;
			return s_araw && o < s_araw_size ? (uint16_t)(s_araw[o] | 0xff00) : 0;
		}
		if (a >= 0xf18000 && a < 0xf1a000)
			return (uint16_t)(q.shared[0][(a - 0xf18000) >> 1] | 0xff00);
		if (a >= 0xf1e000)
			return (uint16_t)(q.shared[1][(a - 0xf1e000) >> 1] | 0xff00);
		if (a == 0xf1c000)
			return s_port_in2 >= 0 ? (uint16_t)rbx_port_read(s_port_in2) : 0xffff;
		if (a == 0xf1c002)
			return s_port_in3 >= 0 ? (uint16_t)rbx_port_read(s_port_in3) : 0xffff;
		if (a == 0xf1c006)   // EEPROMIN: bit 0 the DO
			return (uint16_t)(eepser_do_read_93cxx(&s_eeprom, q.mcyc + m68k_cycles_run()) & 1);
	}
	return 0xffff;
}
static uint8_t main_r8(uint32_t a)
{
	const uint16_t w = main_r16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}
static void main_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xfffffe;
	if (a >= 0x800100 && a < 0x800140) {   // CPS-A
		const int o = (int)((a - 0x800100) >> 1);
		b.cps_a[o] = (uint16_t)((v & mask) | (b.cps_a[o] & ~mask));
		if (o == A_PALETTE)
			build_palette();
	} else if (s_pang3 && a == 0x80017a) {   // EEPROMOUT: DI (bit 0), CLK (6), CS (7)
		if (mask & 0xff) {
			const int64_t t = q.mcyc + m68k_cycles_run();
			eepser_di_write(&s_eeprom, v & 1);
			eepser_clk_write(&s_eeprom, (v >> 6) & 1, t);
			eepser_cs_write(&s_eeprom, (v >> 7) & 1, t);
		}
	} else if (a >= s_cpsb_base && a < s_cpsb_base + 0x40) {
		const int o = (int)((a - s_cpsb_base) >> 1);
		b.cps_b[o] = (uint16_t)((v & mask) | (b.cps_b[o] & ~mask));
	} else if (s_qs && a >= 0xf18000) {
		if (!(mask & 0xff))
			return;
		if (a < 0xf1a000)
			q.shared[0][(a - 0xf18000) >> 1] = (uint8_t)v;
		else if (a >= 0xf1e000)
			q.shared[1][(a - 0xf1e000) >> 1] = (uint8_t)v;
		else if (a == 0xf1c006) {   // EEPROMOUT: DI (bit 0), CLK (6), CS (7)
			const int64_t t = q.mcyc + m68k_cycles_run();
			eepser_di_write(&s_eeprom, v & 1);
			eepser_clk_write(&s_eeprom, (v >> 6) & 1, t);
			eepser_cs_write(&s_eeprom, (v >> 7) & 1, t);
		}
	} else if (a >= 0x800180 && a <= 0x800187)   // the sound command (either byte)
		b.latch[0] = (mask & 0xff) ? (uint8_t)v : (uint8_t)(v >> 8);
	else if (a >= 0x800188 && a <= 0x80018f) {   // the sound timer fade
		if (mask & 0xff)
			b.latch[1] = (uint8_t)v;
	}
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v)
{
	main_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00);
}

// ---- CPS2: the interrupts (IPL1 the vblank, IPL2 the raster: levels 2, 4, 6; both cleared by the acknowledge) ----
static void c2_update_ipl(void)
{
	const int level = c2.ipl & 6;
	m68k_bus_cur->irq_asserted = level ? 1u << level : 0;
	m68k_bus_update_irq();
}
static void c2_int_taken(int level)   // irqack_r
{
	(void)level;
	c2.ipl = 0;
	c2_update_ipl();
}
static int64_t s_c2_line0;   // (the 68000's cycles at the start of the line: the raster counter's horizontal position)
static int s_c2_line_cycles;

static uint16_t c2_cps_b_r(int offset)
{
	const struct CPS1config *c = s_cfg;
	if (offset == c->cpsb_addr / 2)
		return (uint16_t)c->cpsb_value;
	if (offset == c->mult_result_lo / 2)
		return (uint16_t)(((uint32_t)b.cps_b[c->mult_factor1 / 2] * b.cps_b[c->mult_factor2 / 2]) & 0xffff);
	if (offset == c->mult_result_hi / 2)
		return (uint16_t)(((uint32_t)b.cps_b[c->mult_factor1 / 2] * b.cps_b[c->mult_factor2 / 2]) >> 16);
	if (offset == 0x0e / 2) {   // the 2-pixel horizontal position against raster counter 3
		int hpos = 0;
		if (s_c2_line_cycles > 0)
			hpos = (int)((q.mcyc + m68k_cycles_run() - s_c2_line0) * 512 / s_c2_line_cycles);
		if (hpos > 511)
			hpos = 511;
		return (uint16_t)((((c2.rcnt[2] - hpos / 2) << 1) & 0x1fe) | (b.cps_b[0x0e / 2] & 1));
	}
	if (offset == 0x10 / 2 || offset == 0x12 / 2)
		return c2.rcnt[offset & 1] & 0x1ff;
	return 0xffff;
}
static void c2_cps_b_w(int offset, uint16_t v, uint16_t mask)
{
	b.cps_b[offset] = (uint16_t)((v & mask) | (b.cps_b[offset] & ~mask));
	const uint16_t data = b.cps_b[offset];
	if (offset == 0x0e / 2)
		c2.rrel[2] = (data >> 1) & 0xff;
	else if (offset == 0x10 / 2 || offset == 0x12 / 2) {
		c2.rrel[offset & 1] = data & 0x1ff;
		if (data & 0x8000)   // (reloaded by hand)
			c2.rcnt[offset & 1] = c2.rrel[offset & 1];
	}
}

static const uint16_t c2_vol_states[40] = {
	0xf010, 0xf008, 0xf004, 0xf002, 0xf001, 0xe810, 0xe808, 0xe804, 0xe802, 0xe801,
	0xe410, 0xe408, 0xe404, 0xe402, 0xe401, 0xe210, 0xe208, 0xe204, 0xe202, 0xe201,
	0xe110, 0xe108, 0xe104, 0xe102, 0xe101, 0xe090, 0xe088, 0xe084, 0xe082, 0xe081,
	0xe050, 0xe048, 0xe044, 0xe042, 0xe041, 0xe030, 0xe028, 0xe024, 0xe022, 0xe021
};

// cps2_map (cps2_base_map)
static uint16_t c2_r16(uint32_t a)
{
	a &= 0xfffffe;
	if (a >= 0x400000 && a < 0x40000c)
		return s_c2out[(a - 0x400000) >> 1];
	if (a >= 0x618000 && a < 0x61a000)
		return (uint16_t)(q.shared[0][(a - 0x618000) >> 1] | 0xff00);
	if (a == 0x664000)
		return c2.ram664;
	if (a >= 0x708000 && a < 0x710000)
		return c2.objram[(c2.objbank ^ 1) & 1][(a & 0x1fff) >> 1];
	if (s_cps2_dead && a >= 0xfffff0 && a < 0xfffffc)
		return s_c2out[(a - 0xfffff0) >> 1];
	if (a >= 0x800140 && a < 0x800180)
		return c2_cps_b_r((int)((a - 0x800140) >> 1));
	if (a >= 0x804140 && a < 0x804180)
		return c2_cps_b_r((int)((a - 0x804140) >> 1));
	switch (a) {
	case 0x804000: return s_port_in0 >= 0 ? (uint16_t)rbx_port_read(s_port_in0) : 0xffff;
	case 0x804010: return s_port_in1 >= 0 ? (uint16_t)rbx_port_read(s_port_in1) : 0xffff;
	case 0x804020:   // IN2, bit 0 the EEPROM's DO
		if (s_port_in2 < 0)
			return 0xffff;
		rbx_port_custom(s_port_in2, 1, (uint32_t)eepser_do_read_93cxx(&s_eeprom, q.mcyc + m68k_cycles_run()));
		return (uint16_t)rbx_port_read(s_port_in2);
	case 0x804030: return s_cps2_vol ? (uint16_t)s_cps2_vol : c2_vol_states[39];
	case 0x8040b0: case 0x8040b2: {
		const int p = s_port_dsw[(a - 0x8040b0) >> 1];
		return p >= 0 ? (uint16_t)(rbx_port_read(p) & 0xff) : 0xff;
	}
	}
	return 0;
}
static uint8_t c2_r8(uint32_t a)
{
	if (a >= 0x8040b0 && a <= 0x8040b2) {   // (the DIP switches: a byte each)
		const int p = s_port_dsw[a - 0x8040b0];
		return p >= 0 ? (uint8_t)rbx_port_read(p) : 0xff;
	}
	const uint16_t w = c2_r16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}
static void c2_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xfffffe;
	if (a >= 0x400000 && a < 0x40000c) {
		uint16_t *p = &s_c2out[(a - 0x400000) >> 1];
		*p = (uint16_t)((v & mask) | (*p & ~mask));
	} else if (a >= 0x618000 && a < 0x61a000) {
		if (mask & 0xff)
			q.shared[0][(a - 0x618000) >> 1] = (uint8_t)v;
	} else if (a == 0x664000)
		c2.ram664 = (uint16_t)((v & mask) | (c2.ram664 & ~mask));
	else if (a >= 0x700000 && a < 0x702000) {
		uint16_t *p = &c2.objram[c2.objbank & 1][(a - 0x700000) >> 1];
		*p = (uint16_t)((v & mask) | (*p & ~mask));
	} else if (a >= 0x708000 && a < 0x710000) {
		uint16_t *p = &c2.objram[(c2.objbank ^ 1) & 1][(a & 0x1fff) >> 1];
		*p = (uint16_t)((v & mask) | (*p & ~mask));
	} else if (s_cps2_dead && a >= 0xfffff0 && a < 0xfffffc) {
		uint16_t *p = &s_c2out[(a - 0xfffff0) >> 1];
		*p = (uint16_t)((v & mask) | (*p & ~mask));
	} else if ((a >= 0x800100 && a < 0x800140) || (a >= 0x804100 && a < 0x804140)) {   // CPS-A
		const int o = (int)((a & 0x3f) >> 1);
		b.cps_a[o] = (uint16_t)((v & mask) | (b.cps_a[o] & ~mask));
		if (o == A_PALETTE)
			build_palette();
	} else if ((a >= 0x800140 && a < 0x800180) || (a >= 0x804140 && a < 0x804180))   // CPS-B
		c2_cps_b_w((int)(((a & 0x7f) - 0x40) >> 1), v, mask);
	else if (a == 0x804040) {   // cps2_eeprom_port_w
		if (mask & 0xff00) {   // EEPROMOUT: DI (bit 12), CLK (13), CS (14)
			const int64_t t = q.mcyc + m68k_cycles_run();
			eepser_di_write(&s_eeprom, (v >> 12) & 1);
			eepser_clk_write(&s_eeprom, (v >> 13) & 1, t);
			eepser_cs_write(&s_eeprom, (v >> 14) & 1, t);
		}
		if (mask & 0xff) {   // bit 3: the Z80 out of reset
			const int held = !((v >> 3) & 1);
			if (held && !c2.zreset)
				z_start();   // (MAME resets the CPU when its reset line is asserted)
			c2.zreset = (uint8_t)held;
		}
	} else if (a == 0x8040e0) {
		if (mask & 0xff)
			c2.objbank = v & 1;
	}
}
static void c2_w16(uint32_t a, uint16_t v) { c2_w16m(a, v, 0xffff); }
static void c2_w8(uint32_t a, uint8_t v)
{
	c2_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00);
}

// ---- the Z80 (sub_map) ----
static void z_bank(uint8_t v)
{
	if (s_qs) {   // qsound_banksw_w
		int bank = v & 0x0f;
		if (0x10000 + (uint32_t)bank * 0x4000 >= s_zrom_size)
			bank = 0;
		b.zbank = (uint8_t)bank;
	} else
		b.zbank = v & 1;
	const uint32_t base = 0x10000 + (uint32_t)b.zbank * 0x4000;
	for (int p = 0; p < 0x40; p++)
		s_zmap[0x80 + p] = base + (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + base + (uint32_t)p * 256 : NULL;
}
static uint8_t z_read(void *u, uint16_t a)
{
	(void)u;
	const uint8_t *p = s_zmap[a >> 8];
	if (p)
		return p[a & 0xff];
	if (a >= 0xd000 && a < 0xd800)
		return s_zram[a - 0xd000];
	switch (a) {
	case 0xf000: case 0xf001: return rbx_ym_read(0, a & 1);
	case 0xf002: return oki_read(&s_oki);
	case 0xf008: return b.latch[0];
	case 0xf00a: return b.latch[1];
	}
	return 0xff;
}
static void z_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xd000 && a < 0xd800)
		s_zram[a - 0xd000] = v;
	else if (a == 0xf000 || a == 0xf001)
		rbx_ym_write(0, a & 1, v);
	else if (a == 0xf002)
		oki_write(&s_oki, v);
	else if (a == 0xf004)
		z_bank(v);
	else if (a == 0xf006) {   // the OKI's pin 7
		b.oki_pin7 = v & 1;
		s_oki.rate = 1000000 / (b.oki_pin7 ? 132 : 165);
	}
}
static uint8_t qz_read(void *u, uint16_t a)
{
	(void)u;
	const uint8_t *p = s_zmap[a >> 8];
	if (p)
		return p[a & 0xff];
	if (a == 0xd007)
		return qsound_r(&s_qsound);
	return 0xff;
}
static void qz_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xd000)
		q.shared[0][a - 0xc000] = v;
	else if (a >= 0xf000)
		q.shared[1][a - 0xf000] = v;
	else if (a <= 0xd002 && a >= 0xd000)
		qsound_w(&s_qsound, a - 0xd000, v);
	else if (a == 0xd003)
		z_bank(v);
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }
static void ym_irq(int state) { s_ym_irq = state; }
static void z_start(void)
{
	z80_init(&s_z80);
	s_z80.read_byte = s_qs ? qz_read : z_read;
	s_z80.write_byte = s_qs ? qz_write : z_write;
	s_z80.port_in = z_in;
	s_z80.port_out = z_out;
	s_z80.read_map = s_zmap;
	s_z80.op_rom = s_qs ? s_zop : NULL;
}

// ---- drawing ----
#define BANDS 8
static int s_band_next, s_band_left, s_band_active, s_band_helpers;

// a tilemap's line in runs of a tile: pens (15 transparent), colour bases and priority groups of the shown pixels
static void tile_line(int layer, int Y, uint8_t *pen, uint16_t *cbase, uint8_t *grp)
{
	const uint16_t *ram = s_snap->scroll[layer];
	const line_regs *L = &s_snap->line[Y - VIS_Y0];
	const int size = 8 << layer, mapw = 512 << layer, smask = size - 1;
	int sx = L->scrollx[layer];
	if (layer == 1 && (L->videocontrol & 1))   // (line scroll)
		sx += s_snap->other[(Y + L->rowscroll_offs) & 0x3ff];
	const int ly = (Y + L->scrolly[layer]) & (mapw - 1);
	const int r = ly / size;
	for (int x = 0; x < VIS_W;) {
		const int lx = (VIS_X0 + x + sx) & (mapw - 1);
		const int c = lx / size;
		int n = size - (lx & smask);
		if (n > VIS_W - x)
			n = VIS_W - x;
		int idx, code;
		if (layer == 0) {
			idx = (r & 0x1f) + ((c & 0x3f) << 5) + ((r & 0x20) << 6);
			code = bank_mapper(GT_SCROLL1, ram[2 * idx]);
		} else if (layer == 1) {
			idx = (r & 0x0f) + ((c & 0x3f) << 4) + ((r & 0x30) << 6);
			code = bank_mapper(GT_SCROLL2, ram[2 * idx]);
		} else {
			idx = (r & 0x07) + ((c & 0x3f) << 3) + ((r & 0x38) << 6);
			code = bank_mapper(GT_SCROLL3, ram[2 * idx] & 0x3fff);
		}
		const int attr = ram[2 * idx + 1];
		if (code < 0) {
			memset(pen + x, 15, (size_t)n);
			x += n;
			continue;
		}
		int py = ly & smask;
		if (attr & 0x40)
			py = smask - py;
		uint32_t rowofs;
		if (layer == 0)
			rowofs = (uint32_t)(code % (int)s_n8) * 64 + (uint32_t)py * 8 + (uint32_t)(c & 1) * 4;
		else if (layer == 1)
			rowofs = (uint32_t)(code % (int)s_n16) * 128 + (uint32_t)py * 8;
		else
			rowofs = (uint32_t)(code % (int)s_n32) * 512 + (uint32_t)py * 16;
		const uint16_t cb = (uint16_t)(((attr & 0x1f) + 0x20 * (layer + 1)) << 4);
		const uint8_t g = (uint8_t)((attr >> 7) & 3);
		uint8_t row[32];   // (the tile's row, flipped as shown)
		const uint8_t *src = s_gfx + rowofs;
		for (int k = 0; k < size; k += 2) {
			const uint8_t v = src[k >> 1];
			row[k] = v & 15;
			row[k + 1] = v >> 4;
		}
		const int px = lx & smask;
		if (attr & 0x20)
			for (int i = 0; i < n; i++)
				pen[x + i] = row[smask - (px + i)];
		else
			memcpy(pen + x, row + px, (size_t)n);
		for (int i = 0; i < n; i++) {
			cbase[x + i] = cb;
			grp[x + i] = g;
		}
		x += n;
	}
}

// the sprites of a band into a buffer. MAME's prio_transpen: the first sprite drawn on a pixel decides (shown unless
// the priority map masks it) and sets the map to 31, which hides every sprite drawn after it (the mask's bit 31 is
// always on). CPS1 draws the table from its start: drawn here backwards, the buffer keeps the first one's colour.
static uint16_t s_scol[VIS_W * VIS_H];
static uint8_t s_scnt[VIS_W * VIS_H];

static void draw_one(uint32_t code, int col, int flipx, int flipy, int sx, int sy, int y0, int y1)
{
	if (code == (uint32_t)-1 || !s_n16)
		return;
	const uint32_t base = (code % s_n16) * 128;
	for (int yy = 0; yy < 16; yy++) {
		const int dy = sy + yy - VIS_Y0;
		if (dy < y0 || dy >= y1)
			continue;
		const int py = flipy ? 15 - yy : yy;
		uint16_t *sc = s_scol + dy * VIS_W;
		uint8_t *cn = s_scnt + dy * VIS_W;
		for (int xx = 0; xx < 16; xx++) {
			const int dx = sx + xx - VIS_X0;
			if (dx < 0 || dx >= VIS_W)
				continue;
			const int px = flipx ? 15 - xx : xx;
			const uint8_t pen = gpix(base + (uint32_t)py * 8 + (uint32_t)(px >> 3) * 4, px & 7);
			if (pen == 15)
				continue;
			sc[dx] = (uint16_t)((col << 4) + pen);
			cn[dx] = 1;
		}
	}
}

static void draw_sprites(int y0, int y1)
{
	memset(s_scnt + y0 * VIS_W, 0, (size_t)VIS_W * (size_t)(y1 - y0));
	const uint16_t *obj = s_snap->obj;
	int last = 0x400 - 4;
	for (int o = 0; o < 0x400; o += 4)
		if ((obj[o + 3] & 0xff00) == 0xff00) {
			last = o - 4;
			break;
		}
	for (int o = last; o >= 0; o -= 4) {
		const int x = obj[o], y = obj[o + 1];
		const int colour = obj[o + 3], col = colour & 0x1f;
		const int flipx = (colour >> 5) & 1, flipy = (colour >> 6) & 1;
		const int code = bank_mapper(GT_SPRITES, obj[o + 2]);
		if (code == -1)
			continue;
		if (colour & 0xff00) {
			const int nx = ((colour & 0x0f00) >> 8) + 1, ny = ((colour & 0xf000) >> 12) + 1;
			for (int nys = 0; nys < ny; nys++) {
				const int sy = (y + nys * 16) & 0x1ff;
				if (sy - VIS_Y0 + 16 <= y0 || sy - VIS_Y0 >= y1)   // (not in the band: draw_one clips the rest)
					continue;
				for (int nxs = 0; nxs < nx; nxs++) {
					const int sx = (x + nxs * 16) & 0x1ff;
					const int cx = flipx ? ((code + (nx - 1) - nxs) & 0xf) : ((code + nxs) & 0xf);
					const int cy = flipy ? (ny - 1 - nys) : nys;
					draw_one((uint32_t)((code & ~0xf) + cx + 0x10 * cy), col, flipx, flipy, sx, sy, y0, y1);
				}
			}
		} else
			draw_one((uint32_t)code, col, flipx, flipy, x & 0x1ff, y & 0x1ff, y0, y1);
	}
}

// ---- CPS2: cps2_render_sprites into the band's buffer: MAME draws the table from its end, the first sprite drawn on
// a pixel decides (prio_transpen, see draw_one): drawn here from the start, the buffer keeps its colour and priority
static void draw_one2(uint32_t code, int col, int flipx, int flipy, int sx, int sy, int prio, int y0, int y1)
{
	if (!s_n16)
		return;
	const uint32_t base = (code % s_n16) * 128;
	for (int yy = 0; yy < 16; yy++) {
		const int dy = sy + yy - VIS_Y0;
		if (dy < y0 || dy >= y1)
			continue;
		const int py = flipy ? 15 - yy : yy;
		uint16_t *sc = s_scol + dy * VIS_W;
		uint8_t *cn = s_scnt + dy * VIS_W;
		for (int xx = 0; xx < 16; xx++) {
			const int dx = sx + xx - VIS_X0;
			if (dx < 0 || dx >= VIS_W)
				continue;
			const int px = flipx ? 15 - xx : xx;
			const uint8_t pen = gpix(base + (uint32_t)py * 8 + (uint32_t)(px >> 3) * 4, px & 7);
			if (pen == 15)
				continue;
			sc[dx] = (uint16_t)((col << 4) + pen);
			cn[dx] = (uint8_t)(1 | prio << 2);
		}
	}
}

static void draw_sprites2(int y0, int y1)
{
	memset(s_scnt + y0 * VIS_W, 0, (size_t)VIS_W * (size_t)(y1 - y0));
	const uint16_t *base = s_snap->obj2;
	int last = 0x1000 - 4;   // find_last_sprite
	for (int o = 0; o < 0x1000; o += 4)
		if (base[o + 1] >= 0x8000 || base[o + 3] >= 0xff00) {
			last = o - 4;
			break;
		}
	const int xoffs = 64 - s_snap->xoffs, yoffs = 16 - s_snap->yoffs;
	for (int i = 0; i <= last; i += 4) {
		int x = base[i + 0], y = base[i + 1];
		const int priority = (x >> 13) & 7;
		const int code = base[i + 2] + ((y & 0x6000) << 3);
		const int colour = base[i + 3], col = colour & 0x1f;
		const int flipx = (colour >> 5) & 1, flipy = (colour >> 6) & 1;
		if (colour & 0x80) {
			x += s_snap->xoffs;
			y += s_snap->yoffs;
		}
		if (colour & 0xff00) {
			const int nx = ((colour & 0x0f00) >> 8) + 1, ny = ((colour & 0xf000) >> 12) + 1;
			for (int nys = 0; nys < ny; nys++) {
				const int sy = (y + nys * 16 + yoffs) & 0x3ff;
				if (sy - VIS_Y0 + 16 <= y0 || sy - VIS_Y0 >= y1)
					continue;
				for (int nxs = 0; nxs < nx; nxs++) {
					const int sx = (x + nxs * 16 + xoffs) & 0x3ff;
					int c;
					if (flipy)
						c = flipx ? code + (nx - 1) - nxs + 0x10 * (ny - 1 - nys) : code + nxs + 0x10 * (ny - 1 - nys);
					else
						c = flipx ? code + (nx - 1) - nxs + 0x10 * nys : (code & ~0xf) + ((code + nxs) & 0xf) + 0x10 * nys;
					draw_one2((uint32_t)c, col, flipx, flipy, sx, sy, priority, y0, y1);
				}
			}
		} else
			draw_one2((uint32_t)code, col, flipx, flipy, (x + xoffs) & 0x3ff, (y + yoffs) & 0x3ff, priority, y0, y1);
	}
}

// cps2 render_layers: the three tilemaps in the CPS-B order with the priority bits 1, 2, 4, then the sprites
static void run_band2(int y0, int y1)
{
	draw_sprites2(y0, y1);
	uint8_t pen[VIS_W], grp[VIS_W], pri[VIS_W];
	uint16_t cb[VIS_W];
	const uint16_t pc = s_snap->pri_ctrl;
	for (int y = y0; y < y1; y++) {
		const line_regs *L = &s_snap->line[y];
		const uint16_t lc = L->layercontrol;
		int l0 = (lc >> 6) & 3, l1 = (lc >> 8) & 3, l2 = (lc >> 10) & 3, l3 = (lc >> 12) & 3;
		int l0pri = (pc >> 4 * l0) & 15, l1pri = (pc >> 4 * l1) & 15, l2pri = (pc >> 4 * l2) & 15, l3pri = (pc >> 4 * l3) & 15;
		if (l0 == 0) { l0 = l1; l1 = 0; l0pri = l1pri; }
		if (l1 == 0) { l1 = l2; l2 = 0; l1pri = l2pri; }
		if (l2 == 0) { l2 = l3; l3 = 0; l2pri = l3pri; }
		uint32_t primasks[8];
		{
			uint32_t mask0 = 0xaa, mask1 = 0xcc;
			if (l0pri > l1pri) mask0 &= ~0x88u;
			if (l0pri > l2pri) mask0 &= ~0xa0u;
			if (l1pri > l2pri) mask1 &= ~0xc0u;
			primasks[0] = 0xff;
			for (int i = 1; i < 8; i++) {
				if (i <= l0pri && i <= l1pri && i <= l2pri) {
					primasks[i] = 0xfe;
					continue;
				}
				primasks[i] = 0;
				if (i <= l0pri) primasks[i] |= mask0;
				if (i <= l1pri) primasks[i] |= mask1;
				if (i <= l2pri) primasks[i] |= 0xf0;
			}
		}
		uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
		for (int x = 0; x < VIS_W; x++)
			d[x] = 0xbff;
		memset(pri, 0, sizeof(pri));
		const int lay[3] = { l0, l1, l2 };
		for (int k = 0; k < 3; k++) {
			const int l = lay[k];
			if (l < 1 || !L->enable[l - 1])
				continue;
			tile_line(l - 1, VIS_Y0 + y, pen, cb, grp);
			const uint8_t bit = (uint8_t)(1 << k);
			for (int x = 0; x < VIS_W; x++)
				if (pen[x] != 15) {
					d[x] = (uint16_t)(cb[x] + pen[x]);
					pri[x] |= bit;
				}
		}
		const uint16_t *sc = s_scol + y * VIS_W;
		const uint8_t *cn = s_scnt + y * VIS_W;
		for (int x = 0; x < VIS_W; x++) {
			const int c = cn[x];
			if (!c)
				continue;
			if (!((primasks[c >> 2] >> pri[x]) & 1))
				d[x] = sc[x];
		}
	}
}

static void run_band(int band)
{
	const int y0 = VIS_H * band / BANDS, y1 = VIS_H * (band + 1) / BANDS;
	if (s_cps2) {
		run_band2(y0, y1);
		return;
	}
	int spr = 0;
	for (int y = y0; y < y1 && !spr; y++) {
		const uint16_t lc = s_snap->line[y].layercontrol;
		for (int i = 0; i < 4; i++)
			if (!((lc >> (6 + 2 * i)) & 3))
				spr = 1;
	}
	if (spr)
		draw_sprites(y0, y1);
	uint8_t pen[3][VIS_W], grp[3][VIS_W], masked[VIS_W];
	uint16_t cb[3][VIS_W];
	for (int y = y0; y < y1; y++) {
		const line_regs *L = &s_snap->line[y];
		const uint16_t lc = L->layercontrol;
		const int order[4] = { (lc >> 6) & 3, (lc >> 8) & 3, (lc >> 10) & 3, (lc >> 12) & 3 };
		int need[3] = { 0, 0, 0 };
		for (int i = 0; i < 4; i++)
			if (order[i])
				need[order[i] - 1] = L->enable[order[i] - 1];
		for (int l = 0; l < 3; l++)
			if (need[l])
				tile_line(l, VIS_Y0 + y, pen[l], cb[l], grp[l]);
		uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
		for (int x = 0; x < VIS_W; x++)
			d[x] = 0xbff;
		memset(masked, 0, sizeof(masked));
		const uint16_t *sc = s_scol + y * VIS_W;
		const uint8_t *cn = s_scnt + y * VIS_W;
		for (int i = 0; i < 4; i++) {
			const int l = order[i];
			if (l == 0) {   // the sprites, then the priority map 0x1f under them
				for (int x = 0; x < VIS_W; x++)
					if (cn[x]) {
						if (!masked[x])
							d[x] = sc[x];
						masked[x] = 0;
					}
				continue;
			}
			if (!need[l - 1])
				continue;
			const uint8_t *pn = pen[l - 1];
			const uint16_t *c = cb[l - 1];
			for (int x = 0; x < VIS_W; x++)
				if (pn[x] != 15)
					d[x] = (uint16_t)(c[x] + pn[x]);
			if (i < 3 && order[i + 1] == 0) {   // (the sprites next: this layer's masked pens over them)
				const uint8_t *g = grp[l - 1];
				for (int x = 0; x < VIS_W; x++)
					if (pn[x] != 15 && ((s_snap->prio[g[x]] >> pn[x]) & 1))
						masked[x] = 1;
			}
		}
	}
}

static void cps1_render(void)
{
	if (s_snap->pal_dirty || !s_pal_valid) {
		for (int i = 0; i < 0xc00; i++) {
			const uint16_t p = s_snap->palraw[i];
			if (s_pal_valid && s_pal_conv[i] == p)
				continue;
			s_pal_conv[i] = p;
			const int bright = 0x0f + ((p >> 12) << 1);
			const int r = ((p >> 8) & 15) * 0x11 * bright / 0x2d, g = ((p >> 4) & 15) * 0x11 * bright / 0x2d,
			          bb = (p & 15) * 0x11 * bright / 0x2d;
			rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | bb);
		}
		s_pal_valid = 1;
		s_snap->pal_dirty = 0;
	}
	__atomic_store_n(&s_band_next, 0, __ATOMIC_SEQ_CST);
	__atomic_store_n(&s_band_left, BANDS, __ATOMIC_SEQ_CST);
	__atomic_store_n(&s_band_active, 1, __ATOMIC_SEQ_CST);
	for (;;) {
		const int bnd = __atomic_fetch_add(&s_band_next, 1, __ATOMIC_ACQ_REL);
		if (bnd >= BANDS)
			break;
		run_band(bnd);
		__atomic_sub_fetch(&s_band_left, 1, __ATOMIC_ACQ_REL);
	}
	while (__atomic_load_n(&s_band_left, __ATOMIC_ACQUIRE) > 0)
		;
	__atomic_store_n(&s_band_active, 0, __ATOMIC_SEQ_CST);
	while (__atomic_load_n(&s_band_helpers, __ATOMIC_ACQUIRE) > 0)
		;
}

static int cps1_render_help(int helper)
{
	const int one = helper & 0x100;
	helper &= 0xff;
	if (helper < 1 || helper > 2 || !__atomic_load_n(&s_band_active, __ATOMIC_SEQ_CST))
		return 0;
	__atomic_add_fetch(&s_band_helpers, 1, __ATOMIC_SEQ_CST);
	int did = 0;
	if (__atomic_load_n(&s_band_active, __ATOMIC_SEQ_CST))
		for (;;) {
			const int bnd = __atomic_fetch_add(&s_band_next, 1, __ATOMIC_ACQ_REL);
			if (bnd >= BANDS)
				break;
			run_band(bnd);
			__atomic_sub_fetch(&s_band_left, 1, __ATOMIC_ACQ_REL);
			did = 1;
			if (one)
				break;
		}
	__atomic_sub_fetch(&s_band_helpers, 1, __ATOMIC_SEQ_CST);
	return did;
}

// the screen drawn at the vblank (with the objects latched at the last one), then the objects latched, the IRQ
static void vblank(void)
{
	rbx_video_wait_idle();
	for (int l = 0; l < 3; l++) {
		const uint32_t base = cps1_base(A_SCROLL1 + l, 0x4000);
		for (int i = 0; i < 0x2000; i++)
			s_snap->scroll[l][i] = b.gfxram[(base + (uint32_t)i) & 0x1ffff];
	}
	{
		const uint32_t base = cps1_base(A_OTHER, 0x800);
		for (int i = 0; i < 0x400; i++)
			s_snap->other[i] = b.gfxram[(base + (uint32_t)i) & 0x1ffff];
	}
	memcpy(s_snap->obj, s_buffered_obj, sizeof(s_buffered_obj));
	if (memcmp(s_snap->palraw, b.palraw, sizeof(b.palraw))) {
		memcpy(s_snap->palraw, b.palraw, sizeof(b.palraw));
		s_snap->pal_dirty = 1;
	}
	latch_lines(VIS_Y0 + VIS_H - 1);
	memcpy(s_snap->line, s_lines, sizeof(s_lines));
	s_line_next = 0;
	for (int i = 0; i < 4; i++)
		s_snap->prio[i] = s_cfg->priority[i] >= 0 ? b.cps_b[s_cfg->priority[i] / 2] : 0;
	if (s_cps2) {
		memcpy(s_snap->obj2, s_c2obj, sizeof(s_c2obj));
		s_snap->pri_ctrl = c2.pri_ctrl;
		s_snap->xoffs = s_c2out[4];
		s_snap->yoffs = s_c2out[5];
	}
	rbx_video_frame_done();
	if (s_cps2) {   // cps2_objram_latch (the bank at 0x7000, or 0x7080 when swapped), then IPL1
		c2.pri_ctrl = s_c2out[2];
		memcpy(s_c2obj, c2.objram[c2.objbank & 1], sizeof(s_c2obj));
		c2.ipl |= 2;
		c2_update_ipl();
		return;
	}
	{	// cps1_objram_latch
		const uint32_t base = cps1_base(A_OBJ, 0x800);
		for (int i = 0; i < 0x400; i++)
			s_buffered_obj[i] = b.gfxram[(base + (uint32_t)i) & 0x1ffff];
	}
	m68k_bus_hold_irq(2);
}

static void sound_line(void)
{
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&b.ym_frac, Z80_CLOCK, FRAME_MHZ, LINES);
	const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	const uint32_t yr = rbx_ym_rate(0);
	for (int i = 0; i < n; i++) {
		b.oki_acc += s_oki.rate;
		while (b.oki_acc >= yr) {
			b.oki_acc -= yr;
			b.oki_hold = oki_sample(&s_oki);
		}
		s_mix[s_mix_n + i] += b.oki_hold;
	}
	s_mix_n += n;
}

static void qsound_frame(void);
static void cps2_frame(void);
static void cps1_frame(void)
{
	if (s_cps2) {
		cps2_frame();
		return;
	}
	if (s_qs) {
		qsound_frame();
		return;
	}
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0 + VIS_H)
			vblank();
		const int mc = rbx_line_cycles(&b.cpu_frac, s_cpu_clock, FRAME_MHZ, LINES);
		m68k_execute(mc);
		q.mcyc += mc;
		const int zc = rbx_line_cycles(&b.z80_frac, Z80_CLOCK, FRAME_MHZ, LINES);
		z80_run(&s_z80, s_z80.cyc + (unsigned long)zc, &s_ym_irq, 0);
		sound_line();
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	b.frame++;
}

static void qsound_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0 + VIS_H)
			vblank();
		const int mc = rbx_line_cycles(&b.cpu_frac, s_cpu_clock, FRAME_MHZ, LINES);
		m68k_execute(mc);
		q.mcyc += mc;
		qsound_line();
	}
	b.frame++;
}

// a line of the QSound Z80 (8 MHz, its 250 Hz IRQ) and of the DSP
static void qsound_line(void)
{
	{
		int zc = rbx_line_cycles(&b.z80_frac, QZ80_CLOCK, FRAME_MHZ, LINES);
		if (s_cps2 && c2.zreset) {   // (held in reset by the EEPROM port)
			q.zirq_left -= zc;
			while (q.zirq_left <= 0)
				q.zirq_left += QZ80_IRQ;
			zc = 0;
		}
		while (zc > 0) {   // the periodic IRQ (irq0_line_hold)
			if (q.zirq_left <= 0) {
				q.zline = 1;
				q.zirq_left += QZ80_IRQ;
			}
			const int n = zc < q.zirq_left ? zc : q.zirq_left;
			const unsigned long c0 = s_z80.cyc;
			z80_run(&s_z80, c0 + (unsigned long)n, &q.zline, 1);
			const int ran = (int)(s_z80.cyc - c0);
			q.zirq_left -= ran;
			zc -= ran;
		}
		q.qs_acc += (uint32_t)rbx_line_cycles(&q.qs_frac, QS_CLOCK, FRAME_MHZ, LINES);
		while (q.qs_acc >= 2496) {
			q.qs_acc -= 2496;
			int32_t l = 0, r = 0;
			qsound_render(&s_qsound, &l, &r, 1);
			rbx_audio_put_stereo(l, r);
		}
	}
}

// CPS2: raster_scanline (the counters, IRQ 4 at the third counter's pixel), the vblank at line 240
static void cps2_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		int pending = 0;
		for (int i = 0; i < 2; i++) {
			if (line == 0)
				c2.rcnt[i] = c2.rrel[i];
			else
				c2.rcnt[i] = (uint16_t)((c2.rcnt[i] - 1) & 0x1ff);
			if (c2.rcnt[i] == 0)
				pending = 1;
		}
		c2.rcnt[2] = c2.rrel[2];
		if (line == VIS_Y0 + VIS_H)
			vblank();
		const int mc = rbx_line_cycles(&b.cpu_frac, s_cpu_clock, FRAME_MHZ, LINES);
		s_c2_line0 = q.mcyc;
		s_c2_line_cycles = mc;
		int done = 0;
		if (pending) {
			int at = (int)((int64_t)c2.rcnt[2] * 2 * mc / 512);
			if (at > mc)
				at = mc;
			if (at > 0)
				done = m68k_execute(at);
			q.mcyc = s_c2_line0 + done;
			latch_lines(line);   // (MAME: update_partial(vpos))
			c2.ipl |= 4;
			c2_update_ipl();
		}
		if (mc - done > 0)
			m68k_execute(mc - done);
		q.mcyc = s_c2_line0 + mc;
		qsound_line();
	}
	b.frame++;
}

static void cps1_reset(void)
{
	memset(&b, 0, offsetof(__typeof__(b), state_end));
	memset(s_buffered_obj, 0, sizeof(s_buffered_obj));
	b.cps_a[A_OBJ] = 0x9200;
	b.cps_a[A_SCROLL1] = 0x9000;
	b.cps_a[A_SCROLL2] = 0x9040;
	b.cps_a[A_SCROLL3] = 0x9080;
	b.cps_a[A_OTHER] = 0x9100;
	b.oki_pin7 = 1;
	memset(s_zram, 0, sizeof(s_zram));
	memset(s_zmap, 0, sizeof(s_zmap));
	for (uint32_t p = 0; p < 0x80 && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	memset(&q, 0, offsetof(__typeof__(q), state_end));
	memset(&c2, 0, offsetof(__typeof__(c2), state_end));
	memset(s_c2obj, 0, sizeof(s_c2obj));
	c2.rrel[0] = c2.rrel[1] = 0x1ff;   // (init_rasters)
	s_line_next = 0;
	if (s_qs) {
		for (int p = 0; p < 0x10; p++) {
			s_zmap[0xc0 + p] = q.shared[0] + p * 256;
			s_zmap[0xf0 + p] = q.shared[1] + p * 256;
		}
		qsound_reset(&s_qsound);
	}
	z_bank(0);
	z_start();
	s_ym_irq = 0;
	rbx_ym_reset(0);
	oki_reset(&s_oki);
	s_oki.rate = 1000000 / 132;
	s_mix_n = 0;
	s_pal_valid = 0;
	m68k_bus_irq_reset();
	if (s_cps2)
		rbx_m68k_int_hook = c2_int_taken;
	m68k_pulse_reset();
}

static void cps1_exit(void)
{
	if (s_qs || s_pang3)
		eepser_nv_save(&s_eeprom);
	free(s_zop);
	s_zop = NULL;
	free(s_c2op);
	s_c2op = NULL;
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	free(s_snap);
	s_snap = NULL;
	free(s_bp);   // (last: the render thread and the saves are done with it)
	s_bp = NULL;
}

static int cps1_supported(const char *machine)
{
	return !strcmp(machine, "cps1_10MHz") || !strcmp(machine, "cps1_12MHz") || !strcmp(machine, "forgottn") ||
	       !strcmp(machine, "qsound") || !strcmp(machine, "pang3") || !strcmp(machine, "cps2") ||
	       !strcmp(machine, "cps2comm") || !strcmp(machine, "dead_cps2") || !strcmp(machine, "dead_cps2comm");
}

static void c2_unshuffle(uint64_t *buf, int len)
{
	if (len == 2)
		return;
	len /= 2;
	c2_unshuffle(buf, len);
	c2_unshuffle(buf + len, len);
	for (int i = 0; i < len / 2; i++) {
		const uint64_t t = buf[len / 2 + i];
		buf[len / 2 + i] = buf[len + i];
		buf[len + i] = t;
	}
}

// init_cps2crypt: the 64-bit key and the range from the "key" ROM, the opcodes decrypted
static int c2_decrypt(const rbx_game *g)
{
	uint32_t ksz;
	const uint8_t *key = rbx_region("key", &ksz);
	free(s_c2op);
	s_c2op = (uint16_t *)malloc(s_rom_size);
	if (!key || ksz < 20 || !s_c2op) {
		rbx_fail("%s: no key", g->set);
		return -1;
	}
	uint16_t decoded[10] = { 0 };
	for (int bb = 0; bb < 10 * 16; bb++) {
		const int bit = (317 - bb) % 160;
		if ((key[bit / 8] >> ((bit ^ 7) % 8)) & 1)
			decoded[bb / 16] |= (uint16_t)(0x8000 >> (bb % 16));
	}
	const uint32_t k[2] = { (uint32_t)decoded[0] << 16 | decoded[1], (uint32_t)decoded[2] << 16 | decoded[3] };
	uint32_t lower, upper;
	if (decoded[9] == 0xffff) {   // (a dead board: only 0xff0000 .. 0xffffff)
		upper = 0xffffff;
		lower = 0xff0000;
	} else {
		upper = (((~decoded[9] & 0x3ffu) << 14) | 0x3fff) + 1;
		lower = 0;
	}
	cps2_decrypt((const uint16_t *)s_rom, s_c2op, (int)s_rom_size, k, lower / 2, upper / 2);
	return 0;
}

static int cps1_init(const rbx_game *g)
{
	if (!s_bp && !(s_bp = (struct cps1_state *)calloc(1, sizeof(*s_bp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!cps1_supported(g->machine)) {
		rbx_fail("%s: machine %s not emulated", g->set, g->machine);
		return -1;
	}
	s_cps2 = !strcmp(g->machine, "cps2") || !strcmp(g->machine, "cps2comm") || !strncmp(g->machine, "dead_cps2", 9);
	s_cps2_dead = !strncmp(g->machine, "dead_cps2", 9);
	s_cfg = NULL;
	for (const struct CPS1config *c = cps1_config_table; c->name; c++)
		if (!strcmp(c->name, s_cps2 ? "cps2" : g->set))
			s_cfg = c;
	if (!s_cfg && g->parent)
		for (const struct CPS1config *c = cps1_config_table; c->name; c++)
			if (!strcmp(c->name, g->parent))
				s_cfg = c;
	if (!s_cfg) {
		rbx_fail("%s: no CPS-B configuration", g->set);
		return -1;
	}
	s_qs = !strcmp(g->machine, "qsound") || s_cps2;
	s_pang3 = !strcmp(g->machine, "pang3");
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_gfx = rbx_region("gfx", &s_gfx_size);
	s_oki_rom = s_qs ? NULL : rbx_region("oki", &s_oki_size);
	uint32_t qsz = 0, dsz = 0;
	const uint8_t *qrom = s_qs ? rbx_region("qsound", &qsz) : NULL;
	const uint8_t *dsp = s_qs ? rbx_region("qsound:dsp", &dsz) : NULL;
	if (!s_rom || !s_zrom || !s_gfx || (s_qs ? (!qrom || !dsp || dsz < 0x2000 || s_zrom_size < 0x8000) : !s_oki_rom)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_araw = s_qs ? rbx_region("audiorom_raw", &s_araw_size) : NULL;
	if (s_cps2) {
		qsound_init(&s_qsound, dsp, qrom, qsz);
		eepser_init(&s_eeprom, EEPSER_93CXX, 64, 16, CPS2_CLOCK);
		uint32_t esz;
		const uint8_t *e = rbx_region("eeprom", &esz);
		if (e)
			eepser_default(&s_eeprom, e, esz);
		eepser_nv_load(&s_eeprom, g->set);
	} else if (s_qs) {   // the Kabuki keys (init_wof, init_dino, init_punisher, init_slammast)
		const char *root = g->parent ? g->parent : g->set;
		uint32_t k1, k2;
		int ak, xk;
		if (!strncmp(root, "wof", 3)) { k1 = 0x01234567; k2 = 0x54163072; ak = 0x5151; xk = 0x51; }
		else if (!strncmp(root, "dino", 4)) { k1 = 0x76543210; k2 = 0x24601357; ak = 0x4343; xk = 0x43; }
		else if (!strncmp(root, "punisher", 8)) { k1 = 0x67452103; k2 = 0x75316024; ak = 0x2222; xk = 0x22; }
		else if (!strcmp(root, "slammast") || !strncmp(root, "mbomb", 5)) { k1 = 0x54321076; k2 = 0x65432107; ak = 0x3131; xk = 0x19; }
		else {
			rbx_fail("%s: no Kabuki key", g->set);
			return -1;
		}
		s_zop = (uint8_t *)calloc(1, 0x10000);
		if (!s_zop) {
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
		kabuki_cps1(s_zrom, s_zop, k1, k2, ak, xk);
		qsound_init(&s_qsound, dsp, qrom, qsz);
		eepser_init(&s_eeprom, EEPSER_93CXX, 128, 8, 12000000);
		uint32_t esz;
		const uint8_t *e = rbx_region("eeprom", &esz);
		if (e)
			eepser_default(&s_eeprom, e, esz);
		eepser_nv_load(&s_eeprom, g->set);
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	if (s_cps2 && c2_decrypt(g))
		return -1;
	if (s_pang3) {
		if (strcmp(g->set, "pang3b")) {   // init_pang3: the low bytes of the words at 0x80000 .. 0xfffff
			uint16_t *rom = (uint16_t *)s_rom;
			for (uint32_t a = 0x80000; a < 0x100000 && a < s_rom_size; a += 2) {
				const int src = rom[a / 2];
				int dst = src & 0xff00;
				if (src & 0x01) dst ^= 0x04;
				if (src & 0x02) dst ^= 0x21;
				if (src & 0x04) dst ^= 0x01;
				if (~src & 0x08) dst ^= 0x50;
				if (src & 0x10) dst ^= 0x40;
				if (src & 0x20) dst ^= 0x06;
				if (src & 0x40) dst ^= 0x08;
				if (~src & 0x80) dst ^= 0x88;
				rom[a / 2] = (uint16_t)dst;
			}
		}
		eepser_init(&s_eeprom, EEPSER_93CXX, 64, 16, 12000000);
		uint32_t esz;
		const uint8_t *e = rbx_region("eeprom", &esz);
		if (e)
			eepser_default(&s_eeprom, e, esz);
		eepser_nv_load(&s_eeprom, g->set);
	}
	if (s_cps2)   // cps2_gfx_decode: the 2 MB banks unshuffled
		for (uint32_t i = 0; i + 0x200000 <= s_gfx_size; i += 0x200000)
			c2_unshuffle((uint64_t *)(s_gfx + i), 0x200000 / 8);
	decode_gfx();
	s_cpu_clock = s_cps2 ? CPS2_CLOCK : !strcmp(g->machine, "cps1_10MHz") || !strcmp(g->machine, "forgottn") ? 10000000 : 12000000;
	s_cpsb_base = (!strcmp(g->set, "sf2ee") || !strcmp(g->set, "sf2ue")) ? 0x8001c0 : 0x800140;
	memset(&b, 0, sizeof(b));
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus.read8 = s_cps2 ? c2_r8 : main_r8;
	m68k_bus.read16 = s_cps2 ? c2_r16 : main_r16;
	m68k_bus.write8 = s_cps2 ? c2_w8 : main_w8;
	m68k_bus.write16 = s_cps2 ? c2_w16 : main_w16;
	m68k_bus_map(0, 0x3fffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x900000, 0x92ffff, b.gfxram, 0x30000, 1, 1);
	m68k_bus_map(0xff0000, 0xffffff, b.ram, 0x10000, 1, 1);
	if (s_cps2) {
		m68k_bus.op = (const uint8_t *)s_c2op;
		m68k_bus.op_size = s_rom_size;
		m68k_bus_map(0x660000, 0x663fff, c2.ram66, 0x4000, 1, 1);
		s_c2out = s_cps2_dead ? &b.ram[0x7ff8] : c2.output;
		s_cps2_vol = !strcmp(g->set, "mvscjsing") ? 0xd000 : 0;   // (init_singbrd: no digital volume)
	}
	s_port_in0 = rbx_port_find("IN0");
	s_port_in1 = rbx_port_find("IN1");
	s_port_in2 = rbx_port_find("IN2");
	s_port_in3 = rbx_port_find("IN3");
	s_port_dsw[0] = rbx_port_find("DSWA");
	s_port_dsw[1] = rbx_port_find("DSWB");
	s_port_dsw[2] = rbx_port_find("DSWC");
	s_snap = calloc(1, sizeof(*s_snap));
	if (s_qs) {
		if (!s_snap || rbx_audio_setup_stereo(QS_CLOCK / 2496) || rbx_video_setup(VIS_W, VIS_H, 0xc00)) {
			rbx_fail("%s: out of memory", g->set);
			cps1_exit();
			return -1;
		}
		cps1_reset();
		return 0;
	}
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = Z80_CLOCK;
	yc.gain[0] = yc.gain[1] = 90;   // (0.35)
	yc.irq = ym_irq;
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		cps1_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 0xc00)) {
		rbx_fail("%s: out of memory", g->set);
		cps1_exit();
		return -1;
	}
	oki_init(&s_oki, 1000000, 1, s_oki_rom, s_oki_size, 77);   // (16 MHz / 16, pin 7 high, 0.30)
	cps1_reset();
	return 0;
}

static int cps1_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	z80 z = s_z80;
	int32_t ymirq = s_ym_irq;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "cps1", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= rbx_state_io(ctx, "obj", s_buffered_obj, sizeof(s_buffered_obj));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= rbx_state_io(ctx, "ymirq", &ymirq, sizeof(ymirq));
	rc |= rbx_state_io(ctx, "oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	if (s_qs) {
		rc |= rbx_state_io(ctx, "cps1q", &q, (uint32_t)offsetof(__typeof__(q), state_end));
		rc |= rbx_state_io(ctx, "qsound", &s_qsound, QSOUND_STATE_SIZE);
		rc |= eepser_state(&s_eeprom, ctx, load, "eeprom");
	} else
		rc |= rbx_ym_state(0, ctx, load);
	if (s_cps2) {
		rc |= rbx_state_io(ctx, "cps2", &c2, (uint32_t)offsetof(__typeof__(c2), state_end));
		rc |= rbx_state_io(ctx, "cps2obj", s_c2obj, sizeof(s_c2obj));
	}
	if (s_pang3) {
		rc |= rbx_state_io(ctx, "mcyc", &q.mcyc, sizeof(q.mcyc));
		rc |= eepser_state(&s_eeprom, ctx, load, "eeprom");
	}
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		z.read_byte = s_qs ? qz_read : z_read;
		z.write_byte = s_qs ? qz_write : z_write;
		z.op_rom = s_qs ? s_zop : NULL;
		z.port_in = z_in;
		z.port_out = z_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		s_z80 = z;
		s_ym_irq = ymirq;
		z_bank(b.zbank);
		s_oki.rate = 1000000 / (b.oki_pin7 ? 132 : 165);
		s_pal_valid = 0;
		if (s_cps2) {
			rbx_m68k_int_hook = c2_int_taken;
			c2_update_ipl();
		}
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int cps1_save(void *ctx) { return cps1_state(ctx, 0); }
static int cps1_load(void *ctx) { return cps1_state(ctx, 1); }

const rbx_driver rbx_drv_cps1 = { "Capcom CPS1", cps1_init, cps1_exit, cps1_reset, cps1_frame, cps1_save, cps1_load,
                                  cps1_supported, cps1_render, cps1_render_help };
