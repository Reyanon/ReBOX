/*  Toaplan GP9001 boards. Ported from MAME src/mame/toaplan (truxton2.cpp, snowbro2.cpp, enmadaio.cpp, vfive.cpp,
    kbash.cpp, batsugun.cpp, dogyuun.cpp, fixeight.cpp, raizing.cpp, raizing_batrider.cpp; BSD-3-Clause, copyright-holders: Quench, Yochizo, David Haywood).
    A 68000, one or two GP9001 VDPs (the IRQ at line 0xe6 until the game acknowledges it), a text tilemap on some, a
    YM2151 and an OKI; on some the sound chips behind an encrypted NEC V25 whose program the 68000 writes into their
    shared RAM. 432x262 at 6.75 MHz (59.64 Hz), 320x240 shown.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/nec/nec.h"
#include "../../cpu/z80/z80.h"
#include "../../machine/eeprom93c46.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../video/gp9001.h"
#include "../video/txtilemap.h"
#include "../machine/toaplan_v25_tables.h"

#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 59637          // 6.75 MHz / (432 x 262)
#define LINES 262
#define VIS_W 320
#define VIS_H 240
#define IRQ_LINE 0xe6

enum { M_TRUXTON2, M_SNOWBRO2, M_ENMADAIO, M_VFIVE, M_KBASH, M_BATSUGUN, M_DOGYUUN, M_FIXEIGHT, M_MAHOUDAI,
       M_SHIPPUMD, M_BGAREGGA, M_BATRIDER, M_COUNT };
static const char *const s_mnames[M_COUNT] = { "truxton2", "snowbro2", "enmadaio", "vfive", "kbash", "batsugun",
	"dogyuun", "fixeight", "mahoudai", "shippumd", "bgaregga", "batrider" };
#define IS_RAIZING(m) ((m) >= M_MAHOUDAI)

enum { IO_VDP, IO_VDPCOUNT, IO_PORT, IO_OKI, IO_YM, IO_OKIBANK, IO_OKIBANK40K, IO_TXVRAM, IO_TXSEL, IO_TXSCROLL,
       IO_VDP2, IO_SHARED, IO_COINRST, IO_SNDRST, IO_LATCH, IO_COINOKI, IO_VDPX, IO_Z80ROM, IO_BR_LATCH_R, IO_BR_BUSACK,
       IO_BR_LATCH_W, IO_BR_IRQCLR, IO_BR_BUSREQ, IO_BR_DMA_TEXT, IO_BR_DMA_PAL, IO_BR_OBJBANK, IO_NOP };
typedef struct { uint32_t start, end; uint8_t kind; const char *port; } io_range;
#define IO_END { 0, 0, 0xff, NULL }

static const io_range s_io_truxton2[] = { { 0x200000, 0x20000d, IO_VDP, NULL },
	{ 0x400000, 0x401fff, IO_TXVRAM, NULL }, { 0x402000, 0x402fff, IO_TXSEL, NULL }, { 0x403000, 0x4031ff, IO_TXSCROLL, NULL },
	{ 0x600000, 0x600001, IO_VDPCOUNT, NULL }, { 0x700000, 0x700001, IO_PORT, "DSWA" }, { 0x700002, 0x700003, IO_PORT, "DSWB" },
	{ 0x700004, 0x700005, IO_PORT, "JMPR" }, { 0x700006, 0x700007, IO_PORT, "IN1" }, { 0x700008, 0x700009, IO_PORT, "IN2" },
	{ 0x70000a, 0x70000b, IO_PORT, "SYS" }, { 0x700010, 0x700011, IO_OKI, NULL }, { 0x700014, 0x700017, IO_YM, NULL },
	{ 0x70001e, 0x70001f, IO_NOP, NULL }, IO_END };
static const io_range s_io_snowbro2[] = { { 0x300000, 0x30000d, IO_VDP, NULL }, { 0x500000, 0x500003, IO_YM, NULL },
	{ 0x600000, 0x600001, IO_OKI, NULL }, { 0x700000, 0x700001, IO_PORT, "JMPR" }, { 0x700004, 0x700005, IO_PORT, "DSWA" },
	{ 0x700008, 0x700009, IO_PORT, "DSWB" }, { 0x70000c, 0x70000d, IO_PORT, "IN1" }, { 0x700010, 0x700011, IO_PORT, "IN2" },
	{ 0x700014, 0x700015, IO_PORT, "IN3" }, { 0x700018, 0x700019, IO_PORT, "IN4" }, { 0x70001c, 0x70001d, IO_PORT, "SYS" },
	{ 0x700030, 0x700031, IO_OKIBANK, NULL }, { 0x700034, 0x700035, IO_NOP, NULL }, IO_END };
static const io_range s_io_enmadaio[] = { { 0x200000, 0x20000d, IO_VDP, NULL }, { 0x400000, 0x400003, IO_YM, NULL },
	{ 0x500000, 0x500001, IO_OKI, NULL }, { 0x700000, 0x700001, IO_VDPCOUNT, NULL }, { 0x700004, 0x700005, IO_PORT, "DSWA" },
	{ 0x70000c, 0x70000d, IO_PORT, "MISC2" }, { 0x700010, 0x700011, IO_PORT, "MISC3" }, { 0x700014, 0x700015, IO_PORT, "MISC4" },
	{ 0x700018, 0x700019, IO_PORT, "SYS" }, { 0x70001c, 0x70001d, IO_PORT, "UNK" }, { 0x700020, 0x700021, IO_OKIBANK40K, NULL },
	{ 0x700028, 0x70003d, IO_NOP, NULL }, IO_END };
// (the V25 boards: the shared RAM's bytes at the odd addresses, the V25's reset line with the coin counters)
static const io_range s_io_vfive[] = { { 0x200010, 0x200011, IO_PORT, "IN1" }, { 0x200014, 0x200015, IO_PORT, "IN2" },
	{ 0x200018, 0x200019, IO_PORT, "SYS" }, { 0x20001c, 0x20001d, IO_COINRST, NULL }, { 0x210000, 0x21ffff, IO_SHARED, NULL },
	{ 0x300000, 0x30000d, IO_VDP, NULL }, { 0x500000, 0x50000d, IO_VDP2, NULL }, { 0x700000, 0x700001, IO_VDPCOUNT, NULL },
	IO_END };
static const io_range s_io_fixeight[] = { { 0x200000, 0x200001, IO_PORT, "IN1" }, { 0x200004, 0x200005, IO_PORT, "IN2" },
	{ 0x200008, 0x200009, IO_PORT, "IN3" }, { 0x200010, 0x200011, IO_PORT, "SYS" }, { 0x20001c, 0x20001d, IO_NOP, NULL },
	{ 0x280000, 0x28ffff, IO_SHARED, NULL }, { 0x300000, 0x30000d, IO_VDP, NULL }, { 0x500000, 0x501fff, IO_TXVRAM, NULL },
	{ 0x502000, 0x5021ff, IO_TXSEL, NULL }, { 0x503000, 0x5031ff, IO_TXSCROLL, NULL }, { 0x700000, 0x700001, IO_SNDRST, NULL },
	{ 0x800000, 0x800001, IO_VDPCOUNT, NULL }, IO_END };
// raizing.cpp common_mem (the shared RAM's bytes at the odd addresses)
static const io_range s_io_raizing[] = { { 0x218000, 0x21bfff, IO_SHARED, NULL }, { 0x21c01c, 0x21c01d, IO_COINOKI, NULL },
	{ 0x21c020, 0x21c021, IO_PORT, "IN1" }, { 0x21c024, 0x21c025, IO_PORT, "IN2" }, { 0x21c028, 0x21c029, IO_PORT, "SYS" },
	{ 0x21c02c, 0x21c02d, IO_PORT, "DSWA" }, { 0x21c030, 0x21c031, IO_PORT, "DSWB" }, { 0x21c034, 0x21c035, IO_PORT, "JMPR" },
	{ 0x21c03c, 0x21c03d, IO_VDPCOUNT, NULL }, { 0x300000, 0x30000d, IO_VDP, NULL }, { 0x500000, 0x501fff, IO_TXVRAM, NULL },
	{ 0x502000, 0x502fff, IO_TXSEL, NULL }, { 0x503000, 0x5031ff, IO_TXSCROLL, NULL }, { 0x600000, 0x600001, IO_LATCH, NULL },
	IO_END };
// raizing_batrider.cpp batrider_68k_mem
static const io_range s_io_batrider[] = { { 0x300000, 0x37ffff, IO_Z80ROM, NULL }, { 0x400000, 0x40000d, IO_VDPX, NULL },
	{ 0x500000, 0x500001, IO_PORT, "IN" }, { 0x500002, 0x500003, IO_PORT, "SYS-DSW" }, { 0x500004, 0x500005, IO_PORT, "DSW" },
	{ 0x500006, 0x500007, IO_VDPCOUNT, NULL }, { 0x500008, 0x50000b, IO_BR_LATCH_R, NULL }, { 0x50000c, 0x50000d, IO_BR_BUSACK, NULL },
	{ 0x500010, 0x500011, IO_NOP, NULL }, { 0x500020, 0x500023, IO_BR_LATCH_W, NULL }, { 0x500024, 0x500025, IO_NOP, NULL },
	{ 0x500026, 0x500027, IO_BR_IRQCLR, NULL }, { 0x500060, 0x500061, IO_BR_BUSREQ, NULL }, { 0x500080, 0x500081, IO_BR_DMA_TEXT, NULL },
	{ 0x500082, 0x500083, IO_BR_DMA_PAL, NULL }, { 0x5000c0, 0x5000cf, IO_BR_OBJBANK, NULL }, IO_END };
static const io_range s_io_kbash[] = { { 0x200000, 0x200fff, IO_SHARED, NULL }, { 0x208010, 0x208011, IO_PORT, "IN1" },
	{ 0x208014, 0x208015, IO_PORT, "IN2" }, { 0x208018, 0x208019, IO_PORT, "SYS" }, { 0x20801c, 0x20801d, IO_NOP, NULL },
	{ 0x300000, 0x30000d, IO_VDP, NULL }, { 0x700000, 0x700001, IO_VDPCOUNT, NULL }, IO_END };

static int s_m;
static const io_range *s_io;
static int s_ioport[32];
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint32_t s_cpu_clock;
static int s_irq_level;
static int s_has_tx;
static uint32_t s_pal_entries;
static int s_nvdp, s_v25, s_has_oki, s_rst_bit;

static struct toaplan2_state {   // (on the heap while the game runs, ADR-040)
	uint16_t ram[0x8000];
	uint16_t ram2[0x800];        // truxton2 0x403200 ..
	uint16_t pal[0x800];
	uint16_t txgfx[0x8000];      // truxton2: the text layer's graphics in RAM
	uint32_t oki_bank;
	int32_t line;
	uint32_t frame;
	uint64_t cpu_frac, ym_frac;
	uint32_t oki_acc, oki2_acc;
	int32_t oki_hold, oki2_hold;
	// the V25
	uint8_t shram[0x8000];
	uint8_t latch, latch_pending, z80_bank, oki_e[2][8];
	uint8_t br_latch[4], br_nmi, br_irq, br_busreq;
	uint16_t objbank[8];
	uint8_t v25_held;
	int64_t v25_end;
	uint64_t v25_frac;
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)
static gp9001_t s_vdp, s_vdp2;
static nec_t s_nec;
static uint32_t s_v25_clock;
static const uint8_t *s_zrom;
static uint32_t s_zrom_size;
static int s_dsw_pt, s_dsw_p0, s_dsw_p1;
static int s_in_port[3];               // fixeight: the V25 reads the inputs too
static eeprom93c46_t s_eeprom;
static z80 s_z80;
static const uint8_t *s_zmap[256];
static uint32_t s_ymclock;
static const uint8_t *s_text;          // raizing: the text layer's ROM (4 bpp packed, high nibble first)
static uint32_t s_text_size;
static okim6295_t s_oki2;              // batrider's second OKI
static const uint8_t *s_oki2_rom;
static uint32_t s_oki2_size;
static char s_nv_path[256];
static int s_has_eeprom;
static txtilemap_t s_tx;
static int s_line_cyc;
static okim6295_t s_oki;
static const uint8_t *s_oki_rom;
static uint32_t s_oki_size;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;

// ---- the snapshot drawn ----
static struct {
	gp9001_t vdp, vdp2;
	txtilemap_t tx;
	uint16_t txgfx[0x8000];
	uint16_t pal[0x800];
	uint16_t objbank[8];
} *s_snap;
static uint8_t s_pri[VIS_W * VIS_H];
static uint16_t s_bm2[VIS_W * VIS_H];   // batsugun: the second VDP's picture, mixed

static uint8_t tx_pix_ram(void *user, uint32_t code, int x, int y)   // truxton2_tx_tilelayout: words, the low byte
{
	const uint16_t *g = (const uint16_t *)user;
	const uint16_t w = g[(code * 32 + (uint32_t)y * 4 + (uint32_t)(x >> 1)) & 0x7fff];
	return (uint8_t)((x & 1) ? (w & 15) : ((w >> 4) & 15));
}

static void update_irq(void) { m68k_bus_set_line(s_irq_level, s_vdp.vint ? 1 : 0); }
static void br_update_irq(void);

static int vpos_now(void) { return b.line; }
static int hpos_now(void)
{
	const int d = m68k_cycles_run();
	return s_line_cyc > 0 ? (int)((int64_t)(d < s_line_cyc ? d : s_line_cyc) * 432 / s_line_cyc) : 0;
}

static const io_range *io_find(uint32_t a)
{
	for (const io_range *r = s_io; r->kind != 0xff; r++)
		if (a >= r->start && a <= r->end)
			return r;
	return NULL;
}

static uint16_t main_r16(uint32_t a)
{
	a &= 0xfffffe;
	const io_range *r = io_find(a);
	if (!r)
		return 0;
	const int off = (int)((a - r->start) >> 1);
	switch (r->kind) {
	case IO_VDP:
		return gp9001_read(&s_vdp, off, vpos_now());
	case IO_VDPX:   // (batrider: the registers in reverse order)
		return gp9001_read(&s_vdp, off ^ 6, vpos_now());
	case IO_Z80ROM:
		return (uint32_t)off < s_zrom_size ? s_zrom[off] : 0;
	case IO_BR_LATCH_R:
		return b.br_latch[2 + (off & 1)];
	case IO_BR_BUSACK:
		return b.br_busreq;
	case IO_VDP2:
		return gp9001_read(&s_vdp2, off, vpos_now());
	case IO_SHARED:
		return b.shram[off & (s_m == M_KBASH ? 0x7ff : IS_RAIZING(s_m) ? 0x1fff : 0x7fff)];
	case IO_VDPCOUNT:
		return gp9001_vdpcount(vpos_now(), hpos_now());
	case IO_PORT:
		return (uint16_t)rbx_port_read(s_ioport[r - s_io]);
	case IO_OKI:
		return oki_read(&s_oki);
	case IO_YM:
		return rbx_ym_read(0, (uint32_t)(off & 1));
	case IO_TXVRAM:
		return s_tx.vram[off & 0xfff];
	case IO_TXSEL:
		return s_tx.lineselect[off & 0x7ff];
	case IO_TXSCROLL:
		return s_tx.linescroll[off & 0xff];
	default:
		return 0;
	}
}
static uint8_t main_r8(uint32_t a)
{
	const uint16_t w = main_r16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}
static void main_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xfffffe;
	const io_range *r = io_find(a);
	if (!r)
		return;
	const int off = (int)((a - r->start) >> 1);
	switch (r->kind) {
	case IO_VDP:
		gp9001_write(&s_vdp, off, v, mask);
		update_irq();
		break;
	case IO_VDP2:
		gp9001_write(&s_vdp2, off, v, mask);
		break;
	case IO_VDPX:
		gp9001_write(&s_vdp, off ^ 6, v, mask);
		update_irq();
		break;
	case IO_BR_LATCH_W:   // batrider_soundlatch_w / 2: the Z80's NMI
		if (mask & 0xff) {
			b.br_latch[off & 1] = (uint8_t)v;
			if (!b.br_nmi) {
				b.br_nmi = 1;
				z80_gen_nmi(&s_z80);
			}
		}
		break;
	case IO_BR_IRQCLR:
		b.br_irq = 0;
		br_update_irq();
		break;
	case IO_BR_BUSREQ:
		if (mask & 0xff)
			b.br_busreq = v & 1;
		break;
	case IO_BR_DMA_TEXT:   // batrider_textdata_dma_w: the main RAM's 0x8000 bytes to the text graphics
		memcpy(b.txgfx, b.ram, 0x8000);
		break;
	case IO_BR_DMA_PAL: {  // batrider_pal_text_dma_w: text RAM, palette, line select, line scroll
		const uint16_t *m = b.ram;
		memcpy(s_tx.vram, m, 0x2000);
		memcpy(b.pal, m + 0x1000, 0x1000);
		memcpy(s_tx.lineselect, m + 0x1800, 0x200);
		memcpy(s_tx.linescroll, m + 0x1900, 0x200);
		break;
	}
	case IO_BR_OBJBANK:
		if (mask & 0xff)
			b.objbank[off & 7] = v & 0xf;
		break;
	case IO_SHARED:
		if (mask & 0xff)
			b.shram[off & (s_m == M_KBASH ? 0x7ff : IS_RAIZING(s_m) ? 0x1fff : 0x7fff)] = (uint8_t)v;
		break;
	case IO_LATCH:   // bgaregga: the sound latch (its pending line the Z80's INT)
		if (s_m == M_BGAREGGA && (mask & 0xff)) {
			b.latch = (uint8_t)v;
			b.latch_pending = 1;
		}
		break;
	case IO_COINOKI:   // shippumd_coin_w: bit 4 the OKI's bank
		if (s_m == M_SHIPPUMD && (mask & 0xff))
			b.oki_bank = (uint32_t)((v >> 4) & 1) * 0x40000;
		break;
	case IO_SNDRST:    // fixeight sound_reset_w: the V25's reset line (bit 3) held while clear
		if (mask & 0xff) {
			const int held = !((v >> 3) & 1);
			if (b.v25_held && !held)
				nec_reset(&s_nec);
			b.v25_held = (uint8_t)held;
		}
		break;
	case IO_COINRST:   // coin_sound_reset_w: the V25's reset line (bit 4, dogyuun bit 5) held while clear
		if (mask & 0xff) {
			const int held = !((v >> s_rst_bit) & 1);
			if (b.v25_held && !held)
				nec_reset(&s_nec);
			b.v25_held = (uint8_t)held;
		}
		break;
	case IO_OKI:
		if (mask & 0xff)
			oki_write(&s_oki, (uint8_t)v);
		break;
	case IO_YM:
		if (mask & 0xff)
			rbx_ym_write(0, (uint32_t)(off & 1), (uint8_t)v);
		break;
	case IO_OKIBANK:   // (set_rom_bank: 0x40000 a bank)
		if (mask & 0xff)
			b.oki_bank = (uint32_t)(v & 1) * 0x40000;
		break;
	case IO_OKIBANK40K:
		b.oki_bank = (uint32_t)(v & 0xff) * 0x40000;
		break;
	case IO_TXVRAM:
		s_tx.vram[off & 0xfff] = (uint16_t)((v & mask) | (s_tx.vram[off & 0xfff] & ~mask));
		break;
	case IO_TXSEL:
		s_tx.lineselect[off & 0x7ff] = (uint16_t)((v & mask) | (s_tx.lineselect[off & 0x7ff] & ~mask));
		break;
	case IO_TXSCROLL:
		s_tx.linescroll[off & 0xff] = (uint16_t)((v & mask) | (s_tx.linescroll[off & 0xff] & ~mask));
		break;
	default:
		break;
	}
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v)
{
	main_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00);
}

static uint8_t oki_banked(void *user, uint32_t a)
{
	(void)user;
	const uint32_t o = b.oki_bank + a;
	return o < s_oki_size ? s_oki_rom[o] : 0;
}

// ---- the V25 (vfive.cpp, kbash.cpp, batsugun.cpp, dogyuun.cpp sound maps) ----
static uint8_t v25_r8(uint32_t a)
{
	a &= 0xfffff;
	if (s_m == M_KBASH) {
		if (a < 0x800)
			return b.shram[a];
		if (a == 0x4000 || a == 0x4001) {
			const uint8_t st = rbx_ym_read(0, a & 1);
			return st;
		}
		if (a == 0x4002)
			return oki_read(&s_oki);
		if (a >= 0x80000)
			return s_zrom[rbx_wrap((a - 0x80000), s_zrom_size)];
		return 0xff;
	}
	if (a >= 0x80000)
		return b.shram[a & 0x7fff];
	if (s_m == M_FIXEIGHT) {   // the inputs at 0, 2, 4; the YM at 0x0a, the OKI at 0x0c
		if (a == 0 || a == 2 || a == 4)
			return s_in_port[a >> 1] >= 0 ? (uint8_t)rbx_port_read(s_in_port[a >> 1]) : 0xff;
		if (a == 0x0a || a == 0x0b)
			return rbx_ym_read(0, a & 1);
		if (a == 0x0c)
			return oki_read(&s_oki);
		return 0xff;
	}
	if (a < 2)
		return rbx_ym_read(0, a & 1);
	if (a == 4 && s_has_oki)
		return oki_read(&s_oki);
	return 0xff;
}
static uint16_t v25_r16(uint32_t a) { return (uint16_t)(v25_r8(a) | v25_r8(a + 1) << 8); }
static void v25_w8(uint32_t a, uint8_t v)
{
	a &= 0xfffff;
	if (s_m == M_KBASH) {
		if (a < 0x800)
			b.shram[a] = v;
		else if (a == 0x4000 || a == 0x4001)
			rbx_ym_write(0, a & 1, v);
		else if (a == 0x4002)
			oki_write(&s_oki, v);
		return;
	}
	if (a >= 0x80000)
		b.shram[a & 0x7fff] = v;
	else if (s_m == M_FIXEIGHT) {
		if (a == 0x0a || a == 0x0b)
			rbx_ym_write(0, a & 1, v);
		else if (a == 0x0c)
			oki_write(&s_oki, v);
	} else if (a < 2) {
		static int n, reg, kon; n++; if (!(a & 1)) reg = v; else if (reg == 8) kon++; 
		rbx_ym_write(0, a & 1, v);
	}
	else if (a == 4 && s_has_oki)
		oki_write(&s_oki, v);
}
static void v25_w16(uint32_t a, uint16_t v)
{
	v25_w8(a, (uint8_t)v);
	v25_w8(a + 1, (uint8_t)(v >> 8));
}
static void v25_port_out(int port, uint8_t v)   // fixeight P0: the EEPROM's CS (bit 4), CLK (5), DI (6)
{
	if (s_m == M_FIXEIGHT && port == 0)
		eeprom_write_lines(&s_eeprom, (v >> 4) & 1, (v >> 5) & 1, (v >> 6) & 1);
}
static uint8_t v25_port_in(int port)   // P0, P1, P2, PT: the DIP switches (inverted)
{
	if (s_m == M_FIXEIGHT)   // P0 bit 7: the EEPROM's DO
		return port == 0 ? (uint8_t)(eeprom_do(&s_eeprom) ? 0x80 : 0) : 0xff;
	const int p = port == 3 ? s_dsw_pt : port == 0 ? s_dsw_p0 : port == 1 ? s_dsw_p1 : -1;
	return p < 0 ? 0xff : (uint8_t)(rbx_port_read(p) ^ 0xff);
}

static void v25_map(void)
{
	for (int i = 0; i < 256; i++)
		s_nec.omap[i] = s_nec.rmap[i] = NULL, s_nec.wmap[i] = NULL;
	if (s_m == M_KBASH) {
		for (uint32_t a = 0x80000; a < 0x100000; a += 0x8000)
			nec_map(&s_nec, a, a + 0x8000, s_zrom, NULL);
	} else
		for (uint32_t a = 0x80000; a < 0x100000; a += 0x8000)
			nec_map(&s_nec, a, a + 0x8000, b.shram, b.shram);
}

static void v25_line(void)
{
	b.v25_end += rbx_line_cycles(&b.v25_frac, s_v25_clock, FRAME_MHZ, LINES);
	if (b.v25_held)
		s_nec.total = b.v25_end;
	else
		nec_run_until(&s_nec, b.v25_end);
}

// ---- raizing.cpp: the sound Z80 ----
static uint8_t oki_raizing(void *user, uint32_t a)   // the GAL's banks: 0x100 table pages, then 64K pages
{
	const int chip = (int)(intptr_t)user;
	const int k = a < 0x400 ? (int)(a >> 8) : a < 0x10000 ? 4 : 4 + (int)(a >> 16);
	const uint32_t o = (uint32_t)b.oki_e[chip][k] * 0x10000 + (a < 0x10000 ? a : (a & 0xffff));
	const uint8_t *rom = chip ? s_oki2_rom : s_oki_rom;
	return o < (chip ? s_oki2_size : s_oki_size) ? rom[o] : 0;
}
static void raizing_oki_bank_w(int offset, uint8_t data)
{
	int chip = (offset & 4) >> 2;
	b.oki_e[chip][offset & 3] = b.oki_e[chip][4 + (offset & 3)] = data & 0xf;
	offset++;
	data >>= 4;
	chip = (offset & 4) >> 2;
	b.oki_e[chip][offset & 3] = b.oki_e[chip][4 + (offset & 3)] = data & 0xf;
}
static void raizing_z80_bank(uint8_t v)
{
	b.z80_bank = v & 0x0f;
	const uint32_t base = (uint32_t)(b.z80_bank & (s_m == M_BATRIDER ? 15 : 7)) * 0x4000;   // (bgaregga: 8..15 mirror 0..7)
	for (int p = 0; p < 0x40; p++)
		s_zmap[0x80 + p] = base + (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + base + (uint32_t)p * 256 : NULL;
}
static uint8_t rz_read(void *u, uint16_t a)
{
	(void)u;
	const uint8_t *p = s_zmap[a >> 8];
	if (p)
		return p[a & 0xff];
	if (s_m == M_BATRIDER)
		return a >= 0xc000 && a < 0xe000 ? b.shram[a - 0xc000] : 0xff;
	if (a >= 0xc000 && a < 0xe000)
		return b.shram[a - 0xc000];
	if (a == 0xe000 || a == 0xe001)
		return rbx_ym_read(0, a & 1);
	if (a == 0xe004)
		return oki_read(&s_oki);
	if (s_m == M_BGAREGGA) {
		if (a == 0xe01c)
			return b.latch;
		if (a == 0xe01d)
			return b.latch_pending ? 0 : 1;
	}
	return 0xff;
}
static void rz_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_m == M_BATRIDER) {
		if (a >= 0xc000 && a < 0xe000)
			b.shram[a - 0xc000] = v;
		return;
	}
	if (a >= 0xc000 && a < 0xe000)
		b.shram[a - 0xc000] = v;
	else if (a == 0xe000 || a == 0xe001)
		rbx_ym_write(0, a & 1, v);
	else if (a == 0xe004)
		oki_write(&s_oki, v);
	else if (s_m == M_BGAREGGA) {
		if (a >= 0xe006 && a <= 0xe008)
			raizing_oki_bank_w(a - 0xe006, v);
		else if (a == 0xe00a)
			raizing_z80_bank(v);
		else if (a == 0xe00c)
			b.latch_pending = 0;   // (acknowledge_w)
	}
}
static void br_update_irq(void) { m68k_bus_set_line(4, b.br_irq); }
static uint8_t rz_in(z80 *z, uint8_t port)   // batrider_sound_z80_port
{
	(void)z;
	if (s_m != M_BATRIDER)
		return 0xff;
	switch (port) {
	case 0x48: return b.br_latch[0];
	case 0x4a: return b.br_latch[1];
	case 0x80: case 0x81: return rbx_ym_read(0, port & 1);
	case 0x82: return oki_read(&s_oki);
	case 0x84: return oki_read(&s_oki2);
	}
	return 0xff;
}
static void rz_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	if (s_m != M_BATRIDER)
		return;
	if (port >= 0xc0 && port <= 0xc6) {
		raizing_oki_bank_w(port - 0xc0, v);
		return;
	}
	switch (port) {
	case 0x40: b.br_latch[2] = v; break;
	case 0x42: b.br_latch[3] = v; break;
	case 0x44: b.br_irq = 1; br_update_irq(); break;   // (the 68000's IRQ 4)
	case 0x46: b.br_nmi = 0; break;
	case 0x80: case 0x81: rbx_ym_write(0, port & 1, v); break;
	case 0x82: oki_write(&s_oki, v); break;
	case 0x84: oki_write(&s_oki2, v); break;
	case 0x88: raizing_z80_bank(v); break;
	}
}
static void rz_start(void)
{
	z80_init(&s_z80);
	s_z80.read_byte = rz_read;
	s_z80.write_byte = rz_write;
	s_z80.port_in = rz_in;
	s_z80.port_out = rz_out;
	s_z80.read_map = s_zmap;
}
static void rz_line(void)
{
	b.v25_end += rbx_line_cycles(&b.v25_frac, s_m == M_BATRIDER ? 32000000 / 6 : 4000000, FRAME_MHZ, LINES);
	int irq = s_m == M_BGAREGGA && b.latch_pending;
	if (b.br_busreq) {   // (the bus given to the 68000: the Z80 stands still)
		s_z80.cyc = (unsigned long)b.v25_end;
		return;
	}
	if ((int64_t)s_z80.cyc < b.v25_end)
		z80_run(&s_z80, (unsigned long)b.v25_end, &irq, 0);
}
static uint8_t tx_pix_batrider(void *user, uint32_t code, int x, int y)   // batrider_tx_tilelayout (words)
{
	const uint16_t *g = (const uint16_t *)user;
	const uint16_t w = g[(code * 16 + (uint32_t)y * 2 + (uint32_t)(x >> 2)) & 0x3fff];
	static const uint8_t sh[4] = { 4, 0, 12, 8 };
	return (uint8_t)((w >> sh[x & 3]) & 15);
}
static uint32_t br_tile_cb(void *user, int layer, uint32_t code)   // batrider_bank_cb: the object banks of 32K tiles
{
	(void)layer;
	const uint16_t *bank = user ? (const uint16_t *)user : b.objbank;
	return (uint32_t)bank[(code >> 15) & 7] << 15 | (code & 0x7fff);
}
static uint8_t text_pix(void *user, uint32_t code, int x, int y)   // gfx_8x8x4_packed_msb
{
	(void)user;
	const uint32_t o = (code * 32 + (uint32_t)y * 4 + (uint32_t)(x >> 1)) % (s_text_size ? s_text_size : 1);
	const uint8_t v = s_text ? s_text[o] : 0;
	return (uint8_t)((x & 1) ? (v & 15) : (v >> 4));
}

// ---- drawing: bands, shared with the threads waiting for it ----
#define BANDS 8
static int s_band_next, s_band_left, s_band_active, s_band_helpers;

static void run_band(int band)
{
	const int y0 = VIS_H * band / BANDS, y1 = VIS_H * (band + 1) / BANDS;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	if (s_m == M_DOGYUUN) {   // the second VDP first, the first over it (each with its own priorities)
		gp9001_draw(&s_snap->vdp2, bm, pitch, s_pri + y0 * VIS_W, VIS_W, y0, y1, 1);
		gp9001_draw(&s_snap->vdp, bm, pitch, s_pri + y0 * VIS_W, VIS_W, y0, y1, 0);
		return;
	}
	gp9001_draw(&s_snap->vdp, bm, pitch, s_pri + y0 * VIS_W, VIS_W, y0, y1, 1);
	if (s_m == M_BATSUGUN) {   // the PALs' mix of the two pictures
		gp9001_draw(&s_snap->vdp2, s_bm2, VIS_W, s_pri + y0 * VIS_W, VIS_W, y0, y1, 1);
		for (int y = y0; y < y1; y++) {
			uint16_t *d = bm + y * pitch;
			const uint16_t *s2 = s_bm2 + y * VIS_W;
			for (int x = 0; x < VIS_W; x++) {
				const uint16_t g0 = d[x], g1 = s2[x];
				if ((g1 & 0xf) && (!(g0 & 0xf) || (g0 & 0x780) > (g1 & 0x780)))
					d[x] = g1;
			}
		}
	}
	if (s_has_tx)
		txtilemap_draw(&s_snap->tx, rbx_video.bitmap, rbx_video.pitch, VIS_W, y0, y1);
}

static void t2_render(void)
{
	for (uint32_t i = 0; i < s_pal_entries; i++) {   // xBGR_555
		const uint16_t p = s_snap->pal[i];
		const int r = p & 31, g = (p >> 5) & 31, bb = (p >> 10) & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (bb << 3 | bb >> 2));
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

static int t2_render_help(int helper)
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

// screen update before the vblank (VIDEO_UPDATE_BEFORE_VBLANK), then screen_eof
static void vblank(void)
{
	rbx_video_wait_idle();
	s_snap->vdp = s_vdp;
	memcpy(s_snap->objbank, b.objbank, sizeof(b.objbank));
	s_snap->vdp.cb_user = s_snap->objbank;   // (batrider's banks as they were)
	if (s_nvdp > 1)
		s_snap->vdp2 = s_vdp2;
	if (s_has_tx) {
		s_snap->tx = s_tx;
		memcpy(s_snap->txgfx, b.txgfx, sizeof(b.txgfx));
		s_snap->tx.user = s_snap->txgfx;
	}
	memcpy(s_snap->pal, b.pal, sizeof(b.pal));
	rbx_video_frame_done();
	gp9001_eof(&s_vdp);
	if (s_nvdp > 1)
		gp9001_eof(&s_vdp2);
}

static void sound_line(void)
{
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&b.ym_frac, s_ymclock, FRAME_MHZ, LINES);
	const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	const uint32_t yr = rbx_ym_rate(0);
	for (int i = 0; s_has_oki && i < n; i++) {
		b.oki_acc += s_oki.rate;
		while (b.oki_acc >= yr) {
			b.oki_acc -= yr;
			b.oki_hold = oki_sample(&s_oki);
		}
		s_mix[s_mix_n + i] += b.oki_hold;
	}
	for (int i = 0; s_m == M_BATRIDER && i < n; i++) {
		b.oki2_acc += s_oki2.rate;
		while (b.oki2_acc >= yr) {
			b.oki2_acc -= yr;
			b.oki2_hold = oki_sample(&s_oki2);
		}
		s_mix[s_mix_n + i] += b.oki2_hold;
	}
	s_mix_n += n;
}

static void t2_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		b.line = line;
		if (line == VIS_H)
			vblank();
		if (line == IRQ_LINE) {
			s_vdp.vint = 1;
			update_irq();
		}
		s_line_cyc = rbx_line_cycles(&b.cpu_frac, s_cpu_clock, FRAME_MHZ, LINES);
		m68k_execute(s_line_cyc);
		if (s_v25)
			v25_line();
		else if (IS_RAIZING(s_m))
			rz_line();
		sound_line();
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	b.frame++;
}

static void t2_reset(void)
{
	memset(&b, 0, offsetof(__typeof__(b), state_end));
	memset(s_vdp.vram, 0, sizeof(s_vdp.vram));
	memset(s_vdp.spr, 0, sizeof(s_vdp.spr));
	memset(s_vdp.spr_buf, 0, sizeof(s_vdp.spr_buf));
	gp9001_reset(&s_vdp);
	if (s_nvdp > 1) {
		memset(s_vdp2.vram, 0, sizeof(s_vdp2.vram));
		memset(s_vdp2.spr, 0, sizeof(s_vdp2.spr));
		memset(s_vdp2.spr_buf, 0, sizeof(s_vdp2.spr_buf));
		gp9001_reset(&s_vdp2);
	}
	memset(&s_tx, 0, TXTILEMAP_STATE_SIZE);
	if (IS_RAIZING(s_m)) {
		memset(s_zmap, 0, sizeof(s_zmap));
		const uint32_t fixed = (s_m == M_BGAREGGA || s_m == M_BATRIDER) ? 0x8000 : 0xc000;
		for (uint32_t pg = 0; pg < fixed / 256 && pg * 256 + 256 <= s_zrom_size; pg++)
			s_zmap[pg] = s_zrom + pg * 256;
		if (s_m == M_BGAREGGA || s_m == M_BATRIDER)
			raizing_z80_bank(0);
		rz_start();
		if (s_m == M_BATRIDER)
			oki_reset(&s_oki2);
	}
	if (s_v25) {   // (machine_reset: coin_sound_reset_w(0), the V25 held; kbash's runs)
		b.v25_held = s_m != M_KBASH;
		memset(s_nec.iram, 0, sizeof(s_nec.iram));
		s_nec.total = 0;
		nec_reset(&s_nec);
	}
	rbx_ym_reset(0);
	oki_reset(&s_oki);
	s_mix_n = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
}

static void t2_exit(void)
{
	if (s_nv_path[0] && s_eeprom.dirty)
		eeprom_save(&s_eeprom, s_nv_path);
	s_nv_path[0] = 0;
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	free(s_snap);
	s_snap = NULL;
	free(s_bp);   // (last: the render thread and the saves are done with it)
	s_bp = NULL;
}

static int t2_init(const rbx_game *g)
{
	if (!s_bp && !(s_bp = (struct toaplan2_state *)calloc(1, sizeof(*s_bp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_m = -1;
	for (int i = 0; i < M_COUNT; i++)
		if (!strcmp(g->machine, s_mnames[i]))
			s_m = i;
	if (s_m < 0) {
		rbx_fail("%s: machine %s not emulated", g->set, g->machine);
		return -1;
	}
	s_nvdp = (s_m == M_BATSUGUN || s_m == M_DOGYUUN) ? 2 : 1;
	s_v25 = s_m >= M_VFIVE && s_m <= M_FIXEIGHT;
	s_has_oki = s_m != M_VFIVE;
	uint32_t gsize = 0, gsize2 = 0;
	uint8_t *gfx = rbx_region(s_nvdp > 1 ? "gp9001_0" : "gp9001", &gsize);
	uint8_t *gfx2 = s_nvdp > 1 ? rbx_region("gp9001_1", &gsize2) : NULL;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_oki_rom = rbx_region("oki", &s_oki_size);
	if (!s_oki_rom)
		s_oki_rom = rbx_region("oki1", &s_oki_size);   // (raizing.cpp)
	if (!s_rom || !gfx || (s_has_oki && !s_oki_rom) || (s_nvdp > 1 && !gfx2)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	if (gp9001_init(&s_vdp, gfx, gsize) || (gfx2 && gp9001_init(&s_vdp2, gfx2, gsize2))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	memset(&b, 0, sizeof(b));
	memset(&s_tx, 0, sizeof(s_tx));
	s_has_tx = 0;
	s_pal_entries = 0x800;
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
	m68k_bus_map(0, 0x7ffff, s_rom, s_rom_size < 0x80000 ? s_rom_size : 0x80000, 1, 0);
	switch (s_m) {
	case M_TRUXTON2:
		s_cpu_clock = 16000000;
		s_irq_level = 2;
		m68k_bus_map(0x100000, 0x10ffff, b.ram, 0x10000, 1, 1);
		m68k_bus_map(0x300000, 0x300fff, b.pal, 0x1000, 1, 1);
		m68k_bus_map(0x403200, 0x403fff, b.ram2, 0xe00, 1, 1);
		m68k_bus_map(0x500000, 0x50ffff, b.txgfx, 0x10000, 1, 1);
		s_io = s_io_truxton2;
		s_has_tx = 1;
		s_tx.dx = 0x1d5;
		s_tx.dx_flipped = 0x16a;
		break;
	case M_SNOWBRO2:
		s_cpu_clock = 16000000;
		s_irq_level = 4;
		m68k_bus_map(0x100000, 0x10ffff, b.ram, 0x10000, 1, 1);
		m68k_bus_map(0x400000, 0x400fff, b.pal, 0x1000, 1, 1);
		s_io = s_io_snowbro2;
		break;
	case M_ENMADAIO:
		s_cpu_clock = 10000000;
		s_irq_level = 4;
		m68k_bus_map(0x100000, 0x10ffff, b.ram, 0x10000, 1, 1);
		m68k_bus_map(0x300000, 0x300fff, b.pal, 0x1000, 1, 1);
		s_io = s_io_enmadaio;
		break;
	case M_BATRIDER:
		s_cpu_clock = 16000000;
		s_irq_level = 2;
		m68k_bus_map(0, 0x1fffff, s_rom, s_rom_size, 1, 0);
		m68k_bus_map(0x200000, 0x20ffff, b.ram, 0x10000, 1, 1);
		s_io = s_io_batrider;
		s_has_tx = 1;
		s_tx.dx = 0x1d4;
		s_tx.dx_flipped = 0x16b;
		s_vdp.no_sprite_buffer = 1;
		s_vdp.tile_cb = br_tile_cb;
		s_zrom = rbx_region("audiocpu", &s_zrom_size);
		s_oki2_rom = rbx_region("oki2", &s_oki2_size);
		if (!s_zrom || !s_oki2_rom) {
			rbx_fail("%s: ROM regions missing", g->set);
			return -1;
		}
		break;
	case M_MAHOUDAI:
	case M_SHIPPUMD:
	case M_BGAREGGA:
		s_cpu_clock = 16000000;
		s_irq_level = 4;
		m68k_bus_map(0, s_m == M_MAHOUDAI ? 0x7ffff : 0xfffff, s_rom, s_rom_size, 1, 0);
		m68k_bus_map(0x100000, 0x10ffff, b.ram, 0x10000, 1, 1);
		m68k_bus_map(0x400000, 0x400fff, b.pal, 0x1000, 1, 1);
		if (s_m != M_BGAREGGA)
			m68k_bus_map(0x401000, 0x4017ff, b.ram2, 0x800, 1, 1);
		m68k_bus_map(0x503200, 0x503fff, b.txgfx, 0xe00, 1, 1);
		s_io = s_io_raizing;
		s_has_tx = 1;
		s_tx.dx = 0x1d4;
		s_tx.dx_flipped = 0x16b;
		s_text = rbx_region("text", &s_text_size);
		s_zrom = rbx_region("audiocpu", &s_zrom_size);
		if (!s_zrom || !s_text) {
			rbx_fail("%s: ROM regions missing", g->set);
			return -1;
		}
		break;
	default:   // the V25 boards
		s_cpu_clock = s_m == M_VFIVE ? 10000000 : s_m == M_DOGYUUN ? 12500000 : 16000000;
		s_v25_clock = s_cpu_clock;
		s_irq_level = 4;
		m68k_bus_map(0x100000, s_m == M_BATSUGUN ? 0x10ffff : 0x103fff, b.ram, s_m == M_BATSUGUN ? 0x10000 : 0x4000, 1, 1);
		m68k_bus_map(0x400000, 0x400fff, b.pal, 0x1000, 1, 1);
		s_io = s_m == M_KBASH ? s_io_kbash : s_m == M_FIXEIGHT ? s_io_fixeight : s_io_vfive;
		s_rst_bit = s_m == M_DOGYUUN ? 5 : 4;
		if (s_m == M_FIXEIGHT) {
			m68k_bus_map(0x600000, 0x60ffff, b.txgfx, 0x10000, 1, 1);
			s_has_tx = 1;
			s_tx.dx = 0x1d5;
			s_tx.dx_flipped = 0x16a;
		}
		break;
	}
	if (s_v25) {
		s_zrom = rbx_region("audiocpu", &s_zrom_size);
		if (s_m == M_KBASH && !s_zrom) {
			rbx_fail("%s: audiocpu missing", g->set);
			return -1;
		}
		nec_init(&s_nec, NEC_V35, s_m == M_BATSUGUN ? NULL :
		         s_m == M_FIXEIGHT ? toaplan_ts001turbo_decryption_table : toaplan_nitro_decryption_table);   // (batsugun: not encrypted)
		s_nec.read8 = v25_r8;
		s_nec.read16 = v25_r16;
		s_nec.write8 = v25_w8;
		s_nec.write16 = v25_w16;
		s_nec.port_in = v25_port_in;
		s_nec.port_out = v25_port_out;
		v25_map();
		// (the DIP switches on the V25's ports: PT, P0, P1)
		s_dsw_pt = rbx_port_find(s_m == M_DOGYUUN ? "DSWB" : "DSWA");
		s_dsw_p0 = rbx_port_find(s_m == M_DOGYUUN ? "DSWA" : "DSWB");
		s_dsw_p1 = rbx_port_find("JMPR");
		s_in_port[0] = rbx_port_find("IN1");
		s_in_port[1] = rbx_port_find("IN2");
		s_in_port[2] = rbx_port_find("IN3");
	}
	// fixeight's EEPROM: its region first, then the saved file
	s_has_eeprom = s_m == M_FIXEIGHT;
	s_nv_path[0] = 0;
	eeprom_init(&s_eeprom);
	if (s_has_eeprom) {
		uint32_t es;
		const uint8_t *e = rbx_region("eeprom", &es);
		for (uint32_t i = 0; e && i < 64 && 2 * i + 1 < es; i++)
			s_eeprom.data[i] = (uint16_t)(e[2 * i] << 8 | e[2 * i + 1]);
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			eeprom_load(&s_eeprom, s_nv_path);
		}
	}
	s_tx.color_base = 0x400;
	s_tx.screen_w = 432;
	s_tx.pix = s_m == M_BATRIDER ? tx_pix_batrider : IS_RAIZING(s_m) ? text_pix : tx_pix_ram;
	s_tx.user = b.txgfx;
	s_ymclock = (s_m == M_BGAREGGA || s_m == M_BATRIDER) ? 4000000 : 27000000 / 8;
	for (int i = 0; s_io[i].kind != 0xff && i < 32; i++)
		s_ioport[i] = s_io[i].port ? rbx_port_find(s_io[i].port) : -1;
	s_snap = calloc(1, sizeof(*s_snap));
	// sound: YM2151 27 MHz / 8 and an OKI 4 MHz pin 7 low, both 0.5
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = s_ymclock;
	yc.gain[0] = yc.gain[1] = s_m == M_BATRIDER ? 64 : s_m == M_BGAREGGA ? 77 : IS_RAIZING(s_m) ? 90 : 128;   // (0.25 .. 0.5)
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		t2_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 0x800)) {
		rbx_fail("%s: out of memory", g->set);
		t2_exit();
		return -1;
	}
	{	// the OKI: 4 MHz pin 7 low (batsugun 4 MHz low, dogyuun 25 / 24 MHz high, kbash 1 MHz high)
		const uint32_t oc = s_m == M_DOGYUUN ? 25000000 / 24 :
		                    (s_m == M_KBASH || s_m == M_FIXEIGHT || s_m == M_MAHOUDAI || s_m == M_SHIPPUMD) ? 1000000 :
		                    s_m == M_BGAREGGA ? 2000000 : s_m == M_BATRIDER ? 3200000 : 4000000;
		const int pin7 = s_m == M_DOGYUUN || s_m == M_KBASH || s_m == M_FIXEIGHT || IS_RAIZING(s_m);
		oki_init(&s_oki, oc, pin7, s_oki_rom, s_oki_size, s_m == M_BGAREGGA ? 154 : 128);
	}
	if (s_m == M_SNOWBRO2 || s_m == M_ENMADAIO || s_m == M_SHIPPUMD)
		s_oki.read = oki_banked;
	if (s_m == M_BGAREGGA || s_m == M_BATRIDER)
		s_oki.read = oki_raizing;
	if (s_m == M_BATRIDER) {   // the second: pin 7 low, its own banks
		oki_init(&s_oki2, 3200000, 0, s_oki2_rom, s_oki2_size, 128);
		s_oki2.read = oki_raizing;
		s_oki2.user = (void *)(intptr_t)1;
	}
	t2_reset();
	return 0;
}

static int t2_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "t2", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= rbx_state_io(ctx, "vdp", &s_vdp, GP9001_STATE_SIZE);
	if (s_nvdp > 1)
		rc |= rbx_state_io(ctx, "vdp2", &s_vdp2, GP9001_STATE_SIZE);
	if (s_v25)
		rc |= rbx_state_io(ctx, "v25", &s_nec, NEC_STATE_SIZE);
	if (s_has_eeprom)
		rc |= rbx_state_io(ctx, "eeprom", &s_eeprom, sizeof(s_eeprom));
	z80 z = s_z80;
	if (IS_RAIZING(s_m))
		rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "tx", &s_tx, TXTILEMAP_STATE_SIZE);
	rc |= rbx_state_io(ctx, "oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	if (s_m == M_BATRIDER)
		rc |= rbx_state_io(ctx, "oki2", s_oki2.v, sizeof(s_oki2.v) + sizeof(int));
	rc |= rbx_ym_state(0, ctx, load);
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		if (s_v25)
			nec_after_load(&s_nec);
		if (IS_RAIZING(s_m)) {
			z.read_byte = rz_read;
			z.write_byte = rz_write;
			z.port_in = rz_in;
			z.port_out = rz_out;
			z.userdata = NULL;
			z.read_map = s_zmap;
			s_z80 = z;
			if (s_m == M_BGAREGGA || s_m == M_BATRIDER)
				raizing_z80_bank(b.z80_bank);
		}
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int t2_save(void *ctx) { return t2_state(ctx, 0); }
static int t2_load(void *ctx) { return t2_state(ctx, 1); }
static int t2_supported(const char *machine)
{
	for (int i = 0; i < M_COUNT; i++)
		if (!strcmp(machine, s_mnames[i]))
			return 1;
	return 0;
}

const rbx_driver rbx_drv_toaplan2 = { "Toaplan GP9001", t2_init, t2_exit, t2_reset, t2_frame, t2_save, t2_load,
                                      t2_supported, t2_render, t2_render_help };
