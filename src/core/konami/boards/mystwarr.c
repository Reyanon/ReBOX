/*  Konami Mystic Warriors hardware ("Pre-GX"). Ported from MAME src/mame/konami/mystwarr.cpp, mystwarr_v.cpp
    (BSD-3-Clause, copyright-holders: R. Belmont, Phil Stroffolino, Acho A. Tang, Nicola Salmoria) and the devices
    they use (k054321.cpp: Olivier Galibert; k053252.cpp: Luca Elia, David Haywood; eepromser.cpp: Aaron Giles).
    A 68000 at 16 MHz, the K056832 tilemaps, the K055673 sprites, the K055555 / K054338 mixing (the GX mixer of
    konamigx_v.c, as MAME's konamigx_mixer), the K053252 CCU, an ER5911 EEPROM; the sound: a Z80 at 8 MHz, two
    K054539 and the K054321 latches. Dadandarn and Gaiapolis: a K053936 ROZ layer from ROM as the mixer's sub layer 1
    (k053936.cpp's K053936GP_zoom_draw and K053936GP_copyroz32clip: David Haywood), Gaiapolis a K054000.
    docs/KONAMI.md, HANDOFF 22.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../machine/eepromser.h"
#include "../../sound/k054539.h"
#include "../video/k056832.h"
#include "../video/k055673.h"
#include "../video/k053250.h"
#include "konamigx.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define MAIN_CLOCK 16000000u
#define Z80_CLOCK 8000000u
#define SND_RATE 48000
#define VIS_H 224

// ---- the machines (MAME's machine configurations and video starts) ----
enum { MAP_MW, MAP_MM, MAP_MC, MAP_MT, MAP_DD, MAP_GA };   // mystwarr, viostorm, martchmp, metamrph, dadandrn, gaiapols
enum { IRQ_MW, IRQ_MM, IRQ_MC, IRQ_DDD };   // mystwarr_interrupt, metamrph_interrupt, mchamp_interrupt, ddd_interrupt
typedef struct {
	const char *machine;
	int map, irq, layout, dx, dy, tilecb, sprcb, spr_cbshift, vis_x0, vis_y0, vis_w, ccu_clock, chips, in1_custom;
	int offs[4];      // the K056832 layers' x offsets (VIDEO_START)
	double gain[2][8];   // MACHINE_RESET's K054539 gains
	int lvc, y0_init;    // a K053250 (metamrph); the visarea's first line before the CCU sets it (0: 16)
	int roz, roz_xoff;   // the K053936 ROZ's bits per pixel (dadandrn 8, gaiapols 4; 0 none), its K053936GP x offset
	int offs_y0, primode;   // the layer 0's y offset; konamigx_mixer_primode
	int raw_clock, raw_h;   // the screen's set_raw before the CCU sets it (0: 6 MHz, 384)
	int spr_cbmask;
} mw_machine_t;
static const mw_machine_t s_machines[] = {
	{ "mystwarr", MAP_MW, IRQ_MW, K055673_LAYOUT_GX, -48, -24, GX_TILECB_MW, GX_SPRCB_MW, 5, 24, 16, 288, 6000000, 2, 0x08,
	  { -2 - 3, 0 - 3, 2 - 3, 3 - 3 }, { { 0.8, 0.8, 0.8, 0.8, 2, 2, 2, 2 }, { 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 } } },
	{ "viostorm", MAP_MM, IRQ_MM, K055673_LAYOUT_RNG, -62, -23, GX_TILECB_VIOSTORM, GX_SPRCB_METAMRPH, 4, 40, 16, 384, 8000000, 2,
	  0x04, { -2 + 1, 0 + 1, 2 + 1, 3 + 1 }, { { 1, 1, 1, 1, 2, 2, 2, 2 }, { 1, 1, 1, 1, 1, 1, 1, 1 } } },
	{ "martchmp", MAP_MC, IRQ_MC, K055673_LAYOUT_GX, -58, -23, GX_TILECB_5BPP, GX_SPRCB_MARTCHMP, 5, 32, 23, 384, 8000000, 1,
	  0x08, { -2 - 4, 0 - 4, 2 - 4, 3 - 4 }, { { 1, 1, 1, 1, 1.4, 1.4, 1.4, 1.4 }, { 1, 1, 1, 1, 1, 1, 1, 1 } } },
	{ "metamrph", MAP_MT, IRQ_MM, K055673_LAYOUT_RNG, -51, -24, GX_TILECB_VIOSTORM, GX_SPRCB_METAMRPH, 4, 24, 15, 288, 6000000, 2,
	  0x04, { -2 + 4, 0 + 4, 2 + 4, 3 + 4 },
	  { { 0.8, 0.8, 0.8, 0.8, 1.8, 1.8, 1.8, 1.8 }, { 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8 } }, 1, 15 },
	{ "dadandrn", MAP_DD, IRQ_DDD, K055673_LAYOUT_GX, -42, -22, GX_TILECB_5BPP, GX_SPRCB_MW, 3, 24, 17, 288, 6000000, 2, 0x04,
	  { -2 + 4, 0 + 4, 2 + 4, 3 + 4 }, { { 1, 1, 1, 1, 2, 2, 2, 2 }, { 1, 1, 1, 1, 1, 1, 1, 1 } }, 0, 17, 8, -8, 0, 1,
	  0, 0, 0x7f },
	{ "gaiapols", MAP_GA, IRQ_DDD, K055673_LAYOUT_RNG, -61, -22, GX_TILECB_4BPP, GX_SPRCB_GAIAPOLS, 4, 40, 16, 376, 6000000,
	  2, 0x04, { -2 + 2 - 1, 0 + 2, 2 + 2, 3 + 2 }, { { 1, 1, 1, 1, 1, 2, 2, 2 }, { 1, 1, 1, 1, 1, 1, 1, 1 } }, 0, 16, 4,
	  -10, -1, 0, 8000000, 512, 0x7f },
};
static const mw_machine_t *s_m;

// the board's own state (the GX state gx_t holds the video chips' registers and the work RAM)
static struct {
	uint16_t spriteram[0x8000];   // the RAM beside the K053247's (mystwarr 400000, metamrph 211000, martchmp 480000)
	uint8_t k53250[0x4000 + 0x10];   // (viostorm: the K053250's RAM and registers, plain RAM)
	uint16_t prot[0x20];          // the K055550 / K053990 protection's registers
	uint16_t roz_ctrl[16], roz_line[0x800];   // (dadandrn, gaiapols: the K053936's registers and line RAM)
	uint16_t roz_clip;            // (ddd_053936_clip_w)
	int16_t roz_cliprect[4];      // (K053936GP_set_cliprect: min x, max x, min y, max y)
	uint8_t roz_clip_on, roz_enable, roz_rombank;
	uint8_t k54000[0x20];         // (gaiapols)
	uint8_t zram[0x2000], zram2[2][0x400];   // the Z80's c000 RAM, e230-e3ff and e630-e7ff
	uint8_t latch[3];             // the K054321's latches (main -> sound 0, 1; sound -> main 2)
	uint8_t k54321_active, k54321_volume;
	uint8_t mw_irq_control, sound_ctrl, sound_nmi_clk, nmi_line, z80_irq;
	uint64_t z_frac, s_frac, z_end;
	uint8_t state_end;
} mw;

static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static z80 s_z;
static const uint8_t *s_zmap[256];
static k054539_t (*s_k539_hp)[2];
#define s_k539 (*s_k539_hp)
static eepser_t s_eeprom;
static int s_port_in0, s_port_in1, s_port_p12, s_port_p34;
static int32_t s_gain_l, s_gain_r;   // the K054321's volume (x 4096)
static k053250_t s_lvc;              // (metamrph)
static const uint8_t *s_roz_map;     // (gfx4: the ROZ's tilemap in ROM)
static uint32_t s_roz_map_size;
static const uint8_t *s_roz_rom;     // (gfx3: its tiles)
static uint32_t s_roz_rom_size;
static uint8_t *s_roz_gfx;           // (decoded: a byte a pixel, 256 a tile)
static uint32_t s_roz_n;
static struct {   // (the ROZ's snapshot for the drawing)
	uint16_t ctrl[16], line[0x800];
	int16_t cliprect[4];
	uint8_t clip_on, enable;
} s_rvs;

// ---- the K053252 CCU (as konamigx.c: the screen timing it programs) ----
static void ccu_res_change(void)
{
	gx_ccu_t *c = &gx.ccu;
	// (MAME's screen keeps its settings for a visible area outside the screen: metamrph's vertical total of 8, its high
	// byte never written)
	const int vis_h = c->vc - c->vfp - c->vbp - c->vsw;
	if (c->hc && c->vc && c->hbp && c->hfp && c->vbp && c->vfp && c->hsw && c->vsw && vis_h > 0 &&
	    s_m->vis_y0 + vis_h <= c->vc) {
		c->lines = c->vc;
		c->line_clocks = c->hc;
		c->clock = gx.ccu_clock;
		c->vis_y0 = s_m->vis_y0;   // (set_offsets(x, y))
		c->vis_y1 = s_m->vis_y0 + c->vc - c->vfp - c->vbp - c->vsw - 1;
		gx_video_set_y0(s_m->vis_y0);   // (the visible area from the CCU's offsets once it set the screen)
	}
}

static void ccu_w(int r, uint8_t data)
{
	gx_ccu_t *c = &gx.ccu;
	c->regs[r & 15] = data;
	switch (r & 15) {
	case 0x00: case 0x01: c->hc = ((c->regs[0] << 8 | c->regs[1]) & 0x3ff) + 1; ccu_res_change(); break;
	case 0x02: case 0x03: c->hfp = (c->regs[2] << 8 | c->regs[3]) & 0x1ff; ccu_res_change(); break;
	case 0x04: case 0x05: c->hbp = (c->regs[4] << 8 | c->regs[5]) & 0x1ff; ccu_res_change(); break;
	case 0x08: case 0x09: c->vc = ((c->regs[8] << 8 | c->regs[9]) & 0x1ff) + 1; ccu_res_change(); break;
	case 0x0a: c->vfp = c->regs[0x0a]; ccu_res_change(); break;
	case 0x0b: c->vbp = c->regs[0x0b] + 1; ccu_res_change(); break;
	case 0x0c: c->vsw = ((c->regs[0x0c] & 0xf0) >> 4) + 1; c->hsw = (c->regs[0x0c] & 0x0f) + 1; ccu_res_change(); break;
	}
}

static uint8_t ccu_r(int r)
{
	r &= 15;
	if (r == 0x0e || r == 0x0f) {   // (VCT read-back)
		const int v = gx.line - gx.ccu.vc;
		return (uint8_t)(r == 0x0e ? (v >> 8) & 1 : v & 0xff);
	}
	return gx.ccu.regs[r];
}

// ---- the K054321 ----
static void k54321_volume(void)   // (propagate_volume: 2 ^ ((volume - 40) / 10))
{
	const double vol = pow(2.0, ((double)mw.k54321_volume - 40.0) / 10.0);
	s_gain_l = (mw.k54321_active & 2) ? (int32_t)(vol * 4096.0 + 0.5) : 0;
	s_gain_r = (mw.k54321_active & 1) ? (int32_t)(vol * 4096.0 + 0.5) : 0;
}

static void k54321_main_w(int r, uint8_t v)
{
	switch (r) {
	case 0x0: mw.k54321_active = v; k54321_volume(); break;
	case 0x2: mw.k54321_volume = 0; k54321_volume(); break;
	case 0x3:
		if (v && mw.k54321_volume < 64) {
			mw.k54321_volume++;
			k54321_volume();
		}
		break;
	case 0x6: mw.latch[0] = v; break;
	case 0x7: mw.latch[1] = v; break;
	}
}

// ---- the sound board a frame behind on the worker (rbx_aux), as moo.c's: the host's K054321 writes and IRQs queued
// with the samples before them; the host reads the sound latch as of the last finished frame ----
#define Q_MAX 2048
enum { Q_ADV, Q_K54321, Q_IRQ };
typedef struct { uint16_t kind, arg; uint32_t v; } q_ev_t;
static q_ev_t (*s_q_hp)[2][Q_MAX];
#define s_q (*s_q_hp)
static int s_qn[2], s_qcur;
static int32_t s_out_l[2048], s_out_r[2048];
static int s_out_n, s_out_pos;
static uint8_t s_pub;   // (the sound -> main latch as of the last job)
static volatile int s_job_frame;

static void q_put(int kind, int arg, uint32_t v)
{
	const int c = s_qcur;
	if (s_qn[c] < Q_MAX) {
		s_q[c][s_qn[c]].kind = (uint16_t)kind;
		s_q[c][s_qn[c]].arg = (uint16_t)arg;
		s_q[c][s_qn[c]].v = v;
		s_qn[c]++;
	}
}

static uint8_t k54321_main_r(int r)
{
	if (r == 0x8)
		return 0;   // (busy_r)
	if (r == 0xa)
		return s_pub;
	return 0xff;
}

// ---- the main CPU's bus ----
static inline int64_t now_cycles(void) { return gx.cycles_done + m68k_cycles_run(); }

static uint16_t mw_rom_word_r(int offset)   // (k056832_device::mw_rom_word_r)
{
	const k056832_t *k = &gx.k56832;
	const uint32_t bank = 10240u * (uint32_t)k->cur_gfx_banks;
	const uint8_t *rom = k->rom;
#define RB(a) ((uint32_t)(a) < k->rom_size ? rom[a] : 0)
	if (k->regsb[2] & 0x8) {   // (only the 2s)
		const int temp = RB((uint32_t)(offset / 4) * 5 + 4 + bank);
		switch (offset % 4) {
		default:
		case 0: return (uint16_t)((temp & 0x80) << 5 | (temp & 0x40) >> 2);
		case 1: return (uint16_t)((temp & 0x20) << 7 | (temp & 0x10));
		case 2: return (uint16_t)((temp & 0x08) << 9 | (temp & 0x04) << 2);
		case 3: return (uint16_t)((temp & 0x02) << 11 | (temp & 0x01) << 4);
		}
	}
	uint32_t addr = (uint32_t)(offset >> 1) * 5;   // (the 0s and 1s)
	if (offset & 1)
		addr += 2;
	addr += bank;
	return (uint16_t)(RB(addr + 1) | RB(addr) << 8);
#undef RB
}

// the devices of the three maps
enum { D_NONE, D_SPR_MW, D_SPR_MC, D_K53247, D_XRAM, D_K53250, D_K55555, D_K55673ROM, D_K55673REG, D_K53246, D_K54338,
       D_K56832, D_K56832B, D_IRQACK, D_EEP_HI, D_EEP_LO, D_EEP_MC, D_MCCTRL, D_P12, D_P34, D_IN0, D_IN1, D_K54321,
       D_SNDIRQ, D_CCU, D_VRAM, D_ROMRB, D_K055550, D_K053990, D_LVCRAM, D_LVCREG, D_LVCROM, D_K54321H, D_DDDEEP,
       D_ROZCTRL, D_ROZLINE, D_ROZCLIP, D_ROZEN, D_ROZROM0, D_ROZROM1, D_ROZROM2, D_K054000 };
typedef struct { uint32_t lo, hi; int dev; } range_t;
static const range_t s_map_mw[] = {
	{ 0x400000, 0x40ffff, D_SPR_MW }, { 0x480000, 0x4800ff, D_K55555 }, { 0x482000, 0x48200f, D_K55673ROM },
	{ 0x482010, 0x48201f, D_K55673REG }, { 0x484000, 0x484007, D_K53246 }, { 0x48a000, 0x48a01f, D_K54338 },
	{ 0x48c000, 0x48c03f, D_K56832 }, { 0x490000, 0x490001, D_EEP_HI }, { 0x494000, 0x494001, D_P12 },
	{ 0x494002, 0x494003, D_P34 }, { 0x496000, 0x496001, D_IN0 }, { 0x496002, 0x496003, D_IN1 },
	{ 0x498000, 0x49801f, D_K54321 }, { 0x49a000, 0x49a001, D_SNDIRQ }, { 0x49c000, 0x49c01f, D_CCU },
	{ 0x49e000, 0x49e007, D_IRQACK }, { 0x600000, 0x603fff, D_VRAM }, { 0x680000, 0x683fff, D_ROMRB }, { 0, 0, D_NONE } };
static const range_t s_map_mm[] = {   // (metamrph_map / viostorm_map: the K053250 as RAM, the K055550 on viostorm)
	{ 0x210000, 0x210fff, D_K53247 }, { 0x211000, 0x21ffff, D_XRAM }, { 0x240000, 0x240007, D_K53246 },
	{ 0x244000, 0x24400f, D_K55673ROM }, { 0x244010, 0x24401f, D_K55673REG }, { 0x24c000, 0x25000f, D_K53250 },
	{ 0x254000, 0x25401f, D_K54338 }, { 0x258000, 0x2580ff, D_K55555 }, { 0x25c000, 0x25c03f, D_K055550 },
	{ 0x260000, 0x26001f, D_CCU }, { 0x264000, 0x264001, D_SNDIRQ }, { 0x268000, 0x26801f, D_K54321 },
	{ 0x26c000, 0x26c007, D_K56832B }, { 0x270000, 0x27003f, D_K56832 }, { 0x274000, 0x274001, D_P12 },
	{ 0x274002, 0x274003, D_P34 }, { 0x278000, 0x278001, D_IN0 }, { 0x278002, 0x278003, D_IN1 },
	{ 0x27c000, 0x27c001, D_EEP_LO }, { 0x300000, 0x307fff, D_VRAM }, { 0x310000, 0x311fff, D_ROMRB }, { 0, 0, D_NONE } };
static const range_t s_map_mt[] = {   // (metamrph_map: the K053250)
	{ 0x210000, 0x210fff, D_K53247 }, { 0x211000, 0x21ffff, D_XRAM }, { 0x240000, 0x240007, D_K53246 },
	{ 0x244000, 0x24400f, D_K55673ROM }, { 0x244010, 0x24401f, D_K55673REG }, { 0x24c000, 0x24ffff, D_LVCRAM },
	{ 0x250000, 0x25000f, D_LVCREG }, { 0x254000, 0x25401f, D_K54338 }, { 0x258000, 0x2580ff, D_K55555 },
	{ 0x260000, 0x26001f, D_CCU }, { 0x264000, 0x264001, D_SNDIRQ }, { 0x268000, 0x26801f, D_K54321 },
	{ 0x26c000, 0x26c007, D_K56832B }, { 0x270000, 0x27003f, D_K56832 }, { 0x274000, 0x274001, D_P12 },
	{ 0x274002, 0x274003, D_P34 }, { 0x278000, 0x278001, D_IN0 }, { 0x278002, 0x278003, D_IN1 },
	{ 0x27c000, 0x27c001, D_EEP_LO }, { 0x300000, 0x303fff, D_VRAM }, { 0x310000, 0x311fff, D_ROMRB },
	{ 0x320000, 0x321fff, D_LVCROM }, { 0, 0, D_NONE } };
static const range_t s_map_mc[] = {
	{ 0x400000, 0x4000ff, D_K55555 }, { 0x402000, 0x40200f, D_K55673ROM }, { 0x402010, 0x40201f, D_K55673REG },
	{ 0x404000, 0x404007, D_K53246 }, { 0x40a000, 0x40a01f, D_K54338 }, { 0x40c000, 0x40c03f, D_K56832 },
	{ 0x40e000, 0x40e03f, D_K053990 }, { 0x410000, 0x410001, D_EEP_MC }, { 0x412000, 0x412001, D_MCCTRL },
	{ 0x414000, 0x414001, D_P12 }, { 0x414002, 0x414003, D_P34 }, { 0x416000, 0x416001, D_IN0 },
	{ 0x416002, 0x416003, D_IN1 }, { 0x418000, 0x41801f, D_K54321 }, { 0x41a000, 0x41a001, D_SNDIRQ },
	{ 0x41c000, 0x41c01f, D_CCU }, { 0x41e000, 0x41e007, D_K56832B }, { 0x480000, 0x483fff, D_SPR_MC },
	{ 0x680000, 0x683fff, D_VRAM }, { 0x700000, 0x703fff, D_ROMRB }, { 0, 0, D_NONE } };
static const range_t s_map_dd[] = {   // (dadandrn_map; gaiapols_map: the K054000 for the K055550)
	{ 0x400000, 0x40ffff, D_SPR_MW }, { 0x410000, 0x413fff, D_VRAM }, { 0x430000, 0x430007, D_K53246 },
	{ 0x440000, 0x443fff, D_ROMRB }, { 0x450000, 0x45000f, D_K55673ROM }, { 0x450010, 0x45001f, D_K55673REG },
	{ 0x460000, 0x46001f, D_ROZCTRL }, { 0x470000, 0x470fff, D_ROZLINE }, { 0x480000, 0x48003f, D_K56832 },
	{ 0x482000, 0x482007, D_K56832B }, { 0x484000, 0x484003, D_ROZCLIP }, { 0x486000, 0x48601f, D_CCU },
	{ 0x488000, 0x4880ff, D_K55555 }, { 0x48a000, 0x48a01f, D_K54321H }, { 0x48c000, 0x48c01f, D_K54338 },
	{ 0x48e000, 0x48e001, D_IN0 }, { 0x48e020, 0x48e021, D_DDDEEP }, { 0x680000, 0x68003f, D_K055550 },
	{ 0x6a0000, 0x6a0001, D_EEP_LO }, { 0x6c0000, 0x6c0001, D_ROZEN }, { 0x6e0000, 0x6e0001, D_SNDIRQ },
	{ 0x800000, 0x87ffff, D_ROZROM0 }, { 0xa00000, 0xa7ffff, D_ROZROM1 }, { 0xc00000, 0xdfffff, D_ROZROM2 }, { 0, 0, D_NONE } };
static const range_t s_map_ga[] = {
	{ 0x400000, 0x40ffff, D_SPR_MW }, { 0x410000, 0x413fff, D_VRAM }, { 0x430000, 0x430007, D_K53246 },
	{ 0x440000, 0x441fff, D_ROMRB }, { 0x450000, 0x45000f, D_K55673ROM }, { 0x450010, 0x45001f, D_K55673REG },
	{ 0x460000, 0x46001f, D_ROZCTRL }, { 0x470000, 0x470fff, D_ROZLINE }, { 0x480000, 0x48003f, D_K56832 },
	{ 0x482000, 0x482007, D_K56832B }, { 0x484000, 0x484003, D_ROZCLIP }, { 0x486000, 0x48601f, D_CCU },
	{ 0x488000, 0x4880ff, D_K55555 }, { 0x48a000, 0x48a01f, D_K54321H }, { 0x48c000, 0x48c01f, D_K54338 },
	{ 0x48e000, 0x48e001, D_IN0 }, { 0x48e020, 0x48e021, D_DDDEEP }, { 0x660000, 0x66003f, D_K054000 },
	{ 0x6a0000, 0x6a0001, D_EEP_LO }, { 0x6c0000, 0x6c0001, D_ROZEN }, { 0x6e0000, 0x6e0001, D_SNDIRQ },
	{ 0x800000, 0x87ffff, D_ROZROM0 }, { 0xa00000, 0xa7ffff, D_ROZROM1 }, { 0xc00000, 0xdfffff, D_ROZROM2 }, { 0, 0, D_NONE } };
static const range_t *s_map;

// the K054000 (k054000_device: the collision of two boxes; gaiapols)
static int k54000_delta(const uint8_t *b)
{
	int res = b[0] << 16 | b[1] << 8 | b[2];
	return (b[3] & 0x80) ? res - (0x100 - b[3]) : res + b[3];
}
static uint8_t k54000_axis(uint32_t ac, uint32_t bc, uint32_t aa, uint32_t ba)
{
	uint8_t res = 0;
	const int32_t sub = (int32_t)(ac - bc);
	if (sub > 511 || sub <= -1024)
		res |= 1;
	if (((uint32_t)abs(sub) & 0x1ff) > ((aa + ba) & 0x1ff))
		res |= 1;
	return res;
}
static uint8_t k54000_r(int r)
{
	if (r != 0x18)
		return 0;
	const uint8_t *k = mw.k54000;
	const uint32_t acx = (uint32_t)k54000_delta(k + 1), acy = (uint32_t)k54000_delta(k + 9);
	const uint32_t bcx = (uint32_t)(k[0x15] << 16 | k[0x16] << 8 | k[0x17]), bcy = (uint32_t)(k[0x11] << 16 | k[0x12] << 8 | k[0x13]);
	return (uint8_t)(k54000_axis(acx, bcx, k[6], k[0x0e]) | k54000_axis(acy, bcy, k[7], k[0x0f]));
}

// the ROZ's ROM read back (ddd_ / gai_053936_tilerom_N_r)
static uint16_t roz_rom_r(int dev, uint32_t o)
{
	const uint8_t *m = s_roz_map;
#define MB(a) ((uint32_t)(a) < s_roz_map_size ? m[a] : 0)
	if (dev == D_ROZROM0)
		return s_m->map == MAP_GA ? (uint16_t)(MB(0x20000 + o) << 8 | MB(0x60000 + o)) : (uint16_t)(MB(o) << 8 | MB(0x40000 + o));
	if (dev == D_ROZROM1)
		return MB(o / 2);
#undef MB
	const uint32_t a = (s_m->map == MAP_GA ? o / 2 : o) + (uint32_t)mw.roz_rombank * 0x100000;
	return (uint16_t)((a < s_roz_rom_size ? s_roz_rom[a] : 0) << 8);
}

static void roz_clip_w(int offset, uint16_t data, uint16_t mask)   // (ddd_053936_clip_w)
{
	if (offset == 1) {
		if (mask & 0xff00)
			mw.roz_clip_on = (data & 0x0100) != 0;
		return;
	}
	const uint16_t old = mw.roz_clip;
	mw.roz_clip = (uint16_t)((data & mask) | (mw.roz_clip & ~mask));
	if (mw.roz_clip == old)
		return;
	const int clip_x = mw.roz_clip & 0x003f, clip_y = (mw.roz_clip & 0x0fc0) >> 6;
	int size_x = (mw.roz_clip & 0x3000) >> 12, size_y = (mw.roz_clip & 0xc000) >> 14;
	size_x = size_x == 3 ? 1 : size_x == 2 ? 2 : 4;
	size_y = size_y == 3 ? 1 : size_y == 2 ? 2 : 4;
	mw.roz_cliprect[0] = (int16_t)(clip_x << 7);
	mw.roz_cliprect[1] = (int16_t)(((clip_x + size_x) << 7) - 1);
	mw.roz_cliprect[2] = (int16_t)(clip_y << 7);
	mw.roz_cliprect[3] = (int16_t)(((clip_y + size_y) << 7) - 1);
}

static inline const range_t *find(uint32_t a)
{
	for (const range_t *r = s_map; r->dev; r++)
		if (a >= r->lo && a <= r->hi)
			return r;
	return NULL;
}

// the K053247's RAM inside the larger RAM (k053247_scattered_word / k053247_martchmp_word): its index, else -1
static inline int spr_index(int dev, uint32_t o)
{
	if (dev == D_SPR_MW)
		return (o & 0x78) ? -1 : (int)((o & 7) | ((o & 0x7f80) >> 4));
	return (o & 0x18) ? -1 : (int)((o & 7) | ((o & 0x1fe0) >> 2));
}

static uint16_t main_r16(uint32_t a)
{
	a &= 0xffffff;
	const range_t *r = find(a);
	if (!r)
		return 0;
	const uint32_t o = (a - r->lo) >> 1;
	switch (r->dev) {
	case D_SPR_MW: case D_SPR_MC: {
		const int i = spr_index(r->dev, o);
		return i < 0 ? mw.spriteram[o & 0x7fff] : k055673_ram_r(&gx.k55673, i);
	}
	case D_K53247: return k055673_ram_r(&gx.k55673, (int)o);
	case D_XRAM: return mw.spriteram[o & 0x7fff];
	case D_K53250: return (uint16_t)(mw.k53250[(a - r->lo) & ~1u] << 8 | mw.k53250[((a - r->lo) & ~1u) + 1]);
	case D_K55673ROM: return k055673_rom_word_r(&gx.k55673, (int)o);
	case D_LVCRAM: return k053250_ram_r(&s_lvc, (int)o);
	case D_LVCREG: return k053250_reg_r(&s_lvc, (int)o);
	case D_LVCROM: return k053250_rom_r(&s_lvc, (int)o);
	case D_P12: return (uint16_t)rbx_port_read(s_port_p12);
	case D_P34: return (uint16_t)rbx_port_read(s_port_p34);
	case D_IN0: return (uint16_t)rbx_port_read(s_port_in0);
	case D_IN1: {   // (eeprom_r: IN1 in the low byte)
		const int64_t t = now_cycles();
		rbx_port_custom(s_port_in1, 0x03, (uint32_t)(eepser_do_read(&s_eeprom) | eepser_ready_read(&s_eeprom, t) << 1));
		rbx_port_custom(s_port_in1, (uint32_t)s_m->in1_custom, 0);
		return (uint16_t)(rbx_port_read(s_port_in1) & 0xff);
	}
	case D_K54321: return k54321_main_r((int)o);
	case D_CCU: return ccu_r((int)o);
	case D_VRAM: return k056832_ram_word_r(&gx.k56832, (int)(o & 0xfff));
	case D_ROMRB: return mw_rom_word_r((int)o);
	case D_K055550: return mw.prot[o & 0x1f];
	case D_K54321H: return (uint16_t)(k54321_main_r((int)o) << 8);
	case D_DDDEEP: {   // (dddeeprom_r: IN1 in the high byte; the low byte alone P2)
		const int64_t t = now_cycles();
		rbx_port_custom(s_port_in1, 0x03, (uint32_t)(eepser_do_read(&s_eeprom) | eepser_ready_read(&s_eeprom, t) << 1));
		rbx_port_custom(s_port_in1, (uint32_t)s_m->in1_custom, 0);
		return (uint16_t)((rbx_port_read(s_port_in1) & 0xff) << 8);
	}
	case D_ROZLINE: return mw.roz_line[o & 0x7ff];
	case D_ROZROM0: case D_ROZROM1: case D_ROZROM2: return roz_rom_r(r->dev, o);
	case D_K054000: return k54000_r((int)(o & 0x1f));
	default: return 0;
	}
}

static uint8_t main_r8(uint32_t a)
{
	if ((a & 0xffffff) == 0x48e021 && (s_m->map == MAP_DD || s_m->map == MAP_GA))
		return (uint8_t)rbx_port_read(s_port_p12);   // (dddeeprom_r, the low byte: P2)
	const uint16_t w = main_r16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}

static void eeprom_lines(int di, int cs, int clk)
{
	const int64_t t = now_cycles();
	eepser_di_write(&s_eeprom, di);
	eepser_cs_write(&s_eeprom, cs, t);
	eepser_clk_write(&s_eeprom, clk, t);
}

static void k055550_w(int offset, uint16_t data, uint16_t mask);
static void k053990_w(int offset, uint16_t data, uint16_t mask);

static void main_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	const range_t *r = find(a);
	if (!r)
		return;
	const uint32_t o = (a - r->lo) >> 1;
	switch (r->dev) {
	case D_SPR_MW: case D_SPR_MC: {
		const int i = spr_index(r->dev, o);
		if (i >= 0)
			k055673_ram_w(&gx.k55673, i, v, mask);
		else
			mw.spriteram[o & 0x7fff] = (uint16_t)((v & mask) | (mw.spriteram[o & 0x7fff] & ~mask));
		break;
	}
	case D_K53247: k055673_ram_w(&gx.k55673, (int)o, v, mask); break;
	case D_XRAM: mw.spriteram[o & 0x7fff] = (uint16_t)((v & mask) | (mw.spriteram[o & 0x7fff] & ~mask)); break;
	case D_K53250: {
		const uint32_t b = (a - r->lo) & ~1u;
		if (mask & 0xff00) mw.k53250[b] = (uint8_t)(v >> 8);
		if (mask & 0x00ff) mw.k53250[b + 1] = (uint8_t)v;
		break;
	}
	case D_K55555: gx.k55555[o & 0x7f] = (uint8_t)(mask == 0x00ff ? v : v >> 8); break;   // (K055555_word_w)
	case D_LVCRAM: k053250_ram_w(&s_lvc, (int)o, v, mask); break;
	case D_LVCREG: k053250_reg_w(&s_lvc, (int)o, v, mask, (int)gx.frame); break;
	case D_K55673REG: k055673_reg_word_w(&gx.k55673, (int)o, v, mask); break;
	case D_K53246:
		if (mask & 0xff00) k055673_k053246_w(&gx.k55673, (int)(a - r->lo), (uint8_t)(v >> 8));
		if (mask & 0x00ff) k055673_k053246_w(&gx.k55673, (int)(a - r->lo) + 1, (uint8_t)v);
		break;
	case D_K54338: gx.k54338[o & 15] = (uint16_t)((v & mask) | (gx.k54338[o & 15] & ~mask)); break;
	case D_K56832: k056832_word_w(&gx.k56832, (int)o, v, mask); break;
	case D_K56832B: k056832_b_word_w(&gx.k56832, (int)o, v, mask); break;
	case D_IRQACK:   // (irq_ack_w)
		k056832_b_word_w(&gx.k56832, (int)o, v, mask);
		if (o == 3 && (mask & 0x00ff))
			mw.mw_irq_control = (uint8_t)v;
		break;
	case D_EEP_HI:   // (mweeprom_w: the high byte to EEPROMOUT)
		if (mask & 0xff00)
			eeprom_lines(v >> 8 & 1, v >> 9 & 1, v >> 10 & 1);
		break;
	case D_EEP_LO:   // (mmeeprom_w: the low byte)
		if (mask & 0x00ff)
			eeprom_lines(v & 1, v >> 1 & 1, v >> 2 & 1);
		break;
	case D_EEP_MC:   // (mceeprom_w: the IRQ control too)
		if (mask & 0xff00) {
			mw.mw_irq_control = (uint8_t)(v >> 8);
			eeprom_lines(v >> 8 & 1, v >> 9 & 1, v >> 10 & 1);
		}
		break;
	case D_MCCTRL:   // (mccontrol_w: OBJCHA)
		if (mask & 0xff00)
			gx.k55673.objcha_line = (v & 0x04) != 0;
		break;
	case D_K54321:
		if (mask & 0x00ff)
			q_put(Q_K54321, (int)o, (uint8_t)v);
		break;
	case D_SNDIRQ: q_put(Q_IRQ, 0, 0); break;   // (sound_irq_w: the Z80's IRQ, HOLD_LINE)
	case D_CCU:
		if (mask & 0x00ff)
			ccu_w((int)o, (uint8_t)v);
		break;
	case D_VRAM: k056832_ram_word_w(&gx.k56832, (int)(o & 0xfff), v, mask); break;
	case D_K055550: k055550_w((int)o, v, mask); break;
	case D_K54321H:
		if (mask & 0xff00)
			q_put(Q_K54321, (int)o, (uint8_t)(v >> 8));
		break;
	case D_ROZCTRL: mw.roz_ctrl[o & 15] = (uint16_t)((v & mask) | (mw.roz_ctrl[o & 15] & ~mask)); break;
	case D_ROZLINE: mw.roz_line[o & 0x7ff] = (uint16_t)((v & mask) | (mw.roz_line[o & 0x7ff] & ~mask)); break;
	case D_ROZCLIP: roz_clip_w((int)o, v, mask); break;
	case D_ROZEN:   // (ddd_053936_enable_w)
		if (mask & 0xff00) {
			mw.roz_enable = (v & 0x0100) != 0;
			mw.roz_rombank = (uint8_t)((v & 0xc000) >> 14);
		}
		break;
	case D_K054000:
		if (mask & 0x00ff)
			mw.k54000[o & 0x1f] = (uint8_t)v;
		break;
	case D_K053990: k053990_w((int)o, v, mask); break;
	default: break;
	}
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v)
{
	main_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00);
}

// ---- the protection chips (konamigx_m.cpp K055550_word_w, K053990_martchmp_word_w) ----
static inline uint16_t rd16(uint32_t a) { return (uint16_t)m68k_read_memory_16(a & 0xffffff); }
static inline void wr16(uint32_t a, uint16_t v) { m68k_write_memory_16(a & 0xffffff, v); }
static inline uint8_t rd8(uint32_t a) { return (uint8_t)m68k_read_memory_8(a & 0xffffff); }
static inline void wr8(uint32_t a, uint8_t v) { m68k_write_memory_8(a & 0xffffff, v); }

static void k055550_w(int offset, uint16_t data, uint16_t mask)
{
	uint16_t *p = mw.prot;
	offset &= 0x1f;
	p[offset] = (uint16_t)((data & mask) | (p[offset] & ~mask));
	if (offset != 0 || !(mask & 0xff00))
		return;
	switch (data >> 8) {
	case 0x97:   // memset() (Dadandarn at 0x639dc)
	case 0x9f: {   // memset() (Violent Storm at 0x989c)
		const uint32_t adr = (uint32_t)p[7] << 16 | p[8], bsize = (uint32_t)p[10] << 16 | p[11];
		const uint32_t count = (p[0] & 0xff) + 1u, lim = adr + bsize * count;
		for (uint32_t i = adr; i < lim; i += 2)
			wr16(i, p[0x1a / 2]);
		break;
	}
	case 0x98: {   // memcpy of some kind (viostorm: the "color check" palette)
		const uint32_t dst = (uint32_t)p[7] << 16 | p[8], src = (uint32_t)p[2] << 16 | p[3];
		const uint32_t count = ((uint32_t)p[10] << 16 | p[11]) >> 2;
		for (uint32_t x = 0; x < count; x++) {
			const uint32_t s = src + x * 8, d = dst + x * 4;
			const uint16_t hi = rd16(s), lo = rd16(s + 2);
			wr16(d, hi);
			wr16(d + 2, lo);
		}
		break;
	}
	case 0xa0: {   // update the collision detection table (Violent Storm at 0x018b42)
		const uint32_t cnt = p[0] & 0xff;
		const int skip = p[1] >> (8 - 1);
		const uint32_t adr = (uint32_t)p[2] << 16 | p[3], bsize = (uint32_t)p[5] << 16 | p[6];
		const uint32_t srcend = adr + bsize * cnt, tgtend = srcend + bsize;
		for (uint32_t src = adr; src < srcend; src += bsize) {
			const int cx1 = (int16_t)rd16(src), sx1 = (int16_t)rd16(src + 2), wx1 = (int16_t)rd16(src + 4);
			const int cy1 = (int16_t)rd16(src + 6), sy1 = (int16_t)rd16(src + 8), wy1 = (int16_t)rd16(src + 10);
			const int cz1 = (int16_t)rd16(src + 12), sz1 = (int16_t)rd16(src + 14), wz1 = (int16_t)rd16(src + 16);
			uint32_t c = src + (uint32_t)skip, i = c, tgt = src + bsize;
			for (; c < tgt; c++)
				wr8(c, 0);
			for (; tgt < tgtend; i++, tgt += bsize) {
				int c2 = (int16_t)rd16(tgt), s2 = (int16_t)rd16(tgt + 2), w2 = (int16_t)rd16(tgt + 4);
				if (abs((cx1 + sx1) - (c2 + s2)) >= wx1 + w2)
					continue;
				c2 = (int16_t)rd16(tgt + 6); s2 = (int16_t)rd16(tgt + 8); w2 = (int16_t)rd16(tgt + 10);
				if (abs((cy1 + sy1) - (c2 + s2)) >= wy1 + w2)
					continue;
				c2 = (int16_t)rd16(tgt + 12); s2 = (int16_t)rd16(tgt + 14); w2 = (int16_t)rd16(tgt + 16);
				if (abs((cz1 + sz1) - (c2 + s2)) >= wz1 + w2)
					continue;
				wr8(i, 0x80);   // (collision confirmed)
			}
		}
		break;
	}
	case 0xc0: {   // the object's "homes-in" vector (Violent Storm at 0x03da9e)
		const int dx = (int16_t)p[0xc], dy = (int16_t)p[0xd];
		int i;
		if (dx) {
			if (dy) {
				int angle = (int)((atan((double)dy / dx) * 128.0) / 3.14159265358979323846);
				if (dx < 0)
					angle += 128;
				i = (angle - 0x40) & 0xff;
			} else
				i = (dx > 0) ? 0xc0 : 0x40;
		} else if (dy > 0)
			i = 0;
		else if (dy < 0)
			i = 0x80;
		else
			i = 0;   // (MAME: a random direction)
		p[0x10] = (uint16_t)i;
		break;
	}
	default:   // (0x87 and the others: nothing)
		break;
	}
}

static void k053990_w(int offset, uint16_t data, uint16_t mask)
{
	uint16_t *p = mw.prot;
	offset &= 0x1f;
	p[offset] = (uint16_t)((data & mask) | (p[offset] & ~mask));
	if (offset != 0x0c || !(mask & 0xff00))
		return;
	const int mode = (p[0x0d] << 8 & 0xff00) | (p[0x0f] & 0xff);
	int element_size = 1;
	switch (mode) {
	case 0xffff:   // word copy
		element_size = 2;
		/* fallthrough */
	case 0xff00: {   // byte copy
		uint32_t src = p[0x0] | ((uint32_t)p[0x1] << 16 & 0xff0000), dst = p[0x2] | ((uint32_t)p[0x3] << 16 & 0xff0000);
		int count = p[0x8] >> 8, src_skip = p[0xa] & 0xff, dst_skip = p[0xb] & 0xff;
		if ((p[0x8] & 0xff) == 2)
			count <<= 1;
		src_skip += element_size;
		dst_skip += element_size;
		for (int i = count; i; i--) {
			if (element_size == 1)
				wr8(dst, rd8(src));
			else
				wr16(dst, rd16(src));
			src += (uint32_t)src_skip;
			dst += (uint32_t)dst_skip;
		}
		break;
	}
	case 0x00ff: {   // sprite list modifier
		uint32_t src = p[0x0] | ((uint32_t)p[0x1] << 16 & 0xff0000), dst = p[0x2] | ((uint32_t)p[0x3] << 16 & 0xff0000);
		uint32_t mod = p[0x4] | ((uint32_t)p[0x5] << 16 & 0xff0000);
		const int src_skip = p[0x1] >> 8, dst_skip = p[0x3] >> 8, mod_skip = p[0x5] >> 8, mod_offs = (p[0x8] & 0xff) << 1;
		src += (uint32_t)mod_offs;
		dst += (uint32_t)mod_offs;
		for (int i = 0x100; i; i--) {
			const uint16_t mod_val = rd16(mod);
			mod += (uint32_t)mod_skip;
			const uint16_t d = (uint16_t)(rd16(src) + mod_val);
			src += (uint32_t)src_skip;
			wr16(dst, d);
			dst += (uint32_t)dst_skip;
		}
		break;
	}
	default:
		break;
	}
}

// ---- the sound board (mystwarr_sound_map) ----
static void z_bank(void)
{
	const uint32_t base = (uint32_t)(mw.sound_ctrl & 0xf) * 0x4000;
	for (int p = 0; p < 0x40; p++)
		s_zmap[0x80 + p] = base + (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + base + (uint32_t)p * 256 : NULL;
}

static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_zrom_size ? s_zrom[a] : 0xff;
	if (a < 0xc000) {
		const uint32_t o = (uint32_t)(mw.sound_ctrl & 0xf) * 0x4000 + (a - 0x8000);
		return o < s_zrom_size ? s_zrom[o] : 0xff;
	}
	if (a < 0xe000)
		return mw.zram[a - 0xc000];
	if (a < 0xe230)
		return k054539_read(&s_k539[0], a - 0xe000);
	if (a < 0xe400)
		return mw.zram2[0][a - 0xe000];
	if (a < 0xe630 && s_m->chips < 2)
		return 0;   // (martchmp: noprw)
	if (a < 0xe630)
		return k054539_read(&s_k539[1], a - 0xe400);
	if (a < 0xe800)
		return mw.zram2[1][a - 0xe400];
	if (a == 0xf002)
		return mw.latch[0];
	if (a == 0xf003)
		return mw.latch[1];
	return 0xff;
}

static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xc000)
		return;
	if (a < 0xe000)
		mw.zram[a - 0xc000] = v;
	else if (a < 0xe230)
		k054539_write(&s_k539[0], a - 0xe000, v);
	else if (a < 0xe400)
		mw.zram2[0][a - 0xe000] = v;
	else if (a < 0xe630 && s_m->chips < 2)
		{ }   // (martchmp: noprw)
	else if (a < 0xe630)
		k054539_write(&s_k539[1], a - 0xe400, v);
	else if (a < 0xe800)
		mw.zram2[1][a - 0xe400] = v;
	else if (a == 0xf000)
		mw.latch[2] = v;
	else if (a == 0xf800) {   // (sound_ctrl_w)
		if (!(v & 0x10))
			mw.nmi_line = 0;
		mw.sound_ctrl = v;
		z_bank();
	}
}

static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void k054539_nmi_gen(void *user, int state)   // (the first chip's timer: an NMI on the rising edge)
{
	(void)user;
	if ((mw.sound_ctrl & 0x10) && !mw.sound_nmi_clk && state && !mw.nmi_line) {
		mw.nmi_line = 1;
		z80_gen_nmi(&s_z);
	}
	mw.sound_nmi_clk = (uint8_t)state;
}

// n samples of the sound board: the Z80 a sample's cycles, then the chips (their timer between samples)
static void sound_run(int n)
{
	for (int i = 0; i < n; i++) {
		mw.z_frac += Z80_CLOCK;
		const unsigned long cyc = (unsigned long)(mw.z_frac / SND_RATE);
		mw.z_frac -= (uint64_t)cyc * SND_RATE;
		int line = mw.z80_irq;
		mw.z_end += cyc;   // (the target in absolute cycles: the overshoot of a run comes off the next)
		z80_run(&s_z, (unsigned long)mw.z_end, &line, 1);
		mw.z80_irq = (uint8_t)line;
		int32_t l = 0, r = 0;
		k054539_render(&s_k539[0], &l, &r, 1);
		if (s_m->chips > 1)
			k054539_render(&s_k539[1], &l, &r, 1);
		// (the chips' outputs to the speaker inverted, at the K054321's gains)
		if (s_out_pos < 2048) {
			s_out_l[s_out_pos] = (int32_t)((int64_t)r * s_gain_l >> 12);
			s_out_r[s_out_pos] = (int32_t)((int64_t)l * s_gain_r >> 12);
			s_out_pos++;
		}
	}
}

static void job(void)
{
	if (!__atomic_load_n(&s_job_frame, __ATOMIC_ACQUIRE))
		return;
	const int b = s_qcur ^ 1;
	s_out_pos = 0;
	for (int i = 0; i < s_qn[b]; i++) {
		const q_ev_t *e = &s_q[b][i];
		switch (e->kind) {
		case Q_ADV: sound_run((int)e->v); break;
		case Q_K54321: k54321_main_w(e->arg, (uint8_t)e->v); break;
		case Q_IRQ: mw.z80_irq = 1; break;
		}
	}
	s_qn[b] = 0;
	s_out_n = s_out_pos;
	__atomic_store_n(&s_job_frame, 0, __ATOMIC_RELEASE);
}

static void sound_frame(void)
{
	rbx_aux_sync();
	for (int i = 0; i < s_out_n; i++)
		rbx_audio_put_stereo(s_out_l[i], s_out_r[i]);
	s_out_n = 0;
	s_pub = mw.latch[2];
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- the K053936 ROZ (K053936GP_zoom_draw, K053936GP_copyroz32clip) from the ROM tilemap of 512 x 512 16 x 16 tiles ----
static void roz_capture(void)
{
	memcpy(s_rvs.ctrl, mw.roz_ctrl, sizeof(s_rvs.ctrl));
	memcpy(s_rvs.line, mw.roz_line, sizeof(s_rvs.line));
	memcpy(s_rvs.cliprect, mw.roz_cliprect, sizeof(s_rvs.cliprect));
	s_rvs.clip_on = mw.roz_clip_on;
	s_rvs.enable = mw.roz_enable;
}

static inline uint32_t blend_r32(uint32_t d, uint32_t s, int level)   // (alpha_blend_r32)
{
	return ((((s & 0xff00ff) * (uint32_t)level + (d & 0xff00ff) * (uint32_t)(256 - level)) >> 8) & 0xff00ff) |
	       ((((s & 0x00ff00) * (uint32_t)level + (d & 0x00ff00) * (uint32_t)(256 - level)) >> 8) & 0x00ff00);
}

// the tilemap's 512 x 512 tiles from the ROM (get_ult / gai_936_tile_info) once: the tile's offset in s_roz_gfx (bits
// 0-23), its colour without the SUB1 base (24-28: gaiapols), flip x (29)
static uint32_t *s_roz_tmap;

static void roz_tmap_build(void)
{
	const uint8_t *m = s_roz_map;
#define MB(a) ((uint32_t)(a) < s_roz_map_size ? m[a] : 0)
	for (uint32_t ti = 0; ti < 512 * 512; ti++) {
		uint32_t tile, color = 0, fx = 0;
		if (s_m->roz == 8) {   // (dadandrn: 8 bpp, the colour the SUB1 base alone)
			const uint8_t d1 = MB(ti), d2 = MB(0x40000 + ti);
			tile = (uint32_t)(d2 | (d1 & 0x1f) << 8);
			fx = (d1 & 0x40) != 0;
		} else {   // (gaiapols: 4 bpp)
			const uint8_t d2 = MB(0x20000 + ti), d3 = MB(0x60000 + ti), d1 = MB(ti / 2);
			tile = (uint32_t)(d3 | (d2 & 0x3f) << 8);
			color = (ti & 1) ? (uint32_t)(d1 & 0xf) : (uint32_t)(d1 >> 4 & 0xf);
			if (d2 & 0x80)
				color |= 0x10;
		}
		tile = s_roz_n ? tile % s_roz_n : 0;
		s_roz_tmap[ti] = tile * 256 | color << 24 | fx << 29;
	}
#undef MB
}

// one line of copyroz32clip: the source from (cx, cy) by (incxx, incxy), the destination row dy (MAME's: a line below
// the clip's) from x0 to x1; the pixmap's pixel the colour x granularity + the pen, drawn when the pen isn't 0
static void roz_line(uint32_t *frame, int vis_x0, int vis_y0, int vis_w, int vis_h, int dy, int x0, int x1, uint32_t cx,
                     uint32_t cy, int incxx, int incxy, const uint32_t *pens, int colorbase, int blend, int alpha)
{
	if (dy < vis_y0 || dy >= vis_y0 + vis_h || !s_roz_n)
		return;
	uint32_t *o = frame + (dy - vis_y0) * vis_w - vis_x0;
	const int b8 = s_m->roz == 8;
	const uint32_t cb = b8 ? (uint32_t)colorbase * 256 : (uint32_t)colorbase << 8;   // (gaiapols: (c | base << 4) x 16)
	const uint8_t *gfx = s_roz_gfx;
	const int clip = s_rvs.clip_on;
	const int minx = s_rvs.cliprect[0], maxx = s_rvs.cliprect[1], miny = s_rvs.cliprect[2], maxy = s_rvs.cliprect[3];
#define ROZ_PUT(E, PX, PY) do { \
		const uint32_t e_ = (E); \
		const uint32_t pen_ = gfx[(e_ & 0xffffff) + ((PY) & 15) * 16 + ((e_ >> 29 & 1) ? 15 - ((PX) & 15) : ((PX) & 15))]; \
		if (pen_) { \
			const uint32_t c_ = pens[(cb + (b8 ? 0 : (e_ >> 24 & 0x1f) * 16) + pen_) & 8191]; \
			o[x] = blend ? blend_r32(o[x], c_, alpha) : c_; \
		} \
	} while (0)
	if (incxx == 0x10000 && incxy == 0 && !clip) {   // (a pixel a pixel along a map row: by tiles)
		const uint32_t py = (cy >> 16) & 0x1fff;
		const uint32_t *trow = s_roz_tmap + (py >> 4) * 512;
		uint32_t px = (cx >> 16) & 0x1fff;
		for (int x = x0; x <= x1; x++, px = (px + 1) & 0x1fff)
			ROZ_PUT(trow[px >> 4], px, py);
		return;
	}
	for (int x = x0; x <= x1; x++, cx += (uint32_t)incxx, cy += (uint32_t)incxy) {
		const int srcx = (int)((cx >> 16) & 0x1fff), srcy = (int)((cy >> 16) & 0x1fff);
		if (clip && (srcx < minx || srcx > maxx || srcy < miny || srcy > maxy))
			continue;
		ROZ_PUT(s_roz_tmap[(srcy >> 4) * 512 + (srcx >> 4)], srcx, srcy);
	}
#undef ROZ_PUT
}

static void roz_draw(uint32_t *frame, int vis_x0, int vis_y0, int vis_w, int vis_h, int band_y0, int band_y1,
                     const uint32_t *pens, int colorbase, int blend, int alpha)
{
	if (!s_rvs.enable)   // (the mixer's sub1 NULL: nothing)
		return;
	const uint16_t *ctrl = s_rvs.ctrl;
	const int xoff = s_m->roz_xoff, yoff = 0;
	const int cl = vis_x0, cr = vis_x0 + vis_w - 1, ct = vis_y0, cb = vis_y0 + vis_h - 1;
	if (ctrl[0x07] & 0x0040) {   // ("super" mode: per line)
		const int ya = band_y0 - 1 > ct ? band_y0 - 1 : ct, yb = band_y1 - 1 < cb ? band_y1 - 1 : cb;
		for (int y = ya; y <= yb; y++) {
			const uint16_t *la = s_rvs.line + (((y - yoff) & 0x1ff) << 2);
			uint32_t startx = (uint32_t)((int16_t)(la[0] + ctrl[0x00]) << 8);
			uint32_t starty = (uint32_t)((int16_t)(la[1] + ctrl[0x01]) << 8);
			int incxx = (int16_t)la[2], incxy = (int16_t)la[3];
			if (ctrl[0x06] & 0x8000) incxx <<= 8;
			if (ctrl[0x06] & 0x0080) incxy <<= 8;
			startx -= (uint32_t)(xoff * incxx);
			starty -= (uint32_t)(xoff * incxy);
			startx <<= 5; starty <<= 5; incxx <<= 5; incxy <<= 5;
			startx += (uint32_t)cl * (uint32_t)incxx;
			starty += (uint32_t)cl * (uint32_t)incxy;
			roz_line(frame, vis_x0, vis_y0, vis_w, vis_h, y + 1, cl, cr, startx, starty, incxx, incxy, pens, colorbase, blend,
			         alpha);
		}
	} else {   // ("simple" mode)
		uint32_t startx = (uint32_t)((int16_t)ctrl[0x00] << 8), starty = (uint32_t)((int16_t)ctrl[0x01] << 8);
		int incyx = (int16_t)ctrl[0x02], incyy = (int16_t)ctrl[0x03], incxx = (int16_t)ctrl[0x04], incxy = (int16_t)ctrl[0x05];
		if (ctrl[0x06] & 0x4000) { incyx <<= 8; incyy <<= 8; }
		if (ctrl[0x06] & 0x0040) { incxx <<= 8; incxy <<= 8; }
		startx -= (uint32_t)(yoff * incyx);
		starty -= (uint32_t)(yoff * incyy);
		startx -= (uint32_t)(xoff * incxx);
		starty -= (uint32_t)(xoff * incxy);
		startx <<= 5; starty <<= 5; incxx <<= 5; incxy <<= 5; incyx <<= 5; incyy <<= 5;
		startx += (uint32_t)cl * (uint32_t)incxx + (uint32_t)ct * (uint32_t)incyx;
		starty += (uint32_t)cl * (uint32_t)incxy + (uint32_t)ct * (uint32_t)incyy;
		const int ya = band_y0 - 1 > ct ? band_y0 - 1 : ct, yb = band_y1 - 1 < cb ? band_y1 - 1 : cb;
		startx += (uint32_t)(ya - ct) * (uint32_t)incyx;
		starty += (uint32_t)(ya - ct) * (uint32_t)incyy;
		for (int y = ya; y <= yb; y++, startx += (uint32_t)incyx, starty += (uint32_t)incyy)
			roz_line(frame, vis_x0, vis_y0, vis_w, vis_h, y + 1, cl, cr, startx, starty, incxx, incxy, pens, colorbase, blend,
			         alpha);
	}
}

static uint8_t *roz_decode(const uint8_t *rom, uint32_t size, int bpp, uint32_t *n)   // (bglayout_8bpp / 16x16x4 msb)
{
	const uint32_t bytes = bpp == 8 ? 256 : 128;
	*n = size / bytes;
	uint8_t *g = (uint8_t *)malloc((size_t)*n * 256 + 1);
	if (!g)
		return NULL;
	if (bpp == 8)
		memcpy(g, rom, (size_t)*n * 256);
	else
		for (uint32_t i = 0; i < *n * 128; i++) {
			g[i * 2] = rom[i] >> 4;
			g[i * 2 + 1] = rom[i] & 15;
		}
	return g;
}

// ---- init / frame ----
static void mw_exit(void);
static void mw_reset(void);

static int mw_init(const rbx_game *g)
{
	if (!s_k539_hp && !(s_k539_hp = calloc(1, sizeof(*s_k539_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t pcm_size, tile_size, spr_size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("soundcpu", &s_zrom_size);
	const uint8_t *pcm = rbx_region("k054539", &pcm_size);
	const uint8_t *tiles = rbx_region("k056832", &tile_size);
	const uint8_t *spr = rbx_region("k055673", &spr_size);
	if (!s_rom || !s_zrom || !pcm || !tiles || !spr) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_m = &s_machines[0];
	for (int i = 0; i < (int)(sizeof(s_machines) / sizeof(s_machines[0])); i++)
		if (!strcmp(s_machines[i].machine, g->machine))
			s_m = &s_machines[i];
	memset(&gx, 0, sizeof(gx));
	memset(&mw, 0, sizeof(mw));
	gx.mw = 1;
	gx.pal_entries = 2048;
	gx.tilecb = s_m->tilecb;
	gx.sprcb = s_m->sprcb;
	gx.spr_cbshift = s_m->spr_cbshift;
	gx.spr_cbmask = s_m->spr_cbmask;
	gx.bands = 4;   // (the mixer's frame in four bands: gx_render_help)
	gx.alpha_noinvert = s_m->map == MAP_MC;   // (VIDEO_START martchmp: invert_alpha(0))
	gx.mixmode = s_m->map == MAP_MC ? GX_MIXMODE_MARTCHMP : GX_MIXMODE_ALPHA;
	k056832_init(&gx.k56832, tiles, tile_size, K056832_BPP_5, 0, 0);
	k055673_init(&gx.k55673, spr, spr_size, s_m->layout, s_m->dx, s_m->dy);
	gx_video_set_area(s_m->vis_x0, s_m->vis_w);
	if (s_m->lvc) {   // (K053250(config, m_k053250_1, m_palette, m_screen, -7, 0))
		uint32_t lsize;
		const uint8_t *lrom = rbx_region("k053250_1", &lsize);
		if (!lrom || k053250_init(&s_lvc, lrom, lsize, -7, 0)) {
			rbx_fail("%s: K053250", g->set);
			return -1;
		}
		gx.k53250 = &s_lvc;
	}
	gx_video_set_y0(s_m->y0_init ? s_m->y0_init : 16);   // (the machine's visarea until the CCU sets the screen)
	if (s_m->roz) {   // (gfx3: the ROZ's tiles, gfx4: its tilemap)
		s_roz_rom = rbx_region("gfx3", &s_roz_rom_size);
		s_roz_map = rbx_region("gfx4", &s_roz_map_size);
		if (!s_roz_rom || !s_roz_map || !(s_roz_gfx = roz_decode(s_roz_rom, s_roz_rom_size, s_m->roz, &s_roz_n))) {
			rbx_fail("%s: the K053936's ROMs", g->set);
			return -1;
		}
		if (!(s_roz_tmap = (uint32_t *)malloc(512 * 512 * sizeof(uint32_t)))) {
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
		roz_tmap_build();
		gx.sub1_roz = roz_draw;
		gx.pre_capture = roz_capture;
	}
	// the main CPU
	m68k_init();
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	if (s_m->map == MAP_MC) {   // (martchmp_map: the program, the work RAM, the data ROM, the palette)
		m68k_bus_map(0x000000, 0x0fffff, s_rom, s_rom_size < 0x100000 ? s_rom_size : 0x100000, 1, 0);
		m68k_bus_map(0x100000, 0x10ffff, gx.workram, 0x10000, 1, 1);
		if (s_rom_size > 0x300000)
			m68k_bus_map(0x300000, 0x3fffff, s_rom + 0x300000, s_rom_size - 0x300000, 1, 0);
		m68k_bus_map(0x600000, 0x601fff, gx.palram, 0x2000, 1, 1);
		s_map = s_map_mc;
	} else if (s_m->map == MAP_DD || s_m->map == MAP_GA) {   // (dadandrn_map / gaiapols_map)
		m68k_bus_map(0x000000, s_m->map == MAP_GA ? 0x2fffff : 0x1fffff, s_rom, s_rom_size, 1, 0);
		m68k_bus_map(0x420000, 0x421fff, gx.palram, 0x2000, 1, 1);
		m68k_bus_map(0x600000, 0x60ffff, gx.workram, 0x10000, 1, 1);
		s_map = s_m->map == MAP_GA ? s_map_ga : s_map_dd;
	} else {
		m68k_bus_map(0x000000, 0x1fffff, s_rom, s_rom_size, 1, 0);
		m68k_bus_map(0x200000, 0x20ffff, gx.workram, 0x10000, 1, 1);
		m68k_bus_map(s_m->map == MAP_MW ? 0x700000 : 0x330000, s_m->map == MAP_MW ? 0x701fff : 0x331fff, gx.palram, 0x2000, 1, 1);
		s_map = s_m->map == MAP_MW ? s_map_mw : s_m->map == MAP_MT ? s_map_mt : s_map_mm;
	}
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
	// the sound board
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
	const int ddd = s_m->map == MAP_DD || s_m->map == MAP_GA;
	s_port_in0 = rbx_port_find(ddd ? "IN0_P1" : "IN0");
	s_port_in1 = rbx_port_find("IN1");
	const int p13 = s_m->map == MAP_MM || s_m->map == MAP_MT;   // (viostorm's and metamrph's ports: P1_P3, P2_P4)
	s_port_p12 = rbx_port_find(ddd ? "P2" : p13 ? "P1_P3" : "P1_P2");
	s_port_p34 = rbx_port_find(p13 ? "P2_P4" : "P3_P4");
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(s_m->vis_w, VIS_H, 0x10000) || gx_video_init()) {
		rbx_fail("%s: out of memory", g->set);
		mw_exit();
		return -1;
	}
	for (int i = 0; i < 0x10000; i++) {   // (the bitmap: RGB 565)
		const int r = (i >> 11) & 31, gg = (i >> 5) & 63, b = i & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (gg << 2 | gg >> 4) << 8 | (b << 3 | b >> 2));
	}
	for (int l = 0; l < 4; l++)   // (VIDEO_START: the layer offsets)
		k056832_set_layer_offs(&gx.k56832, l, s_m->offs[l], l ? 0 : s_m->offs_y0);
	if (s_m->primode)
		gx_video_set_modes(0, s_m->primode);   // (konamigx_mixer_primode)
	mw_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void mw_exit(void)
{
	eepser_nv_save(&s_eeprom);
	rbx_aux_set_job(NULL);
	gx_video_exit();
	k053250_exit(&s_lvc);
	gx.k53250 = NULL;
	free(s_roz_gfx);
	free(s_roz_tmap);
	s_roz_gfx = NULL;
	s_roz_tmap = NULL;
	s_roz_n = 0;
	gx.sub1_roz = NULL;
	gx.pre_capture = NULL;
	free(s_k539_hp);
	s_k539_hp = NULL;
	free(s_q_hp);
	s_q_hp = NULL;
}

static void mw_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_pub = 0;
	s_job_frame = 0;
	memset(gx.workram, 0, sizeof(gx.workram));
	memset(&gx.ccu, 0, sizeof(gx.ccu));
	gx.ccu_clock = (uint32_t)s_m->ccu_clock;
	gx.ccu.lines = 264;   // (set_raw(6000000, 288+16+32+48, 0, 287, 224+16+8+16, 0, 223) until the CCU is set)
	gx.ccu.line_clocks = s_m->raw_h ? s_m->raw_h : 384;
	gx.ccu.clock = s_m->raw_clock ? (uint32_t)s_m->raw_clock : 6000000;
	gx.ccu.vis_y0 = 16;
	gx.ccu.vis_y1 = 239;
	gx_video_set_y0(s_m->y0_init ? s_m->y0_init : 16);
	if (gx.k53250)
		k053250_reset(gx.k53250);
	gx.cycles_done = 0;
	gx.cyc_frac = gx.snd_frac = 0;
	gx.frame = 0;
	gx.wrport1_0 = 0;   // (konamigx_mixer uses it)
	memset(mw.zram, 0, sizeof(mw.zram));
	memset(mw.zram2, 0, sizeof(mw.zram2));
	memset(mw.latch, 0, sizeof(mw.latch));
	mw.mw_irq_control = 0;
	mw.sound_ctrl = 2;   // (MACHINE_START: the bank entry 2)
	mw.sound_nmi_clk = mw.nmi_line = mw.z80_irq = 0;
	mw.z_frac = mw.s_frac = mw.z_end = 0;
	mw.k54321_active = mw.k54321_volume = 0;
	k54321_volume();
	memset(mw.prot, 0, sizeof(mw.prot));
	memset(mw.roz_ctrl, 0, sizeof(mw.roz_ctrl));
	memset(mw.roz_line, 0, sizeof(mw.roz_line));
	memset(mw.roz_cliprect, 0, sizeof(mw.roz_cliprect));
	mw.roz_clip = 0;
	mw.roz_clip_on = mw.roz_enable = mw.roz_rombank = 0;
	memset(mw.k54000, 0, sizeof(mw.k54000));
	for (int i = 0; i < 2; i++) {   // (MACHINE_RESET: the machine's gains)
		k054539_reset(&s_k539[i]);
		for (int c = 0; c < 8; c++)
			k054539_set_gain(&s_k539[i], c, s_m->gain[i][c]);
	}
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
		s_zmap[0xc0 + p] = mw.zram + p * 256;
	z_bank();
	s_z.read_map = s_zmap;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
}

static void mw_frame(void)
{
	const gx_ccu_t *c = &gx.ccu;
	const int lines = c->lines;
	const uint64_t line_cyc_num = (uint64_t)MAIN_CLOCK * (uint64_t)c->line_clocks;
	rbx_audio.frame_mhz = (uint32_t)((uint64_t)c->clock * 1000 / ((uint64_t)c->line_clocks * (uint64_t)lines));
	for (int line = 0; line < lines; line++) {
		gx.line = (line + c->vis_y1 + 1) % lines;   // (the frame from the vblank start)
		if (line == 0)
			gx_video_capture();
		switch (s_m->irq) {   // (the scanline timer)
		case IRQ_MW:   // (mystwarr_interrupt)
			if (mw.mw_irq_control & 1) {
				if (gx.line == 240)
					m68k_bus_hold_irq(2);
				if (gx.line == 0)
					m68k_bus_hold_irq(4);
			}
			break;
		case IRQ_MM:   // (metamrph_interrupt)
			if (gx.line == 24)
				m68k_bus_hold_irq(6);
			if (gx.line == 248 && (k055673_k053246_read_register(&gx.k55673, 5) & 0x10))
				m68k_bus_hold_irq(5);
			break;
		case IRQ_DDD:   // (ddd_interrupt: the vblank)
			if (line == 0)
				m68k_bus_hold_irq(5);
			break;
		default:   // (mchamp_interrupt)
			if (mw.mw_irq_control & 0x40) {
				if (gx.line == 247 && (k055673_k053246_read_register(&gx.k55673, 5) & 0x10))
					m68k_bus_hold_irq(6);
				if (gx.line == 23)
					m68k_bus_hold_irq(2);
			}
			break;
		}
		gx.cyc_frac += line_cyc_num;
		int64_t cyc = (int64_t)(gx.cyc_frac / c->clock);
		gx.cyc_frac -= (uint64_t)cyc * c->clock;
		while (cyc > 0) {
			const int used = m68k_execute((int)cyc);
			gx.cycles_done += used;
			cyc -= used;
		}
		gx.snd_frac += (uint64_t)SND_RATE * (uint64_t)c->line_clocks;
		const int ns = (int)(gx.snd_frac / c->clock);
		gx.snd_frac -= (uint64_t)ns * c->clock;
		if (ns > 0)
			q_put(Q_ADV, 0, (uint32_t)ns);
	}
	sound_frame();
	gx.frame++;
}

static int mw_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	int rc = 0;
	if ((uint32_t)m68k_context_size() > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	rbx_aux_sync();   // (the job done: the board at the frame's end, its samples waiting)
	rc |= rbx_state_io(ctx, "mw_cpu", cpu, (uint32_t)m68k_context_size());
	rc |= rbx_state_io(ctx, "mw_bus", &m68k_bus.irq_pending, sizeof(uint32_t) * 2);
	rc |= rbx_state_io(ctx, "gx", &gx, (uint32_t)offsetof(gx_t, state_end));
	rc |= k056832_state(&gx.k56832, ctx, load);
	rc |= k055673_state(&gx.k55673, ctx, load);
	if (gx.k53250)
		rc |= k053250_state(gx.k53250, ctx, load);
	rc |= rbx_state_io(ctx, "mw", &mw, (uint32_t)offsetof(__typeof__(mw), state_end));
	int32_t qv[2] = { s_out_n, s_pub };
	rc |= rbx_state_io(ctx, "mw_q", qv, sizeof(qv));
	rc |= rbx_state_io(ctx, "mw_outl", s_out_l, sizeof(s_out_l));
	rc |= rbx_state_io(ctx, "mw_outr", s_out_r, sizeof(s_out_r));
	if (load && !rc) {
		s_out_n = qv[0] >= 0 && qv[0] <= 2048 ? qv[0] : 0;
		s_pub = (uint8_t)qv[1];
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
	}
	rc |= eepser_state(&s_eeprom, ctx, load, "mw_eeprom");
	for (int i = 0; i < 2; i++)
		rc |= k054539_state(&s_k539[i], ctx, load, i ? "mw_k539b_" : "mw_k539a_");
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "mw_z80", &z, sizeof(z));
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

static int mw_save(void *ctx) { return mw_state(ctx, 0); }
static int mw_load(void *ctx) { return mw_state(ctx, 1); }
static int mw_supported(const char *machine)
{
	return !strcmp(machine, "mystwarr") || !strcmp(machine, "viostorm") || !strcmp(machine, "martchmp") ||
	       !strcmp(machine, "metamrph") || !strcmp(machine, "dadandrn") || !strcmp(machine, "gaiapols");
}

const rbx_driver rbx_drv_mystwarr = { "Konami Mystic Warriors", mw_init, mw_exit, mw_reset, mw_frame, mw_save, mw_load,
                                      mw_supported, gx_render, gx_render_help };
