/*  Metro boards (Imagetek I4100 / I4220 / I4300): Last Fortress, Pang Pom's, Sky Alert, The Karate Tournament, Lady
    Killer, Poitto!, Dharma Doujou, Toride II, Gun Master, Daitoride, Pururun, Puzzli, Sankokushi, Mouse Shooter GoGo,
    Bal Cube, Bang Bang Ball, Battle Bubble, Blazing Tornado, Grand Striker 2, Varia Metal, Mouja and the mahjong games.
    Ported from MAME 0.289 src/mame/metro/metro.cpp, metro.h, metro_v.cpp (BSD-3-Clause, copyright-holders: Luca Elia,
    David Haywood), src/devices/video/imagetek_i4100.cpp (BSD-3-Clause, copyright-holders: Luca Elia, David Haywood,
    Angelo Salese), src/devices/video/k053936.cpp (BSD-3-Clause, copyright-holders: David Haywood) and
    src/devices/sound/es8712.cpp (BSD-3-Clause, copyright-holders: Quench, David Graves, R. Belmont).
    A 68000 with the Imagetek VDP (three 2048x2048 tilemaps seen through a 512x256 window, tiles through a table,
    8x8 / 16x16, 4 / 8 bpp; 512 zooming sprites; a ROM-to-tilemap blitter), and one of: a uPD78C10 sound CPU with
    YM2413 or YM2151 + OKI M6295 (the serial line from the 68000), a YMF278B, a YM2413 + M6295 on the 68000, a Z80 +
    YM2610 with a K053936 ROZ layer (Blazing Tornado, Grand Striker 2), an ES8712 + MSM6585 + M6295 (Varia Metal).
    The frame is drawn at vblank from a snapshot (the tilemaps' windows, the palette, the sprites), in bands that the
    render helpers share.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/m68k_spin.h"
#include "../../cpu/upd7810/upd7810.h"
#include "../../cpu/z80/z80.h"
#include "../../machine/eepromser.h"
#include "../../sound/okim6295.h"
#include "../../sound/msm5205.h"
#include "../../sound/rbx_ym.h"
#include "../../video/decoband.h"
static m68k_spin_t s_spin;   // (the 68000's polling loops: m68k_spin)

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

extern int (*rbx_m68k_vector_hook)(int level);   // (cpu/m68k_bus.c)

#define LINES 263

// ---- machine configurations ----
enum { V4100, V4220, V4300 };
enum { MAP_BALCUBE, MAP_DAITOA, MAP_BANGBALL, MAP_BATLBUBL, MAP_MSGOGO, MAP_DAITORID, MAP_DHARMA, MAP_KARATOUR,
       MAP_KOKUSHI, MAP_LASTFORT, MAP_LASTFORG, MAP_GAKUSAI, MAP_GAKUSAI2, MAP_DOKYUSP, MAP_DOKYUSEI, MAP_PANGPOMS,
       MAP_POITTO, MAP_SKYALERT, MAP_PURURUN, MAP_TORIDE2G, MAP_BLZNTRND, MAP_MOUJA, MAP_VMETAL };
enum { VB_NORMAL, VB_KARATOUR, VB_SCANLINE };   // vblank_irq, karatour_vblank_irq, bangball_scanline
enum { SND_YMF, SND_UPD2413, SND_UPD2151, SND_2413, SND_2610, SND_ES8712 };

typedef struct {
	const char *name;
	uint8_t map, vdp;
	uint32_t cpu_clock;
	uint16_t w, h;
	uint32_t fps_mhz;
	uint8_t irq_line;           // the 68000 level of the VDP's line (0: the I4300's levels, ipl_w)
	int8_t vbl_lvl, blit_lvl;   // the VDP's interrupt sources
	uint16_t periodic;          // periodic_interrupt (VDP source 4), Hz
	uint8_t vbl, snd;
	uint32_t oki_clock;
	uint8_t oki_pin7;
	uint16_t ym_gain, oki_gain; // 1/256
	int8_t xoff[3];
	int16_t fxoff[3], fyoff[3]; // flipped offsets (default: minus the offsets)
	uint8_t has_flipoff;
	uint8_t ext_irq;            // ext_ctrl_0 = the irq 5 enable
} machine_t;

static const machine_t s_machines[] = {
	{ "msgogo", MAP_MSGOGO, V4220, 16000000, 320, 224, 58233, 1, 0, 2, 60, VB_NORMAL, SND_YMF, 0, 0, 256, 0, { 2, 2, 2 } },
	{ "balcube", MAP_BALCUBE, V4220, 16000000, 320, 224, 58233, 1, 0, 2, 480, VB_NORMAL, SND_YMF, 0, 0, 256, 0, { 2, 2, 2 } },
	{ "daitoa", MAP_DAITOA, V4220, 16000000, 320, 224, 58233, 1, 0, 2, 480, VB_NORMAL, SND_YMF, 0, 0, 256, 0, { 2, 2, 2 } },
	{ "bangball", MAP_BANGBALL, V4220, 16000000, 320, 224, 60000, 1, 0, 2, 0, VB_SCANLINE, SND_YMF, 0, 0, 256, 0, { 2, 2, 2 } },
	{ "batlbubl", MAP_BATLBUBL, V4220, 16000000, 320, 224, 60000, 1, 0, 2, 0, VB_SCANLINE, SND_YMF, 0, 0, 256, 0, { 2, 2, 2 } },
	{ "daitorid", MAP_DAITORID, V4220, 16000000, 320, 224, 58233, 2, 0, 2, 480, VB_NORMAL, SND_UPD2151, 1200000, 1, 205, 102, { 2, 2, 2 } },
	{ "puzzli", MAP_DAITORID, V4220, 16000000, 320, 224, 58233, 2, 0, 2, 0, VB_SCANLINE, SND_UPD2151, 1200000, 1, 205, 102, { 2, 2, 2 } },
	{ "puzzlia", MAP_PURURUN, V4220, 16000000, 320, 224, 58233, 2, 0, 2, 0, VB_SCANLINE, SND_UPD2151, 1200000, 1, 205, 102, { 2, 2, 2 } },
	{ "dharma", MAP_DHARMA, V4220, 12000000, 320, 224, 58233, 2, 0, 2, 480, VB_NORMAL, SND_UPD2413, 1200000, 1, 230, 26 },
	{ "karatour", MAP_KARATOUR, V4100, 12000000, 320, 240, 58233, 2, 0, 2, 480, VB_KARATOUR, SND_UPD2413, 1200000, 1, 230, 26, { 0 }, { 0 }, { 0 }, 0, 1 },
	{ "sankokushi", MAP_KOKUSHI, V4220, 12000000, 320, 240, 58233, 2, 0, 2, 480, VB_KARATOUR, SND_UPD2413, 1200000, 1, 230, 26, { 0 }, { 0 }, { 0 }, 0, 1 },
	{ "lastfort", MAP_LASTFORT, V4100, 12000000, 360, 224, 58233, 2, 0, 2, 480, VB_NORMAL, SND_UPD2413, 1200000, 0, 230, 26 },
	{ "lastforg", MAP_LASTFORG, V4100, 12000000, 360, 224, 58233, 2, 0, 2, 480, VB_KARATOUR, SND_UPD2413, 1200000, 1, 230, 26, { 0 }, { 0 }, { 0 }, 0, 1 },
	{ "dokyusei", MAP_DOKYUSEI, V4300, 16000000, 320, 224, 58233, 0, 1, 3, 0, VB_NORMAL, SND_2413, 1056000, 1, 230, 64 },
	{ "dokyusp", MAP_DOKYUSP, V4300, 16000000, 384, 224, 58233, 0, 1, 3, 0, VB_NORMAL, SND_2413, 2112000, 1, 230, 64 },
	{ "gakusai", MAP_GAKUSAI, V4300, 13333000, 320, 240, 58233, 0, 1, 3, 0, VB_NORMAL, SND_2413, 2112000, 1, 512, 64 },
	{ "gakusai2", MAP_GAKUSAI2, V4300, 13333000, 320, 240, 58233, 0, 1, 3, 0, VB_NORMAL, SND_2413, 2112000, 1, 512, 64 },
	{ "pangpoms", MAP_PANGPOMS, V4100, 12000000, 360, 224, 58233, 2, 0, 2, 480, VB_NORMAL, SND_UPD2413, 1200000, 1, 230, 26 },
	{ "poitto", MAP_POITTO, V4100, 12000000, 360, 224, 58233, 2, 0, 2, 480, VB_NORMAL, SND_UPD2413, 1200000, 1, 230, 26 },
	{ "pururun", MAP_PURURUN, V4220, 12000000, 320, 224, 58233, 2, 0, 2, 480, VB_NORMAL, SND_UPD2151, 1200000, 1, 205, 102 },
	{ "skyalert", MAP_SKYALERT, V4100, 12000000, 360, 224, 58233, 2, 0, 2, 480, VB_NORMAL, SND_UPD2413, 1200000, 0, 230, 26 },
	{ "toride2g", MAP_TORIDE2G, V4220, 12000000, 320, 224, 58233, 2, 0, 2, 480, VB_NORMAL, SND_UPD2413, 1200000, 1, 230, 26 },
	{ "mouja", MAP_MOUJA, V4300, 16000000, 320, 224, 58233, 0, 1, 2, 0, VB_NORMAL, SND_2413, 2062500, 1, 256, 64 },
	{ "vmetal", MAP_VMETAL, V4220, 16000000, 304, 224, 58233, 1, 0, 2, 480, VB_NORMAL, SND_ES8712, 1000000, 1, 256, 128,
	  { 0, 0, 0 }, { 88, 88, 88 }, { 39, 39, 39 }, 1 },
	{ "blzntrnd", MAP_BLZNTRND, V4220, 16000000, 304, 224, 58233, 1, 0, 3, 480, VB_KARATOUR, SND_2610, 0, 0, 0, 0, { 0 }, { 0 }, { 0 }, 0, 1 },
	{ "gstrik2", MAP_BLZNTRND, V4220, 16000000, 304, 224, 58233, 1, 0, 3, 480, VB_KARATOUR, SND_2610, 0, 0, 0, 0, { 0, 8, 0 }, { 0 }, { 0 }, 0, 1 },
};
#define NMACHINES ((int)(sizeof(s_machines) / sizeof(s_machines[0])))

static const machine_t *s_mc;
static int s_gstrik2, s_blz, s_vblank_lines;   // (karatour_vblank_irq's end: 1500 us after its start)

// ---- ROMs, chips ----
static uint8_t *s_rom, *s_gfx, *s_snd_rom, *s_oki_rom, *s_ymf_rom, *s_roz_gfx, *s_es_rom;
static uint32_t s_rom_size, s_gfx_size, s_snd_size, s_oki_size, s_ymf_size, s_roz_size, s_es_size;
static uint32_t s_gfx_elems[4];   // tiles of the four layouts (8x8x4, 8x8x8, 16x16x4, 16x16x8)
static int s_port_in[4], s_port_dsw[2], s_port_key[5], s_port_coins, s_port_joys, s_port_p1p2, s_port_sys;
static int s_custom_port;           // the port with the sound CPU's busy bit (COINS_SOUND), -1 none
static okim6295_t s_oki;
static msm5205_t s_msm;
static eepser_t s_eeprom;
static int s_has_eeprom, s_has_oki;
static z80 s_z80;
static int32_t *s_mix, *s_mix_r;
static int s_mix_n, s_mix_cap;
static uint32_t s_out_rate;
static uint32_t s_vdp_base;
static int64_t s_cyc_base;
static uint32_t s_zdx[64][8];       // the sprites' zoom: screen size and source step (16.16) of 8..64 pixels
static uint16_t s_zsize[64][8];

enum { ME_PA_M68K = RBX_PA_DRV, ME_PA_SND, ME_PA_YM, ME_PA_BLIT, ME_PA_END };
static const char *const s_pa_names[] = { "m68k", "sndcpu", "sound", "blit" };

static struct metro_state {   // (on the heap while the game runs, ADR-040)
	uint16_t ram[0x10000 / 2];
	uint16_t vram[3][0x10000];
	uint16_t misc[0x5000 / 2];      // 0x70000 scratch RAM, 0x72000 palette, 0x74000 sprite RAM
	uint16_t spr_buf[0x800];        // (BUFFERED_SPRITERAM16)
	uint16_t tiletable[0x400];
	uint16_t blit[7];
	uint16_t window[6], scroll[6];
	uint16_t spr_count, spr_pri, spr_xoff, spr_yoff, spr_color, bg, scr_xoff, scr_yoff, rombank, crtc_h, crtc_v;
	uint8_t layer_pri[3], tile_sel[3], blank, flip, crtc_unlock;
	uint8_t req, irq_enable, irq_levels[8], irq_vectors[8], ipl, line_state;
	uint8_t ext_irq;
	int64_t blit_due;               // (68000 clock of the blitter's irq, 0 = none)
	// sound
	uint8_t sound_data, busy, porta, portb, upd_bank;
	uint8_t oki_bank_lo, oki_bank_hi, mouja_bank, latch, z80_bank, z80_nmi;
	uint16_t input_sel;
	uint8_t es_playing, es_trigger, es_bank, es_gate;
	uint32_t es_start, es_end, es_base;
	uint32_t mouja_mhz;
	uint8_t upd_ram[0x800], z80_ram[0x2000];
	uint16_t k36_ctrl[0x10], k36_line[0x800];
	uint64_t cpu_frac, snd_frac, ym_frac, out_frac, per_frac, mouja_frac, msm_frac;
	uint32_t oki_acc;
	int32_t oki_hold;
	uint64_t z80_target;
	int z80_line;
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)
#define PAL (b.misc + 0x1000)
#define SPR (b.misc + 0x2000)
static uint16_t *s_k36;   // the K053936's tilemap RAM (0x40000 bytes)

// ---- drawing snapshot ----
#define WIN_W 66
#define WIN_H 34
typedef struct {
	int x, y, w, h, flipx, flipy, color, pri, zi;
	uint32_t gfx;
} spr_t;
static struct {
	uint16_t win[3][WIN_H][WIN_W];  // the tiles under each layer's window
	int tc0[3], tr0[3];
	uint16_t pal[0x1000];
	uint16_t spr[0x800];
	uint16_t tiletable[0x400];
	uint16_t window[6], scroll[6];
	uint16_t spr_count, spr_pri, spr_xoff, spr_yoff, spr_color, bg, scr_xoff, scr_yoff;
	uint8_t layer_pri[3], tile_sel[3], blank, flip;
	uint16_t k36_ctrl[0x10], k36_line[0x800];
	uint16_t *k36;                 // (its copy, chunks of 0x100 words updated when written)
	spr_t list[512];
	int nlist;
} *s_snap;
static uint8_t s_k36_dirty[0x200];
static uint8_t *s_pri;
static uint16_t s_pal_last[0x1000];
static int s_pal_valid;

static inline int pal5(int v) { return (v << 3) | (v >> 2); }

// ---- the VDP's interrupts ----
static void update_irq(void)
{
	const uint8_t irqs = b.req & ~b.irq_enable;
	if (s_mc->irq_line) {
		const int st = irqs != 0;
		if (st != b.line_state) {
			b.line_state = (uint8_t)st;
			m68k_bus_set_line(s_mc->irq_line, st);
		}
		return;
	}
	int level = 0;   // (I4300: the highest level of the sources up, ipl_w)
	for (int i = 0; i < 8; i++)
		if ((irqs >> i) & 1 && (b.irq_levels[i] & 7) > level)
			level = b.irq_levels[i] & 7;
	if (level != b.ipl) {
		if (b.ipl)
			m68k_bus_set_line(b.ipl, 0);
		b.ipl = (uint8_t)level;
		if (level)
			m68k_bus_set_line(level, 1);
	}
}
static void set_irq(int level)
{
	if (level < 0 || ((b.req >> level) & 1))
		return;
	b.req |= (uint8_t)(1 << level);
	update_irq();
}
static void clear_irq(int level)
{
	if (!((b.req >> level) & 1))
		return;
	b.req &= (uint8_t)~(1 << level);
	update_irq();
}
static int vector_hook(int level) { return b.irq_vectors[level & 7]; }

static int64_t cpu_now(void) { return s_cyc_base + m68k_cycles_run(); }

// ---- the blitter ----
static void blitter_start(void)
{
	const uint32_t tmap = (uint32_t)b.blit[0] << 16 | b.blit[1];
	uint32_t src = (uint32_t)b.blit[2] << 16 | b.blit[3];
	uint32_t dst = (uint32_t)b.blit[4] << 16 | b.blit[5];
	const int shift = (dst & 0x80) ? 0 : 8;
	const uint16_t mask = (dst & 0x80) ? 0x00ff : 0xff00;
	if (tmap < 1 || tmap > 3 || !s_gfx_size)
		return;
	const unsigned prof = rbx_prof_enter(ME_PA_BLIT);
	uint16_t *vr = b.vram[tmap - 1];
	const uint8_t *rom = s_gfx;
	const uint32_t size = s_gfx_size;
	dst >>= 8;
	for (int guard = 0; guard < (1 << 22); guard++) {   // (MAME has no end without a stop opcode)
		src = rbx_wrap(src, size);
		const uint8_t b1 = rom[src++];
		int count = ((~b1) & 0x3f) + 1;
		uint16_t b2;
		switch (b1 >> 6) {
		case 0:
			if (b1 == 0) {   // stop: the irq 500 us later
				b.blit_due = cpu_now() + s_mc->cpu_clock / 2000;
				rbx_prof_leave(prof);
				return;
			}
			while (count--) {
				src = rbx_wrap(src, size);
				b2 = (uint16_t)(rom[src++] << shift);
				dst &= 0xffff;
				vr[dst] = (uint16_t)((vr[dst] & ~mask) | (b2 & mask));
				dst = ((dst + 1) & 0xff) | (dst & ~0xffu);
			}
			break;
		case 1:   // fill with an increasing value
			src = rbx_wrap(src, size);
			b2 = rom[src++];
			while (count--) {
				dst &= 0xffff;
				vr[dst] = (uint16_t)((vr[dst] & ~mask) | ((uint16_t)(b2 << shift) & mask));
				dst = ((dst + 1) & 0xff) | (dst & ~0xffu);
				b2++;
			}
			break;
		case 2:   // fill with a fixed value
			src = rbx_wrap(src, size);
			b2 = (uint16_t)(rom[src++] << shift);
			while (count--) {
				dst &= 0xffff;
				vr[dst] = (uint16_t)((vr[dst] & ~mask) | (b2 & mask));
				dst = ((dst + 1) & 0xff) | (dst & ~0xffu);
			}
			break;
		default:
			if (b1 == 0xc0) {   // to the next line, back to the starting column
				dst += 0x100;
				dst &= ~0xffu;
				dst |= 0xff & (b.blit[5] >> 8);
			} else
				dst += (uint32_t)count;
			break;
		}
	}
	rbx_prof_leave(prof);
}

// ---- the VDP's registers ----
#define COMBINE(r) ((r) = (uint16_t)(((r) & ~mask) | (data & mask)))

static uint16_t gfxrom_r(uint32_t o)
{
	const uint32_t a = o * 2 + 0x10000u * b.rombank;
	if (a + 1 < s_gfx_size)
		return (uint16_t)(s_gfx[a] << 8 | s_gfx[a + 1]);
	return 0xffff;
}

static inline uint32_t rmw_offs(uint32_t o) { return (o & 0x3f) + ((o & ~0x3fu) * 4); }

static uint16_t vdp_r16(uint32_t off)
{
	if (off < 0x60000)
		return b.vram[off >> 17][(off & 0x1ffff) >> 1];
	if (off < 0x70000)
		return gfxrom_r((off - 0x60000) >> 1);
	if (off < 0x75000)
		return b.misc[(off - 0x70000) >> 1];
	if (off < 0x78000) {
		const uint32_t o = off - 0x75000;
		return b.vram[o >> 12][rmw_offs((o & 0xfff) >> 1)];
	}
	if (off < 0x78800)
		return b.tiletable[(off - 0x78000) >> 1];
	const int v = s_mc->vdp;
	if (v == V4300) {
		switch (off) {
		case 0x78808: return b.scr_xoff;
		case 0x7880a: return b.scr_yoff;
		case 0x78832: return b.req;
		}
		if (off >= 0x78850 && off < 0x7885c) return b.scroll[(off - 0x78850) >> 1];
		if (off >= 0x78860 && off < 0x7886c) return b.window[(off - 0x78860) >> 1];
	} else {
		switch (off) {
		case 0x78850: return b.scr_yoff;
		case 0x78852: return b.scr_xoff;
		case 0x788a2: return b.req;
		}
		if (off >= 0x78860 && off < 0x7886c) return b.window[(off - 0x78860) >> 1];
		if (off >= 0x78870 && off < 0x7887c) return b.scroll[(off - 0x78870) >> 1];
	}
	uint32_t s = 0;
	if (v == V4100 && off >= 0x78800 && off < 0x78814)
		s = off - 0x78800;
	else if (v != V4100 && off >= 0x79700 && off < 0x79714)
		s = off - 0x79700;
	else if (v == V4220 && off >= 0x78800 && off < 0x78814 && off != 0x78802)
		s = off - 0x78800;
	else
		return 0;
	switch (s) {
	case 0x0: return b.spr_count;
	case 0x2: return b.spr_pri;
	case 0x4: return b.spr_yoff;
	case 0x6: return b.spr_xoff;
	case 0x8: return b.spr_color;
	case 0x10: return (uint16_t)(b.layer_pri[2] << 4 | b.layer_pri[1] << 2 | b.layer_pri[0]);
	case 0x12: return b.bg;
	}
	return 0;
}

static void screen_ctrl_w(uint16_t data)
{
	b.ext_irq = data >> 8 & 1;   // (ext_ctrl_0: the irq 5 enable of karatour_vblank_irq)
	for (int i = 0; i < 3; i++)
		b.tile_sel[i] = (data >> (5 + i)) & 1;
	b.blank = (data >> 1) & 1;
	b.flip = data & 1;
}

static void spr_reg_w(uint32_t s, uint16_t data, uint16_t mask)
{
	switch (s) {
	case 0x0: COMBINE(b.spr_count); break;
	case 0x2: COMBINE(b.spr_pri); break;
	case 0x4: COMBINE(b.spr_yoff); break;
	case 0x6: COMBINE(b.spr_xoff); break;
	case 0x8: COMBINE(b.spr_color); break;
	case 0x10:
		b.layer_pri[2] = (data >> 4) & 3;
		b.layer_pri[1] = (data >> 2) & 3;
		b.layer_pri[0] = data & 3;
		break;
	case 0x12: COMBINE(b.bg); b.bg &= 0x0fff; break;
	}
}

static void vdp_w16(uint32_t off, uint16_t data, uint16_t mask)
{
	if (off < 0x60000) {
		uint16_t *p = &b.vram[off >> 17][(off & 0x1ffff) >> 1];
		COMBINE(*p);
		return;
	}
	if (off < 0x70000)
		return;
	if (off < 0x75000) {
		uint16_t *p = &b.misc[(off - 0x70000) >> 1];
		COMBINE(*p);
		return;
	}
	if (off < 0x78000) {
		const uint32_t o = off - 0x75000;
		uint16_t *p = &b.vram[o >> 12][rmw_offs((o & 0xfff) >> 1)];
		COMBINE(*p);
		return;
	}
	if (off < 0x78800) {
		uint16_t *p = &b.tiletable[(off - 0x78000) >> 1];
		COMBINE(*p);
		return;
	}
	if (off >= 0x78840 && off < 0x7884e) {
		const int i = (int)(off - 0x78840) >> 1;
		COMBINE(b.blit[i]);
		if (i == 6)
			blitter_start();
		return;
	}
	const int v = s_mc->vdp;
	if (v == V4300) {
		if (off >= 0x78810 && off < 0x78820) {
			if (mask & 0xff)
				b.irq_levels[(off - 0x78810) >> 1] = (uint8_t)data;
			return;
		}
		if (off >= 0x78820 && off < 0x78830) {
			if (mask & 0xff)
				b.irq_vectors[(off - 0x78820) >> 1] = (uint8_t)data;
			return;
		}
		if (off >= 0x78850 && off < 0x7885c) { COMBINE(b.scroll[(off - 0x78850) >> 1]); return; }
		if (off >= 0x78860 && off < 0x7886c) { COMBINE(b.window[(off - 0x78860) >> 1]); return; }
		switch (off) {
		case 0x78800: b.crtc_unlock = data & 1; return;
		case 0x78802: if (b.crtc_unlock) COMBINE(b.crtc_h); return;
		case 0x78804: if (b.crtc_unlock) COMBINE(b.crtc_v); return;
		case 0x78808: COMBINE(b.scr_xoff); return;
		case 0x7880a: COMBINE(b.scr_yoff); return;
		case 0x7880e: screen_ctrl_w(data); return;
		case 0x78830:
			if (mask & 0xff) { b.irq_enable = (uint8_t)data; update_irq(); }
			return;
		case 0x78832:
			if (mask & 0xff && (b.req & (uint8_t)data)) { b.req &= (uint8_t)~(data & 0x1f); update_irq(); }
			return;
		case 0x78870: COMBINE(b.rombank); return;
		}
		if (off >= 0x79700 && off < 0x79714)
			spr_reg_w(off - 0x79700, data, mask);
		return;
	}
	if (off >= 0x78860 && off < 0x7886c) { COMBINE(b.window[(off - 0x78860) >> 1]); return; }
	if (off >= 0x78870 && off < 0x7887c) { COMBINE(b.scroll[(off - 0x78870) >> 1]); return; }
	switch (off) {
	case 0x78850: COMBINE(b.scr_yoff); return;
	case 0x78852: COMBINE(b.scr_xoff); return;
	case 0x78880: if (b.crtc_unlock) COMBINE(b.crtc_v); return;
	case 0x78890: if (b.crtc_unlock) COMBINE(b.crtc_h); return;
	case 0x788a0: b.crtc_unlock = data & 1; return;
	case 0x788a2:
		if (mask & 0xff && (b.req & (uint8_t)data)) { b.req &= (uint8_t)~(data & 0x1f); update_irq(); }
		return;
	case 0x788a4:
		if (mask & 0xff) { b.irq_enable = (uint8_t)data; update_irq(); }
		return;
	case 0x788aa: COMBINE(b.rombank); return;
	case 0x788ac: screen_ctrl_w(data); return;
	}
	if (v == V4100 && off >= 0x78800 && off < 0x78814)
		spr_reg_w(off - 0x78800, data, mask);
	else if (v == V4220 && off >= 0x79700 && off < 0x79714)
		spr_reg_w(off - 0x79700, data, mask);
	else if (v == V4220 && off >= 0x78800 && off < 0x78814 && off != 0x78802)
		spr_reg_w(off - 0x78800, data, mask);
}

// ---- sound: the uPD7810 boards ----
static void sound_data_w(uint8_t v)
{
	b.sound_data = v;
	b.busy = 1;
	upd7810_set_input(UPD7810_NMI, 1);
	upd7810_set_input(UPD7810_NMI, 0);
}

static int upd_rxd(void)
{
	const uint8_t d = b.sound_data;
	b.sound_data >>= 1;
	return d & 1;
}

static uint8_t upd_read(uint16_t a)
{
	if (a < 0x4000)
		return a < s_snd_size ? s_snd_rom[a] : 0xff;
	if (a < 0x8000) {
		const uint32_t o = (uint32_t)b.upd_bank * 0x4000 + (a - 0x4000);
		return o < s_snd_size ? s_snd_rom[o] : 0xff;
	}
	if (a < 0x8800)
		return b.upd_ram[a - 0x8000];
	return 0xff;
}
static void upd_write(uint16_t a, uint8_t v)
{
	if (a >= 0x8000 && a < 0x8800)
		b.upd_ram[a - 0x8000] = v;
}
static uint8_t upd_port_in(int port) { return port == UPD7810_PORTA ? b.porta : 0xff; }

static void upd_portb_2413(uint8_t data)   // upd7810_portb_w
{
	const uint8_t old = b.portb;
	if ((old & 0x80) && !(data & 0x80)) { b.busy = 0; b.portb = data; return; }
	if ((old & 0x20) && !(data & 0x20)) {
		if (!(data & 0x04))
			rbx_ym_write(0, (data >> 1) & 1, b.porta);
		b.portb = data;
		return;
	}
	if ((old & 0x04) && !(data & 0x04) && !(data & 0x10))
		oki_write(&s_oki, b.porta);
	if ((old & 0x08) && !(data & 0x08) && !(data & 0x10))
		b.porta = oki_read(&s_oki);
	b.portb = data;
}

static void upd_portb_2151(uint8_t data)   // daitorid_portb_w
{
	const uint8_t old = b.portb;
	if ((old & 0x80) && !(data & 0x80)) { b.busy = 0; b.portb = data; return; }
	if ((old & 0x40) && !(data & 0x40)) {
		if (!(data & 0x04))
			rbx_ym_write(0, (data >> 1) & 1, b.porta);
		if (!(data & 0x08))
			b.porta = rbx_ym_read(0, (data >> 1) & 1);
		b.portb = data;
		return;
	}
	if ((old & 0x04) && !(data & 0x04) && !(data & 0x10))
		oki_write(&s_oki, b.porta);
	if ((old & 0x08) && !(data & 0x08) && !(data & 0x10))
		b.porta = oki_read(&s_oki);
	b.portb = data;
}

static void upd_port_out(int port, uint8_t v)
{
	switch (port) {
	case UPD7810_PORTA: b.porta = v; break;
	case UPD7810_PORTB:
		if (s_mc->snd == SND_UPD2151)
			upd_portb_2151(v);
		else
			upd_portb_2413(v);
		break;
	case UPD7810_PORTC: b.upd_bank = (v >> 4) & (s_mc->snd == SND_UPD2151 ? 7 : 3); break;
	}
}

static void ym_irq_upd(int state) { upd7810_set_input(UPD7810_INTF2, state); }
static void ym_irq_68k(int state) { m68k_bus_set_line(2, state); }
static void ym_irq_z80(int state) { b.z80_line = state; }

// ---- sound: the mahjong / Mouja OKI banks ----
static uint8_t oki_read_mouja(void *u, uint32_t a)
{
	(void)u;
	const uint32_t o = a < 0x20000 ? a : (uint32_t)b.mouja_bank * 0x20000 + (a - 0x20000);
	return o < s_oki_size ? s_oki_rom[o] : 0;
}
static void oki_bank_set(void)
{
	s_oki.bank = (uint32_t)((b.oki_bank_lo & 7) + (b.oki_bank_hi & 1) * 8) * 0x40000;
}

// ---- sound: the ES8712 (Varia Metal) ----
static void es_irq(int state)
{
	if (b.es_gate)
		m68k_bus_set_line(3, state);
}
static void es_reset(void)
{
	b.es_playing = 0;
	msm5205_reset_w(&s_msm, 1);
	es_irq(0);
}
static void es_write(int off, uint8_t data)
{
	switch (off) {
	case 0: case 1: case 2: {
		const int sh = off * 8;
		b.es_start = ((b.es_start & ~(0xffu << sh)) | (uint32_t)data << sh) & 0xfffff;
		break;
	}
	case 3: case 4: case 5: {
		const int sh = (off - 3) * 8;
		b.es_end = ((b.es_end & ~(0xffu << sh)) | (uint32_t)data << sh) & 0xfffff;
		break;
	}
	case 6:   // play
		if (b.es_start < b.es_end) {
			if (!b.es_playing) {
				b.es_playing = 1;
				msm5205_reset_w(&s_msm, 0);
				es_irq(0);
				b.es_base = b.es_start;
			}
		} else if (b.es_playing) {
			b.es_playing = 0;
			es_irq(1);
		}
		break;
	}
}
static void es_vck(void)   // msm_int
{
	if (!b.es_playing)
		return;
	if (b.es_base >= 0x100000 || b.es_base > b.es_end) {
		b.es_playing = 0;
		msm5205_reset_w(&s_msm, 1);
		es_irq(1);
		b.es_trigger = 0;
		return;
	}
	const uint32_t a = (uint32_t)b.es_bank * 0x100000 + b.es_base;
	const uint8_t v = a < s_es_size ? s_es_rom[a] : 0;
	msm5205_data_w(&s_msm, b.es_trigger ? (v & 0x0f) : (v >> 4));   // (the HCT157: A = the high nibble)
	b.es_trigger ^= 1;
	if (!b.es_trigger)
		b.es_base++;
}

// ---- the K053936 (Blazing Tornado, Grand Striker 2) ----
static void k36_w(uint32_t o, uint16_t data, uint16_t mask)
{
	o &= 0x1ffff;
	COMBINE(s_k36[o]);
	s_k36_dirty[o >> 8] = 1;
}

// ---- inputs ----
static uint16_t port16(int p) { return p >= 0 ? (uint16_t)rbx_port_read(p) : 0xffff; }
static uint16_t in_port(int i)
{
	if (s_custom_port >= 0 && s_port_in[i] == s_custom_port)
		rbx_port_custom(s_custom_port, 0x80, b.busy ? 0x80 : 0);
	return port16(s_port_in[i]);
}
static uint16_t balcube_dsw_r(uint32_t offset)
{
	const uint16_t dsw0 = port16(s_port_dsw[0]), in2 = port16(s_port_in[2]);
	uint16_t result = 0xffff;
	for (int bt = 0; bt < 16; bt++)
		if (!((offset >> bt) & 1))
			result &= (uint16_t)((((dsw0 >> bt) & 1) << 6) | (((in2 >> bt) & 1) << 7));
	return result;
}
static uint16_t mahjong_input_r(void)
{
	uint16_t r = 0x3f;
	for (int i = 0; i < 5; i++)
		if (!((b.input_sel >> (i + 1)) & 1))
			r &= port16(s_port_key[i]);
	return (uint16_t)((r << 1) | 0xff81);
}
static uint16_t soundstatus_r(void) { return b.busy ? 0x00 : 0x01; }

// ---- the 68000's handlers ----
static uint16_t io_r16(uint32_t a)
{
	switch (s_mc->map) {
	case MAP_BALCUBE: case MAP_DAITOA: case MAP_BANGBALL: case MAP_BATLBUBL: case MAP_MSGOGO: {
		uint32_t ymf = 0x300000, dsw = 0x400000, in = 0x500000;
		switch (s_mc->map) {
		case MAP_DAITOA: case MAP_BATLBUBL: case MAP_MSGOGO: ymf = 0x400000; dsw = 0x300000; in = 0x200000; break;
		case MAP_BANGBALL: ymf = 0xb00000; dsw = 0xc00000; in = 0xd00000; break;
		}
		if (a == ymf)
			return rbx_ym_read(0, 0);
		if (a - dsw < 0x20000)
			return balcube_dsw_r((a - dsw) >> 1);
		if (a - in < 8) {
			const uint32_t o = a - in;
			if (s_mc->map == MAP_BATLBUBL)
				return o == 0 ? in_port(1) : o == 2 ? port16(s_port_dsw[0]) : o == 4 ? in_port(0) : in_port(2);
			if (s_mc->map == MAP_MSGOGO)
				return o == 0 ? port16(s_port_coins) : o == 2 ? port16(s_port_joys) : 0;
			return o == 0 ? in_port(0) : o == 2 ? in_port(1) : 0;
		}
		break;
	}
	case MAP_DAITORID: case MAP_DHARMA: case MAP_KOKUSHI: case MAP_POITTO: case MAP_PURURUN: case MAP_TORIDE2G: {
		const uint32_t in = s_mc->map == MAP_PURURUN ? 0x400000 : s_mc->map == MAP_POITTO || s_mc->map == MAP_TORIDE2G
		                    ? 0x800000 : 0xc00000;
		if (a - in < 8) {
			const uint32_t o = a - in;
			return o == 0 ? in_port(0) : o == 2 ? in_port(1) : o == 4 ? port16(s_port_dsw[0])
			       : s_mc->map == MAP_KOKUSHI ? 0 : in_port(2);
		}
		break;
	}
	case MAP_KARATOUR: case MAP_LASTFORG:
		if (a - 0x400000 < 0x10) {
			switch (a - 0x400000) {
			case 0x0: return soundstatus_r();
			case 0x2: return in_port(0);
			case 0x4: return in_port(1);
			case 0x6: return port16(s_port_dsw[0]);
			case 0xa: return port16(s_port_dsw[1]);
			case 0xc: return in_port(2);
			}
			return 0;
		}
		break;
	case MAP_LASTFORT: case MAP_PANGPOMS: case MAP_SKYALERT: {
		const uint32_t base = s_mc->map == MAP_LASTFORT ? 0xc00000 : s_mc->map == MAP_PANGPOMS ? 0x800000 : 0x400000;
		if (a - base < 0x10) {
			switch (a - base) {
			case 0x0: return soundstatus_r();
			case 0x4: return in_port(0);
			case 0x6: return in_port(1);
			case 0x8: return in_port(2);
			case 0xa: return port16(s_port_dsw[0]);
			case 0xc: return port16(s_port_dsw[1]);
			case 0xe: return in_port(3);
			}
			return 0;
		}
		break;
	}
	case MAP_GAKUSAI: case MAP_GAKUSAI2: case MAP_DOKYUSP: case MAP_DOKYUSEI: {
		const uint32_t vb = s_vdp_base;
		if (a == vb + 0x78880) return mahjong_input_r();
		if (a == vb + 0x78882) return in_port(0);
		if (s_mc->map == MAP_DOKYUSEI) {
			if (a == vb + 0x78884) return port16(s_port_dsw[0]);
			if (a == vb + 0x78886) return port16(s_port_dsw[1]);
			if (a == 0xd00000) return oki_read(&s_oki);
			break;
		}
		const uint32_t oki = s_mc->map == MAP_GAKUSAI2 ? 0xb00000 : 0x700000;
		if (a == oki) return oki_read(&s_oki);
		if (s_mc->map == MAP_DOKYUSP) {
			if (a == 0xd00000) {
				eepser_clk_write(&s_eeprom, 0, cpu_now());
				eepser_clk_write(&s_eeprom, 1, cpu_now());
				return (uint16_t)(eepser_do_read(&s_eeprom) & 1);
			}
		} else if (a == (s_mc->map == MAP_GAKUSAI2 ? 0xe00000u : 0xc00000u))
			return (uint16_t)(eepser_do_read(&s_eeprom) & 1);
		break;
	}
	case MAP_BLZNTRND:
		if (a - 0xe00000 < 0x0a) {
			switch (a - 0xe00000) {
			case 0x0: return port16(s_port_dsw[0]);
			case 0x2: return port16(s_port_dsw[1]);
			case 0x4: return in_port(0);
			case 0x6: return in_port(1);
			case 0x8: return in_port(2);
			}
			return 0;
		}
		break;
	case MAP_MOUJA:
		if (a - 0x478880 < 8) {
			const uint32_t o = a - 0x478880;
			return o == 0 ? in_port(0) : o == 2 ? in_port(1) : o == 4 ? port16(s_port_dsw[0]) : in_port(2);
		}
		if (a == 0xd00000)
			return oki_read(&s_oki);
		break;
	case MAP_VMETAL:
		if (a == 0x200000) return port16(s_port_p1p2);
		if (a == 0x200002) return port16(s_port_sys);
		if (a - 0x300000 < 0x20000) return balcube_dsw_r((a - 0x300000) >> 1);
		if (a == 0x400000) return oki_read(&s_oki);
		break;
	}
	if (a - s_vdp_base < 0x80000)
		return vdp_r16(a - s_vdp_base);
	if (s_k36 && a - 0x400000 < 0x40000)
		return s_k36[(a - 0x400000) >> 1];
	return 0;
}

static void io_w16(uint32_t a, uint16_t data, uint16_t mask)
{
	const int lo = (mask & 0x00ff) != 0;
	switch (s_mc->map) {
	case MAP_BALCUBE: case MAP_DAITOA: case MAP_BANGBALL: case MAP_BATLBUBL: case MAP_MSGOGO: {
		const uint32_t ymf = s_mc->map == MAP_BALCUBE ? 0x300000 : s_mc->map == MAP_BANGBALL ? 0xb00000 : 0x400000;
		if (a - ymf < 0x0c) {
			if (lo)
				rbx_ym_write(0, (a - ymf) >> 1, (uint8_t)data);
			return;
		}
		break;
	}
	case MAP_DAITORID: case MAP_DHARMA: case MAP_KOKUSHI: case MAP_POITTO: case MAP_PURURUN: case MAP_TORIDE2G:
	case MAP_KARATOUR: case MAP_LASTFORT: case MAP_LASTFORG: case MAP_PANGPOMS: case MAP_SKYALERT:
		if (a == s_vdp_base + 0x788a8) {
			if (lo)
				sound_data_w((uint8_t)data);
			return;
		}
		break;
	case MAP_GAKUSAI: case MAP_GAKUSAI2: case MAP_DOKYUSP: case MAP_DOKYUSEI: {
		const uint32_t vb = s_vdp_base;
		if (a == vb + 0x78888) { COMBINE(b.input_sel); return; }
		if (a == vb + 0x78836) return;   // (watchdog)
		uint32_t bank_lo, bank_hi, ym, oki;
		switch (s_mc->map) {
		case MAP_GAKUSAI2: bank_lo = 0x900000; bank_hi = 0xa00000; ym = 0xc00000; oki = 0xb00000; break;
		case MAP_DOKYUSEI: bank_lo = 0xa00000; bank_hi = 0x800000; ym = 0xc00000; oki = 0xd00000; break;
		default: bank_lo = 0x500000; bank_hi = s_mc->map == MAP_GAKUSAI ? 0xd00000 : 0xffffffff; ym = 0x600000; oki = 0x700000; break;
		}
		if (!lo)
			break;
		if (a == bank_lo) { b.oki_bank_lo = (uint8_t)data; oki_bank_set(); return; }
		if (a == bank_hi) { b.oki_bank_hi = (uint8_t)data; oki_bank_set(); return; }
		if (a - ym < 4) { rbx_ym_write(0, (a - ym) >> 1, (uint8_t)data); return; }
		if (a == oki) { oki_write(&s_oki, (uint8_t)data); return; }
		if (s_mc->map == MAP_DOKYUSP) {
			if (a == 0xc00000) { eepser_cs_write(&s_eeprom, data & 1, cpu_now()); return; }
			if (a == 0xd00000) {
				eepser_di_write(&s_eeprom, data & 1);
				eepser_clk_write(&s_eeprom, 0, cpu_now());
				eepser_clk_write(&s_eeprom, 1, cpu_now());
				return;
			}
		} else if (a == (s_mc->map == MAP_GAKUSAI2 ? 0xe00000u : 0xc00000u) && s_mc->map != MAP_DOKYUSEI) {
			eepser_di_write(&s_eeprom, data & 1);
			eepser_cs_write(&s_eeprom, (data >> 2) & 1, cpu_now());
			eepser_clk_write(&s_eeprom, (data >> 1) & 1, cpu_now());
			return;
		}
		break;
	}
	case MAP_BLZNTRND:
		if (a - 0x400000 < 0x40000) { k36_w((a - 0x400000) >> 1, data, mask); return; }
		if (a - 0x500000 < 0x1000) { COMBINE(b.k36_line[(a - 0x500000) >> 1]); return; }
		if (a - 0x600000 < 0x20) { COMBINE(b.k36_ctrl[(a - 0x600000) >> 1]); return; }
		if (a == 0xe00002) {   // the sound latch (a byte on either half: the 68000 puts it on both)
			b.latch = (uint8_t)((mask & 0xff00) ? data >> 8 : data);
			z80_gen_nmi(&s_z80);
			return;
		}
		break;
	case MAP_MOUJA:
		if (a == 0x478834) {   // irq_timer_ctrl_w
			const uint32_t d = data & 0xff;
			b.mouja_mhz = 58000 + (0xff - d) * 10000 / 22;
			b.mouja_frac = 0;
			set_irq(0);
			return;
		}
		if (a == 0x478836) return;
		if (a == 0x800000) { if (lo) b.mouja_bank = (data >> 3) & 7; return; }
		if (a - 0xc00000 < 4) { if (lo) rbx_ym_write(0, (a - 0xc00000) >> 1, (uint8_t)data); return; }
		if (a == 0xd00000) { oki_write(&s_oki, (uint8_t)data); return; }
		break;
	case MAP_VMETAL:
		if (a == 0x200000) {
			if (lo) {   // vmetal_control_w
				b.es_gate = (data >> 6) & 1;
				if (!b.es_gate)
					m68k_bus_set_line(3, 0);
				b.es_bank = (data >> 4) & 1;
			}
			return;
		}
		if (a == 0x400000 || a == 0x400002) { if (lo) oki_write(&s_oki, (uint8_t)data); return; }
		if (a - 0x500000 < 0x0e) {
			if (a == 0x500000 && (mask & 0xff00))
				es_reset();
			if (lo)
				es_write((int)(a - 0x500000) >> 1, (uint8_t)data);
			return;
		}
		break;
	}
	if (a - s_vdp_base < 0x80000)
		vdp_w16(a - s_vdp_base, data, mask);
}

static uint8_t r8(uint32_t a)
{
	const uint16_t w = io_r16(a & 0xfffffe);
	return (a & 1) ? (uint8_t)w : (uint8_t)(w >> 8);
}
static uint16_t r16(uint32_t a) { return io_r16(a & 0xfffffe); }
static void w8(uint32_t a, uint8_t v)
{
	if (a & 1)
		io_w16(a & 0xfffffe, (uint16_t)(v | v << 8), 0x00ff);
	else
		io_w16(a, (uint16_t)(v << 8 | v), 0xff00);
}
static void w16(uint32_t a, uint16_t v) { io_w16(a & 0xfffffe, v, 0xffff); }

static void map_vdp(uint32_t base)
{
	s_vdp_base = base;
	for (int i = 0; i < 3; i++)
		m68k_bus_map(base + 0x20000 * (uint32_t)i, base + 0x20000 * (uint32_t)i + 0x1ffff, b.vram[i], 0x20000, 1, 1);
	m68k_bus_map(base + 0x70000, base + 0x74fff, b.misc, sizeof(b.misc), 1, 1);
	m68k_bus_map(base + 0x78000, base + 0x787ff, b.tiletable, sizeof(b.tiletable), 1, 1);
}

static void map_mem(void)
{
	m68k_bus_reset_map();
	m68k_bus.read8 = r8;
	m68k_bus.read16 = r16;
	m68k_bus.write8 = w8;
	m68k_bus.write16 = w16;
	uint32_t rom_end = 0x7ffff, ram = 0xf00000, vdp = 0;
	switch (s_mc->map) {
	case MAP_BALCUBE: vdp = 0x600000; break;
	case MAP_DAITOA: case MAP_BATLBUBL: case MAP_MSGOGO: case MAP_VMETAL: vdp = 0x100000; break;
	case MAP_BANGBALL: vdp = 0xe00000; break;
	case MAP_DAITORID: rom_end = 0x3ffff; vdp = 0x400000; ram = 0x800000; break;
	case MAP_DHARMA: rom_end = 0x3ffff; vdp = 0x800000; ram = 0x400000; break;
	case MAP_KARATOUR: vdp = 0x800000; break;
	case MAP_KOKUSHI: vdp = 0x800000; ram = 0x700000; break;
	case MAP_LASTFORT: rom_end = 0x3ffff; vdp = 0x800000; ram = 0x400000; break;
	case MAP_LASTFORG: rom_end = 0x3ffff; vdp = 0x880000; ram = 0xc00000; break;
	case MAP_GAKUSAI: case MAP_DOKYUSP: vdp = 0x200000; if (s_mc->map == MAP_DOKYUSP) rom_end = 0x3ffff; break;
	case MAP_GAKUSAI2: vdp = 0x600000; break;
	case MAP_DOKYUSEI: rom_end = 0x3ffff; vdp = 0x400000; break;
	case MAP_PANGPOMS: rom_end = 0x3ffff; vdp = 0x400000; ram = 0xc00000; break;
	case MAP_POITTO: rom_end = 0x3ffff; vdp = 0xc00000; ram = 0x400000; break;
	case MAP_SKYALERT: rom_end = 0x3ffff; vdp = 0x800000; ram = 0xc00000; break;
	case MAP_PURURUN: vdp = 0xc00000; ram = 0x800000; break;
	case MAP_TORIDE2G: vdp = 0xc00000; ram = 0x400000; break;
	case MAP_BLZNTRND: rom_end = 0x1fffff; vdp = 0x200000; break;
	case MAP_MOUJA: vdp = 0x400000; break;
	}
	if (s_mc->map == MAP_BATLBUBL || s_mc->map == MAP_VMETAL)
		rom_end = 0xfffff;
	m68k_bus_map(0, rom_end, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(ram, ram + 0xfffff, b.ram, sizeof(b.ram), 1, 1);
	map_vdp(vdp);
	if (s_k36)
		m68k_bus_map(0x400000, 0x43ffff, s_k36, 0x40000, 1, 0);   // (writes mark its chunks: k36_w)
}

// ---- the Z80 (Blazing Tornado, Grand Striker 2) ----
static uint8_t z80_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_snd_size ? s_snd_rom[a] : 0xff;
	if (a < 0xc000) {
		const uint32_t o = (uint32_t)b.z80_bank * 0x4000 + (a - 0x8000);
		return o < s_snd_size ? s_snd_rom[o] : 0xff;
	}
	if (a >= 0xe000)
		return b.z80_ram[a - 0xe000];
	return 0xff;
}
static void z80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xe000)
		b.z80_ram[a - 0xe000] = v;
}
static uint8_t z80_in(z80 *z, uint8_t port)
{
	(void)z;
	if (port == 0x40)
		return b.latch;
	if (port >= 0x80 && port <= 0x83)
		return rbx_ym_read(0, port & 3);
	return 0xff;
}
static void z80_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	if (port == 0x00)
		b.z80_bank = v & 7;
	else if (port >= 0x80 && port <= 0x83)
		rbx_ym_write(0, port & 3, v);
}
static void z80_setup(void)
{
	s_z80.read_byte = z80_read;
	s_z80.write_byte = z80_write;
	s_z80.port_in = z80_in;
	s_z80.port_out = z80_out;
}

// ---- drawing ----
// the bytes of a tile of each layout; every layout steps 32 bytes a tile (charincrement 32*8 bits)
static const uint32_t s_elem_bytes[4] = { 32, 64, 128, 256 };

// one line of a layer (draw_tilemap), pixels [x0, x1)
static void draw_layer_line(int layer, int y, uint16_t *dst, uint8_t *pri, uint8_t pcode)
{
	const int big = s_mc->vdp != V4100 && s_snap->tile_sel[layer];
	const int shift = big ? 4 : 3, tsz = 1 << shift, tmask = tsz - 1;
	const int width = 256 << shift, height = 256 << shift;
	const int ww = width >> 2, wh = height >> 3;
	const int flip = s_snap->flip;
	int dx = s_mc->xoff[layer], dy = 0;
	if (flip) {
		dx = s_mc->has_flipoff ? -s_mc->fxoff[layer] : s_mc->xoff[layer];
		dy = s_mc->has_flipoff ? -s_mc->fyoff[layer] : 0;
	}
	const int sx = s_snap->scroll[layer * 2 + 1] + dx, sy = s_snap->scroll[layer * 2] + dy;
	const int wx = s_snap->window[layer * 2 + 1], wy = s_snap->window[layer * 2];
	const int resy = flip ? (wy + y - sy) : (sy + y - wy);
	const int scrolly = (flip ? (LINES - resy - 1) : resy) & (wh - 1);
	int srcline = (wy + scrolly) & (height - 1);
	const int trow = srcline >> shift;
	srcline &= tmask;
	const uint16_t *wrow = s_snap->win[layer][(trow - s_snap->tr0[layer]) & 0xff];
	const int tc0 = s_snap->tc0[layer];
	const uint16_t *tt = s_snap->tiletable;
	const int ext = s_mc->vdp != V4100;
	const int scrw = 392;
	int x = 0;
	const int x1 = s_mc->w;
	int scrollx = flip ? 0 : (sx - wx) & (ww - 1);
	while (x < x1) {
		int n;
		if (flip) {
			const int resx = wx + x - sx;
			scrollx = (scrw - resx - 1) & (ww - 1);
			n = 1;
		} else {
			n = ww - scrollx;
		}
		const int srccol = (wx + scrollx) & (width - 1);
		const int px = srccol & tmask;
		if (!flip && n > tsz - px)
			n = tsz - px;
		if (n > x1 - x)
			n = x1 - x;
		const uint16_t code = wrow[((srccol >> shift) - tc0) & 0xff];
		if (code & 0x8000) {   // a tile of one colour
			if ((code & 0xf) != 0xf) {
				const uint16_t pen = code & 0x0fff;
				for (int i = 0; i < n; i++) {
					dst[x + i] = pen;
					pri[x + i] = pcode;
				}
			}
		} else {
			const int ti = (code & 0x1ff0) >> 3;
			const uint32_t tile = (uint32_t)tt[ti] << 16 | tt[ti | 1];
			uint16_t color = (uint16_t)((tile & 0x0ff00000) >> 16);
			int g;
			if ((color & 0xf0) == 0xf0 && ext) {
				color &= 0x0f00;
				g = big ? 3 : 1;
			} else
				g = big ? 2 : 0;
			const uint32_t t2 = (tile & 0xfffff) + ((uint32_t)(code & 0xf) << g);
			if (t2 < s_gfx_elems[g]) {
				const int row = (code & 0x2000) ? tmask - srcline : srcline;
				// the run's source columns c0, c0 + step, ... (step -1: flipped)
				const int step = (code & 0x4000) ? -1 : 1;
				const int c0 = step < 0 ? tmask - px : px;
				const uint8_t *src = s_gfx + t2 * 32;
				uint16_t *dd = dst + x;
				uint8_t *pp = pri + x;
				if (g & 1) {   // 8 bpp
					const uint8_t *s = src + row * tsz + c0;
					for (int i = 0; i < n; i++, s += step) {
						const uint8_t d = *s;
						if (d != 0xff) {
							dd[i] = (uint16_t)(d | color);
							pp[i] = pcode;
						}
					}
				} else {
					src += row * (tsz >> 1);
					for (int i = 0, c = c0; i < n; i++, c += step) {
						const uint8_t d = (src[c >> 1] >> ((c & 1) << 2)) & 0xf;
						if (d != 0xf) {
							dd[i] = (uint16_t)(d | color);
							pp[i] = pcode;
						}
					}
				}
			}
		}
		x += n;
		if (!flip)
			scrollx = (scrollx + n) & (ww - 1);
	}
}

// the K053936's line (blzntrnd_state::screen_update: zoom_draw before the VDP's layers)
static void draw_roz_line(int y, uint16_t *dst)
{
	const uint16_t *ctrl = s_snap->k36_ctrl;
	const int xoff = -77, yoff = s_gstrik2 ? -19 : -21;
	uint32_t cx, cy;
	int32_t incxx, incxy;
	if (ctrl[0x07] & 0x0040) {
		const uint16_t *l = s_snap->k36_line + 4 * ((y - yoff) & 0x1ff);
		uint32_t startx = (uint32_t)(256 * (int16_t)(l[0] + ctrl[0x00]));
		uint32_t starty = (uint32_t)(256 * (int16_t)(l[1] + ctrl[0x01]));
		incxx = (int16_t)l[2];
		incxy = (int16_t)l[3];
		if (ctrl[0x06] & 0x8000) incxx *= 256;
		if (ctrl[0x06] & 0x0080) incxy *= 256;
		startx -= (uint32_t)(xoff * incxx);
		starty -= (uint32_t)(xoff * incxy);
		cx = startx << 5;
		cy = starty << 5;
	} else {
		uint32_t startx = (uint32_t)(256 * (int16_t)ctrl[0x00]);
		uint32_t starty = (uint32_t)(256 * (int16_t)ctrl[0x01]);
		int32_t incyx = (int16_t)ctrl[0x02], incyy = (int16_t)ctrl[0x03];
		incxx = (int16_t)ctrl[0x04];
		incxy = (int16_t)ctrl[0x05];
		if (ctrl[0x06] & 0x4000) { incyx *= 256; incyy *= 256; }
		if (ctrl[0x06] & 0x0040) { incxx *= 256; incxy *= 256; }
		startx -= (uint32_t)(yoff * incyx);
		starty -= (uint32_t)(yoff * incyy);
		startx -= (uint32_t)(xoff * incxx);
		starty -= (uint32_t)(xoff * incxy);
		cx = (startx << 5) + (uint32_t)y * ((uint32_t)incyx << 5);
		cy = (starty << 5) + (uint32_t)y * ((uint32_t)incyy << 5);
	}
	const uint32_t ix = (uint32_t)incxx << 5, iy = (uint32_t)incxy << 5;
	const uint16_t *ram = s_snap->k36;
	const uint8_t *gfx = s_roz_gfx;
	const uint32_t ntiles = s_roz_size / (s_gstrik2 ? 256 : 64);
	for (int x = 0; x < s_mc->w; x++, cx += ix, cy += iy) {
		if (cx >= (2048u << 16) || cy >= (4096u << 16))
			continue;
		const uint32_t px = cx >> 16, py = cy >> 16;
		uint32_t code, a;
		if (s_gstrik2) {
			const uint32_t col = px >> 4, row = py >> 4;
			code = (s_snap->k36[((row & 0x40) >> 6) | (col << 1) | ((row & 0x80) << 1) | ((row & 0x3f) << 9)] & 0x7fff) >> 2;
			a = code * 256 + ((px & 8) ? 64 : 0) + ((py & 8) ? 128 : 0) + (py & 7) * 8 + (px & 7);
		} else {
			code = ram[(py >> 3) * 256 + (px >> 3)] & 0x7fff;
			a = code * 64 + (py & 7) * 8 + (px & 7);
		}
		dst[x] = (uint16_t)(0xe00 + (code < ntiles ? gfx[a] : 0));
	}
}

static void draw_sprite(const spr_t *s, int y0, int y1, uint16_t *bmp, int pitch)
{
	const int w = s_mc->w;
	const int is8 = s_mc->vdp != V4100 && s->color == 0xf;
	const uint32_t need = is8 ? (uint32_t)(s->w * s->h) : (uint32_t)(s->w / 2 * s->h);
	if (s->gfx + need - 1 >= s_gfx_size)
		return;
	const int wi = s->w / 8 - 1, hi = s->h / 8 - 1;
	const int ssw = s_zsize[s->zi][wi], ssh = s_zsize[s->zi][hi];
	if (!ssw || !ssh)
		return;
	int dx = s_zdx[s->zi][wi], dy = s_zdx[s->zi][hi];
	int sx = s->x, sy = s->y, ex = sx + ssw, ey = sy + ssh;
	int xb = 0, yi = 0;
	if (s->flipx) { xb = (ssw - 1) * dx; dx = -dx; }
	if (s->flipy) { yi = (ssh - 1) * dy; dy = -dy; }
	if (sx < 0) { xb += -sx * dx; sx = 0; }
	if (sy < y0) { yi += (y0 - sy) * dy; sy = y0; }
	if (ex > w) ex = w;
	if (ey > y1) ey = y1;
	if (ex <= sx)
		return;
	const uint16_t base = (uint16_t)((s_snap->spr_color & 0x0f) << 8);
	const uint16_t color = is8 ? base : (uint16_t)(base + (s->color << 4));
	const uint8_t prival = (uint8_t)s->pri;
	const uint8_t *src0 = s_gfx + s->gfx;
	for (int y = sy; y < ey; y++, yi += dy) {
		const uint32_t rowp = (uint32_t)(yi >> 16) * (uint32_t)s->w;
		uint16_t *d = bmp + (y - y0) * pitch;
		uint8_t *pr = s_pri + y * w;
		int xi = xb;
		if (is8) {
			const uint8_t *src = src0 + rowp;
			for (int x = sx; x < ex; x++, xi += dx) {
				const uint8_t c = src[xi >> 16];
				if (c != 0xff) {
					if (pr[x] <= prival)
						d[x] = (uint16_t)(color + c);
					pr[x] = 0xff;
				}
			}
		} else {
			for (int x = sx; x < ex; x++, xi += dx) {
				const uint32_t p = rowp + (uint32_t)(xi >> 16);
				const uint8_t c = (src0[p >> 1] >> ((p & 1) << 2)) & 0xf;
				if (c != 0xf) {
					if (pr[x] <= prival)
						d[x] = (uint16_t)(color + c);
					pr[x] = 0xff;
				}
			}
		}
	}
}

// the sprite list in drawing order (draw_sprites: priority class 0 first, then the list order)
static void build_sprites(void)
{
	static const uint16_t zoomtable[0x40] = {
		0xAAC,0x800,0x668,0x554,0x494,0x400,0x390,0x334, 0x2E8,0x2AC,0x278,0x248,0x224,0x200,0x1E0,0x1C8,
		0x1B0,0x198,0x188,0x174,0x164,0x154,0x148,0x13C, 0x130,0x124,0x11C,0x110,0x108,0x100,0x0F8,0x0F0,
		0x0EC,0x0E4,0x0DC,0x0D8,0x0D4,0x0CC,0x0C8,0x0C4, 0x0C0,0x0BC,0x0B8,0x0B4,0x0B0,0x0AC,0x0A8,0x0A4,
		0x0A0,0x09C,0x098,0x094,0x090,0x08C,0x088,0x080, 0x078,0x070,0x068,0x060,0x058,0x050,0x048,0x040 };
	s_snap->nlist = 0;
	const int sprites = s_snap->spr_count & 511;
	if (!sprites)
		return;
	const int max_x = (s_snap->scr_xoff + 1) * 2, max_y = (s_snap->scr_yoff + 1) * 2;
	const int xoffs = s_snap->spr_xoff - (s_snap->scr_xoff + 1), yoffs = s_snap->spr_yoff - (s_snap->scr_yoff + 1);
	const int lp_disable = s_snap->spr_pri & 0x8000;
	const int masknum = s_snap->spr_pri & 0x1f, gpri = (s_snap->spr_pri & 0x300) >> 8, masklayer = (s_snap->spr_pri & 0xc00) >> 10;
	spr_t tmp[512];
	uint8_t cls[512];
	int n = 0, cnt[32];
	memset(cnt, 0, sizeof(cnt));
	const uint16_t *src = s_snap->spr;
	int inc = 4;
	if (!lp_disable) {
		src += (sprites - 1) * 4;
		inc = -4;
	}
	for (int j = 0; j < sprites; j++, src += inc) {
		const int x = src[0], curr = (x & 0xf800) >> 11;
		if (curr == 0x1f)
			continue;
		const int y = src[1];
		const uint16_t attr = src[2], code = src[3];
		spr_t *s = &tmp[n];
		s->flipx = (attr & 0x8000) != 0;
		s->flipy = (attr & 0x4000) != 0;
		s->color = (attr & 0xf0) >> 4;
		s->zi = (y & 0xfc00) >> 10;
		const uint32_t zoom = (uint32_t)zoomtable[s->zi] << 8;
		s->w = (((attr >> 11) & 7) + 1) * 8;
		s->h = (((attr >> 8) & 7) + 1) * 8;
		s->gfx = 32u * (((uint32_t)(attr & 0x000f) << 16) + code);
		int pri = gpri;
		if (!lp_disable && curr > masknum)
			pri = masklayer;
		s->pri = 3 - pri;
		int sx = (x & 0x7ff) - xoffs, sy = (y & 0x3ff) - yoffs;
		if (s_snap->flip) {
			s->flipx = !s->flipx;
			sx = max_x - sx - (int)((s->w * zoom) >> 16);
			s->flipy = !s->flipy;
			sy = max_y - sy - (int)((s->h * zoom) >> 16);
		}
		s->x = sx;
		s->y = sy;
		cls[n] = (uint8_t)curr;
		cnt[curr]++;
		n++;
	}
	int start[32], acc = 0;
	for (int i = 0; i < 32; i++) {
		start[i] = acc;
		acc += cnt[i];
	}
	for (int i = 0; i < n; i++)
		s_snap->list[start[cls[i]]++] = tmp[i];
	s_snap->nlist = n;
}

#define BANDS 8
static void me_band(int band)
{
	const int w = s_mc->w, h = s_mc->h;
	const int y0 = h * band / BANDS, y1 = h * (band + 1) / BANDS;
	uint16_t *bmp = rbx_video.bitmap + y0 * rbx_video.pitch;
	for (int y = y0; y < y1; y++) {
		uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
		uint8_t *pr = s_pri + y * w;
		const uint16_t bg = s_snap->bg;
		for (int x = 0; x < w; x++)
			d[x] = bg;
		if (s_blz)
			draw_roz_line(y, d);
		memset(pr, 0, (size_t)w);
		if (s_snap->blank)
			continue;
		for (int p = 3; p >= 0; p--)
			for (int l = 2; l >= 0; l--)
				if (s_snap->layer_pri[l] == p)
					draw_layer_line(l, y, d, pr, (uint8_t)(3 - p));
	}
	if (s_snap->blank)
		return;
	for (int i = 0; i < s_snap->nlist; i++) {
		const spr_t *s = &s_snap->list[i];
		if (s->y < y1 && s->y + 64 * 32 > y0)
			draw_sprite(s, y0, y1, bmp, rbx_video.pitch);
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, me_band };
static int me_help(int helper) { return decoband_help(&s_bands, helper); }

static void me_render(void)
{
	uint32_t *pal = rbx_video.palette;
	for (int i = 0; i < 0x1000; i++) {   // GRBx 555
		const uint16_t v = s_snap->pal[i];
		if (s_pal_valid && v == s_pal_last[i])
			continue;
		s_pal_last[i] = v;
		pal[i] = (uint32_t)(pal5((v >> 6) & 31) << 16 | pal5((v >> 11) & 31) << 8 | pal5((v >> 1) & 31));
	}
	s_pal_valid = 1;
	build_sprites();
	decoband_run(&s_bands);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	for (int l = 0; l < 3; l++) {
		const int big = s_mc->vdp != V4100 && b.tile_sel[l];
		const int shift = big ? 4 : 3;
		const int tc0 = (b.window[l * 2 + 1] >> shift) & 0xff, tr0 = (b.window[l * 2] >> shift) & 0xff;
		s_snap->tc0[l] = tc0;
		s_snap->tr0[l] = tr0;
		for (int r = 0; r < WIN_H; r++) {
			const uint16_t *src = b.vram[l] + ((tr0 + r) & 0xff) * 256;
			uint16_t *d = s_snap->win[l][r];
			for (int c = 0; c < WIN_W; c++)
				d[c] = src[(tc0 + c) & 0xff];
		}
	}
	memcpy(s_snap->pal, PAL, sizeof(s_snap->pal));
	memcpy(s_snap->spr, s_blz ? b.spr_buf : SPR, sizeof(s_snap->spr));
	memcpy(s_snap->tiletable, b.tiletable, sizeof(s_snap->tiletable));
	memcpy(s_snap->window, b.window, sizeof(b.window));
	memcpy(s_snap->scroll, b.scroll, sizeof(b.scroll));
	memcpy(s_snap->layer_pri, b.layer_pri, 3);
	memcpy(s_snap->tile_sel, b.tile_sel, 3);
	s_snap->spr_count = b.spr_count;
	s_snap->spr_pri = b.spr_pri;
	s_snap->spr_xoff = b.spr_xoff;
	s_snap->spr_yoff = b.spr_yoff;
	s_snap->spr_color = b.spr_color;
	s_snap->bg = b.bg;
	s_snap->scr_xoff = b.scr_xoff;
	s_snap->scr_yoff = b.scr_yoff;
	s_snap->blank = b.blank;
	s_snap->flip = b.flip;
	if (s_blz) {
		memcpy(s_snap->k36_ctrl, b.k36_ctrl, sizeof(b.k36_ctrl));
		memcpy(s_snap->k36_line, b.k36_line, sizeof(b.k36_line));
		for (int i = 0; i < 0x200; i++)
			if (s_k36_dirty[i]) {
				s_k36_dirty[i] = 0;
				memcpy(s_snap->k36 + i * 0x100, s_k36 + i * 0x100, 0x200);
			}
	}
	rbx_video_frame_done();
}

// screen_eof (and the vblank callbacks' other parts)
static void screen_eof(void)
{
	set_irq(s_mc->vbl_lvl);
	if (s_blz)
		memcpy(b.spr_buf, SPR, sizeof(b.spr_buf));
}

// ---- frame ----
static void sound_line(void)
{
	const uint32_t fps = s_mc->fps_mhz;
	int n;
	int32_t *out = s_mix + s_mix_n, *out_r = s_mix_r ? s_mix_r + s_mix_n : NULL;
	const int room = s_mix_cap - s_mix_n;
	const unsigned prof = rbx_prof_enter(ME_PA_YM);
	switch (s_mc->snd) {
	case SND_ES8712:
		n = rbx_line_cycles(&b.out_frac, s_out_rate, fps, LINES);
		if (n > room)
			n = room;
		for (int i = 0; i < n; i++) {
			msm5205_advance(&s_msm, 640000 / 32000);   // (the output rate: 20 master clocks a sample)
			out[i] += msm5205_output(&s_msm);
		}
		break;
	default: {
		uint32_t clock = s_mc->snd == SND_YMF ? 33868800 : s_mc->snd == SND_2610 ? 8000000
		                 : s_mc->snd == SND_UPD2151 ? 3579545 : 3579545;
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&b.ym_frac, clock, fps, LINES);
		if (out_r)
			n = rbx_ym_advance_lr(0, clocks, out, out_r, room);
		else
			n = rbx_ym_advance(0, clocks, out, room);
		break;
	}
	}
	if (s_has_oki)
		for (int i = 0; i < n; i++) {
			b.oki_acc += s_oki.rate;
			while (b.oki_acc >= s_out_rate) {
				b.oki_acc -= s_out_rate;
				b.oki_hold = oki_sample(&s_oki);
			}
			out[i] += b.oki_hold;
		}
	rbx_prof_leave(prof);
	s_mix_n += n;
}

static void me_frame(void)
{
	const machine_t *mc = s_mc;
	rbx_audio.frame_mhz = mc->fps_mhz;
	const int upd = mc->snd == SND_UPD2413 || mc->snd == SND_UPD2151;
	for (int line = 0; line < LINES; line++) {
		if (mc->vbl == VB_SCANLINE) {   // bangball_scanline
			if (line == 224) {
				set_irq(4);
				vblank_capture();
				screen_eof();
			} else if (line < 224 && !(b.irq_enable & 2))
				set_irq(1);
		} else if (line == mc->h) {
			vblank_capture();
			screen_eof();
			if (mc->vbl == VB_KARATOUR && b.ext_irq)
				set_irq(5);
		} else if (mc->vbl == VB_KARATOUR && line == mc->h + s_vblank_lines)
			clear_irq(5);
		if (mc->periodic && rbx_line_cycles(&b.per_frac, mc->periodic, mc->fps_mhz, LINES))
			set_irq(4);
		if (b.mouja_mhz && rbx_line_cycles(&b.mouja_frac, b.mouja_mhz, mc->fps_mhz * 1000, LINES))
			set_irq(0);
		if (b.blit_due && cpu_now() >= b.blit_due) {
			b.blit_due = 0;
			set_irq(mc->blit_lvl);
		}
		rbx_prof_set(ME_PA_M68K);
		// (its polling loops skipped within the slice: m68k_spin, the same cycles and state)
		s_cyc_base += m68k_spin_execute(&s_spin, rbx_line_cycles(&b.cpu_frac, mc->cpu_clock, mc->fps_mhz, LINES), 0);
		if (upd) {
			rbx_prof_set(ME_PA_SND);
			upd7810_execute(rbx_line_cycles(&b.snd_frac, 4000000, mc->fps_mhz, LINES));
		} else if (mc->snd == SND_2610) {
			rbx_prof_set(ME_PA_SND);
			b.z80_target += (uint64_t)rbx_line_cycles(&b.snd_frac, 8000000, mc->fps_mhz, LINES);
			z80_run(&s_z80, (unsigned long)b.z80_target, &b.z80_line, 0);
		}
		sound_line();
	}
	if (s_mix_r) {
		for (int i = 0; i < s_mix_n; i++)
			rbx_audio_put_stereo(s_mix[i], s_mix_r[i]);
		memset(s_mix_r, 0, sizeof(int32_t) * (size_t)s_mix_n);
	} else
		for (int i = 0; i < s_mix_n; i++)
			rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static void me_reset(void)
{
	m68k_spin_clear(&s_spin);
	rbx_video_wait_idle();
	memset(&b, 0, sizeof(b));
	b.irq_enable = 0xff;
	memset(s_k36_dirty, 1, sizeof(s_k36_dirty));
	if (s_k36)
		memset(s_k36, 0, 0x40000);
	map_mem();
	m68k_bus_irq_reset();
	rbx_m68k_vector_hook = s_mc->vdp == V4300 ? vector_hook : NULL;
	m68k_pulse_reset();
	s_cyc_base = 0;
	if (s_has_oki) {
		oki_reset(&s_oki);
		oki_bank_set();
	}
	if (s_mc->snd == SND_UPD2413 || s_mc->snd == SND_UPD2151)
		upd7810_reset();
	if (s_mc->snd == SND_2610) {
		z80_init(&s_z80);
		z80_setup();
	}
	if (s_mc->snd == SND_ES8712) {
		msm5205_reset(&s_msm);
		s_msm.prescaler = 40;   // (MSM6585, S40)
		es_reset();
	} else
		rbx_ym_reset(0);
	s_pal_valid = 0;
	s_mix_n = 0;
}

static const machine_t *find_machine(const char *m)
{
	for (int i = 0; i < NMACHINES; i++) {
		const size_t l = strlen(s_machines[i].name);
		if (!strncmp(m, s_machines[i].name, l) && (m[l] == ':' || !m[l]))
			return &s_machines[i];
	}
	return NULL;
}
static int me_supported(const char *machine) { return find_machine(machine) != NULL; }

static void me_exit(void)
{
	m68k_spin_free(&s_spin);
	rbx_video_wait_idle();
	rbx_m68k_vector_hook = NULL;
	if (s_has_eeprom)
		eepser_nv_save(&s_eeprom);
	if (s_mc && s_mc->snd != SND_ES8712)
		rbx_ym_exit(0);
	if (s_snap)
		free(s_snap->k36);
	free(s_snap);
	s_snap = NULL;
	free(s_mix);
	free(s_mix_r);
	s_mix = s_mix_r = NULL;
	free(s_pri);
	s_pri = NULL;
	free(s_k36);
	s_k36 = NULL;
	free(s_bp);
	s_bp = NULL;
}

static int me_init(const rbx_game *g)
{
	s_mc = find_machine(g->machine);
	if (!s_mc) {
		rbx_fail("%s: machine %s not emulated", g->set, g->machine);
		return -1;
	}
	const char *init = strchr(g->machine, ':');
	init = init ? init + 1 : "";
	s_blz = s_mc->map == MAP_BLZNTRND;
	s_gstrik2 = !strncmp(s_mc->name, "gstrik2", 7);
	s_vblank_lines = (int)(1500ull * LINES * s_mc->fps_mhz / 1000000000ull);
	if (!(s_bp = (struct metro_state *)calloc(1, sizeof(*s_bp))) || !(s_snap = calloc(1, sizeof(*s_snap))) ||
	    !(s_pri = (uint8_t *)calloc((size_t)s_mc->w * s_mc->h, 1))) {
		rbx_fail("%s: out of memory", g->set);
		me_exit();
		return -1;
	}
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_gfx_size = 0;
	s_gfx = rbx_region_opt(s_mc->vdp == V4100 ? "vdp" : s_mc->vdp == V4220 ? "vdp2" : "vdp3", &s_gfx_size);
	if (!s_rom || !s_gfx) {
		rbx_fail("%s: ROM regions missing", g->set);
		me_exit();
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	if (!strcmp(init, "balcube"))   // init_balcube: the GFX ROM data lines
		for (uint32_t i = 0; i < s_gfx_size; i += 2) {
			uint8_t v = s_gfx[i], r = 0;
			for (int k = 0; k < 8; k++)
				r |= (uint8_t)(((v >> k) & 1) << (7 - k));
			s_gfx[i] = r;
		}
	if (!strcmp(init, "dharmak"))   // init_dharmak
		for (uint32_t i = 0; i + 3 < s_gfx_size && i < 0x200000; i += 4) {
			uint8_t d = s_gfx[i + 1];
			s_gfx[i + 1] = (uint8_t)(((d >> 7) & 1) << 7 | ((d >> 3) & 1) << 6 | ((d >> 2) & 1) << 5 | ((d >> 4) & 1) << 4 |
			                         ((d >> 5) & 1) << 3 | ((d >> 6) & 1) << 2 | ((d >> 1) & 1) << 1 | (d & 1));
			d = s_gfx[i + 3];
			s_gfx[i + 3] = (uint8_t)(((d >> 7) & 1) << 7 | ((d >> 2) & 1) << 6 | ((d >> 5) & 1) << 5 | ((d >> 4) & 1) << 4 |
			                         ((d >> 3) & 1) << 3 | ((d >> 6) & 1) << 2 | ((d >> 1) & 1) << 1 | (d & 1));
		}
	for (int i = 0; i < 4; i++)
		s_gfx_elems[i] = s_gfx_size >= s_elem_bytes[i] ? (s_gfx_size - s_elem_bytes[i]) / 32 + 1 : 0;   // (whole tiles only)
	for (int z = 0; z < 64; z++) {   // (draw_spritegfx's sizes and steps)
		static const uint16_t zt[0x40] = {
			0xAAC,0x800,0x668,0x554,0x494,0x400,0x390,0x334, 0x2E8,0x2AC,0x278,0x248,0x224,0x200,0x1E0,0x1C8,
			0x1B0,0x198,0x188,0x174,0x164,0x154,0x148,0x13C, 0x130,0x124,0x11C,0x110,0x108,0x100,0x0F8,0x0F0,
			0x0EC,0x0E4,0x0DC,0x0D8,0x0D4,0x0CC,0x0C8,0x0C4, 0x0C0,0x0BC,0x0B8,0x0B4,0x0B0,0x0AC,0x0A8,0x0A4,
			0x0A0,0x09C,0x098,0x094,0x090,0x08C,0x088,0x080, 0x078,0x070,0x068,0x060,0x058,0x050,0x048,0x040 };
		const uint32_t scale = (uint32_t)zt[z] << 8;
		for (int k = 0; k < 8; k++) {
			const uint32_t size = (uint32_t)(k + 1) * 8;
			const uint32_t ss = (scale * size + 0x8000) >> 16;
			s_zsize[z][k] = (uint16_t)ss;
			s_zdx[z][k] = ss ? (size << 16) / ss : 0;
		}
	}
	static const char *const in_tags[4] = { "IN0", "IN1", "IN2", "IN3" };
	for (int i = 0; i < 4; i++)
		s_port_in[i] = rbx_port_find(in_tags[i]);
	s_port_dsw[0] = rbx_port_find("DSW0");
	s_port_dsw[1] = rbx_port_find("DSW1");
	static const char *const key_tags[5] = { "KEY0", "KEY1", "KEY2", "KEY3", "KEY4" };
	for (int i = 0; i < 5; i++)
		s_port_key[i] = rbx_port_find(key_tags[i]);
	s_port_coins = rbx_port_find("COINS");
	s_port_joys = rbx_port_find("JOYS");
	s_port_p1p2 = rbx_port_find("P1_P2");
	s_port_sys = rbx_port_find("SYSTEM");
	s_custom_port = -1;   // (COINS_SOUND: IN0's bit 7 is the sound CPU's busy line)
	for (const rbx_ipt_port *p = g->ports; p && p->tag; p++)
		if (!strcmp(p->tag, "IN0"))
			for (int f = 0; f < p->count; f++)
				if (g->fields[p->first + f].type == RBX_IPT_CUSTOM && g->fields[p->first + f].mask == 0x80)
					s_custom_port = s_port_in[0];
	// sound
	s_has_oki = s_mc->oki_clock != 0;
	s_oki_rom = rbx_region_opt("oki", &s_oki_size);
	memset(&s_oki, 0, sizeof(s_oki));
	if (s_has_oki) {
		oki_init(&s_oki, s_mc->oki_clock, s_mc->oki_pin7, s_oki_rom, s_oki_size, s_mc->oki_gain);
		if (s_mc->map == MAP_MOUJA)
			s_oki.read = oki_read_mouja;
	}
	s_has_eeprom = s_mc->map == MAP_GAKUSAI || s_mc->map == MAP_GAKUSAI2 || s_mc->map == MAP_DOKYUSP;
	eepser_init(&s_eeprom, EEPSER_93CXX, 64, 16, s_mc->cpu_clock);
	if (s_has_eeprom)
		eepser_nv_load(&s_eeprom, g->set);
	s_snd_rom = rbx_region_opt("audiocpu", &s_snd_size);
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	int stereo = 0;
	switch (s_mc->snd) {
	case SND_YMF:
		s_ymf_rom = rbx_region("ymf", &s_ymf_size);
		if (!s_ymf_rom) { me_exit(); return -1; }
		yc.type = RBX_YMF278B;
		yc.clock = 33868800;
		yc.adpcma = s_ymf_rom;
		yc.adpcma_size = s_ymf_size;
		yc.gain[0] = 256;
		yc.irq = ym_irq_68k;
		break;
	case SND_UPD2413: case SND_UPD2151: {
		if (!s_snd_rom) { rbx_fail("%s: audiocpu missing", g->set); me_exit(); return -1; }
		upd7810_config uc;
		memset(&uc, 0, sizeof(uc));
		uc.read = upd_read;
		uc.write = upd_write;
		uc.port_in = upd_port_in;
		uc.in_ports = 1 << UPD7810_PORTA;
		uc.port_out = upd_port_out;
		uc.rxd = upd_rxd;
		upd7810_init(&uc);
		if (s_mc->snd == SND_UPD2151) {
			yc.type = RBX_YM2151;
			yc.irq = ym_irq_upd;
			yc.gain[0] = yc.gain[1] = s_mc->ym_gain;
		} else {
			yc.type = RBX_YM2413;
			yc.gain[0] = yc.gain[1] = s_mc->ym_gain;
		}
		yc.clock = 3579545;
		break;
	}
	case SND_2413:
		yc.type = RBX_YM2413;
		yc.clock = 3579545;
		yc.gain[0] = yc.gain[1] = s_mc->ym_gain;
		break;
	case SND_2610: {
		if (!s_snd_rom) { rbx_fail("%s: audiocpu missing", g->set); me_exit(); return -1; }
		uint32_t asz = 0, bsz = 0;
		yc.type = RBX_YM2610;
		yc.clock = 8000000;
		yc.adpcma = rbx_region_opt("ymsnd:adpcma", &asz);
		yc.adpcma_size = asz;
		yc.adpcmb = rbx_region_opt("ymsnd:adpcmb", &bsz);
		yc.adpcmb_size = bsz;
		yc.irq = ym_irq_z80;
		if (s_gstrik2) {
			yc.gain[0] = 192; yc.gain[1] = 64; yc.gain[2] = 64;
		} else {
			yc.gain[0] = 192; yc.gain[1] = 256; yc.gain[2] = 256;
			stereo = 1;
		}
		s_roz_gfx = rbx_region("gfx2", &s_roz_size);
		if (!s_roz_gfx || !(s_k36 = (uint16_t *)calloc(1, 0x40000)) || !(s_snap->k36 = (uint16_t *)calloc(1, 0x40000))) {
			me_exit();
			return -1;
		}
		break;
	}
	case SND_ES8712:
		s_es_rom = rbx_region("essnd", &s_es_size);
		if (!s_es_rom) { me_exit(); return -1; }
		msm5205_init(&s_msm, 640000, MSM5205_S48_4B, es_vck);
		s_msm.prescaler = 40;
		s_out_rate = 32000;
		break;
	}
	if (s_mc->snd != SND_ES8712) {
		if (rbx_ym_init(0, &yc)) {
			rbx_fail("%s: out of memory", g->set);
			me_exit();
			return -1;
		}
		s_out_rate = rbx_ym_rate(0);
	}
	s_mix_cap = (int)(s_out_rate / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (stereo)
		s_mix_r = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || (stereo && !s_mix_r) || (stereo ? rbx_audio_setup_stereo(s_out_rate) : rbx_audio_setup(s_out_rate)) ||
	    rbx_video_setup(s_mc->w, s_mc->h, 0x1000)) {
		rbx_fail("%s: out of memory", g->set);
		me_exit();
		return -1;
	}
	rbx_prof_name(RBX_PA_DRV, s_pa_names, ME_PA_END - RBX_PA_DRV);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	me_reset();
	return 0;
}

static int me_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (load)
		rbx_video_wait_idle();
	else
		m68k_get_context(cpu);
	uint32_t irq = m68k_bus_irq_pending();
	int64_t cyc = s_cyc_base;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", &irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "asserted", &m68k_bus.irq_asserted, sizeof(m68k_bus.irq_asserted));
	rc |= rbx_state_io(ctx, "cyc", &cyc, sizeof(cyc));
	rc |= rbx_state_io(ctx, "metro", &b, (uint32_t)offsetof(struct metro_state, state_end));
	if (s_has_oki)
		rc |= rbx_state_io(ctx, "oki", &s_oki, (uint32_t)offsetof(okim6295_t, rom));
	if (s_has_eeprom)
		rc |= eepser_state(&s_eeprom, ctx, load, "eeprom");
	switch (s_mc->snd) {
	case SND_UPD2413: case SND_UPD2151:
		rc |= upd7810_state(ctx, load);
		rc |= rbx_ym_state(0, ctx, load);
		break;
	case SND_2610: {
		z80 z = s_z80;
		rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
		rc |= rbx_ym_state(0, ctx, load);
		rc |= rbx_state_io(ctx, "k36", s_k36, 0x40000);
		if (!rc && load) {
			s_z80 = z;
			z80_setup();
			memset(s_k36_dirty, 1, sizeof(s_k36_dirty));
		}
		break;
	}
	case SND_ES8712: {
		msm5205_t m = s_msm;
		rc |= rbx_state_io(ctx, "msm", &m, (uint32_t)offsetof(msm5205_t, vck));
		if (!rc && load) {
			m.vck = es_vck;
			s_msm = m;
		}
		break;
	}
	default:
		rc |= rbx_ym_state(0, ctx, load);
		break;
	}
	if (!rc && load) {
		m68k_set_context(cpu);
		s_cyc_base = cyc;
		m68k_bus_set_irq_pending(irq);
		m68k_bus_update_irq();
		s_pal_valid = 0;
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int me_save(void *ctx) { return me_state(ctx, 0); }
static int me_load(void *ctx) { return me_state(ctx, 1); }

const rbx_driver rbx_drv_metro = { "Metro", me_init, me_exit, me_reset, me_frame, me_save, me_load,
                                   me_supported, me_render, me_help };
