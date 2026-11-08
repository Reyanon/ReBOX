/*  Seta Hardware (MAME's name; the boards are numbered P0-xxx). Ported from MAME src/mame/seta/seta.cpp and x1_012.cpp
    (BSD-3-Clause, copyright-holders: Luca Elia), src/devices/machine/upd4701.cpp (BSD-3-Clause, copyright-holders:
    smf, AJR), upd4992.cpp (BSD-3-Clause, copyright-holders: Angelo Salese) and ticket.cpp (BSD-3-Clause,
    copyright-holders: Aaron Giles).
    The machines made of a 68000, the X1-001 sprites and little else: one table of memory maps, screens and interrupt
    schemes. The X1-010 plays the sound, or on the bootlegs and hacks a Z80 with a YM2151 and / or an OKI. The Jockey
    Club machines add one X1-012 tile layer.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/x1_010.h"
#include "../../video/x1_001.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LINES 256                    // (set_refresh_hz(60), set_size(64 * 8, 32 * 8), set_vblank_time(0))
#define VIS_Y0 8
#define VIS_Y1 247
#define MAX_PENS 0x800
#define FRAME_MHZ 60000u
#define HOPPER_FRAMES 9              // (TICKET_DISPENSER: 150 ms)
#define PRIZE_FRAMES 180             // (keroppi: prize_w's timer, 3 seconds)

enum {
	IRQ_VBL3_HOLD,     // the vblank: level 3, HOLD_LINE
	IRQ_VBL2_ASSERT,   // the vblank: level 2 until ipl1_ack
	IRQ_VBL3_ASSERT,   // the vblank: level 3 until blockcar_interrupt_w
	IRQ_1_2,           // seta_interrupt_1_and_2
	IRQ_JOCKEY         // jockeyc_state_base::interrupt
};
enum { SND_X1, SND_WIGGIE, SND_THUNDERLBL, SND_BLOCKCARB };
enum { GFX_SPRITES, GFX_WIGGIE, GFX_SUPERBAR };
enum { K_PLAIN, K_WITS, K_THUNDERL, K_THUNDERLBL, K_WIGGIE, K_BLOCKCAR, K_BLOCKCARB, K_PAIRLOVE, K_KRZYBOWL, K_KEROPPI,
       K_JOCKEYC, K_INTTOOTE };

typedef struct {
	const char *name;
	uint32_t cpu_clock, rom_end;
	struct { uint32_t start, end; } ram[3];
	uint32_t pal, pal_end;           // 0: the PROM's colours
	uint32_t spr_y, spr_code;        // (spriteylow + spritectrl, spritecode)
	uint32_t x1snd;                  // 0: none
	uint32_t in, dsw, misc;          // P1, P2, COINS; seta_dsw_r; the word of RAM ("? 0x4000"); 0: none
	int vis_x0, vis_x1;
	int fg_flipy, bg_flipy;          // (set_fg_yoffsets / set_bg_yoffsets, the flipped ones: the others are the same)
	int colorbase;                   // (the gfx entry's, in color codes)
	int irq, snd, gfx, kind;
	uint32_t snd_clock;              // the X1-010's
	int stereo;
} machine_t;

#define MAP_D00(s, e) { { 0xffc000, 0xffffff }, { s, e } }, 0x700000, 0x7003ff, 0xd00000, 0xe00000
#define MAP_E00 0xb00000, 0xb003ff, 0xe00000, 0xc00000

static const machine_t s_machines[] = {
	{ "umanclub", 16000000, 0x03ffff, { { 0x200000, 0x20ffff } }, 0x300000, 0x300fff, 0xa00000, 0xb00000, 0xc00000,
	  0x400000, 0x600000, 0xa80000, 0, 383, -0x12, 0x1, 0, IRQ_VBL3_HOLD, SND_X1, GFX_SPRITES, K_PLAIN, 16000000, 0 },
	{ "wits", 8000000, 0x00ffff, MAP_D00(0xe04000, 0xe07fff), 0x100000,
	  0xb00000, 0x600000, 0xc00000, 0, 383, -0x12, 0x1, 0, IRQ_VBL2_ASSERT, SND_X1, GFX_SPRITES, K_WITS, 16000000, 0 },
	{ "thunderl", 8000000, 0x00ffff, MAP_D00(0, 0), 0x100000,
	  0xb00000, 0x600000, 0xc00000, 0, 383, -0x12, 0x1, 0, IRQ_VBL2_ASSERT, SND_X1, GFX_SPRITES, K_THUNDERL, 16000000, 0 },
	{ "thunderlbl", 8000000, 0x00ffff, MAP_D00(0, 0), 0,
	  0xb00000, 0x600000, 0xc00000, 0, 383, -0x12, 0x1, 0, IRQ_VBL2_ASSERT, SND_THUNDERLBL, GFX_SPRITES, K_THUNDERLBL, 0, 0 },
	{ "wiggie", 8000000, 0x01ffff, MAP_D00(0, 0), 0,
	  0xb00000, 0x600000, 0xc00000, 0, 383, -0x12, 0x1, 0, IRQ_VBL2_ASSERT, SND_WIGGIE, GFX_WIGGIE, K_WIGGIE, 0, 0 },
	{ "superbar", 8000000, 0x01ffff, MAP_D00(0, 0), 0,
	  0xb00000, 0x600000, 0xc00000, 0, 383, -0x12, 0x1, 0, IRQ_VBL2_ASSERT, SND_WIGGIE, GFX_SUPERBAR, K_WIGGIE, 0, 0 },
	{ "jockeyc", 8000000, 0x1fffff, { { 0xffc000, 0xffffff } }, 0, 0, 0xd00000, 0xe00000, 0x900000,
	  0, 0, 0xc00000, 0, 383, -0x12 + 8, 0x1, 0, IRQ_JOCKEY, SND_X1, GFX_SPRITES, K_JOCKEYC, 16000000, 1 },
	{ "inttoote", 16000000, 0x1fffff, { { 0xffc000, 0xffffff } }, 0, 0, 0xd00000, 0xe00000, 0x900000,
	  0, 0, 0xc00000, 0, 383, -0x12 + 8, 0x1, 0, IRQ_JOCKEY, SND_X1, GFX_SPRITES, K_INTTOOTE, 16000000, 1 },
	{ "pairlove", 8000000, 0x03ffff, { { 0xf00000, 0xf0ffff } }, 0xb00000, 0xb00fff, 0xe00000, 0xc00000, 0xa00000,
	  0x500000, 0x300000, 0xd00000, 0, 383, -0x12, 0x1, 0x20, IRQ_1_2, SND_X1, GFX_SPRITES, K_PAIRLOVE, 16000000, 0 },
	{ "blockcar", 8000000, 0x03ffff, { { 0xf00000, 0xf03fff }, { 0xf04000, 0xf041ff }, { 0xf05000, 0xf050ff } }, MAP_E00, 0xa00000,
	  0x500000, 0x300000, 0xd00000, 0, 383, -0x12, 0x1, 0, IRQ_VBL3_ASSERT, SND_X1, GFX_SPRITES, K_BLOCKCAR, 16000000, 0 },
	{ "blockcarb", 8000000, 0x03ffff, { { 0xf00000, 0xf03fff }, { 0xf04000, 0xf041ff }, { 0xf05000, 0xf050ff } }, MAP_E00, 0,
	  0x500000, 0x300000, 0xd00000, 0, 383, -0x12, 0x1, 0, IRQ_VBL3_ASSERT, SND_BLOCKCARB, GFX_SPRITES, K_BLOCKCARB, 0, 0 },
	{ "atehate", 16000000, 0x0fffff, { { 0x900000, 0x9fffff } }, 0x700000, 0x7003ff, 0xa00000, 0xe00000, 0x100000,
	  0xb00000, 0x600000, 0xc00000, 0, 383, -0x12, 0x1, 0, IRQ_1_2, SND_X1, GFX_SPRITES, K_PLAIN, 16000000, 0 },
	{ "krzybowl", 14318181, 0x07ffff, { { 0xf00000, 0xf0ffff }, { 0x8000f0, 0x8000f1 }, { 0x800100, 0x8001ff } }, MAP_E00, 0xa00000,
	  0x500000, 0x300000, 0xd00000, 8, 311, -0x06, -0x3, 0, IRQ_1_2, SND_X1, GFX_SPRITES, K_KRZYBOWL, 14318181, 0 },
	{ "orbs", 14318181 / 2, 0x07ffff, { { 0xf00000, 0xf0ffff }, { 0x8000f0, 0x8000f1 }, { 0x800100, 0x8001ff } }, MAP_E00, 0xa00000,
	  0x500000, 0x300000, 0xd00000, 8, 311, -0x12, 0x1, 0, IRQ_1_2, SND_X1, GFX_SPRITES, K_PLAIN, 14318181, 1 },
	{ "keroppi", 14318180 / 2, 0x07ffff, { { 0xf00000, 0xf0ffff }, { 0x8000f0, 0x8000f1 }, { 0x800100, 0x8001ff } }, MAP_E00, 0xa00000,
	  0x500000, 0x300000, 0xd00000, 0, 319, -0x12, 0x1, 0, IRQ_1_2, SND_X1, GFX_SPRITES, K_KEROPPI, 14318180, 1 },
};

// (keroppi_protection_word)
static const uint16_t s_keroppi_prot[16] = {
	0x0000, 0x0000, 0x0000, 0x0000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x0400, 0x0400, 0x0400, 0x0000, 0x0000, 0x0000
};

typedef struct {   // what a state holds besides the memories (all of it set in st_reset)
	int32_t cpu_over;              // cycles the 68000 ran past the last line's target
	uint32_t soundlatch;
	int z_line;                    // the Z80's IRQ line
	uint32_t z_end;                // the Z80's absolute target
	uint32_t thunderl_prot;
	int32_t prize_hop, prize_timer, prot_count;
	int32_t x[2], y[2];            // the uPD4701s' counters
	uint32_t mux, out;
	int32_t hop_power[2], hop_status[2], hop_count[2];
	uint32_t rtc_frames;
	uint8_t rtc[7];
	uint8_t misc[2];
	uint16_t vctrl[3];
	uint32_t oki_acc;
	int32_t oki_hold;
} st_t;

static const machine_t *s_m;
static uint8_t *s_rom, *s_ram;
static uint32_t s_rom_size, s_ram_size;
static uint8_t s_pal[0x1000];
static uint16_t s_prom_pal[0x200];
static uint16_t s_snap_pal[MAX_PENS];
static int s_npens, s_width;
static x1_001_t *s_x1, *s_snap, *s_lay;      // s_lay: only the X1-012's decoded tiles
static x1_010_t s_snd;
static okim6295_t s_oki;
static z80 s_z;
static const uint8_t *s_zrom;
static uint32_t s_zrom_size;
static const uint8_t *s_zmap[256];
static uint8_t s_zram[0x800];
#define YM_FRAME_MAX 2048
static int32_t s_mix[YM_FRAME_MAX];
static int s_mix_n;
static int s_ym_on;
static uint16_t s_vram[0x2000], s_snap_vram[0x2000], s_snap_vctrl[3];   // (the Jockey Club X1-012)
static uint16_t s_it700[0x82];                  // (inttoote: 0x700000-0x700101)
static uint16_t s_protram[0x100], s_protram_old[0x100];   // (pairlove)
static int s_port_p1, s_port_p2, s_port_coins, s_port_dsw, s_port_p3, s_port_p4;
static int s_port_key1[5], s_port_key2[5], s_port_coin, s_port_service, s_port_dsw1, s_port_dsw2_3, s_port_cabinet;
static uint64_t s_cpu_frac, s_snd_frac, s_z_frac, s_ym_frac;
static st_t s_st;

#define COMBINE(d, v, m) ((d) = (uint16_t)(((d) & ~(m)) | ((v) & (m))))

// ---- the Jockey Club boards ----
static void hopper_motor(int i, int state)   // (ticket_dispenser_device::motor_w)
{
	if (state && !s_st.hop_power[i]) {
		s_st.hop_count[i] = HOPPER_FRAMES;
		s_st.hop_power[i] = 1;
		s_st.hop_status[i] = 0;
	} else if (!state && s_st.hop_power[i]) {
		s_st.hop_power[i] = 0;
		s_st.hop_count[i] = 0;
	}
}

static void update_hoppers(void)
{
	if (s_port_cabinet < 0)
		return;
	const int mux1 = (s_st.mux & 0x0201) != 0, lock1 = (s_st.out & 0x0004) != 0, lock2 = (s_st.out & 0x0008) != 0;
	if (rbx_port_read(s_port_cabinet) & 1) {   // (double hoppers)
		hopper_motor(0, mux1 && !lock1);
		hopper_motor(1, (s_st.mux & 0x0102) && !lock2);
	} else
		hopper_motor(0, mux1 && (!lock1 || !lock2));
}

static void hopper_frame(void)
{
	for (int i = 0; i < 2; i++)
		if (s_st.hop_power[i] && --s_st.hop_count[i] <= 0) {
			s_st.hop_status[i] ^= 1;
			s_st.hop_count[i] = HOPPER_FRAMES;
		}
	rbx_port_custom(s_port_coin, 0x0404, (s_st.hop_status[0] ? 0x0004u : 0) | (s_st.hop_status[1] ? 0x0400u : 0));
}

static uint16_t mux_r(void)
{
	uint32_t result = 0xffff;
	for (int i = 0; i < 5; i++)
		if ((s_st.mux >> (i + 3)) & 1)
			result &= (rbx_port_read(s_port_key2[i]) << 8) | rbx_port_read(s_port_key1[i]);
	return (uint16_t)result;
}

static uint16_t dsw_r(int offset)
{
	const int shift = offset * 4;
	const uint32_t d1 = rbx_port_read(s_port_dsw1), d23 = rbx_port_read(s_port_dsw2_3);
	return (uint16_t)(((d1 >> shift) & 0xf) | (((d23 >> shift) & 0xf) << 4) | (((d23 >> (shift + 8)) & 0xf) << 8));
}

// a D4911C, MAME's upd4992 fed by the host's clock. The driver has no clock of its own to read: a fixed date, running
static uint16_t rtc_r(int offset)
{
	if (offset >= 7)
		offset++;
	if (offset / 2 >= 7)
		return 0;
	return (s_st.rtc[offset / 2] >> ((offset & 1) * 4)) & 0xf;
}

static void bcd_inc(uint8_t *v, int wrap, uint8_t *carry_to, int carry_wrap)
{
	*v = (uint8_t)((*v & 0xf) == 9 ? (*v & 0xf0) + 0x10 : *v + 1);
	if (*v == wrap) {
		*v = 0;
		if (carry_to)
			bcd_inc(carry_to, carry_wrap, NULL, 0);
	}
}

static void rtc_frame(void)
{
	if (++s_st.rtc_frames < 60)
		return;
	s_st.rtc_frames = 0;
	bcd_inc(&s_st.rtc[0], 0x60, &s_st.rtc[1], 0x60);
	if (!s_st.rtc[0] && !s_st.rtc[1])
		bcd_inc(&s_st.rtc[2], 0x24, NULL, 0);
}

// ---- krzybowl: the uPD4701s ----
static uint8_t upd4701_read(int chip, int offset)   // (read_xy)
{
	const uint32_t latch = (uint32_t)((offset & 2) ? s_st.y[chip] : s_st.x[chip]) & 0xfff;
	return (uint8_t)((offset & 1) ? latch >> 8 : latch);
}

// ---- the bus ----
static uint16_t main_read16(uint32_t a)
{
	const machine_t *m = s_m;
	if (a >= m->spr_y && a <= m->spr_y + 0x5ff) return s_x1->ylow[(a - m->spr_y) >> 1];
	if (a >= m->spr_y + 0x600 && a <= m->spr_y + 0x607) return s_x1->ctrl[(a - m->spr_y - 0x600) >> 1];
	if (m->x1snd && a >= m->x1snd && a <= m->x1snd + 0x3fff) return x1_010_word_r(&s_snd, (a - m->x1snd) >> 1);
	if (m->in) {
		if (a == m->in) return (uint16_t)rbx_port_read(s_port_p1);
		if (a == m->in + 2) return (uint16_t)rbx_port_read(s_port_p2);
		if (a == m->in + 4) {
			uint16_t r = (uint16_t)rbx_port_read(s_port_coins);
			if (m->kind == K_KEROPPI && s_st.prize_hop == 2) {   // (coin_r: the prize hopper)
				r &= (uint16_t)~0x0002;
				s_st.prize_hop = 0;
			}
			return r;
		}
	}
	if (m->dsw) {   // (seta_dsw_r)
		if (a == m->dsw) return (uint16_t)((rbx_port_read(s_port_dsw) >> 8) & 0xff);
		if (a == m->dsw + 2) return (uint16_t)(rbx_port_read(s_port_dsw) & 0xff);
	}
	switch (m->kind) {
	case K_WITS:
		if (a == 0xb00008) return (uint16_t)rbx_port_read(s_port_p3);
		if (a == 0xb0000a) return (uint16_t)rbx_port_read(s_port_p4);
		break;
	case K_THUNDERL:
		if (a == 0xb0000c) return (uint16_t)s_st.thunderl_prot;
		break;
	case K_PAIRLOVE:
		if (a >= 0x900000 && a <= 0x9001ff) {   // (prot_r)
			const uint32_t o = (a - 0x900000) >> 1;
			const uint16_t r = s_protram[o];
			s_protram[o] = s_protram_old[o];
			return r;
		}
		break;
	case K_KRZYBOWL:
		if (a >= 0x600000 && a <= 0x60000f) return upd4701_read((int)((a - 0x600000) >> 3) & 1, (int)((a >> 1) & 3));
		break;
	case K_KEROPPI:
		if (a == 0x100000) {   // (protection_r)
			const uint16_t r = s_keroppi_prot[s_st.prot_count];
			if (++s_st.prot_count > 15)
				s_st.prot_count = 15;
			return r;
		}
		if (a == 0x200000) {   // (protection_init_r)
			s_st.prot_count = 0;
			return 0;
		}
		break;
	case K_JOCKEYC:
	case K_INTTOOTE:
		if (a == 0x200000) return mux_r();
		if (a == 0x200002) return (uint16_t)rbx_port_read(s_port_coin);
		if (a == 0x200010) return (uint16_t)rbx_port_read(s_port_service);
		if (a >= 0x500000 && a <= 0x500003) return dsw_r((int)((a - 0x500000) >> 1));
		if (a >= 0x800000 && a <= 0x80001f) return rtc_r((int)((a - 0x800000) >> 1));
		if (m->kind == K_JOCKEYC) {
			if (a == 0x600000 || a == 0x600002) return 0xffff;   // (comm_r)
		} else if (a >= 0x700000 && a <= 0x700101)
			return s_it700[(a - 0x700000) >> 1] & 0x3f;
		break;
	default:
		break;
	}
	if (m->irq == IRQ_VBL2_ASSERT && a == 0x200000)   // (ipl1_ack_r)
		m68k_bus_set_line(2, 0);
	return 0;
}

static uint8_t main_read8(uint32_t a)
{
	return (uint8_t)(main_read16(a & ~1u) >> ((a & 1) ? 0 : 8));
}

static void soundlatch_w(uint8_t v)
{
	s_st.soundlatch = v;
	if (s_m->snd != SND_BLOCKCARB)   // (data_pending_callback)
		s_st.z_line = 1;
}

static void thunderl_protection_w(uint32_t addr)
{
#define B(n) ((addr >> (n)) & 1)
#define N(n) (B(n) ^ 1)
	const uint32_t b56 = B(6) & B(13), or2 = B(2) | N(6) | N(8), b6 = b56 | N(16);
	s_st.thunderl_prot = B(2) | (B(2) & N(3)) << 1 | (B(2) | N(6)) << 2 | or2 << 3 | (B(3) & N(11) & B(15)) << 4
		| b56 << 5 | b6 << 6 | (b6 & or2) << 7;
#undef B
#undef N
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	const machine_t *m = s_m;
	if (a >= m->spr_y && a <= m->spr_y + 0x5ff) {
		if (mask & 0xff)
			s_x1->ylow[(a - m->spr_y) >> 1] = (uint8_t)v;
		return;
	}
	if (a >= m->spr_y + 0x600 && a <= m->spr_y + 0x607) {
		if (mask & 0xff)
			s_x1->ctrl[(a - m->spr_y - 0x600) >> 1] = (uint8_t)v;
		return;
	}
	if (m->x1snd && a >= m->x1snd && a <= m->x1snd + 0x3fff) {
		x1_010_word_w(&s_snd, (a - m->x1snd) >> 1, v);
		return;
	}
	if (m->irq == IRQ_VBL2_ASSERT && a == 0x200000) {   // (ipl1_ack_w)
		m68k_bus_set_line(2, 0);
		return;
	}
	if (m->irq == IRQ_VBL3_ASSERT && a == 0x200000) {   // (blockcar_interrupt_w)
		if ((mask & 0xff) && !(v & 1))
			m68k_bus_set_line(3, 0);
		return;
	}
	switch (m->kind) {
	case K_THUNDERL:
		if (a >= 0x400000 && a <= 0x41ffff)
			thunderl_protection_w(a - 0x400000);
		break;
	case K_THUNDERLBL:
		if (a == 0xb0000c && (mask & 0xff00))   // (the bootleg's first byte of spritectrl)
			s_x1->ctrl[0] = (uint8_t)(v >> 8);
		else if (a == 0xb00008 && (mask & 0xff00))
			soundlatch_w((uint8_t)(v >> 8));
		break;
	case K_WIGGIE:
		if (a == 0xb00008 && (mask & 0xff00))
			soundlatch_w((uint8_t)(v >> 8));
		break;
	case K_BLOCKCARB:
		if (a == 0x500008 && (mask & 0xff))
			oki_write(&s_oki, (uint8_t)v);
		else if (a == 0x50000c && (mask & 0xff))
			soundlatch_w((uint8_t)v);
		break;
	case K_PAIRLOVE:
		if (a >= 0x900000 && a <= 0x9001ff) {   // (prot_w)
			const uint32_t o = (a - 0x900000) >> 1;
			s_protram_old[o] = s_protram[o];
			s_protram[o] = v;
		}
		break;
	case K_KEROPPI:
		if (a == 0x900002 && (v & 0x0010) && !s_st.prize_hop) {   // (prize_w)
			s_st.prize_hop = 1;
			s_st.prize_timer = PRIZE_FRAMES;
		}
		break;
	case K_JOCKEYC:
	case K_INTTOOTE:
		if (a == 0x200000) {   // (jockeyc_mux_w, inttoote_mux_w: lamps and the hoppers)
			uint16_t d = (uint16_t)s_st.mux;
			COMBINE(d, v, mask);
			s_st.mux = d;
			update_hoppers();
		} else if (a == 0x200010) {   // (jockeyc_out_w, inttoote_out_w)
			uint16_t d = (uint16_t)s_st.out;
			COMBINE(d, v, mask);
			s_st.out = d;
			update_hoppers();
		} else if (a >= 0xa00000 && a <= 0xa00005) {   // (x1_012_device::vctrl_w)
			uint16_t *d = &s_st.vctrl[(a - 0xa00000) >> 1];
			COMBINE(*d, v, mask);
		} else if (m->kind == K_INTTOOTE && a >= 0x700000 && a <= 0x700101) {
			uint16_t *d = &s_it700[(a - 0x700000) >> 1];
			COMBINE(*d, v, mask);
		}
		break;
	default:
		break;
	}
}

static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	if (a & 1)
		main_write16m(a & ~1u, v, 0x00ff);
	else
		main_write16m(a, (uint16_t)(v << 8), 0xff00);
}

// ---- the sound Z80 (wiggie_sound_map, thunderlbl_sound_map, blockcarb_sound_map) ----
static uint8_t latch_r(void)
{
	if (s_m->snd != SND_BLOCKCARB)
		s_st.z_line = 0;
	return (uint8_t)s_st.soundlatch;
}

static uint8_t z_read(void *ud, uint16_t a)
{
	(void)ud;
	switch (s_m->snd) {
	case SND_WIGGIE:
		if (a < 0x8000) return a < s_zrom_size ? s_zrom[a] : 0;
		if (a < 0x8800) return s_zram[a & 0x7ff];
		if (a == 0x9800) return oki_read(&s_oki);
		if (a == 0xa000) return latch_r();
		return 0;
	case SND_THUNDERLBL:
		if (a < 0xe000) return a < s_zrom_size ? s_zrom[a] : 0;
		if (a >= 0xf800) return s_zram[a & 0x7ff];
		return 0xff;
	default:
		if (a < 0x8000) return a < s_zrom_size ? s_zrom[a] : 0;
		if (a >= 0xd000 && a < 0xd800) return s_zram[a & 0x7ff];
		if (a == 0xf000 || a == 0xf001) return rbx_ym_read(0, a & 1);
		if (a == 0xf002) return 0xff;
		if (a == 0xf008) return latch_r();
		return 0xff;
	}
}

static void z_write(void *ud, uint16_t a, uint8_t v)
{
	(void)ud;
	switch (s_m->snd) {
	case SND_WIGGIE:
		if (a >= 0x8000 && a < 0x8800) s_zram[a & 0x7ff] = v;
		else if (a == 0x9800) oki_write(&s_oki, v);
		break;
	case SND_THUNDERLBL:
		if (a >= 0xf800) s_zram[a & 0x7ff] = v;
		break;
	default:
		if (a >= 0xd000 && a < 0xd800) s_zram[a & 0x7ff] = v;
		else if (a == 0xf000 || a == 0xf001) rbx_ym_write(0, a & 1, v);
		break;
	}
}

static uint8_t z_in(z80 *z, uint8_t p)
{
	(void)z;
	if (s_m->snd == SND_THUNDERLBL) {
		if (p < 0x40) return rbx_ym_read(0, p & 1);
		if (p >= 0xc0) return latch_r();
		return 0xff;
	}
	return 0;
}

static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (s_m->snd == SND_THUNDERLBL && p < 0x40)
		rbx_ym_write(0, p & 1, v);
}

static void ym_irq(int state)
{
	if (s_m->snd == SND_BLOCKCARB)
		s_st.z_line = state;
}

// ---- video ----
// (seta_layers_update with the one X1-012: opaque, under the sprites; update_scroll, tilemap_t::draw)
static void draw_layer(uint16_t *bm, int pitch, int w, int h)
{
	const uint16_t *v = &s_snap_vram[(s_snap_vctrl[2] & 8) ? 0x1000 : 0];
	const int flip = (s_snap->ctrl[0] & 0x40) >> 6;
	const int xo = flip ? (s_m->kind == K_JOCKEYC ? 126 : 0) : -2;
	const int vis_h = VIS_Y1 - VIS_Y0 + 1;
	int sx = s_snap_vctrl[0] + 0x10 - xo;
	int sy = s_snap_vctrl[1] - (256 - vis_h) / 2;
	if (flip)
		sy -= vis_h;
	const uint8_t *tiles = s_lay->tiles;
	const uint32_t ntiles = s_lay->ntiles;
	for (int y = 0; y < h; y++) {
		// (a position of the unflipped map: flipped, the map is read backwards from the other corner)
		const int vv = flip ? (511 - (y + VIS_Y0 - sy + 256)) & 511 : (y + VIS_Y0 + sy) & 511;
		const int row = vv >> 4;
		const int step = flip ? -1 : 1;
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < w;) {
			const int src_x = x + s_m->vis_x0 + sx;
			const int uu = (flip ? -src_x - 1 : src_x) & 1023;
			const uint16_t code = v[row * 64 + (uu >> 4)];
			const uint16_t attr = v[row * 64 + (uu >> 4) + 0x800];
			const uint8_t *src = tiles + (uint32_t)((code & 0x3fff) % ntiles) * 256
				+ ((code & 0x4000) ? 15 - (vv & 15) : (vv & 15)) * 16;
			const int base = (attr & 0x1f) * 16;
			int cu = uu & 15;
			int n = flip ? cu + 1 : 16 - cu;
			if (n > w - x)
				n = w - x;
			for (int i = 0; i < n; i++, cu += step)
				d[x + i] = (uint16_t)(base + src[(code & 0x8000) ? 15 - cu : cu]);
			x += n;
		}
	}
}

static void st_render(void)   // (screen_update_seta_no_layers, screen_update_seta_layers)
{
	for (int i = 0; i < s_npens; i++) {   // (set_pens)
		int r = (s_snap_pal[i] >> 10) & 31, g = (s_snap_pal[i] >> 5) & 31, b = s_snap_pal[i] & 31;
		r = (r << 3) | (r >> 2);
		g = (g << 3) | (g >> 2);
		b = (b << 3) | (b >> 2);
		rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, h = rbx_video.height;
	if (s_lay)
		draw_layer(bm, pitch, s_width, h);
	else
		for (int y = 0; y < h; y++)
			for (int x = 0; x < s_width; x++)
				bm[y * pitch + x] = 0x1f0;
	x1_draw(s_snap, bm, pitch, s_width, h, s_m->vis_x0, VIS_Y0, LINES, VIS_Y1, 0x1000);
}

static void capture(void)
{
	rbx_video_wait_idle();
	if (s_m->pal)
		memcpy(s_snap_pal, s_pal, (size_t)s_npens * 2);
	else
		memcpy(s_snap_pal, s_prom_pal, sizeof(s_prom_pal));
	memcpy(s_snap->ctrl, s_x1->ctrl, sizeof(s_x1->ctrl));
	memcpy(s_snap->ylow, s_x1->ylow, sizeof(s_x1->ylow));
	memcpy(s_snap->code, s_x1->code, sizeof(s_x1->code));
	s_snap->bgflag = s_x1->bgflag;
	if (s_lay) {
		memcpy(s_snap_vram, s_vram, sizeof(s_vram));
		memcpy(s_snap_vctrl, s_st.vctrl, sizeof(s_snap_vctrl));
	}
}

// ---- init / frame ----
static void st_exit(void);

static void z_reset(void)
{
	z80_init(&s_z);
	s_z.read_byte = z_read;
	s_z.write_byte = z_write;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.read_map = s_zmap;
	s_z.int_data = 0xff;
}

static void st_reset(void)
{
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_ram, 0, s_ram_size);
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_it700, 0, sizeof(s_it700));
	memset(s_protram, 0, sizeof(s_protram));
	memset(s_protram_old, 0, sizeof(s_protram_old));
	memset(s_zram, 0, sizeof(s_zram));
	memset(&s_st, 0, sizeof(s_st));
	// (a fixed date, see rtc_r: BCD seconds, minutes, hours, day of the week, day, month, year)
	static const uint8_t date[7] = { 0x00, 0x00, 0x12, 0x06, 0x07, 0x11, 0x26 };
	memcpy(s_st.rtc, date, sizeof(date));
	x1_reset(s_x1);
	if (s_m->snd == SND_X1)
		x1_010_reset(&s_snd);
	else {
		z_reset();
		if (s_ym_on)
			rbx_ym_reset(0);
		if (s_m->snd != SND_THUNDERLBL)
			oki_reset(&s_oki);
	}
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_cpu_frac = s_snd_frac = s_z_frac = s_ym_frac = 0;
	s_mix_n = 0;
}

// the machine's gfx_layout
static void sprite_layout(x1_layout *l, int gfx, uint32_t gsize)
{
	memset(l, 0, sizeof(*l));
	if (gfx == GFX_SPRITES) {   // (layout_sprites: planes { half + 8, half, 8, 0 }, a row 16 bits, the right half and the lower half further on)
		const uint32_t half = gsize * 4;
		l->total = gsize / 2 / 64;
		l->planes[0] = half + 8;
		l->planes[1] = half;
		l->planes[2] = 8;
		l->planes[3] = 0;
		for (int i = 0; i < 8; i++) {
			l->xoffs[i] = (uint32_t)i;
			l->xoffs[8 + i] = 128u + (uint32_t)i;
			l->yoffs[i] = 16u * (uint32_t)i;
			l->yoffs[8 + i] = 256u + 16u * (uint32_t)i;
		}
		l->charincrement = 512;
		return;
	}
	// (wiggie_layout, superbar_layout: the address lines are swapped in the y offsets)
	static const uint8_t wy[16] = { 0, 16, 4, 20, 2, 18, 6, 22, 1, 17, 5, 21, 3, 19, 7, 23 };
	static const uint8_t sy[16] = { 0, 2, 16, 18, 1, 3, 17, 19, 4, 6, 20, 22, 5, 7, 21, 23 };
	const uint32_t quarter = gsize * 2;
	l->total = gsize / 4 / 32;
	for (int p = 0; p < 4; p++)
		l->planes[p] = (uint32_t)p * quarter;
	for (int i = 0; i < 8; i++) {
		l->xoffs[i] = (uint32_t)i;
		l->xoffs[8 + i] = 64u + (uint32_t)i;
	}
	for (int i = 0; i < 16; i++)
		l->yoffs[i] = (gfx == GFX_WIGGIE ? wy[i] : sy[i]) * 8u;
	l->charincrement = 256;
}

// (layout_tilemap of the X1-012: gfx_drgnunit)
static int layer_init(const uint8_t *gfx, uint32_t size)
{
	x1_layout l;
	memset(&l, 0, sizeof(l));
	l.total = size * 8 / 1024;
	for (int p = 0; p < 4; p++)
		l.planes[p] = 4u * (uint32_t)p;
	for (int i = 0; i < 4; i++) {
		l.xoffs[i] = 384u + (uint32_t)i;
		l.xoffs[4 + i] = 256u + (uint32_t)i;
		l.xoffs[8 + i] = 128u + (uint32_t)i;
		l.xoffs[12 + i] = (uint32_t)i;
	}
	for (int i = 0; i < 8; i++) {
		l.yoffs[i] = 16u * (uint32_t)i;
		l.yoffs[8 + i] = 512u + 16u * (uint32_t)i;
	}
	l.charincrement = 1024;
	s_lay = (x1_001_t *)calloc(1, sizeof(x1_001_t));
	return !s_lay || x1_init_layout(s_lay, gfx, size, 32, &l);
}

static void wiggie_descramble(uint8_t *src, uint32_t len)   // (init_wiggie)
{
	static const uint8_t conv[16] = { 0x0, 0x1, 0x8, 0x9, 0x2, 0x3, 0xa, 0xb, 0x4, 0x5, 0xc, 0xd, 0x6, 0x7, 0xe, 0xf };
	for (uint32_t i = 0; i + 16 <= len; i += 16) {
		uint8_t t[16];
		memcpy(t, src + i, 16);
		for (int j = 0; j < 16; j++)
			src[i + j] = t[conv[j]];
	}
}

static int sound_init(const rbx_game *g)
{
	const machine_t *m = s_m;
	uint32_t size = 0;
	if (m->snd == SND_X1) {
		const uint8_t *snd = rbx_region("x1snd", &size);
		if (!snd) {
			rbx_fail("%s: ROM regions missing", g->set);
			return -1;
		}
		x1_010_init(&s_snd, snd, size);
		const uint32_t rate = m->snd_clock / 512;
		return m->stereo ? rbx_audio_setup_stereo(rate) : rbx_audio_setup(rate);
	}
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	if (!s_zrom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(s_zmap, 0, sizeof(s_zmap));
	const uint32_t rom_end = m->snd == SND_THUNDERLBL ? 0xe000 : 0x8000;
	for (uint32_t i = 0; i < rom_end >> 8 && (i + 1) << 8 <= s_zrom_size; i++)
		s_zmap[i] = s_zrom + (i << 8);
	if (m->snd != SND_THUNDERLBL) {
		const uint8_t *oki = rbx_region("oki", &size);
		if (!oki) {
			rbx_fail("%s: ROM regions missing", g->set);
			return -1;
		}
		// (wiggie: 16 MHz / 16 at 1.0; blockcarb: 1 MHz at 0.5; pin 7 high on both)
		oki_init(&s_oki, 1000000, 1, oki, size, m->snd == SND_WIGGIE ? 256 : 128);
	}
	if (m->snd == SND_WIGGIE)
		return rbx_audio_setup(s_oki.rate);
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = 4000000;   // (thunderlbl: 16 MHz / 4 at 1.0; blockcarb: 4 MHz at 0.5)
	yc.gain[0] = yc.gain[1] = m->snd == SND_THUNDERLBL ? 256 : 128;
	yc.irq = ym_irq;
	if (rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1))
		return -1;
	s_ym_on = 1;
	return rbx_audio_setup(rbx_ym_rate(0));
}

static int st_init(const rbx_game *g)
{
	s_m = NULL;
	for (size_t i = 0; i < sizeof(s_machines) / sizeof(s_machines[0]); i++)
		if (!strcmp(s_machines[i].name, g->machine))
			s_m = &s_machines[i];
	if (!s_m) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	const machine_t *m = s_m;
	uint32_t gsize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	const uint8_t *gfx = rbx_region("gfx1", &gsize);
	if (!s_rom || !gfx) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (m->kind == K_WIGGIE)
		wiggie_descramble(s_rom, s_rom_size);
	m68k_bus_swap16(s_rom, s_rom_size);
	if (m->kind == K_INTTOOTE) {   // (init_inttoote: the code patches MAME has for the unemulated protection)
		uint16_t *r = (uint16_t *)s_rom;
		r[0x4de0 / 2] = 0x4e71;
		r[0x4de2 / 2] = 0x4e71;
		r[0x368a / 2] = 0x50f9;
	}
	s_ram_size = 0;
	for (int i = 0; i < 3 && m->ram[i].end; i++)
		s_ram_size += m->ram[i].end - m->ram[i].start + 1;
	s_ram = (uint8_t *)calloc(s_ram_size, 1);
	s_x1 = (x1_001_t *)calloc(1, sizeof(x1_001_t));
	s_snap = (x1_001_t *)calloc(1, sizeof(x1_001_t));
	x1_layout l;
	sprite_layout(&l, m->gfx, gsize);
	if (!s_ram || !s_x1 || !s_snap || x1_init_layout(s_x1, gfx, gsize, 32, &l)) {
		rbx_fail("%s: out of memory", g->set);
		st_exit();
		return -1;
	}
	s_x1->fg_flipyoffs = m->fg_flipy;
	s_x1->fg_noflipyoffs = 0x0e;
	s_x1->bg_flipyoffs = m->bg_flipy;
	s_x1->bg_noflipyoffs = -0x1;
	s_x1->gfx_colorbase = m->colorbase;
	s_x1->setac_gfxbank = 1;
	*s_snap = *s_x1;   // (configuration and the decoded tiles; RAM is copied at each vblank)

	s_npens = m->kind == K_PAIRLOVE ? 0x800 : 0x200;   // (umanclub's RAM goes on past the 0x200 pens)
	s_width = m->vis_x1 - m->vis_x0 + 1;

	s_port_p1 = rbx_port_find("P1");
	s_port_p2 = rbx_port_find("P2");
	s_port_coins = rbx_port_find("COINS");
	s_port_dsw = rbx_port_find("DSW");
	s_port_p3 = rbx_port_find("P3");
	s_port_p4 = rbx_port_find("P4");
	if (m->kind == K_JOCKEYC || m->kind == K_INTTOOTE) {
		uint32_t psize, lsize;
		const uint8_t *prom = rbx_region("proms", &psize), *layer = rbx_region("gfx2", &lsize);
		if (!prom || !layer || psize < 0x400) {
			rbx_fail("%s: ROM regions missing", g->set);
			st_exit();
			return -1;
		}
		for (int i = 0; i < 0x200; i++)   // (palette_init_RRRRRGGGGGBBBBB_proms)
			s_prom_pal[i] = (uint16_t)(prom[i * 2] << 8 | prom[i * 2 + 1]);
		if (layer_init(layer, lsize)) {
			rbx_fail("%s: out of memory", g->set);
			st_exit();
			return -1;
		}
		for (int i = 0; i < 5; i++) {
			char tag[8];
			snprintf(tag, sizeof(tag), "KEY1.%d", i);
			s_port_key1[i] = rbx_port_find(tag);
			snprintf(tag, sizeof(tag), "KEY2.%d", i);
			s_port_key2[i] = rbx_port_find(tag);
		}
		s_port_coin = rbx_port_find("COIN");
		s_port_service = rbx_port_find("SERVICE");
		s_port_dsw1 = rbx_port_find("DSW1");
		s_port_dsw2_3 = rbx_port_find("DSW2_3");
		s_port_cabinet = rbx_port_find("CABINET");
	}
	if (sound_init(g) || rbx_video_setup(s_width, VIS_Y1 - VIS_Y0 + 1, s_npens)) {
		rbx_fail("%s: out of memory", g->set);
		st_exit();
		return -1;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, m->rom_end, s_rom, s_rom_size, 1, 0);
	uint32_t off = 0;
	for (int i = 0; i < 3 && m->ram[i].end; i++) {
		const uint32_t n = m->ram[i].end - m->ram[i].start + 1;
		m68k_bus_map(m->ram[i].start, m->ram[i].end, s_ram + off, n, 1, 1);
		off += n;
	}
	if (m->pal)
		m68k_bus_map(m->pal, m->pal_end, s_pal, m->pal_end - m->pal + 1, 1, 1);
	if (m->misc)
		m68k_bus_map(m->misc, m->misc + 1, s_st.misc, sizeof(s_st.misc), 1, 1);
	m68k_bus_map(m->spr_code, m->spr_code + 0x3fff, s_x1->code, sizeof(s_x1->code), 1, 1);
	if (s_lay)
		m68k_bus_map(0xb00000, 0xb03fff, s_vram, sizeof(s_vram), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	st_reset();
	return 0;
}

static void st_exit(void)
{
	if (s_ym_on)
		rbx_ym_exit(0);
	s_ym_on = 0;
	if (s_x1)
		x1_exit(s_x1);
	if (s_lay)
		x1_exit(s_lay);
	free(s_x1);
	free(s_snap);
	free(s_lay);
	free(s_ram);
	s_x1 = s_snap = s_lay = NULL;
	s_ram = NULL;
}

static void run_cpus(void)
{
	// (the line's end is an absolute target: what the 68000 ran past it comes off the next line)
	const int want = rbx_line_cycles(&s_cpu_frac, s_m->cpu_clock, FRAME_MHZ, LINES) - s_st.cpu_over;
	if (want > 0)
		s_st.cpu_over = m68k_execute(want) - want;
	else
		s_st.cpu_over = -want;
	if (s_m->snd == SND_X1)
		return;
	s_st.z_end += (uint32_t)rbx_line_cycles(&s_z_frac, 4000000, FRAME_MHZ, LINES);
	z80_run(&s_z, s_st.z_end, &s_st.z_line, 0);
}

static void run_sound(void)
{
	const int snd = s_m->snd;
	if (snd == SND_X1) {
		const int n = rbx_line_cycles(&s_snd_frac, s_m->snd_clock / 512, FRAME_MHZ, LINES);
		if (n <= 0)
			return;
		int32_t l[8] = { 0 }, r[8] = { 0 };
		const int m = n > 8 ? 8 : n;
		x1_010_render(&s_snd, l, r, m);
		for (int i = 0; i < m; i++) {
			if (s_m->stereo)
				rbx_audio_put_stereo(l[i], r[i]);
			else
				rbx_audio_put((l[i] + r[i]) / 2);   // (both sides at 0.5 to the mono speaker)
		}
		return;
	}
	if (snd == SND_WIGGIE) {
		const int n = rbx_line_cycles(&s_snd_frac, s_oki.rate, FRAME_MHZ, LINES);
		for (int i = 0; i < n; i++)
			rbx_audio_put(oki_sample(&s_oki));
		return;
	}
	// (the YM2151 on the worker: its samples taken at the frame's end, the OKI's kept here until then)
	int32_t dummy[64];
	const int n = rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&s_ym_frac, 4000000, FRAME_MHZ, LINES), dummy, 64);
	const uint32_t yr = rbx_ym_rate(0);
	for (int i = 0; i < n && s_mix_n < YM_FRAME_MAX; i++) {
		if (snd == SND_BLOCKCARB) {
			s_st.oki_acc += s_oki.rate;
			while (s_st.oki_acc >= yr) {
				s_st.oki_acc -= yr;
				s_st.oki_hold = oki_sample(&s_oki);
			}
		}
		s_mix[s_mix_n++] = s_st.oki_hold;
	}
}

static void sound_frame_end(void)
{
	if (s_m->snd != SND_THUNDERLBL && s_m->snd != SND_BLOCKCARB)
		return;
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	s_mix_n = 0;
}

static void st_frame(void)
{
	const int irq = s_m->irq;
	if (s_m->kind == K_KRZYBOWL)
		for (int p = 0; p < 2; p++) {   // (the trackballs: PORT_REVERSE, 30 x 70%)
			s_st.x[p] -= rbx_analog_dpad(p + 1, RBX_IPT_TRACKBALL_X, 21);
			s_st.y[p] -= rbx_analog_dpad(p + 1, RBX_IPT_TRACKBALL_Y, 21);
		}
	else if (s_m->kind == K_KEROPPI) {
		if (s_st.prize_timer > 0 && --s_st.prize_timer == 0)
			s_st.prize_hop = 2;
	} else if (s_m->kind == K_JOCKEYC || s_m->kind == K_INTTOOTE) {
		hopper_frame();
		rtc_frame();
	}
	for (int line = 0; line < LINES; line++) {
		if (irq == IRQ_1_2) {
			if (line == 240)
				m68k_bus_hold_irq(1);
			else if (line == 112)
				m68k_bus_hold_irq(2);
		} else if (irq == IRQ_JOCKEY) {
			if (line == 15)
				m68k_bus_hold_irq(4);
			else if (line == 38)
				m68k_bus_hold_irq(1);
			else if (line == 61)
				m68k_bus_hold_irq(2);
			else if (line >= 85 && line % 23 == 0)
				m68k_bus_hold_irq(6);
		}
		if (line == VIS_Y1 + 1) {   // (the vblank)
			capture();
			rbx_video_frame_done();
			if (irq == IRQ_VBL3_HOLD)
				m68k_bus_hold_irq(3);
			else if (irq == IRQ_VBL2_ASSERT)
				m68k_bus_set_line(2, 1);
			else if (irq == IRQ_VBL3_ASSERT)
				m68k_bus_set_line(3, 1);
		}
		run_cpus();
		run_sound();
	}
	sound_frame_end();
}

static int st_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	const uint32_t csize = m68k_context_size();
	if (csize > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus.irq_pending, m68k_bus.irq_asserted };
	uint64_t fr[4] = { s_cpu_frac, s_snd_frac, s_z_frac, s_ym_frac };
	z80 z = s_z;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, s_ram_size);
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "x1ctrl", s_x1->ctrl, sizeof(s_x1->ctrl));
	rc |= rbx_state_io(ctx, "x1ylow", s_x1->ylow, sizeof(s_x1->ylow));
	rc |= rbx_state_io(ctx, "x1code", s_x1->code, sizeof(s_x1->code));
	rc |= rbx_state_io(ctx, "x1bg", &s_x1->bgflag, 1);
	if (s_m->snd == SND_X1)
		rc |= rbx_state_io(ctx, "x1010", &s_snd, X1_010_STATE_SIZE);
	else {
		rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
		rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
		if (s_ym_on)
			rc |= rbx_ym_state(0, ctx, load);
		if (s_m->snd != SND_THUNDERLBL)
			rc |= rbx_state_io(ctx, "oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	}
	rc |= rbx_state_io(ctx, "st", &s_st, sizeof(s_st));
	if (s_lay)
		rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	if (s_m->kind == K_INTTOOTE)
		rc |= rbx_state_io(ctx, "it700", s_it700, sizeof(s_it700));
	if (s_m->kind == K_PAIRLOVE) {
		rc |= rbx_state_io(ctx, "prot", s_protram, sizeof(s_protram));
		rc |= rbx_state_io(ctx, "proto", s_protram_old, sizeof(s_protram_old));
	}
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	if (rc)
		return -1;
	if (load) {
		m68k_set_context(cpu);
		m68k_bus.irq_pending = irq[0];
		m68k_bus.irq_asserted = irq[1];
		m68k_bus_update_irq();
		if (s_m->snd != SND_X1) {
			z.read_byte = z_read;
			z.write_byte = z_write;
			z.port_in = z_in;
			z.port_out = z_out;
			z.userdata = NULL;
			z.read_map = s_zmap;
			s_z = z;
		}
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_z_frac = fr[2];
		s_ym_frac = fr[3];
	}
	return 0;
}

static int st_save(void *ctx) { return st_state(ctx, 0); }
static int st_load(void *ctx) { return st_state(ctx, 1); }
static int st_supported(const char *machine)
{
	for (size_t i = 0; i < sizeof(s_machines) / sizeof(s_machines[0]); i++)
		if (!strcmp(s_machines[i].name, machine))
			return 1;
	return 0;
}

const rbx_driver rbx_drv_seta_sp = { "Seta Hardware", st_init, st_exit, st_reset, st_frame, st_save, st_load,
                                  st_supported, st_render, NULL };
