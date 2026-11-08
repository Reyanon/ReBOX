/*  Space Invaders family on the Intel 8080 (Taito sets and their clones), ported from MAME
    src/mame/midw8080/8080bw.cpp, 8080bw_v.cpp, mw8080bw.cpp, mw8080bw_v.cpp (license BSD-3-Clause,
    copyright-holders: Nicola Salmoria, Tormod Tjaberg, Mirko Buffoni, Lee Taylor, Valerio Verrando, Zsolt Vasvari,
    Aaron Giles, Jonathan Gevaryahu, hap, Robbbert; thanks-to: Michael Strutts, Marco Cassili) and
    src/devices/machine/mb14241.cpp (BSD-3-Clause, Fabio Priuli). Colour overlays from MAME's invaders.lay,
    galactic.lay and escmars.lay (CC0-1.0).
    Memory and I/O maps, the interrupt generator (RST 1 at vpos 0x60, RST 2 at vpos 0xe0 through the INTE line),
    the MB14241 shifter, the watchdogs and the screen updates (colour PROMs, scattered colour RAM, Polaris clouds)
    follow MAME. Sound: si8080_a.c. Straight Flush (M6800 CPU) is not emulated.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/i8080/i8080.h"
#include "si8080.h"

#include <stdlib.h>
#include <string.h>

// screen (MW8080BW_*): 320 x 262 total, 260 x 224 visible
#define HPIX 260
#define VPIX 224
#define VTOTAL 262
#define VBSTART 224
#define VCOUNTER_START_NO_VBLANK 0x20
#define VCOUNTER_START_VBLANK 0xda
#define LINE_RATE 15600   // pixel clock 4.992 MHz / 320

enum { MAP_MW, MAP_SCHASER, MAP_ESCMARS };
enum { IO_INVADERS, IO_INVNOMB, IO_INVADPT2, IO_SPCEWARS, IO_LRESCUE, IO_LRESCUEM2, IO_SCHASER, IO_SCHASERCV,
       IO_CRASHRD, IO_LUPIN3, IO_POLARIS, IO_BALLBOMB, IO_INDIANBT, IO_INDIANBTBR, IO_STEELWKR };
enum { SCR_MW, SCR_INVADERS, SCR_INVADPT2, SCR_BALLBOMB, SCR_SCHASER, SCR_SCHASERCV, SCR_POLARIS, SCR_INDIANBT, SCR_LUPIN3 };
enum { PAL_BW, PAL_RBG, PAL_RGB };
enum { CAB_NONE, CAB_PORT, CAB_IN2_BIT2, CAB_IN2_BIT6 };   // where the flip screen logic reads the cabinet type

typedef struct {
	const char *name;
	int map, io, screen, pal, sound;
	int watchdog;       // vblanks, 0 = none
	uint32_t clock;
	int cab;
} machine_t;

static const machine_t s_machines[] = {
	{ "invaders",   MAP_MW,      IO_INVADERS,   SCR_INVADERS,  PAL_BW,  SIA_INVADERS,   255, 1996800, CAB_PORT },
	{ "invnomb",    MAP_MW,      IO_INVNOMB,    SCR_INVADERS,  PAL_BW,  SIA_INVADERS,   255, 1996800, CAB_PORT },
	{ "invadpt2",   MAP_MW,      IO_INVADPT2,   SCR_INVADPT2,  PAL_RBG, SIA_INVADPT2,   255, 1996800, CAB_PORT },
	{ "spcewars",   MAP_MW,      IO_SPCEWARS,   SCR_INVADERS,  PAL_BW,  SIA_SPCEWARS,   0,   1996800, CAB_PORT },
	{ "lrescue",    MAP_MW,      IO_LRESCUE,    SCR_INVADPT2,  PAL_RBG, SIA_LRESCUE,    0,   1996800, CAB_PORT },
	{ "lrescuem2",  MAP_MW,      IO_LRESCUEM2,  SCR_INVADPT2,  PAL_RBG, SIA_LRESCUE,    0,   1996800, CAB_PORT },
	{ "escmars",    MAP_ESCMARS, IO_LRESCUE,    SCR_MW,        PAL_BW,  SIA_LRESCUE,    0,   1800000, CAB_PORT },
	{ "schaser",    MAP_SCHASER, IO_SCHASER,    SCR_SCHASER,   PAL_RBG, SIA_SCHASER,    255, 1996800, CAB_IN2_BIT6 },
	{ "schasercv",  MAP_SCHASER, IO_SCHASERCV,  SCR_SCHASERCV, PAL_RBG, SIA_SCHASERCV,  0,   1996800, CAB_PORT },
	{ "crashrd",    MAP_SCHASER, IO_CRASHRD,    SCR_SCHASER,   PAL_RBG, SIA_CRASHRD,    255, 1996800, CAB_IN2_BIT6 },
	{ "lupin3",     MAP_MW,      IO_LUPIN3,     SCR_INDIANBT,  PAL_RGB, SIA_LUPIN3,     0,   1996800, CAB_IN2_BIT2 },
	{ "lupin3a",    MAP_SCHASER, IO_LUPIN3,     SCR_LUPIN3,    PAL_RBG, SIA_LUPIN3,     0,   1996800, CAB_IN2_BIT2 },
	{ "polaris",    MAP_SCHASER, IO_POLARIS,    SCR_POLARIS,   PAL_RBG, SIA_POLARIS,    255, 1996800, CAB_IN2_BIT2 },
	{ "ballbomb",   MAP_MW,      IO_BALLBOMB,   SCR_BALLBOMB,  PAL_RBG, SIA_BALLBOMB,   0,   1996800, CAB_PORT },
	{ "indianbt",   MAP_MW,      IO_INDIANBT,   SCR_INDIANBT,  PAL_RGB, SIA_INDIANBT,   0,   1996800, CAB_PORT },
	{ "indianbtbr", MAP_SCHASER, IO_INDIANBTBR, SCR_INDIANBT,  PAL_RGB, SIA_INDIANBTBR, 0,   1996800, CAB_PORT },
	{ "steelwkr",   MAP_MW,      IO_STEELWKR,   SCR_INVADPT2,  PAL_RBG, SIA_INVADPT2,   0,   1996800, CAB_PORT },
	// "sflush" (Straight Flush) runs on a Motorola 6800: not emulated
};

// colour overlays (MAME layouts, in the rotated view: 224 wide, 260 high)
typedef struct { int l, t, r, b; uint8_t cr, cg, cb; } ov_rect;
static const ov_rect ov_invaders[] = {
	{ 0, 0, 224, 260, 255, 255, 255 }, { 0, 184, 224, 240, 32, 255, 32 }, { 16, 240, 134, 260, 32, 255, 32 },
	{ 0, 32, 224, 62, 255, 32, 32 }, { 0, 0, 0, 0, 0, 0, 0 } };
static const ov_rect ov_galactic[] = {
	{ 0, 0, 224, 260, 0, 255, 102 }, { 0, 47, 224, 70, 255, 51, 38 }, { 0, 70, 224, 192, 255, 255, 51 },
	{ 0, 192, 224, 214, 51, 153, 255 }, { 0, 0, 0, 0, 0, 0, 0 } };
static const ov_rect ov_escmars[] = {
	{ 0, 0, 224, 40, 32, 255, 32 }, { 0, 40, 224, 180, 255, 128, 16 }, { 0, 180, 224, 240, 255, 32, 32 },
	{ 0, 240, 224, 260, 32, 255, 32 }, { 0, 0, 0, 0, 0, 0, 0 } };
static const struct { const char *set; const ov_rect *ov; } s_overlays[] = {
	{ "sitv1", ov_invaders }, { "sitv", ov_invaders }, { "invadrmr", ov_invaders }, { "invaderl", ov_invaders },
	{ "spcewars", ov_invaders }, { "spceking", ov_invaders }, { "sinvzen", ov_invaders }, { "ultrainv", ov_invaders },
	{ "jspecter", ov_invaders }, { "jspecter2", ov_invaders },
	{ "galactic", ov_galactic }, { "spacmiss", ov_galactic },
	{ "escmars", ov_escmars }, { "resclunar", ov_escmars },
};

// custom input bits (MAME invaders_state / invaders_clone_state control readers)
enum { CK_NONE, CK_P1, CK_CAB, CK_SICV, CK_PT2_1, CK_PT2_2 };

// ---- state ----
static const machine_t *s_m;
static i8080_t s_cpu;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static const uint8_t *s_prom, *s_cloud;
static uint32_t s_prom_size;
static uint8_t s_ram[0x2000];
static uint8_t s_colorram[0x800];   // m_scattered_colorram
static uint8_t s_zero[0x100];
static uint16_t s_shift_data;
static uint8_t s_shift_count;
static uint8_t s_int_enable;
static int s_line;
static int s_wd_count;
static uint8_t s_cloud_speed, s_cloud_pos;
static uint32_t s_rand;
static sia_latch s_latch;
static uint64_t s_frac;
static int s_port_in[3], s_port_cab, s_port_contp[2], s_port_p2gate;
static int s_ck[3];
static uint32_t s_ck_mask[3];
static int s_ck_shift[3];
static uint8_t *s_overlay;   // palette index of a lit pixel, HPIX x VPIX (B&W screens)

// snapshot for the renderer
typedef struct {
	uint8_t ram[0x2000];
	uint8_t colorram[0x800];
	sia_latch l;
	uint8_t cloud_pos;
} snap_t;
static snap_t *s_snap;

// ---- inputs ----
static int cab_cocktail(void)
{
	return s_port_cab >= 0 ? (int)(rbx_port_read(s_port_cab) & 1) : 0;
}

static uint32_t contp(int p, uint32_t width_mask)
{
	return s_port_contp[p] >= 0 ? rbx_port_read(s_port_contp[p]) & width_mask : 0;
}

static uint8_t port_in(int i)
{
	if (s_port_in[i] < 0)
		return 0;
	uint32_t v = rbx_port_read(s_port_in[i]);
	if (s_ck[i] != CK_NONE) {
		uint32_t w = s_ck_mask[i] >> s_ck_shift[i], c = 0;
		uint32_t cab = s_port_cab >= 0 ? rbx_port_read(s_port_cab) : 0;
		switch (s_ck[i]) {
		case CK_P1: c = contp(0, w); break;                                        // invaders_in1_control_r
		case CK_CAB: c = contp(cab_cocktail() ? 1 : 0, w); break;                  // invaders_in2_control_r
		case CK_SICV:                                                              // sicv_in2_control_r
			c = contp(cab_cocktail() ? 1 : 0, w) | (s_port_p2gate >= 0 ? rbx_port_read(s_port_p2gate) & w : 0);
			break;
		case CK_PT2_1: c = contp(0, w) | (contp(1, w) & ~cab); break;              // invadpt2_in1_control_r
		case CK_PT2_2: c = contp(1, w) | (contp(0, w) & ~cab); break;              // invadpt2_in2_control_r
		}
		v = (v & ~s_ck_mask[i]) | ((c << s_ck_shift[i]) & s_ck_mask[i]);
	}
	return (uint8_t)v;
}

static void latch_cab(void)
{
	switch (s_m->cab) {
	case CAB_PORT: s_latch.cab = (uint8_t)cab_cocktail(); break;
	case CAB_IN2_BIT2: s_latch.cab = (uint8_t)((port_in(2) >> 2) & 1); break;
	case CAB_IN2_BIT6: s_latch.cab = (uint8_t)((port_in(2) >> 6) & 1); break;
	default: s_latch.cab = 0; break;
	}
	s_latch.cab_invaders = (uint8_t)cab_cocktail();
}

static void sound_w(int which, uint8_t data)
{
	latch_cab();
	sia_write(which, data, &s_latch);
}

// ---- MB14241 ----
static void shift_count_w(uint8_t d) { s_shift_count = (uint8_t)(~d & 7); }
static void shift_data_w(uint8_t d) { s_shift_data = (uint16_t)((s_shift_data >> 8) | ((uint16_t)d << 7)); }
static uint8_t shift_result_r(void) { return (uint8_t)((s_shift_data >> s_shift_count) & 0xff); }

static void watchdog_w(void) { s_wd_count = 0; }

// ---- I/O ----
static uint8_t io_r(void *u, uint8_t p)
{
	(void)u;
	switch (s_m->io) {
	case IO_INVADERS: case IO_INVNOMB:
		switch (p & 7) {
		case 0: case 4: return port_in(0);
		case 1: case 5: return port_in(1);
		case 2: case 6: return port_in(2);
		default: return s_m->io == IO_INVADERS ? shift_result_r() : 0;
		}
	case IO_POLARIS:
		switch (p) {
		case 0: {
			uint8_t d = port_in(0);
			return s_latch.flip_screen ? d : (uint8_t)((d & 7) | (port_in(1) & 0xf8));
		}
		case 1: return port_in(1);
		case 2: return port_in(2);
		case 3: return shift_result_r();
		}
		return 0;
	case IO_SCHASERCV:
		if (p == 2) {
			uint8_t d = port_in(2);
			if (s_latch.flip_screen)
				return d;
			uint8_t in1 = port_in(1);
			return (uint8_t)((d & 0x89) | (in1 & 0x70) | (((in1 >> 3) & 1) << 1) | (((in1 >> 7) & 1) << 2));
		}
		break;
	case IO_INDIANBT: case IO_INDIANBTBR:
		if (p == 0 && s_m->io == IO_INDIANBT) {
			// protection / sound hardware check, by the address after the IN instruction
			switch (s_cpu.s.pc) {
			case 0x5fed: return 0x10;
			case 0x5ffc: return 0;
			}
			s_rand = s_rand * 1103515245u + 12345u;
			return (uint8_t)(s_rand >> 16);
		}
		if (p == 1) {
			uint8_t d = port_in(1);
			return s_latch.flip_screen ? (uint8_t)((d & 0x8f) | (port_in(2) & 0x70)) : d;
		}
		break;
	case IO_CRASHRD: case IO_STEELWKR:
		if (p == 0)
			return 0;
		break;
	case IO_LRESCUEM2:
		if (p == 3)
			return 0;
		break;
	}
	switch (p) {
	case 0: return port_in(0);
	case 1: return port_in(1);
	case 2: return port_in(2);
	case 3: return shift_result_r();
	}
	return 0;
}

static void io_w(void *u, uint8_t p, uint8_t d)
{
	(void)u;
	switch (s_m->io) {
	case IO_INVADERS: case IO_INVNOMB:
		switch (p & 7) {
		case 2: if (s_m->io == IO_INVADERS) shift_count_w(d); break;
		case 3: sound_w(1, d); break;
		case 4: if (s_m->io == IO_INVADERS) shift_data_w(d); break;
		case 5: sound_w(2, d); break;
		case 6: watchdog_w(); break;
		}
		return;
	case IO_POLARIS:
		switch (p) {
		case 0: shift_count_w(d); break;
		case 2: sound_w(1, d); break;
		case 3: shift_data_w(d); break;
		case 4: sound_w(2, d); break;
		case 5: watchdog_w(); break;
		case 6: sound_w(3, d); break;
		}
		return;
	case IO_LRESCUEM2:
		if (p == 3) sound_w(1, d);
		else if (p == 5) sound_w(2, d);
		return;
	default:
		break;
	}
	switch (p) {
	case 0: if (s_m->io == IO_LUPIN3) sound_w(3, d); break;
	case 1: if (s_m->io == IO_BALLBOMB) sound_w(3, d); break;
	case 2: shift_count_w(d); break;
	case 3: sound_w(1, d); break;
	case 4: shift_data_w(d); break;
	case 5: sound_w(2, d); break;
	case 6:
		if (s_m->io == IO_INVADPT2 || s_m->io == IO_SCHASER || s_m->io == IO_CRASHRD)
			watchdog_w();
		break;
	case 7: if (s_m->io == IO_INDIANBT) sound_w(3, d); break;
	}
}

// ---- memory ----
static uint8_t mem_r(void *u, uint16_t a)
{
	(void)u;
	if (s_m->map == MAP_SCHASER && a >= 0xc000 && a < 0xe000) {
		uint16_t o = a - 0xc000;
		return s_colorram[(o & 0x1f) | ((o & 0x1f80) >> 2)];
	}
	return 0;   // unmapped
}

static void mem_w(void *u, uint16_t a, uint8_t d)
{
	(void)u;
	if (s_m->map == MAP_MW)
		a &= 0x7fff;
	if ((a & 0xe000) == 0x2000 || (s_m->map == MAP_MW && (a & 0xe000) == 0x6000)) {
		s_ram[a & 0x1fff] = d;
		return;
	}
	if (s_m->map == MAP_SCHASER && a >= 0xc000 && a < 0xe000) {
		uint16_t o = a - 0xc000;
		s_colorram[(o & 0x1f) | ((o & 0x1f80) >> 2)] = d;
	}
}

static void build_pages(void)
{
	for (int p = 0; p < 256; p++) {
		uint32_t a = (uint32_t)p << 8;
		const uint8_t *pg = NULL;
		uint32_t m = s_m->map == MAP_MW ? (a & 0x7fff) : a;
		uint32_t rom_end = s_m->map == MAP_ESCMARS ? 0x5800 : 0x6000;
		if (m < 0x2000)
			pg = s_rom + m;
		else if (m < 0x4000)
			pg = s_ram + (m - 0x2000);
		else if (m < rom_end)
			pg = s_rom + m;
		else if (s_m->map == MAP_MW && m < 0x8000)
			pg = s_ram + (m - 0x6000);
		else if (!(s_m->map == MAP_SCHASER && m >= 0xc000 && m < 0xe000))
			pg = s_zero;
		s_cpu.rpage[p] = pg;
	}
}

// ---- interrupts (mw8080bw_state::interrupt_trigger / interrupt_vector) ----
static uint8_t vpos_to_counter(int vpos)
{
	return (uint8_t)(vpos >= VBSTART ? vpos - VBSTART + VCOUNTER_START_VBLANK : vpos + VCOUNTER_START_NO_VBLANK);
}

static uint8_t irq_ack(void *u, int line)
{
	(void)u;
	if (line != I8080_INTR_LINE)
		return 0xff;
	uint8_t counter = vpos_to_counter(s_line);
	uint8_t vector = (uint8_t)(0xc7 | ((counter & 0x40) >> 2) | ((~counter & 0x40) >> 3));
	i8080_set_input(&s_cpu, I8080_INTR_LINE, 0);
	return vector;
}

static void inte_w(void *u, int state) { (void)u; s_int_enable = (uint8_t)state; }

// ---- video ----
static inline void put(uint16_t *bm, int flip, int row, int x, uint16_t c)
{
	if (flip)
		bm[(VPIX - 1 - row) * HPIX + (HPIX - 1 - x)] = c;
	else
		bm[row * HPIX + x] = c;
}

static void render_bw(const snap_t *s, int offset4)
{
	// mw8080bw / invaders: video RAM is read every 8 pixels starting with pixel 4
	uint16_t *bm = rbx_video.bitmap;
	int flip = offset4 ? s->l.flip_screen : 0;   // screen_update_mw8080bw has no flip
	for (int row = 0; row < VPIX; row++) {
		const uint8_t *src = s->ram + ((row + VCOUNTER_START_NO_VBLANK) << 5);
		for (int x = 0; x < 4; x++)
			put(bm, flip, row, x, 0);
		for (int col = 0; col < 32; col++) {
			uint8_t d = src[col];
			for (int b = 0; b < 8; b++) {
				int x = 4 + col * 8 + b;
				put(bm, flip, row, x, (d >> b) & 1 ? (s_overlay ? s_overlay[row * HPIX + x] : 1) : 0);
			}
		}
	}
}

static void render_clone(const snap_t *s)
{
	uint16_t *bm = rbx_video.bitmap;
	int flip = s->l.flip_screen;
	const uint8_t *prom = s_prom;
	const uint8_t *cmap = prom ? prom + (s->l.color_map && s_prom_size >= 0x800 ? 0x400 : 0) : NULL;
	int clear = 0;
	switch (s_m->screen) {
	case SCR_BALLBOMB: case SCR_SCHASERCV: clear = 2; break;
	case SCR_SCHASER: clear = s->l.schaser_bg_disable ? 0 : 2; break;
	case SCR_POLARIS: clear = 6; break;
	}
	for (int offs = VCOUNTER_START_NO_VBLANK << 5; offs < 0x2000; offs++) {
		int y = offs >> 5, row = y - VCOUNTER_START_NO_VBLANK;
		uint8_t x = (uint8_t)(offs << 3);
		uint8_t data = s->ram[offs];
		int caddr = (offs >> 8 << 5) | (offs & 0x1f);
		uint8_t scat = s->colorram[(offs & 0x1f) | ((offs & 0x1f80) >> 2)];
		int fore = 0, back = 0;
		switch (s_m->screen) {
		case SCR_INVADPT2:
			fore = s->l.screen_red ? 1 : (cmap ? cmap[caddr] & 7 : 7);
			break;
		case SCR_BALLBOMB:
			fore = s->l.screen_red ? 1 : (cmap ? cmap[caddr] & 7 : 7);
			back = 2;
			break;
		case SCR_SCHASER:
			fore = scat & 7;
			if (!s->l.schaser_bg_disable) {
				uint8_t bd = prom ? prom[caddr] : 0;
				back = ((bd & 0x0c) == 0x0c && s->l.schaser_bg_select) ? 4 : 2;
			}
			break;
		case SCR_SCHASERCV:
			fore = scat & 7;
			back = 2;
			break;
		case SCR_INDIANBT:
			fore = cmap ? cmap[caddr] & 7 : 7;
			break;
		case SCR_LUPIN3:
			fore = ~scat & 7;
			break;
		case SCR_POLARIS: {
			uint8_t pm = prom ? prom[caddr] : 0;
			back = (pm & 1) ? 6 : 2;
			fore = ~scat & 7;
			uint8_t cloud_y = (uint8_t)(y - s->cloud_pos);
			if (!((pm & 0x08) || cloud_y >= 64) && s_cloud) {
				for (int i = 0; i < 8; i++) {
					int color;
					if (data & 1)
						color = fore;
					else {
						int bit = 1 << (~x & 3);
						int co = ((x >> 2) & 3) | ((~cloud_y & 0x3f) << 2);
						color = (s_cloud[co] & bit) ? 7 : back;
					}
					put(bm, flip, row, x, (uint16_t)color);
					x++;
					data >>= 1;
				}
				continue;
			}
			break;
		}
		}
		for (int i = 0; i < 8; i++) {
			put(bm, flip, row, x + i, (uint16_t)((data >> i) & 1 ? fore : back));
		}
	}
	for (int row = 0; row < VPIX; row++)
		for (int x = 256; x < HPIX; x++)
			put(bm, flip, row, x, (uint16_t)clear);
}

static void si_render(void)
{
	const snap_t *s = s_snap;
	if (s_m->screen == SCR_MW)
		render_bw(s, 0);
	else if (s_m->screen == SCR_INVADERS)
		render_bw(s, 1);
	else
		render_clone(s);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->ram, s_ram, sizeof(s_ram));
	if (s_m->map == MAP_SCHASER)
		memcpy(s_snap->colorram, s_colorram, sizeof(s_colorram));
	s_snap->l = s_latch;
	s_snap->cloud_pos = s_cloud_pos;
	rbx_video_frame_done();
}

static void setup_palette(const rbx_game *g)
{
	for (int i = 0; i < 16; i++)
		rbx_video.palette[i] = 0;
	for (int i = 0; i < 8; i++) {
		int r = i & 1, gg, b;
		if (s_m->pal == PAL_RGB) { gg = (i >> 1) & 1; b = (i >> 2) & 1; }
		else { gg = (i >> 2) & 1; b = (i >> 1) & 1; }   // RBG_3BIT
		rbx_video.palette[i] = (uint32_t)((r ? 0xff0000 : 0) | (gg ? 0xff00 : 0) | (b ? 0xff : 0));
	}
	if (s_m->pal == PAL_BW) {
		rbx_video.palette[0] = 0;
		rbx_video.palette[1] = 0xffffff;
	}
	// colour overlay: the lit pixels take the colour of the overlay rectangle over them (multiply blend on white)
	const ov_rect *ov = NULL;
	for (size_t i = 0; i < sizeof(s_overlays) / sizeof(s_overlays[0]); i++)
		if (!strcmp(s_overlays[i].set, g->set))
			ov = s_overlays[i].ov;
	if (!ov || (s_m->screen != SCR_MW && s_m->screen != SCR_INVADERS))
		return;
	s_overlay = (uint8_t *)malloc(HPIX * VPIX);
	if (!s_overlay)
		return;
	int n = 0;
	for (; ov[n].r; n++)
		rbx_video.palette[8 + n] = (uint32_t)(ov[n].cr << 16 | ov[n].cg << 8 | ov[n].cb);
	for (int row = 0; row < VPIX; row++)
		for (int x = 0; x < HPIX; x++) {
			// ROT270: view x = native row, view y = HPIX - 1 - native x
			int vx = row, vy = HPIX - 1 - x;
			uint8_t c = 0;   // outside every rectangle: black (multiply with nothing)
			for (int k = 0; k < n; k++)
				if (vx >= ov[k].l && vx < ov[k].r && vy >= ov[k].t && vy < ov[k].b)
					c = (uint8_t)(8 + k);
			s_overlay[row * HPIX + x] = c;
		}
}

// ---- machine ----
static const machine_t *find_machine(const char *name)
{
	for (size_t i = 0; i < sizeof(s_machines) / sizeof(s_machines[0]); i++)
		if (!strcmp(s_machines[i].name, name))
			return &s_machines[i];
	return NULL;
}

static void machine_reset(void)
{
	i8080_reset(&s_cpu);
	i8080_set_input(&s_cpu, I8080_INTR_LINE, 0);
	s_shift_data = 0;
	s_shift_count = 0;
	s_wd_count = 0;
	uint8_t cab = s_latch.cab, cabi = s_latch.cab_invaders;
	memset(&s_latch, 0, sizeof(s_latch));
	s_latch.cab = cab;
	s_latch.cab_invaders = cabi;
	sia_reset();
}

static void si_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_colorram, 0, sizeof(s_colorram));
	i8080_init(&s_cpu, 0);
	s_int_enable = 1;   // mw8080bw_state: m_int_enable(true)
	s_cloud_speed = s_cloud_pos = 0;
	s_frac = 0;
	s_line = 0;
	s_rand = 1;
	machine_reset();
}

static void si_exit(void);

static void find_custom(const rbx_game *g, int i, int kind)
{
	s_ck[i] = CK_NONE;
	if (s_port_in[i] < 0)
		return;
	const rbx_ipt_port *p = &g->ports[s_port_in[i]];
	for (int f = p->first; f < p->first + p->count; f++) {
		const rbx_ipt_field *fd = &g->fields[f];
		if (fd->type == RBX_IPT_CUSTOM && fd->mask != 0xff) {
			s_ck[i] = kind;
			s_ck_mask[i] = fd->mask;
			int sh = 0;
			while (!((fd->mask >> sh) & 1))
				sh++;
			s_ck_shift[i] = sh;
		}
	}
}

static int si_init(const rbx_game *g)
{
	s_m = find_machine(g->machine);
	if (!s_m) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_rom = rbx_region("maincpu", &s_rom_size);
	if (!s_rom || s_rom_size < 0x10000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_prom = rbx_region("proms", &s_prom_size);
	uint32_t csize;
	s_cloud = rbx_region("user1", &csize);
	if (s_m->screen != SCR_POLARIS)
		s_cloud = NULL;
	s_port_in[0] = rbx_port_find("IN0");
	s_port_in[1] = rbx_port_find("IN1");
	s_port_in[2] = rbx_port_find("IN2");
	s_port_cab = rbx_port_find("CAB");
	s_port_contp[0] = rbx_port_find("CONTP1");
	s_port_contp[1] = rbx_port_find("CONTP2");
	s_port_p2gate = rbx_port_find("P2GATE");
	// which control reader the custom bits use: invadpt2 couples the players with the 3-bit cabinet DIP
	int pt2 = 0;
	if (s_port_cab >= 0) {
		const rbx_ipt_port *cp = &g->ports[s_port_cab];
		for (int f = cp->first; f < cp->first + cp->count; f++)
			if (g->fields[f].mask == 0x07)
				pt2 = 1;
	}
	find_custom(g, 0, CK_NONE);
	find_custom(g, 1, pt2 ? CK_PT2_1 : CK_P1);
	find_custom(g, 2, pt2 ? CK_PT2_2 : s_port_p2gate >= 0 ? CK_SICV : CK_CAB);
	s_snap = (snap_t *)calloc(1, sizeof(snap_t));
	if (!s_snap) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (rbx_video_setup(HPIX, VPIX, 16)) {
		si_exit();
		return -1;
	}
	setup_palette(g);
	if (rbx_audio_setup(SIA_RATE) || sia_init(s_m->sound, g->set)) {
		si_exit();
		return -1;
	}
	memset(&s_cpu, 0, sizeof(s_cpu));
	s_cpu.read = mem_r;
	s_cpu.write = mem_w;
	s_cpu.in = io_r;
	s_cpu.out = io_w;
	s_cpu.irq_ack = irq_ack;
	s_cpu.inte = inte_w;
	build_pages();
	memset(&s_latch, 0, sizeof(s_latch));
	si_reset();
	return 0;
}

static void si_exit(void)
{
	sia_exit();
	free(s_snap);
	s_snap = NULL;
	free(s_overlay);
	s_overlay = NULL;
}

static void si_frame(void)
{
	for (int line = 0; line < VTOTAL; line++) {
		s_line = line;
		if (line == 0x60 || line == VBSTART) {
			// interrupt_trigger: counter 0x80 (vpos 0x60) and 0xda (vpos 0xe0)
			i8080_set_input(&s_cpu, I8080_INTR_LINE, s_int_enable);
		}
		if (line == VBSTART) {
			vblank_capture();
			if (s_m->screen == SCR_POLARIS && ++s_cloud_speed >= 4) {   // every 4 frames
				s_cloud_speed = 0;
				s_cloud_pos++;
			}
			if (s_m->watchdog && ++s_wd_count >= s_m->watchdog) {
				rbx_log("si8080: watchdog reset");
				machine_reset();
			}
		}
		for (int half = 0; half < 2; half++) {
			i8080_run(&s_cpu, rbx_line_cycles(&s_frac, s_m->clock, LINE_RATE * 2, 1000));
			rbx_audio_put(sia_sample());
		}
	}
}

static int si_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "cpu", &s_cpu.s, sizeof(s_cpu.s));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "colorram", s_colorram, sizeof(s_colorram));
	rc |= rbx_state_io(ctx, "shift", &s_shift_data, sizeof(s_shift_data));
	rc |= rbx_state_io(ctx, "shiftc", &s_shift_count, 1);
	rc |= rbx_state_io(ctx, "inte", &s_int_enable, 1);
	rc |= rbx_state_io(ctx, "wd", &s_wd_count, sizeof(s_wd_count));
	rc |= rbx_state_io(ctx, "cloud", &s_cloud_pos, 1);
	rc |= rbx_state_io(ctx, "cloudsp", &s_cloud_speed, 1);
	rc |= rbx_state_io(ctx, "latch", &s_latch, sizeof(s_latch));
	rc |= rbx_state_io(ctx, "frac", &s_frac, sizeof(s_frac));
	rc |= rbx_state_io(ctx, "rand", &s_rand, sizeof(s_rand));
	rc |= sia_state(ctx, load);
	return rc ? -1 : 0;
}

static int si_save(void *ctx) { return si_state(ctx, 0); }
static int si_load(void *ctx) { return si_state(ctx, 1); }
static int si_supported(const char *machine) { return find_machine(machine) != NULL; }

const rbx_driver rbx_drv_si8080 = { "Space Invaders (8080)", si_init, si_exit, si_reset, si_frame, si_save, si_load,
                                    si_supported, si_render };
