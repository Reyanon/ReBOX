/*  Namco Pac-Man hardware and its conversions, bootlegs and hacks: Puck Man / Pac-Man, Pac-Man Plus, Ms. Pac-Man (the
    auxiliary board's decode latch and its bootlegs), Crush Roller / Make Trax, Eyes, Mr. TNT, Lizard Wizard, Woodpecker,
    Naughty Mouse, Ponpoko, Ali Baba, Dream Shopper, Van-Van Car, the Epos kits (The Glob, Beastie Feastie, Eeekk!,
    Atlantic City Action), Jump Shot, Shoot the Bull, Big Bucks, Rock-N-Roll Trivia 2, Number Crash, Cannon Ball,
    Super ABC, Pacman Club, Ms Pac Man Twin, the Pengo bootlegs on Pac-Man boards ...
    Ported from MAME 0.289 src/mame/pacman/pacman.cpp, pacman.h, pacman_v.cpp, pacman_m.cpp, pacplus.cpp, jumpshot.cpp
    (BSD-3-Clause, copyright-holders: Nicola Salmoria, Stephane Humbert; pacman_m.cpp: smf, Mike Balfour, David Widel)
    and MAME's compute_resistor_weights (src/emu/video/resnet.cpp, BSD-3-Clause, copyright-holders: Couriersud).
    A Z80 at 3.072 MHz, the LS259 output latch, the 3-voice Namco WSG (or an AY-3-8910 / AY-3-8912 / two SN76496),
    a 36 x 28 tilemap of 8x8 2bpp characters and eight 16x16 2bpp sprites, 288 x 224 at 60.6 Hz.
    The frame is drawn at vblank from a copy of the video RAM: the characters from a cache redrawn where a tile changed.
    This file is part of ReBOX. GPL-2.0-or-later.

    Not done: the S2650 conversions (Driving Force, Eight Ball Action, Porky: no S2650 core), the watchdog, the coin
    counters / lockout and the LEDs, mspactwin's mid-frame video RAM updates (update_now: the whole frame is drawn at
    vblank).
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/namco_wsg.h"
#include "../../sound/ay8910.h"
#include "../../sound/sn76496.h"
#include "../../rbx_prof.h"

#include <stdlib.h>
#include <string.h>

#define SCR_W 288
#define VIS_H 224
#define LINES 264
#define LINE_CYCLES 192          // 18.432 MHz / 6 a 384-pixel line (18.432 MHz / 3)
#define FRAME_MHZ 60606          // 18.432 MHz / 3 / (384 x 264)
#define CPU_CLOCK 3072000
#define WSG_RATE 48000           // (3 samples a line)
#define PSG_CLOCK 1789750        // (AY-3-8910 14.318 MHz / 8, AY-3-8912 and the SN76496s 1.78975 MHz)
#define NPENS 128                // (32 colours x 4: the colour table / palette banks are Pengo's, unused here)

enum { MAP_PACMAN, MAP_WOODPEK, MAP_MSPACMAN, MAP_MSPACTWIN, MAP_NUMCRASH, MAP_ALIBABA, MAP_DREMSHPR, MAP_EPOS,
       MAP_ROCKTRV2, MAP_BIGBUCKS, MAP_MSCHAMP, MAP_SUPERABC, MAP_CRUSHS, MAP_PENGOJPM, MAP_BIRDIY, MAP_CANNONBP };
enum { IO_VECTOR, IO_NONE, IO_PIRANHA, IO_NMOUSE, IO_MSPACII, IO_EPOS, IO_MSCHAMP, IO_BIGBUCKS, IO_DREMSHPR, IO_VANVAN,
       IO_CRUSHS };
enum { SND_WSG, SND_AY, SND_SN2, SND_AY8912 };
enum { LAT_STD, LAT_BIRDIY, LAT_NMI, LAT_ALIBABA };
enum { EPOS_NONE, EPOS_GLOB, EPOS_ACITYA, EPOS_EEEKK };
#define F_SUBLATCH 0x01   // (clubpacm, mspactwin: the generic latch at 5080 / 50c0, the multiplexed joysticks)
#define F_RBG      0x02   // (pacman_rbg_palette)
#define F_CRUSH4   0x04   // (gfx_crush4)
#define F_SUPERABC 0x08   // (gfx_superabc)
#define F_INV_SPR  0x10   // (birdiy: sprites mirrored, no offset hack)
#define F_NARROW   0x20   // (vanvan: 256 pixels, 16 to 271)
#define F_PERIODIC 0x40   // (bigbucks: a 1200 Hz IRQ too)
#define F_NOSPR    0x80   // (no "spriteram" share: rocktrv2, bigbucks)
#define F_NOFLIP   0x100  // (numcrash: latch Q3 not connected)

typedef struct {
	const char *name;
	uint8_t map, io, snd, lat, epos;
	uint16_t flags;
} pm_machine;

static const pm_machine s_machines[] = {
	{ "pacman",      MAP_PACMAN,    IO_VECTOR,   SND_WSG,    LAT_STD,     EPOS_NONE,   0 },
	{ "crush2",      MAP_PACMAN,    IO_VECTOR,   SND_WSG,    LAT_STD,     EPOS_NONE,   0 },
	{ "korosuke",    MAP_PACMAN,    IO_VECTOR,   SND_WSG,    LAT_STD,     EPOS_NONE,   0 },
	{ "crush4",      MAP_PACMAN,    IO_VECTOR,   SND_WSG,    LAT_STD,     EPOS_NONE,   F_CRUSH4 },
	{ "crushs",      MAP_CRUSHS,    IO_CRUSHS,   SND_AY8912, LAT_STD,     EPOS_NONE,   0 },
	{ "pengojpm",    MAP_PENGOJPM,  IO_VECTOR,   SND_WSG,    LAT_STD,     EPOS_NONE,   0 },
	{ "birdiy",      MAP_BIRDIY,    IO_NONE,     SND_WSG,    LAT_BIRDIY,  EPOS_NONE,   F_INV_SPR },
	{ "piranha",     MAP_WOODPEK,   IO_PIRANHA,  SND_WSG,    LAT_STD,     EPOS_NONE,   0 },
	{ "nmouse",      MAP_PACMAN,    IO_NMOUSE,   SND_WSG,    LAT_STD,     EPOS_NONE,   0 },
	{ "mspacman",    MAP_MSPACMAN,  IO_VECTOR,   SND_WSG,    LAT_STD,     EPOS_NONE,   0 },
	{ "woodpek",     MAP_WOODPEK,   IO_VECTOR,   SND_WSG,    LAT_STD,     EPOS_NONE,   0 },
	{ "woodpek_rbg", MAP_WOODPEK,   IO_VECTOR,   SND_WSG,    LAT_STD,     EPOS_NONE,   F_RBG },
	{ "clubpacm",    MAP_WOODPEK,   IO_VECTOR,   SND_WSG,    LAT_STD,     EPOS_NONE,   F_SUBLATCH },
	{ "mspactwin",   MAP_MSPACTWIN, IO_VECTOR,   SND_WSG,    LAT_STD,     EPOS_NONE,   F_SUBLATCH },
	{ "numcrash",    MAP_NUMCRASH,  IO_VECTOR,   SND_WSG,    LAT_STD,     EPOS_NONE,   F_NOFLIP },
	{ "alibaba",     MAP_ALIBABA,   IO_VECTOR,   SND_WSG,    LAT_ALIBABA, EPOS_NONE,   0 },
	{ "dremshpr",    MAP_DREMSHPR,  IO_DREMSHPR, SND_AY,     LAT_NMI,     EPOS_NONE,   0 },
	{ "theglobp",    MAP_EPOS,      IO_EPOS,     SND_WSG,    LAT_STD,     EPOS_GLOB,   0 },
	{ "acitya",      MAP_EPOS,      IO_EPOS,     SND_WSG,    LAT_STD,     EPOS_ACITYA, 0 },
	{ "eeekkp",      MAP_EPOS,      IO_EPOS,     SND_WSG,    LAT_STD,     EPOS_EEEKK,  0 },
	{ "vanvan",      MAP_DREMSHPR,  IO_VANVAN,   SND_SN2,    LAT_NMI,     EPOS_NONE,   F_NARROW },
	{ "bigbucks",    MAP_BIGBUCKS,  IO_BIGBUCKS, SND_WSG,    LAT_STD,     EPOS_NONE,   F_PERIODIC | F_NOSPR },
	{ "rocktrv2",    MAP_ROCKTRV2,  IO_VECTOR,   SND_WSG,    LAT_STD,     EPOS_NONE,   F_NOSPR },
	{ "mspacii",     MAP_WOODPEK,   IO_MSPACII,  SND_WSG,    LAT_STD,     EPOS_NONE,   0 },
	{ "mschamp",     MAP_MSCHAMP,   IO_MSCHAMP,  SND_WSG,    LAT_STD,     EPOS_NONE,   0 },
	{ "superabc",    MAP_SUPERABC,  IO_VECTOR,   SND_WSG,    LAT_STD,     EPOS_NONE,   F_SUPERABC },
	{ "cannonbp",    MAP_CANNONBP,  IO_VECTOR,   SND_WSG,    LAT_STD,     EPOS_NONE,   0 },
};

enum { INIT_NONE, INIT_EYES, INIT_PACPLUS, INIT_JUMPSHOT, INIT_MSPACMAN, INIT_MSPACMBE, INIT_PENGOMC1, INIT_MSPACII,
       INIT_MSPACKPLS, INIT_MSPACTWIN, INIT_CLUBPACMA, INIT_MAKETRAX, INIT_MBRUSH, INIT_WOODPEK, INIT_PONPOKO,
       INIT_ALIBABA, INIT_SPRGLOBP2, INIT_ROCKTRV2, INIT_SUPERABC };
static const char *const s_init_names[] = { "", "eyes", "pacplus", "jumpshot", "mspacman", "mspacmbe", "pengomc1",
                                            "mspacii", "mspackpls", "mspactwin", "clubpacma", "maketrax", "mbrush",
                                            "woodpek", "ponpoko", "alibaba", "sprglobp2", "rocktrv2", "superabc" };

// ---- the set's configuration ----
static const pm_machine *s_m;
static int s_init, s_shootbul;
static int s_vis_w, s_xoff;
static uint8_t *s_rom, *s_user1, *s_proms, *s_wave;
static uint32_t s_rom_size, s_user1_size;
static uint8_t *s_romcopy;        // (a 0x20000-byte view of maincpu when its region is smaller: calloc in init)
static uint8_t *s_opcodes;        // (mspactwin: the decrypted opcodes, 64 KB)
static uint8_t *s_chr, *s_spr, *s_clock;   // decoded, a byte a pixel
static uint32_t s_chr_mask, s_spr_mask;
static uint32_t s_pal[NPENS];
static uint8_t s_spr_trans[32];   // (transpen_mask: the pens of a colour whose palette entry is 0)
static int s_port_in0, s_port_in1, s_port_dsw1, s_port_dsw2, s_port_p1, s_port_p2;

static z80 s_main;
static const uint8_t *s_rmap[256];
static uint8_t *s_wmap[256];
static uint8_t s_page_bf[256], s_page_00[256], s_page_sink[256];
static namco_wsg_t s_wsg;
static ay8910_t s_ay;
static sn76496_t s_sn[2];

// ---- the board's state (saved) ----
static struct {
	uint8_t ram[0x1000];      // 0x4000-0x4fff (video RAM, colour RAM, 0x4800 RAM of some boards, work RAM, sprites)
	uint8_t ram2[0x1000];     // (alibaba 0x9000-0x93ff, pengojpm 0xf000-0xffff)
	uint8_t spr2[0x10];       // (the sprites' coordinates)
	uint8_t latch, latch2;    // (the LS259s' outputs)
	int32_t irq_line;         // (the Z80's IRQ line as z80_run reads it)
	uint8_t irq_mask, nmi_line, vector, entry;
	uint8_t flip, charbank, spritebank, sound_on;
	uint8_t bank;             // (mspacman decode, Epos counter, mschamp / superabc bank)
	uint8_t sublatch;
	uint8_t mt_counter, mt_offset, mt_disable;   // (maketrax protection)
	uint8_t mux, mux_data;    // (mschamp)
	uint8_t rt_prot[4], rt_qbank;   // (rocktrv2)
	uint8_t bb_bank;          // (bigbucks)
	uint8_t cb_bit;           // (cannonbp)
	uint8_t myst_ctrl, myst_clock, myst_pre;   // (alibaba)
	uint8_t tb[2];            // (shootbul's trackball counters)
	uint32_t rng;
	uint32_t per_cnt;         // (bigbucks' periodic IRQ: CPU cycles since the last)
	uint32_t tgt;             // (the line's end, in the Z80's cycle count)
	uint64_t psg_frac;
} b;

// ---- the drawing's copy ----
typedef struct {
	uint8_t vram[0x800];
	uint8_t spr[0x10], spr2[0x10];
	uint8_t flip, charbank, spritebank, myst_ctrl, myst_clock;
} snap_t;
static snap_t *s_snap;
static uint16_t *s_bg;            // (the characters' cache, 288 x 224)
static uint32_t *s_bg_key;        // (what each cache tile holds: code, colour, flip; ~0 = nothing)
static int16_t s_tile_idx[28][36];

static int32_t s_mix[64];

enum { PM_PA_MAIN = RBX_PA_DRV, PM_PA_SOUND, PM_PA_VBLANK, PM_PA_END };
static const char *const s_pa_names[] = { "pm main Z80", "pm sound", "pm vblank copy" };

static inline uint8_t bitswap8(uint8_t v, int b7, int b6, int b5, int b4, int b3, int b2, int b1, int b0)
{
	return (uint8_t)(((v >> b7) & 1) << 7 | ((v >> b6) & 1) << 6 | ((v >> b5) & 1) << 5 | ((v >> b4) & 1) << 4 |
	                 ((v >> b3) & 1) << 3 | ((v >> b2) & 1) << 2 | ((v >> b1) & 1) << 1 | ((v >> b0) & 1));
}

// bitswap<n>(v, bits[0] (the result's MSB) .. bits[n - 1] (its LSB))
static uint32_t bitswapn(uint32_t v, int n, const int *bits)
{
	uint32_t r = 0;
	for (int i = 0; i < n; i++)
		r = r << 1 | ((v >> bits[i]) & 1);
	return r;
}

static uint8_t port8(int port) { return (uint8_t)rbx_port_read(port); }

// ---- the banked ROM views ----
static const uint8_t *rom_at(uint32_t a) { return s_rom + a; }

static void map_rom_pages(int first, int n, const uint8_t *src)
{
	for (int p = 0; p < n; p++) {
		s_rmap[first + p] = src + p * 256;
		s_wmap[first + p] = s_page_sink;
	}
}

// Ms. Pac-Man: the pages with the decode latch's traps go through the handlers
static int msp_trap_page(int p)
{
	return p == 0x00 || p == 0x03 || p == 0x16 || p == 0x21 || p == 0x3f || p == 0x80 || p == 0x97;
}

static void set_rom_banks(void)
{
	switch (s_m->map) {
	case MAP_MSPACMAN: {
		const uint8_t *base = rom_at((uint32_t)b.bank * 0x10000);
		for (int p = 0; p < 0x100; p++) {
			if (p & 0x40)
				continue;
			s_wmap[p] = NULL;
			s_rmap[p] = msp_trap_page(p) ? NULL : base + p * 256;
		}
		break;
	}
	case MAP_EPOS: {
		const uint8_t *base = rom_at(0x10000 + (uint32_t)(b.entry & 3) * 0x4000);
		map_rom_pages(0x00, 0x40, base);
		map_rom_pages(0x80, 0x40, base);
		break;
	}
	case MAP_MSCHAMP:
		map_rom_pages(0x00, 0x40, rom_at((uint32_t)b.bank * 0x8000));
		map_rom_pages(0x80, 0x40, rom_at((uint32_t)b.bank * 0x8000 + 0x4000));
		break;
	case MAP_SUPERABC:
		map_rom_pages(0x00, 0x40, rom_at((uint32_t)b.bank * 0x10000));
		map_rom_pages(0x80, 0x20, rom_at((uint32_t)b.bank * 0x10000 + 0x4000));
		map_rom_pages(0xa0, 0x20, rom_at((uint32_t)b.bank * 0x10000 + 0xa000));
		break;
	default:
		break;
	}
}

// the 0x4000-0x4fff block at page p (its mirrors): video / colour RAM, 0x4800-0x4bff (bus, RAM or nothing), RAM
static void map_ram_block(int p, int ram48)
{
	const int o = p & 0x0f;
	if (o >= 8 && o < 12 && ram48 != 1) {
		s_rmap[p] = ram48 == 0 ? s_page_bf : s_page_00;
		s_wmap[p] = s_page_sink;
	} else {
		s_rmap[p] = b.ram + o * 256;
		s_wmap[p] = b.ram + o * 256;
	}
}

static void build_maps(void)
{
	for (int p = 0; p < 256; p++) {
		s_rmap[p] = s_page_00;
		s_wmap[p] = s_page_sink;
	}
	switch (s_m->map) {
	case MAP_PACMAN: case MAP_CRUSHS: case MAP_BIRDIY: case MAP_CANNONBP: case MAP_EPOS:
	case MAP_WOODPEK: case MAP_MSPACMAN: case MAP_ALIBABA: case MAP_DREMSHPR: case MAP_MSCHAMP: case MAP_SUPERABC: {
		// 0x4000-0x5fff mirrored by A13 and A15 (0xa000); the ROM at 0x0000-0x3fff (mirrored at 0x8000 or not)
		const int ram48 = (s_m->map == MAP_DREMSHPR || s_m->map == MAP_SUPERABC || s_m->map == MAP_CANNONBP) ? 1
		                : s_m->map == MAP_BIRDIY ? 2 : 0;
		for (int p = 0; p < 256; p++) {
			if (!(p & 0x40))
				continue;
			if (p & 0x10) {   // (the I/O block)
				s_rmap[p] = NULL;
				s_wmap[p] = NULL;
			} else
				map_ram_block(p, ram48);
		}
		const int mirror = s_m->map == MAP_PACMAN || s_m->map == MAP_CRUSHS || s_m->map == MAP_BIRDIY ||
		                   s_m->map == MAP_CANNONBP;
		if (s_m->map == MAP_EPOS || s_m->map == MAP_MSPACMAN || s_m->map == MAP_MSCHAMP || s_m->map == MAP_SUPERABC)
			set_rom_banks();
		else {
			map_rom_pages(0x00, 0x40, s_rom);
			if (mirror)
				map_rom_pages(0x80, 0x40, s_rom);
			else if (s_m->map == MAP_ALIBABA) {
				map_rom_pages(0x80, 0x10, s_rom + 0x8000);
				for (int p = 0x90; p < 0xa0; p++)   // (0x9000-0x93ff RAM, mirrored to 0x9fff)
					s_rmap[p] = s_wmap[p] = b.ram2 + (p & 3) * 256;
				for (int p = 0xa0; p < 0xc0; p++)   // (0xa000-0xa7ff ROM, mirrored to 0xbfff)
					s_rmap[p] = s_rom + 0xa000 + (p & 7) * 256, s_wmap[p] = s_page_sink;
			} else
				map_rom_pages(0x80, 0x40, s_rom + 0x8000);
		}
		// (alibaba: 0x4c00-0x4fff all RAM, the sprites at 0x4ef0; dremshpr: 0xb800-0xb87f writes go nowhere, as the ROM's)
		if (s_m->map == MAP_CANNONBP)   // (0x3000-0x3fff: the protection's reads)
			for (int p = 0x30; p < 0x40; p++)
				s_rmap[p] = NULL;
		break;
	}
	case MAP_MSPACTWIN:
		map_rom_pages(0x00, 0x40, s_rom);
		map_rom_pages(0x60, 0x20, s_rom + 0x2000);   // (0x2000-0x3fff mirrored at 0x6000)
		map_rom_pages(0x80, 0x40, s_rom + 0x8000);
		for (int p = 0x40; p < 0x100; p++) {
			if (p >= 0x60 && p < 0x68)
				continue;   // (the ROM's mirror)
			if (p >= 0x80 && p < 0xc0)
				continue;
			if (p & 0x10) {
				s_rmap[p] = NULL;
				s_wmap[p] = NULL;
				continue;
			}
			const int o = p & 0x0f;
			if (o < 8 && (p & 0x20))   // (the video / colour RAM is not mirrored at 0x6000 / 0xe000)
				continue;
			map_ram_block(p, 0);
		}
		break;
	case MAP_NUMCRASH:
		map_rom_pages(0x00, 0x28, s_rom);
		map_rom_pages(0x30, 0x08, s_rom + 0x3000);
		map_rom_pages(0x80, 0x20, s_rom + 0x8000);
		for (int p = 0x40; p < 0x48; p++)
			s_rmap[p] = s_wmap[p] = b.ram + (p & 0x0f) * 256;
		for (int p = 0x4c; p < 0x50; p++)
			s_rmap[p] = s_wmap[p] = b.ram + (p & 0x0f) * 256;
		s_rmap[0x50] = NULL;
		s_wmap[0x50] = NULL;
		break;
	case MAP_ROCKTRV2: case MAP_BIGBUCKS:
		map_rom_pages(0x00, 0x40, s_rom);
		for (int p = 0x40; p < 0x48; p++)
			s_rmap[p] = s_wmap[p] = b.ram + (p & 0x0f) * 256;
		for (int p = 0x4c; p < 0x50; p++)
			s_rmap[p] = s_wmap[p] = b.ram + (p & 0x0f) * 256;
		s_rmap[0x50] = s_wmap[0x50] = NULL;
		if (s_m->map == MAP_ROCKTRV2) {
			s_rmap[0x5f] = s_wmap[0x5f] = NULL;
			map_rom_pages(0x60, 0x20, s_rom + 0x6000);
			for (int p = 0x80; p < 0x100; p++)
				s_rmap[p] = NULL;   // (the question ROMs)
		} else {
			s_rmap[0x51] = s_wmap[0x51] = NULL;
			s_wmap[0x60] = NULL;
			map_rom_pages(0x80, 0x20, s_rom + 0x8000);
		}
		break;
	case MAP_PENGOJPM:
		map_rom_pages(0x00, 0x10, s_rom);
		map_rom_pages(0x40, 0x40, s_rom + 0x4000);
		for (int p = 0x80; p < 0x90; p++)
			s_rmap[p] = s_wmap[p] = b.ram + (p & 0x0f) * 256;
		s_rmap[0x90] = s_wmap[0x90] = NULL;
		for (int p = 0xf0; p < 0x100; p++)
			s_rmap[p] = s_wmap[p] = b.ram2 + (p & 0x0f) * 256;
		break;
	}
}

// ---- the LS259 latch ----
static void sound_enable(int on)
{
	b.sound_on = (uint8_t)on;
	if (s_m->snd == SND_WSG)
		namco_wsg_sound_enable_w(&s_wsg, on);
}

static void irq_mask_w(int state)
{
	b.irq_mask = (uint8_t)state;
	if (!state)
		b.irq_line = 0;
}

static void nmi_mask_w(int state)
{
	b.irq_mask = (uint8_t)state;
	if (!state)
		b.nmi_line = 0;
}

static void latch_w(int bit, int d)
{
	if (((b.latch >> bit) & 1) == d)
		return;
	b.latch = (uint8_t)(b.latch ^ (1 << bit));
	switch (s_m->lat) {
	case LAT_STD:
		if (bit == 0)
			irq_mask_w(d);
		else if (bit == 1)
			sound_enable(d);
		else if (bit == 3 && !(s_m->flags & F_NOFLIP))
			b.flip = (uint8_t)d;
		break;
	case LAT_BIRDIY:
		if (bit == 1)
			irq_mask_w(d);
		else if (bit == 3)
			b.flip = (uint8_t)d;
		break;
	case LAT_NMI:
		if (bit == 0)
			nmi_mask_w(d);
		else if (bit == 3)
			b.flip = (uint8_t)d;
		break;
	default:   // (alibaba latch1: LEDs, coin lockout / counter)
		break;
	}
}

static void latch2_w(int bit, int d)   // (alibaba latch2)
{
	if (((b.latch2 >> bit) & 1) == d)
		return;
	b.latch2 = (uint8_t)(b.latch2 ^ (1 << bit));
	if (bit == 0)
		sound_enable(d);
	else if (bit == 1)
		b.flip = (uint8_t)d;
	else if (bit == 2)
		irq_mask_w(d);
}

// ---- inputs ----
static uint8_t in_port(int n)   // IN0 / IN1 with the multiplexed joysticks and the trackball
{
	uint8_t v = port8(n ? s_port_in1 : s_port_in0);
	if (s_m->flags & F_SUBLATCH) {   // clubpacm_input_r (the custom bits active low)
		uint8_t d = 0x0f;
		if (!((b.latch >> 5) & 1))
			d &= port8(s_port_p1);
		if (!((b.latch >> 4) & 1))
			d &= port8(s_port_p2);
		v = (uint8_t)((v & 0xf0) | (d & 0x0f));
	}
	if (s_shootbul)
		v = (uint8_t)((v & 0xf0) | (b.tb[n] & 0x0f));
	return v;
}

static uint8_t std_io_read(uint16_t a)
{
	if (a >= 0x5080 && a < 0x5100) {   // (the protections installed on these exact addresses)
		if (s_init == INIT_MAKETRAX) {
			static const uint8_t prot2[0x1e] = {   // table at $ebd (odd entries)
				0x00, 0xc0, 0x00, 0x40, 0xc0, 0x40, 0x00, 0xc0, 0x00, 0x40, 0x00, 0xc0, 0x00, 0x40, 0xc0, 0x40,
				0x00, 0xc0, 0x00, 0x40, 0x00, 0xc0, 0x00, 0x40, 0xc0, 0x40, 0x00, 0xc0, 0x00, 0x40 };
			static const uint8_t prot3[0x1e] = {   // table at $ebd (even entries)
				0x1f, 0x3f, 0x2f, 0x2f, 0x0f, 0x0f, 0x0f, 0x3f, 0x0f, 0x0f, 0x1c, 0x3c, 0x2c, 0x2c, 0x0c, 0x0c,
				0x0c, 0x3c, 0x0c, 0x0c, 0x11, 0x31, 0x21, 0x21, 0x01, 0x01, 0x01, 0x31, 0x01, 0x01 };
			if (a < 0x50c0) {   // maketrax_special_port2_r
				const int off = a - 0x5080;
				uint8_t d = (uint8_t)(port8(s_port_dsw1) & 0x3f);
				if (!b.mt_disable)
					return (uint8_t)(prot2[b.mt_offset] | d);
				if (off == 0x01 || off == 0x04)
					d |= 0x40;
				else if (off == 0x05 || off == 0x0e || off == 0x10)
					d |= 0xc0;
				return d;
			}
			const int off = a - 0x50c0;   // maketrax_special_port3_r
			if (!b.mt_disable)
				return prot3[b.mt_offset];
			return off == 0x00 ? 0x1f : off == 0x09 ? 0x30 : off == 0x0c ? 0x00 : 0x20;
		}
		if (s_init == INIT_MBRUSH) {   // mbrush_prot_r
			const int off = a - 0x5080;
			const uint8_t d = (uint8_t)(port8(s_port_dsw1) & 0x3f);
			switch (off) {
			case 0x00: case 0x04: case 0x07: case 0x0e: case 0x0f: return (uint8_t)(0xc0 | d);
			case 0x40: case 0x41: case 0x47: case 0x49: case 0x4d: case 0x5d: return 0x00;
			case 0x42: case 0x43: case 0x46: case 0x4c: case 0x50: return 0x02;
			case 0x7f: return 0x10;
			default: return d;
			}
		}
	}
	if (s_init == INIT_MSPACII && a >= 0x504d && a <= 0x506f)   // mspacii_protection_r
		return (uint8_t)((in_port(1) & 0xef) | (((a - 0x504d) << 4) & 0x10));
	switch ((a >> 6) & 3) {
	case 0: return in_port(0);
	case 1: return in_port(1);
	case 2: return port8(s_port_dsw1);
	default: return (s_m->flags & F_SUBLATCH) ? b.sublatch : port8(s_port_dsw2);
	}
}

static void sound_w(int offset, uint8_t v)
{
	if (s_m->snd == SND_WSG)
		namco_wsg_pacman_sound_w(&s_wsg, offset, v);
}

static void std_io_write(uint16_t a, uint8_t v)
{
	const int o = a & 0xff;
	if (o < 0x40) {
		if (s_init == INIT_MAKETRAX && a == 0x5004) {   // maketrax_protection_w
			if (v == 0) {   // disable protection / reset?
				b.mt_counter = b.mt_offset = 0;
				b.mt_disable = 1;
			} else if (v == 1) {
				b.mt_disable = 0;
				if (++b.mt_counter == 0x3c) {
					b.mt_counter = 0;
					if (++b.mt_offset == 0x1e)
						b.mt_offset = 0;
				}
			}
			return;
		}
		if (s_m->map == MAP_SUPERABC && (o & 3) == 2) {   // superabc_bank_w (0x5002 mirror 0xaf3c)
			const int bank = (v >> 4) & 7;
			b.bank = (uint8_t)bank;
			set_rom_banks();
			b.spritebank = b.charbank = (uint8_t)bank;
			return;
		}
		if (s_m->map == MAP_MSPACTWIN && (a & 0xffc8) != 0x5000)   // (the latch mirrored by 0x0030 only)
			return;
		latch_w(o & 7, v & 1);
	} else if (o < 0x60) {
		if (s_m->map == MAP_CRUSHS || s_m->map == MAP_DREMSHPR)
			return;
		if (s_m->map == MAP_BIRDIY) {   // (0x5040-0x505f: nothing; the sound at 0x5080, the coordinates at 0x50a0)
			return;
		}
		sound_w(o & 0x1f, v);
	} else if (o < 0x70) {
		if (s_m->map != MAP_BIRDIY)
			b.spr2[o & 0x0f] = v;
	} else if (o >= 0x80 && o < 0xc0) {
		if (s_m->map == MAP_BIRDIY) {
			if (o < 0xa0)
				sound_w(o & 0x1f, v);
			else if (o < 0xb0)
				b.spr2[o & 0x0f] = v;
		} else if ((s_m->flags & F_SUBLATCH) && o < 0x80 + 0x40)
			b.sublatch = v;
	}
	// (0x50c0: the watchdog)
}

// ---- the main CPU ----
static void msp_decode(int on)   // mspacman_enable / disable_decode
{
	if (b.bank != on) {
		b.bank = (uint8_t)on;
		set_rom_banks();
	}
}

static uint8_t msp_rom_read(uint16_t a)
{
	const uint16_t t = a & 0xfff8;
	if (t == 0x0038 || t == 0x03b0 || t == 0x1600 || t == 0x2120 || t == 0x3ff0 || t == 0x8000 || t == 0x97f0) {
		msp_decode(0);
		return s_rom[a];
	}
	if (t == 0x3ff8) {
		msp_decode(1);
		return s_rom[0x10000 + a];
	}
	return s_rom[(uint32_t)b.bank * 0x10000 + a];
}

static void msp_rom_write(uint16_t a)
{
	const uint16_t t = a & 0xfff8;
	if (t == 0x0038 || t == 0x03b0 || t == 0x1600 || t == 0x2120 || t == 0x3ff0 || t == 0x8000 || t == 0x97f0)
		msp_decode(0);
	else if (t == 0x3ff8)
		msp_decode(1);
}

static uint8_t alibaba_io_read(uint16_t a)
{
	const int o = a & 0xff;
	if (o >= 0xc0) {
		if (o == 0xc0) {   // mystery_1_r: the mystery item
			b.rng = b.rng * 1103515245u + 12345u;
			return (uint8_t)((b.rng >> 16) & 0x0f);
		}
		if (o == 0xc1)   // mystery_2_r: when the mystery is lit up
			return b.myst_clock >= 24 ? 1 : 0;
		return 0xbf;
	}
	return std_io_read(a);
}

static void alibaba_io_write(uint16_t a, uint8_t v)
{
	const int o = a & 0xff;
	if (o < 0x40)
		latch_w(o & 7, v & 1);   // (latch1; the watchdog at 0x5000)
	else if (o < 0x70) {
		if (o >= 0x50 && o < 0x60)
			b.spr2[o & 0x0f] = v;
		else {   // (the sound region is not contiguous: the offset into 0x00-0x1f)
			const int off = o - 0x40;
			sound_w(((off >> 1) & 0x10) | (off & 0x0f), v);
		}
	} else if (o >= 0x80 && o < 0xc0) {   // mystery_w
		if ((b.myst_ctrl ^ v) & 1)   // d0: start/stop clock
			b.myst_pre = b.myst_clock = 0;
		b.myst_ctrl = v;   // d1: show clock
	} else if (o >= 0xc0 && o < 0xc8)
		latch2_w(o & 7, v & 1);
}

static uint8_t cannonbp_prot_r(uint16_t a)
{
	// (MAME's simulation of the epoxy chip at 6P: enough to play the game)
	switch (a - 0x3000) {
	case 0x0004:
		b.cb_bit = 7;
		return 0x00;
	case 0x0001:   // affects the ball hitting the blocks as well as jump address after bonus round
		if (s_main.pc == 0x2b97)
			return (uint8_t)(((0x46 >> (b.cb_bit-- & 7)) & 1) << 7);
		return 0xff;
	case 0x0107: return 0x40;   // player start y position -> register H
	default: return 0x00;
	}
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	switch (s_m->map) {
	case MAP_MSPACMAN:
		if (!(a & 0x4000))
			return msp_rom_read(a);
		break;
	case MAP_ALIBABA:
		return alibaba_io_read(a);
	case MAP_CANNONBP:
		if (a >= 0x3000 && a < 0x4000)
			return cannonbp_prot_r(a);
		break;
	case MAP_NUMCRASH:
		if (a == 0x5000) return in_port(0);
		if (a == 0x5040) return in_port(1);
		if (a == 0x5080) return port8(s_port_dsw1);
		return 0;
	case MAP_CRUSHS:
		switch ((a >> 6) & 3) {
		case 0: return in_port(0);
		case 2: return port8(s_port_in1);
		default: return 0;
		}
	case MAP_ROCKTRV2:
		if (a >= 0x8000)   // rocktrv2_question_r
			return s_user1 ? s_user1[rbx_wrap((uint32_t)(a & 0x7fff) | (uint32_t)b.rt_qbank * 0x8000, s_user1_size)] : 0;
		if (a >= 0x5f00) {
			if (a == 0x5fe0 || a == 0x5fe4 || a == 0x5fe8 || a == 0x5fec)
				return (uint8_t)(b.rt_prot[(a >> 2) & 3] >> 4);
			if (a == 0x5fff)
				return port8(s_port_dsw2);
			return 0;
		}
		if (a == 0x5000) return in_port(0);
		if (a >= 0x5040 && a < 0x5080) return in_port(1);
		if (a == 0x5080) return port8(s_port_dsw1);
		if (a == 0x50c0) return port8(s_port_dsw2);
		return 0;
	case MAP_BIGBUCKS:
		if (a >= 0x5000 && a < 0x5100) {
			switch ((a >> 6) & 3) {
			case 0: return in_port(0);
			case 1: return in_port(1);
			case 2: return port8(s_port_dsw1);
			default: return port8(s_port_dsw2);
			}
		}
		return 0;
	case MAP_PENGOJPM:
		switch (a) {
		case 0x9000: return in_port(0);
		case 0x9040: return in_port(1);
		case 0x9080: return port8(s_port_dsw1);
		case 0x90c0: return port8(s_port_dsw2);
		default: return 0;
		}
	case MAP_MSCHAMP:
		if (((a >> 6) & 3) == 3)   // (nopr)
			return 0;
		break;
	default:
		break;
	}
	return std_io_read(a);
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	uint8_t *p = s_wmap[a >> 8];
	if (p) {
		p[a & 0xff] = v;
		return;
	}
	switch (s_m->map) {
	case MAP_MSPACMAN:
		if (!(a & 0x4000)) {
			msp_rom_write(a);
			return;
		}
		break;
	case MAP_ALIBABA:
		alibaba_io_write(a, v);
		return;
	case MAP_NUMCRASH:
		if (a < 0x5008)
			latch_w(a & 7, v & 1);
		else if (a >= 0x5040 && a < 0x5060)
			sound_w(a & 0x1f, v);
		else if (a < 0x5070)
			b.spr2[a & 0x0f] = v;
		return;
	case MAP_ROCKTRV2: case MAP_BIGBUCKS:
		if (a < 0x5008)
			latch_w(a & 7, v & 1);
		else if (a >= 0x5040 && a < 0x5060)
			sound_w(a & 0x1f, v);
		else if (s_m->map == MAP_ROCKTRV2 && a >= 0x5fe0 && a < 0x5fe4)
			b.rt_prot[a & 3] = v;
		else if (s_m->map == MAP_ROCKTRV2 && a == 0x5ff0)
			b.rt_qbank = v;
		else if (s_m->map == MAP_BIGBUCKS && a == 0x6000)
			b.bb_bank = v;
		return;
	case MAP_PENGOJPM:
		if (a < 0x9008)
			latch_w(a & 7, v & 1);
		else if (a >= 0x9040 && a < 0x9060)
			sound_w(a & 0x1f, v);
		else if (a >= 0x9060 && a < 0x9070)
			b.spr2[a & 0x0f] = v;
		return;
	default:
		break;
	}
	std_io_write(a, v);
}

static uint8_t main_in(z80 *z, uint8_t port)
{
	switch (s_m->io) {
	case IO_EPOS: {   // epos_decryption_w: the counter up on an even port, down on an odd one
		uint8_t c = b.bank;
		c = (uint8_t)((port & 1) ? (c - 1) & 0x0f : (c + 1) & 0x0f);
		b.bank = c;
		if (c >= 0x08 && c <= 0x0b) {
			b.entry = c & 3;
			set_rom_banks();
		}
		return 0;
	}
	case IO_MSCHAMP:   // mux_r
		if (port == 0)
			return b.mux ? b.mux_data : (uint8_t)(port8(s_port_dsw2) & 0x07);
		return 0;
	case IO_BIGBUCKS: {   // bigbucks_question_r (the whole 16-bit port address)
		if (!s_user1)
			return 0;
		const uint32_t o = ((uint32_t)b.bb_bank << 16) | (uint32_t)(z->mem_ptr ^ 0xffff);
		return o < s_user1_size ? s_user1[o] : 0;
	}
	case IO_CRUSHS:
		if (port == 1)
			return ay8910_data_r(&s_ay);
		if (port == 2)
			return port8(s_port_dsw1);
		return 0;
	default:
		return 0;
	}
}

static void set_vector(uint8_t v)
{
	b.vector = v;
	s_main.int_data = v;
}

static void main_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	switch (s_m->io) {
	case IO_VECTOR: case IO_EPOS:
		if (port == 0)
			set_vector(v);
		break;
	case IO_MSCHAMP:
		if (port == 0)
			set_vector(v);
		else if (port == 0x10 || port == 0x11) {   // mux_w
			b.mux = port & 1;
			b.mux_data = v;
		}
		break;
	case IO_PIRANHA:
		if (port == 0)
			set_vector(v == 0xfa ? 0x78 : v);
		break;
	case IO_NMOUSE:
		if (port == 0)
			set_vector(v == 0xbf ? 0x3c : v == 0xc6 ? 0x40 : v);
		break;
	case IO_MSPACII:   // (any port)
		set_vector(v == 0xfb ? 0xfe : v);
		break;
	case IO_DREMSHPR:   // data_address_w: 6 data, 7 address
		if (port == 6)
			ay8910_data_w(&s_ay, v);
		else if (port == 7)
			ay8910_address_w(&s_ay, v);
		break;
	case IO_VANVAN:
		if (port == 1)
			sn76496_write(&s_sn[0], v);
		else if (port == 2)
			sn76496_write(&s_sn[1], v);
		break;
	case IO_CRUSHS:   // address_data_w: 0 address, 1 data
		if (port == 0)
			ay8910_address_w(&s_ay, v);
		else if (port == 1)
			ay8910_data_w(&s_ay, v);
		break;
	default:
		break;
	}
}

static uint8_t crushs_porta(void *u) { (void)u; return port8(s_port_dsw2); }

// ---- drawing ----
static void build_tile_idx(void)
{
	for (int r = 0; r < 28; r++)
		for (int c = 0; c < 36; c++) {   // pacman_scan_rows
			const int row = r + 2, col = c - 2;
			s_tile_idx[r][c] = (int16_t)((col & 0x20) ? row + ((col & 0x1f) << 5) : col + (row << 5));
		}
}

static void bg_invalidate(void)
{
	if (s_bg_key)
		memset(s_bg_key, 0xff, 36 * 28 * sizeof(s_bg_key[0]));
}

// the characters' cache: the tiles whose code, colour or flip changed redrawn
static void draw_bg(const snap_t *sn)
{
	const int flip = sn->flip;
	const uint32_t bank = (uint32_t)sn->charbank << 8;
	for (int r = 0; r < 28; r++)
		for (int c = 0; c < 36; c++) {
			const int idx = s_tile_idx[r][c];
			const uint32_t code = ((uint32_t)sn->vram[idx] | bank) & s_chr_mask;
			const uint32_t color = sn->vram[0x400 + idx] & 0x1f;
			const uint32_t key = code | color << 16 | (uint32_t)flip << 24;
			uint32_t *k = &s_bg_key[r * 36 + c];
			if (*k == key)
				continue;
			*k = key;
			const uint8_t *src = s_chr + code * 64;
			const uint16_t base = (uint16_t)(color * 4);
			if (!flip) {
				uint16_t *d = s_bg + (r * 8) * SCR_W + c * 8;
				for (int y = 0; y < 8; y++, d += SCR_W, src += 8)
					for (int x = 0; x < 8; x++)
						d[x] = (uint16_t)(base + src[x]);
			} else {   // (the tilemap flipped in x and y: the tile's place and pixels mirrored)
				uint16_t *d = s_bg + ((27 - r) * 8 + 7) * SCR_W + (35 - c) * 8;
				for (int y = 0; y < 8; y++, d -= SCR_W, src += 8)
					for (int x = 0; x < 8; x++)
						d[7 - x] = (uint16_t)(base + src[x]);
			}
		}
}

// drawgfx transmask of a 16x16 sprite, clipped to [x0, x1) x [0, VIS_H) in screen pixels (the output starts at s_xoff)
static void blit_sprite(uint16_t *bm, int pitch, uint32_t code, int color, int fx, int fy, int sx, int sy, int x0, int x1)
{
	const uint8_t *src = s_spr + (code & s_spr_mask) * 256;
	const uint32_t trans = s_spr_trans[color];
	const uint16_t base = (uint16_t)(color * 4);
	const int i0 = sx < x0 ? x0 - sx : 0, i1 = sx + 16 > x1 ? x1 - sx : 16;
	const int j0 = sy < 0 ? -sy : 0, j1 = sy + 16 > VIS_H ? VIS_H - sy : 16;
	if (i0 >= i1 || j0 >= j1)
		return;
	for (int j = j0; j < j1; j++) {
		const uint8_t *s = src + (fy ? 15 - j : j) * 16;
		uint16_t *d = bm + (sy + j) * pitch + sx - s_xoff;
		if (fx) {
			for (int i = i0; i < i1; i++) {
				const uint8_t p = s[15 - i];
				if (!((trans >> p) & 1))
					d[i] = (uint16_t)(base + p);
			}
		} else {
			for (int i = i0; i < i1; i++) {
				const uint8_t p = s[i];
				if (!((trans >> p) & 1))
					d[i] = (uint16_t)(base + p);
			}
		}
	}
}

static void draw_one(const snap_t *sn, uint16_t *bm, int pitch, int offs, int yoff, int x0, int x1)
{
	const int inv = (s_m->flags & F_INV_SPR) ? 1 : 0;
	int sx, sy;
	if (inv) {
		sx = sn->spr2[offs + 1];
		sy = 240 - sn->spr2[offs];
	} else {
		sx = 272 - sn->spr2[offs + 1];
		sy = sn->spr2[offs] - 31;
	}
	const int fx = (sn->spr[offs] & 1) ^ inv, fy = ((sn->spr[offs] & 2) >> 1) ^ inv;
	const int color = sn->spr[offs + 1] & 0x1f;
	const uint32_t code = (uint32_t)((sn->spr[offs] >> 2) | (sn->spritebank << 6));
	blit_sprite(bm, pitch, code, color, fx, fy, sx, sy + yoff, x0, x1);
	blit_sprite(bm, pitch, code, color, fx, fy, sx - 256, sy + yoff, x0, x1);   // (the wraparound: Crush Roller's tunnel)
}

// the clock (alibaba draw_clock): a 24x16 element, colour 1, pen 0 transparent
static void blit_clock(uint16_t *bm, int pitch, int code, int fx, int fy, int sx, int sy)
{
	const uint8_t *src = s_clock + (uint32_t)(code & 31) * (24 * 16);
	for (int j = 0; j < 16; j++) {
		const int y = sy + j;
		if (y < 0 || y >= VIS_H)
			continue;
		const uint8_t *s = src + (fy ? 15 - j : j) * 24;
		for (int i = 0; i < 24; i++) {
			const int x = sx + i - s_xoff;
			if (x < 0 || x >= s_vis_w)
				continue;
			const uint8_t p = s[fx ? 23 - i : i];
			if (p)
				bm[y * pitch + x] = (uint16_t)(4 + p);
		}
	}
}

static void pm_render(void)
{
	const snap_t *sn = s_snap;
	memcpy(rbx_video.palette, s_pal, sizeof(s_pal));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	draw_bg(sn);
	for (int y = 0; y < VIS_H; y++)
		memcpy(bm + y * pitch, s_bg + y * SCR_W + s_xoff, (size_t)s_vis_w * sizeof(uint16_t));
	if (!(s_m->flags & F_NOSPR)) {
		int x0 = 2 * 8, x1 = 34 * 8;
		if (s_m->lat == LAT_ALIBABA) {   // (sprites clipped to a larger area than Pac-Man's)
			if (sn->flip)
				x0 = 6 * 8;
			else
				x1 = 30 * 8;
		}
		const int yoff = (s_m->flags & F_INV_SPR) ? 0 : 1;   // (m_xoffsethack: the first three one pixel off)
		for (int offs = 0x10 - 2; offs > 2 * 2; offs -= 2)
			draw_one(sn, bm, pitch, offs, 0, x0, x1);
		for (int offs = 2 * 2; offs >= 0; offs -= 2)
			draw_one(sn, bm, pitch, offs, yoff, x0, x1);
	}
	if (s_m->lat == LAT_ALIBABA && (sn->myst_ctrl & 2) && s_clock) {
		if (sn->myst_clock <= 16) {   // inactive half
			int x = 120, y = 112;
			if (sn->flip) {
				x = 264 - x;
				y = 208 - y;
			}
			blit_clock(bm, pitch, 0x1f, 0, 0, x, y);
		}
		int x = 120, y = 96 + (sn->myst_clock & 0x10), fx = 0, fy = 0;   // active half
		if (sn->flip) {
			x = 264 - x;
			y = 208 - y;
			fx = fy = 1;
		}
		blit_clock(bm, pitch, sn->myst_clock ^ 0x1f, fx, fy, x, y);
	}
}

static void vblank_capture(void)
{
	const unsigned pa = rbx_prof_enter(PM_PA_VBLANK);
	rbx_video_wait_idle();
	snap_t *sn = s_snap;
	memcpy(sn->vram, b.ram, sizeof(sn->vram));
	memcpy(sn->spr, b.ram + (s_m->lat == LAT_ALIBABA ? 0xef0 : 0xff0), sizeof(sn->spr));
	memcpy(sn->spr2, b.spr2, sizeof(sn->spr2));
	sn->flip = b.flip;
	sn->charbank = b.charbank;
	sn->spritebank = b.spritebank;
	sn->myst_ctrl = b.myst_ctrl;
	sn->myst_clock = b.myst_clock;
	rbx_video_frame_done();
	rbx_prof_leave(pa);
}

// ---- frame ----
static void sound_line(void)
{
	int32_t *mix = s_mix;
	int n = 0;
	switch (s_m->snd) {
	case SND_WSG:
		memset(mix, 0, 3 * sizeof(mix[0]));
		namco_wsg_render(&s_wsg, mix, NULL, 3, WSG_RATE);
		n = 3;
		break;
	case SND_AY: case SND_AY8912: {
		const int clk = rbx_line_cycles(&b.psg_frac, PSG_CLOCK, FRAME_MHZ, LINES);
		const int max = ay8910_out_max(&s_ay, (uint32_t)clk);
		memset(mix, 0, (size_t)max * sizeof(mix[0]));
		n = ay8910_render(&s_ay, (uint32_t)clk, mix, max);
		break;
	}
	case SND_SN2: {
		const int clk = rbx_line_cycles(&b.psg_frac, PSG_CLOCK, FRAME_MHZ, LINES);
		memset(mix, 0, 16 * sizeof(mix[0]));
		n = sn76496_render(&s_sn[0], (uint32_t)clk, mix, 16);
		sn76496_render(&s_sn[1], (uint32_t)clk, mix, 16);
		break;
	}
	}
	for (int i = 0; i < n; i++)
		rbx_audio_put(mix[i]);
}

static void read_controls(void)
{
	if (s_shootbul) {   // (IPT_TRACKBALL_X / _Y, 4 bits; Y reversed; the d-pad without a trackball)
		b.tb[0] = (uint8_t)(b.tb[0] + rbx_analog_dpad(1, RBX_IPT_TRACKBALL_X, 3));
		b.tb[1] = (uint8_t)(b.tb[1] - rbx_analog_dpad(1, RBX_IPT_TRACKBALL_Y, 3));
	}
}

static void pm_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	read_controls();
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_H) {   // vblank: the frame, the IRQ / NMI, Ali Baba's mystery clock
			vblank_capture();
			if (b.irq_mask) {
				if (s_m->lat == LAT_NMI) {
					if (!b.nmi_line)
						z80_gen_nmi(&s_main);
					b.nmi_line = 1;
				} else
					b.irq_line = 1;
			}
			if (s_m->lat == LAT_ALIBABA) {   // mystery_tick: the clock every 64 frames
				b.myst_pre = (uint8_t)((b.myst_pre + 1) & 0x3f);
				if (b.myst_pre == 0 && (b.myst_ctrl & 1))
					b.myst_clock = (uint8_t)((b.myst_clock + 1) & 0x1f);
			}
		}
		if (s_m->flags & F_PERIODIC) {   // (20 x 60 Hz)
			b.per_cnt += LINE_CYCLES;
			if (b.per_cnt >= CPU_CLOCK / 1200) {
				b.per_cnt -= CPU_CLOCK / 1200;
				if (b.irq_mask)
					b.irq_line = 1;
			}
		}
		rbx_prof_set(PM_PA_MAIN);
		b.tgt += LINE_CYCLES;
		z80_run(&s_main, (unsigned long)b.tgt, &b.irq_line, 0);
		rbx_prof_set(PM_PA_SOUND);
		sound_line();
	}
	rbx_prof_set(RBX_PA_DRIVER);
}

// ---- init / reset ----
static void sound_init(void)
{
	b.sound_on = 1;
	switch (s_m->snd) {
	case SND_WSG:
		namco_wsg_init(&s_wsg, NAMCO_WSG_TYPE_WSG, CPU_CLOCK / 32, 0, s_wave, 256);
		break;
	case SND_AY:   // (0.50)
		ay8910_init(&s_ay, AY8910_TYPE_AY8910, PSG_CLOCK, AY8910_LEGACY_OUTPUT, 128, 4, 0);
		break;
	case SND_AY8912:   // (0.75; port A: DSW2)
		ay8910_init(&s_ay, AY8910_TYPE_AY8910, PSG_CLOCK, AY8910_LEGACY_OUTPUT, 192, 4, 0);
		s_ay.port_a_read = crushs_porta;
		break;
	case SND_SN2:   // (0.75 each)
		for (int i = 0; i < 2; i++)
			sn76496_init(&s_sn[i], PSG_CLOCK, SN76496_CONFIG, 192, 16, 0);
		break;
	}
}

static void pm_reset(void)
{
	memset(b.ram, 0, sizeof(b.ram));
	memset(b.ram2, 0, sizeof(b.ram2));
	memset(b.spr2, 0, sizeof(b.spr2));
	b.latch = b.latch2 = 0;
	b.irq_mask = b.irq_line = b.nmi_line = 0;
	b.flip = b.charbank = b.spritebank = 0;
	b.sublatch = 0;
	b.mt_counter = b.mt_offset = b.mt_disable = 0;
	b.mux = 1;
	b.mux_data = 0xff;
	memset(b.rt_prot, 0, sizeof(b.rt_prot));
	b.rt_qbank = b.bb_bank = 0;
	b.cb_bit = 0;
	b.myst_ctrl = b.myst_clock = b.myst_pre = 0;
	b.tb[0] = b.tb[1] = 0;
	b.rng = 0x12345678;
	b.per_cnt = 0;
	b.tgt = 0;
	b.psg_frac = 0;
	switch (s_m->map) {
	case MAP_MSPACMAN: b.bank = 1; break;   // (init_mspacman: set_entry(1))
	case MAP_EPOS: b.bank = s_m->epos == EPOS_GLOB ? 0x0a : s_m->epos == EPOS_ACITYA ? 0x0b : 0x09; break;
	case MAP_MSCHAMP: b.bank = (uint8_t)((port8(s_port_dsw2) >> 3) & 1); break;
	default: b.bank = 0; break;
	}
	b.entry = b.bank & 3;
	build_maps();
	z80_init(&s_main);
	s_main.read_byte = main_read;
	s_main.write_byte = main_write;
	s_main.port_in = main_in;
	s_main.port_out = main_out;
	s_main.read_map = s_rmap;
	s_main.op_rom = s_opcodes;
	b.vector = 0xff;   // (the machines without the IRQ acknowledge callback: the Z80's default 0xff)
	if (s_m->io == IO_VECTOR || s_m->io == IO_EPOS || s_m->io == IO_MSCHAMP || s_m->io == IO_PIRANHA ||
	    s_m->io == IO_NMOUSE || s_m->io == IO_MSPACII)
		b.vector = 0;   // (VIDEO_START: m_interrupt_vector = 0)
	s_main.int_data = b.vector;
	sound_init();
	bg_invalidate();
}

static void pm_exit(void);

static int find_machine(const char *machine, int *init)
{
	char name[32];
	const char *colon = strchr(machine, ':');
	size_t n = colon ? (size_t)(colon - machine) : strlen(machine);
	if (n >= sizeof(name))
		return -1;
	memcpy(name, machine, n);
	name[n] = 0;
	if (init) {
		*init = INIT_NONE;
		if (colon)
			for (int i = 1; i < (int)(sizeof(s_init_names) / sizeof(s_init_names[0])); i++)
				if (!strcmp(colon + 1, s_init_names[i]))
					*init = i;
	}
	for (int i = 0; i < (int)(sizeof(s_machines) / sizeof(s_machines[0])); i++)
		if (!strcmp(name, s_machines[i].name))
			return i;
	return -1;
}

// one element of a MAME gfx_layout (2 planes) from rom (bits past the end read as 0)
static void decode_elem(uint8_t *dst, const uint8_t *rom, uint32_t size, uint32_t base_bits, int w, int h,
                        const uint32_t *planes, const uint32_t *xo, const uint32_t *yo)
{
	for (int y = 0; y < h; y++)
		for (int x = 0; x < w; x++) {
			int p = 0;
			for (int pl = 0; pl < 2; pl++) {
				const uint32_t o = base_bits + planes[pl] + yo[y] + xo[x];
				const int bit = (o >> 3) < size ? (rom[o >> 3] >> (7 - (o & 7))) & 1 : 0;
				p = p << 1 | bit;
			}
			dst[y * w + x] = (uint8_t)p;
		}
}

static void eyes_decode(uint8_t *data)
{
	uint8_t sw[8];
	for (int j = 0; j < 8; j++)
		sw[j] = data[(j & ~5) | ((j & 1) << 2) | ((j >> 2) & 1)];   // (address lines A0 and A2 swapped)
	for (int j = 0; j < 8; j++)
		data[j] = bitswap8(sw[j], 7, 4, 5, 6, 3, 2, 1, 0);   // (data lines D4 and D6 swapped)
}

static uint8_t pacplus_decrypt(int addr, uint8_t e)
{
	static const uint8_t swap_xor_table[6][9] = {
		{ 7,6,5,4,3,2,1,0, 0x00 }, { 7,6,5,4,3,2,1,0, 0x28 }, { 6,1,3,2,5,7,0,4, 0x96 },
		{ 6,1,5,2,3,7,0,4, 0xbe }, { 0,3,7,6,4,2,1,5, 0xd5 }, { 0,3,4,6,7,2,1,5, 0xdd } };
	static const int picktable[32] = { 0,2,4,2,4,0,4,2,2,0,2,2,4,0,4,2, 2,2,4,0,4,2,4,0,0,4,0,4,4,2,4,2 };
	// pick method from bits 0 2 5 7 9 of the address; switch method if bit 11 of the address is set
	int method = picktable[(addr & 0x001) | ((addr & 0x004) >> 1) | ((addr & 0x020) >> 3) | ((addr & 0x080) >> 4) |
	                       ((addr & 0x200) >> 5)];
	if (addr & 0x800)
		method ^= 1;
	const uint8_t *t = swap_xor_table[method];
	return (uint8_t)(bitswap8(e, t[0], t[1], t[2], t[3], t[4], t[5], t[6], t[7]) ^ t[8]);
}

static uint8_t jumpshot_decrypt(int addr, uint8_t e)
{
	static const uint8_t swap_xor_table[6][9] = {
		{ 7,6,5,4,3,2,1,0, 0x00 }, { 7,6,3,4,5,2,1,0, 0x20 }, { 5,0,4,3,7,1,2,6, 0xa4 },
		{ 5,0,4,3,7,1,2,6, 0x8c }, { 2,3,1,7,4,6,0,5, 0x6e }, { 2,3,4,7,1,6,0,5, 0x4e } };
	static const int picktable[32] = { 0,2,4,4,4,2,0,2,2,0,2,4,4,2,0,2, 5,3,5,1,5,3,5,3,1,5,1,5,5,3,5,3 };
	int method = picktable[(addr & 0x001) | ((addr & 0x004) >> 1) | ((addr & 0x020) >> 3) | ((addr & 0x080) >> 4) |
	                       ((addr & 0x200) >> 5)];
	if (addr & 0x800)
		method ^= 1;
	const uint8_t *t = swap_xor_table[method];
	return (uint8_t)(bitswap8(e, t[0], t[1], t[2], t[3], t[4], t[5], t[6], t[7]) ^ t[8]);
}

static void epos_decrypt(uint8_t *rom, int epos)
{
	// D2 is inverted and connected to D1, D5 is inverted and connected to D0; the other six bits through a PAL10H8
	// driven by the counter: the four decryptions it uses made in advance (pacman_m.cpp)
	static const int bs[3][4][8] = {
		{ { 3,7,0,6,4,1,2,5 }, { 1,7,0,3,4,6,2,5 }, { 3,0,4,6,7,1,2,5 }, { 1,0,4,3,7,6,2,5 } },   // theglobp
		{ { 1,6,7,3,4,0,2,5 }, { 7,6,1,3,4,0,2,5 }, { 1,0,7,6,4,3,2,5 }, { 7,0,1,6,4,3,2,5 } },   // acitya
		{ { 7,6,1,3,0,4,2,5 }, { 7,1,4,3,0,6,2,5 }, { 7,6,1,0,3,4,2,5 }, { 7,1,4,0,3,6,2,5 } },   // eeekkp
	};
	static const uint8_t inv[3][4] = { { 0xfc, 0xf6, 0x7d, 0x77 }, { 0xb5, 0xa7, 0xfc, 0xee }, { 0xfd, 0xbf, 0x75, 0x37 } };
	const int k = epos - EPOS_GLOB;
	for (int m = 0; m < 4; m++)
		for (int a = 0; a < 0x4000; a++) {
			const int *s = bs[k][m];
			rom[0x10000 + m * 0x4000 + a] = bitswap8((uint8_t)(rom[a] ^ inv[k][m]), s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7]);
		}
}

static void mspacman_decode(uint8_t *rom)
{
	static const int a12[12] = { 11, 3, 7, 9, 10, 8, 6, 5, 4, 2, 1, 0 };
	static const int a11u5[11] = { 8, 7, 5, 9, 10, 6, 3, 4, 2, 1, 0 };
	static const int a11u6[11] = { 3, 7, 9, 10, 8, 6, 5, 4, 2, 1, 0 };
	uint8_t *d = rom + 0x10000;   // the decrypted Ms. Pac-Man code in the high bank
	for (int i = 0; i < 0x1000; i++) {
		d[0x0000 + i] = rom[0x0000 + i];   // pacman.6e
		d[0x1000 + i] = rom[0x1000 + i];   // pacman.6f
		d[0x2000 + i] = rom[0x2000 + i];   // pacman.6h
		d[0x3000 + i] = bitswap8(rom[0xb000 + bitswapn((uint32_t)i, 12, a12)], 0, 4, 5, 7, 6, 3, 2, 1);   // decrypt u7
	}
	for (int i = 0; i < 0x800; i++) {
		d[0x8000 + i] = bitswap8(rom[0x8000 + bitswapn((uint32_t)i, 11, a11u5)], 0, 4, 5, 7, 6, 3, 2, 1);   // u5
		d[0x8800 + i] = bitswap8(rom[0x9800 + bitswapn((uint32_t)i, 11, a11u6)], 0, 4, 5, 7, 6, 3, 2, 1);   // half of u6
		d[0x9000 + i] = bitswap8(rom[0x9000 + bitswapn((uint32_t)i, 11, a11u6)], 0, 4, 5, 7, 6, 3, 2, 1);   // half of u6
		d[0x9800 + i] = rom[0x1800 + i];   // mirror of pacman.6f high
	}
	for (int i = 0; i < 0x1000; i++) {
		d[0xa000 + i] = rom[0x2000 + i];   // mirror of pacman.6h
		d[0xb000 + i] = rom[0x3000 + i];   // mirror of pacman.6j
	}
	// install the forty 8-byte patches into the decrypted bank (mspacman_install_patches)
	static const uint16_t patch[40][2] = {
		{ 0x0410, 0x8008 }, { 0x08e0, 0x81d8 }, { 0x0a30, 0x8118 }, { 0x0bd0, 0x80d8 }, { 0x0c20, 0x8120 },
		{ 0x0e58, 0x8168 }, { 0x0ea8, 0x8198 }, { 0x1000, 0x8020 }, { 0x1008, 0x8010 }, { 0x1288, 0x8098 },
		{ 0x1348, 0x8048 }, { 0x1688, 0x8088 }, { 0x16b0, 0x8188 }, { 0x16d8, 0x80c8 }, { 0x16f8, 0x81c8 },
		{ 0x19a8, 0x80a8 }, { 0x19b8, 0x81a8 }, { 0x2060, 0x8148 }, { 0x2108, 0x8018 }, { 0x21a0, 0x81a0 },
		{ 0x2298, 0x80a0 }, { 0x23e0, 0x80e8 }, { 0x2418, 0x8000 }, { 0x2448, 0x8058 }, { 0x2470, 0x8140 },
		{ 0x2488, 0x8080 }, { 0x24b0, 0x8180 }, { 0x24d8, 0x80c0 }, { 0x24f8, 0x81c0 }, { 0x2748, 0x8050 },
		{ 0x2780, 0x8090 }, { 0x27b8, 0x8190 }, { 0x2800, 0x8028 }, { 0x2b20, 0x8100 }, { 0x2b30, 0x8110 },
		{ 0x2bf0, 0x81d0 }, { 0x2cc0, 0x80d0 }, { 0x2cd8, 0x80e0 }, { 0x2cf0, 0x81e0 }, { 0x2d60, 0x8160 } };
	for (int k = 0; k < 40; k++)
		for (int i = 0; i < 8; i++)
			d[patch[k][0] + i] = d[patch[k][1] + i];
	for (int i = 0; i < 0x1000; i++) {   // mirror the Pac-Man ROMs into the upper addresses of the normal bank
		rom[0x8000 + i] = rom[0x0000 + i];
		rom[0x9000 + i] = rom[0x1000 + i];
		rom[0xa000 + i] = rom[0x2000 + i];
		rom[0xb000 + i] = rom[0x3000 + i];
	}
}

static void sprglobp2_decode(uint8_t *rom)
{
	static const uint8_t data_xortable[16][8] = {
		{ 0xa8, 0xa8, 0xa8, 0xa8, 0xa8, 0xa8, 0xa8, 0xa8 }, { 0xa0, 0xa0, 0x88, 0x88, 0x88, 0x88, 0xa0, 0xa0 },
		{ 0x00, 0x00, 0x88, 0x88, 0x00, 0x00, 0x88, 0x88 }, { 0xa0, 0xa0, 0x88, 0x88, 0x88, 0x88, 0xa0, 0xa0 },
		{ 0x88, 0x88, 0xa0, 0xa0, 0x28, 0x28, 0x00, 0x00 }, { 0xa0, 0xa0, 0x88, 0x88, 0x88, 0x88, 0xa0, 0xa0 },
		{ 0x20, 0x20, 0x20, 0x20, 0x80, 0x80, 0x80, 0x80 }, { 0xa0, 0xa0, 0x88, 0x88, 0x88, 0x88, 0xa0, 0xa0 },
		{ 0xa8, 0xa8, 0xa8, 0xa8, 0xa8, 0xa8, 0xa8, 0xa8 }, { 0x28, 0x28, 0xa0, 0xa0, 0x00, 0x00, 0x88, 0x88 },
		{ 0x00, 0x00, 0x88, 0x88, 0x00, 0x00, 0x88, 0x88 }, { 0x28, 0x28, 0xa0, 0xa0, 0x00, 0x00, 0x88, 0x88 },
		{ 0x88, 0x88, 0xa0, 0xa0, 0x28, 0x28, 0x00, 0x00 }, { 0x28, 0x28, 0xa0, 0xa0, 0x00, 0x00, 0x88, 0x88 },
		{ 0x20, 0x20, 0x20, 0x20, 0x80, 0x80, 0x80, 0x80 }, { 0x28, 0x28, 0xa0, 0xa0, 0x00, 0x00, 0x88, 0x88 } };
	for (int a = 0; a < 0x4000; a++) {
		const uint8_t src = rom[a];
		// the table from bits 0, 4, 8 and 12 of the address, the offset in it from bits 1, 3 and 5 of the data
		const int i = (a & 1) | ((a >> 4) & 1) << 1 | ((a >> 8) & 1) << 2 | ((a >> 12) & 1) << 3;
		int j = ((src >> 1) & 1) | ((src >> 3) & 1) << 1 | ((src >> 5) & 1) << 2;
		if (src & 0x80)   // the bottom half of the translation table is the mirror image of the top
			j = 7 - j;
		rom[a] = src ^ data_xortable[i][j];
	}
}

static void mspactwin_decode(uint8_t *rom)
{
	uint8_t *op = s_opcodes;   // (decrypted_opcodes at 0x0000-0x3fff mirrored at 0x4000 (only 0x6000-0x7fff), high at 0x8000)
	uint8_t lo[0x4000], hi[0x4000];
	for (int a = 0; a < 0x4000; a += 2) {
		lo[a] = bitswap8(rom[a], 4, 5, 6, 7, 0, 1, 2, 3);
		lo[a + 1] = bitswap8((uint8_t)(rom[a + 1] ^ 0x9a), 6, 4, 5, 7, 2, 0, 3, 1);
		hi[a] = bitswap8(rom[0x8000 + a], 4, 5, 6, 7, 0, 1, 2, 3);
		hi[a + 1] = bitswap8((uint8_t)(rom[0x8000 + a + 1] ^ 0x9a), 6, 4, 5, 7, 2, 0, 3, 1);
		rom[a] = bitswap8(rom[a], 0, 1, 2, 3, 4, 5, 6, 7);
		rom[a + 1] = bitswap8((uint8_t)(rom[a + 1] ^ 0xa3), 2, 4, 6, 3, 7, 0, 5, 1);
		rom[0x8000 + a] = bitswap8(rom[0x8000 + a], 0, 1, 2, 3, 4, 5, 6, 7);
		rom[0x8000 + a + 1] = bitswap8((uint8_t)(rom[0x8000 + a + 1] ^ 0xa3), 2, 4, 6, 3, 7, 0, 5, 1);
	}
	memset(op, 0, 0x10000);
	memcpy(op, lo, 0x4000);
	memcpy(op + 0x6000, lo + 0x2000, 0x2000);
	memcpy(op + 0x8000, hi, 0x4000);
}

static double raw_weights(const int *res, int n, double *w)
{
	// MAME compute_resistor_weights(0, 255, -1.0, ...): each resistor's output alone (no pulldown / pullup)
	double sum = 0;
	for (int k = 0; k < n; k++) {
		double r0 = 1.0 / 1e12, r1 = 1.0 / 1e12;
		for (int j = 0; j < n; j++) {
			if (j == k)
				r1 += 1.0 / res[j];
			else
				r0 += 1.0 / res[j];
		}
		r0 = 1.0 / r0;
		r1 = 1.0 / r1;
		w[k] = 255.0 * r0 / (r1 + r0);
		sum += w[k];
	}
	return sum;
}

static int combine(const double *w, int n, int v)
{
	double s = 0;
	for (int k = 0; k < n; k++)
		s += w[k] * ((v >> k) & 1);
	return (int)(s + 0.5);
}

static void build_palette(void)
{
	static const int res[3] = { 1000, 470, 220 };
	double w3[3], w2[2];
	const double s3 = raw_weights(res, 3, w3), s2 = raw_weights(res + 1, 2, w2);
	const double scale = 255.0 / (s3 > s2 ? s3 : s2);
	for (int k = 0; k < 3; k++)
		w3[k] *= scale;
	for (int k = 0; k < 2; k++)
		w2[k] *= scale;
	uint32_t rgb[32];
	for (int i = 0; i < 32; i++) {
		const uint8_t v = s_proms[i];
		const uint32_t r = (uint32_t)combine(w3, 3, v), c2 = (uint32_t)combine(w3, 3, v >> 3), c3 = (uint32_t)combine(w2, 2, v >> 6);
		rgb[i] = (s_m->flags & F_RBG) ? (r << 16 | c3 << 8 | c2) : (r << 16 | c2 << 8 | c3);
	}
	for (int i = 0; i < NPENS; i++)
		s_pal[i] = rgb[s_proms[0x20 + i] & 0x0f];
	for (int c = 0; c < 32; c++) {
		s_spr_trans[c] = 0;
		for (int p = 0; p < 4; p++)
			if ((s_proms[0x20 + c * 4 + p] & 0x0f) == 0)
				s_spr_trans[c] |= (uint8_t)(1 << p);
	}
}

static int pm_init(const rbx_game *g)
{
	const int mi = find_machine(g->machine, &s_init);
	if (mi < 0) {
		rbx_fail("%s: machine %s not emulated", g->set, g->machine);
		return -1;
	}
	s_m = &s_machines[mi];
	s_shootbul = !strcmp(g->set, "shootbul");
	uint32_t gfx_size = 0, proms_size = 0, gfx2_size = 0, wave_size = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *gfx = rbx_region("gfx1", &gfx_size);
	s_proms = rbx_region("proms", &proms_size);
	s_user1 = rbx_region_opt("user1", &s_user1_size);
	uint8_t *gfx2 = rbx_region_opt("gfx2", &gfx2_size);
	s_wave = rbx_region_opt("namco", &wave_size);
	if (!s_rom || !gfx || !s_proms || proms_size < 0x120 || (s_m->snd == SND_WSG && (!s_wave || wave_size < 0x100))) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	const uint32_t need = s_m->map == MAP_SUPERABC ? 0x80000 : 0x20000;
	if (s_rom_size < need) {   // (a full-size view: the decryptions' banks above 0x10000, the maps' reads past the end)
		s_romcopy = (uint8_t *)calloc(1, need);
		if (!s_romcopy) {
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
		memcpy(s_romcopy, s_rom, s_rom_size);
		s_rom = s_romcopy;
	}
	if (s_m->map == MAP_MSPACTWIN) {
		s_opcodes = (uint8_t *)calloc(1, 0x10000);
		if (!s_opcodes) {
			pm_exit();
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
	}
	// ---- the ROMs' decryptions (driver init) ----
	uint8_t *rom = s_rom;
	switch (s_init) {
	case INIT_EYES:   // data lines D3 and D5 swapped; the graphics as woodpek
		for (int i = 0; i < 0xc000; i++)
			rom[i] = bitswap8(rom[i], 7, 6, 3, 4, 5, 2, 1, 0);
		for (uint32_t i = 0; i + 8 <= gfx_size; i += 8)
			eyes_decode(gfx + i);
		break;
	case INIT_WOODPEK:
		for (uint32_t i = 0; i + 8 <= gfx_size; i += 8)
			eyes_decode(gfx + i);
		break;
	case INIT_PACPLUS:
		for (int i = 0; i < 0x4000; i++)
			rom[i] = pacplus_decrypt(i, rom[i]);
		break;
	case INIT_JUMPSHOT:
		for (int i = 0; i < 0x4000; i++)
			rom[i] = jumpshot_decrypt(i, rom[i]);
		break;
	case INIT_MSPACMAN:
		mspacman_decode(rom);
		break;
	case INIT_MSPACMBE:   // address lines A1 and A0 swapped if A2 = 0 (a 74298 on the 6F ROM)
		for (int i = 0x1000; i < 0x2000; i += 4)
			if (!(i & 8)) {
				const uint8_t t = rom[i + 1];
				rom[i + 1] = rom[i + 2];
				rom[i + 2] = t;
			}
		break;
	case INIT_PENGOMC1:   // some sort of weak protection?
		for (int i = 0; i < 0xc000; i += 0x100)
			for (int j = 0; j < 0x80; j++) {
				const uint8_t t = rom[i | j];
				rom[i | j] = rom[i | (j ^ 0xff)];
				rom[i | (j ^ 0xff)] = t;
			}
		break;
	case INIT_MSPACKPLS:
		for (int i = 0; i < 0xa000; i++)
			rom[i] = bitswap8(rom[i], 7, 6, 5, 3, 4, 2, 1, 0);
		break;
	case INIT_CLUBPACMA:
		for (int i = 0; i < 0xc000; i++)
			rom[i] = bitswap8(rom[i], 6, 7, 5, 4, 3, 2, 1, 0);
		break;
	case INIT_MSPACTWIN:
		mspactwin_decode(rom);
		break;
	case INIT_PONPOKO: {   // the gfx data is swapped wrt the other Pac-Man hardware games
		const uint32_t len = gfx_size / 2;
		for (uint32_t i = 0; i < len; i += 0x10)
			for (int j = 0; j < 8; j++) {
				const uint8_t t = gfx[i + j + 8];
				gfx[i + j + 8] = gfx[i + j];
				gfx[i + j] = t;
			}
		uint8_t *s = gfx + len;
		for (uint32_t i = 0; i < len; i += 0x20)
			for (int j = 0; j < 8; j++) {
				const uint8_t t = s[i + j + 0x18];
				s[i + j + 0x18] = s[i + j + 0x10];
				s[i + j + 0x10] = s[i + j + 0x08];
				s[i + j + 0x08] = s[i + j];
				s[i + j] = t;
			}
		break;
	}
	case INIT_ALIBABA:   // reorder the clock graphics a bit to make it work with gfxdecode
		for (uint32_t i = 0; gfx2 && i < gfx2_size; i++)
			if ((i & 0x3f) < 0x18 && i + 8 < gfx2_size) {
				gfx2[i] = gfx2[i + 8];
				gfx2[i + 8] = 0;
			}
		break;
	case INIT_SPRGLOBP2:
		sprglobp2_decode(rom);
		break;
	case INIT_ROCKTRV2:   // hack to pass the rom check for the bad rom
		rom[0x7ffe] = 0xa7;
		rom[0x7fee] = 0x6d;
		break;
	case INIT_SUPERABC:   // descramble gfx
		if (s_user1 && gfx_size >= 0x10000) {
			for (uint32_t i = 0; i < 0x10000; i++) {
				const uint32_t s = (i & 0xfe0fff) | ((i >> 12) & 1) << 16 | ((i >> 13) & 1) << 15 | ((i >> 14) & 1) << 14 |
				                   ((i >> 16) & 1) << 13 | ((i >> 15) & 1) << 12;
				gfx[i] = s < s_user1_size ? s_user1[s] : 0;
			}
		}
		break;
	default:
		break;
	}
	if (s_m->epos != EPOS_NONE)
		epos_decrypt(rom, s_m->epos);
	// ---- the graphics ----
	static const uint32_t txo[8] = { 64, 65, 66, 67, 0, 1, 2, 3 };
	static const uint32_t tyo[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
	static const uint32_t sxo[16] = { 64, 65, 66, 67, 128, 129, 130, 131, 192, 193, 194, 195, 0, 1, 2, 3 };
	static const uint32_t syo[16] = { 0, 8, 16, 24, 32, 40, 48, 56, 256, 264, 272, 280, 288, 296, 304, 312 };
	uint32_t chr_start = 0, spr_start = 0x1000, half = gfx_size / 2, nchr, nspr;
	uint32_t planes[2] = { 0, 4 };
	if (s_m->flags & F_SUPERABC)
		spr_start = 0x8000;
	if (s_m->flags & F_CRUSH4) {   // (RGN_FRAC(1,4) elements, planes RGN_FRAC(1,2) and RGN_FRAC(0,2) + 4)
		nchr = gfx_size / 4 / 16;
		nspr = gfx_size / 4 / 64;
		planes[0] = gfx_size / 2 * 8;
		planes[1] = 4;
	} else {
		nchr = half / 16;
		nspr = half / 64;
	}
	if (nchr == 0 || nspr == 0 || (nchr & (nchr - 1)) || (nspr & (nspr - 1))) {
		pm_exit();
		rbx_fail("%s: unexpected graphics size", g->set);
		return -1;
	}
	s_chr_mask = nchr - 1;
	s_spr_mask = nspr - 1;
	s_chr = (uint8_t *)malloc((size_t)nchr * 64);
	s_spr = (uint8_t *)malloc((size_t)nspr * 256);
	s_snap = (snap_t *)calloc(1, sizeof(snap_t));
	s_bg = (uint16_t *)calloc(1, SCR_W * VIS_H * sizeof(uint16_t));
	s_bg_key = (uint32_t *)malloc(36 * 28 * sizeof(uint32_t));
	if (gfx2 && s_m->lat == LAT_ALIBABA)
		s_clock = (uint8_t *)malloc(32 * 24 * 16);
	if (!s_chr || !s_spr || !s_snap || !s_bg || !s_bg_key || (gfx2 && s_m->lat == LAT_ALIBABA && !s_clock)) {
		pm_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	for (uint32_t t = 0; t < nchr; t++)
		decode_elem(s_chr + t * 64, gfx, gfx_size, chr_start * 8 + t * 128, 8, 8, planes, txo, tyo);
	for (uint32_t t = 0; t < nspr; t++)
		decode_elem(s_spr + t * 256, gfx, gfx_size, spr_start * 8 + t * 512, 16, 16, planes, sxo, syo);
	if (s_clock) {   // alibaba_clocklayout
		static const uint32_t cxo[24] = { 16*8+7, 16*8+6, 16*8+5, 16*8+4, 16*8+3, 16*8+2, 16*8+1, 16*8+0,
		                                  8*8+7, 8*8+6, 8*8+5, 8*8+4, 8*8+3, 8*8+2, 8*8+1, 8*8+0,
		                                  7, 6, 5, 4, 3, 2, 1, 0 };
		static const uint32_t cyo[16] = { 256+0*8, 256+1*8, 256+2*8, 256+3*8, 256+4*8, 256+5*8, 256+6*8, 256+7*8,
		                                  0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8 };
		const uint32_t cp[2] = { gfx2_size / 2 * 8, 0 };
		const uint32_t n = gfx2_size / 2 / 64;   // (RGN_FRAC(1,2) of 64-byte elements: 32)
		memset(s_clock, 0, 32 * 24 * 16);
		for (uint32_t t = 0; t < n && t < 32; t++)
			decode_elem(s_clock + t * (24 * 16), gfx2, gfx2_size, t * 512, 24, 16, cp, cxo, cyo);
	}
	build_palette();
	build_tile_idx();
	s_port_in0 = rbx_port_find("IN0");
	s_port_in1 = rbx_port_find("IN1");
	s_port_dsw1 = rbx_port_find("DSW1");
	s_port_dsw2 = rbx_port_find("DSW2");
	s_port_p1 = rbx_port_find("P1");
	s_port_p2 = rbx_port_find("P2");
	memset(s_page_bf, 0xbf, sizeof(s_page_bf));
	memset(s_page_00, 0, sizeof(s_page_00));
	s_vis_w = (s_m->flags & F_NARROW) ? 256 : SCR_W;
	s_xoff = (s_m->flags & F_NARROW) ? 16 : 0;
	memset(&b, 0, sizeof(b));
	sound_init();
	const uint32_t rate = s_m->snd == SND_WSG ? WSG_RATE : s_m->snd == SND_SN2 ? sn76496_rate(&s_sn[0]) : ay8910_rate(&s_ay);
	if (rbx_audio_setup(rate) || rbx_video_setup(s_vis_w, VIS_H, NPENS)) {
		pm_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	rbx_prof_name(RBX_PA_DRV, s_pa_names, PM_PA_END - RBX_PA_DRV);
	pm_reset();
	return 0;
}

static void pm_exit(void)
{
	rbx_video_wait_idle();
	free(s_romcopy);
	free(s_opcodes);
	free(s_chr);
	free(s_spr);
	free(s_clock);
	free(s_snap);
	free(s_bg);
	free(s_bg_key);
	s_romcopy = s_opcodes = s_chr = s_spr = s_clock = NULL;
	s_snap = NULL;
	s_bg = NULL;
	s_bg_key = NULL;
}

// ---- save states ----
static int pm_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "pm", &b, sizeof(b));
	z80 c = s_main;
	rc |= rbx_state_io(ctx, "pm_z80", &c, sizeof(c));
	switch (s_m->snd) {
	case SND_WSG: rc |= rbx_state_io(ctx, "pm_wsg", &s_wsg, (uint32_t)NAMCO_WSG_STATE_SIZE); break;
	case SND_AY: case SND_AY8912: rc |= rbx_state_io(ctx, "pm_ay", &s_ay, ay8910_state_size()); break;
	case SND_SN2:
		rc |= rbx_state_io(ctx, "pm_sn0", &s_sn[0], sn76496_state_size());
		rc |= rbx_state_io(ctx, "pm_sn1", &s_sn[1], sn76496_state_size());
		break;
	}
	if (!rc && load) {
		c.read_byte = main_read;
		c.write_byte = main_write;
		c.port_in = main_in;
		c.port_out = main_out;
		c.userdata = NULL;
		c.read_map = s_rmap;
		c.op_rom = s_opcodes;
		s_main = c;
		build_maps();
		bg_invalidate();
	}
	return rc ? -1 : 0;
}

static int pm_save(void *ctx) { return pm_state(ctx, 0); }
static int pm_load(void *ctx) { return pm_state(ctx, 1); }
static int pm_supported(const char *machine) { return find_machine(machine, NULL) >= 0; }

const rbx_driver rbx_drv_pacman = { "Namco Pac-Man", pm_init, pm_exit, pm_reset, pm_frame, pm_save, pm_load,
                                    pm_supported, pm_render, NULL };
